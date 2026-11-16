#!/usr/bin/env python3
"""Fake ESP32 nodes for the wave2d cluster flow, without hardware (GitHub issue #4, 2026-11-16).

Each fake node speaks the same MQTT topics and JSON payloads as the firmware's wire-v2 path
(node-esp32/main/node_v2.c, main.c, dyne_mqtt.c, dyne_ops.c), so the PC tools (scripts/dyne_ota.py,
scripts/wave2d_cluster.py) can be exercised end to end:

  dyne/node/<id>/status   retained: online (ip, actor, ...; plus "udp_port", fake only), configured, running
                          (step / stall / stall_cleared), done (crc32 and counters), err; last will = offline
  dyne/cfg/<id>           -> dyne/cfg/<id>/ack  {"ok","idx","gen","crc32","wire":2} or {"ok":false,"err"}
  dyne/ctl/<id>, dyne/ctl/all   start / stop / report / reboot / ota_actor (-> dyne/node/<id>/ota phases)
  dyne/node/<idx>/field2d, dyne/node/<idx>/rt

What is real and what is not:
  - The numerics are a line-by-line Python port of crates/wave2d-actor (5-point leapfrog, Taylor first step,
    fixed-value / zero-gradient / absorbing faces) including libm's exp, so a correct partition and correct
    halo exchange give the same per-node CRC32 as dyne-sim2d, bit for bit. Only the Gaussian initial
    condition is supported (the experiment's scenes); other initial conditions are rejected in the ack.
  - Neighbours exchange halo values over UDP on 127.0.0.1 with the firmware's step-sync rules (send copies of
    the last two steps, hold early values, answer past and same-step duplicates, resend after 100 ms, stall
    reports every mode.stall_ms, mode.test_drop). The datagram format is a fake one, not the wire-v2 envelope.
  - Each fake node starts with the embedded 1D actor (heat1d_actor_small), like a freshly flashed board, and
    rejects a 2D configuration until an actor with the wire-v2 exports is installed with ota_actor
    (the fake checks the .wasm for the export names; it does not run it).

Usage (repository root; a broker on 127.0.0.1, e.g. `python scripts/dyne_mqtt_lite.py broker --port 1883`):
  python scripts/wave2d_fake_nodes.py --broker 127.0.0.1 --ids 11,12,13,14
Fault injection: --die 12@50 (node 12 goes silent at step 50, to see the neighbours' stall reports).
"""
import argparse
import hashlib
import json
import math
import os
import random
import socket
import struct
import sys
import threading
import time
import urllib.request
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dyne_mqtt_lite  # noqa: E402

# ---- crates/dyne-wire abi (field2d, wave2d) ----
FACE_NAMES = "wesn"
BC_DIRICHLET, BC_INTERFACE, BC_ZERO_GRAD = 0, 1, 2
CFG_IC, CFG_MODE_M, CFG_MODE_N, CFG_AMP, CFG_GX, CFG_GY, CFG_SIGMA, CFG_ABSORB = range(8)
IC_SIN_MODE, IC_GAUSS, IC_COS_MODE = 0, 1, 2
CFG_DEFAULT = [IC_SIN_MODE, 1.0, 1.0, 1.0, 0.5, 0.5, 0.05, 0.0]
SIDE_MAX_SMALL = 48  # the small (hardware) build: blocks up to 48 points per side

# ---- node_v2.c timing ----
TIMEOUT_US = 100000
RESEND_MIN_US = 20000
RECV_SLICE_S = 0.020
STALL_DEFAULT_MS = 10000

V2_EXPORTS = (b"dyne_outbox_read", b"dyne_inbox_write", b"dyne_init2d", b"dyne_set_face", b"dyne_step")
DGRAM = struct.Struct("<4sIQI")  # magic, iface id, step, count  (+ count f64)
MAGIC = b"DYFK"


# ======================= libm exp (crates libm 0.2, origin FreeBSD e_exp.c) =======================

_LN2HI = 6.93147180369123816490e-01
_LN2LO = 1.90821492927058770002e-10
_INVLN2 = 1.44269504088896338700e+00
_P1 = 1.66666666666666019037e-01
_P2 = -2.77777777770155933842e-03
_P3 = 6.61375632143793436117e-05
_P4 = -1.65339022054652515390e-06
_P5 = 4.13813679705723846039e-08


def libm_exp(x):
    """Same operations in the same order as libm::exp (IEEE f64, no fused multiply-add) -> same bits."""
    hx = struct.unpack("<Q", struct.pack("<d", x))[0] >> 32
    sign = hx >> 31
    hx &= 0x7FFFFFFF
    if hx >= 0x4086232B:
        if x != x:
            return x
        if x > 709.782712893383973096:
            return math.inf
        if x < -745.13321910194110842:
            return 0.0
    if hx > 0x3FD62E42:
        if hx >= 0x3FF0A2B2:
            k = int(_INVLN2 * x + (0.5, -0.5)[sign])  # Rust `as i32` truncates toward zero, as int() does
        else:
            k = 1 - sign - sign
        hi = x - float(k) * _LN2HI
        lo = float(k) * _LN2LO
        x = hi - lo
    elif hx > 0x3E300000:
        k, hi, lo = 0, x, 0.0
    else:
        return 1.0 + x
    xx = x * x
    c = x - xx * (_P1 + xx * (_P2 + xx * (_P3 + xx * (_P4 + xx * _P5))))
    y = 1.0 + (x * c / (2.0 - c) - lo + hi)
    return y if k == 0 else math.ldexp(y, k)


# ======================= the wave2d actor, ported =======================

class Wave2DBlock:
    """Port of crates/wave2d-actor/src/lib.rs (Field). Padded arrays of width nx+2, one ghost layer."""

    def __init__(self, nx, ny, ix0, iy0, h, c, dt, kinds, params, cfg):
        self.nx, self.ny, self.w = nx, ny, nx + 2
        self.kind, self.param = kinds, params  # param: iface id (int) or fixed value (float)
        self.cfgv = cfg  # dict key -> value (only keys that were set)
        nu = c * dt / h
        self.nu2 = nu * nu
        self.half_nu2 = 0.5 * self.nu2
        self.kmur = (nu - 1.0) / (nu + 1.0)
        self.step_no = 0
        size = (nx + 2) * (ny + 2)
        self.u = [0.0] * size
        self.p = [0.0] * size
        self.ghost = [[0.0] * max(nx, ny) for _ in range(4)]
        self.got = [False] * 4
        absorb = int(self.get_cfg(CFG_ABSORB))
        self.absorbing = [self.kind[f] != BC_INTERFACE and (absorb >> f) & 1 == 1 for f in range(4)]
        amp, gx, gy, sg = (self.get_cfg(k) for k in (CFG_AMP, CFG_GX, CFG_GY, CFG_SIGMA))
        for j in range(ny):
            y = float(iy0 + j) * h
            for i in range(nx):
                x = float(ix0 + i) * h
                dx, dy = x - gx, y - gy
                self.u[self.at(i, j)] = amp * libm_exp(-(dx * dx + dy * dy) / (2.0 * sg * sg))
        self.apply_dirichlet()

    def get_cfg(self, k):
        return self.cfgv.get(k, CFG_DEFAULT[k])

    def at(self, i, j):
        return (j + 1) * self.w + (i + 1)

    def face_len(self, f):
        return self.ny if f < 2 else self.nx

    def edge_point(self, f, k):
        return ((0, k), (self.nx - 1, k), (k, 0), (k, self.ny - 1))[f]

    def halo_point(self, f, k):
        return ((-1, k), (self.nx, k), (k, -1), (k, self.ny))[f]

    def mirror_point(self, f, k):
        return ((1, k), (self.nx - 2, k), (k, 1), (k, self.ny - 2))[f]

    def apply_dirichlet(self):
        for f in range(4):
            if self.kind[f] != BC_DIRICHLET or self.absorbing[f]:
                continue
            for k in range(self.face_len(f)):
                self.u[self.at(*self.edge_point(f, k))] = self.param[f]

    def edge_values(self, f):
        return [self.u[self.at(*self.edge_point(f, k))] for k in range(self.face_len(f))]

    def put_ghost(self, f, vals):
        self.ghost[f][:len(vals)] = vals
        self.got[f] = True

    def step(self):
        missing = any(self.kind[f] == BC_INTERFACE and not self.got[f] for f in range(4))
        u, p, w = self.u, self.p, self.w
        for f in range(4):  # fill_halo
            for k in range(self.face_len(f)):
                if self.kind[f] == BC_INTERFACE:
                    v = self.ghost[f][k]
                elif self.kind[f] == BC_ZERO_GRAD or self.absorbing[f]:
                    v = u[self.at(*self.mirror_point(f, k))]
                else:
                    v = self.param[f]
                u[self.at(*self.halo_point(f, k))] = v
        nu2, half_nu2 = self.nu2, self.half_nu2
        first = self.step_no == 0
        for j in range(self.ny):
            c0 = (j + 1) * w + 1
            for c in range(c0, c0 + self.nx):
                uc = u[c]
                lap = (u[c - 1] - 2.0 * uc + u[c + 1]) + (u[c - w] - 2.0 * uc + u[c + w])
                p[c] = uc + half_nu2 * lap if first else 2.0 * uc - p[c] + nu2 * lap
        for f in range(4):  # absorbing faces (Mur): edge from the new inner value and old values
            if not self.absorbing[f]:
                continue
            for k in range(self.face_len(f)):
                e, m = self.at(*self.edge_point(f, k)), self.at(*self.mirror_point(f, k))
                p[e] = u[m] + self.kmur * (p[m] - u[e])
        self.u, self.p = p, u
        self.apply_dirichlet()
        self.step_no += 1
        self.got = [False] * 4
        return not missing

    def state_crc(self):
        """CRC32 of all points as little-endian f64 in dyne_get order (k = j*nx + i), with min and max."""
        vals = [self.u[self.at(i, j)] for j in range(self.ny) for i in range(self.nx)]
        return zlib.crc32(struct.pack(f"<{len(vals)}d", *vals)) & 0xFFFFFFFF, min(vals), max(vals)


# ======================= configuration (node_v2.c parse_cfg_v2) =======================

def parse_cfg_v2(d):
    """Returns (cfg dict, None) or (None, error text). Error texts follow the firmware."""
    cl, nd, f2 = d.get("cluster"), d.get("node"), d.get("field2d")
    if not cl or not nd or not f2:
        return None, "cluster/node/field2d missing"
    c = {"gen": int(d.get("gen", 0)), "idx": int(nd.get("idx", -1)), "steps": int(cl.get("steps", 0)),
         "alpha": float(cl.get("alpha", 1.0)), "dt": float(cl.get("dt", 1e-4)),
         "nx": int(f2.get("nx", 0)), "ny": int(f2.get("ny", 0)), "ix0": int(f2.get("ix0", 0)),
         "iy0": int(f2.get("iy0", 0)), "h": float(f2.get("h", 0.0))}
    if c["nx"] == 0 or c["ny"] == 0 or c["steps"] == 0 or not c["h"] > 0.0:
        return None, "bad nx/ny/h/steps"
    kinds, params = [BC_ZERO_GRAD] * 4, [0] * 4
    for e in f2.get("faces", []):
        fc, ky = e.get("face"), e.get("key")
        if not isinstance(fc, str) or not isinstance(ky, str) or fc not in FACE_NAMES:
            return None, "bad face entry"
        f = FACE_NAMES.index(fc)
        if ky == "interface":
            kinds[f], params[f] = BC_INTERFACE, int(e.get("iface", -1))
        elif ky == "dirichlet":
            kinds[f], params[f] = BC_DIRICHLET, float(e.get("value", 0.0))
        elif ky == "zero_grad":
            kinds[f] = BC_ZERO_GRAD
        else:
            return None, "bad face entry"
    cfgv = {}
    for e in f2.get("cfg", []):
        if not isinstance(e, list) or len(e) != 2:
            return None, "bad actor cfg"
        cfgv[int(e[0])] = float(e[1])
    links = []
    for e in d.get("ifaces", []):
        if len(links) >= 4:
            return None, "bad iface entry"
        try:
            ip, port = e["peer"].rsplit(":", 1)
            links.append({"id": int(e["id"]), "peer": (ip, int(port))})
        except (KeyError, ValueError, AttributeError, TypeError):
            return None, "bad iface entry"
    n_if = 0
    for f in range(4):
        if kinds[f] != BC_INTERFACE:
            continue
        n_if += 1
        hits = [ln for ln in links if ln["id"] == params[f]]
        if len(hits) != 1:
            return None, "iface/face mismatch"
        hits[0]["face"] = f
    if n_if != len(links):
        return None, "iface/face mismatch"
    mo = d.get("mode") or {}
    c["field_every"] = int(mo.get("field_every", 50))
    c["rt_every"] = int(mo.get("rt_every", 100))
    c["stall_ms"] = max(0, int(mo.get("stall_ms", STALL_DEFAULT_MS)))
    c["test_drop"] = float(mo.get("test_drop", 0.0))
    if not 0.0 <= c["test_drop"] < 1.0:
        return None, "mode.test_drop: 0 <= p < 1"
    # the actor's dyne_init2d checks (crates/wave2d-actor init: small build)
    if not (3 <= c["nx"] <= SIDE_MAX_SMALL and 3 <= c["ny"] <= SIDE_MAX_SMALL):
        return None, "fake node: block must be 3..48 points per side (small actor)"
    if int(cfgv.get(CFG_IC, CFG_DEFAULT[CFG_IC])) != IC_GAUSS:
        return None, "fake node: only the Gaussian initial condition (cfg key 0 = 1) is emulated"
    c.update(kinds=kinds, params=params, cfgv=cfgv, links=links)
    return c, None


# ======================= one fake node =======================

class FakeNode:
    def __init__(self, nid, args, mqtt):
        self.nid, self.a, self.mqtt = nid, args, mqtt
        self.t_boot = time.time()
        self.boot = 1
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", 0))
        self.udp_port = self.udp.getsockname()[1]
        self.actor = {"src": "embedded", "seq": 0, "name": "heat1d_actor_small"}
        self.v2 = False
        self.act_seq = 0
        self.cfg = None
        self.busy = False
        self.ota_running = False
        self.stop_req = False
        self.report_req = False
        self.dead = False
        self.die_step = args.die.get(nid)
        self.rng = random.Random(args.seed * 1000 + nid)
        self.reset = "poweron"
        self.cl = None
        self.connect()

    # ---- MQTT ----
    def connect(self):
        cl = self.mqtt.Client(self.mqtt.CallbackAPIVersion.VERSION2, client_id=f"dyne-fake-{self.nid}-{self.boot}")
        cl.will_set(f"dyne/node/{self.nid}/status", json.dumps({"state": "offline", "reason": "lwt"}), 1, True)
        cl.on_message = self.on_message
        cl.on_connect = self.on_connect
        cl.connect(self.a.broker, self.a.port, 60)
        self.cl = cl  # before loop_start: on_connect publishes the online status through self.cl
        cl.loop_start()

    def on_connect(self, cl, u, flags, rc, props=None):
        cl.subscribe([(f"dyne/cfg/{self.nid}", 1), ("dyne/ctl/all", 1), (f"dyne/ctl/{self.nid}", 1)])
        self.pub_status("online")

    def pub(self, sub, obj, qos=0, retain=False):
        if self.cl is not None:
            self.cl.publish(f"dyne/{sub}", json.dumps(obj), qos=qos, retain=retain)

    def up_s(self):
        return int(time.time() - self.t_boot)

    def pub_status(self, state, **extra):
        if state == "online":
            d = {"state": "online", "mac": f"02:00:00:00:FF:{self.nid & 0xFF:02X}", "id_src": "mac",
                 "ip": "127.0.0.1", "udp_port": self.udp_port, "fw": "dyne-fake-node", "heap": 150000,
                 "up_s": self.up_s(), "target": "esp32", "ver": "fake", "elf": "0" * 16, "part": "ota_0",
                 "ota_state": "valid", "reset": self.reset, "boot": self.boot, "prev_up_s": 0, "actor": self.actor}
        else:
            d = {"state": state, "idx": self.cfg["idx"] if self.cfg else -1, "up_s": self.up_s()}
        d.update(extra)
        self.pub(f"node/{self.nid}/status", d, qos=1, retain=True)

    def on_message(self, cl, u, m):
        if self.dead:
            return
        if m.topic.startswith("dyne/cfg/"):
            self.on_cfg(m.payload)
        elif m.topic.startswith("dyne/ctl/"):
            if m.retain:  # the firmware ignores retained commands
                return
            try:
                d = json.loads(m.payload)
            except ValueError:
                return
            self.on_ctl(d)

    def on_cfg(self, payload):
        sub = f"cfg/{self.nid}/ack"
        try:
            d = json.loads(payload)
        except ValueError:
            self.pub(sub, {"ok": False, "idx": self.nid, "err": "json parse"}, qos=1)
            return
        if not self.v2:  # the embedded 1D actor: main.c parse_cfg wants "bc"
            self.cfg = None
            self.pub(sub, {"ok": False, "idx": self.nid, "err": "cluster/node/bc missing"}, qos=1)
            return
        if self.busy:
            self.pub(sub, {"ok": False, "idx": self.nid, "err": "running"}, qos=1)
            return
        c, err = parse_cfg_v2(d)
        if err:
            self.cfg = None
            self.pub(sub, {"ok": False, "idx": self.nid, "err": err}, qos=1)
            return
        self.cfg = c
        self.pub(sub, {"ok": True, "idx": c["idx"], "gen": c["gen"],
                       "crc32": f"0x{zlib.crc32(payload) & 0xFFFFFFFF:08X}", "wire": 2}, qos=1)
        self.pub_status("configured", gen=c["gen"])

    def on_ctl(self, d):
        cmd = d.get("cmd")
        if cmd == "start":
            if self.cfg is None:
                self.pub_status("err", err="start without cfg")
            elif not self.busy:
                self.stop_req = False
                self.busy = True
                threading.Thread(target=self.run_bsp_safe, daemon=True).start()
        elif cmd == "stop":
            self.stop_req = True
        elif cmd == "report":
            self.report_req = True
        elif cmd == "reboot":
            threading.Thread(target=self.reboot, args=("sw",), daemon=True).start()
        elif cmd == "ota_actor":
            job = d.get("job", "")
            if self.busy or self.ota_running:
                self.pub(f"node/{self.nid}/ota", {"job": job, "kind": "actor", "phase": "busy", "up_s": self.up_s(),
                                                  "err": "running" if self.busy else "ota in progress"}, qos=1)
            else:
                self.ota_running = True
                threading.Thread(target=self.ota_actor, args=(d,), daemon=True).start()

    def reboot(self, reason):
        self.pub(f"node/{self.nid}/status", {"state": "offline", "reason": "reboot"}, qos=1, retain=True)
        time.sleep(0.3)
        self.stop_req = True
        while self.busy:
            time.sleep(0.05)
        self.cl.disconnect()
        self.cl.loop_stop()
        self.cl = None
        self.cfg = None
        self.boot += 1
        self.reset = reason
        self.t_boot = time.time()
        self.connect()  # the retained cfg comes back, as on the board

    def ota_actor(self, d):
        job, t0 = d.get("job", ""), time.time()
        topic = f"node/{self.nid}/ota"

        def phase(ph, **kw):
            self.pub(topic, dict({"job": job, "kind": "actor", "phase": ph, "up_s": self.up_s()}, **kw), qos=1)

        phase("accepted")
        w = d.get("wasm")
        try:
            if not isinstance(w, dict):
                raise ValueError("fake node: needs a wasm blob")
            data = urllib.request.urlopen(w["url"], timeout=15).read()
            if len(data) != int(w["size"]) or hashlib.sha256(data).hexdigest() != w["sha256"].lower():
                phase("rejected", ms=int((time.time() - t0) * 1000), bytes=len(data), slot=0, seq=0,
                      err="sha256 mismatch")
                self.ota_running = False
                return
        except Exception as e:  # noqa: BLE001 (report any download failure as the firmware does)
            phase("fail", ms=int((time.time() - t0) * 1000), bytes=0, slot=0, seq=0, err=str(e)[:60])
            self.ota_running = False
            return
        self.act_seq += 1
        slot = (self.act_seq - 1) % 2
        phase("downloaded", ms=int((time.time() - t0) * 1000), bytes=len(data), slot=slot, seq=self.act_seq)
        ok = all(x in data for x in V2_EXPORTS)
        if d.get("reboot", True):
            if ok:
                self.actor = {"src": f"slot{slot}.wasm", "seq": self.act_seq, "name": d.get("name", "actor")}
                self.v2 = True
            self.ota_running = False
            self.reboot("sw")
            # main.c falls back to the embedded actor when the new one cannot be loaded
            phase("ok" if ok else "fallback", want_seq=self.act_seq, actor=self.actor)
        else:
            self.ota_running = False

    # ---- BSP (node_v2.c dyne_v2_run_bsp) ----
    def run_bsp_safe(self):
        try:
            self.run_bsp()
        except Exception as e:  # noqa: BLE001
            self.pub_status("err", err=f"fake: {e!r}"[:90])
        finally:
            self.busy = False

    def run_bsp(self):
        c = self.cfg
        blk = Wave2DBlock(c["nx"], c["ny"], c["ix0"], c["iy0"], c["h"], c["alpha"], c["dt"],
                          c["kinds"], c["params"], c["cfgv"])
        links = [dict(ln, slot=[None, None], pend=None, got=False, last_send=0.0, echoed=False, last_echo=0.0)
                 for ln in c["links"]]
        by_id = {ln["id"]: ln for ln in links}
        sock = self.udp
        sock.settimeout(0)
        try:  # the board binds its UDP port only while running: nothing from before the start is kept
            while True:
                sock.recvfrom(65536)
        except OSError:
            pass
        sock.settimeout(RECV_SLICE_S)
        st = dict(sent=0, resend=0, replay=0, echo=0, stall=0, bad_env=0, missing=0, test_dropped=0)
        now_us = lambda: int(time.monotonic() * 1e6)  # noqa: E731
        idx = c["idx"]
        w_step = w_comp = 0.0
        w_n = 0
        aborted = False
        for step in range(c["steps"]):
            if self.die_step is not None and step >= self.die_step:
                self.dead = True  # silent from now on: no datagrams, no MQTT (as if the battery fell off)
                return
            t0 = time.monotonic()
            for ln in links:  # 1) send to every interface, keep the copy
                vals = blk.edge_values(ln["face"])
                dg = DGRAM.pack(MAGIC, ln["id"], step, len(vals)) + struct.pack(f"<{len(vals)}d", *vals)
                ln["slot"][step & 1] = (step, dg)
                sock.sendto(dg, ln["peer"])
                st["sent"] += 1
                ln["last_send"] = now_us()
                ln["echoed"] = False
            for ln in links:  # 2) values that arrived early
                if ln["pend"] is not None and ln["pend"][0] == step:
                    blk.put_ghost(ln["face"], ln["pend"][1])
                    ln["got"] = True
                    ln["pend"] = None
            wait0 = now_us()
            n_stall = 0
            while not all(ln["got"] for ln in links):  # 3) receive and resend until every interface arrived
                if self.stop_req:
                    aborted = True
                    break
                now = now_us()
                for ln in links:
                    if not ln["got"] and now - ln["last_send"] >= TIMEOUT_US:
                        sock.sendto(ln["slot"][step & 1][1], ln["peer"])
                        st["resend"] += 1
                        ln["last_send"] = now + RESEND_MIN_US - TIMEOUT_US
                if c["stall_ms"] > 0 and now - wait0 >= c["stall_ms"] * 1000 * (n_stall + 1):
                    miss = [ln["id"] for ln in links if not ln["got"]]
                    self.pub_status("running", stall=True, step=step, wait_ms=(now - wait0) // 1000, missing=miss,
                                    sent=st["sent"], resend=st["resend"], replay=st["replay"], echo=st["echo"],
                                    bad_env=st["bad_env"])
                    n_stall += 1
                    st["stall"] += 1
                try:
                    data, _ = sock.recvfrom(65536)
                except OSError:  # timeout, or ICMP "port unreachable" reported as an error on Windows
                    continue
                if c["test_drop"] > 0.0 and self.rng.random() < c["test_drop"]:
                    st["test_dropped"] += 1
                    continue
                now = now_us()
                if len(data) < DGRAM.size or data[:4] != MAGIC:
                    st["bad_env"] += 1
                    continue
                _, iid, mstep, cnt = DGRAM.unpack_from(data)
                ln = by_id.get(iid)
                if ln is None:
                    continue
                if mstep > step:  # hold
                    ln["pend"] = (mstep, list(struct.unpack_from(f"<{cnt}d", data, DGRAM.size)))
                elif mstep < step:  # past: answer with that step's copy
                    s = ln["slot"][mstep & 1]
                    if s is not None and s[0] == mstep:
                        sock.sendto(s[1], ln["peer"])
                        st["replay"] += 1
                elif not ln["got"]:  # accept
                    vals = list(struct.unpack_from(f"<{cnt}d", data, DGRAM.size))
                    if len(vals) != blk.face_len(ln["face"]):
                        raise RuntimeError("inbox rejected")
                    blk.put_ghost(ln["face"], vals)
                    ln["got"] = True
                elif not ln["echoed"] or now - ln["last_echo"] >= RESEND_MIN_US:  # same-step duplicate: echo
                    s = ln["slot"][step & 1]
                    if s is not None and s[0] == step:
                        sock.sendto(s[1], ln["peer"])
                        ln["echoed"], ln["last_echo"] = True, now
                        st["replay"] += 1
                        st["echo"] += 1
            if aborted:
                break
            if n_stall > 0:
                self.pub_status("running", step=step, stall_cleared=True, wait_ms=(now_us() - wait0) // 1000)
            t1 = time.monotonic()
            for ln in links:
                ln["got"] = False
            if not blk.step():
                st["missing"] += 1
            t2 = time.monotonic()
            w_step += (t2 - t0) * 1000
            w_comp += (t2 - t1) * 1000
            w_n += 1
            if c["field_every"] > 0 and (step + 1) % c["field_every"] == 0:
                self.pub_summary(blk, step)
            if (c["rt_every"] > 0 and (step + 1) % c["rt_every"] == 0) or self.report_req:
                self.pub(f"node/{idx}/rt", {"step": step, "window": w_n, "step_ms": {"mean": round(w_step / w_n, 3)},
                                            "compute_ms": {"mean": round(w_comp / w_n, 3)}, "wire": 2})
                w_step = w_comp = 0.0
                w_n = 0
                self.pub_status("running", step=step + 1)
                self.report_req = False
        if aborted:
            self.stop_req = False
            self.pub_status("configured", note="stopped")
            return
        end = time.monotonic() + self.a.drain_ms / 1000  # drain: keep answering late neighbours
        while time.monotonic() < end:
            try:
                data, _ = sock.recvfrom(65536)
            except OSError:
                continue
            if len(data) < DGRAM.size or data[:4] != MAGIC:
                continue
            _, iid, mstep, _ = DGRAM.unpack_from(data)
            ln = by_id.get(iid)
            s = ln["slot"][mstep & 1] if ln else None
            if s is not None and s[0] == mstep:
                sock.sendto(s[1], ln["peer"])
                st["replay"] += 1
        crc, lo, hi = blk.state_crc()
        self.pub_summary(blk, c["steps"] - 1)
        extra = dict(wire=2, crc32=f"0x{crc:08X}", sent=st["sent"], resend=st["resend"], replay=st["replay"],
                     echo=st["echo"], stall_reports=st["stall"], bad_env=st["bad_env"], rejected=0,
                     missing=st["missing"], dropped=0)
        if c["test_drop"] > 0.0:
            extra.update(test_drop=round(c["test_drop"], 3), test_dropped=st["test_dropped"])
        self.pub_status("done", **extra)

    def pub_summary(self, blk, step):
        crc, lo, hi = blk.state_crc()
        c = self.cfg
        self.pub(f"node/{c['idx']}/field2d", {"step": step, "nx": c["nx"], "ny": c["ny"], "ix0": c["ix0"],
                                              "iy0": c["iy0"], "crc32": f"0x{crc:08X}", "min": lo, "max": hi})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--broker", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--ids", required=True, help="node ids, comma separated (e.g. 11,12,13,14)")
    ap.add_argument("--die", action="append", default=[], metavar="ID@STEP",
                    help="node ID goes silent at step STEP (fault injection; repeatable)")
    ap.add_argument("--drain-ms", type=int, default=3000, help="answer late neighbours this long after the last step")
    ap.add_argument("--seed", type=int, default=1, help="seed of mode.test_drop (the board uses a hardware RNG)")
    ap.add_argument("--mqtt", default="auto", choices=["auto", "paho", "lite"])
    a = ap.parse_args()
    a.die = {int(x.split("@")[0]): int(x.split("@")[1]) for x in a.die}
    mqtt = dyne_mqtt_lite.pick(a.mqtt)
    ids = [int(x) for x in a.ids.split(",") if x.strip()]
    nodes = [FakeNode(i, a, mqtt) for i in ids]
    print(f"fake nodes up: {', '.join(f'{n.nid} (udp {n.udp_port})' for n in nodes)}", flush=True)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

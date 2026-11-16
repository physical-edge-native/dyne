#!/usr/bin/env python3
"""Run the 2D wave equation (wave2d small actor) on a cluster of nodes and compare with dyne-sim2d
(GitHub issue #4, 2026-11-16). Runbook: docs/2026-11-16-wave2d-cluster-runbook.md.

The 1D tool scripts/mqtt_config.py does not know the 2D configuration (field2d / faces / ifaces of the
wire-v2 path, node-esp32/main/node_v2.c). This tool does the whole 2D flow after the actor is installed
(scripts/dyne_ota.py actor):

  1. expected values: per-node CRC32 / min / max of the final state from dyne-sim2d (scripts/wave2d_expected.py)
  2. preflight: every node online, not running, its IP known (retained status, or the cache of earlier runs)
  3. inject: one retained dyne/cfg/<id> per node (partition, faces, interface ids, peers' ip:port, mode),
     then wait until every node acknowledges the same gen and the CRC32 of the exact payload
  4. start: {"cmd":"start"} to each node of the run (not to dyne/ctl/all, so idle nodes outside the run stay idle)
  5. collect: running / stall reports (translated to "waiting for which neighbour") / done with crc32, and the
     per-step exchange wait (step time minus compute time of the rt reports; issue #42)
  6. compare: PASS (bit-exact) only if every node's crc32 equals dyne-sim2d's for its block

Subcommands (repository root; Windows: .venv\\Scripts\\python.exe, elsewhere python3):
  status    list the nodes the broker knows (id, state, ip, actor)
  expected  write the expected values for a layout to JSON (no broker needed)
  run       steps 1-6 above
  stop      send {"cmd":"stop"} to the given nodes

Examples:
  python scripts/wave2d_cluster.py status --broker 192.168.10.2
  python scripts/wave2d_cluster.py run --broker 192.168.10.2 --layout 2x2 --nodes 11,12,13,14 --test-drop 0.3
  python scripts/wave2d_cluster.py stop --broker 192.168.10.2 --nodes 11,12,13,14

Node order in --nodes is the block order of dyne-sim2d: index = by*px + bx (x first), so for 2x2 the list is
lower-left, lower-right, upper-left, upper-right. Exit codes: 0 PASS, 1 FAIL (mismatch, error, timeout),
2 not started (preflight or configuration rejected).
MQTT: paho-mqtt when installed, otherwise the standard-library client scripts/dyne_mqtt_lite.py (--mqtt).
"""
import argparse
import csv
import json
import os
import sys
import threading
import time
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dyne_mqtt_lite  # noqa: E402
import wave2d_expected as wx  # noqa: E402

DEFAULT_OUT = os.path.join(wx.ROOT, "wave2d_out")
UDP_PORT = 7777  # CONFIG_DYNE_UDP_PORT of node-esp32


def parse_layout(s):
    px, py = (int(v) for v in s.lower().split("x"))
    return px, py


# ---------------------------------------------------------------- MQTT observer

class Observer:
    """Collects status / ack / field2d / rt messages with their arrival time."""

    def __init__(self, mqtt, broker, port, verbose=False):
        self.lock = threading.Lock()
        self.status = {}  # node id -> latest status dict
        self.online = {}  # node id -> latest online status dict (has ip, actor)
        self.events = []  # (t, topic, dict)
        self.verbose = verbose
        self.cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"dyne-wave2d-{os.getpid()}")
        self.cl.on_message = self._on_msg
        self.cl.on_connect = lambda c, u, f, rc, p=None: c.subscribe(
            [("dyne/node/+/status", 1), ("dyne/cfg/+/ack", 1), ("dyne/node/+/field2d", 0), ("dyne/node/+/rt", 0)])
        self.cl.connect(broker, port, 60)
        self.cl.loop_start()

    def _on_msg(self, cl, u, m):
        try:
            d = json.loads(m.payload) if m.payload else None
        except ValueError:
            return
        if not isinstance(d, dict):
            return
        parts = m.topic.split("/")
        now = time.time()
        with self.lock:
            self.events.append((now, m.topic, d))
            if len(parts) == 4 and parts[1] == "node" and parts[3] == "status":
                nid = int(parts[2])
                self.status[nid] = d
                if d.get("state") == "online":
                    self.online[nid] = d
        if self.verbose:
            print(f"  [mqtt] {m.topic} {json.dumps(d)[:200]}", flush=True)

    def since(self, t0, pos=0):
        with self.lock:
            ev = self.events[pos:]
            return [e for e in ev if e[0] >= t0], len(self.events)

    def publish(self, topic, obj, retain=False):
        self.cl.publish(topic, json.dumps(obj) if not isinstance(obj, (bytes, str)) else obj,
                        qos=1, retain=retain).wait_for_publish()

    def close(self):
        self.cl.loop_stop()
        self.cl.disconnect()


# ---------------------------------------------------------------- exchange wait (issue #42)

def wait_summary(waits):
    """Per-step exchange wait of one node: step time minus compute time [ms], one value per rt report.

    Each rt report holds the means over its window of rt_every steps, so the values are window means
    (run with --rt-every 1 to get one value per step). Returns n / median / p95 / max (nearest rank), or {}."""
    w = sorted(waits)
    if not w:
        return {}
    pick = lambda q: w[min(len(w) - 1, max(0, -(-len(w) * q // 100) - 1))]  # noqa: E731  nearest rank
    return {"n": len(w), "median": round(pick(50), 3), "p95": round(pick(95), 3), "max": round(w[-1], 3)}


# ---------------------------------------------------------------- node cache (ip, port, actor)

def load_cache(path):
    try:
        with open(path, encoding="utf-8") as f:
            return {int(k): v for k, v in json.load(f).items()}
    except (OSError, ValueError):
        return {}


def save_cache(path, cache):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump({str(k): v for k, v in sorted(cache.items())}, f, indent=1)


def update_cache(cache, obs, default_port):
    with obs.lock:
        for nid, d in obs.online.items():
            if d.get("ip"):
                cache[nid] = {"ip": d["ip"], "port": int(d.get("udp_port", default_port)),
                              "actor_at_online": (d.get("actor") or {}).get("name", ""), "mac": d.get("mac", ""),
                              "fw": d.get("fw", ""), "seen": time.strftime("%Y-%m-%d %H:%M:%S")}


# ---------------------------------------------------------------- expected values

def expected_for(args, px, py):
    if args.expected:
        with open(args.expected, encoding="utf-8") as f:
            e = json.load(f)
        if (e["px"], e["py"], e["scene"], e["steps"]) != (px, py, args.scene, args.steps):
            raise SystemExit(f"{args.expected}: made for {e['px']}x{e['py']} {e['scene']} {e['steps']} steps, "
                             f"not {px}x{py} {args.scene} {args.steps} steps")
        return e
    sim = args.sim or wx.default_sim()
    if not os.path.exists(sim) or not os.path.exists(args.wasm):
        raise SystemExit(f"need dyne-sim2d ({sim}) and the full wave2d actor ({args.wasm}) to compute the expected "
                         "values; build them (runbook step 0) or pass --expected FILE")
    crc_line, nodes = wx.expected_nodes(sim, px, py, args.scene, args.steps, args.wasm)
    return {"px": px, "py": py, "scene": args.scene, "steps": args.steps, "sim": crc_line,
            "nodes": [dict(n, crc32=f"0x{n['crc32']:08X}") for n in nodes]}


def cmd_expected(args):
    px, py = parse_layout(args.layout)
    e = expected_for(args, px, py)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(e, f, indent=1)
    print(f"{args.scene} {px}x{py}, {args.steps} steps, dyne-sim2d {e['sim']}")
    for n in e["nodes"]:
        print(f"  idx {n['idx']:>2}  block ({n['ix0']},{n['iy0']}) {n['nx']}x{n['ny']}  crc32 {n['crc32']}")
    print(f"written: {args.out}")
    return 0


# ---------------------------------------------------------------- status / stop

def cmd_status(args, mqtt):
    obs = Observer(mqtt, args.broker, args.mqtt_port)
    time.sleep(args.wait)
    cache = load_cache(args.cache)
    update_cache(cache, obs, args.udp_port)
    save_cache(args.cache, cache)
    obs.close()
    print(f"{'id':>4} {'state':<11} {'idx':>4} {'ip:port':<22} {'actor (last online)':<22} {'fw':<16} up_s")
    for nid in sorted(set(obs.status) | set(cache)):
        d = obs.status.get(nid, {})
        c = cache.get(nid, {})
        ipp = f"{c['ip']}:{c['port']}" if c else "?"
        print(f"{nid:>4} {d.get('state', '-'):<11} {str(d.get('idx', '')):>4} {ipp:<22} {c.get('actor_at_online', '?'):<22} "
              f"{c.get('fw', '?'):<16} {d.get('up_s', '')}")
    return 0


def cmd_stop(args, mqtt):
    obs = Observer(mqtt, args.broker, args.mqtt_port)
    ids = [int(x) for x in args.nodes.split(",")] if args.nodes != "all" else None
    for nid in ids or ["all"]:
        obs.publish(f"dyne/ctl/{nid}", {"cmd": "stop"})
        print(f"published dyne/ctl/{nid} stop (retain=False)")
    obs.close()
    return 0


# ---------------------------------------------------------------- run

def cmd_run(args, mqtt):
    px, py = parse_layout(args.layout)
    ids = [int(x) for x in args.nodes.split(",") if x.strip()]
    if len(ids) != px * py or len(set(ids)) != len(ids):
        raise SystemExit(f"--nodes must list {px * py} different node ids (block order), got {ids}")
    exp = expected_for(args, px, py)
    print(f"== {args.scene} {px}x{py} ({px * py} nodes), {args.steps} steps; dyne-sim2d whole field {exp['sim']}")
    run_dir = os.path.join(args.out, time.strftime("%Y%m%d-%H%M%S") + f"_{px}x{py}_{args.scene}"
                           + (f"_drop{args.test_drop:g}" if args.test_drop > 0 else ""))
    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, "expected.json"), "w", encoding="utf-8") as f:
        json.dump(exp, f, indent=1)

    obs = Observer(mqtt, args.broker, args.mqtt_port, args.verbose)
    try:
        return _run(args, obs, px, py, ids, exp, run_dir)
    finally:
        with obs.lock:
            ev = list(obs.events)
        with open(os.path.join(run_dir, "events.jsonl"), "w", encoding="utf-8") as f:
            for t, topic, d in ev:
                f.write(json.dumps({"t": round(t, 3), "topic": topic, "msg": d}) + "\n")
        obs.close()


def _run(args, obs, px, py, ids, exp, run_dir):
    time.sleep(args.wait)  # retained status
    cache = load_cache(args.cache)
    update_cache(cache, obs, args.udp_port)

    # ---- 2. preflight
    def problems():
        out = []
        for i, nid in enumerate(ids):
            st = obs.status.get(nid)
            if st is None:
                out.append((nid, f"idx {i}: node {nid} has no status (never connected to this broker?)"))
            elif st.get("state") == "offline":
                out.append((nid, f"idx {i}: node {nid} is offline ({st.get('reason', '')})"))
            elif st.get("state") == "running":
                out.append((nid, f"idx {i}: node {nid} is still running (stop it first: wave2d_cluster.py stop)"))
            elif nid not in cache:
                out.append((nid, f"idx {i}: node {nid}: IP unknown (its retained status is "
                                 f"'{st.get('state')}', not 'online', and it is not in {args.cache})"))
        return out

    bad = problems()
    if bad and args.reboot_missing:
        targets = [nid for nid, _ in bad if nid not in cache and obs.status.get(nid, {}).get("state") != "running"]
        for nid in targets:
            print(f"  node {nid}: IP unknown -> reboot to get a fresh online status")
            obs.publish(f"dyne/ctl/{nid}", {"cmd": "reboot"})
        t_end = time.time() + 40
        while time.time() < t_end and any(n not in obs.online for n in targets):
            time.sleep(0.5)
        update_cache(cache, obs, args.udp_port)
        bad = problems()
    save_cache(args.cache, cache)
    # the actor name is only in the online status: a node that was configured since then shows "?" here,
    # and the configuration ack is the real check (the 1D actor rejects a wire-v2 configuration)
    actors = {nid: (d.get("actor") or {}).get("name", "?") for nid, d in obs.online.items()}
    print(f"{'idx':>3} {'node':>4} {'state':<11} {'ip:port':<22} {'actor (online)':<22} block")
    for i, nid in enumerate(ids):
        n = exp["nodes"][i]
        c = cache.get(nid, {})
        ipp = f"{c['ip']}:{c['port']}" if c else "?"
        print(f"{i:>3} {nid:>4} {obs.status.get(nid, {}).get('state', '-'):<11} {ipp:<22} "
              f"{actors.get(nid, '?'):<22} ({n['ix0']},{n['iy0']}) {n['nx']}x{n['ny']}")
    if bad:
        for _, msg in bad:
            print(f"NOT STARTED: {msg}")
        print("hint: check `wave2d_cluster.py status`; a node whose IP is unknown can be rebooted (--reboot-missing)")
        return 2
    for nid in ids:
        actor = actors.get(nid)
        if args.actor_name and actor is not None and actor != args.actor_name:
            print(f"WARNING: node {nid} reports actor '{actor}', expected '{args.actor_name}' "
                  "(a configuration for the wire-v2 path is rejected by the 1D actor; OTA the actor first)")

    # ---- 3. inject
    gen = int(time.time())
    peers = {i: f"{cache[nid]['ip']}:{cache[nid]['port']}" for i, nid in enumerate(ids)}
    mode = {"field_every": args.field_every, "rt_every": args.rt_every, "stall_ms": args.stall_ms}
    if args.test_drop > 0:
        mode["test_drop"] = args.test_drop
    payloads = {}
    for i, nid in enumerate(ids):
        b = wx.blocks(px, py)[i]
        cfg = wx.node_cfg(args.scene, b, args.steps, peers, mode, gen)
        payloads[nid] = json.dumps(cfg).encode("utf-8")
        with open(os.path.join(run_dir, f"cfg_node{nid}.json"), "wb") as f:
            f.write(payloads[nid])
    t_cfg = time.time()
    for nid in ids:
        obs.publish(f"dyne/cfg/{nid}", payloads[nid], retain=True)
    print(f"published {len(ids)} configurations (gen {gen}, retained); waiting for the acks")
    acks = {}
    t_end = time.time() + args.ack_timeout
    while time.time() < t_end and len(acks) < len(ids):
        ev, _ = obs.since(t_cfg)
        for t, topic, d in ev:
            p = topic.split("/")
            if len(p) == 4 and p[1] == "cfg" and p[3] == "ack" and int(p[2]) in payloads:
                acks[int(p[2])] = d
        time.sleep(0.1)
    bad = []
    for i, nid in enumerate(ids):
        a = acks.get(nid)
        want_crc = f"0x{zlib.crc32(payloads[nid]) & 0xFFFFFFFF:08X}"
        if a is None:
            bad.append(f"node {nid}: no ack within {args.ack_timeout:g} s (powered? connected? see its status)")
        elif not a.get("ok"):
            hint = " -> the node runs the 1D actor: OTA the wave2d actor first" if "bc missing" in str(a.get("err")) else ""
            bad.append(f"node {nid}: configuration rejected: {a.get('err')}{hint}")
        elif a.get("gen") != gen or a.get("idx") != i or a.get("crc32") != want_crc:
            bad.append(f"node {nid}: ack does not match (gen {a.get('gen')} idx {a.get('idx')} crc {a.get('crc32')}; "
                       f"sent gen {gen} idx {i} crc {want_crc})")
    if bad:
        for b in bad:
            print(f"NOT STARTED: {b}")
        return 2
    print(f"all {len(ids)} acks ok (gen {gen}, payload CRC32 match, wire 2)")
    if args.no_start:
        print("--no-start: configured, not started")
        return 0

    # ---- 4. start
    t_start = time.time()
    for nid in ids:
        obs.publish(f"dyne/ctl/{nid}", {"cmd": "start"})
    print(f"start sent to {len(ids)} nodes")

    # ---- 5. collect
    idx_of = {nid: i for i, nid in enumerate(ids)}
    iface_src = {}  # (receiving idx, iface id) -> sending idx
    for b in wx.blocks(px, py):
        for f in b["faces"]:
            if f is not None:
                iface_src[(b["idx"], f[0])] = f[1]
    done, err, last_step, stalls, final_sum = {}, {}, {}, [], {}
    waits = {}  # idx -> [step_ms - compute_ms of each rt report]
    pos, t_print = 0, time.time()
    deadline = t_start + args.timeout
    while time.time() < deadline and len(done) + len(err) < len(ids):
        time.sleep(0.2)
        ev, pos = obs.since(t_start, pos)
        for t, topic, d in ev:
            p = topic.split("/")
            if len(p) != 4 or p[1] != "node":
                continue
            if p[3] == "field2d" and d.get("step") == args.steps - 1:
                final_sum[int(p[2])] = d  # topic id = idx (node_v2.c publish_summary)
                continue
            if p[3] == "rt":  # topic id = idx, like field2d
                try:
                    waits.setdefault(int(p[2]), []).append(float(d["step_ms"]["mean"]) - float(d["compute_ms"]["mean"]))
                except (KeyError, TypeError, ValueError):
                    pass
                continue
            if p[3] != "status" or int(p[2]) not in idx_of:
                continue
            nid = int(p[2])
            i = idx_of[nid]
            st = d.get("state")
            if st == "running" and d.get("stall"):
                miss = ", ".join(f"iface {m} from idx {iface_src.get((i, m), '?')} (node "
                                 f"{ids[iface_src[(i, m)]] if (i, m) in iface_src else '?'})" for m in d.get("missing", []))
                line = (f"STALL  node {nid} (idx {i}) at step {d.get('step')}, waited {d.get('wait_ms')} ms, "
                        f"missing: {miss}  [sent {d.get('sent')} resend {d.get('resend')} echo {d.get('echo')}]")
                stalls.append(line)
                print(line, flush=True)
            elif st == "running" and d.get("stall_cleared"):
                print(f"STALL cleared: node {nid} (idx {i}) step {d.get('step')} after {d.get('wait_ms')} ms", flush=True)
            elif st == "running" and "step" in d:
                last_step[nid] = d["step"]
            elif st == "done":
                done[nid] = d
                print(f"done   node {nid} (idx {i}) crc32 {d.get('crc32')}  ({t - t_start:.1f} s)", flush=True)
            elif st in ("err", "offline"):
                err[nid] = d
                print(f"ERROR  node {nid} (idx {i}): {json.dumps(d)}", flush=True)
            elif st == "configured" and d.get("note") == "stopped":
                err[nid] = d
                print(f"STOPPED node {nid} (idx {i})", flush=True)
        if time.time() - t_print >= args.progress_s and len(done) + len(err) < len(ids):
            t_print = time.time()
            print(f"  t = {t_print - t_start:.0f} s, steps reported: "
                  + " ".join(f"{ids[i]}:{last_step.get(ids[i], '-')}" for i in range(len(ids))), flush=True)
    timed_out = len(done) + len(err) < len(ids)

    # ---- 6. compare
    rows = []
    for i, nid in enumerate(ids):
        e = exp["nodes"][i]
        d = done.get(nid, {})
        got = d.get("crc32", "")
        s = final_sum.get(i, {})
        mm = ""
        if s:
            mm = "yes" if (s.get("min") == e["min"] and s.get("max") == e["max"]) else "NO"
        rows.append({"idx": i, "node": nid, "ip_port": peers[i], "expected": e["crc32"], "got": got,
                     "match": "yes" if got == e["crc32"] else "NO", "minmax_match": mm,
                     "state": "done" if nid in done else (err.get(nid, {}).get("state") or "timeout"),
                     "last_step": last_step.get(nid, ""), **{k: d.get(k, "") for k in (
                         "sent", "resend", "replay", "echo", "stall_reports", "missing", "test_dropped")},
                     **{f"wait_ms_{k}": v for k, v in wait_summary(waits.get(i, [])).items()}})
    for r in rows:  # same columns in every row (a node without rt reports gets empty cells)
        for k in ("wait_ms_n", "wait_ms_median", "wait_ms_p95", "wait_ms_max"):
            r.setdefault(k, "")
    print(f"\n{'idx':>3} {'node':>4} {'expected':<10} {'got':<10} {'match':<5} {'min/max':<7} {'sent':>6} "
          f"{'resend':>6} {'replay':>6} {'echo':>5} {'stall':>5} {'dropped':>7}")
    for r in rows:
        print(f"{r['idx']:>3} {r['node']:>4} {r['expected']:<10} {r['got'] or r['state']:<10} {r['match']:<5} "
              f"{r['minmax_match'] or '-':<7} {str(r['sent']):>6} {str(r['resend']):>6} {str(r['replay']):>6} "
              f"{str(r['echo']):>5} {str(r['stall_reports']):>5} {str(r['test_dropped']):>7}")
    print(f"\nexchange wait per step = step_ms - compute_ms [ms], over the rt reports "
          f"(each the mean of {args.rt_every} steps; --rt-every 1 gives per-step values)")
    print(f"{'idx':>3} {'node':>4} {'reports':>7} {'median':>8} {'p95':>8} {'max':>8}")
    for r in rows:
        print(f"{r['idx']:>3} {r['node']:>4} {str(r['wait_ms_n']):>7} {str(r['wait_ms_median']):>8} "
              f"{str(r['wait_ms_p95']):>8} {str(r['wait_ms_max']):>8}")
    n_ok = sum(r["match"] == "yes" for r in rows)
    if args.test_drop > 0 and done and all(not r["test_dropped"] for r in rows if r["state"] == "done"):
        print(f"WARNING: --test-drop {args.test_drop} but no node reports test_dropped > 0 (old firmware?)")
    if any(r["minmax_match"] == "NO" for r in rows):
        print("WARNING: final min/max of some node differ from dyne-sim2d (field2d summary)")
    with open(os.path.join(run_dir, "result.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    verdict = n_ok == len(ids)
    with open(os.path.join(run_dir, "result.json"), "w", encoding="utf-8") as f:
        json.dump({"layout": f"{px}x{py}", "scene": args.scene, "steps": args.steps, "test_drop": args.test_drop,
                   "rt_every": args.rt_every,
                   "gen": gen, "nodes": ids, "pass": verdict, "timed_out": timed_out, "stalls": stalls,
                   "seconds": round(time.time() - t_start, 1), "rows": rows, "done": done, "err": err}, f, indent=1)
    print(f"record: {run_dir}")
    if verdict:
        print(f"PASS (bit-exact): {n_ok}/{len(ids)} nodes match dyne-sim2d "
              f"({args.scene} {px}x{py}, {args.steps} steps, {time.time() - t_start:.1f} s)")
        return 0
    why = []
    if timed_out:
        why.append(f"timeout after {args.timeout:g} s ({len(ids) - len(done) - len(err)} nodes not done)")
    if err:
        why.append(f"{len(err)} nodes reported an error or were stopped")
    if done and n_ok < len(done):
        why.append(f"{len(done) - n_ok} finished nodes have a different crc32")
    print(f"FAIL: {n_ok}/{len(ids)} nodes match dyne-sim2d; " + "; ".join(why))
    if timed_out and args.stop_on_fail:
        for nid in ids:
            if nid not in done:
                obs.publish(f"dyne/ctl/{nid}", {"cmd": "stop"})
        print("sent stop to the nodes that did not finish")
    elif timed_out:
        print(f"the nodes are left running; stop them with: wave2d_cluster.py stop --broker {args.broker} "
              f"--nodes {args.nodes}")
    return 1


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def mqtt_opts(p):
        p.add_argument("--broker", required=True, help="MQTT broker address (the PC)")
        p.add_argument("--mqtt-port", type=int, default=1883)
        p.add_argument("--mqtt", default="auto", choices=["auto", "paho", "lite"], help="MQTT client library")
        p.add_argument("--cache", default=os.path.join(DEFAULT_OUT, "nodes.json"),
                       help="node ip/port/actor seen in online statuses (kept between runs)")
        p.add_argument("--udp-port", type=int, default=UDP_PORT, help="data-plane UDP port of the nodes")
        p.add_argument("--wait", type=float, default=1.5, help="seconds to collect retained statuses")

    def problem_opts(p):
        p.add_argument("--layout", default="2x2", help="PXxPY: 2x2, 4x2, 4x4 (any split of 81 points works)")
        p.add_argument("--scene", default="gauss_dirichlet", choices=sorted(wx.SCENES))
        p.add_argument("--steps", type=int, default=wx.STEPS)
        p.add_argument("--sim", help="dyne-sim2d executable (default target/release/dyne-sim2d)")
        p.add_argument("--wasm", default=wx.WASM, help="FULL wave2d actor used by dyne-sim2d for the expected values")
        p.add_argument("--expected", help="expected values written by the expected subcommand (instead of --sim)")

    p = sub.add_parser("status", help="list the nodes")
    mqtt_opts(p)
    p = sub.add_parser("expected", help="write expected values to JSON")
    problem_opts(p)
    p.add_argument("--out", required=True)
    p = sub.add_parser("stop", help="send stop")
    mqtt_opts(p)
    p.add_argument("--nodes", required=True, help="ids or all")
    p = sub.add_parser("run", help="inject, start, collect, compare")
    mqtt_opts(p)
    problem_opts(p)
    p.add_argument("--nodes", required=True, help="node ids in block order (index = by*px + bx)")
    p.add_argument("--test-drop", type=float, default=0.0, help="mode.test_drop: drop this share of received datagrams")
    p.add_argument("--stall-ms", type=int, default=10000, help="mode.stall_ms: report a step that waits this long")
    p.add_argument("--field-every", type=int, default=100)
    p.add_argument("--rt-every", type=int, default=100)
    p.add_argument("--actor-name", default="wave2d_actor_small", help="expected actor name ('' = do not check)")
    p.add_argument("--ack-timeout", type=float, default=10.0)
    p.add_argument("--timeout", type=float, default=600.0, help="seconds from start until all nodes must be done")
    p.add_argument("--progress-s", type=float, default=10.0, help="progress line interval")
    p.add_argument("--reboot-missing", action="store_true", help="reboot nodes whose IP is unknown, then retry")
    p.add_argument("--stop-on-fail", action="store_true", help="on timeout, send stop to the unfinished nodes")
    p.add_argument("--no-start", action="store_true", help="inject and check the acks only")
    p.add_argument("--out", default=DEFAULT_OUT, help="record directory (one subdirectory per run)")
    p.add_argument("--verbose", action="store_true", help="print every MQTT message")
    a = ap.parse_args()
    if a.cmd == "expected":
        return cmd_expected(a)
    mqtt = dyne_mqtt_lite.pick(a.mqtt)
    return {"status": cmd_status, "stop": cmd_stop, "run": cmd_run}[a.cmd](a, mqtt)


if __name__ == "__main__":
    sys.exit(main())

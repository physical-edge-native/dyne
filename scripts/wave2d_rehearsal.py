#!/usr/bin/env python3
"""Rehearse the wave2d cluster runbook end to end without hardware (GitHub issue #4, 2026-11-16).

Runs the same PC commands as docs/2026-11-16-wave2d-cluster-runbook.md, against fake nodes
(scripts/wave2d_fake_nodes.py) and an in-process MQTT broker (scripts/dyne_mqtt_lite.py), so it needs
no mosquitto and no Python packages. scripts/ci-check.sh runs it on Linux, macOS and Windows.

Checks (each prints OK or FAIL):
  cfg-before-ota  a 2D configuration is rejected while the nodes still run the embedded 1D actor
  ota             scripts/dyne_ota.py actor installs the small wave2d actor on all 16 fake nodes
  status          wave2d_cluster.py status lists them with the new actor
  2x2-drop30      2x2 with mode.test_drop 0.3: every node's crc32 equals dyne-sim2d (bit-exact)
  4x2             4x2 (needs the node ip cache: the retained statuses no longer carry the ip)
  4x4             4x4, 16 nodes
  4x2-absorb      4x2 with absorbing outer faces
  stall           a node goes silent at step 20: its neighbours report the stall naming the missing
                  interface, the run fails within the time limit, and stop is sent

Usage (repository root, after building dyne-sim2d and both wave2d actors; see the runbook step 0):
  python scripts/wave2d_rehearsal.py [--sim target/release/dyne-sim2d] [--out target/ci-wave2d] [--quick]
--quick skips 4x4 and 4x2-absorb.
"""
import argparse
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dyne_mqtt_lite  # noqa: E402
import wave2d_expected as wx  # noqa: E402

SMALL = os.path.join(wx.ROOT, "crates", "wave2d-actor", "target-small", "wasm32-unknown-unknown", "release",
                     "wave2d_actor.wasm")


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", default=wx.default_sim())
    ap.add_argument("--wasm", default=wx.WASM, help="full wave2d actor (for dyne-sim2d)")
    ap.add_argument("--small", default=SMALL, help="small wave2d actor (distributed to the nodes)")
    ap.add_argument("--out", default=os.path.join(wx.ROOT, "target", "ci-wave2d"))
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--mqtt", default="lite", choices=["auto", "paho", "lite"],
                    help="client library of the tools (the broker is always the in-process one)")
    a = ap.parse_args()
    a.sim, a.wasm, a.small, a.out = (os.path.abspath(p) for p in (a.sim, a.wasm, a.small, a.out))
    for p in (a.sim, a.wasm, a.small):
        if not os.path.exists(p):
            print(f"missing {p} (build dyne-sim2d and the wave2d actors first)")
            return 1
    os.makedirs(a.out, exist_ok=True)
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1")
    py = sys.executable
    broker = dyne_mqtt_lite.Broker("127.0.0.1", 0)
    port = broker.start()
    print(f"broker 127.0.0.1:{port} (in-process); records in {a.out}", flush=True)
    common = ["--broker", "127.0.0.1", "--mqtt-port", str(port), "--mqtt", a.mqtt]
    cluster = [py, os.path.join(HERE, "wave2d_cluster.py")]
    cache = ["--cache", os.path.join(a.out, "nodes.json"), "--out", a.out, "--sim", a.sim, "--wasm", a.wasm]
    if os.path.exists(os.path.join(a.out, "nodes.json")):
        os.remove(os.path.join(a.out, "nodes.json"))
    procs, fail = [], 0

    def fakes(ids, *extra):
        log = open(os.path.join(a.out, f"fake_{ids[0]}.log"), "w", encoding="utf-8")
        p = subprocess.Popen([py, os.path.join(HERE, "wave2d_fake_nodes.py"), "--broker", "127.0.0.1",
                              "--port", str(port), "--mqtt", a.mqtt, "--ids", ",".join(map(str, ids)), *extra],
                             stdout=log, stderr=subprocess.STDOUT, env=env)
        procs.append((p, log))
        return p

    def check(name, cmd, want_rc, must=(), timeout=900):
        nonlocal fail
        print(f"$ {' '.join(os.path.relpath(c, wx.ROOT) if os.path.isabs(c) else c for c in cmd)}", flush=True)
        t0 = time.time()
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", env=env,
                               timeout=timeout)
            out, rc = r.stdout + r.stderr, r.returncode
        except subprocess.TimeoutExpired as e:
            out, rc = (e.stdout or "") if isinstance(e.stdout, str) else "", "timeout"
        with open(os.path.join(a.out, f"{name}.log"), "w", encoding="utf-8") as f:
            f.write(out)
        missing = [m for m in must if m not in out]
        ok = rc == want_rc and not missing
        print(f"{name:<16} {'OK' if ok else 'FAIL'} (exit {rc}, {time.time() - t0:.1f} s)"
              + ("" if ok else f"  expected exit {want_rc} and {missing}; see {name}.log"), flush=True)
        if not ok:
            fail = 1
            print("\n".join("    | " + ln for ln in out.splitlines()[-25:]))
        return out

    try:
        ids = list(range(11, 27))  # 16 nodes
        fakes(ids)
        time.sleep(1.5)
        nodes = lambda n: ",".join(map(str, ids[:n]))  # noqa: E731
        check("cfg-before-ota", cluster + ["run"] + common + cache + ["--layout", "2x2", "--nodes", nodes(4),
                                                                       "--no-start"],
              2, ["configuration rejected: cluster/node/bc missing", "OTA the wave2d actor first"])
        check("ota", [py, os.path.join(HERE, "dyne_ota.py"), "actor", "--broker", "127.0.0.1", "--mqtt-port",
                      str(port), "--wasm", a.small, "--name", "wave2d_actor_small", "--nodes", nodes(16),
                      "--http-host", "127.0.0.1", "--http-bind", "127.0.0.1", "--http-port", str(free_port()),
                      "--timeout", "60", "--csv", os.path.join(a.out, "ota_results.csv")],
              0, ["16/16"])
        check("status", cluster + ["status"] + common + ["--cache", os.path.join(a.out, "nodes.json")],
              0, ["wave2d_actor_small", "dyne-fake-node"])
        check("2x2-drop30", cluster + ["run"] + common + cache + ["--layout", "2x2", "--nodes", nodes(4),
                                                                   "--test-drop", "0.3"],
              0, ["PASS (bit-exact): 4/4"])
        check("4x2", cluster + ["run"] + common + cache + ["--layout", "4x2", "--nodes", nodes(8)],
              0, ["PASS (bit-exact): 8/8"])
        if not a.quick:
            check("4x4", cluster + ["run"] + common + cache + ["--layout", "4x4", "--nodes", nodes(16)],
                  0, ["PASS (bit-exact): 16/16"])
            check("4x2-absorb", cluster + ["run"] + common + cache + ["--layout", "4x2", "--nodes", nodes(8),
                                                                       "--scene", "gauss_absorb"],
                  0, ["PASS (bit-exact): 8/8"])
        # a node that goes silent: the neighbours report which interface they wait for
        sids = [31, 32, 33, 34]
        fakes(sids, "--die", "32@20")
        time.sleep(1.5)
        check("stall-ota", [py, os.path.join(HERE, "dyne_ota.py"), "actor", "--broker", "127.0.0.1",
                            "--mqtt-port", str(port), "--wasm", a.small, "--name", "wave2d_actor_small",
                            "--nodes", "31,32,33,34", "--http-host", "127.0.0.1", "--http-bind", "127.0.0.1",
                            "--http-port", str(free_port()), "--timeout", "60"], 0, ["4/4"])
        check("stall", cluster + ["run"] + common + cache + ["--layout", "2x2", "--nodes", "31,32,33,34",
                                                              "--steps", "60", "--stall-ms", "1000",
                                                              "--timeout", "8", "--stop-on-fail"],
              1, ["STALL  node 31 (idx 0) at step 20", "from idx 1 (node 32)", "sent stop", "FAIL: 0/4"])
    finally:
        for p, log in procs:
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
            log.close()
        broker.stop()
    print("ALL OK" if not fail else "REHEARSAL FAILED")
    return fail


if __name__ == "__main__":
    sys.exit(main())

"""Analyse a cooperative-run log: latency and jitter, throughput, steps per second (GitHub issue #167).

The log has one JSON object per line (format: docs/design/2026-11-26-coop-perf-log.md):
  {"k":"run",  ...}                                         header (source, nodes, steps; for the simulator also
                                                            step_us, lat_us, lat_jitter_us, loss, seed)
  {"k":"tx",   "node","iface","step","t","bytes","resend"}  a border message sent (t in µs)
  {"k":"rx",   "node","iface","step","t_sent","t","bytes"}  a border message received (t_sent = sender's time)
  {"k":"step", "node","step","t0","t1","t2"}                a finished step: t0 start, t1 all border values in, t2 done
dyne-sim2d writes it with --perf-log (virtual time); the firmware is to write the same format with synchronised clocks.

Reported:
  - one-way latency of border messages (t - t_sent): median, 95th and 99th percentile, maximum, per link and overall;
    jitter as the standard deviation and as the mean absolute difference of consecutive latencies on a link;
  - delivery ratio (messages received / sent, resends included);
  - throughput: bytes and messages sent per second, per device and in total;
  - steps per second per device (one step = one solver step on that device), and the step time split into
    "send and wait for the neighbours" (t1 - t0) and "compute" (t2 - t1).

Usage:
  python scripts/coop_perf.py LOG [--json out.json] [--check-sim]
  --check-sim (simulator logs only): the latency must lie in [lat, lat + 2·jitter] as configured, and its median must
  be within 5 % of lat + jitter (the virtual network of crates/dyne-core draws lat + a uniform integer in
  [0, 2·jitter] µs, at least 1 µs); exit 1 otherwise.
Standard library only.
"""

import argparse
import json
import math
import statistics
import sys
from collections import defaultdict


def pct(sorted_vals, p):
    """Nearest-rank percentile (p in [0, 100]) of a sorted list."""
    if not sorted_vals:
        return float("nan")
    k = max(0, min(len(sorted_vals) - 1, math.ceil(p / 100.0 * len(sorted_vals)) - 1))
    return sorted_vals[k]


def dist(vals):
    v = sorted(vals)
    if not v:
        return {"n": 0}
    return {"n": len(v), "min": v[0], "median": pct(v, 50), "p95": pct(v, 95), "p99": pct(v, 99), "max": v[-1],
            "mean": sum(v) / len(v), "std": statistics.pstdev(v) if len(v) > 1 else 0.0}


def load(path):
    run, tx, rx, steps = None, [], [], []
    with open(path, encoding="utf-8") as f:
        for ln, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            k = r.get("k")
            if k == "run":
                run = r
            elif k == "tx":
                tx.append(r)
            elif k == "rx":
                rx.append(r)
            elif k == "step":
                steps.append(r)
            else:
                raise SystemExit(f"{path}:{ln}: unknown record kind {k!r}")
    if run is None:
        raise SystemExit(f"{path}: no header line (k = run)")
    return run, tx, rx, steps


def analyse(run, tx, rx, steps):
    lat_ms = [(r["t"] - r["t_sent"]) / 1000.0 for r in rx]
    by_link = defaultdict(list)
    for r in sorted(rx, key=lambda r: (r["node"], r["iface"], r["t"])):
        by_link[(r["node"], r["iface"])].append((r["t"] - r["t_sent"]) / 1000.0)
    jit = []
    for vals in by_link.values():
        jit.extend(abs(b - a) for a, b in zip(vals, vals[1:]))

    nodes = sorted({r["node"] for r in steps} | {r["node"] for r in tx})
    per_node = {}
    t_first = min(r["t0"] for r in steps) if steps else 0
    t_last = max(r["t2"] for r in steps) if steps else 0
    for n in nodes:
        st = [r for r in steps if r["node"] == n]
        dur_s = (max(r["t2"] for r in st) - min(r["t0"] for r in st)) / 1e6 if st else float("nan")
        sent = [r for r in tx if r["node"] == n]
        per_node[n] = {
            "steps": len(st),
            "seconds": dur_s,
            "steps_per_s": len(st) / dur_s if dur_s > 0 else float("nan"),
            "bytes_sent_per_s": sum(r["bytes"] for r in sent) / dur_s if dur_s > 0 else float("nan"),
            "msgs_sent_per_s": len(sent) / dur_s if dur_s > 0 else float("nan"),
            "resends": sum(1 for r in sent if r["resend"]),
            "wait_ms": dist([(r["t1"] - r["t0"]) / 1000.0 for r in st]),
            "compute_ms": dist([(r["t2"] - r["t1"]) / 1000.0 for r in st]),
        }
    total_s = (t_last - t_first) / 1e6 if steps else float("nan")
    return {
        "source": run.get("source"),
        "nodes": len(nodes),
        "seconds": total_s,
        "latency_ms": dist(lat_ms),
        "jitter_ms": {"std": dist(lat_ms).get("std", float("nan")),
                      "mean_abs_consecutive_diff": (sum(jit) / len(jit)) if jit else float("nan")},
        "links": {f"{n}:{i}": dist(v) for (n, i), v in sorted(by_link.items())},
        "sent": len(tx),
        "received": len(rx),
        "delivery_ratio": len(rx) / len(tx) if tx else float("nan"),
        "bytes_sent_per_s_total": sum(r["bytes"] for r in tx) / total_s if total_s > 0 else float("nan"),
        "steps_per_s_min": min((p["steps_per_s"] for p in per_node.values()), default=float("nan")),
        "per_node": per_node,
    }


def report(res):
    L = res["latency_ms"]
    print(f"{res['source']}: {res['nodes']} devices, {res['seconds']:.2f} s, {res['sent']} messages sent, "
          f"{res['received']} received (delivery ratio {res['delivery_ratio']:.3f})")
    print(f"latency ms: median {L['median']:.3f} p95 {L['p95']:.3f} p99 {L['p99']:.3f} max {L['max']:.3f} "
          f"min {L['min']:.3f} | jitter ms: std {res['jitter_ms']['std']:.3f}, "
          f"mean |consecutive diff| {res['jitter_ms']['mean_abs_consecutive_diff']:.3f}")
    print(f"throughput: {res['bytes_sent_per_s_total']:.0f} bytes/s sent in total; slowest device "
          f"{res['steps_per_s_min']:.3f} steps/s")
    for n, p in res["per_node"].items():
        print(f"  device {n}: {p['steps']} steps, {p['steps_per_s']:.3f} steps/s, {p['bytes_sent_per_s']:.0f} bytes/s, "
              f"{p['msgs_sent_per_s']:.1f} msgs/s, {p['resends']} resends | wait ms median "
              f"{p['wait_ms'].get('median', float('nan')):.2f} p95 {p['wait_ms'].get('p95', float('nan')):.2f} | "
              f"compute ms median {p['compute_ms'].get('median', float('nan')):.2f}")


def check_sim(run, res):
    """The virtual network draws latency lat + uniform integer in [0, 2·jitter] µs (at least 1 µs)."""
    if run.get("source") != "dyne-sim2d":
        print("--check-sim: not a simulator log")
        return False
    lo = max(run["lat_us"], 1) / 1000.0
    hi = max(run["lat_us"] + 2 * run["lat_jitter_us"], 1) / 1000.0
    mid = (run["lat_us"] + run["lat_jitter_us"]) / 1000.0
    L = res["latency_ms"]
    ok = L["n"] > 0 and L["min"] >= lo - 1e-9 and L["max"] <= hi + 1e-9
    if run["lat_jitter_us"] > 0:
        ok &= abs(L["median"] - mid) <= 0.05 * mid
    print(f"check: latency in [{lo:.3f}, {hi:.3f}] ms and median {L['median']:.3f} within 5 % of {mid:.3f}: "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--json")
    ap.add_argument("--check-sim", action="store_true")
    a = ap.parse_args()
    run, tx, rx, steps = load(a.log)
    res = analyse(run, tx, rx, steps)
    report(res)
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=1)
    if a.check_sim and not check_sim(run, res):
        sys.exit(1)


if __name__ == "__main__":
    main()

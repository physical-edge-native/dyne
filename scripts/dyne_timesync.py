#!/usr/bin/env python3
"""ノードの時刻同期の設定と、同期の質の記録 (実験1、2026-11-11)。

ctl の timesync 命令をノードへ送り (試験モードの開始・停止、時刻サーバのポートなど)、
ノードが報告する dyne/node/<id>/tsync と health を CSV に記録して、要約を表示する。
説明: docs/2026-11-11-timesync-ota.md  必要パッケージ: paho-mqtt (既存の scripts と同じ)

使い方 (リポジトリのルートで):
  試験モード開始 (同期した時刻の整数秒でピン2を切り替え、報告は毎周期):
    python scripts\\dyne_timesync.py --broker 192.168.10.2 --nodes all --pps on --pps-gpio 2 --pps-src sync --report-every 1
  同期なしの比較 (各ノード自身の時計の整数秒): --pps-src local / SNTP だけ: --pps-src sntp
  記録だけ (600秒): python scripts\\dyne_timesync.py --broker ... --record 600 --csv tsync.csv
  試験モード停止: --pps off
"""
import argparse
import csv
import json
import os
import statistics
import sys
import time

import paho.mqtt.client as mqtt


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except AttributeError:
        pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--broker", required=True)
    ap.add_argument("--nodes", default="all", help="all (= dyne/ctl/all) または 1,2,3")
    ap.add_argument("--server", help="時刻サーバのアドレス (既定はノード側: ブローカーと同じ PC)")
    ap.add_argument("--port", type=int, help="往復の交換のポート (dyne_ntp_server.py の --port と合わせる)")
    ap.add_argument("--period-ms", type=int)
    ap.add_argument("--burst", type=int)
    ap.add_argument("--report-every", type=int)
    ap.add_argument("--pps", choices=["on", "off"])
    ap.add_argument("--pps-gpio", type=int)
    ap.add_argument("--pps-src", choices=["sync", "sntp", "local"])
    ap.add_argument("--record", type=float, default=0, help="報告を記録する秒数 (0 = 送るだけ)")
    ap.add_argument("--csv", default="tsync_log.csv")
    args = ap.parse_args()

    cmd = {"cmd": "timesync"}
    for k, v in (("server", args.server), ("port", args.port), ("period_ms", args.period_ms),
                 ("burst", args.burst), ("report_every", args.report_every), ("pps_gpio", args.pps_gpio),
                 ("pps_src", args.pps_src)):
        if v is not None:
            cmd[k] = v
    if args.pps:
        cmd["pps"] = args.pps == "on"

    rows = []
    cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"dyne-tsync-{os.getpid()}")

    def on_msg(c, u, m):
        p = m.topic.split("/")
        if len(p) != 4 or p[3] not in ("tsync", "health"):
            return
        try:
            d = json.loads(m.payload)
        except json.JSONDecodeError:
            return
        d = dict(d)
        pps = d.pop("pps", None)
        ts = d.pop("tsync", None)
        if isinstance(pps, dict):
            for k, v in pps.items():
                d[f"pps_{k}"] = v
        if isinstance(ts, dict):
            for k, v in ts.items():
                d[f"tsync_{k}"] = v
        rows.append({"pc_time": f"{time.time():.3f}", "node": int(p[2]), "topic": p[3], **d})

    cl.on_message = on_msg
    cl.on_connect = lambda c, u, f, rc, pr=None: c.subscribe([("dyne/node/+/tsync", 0), ("dyne/node/+/health", 0)])
    cl.connect(args.broker, 1883, 60)
    cl.loop_start()
    time.sleep(0.5)
    if len(cmd) > 1:
        targets = ["all"] if args.nodes == "all" else args.nodes.split(",")
        for t in targets:
            cl.publish(f"dyne/ctl/{t}", json.dumps(cmd), qos=1, retain=False)
        print(f"送信: {cmd} → {targets}")
    if args.record > 0:
        print(f"{args.record:.0f} 秒記録する")
        time.sleep(args.record)
    else:
        time.sleep(3)
    cl.loop_stop()
    cl.disconnect()
    if not rows:
        print("報告を受け取っていない")
        return 0
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    new = not os.path.exists(args.csv)
    with open(args.csv, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerows(rows)
    print(f"記録 {len(rows)} 行: {args.csv} (追記。列は最初の書き込み時のもの)")
    ts = [r for r in rows if r["topic"] == "tsync" and r.get("synced")]
    print(f"{'node':>4} {'n':>4} {'rtt_min_us':>10} {'rtt_med_us':>10} {'|resid|med_us':>13} {'|resid|max_us':>13} {'drift_ppm':>9} {'pps_late_max':>12}")
    for nid in sorted({r["node"] for r in ts}):
        rs = [r for r in ts if r["node"] == nid]
        rtt = [r["rtt_us"] for r in rs]
        res = [abs(r["resid_us"]) for r in rs]
        print(f"{nid:>4} {len(rs):>4} {min(rtt):>10} {statistics.median(rtt):>10.0f} {statistics.median(res):>13.0f} "
              f"{max(res):>13} {rs[-1]['drift_ppm']:>9.2f} {rs[-1].get('pps_late_max_us', ''):>12}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

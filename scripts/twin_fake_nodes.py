#!/usr/bin/env python3
"""実機の代わりに、dyne-sim2d の出力をノードごとの「形の区画のメッセージ」(dyne/node/<id>/shape)として
MQTT に流す(実機なしで常駐版 scripts/twin_live.py を通しで試すため)。2026-11-13。

何をするか:
  dyne-sim2d の --field-csv(全体の格子の φ・d・w)と、同じ実行の記録(標準出力。node 行に各ノードの区画
  [i0..i1)x[j0..j1) がある)を読み、区画ごとに切り出して scripts/dyne_shape_msg.py の形式で送る。
  --frame を複数渡すと、--frame-seconds ごとに次の場面へ進む(箱を動かす実験6の再現。最後まで行ったら
  --loop で最初へ戻る)。各ノードは --period 秒ごとに自分の区画を送る(ノードごとに少しずらす)。
  計算はしない(ノードが計算した結果の再生)。

使い方(リポジトリの .venv で。mosquitto が 127.0.0.1:1883 で動いていること):
  .venv\\Scripts\\python.exe scripts\\twin_fake_nodes.py --broker 127.0.0.1 ^
      --frame demo_out\\pc_pipeline\\move\\move_280.csv:demo_out\\pc_pipeline\\move\\move_280.log ^
      --frame demo_out\\pc_pipeline\\move\\move_600.csv:demo_out\\pc_pipeline\\move\\move_600.log ^
      --frame-seconds 20 --period 1 --loop
"""
import argparse
import csv
import os
import re
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dyne_shape_msg as msg  # noqa: E402

NODE_RE = re.compile(r"^node\s+(\d+): \[(\d+)\.\.(\d+)\)x\[(\d+)\.\.(\d+)\)")
STEPS_RE = re.compile(r"steps (\d+)")


def read_frame(spec, px, py):
    """'field.csv[:sim.log]' を読む。戻り値 dict(phi, d, w, x0, y0, h, nodes=[(id, i0, i1, j0, j1)], step)"""
    if ":" in spec[2:]:
        k = spec.index(":", 2)
        csv_path, log_path = spec[:k], spec[k + 1:]
    else:
        csv_path, log_path = spec, None
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    gx = np.array([int(r["gx"]) for r in rows])
    gy = np.array([int(r["gy"]) for r in rows])
    nx, ny = gx.max() + 1, gy.max() + 1
    out = {}
    for key in ("phi", "d", "w", "x", "y"):
        a = np.zeros((ny, nx))
        a[gy, gx] = [float(r[key]) for r in rows]
        out[key] = a
    out["x0"], out["y0"] = float(out["x"][0, 0]), float(out["y"][0, 0])
    out["h"] = float(out["x"][0, 1] - out["x"][0, 0])
    nodes, step = [], 0
    if log_path and os.path.exists(log_path):
        with open(log_path, encoding="utf-8", errors="replace") as f:
            for line in f:
                m = NODE_RE.match(line)
                if m:
                    nodes.append(tuple(int(m.group(i)) for i in range(1, 6)))
                elif line.startswith("dyne-sim2d:"):
                    s = STEPS_RE.search(line)
                    step = int(s.group(1)) - 1 if s else 0
    if not nodes:
        # 記録が無ければ等分(dyne-sim2d と同じ分け方とは限らない)
        xs = np.linspace(0, nx, px + 1).round().astype(int)
        ys = np.linspace(0, ny, py + 1).round().astype(int)
        nodes = [(jy * px + ix, xs[ix], xs[ix + 1], ys[jy], ys[jy + 1]) for jy in range(py) for ix in range(px)]
    out["nodes"] = nodes
    out["step"] = step
    out["path"] = csv_path
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--frame", action="append", required=True, help="field.csv[:sim.log]。複数可(順に再生)")
    ap.add_argument("--frame-seconds", type=float, default=20.0, help="1 つの場面を流す時間 [秒]")
    ap.add_argument("--period", type=float, default=1.0, help="各ノードが区画を送る間隔 [秒]")
    ap.add_argument("--px", type=int, default=4, help="記録が無いときの x 方向のノード数")
    ap.add_argument("--py", type=int, default=4)
    ap.add_argument("--loop", action="store_true", help="最後の場面の後、最初へ戻る")
    ap.add_argument("--duration", type=float, default=0.0, help="この秒数で終える(0 = 場面を流し終えるまで / --loop なら無限)")
    ap.add_argument("--drop-node", type=int, action="append", default=[], help="このノードは送らない(欠けたノードの試験)")
    a = ap.parse_args()
    import paho.mqtt.client as mqtt

    frames = [read_frame(s, a.px, a.py) for s in a.frame]
    for fr in frames:
        print("場面 %s: 格子 %d×%d、刻み %.3f m、ノード %d 台、ステップ %d"
              % (fr["path"], fr["phi"].shape[1], fr["phi"].shape[0], fr["h"], len(fr["nodes"]), fr["step"]), flush=True)
    cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="dyne-fake-shape-%d" % os.getpid())
    cl.connect(a.broker, a.port, 60)
    cl.loop_start()
    t0 = time.time()
    sent, nbytes = 0, 0
    last_fi = -1
    try:
        while True:
            el = time.time() - t0
            if a.duration > 0 and el >= a.duration:
                break
            fi = int(el // a.frame_seconds)
            if fi >= len(frames):
                if not a.loop and a.duration <= 0:
                    break
                fi %= len(frames)
            fr = frames[fi]
            if fi != last_fi:
                print("[%6.1f s] 場面 %d: %s" % (el, fi, fr["path"]), flush=True)
                last_fi = fi
            gny, gnx = fr["phi"].shape
            n = len(fr["nodes"])
            for k, (nid, i0, i1, j0, j1) in enumerate(fr["nodes"]):
                if nid in a.drop_node:
                    continue
                # ノードごとに送る時刻をずらす(実機では各ノードが自分の周期で送る)
                time.sleep(a.period / max(n, 1))
                b = msg.pack(nid, fr["step"], int(time.time() * 1e6), i0, j0,
                             fr["phi"][j0:j1, i0:i1], fr["d"][j0:j1, i0:i1], fr["w"][j0:j1, i0:i1],
                             gnx=gnx, gny=gny, h=fr["h"], x0=fr["x0"], y0=fr["y0"])
                cl.publish(msg.topic(nid), b, qos=0)
                sent += 1
                nbytes += len(b)
    except KeyboardInterrupt:
        pass
    cl.loop_stop()
    cl.disconnect()
    print("送信 %d 通 / %.1f kB(%.1f 秒)" % (sent, nbytes / 1e3, time.time() - t0))


if __name__ == "__main__":
    main()

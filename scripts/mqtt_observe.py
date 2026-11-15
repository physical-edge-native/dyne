#!/usr/bin/env python3
"""MQTT制御面のワイルドカード購読 + CSV記録 (発注7、2026-09-14)。

規約: docs/2026-09-14-mqtt-control-plane.md §4。
dyne/# を購読して全メッセージを時刻つきで表示し、観測系トピックを種類別の
CSVへ追記する (温度場・センサ・RSSI・リアルタイム率・界面残差・状態遷移)。

使い方:
  python scripts/mqtt_observe.py --broker localhost [--out mqtt_log]
必要パッケージ: paho-mqtt (リポジトリの .venv に入れてある: .venv/Scripts/python.exe -m pip install -r scripts/requirements-pc.txt)
"""
import argparse
import csv
import json
import os
import time

import paho.mqtt.client as mqtt

# 状態変化の強調表示 (2026-09-18、実機確認⑥のフィードバック):
# 5ノード構成では sensor/rssi/field の流量に status が埋もれるため、
# 状態行を色+行頭マーカーで目立たせる。Windows のVT有効化は os.system("")。
RESET = "\033[0m"
STATE_STYLE = {
    "offline": "\033[1;31m",     # 赤 (離脱・LWT)
    "err": "\033[1;31m",
    "online": "\033[1;32m",      # 緑 (参加)
    "configured": "\033[32m",
    "done": "\033[1;36m",        # シアン (完走)
    "running": "\033[2m",        # 淡色 (高頻度)
}


class Logger:
    def __init__(self, outdir):
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.files = {}

    def row(self, name, header, row):
        if name not in self.files:
            f = open(os.path.join(self.outdir, name + ".csv"), "a", newline="", encoding="utf-8")
            w = csv.writer(f)
            if f.tell() == 0:
                w.writerow(header)
            self.files[name] = (f, w)
        f, w = self.files[name]
        w.writerow(row)
        f.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default="localhost")
    ap.add_argument("--topic", default="dyne/#")
    ap.add_argument("--out", default="mqtt_log", help="CSV出力ディレクトリ")
    ap.add_argument("--quiet-obs", action="store_true",
                    help="観測系 (field/sensor/rssi/rt/residual/innov) は CSV のみに"
                         "記録し、画面には status/ack 等の制御系だけを表示する"
                         " (多ノード時の状態監視用)")
    a = ap.parse_args()
    log = Logger(a.out)
    t0 = time.time()
    os.system("")  # Windows端末のANSIカラー有効化

    def on_connect(cl, u, flags, rc, props=None):
        print(f"connected rc={rc}; subscribing {a.topic}")
        cl.subscribe(a.topic, qos=1)

    def on_message(cl, u, msg):
        ts = time.time() - t0
        parts = msg.topic.split("/")
        try:
            d = json.loads(msg.payload)
        except Exception:
            print(f"[{ts:9.3f}] {msg.topic} (non-json {len(msg.payload)}B)")
            return
        # 表示の振り分け: 制御系 (status/ack/cfg/ctl) は常に表示し、状態行は
        # 色+マーカーで強調。観測系は --quiet-obs のとき画面に出さない (CSVのみ)
        is_status = len(parts) == 4 and parts[1] == "node" and parts[3] == "status"
        is_ctl = parts[1] in ("cfg", "ctl")
        line = f"[{ts:9.3f}] {msg.topic} {msg.payload[:160].decode(errors='replace')}"
        if is_status:
            style = STATE_STYLE.get(d.get("state"), "")
            print(f"{style}>>> {line}{RESET if style else ''}")
        elif is_ctl:
            ok = d.get("ok")
            style = "\033[1;31m" if ok is False else ("\033[32m" if ok is True else "")
            print(f"{style}>>> {line}{RESET if style else ''}")
        elif not a.quiet_obs:
            print(line)
        # 観測系をCSVへ (規約 §2.4〜2.9)
        if len(parts) == 4 and parts[1] == "node":
            nid, kind = parts[2], parts[3]
            if kind == "field":
                log.row("field", ["ts", "node", "step", "t", "i", "x0", "x1", "n", "u"],
                        [ts, nid, d.get("step"), d.get("t"), "", d.get("x0"),
                         d.get("x1"), d.get("n"), " ".join(map(str, d.get("u", [])))])
            elif kind == "sensor":
                log.row("sensor", ["ts", "node", "t_ms", "kind", "temp_c", "rh",
                                   "pres_hpa", "err"],
                        [ts, nid, d.get("t_ms"), d.get("kind"), d.get("temp_c"),
                         d.get("rh"), d.get("pres_hpa"), d.get("err")])
            elif kind == "rssi":
                for p in d.get("peers", []):
                    log.row("rssi", ["ts", "node", "t_ms", "ap_rssi", "mac", "rssi", "n"],
                            [ts, nid, d.get("t_ms"), d.get("ap_rssi"),
                             p.get("mac"), p.get("rssi"), p.get("n")])
                if not d.get("peers"):
                    log.row("rssi", ["ts", "node", "t_ms", "ap_rssi", "mac", "rssi", "n"],
                            [ts, nid, d.get("t_ms"), d.get("ap_rssi"), "", "", ""])
            elif kind == "rt":
                sm, cm = d.get("step_ms", {}), d.get("compute_ms", {})
                log.row("rt", ["ts", "node", "step", "window", "step_ms_mean",
                               "step_ms_min", "step_ms_max", "compute_ms_mean",
                               "dt_ms", "rt_rate"],
                        [ts, nid, d.get("step"), d.get("window"), sm.get("mean"),
                         sm.get("min"), sm.get("max"), cm.get("mean"),
                         d.get("dt_ms"), d.get("rt_rate")])
            elif kind == "innov":
                log.row("innov", ["ts", "node", "step", "i", "y", "d", "y_t_ms"],
                        [ts, nid, d.get("step"), d.get("i"), d.get("y"),
                         d.get("d"), d.get("y_t_ms")])
            elif kind == "status":
                log.row("status", ["ts", "node", "state", "detail"],
                        [ts, nid, d.get("state"), json.dumps(d, ensure_ascii=False)])
        elif len(parts) == 4 and parts[1] == "iface" and parts[3] == "residual":
            log.row("residual", ["ts", "iface", "step", "residual"],
                    [ts, parts[2], d.get("step"), d.get("residual")])

    cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    cl.on_connect = on_connect
    cl.on_message = on_message
    cl.connect(a.broker, 1883, 60)
    print(f"logging to {a.out}/ (Ctrl+C で終了)")
    cl.loop_forever()


if __name__ == "__main__":
    main()

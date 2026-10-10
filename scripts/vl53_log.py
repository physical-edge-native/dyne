#!/usr/bin/env python3
"""VL53L5CX 評価の記録ツール (2026-10-10)。

評価計画: docs/2026-10-10-vl53l5cx-eval-plan.md。相手のファーム: vl53-eval/。
USBシリアルで設定コマンドを送り、Nフレーム測って1条件=1つのCSVに保存する。
あわせて同名の .meta.txt にファームの起動情報・設定・メモを残し、
最後に中央4区画の平均・標準偏差と有効率を表示する(その場の確認用)。

使い方(例: E1 白壁1.0m、8x8・10Hzで300フレーム):
  python scripts/vl53_log.py --port COM7 --out measured/2026-10-xx-vl53l5cx ^
      --label E1_white_1000mm --frames 300 --set "res 8" --set "hz 10" ^
      --note "白壁に正対、メジャー1000mm(センサ前面から)"
必要パッケージ: pyserial (pip install pyserial)

CSVの列: pc_time(PCの時刻、秒), t_ms(ESP32起動からのms), idx, nz, temp_c(センサ内部温度),
  d0..(距離mm), st0..(状態値。5と9が有効), sg0..(ばらつき推定mm),
  sp0..(信号の強さ kcps/SPAD), nt0..(検出物体数), am0..(環境光 kcps/SPAD)
"""
import argparse
import csv
import os
import statistics
import sys
import time

import serial

BAUD = 921600
FIELDS = ["d", "st", "sg", "sp", "nt", "am"]
VALID = {5, 9}


def center_zones(nz):
    """中央4区画の番号 (8x8: 27,28,35,36 / 4x4: 5,6,9,10)"""
    w = 8 if nz == 64 else 4
    c = w // 2
    return [(c - 1) * w + c - 1, (c - 1) * w + c, c * w + c - 1, c * w + c]


class Link:
    def __init__(self, port):
        s = serial.Serial()
        s.port = port
        s.baudrate = BAUD
        s.timeout = 0.2
        # 開くときの自動リセットを避ける (ボードによっては効かない。その場合は起動を待つ)
        s.dtr = False
        s.rts = False
        s.open()
        self.s = s
        self.meta = []

    def readline(self):
        b = self.s.readline()
        if not b:
            return None
        return b.decode("utf-8", errors="replace").rstrip("\r\n")

    def send(self, cmd):
        self.s.write((cmd + "\n").encode())
        self.s.flush()

    def wait_ready(self, timeout=20.0):
        """#CFG が来るまで info を送り続ける (起動直後ならセンサの初期化を待つ)"""
        t_end = time.time() + timeout
        t_next = 0.0
        while time.time() < t_end:
            if time.time() >= t_next:
                self.send("info")
                t_next = time.time() + 2.0
            ln = self.readline()
            if ln is None:
                continue
            if ln.startswith("#"):
                print(ln)
                self.meta.append(ln)
                if ln.startswith("#ERR not_alive") or ln.startswith("#ERR init"):
                    sys.exit("センサが応答しません(配線・電源を確認)")
                if ln.startswith("#CFG"):
                    return
        sys.exit("ファームから応答がありません(ポート・書き込み済みかを確認)")

    def command(self, cmd, timeout=5.0):
        self.send(cmd)
        t_end = time.time() + timeout
        while time.time() < t_end:
            ln = self.readline()
            if ln is None or not ln.startswith("#"):
                continue
            print(ln)
            self.meta.append(ln)
            if ln.startswith("#OK " + cmd):
                return True
            if ln.startswith("#ERR " + cmd):
                return False
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", required=True, help="保存先フォルダ (例: measured/2026-10-xx-vl53l5cx)")
    ap.add_argument("--label", required=True, help="条件名 (ファイル名になる。例: E1_white_1000mm)")
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--set", action="append", default=[], help='測定前に送る設定コマンド (例: "res 8")。複数可')
    ap.add_argument("--note", default="", help="条件のメモ (メジャーの値・対象の材質など)")
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, a.label + ".csv")
    if os.path.exists(path):
        sys.exit(f"{path} は既にあります(上書きしない。--label を変えてください)")

    ln = Link(a.port)
    ln.wait_ready()
    for c in a.set:
        if not ln.command(c):
            sys.exit(f"設定コマンドが失敗しました: {c}")

    started = time.strftime("%Y-%m-%d %H:%M:%S")
    rows = []
    nz = None
    ln.send(f"start {a.frames}")
    t_end = time.time() + a.frames / 1.0 + 30  # 1Hzでも間に合う上限
    while time.time() < t_end:
        line = ln.readline()
        if line is None:
            continue
        if line.startswith("#"):
            print(line)
            ln.meta.append(line)
            if line.startswith("#DONE") or line.startswith("#ERR start"):
                break
            continue
        if not line.startswith("F,"):
            continue
        parts = line.split(",")
        try:
            vals = [int(x) for x in parts[1:]]
        except ValueError:
            continue  # 取りこぼしで壊れた行は捨てる (件数は idx の欠番で分かる)
        t_ms, idx, n, temp = vals[:4]
        if len(vals) != 4 + 6 * n:
            continue
        nz = n
        rows.append([f"{time.time():.3f}", t_ms, idx, n, temp] + vals[4:])
        if idx % 50 == 0:
            print(f"  frame {idx}")
    if not rows:
        sys.exit("フレームが1つも取れませんでした")

    header = ["pc_time", "t_ms", "idx", "nz", "temp_c"] + [f"{f}{i}" for f in FIELDS for i in range(nz)]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    with open(os.path.join(a.out, a.label + ".meta.txt"), "w", encoding="utf-8") as f:
        f.write(f"label: {a.label}\nstarted: {started}\nport: {a.port}\nframes_requested: {a.frames}\n")
        f.write(f"frames_saved: {len(rows)}\nset: {a.set}\nnote: {a.note}\n--- firmware ---\n")
        f.write("\n".join(ln.meta) + "\n")

    # その場の確認: 中央4区画
    idxs = [r[2] for r in rows]
    lost = (max(idxs) - min(idxs) + 1) - len(set(idxs))
    print(f"保存: {path}  ({len(rows)}フレーム、欠番{lost})")
    for z in center_zones(nz):
        d = [r[5 + z] for r in rows]
        st = [r[5 + nz + z] for r in rows]
        ok = [di for di, si in zip(d, st) if si in VALID]
        rate = len(ok) / len(d)
        if len(ok) >= 2:
            print(f"  区画{z:2d}: 平均 {statistics.mean(ok):7.1f} mm  標準偏差 {statistics.stdev(ok):5.1f} mm  有効率 {rate:.0%}")
        else:
            print(f"  区画{z:2d}: 有効な値なし (有効率 {rate:.0%})")


if __name__ == "__main__":
    main()

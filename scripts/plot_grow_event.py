#!/usr/bin/env python3
"""GROWイベント1回の界面残差の「無傷さ」を注記入りPNGにする (スライド用、2026-08-30)。
scripts/plot_add_event.py と同じ様式 (対数軸、配色、ベースライン点線、操作の瞬間の縦線、
図の寸法) で描き、ADDのグラフと並べて対比する。

使い方:
  python scripts/plot_grow_event.py --residual resid.csv --event ev.csv --log sim.log --out grow_event.png
      [--base 100] [--xmin 300 --xmax 700] [--ymin 0.0137 --ymax 36]

注記は2つだけ:
  ① 操作の瞬間: event CSV の resize 行 (step, node, 新しい幅)。旧幅は sim.log の resize 行から取る
  ② 界面残差は変化なし: sim.log の「model output preserved N/M pts bit-exact」を引用
外部依存: matplotlib のみ。
"""
import argparse
import csv
import re
import statistics
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

for cand in ["BIZ UDPGothic", "Meiryo", "Yu Gothic", "MS Gothic", "HGPGothicM"]:
    if any(f.name == cand for f in matplotlib.font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = cand
        break
plt.rcParams["axes.unicode_minus"] = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--residual", required=True)
    ap.add_argument("--event", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--base", type=int, default=100)
    ap.add_argument("--width-old", type=int, default=8, help="GROW前の幅 (--pinn width=... と同じ値)")
    ap.add_argument("--xmin", type=int, default=None)
    ap.add_argument("--xmax", type=int, default=None)
    ap.add_argument("--ymin", type=float, default=None)
    ap.add_argument("--ymax", type=float, default=None)
    ap.add_argument("--title", default="隠れ層の拡張(GROW)1回に対する界面残差 — 乱れなし")
    a = ap.parse_args()

    series = defaultdict(list)
    with open(a.residual, newline="") as f:
        for r in csv.DictReader(f):
            series[int(r["iface"])].append((int(r["step"]), float(r["residual"])))
    evs = [r for r in csv.DictReader(open(a.event, newline="")) if r["kind"] == "resize"]
    assert len(evs) == 1, f"resize イベントは1回のはず: {len(evs)}"
    t_op, node, w_new = int(evs[0]["step"]), int(evs[0]["node"]), int(evs[0]["width"])

    log = open(a.log, encoding="utf-8", errors="replace").read()
    m = re.search(r"width -> (\d+) at t=\d+ms \(step (\d+)\), model output preserved (\d+)/(\d+) pts bit-exact", log)
    assert m, "sim.log に resize applied 行がない"
    preserved, total = int(m.group(3)), int(m.group(4))
    w_old = a.width_old

    adj = [k for k in series if k in (node - 1, node)]  # ノードnodeを挟む界面 (node-1, node)
    base = statistics.median(r for k in series for s, r in series[k] if t_op - a.base <= s < t_op)
    # 操作前後の隣接界面の変化量 (無傷さの数値根拠)
    jumps = {}
    for k in adj:
        v = dict(series[k])
        if t_op - 1 in v and t_op + 1 in v:
            jumps[k] = max(v[t_op] / v[t_op - 1], v[t_op + 1] / v[t_op]) if v[t_op - 1] > 0 and v[t_op] > 0 else float("nan")

    fig, ax = plt.subplots(figsize=(11, 5.6), dpi=150)
    for k in sorted(series):
        s, r = zip(*series[k])
        if k in adj:
            col = "#d62728" if k == max(adj) else "#ff7f0e"
            ax.plot(s, r, color=col, lw=1.6, zorder=5, label=f"界面 {k}(GROWしたノード{node}に接する)")
        else:
            ax.plot(s, r, color="#999", lw=0.8, alpha=0.7, label="他の界面" if k == min(x for x in series if x not in adj) else None)
    ax.axhline(base, color="#2ca02c", ls=":", lw=1.2, label=f"ベースライン {base:.3g}(操作前{a.base}ステップの全界面の中央値)")
    ax.set_yscale("log")
    ax.set_xlim(a.xmin if a.xmin is not None else t_op - 100, a.xmax if a.xmax is not None else t_op + 300)
    ax.set_ylim(a.ymin if a.ymin is not None else base / 10, a.ymax if a.ymax is not None else base * 260)
    ax.set_xlabel("ステップ")
    ax.set_ylabel("界面残差 |∂u/∂x(左) − ∂u/∂x(右)|(対数)")
    ax.set_title(a.title)
    ax.grid(alpha=0.25, which="both")
    ymin, ymax = ax.get_ylim()

    ax.axvline(t_op, color="k", lw=1.2)
    ax.annotate(f"① 操作の瞬間\nstep {t_op}: ノード{node}がGROW(幅{w_old}→{w_new})",
                xy=(t_op, base), xytext=(t_op - 15, ymax * 0.25), ha="right", fontsize=10,
                bbox=dict(boxstyle="round", fc="white", ec="k"), arrowprops=dict(arrowstyle="->", color="k"))
    ax.annotate(f"② 界面残差は変化なし\n(操作前後で予測値 {preserved}/{total} 点ビット一致)",
                xy=(t_op + 1, dict(series[max(adj)]).get(t_op + 1, base)), xytext=(t_op + 40, ymax * 0.25),
                ha="left", fontsize=10, bbox=dict(boxstyle="round", fc="white", ec="#d62728"),
                arrowprops=dict(arrowstyle="->", color="#d62728"))
    ax.legend(loc="upper right", fontsize=8.5, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(a.out)
    print(f"written: {a.out}")
    print(f"resize step={t_op} node={node} width {w_old}->{w_new} preserved={preserved}/{total} baseline={base:.4g}")
    for k, j in jumps.items():
        print(f"  iface {k}: max step-to-step ratio across the operation = {j:.4f}")


if __name__ == "__main__":
    main()

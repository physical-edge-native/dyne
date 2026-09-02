#!/usr/bin/env python3
"""ADDイベント1回の界面残差スパイクと回復を注記入りPNGにする (スライド用、2026-08-30)。

使い方:
  python scripts/plot_add_event.py --residual resid.csv --event ev.csv --out add_event.png
      [--base 100] [--recover-ratio 2] [--xmax 700]

定義 (dyne-sim のログ「join: ... 乱れピーク / 回復時間」と同じ):
  ベースライン = 操作直前 base ステップにおける既存界面の残差の中央値
  ①操作の瞬間 = event CSV の join 行の step
  ②乱れピーク = 操作で新設された界面の、操作後の残差の最大値
  ③回復時間  = ピーク後、残差がベースライン×recover-ratio を初めて下回るまでのステップ数
               (併記: 判定線を最後に超えた時刻=完全に下回るまでのステップ数)
外部依存: matplotlib のみ。
"""
import argparse
import csv
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
    ap.add_argument("--out", required=True)
    ap.add_argument("--base", type=int, default=100)
    ap.add_argument("--recover-ratio", type=float, default=2.0)
    ap.add_argument("--xmin", type=int, default=None, help="横軸の左端 (既定: 参加の100ステップ前)")
    ap.add_argument("--xmax", type=int, default=None, help="横軸の右端 (既定: 参加の300ステップ後)")
    ap.add_argument("--ymin", type=float, default=None, help="縦軸の下端 (既定: ベースライン/10)")
    ap.add_argument("--title", default="ノード参加(ADD)1回に対する界面残差の乱れと回復")
    a = ap.parse_args()

    series = defaultdict(list)
    with open(a.residual, newline="") as f:
        for r in csv.DictReader(f):
            series[int(r["iface"])].append((int(r["step"]), float(r["residual"])))
    joins = [r for r in csv.DictReader(open(a.event, newline="")) if r["kind"] == "join"]
    assert joins, "event CSV に join 行がない"
    t_add, node_add = int(joins[0]["step"]), int(joins[0]["node"])

    old = sorted(k for k, v in series.items() if v[0][0] < t_add)
    new = sorted(k for k, v in series.items() if v[0][0] >= t_add)
    base = statistics.median(r for k in old for s, r in series[k] if t_add - a.base <= s < t_add)
    line = a.recover_ratio * base

    stats = {}
    for k in new:
        seg = [(s, r) for s, r in series[k] if s >= t_add]
        s_pk, v_pk = max(seg, key=lambda x: x[1])
        s_rec = next((s for s, r in seg if s >= s_pk and r < line), None)
        above = [s for s, r in seg if r >= line]
        s_last = (above[-1] + 1) if above else None  # 判定線を最後に超えた次のステップ = 完全回復
        stats[k] = (s_pk, v_pk, s_rec, s_last)
    tgt = max(stats, key=lambda k: stats[k][1])
    s_pk, v_pk, s_rec, s_last = stats[tgt]

    fig, ax = plt.subplots(figsize=(11, 5.6), dpi=150)
    for k in old:
        s, r = zip(*series[k])
        ax.plot(s, r, color="#999", lw=0.8, alpha=0.7, label="既存の界面(参加前から存在)" if k == old[0] else None)
    for k in new:
        s, r = zip(*series[k])
        ax.plot(s, r, color="#d62728" if k == tgt else "#ff7f0e", lw=1.6 if k == tgt else 1.2,
                label=f"界面 {k}(参加で新設{'、注記対象' if k == tgt else ''})", zorder=5)
    ax.axhline(base, color="#2ca02c", ls=":", lw=1.2, label=f"ベースライン {base:.3g}(参加前{a.base}ステップの既存界面の中央値)")
    ax.axhline(line, color="#2ca02c", ls="-.", lw=1.0, alpha=0.9, label=f"回復判定線(ベースライン×{a.recover_ratio:g})")
    ax.set_yscale("log")
    x0 = a.xmin if a.xmin is not None else t_add - 100
    x1 = a.xmax if a.xmax is not None else min(t_add + 300, series[old[0]][-1][0])
    ax.set_xlim(x0, x1)
    ax.set_ylim(a.ymin if a.ymin is not None else base / 10, v_pk * 4)
    ax.set_xlabel("ステップ")
    ax.set_ylabel("界面残差 |∂u/∂x(左) − ∂u/∂x(右)|(対数)")
    ax.set_title(a.title)
    ax.grid(alpha=0.25, which="both")
    ymin, ymax = ax.get_ylim()

    # ① 操作の瞬間
    ax.axvline(t_add, color="k", lw=1.2)
    ax.annotate(f"① 操作の瞬間\nstep {t_add}: ノード{node_add}が参加\n(界面 {new[0]}, {new[-1]} が新設)",
                xy=(t_add, base), xytext=(t_add - 15, v_pk * 0.5), ha="right", fontsize=10,
                bbox=dict(boxstyle="round", fc="white", ec="k"), arrowprops=dict(arrowstyle="->", color="k"))
    # ② 乱れピーク
    ax.plot([s_pk], [v_pk], "o", color="#d62728", ms=8, zorder=6)
    ax.annotate(f"② 乱れピーク\nstep {s_pk}(参加の {s_pk - t_add} ステップ後)\n残差 {v_pk:.3g} = ベースラインの {v_pk / base:.0f} 倍",
                xy=(s_pk, v_pk), xytext=(s_pk + 25, v_pk * 1.1), fontsize=10, va="top",
                bbox=dict(boxstyle="round", fc="white", ec="#d62728"), arrowprops=dict(arrowstyle="->", color="#d62728"))
    # ③ 回復時間
    if s_rec is not None:
        ax.axvline(s_rec, color="#2ca02c", lw=1.2)
        y_arrow = line * 1.6
        ax.annotate("", xy=(s_rec, y_arrow), xytext=(t_add, y_arrow),
                    arrowprops=dict(arrowstyle="<->", color="#2ca02c", lw=1.6))
        others = ", ".join(f"界面 {k}: {stats[k][2] - t_add if stats[k][2] is not None else '未'} ステップ" for k in new if k != tgt)
        if s_last is not None and s_last > s_rec:
            full = f"以後 step {s_last - 1} まで判定線を出入りし、完全に下回るのは参加後 {s_last - t_add} ステップ(破線)"
            ax.axvline(s_last, color="#2ca02c", lw=0.9, ls="--")
        else:
            full = "以後は判定線を再び超えない"
        ax.annotate(f"③ 回復時間 {s_rec - t_add} ステップ\n(step {s_rec} で残差が回復判定線を初めて下回る。{others})\n{full}",
                    xy=((t_add + s_rec) / 2, y_arrow), xytext=(s_rec + 40, base / 4), fontsize=10, ha="left",
                    bbox=dict(boxstyle="round", fc="white", ec="#2ca02c"), arrowprops=dict(arrowstyle="->", color="#2ca02c"))
    else:
        ax.text(t_add + 10, line * 1.6, "③ 回復: 実行終了までに判定線を下回らず", color="#2ca02c", fontsize=10)
    ax.legend(loc="upper right", fontsize=8.5, framealpha=0.9)
    # 拡大枠: 参加前後の数十ステップ
    ins = ax.inset_axes([0.60, 0.40, 0.37, 0.30])
    for k in old:
        sv = [(s, r) for s, r in series[k] if t_add - 5 <= s <= t_add + 40]
        ins.plot(*zip(*sv), color="#999", lw=0.8, alpha=0.7)
    for k in new:
        sv = [(s, r) for s, r in series[k] if s <= t_add + 40]
        ins.plot(*zip(*sv), color="#d62728" if k == tgt else "#ff7f0e", lw=1.4)
    ins.axhline(line, color="#2ca02c", ls="-.", lw=0.9)
    ins.axvline(t_add, color="k", lw=0.9)
    if s_rec is not None:
        ins.axvline(s_rec, color="#2ca02c", lw=0.9)
    ins.set_yscale("log"); ins.set_xlim(t_add - 5, t_add + 40); ins.set_ylim(base / 5, v_pk * 2)
    ins.tick_params(labelsize=7); ins.set_title(f"拡大: step {t_add - 5}〜{t_add + 40}", fontsize=8)
    ins.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(a.out)
    print(f"written: {a.out}")
    print(f"add_step={t_add} node={node_add} baseline={base:.4g} line={line:.4g}")
    for k in new:
        p_s, p_v, r_s, l_s = stats[k]
        print(f"  iface {k}: peak={p_v:.4g}@{p_s} ({p_v / base:.0f}x) recover={r_s - t_add if r_s is not None else None} steps, "
              f"fully below line after {l_s - t_add if l_s is not None else None} steps")


if __name__ == "__main__":
    main()

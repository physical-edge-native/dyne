#!/usr/bin/env python3
"""M-S1b レポート (docs/2026-08-25-ms1b-report.md §5.1) の精度表を棒グラフにする (スライド用、2026-08-30)。
横軸=分割数 (ノード数 1/2/4/8/16)、縦軸=厳密解に対する最大誤差 (対数軸)。
差分法アクター (16ノード) の誤差 8.0e-5 を参照水準の横線として入れる。
数値はレポートの表からの転記 (2000ステップ、界面条件 iface=1、格子点 65)。

使い方: python scripts/plot_ms1b_accuracy.py --out ms1b_accuracy.png
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

for cand in ["BIZ UDPGothic", "Meiryo", "Yu Gothic", "MS Gothic", "HGPGothicM"]:
    if any(f.name == cand for f in matplotlib.font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = cand
        break
plt.rcParams["axes.unicode_minus"] = False

# (ノード数, 1ノードの格子点数, 最大誤差 vs 厳密解)  — docs/2026-08-25-ms1b-report.md 表5.1
ROWS = [(1, "65", 2.5e-4), (2, "32〜33", 3.3e-4), (4, "16〜17", 6.1e-5), (8, "8〜9", 8.4e-5), (16, "4〜5", 2.1e-2)]
FDM_REF = 8.0e-5  # 差分法アクター、16ノード (回帰試験)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="分割数と精度(学習型アクター、2000ステップ)— 8分割までは差分法と同水準、16分割で崩れる")
    a = ap.parse_args()

    xs = list(range(len(ROWS)))
    ys = [r[2] for r in ROWS]
    cols = ["#1f77b4" if r[0] < 16 else "#d62728" for r in ROWS]

    fig, ax = plt.subplots(figsize=(11, 5.6), dpi=150)
    bars = ax.bar(xs, ys, color=cols, width=0.6, zorder=3)
    ax.axhline(FDM_REF, color="#2ca02c", ls=":", lw=1.8, zorder=4,
               label=f"差分法アクターの誤差 {FDM_REF:.1e}(16分割、参照水準)")
    ax.set_yscale("log")
    ax.set_ylim(1e-5, 1e-1)
    ax.set_xticks(xs)
    ax.set_xticklabels([f"{r[0]}分割\n(1ノード {r[1]} 点)" for r in ROWS])
    ax.set_xlabel("分割数(ノード数)")
    ax.set_ylabel("厳密解に対する最大誤差(対数)")
    ax.set_title(a.title, fontsize=12)
    ax.grid(axis="y", alpha=0.3, which="both", zorder=0)
    for b, y in zip(bars, ys):
        ax.text(b.get_x() + b.get_width() / 2, y * 1.25, f"{y:.1e}", ha="center", va="bottom", fontsize=10)
    ax.annotate("精度が100倍以上悪化\n(1ノードの格子点4〜5個では\n49パラメータのNNを決めるには情報不足)",
                xy=(xs[-1], ys[-1]), xytext=(xs[-1] - 1.6, 2e-2), ha="center", fontsize=10,
                bbox=dict(boxstyle="round", fc="white", ec="#d62728"), arrowprops=dict(arrowstyle="->", color="#d62728"))
    ax.legend(loc="upper left", fontsize=9.5, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(a.out)
    print(f"written: {a.out}")


if __name__ == "__main__":
    main()

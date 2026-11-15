#!/usr/bin/env python3
"""サーボの向きの校正用の印刷する分度器(A4 横、半径 135 mm の半円、0.5 度おきの目盛り、中心の十字)。2026-11-13。

出力: 同じフォルダの protractor_a4.pdf と protractor_a4.svg(「実際のサイズ」= 拡大縮小なしで印刷する)。
印刷後に下の「100 mm の物差し」を定規で測り、100 mm でなければ印刷の倍率が狂っている(角度の目盛りは
倍率が狂っても角度としては正しいが、半径の目安は狂う)。
使い方: .venv\\Scripts\\python.exe tools\\print\\protractor_a4.py
"""
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

W, H = 297.0, 210.0        # A4 横 [mm]
R = 135.0                  # 目盛りの外の半径 [mm]
CX, CY = W / 2, 22.0       # 中心(下の余白 22 mm)
MM = 1 / 25.4


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    fig = plt.figure(figsize=(W * MM, H * MM))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.axis("off")
    lw = 0.25
    # 外の弧と内の弧
    for r in (R, R - 14, 40):
        th = [math.radians(k / 10) for k in range(0, 1801)]
        ax.plot([CX + r * math.cos(t) for t in th], [CY + r * math.sin(t) for t in th], color="k", lw=lw)
    ax.plot([CX - R, CX + R], [CY, CY], color="k", lw=lw)
    # 目盛り(0.5 度おき)
    for k in range(0, 361):
        deg = k / 2
        t = math.radians(deg)
        if k % 20 == 0:
            L, w_ = 12.0, 0.35      # 10 度
        elif k % 10 == 0:
            L, w_ = 8.0, 0.3        # 5 度
        elif k % 2 == 0:
            L, w_ = 5.0, 0.2        # 1 度
        else:
            L, w_ = 3.0, 0.15       # 0.5 度
        c, s = math.cos(t), math.sin(t)
        ax.plot([CX + (R - L) * c, CX + R * c], [CY + (R - L) * s, CY + R * s], color="k", lw=w_)
        if k % 20 == 0:
            # 10 度おきの細い放射線(中心の近くは空ける)
            ax.plot([CX + 42 * c, CX + (R - 14) * c], [CY + 42 * s, CY + (R - 14) * s], color="0.6", lw=0.12)
            rot = deg - 90
            ax.text(CX + (R - 18) * c, CY + (R - 18) * s, "%d" % deg, ha="center", va="center", rotation=rot,
                    fontsize=7)
            ax.text(CX + (R - 25) * c, CY + (R - 25) * s, "%d" % (180 - deg), ha="center", va="center", rotation=rot,
                    fontsize=5.5, color="0.4")
    # 中心の印(十字と小さな円)
    ax.plot([CX - 8, CX + 8], [CY, CY], color="r", lw=0.2)
    ax.plot([CX, CX], [CY - 8, CY + 8], color="r", lw=0.2)
    th = [math.radians(k) for k in range(0, 361, 5)]
    ax.plot([CX + 1.0 * math.cos(t) for t in th], [CY + 1.0 * math.sin(t) for t in th], color="r", lw=0.2)
    # 100 mm の物差し(印刷の倍率の確認)
    y0 = 8.0
    ax.plot([15, 115], [y0, y0], color="k", lw=0.3)
    for k in range(0, 101, 10):
        ax.plot([15 + k, 15 + k], [y0, y0 + (3 if k % 50 else 5)], color="k", lw=0.25)
    ax.text(15, y0 + 6, "100 mm(印刷後に定規で確認)", fontsize=6, fontfamily=["Yu Gothic", "MS Gothic", "sans-serif"])
    ax.text(W - 15, y0, "Dyne サーボ校正用の分度器 / 半径 %.0f mm / 0.5 度おき / 中心 = 赤い十字\n"
            "「実際のサイズ」で印刷する(ページに合わせない)" % R,
            fontsize=6, ha="right", va="bottom", fontfamily=["Yu Gothic", "MS Gothic", "sans-serif"])
    for ext in ("pdf", "svg"):
        p = os.path.join(here, "protractor_a4." + ext)
        fig.savefig(p)
        print(p)


if __name__ == "__main__":
    main()

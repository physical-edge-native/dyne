"""実験1(2026-10-08)の図: fig11=温度の時間変化、fig12=保留ノードの予測と実測。

入力は measured/2026-10-08-exp1/ の実測CSV。出力は figs/ に .dat と .png。
実行: python figs/fig11_exp1.py(リポジトリ直下で)
"""
import csv
import datetime as dt
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
M = ROOT / "measured" / "2026-10-08-exp1"
OUT = ROOT / "figs"

# 監視ツールの起動時刻 (measured/README.md)
T0 = {
    "calib": dt.datetime(2026, 10, 8, 18, 37, 32),
    "run1": dt.datetime(2026, 10, 8, 19, 59, 11),
    "run2": dt.datetime(2026, 10, 8, 20, 33, 47),
    "run3": dt.datetime(2026, 10, 8, 21, 7, 50),
    "run4": dt.datetime(2026, 10, 8, 21, 41, 43),
}
RUN_START = {"run1": 19*60+59+21/60, "run2": 20*60+34, "run3": 21*60+7+58/60, "run4": 21*60+41+50/60}
POS = {0: 0.425, 1: 1.328, 2: 2.178, 3: 3.028}
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"

plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "MS Gothic", "sans-serif"]
plt.rcParams["axes.edgecolor"] = INK2
plt.rcParams["axes.labelcolor"] = INK
plt.rcParams["xtick.color"] = INK2
plt.rcParams["ytick.color"] = INK2


def minutes(t):
    return t.hour * 60 + t.minute + t.second / 60


def read_sensor(folder, ts_min=0.0, from_min=None):
    rows = []
    with open(M / folder / "sensor.csv", newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if not r["temp_c"]:
                continue
            ts = float(r["ts"])
            if ts < ts_min:
                continue
            m = minutes(T0[folder] + dt.timedelta(seconds=ts))
            if from_min is not None and m < from_min:
                continue
            rows.append((m, int(r["node"]), float(r["temp_c"])))
    return rows


# ---- fig10: 補正後温度の時間変化(配置後〜run4終了) ----
series = []
series += read_sensor("calib", from_min=19 * 60 + 53 + 15 / 60)  # 移動完了後
for run in ("run1", "run2", "run3", "run4"):
    series += read_sensor(run, ts_min=5)
series.sort()
with open(OUT / "fig11_exp1_temps.dat", "w", encoding="utf-8") as f:
    f.write("# minutes_since_midnight node temp_c(calibrated)  source: measured/2026-10-08-exp1\n")
    for m, n, t in series:
        f.write(f"{m:.4f} {n} {t:.2f}\n")

fig, ax = plt.subplots(figsize=(9, 4.6), dpi=150)
fig.patch.set_facecolor("#fcfcfb")
ax.set_facecolor("#fcfcfb")
for i, (run, s) in enumerate(RUN_START.items()):
    e = s + 33.4
    if i % 2 == 0:
        ax.axvspan(s, e, color="#f0efec", zorder=0, lw=0)
    ax.text((s + e) / 2, 24.27, f"{run}(保留 node {i})", ha="center", va="bottom",
            fontsize=8, color=INK2)
for n in range(4):
    xs = [m for m, k, _ in series if k == n]
    ys = [t for _, k, t in series if k == n]
    ax.plot(xs, ys, color=COLORS[n], lw=1.6, label=f"node {n}(窓から{POS[n]:.2f} m)")
    ax.text(xs[-1] + 1.0, ys[-1], f"node {n}", color=INK, fontsize=8, va="center")
ticks = list(range(20 * 60, 22 * 60 + 31, 30))
ax.set_xticks(ticks)
ax.set_xticklabels([f"{t // 60}:{t % 60:02d}" for t in ticks])
ax.set_xlim(19 * 60 + 53, 22 * 60 + 24)
ax.set_ylim(23.35, 24.35)
ax.set_ylabel("温度(校正後)[℃]")
ax.set_xlabel("時刻(2026-10-08)")
ax.grid(axis="y", color=GRID, lw=0.8)
for sp in ("top", "right"):
    ax.spines[sp].set_visible(False)
ax.legend(loc="lower left", fontsize=8, frameon=False, ncol=2)
ax.set_title("実験1: 4台のDHT20の温度(床置き、窓=0 m〜壁=3.4 m)", fontsize=10, color=INK, loc="left")
fig.tight_layout()
fig.savefig(OUT / "fig11_exp1_temps.png", facecolor=fig.get_facecolor())

# ---- fig11: 各runの保留ノード: 実測 / モデル予測 / 単純平均 ----
fig, axes = plt.subplots(2, 2, figsize=(9, 6), dpi=150, sharex=True)
fig.patch.set_facecolor("#fcfcfb")
lines = []
for k, (run, ax) in enumerate(zip(("run1", "run2", "run3", "run4"), axes.flat)):
    by = {}
    with open(M / run / "innov.csv", newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if float(r["ts"]) < 5:
                continue
            by.setdefault(int(r["step"]), {})[int(r["node"])] = (float(r["y"]), float(r["d"]))
    st, ya, ym, yn = [], [], [], []
    for s in sorted(by):
        v = by[s]
        if len(v) < 4:
            continue
        y = [v[n][0] for n in range(4)]
        st.append(s / 60)
        ya.append(y[k])
        ym.append(y[k] - v[k][1])
        yn.append(sum(y[n] for n in range(4) if n != k) / 3)
        lines.append(f"{run} {s} {y[k]:.4f} {y[k]-v[k][1]:.4f} {yn[-1]:.4f}")
    ax.set_facecolor("#fcfcfb")
    ax.axvspan(0, 200 / 60, color="#f0efec", lw=0, zorder=0)
    ax.plot(st, ya, color=INK, lw=1.6, label="実測(保留したセンサ)")
    ax.plot(st, ym, color=COLORS[0], lw=1.6, label="モデル予測")
    ax.plot(st, yn, color=COLORS[1], lw=1.6, ls="--", label="単純平均(他3台)")
    ax.set_title(f"{run}: 保留 node {k}(窓から{POS[k]:.2f} m)", fontsize=9, color=INK, loc="left")
    ax.grid(axis="y", color=GRID, lw=0.8)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.tick_params(labelsize=8)
for ax in axes[1]:
    ax.set_xlabel("経過時間 [分](1ステップ=1秒。灰色=集計から除いた序盤200ステップ)", fontsize=8)
for ax in axes[:, 0]:
    ax.set_ylabel("温度 [℃]", fontsize=8)
axes[0, 0].legend(fontsize=7, frameon=False, loc="upper right")
fig.suptitle("実験1: 保留ノード位置での実測・モデル予測・単純平均", fontsize=10, color=INK, x=0.01, ha="left")
fig.tight_layout()
fig.savefig(OUT / "fig12_exp1_holdout.png", facecolor=fig.get_facecolor())
with open(OUT / "fig12_exp1_holdout.dat", "w", encoding="utf-8") as f:
    f.write("# run step y_holdout model_pred naive_mean  source: measured/2026-10-08-exp1/*/innov.csv\n")
    f.write("\n".join(lines) + "\n")
print("ok")

"""形の変化への追従の掃引 (2026-11-11、docs/2026-11-11-dynamic-shape.md)。

dyne-sim2d と格子型アクター (sdf2d-grid-actor) を、動く箱のある場面ファイル
(crates/dyne-sim2d/scenes/room6_move.txt / room6_carry.txt) で、古い観測を忘れる仕組みの
設定を変えて走らせ、各ステップの採点 (--track-csv) から次を求める:

  - 跡が消えるまでの時間 t_clear: 移動から、箱の古い位置の付近の幻の表面 (正解の表面から 2h より
    遠い推定表面の通過点) が最後に 0 になるまで
  - 新しい位置が出るまでの時間 t_appear: 箱を置いてから、新しい位置の表面の覆い率が
    最後の 50 ステップの平均の 8 割以上を保つようになるまで
  - 追従の時間 t_follow = max(t_clear, t_appear)
  - 静止した壁・家具の誤差 (移動前の 200〜299 ステップの平均)、覆い率、最後に残る幻の表面の数
  - 分割解と単一ノード解のビット一致 (各実行の PASS (bit-exact) の行)

1ステップ = 測距フレーム1枚 (VL53L5CX の 8x8 で 15Hz) として秒に直す。

使い方 (リポジトリのルートで、ビルド済みの target/release/dyne-sim2d.exe と
sdf2d_grid_actor.wasm を使う):
  .venv\\Scripts\\python.exe scripts\\dynamic_shape_sweep.py [--jobs 6] [--out demo_out\\dynamic_shape]
"""
import argparse
import csv
import math
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.join(ROOT, "target", "release", "dyne-sim2d.exe")
WASM = os.path.join(ROOT, "crates", "sdf2d-grid-actor", "target", "wasm32-unknown-unknown", "release",
                    "sdf2d_grid_actor.wasm")
SCENES = os.path.join(ROOT, "crates", "dyne-sim2d", "scenes")
FPS = 15.0
STEPS = 600
MOVE = 300

# (名前, 追加の引数)。名前は表と図に出す
METHODS = [
    ("w50 (既定)", []),
    ("w20", ["--w-max", "20"]),
    ("w10", ["--w-max", "10"]),
    ("w5", ["--w-max", "5"]),
    ("w3", ["--w-max", "3"]),
    ("減衰0.95+忘却0.3", ["--decay", "0.95", "--w-forget", "0.3"]),
    ("減衰0.9+忘却0.3", ["--decay", "0.9", "--w-forget", "0.3"]),
    ("減衰0.8+忘却0.3", ["--decay", "0.8", "--w-forget", "0.3"]),
    # 注: 忘却の閾値が λ 以上 (例: 減衰0.9+忘却1) だと、1回だけ観測した点 (w=1) が次のステップの
    # 統合の前に忘れられ、全点が w=1 から育たない。アクターはこの設定を初期化で拒む (戻り値 −2)
    ("食い違い0.1", ["--disagree", "0.1", "--disagree-w", "1"]),
    ("食い違い0.05", ["--disagree", "0.05", "--disagree-w", "1"]),
    ("食い違い0.1+減衰0.95+忘却0.3", ["--disagree", "0.1", "--disagree-w", "1", "--decay", "0.95", "--w-forget", "0.3"]),
    ("食い違い0.1+減衰0.9+忘却0.3", ["--disagree", "0.1", "--disagree-w", "1", "--decay", "0.9", "--w-forget", "0.3"]),
]
NOISES = [0.008, 0.02, 0.04]
# (場面ファイル, 置いたステップ)
SCENARIOS = [("room6_move.txt", 300), ("room6_carry.txt", 330)]


def run(job, out_dir):
    name, extra, scene, place, noise = job
    tag = re.sub(r"[^0-9A-Za-z]+", "_", "%s_%s_%s" % (scene.split(".")[0], noise, " ".join(extra) or "default"))
    track = os.path.join(out_dir, "track_%s.csv" % tag)
    cmd = [EXE, "--wasm", WASM, "--scene", os.path.join(SCENES, scene), "--px", "4", "--py", "4",
           "--steps", str(STEPS), "--step-ms", "66.667", "--noise-m", str(noise), "--quiet",
           "--track-csv", track] + extra
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8")
    verdict = [l for l in p.stdout.splitlines() if l.startswith(("PASS", "FAIL"))]
    rows = list(csv.DictReader(open(track, encoding="utf-8")))
    return job, tag, verdict[0] if verdict else "NO VERDICT rc=%d %s" % (p.returncode, p.stderr[-300:]), rows


def f(r, k):
    v = float(r[k])
    return v


def metrics(rows, place):
    by = {int(r["step"]): r for r in rows}
    last = max(by)
    # 跡が消えるまで
    bad = [s for s in by if s >= MOVE and f(by[s], "phantom_old") > 0]
    t_clear = 0.0 if not bad else ((max(bad) + 1 - MOVE) / FPS if max(bad) < last else math.inf)
    # 新しい位置が出るまで
    tail = [f(by[s], "mover_cov") for s in range(last - 49, last + 1)]
    final = sum(tail) / len(tail)
    t_appear = math.inf
    if final > 0:
        ok_from = None
        for s in range(place, last + 1):
            if f(by[s], "mover_cov") >= 0.8 * final:
                if ok_from is None:
                    ok_from = s
            else:
                ok_from = None
        if ok_from is not None:
            t_appear = (ok_from - place) / FPS
    pre = [by[s] for s in range(200, 300)]
    mean = lambda v: sum(v) / len(v)
    return {
        "t_clear": t_clear,
        "t_appear": t_appear,
        "t_follow": max(t_clear, t_appear),
        "static_err_mean_mm": 1000 * mean([f(r, "static_err_mean") for r in pre]),
        "static_err_p95_mm": 1000 * mean([f(r, "static_err_p95") for r in pre]),
        "static_cov_pct": 100 * mean([f(r, "static_cov") for r in pre]),
        "mover_cov_final_pct": 100 * final,
        "mover_err_final_mm": 1000 * mean([f(by[s], "mover_err_mean") for s in range(last - 49, last + 1)]),
        "phantom_final": mean([f(by[s], "phantom") for s in range(last - 49, last + 1)]),
    }


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--out", default=os.path.join(ROOT, "demo_out", "dynamic_shape"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    jobs = []
    for scene, place in SCENARIOS:
        for noise in NOISES:
            for name, extra in METHODS:
                if scene != "room6_move.txt" and noise != 0.008:
                    continue
                jobs.append((name, extra, scene, place, noise))
    results = []
    with ThreadPoolExecutor(a.jobs) as ex:
        for job, tag, verdict, rows in ex.map(lambda j: run(j, a.out), jobs):
            m = metrics(rows, job[3])
            results.append((job, tag, verdict, m, rows))
            print("%-28s %-16s noise %.3f  follow %5.2f s (clear %5.2f, appear %5.2f)  static %.1f/%.1f mm  %s"
                  % (job[0], job[2], job[4], m["t_follow"], m["t_clear"], m["t_appear"],
                     m["static_err_mean_mm"], m["static_err_p95_mm"], verdict), flush=True)
    keys = list(results[0][3].keys())
    with open(os.path.join(a.out, "summary.csv"), "w", encoding="utf-8", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["method", "args", "scene", "noise_m"] + keys + ["bit_exact"])
        for job, tag, verdict, m, _ in results:
            w.writerow([job[0], " ".join(job[1]), job[2], job[4]] + ["%.3f" % m[k] for k in keys] + [verdict])
    plot(results, a.out)
    bad = [r for r in results if not r[2].startswith("PASS (bit-exact)")]
    print("bit-exact: %d / %d PASS" % (len(results) - len(bad), len(results)))
    sys.exit(1 if bad else 0)


def plot(results, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "MS Gothic", "sans-serif"]
    colors = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
    # 図1: 時系列 (room6_move、雑音 8mm) — 古い位置の幻の表面と新しい位置の覆い率
    pick = ["w50 (既定)", "w10", "減衰0.9+忘却0.3", "食い違い0.1", "食い違い0.1+減衰0.95+忘却0.3"]
    fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for ci, name in enumerate(pick):
        for job, tag, verdict, m, rows in results:
            if job[0] == name and job[2] == "room6_move.txt" and job[4] == 0.008:
                t = [(int(r["step"]) - MOVE) / FPS for r in rows]
                axs[0].plot(t, [float(r["phantom"]) for r in rows], color=colors[ci], lw=2, label=name)
                axs[1].plot(t, [100 * float(r["mover_cov"]) for r in rows], color=colors[ci], lw=2, label=name)
    axs[0].set_ylabel("幻の表面の通過点の数")
    axs[1].set_ylabel("箱の今の位置の覆い率 [%]")
    axs[1].set_xlabel("箱を動かしてからの時間 [s] (15Hz、1ステップ=1フレーム)")
    axs[1].set_xlim(-3, 15)
    for ax in axs:
        ax.grid(color="#e0e0e0", lw=0.8)
        ax.axvline(0, color="#888", lw=1, ls="--")
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    axs[0].legend(frameon=False)
    axs[0].set_title("箱を目印Aから目印Bへ動かしたときの追従 (6畳、センサ8個、雑音8mm、4x4ノード)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "follow_timeseries.png"), dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main()

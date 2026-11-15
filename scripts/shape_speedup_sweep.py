"""形の格子型アクターを速くする工夫の、形の質への影響の掃引 (2026-11-13、docs/2026-11-11-shape-speedup.md)。

dyne-sim2d と格子型アクター (sdf2d-grid-actor のフル版。f32 版は target-f32) を、箱を動かす場面で
走らせ、各ステップの採点 (--track-csv) から scripts/dynamic_shape_sweep.py と同じ指標を求める:

  - 追従の時間 t_follow = max(跡が消えるまで, 新しい位置が出るまで) [s]
  - 静止した壁・家具の誤差 (移動前の 200〜299 ステップの平均) と覆い率
  - 箱の新しい位置の覆い率・誤差 (最後の 50 ステップ)、最後に残る幻の表面の数
  - 分割解と単一ノード解のビット一致 (各実行の PASS (bit-exact) の行)

採点の尺度は刻み h ではなく固定の長さ (--scale、既定 5cm。dyne-sim2d の --track-scale-m) にする
(dynamic_shape_sweep.py は刻み h を尺度にしているので、刻み 5cm の値はそのまま比べられる)。

加えて、首振り (PAN4H) で壁が消えないかを見るため、静止した面の覆い率の最小値
(100 ステップ以降、移動の前後を含む) を出す。

比べる工夫 (アクターの設定と格子):
  - 刻み h = 2.5cm / 5cm
  - 計算の型 f64 / f32 (--features f32 のフル版)
  - 距離の反復の間引き --sweep-every 1 / 2 / 4
  - 視野の中だけの減衰 --decay-in-view

使い方 (リポジトリのルートで。先に scripts\\build-actor-wasm.ps1 と cargo build --release -p dyne-sim2d):
  .venv\\Scripts\\python.exe scripts\\shape_speedup_sweep.py [--jobs 6] [--out demo_out\\shape_speedup]
"""
import argparse
import csv
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dynamic_shape_sweep as dss  # noqa: E402  (指標の定義を共有する)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.join(ROOT, "target", "release", "dyne-sim2d.exe")
REL = os.path.join("wasm32-unknown-unknown", "release", "sdf2d_grid_actor.wasm")
WASM = {
    "f64": os.path.join(ROOT, "crates", "sdf2d-grid-actor", "target", REL),
    "f32": os.path.join(ROOT, "crates", "sdf2d-grid-actor", "target-f32", REL),
}
SCENES = os.path.join(ROOT, "crates", "dyne-sim2d", "scenes")
STEPS = 600
# 推奨の忘れる仕組み (docs/2026-11-11-dynamic-shape.md)
FORGET = ["--disagree", "0.1", "--disagree-w", "1", "--decay", "0.9", "--w-forget", "0.3"]
# (場面ファイル, 置いたステップ)
SCENARIOS = {
    "move8": ("room6_move.txt", 300),
    "carry8": ("room6_carry.txt", 330),
    "movePAN4H": ("room6_move_pan4h.txt", 300),
}
# (名前, 型, h, 追加の引数)
METHODS = [
    ("基準 h2.5", "f64", 0.025, []),
    ("h5", "f64", 0.05, []),
    ("h2.5 間引き2", "f64", 0.025, ["--sweep-every", "2"]),
    ("h2.5 間引き4", "f64", 0.025, ["--sweep-every", "4"]),
    ("h2.5 f32", "f32", 0.025, []),
    ("h2.5 f32+間引き2", "f32", 0.025, ["--sweep-every", "2"]),
    ("h5 f32", "f32", 0.05, []),
    ("h2.5 視野内減衰", "f64", 0.025, ["--decay-in-view"]),
    ("h5 視野内減衰", "f64", 0.05, ["--decay-in-view"]),
    ("h2.5 f32+間引き2+視野内減衰", "f32", 0.025, ["--sweep-every", "2", "--decay-in-view"]),
    ("h2.5 視野内減衰+視野外0.99", "f64", 0.025, ["--decay-in-view", "--decay-out", "0.99"]),
    ("h2.5 視野内減衰+視野外0.98", "f64", 0.025, ["--decay-in-view", "--decay-out", "0.98"]),
    ("h5 視野内減衰+視野外0.98", "f64", 0.05, ["--decay-in-view", "--decay-out", "0.98"]),
    ("h2.5 f32+間引き2+視野内減衰+視野外0.98", "f32", 0.025, ["--sweep-every", "2", "--decay-in-view", "--decay-out", "0.98"]),
]


def run(job, out_dir, a_scale):
    name, ty, h, extra, sc_key = job
    scene, place = SCENARIOS[sc_key]
    tag = re.sub(r"[^0-9A-Za-z]+", "_", "%s_%s_h%s_%s" % (sc_key, ty, h, " ".join(extra) or "base"))
    track = os.path.join(out_dir, "track_%s.csv" % tag)
    cmd = [EXE, "--wasm", WASM[ty], "--scene", os.path.join(SCENES, scene), "--px", "4", "--py", "4",
           "--steps", str(STEPS), "--step-ms", "66.667", "--noise-m", "0.008", "--h", str(h),
           "--track-csv", track, "--track-scale-m", str(a_scale)] + FORGET + extra
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8")
    lines = p.stdout.splitlines()
    verdict = [l for l in lines if l.startswith(("PASS", "FAIL"))]
    final = {}
    for l in lines:
        m = re.match(r"shape: surface points \d+ \| \|phi-truth\| mean ([\d.]+) m p95 ([\d.]+) m", l)
        if m:
            final["final_surf_err_mean_mm"] = 1000 * float(m.group(1))
        m = re.match(r"shape: coverage ([\d.]+)%", l)
        if m:
            final["final_cov_pct"] = float(m.group(1))
    rows = list(csv.DictReader(open(track, encoding="utf-8")))
    return job, verdict[0] if verdict else "NO VERDICT rc=%d %s" % (p.returncode, p.stderr[-300:]), rows, final


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--out", default=os.path.join(ROOT, "demo_out", "shape_speedup"))
    ap.add_argument("--scenes", default=",".join(SCENARIOS))
    ap.add_argument("--methods", default="", help="名前にこの文字列を含む方法だけ (空なら全部)")
    ap.add_argument("--scale", type=float, default=0.05,
                    help="採点の尺度 [m] (幻の表面の閾値 2倍・覆い率の半径。刻みの違う実行を同じ基準で比べる。既定 5cm)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    jobs = [(n, t, h, e, s) for s in a.scenes.split(",") for (n, t, h, e) in METHODS if a.methods in n]
    results = []
    with ThreadPoolExecutor(a.jobs) as ex:
        for job, verdict, rows, final in ex.map(lambda j: run(j, a.out, a.scale), jobs):
            m = dss.metrics(rows, SCENARIOS[job[4]][1])
            covs = [float(r["static_cov"]) for r in rows if int(r["step"]) >= 100]
            m["static_cov_min_pct"] = 100 * min(covs)
            m.update(final)
            results.append((job, verdict, m))
            print("%-10s %-28s follow %5.2f s (clear %5.2f, appear %5.2f)  static %.1f/%.1f mm cov %.1f%% (min %.1f%%)"
                  "  box %.1f%% %.1f mm  %s"
                  % (job[4], job[0], m["t_follow"], m["t_clear"], m["t_appear"], m["static_err_mean_mm"],
                     m["static_err_p95_mm"], m["static_cov_pct"], m["static_cov_min_pct"],
                     m["mover_cov_final_pct"], m["mover_err_final_mm"], verdict[:40]), flush=True)
    keys = list(results[0][2].keys())
    with open(os.path.join(a.out, "summary.csv"), "w", encoding="utf-8", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["scene", "method", "type", "h", "args"] + keys + ["bit_exact"])
        for job, verdict, m in results:
            w.writerow([job[4], job[0], job[1], job[2], " ".join(job[3])] + ["%.3f" % m.get(k, float("nan")) for k in keys]
                       + [verdict])
    bad = [r for r in results if not r[1].startswith("PASS (bit-exact)")]
    print("bit-exact: %d / %d PASS" % (len(results) - len(bad), len(results)))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()

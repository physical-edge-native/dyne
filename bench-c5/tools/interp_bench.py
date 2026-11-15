"""素の ESP32 (Xtensa、WAMR インタプリタ) で 2次元の場のアクターが間に合うかを判定する PC 側ツール
(2026-11-11、docs/2026-11-11-esp32-interp-bench.md。形のアクターを速くする工夫の組み合わせごとの
素材は 2026-11-13 追加、docs/2026-11-11-shape-speedup.md)。

  make     … bench-c5/assets/ に次を置く (ファームのビルドの前に1回)
             - sdf2d_grid_actor_small.wasm  形の縮小版の推奨 (--features small,std-math、スタック 4KB。
                                            結果を変えない工夫 + 標準ライブラリの sqrt)
             - sdf2d_nostd_small.wasm       形の縮小版 (--features small。libm の sqrt のまま)
             - sdf2d_f32_small.wasm         形の縮小版の単精度 (--features small,f32,std-math)
             - sdf2d_base_small.wasm        工夫の前の縮小版 (比較用の固定の素材。make は作らない)
             - wave2d_actor_small.wasm      波動方程式のアクターの縮小版 (--features small、スタック 8KB)
             - shape_n13.rec / shape_n5.rec (工夫の前と同じ記録・同じ CRC)、shape_sw2_n5.rec (反復を2ステップに1回)、
               shape_f32_n5.rec、
               shape_h5_n5.rec (刻み 5cm)、shape_p4h.rec (首振り PAN4H + 視野の中だけの減衰)、wave_n5.rec
               dyne-sim2d --dump-node が記録した1ノード分の呼び出し (末尾に期待 CRC)
  verdict  … 実機のシリアル出力 (#BENCH 行) を読み、判定の表を出す
             (ファイルを指定。省略すると標準入力から読む。貼り付けて Ctrl+Z, Enter で終わる)

使い方 (リポジトリ直下で):
  .venv\\Scripts\\python.exe bench-c5\\tools\\interp_bench.py make
  .venv\\Scripts\\python.exe bench-c5\\tools\\interp_bench.py verdict 実機ログ.txt

記録の条件 (make が使う。bench.c の再生とは記録の書式だけで結びつく):
  形: crates/dyne-sim2d/scenes/room6_move.txt、刻み 2.5cm (145×109点)、4×4 分割 (統合の遅れ L = 7)、
      雑音 8mm、推奨の忘れる仕組み (食い違い 0.1 + 減衰 0.9 + 忘却 0.3)、200 ステップ。
      ノード13 = センサを2台持つ区画 (36×27点)、ノード5 = 光線の中継が最も多い区画 (36×27点、界面4つ)。
      shape_p4h は room6_move_pan4h.txt (首振り7台) で、1ステップの燃料が最も多いノード15 (センサ8 ID)。
  波: 81×81点を 4×4 分割、ノード5 (20×20点、界面4つ)、Δt = 0.00625、ガウス型の山、200 ステップ。
  どれも縮小版で分割解を解き、フル版の単一ノード解 (f32 の記録は f32 のフル版) とビット一致することを
  確かめてから記録する。
"""

import argparse
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
ROOT = os.path.dirname(BENCH)
ASSETS = os.path.join(BENCH, "assets")
SIM = os.path.join(ROOT, "target", "release", "dyne-sim2d.exe")

SDF = os.path.join(ROOT, "crates", "sdf2d-grid-actor")
WAVE = os.path.join(ROOT, "crates", "wave2d-actor")
REL = os.path.join("wasm32-unknown-unknown", "release")
SDF_FULL = os.path.join(SDF, "target", REL, "sdf2d_grid_actor.wasm")
SDF_SMALL = os.path.join(SDF, "target-small", REL, "sdf2d_grid_actor.wasm")
SDF_SMALL_STD = os.path.join(SDF, "target-small-std", REL, "sdf2d_grid_actor.wasm")
SDF_SMALL_F32 = os.path.join(SDF, "target-small-f32std", REL, "sdf2d_grid_actor.wasm")
SDF_FULL_F32 = os.path.join(SDF, "target-f32", REL, "sdf2d_grid_actor.wasm")
SCENES = os.path.join(ROOT, "crates", "dyne-sim2d", "scenes")
# PAN4H (首振り7台) の記録に使うノード (1ステップの燃料が最も多いノード。recfuel で選んだ)
P4H_NODE = 15
WAVE_FULL = os.path.join(WAVE, "target", REL, "wave2d_actor.wasm")
WAVE_SMALL = os.path.join(WAVE, "target-small", REL, "wave2d_actor.wasm")

SHAPE_ARGS = ["--scene", os.path.join(ROOT, "crates", "dyne-sim2d", "scenes", "room6_move.txt"),
              "--h", "0.025", "--step-ms", "66.667", "--noise-m", "0.008",
              "--disagree", "0.1", "--decay", "0.9", "--w-forget", "0.3",
              "--px", "4", "--py", "4", "--steps", "200"]
WAVE_ARGS = ["--px", "4", "--py", "4", "--nx", "81", "--ny", "81", "--dt", "0.00625", "--steps", "200",
             "--cfg", "0=1.0", "--cfg", "4=0.3", "--cfg", "5=0.6", "--cfg", "6=0.06"]

# 判定の前提 (docs/2026-11-11-dynamic-shape.md)
FRAME_MS = 1000.0 / 15.0       # 測距 VL53L5CX 8×8 の 15Hz = 1ステップ
FOLLOW_STEPS = 10              # シミュレータの追従 0.67 s = 10 ステップ (うち統合の遅れ L = 7)
FOLLOW_LIMIT_S = 2.0           # 実験6の目安
WIFI_HEAP = 70 * 1024          # Wi-Fi + lwIP + MQTT/UDP が使う内部 RAM の目安 (推定)
WAVE_STEPS_EXP10 = 400         # 実験10の場面 (docs/2026-11-11-wave2d.md §7.3)


def run(cmd, cwd=None, env=None):
    print("$", " ".join(cmd))
    r = subprocess.run(cmd, cwd=cwd, env=env, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        print(r.stdout[-3000:], r.stderr[-3000:])
        sys.exit(f"失敗: {' '.join(cmd)}")
    return r.stdout + r.stderr


def cargo_wasm(crate, small, stack, features="small", target_dir="target-small"):
    env = dict(os.environ)
    env.pop("RUSTFLAGS", None)
    cmd = ["cargo", "build", "--release", "--target", "wasm32-unknown-unknown"]
    if small:
        env["RUSTFLAGS"] = f"-C link-arg=-zstack-size={stack}"
    if features:
        cmd += ["--features", features]
    if target_dir:
        cmd += ["--target-dir", target_dir]
    run(cmd, cwd=crate, env=env)


def cmd_make(a):
    # 形のアクター: フル版 (f64・f32) と縮小版 3種 (libm の sqrt / std-math / f32+std-math)。
    # 縮小版のスタックは 4KB (使用は約1.2KB。2026-11-13、docs/2026-11-11-shape-speedup.md)
    cargo_wasm(SDF, False, 0, features=None, target_dir=None)
    cargo_wasm(SDF, False, 0, features="f32", target_dir="target-f32")
    cargo_wasm(SDF, True, 4096)
    cargo_wasm(SDF, True, 4096, features="small,std-math", target_dir="target-small-std")
    cargo_wasm(SDF, True, 4096, features="small,f32,std-math", target_dir="target-small-f32std")
    cargo_wasm(WAVE, False, 0, features=None, target_dir=None)
    cargo_wasm(WAVE, True, 8192)
    run(["cargo", "build", "--release", "-p", "dyne-sim2d"], cwd=ROOT)
    os.makedirs(ASSETS, exist_ok=True)
    if not os.path.exists(os.path.join(ASSETS, "sdf2d_base_small.wasm")):
        sys.exit("assets/sdf2d_base_small.wasm (工夫の前の縮小版、比較用の固定の素材) がありません")
    for src, dst in [(SDF_SMALL_STD, "sdf2d_grid_actor_small.wasm"), (SDF_SMALL, "sdf2d_nostd_small.wasm"),
                     (SDF_SMALL_F32, "sdf2d_f32_small.wasm"), (WAVE_SMALL, "wave2d_actor_small.wasm")]:
        shutil.copyfile(src, os.path.join(ASSETS, dst))
    h5 = [x if x != "0.025" else "0.05" for x in SHAPE_ARGS]
    p4h = [x if not x.endswith("room6_move.txt") else os.path.join(SCENES, "room6_move_pan4h.txt")
           for x in SHAPE_ARGS] + ["--decay-in-view"]
    # (記録, 縮小版, 単一ノード解のフル版, 引数, ノード)。名前は main/bench.c の ASSET・main/CMakeLists.txt と合わせる
    jobs = [
        ("shape_n13.rec", SDF_SMALL_STD, SDF_FULL, SHAPE_ARGS, 13),
        ("shape_n5.rec", SDF_SMALL_STD, SDF_FULL, SHAPE_ARGS, 5),
        ("shape_sw2_n5.rec", SDF_SMALL_STD, SDF_FULL, SHAPE_ARGS + ["--sweep-every", "2"], 5),
        ("shape_f32_n5.rec", SDF_SMALL_F32, SDF_FULL_F32, SHAPE_ARGS, 5),
        ("shape_h5_n5.rec", SDF_SMALL_STD, SDF_FULL, h5, 5),
        ("shape_p4h.rec", SDF_SMALL_STD, SDF_FULL, p4h, P4H_NODE),
        ("wave_n5.rec", WAVE_SMALL, WAVE_FULL, WAVE_ARGS, 5),
    ]
    total = 0
    for name, small, full, args, node in jobs:
        path = os.path.join(ASSETS, name)
        out = run([SIM, "--wasm", small, "--single-wasm", full, "--quiet"] + args +
                  ["--dump-node", str(node), "--dump-file", path])
        ok = "PASS (bit-exact)" in out
        dump = [l for l in out.splitlines() if l.startswith("dump:")]
        print(("  PASS " if ok else "  FAIL ") + name + " " + (dump[0] if dump else ""))
        if not ok:
            sys.exit("縮小版の分割解がフル版の単一ノード解と一致しない (置き場のあふれ?)")
        total += os.path.getsize(path)
    print(f"記録の合計 {total / 1e6:.2f} MB (ESP32 向けのアプリの領域は 3.9MB)")
    print("assets を作りました。次は build.ps1 -Target esp32 -Mode classic")


# ---------- verdict ----------

def parse(lines):
    rows = []
    for line in lines:
        i = line.find("#BENCH ")
        if i < 0:
            continue
        kv = dict(re.findall(r'(\w+)=("[^"]*"|\S+)', line[i + 7:]))
        rows.append(kv)
    return rows


def f(kv, k, default=float("nan")):
    try:
        return float(kv[k])
    except (KeyError, ValueError):
        return default


def cmd_verdict(a):
    if a.log:
        with open(a.log, encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    else:
        lines = sys.stdin.readlines()
    rows = parse(lines)
    if not rows:
        sys.exit("#BENCH 行が見つかりません")
    errors = [r for r in rows if r.get("bench") == "error"]
    for r in errors:
        print("エラー行:", r)
    starts = {r["mode"]: r for r in rows if r.get("bench") == "start"}
    reps = [r for r in rows if r.get("bench") == "replay"]
    # 項目の名前 = item/opts (opts は 2026-11-13 から。速くする工夫の組み合わせ、docs/2026-11-11-shape-speedup.md)
    lab = lambda r: r["item"] + ("/" + r["opts"] if r.get("opts", "-") != "-" else "")
    sweeps = {(r["mode"], lab(r)): r for r in rows if r.get("bench") == "sweep"}
    parts = {(r["mode"], lab(r)): r for r in rows if r.get("bench") == "parts"}
    heaps = [r for r in rows if r.get("bench") == "heap"]

    print("\n== 1. 再生の結果 (1ステップ = 送信メールボックス + 受信メールボックス + dyne_step の合計) ==")
    print("   内訳 (dyne_step_part があるとき): 反復 = 界面の確認と距離の反復、取込 = 光線の取り込み、統合 = 減衰と統合")
    print(f"{'方式':8} {'項目':28} {'区画':7} {'平均ms':>8} {'最小ms':>8} {'最大ms':>8} {'step/s':>7} "
          f"{'反復ms':>7} {'取込ms':>7} {'統合ms':>7} {'待ち最大':>6} {'ヒープ':>7} {'CRC':>6}")
    best = {}
    for r in reps:
        mode, item = r["mode"], lab(r)
        mean, mn, mx = f(r, "total_ms_mean"), f(r, "total_ms_min"), f(r, "total_ms_max")
        pt = parts.get((mode, item))
        if pt:
            sweep_ms, intake_ms, fuse_ms = f(pt, "sweep_ms_mean"), f(pt, "intake_ms_mean"), f(pt, "fuse_ms_mean")
        else:
            sw = sweeps.get((mode, item))
            sweep_ms = f(sw, "sweep_ms") if sw else float("nan")
            intake_ms, fuse_ms = float("nan"), mean - sweep_ms if sw else float("nan")
        crc_ok = r.get("match") == "1"
        print(f"{mode:8} {item:28} {r['nx']+'x'+r['ny']:7} {mean:8.2f} {mn:8.2f} {mx:8.2f} {1000/mean:7.1f} "
              f"{sweep_ms:7.2f} {intake_ms:7.2f} {fuse_ms:7.2f} {r.get('max_pending','-'):>6} "
              f"{int(f(r,'heap_used',0))//1024:>5}KB {'一致' if crc_ok else '不一致':>6}")
        best.setdefault(mode, {})[item] = (mean, mx, crc_ok)
    if any(r.get("match") != "1" for r in reps):
        print("!! CRC 不一致あり: 実機の計算がシミュレータとビット一致していない (時間の値も参考扱い)")

    print("\n== 2. 実験6 (箱の移動への追従) の見込み (工夫の組み合わせごと) ==")
    print(f"前提: 1ステップ = 測距1フレーム ({FRAME_MS:.1f} ms、15Hz)。追従はシミュレータで {FOLLOW_STEPS} ステップ "
          f"(統合の遅れ L = 7 を含む)。\n      1ステップの周期 = max(最も遅いノードの計算, {FRAME_MS:.1f} ms) + 通信 {a.comm_ms:.0f} ms "
          f"(--comm-ms、未実測の仮定)。")
    print(f"{'方式':8} {'工夫':22} {'最遅ノードms':>11} {'周期ms':>8} {'step/s':>7} {'追従 s':>7} {'判定(2s以内)':>9} {'最悪(最大ms)で追従 s':>18}")
    for mode, items in best.items():
        groups = {}
        for k, v in items.items():
            if k.startswith("shape"):
                groups.setdefault(k.split("/", 1)[1] if "/" in k else "-", []).append(v)
        for opts, shape in groups.items():
            worst_mean = max(v[0] for v in shape)
            worst_max = max(v[1] for v in shape)
            period = max(worst_mean, FRAME_MS) + a.comm_ms
            period_max = max(worst_max, FRAME_MS) + a.comm_ms
            t = FOLLOW_STEPS * period / 1000
            t_max = FOLLOW_STEPS * period_max / 1000
            print(f"{mode:8} {opts:22} {worst_mean:11.1f} {period:8.1f} {1000/period:7.1f} {t:7.2f} "
                  f"{'合格' if t <= FOLLOW_LIMIT_S else '不合格':>9} {t_max:18.2f}")
    print("   (h5 は格子の刻みが 5cm でノードの区画が小さい。p4h は首振り PAN4H の場面。追従のステップ数は場面と"
          "設定で変わる: docs/2026-11-11-shape-speedup.md の表を使う)")

    print("\n== 3. 波動方程式 (実験10、81×81 を 4×4 分割、ノード1台 20×20点) ==")
    for mode, items in best.items():
        if "wave_n5" in items:
            mean = items["wave_n5"][0]
            period = mean + a.comm_ms
            print(f"{mode:8} 計算 {mean:.2f} ms/ステップ ({1000/mean:.0f} step/s)、通信 {a.comm_ms:.0f} ms を足すと "
                  f"{WAVE_STEPS_EXP10} ステップで {WAVE_STEPS_EXP10*period/1000:.1f} s")

    print("\n== 4. メモリ (WROVER の PSRAM が要るか) ==")
    for mode, st in starts.items():
        boot = [h for h in heaps if h["mode"] == mode and h["at"] == "runtime_init"]
        free0 = int(f(boot[0], "free_int", 0)) if boot else 0
        largest0 = int(f(boot[0], "largest_int", 0)) if boot else 0
        used = [int(f(r, "heap_used", 0)) for r in reps if r["mode"] == mode]
        peak = max(used) if used else 0
        min_free = min([int(f(r, "min_free_int", 1 << 30)) for r in reps if r["mode"] == mode] or [0])
        margin = free0 - peak - WIFI_HEAP
        print(f"{mode:8} 計測前の空き {free0//1024}KB (最大の連続 {largest0//1024}KB)、アクター1つの使用 {peak//1024}KB、"
              f"最小の空き {min_free//1024}KB、psram_total={st.get('psram_total')}")
        print(f"         Wi-Fi 等の分 (推定 {WIFI_HEAP//1024}KB) を引いた余り 約 {margin//1024}KB → "
              + ("内部 RAM で足りる見込み (PSRAM は不要。線形メモリを PSRAM に置くとむしろ遅い)"
                 if margin > 30 * 1024 else "余裕が少ない/足りない: PSRAM 付き (WROVER) か、ノードの処理の削減が要る"))
    print("\n注: 線形メモリ 64KB は1つの連続した塊が要る。ESPr (WROOM-32E) の Wi-Fi 稼働中の最大の連続空きは"
          "約 108KB (node-esp32 の実測) なので、1ページなら入る。2ページ (128KB) は PSRAM なしでは入らない。")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("make")
    v = sub.add_parser("verdict")
    v.add_argument("log", nargs="?")
    v.add_argument("--comm-ms", type=float, default=10.0,
                   help="1ステップあたりの通信の時間の仮定 [ms] (既定 10。実験1/実験0の実測で置き換える)")
    a = ap.parse_args()
    {"make": cmd_make, "verdict": cmd_verdict}[a.cmd](a)


if __name__ == "__main__":
    main()

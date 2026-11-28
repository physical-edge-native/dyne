"""bench-c5 の PC 側ツール: 埋め込む素材の作成と、実機の結果と照合する期待値の計算。

  make    … bench-c5/assets/ に次を置く
            - mlp_actor_small.wasm   (crates/mlp-actor の縮小版ビルドの写し)
            - heat1d_actor_small.wasm (node-esp32/main/ にある実機用縮小版の写し)
            - model_a_f64.dynm   2→16→16→1、tanh、重み f64 (scripts/onnx_check.py のモデルAと同じ)
            - model_b_f64.dynm   4→32→32→4、tanh、重み f64
            - model_b_f32.dynm   同じモデルの重みを f32 にしたもの
            - mlp_actor_full.wasm (crates/mlp-actor のフル版ビルドの写し。PSRAM のある基板向け)
            - erg_leg_f64.dynm      67→48→48→8、ReLU、重み f64 (erg の脚ごとの方策と同じ形、6,008 パラメータ)
            - erg_central_f64.dynm  75→48→48→16、ReLU、重み f64 (erg の集中の方策と同じ形、6,784 パラメータ)
              (issue #156。erg の制御の周期 20 ms に収まるかを測るため。縮小版の上限 2,048 には入らないので
               フル版の mlp-actor で動かす。重みは scripts/onnx_check.py の det_values で決定的に作る)
            モデルは onnx.helper で ONNX を作り、scripts/onnx_import.py で .dynm に変換する
            (実際の利用経路と同じ変換を通す)
  check-ref … erg の形の網について、.dynm が今の変換と同じで、mlp-actor の出力が Python の参照計算と
            ビット一致することを確かめる (onnx が要る。CI の21番目)
  expect  … assets/ の wasm と .dynm を wasmtime で、実機と同じ手順・同じ入力で実行し、
            実機が出す「#BENCH ... crc=...」と比べるべき値を表示する

使い方 (リポジトリ直下で):
  .venv\\Scripts\\python.exe bench-c5\\tools\\bench_ref.py make
  .venv\\Scripts\\python.exe bench-c5\\tools\\bench_ref.py expect
  .venv\\Scripts\\python.exe bench-c5\\tools\\bench_ref.py compare 実機ログ.txt
      (実機のシリアル出力を保存したファイルの #BENCH 行と期待値を突き合わせる)

実機側 (bench-c5/main/bench.c) と一致させている約束:
  - mlp: 入力は R 組 (モデル A・B は 64、erg の形は 8。入出力の領域 IO_CAP に入る数)。x[r][i] = (((37r + 11i) mod 201) − 100) / 64
    (2 のべき乗で割るので、どの実装でも丸めなしに同じ値になる)。
    dyne_model_eval_batch(R) を1回呼んだ後の出力 R·n_out 個の f64
    (リトルエンディアンのバイト列) の CRC32 を crc とする。
  - heat1d: 単一ノード・両端 u=0 固定。dyne_init(n, 0, 1, 1.0, dt, 0, 0) の後
    dyne_step を steps 回。dyne_get(0..n-1) を f64 のバイト列に並べた CRC32 を crc とする
    (node-esp32 の BSP done で報告する crc と同じ定義)。
"""

import argparse
import os
import re
import shutil
import struct
import sys
import tempfile
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
ROOT = os.path.dirname(BENCH)
ASSETS = os.path.join(BENCH, "assets")
sys.path.insert(0, os.path.join(ROOT, "scripts"))

MLP_WASM_SRC = os.path.join(ROOT, "crates", "mlp-actor", "target", "small",
                            "wasm32-unknown-unknown", "release", "mlp_actor.wasm")
MLP_FULL_WASM_SRC = os.path.join(ROOT, "crates", "mlp-actor", "target",
                                 "wasm32-unknown-unknown", "release", "mlp_actor.wasm")
HEAT_WASM_SRC = os.path.join(ROOT, "node-esp32", "main", "heat1d_actor_small.wasm")

# (モデル名, 使う mlp-actor の素材, 入力の組数 R)。bench.c の bench_mlp の呼び出しと一致させること
MODELS = [
    ("model_a_f64", "mlp_actor_small.wasm", 64),
    ("model_b_f64", "mlp_actor_small.wasm", 64),
    ("model_b_f32", "mlp_actor_small.wasm", 64),
    ("erg_leg_f64", "mlp_actor_full.wasm", 8),
    ("erg_central_f64", "mlp_actor_full.wasm", 8),
]
# erg の方策の形 (erg の erg/ppo.py ACTOR_HIDDEN = (48, 48)、2026-11-27 に読んだもの)
ERG_SHAPES = {"erg_leg_f64": [67, 48, 48, 8], "erg_central_f64": [75, 48, 48, 16]}
# (点数, dt, ステップ数)。bench.c の HEAT_CASES と一致させること
HEAT_CASES = [(65, 1e-4, 2000), (1025, 1e-7, 200)]


def bench_input(rows, n_in):
    x = np.empty((rows, n_in), dtype="<f8")
    for r in range(rows):
        for i in range(n_in):
            x[r, i] = float(((37 * r + 11 * i) % 201) - 100) / 64.0
    return x


# ---------- make ----------

def cmd_make(_a):
    import onnx
    from onnx import TensorProto
    import onnx_check
    import onnx_import

    os.makedirs(ASSETS, exist_ok=True)
    for src, name in ((MLP_WASM_SRC, "mlp_actor_small.wasm"), (MLP_FULL_WASM_SRC, "mlp_actor_full.wasm"),
                      (HEAT_WASM_SRC, "heat1d_actor_small.wasm")):
        if not os.path.exists(src):
            sys.exit(f"{src} がありません。先にビルドしてください (README の手順1)")
        shutil.copyfile(src, os.path.join(ASSETS, name))
        print(f"写し: {src} → assets/{name} ({os.path.getsize(src)} バイト)")

    work = tempfile.mkdtemp(prefix="bench_c5_")
    mA = onnx_check.model_gemm_tanh(np.random.default_rng(1), [2, 16, 16, 1], np.float64, TensorProto.DOUBLE)
    mB = onnx_check.model_gemm_tanh(np.random.default_rng(3), [4, 32, 32, 4], np.float64, TensorProto.DOUBLE)
    for model, onnx_name, outs in ((mA, "A", [("model_a_f64", "f64")]),
                                   (mB, "B", [("model_b_f64", "f64"), ("model_b_f32", "f32")])):
        p = os.path.join(work, f"{onnx_name}.onnx")
        onnx.save(model, p)
        for out_name, dt in outs:
            data, n_in, layers, n_params, f32w = onnx_import.convert(p, dt)
            with open(os.path.join(ASSETS, out_name + ".dynm"), "wb") as f:
                f.write(data)
            print(f"作成: assets/{out_name}.dynm  入力 {n_in}、層 {len(layers)}、"
                  f"パラメータ {n_params}、{len(data)} バイト")
    for name, dims in ERG_SHAPES.items():
        p = os.path.join(work, f"{name}.onnx")
        onnx.save(onnx_check.model_gemm_relu(dims, np.float64, TensorProto.DOUBLE), p)
        data, n_in, layers, n_params, f32w = onnx_import.convert(p, "f64")
        with open(os.path.join(ASSETS, name + ".dynm"), "wb") as f:
            f.write(data)
        print(f"作成: assets/{name}.dynm  入力 {n_in}、層 {len(layers)}、パラメータ {n_params}、{len(data)} バイト")
    print("次に expect で期待値を確認してください")


# ---------- expect ----------

class Wasm:
    def __init__(self, path):
        import wasmtime
        self.engine = wasmtime.Engine()
        self.store = wasmtime.Store(self.engine)
        self.inst = wasmtime.Instance(self.store, wasmtime.Module.from_file(self.engine, path), [])
        self.ex = self.inst.exports(self.store)
        self.mem = self.ex["memory"]

    def __call__(self, name, *args):
        return self.ex[name](self.store, *args)

    def pages(self):
        return self.mem.size(self.store)


def expected():
    """期待値の辞書の列: {"bench": ..., "model"/"n": ..., "crc": ...}"""
    res = []
    heat_path = os.path.join(ASSETS, "heat1d_actor_small.wasm")
    for m, wasm_name, rows in MODELS:
        w = Wasm(os.path.join(ASSETS, wasm_name))  # 実機と同様、モデルごとに新しいインスタンス
        data = open(os.path.join(ASSETS, m + ".dynm"), "rb").read()
        w.mem.write(w.store, data, w("dyne_load_buf"))
        rc = w("dyne_model_load", len(data))
        n_in, n_out = w("dyne_model_n_in"), w("dyne_model_n_out")
        x = bench_input(rows, n_in)
        w.mem.write(w.store, x.tobytes(), w("dyne_in_buf"))
        r = w("dyne_model_eval_batch", rows)
        ob = w("dyne_out_buf")
        out = bytes(w.mem.read(w.store, ob, ob + 8 * rows * n_out))
        y = np.frombuffer(out, dtype="<f8")
        res.append(dict(bench="mlp", model=m, rows=rows, load=rc, ret=r, n_in=n_in, n_out=n_out,
                        params=w("dyne_model_n_params"), pages=w.pages(),
                        crc=f"0x{zlib.crc32(out):08X}", y0=f"{y[0]:.17g}"))
    for n, dt, steps in HEAT_CASES:
        w = Wasm(heat_path)
        rc = w("dyne_init", n, 0.0, 1.0, 1.0, dt, 0, 0)
        for _ in range(steps):
            w("dyne_step")
        b = b"".join(struct.pack("<d", w("dyne_get", i)) for i in range(n))
        res.append(dict(bench="heat1d", n=n, steps=steps, init=rc, pages=w.pages(),
                        crc=f"0x{zlib.crc32(b):08X}"))
    return res


def cmd_expect(_a):
    global ASSETS
    if getattr(_a, "assets", None):
        ASSETS = _a.assets
    if getattr(_a, "mlp_wasm", None):
        # 作り直した mlp-actor で確かめる (issue #156): 素材を一時フォルダへ写し、wasm だけ差し替える
        tmp = tempfile.mkdtemp(prefix="bench_c5_expect_")
        for f in os.listdir(ASSETS):
            shutil.copyfile(os.path.join(ASSETS, f), os.path.join(tmp, f))
        shutil.copyfile(_a.mlp_wasm, os.path.join(tmp, "mlp_actor_small.wasm"))
        if getattr(_a, "mlp_full_wasm", None):
            shutil.copyfile(_a.mlp_full_wasm, os.path.join(tmp, "mlp_actor_full.wasm"))
        ASSETS = tmp
    for e in expected():
        if e["bench"] == "mlp":
            print(f"#EXPECT bench=mlp model={e['model']} n_in={e['n_in']} n_out={e['n_out']} "
                  f"params={e['params']} rows={e['rows']} load={e['load']} pages={e['pages']} "
                  f"crc={e['crc']} y0={e['y0']}")
        else:
            print(f"#EXPECT bench=heat1d n={e['n']} steps={e['steps']} init={e['init']} "
                  f"pages={e['pages']} crc={e['crc']}")


def cmd_check_ref(_a):
    """erg の形の網 (issue #156): 置いてある .dynm が今の変換で同じバイト列になり、フル版の mlp-actor の出力が
    アクターと同じ演算順の Python の参照計算 (scripts/onnx_check.py の ref_eval) とビット一致することを確かめる"""
    import onnx
    from onnx import TensorProto
    import onnx_check
    import onnx_import
    exp = {e["model"]: e for e in expected() if e["bench"] == "mlp"}
    ok = True
    work = tempfile.mkdtemp(prefix="bench_c5_ref_")
    for name, dims in ERG_SHAPES.items():
        p = os.path.join(work, f"{name}.onnx")
        onnx.save(onnx_check.model_gemm_relu(dims, np.float64, TensorProto.DOUBLE), p)
        data, n_in, layers, _n, f32w = onnx_import.convert(p, "f64")
        same_file = data == open(os.path.join(ASSETS, name + ".dynm"), "rb").read()
        rows = next(r for m, _w, r in MODELS if m == name)
        y = np.array([onnx_check.ref_eval(layers, row, f32w) for row in bench_input(rows, n_in)], dtype="<f8")
        crc = f"0x{zlib.crc32(y.tobytes()):08X}"
        good = same_file and crc == exp[name]["crc"]
        ok &= good
        print(f"bench-ref {name}: .dynm {'same' if same_file else 'DIFFERENT'}, reference {crc}, "
              f"mlp-actor {exp[name]['crc']} -> {'PASS (bit-exact)' if good else 'FAIL'}")
    sys.exit(0 if ok else 1)


def parse_bench(path):
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            i = line.find("#BENCH ")
            if i < 0:
                continue
            kv = dict(re.findall(r"(\w+)=(\S+)", line[i + 7:]))
            rows.append(kv)
    return rows


def cmd_compare(a):
    got = parse_bench(a.log)
    ok = True
    n_checked = 0
    for e in expected():
        if e["bench"] == "mlp":
            hits = [g for g in got if g.get("bench") == "mlp" and g.get("model") == e["model"] and "crc" in g]
            label = f"mlp {e['model']}"
        else:
            hits = [g for g in got if g.get("bench") == "heat1d" and g.get("n") == str(e["n"]) and "crc" in g]
            label = f"heat1d n={e['n']}"
        if not hits:
            print(f"{label}: 実機の結果がログにありません")
            ok = False
            continue
        for g in hits:
            same = g["crc"].upper() == e["crc"].upper()
            n_checked += 1
            ok &= same
            print(f"{label} [{g.get('mode', '?')}]: 実機 {g['crc']} / PC {e['crc']} → "
                  f"{'ビット一致' if same else '不一致'}")
    print("PASS (bit-exact)" if ok and n_checked else "FAIL")
    sys.exit(0 if ok and n_checked else 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("make").set_defaults(fn=cmd_make)
    e = sub.add_parser("expect")
    e.add_argument("--assets", help="素材のフォルダ (既定 bench-c5/assets)。CI は作り直した mlp-actor を入れたフォルダで確かめる")
    e.add_argument("--mlp-wasm", help="mlp_actor_small.wasm の代わりに使う wasm (作り直したもの)")
    e.add_argument("--mlp-full-wasm", help="mlp_actor_full.wasm の代わりに使う wasm (作り直したもの。--mlp-wasm と一緒に使う)")
    e.set_defaults(fn=cmd_expect)
    sub.add_parser("check-ref").set_defaults(fn=cmd_check_ref)
    c = sub.add_parser("compare")
    c.add_argument("log")
    c.set_defaults(fn=cmd_compare)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()

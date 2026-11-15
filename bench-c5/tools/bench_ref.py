"""bench-c5 の PC 側ツール: 埋め込む素材の作成と、実機の結果と照合する期待値の計算。

  make    … bench-c5/assets/ に次を置く
            - mlp_actor_small.wasm   (crates/mlp-actor の縮小版ビルドの写し)
            - heat1d_actor_small.wasm (node-esp32/main/ にある実機用縮小版の写し)
            - model_a_f64.dynm   2→16→16→1、tanh、重み f64 (scripts/onnx_check.py のモデルAと同じ)
            - model_b_f64.dynm   4→32→32→4、tanh、重み f64
            - model_b_f32.dynm   同じモデルの重みを f32 にしたもの
            モデルは onnx.helper で ONNX を作り、scripts/onnx_import.py で .dynm に変換する
            (実際の利用経路と同じ変換を通す)
  expect  … assets/ の wasm と .dynm を wasmtime で、実機と同じ手順・同じ入力で実行し、
            実機が出す「#BENCH ... crc=...」と比べるべき値を表示する

使い方 (リポジトリ直下で):
  .venv\\Scripts\\python.exe bench-c5\\tools\\bench_ref.py make
  .venv\\Scripts\\python.exe bench-c5\\tools\\bench_ref.py expect
  .venv\\Scripts\\python.exe bench-c5\\tools\\bench_ref.py compare 実機ログ.txt
      (実機のシリアル出力を保存したファイルの #BENCH 行と期待値を突き合わせる)

実機側 (bench-c5/main/bench.c) と一致させている約束:
  - mlp: 入力は R=64 組。x[r][i] = (((37r + 11i) mod 201) − 100) / 64
    (2 のべき乗で割るので、どの実装でも丸めなしに同じ値になる)。
    dyne_model_eval_batch(64) を1回呼んだ後の出力 64·n_out 個の f64
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
HEAT_WASM_SRC = os.path.join(ROOT, "node-esp32", "main", "heat1d_actor_small.wasm")

ROWS = 64
MODELS = ["model_a_f64", "model_b_f64", "model_b_f32"]
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
    for src, name in ((MLP_WASM_SRC, "mlp_actor_small.wasm"), (HEAT_WASM_SRC, "heat1d_actor_small.wasm")):
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
    mlp_path = os.path.join(ASSETS, "mlp_actor_small.wasm")
    heat_path = os.path.join(ASSETS, "heat1d_actor_small.wasm")
    for m in MODELS:
        w = Wasm(mlp_path)  # 実機と同様、モデルごとに新しいインスタンス
        data = open(os.path.join(ASSETS, m + ".dynm"), "rb").read()
        w.mem.write(w.store, data, w("dyne_load_buf"))
        rc = w("dyne_model_load", len(data))
        n_in, n_out = w("dyne_model_n_in"), w("dyne_model_n_out")
        x = bench_input(ROWS, n_in)
        w.mem.write(w.store, x.tobytes(), w("dyne_in_buf"))
        r = w("dyne_model_eval_batch", ROWS)
        ob = w("dyne_out_buf")
        out = bytes(w.mem.read(w.store, ob, ob + 8 * ROWS * n_out))
        y = np.frombuffer(out, dtype="<f8")
        res.append(dict(bench="mlp", model=m, load=rc, ret=r, n_in=n_in, n_out=n_out,
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
    for e in expected():
        if e["bench"] == "mlp":
            print(f"#EXPECT bench=mlp model={e['model']} n_in={e['n_in']} n_out={e['n_out']} "
                  f"params={e['params']} rows={ROWS} load={e['load']} pages={e['pages']} "
                  f"crc={e['crc']} y0={e['y0']}")
        else:
            print(f"#EXPECT bench=heat1d n={e['n']} steps={e['steps']} init={e['init']} "
                  f"pages={e['pages']} crc={e['crc']}")


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
    sub.add_parser("expect").set_defaults(fn=cmd_expect)
    c = sub.add_parser("compare")
    c.add_argument("log")
    c.set_defaults(fn=cmd_compare)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()

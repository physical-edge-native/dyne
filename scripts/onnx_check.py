"""ONNX → .dynm → WASMアクター (crates/mlp-actor) の経路の照合。

torch を使わずに onnx.helper で試験用の MLP を作り、
  (1) onnxruntime (ONNX の標準の実行環境) の推論
  (2) 同じ演算順の Python の参照計算 (math.tanh / math.exp)
      ※ math.tanh は OS の C ライブラリの実装で、アクターの libm (純Rust) とは
        最終桁が違うことがある。よってここはビット一致ではなく 1e-14 以内を合格とする
  (3) .dynm を読み込んだ WASMアクターの推論 (wasmtime で実行)
を同じ入力で比べ、最大絶対誤差を報告する。あわせて WASM の大きさと
線形メモリのページ数 (1ページ=64KB) を報告する。

試験するモデル:
  A: 2→16→16→1、tanh、Gemm (transB=1、PyTorch の Linear と同じ形)、f64
     → .dynm f64 と f32 (量子化) の両方
  B: 3→8→8→2、Relu/Sigmoid、MatMul+Add、f32 → .dynm f32
  C: Softmax と Conv を含むモデル → 変換が拒否されること

使い方:
  .venv\\Scripts\\python.exe scripts\\onnx_check.py [--wasm 経路] [--wasm-small 経路] [--out 作業ディレクトリ]
"""

import argparse
import math
import os
import struct
import sys
import tempfile

import numpy as np
import onnx
import onnxruntime as ort
import wasmtime
from onnx import TensorProto, helper, numpy_helper

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import onnx_import  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WASM_FULL = os.path.join(ROOT, "crates", "mlp-actor", "target", "wasm32-unknown-unknown", "release", "mlp_actor.wasm")
WASM_SMALL = os.path.join(ROOT, "crates", "mlp-actor", "target", "small", "wasm32-unknown-unknown", "release", "mlp_actor.wasm")


# ---------- 試験用モデルの作成 ----------

def model_gemm_tanh(rng, dims, np_dtype, tp_dtype):
    """Gemm(transB=1)+Tanh を積んだ MLP。最後の層は活性化なし"""
    nodes, inits = [], []
    cur = "x"
    for k in range(len(dims) - 1):
        w = rng.normal(0, 1.0 / math.sqrt(dims[k]), (dims[k + 1], dims[k])).astype(np_dtype)
        b = rng.normal(0, 0.1, dims[k + 1]).astype(np_dtype)
        inits += [numpy_helper.from_array(w, f"W{k}"), numpy_helper.from_array(b, f"b{k}")]
        nodes.append(helper.make_node("Gemm", [cur, f"W{k}", f"b{k}"], [f"z{k}"], transB=1))
        cur = f"z{k}"
        if k < len(dims) - 2:
            nodes.append(helper.make_node("Tanh", [cur], [f"h{k}"]))
            cur = f"h{k}"
    nodes.append(helper.make_node("Identity", [cur], ["y"]))
    g = helper.make_graph(
        nodes, "mlp_gemm_tanh",
        [helper.make_tensor_value_info("x", tp_dtype, ["N", dims[0]])],
        [helper.make_tensor_value_info("y", tp_dtype, ["N", dims[-1]])],
        inits,
    )
    return helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)], ir_version=8)


def model_matmul_mixed(rng):
    """MatMul+Add、Relu → Relu → Sigmoid、f32"""
    dims = [3, 8, 8, 2]
    acts = ["Relu", "Relu", "Sigmoid"]
    nodes, inits = [], []
    cur = "x"
    for k in range(3):
        w = rng.normal(0, 0.7, (dims[k], dims[k + 1])).astype(np.float32)  # [入力][出力]
        b = rng.normal(0, 0.1, dims[k + 1]).astype(np.float32)
        inits += [numpy_helper.from_array(w, f"W{k}"), numpy_helper.from_array(b, f"b{k}")]
        nodes.append(helper.make_node("MatMul", [cur, f"W{k}"], [f"m{k}"]))
        nodes.append(helper.make_node("Add", [f"m{k}", f"b{k}"], [f"z{k}"]))
        nodes.append(helper.make_node(acts[k], [f"z{k}"], [f"h{k}"] if k < 2 else ["y"]))
        cur = f"h{k}"
    g = helper.make_graph(
        nodes, "mlp_matmul_mixed",
        [helper.make_tensor_value_info("x", TensorProto.FLOAT, ["N", 3])],
        [helper.make_tensor_value_info("y", TensorProto.FLOAT, ["N", 2])],
        inits,
    )
    return helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)], ir_version=8)


def model_unsupported():
    nodes = [
        helper.make_node("Conv", ["x", "W"], ["c"]),
        helper.make_node("Softmax", ["c"], ["y"]),
    ]
    g = helper.make_graph(
        nodes, "bad",
        [helper.make_tensor_value_info("x", TensorProto.FLOAT, [1, 1, 4, 4])],
        [helper.make_tensor_value_info("y", TensorProto.FLOAT, None)],
        [numpy_helper.from_array(np.ones((1, 1, 3, 3), np.float32), "W")],
    )
    return helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)], ir_version=8)


# ---------- 参照計算 (アクターと同じ演算順) ----------

def ref_eval(layers, x, f32w):
    def q(v):
        return float(np.float32(v)) if f32w else float(v)
    h = [float(v) for v in x]
    for l in layers:
        nh = []
        for j in range(l.n_out):
            acc = 0.0
            for i in range(l.n_in):
                acc += q(l.w[j, i]) * h[i]
            acc += q(l.b[j])
            if l.act == "Tanh":
                acc = math.tanh(acc)
            elif l.act == "Relu":
                acc = acc if acc > 0.0 else 0.0
            elif l.act == "Sigmoid":
                acc = 1.0 / (1.0 + math.exp(-acc))
            nh.append(acc)
        h = nh
    return h


# ---------- WASMアクターの駆動 ----------

class Actor:
    def __init__(self, path):
        self.engine = wasmtime.Engine()
        self.store = wasmtime.Store(self.engine)
        module = wasmtime.Module.from_file(self.engine, path)
        self.inst = wasmtime.Instance(self.store, module, [])
        self.ex = self.inst.exports(self.store)
        self.mem = self.ex["memory"]
        self.wasm_bytes = os.path.getsize(path)

    def call(self, name, *args):
        return self.ex[name](self.store, *args)

    def pages(self):
        return self.mem.size(self.store)

    def load(self, data):
        cap = self.call("dyne_load_cap")
        if len(data) > cap:
            return None
        self.mem.write(self.store, data, self.call("dyne_load_buf"))
        return self.call("dyne_model_load", len(data))

    def eval(self, x):
        """x: [行][入力] → [行][出力]"""
        n_in = self.call("dyne_model_n_in")
        n_out = self.call("dyne_model_n_out")
        cap = self.call("dyne_io_cap")
        rows_per = max(1, cap // max(n_in, n_out))
        ib, ob = self.call("dyne_in_buf"), self.call("dyne_out_buf")
        out = []
        for s in range(0, len(x), rows_per):
            chunk = np.ascontiguousarray(x[s:s + rows_per], dtype="<f8")
            self.mem.write(self.store, chunk.tobytes(), ib)
            r = self.call("dyne_model_eval_batch", len(chunk))
            if r != n_out:
                raise RuntimeError(f"dyne_model_eval_batch の戻り値 {r}")
            raw = self.mem.read(self.store, ob, ob + 8 * n_out * len(chunk))
            out.append(np.frombuffer(bytes(raw), dtype="<f8").reshape(len(chunk), n_out))
        return np.vstack(out)


def run_case(name, model, x, dtype, actor, workdir, log):
    path = os.path.join(workdir, f"{name}.onnx")
    onnx.save(model, path)
    data, n_in, layers, n_params, f32w = onnx_import.convert(path, dtype)
    with open(os.path.join(workdir, f"{name}_{'f32' if f32w else 'f64'}.dynm"), "wb") as f:
        f.write(data)
    sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    in_dt = np.float64 if model.graph.input[0].type.tensor_type.elem_type == TensorProto.DOUBLE else np.float32
    y_ort = sess.run(None, {"x": x.astype(in_dt)})[0].astype(np.float64)
    r = actor.load(data)
    if r != 0:
        raise RuntimeError(f"{name}: dyne_model_load の戻り値 {r}")
    # アクターへの入力は f64。f32 のモデルは onnxruntime と同じ f32 に丸めた入力を渡す
    x_act = x.astype(in_dt).astype(np.float64)
    y_wasm = actor.eval(x_act)
    y_ref = np.array([ref_eval(layers, row, f32w) for row in x_act])
    d_ort = float(np.max(np.abs(y_wasm - y_ort)))
    d_ref = float(np.max(np.abs(y_wasm - y_ref)))
    n_bit = int(np.sum(y_wasm.view(np.uint64) == y_ref.view(np.uint64)))
    log(f"[{name} / .dynm {'f32' if f32w else 'f64'}] パラメータ {n_params}、{len(data)} バイト、入力 {len(x)} 組")
    log(f"    onnxruntime との最大絶対誤差      {d_ort:.3e}")
    log(f"    同じ演算順の参照計算との最大誤差  {d_ref:.3e}  (ビット一致 {n_bit}/{y_wasm.size})")
    return d_ort, d_ref, y_wasm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wasm", default=WASM_FULL)
    ap.add_argument("--wasm-small", default=WASM_SMALL)
    ap.add_argument("--out", default=None, help="作業ディレクトリ (省略時は一時ディレクトリ)")
    a = ap.parse_args()
    workdir = a.out or tempfile.mkdtemp(prefix="onnx_check_")
    os.makedirs(workdir, exist_ok=True)
    log = print
    ok = True
    outs = []
    rng = np.random.default_rng(20261108)

    x2 = rng.uniform(-1, 1, (500, 2))
    x3 = rng.uniform(-2, 2, (500, 3))
    mA64 = model_gemm_tanh(np.random.default_rng(1), [2, 16, 16, 1], np.float64, TensorProto.DOUBLE)
    mB = model_matmul_mixed(np.random.default_rng(2))

    for label, wpath in (("フル版", a.wasm), ("縮小版", a.wasm_small)):
        if not os.path.exists(wpath):
            log(f"{label}: {wpath} がありません (ビルドしてください)。飛ばします")
            continue
        actor = Actor(wpath)
        log(f"=== {label}: {wpath}")
        log(f"    WASM {actor.wasm_bytes} バイト、線形メモリ {actor.pages()} ページ "
            f"({actor.pages() * 64} KB)、パラメータ上限 {actor.call('dyne_param_max')}、"
            f"読み込み領域 {actor.call('dyne_load_cap')} バイト")
        d1, r1, y1 = run_case("A_gemm_tanh", mA64, x2, "f64", actor, workdir, log)
        d2, r2, y2 = run_case("A_gemm_tanh", mA64, x2, "f32", actor, workdir, log)
        d3, r3, y3 = run_case("B_matmul_mixed", mB, x3, "auto", actor, workdir, log)
        outs.append((label, np.concatenate([y1.ravel(), y2.ravel(), y3.ravel()])))
        ok &= d1 < 1e-12 and r1 < 1e-14
        ok &= d2 < 1e-5 and r2 < 1e-14
        ok &= d3 < 1e-5 and r3 < 1e-14
        # 壊れたファイルを拒否すること
        data, *_ = onnx_import.convert(os.path.join(workdir, "A_gemm_tanh.onnx"), "f64")
        r = actor.load(data[:-3])
        log(f"    切り詰めたファイルの読み込み: 戻り値 {r} (負なら拒否)")
        ok &= r is not None and r < 0
        r = actor.call("dyne_model_eval_batch", 1)
        log(f"    拒否後の推論: 戻り値 {r} (負なら未読み込みとして拒否)")
        ok &= r < 0

    # フル版と縮小版の出力がビット一致すること (決定論の確認)
    if len(outs) == 2:
        same = outs[0][1].view(np.uint64) == outs[1][1].view(np.uint64)
        log(f"フル版と縮小版の出力のビット一致: {int(same.sum())}/{same.size}")
        ok &= bool(same.all())

    # 未対応の演算の拒否
    path = os.path.join(workdir, "C_unsupported.onnx")
    onnx.save(model_unsupported(), path)
    try:
        onnx_import.convert(path)
        log("C_unsupported: 変換されてしまった (不合格)")
        ok = False
    except onnx_import.ImportError_ as e:
        log(f"[C_unsupported] 期待どおり拒否: {e}")

    log(f"作業ディレクトリ: {workdir}")
    log("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

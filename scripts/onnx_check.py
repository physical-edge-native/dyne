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
  D: 8→32→32→4、Relu、Gemm、f64 (重みは乱数を使わない決定的な値) → .dynm f64 と f32。
     数学関数を使わないので、参照計算と**全件ビット一致**し、.dynm と出力の CRC-32 が固定値と
     合うことを合格とする (issue #155。ReLU の網はほかのリポジトリの制御器が使う形)
  E: erg (ロボットの方策のリポジトリ) が書き出すのと同じ形 (issue #201)。PyTorch の
     nn.Sequential(正規化, Linear, ReLU, Linear, ReLU, Linear) を torch.onnx.export(opset 17,
     dynamo=False) で書き出したときのノード・名前・属性・入力の形 [1, 67] を onnx.helper でそのまま作る。
     大きさは erg と同じ 67→48→48→8。重みは det_values の決定的な値
       E_erg_plain_f64: 正規化なし、float64 → .dynm f64
       E_erg_norm_f64:  入力の正規化 (x - mean) / std つき、float64 → .dynm f64
       E_erg_norm_f32:  同じ網を float32 で書き出したもの → .dynm f32
     どれも、畳み込んだ網の同じ演算順の参照計算と全件ビット一致し、.dynm と出力の CRC-32 が固定値と
     合うことを合格とする。torch が入っていれば、同じ重みの nn.Sequential を実際に torch.onnx.export で
     書き出し、変換した .dynm が onnx.helper で作ったものとバイト列まで一致すること、PyTorch の出力との
     差が下の許容誤差以内であることも確かめる (CI には torch がないので飛ばす)
  F: 正規化の書き方の変種 (5→6→3、f64): Flatten → Reshape([1, -1]、Constant ノード) → Mul (スカラー) →
     Sub ([1, n]) → Add (スカラー) → 定数 - x → Div ([n]) → Gemm。参照計算と全件ビット一致すること
  G: 畳み込めない形が拒否されること (定数 / x、最初の層より後の Sub、2行への Reshape)

畳み込んだ網と元の網 (onnxruntime・PyTorch) の許容誤差 (issue #201。試験の前に決めた値):
  正規化を重みに畳み込むと浮動小数点の演算の順と丸めの回数が変わるので、元の網とはビット一致しない。
  ビット一致の基準は「畳み込んだ網の参照計算」とし、元の網とは相対誤差
      max|y_アクター - y_元| / max(1, max|y_元|)
  で比べる。
    float64: 1e-12。積和の項数は層ごとに 67・48・48 個、項の大きさは出力の大きさの高々 10 倍程度で、
      並べ替え・丸め1回あたりの誤差は 2^-53 ≈ 1.1e-16 の数倍。最悪でも (67+48+48)·10·2.2e-16 ≈ 4e-13 で、
      onnxruntime 自身の和の順 (BLAS) の違いも同じ程度。その2倍余りの余裕を取って 1e-12
    float32: 1e-5。onnxruntime は float32 (2^-24 ≈ 6e-8) で計算し、アクターは float32 に丸めた
      畳み込み後の重みで float64 で計算する。丸めの誤差は符号がばらばらで項数の平方根程度で育つので
      数 1e-7 が目安。これまでの float32 の照合 (モデル A・B・D) と同じ 1e-5 とする

使い方:
  .venv\\Scripts\\python.exe scripts\\onnx_check.py [--wasm 経路] [--wasm-small 経路] [--out 作業ディレクトリ]
"""

import argparse
import math
import os
import struct
import sys
import tempfile
import zlib

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


def det_values(n, salt, scale):
    """乱数を使わない決定的な値の列 (NumPy の乱数の列は版をまたいで同じとは限らないため、
    CRC を固定するモデルはこちらで作る)。値は (k·7919 + salt) mod 2001 を [-1, 1] に写して scale 倍"""
    return [((k * 7919 + salt) % 2001 - 1000) / 1000.0 * scale for k in range(n)]


def model_gemm_relu(dims, np_dtype, tp_dtype):
    """Gemm(transB=1)+Relu を積んだ MLP。最後の層は活性化なし。重みは det_values で作る (CRC の固定用)。
    ReLU と積和だけなので、アクターと同じ演算順の参照計算とはビット一致するはず (数学関数を使わない)"""
    nodes, inits = [], []
    cur = "x"
    for k in range(len(dims) - 1):
        w = np.array(det_values(dims[k] * dims[k + 1], 11 * k + 3, 1.0 / math.sqrt(dims[k])),
                     dtype=np_dtype).reshape(dims[k + 1], dims[k])
        b = np.array(det_values(dims[k + 1], 13 * k + 5, 0.1), dtype=np_dtype)
        inits += [numpy_helper.from_array(w, f"W{k}"), numpy_helper.from_array(b, f"b{k}")]
        nodes.append(helper.make_node("Gemm", [cur, f"W{k}", f"b{k}"], [f"z{k}"], transB=1))
        cur = f"z{k}"
        if k < len(dims) - 2:
            nodes.append(helper.make_node("Relu", [cur], [f"h{k}"]))
            cur = f"h{k}"
    nodes.append(helper.make_node("Identity", [cur], ["y"]))
    g = helper.make_graph(
        nodes, "mlp_gemm_relu",
        [helper.make_tensor_value_info("x", tp_dtype, ["N", dims[0]])],
        [helper.make_tensor_value_info("y", tp_dtype, ["N", dims[-1]])],
        inits,
    )
    return helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)], ir_version=8)


# ReLU だけのモデル D の固定値 (issue #155)。.dynm の形式やアクターの演算が変わると、ここが合わなくなる。
# 変えるときは、変わった理由を PR に書いてから値を更新する
D_DIMS = [8, 32, 32, 4]
D_ROWS = 500
D_PINNED_CRC = {
    # (.dynm の種類): (.dynm のバイト列の CRC-32, アクターの出力 (f64 のリトルエンディアン) の CRC-32)
    "f64": (0x345CCE3D, 0x2B2E8227),
    "f32": (0x417984A5, 0x84688770),
}


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


# erg と同じ書き出し方のモデル E (issue #201)
E_DIMS = [67, 48, 48, 8]
E_ROWS = 300
E_PINNED_CRC = {
    # (ケース名): (.dynm のバイト列の CRC-32, アクターの出力 (f64 のリトルエンディアン) の CRC-32)
    "E_erg_plain_f64": (0x49AB78B2, 0x0BA422D0),
    "E_erg_norm_f64": (0x97C2CF10, 0xCC9319D1),
    "E_erg_norm_f32": (0xEDF89100, 0x3A617269),
}
TOL_F64 = 1e-12
TOL_F32 = 1e-5


def e_params(dims):
    """モデル E の決定的な値: (mean, std, [(W, b), ...])。W は [出力][入力] (PyTorch の Linear と同じ)"""
    mean = np.array(det_values(dims[0], 101, 3.0))
    std = np.array([0.5 + 0.5 * (v + 1.0) for v in det_values(dims[0], 103, 1.0)])  # [0.5, 1.5]
    lin = []
    for k in range(len(dims) - 1):
        w = np.array(det_values(dims[k] * dims[k + 1], 11 * k + 7, 1.0 / math.sqrt(dims[k]))).reshape(dims[k + 1], dims[k])
        b = np.array(det_values(dims[k + 1], 13 * k + 9, 0.1))
        lin.append((w, b))
    return mean, std, lin


def e_inputs(dims, rows):
    """モデル E の入力: 正規化の前の物理量らしい値 (mean ± 2 std 程度)"""
    mean, std, _ = e_params(dims)
    z = np.array(det_values(rows * dims[0], 107, 2.0)).reshape(rows, dims[0])
    return mean + std * z


def model_torch_like(dims, np_dtype, tp_dtype, norm):
    """torch.onnx.export(nn.Sequential(...), opset_version=17, dynamo=False) が出すのと同じグラフを作る。
    PyTorch 2.x の書き出しを手元で確かめた形: 入力名 "input"、形 [1, n] (dynamic_axes なし)、
    正規化は Sub("0.mean") → Div("0.std")、Linear は Gemm(alpha=1, beta=1, transB=1, 重み "k.weight" / "k.bias")、
    ReLU は Relu。最後の Gemm の出力がそのままグラフの出力になる (Identity なし)"""
    mean, std, lin = e_params(dims)
    nodes, inits = [], []
    cur = "input"
    m = 0  # nn.Sequential の中の番号
    if norm:
        inits += [numpy_helper.from_array(mean.astype(np_dtype), "0.mean"),
                  numpy_helper.from_array(std.astype(np_dtype), "0.std")]
        nodes.append(helper.make_node("Sub", [cur, "0.mean"], ["/0/Sub_output_0"], name="/0/Sub"))
        nodes.append(helper.make_node("Div", ["/0/Sub_output_0", "0.std"], ["/0/Div_output_0"], name="/0/Div"))
        cur = "/0/Div_output_0"
        m = 1
    for k, (w, b) in enumerate(lin):
        inits += [numpy_helper.from_array(w.astype(np_dtype), f"{m}.weight"),
                  numpy_helper.from_array(b.astype(np_dtype), f"{m}.bias")]
        last = k == len(lin) - 1
        out = "output" if last else f"/{m}/Gemm_output_0"
        nodes.append(helper.make_node("Gemm", [cur, f"{m}.weight", f"{m}.bias"], [out], name=f"/{m}/Gemm",
                                      alpha=1.0, beta=1.0, transB=1))
        cur = out
        m += 1
        if not last:
            nodes.append(helper.make_node("Relu", [cur], [f"/{m}/Relu_output_0"], name=f"/{m}/Relu"))
            cur = f"/{m}/Relu_output_0"
            m += 1
    g = helper.make_graph(
        nodes, "main_graph",
        [helper.make_tensor_value_info("input", tp_dtype, [1, dims[0]])],
        [helper.make_tensor_value_info("output", tp_dtype, [1, dims[-1]])],
        inits,
    )
    return helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)], ir_version=8)


def torch_export_like(dims, torch_dtype, norm, path):
    """同じ重みの nn.Sequential を torch.onnx.export で実際に書き出す (torch がなければ None)。
    返り値: 入力 [行][n] → 出力 [行][m] を計算する関数"""
    try:
        import torch
    except ImportError:
        return None
    mean, std, lin = e_params(dims)

    class Normalize(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.register_buffer("mean", torch.tensor(mean, dtype=torch_dtype))
            self.register_buffer("std", torch.tensor(std, dtype=torch_dtype))

        def forward(self, x):
            return (x - self.mean) / self.std

    mods = [Normalize()] if norm else []
    for k, (w, b) in enumerate(lin):
        L = torch.nn.Linear(dims[k], dims[k + 1]).to(torch_dtype)
        with torch.no_grad():
            L.weight.copy_(torch.tensor(w, dtype=torch_dtype))
            L.bias.copy_(torch.tensor(b, dtype=torch_dtype))
        mods.append(L)
        if k < len(lin) - 1:
            mods.append(torch.nn.ReLU())
    net = torch.nn.Sequential(*mods).eval()
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        torch.onnx.export(net, (torch.zeros(1, dims[0], dtype=torch_dtype),), path,
                          opset_version=17, dynamo=False)

    def run(x):
        with torch.no_grad():
            return net(torch.tensor(x, dtype=torch_dtype)).double().numpy()
    return run


def model_norm_variants():
    """正規化の書き方の変種 (モデル F)。5→6→3、Relu、f64"""
    rng = np.random.default_rng(3)
    n, h, o = 5, 6, 3
    k_sub = rng.normal(0, 1, (1, n))
    k_div = rng.uniform(0.5, 2.0, n)
    w0, b0 = rng.normal(0, 0.5, (h, n)), rng.normal(0, 0.1, h)
    w1, b1 = rng.normal(0, 0.5, (o, h)), rng.normal(0, 0.1, o)
    inits = [numpy_helper.from_array(a, nm) for a, nm in (
        (np.array([1, -1], np.int64), "shape"), (k_sub, "k_sub"), (k_div, "k_div"), (np.array(0.25), "k_add"),
        (np.array(1.5), "k_rsub"), (w0, "W0"), (b0, "b0"), (w1, "W1"), (b1, "b1"))]
    nodes = [
        helper.make_node("Flatten", ["x"], ["f"], axis=1),
        helper.make_node("Reshape", ["f", "shape"], ["r"]),
        helper.make_node("Constant", [], ["k_mul"], value=numpy_helper.from_array(np.array(0.75), "k_mul")),
        helper.make_node("Mul", ["k_mul", "r"], ["s1"]),
        helper.make_node("Sub", ["s1", "k_sub"], ["s2"]),
        helper.make_node("Add", ["s2", "k_add"], ["s3"]),
        helper.make_node("Sub", ["k_rsub", "s3"], ["s4"]),
        helper.make_node("Div", ["s4", "k_div"], ["s5"]),
        helper.make_node("Gemm", ["s5", "W0", "b0"], ["z0"], transB=1),
        helper.make_node("Relu", ["z0"], ["h0"]),
        helper.make_node("Gemm", ["h0", "W1", "b1"], ["y"], transB=1),
    ]
    g = helper.make_graph(
        nodes, "norm_variants",
        [helper.make_tensor_value_info("x", TensorProto.DOUBLE, [1, n])],  # 1行 (Reshape [1, -1] のため)
        [helper.make_tensor_value_info("y", TensorProto.DOUBLE, [1, o])],
        inits,
    )
    return helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)], ir_version=8)


def model_reject(kind):
    """畳み込めない形 (モデル G)。2→3→1、f64"""
    n, h = 2, 3
    inits = [numpy_helper.from_array(a, nm) for a, nm in (
        (np.array([2.0, 4.0]), "k"), (np.array([2, -1], np.int64), "shape"),
        (np.ones((h, n)), "W0"), (np.zeros(h), "b0"), (np.ones((1, h)), "W1"), (np.zeros(1), "b1"))]
    if kind == "const_div_x":
        pre = [helper.make_node("Div", ["k", "x"], ["p"])]
        mid = []
    elif kind == "sub_after_layer":
        pre = [helper.make_node("Identity", ["x"], ["p"])]
        mid = [helper.make_node("Sub", ["h0", "b0"], ["h0s"])]
    else:  # reshape_two_rows
        pre = [helper.make_node("Reshape", ["x", "shape"], ["p"])]
        mid = []
    h_in = "h0s" if mid else "h0"
    nodes = pre + [helper.make_node("Gemm", ["p", "W0", "b0"], ["z0"], transB=1),
                   helper.make_node("Relu", ["z0"], ["h0"])] + mid + [
        helper.make_node("Gemm", [h_in, "W1", "b1"], ["y"], transB=1)]
    g = helper.make_graph(
        nodes, f"reject_{kind}",
        [helper.make_tensor_value_info("x", TensorProto.DOUBLE, ["N", n])],
        [helper.make_tensor_value_info("y", TensorProto.DOUBLE, ["N", 1])],
        inits,
    )
    return helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)], ir_version=8)


def rel_err(y, y_ref):
    """issue #201 の相対誤差: max|y - y_ref| / max(1, max|y_ref|)"""
    return float(np.max(np.abs(y - y_ref))) / max(1.0, float(np.max(np.abs(y_ref))))


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
    in_name = sess.get_inputs()[0].name
    d0 = model.graph.input[0].type.tensor_type.shape.dim[0]
    if d0.HasField("dim_value") and d0.dim_value == 1:
        # 入力の形が [1, n] に固定されたモデル (PyTorch の書き出しの既定) は1行ずつ流す
        y_ort = np.vstack([sess.run(None, {in_name: x[i:i + 1].astype(in_dt)})[0] for i in range(len(x))])
    else:
        y_ort = sess.run(None, {in_name: x.astype(in_dt)})[0]
    y_ort = y_ort.astype(np.float64)
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
    if layers[0].folded:
        log(f"    入力の正規化 {' → '.join(layers[0].folded)} を最初の層に畳み込み。"
            f"元の網 (onnxruntime) との相対誤差 {rel_err(y_wasm, y_ort):.3e}")
    return d_ort, d_ref, y_wasm, n_bit, data, y_ort


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
    mD = model_gemm_relu(D_DIMS, np.float64, TensorProto.DOUBLE)
    xD = np.array(det_values(D_ROWS * D_DIMS[0], 17, 2.0), dtype=np.float64).reshape(D_ROWS, D_DIMS[0])
    e_models = {
        "E_erg_plain_f64": (model_torch_like(E_DIMS, np.float64, TensorProto.DOUBLE, False), "f64", TOL_F64),
        "E_erg_norm_f64": (model_torch_like(E_DIMS, np.float64, TensorProto.DOUBLE, True), "f64", TOL_F64),
        "E_erg_norm_f32": (model_torch_like(E_DIMS, np.float32, TensorProto.FLOAT, True), "auto", TOL_F32),
    }
    xE = e_inputs(E_DIMS, E_ROWS)
    mF = model_norm_variants()
    xF = np.random.default_rng(4).uniform(-3, 3, (200, 5))

    for label, wpath in (("フル版", a.wasm), ("縮小版", a.wasm_small)):
        if not os.path.exists(wpath):
            log(f"{label}: {wpath} がありません (ビルドしてください)。飛ばします")
            continue
        actor = Actor(wpath)
        log(f"=== {label}: {wpath}")
        log(f"    WASM {actor.wasm_bytes} バイト、線形メモリ {actor.pages()} ページ "
            f"({actor.pages() * 64} KB)、パラメータ上限 {actor.call('dyne_param_max')}、"
            f"読み込み領域 {actor.call('dyne_load_cap')} バイト")
        d1, r1, y1, _, _, _ = run_case("A_gemm_tanh", mA64, x2, "f64", actor, workdir, log)
        d2, r2, y2, _, _, _ = run_case("A_gemm_tanh", mA64, x2, "f32", actor, workdir, log)
        d3, r3, y3, _, _, _ = run_case("B_matmul_mixed", mB, x3, "auto", actor, workdir, log)
        # D: ReLU だけのモデル。同じ演算順の参照計算と全件ビット一致し、CRC-32 が固定値と合うこと (issue #155)
        yd = []
        for kind in ("f64", "f32"):
            dd, rd, y4, nb, dyn, _ = run_case("D_gemm_relu", mD, xD, kind, actor, workdir, log)
            crc_file = zlib.crc32(dyn) & 0xFFFFFFFF
            crc_out = zlib.crc32(np.ascontiguousarray(y4, dtype="<f8").tobytes()) & 0xFFFFFFFF
            pinned = D_PINNED_CRC[kind]
            match = (crc_file, crc_out) == pinned
            log(f"    全件ビット一致: {'はい' if nb == y4.size else 'いいえ'}  "
                f"CRC-32 .dynm {crc_file:08x} 出力 {crc_out:08x}  "
                f"(固定値 {'と一致' if match else 'と不一致: ' + str(tuple(f'{c:08x}' if c is not None else None for c in pinned))})")
            ok &= nb == y4.size and rd == 0.0 and dd < (1e-12 if kind == "f64" else 1e-5) and match
            yd.append(y4.ravel())
        # E: erg と同じ書き出し方 (issue #201)。畳み込んだ網の参照計算と全件ビット一致、CRC-32 が固定値、
        # 元の網 (onnxruntime) との相対誤差が許容誤差以内
        # (パラメータ 6008 個は縮小版の上限 2048 を超えるので、縮小版では読み込みが拒否されることだけを確かめる。
        #  出力はフル版と縮小版の比較に入れない)
        for name, (mdl, kind, tol) in e_models.items():
            path = os.path.join(workdir, f"{name}.onnx")
            onnx.save(mdl, path)
            dyn, _, _, n_params, _ = onnx_import.convert(path, kind)
            if n_params > actor.call("dyne_param_max"):
                r = actor.load(dyn)
                log(f"[{name}] パラメータ {n_params} は上限 {actor.call('dyne_param_max')} を超える。"
                    f"読み込み: {'拒否 (読み込み領域より大きい)' if r is None else f'戻り値 {r}'}")
                ok &= r is None or r < 0
                continue
            de, re_, y5, nb, dyn, y_ort = run_case(name, mdl, xE, kind, actor, workdir, log)
            crc_file = zlib.crc32(dyn) & 0xFFFFFFFF
            crc_out = zlib.crc32(np.ascontiguousarray(y5, dtype="<f8").tobytes()) & 0xFFFFFFFF
            pinned = E_PINNED_CRC[name]
            match = (crc_file, crc_out) == pinned
            rel = rel_err(y5, y_ort)
            log(f"    全件ビット一致: {'はい' if nb == y5.size else 'いいえ'}  "
                f"CRC-32 .dynm {crc_file:08x} 出力 {crc_out:08x}  "
                f"(固定値 {'と一致' if match else 'と不一致: ' + str(tuple(f'{c:08x}' if c is not None else None for c in pinned))})  "
                f"元の網との相対誤差 {rel:.3e} (許容 {tol:g})")
            ok &= nb == y5.size and re_ == 0.0 and rel <= tol and match
        # F: 正規化の書き方の変種。参照計算と全件ビット一致、元の網との相対誤差 1e-12 以内
        df, rf, y6, nb, _, y_ort = run_case("F_norm_variants", mF, xF, "f64", actor, workdir, log)
        log(f"    全件ビット一致: {'はい' if nb == y6.size else 'いいえ'}  "
            f"元の網との相対誤差 {rel_err(y6, y_ort):.3e} (許容 {TOL_F64:g})")
        ok &= nb == y6.size and rf == 0.0 and rel_err(y6, y_ort) <= TOL_F64
        yd.append(y6.ravel())
        outs.append((label, np.concatenate([y1.ravel(), y2.ravel(), y3.ravel()] + yd)))
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

    # G: 畳み込めない形の拒否 (issue #201)
    for kind in ("const_div_x", "sub_after_layer", "reshape_two_rows"):
        path = os.path.join(workdir, f"G_{kind}.onnx")
        onnx.save(model_reject(kind), path)
        try:
            onnx_import.convert(path)
            log(f"[G_{kind}] 変換されてしまった (不合格)")
            ok = False
        except onnx_import.ImportError_ as e:
            log(f"[G_{kind}] 期待どおり拒否: {e}")

    # E を PyTorch で実際に書き出したもの (torch があるときだけ。CI にはないので飛ばす)
    for name, (mdl, kind, tol) in e_models.items():
        tdt = None
        try:
            import torch
            tdt = torch.float32 if name.endswith("f32") else torch.float64
        except ImportError:
            pass
        if tdt is None:
            log(f"[{name} / torch] torch がないので、PyTorch で実際に書き出す照合は飛ばします")
            break
        tpath = os.path.join(workdir, f"{name}_torch.onnx")
        run = torch_export_like(E_DIMS, tdt, "norm" in name, tpath)
        mine, *_ = onnx_import.convert(os.path.join(workdir, f"{name}.onnx"), kind)
        theirs, _, t_layers, _, t_f32 = onnx_import.convert(tpath, kind)
        x_t = xE.astype(np.float32 if t_f32 else np.float64)
        y_t = run(x_t)
        y_ref = np.array([ref_eval(t_layers, row, t_f32) for row in x_t.astype(np.float64)])
        rel = rel_err(y_ref, y_t)
        same = mine == theirs
        log(f"[{name} / torch] torch.onnx.export の .dynm が onnx.helper で作ったものと"
            f"{'バイト列まで一致' if same else '不一致'}、PyTorch の出力との相対誤差 {rel:.3e} (許容 {tol:g})")
        ok &= same and rel <= tol

    log(f"作業ディレクトリ: {workdir}")
    log("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

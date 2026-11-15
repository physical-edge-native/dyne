"""ONNX のモデルを、マイコン上の WASMアクター (crates/mlp-actor) が読む
「計算グラフ+重み」の2進ファイル (.dynm) に変換する。

ONNX (NNの計算グラフと重みの共通交換形式) は PyTorch 等から書き出せる。
変換後のファイルだけをマイコンに渡せば、ONNX の実行環境なしで推論できる。

対応する形 (今年度の範囲): 全結合層と要素ごとの活性化関数を一直線に積んだ
多層パーセプトロン (MLP)。
  - 全結合層: Gemm (transA=0。transB は 0/1 どちらも可、alpha/beta は重みに畳み込む)、
    または MatMul (+ 直後の Add でバイアス)
  - 活性化: Tanh / Relu / Sigmoid (全結合層の直後に1つまで)
  - Identity は素通しとして扱う。Constant は重みの供給元として扱う
それ以外の演算を含むモデルは、未対応の演算の一覧を示して拒否する。

.dynm 形式 第1版 (リトルエンディアン) の定義は crates/mlp-core/src/lib.rs の冒頭と同じ:
  識別子 b"DYNM" / 版 u16=1 / フラグ u16 (bit0=重みがf32) / 入力次元 u32 /
  層数 u32 / パラメータ総数 u32 / 予約 u32=0 /
  層ごとに (演算 u32=1, 入力次元 u32, 出力次元 u32, 活性化 u32) /
  層ごとに 重み W[出力][入力] (行優先) → バイアス b[出力]

使い方:
  .venv\\Scripts\\python.exe scripts\\onnx_import.py model.onnx -o model.dynm [--dtype auto|f64|f32]
"""

import argparse
import struct
import sys
from collections import Counter

import numpy as np

MAGIC = b"DYNM"
VERSION = 1
FLAG_F32 = 1
OP_DENSE = 1
ACT_CODES = {"none": 0, "Tanh": 1, "Relu": 2, "Sigmoid": 3}
ACT_NAMES = {v: k for k, v in ACT_CODES.items()}
SUPPORTED_OPS = {"Gemm", "MatMul", "Add", "Tanh", "Relu", "Sigmoid", "Identity", "Constant"}
# mlp-actor の上限 (フル版)。縮小版は PARAM_MAX=2048
PARAM_MAX_FULL = 8192
PARAM_MAX_SMALL = 2048
LAYER_MAX = 16
WIDTH_MAX = 256


class ImportError_(Exception):
    """変換できないモデル (理由は日本語の文で持つ)"""


class Layer:
    def __init__(self, w, b):
        self.w = np.asarray(w, dtype=np.float64)  # [出力][入力]
        self.b = np.asarray(b, dtype=np.float64).reshape(-1)  # [出力]
        self.act = "none"

    @property
    def n_in(self):
        return self.w.shape[1]

    @property
    def n_out(self):
        return self.w.shape[0]


def parse_onnx(model):
    """ONNX のモデルを全結合層の列に変換する。返り値: (入力次元, 層の列, 元の重みの型名)"""
    import onnx
    from onnx import numpy_helper

    g = model.graph
    # 未対応の演算を先にまとめて調べる
    bad = Counter(n.op_type for n in g.node if n.op_type not in SUPPORTED_OPS)
    if bad:
        lst = ", ".join(f"{op} ({c}個)" for op, c in sorted(bad.items()))
        raise ImportError_(
            "未対応の演算を含むため変換できません: " + lst
            + "\n  対応している演算: " + ", ".join(sorted(SUPPORTED_OPS))
        )

    inits = {t.name: numpy_helper.to_array(t) for t in g.initializer}
    for n in g.node:
        if n.op_type == "Constant":
            for a in n.attribute:
                if a.name == "value":
                    inits[n.output[0]] = numpy_helper.to_array(a.t)
                else:
                    raise ImportError_(f"Constant の属性 {a.name} は未対応です")
    dtypes = {str(v.dtype) for v in inits.values()}

    inputs = [i for i in g.input if i.name not in inits]
    if len(inputs) != 1 or len(g.output) != 1:
        raise ImportError_(
            f"入力と出力はそれぞれ1つに限ります (入力 {len(inputs)} 個、出力 {len(g.output)} 個)"
        )
    cur = inputs[0].name
    out_name = g.output[0].name

    consumers = {}
    for n in g.node:
        if n.op_type == "Constant":
            continue
        for x in n.input:
            if x and x not in inits:
                consumers.setdefault(x, []).append(n)

    def attr(n, name, default):
        for a in n.attribute:
            if a.name == name:
                return onnx.helper.get_attribute_value(a)
        return default

    def init_of(n, name):
        if name not in inits:
            raise ImportError_(f"{n.op_type} ({n.name}) の {name} が定数 (重み) ではありません")
        return inits[name].astype(np.float64)

    layers = []
    used = 0
    n_nodes = sum(1 for n in g.node if n.op_type != "Constant")
    while cur != out_name:
        cs = consumers.get(cur, [])
        if len(cs) != 1:
            raise ImportError_(
                f"テンソル {cur} の使い手が {len(cs)} 個あります (一直線のグラフに限ります。分岐・合流は未対応)"
            )
        n = cs[0]
        used += 1
        op = n.op_type
        if op == "Gemm":
            if attr(n, "transA", 0) != 0:
                raise ImportError_("Gemm の transA=1 は未対応です")
            if n.input[0] != cur:
                raise ImportError_("Gemm の第1入力が前の層の出力ではありません")
            bm = init_of(n, n.input[1])
            w = bm if attr(n, "transB", 0) == 1 else bm.T
            w = w * float(attr(n, "alpha", 1.0))
            if len(n.input) > 2 and n.input[2]:
                c = init_of(n, n.input[2]) * float(attr(n, "beta", 1.0))
                b = np.broadcast_to(c.reshape(-1) if c.size > 1 else c.reshape(()), (w.shape[0],))
            else:
                b = np.zeros(w.shape[0])
            layers.append(Layer(w, b))
            cur = n.output[0]
        elif op == "MatMul":
            other = [x for x in n.input if x != cur]
            if len(other) != 1 or n.input[0] != cur:
                raise ImportError_("MatMul は (前の層の出力) × (重みの定数) の形に限ります")
            wm = init_of(n, other[0])
            if wm.ndim != 2:
                raise ImportError_(f"MatMul の重みは2次元に限ります (形 {wm.shape})")
            w = wm.T
            cur = n.output[0]
            b = np.zeros(w.shape[0])
            nxt = consumers.get(cur, [])
            if len(nxt) == 1 and nxt[0].op_type == "Add":
                a = nxt[0]
                other = [x for x in a.input if x != cur]
                if len(other) == 1 and other[0] in inits:
                    c = inits[other[0]].astype(np.float64)
                    if c.size not in (1, w.shape[0]):
                        raise ImportError_(f"Add のバイアスの形 {c.shape} が出力次元 {w.shape[0]} と合いません")
                    b = np.broadcast_to(c.reshape(-1), (w.shape[0],)).copy()
                    cur = a.output[0]
                    used += 1
            layers.append(Layer(w, b))
        elif op == "Add":
            raise ImportError_("MatMul の直後以外の Add (残差接続など) は未対応です")
        elif op in ("Tanh", "Relu", "Sigmoid"):
            if not layers:
                raise ImportError_(f"最初の全結合層より前の活性化 ({op}) は未対応です")
            if layers[-1].act != "none":
                raise ImportError_(f"活性化が2つ続いています ({layers[-1].act} → {op})。未対応です")
            layers[-1].act = op
            cur = n.output[0]
        elif op == "Identity":
            cur = n.output[0]
        else:
            raise ImportError_(f"演算 {op} は未対応です")
    if used != n_nodes:
        raise ImportError_(f"入力から出力への経路に乗らない演算が {n_nodes - used} 個あります")
    if not layers:
        raise ImportError_("全結合層がありません")

    n_in = layers[0].n_in
    for k in range(1, len(layers)):
        if layers[k].n_in != layers[k - 1].n_out:
            raise ImportError_(f"層 {k} の入力次元 {layers[k].n_in} が前の層の出力 {layers[k - 1].n_out} と合いません")
    # 入力の形 (最後の次元が決まっていれば照合する)
    dims = inputs[0].type.tensor_type.shape.dim
    if dims and dims[-1].HasField("dim_value") and dims[-1].dim_value != n_in:
        raise ImportError_(f"モデルの入力次元 {dims[-1].dim_value} と最初の層の入力 {n_in} が合いません")
    return n_in, layers, dtypes


def encode(n_in, layers, f32w):
    """層の列を .dynm のバイト列にする"""
    n_params = sum(l.w.size + l.b.size for l in layers)
    out = bytearray()
    out += MAGIC
    out += struct.pack("<HHIIII", VERSION, FLAG_F32 if f32w else 0, n_in, len(layers), n_params, 0)
    for l in layers:
        out += struct.pack("<IIII", OP_DENSE, l.n_in, l.n_out, ACT_CODES[l.act])
    dt = "<f4" if f32w else "<f8"
    for l in layers:
        out += np.ascontiguousarray(l.w, dtype=dt).tobytes()
        out += np.ascontiguousarray(l.b, dtype=dt).tobytes()
    return bytes(out), n_params


def check_limits(n_in, layers, n_params):
    """mlp-actor の上限との照合。警告文の列を返す (変換は止めない)"""
    msgs = []
    if len(layers) > LAYER_MAX:
        msgs.append(f"層数 {len(layers)} がアクターの上限 {LAYER_MAX} を超えています")
    w = max([n_in] + [l.n_out for l in layers])
    if w > WIDTH_MAX:
        msgs.append(f"層の幅 {w} がアクターの上限 {WIDTH_MAX} を超えています")
    if n_params > PARAM_MAX_FULL:
        msgs.append(f"パラメータ数 {n_params} がフル版の上限 {PARAM_MAX_FULL} を超えています")
    elif n_params > PARAM_MAX_SMALL:
        msgs.append(f"パラメータ数 {n_params} は縮小版 (実機向け) の上限 {PARAM_MAX_SMALL} を超えています (フル版なら可)")
    return msgs


def convert(onnx_path, dtype="auto"):
    """ONNX ファイルを読み、(バイト列, 入力次元, 層の列, パラメータ数, f32か) を返す"""
    import onnx

    model = onnx.load(onnx_path)
    n_in, layers, dtypes = parse_onnx(model)
    if dtype == "auto":
        f32w = dtypes <= {"float32", "float16"}
    else:
        f32w = dtype == "f32"
    data, n_params = encode(n_in, layers, f32w)
    return data, n_in, layers, n_params, f32w


def summary(n_in, layers, n_params, f32w, nbytes):
    lines = [f"入力次元 {n_in}、層数 {len(layers)}"]
    for k, l in enumerate(layers):
        lines.append(f"  層{k}: 全結合 {l.n_in}→{l.n_out}、活性化 {l.act}、パラメータ {l.w.size + l.b.size}")
    lines.append(f"パラメータ総数 {n_params}、重みの型 {'f32' if f32w else 'f64'}、ファイル {nbytes} バイト")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="ONNX → .dynm (mlp-actor 用の計算グラフ+重み)")
    ap.add_argument("onnx")
    ap.add_argument("-o", "--out", help="出力先 (省略時は拡張子を .dynm に替えた名前)")
    ap.add_argument("--dtype", choices=["auto", "f64", "f32"], default="auto",
                    help="重みの型。auto は元のモデルが float32 なら f32、それ以外は f64")
    a = ap.parse_args()
    out = a.out or (a.onnx.rsplit(".", 1)[0] + ".dynm")
    try:
        data, n_in, layers, n_params, f32w = convert(a.onnx, a.dtype)
    except ImportError_ as e:
        print(f"エラー: {e}", file=sys.stderr)
        sys.exit(2)
    with open(out, "wb") as f:
        f.write(data)
    print(summary(n_in, layers, n_params, f32w, len(data)))
    for m in check_limits(n_in, layers, n_params):
        print("警告: " + m)
    print(f"書き出し: {out}")


if __name__ == "__main__":
    main()

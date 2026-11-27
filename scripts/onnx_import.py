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
  - 入力の正規化 (issue #201): 最初の全結合層より前で、入力に定数を Sub / Add / Mul / Div する演算
    (例: PyTorch の (x - mean) / std) は、何個・どの順でも、最初の全結合層の重みとバイアスに畳み込む。
    定数は 1個 (スカラー) か入力次元と同じ個数 (形 [n] または [1, n])。定数 / x は線形でないので拒否する
  - 1行の Flatten (axis 0/1/-1) と Reshape (目標の形が [n]・[-1]・[1, n]・[1, -1]・[-1, n] など、
    最後以外の次元が 1 か -1 のもの) は、[1, n] の行に対しては何もしないので取り除く
それ以外の演算を含むモデルは、未対応の演算の一覧を示して拒否する。

畳み込みの演算順: 正規化の演算を出力側から順にたどり、Mul / Div は重みの列に掛ける (割る)、
Sub / Add は「その時点の重み × 定数」をバイアスから引く (足す)。バイアスの和は math.fsum (正しく丸めた和)
で取るので、BLAS や OS によらず同じビット列になる。浮動小数点の演算の順が元のグラフと変わるので、
畳み込んだ網の出力は元の網の出力とビット一致しない (照合の許容誤差は scripts/onnx_check.py に書く)。

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
SUPPORTED_OPS = {"Gemm", "MatMul", "Add", "Tanh", "Relu", "Sigmoid", "Identity", "Constant",
                 "Sub", "Mul", "Div", "Flatten", "Reshape"}
# 最初の全結合層より前でだけ受け付ける、入力の正規化の演算 (定数との要素ごとの演算)
NORM_OPS = ("Sub", "Add", "Mul", "Div")
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
        self.folded = []  # 畳み込んだ入力の正規化の演算名 (最初の層だけ。表示用)

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
    # 重みの型 (Reshape の目標の形のような整数の定数は数えない)
    dtypes = {str(v.dtype) for v in inits.values() if v.dtype.kind == "f"}

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
            raise ImportError_(f"{n.op_type} ({n.name or '名前なし'}) の {name} が定数 (重み) ではありません")
        return inits[name].astype(np.float64)

    layers = []
    pre = []  # 最初の全結合層より前の正規化: (演算, 定数, 定数が左側か, ノード名) の列
    shape_checks = []  # Reshape の目標の最後の次元: (その時点の層数, 次元, ノード名)
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
        elif op in NORM_OPS:
            if layers:
                raise ImportError_(
                    f"{op} ({n.name or '名前なし'}) は最初の全結合層より後にあります。定数との {op} は、"
                    "入力の正規化として最初の全結合層より前でだけ受け付けます (残差接続などは未対応)"
                )
            if len(n.input) != 2 or list(n.input).count(cur) != 1:
                raise ImportError_(f"{op} ({n.name or '名前なし'}) は入力と定数の演算に限ります")
            k_name = n.input[1] if n.input[0] == cur else n.input[0]
            if k_name not in inits:
                raise ImportError_(f"{op} ({n.name or '名前なし'}) の相手 {k_name} が定数ではありません")
            k = inits[k_name].astype(np.float64)
            if k.ndim > 2 or (k.ndim == 2 and k.shape[0] != 1):
                raise ImportError_(f"{op} ({n.name or '名前なし'}) の定数の形 {k.shape} は未対応です (スカラー、[n]、[1, n] に限ります)")
            left = n.input[0] == k_name
            if left and op == "Div":
                raise ImportError_(f"Div ({n.name or '名前なし'}) の 定数 / 入力 は線形でないので畳み込めません")
            pre.append((op, k.reshape(-1), left, n.name))
            cur = n.output[0]
        elif op == "Flatten":
            ax = attr(n, "axis", 1)
            if ax not in (0, 1, -1):
                raise ImportError_(f"Flatten ({n.name or '名前なし'}) の axis={ax} は未対応です (1行の Flatten に限ります)")
            cur = n.output[0]
        elif op == "Reshape":
            if attr(n, "allowzero", 0) != 0:
                raise ImportError_(f"Reshape ({n.name or '名前なし'}) の allowzero=1 は未対応です")
            if len(n.input) != 2 or n.input[0] != cur or n.input[1] not in inits:
                raise ImportError_(f"Reshape ({n.name or '名前なし'}) の目標の形が定数ではありません")
            shp = [int(v) for v in inits[n.input[1]].reshape(-1)]
            if not shp or any(v not in (1, -1) for v in shp[:-1]) or shp.count(-1) > 1 or shp[-1] == 0 or shp[-1] < -1:
                raise ImportError_(
                    f"Reshape ({n.name or '名前なし'}) の目標の形 {shp} は未対応です "
                    "(1行の Reshape に限ります。最後以外の次元は 1 か -1)"
                )
            if shp[-1] != -1:
                shape_checks.append((len(layers), shp[-1], n.name))
            cur = n.output[0]
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
    for k, d, name in shape_checks:
        width = n_in if k == 0 else layers[k - 1].n_out
        if d != width:
            raise ImportError_(f"Reshape ({name}) の目標の最後の次元 {d} がその位置の幅 {width} と合いません")
    if pre:
        fold_input_affine(layers[0], pre)
    # 入力の形 (最後の次元が決まっていれば照合する)
    dims = inputs[0].type.tensor_type.shape.dim
    if dims and dims[-1].HasField("dim_value") and dims[-1].dim_value != n_in:
        raise ImportError_(f"モデルの入力次元 {dims[-1].dim_value} と最初の層の入力 {n_in} が合いません")
    return n_in, layers, dtypes


def fold_input_affine(layer, pre):
    """入力の正規化 pre (入力に近い順) を最初の層 layer の重みとバイアスに畳み込む。

    出力側から順にたどる。y = W·t(x) + b で t が
      x - k: b ← b - W·k      x + k: b ← b + W·k     k - x: b ← b + W·k, W ← -W
      x · k: W ← W·diag(k)    x / k: W ← W·diag(1/k) (列ごとに k で割る。逆数を作らないので丸めは1回)
    バイアスの和は math.fsum で取る (正しく丸めた和。BLAS や OS によらず同じビット列)"""
    import math

    n = layer.n_in
    w = layer.w.copy()
    b = layer.b.copy()
    for op, k, left, name in reversed(pre):
        if k.size not in (1, n):
            raise ImportError_(f"{op} ({name}) の定数の個数 {k.size} が入力次元 {n} と合いません")
        kv = np.broadcast_to(k, (n,)).astype(np.float64)
        if op == "Mul":
            w = w * kv[None, :]
        elif op == "Div":
            if np.any(kv == 0.0):
                raise ImportError_(f"Div ({name}) の定数に 0 があります")
            w = w / kv[None, :]
        else:
            sign = -1.0 if (op == "Sub" and not left) else 1.0
            b = np.array([math.fsum([b[j]] + [sign * (w[j, i] * kv[i]) for i in range(n)])
                          for j in range(w.shape[0])])
            if op == "Sub" and left:
                w = -w
    if not (np.all(np.isfinite(w)) and np.all(np.isfinite(b))):
        raise ImportError_("入力の正規化を畳み込んだ重みに有限でない値があります")
    layer.w = w
    layer.b = b
    layer.folded = [op for op, *_ in pre]


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
        if l.folded:
            lines.append(f"    (入力の正規化 {' → '.join(l.folded)} をこの層の重みとバイアスに畳み込みました)")
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

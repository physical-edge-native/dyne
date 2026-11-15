"""2次元の初期場のモデルを PyTorch で学習し、ONNX → .dynm に変換して書き出す。

dyne-sim2d の --model (crates/heat2d-mlp-actor 用) に渡すモデルを作る:
  入力 (x, y) ∈ [0,1]² → 隠れ層2層・幅16・tanh → 出力 u
  目標 u(x, y) = sin(πx)·sin(πy) (heat2d-actor の初期条件と同じ関数)
学習は f64、L-BFGS (準ニュートン法の一種)。書き出した ONNX を scripts/onnx_import.py で
.dynm (f64) に変換し、PyTorch の出力と .dynm を読んだ推論専用アクター (crates/mlp-actor)
の出力の最大絶対誤差も報告する。

使い方:
  .venv\\Scripts\\python.exe scripts\\onnx_torch_field2d.py -o field2d.dynm [--onnx field2d.onnx]
"""

import argparse
import math
import os
import sys
import tempfile

try:
    import torch
except ImportError:
    print("torch が .venv にありません。2次元の初期場のモデルは作れないので飛ばします")
    sys.exit(2)

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import onnx_import  # noqa: E402
from onnx_check import Actor, WASM_FULL  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", required=True, help="書き出す .dynm")
    ap.add_argument("--onnx", default=None, help="ONNX の書き出し先 (省略時は一時ディレクトリ)")
    a = ap.parse_args()

    # 小さなNNなので1スレッドの方が速く、他の作業と並べても重くならない
    torch.set_num_threads(1)
    torch.manual_seed(20261108)
    torch.set_default_dtype(torch.float64)
    net = torch.nn.Sequential(
        torch.nn.Linear(2, 16), torch.nn.Tanh(),
        torch.nn.Linear(16, 16), torch.nn.Tanh(),
        torch.nn.Linear(16, 1),
    )
    g = torch.linspace(0.0, 1.0, 21)
    xx, yy = torch.meshgrid(g, g, indexing="ij")
    xy = torch.stack([xx.reshape(-1), yy.reshape(-1)], dim=1)
    target = (torch.sin(math.pi * xy[:, 0]) * torch.sin(math.pi * xy[:, 1])).reshape(-1, 1)
    opt = torch.optim.LBFGS(net.parameters(), lr=1.0, max_iter=800, tolerance_grad=1e-12,
                            tolerance_change=1e-15, history_size=50, line_search_fn="strong_wolfe")

    def closure():
        opt.zero_grad()
        l = ((net(xy) - target) ** 2).mean()
        l.backward()
        return l

    opt.step(closure)
    with torch.no_grad():
        loss = float(((net(xy) - target) ** 2).mean())
    print(f"[PyTorch 学習] 2→16→16→1・tanh、目標 sin(πx)·sin(πy) (21×21点)、平均二乗誤差 {loss:.3e}")

    onnx_path = a.onnx or os.path.join(tempfile.mkdtemp(prefix="field2d_"), "field2d.onnx")
    torch.onnx.export(net, torch.zeros(1, 2), onnx_path, input_names=["x"], output_names=["y"],
                      dynamic_axes={"x": {0: "N"}, "y": {0: "N"}}, opset_version=17, dynamo=False)
    data, n_in, layers, n_params, f32w = onnx_import.convert(onnx_path, "f64")
    with open(a.out, "wb") as f:
        f.write(data)
    print(f"[.dynm] {a.out}: パラメータ {n_params}、{len(data)} バイト")

    if os.path.exists(WASM_FULL):
        actor = Actor(WASM_FULL)
        assert actor.load(data) == 0
        pts = np.random.default_rng(1).uniform(0, 1, (500, 2))
        with torch.no_grad():
            y_t = net(torch.from_numpy(pts)).numpy()
        d = float(np.max(np.abs(actor.eval(pts) - y_t)))
        print(f"    PyTorch と WASM (mlp-actor) の最大絶対誤差 {d:.3e}")
        if d > 1e-13:
            print("FAIL")
            sys.exit(1)
    print("OK")


if __name__ == "__main__":
    main()

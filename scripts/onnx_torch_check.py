"""PyTorch で学習した PINN を ONNX で書き出し、.dynm 経由で WASMアクターが同じ値を出すかの照合。

PINN (物理情報ニューラルネットワーク、物理方程式を満たすよう学習するNN) の形は
crates/heat1d-pinn-actor の既定と同じにする:
  入力 ξ = 2x − 1 (区間 [0,1] を [−1,1] に写したもの) → 隠れ層1層・幅16・tanh → 出力 u
  (パラメータ 16+16+16+1 = 49 個)

解く問題は heat1d-pinn-actor の1ステップと同じ「時間を1刻み進める後退オイラー型の式」:
  u(x) − α·dt·u''(x) = u_prev(x) = sin(πx)、u(0) = u(1) = 0
この厳密解は u(x) = sin(πx) / (1 + α·dt·π²) なので、学習の出来も確かめられる。
u'' は PyTorch の自動微分で求め、L-BFGS (準ニュートン法の一種) で f64 のまま学習する。

照合:
  (1) PyTorch の出力 と .dynm(f64) を読んだ WASMアクター (crates/mlp-actor) の出力の最大絶対誤差
  (2) 同じく .dynm(f32、重みを単精度に丸めたもの) との最大絶対誤差
  (3) 学習した NN と厳密解の最大絶対誤差 (参考)
  (4) 学習用アクター (crates/mlp-train-actor) があれば、同じ .dynm を読み込ませて
      推論が (1) とビット一致すること、さらに別の目標 (α·dt を2倍にした厳密解) へ
      マイコン上で LM法 (レーベンバーグ・マルカート法) で追加学習して損失が下がることを確かめる

torch が入っていなければ、その旨を表示して終了コード 0 で抜ける (check-onnx.ps1 が飛ばす)。

使い方:
  .venv\\Scripts\\python.exe scripts\\onnx_torch_check.py [--out 作業ディレクトリ] [--keep-onnx 経路]
"""

import argparse
import math
import os
import sys
import tempfile

try:
    import torch
except ImportError:
    print("torch が .venv にありません。PyTorch 経路の照合は飛ばします")
    print("  入れる場合: .venv\\Scripts\\python.exe -m pip install torch --index-url https://download.pytorch.org/whl/cpu")
    sys.exit(0)

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import onnx_import  # noqa: E402
from onnx_check import Actor, WASM_FULL, WASM_SMALL, ROOT  # noqa: E402

WASM_TRAIN = os.path.join(ROOT, "crates", "mlp-train-actor", "target", "wasm32-unknown-unknown", "release", "mlp_train_actor.wasm")
WASM_TRAIN_SMALL = os.path.join(ROOT, "crates", "mlp-train-actor", "target", "small", "wasm32-unknown-unknown", "release", "mlp_train_actor.wasm")

ALPHA = 1.0
DT = 1e-2
WIDTH = 16


def exact(x, alpha_dt):
    return np.sin(np.pi * x) / (1.0 + alpha_dt * np.pi ** 2)


def train_pinn(log):
    torch.set_num_threads(1)  # 小さなNNなので1スレッドの方が速い
    torch.manual_seed(20261108)
    torch.set_default_dtype(torch.float64)
    net = torch.nn.Sequential(torch.nn.Linear(1, WIDTH), torch.nn.Tanh(), torch.nn.Linear(WIDTH, 1))
    # 残差評価点 (格子65点) と境界点
    x = torch.linspace(0.0, 1.0, 65).reshape(-1, 1)
    xb = torch.tensor([[0.0], [1.0]])
    u_prev = torch.sin(math.pi * x)

    def loss_fn():
        xr = x.clone().requires_grad_(True)
        u = net(2.0 * xr - 1.0)
        du = torch.autograd.grad(u.sum(), xr, create_graph=True)[0]
        d2u = torch.autograd.grad(du.sum(), xr, create_graph=True)[0]
        r = (u - u_prev) - ALPHA * DT * d2u
        rb = net(2.0 * xb - 1.0)
        return (r ** 2).mean() + (rb ** 2).mean()

    opt = torch.optim.LBFGS(net.parameters(), lr=1.0, max_iter=2000, tolerance_grad=1e-14,
                            tolerance_change=1e-16, history_size=50, line_search_fn="strong_wolfe")

    def closure():
        opt.zero_grad()
        l = loss_fn()
        l.backward()
        return l

    l0 = float(loss_fn().detach())
    opt.step(closure)
    l1 = float(loss_fn().detach())
    log(f"[PyTorch 学習] 幅{WIDTH}・tanh・1→16→1、L-BFGS、損失 {l0:.3e} → {l1:.3e}")
    return net


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None, help="作業ディレクトリ (省略時は一時ディレクトリ)")
    a = ap.parse_args()
    workdir = a.out or tempfile.mkdtemp(prefix="onnx_torch_")
    os.makedirs(workdir, exist_ok=True)
    log = print
    ok = True
    log(f"torch {torch.__version__}")

    net = train_pinn(log)
    onnx_path = os.path.join(workdir, "pinn_heat1d.onnx")
    dummy = torch.zeros(1, 1)
    # 旧来の書き出し器 (TorchScript 経由) を使う。可変長のバッチ次元をつける
    torch.onnx.export(net, dummy, onnx_path, input_names=["x"], output_names=["y"],
                      dynamic_axes={"x": {0: "N"}, "y": {0: "N"}}, opset_version=17, dynamo=False)
    import onnx
    ops = sorted({n.op_type for n in onnx.load(onnx_path).graph.node})
    log(f"[ONNX 書き出し] {onnx_path}  演算: {', '.join(ops)}")

    # 照合点: 格子点ではない点も含める
    xs = np.linspace(0.0, 1.0, 401)
    xi = (2.0 * xs - 1.0).reshape(-1, 1)
    with torch.no_grad():
        y_torch = net(torch.from_numpy(xi)).numpy().astype(np.float64)
    err_exact = float(np.max(np.abs(y_torch[:, 0] - exact(xs, ALPHA * DT))))
    log(f"    学習した NN と厳密解の最大絶対誤差 {err_exact:.3e}")
    ok &= err_exact < 1e-3

    dynm = {}
    for dt in ("f64", "f32"):
        data, n_in, layers, n_params, f32w = onnx_import.convert(onnx_path, dt)
        p = os.path.join(workdir, f"pinn_heat1d_{dt}.dynm")
        with open(p, "wb") as f:
            f.write(data)
        dynm[dt] = data
        log(f"[.dynm {dt}] パラメータ {n_params}、{len(data)} バイト、層 "
            + " → ".join(f"{l.n_in}x{l.n_out}({l.act})" for l in layers))

    y_ref_bits = None
    for label, wpath in (("フル版", WASM_FULL), ("縮小版", WASM_SMALL)):
        if not os.path.exists(wpath):
            log(f"{label}: {wpath} がありません。飛ばします")
            continue
        actor = Actor(wpath)
        for dt, tol in (("f64", 1e-13), ("f32", 1e-5)):
            r = actor.load(dynm[dt])
            if r != 0:
                log(f"{label} {dt}: dyne_model_load の戻り値 {r}")
                ok = False
                continue
            y = actor.eval(xi)
            d = float(np.max(np.abs(y - y_torch)))
            log(f"[WASM {label} / .dynm {dt}] PyTorch との最大絶対誤差 {d:.3e} (合格基準 < {tol:g})")
            ok &= d < tol
            if dt == "f64":
                if y_ref_bits is None:
                    y_ref_bits = y.copy()
                else:
                    same = int(np.sum(y.view(np.uint64) == y_ref_bits.view(np.uint64)))
                    log(f"    フル版とのビット一致 {same}/{y.size}")
                    ok &= same == y.size

    # ---- 学習用アクターでの追加学習 (あれば) ----
    for label, wpath in (("学習用フル版", WASM_TRAIN), ("学習用縮小版", WASM_TRAIN_SMALL)):
        if not os.path.exists(wpath):
            log(f"{label}: {wpath} がありません。追加学習の確認は飛ばします")
            continue
        t = Actor(wpath)
        log(f"=== {label}: WASM {t.wasm_bytes} バイト、線形メモリ {t.pages()} ページ ({t.pages() * 64} KB)、"
            f"LM法のパラメータ上限 {t.call('dyne_train_param_max')}")
        r = t.load(dynm["f64"])
        if r != 0:
            log(f"    dyne_model_load の戻り値 {r}")
            ok = False
            continue
        y0 = t.eval(xi)
        if y_ref_bits is not None:
            same = int(np.sum(y0.view(np.uint64) == y_ref_bits.view(np.uint64)))
            log(f"    読み込み直後の推論: 推論専用アクターとのビット一致 {same}/{y0.size}")
            ok &= same == y0.size
        # 目標: α·dt を2倍にした厳密解 (格子33点)。ホストが入力と目標を書き込む
        xt = np.linspace(0.0, 1.0, 33)
        tgt = exact(xt, 2.0 * ALPHA * DT)
        t.mem.write(t.store, np.ascontiguousarray(2.0 * xt - 1.0, dtype="<f8").tobytes(), t.call("dyne_train_x_buf"))
        t.mem.write(t.store, np.ascontiguousarray(tgt, dtype="<f8").tobytes(), t.call("dyne_train_t_buf"))
        rc = t.call("dyne_train_set", len(xt))
        l_before = t.call("dyne_train_loss")
        l_after = t.call("dyne_train_lm", 20)
        y1 = t.eval(xi)
        d_new = float(np.max(np.abs(y1[:, 0] - exact(xs, 2.0 * ALPHA * DT))))
        log(f"    追加学習 (LM法20反復、目標=α·dtを2倍にした厳密解 33点): 設定 {rc}、"
            f"損失 {l_before:.3e} → {l_after:.3e}、新しい厳密解との最大誤差 {d_new:.3e}")
        ok &= rc == 0 and l_after < l_before * 1e-2 and d_new < 1e-3

    log(f"作業ディレクトリ: {workdir}")
    log("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

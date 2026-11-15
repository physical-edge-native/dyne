# 2次元の波動方程式: クーラン数 ν を安定の限界をまたいで掃引し、dyne-sim2d(2×2ノード、
# wave2d-actor の WASM)で発散が始まる ν が理論(wave2d.py)の予測と一致するかを確かめる。
#
# 出所: Claude Code が 2026-11-11 に作成。記録は docs/2026-11-11-wave2d.md。
# 出力: theory/guarantee/out/wave2d_sweep.dat(表)と wave2d_sweep.png(図)
#
# 使い方(リポジトリ直下で):
#   .venv\Scripts\python.exe theory\guarantee\wave2d_sweep.py [dyne-sim2d.exe の経路]
# 条件: 41×41点(h = 1/40)、2×2ノード、損失0.1、2000ステップ、初期条件はガウス型の山
# (中心 (0.3, 0.6)、σ = 0.06、初速度0)。外周は値固定0・ゼロ勾配・吸収境界(4面)の3通り。
# 判定: 最後の max|u| が初期の最大値1を超えたら「発散」。発散した場合は、最後の200ステップの
# 1ステップあたりの増幅率を、--steps 1800 と 2000 の2回の実行の max|u| の比から求め、
# 理論のスペクトル半径と比べる。
import math
import os
import re
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wave2d import critical_nu_fast, rho_fast, vn_gmax  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
WASM = os.path.join(ROOT, 'crates', 'wave2d-actor', 'target', 'wasm32-unknown-unknown', 'release', 'wave2d_actor.wasm')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
N = 41
STEPS = 2000
NUS = [0.60, 0.65, 0.70, 0.705, 0.7071, 0.70712, 0.7072, 0.7074, 0.7076, 0.70765, 0.7077, 0.7078, 0.708, 0.709, 0.71, 0.72]
GAUSS = ['--cfg', '0=1', '--cfg', '4=0.3', '--cfg', '5=0.6', '--cfg', '6=0.06']
BCS = {
    'dirichlet': [],
    'zero_grad': ['--outer-bc', 'zero_grad'],
    'absorb': ['--outer-bc', 'zero_grad', '--cfg', '7=15'],
}
# 吸収境界の限界は行列の方法(wave2d.py の出力)。n=11, 21 とも 1/√2 に一致した
ABSORB_LIMIT = 1.0 / math.sqrt(2.0)


def run(exe, nu, bc, steps):
    dt = repr(nu / (N - 1))
    cmd = [exe, '--wasm', WASM, '--px', '2', '--py', '2', '--nx', str(N), '--ny', str(N), '--dt', dt,
           '--steps', str(steps), '--loss', '0.1', '--quiet'] + BCS[bc] + GAUSS
    out = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8').stdout
    m = re.search(r'max\|u\| = (\S+)', out)
    amax = float(m.group(1)) if m else float('nan')
    ok = 'PASS (bit-exact)' in out
    nu_act = float(re.search(r'courant nu = (\S+)', out).group(1))
    return amax, ok, nu_act


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    exe = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'target', 'release', 'dyne-sim2d.exe')
    os.makedirs(OUT, exist_ok=True)
    lim = {'dirichlet': critical_nu_fast(N, 'dirichlet'), 'zero_grad': critical_nu_fast(N, 'zero_grad'),
           'absorb': ABSORB_LIMIT}
    rows = []
    for bc in BCS:
        for nu in NUS:
            a2, ok2, nu_act = run(exe, nu, bc, STEPS)
            a1, ok1, _ = run(exe, nu, bc, STEPS - 200)
            rate = (a2 / a1) ** (1 / 200) if (np.isfinite(a2) and a2 > 1e3 and a1 > 0) else float('nan')
            rho = rho_fast(N, nu, bc) if bc != 'absorb' else float('nan')
            pred = 'unstable' if nu > lim[bc] else 'stable'
            meas = 'diverged' if not (a2 <= 1.0) else 'bounded'
            rows.append((bc, nu, nu_act, a2, rate, rho, vn_gmax(nu), pred, meas, ok1 and ok2))
            print(f'{bc:<9s} nu={nu:<8} max|u|={a2:.3e} rate={rate:.6f} rho_theory={rho:.6f} '
                  f'pred={pred:<8s} meas={meas:<8s} bit-exact={ok1 and ok2}', flush=True)
    with open(os.path.join(OUT, 'wave2d_sweep.dat'), 'w', encoding='utf-8') as f:
        f.write('# 2次元の波動方程式: クーラン数の掃引 (dyne-sim2d, wave2d-actor, 41x41, 2x2ノード, 損失0.1, '
                f'{STEPS}ステップ, ガウス型の山)\n')
        f.write(f'# 理論の限界: 値固定 {lim["dirichlet"]:.8f} / ゼロ勾配 {lim["zero_grad"]:.8f} / '
                f'吸収 {lim["absorb"]:.8f} (無限格子 1/sqrt2 = {1/math.sqrt(2):.8f})\n')
        f.write('# bc nu nu_actual max|u|_final measured_rate rho_bounded(theory) g_max_infinite(theory) '
                'predicted measured bit_exact\n')
        for r in rows:
            f.write(f'{r[0]} {r[1]} {r[2]:.8f} {r[3]:.6e} {r[4]:.6f} {r[5]:.6f} {r[6]:.6f} {r[7]} {r[8]} {r[9]}\n')
    agree = all((r[7] == 'unstable') == (r[8] == 'diverged') for r in rows)
    print('予測と実測の一致:', agree)

    plot(lim)


def plot(lim=None):
    """out/wave2d_sweep.dat から図を作る (--plot-only でシミュレータを動かさずに作り直せる)。"""
    if lim is None:
        lim = {'dirichlet': critical_nu_fast(N, 'dirichlet'), 'zero_grad': critical_nu_fast(N, 'zero_grad'),
               'absorb': ABSORB_LIMIT}
    rows = []
    with open(os.path.join(OUT, 'wave2d_sweep.dat'), encoding='utf-8') as f:
        for line in f:
            if line.startswith('#'):
                continue
            t = line.split()
            rows.append((t[0], float(t[1]), float(t[3])))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2))
    colors = {'dirichlet': '#1f77b4', 'zero_grad': '#d62728', 'absorb': '#2ca02c'}
    labels = {'dirichlet': 'fixed value 0', 'zero_grad': 'zero gradient', 'absorb': 'absorbing (Mur)'}
    for ax, (x0, x1) in zip(axes, ((0.595, 0.725), (0.7068, 0.7080))):
        for bc in BCS:
            xs = [r[1] for r in rows if r[0] == bc]
            ys = [min(math.log10(r[2]), 300.0) if np.isfinite(r[2]) else 300.0 for r in rows if r[0] == bc]
            ax.plot(xs, ys, 'o-', color=colors[bc], label=labels[bc], ms=4)
            ax.axvline(lim[bc], color=colors[bc], ls=':', lw=1.2)
        ax.axvline(1 / math.sqrt(2), color='k', ls='--', lw=1, label='von Neumann limit 1/sqrt(2)')
        ax.set_xlim(x0, x1)
        ax.set_ylim(-8, 310)
        ax.set_xlabel('Courant number  c dt / h')
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel(f'log10 max|u| after {STEPS} steps (NaN drawn at 300)')
    axes[1].set_ylim(-8, 80)
    axes[0].set_title('whole sweep')
    axes[1].set_title('near the limit (dotted: bounded-domain limit)')
    axes[0].legend(fontsize=8, loc='upper left')
    fig.suptitle('2D wave equation, 41x41 grid, 2x2 nodes, dyne-sim2d')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'wave2d_sweep.png'), dpi=120)

if __name__ == '__main__':
    if '--plot-only' in sys.argv:
        plot()
    else:
        main()

# 2次元の波動方程式の陽解法(時間の中心差分+5点ラプラシアン、crates/wave2d-actor)の安定性。
#
# 出所: Claude Code が 2026-11-11 に作成。記録は docs/2026-11-11-wave2d.md。
# 2つの方法で安定の限界(クーラン数 ν = c·Δt/h の上限)を求める。
#
# (1) フォン・ノイマンの方法(一様で無限に広い格子。境界を考えない)
#     u_{j,k}^n = g^n·exp(i(jξ + kη)) を代入すると、増幅率 g は
#         g² − 2b·g + 1 = 0、 b = 1 − 2ν²(sin²(ξ/2) + sin²(η/2))
#     を満たす。2根の積は1なので、|b| ≤ 1 なら |g| = 1(振幅を保つ)、|b| > 1 なら片方の根が
#     |g| > 1。b の最小は ξ = η = π で 1 − 4ν²。よって安定 ⇔ 1 − 4ν² ≥ −1 ⇔ ν ≤ 1/√2。
#     ちょうど ν = 1/√2 では ξ = η = π で g = −1 の重根になり、振幅が時間に比例して増える
#     (弱い不安定)。
#
# (2) 有界な領域の行列の方法(実際の更新式・境界の扱いそのもの)
#     状態 (u^n, u^{n−1}) から (u^{n+1}, u^n) への1ステップの写像(2つの時刻を並べた
#     「コンパニオン形式」の行列)を、アクターと同じ式・同じ境界処理を単位ベクトルに
#     当てて組み立て、その固有値の絶対値の最大(スペクトル半径)が1を超えるかで判定する。
#     値固定とゼロ勾配では、空間の作用素の固有値 λ(5点ラプラシアン、1/h² を除いた形)から
#     b = 1 + ν²λ/2 となるので、限界は ν* = 2/√|λ|max でも求まる(速い方法。両者を照合する)。
#     吸収境界(ムーアの方法)は対称性がないので、行列の方法だけで求める。
#
# 判定の許容: スペクトル半径が 1 + 1e-6 を超えたら不安定とみなす。値固定以外では一定値の
# モード(λ = 0)が g = 1 の重根(ジョルダン細胞)になり、数値計算した固有値が √(丸め誤差)
# ≈ 1e-8 程度ずれるため、theory/guarantee/checker.py の 1e-9 より緩くしている。
# 限界のすぐ上では ρ − 1 が √(ν − ν*) に比例して立ち上がるので、ν の分解能はほぼ落ちない。
#
# 使い方(リポジトリ直下で):
#   .venv\Scripts\python.exe theory\guarantee\wave2d.py           … 表を出す
import math
import sys

import numpy as np

STAB_TOL = 1e-6
BCS = ('dirichlet', 'zero_grad', 'absorb')


# ---------------------------------------------------------------- (1) フォン・ノイマンの方法
def vn_gmax(nu, nk=401):
    """無限格子での増幅率の絶対値の最大(ξ, η ∈ [0, π] の格子で探す。π を含む)。"""
    xi = np.linspace(0.0, math.pi, nk)
    s = np.sin(xi / 2) ** 2
    b = 1.0 - 2.0 * nu * nu * (s[:, None] + s[None, :])
    # |b| ≤ 1 なら |g| = 1、|b| > 1 なら |g| = |b| + √(b² − 1)
    ab = np.abs(b)
    g = np.where(ab <= 1.0, 1.0, ab + np.sqrt(np.maximum(b * b - 1.0, 0.0)))
    return float(g.max())


def vn_limit():
    """フォン・ノイマンの方法の限界(解析値 1/√2)と、vn_gmax の二分法による数値確認。"""
    lo, hi = 0.5, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if vn_gmax(mid) > 1.0 + 1e-12:
            hi = mid
        else:
            lo = mid
    return 1.0 / math.sqrt(2.0), lo


# ---------------------------------------------------------------- (2) 有界な領域の行列の方法
def actor_step(u, p, nu, bc, first=False):
    """wave2d-actor の dyne_step と同じ式・同じ順序の1ステップ(全域1ノード、外周4面とも bc)。
    u = u^n、p = u^{n−1}(n×n、添字 [j, i])。値固定の値は0(誤差の写像なので定数項は落とす)。
    戻り値 u^{n+1}。"""
    n = u.shape[0]
    nu2 = nu * nu
    pad = np.zeros((n + 2, n + 2))
    pad[1:-1, 1:-1] = u
    if bc in ('zero_grad', 'absorb'):
        # 面の順 W, E, S, N で枠を埋める(枠の角は使わない)
        pad[1:-1, 0] = u[:, 1]
        pad[1:-1, -1] = u[:, -2]
        pad[0, 1:-1] = u[1, :]
        pad[-1, 1:-1] = u[-2, :]
    c = pad[1:-1, 1:-1]
    lap = (pad[1:-1, :-2] - 2.0 * c + pad[1:-1, 2:]) + (pad[:-2, 1:-1] - 2.0 * c + pad[2:, 1:-1])
    if first:
        new = c + 0.5 * nu2 * lap
    else:
        new = 2.0 * c - p + nu2 * lap
    if bc == 'absorb':
        k = (nu - 1.0) / (nu + 1.0)
        # W, E, S, N の順に端の点を上書き(角は後の面が勝つ)
        new[:, 0] = u[:, 1] + k * (new[:, 1] - u[:, 0])
        new[:, -1] = u[:, -2] + k * (new[:, -2] - u[:, -1])
        new[0, :] = u[1, :] + k * (new[1, :] - u[0, :])
        new[-1, :] = u[-2, :] + k * (new[-2, :] - u[-1, :])
    elif bc == 'dirichlet':
        new[:, 0] = 0.0
        new[:, -1] = 0.0
        new[0, :] = 0.0
        new[-1, :] = 0.0
    return new


def companion_matrix(n, nu, bc):
    """(u^n, u^{n−1}) → (u^{n+1}, u^n) の 2n²×2n² 行列(一般のステップ。最初の1ステップは除く)。
    値固定では外周の点を状態から除く(常に0なので)。"""
    N = n * n
    idx = np.arange(N)
    if bc == 'dirichlet':
        jj, ii = np.divmod(idx, n)
        keep = idx[(ii > 0) & (ii < n - 1) & (jj > 0) & (jj < n - 1)]
    else:
        keep = idx
    m = len(keep)
    M = np.zeros((2 * m, 2 * m))
    for col in range(2 * m):
        u = np.zeros(N)
        p = np.zeros(N)
        if col < m:
            u[keep[col]] = 1.0
        else:
            p[keep[col - m]] = 1.0
        new = actor_step(u.reshape(n, n), p.reshape(n, n), nu, bc).ravel()
        M[:m, col] = new[keep]
        M[m:, col] = u[keep]
    return M


def spectral_radius(n, nu, bc):
    return float(np.max(np.abs(np.linalg.eigvals(companion_matrix(n, nu, bc)))))


def critical_nu_matrix(n, bc, lo=0.6, hi=0.8, iters=30):
    """行列の方法の二分法で、スペクトル半径が 1 + STAB_TOL を超え始める ν を探す。"""
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if spectral_radius(n, mid, bc) > 1.0 + STAB_TOL:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def laplacian_eigs(n, bc):
    """空間の作用素(5点ラプラシアン×h²、外周の扱い込み)の固有値。値固定・ゼロ勾配のみ。"""
    if bc == 'dirichlet':
        k = np.arange(1, n - 1)
        lam1 = -4.0 * np.sin(k * math.pi / (2 * (n - 1))) ** 2
    elif bc == 'zero_grad':
        k = np.arange(0, n)
        lam1 = -4.0 * np.sin(k * math.pi / (2 * (n - 1))) ** 2
    else:
        raise ValueError(bc)
    return (lam1[:, None] + lam1[None, :]).ravel()


def critical_nu_fast(n, bc):
    """値固定・ゼロ勾配の限界 ν* = 2/√|λ|max(b = 1 + ν²λ/2 ≥ −1)。"""
    return 2.0 / math.sqrt(float(np.max(np.abs(laplacian_eigs(n, bc)))))


def rho_fast(n, nu, bc):
    """値固定・ゼロ勾配のスペクトル半径(各固有値 λ ごとに g² − 2b g + 1 = 0 を解く)。"""
    b = 1.0 + 0.5 * nu * nu * laplacian_eigs(n, bc)
    ab = np.abs(b)
    g = np.where(ab <= 1.0, 1.0, ab + np.sqrt(np.maximum(b * b - 1.0, 0.0)))
    return float(g.max())


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    exact, num = vn_limit()
    print(f'フォン・ノイマンの方法(無限格子): 限界 1/√2 = {exact:.10f}、二分法 {num:.10f}')
    print()
    print('有界な領域(全域1ノード、外周4面とも同じ境界):')
    print('  n   境界        速い方法(固有値)   行列の方法(コンパニオン行列の二分法)')
    for n in (11, 21):
        for bc in BCS:
            fast = critical_nu_fast(n, bc) if bc != 'absorb' else float('nan')
            mat = critical_nu_matrix(n, bc)
            print(f'  {n:<3d} {bc:<11s} {fast:.8f}         {mat:.8f}', flush=True)
    n = 41
    for bc in ('dirichlet', 'zero_grad'):
        print(f'  {n:<3d} {bc:<11s} {critical_nu_fast(n, bc):.8f}         (n=41 は速い方法のみ)')
    print()
    # n=41 の吸収境界はコンパニオン行列が 3362 次になり、固有値計算が手元の PC で数十分かかったため
    # 打ち切った (2026-11-11)。n=31 (1922次) で限界をはさむ ν のスペクトル半径を出す
    print('n=31 の吸収境界: 限界をはさむ ν でのスペクトル半径(行列の方法)')
    for nu in (0.5, 0.7071, 0.70712, 0.7072):
        print(f'  ν = {nu}: ρ = {spectral_radius(31, nu, "absorb"):.8f}', flush=True)


if __name__ == '__main__':
    main()

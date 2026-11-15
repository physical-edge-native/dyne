# 合成の検証器(最小版)— 差分法と学習型の担当計算機を1次元の鎖に並べた構成の安定性を、
# 理想化した1ステップの線形写像(更新行列)の固有値から判定する。
#
# 出所: Claude Code が 2026-11-08 に作成。シミュレータの出力ではなく、理想化した理論の予測である。
#
# 理想化の仮定(theory/acute.py と reference/2026-10-03-learned-stability-region.md 第3.1節に揃える):
#   - 差分法(前進オイラー法): すべての格子点を u + r(左 − 2u + 右) で更新する。区画の端の点は、
#     隣の区画の端点の「前の時刻の値」をゴースト点として使う(heat1d-actor と同じ)。
#   - 学習型(時間離散化した物理情報ニューラルネットワーク): 最適化が毎ステップ完全に収束し、
#     どんな値の並びも表現できると仮定する。このとき
#       * 隣の区画と接する端点(と透過境界の端点)は、差分法と同じ陽的な式で作った目標値になる
#         (heat1d-pinn-actor の界面条件の方式1と透過境界の目標値)。
#       * それ以外の点は後退オイラー法 (1+2r)u_i − r(u_{i−1}+u_{i+1}) = u_i^前 を厳密に解く。
#         陽的に決まった端点は新しい値を境界値として使う。温度固定の端は0、ゼロ勾配の端は鏡映
#         u_{−1} = u_1 で連立方程式に入れる。
#     実装では端点にも方程式残差が課され、最小二乗で折り合うが、ここでは無視する(注意点として報告)。
#   - 判定は誤差の写像で行うので、温度固定の値・観測値などの定数項は落とす(線形写像になる)。
#   - ナッジング(観測点で u ← u + γ(y − u))は1ステップの計算の後に掛ける。誤差は (1−γ) 倍になる。
#   - 透過境界は theory/tbc.py と同じ指数和(補助変数)と3方式(semi=半陰的、delay=1ステップ遅延、
#     drop=直近項を落とす)。補助変数も状態に含めて更新行列を作る。
#
# 使い方: python theory/guarantee/checker.py 構成.json [--r 値] [--scan]
import json
import sys
import numpy as np

STAB_TOL = 1e-9  # スペクトル半径がこれだけ1を超えたら不安定とみなす(theory/ の各スクリプトと同程度)


# ---------------------------------------------------------------- 構成の読み込み
def load_config(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def layout(cfg):
    """区画ごとの格子点の範囲 [(種別, 開始, 終了(含まない)), ...] と総点数を返す。"""
    blocks = []
    s = 0
    for b in cfg['blocks']:
        t = b['type']
        assert t in ('fd', 'learned'), f'未知の種別 {t}'
        n = int(b['points'])
        assert n >= 1
        blocks.append((t, s, s + n))
        s += n
    return blocks, s


def obs_points(cfg, blocks):
    """観測点: 'center' なら各区画の 開始+点数/2(dyne-sim と同じ)。リストなら格子点番号。"""
    nd = cfg.get('nudging')
    if not nd:
        return [], 0.0
    obs = nd.get('obs', 'center')
    if obs == 'center':
        obs = [s + (e - s) // 2 for (_, s, e) in blocks]
    return list(obs), float(nd['gamma'])


def soe(dt, tmax, hq=0.35, c1=1e9, c2=2e4):
    """透過境界の指数和の節点と重み(theory/tbc.py と同一)。"""
    xlo = 0.5 * np.log(1 / (c1 * tmax))
    xhi = 0.5 * np.log(c2 / dt)
    xs = np.arange(xlo, xhi, hq)
    return np.exp(2 * xs), (2 / np.sqrt(np.pi)) * hq * np.exp(xs)


# ---------------------------------------------------------------- 1ステップの写像
class Model:
    def __init__(self, cfg, r, gamma=None):
        self.cfg = cfg
        self.r = float(r)
        self.alpha = float(cfg.get('alpha', 1.0))
        self.blocks, self.n = layout(cfg)
        self.h = float(cfg['h']) if 'h' in cfg else 1.0 / (self.n - 1)
        self.dt = self.r * self.h * self.h / self.alpha
        bc = cfg.get('boundary', {})
        self.bc = [bc.get('left', 'dirichlet'), bc.get('right', 'dirichlet')]
        for b in self.bc:
            assert b in ('dirichlet', 'zero_grad', 'linear_extrap', 'transparent'), f'未知の境界 {b}'
        self.obs, g = obs_points(cfg, self.blocks)
        self.gamma = g if gamma is None else float(gamma)
        # 透過境界の補助変数
        tb = cfg.get('transparent', {})
        self.tmode = tb.get('mode', 'semi')
        self.aux = []  # 面ごとに (lam, w) または None
        for face in range(2):
            if self.bc[face] == 'transparent':
                lam, w = soe(self.dt, float(tb.get('tmax', 0.2)), float(tb.get('hq', 0.35)))
                self.aux.append((lam, w))
            else:
                self.aux.append(None)
        self.naux = [0 if a is None else len(a[0]) for a in self.aux]
        self.size = self.n + sum(self.naux)
        for (t, s, e) in self.blocks:
            for face, p in ((0, 0), (1, self.n - 1)):
                if t == 'learned' and self.bc[face] == 'linear_extrap' and s <= p < e:
                    raise ValueError('学習型の区画に linear_extrap の端は未対応')

    def _split(self, x):
        u = x[:self.n]
        ys = []
        k = self.n
        for m in self.naux:
            ys.append(x[k:k + m])
            k += m
        return u, ys

    def _outer_explicit(self, u, ys, face):
        """全体の端の点の陽的な新しい値と、透過境界の増分 D を返す。温度固定なら0。"""
        r = self.r
        p, q = (0, 1) if face == 0 else (self.n - 1, self.n - 2)
        b = self.bc[face]
        if b == 'dirichlet':
            return 0.0, None
        if b == 'zero_grad':
            return u[p] + 2 * r * (u[q] - u[p]), None
        if b == 'linear_extrap':
            return u[p], None  # ゴースト 2u0−u1 で中心差分が0になり、端点は時間凍結する
        lam, w = self.aux[face]
        y = ys[face]
        c = np.sqrt(self.alpha / np.pi)
        dec = np.exp(-lam * self.dt)
        half = np.exp(-lam * self.dt / 2)
        K = 2 * r * self.h / self.alpha
        ahist = c * np.sum(w * dec * y)
        if self.tmode == 'semi':
            kap = c * np.sum(w * half)
            D = (2 * r * (u[q] - u[p]) - K * ahist) / (1 + K * kap)
        elif self.tmode == 'delay':
            D = 2 * r * (u[q] - u[p]) - K * c * np.sum(w * y)
        elif self.tmode == 'drop':
            D = 2 * r * (u[q] - u[p]) - K * ahist
        else:
            raise ValueError(self.tmode)
        return u[p] + D, D

    def step(self, x):
        r = self.r
        n = self.n
        u, ys = self._split(x)
        # 1) 全点の陽的な候補(前の時刻の値だけを使う)
        expl = np.zeros(n)
        if n >= 3:
            expl[1:n - 1] = u[1:n - 1] + r * (u[:-2] - 2 * u[1:n - 1] + u[2:])
        Ds = [None, None]
        expl[0], Ds[0] = self._outer_explicit(u, ys, 0)
        expl[n - 1], Ds[1] = self._outer_explicit(u, ys, 1)
        new = expl.copy()
        # 2) 学習型の区画は内部を後退オイラー法で解き直す
        for (t, s, e) in self.blocks:
            if t != 'learned':
                continue
            fixed = {}  # 区画内で値が先に決まる点 -> 新しい値
            implicit_mirror = {}  # ゼロ勾配の端: 鏡映で連立に入れる
            for p in (s, e - 1):
                if s == e - 1 and not (p in (0, n - 1) and self.bc[0 if p == 0 else 1] == 'dirichlet'):
                    fixed[p] = expl[p]  # 1点だけの区画は陽的な目標値そのもの(縮退した場合の約束)
                    continue
                if p == 0 or p == n - 1:
                    face = 0 if p == 0 else 1
                    b = self.bc[face]
                    if b == 'dirichlet':
                        fixed[p] = 0.0
                    elif b == 'transparent':
                        fixed[p] = expl[p]
                    elif b == 'zero_grad':
                        implicit_mirror[p] = (1 if p == 0 else n - 2)
                else:
                    fixed[p] = expl[p]  # 隣と接する端点: 陽的な目標値
            idx = [i for i in range(s, e) if i not in fixed]
            if not idx:
                for p, v in fixed.items():
                    new[p] = v
                continue
            pos = {i: k for k, i in enumerate(idx)}
            m = len(idx)
            A = np.zeros((m, m))
            rhs = u[idx].astype(float).copy()
            for k, i in enumerate(idx):
                A[k, k] = 1 + 2 * r
                if i in implicit_mirror:
                    j = implicit_mirror[i]
                    if j in pos:
                        A[k, pos[j]] += -2 * r
                    else:
                        rhs[k] += 2 * r * fixed[j]
                    continue
                for j in (i - 1, i + 1):
                    if j in pos:
                        A[k, pos[j]] += -r
                    elif j in fixed:
                        rhs[k] += r * fixed[j]
                    else:
                        raise AssertionError('区画外の点を陰的に参照した')
            sol = np.linalg.solve(A, rhs)
            for k, i in enumerate(idx):
                new[i] = sol[k]
            for p, v in fixed.items():
                new[p] = v
        # 3) ナッジング(誤差は (1−γ) 倍)
        for p in self.obs:
            new[p] *= (1 - self.gamma)
        # 4) 透過境界の補助変数。増分 D は端点の値の変化(ナッジング前の値で作る)
        out = [new]
        for face in range(2):
            if self.aux[face] is None:
                continue
            lam, _ = self.aux[face]
            dec = np.exp(-lam * self.dt)
            half = np.exp(-lam * self.dt / 2)
            out.append(dec * ys[face] + half * Ds[face])
        return np.concatenate(out)

    def matrix(self):
        S = self.size
        G = np.zeros((S, S))
        for i in range(S):
            e = np.zeros(S)
            e[i] = 1.0
            G[:, i] = self.step(e)
        return G


# ---------------------------------------------------------------- 判定
def analyze(cfg, r, gamma=None):
    m = Model(cfg, r, gamma)
    G = m.matrix()
    ev = np.linalg.eigvals(G)
    k = int(np.argmax(np.abs(ev)))
    rho = float(abs(ev[k]))
    res = {
        'r': float(r),
        'rho': rho,
        'dominant_eig': complex(ev[k]),
        'margin': 1.0 - rho,
        'verdict': 'UNSTABLE' if rho > 1 + STAB_TOL else 'STABLE',
    }
    # 行和ノルム(誤差の最大値が1ステップで増えないことの十分条件)。補助変数がある場合は温度の部分のみ参考値
    nu = m.n
    res['rowsum_norm'] = float(np.max(np.sum(np.abs(G[:nu, :nu]), axis=1)))
    res['rowsum_valid'] = sum(m.naux) == 0
    return res


def critical_r(cfg, r_lo=0.02, r_hi=3.0, dr=0.01, iters=40, gamma=None):
    """r を r_lo から粗く走査し、最初に不安定になる区間を二分法で詰める。見つからなければ None。"""
    if analyze(cfg, r_lo, gamma)['verdict'] == 'UNSTABLE':
        return r_lo
    prev = r_lo
    r = r_lo + dr
    while r <= r_hi + 1e-12:
        if analyze(cfg, r, gamma)['verdict'] == 'UNSTABLE':
            lo, hi = prev, r
            for _ in range(iters):
                mid = 0.5 * (lo + hi)
                if analyze(cfg, mid, gamma)['verdict'] == 'UNSTABLE':
                    hi = mid
                else:
                    lo = mid
            return 0.5 * (lo + hi)
        prev = r
        r += dr
    return None


def critical_gamma(cfg, r, g_lo=0.0, g_hi=12.0, dg=0.02, iters=30):
    """r を固定してγを走査し、最初に不安定になるγを二分法で詰める(theory/nudge.py と同じ流儀)。"""
    prev = g_lo
    g = g_lo + dg
    while g <= g_hi + 1e-12:
        if analyze(cfg, r, g)['verdict'] == 'UNSTABLE':
            lo, hi = prev, g
            for _ in range(iters):
                mid = 0.5 * (lo + hi)
                if analyze(cfg, r, mid)['verdict'] == 'UNSTABLE':
                    hi = mid
                else:
                    lo = mid
            return 0.5 * (lo + hi)
        prev = g
        g += dg
    return None


def main(argv):
    if len(argv) < 2:
        print(__doc__ or '使い方: checker.py 構成.json [--r 値] [--scan]')
        return 2
    cfg = load_config(argv[1])
    r = None
    if '--r' in argv:
        r = float(argv[argv.index('--r') + 1])
    elif 'r' in cfg:
        r = float(cfg['r'])
    blocks, n = layout(cfg)
    print('構成:', ' | '.join(f'{t}{e - s}' for (t, s, e) in blocks), f'(計{n}点)',
          '境界:', cfg.get('boundary', {}))
    print('※ 理想化した理論の予測(最適化の完全収束・任意の値の並びを表現できると仮定)。シミュレータとは未照合。')
    if r is not None:
        a = analyze(cfg, r)
        print(f"r={a['r']:.4f}  スペクトル半径={a['rho']:.6f}  余裕(1−半径)={a['margin']:+.3e}  "
              f"判定={a['verdict']}  最大固有値={a['dominant_eig']:.4f}")
        if a['rowsum_valid']:
            ok = a['rowsum_norm'] <= 1 + 1e-12
            print(f"  行和ノルム={a['rowsum_norm']:.6f}  "
                  f"({'誤差の最大値は毎ステップ増えない(十分条件を満たす)' if ok else '十分条件は満たさない'})")
    if '--scan' in argv or r is None:
        rc = critical_r(cfg)
        print('安定限界 r* =', 'r≤3.0 では不安定化しない' if rc is None else f'{rc:.4f}')
        if rc is not None and r is not None:
            # スペクトル半径の余裕は最も遅く減衰する物理的な成分で決まり0に近くなりがちなので、
            # 運用上の余裕は r の側で示す
            print(f'  r の余裕: r*/r = {rc / r:.3f}({"安定側" if r < rc else "不安定側"})')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

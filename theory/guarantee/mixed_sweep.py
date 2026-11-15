# 差分法と学習型を混ぜた鎖の安定限界 r* を、合成の検証器(checker.py)で求めて表と図にする。
# 出所: Claude Code が 2026-11-08 に作成。理想化した理論の予測であり、シミュレータとは未照合。
# 出力: theory/guarantee/out/mixed_table.dat, mixed_size.dat, mixed_size.png
# 実行: .venv\Scripts\python.exe theory\guarantee\mixed_sweep.py(リポジトリ直下で)
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from checker import Model, critical_r  # noqa: E402

OUT = os.path.join(HERE, 'out')
F, L = 'fd', 'learned'
HDR = '# 出所: theory/guarantee/mixed_sweep.py(理想化した更新行列の固有値。シミュレータの出力ではない。未照合)\n'


def cfg_of(kinds, pts, left='dirichlet', right='dirichlet', tmode=None):
    c = {'blocks': [{'type': k, 'points': int(p)} for k, p in zip(kinds, pts)],
         'boundary': {'left': left, 'right': right}}
    if tmode:
        c['transparent'] = {'mode': tmode, 'tmax': 0.2}
    return c


def split(n, k):
    base, rem = divmod(n, k)
    return [base + (rem if i == 0 else 0) for i in range(k)]


def fd_share(cfg, r):
    """r での絶対値最大の固有値の固有ベクトルのうち、差分法の格子点に乗っている割合(二乗和の比)。"""
    m = Model(cfg, r)
    G = m.matrix()
    ev, V = np.linalg.eig(G)
    v = np.abs(V[:m.n, int(np.argmax(np.abs(ev)))]) ** 2
    mask = np.zeros(m.n, bool)
    for (t, s, e) in m.blocks:
        if t == F:
            mask[s:e] = True
    return float(v[mask].sum() / v.sum())


def table():
    cases = [
        ('差分法1台', [F], [65], {}),
        ('学習型1台', [L], [65], {}),
        ('学習型4分割', [L] * 4, split(65, 4), {}),
        ('差分法|学習型', [F, L], [33, 32], {}),
        ('学習型|差分法', [L, F], [33, 32], {}),
        ('差分法|学習型|差分法', [F, L, F], [22, 22, 21], {}),
        ('学習型|差分法|学習型', [L, F, L], [22, 22, 21], {}),
        ('交互4分割(差,学,差,学)', [F, L] * 2, split(65, 4), {}),
        ('学習型3+差分法1', [L, L, L, F], split(65, 4), {}),
        ('差分法3+学習型1', [F, F, F, L], split(65, 4), {}),
        ('交互8分割', [F, L] * 4, split(65, 8), {}),
        ('交互16分割', [F, L] * 8, split(65, 16), {}),
        ('差分法|学習型 両端ゼロ勾配', [F, L], [33, 32], dict(left='zero_grad', right='zero_grad')),
        ('学習型4分割 両端ゼロ勾配', [L] * 4, split(65, 4), dict(left='zero_grad', right='zero_grad')),
        ('学習型4分割 左端透過(半陰的)', [L] * 4, split(65, 4), dict(left='transparent', tmode='semi')),
        ('学習型1台 左端透過(半陰的)', [L], [65], dict(left='transparent', tmode='semi')),
        ('学習型|差分法 左端透過(半陰的)', [L, F], [33, 32], dict(left='transparent', tmode='semi')),
    ]
    rows = []
    for name, k, p, kw in cases:
        c = cfg_of(k, p, **kw)
        lo = 0.2 if 'transparent' in kw.values() else 0.02
        rc = critical_r(c, r_lo=lo)
        share = fd_share(c, rc + 1e-3) if (rc is not None and F in k) else float('nan')
        rows.append((name, '|'.join(f'{t[0].upper()}{n}' for t, n in zip(k, p)), rc, share))
        print(f'{name:28s} {rows[-1][1]:30s} r*={"なし(≤3.0)" if rc is None else f"{rc:.4f}"}  '
              f'差分法側の割合={share:.2f}')
    with open(os.path.join(OUT, 'mixed_table.dat'), 'w', encoding='utf-8') as f:
        f.write(HDR)
        f.write('# 65点(格子間隔1/64)、α=1。指定のない端は温度固定。F=差分法、L=学習型、数字は格子点数\n')
        f.write('# 列: 構成  区画  安定限界r*(なし=r≤3.0で不安定化しない)  限界直上の不安定な固有ベクトルが差分法の点に乗る割合\n')
        for name, lay, rc, share in rows:
            f.write(f'{name}\t{lay}\t{"なし" if rc is None else f"{rc:.4f}"}\t{share:.3f}\n')


def size_sweep():
    ks = list(range(1, 22))
    data = []
    for k in ks:
        a = (65 - k) // 2
        b = 65 - k - a
        r1 = critical_r(cfg_of([L, F, L], [a, k, b]))  # 学習型の中に差分法 k 点
        r2 = critical_r(cfg_of([F, L, F], [a, k, b]))  # 差分法の中に学習型 k 点
        # 格子を細かくしても同じか(129点): 区画の点数 k は同じにする
        a2 = (129 - k) // 2
        r3 = critical_r(cfg_of([L, F, L], [a2, k, 129 - k - a2]))
        data.append((k, r1, r2, r3))
        print(f'k={k:2d}  学|差k|学 r*={r1:.4f}  差|学k|差 r*={r2:.4f}  学|差k|学(129点) r*={r3:.4f}')
    with open(os.path.join(OUT, 'mixed_size.dat'), 'w', encoding='utf-8') as f:
        f.write(HDR)
        f.write('# 両端温度固定、α=1。中央の区画の格子点数 k を変える(残りを両側に等分)\n')
        f.write('# 列: k  r*(学習型|差分法k点|学習型、65点)  r*(差分法|学習型k点|差分法、65点)  r*(学習型|差分法k点|学習型、129点)\n')
        for k, r1, r2, r3 in data:
            f.write(f'{k} {r1:.5f} {r2:.5f} {r3:.5f}\n')
    return data


def plot(data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family'] = ['Yu Gothic', 'Meiryo', 'MS Gothic', 'sans-serif']
    k = [d[0] for d in data]
    fig, ax = plt.subplots(figsize=(6.4, 3.8), dpi=150)
    ax.axhline(0.5, color='#888888', lw=0.8, ls='--')
    ax.axhline((7 - np.sqrt(17)) / 4, color='#888888', lw=0.8, ls='-.')
    ax.text(21.3, 0.497, '差分法 1/2(十分条件で保証)', fontsize=7, va='top', ha='right', color='#555555')
    ax.text(21.3, 0.7192, '学習型の分割 0.719', fontsize=7, va='bottom', ha='right', color='#555555')
    ax.plot(k, [d[1] for d in data], 'o-', color='#2a6fdb', ms=3.5, lw=1.4,
            label='学習型|差分法 k点|学習型(65点)')
    ax.plot(k, [d[3] for d in data], 'x:', color='#2a6fdb', ms=4, lw=1.0,
            label='同上(129点)')
    ax.plot(k, [d[2] for d in data], 's-', color='#d9622b', ms=3.5, lw=1.4,
            label='差分法|学習型 k点|差分法(65点)')
    ax.set_xticks(range(1, 22, 2))
    ax.set_xlabel('中央の区画の格子点数 k')
    ax.set_ylabel('安定限界 r*')
    ax.set_ylim(0.45, 0.78)
    ax.set_title('差分法と学習型を混ぜた鎖の安定限界(理想化した理論の予測・シミュレータと未照合)',
                 fontsize=8.5, loc='left')
    ax.legend(fontsize=7, frameon=False, loc='center right')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'mixed_size.png'))


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    table()
    plot(size_sweep())

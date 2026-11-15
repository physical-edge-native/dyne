# 合成の検証器の照合テスト: 既知の理論値(theory/ の各スクリプトと reference/2026-10-03-*.md)を再現するか。
# 実行: .venv\Scripts\python.exe -m unittest theory/guarantee/test_checker.py(リポジトリ直下で)
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checker import analyze, critical_r, critical_gamma  # noqa: E402


def split65(kind, nodes):
    """65点を dyne-sim と同じく割る(余りは先頭ノードへ)。"""
    base, rem = divmod(65, nodes)
    pts = [base + (rem if i == 0 else 0) for i in range(nodes)]
    if isinstance(kind, str):
        kind = [kind] * nodes
    return {'blocks': [{'type': k, 'points': p} for k, p in zip(kind, pts)]}


class KnownNumbers(unittest.TestCase):
    def test_fd_half(self):
        # 差分法の陽解法: 分割数・境界の種類によらず r ≤ 1/2(65点では最大固有値 1−4r cos²(π/128) に由来)
        for nodes in (1, 4, 16):
            for bc in ('dirichlet', 'zero_grad'):
                cfg = split65('fd', nodes)
                cfg['boundary'] = {'left': bc, 'right': bc}
                rc = critical_r(cfg)
                self.assertAlmostEqual(rc, 0.5, delta=0.0015, msg=f'{nodes} {bc} {rc}')

    def test_fd_exact_dirichlet(self):
        # 温度固定・内部63点: 限界は 1/(2 cos²(π/128)) をわずかに上回る値
        rc = critical_r(split65('fd', 4))
        # 固有値 1 − 4r sin²(kπ/128) の最小(k=63)が −1 になる r
        exact = 1 / (2 * np.sin(63 * np.pi / 128) ** 2)
        self.assertAlmostEqual(rc, exact, places=6)

    def test_learned_single_unconditional(self):
        # 学習型1台: 後退オイラー法と同じなので r によらず安定。r=5 で 0.988(acute.py)
        a = analyze(split65('learned', 1), 5.0)
        self.assertEqual(a['verdict'], 'STABLE')
        self.assertAlmostEqual(a['rho'], 0.988, places=3)
        self.assertIsNone(critical_r(split65('learned', 1)))

    def test_learned_split_0719(self):
        # 学習型を並べた理想化写像: 2〜8分割で (7−√17)/4 ≈ 0.7192、16分割 0.7073、32分割 0.5003(learned_theory.dat)
        closed = (7 - np.sqrt(17)) / 4
        expect = {2: 0.7192, 4: 0.7192, 8: 0.7192, 16: 0.7073, 32: 0.5003}
        for nodes, v in expect.items():
            rc = critical_r(split65('learned', nodes))
            self.assertAlmostEqual(rc, v, delta=6e-5, msg=f'{nodes}分割 {rc}')
        self.assertAlmostEqual(critical_r(split65('learned', 4)), closed, delta=1e-4)

    def test_nudging_gamma_boundary(self):
        # ナッジング: 差分法4分割、観測点=各区画の中央(8,25,41,57)。nudging_boundary.dat の γ*
        cfg = split65('fd', 4)
        cfg['nudging'] = {'gamma': 0.0, 'obs': 'center'}
        for r, g in ((0.02, 2.0421), (0.10, 2.2677)):
            gs = critical_gamma(cfg, r, g_lo=2.0)
            self.assertAlmostEqual(gs, g, delta=6e-5, msg=f'r={r} {gs}')
        # 0 ≤ γ ≤ 2、r ≤ 1/2 では行和ノルム ≤ 1(誤差の最大値が増えない)
        for g in (0.0, 0.5, 1.0, 1.5, 2.0):
            a = analyze(cfg, 0.5, g)
            self.assertLessEqual(a['rowsum_norm'], 1 + 1e-12)

    def test_transparent_three_modes(self):
        # 透過境界(左端)+右端温度固定、65点1台: 半陰的0.500、1ステップ遅延0.319、直近項を落とす0.438
        for mode, v in (('semi', 0.500), ('delay', 0.319), ('drop', 0.438)):
            cfg = split65('fd', 1)
            cfg['boundary'] = {'left': 'transparent', 'right': 'dirichlet'}
            cfg['transparent'] = {'mode': mode, 'tmax': 0.2}
            rc = critical_r(cfg, r_lo=0.2)
            self.assertAlmostEqual(rc, v, delta=0.0015, msg=f'{mode} {rc}')

    def test_mixed_rowsum_half(self):
        # 新しい主張の確認: 差分法と学習型を任意に混ぜても r ≤ 1/2 なら行和ノルム ≤ 1
        rng = np.random.default_rng(1)
        for _ in range(20):
            k = int(rng.integers(2, 7))
            pts = rng.integers(1, 20, size=k)
            kinds = rng.choice(['fd', 'learned'], size=k)
            bc = rng.choice(['dirichlet', 'zero_grad'], size=2)
            cfg = {'blocks': [{'type': str(t), 'points': int(p)} for t, p in zip(kinds, pts)],
                   'boundary': {'left': str(bc[0]), 'right': str(bc[1])}}
            if sum(pts) < 3:
                continue
            for r in (0.1, 0.3, 0.5):
                a = analyze(cfg, r)
                self.assertLessEqual(a['rowsum_norm'], 1 + 1e-12, msg=str(cfg))
                self.assertEqual(a['verdict'], 'STABLE')


if __name__ == '__main__':
    unittest.main()

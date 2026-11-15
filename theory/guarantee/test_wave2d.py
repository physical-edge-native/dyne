# 2次元の波動方程式の安定性(wave2d.py)の照合テスト。既存の test_checker.py とは独立。
# 実行: .venv\Scripts\python.exe -m unittest theory/guarantee/test_wave2d.py(リポジトリ直下で)
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wave2d import (actor_step, critical_nu_fast, critical_nu_matrix, rho_fast,  # noqa: E402
                    spectral_radius, vn_gmax, vn_limit)


class VonNeumann(unittest.TestCase):
    def test_limit_is_one_over_sqrt2(self):
        exact, num = vn_limit()
        self.assertAlmostEqual(exact, 1 / math.sqrt(2), places=15)
        self.assertAlmostEqual(num, exact, places=9)

    def test_amplitude_preserved_below_limit(self):
        # 限界以下では全ての波数で |g| = 1(減衰も増幅もしない)
        for nu in (0.1, 0.5, 0.7):
            self.assertEqual(vn_gmax(nu), 1.0)
        self.assertGreater(vn_gmax(0.71), 1.0)


class BoundedDomain(unittest.TestCase):
    def test_fast_formulas(self):
        # 値固定: 最大の固有値は内部の最も細かいモード → 限界は 1/(√2·sin((n−2)π/(2(n−1))))
        for n in (11, 41):
            self.assertAlmostEqual(critical_nu_fast(n, 'dirichlet'),
                                   1 / (math.sqrt(2) * math.sin((n - 2) * math.pi / (2 * (n - 1)))), places=12)
            # ゼロ勾配: 市松模様のモードがあるので無限格子と同じ 1/√2
            self.assertAlmostEqual(critical_nu_fast(n, 'zero_grad'), 1 / math.sqrt(2), places=12)

    def test_matrix_agrees_with_fast(self):
        # 実際の更新式から組み立てたコンパニオン行列の二分法と、固有値の公式が一致する
        for bc in ('dirichlet', 'zero_grad'):
            self.assertAlmostEqual(critical_nu_matrix(11, bc), critical_nu_fast(11, bc), delta=2e-6)
            for nu in (0.6, 0.72):
                self.assertAlmostEqual(spectral_radius(11, nu, bc), rho_fast(11, nu, bc), delta=1e-6)

    def test_absorbing_limit(self):
        # ムーアの吸収境界(1次)を付けても、限界は 1/√2 のまま(n = 11 の行列の方法)
        self.assertAlmostEqual(critical_nu_matrix(11, 'absorb'), 1 / math.sqrt(2), delta=2e-6)
        # 限界以下では減衰する(スペクトル半径 < 1。一定値のモードだけは 1)
        self.assertLessEqual(spectral_radius(11, 0.5, 'absorb'), 1.0 + 1e-6)

    def test_actor_step_is_linear_and_first_step(self):
        # 最初の1ステップの式 u + (ν²/2)Lu は、u^{−1} = u^1 と置いた一般の式と同じ値になる
        rng = np.random.default_rng(0)
        u = rng.standard_normal((9, 9))
        u[:, 0] = u[:, -1] = u[0, :] = u[-1, :] = 0
        u1 = actor_step(u, None, 0.5, 'dirichlet', first=True)
        u1b = actor_step(u, u1, 0.5, 'dirichlet')
        self.assertTrue(np.allclose(u1, u1b, atol=1e-14))


if __name__ == '__main__':
    unittest.main()

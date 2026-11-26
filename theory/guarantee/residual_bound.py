"""Error bound when learned regions leave a residual (GitHub issue #159).

Statement (heat equation, 1D chain of classical and learned regions, as in checker.py):
  Let Phi be the idealised one-step map (classical points explicit; learned interiors solved exactly by backward
  Euler), and assume ||Phi||_inf <= 1 (proven for r <= 1/2 with fixed-value or zero-gradient ends, also with nudging
  0 <= gamma <= 2; docs-site/theory.md). Let the computed sequence satisfy u_{k+1} = Phi u_k + rho_k, where the
  one-step defect rho_k is what the learned solver leaves (its values minus the exact step from the same previous
  values), and let v_k be the exact sequence v_{k+1} = Phi v_k. Then
      ||u_n - v_n||_inf <= ||u_0 - v_0||_inf + sum_{k<n} ||rho_k||_inf <= ||u_0 - v_0||_inf + n * delta
  if every ||rho_k||_inf <= delta.
  Proof: u_{k+1} - v_{k+1} = Phi (u_k - v_k) + rho_k, so ||e_{k+1}|| <= ||Phi|| ||e_k|| + ||rho_k|| <= ||e_k|| + ||rho_k||;
  induction on k.
Link to the training residual: inside a learned region the exact values w* solve B w* = b, the computed values solve
  B w = b + q with q the backward-Euler residual at the grid points, so w - w* = B^{-1} q, and ||B^{-1}||_inf <= 1
  (B has diagonal 1 + 2r and off-diagonal -r: every row sum of |B| minus twice the off-diagonal part is >= 1). Values
  that the learned region takes as targets at its ends add their own deviation. So the defect is at most the largest
  end deviation plus the largest residual.
The bound grows linearly in n and cannot be improved in general: with zero-gradient ends Phi keeps constants, and a
constant defect of +delta in every point gives an error of exactly n * delta.

Checks (run from the repository root; numpy only, plus dyne-sim for the second part):
  1. Synthetic: several mixed chains and r in {0.1, 0.3, 0.5}: ||Phi||_inf <= 1 is confirmed, then 200 steps with
     random and with constant (worst-case) defects of size delta on the learned points; the error must stay under the
     bound, and the constant defect on a single learned region with zero-gradient ends must reach it.
  2. Simulator: dyne-sim runs the learned solver (crates/heat1d-pinn-actor) on one device with fixed-value ends and
     writes the field after every step; the defect of every step against the exact backward-Euler step is measured,
     and the distance from the exact sequence started at the same field must stay under the sum of the defects.

Usage:
  python theory/guarantee/residual_bound.py [--sim target/release/dyne-sim --wasm <heat1d_pinn_actor.wasm>]
"""

import argparse
import csv
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from checker import Model  # noqa: E402

DELTA = 1e-3
STEPS = 200
EPS = 1e-12  # rounding allowance per step


def chains():
    def blocks(kinds_points):
        return [{'type': t, 'points': p} for t, p in kinds_points]
    return {
        'fd|learned|fd (fixed ends)': {'blocks': blocks([('fd', 22), ('learned', 22), ('fd', 21)]),
                                       'boundary': {'left': 'dirichlet', 'right': 'dirichlet'}},
        'learned x4 (fixed ends)': {'blocks': blocks([('learned', 17), ('learned', 16), ('learned', 16), ('learned', 16)]),
                                    'boundary': {'left': 'dirichlet', 'right': 'dirichlet'}},
        'alternating x16 (zero-gradient ends)': {'blocks': blocks([('fd' if i % 2 else 'learned', 5 if i else 5)
                                                                   for i in range(16)] + [('fd', 1)]),
                                                 'boundary': {'left': 'zero_grad', 'right': 'zero_grad'}},
        'learned x4, nudging 1.5 (zero-gradient ends)': {
            'blocks': blocks([('learned', 17), ('learned', 16), ('learned', 16), ('learned', 16)]),
            'boundary': {'left': 'zero_grad', 'right': 'zero_grad'}, 'nudging': {'gamma': 1.5, 'obs': 'center'}},
        'one learned region (zero-gradient ends)': {'blocks': blocks([('learned', 65)]),
                                                    'boundary': {'left': 'zero_grad', 'right': 'zero_grad'}},
    }


def learned_points(model):
    pts = []
    for t, s, e in model.blocks:
        if t == 'learned':
            pts.extend(range(s, e))
    return np.array(pts, dtype=int)


def synthetic(log):
    ok = True
    rng = np.random.default_rng(159)
    tight = None
    for name, cfg in chains().items():
        for r in (0.1, 0.3, 0.5):
            m = Model(cfg, r)
            g = m.matrix()
            norm = np.abs(g).sum(axis=1).max()
            pts = learned_points(m)
            worst = 0.0
            for mode in ('random', 'constant'):
                u = rng.uniform(-1, 1, m.size)
                v = u + rng.uniform(-1e-2, 1e-2, m.size)
                e0 = np.abs(u - v).max()
                for k in range(STEPS):
                    rho = np.zeros(m.size)
                    rho[pts] = rng.uniform(-DELTA, DELTA, len(pts)) if mode == 'random' else DELTA
                    u = m.step(u) + rho
                    v = m.step(v)
                    err = np.abs(u - v).max()
                    bound = e0 + (k + 1) * DELTA + (k + 1) * EPS
                    if err > bound:
                        ok = False
                        log(f"  VIOLATED {name} r={r} {mode} step {k + 1}: error {err:.3e} > bound {bound:.3e}")
                        break
                    worst = max(worst, (err - 0.0) / bound)
                if mode == 'constant' and name.startswith('one learned') and r == 0.5:
                    tight = (err, e0 + STEPS * DELTA)
            ok &= norm <= 1 + 1e-12
            log(f"  {name:45s} r={r:.1f}  ||Phi||_inf {norm:.12f}  max error/bound {worst:.3f}")
    if tight is not None:
        log(f"  worst case (one learned region, zero-gradient ends, constant defect, r=0.5): error {tight[0]:.6f} "
            f"vs bound {tight[1]:.6f}")
        ok &= tight[0] >= 0.99 * (tight[1] - 1e-2)  # the bound is reached up to the initial difference
    return ok


def b_inverse_norm(r, m):
    b = np.diag([1 + 2 * r] * m) + np.diag([-r] * (m - 1), 1) + np.diag([-r] * (m - 1), -1)
    return np.abs(np.linalg.inv(b)).sum(axis=1).max()


def simulator(sim, wasm, log, points=17, steps=40, dt=1e-4, alpha=1.0):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'field.csv')
        cmd = [sim, '--wasm', wasm, '--nodes', '1', '--points', str(points), '--steps', str(steps),
               '--dt', repr(dt), '--alpha', repr(alpha), '--ram-cap-kb', '100000', '--quiet',
               '--field-csv', path, '--field-every', '1']
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with open(path, newline='', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
    field = {}
    for row in rows:
        field.setdefault(int(row['step']), np.zeros(points))[int(row['i'])] = float(row['u'])
    h = 1.0 / (points - 1)
    r = alpha * dt / (h * h)
    m = Model({'blocks': [{'type': 'learned', 'points': points}],
               'boundary': {'left': 'dirichlet', 'right': 'dirichlet'}}, r)
    ks = sorted(field)
    u0 = field[ks[0]]
    v = u0.copy()
    total = 0.0
    ok = True
    worst = 0.0
    lemma_ok = True
    b = np.diag([1 + 2 * r] * (points - 2)) + np.diag([-r] * (points - 3), 1) + np.diag([-r] * (points - 3), -1)
    for k in ks[:-1]:
        u_prev, u_next = field[k], field[k + 1]
        rho = u_next - m.step(u_prev)
        d = np.abs(rho).max()
        total += d
        v = m.step(v)
        err = np.abs(u_next - v).max()
        ok &= err <= total + 1e-12
        worst = max(worst, err / total if total > 0 else 0.0)
        # the defect is at most the end deviation plus the backward-Euler residual at the grid points
        ends = max(abs(u_next[0]), abs(u_next[-1]))
        b_vec = u_prev[1:-1].copy()
        b_vec[0] += r * u_next[0]
        b_vec[-1] += r * u_next[-1]
        q = np.abs(b @ u_next[1:-1] - b_vec).max()
        lemma_ok &= d <= ends + q + 1e-12
    log(f"  learned solver on one device ({points} points, r={r:.4f}, {len(ks) - 1} steps): final error "
        f"{err:.3e}, sum of defects {total:.3e}, largest error/bound {worst:.3f}; ||B^-1||_inf {b_inverse_norm(r, points - 2):.12f}")
    log(f"  defect <= end deviation + grid residual in every step: {lemma_ok}")
    return ok and lemma_ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim', help='dyne-sim binary (for the simulator check)')
    ap.add_argument('--wasm', help='heat1d_pinn_actor.wasm')
    a = ap.parse_args()
    log = print
    log('synthetic defects on the idealised mixed map:')
    ok = synthetic(log)
    if a.sim and a.wasm:
        log('learned solver in the simulator:')
        ok &= simulator(a.sim, a.wasm, log)
    else:
        log('simulator check skipped (give --sim and --wasm)')
    log('PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

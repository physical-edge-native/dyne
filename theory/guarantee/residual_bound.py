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
  2. Simulator: dyne-sim runs the learned solver (crates/heat1d-pinn-actor) on one device, and on four devices
     (issue #197), with fixed-value ends and writes the field after every step; the defect of every step against the
     idealised map of the same split is measured, the distance from the exact sequence started at the same field must
     stay under the sum of the defects, and in every region the defect must be at most the largest deviation at the
     region's ends plus its backward-Euler residual.

  3. Synthetic in 2D and 3D (issue #197; heat_nd.py): the same argument holds for the explicit scheme with
     r <= 1/(2d) (1/4 in 2D, 1/6 in 3D). Mixed arrangements of classical and learned blocks on a 20 x 20 and an
     8 x 8 x 8 grid: ||Phi||_inf <= 1 is confirmed, defects as in 1 stay under the bound, the constant defect on
     one learned region with zero-gradient ends reaches it, and just above r = 1/(2d) a classical grid has
     ||Phi||_inf > 1 (the limit is sharp).

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
from heat_nd import MixedHeatND, block_centres, grid_blocks  # noqa: E402

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


def chains_nd():
    """2D and 3D arrangements: (name, shape, splits, kind of block, outer edges, nudging gamma)."""
    lrn = lambda bi: 'learned'  # noqa: E731
    checker = lambda bi: 'learned' if sum(bi) % 2 == 0 else 'fd'  # noqa: E731
    diag = lambda bi: 'learned' if len(set(bi)) == 1 else 'fd'  # noqa: E731
    return [
        ('2D fd/learned 2x2 checkerboard (fixed ends)', (20, 20), (2, 2), checker, 'dirichlet', 0.0),
        ('2D learned 3x3 (zero-gradient ends)', (20, 20), (3, 3), lrn, 'zero_grad', 0.0),
        ('2D fd/learned 4x3 checkerboard, uneven (zero-gr.)', (20, 20), (4, 3), checker, 'zero_grad', 0.0),
        ('2D learned 2x2, nudging 1.5 (zero-gradient ends)', (20, 20), (2, 2), lrn, 'zero_grad', 1.5),
        ('2D one learned region (zero-gradient ends)', (20, 20), (1, 1), lrn, 'zero_grad', 0.0),
        ('3D fd/learned 2x2x2 checkerboard (fixed ends)', (8, 8, 8), (2, 2, 2), checker, 'dirichlet', 0.0),
        ('3D learned on the diagonal 2x2x2 (zero-gradient)', (8, 8, 8), (2, 2, 2), diag, 'zero_grad', 0.0),
        ('3D one learned region (zero-gradient ends)', (8, 8, 8), (1, 1, 1), lrn, 'zero_grad', 0.0),
    ]


def synthetic_nd(log):
    ok = True
    rng = np.random.default_rng(197)
    for name, shape, splits, kinds, bc, gamma in chains_nd():
        d = len(shape)
        rmax = 1.0 / (2 * d)
        blocks = grid_blocks(shape, splits, kinds)
        obs = block_centres(shape, blocks) if gamma else ()
        for r in (rmax / 5, rmax / 2, rmax):
            m = MixedHeatND(shape, blocks, r, bc, gamma, obs)
            norm = m.norm()
            pts = np.array([f for f in range(m.n) if blocks[m.owner.ravel()[f]][0] == 'learned'], dtype=int)
            worst = 0.0
            for mode in ('random', 'constant'):
                u = rng.uniform(-1, 1, m.n)
                v = u + rng.uniform(-1e-2, 1e-2, m.n)
                e0 = np.abs(u - v).max()
                for k in range(STEPS):
                    rho = np.zeros(m.n)
                    rho[pts] = rng.uniform(-DELTA, DELTA, len(pts)) if mode == 'random' else DELTA
                    u = m.step(u) + rho
                    v = m.step(v)
                    err = np.abs(u - v).max()
                    bound = e0 + (k + 1) * DELTA + (k + 1) * EPS
                    if err > bound:
                        ok = False
                        log(f"  VIOLATED {name} r={r} {mode} step {k + 1}: error {err:.3e} > bound {bound:.3e}")
                        break
                    worst = max(worst, err / bound)
                if mode == 'constant' and 'one learned' in name and r == rmax:
                    tight = err >= 0.99 * (e0 + STEPS * DELTA - 1e-2)
                    ok &= tight
                    log(f"  worst case {d}D (one learned region, zero-gradient ends, constant defect, r=1/{2 * d}): "
                        f"error {err:.6f} vs bound {e0 + STEPS * DELTA:.6f}")
            ok &= norm <= 1 + 1e-12
            log(f"  {name:50s} r={r:.4f}  ||Phi||_inf {norm:.12f}  max error/bound {worst:.3f}")
    for d, shape in ((2, (20, 20)), (3, (8, 8, 8))):
        r = 1.0 / (2 * d) * 1.05
        norm = MixedHeatND(shape, grid_blocks(shape, (1,) * d, lambda bi: 'fd'), r, 'zero_grad').norm()
        ok &= norm > 1 + 1e-3
        log(f"  {d}D classical grid just above the limit (r = 1.05/{2 * d}): ||Phi||_inf {norm:.6f} > 1")
    return ok


def b_inverse_norm(r, m):
    b = np.diag([1 + 2 * r] * m) + np.diag([-r] * (m - 1), 1) + np.diag([-r] * (m - 1), -1)
    return np.abs(np.linalg.inv(b)).sum(axis=1).max()


def sim_blocks(nodes, points):
    """dyne-sim's split of the grid: no overlap, the first points % nodes devices get one point more."""
    base, extra = divmod(points, nodes)
    return [{'type': 'learned', 'points': base + (1 if i < extra else 0)} for i in range(nodes)]


def simulator(sim, wasm, log, nodes=1, points=17, steps=40, dt=1e-4, alpha=1.0):
    """Run the learned solver on `nodes` devices in dyne-sim (fixed-value ends) and check, against the idealised map
    of the same split (checker.Model): the distance from the exact sequence stays under the sum of the one-step
    defects, and in every step and every region the defect is at most the largest deviation at the region's ends
    (from the explicit border target, or from 0 at a fixed-value end) plus the backward-Euler residual inside."""
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'field.csv')
        cmd = [sim, '--wasm', wasm, '--nodes', str(nodes), '--points', str(points), '--steps', str(steps),
               '--dt', repr(dt), '--alpha', repr(alpha), '--ram-cap-kb', '100000', '--quiet',
               '--field-csv', path, '--field-every', '1']
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with open(path, newline='', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
    h = 1.0 / (points - 1)
    field = {}
    for row in rows:
        gi = int(round(float(row['x']) / h))  # the CSV gives each device's local index and the global x
        field.setdefault(int(row['step']), np.full(points, np.nan))[gi] = float(row['u'])
    r = alpha * dt / (h * h)
    m = Model({'blocks': sim_blocks(nodes, points), 'boundary': {'left': 'dirichlet', 'right': 'dirichlet'}}, r)
    ks = sorted(field)
    assert all(not np.isnan(field[k]).any() for k in ks), 'missing points in the field CSV'
    v = field[ks[0]].copy()
    total = 0.0
    ok = True
    worst = 0.0
    lemma_ok = True
    for k in ks[:-1]:
        u_prev, u_next = field[k], field[k + 1]
        rho = u_next - m.step(u_prev)
        total += np.abs(rho).max()
        v = m.step(v)
        err = np.abs(u_next - v).max()
        ok &= err <= total + 1e-12
        worst = max(worst, err / total if total > 0 else 0.0)
        for _, s0, e0 in m.blocks:
            ends = []
            for p, q in ((s0, s0 + 1), (e0 - 1, e0 - 2)):
                if p in (0, points - 1):
                    target = 0.0  # fixed-value end
                else:
                    o = p - 1 if p == s0 else p + 1  # the neighbour's previous value (sent across the border)
                    target = u_prev[p] + r * (u_prev[o] - 2 * u_prev[p] + u_prev[q])
                ends.append(abs(u_next[p] - target))
            n_in = e0 - s0 - 2
            b = np.diag([1 + 2 * r] * n_in) + np.diag([-r] * (n_in - 1), 1) + np.diag([-r] * (n_in - 1), -1)
            b_vec = u_prev[s0 + 1:e0 - 1].copy()
            b_vec[0] += r * u_next[s0]
            b_vec[-1] += r * u_next[e0 - 1]
            q = np.abs(b @ u_next[s0 + 1:e0 - 1] - b_vec).max()
            lemma_ok &= np.abs(rho[s0:e0]).max() <= max(ends) + q + 1e-12
    where = 'one device' if nodes == 1 else f'{nodes} devices'
    log(f"  learned solver on {where} ({points} points, r={r:.4f}, {len(ks) - 1} steps): final error "
        f"{err:.3e}, sum of defects {total:.3e}, largest error/bound {worst:.3f}; ||B^-1||_inf "
        f"{b_inverse_norm(r, points - 2):.12f}")
    log(f"  defect <= end deviation + grid residual in every step and region ({where}): {lemma_ok}")
    return ok and lemma_ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim', help='dyne-sim binary (for the simulator check)')
    ap.add_argument('--wasm', help='heat1d_pinn_actor.wasm')
    a = ap.parse_args()
    log = print
    log('synthetic defects on the idealised mixed map:')
    ok = synthetic(log)
    log('synthetic defects on the idealised mixed map in 2D and 3D (issue #197):')
    ok &= synthetic_nd(log)
    if a.sim and a.wasm:
        log('learned solver in the simulator:')
        ok &= simulator(a.sim, a.wasm, log)
        ok &= simulator(a.sim, a.wasm, log, nodes=4, points=65, steps=40, dt=1e-5)
    else:
        log('simulator check skipped (give --sim and --wasm)')
    log('PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

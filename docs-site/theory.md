# Stability theory

When regions are solved on different devices and exchange only the previous step's border values, does the coupled
computation stay stable, and how large can errors grow? This page summarises what is known, separating what is
proven from what is derived under idealisation, checked numerically, or still open.

Two kinds of error are kept apart throughout:

- **numerical error** — the difference from the same computation carried out exactly (splitting adds none: the split
  result is bit-identical);
- **estimation error** — the difference from the true physical value (sensor noise, model approximations).

Norms are the maximum norms: for a vector, `‖e‖∞ = max_j |e_j|`; for a matrix, `‖M‖∞ = max_i Σ_j |M_ij|` (largest
row sum). "Stable" below means the largest numerical error never increases.

## Border values in an explicit scheme

For the 1D heat equation with `r = αΔt/h²`, every interior point is updated from the previous step:

```
u(j, t) = r·u(j−1, t−1) + (1 − 2r)·u(j, t−1) + r·u(j+1, t−1)
```

At the left end of a region, the missing neighbour value is the value `ũ(t−1)` received from the left neighbour:
`u(0, t) = u(0, t−1) + r·(ũ(t−1) − 2u(0, t−1) + u(1, t−1))`. The neighbour's value enters only through the `t−1`
term — the same term as in the single-solver formula — so the split result is identical.

## Border values in a learned solver

The learned solver (a small physics-informed network, `crates/heat1d-pinn-actor`) represents `u(x)` and, at every
step, is trained by a few Levenberg–Marquardt iterations to make the backward-Euler residual vanish. If training
converged exactly, a step would be: compute the two end points explicitly from the neighbours' `t−1` values, then
solve the interior implicitly, `B·u_in(t) = u_in(t−1) + r·(u(0,t)·e₁ + u(n−1,t)·e_last)`, with `B` tridiagonal
(diagonal `1 + 2r`, off-diagonal `−r`). Only the previous step's border values cross between devices, so no
iteration between devices is needed.

## Results

| Status | Result |
|---|---|
| **Proven** | Explicit heat scheme: for `r ≤ 1/2`, `‖G‖∞ ≤ 1` — the largest numerical error never increases (fixed-value and insulated boundaries alike). |
| **Proven** | Nudging to sensors `u ← u + γ(y − u)`: for `r ≤ 1/2` and `0 ≤ γ ≤ 2`, the error never increases, whatever the number, position and period of observations (`‖I − γP‖∞ = max(1, |1 − γ|)`). `γ = 2` cannot be improved. |
| **Proven** | Splitting does not change the result (same operations in the same order). |
| **Proven** | 2D wave equation (von Neumann analysis on an infinite uniform grid): stable if and only if the Courant number `ν = cΔt/h ≤ 1/√2 ≈ 0.70711` (`1/√3` in 3D). |
| **Proven** | Learned regions that leave a residual: if the step with exact training never increases the error (as above for `r ≤ 1/2`), and every step's learned output differs from the exact step by at most `δ`, the error after `n` steps is at most the initial error plus `n·δ`; `δ` is at most the deviation at the region's ends plus the training residual at the grid points. See [below](#learned-regions-that-leave-a-residual). |
| **Proven** | The same bound in 2D and 3D: for the explicit heat scheme with `r ≤ 1/4` (2D) or `r ≤ 1/6` (3D), with classical and learned regions in any arrangement of rectangular blocks, the step with exact training has `‖Φ‖∞ ≤ 1`, so the error after `n` steps is at most the initial error plus `n·δ`. See [below](#in-2d-and-3d). |
| **Derived under idealisation** (training converges exactly) | Classical and learned regions mixed in any arrangement: for `r ≤ 1/2`, one step never increases the error (`B` is diagonally dominant with non-positive off-diagonals, so `B⁻¹ ≥ 0`). Learned regions alone: stability limit `r = (7 − √17)/4 ≈ 0.719`. |
| **Checked numerically** | Wave equation on bounded grids: limit 0.70765 with fixed-value walls, `1/√2` with zero-gradient and absorbing walls, matching where the simulator starts to diverge; growth factors match theory to 5–6 digits. Transparent borders: limits 0.500 / 0.319 / 0.438 depending on the variant. The `γ > 2` boundary matches experiments. |
| **Open** | Why the learned solver's measured limit lies above the idealised 0.719; a proof that transparent borders never create heat; matching the mixed-solver predictions in the simulator and on hardware. |

## Learned regions that leave a residual

The mixed result above assumes that training converges exactly in every step. In practice each step of a learned
region leaves a small residual. Let `Φ` be the step with exact training (classical points explicit, learned
interiors solved exactly by backward Euler), and let the computed values satisfy `u(t) = Φ·u(t−1) + ρ(t)`, where the
*one-step defect* `ρ(t)` is the difference between what the learned solver produced and the exact step from the same
previous values. The exact sequence satisfies `v(t) = Φ·v(t−1)`. Then

```
u(t) − v(t) = Φ·(u(t−1) − v(t−1)) + ρ(t)   ⇒   ‖u(n) − v(n)‖∞ ≤ ‖u(0) − v(0)‖∞ + Σₜ ‖ρ(t)‖∞ ≤ ‖u(0) − v(0)‖∞ + n·δ
```

whenever `‖Φ‖∞ ≤ 1` (proven for `r ≤ 1/2`, any arrangement of classical and learned regions, fixed-value or
zero-gradient ends, nudging with `0 ≤ γ ≤ 2`) and every `‖ρ(t)‖∞ ≤ δ`.

The defect is bounded by what training controls. Inside a learned region the exact values `w*` solve `B·w* = b`; the
computed values solve `B·w = b + q`, where `q` is the backward-Euler residual at the grid points. Hence
`w − w* = B⁻¹·q`, and `‖B⁻¹‖∞ ≤ 1` (each row of `B` has `1 + 2r` on the diagonal and `−r` beside it, so it exceeds
the off-diagonal sum by 1). Values the region takes as targets at its ends add their own deviation. So
`‖ρ(t)‖∞ ≤` (largest deviation at the ends) `+ ‖q‖∞`.

The bound grows linearly with the number of steps and cannot be improved in general: with zero-gradient ends `Φ`
keeps constant fields, and a constant defect `δ` in every point gives an error of exactly `n·δ`. It says that the
error does not grow faster than the residuals add up — not that it stays small.

Checked in `scripts/ci-check.sh` (step 23, `theory/guarantee/residual_bound.py`):

- on the idealised map of several mixed chains with `r = 0.1, 0.3, 0.5`, `‖Φ‖∞ ≤ 1` and 200 steps with random and
  constant defects stay under the bound, and the constant defect on one learned region with zero-gradient ends
  reaches it (0.2031 against 0.2097, the gap being the part of the initial difference that has decayed);
- the learned solver (`crates/heat1d-pinn-actor`) running in the simulator on one device (17 points, fixed-value ends):
  its distance from the exact backward-Euler sequence started at the same field stays under the sum of its measured
  one-step defects (0.037 against 0.153 after 39 steps), and each defect is at most the end deviation plus the grid
  residual;
- the same solver on four devices in the simulator (65 points split 17, 16, 16, 16, fixed-value ends, `r = 0.041`).
  The idealised map uses the simulator's split, and each device's border points target the explicit step from the
  neighbour's previous value, as the solver does. The distance stays under the sum of the defects
  (5.1 × 10⁻⁵ against 1.4 × 10⁻⁴ after 39 steps). In every step and every device, the defect is at most the largest
  deviation at the device's ends plus its grid residual; the defect reaches up to 0.87 of that sum, so the check is
  not loose. The deviation at the borders between devices is small (median 1.9 × 10⁻⁷), which shows that the
  simulator's border points follow the idealised map.

### In 2D and 3D

The argument above uses only `‖Φ‖∞ ≤ 1`, so it carries over once that is shown for the scheme in `d` dimensions
(`d = 2` or `3`; the same `h` in every direction). Each point has `2d` neighbours. Classical points, and points of a
learned region that have a neighbour in another region, are updated explicitly:
`u_i(t) = (1 − 2d·r)·u_i(t−1) + r·Σ_j u_j(t−1)`. Their coefficients are non-negative when `r ≤ 1/(2d)` and sum to at
most 1 (exactly 1 with zero-gradient ends, where the missing neighbour mirrors the one inside; less with fixed-value
ends). The other points of a learned region solve backward Euler with the region's border points as targets:
`B·w(t) = w(t−1) + r·(sum of border neighbours at t)`, where `B` has `1 + 2d·r` on the diagonal and `−r` for every
neighbour inside the region. `B` is diagonally dominant with non-positive off-diagonals, so `B⁻¹ ≥ 0` elementwise.
If `c_i` counts point `i`'s neighbours that are border points of the region, then `B·1 ≥ 1 + r·c` elementwise
(equality without fixed-value ends), so `B⁻¹·(1 + r·c) ≤ 1`. The new values are therefore a combination, with
non-negative weights summing to at most 1, of previous values and border values, which themselves have row sums at
most 1. Hence `‖Φ‖∞ ≤ 1` for `r ≤ 1/(2d)`, in any arrangement of rectangular regions, and nudging with
`0 ≤ γ ≤ 2` keeps it. The bound `‖u(n) − v(n)‖∞ ≤ ‖u(0) − v(0)‖∞ + n·δ` and the bound on `δ` (with `‖B⁻¹‖∞ ≤ 1`,
since every row of `B` exceeds its off-diagonal sum by at least 1) follow as in 1D. The limit `1/(2d)` is sharp for
classical points: just above it, `|1 − 2d·r| + 2d·r > 1`.

Checked in `scripts/ci-check.sh` (step 23, `theory/guarantee/heat_nd.py` and `residual_bound.py`): on a 20 × 20
and an 8 × 8 × 8 grid, with checkerboards of classical and learned regions (also uneven sizes), all-learned
splits, nudging with `γ = 1.5`, fixed-value and zero-gradient ends, and `r` up to `1/4` and `1/6`, `‖Φ‖∞ ≤ 1`,
200 steps with random and constant defects stay under the bound, and the constant defect on one learned region with
zero-gradient ends reaches it (2D: 0.2004 against 0.2100; 3D: 0.1997 against 0.2100). Just above the limit
(`r = 1.05/(2d)`), a classical grid has `‖Φ‖∞ = 1.1`.

This is a result on the idealised map (training converges exactly up to the defect `δ`); the 2D and 3D learned
solvers themselves have not been run in the simulator.

Not covered: classical and learned devices mixed in the simulator (the simulator runs one solver on every device),
learned solvers in 2D and 3D, the wave equation (it needs an energy norm instead of
the maximum norm), and transparent borders.

## Transparent borders

Where a region borders space that no device observes, Dyne can close it with a *transparent* border: the material is
assumed to continue unchanged to infinity. The heat flowing out then depends on the whole history `g(t)` of the
border temperature, `−∂u/∂x(0,t) = (1/√(πα)) ∫₀ᵗ g′(τ)/√(t−τ) dτ`. Dyne approximates `1/√t` by a sum of `M`
exponentials, so each step needs constant work and memory (`M = 32`: relative error 7.7 × 10⁻⁴; devices use
`M = 25`). The assumption breaks once heat reaches beyond the unobserved width.

The derivations and experiments are recorded (in Japanese) in `docs/2026-11-08-stability-theory-primer.md`,
`docs/2026-11-08-stability-checker.md`, `docs/2026-11-11-wave2d.md` and `docs/2026-09-13-transparent-boundary-derivation.md`.

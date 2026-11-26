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
  residual.

Not covered: several learned devices in the simulator (the simulator's arrangement of border points has not yet been
matched to the idealised map), the wave equation, and transparent borders.

## Transparent borders

Where a region borders space that no device observes, Dyne can close it with a *transparent* border: the material is
assumed to continue unchanged to infinity. The heat flowing out then depends on the whole history `g(t)` of the
border temperature, `−∂u/∂x(0,t) = (1/√(πα)) ∫₀ᵗ g′(τ)/√(t−τ) dτ`. Dyne approximates `1/√t` by a sum of `M`
exponentials, so each step needs constant work and memory (`M = 32`: relative error 7.7 × 10⁻⁴; devices use
`M = 25`). The assumption breaks once heat reaches beyond the unobserved width.

The derivations and experiments are recorded (in Japanese) in `docs/2026-11-08-stability-theory-primer.md`,
`docs/2026-11-08-stability-checker.md`, `docs/2026-11-11-wave2d.md` and `docs/2026-09-13-transparent-boundary-derivation.md`.

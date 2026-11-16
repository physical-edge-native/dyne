# Design record: wider borders exchange directly with diagonal neighbours

- **Date decided:** 2026-11-16 (provisional; may be revisited when the scheme is brought to Dyne's devices)
- **Decided by:** the maintainer
- **Issue:** physical-edge-native/barye#6 (keep scaling when the wave update is limited by memory bandwidth)
- **Moved from:** physical-edge-native/barye@760cd20 on 2026-11-16 (issue #38); command names and paths
  are the ones used in this repository.

## Context

With k border layers and one exchange every k steps, the dependence cone of the 5-point (2D) or
7-point (3D) stencil reaches data owned by neighbours that share only an edge or a corner with a patch.
Those values have to arrive somehow. The leapfrog scheme also needs the previous time level in the
overlap, so a message carries k layers of the current level and k − 1 layers of the previous one
(one IFACE_BAND message; for k = 1 the message is byte-identical to before).

## Decision

**Exchange directly with diagonal neighbours** (up to 8 neighbours in 2D, 26 in 3D), in one round per
exchange. Diagonal links are numbered after all face interfaces, so existing interface numbers do not
change. The default stays k = 1.

## Alternatives considered

- **Exchange in axis order** (x faces, then y faces, then z faces), carrying corner values through face
  neighbours. Only face neighbours are contacted, which fits Dyne's devices better (no radio link to
  diagonal neighbours is needed). But each exchange then waits once per dimension, so the number of
  waits drops by D/k instead of 1/k — no reduction at all for k = 2 in 2D. Since the purpose of physical-edge-native/barye#6 is
  fewer waits, this was not chosen.

## Consequences

- Runs with k ≥ 2 cannot include Dyne's WebAssembly patches, which only handle k = 1.
- Bit-exactness against the single-patch solve holds for every k (the arithmetic per point is unchanged).
- On the development laptop (battery, preliminary), k > 1 did not speed things up: in 2D the difference
  is within the noise, and in 3D it is slower because of the redundant work in the overlap and the larger
  messages. The measurements and the analysis are in `docs/2026-11-16-wide-halo.md`. The option is kept
  because the same diagonal exchange is needed by other stencils (e.g. lattice Boltzmann streaming along
  edge directions) and by temporal blocking.
- If the scheme is later brought to Dyne's devices, the axis-order exchange should be reconsidered.

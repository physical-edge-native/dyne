# Design record: a native patch solver that reproduces Dyne's WebAssembly solver bit for bit

- **Date decided:** 2026-11-16
- **Decided by:** the maintainer
- **Issue:** physical-edge-native/barye#2 (first development milestone, 2D wave equation)
- **Moved from:** physical-edge-native/barye@760cd20 on 2026-11-16 (issue #40), where the native solvers
  lived in a separate repository (Barye) that depended on Dyne's crates pinned to one commit. Command
  names, paths and the wording about the pinned commit are updated for this repository.

## Context

Dyne runs its local solvers as WebAssembly modules, so that the same binary runs on microcontrollers
and in Dyne's simulators. The native solvers (`crates/dyne-native`) share Dyne's border-exchange format and solver interface. For the
first milestone (the 2D wave equation split into patches, one thread per patch) we had to choose how a
patch is advanced on a PC:

- **(a)** run Dyne's WebAssembly wave solver (`wave2d-actor`) in an interpreter (`wasmi`), or
- **(b)** write a native Rust solver that performs the same floating-point operations in the same order.

## Decision

**(b) is the main path; (a) is kept as an option** (`dyne-native-wave2d --wasm`) and as a check.

The equality of (b) and (a) is guaranteed in two steps, both run in CI:

1. Without WebAssembly, on every OS: the native solver reproduces the ten field hashes (CRC-32 of the
   field bits) that Dyne recorded for its own wave scenarios.
2. With WebAssembly: CI builds `crates/wave2d-actor` in the same repository and compares the fields
   bit for bit and every border message byte for byte (single patch and a 3×2 split, four boundary
   conditions).

To keep the operations identical, the native solver fixes the order of the Laplacian terms and of the
update, computes the coefficients once at initialization, uses the pure-Rust `libm` for initial
conditions (as Dyne does), and applies the outer boundaries in Dyne's order. Rust does not fuse
multiply and add on its own, so auto-vectorization does not change the per-point operations.

## Reasons

- Measuring parallel efficiency is a goal of the milestone. Under the interpreter, a step is about 80 to
  120 times slower (192×192 patch: 3.3 ms against 0.03–0.04 ms per step), so interpretation would
  dominate and the ratio of computation to communication would not resemble a real deployment.
- Dyne's WebAssembly solver allocates its grid statically (at most 196 points per side), which limits
  the patch size; the native solver has no such limit.
- The native path needs no WebAssembly toolchain for users who only run the native solvers.
- Keeping (a) and the byte-level comparison means a patch on a PC and a patch on a Dyne device are
  still known to compute the same numbers.

## Alternatives considered

- **(a) only:** one binary everywhere, but too slow for scaling measurements and limited in patch size.
- **A faster WebAssembly runtime (ahead-of-time or just-in-time compilation):** would narrow the speed
  gap, but adds a heavy dependency and does not remove the patch-size limit; not needed while (b)
  is checked against (a).
- **(b) without the comparison:** simpler, but the native solver could drift from Dyne's without anyone
  noticing.

## Consequences

- Every change to Dyne's wave solver that changes its bits must be mirrored in the native solver; the
  CI comparison fails otherwise.
- A 3D solver will follow the same pattern: native first, with a bit-level comparison once Dyne has a
  3D WebAssembly solver interface (not yet defined).

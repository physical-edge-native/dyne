# Design record: native patch solvers on a PC, moved in from barye

- **Date decided:** 2026-11-16
- **Decided by:** the maintainer
- **Issue:** [#34](https://github.com/physical-edge-native/dyne/issues/34)
- **Related:** [#32](https://github.com/physical-edge-native/dyne/issues/32) (central compute may be
  borrowed only for forecasting)

This is the first record in `docs/design/`: short English records of design decisions (what was
decided, why, what else was considered, and when). Internal working notes stay in `docs/` in Japanese.

## Context

Dyne runs numerical solvers on a swarm of microcontrollers. The computational domain is split into
blocks, one block per device; each device advances its block with a small solver compiled to
WebAssembly, and devices exchange only the values at the borders between their blocks. The acceptance
criterion is bit-exactness: the split solution equals the single-device solution bit for bit.

A sibling repository, [physical-edge-native/barye](https://github.com/physical-edge-native/barye),
took the same scheme to ordinary CPU cores: native Rust solvers for the 2D and 3D wave equations, one
thread per block, exchanging border messages in Dyne's wire format and bit-exact against the
single-block solve and against Dyne's WebAssembly wave solver. It depended on Dyne's crates
(`dyne-core`, `dyne-wire`) through a git dependency pinned to one commit.

barye was positioned as a field-physics package on CPUs that complements robot simulators. Before
adding more physics, the maintainer reviewed that positioning against existing tools.

## Findings that led to the decision

- **As a CPU field-physics package, barye has no reason to exist on its own.** Established open-source
  tools already cover field physics on CPUs at any scale: general finite-volume CFD suites (for example
  OpenFOAM), parallel libraries for partial differential equations with domain decomposition (for
  example PETSc), and lattice Boltzmann frameworks (for example waLBerla). They are faster, more
  complete and widely validated. Splitting a domain into blocks that exchange borders is standard
  practice there, not a new idea.
- **What is new lies in Dyne's theme**: a swarm of small devices that compute physics together, with
  devices joining, leaving and losing messages, and with results that do not depend on the split. The
  native solvers are useful precisely as part of that theme, not beside it.
- **Emulating devices needs Dyne's measurements.** A PC model of how a swarm behaves (speed per
  device, message delays, losses) is only credible when its parameters come from measured devices, and
  those measurements are made in this repository.

## Decision

1. **barye is integrated into Dyne** as one new crate, `crates/dyne-native/` (binaries
   `dyne-native-wave2d` and `dyne-native-wave3d`). barye is not developed further as a separate
   project. Archiving the barye repository is left to the maintainer.
2. **The native solvers are not a general field-physics package.** Their roles are:
   - **(a) reference solver** for verifying the swarm: the native solvers give the same bits as Dyne's
     WebAssembly solvers, so a swarm run can be checked against a fast PC run;
   - **(b) fast native backend** for simulating many nodes: the native solver is about 80 to 120 times
     faster than interpreting the WebAssembly solver on a PC, so swarms of many blocks can be simulated
     in reasonable time;
   - **(c) later: device-parameterized emulation**, where per-device speed, delays and losses come from
     Dyne's hardware measurements, so that claims about a swarm can be stated under explicit
     conditions;
   - **(d) later: on-demand central forecasting** ([#32](https://github.com/physical-edge-native/dyne/issues/32)):
     when a forecast faster than real time is needed, a PC joins temporarily as a large node, receives
     the assimilated field state (not raw sensor data), computes and leaves; because the PC runs a
     solver with the same bits, hand-over between the swarm and the PC keeps the guarantees.
3. **Scope that barye had and Dyne does not take over as a goal**: competing on general field
   physics, adding physics for its own sake, and throughput comparisons with other simulators.

## Guarantees and their fault model

The bit-exactness guarantees (of the swarm and of the native solvers) hold under this fault model:

- **All devices are honest.** The faults considered are message loss, message delay, devices pausing
  and devices leaving. A device that sends wrong values on purpose, or that is controlled by another
  owner, is not considered.
- **Trust between devices of different owners is future work.** It needs a different fault model
  (devices that may lie) and is not covered by the current checks.
- **Bit-exactness under loss is obtained by waiting.** A block does not advance until it has its
  neighbours' border values for the step; lost messages are sent again after a timeout. Results therefore do not
  change under loss, but each lost message adds delay. Nothing is approximated to hide a loss.

## Alternatives considered

- **Keep barye as a separate repository** depending on Dyne's crates: rejected, because its only
  distinct purpose is part of Dyne's theme, and two repositories double the work of keeping the pinned
  dependency, CI and documents in step.
- **Develop barye as a general field-physics package**: rejected for the reasons above (established
  tools cover it better).
- **Drop the native solvers and keep only the WebAssembly solvers in Dyne's simulators**: rejected,
  because roles (a), (b) and (d) need a fast native solver with the same bits.

## How the history is carried over

The code is moved as **new, small commits** in this repository, one pull request per step, rather than
by merging barye's git history. Each commit and pull request states where the code comes from (for
example "Moved from physical-edge-native/barye@760cd20 (physical-edge-native/barye#12)") and links the
original commits. Reasons: the moved code changes names (crate, binaries, environment variables) and
its CI integration, so the barye history would not build as is in this repository; small reviewed
steps keep this repository's history readable; barye's repository stays public, so its full history
remains available. Only identifiers, strings and documentation comments change; numerical code is
moved as is, and the pinned field hashes (`18e44331` for the 2D check, `2ca26bb5` for the 3D check)
must stay the same.

## File map

| barye (at `760cd20`) | Dyne |
|---|---|
| `crates/barye-core/src/lib.rs`, `decomp.rs`, `patch.rs` | `crates/dyne-native/src/` (same names) |
| `crates/barye-core/src/halo.rs`, `wave2d.rs`, `wasm2d.rs`, `wave3d.rs`, `convergence.rs` | `crates/dyne-native/src/` (same names) |
| `crates/barye-core/src/bin/barye-wave2d.rs`, `barye-wave3d.rs` | `crates/dyne-native/src/bin/dyne-native-wave2d.rs`, `dyne-native-wave3d.rs` |
| `crates/barye-core/tests/dyne_dependency.rs` | `crates/dyne-native/tests/wire_contract.rs` |
| `crates/barye-core/tests/wave2d_split.rs`, `wave3d_split.rs`, `wasm_actor.rs`, `convergence.rs` | `crates/dyne-native/tests/` (same names) |
| `scripts/ci-check.sh` (its steps) | steps added to `scripts/ci-check.sh` |
| `docs/design/2026-11-15-positioning.md` | `docs/design/` (marked as superseded by this record) |
| `docs/design/2026-11-16-native-patch-solver.md`, `2026-11-16-wide-borders-corners.md` | `docs/design/` |
| `docs/2026-11-16-wave2d-milestone.md`, `wide-halo.md`, `wave3d.md`, `convergence.md` | `docs/` (internal notes, Japanese) |

Renamed identifiers: crate `barye-core` → `dyne-native` (`barye_core` → `dyne_native`), binaries
`barye-wave2d`/`barye-wave3d` → `dyne-native-wave2d`/`dyne-native-wave3d`, environment variable
`BARYE_WAVE2D_WASM` → `DYNE_WAVE2D_WASM`. The WebAssembly comparison now uses `crates/wave2d-actor`
built in this repository instead of a pinned commit.

## Follow-up issues

- From barye, transferred to this repository: re-measure the 2D scaling on AC power
  (physical-edge-native/barye#7) and measure where each block's step time goes (compute, waiting,
  message handling; physical-edge-native/barye#13).
- Closed in barye: temporal blocking (physical-edge-native/barye#14, deprioritised; to be re-opened
  here if needed) and the path to 3D (physical-edge-native/barye#3, superseded; its scope is to be
  re-created here from the viewpoint of swarm verification and forecasting).
- To be opened when the work starts: device-parameterized emulation (role c); hand-over of the field
  state between the swarm and a PC for forecasting (role d); a 3D interface for WebAssembly solvers so
  that the 3D native solver can be compared with a device solver as the 2D one is.

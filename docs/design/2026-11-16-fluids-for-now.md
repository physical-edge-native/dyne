# Design record: how Dyne handles fluids for now

- **Date decided:** 2026-11-16
- **Decided by:** the maintainer
- **Issues:** #61 (this record), #57 (export and import), #60 (feasibility of flow on devices)

## Context

When the separate CPU simulation environment was merged into Dyne
(`2026-11-16-native-backend-from-barye.md`), Dyne gave up the aim of being a general field-physics
package that competes with computational fluid dynamics (CFD) packages such as OpenFOAM. That
raised the question whether Dyne gives up on fluids altogether. It does not.

## Decision

1. **For now, the airflow is given from outside.** The first assimilated 3D field is the
   advection–diffusion of quantities carried by the air (temperature, CO2, humidity). The velocity
   field is imported, first from an OpenFOAM computation mapped onto Dyne's regular grid (#57).
   The devices assimilate the transported quantities with their own sensors.
2. **Whether the devices can solve flow themselves is studied separately (#60)**, and not only for
   indoor air. Candidates: air (lattice Boltzmann method), water on surfaces (shallow-water
   equations), flow through soil (Darcy/Richards equations), pipe networks, smoke.

## Reasons

- Import plus export makes the connection to established CFD tools work in both directions,
  without competing with them.
- Solving flow on devices fits Dyne's subject (devices splitting the physics through neighbour
  exchange) best, but needs a feasibility study first:
  - **Locality.** Incompressible solvers usually solve a global pressure equation, which breaks
    neighbour-only exchange. Lattice Boltzmann, artificial compressibility and explicit
    shallow-water or porous-flow schemes stay local.
  - **Network speed limit.** Region size ÷ exchange delay must exceed the fastest propagation
    speed in the scheme. Indoor air moves at about 0.1–1 m/s; even the artificial sound speed of a
    lattice Boltzmann scheme (typically about ten times the flow speed) appears to stay well below
    the roughly 150 m/s estimated from the measured delays so far — an estimate to be checked.
    Water hammer in pipes (about 1000 m/s) is expected to fail.
  - **Memory and step time** on microcontrollers (e.g. 19 values per point for D3Q19 lattice
    Boltzmann).
  - **Stability guarantees** are only partial for nonlinear flow.

## Alternatives considered

- **Solve flow on the devices first.** Closest to the subject, but the risks above are unmeasured;
  postponed to the feasibility study.
- **Estimate flow from sparse velocity sensors.** Matches real data but gives a coarse field;
  may complement the imported flow later.
- **Leave fluids out.** Rejected: transported quantities in air are the most natural slow fields
  for a room, and flow on devices may turn out feasible.

# Design record: positioning against existing simulators

> **Superseded by [2026-11-16-native-backend-from-barye.md](2026-11-16-native-backend-from-barye.md).**
> This record was written for the separate barye repository (physical-edge-native/barye@760cd20) and
> is kept as history. Its decision (barye as a field-physics package on CPUs next to robot simulators)
> no longer holds: the native solvers are now part of Dyne, with the roles stated in the newer record.

- **Date decided:** 2026-11-15
- **Decided by:** the maintainer
- **Issue:** physical-edge-native/barye#1 (project foundations)
- **Source of the survey:** Dyne's
  [`reference/physics_simulators_comparison.md`](https://github.com/physical-edge-native/dyne/blob/main/reference/physics_simulators_comparison.md)
  (Japanese; a survey of robot simulators as of 2026-11: Isaac Sim / Isaac Lab with PhysX and Newton,
  MuJoCo / MJX / MuJoCo Warp, Genesis, ManiSkill3, Gazebo, Webots, CoppeliaSim, PyBullet, Drake, Brax,
  RaiSim, Project Chrono). Support levels change quickly between versions; the summary below is only as
  current as that survey.

## Context

Barye splits a physical simulation into many small patches. Each patch is advanced by its own local
solver (classical or learned), and patches cooperate only by exchanging values at their borders. It
runs on ordinary CPU cores and shares its border-exchange format and solver interface with Dyne, which
runs the same scheme on microcontrollers.

Before writing the first solver, we need to decide what Barye is *for* relative to the simulators that
people already use, so that later choices (which physics, which benchmarks, which dimension) follow
from one written decision.

## What existing simulators cover (summary of the survey)

- **Rigid bodies, articulation and contact** are handled by all of them; Isaac Lab (PhysX, Newton),
  MuJoCo, RaiSim and Drake lead in accuracy and speed.
- **Deformable bodies and particle materials** (soft bodies, cloth, cables, granular media via the
  material point method, particle fluids) are strongest in Genesis, Newton and Project Chrono.
- **Field physics** — fluid flow solved from the Navier–Stokes equations, heat conduction,
  electromagnetism — is **outside the scope of almost all robot simulators**. Gazebo, MuJoCo and Webots
  offer approximate fluid *forces* (buoyancy, drag, lift, wind) without solving the flow. Users who need
  the fields couple to dedicated tools such as OpenFOAM, ANSYS or COMSOL.
- **Differentiability** is offered by Brax, MJX, Newton (some solvers), Genesis (some solvers) and Drake.
- **Compute:** GPU simulators (Isaac Lab, MuJoCo Warp, ManiSkill3's GPU mode) run thousands to tens of
  thousands of rigid-body environments per GPU for reinforcement learning; deformables and fluids cost
  tens to hundreds of times more. CPU simulators (MuJoCo, Gazebo, Webots, Drake) are light per
  environment but not built for mass sample collection.

## Decision

1. **Barye complements robot simulators; it does not replace them.** The gap it addresses is field
   physics next to a robot simulator, not the robot itself.
2. **The goal changes.** Barye no longer aims to "approach Isaac Lab without a GPU" in speed. Throughput
   comparisons against GPU simulators are not a goal or a claim.
3. **The target is 3D physics, as in other simulators.** 2D problems are development milestones only;
   designs that cannot extend to 3D (patch layout, border exchange, stability arguments) are not chosen.
4. Mixing measured patches (patches whose values come from sensors) is not a Barye goal; it stays with
   Dyne.

## Intended strengths

- **Field physics on CPUs:** wave propagation (vibration fields), heat, and coarse airflow via the
  lattice Boltzmann method — the area that the survey shows robot simulators leave out.
- **Bit-exact results across hardware:** a solution split into patches matches the single-patch
  solution bit for bit, on Linux, macOS and Windows (the acceptance criterion inherited from Dyne).
- **Mixed classical and learned patches** with explicit stability conditions at the borders between
  them, so a learned solver can replace part of the domain with known guarantees (or with a stated
  limit where only partial guarantees exist).
- **The same patch solvers run on Dyne devices**, because the solver interface (WebAssembly) and the
  border-exchange format are shared with Dyne rather than copied.

## Known weaknesses (not goals for now)

- **Rigid-body contact and articulated robots:** use Isaac Lab, MuJoCo, Drake or similar.
- **Reinforcement-learning throughput:** GPU simulators run far more environments per second.
- **Rendering and sensor simulation** (cameras, depth, LiDAR): none.
- **Ecosystem:** no task suites, robot models or user community yet.
- **Fluids:** the first fluid solvers will be limited (e.g. coarse lattice Boltzmann flow); there is no
  general incompressible Navier–Stokes solver, and no free-surface or particle fluids.
- **No differentiability for now.** Because each patch only talks to its neighbours, the adjoint
  (reverse-mode gradient) of a step is also local, so gradients could be added patch by patch later.
  This is a possibility, not a plan.

## Alternatives considered

- **Compete with GPU robot simulators on speed without a GPU** (the previous framing in the README):
  dropped, because the survey shows the gap Barye can fill is field physics, not rigid-body throughput,
  and a speed race against GPU simulators is not a meaningful target for CPU patches.
- **Stay 2D:** rejected; other simulators work in 3D, and a 2D-only tool would not be usable next to
  them. 2D remains useful for small, fast development steps.

## Consequences

- First problems are chosen from field physics (e.g. the wave equation), first in 2D as a milestone,
  with a path to 3D stated in each design.
- Benchmarks compare accuracy against reference solvers and bit-exactness across platforms, not
  environment throughput against GPU simulators.
- Coupling with robot simulators (e.g. exporting fields to formats they read) is a natural later step.

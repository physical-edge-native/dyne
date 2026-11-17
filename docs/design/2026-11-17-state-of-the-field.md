# Design record: state of the field — what Dyne overlaps with and what it adds

- **Date written:** 2026-11-17
- **Decided by:** the maintainer (which claims to make, and where); this record lists the material
- **Issue:** #68
- **Purpose:** comparison targets for the "State of the field" section of a future JOSS paper. The
  survey behind it was a few searches per topic, read mostly from abstracts and documentation; it is
  not exhaustive, and "not found" does not mean "does not exist".

## Summary

Every element Dyne uses has prior work. What remains is a combination, and part of it is not built or
not proved yet. The paper should say so plainly.

| Candidate contribution | Verdict | Status in Dyne |
|---|---|---|
| A. Verification method: bit-for-bit equality with a single-solver reference as the pass criterion, for a physical model split over devices on a lossy network that join and leave, with the same solver binary in a deterministic simulator and on hardware, and replay of recorded sensor input | Partly remains, as a combination only | Built and tested in CI (simulator, replay on synthetic records); small hardware runs (2 and 4 ESP32) |
| B. Stability theory for borders between learned and classical solvers, when the assignment changes while running and exchanges are delayed | Open; no theorem yet | Experimental stability maps and unproved conjectures only |
| C. Placement by physical layout: cutting the domain to match device positions, what each device senses, and radio neighbourhood | Would partly remain, but not built | Regions are assigned by hand in the configuration |

## A. Verification method

| Comparison target | What overlaps | What Dyne adds |
|---|---|---|
| CIME test system used by E3SM and CESM: PEM (change the number of MPI tasks, compare bit for bit), ERS (exact restart) — <https://esmci.github.io/cime/versions/master/users_guide/testing.html> | The pass criterion itself: a run split differently must be bit-for-bit identical. Routine in climate-model regression testing. | Devices on a lossy network, joining and leaving *while running* and taking over a neighbour's state; the same criterion applied on microcontrollers. ERS changes the task count only between stop and restart. |
| Hallberg and Adcroft, "An order-invariant real-to-integer conversion sum", *Parallel Computing* 40, 2014 | Reproducibility regardless of decomposition. | Nothing new here: Dyne's finite-difference splits keep the order of operations, so the problem this paper solves does not arise. |
| FoundationDB (Zhou et al., *ACM SIGMOD* 2021); TigerBeetle VOPR (<https://docs.tigerbeetle.com/about/vopr>) | Deterministic simulation testing: time, network and faults controlled from a seed; production code run in the simulator; failures reproducible from the seed. | Pass criterion is equality with a numerical reference instead of database invariants; simulator results are compared with hardware results. |
| Renode (Antmicro, <https://renode.io>) | Several embedded devices, including radio, emulated deterministically under one virtual clock with the same binaries. | Not more faithful: Renode emulates whole devices at instruction level, Dyne simulates only the WebAssembly solver (not firmware or radio stack). Dyne adds the numerical reference as the standard pass criterion. |
| Record and replay for wireless sensor networks (several systems; bibliographic details to be checked before citing) | Recording non-deterministic inputs of sensor nodes and replaying them. | Purpose: deciding correctness of a split numerical computation rather than debugging. |
| WebAssembly runtimes on microcontrollers (WAMR and others) | Dyne runs its solvers on WAMR. | Dyne is a user of these runtimes, not a competitor. |

Determinism caveat that the paper must state: WebAssembly leaves NaN bit patterns and relaxed-SIMD
instructions non-deterministic, so "same binary, same bits" holds only for solvers that avoid them.

Honest limits of A: bit-exactness under loss is obtained by waiting for missing messages (loss
becomes delay), a known design; the novelty is not the waiting but checking the property
systematically with seeded faults. No evidence was found yet of users who need to verify physical
computations on microcontroller swarms; demand can be stated only as anticipated.

## B. Border stability between learned and classical solvers

| Comparison target | What overlaps | What remains open |
|---|---|---|
| Landajuela, "Robin-Neumann Coupling of PINN and FEM Solvers: A Steklov-Poincaré View, with Application to Fluid-Structure Interaction with Contact", arXiv:2606.14181, 2026 | Closest. Couples a physics-informed network and a finite-element subdomain, analyses convergence with domain-decomposition theory, proves a contraction result. | Time-dependent explicit stepping with an exchange every step; assignment of a region (learned or classical) changing while running; delayed exchange. |
| Snyder, Tezaur and Wentland, arXiv:2311.00224, 2023 (Schwarz coupling of PINNs with classical subdomains) | Mixed learned/classical subdomains via the Schwarz method. | Steady 1D problems; no run-time reassignment or delay. |
| Magoulès, Szyld and Venet, *Numerische Mathematik* 137, 2017 (asynchronous optimized Schwarz methods) | Convergence with delayed and out-of-order exchange. | Steady problems, classical solvers on both sides. |
| Classical stability analysis of interface conditions between partitioned solvers (e.g. thermal coupling of fluid and solid solvers; citation to be checked) | The framework Dyne's analysis should build on. | Learned subdomains are not covered. |

Status: Dyne has no theorem here. It has stability maps from experiments (learned solver regions on
many split configurations) and stated conjectures. This item can enter the paper only as an open
question, or after a proof. The field moves fast; others may publish the time-dependent case first.

## C. Placement by physical layout

| Comparison target | What overlaps | What Dyne would add |
|---|---|---|
| DDFlow (Noor, Tseng, Garcia and Srivastava, *ACM/IEEE IoTDI* 2019) | Binds data-flow tasks to spatial regions and places them under network constraints, re-placing when the network changes. | Cutting a partial differential equation's domain itself (region shape, border width, time step) to fit device positions, sensed quantities and radio neighbourhood. |
| Topology-aware mapping in high-performance computing (Hoefler and Snir, *ACM ICS* 2011) | Mapping a graph of subdomains onto a machine's network. | Device positions come first and regions are cut to contain each device's sensors; the machine graph has no notion of what each processor can observe. |
| Aggregate-computing deployment (pulverization), macroprogramming (Regiment), Akri, Brick | Placing computation by location or attached devices. | Same as above; these target coordination, queries or generic workloads, not solver subdomains. |

Status: not built. Region assignment is currently written by hand. It is on the roadmap
("automatic region assignment from device positions and what each device senses"); it cannot be
claimed before it exists.

## Consequences for the paper

- State of the field should cite at least the CIME tests, FoundationDB/TigerBeetle and Renode for A,
  because reviewers are likely to know them and omitting them would overstate novelty.
- Claim A only as a combination; mention B as open and C as planned, or leave them out.
- No claim of superiority over centralised setups.

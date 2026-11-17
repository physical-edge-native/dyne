# Dyne

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.24241009.svg)](https://doi.org/10.5281/zenodo.24241009)

**Dyne splits a physical model across networked devices, runs the same
WebAssembly solver in a deterministic network simulator and on real devices,
and checks the result bit for bit against a single-solver reference** —
including under message loss, delay and devices joining or leaving, with
recorded sensor input that can be replayed.

## What you do with Dyne

Commands are run from the repository root. Steps 1, 2 and 5 need no hardware.

**1. Write a solver against the actor interface and build it to WebAssembly.**
A solver ("actor") is a small `no_std` Rust crate that exports a handful of
functions: `dyne_init`, `dyne_step`, `dyne_set_neighbor` and
`dyne_boundary_value` / `dyne_boundary_grad` (the border values exchanged with
neighbours), `dyne_get` / `dyne_len` (the state) and `dyne_observe` (feeding a
measurement). `crates/heat1d-actor` (1D) and `crates/heat2d-actor` (2D) are
examples.

```sh
rustup target add wasm32-unknown-unknown
(cd crates/heat2d-actor && cargo build --release --target wasm32-unknown-unknown)
```

**2. Check it in the simulator, with faults.** The module runs split over
simulated devices on a discrete-event clock. Loss, delay, jitter, pauses and
network partitions come from a fixed random seed, so every run is reproducible
across operating systems; devices can also join, leave or take over a
neighbour's state. The pass criterion is that the split solution equals the
single-solver solution bit for bit.

```sh
# 2x2 devices, 10% loss, 5 ms delay, device 3 paused, devices 0 and 1 cut off from each other for a while
cargo run --release -p dyne-sim2d -- --wasm crates/heat2d-actor/target/wasm32-unknown-unknown/release/heat2d_actor.wasm \
  --px 2 --py 2 --steps 500 --loss 0.1 --lat-ms 5 --pause 3@2000:6000 --partition 0-1@10000:15000
# 1D, 8 devices: device 3 goes silent at 30 s and is replaced at 60 s by a new instance that takes over its state
(cd crates/heat1d-actor && cargo build --release --target wasm32-unknown-unknown)
cargo run --release -p dyne-sim -- --wasm crates/heat1d-actor/target/wasm32-unknown-unknown/release/heat1d_actor.wasm \
  --nodes 8 --points 65 --steps 2000 --handover 3@30000:60000
# expected in both: "PASS (bit-exact)"
```

**3. Deploy it to ESP32 devices** (needs boards and ESP-IDF 5.5). All devices
run the same firmware (`node-esp32/`) and get their role from configuration at
run time. PC-side tools flash boards one after another and record which board is
which, synchronise clocks over Wi-Fi, replace the solver on all devices over the
air, and send each device its region, neighbours and boundary conditions. The
assignment of regions to devices is currently written by hand.

```sh
powershell -ExecutionPolicy Bypass -File node-esp32/build.ps1 -Board devkitc-ve   # build the firmware
python scripts/first_flash.py                         # flash boards over USB, one after another
python scripts/dyne_ntp_server.py --port 123          # time server on the PC for clock synchronisation
python scripts/dyne_ota.py actor --broker <PC address> --wasm <solver.wasm> --name <solver name> --nodes all
```

**4. Check on hardware against the simulator.** Devices report a CRC-32 of their
state; the run passes only if every device's checksum equals the simulator's for
its region. Sensor readings are one-off inputs, so devices can record the inputs
they hand to the solver, and the recording is replayed in the simulator device by
device and checkpoint by checkpoint, to locate where hardware and simulator part.

```sh
# 2D wave equation on four devices: configure, start, collect checksums, compare with dyne-sim2d
python scripts/wave2d_cluster.py run --broker <PC address> --layout 2x2 --nodes 11,12,13,14
# record a run with sensor input, then replay it in dyne-sim2d (same layout and steps as the run)
python scripts/hw_replay.py record --broker <PC address> --out run.jsonl
python scripts/hw_replay.py check run.jsonl --wasm <solver.wasm> -- <dyne-sim2d arguments of the run>
```

**5. Use the result or hand it over.** Each device keeps its region fitted to its
sensors by nudging (data assimilation in which the model value at a sensor is
pulled towards the measurement at every observation). The state can be exported
to VTK, OpenFOAM and USD (status of each in
[Connecting to other tools](#connecting-to-other-tools)).

```sh
cargo run --release -p dyne-sim2d -- --wasm crates/heat2d-actor/target/wasm32-unknown-unknown/release/heat2d_actor.wasm \
  --px 2 --py 2 --steps 500 --vti heat2d.vti                                     # VTK image data for ParaView
cargo run --release -p dyne-export --bin dyne-foam -- case --vti heat2d.vti --out heat2dCase   # OpenFOAM case
```

`bash scripts/ci-check.sh` runs steps 1, 2, 5 and the replay check of step 4 (on
synthetic recordings) as a regression suite; CI runs it on Linux, macOS and
Windows. The hardware steps 3 and 4 are summarised under
[Verified so far](#verified-so-far) and [Roadmap](#roadmap).

> **Status: research prototype (v0.x).** Interfaces change without notice.
> Development records in `docs/` are written in Japanese.

## About

Each device computes the physics of its own region (for example, heat in part of
a room), exchanges only the values at the borders with its neighbours, and keeps
its region fitted to its own sensor readings. Because the same compiled solver
runs everywhere, a split run on microcontrollers can be compared with a run in
the simulator and with an unsplit reference, digit for digit.

Dyne is a research tool. It is part of the
[Physical-Edge Native Architecture](https://github.com/physical-edge-native)
project; the assumptions behind it (cheap devices of different owners sharing a
space, fields governed by neighbour coupling, radio costing more than local
computation, global reasoning staying with central models) are stated there.

### Why WebAssembly

Benefits:

- **One binary, identical bits.** The same module gives the same bits on an
  Xtensa ESP32 and on an x86 PC. WebAssembly fixes IEEE 754 arithmetic strictly
  (every operation rounded separately, no implicit fused multiply-add), with two
  exceptions the standard leaves non-deterministic: the bit patterns of NaN
  (not-a-number) results and the relaxed-SIMD instructions. Dyne's solvers
  avoid relying on either: they are built without relaxed-SIMD instructions and
  are not designed to produce NaN, and a run that does produce one is outside
  this guarantee.
- **The simulator runs the very same binary** as the devices, so a simulator
  run checks the code that is deployed, not a re-implementation.
- **Isolation.** A solver cannot touch memory outside its own or call anything
  it has not been granted. This matters when devices of different owners host
  each other's solvers.
- **The interface is the border contract.** What a solver exports and imports is
  exactly what it exchanges at its borders, so solvers of different makers or
  kinds (classical or learned) plug into the same place.
- **Replaceable over the air** without reflashing the firmware.

Costs:

- **Interpretation is slow on the ESP32**: about 0.7–1.0 s per step for the
  room-shape solver; on a PC, interpreting is 80–120 times slower than the
  native Rust solvers.
- **64-bit floats are emulated in software** on the ESP32, whose floating-point
  unit handles single precision only.

Splitting a solution does not change its bits because each region performs the
same floating-point operations in the same order as the unsplit solver.

## Limits stated up front

- **A speed limit set by the network.** With neighbour-only exchange,
  information crosses one region per exchange, so following a field in real
  time requires region size ÷ exchange delay to exceed the speed at which the
  physics propagates — a CFL-like condition on the network. With the radio
  delays measured so far this excludes fast phenomena such as sound and leaves
  slow fields (indoor heat, CO2, humidity). Slow fields can often be computed by
  a single device: the value of many devices lies in sensing spread through the
  field, not in raw compute.
- **Bit-exact under loss, at the cost of delay.** Devices wait for missing
  border messages, so loss turns into delay rather than error.
- **Only the solver is simulated.** The simulator runs the same WebAssembly
  solver as the devices, but not their firmware or radio stack; the bit-for-bit
  comparison between hardware and simulator covers the computation, not the
  rest of the device.
- **Honest devices only.** The guarantees assume that every device is honest and
  only faults occur (loss, delay, devices leaving). Authentication and checks
  against implausible border values are future work.
- **No claim of superiority** over centralised or GPU-based setups, and no
  attempt to replace computational fluid dynamics packages: Dyne aims to export
  its assimilated state to such tools for detailed studies.
- **Fluids: flow given from outside, for now.** The airflow is to be imported
  (first from OpenFOAM; the import is implemented but not yet run with
  OpenFOAM) and the devices assimilate what it carries (temperature, CO2,
  humidity). Whether the devices can solve flow themselves — air, water on
  surfaces, flow through soil — with neighbour-only exchange is being studied
  ([design record](docs/design/2026-11-16-fluids-for-now.md)).

## Verified so far

Details and raw data are in `docs/` and `measured/`.

| | Result |
|---|---|
| 1D heat equation, 2–16 devices in the simulator | bit-exact vs single device, also under loss/delay/partitions |
| 2D heat equation, up to 4×4 partitions | bit-exact, second-order convergence vs analytic solution |
| Replay of recorded device input (synthetic records) | replay matches, and a single flipped bit is detected; not yet run with records from hardware |
| ESP32 (Xtensa), WebAssembly interpreter (WAMR) | all digits match the PC result |
| 2 × ESP32 over Wi-Fi | bit-exact split solution |
| 4 × ESP32 on batteries with temperature sensors, 4 runs of ~33 min | all runs completed without restart |

## Quick start (no hardware needed)

Requirements: Rust (stable) with the `wasm32-unknown-unknown` target.

```sh
rustup target add wasm32-unknown-unknown
bash scripts/ci-check.sh          # builds everything and runs the regression suite
```

Or step by step:

```sh
# build the 1D heat-equation solver as a WebAssembly module
(cd crates/heat1d-actor && cargo build --release --target wasm32-unknown-unknown)
# run it split over 16 simulated devices and compare with the single-device solution
cargo run --release -p dyne-sim -- \
  --wasm crates/heat1d-actor/target/wasm32-unknown-unknown/release/heat1d_actor.wasm \
  --nodes 16 --points 65 --steps 2000
# expected: "RESULT: PASS (bit-exact)"
```

Running on ESP32 boards requires ESP-IDF 5.5; see `node-esp32/` and
`docs/2026-09-21-experiment1-runbook.md`.

The English documentation is in [docs/en/](docs/en/README.md): [installation](docs/en/installation.md)
and a [tutorial](docs/en/tutorial.md) that writes a new solver and checks it bit for bit in the simulator.

## Applications and examples built on the core

The following use the same solver interface, simulator and checks. Several are
implemented and awaiting verification on a 16-device hardware setup (see
[Roadmap](#roadmap)).

- **Classical and learned solvers side by side.** Finite-difference solvers and
  a small physics-informed neural network (a network trained so that its output
  satisfies the governing equation) plug into the same interface
  (`crates/heat1d-pinn-actor`). Small networks trained in PyTorch can be
  imported via ONNX.
- **Coupling to measurements.** Nudging (step 5 above) is verified in the
  simulator with a stability map and, on hardware, as a mechanism with
  temperature sensors.
- **Room-shape reconstruction** from range sensors (`crates/sdf2d-grid-actor`,
  simulator stage).
- **Inertial-sensor (IMU) fusion across devices** (`crates/imu-fusion-actor`).
- **Flocking demo** on the same runtime (`crates/swarm-*`).
- **Native solvers on a PC** and **connections to other tools** (VTK, USD,
  OpenFOAM), described below.

### Native solvers on a PC (reference and fast backend)

`crates/dyne-native` runs the same split computation on the threads of one PC, with native Rust
solvers instead of WebAssembly: one thread per patch, exchanging only border values in the same
message format. The 2D wave solver performs the same floating-point operations in the same order as
the WebAssembly wave solver (`crates/wave2d-actor`), so both give identical bits; CI compares the
fields bit for bit and every border message byte for byte. The native solvers serve as a reference
for checking runs on devices and as a fast backend for simulating many devices (on one laptop, about
80–120 times faster per step than interpreting the WebAssembly solver). They are not a general
field-physics package. A 3D wave solver (seven-point stencil, documented order of operations) is
bit-exact across splits as well; it has no WebAssembly counterpart for devices yet.

```sh
# every split must equal the single-patch solution bit for bit
cargo run --release -p dyne-native --bin dyne-native-wave2d -- check --grid 41x41 --steps 400 --splits 1x1,2x2,4x4,8x8,3x5
cargo run --release -p dyne-native --bin dyne-native-wave3d -- check --grid 25x23x21 --steps 100 --splits 1x1x1,2x2x2,3x2x1,4x4x2,1x3x5
# accuracy: observed convergence order against analytic standing waves, which must lie in [1.9, 2.1]
cargo run --release -p dyne-native --bin dyne-native-wave2d -- convergence
```

Both tools also measure parallel scaling (`scaling --mode strong|weak`) and accept wider borders
(`--halo k`: `k` layers exchanged once every `k` steps, still bit-identical). The code was moved from
the [barye](https://github.com/physical-edge-native/barye) repository; the decision and the roles of
these solvers are recorded in
[docs/design/2026-11-16-native-backend-from-barye.md](docs/design/2026-11-16-native-backend-from-barye.md).

### Connecting to other tools

Dyne does not try to replace established simulators; it hands them the state of the
real space that the devices keep up to date.

| Target | What is exported | Use | Status |
|---|---|---|---|
| USD (Isaac Sim / Isaac Lab and other USD-based tools) | room shape and objects, with collision information | robot learning and simulation in a scene that matches the real room | implemented, awaiting hardware verification |
| VTK image data (ParaView, PyVista) | 2D/3D field state | inspection and post-processing without licences; checked bit for bit in CI | implemented (`--vti PATH`) |
| OpenFOAM case (scalarTransportFoam) | grid, initial field, boundary conditions, given velocity and diffusivity | detailed what-if studies with an established CFD package; case format and read-back checked in CI | implemented (`dyne-foam case`); not yet run with an OpenFOAM installation |
| OpenFOAM case with the room's walls and furniture | the cells inside walls and obstacles of the room shape built by the devices are left out; their faces form the patch `walls` | the real room in a CFD study | implemented (`dyne-foam case --shape`); a simulated room passes OpenFOAM's `checkMesh` and runs with `scalarTransportFoam` (OpenFOAM.com v2606 release candidate) |
| OpenFOAM velocity field (import) | airflow computed by OpenFOAM, mapped onto Dyne's grid | the given airflow that carries temperature, CO2 and humidity | implemented (`dyne-foam velocity`, checked in CI on a synthetic file); not yet run with an OpenFOAM installation |

```sh
# write the final field of a run as VTK image data (open it in ParaView), then print each array's CRC-32
cargo run --release -p dyne-native --bin dyne-native-wave2d -- check --grid 41x41 --steps 400 --splits 1x1,2x2 --vti wave2d.vti
cargo run --release -p dyne-export --bin dyne-vti -- info wave2d.vti
```

Values are stored losslessly (raw little-endian 64-bit floats), so the file reads back bit for bit;
`dyne-sim2d --vti PATH` writes the field of a split simulator run in the same way.

OpenFOAM works with cells rather than points: each point of Dyne's grid becomes the centre of one
cell of a single `blockMesh` block, so the values carry over one to one (the domain extends half a
spacing beyond the outer points). Numbers are written in the shortest decimal form that reads back to
the same 64-bit float.

```sh
# Dyne -> OpenFOAM: a scalarTransportFoam case from a .vti field
# (faces west,east,south,north[,bottom,top]; diffusivity --diffusivity, default 2.2e-5 m^2/s)
cargo run --release -p dyne-export --bin dyne-foam -- case --vti wave2d.vti --out wave2dCase --velocity 0.1,0,0 --faces fixed:0,zero-gradient,zero-gradient,zero-gradient
cargo run --release -p dyne-export --bin dyne-foam -- info wave2dCase   # reads it back, prints the CRC-32 of T
# then, with OpenFOAM (written for OpenFOAM.com v2xxx; not yet tried): cd wave2dCase && blockMesh && scalarTransportFoam

# OpenFOAM -> Dyne: map the velocity of a CFD case (roomFlow) onto Dyne's grid, then read it
cargo run --release -p dyne-export --bin dyne-foam -- case --vti wave2d.vti --out flowGrid
(cd flowGrid && blockMesh && mapFields ../roomFlow -sourceTime latestTime)
cargo run --release -p dyne-export --bin dyne-foam -- velocity flowGrid 0 --vti flow.vti
```

With the room shape (`--shape`: the `.vti` of `dyne-sim2d --vti`, or the `shape.csv` that
`scripts/twin_live.py` writes from the devices), the grid points inside walls and furniture get no
cell, and the mesh is written directly as `constant/polyMesh`, so `blockMesh` is not run:

```sh
cargo run --release -p dyne-export --bin dyne-foam -- case --vti room.vti --shape room.vti --out roomCase
cd roomCase && checkMesh && scalarTransportFoam
```

A grid point counts as solid where the signed distance `phi` is negative or where it was not observed
(`--unobserved fluid` treats unobserved points as free), as in the USD export; only the largest
connected free region is kept. The choice of this representation is recorded in
[docs/design/2026-11-18-foam-walls-and-sensor-field.md](docs/design/2026-11-18-foam-walls-and-sensor-field.md).

The other OpenFOAM steps above have not yet been run with an OpenFOAM installation, neither in Dyne's tests nor by
hand: CI checks only the file format and the bit-for-bit read-back. The details, the mapping of boundary
types and the differences in the Foundation releases (openfoam.org) are in the documentation of
`dyne_export::foam` (`cargo doc -p dyne-export --open`). Results of OpenFOAM and Dyne differ by their
discretisation, so comparing them needs a stated tolerance.

## Related work

The individual elements have prior work; Dyne studies their combination on real
devices, checked bit for bit. Where a closer comparison is useful, it
is in [docs/design/2026-11-17-state-of-the-field.md](docs/design/2026-11-17-state-of-the-field.md).

- **Bit-for-bit tests across decompositions.** Climate models routinely check
  that a run split over a different number of processes gives bit-for-bit
  identical results (the PEM test, and the exact-restart test ERS, of the CIME
  test system used by E3SM and CESM;
  <https://esmci.github.io/cime/versions/master/users_guide/testing.html>).
  Dyne applies the same criterion to a solution split over devices that
  exchange messages over a lossy network and may join or leave while running.
  Split solutions that are reproducible regardless of the decomposition are
  also used in ocean and climate models (Hallberg and Adcroft, "An order-invariant real-to-integer conversion sum", *Parallel
  Computing* 40, 2014).
- **Deterministic simulation testing.** FoundationDB (Zhou et al., "FoundationDB:
  A Distributed Unbundled Transactional Key Value Store", *ACM SIGMOD* 2021) and
  TigerBeetle (VOPR, <https://docs.tigerbeetle.com/about/vopr>) run their
  production code in a simulator that controls time, network and faults from a
  seed. Their pass criteria are invariants of a database; Dyne's is equality
  with a numerical reference, and it also compares the simulator with hardware.
- **Multi-node deterministic emulation.** Renode (Antmicro,
  <https://renode.io>) emulates several embedded devices, including their
  radio, deterministically under one virtual clock. It emulates whole devices
  at instruction level and is therefore more faithful than Dyne's simulator,
  which runs only the solver.
- **Placing tasks by space.** DDFlow (Noor, Tseng, Garcia and Srivastava,
  "DDFlow: visualized declarative programming for heterogeneous IoT networks",
  *ACM/IEEE IoTDI* 2019) binds data-flow tasks to spatial regions and places
  them under network constraints. Dyne does not yet place regions
  automatically.
- **Coupling learned and classical subdomains.** Landajuela, "Robin-Neumann
  Coupling of PINN and FEM Solvers: A Steklov-Poincaré View, with Application to
  Fluid-Structure Interaction with Contact" (arXiv:2606.14181, 2026) analyses the
  convergence of coupling a physics-informed network with a finite-element
  subdomain. Dyne mixes such solvers in time-dependent runs, for which it has
  experimental stability maps but no theorem yet.
- Amorphous and aggregate computing program many locally communicating devices
  placed in space (Abelson et al., "Amorphous Computing", *Commun. ACM* 43(5),
  2000; Beal, Pianini and Viroli, "Aggregate Programming for the Internet of
  Things", *IEEE Computer* 48(9), 2015).
- Distributed estimation of fields governed by partial differential equations,
  where each node holds only a local subsystem (Khan and Moura, "Distributing the
  Kalman Filter for Large-Scale Systems", *IEEE Trans. Signal Processing* 56(10),
  2008; Battistelli et al., *IEEE Trans. Automatic Control* 62(7), 2017).
- Indoor airflow and temperature models fitted to sensor data (computational
  fluid dynamics combined with data assimilation) run on one computer that
  collects the data; Dyne keeps the fitted state on the devices.

## Repository layout

| Path | Contents |
|---|---|
| `crates/dyne-wire` | wire format shared by every host (`no_std`) |
| `crates/dyne-core` | deterministic RNG, discrete-event clock, virtual network, WebAssembly loader |
| `crates/dyne-sim`, `crates/dyne-sim2d` | N-device simulators (1D / 2D) |
| `crates/dyne-native` | native PC solvers (2D/3D wave), bit-exact with the WebAssembly solvers; reference and fast backend |
| `crates/dyne-export` | connections to other tools: VTK image data, OpenFOAM cases and velocity import (`dyne-vti`, `dyne-foam`) |
| `crates/heat1d-actor`, `crates/heat2d-actor` | finite-difference solvers compiled to WebAssembly |
| `crates/heat1d-pinn-actor` | learned solver (time-discrete physics-informed network) |
| `crates/sdf2d-grid-actor` | room-shape reconstruction from range sensors (simulator stage) |
| `crates/swarm-*` | flocking demo on the same runtime |
| `crates/dyne-host`, `crates/dyne-proto`, `m1-esp32/`, `m2-esp32/` | earlier milestones, kept for reproducibility |
| `node-esp32/` | current ESP32 firmware (all devices identical; role set at runtime) |
| `vl53-eval/` | evaluation firmware for the VL53L5CX range sensor |
| `theory/`, `reference/` | stability analysis and theoretical values |
| `measured/`, `figs/` | raw measurements from hardware runs and figures |
| `docs/` | design documents, reports and status (Japanese) |
| `scripts/` | PC-side tools: configuration, observation, analysis, regression checks |

## Roadmap

Implemented and awaiting verification on a 16-device hardware setup (2026-11):
time synchronisation over Wi-Fi, over-the-air deployment of WebAssembly modules
to all devices, replay of recorded device input against the simulator, the 2D
wave equation on a device cluster, inertial-sensor (IMU) fusion across devices,
small networks trained in PyTorch imported via ONNX, room-shape reconstruction
from range sensors, and USD export (see
[Connecting to other tools](#connecting-to-other-tools)).

Planned for 2026–2027 (tracked as GitHub issues):
a slow indoor field (temperature, CO2 and humidity carried by a given airflow) as
the first assimilated 3D field; forecasting by handing the devices' state to a PC
with identical bits; simulation of many devices with device capabilities (speed,
memory, radio delay and loss) as parameters, calibrated by hardware
measurements; automatic region assignment from device positions and what each
device senses.

## Contributing

Issues and pull requests are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md).

## How this software is developed

Design decisions, experiments and evaluation are made by the author.
Much of the implementation is written with the help of AI coding assistants
(Anthropic Claude); every change is reviewed and verified by the author against
the bit-exact regression suite above.

## Citing

See [CITATION.cff](CITATION.cff).

## Acknowledgements

This work is supported by the IPA MITOU IT program (FY2026).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <https://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this work
by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

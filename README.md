# Dyne

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.24241009.svg)](https://doi.org/10.5281/zenodo.24241009)

**Dyne keeps a physical model of a space running on the many small devices
placed in it.** Each microcontroller computes the physics of its own region,
exchanges only the values at the borders with its neighbours, and keeps its
region fitted to its own sensor readings. Together the devices hold a live model
of the whole space — a digital twin whose state lives on the devices, not on a
server. When a forecast faster than real time is needed, a PC can join for a
while, continue the same computation from the devices' state with identical
bits, and leave again.

Dyne is a research tool for this setting: write a solver once, run it split over
simulated or real devices, and check the result bit for bit against a
single-solver reference. It is part of the
[Physical-Edge Native Architecture](https://github.com/physical-edge-native)
project; the assumptions behind it (cheap devices of different owners sharing a
space, fields governed by neighbour coupling, radio costing more than local
computation, global reasoning staying with central models) are stated there.

> **Status: research prototype (v0.x).** Interfaces change without notice.
> Development records in `docs/` are written in Japanese.

## What it does

- **Same answer everywhere.** A numerical model is compiled once to
  WebAssembly. The same binary runs on a PC, in the simulator and on an ESP32,
  and a solution split over *N* devices matches the single-device solution
  bit for bit.
- **Deterministic network simulator.** Packet loss, delay, jitter, pauses and
  network partitions are injected with a fixed random seed, so every run is
  reproducible across operating systems. Records of sensor input from hardware
  runs can be replayed in the simulator and checked device by device.
- **Devices can join and leave while running.** A device can be added,
  removed, or handed its neighbour's state during a run, and the solution stays
  identical to a reference computation.
- **Classical and learned solvers side by side.** Finite-difference solvers and
  a small physics-informed neural network (a network trained so that its output
  satisfies the governing equation) plug into the same interface.
- **Coupling to measurements.** Each device feeds its own sensor readings into
  its local model (data assimilation by nudging: the model value at a sensor is
  pulled towards the measurement at every observation). Verified in the
  simulator with a stability map and, on hardware, as a mechanism with
  temperature sensors.
- **Native solvers on a PC.** Native Rust versions of the solvers, bit-identical
  to the WebAssembly ones, serve as references and as a fast backend for
  simulating many devices (below).

## Connecting to other tools

Dyne does not try to replace established simulators; it hands them the state of the
real space that the devices keep up to date.

| Target | What is exported | Use | Status |
|---|---|---|---|
| USD (Isaac Sim / Isaac Lab and other USD-based tools) | room shape and objects, with collision information | robot learning and simulation in a scene that matches the real room | implemented, awaiting hardware verification |
| VTK image data (ParaView, PyVista) | 2D/3D field state | inspection and post-processing without licences; checked bit for bit in CI | planned (#57) |
| OpenFOAM case | grid, initial field and boundary conditions | detailed what-if studies with an established CFD package | planned (#57) |

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
- **Honest devices only.** The guarantees assume that every device is honest and
  only faults occur (loss, delay, devices leaving). Authentication and checks
  against implausible border values are future work.
- **No claim of superiority** over centralised or GPU-based setups, and no
  attempt to replace computational fluid dynamics packages: Dyne aims to export
  its assimilated state to such tools for detailed studies.
- **Fluids: flow given from outside, for now.** The airflow is imported (first
  from OpenFOAM) and the devices assimilate what it carries (temperature, CO2,
  humidity). Whether the devices can solve flow themselves — air, water on
  surfaces, flow through soil — with neighbour-only exchange is being studied
  ([design record](docs/design/2026-11-16-fluids-for-now.md)).

## Related work

The individual elements have prior work; Dyne studies their combination on real
device swarms with numerical guarantees.

- Amorphous and aggregate computing program many locally communicating devices
  placed in space (Abelson et al., "Amorphous Computing", *Commun. ACM* 43(5),
  2000; Beal, Pianini and Viroli, "Aggregate Programming for the Internet of
  Things", *IEEE Computer* 48(9), 2015).
- Distributed estimation of fields governed by partial differential equations,
  where each node holds only a local subsystem (Khan and Moura, "Distributing the
  Kalman Filter for Large-Scale Systems", *IEEE Trans. Signal Processing* 56(10),
  2008; Battistelli et al., *IEEE Trans. Automatic Control* 62(7), 2017).
- Split solutions that are reproducible regardless of the decomposition are used
  in ocean and climate models (Hallberg and Adcroft, "An order-invariant
  real-to-integer conversion sum", *Parallel Computing* 40, 2014).
- Indoor airflow and temperature models fitted to sensor data (computational
  fluid dynamics combined with data assimilation) run on one computer that
  collects the data; Dyne keeps the fitted state on the devices.

Verified so far (details and raw data in `docs/` and `measured/`):

| | Result |
|---|---|
| 1D heat equation, 2–16 devices in the simulator | bit-exact vs single device, also under loss/delay/partitions |
| 2D heat equation, up to 4×4 partitions | bit-exact, second-order convergence vs analytic solution |
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

## Native solvers on a PC (reference and fast backend)

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

## Repository layout

| Path | Contents |
|---|---|
| `crates/dyne-wire` | wire format shared by every host (`no_std`) |
| `crates/dyne-core` | deterministic RNG, discrete-event clock, virtual network, WebAssembly loader |
| `crates/dyne-sim`, `crates/dyne-sim2d` | N-device simulators (1D / 2D) |
| `crates/dyne-native` | native PC solvers (2D/3D wave), bit-exact with the WebAssembly solvers; reference and fast backend |
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
to all devices, the 2D wave equation on a device cluster, inertial-sensor (IMU)
fusion across devices, small networks trained in PyTorch imported via ONNX,
room-shape reconstruction from range sensors, and USD export (see
[Connecting to other tools](#connecting-to-other-tools)).

Planned for 2026–2027 (tracked as GitHub issues):
a slow indoor field (temperature, CO2 and humidity carried by a given airflow) as
the first assimilated 3D field; forecasting by handing the devices' state to a PC
with identical bits; simulation of many devices with device capabilities (speed,
memory, radio delay and loss) as parameters, calibrated by hardware
measurements; export of the field state to VTK and OpenFOAM (see
[Connecting to other tools](#connecting-to-other-tools)).

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

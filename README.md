# Dyne

**Dyne runs a physical model split across many microcontrollers.** Each small
device computes the physics of its own patch of space, exchanges only the values
at the borders with its neighbours, and the patches together form one simulation
of the whole space — without a central server doing the computation.

Dyne is the first toolchain of the
[Physical-Edge Native Architecture](https://github.com/physical-edge-native)
project, which studies how a physical space can be computed by many small,
heterogeneous devices that cooperate only through their neighbours.

> **Status: research prototype (v0.x).** Interfaces change without notice.
> Development records in `docs/` are written in Japanese.

## What it does

- **Same answer everywhere.** A numerical model is compiled once to
  WebAssembly. The same binary runs on a PC, in the simulator and on an ESP32,
  and a solution split over *N* devices matches the single-device solution
  bit for bit.
- **Deterministic network simulator.** Packet loss, delay, jitter, pauses and
  network partitions are injected with a fixed random seed, so every run is
  reproducible across operating systems.
- **Devices can join and leave while running.** A device can be added,
  removed, or handed its neighbour's state during a run, and the solution stays
  identical to a reference computation.
- **Classical and learned solvers side by side.** Finite-difference solvers and
  a small physics-informed neural network (a network trained so that its output
  satisfies the governing equation) plug into the same interface.
- **Coupling to measurements.** On real hardware, each device feeds its own
  sensor readings into its local model (data assimilation by nudging).

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

## Repository layout

| Path | Contents |
|---|---|
| `crates/dyne-wire` | wire format shared by every host (`no_std`) |
| `crates/dyne-core` | deterministic RNG, discrete-event clock, virtual network, WebAssembly loader |
| `crates/dyne-sim`, `crates/dyne-sim2d` | N-device simulators (1D / 2D) |
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

Planned for 2026–2027 (tracked as GitHub issues):
time synchronisation of sensors over Wi-Fi, over-the-air deployment of
WebAssembly modules, importing models trained in PyTorch via ONNX,
IMU (inertial sensor) arrays, harder physics (wave and fluid problems), and
USD export usable from robot-learning environments such as Isaac Lab.

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

TODO: license not yet chosen.

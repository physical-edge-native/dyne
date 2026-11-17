# Installation

Everything except running on ESP32 boards works on Linux, macOS and Windows
without hardware. Commands are run from the root of the repository; on Windows,
use Git Bash for the `bash` and `sh` commands.

## 1. Rust and the WebAssembly target (required)

Install Rust (stable) with [rustup](https://rustup.rs/), then add the target
that solvers are compiled to:

```sh
rustup target add wasm32-unknown-unknown
```

Solvers ("actors") are separate crates outside the main Cargo workspace,
because they are built for `wasm32-unknown-unknown` while the simulators and
tools are built for your PC.

## 2. Check that everything works

```sh
git clone https://github.com/physical-edge-native/dyne.git
cd dyne
bash scripts/ci-check.sh
```

This builds the simulators, the tools and the solvers, and runs the regression
suite that CI runs on every change (it takes several minutes the first time).
Every check prints `PASS`; the script exits with status 0 when all pass. The
suite also uses:

- **Python 3.8 or newer** (standard library only) for the replay and cluster
  rehearsal checks. The script looks for `.venv`, then `python3`, then
  `python`; set `PYTHON=...` to choose another one.
- **A C compiler** (`cc`, `gcc` or `clang`) for the host tests of the firmware.
  Without one, those two checks print `SKIP`.

## 3. A first run of your own

1D heat equation, split over 16 simulated devices, compared with the unsplit
solution:

```sh
(cd crates/heat1d-actor && cargo build --release --target wasm32-unknown-unknown)
cargo run --release -p dyne-sim -- \
  --wasm crates/heat1d-actor/target/wasm32-unknown-unknown/release/heat1d_actor.wasm \
  --nodes 16 --points 65 --steps 2000
# last line: RESULT: PASS (bit-exact)
```

2D heat equation on 2 x 2 simulated devices with 10% message loss and 5 ms
delay:

```sh
(cd crates/heat2d-actor && cargo build --release --target wasm32-unknown-unknown)
cargo run --release -p dyne-sim2d -- \
  --wasm crates/heat2d-actor/target/wasm32-unknown-unknown/release/heat2d_actor.wasm \
  --px 2 --py 2 --steps 500 --loss 0.1 --lat-ms 5
# prints: PASS (bit-exact): 1681 points
```

Fault options of both simulators include `--loss P` (probability of losing a
message), `--lat-ms` and `--lat-jitter-ms` (delay), `--pause N@START:END`
(device N stops, times in virtual milliseconds), `--partition` (link cut for a
while) and `--seed` (random seed; the default is fixed, so runs repeat
exactly). The 1D simulator also has `--handover N@LEAVE:REJOIN` (device N goes
silent and a new instance takes over its state) and `--add N@T` (device N joins
later).

## 4. Python for the PC tools (optional)

The PC-side tools in `scripts/` (configuration, over-the-air updates,
observation, analysis) are Python. Install their packages into a virtual
environment in the repository, not globally:

```sh
python -m venv .venv
# Windows:
.venv\Scripts\python.exe -m pip install -r scripts\requirements-pc.txt
# Linux / macOS:
.venv/bin/python -m pip install -r scripts/requirements-pc.txt
```

`scripts/requirements-pc.txt` lists everything, including packages that only
some tools need (USD export, MuJoCo, reinforcement learning, ONNX). For the
hardware tools alone, `paho-mqtt` and `pyserial` are enough; several tools fall
back to a small MQTT client written with the standard library when `paho-mqtt`
is missing.

## 5. ESP-IDF, only for hardware

Building and flashing the firmware for ESP32 boards (`node-esp32/`) needs
[ESP-IDF](https://docs.espressif.com/projects/esp-idf/) 5.5. It is not needed
for anything above. See the [hardware guide](hardware.md).

# Verification

Dyne's pass criterion is strict: a solution split over devices must be **identical, bit for bit**, to the same
model solved by one solver. Where a result cannot be identical (for example, Dyne against OpenFOAM, which
discretises differently, or against an analytic solution), the tolerance and its reason are stated first.

## The checks, and where they run

| Check | What must hold | Runs |
|---|---|---|
| Split vs single solver | every split equals the single-solver solution bit for bit | simulator, CI |
| Faults | the same, under message loss, delay, jitter, pauses, partitions, devices joining, leaving and being replaced | simulator, CI |
| Across operating systems | the simulator is deterministic (fixed seed, discrete-event clock): same output on Linux, macOS and Windows | CI |
| WebAssembly vs native solvers | the native PC wave solver equals the WebAssembly wave solver; every border message equal byte for byte | CI |
| Accuracy | observed convergence order against analytic standing waves lies in [1.9, 2.1] | CI |
| Hardware vs simulator | every device's CRC-32 of its region equals the simulator's for that region | hardware |
| Replay | sensor inputs recorded on the devices are replayed in the simulator device by device, checkpoint by checkpoint; a single flipped bit is detected | CI (synthetic records), hardware |

## Fault injection in the simulator

The simulator runs the solver modules split over simulated devices on a discrete-event clock. Faults come from a
fixed random seed, so every run is reproducible.

```sh
# 2x2 devices, 10% loss, 5 ms delay, device 3 paused, devices 0 and 1 cut off from each other for a while
cargo run --release -p dyne-sim2d -- --wasm crates/heat2d-actor/target/wasm32-unknown-unknown/release/heat2d_actor.wasm \
  --px 2 --py 2 --steps 500 --loss 0.1 --lat-ms 5 --pause 3@2000:6000 --partition 0-1@10000:15000
# 1D, 8 devices: device 3 goes silent at 30 s and is replaced at 60 s by a new instance that takes over its state
cargo run --release -p dyne-sim -- --wasm crates/heat1d-actor/target/wasm32-unknown-unknown/release/heat1d_actor.wasm \
  --nodes 8 --points 65 --steps 2000 --handover 3@30000:60000
# expected in both: "PASS (bit-exact)"
```

## Native solvers as a reference

`crates/dyne-native` runs the same split computation on the threads of one PC with native Rust solvers (one thread
per region, the same message format). It serves as a reference for runs on devices and as a fast backend for
simulating many devices (80–120 times faster per step than interpreting the WebAssembly solver).

```sh
cargo run --release -p dyne-native --bin dyne-native-wave2d -- check --grid 41x41 --steps 400 --splits 1x1,2x2,4x4,8x8,3x5
cargo run --release -p dyne-native --bin dyne-native-wave3d -- check --grid 25x23x21 --steps 100 --splits 1x1x1,2x2x2,3x2x1,4x4x2,1x3x5
cargo run --release -p dyne-native --bin dyne-native-wave2d -- convergence
```

## On hardware

Devices report a CRC-32 of their region's state; a run passes only if every device matches the simulator's
checksum for its region. The expected table comes from `scripts/wave2d_expected.py`.

```sh
python scripts/wave2d_cluster.py run --broker <PC address> --layout 4x4 --nodes <16 device ids> --steps 400
# with 30 % of received datagrams dropped on purpose
python scripts/wave2d_cluster.py run --broker <PC address> --layout 4x4 --nodes <16 device ids> --steps 400 --test-drop 0.3
```

Results of the 2026-11-21 run: 2 × 2 (4/4), 4 × 2 (8/8) and 4 × 4 (16/16) devices bit-exact; 4 × 4 with 30 % loss
also 16/16 bit-exact, in 96.5 s instead of 45.4 s. See the [wave tutorial](tutorials/wave-equation.md).

## Continuous integration

`bash scripts/ci-check.sh` builds every solver and runs the checks above that need no hardware. GitHub Actions runs
it on Linux, macOS and Windows for every pull request; a change is merged only when all three pass.

## What the bit-exact comparison covers

The simulator runs the same WebAssembly solver as the devices, but not their firmware or radio stack: the comparison
between hardware and simulator covers the computation, not the rest of the device.

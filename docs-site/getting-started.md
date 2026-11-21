# Getting started (no hardware needed)

You need Rust (stable) with the `wasm32-unknown-unknown` target.

```sh
git clone https://github.com/physical-edge-native/dyne.git
cd dyne
rustup target add wasm32-unknown-unknown
# build the 1D heat-equation solver as a WebAssembly module
cd crates/heat1d-actor && cargo build --release --target wasm32-unknown-unknown && cd ../..
# run it split over 16 simulated devices and compare with the single-device solution
cd crates/dyne-sim && cargo run --release -- \
  --wasm ../heat1d-actor/target/wasm32-unknown-unknown/release/heat1d_actor.wasm \
  --nodes 16 --points 65 --steps 2000
```

Expected: `RESULT: PASS (bit-exact)`. The simulator is deterministic: the same input gives the same output on
Linux, macOS and Windows. The full set of checks that CI runs is `scripts/ci-check.sh`.

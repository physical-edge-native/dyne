#!/usr/bin/env bash
# Portable regression check used by CI (Linux / macOS / Windows Git Bash).
# Runs without any hardware: everything goes through the deterministic simulators.
#
#   1. unit tests of the native workspace
#   2. build the WebAssembly actors (heat1d, heat2d)
#   3. 1D: a 16-node split solution must match the single-node solution bit for bit
#   4. 2D: several partitions with packet loss / latency / pauses must match bit for bit
#   5. determinism: two runs with the same arguments produce identical output
#
# Usage (from the repository root):  bash scripts/ci-check.sh
set -euo pipefail

EXE=""
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) EXE=".exe" ;; esac

echo "== [1] unit tests =="
cargo test --release --workspace

echo "== [2] build WebAssembly actors =="
(cd crates/heat1d-actor && cargo build --release --target wasm32-unknown-unknown)
(cd crates/heat2d-actor && cargo build --release --target wasm32-unknown-unknown)
cargo build --release -p dyne-sim -p dyne-sim2d

SIM1="target/release/dyne-sim${EXE}"
SIM2="target/release/dyne-sim2d${EXE}"
W1="crates/heat1d-actor/target/wasm32-unknown-unknown/release/heat1d_actor.wasm"
W2="crates/heat2d-actor/target/wasm32-unknown-unknown/release/heat2d_actor.wasm"

fail=0
expect_pass() { # name, command...
  local name="$1"; shift
  local line
  line="$("$@" | grep -E "PASS|FAIL" | tail -n 1 || true)"
  printf '%-16s %s\n' "$name" "$line"
  case "$line" in *"PASS (bit-exact)"*) ;; *) fail=1 ;; esac
}

echo "== [3] 1D split vs single node (bit-exact) =="
expect_pass "1d-16nodes" "$SIM1" --wasm "$W1" --nodes 16 --points 65 --steps 2000

echo "== [4] 2D partitions under loss/latency/pause (bit-exact) =="
expect_pass "basic2x2"   "$SIM2" --wasm "$W2" --quiet --px 2 --py 2 --steps 500
expect_pass "loss10_lat" "$SIM2" --wasm "$W2" --quiet --px 2 --py 2 --steps 500 --loss 0.1 --lat-ms 5 --lat-jitter-ms 3
expect_pass "3x2_uneven" "$SIM2" --wasm "$W2" --quiet --px 3 --py 2 --nx 40 --ny 33 --steps 400 --loss 0.15 --lat-ms 5 --lat-jitter-ms 10
expect_pass "pause_part" "$SIM2" --wasm "$W2" --quiet --px 2 --py 2 --steps 500 --loss 0.1 --pause 3@2000:6000 --partition 0-1@10000:15000 --partition 1-3@12000:20000

echo "== [5] determinism =="
a="$("$SIM2" --wasm "$W2" --px 2 --py 2 --steps 500 --loss 0.1 --pause 3@2000:6000 | cksum)"
b="$("$SIM2" --wasm "$W2" --px 2 --py 2 --steps 500 --loss 0.1 --pause 3@2000:6000 | cksum)"
if [ "$a" = "$b" ]; then echo "identical output: OK"; else echo "outputs differ: FAIL"; fail=1; fi

if [ "$fail" -ne 0 ]; then echo "CI CHECK FAILED"; exit 1; fi
echo "ALL CHECKS PASSED"

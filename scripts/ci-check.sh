#!/usr/bin/env bash
# Portable regression check used by CI (Linux / macOS / Windows Git Bash).
# Runs without any hardware: everything goes through the deterministic simulators.
#
#   1. unit tests of the native workspace
#   2. build the WebAssembly actors (heat1d, heat1d-pinn, heat2d)
#   3. 1D: a 16-node split solution must match the single-node solution bit for bit
#   4. 2D: several partitions with packet loss / latency / pauses must match bit for bit;
#      a ring of waiting nodes (2x2, heavy loss) finishes within a time limit (issue #26)
#   5. determinism: two runs with the same arguments produce identical output
#   6. firmware host tests, built with the host C compiler: the node id selection of node-esp32, and the
#      step sync replies (a same-step duplicate is answered, so a ring of nodes cannot deadlock; issue #29)
#   7. hardware-input replay: synthetic records (range frames, IMU samples) replay bit for bit;
#      a one-bit change in an input is detected (needs Python 3.8+, standard library only)
#   8. 1D baseline: all outputs of the 12 regression scenarios match the stored hashes of the
#      reference build byte for byte (scripts/check-baseline12.sh, scripts/baseline12.sha256)
#   9. wave2d on a cluster, rehearsed without hardware: the runbook's PC commands (OTA of the actor, 2D
#      configuration, start, collection of per-node hashes) against fake nodes and an in-process MQTT broker;
#      every node's hash must equal dyne-sim2d's, also with 30% datagram loss; a silent node must produce
#      stall reports (scripts/wave2d_rehearsal.py; Python 3.8+, standard library only; issue #4)
#
# Usage (from the repository root):  bash scripts/ci-check.sh
set -euo pipefail

EXE=""
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) EXE=".exe" ;; esac

echo "== [1] unit tests =="
cargo test --release --workspace

echo "== [2] build WebAssembly actors =="
(cd crates/heat1d-actor && cargo build --release --target wasm32-unknown-unknown)
(cd crates/heat1d-pinn-actor && cargo build --release --target wasm32-unknown-unknown)
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

# Issue #26: in a 2x2 partition the four nodes form a ring. Each node could wait at the same step for exactly
# one neighbour while that neighbour ignored its resends, and the run never finished. Both cases must end
# within a time limit: the lossy ring passes, and total loss stops with exit code 2 and a STALL message.
echo "== [4b] 2D ring of waiting nodes finishes within a time limit (issue #26) =="
RL="target/ci-ring"
rm -rf "$RL"; mkdir -p "$RL"
expect_within() { # name, seconds, expected exit code, regex the output must contain, command...
  local name="$1" limit="$2" want="$3" pat="$4"; shift 4
  "$@" > "$RL/$name.log" 2>&1 &
  local pid=$! waited=0 got=0
  while kill -0 "$pid" 2>/dev/null && [ "$waited" -lt "$limit" ]; do sleep 1; waited=$((waited + 1)); done
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    printf '%-16s FAIL (still running after %s s)\n' "$name" "$limit"; fail=1; return
  fi
  wait "$pid" || got=$?
  if [ "$got" -eq "$want" ] && grep -qE "$pat" "$RL/$name.log"; then printf '%-16s OK (exit %s)\n' "$name" "$got"
  else printf '%-16s FAIL (exit %s, expected %s and /%s/; see %s)\n' "$name" "$got" "$want" "$pat" "$RL/$name.log"; fail=1; fi
}
expect_within "ring_loss30" 60 0 "^PASS \(bit-exact\)" "$SIM2" --wasm "$W2" --quiet --px 2 --py 2 --steps 200 --loss 0.3 --lat-jitter-ms 20
expect_within "stall_loss1" 60 2 "^STALL" "$SIM2" --wasm "$W2" --quiet --px 2 --py 2 --steps 50 --loss 1.0

echo "== [5] determinism =="
a="$("$SIM2" --wasm "$W2" --px 2 --py 2 --steps 500 --loss 0.1 --pause 3@2000:6000 | cksum)"
b="$("$SIM2" --wasm "$W2" --px 2 --py 2 --steps 500 --loss 0.1 --pause 3@2000:6000 | cksum)"
if [ "$a" = "$b" ]; then echo "identical output: OK"; else echo "outputs differ: FAIL"; fail=1; fi

# Firmware logic that does not depend on ESP-IDF, compiled with the host C compiler (issues #19, #29).
# Set CC to choose the compiler (e.g. CC="zig cc"); skipped if no C compiler is found.
#   node_id: node id selection (issue #19)
#   sync:    how a node answers messages while it waits for a step, including the reply to a same-step duplicate
#            that keeps a ring of nodes from deadlocking (issue #29; same rule as dyne-sim2d, issue #26)
echo "== [6] firmware host tests: node id selection, step sync replies =="
cc_bin="${CC:-}"
if [ -z "$cc_bin" ]; then
  for c in cc gcc clang; do
    if command -v "$c" >/dev/null 2>&1; then cc_bin="$c"; break; fi
  done
fi
host_test() { # name, test source, core source
  local name="$1" src="$2" core="$3" line
  # shellcheck disable=SC2086 # CC may hold a command with arguments
  $cc_bin -std=c11 -Wall -Wextra -O2 -Inode-esp32/main "$src" "$core" -o "$tdir/${name}_test${EXE}"
  line="$("$tdir/${name}_test${EXE}" | tail -n 1 || true)"
  printf '%-16s %s\n' "$name" "$line"
  [ "$line" = "PASS" ] || fail=1
}
if [ -z "$cc_bin" ]; then
  echo "node_id          SKIP (no C compiler)"
  echo "sync             SKIP (no C compiler)"
else
  tdir="$(mktemp -d)"
  host_test node_id node-esp32/host_test/node_id_test.c node-esp32/main/dyne_node_id_core.c
  host_test sync node-esp32/host_test/sync_test.c node-esp32/main/dyne_sync_core.c
  rm -rf "$tdir"
fi

echo "== [7] hardware-input replay on synthetic records (issue #2) =="
# A recorded run (range frames + node hashes, same format the firmware publishes over MQTT) must replay
# bit for bit even with a different seed and network loss; flipping one bit of one input must fail.
# Same for the IMU fusion actor with a synthetic IMU record.
PY="${PYTHON:-}"
if [ -z "$PY" ]; then
  for c in .venv/Scripts/python.exe .venv/bin/python python3 python; do
    if "$c" -c "import sys; sys.exit(sys.version_info < (3, 8))" > /dev/null 2>&1; then PY="$c"; break; fi
  done
fi
if [ -z "$PY" ]; then echo "no Python 3.8+ found (set PYTHON=...): FAIL"; fail=1; else
  (cd crates/sdf2d-grid-actor && cargo build --release --target wasm32-unknown-unknown)
  (cd crates/imu-fusion-actor && cargo build --release --target wasm32-unknown-unknown)
  WS="crates/sdf2d-grid-actor/target/wasm32-unknown-unknown/release/sdf2d_grid_actor.wasm"
  WI="crates/imu-fusion-actor/target/wasm32-unknown-unknown/release/imu_fusion_actor.wasm"
  RT="target/ci-replay"
  rm -rf "$RT"; mkdir -p "$RT"
  expect_code() { # name, expected exit code, regex the output must contain, command...
    local name="$1" want="$2" pat="$3"; shift 3
    local got=0
    "$@" > "$RT/$name.log" 2>&1 || got=$?
    if [ "$got" -eq "$want" ] && grep -qE "$pat" "$RT/$name.log"; then printf '%-16s OK (exit %s)\n' "$name" "$got"
    else printf '%-16s FAIL (exit %s, expected %s and /%s/; see %s)\n' "$name" "$got" "$want" "$pat" "$RT/$name.log"; fail=1; fi
  }
  SHAPE=(--scene crates/dyne-sim2d/scenes/empty.txt --px 2 --py 2 --steps 120 --quiet)
  expect_code "range-record" 0 "^PASS \(bit-exact\)" "$SIM2" --wasm "$WS" "${SHAPE[@]}" --loss 0.1 --replay-record "$RT/range.jsonl" --replay-hash-every 10
  expect_code "range-replay" 0 "^ALL PASS" "$PY" scripts/hw_replay.py check "$RT/range.jsonl" --wasm "$WS" --sim "$SIM2" --out "$RT/r1" -- "${SHAPE[@]}" --seed 7 --loss 0.2 --lat-ms 5
  "$PY" scripts/hw_replay.py corrupt "$RT/range.jsonl" "$RT/range_bad.jsonl" --node 1 --kind range --step 30 > /dev/null
  expect_code "range-1bit" 1 "^FAIL[[:space:]]*$" "$PY" scripts/hw_replay.py check "$RT/range_bad.jsonl" --wasm "$WS" --sim "$SIM2" --out "$RT/r2" -- "${SHAPE[@]}"
  IMU=(--px 2 --py 2 --nx 6 --ny 6 --dt 0.02 --steps 120 --quiet)
  "$PY" scripts/hw_replay.py synth-imu "$RT/imu_synth.jsonl" --nodes 4 --steps 120 --dt 0.02 > /dev/null
  "$PY" scripts/hw_replay.py convert "$RT/imu_synth.jsonl" --out "$RT/imu_synth.txt" --steps 120 > /dev/null
  expect_code "imu-record" 0 "^REPLAY: no node hashes" "$SIM2" --wasm "$WI" "${IMU[@]}" --replay "$RT/imu_synth.txt" --replay-record "$RT/imu.jsonl" --replay-hash-every 10
  expect_code "imu-replay" 0 "^ALL PASS" "$PY" scripts/hw_replay.py check "$RT/imu.jsonl" --wasm "$WI" --sim "$SIM2" --out "$RT/i1" -- "${IMU[@]}" --seed 7 --loss 0.2
  "$PY" scripts/hw_replay.py corrupt "$RT/imu.jsonl" "$RT/imu_bad.jsonl" --node 2 --kind imu --step 30 > /dev/null
  expect_code "imu-1bit" 1 "^FAIL[[:space:]]*$" "$PY" scripts/hw_replay.py check "$RT/imu_bad.jsonl" --wasm "$WI" --sim "$SIM2" --out "$RT/i2" -- "${IMU[@]}"
fi

echo "== [8] 1D baseline: 12 scenarios vs stored hashes of the reference build (issue #25) =="
bash scripts/check-baseline12.sh || fail=1

echo "== [9] wave2d cluster flow on fake nodes over MQTT (issue #4) =="
if [ -z "$PY" ]; then echo "no Python 3.8+ found (set PYTHON=...): FAIL"; fail=1; else
  (cd crates/wave2d-actor && cargo build --release --target wasm32-unknown-unknown)
  (cd crates/wave2d-actor && RUSTFLAGS="-C link-arg=-zstack-size=8192" \
    cargo build --release --target wasm32-unknown-unknown --features small --target-dir target-small)
  "$PY" scripts/wave2d_rehearsal.py --sim "$SIM2" --out target/ci-wave2d || fail=1
fi

if [ "$fail" -ne 0 ]; then echo "CI CHECK FAILED"; exit 1; fi
echo "ALL CHECKS PASSED"

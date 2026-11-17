#!/usr/bin/env bash
# Portable regression check used by CI (Linux / macOS / Windows Git Bash).
# Runs without any hardware: everything goes through the deterministic simulators.
#
#   1. unit tests of the native workspace; wave2d-actor (WebAssembly) is built first, because the
#      dyne-native tests compare the native 2D wave solver with it bit for bit and byte for byte (issue #40)
#      The dyne-native tests also check the convergence order of the native 2D and 3D wave solvers against
#      analytic standing waves (every observed order in [1.9, 2.1]; crates/dyne-native/tests/convergence.rs;
#      run only here, not again as a separate step) (issue #43)
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
#  10. native 2D wave solver on a PC (crates/dyne-native): a 41x41 grid split 1x1, 2x2, 4x4, 8x8 and 3x5
#      (uneven), one thread per patch: every split must print "PASS (bit-exact)", and the field hash must
#      be the one of the wave scenario of wave2d-actor (18e44331) (issue #38)
#  11. wider borders on the native solvers (k layers exchanged once every k steps, also with edge and
#      corner neighbours): 2D 41x41, even and uneven splits, k = 1, 2, 3, 4, with fixed, zero-gradient
#      and absorbing outer faces: every run must print "PASS (bit-exact)"; a width larger than a patch
#      must be rejected with an error naming the halo width (issue #38)
#  12. native 3D wave solver on a PC: a 25x23x21 grid (100 steps) split 1x1x1, 2x2x2, 3x2x1, 4x4x2 and
#      1x3x5 (uneven), one thread per patch: every split must print "PASS (bit-exact)", and the field hash
#      must be the pinned one (2ca26bb5), the same on every OS; then wider borders as in step 11, k = 1..4,
#      splits 2x2x2, 3x2x1, 4x4x2, 1x3x5, three outer face types: every run "PASS (bit-exact)" (issue #41)
#  13. field export to VTK ImageData (.vti, crates/dyne-export): the native 2D and 3D wave solvers and
#      dyne-sim2d (heat, 2x2 split) write their final field with --vti; dyne-vti reads each file back, and
#      the CRC-32 of the array must equal the hash the run printed (for the wave solvers also the pinned
#      hashes of steps 10 and 12): the field was written and read back bit for bit. The unit tests of
#      dyne-export also cover NaN (with payloads), infinities, -0.0 and subnormal values (issue #57)
#  14. OpenFOAM cases (crates/dyne-export; OpenFOAM itself is not run): the .vti fields of step 13
#      (dyne-sim2d heat 2D, native wave 3D) are written as scalarTransportFoam cases with dyne-foam; reading
#      each case back (every file checked against the others) must give the same cell counts and the same
#      CRC-32 of T as the run. A synthetic OpenFOAM velocity field (crates/dyne-export/tests/fixtures/
#      openfoam-velocity, hand-written in OpenFOAM's layout) is imported onto Dyne's grid: the CRC-32 of each
#      component must be the pinned one, also after writing it to .vti and reading it back (issue #57)
#  15. tutorial solver (examples/decay1d-actor, docs/en/tutorial.md): built as written in the tutorial, it must
#      give "RESULT: PASS (bit-exact)" split over 4 devices, and over 8 devices with 10% loss, delay and one
#      device replaced while running (issue #7)
#  16. API documentation of the core crates (dyne-wire, dyne-core, dyne-native, dyne-export): cargo doc with
#      warnings as errors, so broken links and undocumented public items fail (issue #7)
#
# Usage (from the repository root):  bash scripts/ci-check.sh
set -euo pipefail

EXE=""
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) EXE=".exe" ;; esac

echo "== [1] unit tests =="
# The WebAssembly wave solver is built first: the dyne-native tests compare the native 2D wave solver
# with it bit for bit (crates/dyne-native/tests/wasm_actor.rs; a missing file fails when CI=true).
(cd crates/wave2d-actor && cargo build --release --target wasm32-unknown-unknown)
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
  (cd crates/wave2d-actor && RUSTFLAGS="-C link-arg=-zstack-size=8192" \
    cargo build --release --target wasm32-unknown-unknown --features small --target-dir target-small)
  "$PY" scripts/wave2d_rehearsal.py --sim "$SIM2" --out target/ci-wave2d || fail=1
fi

echo "== [10] native 2D wave solver: split = single patch (bit-exact) (issue #38) =="
cargo build --release -p dyne-native
NW2="target/release/dyne-native-wave2d${EXE}"
out=$("$NW2" check --grid 41x41 --steps 400 --splits 1x1,2x2,4x4,8x8,3x5) || true
echo "$out"
passes=$(printf '%s\n' "$out" | grep -c "PASS (bit-exact)" || true)
if [ "$passes" -eq 5 ] && printf '%s\n' "$out" | grep -q "single patch      crc32 18e44331"; then
  echo "wave2d split     OK"
else
  echo "wave2d split     FAIL"; fail=1
fi

echo "== [11] native solvers, wider borders: k layers every k steps = single patch (bit-exact) (issue #38) =="
for outer in fixed zero-gradient absorbing; do
  out=$("$NW2" check --grid 41x41 --steps 101 --splits 2x2,4x4,8x8,3x5 --halo 1,2,3,4 --outer "$outer") || true
  passes=$(printf '%s\n' "$out" | grep -c "PASS (bit-exact)" || true)
  printf '%s\n' "$out" | sed -n 2p
  if [ "$passes" -eq 16 ]; then
    echo "wave2d halo $outer: 16 of 16 bit-exact  OK"
  else
    echo "$out"; echo "wave2d halo $outer  FAIL"; fail=1
  fi
done
# 8x8 patches of a 41x41 grid have 5 or 6 points per axis: a width of 6 must be refused.
if out=$("$NW2" check --grid 41x41 --steps 10 --splits 8x8 --halo 6 2>&1); then
  echo "$out"; echo "halo too wide    FAIL (accepted)"; fail=1
elif printf '%s\n' "$out" | grep -q "halo width 6 is larger than the patch"; then
  echo "halo too wide    OK (rejected)"
else
  echo "$out"; echo "halo too wide    FAIL (wrong error)"; fail=1
fi

echo "== [12] native 3D wave solver: split and wider borders = single patch (bit-exact) (issue #41) =="
NW3="target/release/dyne-native-wave3d${EXE}"
out=$("$NW3" check --grid 25x23x21 --steps 100 --splits 1x1x1,2x2x2,3x2x1,4x4x2,1x3x5) || true
echo "$out"
passes=$(printf '%s\n' "$out" | grep -c "PASS (bit-exact)" || true)
if [ "$passes" -eq 5 ] && printf '%s\n' "$out" | grep -q "single patch         crc32 2ca26bb5"; then
  echo "wave3d split     OK"
else
  echo "wave3d split     FAIL"; fail=1
fi
for outer in fixed zero-gradient absorbing; do
  out=$("$NW3" check --grid 25x23x21 --steps 41 --splits 2x2x2,3x2x1,4x4x2,1x3x5 --halo 1,2,3,4 --outer "$outer") || true
  passes=$(printf '%s\n' "$out" | grep -c "PASS (bit-exact)" || true)
  printf '%s\n' "$out" | sed -n 2p
  if [ "$passes" -eq 16 ]; then
    echo "wave3d halo $outer: 16 of 16 bit-exact  OK"
  else
    echo "$out"; echo "wave3d halo $outer  FAIL"; fail=1
  fi
done

echo "== [13] field export to VTK ImageData: write, read back, same CRC-32 (issue #57) =="
cargo build --release -p dyne-export
VTI="target/release/dyne-vti${EXE}"
VD="target/ci-vti"
rm -rf "$VD"; mkdir -p "$VD"
vti_check() { # name, file, array, expected crc32 (8 hex digits), expected points (NXxNYxNZ)
  local name="$1" file="$2" arr="$3" crc="$4" pts="$5" info
  if info=$("$VTI" info "$file" 2>&1) \
     && printf '%s\n' "$info" | grep -q "^$file: $pts points" \
     && printf '%s\n' "$info" | grep -q "^array $arr: .* crc32 $crc\$"; then
    printf '%-16s OK (array %s, %s points, crc32 %s)\n' "$name" "$arr" "$pts" "$crc"
  else
    echo "$info"; printf '%-16s FAIL (expected array %s, %s points, crc32 %s)\n' "$name" "$arr" "$pts" "$crc"; fail=1
  fi
}
out=$("$NW2" check --grid 41x41 --steps 400 --splits 2x2 --vti "$VD/wave2d.vti") || true
if printf '%s\n' "$out" | grep -q "single patch      crc32 18e44331"; then
  vti_check "vti wave2d" "$VD/wave2d.vti" u 18e44331 41x41x1
else
  echo "$out"; echo "vti wave2d       FAIL (run)"; fail=1
fi
out=$("$NW3" check --grid 25x23x21 --steps 100 --splits 2x2x2 --vti "$VD/wave3d.vti") || true
if printf '%s\n' "$out" | grep -q "single patch         crc32 2ca26bb5"; then
  vti_check "vti wave3d" "$VD/wave3d.vti" u 2ca26bb5 25x23x21
else
  echo "$out"; echo "vti wave3d       FAIL (run)"; fail=1
fi
out=$("$SIM2" --wasm "$W2" --quiet --px 2 --py 2 --nx 33 --ny 25 --steps 200 --vti "$VD/heat2d.vti") || true
crc=$(printf '%s\n' "$out" | sed -n 's/^crc32: split \([0-9a-f]\{8\}\) .*/\1/p')
if [ -n "$crc" ] && printf '%s\n' "$out" | grep -q "PASS (bit-exact)"; then
  vti_check "vti sim2d heat" "$VD/heat2d.vti" u "$crc" 33x25x1
else
  echo "$out"; echo "vti sim2d heat   FAIL (run)"; fail=1
fi

echo "== [14] OpenFOAM case export and velocity import (OpenFOAM not run) (issue #57) =="
FOAM="target/release/dyne-foam${EXE}"
FD="target/ci-foam"
rm -rf "$FD"; mkdir -p "$FD"
foam_check() { # name, case dir, expected cells (NXxNYxNZ), expected crc32 of T
  local name="$1" dir="$2" cells="$3" crc="$4" info
  if info=$("$FOAM" info "$dir" 2>&1) \
     && printf '%s\n' "$info" | grep -q "^$dir: .* case, $cells cells," \
     && printf '%s\n' "$info" | grep -q "^T: .* crc32 $crc\$"; then
    printf '%-16s OK (%s cells, T crc32 %s)\n' "$name" "$cells" "$crc"
  else
    echo "$info"; printf '%-16s FAIL (expected %s cells, T crc32 %s)\n' "$name" "$cells" "$crc"; fail=1
  fi
}
if [ -n "${crc:-}" ] && "$FOAM" case --vti "$VD/heat2d.vti" --out "$FD/heat2d" --diffusivity 1e-3 --velocity 0.05,0.02,0 \
     --faces fixed:0,fixed:0,zero-gradient,inlet-outlet:0 --end-time 10 --delta-t 0.01 --write-interval 1 > /dev/null; then
  foam_check "foam sim2d heat" "$FD/heat2d" 33x25x1 "$crc"
else
  echo "foam sim2d heat  FAIL (export)"; fail=1
fi
if "$FOAM" case --vti "$VD/wave3d.vti" --out "$FD/wave3d" > /dev/null; then
  foam_check "foam wave3d" "$FD/wave3d" 25x23x21 2ca26bb5
else
  echo "foam wave3d      FAIL (export)"; fail=1
fi
FX="crates/dyne-export/tests/fixtures/openfoam-velocity"
U_CRC="ux 488e48e0 uy 1c058935 uz bca80f3a"
u_ok() { # output of dyne-foam velocity or dyne-vti info
  local out="$1" a c
  set -- $U_CRC
  while [ $# -gt 0 ]; do
    a="$1"; c="$2"; shift 2
    printf '%s\n' "$out" | grep -q "^array $a: 12 values, crc32 $c\$" || return 1
  done
}
if out=$("$FOAM" velocity "$FX" 0 --vti "$FD/velocity.vti" 2>&1) && u_ok "$out" \
   && out2=$("$VTI" info "$FD/velocity.vti" 2>&1) && u_ok "$out2"; then
  echo "foam velocity    OK (3x2x2 fixture, $U_CRC)"
else
  echo "$out"; echo "${out2:-}"; echo "foam velocity    FAIL (expected $U_CRC)"; fail=1
fi

echo "== [15] tutorial solver: decay1d (docs/en/tutorial.md) =="
(cd examples/decay1d-actor && cargo build --release --target wasm32-unknown-unknown)
WT="examples/decay1d-actor/target/wasm32-unknown-unknown/release/decay1d_actor.wasm"
expect_pass "tutorial-4" "$SIM1" --wasm "$WT" --nodes 4 --points 65 --steps 2000
expect_pass "tutorial-faults" "$SIM1" --wasm "$WT" --nodes 8 --points 65 --steps 2000 --loss 0.1 --lat-ms 5 --lat-jitter-ms 3 --handover 3@30000:60000

echo "== [16] API documentation of the core crates (no warnings) =="
if RUSTDOCFLAGS="-D warnings" cargo doc --no-deps -p dyne-wire -p dyne-core -p dyne-native -p dyne-export; then
  echo "cargo doc        OK"
else
  echo "cargo doc        FAIL (warnings in the API documentation)"; fail=1
fi

if [ "$fail" -ne 0 ]; then echo "CI CHECK FAILED"; exit 1; fi
echo "ALL CHECKS PASSED"

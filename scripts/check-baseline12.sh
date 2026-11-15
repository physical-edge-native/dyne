#!/usr/bin/env bash
# The 12 one-dimensional regression scenarios (s01-s12): every output of dyne-sim (stdout, stderr and
# all CSV files, 30 files) must match, byte for byte, the output of the reference build.
#
# The reference is commit 2f7bdcf ("記号の衝突解消", 2026-09-02), the last commit before the shared
# simulator parts were moved into dyne-core. Before the history rewrite of 2026-11-15 the same commit was
# a435bf6 (the code is identical; the rewrite only replaced MAC addresses and a few strings in documents).
# The expected SHA-256 of each output file is stored in scripts/baseline12.sha256, so no old commit has to
# be built. Simulator outputs contain no paths or wall-clock times and are identical on Windows, Linux and
# macOS (the simulator is deterministic and uses virtual time).
#
# The current simulator runs with --unobserved-bc dirichlet:0 (the behaviour before that option existed);
# the line "unobserved-bc = ..." that only the current simulator prints is removed from out.log before
# hashing. Everything else, including every number, has to match exactly.
#
# Usage (from the repository root, after building dyne-sim and the heat1d / heat1d-pinn actors):
#   bash scripts/check-baseline12.sh                 # compare against scripts/baseline12.sha256
#   bash scripts/check-baseline12.sh --exe PATH --legacy
#                                                    # check another dyne-sim build, e.g. the reference
#                                                    # commit itself (it predates --unobserved-bc)
#   bash scripts/check-baseline12.sh --write         # rewrite scripts/baseline12.sha256 from this run
# To regenerate the expected hashes from the reference commit:
#   git worktree add ../dyne-base-2f7bdcf 2f7bdcf
#   cargo build --release -p dyne-sim --manifest-path ../dyne-base-2f7bdcf/Cargo.toml
#   bash scripts/check-baseline12.sh --exe ../dyne-base-2f7bdcf/target/release/dyne-sim --legacy --write
# The actors are always the current ones (built from crates/heat1d-actor and crates/heat1d-pinn-actor).
# Outputs are kept in target/baseline12/<scenario>/ for inspection.
set -euo pipefail

EXE=""
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) EXE=".exe" ;; esac

root="$(pwd)"
sim="$root/target/release/dyne-sim${EXE}"
legacy=0
write=0
while [ $# -gt 0 ]; do
  case "$1" in
    --exe) sim="$2"; shift 2 ;;
    --legacy) legacy=1; shift ;;
    --write) write=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
case "$sim" in /*|[A-Za-z]:*) ;; *) sim="$root/$sim" ;; esac
[ -x "$sim" ] || [ -x "$sim$EXE" ] || { echo "dyne-sim not found: $sim" >&2; exit 2; }
[ -x "$sim" ] || sim="$sim$EXE"

W="$root/crates/heat1d-actor/target/wasm32-unknown-unknown/release/heat1d_actor.wasm"
WP="$root/crates/heat1d-pinn-actor/target/wasm32-unknown-unknown/release/heat1d_pinn_actor.wasm"
for f in "$W" "$WP"; do [ -f "$f" ] || { echo "actor not built: $f" >&2; exit 2; }; done

expected="$root/scripts/baseline12.sha256"
work="$root/target/baseline12"
rm -rf "$work"; mkdir -p "$work"

if command -v sha256sum > /dev/null 2>&1; then hash() { sha256sum "$@"; }
else hash() { shasum -a 256 "$@"; }; fi

# name followed by the arguments; CSV files are written with relative names into the scenario directory.
scenarios=(
  "s01_basic16        --wasm $W --nodes 16 --points 65 --steps 2000"
  "s02_chaos16        --wasm $W --nodes 16 --points 65 --steps 2000 --loss 0.10 --lat-ms 5 --lat-jitter-ms 3"
  "s03_pause          --wasm $W --nodes 8 --points 65 --steps 2000 --pause 3@20000:40000"
  "s04_partition_loss --wasm $W --nodes 8 --points 65 --steps 2000 --partition 4@60000:90000 --loss 0.05 --lat-ms 2 --lat-jitter-ms 2"
  "s05_handover       --wasm $W --nodes 8 --points 65 --steps 2000 --handover 3@30000:60000"
  "s06_add_resid      --wasm $W --nodes 4 --points 65 --steps 2000 --add 3@30000 --residual-csv resid.csv"
  "s07_add_chaos_ho   --wasm $W --nodes 8 --points 65 --steps 2000 --add 5@30000 --handover 2@50000:70000 --loss 0.10 --lat-ms 4 --lat-jitter-ms 5"
  "s08_csv4           --wasm $W --nodes 8 --points 65 --steps 500 --event-csv ev.csv --field-csv field.csv --field-every 50 --error-csv err.csv"
  "s09_add_autogrow   --wasm $W --nodes 8 --points 65 --steps 2000 --add 4@30000 --auto-grow on --event-csv ag_ev.csv --residual-csv ag_resid.csv"
  "s10_pinn_basic     --wasm $WP --nodes 4 --points 65 --steps 300 --ram-cap-kb 4096"
  "s11_pinn_resize    --wasm $WP --nodes 4 --points 65 --steps 300 --ram-cap-kb 4096 --resize 1@10000:24"
  "s12_pinn_add_ag    --wasm $WP --nodes 4 --points 65 --steps 300 --ram-cap-kb 4096 --add 3@8000 --auto-grow on"
)
extra=(--unobserved-bc dirichlet:0)
[ "$legacy" -eq 1 ] && extra=()

# The scenarios are independent and use virtual time, so they run in parallel (the three with the learning
# actor take about half a minute each; the others a few seconds).
for line in "${scenarios[@]}"; do
  # shellcheck disable=SC2206 # the scenario arguments contain no spaces
  words=($line)
  name="${words[0]}"
  mkdir -p "$work/$name"
  # A failing run is reported through its outputs (the hashes will not match), not by stopping here.
  # ${extra[@]+...}: an empty array is "unbound" under set -u in the bash 3.2 of macOS.
  (cd "$work/$name" && "$sim" ${extra[@]+"${extra[@]}"} "${words[@]:1}" > out.raw 2> err.log) &
done
wait || true
for line in "${scenarios[@]}"; do
  name="${line%% *}"
  grep -v '^unobserved-bc = ' "$work/$name/out.raw" > "$work/$name/out.log" || true
  rm "$work/$name/out.raw"
done

# One line per output file: "<sha256>  <scenario>/<file>", sorted.
actual="$work/actual.sha256"
(cd "$work" && for d in s[0-9][0-9]_*; do for f in "$d"/*; do
  printf '%s  %s\n' "$(hash < "$f" | cut -d ' ' -f 1)" "$f"
done; done) | LC_ALL=C sort -k 2 > "$actual"

if [ "$write" -eq 1 ]; then
  cp "$actual" "$expected"
  echo "wrote $expected ($(wc -l < "$actual" | tr -d ' ') files)"
  exit 0
fi

# Remove carriage returns in case the expected file was checked out with CRLF line endings.
tr -d '\r' < "$expected" > "$work/expected.sha256"

# Files that depend on the host C math library, not on the simulation: err.csv compares each node with the
# analytic solution sin(pi x) exp(-alpha pi^2 t), evaluated natively with the platform's sin/exp. The
# simulation itself runs in WebAssembly and is bit-exact on every OS, but these last bits differ between
# Windows (where the reference hashes were made) and glibc/macOS. Compare them only on Windows.
host_libm_files=("s08_csv4/err.csv")
if [ -z "$EXE" ]; then
  for f in "${host_libm_files[@]}"; do
    grep -v "  $f\$" "$work/expected.sha256" > "$work/e.tmp" && mv "$work/e.tmp" "$work/expected.sha256"
    grep -v "  $f\$" "$actual" > "$work/a.tmp" && mv "$work/a.tmp" "$actual"
    echo "note: $f not compared on this OS (depends on the host math library, see script)"
  done
fi
fail=0
for line in "${scenarios[@]}"; do
  name="${line%% *}"
  if diff <(grep " $name/" "$work/expected.sha256") <(grep " $name/" "$actual") > "$work/diff-$name.txt"; then
    printf 'MATCH    %-20s (%s files)\n' "$name" "$(grep -c " $name/" "$actual")"
  else
    printf 'MISMATCH %-20s\n' "$name"
    # Files whose hash differs or that are missing / extra on one side.
    sed -n 's/^[<>] [0-9a-f]*  /  DIFF: /p' "$work/diff-$name.txt" | LC_ALL=C sort -u
    fail=1
  fi
done
if [ "$fail" -eq 0 ]; then echo "ALL 12 SCENARIOS MATCH (bit-exact vs 2f7bdcf, formerly a435bf6)"
else echo "!! MISMATCH AGAINST BASELINE !! (outputs in target/baseline12/)"; exit 1; fi

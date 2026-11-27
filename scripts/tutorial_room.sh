#!/usr/bin/env bash
# Room tutorial (docs/en/tutorial-room.md, GitHub issue #96): a dummy room with simulated devices,
# from dummy sensor readings to Dyne's shape and temperature, then USD, a 3D dashboard, an OpenFOAM case and
# a MuJoCo model.
# No hardware is needed. Run from the repository root:
#
#   bash scripts/tutorial_room.sh                # steps 1-3 (and step 4 if you add --openfoam)
#   bash scripts/tutorial_room.sh --sim-only     # only what needs Rust: steps 1, 2, the OpenFOAM case and the MuJoCo model
#                                                # (no Python; this is what scripts/ci-check.sh runs)
#   bash scripts/tutorial_room.sh --openfoam     # also run the case with OpenFOAM (optional step 4;
#                                                # needs OpenFOAM: blockMesh on PATH, or the openfoam2606 command)
#   bash scripts/tutorial_room.sh --figures DIR  # also draw the pictures of the tutorial into DIR
#
# Outputs go to demo_out/tutorial/ (not tracked by git). Windows PowerShell: scripts/tutorial_room.ps1.
set -euo pipefail

SIM_ONLY=0
OPENFOAM=0
FIGDIR=""
OUT="demo_out/tutorial"
while [ $# -gt 0 ]; do
  case "$1" in
    --sim-only) SIM_ONLY=1 ;;
    --openfoam) OPENFOAM=1 ;;
    --figures) FIGDIR="$2"; shift ;;
    --out) OUT="$2"; shift ;;
    *) echo "unknown option $1 (see the top of $0)"; exit 2 ;;
  esac
  shift
done

EXE=""
case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) EXE=".exe" ;; esac
SCENE="crates/dyne-sim2d/scenes/room6.txt"
READINGS="crates/dyne-sim2d/scenes/room6_temp.csv"
SIM2="target/release/dyne-sim2d${EXE}"
FOAM="target/release/dyne-foam${EXE}"
WS="crates/sdf2d-grid-actor/target/wasm32-unknown-unknown/release/sdf2d_grid_actor.wasm"
WH="crates/heat2d-actor/target/wasm32-unknown-unknown/release/heat2d_actor.wasm"

echo "== [0] build: two WebAssembly actors (shape, heat) and the simulator and exporter =="
(cd crates/sdf2d-grid-actor && cargo build --release --target wasm32-unknown-unknown)
(cd crates/heat2d-actor && cargo build --release --target wasm32-unknown-unknown)
cargo build --release -p dyne-sim2d -p dyne-export
mkdir -p "$OUT"

echo "== [1] the dummy room: $SCENE (walls, furniture, 54 distance sensors on 9 devices)"
echo "       and the dummy thermometer readings: $READINGS"
grep -v '^#' "$READINGS"

echo "== [2a] shape: 32 simulated devices (8 x 4) build the room's signed distance field from simulated distance frames"
"$SIM2" --wasm "$WS" --scene "$SCENE" --steps 40 --px 8 --py 4 --h 0.025 --zones 64 --fov-deg 60 --noise-m 0.002 \
  --field-csv "$OUT/shape.csv" --vti "$OUT/shape.vti" > "$OUT/shape.log"
grep -E "^(shape: surface|crc32|PASS|FAIL)" "$OUT/shape.log"

echo "== [2b] temperature: the same 32 devices run the 2D heat model, nudged to the 12 dummy readings"
"$SIM2" --wasm "$WH" --scene "$SCENE" --h 0.1 --px 8 --py 4 --alpha 1 --dt 0.002 --outer-bc zero_grad \
  --init-value 20 --temp-sensors "$READINGS" --nudge-gamma 0.8 --steps 2000 --vti "$OUT/temperature.vti" > "$OUT/temperature.log"
grep -E "^(nudging|crc32|PASS|FAIL)" "$OUT/temperature.log"
grep -q "^PASS (bit-exact)" "$OUT/shape.log" && grep -q "^PASS (bit-exact)" "$OUT/temperature.log" \
  || { echo "the split solution differs from the single-device solution (see $OUT/*.log)"; exit 1; }

PY="${PYTHON:-}"
if [ "$SIM_ONLY" -eq 0 ] && [ -z "$PY" ]; then
  for c in .venv/Scripts/python.exe .venv/bin/python python3 python; do
    if "$c" -c "import numpy" > /dev/null 2>&1; then PY="$c"; break; fi
  done
  [ -n "$PY" ] || echo "(no Python with numpy found: the Python parts are skipped; see docs/en/installation.md)"
fi
export PYTHONIOENCODING=utf-8

if [ -n "$PY" ]; then
  echo "== [2c] check: leave each thermometer out, rerun, predict it (tolerance 0.5 C); 12 more runs"
  # FAIL is the expected result with this model (no heater, no window): see docs/en/tutorial-room.md
  "$PY" scripts/tutorial_room.py check --scene "$SCENE" --heat-vti "$OUT/temperature.vti" --readings "$READINGS" \
    --sim "$SIM2" --wasm "$WH" --gamma 0.8 --work "$OUT" \
    || echo "(expected with this model: it has no heater and no window; see docs/en/tutorial-room.md)"
fi

echo "== [3a] OpenFOAM case from Dyne's temperature field (OpenFOAM itself is not needed for this)"
FOAM_OPTS=(--end-time 3600 --delta-t 10 --write-interval 600)
rm -rf "$OUT/openfoam"
WALLS=0
FOAM_USAGE="$("$FOAM" 2>&1 || true)"
if [[ "$FOAM_USAGE" == *"--shape"* ]]; then
  # Walls and furniture as solid cells (dyne-foam --shape, GitHub issue #91). The shape grid (2.5 cm) is
  # taken at the points of the heat grid (10 cm); the uniform velocity stays 0 (a uniform flow with walls diverges).
  if [ -n "$PY" ]; then
    "$PY" scripts/tutorial_room.py coarse-shape --shape-vti "$OUT/shape.vti" --heat-vti "$OUT/temperature.vti" --out "$OUT/shape_10cm.vti"
    "$FOAM" case --vti "$OUT/temperature.vti" --shape "$OUT/shape_10cm.vti" --out "$OUT/openfoam" "${FOAM_OPTS[@]}"
    WALLS=1
  fi
fi
if [ "$WALLS" -eq 0 ]; then
  # TODO(#91): once dyne-foam has --shape (issue #91), the walls are added automatically (branch above).
  echo "(walls: this dyne-foam has no --shape yet (issue #91) or Python is missing; the case is the whole rectangle)"
  "$FOAM" case --vti "$OUT/temperature.vti" --out "$OUT/openfoam" "${FOAM_OPTS[@]}"
fi
"$FOAM" info "$OUT/openfoam" | grep -E "^(T:|mesh|faces)"

echo "== [3c] MuJoCo model (MJCF) of the room shape and its free floor for robot spawn points (no Python, no MuJoCo needed)"
"target/release/dyne-mjcf${EXE}" --shape "$OUT/shape.vti" --out "$OUT/room_mujoco.xml"

if [ "$SIM_ONLY" -eq 0 ] && [ -n "$PY" ]; then
  echo "== [3b] USD (physics colliders), MuJoCo model and the 3D dashboard (HTML)"
  if "$PY" -c "import pxr" > /dev/null 2>&1; then
    "$PY" scripts/twin_demo.py --scene "$SCENE" --shape-csv "$OUT/shape.csv" --sim-log "$OUT/shape.log" --out "$OUT/room" \
      --zones 64 --fov-deg 60 --truth-usd --temp-vti "$OUT/temperature.vti" --temp-readings "$READINGS" \
      --title "Dyne room tutorial (simulated devices, dummy readings)"
    "$PY" scripts/usd_check.py "$OUT/room.usda" --scene "$SCENE" | tail -n 1
  else
    echo "(usd-core is not installed in this Python: USD and the dashboard are skipped; pip install -r scripts/requirements-pc.txt in the .venv)"
  fi
fi

if [ "$OPENFOAM" -eq 1 ]; then
  echo "== [4] optional: run the case with OpenFOAM (scalarTransportFoam, 1 hour of still-air diffusion)"
  if [ "$WALLS" -eq 1 ]; then RUN="checkMesh > log.checkMesh 2>&1"; else RUN="blockMesh > log.blockMesh 2>&1"; fi
  RUN="$RUN && scalarTransportFoam > log.scalarTransportFoam 2>&1 && postProcess -func writeCellCentres -time 0 > log.cellCentres 2>&1"
  if command -v blockMesh > /dev/null 2>&1; then
    (cd "$OUT/openfoam" && bash -c "$RUN")
  elif command -v openfoam2606 > /dev/null 2>&1; then
    (cd "$OUT/openfoam" && openfoam2606 -c "$RUN")
  else
    echo "(OpenFOAM not found: put blockMesh on PATH or install openfoam2606; on Windows use scripts/tutorial_room.ps1 -OpenFOAM, which runs it in WSL)"
  fi
  ls "$OUT/openfoam"
fi

if [ -n "$FIGDIR" ] && [ -n "$PY" ]; then
  echo "== pictures -> $FIGDIR"
  "$PY" scripts/tutorial_room.py figures --scene "$SCENE" --readings "$READINGS" --shape-csv "$OUT/shape.csv" \
    --heat-vti "$OUT/temperature.vti" --foam-case "$OUT/openfoam" --out-dir "$FIGDIR"
  if [ -f "$OUT/room.html" ]; then
    "$PY" scripts/tutorial_room.py screenshot --html "$OUT/room.html" --out "$FIGDIR/04-dashboard.png"
  fi
fi

echo "== done: outputs in $OUT/"

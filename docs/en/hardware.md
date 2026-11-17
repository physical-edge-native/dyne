# Hardware guide: running on ESP32 boards

This page summarises how a Dyne solver is put on ESP32 boards and checked
against the simulator. **Everything here needs hardware** (ESP32 boards, a
Wi-Fi access point, a PC on the same network); nothing on this page is needed
for the simulator, the tutorial or the regression suite. The detailed runbooks
are in Japanese and are linked from each section.

Status: two ESP32 boards over Wi-Fi have produced a bit-exact split solution,
and four battery-powered boards with temperature sensors completed four runs of
about 33 minutes. Time synchronisation, over-the-air updates, the 2D wave
equation on a cluster and replay of recorded device input are implemented and
rehearsed against simulated devices, and await verification on a 16-board
setup (see the README's Roadmap).

## What you need

- ESP32 boards. The current firmware targets the ESP32-DevKitC-VE (8 MB flash,
  8 MB PSRAM); other boards have their own `node-esp32/sdkconfig.board.*`.
- A Wi-Fi access point that the boards and the PC join.
- On the PC: [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) 5.5, an
  MQTT broker (for example Mosquitto, port 1883), and the Python environment
  of the [installation](installation.md) guide (`paho-mqtt`, `pyserial`).

The PC only configures, observes and collects results. The computation and the
messages between neighbouring regions stay on the boards (directly between
them, over UDP).

Commands below are run from the repository root; `$PY` stands for the Python of
the repository's virtual environment (`.venv\Scripts\python.exe` on Windows,
`.venv/bin/python` elsewhere), and `<PC>` for the PC's address on the access
point's network.

## 1. Build the firmware

All boards run the same firmware (`node-esp32/`); a board's role comes from the
configuration it receives at run time. Wi-Fi name and password and the broker
address go into `node-esp32/sdkconfig.local`, which is not tracked by git
(never commit Wi-Fi credentials).

```powershell
# Windows, after initialising ESP-IDF 5.5 in this PowerShell window
powershell -ExecutionPolicy Bypass -File node-esp32\build.ps1 -Board devkitc-ve
```

Details: `node-esp32/README.md`, `docs/2026-11-13-firmware-nodes.md`.

## 2. Flash many boards in a row

```powershell
& $PY scripts\first_flash.py
```

Connect one board at a time over USB. The tool writes the same image to each
board without rebuilding, checks the boot log (memory, self-test, the image
that runs), prints the board's node number in large type for labelling, and
appends a line to `first_flash_out\first_flash.csv`. The node number comes
from the configuration, a value stored on the board, or the last byte of the
board's MAC address; two boards can collide, and the tool reports it.

Details: `docs/2026-11-13-firmware-nodes.md` (section 9).

## 3. Synchronise clocks

```powershell
& $PY scripts\dyne_ntp_server.py --port 123                                  # time server on the PC
& $PY scripts\dyne_timesync.py --broker <PC> --nodes all --record 600 --csv tsync.csv   # record sync quality
```

Each board estimates the offset and rate of its clock against the PC's time
server; `dyne_timesync.py` records what the boards report (offset, round-trip
time) and can toggle a pin on each synchronised second for measuring the
residual error with an oscilloscope. The split computation itself does not
need synchronised clocks (steps are aligned by the exchange with neighbours);
time stamps on sensor data do.

Details: `docs/2026-11-11-timesync-ota.md` (section 7).

## 4. Update firmware and solvers over the air

```powershell
& $PY scripts\dyne_ota.py status --broker <PC>
& $PY scripts\dyne_ota.py fw    --broker <PC> --bin <firmware .bin> --nodes all --csv ota_results.csv
& $PY scripts\dyne_ota.py actor --broker <PC> --wasm <solver .wasm> --name <solver name> --nodes all
```

The PC serves the file over HTTP and tells the boards over MQTT to fetch it;
each board's result (success, rolled back, refused, failed, timed out) is
written to the CSV. A firmware image that fails its self-test after a restart
is rolled back. Solvers are replaced without reflashing the firmware.

Solvers sent to boards are the "small" builds, which fit a 64 KB WebAssembly
memory: `powershell -ExecutionPolicy Bypass -File scripts\build-actor-wasm.ps1`
builds the full and small versions of each solver; both compute identically
(checked in the simulator).

Details: `docs/2026-11-11-timesync-ota.md` (section 4).

## 5. Configure, run and compare: the 2D wave equation on a cluster

`scripts/wave2d_cluster.py` does the whole run for the 2D wave equation: it
computes the expected per-board checksums with `dyne-sim2d`, checks that every
board is online, sends each board its configuration (its block of the grid,
its faces, its neighbours' addresses), starts the run, collects each board's
final CRC-32, and compares.

```powershell
& $PY scripts\dyne_ota.py actor --broker <PC> --wasm crates\wave2d-actor\target-small\wasm32-unknown-unknown\release\wave2d_actor.wasm --name wave2d_actor_small --nodes all
& $PY scripts\wave2d_cluster.py status --broker <PC>
& $PY scripts\wave2d_cluster.py run --broker <PC> --layout 2x2 --nodes 11,12,13,14
```

The order of `--nodes` is the block order of `dyne-sim2d`: index = by × px + bx
(x first), so for 2 × 2: lower left, lower right, upper left, upper right. The
result is `PASS (bit-exact)` only if every board's checksum equals the
simulator's for its block. `--test-drop 0.3` makes the boards drop 30% of the
border messages on purpose, to check recovery from loss; the result must still
be bit-exact. The assignment of blocks to boards is written by hand on the
command line.

Rehearse the same commands without hardware, against simulated boards and a
broker inside the PC (this is step 9 of the regression suite):

```sh
python scripts/wave2d_rehearsal.py
# ends with: ALL OK
```

For the 1D heat equation, `scripts/mqtt_config.py` configures a chain of boards
(see `node-esp32/README.md`).

Details: `docs/2026-11-16-wave2d-cluster-runbook.md`.

## 6. Replay recorded sensor input

Sensor readings happen once, so a hardware run with sensors can be compared with
the simulator only if the inputs are recorded. With `"replay":{"log":true}` in
a run's configuration, the boards publish every input they hand to the solver
and their state checksums; the PC records them and replays them in
`dyne-sim2d`:

```powershell
& $PY scripts\hw_replay.py record --broker <PC> --out run.jsonl     # during the run
& $PY scripts\hw_replay.py check run.jsonl --wasm <solver .wasm> -- <dyne-sim2d arguments of the run>
```

`check` reports `PASS`, `FAIL` or `INCOMPLETE` for each board, and for each
checkpoint whether the board's checksum equals the simulator's. The regression
suite runs the same check on synthetic recordings (step 7); it has not yet been
run with recordings from hardware.

Details: `docs/2026-11-15-hw-replay.md`.

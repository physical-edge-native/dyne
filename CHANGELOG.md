# Changelog

All notable changes to this project are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project uses [Semantic Versioning](https://semver.org/) (0.x: interfaces may change).

## [Unreleased]

Entries for the next release are collected in [`changelog.d/`](changelog.d/README.md).

## [0.2.0] - 2026-11-27

Room twin on real hardware (2026-11-21), English documentation site, native PC solvers moved in from barye,
export to OpenFOAM, VTK and USD, and checks of learned solvers and cooperative-run logs without hardware.
The tag `v0.2.0` points to 2919654, which also contains #199 (listed below under Added).

### Added
- `scripts/first_flash.py`: first USB flash of many ESP32 boards in a row, with boot-log checks and a MAC/node-id CSV (#1).
- `dyne-sim2d --replay` / `--replay-record` and `scripts/hw_replay.py`: replay the sensor input that hardware nodes fed to their actors and check the nodes' state hashes bit for bit; the firmware publishes these records (QoS 1) when the run configuration sets `"replay":{"log":true}` (#2).
- `scripts/wave2d_cluster.py`: run the 2D wave equation on a node cluster (configure, start, collect per-node hashes, compare bit for bit with dyne-sim2d), with fake nodes and a standard-library MQTT broker so CI rehearses the whole flow without hardware (#4).
- `docs/en/hardware.md`: English hardware guide (firmware build, flashing many boards, clock synchronisation, over-the-air updates, a 2D cluster run checked against the simulator, replay of recorded input) (#7).
- `docs/en/`: English installation guide and a tutorial that writes a new solver (`examples/decay1d-actor`) and checks it bit for bit in the simulator; CI runs the tutorial (#7).
- `node-esp32`: the node id can be stored in NVS with the MQTT control command `node_id` (targeted by MAC address, applied at the next boot), so one firmware image fits every board; the online status reports the id source (`config`, `nvs` or `mac`) (#19).
- Design record for moving the native PC patch solvers from the barye repository into Dyne (`docs/design/2026-11-16-native-backend-from-barye.md`) (#34).
- `crates/dyne-native`: native patch-solver crate (block decomposition of a grid of any dimension, one thread per patch exchanging border messages in wire format v2, field CRC-32), moved from the barye repository (#36).
- `dyne-native-wave2d`: native 2D wave patch solver on PC threads, bit-exact across splits and with `crates/wave2d-actor`, with wider borders (k layers exchanged once every k steps) and CI checks; moved from the barye repository (#38).
- `crates/dyne-native`: CI compares the native 2D wave solver with `crates/wave2d-actor` bit for bit (fields) and byte for byte (border messages); `wave2d-actor` is now built before the unit tests, independent of Python (#40).
- `dyne-native-wave3d`: native 3D wave patch solver (seven-point Laplacian, documented order of floating-point operations), bit-exact across splits and border widths, with CI checks; moved from the barye repository (#41).
- `scripts/wave2d_cluster.py run`: report the per-step exchange wait of each node (step time minus compute time, from the `rt` reports) and save it in `result.csv` / `result.json`; the runbook records node-to-PC round-trip and one-way delays (#42).
- `dyne-native-wave2d convergence` and `dyne-native-wave3d convergence`: convergence order of the native wave solvers against analytic standing waves (observed orders must lie in [1.9, 2.1]), checked once in CI by the unit tests; moved from the barye repository (#43).
- `dyne-foam` (crates/dyne-export): write a 2D or 3D field as an OpenFOAM case for `scalarTransportFoam` (one cell per grid point, lossless decimal values, face conditions, given velocity and diffusivity), read the case back bit for bit, and import a velocity field computed by OpenFOAM onto Dyne's grid (part 2 of #57: OpenFOAM export and import) (#57).
- `crates/dyne-export` and `--vti PATH` on `dyne-native-wave2d check`, `dyne-native-wave3d check` and `dyne-sim2d`: write the final 2D or 3D field as VTK ImageData (`.vti`, lossless raw Float64) for ParaView or PyVista; `dyne-vti info` reads it back and CI checks the round trip bit for bit (part 1 of #57: VTK export; OpenFOAM export and import follow) (#57).
- Design record and README note on fluids: flow given from outside for now (imported from OpenFOAM), transported quantities assimilated on the devices; flow on devices under study (#61).
- `dyne-foam case --shape SHAPE` (crates/dyne-export): leave out the cells inside the walls and furniture of the room shape that the devices build (a `.vti` of `dyne-sim2d` or the `shape.csv` of `scripts/twin_live.py`); the mesh is written as `constant/polyMesh` with the patch `walls`, checked without OpenFOAM in CI and with OpenFOAM's `checkMesh` and `scalarTransportFoam` by hand (#91).
- `dyne-foam case --sensors READINGS.csv` (crates/dyne-export): the initial temperature of an OpenFOAM case from a few point sensors (e.g. the DHT20 / BME280 of the devices), spread over the grid by inverse-distance weighting, which stays within the range of the readings; with `--shape`, also in a room with walls (#92).
- `scripts/isaaclab_go2_room.py`: load the exported room USD into Isaac Lab as static collision geometry and walk a Unitree Go2 (trained velocity-tracking policy plus a simple go-to-goal rule) through goal points; `--check-only` checks the USD and picks reachable start and goal points without Isaac (Isaac part not yet run on a GPU) (#93).
- Capture procedure for hardware runs: what to photograph and screen-record (real room next to its twin from the same viewpoint, one node leaving and rejoining), file names, where to store material per site, and a privacy check before anything is published (#94).
- Room tutorial without hardware (`docs/en/tutorial-room.md`, `scripts/tutorial_room.ps1` / `.sh`, CI step 19): simulated devices compute the shape and the temperature of a dummy room, then USD with colliders, a 3D dashboard and an OpenFOAM case are written; `dyne-sim2d --temp-sensors` nudges the 2D heat model (`heat2d-actor`: new `dyne_observe`, `dyne_set`) towards point readings, still bit-exact across splits; the tutorial checks the field by leaving each thermometer out, and documents that the heat model without heat sources fails that check (#96).
- English user documentation site (MkDocs Material) published to GitHub Pages at https://physical-edge-native.github.io/dyne/.
- Docs site: wave-equation tutorial with the 2026-11-21 hardware results.
- Docs site: include the English guides from docs/en/ at build time and add pages on how Dyne works, verification, connections to other tools, stability theory, status and limits, and components.
- Publish the 2026-11-21 IMU hardware record (7 devices on one board) under measured/ with an English report on the docs site.
- Discussion category forms for Ideas, Q&A and Show and tell (#134).
- README and documentation site: links to GitHub Discussions for questions and ideas, and to Issues for bugs (#136).
- `scripts/ci-check.sh` now checks the learned-solver path (ONNX → `.dynm` → `mlp-actor`) on Linux, macOS and Windows, including a ReLU-only network that must match the reference bit for bit with pinned CRC-32 values (#155).
- `dyne-sim2d --growth-csv` and `--snapshot-every` record how the room twin grows step by step; `scripts/twin_growth.py` draws it, and a docs-site page shows it (#157).
- `scripts/imu_record_replay.py` replays the 2026-11-21 IMU hardware record through the IMU fusion solver in the simulator (bit-exact against its reference model), and the documentation now states that fusion has not yet run on the devices (#158).
- Theory: proven error bound for classical and learned regions mixed when training leaves a residual (initial error plus the sum of one-step defects), checked in `scripts/ci-check.sh` on synthetic defects and on the learned solver in the simulator (#159).
- `dyne-sim2d --perf-log` writes a cooperative-run log (per message and per step times and bytes), and `scripts/coop_perf.py` reports latency and jitter distributions, throughput and steps per second from it (#167).
- `CONTRIBUTING.md` has a "Your first contribution" section: where to find good first issues and help-wanted issues, how to run the checks, how to hide private details in hardware records, and where to ask (#176).
- `theory/guarantee/heat_nd.py` and step 23 of `scripts/ci-check.sh`: the error bound for learned regions that leave a residual is extended to the explicit heat scheme in 2D (`r ≤ 1/4`) and 3D (`r ≤ 1/6`), proven on the theory page and checked on synthetic defects (#197).

### Changed
- Live twin exporter runs in-process and caches static parts: per-export time 3.3–3.5 s → about 0.5 s (#3).
- `scripts/build-actor-wasm.ps1` also builds `imu-fusion-actor` and lists every actor output (#5).
- English API documentation (`cargo doc`) for `dyne-wire` and `dyne-core`, no documentation warnings in the core crates (checked in CI, step 16), and `docs/en/reference.md` on the solver interface and message format (#7).
- README: a section on the native PC solvers (`crates/dyne-native`) and their role as reference and fast backend; `docs/design/` gains the superseded positioning record moved from the barye repository (#44).
- README describes the revised picture (assimilated model state on the devices, forecasting by handing the state to a PC with identical bits), the limits stated up front and related work (#52).
- README has a section on connecting to other tools: USD (Isaac Sim / Isaac Lab), VTK image data and OpenFOAM cases (#58).
- `README.md` restructured around the core workflow (write a solver, check it in the simulator with faults, deploy to ESP32 devices, check hardware against the simulator, use or export the result); states when results are the same bits across platforms; related work extended; `docs/design/2026-11-17-state-of-the-field.md` lists comparison targets (#68).
- `heat2d-actor`: a step copies only the block's own points instead of swapping the two whole state arrays, about 30 times faster in `dyne-sim2d` for small blocks; results are unchanged bit for bit (#96).
- README: what Dyne is for comes first (a room digital twin kept up to date by small devices, robot training in it, fields kept in step with sensors), with a picture from the room tutorial and the status of each field; the technical summary follows (#97).
- Show the 2026-11-21 real-room hardware run at the top of the README (photos, twin, Go2 in Isaac Sim) and compare simulator and hardware accuracy honestly.
- Run the room-shape OpenFOAM export on the real room of the 2026-11-21 hardware run with OpenFOAM.com v2606 and show the result in the README and docs.
- Capture procedure: media from site A may now be published after the same pre-publication check as site B.
- README: update the status after the 2026-11-21 hardware run and link the documentation and architecture sites.
- The ESP32 benchmark firmware (`bench-c5`) uses the current small `mlp-actor`, and the values the boards must report are pinned in `bench-c5/expected.txt` and checked in CI (#156).

### Fixed
- Firmware: `range` and `range_fed` MQTT messages no longer share one JSON buffer (#8).
- The 12-scenario 1D regression (`scripts/check-baseline12.sh`, now also run by `scripts/ci-check.sh`) compares against stored hashes instead of rebuilding a commit that is no longer in the public history (#25).
- `dyne-sim2d`: runs with a 2×2 or larger partition and heavy packet loss no longer hang. A node now answers a repeated message for its current step with its own copy, so nodes waiting on each other in a ring recover; a run that still makes no progress stops with a `STALL` message and exit code 2 (#26).
- `node-esp32` (wire v2 path): a 2×2 or larger grid of nodes can no longer deadlock when losses coincide. A node now answers a repeated message for its current step with its own copy, at most once per resend interval per interface (the same rule as `dyne-sim2d`), and a step that waits longer than `mode.stall_ms` (default 10 s) is reported in the node's status with the missing interfaces and counters (#29).
- node-esp32: WebAssembly linear memory is placed in PSRAM on boards that have it, leaving enough internal RAM for actor and firmware OTA on ESP32-DevKitC-VE (#82).
- scripts/imu_analyze.py reads the sample rate from the record, removes per-device zero offsets before the static statistics, finds the device that worsens an axis, estimates tilt independently of mounting yaw, and no longer stops on an empty imu_fused.csv; CI checks it on a 12.5 Hz synthetic record.
- Documentation site, Status and limits: the 2026-11-21 power-loss row now says the device reconnects but does not yet rejoin the computation on hardware (#131).
- README: the 2026-11-21 power-loss row now says the device reconnects but does not yet rejoin the computation on hardware (#131).
- `dyne-sim` and `dyne-sim2d`: `--steps 0` no longer runs one step on the split nodes, so the run keeps the initial field and matches the single-node reference (#138).
- `docs-site/concepts.md` no longer says that a device that lost power rejoins the computation; it reconnects but does not yet rejoin (#145).
- Firmware comment examples use a locally administered MAC address (`02:00:00:…`) instead of one with a real vendor prefix (#146).

## [0.1.0] - 2026-11-15

First public release. Development history since 2026-08 is included.

### Added
- WebAssembly solvers for the 1D/2D heat equation and a learned 1D solver
  (time-discrete physics-informed neural network).
- Deterministic N-device simulators (1D, 2D) with injected packet loss, delay,
  pauses and partitions; split solutions are bit-exact with single-device solutions.
- Adding, removing and handing over devices at runtime.
- ESP32 firmware running the same WebAssembly solvers over Wi-Fi with
  configuration and monitoring over MQTT, and on-device data assimilation.
- Room-shape reconstruction from range sensors (simulator stage).
- Cross-platform regression script (`scripts/ci-check.sh`) and CI.

### Changed
- Wi-Fi scan logs in `measured/` have MAC addresses replaced by pseudonyms
  (consistent within the data set) before publication.

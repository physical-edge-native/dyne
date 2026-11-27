# Components

| Path | Contents |
|---|---|
| `crates/dyne-wire` | message format shared by every host (`no_std`): a 24-byte header (version, kind, quantity, sender, step, timestamp) and a payload per kind (border value and gradient, a band of values, position, hearsay) |
| `crates/dyne-core` | deterministic random numbers, discrete-event clock, virtual network (loss, delay, partitions), WebAssembly loader |
| `crates/dyne-sim`, `crates/dyne-sim2d` | N-device simulators (1D / 2D) |
| `crates/dyne-native` | native PC solvers (2D/3D wave), bit-exact with the WebAssembly solvers; reference and fast backend |
| `crates/dyne-export` | connections to other tools: VTK image data, OpenFOAM cases and velocity import, the room shape as MJCF for MuJoCo (`dyne-vti`, `dyne-foam`, `dyne-mjcf`) |
| `crates/heat1d-actor`, `crates/heat2d-actor`, `crates/wave2d-actor` | finite-difference solvers compiled to WebAssembly |
| `crates/heat1d-pinn-actor` | learned solver (time-discrete physics-informed network) |
| `crates/sdf2d-grid-actor` | room shape from distance sensors |
| `crates/imu-fusion-actor` | accelerometer fusion across devices |
| `crates/swarm-*` | flocking demo on the same runtime |
| `node-esp32/` | ESP32 firmware (all devices identical; role set at run time) |
| `scripts/` | PC tools: configuration, time server, over-the-air updates, observation, live twin and export, analysis, regression checks |
| `theory/`, `reference/` | stability analysis and theoretical values |
| `measured/`, `figs/` | raw measurements from hardware runs and figures |
| `docs/` | design documents, reports and status (Japanese); `docs/en/` is the source of the guides on this site |
| `crates/dyne-host`, `crates/dyne-proto`, `m1-esp32/`, `m2-esp32/` | earlier milestones, kept for reproducibility |

## PC tools used most

| Script | Purpose |
|---|---|
| `scripts/first_flash.py` | flash boards over USB one after another and record which board is which |
| `scripts/dyne_ntp_server.py` | time server for clock synchronisation |
| `scripts/dyne_ota.py` | update the firmware or the solver module on all devices over the air |
| `scripts/wave2d_cluster.py` | configure, run and check the 2D wave equation on a device cluster |
| `scripts/hw_replay.py` | record device input and replay it in the simulator |
| `scripts/twin_live.py` | live twin in the browser; export to USD and `shape.csv` while devices run |
| `scripts/ci-check.sh` | the regression suite run by CI |

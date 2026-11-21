# Status and limits

**Research prototype (v0.x).** Interfaces change without notice. Development records in `docs/` are in Japanese;
the user documentation (this site) is in English.

## Verified so far

| What | Result | Shown on |
|---|---|---|
| 2D wave equation on 4, 8 and 16 ESP32 devices (81 × 81 grid, 400 steps) | every device's region equals the single-solver solution bit for bit; also with 30 % of received messages dropped | hardware, 2026-11-21 |
| Room shape from six devices with pan-servo distance sensors (4.0 m × 2.6 m room) | walls within 49 mm on average (max 100 mm), 83 % of visible surfaces; 7.7 mm in the simulator with the same placement | hardware, 2026-11-21 |
| A device leaving and rejoining during a room run | neighbours report the silence within about 10 s and hold; the device rejoins about 40 s after power returns | hardware, 2026-11-21 |
| Seven accelerometer/gyroscope modules on one board ([report](results/imu-2026-11-21.md)) | gyroscopes follow √N; horizontal accelerometer axes too once one shifted module is left out; tilt within 0.3° of an angle meter; synchronised timestamps halve the disagreement while rocking | hardware, 2026-11-21 |
| Room → USD → Isaac Sim | a Unitree Go2 walks in the room computed by the devices (pretrained policy, inference only) | hardware + Isaac Sim |
| Room → OpenFOAM | 912-cell mesh passes `checkMesh`; `scalarTransportFoam` runs (dummy initial temperatures) | hardware + OpenFOAM v2606 |
| 4 ESP32 devices on batteries with temperature sensors, 4 runs of about 33 min | all completed without restart | hardware |
| 2 ESP32 devices over Wi-Fi | split solution bit-exact | hardware, 2026-08 |
| ESP32 WebAssembly interpreter (WAMR) | all digits match the PC | hardware |
| 1D heat, 2–16 devices; 2D heat up to 4 × 4 | bit-exact, also under loss, delay, partitions, replacement; second-order convergence | simulator, CI |
| Native wave solvers (2D/3D) | bit-exact across splits and against the WebAssembly solver | CI |
| Replay of recorded device input | replay matches; a single flipped bit is detected | CI (synthetic records) |

## Limits stated up front

- **A speed limit set by the network.** With neighbour-only exchange, information crosses one region per exchange,
  so following a field in real time requires region size ÷ exchange delay to exceed the speed at which the physics
  propagates — a CFL-like condition on the network. With today's radio delays this excludes fast phenomena such as
  sound and leaves slow fields (indoor heat, CO2, humidity).
- **Bit-exact under loss, at the cost of delay.** Devices wait for missing border messages.
- **Only the solver is simulated.** The simulator runs the devices' WebAssembly solver, not their firmware or radio.
- **Honest devices only.** Guarantees assume only faults (loss, delay, devices leaving). Authentication and checks
  against implausible border values are future work.
- **No claim of superiority** over centralised or GPU-based setups, and no attempt to replace CFD packages.
- **Fluids: flow given from outside, for now.** Airflow is imported (first from OpenFOAM) and the devices assimilate
  what it carries.
- **Speed on microcontrollers.** Interpreted WebAssembly with software 64-bit floats: about 0.7–1.0 s per step for
  the room-shape solver on an ESP32.
- **Throughput to the PC.** Devices currently publish about one MQTT message per second each, so the dashboard lags
  the devices by tens of seconds ([#114](https://github.com/physical-edge-native/dyne/issues/114)).

## Next

- A twin of a home room that follows furniture being moved.
- Training robots inside the twin of any room (Isaac Lab) ([#119](https://github.com/physical-edge-native/dyne/issues/119)).
- A slow indoor field (temperature, CO2, humidity carried by a given airflow) as the first assimilated 3D field.
- Forecasting by handing the devices' state to a PC with identical bits.
- Simulating many devices with their capabilities (speed, memory, radio delay and loss) as parameters.
- Automatic assignment of regions from device positions.
- Submission to the Journal of Open Source Software (from 2027-05).

Open issues: [github.com/physical-edge-native/dyne/issues](https://github.com/physical-edge-native/dyne/issues).

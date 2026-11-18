# Dyne documentation (English)

This folder holds the English documentation for users. The rest of `docs/`
contains the development records (design notes, reports, status), which are
written in Japanese and named by date; you do not need them to install, run or
extend Dyne.

| Page | What it covers |
|---|---|
| [Installation](installation.md) | Rust and the WebAssembly target, the regression suite, a first 1D and 2D run, Python for the PC tools, ESP-IDF for hardware |
| [Tutorial: a room from simulated sensors to USD and OpenFOAM](tutorial-room.md) | What Dyne is for, end to end without hardware: a dummy room, simulated devices with distance sensors and thermometers, Dyne's shape and temperature, USD with colliders, a 3D dashboard, an OpenFOAM case |
| [Tutorial: write your own solver](tutorial.md) | A new solver from scratch, checked bit for bit in the simulator under loss, delay and a device being replaced |
| [Hardware guide](hardware.md) | ESP32 boards: building and flashing the firmware, clock synchronisation, over-the-air updates, a 2D run on a cluster checked against the simulator, replay of recorded input |
| [Reference](reference.md) | The solver interface, the message format, and how to build the API documentation (`cargo doc`) |

Start with the [README](../../README.md) for what Dyne is and the five steps of
using it.

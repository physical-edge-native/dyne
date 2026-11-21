# The 2D wave equation, split over devices

This tutorial solves the 2D wave equation split into patches, first on a PC and then on ESP32 devices, and checks
that every split gives exactly the same numbers as solving it on one computer.

## 1. On a PC (no hardware)

```sh
# every split must equal the single-patch solution bit for bit
cargo run --release -p dyne-native --bin dyne-native-wave2d -- check --grid 41x41 --steps 400 --splits 1x1,2x2,4x4,8x8,3x5
# accuracy: observed convergence order against analytic standing waves, which must lie in [1.9, 2.1]
cargo run --release -p dyne-native --bin dyne-native-wave2d -- convergence
```

Each patch updates its own grid points with the standard five-point scheme and needs, at every step, only the
values on the edges of its neighbours from the previous step. Because every patch performs the same floating-point
operations in the same order as the single-patch solver, the results agree bit for bit; the check compares a
CRC-32 of the whole field.

The table of expected checksums for each device in a hardware run comes from:

```sh
python scripts/wave2d_expected.py   # 81 x 81 points, Courant number 0.5, 400 steps; 2x2, 4x2 and 4x4 layouts
```

## 2. On devices

!!! warning "Needs hardware"
    ESP32 devices running `node-esp32`, an MQTT broker, and the wave solver module sent over Wi-Fi.

```sh
python scripts/wave2d_cluster.py run --broker <PC address> --layout 4x4 --nodes <16 device ids> --steps 400 --out run4x4
# the same with 30 % of received datagrams dropped on purpose (lost ones are resent)
python scripts/wave2d_cluster.py run --broker <PC address> --layout 4x4 --nodes <16 device ids> --steps 400 --test-drop 0.3 --out run4x4_drop
```

The script sends each device its patch, starts all of them, collects the checksum of each patch at the end, and
compares it with the expected table.

## 3. Results of the 2026-11-21 hardware run

| Layout | Devices matching the expected checksum | Time for 400 steps |
|---|---|---|
| 2 × 2 | 4 / 4 | 57.6 s |
| 4 × 2 | 8 / 8 | 47.2 s |
| 4 × 4 | 16 / 16 | 45.4 s |
| 4 × 4, 30 % of received datagrams dropped | 16 / 16 | 96.5 s |

With losses, each device dropped 665–1,612 datagrams and resent about 1,600–1,800; the result was still identical
bit for bit, and the run took about twice as long. The median wait for a neighbour's edge values was 23–37 ms.

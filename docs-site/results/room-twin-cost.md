# What a room twin costs and how long it takes

This page gathers in one table what it takes to make a twin of a room usable for robot training: the parts for
each device, the time until the twin covers most of the room, and the time to export it to a robot simulator.
Each number says whether it was measured or estimated, and links to its source.

!!! note "How this page was made"
    Prepared by an AI assistant (Claude Code) on 2026-11-27 from the project's records and public catalogue
    prices. The prices are list prices for single units found in online catalogues on 2026-11-27, before tax and
    shipping. They are not records of what the project paid, and they change over time.

## Parts per device

The device used in the 2026-11-21 room run (see [Hardware](../hardware.md)): an ESP32 development board, a
VL53L5CX 8×8-zone distance sensor on an SG90-class pan servo, a breadboard, and two 2×AA battery holders.

| Part | Price range (US$) | Kind | Source |
|---|---|---|---|
| ESP32 development board (ESP32-DevKitC family) | 8–15 | catalogue | [DigiKey ESP32-DEVKITC-32E](https://www.digikey.cn/en/products/detail/espressif-systems/ESP32-DEVKITC-32E/12091810), [Mouser ESP32-DevKitC-32E](https://www.mouser.ca/c/semiconductors/?q=esp32) |
| VL53L5CX distance sensor board | 20–26 | catalogue | [Pololu 3417](https://www.pololu.com/product/3417) (19.95), [SparkFun SEN-19013](https://www.sparkfun.com/sparkfun-qwiic-mini-tof-imager-vl53l5cx.html) (25.95) |
| SG90-class micro servo | 6–8 | catalogue | [Adafruit 169](https://www.adafruit.com/products/169) (SG92R, 5.95), [Soldered TowerPro SG90](https://soldered.com/products/servo-motor-towerpro-sg90) (€5.50) |
| Half-size breadboard | about 5–8 | estimate (converted from A$11.70) | [Little Bird AF-64](https://littlebirdelectronics.com.au/products/half-size-breadboard) |
| Two 2×AA battery holders | 2–8 for both | catalogue | [Adafruit 3951](https://www.adafruit.com/product/3951) (0.95), [Adafruit 4194](https://www.adafruit.com/product/4194) (1.95), [The Pi Hut, with jumper wires](https://www.thepihut.com/products/2-x-aa-battery-holder-with-premium-jumper-header-wires) (£1.70) |
| **One device** | **about 41–65** | sum of the rows above | |
| **Six devices** (as in the 2026-11-21 run) | **about 250–390** | six times one device | |

Not included: AA batteries, jumper wires, USB cables, and the PC and Wi-Fi access point that run the MQTT broker,
DHCP and the time server (see [Hardware](../hardware.md)). The firmware targets the ESP32-DevKitC-VE (8 MB PSRAM);
other ESP32 boards need their own board configuration.

## Time

| Step | Time | Kind | Source |
|---|---|---|---|
| Setting up six devices (placing, power, sending the configuration) | not recorded | — | the 2026-11-21 run did not record it |
| Until 50 % of the final surface coverage | about 14–20 s (20 steps) | estimate: simulator steps × 0.7–1.0 s per step measured on an ESP32 | [How the room twin grows](twin-growth.md) |
| Until 80 % of the final surface coverage | about 25–36 s (36 steps) | estimate, as above | [How the room twin grows](twin-growth.md) |
| Export to USD (Isaac Sim) and MJCF (MuJoCo), with an HTML view | 1.3 s | measured on 2026-11-27, see below | `scripts/twin_demo.py` |
| Export to an OpenFOAM case | under 0.2 s | measured on 2026-11-27, see below | `dyne-foam case` ([Connecting to other tools](../connect.md#openfoam)) |

The coverage times come from the simulator. The same measurement on hardware needs a published record of the
2026-11-21 run, which is tracked in [#161](https://github.com/physical-edge-native/dyne/issues/161).

The export times were measured on the room shape computed by six devices on 2026-11-21
([`docs/demo/siteA-openfoam/shape.csv`](https://github.com/physical-edge-native/dyne/tree/main/docs/demo/siteA-openfoam),
41 × 27 grid points at 0.1 m), on a cloud machine with 4 virtual CPUs (Intel Xeon, 2.3 GHz). Each time is the
wall-clock time of three runs (1.34–1.36 s for USD, MJCF and HTML; 0.004–0.13 s for OpenFOAM, the first run being
the slowest):

```sh
python scripts/twin_demo.py --shape-csv docs/demo/siteA-openfoam/shape.csv --out room   # room.usda, room.xml, room.html
cargo run --release -p dyne-export --bin dyne-foam -- case --sensors docs/demo/siteA-openfoam/readings.csv \
  --shape docs/demo/siteA-openfoam/shape.csv --out roomCase --walls zero-gradient --end-time 3600 \
  --write-interval 600 --delta-t 1
```

`scripts/usd_check.py room.usda` accepts the USD written this way (0 validation findings, 159 collision shapes).
During a live run, `scripts/twin_live.py` writes the USD every 5 s by default.

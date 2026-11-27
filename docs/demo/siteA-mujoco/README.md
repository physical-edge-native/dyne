# MuJoCo model of the real room (hardware run, 2026-11-21)

- `room.xml`: the room shape computed by six devices (`../siteA-openfoam/shape.csv`, 41 × 27 grid, 0.1 m) as an MJCF
  file for MuJoCo: 56 geoms (a floor plane, 4 boxes that close the grid, 40 boxes of solid cells, 11 boxes of cells
  no sensor observed, all 2.4 m high).
- `room_spawn.json`: the free floor where a robot's centre can be placed (912 grid points, 9.12 m², clearance 0).

```sh
cargo run --release -p dyne-export --bin dyne-mjcf -- --shape ../siteA-openfoam/shape.csv --out room.xml
python ../../../scripts/mjcf_room_check.py room.xml room_spawn.json --geoms 56   # needs pip install mujoco
```

`scripts/ci-check.sh` writes both files again and compares them with the ones here byte for byte. With
`--clearance 0.3` (a robot about 0.6 m across) 492 spawn points remain (4.92 m²). Frame, units and file layout: the
"MuJoCo" section of the docs site's connect page (`docs-site/connect.md`).

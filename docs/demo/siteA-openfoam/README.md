# OpenFOAM on the real room (hardware run, 2026-11-21)

- `shape.csv`: the room shape computed by six devices (41 × 27 grid, 0.1 m), as written by `scripts/twin_live.py`.
- `readings.csv`: dummy temperatures at the six device positions (not measurements).
- `plot_foam.py`: draws `0/T` and `3600/T` of the case on the cell centres.

```sh
cargo run --release -p dyne-export --bin dyne-foam -- case --sensors readings.csv --shape shape.csv \
  --out roomCase --walls zero-gradient --end-time 3600 --write-interval 600 --delta-t 1
cd roomCase && checkMesh && scalarTransportFoam && postProcess -func writeCellCentres -time 0 && cd ..
python plot_foam.py roomCase openfoam_room.png
```

Run with OpenFOAM.com v2606 (Ubuntu 26.04 package `openfoam`, under WSL): 912 cells, `Mesh OK`; T goes from
18.0–24.0 to 18.4–23.5 after 3600 s.

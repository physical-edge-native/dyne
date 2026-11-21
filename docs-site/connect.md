# Connecting to other tools

Dyne does not try to replace established simulators. It keeps a coarse model fitted to the sensors on the devices
and hands the state of the real space to the tools you already use.

| Target | What is exported | Use | Status |
|---|---|---|---|
| USD (Isaac Sim, Isaac Lab and other USD tools) | room shape and objects with collision shapes | robot simulation and learning in a scene that matches the real room | works with the room computed by six devices (2026-11-21): a Unitree Go2 walks in it with a pretrained policy |
| VTK image data (ParaView, PyVista) | 2D/3D field state, stored losslessly | inspection and post-processing | implemented (`--vti PATH`), read-back checked bit for bit in CI |
| OpenFOAM case (`scalarTransportFoam`) | grid, initial field, boundary conditions, given velocity and diffusivity | detailed what-if studies | implemented (`dyne-foam case`) |
| OpenFOAM case with the room's walls and furniture | cells inside walls and objects left out; their faces form the patch `walls` | the real room in a CFD study | runs with OpenFOAM v2606 on the real room of 2026-11-21 (912 cells, `checkMesh` passes) |
| OpenFOAM initial temperature from point sensors | a few readings spread over the grid (inverse-distance weighting) | start a CFD study from the measured state | implemented (`dyne-foam case --sensors`), runs with OpenFOAM v2606 |
| OpenFOAM velocity (import) | airflow computed by OpenFOAM, mapped onto Dyne's grid | the given airflow that carries temperature, CO2, humidity | implemented, checked on a synthetic file |

## Robot simulators (USD)

![Unitree Go2 walking in the USD exported from the room computed by six devices](img/go2.jpg)

The shape computed by the devices is a signed distance field (distance to the nearest surface, negative inside
objects). The export turns surface lines into thick boards with colliders (walls 10 cm, furniture 5 cm) and treats
regions no sensor has observed as solid, so a robot cannot leave through a hole. `scripts/twin_live.py` writes the
USD while the devices run; `scripts/usd_check.py` checks it. The procedure for Isaac Sim and Isaac Lab with a Go2 is in
`docs/2026-11-18-isaac-go2-procedure.md` (Japanese).

## ParaView (VTK)

```sh
# write the final field of a run as VTK image data, then print each array's CRC-32
cargo run --release -p dyne-native --bin dyne-native-wave2d -- check --grid 41x41 --steps 400 --splits 1x1,2x2 --vti wave2d.vti
cargo run --release -p dyne-export --bin dyne-vti -- info wave2d.vti
```

Values are stored as raw little-endian 64-bit floats, so the file reads back bit for bit. `dyne-sim2d --vti PATH`
writes the field of a split simulator run the same way.

## OpenFOAM

![OpenFOAM on the mesh made from the real room shape; dummy initial temperatures](img/openfoam.png)

Each point of Dyne's grid becomes the centre of one OpenFOAM cell, so values carry over one to one; numbers are
written in the shortest decimal form that reads back to the same 64-bit float.

```sh
# a scalarTransportFoam case from a .vti field (faces west,east,south,north[,bottom,top])
cargo run --release -p dyne-export --bin dyne-foam -- case --vti wave2d.vti --out wave2dCase --velocity 0.1,0,0 \
  --faces fixed:0,zero-gradient,zero-gradient,zero-gradient
cd wave2dCase && blockMesh && scalarTransportFoam

# the real room: shape from the devices, initial temperature from a few point readings
cargo run --release -p dyne-export --bin dyne-foam -- case --sensors readings.csv --shape shape.csv \
  --out roomCase --walls zero-gradient --end-time 3600 --write-interval 600 --delta-t 1
cd roomCase && checkMesh && scalarTransportFoam

# OpenFOAM -> Dyne: map the velocity of a CFD case onto Dyne's grid, then read it
cargo run --release -p dyne-export --bin dyne-foam -- case --vti wave2d.vti --out flowGrid
(cd flowGrid && blockMesh && mapFields ../roomFlow -sourceTime latestTime)
cargo run --release -p dyne-export --bin dyne-foam -- velocity flowGrid 0 --vti flow.vti
```

The inputs of the real-room run (the room shape and the dummy readings) and the plotting script are in
[`docs/demo/siteA-openfoam/`](https://github.com/physical-edge-native/dyne/tree/main/docs/demo/siteA-openfoam).
Results of OpenFOAM and Dyne differ by their discretisation, so comparing them needs a stated tolerance. The
mapping of boundary types and notes on the openfoam.org releases are in the API documentation of
`dyne_export::foam` (`cargo doc -p dyne-export --open`).

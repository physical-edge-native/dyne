# Design record: walls of the room and the initial temperature from sensors in the OpenFOAM export

- **Date decided:** 2026-11-18
- **Decided by:** the maintainer (what to export: the real room shape for the 2026-11-22 talk);
  the representation below was chosen while implementing and is open to review
- **Issues:** #91 (walls and obstacles), #92 (initial temperature from point sensors), #57 (OpenFOAM export)

## Part 1: walls and obstacles (#91)

### Context

`dyne-foam` writes a `scalarTransportFoam` case in which every Dyne grid point is the centre of one
cell of a single `blockMesh` block. The room shape that the devices build from distance sensors (a
signed distance field `phi` with observation weights `w`, the same arrays that feed the USD export)
was not passed on, so walls inside the domain and furniture were missing.

### Decision

1. **Solid points.** A grid point is solid if it was observed (`w > 0`) and `phi < 0`; a point that
   was not observed (`w = 0`) is solid by default, as in the USD export (`Unobserved`), or free with
   `--unobserved fluid`. Only the largest connected region of free cells is kept (`--regions all`
   keeps all), so pockets enclosed by walls do not become separate pieces of the mesh.
2. **The mesh is written directly in OpenFOAM's own format** (`constant/polyMesh`: `points`, `faces`,
   `owner`, `neighbour`, `boundary`) with only the free cells. Faces between a free and a solid cell
   form the patch `walls` (type `wall`); the sides of the grid keep their patches (`west`, ...), and a
   2D case keeps `frontAndBack` (`empty`). `T` on `walls` is zero gradient by default (`--walls`);
   `U` is zero there.
3. `system/dyneGrid` lists the solid points, so the case reads back and a velocity field can be
   imported onto Dyne's grid. Without a shape, the export is unchanged byte for byte.

### Reasons

- Writing the mesh directly needs no OpenFOAM step whose behaviour on fields we would have to trust
  (the run is just `checkMesh` and the solver), and the result is fully checkable without OpenFOAM:
  CI compares a small case byte for byte with a fixture and checks closed cells, outward faces,
  face order and connectivity in code.
- The same code writes 2D and 3D meshes (a cell is a box on any axis), so it extends to 3D.
- Checked with OpenFOAM (ESI openfoam2606, release candidate 2606.0~rc2, WSL Ubuntu, 2026-11-18):
  `checkMesh` reports "Mesh OK." for the fixture (4x3 grid), the simulated furnished room
  (`scenes/room6.txt`, 73x55 grid, 2711 cells, 387 wall faces) and a 3D test case (8x6x5, 222 cells);
  `scalarTransportFoam` runs on all three. In the room run (zero velocity, zero gradient everywhere)
  the sum of `T` over the cells changed by 3.6e-11 relative after 20 steps, i.e. no heat leaks
  through the walls beyond the linear solver tolerance (1e-10).
- A uniform non-zero velocity together with walls is not a valid flow: it runs into the walls, where
  `U` is zero, so the flux is not conserved. In a test run (`--velocity 0.02,0.01,0`, 60 s) `T` grew
  to values between −434 and 2692 from an initial range of 20.9–23.1. `dyne-foam` prints a warning in
  that case; with walls, use zero velocity or a velocity field computed on the same mesh.
- `checkMesh` notes "1 bad edges (possibly relevant for finite-area)" for the room: two free cells
  that touch only along an edge (diagonal neighbours between two solid cells). It is not a mesh error
  for finite-volume solvers.

### Alternatives considered

- **`blockMesh` + `topoSet` + `subsetMesh -patch walls`** in the case scripts: standard tools, but the
  cell removal happens inside OpenFOAM, so CI could not check the result, and how `subsetMesh` sets
  the field values on the new patch would have had to be verified per release.
- **One `blockMesh` block per free cell**: `blockMesh` merges the shared faces itself, but thousands
  of blocks make a large dictionary and the wall faces must still be listed.
- **`snappyHexMesh` with a surface of the room** (e.g. from the USD export): body-fitted walls, but
  many settings and no exact correspondence between cells and Dyne's grid points; a later option when
  a finer mesh than Dyne's grid is wanted.
- **Keep all cells and make walls a region of very low diffusivity**: no mesh change, but not a wall
  for the flow and not visible as a patch.

## Part 2: initial temperature from a few point sensors (#92)

### Context

For the talk, the temperatures read at 4–6 points of the room (DHT20 / BME280 on the devices) become
the initial temperature field of the OpenFOAM case. A field of the running twin (a heat model nudged
to the sensors) is not available on the devices yet, so the field is built from the readings.

### Decision

1. **Inverse-distance weighting** (Shepard's method, power 2, straight-line distances in the plane
   of a 2D grid): `T(p) = Σ v_i/d_i² / Σ 1/d_i²`, the reading itself at a sensor's position.
   `dyne-foam case --sensors READINGS.csv` (columns `x`, `y`, optional `z`, `T`; the grid comes from
   `--vti` or `--shape`).
2. **Stated meaning and error, before any comparison:** the field is an initial guess, not a
   measurement. Exact properties: every value lies between the smallest and largest reading (clamped,
   so rounding cannot leave the range); a grid point at a sensor gets its reading; equal readings give
   a uniform field; far from the sensors the value tends to their mean. The error between sensors
   cannot be stated (the readings do not resolve the field); the only bound is the spread of the
   readings plus the sensor accuracy (about ±0.5 °C DHT20, ±1 °C BME280, from the data sheets).
3. The sums run in the order of the file, so the field is the same bit for bit on every machine;
   CI pins its CRC-32 for five dummy readings in the simulated room.

### Reasons

- With 4–6 points, a method with no parameters to tune and no overshoot is the robust choice: it
  cannot produce temperatures outside the measured range, and it works for any layout of sensors
  (no triangulation, no minimum number of points).
- It extends to 3D unchanged (`z` column, distances in 3D).
- Checked with OpenFOAM (openfoam2606 rc, 2026-11-18): the case with the walls of the simulated room
  and the five dummy readings passes `checkMesh` and runs with `scalarTransportFoam`; with walls held
  at 19 °C, `T` stays between 19.0 and 23.1 and decays towards 19 °C.

### Alternatives considered

- **Field of the running twin** (heat model nudged to the sensors): the better source once the
  devices run it; then the existing `--vti` path is used instead of `--sensors`.
- **Harmonic interpolation** (steady heat conduction with the readings as fixed values, walls
  insulating): respects walls and is physically motivated, but needs an iterative solve and a
  tolerance; a candidate if the straight-line weighting through furniture turns out misleading.
- **Linear interpolation on a triangulation, constant outside the convex hull**: smoother inside, but
  needs at least three points not in a line, and most of a room lies outside the hull of 4–6 sensors.
- **Gaussian process / kriging**: gives an error estimate, but its length scale and noise must be
  chosen, and with 4–6 points those choices dominate the result.

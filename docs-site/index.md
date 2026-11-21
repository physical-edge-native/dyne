# Dyne

Dyne runs physics models (numerical solvers and small neural networks) on a group of microcontrollers placed in a
real space. Each device computes its own part of the space and exchanges only the values on the edges of its part
with its neighbours. The result is a digital twin that lives on the devices; a PC is only needed to configure,
watch and export it (to USD for Isaac Sim, to VTK for ParaView, to OpenFOAM cases).

![A real 4.0 m × 2.6 m room with six devices](img/real.jpg){ width="49%" }
![The twin of the same room, computed on the devices](img/twin.jpg){ width="49%" }

!!! note "Simulator and hardware"
    Everything in these pages can be run on a PC with the simulator, without hardware. Pages that need hardware
    say so. The photos above are from a hardware run on 2026-11-21; most other figures come from the simulator.

## Where to start

- [Getting started](getting-started.md): build a solver and run it split over 16 simulated devices (no hardware).
- [Concepts](concepts.md): patches, edges, why results match bit for bit.
- [A real room, end to end](tutorials/real-room.md): six devices map a room, the twin goes to Isaac Sim.
- [The 2D wave equation](tutorials/wave-equation.md): split solver on a PC and on 16 devices, bit for bit.
- [Hardware](hardware.md): the parts used and how devices are configured.

Source code: [github.com/physical-edge-native/dyne](https://github.com/physical-edge-native/dyne).
Cite with the concept DOI [10.5281/zenodo.24241009](https://doi.org/10.5281/zenodo.24241009).

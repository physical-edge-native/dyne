# Concepts

- **Patch.** The space is split into rectangular parts; each device owns one part and computes it.
- **Edge values.** Neighbouring devices exchange only the values on the shared edge of their parts, once per step.
  A step uses the neighbour's values from the previous step.
- **Bit-exact check.** A split run must give exactly the same numbers, bit for bit, as the same model solved on one
  device. This is how every solver in Dyne is tested, in the simulator and on hardware.
- **Solvers as WebAssembly modules.** The same compiled module runs in the PC simulator and on the ESP32 devices,
  so a result can be replayed on a PC and compared bit for bit.
- **Joining and leaving.** Devices can stop and come back while a run goes on; neighbours detect the silence,
  hold the run, and resume when the device returns.
- **Export.** The twin's state can be written as USD (Isaac Sim, Omniverse), VTK image data (ParaView) or an
  OpenFOAM case.

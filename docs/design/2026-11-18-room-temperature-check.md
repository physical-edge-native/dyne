# Room temperature: how the tutorial's field is checked, and why it fails

Date: 2026-11-18. Issue: #96 (room tutorial). Decided by the maintainer; this record was written with an AI assistant.

## Decision

1. The room tutorial's temperature field is checked by **hold-out**: each thermometer in turn is left out of
   the run, and the model must predict its reading within 0.5 C (the accuracy class of common room
   thermometers). The splitting of the room over devices is checked separately, bit for bit.
2. The first check (agreement at the nudged thermometers, 0.5 C) is withdrawn. A nudged model agrees with the
   points it is nudged to whenever gamma is large enough (gamma = 1 copies the reading). The first version of
   the tutorial failed it with gamma = 0.5 and then passed it with gamma = 0.8; that passed nothing about the
   field.
3. The tutorial keeps today's heat model (no heater, no window), which **fails** the hold-out check, and says
   so. Learning the missing heat sources from the readings is issue #100.
4. Temperature stays the field of the room tutorial (see "Other fields" below).

## Where the error comes from

Readings: 12 dummy thermometers (`crates/dyne-sim2d/scenes/room6_temp.csv`), sampled from a made-up steady
field with a heater (30 C, west wall) and a window (12 C, north wall). Commands: `scripts/tutorial_room.py check`
(Dyne itself), `scripts/room_temp_model_error.py` and `scripts/room_temp_learn_walls.py` (PC, numpy).

| Case | Hold-out error, mean / max |
|---|---|
| Dyne, today's model, gamma 0.8 (`tutorial_room.py check`) | 1.68 / 6.16 C |
| Dyne, today's model, gamma 0.5 | 1.87 / 6.57 C |
| PC, model with the heater and window given (the made-up field's own equation), gamma 0.5 or 0.8 | 0.13 / 0.29-0.30 C |
| PC, heat input along every wall cell learned from the readings, 12 thermometers | 0.24 / 0.86 C |
| same, 24 thermometers | 0.14 / 0.47 C |
| same, 96 thermometers | 0.11 / 0.38 C |

- Splitting: the 32-device field is bit-identical to the single-device field, and the numpy re-implementation
  of today's model matches Dyne's output to 7e-15. The split adds no error.
- Time: 2000 and 20000 steps give the same field (the error is not a matter of running longer).
- gamma: 0.2 to 1.0 change the error at the nudged points, not the field between them.
- Model: with the heater and window in the model, every gamma from 0.2 to 1.0 passes. **The cause is the
  model's missing heat sources**, not the architecture.
- Learning: with no information about the heater or the window, the fit put the largest heat input on the west
  wall at y = 1.3-1.7 m (heater: 0.8-1.8 m) and the largest heat loss on the north wall at x = 1.3-1.7 m
  (window: 1.3-2.9 m), with 12 thermometers already. These numbers favour the fit: the made-up field was made
  with the same kind of equation, the regularisation strength was picked by the same hold-out error that is
  reported, and the furniture was taken from the scene file (on the devices it would come from the measured
  room shape).

## Next: learning the heat input on the devices (issue #100)

Chosen: keep the heat model and learn, on each device, the heat input of the wall cells in its own region,
by the gradient of the misfit to the readings. Heat diffusion is self-adjoint, so the gradient is a second run
of the same heat model with the misfits as sources; it uses the same interface exchange and stays bit-exact
across splits. Only the devices whose region touches a wall carry the learning part; the others run the
classical heat solver only (forward and adjoint runs), since a learned model everywhere is not needed.
The furniture (no heat through it) comes from the room shape the distance sensors measure. The tutorial would
use 24 thermometers, so the 0.5 C hold-out tolerance can be met.

Other options considered:
- Give the heater and window to the model as fixed-temperature bands: passes the check, but someone has to know
  and enter where they are and how hot; rejected as the main path.
- Add each misfit near a wall directly to that wall's heat input: simpler, but walls far from thermometers are
  never corrected.
- A neural network for the whole field (the time-discretised PINN actor in 2D): far more unknowns than 12-24
  thermometers can fix, and no physical guarantees.

## Other fields

The question was whether a different field (air flow, electromagnetic) would give a sharper pass criterion.
It would not: the split is checked bit for bit whatever the field, and the assimilation is checked against
held-out measurements whatever the field. Temperature suits a room on microcontrollers: slow (fits a step of
about 20 ms exchange), cheap sensors, a stable equation, and links to OpenFOAM's thermal solvers. Air flow
(nonlinear, turbulent, expensive and directional sensors) and indoor radio fields (12 cm wavelength at
2.4 GHz, nanosecond steps) do not fit microcontrollers; sound needs exchanges at kHz rates. Stress and strain
in a beam or plate are a well-posed later candidate. A sharp criterion for a known field already exists on the
hardware: the 2D wave with an analytic solution (wave2d).

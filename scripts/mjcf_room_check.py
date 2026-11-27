#!/usr/bin/env python3
"""Load a room written by dyne-mjcf in MuJoCo and check it (GitHub issue #193).

    python scripts/mjcf_room_check.py ROOM.xml ROOM_spawn.json --geoms N

Checks:
  1. mujoco.MjModel.from_xml_path loads the file, and it has N geoms (the count dyne-mjcf prints);
     every geom is static (on the world body) and every box stands on the floor (bottom at z = 0).
  2. No point of the spawn rectangles (every grid point inside, edges included) lies inside a box geom.
  3. A ball (radius 0.1 m) dropped from 0.5 m at the middle spawn point rests on the floor after 1 s
     (centre height 0.1 m within 5 mm), and stays within 5 mm of its start in x and y.
Prints "mjcf-room OK ..." and exits 0, or prints the failures and exits 1.
Needs the mujoco package (pip install mujoco) and numpy.
"""
import argparse
import json
import sys

import mujoco
import numpy as np


def spawn_points(spawn):
    """Every spawn grid point of the rectangles, as an (n, 2) array."""
    sx, sy = spawn["grid"]["spacing"]
    pts = []
    for x0, y0, x1, y1 in spawn["rectangles"]:
        nx = int(round((x1 - x0) / sx)) + 1
        ny = int(round((y1 - y0) / sy)) + 1
        for j in range(ny):
            for i in range(nx):
                pts.append((x0 + i * sx, y0 + j * sy))
    return np.array(pts, float).reshape(-1, 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("xml")
    ap.add_argument("spawn")
    ap.add_argument("--geoms", type=int, required=True, help="expected number of geoms")
    a = ap.parse_args()
    bad = []
    m = mujoco.MjModel.from_xml_path(a.xml)
    with open(a.spawn, encoding="utf-8") as f:
        spawn = json.load(f)

    if m.ngeom != a.geoms:
        bad.append("%d geoms, expected %d" % (m.ngeom, a.geoms))
    if m.nbody != 1 or np.any(m.geom_bodyid != 0):
        bad.append("not every geom is on the world body (nbody %d)" % m.nbody)
    boxes = np.flatnonzero(m.geom_type == mujoco.mjtGeom.mjGEOM_BOX)
    planes = np.flatnonzero(m.geom_type == mujoco.mjtGeom.mjGEOM_PLANE)
    if len(planes) != 1 or len(boxes) != m.ngeom - 1:
        bad.append("expected one plane and only boxes (%d planes, %d boxes)" % (len(planes), len(boxes)))
    bottom = m.geom_pos[boxes, 2] - m.geom_size[boxes, 2]
    if np.any(np.abs(bottom) > 1e-9):
        bad.append("a box does not stand on the floor (bottom z from %g to %g)" % (bottom.min(), bottom.max()))

    pts = spawn_points(spawn)
    if len(pts) != spawn["spawn_points"]:
        bad.append("%d points in the rectangles, spawn_points says %d" % (len(pts), spawn["spawn_points"]))
    lo = m.geom_pos[boxes, :2] - m.geom_size[boxes, :2]
    hi = m.geom_pos[boxes, :2] + m.geom_size[boxes, :2]
    eps = 1e-9
    inside = np.zeros(len(pts), bool)
    for k in range(len(boxes)):
        inside |= np.all((pts > lo[k] + eps) & (pts < hi[k] - eps), axis=1)
    if inside.any():
        bad.append("%d spawn points inside a box, e.g. %s" % (inside.sum(), pts[inside][0]))

    if len(pts):
        cx, cy = pts[len(pts) // 2]
        with open(a.xml, encoding="utf-8") as f:
            text = f.read()
        ball = ('    <body name="ball" pos="%.6f %.6f 0.5"><freejoint/>'
                '<geom type="sphere" size="0.1" mass="1"/></body>\n  </worldbody>' % (cx, cy))
        mb = mujoco.MjModel.from_xml_string(text.replace("  </worldbody>", ball, 1))
        d = mujoco.MjData(mb)
        for _ in range(int(round(1.0 / mb.opt.timestep))):
            mujoco.mj_step(mb, d)
        x, y, z = d.qpos[:3]
        if abs(z - 0.1) > 5e-3 or abs(x - cx) > 5e-3 or abs(y - cy) > 5e-3:
            bad.append("ball at (%.4f, %.4f, %.4f) after 1 s, expected (%.4f, %.4f, 0.1)" % (x, y, z, cx, cy))
        rest = "ball rests at z %.4f m" % z
    else:
        bad.append("no spawn point")
        rest = "no ball"

    if bad:
        for b in bad:
            print("mjcf-room FAIL: " + b)
        return 1
    print("mjcf-room OK (mujoco %s: %d geoms, %d boxes on the floor; %d spawn points outside every box; %s)"
          % (mujoco.__version__, m.ngeom, len(boxes), len(pts), rest))
    return 0


if __name__ == "__main__":
    sys.exit(main())

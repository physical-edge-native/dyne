#!/usr/bin/env python3
"""Separate why gamma=0.5 failed in the room tutorial (#96): Dyne's split, the nudging method, or the model.

M0 = Dyne's heat model as run in the tutorial (whole rectangle, no heater/window, insulated outer edge).
     Re-implemented in numpy on one domain; checked against Dyne's .vti (Dyne's split == single node, bit-exact).
M1 = the "right" model: the same equation that made the dummy truth (steady diffusion with a 20 C relaxation,
     length 1.2 m, heater 30 C and window 12 C as fixed bands, furniture insulated), time-stepped at h=0.1.
Both are nudged at the nearest grid point after each step: u_k += gamma (y - u_k).

Usage (from the repository root, after scripts/tutorial_room.sh has built dyne-sim2d; the comparison with
Dyne's own output needs demo_out/iso_g0.5.vti and demo_out/iso_g0.8.vti, see docs/design/2026-11-18-room-temperature-check.md):
    python scripts/room_temp_model_error.py .
"""
import sys, os
sys.path.insert(0, sys.argv[1] + "/scripts")
import numpy as np
import twin_demo, tutorial_room as tr

REPO = sys.argv[1]
sc = twin_demo.parse_scene(REPO + "/crates/dyne-sim2d/scenes/room6.txt")
rd = tr.read_readings(REPO + "/crates/dyne-sim2d/scenes/room6_temp.csv")
tf = tr.truth_field(sc)
x0, y0, x1, y1 = tf["bounds"]
h, dt, alpha = 0.1, 0.002, 1.0
nx, ny = int(round((x1 - x0) / h)) + 1, int(round((y1 - y0) / h)) + 1
r = alpha * dt / h / h
xs, ys = x0 + h * np.arange(nx), y0 + h * np.arange(ny)
sens = [(int(np.floor((float(q["x"]) - x0) / h + 0.5)), int(np.floor((float(q["y"]) - y0) / h + 0.5)),
         float(q["T"]), float(q["truth"]), q["id"], float(q["x"]), float(q["y"])) for q in rd]

free = np.zeros((ny, nx), bool)
for j in range(ny):
    for i in range(nx):
        free[j, i] = twin_demo.is_free(sc, min(max(xs[i], x0 + 1e-6), x1 - 1e-6), min(max(ys[j], y0 + 1e-6), y1 - 1e-6))
truth = np.array([[twin_demo.sample_temp(tf, xs[i], ys[j]) for i in range(nx)] for j in range(ny)])

# M1 ingredients at h = 0.1 (same band rule as twin_demo.dummy_temperature)
fixed = np.zeros((ny, nx), bool); fval = np.zeros((ny, nx))
for seg, t in ((tf["heater"], 30.0), (tf["window"], 12.0)):
    (ax, ay), (bx, by) = seg
    ex, ey = bx - ax, by - ay
    for j in range(ny):
        for i in range(nx):
            u = max(0.0, min(1.0, ((xs[i] - ax) * ex + (ys[j] - ay) * ey) / (ex * ex + ey * ey)))
            if np.hypot(ax + u * ex - xs[i], ay + u * ey - ys[j]) < 0.6 * h:
                fixed[j, i] = True; fval[j, i] = t
fixed &= free


def lap_m0(T):
    P = np.pad(T, 1, mode="reflect")  # zero_grad of heat2d-actor: ghost = mirror point (1 / n-2)
    return P[:-2, 1:-1] + P[2:, 1:-1] + P[1:-1, :-2] + P[1:-1, 2:] - 4 * T


def lap_m1(T):
    P = np.pad(T, 1, mode="edge"); F = np.pad(free, 1, constant_values=False)
    s = np.zeros_like(T)
    for dj, di in ((0, 1), (2, 1), (1, 0), (1, 2)):
        s += np.where(F[dj:dj + ny, di:di + nx], P[dj:dj + ny, di:di + nx], T)
    return s - 4 * T


def run(model, gamma, steps, use=None, init=20.0, noisy=True):
    T = np.full((ny, nx), init)
    if model == "m1":
        T[fixed] = fval[fixed]
    use = range(len(sens)) if use is None else use
    k = h * h / 1.2 ** 2
    for _ in range(steps):
        if model == "m0":
            T = T + r * lap_m0(T)
        else:
            T = T + r * (lap_m1(T) - k * (T - 20.0))
            T[fixed] = fval[fixed]
        for s in use:
            gi, gj, y, yt = sens[s][:4]
            if gamma > 0:
                T[gj, gi] += gamma * ((y if noisy else yt) - T[gj, gi])
    return T


def at(T, x, y):
    u, v = (x - x0) / h, (y - y0) / h
    i, j = min(max(int(u), 0), nx - 2), min(max(int(v), 0), ny - 2); s, t = u - i, v - j
    return (1 - s) * (1 - t) * T[j, i] + s * (1 - t) * T[j, i + 1] + s * t * T[j + 1, i + 1] + (1 - s) * t * T[j + 1, i]


def report(name, T):
    res = [abs(q[2] - at(T, q[5], q[6])) for q in sens]
    e = np.abs(T - truth)[free]
    print("%-34s reading-resid max %.3f mean %.3f | field err vs truth mean %.2f p95 %.2f max %.2f"
          % (name, max(res), np.mean(res), e.mean(), np.percentile(e, 95), e.max()))


def holdout(model, gamma, steps):
    errs = []
    for s in range(len(sens)):
        T = run(model, gamma, steps, use=[q for q in range(len(sens)) if q != s])
        errs.append(abs(at(T, sens[s][5], sens[s][6]) - sens[s][2]))
    return np.array(errs)


# 0. numpy M0 reproduces Dyne
for g in (0.5, 0.8):
    p = REPO + "/demo_out/iso_g%s.vti" % g
    if os.path.exists(p):
        v = twin_demo.read_temp_vti(p)["T"]
        print("numpy M0 vs Dyne .vti gamma %.1f: max |diff| %.2e" % (g, np.abs(run("m0", g, 2000) - v).max()))

print("\n-- 1. the model alone (no nudging) --")
report("M0 no nudge, 2000 steps", run("m0", 0, 2000))
report("M1 no nudge, 5000 steps", run("m1", 0, 5000))
print("\n-- 2. gamma sweep, tutorial length (2000 steps = 4 s) and long (20000 = 40 s) --")
for g in (0.2, 0.5, 0.8, 1.0):
    report("M0 gamma %.1f  2000" % g, run("m0", g, 2000))
for g in (0.2, 0.5, 0.8, 1.0):
    report("M0 gamma %.1f 20000" % g, run("m0", g, 20000))
for g in (0.2, 0.5, 0.8, 1.0):
    report("M1 gamma %.1f  5000" % g, run("m1", g, 5000))
print("\n-- 3. hold-out: leave one thermometer out, error at it vs its reading --")
for m, st in (("m0", 2000), ("m1", 5000)):
    for g in (0.5, 0.8):
        e = holdout(m, g, st)
        print("%s gamma %.1f: hold-out |err| mean %.2f max %.2f  per id %s"
              % (m.upper(), g, e.mean(), e.max(), " ".join("%s:%.2f" % (sens[i][4], e[i]) for i in range(len(e)))))

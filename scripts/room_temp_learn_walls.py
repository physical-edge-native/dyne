#!/usr/bin/env python3
"""Can the heater/window be learned from thermometer readings instead of being given? (#96 follow-up)

Model: steady  -lap T + k (T - Ta) = q  on the free floor (furniture insulated: taken from the room shape).
Unknowns: q on every cell along the room's outer walls (heat put in or taken out there: heater, window, or 0),
          and Ta (a constant). k is scanned. Nothing tells the fit where the heater or the window is.
Fit: least squares to the readings + lambda * (differences of q along the wall)^2, lambda picked by
     leave-one-out (LOO) error. Reported: LOO error (each thermometer predicted without it), field error vs truth,
     and where the fitted q is largest (does it find the heater and the window?).

Usage (from the repository root): python scripts/room_temp_learn_walls.py .
A PC calculation of what the data can tell (identifiability); not yet run on the simulated devices.
"""
import sys
sys.path.insert(0, sys.argv[1] + "/scripts")
import numpy as np
import twin_demo, tutorial_room as tr

REPO = sys.argv[1]
sc = twin_demo.parse_scene(REPO + "/crates/dyne-sim2d/scenes/room6.txt")
rd = tr.read_readings(REPO + "/crates/dyne-sim2d/scenes/room6_temp.csv")
tf = tr.truth_field(sc)
x0, y0, x1, y1 = tf["bounds"]
h = 0.1
nx, ny = int(round((x1 - x0) / h)) + 1, int(round((y1 - y0) / h)) + 1
xs, ys = x0 + h * np.arange(nx), y0 + h * np.arange(ny)
free = np.zeros((ny, nx), bool)
for j in range(ny):
    for i in range(nx):
        free[j, i] = twin_demo.is_free(sc, min(max(xs[i], x0 + 1e-6), x1 - 1e-6), min(max(ys[j], y0 + 1e-6), y1 - 1e-6))
truth = np.array([[twin_demo.sample_temp(tf, xs[i], ys[j]) for i in range(nx)] for j in range(ny)])
cells = [(j, i) for j in range(ny) for i in range(nx) if free[j, i]]
idx = {c: n for n, c in enumerate(cells)}
N = len(cells)
# wall cells: free cells on the outer edge of the grid, ordered around the room
ring = [(0, i) for i in range(nx)] + [(j, nx - 1) for j in range(1, ny)] + \
       [(ny - 1, i) for i in range(nx - 2, -1, -1)] + [(j, 0) for j in range(ny - 2, 0, -1)]
wall = [c for c in ring if free[c]]
W = len(wall)

L0 = np.zeros((N, N))
for n, (j, i) in enumerate(cells):
    for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        jj, ii = j + dj, i + di
        if 0 <= jj < ny and 0 <= ii < nx and free[jj, ii]:
            L0[n, n] += 1; L0[n, idx[(jj, ii)]] -= 1   # insulated neighbours / outer edge: no flux


def design(k):
    A = L0 + h * h * k * np.eye(N)
    Ainv = np.linalg.inv(A)
    B = np.zeros((N, W + 1))
    for w, c in enumerate(wall):
        B[idx[c], w] = 1.0             # q at wall cell (scaled by h^2)
    B[:, W] = h * h * k                # Ta
    return Ainv @ B                    # T = G @ [q..., Ta]


D = np.zeros((W - 1, W + 1))
for w in range(W - 1):
    D[w, w], D[w, w + 1] = -1, 1


def fit(G, obs, y, lam):
    Go = G[obs]
    M = Go.T @ Go + lam * D.T @ D + 1e-10 * np.eye(W + 1)
    return np.linalg.solve(M, Go.T @ y)


def loo(G, obs, y, lam):
    e = []
    for s in range(len(obs)):
        keep = [t for t in range(len(obs)) if t != s]
        p = fit(G, [obs[t] for t in keep], y[keep], lam)
        e.append(abs(G[obs[s]] @ p - y[s]))
    return np.array(e)


rng = np.random.default_rng(97)
base = []
for q in rd:
    gi = int(np.floor((float(q["x"]) - x0) / h + 0.5)); gj = int(np.floor((float(q["y"]) - y0) / h + 0.5))
    base.append((idx[(gj, gi)], float(q["T"])))
extra_pool = [n for n in rng.permutation(N)]
fe = free.copy()
for nsens in (12, 24, 48, 96):
    obs = [b[0] for b in base]; y = [b[1] for b in base]
    for n in extra_pool:
        if len(obs) >= nsens:
            break
        if n in obs:
            continue
        j, i = cells[n]
        obs.append(n); y.append(round(truth[j, i] + rng.normal(0, 0.1), 2))
    y = np.array(y)
    best = None
    for k in (0.25, 0.5, 0.694, 1.0, 2.0):
        G = design(k)
        for lam in (1e-4, 1e-3, 1e-2, 1e-1, 1.0):
            e = loo(G, obs, y, lam)
            if best is None or e.mean() < best[0]:
                best = (e.mean(), e.max(), k, lam, G)
    _, _, k, lam, G = best
    p = fit(G, obs, y, lam)
    T = G @ p
    err = np.abs(T - np.array([truth[c] for c in cells]))
    e = loo(G, obs, y, lam)
    q = p[:W]
    top_hot = [wall[w] for w in np.argsort(-q)[:3]]
    top_cold = [wall[w] for w in np.argsort(q)[:3]]
    fmt = lambda cs: " ".join("(%.1f,%.1f)" % (xs[c[1]], ys[c[0]]) for c in cs)
    print("%3d thermometers: k=%.3g lam=%.0e  LOO mean %.2f max %.2f | field err mean %.2f p95 %.2f max %.2f | Ta %.1f"
          % (nsens, k, lam, e.mean(), e.max(), err.mean(), np.percentile(err, 95), err.max(), p[W]))
    print("      largest heat in at %s ; largest heat out at %s" % (fmt(top_hot), fmt(top_cold)))
print("truth: heater x=%.1f y=%.1f..%.1f ; window y=%.1f x=%.1f..%.1f ; k_true=%.3f"
      % (tf["heater"][0][0], tf["heater"][0][1], tf["heater"][1][1], tf["window"][0][1], tf["window"][0][0], tf["window"][1][0], 1 / 1.2 ** 2))

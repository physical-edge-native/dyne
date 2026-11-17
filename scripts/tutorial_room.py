#!/usr/bin/env python3
"""Helpers of the room tutorial (docs/en/tutorial-room.md, GitHub issue #96).

The tutorial runs Dyne's simulator on a dummy room; this script does the PC-side steps around it.
Every subcommand reads or writes plain files; none of them computes the temperature shown in the
tutorial (that is Dyne's heat model, run by dyne-sim2d).

  readings      write the dummy temperature readings (the committed file
                crates/dyne-sim2d/scenes/room6_temp.csv was made with this)
  coarse-shape  take the room shape (.vti of the shape grid actor) at the points of the heat grid,
                so that the OpenFOAM export can use it for walls (both grids share their points)
  check         compare Dyne's temperature field with the readings (pass/fail) and with the made-up
                field the readings were sampled from (reported only)
  figures       draw the tutorial's pictures (PNG) with matplotlib
  screenshot    take a picture of the 3D dashboard (HTML) with a headless Edge or Chrome

What is real and what is dummy:
  - The made-up temperature field ("truth") is scripts/twin_demo.py's dummy_temperature: a steady heat
    solve with a heater on the west wall (30 C), a window on the north wall (12 C) and 20 C far away.
    It exists only to produce the dummy readings and to measure how far Dyne's field is from it.
  - The readings are that field at a few points, plus a small fixed error (normal, sigma 0.1 C, seed 96),
    rounded to 0.01 C, as a real thermometer would report them.
  - The temperature field in the tutorial's outputs is Dyne's: the 2D heat actor (crates/heat2d-actor)
    split over the simulated devices, nudged towards the readings (dyne-sim2d --temp-sensors).

Needs numpy and matplotlib (the repository's .venv). Usage: python scripts/tutorial_room.py SUBCOMMAND -h
"""
import argparse
import csv
import math
import os
import re
import shutil
import struct
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import twin_demo  # noqa: E402

# Thermometers that are not on a device with a distance sensor: wall-mounted ones where the made-up
# field changes most (near the heater and the window) and one in the far corner. (id, x, y)
WALL_THERMOMETERS = [("W1", 0.10, 1.30), ("W2", 2.10, 2.60), ("W3", 3.50, 2.30)]

# Tolerance of the check, written before the first run (CLAUDE.md rule 6). See docs/en/tutorial-room.md.
#   At the reading points the nudged model must stay within 0.5 C of the reading: the accuracy class of
#   common room thermometers (for example +-0.5 C), so a model further away than that would contradict
#   the readings it was given.
READING_TOL_C = 0.5


# ---------------------------------------------------------------- made-up field


def truth_field(sc):
    """The made-up steady field of scripts/twin_demo.py (same heater, window and constants)."""
    xs = [p[0] for p in sc["room"]]
    ys = [p[1] for p in sc["room"]]
    b = (min(xs), min(ys), max(xs), max(ys))
    heater = ((b[0], b[1] + 0.8), (b[0], b[1] + 1.8))
    window = ((b[0] + 1.3, b[3]), (b[0] + 2.9, b[3]))
    tf = twin_demo.dummy_temperature(sc, b, 0.05, heater, window, 30.0, 12.0, 20.0, 1.2)
    tf["heater"], tf["window"], tf["bounds"] = heater, window, b
    return tf


def thermometer_points(sc):
    """One thermometer on every device position of the scene (distance sensors share a device when they
    share a position), then the wall-mounted ones."""
    pts, seen = [], set()
    for s in sc["sensors"]:
        key = (s["x"], s["y"])
        if key in seen:
            continue
        seen.add(key)
        pts.append(("D%d" % (len(pts) + 1), s["x"], s["y"]))
    return pts + WALL_THERMOMETERS


def cmd_readings(a):
    sc = twin_demo.parse_scene(a.scene)
    tf = truth_field(sc)
    rng = np.random.default_rng(a.seed)
    rows = []
    for (tid, x, y) in thermometer_points(sc):
        t = twin_demo.sample_temp(tf, x, y)
        err = float(rng.normal(0.0, a.sigma)) if a.sigma > 0 else 0.0
        rows.append((tid, x, y, round(t + err, 2), round(t, 3)))
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        f.write("# Dummy temperature readings for the room tutorial (docs/en/tutorial-room.md, issue #96).\n")
        f.write("# Made by: python scripts/tutorial_room.py readings --scene %s --sigma %g --seed %d\n"
                % (a.scene.replace("\\", "/"), a.sigma, a.seed))
        f.write("# T = made-up field (scripts/twin_demo.py dummy_temperature) + normal error, rounded to 0.01 C.\n")
        f.write("# truth = the made-up field at the point (for the comparison only; Dyne reads id, x, y, T).\n")
        f.write("id,x,y,T,truth\n")
        for r in rows:
            f.write("%s,%s,%s,%.2f,%.3f\n" % r)
    print("readings: %d thermometers -> %s" % (len(rows), a.out))


def read_readings(path):
    with open(path, encoding="utf-8") as f:
        lines = [l for l in f if l.strip() and not l.startswith("#")]
    return [dict(r) for r in csv.DictReader(lines)]


# ---------------------------------------------------------------- .vti (layout of crates/dyne-export/src/vti.rs)


def read_vti(path):
    b = open(path, "rb").read()
    start = b.index(b'<AppendedData encoding="raw">')
    head = b[:start].decode("utf-8")
    base = b.index(b"_", start) + 1
    ext = [int(v) for v in re.search(r'WholeExtent="([^"]+)"', head).group(1).split()]
    origin = [float(v) for v in re.search(r'Origin="([^"]+)"', head).group(1).split()]
    spacing = [float(v) for v in re.search(r'Spacing="([^"]+)"', head).group(1).split()]
    pts = (ext[1] + 1, ext[3] + 1, ext[5] + 1)
    n = pts[0] * pts[1] * pts[2]
    arrays = {}
    for name, off in re.findall(r'Name="([^"]+)" NumberOfComponents="1" format="appended" offset="(\d+)"', head):
        o = base + int(off)
        size = struct.unpack_from("<Q", b, o)[0]
        assert size == 8 * n, (name, size, n)
        arrays[name] = np.frombuffer(b, "<f8", n, o + 8).copy()
    order = re.findall(r'Name="([^"]+)" NumberOfComponents', head)
    return {"points": pts, "origin": origin, "spacing": spacing, "arrays": arrays, "order": order}


def rust_float(v):
    """Rust's Debug form of f64 for the numbers written here (finite, not tiny or huge)."""
    r = repr(float(v))
    return r


def write_vti(path, pts, origin, spacing, arrays):
    """Writes exactly the layout of dyne_export::vti::ImageData::to_vti_bytes (2D: nz = 1)."""
    ext = "0 %d 0 %d 0 %d" % (pts[0] - 1, pts[1] - 1, pts[2] - 1)
    tri = lambda v: " ".join(rust_float(x) for x in v)  # noqa: E731
    x = ['<?xml version="1.0"?>\n',
         '<VTKFile type="ImageData" version="1.0" byte_order="LittleEndian" header_type="UInt64">\n',
         '  <ImageData WholeExtent="%s" Origin="%s" Spacing="%s">\n' % (ext, tri(origin), tri(spacing)),
         '    <Piece Extent="%s">\n' % ext,
         '      <PointData Scalars="%s">\n' % arrays[0][0]]
    off = 0
    for name, v in arrays:
        x.append('        <DataArray type="Float64" Name="%s" NumberOfComponents="1" format="appended" offset="%d"/>\n'
                 % (name, off))
        off += 8 + 8 * len(v)
    x.append("      </PointData>\n      <CellData>\n      </CellData>\n    </Piece>\n  </ImageData>\n")
    x.append('  <AppendedData encoding="raw">\n   _')
    out = bytearray("".join(x).encode("utf-8"))
    for name, v in arrays:
        v = np.ascontiguousarray(v, "<f8")
        out += struct.pack("<Q", 8 * len(v)) + v.tobytes()
    out += b"\n  </AppendedData>\n</VTKFile>\n"
    with open(path, "wb") as f:
        f.write(out)


def cmd_coarse_shape(a):
    s = read_vti(a.shape_vti)
    t = read_vti(a.heat_vti)
    ratio = t["spacing"][0] / s["spacing"][0]
    k = int(round(ratio))
    ok = (abs(ratio - k) < 1e-9 and abs(t["spacing"][1] / s["spacing"][1] - k) < 1e-9
          and all(abs(t["origin"][i] - s["origin"][i]) < 1e-9 for i in range(2))
          and (t["points"][0] - 1) * k <= s["points"][0] - 1 and (t["points"][1] - 1) * k <= s["points"][1] - 1)
    if not ok:
        sys.exit("coarse-shape: the heat grid's points are not points of the shape grid "
                 "(origin %s / %s, spacing %s / %s)" % (t["origin"], s["origin"], t["spacing"], s["spacing"]))
    sx, sy = s["points"][0], s["points"][1]
    tx, ty = t["points"][0], t["points"][1]
    arrays = []
    for name in ("phi", "w"):
        g = s["arrays"][name].reshape(sy, sx)
        arrays.append((name, g[0:(ty - 1) * k + 1:k, 0:(tx - 1) * k + 1:k].reshape(-1)))
    write_vti(a.out, (tx, ty, 1), t["origin"], t["spacing"], arrays)
    print("coarse-shape: %dx%d shape points, every %d-th -> %dx%d (the heat grid) -> %s" % (sx, sy, k, tx, ty, a.out))


# ---------------------------------------------------------------- check


def field_at(v, x, y):
    """Value of a 2D .vti field at the grid point nearest to (x, y) (the point the simulator nudges)."""
    nx, ny = v["points"][0], v["points"][1]
    i = int(math.floor((x - v["origin"][0]) / v["spacing"][0] + 0.5))
    j = int(math.floor((y - v["origin"][1]) / v["spacing"][1] + 0.5))
    return float(v["arrays"]["u"][j * nx + i])


def compare(a):
    sc = twin_demo.parse_scene(a.scene)
    v = read_vti(a.heat_vti)
    rd = read_readings(a.readings)
    res = [abs(float(r["T"]) - field_at(v, float(r["x"]), float(r["y"]))) for r in rd]
    tf = truth_field(sc)
    nx, ny = v["points"][0], v["points"][1]
    u = v["arrays"]["u"].reshape(ny, nx)
    errs = []
    for j in range(ny):
        for i in range(nx):
            x = v["origin"][0] + i * v["spacing"][0]
            y = v["origin"][1] + j * v["spacing"][1]
            px = min(max(x, tf["x0"] + 1e-6), tf["x0"] + (tf["nx"] - 1) * tf["step"] - 1e-6)
            py = min(max(y, tf["y0"] + 1e-6), tf["y0"] + (tf["ny"] - 1) * tf["step"] - 1e-6)
            if twin_demo.is_free(sc, px, py):
                errs.append(abs(u[j, i] - twin_demo.sample_temp(tf, x, y)))
    return np.array(res), np.array(errs), u


def cmd_check(a):
    res, errs, u = compare(a)
    print("check: Dyne field %.2f..%.2f C" % (u.min(), u.max()))
    print("check: at the %d reading points |reading - Dyne| mean %.3f C, max %.3f C (tolerance %.1f C)"
          % (len(res), res.mean(), res.max(), READING_TOL_C))
    print("check: over the free floor (%d grid points) |Dyne - made-up field| mean %.2f C, 95%% %.2f C, max %.2f C "
          "(reported, not a pass criterion)" % (len(errs), errs.mean(), np.percentile(errs, 95), errs.max()))
    ok = bool(res.max() <= READING_TOL_C) and np.isfinite(u).all()
    print("check: %s" % ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


# ---------------------------------------------------------------- figures


def cmd_figures(a):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import TwoSlopeNorm  # noqa: F401
    os.makedirs(a.out_dir, exist_ok=True)
    sc = twin_demo.parse_scene(a.scene)
    tf = truth_field(sc)
    rd = read_readings(a.readings)
    b = tf["bounds"]
    plt.rcParams.update({"font.size": 9, "axes.titlesize": 10, "figure.dpi": 100})
    ink, muted, grid = "#333333", "#777777", "#d0d0d0"

    def room(ax, partition=None):
        ring = sc["room"] + [sc["room"][0]]
        ax.plot([p[0] for p in ring], [p[1] for p in ring], color=ink, lw=2)
        for k, p in enumerate(sc["polys"]):
            q = p + [p[0]]
            ax.fill([r[0] for r in q], [r[1] for r in q], color="#e8d9c4", ec="#8a6d45", lw=1, zorder=1)
            cx = sum(r[0] for r in p) / len(p)
            cy = sum(r[1] for r in p) / len(p)
            # the scene file names furniture in Japanese; the pictures are in English
            name = {"机": "desk", "ベッド": "bed", "本棚": "bookshelf"}.get(sc["names"][k] or "", "furniture")
            ax.text(cx, cy, name, ha="center", va="center", fontsize=7, color="#6b5332")
        ax.set_aspect("equal")
        ax.set_xlim(b[0] - 0.15, b[2] + 0.15)
        ax.set_ylim(b[1] - 0.15, b[3] + 0.15)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")

    def blocks(ax, nx, ny, h, x0, y0, px, py):
        def split(n, p):
            base, extra, out, i0 = n // p, n % p, [], 0
            for i in range(p):
                c = base + (1 if i < extra else 0)
                out.append((i0, c))
                i0 += c
            return out
        for (i0, c) in split(nx, px)[1:]:
            xx = x0 + (i0 - 0.5) * h
            ax.plot([xx, xx], [b[1], b[3]], color=muted, lw=0.6, ls="--", zorder=2)
        for (j0, c) in split(ny, py)[1:]:
            yy = y0 + (j0 - 0.5) * h
            ax.plot([b[0], b[2]], [yy, yy], color=muted, lw=0.6, ls="--", zorder=2)

    # 1. the dummy room: devices, distance sensors, thermometers, the split into 32 regions
    fig, ax = plt.subplots(figsize=(6.4, 5.0))
    room(ax)
    hx = 0.025
    nxs, nys = int(round((b[2] - b[0]) / hx)) + 1, int(round((b[3] - b[1]) / hx)) + 1
    blocks(ax, nxs, nys, hx, b[0], b[1], a.px, a.py)
    for s in sc["sensors"]:
        ax.plot([s["x"], s["x"] + 0.28 * math.cos(s["yaw"])], [s["y"], s["y"] + 0.28 * math.sin(s["yaw"])],
                color="#4a7fd4", lw=1, zorder=3)
    dev = [r for r in rd if r["id"].startswith("D")]
    wall = [r for r in rd if not r["id"].startswith("D")]
    ax.scatter([float(r["x"]) for r in dev], [float(r["y"]) for r in dev], s=40, color="#2e9e57", zorder=4,
               label="device: distance sensors + thermometer")
    ax.scatter([float(r["x"]) for r in wall], [float(r["y"]) for r in wall], s=40, marker="s", color="#d9822b",
               zorder=4, label="wall thermometer")
    ax.plot([], [], color="#4a7fd4", lw=1, label="direction of a distance sensor")
    ax.plot([], [], color=muted, lw=0.6, ls="--", label="regions of the 32 simulated devices")
    (hx0, hy0), (hx1, hy1) = tf["heater"]
    ax.plot([hx0, hx1], [hy0, hy1], color="#c0392b", lw=5, solid_capstyle="butt", label="heater (dummy, 30 C)")
    (wx0, wy0), (wx1, wy1) = tf["window"]
    ax.plot([wx0, wx1], [wy0, wy1], color="#2c7fb8", lw=5, solid_capstyle="butt", label="window (dummy, 12 C)")
    ax.set_title("Step 1: the dummy room (3.6 m x 2.7 m)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=2, fontsize=7, frameon=False)
    fig.tight_layout()
    save(fig, os.path.join(a.out_dir, "01-room.png"))

    # 2. the shape Dyne built
    g = twin_demo.read_shape(a.shape_csv)
    fig, ax = plt.subplots(figsize=(6.4, 4.6))
    xs = g["x0"] + g["h"] * np.arange(g["nx"])
    ys = g["y0"] + g["h"] * np.arange(g["ny"])
    unobs = np.where(g["w"] <= 0, 1.0, np.nan)
    ax.pcolormesh(xs, ys, unobs, cmap="Greys", vmin=0, vmax=3, shading="nearest", zorder=0)
    ring = sc["room"] + [sc["room"][0]]
    ax.plot([p[0] for p in ring], [p[1] for p in ring], color="#9ecae1", lw=4, zorder=1)
    for p in sc["polys"]:
        q = p + [p[0]]
        ax.plot([r[0] for r in q], [r[1] for r in q], color="#9ecae1", lw=4, zorder=1)
    phi = np.where(g["w"] > 0, g["phi"], np.nan)
    ax.contour(xs, ys, phi, levels=[0.0], colors=["#c0392b"], linewidths=1.2, zorder=2)
    ax.plot([], [], color="#9ecae1", lw=4, label="true walls and furniture (not given to Dyne)")
    ax.plot([], [], color="#c0392b", lw=1.2, label="surface computed by Dyne (zero of the signed distance)")
    ax.fill([], [], color="#d9d9d9", label="never seen by a distance sensor")
    ax.set_aspect("equal")
    ax.set_xlim(b[0] - 0.15, b[2] + 0.15)
    ax.set_ylim(b[1] - 0.15, b[3] + 0.15)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Step 2a: room shape from the simulated distance sensors")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=1, fontsize=7, frameon=False)
    fig.tight_layout()
    save(fig, os.path.join(a.out_dir, "02-shape.png"))

    # 3. temperature: made-up field (source of the readings), Dyne's field, difference
    v = read_vti(a.heat_vti)
    nx, ny = v["points"][0], v["points"][1]
    u = v["arrays"]["u"].reshape(ny, nx)
    ux = v["origin"][0] + v["spacing"][0] * np.arange(nx)
    uy = v["origin"][1] + v["spacing"][1] * np.arange(ny)
    tx = tf["x0"] + tf["step"] * np.arange(tf["nx"])
    ty = tf["y0"] + tf["step"] * np.arange(tf["ny"])
    T = tf["T"]
    lo, hi = math.floor(min(T.min(), u.min())), math.ceil(max(T.max(), u.max()))
    fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.7), layout="constrained")
    for ax, (X, Y, F, title) in zip(axs, ((tx, ty, T, "made-up field (only makes the readings)"),
                                          (ux, uy, u, "Dyne: 32 devices, nudged to the readings"))):
        m = ax.pcolormesh(X, Y, F, cmap="RdBu_r", vmin=lo, vmax=hi, shading="nearest")
        ring = sc["room"] + [sc["room"][0]]
        ax.plot([p[0] for p in ring], [p[1] for p in ring], color=ink, lw=1.5)
        for p in sc["polys"]:
            q = p + [p[0]]
            ax.plot([r[0] for r in q], [r[1] for r in q], color=ink, lw=0.8, ls=":")
        ax.scatter([float(r["x"]) for r in rd], [float(r["y"]) for r in rd], s=14, facecolor="white",
                   edgecolor=ink, lw=0.8, zorder=3)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("x [m]")
        ax.set_xlim(b[0] - 0.05, b[2] + 0.05)
        ax.set_ylim(b[1] - 0.05, b[3] + 0.05)
    axs[0].set_ylabel("y [m]")
    cb = fig.colorbar(m, ax=axs, shrink=0.9)
    cb.set_label("temperature [C]")
    fig.suptitle("Step 2b: temperature (white dots: thermometers)", fontsize=10)
    save(fig, os.path.join(a.out_dir, "03-temperature.png"))

    # 5. OpenFOAM result (optional; only when the case was run)
    if a.foam_case and os.path.isdir(a.foam_case):
        times = [d for d in os.listdir(a.foam_case) if re.fullmatch(r"[0-9.eE+-]+", d) and d != "0"]
        cfile = os.path.join(a.foam_case, "0", "C")
        if times and os.path.exists(cfile):
            last = max(times, key=float)
            T1 = foam_scalar(os.path.join(a.foam_case, last, "T"))
            T0 = foam_scalar(os.path.join(a.foam_case, "0", "T"))
            C = foam_vectors(cfile)
            fig, axs = plt.subplots(1, 2, figsize=(9.6, 3.7), layout="constrained")
            for ax, (F, title) in zip(axs, ((T0, "OpenFOAM, t = 0: Dyne's field handed over"),
                                            (T1, "OpenFOAM, t = %s s: scalarTransportFoam" % last))):
                # cell centres are Dyne's grid points: put each cell back on the grid (solid cells stay empty)
                G = np.full((ny, nx), np.nan)
                gi = np.rint((C[:, 0] - v["origin"][0]) / v["spacing"][0]).astype(int)
                gj = np.rint((C[:, 1] - v["origin"][1]) / v["spacing"][1]).astype(int)
                G[gj, gi] = F
                m = ax.pcolormesh(ux, uy, G, cmap="RdBu_r", vmin=lo, vmax=hi, shading="nearest")
                ring = sc["room"] + [sc["room"][0]]
                ax.plot([p[0] for p in ring], [p[1] for p in ring], color=ink, lw=1.5)
                ax.set_aspect("equal")
                ax.set_title(title, fontsize=9)
                ax.set_xlabel("x [m]")
                ax.set_xlim(b[0] - 0.05, b[2] + 0.05)
                ax.set_ylim(b[1] - 0.05, b[3] + 0.05)
            axs[0].set_ylabel("y [m]")
            cb = fig.colorbar(m, ax=axs, shrink=0.9)
            cb.set_label("temperature [C]")
            fig.suptitle("Step 4 (optional): the case run by OpenFOAM, read back (%d cells)" % len(T1), fontsize=10)
            save(fig, os.path.join(a.out_dir, "05-openfoam.png"))
        else:
            print("figures: %s has no result or no cell centres (0/C); skipped the OpenFOAM picture" % a.foam_case)


def save(fig, path):
    import matplotlib.pyplot as plt
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print("figure: %s (%d kB)" % (path, os.path.getsize(path) // 1024))


def _foam_list(path):
    txt = open(path, encoding="utf-8", errors="replace").read()
    m = re.search(r"internalField\s+nonuniform\s+List<(\w+)>\s*(\d+)\s*\(", txt)
    if not m:
        m2 = re.search(r"internalField\s+uniform\s+([^;]+);", txt)
        raise ValueError("%s: no nonuniform internalField (%s)" % (path, m2.group(1) if m2 else "?"))
    n = int(m.group(2))
    body = txt[m.end():]
    return m.group(1), n, body


def foam_scalar(path):
    kind, n, body = _foam_list(path)
    vals = np.array(body[:body.index(")")].split(), float)
    assert len(vals) == n
    return vals


def foam_vectors(path):
    kind, n, body = _foam_list(path)
    vecs = re.findall(r"\(\s*([-0-9.eE+]+)\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)\s*\)", body)[:n]
    assert len(vecs) == n
    return np.array(vecs, float)


# ---------------------------------------------------------------- screenshot


def find_browser():
    cands = [os.environ.get("DYNE_BROWSER", ""),
             r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
             r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
             r"C:\Program Files\Google\Chrome\Application\chrome.exe",
             "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"]
    for n in ("chromium", "chromium-browser", "google-chrome", "microsoft-edge", "msedge", "chrome"):
        p = shutil.which(n)
        if p:
            cands.append(p)
    for c in cands:
        if c and os.path.exists(c):
            return c
    return None


def cmd_screenshot(a):
    br = find_browser()
    if not br:
        print("screenshot: no Edge or Chrome found (set DYNE_BROWSER); skipped")
        return
    out = os.path.abspath(a.out)
    url = "file:///" + os.path.abspath(a.html).replace("\\", "/").lstrip("/")
    prof = os.path.abspath(os.path.join(os.path.dirname(out), ".browser-profile"))
    cmd = [br, "--headless=new", "--disable-gpu-sandbox", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
           "--hide-scrollbars", "--window-size=%s" % a.size, "--virtual-time-budget=%d" % a.wait_ms,
           "--user-data-dir=%s" % prof, "--screenshot=%s" % out, url]
    try:
        subprocess.run(cmd, check=True, timeout=120, capture_output=True)
    except (subprocess.SubprocessError, OSError) as e:
        print("screenshot: the browser failed (%s); skipped" % e)
        return
    shutil.rmtree(prof, ignore_errors=True)
    if a.max_kb and os.path.exists(out):
        shrink_png(out, a.max_kb)
    print("screenshot: %s (%d kB)" % (out, os.path.getsize(out) // 1024))


def shrink_png(path, max_kb):
    """Reduce a PNG to at most max_kb by scaling it down (matplotlib's image reader and writer)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    img = plt.imread(path)
    scale = 1.0
    while os.path.getsize(path) > max_kb * 1024 and scale > 0.3:
        scale *= 0.85
        h, w = img.shape[0], img.shape[1]
        ys = (np.arange(int(h * scale)) / scale).astype(int)
        xs = (np.arange(int(w * scale)) / scale).astype(int)
        plt.imsave(path, img[ys][:, xs])


# ---------------------------------------------------------------- main


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("readings", help="write the dummy temperature readings")
    p.add_argument("--scene", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--sigma", type=float, default=0.1, help="error of the dummy thermometers [C] (default 0.1)")
    p.add_argument("--seed", type=int, default=96)
    p.set_defaults(fn=cmd_readings)
    p = sub.add_parser("coarse-shape", help="the room shape at the points of the heat grid")
    p.add_argument("--shape-vti", required=True)
    p.add_argument("--heat-vti", required=True)
    p.add_argument("--out", required=True)
    p.set_defaults(fn=cmd_coarse_shape)
    p = sub.add_parser("check", help="compare Dyne's field with the readings and the made-up field")
    p.add_argument("--scene", required=True)
    p.add_argument("--heat-vti", required=True)
    p.add_argument("--readings", required=True)
    p.set_defaults(fn=cmd_check)
    p = sub.add_parser("figures", help="draw the tutorial's pictures")
    p.add_argument("--scene", required=True)
    p.add_argument("--readings", required=True)
    p.add_argument("--shape-csv", required=True)
    p.add_argument("--heat-vti", required=True)
    p.add_argument("--foam-case", default=None)
    p.add_argument("--px", type=int, default=8)
    p.add_argument("--py", type=int, default=4)
    p.add_argument("--out-dir", required=True)
    p.set_defaults(fn=cmd_figures)
    p = sub.add_parser("screenshot", help="picture of the 3D dashboard with a headless browser")
    p.add_argument("--html", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--size", default="1400,860")
    p.add_argument("--wait-ms", type=int, default=8000)
    p.add_argument("--max-kb", type=int, default=290)
    p.set_defaults(fn=cmd_screenshot)
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()

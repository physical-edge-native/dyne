"""How the room twin grows over time (GitHub issue #157).

Reads the outputs of dyne-sim2d's shape grid solver run with

    --growth-csv growth.csv --snapshot-every N --snapshot-dir snaps/

and writes

    growth.png      coverage of the surfaces and the free grid points the distance has not reached yet, per step;
                    a second axis gives the time on hardware at the measured 0.7-1.0 s per step (an estimate)
    snapshots.png   a few snapshots side by side (distance field, estimated surface, true walls)
    growth.gif      all snapshots as a short animation (only with --gif)

and prints the steps at which the coverage first reaches 50 % and 80 % of its final value.

Why the twin grows step by step: every grid point is updated from its neighbours' values of the previous step
(the step before, also across the border between two devices), so a distance travels one grid cell per step.
This needs only one exchange between neighbouring devices per step, and the split computation equals the
single-device one bit for bit.

The surface error in growth.csv counts surface points whose distance has not been computed yet, so early values can
be very large; it is the same measure as the final "shape:" lines of dyne-sim2d. The figures do not plot it.

Usage:
    python scripts/twin_growth.py --growth growth.csv --snapshots snaps/ --out outdir/ [--gif] [--panels 8,16,32,64]
    python scripts/twin_growth.py --growth growth.csv --summary-only      (no matplotlib needed)
"""

import argparse
import csv
import glob
import os
import re
import sys

HW_STEP_S = (0.7, 1.0)  # measured seconds per step of the shape solver on an ESP32 (docs-site/status.md)


def read_growth(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: no rows")
    return [{k: float(v) for k, v in r.items()} for r in rows]


def first_reach(rows, frac):
    final = rows[-1]["coverage_pct"]
    for r in rows:
        if r["coverage_pct"] >= frac * final:
            return int(r["steps_done"])
    return None


def summary(rows):
    final = rows[-1]
    lines = [f"final after {int(final['steps_done'])} steps: coverage {final['coverage_pct']:.1f} % "
             f"({int(final['covered'])} / {int(final['surface_samples'])} surface samples), "
             f"no distance yet {int(final['no_distance_yet'])} of {int(final['free_points'])} free points"]
    for frac in (0.5, 0.8):
        s = first_reach(rows, frac)
        lo, hi = s * HW_STEP_S[0], s * HW_STEP_S[1]
        lines.append(f"{int(frac * 100)} % of final coverage at step {s} "
                     f"(on hardware about {lo:.0f}-{hi:.0f} s at {HW_STEP_S[0]}-{HW_STEP_S[1]} s per step, estimate)")
    reached = next((int(r["steps_done"]) for r in rows if r["no_distance_yet"] == 0), None)
    lines.append(f"distance reached every free point at step {reached}")
    return "\n".join(lines)


def read_snapshot(path):
    import numpy as np
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    nx = max(int(r["gx"]) for r in rows) + 1
    ny = max(int(r["gy"]) for r in rows) + 1
    def grid(key):
        a = np.zeros((ny, nx))
        for r in rows:
            a[int(r["gy"]), int(r["gx"])] = float(r[key])
        return a
    x = sorted({float(r["x"]) for r in rows})
    y = sorted({float(r["y"]) for r in rows})
    return np.array(x), np.array(y), grid("phi"), grid("w"), grid("truth")


def draw_snapshot(ax, path, step):
    import numpy as np
    x, y, phi, w, truth = read_snapshot(path)
    shown = np.where(phi >= 1e29, np.nan, phi)
    ext = (x[0], x[-1], y[0], y[-1])
    ax.imshow(np.clip(shown, -0.3, 1.2), origin="lower", extent=ext, cmap="viridis", vmin=-0.3, vmax=1.2)
    ax.imshow(np.where(phi >= 1e29, 1.0, np.nan), origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=2, alpha=0.6)
    ax.contour(x, y, truth, levels=[0.0], colors="white", linewidths=0.8)
    est = np.where((w > 0) & (np.abs(phi) < 0.2), phi, np.nan)
    if np.isfinite(est).any():
        ax.contour(x, y, est, levels=[0.0], colors="red", linewidths=1.2)
    ax.set_title(f"step {step}", fontsize=9)
    ax.set_xticks([]); ax.set_yticks([]); ax.set_aspect("equal")


def snapshot_files(d):
    out = []
    for p in glob.glob(os.path.join(d, "shape_step*.csv")):
        m = re.search(r"shape_step(\d+)\.csv$", p)
        if m:
            out.append((int(m.group(1)), p))
    return sorted(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--growth", required=True)
    ap.add_argument("--snapshots")
    ap.add_argument("--out", default=".")
    ap.add_argument("--panels", default="", help="steps to show side by side (default: four evenly spaced snapshots)")
    ap.add_argument("--gif", action="store_true")
    ap.add_argument("--summary-only", action="store_true")
    a = ap.parse_args()

    rows = read_growth(a.growth)
    print(summary(rows))
    if a.summary_only:
        return

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(a.out, exist_ok=True)
    st = [r["steps_done"] for r in rows]
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    ax.plot(st, [r["coverage_pct"] for r in rows], color="tab:blue", label="surfaces covered (%)")
    ax.set_xlabel("step")
    ax.set_ylabel("surfaces covered (%)", color="tab:blue")
    ax.set_ylim(0, 100)
    ax2 = ax.twinx()
    ax2.plot(st, [100.0 * r["no_distance_yet"] / r["free_points"] for r in rows], color="tab:orange",
             label="free points without a distance yet (%)")
    ax2.set_ylabel("free points without a distance yet (%)", color="tab:orange")
    ax2.set_ylim(0, 100)
    top = ax.secondary_xaxis("top", functions=(lambda s: s * HW_STEP_S[1], lambda t: t / HW_STEP_S[1]))
    top.set_xlabel(f"time on hardware at {HW_STEP_S[1]} s per step (s, estimate)")
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "growth.png"), dpi=120)
    plt.close(fig)

    if a.snapshots:
        snaps = snapshot_files(a.snapshots)
        if not snaps:
            raise SystemExit(f"{a.snapshots}: no shape_step*.csv")
        if a.panels:
            want = [int(v) for v in a.panels.split(",")]
            pick = [(s, p) for s, p in snaps if s in want]
        else:
            idx = sorted({round(i * (len(snaps) - 1) / 3) for i in range(4)})
            pick = [snaps[i] for i in idx]
        fig, axs = plt.subplots(1, len(pick), figsize=(2.6 * len(pick), 2.4))
        for ax, (s, p) in zip(axs if len(pick) > 1 else [axs], pick):
            draw_snapshot(ax, p, s)
        fig.suptitle("distance field (colour), not reached yet (grey), estimated surface (red), true walls (white)",
                     fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, "snapshots.png"), dpi=110)
        plt.close(fig)

        if a.gif:
            from PIL import Image
            frames = []
            for s, p in snaps:
                fig, ax = plt.subplots(figsize=(3.2, 2.6))
                draw_snapshot(ax, p, s)
                fig.tight_layout()
                fig.canvas.draw()
                frames.append(Image.frombuffer("RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba()).convert("P"))
                plt.close(fig)
            frames[0].save(os.path.join(a.out, "growth.gif"), save_all=True, append_images=frames[1:], duration=250, loop=0)
    print(f"written to {a.out}")


if __name__ == "__main__":
    sys.exit(main())

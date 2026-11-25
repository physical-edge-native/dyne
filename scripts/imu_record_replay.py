"""Replay the 2026-11-21 IMU hardware record through the IMU fusion solver (GitHub issue #158).

On 2026-11-21 seven devices with accelerometer/gyroscope modules recorded on one board; the result was analysed on
a PC, and the fusion solver (`crates/imu-fusion-actor`, neighbour-only averaging) did not run on the devices.
This script feeds the recorded samples (measured/2026-11-21-siteA-imu/run2/imu_raw.csv, synchronised timestamps)
to seven instances of the solver connected in a ring, in the simulator, and checks:

1. The WebAssembly solver and its Python reference model (scripts/imu_fusion_ref.py) give the same bits for every
   output and every message (bit-exact).
2. Each device's fused mean for each time window equals the mean of the seven devices' window means computed in one
   place (same window assignment), within a tolerance of 1e-6 (m/s^2 and rad/s). Reason: neighbour-only averaging
   repeats the averaging M times per window; the remaining difference shrinks by the second-largest eigenvalue of
   the averaging matrix per repetition (printed below; about 0.75 for a ring of seven), times the spread between
   devices (about 0.1 m/s^2), so about 1e-7 for M = 50. Bit-exact equality with the one-place mean is not expected.
3. The CRC-32 of all solver outputs is the pinned value (detects any change in the solver or in this replay).

It also reports, without asserting, the noise of the gyroscope means over time: each device alone, the solver's fused
output as it is, and the one-place mean after removing each device's zero offset over windows where all seven devices
have samples. The last one is what the PC analysis did (docs-site/results/imu-2026-11-21.md).

Usage (repository root; needs numpy and wasmtime, and the WebAssembly solver built):
    (cd crates/imu-fusion-actor && cargo build --release --target wasm32-unknown-unknown)
    python scripts/imu_record_replay.py [--seconds 120] [--check] [--json out.json]
"""

import argparse
import csv
import json
import math
import os
import struct
import sys
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import imu_fusion_ref as R  # noqa: E402

RECORD = os.path.join(R.ROOT, "measured", "2026-11-21-siteA-imu", "run2", "imu_raw.csv")
DT = 0.02         # one averaging repetition (50 Hz), as in scripts/imu_sim.py
M = 50            # repetitions per window: windows of 1 s, as in the PC analysis
TOL = 1e-6        # fused mean vs one-place mean (see the module docstring)
PINNED_CRC = {120.0: 0xE2C60E10}  # CRC-32 of all WebAssembly outputs, per --seconds (issue #158)


def load_pushes(path, seconds):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    nodes = sorted({int(r["node"]) for r in rows})
    e_us = int(DT * 1e6 + 0.5) * M
    t0 = (min(int(r["ts_us"]) for r in rows) // e_us + 1) * e_us  # first window boundary after the first sample
    steps = int(round(seconds / DT))
    pushes = [[] for _ in nodes]
    for r in rows:
        ts = int(r["ts_us"]) - t0 + e_us  # window 1 starts at t0
        if ts <= e_us:
            continue
        e = (ts - 1) // e_us
        close = (e + 1) * M - 1  # the sample is pushed before the window closes (timestamp mode)
        if close >= steps:
            continue
        pushes[nodes.index(int(r["node"]))].append(
            (close, ts, tuple(float(r[k]) for k in ("ax", "ay", "az", "gx", "gy", "gz"))))
    for p in pushes:
        p.sort(key=lambda x: (x[0], x[1]))
    return nodes, pushes, steps, e_us


def one_place_means(pushes, n_win, e_us):
    n = len(pushes)
    s = np.zeros((n, n_win, 6))
    c = np.zeros((n, n_win))
    for i, p in enumerate(pushes):
        for _, ts, v in p:
            e = (ts - 1) // e_us
            if e < n_win:
                s[i, e] += v
                c[i, e] += 1
    node_means = s / np.where(c > 0, c, 1)[..., None]
    have = c > 0
    central = np.full((n_win, 6), np.nan)
    for e in range(n_win):
        k = have[:, e]
        if k.any():
            central[e] = node_means[k, e].mean(axis=0)
    return node_means, have, central


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", default=RECORD)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--check", action="store_true", help="exit 1 unless every check passes")
    ap.add_argument("--json")
    a = ap.parse_args()

    nodes, pushes, steps, e_us = load_pushes(a.record, a.seconds)
    n = len(nodes)
    links = R.topology("ring", n)

    def cfg(i):
        return {R.CFG_EPOCH_STEPS: M, R.CFG_TIME_MODE: R.TIME_STAMP}

    w = R.metropolis_matrix(n, links)
    lam2 = sorted(np.abs(np.linalg.eigvals(np.array(w, dtype=float))))[-2]
    r_ref, m_ref = R.run_network("ref", n, links, cfg, DT, steps, pushes, record_every=M, check_msgs=steps)
    r_wasm, m_wasm = R.run_network("wasm", n, links, cfg, DT, steps, pushes, record_every=M, check_msgs=steps)

    total = mismatched = 0
    crc = 0
    for s in sorted(r_wasm):
        for a_vals, b_vals in zip(r_ref[s], r_wasm[s]):
            for x, y in zip(a_vals, b_vals):
                total += 1
                bx, by = struct.pack("<d", x), struct.pack("<d", y)
                mismatched += bx != by
                crc = zlib.crc32(by, crc)
    crc &= 0xFFFFFFFF
    msgs_same = m_ref == m_wasm

    n_win = steps // M
    node_means, have, central = one_place_means(pushes, n_win, e_us)
    diffs = []
    fused = {i: {} for i in range(n)}
    local = {i: {} for i in range(n)}
    for s, vals in r_wasm.items():
        for i, v in enumerate(vals):
            e = int(v[R.G_EPOCH])
            if 1 <= e < n_win and v[R.G_FUSED_N] > 0:
                f = np.array(v[R.G_FUSED:R.G_FUSED + 6])
                diffs.append(float(np.abs(f - central[e]).max()))
                fused[i][e] = f
            if 1 <= e < n_win and v[R.G_LOCAL_N] > 0:
                local[i][e] = np.array(v[R.G_LOCAL:R.G_LOCAL + 6])
    max_diff = max(diffs) if diffs else float("nan")

    # Noise of the gyroscope means over time (reported, not asserted)
    full = [e for e in range(1, n_win) if have[:, e].all()]
    offsets = np.array([node_means[i, full].mean(axis=0) for i in range(n)])
    single = np.array([[node_means[i, e] - offsets[i] for e in full] for i in range(n)])
    one_place = np.array([(node_means[:, e] - offsets).mean(axis=0) for e in full])
    sd_single = np.median(single[:, :, 3:].std(axis=1), axis=0)
    sd_one_place = one_place[:, 3:].std(axis=0)
    sd_fused_raw = np.median([np.array(list(fused[i].values()))[:, 3:].std(axis=0) for i in range(n)], axis=0)

    pinned = PINNED_CRC.get(a.seconds)
    ok_bits = mismatched == 0 and msgs_same
    ok_tol = diffs and max_diff <= TOL
    ok_crc = pinned is None or crc == pinned
    res = dict(record=os.path.relpath(a.record, R.ROOT), devices=nodes, seconds=a.seconds, steps=steps,
               window_s=DT * M, samples=sum(map(len, pushes)), windows=n_win - 1,
               windows_with_all_devices=len(full),
               outputs=total, outputs_mismatched=mismatched, messages_identical=msgs_same,
               crc32=f"{crc:08x}", crc32_pinned=None if pinned is None else f"{pinned:08x}",
               second_eigenvalue=float(lam2), max_fused_minus_one_place=max_diff, tolerance=TOL,
               gyro_std_single=sd_single.tolist(), gyro_std_fused_as_is=sd_fused_raw.tolist(),
               gyro_std_one_place_offsets_removed=sd_one_place.tolist(),
               gyro_noise_ratio_offsets_removed=(sd_single / sd_one_place).tolist())
    print(f"record {res['record']}: {n} devices {nodes}, {a.seconds:.0f} s, {res['samples']} samples, "
          f"{res['windows']} windows of {DT * M:.1f} s ({len(full)} with all devices)")
    print(f"WebAssembly vs reference model: {total - mismatched}/{total} outputs bit-exact, "
          f"messages identical: {msgs_same}")
    print(f"fused mean vs one-place mean: max |diff| {max_diff:.3e} (tolerance {TOL:g}; second eigenvalue of the "
          f"averaging {lam2:.4f}, ^{M} = {lam2 ** M:.1e})")
    print(f"CRC-32 of the outputs {crc:08x}" + ("" if pinned is None else f" (pinned {pinned:08x})"))
    print("gyroscope noise over time (rad/s, x y z): one device " + " ".join(f"{v:.2e}" for v in sd_single)
          + " | fused as it is " + " ".join(f"{v:.2e}" for v in sd_fused_raw)
          + " | one place, offsets removed, all devices " + " ".join(f"{v:.2e}" for v in sd_one_place)
          + f" (ratio {' '.join(f'{v:.2f}' for v in sd_single / sd_one_place)}; sqrt({n}) = {math.sqrt(n):.2f})")
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=1)
    ok = ok_bits and ok_tol and ok_crc
    print("PASS" if ok else "FAIL")
    if a.check and not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()

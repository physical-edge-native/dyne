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
3. The CRC-32 of the solver outputs 0..21 (all outputs before 2026-11-27) is the pinned value (detects any change
   in the solver or in this replay; with the default settings, adding output 22 must not change them).
4. Per-device zero offsets (issue #194): the same replay with `CFG_OFFSET_EPOCHS` = 10 (each device estimates its
   offset over the first 10 windows, while the board lies still) and `CFG_DEVICES` = 7. The WebAssembly solver and
   the reference model give the same bits; every fused window reports the number of devices that had samples in it
   (`G_FUSED_DEVICES`, compared with the count from the record); the fused gyroscope mean after the offset windows
   is near zero (|mean| below 3 times one device's noise), the fused acceleration keeps gravity (|a| within 0.5 % of
   the one-place mean), and the fused noise over time is at least 2 times lower than one device's (sqrt(7) = 2.65
   for seven equal devices). The CRC-32 of all outputs is pinned.

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
PINNED_CRC = {120.0: 0xE2C60E10}  # CRC-32 of WebAssembly outputs 0..21, per --seconds (issue #158)
OFFSET_EPOCHS = 10  # windows used to estimate each device's zero offset (issue #194)
SETTLE = 5          # windows after the offset windows left out of the noise check (the averaging settles)
MIN_RATIO = 2.0     # fused noise must be at least this much lower than one device's (sqrt(7) = 2.65 if ideal)
PINNED_CRC_OFFSETS = {120.0: 0xF1D164E3}  # CRC-32 of all outputs with offsets removed (issue #194)


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


def offsets_check(n, links, steps, pushes, node_means, have, central, sd_single, seconds):
    def cfg(i):
        return {R.CFG_EPOCH_STEPS: M, R.CFG_TIME_MODE: R.TIME_STAMP, R.CFG_OFFSET_EPOCHS: OFFSET_EPOCHS,
                R.CFG_DEVICES: n}

    r_ref, m_ref = R.run_network("ref", n, links, cfg, DT, steps, pushes, record_every=M, check_msgs=steps)
    r_wasm, m_wasm = R.run_network("wasm", n, links, cfg, DT, steps, pushes, record_every=M, check_msgs=steps)
    total = mismatched = 0
    crc = 0
    for s in sorted(r_wasm):
        for a_vals, b_vals in zip(r_ref[s], r_wasm[s]):
            for x, y in zip(a_vals, b_vals):
                total += 1
                by = struct.pack("<d", y)
                mismatched += struct.pack("<d", x) != by
                crc = zlib.crc32(by, crc)
    crc &= 0xFFFFFFFF
    n_win = steps // M
    count_ok = count_total = 0
    fused = {i: {} for i in range(n)}
    for s, vals in r_wasm.items():
        for i, v in enumerate(vals):
            e = int(v[R.G_EPOCH])
            if 0 <= e < n_win:
                count_total += 1
                count_ok += v[R.G_FUSED_DEVICES] == float(have[:, e].sum())
                if e >= OFFSET_EPOCHS + SETTLE and v[R.G_FUSED_N] > 0:
                    fused[i][e] = np.array(v[R.G_FUSED:R.G_FUSED + 6])
    f = np.array([np.array([fused[i][e] for e in sorted(fused[i])]) for i in range(n)])  # (n, windows, 6)
    eps = sorted(fused[0])
    gyro_mean = np.abs(f[:, :, 3:].mean(axis=1)).max(axis=0)
    g_fused = np.linalg.norm(f[:, :, :3].mean(axis=1), axis=1)
    g_one = np.linalg.norm(np.nanmean(central[eps, :3], axis=0))
    gravity_rel = float(np.abs(g_fused / g_one - 1).max())
    # noise over time against one device's: one device over the windows where every device has samples, the fused
    # output over all windows after the offset windows (including those where some devices had no samples)
    full = [e for e in eps if have[:, e].all()]
    single = np.array([node_means[i, full] - node_means[i, full].mean(axis=0) for i in range(n)])
    sd_one = np.median(single.std(axis=1), axis=0)
    sd_fused = np.median(f.std(axis=1), axis=0)
    ratio = sd_one / sd_fused
    pinned = PINNED_CRC_OFFSETS.get(seconds)
    ok = (mismatched == 0 and m_ref == m_wasm and count_ok == count_total and count_total > 0
          and (gyro_mean < 3 * sd_single).all() and gravity_rel < 5e-3 and (ratio >= MIN_RATIO).all()
          and (pinned is None or crc == pinned))
    return dict(offset_epochs=OFFSET_EPOCHS, outputs=total, outputs_mismatched=mismatched,
                messages_identical=m_ref == m_wasm, count_ok=count_ok, count_total=count_total,
                crc32=f"{crc:08x}", crc32_pinned=None if pinned is None else f"{pinned:08x}",
                gyro_mean=gyro_mean.tolist(), gravity_rel=gravity_rel, ratio=ratio.tolist(), ok=bool(ok))


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
            for k, (x, y) in enumerate(zip(a_vals, b_vals)):
                total += 1
                bx, by = struct.pack("<d", x), struct.pack("<d", y)
                mismatched += bx != by
                if k < R.G_LEN_V1:
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

    off = offsets_check(n, links, steps, pushes, node_means, have, central, sd_single, a.seconds)

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
               gyro_noise_ratio_offsets_removed=(sd_single / sd_one_place).tolist(), offsets=off)
    print(f"record {res['record']}: {n} devices {nodes}, {a.seconds:.0f} s, {res['samples']} samples, "
          f"{res['windows']} windows of {DT * M:.1f} s ({len(full)} with all devices)")
    print(f"WebAssembly vs reference model: {total - mismatched}/{total} outputs bit-exact, "
          f"messages identical: {msgs_same}")
    print(f"fused mean vs one-place mean: max |diff| {max_diff:.3e} (tolerance {TOL:g}; second eigenvalue of the "
          f"averaging {lam2:.4f}, ^{M} = {lam2 ** M:.1e})")
    print(f"CRC-32 of the outputs 0..{R.G_LEN_V1 - 1} {crc:08x}" + ("" if pinned is None else f" (pinned {pinned:08x})"))
    print("gyroscope noise over time (rad/s, x y z): one device " + " ".join(f"{v:.2e}" for v in sd_single)
          + " | fused as it is " + " ".join(f"{v:.2e}" for v in sd_fused_raw)
          + " | one place, offsets removed, all devices " + " ".join(f"{v:.2e}" for v in sd_one_place)
          + f" (ratio {' '.join(f'{v:.2f}' for v in sd_single / sd_one_place)}; sqrt({n}) = {math.sqrt(n):.2f})")
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=1)
    print(f"zero offsets removed in the solver ({OFFSET_EPOCHS} windows, issue #194): WebAssembly vs reference "
          f"{off['outputs'] - off['outputs_mismatched']}/{off['outputs']} bit-exact, messages identical: "
          f"{off['messages_identical']}; device counts match the record in {off['count_ok']}/{off['count_total']} "
          f"windows; CRC-32 {off['crc32']}" + ("" if off['crc32_pinned'] is None else f" (pinned {off['crc32_pinned']})"))
    print("  fused after the offset windows: gyro mean " + " ".join(f"{v:.1e}" for v in off['gyro_mean'])
          + f" rad/s, |a| / one-place |a| - 1 = {off['gravity_rel']:.1e}; noise ratio to one device: accel "
          + " ".join(f"{v:.2f}" for v in off['ratio'][:3]) + ", gyro " + " ".join(f"{v:.2f}" for v in off['ratio'][3:])
          + f" (at least {MIN_RATIO}) -> {'PASS' if off['ok'] else 'FAIL'}")
    ok = ok_bits and ok_tol and ok_crc and off['ok']
    print("PASS" if ok else "FAIL")
    if a.check and not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()

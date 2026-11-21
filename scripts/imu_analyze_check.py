"""Regression check for scripts/imu_analyze.py on a low-rate synthetic record (Issue #121).

The 2026-11-21 hardware record was sampled at about 12.5 Hz, each device had its own zero offset, the boards
were rotated on the plywood by up to about 10 degrees, and imu_fused.csv held only its header. This script
builds a deterministic record with the same features (4 devices, 12.5 Hz, one sample in 20 dropped, offsets
up to 0.3 m/s^2, mounting yaw -10..+10 degrees, a 0-degree and a 10-degree hold) and checks that the analyser

  - does not crash on the empty imu_fused.csv,
  - picks a 1 s epoch from the measured rate (12.5 Hz -> 8 or more samples per epoch),
  - gives a static noise reduction close to sqrt(4) = 2 on every accelerometer axis once offsets are removed
    (accepted range 1.6 to 2.4),
  - estimates the 10-degree tilt from the change of each device's gravity direction within 0.2 degrees.
    (The zero offsets are inside the measured gravity vector and cannot be told apart from how the device is
    tilted on the board without a calibration; offsets of 0.3 m/s^2 bias the estimate by about 0.1 degrees.)

Prints "IMU ANALYZE: PASS" and exits 0, or prints the failed checks and exits 1.
Usage: python scripts/imu_analyze_check.py <work dir>
"""
import json
import math
import os
import subprocess
import sys

import numpy as np

G0 = 9.80665


def rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])  # z


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(121)
    odr = 12.5
    t_start = 1_800_000_000_000_000  # us
    static_s, gap_s, hold_s = 400.0, 5.0, 60.0
    # timeline: static (level), then a 0-degree hold, then a 10-degree hold (board tilted about its x axis)
    seg = [(0.0, static_s, 0.0), (static_s + gap_s, static_s + gap_s + hold_s, 0.0),
           (static_s + 2 * gap_s + hold_s, static_s + 2 * gap_s + 2 * hold_s, 10.0)]
    t_end = seg[-1][1]
    rows = []
    for node, yaw in zip((11, 12, 13, 14), (-10.0, -3.0, 4.0, 10.0)):
        off = rng.uniform(-0.3, 0.3, 3)
        goff = rng.uniform(-5e-3, 5e-3, 3)
        mount = rot('z', yaw)
        n = int(t_end * odr)
        t = np.arange(n) / odr + rng.uniform(0, 1 / odr)
        keep = rng.random(n) > 0.05
        for k in np.where(keep)[0]:
            tk = t[k]
            tilt = next((d for a, b, d in seg if a <= tk < b), None)
            if tilt is None:
                continue
            g_board = rot('x', tilt).T @ np.array([0.0, 0.0, G0])
            acc = mount.T @ g_board + off + rng.normal(0, 2e-3, 3)
            gyr = goff + rng.normal(0, 1e-4, 3)
            rows.append((node, k, t_start + int(tk * 1e6), *acc, *gyr))
    rows.sort(key=lambda r: (r[0], r[2]))
    with open(os.path.join(out, 'imu_raw.csv'), 'w', encoding='utf-8') as f:
        f.write('node,seq,ts_us,local_us,ax,ay,az,gx,gy,gz,temp_c\n')
        for r in rows:
            f.write('%d,%d,%d,%d,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,\n' % (r[0], r[1], r[2], r[2] - t_start, *r[3:]))
    with open(os.path.join(out, 'imu_fused.csv'), 'w', encoding='utf-8') as f:
        f.write('node,step,epoch,ts_us,ax,ay,az,gx,gy,gz\n')  # header only, as in the hardware record
    with open(os.path.join(out, 'level.csv'), 'w', encoding='utf-8') as f:
        f.write('t0_us,t1_us,roll_deg,pitch_deg,label\n')
        for a, b, d in seg[1:]:
            f.write('%d,%d,%.2f,0,%s\n' % (t_start + int((a + 5) * 1e6), t_start + int((b - 5) * 1e6), d, 'hold%d' % d))
    here = os.path.dirname(os.path.abspath(__file__))
    env = dict(os.environ, PYTHONIOENCODING='utf-8')
    r = subprocess.run([sys.executable, os.path.join(here, 'imu_analyze.py'), out, '--static', '10,390'],
                       capture_output=True, text=True, encoding='utf-8', env=env)
    with open(os.path.join(out, 'analyze.log'), 'w', encoding='utf-8') as f:
        f.write(r.stdout + r.stderr)
    fails = []
    if r.returncode != 0:
        fails.append('imu_analyze.py exited with %d (see %s)' % (r.returncode, os.path.join(out, 'analyze.log')))
    else:
        s = json.load(open(os.path.join(out, 'analysis', 'summary.json'), encoding='utf-8'))
        if s['epoch_s'] != 1.0:
            fails.append('epoch %.2f s, expected 1.0 s for %.1f Hz' % (s['epoch_s'], s['odr_hz']))
        for k in ('ax', 'ay', 'az'):
            g = s['static_gain_all'][k]
            if not 1.6 <= g <= 2.4:
                fails.append('static gain %s = %.2f, expected 1.6..2.4 (sqrt(4) = 2)' % (k, g))
        hold = [t for t in s['tilt'] if t['label'] == 'hold10']
        if not hold or abs(hold[0]['tilt_change_err']) > 0.2:
            fails.append('10-degree hold: tilt error %s, expected within 0.2 deg' % (hold[0].get('tilt_change_err') if hold else None))
    if fails:
        for f_ in fails:
            print('FAIL:', f_)
        sys.exit(1)
    print('IMU ANALYZE: PASS (epoch %.1f s, gains ax %.2f ay %.2f az %.2f, 10-degree tilt error %+.3f deg)' % (
        s['epoch_s'], s['static_gain_all']['ax'], s['static_gain_all']['ay'], s['static_gain_all']['az'],
        hold[0]['tilt_change_err']))


if __name__ == '__main__':
    main()

# run2_events.txt (JST) -> level.csv (ts_us の時刻印、角度計の読み) と、静止・揺らしの範囲 (最初の標本からの秒)
import datetime as dt
import os
import sys
import numpy as np

raw, out = sys.argv[1], sys.argv[2]
JST = dt.timezone(dt.timedelta(hours=9))


def us(hms):
    h, m, s = map(int, hms.split(':'))
    return int(dt.datetime(2026, 11, 21, h, m, s, tzinfo=JST).timestamp() * 1e6)


ts = np.genfromtxt(raw, delimiter=',', names=True, usecols=(2,), dtype=np.int64)['ts_us']
ts = ts[ts > 0]
t_first = int(ts.min())
print('first sample', dt.datetime.fromtimestamp(t_first / 1e6, JST), 'last', dt.datetime.fromtimestamp(ts.max() / 1e6, JST))
M = 5_000_000  # 5 s inside each hold
holds = [('0deg', '06:53:40', '06:54:16', 0.00), ('9deg', '06:55:11', '06:56:51', 9.00),
         ('18deg', '06:59:05', '07:01:27', 17.65), ('back0', '07:01:53', '07:02:23', 0.10)]
with open(out, 'w', encoding='utf-8') as f:
    f.write('t0_us,t1_us,roll_deg,pitch_deg,label\n')
    for lab, a, b, ang in holds:
        f.write('%d,%d,%.2f,0,%s\n' % (us(a) + M, us(b) - M, ang, lab))
print('static_s %.1f,%.1f' % ((us('06:40:10') - t_first) / 1e6 + 5, (us('06:53:40') - t_first) / 1e6 - 5))
print('motion_us', us('07:04:25'), us('07:05:25'))

# run2 の追加の解析: (1) 台 232 のずれ方 (2) 232 を除いた6台の静止の利得 (3) 揺らしの区間で、時刻同期した
# 時刻印と、届いた時刻 (バッチの到着時刻から並べた時刻) で、各台と他の台の平均の一致を比べる
import sys, os, math, json
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "scripts"))
import imu_analyze as A

d = sys.argv[1]
nodes = A.load_raw(os.path.join(d, 'imu_raw.csv'))
ids = sorted(nodes)
t_first = min(nd['ts'][0] for nd in nodes.values())
s_lo, s_hi = t_first + 34.5e6, t_first + 834.5e6
res = {}

# (1) drift per node over the static part: linear slope of 60 s means
print('## 静止の中のゆっくりした変化 (60 s ごとの平均の、最大 − 最小) [m/s²]')
print('| node | ax | ay | az |\n|---|---|---|---|')
drift = {}
for i in ids:
    nd = nodes[i]
    m = (nd['ts'] > s_lo) & (nd['ts'] <= s_hi)
    t = (nd['ts'][m] - s_lo) / 1e6
    b = (t // 60).astype(int)
    means = np.array([nd['v'][m][b == k, :3].mean(axis=0) for k in np.unique(b)])
    drift[i] = np.ptp(means, axis=0)
    print('| %d | %.4f | %.4f | %.4f |' % (i, *drift[i]))
res['drift_ptp_60s'] = {str(i): v.tolist() for i, v in drift.items()}

# (2) gain with 6 nodes (without 232)
E = 1_000_000
t0 = (t_first // E) * E
nc, _ = A.remove_offsets(nodes, s_lo, s_hi)
e_lo = A.epoch_index(int(s_lo), t0, E) + 1
e_hi = A.epoch_index(int(s_hi), t0, E)


def stdmean(sel):
    S, n = A.epoch_sums(nc, sel, t0, E, e_lo, e_hi)
    with np.errstate(invalid='ignore'):
        c = S / n[:, None]
    return np.nanstd(c, axis=0)


six = [i for i in ids if i != 232]
single6 = np.mean([stdmean([i]) for i in six], axis=0)
g6 = single6 / stdmean(six)
single7 = np.mean([stdmean([i]) for i in ids], axis=0)
g7 = single7 / stdmean(ids)
names = ('ax', 'ay', 'az', 'gx', 'gy', 'gz')
print('\n## 静止の利得 (1台の平均 ÷ 全台の平均の標準偏差、1 s の区間)')
print('| | ' + ' | '.join(names) + ' |\n|---|' + '---|' * 6)
print('| 7台 (目安 2.65) | ' + ' | '.join('%.2f' % x for x in g7) + ' |')
print('| 232 を除く6台 (目安 2.45) | ' + ' | '.join('%.2f' % x for x in g6) + ' |')
res['gain7'] = dict(zip(names, g7.tolist()))
res['gain6_without_232'] = dict(zip(names, g6.tolist()))
res['std1_mean_6'] = dict(zip(names, single6.tolist()))
res['std6'] = dict(zip(names, stdmean(six).tolist()))

# (3) motion: synced timestamps vs arrival-derived timestamps
bat = np.genfromtxt(os.path.join(d, 'imu_batches.csv'), delimiter=',', names=True, dtype=None, encoding='utf-8')
mo_lo, mo_hi = int(sys.argv[2]), int(sys.argv[3])
grid = np.arange(mo_lo, mo_hi, 40_000)  # 25 Hz grid
odr = A.odr_of(nodes)


def arrival_ts(i):
    """バッチの到着時刻 (PC) から、バッチの中の標本を出力の周期で後ろから並べた時刻 [us]。
    標本はノードごとに seq の順で、バッチの n の数ずつ割り当てる"""
    m = bat['node'] == i
    b = bat[m]
    o = np.argsort(b['batch'])
    b = b[o]
    arr = np.repeat((b['pc_time'] * 1e6).astype(np.int64), b['n'])
    k = np.concatenate([np.arange(n)[::-1] for n in b['n']])
    t = arr - (k * 1e6 / odr).astype(np.int64)
    return t


def resample(ts, v):
    o = np.argsort(ts)
    return np.interp(grid, ts[o], v[o])


# the motion axis: largest variance of gyro in the window (central)
out = {}
for mode in ('synced', 'arrival'):
    sig = {}
    for i in ids:
        nd = nc[i]
        if mode == 'synced':
            ts = nd['ts']
        else:
            ta = arrival_ts(i)
            nn = min(len(ta), len(nd['ts']))
            # 記録は同期した時刻のある標本だけ。バッチの合計と数が違えば、末尾をそろえる
            ts = ta[-nn:] if len(ta) >= len(nd['ts']) else None
            if ts is None:
                ts = ta
                nd = dict(nd, v=nd['v'][-len(ta):])
            else:
                nd = dict(nd, v=nd['v'][-nn:])
        m = (ts > mo_lo - 2e6) & (ts < mo_hi + 2e6)
        sig[i] = np.stack([resample(ts[m], nd['v'][m][:, k]) for k in range(6)], axis=1)
    out[mode] = sig
cen = np.mean([out['synced'][i] for i in ids], axis=0)
ax_k = int(np.argmax(cen[:, 3:].std(axis=0))) + 3
amp = float(cen[:, ax_k].std())
print('\n## 揺らし (%.0f s): 動きの大きい成分 %s、他の台の平均との差の RMS' % ((mo_hi - mo_lo) / 1e6, names[ax_k]))
print('| 時刻 | 各台 − 他の台の平均 (RMS、台の平均) | 信号の大きさ (標準偏差) |\n|---|---|---|')
for mode in ('synced', 'arrival'):
    errs = []
    for i in ids:
        others = np.mean([out[mode][j][:, ax_k] for j in ids if j != i], axis=0)
        errs.append(np.sqrt(np.mean((out[mode][i][:, ax_k] - others) ** 2)))
    out[mode + '_rms'] = float(np.mean(errs))
    print('| %s | %.4f rad/s | %.4f rad/s |' % ({'synced': '時刻同期した時刻印', 'arrival': '届いた時刻から並べた時刻'}[mode], np.mean(errs), amp))
res['motion'] = dict(axis=names[ax_k], amplitude_std=amp, rms_synced=out['synced_rms'], rms_arrival=out['arrival_rms'])
json.dump(res, open(os.path.join(d, 'analysis', 'extra.json'), 'w'), indent=1)

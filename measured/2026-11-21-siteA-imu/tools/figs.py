# docs-site の IMU の頁の図 2 枚 (小さく): 静止の利得、台 232 のゆっくりした変化
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "scripts"))
import imu_analyze as A

d, out = sys.argv[1], sys.argv[2]
ex = json.load(open(os.path.join(d, 'analysis', 'extra.json')))
INK, MUTED, GRID = '#1d1d1b', '#5b5b57', '#e4e3df'
C1, C2 = '#2a78d6', '#eb6834'
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.edgecolor': '#8a8984', 'axes.grid': True, 'grid.color': GRID, 'figure.dpi': 110})

names = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
g7 = [ex['gain7'][k] for k in names]
g6 = [ex['gain6_without_232'][k] for k in names]
fig, ax = plt.subplots(figsize=(6.4, 3.0))
x = np.arange(6)
ax.bar(x - 0.2, g7, 0.38, color=C1, label='7 devices', edgecolor='white', linewidth=2)
ax.bar(x + 0.2, g6, 0.38, color=C2, label='6 devices (without node 232)', edgecolor='white', linewidth=2)
ax.axhline(np.sqrt(7), color=C1, ls='--', lw=1.2)
ax.axhline(np.sqrt(6), color=C2, ls=':', lw=1.2)
ax.text(5.65, np.sqrt(7) + 0.04, '√7', color=MUTED, fontsize=9, ha='right')
ax.text(5.65, np.sqrt(6) - 0.17, '√6', color=MUTED, fontsize=9, ha='right')
ax.set_xticks(x, ['accel x', 'accel y', 'accel z', 'gyro x', 'gyro y', 'gyro z'])
ax.set_ylabel('noise reduction\n(1 device ÷ average)')
ax.set_ylim(0, 3.0)
ax.legend(frameon=False, fontsize=9, loc='upper left', ncol=2)
fig.tight_layout()
fig.savefig(os.path.join(out, 'imu-static-gain.png'))
plt.close(fig)

nodes = A.load_raw(os.path.join(d, 'imu_raw.csv'))
t_first = min(nd['ts'][0] for nd in nodes.values())
lo, hi = t_first + 34.5e6, t_first + 834.5e6
fig, ax = plt.subplots(figsize=(6.4, 2.8))
for i in sorted(nodes):
    nd = nodes[i]
    m = (nd['ts'] > lo) & (nd['ts'] <= hi)
    t = (nd['ts'][m] - lo) / 1e6
    b = (t // 30).astype(int)
    tt = np.array([t[b == k].mean() for k in np.unique(b)]) / 60
    v = np.array([nd['v'][m][b == k, 1].mean() for k in np.unique(b)])
    v -= v.mean()
    if i == 232:
        ax.plot(tt, v * 1000, color=C2, lw=2.2, label='node 232', zorder=3)
    else:
        ax.plot(tt, v * 1000, color='#9a9a95', lw=1.2, label='other 6 nodes' if i == 40 else None)
ax.set_xlabel('time in the static part [min]')
ax.set_ylabel('accel y, 30 s mean\nminus its average [mm/s²]')
ax.legend(frameon=False, fontsize=9)
fig.tight_layout()
fig.savefig(os.path.join(out, 'imu-node232-drift.png'))
plt.close(fig)
print('ok')

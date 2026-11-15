# IMU の協調によるノイズ除去 (実験9、S8、E5) のシミュレーション: 同じ板に載せた N 台の IMU の
# 合成データを作り、隣との通信だけで融合するアクター (crates/imu-fusion-actor) の参照モデル
# (scripts/imu_fusion_ref.py) で融合して、台数・つながり方・時刻同期の有無で比べる。
#
# 出所: Claude Code が 2026-11-13 に作成。記録は docs/2026-11-13-imu-fusion.md §4。
# 使い方 (リポジトリ直下で):
#   .venv\Scripts\python.exe scripts\imu_sim.py all            … 全部 (数分。図と数値は demo_out\imu\)
#   .venv\Scripts\python.exe scripts\imu_sim.py static|allan|tilt|motion|wasm
#   .venv\Scripts\python.exe scripts\imu_sim.py logs <出力先>  … 実機の記録と同じ形式の合成記録 (imu_analyze.py の確認用)
# 乱数のシードは固定 (同じ入力なら同じ数値)。wasm の照合には crates\imu-fusion-actor のビルドが要る:
#   cd crates\imu-fusion-actor && cargo build --release --target wasm32-unknown-unknown
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_fusion_ref as R  # noqa: E402

OUT = os.path.join(R.ROOT, 'demo_out', 'imu')
G0 = 9.80665
D2R = math.pi / 180.0

# ---- センサの模型 (LSM6DSOX のデータシートの代表値に基づく仮定。実測で置き換える) ----
# 雑音密度: 加速度 70 µg/√Hz、角速度 3.8 mdps/√Hz (高性能モード)。帯域 ≒ 出力の周期の半分 (52Hz)
P = dict(
    odr=104.0, odr_tol=0.01,
    a_white=70e-6 * G0 * math.sqrt(52.0), g_white=3.8e-3 * D2R * math.sqrt(52.0),
    a_bias=5e-3 * G0, g_bias=0.3 * D2R,                   # 個体ごとのゼロ点のずれ (標準偏差)
    a_gm=(1e-4, 200.0), g_gm=(2e-5, 200.0),               # ゼロ点のゆらぎ (1次のガウス・マルコフ: σ, 時定数 s)
    dT=(3.0, 900.0),                                      # 板の温度上昇 [℃] と時定数 [s] (全台共通)
    a_tc=(0.05e-3 * G0, 0.05e-3 * G0), g_tc=(0.005 * D2R, 0.005 * D2R),  # 温度係数 (平均, 個体差) /℃
    vib_a=2e-3, vib_g=5e-5, vib_band=(5.0, 30.0),         # 机の振動 (全台共通) [m/s²], [rad/s]
    sync_off_us=300.0, sync_jit_us=50.0,                  # 同期ありの時刻印の誤差 (ノードごとのずれ, 揺らぎ)
    fifo_ms=20.0, lat_max_ms=60.0, lat_jit_ms=5.0,        # 届いた順の模型: FIFO の読み出し周期、ステップの遅れ
)
DT = 0.02      # 合意の1反復 = 50Hz
M = 10         # 1区間 = 10反復 = 0.2s
E_S = DT * M
CAL = 25       # 角速度のゼロ点の較正 (最初の 5s)
COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']


# ---------- 真の動き ----------

def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return 0.5 - 0.5 * np.cos(np.pi * x)


def tilt_schedule():
    """傾きの階段: (角度[度], 保持[s]) と移り変わり 3s"""
    return [(0.0, 20.0), (10.0, 30.0), (20.0, 30.0)], 3.0


def truth_angles(scn, t):
    """(roll, pitch [rad], 並進の加速度 (3, len)) を返す"""
    t = np.asarray(t, dtype=float)
    roll = np.zeros_like(t)
    pitch = np.zeros_like(t)
    lin = np.zeros((3,) + t.shape)
    if scn == 'tilt':
        holds, tr = tilt_schedule()
        start = 0.0
        prev = holds[0][0]
        for ang, dur in holds:
            if ang != prev:
                roll = roll + (ang - prev) * D2R * smoothstep((t - start) / tr)
                start += tr
            start += dur
            prev = ang
    elif scn == 'motion':
        env = smoothstep((t - 20.0) / 3.0) * (1.0 - smoothstep((t - 77.0) / 3.0))
        roll = env * 15.0 * D2R * np.sin(2 * np.pi * 0.3 * (t - 20.0))
        pitch = env * 10.0 * D2R * np.sin(2 * np.pi * 0.17 * (t - 20.0))
        lin[0] = env * 0.8 * np.sin(2 * np.pi * 1.5 * (t - 20.0))
    return roll, pitch, lin


def scenario_len(scn):
    if scn == 'tilt':
        holds, tr = tilt_schedule()
        return sum(d for _, d in holds) + tr * (len(holds) - 1)
    if scn == 'motion':
        return 100.0
    return 600.0


def truth6(scn, t):
    """真の加速度 (比力) と角速度 (6, len)"""
    h = 1e-4
    r, p, lin = truth_angles(scn, t)
    r1, p1, _ = truth_angles(scn, t + h)
    r0, p0, _ = truth_angles(scn, t - h)
    rd = (r1 - r0) / (2 * h)
    pd = (p1 - p0) / (2 * h)
    a = np.stack([-G0 * np.sin(p), G0 * np.sin(r) * np.cos(p), G0 * np.cos(r) * np.cos(p)]) + lin
    g = np.stack([rd, pd * np.cos(r), -pd * np.sin(r)])
    return np.concatenate([a, g])


def truth_epochs(scn, n_ep):
    """区間 e = (e·E, (e+1)·E] の真の値の平均 (1kHz の格子で積分)。(n_ep, 6) と roll, pitch (n_ep)"""
    k = int(round(E_S * 1000))
    t = (np.arange(n_ep * k) + 0.5) / 1000.0
    v = truth6(scn, t).reshape(6, n_ep, k).mean(axis=2).T
    r, p, _ = truth_angles(scn, t)
    return v, r.reshape(n_ep, k).mean(axis=1), p.reshape(n_ep, k).mean(axis=1)


# ---------- 雑音 ----------

def band_noise(rng, t_end, fs, band, sigma, n_series):
    """共通の振動: 白色雑音を周波数領域で帯域に切り出す (n_series, len)"""
    n = int(t_end * fs) + 2
    w = rng.standard_normal((n_series, n))
    F = np.fft.rfft(w, axis=1)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    F[:, (f < band[0]) | (f > band[1])] = 0.0
    x = np.fft.irfft(F, n, axis=1)
    x *= sigma / x.std(axis=1, keepdims=True)
    return x


def gauss_markov(rng, n, sigma, tau, fs, ch):
    a = math.exp(-1.0 / (fs * tau))
    q = sigma * math.sqrt(1 - a * a)
    w = rng.standard_normal((n, ch)) * q
    x = np.empty((n, ch))
    s = rng.standard_normal(ch) * sigma
    for i in range(n):
        s = a * s + w[i]
        x[i] = s
    return x


def gen_streams(scn, n_nodes, seed, common=True, t_end=None, params=P):
    """各ノードの標本: dict(t=真の取得時刻 s, v=(len,6) 測定値, temp)。ノードの個体差もここで決める"""
    p = params
    rng = np.random.default_rng(seed)
    t_end = t_end or scenario_len(scn)
    vib = band_noise(rng, t_end + 1, 1000.0, p['vib_band'], 1.0, 6)
    vib[:3] *= p['vib_a'] if common else 0.0
    vib[3:] *= p['vib_g'] if common else 0.0
    grid = np.arange(vib.shape[1]) / 1000.0
    nodes = []
    for i in range(n_nodes):
        odr = p['odr'] * (1 + rng.uniform(-p['odr_tol'], p['odr_tol']))
        ph = rng.uniform(0, 1.0 / odr)
        t = ph + np.arange(int((t_end - ph) * odr)) / odr
        v = truth6(scn, t).T.copy()
        n = len(t)
        sw = np.array([p['a_white']] * 3 + [p['g_white']] * 3)
        v += rng.standard_normal((n, 6)) * sw
        v[:, :3] += rng.standard_normal(3) * p['a_bias']
        v[:, 3:] += rng.standard_normal(3) * p['g_bias']
        v[:, :3] += gauss_markov(rng, n, p['a_gm'][0], p['a_gm'][1], odr, 3)
        v[:, 3:] += gauss_markov(rng, n, p['g_gm'][0], p['g_gm'][1], odr, 3)
        dT = p['dT'][0] * (1 - np.exp(-t / p['dT'][1]))
        km_a = p['a_tc'][0] if common else 0.0
        km_g = p['g_tc'][0] if common else 0.0
        ka = km_a + rng.standard_normal(3) * p['a_tc'][1]
        kg = km_g + rng.standard_normal(3) * p['g_tc'][1]
        v[:, :3] += dT[:, None] * ka
        v[:, 3:] += dT[:, None] * kg
        for k in range(6):
            v[:, k] += np.interp(t, grid, vib[k])
        nodes.append(dict(t=t, v=v, temp=25.0 + dT, odr=odr))
    return nodes


# ---------- ノードの時計と、標本をアクターに入れる時点 ----------

def clock_model(nodes, seed, steps, p=P):
    """各ノード: 時刻印 ts_us (同期あり)、届く時刻、ステップを呼ぶ時刻"""
    rng = np.random.default_rng(seed + 1000)
    for nd in nodes:
        off = rng.normal(0, p['sync_off_us']) * 1e-6
        jit = rng.normal(0, p['sync_jit_us'], len(nd['t'])) * 1e-6
        nd['ts_us'] = np.round((nd['t'] + off + jit) * 1e6).astype(np.int64)
        fifo = p['fifo_ms'] * 1e-3
        fph = rng.uniform(0, fifo)
        nd['arrive'] = np.ceil((nd['t'] + 1e-3 - fph) / fifo) * fifo + fph
        lat = rng.uniform(0, p['lat_max_ms']) * 1e-3
        calls = (np.arange(steps) + 1) * DT + lat + np.abs(rng.normal(0, p['lat_jit_ms'] * 1e-3, steps))
        nd['call_t'] = np.maximum.accumulate(calls)
        # 同期なしの時計 (起動からの esp_timer): 起動の時刻の差 (数秒) とドリフト (±30ppm)
        boot = rng.uniform(0, 5.0)
        drift = rng.uniform(-30e-6, 30e-6)
        nd['local_us'] = np.round((nd['t'] - boot + nd['t'] * drift) * 1e6 + 5e6).astype(np.int64)


def push_schedule(nodes, mode, steps, offsets_us=None):
    """pushes[i] = [(呼び出し番号, ts_us, 値6)]。時刻印の割り当てでは、区間が閉じる前に必ず入れる
    (ファームが dyne_imu_need_until_us まで待つことに当たる)"""
    E_us = int(DT * 1e6 + 0.5) * M
    out = []
    for i, nd in enumerate(nodes):
        c = np.searchsorted(nd['call_t'], nd['arrive'], side='left')
        if mode == R.TIME_STAMP:
            off = 0 if offsets_us is None else int(offsets_us[i])
            e = (nd['ts_us'] + off - 1) // E_us
            close = (e + 1) * M - 1
            c = np.minimum(c, close)
        c = np.maximum(c, 0)
        keep = c < steps
        order = np.argsort(c[keep], kind='stable')
        cc = c[keep][order]
        ts = nd['ts_us'][keep][order]
        vv = nd['v'][keep][order]
        out.append([(int(a), int(b), tuple(float(x) for x in row)) for a, b, row in zip(cc, ts, vv)])
    return out


def central_epochs(pushes, mode, n_ep, offsets_us=None):
    """アクターと同じ区間の割り当てで、PC が全ノードの値をまとめて平均したもの (中央の融合) (n_ep, 6)。
    アクターと同じく、ノードごとの平均を等しい重みで平均する"""
    E_us = int(DT * 1e6 + 0.5) * M
    tot = np.zeros((n_ep, 6))
    cnt = np.zeros(n_ep)
    for i, lst in enumerate(pushes):
        off = 0 if offsets_us is None else int(offsets_us[i])
        S = np.zeros((n_ep, 6))
        n = np.zeros(n_ep)
        for c, ts, v in lst:
            e = c // M if mode == R.TIME_ARRIVAL else (ts + off - 1) // E_us
            if 0 <= e < n_ep:
                S[e] += v
                n[e] += 1
        ok = n > 0
        tot[ok] += S[ok] / n[ok, None]
        cnt[ok] += 1
    with np.errstate(invalid='ignore'):
        return tot / cnt[:, None]


def run_actor(nodes, topo, mode=R.TIME_STAMP, offsets_us=None, impl='ref', rows=2, alpha=0.9, cons=R.CONS_DYNAMIC):
    n = len(nodes)
    steps = int(round(scenario_len_nodes(nodes) / DT))
    links = R.topology(topo, n, rows=rows)
    pushes = push_schedule(nodes, mode, steps, offsets_us)

    def cfg(i):
        c = {R.CFG_EPOCH_STEPS: M, R.CFG_TIME_MODE: mode, R.CFG_CF_ALPHA: alpha, R.CFG_GYRO_CAL_EPOCHS: CAL,
             R.CFG_CONSENSUS: cons}
        if offsets_us is not None:
            c[R.CFG_TS_OFFSET_US] = float(offsets_us[i])
        return c

    rec, _ = R.run_network(impl, n, links, cfg, DT, steps, pushes, record_every=M)
    # rec[q·M] = 区間 q−2 の確定値
    n_ep = steps // M
    out = np.full((n, n_ep, R.G_LEN), np.nan)
    for s, vals in rec.items():
        e = s // M - 2
        if e >= 0:
            out[:, e, :] = np.array(vals)
    return out, pushes, n_ep


def scenario_len_nodes(nodes):
    return math.floor(min(nd['t'][-1] for nd in nodes) / E_S) * E_S


# ---------- 解析の道具 ----------

def allan_dev(x, fs, n_tau=40):
    """重なりありのアラン偏差 (tau, adev)"""
    x = np.asarray(x, float)
    n = len(x)
    cs = np.concatenate([[0.0], np.cumsum(x)])
    ms = np.unique(np.logspace(0, math.log10(n // 3), n_tau).astype(int))
    taus, ad = [], []
    for m in ms:
        a = (cs[2 * m:] - 2 * cs[m:-m] + cs[:-2 * m]) / m
        taus.append(m / fs)
        ad.append(math.sqrt(0.5 * np.mean(a * a)))
    return np.array(taus), np.array(ad)


def plt_setup():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True,
                         'grid.color': '#e4e3df', 'grid.linewidth': 0.8, 'axes.edgecolor': '#8a8984',
                         'font.size': 9, 'lines.linewidth': 2.0, 'figure.dpi': 120,
                         'axes.prop_cycle': matplotlib.cycler(color=COLORS)})
    return plt


# ---------- 実験 ----------

def exp_static(summary):
    """① 静止: 台数 N と標準偏差、中央の融合と隣だけの融合"""
    plt = plt_setup()
    res = {}
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.6))
    for common in (False, True):
        nodes8 = gen_streams('static', 8, seed=11, common=common)
        clock_model(nodes8, 11, int(scenario_len_nodes(nodes8) / DT))
        rows = []
        cases = [('line', 1, R.CONS_DYNAMIC), ('line', 2, R.CONS_DYNAMIC), ('line', 4, R.CONS_DYNAMIC),
                 ('line', 8, R.CONS_DYNAMIC), ('ring', 8, R.CONS_DYNAMIC), ('grid', 8, R.CONS_DYNAMIC),
                 ('line', 8, R.CONS_RESET), ('grid', 8, R.CONS_RESET)]
        for topo, N, cons in cases:
            nodes = nodes8[:N]
            out, pushes, n_ep = run_actor(nodes, topo, cons=cons)
            cen = central_epochs(pushes, R.TIME_STAMP, n_ep)
            sl = slice(CAL + 2, n_ep - 1)

            def sstd(x):
                # 短時間のばらつき: 隣の区間との差の標準偏差 / √2 (ゆっくりした漂い (温度・ゼロ点のゆらぎ) を除く)
                return float(np.std(np.diff(x)) / math.sqrt(2))

            fz = out[:, sl, R.G_FUSED + 2]
            fg = out[:, sl, R.G_FUSED + 3]
            row = dict(topo=topo, N=N, cons='dynamic' if cons == R.CONS_DYNAMIC else 'reset',
                       std_az_fused=float(np.mean(np.std(fz, axis=1))),
                       std_az_central=float(np.std(cen[sl, 2])),
                       sstd_az_fused=float(np.mean([sstd(x) for x in fz])),
                       sstd_az_central=sstd(cen[sl, 2]),
                       std_gx_fused=float(np.mean(np.std(fg, axis=1))),
                       std_gx_central=float(np.std(cen[sl, 3])),
                       sstd_gx_fused=float(np.mean([sstd(x) for x in fg])),
                       sstd_gx_central=sstd(cen[sl, 3]),
                       rms_cons_minus_central_az=float(np.sqrt(np.mean((fz - cen[sl, 2]) ** 2))),
                       rms_cons_minus_central_gx=float(np.sqrt(np.mean((fg - cen[sl, 3]) ** 2))),
                       max_abs_mean_offset_az=float(np.max(np.abs(np.mean(fz - cen[sl, 2], axis=1)))))
            rows.append(row)
            print('static common=%d %s%d %-7s short-std az fused %.3e central %.3e | std az %.3e/%.3e | '
                  'short gx %.3e/%.3e | cons-central az rms %.1e maxoff %.1e'
                  % (common, topo, N, row['cons'], row['sstd_az_fused'], row['sstd_az_central'], row['std_az_fused'],
                     row['std_az_central'], row['sstd_gx_fused'], row['sstd_gx_central'],
                     row['rms_cons_minus_central_az'], row['max_abs_mean_offset_az']))
        res['common' if common else 'independent'] = rows
        line = [r for r in rows if r['topo'] == 'line' and r['cons'] == 'dynamic']
        Ns = np.array([r['N'] for r in line])
        for ax, key, unit in ((axes[0], 'az', 'm/s²'), (axes[1], 'gx', 'rad/s')):
            y = np.array([r['sstd_%s_fused' % key] for r in line])
            yc = np.array([r['sstd_%s_central' % key] for r in line])
            lab = 'with common-mode' if common else 'independent only'
            col = COLORS[1] if common else COLORS[0]
            ax.plot(Ns, y, 'o-', color=col, label='neighbour-only, ' + lab)
            ax.plot(Ns, yc, 'x', color=col, markersize=9, label='central, ' + lab)
            if not common:
                ax.plot(Ns, y[0] / np.sqrt(Ns), ':', color='#8a8984', label='1/√N')
            ax.set_xscale('log', base=2)
            ax.set_yscale('log')
            ax.set_xticks([1, 2, 4, 8], ['1', '2', '4', '8'])
            ax.set_xlabel('N (nodes, line topology)')
            ax.set_ylabel('short-term std of 0.2 s mean, %s [%s]' % (key, unit))
    axes[0].legend(fontsize=7, frameon=False)
    fig.suptitle('Static: noise vs N (600 s, epoch 0.2 s)')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'static_std_vs_n.png'))
    plt.close(fig)
    # 合意の収束の速さ (メトロポリスの重みの第2固有値)
    conv = {}
    for topo, N in [('line', 2), ('line', 4), ('line', 8), ('ring', 8), ('grid', 8)]:
        W = R.metropolis_matrix(N, R.topology(topo, N, rows=2))
        ev = np.sort(np.abs(np.linalg.eigvalsh(W)))[::-1]
        lam = float(ev[1]) if N > 1 else 0.0
        conv['%s%d' % (topo, N)] = dict(lambda2=lam, factor_after_M=lam ** M)
    res['convergence'] = conv
    print('convergence', conv)
    summary['static'] = res


def exp_allan(summary):
    """① 静止: アラン偏差 (標本の番号で揃えた中央の平均。2時間)"""
    plt = plt_setup()
    rng = np.random.default_rng(21)
    fs = P['odr']
    T = 7200.0
    n = int(T * fs)
    N = 8
    vib = band_noise(rng, T, fs, P['vib_band'], 1.0, 2)
    vib[0] *= P['vib_a']
    vib[1] *= P['vib_g']
    t = np.arange(n) / fs
    dT = P['dT'][0] * (1 - np.exp(-t / P['dT'][1]))
    ax_ = np.zeros((N, n))
    gx_ = np.zeros((N, n))
    for i in range(N):
        ax_[i] = (rng.standard_normal(n) * P['a_white'] + gauss_markov(rng, n, P['a_gm'][0], P['a_gm'][1], fs, 1)[:, 0]
                  + dT * (P['a_tc'][0] + rng.standard_normal() * P['a_tc'][1]) + vib[0][:n])
        gx_[i] = (rng.standard_normal(n) * P['g_white'] + gauss_markov(rng, n, P['g_gm'][0], P['g_gm'][1], fs, 1)[:, 0]
                  + dT * (P['g_tc'][0] + rng.standard_normal() * P['g_tc'][1]) + vib[1][:n])
    res = {}
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.6))
    for k, (name, data, unit, scale) in enumerate((('ax', ax_, 'm/s²', 1.0), ('gx', gx_, 'deg/h', 180 / math.pi * 3600))):
        res[name] = {}
        for j, Nn in enumerate((1, 2, 4, 8)):
            tau, ad = allan_dev(data[:Nn].mean(axis=0), fs)
            i1 = int(np.argmin(np.abs(tau - 1.0)))
            res[name]['N%d' % Nn] = dict(adev_at_1s=float(ad[i1] * scale), min_adev=float(ad.min() * scale),
                                         tau_at_min=float(tau[np.argmin(ad)]))
            axes[k].loglog(tau, ad * scale, color=COLORS[j], label='N=%d' % Nn)
        axes[k].set_xlabel('τ [s]')
        axes[k].set_ylabel('Allan deviation of %s [%s]' % (name, unit))
        axes[k].legend(frameon=False, fontsize=8)
    fig.suptitle('Static 2 h: Allan deviation of the central mean (with common-mode terms)')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'allan.png'))
    plt.close(fig)
    print('allan', json.dumps(res, indent=1))
    summary['allan'] = res


def exp_tilt(summary, seeds=32):
    """② 傾き: 0°→10°→20° を保持した区間の、融合した傾きの誤差"""
    plt = plt_setup()
    holds, tr = tilt_schedule()
    res = {}
    example = None
    for N, cons in ((1, R.CONS_DYNAMIC), (2, R.CONS_DYNAMIC), (4, R.CONS_DYNAMIC), (8, R.CONS_DYNAMIC),
                    (8, R.CONS_RESET)):
        key = 'N%d' % N + ('' if cons == R.CONS_DYNAMIC else '_reset')
        errs, errs_cal, errs_all = [], [], []
        for sd in range(seeds):
            nodes = gen_streams('tilt', 8, seed=100 + sd)[:N]
            clock_model(nodes, 100 + sd, int(scenario_len_nodes(nodes) / DT))
            out, _, n_ep = run_actor(nodes, 'line', cons=cons)
            _, troll, _ = truth_epochs('tilt', n_ep)
            roll = out[0, :, R.G_FUSED_ROLL]
            # 保持の区間 (移り変わりの後 8s 以降)
            start = 0.0
            masks = []
            for idx, (ang, dur) in enumerate(holds):
                if idx > 0:
                    start += tr
                tc = (np.arange(n_ep) + 0.5) * E_S
                m = (tc > start + 8.0) & (tc < start + dur - 0.5)
                masks.append((ang, m))
                start += dur
            zero = np.nanmean(roll[masks[0][1]] - troll[masks[0][1]])
            for ang, m in masks[1:]:
                e = (roll[m] - troll[m]) / D2R
                errs.append(np.nanmean(e))
                errs_cal.append(np.nanmean(e - zero / D2R))
                # 全ノードの平均の誤差 (ノード0だけでなく)
                errs_all.append(np.nanmean((out[:, m, R.G_FUSED_ROLL] - troll[m]) / D2R))
            if N == 8 and sd == 0 and cons == R.CONS_DYNAMIC:
                example = (np.arange(n_ep) * E_S + E_S, roll / D2R, out[0, :, R.G_LOCAL_ROLL] / D2R, troll / D2R)
        res[key] = dict(rms_err_deg=float(np.sqrt(np.mean(np.square(errs)))),
                        rms_err_after_zero_cal_deg=float(np.sqrt(np.mean(np.square(errs_cal)))),
                        rms_err_all_nodes_deg=float(np.sqrt(np.mean(np.square(errs_all)))))
        print('tilt', key, res[key])
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.4))
    Ns = [1, 2, 4, 8]
    ax[0].plot(Ns, [res['N%d' % n]['rms_err_deg'] for n in Ns], 'o-', label='no calibration')
    ax[0].plot(Ns, [res['N%d' % n]['rms_err_after_zero_cal_deg'] for n in Ns], 'o-', label='zero at 0° (level)')
    ax[0].set_xscale('log', base=2)
    ax[0].set_xticks(Ns, [str(n) for n in Ns])
    ax[0].set_xlabel('N (nodes)')
    ax[0].set_ylabel('RMS tilt error at 10°/20° holds [deg]')
    ax[0].legend(frameon=False, fontsize=8)
    t, fr, lr, tr_ = example
    ax[1].plot(t, tr_, color='#8a8984', label='truth')
    ax[1].plot(t, lr, color=COLORS[1], linewidth=1.2, label='node 0 alone')
    ax[1].plot(t, fr, color=COLORS[0], linewidth=1.2, label='fused, N=8')
    ax[1].set_xlabel('time [s]')
    ax[1].set_ylabel('roll [deg]')
    ax[1].legend(frameon=False, fontsize=8)
    fig.suptitle('Tilt steps (complementary filter, %d random sensor sets)' % seeds)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'tilt.png'))
    plt.close(fig)
    summary['tilt'] = res


DESYNC_OFFSETS = [20000 * i for i in range(8)]   # わざとずらす量の例: ノード i を 20ms·i


def motion_conditions():
    return [('sync', R.TIME_STAMP, None), ('desync_offset', R.TIME_STAMP, DESYNC_OFFSETS),
            ('arrival', R.TIME_ARRIVAL, None)]


def exp_motion(summary):
    """③ 動き: 時刻同期あり/なし (わざとずらす、届いた順) で融合の誤差を比べる"""
    plt = plt_setup()
    nodes8 = gen_streams('motion', 8, seed=31)
    clock_model(nodes8, 31, int(scenario_len_nodes(nodes8) / DT))
    res = {}
    series = {}
    for N, topo in ((1, 'line'), (8, 'grid')):
        nodes = nodes8[:N]
        for name, mode, offs in motion_conditions():
            o = None if offs is None else offs[:N]
            out, pushes, n_ep = run_actor(nodes, topo, mode=mode, offsets_us=o, rows=2)
            tv, troll, _ = truth_epochs('motion', n_ep)
            # 同期した時刻印で PC がまとめて平均したもの (実機で真値の代わりに使える基準)
            ref_sync = central_epochs(push_schedule(nodes, R.TIME_STAMP, n_ep * M), R.TIME_STAMP, n_ep)
            tc = (np.arange(n_ep) + 0.5) * E_S
            moving = (tc > 25.0) & (tc < 75.0)
            before = (tc > 8.0) & (tc < 18.0)
            still = before | ((tc > 82.0) & (tc < 98.0))
            f = out[:, :, R.G_FUSED:R.G_FUSED + 6]
            # 誤差は、動く前の静止 (8〜18s) の平均の誤差 (ゼロ点のずれ) を各ノードで引いてから測る。
            # 引かないと、ゼロ点のずれ (台数を増やしても平均でしか減らない) が同期の効果を隠す
            err = f - tv[None]
            err = err - np.nanmean(err[:, before], axis=1, keepdims=True)
            err_raw = f - tv[None]
            err_ref = f - ref_sync[None]
            loc = out[:, :, R.G_LOCAL:R.G_LOCAL + 6]
            with np.errstate(all='ignore'):
                spread = loc - np.nanmean(loc, axis=0, keepdims=True)
            spread = spread - np.nanmean(spread[:, before], axis=1, keepdims=True)

            def rms(a, m, k):
                return float(np.sqrt(np.nanmean(a[:, m, k] ** 2)))

            r = {}
            for seg, m in (('still', still), ('moving', moving)):
                r[seg] = dict(ay_vs_truth=rms(err, m, 1), ay_vs_truth_no_zero_cal=rms(err_raw, m, 1), ax_vs_truth=rms(err, m, 0), gx_vs_truth=rms(err, m, 3),
                              ay_vs_sync_central=rms(err_ref, m, 1), gx_vs_sync_central=rms(err_ref, m, 3),
                              ay_node_spread=rms(spread, m, 1),
                              roll_err_deg=float(np.sqrt(np.nanmean((out[:, m, R.G_FUSED_ROLL] - troll[m]) ** 2)) / D2R))
            r['late'] = float(out[:, -1, R.G_LATE].sum())
            r['early'] = float(out[:, -1, R.G_EARLY].sum())
            res['N%d_%s' % (N, name)] = r
            print('motion N=%d %-13s still ay %.4f moving ay %.4f (vs sync-central %.4f) spread %.4f roll %.2f deg'
                  % (N, name, r['still']['ay_vs_truth'], r['moving']['ay_vs_truth'],
                     r['moving']['ay_vs_sync_central'], r['moving']['ay_node_spread'], r['moving']['roll_err_deg']))
            if N == 8:
                series[name] = (tc, err[0, :, 1])
    fig, ax = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)
    labels = {'sync': 'synced timestamps', 'desync_offset': 'clocks offset 0–140 ms', 'arrival': 'arrival order (no timestamps)'}
    for j, (name, _, _) in enumerate(motion_conditions()):
        tc, e = series[name]
        ax[1].plot(tc, e, linewidth=1.0, color=COLORS[j], label=labels[name])
    tcf = np.linspace(0, 100, 5000)
    tvv = truth6('motion', tcf)
    ax[0].plot(tcf, tvv[1], color='#8a8984', linewidth=1.0)
    ax[0].set_ylabel('true ay [m/s²]')
    ax[1].set_ylabel('fused ay − truth [m/s²]')
    ax[1].set_xlabel('time [s]')
    ax[1].legend(frameon=False, fontsize=8, loc='upper right')
    fig.suptitle('Motion (N=8, 2×4 grid): error of the fused 0.2 s mean, node 0')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'motion_sync.png'))
    plt.close(fig)
    # 棒グラフ: 動いている間の誤差
    fig, ax = plt.subplots(figsize=(6.5, 3.2))
    names = [c[0] for c in motion_conditions()]
    x = np.arange(len(names))
    for j, N in enumerate((1, 8)):
        ax.bar(x + (j - 0.5) * 0.36, [res['N%d_%s' % (N, nm)]['moving']['ay_vs_truth'] for nm in names], 0.34,
               color=COLORS[j], label='N=%d' % N)
    ax.set_xticks(x, [labels[n] for n in names], fontsize=8)
    ax.set_ylabel('RMS error of ay while moving [m/s²]')
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'motion_bars.png'))
    plt.close(fig)
    summary['motion'] = res


def exp_wasm(summary):
    """参照モデルと実物の WASM の照合 (動きの場面、8台の格子、3条件)。全出力・全メッセージのビット一致"""
    import struct
    if not os.path.exists(R.WASM):
        print('wasm がない: ' + R.WASM)
        summary['wasm'] = 'not built'
        return
    nodes = gen_streams('motion', 8, seed=31)
    steps = int(scenario_len_nodes(nodes) / DT)
    clock_model(nodes, 31, steps)
    links = R.topology('grid', 8, rows=2)
    res = {}
    for name, mode, offs in motion_conditions():
        pushes = push_schedule(nodes, mode, steps, offs)

        def cfg(i):
            c = {R.CFG_EPOCH_STEPS: M, R.CFG_TIME_MODE: mode, R.CFG_GYRO_CAL_EPOCHS: CAL}
            if offs is not None:
                c[R.CFG_TS_OFFSET_US] = float(offs[i])
            return c

        t0 = time.time()
        r1, m1 = R.run_network('ref', 8, links, cfg, DT, steps, pushes, record_every=M, check_msgs=steps)
        t1 = time.time()
        r2, m2 = R.run_network('wasm', 8, links, cfg, DT, steps, pushes, record_every=M, check_msgs=steps)
        t2 = time.time()
        diff = 0
        total = 0
        for s in r1:
            for a, b in zip(r1[s], r2[s]):
                for x, y in zip(a, b):
                    total += 1
                    diff += struct.pack('<d', x) != struct.pack('<d', y)
        res[name] = dict(values=total, mismatched=diff, messages_identical=(m1 == m2), n_messages_bytes=sum(map(len, m1)),
                         pushes=sum(len(p) for p in pushes), sec_ref=round(t1 - t0, 1), sec_wasm=round(t2 - t1, 1))
        print('wasm vs ref', name, res[name])
    summary['wasm'] = res


# ---------- 実機と同じ形式の合成記録 ----------

def write_logs(outdir, scn='motion', seed=31, mode=R.TIME_STAMP, offsets=None):
    """imu_raw.csv (生の標本)、imu_fused.csv (アクターの出力)、level.csv (水平器の値)。
    形式は docs/2026-11-13-imu-fusion.md §5"""
    os.makedirs(outdir, exist_ok=True)
    nodes = gen_streams(scn, 8, seed=seed)
    clock_model(nodes, seed, int(scenario_len_nodes(nodes) / DT))
    t_base = 1_763_000_000_000_000  # 同期した時刻の例 (1970年からの µs)
    with open(os.path.join(outdir, 'imu_raw.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['node', 'seq', 'ts_us', 'local_us', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c'])
        for i, nd in enumerate(nodes):
            for j in range(len(nd['t'])):
                v = nd['v'][j]
                w.writerow([i, j, int(nd['ts_us'][j]) + t_base, int(nd['local_us'][j])] +
                           ['%.6f' % x for x in v[:3]] + ['%.7f' % x for x in v[3:]] + ['%.2f' % nd['temp'][j]])
    out, _, n_ep = run_actor(nodes, 'grid', mode=mode, offsets_us=offsets)
    with open(os.path.join(outdir, 'imu_fused.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['node', 'step', 'epoch', 'epoch_end_us'] + ['v%d' % k for k in range(R.G_LEN)])
        for i in range(out.shape[0]):
            for e in range(n_ep):
                if np.isnan(out[i, e, 0]):
                    continue
                w.writerow([i, (e + 2) * M, e, t_base + (e + 1) * int(E_S * 1e6)] + [repr(float(x)) for x in out[i, e]])
    with open(os.path.join(outdir, 'level.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_start_us', 't_end_us', 'roll_deg', 'pitch_deg', 'label'])
        if scn == 'tilt':
            holds, tr = tilt_schedule()
            start = 0.0
            for idx, (ang, dur) in enumerate(holds):
                if idx > 0:
                    start += tr
                w.writerow([t_base + int((start + 8.0) * 1e6), t_base + int((start + dur - 0.5) * 1e6), ang, 0.0,
                            'hold%d' % idx])
                start += dur
        else:
            w.writerow([t_base + int(8e6), t_base + int(18e6), 0.0, 0.0, 'still_before'])
    meta = dict(scenario=scn, seed=seed, epoch_s=E_S, dt_s=DT, epoch_steps=M, t0_us=t_base,
                time_mode=mode, topology='grid 2x4', offsets_us=offsets, synthetic=True)
    with open(os.path.join(outdir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    print('wrote', outdir)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('what', choices=['all', 'static', 'allan', 'tilt', 'motion', 'wasm', 'logs'])
    ap.add_argument('outdir', nargs='?')
    ap.add_argument('--scenario', default='motion', choices=['static', 'tilt', 'motion'])
    ap.add_argument('--cond', default='sync', choices=['sync', 'desync_offset', 'arrival'],
                    help='logs: アクターの割り当て (sync=時刻印、desync_offset=時刻印+わざとずらす、arrival=届いた順)')
    a = ap.parse_args()
    if a.what == 'logs':
        cond = {c[0]: c for c in motion_conditions()}[a.cond]
        write_logs(a.outdir or os.path.join(OUT, 'logs_%s_%s' % (a.scenario, a.cond)), scn=a.scenario,
                   mode=cond[1], offsets=cond[2])
        return
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, 'summary.json')
    summary = {}
    if os.path.exists(path):
        with open(path) as f:
            summary = json.load(f)
    todo = ['static', 'allan', 'tilt', 'motion', 'wasm'] if a.what == 'all' else [a.what]
    for w in todo:
        t0 = time.time()
        globals()['exp_' + w](summary)
        print('== %s: %.1fs' % (w, time.time() - t0))
    summary['params'] = dict(P=P, DT=DT, M=M, CAL=CAL)
    with open(path, 'w') as f:
        json.dump(summary, f, indent=1)
    print('summary ->', path)


if __name__ == '__main__':
    main()

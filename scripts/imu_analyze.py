# 実験9 (IMU の協調によるノイズ除去と時刻同期、G14) の解析: 実機の記録 (または imu_sim.py の合成記録)
# から、台数 N と標準偏差、アラン偏差、水平器に対する傾きの誤差、時刻同期あり/なしの比較を出す。
#
# 出所: Claude Code が 2026-11-13 に作成。入力の形式と表の読み方は docs/2026-11-13-imu-fusion.md §5・§7。
# 使い方 (リポジトリ直下で):
#   .venv\Scripts\python.exe scripts\imu_analyze.py <記録のディレクトリ> [<比べる記録のディレクトリ> ...]
#       [--static A,B] [--replay 300] [--topo grid] [--out 出力先]
# 記録のディレクトリ: imu_raw.csv (必須)、imu_fused.csv・level.csv・meta.json (あれば使う)。
# 出力: <最初のディレクトリ>/analysis/ に summary.json と図。表は画面に Markdown で出す。
#
# 基準: 時刻同期した時刻印で全ノードの標本を区間ごとに PC で平均したもの (「中央の融合」)。
# 動いている間の真値は無いので、時刻同期なしの条件の誤差はこの基準との差で測る (同じ記録の生の標本は
# 同期した時刻印つきなので、アクターの入力をずらしても基準は正しいまま)。
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import imu_fusion_ref as R  # noqa: E402
from imu_sim import allan_dev, plt_setup, COLORS  # noqa: E402

D2R = math.pi / 180.0


def load_raw(path):
    d = np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding='utf-8',
                      usecols=(0, 1, 2, 4, 5, 6, 7, 8, 9))
    nodes = {}
    ids = np.unique(d['node'])
    for i in ids:
        # 同期した時刻が 0 (未同期) の標本は使わない (数は health の unsynced に出す)
        m = (d['node'] == i) & (d['ts_us'] > 0)
        ts = d['ts_us'][m].astype(np.int64)
        o = np.argsort(ts, kind='stable')
        v = np.stack([d[k][m] for k in ('ax', 'ay', 'az', 'gx', 'gy', 'gz')], axis=1)[o]
        nodes[int(i)] = dict(ts=ts[o], seq=d['seq'][m][o].astype(np.int64), v=v,
                             unsynced=int(np.sum((d['node'] == i) & (d['ts_us'] <= 0))))
    return nodes


def load_fused(path):
    if not os.path.exists(path):
        return None
    d = np.genfromtxt(path, delimiter=',', skip_header=1)
    if d.ndim == 1:
        d = d[None]
    out = {}
    for i in np.unique(d[:, 0]).astype(int):
        m = d[:, 0] == i
        rows = d[m]
        out[i] = dict(epoch=rows[:, 2].astype(np.int64), v=rows[:, 4:4 + R.G_LEN])
    return out


def load_level(path):
    if not os.path.exists(path):
        return []
    rows = []
    with open(path, encoding='utf-8') as f:
        next(f)
        for line in f:
            p = line.strip().split(',')
            if len(p) >= 4 and p[0]:
                rows.append(dict(t0=int(p[0]), t1=int(p[1]), roll=float(p[2]), pitch=float(p[3]),
                                 label=p[4] if len(p) > 4 else ''))
    return rows


def epoch_index(ts, t0, E_us):
    return (ts - t0 - 1) // E_us


def epoch_sums(nodes, ids, t0, E_us, e_lo, e_hi):
    """区間 e_lo..e_hi−1 の (ノードごとの平均の和 (n_ep,6), 標本のあったノードの数 (n_ep))。
    S / n がノードごとに等しい重みの平均 (アクターと同じ重み)"""
    n_ep = e_hi - e_lo
    S = np.zeros((n_ep, 6))
    n = np.zeros(n_ep)
    for i in ids:
        nd = nodes[i]
        e = epoch_index(nd['ts'], t0, E_us) - e_lo
        m = (e >= 0) & (e < n_ep)
        ni = np.bincount(e[m], minlength=n_ep).astype(float)
        ok = ni > 0
        n += ok
        for k in range(6):
            Si = np.bincount(e[m], weights=nd['v'][m, k], minlength=n_ep)
            S[ok, k] += Si[ok] / ni[ok]
    return S, n


def load_batches(path):
    """imu_log.py の imu_batches.csv: ノードごとの (バッチの通番の飛び, FIFO のあふれの最大)"""
    out = {}
    if not os.path.exists(path):
        return out
    d = np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding='utf-8')
    for i in np.unique(d['node']):
        b = np.sort(d['batch'][d['node'] == i].astype(np.int64))
        ovr = d['ovr'][d['node'] == i]
        out[int(i)] = dict(batch_gaps=int(np.sum(np.diff(b) - 1)) if len(b) > 1 else 0,
                           fifo_ovr=int(np.nanmax(ovr)) if len(ovr) else 0)
    return out


def health(nodes, batches=None):
    res = {}
    for i, nd in nodes.items():
        dts = np.diff(nd['ts'])
        gaps = int(np.sum(np.diff(nd['seq']) != 1))
        res[i] = dict(samples=int(len(nd['ts'])), odr_hz=float(1e6 / np.median(dts)) if len(dts) else 0.0,
                      seq_gaps=gaps, ts_nonincreasing=int(np.sum(dts <= 0)),
                      dur_s=float((nd['ts'][-1] - nd['ts'][0]) / 1e6), unsynced=nd['unsynced'])
        if batches and i in batches:
            res[i].update(batches[i])
    return res


def detect_motion(cen, thr_g=0.03, thr_a=0.2, pad=10):
    """区間ごとに動いているか: 中央の平均の角速度の大きさ (ゼロ点のずれ 0.3°/s 程度より十分大きい
    0.03 rad/s)、または加速度の大きさの 1g からの差。前後 pad 区間に広げる (ゆっくり往復する動きの
    折り返しで角速度が一瞬0になる所を静止と誤らないため)"""
    gm = np.linalg.norm(cen[:, 3:], axis=1)
    am = np.abs(np.linalg.norm(cen[:, :3], axis=1) - 9.80665)
    mv = np.nan_to_num((gm > thr_g) | (am > thr_a)).astype(bool)
    out = mv.copy()
    for k in range(1, pad + 1):
        out[k:] |= mv[:-k]
        out[:-k] |= mv[k:]
    return out


def longest_run(mask):
    """True が最も長く続く範囲 (始め, 終わり+1)"""
    best = (0, 0)
    start = None
    for i, v in enumerate(list(mask) + [False]):
        if v and start is None:
            start = i
        elif not v and start is not None:
            if i - start > best[1] - best[0]:
                best = (start, i)
            start = None
    return best


def replay(nodes, ids, topo, t0, E_us, dt, M, t_lo, t_hi, rows=2):
    """記録した標本をアクターの参照モデル (実物とビット一致) に時刻印の割り当てで入れ直し、
    隣だけの融合の出力を得る ((n, n_ep, G_LEN)、区間番号の始まり)"""
    n = len(ids)
    links = R.topology(topo if n > 2 or topo == 'line' else 'line', n, rows=rows)
    e0 = (t_lo - t0) // E_us
    steps = int(((t_hi - t0) // E_us - e0) * M)
    t0r = t0 + e0 * E_us
    pushes = []
    for i in ids:
        nd = nodes[i]
        m = (nd['ts'] > t0r) & (nd['ts'] <= t0r + steps // M * E_us)
        ts = nd['ts'][m]
        e = (ts - t0r - 1) // E_us
        c = (e + 1) * M - 1
        pushes.append([(int(a), int(b), tuple(float(x) for x in v)) for a, b, v in zip(c, ts - t0r, nd['v'][m])])
    cfg = {R.CFG_EPOCH_STEPS: M, R.CFG_TIME_MODE: R.TIME_STAMP}
    rec, _ = R.run_network('ref', n, links, cfg, dt, steps, pushes, record_every=M)
    n_ep = steps // M
    out = np.full((n, n_ep, R.G_LEN), np.nan)
    for s, vals in rec.items():
        e = s // M - 2
        if e >= 0:
            out[:, e] = np.array(vals)
    return out, e0


def analyze(dirs, a):
    plt = plt_setup()
    main = dirs[0]
    outdir = a.out or os.path.join(main, 'analysis')
    os.makedirs(outdir, exist_ok=True)
    meta = {}
    if os.path.exists(os.path.join(main, 'meta.json')):
        with open(os.path.join(main, 'meta.json')) as f:
            meta = json.load(f)
    E_s = meta.get('epoch_s', a.epoch)
    M = meta.get('epoch_steps', a.m)
    dt = E_s / M
    E_us = int(dt * 1e6 + 0.5) * M
    nodes = load_raw(os.path.join(main, 'imu_raw.csv'))
    ids = sorted(nodes)
    t_first = min(nd['ts'][0] for nd in nodes.values())
    t_last = min(nd['ts'][-1] for nd in nodes.values())
    t0 = meta.get('t0_us', (t_first // E_us) * E_us)
    summary = dict(dirs=dirs, health=health(nodes, load_batches(os.path.join(main, 'imu_batches.csv'))), epoch_s=E_s)
    print('## 記録の健全性')
    print('| node | samples | ODR [Hz] | seq の飛び | バッチの飛び | FIFO あふれ | 未同期の標本 | 時刻印の逆行 | 長さ [s] |')
    print('|---|---|---|---|---|---|---|---|---|')
    for i, h in summary['health'].items():
        print('| %d | %d | %.2f | %d | %s | %s | %d | %d | %.1f |' % (
            i, h['samples'], h['odr_hz'], h['seq_gaps'], h.get('batch_gaps', '-'), h.get('fifo_ovr', '-'),
            h['unsynced'], h['ts_nonincreasing'], h['dur_s']))

    e_lo = epoch_index(t_first, t0, E_us) + 1
    e_hi = epoch_index(t_last, t0, E_us)
    S, n = epoch_sums(nodes, ids, t0, E_us, e_lo, e_hi)
    with np.errstate(invalid='ignore'):
        cen = S / n[:, None]
    tcen = (np.arange(e_lo, e_hi) + 0.5) * E_us + t0
    moving = detect_motion(cen)

    # ---- ① 静止: 台数と標準偏差 ----
    if a.static:
        lo, hi = [float(x) for x in a.static.split(',')]
        sm = (tcen >= t_first + lo * 1e6) & (tcen <= t_first + hi * 1e6)
    else:
        # 動きの検出で静止とされた区間のうち、最も長く続く範囲 (アラン偏差にも同じ範囲を使う)
        r0, r1 = longest_run(~moving)
        sm = np.zeros_like(moving)
        sm[r0:r1] = True
        lo = (tcen[r0] - E_us / 2 - t_first) / 1e6 if r1 > r0 else 0.0
        hi = (tcen[r1 - 1] + E_us / 2 - t_first) / 1e6 if r1 > r0 else 0.0
        print('\n静止の範囲 (動きの検出): 最初の標本から %.1f〜%.1f s' % (lo, hi))
    summary['static_range_s'] = [lo, hi]
    Ns = [k for k in (1, 2, 4, 8, 16) if k <= len(ids)]
    if len(ids) not in Ns:
        Ns.append(len(ids))
    rows = []
    for N in Ns:
        Sx, nx = epoch_sums(nodes, ids[:N], t0, E_us, e_lo, e_hi)
        with np.errstate(invalid='ignore'):
            c = (Sx / nx[:, None])[sm]
        rows.append(dict(N=N, epochs=int(sm.sum()),
                         std=[float(np.nanstd(c[:, k])) for k in range(6)],
                         short_std=[float(np.nanstd(np.diff(c[:, k])) / math.sqrt(2)) for k in range(6)]))
    summary['static_std_central'] = rows
    print('\n## ① 静止: 区間 %.2fs の平均の標準偏差 (中央の融合、先頭 N 台)' % E_s)
    print('| N | 区間数 | ax | ay | az [m/s²] | gx | gy | gz [rad/s] | az の短時間 | az の 1/√N の目安 |\n|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        print('| %d | %d | %s | %.2e | %.2e |' % (r['N'], r['epochs'], ' | '.join('%.2e' % x for x in r['std']),
                                               r['short_std'][2], rows[0]['short_std'][2] / math.sqrt(r['N'])))

    # 隣だけの融合 (参照モデルで再生)
    if a.replay > 0 and sm.any():
        idx = np.where(sm)[0]
        t_lo = int(tcen[idx[0]] - E_us // 2)
        t_hi = int(min(tcen[idx[-1]], t_lo + a.replay * 1e6))
        rep_rows = []
        for N in Ns:
            out, e0 = replay(nodes, ids[:N], a.topo, t0, E_us, dt, M, t_lo, t_hi)
            Sx, nx = epoch_sums(nodes, ids[:N], t0, E_us, e0, e0 + out.shape[1])
            with np.errstate(invalid='ignore'):
                cx = Sx / nx[:, None]
            f = out[:, 2:-1, R.G_FUSED:R.G_FUSED + 6]
            d = f - cx[None, 2:-1]
            rep_rows.append(dict(N=N, std_fused=[float(np.nanmean(np.nanstd(f[:, :, k], axis=1))) for k in range(6)],
                                 rms_minus_central=[float(np.sqrt(np.nanmean(d[:, :, k] ** 2))) for k in range(6)]))
        summary['static_replay_neighbour_only'] = dict(topo=a.topo, seconds=(t_hi - t_lo) / 1e6, rows=rep_rows)
        print('\n## ① 隣だけの融合 (記録の再生、%s、%.0fs): 標準偏差と中央の融合との差' % (a.topo, (t_hi - t_lo) / 1e6))
        print('| N | az の標準偏差 | gx の標準偏差 | az の中央との差 (RMS) | gx の中央との差 (RMS) |\n|---|---|---|---|---|')
        for r in rep_rows:
            print('| %d | %.2e | %.2e | %.1e | %.1e |' % (r['N'], r['std_fused'][2], r['std_fused'][3],
                                                       r['rms_minus_central'][2], r['rms_minus_central'][3]))

    # アラン偏差 (10ms の箱で揃えた中央の平均)
    bin_us = a.allan_bin_us
    fs = 1e6 / bin_us
    b_lo = (t_first - t0) // bin_us + 1
    b_hi = (t_last - t0) // bin_us
    nb = int(b_hi - b_lo)
    sm_bins = slice(int(lo * fs), int(hi * fs))
    allan = {}
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.6))
    for j, N in enumerate(Ns):
        Sb = np.zeros((nb, 6))
        nbn = np.zeros(nb)
        for i in ids[:N]:
            nd = nodes[i]
            b = (nd['ts'] - t0 - 1) // bin_us - b_lo
            m = (b >= 0) & (b < nb)
            nbn += np.bincount(b[m], minlength=nb)
            for k in range(6):
                Sb[:, k] += np.bincount(b[m], weights=nd['v'][m, k], minlength=nb)
        with np.errstate(invalid='ignore'):
            x = Sb / nbn[:, None]
        if sm_bins is not None:
            x = x[sm_bins]
        # 空の箱は直前の値で埋める (標本の抜けは health で確認)
        for k in range(6):
            col = x[:, k]
            bad = np.isnan(col)
            if bad.any() and (~bad).any():
                idxs = np.where(~bad, np.arange(len(col)), 0)
                np.maximum.accumulate(idxs, out=idxs)
                col[:] = col[idxs]
        allan['N%d' % N] = {}
        for k, name in enumerate(('ax', 'ay', 'az', 'gx', 'gy', 'gz')):
            tau, ad = allan_dev(x[:, k], fs)
            i1 = int(np.argmin(np.abs(tau - 1.0)))
            allan['N%d' % N][name] = dict(adev_1s=float(ad[i1]), min=float(ad.min()), tau_min=float(tau[np.argmin(ad)]))
            if name in ('az', 'gx'):
                axx = axes[0 if name == 'az' else 1]
                axx.loglog(tau, ad, color=COLORS[j % len(COLORS)], label='N=%d' % N)
                axx.set_xlabel('τ [s]')
                axx.set_ylabel('Allan dev. of %s [%s]' % (name, 'm/s²' if name == 'az' else 'rad/s'))
    axes[0].legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, 'allan.png'))
    plt.close(fig)
    summary['allan_central'] = allan
    print('\n## ① アラン偏差 (中央の平均、箱 %.0fms)' % (bin_us / 1000))
    print('| N | az: τ=1s | az: 最小 (τ) | gx: τ=1s [rad/s] | gx: 最小 (τ) |\n|---|---|---|---|---|')
    for N in Ns:
        q = allan['N%d' % N]
        print('| %d | %.2e | %.2e (%.0fs) | %.2e | %.2e (%.0fs) |' % (N, q['az']['adev_1s'], q['az']['min'], q['az']['tau_min'],
                                                                   q['gx']['adev_1s'], q['gx']['min'], q['gx']['tau_min']))

    # ---- ② 傾き: 水平器との差 ----
    level = load_level(os.path.join(main, 'level.csv'))
    fused = load_fused(os.path.join(main, 'imu_fused.csv'))
    if level:
        tilt_rows = []
        for lv in level:
            r = dict(label=lv['label'], level_roll=lv['roll'], level_pitch=lv['pitch'])
            for N in Ns:
                tot = np.zeros(6)
                cnt = 0
                for i in ids[:N]:
                    nd = nodes[i]
                    m = (nd['ts'] > lv['t0']) & (nd['ts'] <= lv['t1'])
                    tot += nd['v'][m].sum(axis=0)
                    cnt += int(m.sum())
                mv = tot / max(cnt, 1)
                r['roll_N%d' % N] = math.degrees(math.atan2(mv[1], mv[2]))
                r['pitch_N%d' % N] = math.degrees(math.atan2(-mv[0], math.hypot(mv[1], mv[2])))
            if fused:
                vals = []
                for i, fd in fused.items():
                    ee = fd['epoch']
                    t_end = t0 + (ee + 1) * E_us
                    m = (t_end - E_us >= lv['t0']) & (t_end <= lv['t1'])
                    if m.any():
                        vals.append(fd['v'][m][:, [R.G_FUSED_ROLL, R.G_FUSED_PITCH]].mean(axis=0))
                if vals:
                    vv = np.mean(vals, axis=0)
                    r['actor_roll'] = math.degrees(vv[0])
                    r['actor_pitch'] = math.degrees(vv[1])
            tilt_rows.append(r)
        z = next((r for r in tilt_rows if r['level_roll'] == 0.0 and r['level_pitch'] == 0.0), None)
        print('\n## ② 傾き: 推定 − 水平器 [度] (加速度の平均から。括弧内は 0° の行で零点を合わせた後)')
        print('| 区間 | 水平器 roll | ' + ' | '.join('N=%d' % N for N in Ns) + ' | アクター (相補フィルタ) |\n|---|---|' + '---|' * (len(Ns) + 1))
        for r in tilt_rows:
            cells = []
            for N in Ns:
                e = r['roll_N%d' % N] - r['level_roll']
                ez = e - (z['roll_N%d' % N] - z['level_roll']) if z else float('nan')
                r['err_roll_N%d' % N] = e
                r['err_roll_zero_N%d' % N] = ez
                cells.append('%+.3f (%+.3f)' % (e, ez))
            act = '%+.3f' % (r['actor_roll'] - r['level_roll']) if 'actor_roll' in r else '—'
            print('| %s | %.2f | %s | %s |' % (r['label'], r['level_roll'], ' | '.join(cells), act))
        summary['tilt'] = tilt_rows

    # ---- ③ 同期あり/なし: アクターの融合と、同期した時刻印による中央の融合の差 ----
    comp = []
    for d in dirs:
        fu = load_fused(os.path.join(d, 'imu_fused.csv'))
        if not fu:
            continue
        mt = {}
        if os.path.exists(os.path.join(d, 'meta.json')):
            with open(os.path.join(d, 'meta.json')) as f:
                mt = json.load(f)
        nd_ = load_raw(os.path.join(d, 'imu_raw.csv')) if d != main else nodes
        ids_ = sorted(nd_)
        tf = min(x['ts'][0] for x in nd_.values())
        tl = min(x['ts'][-1] for x in nd_.values())
        t0d = mt.get('t0_us', (tf // E_us) * E_us)
        el = epoch_index(tf, t0d, E_us) + 1
        eh = epoch_index(tl, t0d, E_us)
        Sd, ndn = epoch_sums(nd_, ids_, t0d, E_us, el, eh)
        with np.errstate(invalid='ignore'):
            cd = Sd / ndn[:, None]
        mv = detect_motion(cd)
        errs = {'still': [], 'moving': []}
        for i, fd in fu.items():
            ee = fd['epoch'] - el
            ok = (ee >= 0) & (ee < len(cd))
            dlt = fd['v'][ok][:, R.G_FUSED:R.G_FUSED + 6] - cd[ee[ok]]
            mm = mv[ee[ok]]
            errs['moving'].append(dlt[mm])
            errs['still'].append(dlt[~mm])
        row = dict(dir=d, time_mode=mt.get('time_mode'), offsets_us=mt.get('offsets_us'))
        for seg in ('still', 'moving'):
            x = np.concatenate(errs[seg]) if errs[seg] else np.zeros((0, 6))
            row[seg] = dict(epochs=int(len(x)), rms=[float(np.sqrt(np.mean(x[:, k] ** 2))) if len(x) else float('nan')
                                                      for k in range(6)])
        comp.append(row)
    if comp:
        summary['sync_compare'] = comp
        print('\n## ③ アクターの融合 − 同期した時刻印での中央の融合 (RMS)')
        print('| 記録 | 割り当て | 静止 ay | 動き ay [m/s²] | 静止 gx | 動き gx [rad/s] |\n|---|---|---|---|---|---|')
        for r in comp:
            print('| %s | %s | %.2e | %.2e | %.2e | %.2e |' % (os.path.basename(os.path.normpath(r['dir'])), {0: '時刻印', 1: '届いた順'}.get(r['time_mode'], r['time_mode']),
                                                             r['still']['rms'][1], r['moving']['rms'][1],
                                                             r['still']['rms'][3], r['moving']['rms'][3]))
        fig, ax = plt.subplots(figsize=(6.5, 3.2))
        x = np.arange(len(comp))
        ax.bar(x - 0.18, [r['still']['rms'][1] for r in comp], 0.34, color=COLORS[0], label='still')
        ax.bar(x + 0.18, [r['moving']['rms'][1] for r in comp], 0.34, color=COLORS[1], label='moving')
        ax.set_xticks(x, [os.path.basename(os.path.normpath(r['dir'])) for r in comp], fontsize=8)
        ax.set_ylabel('RMS(fused ay − synced central) [m/s²]')
        ax.legend(frameon=False)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, 'sync_compare.png'))
        plt.close(fig)

    with open(os.path.join(outdir, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=1, ensure_ascii=False, default=float)
    print('\nsummary ->', os.path.join(outdir, 'summary.json'))


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--static', help='静止とみなす範囲 (最初の標本からの秒) "A,B"。省くと動きの検出で決める')
    ap.add_argument('--replay', type=float, default=120.0, help='隣だけの融合を再生する長さ [s] (0 で省く)')
    ap.add_argument('--topo', default='grid', choices=['line', 'ring', 'grid'])
    ap.add_argument('--epoch', type=float, default=0.2, help='meta.json が無いときの区間の長さ [s]')
    ap.add_argument('--m', type=int, default=10, help='meta.json が無いときの1区間のステップ数')
    ap.add_argument('--allan-bin-us', type=int, default=10000)
    ap.add_argument('--out')
    a = ap.parse_args()
    analyze(a.dirs, a)


if __name__ == '__main__':
    main()

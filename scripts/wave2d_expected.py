# 2次元の波動方程式を実機のノード群で解く実験(実験10)の照合用: シミュレータで各ノードの
# 期待値(区画の状態の CRC32・最小・最大)を作り、ノードごとの設定注入の下書きを出す。
#
# 出所: Claude Code が 2026-11-11 に作成。記録は docs/2026-11-11-wave2d.md の §7。
# 実機のファーム (node-esp32/main/node_v2.c) は完了時と field_every ごとに、区画の全点を
# dyne_get の順 (k = j·nx + i) に並べた f64 のバイト列の CRC32 と最小・最大を発行する。
# 本スクリプトはシミュレータの最後のステップの場 (--field-csv) から同じ量を区画ごとに計算する。
#
# 使い方(リポジトリ直下で):
#   .venv\Scripts\python.exe scripts\wave2d_expected.py [dyne-sim2d.exe] [--json 出力先ディレクトリ]
import json
import os
import struct
import subprocess
import sys
import tempfile
import zlib

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
# フル版を使う: dyne-sim2d は同じ wasm で全域1ノードの解も作って照合するが、81×81点は縮小版
# (1辺48点まで) に入らない。縮小版とフル版は配列の大きさ以外同じコードで、同じ条件の結果が
# ビット一致することを scripts\check-wave2d.ps1 の [4] で確かめている
WASM = os.path.join(ROOT, 'crates', 'wave2d-actor', 'target', 'wasm32-unknown-unknown', 'release',
                    'wave2d_actor.wasm')

# 実験10の場面: 81×81点 (h = 1/80)、c = 1、ν = 0.5 (dt = 1/160)、400ステップ (t = 2.5)。
# 4台 (2×2)・8台 (4×2)・16台 (4×4) で同じ全域の問題を解く (区画は縮小版の上限1辺48点に収まる)
N = 81
H = '0.0125'
DT = '0.00625'
STEPS = 400
SCENES = {
    'gauss_dirichlet': {'outer': [], 'cfg': [(0, 1.0), (4, 0.3), (5, 0.6), (6, 0.06)]},
    'gauss_absorb': {'outer': ['--outer-bc', 'zero_grad'], 'cfg': [(0, 1.0), (4, 0.3), (5, 0.6), (6, 0.06), (7, 15.0)]},
}
LAYOUTS = [(2, 2), (4, 2), (4, 4)]


def split(n, p):
    """dyne-sim2d の split と同じ (余りは先頭のブロックから1点ずつ)。"""
    base, extra = divmod(n, p)
    out, idx = [], 0
    for i in range(p):
        cnt = base + (1 if i < extra else 0)
        out.append((idx, cnt))
        idx += cnt
    return out


def faces(px, py, bx, by):
    """面 (W, E, S, N) ごとに (界面ID, 相手ノード) か None。dyne-sim2d の build_blocks と同じ規則。"""
    v_count = (px - 1) * py
    node = by * px + bx
    f = [None] * 4
    if bx > 0:
        f[0] = (by * (px - 1) + bx - 1, node - 1)
    if bx + 1 < px:
        f[1] = (by * (px - 1) + bx, node + 1)
    if by > 0:
        f[2] = (v_count + (by - 1) * px + bx, node - px)
    if by + 1 < py:
        f[3] = (v_count + by * px + bx, node + px)
    return f


def run_sim(exe, px, py, scene, csv_path):
    sc = SCENES[scene]
    cmd = [exe, '--wasm', WASM, '--px', str(px), '--py', str(py), '--nx', str(N), '--ny', str(N),
           '--dt', DT, '--steps', str(STEPS), '--quiet', '--field-csv', csv_path] + sc['outer']
    for k, v in sc['cfg']:
        cmd += ['--cfg', f'{k}={v}']
    out = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8').stdout
    assert 'PASS (bit-exact)' in out, out
    crc_line = [l for l in out.splitlines() if l.startswith('crc32')][0]
    return crc_line


def read_field(csv_path):
    u = [0.0] * (N * N)
    with open(csv_path, encoding='utf-8') as f:
        next(f)
        for line in f:
            gx, gy, _, _, us, _ = line.strip().split(',')
            u[int(gy) * N + int(gx)] = float(us)  # Rust の {:e} は往復で同じ値に戻る最短表記
    return u


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    args = sys.argv[1:]
    json_dir = None
    if '--json' in args:
        i = args.index('--json')
        json_dir = args[i + 1]
        del args[i:i + 2]
    exe = args[0] if args else os.path.join(ROOT, 'target', 'release', 'dyne-sim2d.exe')
    tmp = tempfile.mkdtemp(prefix='dyne-wave2d-expected-')
    print(f'# 場面: {N}x{N}点, h = {H}, c = 1, dt = {DT} (クーラン数 0.5), {STEPS}ステップ')
    for scene in SCENES:
        for px, py in LAYOUTS:
            csv_path = os.path.join(tmp, f'{scene}_{px}x{py}.csv')
            crc_line = run_sim(exe, px, py, scene, csv_path)
            u = read_field(csv_path)
            print(f'\n## {scene} {px}x{py} ({px * py}台)  全域 {crc_line}')
            print('node  ix0 iy0  nx ny   crc32       min                     max                     界面の1通(B)')
            xs, ys = split(N, px), split(N, py)
            for by in range(py):
                for bx in range(px):
                    node = by * px + bx
                    (ix0, nx), (iy0, ny) = xs[bx], ys[by]
                    vals = [u[(iy0 + j) * N + ix0 + i] for j in range(ny) for i in range(nx)]
                    b = b''.join(struct.pack('<d', v) for v in vals)
                    crc = zlib.crc32(b) & 0xFFFFFFFF
                    fc = faces(px, py, bx, by)
                    sizes = []
                    for fi, f in enumerate(fc):
                        if f is not None:
                            npts = ny if fi < 2 else nx
                            sizes.append(36 + 4 + 24 + 8 + 8 * npts)  # 外箱36 + 枠4 + ヘッダ24 + 個数8 + 8·点数
                    print(f'{node:>4d}  {ix0:>3d} {iy0:>3d} {nx:>3d} {ny:>2d}   0x{crc:08X}  {min(vals):<23.17g} '
                          f'{max(vals):<23.17g} {max(sizes) if sizes else 0}')
                    if json_dir:
                        os.makedirs(json_dir, exist_ok=True)
                        sc = SCENES[scene]
                        outer_key = 'zero_grad' if '--outer-bc' in sc['outer'] else 'dirichlet'
                        fl = []
                        for fi, f in enumerate(fc):
                            name = 'wesn'[fi]
                            if f is None:
                                fl.append({'face': name, 'key': outer_key, 'value': 0.0})
                            else:
                                fl.append({'face': name, 'key': 'interface', 'iface': f[0]})
                        cfgj = {
                            'gen': 1,
                            'cluster': {'steps': STEPS, 'alpha': 1.0, 'dt': float(DT), 'step_period_ms': 0},
                            'node': {'idx': node},
                            'field2d': {'nx': nx, 'ny': ny, 'ix0': ix0, 'iy0': iy0, 'h': float(H), 'faces': fl,
                                        'cfg': [[k, v] for k, v in sc['cfg']]},
                            'ifaces': [{'id': f[0], 'peer': f'<node {f[1]} の ip:port>'} for f in fc if f is not None],
                            'mode': {'field_every': 100, 'rt_every': 100},
                            '_expected': {'crc32': f'0x{crc:08X}', 'min': min(vals), 'max': max(vals)},
                        }
                        p = os.path.join(json_dir, f'{scene}_{px}x{py}_node{node:02d}.json')
                        with open(p, 'w', encoding='utf-8') as fo:
                            json.dump(cfgj, fo, ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()

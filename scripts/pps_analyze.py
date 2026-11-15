#!/usr/bin/env python3
"""ロジックアナライザの記録から、ノード間の時計のずれを求める (実験1、2026-11-11)。

各ノードは試験モードで「時計の整数秒ちょうど」にピンを切り替える (偶数秒で高、奇数秒で低)。
同じ時刻に起きるはずの切り替わり (エッジ) の時刻の差が、ノード間の時計のずれである。
基準はロジックアナライザ自身の時計 (全チャンネル共通) なので、ノード間の差だけを見る。
説明: docs/2026-11-11-timesync-ota.md

入力: PulseView の書き出し
  - VCD (推奨。変化した時刻だけが入るので長い記録でも小さい): ファイル → Export → Value Change Dump
  - CSV (sigrok の CSV 形式。';' で始まる行は注記。見出しに Time 列があればそれを使い、
    無ければ --samplerate で行番号から時刻を作る)
標準ライブラリだけで動く。

使い方 (リポジトリのルートで):
  python scripts\\pps_analyze.py rec.vcd --names D0=1,D1=2,D2=3 --out skew.csv
  python scripts\\pps_analyze.py rec.csv --samplerate 1e6
  python scripts\\pps_analyze.py --selftest          合成した記録で解析の正しさを確かめる

出力:
  - 1秒ごと (切り替わりのまとまりごと) の、各チャンネルの「全チャンネル平均との差」と最大差 (広がり)
  - 広がりの統計 (中央値・95%点・99%点・最大)、チャンネルごとの平均のずれとばらつき
  - チャンネルごとの切り替わりの間隔の平均 → 1秒からのずれ [ppm] (ロジックアナライザの時計が基準)
  - 同期なしの記録 (ずれが 0.25 秒を超えうる) では --mode phase で、基準チャンネルとの位相差の
    時間変化 (ドリフトの差 [ppm]) を出す
"""
import argparse
import csv
import math
import random
import statistics
import sys


# ==== 読み込み ====

def read_vcd(path):
    """返り値: {名前: [(時刻 s, 新しい値)]}"""
    scale = 1e-9
    ids = {}
    edges = {}
    last = {}
    t = 0.0
    in_defs = True
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        data = f.read()
    toks = data.split()
    i = 0
    while i < len(toks):
        tk = toks[i]
        if in_defs:
            if tk == "$timescale":
                j = i + 1
                s = ""
                while toks[j] != "$end":
                    s += toks[j]
                    j += 1
                num = "".join(c for c in s if c.isdigit() or c == ".") or "1"
                unit = s[len(num):] if s.startswith(num) else "".join(c for c in s if c.isalpha())
                scale = float(num) * {"s": 1, "ms": 1e-3, "us": 1e-6, "ns": 1e-9, "ps": 1e-12, "fs": 1e-15}[unit]
                i = j + 1
                continue
            if tk == "$var":
                # $var wire 1 <id> <name> $end
                ident, name = toks[i + 3], toks[i + 4]
                ids[ident] = name
                edges[name] = []
                j = i + 5
                while toks[j] != "$end":
                    j += 1
                i = j + 1
                continue
            if tk == "$enddefinitions":
                in_defs = False
                while toks[i] != "$end":
                    i += 1
                i += 1
                continue
            i += 1
            continue
        if tk.startswith("#"):
            t = int(tk[1:]) * scale
        elif tk[0] in "01xXzZ" and len(tk) > 1 and tk[1:] in ids:
            name = ids[tk[1:]]
            v = 1 if tk[0] == "1" else 0
            if name in last and last[name] != v:
                edges[name].append((t, v))
            last[name] = v
        elif tk.startswith("$"):
            # $dumpvars などの区切り。中の値の変化は上で扱う
            pass
        i += 1
    return edges


def read_csv(path, samplerate=None):
    edges, last, names = {}, {}, None
    tcol = None
    row_i = 0
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as f:
        for row in csv.reader(f):
            if not row or row[0].startswith(";") or row[0].startswith("#"):
                continue
            if names is None:
                try:
                    [float(x) for x in row]
                    names = [f"D{k}" for k in range(len(row))]
                    if samplerate is None:
                        raise SystemExit("見出しの無い CSV には --samplerate が要る")
                except ValueError:
                    names = [c.strip() for c in row]
                    for k, n in enumerate(names):
                        if "time" in n.lower():
                            tcol = k
                    if tcol is None and samplerate is None:
                        raise SystemExit("時刻の列が無い CSV には --samplerate が要る")
                    for k, n in enumerate(names):
                        if k != tcol:
                            edges[n] = []
                    continue
                for n in names:
                    edges[n] = []
            t = float(row[tcol]) if tcol is not None else row_i / samplerate
            row_i += 1
            for k, n in enumerate(names):
                if k == tcol or k >= len(row):
                    continue
                v = 1 if row[k].strip() in ("1", "1.0") else 0
                if n in last and last[n] != v:
                    edges[n].append((t, v))
                last[n] = v
    return edges


# ==== 解析 ====

def pct(xs, p):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    k = (len(xs) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def cluster(edges, gap=0.25):
    """全チャンネルの切り替わりを時刻順に並べ、gap 秒より近いものを同じ「1秒」にまとめる"""
    allv = sorted((t, ch, v) for ch, es in edges.items() for (t, v) in es)
    groups, cur = [], []
    for e in allv:
        if cur and e[0] - cur[0][0] > gap:
            groups.append(cur)
            cur = []
        cur.append(e)
    if cur:
        groups.append(cur)
    return groups


def analyze_sync(edges, names, out=None):
    chans = [c for c in edges if edges[c]]
    if len(chans) < 2:
        print(f"切り替わりのあるチャンネルが {len(chans)} 本しかない (2本以上必要)")
        return 1
    groups = cluster({c: edges[c] for c in chans})
    rows, spreads, dev = [], [], {c: [] for c in chans}
    dup = level_mismatch = 0
    complete = 0
    for g in groups:
        by = {}
        for (t, ch, v) in g:
            if ch in by:
                dup += 1
                continue
            by[ch] = (t, v)
        if len({v for (_, v) in by.values()}) > 1:
            level_mismatch += 1
        if len(by) < 2:
            continue
        ts = [t for (t, _) in by.values()]
        mean = sum(ts) / len(ts)
        spread = max(ts) - min(ts)
        if len(by) == len(chans):
            complete += 1
        spreads.append(spread)
        r = {"t_s": f"{mean:.6f}", "n": len(by), "spread_us": f"{spread * 1e6:.1f}"}
        for c in chans:
            if c in by:
                d = (by[c][0] - mean) * 1e6
                dev[c].append(d)
                r[f"{names.get(c, c)}_us"] = f"{d:.1f}"
            else:
                r[f"{names.get(c, c)}_us"] = ""
        rows.append(r)
    sp = [s * 1e6 for s in spreads]
    print(f"チャンネル {len(chans)} 本、1秒のまとまり {len(groups)} 個 (全チャンネルそろい {complete} 個)")
    print(f"ノード間の最大差 (広がり) [us]: 中央値 {pct(sp, .5):.1f} / 95% {pct(sp, .95):.1f} / "
          f"99% {pct(sp, .99):.1f} / 最大 {max(sp):.1f} / 平均 {sum(sp) / len(sp):.1f}")
    if dup:
        print(f"注意: 同じまとまりに同じチャンネルの切り替わりが2回以上 ({dup} 件)。ずれが 0.25 秒を超えているなら --mode phase を使う")
    if level_mismatch:
        print(f"注意: 切り替わりの向き (高/低) が食い違うまとまり {level_mismatch} 個 = 整数秒の番号がずれているノードがある")
    print(f"{'ch':<8} {'node':<6} {'n':>5} {'平均の差us':>11} {'標準偏差us':>11} {'最大|差|us':>11} {'間隔の誤差ppm':>14}")
    for c in chans:
        d = dev[c]
        es = edges[c]
        per = [(es[k + 1][0] - es[k][0]) for k in range(len(es) - 1) if 0.5 < es[k + 1][0] - es[k][0] < 1.5]
        ppm = (statistics.mean(per) - 1.0) * 1e6 if per else float("nan")
        print(f"{c:<8} {names.get(c, ''):<6} {len(d):>5} {statistics.mean(d) if d else float('nan'):>11.1f} "
              f"{statistics.pstdev(d) if len(d) > 1 else 0:>11.1f} {max(abs(x) for x in d) if d else float('nan'):>11.1f} {ppm:>14.2f}")
    if out:
        with open(out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["t_s", "n", "spread_us"] + [f"{names.get(c, c)}_us" for c in chans])
            w.writeheader()
            w.writerows(rows)
        print(f"記録: {out}")
    return 0


def analyze_phase(edges, names, ref, out=None):
    """同期なしの記録: 基準チャンネルとの位相差 (-0.5〜0.5 秒) の時間変化から、ドリフトの差を出す"""
    chans = [c for c in edges if edges[c]]
    ref = ref or chans[0]
    rt = [t for (t, _) in edges[ref]]
    rows = []
    print(f"基準チャンネル {ref}")
    print(f"{'ch':<8} {'node':<6} {'n':>5} {'最初の位相ms':>12} {'最後の位相ms':>12} {'ドリフトの差ppm':>15}")
    import bisect
    for c in chans:
        if c == ref:
            continue
        pts = []
        for (t, _) in edges[c]:
            k = bisect.bisect_left(rt, t)
            cands = [rt[j] for j in (k - 1, k) if 0 <= j < len(rt)]
            if not cands:
                continue
            r0 = min(cands, key=lambda x: abs(x - t))
            ph = t - r0
            if abs(ph) <= 0.5:
                pts.append((t, ph))
                rows.append({"ch": c, "node": names.get(c, ""), "t_s": f"{t:.6f}", "phase_us": f"{ph * 1e6:.1f}"})
        if len(pts) < 2:
            continue
        # 位相の巻き戻り (±0.5 秒をまたぐ) を直してから直線を当てはめる
        un = [pts[0][1]]
        for k in range(1, len(pts)):
            d = pts[k][1] - pts[k - 1][1]
            d -= round(d)
            un.append(un[-1] + d)
        xs = [p[0] for p in pts]
        mx, my = statistics.mean(xs), statistics.mean(un)
        sxx = sum((x - mx) ** 2 for x in xs)
        b = sum((x - mx) * (y - my) for x, y in zip(xs, un)) / sxx if sxx > 0 else 0.0
        print(f"{c:<8} {names.get(c, ''):<6} {len(pts):>5} {pts[0][1] * 1e3:>12.3f} {pts[-1][1] * 1e3:>12.3f} {b * 1e6:>15.2f}")
    if out:
        with open(out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["ch", "node", "t_s", "phase_us"])
            w.writeheader()
            w.writerows(rows)
        print(f"記録: {out}")
    return 0


# ==== 合成した記録での自己確認 ====

def selftest(tmpdir):
    import os
    random.seed(1)
    true_off = {"D0": 0.0, "D1": 120e-6, "D2": -80e-6, "D3": 35e-6}
    jit = 20e-6
    # VCD (1ns 単位)。整数秒ごとに切り替え、偶数秒で高
    path = os.path.join(tmpdir, "pps_selftest.vcd")
    ids = {"D0": "!", "D1": '"', "D2": "#", "D3": "$"}
    ev = []
    for s in range(1, 61):
        for ch, o in true_off.items():
            if ch == "D3" and s == 30:
                continue  # 1回の取りこぼし
            t = s + o + random.uniform(-jit, jit)
            ev.append((round(t * 1e9), ids[ch], 1 if s % 2 == 0 else 0))
    ev.sort()
    with open(path, "w") as f:
        f.write("$timescale 1 ns $end\n$scope module libsigrok $end\n")
        for ch, i in ids.items():
            f.write(f"$var wire 1 {i} {ch} $end\n")
        f.write("$upscope $end\n$enddefinitions $end\n#0 1! 1\" 1# 1$\n")
        for (t, i, v) in ev:
            f.write(f"#{t}\n{v}{i}\n")
    e = read_vcd(path)
    groups = cluster(e)
    ok = True
    # 期待: 平均からの差 ≈ 真のずれ - 真のずれの平均
    m = sum(true_off.values()) / 4
    for ch in true_off:
        ds = []
        for g in groups:
            by = {c: t for (t, c, _) in g}
            if len(by) == 4:
                ds.append(by[ch] - sum(by.values()) / 4)
        est = statistics.mean(ds)
        good = abs(est - (true_off[ch] - m)) < 10e-6
        ok &= good
        print(f"  VCD {ch}: 推定 {est * 1e6:7.1f} us  真 {(true_off[ch] - m) * 1e6:7.1f} us  {'ok' if good else 'NG'}")
    ok &= len(groups) == 60
    # CSV (時刻の列つき、1us ごとの標本は大きすぎるので変化点の前後だけを書く)
    path2 = os.path.join(tmpdir, "pps_selftest.csv")
    with open(path2, "w", newline="") as f:
        f.write("; synthetic\nTime [s],D0,D1\n")
        lv = {"D0": 1, "D1": 1}
        f.write(f"0.0,1,1\n")
        evs = sorted((s + true_off[ch], ch, 1 if s % 2 == 0 else 0) for s in range(1, 11) for ch in ("D0", "D1"))
        for (t, ch, v) in evs:
            lv[ch] = v
            f.write(f"{t:.9f},{lv['D0']},{lv['D1']}\n")
    e2 = read_csv(path2)
    g2 = cluster(e2)
    d = statistics.mean(dict((c, t) for (t, c, _) in g)["D1"] - dict((c, t) for (t, c, _) in g)["D0"] for g in g2)
    good = abs(d - 120e-6) < 1e-6 and len(g2) == 10
    ok &= good
    print(f"  CSV D1-D0: {d * 1e6:.1f} us (真 120.0)  {'ok' if good else 'NG'}")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    try:
        sys.stdout.reconfigure(errors="replace")
    except AttributeError:
        pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?")
    ap.add_argument("--samplerate", type=float, help="時刻の列が無い CSV の標本化周波数 [Hz]")
    ap.add_argument("--names", default="", help="チャンネル=ノード番号 (例 D0=1,D1=2)")
    ap.add_argument("--channels", default="", help="使うチャンネルだけ (例 D0,D1,D2)")
    ap.add_argument("--mode", choices=["sync", "phase"], default="sync",
                    help="sync = 同期ありの記録 (ずれ < 0.25 秒) / phase = 同期なしの記録")
    ap.add_argument("--ref", help="phase の基準チャンネル (既定: 最初)")
    ap.add_argument("--out", help="1秒ごとの結果の CSV")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        import tempfile
        return selftest(tempfile.gettempdir())
    if not args.path:
        ap.error("記録のファイルを指定する")
    edges = read_vcd(args.path) if args.path.lower().endswith(".vcd") else read_csv(args.path, args.samplerate)
    if args.channels:
        keep = set(args.channels.split(","))
        edges = {c: e for c, e in edges.items() if c in keep}
    names = dict(kv.split("=", 1) for kv in args.names.split(",") if "=" in kv)
    if args.mode == "phase":
        return analyze_phase(edges, names, args.ref, args.out)
    return analyze_sync(edges, names, args.out)


if __name__ == "__main__":
    sys.exit(main())

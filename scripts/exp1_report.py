#!/usr/bin/env python3
"""実験1 解析スクリプト (発注11、2026-09-21)。

入力は mqtt_log/ のCSV群 (実機) または dyne-sim の出力CSV (リハーサル)。
生成物:
  (a) 保留ノードのイノベーション統計とナイーブ基準の比較表 (stdout + report内)
  (b) 温度場の時系列ヒートマップ (1枚HTML、dashboard.py の流儀=外部依存なし)
  (c) RSSI行列 (MAC間受信強度) の時系列 (HTML内テーブル+時間ビン)

使い方:
  リハーサル:  python scripts/exp1_report.py --innov rehearsal.csv --holdout 2
  実機:        python scripts/exp1_report.py --innov mqtt_log/innov.csv --holdout 2 \
                 --field mqtt_log/field.csv --rssi mqtt_log/rssi.csv --out exp1_report.html
  ローテーション集計 (run別CSVを 保留番号:ファイル で列挙):
               python scripts/exp1_report.py --rotation 0:run_h0.csv --rotation 1:run_h1.csv ...

ナイーブ基準 (設計書§1): 保留ノードの温度を「他の観測ノードの単純平均」で予測した
ときの誤差。モデル (分布推定) がこれに勝てば「点の集まりではなく分布の推定」の証明。
判定は保留ローテーション平均で行い、保留位置ごとの内訳を必ず併記する
(設計書§1運用細則、2026-09-21検収裁定。副基準±1.0℃もローテーション平均に適用)。
"""
import argparse
import csv
import html as html_mod
import json
import statistics


def read_innov(path):
    """sim形式 (node,step,gi,y,innovation) と mqtt形式 (ts,node,step,i,y,d,y_t_ms)
    の両方を受ける。返り値: {step: {node: (y, d)}}"""
    rows = {}
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        dcol = "innovation" if "innovation" in (r.fieldnames or []) else "d"
        for row in r:
            try:
                step = int(row["step"])
                node = int(row["node"])
                y = float(row["y"])
                d = float(row[dcol])
            except (KeyError, ValueError):
                continue
            rows.setdefault(step, {})[node] = (y, d)
    return rows


def holdout_stats(rows, holdout, warmup):
    """(a) モデル誤差 |d_holdout| とナイーブ誤差 |y_holdout − mean(他ノードのy)|"""
    model, naive = [], []
    for step in sorted(rows):
        if step < warmup:
            continue
        per = rows[step]
        if holdout not in per:
            continue
        yh, dh = per[holdout]
        others = [y for n, (y, _) in per.items() if n != holdout]
        if not others:
            continue
        model.append(abs(dh))
        naive.append(abs(yh - sum(others) / len(others)))
    if not model:
        return None
    def s(v):
        return {"mean": statistics.fmean(v), "median": statistics.median(v),
                "max": max(v), "n": len(v)}
    return {"model": s(model), "naive": s(naive)}


def fmt_stats(name, st):
    if st is None:
        return f"{name}: (データなし)"
    m, nv = st["model"], st["naive"]
    verdict = "モデル優位" if m["mean"] < nv["mean"] else "ナイーブ優位"
    return (f"{name}: モデル|d| mean={m['mean']:.3f} med={m['median']:.3f} "
            f"max={m['max']:.3f} / ナイーブ mean={nv['mean']:.3f} "
            f"med={nv['median']:.3f} (n={m['n']}) → {verdict} "
            f"(比 {m['mean']/nv['mean']:.2f})")


def read_field(path):
    """sim形式 (node,step,i,x,u) と mqtt形式 (ts,node,step,t,i,x0,x1,n,u=空白区切り)
    → [(step, [(x, u), ...])] へ正規化"""
    frames = {}
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        cols = r.fieldnames or []
        mqtt = "x0" in cols
        for row in r:
            try:
                step = int(row["step"])
                if mqtt:
                    x0, x1 = float(row["x0"]), float(row["x1"])
                    us = [float(v) for v in row["u"].split()]
                    n = len(us)
                    h = (x1 - x0) / (n - 1) if n > 1 else 0.0
                    for k, u in enumerate(us):
                        frames.setdefault(step, []).append((x0 + k * h, u))
                else:
                    frames.setdefault(step, []).append((float(row["x"]), float(row["u"])))
            except (KeyError, ValueError):
                continue
    return [(s, sorted(v)) for s, v in sorted(frames.items())]


def read_rssi(path, bin_s=60.0):
    """mqtt形式 (ts,node,t_ms,ap_rssi,mac,rssi,n) → 時間ビンごとの (node→mac) 行列"""
    bins = {}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                ts = float(row["ts"])
                node = int(row["node"])
                mac = row["mac"]
                rssi = float(row["rssi"]) if row["rssi"] else None
            except (KeyError, ValueError):
                continue
            if not mac or rssi is None:
                continue
            b = int(ts // bin_s)
            bins.setdefault(b, {}).setdefault((node, mac), []).append(rssi)
    out = []
    for b in sorted(bins):
        mat = {k: statistics.fmean(v) for k, v in bins[b].items()}
        out.append((b * bin_s, mat))
    return out


def heat_color(v, lo, hi):
    t = 0.0 if hi <= lo else max(0.0, min(1.0, (v - lo) / (hi - lo)))
    r = int(255 * t)
    b = int(255 * (1 - t))
    g = int(96 * (1 - abs(2 * t - 1)))
    return f"rgb({r},{g},{b})"


def build_html(args, stats_lines, field_frames, rssi_bins):
    parts = ["<meta charset='utf-8'><title>実験1 解析レポート</title>",
             "<style>body{font-family:sans-serif;margin:16px} td,th{padding:2px 6px;"
             "border:1px solid #ccc;font-size:12px} table{border-collapse:collapse}</style>",
             "<h1>実験1 解析レポート</h1>",
             f"<p>入力: innov={html_mod.escape(str(args.innov))} holdout={args.holdout} "
             f"warmup={args.warmup}steps</p>",
             "<h2>(a) 保留ノード: モデル vs ナイーブ基準</h2><pre>"]
    parts += [html_mod.escape(l) for l in stats_lines]
    parts.append("</pre>")

    if field_frames:
        # (b) ヒートマップ: 横=時間(フレーム)、縦=位置x。canvasに直接描画
        xs = sorted({x for _, pts in field_frames for x, _ in pts})
        vals = [u for _, pts in field_frames for _, u in pts]
        lo, hi = min(vals), max(vals)
        grid = []
        for _, pts in field_frames:
            m = dict(pts)
            grid.append([m.get(x) for x in xs])
        parts.append(f"<h2>(b) 温度場ヒートマップ (横=時間、縦=位置。{lo:.1f}〜{hi:.1f})</h2>")
        parts.append(f"<canvas id='hm' width='{len(grid)}' height='{len(xs)}' "
                     f"style='width:900px;height:300px;image-rendering:pixelated;"
                     f"border:1px solid #888'></canvas>")
        parts.append("<script>const G=" + json.dumps(grid) +
                     f";const LO={lo},HI={hi};" + """
const c=document.getElementById('hm'),ctx=c.getContext('2d');
for(let t=0;t<G.length;t++)for(let i=0;i<G[t].length;i++){const v=G[t][i];
 if(v==null)continue;const s=Math.max(0,Math.min(1,(v-LO)/(HI-LO||1)));
 ctx.fillStyle=`rgb(${Math.round(255*s)},${Math.round(96*(1-Math.abs(2*s-1)))},${Math.round(255*(1-s))})`;
 ctx.fillRect(t,G[t].length-1-i,1,1);}
</script>""")

    if rssi_bins:
        parts.append("<h2>(c) RSSI行列の時系列 (ビン平均[dBm]、行=受信ノード、列=送信元MAC)</h2>")
        macs = sorted({mac for _, m in rssi_bins for (_, mac) in m})
        nodes = sorted({n for _, m in rssi_bins for (n, _) in m})
        for t0, mat in rssi_bins:
            parts.append(f"<h3>t={t0:.0f}s〜</h3><table><tr><th></th>" +
                         "".join(f"<th>{html_mod.escape(mac[-8:])}</th>" for mac in macs) + "</tr>")
            for n in nodes:
                cells = []
                for mac in macs:
                    v = mat.get((n, mac))
                    if v is None:
                        cells.append("<td></td>")
                    else:
                        cells.append(f"<td style='background:{heat_color(v, -90, -30)};"
                                     f"color:#fff'>{v:.0f}</td>")
                parts.append(f"<tr><th>node {n}</th>" + "".join(cells) + "</tr>")
            parts.append("</table>")
    return "\n".join(parts)


def rotation_lines(specs, warmup, sub_thresh=1.0):
    """保留ローテーション集計 (設計書§1運用細則): 位置ごとの内訳を必ず併記し、
    主基準・副基準±1.0℃ともローテーション平均で判定する。"""
    lines, models, naives = [], [], []
    for hold, path in specs:
        st = holdout_stats(read_innov(path), hold, warmup)
        lines.append(fmt_stats(f"holdout={hold} ({path})", st))
        if st is not None:
            models.append(st["model"]["mean"])
            naives.append(st["naive"]["mean"])
    if models:
        mm = statistics.fmean(models)
        nm = statistics.fmean(naives)
        main_v = "主基準 合格 (モデル優位)" if mm < nm else "主基準 不合格 (ナイーブ優位)"
        sub_v = "合格" if mm <= sub_thresh else "不合格"
        lines.append(f"ローテーション平均 ({len(models)}run): モデル|d| mean={mm:.3f} / "
                     f"ナイーブ mean={nm:.3f} → 比 {mm/nm:.2f} … {main_v} / "
                     f"副基準±{sub_thresh:.1f}℃ … {sub_v} ({mm:.3f}℃)")
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--innov", help="イノベーションCSV (sim/mqtt両形式可)")
    ap.add_argument("--holdout", type=int, help="保留ノードの番号")
    ap.add_argument("--rotation", action="append", metavar="H:CSV",
                    help="ローテーション集計: 保留番号:CSV を run数ぶん繰り返す")
    ap.add_argument("--warmup", type=int, default=200,
                    help="統計から除外する序盤ステップ数 (初期過渡)")
    ap.add_argument("--field", help="温度場CSV (sim/mqtt両形式可。ヒートマップ用)")
    ap.add_argument("--rssi", help="RSSI CSV (mqtt_log/rssi.csv)")
    ap.add_argument("--out", help="HTMLレポートの出力先 (省略時は統計のみ表示)")
    a = ap.parse_args()

    if a.rotation:
        specs = []
        for spec in a.rotation:
            h, path = spec.split(":", 1)
            specs.append((int(h), path))
        lines = rotation_lines(specs, a.warmup)
        a.innov = a.innov or "(rotation)"
        a.holdout = "rotation"
    elif a.innov is not None and a.holdout is not None:
        rows = read_innov(a.innov)
        lines = [fmt_stats(f"holdout={a.holdout} (warmup {a.warmup}除外)",
                           holdout_stats(rows, a.holdout, a.warmup)),
                 fmt_stats(f"holdout={a.holdout} (全期間)",
                           holdout_stats(rows, a.holdout, 0))]
    else:
        ap.error("--innov と --holdout の組、または --rotation を指定する")
    for l in lines:
        print(l)

    if a.out:
        field_frames = read_field(a.field) if a.field else []
        rssi_bins = read_rssi(a.rssi) if a.rssi else []
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(build_html(a, lines, field_frames, rssi_bins))
        print(f"written: {a.out}")


if __name__ == "__main__":
    main()

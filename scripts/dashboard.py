#!/usr/bin/env python3
"""dyne-sim の出力 CSV とログから、1枚の自己完結 HTML ダッシュボードを作る (M-S1c, 2026-08-29)。

表示内容:
  1. 温度場アニメーション (各ノードの担当区間を色分け。不在ノードは空白)
  2. 担当区間と隠れ層幅 (GROW で幅が変わると帯の高さが変わる)
  3. 界面残差の時系列 (対数軸。再生位置のカーソル、参加・自動GROW のマーカー)
  4. GROW 発火タイムライン (ノードごとの幅の推移)
  5. (任意) 参照解との誤差の時系列

使い方:
  python scripts/dashboard.py --field field.csv --residual resid.csv --event ev.csv \
      [--error err.csv] [--log sim.log] [--title "..."] --out dash.html

入力は dyne-sim の --field-csv / --residual-csv / --event-csv / --error-csv。
外部ライブラリ不要 (標準ライブラリのみ)。生成 HTML も外部依存なし。
"""
import argparse
import csv
import html
import json
import re


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--field", required=True)
    ap.add_argument("--residual", required=True)
    ap.add_argument("--event", required=True)
    ap.add_argument("--error")
    ap.add_argument("--log")
    ap.add_argument("--title", default="Dyne: 動的に協調するニューラルネットワーク群")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    # 温度場: step -> node -> (x[], u[])
    field = {}
    for r in read_csv(a.field):
        s = int(r["step"])
        n = int(r["node"])
        d = field.setdefault(s, {}).setdefault(n, [[], []])
        d[0].append(float(r["x"]))
        d[1].append(float(r["u"]))
    steps = sorted(field)
    nodes = sorted({n for s in field.values() for n in s})
    frames = [[s, [[n, field[s][n][0], field[s][n][1]] for n in sorted(field[s])]] for s in steps]

    # 界面残差: iface -> [[step, r], ...]
    resid = {}
    for r in read_csv(a.residual):
        resid.setdefault(int(r["iface"]), []).append([int(r["step"]), float(r["residual"])])

    events = []
    for r in read_csv(a.event):
        events.append({
            "kind": r["kind"], "step": int(r["step"]), "t_ms": int(r["t_ms"]), "node": int(r["node"]),
            "iface": int(r["iface"]) if r["iface"] != "" else None,
            "width": int(r["width"]) if r["width"] != "" else None,
            "current": float(r["current"]), "baseline": float(r["baseline"]),
        })

    err = {}
    if a.error:
        for r in read_csv(a.error):
            err.setdefault(int(r["node"]), []).append([int(r["step"]), float(r["linf"])])

    info = {}
    if a.log:
        txt = open(a.log, encoding="utf-8", errors="replace").read()
        for key, pat in [("result", r"^RESULT: .*$"), ("actor", r"^actor = .*$"), ("params", r"^params = .*$"),
                         ("fires", r"^fires = .*$"), ("widths", r"^final widths = .*$")]:
            m = re.search(pat, txt, re.M)
            if m:
                info[key] = m.group(0)

    data = {"frames": frames, "nodes": nodes, "resid": {str(k): v for k, v in resid.items()},
            "events": events, "err": {str(k): v for k, v in err.items()}, "info": info}
    payload = json.dumps(data, separators=(",", ":"))
    page = TEMPLATE.replace("__TITLE__", html.escape(a.title)).replace("__DATA__", payload)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(page)
    print(f"written: {a.out} ({len(frames)} frames, {len(nodes)} nodes, {len(events)} events)")


TEMPLATE = r"""<!doctype html>
<html lang="ja"><head><meta charset="utf-8"><title>__TITLE__</title>
<style>
 body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#eee}
 h1{font-size:18px;margin:10px 16px 4px}
 .info{font-size:12px;color:#aaa;margin:0 16px 8px;white-space:pre-wrap}
 .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;padding:0 16px 16px}
 .panel{background:#1b1b1b;border:1px solid #333;border-radius:6px;padding:6px}
 .panel h2{font-size:13px;margin:2px 4px 4px;color:#ccc;font-weight:normal}
 canvas{width:100%;display:block}
 .ctl{display:flex;gap:10px;align-items:center;padding:6px 16px;font-size:13px}
 input[type=range]{flex:1}
 button{background:#333;color:#eee;border:1px solid #555;border-radius:4px;padding:4px 10px}
 .legend{font-size:11px;color:#aaa;margin:2px 4px}
</style></head><body>
<h1>__TITLE__</h1>
<div class="info" id="info"></div>
<div class="ctl">
 <button id="play">▶ 再生</button>
 <input type="range" id="slider" min="0" max="0" value="0">
 <span id="stepLabel"></span>
 <label>速度 <select id="speed"><option>1</option><option selected>2</option><option>4</option><option>8</option></select>×</label>
</div>
<div class="grid">
 <div class="panel"><h2>1. 温度場 u(x) — 色=担当ノード(不在ノードは空白)</h2><canvas id="c1" data-h="260"></canvas>
  <div class="legend" id="leg1"></div></div>
 <div class="panel"><h2>2. 担当区間と隠れ層の幅(帯の高さ=幅。GROWで太くなる)</h2><canvas id="c2" data-h="260"></canvas></div>
 <div class="panel"><h2>3. 界面残差 |grad_A − grad_B|(対数軸)。▲=参加、●=自動GROW発火</h2><canvas id="c3" data-h="260"></canvas></div>
 <div class="panel"><h2>4. GROW発火タイムライン(ノードごとの幅の推移)</h2><canvas id="c4" data-h="260"></canvas></div>
 <div class="panel" id="p5"><h2>5. 参照解との最大誤差(ノードごと、対数軸)</h2><canvas id="c5" data-h="260"></canvas></div>
</div>
<script>
const D = __DATA__;
const F = D.frames, N = D.nodes;
const col = i => `hsl(${(i*137.5)%360},70%,60%)`;
const lastStep = F.length ? F[F.length-1][0] : 0;
document.getElementById('info').textContent = Object.values(D.info).join('\n');
if (!Object.keys(D.err).length) document.getElementById('p5').style.display='none';
document.getElementById('leg1').innerHTML = N.map(n=>`<span style="color:${col(n)}">■ node ${n}</span>`).join(' ');

// 幅の推移: node -> [[step, width]] (初期幅は最初のイベントから推定、無ければ16)
const W0 = 16;
const widthSeries = {};
N.forEach(n => widthSeries[n] = [[0, W0]]);
D.events.filter(e => e.width != null).sort((a,b)=>a.step-b.step).forEach(e => widthSeries[e.node].push([e.step, e.width]));
const widthAt = (n, s) => { let w = W0; for (const [st, ww] of widthSeries[n]) { if (st <= s) w = ww; else break; } return w; };
const joinStep = {}; D.events.filter(e=>e.kind==='join').forEach(e=>joinStep[e.node]=e.step);
const presentAt = (n, s) => !(n in joinStep) || s >= joinStep[n];

function setup(id){ const c=document.getElementById(id); const r=c.getBoundingClientRect(); c.width=r.width*devicePixelRatio; const H=+c.dataset.h; c.height=H*devicePixelRatio; c.style.height=H+'px'; const g=c.getContext('2d'); g.scale(devicePixelRatio,devicePixelRatio); return [g, r.width, H]; }

function draw(fi){
  const [s, parts] = F[fi];
  document.getElementById('stepLabel').textContent = `step ${s} / ${lastStep}`;
  // 1. 温度場
  { const [g,W,H]=setup('c1'); g.clearRect(0,0,W,H); const pl=36, pr=8, pt=8, pb=22;
    let umax=0.05; for (const [,,us] of parts) for (const u of us) umax=Math.max(umax,u); umax*=1.05;
    const X=x=>pl+x*(W-pl-pr), Y=u=>pt+(1-Math.max(0,Math.min(1,u/umax)))*(H-pt-pb);
    g.strokeStyle='#444'; g.beginPath(); g.moveTo(pl,Y(0)); g.lineTo(W-pr,Y(0)); g.moveTo(pl,pt); g.lineTo(pl,Y(0)); g.stroke();
    g.fillStyle='#888'; g.font='10px sans-serif'; g.fillText('u='+umax.toPrecision(2),4,Y(umax)+4); g.fillText('u=0',4,Y(0)+4); g.fillText('x=0',pl,H-6); g.fillText('x=1',W-pr-20,H-6);
    for (const [n,xs,us] of parts){ g.strokeStyle=col(n); g.lineWidth=2; g.beginPath(); xs.forEach((x,i)=>{ i?g.lineTo(X(x),Y(us[i])):g.moveTo(X(x),Y(us[i])); }); g.stroke(); }
  }
  // 2. 担当区間 + 幅
  { const [g,W,H]=setup('c2'); g.clearRect(0,0,W,H); const pl=36, pr=8;
    const X=x=>pl+x*(W-pl-pr); const base=H-30; const maxW=32;
    for (const [n,xs] of parts){ const w=widthAt(n,s); const h=(w/maxW)*(H-50); g.fillStyle=col(n); g.globalAlpha=0.75;
      g.fillRect(X(xs[0]),base-h,X(xs[xs.length-1])-X(xs[0]),h); g.globalAlpha=1; g.fillStyle='#fff'; g.font='11px sans-serif';
      g.fillText(`${n}:w${w}`,X(xs[0])+2,base-h-3); }
    N.forEach(n=>{ if(!presentAt(n,s)){ g.fillStyle='#666'; g.font='11px sans-serif'; g.fillText(`node ${n}: 不在`, pl, 14+12*(n%3)); } });
    g.fillStyle='#888'; g.font='10px sans-serif'; g.fillText('x=0',pl,H-8); g.fillText('x=1',W-pr-20,H-8);
  }
  // 3. 残差
  { const [g,W,H]=setup('c3'); g.clearRect(0,0,W,H); const pl=44, pr=8, pt=8, pb=20;
    let lo=Infinity, hi=-Infinity; for (const k in D.resid) for (const [,r] of D.resid[k]) if (r>0){ lo=Math.min(lo,r); hi=Math.max(hi,r); }
    if(!isFinite(lo)){lo=1e-6;hi=1;} lo=Math.max(lo,hi*1e-4); const L=v=>Math.log10(Math.max(v,lo));
    const X=st=>pl+st/lastStep*(W-pl-pr), Y=r=>pt+(1-(L(r)-L(lo))/(L(hi)-L(lo)||1))*(H-pt-pb);
    g.strokeStyle='#444'; g.beginPath(); g.moveTo(pl,pt); g.lineTo(pl,H-pb); g.lineTo(W-pr,H-pb); g.stroke();
    g.fillStyle='#888'; g.font='10px sans-serif'; g.fillText(hi.toExponential(0),2,pt+8); g.fillText(lo.toExponential(0),2,H-pb);
    for (const k in D.resid){ const j=+k; g.strokeStyle=col(j); g.lineWidth=1.2; g.beginPath(); let first=true;
      for (const [st,r] of D.resid[k]) { const x=X(st), y=Y(r); first?g.moveTo(x,y):g.lineTo(x,y); first=false; } g.stroke();
      const pts=D.resid[k]; const last=pts[pts.length-1]; g.fillStyle=col(j); g.fillText(`iface ${j}`, Math.min(X(last[0])+2,W-40), Y(last[1])); }
    for (const e of D.events){ if(e.kind==='join'){ g.fillStyle='#ff0'; g.beginPath(); g.moveTo(X(e.step),H-pb); g.lineTo(X(e.step)-5,H-pb+10); g.lineTo(X(e.step)+5,H-pb+10); g.fill(); }
      if(e.kind==='auto_grow'){ g.fillStyle='#f66'; g.beginPath(); g.arc(X(e.step),Y(e.current),4,0,7); g.fill(); } }
    g.strokeStyle='#fff'; g.beginPath(); g.moveTo(X(s),pt); g.lineTo(X(s),H-pb); g.stroke();
  }
  // 4. GROW タイムライン
  { const [g,W,H]=setup('c4'); g.clearRect(0,0,W,H); const pl=44, pr=8, pt=8, pb=20; const rows=N.length; const rh=(H-pt-pb)/rows;
    const X=st=>pl+st/lastStep*(W-pl-pr);
    N.forEach((n,i)=>{ const y0=pt+i*rh; g.fillStyle='#888'; g.font='10px sans-serif'; g.fillText(`node ${n}`,2,y0+rh*0.7);
      const ws=widthSeries[n]; for(let k=0;k<ws.length;k++){ const a=ws[k][0], b=k+1<ws.length?ws[k+1][0]:lastStep; const w=ws[k][1];
        const st = presentAt(n,a)?a:(joinStep[n]??a); if (b<st) continue;
        g.fillStyle=col(n); g.globalAlpha=0.25+0.7*(w/32); g.fillRect(X(Math.max(st,a)),y0+2,X(b)-X(Math.max(st,a)),rh-4); g.globalAlpha=1;
        g.fillStyle='#fff'; g.fillText(`w${w}`,X(Math.max(st,a))+2,y0+rh*0.7); } });
    for (const e of D.events){ if(e.kind==='auto_grow'){ const i=N.indexOf(e.node); g.strokeStyle='#f66'; g.lineWidth=2; g.beginPath(); g.moveTo(X(e.step),pt+i*rh); g.lineTo(X(e.step),pt+(i+1)*rh); g.stroke(); }
      if(e.kind==='join'){ const i=N.indexOf(e.node); g.strokeStyle='#ff0'; g.lineWidth=2; g.beginPath(); g.moveTo(X(e.step),pt+i*rh); g.lineTo(X(e.step),pt+(i+1)*rh); g.stroke(); } }
    g.strokeStyle='#fff'; g.lineWidth=1; g.beginPath(); g.moveTo(X(s),pt); g.lineTo(X(s),H-pb); g.stroke();
    g.fillStyle='#888'; g.fillText('step 0',pl,H-6); g.fillText(`step ${lastStep}`,W-pr-50,H-6);
  }
  // 5. 誤差
  if (Object.keys(D.err).length) { const [g,W,H]=setup('c5'); g.clearRect(0,0,W,H); const pl=44, pr=8, pt=8, pb=20;
    let lo=Infinity, hi=-Infinity; for (const k in D.err) for (const [,r] of D.err[k]) if (r>0){ lo=Math.min(lo,r); hi=Math.max(hi,r); }
    if(!isFinite(lo)){lo=1e-6;hi=1;} lo=Math.max(lo,hi*1e-4); const L=v=>Math.log10(Math.max(v,lo));
    const X=st=>pl+st/lastStep*(W-pl-pr), Y=r=>pt+(1-(L(r)-L(lo))/(L(hi)-L(lo)||1))*(H-pt-pb);
    g.strokeStyle='#444'; g.beginPath(); g.moveTo(pl,pt); g.lineTo(pl,H-pb); g.lineTo(W-pr,H-pb); g.stroke();
    g.fillStyle='#888'; g.font='10px sans-serif'; g.fillText(hi.toExponential(0),2,pt+8); g.fillText(lo.toExponential(0),2,H-pb);
    for (const k in D.err){ const n=+k; g.strokeStyle=col(n); g.lineWidth=1.2; g.beginPath(); let first=true;
      for (const [st,r] of D.err[k]) { const x=X(st), y=Y(r); first?g.moveTo(x,y):g.lineTo(x,y); first=false; } g.stroke(); }
    g.strokeStyle='#fff'; g.beginPath(); g.moveTo(X(s),pt); g.lineTo(X(s),H-pb); g.stroke();
  }
}
const slider=document.getElementById('slider'); slider.max=Math.max(0,F.length-1);
let playing=false, fi=0, timer=null;
function show(i){ fi=i; slider.value=i; draw(i); }
slider.oninput=()=>show(+slider.value);
function tick(){ if(!playing) return; const sp=+document.getElementById('speed').value; fi=(fi+sp)%F.length; show(fi); timer=setTimeout(tick,50); }
document.getElementById('play').onclick=function(){ playing=!playing; this.textContent=playing?'❚❚ 停止':'▶ 再生'; if(playing) tick(); };
window.onresize=()=>draw(fi);
if (F.length) show(0);
</script></body></html>
"""

if __name__ == "__main__":
    main()

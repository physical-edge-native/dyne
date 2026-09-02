#!/usr/bin/env python3
"""dyne-swarm の出力から群れの軌跡アニメーション HTML を作る (タスク4b、2026-08-29)。

使い方:
  python scripts/swarm_anim.py --dir swarm_out [--compare "通信なし=swarm_out_solo" ...] \
      [--title "..."] --out swarm.html

--dir には dyne-swarm の --out-dir (traj.csv / metrics.csv / field.csv / summary.txt) を渡す。
--compare は「凡例名=別の出力ディレクトリ」で、指標グラフに重ねて比較する (複数可)。
外部ライブラリ不要。生成 HTML も外部依存なし。
"""
import argparse
import csv
import html
import json
import os


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def load_metrics(d):
    return [[int(r["step"]), float(r["frac_ok"]), float(r["mean_dev"]), float(r["spread"])]
            for r in read_csv(os.path.join(d, "metrics.csv"))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--compare", action="append", default=[])
    ap.add_argument("--title", default="Dyne: 生物の群れの再現 — 局所知覚と近傍通信だけで最適温度域へ")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    frames = {}
    for r in read_csv(os.path.join(a.dir, "traj.csv")):
        frames.setdefault(int(r["step"]), []).append([float(r["x"]), float(r["y"]), float(r["temp"]), int(r["n_nbr"])])
    steps = sorted(frames)
    fr = [[s, frames[s]] for s in steps]

    grid = {}
    for r in read_csv(os.path.join(a.dir, "field.csv")):
        grid[(int(r["i"]), int(r["j"]))] = float(r["temp"])
    g = max(i for i, _ in grid)
    field = [[grid[(i, j)] for i in range(g + 1)] for j in range(g + 1)]

    summary = open(os.path.join(a.dir, "summary.txt"), encoding="utf-8").read()
    kv = dict(tok.split("=", 1) for tok in summary.splitlines()[0].split() if "=" in tok)
    t_opt, tol = float(kv.get("t_opt", 24)), float(kv.get("tol", 1))

    series = [{"name": "この実行 (" + kv.get("mode", "") + ")", "m": load_metrics(a.dir)}]
    for c in a.compare:
        name, d = c.split("=", 1)
        series.append({"name": name, "m": load_metrics(d)})

    data = {"frames": fr, "field": field, "g": g, "t_opt": t_opt, "tol": tol, "summary": summary, "series": series}
    page = TEMPLATE.replace("__TITLE__", html.escape(a.title)).replace("__DATA__", json.dumps(data, separators=(",", ":")))
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(page)
    print(f"written: {a.out} ({len(fr)} frames, {len(fr[0][1]) if fr else 0} agents, {len(series)} series)")


TEMPLATE = r"""<!doctype html>
<html lang="ja"><head><meta charset="utf-8"><title>__TITLE__</title>
<style>
 body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#eee}
 h1{font-size:18px;margin:10px 16px 4px}
 .info{font-size:12px;color:#aaa;margin:0 16px 8px;white-space:pre-wrap}
 .grid{display:grid;grid-template-columns:1.2fr 1fr;gap:10px;padding:0 16px 16px}
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
 <label><input type="checkbox" id="trail" checked> 軌跡</label>
</div>
<div class="grid">
 <div class="panel"><h2>1. 温度場と群れ(青=冷 → 赤=暖。白い等高線=最適温度域の境界。点:緑=最適域内、橙=暑すぎ、水色=寒すぎ。点の大きさ=通信相手の数)</h2>
  <canvas id="c1" data-h="520"></canvas></div>
 <div>
  <div class="panel"><h2>2. 最適域にいる個体の割合(比較線つき)</h2><canvas id="c2" data-h="250"></canvas><div class="legend" id="leg2"></div></div>
  <div class="panel" style="margin-top:10px"><h2>3. 平均 |T − T_opt|(℃)</h2><canvas id="c3" data-h="250"></canvas></div>
 </div>
</div>
<script>
const D=__DATA__; const F=D.frames; const G=D.g; const last=F[F.length-1][0];
document.getElementById('info').textContent=D.summary;
let tmin=Infinity,tmax=-Infinity; for(const row of D.field) for(const t of row){tmin=Math.min(tmin,t);tmax=Math.max(tmax,t);}
const scol=i=>`hsl(${(i*137.5+200)%360},80%,65%)`;
document.getElementById('leg2').innerHTML=D.series.map((s,i)=>`<span style="color:${scol(i)}">— ${s.name}</span>`).join('  ');
function setup(id){const c=document.getElementById(id);const r=c.getBoundingClientRect();const H=+c.dataset.h;c.width=r.width*devicePixelRatio;c.height=H*devicePixelRatio;c.style.height=H+'px';const g=c.getContext('2d');g.scale(devicePixelRatio,devicePixelRatio);return [g,r.width,H];}
// 温度→色 (青→白→赤)
function tcol(t){const u=(t-tmin)/(tmax-tmin);const r=Math.round(255*Math.min(1,2*u)),b=Math.round(255*Math.min(1,2*(1-u))),g=Math.round(255*(1-Math.abs(2*u-1)));return `rgb(${r*0.8},${g*0.8},${b*0.8})`;}
let fieldImg=null;
function drawField(g,W,H,S,ox,oy){
  if(!fieldImg){const c=document.createElement('canvas');c.width=G+1;c.height=G+1;const cg=c.getContext('2d');
    for(let j=0;j<=G;j++)for(let i=0;i<=G;i++){cg.fillStyle=tcol(D.field[j][i]);cg.fillRect(i,G-j,1,1);} fieldImg=c;}
  g.imageSmoothingEnabled=true; g.drawImage(fieldImg,ox,oy,S,S);
  // 最適域の境界 (格子上で |T-Topt|<tol の内外が変わる辺を描く簡易等高線)
  g.strokeStyle='rgba(255,255,255,0.85)'; g.lineWidth=1.2; g.beginPath();
  const inb=(i,j)=>Math.abs(D.field[j][i]-D.t_opt)<D.tol;
  for(let j=0;j<G;j++)for(let i=0;i<G;i++){const a=inb(i,j),b=inb(i+1,j),c=inb(i,j+1);
    const x=ox+i/G*S,y=oy+(1-j/G)*S,dx=S/G;
    if(a!==b){g.moveTo(x+dx,y);g.lineTo(x+dx,y-dx);} if(a!==c){g.moveTo(x,y-dx);g.lineTo(x+dx,y-dx);} }
  g.stroke();
}
function draw(fi){
  const [s,ag]=F[fi]; document.getElementById('stepLabel').textContent=`step ${s} / ${last}`;
  {const [g,W,H]=setup('c1'); g.clearRect(0,0,W,H); const S=Math.min(W,H)-10, ox=(W-S)/2, oy=5;
   drawField(g,W,H,S,ox,oy);
   const X=x=>ox+x*S, Y=y=>oy+(1-y)*S;
   if(document.getElementById('trail').checked){ g.lineWidth=1; const from=Math.max(0,fi-30);
     for(let k=0;k<ag.length;k++){ g.strokeStyle='rgba(255,255,255,0.35)'; g.beginPath();
       for(let f=from;f<=fi;f++){const p=F[f][1][k]; f===from?g.moveTo(X(p[0]),Y(p[1])):g.lineTo(X(p[0]),Y(p[1]));} g.stroke(); } }
   for(const [x,y,t,nn] of ag){ const d=t-D.t_opt; g.fillStyle=Math.abs(d)<D.tol?'#3f3':(d>0?'#fa0':'#0ef');
     g.beginPath(); g.arc(X(x),Y(y),3+Math.min(nn,6)*0.6,0,7); g.fill(); g.strokeStyle='#000'; g.lineWidth=0.8; g.stroke(); }
   g.fillStyle='#fff'; g.font='11px sans-serif'; g.fillText(`冷 ${tmin.toFixed(0)}℃`,ox+4,oy+S-6); g.fillText(`暖 ${tmax.toFixed(0)}℃`,ox+S-60,oy+S-6);
  }
  const line=(id,idx,ymax,fmt)=>{const [g,W,H]=setup(id); g.clearRect(0,0,W,H); const pl=40,pr=8,pt=8,pb=20;
   const X=st=>pl+st/last*(W-pl-pr), Y=v=>pt+(1-v/ymax)*(H-pt-pb);
   g.strokeStyle='#444'; g.beginPath(); g.moveTo(pl,pt); g.lineTo(pl,H-pb); g.lineTo(W-pr,H-pb); g.stroke();
   g.fillStyle='#888'; g.font='10px sans-serif'; g.fillText(fmt(ymax),2,pt+8); g.fillText(fmt(0),2,H-pb);
   D.series.forEach((sr,i)=>{ g.strokeStyle=scol(i); g.lineWidth=i===0?2:1.2; g.beginPath(); let first=true;
     for(const m of sr.m){const x=X(m[0]),y=Y(Math.min(m[idx],ymax)); first?g.moveTo(x,y):g.lineTo(x,y); first=false;} g.stroke(); });
   g.strokeStyle='#fff'; g.lineWidth=1; g.beginPath(); g.moveTo(X(s),pt); g.lineTo(X(s),H-pb); g.stroke();
   g.fillStyle='#888'; g.fillText('step 0',pl,H-6); g.fillText(`step ${last}`,W-pr-50,H-6); };
  line('c2',1,1.0,v=>(v*100).toFixed(0)+'%');
  let dmax=0; for(const sr of D.series) for(const m of sr.m) dmax=Math.max(dmax,m[2]); line('c3',2,Math.ceil(dmax),v=>v.toFixed(0));
}
const slider=document.getElementById('slider'); slider.max=F.length-1; let playing=false,fi=0;
function show(i){fi=i;slider.value=i;draw(i);} slider.oninput=()=>show(+slider.value);
document.getElementById('trail').onchange=()=>draw(fi);
function tick(){if(!playing)return;const sp=+document.getElementById('speed').value;fi=(fi+sp)%F.length;show(fi);setTimeout(tick,50);}
document.getElementById('play').onclick=function(){playing=!playing;this.textContent=playing?'❚❚ 停止':'▶ 再生';if(playing)tick();};
window.onresize=()=>draw(fi); show(0);
</script></body></html>
"""

if __name__ == "__main__":
    main()

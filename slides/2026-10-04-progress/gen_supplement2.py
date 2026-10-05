# -*- coding: utf-8 -*-
# 第2補足 (4枚):
#  1. r の説明 … 本体スライド18「理論: 境目の条件を変えても…」の直前に挿入
#  2. 透過境界条件とは(作り直し) … 本体スライド19「透過境界条件の導き方」と差し替え
#  3. 導出1(ラプラス変換からの筋道) … 2の直後に挿入
#  4. 導出2(指数和近似と補助変数) … 3の直後に挿入
from pptx import Presentation
from pptx.util import Inches, Pt
from PIL import Image
import shutil

SRC = "work.pptx"
OUT = "2026-10-04-supplement2.pptx"
shutil.copy(SRC, OUT)
prs = Presentation(OUT)
layout_obj = prs.slides[3].slide_layout

def add_image(slide, path, x, y, w, max_bottom=5.42):
    im = Image.open(path); aw, ah = im.size
    h = w * ah / aw
    if y + h > max_bottom:
        h = max_bottom - y; w = h * aw / ah
    slide.shapes.add_picture(path, Inches(x), Inches(y), Inches(w), Inches(h))

def new_slide(title, body, images, note):
    s = prs.slides.add_slide(layout_obj)
    for ph in s.placeholders:
        if ph.placeholder_format.idx == 0:
            ph.text_frame.text = title
            for p0 in ph.text_frame.paragraphs:
                for r0 in p0.runs: r0.font.name = "Arial"
    for ph in s.placeholders:
        if ph.placeholder_format.idx == 1:
            tf = ph.text_frame; tf.clear(); tf.word_wrap = True
            for i, (lvl, text, size) in enumerate(body):
                p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
                p.level = lvl
                r = p.add_run(); r.text = text
                r.font.size = Pt(size); r.font.name = "Arial"
    for (path, x, y, w) in images:
        add_image(s, path, x, y, w)
    s.notes_slide.notes_text_frame.text = note
    return s

FI = "formula_imgs/"

# 1. r の説明
new_slide("r = αΔt/h² : 時間刻みの大きさの指標",
 [(0,"差分法は、格子幅 h ごとの点を、時間刻み Δt ごとに更新する", 16),
  (0,"r は「1ステップでどれだけ大股に時間を進めるか」を表す単位なしの数", 16),
  (1,"Δt を大きくするほど、また格子を細かくする(hを小さくする)ほど、r は大きくなる", 15),
  (0,"陽的な差分法(前進オイラー法)は r ≤ 1/2 でないと発散する", 16),
  (1,"理由は後半の理論スライドで(細かいギザギザ成分の増幅)", 15),
  (0,"以降のスライドの「安定の条件」は、すべてこの r で表す", 16)],
 [(FI+"f_r.png", 3.3, 3.9, 2.6)],
 "挿入位置: 本体スライド18「理論: 境目の条件を変えても時間刻みの上限は変わらない」の直前。"
 "出典: stability-by-eigenvalues 6.3節。式: r = αΔt/h²(α=熱拡散率、Δt=時間刻み、h=格子間隔)。")

# 2. 透過境界条件とは(作り直し。本体スライド19と差し替え)
new_slide("透過境界条件: 条件式とその意味",
 [(0,"仮定: 境目の外側に、同じ材質が無限に続いている", 16),
  (0,"このとき、境目から外へ逃げる熱(境目の勾配)は下の式で厳密に決まる", 16),
  (0,"g(t) = 自分の境目の温度の履歴。昔の温度変化 g′(τ) ほど小さな重み 1/√(t−τ) で効く", 15),
  (1,"必要なのは自分の境目の温度の履歴だけ(隣の観測も、外側の長さの情報も不要)", 14),
  (1,"この積分の素直な計算は毎ステップ履歴全体をたどる → 次の2枚で軽くする工夫", 14)],
 [(FI+"f_dtn.png", 2.6, 3.25, 4.4)],
 "差し替え: 本体スライド19「透過境界条件の導き方」をこのスライドに置き換え、続けて導出1・2を挿入。"
 "出典: transparent-boundary-stability 3節。式: -∂u/∂x(0,t) = (1/√(πα)) ∫₀ᵗ g'(τ)/√(t-τ) dτ"
 "(半無限媒質のディリクレ・ノイマン写像)。")

# 3. 導出1
new_slide("導出1: この式はどこから来るか",
 [(0,"外側(x>0)の熱方程式を、境目の温度履歴 g(t) を境界条件として解く", 15),
  (0,"時間方向にラプラス変換すると、遠方で有界な解とその境目の勾配が求まる(下段左)", 15),
  (1,"√s は時間の「2分の1階微分」。逆変換すると前ページの畳み込みの式になる", 14),
  (0,"検算: 境目の温度を階段状に U₀ へ上げた場合の厳密解(下段右)", 15),
  (1,"境目の勾配 U₀/√(παt) は時間とともに減る = 固定端と断熱の間を物理法則どおりに補間", 14)],
 [(FI+"f_laplace.png", 0.7, 3.15, 2.8), (FI+"f_sgrad.png", 0.9, 4.35, 2.6),
  (FI+"f_erfc.png", 5.3, 3.15, 3.0), (FI+"f_ustep.png", 5.5, 4.35, 2.5)],
 "挿入位置: 作り直した透過境界条件スライドの直後。出典: transparent-boundary-stability 2-3節"
 "(階段応答は Carslaw & Jaeger の古典問題)。式: û(x,s)=ĝ(s)e^{-x√(s/α)} / -û_x(0,s)=√(s/α)ĝ(s) / "
 "u(x,t)=U₀erfc(x/(2√(αt))) / -∂u/∂x(0,t)=U₀/√(παt)。")

# 4. 導出2
new_slide("導出2: 計算を軽くする(指数和近似と補助変数)",
 [(0,"重み 1/√t を厳密な積分表示(下段左)から、対数軸の台形公式でM項に離散化", 15),
  (1,"= 指数関数の和で近似する「指数和近似」(下段中央)", 14),
  (0,"指数関数の半群性 e^{−λ(a+b)} = e^{−λa}·e^{−λb} により、履歴の畳み込みは", 15),
  (0,"補助変数 y_j(忘却係数 λ_j の1次の漸化式。下段右)の更新だけで計算できる", 15),
  (1,"毎ステップの計算量・記憶量が一定(履歴全体を覚え直さない)", 14),
  (1,"精度: 刻み0.6で M=32項・相対誤差 7.7e-4。直接計算との相対差 9.1e-6 を検証済み。実機は25項", 13)],
 [(FI+"f_integral.png", 0.5, 3.6, 3.1), (FI+"f_soe.png", 3.9, 3.7, 2.4),
  (FI+"f_bucket.png", 6.5, 3.75, 3.3)],
 "挿入位置: 導出1の直後。出典: transparent-boundary-stability 4-5節。"
 "式: 1/√t=(1/√π)∫λ^{-1/2}e^{-λt}dλ / 1/√t≈Σw_j e^{-λ_j t} / y_j^{m+1}=e^{-λ_jΔt}y_j^m+e^{-λ_jΔt/2}Δg^m。"
 "本体スライド20(離散化3方式と安定限界)はこの直後にそのまま続く。")

# 既存47枚を削除
xml_slides = prs.slides._sldIdLst
ids = list(xml_slides)
RNS = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"
for sldId in ids[:-4]:
    prs.part.drop_rel(sldId.get(RNS))
    xml_slides.remove(sldId)
prs.save(OUT)
print("saved", OUT, "slides:", len(list(xml_slides)))

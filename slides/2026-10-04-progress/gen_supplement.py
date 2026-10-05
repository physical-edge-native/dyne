# -*- coding: utf-8 -*-
# 追加スライド2枚(学習型アクターの式の読み方 / 最適化の中身)を、
# 前回スライドと同じデザイン(同じレイアウト)で独立の pptx として生成する。
# 「2種類の計算機」のページの直後に手で挿入して使う想定。
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from PIL import Image
import shutil

SRC = "work.pptx"
OUT = "2026-10-04-supplement.pptx"
FI = "formula_imgs"
shutil.copy(SRC, OUT)
prs = Presentation(OUT)
layout_obj = prs.slides[3].slide_layout   # OBJECT (タイトル+本文)

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

new_slide("学習型アクターの式の読み方",
 [(0,"温度分布を「位置 x → 温度」の滑らかな関数として、49個の数で表す", 16),
  (1,"x: 区間内の位置(入力) / u_θ(x): その位置の温度(出力)", 15),
  (1,"tanh: なめらかなS字型の関数。これを16個重ねて形を作る", 15),
  (1,"a_k: S字の傾き / b_k: S字の横位置 / c_k: S字の高さ / e: 全体の底上げ", 15),
  (1,"θ はこれら調整できる数(パラメータ)の集まり: 16×3+1 = 49個", 15),
  (0,"2階微分 u_θ''(x)(曲がり具合)は、この式を2回微分した式で厳密に計算できる", 15)],
 [("formula_imgs/f_nn.png", 2.3, 3.6, 4.8)],
 "「2種類の計算機」の直後に挿入。出典: learned-stability-region 0節(幅16でパラメータ3w+1=49個)、"
 "実装 crates/heat1d-pinn-actor(入力は区間を[-1,1]へ正規化してからtanhに入れる。ジェット方式で u,u',u'' を同時に厳密計算)。"
 "式: u_θ(x)=Σ_{k=1}^{16} c_k tanh(a_k x + b_k) + e")

new_slide("学習型アクターの1ステップ = 最適化",
 [(0,"R(x) は「熱伝導方程式の時間差分版をどれだけ破っているか」(残差)", 15),
  (1,"u_prev: 前の時刻の分布(確定済み) / Δt: 時間刻み / α: 熱の伝わりやすさ", 14),
  (1,"2階微分を「新しい分布 u_θ」で評価 = 後退オイラー法の形(これが安定性の鍵)", 14),
  (0,"各格子点(+中点)での R(x)² の和 + 境目の条件のずれ² を損失にして最小化", 15),
  (0,"最適化はレーベンバーグ・マーカート法(LM法): 小さな非線形最小二乗の定番", 15),
  (1,"残差を各パラメータで微分した表(ヤコビ行列)から修正量を連立方程式で解く", 14),
  (1,"パラメータが49個と少ないので行列が小さく、1ステップあたり5回の反復で足りる", 14),
  (1,"勾配だけを使う方法(Adam等)は u'' の項の硬さで収束せず、不採用だった", 14)],
 [("formula_imgs/f_resid.png", 2.6, 4.1, 4.2)],
 "出典: learned-stability-region 1節、levenberg-marquardt 前半(ガウス・ニュートン法に減衰項を加えた (JᵀJ+λdiag(JᵀJ))δ=−Jᵀr を解く)、"
 "実装報告(2026-08-25): Adam/SGDは不成立でLM法を採用、既定は1ステップ5反復。"
 "式: R(x)=[u_θ(x)−u_prev(x)]/Δt − α u_θ''(x)")

# 既存の47枚を削除して、追加した2枚だけ残す
xml_slides = prs.slides._sldIdLst
ids = list(xml_slides)
RNS = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"
for sldId in ids[:-2]:
    prs.part.drop_rel(sldId.get(RNS))
    xml_slides.remove(sldId)
prs.save(OUT)
print("saved", OUT, "slides:", len(list(xml_slides)))

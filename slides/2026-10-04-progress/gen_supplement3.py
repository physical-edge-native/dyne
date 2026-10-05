# -*- coding: utf-8 -*-
# 第3補足 (3枚): 本体スライド30「理論: なぜ r ≤ 1/2 か——固有値で見る」を
# この3枚に差し替える(拡張版)。
#  1. 計算全体は「行列の掛け算」(行列 A の導入)
#  2. 固有ベクトルに分解すると成分ごとの掛け算になる(λ と z の導入)
#  3. なぜ r ≤ 1/2 か(結論)
from pptx import Presentation
from pptx.util import Inches, Pt
from PIL import Image
import shutil

SRC = "work.pptx"
OUT = "2026-10-04-supplement3.pptx"
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

# 1. 行列の導入
new_slide("理論: 1ステップは「行列を1回掛ける」操作",
 [(0,"時間を離散化する前に、空間だけ離散化すると、全格子点の温度をまとめた", 15),
  (0,"ベクトル u の時間変化が、1本の線形な式になる(A は3本の帯だけの行列)", 15),
  (0,"前進オイラー法で時間も離散化すると、1ステップ = 行列 (I+ΔtA) を1回掛けること", 15),
  (1,"「2000ステップ計算して発散するか」=「同じ行列を2000回掛けてベクトルが育つか縮むか」", 14),
  (1,"→ 行列の掛け算の繰り返しの性質さえ分かれば、安定かどうかが分かる", 14)],
 [(FI+"f_dudt.png", 0.7, 3.35, 1.7), (FI+"f_Amat.png", 2.9, 3.15, 3.5),
  (FI+"f_euler_mat.png", 6.7, 3.4, 2.9)],
 "差し替え: 本体スライド30をこの3枚に置き換える(1枚目)。出典: stability-by-eigenvalues 3節(線の方法)・6節。"
 "式: du/dt=Au(Aは対角-2α/h²・両隣α/h²の三重対角行列)/ u^{m+1}=(I+ΔtA)u^m。"
 "口頭: 両端は温度0固定なので内部63点のベクトル。")

# 2. 固有ベクトル分解
new_slide("理論: 固有ベクトルに分ければ、成分ごとの「ただの掛け算」になる",
 [(0,"A の固有ベクトル v_k は「格子の上の sin 波」(k=1は最も緩やか、k=63は最も細かいギザギザ)", 14),
  (1,"固有値 λ_k はすべて負の実数(下の式)。どんな初期値も sin 波の重ね合わせに分解できる", 14),
  (0,"固有ベクトルに行列を掛けると、向きは変わらず「数が掛かるだけ」:", 15),
  (0,"→ 63点の連立の問題が、成分ごとに独立な63本の掛け算にばらける。これが固有値を見る理由", 15),
  (1,"記法: z_k = Δt·λ_k(固有値を1ステップぶんに換算した数)、φ(z) = 1+z を増幅因子と呼ぶ", 14),
  (1,"成分 k は1ステップごとに φ(z_k) 倍される", 14)],
 [(FI+"f_eig.png", 0.6, 3.75, 3.2), (FI+"f_eigrel.png", 4.2, 3.8, 5.3),
  (FI+"f_zdef.png", 4.6, 4.75, 3.6)],
 "差し替え2枚目。出典: stability-by-eigenvalues 4節(固有値の導出)・6.1節(φ(ΔtA)の形になる理由)。"
 "式: λ_k=-(4α/h²)sin²(kπh/2) / Av_k=λ_k v_k ⇒ (I+ΔtA)v_k=(1+Δtλ_k)v_k / z_k=Δtλ_k, φ(z)=1+z。"
 "口頭: 固有値=その方向だけ見れば行列が「ただの数」になる倍率。")

# 3. 結論
new_slide("理論: なぜ r ≤ 1/2 か",
 [(0,"安定の条件 = すべての成分で |φ(z_k)| ≤ 1(どの成分も1ステップで育たない)", 15),
  (0,"固有値の範囲から z_k は区間 [−4r, 0) に並ぶ(最も細かい成分が z ≈ −4r)", 15),
  (0,"前進オイラー法 φ(z)=1+z にこの範囲を入れると:", 15),
  (0,"r > 1/2 で最初に破れるのは最も細かいギザギザ成分:", 15),
  (1,"倍率が −1 を下回り、符号を反転しながら増大 → 発散がギザギザ模様に見える理由", 14),
  (1,"(次ページ: 後退オイラー法 φ(z)=1/(1−z) は負の z すべてで |φ|≤1 → 時間刻みの制限なし)", 14)],
 [(FI+"f_zrange.png", 0.8, 3.75, 2.9), (FI+"f_phicond.png", 4.1, 3.75, 5.0),
  (FI+"f_sawtooth.png", 2.7, 4.7, 3.6)],
 "差し替え3枚目。出典: stability-by-eigenvalues 6.3節(安定条件の表)・7節(鋸歯状の発散)。"
 "式: z_k∈[-4r,0) / |1+z|≤1⇔-2≤z≤0⇒4r≤2(=r≤1/2) / φ(z_{n-1})≈1-4r<-1。"
 "この直後に本体スライド31(分割しても一致する理由)、32(後退オイラー法)が続く。")

xml_slides = prs.slides._sldIdLst
ids = list(xml_slides)
RNS = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"
for sldId in ids[:-3]:
    prs.part.drop_rel(sldId.get(RNS))
    xml_slides.remove(sldId)
prs.save(OUT)
print("saved", OUT, "slides:", len(list(xml_slides)))

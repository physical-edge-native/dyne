# slides/ — 発表スライドの生成物

スライド本体(.pptx)はGoogleスライドで管理するため**gitでは追跡しない**(.gitignore)。
追跡するのは構成案と生成スクリプトのみ(次回のスライド作成で再利用するため)。

## 2026-10-04-progress/(未踏IT進捗発表)

| ファイル | 追跡 | 内容 |
|---|---|---|
| plan.md | ○ | 構成案(スライド一覧・出典・用語の規則・食い違い一覧) |
| gen_formulas.py | ○ | 式画像の生成(matplotlib、透明背景) |
| gen_slides.py | ○ | 本体65枚の生成(python-pptx。前回スライドのコピーに挿入) |
| gen_supplement*.py | ○ | 補足スライド(変数説明/r・透過境界の導出/固有値の筋道) |
| *.pptx / formula_imgs/ / final.md / old25_degradation.jpg | × | 生成物(スクリプトで再生成可能。本体はGoogleスライドが正) |

再生成手順: reference/previous_deck.pptx を work.pptx としてコピー →
`python gen_formulas.py` → `python gen_slides.py`(実測図は ../../figs/ を参照)。

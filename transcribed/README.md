# transcribed/ — 報告書からの手転記データ(照合済み)

web版Claude(数値計算の壁打ち会話)が理論文書・図の作成用に、docs/ の報告書の
表から**手で転記**したデータ。2026-10-04に recovered/(会話記録からの復元実測)と
突き合わせ、実質的な食い違いがないことを確認済み(丸め1件のみ: 6e26 vs 6.257e26)。

| ファイル | 内容 |
|---|---|
| learned_sweep.dat | 主な掃引の掃引1(32セル)。reference/learned_map.gp・learned_error.gp が参照 |
| learned_width32.dat | 幅32の測定(6行。r=1.0〜1.5の発散3行は含まず=recovered/a4_width.csv 参照) |
| learned_width.dat | 幅ごとの発散境界の区間。reference/learned_width.gp が参照 |
| nudging_cells.dat | ナッジング安定地図60セル+判別2セルの実測と予測スペクトル半径。**ρ列は2026-10-04に実装照合済みの観測点(8,25,41,57)で再計算済み**(判定の反転なし) |

図を作り直す場合は、手転記の本ディレクトリより recovered/ を元にした figs/ を正とする。

# reference/ — 数値計算の安定性の理論文書(正本)と理論図

web版Claude(数値計算の壁打ち会話)でまとめた、実装とシミュレーション結果の
理論的な保証のための文書群(2026-10-03)。

| ファイル | 内容 |
|---|---|
| 2026-10-03-stability-by-eigenvalues.md | 親文書: 固有値による安定性判定の導出と、実装・実験との対応一覧 |
| 2026-10-03-learned-stability-region.md | 学習型アクターを並べたときの安定限界(理論と掃引実験) |
| 2026-10-03-transparent-boundary-stability.md | 透過境界条件の導出・離散化3方式の安定性 |
| 2026-10-03-nudging-stability.md | ナッジングの定理(γ≤2)と実験照合。※§3.2の観測点「8,24,40,56と仮定」は実装照合で 8,25,41,57 と判明(予測値は4桁で不変。theory/README.md) |
| 2026-10-03-levenberg-marquardt.md | 最適化(LM法)の導出と実装対応 |
| stability_figs/ | 理論値の図2枚(transparent_rho.png・nudging_boundary.png) |
| learned_*.gp | 実測図の作図スクリプト(参照する .dat は transcribed/ にある。実測図の正本は figs/) |

previous_deck.pptx / .pdf(前回の発表スライド)はGoogleスライドで管理しているため
gitでは追跡しない(.gitignore)。

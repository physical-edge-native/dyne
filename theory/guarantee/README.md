# theory/guarantee/ — 合成の検証器(安定性の保証の理論コード)

差分法の担当計算機と学習型(時間離散化した物理情報ニューラルネットワーク)の担当計算機を
1次元の鎖に並べた構成を宣言的に書き(JSON)、理想化した1ステップの線形写像(更新行列)を組み立てて、
スペクトル半径による安定・不安定の判定と、安定限界 r*(r = αΔt/h²)の二分法による探索を行う。
作成: Claude Code、2026-11-08。経緯と結果は docs/2026-11-08-stability-checker.md。

**ここの数値はすべて理想化した理論の予測であり、シミュレータ・実機とはまだ照合していない。**

| ファイル | 内容 |
|---|---|
| checker.py | 検証器の本体(ライブラリ兼コマンド)。理想化の仮定は冒頭のコメント |
| test_checker.py | 既知の理論値の再現テスト(差分法1/2、学習型の分割0.7192/0.7073/0.5003、学習型1台の無条件安定、ナッジングの安定境界γ*、透過境界3方式の0.500/0.319/0.438)と、混在構成の行和ノルムの性質 |
| mixed_sweep.py | 差分法と学習型を混ぜた鎖の r* を計算し、out/ に表と図を出す |
| configs/*.json | 構成の記述例 |
| out/mixed_table.dat | 代表的な構成ごとの r* |
| out/mixed_size.dat / .png | 中央の区画の点数 k を変えたときの r* |
| wave2d.py | 2次元の波動方程式(crates/wave2d-actor)の安定性: フォン・ノイマンの方法(無限格子、限界 1/√2)と、実際の更新式から組み立てたコンパニオン行列(2時刻を並べた1ステップの写像)の固有値による有界な領域の限界(値固定・ゼロ勾配・吸収境界)。2026-11-11、docs/2026-11-11-wave2d.md |
| test_wave2d.py | wave2d.py の照合テスト(6件。test_checker.py とは独立) |
| wave2d_sweep.py | dyne-sim2d でクーラン数を限界をまたいで掃引し、発散の始まりと増幅率を理論と比べる |
| out/wave2d_sweep.dat / .png | その表と図(**シミュレータの実測**。上の「理論の予測」の注意の例外) |

## 実行(リポジトリ直下で。Windowsでは日本語出力のため PYTHONUTF8=1 を推奨)

    .venv\Scripts\python.exe theory\guarantee\checker.py theory\guarantee\configs\fd_learned_fd.json --scan
    .venv\Scripts\python.exe -m unittest theory/guarantee/test_checker.py
    .venv\Scripts\python.exe theory\guarantee\mixed_sweep.py
    .venv\Scripts\python.exe theory\guarantee\wave2d.py
    .venv\Scripts\python.exe -m unittest theory/guarantee/test_wave2d.py
    .venv\Scripts\python.exe theory\guarantee\wave2d_sweep.py [dyne-sim2d.exe]

必要なもの: numpy(判定)、matplotlib(作図のみ)。どちらも .venv に入っている。

## 構成の書き方

    {
      "alpha": 1.0,                      // 熱拡散率(省略時1)
      "h": 0.015625,                     // 格子間隔(省略時 1/(総点数−1))
      "blocks": [                        // 左から順に。type は "fd"(差分法)か "learned"(学習型)
        {"type": "fd", "points": 22},
        {"type": "learned", "points": 22}
      ],
      "boundary": {"left": "dirichlet", "right": "zero_grad"},
                                         // dirichlet(温度固定)/ zero_grad(断熱)/
                                         // transparent(透過)/ linear_extrap(差分法のみ)
      "transparent": {"mode": "semi", "tmax": 0.2},   // semi=半陰的 / delay=1ステップ遅延 / drop=直近項を落とす
      "nudging": {"gamma": 1.5, "obs": "center"},     // 観測点は "center"(各区画の 開始+点数/2)か格子点番号のリスト
      "r": 0.45                          // 判定する r(--r でも指定可)
    }

(JSONにコメントは書けないので、上の // は説明のため。configs/ の例を参照)

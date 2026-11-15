# PC 側の常駐版・衝突判定の箱化・未観測を物とみなす書き出し・CPU での RL の課題 (docs\2026-11-13-pc-pipeline-rl.md)

2026-11-13。Claude Code(この環境)。計画 docs/2026-11-11-two-week-plan.md の G5(PC 側の常駐版)・G7 の続き・
G8(RL の課題)と、ユーザー決定 B1/F2・B2・B3・B4・D2 の実装。コミットはしていない。

前提(CLAUDE.md): PC がするのは設定注入・監視・可視化・書き出しだけ。形(符号つき距離の場)の計算は
ノードの格子型アクターが行い、PC はその結果を区画ごとに受け取って並べ、USD(3 次元の場面の記述形式)に書き出す。
本書の常駐版もこの範囲を超えない。

---

## 1. 要旨

| 項目 | 結果 |
|---|---|
| B3 衝突判定を箱の基本形に | USD の衝突判定を「板 1 枚 = UsdGeom.Cube 1 つ(位置・z 軸回りの回転・大きさ + CollisionAPI)」に変えた(既定)。理想条件の 6 畳の部屋で USD の検査 PASS(箱 90 個、OpenUSD の検査の指摘 0 件)、MuJoCo(CPU で動く物理シミュレータ)の厳しい検査 PASS(止まる位置の差 −1.2〜+1.1 mm、めり込み最大 3.6 mm)。従来の三角形メッシュとまったく同じ数値。メッシュは `--collider-shape mesh` で残した |
| B1/F2 未観測を物とみなす | 統合の重み w = 0 の格子点を、貪欲法で長方形にまとめた箱(/Room/Collision/Unobserved)にした(既定で有効。見た目は変えない)。抜け出しの試験(多数の出発点 × 8 方向に押す)で、物の多い部屋・PAN4H・箱を動かす部屋のどれも **抜け出し 0 回**。無効にすると 835/880・71/528・445/520 回抜け出す |
| G5 常駐版 | 「形の区画のメッセージ」(MQTT のトピック dyne/node/<id>/shape、52 バイトの見出し + φ・d・w の f32)を定義し、PC 側の受信・まとめ・定期的な書き出し(時刻つきのフォルダ + latest/ への写し)・ダッシュボードの自動更新を作った。ファームはまだこのメッセージを送らない(§5.2 をファームウェアの担当へ)。シミュレータの出力を流す偽のノードで通しで動作を確認(16 台、40 秒で 624 通、壊れたメッセージ 0、ブラウザで再読み込みなしに表示が差し替わる) |
| D2 | paho-mqtt 2.2.0 を .venv に入れ、PC の道具を .venv で動かすようにした。時刻同期・OTA の検査(-E2E つき)が .venv で ALL PASS |
| ネットワーク | Windows の「インターネット接続の共有」の手順(§7.1)と、代わりの小さな DHCP サーバ(標準ライブラリだけ。自己検査 PASS、実網では未検証) |
| G8 RL(CPU) | Gymnasium の形の課題(差動駆動の台車が目標地点へ)と PPO の学習。40 万手を CPU で 526 秒、到達率 0% → 8 万手で 98%、学習後の評価で 98%(学習した部屋)。1 環境の速さは 1 秒あたり 1550 手(物理 15500 段)。Isaac Lab 版の骨組みは未検証 |
| 分度器 | A4 横・半径 135 mm・0.5 度おき・中心の十字・100 mm の物差しつき(tools/print/) |

---

## 2. 変更・追加したファイル

| ファイル | 変更 |
|---|---|
| scripts/twin_physics.py | 箱の基本形の衝突判定(collider_boxes / colliders)、未観測の格子点を長方形にまとめる関数(greedy_rectangles / unobserved_colliders)、MJCF(MuJoCo の模型の記述形式)に格子の範囲と天井の高さを `<custom>` で記録 |
| scripts/twin_demo.py | `--collider-shape box\|mesh`(既定 box)、`--unobserved-solid on\|off`(既定 on)、`--unobserved-min-w`、`--json`、`--scene` を省略可に(図面なしは格子の範囲を部屋とみなす)、場面ファイルの `movebox`/`move` を読み飛ばす(動く箱は Other のまま。決定 B2)、正解の USD も箱に |
| scripts/twin_demo_view.html | `?poll=秒` の自動更新(§5.4)。**既存の不具合の修正**: 「USD」ボタンの alert の文字列に改行がそのまま入っていて(2026-11-11 の変更)、ページの台本全体が構文の誤りで動いていなかった。`\n` に直した |
| scripts/usd_check.py | 箱の基本形の検査(大きさ・extent・変換の種類・せん断なし・鏡映なし)、/Room/Collision の直下の名前ごとに外接箱をまとめる、Isaac 向きの指摘(近似 none の三角形メッシュ)と `--isaac-strict` |
| scripts/mjcf_check.py | USD の箱を向きつきで MuJoCo に写す(xyaxes)、名前の重複を避ける、出発点を推定の衝突判定に重ならない点から選ぶ、抜け出しの試験 `--escape` |
| scripts/twin_demo.ps1 | MuJoCo の検査に `--escape` を追加 |
| scripts/check-timesync-ota.ps1 | -E2E の部分で素の `python` を .venv の python に(D2) |
| scripts/mqtt_observe.py, scripts/mqtt_config.py | 説明文の paho-mqtt の入れ方を .venv に |
| scripts/requirements-pc.txt(新) | PC の道具の依存の一覧(.venv に入れる) |
| scripts/dyne_shape_msg.py(新) | 形の区画のメッセージの組み立て・読み取り(参照実装と自己検査) |
| scripts/twin_fake_nodes.py(新) | dyne-sim2d の出力をノードごとの区画として MQTT に流す偽のノード |
| scripts/twin_live.py(新) | 常駐版(受信 → まとめ → 定期的な書き出し → HTTP でダッシュボードを配る) |
| scripts/twin_live_demo.ps1(新) | 実機なしの通しの試験(シミュレータ → 偽のノード → 常駐版) |
| scripts/dyne_dhcp_server.py(新) | 小さな DHCP サーバ |
| scripts/rl_room_env.py(新) | RL の課題(Gymnasium の形、MuJoCo、CPU) |
| scripts/rl_train_ppo.py(新) | PPO の学習・学習曲線・評価 |
| scripts/isaaclab_nav_task.py(新) | 【未検証】Isaac Lab 版の課題の骨組み |
| tools/print/protractor_a4.py・.pdf・.svg(新) | 印刷する分度器 |

docs/ は本書だけを追加した(STATUS.md・既存の文書は編集していない)。

---

## 3. 衝突判定を箱の基本形にした(決定 B3)

### 3.1 形

- 推定表面の折れ線を間引き、物の側へ厚みのある板を立てるところまでは従来どおり(docs/2026-11-11-usd-physics.md §4.1)。
- 板 1 枚を `UsdGeom.Cube`(大きさ 1)に `xformOp:translate`・`xformOp:rotateZ`・`xformOp:scale` を付けて置く。
  `extent` も書く。各 Cube に `PhysicsCollisionAPI` と物性の割り当て(MaterialBindingAPI、purpose = physics)。
- 構成: `/Room/Collision/Walls`(Xform、見えない)の下に `Box_0000`, `Box_0001`, …。家具・Other・Unobserved も同じ。
  **親のパスは書き出し直しても変わらない。子の番号は板の数が変われば変わる**(参照する側は親のパスを使う)。
- 理由: Isaac Sim(PhysX)は三角形メッシュの衝突判定を静的な物にしか使えず、近似の指定(convexHull など)に依存する。
  箱なら近似が要らず、静的でも動かす物でも使える。MJCF は以前から box の geom なので変わらない。
- 正解の形の USD(比較用)も、壁は箱、長方形の家具は箱 1 つ、円柱・多角形の家具は凸の閉じたメッシュ + 近似 convexHull にした。
- 従来の閉じた三角形メッシュは `--collider-shape mesh` で書ける。

### 3.2 検査の結果(理想条件の 6 畳の部屋 room6、demo_out/twin/shape.csv から)

| 検査 | 箱(新しい既定) | メッシュ(従来、`--collider-shape mesh --unobserved-solid off`) |
|---|---|---|
| usd_check.py | PASS。箱 90 個(Walls 29・Furniture_0 16・Furniture_1 23・Furniture_2 9・Unobserved 13)問題 0、UsdValidation の指摘 0 件、Isaac 向きの指摘 0 | PASS(Isaac 向きの「近似 none のメッシュ」の注意 1 件。合否には含めない) |
| 外接箱の正解との差(Walls / 机 / ベッド / 本棚) | 7.1 / 23.4 / 19.8 / 17.6 mm(はみ出しは許容 30 mm 以内) | 同じ |
| mjcf_check.py --strict(+x/−x/+y/−y の止まる位置の差) | +1.1 / +0.2 / −1.2 / −0.1 mm、めり込み最大 3.6 mm。MJCF 経由と USD 経由で同じ値 | 同じ(USD 経由) |
| 抜け出しの試験(半径 0.08 m、66 点 × 8 方向) | 0 / 528 回 | − |
| 正解の USD(--truth-usd) | PASS(箱 7 個) | − |

数値は docs/2026-11-11-usd-physics.md §5.1 の表と同じ(+1.1 mm など)。箱にしても形は変わっていない。

---

## 4. 未観測を物とみなす(決定 B1/F2)

### 4.1 作り方

- 格子点のうち統合の重み w ≤ `--unobserved-min-w`(既定 0)の点を「未観測」とし、その点を中心とする一辺 h の升を物とみなす。
- 升の集まりを、行ごとに左から「右へ伸ばせるだけ伸ばし、同じ幅で上へ伸ばせるだけ伸ばす」貪欲法で重ならない長方形に
  まとめ、1 つの長方形を 1 つの箱(床から天井まで)にする。room6 では未観測 3946 点 → 箱 13 個。
- RL・物理の書き出し(USD の /Room/Collision と MJCF)にだけ入れる。見た目(/Room/Visual・HTML の推定の形)は変えない。
  HTML の木には「未観測(物とみなす)」として、既定で非表示で入る(「衝突判定」ボタンで見える)。
- 既定で有効。`--unobserved-solid off` で従来どおり穴のまま。
- 未観測の升は観測済みの点の側へ最大 h/2 はみ出す(壁際の影になった所で、部屋の内側へ最大 h/2 = 2.5 cm 程度)。安全側。

### 4.2 抜け出しの試験(mjcf_check.py --escape)

正解の部屋の空き(正解の面から 0.11 m 以上離れた点)に 0.3 m おきに半径 0.08 m の円柱を並べ(互いには当たらない)、
8 方向へ 12 秒押す。中心が正解の部屋の外か家具の中に入ったら「抜け出し」。出発点で推定の物(未観測の箱など)に
重なる円柱は除く。いずれも MJCF で実施(USD 経由が同じ結果になることは §3.2 で確認)。

| 部屋(入力) | 未観測の割合 → 箱 | 有効: 抜け出し / 試行(除いた出発点) | 無効: 抜け出し / 試行 |
|---|---|---|---|
| 物の多い部屋(twin_demo.ps1 -Preset cluttered。センサ 4 台、見える面の覆い率 5.2%) | 78.3% → 445 個 | **0 / 48**(110 点中 104 点が物の中で除外) | 835 / 880 |
| PAN4H(docs/2026-11-11-sensor-coverage-study.md。覆い率 84.9%) | 31.0% → 687 個 | **0 / 168**(66 点中 45 点除外) | 71 / 528(例: ベッドの面 x = 2.6 m を通り抜けた) |
| 箱を動かす部屋 room6_move の 600 ステップ(箱は目印 B。§5.5 の条件。覆い率 9.1%) | 53.4% → 291 個 | **0 / 128**(66 点中 50 点除外) | 445 / 520 |

- 有効にすると、どの部屋でも穴を通って抜け出さない。代わりに、観測の少ない部屋では**台車の動ける所が大きく減る**
  (物の多い部屋では出発点の 95% が物の中、RL の課題で置ける点は room6_move で 75 点(0.1 m おき)・最大のつながった
  領域 34 点 = 約 0.34 m²)。実機のセンサの数・向きで覆い率が上がらないと、RL の課題として狭すぎる部屋になる。
- 球の落下・円柱を押す試験(--strict なし)は 3 部屋とも有効の場合に PASS(めり込み 1 cm 未満)。
- 試験の出発点の選び方を変えた: 以前は正解の部屋で最も空いた点に置いていたが、未観測の箱の中に置いてしまうので、
  推定の衝突判定に重ならない点から選ぶようにした(room6 は従来と同じ (1.55, 1.50))。

---

## 5. PC 側の常駐版(G5)

### 5.1 流れ

```
ノード(格子型アクター)──MQTT dyne/node/<id>/shape──▶ twin_live.py
   区画の φ・d・w                                       ├ ノードごとに最新の区画を持つ
                                                        ├ --every 秒ごと(新しい区画があれば)全体の格子に並べる
                                                        │   届いていない区画は未観測(w = 0)
                                                        ├ twin_demo.py を呼ぶ → frames/<UTC 時刻>_<通し番号>/room.{usda,xml,html,json}
                                                        ├ latest/ に同じ名前で写す(一時ファイル → os.replace)+ latest/status.json
                                                        └ HTTP(既定 127.0.0.1:8765)で出力先を配る
ブラウザ: http://127.0.0.1:8765/latest/room.html?poll=3  … 3 秒ごとに latest/room.json を読み、変わっていれば差し替え
```

- 時刻つきのフォルダは `--keep`(既定 30)個まで残し、古いものから消す。Windows の記号リンクは管理者の権限が要るので、
  「最新」はリンクでなく写し(latest/)にした。
- USD の prim のパスは書き出しごとに同じ(/Room/Collision/Walls など。§3.1 の注意: 子の箱の番号は変わる)。
  USD の customLayerData の dyne に sequence(通し番号)と createdUtc(作成時刻)を記録する。
- 図面(実験4)を場面ファイルの書式で書けば `--scene` で渡せる(家具の名前・高さ・図面との誤差が出る)。
  省略すると格子の範囲を部屋とみなし、家具は Other、高さは `--furniture-height` の仮の値(決定 B4)。
- 送るのはノード、まとめるのは PC。PC は計算をしない(並べて書き出すだけ)。

### 5.2 形の区画のメッセージ(ファームウェアの担当へ: 未実装)

今のファーム(node-esp32/main/node_v2.c)は `dyne/node/<idx>/field2d` に要約(ステップ・区画・φ の CRC・最小・最大)しか
送らない。PC で形を組み立てるには区画の値そのものが要るので、次のメッセージを定義した。PC 側の参照実装は
scripts/dyne_shape_msg.py(`pack` / `unpack`、自己検査あり)。

- トピック: `dyne/node/<node.idx>/shape`、QoS 0、retain なし、ペイロードはバイナリ(リトルエンディアン)
- 送る間隔: 設定注入の `mode` に `"shape_every": k`(k ステップごと。0 = 送らない)を足す案。PC 側は届いた最新を使うので、
  間隔は自由(1 秒に 1 回程度を想定)。

| 位置 | 大きさ | 型 | 中身 |
|---|---|---|---|
| 0 | 4 | char | 識別 "DSF1" |
| 4 | 1 | u8 | 版 = 1 |
| 5 | 1 | u8 | 値の符号化 = 1(φ・d・w を f32 で 3 面) |
| 6 | 2 | u16 | 見出しの長さ = 52 |
| 8 | 2 | u16 | ノード番号(設定注入の node.idx) |
| 10 | 2 | u16 | 予約 = 0 |
| 12 | 4 | u32 | ステップ(この値を計算し終えたステップ) |
| 16 | 8 | u64 | 時刻印 [µs](時刻同期した時計。同期前は 0) |
| 24 | 2×4 | u16 | ix0, iy0(区画の左下の格子点の全体での番号)、nx, ny(区画の格子点の数)= 設定注入の field2d と同じ値 |
| 32 | 2×2 | u16 | gnx, gny(全体の格子の点の数。分からなければ 0) |
| 36 | 4×3 | f32 | h(格子の刻み [m])、x0, y0(全体の格子の原点 [m]) |
| 48 | 4 | u32 | 本体の CRC-32(IEEE 802.3。zlib.crc32 と同じ。dyne_wire2 の外箱と同じ多項式) |
| 52 | 12·nx·ny | f32 | φ[nx·ny]、d[nx·ny]、w[nx·ny](格子点 k = j·nx + i の順。`dyne_get(k)`、`dyne_get_aux(AUX_D=1, k)`、`dyne_get_aux(AUX_W=2, k)` を f32 にしたもの) |

- 大きさ: room6_move(4×4 ノード、h = 5 cm)で区画 18〜19 × 13〜14 点 → 1 通 約 3.1 kB。room6(8×4 ノード、h = 2.5 cm)
  で区画 18 × 27 点 → 約 5.9 kB。UDP の 1 データグラムの上限は関係ない(MQTT は TCP)。
  **要確認**: ESP-IDF の MQTT の送信で 6 kB の 1 通が送れるか(送信の緩衝の大きさ `buffer.out_size` と、分けて書く動作)。
- gnx・gny・h・x0・y0 はノードが知らなければ 0 でよい(PC は区画の最大から全体の大きさを決める)。h と原点は全ノードで同じでなければ
  PC はまとめない(誤りを表示して飛ばす)。
- 未対応: f32 より小さい符号化(版 2 で量子化を足す余地として「値の符号化」の欄を置いた)。

### 5.3 実機なしの通しの試験

`scripts/twin_live_demo.ps1`(シミュレータ → 偽のノード → 常駐版)。偽のノード(twin_fake_nodes.py)は dyne-sim2d の
`--field-csv` と同じ実行の記録(node 行の区画)を読み、区画ごとに上の形式で流す。計算はしない(結果の再生)。

場面: room6_move.txt、4×4 ノード、雑音 8 mm、推奨の忘却の設定(食い違い 0.1 m + 減衰 0.9 + 忘却 0.3、
docs/2026-11-11-dynamic-shape.md §6)。280 ステップ(箱は目印 A)と 600 ステップ(目印 B)の 2 つの場を 15〜20 秒ごとに
入れ替えて流した(実験6 で箱を動かす様子の代わり。中間の過程は流していない)。どちらも PASS (bit-exact)。

| 測ったもの | 値 |
|---|---|
| 受信 | 16 ノード、40 秒で 544〜624 通、壊れていたもの 0 通 |
| 書き出し 1 回の時間(格子 73 × 55) | 5.1〜6.5 秒(他の重い計算と同時に動かした値。最大 39 秒まで延びた)。room6(格子 145 × 109)の twin_demo.py 単体は 3.5 秒 |
| 時刻印の遅れ(まとめた時点 − 区画の時刻印) | 0.01〜1.4 秒(偽のノードは PC の時計で印を付ける。1 秒ごとに送るので最大約 1 秒は送る間隔) |
| 場面の切り替わり | 箱の移動の前後(ステップ 279 → 599)で、板の数 26 → 19、Other の板は 12 → 12。切り替わりの途中の書き出しではステップが 279〜599 で混ざる(ノードごとに届く時刻が違う) |
| ダッシュボード(Chrome) | `?poll=3` で開き、通し番号 #5 → #8 に**ページを読み直さずに**差し替わった(視点・表示の切り替えは引き継ぐ)。常駐版を止めると「常駐版: 読めない」を赤で出し、最後の表示を残す |

### 5.4 ダッシュボードの自動更新(twin_demo_view.html)

- URL に `?poll=秒` を付けると有効。読む先は同じ名前の .json(`&src=` で変えられる)。
- JSON の `stamp`(sequence と createdUtc)が変わったら、表示のデータを差し替える(WebGL の緩衝を作り直す)。
  視点・選んだ要素・表示/非表示・木の開閉はパスで引き継ぐ。
- file:// では fetch が使えないので、常駐版の HTTP サーバ経由で開く。`?poll` なしなら従来どおりの単独の HTML。

---

## 6. paho-mqtt を .venv に(決定 D2)

- `.venv\Scripts\python.exe -m pip install "paho-mqtt>=2"` → paho-mqtt 2.2.0(既存の台本の `mqtt.CallbackAPIVersion.VERSION2`
  の書き方は 2.x 用で、そのまま動く)。
- paho を使う既存の台本: mqtt_observe.py・mqtt_config.py・dyne_ota.py・dyne_timesync.py(と node-esp32/host_test/fake_node.py)。
  どれも .venv の python で起動することを確かめた(`--help`)。
- check-timesync-ota.ps1 の -E2E の部分が素の `python` を呼んでいたので .venv の python に変えた。
  `powershell -ExecutionPolicy Bypass -File scripts\check-timesync-ota.ps1 -E2E` → **ALL PASS**(OTA の一斉配布 6 通りを含む)。
- RL のために gymnasium 1.4.0・stable-baselines3 2.9.2 も .venv に入れた(torch は既存の 2.15.0+cpu)。
- 依存の一覧: scripts/requirements-pc.txt(`.venv\Scripts\python.exe -m pip install -r scripts\requirements-pc.txt`)。

---

## 7. ネットワークの準備(ユーザーが設定を変える)

構成(docs/2026-11-11-order-list.md): PC ⇔ USB-C の有線 LAN の変換器 ⇔ PoE(LAN の線で電気も送る仕組み)の給電器 ⇔
TP-Link EAP653(無線の親機)⇔ ノード。ノードに IP アドレスを配る方法は 2 通り。

### 7.1 本命: Windows の「インターネット接続の共有」(ICS)

1. 変換器を PC に挿し、給電器・EAP653 をつなぐ。
2. `Win + R` → `ncpa.cpl`(ネットワーク接続の一覧)。変換器の口(「イーサネット 2」など。名前を控える)を確かめる。
3. **インターネットにつながっている口**(Wi-Fi など)を右クリック → プロパティ → 「共有」タブ。
4. 「ネットワークのほかのユーザーに、このコンピューターのインターネット接続をとおしての接続を許可する」にチェック。
   「ホーム ネットワーク接続」で手順 2 の変換器の口を選ぶ → OK。
5. 変換器の口の IPv4 アドレスが自動で **192.168.137.1/24** になる(ipconfig で確認)。以後この網の機器には 192.168.137.x が配られる。
6. EAP653 はこの網から IP アドレスを受け取る。EAP653 の設定(SSID・パスワード)は Omada のアプリか、EAP653 の Web の画面で行う
   (EAP の IP アドレスは ICS が配ったもの。分からなければ `arp -a` で 192.168.137.x を探す)。
7. ノードの MQTT の相手(ブローカー)と時刻同期の相手は **192.168.137.1** にする。
8. Windows のファイアウォール: この網が「パブリック」に分類されると受信が止まる。設定 → ネットワーク → 変換器の口 →
   「プライベート ネットワーク」にする。mosquitto(1883/TCP)・時刻のサーバ(dyne_ntp_server.py の口)・OTA の HTTP の口・
   常駐版の HTTP(8765/TCP。PC の中だけで見るなら不要)を許可する。mosquitto は既に `listener 1883`(全部の口)で動いている。

注意(要確認): ICS は再起動後に共有が外れる・アドレスが変わる事例がある。ICS が有効な間は Windows 自身が DHCP の口(67/UDP)を
使うので、§7.2 の DHCP サーバとは同時に使えない。ICS は PC の Wi-Fi のインターネットをノード側へも通す(ノードは
インターネットを使わないが、閉じたいなら §7.2 の方法)。

### 7.2 代わり: 手で固定のアドレス + 小さな DHCP サーバ(scripts/dyne_dhcp_server.py)

1. ICS を使っていたら外す(§7.1 の手順 4 のチェックを外す)。
2. `ncpa.cpl` → 変換器の口 → プロパティ → 「インターネット プロトコル バージョン 4 (TCP/IPv4)」→ 「次の IP アドレスを使う」:
   IP アドレス 192.168.50.1、サブネット マスク 255.255.255.0、デフォルト ゲートウェイは空 → OK。
3. DHCP サーバを起動(管理者の権限は不要。初回にファイアウォールの許可を求められる):
   ```
   .venv\Scripts\python.exe scripts\dyne_dhcp_server.py --bind 192.168.50.1 --range 192.168.50.100-192.168.50.199
   ```
   - 静的な割り当て: `--static aa:bb:cc:dd:ee:ff=192.168.50.11`(何度でも)
   - 既定では既定経路も DNS も配らない(閉じた網)。配るなら `--router 192.168.50.1 --dns 192.168.50.1`
   - 貸し出しの記録 demo_out/dhcp/leases.json(再起動しても同じ機械に同じアドレス)、記録 demo_out/dhcp/dhcp_log.csv
   - 安全のため 0.0.0.0(すべての口)には `--any` が無いと結び付けない(家庭の網に誤って配らないため)
4. ノードの MQTT・時刻同期の相手は 192.168.50.1。

自己検査(網を使わない): `.venv\Scripts\python.exe scripts\dyne_dhcp_server.py --selftest` → PASS
(DISCOVER/OFFER、REQUEST/ACK、使用中のアドレスへの NAK、静的な割り当て、空き無し)。**実網での動作は未検証**
(Windows で特定のアドレスに結び付けた口がブロードキャストの DISCOVER を受け取れることは、機材が届いてから確かめる)。

---

## 8. RL の課題(G8)

### 8.1 CPU 版(scripts/rl_room_env.py・rl_train_ppo.py)

- 部屋: twin_demo.py が書いた room.xml(床・壁・家具・未観測の箱。すべて静的な箱)。部屋の形はノード群の計算の書き出しで、
  RL の環境は形を計算しない。
- 台車: 差動駆動。寸法は Isaac Sim の Jetbot に寄せた**仮の値(要確認)**: 車輪の半径 0.0325 m、車輪の間隔 0.1125 m、
  車輪の最大の角速度 10 rad/s(最大 0.325 m/s)、衝突判定は半径 0.08 m・高さ 0.10 m の円柱。
  車輪と床の摩擦は計算せず、平面の 3 自由度(x・y・向き)に、車輪の速さから求めた速度を上限の力つきの速度の追従で与える。
  壁・家具には接触の力で止まる。物理の刻み 0.01 秒、行動は 0.1 秒ごと。
- 行動: 左右の車輪の角速度 [−1, 1]²。観測(21 個): 台車から見た目標の前・左、距離、前進・回る速さ、周り 16 方向の距離
  (MuJoCo の光線の計算、2 m で打ち切り)。報酬: 近づいた距離 × 10 − 0.01/手 − 0.05(触れている間)、到達(0.15 m 以内)で +5。
  200 手(20 秒)で打ち切り。
- 出発点と目標: MJCF の `<custom>` の格子の範囲の中で、台車を置いて 3 cm 以内に何も無い点(0.1 m おき)の、つながった最大の
  領域から選ぶ(目標までの直線距離 0.8〜2.5 m)。
- 学習: stable-baselines3 の PPO(近接方策最適化。方策の更新幅を制限する強化学習の手法)、方策の網 64-64、環境 8 個を別プロセスで。

結果(room6 の推定の部屋、未観測の箱あり。demo_out/pc_pipeline/rl/):

| 手数 | 平均の報酬(直近 100 回) | 到達率 |
|---|---|---|
| 4,096 | −1.65 | 0% |
| 20,480 | 3.44 | 1% |
| 40,960 | 15.2 | 74% |
| 81,920 | 17.6 | 98% |
| 401,408 | 18.0 | 100% |

- 40 万手で 526 秒(更新を含めて 1 秒あたり 760 手。他の重い計算と同時に動かした値)。学習曲線: demo_out/pc_pipeline/rl/curve.png・curve.csv。
- 学習後の評価(決定的な方策、100 回): 学習した部屋 98%、**学習していない部屋**(形が変わった USD の代わり): room6_move の
  600 ステップの部屋 99%、PAN4H の部屋 92%。ただし room6_move は未観測の箱で置ける領域が約 0.34 m² しか無く、目標が近い
  (§4.2)ので易しい。壁・家具に触れた手は 100 回で 399〜1760。
- 課題は易しい(8 万手で頭打ち)。家具を回り込む必要のある目標の選び方・狭い部屋は今後の調整(ユーザーの判断)。

1 環境の速さ(無作為な行動、1 プロセス):

| 部屋 | geom の数 | 1 秒あたりの手数 | 物理の段 / 秒 |
|---|---|---|---|
| room6(箱 90) | 94 | 1550 | 15,500 |
| room6_move 600 ステップ(未観測の箱 291) | 314 | 564 | 5,640 |
| PAN4H(未観測の箱 687) | 888 | 567 | 5,670 |

geom が多いと光線の計算(16 方向 × 全 geom)が重くなる。未観測の箱を減らす(格子より粗くまとめる)か、光線の計算を
`mj_multiRay` にすれば速くなる見込み(未実施)。

### 8.2 Isaac Lab 版の骨組み(scripts/isaaclab_nav_task.py、未検証)

- Isaac Lab 2.x の DirectRLEnv。部屋は USD の /Room を参照で取り込んで環境の数だけ複製、台車は Isaac 標準の Jetbot。
  観測・行動・報酬は CPU 版と同じ形。
- 未実装・要確認: 周り 16 方向の距離(RayCaster。骨組みでは 0 で埋めた)、Jetbot の車輪の関節の名前と速度制御の設定、
  資産のパス、出発点と目標の選び方(骨組みでは格子の中央 60% から一様に選ぶので家具の中に入りうる。CPU 版の「置ける点」を
  書き出して渡すのが次の作業)。
- 借りた GPU では、まず scripts/isaaclab_room.py(取り込みの確認)→ この骨組みを無作為な行動で回す → 学習、の順。

---

## 9. 印刷する分度器(tools/print/)

- protractor_a4.pdf / .svg: A4 横、半径 135 mm の半円、0.5 度おきの目盛り(1 度・5 度・10 度で長さを変える)、10 度ごとの
  放射線、内外 2 通りの数字、中心の赤い十字、100 mm の物差し。
- 「実際のサイズ」(ページに合わせない)で印刷し、物差しが 100 mm かを定規で確かめる。作り直しは
  `.venv\Scripts\python.exe tools\print\protractor_a4.py`。

---

## 10. 実行方法(リポジトリの直下、PowerShell)

```
# 依存(初回だけ)
.venv\Scripts\python.exe -m pip install -r scripts\requirements-pc.txt

# 箱の衝突判定と未観測の箱つきで書き出し、検査(従来の一括。抜け出しの試験も入った)
powershell -ExecutionPolicy Bypass -File scripts\twin_demo.ps1

# 個別に
.venv\Scripts\python.exe scripts\twin_demo.py --scene crates\dyne-sim2d\scenes\room6.txt --shape-csv demo_out\twin\shape.csv `
    --sim-log demo_out\twin\sim.log --out demo_out\pc_pipeline\room6\room --zones 64 --fov-deg 60 --truth-usd
.venv\Scripts\python.exe scripts\usd_check.py demo_out\pc_pipeline\room6\room.usda --scene crates\dyne-sim2d\scenes\room6.txt
.venv\Scripts\python.exe scripts\mjcf_check.py --xml demo_out\pc_pipeline\room6\room.xml --usd demo_out\pc_pipeline\room6\room.usda `
    --scene crates\dyne-sim2d\scenes\room6.txt --strict --escape
#   従来の形: --collider-shape mesh / 未観測を穴のまま: --unobserved-solid off

# 常駐版(実機なしの通し。開く: http://127.0.0.1:8765/latest/room.html?poll=3)
powershell -ExecutionPolicy Bypass -File scripts\twin_live_demo.ps1 -Seconds 120
# 常駐版(実機。ノードが dyne/node/<id>/shape を送るようになってから)
.venv\Scripts\python.exe scripts\twin_live.py --broker 127.0.0.1 --scene <実験4の図面.txt> --every 5 --min-nodes 16
# 偽のノードだけ
.venv\Scripts\python.exe scripts\twin_fake_nodes.py --frame <field.csv>:<sim.log> [--frame ...] --frame-seconds 20 --period 1 --loop
# 形式の自己検査
.venv\Scripts\python.exe scripts\dyne_shape_msg.py

# RL(CPU)
.venv\Scripts\python.exe scripts\rl_room_env.py --xml demo_out\pc_pipeline\room6\room.xml --bench 10000
.venv\Scripts\python.exe scripts\rl_train_ppo.py --xml demo_out\pc_pipeline\room6\room.xml --steps 400000 --envs 8 --subproc `
    --out demo_out\pc_pipeline\rl --eval-xml demo_out\pc_pipeline\esc\move600_on.xml

# DHCP(§7.2)
.venv\Scripts\python.exe scripts\dyne_dhcp_server.py --selftest
.venv\Scripts\python.exe scripts\dyne_dhcp_server.py --bind 192.168.50.1 --range 192.168.50.100-192.168.50.199
```

§4.2 の 3 部屋の入力: 物の多い部屋 = demo_out/twin_cluttered/shape.csv(twin_demo.ps1 -Preset cluttered の出力)、
PAN4H = demo_out/coverage_study/PAN4H/shape.csv(場面 crates/dyne-sim2d/scenes/study/pan4.txt、--zones 16 --fov-deg 45)、
room6_move = dyne-sim2d を `--scene crates\dyne-sim2d\scenes\room6_move.txt --px 4 --py 4 --steps 600 --step-ms 66.667
--noise-m 0.008 --decay 0.9 --w-forget 0.3 --disagree 0.1 --disagree-w 1 --field-csv ...` で実行(twin_live_demo.ps1 と同じ)。
出力は demo_out/pc_pipeline/(git 管理外)。

---

## 11. 未検証・まだできていないこと

- **ファームが形の区画のメッセージを送らない**(§5.2。ファームウェアの担当)。実機からの受信は未検証。
- Isaac Sim / Isaac Lab での箱の衝突判定の取り込み、isaaclab_nav_task.py は未実行(GPU が無い)。
- DHCP サーバの実網での動作、ICS の手順は機材が届いてから確かめる。
- 常駐版の書き出しは 1 回 3.5〜6.5 秒(Python の twin_demo.py を毎回起動)。実験6 で「数秒で追従」を見せるには、書き出しの
  間隔 5 秒 + 書き出しの時間 + ノードの送る間隔が加わる。速くするなら温度のダミーと HTML を毎回作らない、などが要る。
- 偽のノードの再生は、箱の移動の前後の 2 つの場だけ(中間の追従の過程は流していない)。中間を流すには dyne-sim2d に
  一定のステップごとに場を書き出す引数が要る(シミュレータは他の担当の範囲なので触っていない)。
- 台車の寸法・車輪の速さ・摩擦・物性は仮の値(決定 B4)。

---

## 12. ユーザーが決めること

1. **未観測を物とみなす既定**: 有効(本書の既定)のままでよいか。観測の少ない部屋では台車の置ける所がほとんど無くなる(§4.2)。
   代わりの案: (a) 有効のまま、実機の覆い率を上げる(センサの向き・サーボで振る) / (b) 床の近くだけ観測されていれば空きとみなす
   ような緩め方(未観測の升を 1 升だけ縮める等) / (c) RL の書き出しを 2 通り(有効・無効)出して比べる。推奨は (a)+(c):
   穴を通り抜ける RL の結果は意味が無いので有効を本命に、無効は比較用に残す。
2. **形の区画のメッセージ(§5.2)の形式**: f32 の 3 面(1 通 3〜6 kB)でよいか。量子化(φ・d を 1 mm 単位の i16、w を u8)にすると
   約 1/2.4。推奨は、まず f32 で通し、帯域が足りなければ版 2 で量子化。
3. **最新の書き出しの置き方**: latest/ への写し(本書)でよいか(記号リンクは Windows で管理者の権限が要る)。
4. RL の課題の難しさ(目標の選び方)と、学習していない部屋での評価の仕方(実験7 の ③「形が変わった USD でも回す」)。
5. 台車の寸法の正本(Jetbot の実寸を Isaac の資産から読むか、実機の台車を買うか)。

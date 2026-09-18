# 発注7 実装レポート(下書き) (docs\2026-09-14-order7-report.md)

2026-09-14。発注7 = MQTT制御面第1段(7a)+実機ファーム更新(7b)。
根拠: 統合ロードマップv3(docs\2026-09-14-roadmap.md)3章原則7・4章実験1・6章。
規約の正本: docs\2026-09-14-mqtt-control-plane.md。

## 1. 7a: 制御面MQTT第1段

- **規約を文書化**(docs\2026-09-14-mqtt-control-plane.md): トピックをノードID・
  界面IDに対応させ(`dyne/node/<id>/...`、`dyne/iface/<id>/residual`)、各ノードが
  必ず応答すべきトピックとメッセージ形式を規定。独立の「軽量API」は設けず、
  **規約そのものが差込口**。
  - 設定注入: `dyne/cfg/<id>`(retained)で境界条件キー・担当区間・シード・
    モード・界面の結線(データ面UDPの相手)・観測周期を一括注入。応答義務
    `ack`(受理/拒否+ペイロードCRC32)。
  - 参加・離脱: `status`(retained)+**LWT**(遺言メッセージ)で不意の離脱も
    通知。`online` にMACとIPを載せ、RSSIのMAC→ノードID解決と結線情報の
    材料にする。
  - 観測: 温度場(field)・界面残差(residual、界面の左側ノードのみ=
    シミュレータの記録規則と同一)・リアルタイム率(rt)・センサ(sensor)・
    RSSI(rssi)。**ワイルドカード購読**(`dyne/node/+/field` 等)で全域観測。
  - 層の規律: データ面(40バイト意味論+UDP直送)はMQTTを通らない。制御面の
    MQTT依存はノード側1ファイル(dyne_mqtt.c)に閉じ、第2段(B-15判定次第の
    zenoh-pico移行)ではその層だけ差し替える。
- **PC側ツール**(ワイルドカード購読の実現):
  - scripts/mqtt_observe.py … `dyne/#` 購読+種類別CSV記録(field/sensor/rssi/
    rt/residual/status)。
  - scripts/mqtt_config.py … 1次元チェーン設定の生成・注入・start。格子分割は
    シミュレータと同一規則。`--absent`(部分観測)と `--bc` 4キー対応。
    transparent の指数和係数はPC側で計算して注入(規約§2.1)。
  - どちらも paho-mqtt 使用(pip導入済み)。ブローカーは mosquitto 等を想定
    (ユーザーのPCへ別途インストール)。

## 2. 7b: 実機ファーム更新(node-esp32/、m2-esp32ベース)

m2-esp32(2台固定・コンパイル時役割)を土台に、**設定注入駆動の汎用ノード
ファーム node-esp32/** を新設(m2はマイルストーン成果物として凍結)。
全ノード同一ファームウェアで、ブートストラップは menuconfig の3項目
(Wi-Fi・ブローカーURI・ノードID)のみ。

| 発注項目 | 実装 |
|---|---|
| (i) 規約への応答 | cfg受理(cJSONで検証、ack、状態遷移)、ctl(start/stop/report/reboot)、status+LWT、観測の周期発行。BSP協調はm2の単一界面ループを**最大2界面へ一般化**(送信控え・応答再送・再送バックオフ・終了時ドレインはM0/M1知見のまま) |
| (ii) センサ読取 | DHT20(0x38、CRC8検証)と BME280(0x76/0x77、データシート整数補償式)のI2Cドライバ(sensors.c、IDF 5.5の新i2c_masterドライバ)。切替は設定注入 `mode.sensor`。読取値は sensor トピックへログ発行のみ(同化接続=発注8) |
| (iii) F1境界条件キー | **4キーすべて対応**。dirichlet(値つき)/zero_grad/linear_extrap はbcコードの注入のみ(実機wasmをF1対応版heat1d_actorに更新)。transparent は指数和係数をPCが計算し設定注入→ファームは dyne_transparent_coef へ流し込むだけ(要否報告は§3) |
| (iv) RSSI収集 | AP受信強度+プロミスキャス受信による近傍MACごとの受信強度(EWMA+フレーム数)。データ面のUDPフレーム自体が測定源になるため計算参加ノード間は追加通信なし。rssi トピックへ周期発行(rssi.c、Kconfigで無効化可) |
| (v) リアルタイム率 | ステップ実時間・計算時間・同期待ちの統計(窓ごとの mean/min/max)と rt_rate = Δt/実ステップ時間、Δt余裕を rt トピックへ周期発行。有界非同期転換(12月)の布石データ |

## 3. transparent の実機実装の要否報告(発注7b(iii))

**実装済み・推奨「要」**。コストは次のとおり極小:

- **メモリ**: 新wasm(F1対応版)の線形メモリ宣言は**2ページ=128KBで旧版から
  不変**(補助変数・係数の静的領域 約2.6KB は既存ページ内に収まる)。wasmファイル
  +1,310B(2,181→3,491B)。設定注入の係数JSON約2KBは受信バッファ(8KB)内の
  一時消費のみ。
- **演算**: 1ステップ・1面あたり、半陰的更新の積和 約4×M=128回(M=32)。
  M1実測の1ステップ79.5msに対し完全に無視できる(µs級)。
- ファーム側に exp/log の実装は不要(係数はPCで計算して注入する規約)。
- **未実測(ユーザー実機確認待ち)**: 実機での実ステップ時間への影響と
  ヒープ実測。rt トピックの compute_ms を dirichlet 設定時と transparent 設定時で
  比較すれば1コマンドで検証できる。

## 4. 検証

| 項目 | 結果 |
|---|---|
| シミュレータ回帰の不変 | **合格**。本発注はシミュレータ系(crates/)に一切手を入れていない(git status で確認)。基本16ノードの再実行で PASS (bit-exact) を確認 |
| ファームのコンパイル | **合格**。ESP-IDF v5.5.5・target esp32 でフルビルド成功(dyne_node.bin = 1.13MB、アプリパーティション残27%、自作ソースの警告0) |
| 設定生成の妥当性 | mqtt_config.py の --dry-run で2ノード/部分観測+transparent(M=32係数)のJSON生成を確認。格子分割はシミュレータと同一規則 |
| **ユーザー実機確認待ち**(ハード依存で本環境では検証不能) | ①ESPr32×2以上での 設定注入→ack→start→BSP協調→done(CRC32)の一連(§5の手順)。②DHT20/BME280 の実測値が室温レンジで取れること。③RSSI(プロミスキャス)の近傍検出とCPU負荷。④リアルタイム率の実測値。⑤transparent 設定時の compute_ms 増分(§3)。⑥MQTT切断時のLWT発火 |

BSP協調の数値検証の手立て: `done` の CRC32(温度配列バイト列)を、シミュレータ
同一構成の値と照合する(M2と同じ流儀。実機は同一wasm・同一初期条件・同一交換
規則なのでビット一致が期待値。ただし2台とも完走した場合のみ意味を持つ)。

## 5. ユーザー実機確認の手順(最小2台)

1. PCで mosquitto を起動(未導入なら https://mosquitto.org/ から。
   `mosquitto -v` で1883待受、リスナーに `allow_anonymous true`)。
   PCのモバイルホットスポット(またはルーター)に全ノードとPCを接続。
2. 各ボード: node-esp32/ で `idf.py menuconfig` → Dyne Node(SSID・パスワード・
   `mqtt://<PCのIP>:1883`・ノードID 0/1)→ `idf.py -p COMx flash monitor`。
3. `python scripts\mqtt_observe.py --broker localhost` → 各ノードの
   `status: online`(MAC・IP)を確認。
4. `python scripts\mqtt_config.py --broker localhost --nodes 2 --points 65
   --steps 2000 --peers <IP0>,<IP1> --sensor dht20 --start`
5. 期待: ack(ok)→ configured → running(rt・field・residual が流れる)→
   約3分で done×2(CRC32)。mqtt_log/ にCSVが残る。
6. 比較用シミュレータ値:
   `dyne-sim.exe --wasm ...heat1d_actor.wasm --nodes 2 --points 65 --steps 2000`
   の各ノード温度配列のCRC32(手順は m2 の CRC照合と同じ)。

## 6. 変更ファイル一覧

- docs/2026-09-14-mqtt-control-plane.md … **新規**: 制御面規約(正本)
- node-esp32/ … **新規**: 汎用ノードファーム(main.c=BSP多界面化+設定注入、
  dyne_mqtt.c=MQTT層、sensors.c=DHT20/BME280、rssi.c、Kconfig.projbuild、
  README.md、F1対応版 heat1d_actor.wasm 同梱)
- scripts/mqtt_observe.py / scripts/mqtt_config.py … **新規**: PC側ツール
- .gitignore … node-esp32のビルド生成物と mqtt_log/ を追加
- docs/2026-09-14-order7-report.md … 本書
- docs/STATUS.md … 進捗反映
- シミュレータ系(crates/)・m2-esp32 は**無変更**

## 6.5 検収回答(ローカル分、2026-09-18)への追記3点

ローカル検証項目はすべて合格、transparent実機実装「要」承認済み。以下は
検収回答で求められた明文化(規約 §3.5 にも同内容を追記済み):

1. **LWT発火時の残存ノードのBSPループ**: **ストール**(再送バックオフを続けて
   その界面で待つ)。pause扱い(相手復帰での自動再開)ではない——再起動した
   相手はステップ0から始まり整合しないため。回収は `ctl: stop`(走行中の受信待ち
   ループが毎スライスで stop を確認する実装)→ 再設定 → 再start。離脱の自動治癒
   (handover/ADDの実機化)は第1段のスコープ外(発注9以降)。
2. **ackの再送・タイムアウト・部分成功**: cfgはretained+QoS1で、ノードは受信の
   たびにackを返す(冪等=再注入がそのまま再送)。規約: **startは全計算ノードの
   ack(ok)確認後に発行**、ack欠落はタイムアウト(目安10秒)でcfg再注入。部分成功
   のままstartした場合も挙動は定義済み(未受理ノードは status err で無視、受理側は
   ストール→stopで回収)。mqtt_config.py の --start は簡易(1秒猶予)であり、
   確実な運用は mqtt_observe.py でack確認後にstartのみ発行。
3. **プロミスキャスRSSIの縮退**: 2段階を用意。(a) 実行時: `rssi_period_ms: 0` の
   設定注入で**プロミスキャス受信そのものを停止**(受信コールバックのCPUコスト
   ゼロ。再注入で再開)——検収回答を受けて追加実装し、再ビルド成功を確認。
   (b) ビルド時: Kconfig DYNE_RSSI_SNIFF でスニファをコンパイルから外す。

## 7. 実装上の判断(事後報告の小さな判断)

1. **新ディレクトリ node-esp32/**(m2-esp32 は凍結保存)。マイルストーン成果物を
   壊さず、以後の実験1〜5の共通ファームとして育てる。
2. **Wi-FiはSTA専用**(APはPCのホットスポット/ルーター)。M2のノードA=AP方式は
   ブローカー(PC)との同一セグメント化が煩雑なため第1段では不採用。
   17ノード規模の通信構成はB-18(ロードマップ8章)で判定。
3. **transparent係数はPC計算・設定注入**。ノード側のexp/log再実装を避け、
   数値の出所を一元化(規約§2.1に明記)。
4. **近傍RSSIはプロミスキャス受信**でデータ面フレームを測定源にする(追加通信
   なし)。CPU負荷が問題になった場合に備え Kconfig で無効化可。
5. status の `done` に CRC32・L2/Linf を載せ、M2の検証流儀(CRC照合)を
   規約に引き継いだ。

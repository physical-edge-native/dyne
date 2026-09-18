# Dyne node-esp32: 設定注入駆動の汎用ノードファームウェア(発注7)

全ノード同一ファームウェア。役割は起動時のMQTT設定注入で決まる
(ロードマップv3 4章)。m2-esp32(2台固定・コンパイル時役割)がベース。
m2-esp32 はマイルストーン成果物として凍結し、以後の実機実験は本ファームを使う。

- 制御面の規約: `../docs/2026-09-14-mqtt-control-plane.md`(トピック・メッセージ形式)
- レポート: `../docs/2026-09-14-order7-report.md`

## 構成

- ブートストラップ設定(menuconfig「Dyne Node」): Wi-Fi SSID/パスワード、
  ブローカーURI、ノードID、I2Cピン(既定 SDA=21/SCL=22)、BME280アドレス。
  **それ以外の設定はすべてMQTTで注入**(担当区間・境界条件キー・界面の結線・
  シード・モード・観測周期)。
- データ面(境界同期)は従来どおり 40バイト意味論+UDP直送(ポート7777)。
- センサ: DHT20(0x38)/ BME280(0x76 or 0x77)。設定注入 `mode.sensor` で切替。
- RSSI: AP受信強度+プロミスキャス受信による近傍MACごとの受信強度。
- リアルタイム率: ステップ実時間・計算時間・Δt余裕の統計を周期発行。

## 使い方(最小2台)

1. PC側: mosquitto を起動(ポート1883、匿名許可)。PCのモバイルホットスポット
   (またはルーター)に全ノードとPCを同一セグメントで接続。
2. 各ボード: `idf.py menuconfig` → Dyne Node で SSID/パスワード/ブローカーURI/
   ノードID(0, 1, …)を設定 → `idf.py build flash monitor`。
3. 観測を開始: `python scripts/mqtt_observe.py --broker localhost`
   → 各ノードの `status: online`(MACとIP)が見える。
4. 表示されたIPで設定を注入し開始:
   `python scripts/mqtt_config.py --broker localhost --nodes 2 --points 65
    --steps 2000 --peers <IP0>,<IP1> --sensor dht20 --start`
5. `ack` → `configured` → `running` → 完走で `done`(温度配列CRC32つき)。
   2台のCRCの組をシミュレータの同一構成の値と照合すれば協調の検証になる。

## 備考

- Wi-FiはSTA専用(APはPC側)。M2のノードA=AP方式は第1段では使わない
  (制御面の中心=接続時のみのPC、の原則に合わせた)。
- `ctl: start` は再実行可(dyne_init のやり直しで場と透過境界の補助変数が
  リセットされる=担当替えのリセット規定と同じ扱い)。
- transparent の指数和係数はPCが計算して設定注入する(規約§2.1。ノード側で
  exp/log を再実装しない)。

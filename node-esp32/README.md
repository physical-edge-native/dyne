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
4. 設定を注入し開始 (`--peers auto` が status の online から IP を自動解決):
   `python scripts/mqtt_config.py --broker localhost --nodes 2 --points 65
    --steps 2000 --peers auto --sensor dht20 --start`
5. `ack` → `configured` → `running` → 完走で `done`(温度配列CRC32つき)。
   2台のCRCの組をシミュレータの同一構成の値(`dyne-sim.exe ... --crc`)と
   照合すれば協調の検証になる。
6. 復旧(ストール・離脱時): `python scripts/mqtt_config.py --broker localhost --stop`
   → 再設定(手順4)→ start(規約§3.5の stop→再設定→再start)。

## 備考

- **WASMは縮小版 heat1d_actor_small.wasm(線形メモリ1ページ=64KB)を使う**
  (2026-09-18 実機不具合対応(b)): ESPr32(PSRAMなし)はDRAM区画の都合で連続
  ヒープが最大約108KBのため、フル版の128KBは構造的に確保できない。縮小版は
  N_MAX=2048・スタック8KBで、数値経路はフル版と同一(シミュレータで全出力
  バイト一致を確認済み)。再生成は `scripts\build-actor-wasm.ps1`。
  WASM初期化はWi-Fiより先(起動直後の未断片ヒープで確保)。起動ログの
  `[dyne] heap(...)` 4点が余裕の定点観測。経緯はレポート§6.6。
- Wi-FiはSTA専用(APはPC側)。M2のノードA=AP方式は第1段では使わない
  (制御面の中心=接続時のみのPC、の原則に合わせた)。
- `ctl: start` は再実行可(dyne_init のやり直しで場と透過境界の補助変数が
  リセットされる=担当替えのリセット規定と同じ扱い)。
- transparent の指数和係数はPCが計算して設定注入する(規約§2.1。ノード側で
  exp/log を再実装しない)。

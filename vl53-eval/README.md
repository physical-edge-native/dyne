# vl53-eval: VL53L5CX 性能評価用ファーム(2026-10-10)

評価計画: `../docs/2026-10-10-vl53l5cx-eval-plan.md`。現役の node-esp32 とは独立した
小さなESP-IDFプロジェクト。測定値をUSBシリアルにCSVで流し、PC側の
`../scripts/vl53_log.py` が1条件=1つのCSVに記録する。Wi-Fi・MQTTは使わない。

- ドライバ: ESP-IDFコンポーネント rjrp44/vl53l5cx 4.0.1(ST ULD 2.0.0の移植、BSD-3-Clause)。
  初回ビルド時にレジストリから managed_components/ へ自動取得される(git管理外)。
- 対象: ESPr Developer 32 Type-C(ESP32)、ESP-IDF 5.5.5。
- 状態: ビルドは確認済み(2026-10-10)、実機は未確認。

## 配線

SparkFun Qwiic Mini ToF Imager(スイッチサイエンス SKU 7934)を Qwiicケーブル(4ピンメス)で:

| ケーブルの色 | 信号 | ESPr32 |
|---|---|---|
| 黒 | GND | GND |
| 赤 | 3.3V | 3V3(5Vにつながないこと) |
| 青 | SDA | GPIO21 |
| 黄 | SCL | GPIO22 |

INT・RSTは未接続。ケーブルの先はメスなので、オス-オスのジャンパ線で中継してブレッドボードへ。
実験1のDHT20を同じバスにつないだままでもよい(番地0x38と0x29で衝突しない)。

## 書き込み

ESP-IDFのPowerShell(スタートメニューの「ESP-IDF 5.5 PowerShell」等)で:

```
cd vl53-eval
idf.py -p COM7 build flash
```

起動直後の表示を確かめたいときは `idf.py -p COM7 monitor`(速度は sdkconfig の 921600bps を
自動で使う)。正常なら `#BOOT` → `#INFO i2c_scan=0x29 ...` → `#INFO ... init_ms=...` → `#CFG ...`
の順に出る。モニタは記録の前に必ず閉じる(ポートを取り合うため)。

## 記録

```
python -m venv .venv                               # 初回のみ (リポジトリ直下。git管理外)
.venv\Scripts\python.exe -m pip install pyserial   # 初回のみ (グローバル環境には入れない)
.venv\Scripts\python.exe scripts\vl53_log.py --port COM7 --out measured\2026-10-xx-vl53l5cx --label E1_white_1000mm --frames 300 --set "res 8" --set "hz 10" --note "白壁に正対、センサ前面から1000mm"
```

- 出力: `<label>.csv`(1行=1フレーム)と `<label>.meta.txt`(起動情報・設定・メモ)。
  同名のファイルがあれば上書きせずに止まる。
- 終わると中央4区画の平均・標準偏差・有効率を表示する(その場の確認用)。
- 設定コマンド(`--set` で送る。測定の停止中のみ):
  `res 4|8`、`hz N`、`mode cont|auto`、`it N`(積分時間ms。autoのときだけ効く)、
  `sharp N`(0〜99%)、`order closest|strongest`。

## 出力行の形式(ファーム→PC)

- `#` で始まる行: 情報と応答(`#BOOT` `#INFO` `#CFG` `#OK` `#ERR` `#DONE`)
- `F,t_ms,idx,nz,temp_c,` に続けて区画数nzずつ: 距離mm、状態値(5と9が有効)、
  ばらつき推定mm、信号の強さ、検出物体数、環境光

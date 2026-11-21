# measured/ — 実機実測の記録(正本)

実機実験で監視ツール(scripts\mqtt_observe.py)が記録したCSVを、実験日ごとに保存する。
.gitignore の `*.csv` の例外として git で追跡する(recovered/ と同じく、実測CSVの消失を防ぐため)。
削除・上書きはしない。

## 2026-10-08-exp1/ — 実験1(1次元温度場、ESPr32×4、DHT20、乾電池駆動)

当日の時刻つき記録: docs\2026-10-08-experiment1-log.md

| フォルダ | 内容 | 監視ツールの起動時刻(ts=0) |
|---|---|---|
| calib/ | 校正(18:39:39〜19:09:39、同一地点に4台)・補正値注入・配置・温度差の確認 | 18:37:32 |
| run1/ | 本計測run1(保留=node 0) | 19:59:11ごろ |
| run2/ | 本計測run2(保留=node 1) | 20:33:47ごろ |
| run3/ | 本計測run3(保留=node 2) | 21:07:50ごろ |
| run4/ | 本計測run4(保留=node 3)と撤収時のoffline(22:20〜22:21) | 21:41:43ごろ |
| mqtt_log_all/ | 既定の記録先 mqtt_log\ の一式(runbook §8の退避)。過去の作業日からの追記に、当日の午前(電池テスト・2台CRC、ts=0 は10:39:10)と夕方(組み立て・再書き込み、ts=0 は18:05:28)が続く。ts は起動ごとに0へ戻るので、ts の巻き戻りで区切る | 複数 |

- 各CSVの ts 列は監視ツールの起動からの経過秒。時刻に直すには上表の起動時刻を足す
  (run1〜3の起動時刻は、ack・running の記録と号令の実時刻から逆算した推定で±1秒程度)
- sensor.csv の temp_c は補正後の値(生値の列は無い。生値=temp_c−補正値。補正値は
  node 0〜3 の順に −0.06 / +0.13 / −0.03 / −0.04 ℃、19:10:05以降に適用)
- 各フォルダの冒頭には、ブローカーに保持されていた直前の状態(前のrunのdoneなど)が
  ts≈0 で入っている。そのrunの記録として扱うのは号令以降の行
- 主なファイル: status.csv(状態・online/offline)、sensor.csv(温度・湿度)、
  innov.csv(イノベーション=実測−モデル予測)、rt.csv(ステップ時間)、rssi.csv(受信強度)

## 2026-10-11-vl53l5cx/ — 距離センサVL53L5CXの評価(ESPr32×1、USB給電)

当日の記録: docs\2026-10-11-vl53l5cx-eval-log.md。評価計画: docs\2026-10-10-vl53l5cx-eval-plan.md。
記録ツール scripts\vl53_log.py が1条件=1つのCSV(1行=1フレーム)と同名の .meta.txt
(ファームの起動情報・送った設定・メモ)を書く。ファイル名の先頭は評価項目の番号(E0〜E9)。
CSVの列の意味は scripts\vl53_log.py の冒頭。

## 2026-11-17-siteA-wave2d/ — 最初のUSB書き込み(16台)と2次元の波動方程式の実機(2×2)(場所A)

場所A = 2026-11-27 までの仮の部屋(以後の場所Bとは部屋の形・電波の環境が違うので、遅れ・損失の値は直接比べない)。
当日のまとめ: docs\2026-11-17-wave2d-hw-day1.md。手順: docs\2026-11-16-wave2d-cluster-runbook.md。
ESP32-DevKitC-VE ×16、アクセスポイント経由の Wi-Fi、PC が MQTT のブローカーと時刻同期のサーバ(UDP 12300)。

| フォルダ・ファイル | 内容 | 元 |
|---|---|---|
| first_flash/first_flash.csv | USB 書き込み1回=1行(16台の初回と、修正版のファームでの書き直し16回) | first_flash_out\ |
| first_flash/logs/ | 各回の esptool と起動のログ。ファイル名の MAC も仮の値 | first_flash_out\logs\ |
| wave2d/runs/<日時>_<配置>_<場面>/ | scripts\wave2d_cluster.py run の1回分(expected.json・cfg_node<id>.json・result.json・result.csv・events.jsonl)。19:30:48 の吸収境界の回は結果が無い(node 201 の再起動で中断) | wave2d_out\ |
| wave2d/nodes.json | 最後に見えたノードの IP・MAC | wave2d_out\ |
| wave2d/ota_actor.csv・ota_fw.csv | アクター・ファームの OTA(無線での更新)の結果 | wave2d_out\ |
| wave2d/tsync_*.csv・tsync_record.txt | 時刻同期の記録(warmup=立ち上がり、rtt_oneway=600 秒の記録、record=その集計) | wave2d_out\ |
| wave2d/stall_run.txt | node 232 の電源を切った回(19:40:13 開始)の画面の出力 | wave2d_out\ |

- 写さなかったもの: wave2d_out\ntp_log.csv(空)だけ。events.jsonl はすべて写した(最大は --rt-every 1 の回の約 430 KB)。
- 置き換え(写すときに一回限りの処理で実施): 自分の基板の実在の MAC(最後の1バイトを YY として)→ **02:00:00:00:02:YY**
  (最後の1バイト=MAC 由来のノード番号を保つ)。最後の1バイトが 0xC8 で重なった2台のうち、NVS で 201 にした基板だけ
  **02:00:00:00:03:C8**(もう1台の node 200 は 02:00:00:00:02:C8)。アクセスポイントの BSSID → 02:00:00:00:00:01。
  大文字・小文字と区切りの書き方は元のまま。USB 変換チップのシリアル番号 → USBSERIAL01〜16(初出順、基板ごとに一定)。
  ログの Wi-Fi の SSID → (SSID)。stall_run.txt の手元の絶対パス → 相対パス。
- IP アドレス(アクセスポイントの内側の 192.168.50.x)は置き換えていない(runbook §8: 置き換えるかはユーザーが決める)。
- 時刻の列(t・pc_time・ts_us)は UNIX 時刻(秒・マイクロ秒)。events.jsonl の rt のトピックの番号は配置の番号(idx)で、
  ノード番号ではない(2×2 では idx 0〜3 = node 56・172・201・232)。

## 2026-11-21-siteA-imu/ — 加速度・角速度センサ 7台を1枚の板に貼った記録(場所A、run2)

当日のまとめ: docs\2026-11-21-imu-hw-run2.md。英語の報告: 文書のサイトの「IMU fusion on hardware (2026-11-21)」
(docs-site/results/imu-2026-11-21.md)。ESP32-DevKitC-VE ×7 に Arduino Modulino Movement(LSM6DSOX)を1台ずつ。
合板に貼り、PC の時刻同期のサーバ(UDP 12300)に7台とも合わせてから記録した。出力 12.5 Hz、1 秒ごとにまとめて MQTT で送る
(1台あたり約1通/秒の上限のため。GitHub dyne Issue #114)。

| フォルダ・ファイル | 内容 |
|---|---|
| run2/imu_raw.csv | 標本1つ=1行(node、seq、ts_us=同期した時刻 [UNIX µs]、local_us=台の時計、ax〜gz [m/s²・rad/s]、temp_c は空) |
| run2/imu_batches.csv | 届いたバッチ1つ=1行(pc_time=PC に届いた時刻 [UNIX s]、batch=通番、n=標本の数、時刻合わせの状態) |
| run2/imu_fused.csv | 見出しだけ(隣だけの融合のアクターは今回動かしていない) |
| run2/events.txt | 当日の号令の記録(日本時間)。静止 06:40:10〜06:53:40、傾き 0°/9.00°/17.70→17.60°/0.10°(デジタル角度計の読み)、揺らし 07:04:25〜07:05:25 |
| run2/level.csv | events.txt から作った、傾きの各区間(両端5秒を除く)の時刻と角度計の読み(tools/make_level.py) |
| analysis/imu_analyze.txt・summary.json | scripts\imu_analyze.py(GitHub dyne Issue #121 の修正後)の出力。静止 = 最初の標本から 34.5〜834.5 s |
| analysis/extra.txt・extra.json | tools/extra.py の出力: 台ごとのゆっくりした変化、232 を除いた6台の利得、揺らしでの時刻の比較 |
| tools/ | 上の解析と、文書のサイトの図(tools/figs.py)を作る台本。リポジトリ直下から `python measured/2026-11-21-siteA-imu/tools/extra.py measured/2026-11-21-siteA-imu/run2 1795212265000000 1795212325000000` |

- 置き換え: 不要だった(これらのファイルに MAC アドレス・IP アドレス・SSID は入っていない。node はノード番号)。
- 7台の板の上での向きは目で合わせた(±10° 程度)。node 232 は記録の前にセンサを一度抜き差しした台で、静止の間に
  加速度の y が段状に約 0.045 m/s² 変わった(取り付けがずれた疑い)。
- 1回目の記録(run1: 3台が時刻合わせできていなかった)は公開していない。

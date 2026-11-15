# 時刻同期と OTA(無線経由の書き換え)をファームと PC 側ツールに入れた (docs\2026-11-11-timesync-ota.md)

作成: 2026-11-11(Claude Code)。対象: docs/2026-11-11-two-week-plan.md の P6(時刻同期と OTA を必ず実装)、
§5 の G1(時刻同期)・G2(OTA)・G11(再起動の理由と稼働時間の報告)、§4 の実験1(時刻同期のずれの実測)・
実験2(OTA の一斉配布)。

**実機では一度も動かしていない**(XIAO ESP32C5 は未着、ESPr も未接続)。確認できたのはビルドと、PC 上の
検査(C の計算部分・PC 側ツール・ノードのふりをするプログラムを使った通しの確認)まで。§12 に未確認の点を列挙する。

用語(本書での意味):
- **OTA**(Over-The-Air、無線経由の書き換え): USB をつながずに、Wi-Fi 経由でファームや計算部品を書き換えること。
- **計算部品**: WASM アクター(.wasm)と、その事前コンパイル版(.aot。WAMR の AOT 形式)。
- **SNTP**(簡易版ネットワーク時刻プロトコル): ESP-IDF 標準の時刻合わせ。ノードのシステムの時計を合わせる。
- **往復の時刻印の交換**: ノード送信 t1 → PC 受信 t2 → PC 送信 t3 → ノード受信 t4 の4つの時刻から、
  ずれ = ((t2−t1)+(t3−t4))/2 と往復時間 = (t4−t1)−(t3−t2) を求める方法(NTP と同じ式)。
- **同期した時刻**: ノードの esp_timer(起動からの µs)に、往復の交換で推定したずれを足した時刻。
  単位は 1970-01-01 からの µs で、基準は PC の時刻サーバ。
- **ドリフト**: ノードの水晶発振子の周波数の誤差による、ずれの時間変化(ppm = 100万分の1)。
- **試験モード**: 同期した時刻の整数秒ちょうどに GPIO を切り替えるモード。ロジックアナライザで
  ノード間のずれを直接測るために使う。
- **自己確認**: OTA で書き換えた直後の起動で、Wi-Fi・MQTT・起動時の自己検査がそろったかを確かめること。
  そろわなければ前のファームに戻す(ESP-IDF のアプリのロールバック機能)。

---

## 1. 結論の要約

| 項目 | できたこと | 確認の方法 |
|---|---|---|
| G1 時刻同期 | SNTP + 独自の往復の交換(最小の往復時間の標本を採り、直近32標本に直線を当てはめてずれとドリフトを推定)。推定の質(往復時間・推定との差・ドリフト)を MQTT で報告。センサ・RSSI・ワイヤ形式v2 の時刻印の欄に同期した時刻を付ける。試験モード(整数秒でピンを切り替え、同期あり/SNTP のみ/同期なしの3条件) | C の計算部分を PC で模擬(揺らぎ小で最大誤差 14µs、ドリフト誤差 0.06ppm)。PC の時刻サーバと C・Python の問い合わせ側で実際に UDP の往復 |
| G1 PC 側 | 標準ライブラリだけの NTP サーバ(scripts/dyne_ntp_server.py)、設定と記録(scripts/dyne_timesync.py)、ロジックアナライザの記録の解析(scripts/pps_analyze.py) | 合成した記録で解析の正しさを確認 |
| G2 OTA | パーティションテーブルを アプリ2面 + otadata + 計算部品のパーティション に変更(4MB と 8MB の2種)。ファームの OTA(HTTP、像の SHA-256 の照合、自己確認に失敗したら元に戻す)。計算部品の OTA(2枠、SHA-256 と長さを照合、前の計算部品を残す、壊れていれば前の枠 → 埋め込みの順に戻る) | ESP32・ESP32-C5 の両方でビルド成功。壊した像を esptool で独立に検査。ノードのふりをするプログラムで PC 側ツールを通しで確認(6通りすべて期待どおり) |
| G2 PC 側 | 一斉配布ツール(scripts/dyne_ota.py): HTTP で配り、MQTT で N 台へ指示し、結果と所要時間を CSV に。わざと壊したファーム・計算部品を作って配れる | 同上 |
| G11 | 起動時に再起動の理由(電圧低下 brownout を含む)・起動回数・**前回の稼働時間**を online の status で、稼働時間などを 10 秒ごとに health で報告 | ビルドのみ |
| 物理シミュレーション環境 | 作っていない(指示どおり)。§10 にモデル化の方針だけを書いた | — |

---

## 2. 変更・追加したファイル

| ファイル | 内容 |
|---|---|
| node-esp32/main/dyne_timesync_core.c/.h(新規) | 時刻同期の計算部分(NTP パケット、ずれ・往復時間、外れ標本の除去、直線の当てはめ、次の整数秒)。ESP-IDF に依存しない |
| node-esp32/main/dyne_timesync.c/.h(新規) | SNTP の起動、往復の交換のタスク、tsync の報告、試験モード(ピンの切り替え) |
| node-esp32/main/dyne_actor_slot.c/.h(新規) | 計算部品のパーティションの見出しの形式と、どの枠を使うか・どこへ書くかの規則。ESP-IDF に依存しない |
| node-esp32/main/dyne_ops.c/.h(新規) | ファームと計算部品の OTA、計算部品の読み出し、起動後の自己確認と元に戻す処理、再起動の理由・稼働時間の報告、NVS の初期化 |
| node-esp32/main/main.c | 計算部品をパーティションから先に試す(無ければ埋め込み)。計算部品が積めなくても Wi-Fi・MQTT は上げる(OTA で直すため)。ctl の新しい命令の受け口。計算中の印(g_busy)。innov に y_ts_us |
| node-esp32/main/dyne_mqtt.c | online の status に対象チップ・版・起動パーティション・再起動の理由・計算部品を追加。全 status に up_s |
| node-esp32/main/dyne_node.h | g_busy の宣言 |
| node-esp32/main/node_v2.c | ワイヤ形式v2 の経路で、時刻印の欄が 0 のメッセージに同期した時刻を付ける |
| node-esp32/main/sensors.c, rssi.c | JSON に ts_us(同期した時刻)を追加。t_ms は従来どおり |
| node-esp32/main/Kconfig.projbuild | 時刻サーバ・ポート・周期・試験モードのピン・自己確認の期限・health の周期 |
| node-esp32/main/CMakeLists.txt | 新しいソースと依存コンポーネント |
| node-esp32/partitions_4mb.csv, partitions_8mb.csv(新規) | パーティションテーブル(§4.1) |
| node-esp32/sdkconfig.defaults | パーティションテーブルを独自のものに、ロールバック有効、平文 HTTP の OTA を許可、SNTP の間隔 15 秒 |
| node-esp32/sdkconfig.defaults.esp32c5(新規) | C5 は 8MB フラッシュと partitions_8mb.csv |
| node-esp32/build.ps1(新規) | OTA 対応版のビルド(ターゲットごとに build-ota-esp32/・build-ota-esp32c5/) |
| node-esp32/host_test/timesync_test.c, actor_slot_test.c(新規) | C の計算部分の PC 上の検査 |
| node-esp32/host_test/fake_node.py(新規) | ノードのふりをするプログラム(OTA ツールの通しの確認用) |
| scripts/dyne_ntp_server.py(新規) | PC の NTP サーバ(標準ライブラリのみ)と問い合わせ側 |
| scripts/dyne_timesync.py(新規) | timesync 命令の送信と、tsync・health の CSV 記録と要約 |
| scripts/pps_analyze.py(新規) | PulseView の記録(VCD/CSV)からノード間のずれを求める |
| scripts/dyne_ota.py(新規) | 一斉 OTA ツール |
| scripts/check-timesync-ota.ps1(新規) | 実機なしでできる確認の一括実行 |
| .gitignore | node-esp32/build-ota-*/ を追加 |

Rust は変更していない。既存の build/・build-c5/ は従来どおり「アプリ1個」のパーティションテーブルのままビルドできる(§11.1)。

---

## 3. 時刻同期(G1)

### 3.1 構成

```
PC: scripts/dyne_ntp_server.py (UDP 123 番、または別のポート)
      ▲ SNTP (ESP-IDF 標準、15秒ごと)      → ノードのシステムの時計 (gettimeofday)
      ▲ 往復の交換 (2秒ごとに4往復)        → ノードの「同期した時刻」= esp_timer + 推定したずれ
ノード → dyne/node/<id>/tsync (推定の質)、health (要約)
```

**SNTP だけでは足りない理由**(ESP-IDF 5.5.5 のソースで確認): ESP-IDF の lwIP の SNTP は
`SNTP_COMP_ROUNDTRIP = 0`(往復時間の補正なし)で、サーバの送信時刻をそのまま自分の時刻にする。
このため、片道の遅延(Wi-Fi で数 ms、ばらつきあり)がそのままずれになる。また応答の検査も最小限
(`SNTP_CHECK_RESPONSE = 0`)。そこで、同じ NTP のパケットで独自に往復の交換を行い、往復時間を測って補正する。
SNTP は「システムの時計を大まかに合わせる」役に残し、両者の差(sntp_minus_sync_us)も報告する
(SNTP だけの精度を数値で示せる)。

### 3.2 推定の方法(dyne_timesync_core.c)

1. 2秒ごと(設定可)に 30ms 間隔で4往復し、往復時間が最小の標本を1つ採る(遅延の揺らぎの小さい標本ほど正確)。
2. 窓(直近の採用標本)の最小の往復時間の 2倍 + 2ms を超える標本は捨てる。4回続けて捨てたら受け入れる
   (経路が恒常的に遅くなった場合への追従)。
3. 直近32標本(2秒ごとなら約64秒)に最小二乗の直線を当てはめ、ずれとドリフトを推定する
   (5秒以上の幅と3標本がそろうまでは、最新の標本のずれをそのまま使う)。ドリフトは ±500ppm で打ち切る。
4. 推定との差が 50ms を超える標本が来たら、PC 側の時計の基準が変わった(サーバの再起動など)とみなして推定をやり直す。
5. 報告の resid_us(新しい標本と、加える前の推定との差)が同期の質の目安になる。

PC での模擬(host_test/timesync_test.c、ドリフト 25ppm / −40ppm を与えた):

| 条件 | 推定の最大誤差 | 二乗平均 | ドリフトの推定誤差 |
|---|---|---|---|
| 片道 1ms + 0〜200µs の揺らぎ | 14 µs | 5 µs | 0.06 ppm |
| 片道 1.5ms + 0〜6ms の長い裾 | 481 µs | 181 µs | 3.9 ppm |
| 途中で PC の時計が 1 秒跳ぶ | 回復後 16 µs | — | — |

これは計算部分の正しさの確認であり、**実機の精度の予想ではない**(実際の Wi-Fi の遅延の非対称さ、
PC 側の Python の受信の遅れ、ノード側のタスクの起床の遅れは模擬していない)。実機の値は実験1で測る。

### 3.3 PC の時刻サーバ(scripts/dyne_ntp_server.py)

- 標準ライブラリのみ。`--bind`・`--port` で待ち受けを指定、`--log` で要求ごとの記録。
- 基準の時計は「起動時の PC の時刻 + 高分解能カウンタの経過」。起動後に Windows が時計を合わせ直しても基準は跳ばない
  (全ノードを同じ基準にそろえるのが目的で、PC の時計の絶対的な正しさは問わない)。`--wall` で PC の時計そのもの。
- **この PC では UDP 123 番が使用中**(netstat で確認。Windows の時刻サービス w32time と思われる)。対処は2通り:
  (a) 管理者の PowerShell で `net stop w32time` してから 123 番で起動する(SNTP と往復の交換の両方がこのサーバを使う。推奨)、
  (b) 別のポート(例 12300)で起動し、ノードへ `port` を設定注入する(SNTP は w32time に問い合わせることになり、
  基準が PC の時計そのものになる。往復の交換はこのサーバを使う)。SNTP のポートは ESP-IDF で 123 固定。
- Windows のファイアウォールで、受信の UDP(123 または選んだポート)と TCP(OTA の HTTP、既定 8070)を許可する必要がある。

### 3.4 時刻印

| 経路 | 時刻印 |
|---|---|
| センサ(dyne/node/<id>/sensor) | 従来の t_ms(起動からの ms)に加えて **ts_us**(読み取りの直前の時点の同期した時刻) |
| RSSI(dyne/node/<id>/rssi) | 同上(ts_us) |
| 観測点ナッジングのイノベーション(innov) | **y_ts_us**(使った観測値の取得時刻。精度は y_t_ms と同じ ms) |
| ワイヤ形式v2 の経路(node_v2.c) | 各メッセージのヘッダの時刻印の欄(t_us、オフセット16)が 0 なら、送信メールボックスを読み出した時点(= 境界の値ができた時点)の同期した時刻を入れる。アクターが自分で入れた値は変えない。受け手のアクターは t_us を計算に使わないので数値の結果は変わらない |
| 1次元の従来の経路(40バイトの実機形式) | **時刻印の欄が無いので付けない**(形式を変えると既存の回帰と実験1の構成が崩れるため)。この経路の時刻は MQTT の観測(field など)の受信時刻でしか分からない |
| 測距フレーム(RANGE_FRAME、G3 で今後) | 測距が終わった時点の esp_timer を `dyne_time_at_local()` で同期した時刻にして、ヘッダの t_us に入れる(関数は用意済み) |

未同期のときは 0。

### 3.5 試験モード(整数秒でピンを切り替える)

- ctl の timesync 命令で開始・停止(`pps`、`pps_gpio`、`pps_src`)。偶数秒で高、奇数秒で低にするので、
  全ノードの位相がそろい、切り替わりの向きの食い違いで「1秒ずれたノード」も見分けられる。
- `pps_src`: **sync**(同期した時刻)/ **sntp**(SNTP で合わせたシステムの時計)/ **local**(同期なし、各ノードの起動からの時計)。
  実験1の「同期なし → 同期あり」に加え、SNTP だけの場合も同じ道具で測れる。
- 実装: 最高の優先度のタスクを、切り替えの 2ms 前に esp_timer で起こし、残りは空回りで待ち、最後の 30µs は
  割り込みを止めて待ってから GPIO を書く。実際に書いた時刻と目標の差(late_max_us、late_mean_us)と、
  起こされるのが遅れて飛ばした回数(skip)を tsync で報告する。2コアの ESP32 では Wi-Fi と別のコア(1)に置く。
  1コアの C5 では毎秒 約2ms CPU を占有する(試験モード中のみ)。

---

## 4. OTA(G2)

### 4.1 パーティションテーブル

| パーティション | 4MB(ESPr、partitions_4mb.csv) | 8MB(XIAO ESP32C5、partitions_8mb.csv) |
|---|---|---|
| nvs | 0x9000, 16KB | 同左 |
| otadata(OTA の起動先の記録) | 0xD000, 8KB | 同左 |
| phy_init | 0xF000, 4KB | 同左 |
| ota_0 | 0x10000, 1,664KB(0x1A0000) | 0x10000, 3MB |
| ota_1 | 0x1B0000, 1,664KB | 0x310000, 3MB |
| actors(計算部品、2枠) | 0x350000, 704KB(1枠 352KB) | 0x610000, 1,984KB(1枠 992KB) |

アプリの大きさとパーティションの空き: §11.1。

**XIAO ESP32C5 のフラッシュは 8MB と想定した**(Seeed の製品説明の記憶による。届いたら `idf.py -p COMx flash_id`
の "Detected flash size" で確かめる)。4MB だった場合は、menuconfig でフラッシュの大きさを 4MB、
パーティションテーブルを partitions_4mb.csv にするか、sdkconfig.defaults.esp32c5 の2行を消してビルドし直す。

### 4.2 ファームの OTA

ctl 命令(dyne/ctl/<id>。retain の号令は従来どおり無視):
```
{"cmd":"ota_fw","job":"J","url":"http://PC:8070/x.bin","size":N,"img_sha256":"64桁の16進","reboot":true}
```
流れ: 計算中なら busy で断る → ESP-IDF の esp_https_ota で次のパーティションへ書く(像の検査和・付属の SHA-256・チップの種類は
ESP-IDF が検査)→ 長さと `img_sha256`(.bin の末尾32バイト = ESP-IDF が像に付ける SHA-256)を照合 →
合わなければ起動先を戻して rejected → 合えば NVS にジョブ名と書いたパーティションを記録して再起動。

新しいファームの最初の起動は「確認待ち」(ESP-IDF の PENDING_VERIFY)。自己確認のタスクが、
CONFIG_DYNE_OTA_VALIDATE_TIMEOUT_S(既定 120 秒)以内に **MQTT 接続(= Wi-Fi も上がった)・ワイヤ形式v2 の外箱の
自己検査・計算部品の積み込み・壊れたファームの試験用の目印** がそろえば「有効」にする。そろわなければ
selftest_fail を報告して(MQTT が使えれば)前のファームに戻して再起動する。確認待ちのまま電源が落ちた・
異常終了した場合は、ブートローダが次の起動で前のファームに戻す。

結果は dyne/node/<id>/ota に `{"job","kind":"fw","phase",...}` で出る。phase:
accepted → downloaded(ms・bytes つき)→ 再起動 → **ok**(書いたパーティションで起動して有効になった)/
**rolled_back**(前のパーティションで起動している)。途中の失敗は **rejected**(像や照合の不一致)/ **fail** / **busy**。

### 4.3 計算部品の OTA

```
{"cmd":"ota_actor","job":"J","name":"heat1d_actor_small",
 "wasm":{"url":..,"size":N,"sha256":"64桁"}, "aot":{"url":..,"size":N,"sha256":".."}, "reboot":true}
```
- パーティション actors を2枠に分け、**いま使っていない方の枠**に書く(使っている計算部品を残す)。
  枠の見出し(128バイト、通し番号・長さ・SHA-256・名前・CRC32)は**照合が済んでから最後に**書く。
  途中で電源が落ちても、見出しの無い枠は空として扱われる。
- 切り替えは**再起動で行う**(計算中は busy で断る。計算部品は Wi-Fi より先、断片化していない
  ヒープに積む必要があるため。2026-09-18 の ESPr の実機不具合の対策を崩さない)。
- 起動時は、新しい枠の .aot → .wasm → 古い枠の .aot → .wasm → ファームに埋め込んだ heat1d_actor_small の順に試し、
  各候補は SHA-256 を照合してから積む。積み込み・差込口の確認に失敗したら次へ。
- 同じ新しい計算部品で2回起動に失敗したら(自己確認まで至らない場合)、3回目からはその枠を飛ばす。
- 結果: 再起動後に **ok**(新しい通し番号の計算部品で動いている)/ **fallback**(別のもので動いている)。
- `{"cmd":"actor_rollback"}` は新しい方の枠の見出しを消して再起動する(前の計算部品、無ければ埋め込みに戻る)。
- .aot は WAMR の AOT の読み込み(CONFIG_WAMR_ENABLE_AOT、既存の sdkconfig で有効)があるときだけ試す。
  チップごとに別の .aot が要る(PC 側ツールはノードの target を見て選ぶ)。

### 4.4 PC 側の一斉配布ツール(scripts/dyne_ota.py)

- `status`: ノードの一覧(対象チップ・起動パーティション・版・計算部品・再起動の理由・起動回数)。
- `fw` / `actor` / `actor-rollback`: ファイルを一時ディレクトリに置いて HTTP で配り(既定 8070 番)、
  MQTT で全台(`--nodes all` = online のノード)または指定のノードへ命令を送り、結果を待って表と CSV にする。
  CSV の列: job, kind, variant, node, target, result, expect, match, t_total_s(命令から最終結果まで), t_download_ms
  (ノードが測った書き込みまで), bytes, err, part_before/after, elf_before/after, actor_after, phases(各段階の経過秒)。
- 対象チップが .bin と違うノードは飛ばす(ESP32 用の像を C5 に送らない)。
- **わざと壊したものを配る**(`--broken`):

| 変種 | 作り方 | 期待する結果 |
|---|---|---|
| fw selftest-fail | ファーム中の目印 `DYNE_SELFTEST_MARK:pass` を `FAIL` に書き換え、検査和と SHA-256 を付け直す(像としては正しい) | 書き換えは成功し、起動後の自己確認で失敗 → **rolled_back** |
| fw corrupt | 中身の1バイトを反転し、付け直さない | ESP-IDF の像の検査で拒否 → **rejected**(書き換え前のファームのまま、再起動もしない) |
| actor garbage | 乱数の中身に、正しい SHA-256 の目録 | 書き込みは成功、起動時の積み込みに失敗 → **fallback** |
| actor badsha | 目録の SHA-256 を中身と違うものに | 照合で拒否 → **rejected** |

- `info` / `make-broken` はノード無しで使える(.bin の版・チップ・SHA-256・目印の確認、壊した像の作成)。
- 必要なもの: paho-mqtt(既存の scripts/mqtt_config.py と同じ。グローバルの Python に入っている。.venv には無い)。

### 4.5 安全性の限界

閉じた LAN の中での使用に限る。**署名・暗号化・認証は無い**: HTTP は平文、MQTT の ctl を送れる者は誰でも
任意のファームを書き込める(ESP-IDF の Secure Boot・フラッシュ暗号化・HTTPS は使っていない)。
SHA-256 の照合は「壊れていない・取り違えていない」ことの確認であり、「正当な発行元である」ことの確認ではない。
発表後に外のネットワークへ出す場合は、HTTPS + サーバ証明書、像の署名(Secure Boot v2)、MQTT の認証が必要。

---

## 5. 再起動の理由と稼働時間(G11)

- 起動時(online の status、retained): `reset`(poweron / sw / panic / task_wdt / int_wdt / **brownout** ほか)、
  `boot`(起動回数、NVS)、`prev_up_s`(**前回の稼働時間**。60 秒ごとに NVS に書き、次の起動で報告)、
  `target`・`ver`・`elf`(ELF の SHA-256 の先頭16桁)・`part`(起動パーティション)・`ota_state`・`actor`。
- 定期(dyne/node/<id>/health、10 秒ごと): `up_s`・`reset`・`heap`・`heap_min`・`largest`(最大連続ブロック)・
  `ap_rssi`・`busy`・`ts_us`・時刻同期の要約。
- 全 status に `up_s` を追加。
- 乾電池の確認(実験0④)への注意: 電池が尽きて電源が落ちると、次の起動の理由は brownout ではなく **poweron**
  になることが多い(電圧低下の検出が働く前に止まる場合)。そのため「前回どれだけ動いたか」を `prev_up_s` で見る
  (誤差は最大 60 秒)。電圧が下がって再起動を繰り返す場合は brownout が出る。

---

## 6. 小さな実装判断(事後に明記)

1. **同期した時刻は、システムの時計ではなく esp_timer + 推定したずれ**にした。SNTP はシステムの時計を
   ステップで書き換える(往復時間の補正もない)ため、時刻印と試験モードには使わない。SNTP は残して差を報告する。
2. 往復の交換は**NTP と同じパケット**で行い、PC 側は1つのサーバで SNTP と兼用にした(独自の形式を作らない)。
3. ノードの時刻同期の相手の既定は**ブローカーと同じ PC**(menuconfig の DYNE_TIME_SERVER で上書き可)。設定を増やさないため。
4. 時刻同期・試験モード・OTA の命令は **ctl の命令**にした(dyne/cfg は計算の設定で、構文検査で弾かれるため)。
   ctl は retain 禁止なので、再起動すると試験モードは止まる(時刻同期そのものは起動時に自動で始まる)。
5. 計算部品の切り替えは**再起動で行う**(実行中の差し替えはしない)。ヒープの断片化の対策(Wi-Fi より先に積む)を崩さないため。
6. 計算部品の目録(長さ・SHA-256)は MQTT の命令の中に入れた(ノードが別に目録ファイルを取りに行かない)。
7. 計算部品を積めなくても Wi-Fi・MQTT は上げるようにした(**従来は止まっていた**)。OTA で直せるようにするため。
   この場合 start は "no actor loaded" で断る。
8. パーティションテーブルの変更後に NVS の中身が合わない場合は**消して作り直す**(NVS に置いているのは Wi-Fi の較正値と本機能の記録のみ)。
9. ファームの OTA は **ESP-IDF の esp_https_ota を平文 HTTP で**使う(CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP)。
10. 壊れたファームの試験は、**別のファームをビルドせず**、PC 側で目印を書き換えて SHA-256 を付け直す方法にした。
    esptool の image_info で、書き換えた像が正しい像として通ること・反転した像が通らないことを確認した。
11. 既存の build/・build-c5/ は残し、OTA 対応版は**新しいビルドディレクトリ**(build-ota-esp32/・build-ota-esp32c5/)
    にした。sdkconfig.defaults は OTA 版に変えたが、既存の sdkconfig は値を保つので従来のビルドは変わらない。
12. PC 側ツールの MQTT は既存の scripts と同じ paho-mqtt を使った(標準ライブラリだけで MQTT を書き直さない)。
    NTP サーバと記録の解析は標準ライブラリのみ。

---

## 7. 実機での手順

### 7.1 最初の1回(USB で書き込む。1台ずつ)

1. アクセスポイントの SSID・パスワードと、PC の有線 LAN 側のアドレス(例 192.168.10.2)を決める。
2. ビルドの設定(ターゲットごとに1回):
   ```
   powershell -ExecutionPolicy Bypass -File node-esp32\build.ps1 -Target esp32c5 -Menuconfig
   ```
   menuconfig の「Dyne Node」で Wi-Fi SSID・パスワード・MQTT broker URI(`mqtt://192.168.10.2:1883`)を設定する。
   ノード ID は -1(MAC の末尾)のままでよい(全台同一のファームのため)。ESPr は `-Target esp32`。
3. 書き込み(パーティションテーブル・ブートローダ・otadata の初期値を含めて全体を書く。**パーティションテーブルが変わるので、
   以前のファームの入った基板も一度は USB で書き直す必要がある**):
   ```
   powershell -ExecutionPolicy Bypass -File node-esp32\build.ps1 -Target esp32c5 -Flash COM5
   ```
   モニタに `[dyne] boot #1 reset=... part=ota_0 ota_state=...`、`actor in use: embedded`、
   `wire v2 envelope selftest: PASS` が出ることを確かめる。
4. 16台ぶん 3 を繰り返す。以後は USB 不要。

### 7.2 PC 側の常駐

```
net stop w32time                       (管理者の PowerShell。123 番を空ける。§3.3)
python scripts\dyne_ntp_server.py --port 123 --log ntp_log.csv
(mosquitto は従来どおり)
python scripts\dyne_ota.py status --broker 192.168.10.2
```
ファイアウォールで UDP 123 と TCP 8070(OTA)を許可しておく。

### 7.3 以後の書き換え(OTA)

```
python scripts\dyne_ota.py fw --broker 192.168.10.2 --bin node-esp32\build-ota-esp32c5\dyne_node.bin --nodes all --csv ota.csv
python scripts\dyne_ota.py actor --broker 192.168.10.2 --wasm <x.wasm> --aot esp32c5=<x.aot> --name <名前> --nodes all --csv ota.csv
```

---

## 8. 実験1(時刻同期のずれの実測)の手順

配線: 8台の同じピン(例 XIAO の D 番号のピン1本。**GPIO 番号は Seeed のピン配置図で確認**。ESPr なら GPIO 25 など
空いているピン)をロジックアナライザの ch0〜7 へ、**GND を全台とロジックアナライザで共通**にする。
乾電池で動かす場合も GND の共通は必須。

1. ノード8台を起動し、`dyne_ota.py status` で8台が online であることを確かめる。ノード ID と ch の対応を控える。
2. 同期の状態を確かめる(10 秒ほどで synced=true になる):
   ```
   python scripts\dyne_timesync.py --broker 192.168.10.2 --report-every 1 --record 30 --csv tsync_check.csv
   ```
3. **同期なし**: `--pps on --pps-gpio <N> --pps-src local` を送り、PulseView で10分記録(標本化 1MHz 以上)。
   ファイル → Export → **Value Change Dump (VCD)** で書き出す(CSV より桁違いに小さい)。
4. **SNTP のみ**: `--pps-src sntp` で同じく10分。
5. **同期あり**: `--pps-src sync` で同じく10分。同時に `dyne_timesync.py --record 600 --csv tsync_sync.csv` で
   ノードの自己報告(往復時間・推定との差・ドリフト・切り替えの遅れ)も記録する。
6. 同期ありで、アクセスポイントからの距離を変えて 5 を繰り返す。
7. 解析:
   ```
   python scripts\pps_analyze.py sync.vcd --names D0=<id>,D1=<id>,... --out skew_sync.csv
   python scripts\pps_analyze.py local.vcd --mode phase --names ...      (同期なし: 位相のずれとドリフトの差)
   ```
   sync の出力の「ノード間の最大差(広がり)」の中央値・95%点・最大が、実験1の合格の判断に使う数値
   (目標 1ms 程度、two-week-plan §4)。「間隔の誤差 ppm」は、ロジックアナライザの時計を基準にした
   各ノードの1秒の長さの誤差。
8. 終わったら `--pps off`。

注意: ノード間の差には、時刻同期の誤差のほかに、試験モードの切り替えの遅れ(tsync の pps.late_max_us)が含まれる。

## 9. 実験2(OTA の一斉配布)の手順

1. 全台に OTA 対応のファームを USB で書き込む(§7.1)。
2. 計算部品の一斉配布: `dyne_ota.py actor --wasm ... --nodes all --csv exp2.csv`。表の result が全台 ok であること。
3. ファームの一斉配布: `dyne_ota.py fw --bin ... --nodes all --csv exp2.csv`。全台 ok で、part が入れ替わること。
4. 壊れたもの: `fw --broken selftest-fail`(全台 rolled_back)、`fw --broken corrupt`(全台 rejected)、
   `actor --broken garbage`(全台 fallback)、`actor --broken badsha`(全台 rejected)。各回の後に `status` で
   全台 online・計算可能であることを確かめる(合格の目安「壊れたものを配っても動作が止まらない」)。
5. 記録: exp2.csv(所要時間 t_total_s・t_download_ms、成功・戻った台数)。16台同時の HTTP の取得が遅い・
   失敗する場合は `--stagger-ms 500` で間隔を空ける。

---

## 10. 物理シミュレーション環境での扱い(方針のみ。作っていない)

P6 により、同じ仕組みを将来の物理シミュレーション環境(G9)とシミュレータにも入れる。モデル化の方針:

- **時計**: ノードごとに局所時計 `local(t) = (1 + d_i)·t + o_i`(d_i = ドリフト、o_i = 初期のずれ。
  シード固定の乱数で与え、必要なら温度による変化も加える)を仮想時刻 t の上に持たせる。
  時刻同期は、仮想ネット(dyne-core の net: 損失・遅延・分断)上で4つの時刻印を交換するイベントとして扱い、
  **ファームと同じ dyne_timesync_core の式**(往復の選別・直線の当てはめ)を通す
  (C のコードを共有するか、Rust に移して照合用の数値で一致を確かめる)。片道の遅延を非対称にすると、
  実機で見える「往復の式の限界」も再現できる。時刻印・試験モードの切り替えは局所時計+推定で決まる。
- **OTA**: 配布を「サイズ / 帯域 + 遅延」の転送イベントとして仮想時刻に置き、書き込み時間・再起動の時間
  (実験2で測った値)を加える。新しい計算部品は再起動の後のステップから使う(ファームと同じ)。
  自己確認の失敗・積み込みの失敗を注入して、元に戻る経路を決定論的に再現する。
- 決定論(同じ入力なら同じ出力)を保つため、時計・OTA の乱れはすべてシード固定の乱数から作る。

---

## 11. 検証の結果

### 11.1 ビルドとサイズ

すべて警告なしでビルド成功(ESP-IDF 5.5.5、2026-11-11)。`idf.py build` の表示から:

| ビルド | パーティションテーブル | アプリ(.bin) | アプリのパーティション | 空き | ブートローダ |
|---|---|---|---|---|---|
| **OTA 版 ESP32**(build-ota-esp32/) | partitions_4mb.csv | 0x127320 = 1,209,120 B | 0x1A0000 = 1,703,936 B | **29%** | 0x6660(空き 9%、ロールバック込み) |
| **OTA 版 ESP32-C5**(build-ota-esp32c5/) | partitions_8mb.csv | 0x161C90 = 1,449,104 B | 0x300000 = 3,145,728 B | **54%** | 0x56F0(空き 9%) |
| 従来の build/(ESP32) | アプリ1個 | 0x127180 = 1,208,704 B | 0x177000 | 21% | 0x6610 |
| 従来の build-c5/(C5) | アプリ1個 | 0x161AD0 = 1,448,656 B | 0x177000 | **6%** | 0x5660 |

- 2026-11-08 の値(ESP32 1,140,208 B、C5 1,366,528 B)からの増分は ESP32 +69KB、C5 +83KB
  (esp_https_ota・HTTP クライアント・SNTP・本機能)。ESP32 の静的 RAM(DRAM)は 63,744 → 65,128 B(+1.4KB)。
- C5 を 4MB のパーティションテーブルで使う場合の空きは 1,449,104 / 1,703,936 = 15%(計算値。ビルドはしていない)。
- 従来の build-c5/(アプリ1個、1.5MB)は空きが 6% まで減った。C5 は OTA 版(8MB)へ移ることを勧める。
- 既存の sdkconfig(build/・build-c5/)は値を保つため、従来のビルドは「アプリ1個」のパーティションテーブルのまま。この場合
  OTA の命令は "no ota partition" / "no actors partition" で失敗し、計算部品は従来どおり埋め込みを使う。

### 11.2 PC 上の検査

`scripts\check-timesync-ota.ps1 -E2E` で一括実行できる。結果(2026-11-11):

| 検査 | 結果 |
|---|---|
| wire2_selftest(既存、外箱の C 実装) | PASS |
| timesync_test(NTP の時刻印の変換 20万通りの往復一致、往復の式、模擬での推定精度 §3.2、外れ標本の除去、基準の跳びからの回復、次の整数秒 10万通り) | PASS |
| timesync_test を PC の時刻サーバ(127.0.0.1:12300)に向けて実行(ファームと同じ C の計算部分で実際に UDP の往復) | 応答 20/20、最小往復 413µs(同じ PC の中。Python のサーバの処理時間を含む) |
| dyne_ntp_server.py --query(Python の問い合わせ側) | 応答 10/10、往復 最小 490µs / 中央 691µs |
| actor_slot_test(見出しの符号化、128 バイトのどの1ビットを反転しても受理しない 1024通り、書き込みと選び方の模擬、電源断で見出しが無い枠を飛ばす、起動の失敗が続く計算部品を飛ばす) | PASS |
| pps_analyze.py --selftest(合成した VCD で4チャンネルのずれを ±10µs 以内で復元、CSV の読み込み) | PASS |
| dyne_ota.py info / make-broken(ESP32・C5 の .bin) | 正しい像は検査を通る、selftest-fail の像も通る、corrupt の像は検出される |
| 通しの確認(-E2E、ノードのふり2台) | 6通りすべて期待どおり |

ノードのふりをするプログラム(host_test/fake_node.py。判定をファームと同じ規則で、PC 側ツールのコードを使わずに書いた)
への通しの確認(ESP32 の実際の .bin と heat1d_actor_small.wasm を配布):

| 試験 | 結果 |
|---|---|
| fw 正常(3台、うち1台は計算中のふり、別に C5 のふりの1台) | 2台 ok(1.07 秒)、計算中の1台は busy、C5 は対象外として飛ばした |
| fw selftest-fail | 2台とも rolled_back |
| fw corrupt | 2台とも rejected(像の検査) |
| actor 正常(ESP32 2台 + C5 1台) | 3台 ok |
| actor garbage | 2台とも fallback |
| actor badsha | 2台とも rejected(sha256 mismatch) |

壊した像の独立の確認(esptool の image_info): selftest-fail の像は Checksum valid・Validation Hash valid、
corrupt の像は Checksum invalid・Hash invalid。元の像の Validation Hash はツールの img_sha256 と一致。

### 11.3 回帰

Rust は変更していないので、scripts\check-baseline12.ps1 と scripts\check-sim2d.ps1 は実行していない(指示により省略可)。
ファームの既存の部分(1次元の経路・v2 の経路の計算)は、計算部品の積み込み方(候補を順に試す)と v2 の時刻印の欄の
書き込み以外は変えていない。従来の build/・build-c5/ も警告なしでビルドできることを確認した(§11.1)。

---

## 12. 既知の限界・実機で未確認のこと

- **すべて実機未確認**: Wi-Fi 上の往復時間と同期の精度、試験モードの切り替えの遅れ、SNTP と独自推定の差、
  esp_https_ota の平文 HTTP での動作、ロールバック(確認待ち → 有効、失敗 → 元に戻る)、計算部品のパーティションからの積み込み
  (特に .aot の実行可能メモリの確保)、16台同時の HTTP 取得、NVS の作り直し、再起動の理由の報告。
- 計算部品のパーティションからの .aot の積み込みは、bench-c5 で確認済みの AOT の積み込みと同じ API だが、
  パーティションから malloc した領域へ読む経路は未確認。ESPr(PSRAM なし)では大きな .aot は内部 RAM に載らない可能性がある。
- 時刻同期の基準は PC の Python。PC の負荷(他のプログラム)で応答が遅れると往復時間が伸びる
  (最小の往復時間の標本を採る方法で多くは除けるが、PC 側の受信時刻の遅れの非対称は除けない)。
- 試験モードは 1コアの C5 で毎秒約2ms CPU を占有する。計算の実験と同時には使わない。
- NTP の時刻印の2036年問題(32ビットの秒の桁あふれ)は扱っていない。
- 計算部品の OTA は再起動を伴う(Wi-Fi・MQTT の再接続に数秒)。ファームの OTA の書き込み中も MQTT は動くが、
  計算中は断る。
- 1次元の従来の経路(40バイト)には時刻印が無い(§3.4)。
- 署名・暗号化・認証なし(§4.5)。
- PC 側ツールの MQTT は paho-mqtt(グローバルの Python。.venv には入っていない)。

## 13. ユーザーが決めること

1. **UDP 123 番の扱い**: w32time を止めて本サーバを 123 番で使う(推奨: SNTP と独自の交換の基準が一致する)か、
   別のポートで併用するか(§3.3)。
2. **実験1の合格の目標値**: 模擬では揺らぎの小さい条件で数十 µs、大きい条件で数百 µs。実機の結果を見て決める
   (計画では 1ms 程度を想定)。
3. **試験モードのピン**: XIAO ESP32C5 のどのピンを使うか(ピン配置図で確認。I2C・測距と重ならないもの)。
4. **XIAO ESP32C5 のフラッシュの大きさ**の確認(8MB と想定。違えば §4.1 の手順)。
5. **計算部品の切り替えを再起動で行う方式でよいか**(実行中の差し替えは、ヒープの断片化の危険があるため見送った)。
6. **PC 側ツールの MQTT を paho-mqtt のままにするか**(.venv に入れるか、標準ライブラリで書き直すか)。
7. 発表後の安全性(HTTPS・署名・認証)をいつ入れるか(§4.5)。

## 改訂履歴

- 2026-11-11 初版(Claude Code)。

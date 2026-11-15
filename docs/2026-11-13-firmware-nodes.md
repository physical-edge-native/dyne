# 実機のノードのファーム: DevKitC-VE 対応・測距・サーボの首振り・IMU (docs\2026-11-13-firmware-nodes.md)

作成: 2026-11-13(Claude Code)。対象: docs/2026-11-11-two-week-plan.md の P11(測距は7台をサーボで首振り)、
§5 の G3(測距ドライバのノードへの組み込み)・G10(1回の配送が UDP の1データグラムを超える)・
G12(IMU のドライバのノードへの組み込み)、§4 の実験0(机の上での立ち上げ)・実験9(IMU)・実験10(波動方程式)。

**実機では一度も動かしていない**(機材は未着。ESPr も未接続)。確認できたのは、2種類の基板向けのビルドと、
PC 上の検査(C の計算部分。Rust のワイヤ形式の定義とのバイト一致を含む)まで。§10 に未確認の点を列挙する。

用語(本書での意味):
- **DevKitC-VE**: Espressif の開発基板 ESP32-DevKitC-VE(部品 ESP32-WROVER-E。PSRAM 8MB)。今回 15 台買う基板。
- **ESPr**: 手持ちの ESPr Developer 32(部品 ESP32-WROOM-32E。フラッシュ 4MB、PSRAM なし)。
- **PSRAM**: 基板に外付けされた RAM。ESP32 が直接使えるのは 8MB のうち 4MB。内部の RAM より遅い。
- **測距**: VL53L5CX(8×8 区画の距離センサ)で距離を測ること。1回ぶんの 64 区画の値を **フレーム** と呼ぶ。
- **首振り**: 測距センサを小型のサーボ(SG90)に載せて向きを変えること。docs/2026-11-11-sensor-coverage-study.md の
  構成 PAN4H(内向き 180 度を、4方向 × 半区画 2.8125 度ずらし = 8 位置)。
- **IMU**: 加速度と角速度のセンサ(Arduino Modulino Movement = LSM6DSOX)。
- **FIFO**: センサの中のため置き。IMU が測った値を順にためておき、まとめて読み出す。
- **計算部品**: WASM アクター(.wasm)。OTA(無線経由の書き換え、docs/2026-11-11-timesync-ota.md)で配る。
- **ワイヤ形式v2 の経路**: node-esp32/main/node_v2.c。メールボックス方式の計算部品(2次元の場、形、IMU の融合)を
  隣と同期させながら動かす経路(docs/2026-11-08-firmware-wire-v2.md)。
- **ctl 命令**: PC が MQTT の dyne/ctl/<id>(または dyne/ctl/all)へ送る JSON の命令。

---

## 1. 結論の要約

| 項目 | できたこと | 確認の方法 |
|---|---|---|
| DevKitC-VE 向けの設定 | 基板の切り替え(build.ps1 -Board)。PSRAM を有効にし、計算部品と WAMR の内部の表・線形メモリは「内部 RAM を先に、足りなければ PSRAM」で確保。Wi-Fi・lwIP のバッファは PSRAM へ。8MB のパーティションテーブル(アプリ2面 + 計算部品のパーティション) | ビルド成功(§8) |
| ESPr 向け | 従来どおり(PSRAM なし、4MB)。同じピン割り当て | ビルド成功(§8) |
| G3 測距 | rjrp44/vl53l5cx 4.0.1(ST の ULD 2.0.0 の移植。BSD-3-Clause の条件で使う)をノードへ。起動時に探し、居れば 8×8・15Hz で測る。フレームを MQTT へ発行し、ワイヤ形式v2 の経路では RANGE_FRAME としてアクターへ渡す | RANGE_FRAME の組み立てを Rust の定義(dyne-wire)が作るバイト列と照合(一致) |
| 首振り(P11) | LEDC(パルス幅変調の周辺回路)で SG90 を動かす。位置の順・常に角度の小さい側から近づける・動いている間に積分したフレームを捨てる・落ち着いたら PWM を止める・位置ごとに別のセンサID・較正表 | 手順の計算部分を PC で模擬(3巡、順序・捨て方・近づけ方を検査) |
| G12 IMU | LSM6DSOX を FIFO + センサの時刻印で読む。センサの時刻印 → esp_timer → 同期した時刻 の対応づけ。生の標本を MQTT へ(実験9の記録)。IMU の融合のアクター(crates/imu-fusion-actor)の dyne_imu_push へ渡す | FIFO の語の解釈と時刻の当てはめを PC で模擬(発振の誤差 1.2%・読み出しの揺らぎ ±150µs で誤差 50µs 程度) |
| G10 分割送信 | **作らなかった**(§6)。実機用の縮小版のアクターは構造上 1 データグラムに収まる(シミュレータで最大 1,168 バイト < 上限 1,436 バイト) | dyne-sim2d の max delivery unit |
| 2次元の計算部品 | wave2d 縮小版・sdf2d 縮小版・imu-fusion-actor は、ワイヤ形式v2 の経路が必要とする差込口を全て持つ | wasm の差込口の一覧(§7) |

---

## 2. 変更・追加したファイル

| ファイル | 内容 |
|---|---|
| node-esp32/main/dyne_sense_core.c/.h(新規) | 計算部分(ESP-IDF に依存しない): RANGE_FRAME の組み立て・行の取り出しと左右の反転、首振りの手順、IMU の FIFO の語の解釈、時刻印の当てはめ、LSM6DSOX の設定値の表 |
| node-esp32/main/dyne_range.c(新規) | 測距(VL53L5CX)とサーボ(LEDC)。ctl 命令 range・pan |
| node-esp32/main/dyne_imu.c(新規) | IMU(LSM6DSOX)。ctl 命令 imu |
| node-esp32/main/dyne_sense.h(新規) | 上の3つの公開の関数 |
| node-esp32/main/node_v2.c | 設定に feed(range / imu)を追加。測距のフレームと IMU の標本をアクターへ渡す。センサの一覧をアクターへ注入。終了時の報告に計数 |
| node-esp32/main/main.c | ctl 命令 range・pan・imu の受け口。測距・IMU の開始。PSRAM のある基板で WAMR の確保を「内部 RAM を先に」 |
| node-esp32/main/dyne_ops.c, dyne_node.h | 計算部品の読み出しの確保を「内部 RAM を先に」(PSRAM なしの基板では従来と同じ) |
| node-esp32/main/sensors.c | I2C のバスを共有する関数 dyne_i2c_bus()(測距・IMU と同じバス) |
| node-esp32/main/Kconfig.projbuild | 基板の種類(ピンの既定値)、サーボのピン、測距・IMU を起動時に探すか、送る行、I2C の速さ。試験モードのピンの既定を ESP32 の基板で 26 に |
| node-esp32/main/CMakeLists.txt, idf_component.yml | 新しいソース、LEDC、VL53L5CX のドライバ(版を固定) |
| node-esp32/sdkconfig.defaults | 1ms 刻みの OS の時計(ドライバの待ちのため)、VL53L5CX の設定(1区画1物体、I2C の待ちの上限 1 秒) |
| node-esp32/sdkconfig.board.espr / sdkconfig.board.devkitc-ve / sdkconfig.flash4mb(新規) | 基板ごとの設定(§3) |
| node-esp32/build.ps1 | -Board・-Flash4MB・-Reconfigure、手元の設定 sdkconfig.local、既定の設定が新しければ sdkconfig を作り直す |
| node-esp32/host_test/sense_test.c(新規) | 計算部分の PC 上の検査 |
| .gitignore | node-esp32/sdkconfig.local(Wi-Fi のパスワードなどを書く手元の設定) |

---

## 3. 基板とピン

### 3.1 ビルドの切り替え

```powershell
powershell -ExecutionPolicy Bypass -File node-esp32\build.ps1 -Board devkitc-ve   # build-ota-devkitc-ve\
powershell -ExecutionPolicy Bypass -File node-esp32\build.ps1 -Board espr         # build-ota-esp32\ (従来と同じ)
powershell -ExecutionPolicy Bypass -File node-esp32\build.ps1 -Board xiao-c5      # build-ota-esp32c5\ (従来と同じ)
```

| 基板 | 重ねる設定 | フラッシュ・パーティションテーブル | PSRAM |
|---|---|---|---|
| devkitc-ve | sdkconfig.defaults → sdkconfig.board.devkitc-ve → (sdkconfig.local) | 8MB、partitions_8mb.csv(アプリ 3MB × 2面、計算部品のパーティション 約1.9MB) | 有効。malloc は 16KB 以上を PSRAM に。Wi-Fi・lwIP のバッファも PSRAM に。チップの改版3以上(WROVER-E は改版3)向けにして PSRAM の古い回避策を外した |
| espr | sdkconfig.defaults → sdkconfig.board.espr → (sdkconfig.local) | 4MB、partitions_4mb.csv | なし |

- **DevKitC-VE のフラッシュが 4MB だった場合**: 秋月の商品名の「8MB」がフラッシュか PSRAM かは確認できていない
  (Espressif の資料では DevKitC-VE は フラッシュ 8MB・PSRAM 8MB)。最初の USB 書き込みで esptool が
  `Detected flash size: 4MB` と出したら、`-Flash4MB` を付けて作り直す(パーティションテーブルが 4MB 用になる。ビルドディレクトリは
  build-ota-devkitc-ve-4mb\)。
- **DevKitC-VE 用のビルドを ESPr に書かない**(PSRAM が見つからず起動時に止まる)。
- Wi-Fi の SSID・パスワード・ブローカーの URI は **node-esp32\sdkconfig.local**(git 管理外)に書く:
  ```
  CONFIG_DYNE_WIFI_SSID="dyne-exp1"
  CONFIG_DYNE_WIFI_PASS="パスワード"
  CONFIG_DYNE_BROKER_URI="mqtt://192.168.137.1:1883"
  ```
  build.ps1 は、既定の設定のファイルがビルドディレクトリの sdkconfig より新しいと sdkconfig を作り直す
  (ESP-IDF は既にある sdkconfig の値を既定の設定で上書きしないため)。menuconfig で変えた値はこのとき消えるので、
  消したくない値は sdkconfig.local に書く。ノードID は全台同じファームにするため既定(-1 = MAC の末尾)のまま。

### 3.2 PSRAM と計算部品

- 計算部品の本体(.wasm)、WAMR の内部の表、線形メモリは **内部 RAM を先に、足りなければ PSRAM** に置く
  (線形メモリは WAMR の ESP-IDF 用の層が元から この順で確保する。本体と内部の表は main.c の確保の関数を WAMR に渡した)。
  内部 RAM の方が速いので、縮小版(線形メモリ 64〜128KB)は内部 RAM に入れば ESPr と同じ速さ、入らなければ
  PSRAM で動く(遅くなる。速さは未計測)。
- 大きな計算部品: ESP32 が使える PSRAM は 4MB なので、線形メモリが 4MB 以下なら積める見込み。
  形のフル版(136 ページ = 8.9MB)は入らない。
- **注意**: 形の縮小版(crates/sdf2d-grid-actor --features small)は、今日 (11:26) のビルドで線形メモリが **2 ページ(128KB)**
  になっていた(11/11 の計測の時点では 1 ページ)。ESPr(連続した空きが約 108KB)では積めない見込み。DevKitC-VE では
  内部 RAM か PSRAM に入る。

### 3.3 ピン(DevKitC-VE と ESPr で同じ)

| 用途 | GPIO | 備考 |
|---|---|---|
| I2C SDA | 21 | 測距・IMU・温湿度センサで共有(番地 0x29 / 0x6A・0x6B / 0x38・0x76) |
| I2C SCL | 22 | 同上。内部のプルアップも有効(各モジュールにもプルアップあり) |
| サーボの信号 | 25 | LEDC 50Hz。ctl の pan.gpio で変えられる(サーボの初期化の前だけ) |
| 試験モードのピン(時刻同期の実験1) | 26 | 同期した時刻の整数秒ごとに切り替え。ctl の timesync.pps_gpio で変えられる |
| 予備 | 27, 32, 33, 4, 13, 14 | 4・13・14 は JTAG と兼用だが使える |

避けたピン: **16・17(WROVER-E では PSRAM が使う)**、6〜11(フラッシュ)、0・2・5・12・15(起動時の状態を決めるピン)、
1・3(USB シリアル)、34〜39(入力専用)。

---

## 4. ノードの種類ごとの配線

部品: DevKitC-VE、VL53L5CX のモジュール(SparkFun Qwiic Mini ToF Imager)、Modulino Movement(LSM6DSOX、Qwiic)、
SG90、電池ボックス 単3×4(マイコン用)・単3×3(サーボ用)、470µF 以上の電解コンデンサ。
Qwiic のケーブルの色: 黒 = GND、赤 = 3.3V、青 = SDA、黄 = SCL。

### 4.1 測距 + サーボのノード(PAN4H の7台)

| 相手 | 線 | DevKitC-VE / ESPr |
|---|---|---|
| VL53L5CX(Qwiic) | 黒 / 赤 / 青 / 黄 | GND / **3V3**(5V につながない) / GPIO21 / GPIO22 |
| SG90 | 茶(GND) | サーボ用電池の − **と** マイコンの GND(GND は共通) |
| SG90 | 赤(電源) | サーボ用電池(単3×3)の + |
| SG90 | 橙(信号) | GPIO25 |
| コンデンサ 470µF〜 | + / − | サーボの赤 / 茶 のすぐそば(極性に注意: 帯の側が −) |
| マイコン用電池(単3×4) | + / − | **5V ピン** / GND |

- センサはサーボの腕(ホーン)に載せる。サーボの軸からセンサの前面までの距離を測って ctl の range.arm_m に入れる。
- ケーブルがサーボの動き(180 度)で引っ張られないよう、余長を持たせる。
- 高さ 25〜30cm の台に置く(P12、7台とも同じ高さ)。

### 4.2 IMU のノード(実験9の8台)

| 相手 | 線 | DevKitC-VE / ESPr |
|---|---|---|
| Modulino Movement(Qwiic) | 黒 / 赤 / 青 / 黄 | GND / 3V3 / GPIO21 / GPIO22 |
| マイコン用電池(単3×4) | + / − | 5V ピン / GND |

IMU の I2C 番地は 0x6A と 0x6B の両方を探す(Arduino の資料の既定は 0x6A と理解しているが未確認。どちらでも動く)。
板に固定するときは、全台の IMU の向き(x・y・z の矢印)をそろえる。

### 4.3 素のノード(波動方程式・熱など計算だけ)

電池(単3×4 → 5V ピン / GND)だけ。センサは不要(起動時に探して居なければ何もしない)。

### 4.4 電源の注意

- **USB と電池を同時につながない**(書き込み・ログを見るときは電池のスイッチを切る)。
  DevKitC-VE は回路図上 USB 側とダイオードで分かれているが(docs/2026-11-11-order-list.md)、取り違えの事故を避けるため。
- 単3×4(新品 約6.4V)を 5V ピンへ。単3×3 では 3.3V の電源回路(AMS1117)の電圧差が足りなくなる(同上の文書)。
- サーボは**別の電池**(単3×3)から。マイコンの 3.3V・5V から取らない。GND だけは共通にする(信号の基準のため)。
- サーボの近くに 470µF 以上のコンデンサ(動き出しの電流の山で電圧が落ちるのを和らげる)。
- ファームは位置に着いて落ち着いたら PWM を止める(既定)。止めるとサーボは保持の電流を流さない。

---

## 5. 設定

### 5.1 menuconfig(Dyne Node。基板の選択で既定値が決まる)

| キー | 既定 | 意味 |
|---|---|---|
| DYNE_BOARD | ESP32 なら ESPr | 基板の種類(ピンの既定値)。build.ps1 -Board が決める |
| DYNE_SERVO_GPIO | 25(ESP32 の基板) | サーボの信号。-1 = 無し |
| DYNE_PPS_GPIO | 26(ESP32 の基板) | 試験モードのピン(従来は -1) |
| DYNE_RANGE_AUTO | y | 起動時に VL53L5CX を探して、居れば測る |
| DYNE_RANGE_ROW | 3 | アクターへ送る行(0〜7)。-1 = 全区画 |
| DYNE_RANGE_I2C_HZ | 400000 | VL53L5CX の I2C の速さ(評価ファームと同じ) |
| DYNE_IMU_AUTO | y | 起動時に LSM6DSOX を探して、居れば測る |

### 5.2 ctl 命令(dyne/ctl/<id>。省いた欄は今の値のまま。応答は dyne/node/<id>/sense/ack)

**range**(測距):

| 欄 | 既定 | 意味 |
|---|---|---|
| en | true | 測るか |
| probe | false | センサを探し直す(後から挿した場合) |
| res | 8 | 4(4×4)/ 8(8×8) |
| hz | 15 | 測る周期 [Hz](8×8: 1〜15、4×4: 1〜60) |
| mode, it_ms, sharp, order | センサの既定 | "auto"/"cont"、積分時間 [ms]、シャープナー [%]、"closest"/"strongest" |
| row | 3 | 送る行(0〜辺−1)。-1 = 全区画 |
| flip | true | 各行の左右を反転(下記) |
| pub_every | 1 | dyne/node/<id>/range に発行するフレームの間隔(0 = 発行しない) |
| sid | 0 | 首振りなしのときのセンサID(0〜63) |
| x, y, yaw_deg | 0 | センサの位置 [m]・向き [度](部屋の座標、x 軸から反時計回り)。首振りのときはサーボの軸の位置と、サーボの中央の向き。**実験4で測った値を PC が注入する**(測距の値から推定しない) |
| arm_m | 0 | サーボの軸からセンサまでの距離 [m] |

- **flip について**: アクター(と dyne-wire の zone_angle_offset)は「区画0 = センサから見て左端」。ST のドライバの並びは
  左右が逆だという説明(SparkFun の例が左右を反転して表示している)に合わせて既定で反転した。**実機で未確認**(§10。確かめ方は §9.1 の⑤)。

**pan**(首振り):

| 欄 | 既定 | 意味 |
|---|---|---|
| en | false | 首振りを始める |
| gpio | 25 | サーボの信号のピン |
| deg | [22.5, 25.3125, 67.5, 70.3125, 112.5, 115.3125, 157.5, 160.3125] | 各位置のサーボへの指令の角度 [度]。この順に回る(シミュレータの場面 room6_move_pan4h.txt と同じ順) |
| true_deg | 無し | **較正表**: 各位置の実際の向き − 取り付けの向き [度](反時計回りが正)。無ければ dir·(deg − center_deg) |
| center_deg, dir | 90, 1 | 較正表が無いときの換算。サーボの角度が増えると反時計回りなら dir = 1、時計回りなら −1 |
| id_base | 0 | 位置 k のセンサID = id_base + k(PAN4H では台番号 × 8。id_base + 位置の数 ≤ 64) |
| backlash_deg | 4 | 目標より小さい角度から近づける(ガタの向きをそろえる)ために、いったん寄せる手前の角度 |
| ms_per_deg, settle_ms | 2.5, 150 | 移動の時間の見込み [ms/度] と、その後の落ち着きの時間 [ms] |
| frames, discard | 3, 0 | 1位置で採るフレームの数、落ち着いた後さらに捨てる数 |
| dwell_max_ms | 1500 | フレームが来なくても次の位置へ進むまでの時間(センサなしでサーボだけ確かめるとき) |
| hold | false | 落ち着いた後も PWM を出し続けるか |
| us_min, us_max, deg_max | 500, 2400, 180 | 0 度と deg_max 度のパルス幅 [µs] |
| goto | 無し | 首振りを止めてこの角度に向けたままにする(取り付けの向き・較正表を測るとき) |

フレームの採り方: データが揃った時刻を t として、積分の区間(t − 周期, t] が「動きが終わる見込みの時刻」より前と
重なるフレームは捨てる(動いている間に測った値を使わない)。1位置あたり約 0.4〜0.5 秒、8 位置の一巡は約 4 秒の見込み
(PC の模擬での値。実物の SG90 の速さで変わる)。

**imu**:

| 欄 | 既定 | 意味 |
|---|---|---|
| en | true | 測るか |
| probe | false | 探し直す |
| odr | 104 | 測る周期 [Hz](12.5/26/52/104/208/416/833) |
| fs_g, fs_dps | 4, 500 | 加速度の範囲 [±g](2/4/8/16)、角速度の範囲 [±dps](250/500/1000/2000) |
| batch_ms | 100 | FIFO を読み出す間隔 [ms](odr × batch_ms ≤ 120 標本) |
| pub | true | dyne/node/<id>/imu に生の標本を発行するか |

### 5.3 発行するトピック(dyne/node/<id>/…)

| トピック | 中身 |
|---|---|
| sense | 測距・首振り・IMU の状態(見つかったか、設定、計数、誤り)。10 秒ごとと設定の変更時。測距と首振りは `{"range":…,"pan":…}`、IMU は `{"imu":…}` の別の発行 |
| range | 採ったフレーム: `{"seq","step":-1,"sid","pos","t_us"(同期した時刻),"t_loc"(esp_timer),"count","cols","d_mm":[…],"st":[…]}`(送る区画だけ。反転済み) |
| range_fed | ワイヤ形式v2 の経路でアクターへ渡したフレーム(同じ形、step = 渡したステップ)。PC がシミュレータで同じ入力を再生できる(距離 [m] = d_mm / 1000.0 で、ファームと同じ値になる) |
| imu | 1回の読み出しぶんの標本: `{"seq","odr","fs_g","fs_dps","sa"(g/LSB),"sg"(dps/LSB),"synced","us_per_tick","fit_resid_us","ovr","orphan","s":[[t_us, t_loc, ax, ay, az, gx, gy, gz],…]}`。ax〜gz は生の値 |

### 5.4 ワイヤ形式v2 の経路の設定(dyne/cfg/<id>)への追加

```json
"feed": {"range": true, "imu": false}
```

- range: 初期化で、PC の field2d.cfg(形のアクターの設定)の後に、CFG_ZONE_ROW(行を取り出して送るなら 0)と、
  センサの一覧(CFG_SENSOR_SEL/X/Y/YAW。首振りなら位置ごとに ID = id_base + k、向き = yaw_deg + 較正表)を注入する
  (dyne-sim2d と同じく「PC の設定 → センサ」の順)。各ステップでは、送信の後・受信待ちの前に、最新のフレーム1枚を
  RANGE_FRAME(src = センサID、step = そのステップ、時刻印 = 同期した時刻)として受信メールボックスへ入れる
  (dyne-sim2d と同じ位置)。その前の未渡しのフレームは飛ばして数える(range_skip)。
- imu: dyne_step の直前に、前回からの標本を dyne_imu_push(t_us、m/s²、rad/s)で入れる。アクターが
  dyne_imu_need_until_us で時刻を示したら、その時刻までの標本が届くまで最大 300ms 待つ(超えたら imu_wait_to を数える)。
- 終了時の done の status に計数を足した: range_fed・range_skip・range_rej、imu_pushed・imu_late・imu_early・imu_lost・imu_wait_to。

実機の測距・IMU の値はその場限りの入力なので、実機の結果とシミュレータの結果のビット一致は、range_fed と imu の
記録をシミュレータで再生したときにだけ意味を持つ(再生の道具は未作成。§10)。

---

## 6. G10(1 データグラムを超える配送)について: 分割送信は作らなかった

ワイヤ形式v2 の外箱の上限は 1 データグラム 1,472 バイト(中身 1,436 バイト)。今のファームは超えると誤りで止まる。

実機用の縮小版のアクターで、1回の配送(1つの界面・1ステップぶん)の最大をシミュレータで測った(今日のビルド):

| 計算部品 | 条件 | 最大の配送 |
|---|---|---|
| 形の縮小版 | room6_move.txt、4×4 分割、200 ステップ | 1,040 バイト(ビット一致 PASS) |
| 形の縮小版 | room6_move.txt、5×3 分割 | 840 バイト(PASS) |
| 形の縮小版 | study/pan4.txt(28 台が同時に測る。想定外の負荷)、5×3 分割 | 1,168 バイト(あふれで FAIL。大きさは上限内) |
| 形の縮小版 | study/pan4_4mounts.txt、5×3 分割 | 928 バイト(PASS) |
| 形の縮小版 | **構造上の上限**: 界面サンプル列 40 点 + 中継の光線 24 本(FWD_MAX) | 4+32+320 + 4+32+960 = **1,352 バイト** |
| 波動方程式の縮小版 | 1辺 48 点まで | 約 420 バイト |
| IMU の融合 | 1点 × 8 量 | 100 バイト |

縮小版は送信待ちの光線を面ごとに 24 本に制限しているので、どの場面でも 1,436 バイトを超えない(超える分はアクターが
あふれとして数える)。2週間計画の「最大 14KB」はフル版の値で、フル版は ESP32 のメモリに入らない(§3.2)。
よって**今は分割送信は不要**と判断した(小さな実装判断)。PSRAM を使って縮小版より大きい版を作る場合の選択肢:

- (a) 外箱に「分割の番号 / 分割の数」を足す(dyne-wire の envelope.rs の版を上げる。シミュレータの配送も合わせる)。
- (b) lwIP の IP 層の分割と再組み立て(CONFIG_LWIP_IP4_REASSEMBLY)を有効にし、外箱の上限だけを上げる
  (ファームの変更は小さいが、1片の損失で全体を失う。今の再送の仕組みで回復はする)。
- (c) アクターの送信待ちの上限で抑える(今の方式)。

---

## 7. 2次元の計算部品がワイヤ形式v2 の経路で動くか

ワイヤ形式v2 の経路は、計算部品が dyne_outbox_read・dyne_inbox_write(メールボックス方式)を持てば有効になり、
さらに dyne_set_face・dyne_init2d・dyne_msg_buf・dyne_msg_cap・dyne_step・dyne_len・dyne_get を要する
(dyne_cfg・dyne_dropped・dyne_rejected・dyne_missing は任意)。今日のビルドの wasm の差込口を調べた:

| 計算部品 | 必須の差込口 | 任意 | 線形メモリ | 備考 |
|---|---|---|---|---|
| wave2d 縮小版(crates/wave2d-actor/target-small) | 全てあり | dyne_cfg・dropped・rejected・missing あり | 1 ページ(64KB) | |
| sdf2d 縮小版(crates/sdf2d-grid-actor/target-small) | 全てあり | 同上 + dyne_get_aux など | **2 ページ(128KB)** | §3.2 の注意。feed.range で測距を渡す |
| imu-fusion-actor | 全てあり(別の作業者のクレート、今日のもの) | dyne_imu_push・dyne_imu_need_until_us あり | — | feed.imu で IMU を渡す。dyne_init2d の nx・ny・h は使わないが、ファームの設定の検査のため PC は正の値を入れる |

配り方は従来の計算部品の OTA(scripts/dyne_ota.py、docs/2026-11-11-timesync-ota.md)。ファームに埋め込む計算部品は
従来どおり heat1d の縮小版。

---

## 8. ビルドの結果

ESP-IDF 5.5.5、2026-11-13。どちらも警告 0 件。

| 基板 | アプリの大きさ | アプリのパーティション(1面) | 静的 RAM(DRAM: .data + .bss) | IRAM |
|---|---|---|---|---|
| espr(build-ota-esp32) | 1,340,291 B | 1,703,936 B(79%) | 66,984 B(11/11 の版 65,128 B から **+1.9KB**) | 96,763 B |
| devkitc-ve(build-ota-devkitc-ve) | 1,346,835 B | 3,145,728 B(43%) | 67,196 B | 102,091 B(PSRAM の処理の分 +5KB) |

- 静的 RAM の増分を小さく保つため、測距・IMU の作業領域(ドライバの構造体 約4KB、IMU のため置き 約6KB、発行の組み立て 約5KB、
  時刻の当てはめ 約1KB)は**センサが見つかったときにだけ**ヒープに確保する。センサの無いノードのヒープは
  タスク2つのスタック(測距 8KB、IMU 4KB)のぶんだけ減る。ESPr で計算部品の線形メモリ(64KB の連続)が取れるかは、
  計算部品を Wi-Fi より先に積む従来の順のままなので影響は小さい見込み(**未確認**。起動ログの largest で確かめる)。
- XIAO ESP32C5(-Board xiao-c5)は今回ビルドしていない(LEDC の低速モードなど C5 にもある機能だけを使ったが未確認)。
- PC 上の検査: `node-esp32\host_test\sense_test.c` → PASS(RANGE_FRAME の Rust とのバイト一致、区画の選び方、首振りの3巡の
  模擬、FIFO の語、時刻の当てはめ: 公称 25µs・実際 25.3µs で当てはめた刻み 25.3000µs、標本の時刻の誤差の最大 50µs)。
  手順は sense_test.c の冒頭。

---

## 9. 実験0の手順(各部分を1つずつ)

前提: ブローカー(mosquitto)と PC の時刻サーバ(scripts/dyne_ntp_server.py)を起動、`scripts/mqtt_observe.py` などで
dyne/node/# を見られること。

### 9.0 最初の1回の書き込み(USB)、以後は OTA

1. 電池を外し、USB(マイクロ USB、変換 IC は CP2102N)でつなぐ。ポート名は
   `Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match '\(COM\d+\)' } | Select-Object Name`。
2. `powershell -ExecutionPolicy Bypass -File node-esp32\build.ps1 -Board devkitc-ve -Flash COM5`
   - 書き込みが始まらない(`Connecting....___` が続く)ときは、**BOOT ボタンを押したまま**もう一度実行し、
     `Writing at …` が出たら離す。DevKitC は普通は自動で書き込みの状態に入る。
   - esptool の表示 `Detected flash size` を記録する(4MB なら §3.1)。起動のログで `SPI SRAM memory test OK`・
     `Found 8MB SPI RAM`(または 4MB として使う旨)を確かめる。
3. 以後は OTA(docs/2026-11-11-timesync-ota.md の scripts/dyne_ota.py)。

#### 9.0.1 何台も続けて書く道具(scripts/first_flash.py、2026-11-15 追記。GitHub issue #1)

上の 2. を 15 台ぶん手で繰り返す代わりに使う。同じ像を基板ごとに作り直さずに書き、確かめた結果と
MAC(基板ごとに工場で書き込まれた通信の番地)とノード番号の対応を 1 台 1 行で CSV に残す。
**実機では未確認**(作った時点で基板は未着。ログの読み取りは合成したログで検査した)。

1. 先にビルドだけしておく: `powershell -ExecutionPolicy Bypass -File node-esp32\build.ps1 -Board devkitc-ve`
2. `powershell -ExecutionPolicy Bypass -File scripts\first_flash.ps1`
   (中身は `.venv\Scripts\python.exe scripts\first_flash.py`。ESP-IDF の初期化は要らない。esptool は ESP-IDF の Python 環境のものを自動で使う)
3. 基板を 1 台 USB でつなぐと、新しい COM ポート(変換 IC が CP2102N のもの)を見つけて次を自動で行う:
   - esptool で MAC と実際のフラッシュの大きさを読む。ビルドの想定(8MB)と違えば**書かずに止める**(4MB なら §3.1)。
   - ブートローダ・パーティションテーブル・otadata・アプリを書く(build-ota-devkitc-ve\flasher_args.json のとおり。`-Flash` と同じ内容)。
   - 基板をリセットして起動のログを最長 15 秒読み、確かめる: PSRAM(ESP-IDF 5.5 では `Found 8MB PSRAM device`。
     上の 2. の `Found 8MB SPI RAM` は古い版の文言)と `SPI SRAM memory test OK`、`wire v2 envelope selftest: PASS`、
     `[dyne] heap(after-wasm)` の largest(記録する)、起動の見出しのノード番号、書いた像と動いている像の一致、
     異常終了や再起動の繰り返しが無いこと。
   - 結果(PASS / FAIL と理由)とノード番号を大きく表示する。番号を基板のラベルに書き、基板を外すと次を待つ。終わりは Ctrl+C。
4. 書き込みが始まらないときは BOOT ボタンを押したまま待つ(道具が 3 回までやり直す)。
5. 記録: `first_flash_out\first_flash.csv`(MAC・ノード番号・フラッシュの大きさ・PSRAM・結果・時刻など)と
   `first_flash_out\logs\`(1 台ごとの esptool と起動のログの全文)。**git 管理外**(残すなら measured\ へ写す)。

- ノード番号は MAC の末尾 1 バイト(dyne_node_id.c の dyne_node_id。menuconfig の番号を -1 のままにし、NVS に番号を
  書いていない場合。NVS の番号は §9.0.2)。
  道具は起動のログの番号を正とし、MAC から計算した番号と照らす(NVS の番号で起動した基板は照らさない)。**番号が別の基板と重なることがある**
  (末尾 1 バイトがでたらめなら、15 台で約 34%。同じ時期の基板は MAC が続き番号のことが多く、実際はもっと低いかもしれない)。
  CSV に同じ番号の別の MAC があれば(MAC ごとに最新の行で比べる)FAIL(理由 dup_id)にして知らせ、直し方の命令の例を表示する。
  直し方は §9.0.2(2026-11-15 追記。§11 の 6. の (b))。
- 1 台だけ: `... first_flash.ps1 --port COM5`。書かずに MAC と番号だけ記録し直す: `--skip-flash --port COM5`。
  保存したログを確かめ直す(基板不要): `--parse-log first_flash_out\logs\XXXX.log`。その他のオプションは `--help`。
- 検査: `.venv\Scripts\python.exe -m unittest scripts/tests/test_first_flash.py -v`(合成したログ。scripts\tests\fixtures\first_flash\)。

#### 9.0.2 ノード番号を NVS に置く(2026-11-15 追記。GitHub issue #19)

全台に同じ像を書いたまま、MAC の末尾 1 バイトで番号が重なった基板にだけ別の番号を与えるため、番号を NVS
(フラッシュの設定の保存領域。ESP-IDF の nvs_flash。名前空間 dyne、キー node_id、int32)に置けるようにした(§11 の 6. の (b))。
**実機では未確認**(ビルドと PC 上の検査まで)。

**番号の決まり方**(起動時に1回。node-esp32/main/dyne_node_id_core.c の dyne_node_id_select):

| 順 | 条件 | 番号 | status の id_src |
|---|---|---|---|
| 1 | menuconfig の CONFIG_DYNE_NODE_ID が 0 以上 | その値(ビルドで固定。従来どおり。NVS より優先) | config |
| 2 | NVS に番号があり、0〜255 の範囲 | NVS の番号 | nvs |
| 3 | それ以外(既定) | Wi-Fi の MAC の末尾 1 バイト(従来の -1 の動き) | mac |

- 範囲を 0〜255 にしたのは、MAC の末尾 1 バイトと同じ範囲にして、PC 側の道具の前提を変えないため。範囲外の値が NVS に
  あれば無視して MAC の番号にする(起動のログに警告)。
- 起動のログ: `[dyne] node id 38 source=nvs (nvs=38, mac last byte=37)` と、従来の見出し `==== Dyne node-esp32 (node id 38) ====`。
  見出しは NVS を読んだ後に出すため、`[dyne] heap(boot)` と `[dyne] boot #…` の行の後に移した。
- online の status(dyne/node/<id>/status)に `"id_src":"mac"|"nvs"|"config"` を足した。NVS を書き換えて次の起動で
  番号が変わる場合は `"id_next":N` も載る(再起動を忘れていることが分かる)。
- USB の書き込み(build.ps1 -Flash、first_flash.py)と OTA は NVS の区画を書かないので、NVS の番号は書き直しても残る。
  消えるのは、フラッシュ全体の消去(first_flash.py --erase、idf.py erase-flash)と、パーティションテーブルの変更などで
  NVS を作り直したとき(dyne_ops.c の dyne_ops_boot_early)。

**ctl 命令 node_id**(dyne/ctl/<id> または dyne/ctl/all。返事は dyne/node/<今の番号>/id/ack、QoS1、retain なし):

| 命令 | 動き |
|---|---|
| `{"cmd":"node_id"}` | 今の番号・決め方・NVS の値・次の起動の番号を返す。dyne/ctl/all に送ると全台が返事をする(番号と MAC の一覧になる) |
| `{"cmd":"node_id","mac":"02:00:00:00:02:25","id":38}` | この MAC の基板の NVS に 38 を書く。ほかの基板は黙って無視する |
| `{"cmd":"node_id","mac":"02:00:00:00:02:25","id":-1}` | NVS の番号を消す(次の起動から MAC の番号に戻る) |
| 上の2つに `"reboot":true` を足す | 番号が変わる場合は、返事の後にすぐ再起動する |

- **mac は設定・消去では必須**。番号の重なった 2 台は同じ dyne/ctl/<id> を購読しているので、mac が無いと両方が同じ番号に
  書き換わる。mac の書き方は `02:00:00:00:02:25`・`02-00-00-00-02-25`・`020000000225`(大文字小文字は問わない)。
  mac が無い設定の命令は `"err":"mac required"` で断る。
- id は 0〜255 の整数か -1(消す)。計算の実行中(BSP の最中)は `"err":"running"` で断る(フラッシュへの書き込みの間は
  フラッシュ上のコードが止まり、刻みの時間を乱すため)。
- 返事の例: `{"ok":true,"op":"set","mac":"02:00:00:00:02:25","id":37,"src":"mac","stored":38,"next":38,"next_src":"nvs","reboot":false}`
  (id・src は今動いている番号、next・next_src は次の起動の番号)。menuconfig で番号を固定した像では、NVS に書いても
  next は config の番号のまま(next_src = config)。

**新しい番号が効くのは次の起動から**(既定では再起動しない)。番号は MQTT の購読(dyne/cfg/<id>・dyne/ctl/<id>)、
接続時に登録する遺言(LWT。切断時にブローカーが dyne/node/<id>/status へ offline を貼る)、貼り置きの status、
ワイヤ形式v2 の送信元、IMU の標本のトピックに使われている。走っている途中で変えると古い番号の購読・遺言・貼り置きが
残り、計算の途中なら送信元が変わる。再起動すれば全部が新しい番号でそろう。`"reboot":true` か、ctl 命令 reboot で再起動する。

**手順**(番号の重なった基板 B に 38 を与える。B の MAC は first_flash.py の CSV か、`{"cmd":"node_id"}` の返事で分かる):

1. 全台を Wi-Fi とブローカーにつないだ状態で(Git Bash の例。PowerShell 5.1 では -m の中の `"` を `\"` にする):
   `mosquitto_sub -h <ブローカー> -t 'dyne/node/+/id/ack' -v` を別の窓で開いておき、
   `mosquitto_pub -h <ブローカー> -t dyne/ctl/37 -m '{"cmd":"node_id","mac":"02:00:00:00:02:25","id":38,"reboot":true}'`
2. 返事が `"ok":true,"next":38` で、B が再起動して dyne/node/38/status に `"id_src":"nvs"` の online が出れば完了。
   もう 1 台(A)は 37 のまま(返事もしない)。
3. 記録を直す: B を USB につないで `... first_flash.ps1 --skip-flash --port COM5`(CSV に 38 の行が足される。dup_id の判定は
   MAC ごとに最新の行で比べるので、B の古い 37 の行は数えない)。基板のラベルも書き直す。

- dyne/ctl は retain 禁止の規約どおり、貼り置いた命令は無視される(-r を付けない)。
- 検査(基板不要): `node-esp32/host_test/node_id_test.c`(決め方の順、範囲外の NVS の値、MAC の文字列、命令の解釈、
  MAC の番号が重なった 2 台に別の番号を与える流れ)。ビルドと実行は同ファイルの先頭。`scripts/ci-check.sh` の [6] でも動く
  (C コンパイラが無ければ飛ばす)。

| ファイル | 内容 |
|---|---|
| node-esp32/main/dyne_node_id_core.c/.h(新規) | 決め方と命令の解釈(ESP-IDF に依存しない) |
| node-esp32/main/dyne_node_id.c/.h(新規) | dyne_node_id()(main.c から移した)、NVS の読み書き、ctl 命令 node_id、status の項目 |
| node-esp32/main/main.c | 命令の受け口。起動の見出しを NVS の初期化の後へ |
| node-esp32/main/dyne_mqtt.c | online の status に id_src・id_next |
| node-esp32/main/Kconfig.projbuild | DYNE_NODE_ID の説明(-1 = NVS、無ければ MAC) |
| node-esp32/main/CMakeLists.txt | 新しいソース |
| node-esp32/host_test/node_id_test.c(新規) | PC 上の検査 |
| scripts/first_flash.py, scripts/tests/test_first_flash.py | 起動のログの決め方の行を読む。NVS の番号は MAC と照らさない。重なりの判定を MAC ごとの最新の行で。dup_id のときに命令の例を表示 |
| scripts/ci-check.sh | [6] 上の検査を C コンパイラでビルドして動かす |

### 9.1 各部分の確認(1台、USB 給電のまま)

| # | 確かめること | やり方 | 合格 |
|---|---|---|---|
| ① | 起動・自己検査 | モニタで `wire v2 envelope selftest: PASS`、`[dyne] heap(...)` の4行 | PASS。after-wasm の largest(内部 RAM の連続の空き)を記録 |
| ② | Wi-Fi・MQTT・時刻同期 | status の online、tsync | 従来どおり |
| ③ | 測距(センサだけ) | VL53L5CX をつないで起動。sense の range.found = true、init_ms ≈ 2300。range に 15Hz でフレーム | 欠番なし(seq が連続)、d_mm が妥当 |
| ④ | 区画の上下(どの行が水平か) | `{"cmd":"range","row":-1}` で全区画を出し、机の端など水平な縁を見て、水平の中央に当たる行を決める | 行を記録し、row に入れる |
| ⑤ | **区画の左右** | row を決めた状態で、センサの**左側**(センサから見て)だけに手をかざす | **d_mm の先頭の方(区画0側)が近くなれば flip=true のままで正しい**。末尾側が近くなれば `{"cmd":"range","flip":false}` |
| ⑥ | サーボ(センサなし) | サーボを別の電池でつなぐ。`{"cmd":"pan","en":true}` → 8 位置を約 4 秒で回る(フレームが無いので各位置 1.5 秒で進む)。`{"cmd":"pan","goto":90}` で中央に止まる | 順に回る。止まった後にうなり・震えが無い(PWM が止まる) |
| ⑦ | サーボの向きの較正 | `{"cmd":"pan","goto":<各位置の deg>}` で1位置ずつ止め、竹串の指針と分度器で実際の向きを読む(同じ位置へ必ず**小さい角度の側から**戻して3回読む) | 各位置の 実際の向き − 取り付けの向き を true_deg に。繰り返しのばらつき(1度以内か)を記録 |
| ⑧ | 首振り + 測距 | `{"cmd":"pan","en":true,"id_base":0}` で range の sid が 0〜7 と順に変わる。sense の pan.discard/accept | 一巡 4 秒前後。動いている間のフレームが採られていない |
| ⑨ | IMU | Modulino をつないで起動。sense の imu.found = true、addr。imu に 100ms ごとに約 10 標本 | 静止で az ≈ 1g(生の値 ≈ 1/sa)、ovr = 0、orphan = 0、us_per_tick が 25 の ±1.5% 以内 |
| ⑩ | 電池 | 単3×4 を 5V ピンへ(USB は外す)。Wi-Fi + 測距(または IMU)の連続動作で、止まる・再起動するまでの時間 | 実験1回ぶん(数時間)。再起動の理由は online の status(G11、実装済み) |
| ⑪ | サーボの電流 | ⑩の状態で首振りを続け、マイコンが再起動しないか | 再起動なし |

### 9.2 計算部品と組み合わせる(机の上、2台以上)

- 形: sdf2d 縮小版を OTA で配り、dyne/cfg/<id> に field2d と `"feed":{"range":true}` を入れて start。
  done の range_fed > 0・range_rej = 0、field2d の CRC が回ること。
- IMU: imu-fusion-actor を配り、`"feed":{"imu":true}` で start。done の imu_pushed > 0、imu_late・imu_early が小さいこと。
- 波動方程式: wave2d 縮小版で、シミュレータの同じ構成と CRC が一致すること(実験10)。

---

## 10. 未確認の点・作っていないもの

1. **全て実機で未確認**(DevKitC-VE も ESPr も、この版のファームは一度も動かしていない)。
2. VL53L5CX の区画の**左右**(flip の既定 true)と、どの**行**が水平の中央か(既定 3)。実験0の④⑤で決める。
3. LSM6DSOX の FIFO で、時刻印の語が値の語の**前**に入るという前提(ST の資料の図からの理解)。逆なら標本の時刻が
   1 周期(104Hz で 9.6ms)早くずれる。orphan の計数と、手で机を叩いた瞬間の時刻を複数台で比べれば分かる
   (実験9の準備で確かめる)。FIFO の大きさ(3KB と見て余裕を取った)も資料で要確認。
4. Modulino Movement の I2C 番地(0x6A と 0x6B の両方を探すので動作には影響しない)。
5. SG90 が 3.3V の信号で動くか(一般には動くと言われる)、速さ(ms_per_deg = 2.5 は見込み)、PWM を止めたときに
   姿勢が保たれるか、繰り返し精度。
6. PSRAM に置いた計算部品の速さ(内部 RAM に入らなかった場合)。DevKitC-VE で内部 RAM の連続の空きがどれだけ残るか。
7. OS の時計を 1ms 刻み(CONFIG_FREERTOS_HZ=1000)にした影響(既存の処理は pdMS_TO_TICKS なので意味は変わらないが、
   割り込みが 10 倍になる)。
8. 実機の測距・IMU の入力でのビット一致の確かめ方: range_fed / imu の記録をシミュレータで再生する道具は**未作成**。
9. 分割送信は作っていない(§6)。
10. 首振りの位置の変更(deg の変更)は、走っている計算(BSP)の途中では反映されない(センサの一覧の注入は start のとき)。
    首振りの設定は start の前に済ませる。

---

## 11. ユーザーに決めてほしいこと

1. **flip の既定**(区画の左右の反転): 今は「反転する」(SparkFun の例に合わせた推測)。実験0の⑤で確かめてから、
   既定を固定するか。
2. **送る行**: 今は 8×8 の 1 行(既定 3)だけをアクターへ送る(シミュレータの模擬センサと同じ1行の形)。
   全区画を送ってアクターに行を選ばせる案(row = -1)もある。推奨は1行(受け渡しが小さく、シミュレータと同じ形)。
3. **G10**: 分割送信は今は作らない(§6 の (c))でよいか。
4. **首振りの1位置のフレーム数**(既定3)と一巡の時間(約4秒): 実験6の「箱の追従」の速さと、覆いの密度のどちらを取るか。
5. **形の縮小版が 2 ページ(128KB)になった件**: ESPr で使わないなら問題なし。ESPr でも使うなら、形の速さの作業(別の作業者)に
   1 ページへ戻せるか確認が要る。
6. **ノード番号が MAC の末尾 1 バイトで重なった場合の直し方**(2026-11-15 追記、§9.0.1): (a) 重なった基板だけ
   menuconfig で番号を固定してビルドし直す(同じ像を全台に書く方針から外れる。OTA で全台に配るときも別の像が要る)、
   (b) 番号を NVS(フラッシュの設定の保存領域)に置き、ctl 命令や最初の書き込みの道具で書き込めるようにする(ファームの変更が要る)、
   (c) MAC の末尾 2 バイトなど重なりにくい決め方に変える(番号が 255 を超え、PC 側の道具とトピックの扱いの確認が要る)。
   推奨は (b)(像は 1 つのまま、基板のラベルと番号を人が決められる)。15 台が届いて重なりが無ければ当面は不要。
   → **(b) に決定**(ユーザー、GitHub issue #19)。作ったものは §9.0.2(2026-11-15)。最初の書き込みの道具からは書かず、
   ctl 命令 node_id(Wi-Fi・ブローカーが要る)で書く。シリアルから書く方法(Wi-Fi なしで書ける)は作っていない。

---

## 12. 追記(2026-11-13): 形の区画のメッセージをファームから送る

PC 側の常駐版(scripts/twin_live.py)が受ける「形の区画のメッセージ」(docs/2026-11-13-pc-pipeline-rl.md §5.2、
参照実装 scripts/dyne_shape_msg.py)を、ワイヤ形式v2 の経路から送るようにした。

### 12.1 何をするか

- 対象: 積んだ計算部品が **dyne_get_aux を持つ**とき(形の格子型アクター。縮小版も持つ。クレートは変更していない)。
- 送る間隔: dyne/cfg/<id> の `mode.shape_every`(ステップ数)。形のアクターなら**既定 15**(15Hz で約1秒)、0 = 送らない。
  形のアクターでないのに 1 以上を指定すると設定を断る。最後のステップの後にも必ず1通送る。
- 中身: §5.2 のとおり。トピック dyne/node/<node.idx>/shape、QoS 0、retain なし。52 バイトの見出し + φ・d・w(f32)。
  - φ = dyne_get(k)、d = dyne_get_aux(1, k)、w = dyne_get_aux(2, k) を f32 に丸めたもの(k = j·nx + i、k < dyne_len)。
  - ステップ = 計算し終えたステップ番号(field2d の要約と同じ番号)。時刻印 = 送る時点の同期した時刻(同期前は 0)。
  - ix0・iy0・nx・ny・h = 設定注入の field2d の値。gnx・gny = field2d に任意で足した `"gnx"`・`"gny"`(無ければ 0)。
  - x0・y0 = 設定注入の field2d.cfg の形のアクターのキー 0・1(CFG_ORIGIN_X/Y。無ければ 0)。
  - CRC-32 は外箱と同じ関数(dyne_env_crc32、zlib.crc32 と同じ値)。
- done の status に shape_sent・shape_skip を足した。

### 12.2 MQTT の送り方と大きさ

- 1 通の大きさ: 52 + 12·nx·ny バイト。形の縮小版の上限(枠なし 1,080 点)で **13,012 バイト**、
  room6_move の 4×4 分割(約 36×27 点)で約 11.7 kB、§5.2 の例(18×27 点)で 5,884 バイト。
- **送信の緩衝(buffer.out_size = 2,048)は大きくしなかった**。ESP-IDF 5.5.5 の MQTT は、緩衝より大きいメッセージを
  esp_mqtt_client_enqueue で積むと、残りをアウトボックスへ丸ごと写して分けて書く(mqtt_client.c の fragmented の処理を読んで
  確認)。分けて送るのは TCP の上なので、PC が受け取るのは1通のまま(分割の仕組みは要らない)。
- 計算(BSP)を止めないよう、送信を待たずにアウトボックスへ積む。アウトボックスが **32 KB** を超えていたら(ネットワークが
  遅い・切れた)送らずに shape_skip を数える(メモリを食い尽くさないため)。
- 組み立ての領域(最大 13 KB)は送るたびに確保して、積んだらすぐ放す(アウトボックスが自分の写しを持つ)。
- 読み出しの時間: 1 通に 3·nx·ny 回の計算部品の呼び出し(1,080 点で 3,240 回)。インタプリタでは 1 回数 µs〜十数 µs と見て、
  数十 ms かかる見込み(**未計測**)。15 ステップに1回なので、1ステップあたりでは数 ms 程度。

### 12.3 確認

- PC 上の照合: `node-esp32\host_test\shape_msg_test.c`(C で 4 通り(1×1、5×4、36×27、18×27)の区画を組み立ててファイルへ)+
  `node-esp32\host_test\shape_msg_check.py`(scripts/dyne_shape_msg.py の unpack で読み、同じ値を pack で組み立て直して比べる)
  → **4 通りともバイト一致、PASS**。手順は shape_msg_test.c の冒頭。
  - 気づいた点: 照合の値を C で作るとき、コンパイラが掛け算と足し算を1命令(FMA)にまとめると Python の値と最後の1ビットが
    ずれた(-ffp-contract=off で解消)。メッセージの組み立ての誤りではなく、検査の値の作り方の問題。ファームは値を
    計算部品から読んで f32 に丸めるだけなので影響しない。
- ビルド: 下の表。
- 実機・PC の常駐版との通しの確認は未実施(ESP-IDF の MQTT が 13 KB の 1 通を実際に送れるかを含む)。

### 12.4 変更したファイル

| ファイル | 内容 |
|---|---|
| node-esp32/main/dyne_shape_msg.c/.h(新規) | メッセージの組み立て(ESP-IDF に依存しない) |
| node-esp32/main/node_v2.c | mode.shape_every・field2d.gnx/gny、dyne_get_aux の読み出し、publish_shape、done の計数 |
| node-esp32/main/dyne_mqtt.c, dyne_node.h | dyne_mqtt_pub_bin(バイナリをアウトボックスへ。アウトボックスの上限つき) |
| node-esp32/main/CMakeLists.txt | dyne_shape_msg.c |
| node-esp32/host_test/shape_msg_test.c, shape_msg_check.py(新規) | PC 上の照合 |

### 12.5 ビルドの結果(本節の変更の後、警告 0 件)

| 基板 | アプリの大きさ | アプリのパーティション(1面) | 静的 RAM(DRAM) | IRAM |
|---|---|---|---|---|
| espr | 1,342,039 B(+1.7 KB) | 79% | 67,000 B(+16 B) | 96,763 B |
| devkitc-ve | 1,348,611 B(+1.8 KB) | 43% | 67,212 B(+16 B) | 102,091 B |

注意: 形の縮小版は線形メモリが 2 ページ(§3.2)なので、このメッセージを実際に送るのは DevKitC-VE の見込み。

---

## 13. 追記(2026-11-13): IMU の融合のアクターを実機で動かすための対応(imu-fusion の §3.5・§9 U3)

docs/2026-11-13-imu-fusion.md の「ファームへの依頼」2件と、実験9③のための「わざと同期を崩す」設定を入れた。
対象はワイヤ形式v2 の経路で `"feed":{"imu":true}` のとき(計算部品 = crates/imu-fusion-actor)。

### 13.1 融合の出力の発行(依頼 (1))

- 各ステップの dyne_step の後、区間の番号 dyne_get(0) が前回と変わったら(= 新しい区間の融合が確定したら)、
  `dyne/node/<id>/imu_fused` に `{"step":s,"v":[dyne_get(0),…,dyne_get(21)]}` を発行する(QoS 0、アウトボックスへ積む)。
  s は計算し終えたステップ番号、v の個数は dyne_len(= G_LEN = 22。上限 32)。区間の番号が −1(まだ無い)のうちは送らない。
  区間の長さ M を見ずに番号の変化で決めるので、M を変えても境目を取りこぼさない。
- 値は %.17g(f64 を正確に戻せる桁)。有限でない値は Python の json が読む `NaN` / `Infinity` / `-Infinity` で書く
  (scripts/imu_log.py は json.loads で読む)。
- `<id>` は生の標本(dyne/node/<id>/imu)と同じ **dyne_node_id()**(MQTT の dyne/cfg/<id> の番号)。imu_log.py が
  生の標本と融合の出力を同じノード番号で並べるため。field2d の要約などは node.idx を使うので、IMU の実験では
  node.idx とノードID を同じにしておくのがよい。
- done の status に imu_fused_sent を足した。

### 13.2 開始の時刻と進み方(依頼 (2))

- ステップの周期: `cluster.step_period_ms` を省くと dt から決める(dt = 0.02 → 20 ms)。dt と違う値を入れると設定を断る。
  dt が整数の ms でない場合も断る。
- 開始の時刻 t0(同期した時刻 [µs])の決め方(上ほど優先):
  1. 設定の `"imu":{"t0_us":T}`
  2. field2d.cfg のキー 2(CFG_T0_US)。docs/2026-11-13-imu-fusion.md §7.2 の書き方
  3. 自動: 今 + start_delay_ms(既定 3000)以降で最初の grid_ms(既定 10000)の倍数。全ノードがほぼ同時に start を
     受ければ同じ t0 になる(start の到着の差が 10 秒の境目をまたぐと食い違う。確実にするには PC が t0 を入れる)
- 決めた t0 をアクターの CFG_T0_US に入れ(PC の設定の後に入れるので、PC の値と同じか上書き)、running の status に
  imu_t0_us・imu_time_mode・imu_offset_us・step_period_ms を載せる。
- ステップ0の前に、同期した時刻が t0 になるまで待つ(stop で中断できる)。以後のステップ s は esp_timer の上で
  t0 + s·dt に当たる時刻に始める。t0 を過ぎてから start した場合は、遅れたぶん各ステップを待たずに進めて追いつく
  (遅れは done の imu_late_start_ms)。開始前にため置きにあった標本は捨てる。
- 時刻同期ができていないノードは start を断る(err "imu run needs time sync")。
- 注意: 標本は FIFO の読み出しの間隔(imu.batch_ms、既定 100 ms)ごとにまとめて届くので、区間の境目では最大 300 ms の
  待ち(feed.imu)が毎回 100 ms 程度かかりうる。その後のステップは追いつくために詰めて進む。待ちを減らすには
  `{"cmd":"imu","batch_ms":20}`(104 Hz で 1 回 約2標本)を勧める(**未計測**)。

### 13.3 わざと同期を崩す(実験9③)

設定の `"imu"` に:

| 欄 | 意味 | アクターのキー |
|---|---|---|
| `"offset_us": a` と `"offset_per_idx_us": b` | 時刻印に足すずれ = a + b × node.idx [µs](例: b = 20000 で ノード i に 20 ms × i) | CFG_TS_OFFSET_US(3) |
| `"time_mode": "arrival"` | 時刻印を使わず届いた順で区間に入れる(アクターの TIME_ARRIVAL) | CFG_TIME_MODE(1) |
| `"time_mode": "stamp"` | 時刻印で決める(既定) | 同上 |

ずれはアクターが区間を決めるときにだけ足す(ファームの時刻同期・生の標本の時刻印は正しいまま。imu-fusion §2)。
どれも省けば注入しない(PC が field2d.cfg に直接入れた値が効く)。

設定の例(2×4 の格子のノード i):

```json
{"gen":1, "cluster":{"steps":90000,"alpha":1,"dt":0.02},
 "node":{"idx":3},
 "field2d":{"nx":1,"ny":1,"ix0":0,"iy0":0,"h":1,
            "faces":[{"face":"w","key":"interface","iface":2},{"face":"e","key":"interface","iface":3}],
            "cfg":[[0,10],[5,25],[6,1]]},
 "ifaces":[{"id":2,"peer":"192.168.137.12:7777"},{"id":3,"peer":"192.168.137.14:7777"}],
 "feed":{"imu":true},
 "imu":{"t0_us":1763020800000000, "offset_per_idx_us":20000}}
```

(nx・ny・h は融合のアクターは使わないが、ファームの設定の検査のため正の値を入れる。)

### 13.4 確認

- PC 上の検査: `sense_test.c` に [6] を追加(t0 の自動の決め方、imu_fused の JSON の組み立て・容量不足・有限でない値)。
  書き出した JSON を `node-esp32\host_test\imu_fused_check.py` が json.loads で読み、22 個の値が f64 としてビット単位で
  戻ることを確かめた → PASS。
  - 気づいた点: §12 と同じく、検査の値を C で作るときの FMA(掛け算と足し算を1命令にまとめる)で Python の値と
    最後の1ビットがずれたので、sense_test も -ffp-contract=off でビルドする(手順は冒頭に記載)。
- 実機・2台以上での通し(t0 で全ノードがそろって始まるか、遅れ・先すぎの計数)は未確認。

### 13.5 変更したファイル

| ファイル | 内容 |
|---|---|
| node-esp32/main/node_v2.c | 設定 `imu`(t0_us・start_delay_ms・grid_ms・time_mode・offset_us・offset_per_idx_us)、step_period_ms = dt、t0 の注入と待ち、imu_fused の発行、done の計数 |
| node-esp32/main/dyne_sense_core.c/.h | dyne_imu_auto_t0、dyne_imu_fused_json(ESP-IDF に依存しない) |
| node-esp32/host_test/sense_test.c | [6] の追加、-ffp-contract=off |
| node-esp32/host_test/imu_fused_check.py(新規) | JSON を Python で読んで値を照合 |

### 13.6 ビルドの結果(本節の変更の後、警告 0 件)

| 基板 | アプリの大きさ | アプリのパーティション(1面) | 静的 RAM(DRAM) | IRAM |
|---|---|---|---|---|
| espr | 1,345,203 B | 79% | 67,024 B | 96,763 B |
| devkitc-ve | 1,351,795 B | 43% | 67,236 B | 102,091 B |

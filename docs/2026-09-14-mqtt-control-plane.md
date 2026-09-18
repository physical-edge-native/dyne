# MQTT制御面 第1段 規約 (docs\2026-09-14-mqtt-control-plane.md)

2026-09-14、発注7(7a)。統合ロードマップv3 3章原則7の確定設計に基づく、
制御面(コントロールプレーン)第1段の通信規約。本書が**規約の正本**であり、
独立の「軽量API」は設けない——各ノードが必ず応答すべきMQTTトピックと
メッセージ形式そのものが差込口である。

適用範囲と前提:

- **制御面のみ**。データ面(境界同期)は従来どおり自前40バイト意味論+UDP直送
  (ワイヤ形式v2)で、本規約の対象外。層の区別はロードマップ3章原則7
  (層1=メッセージ規約、層2=配送、層3=搬送路)。
- ブローカーは**ユーザーのPC上**(mosquitto等、認証なし・平文の閉域Wi-Fi前提)。
  第2段でzenoh-picoへ移行しMQTTが退役する可能性があるため、ノード実装は
  MQTTクライアント層を1ファイルに閉じ込める(node-esp32/main/dyne_mqtt.c)。
- ノードの起動時設定(ブートストラップ)は Wi-Fi SSID/パスワード・ブローカーURI・
  ノードIDの3つだけ(menuconfig/NVS)。**それ以外の設定はすべて本規約の
  設定注入で与える**(ロードマップ4章「全ノードは同一ファームウェアで、役割は
  起動時の設定注入で決まる」)。

## 1. トピック体系

`dyne/` を接頭辞とし、**ノードID**(整数。シミュレータのノード番号と同一)と
**界面ID**(クラスタ一意。界面 j = ノード j と j+1 の間、M0知見3)をトピック名に
対応させる。観測はワイルドカード購読で実現する(例は §4)。

| トピック | 向き | retain | 内容 |
|---|---|---|---|
| `dyne/cfg/<node_id>` | PC→ノード | **あり** | 設定一式(§2.1)。retainedなので後から起動したノードも即受信 |
| `dyne/cfg/<node_id>/ack` | ノード→PC | なし | 設定の受理/拒否(§2.2) |
| `dyne/ctl/all`、`dyne/ctl/<node_id>` | PC→ノード | なし | 制御コマンド(§2.3)。全体宛と個別宛 |
| `dyne/node/<node_id>/status` | ノード→PC | **あり** | 参加・離脱・状態(§2.4)。LWT(遺言)で不意の離脱も通知 |
| `dyne/node/<node_id>/field` | ノード→PC | なし | 温度場スナップショット(§2.5) |
| `dyne/node/<node_id>/sensor` | ノード→PC | なし | センサ読取値(§2.6。当面ログ発行のみ、同化への接続は発注8) |
| `dyne/node/<node_id>/rssi` | ノード→PC | なし | 近傍RSSI(§2.7) |
| `dyne/node/<node_id>/rt` | ノード→PC | なし | リアルタイム率統計(§2.8) |
| `dyne/iface/<iface_id>/residual` | ノード→PC | なし | 界面残差(§2.9)。**界面の左側ノードのみ**が発行(シミュレータの記録規則と同一) |

QoS: `cfg` / `ack` / `ctl` / `status` は QoS1(取りこぼし不可)、観測系
(`field` / `sensor` / `rssi` / `rt` / `residual`)は QoS0(周期発行のため
取りこぼし許容)。

## 2. メッセージ形式

すべてUTF-8のJSON。浮動小数点は往復変換で値が保存される表現(実装は `%.17g`)で
書く。時刻 `t_ms` はノードの起動からの経過ミリ秒(ノード間で同期しない。
順序づけはステップ番号で行う)。

### 2.1 設定注入 `dyne/cfg/<node_id>`(PC→ノード、retained、QoS1)

```json
{
  "ver": 1,
  "cluster": { "n_nodes": 5, "points": 65, "alpha": 1.0, "dt": 1e-4,
               "steps": 2000, "seed": 42 },
  "node":    { "idx": 2, "n": 13, "x0": 0.25, "x1": 0.4375 },
  "bc": {
    "left":  { "key": "interface" },
    "right": { "key": "zero_grad" }
  },
  "ifaces": [
    { "id": 1, "face": "left",  "peer": "192.168.4.11:7777" },
    { "id": 2, "face": "right", "peer": "192.168.4.13:7777" }
  ],
  "mode": { "role": "compute", "sensor": "dht20",
            "sensor_period_ms": 5000, "rssi_period_ms": 10000,
            "field_every": 50, "residual_every": 10, "rt_every": 100 },
  "assim": { "gamma": 0.3, "obs_period": 10, "obs_point": -1 }
}
```

- `bc.*.key` は F1 の境界条件キーに対応:
  `dirichlet`(+`"value": 0.0`)/ `interface` / `zero_grad` / `linear_extrap` /
  `transparent`。コードへの対応は dyne-core::actor と同一
  (0=dirichlet, 1=interface, 2=zero_grad, 3=linear_extrap, 4=transparent)。
- `transparent` は指数和係数を**PCが計算して注入**する:
  `{ "key": "transparent", "m": 32, "decay": [...], "half": [...], "weight": [...] }`。
  ノード側でexp/logを再実装せず、係数の出所をPC(dyne-core::transparent、
  構成パラメータ固定)に一元化して全ノードの数値一貫性を保つ。
- `ifaces[].peer` はデータ面(UDP直送)の相手アドレス。制御面が
  データ面の結線情報を配る、という役割分担(データ面自体はMQTTを通らない)。
- `seed` は現行の熱伝導アクターでは未使用(将来の学習型・群れアクターの
  重み初期化・個体シード用に規約へ含める)。
- `mode.role`: `compute`(計算参加)| `observe`(センサ・RSSI発行のみ。
  計算には参加しない)。`mode.sensor`: `none` | `dht20` | `bme280`。
- 観測周期(`*_every` はステップ単位、`*_period_ms` は実時間ミリ秒)を 0 に
  すると当該観測は無効。
- `assim` はデータ同化の最小形=観測点ナッジング(発注8、2026-09-18追記)。
  `gamma`(ナッジング強度 γ、0=無効。u←u+γ(y−u))、`obs_period`(観測周期、
  **ステップ単位**=シミュレータと同一の意味論)、`obs_point`(観測点=担当区間内の
  局所格子点。-1 = 中央 n/2、シミュレータの既定と同一)。観測値 y はセンサの
  最新読取(temp_c)。式はアクター内の dyne_observe にあり、シミュレータと実機は
  同一 wasm を使うため**式の共有が自動で成立**する(二重実装なし)。

### 2.2 設定応答 `dyne/cfg/<node_id>/ack`(ノード→PC、QoS1)

ノードは cfg を受信するたび**必ず**応答する:

```json
{ "ok": true,  "idx": 2, "gen": 1789700000, "crc32": "0x5A2B1C3D" }
{ "ok": false, "idx": 2, "err": "bad bc key: robin" }
```

`gen` は受理した設定の世代番号のエコー(§3.6。cfg に無ければ 0)。

`crc32` は受信したcfgペイロード(バイト列)のCRC32。PC側はこれで意図した設定が
届いたことを照合できる。拒否時は理由文字列。設定は `ctl` の `start` まで
何度でも上書きできる(最後に受理したものが有効)。

### 2.3 制御 `dyne/ctl/all`・`dyne/ctl/<node_id>`(PC→ノード、QoS1)

```json
{ "cmd": "start" }    // 受理済み設定でBSP計算を開始
{ "cmd": "stop" }     // 計算を中断し idle へ(設定は保持)
{ "cmd": "report" }   // 現在の統計 (rt) と status を即時発行
{ "cmd": "reboot" }   // 再起動
```

ノードは `start` を受けたら `status` を `running` にし、データ面のBSP協調を
開始する。設定未受理で `start` を受けた場合は `status` に `err` を発行して
無視する。
**号令の打ち消し規則(2026-09-18追記、事故究明の帰結)**: `start` はそれ以前に
受けた未消費の `stop` を無効化する(startは常に新しい意思の表明。idle中に
受けたstopが残留して次のstartを即中断する事故を防ぐ)。start より後に届いた
stop は従来どおり有効。ノードは stop 受信をログに可視化する。

### 2.4 状態 `dyne/node/<node_id>/status`(ノード→PC、retained、QoS1)

参加・離脱の通知。接続時に必ず発行し、LWT(遺言メッセージ。ブローカーが
切断を検知した時に代理発行)にも同トピックで `offline` を登録しておく:

```json
{ "state": "online",  "mac": "AA:BB:CC:DD:EE:FF", "ip": "192.168.4.12",
  "fw": "node-esp32 exp1", "heap": 123456 }        // 起動・ブローカー接続時
{ "state": "configured", "idx": 2 }                 // cfg受理後
{ "state": "running", "idx": 2, "step": 500 }       // start後 (rt_every毎に更新)
{ "state": "done", "idx": 2, "crc32": "0x...", "l2": 1.2e-4, "linf": 3.4e-4 }
                                                    // steps完走。crc32は温度配列
                                                    // バイト列のCRC32 (M2と同じ検証)
{ "state": "err", "idx": 2, "err": "..." }
{ "state": "offline", "reason": "lwt" }             // LWT (不意の離脱)
{ "state": "offline", "reason": "reboot" }          // 明示的な離脱
```

`mac` はRSSIマップ(§2.7)のMAC→ノードID対応表の材料。

### 2.5 温度場 `dyne/node/<node_id>/field`(QoS0、field_everyステップ毎)

```json
{ "step": 500, "t": 0.05, "x0": 0.25, "x1": 0.4375, "n": 13,
  "u": [0.1, 0.2, ...] }
```

シミュレータの `--field-csv` と同じ意味論(担当区間の格子点値)。ワイルド
カード購読 `dyne/node/+/field` で全域の場が観測できる。

### 2.6 センサ `dyne/node/<node_id>/sensor`(QoS0、sensor_period_ms毎)

```json
{ "t_ms": 123456, "kind": "dht20",  "temp_c": 25.3, "rh": 48.2 }
{ "t_ms": 123456, "kind": "bme280", "temp_c": 25.1, "rh": 47.9, "pres_hpa": 1013.2 }
```

読取失敗時は `{ "t_ms": ..., "kind": "dht20", "err": "i2c timeout" }`。
当面はログ発行のみ(データ同化への接続=発注8)。

### 2.7 近傍RSSI `dyne/node/<node_id>/rssi`(QoS0、rssi_period_ms毎)

```json
{ "t_ms": 123456, "ap_rssi": -48,
  "peers": [ { "mac": "AA:BB:CC:DD:EE:01", "rssi": -55, "n": 42 }, ... ] }
```

`ap_rssi` = 接続中APの受信強度(常に取得可能)。`peers` = 混雑モード
(プロミスキャス受信)で拾った**送信元MACごと**の受信強度(周期内の指数移動
平均)と観測フレーム数 `n`。データ面のUDPフレーム自体が測定源になるため、
計算参加ノード同士は追加通信なしで相互RSSIが得られる。MAC→ノードIDの対応は
PC側が `status` の `mac` で解決する(実験2の位置推定の入力)。

### 2.8 リアルタイム率 `dyne/node/<node_id>/rt`(QoS0、rt_every ステップ毎)

```json
{ "step": 500, "window": 100,
  "step_ms":    { "mean": 81.2, "min": 79.8, "max": 95.0 },
  "compute_ms": { "mean": 79.5, "min": 79.1, "max": 80.2 },
  "wait_ms":    { "mean": 1.7 },
  "dt_ms": 0.1, "rt_rate": 0.00123 }
```

`rt_rate` = 仮想時間刻み Δt / 実ステップ時間(平均)。1.0以上でリアルタイム。
`dt_ms − compute_ms` が有界非同期転換(12月)の「Δt ≥ 計算時間」制約の余裕
そのもの。BSPの現状値を布石として記録する(ロードマップ4章実験1)。

### 2.9 界面残差 `dyne/iface/<iface_id>/residual`(QoS0、residual_every毎)

```json
{ "step": 500, "residual": 1.23e-3 }
```

|自分の端の勾配 − 相手から受けた勾配|。**界面の左側ノード(面=右)のみ**が
発行する(シミュレータの二重記録回避と同じ規則)。Trig2系の監視・ダッシュ
ボードの入力。

### 2.10 イノベーション `dyne/node/<node_id>/innov`(QoS0、obs_period毎。2026-09-18追記)

```json
{ "step": 500, "i": 8, "y": 25.31, "d": 0.042, "y_t_ms": 123456 }
```

観測点ナッジング(§2.1の assim)の適用ごとに発行。`i` = 観測点(局所格子点)、
`y` = 観測値(センサ最新読取)、`d` = イノベーション(**適用前**の y − u_i)、
`y_t_ms` = その読取のノード時刻。閾値判定はしない(計測のみ。イベント検知の
2信号のうちイノベーション側の計測開始 = ロードマップ9章)。センサ読取が一度も
成功していない間は発行しない(ナッジングも適用されない)。

## 3. ノードの応答義務(まとめ)

1. ブローカー接続時: `status` = `online`(retained)を発行し、LWTを登録する。
2. `dyne/cfg/<自ID>` 受信時: 検証して**必ず** `ack` を発行。受理なら
   `status` = `configured`。
3. `dyne/ctl/all`・`dyne/ctl/<自ID>` の `cmd` に応答(§2.3)。
4. `running` 中: `field` / `rt` / `residual`(界面左側のみ)/ `sensor` / `rssi` を
   設定周期で発行。
5. 完走時: `status` = `done`(CRC32つき)。切断時: LWTにより `offline`。

## 3.5 異常時・部分成功の扱い(2026-09-18追記、検収回答の3点)

1. **離脱(LWT発火)時の残存ノードの挙動**: 制御面のLWTは**PCへの通知**であり、
   データ面のBSPループには自動では伝わらない。界面の相手を失った残存ノードは
   再送バックオフを続けながら**その界面で待ち続ける(ストール)**。pause扱い
   (相手復帰での自動再開)では**ない**——相手が再起動して再startしても
   ステップ0から始まるため整合しない。回収手段は制御面からの
   `ctl: stop`(受信待ちループ内で毎スライス確認しており、走行中でも設定保持の
   まま idle へ戻せる)→ 全ノードへ再設定・再start。離脱の自動治癒
   (handover/ADDのβ意味論の実機化)は第1段のスコープ外で、立ち上げ層
   (発注9)以降で扱う。
2. **設定注入の再送・タイムアウト・部分成功**: cfg は retained+QoS1 なので
   配送はブローカーが保証し、ノードは cfg を受信する**たび**に ack を返す
   (冪等。再注入=再send がそのまま再送手順)。PC側の規約: **start は対象の
   全計算ノードの ack(ok) を確認してから発行する**。ack がないノードには
   タイムアウト(目安10秒)で cfg を再注入する(retained のため、ノードの
   再接続時にも自動配送される)。部分成功のまま start を発行した場合の挙動は
   定義済み: cfg 未受理ノードは `status: err`(start without cfg)を発行して
   無視し、受理済みノードはBSPで相手を待ってストールする(1.と同じ回収手順:
   stop → 再設定 → start)。scripts/mqtt_config.py の `--start` は簡易実装
   (1秒猶予)であり、確実な運用では mqtt_observe.py で ack を確認してから
   start だけを発行する。
3. **プロミスキャスRSSIの縮退設定**: 2段階ある。(a) **実行時**: 設定注入で
   `rssi_period_ms: 0` にすると発行を止めるだけでなく**プロミスキャス受信
   そのものを停止**する(受信コールバックのCPUコストがゼロになる。period>0 の
   再注入で再開)。AP RSSI も止まる。(b) **ビルド時**: Kconfig の
   DYNE_RSSI_SNIFF を無効にするとスニファをコンパイルから外す(rssi トピックは
   AP RSSI のみになる)。

## 3.6 設定・号令の鮮度管理(2026-09-18追記、実機確認⑥の知見)

メッセージを**宣言型(状態)**と**命令型(号令)**に区別し、retain の可否を固定する:

1. **cfg は宣言型 = retained が正**。「このノードの現在あるべき設定」をブローカーが
   常に保持し、**再起動・後着のノードが号令なしに自動で configured になるのは
   意図した仕様**(§2.1。再起動のたびに再注入する手間を省き、LWT復帰も自動化する)。
   鮮度は**世代番号 `gen`**(cfg 直下、注入ツールが単調増加で付与。実装は
   UNIX秒)で管理する: ノードは受理した gen を ack と status(configured)に
   エコーし、PC側は「全ノードが同一 gen か」で設定の斉一性を照合できる
   (古い貼り置き cfg で走る事故の検出)。ノード構成を変えて cfg が不要になった
   IDは、空ペイロードの retained で掃除する:
   `mosquitto_pub -r -n -t dyne/cfg/<id>`。
2. **ctl は命令型 = retain 禁止**。号令は鮮度が命であり、貼り置くと start 直後に
   古い stop が再配達されて即停止する等の事故になる(実機確認⑥で実発生)。
   発行側は retain=False を明示し(mqtt_config.py 実装済み)、**受信側(ノード)も
   retained フラグの立った ctl を無視**する(二重の防御。無視時は警告ログ)。
   誤って貼り置かれた ctl の掃除: `mosquitto_pub -r -n -t dyne/ctl/all`。
3. status は宣言型(retained)= 各ノードの最新状態の掲示板。LWT も同トピック。
   観測系(field 等)は非 retained(時系列であり最新値の掲示に意味がないため)。

## 4. PC側の観測(ワイルドカード購読の例)

```
mosquitto_sub -t "dyne/#" -v                 # 全部
mosquitto_sub -t "dyne/node/+/field" -v      # 全ノードの温度場
mosquitto_sub -t "dyne/iface/+/residual" -v  # 全界面の残差
mosquitto_sub -t "dyne/node/+/sensor" -v     # 全センサ
python scripts\mqtt_observe.py --broker localhost   # 購読+CSV記録 (発注7同梱)
python scripts\mqtt_config.py --broker localhost --nodes 2 --points 65 \
    --bc zero_grad --peers 192.168.4.11,192.168.4.12 --start
                                             # チェーン設定の生成・注入・開始
```

## 5. 第2段(zenoh移行)への備え

- 本規約の意味論(トピック=リソース名、retained cfg、LWT)はzenohのキー空間・
  クエリアブルへ機械的に写せる範囲に留めた。
- ノード側はMQTT依存を dyne_mqtt.c に閉じ込め、規約の意味論(§2〜3)は
  搬送路から独立に保つ。B-15判定(C6検証、10月)でzenoh-picoが立てば
  クライアント層のみ差し替える。

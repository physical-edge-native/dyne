# 2026-09-02 発注6: 群れ移植・案Bの具体設計(docs\2026-09-02-swarm-port-design.md)

docs\2026-09-02-swarm-port-options.md §6 のユーザー決定(2026-09-02、案B+小問指定)を
受けた具体設計。第1段(共通基盤 dyne-core の切り出し)は実装済み。第2段(群れ部分)は
**ユーザーが手元で全回帰のビット一致を確認した後**に着手する。

## 1. 決定事項の要約(2026-09-02 ユーザー決定)

- **案B**: 共通基盤をライブラリ(crates/dyne-core)へ切り出し、dyne-sim をその上に
  再構成(全回帰のビット一致で無変化を証明)。群れは新ホスト crates/dyne-swarm-sim
  として実装。旧 crates/dyne-swarm は参照実装として凍結。
- 差込口=汎用メッセージバッファ方式。**バッファ上のメッセージ形式はワイヤ形式v2と
  同一規約(二重定義の禁止)**。バッファは固定長、**あふれは破棄+計数**。
  既存の熱伝導アクターの差込口は当面不変(ホスト側で吸収、移行は2次元化の際)。
- 乱数=アクター内蔵+シード注入。個体シードは**大域シードと個体IDからの固定式**で
  導出(下記4章)。実装は整数演算のみ。
- 受け入れ基準に**定量帯**を含める: 同一シナリオで最適温度域への到達率が旧実装記録
  (93〜98%)と同水準(下記6章)。

## 2. クレート構成(第1段=実装済み)

```
crates/dyne-wire  … ワイヤ形式v2の単一定義 (no_std)。ホストとWASMアクターの双方が依存
crates/dyne-core  … ホスト側共通基盤 (rng / event / net / actor、wire を再エクスポート)
crates/dyne-sim   … dyne-core の上に再構成 (ロジック不変。全回帰ビット一致で証明)
```

dyne-core の内容(dyne-sim からの移設。**ビット互換**、ロジック変更なし):

| モジュール | 内容 | 移設元 |
|---|---|---|
| rng | xorshift64*(整数演算のみ)+ 個体シード導出 derive_seed | dyne-sim の Rng |
| event | EventQueue(離散イベント・仮想時刻。同時刻は予約順=seq昇順) | Sim の heap/events/seq |
| net | VirtNet(損失・遅延ジッタ・分断。乱数消費順は従来と同一) | Sim::transmit/partition_blocks |
| actor | wasmi 実行ラッパ Actor(Dyne C ABI)と FACE_*/BC_* 定数 | dyne-sim の Actor |

v1(M0の40バイト形式)は crates/dyne-proto に残る歴史的定義であり触らない。
v2 の IFACE_SCALAR(計40バイト)がその意味論を包含する。

## 3. ワイヤ形式v2(crates/dyne-wire が単一定義)

roadmap 2.5 の v2 改訂(時刻印・物理量種別・帯ペイロード)+ 発注6 の追加種別
(位置・伝聞)。リトルエンディアン。

```
offset size field
0      1    version  = 2
1      1    kind     0=IFACE_SCALAR / 1=IFACE_BAND / 2=POSITION / 3=HEARSAY
2      1    quantity 物理量種別: 0=温度, 1=流速, 255=指定なし
3      1    reserved = 0
4      4    src      u32: kind=IFACE_*では界面ID、POSITION/HEARSAYでは個体ID
8      8    step     u64: 論理ステップ
16     8    t_us     u64: 時刻印 (仮想時刻µs。未使用なら0)
24     -    payload:
  IFACE_SCALAR: value f64, grad f64                     (計40B。v1の意味論を包含)
  IFACE_BAND:   count u32, pad u32, f64×count            (2次元界面=帯の先行定義)
  POSITION:     x, y, temp, gx, gy (f64×5)               (計64B)
  HEARSAY:      x, y, score f64, age_steps u64           (計56B)
```

POSITION/HEARSAY のフィールドは現行 dyne-swarm が交換している量に合わせた初期定義。
群れ実装の段で確定し、変更時は dyne-wire のコメントと本書を併せて改訂する。

**受信箱/送信箱の枠(固定長バッファ)**: メッセージは先頭から `[u32 len][メッセージ]`
で連続に詰める(len にヘッダを含む)。終端は len=0 または容量切れ。
**あふれ時**: 入り切らないメッセージは**破棄し、破棄計数を+1**(書いた側が計数を持つ。
ホストは統計に報告、アクターは自分の破棄数をエクスポートで返せるようにする)。
部分書き込みは行わない(壊れたメッセージをバッファに残さない)。
dyne-wire の MailboxWriter / MailboxReader がこの規約の実装であり、単体テストで
あふれ→破棄+計数を検証済み。

## 4. 個体シードの導出規則(固定式、文書化=本節)

```
seed_i = splitmix64( global_seed XOR (id+1) × 0x9E3779B97F4A7C15 )
(結果が 0 なら 0x9E3779B97F4A7C15 に置き換える)
```

- splitmix64 は標準的な整数ミキサ(乗算・XOR・シフトのみ)。実装は
  crates/dyne-core/src/rng.rs の derive_seed(単体テストつき)。
- id+1 とするのは id=0 で global_seed が素通りするのを避けるため。splitmix64 を
  挟むのは近い id 同士のシード相関(xorshift系の弱点)を切るため。
- 運用: ホストが derive_seed(大域シード, 個体ID) を計算し、dyne_cfg で
  アクターへ注入する。アクターは注入されたシードで内蔵 xorshift64* を初期化する
  (実機で各個体が自律動作する形と一致)。

## 5. 群れアクターの差込口(2026-09-04 実装で確定)

既存 C ABI(dyne_init / dyne_cfg / dyne_step 系)の拡張。実装
crates/swarm-agent-actor での確定版:

```
swarm_init(id: u32, x: f64, y: f64, t0: f64) -> i32 … 個体の初期化(初期位置と
    その真の温度。t0 は solo の自己履歴の初期値)
dyne_cfg(key: u32, val: f64)                  … パラメータ・シード注入。キーと
    個体規則の番号は dyne-wire::abi(CFG_MODE / CFG_T_OPT / CFG_TOL / CFG_SPEED /
    CFG_NOISE / CFG_RADIUS / CFG_GOSSIP_MAX_AGE / CFG_SEED_LO / CFG_SEED_HI、
    MODE_SOLO〜MODE_GOSSIP)に一元定義(二重定義の禁止を定数にも適用)。
    シードは LO→HI の順に注入し、HI の書き込みで内蔵乱数が初期化される
swarm_sense(temp: f64)                        … 知覚+送信箱の作成(temp=自位置の
    真の温度。センサ雑音はアクターが内蔵乱数で加える)
swarm_step()                                  … 受信箱を消費して判断・移動
swarm_x() / swarm_y() -> f64                  … 現在位置(ホストの記録・場計算用)
dyne_msg_buf() -> u32 / dyne_msg_cap() -> u32 … 転送用スクラッチ領域の位置と容量
    (ホストはここへ書き/ここから読む。現行容量4096バイト)
dyne_inbox_write(ptr: u32, len: u32) -> i32   … ホスト→アクター。受信箱バッファへ
    v2メッセージ列(3章の枠)を書き込む。0=全て受理 / 1=一部あふれ(アクター側で
    破棄+計数)/ -1=形式不正(不正な列は受信箱に入れない)
dyne_outbox_read(ptr: u32, cap: u32) -> u32   … アクター→ホスト。送信箱の内容
    (v2メッセージ列)を書き出し、使用バイト数を返す。cap 超過分は破棄+計数
dyne_dropped() -> u64                         … あふれで破棄したメッセージの累計
```

- 既存の熱伝導アクター(heat1d-actor / heat1d-pinn-actor)は**当面このABIに移行しない**
  (ホスト側で吸収)。移行は2次元化(帯ペイロード導入)の際に行う。
- 将来の場アクターの帯ペイロード(2次元界面)にも同じ差込口が使える。
- POSITION / HEARSAY のフィールド定義(3章)は実装の結果、変更なしで確定
  (2026-09-04)。

## 6. 受け入れ基準(第2段の完了条件)

1. **既存全回帰の維持**: dyne-sim の全シナリオが切り出し前とビット一致
   (第1段で証明済み。第2段でも維持)。
2. **決定論**: dyne-swarm-sim 自身の2回実行で全出力(軌跡CSV等)がビット一致。
3. **定性的再現**: 旧 dyne-swarm と同一シナリオ・同一シード指定で、
   gossip が9割以上到達・consensus が罠の丘に吸われる等の性質が再現。
4. **定量帯(2026-09-06 改訂、ユーザー指定)**: 事前に固定したシード50本
   (42 および 1〜49)で新旧両実装を既定シナリオ(complex場、N=60、半径0.15、
   k=6、800ステップ、gossip)で実行し、最適温度域到達率の分布が次の3条件を
   **すべて**満たすこと:
   - (i) 平均の差が ±10ポイント以内
   - (ii) 中央値の差が ±10ポイント以内
   - (iii) 90%以上到達したシード本数の差が 50本中8本以内

   旧実装とのビット一致は求めない(乱数の消費順が構造的に変わるため。
   docs\2026-09-02-swarm-port-options.md §5-3)。本基準は**データ取得の前に
   確定**した(2026-09-06。データを見てから基準を動かす形にしないため)。
   旧基準(シード42と1〜3で93〜98%と同水準)からの改訂の経緯は改訂履歴と
   docs\2026-09-04-swarm-port-report.md §3 を参照。

## 改訂履歴

- 2026-09-02: 初版(ユーザー決定 2026-09-02 の具体化)。
- 2026-09-04: §5 差込口を実装で確定(POSITION/HEARSAY のフィールドは変更なし)。
  §6 基準④に検証結果の注記を追加。
- 2026-09-06: §6 基準④を「シード50本(42と1〜49)の分布比較: 平均±10pt・
  中央値±10pt・90%以上到達本数の差8本以内」へ改訂(ユーザー指示)。
  理由: 旧基準の93〜98%は当たりシード4本による標本の偏りと判明したため
  分布比較へ改訂。基準を先に固定し、その後に50本の実験を行う。

## 7. 第1段の検証結果(2026-09-02、Claude Code環境)

切り出し前後で全12シナリオの stdout・stderr・CSV(計31ファイル)が**ビット一致**
(唯一の差はCSV書き出し先ディレクトリ名のログ行のみ=出力先の指定差)。
シナリオ: 基本16ノード / カオス(loss10%+遅延5±3ms) / pause / partition+loss /
handover / ADD(+残差CSV) / ADD+カオス+handover複合 / CSV4種出力 / ADD+auto-grow /
学習型基本 / 学習型resize / 学習型ADD+auto-grow。
dyne-core・dyne-wire の単体テスト10件パス(乱数の固定ベクトル、イベント順序、
分断判定、v2往復、あふれ破棄+計数)。

ユーザーの手元確認用コマンドは STATUS.md「次の作業」と最終報告に記載。

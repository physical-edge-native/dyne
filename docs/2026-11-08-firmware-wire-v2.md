# 実機ファームの界面の受け渡しをワイヤ形式v2にそろえる(2026-11-08 下書き)

設計書 docs/2026-10-10-order10-design.md の D5(実機のメッセージ形式。決定=案a
「実機もv2にそろえ、v2の外側に通番とCRCを付ける」)の実装メモ。
実機は未接続のため、**実機上での動作確認は未実施**(§5)。

## 1. 何をしたか

| 対象 | 内容 |
|---|---|
| crates/dyne-wire/src/envelope.rs(新規) | UDPの1データグラムの外箱の単一定義(符号化・復号・検査)と単体テスト7件 |
| crates/dyne-wire/src/lib.rs | `pub mod envelope;` の1行のみ追加(既存の定義・挙動は無変更) |
| node-esp32/main/dyne_wire2.h/.c(新規) | 外箱のC実装(ESP-IDF非依存)と起動時自己検査 `dyne_env_selftest()` |
| node-esp32/main/dyne_wire2_golden.h(新規) | 照合用バイト列(88バイト)。Rustのテストと C の自己検査の両方が同じ配列を検査する |
| node-esp32/main/node_v2.c/.h(新規) | メールボックス方式のアクター用の同期の経路(設定注入の解釈・BSP(全ノードが歩調をそろえて1ステップずつ進む方式)) |
| node-esp32/main/main.c | 分岐の追加のみ(下記4か所)。1次元の経路の本体は無変更 |
| node-esp32/main/CMakeLists.txt | ソース2つを追加 |
| node-esp32/host_test/wire2_selftest.c(新規) | C実装を PC で検査する小さな実行ファイルの元 |
| .gitignore | `node-esp32/build-c5/` を末尾に追記 |

main.c の変更点: (1) include 2行、(2) `wasm_up()` の最後で `dyne_v2_bind()` が真なら新経路、
(3) `dyne_on_cfg()` の冒頭で新経路なら `dyne_v2_on_cfg()` へ、(4) start 時に新経路なら
`dyne_v2_run_bsp()`、それ以外は従来の `run_bsp()`。加えて起動時に自己検査の結果を1行表示する。

## 2. 外箱(UDPの1データグラム)

中身は「1つの界面・1つのステップぶんの配送単位」= 枠つき v2 メッセージ列
`[u32 len][msg]...`(終端の len=0 は含めない)。ファームは中身を変換しない。

```text
offset size field
0      4    magic        "DYN2"
4      1    env_version  1
5      1    flags        0(予約。0以外は不正)
6      2    reserved     0
8      4    sender       送信元ノードID
12     4    seq          通番(ステップごとに+1。再送は同じ値)
16     4    iface        界面ID(中身の src と一致)
20     4    payload_len  中身のバイト数 N
24     8    step         ステップ(中身の step と一致)
32     N    payload
32+N   4    crc32        IEEE CRC32(0〜32+N−1。既存の実機形式・dyne-proto と同じ式)
```

- 上限: 1データグラム 1472 バイト(分割されずに届く大きさ)→ 中身は 1436 バイトまで。
  値のみの界面サンプル列なら1辺175点まで1通に入る(設計書D4の見積もり 1辺40点=352バイトは十分収まる)。
- 上限超えの扱い: 送信側は**送らずに誤りで停止**(状態に `"err":"v2: unit oversize"`)。
  受信側は 1473 バイトで受けて上限超えを検出し、読まずに捨てて計数(`bad_env`)。分割送信は未対応。
- 復号の検査順: 長さ → 識別子 → 長さ欄 → CRC → 版・予約欄 → 中身(空でない、枠の整合、
  各メッセージが v2 として正しい、界面で運ぶ種別、外箱の iface/step と中身の一致)。
- iface/step を外箱にも置く理由: 受け手が中身を解読せずに配送先と新旧(取り置き・再送応答)を判定するため。
- 小さな判断(事後明記): 版の欄は外箱独自に1から始めた。sender は u32(dyne_node_id() の int をそのまま)。

## 3. ファームの新経路(node_v2.c)

- **選択**: 積み込んだ wasm が `dyne_outbox_read` と `dyne_inbox_write` を持てば新経路
  (他に `dyne_set_face`・`dyne_init2d`・`dyne_msg_buf`・`dyne_msg_cap`・`dyne_step`・`dyne_len`・`dyne_get` が必須。
  `dyne_cfg`・`dyne_dropped`・`dyne_rejected`・`dyne_missing` は任意)。現行の
  heat1d_actor_small はこれらを持たないので、従来どおり1次元の経路(40バイトの実機形式)で動く。
- 同期の組み立ては既存の `run_bsp` と同じ: 送信控え(直近2ステップ)、先着分の取り置き、
  過去ステップを受けたら控えを再送で応答、返事待ち100msで再送(バックオフ20ms)、終了時ドレイン3秒。
  初期化の順は dyne-sim2d と同じ(`dyne_cfg` → `dyne_set_face`×4 → `dyne_init2d`)。
- 受信メールボックスが受理しない(界面ID・ステップ・点の数の不一致)ときは誤りで停止する
  (シミュレータでは assert で止めている箇所と同じ扱い)。
- メモリ: 界面ごとの送信控え・取り置き・組み立て用の領域(約5.9KB/界面)は**実行開始時に malloc** し、
  終了時に解放する。静的領域に置くと1次元の経路の空きメモリ(WASM線形メモリ用の連続ブロック)を
  減らすため。
- 観測: `node/<idx>/field2d` に区画の状態の CRC32・最小・最大を `field_every` ごとに発行
  (2次元の全点は JSON に収まらないため要約のみ)。完了時の status に `wire:2, crc32, sent, resend,
  replay, bad_env, rejected, missing, dropped`。CRC32 は dyne_get の全点の f64 バイト列で、
  シミュレータの同じ区画と比べればビット一致を確かめられる。

### 設定注入の形(暫定。**ユーザーの確認が必要**)

制御面の規約 docs/2026-09-14-mqtt-control-plane.md には未記載。新経路のときだけ解釈する:

```json
{ "gen":1, "cluster":{"steps":500,"alpha":1.0,"dt":1e-4,"step_period_ms":0},
  "node":{"idx":0},
  "field2d":{"nx":21,"ny":21,"ix0":0,"iy0":0,"h":0.025,
             "faces":[{"face":"e","key":"interface","iface":0},
                      {"face":"w","key":"dirichlet","value":0.0}],
             "cfg":[[0,0.0],[2,0.1]]},
  "ifaces":[{"id":0,"peer":"192.168.137.12:7777"}],
  "mode":{"field_every":50,"rt_every":100} }
```

省略した面はゼロ勾配。界面の面ごとに同じIDの結線がちょうど1つあることを受理時に検査する。

## 4. 検証したこと

| 項目 | 結果 |
|---|---|
| `cargo test -p dyne-wire` | 既存8件+外箱7件、全件 ok |
| 照合用バイト列の Rust 側検査 | テスト `golden_matches_c_header` が C のヘッダを読み、Rust の符号化結果と一致 |
| 照合用バイト列の C 側検査(PC) | `wire2_selftest.exe`(.venv の ziglang の C コンパイラでビルド)が PASS |
| CRC の独立確認 | Python の zlib.crc32 で同じ 0x4C13C10A。標準の確認値 crc32("123456789")=0xCBF43926 も Rust・C とも一致 |
| 破損の検出 | 88バイトのどの1ビットを反転しても受理しない(Rust・C とも全704通り) |
| scripts\check-sim2d.ps1 / check-baseline12.ps1 | §6 の結果の欄を参照 |
| ESP32 ビルド | 警告なしで成功(サイズは §6) |
| ESP32-C5 ビルド | 別ビルド build-c5/(sdkconfig も build-c5/ 内。既定の sdkconfig は無変更を cmp で確認) |

PC での C 実装の検査手順(リポジトリのルートで):

```
.venv\Scripts\python.exe -m ziglang cc -std=c11 -Wall -Wextra -O2 -Inode-esp32\main ^
    node-esp32\host_test\wire2_selftest.c node-esp32\main\dyne_wire2.c -o %TEMP%\wire2_selftest.exe
%TEMP%\wire2_selftest.exe
```

## 5. 検証していないこと・未対応

- **実機での動作は未確認**(未接続)。起動時の自己検査の表示、新経路での2台の同期、無線上での再送・破損検出はすべて未確認。
- 現行の1次元の経路は、コード上は分岐を足しただけで本体は無変更。ただし実機で実験1の構成を再実行しての確認はしていない。
- **新経路を実際に通す wasm がまだ無い**: ファームが積み込む wasm は heat1d_actor_small の1本に固定
  (CMakeLists の EMBED_FILES)。heat2d-actor・sdf2d-grid-actor はフル版で線形メモリが大きく
  (heat2d は状態だけで約640KB)、ESP32(PSRAMなし)には載らない。実機用の縮小版と、積み込む wasm の
  切り替え方法(menuconfig の選択など)は次の作業(**選択肢をユーザーに提示して決める**)。
- 測距フレームの受け渡し(センサのあるノードへの入力)は未対応。格子型アクターを実機で動かすには必要。
- 1436バイトを超える配送単位の分割送信は未対応(光線束が多いと超えうる)。
- 1ステップの実時間の統計は平均のみ(1次元の経路の min/max・待ち時間は未実装)。

## 6. 回帰確認とサイズ

回帰確認(2026-11-08、Windows):

- `scripts\check-sim2d.ps1`: dyne-wire テスト15件 ok、2次元の熱のビット一致9シナリオ全 PASS、決定論(fc 相違なし)、
  2次精度(6.975e-4 / 1.741e-4 / 4.350e-5)、格子型アクター8シナリオ全 PASS、対照実験は期待どおり FAIL。
  注: 実行時点で crates/dyne-sim2d・heat2d-actor には並行作業中の別の変更があった(本作業では触れていない)。
- `scripts\check-baseline12.ps1`: ALL 12 SCENARIOS MATCH(1次元の既存回帰、a435bf6 とバイト一致)。

ファームのサイズ(`idf.py size`。「変更前」は HEAD の node-esp32 を一時領域に取り出して同じ sdkconfig でビルド):

| | 変更前 ESP32 | 変更後 ESP32 | 変更後 ESP32-C5 |
|---|---|---|---|
| アプリ(.bin) | 1,129,744 B | 0x1165F0 = 1,140,208 B(区画の26%空き) | 0x14DA00 = 1,366,528 B(区画の11%空き) |
| Flash Code | 851,916 B | 860,948 B(+9.0KB) | — |
| DRAM 計 / .bss | 63,624 / 40,872 B | 63,744 / 40,992 B(+120 B) | HP SRAM 165,890 B / .bss 47,312 B |

ESP32 の静的 RAM の増分は 120 バイト(関数参照と計数のみ。設定と送受信の領域は新経路が有効なときだけヒープに確保)。
C5 は ESP-IDF の既定の設定に sdkconfig.defaults を重ねたもの(調整なし)。区画の空きが11%と少ない。

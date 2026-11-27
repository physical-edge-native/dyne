# 素の ESP32 のインタプリタで間に合うか: 形と波動方程式のアクターの速度計測(2026-11-11)

目的: マイコン15台を買う前に、**素の ESP32(Xtensa、WAMR インタプリタ、AOT なし)** で2週間の実験
(特に実験6「箱を動かして形が追従する」と実験10「波動方程式」)に足りる速さが出るかを、手持ちの
ESPr Developer 32 Type-C(ESP32-WROOM-32E、PSRAM なし)1台で今日測って決める。
候補は ESP32-DevKitC-VE(ESP32-WROVER-E、PSRAM 8MB、技適あり、Xtensa)と XIAO ESP32C6(RISC-V、
AOT 未検証、Amazon の掲載に技適の表示なし)。

用語: WAMR = WebAssembly Micro Runtime(マイコン向けの WebAssembly 実行環境)。classic = WAMR の素朴な
インタプリタ(node-esp32 と同じ)、fast = 命令を前処理する高速インタプリタ、AOT = PC で前もって
機械語に変換する方式。線形メモリ = WebAssembly のアクターが使うメモリ領域(1ページ = 64KB)。

## 1. 作ったもの

### 1.1 形の格子型アクターの縮小版(2週間計画の G4)

crates/sdf2d-grid-actor に `--features small` を足した(wave2d・heat1d と同じ管理方法: 同じクレート+
機能フラグ)。**線形メモリ1ページ(64KB)** に入る。

| 項目 | フル版(シミュレータ用) | 縮小版(実機用) |
|---|---|---|
| 区画 | 1辺196点、枠つき40,000点 | 1辺40点、枠つき1,210点、枠なし1,080点 |
| 想定 | — | 6畳(3.6m×2.7m)を刻み2.5cm(145×109点)で 4×4 分割(最大 37×28点)または 5×3 分割(29×37点) |
| センサの数(ID) | 64 | 64(サーボ4方向×半区画ずらし8位置×7台 = 56 ID が入る) |
| 1ステップに受け取る光線 | 8,192 | 64(部屋全体で 8台×8区画 = 64本/ステップを想定) |
| 統合待ちの光線 | 65,536 | 304(シミュレータの最大 234本 に余裕) |
| 面ごとの送信待ちの光線 | 4,096 | 24 |
| スタック | 32KB | 6KB(8KB では1ページに入らなかった) |
| 線形メモリ | 136ページ | **1ページ** |
| wasm の大きさ | 33,537バイト | 34,499バイト |

計算結果を変えない省メモリの書き方を3つ使った(縮小版のみ。フル版の命令列は変えていない):

1. 距離の反復(アイコナール方程式の Jacobi 反復)で φ の2枚目を持たない。前の行と今の行の古い値だけを
   写して、同じ式・同じ入力値で上書きする。
2. 統合待ちの光線に期限を持たせず、取得ステップ + 統合の遅れ L で求める(取り込み時に入れていた値と同じ)。
3. 期限の来た光線を別の列に取り出さず、統合待ちの列を安定に並べ替えて先頭から統合する。統合待ちには
   取得ステップが「今 − L」以上のものしか無いので、期限の来た光線は鍵が最小で先頭に集まり、同じ鍵の中の
   順も取り込みの順のまま = フル版と同じ順に統合する。

計測用に `dyne_bench_sweep(n)`(距離の反復だけを n 回)を縮小版にだけ足した(照合の後にだけ呼ぶ)。

**置き場があふれると結果がフル版とずれる**(あふれは dyne_overflow で数える)。全部のセンサが毎ステップ
同時に測る場面(シミュレータの比較用の場面 room6.txt = 54台、study/pan4.txt = 28台)はあふれる。
実機ではサーボで振るので1ステップに測るのは7〜8台で、これは想定内。

### 1.2 ビット一致の確認(シミュレータ)

dyne-sim2d に、単一ノード解だけを別の wasm で解く `--single-wasm` を足し(縮小版は全域を1区画に持てない
ため)、**縮小版の分割解とフル版の単一ノード解** を比べた。刻み 2.5cm、雑音 8mm、推奨の忘れる仕組み
(食い違いの閾値 0.1m + 減衰 0.9 + 忘却 0.3)、15Hz。

| 場面 | 分割 | ステップ | 結果 | CRC32(φ / d / w) |
|---|---|---|---|---|
| room6_move.txt(箱の瞬間移動、8台) | 4×4 | 600 | PASS (bit-exact) | c5323035 / 30b01fdb / b50d812f(フル版どうしの値と同じ) |
| room6_move.txt | 5×3 | 600 | PASS (bit-exact) | 同上 |
| room6_carry.txt(箱を運ぶ)、損失10%・遅延5ms±10ms | 4×4 | 600 | PASS (bit-exact) | c79fea6b / fd6049b7 / 2ff16c62 |
| room6.txt(54台が同時に測る) | 4×4 | 400 | FAIL(あふれ 167,728) | 想定外の負荷。上の注意 |
| study/pan4.txt(28台が同時に測る) | 4×4 | 200 | FAIL(あふれ 13,217) | 同上 |

wave2d の縮小版(既存)も、81×81点・4×4分割・200ステップで PASS (bit-exact)(全域 CRC32 791fcf5b)。

フル版の出力が変わっていないことの確認:

- scripts\check-sim2d.ps1: ビット一致 9/9 PASS、決定論(fc 相違なし)、2次精度(6.975e-4 / 1.741e-4 /
  4.350e-5)、格子型アクター 8/8 PASS・決定論、対照実験 FAIL(期待どおり)。
- scripts\check-baseline12.ps1: 「ALL 12 SCENARIOS MATCH (bit-exact vs a435bf6)」(1次元の回帰)。
- 変更前のソースを復元してビルドしたフル版と変更後のフル版で、格子型アクターの9シナリオ(check-sim2d の
  [5][6] と room6_move 2条件)の**標準出力の全体がバイト一致**(wasm のバイト列は関数の切り出しで違う)。

### 1.3 記録と再生(dyne-sim2d → bench-c5)

実機の1ノードの仕事をそのまま測るため、dyne-sim2d に `--dump-node N --dump-file X.rec` を足した。
ノード N のアクターへの呼び出し(設定・面・初期化・受信メールボックス・送信メールボックス・ステップ)を
順に記録し、最後に区画の φ・d・w の CRC32 を付ける。計測ファーム bench-c5 はこれを ESP32 上で再生し、
**1ステップの時間** と **最後の CRC の一致** を報告する(通信の待ちだけを除いた、実機の1ノードと同じ処理)。

| 記録 | 中身 | 大きさ |
|---|---|---|
| shape_n13.rec | 形、room6_move、4×4 のノード13(区画 36×27点、**センサ2台**、界面3つ)、200ステップ | 349KB |
| shape_n5.rec | 形、同じ条件のノード5(36×27点、界面4つ、**光線の中継が最も多い**。統合待ちの最大 234本) | 624KB |
| wave_n5.rec | 波動方程式、81×81点を 4×4、ノード5(20×20点、界面4つ)、Δt = 0.00625、ガウス型の山、200ステップ | 165KB |

記録の合計が約1.1MB なので、bench-c5 の ESP32 向けビルドだけアプリの領域を 3MB にした
(bench-c5\partitions_esp32.csv、4MB フラッシュ)。ファームの大きさは 1.52MB(classic)。

### 1.4 bench-c5 の追加(既存の計測はそのまま残した)

- main\bench.c: `bench_replay`(上の再生)。起動後、形2つ・波1つを先に測り、続けて従来の mlp・heat1d を
  測る(Kconfig の BENCH_RUN_LEGACY で外せる)。
- 出力行:
  - `#BENCH bench=replay … item=shape_n13 … total_ms_min/mean/max …(1ステップ = 送信 + 受信 + dyne_step の合計)
    step_ms_*(dyne_step だけ) inbox_ms_*(受信メールボックス1回) max_pending heap_used min_free_int
    crc=… expect=… match=1`
  - `#BENCH bench=sweep … sweep_ms=…`(距離の反復1回の時間。**1ステップ − これ = 光線の処理**
    (取り込み・中継の判定・減衰・統合)の時間)
- tools\interp_bench.py: `make`(縮小版のビルド・記録の作成。縮小版の分割解がフル版とビット一致しなければ
  止まる)と `verdict`(貼った #BENCH 行から判定の表を出す)。

## 2. ユーザーの手順(今日、ESPr Developer 32 Type-C 1台)

PowerShell で、リポジトリ直下(`C:\Users\koyab\Documents\dyne`)から。ビルドは済んでいる
(bench-c5\build-esp32-classic\、bench-c5\build-esp32-fast\)。

1. ESPr を USB でつなぎ、ポート名を調べる:
   ```powershell
   Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match '\(COM\d+\)' } | Select-Object Name
   ```
   つなぐ前後で増えた `…(COMn)` がその基板(USB-シリアル変換の名前が出る)。以下 `COM3` を置き換える。
2. classic(node-esp32 と同じ方式)を書き込んで測る:
   ```powershell
   powershell -ExecutionPolicy Bypass -File bench-c5\build.ps1 -Target esp32 -Mode classic -Flash COM3
   ```
   - 書き込み後にモニタが開く。`#BENCH bench=done mode=classic` が出たら完了(目安 1〜3分。形の再生が
     1つ数十秒かかりうる)。Ctrl+] でモニタを閉じる。
   - 書き込みが始まらないときは、基板の BOOT(IO0)ボタンを押したまま、もう一度実行する。
3. fast も同じく:
   ```powershell
   powershell -ExecutionPolicy Bypass -File bench-c5\build.ps1 -Target esp32 -Mode fast -Flash COM3
   ```
4. `#BENCH` で始まる行を**すべて**(両方の方式)コピーして会話に貼る。手元で判定するなら、
   メモ帳で bench-c5\esp32-log.txt に貼って保存し:
   ```powershell
   .venv\Scripts\python.exe bench-c5\tools\interp_bench.py verdict bench-c5\esp32-log.txt
   ```
   (`--comm-ms 20` のように1ステップの通信の時間の仮定を変えられる。既定 10ms)

注意: 計測中は Wi-Fi を使わない(実運用では Wi-Fi の割り込みが CPU を少し取る)。もう一度測るときは
基板の EN(リセット)ボタンを押す。

## 3. 出力の読み方と判定の基準

- **正しさ**: `bench=replay` の `match=1` がすべての行で出ること(実機の計算がシミュレータ(wasmi)と
  ビット一致)。0 なら時間の値も参考扱いにして、行を貼ってもらって調べる。
- **実験6(形の追従)**: シミュレータの追従 0.67秒 = 10ステップ(うち統合の遅れ L = 7、4×4 分割)。
  1ステップの周期 = max(最も遅いノードの1ステップ, 66.7ms(15Hz)) + 通信の時間。
  追従の見込み = 10 × 周期。**2秒以内なら合格**(dynamic-shape の「2秒以内に追従」の目安)。
  逆算すると、通信 10ms の仮定で **最も遅いノードの1ステップ ≤ 約190ms** なら合格。
  15Hz に追いつく(1フレームも捨てない)には ≤ 約57ms。
  計算が 66.7ms より遅いとセンサのフレームが間引かれるが、追従のステップ数は変わらない(箱が見えてから
  約3フレーム + L)ので、時間は計算の周期で伸びる。
- **波動方程式(実験10)**: 400ステップの所要 = 400 ×(1ステップ + 通信)。
- **PSRAM が要るか**: 縮小版の線形メモリは1ページなので、Wi-Fi 稼働中の最大の連続空き(約108KB)に入る。
  `heap_used`(アクター1つでの内部 RAM の消費)+ Wi-Fi 等の約70KB(推定)が、計測前の空きに収まれば
  PSRAM は不要。PSRAM に線形メモリを置くとキャッシュを外れて遅くなるので、速さの面では WROVER の利点はない
  (メモリが足りない場合だけ効く)。fast は前処理した命令の分だけ多くの RAM を使うので、fast の `heap_used`
  を特に見る。

## 4. 見込み(推定。実測で置き換える)

**ここは文献と一般的な知識からの推定で、この環境では測っていない。**

| 比較 | 推定 | 根拠・注意 |
|---|---|---|
| fast ÷ classic | 1.5〜2.5倍速い | WAMR の公表(高速インタプリタは古典的インタプリタより大幅に速い、最大で2倍強)。メモリは増える |
| AOT ÷ classic(整数中心の処理) | 10〜50倍 | WAMR・wasm3 などの公表値の一般的な幅(AOT は機械語に近い速さ) |
| **AOT ÷ classic(このアクター)** | **3〜8倍(推定)** | 形・波のアクターは f64(倍精度)の演算が中心。ESP32(Xtensa LX6 は単精度の演算器のみ)も ESP32-C6(rv32imac、演算器なし)も **f64 はソフトウェアの関数呼び出し**(1演算に数十〜百数十サイクル)で、これは AOT でもインタプリタでも同じ。インタプリタの命令の取り出しの手間(1命令10〜30サイクル程度)を除けるだけなので、整数中心の処理ほどは速くならない |
| C6 の classic ÷ ESP32 の classic | 0.6〜0.8倍 | 周波数 160MHz 対 240MHz(どちらも f64 はソフトウェア) |
| C6 の AOT ÷ ESP32 の classic | 約2〜5倍 | 上の2つの積。**C6 での AOT は未検証**(wamrc 2.4.0 の入手、RISC-V で実行可能な RAM を得るためのメモリ保護の設定(C5 では IRAM/DRAM の分割を外した)、f64 のソフトウェアの関数との結合を確かめる必要がある) |

判断の材料:

- ESP32 の classic で実験6の基準(1ステップ ≤ 約190ms)を満たせば、**DevKitC-VE(技適が確実、
  node-esp32 がそのまま動く)で足りる**。AOT の速さは不要。
- classic で足りず fast で足りるなら、fast のメモリ(`heap_used` + Wi-Fi 約70KB)が内部 RAM に入るかで
  決める。PSRAM(WROVER)は線形メモリが1ページなら不要。
- fast でも足りないなら、(a) 分割を細かくする(区画が小さくなる。ただし統合の遅れ L が伸びる)、
  (b) 光線の処理を軽くする(標本化の刻み、光線の長さの上限)、(c) RISC-V+AOT(C6)に賭ける、の選択になる。
  (c) は AOT が未検証なのと技適の表示がないことが危険要因。

## 5. 分からないこと・制約

- 計測は1ノード分の処理で、Wi-Fi・通信の待ちを含まない。通信の時間は `--comm-ms` の仮定(既定 10ms)。
- 縮小版は「1ステップに部屋全体で約64本の光線」までの想定。全センサが同時に測る場面はあふれる(§1.1)。
  4×4 より粗い分割(1区画が 40×28点を超える)は入らない。
- 統合待ちの上限 304本は room6_move(8台)の最大 234本からの値。センサが多い・光線が長い配置では
  あふれうる(あふれれば dyne_overflow が増え、CRC がずれる)。
- WROVER(DevKitC-VE)向けの設定は用意していない。PSRAM を使わないなら同じ ESP32 向けのビルドで動く
  (WROVER-E の PSRAM を有効にするなら `CONFIG_SPIRAM=y` を足すが、今回の判断には不要)。
- 形の記録は 200ステップ(箱の移動の前)。忘れる仕組みは働いているが、箱が動いた直後の食い違いが多い
  ステップは含まない(1ステップの時間への影響は小さいと見込む。未確認)。
- (2026-11-26 更新、GitHub issue #156)bench-c5 の mlp-actor の素材を今の版に写し直した。作り直した mlp-actor でも
  期待 CRC は5件とも以前と同じだった(wasm のバイト列は変わったが、出力は変わらない)。期待値は bench-c5/expected.txt に
  固定し、CI で確かめている。実機での測り直しはまだ。

## 6. 変更したファイル

| ファイル | 変更 |
|---|---|
| crates/sdf2d-grid-actor/Cargo.toml | `[features] small` |
| crates/sdf2d-grid-actor/src/lib.rs | 縮小版の容量・省メモリの書き方3つ・`dyne_bench_sweep`(縮小版のみ)。フル版は統合の内側を関数に切り出しただけ(出力はバイト一致) |
| crates/dyne-sim2d/src/main.rs | `--dump-node` / `--dump-file`(呼び出しの記録)、`--single-wasm`(単一ノード解だけ別の wasm)。追加のみ、既存の出力は不変 |
| bench-c5/main/bench.c | 2次元のアクターの再生と計測(追加) |
| bench-c5/main/CMakeLists.txt、Kconfig.projbuild | 素材の追加、BENCH_RUN_LEGACY |
| bench-c5/sdkconfig.defaults.esp32、partitions_esp32.csv(新規) | ESP32 向けだけアプリの領域を 3MB に |
| bench-c5/tools/interp_bench.py(新規) | 素材の作成と判定 |
| bench-c5/assets/(sdf2d_grid_actor_small.wasm、wave2d_actor_small.wasm、*.rec) | 素材(.rec は .gitignore。make で再生成) |
| scripts/build-actor-wasm.ps1 | [5] sdf2d 縮小版のビルドを追加 |
| .gitignore | crates/sdf2d-grid-actor/target-small/、bench-c5/assets/*.rec |

## 7. 小さな実装判断(事後の明記)

- 縮小版のスタックを 6KB にした(8KB では統合待ち 304本と両立せず2ページになった)。スタックがあふれると
  静的領域が壊れて CRC がずれるので、シミュレータのビット一致で間接的に確かめている。
- 記録の再生は「送信メールボックス → 受信メールボックス(記録の順)→ ステップ」をシミュレータと同じ順で行う。
- 計測ファームの容量のため、ESP32 向けのビルドだけパーティションテーブルを独自にした(C5 向けは従来どおり)。

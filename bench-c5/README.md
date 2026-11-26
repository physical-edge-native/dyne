# bench-c5 — WASMアクターの速度・メモリ計測ファーム (ESP32-C5 向け、ESP32 でも動く)

WebAssembly (どの計算機でも同じ結果になる中間形式のプログラム) で書いたアクター
(計算の部品) を、マイコン向けの実行環境 WAMR (WebAssembly Micro Runtime) で動かし、
次を測るための単体のファームです。node-esp32 (現役の本番ファーム) とは独立しています。

- (a) mlp-actor 縮小版: 学習済みの多層パーセプトロン (全結合層を重ねたNN) を
  .dynm (計算グラフ+重みの2進ファイル) から読み込んで推論する。
  推論1回あたりの時間、ヒープ (自由に使えるRAM) の消費、線形メモリ
  (WebAssembly が使うメモリ領域、1ページ=64KB) のページ数、出力の CRC32 を報告する。
- (b) heat1d 縮小版: 1次元熱伝導の格子計算。1秒あたりのステップ数と、
  温度配列の CRC32 (node-esp32 が報告する crc と同じ定義) を報告する。

実行方式は3通りを、ビルドを分けて比べられます。

| 方式 | 内容 |
|---|---|
| classic | WAMR の素朴なインタプリタ (node-esp32 と同じ。既定) |
| fast | WAMR の高速インタプリタ (命令を前処理する。メモリをやや多く使う) |
| aot | wamrc で事前にマイコンの機械語へ変換した .aot を実行 (既定では無効。下の「AOT」) |

CRC は PC 上 (wasmtime という別の WebAssembly 実行環境) で同じ入力・同じ手順で
計算した値とビット一致するはずです。一致すれば「マイコン上の推論・格子計算が
PC と1ビットも違わない」ことの確認になります。

## 配線

なし (XIAO ESP32C5 を USB-C で PC につなぐだけ)。

## 手順

PowerShell で、リポジトリ直下 (`C:\Users\koyab\Documents\dyne`) から実行します。

### 1. 素材を作る (済み。mlp-actor を作り直したときだけ再実行)

```powershell
cd crates\mlp-actor
cargo build --release --target wasm32-unknown-unknown --features small --target-dir target\small
cd ..\..
.venv\Scripts\python.exe bench-c5\tools\bench_ref.py make
.venv\Scripts\python.exe bench-c5\tools\bench_ref.py expect
```

`make` は bench-c5\assets\ に wasm 2つ (mlp-actor 縮小版・heat1d 縮小版の写し) と
試験モデル3つを置きます。

| ファイル | 中身 |
|---|---|
| model_a_f64.dynm | 2→16→16→1、tanh、重み f64、パラメータ 337 (scripts\onnx_check.py のモデルAと同じ) |
| model_b_f64.dynm | 4→32→32→4、tanh、重み f64、パラメータ 1348 |
| model_b_f32.dynm | 上と同じモデルの重みを f32 に丸めたもの |

`expect` は実機が出すべき CRC を `#EXPECT` 行で表示します。
その出力は `bench-c5/expected.txt` に固定してあり、CI (scripts/ci-check.sh の21番目) が、置いてある素材と、
作り直した mlp-actor の両方で同じ値になることを毎回確かめます (2026-11-26、GitHub issue #156)。
mlp-actor を作り直したら、上の `make` の代わりに wasm の写し (1行目の cargo build の後の
`crates/mlp-actor/target/small/.../mlp_actor.wasm` → `assets/mlp_actor_small.wasm`) だけでよく、
期待値が変わらなければ `expected.txt` もそのままです (`make` は .dynm も作り直すので、NumPy の版によって重みが変わりうる)。

### 2. ビルドと書き込み

```powershell
powershell -ExecutionPolicy Bypass -File bench-c5\build.ps1 -Target esp32c5 -Mode classic -Flash COM5
```

- `COM5` はデバイスマネージャで確認したポート名に置き換えてください。
- `-Flash` を付けないとビルドだけ行います。ビルドディレクトリは
  `bench-c5\build-<ターゲット>-<方式>\` に分かれます。
- 書き込み後にモニタが開き、起動から十数秒で計測が終わります (`#BENCH bench=done` が出たら完了)。
  モニタは Ctrl+] で閉じます。もう一度測るときは基板のリセットボタンを押します。
- 書き込みが始まらないときは、BOOT ボタンを押しながら USB を挿し直してから再実行してください。
- 続けて `-Mode fast` でも同様に書き込んで測ってください。

### 3. 結果を貼る

モニタに出た `#BENCH` で始まる行を**すべて**コピーして会話に貼ってください
(方式ごとに約12行)。ファイルに保存した場合は PC 側で照合できます。

```powershell
.venv\Scripts\python.exe bench-c5\tools\bench_ref.py compare 保存したログ.txt
```

全項目がビット一致なら `PASS (bit-exact)` と表示されます。

## 出力行の読み方

| 行 | 主な項目 |
|---|---|
| `bench=start` | ターゲット、CPU周波数、ESP-IDF と WAMR の版、PSRAM の総量 (0 なら PSRAM 未検出) |
| `bench=heap` | 各時点の内部RAMの空き (`free_int`)、最大の連続空き (`largest_int`)、最小空き、PSRAM の空き |
| `bench=actor` | アクターの読み込み: 大きさ、形式 (bytecode/aot)、読み込み・実体化の時間、それぞれのヒープ消費、線形メモリのページ数、線形メモリが PSRAM 上か (`linmem_psram`) |
| `bench=mlp` | `us_per_inf_batch` (64組まとめて推論したときの1組あたりµs)、`us_per_inf_single` (1組ずつ呼んだときの1回あたりµs、呼び出しの手間を含む)、`heap_used` (アクター1つを動かす間に使ったヒープの合計)、`crc` |
| `bench=heat1d` | `steps_per_s`、`ns_per_point` (1格子点1ステップあたり)、`crc` |

計測条件 (入力の作り方など) は tools\bench_ref.py の冒頭に書いてあり、
bench.c と一致させています。片方を変えたら両方を直してください。

## AOT (事前コンパイル)

AOT は WebAssembly を PC 上で前もって対象マイコンの機械語に変換しておく方式で、
インタプリタより大幅に速くなる見込みです。2026-11-08 時点で、この PC には
変換器 wamrc がないため**既定では無効**にしてあります (Kconfig の `BENCH_USE_AOT`、
既定 n)。

### wamrc の入手

WAMR は版ごとに .aot の形式が変わるので、**実機側と同じ 2.4.0** の wamrc を使います
(bench-c5 は ESP-IDF のコンポーネント espressif/wasm-micro-runtime 2.4.0 を使用)。

- 方法1 (手軽): WAMR の GitHub のリリース (https://github.com/bytecodealliance/wasm-micro-runtime/releases)
  の WAMR-2.4.0 から Windows 用の wamrc のアーカイブを取得して展開する
  (アーカイブ名は `wamrc-2.4.0-x86_64-windows-…zip` のような形。リリースの一覧で確認してください)。
- 方法2: ソースからビルドする (LLVM のビルドを伴うので数十分〜1時間以上かかる)。
  WAMR の `wamr-compiler/` の README の手順に従う。

### .aot を作ってビルド

```powershell
powershell -ExecutionPolicy Bypass -File bench-c5\tools\make_aot.ps1 -Target esp32c5 -Wamrc C:\どこか\wamrc.exe
powershell -ExecutionPolicy Bypass -File bench-c5\build.ps1 -Target esp32c5 -Mode aot -Flash COM5
```

- ESP32-C5 は RISC-V 32ビットで、**浮動小数点演算器を持たない** (rv32imac)。
  そのため `--target=riscv32 --target-abi=ilp32 --cpu=generic-rv32 --cpu-features=+m,+a,+c`
  で変換する (f64 の演算はソフトウェアで行われる)。
- AOT の機械語は実行可能な RAM に置く必要がある。RISC-V 系の ESP32 では既定の
  メモリ保護 (IRAM/DRAM の分割) がこれを妨げるため、aot 方式のビルドだけ
  `CONFIG_ESP_SYSTEM_PMP_IDRAM_SPLIT=n` にしている (sdkconfig.aot)。
- .aot ファイルは git 管理外 (リポジトリの .gitignore の `*.aot`)。wamrc で再生成できる。

## ESP32 (素の ESP32) でも測る場合

```powershell
powershell -ExecutionPolicy Bypass -File bench-c5\build.ps1 -Target esp32 -Mode classic -Flash COM3
```

node-esp32 と同じ基板でも同じ計測ができ、C5 との比較に使えます。

## 構成

| 経路 | 役割 |
|---|---|
| main\bench.c | 計測本体 |
| main\Kconfig.projbuild | 繰り返し回数・実行スタック・AOT の切り替え |
| sdkconfig.defaults (+ .esp32c5 / .esp32) | 共通設定とターゲット別の追加 (C5 は PSRAM 有効、malloc は内部RAMのまま) |
| sdkconfig.fast / sdkconfig.aot | 方式ごとの差分 (build.ps1 が重ねる) |
| assets\ | 埋め込む wasm・.dynm (・.aot) |
| tools\bench_ref.py | 素材の作成 (make)、期待値 (expect)、実機ログとの照合 (compare) |
| tools\make_aot.ps1 | wamrc で .aot を作る |
| build.ps1 | ビルド・書き込み |

## 2次元のアクターの再生 (素の ESP32 で間に合うかの判定、2026-11-11)

形の格子型アクターの縮小版と波動方程式のアクターの縮小版について、dyne-sim2d が記録した1ノード分の
呼び出し (assets\*.rec) を再生し、1ステップの時間と CRC の一致を測る。起動直後に mlp・heat1d より先に行う。
詳細・判定の基準は docs\2026-11-11-esp32-interp-bench.md。

```powershell
.venv\Scripts\python.exe bench-c5\tools\interp_bench.py make        # 素材 (縮小版 wasm・記録) を作る
powershell -ExecutionPolicy Bypass -File bench-c5\build.ps1 -Target esp32 -Mode classic -Flash COM3
powershell -ExecutionPolicy Bypass -File bench-c5\build.ps1 -Target esp32 -Mode fast -Flash COM3
.venv\Scripts\python.exe bench-c5\tools\interp_bench.py verdict 保存したログ.txt
```

ESP32 向けのビルドは記録 (約1.1MB) を埋め込むため、アプリの領域を 3MB にしている (partitions_esp32.csv)。

# Dyne M0 — PC上2プロセスの縦切りPoC

物理モデルを空間方向に分割し、マイコン群だけで協調推論するプラットフォーム
「Dyne」のマイルストーン0。1次元熱伝導方程式を2つのWASMアクターに分割し、
PC上の2プロセスがUDP越しのBoundary Sync Protocol(ステップ同期)で協調して
解くことを検証する。

**検証結果(2026-08-05): 合格。** 2ノード分割解が単一ノード解と全65格子点で
ビット完全一致(最大差 0.0)。解析解との誤差は L2=5.6e-5 / L∞=8.0e-5
(空間2次精度の理論値どおり)。詳細は `docs/2026-08-05-m0-report.md`。

## 構成

```
dyne/
├── Cargo.toml                  ワークスペース定義 (dyne-host, dyne-proto)
├── run_m0.sh                   一括検証スクリプト (Linux/macOS)
├── run_m0.ps1                  一括検証スクリプト (Windows PowerShell)
├── scripts/compare_m0.py       ビット一致検証 (両スクリプトから使用可)
├── crates/
│   ├── heat1d-actor/           WASMアクター (ワークスペース外・wasm32向けに別ビルド)
│   │   └── src/lib.rs          1D熱伝導ソルバ。#![no_std]・ヒープなし・純計算体
│   ├── dyne-host/              ホストランタイム (ネイティブ)
│   │   └── src/main.rs         wasmiでアクターを実行し、UDP境界同期を注入する
│   └── dyne-proto/             Boundary Sync Protocol ワイヤ形式
│       └── src/lib.rs          BoundaryMsg 40バイト固定長 + CRC32
└── docs/
    ├── 2026-08-05-m0-report.md          M0完了レポート (知見3件を含む)
    └── 2026-08-04-dyne-spec-review.md   仕様具体化・ロードマップ文書
```

役割分担は仕様書A-3のとおり:

- **アクター** (heat1d-actor) は純粋な計算体。通信・時刻・同期を一切知らない。
  DyneSolverトレイトをC ABIエクスポート (`dyne_init` / `dyne_step` /
  `dyne_boundary_value` / `dyne_set_neighbor` など) に写像したもので、
  同一バイナリが「単一ノード全域」でも「分割サブドメイン」でも動く。
- **ホスト** (dyne-host) がランタイムレイヤー役。WASMをロードし、境界同期
  (P2P/UDP・BSPループ・再送/応答・終了時ドレイン) を注入する。
- **プロトコル** (dyne-proto) は1境界面×1論理ステップ=40バイト固定長。
  壁時計を使わず論理ステップのみで同期する。

## 必要なもの

- Rust 1.91前後 (それ以前でも概ね可) + `wasm32-unknown-unknown` ターゲット
- Python 3 (検証スクリプトの突き合わせに使用)
- OSはWindows/Linux/macOSのいずれでもよい (UDPループバックが使えればよい)。
  **WindowsでもWSL2は不要**。ホストは標準ライブラリのUDPソケットのみを使う
  純Rustプログラムで、rustupのWindows版がwasm32ターゲットに標準対応している

### rustupを使う場合 (推奨)

```sh
rustup target add wasm32-unknown-unknown
```

**注意**: `crates/heat1d-actor/.cargo/config.toml` は Ubuntu apt版Rust向けに
リンカを `wasm-ld` (lldパッケージ) へ差し替える設定。rustup環境では不要なので
削除するか、`lld` をインストールしておくこと (rustupなら標準の `rust-lld` で足りる)。

### Ubuntu apt版Rustを使う場合 (rustupが使えない環境)

apt版はwasm32用標準ライブラリを同梱しないため、`rust-src` から自前ビルドする。

```sh
apt install rustc-1.91 cargo-1.91 rust-1.91-src lld
export PATH=/usr/lib/rust-1.91/bin:$PATH
# sysrootにソースをリンク (一度だけ)
SYSROOT=$(rustc --print sysroot)
mkdir -p $SYSROOT/lib/rustlib/src
ln -sfn /usr/src/rustc-1.91.1 $SYSROOT/lib/rustlib/src/rust
```

## ビルド

```sh
# 1) アクターをwasm32向けにビルド
cd crates/heat1d-actor

# rustup環境:
cargo build --release --target wasm32-unknown-unknown

# apt版Rust環境 (build-stdでcoreを自前ビルド):
RUSTC_BOOTSTRAP=1 cargo build --release --target wasm32-unknown-unknown \
  -Z build-std=core -Z build-std-features=compiler-builtins-mem

# 2) ホストとプロトコルをビルド
cd ../..
cargo build --release

# 3) プロトコルの単体テスト
cargo test -p dyne-proto --release
```

成果物:

- `crates/heat1d-actor/target/wasm32-unknown-unknown/release/heat1d_actor.wasm` (約2.2KB)
- `target/release/dyne-host`

## 実行

### 一括検証 (推奨)

```sh
./run_m0.sh
```

単一ノード参照解 → 2ノード分割協調推論 → ビット一致検証 の順に実行し、
最後に `M0 検証合格` と表示されれば成功。出力先は環境変数 `OUT` で変更可
(既定 `/tmp`)。

Windows (ネイティブ) では PowerShell 版を使う:

```powershell
.\run_m0.ps1
```

Windowsでの注意:
- rustup環境なら `crates\heat1d-actor\.cargo\config.toml` を削除してからビルドする
  (Ubuntu apt版Rust向けのリンカ差し替え設定のため。rustup標準の `rust-lld` で足りる)。
- Pythonは `python` または `py` コマンドで呼べればよい (スクリプトが自動検出)。
- 通信は127.0.0.1のループバックのみなので、通常はWindowsファイアウォールの
  確認ダイアログは出ない。もし出た場合も許可すれば以後は表示されない。
- Git BashやMSYS2があれば `./run_m0.sh` も動くが、必須ではない。

### 手動実行

問題設定: 全域 [0,1] を65点 (h=1/64)、初期条件 u(x,0)=sin(πx)、両端 u=0、
α=1、dt=1e-4、2000ステップ (t=0.2)。

```sh
HOST=target/release/dyne-host
WASM=crates/heat1d-actor/target/wasm32-unknown-unknown/release/heat1d_actor.wasm

# 単一ノード参照解: 全域を1プロセスで解く
$HOST --wasm $WASM --label single --n 65 --x0 0.0 --x1 1.0 \
  --bc-left dirichlet --bc-right dirichlet \
  --alpha 1.0 --dt 1e-4 --steps 2000 --out /tmp/single.csv

# 2ノード分割: A=[0, 0.5] 33点 / B=[0.5+h, 1.0] 32点 (重複なし)
$HOST --wasm $WASM --label nodeA --n 33 --x0 0.0 --x1 0.5 \
  --bc-left dirichlet --bc-right interface \
  --listen 127.0.0.1:7801 --peer-right 127.0.0.1:7802 \
  --alpha 1.0 --dt 1e-4 --steps 2000 --out /tmp/nodeA.csv &

$HOST --wasm $WASM --label nodeB --n 32 --x0 0.515625 --x1 1.0 \
  --bc-left interface --bc-right dirichlet \
  --listen 127.0.0.1:7802 --peer-left 127.0.0.1:7801 \
  --alpha 1.0 --dt 1e-4 --steps 2000 --out /tmp/nodeB.csv &
wait
```

出力例:

```
[nodeA] steps=2000 t=0.2000 n=33 interfaces=1 elapsed=793ms resend=1 replay=0 crc_err=0 L2=5.676e-5 Linf=8.027e-5
```

`resend` (タイムアウト再送) と `replay` (過去ステップ再要求への応答) が
数回出るのは正常。起動レースをプロトコルが自動回復した痕跡である。

### dyne-host のオプション

| オプション | 意味 | 既定値 |
|---|---|---|
| `--wasm PATH` | アクターWASMのパス (必須) | — |
| `--n N` | 担当区間の格子点数 (両端含む) | 65 |
| `--x0 / --x1` | 担当区間 | 0.0 / 1.0 |
| `--alpha` | 熱拡散係数 | 1.0 |
| `--dt` | 時間刻み | 1e-4 |
| `--steps` | ステップ数 | 2000 |
| `--bc-left / --bc-right` | 境界条件: `dirichlet` (u=0固定) か `interface` (隣接ノード) | dirichlet |
| `--listen ADDR` | 自分のUDP待受アドレス (interfaceがある場合必須) | — |
| `--peer-left / --peer-right ADDR` | 各面の相手ノードのアドレス | — |
| `--out PATH` | 結果CSV (x, 数値解, 解析解) の出力先 | なし |
| `--label NAME` | ログ表示名 | node |

環境変数 `DYNE_TRACE=1` でステップ進行・再送・ゴースト注入のトレースを表示。

安定条件 α·dt/h² ≤ 1/2 (陽解法) はホストが起動時に検証し、違反時は停止する。

## プロトコル仕様の要点 (M0で確定した部分)

- 1メッセージ = 1境界面 × 1論理ステップ。40バイト固定長、CRC32つき、
  ブローカー非経由のP2P/UDP。
- BSP同期: 全境界面のstep nの値を交換し終えてからn+1へ進む。
- **過去ステップ再送応答**: 各ノードは直近2ステップの送信控えを保持し、
  過去ステップの重複メッセージを受信したら控えを再送して応答する。
  起動レースとパケット消失を同一機構で治癒する (BSPではずれ最大1ステップ)。
- **終了時ドレイン**: 最終ステップ後も500ms受信を続け、再要求に応答してから終了。
- 既知の課題: face_idは現状「送信者から見た面」で受信側の変換が必要。
  M1で「クラスタ全体で一意な界面ID」に変更する (M0で変換漏れバグが実際に
  混入したため。経緯は docs/2026-08-05-m0-report.md 知見3)。

## M0の意図的な簡略化

- WASMランタイムは wasmi (純Rust・通訳方式)。正しさの検証が目的のため。
  C ABIはランタイム非依存で、M1の実機では WAMR (事前コンパイル方式) を使う。
- ノード数は2固定。有界非同期・欠損時外挿は未実装 (仕様A-4の段階案どおり)。

## 次のマイルストーン

M1: ESPr Developer 32 Type-C (ESP32-WROOM-32) 上にWAMRを構築し、
このアクターを実機で動かして限界レポート第2号 (RAM・フラッシュ・
ステップ実行時間) を取る。ロードマップ全体は
`docs/2026-08-04-dyne-spec-review.md` C章を参照。

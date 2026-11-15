# 熱伝導アクターwasmの2系統ビルド (発注7 実機不具合対応(b)、2026-09-18)
# リポジトリのルートで実行: powershell -ExecutionPolicy Bypass -File scripts\build-actor-wasm.ps1
#
#  1) フル版  (N_MAX=4096, スタック32KB, 線形メモリ2ページ=128KB) … シミュレータ用
#     → crates\heat1d-actor\target\wasm32-unknown-unknown\release\heat1d_actor.wasm
#  2) 縮小版  (--features small: N_MAX=2048, スタック8KB, 線形メモリ1ページ=64KB)
#     … 実機ESPr32用 (PSRAMなし、最大連続ヒープ~108KBのため128KBは確保不能)
#     → node-esp32\main\heat1d_actor_small.wasm へ複製
#
# 管理方法: 同一クレート+featureフラグ (数値経路は配列サイズ以外同一)。
# 縮小版の正しさは、シミュレータで縮小版を指定した回帰がフル版と全出力
# バイト一致することで確認する (本スクリプト実行後に scripts\check-f1.ps1 等)。

$ErrorActionPreference = "Stop"

Push-Location crates\heat1d-actor

Write-Host "== [1] フル版 (シミュレータ用) =="
cargo build --release --target wasm32-unknown-unknown

Write-Host "== [2] 縮小版 (実機用: N_MAX=2048, スタック8KB) =="
# RUSTFLAGS は .cargo/config.toml の rustflags を置き換えるので、スタックのみ8KBへ
$env:RUSTFLAGS = "-C link-arg=-zstack-size=8192"
try {
    cargo build --release --target wasm32-unknown-unknown --features small --target-dir target-small
} finally {
    Remove-Item Env:RUSTFLAGS
}
Pop-Location

Copy-Item crates\heat1d-actor\target-small\wasm32-unknown-unknown\release\heat1d_actor.wasm `
    node-esp32\main\heat1d_actor_small.wasm -Force
Write-Host "node-esp32\main\heat1d_actor_small.wasm updated:"
Get-Item node-esp32\main\heat1d_actor_small.wasm | Select-Object Length, LastWriteTime

# ---- 2次元の波動方程式のアクター (2026-11-11、docs/2026-11-11-wave2d.md) ----
# フル版 (シミュレータ用、線形メモリ11ページ) と縮小版 (実機用: --features small、区画は1辺48点まで、
# スタック8KB、線形メモリ1ページ=64KB)。node-esp32 への複製はしない (積み込む wasm の切り替えは未決定)
Push-Location crates\wave2d-actor
Write-Host "== [3] wave2d フル版 =="
cargo build --release --target wasm32-unknown-unknown
Write-Host "== [4] wave2d 縮小版 =="
$env:RUSTFLAGS = "-C link-arg=-zstack-size=8192"
try {
    cargo build --release --target wasm32-unknown-unknown --features small --target-dir target-small
} finally {
    Remove-Item Env:RUSTFLAGS
}
Pop-Location

# ---- 形の格子型アクターの縮小版 (2026-11-11、docs/2026-11-11-esp32-interp-bench.md) ----
# フル版はシミュレータ用 (check-sim2d.ps1 がビルドする)。縮小版 (--features small) は区画 1辺40点・
# 枠なし1080点まで、光線は1ステップ64本まで、線形メモリ1ページ=64KB。
# 2026-11-13: スタックを 6KB → 4KB にした (使用は約1.2KB。視野の中だけの減衰の印 136 バイトを足すと
# 6KB では2ページになる)。実機の推奨は std-math 版 (標準ライブラリの sqrt。結果は同じで速い)。
# f32 版は単精度 (結果が変わる)。docs/2026-11-11-shape-speedup.md
# 実機の計測ファーム bench-c5 への写しは bench-c5\tools\interp_bench.py make が行う
Push-Location crates\sdf2d-grid-actor
$env:RUSTFLAGS = "-C link-arg=-zstack-size=4096"
try {
    Write-Host "== [5] sdf2d 縮小版 =="
    cargo build --release --target wasm32-unknown-unknown --features small --target-dir target-small
    Write-Host "== [6] sdf2d 縮小版 + std-math (推奨) =="
    cargo build --release --target wasm32-unknown-unknown --features "small,std-math" --target-dir target-small-std
    Write-Host "== [7] sdf2d 縮小版 + f32 + std-math =="
    cargo build --release --target wasm32-unknown-unknown --features "small,f32,std-math" --target-dir target-small-f32std
} finally {
    Remove-Item Env:RUSTFLAGS
}
Write-Host "== [8] sdf2d フル版 f32 (単一ノード解の照合用) =="
cargo build --release --target wasm32-unknown-unknown --features f32 --target-dir target-f32
Pop-Location

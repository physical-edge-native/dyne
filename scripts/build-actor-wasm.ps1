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

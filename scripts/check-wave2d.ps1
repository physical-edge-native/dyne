# 2次元の波動方程式のアクター (wave2d-actor) の手元確認スクリプト (2026-11-11)
# リポジトリのルートで実行する: powershell -ExecutionPolicy Bypass -File scripts\check-wave2d.ps1
# 記録: docs/2026-11-11-wave2d.md
#
# 引数 -Sim で dyne-sim2d.exe の場所を変えられる (既定 target\release\dyne-sim2d.exe。
# -Sim を指定したときは dyne-sim2d のビルドを省く)。
#
# 確認内容と期待結果:
#   [1] ビルド … フル版と縮小版 (実機用、線形メモリ1ページ) の wasm、ネイティブの検証プログラム
#   [2] ビット一致 … 11シナリオすべて "PASS (bit-exact)" (分割の形 2x2/3x2(割り切れない)/4x4/1x4/3x3、
#       外周 値固定・ゼロ勾配・吸収境界・混在、損失・遅延・一時停止・分断、限界を超えて発散する場合も)
#   [3] 決定論 … 同じ引数の2回実行で標準出力がバイト一致 (fc「相違点は検出されませんでした」)
#   [4] 縮小版 = フル版 … 同じ条件の crc32 が一致
#   [5] ネイティブ = WASM … wave_study crc の値が [2] の dir_2x2 の single と一致 (064e7eb0)
#   [6] 2次精度とエネルギー保存 … 誤差の比が約4.00、値固定・ゼロ勾配のエネルギーの相対変動が1e-13未満
# 他の作業と %TEMP% の作業領域が衝突しないよう、本スクリプト専用の dyne-wave2d-check を使う。

param([string]$Sim = "")

$ErrorActionPreference = "Stop"
$root = (Get-Location).Path
$w = Join-Path $root "crates\wave2d-actor\target\wasm32-unknown-unknown\release\wave2d_actor.wasm"
$ws = Join-Path $root "crates\wave2d-actor\target-small\wasm32-unknown-unknown\release\wave2d_actor.wasm"
$study = Join-Path $root "crates\wave2d-actor\target\release\examples\wave_study.exe"

Write-Host "== [1] build =="
Push-Location crates\wave2d-actor
cargo build --release --target wasm32-unknown-unknown
$env:RUSTFLAGS = "-C link-arg=-zstack-size=8192"
try {
    cargo build --release --target wasm32-unknown-unknown --features small --target-dir target-small
} finally {
    Remove-Item Env:RUSTFLAGS
}
cargo build --release --example wave_study
Pop-Location
if ($Sim -eq "") {
    cargo build --release -p dyne-sim2d
    $exe = Join-Path $root "target\release\dyne-sim2d.exe"
} else {
    $exe = $Sim
}

# nu = 0.5 (dt = 0.5 h, h = 1/(nx-1))
$gauss = @("--cfg","0=1","--cfg","4=0.3","--cfg","5=0.6","--cfg","6=0.06")
$scenarios = [ordered]@{
    "dir_2x2"        = @("--px","2","--py","2","--dt","0.0125","--steps","400")
    "dir_loss"       = @("--px","2","--py","2","--dt","0.0125","--steps","400","--loss","0.1","--lat-ms","5","--lat-jitter-ms","3")
    "gauss_pause"    = @("--px","2","--py","2","--dt","0.0125","--steps","400","--loss","0.1","--pause","3@2000:6000","--partition","0-1@10000:15000","--partition","1-3@12000:20000") + $gauss
    "3x2_uneven"     = @("--px","3","--py","2","--nx","40","--ny","33","--dt","0.01282051282051282","--steps","300","--loss","0.15","--lat-ms","5","--lat-jitter-ms","10") + $gauss
    "1x4_strip"      = @("--px","1","--py","4","--dt","0.0125","--steps","300","--loss","0.1") + $gauss
    "4x4_pause"      = @("--px","4","--py","4","--nx","61","--ny","61","--dt","0.008333333333333333","--steps","300","--loss","0.1","--lat-ms","3","--lat-jitter-ms","3","--pause","5@3000:8000") + $gauss
    "zero_grad_cos"  = @("--px","2","--py","2","--dt","0.0125","--steps","400","--outer-bc","zero_grad","--loss","0.1","--cfg","0=2")
    "zero_grad_gs"   = @("--px","3","--py","3","--dt","0.0125","--steps","400","--outer-bc","zero_grad","--loss","0.1") + $gauss
    "absorb_3x3"     = @("--px","3","--py","3","--nx","45","--ny","31","--dt","0.011363636363636364","--steps","300","--outer-bc","zero_grad","--cfg","7=15","--loss","0.2","--lat-jitter-ms","50") + $gauss
    "mixed_dir05_WN" = @("--px","2","--py","3","--dt","0.0125","--steps","300","--outer-bc","dirichlet(0.5)","--cfg","7=9","--loss","0.1") + $gauss
    "unstable_072"   = @("--px","2","--py","2","--dt","0.018","--steps","400","--loss","0.1") + $gauss
}

Write-Host "== [2] bit-exact (all PASS) =="
foreach ($k in $scenarios.Keys) {
    $lines = & $exe --wasm $w --quiet @($scenarios[$k]) | Select-String -Pattern "PASS|FAIL|crc32|max\|u\|"
    Write-Host ("{0,-15} {1}" -f $k, ($lines -join " | "))
}

Write-Host "== [3] determinism (no differences) =="
$work = Join-Path $env:TEMP "dyne-wave2d-check"
if (Test-Path $work) { Remove-Item -Recurse -Force $work }
New-Item -ItemType Directory $work | Out-Null
$a3 = $scenarios["gauss_pause"]
& $exe --wasm $w @a3 > (Join-Path $work "a.txt")
& $exe --wasm $w @a3 > (Join-Path $work "b.txt")
fc.exe (Join-Path $work "a.txt") (Join-Path $work "b.txt")

Write-Host "== [4] small build = full build (same crc32) =="
foreach ($k in "dir_2x2", "gauss_pause", "absorb_3x3") {
    $f = & $exe --wasm $w --quiet @($scenarios[$k]) | Select-String -Pattern "crc32"
    $s = & $exe --wasm $ws --quiet @($scenarios[$k]) | Select-String -Pattern "crc32|wasm memory"
    Write-Host ("{0,-12} full: {1}" -f $k, $f)
    Write-Host ("{0,-12} small: {1}" -f "", ($s -join " | "))
}

Write-Host "== [5] native = wasm (expect 064e7eb0) =="
& $study crc 41 0.0125 400

Write-Host "== [6] convergence and energy =="
& $study conv
& $study energy

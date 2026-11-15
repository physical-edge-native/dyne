# 部屋のデジタルツインのデモを一括で作る (2026-11-08)。
#   1. 形状の格子型アクター(sdf2d-grid-actor)と 2 次元シミュレータ(dyne-sim2d)をビルド
#   2. 場面ファイルの部屋で形状推定を実行して CSV を書き出す
#   3. scripts/twin_demo.py で 3 次元表示の HTML と USD(.usda)を作る
#      (2026-11-11: USD に物理の衝突判定を付け、正解の形の USD と MuJoCo の模型 room.xml も作る)
#   4. scripts/usd_check.py と scripts/mjcf_check.py で、物理シミュレータに取り込めるかを検査する(-NoCheck で省略)
# 出力先: demo_out/twin/ (git 管理外)
#
# 既定は理想条件の 6 畳の部屋 (scenes/room6.txt): 8x4=32 ノード、54 台のセンサ、
#   1 台 64 方向・視野 60 度、測距の雑音 2 mm、格子の刻み 2.5 cm。
# 以前の「物の多い部屋・4 台のセンサ・2x2 ノード」は -Preset cluttered で実行できる。
#
# 使い方 (リポジトリ直下で):
#   powershell -ExecutionPolicy Bypass -File scripts\twin_demo.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\twin_demo.ps1 -Preset cluttered
param(
    [ValidateSet("room6", "cluttered")]
    [string]$Preset = "room6",
    [string]$Scene = "",
    [int]$Steps = 0,
    [string]$OutDir = "demo_out\twin",
    [switch]$NoCheck
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

if ($Preset -eq "room6") {
    if ($Scene -eq "") { $Scene = "crates\dyne-sim2d\scenes\room6.txt" }
    if ($Steps -eq 0) { $Steps = 40 }
    $simArgs = @("--px", "8", "--py", "4", "--h", "0.025", "--zones", "64", "--fov-deg", "60", "--noise-m", "0.002")
    $pyExtra = @("--zones", "64", "--fov-deg", "60")
} else {
    if ($Scene -eq "") { $Scene = "crates\dyne-sim2d\scenes\cluttered.txt" }
    if ($Steps -eq 0) { $Steps = 400 }
    $simArgs = @()
    $pyExtra = @("--zones", "8", "--fov-deg", "45", "--room-height", "2.4")
}

Write-Host "== アクターとシミュレータをビルド"
Push-Location crates\sdf2d-grid-actor
cargo build --release --target wasm32-unknown-unknown
if ($LASTEXITCODE -ne 0) { Pop-Location; throw "sdf2d-grid-actor のビルドに失敗" }
Pop-Location
cargo build --release -p dyne-sim2d
if ($LASTEXITCODE -ne 0) { throw "dyne-sim2d のビルドに失敗" }

New-Item -ItemType Directory -Force $OutDir | Out-Null
$wasm = "crates\sdf2d-grid-actor\target\wasm32-unknown-unknown\release\sdf2d_grid_actor.wasm"
$shape = Join-Path $OutDir "shape.csv"
$log = Join-Path $OutDir "sim.log"

Write-Host "== 形状推定を実行 ($Preset, $Scene, $Steps ステップ)"
$simOut = & target\release\dyne-sim2d.exe --wasm $wasm --scene $Scene --steps $Steps @simArgs --field-csv $shape
if ($LASTEXITCODE -ne 0) { throw "dyne-sim2d が失敗" }
# 記録は UTF-8 (BOM なし) で書く
[System.IO.File]::WriteAllLines((Join-Path $root $log), [string[]]$simOut, (New-Object System.Text.UTF8Encoding($false)))
$simOut | Select-String -Pattern "^(shape:|PASS|FAIL)" | ForEach-Object { Write-Host $_.Line }

$py = Join-Path $root ".venv\Scripts\python.exe"
if (-not (Test-Path $py)) { $py = "python" }
$env:PYTHONIOENCODING = "utf-8"
$pyArgs = @("scripts\twin_demo.py", "--scene", $Scene, "--shape-csv", $shape, "--sim-log", $log, "--out", (Join-Path $OutDir "room")) + $pyExtra

$pyArgs += @("--truth-usd")

Write-Host "== HTML と USD を作成"
& $py @pyArgs
if ($LASTEXITCODE -ne 0) { throw "twin_demo.py が失敗" }
Write-Host ("出力: " + (Resolve-Path (Join-Path $OutDir "room.html")) + " / room.usda / room_truth.usda / room.xml")

# 物理シミュレータへの取り込みの検査 (2026-11-11、docs/2026-11-11-usd-physics.md)
#   USD の形式(UsdPhysics の衝突判定・閉じたメッシュ・単位)は常に合格が必要。
#   正解との外接箱の差と、MuJoCo での通り抜け(推定の形の穴)は、理想条件の room6 だけ合格を求める
if (-not $NoCheck) {
    $usd = Join-Path $OutDir "room.usda"
    Write-Host "== USD の検査 (scripts\usd_check.py)"
    $chk = @("scripts\usd_check.py", $usd, "--scene", $Scene)
    if ($Preset -ne "room6") { $chk += @("--bbox-warn-only") }
    & $py @chk
    if ($LASTEXITCODE -ne 0) { throw "usd_check.py が FAIL" }
    & $py -c "import mujoco" 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "== MuJoCo での衝突の検査 (scripts\mjcf_check.py)"
        # 2026-11-13: 抜け出しの試験 (--escape。未観測を物とみなした衝突判定に穴が無いこと) も行う
        $mj = @("scripts\mjcf_check.py", "--xml", (Join-Path $OutDir "room.xml"), "--usd", $usd, "--scene", $Scene, "--escape")
        if ($Preset -eq "room6") { $mj += @("--strict") }
        & $py @mj
        if ($LASTEXITCODE -ne 0) { throw "mjcf_check.py が FAIL" }
    } else {
        Write-Host "== mujoco が .venv に無いので MuJoCo の検査は省略 (.venv\Scripts\python.exe -m pip install mujoco)"
    }
}

# Room tutorial (docs/en/tutorial-room.md, GitHub issue #96): a dummy room with simulated devices,
# from dummy sensor readings to Dyne's shape and temperature, then USD, a 3D dashboard and an OpenFOAM case.
# No hardware is needed. The same steps as scripts/tutorial_room.sh, for Windows PowerShell.
#
# Usage (from the repository root):
#   powershell -ExecutionPolicy Bypass -File scripts\tutorial_room.ps1             # steps 1-3
#   powershell -ExecutionPolicy Bypass -File scripts\tutorial_room.ps1 -SimOnly    # Rust parts only (no Python)
#   powershell -ExecutionPolicy Bypass -File scripts\tutorial_room.ps1 -OpenFOAM   # also step 4: run the case
#       with OpenFOAM inside WSL (Windows Subsystem for Linux); -WslDistro Ubuntu picks the distribution
#   powershell -ExecutionPolicy Bypass -File scripts\tutorial_room.ps1 -Figures docs\demo\tutorial
#       # also draw the tutorial's pictures into that folder
#
# Outputs go to demo_out\tutorial\ (not tracked by git).
param(
    [switch]$SimOnly,
    [switch]$OpenFOAM,
    [string]$WslDistro = "",
    [string]$Figures = "",
    [string]$OutDir = "demo_out\tutorial"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$utf8 = New-Object System.Text.UTF8Encoding($false)

$scene = "crates\dyne-sim2d\scenes\room6.txt"
$readings = "crates\dyne-sim2d\scenes\room6_temp.csv"
$sim2 = "target\release\dyne-sim2d.exe"
$foam = "target\release\dyne-foam.exe"
$ws = "crates\sdf2d-grid-actor\target\wasm32-unknown-unknown\release\sdf2d_grid_actor.wasm"
$wh = "crates\heat2d-actor\target\wasm32-unknown-unknown\release\heat2d_actor.wasm"

# Runs a native program whose standard error may be written (PowerShell 5.1 turns redirected standard error
# into errors when ErrorActionPreference is Stop); returns its exit code
function Test-Native([scriptblock]$block) {
    $old = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try { & $block 2>$null | Out-Null } catch {}
    $rc = $LASTEXITCODE
    $ErrorActionPreference = $old
    return $rc
}

function Invoke-Checked([string]$what, [scriptblock]$block) {
    & $block
    if ($LASTEXITCODE -ne 0) { throw "$what failed (exit $LASTEXITCODE)" }
}

# Runs a program, writes its standard output to a UTF-8 file (no BOM) and returns the lines
function Invoke-Logged([string]$exe, [string[]]$argv, [string]$log) {
    $out = & $exe @argv
    if ($LASTEXITCODE -ne 0) { throw "$exe failed (exit $LASTEXITCODE)" }
    [System.IO.File]::WriteAllLines((Join-Path $root $log), [string[]]$out, $utf8)
    return $out
}

Write-Host "== [0] build: two WebAssembly actors (shape, heat) and the simulator and exporter"
Push-Location crates\sdf2d-grid-actor
Invoke-Checked "sdf2d-grid-actor build" { cargo build --release --target wasm32-unknown-unknown }
Pop-Location
Push-Location crates\heat2d-actor
Invoke-Checked "heat2d-actor build" { cargo build --release --target wasm32-unknown-unknown }
Pop-Location
Invoke-Checked "dyne-sim2d / dyne-export build" { cargo build --release -p dyne-sim2d -p dyne-export }
New-Item -ItemType Directory -Force $OutDir | Out-Null

Write-Host "== [1] the dummy room: $scene (walls, furniture, 54 distance sensors on 9 devices)"
Write-Host "       and the dummy thermometer readings: $readings"
Get-Content $readings | Where-Object { -not $_.StartsWith("#") } | ForEach-Object { Write-Host $_ }

Write-Host "== [2a] shape: 32 simulated devices (8 x 4) build the room's signed distance field from simulated distance frames"
$shapeLog = Join-Path $OutDir "shape.log"
$o = Invoke-Logged $sim2 @("--wasm", $ws, "--scene", $scene, "--steps", "40", "--px", "8", "--py", "4", "--h", "0.025",
    "--zones", "64", "--fov-deg", "60", "--noise-m", "0.002",
    "--field-csv", (Join-Path $OutDir "shape.csv"), "--vti", (Join-Path $OutDir "shape.vti")) $shapeLog
$o | Select-String -Pattern "^(shape: surface|crc32|PASS|FAIL)" | ForEach-Object { Write-Host $_.Line }

Write-Host "== [2b] temperature: the same 32 devices run the 2D heat model, nudged to the 12 dummy readings"
$tempVti = Join-Path $OutDir "temperature.vti"
$o2 = Invoke-Logged $sim2 @("--wasm", $wh, "--scene", $scene, "--h", "0.1", "--px", "8", "--py", "4", "--alpha", "1",
    "--dt", "0.002", "--outer-bc", "zero_grad", "--init-value", "20", "--temp-sensors", $readings, "--nudge-gamma", "0.8",
    "--steps", "2000", "--vti", $tempVti) (Join-Path $OutDir "temperature.log")
$o2 | Select-String -Pattern "^(nudging|crc32|PASS|FAIL)" | ForEach-Object { Write-Host $_.Line }
if (-not (($o | Select-String "^PASS \(bit-exact\)") -and ($o2 | Select-String "^PASS \(bit-exact\)"))) {
    throw "the split solution differs from the single-device solution (see $OutDir\*.log)"
}

$py = $null
if (-not $SimOnly) {
    foreach ($c in @($env:PYTHON, (Join-Path $root ".venv\Scripts\python.exe"), "python")) {
        if (-not $c -or -not (Get-Command $c -ErrorAction SilentlyContinue)) { continue }
        if ((Test-Native { & $c -c "import numpy" }) -eq 0) { $py = $c; break }
    }
    if (-not $py) { Write-Host "(no Python with numpy found: the Python parts are skipped; see docs\en\installation.md)" }
}
$env:PYTHONIOENCODING = "utf-8"

if ($py) {
    Write-Host "== [2c] check: Dyne's temperature against the readings (tolerance 0.5 C) and the made-up field (reported)"
    Invoke-Checked "tutorial_room.py check" { & $py scripts\tutorial_room.py check --scene $scene --heat-vti $tempVti --readings $readings }
}

Write-Host "== [3a] OpenFOAM case from Dyne's temperature field (OpenFOAM itself is not needed for this)"
$case = Join-Path $OutDir "openfoam"
if (Test-Path $case) { Remove-Item -Recurse -Force $case }
$foamOpts = @("--end-time", "3600", "--delta-t", "10", "--write-interval", "600")
$oldEap = $ErrorActionPreference; $ErrorActionPreference = "Continue"
$usage = (& $foam 2>&1 | Out-String)
$ErrorActionPreference = $oldEap
$walls = $false
if ($usage -match "--shape" -and $py) {
    # Walls and furniture as solid cells (dyne-foam --shape, GitHub issue #91). The shape grid (2.5 cm) is
    # taken at the points of the heat grid (10 cm); the uniform velocity stays 0 (a uniform flow with walls diverges).
    $coarse = Join-Path $OutDir "shape_10cm.vti"
    Invoke-Checked "coarse-shape" { & $py scripts\tutorial_room.py coarse-shape --shape-vti (Join-Path $OutDir "shape.vti") --heat-vti $tempVti --out $coarse }
    Invoke-Checked "dyne-foam case" { & $foam case --vti $tempVti --shape $coarse --out $case @foamOpts }
    $walls = $true
} else {
    # TODO(#91): once dyne-foam has --shape (issue #91), the walls are added automatically (branch above).
    Write-Host "(walls: this dyne-foam has no --shape yet (issue #91) or Python is missing; the case is the whole rectangle)"
    Invoke-Checked "dyne-foam case" { & $foam case --vti $tempVti --out $case @foamOpts }
}
& $foam info $case | Select-String -Pattern "^(T:|mesh|faces)" | ForEach-Object { Write-Host $_.Line }

if ($py -and -not $SimOnly) {
    Write-Host "== [3b] USD (physics colliders), MuJoCo model and the 3D dashboard (HTML)"
    if ((Test-Native { & $py -c "import pxr" }) -eq 0) {
        Invoke-Checked "twin_demo.py" { & $py scripts\twin_demo.py --scene $scene --shape-csv (Join-Path $OutDir "shape.csv") `
            --sim-log $shapeLog --out (Join-Path $OutDir "room") --zones 64 --fov-deg 60 --truth-usd `
            --temp-vti $tempVti --temp-readings $readings --title "Dyne room tutorial (simulated devices, dummy readings)" }
        Invoke-Checked "usd_check.py" { & $py scripts\usd_check.py (Join-Path $OutDir "room.usda") --scene $scene | Select-Object -Last 1 }
        Write-Host ("dashboard: " + (Resolve-Path (Join-Path $OutDir "room.html")))
    } else {
        Write-Host "(usd-core is not installed in this Python: USD and the dashboard are skipped; pip install -r scripts\requirements-pc.txt in the .venv)"
    }
}

if ($OpenFOAM) {
    Write-Host "== [4] optional: run the case with OpenFOAM in WSL (scalarTransportFoam, 1 hour of still-air diffusion)"
    $full = (Resolve-Path $case).Path
    $wslPath = "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
    $mesh = if ($walls) { "checkMesh > log.checkMesh 2>&1" } else { "blockMesh > log.blockMesh 2>&1" }
    $run = "$mesh && scalarTransportFoam > log.scalarTransportFoam 2>&1 && postProcess -func writeCellCentres -time 0 > log.cellCentres 2>&1"
    $sh = "cd '$wslPath' && if command -v blockMesh >/dev/null 2>&1; then bash -c '$run'; elif command -v openfoam2606 >/dev/null 2>&1; then openfoam2606 -c '$run'; else echo 'OpenFOAM not found in WSL'; exit 3; fi"
    $wslArgs = @()
    if ($WslDistro) { $wslArgs += @("-d", $WslDistro) }
    & wsl @wslArgs -- bash -lc $sh
    if ($LASTEXITCODE -ne 0) { Write-Host "(OpenFOAM run failed or not installed in WSL: exit $LASTEXITCODE; see $case\log.*)" }
    else { Write-Host "OpenFOAM: done, results in $case (times 600 ... 3600 s)" }
}

if ($Figures -and $py) {
    Write-Host "== pictures -> $Figures"
    Invoke-Checked "figures" { & $py scripts\tutorial_room.py figures --scene $scene --readings $readings `
        --shape-csv (Join-Path $OutDir "shape.csv") --heat-vti $tempVti --foam-case $case --out-dir $Figures }
    $html = Join-Path $OutDir "room.html"
    if (Test-Path $html) { & $py scripts\tutorial_room.py screenshot --html $html --out (Join-Path $Figures "04-dashboard.png") }
}

Write-Host "== done: outputs in $OutDir\"

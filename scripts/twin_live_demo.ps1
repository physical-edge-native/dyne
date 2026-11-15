# End-to-end test of the PC resident pipeline without hardware (2026-11-13, docs/2026-11-13-pc-pipeline-rl.md).
#   1. (once) run dyne-sim2d on scenes/room6_move.txt to 280 steps (box at mark A) and 600 steps (box at mark B)
#   2. start scripts/twin_fake_nodes.py: replays the two fields as per-node shape tiles over MQTT (dyne/node/<id>/shape)
#   3. start scripts/twin_live.py: receives tiles, re-exports USD/MJCF/HTML/JSON every few seconds, serves the dashboard
#      open http://127.0.0.1:8765/latest/room.html?poll=3 while it runs
# Needs: mosquitto on 127.0.0.1:1883, repo .venv with paho-mqtt, usd-core, numpy, mujoco.
# Usage (repo root): powershell -ExecutionPolicy Bypass -File scripts\twin_live_demo.ps1 [-Seconds 120] [-Sim <dyne-sim2d.exe>] [-Wasm <sdf2d_grid_actor.wasm>]
param(
    [int]$Seconds = 120,
    [string]$Sim = "target\release\dyne-sim2d.exe",
    [string]$Wasm = "crates\sdf2d-grid-actor\target\wasm32-unknown-unknown\release\sdf2d_grid_actor.wasm",
    [string]$OutDir = "demo_out\twin_live",
    [int]$HttpPort = 8765
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$py = Join-Path $root ".venv\Scripts\python.exe"
$env:PYTHONIOENCODING = "utf-8"
$mv = Join-Path $OutDir "sim"
New-Item -ItemType Directory -Force $mv | Out-Null
$scene = "crates\dyne-sim2d\scenes\room6_move.txt"
foreach ($st in 280, 600) {
    $csv = Join-Path $mv "move_$st.csv"
    if (-not (Test-Path $csv)) {
        Write-Host "== dyne-sim2d room6_move, $st steps (recommended forgetting settings, noise 8 mm)"
        $o = & $Sim --wasm $Wasm --scene $scene --px 4 --py 4 --steps $st --step-ms 66.667 --noise-m 0.008 `
            --decay 0.9 --w-forget 0.3 --disagree 0.1 --disagree-w 1 --field-csv $csv
        if ($LASTEXITCODE -ne 0) { throw "dyne-sim2d failed" }
        [System.IO.File]::WriteAllLines((Join-Path $root (Join-Path $mv "move_$st.log")), [string[]]$o, (New-Object System.Text.UTF8Encoding($false)))
    }
}
$f1 = (Join-Path $mv "move_280.csv") + ":" + (Join-Path $mv "move_280.log")
$f2 = (Join-Path $mv "move_600.csv") + ":" + (Join-Path $mv "move_600.log")
Write-Host "== fake nodes (16 tiles/s, scene switches every 20 s)"
$fk = Start-Process -FilePath $py -ArgumentList "scripts\twin_fake_nodes.py", "--frame", $f1, "--frame", $f2, "--frame-seconds", "20", "--period", "1", "--loop", "--duration", "$Seconds" -PassThru -WindowStyle Hidden
Write-Host "== resident pipeline: http://127.0.0.1:$HttpPort/latest/room.html?poll=3"
& $py scripts\twin_live.py --out $OutDir --scene $scene --every 5 --min-nodes 16 --timeout ($Seconds + 5) --http-port $HttpPort
$fk.WaitForExit()
Write-Host "outputs: $OutDir\latest (room.usda / room.xml / room.html / room.json / status.json), $OutDir\frames"

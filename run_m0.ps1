# Run M0 end-to-end (Windows PowerShell 5.1 / pwsh 7)
# single-node reference -> 2-node split cooperative run -> bit-exact check
# NOTE: messages are ASCII-only on purpose (PS 5.1 encoding quirks).
$ErrorActionPreference = "Stop"

$HostExe = "target\release\dyne-host.exe"
$Wasm    = "crates\heat1d-actor\target\wasm32-unknown-unknown\release\heat1d_actor.wasm"
$Out     = $env:TEMP

$Py = if (Get-Command python -ErrorAction SilentlyContinue) { "python" } else { "py" }

# 1) single-node reference
& $HostExe --wasm $Wasm --label single --n 65 --x0 0.0 --x1 1.0 `
  --bc-left dirichlet --bc-right dirichlet `
  --alpha 1.0 --dt 1e-4 --steps 2000 --out "$Out\single.csv"
if ($LASTEXITCODE -ne 0) { throw "single-node run failed (exit=$LASTEXITCODE)" }

# 2) 2-node split: A=[0,0.5] 33 pts / B=[0.5+h,1.0] 32 pts
$argsA = @("--wasm", $Wasm, "--label", "nodeA", "--n", "33", "--x0", "0.0", "--x1", "0.5",
  "--bc-left", "dirichlet", "--bc-right", "interface",
  "--listen", "127.0.0.1:7801", "--peer-right", "127.0.0.1:7802",
  "--alpha", "1.0", "--dt", "1e-4", "--steps", "2000", "--out", "$Out\nodeA.csv")
$argsB = @("--wasm", $Wasm, "--label", "nodeB", "--n", "32", "--x0", "0.515625", "--x1", "1.0",
  "--bc-left", "interface", "--bc-right", "dirichlet",
  "--listen", "127.0.0.1:7802", "--peer-left", "127.0.0.1:7801",
  "--alpha", "1.0", "--dt", "1e-4", "--steps", "2000", "--out", "$Out\nodeB.csv")

$pA = Start-Process -FilePath $HostExe -ArgumentList $argsA -NoNewWindow -PassThru
$pB = Start-Process -FilePath $HostExe -ArgumentList $argsB -NoNewWindow -PassThru
$pA.WaitForExit(); $pB.WaitForExit()

# PS 5.1 may report $null ExitCode for -NoNewWindow -PassThru processes.
# Treat $null as success; the bit-exact check below is the real arbiter.
if (($null -ne $pA.ExitCode -and $pA.ExitCode -ne 0) -or
    ($null -ne $pB.ExitCode -and $pB.ExitCode -ne 0)) {
  throw "node run failed (A=$($pA.ExitCode) B=$($pB.ExitCode))"
}

# 3) bit-exact verification (this is the authoritative pass/fail)
& $Py scripts\compare_m0.py "$Out\single.csv" "$Out\nodeA.csv" "$Out\nodeB.csv"
if ($LASTEXITCODE -ne 0) { throw "verification failed" }
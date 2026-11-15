# The 12 one-dimensional regression scenarios (s01-s12), Windows entry point.
# Builds dyne-sim and the heat1d / heat1d-pinn actors, then runs scripts/check-baseline12.sh with the
# bash of Git for Windows. The comparison itself (expected SHA-256 of all 30 output files, stored in
# scripts/baseline12.sha256) lives in that script, which also runs on Linux/macOS and in scripts/ci-check.sh.
# Reference: commit 2f7bdcf (formerly a435bf6 before the 2026-11-15 history rewrite); see the .sh header.
#
# Run from the repository root:
#   powershell -ExecutionPolicy Bypass -File scripts\check-baseline12.ps1
# Extra arguments are passed to the .sh script (e.g. --exe PATH --legacy, --write).
# Expected result: 12 lines "MATCH" and "ALL 12 SCENARIOS MATCH". Outputs are kept in target\baseline12\.

$ErrorActionPreference = "Stop"
$root = (Get-Location).Path

Push-Location (Join-Path $root "crates\heat1d-actor")
cargo build --release --target wasm32-unknown-unknown
Pop-Location
Push-Location (Join-Path $root "crates\heat1d-pinn-actor")
cargo build --release --target wasm32-unknown-unknown
Pop-Location
cargo build --release -p dyne-sim
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# Use the bash that comes with Git for Windows (not C:\Windows\System32\bash.exe, which is WSL).
$gitExec = (git --exec-path).Trim()          # e.g. C:/Program Files/Git/mingw64/libexec/git-core
$gitRoot = Split-Path (Split-Path (Split-Path $gitExec))
$bash = Join-Path $gitRoot "bin\bash.exe"
if (-not (Test-Path $bash)) { throw "bash of Git for Windows not found at $bash" }

& $bash scripts/check-baseline12.sh @args
exit $LASTEXITCODE

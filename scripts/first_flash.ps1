# 最初の USB 書き込みを1台ずつ続けて行い、MAC とノード番号の対応を CSV に残す (scripts/first_flash.py を .venv の Python で動かす)
#   使い方 (リポジトリのルートで): powershell -ExecutionPolicy Bypass -File scripts\first_flash.ps1 [first_flash.py のオプション]
#   例: powershell -ExecutionPolicy Bypass -File scripts\first_flash.ps1 --port COM5
#   オプションの説明: .venv\Scripts\python.exe scripts\first_flash.py --help
# 説明: docs/2026-11-13-firmware-nodes.md §9.0
$root = Split-Path $PSScriptRoot -Parent
& (Join-Path $root ".venv\Scripts\python.exe") (Join-Path $PSScriptRoot "first_flash.py") @args
exit $LASTEXITCODE

# 時刻同期と OTA の、実機なしでできる確認をまとめて実行する (2026-11-11)
#   使い方 (リポジトリのルートで): powershell -ExecutionPolicy Bypass -File scripts\check-timesync-ota.ps1 [-E2E]
#   -E2E : PC の mosquitto (127.0.0.1:1883) に、ノードのふりをするプログラムをつないで
#          dyne_ota.py の一斉配布を通しで確かめる (約1分。paho-mqtt が要る)
# 説明: docs/2026-11-11-timesync-ota.md
param([switch]$E2E)
$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent
Set-Location $root
$py = Join-Path $root ".venv\Scripts\python.exe"
$tmp = Join-Path $env:TEMP "dyne_check_tsota"
New-Item -ItemType Directory -Force $tmp | Out-Null
$fail = 0
function Check($name, $ok) {
    if ($ok) { Write-Host "[OK]   $name" } else { Write-Host "[FAIL] $name"; $script:fail++ }
}

# 1. C の計算部分 (PC でビルドして実行)
$tests = @(
    @("wire2_selftest", "node-esp32\host_test\wire2_selftest.c", "node-esp32\main\dyne_wire2.c", ""),
    @("timesync_test", "node-esp32\host_test\timesync_test.c", "node-esp32\main\dyne_timesync_core.c", "-lws2_32"),
    @("actor_slot_test", "node-esp32\host_test\actor_slot_test.c", "node-esp32\main\dyne_actor_slot.c", "")
)
foreach ($t in $tests) {
    $exe = Join-Path $tmp "$($t[0]).exe"
    $cargs = @("-m", "ziglang", "cc", "-std=c11", "-Wall", "-Wextra", "-O2", "-Inode-esp32\main", $t[1], $t[2], "-o", $exe)
    if ($t[3]) { $cargs += $t[3] }
    & $py @cargs
    $out = & $exe
    Check "$($t[0])" (($LASTEXITCODE -eq 0) -and ($out[-1] -eq "PASS"))
}

# 2. PC の時刻サーバと、Python・C の問い合わせ側
$srv = Start-Process -FilePath $py -ArgumentList "scripts\dyne_ntp_server.py", "--bind", "127.0.0.1", "--port", "12300", "--quiet" -PassThru -WindowStyle Hidden
Start-Sleep -Milliseconds 800
try {
    & $py scripts\dyne_ntp_server.py --query 127.0.0.1:12300 -n 10 | Select-Object -Last 1
    Check "ntp server: python query" ($LASTEXITCODE -eq 0)
    $out = & (Join-Path $tmp "timesync_test.exe") 127.0.0.1 12300
    $out | Select-String "応答|rtt" | Select-Object -Last 2
    Check "ntp server: C client (firmware core)" (($LASTEXITCODE -eq 0) -and ($out[-1] -eq "PASS"))
} finally {
    Stop-Process -Id $srv.Id -ErrorAction SilentlyContinue
}

# 3. ロジックアナライザの記録の解析 (合成した記録)
$out = & $py scripts\pps_analyze.py --selftest
Check "pps_analyze selftest" ($out[-1] -eq "PASS")

# 4. OTA の像の扱い (ビルド済みの .bin があれば)
foreach ($b in "node-esp32\build-ota-esp32\dyne_node.bin", "node-esp32\build-ota-esp32c5\dyne_node.bin") {
    if (-not (Test-Path $b)) { Write-Host "[skip] $b が無い"; continue }
    & $py scripts\dyne_ota.py info --bin $b | Select-Object -Last 2
    Check "image ok: $b" ($LASTEXITCODE -eq 0)
    $bs = Join-Path $tmp "broken_st.bin"; $bc = Join-Path $tmp "broken_cor.bin"
    & $py scripts\dyne_ota.py make-broken --bin $b --mode selftest-fail --out $bs | Out-Null
    & $py scripts\dyne_ota.py info --bin $bs | Out-Null
    Check "selftest-fail image still valid: $b" ($LASTEXITCODE -eq 0)
    & $py scripts\dyne_ota.py make-broken --bin $b --mode corrupt --out $bc | Out-Null
    & $py scripts\dyne_ota.py info --bin $bc | Out-Null
    Check "corrupt image detected: $b" ($LASTEXITCODE -ne 0)
}

# 5. 一斉配布の通し (任意)
if ($E2E) {
    $b = "node-esp32\build-ota-esp32\dyne_node.bin"
    $w = "node-esp32\main\heat1d_actor_small.wasm"
    $csvp = Join-Path $tmp "ota_e2e.csv"
    Remove-Item $csvp -ErrorAction SilentlyContinue
    # 2026-11-13: paho-mqtt はリポジトリの .venv に入れた。素の python ではなく .venv の python で動かす
    $fk = Start-Process -FilePath $py -ArgumentList "node-esp32\host_test\fake_node.py", "--ids", "901,902", "--lifetime", "60" -PassThru -WindowStyle Hidden
    Start-Sleep -Seconds 2
    $o = @("--broker", "127.0.0.1", "--csv", $csvp, "--timeout", "20", "--http-port", "8071", "--nodes", "901,902")
    & $py scripts\dyne_ota.py fw @o --bin $b | Out-Null; Check "e2e fw normal -> ok" ($LASTEXITCODE -eq 0)
    & $py scripts\dyne_ota.py fw @o --bin $b --broken selftest-fail | Out-Null; Check "e2e fw selftest-fail -> rolled_back" ($LASTEXITCODE -eq 0)
    & $py scripts\dyne_ota.py fw @o --bin $b --broken corrupt | Out-Null; Check "e2e fw corrupt -> rejected" ($LASTEXITCODE -eq 0)
    & $py scripts\dyne_ota.py actor @o --wasm $w | Out-Null; Check "e2e actor normal -> ok" ($LASTEXITCODE -eq 0)
    & $py scripts\dyne_ota.py actor @o --wasm $w --broken garbage | Out-Null; Check "e2e actor garbage -> fallback" ($LASTEXITCODE -eq 0)
    & $py scripts\dyne_ota.py actor @o --wasm $w --broken badsha | Out-Null; Check "e2e actor badsha -> rejected" ($LASTEXITCODE -eq 0)
    # ノードのふりは寿命 (60秒) で自分の retained の status を消して終わる。それを待つ
    $fk.WaitForExit()
    Write-Host "結果の CSV: $csvp"
}

if ($fail -eq 0) { Write-Host "ALL PASS" } else { Write-Host "$fail FAILED"; exit 1 }

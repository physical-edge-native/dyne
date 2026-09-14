# 基準①(既存全回帰の維持)の手元確認スクリプト (2026-09-06、2026-09-14 F1対応)。任意実行。
# 切り出し前のコミット a435bf6 を一時worktreeに取り出して dyne-sim をビルドし、
# 現行の dyne-sim と同一コマンド・同一アクターWASMで全12シナリオを実行、
# 標準出力・標準エラー・全CSV(計30ファイル)をバイト比較する。
# F1対応 (2026-09-14): 現行側は --unobserved-bc dirichlet:0 を指定して従来挙動に固定し、
# 現行側の out.log にだけ出る設定表示行 "unobserved-bc = ..."(--add のあるシナリオのみ)
# を除いて比較する。数値出力 (CSV・RESULT行) は従来どおり全てバイト一致が条件。
# リポジトリのルートで実行する: powershell -ExecutionPolicy Bypass -File scripts\check-baseline12.ps1
# 期待結果: 12行すべて "MATCH"。初回はベースラインのビルドで数分かかる。
# 後片付け: git worktree remove --force ..\dyne-base-a435bf6

$ErrorActionPreference = "Stop"
$root = (Get-Location).Path
$baseTree = Join-Path (Split-Path $root) "dyne-base-a435bf6"

if (-not (Test-Path $baseTree)) {
    git worktree add $baseTree a435bf6
}
cargo build --release -p dyne-sim --manifest-path (Join-Path $baseTree "Cargo.toml")
cargo build --release -p dyne-sim

$baseExe = Join-Path $baseTree "target\release\dyne-sim.exe"
$curExe  = Join-Path $root "target\release\dyne-sim.exe"
$w  = Join-Path $root "crates\heat1d-actor\target\wasm32-unknown-unknown\release\heat1d_actor.wasm"
$wp = Join-Path $root "crates\heat1d-pinn-actor\target\wasm32-unknown-unknown\release\heat1d_pinn_actor.wasm"
$work = Join-Path $env:TEMP "dyne-regress12"
if (Test-Path $work) { Remove-Item -Recurse -Force $work }

# 12シナリオ(第1段=切り出し時と同じ構成。CSVは相対名で出し、実行ディレクトリごと比較する)
$scenarios = [ordered]@{
    "s01_basic16"        = @("--wasm", $w, "--nodes", "16", "--points", "65", "--steps", "2000")
    "s02_chaos16"        = @("--wasm", $w, "--nodes", "16", "--points", "65", "--steps", "2000", "--loss", "0.10", "--lat-ms", "5", "--lat-jitter-ms", "3")
    "s03_pause"          = @("--wasm", $w, "--nodes", "8", "--points", "65", "--steps", "2000", "--pause", "3@20000:40000")
    "s04_partition_loss" = @("--wasm", $w, "--nodes", "8", "--points", "65", "--steps", "2000", "--partition", "4@60000:90000", "--loss", "0.05", "--lat-ms", "2", "--lat-jitter-ms", "2")
    "s05_handover"       = @("--wasm", $w, "--nodes", "8", "--points", "65", "--steps", "2000", "--handover", "3@30000:60000")
    "s06_add_resid"      = @("--wasm", $w, "--nodes", "4", "--points", "65", "--steps", "2000", "--add", "3@30000", "--residual-csv", "resid.csv")
    "s07_add_chaos_ho"   = @("--wasm", $w, "--nodes", "8", "--points", "65", "--steps", "2000", "--add", "5@30000", "--handover", "2@50000:70000", "--loss", "0.10", "--lat-ms", "4", "--lat-jitter-ms", "5")
    "s08_csv4"           = @("--wasm", $w, "--nodes", "8", "--points", "65", "--steps", "500", "--event-csv", "ev.csv", "--field-csv", "field.csv", "--field-every", "50", "--error-csv", "err.csv")
    "s09_add_autogrow"   = @("--wasm", $w, "--nodes", "8", "--points", "65", "--steps", "2000", "--add", "4@30000", "--auto-grow", "on", "--event-csv", "ag_ev.csv", "--residual-csv", "ag_resid.csv")
    "s10_pinn_basic"     = @("--wasm", $wp, "--nodes", "4", "--points", "65", "--steps", "300", "--ram-cap-kb", "4096")
    "s11_pinn_resize"    = @("--wasm", $wp, "--nodes", "4", "--points", "65", "--steps", "300", "--ram-cap-kb", "4096", "--resize", "1@10000:24")
    "s12_pinn_add_ag"    = @("--wasm", $wp, "--nodes", "4", "--points", "65", "--steps", "300", "--ram-cap-kb", "4096", "--add", "3@8000", "--auto-grow", "on")
}

$allOk = $true
foreach ($name in $scenarios.Keys) {
    $sArgs = $scenarios[$name]
    foreach ($side in "base", "cur") {
        $d = Join-Path $work "$side\$name"
        New-Item -ItemType Directory -Force $d | Out-Null
        Push-Location $d
        if ($side -eq "base") { $exe = $baseExe; $extra = @() }
        else { $exe = $curExe; $extra = @("--unobserved-bc", "dirichlet:0") }  # F1: 従来挙動に固定
        # PS5.1 で stderr をリダイレクトすると ErrorRecord 化するため cmd.exe 経由で捕捉する
        $q = (($extra + $sArgs) | ForEach-Object { '"' + $_ + '"' }) -join " "
        cmd /c "`"$exe`" $q > out.log 2> err.log"
        Pop-Location
    }
    $ok = $true
    $files = Get-ChildItem (Join-Path $work "base\$name") | Select-Object -ExpandProperty Name
    foreach ($f in $files) {
        $curPath = Join-Path $work "cur\$name\$f"
        if ($f -eq "out.log") {
            # F1の設定表示行 (現行側にのみ出る) を除いたコピーで比較する
            $filtered = "$curPath.filtered"
            Get-Content $curPath | Where-Object { $_ -notmatch "^unobserved-bc = " } |
                Set-Content -Encoding utf8 $filtered
            # ベースライン側も同じ読み書きを通して行末・BOM差を消す
            $baseNorm = (Join-Path $work "base\$name\$f") + ".norm"
            Get-Content (Join-Path $work "base\$name\$f") | Set-Content -Encoding utf8 $baseNorm
            fc.exe (Resolve-Path $baseNorm) (Resolve-Path $filtered) | Out-Null
        } else {
            fc.exe /b (Join-Path $work "base\$name\$f") $curPath | Out-Null
        }
        if ($LASTEXITCODE -ne 0) { $ok = $false; Write-Host "  DIFF: $name/$f" }
    }
    if ($ok) { Write-Host "MATCH  $name ($($files.Count) files)" }
    else { Write-Host "MISMATCH $name"; $allOk = $false }
}
if ($allOk) { Write-Host "ALL 12 SCENARIOS MATCH (bit-exact vs a435bf6)" }
else { Write-Host "!! MISMATCH AGAINST BASELINE !!" }

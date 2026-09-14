# 発注3 F1+F2 の手元確認スクリプト (2026-09-14)
# リポジトリのルートで実行する: powershell -ExecutionPolicy Bypass -File scripts\check-f1.ps1
#
# 確認内容と期待結果 (差分法アクターのみ。約2〜3分):
#   [1] ビルド (dyne-sim + heat1d-actor wasm + dyne-core 単体テスト8件)
#   [2] 4キーのビット一致 … β型ADD (s06) で dirichlet:0 / zero_grad / linear_extrap /
#       transparent すべて "RESULT: PASS (bit-exact)"。ウォームスタートでも PASS。
#   [3] F2の効果 … 参加ステップ(378)の界面残差: 白紙=2.395e0、warm=0 (厳密)
#   [4] 部分観測の既知値 (決定論なので完全一致するはず):
#         100ステップ: dirichlet 6.713e-1 / zero_grad 2.348e-1 / transparent 1.517e-1
#         2000ステップ: dirichlet 1.386e-1 / zero_grad 8.076e-1 / transparent 6.772e-1
#       (transparent は浸透深さ√(αt)が未観測幅より小さい間だけ最良: レポート§4)
#   [5] transparent の決定論 (2回実行の全出力一致) と長時間安定 (4000ステップ発散なし)
# 出力先 f1_check_out/ は削除してよい (.gitignore 追加済み)
# 12シナリオの回帰一括確認は scripts\check-baseline12.ps1 (F1対応済み) を使う。

$ErrorActionPreference = "Stop"

Write-Host "== [1] ビルドと単体テスト =="
cargo build --release -p dyne-sim
cargo test -p dyne-core --release
Push-Location crates\heat1d-actor
cargo build --release --target wasm32-unknown-unknown
Pop-Location

$sim = "target\release\dyne-sim.exe"
$w = "crates\heat1d-actor\target\wasm32-unknown-unknown\release\heat1d_actor.wasm"
$out = "f1_check_out"
if (Test-Path $out) { Remove-Item -Recurse -Force $out }
New-Item -ItemType Directory -Force $out | Out-Null

# PS5.1 で stderr をリダイレクトすると ErrorRecord 化するため cmd.exe 経由で実行する
function Invoke-Sim([string[]]$simArgs) {
    $q = ($simArgs | ForEach-Object { '"' + $_ + '"' }) -join " "
    cmd /c "`"$sim`" --wasm `"$w`" $q 2> nul"
}

Write-Host "== [2] 4キーのビット一致 (すべて PASS (bit-exact) のこと) =="
foreach ($bc in "dirichlet:0", "zero_grad", "linear_extrap", "transparent") {
    $r = (Invoke-Sim @("--nodes", "4", "--points", "65", "--steps", "2000", "--add", "3@30000",
        "--unobserved-bc", $bc) | Select-String "RESULT").Line
    Write-Host ("  {0,-14} {1}" -f $bc, $r)
}
$r = (Invoke-Sim @("--nodes", "8", "--points", "65", "--steps", "2000", "--add", "5@30000",
    "--join-init", "warm") | Select-String "RESULT").Line
Write-Host "  warm(両隣あり)  $r"

Write-Host "== [3] F2: 参加ステップの界面残差 (白紙=2.395e0、warm=0) =="
foreach ($ji in "blank", "warm") {
    Invoke-Sim @("--nodes", "4", "--points", "65", "--steps", "2000", "--add", "3@30000",
        "--join-init", $ji, "--residual-csv", "$out\resid_$ji.csv") | Out-Null
    $line = (Get-Content $out\resid_$ji.csv | Select-String "^2,378,").Line
    Write-Host "  join-init=$ji : $line"
}

Write-Host "== [4] 部分観測の既知値照合 =="
foreach ($steps in 100, 2000) {
    foreach ($bc in "dirichlet:0", "zero_grad", "transparent") {
        $r = (Invoke-Sim @("--nodes", "8", "--points", "65", "--steps", "$steps",
            "--absent", "0,1,6,7", "--unobserved-bc", $bc) |
            Select-String "^partial observation" | Select-Object -First 1).Line
        Write-Host ("  steps={0,-5} {1,-12} {2}" -f $steps, $bc, $r)
    }
}

Write-Host "== [5] transparent の決定論と長時間安定 =="
foreach ($d in "a", "b") {
    New-Item -ItemType Directory -Force $out\det_$d | Out-Null
    cmd /c "`"$sim`" --wasm `"$w`" --nodes 4 --points 65 --steps 2000 --add 3@30000 --unobserved-bc transparent --residual-csv $out\det_$d\resid.csv --error-csv $out\det_$d\err.csv > $out\det_$d\out.log 2> $out\det_$d\err.log"
}
foreach ($f in "out.log", "resid.csv", "err.csv") {
    fc.exe /b $out\det_a\$f $out\det_b\$f
}
$r = (Invoke-Sim @("--nodes", "8", "--points", "65", "--steps", "4000", "--absent", "0,1,6,7",
    "--unobserved-bc", "transparent") | Select-String "RESULT").Line
Write-Host "  4000ステップ (期待: DONE ... 7.510e-1、発散なし): $r"

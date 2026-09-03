# CSV出力の検証スクリプト(発注6 第1段の手元確認用)
# リポジトリのルートで実行する: powershell -ExecutionPolicy Bypass -File scripts\check-csv.ps1
# 内容: CSV出力つきシナリオ2本を各2回実行し、全CSVのビット一致(決定論)を fc で確認する。
# 期待結果: 実行4回すべて "RESULT: PASS (bit-exact)"、fc は5組とも「相違点は検出されませんでした」。
# 注: ev.csv(基本シナリオ)はヘッダのみで正常(イベントCSVは resize/auto-grow/handover/ADD の
#     動的再構成イベントを記録するため、それらが無いシナリオでは行が出ない)。
#     ag_ev.csv には ADD と auto-grow のイベント行が入る。

$ErrorActionPreference = "Stop"
$sim = "target\release\dyne-sim.exe"
$w = "crates\heat1d-actor\target\wasm32-unknown-unknown\release\heat1d_actor.wasm"

New-Item -ItemType Directory -Force check1, check2 | Out-Null

foreach ($d in "check1", "check2") {
    & $sim --wasm $w --nodes 8 --points 65 --steps 500 `
        --event-csv $d\ev.csv --field-csv $d\field.csv --field-every 50 --error-csv $d\err.csv
    & $sim --wasm $w --nodes 8 --points 65 --steps 2000 --add 4@30000 --auto-grow on `
        --event-csv $d\ag_ev.csv --residual-csv $d\ag_resid.csv
}

foreach ($f in "ev.csv", "field.csv", "err.csv", "ag_ev.csv", "ag_resid.csv") {
    fc.exe /b check1\$f check2\$f
}

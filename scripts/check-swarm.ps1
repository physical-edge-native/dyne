# 発注6(群れデモのDyne土台移植)検収の手元確認スクリプト (2026-09-06)
# リポジトリのルートで実行する: powershell -ExecutionPolicy Bypass -File scripts\check-swarm.ps1
#
# 確認内容と期待結果:
#   [1] ビルドと単体テスト … dyne-core 5件 / dyne-wire 5件 / swarm-agent-actor 4件 すべて ok
#   [2] 既存回帰の抜き取り … dyne-sim 16ノード基本・カオスとも "RESULT: PASS (bit-exact)"
#   [3] 新ホストの決定論   … 平常・混沌注入とも2回実行で全4出力が fc「相違点は検出されませんでした」
#   [4] 既知値の照合(決定論なので完全一致するはず):
#         新実装 gossip seed=1  → final: in-band=98.3%
#         新実装 gossip seed=42 → final: in-band=63.3%
#         新実装 gossip seed=42 + 損失20%+遅延30±20ms → final: in-band=58.3%、actor_dropped=0
#         旧実装 gossip seed=42 → final: in-band=93.3%
#       (シード50本の分布比較の判定は docs/2026-09-04-swarm-port-report.md §7)
# 出力先 swarm_sim_out/ swarm_old_out/ は .gitignore 済み(確認後は削除してよい)

$ErrorActionPreference = "Stop"

Write-Host "== [1] ビルドと単体テスト =="
cargo build --release
cargo test -p dyne-core -p dyne-wire --release
Push-Location crates\swarm-agent-actor
cargo test --release
cargo build --release --target wasm32-unknown-unknown
Pop-Location

$sim = "target\release\dyne-sim.exe"
$hw  = "crates\heat1d-actor\target\wasm32-unknown-unknown\release\heat1d_actor.wasm"
$ss  = "target\release\dyne-swarm-sim.exe"
$old = "target\release\dyne-swarm.exe"
$aw  = "crates\swarm-agent-actor\target\wasm32-unknown-unknown\release\swarm_agent_actor.wasm"

Write-Host "== [2] 既存回帰の抜き取り(PASS (bit-exact) が2回出ること)=="
& $sim --wasm $hw --nodes 16 --points 65 --steps 2000 | Select-Object -Last 1
& $sim --wasm $hw --nodes 16 --points 65 --steps 2000 --loss 0.10 --lat-ms 5 --lat-jitter-ms 3 | Select-Object -Last 1

Write-Host "== [3] 新ホストの決定論(fc 8組とも相違なしのこと)=="
foreach ($d in "a", "b") {
    & $ss --wasm $aw --mode gossip --n 60 --radius 0.15 --steps 800 --seed 1 `
        --out-dir swarm_sim_out\det_$d --quiet | Out-Null
    & $ss --wasm $aw --mode gossip --n 60 --radius 0.15 --steps 800 --seed 42 `
        --loss 0.2 --lat-ms 30 --lat-jitter-ms 20 --out-dir swarm_sim_out\chaos_$d --quiet | Out-Null
}
foreach ($f in "traj.csv", "metrics.csv", "field.csv", "summary.txt") {
    fc.exe /b swarm_sim_out\det_a\$f swarm_sim_out\det_b\$f
    fc.exe /b swarm_sim_out\chaos_a\$f swarm_sim_out\chaos_b\$f
}

Write-Host "== [4] 既知値の照合 =="
& $ss --wasm $aw --mode gossip --n 60 --radius 0.15 --steps 800 --seed 42 `
    --out-dir swarm_sim_out\g42 --quiet | Out-Null
& $old --mode gossip --n 60 --radius 0.15 --steps 800 --seed 42 --out-dir swarm_old_out\g42 --quiet | Out-Null
Write-Host "新 seed=1  (期待 98.3%):" (Select-String "in-band" swarm_sim_out\det_a\summary.txt).Line
Write-Host "新 seed=42 (期待 63.3%):" (Select-String "in-band" swarm_sim_out\g42\summary.txt).Line
Write-Host "新 seed=42 混沌 (期待 58.3%):" (Select-String "in-band" swarm_sim_out\chaos_a\summary.txt).Line
Write-Host "同 net統計 (期待 actor_dropped=0):" (Select-String "actor_dropped" swarm_sim_out\chaos_a\summary.txt).Line
Write-Host "旧 seed=42 (期待 93.3%):" (Select-String "in-band" swarm_old_out\g42\summary.txt).Line

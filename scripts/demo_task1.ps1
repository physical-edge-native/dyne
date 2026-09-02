# Task-1 demo (2026-08-29): 8 nodes, node 4 joins mid-run (beta semantics), auto-GROW on interface residual,
# then build the dashboard HTML. Run from the repository root. Takes about 4 minutes.
# Requires: target\release\dyne-sim.exe and the heat1d-pinn-actor wasm already built.
$sim  = ".\target\release\dyne-sim.exe"
$wasm = ".\crates\heat1d-pinn-actor\target\wasm32-unknown-unknown\release\heat1d_pinn_actor.wasm"
$out  = "demo_out"
New-Item -ItemType Directory -Force $out | Out-Null
& $sim --wasm $wasm --ram-cap-kb 4096 --nodes 8 --points 65 --steps 1200 --add 4@31800 --auto-grow on `
    --residual-csv "$out\resid.csv" --error-csv "$out\err.csv" --event-csv "$out\ev.csv" --field-csv "$out\field.csv" `
    2>&1 | Tee-Object -FilePath "$out\sim.log"
python .\scripts\dashboard.py --field "$out\field.csv" --residual "$out\resid.csv" --event "$out\ev.csv" `
    --error "$out\err.csv" --log "$out\sim.log" --title "Dyne demo: 8 nodes, node 4 joins, auto-GROW" `
    --out "$out\dashboard.html"
Write-Host "done: open $out\dashboard.html in a browser"

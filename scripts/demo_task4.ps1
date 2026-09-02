# Task-4b demo (2026-08-29): swarm in a complex 2-D temperature field with 3 separate optimal spots and a trap hill.
# 60 individuals start scattered uniformly at random. Each senses only its own local temperature and talks only
# to its k=6 nearest neighbours within radius 0.15. Four rules are run and compared:
#   gossip    (share position+temperature+gradient estimate, plus "best site I know" relayed hop by hop)  <- main
#   consensus (share position+temperature, agree on local gradient; no relayed knowledge)
#   follow    (share temperature only, move toward the best neighbour)
#   solo      (no communication, control group)
# Run from the repository root. Takes a few seconds. Requires target\release\dyne-swarm.exe (cargo build --release -p dyne-swarm).
$sw  = ".\target\release\dyne-swarm.exe"
$out = "demo_out\swarm"
foreach ($m in @("gossip", "consensus", "follow", "solo")) {
    & $sw --mode $m --n 60 --radius 0.15 --steps 800 --out-dir "$out\$m" --quiet
}
python .\scripts\swarm_anim.py --dir "$out\gossip" `
    --compare "consensus (gradient agreement only)=$out\consensus" `
    --compare "follow (temperature only)=$out\follow" `
    --compare "solo (no communication)=$out\solo" `
    --title "Dyne demo: swarm finds the optimal temperature spots with local sensing and neighbour talk only" `
    --out "$out\swarm.html"
Write-Host "done: open $out\swarm.html in a browser"

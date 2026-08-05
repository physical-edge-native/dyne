#!/bin/sh
# M0 一括実行: 単一ノード参照解 → 2ノード分割協調推論 → ビット一致検証
set -e
HOST=target/release/dyne-host
WASM=crates/heat1d-actor/target/wasm32-unknown-unknown/release/heat1d_actor.wasm
OUT=${OUT:-/tmp}

$HOST --wasm $WASM --label single --n 65 --x0 0.0 --x1 1.0 \
  --bc-left dirichlet --bc-right dirichlet \
  --alpha 1.0 --dt 1e-4 --steps 2000 --out $OUT/single.csv

$HOST --wasm $WASM --label nodeA --n 33 --x0 0.0 --x1 0.5 \
  --bc-left dirichlet --bc-right interface \
  --listen 127.0.0.1:7801 --peer-right 127.0.0.1:7802 \
  --alpha 1.0 --dt 1e-4 --steps 2000 --out $OUT/nodeA.csv &
$HOST --wasm $WASM --label nodeB --n 32 --x0 0.515625 --x1 1.0 \
  --bc-left interface --bc-right dirichlet \
  --listen 127.0.0.1:7802 --peer-left 127.0.0.1:7801 \
  --alpha 1.0 --dt 1e-4 --steps 2000 --out $OUT/nodeB.csv &
wait

python3 - << PYEOF
import csv
def load(p):
    with open(p) as f:
        return {round(float(r["x"]),9): float(r["u_numeric"]) for r in csv.DictReader(f)}
s = load("$OUT/single.csv"); d = load("$OUT/nodeA.csv"); d.update(load("$OUT/nodeB.csv"))
diffs = [x for x in s if s[x] != d[x]]
mx = max((abs(s[x]-d[x]) for x in s), default=0.0)
print(f"分割解 vs 単一ノード解: 差のある点 {len(diffs)}/{len(s)}個, 最大差 {mx:.3e}")
assert not diffs, "ビット一致検証に失敗"
print("M0 検証合格")
PYEOF

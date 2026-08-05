# M0検証: 分割解(nodeA+nodeB)と単一ノード解のビット一致を確認する
# 使い方: python compare_m0.py <single.csv> <nodeA.csv> <nodeB.csv>
import csv, sys

def load(p):
    with open(p) as f:
        return {round(float(r["x"]), 9): float(r["u_numeric"]) for r in csv.DictReader(f)}

single = load(sys.argv[1])
dist = load(sys.argv[2])
dist.update(load(sys.argv[3]))

assert set(single) == set(dist), "格子点が一致しません"
diffs = [x for x in single if single[x] != dist[x]]
mx = max((abs(single[x] - dist[x]) for x in single), default=0.0)
print(f"分割解 vs 単一ノード解: 差のある点 {len(diffs)}/{len(single)}個, 最大差 {mx:.3e}")
assert not diffs, "ビット一致検証に失敗"
print("M0 検証合格")

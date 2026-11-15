"""sense_test.c が書いた imu_fused の JSON を、scripts/imu_log.py と同じ json.loads で読めるか・値が正確に戻るかを確かめる
(2026-11-13)。使い方は sense_test.c の冒頭。"""
import json
import math
import sys

d = json.loads(open(sys.argv[1], "rb").read())
v = d["v"]
want = [i * 0.1 - 1.0 for i in range(22)]
want[0], want[3], want[8], want[9], want[10], want[11] = 41.0, 9.80665, math.nan, math.inf, -math.inf, 1e-300
ok = d["step"] == 420 and len(v) == 22 and int(v[0]) == 41
for a, b in zip(v, want):
    ok &= (math.isnan(a) and math.isnan(b)) or a == b  # %.17g なので f64 がビット単位で戻る
print("PASS" if ok else "FAIL: %r" % d)
sys.exit(0 if ok else 1)

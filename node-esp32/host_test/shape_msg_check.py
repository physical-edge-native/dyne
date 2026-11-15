"""shape_msg_test.c が書いた区画のメッセージを scripts/dyne_shape_msg.py で読み、組み立て直してバイト一致を確かめる
(2026-11-13)。使い方は shape_msg_test.c の冒頭。リポジトリの .venv の Python で実行する。"""
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "scripts"))
import dyne_shape_msg as m  # noqa: E402

# shape_msg_test.c の tc[] と同じ条件
CASES = [
    (0, 0, 0, 0, 0, 1, 1, 0, 0, 0.0, 0.0, 0.0),
    (7, 123, 1731456789123456, 10, 20, 5, 4, 73, 55, 0.05, 0.0, 0.0),
    (13, 599, 1731456789999999, 109, 81, 36, 27, 145, 109, 0.025, -0.125, 0.25),
    (31, 4000000000, 0xFFFFFFFFFFFF, 0, 0, 18, 27, 0, 0, 0.025, 0.0, 0.0),
]


def main(path):
    data = open(path, "rb").read()
    pos, c, fail = 0, 0, 0
    while pos < len(data):
        (n,) = struct.unpack_from("<I", data, pos)
        msg = data[pos + 4:pos + 4 + n]
        pos += 4 + n
        node, step, t_us, ix0, iy0, nx, ny, gnx, gny, h, x0, y0 = CASES[c]
        k = np.arange(nx * ny, dtype=float)
        phi = ((k * 0.37 - 3.1) * (c + 1)).astype(np.float32).reshape(ny, nx)
        d = (k * 0.01 - 0.05).astype(np.float32).reshape(ny, nx)
        w = ((k % 7) * 0.5).astype(np.float32).reshape(ny, nx)
        u = m.unpack(msg)  # 形式・CRC の検査を含む
        ok = (u["node"], u["step"], u["t_us"], u["ix0"], u["iy0"], u["nx"], u["ny"], u["gnx"], u["gny"]) == \
             (node, step, t_us, ix0, iy0, nx, ny, gnx, gny)
        ok &= np.array_equal(u["phi"].astype(np.float32), phi) and np.array_equal(u["d"].astype(np.float32), d) \
            and np.array_equal(u["w"].astype(np.float32), w)
        ref = m.pack(node, step, t_us, ix0, iy0, phi, d, w, gnx=gnx, gny=gny, h=h, x0=x0, y0=y0)
        same = ref == msg
        print("tile %d: %d bytes, fields %s, bytes %s" % (c, n, "OK" if ok else "NG", "identical" if same else "DIFFER"))
        fail += (not ok) + (not same)
        c += 1
    if c != len(CASES):
        print("tile count %d != %d" % (c, len(CASES)))
        fail += 1
    print("FAIL" if fail else "PASS")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))

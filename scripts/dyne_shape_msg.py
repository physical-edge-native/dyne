#!/usr/bin/env python3
"""ノードの形の場(φ・d・w)の 1 区画ぶんを PC へ送る MQTT のメッセージ(形の区画の書き出し、版 1)。2026-11-13。

定義の正本は docs/2026-11-13-pc-pipeline-rl.md の「形の区画のメッセージ」。この台本はその PC 側の
組み立て・読み取りの参照実装(scripts/twin_fake_nodes.py が組み立て、scripts/twin_live.py が読む)。
ファームウェア(node-esp32/main/node_v2.c)はまだ送らない(実装はファームウェアの担当)。

トピック: dyne/node/<ノード番号>/shape   QoS 0、retain なし、ペイロードはバイナリ(リトルエンディアン)
  位置 大きさ 型    中身
  0    4     char  識別 "DSF1"
  4    1     u8    版 = 1
  5    1     u8    値の符号化 = 1(φ・d・w を f32 で 3 面。将来 2 = 量子化などを足す)
  6    2     u16   見出しの長さ = 52
  8    2     u16   ノード番号(設定注入の node.idx)
  10   2     u16   予約 = 0
  12   4     u32   ステップ(この値を計算し終えたステップ番号)
  16   8     u64   時刻印 [µs](時刻同期した時計。同期前は 0)
  24   2     u16   ix0  区画の左下の格子点の、全体の格子での番号(x)
  26   2     u16   iy0  同(y)
  28   2     u16   nx   区画の格子点の数(x)
  30   2     u16   ny   同(y)
  32   2     u16   gnx  全体の格子の点の数(x)。分からなければ 0
  34   2     u16   gny  同(y)
  36   4     f32   h    格子の刻み [m]
  40   4     f32   x0   全体の格子の原点(格子点 (0,0) の位置)x [m]
  44   4     f32   y0   同 y [m]
  48   4     u32   本体の CRC-32(IEEE 802.3、zlib.crc32 と同じ)
  52   …     本体: φ[nx·ny]、d[nx·ny]、w[nx·ny](各 f32、格子点 k = j·nx + i の順。アクターの dyne_get(k)・
             dyne_get_aux(AUX_D, k)・dyne_get_aux(AUX_W, k) をそのまま f32 にしたもの)
"""
import struct
import zlib

import numpy as np

MAGIC = b"DSF1"
VERSION = 1
ENC_F32 = 1
HDR = struct.Struct("<4sBBHHHIQHHHHHHfffI")
assert HDR.size == 52


def topic(node_id):
    return "dyne/node/%d/shape" % node_id


def pack(node_id, step, t_us, ix0, iy0, phi, d, w, gnx=0, gny=0, h=0.0, x0=0.0, y0=0.0):
    """phi・d・w は (ny, nx) の配列"""
    phi = np.asarray(phi, "<f4")
    ny, nx = phi.shape
    body = phi.tobytes() + np.asarray(d, "<f4").reshape(ny, nx).tobytes() + np.asarray(w, "<f4").reshape(ny, nx).tobytes()
    head = HDR.pack(MAGIC, VERSION, ENC_F32, HDR.size, node_id, 0, step & 0xFFFFFFFF, t_us, ix0, iy0, nx, ny,
                    gnx, gny, h, x0, y0, zlib.crc32(body) & 0xFFFFFFFF)
    return head + body


def unpack(buf):
    """戻り値 dict(node, step, t_us, ix0, iy0, nx, ny, gnx, gny, h, x0, y0, phi, d, w)。形式の誤りは ValueError"""
    if len(buf) < HDR.size:
        raise ValueError("短すぎる (%d バイト)" % len(buf))
    (magic, ver, enc, hlen, node, _r, step, t_us, ix0, iy0, nx, ny, gnx, gny, h, x0, y0, crc) = HDR.unpack_from(buf, 0)
    if magic != MAGIC:
        raise ValueError("識別が違う %r" % magic)
    if ver != VERSION or enc != ENC_F32:
        raise ValueError("対応していない版 %d / 符号化 %d" % (ver, enc))
    n = nx * ny
    body = bytes(buf[hlen:])
    if len(body) != 12 * n:
        raise ValueError("本体の長さ %d が nx·ny·12 = %d と違う" % (len(body), 12 * n))
    if zlib.crc32(body) & 0xFFFFFFFF != crc:
        raise ValueError("CRC が合わない")
    a = np.frombuffer(body, "<f4").astype(float).reshape(3, ny, nx)
    return {"node": node, "step": step, "t_us": t_us, "ix0": ix0, "iy0": iy0, "nx": nx, "ny": ny,
            "gnx": gnx, "gny": gny, "h": h, "x0": x0, "y0": y0, "phi": a[0], "d": a[1], "w": a[2]}


if __name__ == "__main__":
    # 自己検査
    rng = np.random.default_rng(1)
    phi, d, w = rng.normal(size=(3, 4, 5))
    b = pack(7, 123, 456789, 10, 20, phi, d, w, gnx=73, gny=55, h=0.05, x0=0.0, y0=0.0)
    m = unpack(b)
    assert m["node"] == 7 and m["step"] == 123 and m["nx"] == 5 and m["ny"] == 4 and m["gnx"] == 73
    assert np.allclose(m["phi"], phi.astype(np.float32)) and np.allclose(m["w"], w.astype(np.float32))
    bad = bytearray(b)
    bad[-1] ^= 1
    try:
        unpack(bytes(bad))
        raise SystemExit("FAIL: CRC の誤りを見逃した")
    except ValueError:
        pass
    print("PASS: %d バイト(見出し %d + 本体 %d)" % (len(b), HDR.size, len(b) - HDR.size))

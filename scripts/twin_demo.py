#!/usr/bin/env python3
"""部屋のデジタルツインのデモ出力を作る (3次元表示の HTML と USD ファイル)。2026-11-08。

何をするか:
  dyne-sim2d が書き出す形状の格子型アクター(測距から符号つき距離場を作るアクター)の
  CSV を読み、推定された表面(符号つき距離 phi のゼロ等値線)を格子全体で
  マーチングスクエア法(格子の各升で等値線の通り道を求める方法)により取り出して
  折れ線につなぎ、壁・家具として高さ方向に押し出す。そこに温度場を床面の色として重ね、
    (A) <out>.usda … USD の ASCII 形式(Z 軸が上、1 単位 = 1 m、1 質量単位 = 1 kg)。推定形状の見た目・
        衝突判定(UsdPhysics。床・壁・家具。2026-11-11 追加)・正解形状(比較用)・
        ノードとセンサ(衝突判定なし)・温度場の床(頂点ごとの色と temperature の値)を含む。
        構成と prim のパスは twin_physics.py の冒頭を参照。pxr(usd-core)で書く
        2026-11-13: 衝突判定の既定は箱の基本形(Cube)。未観測(w = 0)を物とみなす箱 /Room/Collision/Unobserved を
        既定で足す(--unobserved-solid off で外す)。--scene は省略可(図面なしは格子の範囲を部屋とみなす)。
        --json で表示のデータ <out>.json も書く(常駐版 twin_live.py のダッシュボードの自動更新が読む)
    (A2) <out>.xml … 同じ衝突判定の形を MuJoCo の模型(MJCF)にしたもの(--no-mjcf で省略)
    (A3) <out>_truth.usda … 場面ファイルの正解の形を同じ構成で書いたもの(--truth-usd のとき)
    (B) <out>.html … 自己完結の 3 次元ダッシュボード(外部の配信元に頼らずオフラインで開ける。
        WebGL。左: 構成の木、中央: 3 次元表示、右: 選んだ要素の属性、下: 記録と統計)
  を出力する。数値計算に numpy を使う(リポジトリの .venv)。

何が本物で、何がダミーか:
  - 本物(Dyne の計算そのもの): 形状。測距フレームから各ノードの WASM アクターが
    計算し、ノード間でやり取りして合わせた符号つき距離場 phi と重み w。
  - 理想化した入力: 測距フレームはシミュレータが正解の部屋から模擬生成したもの(雑音・
    区画数・視野は dyne-sim2d の引数どおり)。
  - ダミー: 温度場。暖房(壁の発熱面)と窓(冷たい面)を置いた定常の熱拡散を部屋の格子
    (家具は熱を通さない)で解いた架空の場。ノードの読み値はその場の値をそのまま
    取ったもの(理想の温度センサ)。Dyne の計算ではない。
  - 家具の高さ: 平面(2 次元)の計算なので高さの情報はない。推定表面の裏側にある場面ファイルの
    家具の高さ(box の 5 番目の値)を使い、外壁は天井まで伸ばす。

使い方:
  python scripts/twin_demo.py --scene crates/dyne-sim2d/scenes/room6.txt \
      --shape-csv shape.csv --sim-log sim.log --out demo_out/twin/room
  (一括実行は scripts/twin_demo.ps1)
"""
import argparse
import csv
import json
import math
import os
import re

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
import sys  # noqa: E402

sys.path.insert(0, HERE)
import twin_physics  # noqa: E402

# ---------------------------------------------------------------- 場面ファイル


def parse_scene(path):
    sc = {"room": [], "polys": [], "heights": [], "names": [], "circles": [], "sensors": []}

    def rect(v):
        x0, y0, x1, y1 = min(v[0], v[2]), min(v[1], v[3]), max(v[0], v[2]), max(v[1], v[3])
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    with open(path, encoding="utf-8") as f:
        for raw in f:
            parts = raw.split("#", 1)
            body = parts[0].strip()
            note = parts[1].strip() if len(parts) > 1 else ""
            if not body:
                continue
            t = body.split()
            k = t[0]
            if k in ("movebox", "move"):
                # 動く箱(実験6の場面)は正解の形に入れない。推定では「その他」(Other)になる(ユーザー決定 B2)
                continue
            v = [float(s) for s in t[1:]]
            label = note.split("(")[0].split("(")[0].strip()
            if k == "room":
                sc["room"] = rect(v)
            elif k == "roompoly":
                sc["room"] = list(zip(v[0::2], v[1::2]))
            elif k == "box":
                sc["polys"].append(rect(v[:4]))
                sc["heights"].append(v[4] if len(v) > 4 else None)
                sc["names"].append(label)
            elif k == "poly":
                sc["polys"].append(list(zip(v[0::2], v[1::2])))
                sc["heights"].append(None)
                sc["names"].append(label)
            elif k == "circle":
                sc["circles"].append(tuple(v[:3]))
            elif k == "sensor":
                sc["sensors"].append({"id": int(v[0]), "x": v[1], "y": v[2],
                                      "yaw": math.radians(v[3]), "height": v[4] if len(v) > 4 else 0.0})
    return sc


def ring_edges(p):
    return [(p[i], p[(i + 1) % len(p)]) for i in range(len(p))]


def scene_edges(sc):
    out = ring_edges(sc["room"])
    for p in sc["polys"]:
        out += ring_edges(p)
    return out


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            inside = not inside
    return inside


def obstacle_at(sc, x, y):
    """(x, y) を含む遮蔽物の番号(多角形)。円は -2、なければ -1"""
    for k, p in enumerate(sc["polys"]):
        if point_in_poly(x, y, p):
            return k
    for cx, cy, r in sc["circles"]:
        if math.hypot(x - cx, y - cy) < r:
            return -2
    return -1


def is_free(sc, x, y):
    return point_in_poly(x, y, sc["room"]) and obstacle_at(sc, x, y) == -1


def raycast(sc, ox, oy, vx, vy):
    """scene.rs の raycast と同じ計算(説明用の光線を引くため)"""
    best = None
    for (ax, ay), (bx, by) in scene_edges(sc):
        ex, ey = bx - ax, by - ay
        den = vx * ey - vy * ex
        if abs(den) < 1e-15:
            continue
        wx, wy = ax - ox, ay - oy
        t = (wx * ey - wy * ex) / den
        u = (wx * vy - wy * vx) / den
        if 0.0 <= u <= 1.0 and t > 1e-9 and (best is None or t < best):
            best = t
    for cx, cy, r in sc["circles"]:
        px, py = ox - cx, oy - cy
        b = px * vx + py * vy
        c = px * px + py * py - r * r
        disc = b * b - c
        if disc >= 0:
            sq = math.sqrt(disc)
            for t in (-b - sq, -b + sq):
                if t > 1e-9 and (best is None or t < best):
                    best = t
    return best


def zone_offset(c, cols, fov):
    """dyne_wire::zone_angle_offset と同じ(区画 c の中心方向の、正面からのずれ)"""
    return fov * ((cols - 1) * 0.5 - c) / cols


def truth_segments(sc):
    segs = [np.array(e, float) for e in scene_edges(sc)]
    for cx, cy, r in sc["circles"]:
        n = 24
        pts = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
        segs += [np.array(e, float) for e in ring_edges(pts)]
    return np.array(segs)  # (S, 2, 2)


def dist_to_segments(pts, segs):
    """点の列 (N,2) から線分の集合 (S,2,2) までの最短距離 (N,)"""
    a = segs[:, 0, :][None]
    e = (segs[:, 1, :] - segs[:, 0, :])[None]
    p = pts[:, None, :]
    l2 = np.maximum((e * e).sum(-1), 1e-18)
    t = np.clip(((p - a) * e).sum(-1) / l2, 0.0, 1.0)
    q = a + t[..., None] * e
    return np.sqrt(((p - q) ** 2).sum(-1)).min(1)


# ---------------------------------------------------------------- 形状 CSV


def read_shape(path):
    data = {}
    with open(path, newline="", encoding="utf-8") as f:
        rd = csv.DictReader(f)
        rows = list(rd)
    gx = np.array([int(r["gx"]) for r in rows])
    gy = np.array([int(r["gy"]) for r in rows])
    nx, ny = gx.max() + 1, gy.max() + 1
    for key in ("phi", "w", "d", "x", "y"):
        a = np.zeros((ny, nx))
        a[gy, gx] = [float(r[key]) for r in rows]
        data[key] = a
    has_truth = "truth" in rows[0] and rows[0]["truth"] != ""
    data["truth"] = None
    if has_truth:
        a = np.zeros((ny, nx))
        a[gy, gx] = [float(r["truth"]) for r in rows]
        data["truth"] = a
    data["nx"], data["ny"] = int(nx), int(ny)
    data["x0"], data["y0"] = float(data["x"][0, 0]), float(data["y"][0, 0])
    data["h"] = float(data["x"][0, 1] - data["x"][0, 0])
    return data


def marching_squares(g, mu):
    """phi のゼロ等値線を、観測済み (w>0) で |phi| が mu 程度以下の升だけで取り出す。
    戻り値: 線分の列 [((x0,y0),(x1,y1), key0, key1)]。key は升の辺の識別子(つなぐため)"""
    h = g["h"]
    # 外周の外に 1 列ずつ仮の点を足す (外壁は格子の端に乗るので、端の外へ線形に外挿して等値線を拾う)
    phi = np.pad(g["phi"], 1, mode="reflect", reflect_type="odd")
    w = np.pad(g["w"], 1, mode="edge")
    ny, nx = phi.shape
    X0, Y0 = g["x0"] - h, g["y0"] - h
    ok = (w > 0) & (np.abs(phi) < 2.0 * mu)
    segs = []

    def cross(ka, kb, pa, pb, fa, fb):
        t = fa / (fa - fb)
        return (pa[0] + t * (pb[0] - pa[0]), pa[1] + t * (pb[1] - pa[1]))

    for j in range(ny - 1):
        for i in range(nx - 1):
            if not (ok[j, i] and ok[j, i + 1] and ok[j + 1, i + 1] and ok[j + 1, i]):
                continue
            f = [phi[j, i], phi[j, i + 1], phi[j + 1, i + 1], phi[j + 1, i]]
            s = [v < 0 for v in f]
            if all(s) or not any(s):
                continue
            p = [(X0 + i * h, Y0 + j * h), (X0 + (i + 1) * h, Y0 + j * h),
                 (X0 + (i + 1) * h, Y0 + (j + 1) * h), (X0 + i * h, Y0 + (j + 1) * h)]
            keys = [("h", i, j), ("v", i + 1, j), ("h", i, j + 1), ("v", i, j)]
            pts = {}
            for e in range(4):
                a, b = e, (e + 1) % 4
                if s[a] != s[b]:
                    pts[e] = cross(keys[e], None, p[a], p[b], f[a], f[b])
            es = sorted(pts)
            if len(es) == 2:
                segs.append((pts[es[0]], pts[es[1]], keys[es[0]], keys[es[1]]))
            elif len(es) == 4:
                c = sum(f) / 4
                # 鞍点: 中心の符号で組み合わせを選ぶ
                if (c < 0) == s[0]:
                    pairs = [(0, 1), (2, 3)]
                else:
                    pairs = [(3, 0), (1, 2)]
                for a_, b_ in pairs:
                    segs.append((pts[a_], pts[b_], keys[a_], keys[b_]))
    return segs


def chain(segs):
    """線分を端点の識別子でつないで折れ線の列にする(各折れ線は線分番号の列)"""
    adj = {}
    for k, (_, _, ka, kb) in enumerate(segs):
        adj.setdefault(ka, []).append(k)
        adj.setdefault(kb, []).append(k)
    used = [False] * len(segs)
    lines = []
    for start in range(len(segs)):
        if used[start]:
            continue
        used[start] = True
        line = [start]
        for direction in (1, 0):
            cur = start
            key = segs[start][3] if direction else segs[start][2]
            while True:
                nxt = [k for k in adj.get(key, []) if not used[k]]
                if not nxt:
                    break
                k = nxt[0]
                used[k] = True
                line = line + [k] if direction else [k] + line
                key = segs[k][3] if segs[k][2] == key else segs[k][2]
                cur = k
        lines.append(line)
    return lines


def furniture_cap(g, s2, top, bounds):
    """家具の上面(表示用): 推定した面の外接長方形の中で、phi が負か未観測の升を高さ top に敷く"""
    xs = [c for (p, q) in s2 for c in (p[0], q[0])]
    ys = [c for (p, q) in s2 for c in (p[1], q[1])]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    h = g["h"]
    tris = []
    for j in range(g["ny"] - 1):
        for i in range(g["nx"] - 1):
            cx, cy = g["x0"] + (i + 0.5) * h, g["y0"] + (j + 0.5) * h
            if not (x0 - 0.3 * h <= cx <= x1 + 0.3 * h and y0 - 0.3 * h <= cy <= y1 + 0.3 * h):
                continue
            ph = g["phi"][j:j + 2, i:i + 2]
            ww = g["w"][j:j + 2, i:i + 2]
            if (ph.mean() <= 0) or (ww.max() <= 0):
                a = (max(cx - h / 2, x0), max(cy - h / 2, y0))
                b = (min(cx + h / 2, x1), min(cy + h / 2, y1))
                if b[0] <= a[0] or b[1] <= a[1]:
                    continue
                p0, p1, p2, p3 = (a[0], a[1], top), (b[0], a[1], top), (b[0], b[1], top), (a[0], b[1], top)
                tris += [(p0, p1, p2), (p0, p2, p3)]
    return tris


def phi_grad(g, x, y):
    h = g["h"]
    i = int(round((x - g["x0"]) / h))
    j = int(round((y - g["y0"]) / h))
    i = min(max(i, 1), g["nx"] - 2)
    j = min(max(j, 1), g["ny"] - 2)
    gx = (g["phi"][j, i + 1] - g["phi"][j, i - 1]) / (2 * h)
    gy = (g["phi"][j + 1, i] - g["phi"][j - 1, i]) / (2 * h)
    n = math.hypot(gx, gy) or 1.0
    return gx / n, gy / n


# ---------------------------------------------------------------- 温度(ダミー)


def dummy_temperature(sc, bounds, step, heater, window, t_heater, t_window, t_amb, decay):
    """【ダミー】定常の熱拡散 -∇²T + (T - T_amb)/L² = 0 を部屋の格子で解く。
    暖房・窓は外壁に接する固定温度の帯、家具は熱を通さない(断熱)。ヤコビ反復"""
    x0, y0, x1, y1 = bounds
    nx = int(round((x1 - x0) / step)) + 1
    ny = int(round((y1 - y0) / step)) + 1
    xs = x0 + step * np.arange(nx)
    ys = y0 + step * np.arange(ny)
    free = np.zeros((ny, nx), bool)
    for j in range(ny):
        for i in range(nx):
            # 境界上の点は少し内側で判定
            px = min(max(xs[i], x0 + 1e-6), x1 - 1e-6)
            py = min(max(ys[j], y0 + 1e-6), y1 - 1e-6)
            free[j, i] = is_free(sc, px, py)
    fixed = np.zeros((ny, nx), bool)
    val = np.full((ny, nx), t_amb)

    def band(seg, t):
        (ax, ay), (bx, by) = seg
        for j in range(ny):
            for i in range(nx):
                ex, ey = bx - ax, by - ay
                l2 = ex * ex + ey * ey
                u = max(0.0, min(1.0, ((xs[i] - ax) * ex + (ys[j] - ay) * ey) / l2))
                if math.hypot(ax + u * ex - xs[i], ay + u * ey - ys[j]) < 0.6 * step:
                    fixed[j, i] = True
                    val[j, i] = t
    band(heater, t_heater)
    band(window, t_window)
    fixed &= free
    T = np.full((ny, nx), t_amb)
    T[fixed] = val[fixed]
    k = step * step / (decay * decay)
    for _ in range(8000):
        P = np.pad(T, 1, mode="edge")
        F = np.pad(free, 1, constant_values=False)
        s = np.zeros_like(T)
        for dj, di in ((0, 1), (2, 1), (1, 0), (1, 2)):
            nb = P[dj:dj + ny, di:di + nx]
            nf = F[dj:dj + ny, di:di + nx]
            s += np.where(nf, nb, T)  # 断熱: 熱を通さない隣は自分の値
        Tn = (s + k * t_amb) / (4 + k)
        Tn[fixed] = val[fixed]
        Tn[~free] = T[~free]
        if np.abs(Tn - T).max() < 1e-7:
            T = Tn
            break
        T = Tn
    # 家具の中は周りの値でならす(表示用)
    for _ in range(200):
        P = np.pad(T, 1, mode="edge")
        avg = (P[0:ny, 1:nx + 1] + P[2:ny + 2, 1:nx + 1] + P[1:ny + 1, 0:nx] + P[1:ny + 1, 2:nx + 2]) / 4
        T[~free] = avg[~free]
    return {"nx": nx, "ny": ny, "x0": x0, "y0": y0, "step": step, "T": T, "free": free}


def sample_temp(tf, x, y):
    u = (x - tf["x0"]) / tf["step"]
    v = (y - tf["y0"]) / tf["step"]
    i = min(max(int(u), 0), tf["nx"] - 2)
    j = min(max(int(v), 0), tf["ny"] - 2)
    s, t = u - i, v - j
    T = tf["T"]
    return float((1 - s) * (1 - t) * T[j, i] + s * (1 - t) * T[j, i + 1] + s * t * T[j + 1, i + 1] + (1 - s) * t * T[j + 1, i])


# 色: 低温=紺〜青、中間=白に近い灰、高温=橙〜赤(発散型)
CMAP = [(0.0, (0.13, 0.25, 0.60)), (0.25, (0.27, 0.53, 0.82)), (0.5, (0.86, 0.87, 0.88)),
        (0.75, (0.94, 0.58, 0.32)), (1.0, (0.72, 0.13, 0.15))]


def color_map(t, tmin, tmax):
    s = 0.0 if tmax <= tmin else max(0.0, min(1.0, (t - tmin) / (tmax - tmin)))
    for k in range(len(CMAP) - 1):
        a, ca = CMAP[k]
        b, cb = CMAP[k + 1]
        if s <= b:
            u = (s - a) / (b - a)
            return tuple(ca[i] + u * (cb[i] - ca[i]) for i in range(3))
    return CMAP[-1][1]


# ---------------------------------------------------------------- 幾何(3 次元)


def extrude(segs2d, z0, z1):
    """2 次元の線分の列を z0〜z1 に押し出した三角形の列 [(p,q,r), ...]"""
    tris = []
    for (ax, ay), (bx, by) in segs2d:
        a0, b0, b1, a1 = (ax, ay, z0), (bx, by, z0), (bx, by, z1), (ax, ay, z1)
        tris += [(a0, b0, b1), (a0, b1, a1)]
    return tris


def prism(poly, z0, z1, top=True):
    tris = extrude(ring_edges(poly), z0, z1)
    if top:
        p = [(x, y, z1) for x, y in poly]
        for k in range(1, len(p) - 1):
            tris.append((p[0], p[k], p[k + 1]))
    return tris


def box_tris(cx, cy, cz, sx, sy, sz, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    corners = []
    for dz in (-sz / 2, sz / 2):
        for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)):
            corners.append((cx + c * dx - s * dy, cy + s * dx + c * dy, cz + dz))
    q = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    tris = []
    for a, b, cc, d in q:
        tris += [(corners[a], corners[b], corners[cc]), (corners[a], corners[cc], corners[d])]
    return tris


def flat(tris):
    return [round(v, 4) for t in tris for p in t for v in p]


def fmt(v):
    return ("%.4f" % v).rstrip("0").rstrip(".") if v != 0 else "0"


# ---------------------------------------------------------------- 記録の読み取り

NODE_RE = re.compile(r"^node\s+(\d+): \[(\d+)\.\.(\d+)\)x\[(\d+)\.\.(\d+)\) ifaces \[([^\]]*)\] sent (\d+) \((\d+) B\) lost (\d+) resend (\d+).*?sensors \[([^\]]*)\]")


def read_log(path):
    lines, nodes = [], []
    if path and os.path.exists(path):
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.rstrip("\n")
                lines.append(line)
                m = NODE_RE.match(line)
                if m:
                    nodes.append({"id": int(m.group(1)), "i0": int(m.group(2)), "i1": int(m.group(3)),
                                  "j0": int(m.group(4)), "j1": int(m.group(5)),
                                  "ifaces": [int(s) for s in m.group(6).split(",") if s.strip()],
                                  "sent": int(m.group(7)), "bytes": int(m.group(8)),
                                  "lost": int(m.group(9)), "resend": int(m.group(10)),
                                  "sensors": [int(s) for s in m.group(11).split(",") if s.strip()]})
    return lines, nodes


def find(lines, pat):
    for l in lines:
        m = re.search(pat, l)
        if m:
            return m
    return None


# ---------------------------------------------------------------- USD


def usd_mesh(L, ind, name, tris, color, opacity=None, vcolors=None, temps=None, extra=()):
    pts = [p for t in tris for p in t]
    n = len(tris)
    sp = " " * ind
    L.append(sp + 'def Mesh "%s"' % name)
    L.append(sp + "{")
    L.append(sp + "    uniform bool doubleSided = 1")
    L.append(sp + "    int[] faceVertexCounts = [%s]" % ", ".join(["3"] * n))
    L.append(sp + "    int[] faceVertexIndices = [%s]" % ", ".join(str(i) for i in range(3 * n)))
    L.append(sp + "    point3f[] points = [%s]" % ", ".join("(%s, %s, %s)" % tuple(fmt(c) for c in p) for p in pts))
    if vcolors is None:
        L.append(sp + '    color3f[] primvars:displayColor = [(%s, %s, %s)] (interpolation = "constant")' % tuple(fmt(c) for c in color))
    else:
        L.append(sp + '    color3f[] primvars:displayColor = [%s] (interpolation = "vertex")'
                 % ", ".join("(%s, %s, %s)" % tuple(fmt(c) for c in col) for col in vcolors))
    if temps is not None:
        L.append(sp + '    float[] primvars:temperature = [%s] (interpolation = "vertex")' % ", ".join(fmt(t) for t in temps))
    if opacity is not None:
        L.append(sp + '    float[] primvars:displayOpacity = [%s] (interpolation = "constant")' % fmt(opacity))
    L.append(sp + '    uniform token subdivisionScheme = "none"')
    for e in extra:
        L.append(sp + "    " + e)
    L.append(sp + "}")


def usd_name(s):
    s = re.sub(r"[^A-Za-z0-9_]", "_", s)
    return s if s and not s[0].isdigit() else "_" + s


# ---------------------------------------------------------------- 本体


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=None,
                    help="場面ファイル(実機では実験4の図面を同じ書式で書いたもの)。省略時は格子の範囲を部屋とみなし、"
                         "家具の名前・高さ・正解との比較は無い(2026-11-13)")
    ap.add_argument("--shape-csv", required=True)
    ap.add_argument("--sim-log")
    ap.add_argument("--mu", type=float, default=None, help="表面の近くとみなす |phi| の目安 [m](既定 3h)")
    ap.add_argument("--room-height", type=float, default=2.4)
    ap.add_argument("--furniture-height", type=float, default=0.7, help="場面ファイルに高さの無い家具の高さ [m]")
    ap.add_argument("--zones", type=int, default=64)
    ap.add_argument("--fov-deg", type=float, default=60.0)
    ap.add_argument("--max-range", type=float, default=4.0)
    ap.add_argument("--slice-z", type=float, default=1.2, help="温度の水平断面を置く高さ [m]")
    ap.add_argument("--title", default="Dyne 部屋のデジタルツイン")
    ap.add_argument("--out", required=True, help="出力の接頭辞(.usda と .html を付ける)")
    # 衝突判定(2026-11-11)
    ap.add_argument("--wall-thickness", type=float, default=0.10, help="外壁の衝突判定の板の厚み [m](表面から外側へ)")
    ap.add_argument("--furniture-thickness", type=float, default=0.05, help="家具の衝突判定の板の厚み [m](表面から家具の内側へ)")
    ap.add_argument("--simplify-tol", type=float, default=0.005, help="衝突判定の折れ線を間引くときの許容のずれ [m]")
    ap.add_argument("--floor-margin", type=float, default=0.5, help="床の直方体を格子の外へ広げる幅 [m]")
    ap.add_argument("--approximation", default="none",
                    help="UsdPhysics.MeshCollisionAPI の approximation(none=三角形メッシュのまま。静的なので既定)")
    ap.add_argument("--truth-usd", action="store_true", help="正解の形の USD (<out>_truth.usda) も書く")
    ap.add_argument("--no-mjcf", action="store_true", help="MuJoCo の模型 (<out>.xml) を書かない")
    ap.add_argument("--sequence", type=int, default=0, help="書き出しの通し番号(定期的な書き出しで増やす)")
    ap.add_argument("--timestamp", default=None, help="記録する作成時刻(既定は現在の UTC)")
    # 2026-11-13(docs/2026-11-13-pc-pipeline-rl.md)
    ap.add_argument("--collider-shape", choices=["box", "mesh"], default="box",
                    help="衝突判定の形: box = 板 1 枚を箱の基本形 1 つに(既定。Isaac で近似が要らない)/ mesh = 閉じた三角形メッシュ")
    ap.add_argument("--unobserved-solid", choices=["on", "off"], default="on",
                    help="未観測(w = 0)の格子点を物とみなす衝突判定 /Room/Collision/Unobserved を足す(RL・物理用。見た目は変えない)")
    ap.add_argument("--unobserved-min-w", type=float, default=0.0, help="この重み以下を未観測とみなす")
    ap.add_argument("--json", action="store_true", help="表示のデータを <out>.json にも書く(常駐版のダッシュボードが読む)")
    # 旧版との互換(使わない)
    ap.add_argument("--heat-csv")
    ap.add_argument("--temps")
    a = ap.parse_args()

    g = read_shape(a.shape_csv)
    h = g["h"]
    if a.scene:
        sc = parse_scene(a.scene)
    else:
        # 図面なし: 格子の範囲を部屋とみなす(外壁は格子の端に乗る前提。シミュレータと実機の設定注入の約束)
        gx1_, gy1_ = g["x0"] + (g["nx"] - 1) * h, g["y0"] + (g["ny"] - 1) * h
        sc = {"room": [(g["x0"], g["y0"]), (gx1_, g["y0"]), (gx1_, gy1_), (g["x0"], gy1_)],
              "polys": [], "heights": [], "names": [], "circles": [], "sensors": []}
    scene_name = os.path.basename(a.scene) if a.scene else "(図面なし: 格子の範囲)"
    mu = a.mu or 3 * h
    xs = [p[0] for p in sc["room"]]
    ys = [p[1] for p in sc["room"]]
    bounds = (min(xs), min(ys), max(xs), max(ys))
    log, nodes = read_log(a.sim_log)
    console = []

    def say(s):
        print(s)
        console.append(s)

    # --- 推定表面(ゼロ等値線 → 折れ線)
    segs = marching_squares(g, mu)
    lines = chain(segs)
    say("推定表面: 線分 %d 本 → 折れ線 %d 本" % (len(segs), len(lines)))

    # 線分ごとに「裏側(phi<0 の側)」を調べて、外壁か、どの家具か、を決める
    cats = []
    for (p, q, _, _) in segs:
        mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
        nx_, ny_ = phi_grad(g, mx, my)
        cat = None
        for depth in (0.05, 0.1, 0.03):
            bx, by = mx - nx_ * depth, my - ny_ * depth
            if not point_in_poly(bx, by, sc["room"]):
                cat = "wall"
                break
            k = obstacle_at(sc, bx, by)
            if k >= 0:
                cat = k
                break
            if k == -2:
                cat = "misc"
                break
        if cat is None:
            near = min(mx - bounds[0], bounds[2] - mx, my - bounds[1], bounds[3] - my)
            cat = "wall" if near < 0.1 else "misc"
        cats.append(cat)

    # --- 精度(正解との比較)
    tsegs = truth_segments(sc)
    mids = np.array([((p[0] + q[0]) / 2, (p[1] + q[1]) / 2) for (p, q, _, _) in segs]) if segs else np.zeros((0, 2))
    err = dist_to_segments(mids, tsegs) if len(mids) else np.zeros(0)
    # 正解表面の標本(h おき)。見える面(片側が空き)だけを覆い率の分母にする
    samples, visible = [], []
    for (ax, ay), (bx, by) in scene_edges(sc):
        L_ = math.hypot(bx - ax, by - ay)
        n = max(1, int(L_ / h))
        ex, ey = (bx - ax) / L_, (by - ay) / L_
        for k in range(n):
            t = (k + 0.5) / n
            x, y = ax + t * (bx - ax), ay + t * (by - ay)
            f1 = is_free(sc, x - ey * 2 * h, y + ex * 2 * h)
            f2 = is_free(sc, x + ey * 2 * h, y - ex * 2 * h)
            samples.append((x, y))
            visible.append(f1 != f2)
    samples = np.array(samples)
    visible = np.array(visible)
    allpts = np.array([p for (p, q, _, _) in segs] + [q for (p, q, _, _) in segs]) if segs else np.zeros((0, 2))
    if len(allpts):
        dmin = np.sqrt(((samples[:, None, :] - allpts[None]) ** 2).sum(-1)).min(1)
    else:
        dmin = np.full(len(samples), 1e9)
    covered = dmin < 1.5 * h
    cov_vis = covered[visible].mean() * 100 if visible.any() else 0.0
    cov_all = covered.mean() * 100
    stats = {}
    stats["推定表面と正解の面の距離 平均"] = "%.1f mm" % (err.mean() * 1000) if len(err) else "-"
    stats["同 95%点 / 最大"] = "%.1f mm / %.1f mm" % (np.percentile(err, 95) * 1000, err.max() * 1000) if len(err) else "-"
    stats["見える面の覆い率"] = "%.1f %% (%d / %d 点)" % (cov_vis, covered[visible].sum(), visible.sum())
    stats["隠れた面を含む覆い率"] = "%.1f %%" % cov_all
    say("推定表面と正解の面の距離: 平均 %s、95%%点・最大 %s" % (stats["推定表面と正解の面の距離 平均"], stats["同 95%点 / 最大"]))
    say("覆い率: 見える面 %.1f%%(家具の裏などの隠れた面を含めると %.1f%%)" % (cov_vis, cov_all))

    m = find(log, r"^dyne-sim2d: (\d+)x(\d+) nodes, grid (\d+)x(\d+) \(h = ([0-9.]+) m")
    if m:
        stats["ノード数"] = "%d (%s × %s)" % (int(m.group(1)) * int(m.group(2)), m.group(1), m.group(2))
        stats["格子"] = "%s × %s 点、刻み %s m" % (m.group(3), m.group(4), m.group(5))
    m = find(log, r"^scene: .*\| sensors (\d+) \| zones (\d+) fov ([0-9.]+) deg max-range ([0-9.]+) m noise ([0-9.e-]+) m")
    if m:
        stats["センサ数"] = m.group(1)
        stats["1台の測る方向の数 / 視野"] = "%s 方向 / %s 度" % (m.group(2), m.group(3))
        stats["測距の雑音(理想化した入力)"] = "%.1f mm" % (float(m.group(5)) * 1000)
    m = find(log, r"^shape: surface points (\d+) \| \|phi-truth\| mean ([0-9.]+) m p95 ([0-9.]+) m max ([0-9.]+) m")
    if m:
        stats["表面の格子点の距離の誤差(シミュレータの採点) 平均"] = "%.1f mm (95%%点 %.1f mm)" % (float(m.group(2)) * 1000, float(m.group(3)) * 1000)
    m = find(log, r"^(PASS \(bit-exact\).*|FAIL.*)$")
    stats["分割解と1台の解の一致"] = m.group(1) if m else "-"
    if nodes:
        stats["送信したメッセージ(合計)"] = "%d 通 / %.1f MB" % (sum(n["sent"] for n in nodes), sum(n["bytes"] for n in nodes) / 1e6)
        stats["損失 / 再送(合計)"] = "%d / %d" % (sum(n["lost"] for n in nodes), sum(n["resend"] for n in nodes))
    m = find(log, r"^rays: (.*)$")
    if m:
        stats["光線の遅着・溢れ"] = m.group(1).split("|")[0].strip()
    m = find(log, r"^virtual time: (.*)$")
    if m:
        stats["仮想時間"] = m.group(1)

    # --- 温度場(ダミー)
    W, H = bounds[2] - bounds[0], bounds[3] - bounds[1]
    heater = ((bounds[0], bounds[1] + 0.8), (bounds[0], bounds[1] + 1.8))         # 西の壁の発熱面
    window = ((bounds[0] + 1.3, bounds[3]), (bounds[0] + 2.9, bounds[3]))        # 北の壁の窓
    tf = dummy_temperature(sc, bounds, 0.05, heater, window, 30.0, 12.0, 20.0, 1.2)
    T = tf["T"]
    tmin, tmax = float(np.floor(T.min())), float(np.ceil(T.max()))
    say("温度場(ダミー): 定常の熱拡散、暖房 30 度・窓 12 度・周囲 20 度、範囲 %.1f〜%.1f 度" % (T.min(), T.max()))

    # --- 3 次元の要素(表示用の木構造)
    prims = []

    def add(path, kind, props=None, tris=None, color=(0.7, 0.7, 0.7), alpha=1.0, vcolors=None, lines3=None,
            visible=True, pick=True, label=None):
        prims.append({"path": path, "kind": kind, "label": label or path.split("/")[-1], "props": props or {},
                      "tris": flat(tris) if tris else None, "color": color, "alpha": alpha,
                      "vcol": [round(c, 3) for col in vcolors for c in col] if vcolors else None,
                      "lines": [round(v, 4) for v in lines3] if lines3 else None,
                      "visible": visible, "pick": pick})

    add("/Room", "Xform", {"種類": "部屋(全体)", "外寸": "%.2f m × %.2f m × 高さ %.2f m" % (W, H, a.room_height),
                           "場面ファイル": scene_name})
    # 推定の壁・家具
    add("/Room/Estimated", "Scope", {"説明": "Dyne の分担計算で推定した形(符号つき距離場のゼロ等値線を押し出したもの)",
                                     "線分 / 折れ線": "%d / %d" % (len(segs), len(lines))})
    groups = {}
    for k, cat in enumerate(cats):
        groups.setdefault(cat, []).append(k)
    for cat, ks in sorted(groups.items(), key=lambda kv: str(kv[0])):
        s2 = [(segs[k][0], segs[k][1]) for k in ks]
        e = err[ks]
        if cat == "wall":
            name, top, col = "Walls", a.room_height, (0.80, 0.82, 0.86)
            title = "外壁"
        elif cat == "misc":
            name, top, col = "Other", a.furniture_height, (0.85, 0.70, 0.45)
            title = "その他の表面"
        else:
            hh = sc["heights"][cat] or a.furniture_height
            title = sc["names"][cat] or ("家具%d" % cat)
            name, top, col = "Furniture_%d" % cat, hh, (0.93, 0.62, 0.30)
        tris = extrude(s2, 0.0, top)
        if cat not in ("wall",):
            tris += furniture_cap(g, s2, top, bounds)
        # 上端の線
        l3 = []
        for (p, q) in ([] if cat == "wall" else s2):
            l3 += [p[0], p[1], top, q[0], q[1], top]
        add("/Room/Estimated/" + name, "Mesh",
            {"種類": "推定した面(" + title + ")", "線分の数": len(ks), "高さ": "%.2f m" % top,
             "高さの出所": "天井まで" if cat == "wall" else "場面ファイルの家具の高さ",
             "正解の面との距離 平均": "%.1f mm" % (e.mean() * 1000), "同 最大": "%.1f mm" % (e.max() * 1000)},
            tris=tris, color=col, lines3=l3, label=title)
    # 正解
    add("/Room/GroundTruth", "Scope", {"説明": "比べるための正解の形(場面ファイル)。Dyne には渡していない"}, visible=False)
    wall_tris = extrude(ring_edges(sc["room"]), 0.0, a.room_height)
    add("/Room/GroundTruth/Walls", "Mesh", {"種類": "正解の外壁"}, tris=wall_tris, color=(0.55, 0.75, 0.95), alpha=0.25, visible=False, label="外壁(正解)")
    for k, p in enumerate(sc["polys"]):
        hh = sc["heights"][k] or a.furniture_height
        xs_ = [q[0] for q in p]
        ys_ = [q[1] for q in p]
        add("/Room/GroundTruth/Furniture_%d" % k, "Mesh",
            {"種類": "正解の家具", "名前": sc["names"][k] or "-",
             "範囲 x": "%.2f〜%.2f m" % (min(xs_), max(xs_)), "範囲 y": "%.2f〜%.2f m" % (min(ys_), max(ys_)), "高さ": "%.2f m" % hh},
            tris=prism(p, 0.0, hh), color=(0.55, 0.75, 0.95), alpha=0.35, visible=False,
            label=(sc["names"][k] or "家具%d" % k) + "(正解)")
    # 床
    add("/Room/Floor", "Mesh", {"種類": "床(表示用)"}, tris=[((bounds[0], bounds[1], -0.002), (bounds[2], bounds[1], -0.002), (bounds[2], bounds[3], -0.002)),
                                                         ((bounds[0], bounds[1], -0.002), (bounds[2], bounds[3], -0.002), (bounds[0], bounds[3], -0.002))],
        color=(0.30, 0.31, 0.33), visible=False, label="床")

    # 温度場の床と断面
    step = tf["step"]
    tt, tc = [], []
    for j in range(tf["ny"] - 1):
        for i in range(tf["nx"] - 1):
            P = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
            for tri in ((0, 1, 2), (0, 2, 3)):
                tt.append(tuple((tf["x0"] + P[v][0] * step, tf["y0"] + P[v][1] * step, 0.003) for v in tri))
                tc += [color_map(T[P[v][1], P[v][0]], tmin, tmax) for v in tri]
    tprops = {"種類": "温度場(ダミー)", "注意": "架空の定常熱拡散の解。Dyne の計算ではない",
              "暖房": "西の壁 y=%.1f〜%.1f m、30 度" % (heater[0][1], heater[1][1]),
              "窓": "北の壁 x=%.1f〜%.1f m、12 度" % (window[0][0], window[1][0]),
              "範囲": "%.1f〜%.1f 度" % (T.min(), T.max()), "刻み": "%.2f m" % step}
    add("/Room/TemperatureField", "Scope", tprops, label="温度場(ダミー)")
    add("/Room/TemperatureField/Floor", "Mesh", tprops, tris=tt, vcolors=tc, label="床の温度")
    tt2 = [tuple((p[0], p[1], a.slice_z) for p in t) for t in tt]
    add("/Room/TemperatureField/Slice", "Mesh", dict(tprops, 高さ="%.2f m(同じ値を表示)" % a.slice_z), tris=tt2, vcolors=tc, alpha=0.45,
        visible=False, label="水平断面 %.1f m" % a.slice_z)

    # ノードとセンサ
    sens_by_id = {s["id"]: s for s in sc["sensors"]}
    fov = math.radians(a.fov_deg)
    add("/Room/Nodes", "Scope", {"説明": "マイコン(ノード)。各ノードが自分の区画の形を計算し、隣と境目の値・光線を受け渡す",
                                 "ノード数": len(nodes)}, label="ノード")
    node_lines = []
    readings = []
    for nd in nodes:
        x0n, x1n = g["x0"] + nd["i0"] * h, g["x0"] + (nd["i1"] - 1) * h
        y0n, y1n = g["y0"] + nd["j0"] * h, g["y0"] + (nd["j1"] - 1) * h
        cx, cy = (x0n + x1n) / 2, (y0n + y1n) / 2
        tval = sample_temp(tf, cx, cy)
        readings.append(tval)
        sub_phi = g["w"][nd["j0"]:nd["j1"], nd["i0"]:nd["i1"]]
        node_lines += [x0n, y0n, 0.01, x1n, y0n, 0.01, x1n, y0n, 0.01, x1n, y1n, 0.01,
                       x1n, y1n, 0.01, x0n, y1n, 0.01, x0n, y1n, 0.01, x0n, y0n, 0.01]
        border = []
        for (p, q) in (((x0n, y0n), (x1n, y0n)), ((x1n, y0n), (x1n, y1n)), ((x1n, y1n), (x0n, y1n)), ((x0n, y1n), (x0n, y0n))):
            border += [p[0], p[1], 0.012, q[0], q[1], 0.012]
        add("/Room/Nodes/Node_%d" % nd["id"], "Xform",
            {"種類": "ノード(マイコン)", "番号": nd["id"], "受け持つ区画 x": "%.3f〜%.3f m" % (x0n, x1n),
             "受け持つ区画 y": "%.3f〜%.3f m" % (y0n, y1n), "格子点": "%d × %d" % (nd["i1"] - nd["i0"], nd["j1"] - nd["j0"]),
             "観測済みの格子点": "%.0f %%" % ((sub_phi > 0).mean() * 100),
             "隣との境目": len(nd["ifaces"]), "送信": "%d 通 / %.1f kB" % (nd["sent"], nd["bytes"] / 1e3),
             "損失 / 再送": "%d / %d" % (nd["lost"], nd["resend"]), "センサ": len(nd["sensors"]),
             "温度の読み値(ダミー)": "%.2f 度" % tval},
            tris=box_tris(cx, cy, 0.03, 0.10, 0.07, 0.04), color=(0.36, 0.78, 0.45), lines3=border,
            label="ノード %d" % nd["id"])
        for sid in nd["sensors"]:
            s = sens_by_id.get(sid)
            if not s:
                continue
            z = s["height"] or 0.5
            rays = []
            hits = 0
            for c in range(a.zones):
                th = s["yaw"] + zone_offset(c, a.zones, fov)
                vx, vy = math.cos(th), math.sin(th)
                t = raycast(sc, s["x"], s["y"], vx, vy)
                if t is None or t > a.max_range:
                    t = a.max_range
                else:
                    hits += 1
                rays += [s["x"], s["y"], z, s["x"] + vx * t, s["y"] + vy * t, z]
            add("/Room/Nodes/Node_%d/Sensor_%d" % (nd["id"], sid), "Mesh",
                {"種類": "測距センサ(模擬)", "番号": sid, "位置": "(%.2f, %.2f, %.2f) m" % (s["x"], s["y"], z),
                 "向き": "%.0f 度(x 軸から反時計回り)" % math.degrees(s["yaw"]), "視野": "%.0f 度 / %d 方向" % (a.fov_deg, a.zones),
                 "面に当たる方向": "%d / %d" % (hits, a.zones), "所属ノード": nd["id"]},
                tris=box_tris(s["x"] + 0.03 * math.cos(s["yaw"]), s["y"] + 0.03 * math.sin(s["yaw"]), z, 0.06, 0.035, 0.025, s["yaw"]),
                color=(0.35, 0.62, 0.98), lines3=rays, label="センサ %d" % sid)
    if readings:
        stats["温度の読み値(ダミー、ノード中心)"] = "%.1f〜%.1f 度" % (min(readings), max(readings))

    # --- 衝突判定の形(推定表面 → 間引いた折れ線 → 物の側へ立てた閉じた板)
    fheights = {"misc": a.furniture_height}
    for k in range(len(sc["polys"])):
        fheights[k] = sc["heights"][k] or a.furniture_height
    colliders, cst = twin_physics.build_colliders(
        segs, lines, cats, lambda x, y: phi_grad(g, x, y), fheights, a.room_height,
        a.simplify_tol, a.wall_thickness, a.furniture_thickness)
    say("衝突判定: 表面の線分 %d 本 → 間引き(許容 %.1f mm)後の板 %d 枚(%s)"
        % (cst["segments_in"], a.simplify_tol * 1000, cst["slabs"],
           "、".join("%s %d" % (n, len(colliders[n]["slabs"])) for n in sorted(colliders))))
    gx1, gy1 = g["x0"] + (g["nx"] - 1) * h, g["y0"] + (g["ny"] - 1) * h
    mg = a.floor_margin
    ground_extent = (g["x0"] - mg, g["y0"] - mg, gx1 + mg, gy1 + mg)
    # 未観測を物とみなす(RL・物理の書き出し専用。見た目の /Room/Estimated と /Room/Visual は変えない)
    ust = None
    if a.unobserved_solid == "on":
        uslabs, ust = twin_physics.unobserved_colliders(g["w"], g["x0"], g["y0"], h, a.room_height, a.unobserved_min_w)
        if uslabs:
            colliders["Unobserved"] = {"category": "unobserved", "slabs": uslabs, "height": a.room_height, "thickness": 0.0}
        say("未観測を物とみなす衝突判定: 未観測の格子点 %d 点(%.1f%%)→ 長方形にまとめた箱 %d 個"
            % (ust["cells"], ust["fraction"] * 100, ust["boxes"]))
    add("/Room/Collision", "Scope", {"説明": "物理シミュレータ用の静的な衝突判定(USD の UsdPhysics と MuJoCo の模型に同じ形を書く)",
                                     "作り方": "推定表面の折れ線を許容 %.1f mm で間引き、物の側へ厚みのある板を立てる" % (a.simplify_tol * 1000),
                                     "板の厚み": "外壁 %.2f m / 家具 %.2f m" % (a.wall_thickness, a.furniture_thickness),
                                     "板の数": cst["slabs"],
                                     "形": "箱の基本形(USD の Cube)" if a.collider_shape == "box" else "閉じた三角形メッシュ",
                                     "未観測": ("物とみなす(箱 %d 個)" % ust["boxes"]) if ust else "穴のまま"},
        visible=False, label="衝突判定")
    for name in sorted(colliders):
        V, F = twin_physics.slabs_mesh(colliders[name]["slabs"])
        unob = name == "Unobserved"
        add("/Room/Collision/" + name, "Mesh",
            {"種類": "未観測を物とみなした箱(RL・物理用)" if unob else "衝突判定(閉じた板の集まり)",
             "箱の数" if unob else "板の数": len(colliders[name]["slabs"]), "高さ": "%.2f m" % colliders[name]["height"]},
            tris=[tuple(V[i] for i in f) for f in F], color=(0.45, 0.45, 0.5) if unob else (0.90, 0.30, 0.30),
            alpha=0.35 if unob else 0.55, visible=False, label=("未観測(物とみなす)" if unob else "衝突判定 " + name))
    meta = {"generator": "scripts/twin_demo.py", "source": "dyne-sim2d shape grid actor (signed distance field)",
            "createdUtc": a.timestamp or twin_physics.now_utc(), "sequence": a.sequence,
            "scene": scene_name, "shapeCsv": os.path.basename(a.shape_csv),
            "colliderShape": a.collider_shape, "unobservedSolid": a.unobserved_solid,
            "unobservedBoxes": ust["boxes"] if ust else 0, "unobservedCells": ust["cells"] if ust else 0,
            "gridH": float(h), "gridNx": g["nx"], "gridNy": g["ny"], "gridOrigin": [float(g["x0"]), float(g["y0"])],
            "muM": float(mu), "roomHeightM": float(a.room_height), "furnitureHeightM": float(a.furniture_height),
            "wallThicknessM": float(a.wall_thickness), "furnitureThicknessM": float(a.furniture_thickness),
            "simplifyTolM": float(a.simplify_tol), "meshApproximation": a.approximation,
            "colliderSlabs": cst["slabs"], "surfaceSegments": len(segs),
            "bitExact": stats.get("分割解と1台の解の一致", "-")}
    meta["groundExtent"] = [float(v) for v in ground_extent]

    # --- USD
    write_usda(a.out + ".usda", sc, segs, cats, groups, a, bounds, tf, nodes, sens_by_id, g, h, tmin, tmax, err,
               colliders, meta, ground_extent)
    say("USD: " + a.out + ".usda")
    if a.truth_usd:
        write_truth_usda(a.out + "_truth.usda", sc, a, meta, ground_extent)
        say("USD(正解の形): " + a.out + "_truth.usda")
    if not a.no_mjcf:
        twin_physics.write_mjcf(a.out + ".xml", colliders, ground_extent,
                                comment="Dyne room colliders, generated by scripts/twin_demo.py at %s from %s (sequence %d)"
                                % (meta["createdUtc"], meta["scene"], a.sequence),
                                grid_extent=(g["x0"], g["y0"], gx1, gy1), room_height=a.room_height)
        say("MJCF: " + a.out + ".xml")

    # --- HTML
    data = {"title": a.title, "bounds": bounds, "roomH": a.room_height, "prims": prims, "stats": stats,
            "log": log, "console": console, "tmin": tmin, "tmax": tmax,
            "cmap": [[s, list(c)] for s, c in CMAP],
            "stamp": {"sequence": a.sequence, "createdUtc": meta["createdUtc"]}}
    if a.json:
        with open(a.out + ".json", "w", encoding="utf-8", newline="\n") as f:
            json.dump(data, f, ensure_ascii=False, separators=(",", ":"))
        say("JSON: " + a.out + ".json")
    with open(os.path.join(HERE, "twin_demo_view.html"), encoding="utf-8") as f:
        tpl = f.read()
    out = tpl.replace("/*__DATA__*/null", json.dumps(data, ensure_ascii=False, separators=(",", ":")))
    out = out.replace("__TITLE__", a.title)
    with open(a.out + ".html", "w", encoding="utf-8", newline="\n") as f:
        f.write(out)
    say("HTML: " + a.out + ".html")


def visual_soup(tris):
    """三角形の列を頂点・面の配列にする(見た目用。面積が 0 の三角形は除く)"""
    V, F = [], []
    for t in tris:
        a, b, c = (np.array(p, float) for p in t)
        if np.linalg.norm(np.cross(b - a, c - a)) < 1e-10:
            continue
        o = len(V)
        V += [tuple(p) for p in t]
        F.append((o, o + 1, o + 2))
    return V, F


def write_usda(path, sc, segs, cats, groups, a, bounds, tf, nodes, sens_by_id, g, h, tmin, tmax, err,
               colliders, meta, ground_extent):
    """USD(物理のスキーマつき)を書く。構成は twin_physics.py の冒頭を参照"""
    doc = ("Dyne 部屋のデジタルツイン(デモ)。Visual は格子型アクターの符号つき距離場のゼロ等値線を押し出したもの、"
           "Collision は同じ表面から物の側へ立てた閉じた板(静的な衝突判定)。"
           "測距は理想化した模擬入力、TemperatureField はダミー(架空の定常熱拡散)。")
    W = twin_physics.UsdRoomWriter(doc, meta, ground_extent)
    # 見た目(推定)
    for cat, ks in sorted(groups.items(), key=lambda kv: str(kv[0])):
        s2 = [(segs[k][0], segs[k][1]) for k in ks]
        if cat == "wall":
            name, top, col = "Walls", a.room_height, (0.8, 0.82, 0.86)
        elif cat == "misc":
            name, top, col = "Other", a.furniture_height, (0.85, 0.7, 0.45)
        else:
            name, top, col = "Furniture_%d" % cat, sc["heights"][cat] or a.furniture_height, (0.93, 0.62, 0.3)
        tris = extrude(s2, 0.0, top) + ([] if cat == "wall" else furniture_cap(g, s2, top, bounds))
        V, F = visual_soup(tris)
        W.mesh("/Room/Visual/" + name, V, F, color=col,
               custom={"dyne:meanErrorM": ("Double", float(err[ks].mean())), "dyne:segments": ("Int", len(ks))})
    # 衝突判定(推定)
    for name in sorted(colliders):
        ent = colliders[name]
        if not ent["slabs"]:
            continue
        W.colliders("/Room/Collision/" + name, ent["slabs"], shape=a.collider_shape, approximation=a.approximation,
                    custom={"dyne:slabs": ("Int", len(ent["slabs"])), "dyne:heightM": ("Double", float(ent["height"])),
                            "dyne:thicknessM": ("Double", float(ent["thickness"])),
                            "dyne:category": ("String", str(ent["category"]))})
    # 正解(比較用。見た目のみ、既定で非表示)
    gt = W.UsdGeom.Scope.Define(W.stage, "/Room/Reference/GroundTruth")
    gt.CreateVisibilityAttr(W.UsdGeom.Tokens.invisible)
    V, F = visual_soup(extrude(ring_edges(sc["room"]), 0.0, a.room_height))
    W.mesh("/Room/Reference/GroundTruth/Walls", V, F, color=(0.55, 0.75, 0.95), opacity=0.25)
    for k, p in enumerate(sc["polys"]):
        V, F = twin_physics.closed_prism(list(p), 0.0, sc["heights"][k] or a.furniture_height)
        W.mesh("/Room/Reference/GroundTruth/Furniture_%d" % k, V, F, color=(0.55, 0.75, 0.95), opacity=0.35)
    # 温度場の床(格子の頂点を共有する四角形のメッシュ。床の衝突判定の上面 z=0 と重ならないよう 3 mm 上げる)
    nx, ny, step = tf["nx"], tf["ny"], tf["step"]
    pts, cols, temps, idx = [], [], [], []
    for j in range(ny):
        for i in range(nx):
            pts.append((tf["x0"] + i * step, tf["y0"] + j * step, 0.003))
            t = float(tf["T"][j, i])
            temps.append(t)
            cols.append(color_map(t, tmin, tmax))
    for j in range(ny - 1):
        for i in range(nx - 1):
            q = j * nx + i
            idx += [q, q + 1, q + 1 + nx, q + nx]
    W.UsdGeom.Scope.Define(W.stage, "/Room/TemperatureField")
    W.mesh("/Room/TemperatureField/Floor", pts, idx, counts=[4] * ((nx - 1) * (ny - 1)), vcolors=cols,
           primvars={"temperature": (temps, W.UsdGeom.Tokens.vertex)},
           custom={"dyne:note": ("String", "ダミーの温度場(摂氏)。架空の定常熱拡散の解で、実測でも Dyne の計算でもない")})
    # ノードとセンサの目印(衝突判定なし)
    Gf, UsdGeom = W.Gf, W.UsdGeom
    UsdGeom.Scope.Define(W.stage, "/Room/Markers/Nodes")
    for nd in nodes:
        x0n, x1n = g["x0"] + nd["i0"] * h, g["x0"] + (nd["i1"] - 1) * h
        y0n, y1n = g["y0"] + nd["j0"] * h, g["y0"] + (nd["j1"] - 1) * h
        cx, cy = (x0n + x1n) / 2, (y0n + y1n) / 2
        base = "/Room/Markers/Nodes/Node_%d" % nd["id"]
        x = UsdGeom.Xform.Define(W.stage, base)
        UsdGeom.XformCommonAPI(x).SetTranslate(Gf.Vec3d(cx, cy, 0.03))
        pr = x.GetPrim()
        pr.CreateAttribute("dyne:nodeId", W.Sdf.ValueTypeNames.Int, custom=True).Set(nd["id"])
        pr.CreateAttribute("dyne:regionMin", W.Sdf.ValueTypeNames.Double2, custom=True).Set(Gf.Vec2d(x0n, y0n))
        pr.CreateAttribute("dyne:regionMax", W.Sdf.ValueTypeNames.Double2, custom=True).Set(Gf.Vec2d(x1n, y1n))
        pr.CreateAttribute("dyne:temperatureDummy", W.Sdf.ValueTypeNames.Double, custom=True).Set(sample_temp(tf, cx, cy))
        body = UsdGeom.Cube.Define(W.stage, base + "/Body")
        body.CreateSizeAttr(0.06)
        body.CreateDisplayColorAttr([Gf.Vec3f(0.36, 0.78, 0.45)])
        for sid in nd["sensors"]:
            s = sens_by_id.get(sid)
            if not s:
                continue
            z = s["height"] or 0.5
            c = UsdGeom.Cube.Define(W.stage, base + "/Sensor_%d" % sid)
            c.CreateSizeAttr(0.04)
            c.AddTranslateOp().Set(Gf.Vec3d(s["x"] - cx, s["y"] - cy, z - 0.03))
            c.AddRotateZOp().Set(math.degrees(s["yaw"]))
            c.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.62, 0.98)])
            sp = c.GetPrim()
            sp.CreateAttribute("dyne:sensorId", W.Sdf.ValueTypeNames.Int, custom=True).Set(sid)
            sp.CreateAttribute("dyne:fovDeg", W.Sdf.ValueTypeNames.Double, custom=True).Set(a.fov_deg)
            sp.CreateAttribute("dyne:zones", W.Sdf.ValueTypeNames.Int, custom=True).Set(a.zones)
    W.save(path)


def write_truth_usda(path, sc, a, meta, ground_extent):
    """場面ファイルの正解の形を、推定と同じ構成(Visual / Collision)の USD に書く(比較用)"""
    doc = "Dyne 部屋のデジタルツインの比較用の正解の形(場面ファイルから。Dyne の計算ではない)。構成は推定の USD と同じ。"
    W = twin_physics.UsdRoomWriter(doc, dict(meta, kind="ground-truth"), ground_extent)
    walls, furn = twin_physics.truth_colliders(sc, a.room_height, a.furniture_height, a.wall_thickness)
    V, F = visual_soup(extrude(ring_edges(sc["room"]), 0.0, a.room_height))
    W.mesh("/Room/Visual/Walls", V, F, color=(0.55, 0.75, 0.95))
    W.colliders("/Room/Collision/Walls", walls, shape=a.collider_shape, approximation=a.approximation)
    for name, ((V, F), label) in furn.items():
        W.mesh("/Room/Visual/" + name, V, F, color=(0.55, 0.75, 0.95), double_sided=False,
               custom={"dyne:label": ("String", label or "-")})
        xs_, ys_ = [v[0] for v in V], [v[1] for v in V]
        rect = len(V) == 8 and len(set(round(x, 9) for x in xs_)) == 2 and len(set(round(y, 9) for y in ys_)) == 2
        if a.collider_shape == "box" and rect:
            W.collider_boxes("/Room/Collision/" + name, [twin_physics.box_slab(min(xs_), min(ys_), max(xs_), max(ys_),
                                                                              min(v[2] for v in V), max(v[2] for v in V))])
        else:
            # 箱でない家具(多角形・円柱)は凸の閉じたメッシュ。Isaac でも使えるよう凸包の近似を指定する
            W.collider_mesh("/Room/Collision/" + name, V, F,
                            approximation="convexHull" if a.collider_shape == "box" else a.approximation)
    W.save(path)


if __name__ == "__main__":
    main()

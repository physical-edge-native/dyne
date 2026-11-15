#!/usr/bin/env python3
"""推定した部屋の形から、物理シミュレータで使える衝突判定の形を作り、USD と MJCF に書き出す。2026-11-11。

twin_demo.py から呼ばれる部品(単独では実行しない)。

何をするか:
  1. マーチングスクエア法で取り出した表面の線分(twin_demo.py の segs)を折れ線にし、
     分類(外壁 / 家具k / その他)が変わる所で切る。
  2. 折れ線を Ramer-Douglas-Peucker 法(元の線からのずれが許容値以下になるよう頂点を間引く方法)で
     間引く。
  3. 間引いた線分ごとに、表面から物の側(符号つき距離 phi が負の側)へ厚み t の「板」(直方体)を
     立てる。板は 1 枚ずつが閉じた凸の立体なので、
       - USD: 分類ごとに 1 つの Mesh(閉じた直方体の集まり)にまとめ、UsdPhysics の衝突判定を付ける
       - MJCF(MuJoCo の模型の記述形式): 板 1 枚を box の geom 1 つにする
     のどちらにもそのまま使える(MuJoCo は mesh の geom を凸包で扱うので、凸の部品に分けておく必要がある)。
  4. 床は厚み 0.1 m の直方体(上面が z=0)。

2026-11-13 の変更(docs/2026-11-13-pc-pipeline-rl.md):
  - 衝突判定の既定を「板 1 枚 = 箱の基本形 1 つ」(UsdGeom.Cube + 位置・z 軸回りの回転・大きさ + CollisionAPI)
    にした。Isaac Sim(PhysX)は三角形メッシュの衝突判定を静的な物にしか使えず、近似の指定に依存するので、
    最初から近似の要らない箱にしておく(ユーザー決定 B3)。従来の閉じたメッシュは --collider-shape mesh で残す。
  - 未観測(統合の重み w = 0)の格子点を「物がある」とみなす衝突判定(Unobserved)を足せるようにした
    (RL・物理の書き出し専用。見た目は変えない。ユーザー決定 B1/F2)。格子の上で隣り合う未観測の点を
    貪欲法で長方形にまとめ、箱の数を減らす。

USD の構成(prim のパスは定期的に書き出し直しても変えない):
  /Room                         defaultPrim。Xform、kind=assembly。customData に作り方の記録(dyne)
  /Room/Visual/<名前>           見た目の形(従来の押し出した面。両面表示、衝突判定なし)
  /Room/Collision/Floor         床(Cube、衝突判定あり、見える)
  /Room/Collision/<名前>        壁・家具の衝突判定。既定は Xform の下に箱 Box_0000, Box_0001, …(Cube、見えない)。
                                --collider-shape mesh のときは Mesh(閉じた板の集まり、見えない)
  /Room/Collision/Unobserved    未観測を物とみなした箱(--unobserved-solid on のとき)。箱の番号は書き出しごとに
                                変わりうる(親のパスは変わらない)
  /Room/PhysicsMaterials/Default 摩擦・反発の物性(UsdPhysics.MaterialAPI)
  /Room/TemperatureField/Floor  温度場(ダミー)。衝突判定なし
  /Room/Markers/Nodes/Node_i    ノードとセンサの目印。衝突判定なし
  /Room/Reference/GroundTruth   比較用の正解の形(見た目のみ、既定で非表示)
  /PhysicsScene                 重力。defaultPrim の外に置く(参照で取り込む側の物理の場と二重にならないように)
  <名前> は Walls / Furniture_<場面ファイルの家具の番号> / Other。
"""
import datetime
import math

import numpy as np

# ---------------------------------------------------------------- 折れ線

def seg_polylines(segs, lines):
    """chain() の結果(線分番号の列)を点列にする。戻り値 [(点の列, 閉じているか, 線分番号の列)]。
    点の列の i 番目と i+1 番目を結ぶのが線分番号の列の i 番目(閉じていれば最後の点から最初の点へも)"""
    out = []
    for line in lines:
        if len(line) == 1:
            p, q, _, _ = segs[line[0]]
            out.append(([p, q], False, list(line)))
            continue

        def shared(a, b):
            s = {segs[a][2], segs[a][3]} & {segs[b][2], segs[b][3]}
            return next(iter(s)) if s else None

        def other(k, key):
            return segs[k][3] if segs[k][2] == key else segs[k][2]

        def kp(k, key):
            return segs[k][0] if segs[k][2] == key else segs[k][1]

        first_key = other(line[0], shared(line[0], line[1]))
        pts = [kp(line[0], first_key)]
        for a, b in zip(line, line[1:]):
            pts.append(kp(a, shared(a, b)))
        last_key = other(line[-1], shared(line[-2], line[-1]))
        pts.append(kp(line[-1], last_key))
        closed = len(line) > 2 and last_key == first_key
        if closed:
            pts = pts[:-1]
        out.append((pts, closed, list(line)))
    return out


def split_by_category(polys, cats):
    """折れ線を分類の変わり目で切る。戻り値 [(分類, 点の列, 閉じているか)]"""
    runs = []
    for pts, closed, ks in polys:
        c = [cats[k] for k in ks]
        n = len(ks)
        if all(x == c[0] for x in c):
            runs.append((c[0], list(pts), closed))
            continue
        if closed:
            # 分類の変わり目から始まるように回す
            s = next(i for i in range(n) if c[i] != c[i - 1])
            ks_idx = list(range(s, n)) + list(range(0, s))
            P = [pts[i % len(pts)] for i in range(s, s + n + 1)]  # 点 n+1 個(最後=最初)
        else:
            ks_idx = list(range(n))
            P = list(pts)
        cur, start = c[ks_idx[0]], 0
        for m in range(1, n + 1):
            if m == n or c[ks_idx[m]] != cur:
                runs.append((cur, P[start:m + 1], False))
                if m < n:
                    cur, start = c[ks_idx[m]], m
    return runs


def rdp(pts, tol):
    """Ramer-Douglas-Peucker 法(開いた折れ線)"""
    P = np.asarray(pts, float)
    if len(P) <= 2:
        return [tuple(p) for p in P]
    keep = np.zeros(len(P), bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(P) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = P[i], P[j]
        e = b - a
        L = math.hypot(*e)
        q = P[i + 1:j] - a
        if L < 1e-12:
            d = np.hypot(q[:, 0], q[:, 1])
        else:
            d = np.abs(q[:, 0] * e[1] - q[:, 1] * e[0]) / L
        k = int(np.argmax(d))
        if d[k] > tol:
            m = i + 1 + k
            keep[m] = True
            stack += [(i, m), (m, j)]
    return [tuple(p) for p in P[keep]]


def simplify(pts, closed, tol):
    """閉じた折れ線は、最初の点と最も遠い点で 2 つに分けてから間引く。戻り値は線分 [(p, q)] の列"""
    if closed:
        P = np.asarray(pts, float)
        far = int(np.argmax(((P - P[0]) ** 2).sum(1)))
        a = rdp(list(pts[:far + 1]), tol)
        b = rdp(list(pts[far:]) + [pts[0]], tol)
        ring = a[:-1] + b[:-1]
        return [(ring[i], ring[(i + 1) % len(ring)]) for i in range(len(ring))]
    s = rdp(pts, tol)
    return list(zip(s[:-1], s[1:]))


# ---------------------------------------------------------------- 板(閉じた直方体)

def slab_from_segment(p, q, solid_dir, t, z0, z1):
    """表面の線分 p→q から、物の側(solid_dir との内積が正の側)へ厚み t の板を立てる。
    戻り値 dict(4 隅の多角形・反時計回り、z0, z1、MJCF 用の中心・半寸法・向き)"""
    dx, dy = q[0] - p[0], q[1] - p[1]
    L = math.hypot(dx, dy)
    nx, ny = -dy / L, dx / L
    if nx * solid_dir[0] + ny * solid_dir[1] < 0:
        nx, ny = -nx, -ny
    poly = [p, q, (q[0] + t * nx, q[1] + t * ny), (p[0] + t * nx, p[1] + t * ny)]
    area = sum(poly[i][0] * poly[(i + 1) % 4][1] - poly[(i + 1) % 4][0] * poly[i][1] for i in range(4))
    if area < 0:
        poly = poly[::-1]
    cx = (p[0] + q[0]) / 2 + nx * t / 2
    cy = (p[1] + q[1]) / 2 + ny * t / 2
    return {"poly": poly, "z0": z0, "z1": z1, "center": (cx, cy, (z0 + z1) / 2),
            "half": (L / 2, t / 2, (z1 - z0) / 2), "yaw": math.atan2(dy, dx), "normal": (nx, ny)}


def closed_prism(poly, z0, z1):
    """反時計回りの凸でも凹でもよい単純多角形を押し出した閉じた立体(頂点を共有する三角形)。
    上下の面は扇形に分けるので、凸の多角形に限って正しい。戻り値 (頂点の列, 三角形の頂点番号の列)"""
    n = len(poly)
    area = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
    if area < 0:
        poly = poly[::-1]
    V = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    F = []
    for i in range(n):
        j = (i + 1) % n
        F += [(i, j, n + j), (i, n + j, n + i)]            # 側面(外向き)
    for k in range(1, n - 1):
        F.append((n, n + k, n + k + 1))                      # 上面(+z)
        F.append((0, k + 1, k))                              # 下面(-z)
    return V, F


def merge(parts):
    """閉じた立体の列 [(V, F)] を 1 つの頂点・面の配列にまとめる"""
    V, F = [], []
    for v, f in parts:
        o = len(V)
        V += v
        F += [(a + o, b + o, c + o) for a, b, c in f]
    return V, F


def build_colliders(segs, lines, cats, phi_grad_fn, heights, room_height, tol, t_wall, t_furn, min_len=1e-3):
    """推定表面から、分類ごとの板の列を作る。
    heights: 分類(家具の番号 / "misc")→ 高さ。戻り値 {名前: {"slabs": [...], "category": ...}} と統計"""
    polys = seg_polylines(segs, lines)
    runs = split_by_category(polys, cats)
    out = {}
    n_in, n_out, n_short = 0, 0, 0
    for cat, pts, closed in runs:
        n_in += len(pts) - (0 if closed else 1)
        if cat == "wall":
            name, top, t = "Walls", room_height, t_wall
        elif cat == "misc":
            name, top, t = "Other", heights["misc"], t_furn
        else:
            name, top, t = "Furniture_%d" % cat, heights[cat], t_furn
        ent = out.setdefault(name, {"category": cat, "slabs": [], "height": top, "thickness": t})
        for p, q in simplify(pts, closed, tol):
            if math.hypot(q[0] - p[0], q[1] - p[1]) < min_len:
                n_short += 1
                continue
            gx, gy = phi_grad_fn((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
            ent["slabs"].append(slab_from_segment(p, q, (-gx, -gy), t, 0.0, top))
            n_out += 1
    stats = {"runs": len(runs), "segments_in": n_in, "slabs": n_out, "dropped_short": n_short}
    return out, stats


def truth_colliders(sc, room_height, furniture_height, t_wall):
    """場面ファイルの正解の形から衝突判定を作る(比較用)。壁は部屋の多角形の外側へ板、家具は閉じた柱"""
    room = sc["room"]
    n = len(room)
    area = sum(room[i][0] * room[(i + 1) % n][1] - room[(i + 1) % n][0] * room[i][1] for i in range(n))
    walls = []
    for i in range(n):
        p, q = room[i], room[(i + 1) % n]
        dx, dy = q[0] - p[0], q[1] - p[1]
        out_dir = (dy, -dx) if area > 0 else (-dy, dx)   # 反時計回りなら右手側が外
        walls.append(slab_from_segment(p, q, out_dir, t_wall, 0.0, room_height))
    furn = {}
    for k, poly in enumerate(sc["polys"]):
        furn["Furniture_%d" % k] = (closed_prism(list(poly), 0.0, sc["heights"][k] or furniture_height), sc["names"][k])
    for m, (cx, cy, r) in enumerate(sc["circles"]):
        ring = [(cx + r * math.cos(2 * math.pi * i / 24), cy + r * math.sin(2 * math.pi * i / 24)) for i in range(24)]
        furn["Circle_%d" % m] = (closed_prism(ring, 0.0, furniture_height), "円柱")
    return walls, furn


def slabs_mesh(slabs):
    return merge([closed_prism(s["poly"], s["z0"], s["z1"]) for s in slabs])


def box_slab(x0, y0, x1, y1, z0, z1):
    """軸にそろった直方体を板と同じ dict の形にする"""
    poly = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return {"poly": poly, "z0": z0, "z1": z1, "center": ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
            "half": ((x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2), "yaw": 0.0, "normal": (0.0, 0.0)}


def greedy_rectangles(mask):
    """真偽の 2 次元配列 mask[j, i] の真の升を、重ならない長方形の列に貪欲にまとめる。
    行ごとに左から、まだ覆っていない真の升を起点に右へ伸ばし、同じ幅で上へ伸ばせるだけ伸ばす。
    戻り値 [(i0, j0, i1, j1)](両端を含む)"""
    m = np.asarray(mask, bool).copy()
    ny, nx = m.shape
    out = []
    for j in range(ny):
        i = 0
        while i < nx:
            if not m[j, i]:
                i += 1
                continue
            i1 = i
            while i1 + 1 < nx and m[j, i1 + 1]:
                i1 += 1
            j1 = j
            while j1 + 1 < ny and m[j1 + 1, i:i1 + 1].all():
                j1 += 1
            m[j:j1 + 1, i:i1 + 1] = False
            out.append((i, j, i1, j1))
            i = i1 + 1
    return out


def unobserved_colliders(w, x0, y0, h, height, min_w=0.0):
    """未観測(w <= min_w)の格子点を、その点を中心とする一辺 h の升の集まりとして物とみなし、
    長方形にまとめた箱の列にする。戻り値 (板と同じ形の dict の列, 統計)"""
    mask = np.asarray(w) <= min_w
    rects = greedy_rectangles(mask)
    slabs = [box_slab(x0 + (i0 - 0.5) * h, y0 + (j0 - 0.5) * h, x0 + (i1 + 0.5) * h, y0 + (j1 + 0.5) * h, 0.0, height)
             for i0, j0, i1, j1 in rects]
    return slabs, {"cells": int(mask.sum()), "boxes": len(slabs), "fraction": float(mask.mean())}


# ---------------------------------------------------------------- USD

def _require_pxr():
    try:
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt  # noqa: F401
    except ImportError as e:
        raise SystemExit("pxr (usd-core) が見つからない。リポジトリの .venv で実行すること: %s" % e)


class UsdRoomWriter:
    """部屋の USD を pxr で組み立てる(見た目・衝突判定・物理の場・作り方の記録)"""

    def __init__(self, doc, meta, ground_extent, floor_color=(0.30, 0.31, 0.33),
                 friction=(0.8, 0.6), restitution=0.0):
        _require_pxr()
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
        self.Gf, self.Sdf, self.Usd, self.UsdGeom, self.UsdPhysics, self.UsdShade = Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
        st = Usd.Stage.CreateInMemory()
        self.stage = st
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(st, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
        from pxr import Vt
        meta = {k: (Vt.DoubleArray([float(x) for x in v]) if isinstance(v, (list, tuple)) else v) for k, v in meta.items()}
        layer = st.GetRootLayer()
        layer.documentation = doc
        layer.customLayerData = {"dyne": meta}
        room = UsdGeom.Xform.Define(st, "/Room")
        st.SetDefaultPrim(room.GetPrim())
        self.Usd.ModelAPI(room.GetPrim()).SetKind("assembly")
        room.GetPrim().SetCustomDataByKey("dyne", meta)
        for p in ("/Room/Visual", "/Room/Collision", "/Room/Markers", "/Room/Reference", "/Room/PhysicsMaterials"):
            UsdGeom.Scope.Define(st, p)
        # 物理の場(defaultPrim の外)
        scene = UsdPhysics.Scene.Define(st, "/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)
        # 物性
        mat = UsdShade.Material.Define(st, "/Room/PhysicsMaterials/Default")
        pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        pm.CreateStaticFrictionAttr(friction[0])
        pm.CreateDynamicFrictionAttr(friction[1])
        pm.CreateRestitutionAttr(restitution)
        self.material = mat
        # 床: 上面が z=0、厚み 0.1 m の直方体
        x0, y0, x1, y1 = ground_extent
        cube = UsdGeom.Cube.Define(st, "/Room/Collision/Floor")
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.XformCommonAPI(cube)
        xf.SetTranslate(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, -0.05))
        xf.SetScale(Gf.Vec3f(x1 - x0, y1 - y0, 0.1))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*floor_color)])
        cube.GetPrim().SetCustomDataByKey("dyne:role", "floor collider (top at z=0)")
        self._collider(cube.GetPrim())

    def _collider(self, prim, approximation=None):
        UsdPhysics = self.UsdPhysics
        UsdPhysics.CollisionAPI.Apply(prim)
        if approximation is not None:
            mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mc.CreateApproximationAttr(approximation)
        b = self.UsdShade.MaterialBindingAPI.Apply(prim)
        b.Bind(self.material, self.UsdShade.Tokens.weakerThanDescendants, "physics")

    def mesh(self, path, V, F, color=None, vcolors=None, opacity=None, double_sided=True, primvars=None,
             custom=None, visible=True, counts=None):
        """V: 頂点の列、F: 面の頂点番号の列(counts を渡せば平らな列)"""
        from pxr import Gf, Sdf, Vt
        UsdGeom = self.UsdGeom
        m = UsdGeom.Mesh.Define(self.stage, path)
        pts = Vt.Vec3fArray([Gf.Vec3f(*p) for p in V])
        m.CreatePointsAttr(pts)
        if counts is None:
            m.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in F]))
            m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in F for i in f]))
        else:
            m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
            m.CreateFaceVertexIndicesAttr(Vt.IntArray(F))
        m.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(pts))
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        m.CreateDoubleSidedAttr(bool(double_sided))
        if vcolors is not None:
            pv = m.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
            pv.Set(Vt.Vec3fArray([Gf.Vec3f(*c) for c in vcolors]))
        elif color is not None:
            pv = m.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
            pv.Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
        if opacity is not None:
            m.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant).Set(Vt.FloatArray([opacity]))
        for name, (vals, interp) in (primvars or {}).items():
            UsdGeom.PrimvarsAPI(m).CreatePrimvar(name, Sdf.ValueTypeNames.FloatArray, interp).Set(Vt.FloatArray(vals))
        for name, (tname, val) in (custom or {}).items():
            m.GetPrim().CreateAttribute(name, getattr(Sdf.ValueTypeNames, tname), custom=True).Set(val)
        if not visible:
            m.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        return m

    def collider_mesh(self, path, V, F, custom=None, approximation="none"):
        """閉じた三角形メッシュの衝突判定(見えない)。静的なので RigidBodyAPI は付けない"""
        m = self.mesh(path, V, F, color=(0.9, 0.3, 0.3), double_sided=False, custom=custom, visible=False)
        self._collider(m.GetPrim(), approximation)
        m.GetPrim().SetCustomDataByKey("dyne:role", "static collider (closed slabs, invisible)")
        return m

    def collider_boxes(self, path, slabs, custom=None, color=(0.9, 0.3, 0.3)):
        """板の列を、箱の基本形(Cube。大きさ 1 を translate・rotateZ・scale で置く)の集まりにする。
        親は Xform(見えない)、子は Box_0000 から。どれも CollisionAPI と物性の割り当てを持つ。
        Isaac Sim(PhysX)は Cube を近似なしの箱として扱い、静的でも動く物でも使える"""
        from pxr import Gf, Sdf, Vt
        UsdGeom = self.UsdGeom
        grp = UsdGeom.Xform.Define(self.stage, path)
        grp.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        gp = grp.GetPrim()
        gp.SetCustomDataByKey("dyne:role", "static colliders (box primitives, invisible)")
        for name, (tname, val) in (custom or {}).items():
            gp.CreateAttribute(name, getattr(Sdf.ValueTypeNames, tname), custom=True).Set(val)
        for k, s in enumerate(slabs):
            c = UsdGeom.Cube.Define(self.stage, "%s/Box_%04d" % (path, k))
            c.CreateSizeAttr(1.0)
            c.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)]))
            cx, cy, cz = s["center"]
            hx, hy, hz = s["half"]
            c.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
            c.AddRotateZOp().Set(math.degrees(s["yaw"]))
            c.AddScaleOp().Set(Gf.Vec3f(2 * hx, 2 * hy, 2 * hz))
            c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            self._collider(c.GetPrim())
        return grp

    def colliders(self, path, slabs, shape="box", approximation="none", custom=None):
        """衝突判定を書く(shape = box: 箱の基本形 / mesh: 閉じた三角形メッシュ)"""
        if shape == "mesh":
            V, F = slabs_mesh(slabs)
            return self.collider_mesh(path, V, F, custom=custom, approximation=approximation)
        return self.collider_boxes(path, slabs, custom=custom)

    def save(self, path):
        self.stage.GetRootLayer().Export(path)


def now_utc():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ---------------------------------------------------------------- MJCF(MuJoCo)

def write_mjcf(path, colliders, ground_extent, model="dyne_room", friction=0.8, comment="", grid_extent=None,
               room_height=None):
    """板を box の geom として並べた MuJoCo の模型(静的な部屋だけ。試験用の物体は mjcf_check.py が足す)。
    grid_extent(格子の範囲 x0 y0 x1 y1)と room_height は <custom> に記録する(RL の環境が出発点・目標を
    選ぶ範囲に使う)"""
    x0, y0, x1, y1 = ground_extent
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    L = ['<mujoco model="%s">' % model]
    if comment:
        L.append("  <!-- %s -->" % comment.replace("--", "- -"))
    L += ['  <compiler angle="radian"/>',
          '  <option gravity="0 0 -9.81" timestep="0.002"/>',
          '  <default><geom friction="%g 0.005 0.0001" condim="3"/></default>' % friction]
    cust = []
    if grid_extent is not None:
        cust.append('    <numeric name="dyne_grid_extent" data="%.5f %.5f %.5f %.5f"/>' % tuple(grid_extent))
    if room_height is not None:
        cust.append('    <numeric name="dyne_room_height" data="%.5f"/>' % room_height)
    if cust:
        L += ["  <custom>"] + cust + ["  </custom>"]
    L += ["  <worldbody>",
          '    <light pos="%.4f %.4f 3" dir="0 0 -1"/>' % (cx, cy),
          '    <geom name="Floor" type="plane" pos="%.4f %.4f 0" size="%.4f %.4f 0.1" rgba="0.3 0.31 0.33 1"/>'
          % (cx, cy, (x1 - x0) / 2, (y1 - y0) / 2)]
    colors = {"Walls": "0.8 0.82 0.86 1", "Other": "0.85 0.7 0.45 1", "Unobserved": "0.45 0.45 0.5 0.5"}
    for name in sorted(colliders):
        rgba = colors.get(name, "0.93 0.62 0.3 1")
        for k, s in enumerate(colliders[name]["slabs"]):
            c, h = s["center"], s["half"]
            L.append('    <geom name="%s_%03d" type="box" pos="%.5f %.5f %.5f" size="%.5f %.5f %.5f" euler="0 0 %.6f" rgba="%s"/>'
                     % (name, k, c[0], c[1], c[2], h[0], h[1], h[2], s["yaw"], rgba))
    L += ["  </worldbody>", "</mujoco>"]
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(L) + "\n")

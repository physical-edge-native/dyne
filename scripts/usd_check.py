#!/usr/bin/env python3
"""Dyne が書き出した部屋の USD を、物理シミュレータに取り込める形かどうか検査する。2026-11-11。

検査すること(どれか 1 つでも満たさなければ FAIL):
  1. 舞台の単位: upAxis = Z、metersPerUnit = 1、kilogramsPerUnit = 1、defaultPrim = Room
  2. 物理の場(UsdPhysics.Scene)が 1 つあり、重力が -Z 向き・9.81 m/s²
  3. /Room/Collision の下の形(Gprim)はすべて CollisionAPI を持ち、それ以外(見た目・目印)は持たない。
     どこにも RigidBodyAPI が無い(部屋は静的)。床(/Room/Collision/Floor)がある
  4. 衝突判定のメッシュ: 頂点番号が範囲内、面積 0 の面が無い、閉じている(どの辺もちょうど 2 つの面が共有)、
     向きがそろっている(同じ向きの辺が 2 度現れない)、連結成分ごとの符号つき体積が正(法線が外向き)、
     extent が頂点の範囲と一致
  5. 見た目のメッシュ: 頂点番号が範囲内、面積 0 の面が無い
  6. 外接箱: 場面ファイル(--scene)を渡したとき、壁の衝突判定が部屋の外寸(+板の厚み)と、家具 k の衝突判定が
     場面ファイルの家具 k と、許容 --tol 以内で合う。床が部屋を覆い、上面が z = 0
  7. 参考: OpenUSD の検査の枠組み(UsdValidation。旧 UsdUtils.ComplianceChecker の後継)の結果を数える
     (この項目は PASS/FAIL に含めない。--strict-validation で含める)
  8. (2026-11-13)箱の基本形(Cube)の衝突判定: 大きさが正・有限、extent が大きさと合う、変換の操作が
     translate / rotate / orient / scale だけ、ワールドの変換にせん断が無く(軸が直交)拡大率が正。
     衝突判定の名前(/Room/Collision の直下の名前)ごとにまとめて外接箱を比べる(Walls の下の Box_0000 など)。
     Isaac(PhysX)に安全か: Cube と、近似(convexHull など)を指定した閉じたメッシュは安全。
     近似 none の三角形メッシュは静的な物にしか使えないので「注意」として数える(--isaac-strict で FAIL)

使い方(リポジトリの .venv で):
  .venv\\Scripts\\python.exe scripts\\usd_check.py demo_out\\twin\\room.usda --scene crates\\dyne-sim2d\\scenes\\room6.txt
"""
import argparse
import math
import os
import sys

import numpy as np

try:
    from pxr import Usd, UsdGeom, UsdPhysics
except ImportError:
    sys.exit("pxr (usd-core) が見つからない。リポジトリの .venv で実行すること")

HERE = os.path.dirname(os.path.abspath(__file__))


def mesh_arrays(prim, xf):
    m = UsdGeom.Mesh(prim)
    P = np.array(m.GetPointsAttr().Get() or [], float).reshape(-1, 3)
    if len(P):
        M = np.array(xf, float)  # 行ベクトル規約(p' = p M)
        P = np.c_[P, np.ones(len(P))] @ M
        P = P[:, :3]
    C = list(m.GetFaceVertexCountsAttr().Get() or [])
    I = list(m.GetFaceVertexIndicesAttr().Get() or [])
    return m, P, C, I


def triangles(C, I):
    """多角形を扇形に三角形へ分ける(面の番号つき)"""
    out, o = [], 0
    for f, c in enumerate(C):
        for k in range(1, c - 1):
            out.append((I[o], I[o + k], I[o + k + 1], f))
        o += c
    return out


def check_mesh(P, C, I, closed):
    r = {"faces": len(C), "points": len(P), "problems": []}
    if sum(C) != len(I):
        r["problems"].append("faceVertexCounts の合計 %d と faceVertexIndices の長さ %d が違う" % (sum(C), len(I)))
        return r
    if len(I) and (min(I) < 0 or max(I) >= len(P)):
        r["problems"].append("頂点番号が範囲外")
        return r
    if not np.isfinite(P).all():
        r["problems"].append("頂点に有限でない値")
    T = triangles(C, I)
    deg = 0
    for a, b, c, _ in T:
        if np.linalg.norm(np.cross(P[b] - P[a], P[c] - P[a])) < 1e-10:
            deg += 1
    r["degenerate"] = deg
    if deg:
        r["problems"].append("面積 0 の三角形 %d 個" % deg)
    if not closed:
        return r
    # 閉じているか・向きがそろっているか(面の多角形の辺で数える)
    directed, undirected = {}, {}
    o = 0
    for c in C:
        f = I[o:o + c]
        o += c
        for k in range(c):
            a, b = f[k], f[(k + 1) % c]
            directed[(a, b)] = directed.get((a, b), 0) + 1
            key = (min(a, b), max(a, b))
            undirected[key] = undirected.get(key, 0) + 1
    open_edges = sum(1 for v in undirected.values() if v != 2)
    flipped = sum(1 for v in directed.values() if v > 1) + sum(1 for (a, b) in directed if (b, a) not in directed)
    r["open_edges"] = open_edges
    r["orientation_errors"] = flipped
    if open_edges:
        r["problems"].append("閉じていない(2 面に共有されない辺 %d 本)" % open_edges)
    if flipped:
        r["problems"].append("面の向きがそろっていない辺 %d 本" % flipped)
    # 連結成分(頂点の共有)と、成分ごとの符号つき体積
    parent = list(range(len(P)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for a, b, c, _ in T:
        for u, v in ((a, b), (b, c)):
            ru, rv = find(u), find(v)
            if ru != rv:
                parent[ru] = rv
    vol = {}
    for a, b, c, _ in T:
        vol[find(a)] = vol.get(find(a), 0.0) + float(np.dot(P[a], np.cross(P[b], P[c]))) / 6.0
    r["components"] = len(vol)
    r["min_component_volume"] = min(vol.values()) if vol else 0.0
    r["total_volume"] = sum(vol.values())
    neg = sum(1 for v in vol.values() if v <= 1e-12)
    if neg:
        r["problems"].append("体積が正でない(内向きの)成分 %d 個" % neg)
    return r


def parse_scene(path):
    sys.path.insert(0, HERE)
    import twin_demo  # 場面ファイルの読み方を共有する
    return twin_demo.parse_scene(path)


def run_validation(stage):
    """OpenUSD の検査の枠組み。戻り値 (検査器の数, [(名前, 種類, 文)])、使えなければ None"""
    try:
        from pxr import UsdValidation
    except ImportError:
        UsdValidation = None
    if UsdValidation is not None:
        reg = UsdValidation.ValidationRegistry()
        vals = reg.GetOrLoadAllValidators()
        ctx = UsdValidation.ValidationContext(vals)
        errs = ctx.Validate(stage)
        out = []
        for e in errs:
            if e.HasNoError():
                continue
            out.append((e.GetName(), str(e.GetType()).split(".")[-1], e.GetMessage()))
        return "UsdValidation", len(vals), out
    try:
        from pxr import UsdUtils
        cc = UsdUtils.ComplianceChecker(rootPackageOnly=False, skipVariants=False)
        cc.CheckCompliance(stage.GetRootLayer().identifier)
        out = [("ComplianceChecker", "Error", m) for m in cc.GetErrors()] + \
              [("ComplianceChecker", "Failed", m) for m in cc.GetFailedChecks()] + \
              [("ComplianceChecker", "Warning", m) for m in cc.GetWarnings()]
        return "ComplianceChecker", 0, out
    except (ImportError, AttributeError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("usd")
    ap.add_argument("--scene", help="場面ファイル(外接箱を比べる)")
    ap.add_argument("--tol", type=float, default=0.03, help="外接箱の許容のずれ [m]")
    ap.add_argument("--room-height", type=float, default=None, help="既定は USD の記録(roomHeightM)")
    ap.add_argument("--bbox-warn-only", action="store_true",
                    help="正解との外接箱の差は報告だけにする(推定の精度が低い条件で、形式の検査だけ合否に使う)")
    ap.add_argument("--strict-validation", action="store_true", help="UsdValidation の Error も FAIL に含める")
    ap.add_argument("--isaac-strict", action="store_true",
                    help="近似 none の三角形メッシュの衝突判定(PhysX では静的な物に限る)も FAIL にする")
    a = ap.parse_args()

    stage = Usd.Stage.Open(a.usd)
    fails, lines = [], []

    warns = []

    def ok(cond, msg, warn_only=False):
        lines.append(("OK  " if cond else ("注意 " if warn_only else "NG  ")) + msg)
        if not cond:
            (warns if warn_only else fails).append(msg)

    # 1. 単位
    up = UsdGeom.GetStageUpAxis(stage)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    kpu = UsdPhysics.GetStageKilogramsPerUnit(stage)
    dp = stage.GetDefaultPrim()
    ok(up == "Z", "upAxis = %s" % up)
    ok(abs(mpu - 1.0) < 1e-12, "metersPerUnit = %g" % mpu)
    ok(stage.HasAuthoredMetadata("kilogramsPerUnit") and abs(kpu - 1.0) < 1e-12, "kilogramsPerUnit = %g" % kpu)
    ok(bool(dp) and dp.GetPath().pathString == "/Room", "defaultPrim = %s" % (dp.GetPath() if dp else "なし"))
    meta = (stage.GetRootLayer().customLayerData or {}).get("dyne", {})
    lines.append("    記録: createdUtc=%s sequence=%s scene=%s kind=%s" % (meta.get("createdUtc"), meta.get("sequence"),
                                                                     meta.get("scene"), meta.get("kind", "estimated")))

    # 2. 物理の場
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    ok(len(scenes) == 1, "UsdPhysics.Scene の数 = %d" % len(scenes))
    if scenes:
        s = UsdPhysics.Scene(scenes[0])
        d = s.GetGravityDirectionAttr().Get()
        g = s.GetGravityMagnitudeAttr().Get()
        ok(d is not None and abs(d[0]) < 1e-9 and abs(d[1]) < 1e-9 and d[2] < 0 and g is not None and abs(g - 9.81) < 1e-6,
           "重力 方向 %s 大きさ %s m/s² (%s)" % (tuple(d) if d else None, g, scenes[0].GetPath()))
        ok(not scenes[0].GetPath().HasPrefix(dp.GetPath()) if dp else True,
           "物理の場は defaultPrim の外(参照で取り込む側の物理の場と二重にならない)")

    # 3. 衝突判定の付き方
    coll_root = stage.GetPrimAtPath("/Room/Collision")
    ok(bool(coll_root), "/Room/Collision がある")
    xc = UsdGeom.XformCache()
    colliders, visuals, n_rigid, stray = [], [], 0, []
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            n_rigid += 1
        in_coll = p.GetPath().HasPrefix("/Room/Collision")
        if p.IsA(UsdGeom.Gprim):
            if in_coll:
                colliders.append(p)
            else:
                visuals.append(p)
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    stray.append(p.GetPath().pathString)
    ok(n_rigid == 0, "RigidBodyAPI を持つ prim = %d(部屋は静的)" % n_rigid)
    no_api = [p.GetPath().pathString for p in colliders if not p.HasAPI(UsdPhysics.CollisionAPI)]
    ok(not no_api, "/Room/Collision の形 %d 個のうち CollisionAPI が無いもの = %d %s" % (len(colliders), len(no_api), no_api[:5]))
    ok(not stray, "見た目・目印で CollisionAPI を持つもの = %d %s" % (len(stray), stray[:5]))
    markers = [p for p in visuals if p.GetPath().HasPrefix("/Room/Markers")]
    lines.append("    目印(ノード・センサ)の形 %d 個、見た目の形 %d 個 は衝突判定なし" % (len(markers), len(visuals) - len(markers)))
    floor = stage.GetPrimAtPath("/Room/Collision/Floor")
    ok(bool(floor) and floor.HasAPI(UsdPhysics.CollisionAPI), "床 /Room/Collision/Floor に CollisionAPI")

    # 4. 衝突判定のメッシュ・箱
    bbox = {}
    total_faces = 0
    cube_stats = {}      # 名前 → [箱の数, 問題のある箱の数, 例]
    isaac_unsafe = []

    def grow(name, lo, hi):
        if name in bbox:
            bbox[name] = (np.minimum(bbox[name][0], lo), np.maximum(bbox[name][1], hi))
        else:
            bbox[name] = (lo, hi)
    allowed_ops = ("xformOp:translate", "xformOp:rotate", "xformOp:orient", "xformOp:scale")
    for p in colliders:
        rel = p.GetPath().MakeRelativePath("/Room/Collision").pathString
        name = rel.split("/")[0]
        xf = xc.GetLocalToWorldTransform(p)
        if p.IsA(UsdGeom.Cube) and name != "Floor":
            s = UsdGeom.Cube(p).GetSizeAttr().Get()
            prob = []
            if s is None or not np.isfinite(s) or s <= 0:
                prob.append("大きさ %s" % s)
                s = 1.0
            ext = UsdGeom.Cube(p).GetExtentAttr().Get()
            if ext is None or not np.allclose(np.array(ext, float), [[-s / 2] * 3, [s / 2] * 3], atol=1e-6):
                prob.append("extent %s が大きさと合わない" % (ext,))
            ops = [op.GetOpName() for op in UsdGeom.Xformable(p).GetOrderedXformOps()]
            bad_ops = [o for o in ops if not o.startswith(allowed_ops)]
            if bad_ops:
                prob.append("変換の操作 %s" % bad_ops)
            M = np.array(xf, float)
            axes = M[:3, :3]
            sc_ = np.linalg.norm(axes, axis=1)
            if not np.isfinite(M).all() or np.any(sc_ <= 1e-9):
                prob.append("拡大率が 0 か有限でない")
            else:
                G = (axes / sc_[:, None]) @ (axes / sc_[:, None]).T
                if np.abs(G - np.eye(3)).max() > 1e-6:
                    prob.append("せん断がある(軸が直交しない)")
                if np.linalg.det(axes) <= 0:
                    prob.append("鏡映(行列式が負)")
            corners = np.array([[x, y, z, 1.0] for x in (-s / 2, s / 2) for y in (-s / 2, s / 2) for z in (-s / 2, s / 2)])
            Wc = (corners @ M)[:, :3]
            grow(name, Wc.min(0), Wc.max(0))
            st_ = cube_stats.setdefault(name, [0, 0, ""])
            st_[0] += 1
            if prob:
                st_[1] += 1
                st_[2] = st_[2] or "%s: %s" % (p.GetName(), "; ".join(prob))
            continue
        if p.IsA(UsdGeom.Mesh):
            m, P, C, I = mesh_arrays(p, xf)
            r = check_mesh(P, C, I, closed=True)
            appr = UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() if p.HasAPI(UsdPhysics.MeshCollisionAPI) else "(MeshCollisionAPI 無し)"
            ext = m.GetExtentAttr().Get()
            Pl = np.array(m.GetPointsAttr().Get(), float)
            ext_ok = ext is not None and np.allclose(np.array(ext, float), [Pl.min(0), Pl.max(0)], atol=1e-5)
            if not ext_ok:
                r["problems"].append("extent が頂点の範囲と違う")
            vis = UsdGeom.Imageable(p).ComputeVisibility()
            total_faces += r["faces"]
            ok(not r["problems"], "衝突判定 %-12s 面 %4d 頂点 %4d 成分 %3d 体積 %.4f m³ 開いた辺 %d 向きの誤り %d 面積0 %d approximation=%s 表示=%s %s"
               % (name, r["faces"], r["points"], r.get("components", 0), r.get("total_volume", 0.0), r.get("open_edges", -1),
                  r.get("orientation_errors", -1), r.get("degenerate", -1), appr, vis, "; ".join(r["problems"])))
            grow(name, P.min(0), P.max(0))
            if appr in ("none", "(MeshCollisionAPI 無し)"):
                isaac_unsafe.append(name)
        elif p.IsA(UsdGeom.Cube):
            s = UsdGeom.Cube(p).GetSizeAttr().Get()
            corners = np.array([[x, y, z, 1.0] for x in (-s / 2, s / 2) for y in (-s / 2, s / 2) for z in (-s / 2, s / 2)])
            W = (corners @ np.array(xf, float))[:, :3]
            grow(name, W.min(0), W.max(0))
            lines.append("    衝突判定 %-12s Cube 外接箱 (%.3f, %.3f, %.3f)〜(%.3f, %.3f, %.3f)" % ((name,) + tuple(W.min(0)) + tuple(W.max(0))))
        else:
            lines.append("    衝突判定 %-12s 型 %s(形の検査は省略)" % (name, p.GetTypeName()))
            isaac_unsafe.append(name)
    for name in sorted(cube_stats):
        n, nbad, ex = cube_stats[name]
        vis = UsdGeom.Imageable(stage.GetPrimAtPath("/Room/Collision/" + name)).ComputeVisibility()
        ok(nbad == 0, "衝突判定 %-12s 箱の基本形 %4d 個(問題 %d 個)表示=%s %s" % (name, n, nbad, vis, ex))
    n_cubes = sum(v[0] for v in cube_stats.values())
    lines.append("    衝突判定のメッシュの面の合計 %d、箱の基本形の合計 %d" % (total_faces, n_cubes))
    ok(not isaac_unsafe, warn_only=not a.isaac_strict,
       msg="Isaac(PhysX)向き: 近似 none の三角形メッシュの衝突判定 = %d %s(静的な物なら使えるが、動かす物には使えない)"
       % (len(isaac_unsafe), sorted(set(isaac_unsafe))))

    # 5. 見た目のメッシュ
    vis_prob, vis_faces = 0, 0
    for p in visuals:
        if p.IsA(UsdGeom.Mesh):
            _, P, C, I = mesh_arrays(p, xc.GetLocalToWorldTransform(p))
            r = check_mesh(P, C, I, closed=False)
            vis_faces += r["faces"]
            if r["problems"]:
                vis_prob += 1
                lines.append("NG  見た目 %s: %s" % (p.GetPath(), "; ".join(r["problems"])))
    ok(vis_prob == 0, "見た目のメッシュの問題 %d 個(面の合計 %d)" % (vis_prob, vis_faces))

    # 6. 外接箱
    if "Floor" in bbox:
        fmin, fmax = bbox["Floor"]
        ok(abs(fmax[2]) < 1e-6, "床の上面 z = %.6f" % fmax[2])
    if a.scene:
        sc = parse_scene(a.scene)
        H = a.room_height or float(meta.get("roomHeightM", 2.4))
        tw = float(meta.get("wallThicknessM", 0.1))
        rx = [q[0] for q in sc["room"]]
        ry = [q[1] for q in sc["room"]]
        if "Floor" in bbox:
            fmin, fmax = bbox["Floor"]
            ok(fmin[0] <= min(rx) - tw and fmin[1] <= min(ry) - tw and fmax[0] >= max(rx) + tw and fmax[1] >= max(ry) + tw,
               "床が部屋と壁の板を覆う (%.2f, %.2f)〜(%.2f, %.2f)" % (fmin[0], fmin[1], fmax[0], fmax[1]))
        exp = {"Walls": (np.array([min(rx) - tw, min(ry) - tw, 0.0]), np.array([max(rx) + tw, max(ry) + tw, H]))}
        for k, poly in enumerate(sc["polys"]):
            hh = sc["heights"][k] or float(meta.get("furnitureHeightM", 0.7))
            exp["Furniture_%d" % k] = (np.array([min(q[0] for q in poly), min(q[1] for q in poly), 0.0]),
                                       np.array([max(q[0] for q in poly), max(q[1] for q in poly), hh]))
        for name in sorted(bbox):
            if name == "Floor":
                continue
            bmin, bmax = bbox[name]
            if name not in exp:
                lines.append("    外接箱 %-12s (%.3f, %.3f, %.3f)〜(%.3f, %.3f, %.3f)(比べる正解なし)" % ((name,) + tuple(bmin) + tuple(bmax)))
                continue
            emin, emax = exp[name]
            note = ""
            if name != "Walls":
                # 部屋の外(壁の中)へ出た部分は、もともと物の中なので比べない(壁際の角の丸みで板が壁へ入ることがある)
                cmin = bmin.copy()
                cmax = bmax.copy()
                cmin[:2] = np.maximum(bmin[:2], [min(rx), min(ry)])
                cmax[:2] = np.minimum(bmax[:2], [max(rx), max(ry)])
                if np.any(cmin != bmin) or np.any(cmax != bmax):
                    note = "(部屋の外へ出た部分 最大 %.1f mm は除いて比べる)" % (1000 * float(max(np.max(cmin - bmin), np.max(bmax - cmax))))
                bmin, bmax = cmin, cmax
            dev = np.r_[np.abs(bmin - emin), np.abs(bmax - emax)]
            # 隠れた面は推定に無いので、推定の外接箱が正解の内側にあれば縮みは許す(はみ出しだけ許容と比べる)
            over = float(max(np.max(emin - bmin), np.max(bmax - emax), 0.0))
            ok(over <= a.tol, warn_only=a.bbox_warn_only, msg="外接箱 %-12s (%.3f, %.3f, %.3f)〜(%.3f, %.3f, %.3f) 正解との差 最大 %.1f mm、はみ出し %.1f mm(許容 %.0f mm)%s"
               % ((name,) + tuple(bmin) + tuple(bmax) + (dev.max() * 1000, over * 1000, a.tol * 1000, note)))

    # 7. OpenUSD の検査の枠組み
    v = run_validation(stage)
    if v is None:
        lines.append("    OpenUSD の検査の枠組み: 使えない")
    else:
        kind, nval, errs = v
        counts = {}
        for name, typ, msg in errs:
            counts[(name, typ)] = counts.get((name, typ), 0) + 1
        lines.append("    %s: 検査器 %d 個、指摘 %d 件" % (kind, nval, len(errs)))
        for (name, typ), n in sorted(counts.items()):
            ex = next(m for nm, t, m in errs if nm == name and t == typ)
            lines.append("      [%s] %s × %d 例: %s" % (typ, name, n, ex[:160].replace("\n", " ")))
        n_err = sum(1 for _, t, _ in errs if t == "Error")
        if a.strict_validation:
            ok(n_err == 0, "%s の Error = %d" % (kind, n_err))

    print("USD の検査: %s" % a.usd)
    for l in lines:
        print(l)
    if warns:
        print("注意: %d 項目(正解との外接箱の差は --bbox-warn-only、Isaac 向きの指摘は --isaac-strict なしのため合否に含めない)"
              % len(warns))
    if fails:
        print("FAIL: %d 項目" % len(fails))
        sys.exit(1)
    print("PASS: 物理シミュレータに静的な部屋として取り込める形(衝突判定 %d 個)" % len(colliders))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""部屋の衝突判定が物理シミュレータで本当に効くかを、CPU で動く MuJoCo で確かめる。2026-11-11。

Isaac Sim(GPU が要る)の代わりの保険。同じ部屋を 2 通りで読み込む:
  --xml room.xml   twin_demo.py が書いた MJCF(板 1 枚 = box の geom 1 つ)
  --usd room.usda  USD の /Room/Collision を読み、閉じたメッシュを連結成分(凸の板)ごとに MuJoCo の mesh の
                   geom にする(MuJoCo は mesh を凸包で扱うので、成分ごとに分ければ形が変わらない)。
                   USD に書いた衝突判定そのものが物理エンジンで効くことの確認になる
試験:
  1. 球(半径 0.1 m)を高さ 1 m から落とし、2 秒後に床の上(中心の高さ = 半径)で止まっていること
  2. 円柱の「ロボット」(半径 0.15 m、高さ 0.2 m、床から 5 cm 浮かせ、x・y にだけ動ける)を、部屋の中で
     最も広く空いた点から ±x・±y の 4 方向へ一定の力で押す。止まった位置を、場面ファイルの正解の形から
     計算した「ぶつかるはずの位置」と比べる。正解の面に届く前に止まらず通り抜けたら、推定の形の穴(未観測)
  3. (2026-11-13、--escape)抜け出しの試験: 正解の部屋の空きに 0.3 m おきに小さな円柱のロボットを並べ、
     8 方向へ押す。中心が正解の部屋の外・家具の中に入ったら「抜け出し」= 推定の衝突判定の穴を通った。
     未観測を物とみなす(twin_demo.py --unobserved-solid on)ときに穴が残っていないことの確認に使う
  (2026-11-13)ロボットと球の出発点は、推定の衝突判定(未観測を物とみなした箱を含む)に重ならない点から選ぶ。
  USD の箱の基本形(Cube)は向きつきの box の geom に写す。
合格: 球が床で止まる、めり込みが 1 cm 未満(物理エンジンとして衝突判定が効いていること)。
      --strict のときは加えて、止まった位置の正解との差が --tol 以内で、通り抜け(推定の形の穴)が無いこと
      (推定の形の精度。理想条件の room6 で使う)。

使い方(リポジトリの .venv。mujoco は .venv に入れる):
  .venv\\Scripts\\python.exe scripts\\mjcf_check.py --xml demo_out\\twin\\room.xml --scene crates\\dyne-sim2d\\scenes\\room6.txt
  .venv\\Scripts\\python.exe scripts\\mjcf_check.py --usd demo_out\\twin\\room.usda --scene crates\\dyne-sim2d\\scenes\\room6.txt
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import twin_demo  # noqa: E402  場面ファイルの読み方・光線の計算を共有する

R_ROBOT, HALF_H, Z_ROBOT = 0.15, 0.10, 0.15
R_BALL = 0.10


def usd_to_mjcf(usd_path):
    """USD の /Room/Collision を MJCF の文字列にする"""
    from pxr import Usd, UsdGeom, UsdPhysics
    st = Usd.Stage.Open(usd_path)
    xc = UsdGeom.XformCache()
    assets, geoms, n_comp = [], [], 0
    for p in Usd.PrimRange(st.GetPrimAtPath("/Room/Collision")):
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            continue
        M = np.array(xc.GetLocalToWorldTransform(p), float)
        # 名前は /Room/Collision からの相対パス(Walls/Box_0003 → Walls_Box_0003。MuJoCo は名前の重複を許さない)
        name = p.GetPath().MakeRelativePath("/Room/Collision").pathString.replace("/", "_")
        if p.IsA(UsdGeom.Cube):
            # 2026-11-13: 箱の基本形(z 軸回りに回した板)を向きつきで写す。行ベクトル規約なので M の行 i が局所の軸 i
            s = UsdGeom.Cube(p).GetSizeAttr().Get()
            c = np.array([0, 0, 0, 1.0]) @ M
            half = [s / 2 * np.linalg.norm(M[i, :3]) for i in range(3)]
            ax = M[0, :3] / np.linalg.norm(M[0, :3])
            ay = M[1, :3] / np.linalg.norm(M[1, :3])
            geoms.append('<geom name="%s" type="box" pos="%g %g %g" size="%g %g %g" xyaxes="%.9g %.9g %.9g %.9g %.9g %.9g"/>'
                         % ((name,) + tuple(c[:3]) + tuple(half) + tuple(ax) + tuple(ay)))
            n_comp += 1
            continue
        m = UsdGeom.Mesh(p)
        P = np.array(m.GetPointsAttr().Get(), float)
        P = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        C = list(m.GetFaceVertexCountsAttr().Get())
        I = list(m.GetFaceVertexIndicesAttr().Get())
        parent = list(range(len(P)))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        o = 0
        for c in C:
            f = I[o:o + c]
            o += c
            for k in range(1, c):
                ra, rb = find(f[0]), find(f[k])
                if ra != rb:
                    parent[ra] = rb
        comps = {}
        for i in range(len(P)):
            comps.setdefault(find(i), []).append(i)
        for k, idx in enumerate(sorted(comps.values())):
            mn = "%s_%03d" % (name, k)
            assets.append('<mesh name="%s" vertex="%s"/>' % (mn, " ".join("%.6g" % v for v in P[idx].ravel())))
            geoms.append('<geom name="%s" type="mesh" mesh="%s"/>' % (mn, mn))
            n_comp += 1
    xml = ('<mujoco model="dyne_room_from_usd"><compiler angle="radian"/><option gravity="0 0 -9.81" timestep="0.002"/>'
           '<default><geom friction="0.8 0.005 0.0001" condim="3"/></default>'
           "<asset>%s</asset><worldbody>%s</worldbody></mujoco>" % ("".join(assets), "".join(geoms)))
    return xml, n_comp


def free_point(sc, step=0.05, xml=None):
    """部屋の中で正解の面から最も遠い点(ロボットを置く)。
    xml を渡すと(2026-11-13)、推定の衝突判定(未観測を物とみなした箱を含む)にロボットが重ならない点の中で選ぶ"""
    segs = twin_demo.truth_segments(sc)
    xs = [p[0] for p in sc["room"]]
    ys = [p[1] for p in sc["room"]]
    gx, gy = np.meshgrid(np.arange(min(xs) + step, max(xs), step), np.arange(min(ys) + step, max(ys), step))
    pts = np.c_[gx.ravel(), gy.ravel()]
    free = np.array([twin_demo.is_free(sc, x, y) for x, y in pts])
    d = twin_demo.dist_to_segments(pts, segs)
    d[~free] = -1
    if xml is None:
        k = int(np.argmax(d))
        return tuple(pts[k]), float(d[k])
    import mujoco
    root = ET.fromstring(xml)
    b = ET.SubElement(root.find("worldbody"), "body", name="probe", pos="0 0 %g" % Z_ROBOT)
    ET.SubElement(b, "joint", name="px", type="slide", axis="1 0 0")
    ET.SubElement(b, "joint", name="py", type="slide", axis="0 1 0")
    ET.SubElement(b, "geom", name="probe_geom", type="cylinder", size="%g %g" % (R_ROBOT + 0.02, HALF_H), contype="2", conaffinity="1")
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    dd = mujoco.MjData(m)
    g = m.geom("probe_geom").id
    for k in np.argsort(-d):
        if d[k] <= R_ROBOT:
            break
        dd.qpos[:2] = pts[k]
        mujoco.mj_forward(m, dd)
        if not any(g in (dd.contact[c].geom1, dd.contact[c].geom2) for c in range(dd.ncon)):
            return tuple(pts[k]), float(d[k])
    raise SystemExit("推定の衝突判定に重ならずにロボットを置ける点が無い")


def expected_travel(sc, start, direction, r):
    """半径 r の円が start から direction へ進んで正解の面に触れるまでの距離(円の前縁を 41 本の光線で近似)"""
    dx, dy = direction
    px, py = -dy, dx
    best = math.inf
    for o in np.linspace(-r, r, 41):
        t = twin_demo.raycast(sc, start[0] + px * o, start[1] + py * o, dx, dy)
        if t is not None:
            best = min(best, t - math.sqrt(max(r * r - o * o, 0.0)))
    return best


def add_ball(xml, start):
    root = ET.fromstring(xml)
    b = ET.SubElement(root.find("worldbody"), "body", name="ball", pos="%g %g 1.0" % start)
    ET.SubElement(b, "freejoint", name="ball_free")
    ET.SubElement(b, "geom", name="ball_geom", type="sphere", size=str(R_BALL), mass="1", rgba="0.2 0.5 1 1")
    return ET.tostring(root, encoding="unicode")


def add_test_bodies(xml, start, axis, sign):
    root = ET.fromstring(xml)
    wb = root.find("worldbody")
    rb = ET.SubElement(wb, "body", name="robot", pos="%g %g %g" % (start[0], start[1], Z_ROBOT))
    ET.SubElement(rb, "joint", name="jx", type="slide", axis="1 0 0", damping="60")
    ET.SubElement(rb, "joint", name="jy", type="slide", axis="0 1 0", damping="60")
    ET.SubElement(rb, "geom", name="robot_geom", type="cylinder", size="%g %g" % (R_ROBOT, HALF_H), mass="10", rgba="0.2 0.8 0.3 1")
    act = root.find("actuator")
    if act is None:
        act = ET.SubElement(root, "actuator")
    ET.SubElement(act, "motor", name="push", joint="jx" if axis == 0 else "jy", gear=str(30.0 * sign))
    return ET.tostring(root, encoding="unicode")


def run(xml, sc, tol, strict, label):
    import mujoco
    start, clear = free_point(sc, xml=xml)
    xs = [p[0] for p in sc["room"]]
    ys = [p[1] for p in sc["room"]]
    room_diag = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    fails, gaps = [], []
    base = mujoco.MjModel.from_xml_string(xml)
    print("[%s] 読み込み OK: geom %d 個(うち mesh %d)、mesh の資産 %d 個" %
          (label, base.ngeom, int((base.geom_type == mujoco.mjtGeom.mjGEOM_MESH).sum()), base.nmesh))
    print("[%s] ロボットの出発点 (%.2f, %.2f)、正解の面までの余裕 %.2f m" % (label, start[0], start[1], clear))
    m = mujoco.MjModel.from_xml_string(add_ball(xml, start))
    d = mujoco.MjData(m)
    while d.time < 2.0:
        mujoco.mj_step(m, d)
    ball_z = float(d.qpos[m.joint("ball_free").qposadr[0] + 2])
    ok = abs(ball_z - R_BALL) < 0.005
    print("[%s] 球: 2 秒後の中心の高さ %.4f m(期待 %.3f m)%s" % (label, ball_z, R_BALL, "" if ok else "  ← NG"))
    if not ok:
        fails.append("球が床で止まらない")
    for n, (axis, sign) in enumerate(((0, 1), (0, -1), (1, 1), (1, -1))):
        m = mujoco.MjModel.from_xml_string(add_test_bodies(xml, start, axis, sign))
        d = mujoco.MjData(m)
        d.ctrl[:] = 1.0
        jx, jy = m.joint("jx").qposadr[0], m.joint("jy").qposadr[0]
        min_dist = 0.0
        steps = int(14.0 / m.opt.timestep)
        for s in range(steps):
            mujoco.mj_step(m, d)
            for c in range(d.ncon):
                min_dist = min(min_dist, d.contact[c].dist)
            if abs(d.qpos[jx]) > room_diag + 1 or abs(d.qpos[jy]) > room_diag + 1:
                break
        moved = abs(d.qpos[jx if axis == 0 else jy])
        dirv = (sign, 0.0) if axis == 0 else (0.0, sign)
        exp = expected_travel(sc, start, dirv, R_ROBOT)
        name = ("+x", "-x", "+y", "-y")[n]
        vel = abs(d.qvel[m.joint("jx" if axis == 0 else "jy").dofadr[0]])
        if moved > exp + 0.25 or vel > 0.05:
            gaps.append(name)
            print("[%s] ロボット %s: 正解の面(%.3f m 先)で止まらず通り抜けた(移動 %.2f m)= 推定の形の穴" % (label, name, exp, moved))
            continue
        diff = moved - exp
        pos_ok = abs(diff) <= tol
        pen_ok = min_dist > -0.01
        mark = "" if (pos_ok and pen_ok) else ("  ← NG" if (strict or not pen_ok) else "  ← 注意(推定の形の誤差)")
        print("[%s] ロボット %s: 移動 %.4f m、正解なら %.4f m、差 %+.1f mm、最大のめり込み %.1f mm%s"
              % (label, name, moved, exp, diff * 1000, -min_dist * 1000, mark))
        if not pen_ok:
            fails.append("ロボット %s のめり込み" % name)
        if strict and not pos_ok:
            fails.append("ロボット %s の止まる位置" % name)
    if strict and gaps:
        fails.append("通り抜け %s" % gaps)
    return fails, gaps


def escape_test(xml, sc, radius, spacing, directions, seconds, label):
    """抜け出しの試験(2026-11-13): 正解の部屋の空き(面から radius+3cm 以上離れた点)に spacing おきに
    円柱のロボットを並べ、directions 方向へ一定の力で押す。ロボットどうしは当たらない(contype/conaffinity)。
    途中でロボットの中心が正解の部屋の外か家具の中に入ったら「抜け出し」(推定の衝突判定の穴を通った)。
    出発点で既に推定の物(未観測を物とみなした箱など)に重なっているロボットは除く。
    戻り値 (抜け出しの数, 試した数, 除いた出発点の数, 例)"""
    import mujoco
    segs = twin_demo.truth_segments(sc)
    xs = [p[0] for p in sc["room"]]
    ys = [p[1] for p in sc["room"]]
    gx, gy = np.meshgrid(np.arange(min(xs) + spacing / 2, max(xs), spacing), np.arange(min(ys) + spacing / 2, max(ys), spacing))
    pts = np.c_[gx.ravel(), gy.ravel()]
    free = np.array([twin_demo.is_free(sc, x, y) for x, y in pts])
    d = twin_demo.dist_to_segments(pts, segs)
    starts = pts[free & (d > radius + 0.03)]
    zr = 0.05 + 0.05  # 床から 5 cm 浮かせる(半高 5 cm)
    root = ET.fromstring(xml)
    wb = root.find("worldbody")
    act = root.find("actuator")
    if act is None:
        act = ET.SubElement(root, "actuator")
    for k, (x, y) in enumerate(starts):
        b = ET.SubElement(wb, "body", name="r%d" % k, pos="%g %g %g" % (x, y, zr))
        ET.SubElement(b, "joint", name="r%dx" % k, type="slide", axis="1 0 0", damping="60")
        ET.SubElement(b, "joint", name="r%dy" % k, type="slide", axis="0 1 0", damping="60")
        ET.SubElement(b, "geom", name="r%dg" % k, type="cylinder", size="%g 0.05" % radius, mass="5",
                      contype="2", conaffinity="1", rgba="0.2 0.8 0.3 1")
        ET.SubElement(act, "motor", name="r%dmx" % k, joint="r%dx" % k, gear="30")
        ET.SubElement(act, "motor", name="r%dmy" % k, joint="r%dy" % k, gear="30")
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    n = len(starts)
    qx = np.array([m.joint("r%dx" % k).qposadr[0] for k in range(n)])
    qy = np.array([m.joint("r%dy" % k).qposadr[0] for k in range(n)])
    gid = np.array([m.geom("r%dg" % k).id for k in range(n)])
    gid_to_k = {int(g): k for k, g in enumerate(gid)}
    # 出発点で推定の物に重なっているか
    d0 = mujoco.MjData(m)
    mujoco.mj_forward(m, d0)
    blocked = set()
    for c in range(d0.ncon):
        con = d0.contact[c]
        for g in (con.geom1, con.geom2):
            if g in gid_to_k and con.dist < -0.002:
                blocked.add(gid_to_k[g])
    active = [k for k in range(n) if k not in blocked]
    escapes, tried, examples = 0, 0, []
    for th in np.linspace(0, 2 * math.pi, directions, endpoint=False):
        dd = mujoco.MjData(m)
        dd.ctrl[0::2] = math.cos(th)
        dd.ctrl[1::2] = math.sin(th)
        bad = set()
        steps = int(seconds / m.opt.timestep)
        for s in range(steps):
            mujoco.mj_step(m, dd)
            if s % 25 == 24 or s == steps - 1:
                X = starts[:, 0] + dd.qpos[qx]
                Y = starts[:, 1] + dd.qpos[qy]
                for k in active:
                    if k not in bad and not twin_demo.is_free(sc, X[k], Y[k]):
                        bad.add(k)
                        if len(examples) < 5:
                            examples.append("出発 (%.2f, %.2f) 向き %.0f 度 → (%.2f, %.2f)" % (starts[k][0], starts[k][1],
                                                                                    math.degrees(th), X[k], Y[k]))
        escapes += len(bad)
        tried += len(active)
    print("[%s] 抜け出しの試験: ロボット半径 %.2f m、出発点 %d 点(%.2f m おき、うち推定の物に重なり除いた %d 点)× %d 方向 × %.0f 秒"
          % (label, radius, n, spacing, len(blocked), directions, seconds))
    print("[%s]   抜け出し %d / %d 回%s" % (label, escapes, tried, ("。例: " + " / ".join(examples)) if examples else ""))
    return escapes, tried, len(blocked), examples


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", help="twin_demo.py が書いた MJCF")
    ap.add_argument("--usd", help="twin_demo.py が書いた USD(/Room/Collision を MuJoCo に変換して試す)")
    ap.add_argument("--scene", required=True, help="場面ファイル(止まるはずの位置の計算に使う)")
    ap.add_argument("--tol", type=float, default=0.03, help="止まる位置の許容の差 [m]")
    ap.add_argument("--strict", action="store_true", help="止まる位置の差と通り抜け(推定の形の精度)も合否に含める")
    ap.add_argument("--escape", action="store_true",
                    help="抜け出しの試験(多数の出発点 × 多方向。正解の部屋の外・家具の中に入ったら抜け出し)を行い、合否に含める")
    ap.add_argument("--escape-radius", type=float, default=0.08, help="抜け出しの試験のロボットの半径 [m]")
    ap.add_argument("--escape-spacing", type=float, default=0.3, help="抜け出しの試験の出発点の間隔 [m]")
    ap.add_argument("--escape-dirs", type=int, default=8, help="抜け出しの試験の押す方向の数")
    ap.add_argument("--escape-seconds", type=float, default=12.0, help="抜け出しの試験で押す時間 [秒]")
    ap.add_argument("--escape-only", action="store_true", help="抜け出しの試験だけを行う")
    a = ap.parse_args()
    try:
        import mujoco
    except ImportError:
        sys.exit("mujoco が見つからない(.venv に pip install mujoco)。MJCF の構造だけなら --xml を目で確かめる")
    print("MuJoCo %s" % mujoco.__version__)
    sc = twin_demo.parse_scene(a.scene)
    all_fails = []
    models = []
    if a.xml:
        with open(a.xml, encoding="utf-8") as f:
            models.append(("MJCF", f.read()))
    if a.usd:
        xml, nc = usd_to_mjcf(a.usd)
        print("[USD] /Room/Collision から凸の部品(箱・凸のメッシュ)%d 個を作った" % nc)
        models.append(("USD", xml))
    for label, xml in models:
        if not a.escape_only:
            f_, g_ = run(xml, sc, a.tol, a.strict, label)
            all_fails += f_
        if a.escape or a.escape_only:
            ne, nt, nb, ex = escape_test(xml, sc, a.escape_radius, a.escape_spacing, a.escape_dirs, a.escape_seconds, label)
            if ne:
                all_fails.append("[%s] 抜け出し %d / %d 回" % (label, ne, nt))
    if all_fails:
        print("FAIL: %s" % "; ".join(all_fails))
        sys.exit(1)
    if a.strict:
        print("PASS: 衝突判定が MuJoCo で効く(球が床で止まり、ロボットが正解の面の位置で止まる)")
    else:
        print("PASS: 衝突判定が MuJoCo で効く(球が床で止まり、めり込みが 1 cm 未満)。推定の形の精度は上の行を参照")


if __name__ == "__main__":
    main()

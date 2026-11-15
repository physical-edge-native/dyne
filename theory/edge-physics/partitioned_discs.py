"""区画で分担する2次元剛体(円板)の物理の使い捨て試作 (2026-11-11、設計書 docs/2026-11-11-edge-physics-design.md の根拠)

目的(実装ではなく、設計の判断材料):
  1. ペナルティ法(めり込みに比例するばね・ダンパの力)+半陰的オイラー法で、
     区画ごとに分担した計算が単一ノードの計算と**ビット一致**するための規約を確かめる。
       - 物体の持ち主 = その時点で中心がある区画のノード
       - 接触の力は (小さい番号, 大きい番号) の向きで1つの関数が計算する(両側のノードが同じ値を得る)
       - 物体ごとの力の和は、相手の番号の昇順という決まった順序で足す
       - 界面から一定の幅(のりしろ)にある物体の状態だけを隣へ写す(ゴースト)
  2. 1ステップあたりの演算の回数(接触の候補の判定、成立した接触、壁との判定、積分)と、
     ノード間で送るバイト数・相手の数(斜めの隣を含む)を数える。

壁・家具は格子に標本化した符号つき距離の場(sdf2d-grid-actor と同じ考え方、刻み h=2.5cm)を
双一次補間して引く。格子は解析的な距離から作る(試作なので測距からは作らない)。
Python の float は IEEE 754 の倍精度なので、ビット一致の判定は WASM の f64 と同じ意味を持つ。

使い方: .venv\\Scripts\\python.exe theory\\edge-physics\\partitioned_discs.py
"""
import math
import random
import struct
import sys

# ---------- 場面 (scenes/room6.txt に近い 3.6m x 2.7m の部屋。家具は箱2つ) ----------
W, H = 3.6, 2.7
BOXES = [(0.0, 0.0, 0.9, 0.6), (2.4, 1.9, 3.6, 2.7)]  # (x0,y0,x1,y1) 壁際の家具
HG = 0.025  # 距離の場の格子の刻み
NXG, NYG = int(round(W / HG)) + 1, int(round(H / HG)) + 1

DT = 0.002          # 物理の時間刻み [s]
K_N = 2.0e4         # 法線方向のばね [N/m]
C_N = 40.0          # 法線方向のダンパ [N s/m]
MU = 0.5            # クーロン摩擦係数
C_T = 30.0          # 接線方向の正則化した摩擦 (滑りの速さに比例、上限 MU*fn)
GRAV = 9.81
FLOOR_MU = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3  # 床との摩擦 (速さに比例する抵抗+クーロンで近似)
R_MAX = 0.15
HALO = 2 * R_MAX + 0.02  # のりしろの幅: 接触しうる相手の中心は 2*R_MAX 以内


def box_sd(px, py, b):
    x0, y0, x1, y1 = b
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    hx, hy = (x1 - x0) / 2, (y1 - y0) / 2
    dx, dy = abs(px - cx) - hx, abs(py - cy) - hy
    ox, oy = max(dx, 0.0), max(dy, 0.0)
    return math.hypot(ox, oy) + min(max(dx, dy), 0.0)


def make_grid():
    g = [[0.0] * NXG for _ in range(NYG)]
    for j in range(NYG):
        for i in range(NXG):
            x, y = i * HG, j * HG
            d = min(x, y, W - x, H - y)  # 部屋の内側までの距離 (壁)
            for b in BOXES:
                d = min(d, box_sd(x, y, b))
            g[j][i] = d
    return g


GRID = make_grid()
CNT = {"pair_check": 0, "contact": 0, "world_q": 0, "world_contact": 0, "integrate": 0, "drive": 0}


def sdf_query(x, y, touched=None):
    """双一次補間で φ と ∇φ を返す。touched に参照した格子の番号を記録する"""
    CNT["world_q"] += 1
    fx, fy = x / HG, y / HG
    i = min(max(int(math.floor(fx)), 0), NXG - 2)
    j = min(max(int(math.floor(fy)), 0), NYG - 2)
    tx, ty = fx - i, fy - j
    a, b = GRID[j][i], GRID[j][i + 1]
    c, d = GRID[j + 1][i], GRID[j + 1][i + 1]
    if touched is not None:
        touched.update([(i, j), (i + 1, j + 1)])
    phi = (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty
    gx = ((b - a) * (1 - ty) + (d - c) * ty) / HG
    gy = ((c - a) * (1 - tx) + (d - b) * tx) / HG
    return phi, gx, gy


class Body:
    __slots__ = ("id", "x", "y", "th", "vx", "vy", "w", "r", "m", "I", "robot")

    def __init__(self, id, x, y, r, m, robot=False):
        self.id, self.x, self.y, self.th = id, x, y, 0.0
        self.vx = self.vy = self.w = 0.0
        self.r, self.m, self.I, self.robot = r, m, 0.5 * m * r * r, robot

    def copy(self):
        b = Body(self.id, self.x, self.y, self.r, self.m, self.robot)
        b.th, b.vx, b.vy, b.w = self.th, self.vx, self.vy, self.w
        return b

    def key(self):
        return struct.pack("<i6d", self.id, self.x, self.y, self.th, self.vx, self.vy, self.w)


def pair_force(a, b):
    """a.id < b.id の向きで呼ぶ。a が受ける (fx, fy, τa) と b が受けるトルク τb を返す。接触なしなら None"""
    assert a.id < b.id
    CNT["pair_check"] += 1
    dx, dy = b.x - a.x, b.y - a.y
    d2 = dx * dx + dy * dy
    rs = a.r + b.r
    if d2 >= rs * rs:
        return None
    CNT["contact"] += 1
    d = math.sqrt(d2)
    nx, ny = dx / d, dy / d
    pen = rs - d
    # 接触点での相対速度 (b − a)
    # a の接触点 = a + r_a n、b の接触点 = b − r_b n。点の速さ = v + ω × (点 − 中心)
    rvx = (b.vx + b.w * b.r * ny) - (a.vx - a.w * a.r * ny)
    rvy = (b.vy - b.w * b.r * nx) - (a.vy + a.w * a.r * nx)
    vn = rvx * nx + rvy * ny
    fn = K_N * pen - C_N * vn
    if fn < 0.0:
        fn = 0.0
    tx, ty = -ny, nx
    vt = rvx * tx + rvy * ty
    ft = C_T * vt
    lim = MU * fn
    if ft > lim:
        ft = lim
    elif ft < -lim:
        ft = -lim
    # a が受ける力 (法線は a から b へ向くので a は −n 方向に押される)
    fx = -fn * nx + ft * tx
    fy = -fn * ny + ft * ty
    ta = a.r * (nx * fy - ny * fx)
    tb = b.r * (nx * fy - ny * fx)
    return fx, fy, ta, tb


def world_force(a, touched=None):
    phi, gx, gy = sdf_query(a.x, a.y, touched)
    pen = a.r - phi
    if pen <= 0.0:
        return None
    CNT["world_contact"] += 1
    gn = math.sqrt(gx * gx + gy * gy)
    nx, ny = gx / gn, gy / gn  # 壁から離れる向き
    vn = a.vx * nx + a.vy * ny
    fn = K_N * pen - C_N * vn
    if fn < 0.0:
        fn = 0.0
    tx, ty = -ny, nx
    vt = a.vx * tx + a.vy * ty - a.w * a.r
    ft = -C_T * vt
    lim = MU * fn
    ft = max(-lim, min(lim, ft))
    return fn * nx + ft * tx, fn * ny + ft * ty, -a.r * ft


def drive_force(a, act):
    """移動台車 (差動二輪を平面の1物体で近似): 前後の速さ・旋回の速さの指令に追従する力と、横滑りを止める力"""
    CNT["drive"] += 1
    v_cmd, w_cmd = act
    c, s = math.cos(a.th), math.sin(a.th)
    v_f = a.vx * c + a.vy * s
    v_l = -a.vx * s + a.vy * c
    f_f = 20.0 * (v_cmd - v_f)
    f_l = -60.0 * v_l
    tq = 0.8 * (w_cmd - a.w)
    return f_f * c - f_l * s, f_f * s + f_l * c, tq


def floor_force(a):
    sp = math.sqrt(a.vx * a.vx + a.vy * a.vy)
    if sp == 0.0:
        return 0.0, 0.0, -0.05 * a.w
    k = min(FLOOR_MU * a.m * GRAV, 5.0 * a.m * sp) / sp
    return -k * a.vx, -k * a.vy, -0.05 * a.w


def step_bodies(owned, visible, act, touched=None):
    """owned の各物体の新しい状態を返す。visible = owned ∪ ゴースト (id → Body)。
    力は (相手の id の昇順の接触) → 壁 → 床 → 駆動 の決まった順序で足す。"""
    ids = sorted(visible)
    out = []
    for a in sorted(owned, key=lambda b: b.id):
        fx = fy = tq = 0.0
        for jd in ids:
            if jd == a.id:
                continue
            o = visible[jd]
            if a.id < jd:
                f = pair_force(a, o)
                if f is not None:
                    fx += f[0]; fy += f[1]; tq += f[2]
            else:
                f = pair_force(o, a)
                if f is not None:
                    fx += -f[0]; fy += -f[1]; tq += f[3]
        f = world_force(a, touched)
        if f is not None:
            fx += f[0]; fy += f[1]; tq += f[2]
        f = floor_force(a)
        fx += f[0]; fy += f[1]; tq += f[2]
        if a.robot:
            f = drive_force(a, act)
            fx += f[0]; fy += f[1]; tq += f[2]
        CNT["integrate"] += 1
        n = a.copy()
        n.vx = a.vx + DT * fx / a.m
        n.vy = a.vy + DT * fy / a.m
        n.w = a.w + DT * tq / a.I
        n.x = a.x + DT * n.vx
        n.y = a.y + DT * n.vy
        n.th = a.th + DT * n.w
        out.append(n)
    return out


def make_bodies(n, seed):
    rng = random.Random(seed)
    bodies = [Body(0, 1.8, 1.2, 0.15, 3.0, robot=True)]
    while len(bodies) < n:
        r = rng.uniform(0.05, 0.12)
        x, y = rng.uniform(0.2, W - 0.2), rng.uniform(0.2, H - 0.2)
        cand = Body(len(bodies), x, y, r, rng.uniform(0.2, 1.0))
        if sdf_query(x, y)[0] < r + 0.02:
            continue
        if any(math.hypot(b.x - x, b.y - y) < b.r + r + 0.01 for b in bodies):
            continue
        vm = float(sys.argv[4]) if len(sys.argv) > 4 else 0.6
        cand.vx, cand.vy = rng.uniform(-vm, vm), rng.uniform(-vm, vm)
        bodies.append(cand)
    return bodies


def actions(steps, seed, hold=50):
    rng = random.Random(seed + 1)
    acts = []
    for s in range(steps):
        if s % hold == 0:
            cur = (rng.uniform(-0.3, 0.6), rng.uniform(-1.5, 1.5))
        acts.append(cur)
    return acts


def run_single(bodies, acts):
    st = {b.id: b.copy() for b in bodies}
    for act in acts:
        new = step_bodies(list(st.values()), st, act)
        st = {b.id: b for b in new}
    return st


def block_of(px, py, x, y):
    bx = min(max(int(x / (W / px)), 0), px - 1)
    by = min(max(int(y / (H / py)), 0), py - 1)
    return by * px + bx


def dist_to_block(px, py, k, x, y):
    bx, by = k % px, k // px
    x0, x1 = bx * W / px, (bx + 1) * W / px
    y0, y1 = by * H / py, (by + 1) * H / py
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)


def run_partitioned(bodies, acts, px, py):
    nn = px * py
    own = [dict() for _ in range(nn)]
    for b in bodies:
        own[block_of(px, py, b.x, b.y)][b.id] = b.copy()
    stats = {"ghost_msgs": 0, "ghost_bytes_max_link": 0, "diag_msgs": 0, "migrations": 0,
             "max_owned": 0, "max_visible": 0, "max_neighbors": 0, "halo_cells_ok": True}
    GHOST_B = 4 + 6 * 8  # id + (x,y,θ,vx,vy,ω) の倍精度
    for s, act in enumerate(acts):
        # 1. ゴーストの写し (界面の受け渡し、1ステップに1回)
        inbox = [dict() for _ in range(nn)]
        for k in range(nn):
            per_dest = {}
            for b in own[k].values():
                for q in range(nn):
                    if q != k and dist_to_block(px, py, q, b.x, b.y) <= HALO:
                        per_dest.setdefault(q, []).append(b)
            stats["max_neighbors"] = max(stats["max_neighbors"], len(per_dest))
            for q, lst in per_dest.items():
                stats["ghost_msgs"] += 1
                if abs(q % px - k % px) == 1 and abs(q // px - k // px) == 1:
                    stats["diag_msgs"] += 1
                stats["ghost_bytes_max_link"] = max(stats["ghost_bytes_max_link"], 24 + GHOST_B * len(lst))
                for b in lst:
                    inbox[q][b.id] = b
        # 2. 力と積分 (自分の物体だけ)
        new_own = [dict() for _ in range(nn)]
        for k in range(nn):
            vis = dict(own[k])
            vis.update(inbox[k])
            stats["max_owned"] = max(stats["max_owned"], len(own[k]))
            stats["max_visible"] = max(stats["max_visible"], len(vis))
            touched = set()
            for a in own[k].values():  # 界面をまたぐ接触 (相手がゴースト) の回数
                for g in inbox[k].values():
                    if (a.x - g.x) ** 2 + (a.y - g.y) ** 2 < (a.r + g.r) ** 2:
                        stats["cross_contacts"] = stats.get("cross_contacts", 0) + 1
            for nb in step_bodies(list(own[k].values()), vis, act, touched):
                # 3. 持ち主の移動 (中心が区画を出たら新しい持ち主へ全状態を渡す)
                q = block_of(px, py, nb.x, nb.y)
                if q != k:
                    stats["migrations"] += 1
                new_own[q][nb.id] = nb
            # 参照した距離の場の格子が、区画+1格子の帯に収まるか (既存の界面サンプル列で足りるか)
            bx, by = k % px, k // px
            i0, i1 = bx * W / px / HG - 1, (bx + 1) * W / px / HG + 1
            j0, j1 = by * H / py / HG - 1, (by + 1) * H / py / HG + 1
            for (i, j) in touched:
                if not (i0 - 1e-9 <= i <= i1 + 1e-9 and j0 - 1e-9 <= j <= j1 + 1e-9):
                    stats["halo_cells_ok"] = False
        own = new_own
    st = {}
    for k in range(nn):
        st.update(own[k])
    return st, stats


def main():
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 3000
    nb = int(sys.argv[2]) if len(sys.argv) > 2 else 24
    seed = 7
    bodies = make_bodies(nb, seed)
    acts = actions(steps, seed)
    for k in CNT:
        CNT[k] = 0
    ref = run_single(bodies, acts)
    single_cnt = dict(CNT)
    print(f"bodies={nb} steps={steps} dt={DT} (仮想 {steps*DT:.1f} 秒)")
    print("単一ノード: 1ステップあたり", {k: round(v / steps, 2) for k, v in single_cnt.items()})
    for px, py in [(2, 2), (4, 4)]:
        for k in CNT:
            CNT[k] = 0
        st, stats = run_partitioned(bodies, acts, px, py)
        same = all(st[i].key() == ref[i].key() for i in ref) and len(st) == len(ref)
        nn = px * py
        print(f"--- {px}x{py} = {nn} ノード: {'PASS (bit-exact)' if same else 'FAIL'}")
        print("  全ノード合計の1ステップあたり", {k: round(v / steps, 2) for k, v in CNT.items()})
        print("  ", {k: (round(v / steps, 3) if k in ('ghost_msgs', 'diag_msgs', 'migrations') else v)
                     for k, v in stats.items()},
              "(ghost_msgs・diag_msgs・migrations は全ノード合計の1ステップあたり)")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""少ない測距センサで部屋の形をどこまで作れるかの比較 (2026-11-11、docs/2026-11-11-sensor-coverage-study.md)。

何をするか:
  6 畳の部屋 (crates/dyne-sim2d/scenes/room6.txt と同じ家具) で、センサの台数・置き方・
  サーボで振る/手で回す場合を場面ファイル (crates/dyne-sim2d/scenes/study/) にして、
  既存の道具をそのまま使って比べる:
    1. dyne-sim2d + 形状の格子型アクター (sdf2d-grid-actor) で形状を推定 (CSV)
    2. scripts/twin_demo.py で USD / MJCF を作る
    3. scripts/usd_check.py (USD の形式) と scripts/mjcf_check.py --strict (MuJoCo での通り抜け)
  に加え、この台本の中で次を計算する:
    - 見える面の覆い率 (外壁・家具ごと)、推定表面の誤差 (平均・95%点)
    - 閉じているか: 推定の衝突判定の線 (マーチングスクエアの線分) を半径 0.15 m のロボットの分だけ太らせ、
      ロボットの出発点から塗りつぶして、正解の面の向こう (5 cm 以上) へ出られるか (未観測 = 空き として扱う場合)
    - 未観測 = 壁 として扱う場合に、ロボットが行ける床の面積が正解の何割か。観測済みの空きは
      「各区画の視野の錐 (幅 45/8 度) の、測距値の手前まで」とする (錐の中の 5 本の光線の最短距離)。
      これは観測の幾何から見た上限で、今のアクターの w (光線の中心線だけ) より広い
    - 上から見た図 (正解の形・推定表面・センサ・閉じていない所)

ツール (dyne-sim2d.exe / sdf2d_grid_actor.wasm) は --bin で渡す (他の作業と衝突しないよう写しを使える)。
使い方 (リポジトリのルートで):
  .venv\\Scripts\\python.exe scripts\\coverage_study.py --bin <exe と wasm のあるフォルダ> [--only F8G,PAN4]
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import twin_demo  # noqa: E402
import mjcf_check  # noqa: E402

SCN_DIR = os.path.join(ROOT, "crates", "dyne-sim2d", "scenes", "study")
OUT = os.path.join(ROOT, "demo_out", "coverage_study")
PY = sys.executable

HEADER = """room 0 0 3.6 2.7
box 0.0 0.0 1.1 0.55 0.72      # 机 (110x55cm、天板の高さ72cm)
box 2.6 0.0 3.6 1.95 0.45      # ベッド (100x195cm、高さ45cm)
box 0.0 2.35 0.9 2.7 1.8       # 本棚 (90x35cm、高さ180cm)
"""
FURN = ["机", "ベッド", "本棚"]
FOV = 45.0
R_ROBOT = 0.15

# 手で決めた置き方 B (部屋の空きの凹んだ角 7 か所 + 北の壁の中央。角の二等分線の向き)
CORNERS = [
    (0.05, 0.60, 45, "西の壁と机の角"),
    (1.15, 0.05, 45, "南の壁と机の角"),
    (2.55, 0.05, 135, "南の壁とベッドの角"),
    (3.55, 2.00, 135, "東の壁とベッドの角"),
    (3.55, 2.65, 225, "北東の角"),
    (0.95, 2.65, -45, "北の壁と本棚の角"),
    (0.05, 2.30, -45, "西の壁と本棚の角"),
    (1.80, 2.65, -90, "北の壁の中央"),
]
# 置き方 A = crates/dyne-sim2d/scenes/room6_move.txt のセンサ (壁の前から部屋の中を向く、実験6の仮置き)
WALLS = [
    (0.05, 1.0, 0, "西の壁の前から東"),
    (0.05, 2.0, -15, "西の壁の前 (本棚の手前) から東南東"),
    (1.3, 0.05, 75, "南の壁の前 (机とベッドの間) から北"),
    (2.45, 0.05, 120, "南の壁の前 (ベッドの脇) から北西"),
    (2.55, 1.0, 180, "ベッドの脇から西"),
    (3.5, 2.6, 205, "北東の角 (ベッドの足元) から南西"),
    (1.6, 2.65, -95, "北の壁の前から南"),
    (1.0, 2.65, -60, "北の壁の前 (本棚の脇) から南東"),
]


def scene_dict():
    os.makedirs(SCN_DIR, exist_ok=True)
    p = os.path.join(SCN_DIR, "_header.tmp")
    with open(p, "w", encoding="utf-8") as f:
        f.write(HEADER)
    sc = twin_demo.parse_scene(p)
    os.remove(p)
    return sc


# ---------------------------------------------------------------- 置き方の探索 (固定の 7/8 台)

def visible_samples(sc, h=0.025):
    """見える面の標本 (位置, 分類)。分類: 'wall' / 家具の番号"""
    out = []
    edges = twin_demo.scene_edges(sc)
    n_room = len(sc["room"])
    for ei, ((ax, ay), (bx, by)) in enumerate(edges):
        L = math.hypot(bx - ax, by - ay)
        n = max(1, int(L / h))
        ex, ey = (bx - ax) / L, (by - ay) / L
        cat = "wall" if ei < n_room else (ei - n_room) // 4
        for k in range(n):
            t = (k + 0.5) / n
            x, y = ax + t * (bx - ax), ay + t * (by - ay)
            f1 = twin_demo.is_free(sc, x - ey * 2 * h, y + ex * 2 * h)
            f2 = twin_demo.is_free(sc, x + ey * 2 * h, y - ex * 2 * h)
            out.append((x, y, cat, f1 != f2))
    return out


def sensor_hits(sc, x, y, yaw_deg, zones=8, fov=FOV, max_range=4.0):
    pts = []
    for c in range(zones):
        th = math.radians(yaw_deg) + twin_demo.zone_offset(c, zones, math.radians(fov))
        t = twin_demo.raycast(sc, x, y, math.cos(th), math.sin(th))
        if t is not None and t <= max_range:
            pts.append((x + t * math.cos(th), y + t * math.sin(th)))
    return pts


def greedy_placement(sc, n, radius=0.04):
    """見える面の標本を最も多く覆う置き方を貪欲に選ぶ (光線の当たり点から radius 以内を覆ったとみなす近似)"""
    vs = [(x, y) for x, y, c, v in visible_samples(sc) if v]
    V = np.array(vs)
    cands = []
    edges = twin_demo.scene_edges(sc)
    for (ax, ay), (bx, by) in edges:
        L = math.hypot(bx - ax, by - ay)
        ex, ey = (bx - ax) / L, (by - ay) / L
        for k in range(int(L / 0.15) + 1):
            t = min(max(k * 0.15, 0.06), L - 0.06) / L if L > 0.12 else 0.5
            x, y = ax + t * (bx - ax), ay + t * (by - ay)
            for nx_, ny_ in ((-ey, ex), (ey, -ex)):
                px, py = x + nx_ * 0.05, y + ny_ * 0.05
                if not twin_demo.is_free(sc, px, py) or min(px, 3.6 - px, py, 2.7 - py) < 0.04:
                    continue
                # 他の面にも近すぎないこと (5 cm 未満は角の外)
                if twin_demo.dist_to_segments(np.array([[px, py]]), twin_demo.truth_segments(sc))[0] < 0.045:
                    continue
                base = math.degrees(math.atan2(ny_, nx_))
                for d in range(-75, 76, 15):
                    hits = sensor_hits(sc, px, py, base + d)
                    if not hits:
                        continue
                    H = np.array(hits)
                    cov = (np.sqrt(((V[:, None, :] - H[None]) ** 2).sum(-1)).min(1) < radius)
                    cands.append(((round(px, 3), round(py, 3), round(base + d, 1)), cov))
    chosen, union = [], np.zeros(len(V), bool)
    for _ in range(n):
        best = max(cands, key=lambda c: (c[1] & ~union).sum())
        # 同じ位置に二つは置かない
        chosen.append(best[0])
        union |= best[1]
        cands = [c for c in cands if math.hypot(c[0][0] - best[0][0], c[0][1] - best[0][1]) > 0.3]
    return chosen, union.mean()


# ---------------------------------------------------------------- 場面ファイル

def write_scene(name, comment, sensors):
    os.makedirs(SCN_DIR, exist_ok=True)
    p = os.path.join(SCN_DIR, name + ".txt")
    with open(p, "w", encoding="utf-8") as f:
        f.write("# センサの台数・置き方の比較 (2026-11-11、docs/2026-11-11-sensor-coverage-study.md)。\n")
        f.write("# 部屋と家具は room6.txt と同じ。scripts/coverage_study.py が生成した。\n")
        for line in comment.strip().splitlines():
            f.write("# " + line + "\n")
        f.write(HEADER)
        for i, (x, y, yaw, note) in enumerate(sensors):
            f.write("sensor %d %.3f %.3f %.1f 0.1%s\n" % (i, x, y, yaw, ("    # " + note) if note else ""))
    return p


def pan_sensors(mounts, k):
    """サーボで振る: 取り付け位置ごとに k 方向 (k=4: 内向きの 180 度、k=8: 全周)。同じ位置の向き違いのセンサとして書く"""
    out = []
    for (x, y, yaw, note) in mounts:
        if k == 8:
            yaws = [yaw + 45 * j for j in range(8)]
        else:
            yaws = [yaw - 67.5 + 45 * j for j in range(4)]
        for j, yw in enumerate(yaws):
            out.append((x, y, ((yw + 180) % 360) - 180, "%s (向き %d/%d)" % (note, j + 1, k)))
    return out


def make_configs():
    sc = scene_dict()
    cfgs = {}
    cfgs["REF"] = dict(scene=os.path.join(ROOT, "crates", "dyne-sim2d", "scenes", "room6.txt"),
                       sim=["--px", "8", "--py", "4", "--zones", "64", "--fov-deg", "60", "--noise-m", "0.002"],
                       zones=64, fov=60, label="参照: 54台 × 64方向 (視野60度、雑音2 mm)", time="-")
    real = ["--px", "4", "--py", "2", "--zones", "8", "--fov-deg", "45", "--noise-m", "0.008"]
    cfgs["F8A"] = dict(scene=write_scene("f8_walls", "固定 8 台、置き方 A (room6_move.txt と同じ、壁の前から部屋の中を向く)", WALLS),
                       sim=real, zones=8, fov=45, label="固定8台 A (壁の前、room6_move と同じ)", time="即時 (1フレーム)")
    cfgs["F8B"] = dict(scene=write_scene("f8_corners", "固定 8 台、置き方 B (空きの凹んだ角 7 か所 + 北の壁の中央、角の二等分線を向く)", CORNERS),
                       sim=real, zones=8, fov=45, label="固定8台 B (角から斜め)", time="即時")
    g8, prox8 = greedy_placement(sc, 8)
    g = [(x, y, yaw, "貪欲法の %d 番目" % (i + 1)) for i, (x, y, yaw) in enumerate(g8)]
    cfgs["F8G"] = dict(scene=write_scene("f8_greedy", "固定 8 台、置き方 G (見える面の覆いを貪欲法で最大化した置き方。近似の覆い率 %.0f%%)" % (prox8 * 100), g),
                       sim=real, zones=8, fov=45, label="固定8台 G (貪欲法で最適化)", time="即時")
    cfgs["F7G"] = dict(scene=write_scene("f7_greedy", "固定 7 台、置き方 G の最初の 7 台", g[:7]),
                       sim=real, zones=8, fov=45, label="固定7台 G", time="即時")
    m7 = CORNERS[:7]
    cfgs["PAN4"] = dict(scene=write_scene("pan4", "7 か所 (置き方 B の角) × サーボで 4 方向 (45 度おき、内向きの 180 度)。\n静止した部屋なので、同じ位置の向き違いのセンサが同時に測るのと同じ", pan_sensors(m7, 4)),
                        sim=real, zones=8, fov=45, label="7台 × サーボ4方向 (180度)", time="4位置 × 0.5 s = 2 s")
    cfgs["PAN8"] = dict(scene=write_scene("pan8", "7 か所 (置き方 B の角) × サーボで 8 方向 (45 度おき、全周)", pan_sensors(m7, 8)),
                        sim=real, zones=8, fov=45, label="7台 × サーボ8方向 (360度)", time="8位置 × 0.5 s = 4 s")
    # 細かく振る: 45 度の窓の中を区画の 1/4 (1.4 度) ずつずらす = 1 台 32 区画として模擬 (4 × 4 = 16 位置)
    fine = ["--px", "4", "--py", "2", "--zones", "32", "--fov-deg", "45", "--noise-m", "0.008"]
    cfgs["PAN4F"] = dict(scene=cfgs["PAN4"]["scene"], sim=fine, zones=32, fov=45,
                         label="7台 × サーボ16位置 (180度を1.4度刻みの細かい振り)", time="16位置 × 0.5 s = 8 s")
    half = ["--px", "4", "--py", "2", "--zones", "16", "--fov-deg", "45", "--noise-m", "0.008"]
    cfgs["PAN4H"] = dict(scene=cfgs["PAN4"]["scene"], sim=half, zones=16, fov=45,
                         label="7台 × サーボ8位置 (180度を2.8度刻み)", time="8位置 × 0.5 s = 4 s")
    m4 = [CORNERS[i] for i in (0, 2, 4, 6)]
    cfgs["PAN4F_4"] = dict(scene=write_scene("pan4_4mounts", "4 か所 (置き方 B の角 1,3,5,7) × サーボで 4 方向。細かく振る場合 (--zones 32) に使う", pan_sensors(m4, 4)),
                           sim=fine, zones=32, fov=45, label="4台 × サーボ16位置 (細かい振り)", time="16位置 × 0.5 s = 8 s")
    return cfgs, sc


# ---------------------------------------------------------------- 実行

def run(cmd, log=None):
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace", env=env)
    out = r.stdout + r.stderr
    if log:
        with open(log, "w", encoding="utf-8") as f:
            f.write(out)
    return r.returncode, out


def seg_hits(segs_arr, ox, oy, vx, vy):
    a = segs_arr[:, 0, :]
    e = segs_arr[:, 1, :] - a
    den = vx * e[:, 1] - vy * e[:, 0]
    w = a - np.array([ox, oy])
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (w[:, 0] * e[:, 1] - w[:, 1] * e[:, 0]) / den
        u = (w[:, 0] * vy - w[:, 1] * vx) / den
    ok = (np.abs(den) > 1e-15) & (u >= 0) & (u <= 1) & (t > 1e-9)
    return t[ok].min() if ok.any() else None


def disk_dilate(m, r_cells):
    out = m.copy()
    R = int(math.ceil(r_cells))
    ny, nx = m.shape
    for dy in range(-R, R + 1):
        for dx in range(-R, R + 1):
            if dx * dx + dy * dy > r_cells * r_cells:
                continue
            ys0, ys1 = max(0, dy), min(ny, ny + dy)
            xs0, xs1 = max(0, dx), min(nx, nx + dx)
            out[ys0:ys1, xs0:xs1] |= m[ys0 - dy:ys1 - dy, xs0 - dx:xs1 - dx]
    return out


def flood(free, start):
    reach = np.zeros_like(free)
    if not free[start]:
        return reach
    reach[start] = True
    while True:
        n = reach.copy()
        n[1:, :] |= reach[:-1, :]
        n[:-1, :] |= reach[1:, :]
        n[:, 1:] |= reach[:, :-1]
        n[:, :-1] |= reach[:, 1:]
        n &= free
        if (n == reach).all():
            return reach
        reach = n


class FineGrid:
    def __init__(self, sc, step=0.02, margin=0.4):
        self.step = step
        self.x0, self.y0 = -margin, -margin
        self.nx = int(round((3.6 + 2 * margin) / step)) + 1
        self.ny = int(round((2.7 + 2 * margin) / step)) + 1
        xs = self.x0 + step * np.arange(self.nx)
        ys = self.y0 + step * np.arange(self.ny)
        self.X, self.Y = np.meshgrid(xs, ys)
        pts = np.c_[self.X.ravel(), self.Y.ravel()]
        free = np.array([twin_demo.is_free(sc, x, y) for x, y in pts]).reshape(self.X.shape)
        d = twin_demo.dist_to_segments(pts, twin_demo.truth_segments(sc)).reshape(self.X.shape)
        self.sdf = np.where(free, d, -d)  # 正解の符号つき距離 (空きで正)

    def idx(self, x, y):
        return (int(round((y - self.y0) / self.step)), int(round((x - self.x0) / self.step)))

    def raster(self, segs):
        m = np.zeros(self.X.shape, bool)
        for (p, q) in segs:
            L = math.hypot(q[0] - p[0], q[1] - p[1])
            n = max(2, int(L / (self.step * 0.25)) + 1)
            for t in np.linspace(0, 1, n):
                j, i = self.idx(p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]))
                if 0 <= j < self.ny and 0 <= i < self.nx:
                    m[j, i] = True
        return m

    def cones_free(self, sc, sensors, zones, fov, width):
        """各区画の視野の錐 (幅 width 度) の、測距値 (錐の中の 5 本の光線の最短) の 2 cm 手前までを空きとする"""
        known = np.zeros(self.X.shape, bool)
        zw = math.radians(width)
        for s in sensors:
            for c in range(zones):
                th0 = s["yaw"] + twin_demo.zone_offset(c, zones, math.radians(fov))
                rng = min((twin_demo.raycast(sc, s["x"], s["y"], math.cos(th0 + o), math.sin(th0 + o)) or 4.0)
                          for o in np.linspace(-zw / 2, zw / 2, 5))
                rng = min(rng, 4.0) - 0.02
                dx, dy = self.X - s["x"], self.Y - s["y"]
                r = np.hypot(dx, dy)
                ang = np.abs((np.arctan2(dy, dx) - th0 + np.pi) % (2 * np.pi) - np.pi)
                known |= (r <= rng) & (ang <= zw / 2)
        return known


def analyse(name, cfg, sc, fg, odir):
    g = twin_demo.read_shape(os.path.join(odir, "shape.csv"))
    h = g["h"]
    segs = twin_demo.marching_squares(g, 3 * h)
    S = np.array([(p, q) for (p, q, _, _) in segs]) if segs else np.zeros((0, 2, 2))
    res = {}
    # 誤差
    if len(S):
        err = twin_demo.dist_to_segments(S.mean(1), twin_demo.truth_segments(sc))
        res["err_mean_mm"] = float(err.mean() * 1000)
        res["err_p95_mm"] = float(np.percentile(err, 95) * 1000)
    # 覆い率 (分類ごと)
    vs = visible_samples(sc, h)
    allpts = S.reshape(-1, 2)
    cov = {}
    for cat in ["wall", 0, 1, 2]:
        P = np.array([(x, y) for x, y, c, v in vs if v and c == cat])
        if len(allpts):
            d = np.sqrt(((P[:, None, :] - allpts[None]) ** 2).sum(-1)).min(1)
            cov[cat] = float((d < 1.5 * h).mean() * 100)
        else:
            cov[cat] = 0.0
    Pall = np.array([(x, y) for x, y, c, v in vs if v])
    d = np.sqrt(((Pall[:, None, :] - allpts[None]) ** 2).sum(-1)).min(1) if len(allpts) else np.full(len(Pall), 9.0)
    res["cov_all"] = float((d < 1.5 * h).mean() * 100)
    res["cov"] = {("外壁" if k == "wall" else FURN[k]): v for k, v in cov.items()}
    # 閉じているか (未観測 = 空き)
    start, _ = mjcf_check.free_point(sc)
    st = fg.idx(*start)
    blocked = disk_dilate(fg.raster([(p, q) for p, q in S]), R_ROBOT / fg.step)
    reach_free = flood(~blocked, st)
    truth_ok = flood(fg.sdf >= R_ROBOT, st)  # 正解の部屋でロボットの中心が行ける所
    escape = reach_free & (fg.sdf < R_ROBOT - 0.05)
    res["escape_area_m2"] = float(escape.sum() * fg.step ** 2)
    res["closed"] = bool(escape.sum() == 0)
    # 未観測 = 壁
    sens = twin_demo.parse_scene(cfg["scene"])["sensors"]
    # 実際の区画の幅は 45/8 度 (細かく振る場合も錐は 5.6 度で、向きが 1.4 度ずつずれる)。参照は 60/64 度
    width = cfg["fov"] / cfg["zones"] if name == "REF" else FOV / 8
    known = fg.cones_free(sc, sens, cfg["zones"], cfg["fov"], width)
    blocked_w = disk_dilate(~known | fg.raster([(p, q) for p, q in S]), R_ROBOT / fg.step)
    reach_w = flood(~blocked_w, st)
    res["reach_wall_pct"] = float((reach_w & truth_ok).sum() / truth_ok.sum() * 100)
    res["truth_area_m2"] = float(truth_ok.sum() * fg.step ** 2)
    res["start"] = start
    return res, S, escape, reach_w, truth_ok, known


def draw(name, cfg, sc, S, escape, reach_w, truth_ok, known, res, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Wedge
    matplotlib.rcParams["font.family"] = ["Yu Gothic", "MS Gothic", "sans-serif"]
    fig, axs = plt.subplots(1, 2, figsize=(13, 5.2))
    sens = twin_demo.parse_scene(cfg["scene"])["sensors"]
    ext = (fg_g.x0 - fg_g.step / 2, fg_g.x0 + fg_g.step * (fg_g.nx - 0.5), fg_g.y0 - fg_g.step / 2, fg_g.y0 + fg_g.step * (fg_g.ny - 0.5))
    for k, ax in enumerate(axs):
        ax.set_aspect("equal")
        ax.set_xlim(-0.3, 3.9)
        ax.set_ylim(-0.3, 3.0)
        if k == 0:
            img = np.zeros(escape.shape + (4,))
            img[escape] = (0.95, 0.2, 0.2, 0.55)
            ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")
        else:
            img = np.zeros(reach_w.shape + (4,))
            img[known] = (0.75, 0.85, 1.0, 0.5)
            img[truth_ok & ~reach_w] = (1.0, 0.8, 0.4, 0.45)
            img[reach_w] = (0.3, 0.75, 0.35, 0.6)
            ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")
        ax.add_patch(Polygon(sc["room"], closed=True, fill=False, ec="0.55", lw=6, alpha=0.35))
        for p in sc["polys"]:
            ax.add_patch(Polygon(p, closed=True, fc="0.85", ec="0.55", lw=6, alpha=0.5))
        for (p, q) in S:
            ax.plot([p[0], q[0]], [p[1], q[1]], color="#1f4fd1", lw=1.8, solid_capstyle="butt")
        seen = set()
        for s in sens:
            key = (round(s["x"], 3), round(s["y"], 3))
            if k == 0 and len(sens) <= 8:
                ax.add_patch(Wedge((s["x"], s["y"]), 0.45, math.degrees(s["yaw"]) - cfg["fov"] / 2,
                                   math.degrees(s["yaw"]) + cfg["fov"] / 2, fc="#d14f1f", alpha=0.18))
            ax.annotate("", xy=(s["x"] + 0.18 * math.cos(s["yaw"]), s["y"] + 0.18 * math.sin(s["yaw"])),
                        xytext=(s["x"], s["y"]), arrowprops=dict(arrowstyle="->", color="#d14f1f", lw=1.2))
            if key not in seen:
                ax.plot(*key, "o", color="#d14f1f", ms=6)
                seen.add(key)
        ax.plot(*res["start"], "*", color="k", ms=12)
    axs[0].set_title("推定表面 (青) と正解 (灰)。赤 = ロボットが正解の面の向こうへ出られる所\n覆い率 %.0f%% / %s"
                     % (res["cov_all"], "閉じている" if res["closed"] else "閉じていない (通り抜け %.2f m²)" % res["escape_area_m2"]), fontsize=10)
    axs[1].set_title("未観測 = 壁 とした場合: 緑 = ロボットが行ける所、橙 = 本当は行けるが行けない所\n水色 = 視野の錐で観測済みの空き。行ける床 %.0f%%"
                     % res["reach_wall_pct"], fontsize=10)
    fig.suptitle("%s: %s" % (name, cfg["label"]), fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def angle_study(cfgs, sc, names, sigmas=(0.5, 1.0, 2.0, 3.0), trials=200, seed=1):
    """向きの誤差 (サーボの角度の誤差・手測りの向きの誤差) が表面の位置の誤差になる大きさ (光線だけの計算)。
    向きごとに一つの誤差 δ ~ N(0, σ) (同じ向きの 8 区画は同じだけずれる)。測距値は本当の向きの光線で測った値、
    点は名目の向きに置く。点から正解の面までの距離を集める (センサの雑音は含めない)"""
    rng = np.random.default_rng(seed)
    T = twin_demo.truth_segments(sc)
    out = {}
    for name in names:
        cfg = cfgs[name]
        sens = twin_demo.parse_scene(cfg["scene"])["sensors"]
        zones = cfg["zones"]
        res = {}
        for sg in sigmas:
            errs = []
            for _ in range(max(1, trials // len(sens))):
                for s in sens:
                    dlt = math.radians(rng.normal(0, sg))
                    for c in range(zones):
                        th = s["yaw"] + twin_demo.zone_offset(c, zones, math.radians(cfg["fov"]))
                        t = twin_demo.raycast(sc, s["x"], s["y"], math.cos(th + dlt), math.sin(th + dlt))
                        if t is None or t > 4.0:
                            continue
                        errs.append((s["x"] + t * math.cos(th), s["y"] + t * math.sin(th)))
            d = twin_demo.dist_to_segments(np.array(errs), T)
            res["%.1f" % sg] = (float(d.mean() * 1000), float(np.percentile(d, 95) * 1000))
        out[name] = res
        print(name, json.dumps(res), flush=True)
    with open(os.path.join(OUT, "angle_study.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)


def face_study(cfgs, sc, names):
    """面 (外壁の見える区間・家具の見える面) ごとに、光線の当たり点がいくつ・どれだけ離れて付くか。
    「部屋は直線の壁と箱の家具でできている」という前提で面を当てはめる後処理が使えるかの目安
    (2 点以上・20 cm 以上離れて当たれば、その面の直線を決められる)"""
    edges = twin_demo.scene_edges(sc)
    vs = visible_samples(sc)
    # 見える区間がある辺だけ
    vis_edges = []
    for ei, e in enumerate(edges):
        (ax, ay), (bx, by) = e
        L = math.hypot(bx - ax, by - ay)
        n = max(1, int(L / 0.025))
        flags = vs[sum(max(1, int(math.hypot(q[0] - p[0], q[1] - p[1]) / 0.025)) for p, q in edges[:ei]):][:n]
        if any(f[3] for f in flags):
            vis_edges.append((ei, e))
    out = {}
    for name in names:
        cfg = cfgs[name]
        sens = twin_demo.parse_scene(cfg["scene"])["sensors"]
        hits = []
        for s in sens:
            hits += sensor_hits(sc, s["x"], s["y"], math.degrees(s["yaw"]), cfg["zones"], cfg["fov"])
        H = np.array(hits)
        ok = 0
        detail = []
        for ei, ((ax, ay), (bx, by)) in vis_edges:
            seg = np.array([[[ax, ay], [bx, by]]])
            d = twin_demo.dist_to_segments(H, seg)
            on = H[d < 1e-6]
            L = math.hypot(bx - ax, by - ay)
            span = 0.0
            if len(on) >= 2:
                tt = ((on[:, 0] - ax) * (bx - ax) + (on[:, 1] - ay) * (by - ay)) / L
                span = float(tt.max() - tt.min())
            good = len(on) >= 2 and span >= 0.2
            ok += good
            detail.append((ei, len(on), round(span, 2)))
        out[name] = {"faces_ok": ok, "faces": len(vis_edges), "detail": detail}
        print(name, ok, "/", len(vis_edges), detail, flush=True)
    with open(os.path.join(OUT, "face_study.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)


def main():
    global fg_g
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True, help="dyne-sim2d.exe と sdf2d_grid_actor.wasm のあるフォルダ")
    ap.add_argument("--only", default="")
    ap.add_argument("--steps", type=int, default=40)
    ap.add_argument("--skip-sim", action="store_true")
    ap.add_argument("--angle-study", action="store_true")
    ap.add_argument("--face-study", action="store_true")
    a = ap.parse_args()
    exe = os.path.join(a.bin, "dyne-sim2d.exe")
    wasm = os.path.join(a.bin, "sdf2d_grid_actor.wasm")
    cfgs, sc = make_configs()
    fg = FineGrid(sc)
    fg_g = fg
    names = [n for n in cfgs if not a.only or n in a.only.split(",")]
    if a.face_study:
        face_study(cfgs, sc, names)
        return
    if a.angle_study:
        angle_study(cfgs, sc, names)
        return
    summary = {}
    sp = os.path.join(OUT, "summary.json")
    if os.path.exists(sp):
        with open(sp, encoding="utf-8") as f:
            summary = json.load(f)
    for name in names:
        cfg = cfgs[name]
        odir = os.path.join(OUT, name)
        os.makedirs(odir, exist_ok=True)
        print("== %s (%s)" % (name, cfg["label"]), flush=True)
        if not a.skip_sim:
            rc, out = run([exe, "--wasm", wasm, "--scene", cfg["scene"], "--steps", str(a.steps), "--h", "0.025"] + cfg["sim"]
                          + ["--field-csv", os.path.join(odir, "shape.csv")], os.path.join(odir, "sim.log"))
            if rc != 0:
                print(out[-2000:])
                continue
            rc, out = run([PY, os.path.join(HERE, "twin_demo.py"), "--scene", cfg["scene"], "--shape-csv", os.path.join(odir, "shape.csv"),
                           "--sim-log", os.path.join(odir, "sim.log"), "--out", os.path.join(odir, "room"), "--zones", str(cfg["zones"]),
                           "--fov-deg", str(cfg["fov"]), "--truth-usd"], os.path.join(odir, "twin_demo.log"))
            if rc != 0:
                print(out[-2000:])
                continue
            run([PY, os.path.join(HERE, "usd_check.py"), os.path.join(odir, "room.usda"), "--scene", cfg["scene"]], os.path.join(odir, "usd_check.log"))
            run([PY, os.path.join(HERE, "mjcf_check.py"), "--usd", os.path.join(odir, "room.usda"), "--scene", cfg["scene"], "--strict"],
                os.path.join(odir, "mjcf_check.log"))
        res, S, escape, reach_w, truth_ok, known = analyse(name, cfg, sc, fg, odir)
        simlog = open(os.path.join(odir, "sim.log"), encoding="utf-8").read()
        res["bitexact"] = "PASS (bit-exact)" in simlog
        ul = open(os.path.join(odir, "usd_check.log"), encoding="utf-8").read().strip().splitlines()
        res["usd_check"] = ul[-1] if ul else ""
        ml = open(os.path.join(odir, "mjcf_check.log"), encoding="utf-8").read()
        res["mjcf_check"] = ml.strip().splitlines()[-1] if ml.strip() else ""
        res["mjcf_gaps"] = re.findall(r"ロボット ([+-][xy]): 正解の面", ml)
        res["n_sensors"] = len(twin_demo.parse_scene(cfg["scene"])["sensors"])
        res["label"] = cfg["label"]
        res["time"] = cfg["time"]
        res["scene"] = os.path.relpath(cfg["scene"], ROOT)
        draw(name, cfg, sc, S, escape, reach_w, truth_ok, known, res, os.path.join(OUT, name + ".png"))
        summary[name] = res
        print(json.dumps(res, ensure_ascii=False), flush=True)
        with open(sp, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()

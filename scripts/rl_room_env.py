#!/usr/bin/env python3
"""Dyne が書き出した部屋(MuJoCo の模型 room.xml)で、車輪 2 つの移動台車が目標地点へ進む RL の課題
(Gymnasium の形の環境。CPU で動く)。2026-11-13。計画の G8 / 実験7 の保険(docs/2026-11-13-pc-pipeline-rl.md)。

部屋:   scripts/twin_demo.py が書いた room.xml(床・壁・家具・未観測を物とみなした箱。どれも静的な箱)。
        RL の環境は部屋の形を計算しない。形はノード群が計算した結果を書き出したもの。
台車:   差動駆動(左右の車輪の速さの差で曲がる)。寸法は Isaac Sim の Jetbot に寄せた仮の値(要確認):
          車輪の半径 0.0325 m、左右の車輪の間隔 0.1125 m、車輪の最大の角速度 10 rad/s(最大 0.325 m/s)、
          衝突判定は半径 0.08 m・高さ 0.10 m の円柱(床から 1 cm 浮かせる)。
        車輪と床の摩擦は計算しない(平面の 3 自由度 x・y・向きの関節に、車輪の速さから求めた速度を
        速度の追従(上限の力つき)で与える)。壁・家具に当たれば、接触の力で止まる。
行動:   左右の車輪の角速度 [-1, 1]²(最大値で割ったもの)
観測:   目標の位置(台車から見た前・左、2 m で割る)、目標までの距離、今の前進の速さ・回る速さ、
        台車の周りの 16 方向の距離(MuJoCo の光線の計算、2 m で打ち切り、2 m で割る)。計 21 個
報酬:   目標へ近づいた距離 × 10 − 0.01(1 手ごと)− 0.05(壁・家具に触れている間)、到達で +5 して終える
        (到達 = 目標まで 0.15 m 以内)。200 手(1 手 0.1 秒 = 20 秒)で打ち切り。
出発点と目標: 部屋の格子の範囲(room.xml の <custom> の dyne_grid_extent)の中で、台車を置いても 3 cm 以内に
        何も無い点を 0.1 m おきに調べ、つながった最大の領域から選ぶ(目標までの直線距離 0.8〜2.5 m)。

使い方:
  .venv\\Scripts\\python.exe scripts\\rl_room_env.py --xml demo_out\\pc_pipeline\\room6\\room.xml --bench 20000
"""
import argparse
import math
import os
import time
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # gymnasium が無くても --bench は動くようにする
    gym = None

import mujoco

WHEEL_R = 0.0325     # 要確認(Jetbot)
WHEEL_BASE = 0.1125  # 要確認(Jetbot)
WHEEL_MAX = 10.0     # rad/s、要確認
ROBOT_R = 0.08       # 衝突判定の円柱の半径、要確認
ROBOT_HALF_H = 0.05
N_RAYS = 16
RAY_MAX = 2.0
CTRL_DT = 0.1
SIM_DT = 0.01
GOAL_TOL = 0.15
MAX_STEPS = 200


def build_model(xml_text):
    root = ET.fromstring(xml_text)
    opt = root.find("option")
    if opt is not None:
        opt.set("timestep", str(SIM_DT))
    wb = root.find("worldbody")
    b = ET.SubElement(wb, "body", name="robot", pos="0 0 %g" % (ROBOT_HALF_H + 0.01))
    ET.SubElement(b, "joint", name="rx", type="slide", axis="1 0 0")
    ET.SubElement(b, "joint", name="ry", type="slide", axis="0 1 0")
    ET.SubElement(b, "joint", name="rth", type="hinge", axis="0 0 1")
    ET.SubElement(b, "geom", name="robot_geom", type="cylinder", size="%g %g" % (ROBOT_R, ROBOT_HALF_H), mass="1.0",
                  contype="2", conaffinity="1", rgba="0.2 0.8 0.3 1")
    ET.SubElement(b, "geom", name="robot_nose", type="box", size="0.02 0.01 0.01", pos="%g 0 0.04" % (ROBOT_R - 0.02),
                  contype="0", conaffinity="0", mass="0.001", rgba="0.9 0.9 0.2 1")
    goal = ET.SubElement(wb, "body", name="goal", mocap="true", pos="0 0 0.01")
    ET.SubElement(goal, "geom", name="goal_geom", type="cylinder", size="%g 0.005" % GOAL_TOL, contype="0",
                  conaffinity="0", rgba="1 0.3 0.3 0.4")
    act = root.find("actuator")
    if act is None:
        act = ET.SubElement(root, "actuator")
    for j, kv, fr in (("rx", 60.0, 30.0), ("ry", 60.0, 30.0), ("rth", 2.0, 2.0)):
        ET.SubElement(act, "velocity", name="v_" + j, joint=j, kv=str(kv), forcelimited="true", forcerange="%g %g" % (-fr, fr))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


class RoomNavEnv(gym.Env if gym else object):
    metadata = {"render_modes": []}

    def __init__(self, xml_path, seed=None, max_steps=MAX_STEPS):
        with open(xml_path, encoding="utf-8") as f:
            self.m = build_model(f.read())
        self.d = mujoco.MjData(self.m)
        self.max_steps = max_steps
        self.nsub = int(round(CTRL_DT / self.m.opt.timestep))
        self.qx, self.qy, self.qt = (self.m.joint(n).qposadr[0] for n in ("rx", "ry", "rth"))
        self.vx, self.vy, self.vt = (self.m.joint(n).dofadr[0] for n in ("rx", "ry", "rth"))
        self.robot_geom = self.m.geom("robot_geom").id
        self.robot_body = self.m.body("robot").id
        self.floor_geom = self.m.geom("Floor").id   # 床(平面)との接触は数えない(台車は床から 1 cm 浮いている)
        self.goal_mocap = self.m.body("goal").mocapid[0]
        ext = None
        for k in range(self.m.nnumeric):
            if self.m.numeric(k).name == "dyne_grid_extent":
                ext = [float(v) for v in self.m.numeric(k).data]
        if ext is None:
            raise ValueError("room.xml に dyne_grid_extent が無い(2026-11-13 以降の twin_demo.py で書き出すこと)")
        self.extent = ext
        self.rng = np.random.default_rng(seed)
        self.free = self._free_points()
        ang = np.arange(N_RAYS) * 2 * math.pi / N_RAYS
        self.ray_dirs = np.c_[np.cos(ang), np.sin(ang)]
        if gym:
            self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)
            self.observation_space = spaces.Box(-np.inf, np.inf, (5 + N_RAYS,), np.float32)

    # ---- 置ける点
    def _place(self, x, y, th=0.0):
        self.d.qpos[self.qx], self.d.qpos[self.qy], self.d.qpos[self.qt] = x, y, th
        self.d.qvel[:] = 0

    def _touching(self, margin=0.0):
        for c in range(self.d.ncon):
            con = self.d.contact[c]
            if self.floor_geom in (con.geom1, con.geom2):
                continue
            if (con.geom1 == self.robot_geom or con.geom2 == self.robot_geom) and con.dist < margin:
                return True
        return False

    def _free_points(self, step=0.1, clearance=0.03):
        x0, y0, x1, y1 = self.extent
        xs = np.arange(x0 + step / 2, x1, step)
        ys = np.arange(y0 + step / 2, y1, step)
        old = self.m.geom_margin[self.robot_geom]
        self.m.geom_margin[self.robot_geom] = clearance
        ok = np.zeros((len(ys), len(xs)), bool)
        for j, y in enumerate(ys):
            for i, x in enumerate(xs):
                self._place(x, y)
                mujoco.mj_forward(self.m, self.d)
                ok[j, i] = not self._touching(clearance)
        self.m.geom_margin[self.robot_geom] = old
        # つながった最大の領域(4 近傍)
        lab = -np.ones(ok.shape, int)
        best, best_n, n_lab = -1, 0, 0
        for j in range(ok.shape[0]):
            for i in range(ok.shape[1]):
                if ok[j, i] and lab[j, i] < 0:
                    q = deque([(j, i)])
                    lab[j, i] = n_lab
                    cnt = 0
                    while q:
                        a, b = q.popleft()
                        cnt += 1
                        for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                            u, v = a + da, b + db
                            if 0 <= u < ok.shape[0] and 0 <= v < ok.shape[1] and ok[u, v] and lab[u, v] < 0:
                                lab[u, v] = n_lab
                                q.append((u, v))
                    if cnt > best_n:
                        best, best_n = n_lab, cnt
                    n_lab += 1
        if best < 0:
            raise ValueError("台車を置ける点が無い")
        jj, ii = np.nonzero(lab == best)
        self.free_stats = {"candidates": int(ok.size), "free": int(ok.sum()), "largest": int(best_n), "regions": n_lab}
        return np.c_[xs[ii], ys[jj]]

    # ---- 観測
    def _rays(self, x, y, th):
        c, s = math.cos(th), math.sin(th)
        out = np.empty(N_RAYS)
        p = np.array([x, y, ROBOT_HALF_H + 0.01])
        gid = np.zeros(1, np.int32)
        for k, (dx, dy) in enumerate(self.ray_dirs):
            v = np.array([c * dx - s * dy, s * dx + c * dy, 0.0])
            t = mujoco.mj_ray(self.m, self.d, p, v, None, 1, self.robot_body, gid)
            out[k] = RAY_MAX if (t < 0 or t > RAY_MAX) else t
        return out

    def _obs(self):
        x, y, th = self.d.qpos[self.qx], self.d.qpos[self.qy], self.d.qpos[self.qt]
        gx, gy = self.goal
        dx, dy = gx - x, gy - y
        c, s = math.cos(th), math.sin(th)
        fwd, left = c * dx + s * dy, -s * dx + c * dy
        dist = math.hypot(dx, dy)
        v = c * self.d.qvel[self.vx] + s * self.d.qvel[self.vy]
        w = self.d.qvel[self.vt]
        o = np.r_[fwd / 2.0, left / 2.0, min(dist, 4.0) / 2.0, v / (WHEEL_R * WHEEL_MAX),
                  w / (2 * WHEEL_R * WHEEL_MAX / WHEEL_BASE), self._rays(x, y, th) / RAY_MAX]
        return o.astype(np.float32), dist

    # ---- Gymnasium の口
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.m, self.d)
        for _ in range(100):
            a, b = self.rng.integers(len(self.free), size=2)
            dd = np.linalg.norm(self.free[a] - self.free[b])
            if 0.8 <= dd <= 2.5:
                break
        sx, sy = self.free[a]
        self.goal = tuple(self.free[b])
        self.d.mocap_pos[self.goal_mocap] = [self.goal[0], self.goal[1], 0.01]
        self._place(sx, sy, self.rng.uniform(-math.pi, math.pi))
        mujoco.mj_forward(self.m, self.d)
        self.t = 0
        obs, self.prev = self._obs()
        return obs, {}

    def step(self, action):
        a = np.clip(np.asarray(action, float), -1, 1) * WHEEL_MAX
        wl, wr = a[0], a[1]
        v = WHEEL_R * (wl + wr) / 2
        w = WHEEL_R * (wr - wl) / WHEEL_BASE
        touched = False
        for _ in range(self.nsub):
            th = self.d.qpos[self.qt]
            self.d.ctrl[0] = v * math.cos(th)
            self.d.ctrl[1] = v * math.sin(th)
            self.d.ctrl[2] = w
            mujoco.mj_step(self.m, self.d)
            touched = touched or self._touching(0.0)
        self.t += 1
        obs, dist = self._obs()
        r = 10.0 * (self.prev - dist) - 0.01 - (0.05 if touched else 0.0)
        self.prev = dist
        success = dist < GOAL_TOL
        if success:
            r += 5.0
        trunc = self.t >= self.max_steps
        return obs, float(r), bool(success), bool(trunc and not success), {"is_success": success, "touched": touched}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", required=True)
    ap.add_argument("--bench", type=int, default=5000, help="無作為な行動でこの手数だけ進め、1 秒あたりの手数を測る")
    a = ap.parse_args()
    t0 = time.time()
    env = RoomNavEnv(a.xml, seed=0)
    t1 = time.time()
    print("環境: %s、geom %d 個、置ける点 %s、準備 %.2f 秒" % (os.path.basename(a.xml), env.m.ngeom, env.free_stats, t1 - t0))
    env.reset(seed=0)
    rng = np.random.default_rng(0)
    n_ep, n_succ = 0, 0
    t0 = time.time()
    for k in range(a.bench):
        _, _, term, trunc, info = env.step(rng.uniform(-1, 1, 2))
        if term or trunc:
            n_ep += 1
            n_succ += int(info["is_success"])
            env.reset()
    dt = time.time() - t0
    print("無作為な行動: %d 手 / %.2f 秒 = %.0f 手/秒(1 手 = %.1f 秒の制御 = 物理 %d 段)、物理の段 %.0f 段/秒、エピソード %d(到達 %d)"
          % (a.bench, dt, a.bench / dt, CTRL_DT, env.nsub, a.bench * env.nsub / dt, n_ep, n_succ))


if __name__ == "__main__":
    main()

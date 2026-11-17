#!/usr/bin/env python3
"""【未検証 UNTESTED】Dyne の部屋の USD で、Jetbot(Isaac の標準の車輪 2 つの台車)が目標地点へ進む RL の課題の
骨組み(Isaac Lab の DirectRLEnv)。CPU 版 scripts/rl_room_env.py と同じ観測・行動・報酬にそろえてある。2026-11-13。

2026-11-18: 発表では車輪の台車の代わりに 4 脚ロボット Go2 を使う(ユーザー決定)。Go2 版は scripts/isaaclab_go2_room.py
(手順は docs/2026-11-18-isaac-go2-procedure.md)。この台本は、自分で学習する課題の骨組みとして残す。

この PC には GPU が無いので一度も実行していない。借りた GPU の計算機で、まず
scripts/isaaclab_room.py(取り込みの確認)を通してから使う。直すことになりそうな所に「要確認」を付けた。
想定する版: Isaac Lab 2.x(isaaclab.* の名前空間)。

実行(Isaac Lab の入口から。学習は Isaac Lab 付属の rsl_rl / skrl の台本に gym の名前で渡すのが普通だが、
ここでは骨組みが動くかを見るために、無作為な行動で数百手まわすだけ):
  Linux:   ./isaaclab.sh -p scripts/isaaclab_nav_task.py --usd demo_out/pc_pipeline/room6/room.usda --num_envs 64 --headless
  Windows: isaaclab.bat -p scripts\\isaaclab_nav_task.py --usd demo_out\\pc_pipeline\\room6\\room.usda --num_envs 64 --headless

課題(CPU 版と同じ):
  行動  左右の車輪の角速度 [-1, 1]² × 10 rad/s(要確認: Jetbot の関節の名前・最大の速さ)
  観測  目標の位置(台車から見た前・左、/2 m)、距離、前進の速さ、回る速さ、16 方向の距離(CPU 版は光線。
        ここでは RayCaster を使う想定だが、骨組みでは 0 で埋めてある = 要実装)
  報酬  近づいた距離 × 10 − 0.01、到達(0.15 m 以内)で +5 して終える、20 秒で打ち切り
  出発点と目標: 部屋の USD の customData(dyne の gridOrigin・gridNx・gridNy・gridH)から格子の範囲を読み、
        CPU 版で作った「置ける点」の表(--free-points、rl_room_env.py で書き出す想定。無ければ格子の中央付近)から選ぶ
"""
import argparse
import math
import os

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True, help="twin_demo.py が書いた room.usda(箱の基本形の衝突判定)")
ap.add_argument("--num_envs", type=int, default=16)
ap.add_argument("--steps", type=int, default=600, help="無作為な行動で進める手数")
ap.add_argument("--spacing", type=float, default=6.0, help="複製した部屋どうしの間隔 [m](部屋より大きく)")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg  # noqa: E402
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR  # noqa: E402

WHEEL_R, WHEEL_BASE, WHEEL_MAX = 0.0325, 0.1125, 10.0   # 要確認(Jetbot)
N_RAYS = 16


def room_extent(usd_path):
    """USD の customLayerData の dyne の記録から格子の範囲 (x0, y0, x1, y1) を読む"""
    from pxr import Usd
    meta = (Usd.Stage.Open(usd_path).GetRootLayer().customLayerData or {}).get("dyne", {})
    ox, oy = meta.get("gridOrigin", (0.0, 0.0))
    h = float(meta.get("gridH", 0.05))
    return ox, oy, ox + (int(meta.get("gridNx", 2)) - 1) * h, oy + (int(meta.get("gridNy", 2)) - 1) * h


@configclass
class RoomSceneCfg(InteractiveSceneCfg):
    # 部屋: defaultPrim の /Room を参照で取り込み、環境の数だけ複製(/PhysicsScene は取り込まれない)
    room = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Room",
                        spawn=sim_utils.UsdFileCfg(usd_path=os.path.abspath(args.usd)))
    # Jetbot: Isaac の標準の資産(パスは版で変わる。要確認)
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Jetbot/jetbot.usd"),
        init_state=ArticulationCfg.InitialStateCfg(pos=(1.5, 1.5, 0.05)),
        actuators={},  # 要確認: 車輪の関節に速度制御の ImplicitActuatorCfg を付ける
    )
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))


@configclass
class RoomNavEnvCfg(DirectRLEnvCfg):
    decimation = 6                     # 物理 1/60 秒 × 6 = 0.1 秒ごとに行動(CPU 版と同じ)
    episode_length_s = 20.0
    action_space = 2
    observation_space = 5 + N_RAYS
    state_space = 0
    sim: SimulationCfg = SimulationCfg(dt=1.0 / 60.0)
    scene: RoomSceneCfg = RoomSceneCfg(num_envs=16, env_spacing=6.0)


class RoomNavEnv(DirectRLEnv):
    cfg: RoomNavEnvCfg

    def __init__(self, cfg, render_mode=None, **kw):
        super().__init__(cfg, render_mode, **kw)
        self.robot = self.scene["robot"]
        self.wheels, _ = self.robot.find_joints(["left_wheel_joint", "right_wheel_joint"])  # 要確認: 関節の名前
        x0, y0, x1, y1 = room_extent(args.usd)
        self.extent = torch.tensor([x0, y0, x1, y1], device=self.device)
        self.goal = torch.zeros(self.num_envs, 2, device=self.device)
        self.prev = torch.zeros(self.num_envs, device=self.device)
        self.act = torch.zeros(self.num_envs, 2, device=self.device)

    def _pose(self):
        p = self.robot.data.root_pos_w[:, :2] - self.scene.env_origins[:, :2]
        q = self.robot.data.root_quat_w  # (w, x, y, z)
        yaw = torch.atan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))
        return p, yaw

    def _pre_physics_step(self, actions):
        self.act = actions.clamp(-1, 1) * WHEEL_MAX

    def _apply_action(self):
        self.robot.set_joint_velocity_target(self.act, joint_ids=self.wheels)

    def _get_observations(self):
        p, yaw = self._pose()
        d = self.goal - p
        c, s = torch.cos(yaw), torch.sin(yaw)
        fwd, left = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
        dist = d.norm(dim=1)
        v = self.robot.data.root_lin_vel_b[:, 0]
        w = self.robot.data.root_ang_vel_b[:, 2]
        rays = torch.zeros(self.num_envs, N_RAYS, device=self.device)  # 要実装: RayCaster(部屋の衝突判定に当てる)
        obs = torch.cat([(fwd / 2)[:, None], (left / 2)[:, None], (dist.clamp(max=4) / 2)[:, None],
                         (v / (WHEEL_R * WHEEL_MAX))[:, None], (w / (2 * WHEEL_R * WHEEL_MAX / WHEEL_BASE))[:, None], rays], 1)
        return {"policy": obs}

    def _get_rewards(self):
        p, _ = self._pose()
        dist = (self.goal - p).norm(dim=1)
        r = 10.0 * (self.prev - dist) - 0.01 + 5.0 * (dist < 0.15).float()
        self.prev = dist
        return r

    def _get_dones(self):
        p, _ = self._pose()
        reached = (self.goal - p).norm(dim=1) < 0.15
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        return reached, timeout

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        n = len(env_ids)
        x0, y0, x1, y1 = self.extent.tolist()
        # 要改善: CPU 版の「置ける点」の表から選ぶ。骨組みでは格子の中央 60% から一様に選ぶ(家具の中に入りうる)
        cx, cy, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, 0.3 * (x1 - x0), 0.3 * (y1 - y0)
        start = torch.stack([cx + hx * (2 * torch.rand(n, device=self.device) - 1),
                             cy + hy * (2 * torch.rand(n, device=self.device) - 1)], 1)
        self.goal[env_ids] = torch.stack([cx + hx * (2 * torch.rand(n, device=self.device) - 1),
                                          cy + hy * (2 * torch.rand(n, device=self.device) - 1)], 1)
        state = self.robot.data.default_root_state[env_ids].clone()
        state[:, :2] = start + self.scene.env_origins[env_ids, :2]
        yaw = (2 * torch.rand(n, device=self.device) - 1) * math.pi
        state[:, 3] = torch.cos(yaw / 2)
        state[:, 6] = torch.sin(yaw / 2)
        state[:, 4:6] = 0
        self.robot.write_root_state_to_sim(state, env_ids)
        self.prev[env_ids] = (self.goal[env_ids] - start).norm(dim=1)


cfg = RoomNavEnvCfg()
cfg.scene.num_envs = args.num_envs
cfg.scene.env_spacing = args.spacing
env = RoomNavEnv(cfg)
env.reset()
import time  # noqa: E402
t0 = time.time()
ret = torch.zeros(env.num_envs, device=env.device)
for i in range(args.steps):
    a = 2 * torch.rand(env.num_envs, 2, device=env.device) - 1
    obs, r, term, trunc, _ = env.step(a)
    ret += r
dt = time.time() - t0
print("[dyne] %d 環境 × %d 手 = %.0f 手/秒(GPU、無作為な行動)。平均の累積報酬 %.2f"
      % (env.num_envs, args.steps, env.num_envs * args.steps / dt, ret.mean().item()))
env.close()
app.close()

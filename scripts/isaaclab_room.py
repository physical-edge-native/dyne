#!/usr/bin/env python3
"""【未検証 UNTESTED】Dyne の部屋の USD を Isaac Lab の場面に取り込み、剛体の円柱(ロボットの代わり)を
壁へ押して、衝突判定で止まることを見る。RL の環境に組み込む前の最小の取り込み確認。2026-11-11。

この PC には GPU が無いので一度も実行していない(docs/2026-11-11-usd-physics.md の「Isaac での確認手順」)。
想定する版: Isaac Lab 2.x(isaaclab.* の名前空間。1.x は omni.isaac.lab.* で、この台本は動かない)。
実行(Isaac Lab の入口から):
  Linux:   ./isaaclab.sh -p scripts/isaaclab_room.py --usd demo_out/twin/room.usda [--headless]
  Windows: isaaclab.bat -p scripts\\isaaclab_room.py --usd demo_out\\twin\\room.usda [--headless]

RL の環境に入れるときは、同じ UsdFileCfg を InteractiveSceneCfg の AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Room",
spawn=sim_utils.UsdFileCfg(usd_path=...)) にすれば、環境の数だけ複製される(地面は部屋の床があるので不要)。
"""
import argparse
import os

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True, help="twin_demo.py が書いた room.usda")
ap.add_argument("--steps", type=int, default=600, help="物理のステップ数(1/120 秒ずつ)")
ap.add_argument("--start", type=float, nargs=2, default=[1.55, 1.50], help="円柱の出発点 x y [m]")
ap.add_argument("--force", type=float, default=100.0, help="+x 方向に押す力 [N](質量 10 kg、床の摩擦に勝つ大きさ)")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import RigidObject, RigidObjectCfg  # noqa: E402

sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1.0 / 120.0, device=args.device))
sim.set_camera_view([5.0, -2.5, 4.0], [1.8, 1.35, 0.0])

# 部屋(defaultPrim の /Room を参照で取り込む。/PhysicsScene は取り込まれない)
room_cfg = sim_utils.UsdFileCfg(usd_path=os.path.abspath(args.usd))
room_cfg.func("/World/Room", room_cfg)
light_cfg = sim_utils.DomeLightCfg(intensity=2000.0)
light_cfg.func("/World/Light", light_cfg)

x0, y0 = args.start
robot = RigidObject(RigidObjectCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.CylinderCfg(
        radius=0.15, height=0.2, axis="Z",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=10.0),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.8, 0.3))),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(x0, y0, 0.11))))

sim.reset()
forces = torch.zeros((1, 1, 3), device=sim.device)
forces[..., 0] = args.force
torques = torch.zeros_like(forces)
dt = sim.get_physics_dt()
for i in range(args.steps):
    robot.set_external_force_and_torque(forces, torques)
    robot.write_data_to_sim()
    sim.step()
    robot.update(dt)
    if i % 60 == 0 or i == args.steps - 1:
        p = robot.data.root_pos_w[0].tolist()
        print("[dyne] t=%5.2f s cylinder=(%.3f, %.3f, %.3f)" % (i * dt, p[0], p[1], p[2]))

p = robot.data.root_pos_w[0].tolist()
print("[dyne] 結果: 円柱の最後の x = %.3f m(room6・既定の出発点なら、ベッドの面 x = 2.6 m から半径 0.15 m 手前の"
      " 約 2.45 m で止まるはず。MuJoCo では 2.451 m)、z = %.3f m(床の上なら約 0.10 m)" % (p[0], p[2]))
app.close()

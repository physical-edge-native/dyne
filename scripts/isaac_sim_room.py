#!/usr/bin/env python3
"""【未検証 UNTESTED】Dyne の部屋の USD を Isaac Sim に静的な場面として読み込み、市販の車輪ロボット
(Jetbot。Isaac Sim の標準の資産)を走らせて、壁・家具に当たって止まることを見る。2026-11-11。

この PC には GPU が無いので一度も実行していない。借りた GPU の計算機で最初に動かす前に、
docs/2026-11-11-usd-physics.md の「Isaac での確認手順」を読むこと。

想定する版: Isaac Sim 4.5 / 5.x(isaacsim.* の名前空間)。4.0〜4.2 は omni.isaac.* に自動で切り替える。
実行(Isaac Sim の同梱 Python で):
  Linux:   ./python.sh  scripts/isaac_sim_room.py --usd demo_out/twin/room.usda
  Windows: python.bat   scripts\\isaac_sim_room.py --usd demo_out\\twin\\room.usda [--headless]

見るもの(出力の行):
  - 球(半径 0.1 m)を高さ 1 m から落とし、止まった中心の高さ ≈ 0.1 m(床の衝突判定が効いている)
  - Jetbot を +x 方向へ一定の車輪速度で走らせ、x 座標が途中で止まる(部屋の場面 room6 で出発点 (1.55, 1.50)
    なら、x = 2.6 m のベッドの手前で止まるはず)。通り抜けたら衝突判定が効いていない
"""
import argparse
import os

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True, help="twin_demo.py が書いた room.usda")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--steps", type=int, default=1200, help="物理のステップ数(1/60 秒ずつ)")
ap.add_argument("--start", type=float, nargs=2, default=[1.55, 1.50], help="ロボットと球の出発点 x y [m]")
ap.add_argument("--wheel-speed", type=float, default=8.0, help="車輪の角速度 [rad/s]")
args = ap.parse_args()

try:
    from isaacsim import SimulationApp  # Isaac Sim 4.5 以降
except ImportError:
    from omni.isaac.kit import SimulationApp  # Isaac Sim 4.0〜4.2

app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402

try:  # Isaac Sim 4.5 / 5.x
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicSphere
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot
    from isaacsim.storage.native import get_assets_root_path
except ImportError:  # Isaac Sim 4.0〜4.2
    from omni.isaac.core import World
    from omni.isaac.core.objects import DynamicSphere
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.utils.types import ArticulationAction
    from omni.isaac.wheeled_robots.robots import WheeledRobot
import omni.client  # noqa: E402

usd = os.path.abspath(args.usd)
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
# 部屋: defaultPrim (/Room) を参照で取り込む。room.usda の /PhysicsScene は defaultPrim の外なので
# 取り込まれず、World が作る物理の場(重力 -Z 9.81)だけになる
add_reference_to_stage(usd_path=usd, prim_path="/World/Room")

# 車輪ロボット(Jetbot)。資産の場所は版で違う
root = get_assets_root_path()
if root is None:
    raise SystemExit("Isaac の資産の場所(Nucleus / S3)が見つからない。ネットワークと資産の設定を確認すること")
robot_usd = None
for rel in ("/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd", "/Isaac/Robots/Jetbot/jetbot.usd"):
    if omni.client.stat(root + rel)[0] == omni.client.Result.OK:
        robot_usd = root + rel
        break
if robot_usd is None:
    raise SystemExit("jetbot.usd が見つからない(%s の下)。資産の一覧で車輪ロボットの場所を確かめること" % root)
print("[dyne] room =", usd)
print("[dyne] robot =", robot_usd)

x0, y0 = args.start
robot = world.scene.add(WheeledRobot(prim_path="/World/Jetbot", name="jetbot",
                                     wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                                     create_robot=True, usd_path=robot_usd,
                                     position=np.array([x0, y0, 0.05])))
# 球はロボットと重ならないよう y を 0.6 m ずらす
ball = world.scene.add(DynamicSphere(prim_path="/World/Ball", name="ball", radius=0.1,
                                     position=np.array([x0, y0 + 0.6, 1.0]), color=np.array([0.2, 0.5, 1.0])))
world.reset()

last_x, stalled = None, 0
for i in range(args.steps):
    robot.apply_wheel_actions(ArticulationAction(joint_velocities=np.array([args.wheel_speed, args.wheel_speed])))
    world.step(render=not args.headless)
    if i % 60 == 0 or i == args.steps - 1:
        p, _ = robot.get_world_pose()
        b, _ = ball.get_world_pose()
        print("[dyne] t=%5.2f s robot=(%.3f, %.3f, %.3f) ball_z=%.3f" % (i / 60.0, p[0], p[1], p[2], b[2]))
        if last_x is not None and abs(p[0] - last_x) < 0.002:
            stalled += 1
        last_x = p[0]

p, _ = robot.get_world_pose()
b, _ = ball.get_world_pose()
print("[dyne] 結果: 球の中心の高さ %.3f m(期待 0.10 m)、ロボットの最後の x = %.3f m、止まっていた区間 %d 回"
      % (b[2], p[0], stalled))
print("[dyne] 判定の目安: 球 0.09〜0.11 m、ロボットは x < 2.6 m で止まる(room6、出発点の既定値のとき)")
app.close()

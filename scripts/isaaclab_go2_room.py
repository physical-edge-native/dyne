#!/usr/bin/env python3
"""Dyne の部屋の USD に Unitree Go2(4 脚ロボット)を置き、学習済みの歩行の方策(速度の指令に従って歩く NN)を
動かして、部屋の中の目標の点を順にめぐらせる。2026-11-18、Issue #93。

2 つの使い方がある:
  (1) 事前の確認(Isaac なし。リポジトリの .venv の pxr と numpy だけで動く。この PC で確認済み):
        .venv\\Scripts\\python.exe scripts\\isaaclab_go2_room.py --usd demo_out\\twin\\room.usda --check-only
      USD の単位・defaultPrim・衝突判定の箱を読み、Go2 が入れる場所(壁・家具・未観測の箱から --clearance 以上
      離れた点)を求めて、出発点と目標の点の列を表示する。置ける場所が無ければ終了コード 1。
  (2) 【未検証 UNTESTED】GPU の計算機の Isaac Lab で実行(この PC には GPU が無いので一度も実行していない):
        ./isaaclab.sh -p /path/to/dyne/scripts/isaaclab_go2_room.py --usd /path/to/room.usda \\
            --policy logs/rsl_rl/unitree_go2_flat/<日時>/exported/policy.pt [--video] [--headless]
      手順は docs/2026-11-18-isaac-go2-procedure.md。

構成(いちばん小さく動く形として選んだもの。ほかの案は手順書の「案の比較」):
  - 下の層: Isaac Lab 付属の課題 Isaac-Velocity-Flat-Unitree-Go2-v0(平らな床で速度の指令に従って歩く)を
    学習した方策。Isaac Lab の play.py が書き出す TorchScript の policy.pt を読む。
  - 上の層: 学習ではなく、単純な規則(目標の方向へ向きを変え、距離に応じて前へ進む指令を出す)。
    障害物を避ける判断はしない。代わりに、出発点から目標の点までの直線が障害物から --clearance 以上
    離れているものだけを目標に選ぶ(この台本の check の部分)。
  - 部屋: USD の defaultPrim(/Room)を各環境に参照で取り込み、静的な衝突判定として使う。Isaac Lab の平らな
    床(z = 0)はそのまま残す(部屋の床の上面も z = 0 で、重なっても静的どうしなので問題にならない想定。要確認)。
    出発点が各環境の原点に来るよう、部屋を (-出発点x, -出発点y, 0) だけずらして置く。

想定する版: Isaac Lab 2.3.x(Isaac Sim 5.1)。2.1・2.2 も同じ書き方のはず(要確認)。Isaac Lab 3.0 は課題の
名前・起動の方法・データの型が変わっているので、そのままでは動かない可能性が高い(手順書の「版の選び方」)。
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GO2_HALF_DIAG = 0.39   # Go2 の胴と脚の水平の外形 約 0.70 m × 0.31 m(Unitree の公表値)の対角の半分 [m]
Z_BAND = (0.05, 0.25, 0.45)  # 障害物とみなす高さ [m](Go2 の脚・胴の高さの範囲で調べる)


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--usd", required=True, help="twin_demo.py / twin_live.py が書いた room.usda")
    ap.add_argument("--check-only", action="store_true", help="Isaac を起動せず、USD と置ける場所の確認だけをする")
    ap.add_argument("--policy", help="Isaac Lab の play.py が書き出した exported/policy.pt(TorchScript)")
    ap.add_argument("--clearance", type=float, default=0.45,
                    help="Go2 の中心から障害物までに空ける距離 [m](既定 0.45。体の対角の半分 約 0.39 m + 余裕)")
    ap.add_argument("--res", type=float, default=0.05, help="置ける場所を調べる格子の刻み [m]")
    ap.add_argument("--spawn", type=float, nargs=2, metavar=("X", "Y"), help="出発点(部屋の座標 [m])。既定は自動")
    ap.add_argument("--goals", type=float, nargs="+", metavar="XY",
                    help="目標の点 x1 y1 x2 y2 ...(部屋の座標 [m])。既定は自動で --num-goals 個")
    ap.add_argument("--num-goals", type=int, default=4)
    ap.add_argument("--min-goal-dist", type=float, default=0.8, help="自動で選ぶ目標の点どうしの最小の間隔 [m]")
    ap.add_argument("--goal-margin", type=float, default=0.10,
                    help="自動で選ぶ目標の点には --clearance よりこれだけ広い余裕を求める [m](止まるときの行き過ぎの分)")
    ap.add_argument("--yaw", type=float, default=0.0, help="出発のときの向き [度]")
    ap.add_argument("--seconds", type=float, default=60.0, help="動かす時間 [s](シミュレーションの時刻)")
    ap.add_argument("--max-speed", type=float, default=0.6, help="前へ進む速さの指令の上限 [m/s]")
    ap.add_argument("--goal-tol", type=float, default=0.25, help="目標に着いたとみなす距離 [m]")
    ap.add_argument("--num_envs", type=int, default=1, help="並べる部屋の数(見せるだけなら 1)")
    ap.add_argument("--spacing", type=float, default=8.0, help="複製した部屋どうしの間隔 [m](部屋より大きく)")
    ap.add_argument("--video", action="store_true", help="動画を書く(--enable_cameras が自動で付く)")
    ap.add_argument("--video_length", type=int, default=600, help="動画の長さ [手](1 手 = 0.02 s)")
    ap.add_argument("--out", default="go2_room_out", help="記録(軌跡の CSV・動画)を書くフォルダ")
    return ap


# ---------------------------------------------------------------- Isaac なしで動く部分(pxr と numpy)


def read_room(usd_path):
    """USD を開き、単位の確認・格子の範囲・衝突判定の箱(Cube)の逆変換を返す。問題は errors に入れる。"""
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics

    errors = []
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        return None, None, None, ["USD を開けない: %s" % usd_path]
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        errors.append("upAxis が Z でない")
    if abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-9:
        errors.append("metersPerUnit が 1 でない")
    dp = stage.GetDefaultPrim()
    if not dp or dp.GetPath() != "/Room":
        errors.append("defaultPrim が /Room でない(参照で取り込めない)")
    meta = (stage.GetRootLayer().customLayerData or {}).get("dyne", {})
    ox, oy = meta.get("gridOrigin", (0.0, 0.0))
    h = float(meta.get("gridH", 0.05))
    nx, ny = int(meta.get("gridNx", 0)), int(meta.get("gridNy", 0))
    if nx < 2 or ny < 2:
        errors.append("customLayerData の dyne.gridNx / gridNy が無い(twin_demo.py が書いた USD か)")
        extent = None
    else:
        extent = (float(ox), float(oy), float(ox) + (nx - 1) * h, float(oy) + (ny - 1) * h)
    col = stage.GetPrimAtPath("/Room/Collision")
    if not col:
        errors.append("/Room/Collision が無い")
        return extent, [], meta, errors
    cache = UsdGeom.XformCache()
    boxes = []
    n_mesh = 0
    for p in Usd.PrimRange(col):
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if p.IsA(UsdGeom.Cube):
            if p.GetName() == "Floor":
                continue
            M = np.array(cache.GetLocalToWorldTransform(p), float)  # 行ベクトル規約(p' = p M)
            size = float(UsdGeom.Cube(p).GetSizeAttr().Get())
            boxes.append((np.linalg.inv(M), size / 2.0, str(p.GetPath())))
        elif p.IsA(UsdGeom.Mesh):
            n_mesh += 1
    if n_mesh:
        errors.append("衝突判定のメッシュが %d 個ある。この台本は箱(Cube)だけを障害物として数える"
                      "(twin_demo.py の既定 --collider-shape box で書き出すこと)" % n_mesh)
    if not boxes:
        errors.append("床のほかに衝突判定の箱が無い")
    return extent, boxes, meta, errors


def occupancy(boxes, extent, res):
    """格子の各点が、どれかの箱の中(高さ Z_BAND のどれか)にあるかを返す。部屋の範囲の外は障害物とみなす。"""
    import numpy as np
    x0, y0, x1, y1 = extent
    xs = np.arange(x0 + res / 2, x1, res)
    ys = np.arange(y0 + res / 2, y1, res)
    X, Y = np.meshgrid(xs, ys)
    occ = np.zeros(X.shape, bool)
    for z in Z_BAND:
        P = np.c_[X.ravel(), Y.ravel(), np.full(X.size, z), np.ones(X.size)]
        for Minv, half, _ in boxes:
            L = P @ Minv
            occ |= (np.abs(L[:, :3]) <= half + 1e-9).all(axis=1).reshape(X.shape)
    return xs, ys, occ


def clearance_map(xs, ys, occ):
    """各点から、いちばん近い障害物の点(または部屋の範囲の外)までの距離 [m]。"""
    import numpy as np
    X, Y = np.meshgrid(xs, ys)
    res = xs[1] - xs[0] if len(xs) > 1 else 0.05
    # 部屋の範囲の外も障害物(範囲の端から半刻み外)
    border = np.minimum.reduce([X - (xs[0] - res / 2), (xs[-1] + res / 2) - X, Y - (ys[0] - res / 2), (ys[-1] + res / 2) - Y])
    O = np.c_[X[occ], Y[occ]]
    F = np.c_[X.ravel(), Y.ravel()]
    d = np.full(F.shape[0], np.inf)
    for i in range(0, len(O), 2000):
        blk = O[i:i + 2000]
        dd = np.sqrt(((F[:, None, :] - blk[None, :, :]) ** 2).sum(-1)).min(1)
        d = np.minimum(d, dd)
    # 格子の点どうしの距離なので、刻みの半分だけ短く見積もる(安全側)
    return np.minimum(d.reshape(X.shape) - res / 2, border)


def segment_clear(xs, ys, clr, a, b, need):
    """a から b への直線上のすべての点で、障害物までの距離が need 以上か(格子の最近点で調べる)"""
    import numpy as np
    res = xs[1] - xs[0]
    n = max(2, int(math.ceil(math.dist(a, b) / (res / 2))) + 1)
    for t in np.linspace(0.0, 1.0, n):
        x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        i, j = int(round((x - xs[0]) / res)), int(round((y - ys[0]) / res))
        if not (0 <= i < len(xs) and 0 <= j < len(ys)) or clr[j, i] < need:
            return False
    return True


def plan(xs, ys, clr, need, spawn=None, goals=None, num_goals=4, min_goal_dist=0.8, goal_margin=0.1):
    """出発点(既定は最も広い点)と、直線で行ける目標の点の列を選ぶ。決定論(乱数なし)。"""
    import numpy as np
    X, Y = np.meshgrid(xs, ys)
    ok = clr >= need + goal_margin
    if spawn is None:
        j, i = np.unravel_index(np.argmax(clr), clr.shape)
        spawn = (float(X[j, i]), float(Y[j, i]))
    route = []
    if goals is not None:
        route = [tuple(g) for g in goals]
    else:
        cand = [(float(x), float(y)) for x, y in zip(X[ok], Y[ok])]
        cur = spawn
        visited = [spawn]
        for _ in range(num_goals):
            best, best_d = None, 0.0
            for c in cand:
                if min(math.dist(c, v) for v in visited) < min_goal_dist:
                    continue
                d = math.dist(c, cur)
                if d > best_d and segment_clear(xs, ys, clr, cur, c, need):
                    best, best_d = c, d
            if best is None:
                break
            route.append(best)
            visited.append(best)
            cur = best
    return spawn, route


def check(args):
    """(1) の事前の確認。成功なら (spawn, goals, extent) を返し、失敗なら None。"""
    import numpy as np
    if not os.path.isfile(args.usd):
        print("[dyne] FAIL: USD が無い: %s" % args.usd)
        return None
    extent, boxes, meta, errors = read_room(args.usd)
    for e in errors:
        print("[dyne] 問題: " + e)
    if extent is None or not boxes:
        print("[dyne] FAIL: 部屋の範囲か衝突判定の箱が読めない")
        return None
    print("[dyne] 部屋の USD: %s(作成 %s、場面 %s、通し番号 %s)"
          % (args.usd, meta.get("createdUtc", "?"), meta.get("scene", "?"), meta.get("sequence", "?")))
    print("[dyne] 格子の範囲 x %.3f〜%.3f m、y %.3f〜%.3f m、衝突判定の箱 %d 個(床を除く)"
          % (extent[0], extent[2], extent[1], extent[3], len(boxes)))
    xs, ys, occ = occupancy(boxes, extent, args.res)
    clr = clearance_map(xs, ys, occ)
    need = args.clearance
    ok = clr >= need
    print("[dyne] 障害物の点 %.1f%%、Go2 の中心を置ける点(障害物から %.2f m 以上)%d 点 = %.2f m²、最も広い点の余裕 %.2f m"
          % (100.0 * occ.mean(), need, int(ok.sum()), ok.sum() * args.res ** 2, float(clr.max())))
    if args.clearance < GO2_HALF_DIAG:
        print("[dyne] 注意: --clearance %.2f m は Go2 の対角の半分 %.2f m より小さい(その場で回ると当たりうる)"
              % (args.clearance, GO2_HALF_DIAG))
    goals = None
    if args.goals:
        if len(args.goals) % 2:
            print("[dyne] FAIL: --goals は x y の組で渡す")
            return None
        goals = list(zip(args.goals[0::2], args.goals[1::2]))
    spawn, route = plan(xs, ys, clr, need, args.spawn, goals, args.num_goals, args.min_goal_dist,
                        args.goal_margin)

    def clr_at(p):
        i = int(np.clip(round((p[0] - xs[0]) / args.res), 0, len(xs) - 1))
        j = int(np.clip(round((p[1] - ys[0]) / args.res), 0, len(ys) - 1))
        return float(clr[j, i])

    bad = clr_at(spawn) < need
    print("[dyne] 出発点 (%.2f, %.2f) 余裕 %.2f m%s" % (spawn[0], spawn[1], clr_at(spawn), "  ← 狭い" if bad else ""))
    prev = spawn
    for k, g in enumerate(route):
        straight = segment_clear(xs, ys, clr, prev, g, need)
        print("[dyne] 目標 %d (%.2f, %.2f) 余裕 %.2f m、前の点からの直線 %.2f m %s"
              % (k + 1, g[0], g[1], clr_at(g), math.dist(prev, g), "通れる" if straight else "← 障害物に近い"))
        bad |= not straight or clr_at(g) < need
        prev = g
    if not route:
        print("[dyne] FAIL: 直線で行ける目標の点が無い(--clearance を小さくするか、--goals で渡す)")
        return None
    if bad:
        print("[dyne] FAIL: 出発点か目標の点が狭い(上の ←)")
        return None
    print("[dyne] PASS: Go2 を置いて目標の点 %d 個をめぐれる形(障害物を避ける判断はしないので、直線で通れる点だけを選んだ)"
          % len(route))
    return spawn, route, extent


# ---------------------------------------------------------------- Isaac Lab の部分(GPU の計算機で実行)


def _t(x):
    """Isaac Lab 3.0 のデータ(ProxyArray)にも 2.x の torch のテンソルにも使える"""
    return x.torch if hasattr(x, "torch") else x


def run_isaac(args, spawn, route, extent):
    # 呼び出し元(main)で AppLauncher を起動した後に読む
    import gymnasium as gym
    import torch

    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.envs import ManagerBasedRLEnv

    try:  # Isaac Lab 2.x
        from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.flat_env_cfg import (
            UnitreeGo2FlatEnvCfg_PLAY as Go2Cfg)
    except ImportError:  # Isaac Lab 3.0(要確認。PLAY 版が無ければ学習用の設定から同じ変更をする)
        from isaaclab_tasks.core.velocity.config.go2.flat_env_cfg import UnitreeGo2FlatEnvCfg as Go2Cfg

    cfg = Go2Cfg()
    cfg.scene.num_envs = args.num_envs
    cfg.scene.env_spacing = args.spacing
    if getattr(args, "device", None):
        cfg.sim.device = args.device
    sx, sy = spawn
    # 部屋: 出発点が環境の原点に来るようにずらして取り込む(/PhysicsScene は defaultPrim の外なので取り込まれない)
    cfg.scene.room = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Room",
        spawn=sim_utils.UsdFileCfg(usd_path=os.path.abspath(args.usd)),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(-sx, -sy, 0.0)))
    # 学習のときの乱れを外す(PLAY 版と同じ。3.0 の学習用の設定のときのため)
    cfg.observations.policy.enable_corruption = False
    for name in ("base_external_force_torque", "push_robot"):
        if hasattr(cfg.events, name):
            setattr(cfg.events, name, None)
    yaw = math.radians(args.yaw)
    cfg.events.reset_base.params["pose_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (yaw, yaw)}
    # 速度の指令はこの台本が毎手書く(自動の抽選・向きの制御・止まる環境を外す)
    c = cfg.commands.base_velocity
    c.heading_command = False
    c.rel_heading_envs = 0.0
    c.rel_standing_envs = 0.0
    c.resampling_time_range = (1.0e6, 1.0e6)
    c.debug_vis = True
    cfg.episode_length_s = args.seconds + 10.0
    # 視点: 0 番の環境の部屋を斜め上から
    cx, cy = (extent[0] + extent[2]) / 2 - sx, (extent[1] + extent[3]) / 2 - sy
    cfg.viewer.origin_type = "env"
    cfg.viewer.env_index = 0
    cfg.viewer.eye = (cx, cy - 4.0, 4.5)
    cfg.viewer.lookat = (cx, cy, 0.0)

    env = ManagerBasedRLEnv(cfg=cfg, render_mode="rgb_array" if args.video else None)
    os.makedirs(args.out, exist_ok=True)
    if args.video:
        env = gym.wrappers.RecordVideo(env, video_folder=os.path.join(args.out, "videos"),
                                       step_trigger=lambda s: s == 0, video_length=args.video_length,
                                       disable_logger=True)
    base = env.unwrapped
    dev = base.device
    policy = torch.jit.load(args.policy, map_location=dev)
    policy.eval()
    robot = base.scene["robot"]
    cmd = base.command_manager.get_term("base_velocity")
    origins = base.scene.env_origins[:, :2]
    goals = torch.tensor(route, device=dev, dtype=torch.float32)
    idx = torch.zeros(base.num_envs, dtype=torch.long, device=dev)
    reached = torch.zeros(base.num_envs, dtype=torch.long, device=dev)
    resets = 0
    dt = base.step_dt
    n_steps = int(args.seconds / dt)
    print("[dyne] 環境 %d 個、1 手 %.3f s、%d 手。出発点 (%.2f, %.2f)、目標 %d 個"
          % (base.num_envs, dt, n_steps, sx, sy, len(route)))
    log = open(os.path.join(args.out, "trajectory_env0.csv"), "w", encoding="utf-8")
    log.write("t_s,x_m,y_m,yaw_rad,goal_index,goal_x_m,goal_y_m,cmd_vx,cmd_wz,reset\n")
    obs, _ = env.reset()
    with torch.inference_mode():
        for k in range(n_steps):
            pos = _t(robot.data.root_pos_w)[:, :2] - origins + torch.tensor([sx, sy], device=dev)
            yaw_w = _t(robot.data.heading_w)
            g = goals[idx % len(route)]
            d = g - pos
            dist = d.norm(dim=1)
            err = torch.atan2(d[:, 1], d[:, 0]) - yaw_w
            err = torch.atan2(torch.sin(err), torch.cos(err))
            vx = (1.5 * dist).clamp(max=args.max_speed) * torch.cos(err).clamp(min=0.0) ** 2
            wz = (1.5 * err).clamp(-1.0, 1.0)
            hit = dist < args.goal_tol
            if hit.any():
                for e in hit.nonzero().flatten().tolist():
                    print("[dyne] t=%6.2f s 環境 %d が目標 %d に到着" % (k * dt, e, int(idx[e] % len(route)) + 1))
                idx[hit] += 1
                reached[hit] += 1
            vc = _t(cmd.vel_command_b)
            vc[:, 0] = vx
            vc[:, 1] = 0.0
            vc[:, 2] = wz
            act = policy(obs["policy"])
            obs, _, term, trunc, _ = env.step(act)
            done = term | trunc
            if done.any():
                resets += int(done.sum())
                print("[dyne] t=%6.2f s 環境 %s がやり直し(転倒か胴の接触。部屋の箱に胴が当たった可能性)"
                      % (k * dt, done.nonzero().flatten().tolist()))
            log.write("%.3f,%.4f,%.4f,%.4f,%d,%.3f,%.3f,%.3f,%.3f,%d\n"
                      % (k * dt, pos[0, 0], pos[0, 1], yaw_w[0], int(idx[0] % len(route)), g[0, 0], g[0, 1],
                         vx[0], wz[0], int(done[0])))
            if k % int(round(1.0 / dt)) == 0:
                print("[dyne] t=%6.2f s 環境 0: 位置 (%.2f, %.2f)、目標 %d まで %.2f m"
                      % (k * dt, pos[0, 0], pos[0, 1], int(idx[0] % len(route)) + 1, dist[0]))
    log.close()
    print("[dyne] 結果: 到着の合計 %s 回(環境ごと)、やり直し %d 回。軌跡 %s"
          % (reached.tolist(), resets, os.path.join(args.out, "trajectory_env0.csv")))
    env.close()


def main(argv=None):
    ap = build_parser()
    pre, _ = ap.parse_known_args(argv)
    if pre.check_only:
        args = ap.parse_args(argv)
        return 0 if check(args) else 1
    try:
        from isaaclab.app import AppLauncher
    except ImportError:
        print("[dyne] isaaclab が読めない。Isaac Lab の入口(./isaaclab.sh -p)から実行するか、--check-only を付ける")
        return 2
    AppLauncher.add_app_launcher_args(ap)
    args = ap.parse_args(argv)
    if not args.policy or not os.path.isfile(args.policy):
        ap.error("--policy に、play.py が書き出した exported/policy.pt を渡す(--check-only なら不要)")
    if not os.path.isfile(args.usd):
        ap.error("USD が無い: %s" % args.usd)
    if args.video:
        args.enable_cameras = True
    app = AppLauncher(args).app
    try:
        # Isaac の pxr は起動の後でないと読めないことがあるので、USD の確認は起動の後にする
        res = check(args)
        if not res:
            return 1
        run_isaac(args, *res)
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

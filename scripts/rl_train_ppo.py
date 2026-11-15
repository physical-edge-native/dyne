#!/usr/bin/env python3
"""scripts/rl_room_env.py の課題(移動台車が Dyne の部屋で目標へ進む)を、PPO(近接方策最適化。方策の更新幅を
制限する強化学習の手法)で CPU だけで短く学習し、学習曲線を書く。2026-11-13。

ライブラリ: stable-baselines3(PPO の実装)+ PyTorch の CPU 版 + gymnasium(すべてリポジトリの .venv)。
出力(--out): curve.csv(更新ごとの手数・平均の報酬・到達率・経過秒)、curve.png、ppo_room.zip(学習した方策)、
             eval.txt(学習後の到達率。学習に使った部屋と、--eval-xml で渡した別の部屋)

使い方:
  .venv\\Scripts\\python.exe scripts\\rl_train_ppo.py --xml demo_out\\pc_pipeline\\room6\\room.xml ^
      --steps 300000 --envs 8 --out demo_out\\pc_pipeline\\rl ^
      --eval-xml demo_out\\pc_pipeline\\esc\\move600_on.xml
"""
import argparse
import csv
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def make_env(xml, seed):
    def f():
        from rl_room_env import RoomNavEnv
        return RoomNavEnv(xml, seed=seed)
    return f


def evaluate(model, xml, episodes, seed=1000):
    from rl_room_env import RoomNavEnv
    env = RoomNavEnv(xml, seed=seed)
    succ, rets, touch = 0, [], 0
    for e in range(episodes):
        obs, _ = env.reset(seed=seed + e)
        done, ret = False, 0.0
        while not done:
            act, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(act)
            ret += r
            touch += int(info["touched"])
            done = term or trunc
        succ += int(info["is_success"])
        rets.append(ret)
    return succ / episodes, float(np.mean(rets)), touch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", required=True, help="学習に使う部屋(twin_demo.py が書いた room.xml)")
    ap.add_argument("--steps", type=int, default=300000)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=os.path.join("demo_out", "pc_pipeline", "rl"))
    ap.add_argument("--eval-episodes", type=int, default=100)
    ap.add_argument("--eval-xml", action="append", default=[], help="学習後に方策をそのまま試す別の部屋(形が変わった部屋など)")
    ap.add_argument("--subproc", action="store_true", help="環境を別のプロセスで動かす(SubprocVecEnv)")
    a = ap.parse_args()
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor

    torch.set_num_threads(1)
    os.makedirs(a.out, exist_ok=True)
    fns = [make_env(a.xml, a.seed + k) for k in range(a.envs)]
    venv = VecMonitor(SubprocVecEnv(fns) if a.subproc else DummyVecEnv(fns), info_keywords=("is_success",))
    rows = []
    t0 = time.time()

    class Curve(BaseCallback):
        def _on_rollout_end(self):
            buf = list(self.model.ep_info_buffer)
            if not buf:
                return
            ret = float(np.mean([e["r"] for e in buf]))
            succ = float(np.mean([e.get("is_success", 0.0) for e in buf]))
            el = time.time() - t0
            rows.append((self.num_timesteps, ret, succ, el))
            print("手数 %7d  平均の報酬 %7.2f  到達率 %5.1f%%  経過 %6.1f 秒(%.0f 手/秒)"
                  % (self.num_timesteps, ret, succ * 100, el, self.num_timesteps / el), flush=True)

        def _on_step(self):
            return True

    model = PPO("MlpPolicy", venv, n_steps=512, batch_size=256, n_epochs=10, learning_rate=3e-4, gamma=0.99,
                gae_lambda=0.95, ent_coef=0.0, policy_kwargs=dict(net_arch=[64, 64]), seed=a.seed, verbose=0, device="cpu")
    model.learn(total_timesteps=a.steps, callback=Curve())
    total = time.time() - t0
    model.save(os.path.join(a.out, "ppo_room"))
    with open(os.path.join(a.out, "curve.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timesteps", "mean_return_last100", "success_rate_last100", "elapsed_s"])
        w.writerows(rows)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(9, 3.4))
        ts = [r[0] for r in rows]
        ax[0].plot(ts, [r[1] for r in rows], color="#2a6fdb")
        ax[0].set_xlabel("environment steps")
        ax[0].set_ylabel("mean episode return (last 100)")
        ax[1].plot(ts, [r[2] * 100 for r in rows], color="#1a9a50")
        ax[1].set_xlabel("environment steps")
        ax[1].set_ylabel("goal reached [%] (last 100)")
        ax[1].set_ylim(0, 100)
        for x in ax:
            x.grid(alpha=0.3)
        fig.suptitle("PPO on Dyne room (%s), CPU, %d envs" % (os.path.basename(os.path.dirname(os.path.abspath(a.xml))), a.envs))
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, "curve.png"), dpi=120)
    except Exception as e:  # noqa: BLE001
        print("図は書けなかった: %s" % e)
    lines = ["学習: %d 手、%.1f 秒(%.0f 手/秒、環境 %d 個、%s)" % (a.steps, total, a.steps / total, a.envs,
                                                         "SubprocVecEnv" if a.subproc else "DummyVecEnv")]
    for xml in [a.xml] + a.eval_xml:
        s, r, tch = evaluate(model, xml, a.eval_episodes)
        lines.append("評価 %s: 到達率 %.1f%%(%d 回)、平均の報酬 %.2f、壁・家具に触れた手 %d"
                     % (xml, s * 100, a.eval_episodes, r, tch))
    for l in lines:
        print(l)
    with open(os.path.join(a.out, "eval.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()

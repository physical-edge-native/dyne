#!/usr/bin/env python3
"""1次元チェーンの設定を生成して MQTT 制御面へ注入する (発注7、2026-09-14)。

規約: docs/2026-09-14-mqtt-control-plane.md §2.1〜2.3。
格子の分割はシミュレータ (dyne-sim) と同一規則 (points を nodes で割り、余りを
先頭ノードから1点ずつ)。外縁は dirichlet(0) 固定、不在隣接への面は --bc のキー。

使い方 (例: 2ノード、PCブローカー、ノードのIPは status で確認したもの):
  python scripts/mqtt_config.py --broker 192.168.137.1 --nodes 2 --points 65 \
      --steps 2000 --peers 192.168.137.10,192.168.137.11 --start
部分観測 (実験1の構図。ノード0,3を不在にし、境界は transparent):
  python scripts/mqtt_config.py --broker ... --nodes 4 --points 65 \
      --absent 0,3 --bc transparent "--peers=-,192.168.137.10,192.168.137.11,-"

必要パッケージ: paho-mqtt (pip install paho-mqtt)
"""
import argparse
import json
import math
import time

import paho.mqtt.client as mqtt

UDP_PORT = 7777

# transparent の指数和係数 (dyne-core::transparent と同一の構成式・固定パラメータ)。
# 注意: PC側実装は Python の math (プラットフォームの libm) であり、シミュレータの
# libm (純Rust) と最終ビットが一致する保証はない。実機ノード間の一貫性は
# 「全ノードへ同一係数を注入する」ことで担保される (規約 §2.1)。
SOE_H, SOE_C1, SOE_C2, SOE_T_REF = 0.6, 1e9, 2e4, 0.2


def transparent_coef(alpha, dt):
    x_lo = 0.5 * math.log(1.0 / (SOE_C1 * SOE_T_REF))
    x_hi = 0.5 * math.log(SOE_C2 / dt)
    kfac = 1.0 / math.sqrt(math.pi * alpha)
    wfac = (2.0 / math.sqrt(math.pi)) * SOE_H
    decay, half, weight = [], [], []
    j = 0
    while True:
        x = x_lo + j * SOE_H
        if x >= x_hi:
            break
        lam = math.exp(2 * x)
        decay.append(math.exp(-lam * dt))
        half.append(math.exp(-lam * 0.5 * dt))
        weight.append(wfac * math.exp(x) * kfac)
        j += 1
    return decay, half, weight


def split(points, nodes):
    """dyne-sim と同一の分割: base + 先頭 extra ノードに1点ずつ"""
    base, extra = divmod(points, nodes)
    starts, idx = [], 0
    for i in range(nodes):
        cnt = base + (1 if i < extra else 0)
        starts.append((idx, cnt))
        idx += cnt
    return starts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default="localhost")
    ap.add_argument("--nodes", type=int, required=True)
    ap.add_argument("--points", type=int, default=65)
    ap.add_argument("--steps", type=int, default=2000)
    ap.add_argument("--alpha", type=float, default=1.0)
    ap.add_argument("--dt", type=float, default=1e-4)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--bc", default="zero_grad",
                    choices=["dirichlet", "zero_grad", "linear_extrap", "transparent"],
                    help="不在隣接への面の境界条件キー (外縁は常に dirichlet 0)")
    ap.add_argument("--dirichlet-value", type=float, default=0.0)
    ap.add_argument("--absent", default="", help="恒久不在ノードのカンマ区切り (部分観測)")
    ap.add_argument("--peers", required=True,
                    help="ノード順のIPカンマ区切り (不在ノードは '-')")
    ap.add_argument("--sensor", default="none", choices=["none", "dht20", "bme280"])
    ap.add_argument("--sensor-period-ms", type=int, default=5000)
    ap.add_argument("--rssi-period-ms", type=int, default=10000)
    ap.add_argument("--field-every", type=int, default=50)
    ap.add_argument("--residual-every", type=int, default=10)
    ap.add_argument("--rt-every", type=int, default=100)
    ap.add_argument("--start", action="store_true", help="設定注入後に start を発行")
    ap.add_argument("--dry-run", action="store_true", help="接続せずJSONを表示")
    a = ap.parse_args()

    absent = set(int(s) for s in a.absent.split(",") if s != "")
    ips = a.peers.split(",")
    assert len(ips) == a.nodes, "--peers はノード数と同数"
    starts = split(a.points, a.nodes)
    h = 1.0 / (a.points - 1)

    def ubc_obj():
        if a.bc == "dirichlet":
            return {"key": "dirichlet", "value": a.dirichlet_value}
        if a.bc == "transparent":
            d, hf, w = transparent_coef(a.alpha, a.dt)
            return {"key": "transparent", "m": len(d), "decay": d, "half": hf, "weight": w}
        return {"key": a.bc}

    cfgs = {}
    for i in range(a.nodes):
        if i in absent:
            continue
        s, cnt = starts[i]
        bc, ifaces = {}, []
        for side, nb, iface_id in (("left", i - 1, i - 1), ("right", i + 1, i)):
            edge = nb < 0 or nb >= a.nodes
            if edge:
                bc[side] = {"key": "dirichlet", "value": 0.0}  # グローバル外縁
            elif nb in absent:
                bc[side] = ubc_obj()                            # 未観測領域境界 (F1)
            else:
                bc[side] = {"key": "interface"}
                ifaces.append({"id": iface_id, "face": side,
                               "peer": f"{ips[nb]}:{UDP_PORT}"})
        cfgs[i] = {
            "ver": 1,
            "cluster": {"n_nodes": a.nodes, "points": a.points, "alpha": a.alpha,
                        "dt": a.dt, "steps": a.steps, "seed": a.seed},
            "node": {"idx": i, "n": cnt, "x0": s * h, "x1": (s + cnt - 1) * h},
            "bc": bc,
            "ifaces": ifaces,
            "mode": {"role": "compute", "sensor": a.sensor,
                     "sensor_period_ms": a.sensor_period_ms,
                     "rssi_period_ms": a.rssi_period_ms,
                     "field_every": a.field_every,
                     "residual_every": a.residual_every,
                     "rt_every": a.rt_every},
        }

    if a.dry_run:
        for i, c in cfgs.items():
            print(f"---- dyne/cfg/{i} ----")
            print(json.dumps(c, indent=1)[:2000])
        return

    cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    cl.connect(a.broker, 1883, 60)
    cl.loop_start()
    for i, c in cfgs.items():
        r = cl.publish(f"dyne/cfg/{i}", json.dumps(c), qos=1, retain=True)
        r.wait_for_publish()
        print(f"published dyne/cfg/{i} (retained, {len(json.dumps(c))} bytes)")
    if a.start:
        time.sleep(1.0)  # ack を待つ猶予 (厳密には mqtt_observe で ack を確認してから)
        r = cl.publish("dyne/ctl/all", json.dumps({"cmd": "start"}), qos=1)
        r.wait_for_publish()
        print("published dyne/ctl/all start")
    cl.loop_stop()


if __name__ == "__main__":
    main()

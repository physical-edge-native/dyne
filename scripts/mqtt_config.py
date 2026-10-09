#!/usr/bin/env python3
"""1次元チェーンの設定を生成して MQTT 制御面へ注入する (発注7、2026-09-14)。

規約: docs/2026-09-14-mqtt-control-plane.md §2.1〜2.3。
格子の分割はシミュレータ (dyne-sim) と同一規則 (points を nodes で割り、余りを
先頭ノードから1点ずつ)。外縁は --outer-bc (既定 dirichlet:0、実験1は zero_grad)、
不在隣接への面は --bc のキー。

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


def resolve_peers(broker, nodes, absent, timeout=3.0):
    """--peers auto (2026-09-20): status の retained (online の MAC⇔IP 対応) から
    ノードID順に IP を自動解決する。ノードは MQTT (再)接続のたびに online
    (IPつき・retained) を貼り直すため、ホットスポット再起動後もそのまま使える。
    IP が取れないノード (retained が configured/done 等で IP を含まない) があれば
    エラーで列挙する (対処: ノードを再起動 or Wi-Fi 再接続で online を貼り直す)。"""
    got = {}  # node_id -> (state, ip or None)

    def on_msg(cl, u, m):
        parts = m.topic.split("/")
        if len(parts) != 4 or parts[1] != "node" or parts[3] != "status":
            return
        try:
            d = json.loads(m.payload)
        except Exception:
            return
        nid = int(parts[2])
        ip = d.get("ip")
        if ip:
            got[nid] = (d.get("state"), ip)
        elif nid not in got:
            got[nid] = (d.get("state"), None)

    cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    cl.on_message = on_msg
    cl.connect(broker, 1883, 60)
    cl.subscribe("dyne/node/+/status", qos=1)
    cl.loop_start()
    time.sleep(timeout)
    cl.loop_stop()
    cl.disconnect()

    ips, missing = [], []
    for i in range(nodes):
        if i in absent:
            ips.append("-")
            continue
        st = got.get(i)
        if st and st[1]:
            ips.append(st[1])
            print(f"  peers auto: node {i} -> {st[1]} (state={st[0]})")
        else:
            missing.append(f"node {i} ({st[0] if st else 'statusなし'})")
    if missing:
        raise SystemExit(
            "peers auto: IPを解決できないノードがある: " + ", ".join(missing)
            + "\n対処: 当該ノードを再起動 (またはWi-Fi再接続) して online を貼り直すこと")
    return ips


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
    ap.add_argument("--stop", action="store_true",
                    help="ctl: stop だけを発行して終了 (設定注入なし)。ストール時の"
                         "復旧手順 stop→再設定→再start の第1手 (規約 §3.5)")
    ap.add_argument("--nodes", type=int)
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
    ap.add_argument("--absent-observe", action="store_true",
                    help="不在ノードにも「観測のみ」役 (mode.role=observe) の cfg を注入"
                         "する (計算に参加せずセンサ・RSSIだけ発行。部分観測runで"
                         "未観測区間の実温度を記録するため。2026-09-22)")
    ap.add_argument("--peers",
                    help="ノード順のIPカンマ区切り (不在ノードは '-')。"
                         "'auto' で status の online (retained) からノードID順に自動解決")
    ap.add_argument("--sensor", default="none",
                    help="センサ種別: none|dht20|bme280。カンマ区切りでノードごとに"
                         "指定可 (例: dht20,bme280,none,dht20)。単一値は全ノード共通")
    ap.add_argument("--sensor-period-ms", type=int, default=5000)
    ap.add_argument("--rssi-period-ms", type=int, default=10000)
    ap.add_argument("--field-every", type=int, default=50)
    ap.add_argument("--residual-every", type=int, default=10)
    ap.add_argument("--rt-every", type=int, default=100)
    ap.add_argument("--outer-bc", default="dirichlet",
                    choices=["dirichlet", "zero_grad"],
                    help="全域外縁 (両端) の境界条件 (発注11。実験1は zero_grad)")
    ap.add_argument("--length", type=float, default=1.0,
                    help="空間の全長 L[m] (発注11。格子幅 h=L/(points-1))")
    ap.add_argument("--calib", default="",
                    help="ノード順の校正オフセット[℃]カンマ区切り (省略=全0)")
    ap.add_argument("--holdout", default="-1",
                    help="保留ノード番号 (抜き取り検証。-1=なし)。'all' で全ノード"
                         "保留=同化なし対照run (γ>0のままイノベーション記録のみ。"
                         "ファームはγ=0だとinnovを発行しないため、実機の対照は"
                         "この形で行う。2026-09-23)")
    ap.add_argument("--step-period-ms", type=int, default=0,
                    help="1ステップの実時間周期[ms] (0=従来の最速。実験1は1000)")
    ap.add_argument("--init", type=float, default=None,
                    help="初期場の一様値[℃] (実測平均。省略=従来のsin初期値)")
    ap.add_argument("--gamma", type=float, default=0.0,
                    help="観測点ナッジングの強度 γ (0=無効。発注8)")
    ap.add_argument("--obs-period", type=int, default=10, help="観測周期 (ステップ)")
    ap.add_argument("--obs-point", type=int, default=-1,
                    help="観測点 = 局所格子点 (-1 = 中央 n/2)")
    ap.add_argument("--start", action="store_true", help="設定注入後に start を発行")
    ap.add_argument("--dry-run", action="store_true", help="接続せずJSONを表示")
    a = ap.parse_args()

    if a.stop:
        # 単独発行モード: 設定注入なしに全ノードへ stop (走行中でも受理され、
        # 設定保持のまま idle へ戻る。復旧は 再設定 → --start)
        cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        cl.connect(a.broker, 1883, 60)
        cl.loop_start()
        # ctl は retain 禁止 (規約 §3.6: 号令は鮮度が命。貼り置くと start 直後に
        # 再配達されて即停止する事故になる)。明示的に retain=False で発行する
        r = cl.publish("dyne/ctl/all", json.dumps({"cmd": "stop"}), qos=1, retain=False)
        r.wait_for_publish()
        cl.loop_stop()
        print("published dyne/ctl/all stop (retain=False)")
        return

    assert a.nodes is not None and a.peers is not None, "--nodes と --peers は必須 (--stop 単独時を除く)"
    absent = set(int(s) for s in a.absent.split(",") if s != "")
    if a.peers == "auto":
        ips = resolve_peers(a.broker, a.nodes, absent)
    else:
        ips = a.peers.split(",")
    assert len(ips) == a.nodes, "--peers はノード数と同数"
    starts = split(a.points, a.nodes)
    h = a.length / (a.points - 1)
    r = a.alpha * a.dt / (h * h)
    print(f"r = alpha*dt/h^2 = {r:.4f}  (h={h:.4f} m)")
    if r > 0.5:
        print(f"!!! 警告: r={r:.3f} > 1/2 — 差分法の安定条件を破っており発散する。"
              f" dt を {0.5*h*h/a.alpha:.3f} 秒以下にするか alpha/length を見直すこと"
              f" (step_period_ms も合わせる)")
    calib = [float(x) for x in a.calib.split(",")] if a.calib else [0.0] * a.nodes
    assert len(calib) == a.nodes, "--calib must have one value per node"
    hold_all = a.holdout == "all"
    hold_n = -1 if hold_all else int(a.holdout)
    sensors = a.sensor.split(",")
    if len(sensors) == 1:
        sensors = sensors * a.nodes
    assert len(sensors) == a.nodes, "--sensor: 単一値かノード数ぶんのカンマ区切り"
    for kind in sensors:
        assert kind in ("none", "dht20", "bme280"), f"--sensor: 不正な種別 {kind}"

    def ubc_obj():
        if a.bc == "dirichlet":
            return {"key": "dirichlet", "value": a.dirichlet_value}
        if a.bc == "transparent":
            d, hf, w = transparent_coef(a.alpha, a.dt)
            return {"key": "transparent", "m": len(d), "decay": d, "half": hf, "weight": w}
        return {"key": a.bc}

    cfgs = {}
    for i in range(a.nodes):
        if i in absent and not a.absent_observe:
            continue
        s, cnt = starts[i]
        if i in absent:
            # 「観測のみ」役: 計算に参加せずセンサ・RSSIだけ発行 (ファームは
            # role=observe で BSP を起動しない)。bc/ifaces は計算に使われないため
            # zero_grad・空でよい (cfg 検証は cluster/node/bc の存在と n≥3 のみ)
            cfgs[i] = {
                "ver": 1,
                "gen": int(time.time()),
                "cluster": {"n_nodes": a.nodes, "points": a.points, "alpha": a.alpha,
                            "dt": a.dt, "steps": a.steps, "seed": a.seed,
                            "step_period_ms": a.step_period_ms},
                "node": {"idx": i, "n": cnt, "x0": s * h, "x1": (s + cnt - 1) * h,
                         "calib_offset": calib[i]},
                "bc": {"left": {"key": "zero_grad"}, "right": {"key": "zero_grad"}},
                "ifaces": [],
                "mode": {"role": "observe", "sensor": sensors[i],
                         "sensor_period_ms": a.sensor_period_ms,
                         "rssi_period_ms": a.rssi_period_ms,
                         "field_every": 0, "residual_every": 0, "rt_every": 0},
                "assim": {"gamma": 0.0, "obs_period": a.obs_period,
                          "obs_point": -1, "holdout": False},
            }
            continue
        bc, ifaces = {}, []
        for side, nb, iface_id in (("left", i - 1, i - 1), ("right", i + 1, i)):
            edge = nb < 0 or nb >= a.nodes
            if edge:
                # グローバル外縁 (--outer-bc。実験1=zero_grad、既定=従来dirichlet:0)
                bc[side] = ({"key": "zero_grad"} if a.outer_bc == "zero_grad"
                            else {"key": "dirichlet", "value": 0.0})  # グローバル外縁
            elif nb in absent:
                bc[side] = ubc_obj()                            # 未観測領域境界 (F1)
            else:
                bc[side] = {"key": "interface"}
                ifaces.append({"id": iface_id, "face": side,
                               "peer": f"{ips[nb]}:{UDP_PORT}"})
        cfgs[i] = {
            "ver": 1,
            # 世代番号 (規約 §3.6 鮮度管理): 注入のたびに単調増加。ノードは ack と
            # status (configured) にエコーし、PC側で全ノードの世代一致を照合できる
            "gen": int(time.time()),
            "cluster": {"n_nodes": a.nodes, "points": a.points, "alpha": a.alpha,
                        "dt": a.dt, "steps": a.steps, "seed": a.seed,
                        "step_period_ms": a.step_period_ms},
            "node": {"idx": i, "n": cnt, "x0": s * h, "x1": (s + cnt - 1) * h,
                     "calib_offset": calib[i],
                     **({"init": a.init} if a.init is not None else {})},
            "bc": bc,
            "ifaces": ifaces,
            "mode": {"role": "compute", "sensor": sensors[i],
                     "sensor_period_ms": a.sensor_period_ms,
                     "rssi_period_ms": a.rssi_period_ms,
                     "field_every": a.field_every,
                     "residual_every": a.residual_every,
                     "rt_every": a.rt_every},
            "assim": {"gamma": a.gamma, "obs_period": a.obs_period,
                      "obs_point": a.obs_point,
                      "holdout": hold_all or i == hold_n},
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
        r = cl.publish("dyne/ctl/all", json.dumps({"cmd": "start"}), qos=1, retain=False)
        r.wait_for_publish()
        print("published dyne/ctl/all start (retain=False)")
    cl.loop_stop()


if __name__ == "__main__":
    main()

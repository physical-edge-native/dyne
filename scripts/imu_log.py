# 実験9 (IMU の協調によるノイズ除去と時刻同期) の記録: MQTT で届く IMU の生の標本とアクターの
# 融合の出力を CSV に書く。CSV の形式は docs/2026-11-13-imu-fusion.md §5 (解析は scripts/imu_analyze.py)。
#
# 出所: Claude Code が 2026-11-13 に作成。生の標本の JSON はファーム (node-esp32/main/dyne_imu.c の冒頭) の形:
#   dyne/node/<id>/imu   {"seq":バッチの通番,"odr":104,"fs_g":4,"fs_dps":500,"sa":<g/LSB>,"sg":<dps/LSB>,
#                         "synced":true,"us_per_tick":..,"fit_resid_us":..,"ovr":..,"orphan":..,
#                         "s":[[t_us, t_loc, ax, ay, az, gx, gy, gz], ...]}
#     t_us = 同期した時刻 [µs] (未同期は 0)、t_loc = esp_timer [µs]、ax..gz = 生の値 (符号つき16ビット)。
#     加速度 = 生 × sa [g] × 9.80665、角速度 = 生 × sg [dps] × π/180 にして SI で書く。
# 融合の出力 (ファームへの依頼。docs/2026-11-13-imu-fusion.md §5.3):
#   dyne/node/<id>/imu_fused  {"step":s, "v":[dyne_get(0)..dyne_get(21)]}   (区間の境目ごと)
#
# 使い方 (リポジトリ直下で):
#   .venv\Scripts\python.exe scripts\imu_log.py --broker localhost --out measured\2026-11-xx-imu\static1
#   (止めるときは Ctrl+C。条件ごとに --out を分ける。水平器の値は level.csv に手で書く)
import argparse
import csv
import json
import math
import os
import time

import paho.mqtt.client as mqtt

G_LEN = 23  # 22 before 2026-11-27 (issue #194: G_FUSED_DEVICES added)
G0 = 9.80665


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--broker', default='localhost')
    ap.add_argument('--out', required=True)
    ap.add_argument('--epoch-us', type=int, default=200000, help='区間の長さ [µs] (fused の epoch_end_us の計算用)')
    ap.add_argument('--t0-us', type=int, default=0, help='アクターに入れた CFG_T0_US')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    fr = open(os.path.join(a.out, 'imu_raw.csv'), 'a', newline='')
    ff = open(os.path.join(a.out, 'imu_fused.csv'), 'a', newline='')
    fb = open(os.path.join(a.out, 'imu_batches.csv'), 'a', newline='')
    wr, wf, wb = csv.writer(fr), csv.writer(ff), csv.writer(fb)
    if fr.tell() == 0:
        wr.writerow(['node', 'seq', 'ts_us', 'local_us', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c'])
    if ff.tell() == 0:
        wf.writerow(['node', 'step', 'epoch', 'epoch_end_us'] + ['v%d' % k for k in range(G_LEN)])
    if fb.tell() == 0:
        wb.writerow(['pc_time', 'node', 'batch', 'n', 'synced', 'ovr', 'orphan', 'us_per_tick', 'fit_resid_us'])
    seqn = {}
    cnt = {'imu': 0, 'fused': 0, 'bad': 0, 'unsynced': 0}

    def on_connect(cl, u, flags, rc, props=None):
        print('connected rc=%s' % rc)
        cl.subscribe('dyne/node/+/imu', qos=0)
        cl.subscribe('dyne/node/+/imu_fused', qos=0)

    def on_message(cl, u, msg):
        parts = msg.topic.split('/')
        try:
            node = int(parts[2])
            d = json.loads(msg.payload)
        except (ValueError, IndexError):
            cnt['bad'] += 1
            return
        if parts[3] == 'imu':
            sa = float(d['sa']) * G0
            sg = float(d['sg']) * math.pi / 180.0
            s = d.get('s', [])
            wb.writerow(['%.3f' % time.time(), node, d.get('seq'), len(s), d.get('synced'), d.get('ovr'),
                         d.get('orphan'), d.get('us_per_tick'), d.get('fit_resid_us')])
            for row in s:
                t_us, t_loc = int(row[0]), int(row[1])
                if t_us == 0:
                    cnt['unsynced'] += 1
                k = seqn.get(node, 0)
                seqn[node] = k + 1
                wr.writerow([node, k, t_us, t_loc] + ['%.6f' % (x * sa) for x in row[2:5]] +
                            ['%.7f' % (x * sg) for x in row[5:8]] + [''])
            cnt['imu'] += 1
        elif parts[3] == 'imu_fused':
            v = d['v']
            e = int(v[0])
            wf.writerow([node, d['step'], e, a.t0_us + (e + 1) * a.epoch_us] + [repr(float(x)) for x in v])
            cnt['fused'] += 1

    cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    cl.on_connect = on_connect
    cl.on_message = on_message
    cl.connect(a.broker, 1883, 60)
    cl.loop_start()
    try:
        while True:
            time.sleep(5)
            for f in (fr, ff, fb):
                f.flush()
            print('imu batches %(imu)d, fused %(fused)d, bad %(bad)d, unsynced samples %(unsynced)d' % cnt)
    except KeyboardInterrupt:
        pass
    cl.loop_stop()
    for f in (fr, ff, fb):
        f.close()


if __name__ == '__main__':
    main()

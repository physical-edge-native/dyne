#!/usr/bin/env python3
"""PC 側の常駐版: ノードから形の区画を受け取り、全体の形にまとめ、定期的に USD・MuJoCo の模型・
ダッシュボードを書き出す(計画の G5。docs/2026-11-13-pc-pipeline-rl.md)。2026-11-13。

PC がするのは受信・まとめ・書き出し・表示だけ。形の計算(測距の統合・距離の場)は各ノードのアクターが行い、
PC はその結果(φ・d・w)を区画ごとに受け取って並べるだけ(CLAUDE.md の前提: PC の責務は設定注入・監視・
可視化・書き出し)。

流れ:
  1. MQTT で dyne/node/+/shape(scripts/dyne_shape_msg.py の形式)を受け、ノードごとに最新の区画を持つ
  2. --every 秒ごとに(新しい区画が届いていれば)全体の格子に並べる。届いていない区画は未観測(w = 0)
  3. scripts/twin_demo.py を呼び、frames/<時刻>_<通し番号>/ に room.usda・room.xml・room.html・room.json を書く
     (USD の prim のパスは書き出し直しても変わらない。twin_physics.py の冒頭を参照)
  4. latest/ に同じ名前で写す(置き換えは一時ファイルからの os.replace なので、読み手が書きかけを見ない)。
     Windows では記号リンクに管理者の権限が要るので、リンクでなく写しにした
  5. HTTP サーバ(既定 127.0.0.1:8765)で出力先を配る。ダッシュボードは
       http://127.0.0.1:8765/latest/room.html?poll=3
     で開くと、3 秒ごとに latest/room.json を読み直して表示を差し替える
  latest/status.json に、ノードごとの最終受信・ステップ・時刻印の遅れを書く。

使い方(リポジトリの .venv で):
  .venv\\Scripts\\python.exe scripts\\twin_live.py --broker 127.0.0.1 --scene crates\\dyne-sim2d\\scenes\\room6_move.txt
  (実機なしの通しの試験は scripts\\twin_live_demo.ps1。偽のノードは scripts\\twin_fake_nodes.py)
"""
import argparse
import datetime
import functools
import http.server
import json
import os
import shutil
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import dyne_shape_msg as msg  # noqa: E402


class Tiles:
    """ノードごとの最新の区画(受信のスレッドと書き出しのスレッドで共有)"""

    def __init__(self):
        self.lock = threading.Lock()
        self.tiles = {}
        self.rx = 0
        self.errors = 0
        self.last_error = ""
        self.version = 0

    def put(self, t):
        with self.lock:
            t["recv"] = time.time()
            self.tiles[t["node"]] = t
            self.rx += 1
            self.version += 1

    def bad(self, e):
        with self.lock:
            self.errors += 1
            self.last_error = str(e)

    def snapshot(self):
        with self.lock:
            return dict(self.tiles), self.version


def assemble(tiles):
    """区画を全体の格子に並べる。戻り値 dict(phi, d, w, x0, y0, h, nodes) か None"""
    if not tiles:
        return None
    ts = list(tiles.values())
    gnx = max(max(t["gnx"] for t in ts), max(t["ix0"] + t["nx"] for t in ts))
    gny = max(max(t["gny"] for t in ts), max(t["iy0"] + t["ny"] for t in ts))
    h = ts[0]["h"]
    x0, y0 = ts[0]["x0"], ts[0]["y0"]
    for t in ts:
        if abs(t["h"] - h) > 1e-6 or abs(t["x0"] - x0) > 1e-6 or abs(t["y0"] - y0) > 1e-6:
            raise ValueError("ノード %d の格子 (h=%g, 原点 %g,%g) が他と違う" % (t["node"], t["h"], t["x0"], t["y0"]))
    phi = np.zeros((gny, gnx))
    d = np.zeros((gny, gnx))
    w = np.zeros((gny, gnx))
    for t in ts:
        j0, i0 = t["iy0"], t["ix0"]
        phi[j0:j0 + t["ny"], i0:i0 + t["nx"]] = t["phi"]
        d[j0:j0 + t["ny"], i0:i0 + t["nx"]] = t["d"]
        w[j0:j0 + t["ny"], i0:i0 + t["nx"]] = t["w"]
    return {"phi": phi, "d": d, "w": w, "x0": x0, "y0": y0, "h": h, "nx": gnx, "ny": gny}


def write_inputs(field, tiles, frame_dir, now):
    """twin_demo.py の入力(形の CSV と、ノードの区画を書いた記録)を書く"""
    os.makedirs(frame_dir, exist_ok=True)
    csv_path = os.path.join(frame_dir, "shape.csv")
    h, x0, y0 = field["h"], field["x0"], field["y0"]
    lines = ["gx,gy,x,y,phi,d,w,truth"]
    for j in range(field["ny"]):
        for i in range(field["nx"]):
            lines.append("%d,%d,%.6f,%.6f,%.7e,%.7e,%.7e," % (i, j, x0 + i * h, y0 + j * h,
                                                            field["phi"][j, i], field["d"][j, i], field["w"][j, i]))
    with open(csv_path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    log_path = os.path.join(frame_dir, "live.log")
    L = ["twin_live: %s | nodes %d | grid %dx%d h = %g m" % (now, len(tiles), field["nx"], field["ny"], h)]
    for nid in sorted(tiles):
        t = tiles[nid]
        # twin_demo.py の read_log が読む node 行の形(通信の統計はノードから来ないので 0)
        L.append("node %2d: [%d..%d)x[%d..%d) ifaces [] sent 0 (0 B) lost 0 resend 0 step %d age %.1f s sensors []"
                 % (nid, t["ix0"], t["ix0"] + t["nx"], t["iy0"], t["iy0"] + t["ny"], t["step"], time.time() - t["recv"]))
    with open(log_path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(L) + "\n")
    return csv_path, log_path


def publish_latest(frame_dir, latest_dir, names):
    os.makedirs(latest_dir, exist_ok=True)
    for n in names:
        src = os.path.join(frame_dir, n)
        if not os.path.exists(src):
            continue
        tmp = os.path.join(latest_dir, "." + n + ".tmp")
        shutil.copyfile(src, tmp)
        for _ in range(20):  # 読み手が開いている間は Windows で置き換えに失敗することがある
            try:
                os.replace(tmp, os.path.join(latest_dir, n))
                break
            except PermissionError:
                time.sleep(0.05)


def write_json_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1)
    for _ in range(20):
        try:
            os.replace(tmp, path)
            return
        except PermissionError:
            time.sleep(0.05)


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, *args):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--out", default=os.path.join("demo_out", "twin_live"), help="出力先(frames/ と latest/ を作る)")
    ap.add_argument("--every", type=float, default=5.0, help="書き出しの間隔 [秒](新しい区画が無ければ書き出さない)")
    ap.add_argument("--min-nodes", type=int, default=1, help="この台数の区画が揃うまで書き出さない")
    ap.add_argument("--keep", type=int, default=30, help="frames/ に残す書き出しの数(古いものから消す。0 = 全部残す)")
    ap.add_argument("--http-port", type=int, default=8765, help="ダッシュボードを配る HTTP の口(0 = 配らない)")
    ap.add_argument("--http-bind", default="127.0.0.1")
    ap.add_argument("--once", action="store_true", help="1 回書き出したら終える(試験用)")
    ap.add_argument("--max-exports", type=int, default=0, help="この回数書き出したら終える(0 = 無限)")
    ap.add_argument("--timeout", type=float, default=0.0, help="この秒数で終える(0 = 無限)")
    # twin_demo.py へそのまま渡すもの
    ap.add_argument("--scene", default=None, help="場面ファイル・図面(省略可。twin_demo.py の --scene)")
    ap.add_argument("--room-height", type=float, default=2.4)
    ap.add_argument("--zones", type=int, default=8)
    ap.add_argument("--fov-deg", type=float, default=45.0)
    ap.add_argument("--unobserved-solid", choices=["on", "off"], default="on")
    ap.add_argument("--collider-shape", choices=["box", "mesh"], default="box")
    ap.add_argument("--title", default="Dyne 部屋のデジタルツイン(常駐版)")
    a = ap.parse_args()
    import paho.mqtt.client as mqtt

    out = os.path.abspath(a.out)
    frames_dir = os.path.join(out, "frames")
    latest = os.path.join(out, "latest")
    os.makedirs(frames_dir, exist_ok=True)
    os.makedirs(latest, exist_ok=True)
    tiles = Tiles()

    def on_connect(cl, u, flags, rc, props=None):
        cl.subscribe("dyne/node/+/shape", 0)
        print("MQTT: %s:%d に接続、dyne/node/+/shape を購読" % (a.broker, a.port), flush=True)

    def on_message(cl, u, m):
        try:
            t = msg.unpack(m.payload)
            want = int(m.topic.split("/")[2])
            if want != t["node"]:
                raise ValueError("トピックのノード番号 %d と中身 %d が違う" % (want, t["node"]))
            tiles.put(t)
        except Exception as e:  # noqa: BLE001  壊れたメッセージは数えて捨てる
            tiles.bad("%s: %s" % (m.topic, e))

    cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="dyne-twin-live-%d" % os.getpid())
    cl.on_connect = on_connect
    cl.on_message = on_message
    cl.connect(a.broker, a.port, 60)
    cl.loop_start()

    httpd = None
    if a.http_port:
        handler = functools.partial(QuietHandler, directory=out)
        httpd = http.server.ThreadingHTTPServer((a.http_bind, a.http_port), handler)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        print("ダッシュボード: http://%s:%d/latest/room.html?poll=3" % (a.http_bind, a.http_port), flush=True)

    py = sys.executable
    seq = 0
    seen_version = 0
    t_start = time.time()
    next_t = time.time() + a.every
    history = []
    try:
        while True:
            if a.timeout and time.time() - t_start > a.timeout:
                print("時間切れで終える", flush=True)
                break
            time.sleep(max(0.0, min(0.2, next_t - time.time())))
            if time.time() < next_t:
                continue
            next_t = max(next_t + a.every, time.time())
            snap, ver = tiles.snapshot()
            t_snap = time.time()
            if ver == seen_version or len(snap) < a.min_nodes:
                continue
            seen_version = ver
            seq += 1
            now = datetime.datetime.now(datetime.timezone.utc)
            stamp = now.strftime("%Y-%m-%dT%H:%M:%SZ")
            frame_dir = os.path.join(frames_dir, "%s_%06d" % (now.strftime("%Y%m%dT%H%M%SZ"), seq))
            t0 = time.time()
            try:
                field = assemble(snap)
            except ValueError as e:
                print("まとめられない: %s" % e, flush=True)
                continue
            csv_path, log_path = write_inputs(field, snap, frame_dir, stamp)
            cmd = [py, os.path.join(HERE, "twin_demo.py"), "--shape-csv", csv_path, "--sim-log", log_path,
                   "--out", os.path.join(frame_dir, "room"), "--sequence", str(seq), "--timestamp", stamp, "--json",
                   "--room-height", str(a.room_height), "--zones", str(a.zones), "--fov-deg", str(a.fov_deg),
                   "--unobserved-solid", a.unobserved_solid, "--collider-shape", a.collider_shape, "--title", a.title]
            if a.scene:
                cmd += ["--scene", a.scene]
            p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                               env=dict(os.environ, PYTHONIOENCODING="utf-8"))
            if p.returncode != 0:
                print("twin_demo.py が失敗 (%d): %s" % (p.returncode, p.stderr[-800:]), flush=True)
                continue
            publish_latest(frame_dir, latest, ["room.usda", "room.xml", "room.json", "room.html"])
            dt = time.time() - t0
            steps = [t["step"] for t in snap.values()]
            # 時刻印の遅れ = まとめた時点の時計 − 区画の時刻印(同じ時計に同期している前提。偽のノードは PC の時計)
            lag = [(t_snap * 1e6 - t["t_us"]) / 1e6 for t in snap.values() if t["t_us"]]
            w_obs = float((field["w"] > 0).mean())
            status = {"sequence": seq, "createdUtc": stamp, "frame": os.path.relpath(frame_dir, out).replace("\\", "/"),
                      "exportSeconds": round(dt, 3), "nodes": len(snap), "messages": tiles.rx, "badMessages": tiles.errors,
                      "lastError": tiles.last_error, "grid": [field["nx"], field["ny"], field["h"]],
                      "observedFraction": round(w_obs, 4), "stepRange": [min(steps), max(steps)],
                      "perNode": {str(k): {"step": t["step"], "ageS": round(t_snap - t["recv"], 2),
                                           "region": [t["ix0"], t["iy0"], t["nx"], t["ny"]]} for k, t in sorted(snap.items())}}
            write_json_atomic(os.path.join(latest, "status.json"), status)
            summary = [l for l in p.stdout.splitlines() if l.startswith(("推定表面:", "衝突判定:", "未観測"))]
            print("[#%d %s] ノード %d 台、ステップ %d〜%d、観測済み %.1f%%、時刻印の遅れ %s、書き出し %.2f 秒 | %s"
                  % (seq, stamp, len(snap), min(steps), max(steps), w_obs * 100,
                     ("%.2f〜%.2f 秒" % (min(lag), max(lag))) if lag else "-", dt, " / ".join(summary)), flush=True)
            history.append(frame_dir)
            if a.keep and len(history) > a.keep:
                old = history.pop(0)
                shutil.rmtree(old, ignore_errors=True)
            if a.once or (a.max_exports and seq >= a.max_exports):
                break
    except KeyboardInterrupt:
        pass
    cl.loop_stop()
    cl.disconnect()
    if httpd:
        httpd.shutdown()
    print("受信 %d 通(壊れていたもの %d 通%s)、書き出し %d 回"
          % (tiles.rx, tiles.errors, (": " + tiles.last_error) if tiles.errors else "", seq), flush=True)


if __name__ == "__main__":
    main()

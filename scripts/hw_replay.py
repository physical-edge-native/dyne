#!/usr/bin/env python3
"""実機のノードがアクターへ渡した入力を dyne-sim2d で再生し、ノードが報告した CRC32 とビット一致を確かめる
(2026-11-15、GitHub issue #2。説明と記録の形式: docs/2026-11-15-hw-replay.md)。

実機の測距・IMU の値はその場限りの入力なので、「実機の結果 = シミュレータの結果」は、ノードがアクターへ
渡した入力をそのまま記録してシミュレータで再生したときにだけ確かめられる。

サブコマンド(リポジトリのルートで、.venv の Python で実行する。record 以外は標準ライブラリだけで動く):
  record  MQTT の dyne/# を記録する(1行1メッセージの JSON Lines)。実機の実行の間、回しておく(paho-mqtt が要る)
            .venv\\Scripts\\python.exe scripts\\hw_replay.py record --broker 127.0.0.1 --out run.jsonl
  check   記録 → 再生の入力(テキスト)に変換 → dyne-sim2d --replay で再生 → ノードごとに PASS/FAIL/INCOMPLETE
            .venv\\Scripts\\python.exe scripts\\hw_replay.py check run.jsonl --wasm <アクター.wasm> -- <dyne-sim2d の引数>
          (-- の後ろは実機の実行と同じ分割・格子・ステップ数にする dyne-sim2d の引数。例: --scene ... --px 2 --py 2 --steps 300)
          --single を付けると単一ノード解との比較もする(既定はしない。判定は実機とシミュレータの比較)
  convert 記録 → 再生の入力(テキスト)だけを作る(check の前半)
  corrupt 試験用: 記録の1つの値の1ビットを反転する(既定は仮数のビット 40。FAIL になることの確認)
  synth-imu 試験用: IMU の融合のアクター向けの合成の記録(imu_fed と actor_setup。CRC は無い)を作る

実機の側は、実行の設定(dyne/cfg/<id>)に "replay":{"log":true} を入れた実行だけが記録を発行する(QoS 1)。
記録の形(1行 = {"t": PC の受信時刻 [s], "topic": "dyne/node/<id>/<種類>", "payload": JSON})で使う種類:
  range_fed    ノードが受信メールボックスへ渡した測距フレーム(step つき)
  imu_fed      ノードが dyne_imu_push へ渡した IMU の標本
  actor_setup  アクターへの dyne_cfg の全呼び出し・面・初期化の値(無ければシミュレータ自身の設定で再生)
  field2d      途中の CRC32(field_every ごと。ファームの既存の発行。node.idx の番号)
  status       done の CRC32 と計数(ファームの既存の発行。replay_skip = 送らなかった記録の数)

判定: PASS = 記録が揃っていて全ての CRC32 が一致。FAIL = 記録が揃っているのに食い違う(または設定の食い違い・
CRC32 が無い)。INCOMPLETE = 記録が欠けている(MQTT のアウトボックスがあふれた・数が合わない)ので比べられない。
1台でも INCOMPLETE なら、隣の食い違いはその欠けから来うるので、全体を INCOMPLETE とする(終了コード 2)。
"""
import argparse
import json
import os
import random
import re
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TOPIC_RE = re.compile(r"^dyne/node/(-?\d+)/([A-Za-z0-9_]+)$")


def hexbits(v):
    """f64 のビット列を 16 桁の16進数で(再生の入力は値を正確に運ぶため全てビット列で書く)"""
    return "%016x" % struct.unpack("<Q", struct.pack("<d", float(v)))[0]


def load_log(path):
    recs = []
    with open(path, encoding="utf-8") as f:
        for ln, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError as e:
                raise SystemExit("%s:%d: JSON として読めない (%s)" % (path, ln, e))
            m = TOPIC_RE.match(r.get("topic", ""))
            if m and isinstance(r.get("payload"), dict):
                recs.append((float(r.get("t", 0.0)), int(m.group(1)), m.group(2), r["payload"]))
    return recs


def parse_crc(s):
    return int(str(s), 16) if isinstance(s, str) else int(s)


def convert(log_path, steps, idmap, since):
    """記録を読み、ノード(node.idx)ごとに再生の入力の行と、完全さの確認の結果を返す"""
    recs = [r for r in load_log(log_path) if r[0] >= since]
    # MQTT の <id> (dyne_node_id) → node.idx。status と actor_setup の idx から(--map が優先)
    id2idx = {}
    for _, nid, kind, p in recs:
        if kind in ("status", "actor_setup") and isinstance(p.get("idx"), int) and p["idx"] >= 0:
            id2idx[nid] = p["idx"]
    id2idx.update(idmap)

    def idx_of(nid, kind):
        return nid if kind == "field2d" else id2idx.get(nid, nid)  # field2d はファームが node.idx で発行する

    # 実行の区切り: そのノードの最後の actor_setup より後だけを使う(無ければ全て)
    last_setup = {}
    for i, (_, nid, kind, p) in enumerate(recs):
        if kind == "actor_setup":
            last_setup[idx_of(nid, kind)] = i
    nodes = {}

    def node(n):
        # warn = 注意だけ、lost = 記録が欠けている(比べられない)
        return nodes.setdefault(n, {"setup": None, "range": [], "imu": {}, "checks": [], "done": None,
                                    "warn": [], "lost": []})

    for i, (_, nid, kind, p) in enumerate(recs):
        n = idx_of(nid, kind)
        if i < last_setup.get(n, -1):
            continue
        if kind == "actor_setup":
            node(n)["setup"] = p
        elif kind == "range_fed":
            node(n)["range"].append(p)
        elif kind == "imu_fed":
            node(n)["imu"].setdefault(int(p["step"]), []).append(p)
        elif kind == "field2d" and "crc32" in p:
            node(n)["checks"].append((int(p["step"]), parse_crc(p["crc32"]), "field2d"))
        elif kind == "status" and p.get("state") == "done" and "crc32" in p:
            node(n)["done"] = p

    lines = ["dyne-replay 1", "# from %s" % os.path.basename(log_path)]
    info = {}
    for n in sorted(nodes):
        nd = nodes[n]
        st = nd["setup"]
        if st is not None:
            if st.get("trunc"):
                nd["warn"].append("actor_setup was truncated on the node; replaying with the simulator's own setup")
            else:
                ini = st["init"]
                lines.append("setup %d %d %d %d %d %s %s %s" % (n, ini["nx"], ini["ny"], ini["ix0"], ini["iy0"],
                                                               hexbits(ini["h"]), hexbits(ini["alpha"]),
                                                               hexbits(ini["dt"])))
                for k, v in st["cfg"]:
                    lines.append("cfg %d %d %s" % (n, int(k), hexbits(v)))
                for f, (kind, param) in enumerate(st.get("faces", [])):
                    lines.append("face %d %d %d %d" % (n, f, int(kind), int(param)))
        # 測距: ファームは1ステップに最大1枚。同じステップが2回あれば複数の実行が混ざっている
        seen = set()
        for p in sorted(nd["range"], key=lambda q: int(q["step"])):
            s = int(p["step"])
            if (s, p.get("sid")) in seen:
                raise SystemExit("node %d: range_fed のステップ %d が2回ある(複数の実行が混ざっている。--since で切る)"
                                 % (n, s))
            seen.add((s, p.get("sid")))
            cnt, cols = int(p["count"]), int(p["cols"])
            # 距離 [m]: d_m(シミュレータの記録だけが持つ正確な値)があればそれ、無ければファームと同じ d_mm / 1000.0
            ds = p["d_m"] if "d_m" in p else [mm / 1000.0 for mm in p["d_mm"]]
            if len(ds) != cnt or len(p["st"]) != cnt:
                nd["lost"].append("range_fed step %d: count %d but %d values (truncated JSON?)" % (s, cnt, len(ds)))
                continue
            t_us = int(p.get("t_us", 0))
            t_us = t_us if t_us > 0 else 0  # ファームは同期前の時刻を 0 にして渡す (node_v2.c feed_range)
            lines.append("range %d %d %d %d %d %d %s %s" % (n, s, int(p["sid"]), t_us, cnt, cols,
                                                          " ".join(hexbits(d) for d in ds),
                                                          " ".join(str(int(x)) for x in p["st"])))
        n_imu = 0
        for s in sorted(nd["imu"]):
            got = 0
            for p in sorted(nd["imu"][s], key=lambda q: int(q.get("i0", 0))):
                if int(p.get("i0", 0)) != got:
                    nd["lost"].append("imu_fed step %d: samples %d.. missing" % (s, got))
                for smp in p["s"]:
                    lines.append("imu %d %d %d %s" % (n, s, int(smp[0]), " ".join(hexbits(v) for v in smp[1:7])))
                    got += 1
            n_imu += got
        checks = sorted(set(nd["checks"]))
        d = nd["done"]
        if d is not None:
            if steps is None:
                fs = [c[0] for c in checks]
                if not fs:
                    nd["warn"].append("done has a CRC but the step count is unknown (give --steps)")
                else:
                    checks.append((max(fs), parse_crc(d["crc32"]), "done"))
            else:
                checks.append((steps - 1, parse_crc(d["crc32"]), "done"))
            # 完全さ: ノードが送らなかった記録の数と、ノードの計数と記録の数
            if int(d.get("replay_skip", 0)) > 0:
                nd["lost"].append("node skipped %d replay records (MQTT outbox full or disconnected)"
                                  % int(d["replay_skip"]))
            if "replay_skip" not in d and (nd["range"] or n_imu or nd["setup"] is not None):
                nd["warn"].append("done has no replay_skip (firmware before 2026-11-15?)")
            if "range_fed" in d:
                lo, hi = int(d["range_fed"]), int(d["range_fed"]) + int(d.get("range_rej", 0))
                if not lo <= len(nd["range"]) <= hi:
                    nd["lost"].append("range_fed records %d, node counted fed %d rej %d"
                                      % (len(nd["range"]), lo, hi - lo))
            if "imu_pushed" in d:
                calls = int(d["imu_pushed"]) + int(d.get("imu_late", 0)) + int(d.get("imu_early", 0))
                if n_imu != calls:
                    nd["lost"].append("imu_fed samples %d, node pushed %d (or replay.log was off)" % (n_imu, calls))
        else:
            nd["warn"].append("no done status: completeness of the records is unknown")
        for s, c, src in sorted(checks):
            lines.append("check %d %d %08x %s" % (n, s, c, src))
        info[n] = {"range": len(nd["range"]), "imu": n_imu, "checks": len(checks),
                   "setup": st is not None and not st.get("trunc"), "warn": nd["warn"], "lost": nd["lost"]}
    return "\n".join(lines) + "\n", info


def parse_map(items):
    m = {}
    for it in items or []:
        a, b = it.split(":")
        m[int(a)] = int(b)
    return m


def sim_steps(sim_args):
    for i, a in enumerate(sim_args):
        if a == "--steps" and i + 1 < len(sim_args):
            return int(sim_args[i + 1])
    return 500  # dyne-sim2d の既定


def cmd_convert(a):
    text, info = convert(a.log, a.steps, parse_map(a.map), a.since)
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    for n, i in sorted(info.items()):
        print("node %d: range %d, imu %d, checks %d, setup %s" % (n, i["range"], i["imu"], i["checks"],
                                                                 "recorded" if i["setup"] else "simulator"))
        for w in i["lost"]:
            print("  incomplete: " + w)
        for w in i["warn"]:
            print("  warning: " + w)
    print("wrote " + a.out)


def cmd_check(a):
    sim_args = list(a.sim_args)
    steps = sim_steps(sim_args)
    text, info = convert(a.log, steps, parse_map(a.map), a.since)
    out_dir = a.out or os.path.splitext(a.log)[0] + "_replay"
    os.makedirs(out_dir, exist_ok=True)
    rp = os.path.join(out_dir, "replay.txt")
    with open(rp, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    exe = os.path.abspath(a.sim) if a.sim else os.path.join(ROOT, "target", "release", "dyne-sim2d")
    if os.name == "nt" and not exe.lower().endswith(".exe"):
        exe += ".exe"
    if not os.path.isfile(exe):
        raise SystemExit("dyne-sim2d not found: %s (cargo build --release -p dyne-sim2d, or give --sim)" % exe)
    cmd = [exe, "--wasm", a.wasm, "--replay", rp] + (["--replay-single"] if a.single else []) + sim_args
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    with open(os.path.join(out_dir, "sim.log"), "w", encoding="utf-8", newline="\n") as f:
        f.write(r.stdout)
        f.write(r.stderr)
    if not r.stdout.strip():
        print(r.stderr)
        raise SystemExit("dyne-sim2d failed (exit %d)" % r.returncode)
    checks, setup_bad, final = {}, {}, {}
    for line in r.stdout.splitlines():
        m = re.match(r"replay-check node (\d+) step (\d+) (\S+) sim ([0-9a-f]{8}) node ([0-9a-f]{8}) (OK|DIFF)", line)
        if m:
            checks.setdefault(int(m.group(1)), []).append((int(m.group(2)), m.group(3), m.group(4), m.group(5),
                                                           m.group(6) == "OK"))
            continue
        m = re.match(r"replay-setup node (\d+): (.*)", line)
        if m:
            setup_bad.setdefault(int(m.group(1)), []).append(m.group(2))
            continue
        m = re.match(r"replay-node (\d+): .* crc32 ([0-9a-f]{8})", line)
        if m:
            final[int(m.group(1))] = m.group(2)
    single = None
    for line in r.stdout.splitlines():
        if line.startswith("PASS (bit-exact)") or line.startswith("FAIL:"):
            single = line
    nodes = sorted(set(info) | set(checks) | set(final))
    print("replay of %s (%d nodes) with %s" % (a.log, len(nodes), os.path.basename(a.wasm)))
    verdict = {}
    for n in nodes:
        cs = checks.get(n, [])
        bad = [c for c in cs if not c[4]]
        i = info.get(n, {"range": 0, "imu": 0, "setup": False, "warn": [], "lost": []})
        why = ""
        if i["lost"]:
            v = "INCOMPLETE"
        elif cs and not bad and n not in setup_bad:
            v = "PASS"
        else:
            v = "FAIL"
        if not cs:
            why = " (no hash reported by the node)"
        elif bad:
            s0 = min(c[0] for c in bad)
            first = [c for c in bad if c[0] == s0][0]
            why = " (first differing step %d: sim %s node %s)" % (s0, first[2], first[3])
        if n in setup_bad:
            why += " (setup differs: %s)" % "; ".join(setup_bad[n])
        verdict[n] = v
        print("node %2d: %-10s checks %d/%d  range %d  imu %d  setup %s%s"
              % (n, v, len(cs) - len(bad), len(cs), i["range"], i["imu"],
                 "recorded" if i["setup"] else "simulator", why))
        for w in i["lost"]:
            print("         incomplete: " + w)
        for w in i["warn"]:
            print("         warning: " + w)
    if single is not None:
        print("single-node comparison: " + single)
    incomplete = [n for n in nodes if verdict[n] == "INCOMPLETE"]
    if not nodes:
        print("FAIL (no node in the record)")
        code = 1
    elif incomplete:
        print("INCOMPLETE: records of node(s) %s are missing; differences on other nodes may follow from that"
              % ", ".join(str(n) for n in incomplete))
        code = 2
    elif all(v == "PASS" for v in verdict.values()) and (single is None or single.startswith("PASS")):
        print("ALL PASS (bit-exact with the nodes)")
        code = 0
    else:
        print("FAIL")
        code = 1
    print("(replay input %s, simulator log %s)" % (rp, os.path.join(out_dir, "sim.log")))
    sys.exit(code)


def cmd_record(a):
    import paho.mqtt.client as mqtt
    f = open(a.out, "a", encoding="utf-8", newline="\n")
    n = [0]

    def on_message(cl, ud, msg):
        try:
            p = json.loads(msg.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return  # 形の区画のメッセージ (バイナリ) などは記録しない
        f.write(json.dumps({"t": time.time(), "topic": msg.topic, "payload": p}, separators=(",", ":")) + "\n")
        n[0] += 1

    cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="dyne-hw-replay-%d" % os.getpid())
    cl.on_message = on_message
    cl.connect(a.broker, a.port, 60)
    cl.subscribe(a.topic, qos=1)
    cl.loop_start()
    t0 = time.time()
    print("recording %s -> %s (Ctrl+C to stop)" % (a.topic, a.out), flush=True)
    try:
        while a.duration <= 0 or time.time() - t0 < a.duration:
            time.sleep(1.0)
            f.flush()
    except KeyboardInterrupt:
        pass
    cl.loop_stop()
    cl.disconnect()
    f.close()
    print("%d messages" % n[0])


def flip_bit(v, bit):
    """f64 の1ビットを反転する (bit 0 = 仮数の最下位)。最下位のビット (1 ulp) の差は、アクターの計算の丸めに
    吸収されて状態に現れないことがある (2026-11-15 の確認) ので、既定は仮数のビット 40 (相対で約 2^-12)"""
    b = struct.unpack("<Q", struct.pack("<d", float(v)))[0] ^ (1 << bit)
    return struct.unpack("<d", struct.pack("<Q", b))[0]


def cmd_corrupt(a):
    out, done = [], False
    with open(a.log, encoding="utf-8") as f:
        for line in f:
            r = json.loads(line) if line.strip() else None
            m = TOPIC_RE.match(r.get("topic", "")) if r else None
            if not done and m and int(m.group(1)) == a.node and m.group(2) == a.kind + "_fed" \
                    and int(r["payload"].get("step", -1)) >= a.step:
                p = r["payload"]
                if a.kind == "range":
                    if "d_m" in p:
                        p["d_m"][0] = flip_bit(p["d_m"][0], a.bit)
                    else:
                        p["d_mm"][0] += 1  # 実機の記録は mm の整数: 1 mm だけ変える
                else:
                    p["s"][0][1] = flip_bit(p["s"][0][1], a.bit)
                print("corrupted %s at step %d (node %d, bit %d)" % (r["topic"], p["step"], a.node, a.bit))
                done = True
                line = json.dumps(r, separators=(",", ":")) + "\n"
            out.append(line)
    if not done:
        raise SystemExit("no %s_fed record of node %d at step >= %d" % (a.kind, a.node, a.step))
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        f.writelines(out)


def cmd_synth_imu(a):
    """IMU の融合のアクター向けの合成の記録。標本はノードごとに 104 Hz、時刻 t の標本は t < t0 + (s+1)·dt の
    最初のステップ s の計算の前に渡す(ファームの feed_imu と同じ考え方)。値は重力 + ノードごとのずれ + 雑音"""
    rng = random.Random(a.seed)
    t0 = 1_763_020_800_000_000
    dt_us = int(round(a.dt * 1e6))
    per = 1e6 / 104.0
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        def put(topic, p):
            f.write(json.dumps({"t": 0.0, "topic": topic, "payload": p}, separators=(",", ":")) + "\n")
        for n in range(a.nodes):
            put("dyne/node/%d/actor_setup" % n,
                {"idx": n, "id": n, "cfg": [[0, a.epoch], [2, t0], [6, 1]], "faces": [],
                 "init": {"nx": 1, "ny": 1, "ix0": 0, "iy0": 0, "h": 1.0, "alpha": 1.0, "dt": a.dt}, "trunc": False})
        phase = [rng.random() * per for _ in range(a.nodes)]
        bias = [[rng.gauss(0, 0.05) for _ in range(6)] for _ in range(a.nodes)]
        k = [0] * a.nodes
        for s in range(a.steps):
            t_end = t0 + (s + 1) * dt_us
            for n in range(a.nodes):
                smp = []
                while True:
                    t = int(t0 - 0.5 * per + phase[n] + k[n] * per)
                    if t >= t_end:
                        break
                    v = [bias[n][0] + rng.gauss(0, 0.02), bias[n][1] + rng.gauss(0, 0.02),
                         9.80665 + bias[n][2] + rng.gauss(0, 0.02)] + [bias[n][3 + j] * 0.1 + rng.gauss(0, 0.002)
                                                                     for j in range(3)]
                    smp.append([t] + v)
                    k[n] += 1
                if smp:
                    put("dyne/node/%d/imu_fed" % n, {"step": s, "i0": 0, "s": smp})
    print("wrote %s (%d nodes, %d steps)" % (a.out, a.nodes, a.steps))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("record")
    p.add_argument("--broker", default="127.0.0.1")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--topic", default="dyne/#")
    p.add_argument("--out", required=True)
    p.add_argument("--duration", type=float, default=0.0, help="秒(0 = Ctrl+C まで)")
    p.set_defaults(func=cmd_record)
    for name in ("convert", "check"):
        p = sub.add_parser(name)
        p.add_argument("log")
        p.add_argument("--map", action="append", help="MQTT の <id>:node.idx(status から分からないとき)")
        p.add_argument("--since", type=float, default=0.0, help="この PC の時刻 [s] より後の記録だけ使う")
        if name == "convert":
            p.add_argument("--out", required=True)
            p.add_argument("--steps", type=int, help="実行のステップ数(done の CRC を最後のステップに当てる)")
            p.set_defaults(func=cmd_convert)
        else:
            p.add_argument("--wasm", required=True, help="ノードと同じアクターの .wasm")
            p.add_argument("--sim", help="dyne-sim2d の実行ファイル(既定 target/release/dyne-sim2d)")
            p.add_argument("--out", help="再生の入力とシミュレータの記録の置き場(既定 <記録>_replay/)")
            p.add_argument("--single", action="store_true",
                           help="単一ノード解との比較もする(dyne-sim2d --replay-single。既定はしない)")
            p.set_defaults(func=cmd_check)
    p = sub.add_parser("corrupt")
    p.add_argument("log")
    p.add_argument("out")
    p.add_argument("--node", type=int, default=0)
    p.add_argument("--kind", choices=["range", "imu"], default="range")
    p.add_argument("--step", type=int, default=0, help="このステップ以降の最初の記録を変える")
    p.add_argument("--bit", type=int, default=40, help="反転する f64 のビット (0 = 仮数の最下位。既定 40)")
    p.set_defaults(func=cmd_corrupt)
    p = sub.add_parser("synth-imu")
    p.add_argument("out")
    p.add_argument("--nodes", type=int, default=4)
    p.add_argument("--steps", type=int, default=200)
    p.add_argument("--dt", type=float, default=0.02)
    p.add_argument("--epoch", type=int, default=10, help="CFG_EPOCH_STEPS")
    p.add_argument("--seed", type=int, default=1)
    p.set_defaults(func=cmd_synth_imu)
    # "--" の後ろは dyne-sim2d の引数 (argparse の REMAINDER は位置引数の後の選択肢まで飲み込むので、先に分ける)
    argv = sys.argv[1:]
    sim_args = argv[argv.index("--") + 1:] if "--" in argv else []
    argv = argv[:argv.index("--")] if "--" in argv else argv
    a = ap.parse_args(argv)
    a.sim_args = sim_args
    a.func(a)


if __name__ == "__main__":
    main()

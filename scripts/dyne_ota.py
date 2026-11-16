#!/usr/bin/env python3
"""ノードの一斉 OTA (無線経由のファームウェア・計算部品の書き換え) の PC 側ツール (2026-11-11)。

PC で HTTP サーバを立ててファイルを配り、MQTT の ctl 命令で N 台に一斉に書き換えを指示し、
ノードごとの結果 (成功 / 元に戻った / 拒否 / 失敗 / 実行中で断った / 時間切れ) と所要時間を CSV に記録する。
説明: docs/2026-11-11-timesync-ota.md (ノード側の命令と結果の形もそこに)

必要パッケージ: paho-mqtt (既存の scripts/mqtt_config.py と同じ。2026-11-16 から、無ければ標準ライブラリだけの
scripts/dyne_mqtt_lite.py を使う)。それ以外は標準ライブラリ。

使い方 (リポジトリのルートで。--broker は PC の LAN 側アドレス):
  ノードの一覧:
    python scripts\\dyne_ota.py status --broker 192.168.10.2
  ファームを全台へ (対象チップが .bin と違うノードは飛ばす):
    python scripts\\dyne_ota.py fw --broker 192.168.10.2 --bin node-esp32\\build-ota-esp32c5\\dyne_node.bin --nodes all --csv ota_results.csv
  計算部品を全台へ (.aot はチップごとに指定。無ければ .wasm だけ):
    python scripts\\dyne_ota.py actor --broker 192.168.10.2 --wasm node-esp32\\main\\heat1d_actor_small.wasm --name heat1d_actor_small --nodes all
  わざと壊したものを配り、元に戻ることを確かめる:
    ... fw --broken selftest-fail   像としては正しいが、起動後の自己確認に失敗する → rolled_back を期待
    ... fw --broken corrupt         像の SHA-256 が合わない → 書き換え前に rejected を期待
    ... actor --broken garbage      SHA-256 は合うが中身が WASM でない → 積み込みに失敗し fallback を期待
    ... actor --broken badsha       目録の SHA-256 が中身と違う → rejected を期待
  前の計算部品に戻す:
    python scripts\\dyne_ota.py actor-rollback --broker ... --nodes 3,5
  .bin の中身の確認・壊した像の作成だけ (ノード不要):
    python scripts\\dyne_ota.py info --bin X.bin
    python scripts\\dyne_ota.py make-broken --bin X.bin --mode selftest-fail --out Y.bin
"""
import argparse
import csv
import functools
import hashlib
import http.server
import json
import os
import shutil
import socket
import sys
import tempfile
import threading
import time

CHIP_NAMES = {0: "esp32", 2: "esp32s2", 5: "esp32c3", 9: "esp32s3", 12: "esp32c2", 13: "esp32c6",
              16: "esp32h2", 18: "esp32p4", 23: "esp32c5", 20: "esp32c61"}
SELFTEST_MARK = b"DYNE_SELFTEST_MARK:pass"
SELFTEST_FAIL = b"DYNE_SELFTEST_MARK:FAIL"
FINAL_FW = {"ok", "rolled_back", "rejected", "fail", "busy"}
FINAL_ACTOR = {"ok", "fallback", "rejected", "fail", "busy"}


# ==== ESP-IDF のアプリ像 (.bin) ====

def parse_image(b):
    """ESP-IDF のアプリ像を解釈する。返り値: dict。並び:
    ヘッダ 24 バイト (0xE9, 区切りの数, …, chip_id は 12〜13, hash_appended は 23) → 区切りごとに
    (読み込み先 4, 長さ 4, 中身) → 16 バイト境界の最後の1バイトに検査和 (0xEF と全区切りの中身の XOR) →
    hash_appended=1 なら直前までの SHA-256 (32 バイト)。最初の区切りの先頭がアプリの記述 (esp_app_desc_t)"""
    if len(b) < 24 or b[0] != 0xE9:
        raise ValueError("ESP-IDF のアプリ像ではない (先頭が 0xE9 でない)")
    nseg = b[1]
    chip = int.from_bytes(b[12:14], "little")
    hash_app = b[23]
    off = 24
    segs = []
    for _ in range(nseg):
        load, ln = int.from_bytes(b[off:off + 4], "little"), int.from_bytes(b[off + 4:off + 8], "little")
        segs.append((off + 8, ln, load))
        off += 8 + ln
    csum_off = (off // 16) * 16 + 15
    end = csum_off + 1 + (32 if hash_app == 1 else 0)
    desc = {}
    d0 = segs[0][0] if segs else 0
    if segs and int.from_bytes(b[d0:d0 + 4], "little") == 0xABCD5432:
        cstr = lambda s: s.split(b"\0", 1)[0].decode(errors="replace")
        desc = {"version": cstr(b[d0 + 16:d0 + 48]), "project": cstr(b[d0 + 48:d0 + 80]),
                "time": cstr(b[d0 + 80:d0 + 96]), "date": cstr(b[d0 + 96:d0 + 112]),
                "idf": cstr(b[d0 + 112:d0 + 144]), "elf_sha256": b[d0 + 144:d0 + 176].hex()}
    return {"chip_id": chip, "chip": CHIP_NAMES.get(chip, f"chip{chip}"), "segments": segs,
            "csum_off": csum_off, "hash_appended": hash_app == 1, "image_len": end, "file_len": len(b),
            "desc": desc}


def calc_checksum(b, segs):
    c = 0xEF
    for (o, ln, _) in segs:
        for x in b[o:o + ln]:
            c ^= x
    return c


def verify_image(b):
    """検査和と付属の SHA-256 を確かめる。返り値: (ok, 説明)"""
    im = parse_image(b)
    if im["image_len"] > len(b):
        return False, "短すぎる"
    if calc_checksum(b, im["segments"]) != b[im["csum_off"]]:
        return False, "検査和が合わない"
    if im["hash_appended"]:
        h = hashlib.sha256(b[:im["csum_off"] + 1]).digest()
        if h != b[im["csum_off"] + 1:im["csum_off"] + 33]:
            return False, "付属の SHA-256 が合わない"
    return True, "ok"


def img_sha256(b):
    """ノードが esp_partition_get_sha256 で返す値 = 付属の SHA-256 (像の末尾 32 バイト)"""
    im = parse_image(b)
    if not im["hash_appended"]:
        raise ValueError("SHA-256 が付属していない像")
    return b[im["csum_off"] + 1:im["csum_off"] + 33].hex()


def make_broken(b, mode):
    """mode = selftest-fail: 目印を "FAIL" に書き換え、検査和と SHA-256 を付け直す (像としては正しい)。
    mode = corrupt: 中身の1バイトを反転し、付け直さない (書き換えの前に検査で拒否される)"""
    b = bytearray(b)
    im = parse_image(bytes(b))
    if mode == "selftest-fail":
        n = bytes(b).count(SELFTEST_MARK)
        if n != 1:
            raise ValueError(f"目印 {SELFTEST_MARK!r} が {n} 回 (1回であるべき)。目印を持たない古いファーム?")
        i = bytes(b).index(SELFTEST_MARK)
        if not any(o <= i < o + ln for (o, ln, _) in im["segments"]):
            raise ValueError("目印が区切りの中にない")
        b[i:i + len(SELFTEST_FAIL)] = SELFTEST_FAIL
        b[im["csum_off"]] = calc_checksum(b, im["segments"])
        if im["hash_appended"]:
            b[im["csum_off"] + 1:im["csum_off"] + 33] = hashlib.sha256(bytes(b[:im["csum_off"] + 1])).digest()
    elif mode == "corrupt":
        o, ln, _ = max(im["segments"], key=lambda s: s[1])
        b[o + ln // 2] ^= 0xFF
    else:
        raise ValueError(mode)
    return bytes(b)


# ==== HTTP サーバ ====

class LoggingHandler(http.server.SimpleHTTPRequestHandler):
    log = None

    def log_message(self, fmt, *args):
        if LoggingHandler.log is not None:
            LoggingHandler.log.append((time.time(), self.client_address[0], fmt % args))


def start_http(root, host, port):
    LoggingHandler.log = []
    handler = functools.partial(LoggingHandler, directory=root)
    srv = http.server.ThreadingHTTPServer((host, port), handler)
    srv.daemon_threads = True
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    return srv


def guess_host(broker):
    """ノードから見た PC のアドレス = ブローカーへの経路の送信元アドレス"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((broker, 1883))
        return s.getsockname()[0]
    finally:
        s.close()


# ==== MQTT ====

class Fleet:
    """ノードの status と ota の結果を集める"""

    def __init__(self, broker, port=1883):
        # 必要なときだけ読み込む (info / make-broken は不要)。2026-11-16 (issue #4): paho-mqtt が無ければ
        # 標準ライブラリだけの scripts/dyne_mqtt_lite.py (CI で偽のノードに対して動かすため)
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import dyne_mqtt_lite
        mqtt = dyne_mqtt_lite.pick("auto")
        self.mqtt = mqtt
        self.status = {}   # id -> (受信時刻, dict)
        self.events = []   # (受信時刻, id, dict)
        self.lock = threading.Lock()
        self.cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"dyne-ota-{os.getpid()}")
        self.cl.on_message = self._on_msg
        self.cl.on_connect = lambda c, u, f, rc, p=None: c.subscribe(
            [("dyne/node/+/status", 1), ("dyne/node/+/ota", 1)])  # 再接続でも購読し直す
        self.cl.connect(broker, port, 60)
        self.cl.loop_start()

    def _on_msg(self, cl, u, m):
        parts = m.topic.split("/")
        if len(parts) != 4:
            return
        try:
            nid = int(parts[2])
            d = json.loads(m.payload) if m.payload else None
        except (ValueError, json.JSONDecodeError):
            return
        now = time.time()
        with self.lock:
            if parts[3] == "status":
                if d is None:
                    self.status.pop(nid, None)
                else:
                    prev = self.status.get(nid, (0, {}))[1]
                    merged = dict(prev) if d.get("state") != "online" else {}
                    merged.update(d)
                    self.status[nid] = (now, merged)
                    self.events.append((now, nid, {"status": d}))
            elif parts[3] == "ota" and d is not None:
                self.events.append((now, nid, d))

    def wait_retained(self, sec=1.5):
        time.sleep(sec)

    def online_nodes(self):
        with self.lock:
            return {nid: d for nid, (_, d) in self.status.items() if d.get("state") != "offline"}

    def send(self, nid, cmd):
        self.cl.publish(f"dyne/ctl/{nid}", json.dumps(cmd), qos=1, retain=False)

    def close(self):
        self.cl.loop_stop()
        self.cl.disconnect()


def select_nodes(fleet, spec):
    nodes = fleet.online_nodes()
    if spec == "all":
        return sorted(nodes)
    want = [int(x) for x in spec.split(",") if x.strip()]
    missing = [n for n in want if n not in nodes]
    if missing:
        print(f"注意: status が online でないノード {missing} にも送る (届かなければ時間切れになる)")
    return want


def print_status(fleet):
    nodes = fleet.online_nodes()
    if not nodes:
        print("online のノードなし")
    print(f"{'id':>4} {'state':<11} {'target':<8} {'part':<6} {'ver':<18} {'elf':<17} {'actor':<28} {'reset':<9} {'boot':>4} {'up_s':>6}")
    for nid in sorted(nodes):
        d = nodes[nid]
        a = d.get("actor") or {}
        act = f"{a.get('src', '')} #{a.get('seq', '')} {a.get('name', '')}" if a else ""
        print(f"{nid:>4} {d.get('state', ''):<11} {d.get('target', '?'):<8} {d.get('part', '?'):<6} "
              f"{str(d.get('ver', '?'))[:18]:<18} {str(d.get('elf', '?')):<17} {act[:28]:<28} "
              f"{d.get('reset', '?'):<9} {d.get('boot', ''):>4} {d.get('up_s', ''):>6}")


# ==== 一斉配布と結果の集計 ====

CSV_FIELDS = ["job", "kind", "variant", "node", "target", "result", "expect", "match", "t_total_s",
              "t_download_ms", "bytes", "err", "part_before", "part_after", "elf_before", "elf_after",
              "actor_after", "phases"]


def run_job(fleet, args, job, kind, variant, expect, cmds, before):
    """cmds: {node: 命令 dict}。結果を集めて CSV に書く"""
    finals = FINAL_FW if kind == "fw" else FINAL_ACTOR
    t_send = {}
    ev0 = len(fleet.events)
    for nid, cmd in cmds.items():
        t_send[nid] = time.time()
        fleet.send(nid, cmd)
        if args.stagger_ms > 0:
            time.sleep(args.stagger_ms / 1000)
    print(f"[{job}] {len(cmds)} 台へ送信。結果を待つ (最大 {args.timeout:.0f} 秒)")
    res = {nid: {"phases": [], "final": None} for nid in cmds}
    t0 = time.time()
    deadline = t0 + args.timeout
    seen = ev0
    last_done = 0
    while time.time() < deadline and any(r["final"] is None for r in res.values()):
        time.sleep(0.2)
        with fleet.lock:
            new = fleet.events[seen:]
            seen = len(fleet.events)
        for (t, nid, d) in new:
            if nid not in res or "status" in d:
                continue
            if d.get("job") != job:
                continue
            r = res[nid]
            r["phases"].append(f"{d.get('phase')}@{t - t_send[nid]:.1f}")
            ph = d.get("phase")
            if ph == "downloaded":
                r["dl"] = d
                if not cmds[nid].get("reboot", True):
                    r["final"], r["t_final"] = "downloaded", t
            elif ph == "selftest_fail":
                r["selftest"] = d
            elif ph in finals:
                r["final"], r["t_final"], r["last"] = ph, t, d
                if ph in ("rejected", "fail", "busy"):
                    r["dl"] = d
        done = sum(1 for r in res.values() if r["final"])
        if done != last_done:
            print(f"  完了 {done}/{len(res)} ({time.time() - t0:.1f} 秒)", flush=True)
            last_done = done
    time.sleep(1.0)  # 再起動後の online の status を拾う
    after = fleet.online_nodes()
    rows = []
    for nid in sorted(res):
        r = res[nid]
        fin = r["final"] or "timeout"
        dl = r.get("dl", {})
        a = after.get(nid, {})
        b = before.get(nid, {})
        act = a.get("actor") or {}
        err = dl.get("err", "") or (r.get("last") or {}).get("err", "")
        row = {"job": job, "kind": kind, "variant": variant, "node": nid, "target": b.get("target", "?"),
               "result": fin, "expect": expect, "match": "yes" if fin == expect else "NO",
               "t_total_s": f"{r['t_final'] - t_send[nid]:.2f}" if r.get("t_final") else "",
               "t_download_ms": dl.get("ms", ""), "bytes": dl.get("bytes", ""), "err": err,
               "part_before": b.get("part", ""), "part_after": a.get("part", ""),
               "elf_before": b.get("elf", ""), "elf_after": a.get("elf", ""),
               "actor_after": f"{act.get('src', '')}#{act.get('seq', '')}" if act else "",
               "phases": " ".join(r["phases"])}
        rows.append(row)
    print(f"{'node':>4} {'result':<12} {'expect':<12} {'total_s':>8} {'dl_ms':>7} {'bytes':>8}  err")
    for row in rows:
        print(f"{row['node']:>4} {row['result']:<12} {row['expect']:<12} {row['t_total_s']:>8} "
              f"{str(row['t_download_ms']):>7} {str(row['bytes']):>8}  {row['err']}")
    ok = sum(1 for row in rows if row["match"] == "yes")
    print(f"期待どおり {ok}/{len(rows)} 台")
    if args.csv:
        new = not os.path.exists(args.csv)
        with open(args.csv, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if new:
                w.writeheader()
            w.writerows(rows)
        print(f"記録: {args.csv}")
    if LoggingHandler.log:
        print(f"HTTP の要求 {len(LoggingHandler.log)} 件")
    return 0 if ok == len(rows) else 1


def cmd_fw(fleet, args):
    data = open(args.bin, "rb").read()
    ok, why = verify_image(data)
    if not ok:
        print(f"{args.bin}: 像の検査に失敗 ({why})")
        return 2
    im = parse_image(data)
    sha = img_sha256(data)
    variant, expect = "normal", "ok"
    if args.broken:
        data = make_broken(data, args.broken)
        variant = args.broken
        if args.broken == "selftest-fail":
            sha = img_sha256(data)  # 付け直した SHA-256 (ノードは像としては受け入れる)
            expect = "rolled_back"
        else:
            expect = "rejected"
    if args.no_reboot:
        expect = "downloaded" if expect == "ok" else expect
    job = args.job or time.strftime("fw-%m%d-%H%M%S")
    root = tempfile.mkdtemp(prefix="dyne_ota_")
    fname = f"{job}.bin"
    with open(os.path.join(root, fname), "wb") as f:
        f.write(data)
    host = args.http_host or guess_host(args.broker)
    srv = start_http(root, args.http_bind, args.http_port)
    url = f"http://{host}:{args.http_port}/{fname}"
    print(f"像: {args.bin} ({len(data)} バイト, {im['chip']}, 版 {im['desc'].get('version')}, "
          f"elf {im['desc'].get('elf_sha256', '')[:16]}) 変種={variant}\n配布: {url}")
    try:
        fleet.wait_retained()
        nodes = select_nodes(fleet, args.nodes)
        before = fleet.online_nodes()
        cmds = {}
        for nid in nodes:
            tgt = before.get(nid, {}).get("target")
            if tgt and tgt != im["chip"] and not args.ignore_target:
                print(f"  ノード {nid}: 対象チップが {tgt} で像 ({im['chip']}) と違うので飛ばす")
                continue
            cmds[nid] = {"cmd": "ota_fw", "job": job, "url": url, "size": len(data), "img_sha256": sha,
                         "reboot": not args.no_reboot}
        if not cmds:
            print("送り先なし")
            return 1
        return run_job(fleet, args, job, "fw", variant, expect, cmds, before)
    finally:
        srv.shutdown()
        shutil.rmtree(root, ignore_errors=True)


def cmd_actor(fleet, args):
    wasm = open(args.wasm, "rb").read() if args.wasm else None
    aots = {}
    for spec in args.aot or []:
        tgt, path = spec.split("=", 1)
        aots[tgt] = open(path, "rb").read()
    if wasm is None and not aots:
        print("--wasm か --aot が必要")
        return 2
    variant, expect = "normal", "ok"
    job = args.job or time.strftime("act-%m%d-%H%M%S")
    root = tempfile.mkdtemp(prefix="dyne_ota_")
    files = {}

    def put(name, data, sha=None):
        with open(os.path.join(root, name), "wb") as f:
            f.write(data)
        files[name] = {"size": len(data), "sha256": sha or hashlib.sha256(data).hexdigest()}
        return name

    if args.broken == "garbage":
        variant, expect = "garbage", "fallback"
        rnd = os.urandom(len(wasm) if wasm else 4096)
        wasm, aots = rnd, {}
    if wasm is not None:
        sha = None
        if args.broken == "badsha":
            variant, expect = "badsha", "rejected"
            sha = hashlib.sha256(wasm + b"x").hexdigest()
        put(f"{job}.wasm", wasm, sha)
    for tgt, data in aots.items():
        put(f"{job}.{tgt}.aot", data)
    host = args.http_host or guess_host(args.broker)
    srv = start_http(root, args.http_bind, args.http_port)
    base = f"http://{host}:{args.http_port}/"
    print(f"計算部品 {args.name}: " + ", ".join(f"{k} ({v['size']} バイト)" for k, v in files.items())
          + f" 変種={variant}\n配布: {base}")
    try:
        fleet.wait_retained()
        nodes = select_nodes(fleet, args.nodes)
        before = fleet.online_nodes()
        cmds = {}
        for nid in nodes:
            tgt = before.get(nid, {}).get("target")
            cmd = {"cmd": "ota_actor", "job": job, "name": args.name, "reboot": not args.no_reboot}
            wn = f"{job}.wasm"
            if wn in files:
                cmd["wasm"] = {"url": base + wn, **files[wn]}
            an = f"{job}.{tgt}.aot"
            if an in files:
                cmd["aot"] = {"url": base + an, **files[an]}
            if "wasm" not in cmd and "aot" not in cmd:
                print(f"  ノード {nid}: 対象チップ {tgt} 向けのファイルがないので飛ばす")
                continue
            cmds[nid] = cmd
        if not cmds:
            print("送り先なし")
            return 1
        if args.no_reboot and expect == "ok":
            expect = "downloaded"
        return run_job(fleet, args, job, "actor", variant, expect, cmds, before)
    finally:
        srv.shutdown()
        shutil.rmtree(root, ignore_errors=True)


def cmd_actor_rollback(fleet, args):
    job = args.job or time.strftime("rb-%m%d-%H%M%S")
    fleet.wait_retained()
    nodes = select_nodes(fleet, args.nodes)
    before = fleet.online_nodes()
    cmds = {nid: {"cmd": "actor_rollback", "job": job} for nid in nodes}
    return run_job(fleet, args, job, "actor", "rollback", "ok", cmds, before)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--broker", required=True, help="MQTT ブローカー (PC) のアドレス")
        p.add_argument("--mqtt-port", type=int, default=1883)
        p.add_argument("--nodes", default="all", help="all または 1,2,3")
        p.add_argument("--http-host", help="ノードから見た PC のアドレス (既定: ブローカーへの経路から推定)")
        p.add_argument("--http-bind", default="0.0.0.0")
        p.add_argument("--http-port", type=int, default=8070)
        p.add_argument("--timeout", type=float, default=240.0, help="結果を待つ上限 [s]")
        p.add_argument("--stagger-ms", type=float, default=0.0, help="ノードごとの送信の間隔 [ms]")
        p.add_argument("--csv", help="結果を追記する CSV")
        p.add_argument("--job", help="ジョブ名 (既定: 時刻から作る)")
        p.add_argument("--no-reboot", action="store_true", help="書き込みまで (再起動しない)")

    p = sub.add_parser("status", help="ノードの一覧")
    p.add_argument("--broker", required=True)
    p.add_argument("--mqtt-port", type=int, default=1883)
    p = sub.add_parser("fw", help="ファームの一斉書き換え")
    common(p)
    p.add_argument("--bin", required=True)
    p.add_argument("--broken", choices=["selftest-fail", "corrupt"])
    p.add_argument("--ignore-target", action="store_true", help="対象チップの照合をしない")
    p = sub.add_parser("actor", help="計算部品の一斉書き換え")
    common(p)
    p.add_argument("--wasm")
    p.add_argument("--aot", action="append", help="チップ=パス (例 esp32c5=x.aot)。複数可")
    p.add_argument("--name", default="actor")
    p.add_argument("--broken", choices=["garbage", "badsha"])
    p = sub.add_parser("actor-rollback", help="前の計算部品に戻す")
    common(p)
    p = sub.add_parser("info", help=".bin の中身の確認")
    p.add_argument("--bin", required=True)
    p = sub.add_parser("make-broken", help="壊した像を作る")
    p.add_argument("--bin", required=True)
    p.add_argument("--mode", required=True, choices=["selftest-fail", "corrupt"])
    p.add_argument("--out", required=True)
    args = ap.parse_args()

    if args.cmd == "info":
        b = open(args.bin, "rb").read()
        im = parse_image(b)
        ok, why = verify_image(b)
        print(json.dumps({k: v for k, v in im.items() if k != "segments"}, ensure_ascii=False, indent=1))
        print(f"区切り {len(im['segments'])} 個、検査: {why}、img_sha256 = {img_sha256(b) if im['hash_appended'] else '-'}")
        print(f"自己確認の目印: {b.count(SELFTEST_MARK)} 回")
        return 0 if ok else 1
    if args.cmd == "make-broken":
        b = make_broken(open(args.bin, "rb").read(), args.mode)
        open(args.out, "wb").write(b)
        ok, why = verify_image(b)
        print(f"{args.out}: {len(b)} バイト、検査: {why}")
        return 0
    fleet = Fleet(args.broker, args.mqtt_port)
    try:
        if args.cmd == "status":
            fleet.wait_retained()
            print_status(fleet)
            return 0
        if args.cmd == "fw":
            return cmd_fw(fleet, args)
        if args.cmd == "actor":
            return cmd_actor(fleet, args)
        if args.cmd == "actor-rollback":
            return cmd_actor_rollback(fleet, args)
    finally:
        fleet.close()


if __name__ == "__main__":
    sys.exit(main())

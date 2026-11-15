#!/usr/bin/env python3
"""OTA の PC 側ツール (scripts/dyne_ota.py) を実機なしで確かめるための、ノードのふりをするプログラム (2026-11-11)。

ファーム (main/dyne_ops.c) と同じ MQTT の命令・結果の形で応答する。判定もファームと同じ規則で、
ただし PC 側ツールのコードは使わず独立に書いている (同じ誤りを共有しないため):
  ota_fw    : HTTP で取得 → 長さ・像の付属 SHA-256 (末尾32バイト = 直前までの SHA-256) を確かめる →
              合わなければ rejected → 合えば downloaded → 再起動のふり → 目印が "FAIL" なら
              selftest_fail のあと rolled_back、そうでなければ ok (起動区画が入れ替わる)
  ota_actor : HTTP で取得 → 長さと SHA-256 を目録と照合 (違えば rejected) → downloaded → 再起動のふり →
              中身が WASM (先頭が \\0asm) なら ok、そうでなければ fallback
  計算の実行中のふり (--busy で指定したノード) は busy を返す。

使い方: python node-esp32\\host_test\\fake_node.py --broker 127.0.0.1 --ids 901,902,903 [--target esp32] [--busy 903]
終了時 (Ctrl+C か --lifetime 秒後) に、自分の貼った retained の status を消す。
"""
import argparse
import hashlib
import json
import os
import threading
import time
import urllib.request

import paho.mqtt.client as mqtt

MARK_FAIL = b"DYNE_SELFTEST_MARK:FAIL"


class FakeNode:
    def __init__(self, cl, nid, target, busy):
        self.cl, self.id, self.target, self.busy = cl, nid, target, busy
        self.part = "ota_0"
        self.boot = 1
        self.actor = {"src": "embedded", "seq": 0, "name": "heat1d_actor_small"}
        self.seq = 0
        self.t0 = time.time()

    def pub(self, sub, d, retain=False):
        self.cl.publish(f"dyne/node/{self.id}/{sub}", json.dumps(d), qos=1, retain=retain)

    def online(self):
        self.pub("status", {"state": "online", "ip": "127.0.0.1", "fw": "fake", "target": self.target,
                            "ver": "fake", "elf": "fake" + self.part[-1] * 12, "part": self.part,
                            "ota_state": "valid", "reset": "sw", "boot": self.boot,
                            "up_s": int(time.time() - self.t0), "actor": self.actor}, retain=True)

    def ota(self, job, kind, phase, **kw):
        self.pub("ota", {"job": job, "kind": kind, "phase": phase, **kw})

    def reboot(self):
        self.pub("status", {"state": "offline", "reason": "ota"}, retain=True)
        time.sleep(0.8)
        self.boot += 1
        self.t0 = time.time()

    def on_ctl(self, d):
        cmd = d.get("cmd")
        job = d.get("job", "")
        if cmd not in ("ota_fw", "ota_actor", "actor_rollback"):
            return
        kind = "fw" if cmd == "ota_fw" else "actor"
        if self.busy:
            self.ota(job, kind, "busy", err="running")
            return
        threading.Thread(target=self.fw if cmd == "ota_fw" else self.act, args=(d,), daemon=True).start()

    def fetch(self, url):
        with urllib.request.urlopen(url, timeout=15) as r:
            return r.read()

    def fw(self, d):
        job = d["job"]
        self.ota(job, "fw", "accepted")
        t0 = time.time()
        try:
            b = self.fetch(d["url"])
        except Exception as e:
            self.ota(job, "fw", "fail", err=f"download {e}")
            return
        ms = int((time.time() - t0) * 1000)
        if d.get("size") and len(b) != d["size"]:
            self.ota(job, "fw", "fail", err="size mismatch", ms=ms, bytes=len(b))
            return
        body, tail = b[:-32], b[-32:]
        if hashlib.sha256(body).digest() != tail:
            self.ota(job, "fw", "rejected", err="image invalid", ms=ms, bytes=len(b))
            return
        if d.get("img_sha256") and d["img_sha256"] != tail.hex():
            self.ota(job, "fw", "rejected", err="img_sha256 mismatch", ms=ms, bytes=len(b))
            return
        target = "ota_1" if self.part == "ota_0" else "ota_0"
        self.ota(job, "fw", "downloaded", ms=ms, bytes=len(b), from_part=self.part, to_part=target)
        if not d.get("reboot", True):
            return
        self.reboot()
        if MARK_FAIL in b:
            # 新しいファームで起動 → 自己確認に失敗 → 元に戻して再起動
            self.ota(job, "fw", "selftest_fail", mark=False)
            time.sleep(0.3)
            self.reboot()
            self.online()
            self.ota(job, "fw", "rolled_back", part=self.part, target_part=target)
        else:
            self.part = target
            self.online()
            self.ota(job, "fw", "ok", part=self.part, target_part=target)

    def act(self, d):
        job = d["job"]
        self.ota(job, "actor", "accepted")
        if d["cmd"] == "actor_rollback":
            self.ota(job, "actor", "downloaded", erased_slot=0)
            self.reboot()
            self.actor = {"src": "embedded", "seq": 0, "name": "heat1d_actor_small"}
            self.online()
            self.ota(job, "actor", "fallback", actor=self.actor)
            return
        t0 = time.time()
        got = {}
        for k in ("wasm", "aot"):
            if k not in d:
                continue
            m = d[k]
            try:
                b = self.fetch(m["url"])
            except Exception as e:
                self.ota(job, "actor", "fail", err=f"download {e}")
                return
            if len(b) != m["size"]:
                self.ota(job, "actor", "rejected", err="size mismatch")
                return
            if hashlib.sha256(b).hexdigest() != m["sha256"]:
                self.ota(job, "actor", "rejected", err="sha256 mismatch", bytes=len(b))
                return
            got[k] = b
        self.seq += 1
        ms = int((time.time() - t0) * 1000)
        self.ota(job, "actor", "downloaded", ms=ms, bytes=sum(len(x) for x in got.values()), seq=self.seq)
        if not d.get("reboot", True):
            return
        self.reboot()
        if got.get("wasm", b"").startswith(b"\0asm"):
            self.actor = {"src": "slot0.wasm", "seq": self.seq, "name": d.get("name", "")}
            self.online()
            self.ota(job, "actor", "ok", want_seq=self.seq, actor=self.actor)
        else:
            self.online()
            self.ota(job, "actor", "fallback", want_seq=self.seq, actor=self.actor)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default="127.0.0.1")
    ap.add_argument("--ids", default="901,902,903")
    ap.add_argument("--target", default="esp32")
    ap.add_argument("--busy", default="")
    ap.add_argument("--lifetime", type=float, default=0)
    args = ap.parse_args()
    cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"dyne-fake-{os.getpid()}")
    busy = {int(x) for x in args.busy.split(",") if x}
    nodes = {int(x): FakeNode(cl, int(x), args.target, int(x) in busy) for x in args.ids.split(",")}

    def on_msg(c, u, m):
        if m.retain:
            return  # 実機と同じく retain の号令は無視
        try:
            nid = int(m.topic.split("/")[2])
            d = json.loads(m.payload)
        except (ValueError, IndexError, json.JSONDecodeError):
            return
        if nid in nodes:
            nodes[nid].on_ctl(d)

    def on_connect(c, u, flags, rc, props=None):
        # 再接続のたびに購読し直し、online を貼り直す (実機と同じ)
        for nid in nodes:
            c.subscribe(f"dyne/ctl/{nid}", 1)
        for n in nodes.values():
            n.online()

    cl.on_message = on_msg
    cl.on_connect = on_connect
    cl.connect(args.broker, 1883, 60)
    cl.loop_start()
    time.sleep(0.5)
    print(f"fake nodes {sorted(nodes)} online", flush=True)
    try:
        t_end = time.time() + args.lifetime if args.lifetime > 0 else None
        while t_end is None or time.time() < t_end:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for nid in nodes:
            cl.publish(f"dyne/node/{nid}/status", b"", qos=1, retain=True).wait_for_publish(2)
        cl.loop_stop()
        cl.disconnect()


if __name__ == "__main__":
    main()

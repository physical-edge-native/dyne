#!/usr/bin/env python3
"""ノード用の小さな DHCP サーバ(IP アドレスを自動で配る仕組み)。標準ライブラリだけで動く。2026-11-13。

Windows の「インターネット接続の共有」(ICS)が使えないとき・使いたくないときの代わり
(docs/2026-11-13-pc-pipeline-rl.md の「ネットワークの準備」)。PC ⇔ USB-C の有線 LAN の変換器 ⇔
PoE の給電器 ⇔ 無線の親機(TP-Link EAP653)の閉じた網で、ノード(ESP32)に IP アドレスを配る。

安全のため、特定の IP アドレス(USB の有線 LAN の変換器に手で付けたもの)にしか結び付けない。
0.0.0.0(すべての口)は --any を付けたときだけ(家庭の網に誤ってアドレスを配らないように)。
ICS を有効にしていると Windows 自身の DHCP が 67 番の口を使うので、この台本とは同時に使えない。

使い方(管理者の権限は不要。初回は Windows のファイアウォールの許可を求められる):
  1. 変換器の IP アドレスを手で 192.168.50.1 / 255.255.255.0 にする(手順は文書)
  2. .venv\\Scripts\\python.exe scripts\\dyne_dhcp_server.py --bind 192.168.50.1 ^
         --range 192.168.50.100-192.168.50.199 --log demo_out\\dhcp\\leases.csv
  静的な割り当て: --static aa:bb:cc:dd:ee:ff=192.168.50.11(何度でも)
  既定では既定経路(router)も DNS も配らない(閉じた網。MQTT の相手は PC の IP を設定で渡す)。
  配るなら --router 192.168.50.1 --dns 192.168.50.1
  自己検査(網を使わない): --selftest

対応する通信: DISCOVER → OFFER、REQUEST → ACK / NAK、RELEASE、DECLINE、INFORM → ACK(RFC 2131 の最小限)。
貸し出しの記録は --leases の JSON(既定 demo_out/dhcp/leases.json)に残し、再起動しても同じ機械に同じアドレスを配る。
"""
import argparse
import csv
import datetime
import ipaddress
import json
import os
import socket
import struct
import sys
import time

MAGIC = b"\x63\x82\x53\x63"
DISCOVER, OFFER, REQUEST, DECLINE, ACK, NAK, RELEASE, INFORM = 1, 2, 3, 4, 5, 6, 7, 8
NAMES = {1: "DISCOVER", 2: "OFFER", 3: "REQUEST", 4: "DECLINE", 5: "ACK", 6: "NAK", 7: "RELEASE", 8: "INFORM"}


def mac_str(b):
    return ":".join("%02x" % x for x in b)


def parse_packet(data):
    if len(data) < 240 or data[236:240] != MAGIC:
        return None
    op, htype, hlen, hops, xid, secs, flags = struct.unpack("!BBBBIHH", data[:12])
    ciaddr, yiaddr, siaddr, giaddr = (socket.inet_ntoa(data[12 + 4 * k:16 + 4 * k]) for k in range(4))
    chaddr = data[28:28 + 16]
    opts = {}
    i = 240
    while i < len(data):
        code = data[i]
        if code == 255:
            break
        if code == 0:
            i += 1
            continue
        ln = data[i + 1]
        opts[code] = data[i + 2:i + 2 + ln]
        i += 2 + ln
    return {"op": op, "xid": xid, "flags": flags, "ciaddr": ciaddr, "giaddr": giaddr, "chaddr": chaddr,
            "mac": mac_str(chaddr[:hlen if 0 < hlen <= 16 else 6]), "opts": opts,
            "type": opts.get(53, b"\x00")[0], "hostname": opts.get(12, b"").decode("ascii", "replace")}


def build_reply(req, mtype, yiaddr, server_ip, cfg):
    hdr = struct.pack("!BBBBIHH", 2, 1, 6, 0, req["xid"], 0, req["flags"])
    hdr += socket.inet_aton(req["ciaddr"] if mtype == ACK and req["type"] == INFORM else "0.0.0.0")
    hdr += socket.inet_aton(yiaddr or "0.0.0.0")
    hdr += socket.inet_aton(server_ip)
    hdr += socket.inet_aton(req["giaddr"])
    hdr += req["chaddr"].ljust(16, b"\x00")
    hdr += b"\x00" * 192
    o = bytearray(MAGIC)

    def opt(code, val):
        o.extend(bytes([code, len(val)]) + val)
    opt(53, bytes([mtype]))
    opt(54, socket.inet_aton(server_ip))
    if mtype != NAK:
        if req["type"] != INFORM:
            opt(51, struct.pack("!I", cfg["lease"]))
            opt(58, struct.pack("!I", cfg["lease"] // 2))
            opt(59, struct.pack("!I", cfg["lease"] * 7 // 8))
        opt(1, socket.inet_aton(cfg["mask"]))
        if cfg.get("router"):
            opt(3, socket.inet_aton(cfg["router"]))
        if cfg.get("dns"):
            opt(6, socket.inet_aton(cfg["dns"]))
        if cfg.get("broadcast"):
            opt(28, socket.inet_aton(cfg["broadcast"]))
    o.append(255)
    pkt = hdr + bytes(o)
    return pkt + b"\x00" * max(0, 300 - len(pkt))


class Server:
    def __init__(self, server_ip, pool, cfg, statics, leases_path, log_path):
        self.ip, self.pool, self.cfg, self.statics = server_ip, pool, cfg, statics
        self.leases_path, self.log_path = leases_path, log_path
        self.leases = {}   # mac → {"ip", "expires", "hostname"}
        self.offers = {}   # mac → ip(申し出中)
        self.declined = set()
        if leases_path and os.path.exists(leases_path):
            with open(leases_path, encoding="utf-8") as f:
                self.leases = json.load(f)

    def save(self):
        if self.leases_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.leases_path)), exist_ok=True)
            with open(self.leases_path, "w", encoding="utf-8") as f:
                json.dump(self.leases, f, indent=1)

    def log(self, event, mac, ip, host=""):
        line = "%s %-8s %s %s %s" % (datetime.datetime.now().strftime("%H:%M:%S"), event, mac, ip or "-", host)
        print(line, flush=True)
        if self.log_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.log_path)), exist_ok=True)
            new = not os.path.exists(self.log_path)
            with open(self.log_path, "a", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                if new:
                    w.writerow(["time", "event", "mac", "ip", "hostname"])
                w.writerow([datetime.datetime.now().isoformat(timespec="seconds"), event, mac, ip or "", host])

    def in_use(self, ip, mac):
        now = time.time()
        for m, l in self.leases.items():
            if m != mac and l["ip"] == ip and l["expires"] > now:
                return True
        if ip in self.statics.values() and self.statics.get(mac) != ip:
            return True
        return any(m != mac and v == ip for m, v in self.offers.items())

    def choose(self, mac, wanted=None):
        if mac in self.statics:
            return self.statics[mac]
        if mac in self.leases and self.leases[mac]["ip"] in self.pool and not self.in_use(self.leases[mac]["ip"], mac):
            return self.leases[mac]["ip"]
        if wanted and wanted in self.pool and wanted not in self.declined and not self.in_use(wanted, mac):
            return wanted
        for ip in self.pool:
            if ip not in self.declined and not self.in_use(ip, mac):
                return ip
        return None

    def handle(self, req):
        """戻り値 (返事のバイト列, 種類) か None"""
        if req is None or req["op"] != 1:
            return None
        mac, t = req["mac"], req["type"]
        want = socket.inet_ntoa(req["opts"][50]) if 50 in req["opts"] and len(req["opts"][50]) == 4 else None
        sid = socket.inet_ntoa(req["opts"][54]) if 54 in req["opts"] and len(req["opts"][54]) == 4 else None
        if t == DISCOVER:
            ip = self.choose(mac, want)
            if not ip:
                self.log("NOFREE", mac, None, req["hostname"])
                return None
            self.offers[mac] = ip
            self.log("OFFER", mac, ip, req["hostname"])
            return build_reply(req, OFFER, ip, self.ip, self.cfg), OFFER
        if t == REQUEST:
            if sid and sid != self.ip:      # 他のサーバを選んだ
                self.offers.pop(mac, None)
                return None
            ip = want or (req["ciaddr"] if req["ciaddr"] != "0.0.0.0" else None)
            ok = ip is not None and (ip in self.pool or self.statics.get(mac) == ip) and not self.in_use(ip, mac)
            if mac in self.statics:
                ok = ok and ip == self.statics[mac]
            if not ok:
                self.log("NAK", mac, ip, req["hostname"])
                return build_reply(req, NAK, None, self.ip, self.cfg), NAK
            self.offers.pop(mac, None)
            self.leases[mac] = {"ip": ip, "expires": time.time() + self.cfg["lease"], "hostname": req["hostname"]}
            self.save()
            self.log("ACK", mac, ip, req["hostname"])
            return build_reply(req, ACK, ip, self.ip, self.cfg), ACK
        if t == RELEASE:
            if mac in self.leases:
                self.leases[mac]["expires"] = 0
                self.save()
            self.log("RELEASE", mac, req["ciaddr"], req["hostname"])
            return None
        if t == DECLINE:
            if want:
                self.declined.add(want)
            self.log("DECLINE", mac, want, req["hostname"])
            return None
        if t == INFORM:
            self.log("INFORM", mac, req["ciaddr"], req["hostname"])
            return build_reply(req, ACK, None, self.ip, self.cfg), ACK
        return None


def make_request(mac, mtype, xid=0x1234, want=None, server=None, hostname="dyne-node"):
    chaddr = bytes(int(x, 16) for x in mac.split(":"))
    pkt = struct.pack("!BBBBIHH", 1, 1, 6, 0, xid, 0, 0x8000) + b"\x00" * 16 + chaddr.ljust(16, b"\x00") + b"\x00" * 192
    o = bytearray(MAGIC) + bytes([53, 1, mtype])
    if want:
        o += bytes([50, 4]) + socket.inet_aton(want)
    if server:
        o += bytes([54, 4]) + socket.inet_aton(server)
    o += bytes([12, len(hostname)]) + hostname.encode()
    o.append(255)
    return pkt + bytes(o)


def selftest():
    pool = ["192.168.50.%d" % k for k in range(100, 103)]
    cfg = {"lease": 3600, "mask": "255.255.255.0", "router": None, "dns": None, "broadcast": "192.168.50.255"}
    s = Server("192.168.50.1", pool, cfg, {"aa:aa:aa:aa:aa:01": "192.168.50.11"}, None, None)
    fails = []

    def ask(mac, t, **kw):
        r = s.handle(parse_packet(make_request(mac, t, **kw)))
        if r is None:
            return None, None
        p = parse_packet(r[0])
        return p["type"], socket.inet_ntoa(r[0][16:20])
    t, ip = ask("02:00:00:00:00:01", DISCOVER)
    if (t, ip) != (OFFER, "192.168.50.100"):
        fails.append("DISCOVER → %s %s" % (t, ip))
    t, ip2 = ask("02:00:00:00:00:01", REQUEST, want=ip, server="192.168.50.1")
    if (t, ip2) != (ACK, ip):
        fails.append("REQUEST → %s %s" % (t, ip2))
    t, ip3 = ask("02:00:00:00:00:02", DISCOVER)
    if ip3 != "192.168.50.101":
        fails.append("2 台目の DISCOVER → %s" % ip3)
    t, _ = ask("02:00:00:00:00:02", REQUEST, want=ip, server="192.168.50.1")
    if t != NAK:
        fails.append("使用中のアドレスを求めたのに %s" % t)
    t, ip4 = ask("aa:aa:aa:aa:aa:01", DISCOVER)
    if ip4 != "192.168.50.11":
        fails.append("静的な割り当て → %s" % ip4)
    for k in range(3, 6):
        ask("02:00:00:00:00:%02x" % k, DISCOVER)
    t, ip5 = ask("02:00:00:00:00:09", DISCOVER)
    if t is not None:
        fails.append("空きが無いのに %s %s" % (t, ip5))
    if fails:
        print("FAIL: " + "; ".join(fails))
        sys.exit(1)
    print("PASS: DISCOVER/OFFER・REQUEST/ACK・使用中の NAK・静的な割り当て・空き無し")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", help="USB の有線 LAN の変換器に付けた IP アドレス(例 192.168.50.1)")
    ap.add_argument("--any", action="store_true", help="0.0.0.0 に結び付けることを許す(勧めない)")
    ap.add_argument("--range", help="配る範囲 例 192.168.50.100-192.168.50.199")
    ap.add_argument("--mask", default="255.255.255.0")
    ap.add_argument("--router", default=None)
    ap.add_argument("--dns", default=None)
    ap.add_argument("--lease", type=int, default=12 * 3600, help="貸し出しの期間 [秒]")
    ap.add_argument("--static", action="append", default=[], help="MAC=IP(何度でも)")
    ap.add_argument("--leases", default=os.path.join("demo_out", "dhcp", "leases.json"))
    ap.add_argument("--log", default=os.path.join("demo_out", "dhcp", "dhcp_log.csv"))
    ap.add_argument("--port", type=int, default=67)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return
    if not a.bind or not a.range:
        ap.error("--bind と --range が要る")
    if a.bind == "0.0.0.0" and not a.any:
        ap.error("0.0.0.0 には --any が要る(家庭の網に誤ってアドレスを配らないため)")
    lo, hi = (ipaddress.IPv4Address(x) for x in a.range.split("-"))
    net = ipaddress.IPv4Network("%s/%s" % (a.bind, a.mask), strict=False) if a.bind != "0.0.0.0" else None
    if net and (lo not in net or hi not in net):
        ap.error("範囲 %s が %s の網 %s の外" % (a.range, a.bind, net))
    pool = [str(ipaddress.IPv4Address(k)) for k in range(int(lo), int(hi) + 1) if str(ipaddress.IPv4Address(k)) != a.bind]
    statics = {}
    for s in a.static:
        m, ip = s.split("=")
        statics[m.lower()] = ip
    cfg = {"lease": a.lease, "mask": a.mask, "router": a.router, "dns": a.dns,
           "broadcast": str(net.broadcast_address) if net else None}
    srv = Server(a.bind, pool, cfg, statics, a.leases, a.log)
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sk.bind((a.bind, a.port))
    except OSError as e:
        sys.exit("口 %s:%d に結び付けられない (%s)。ICS が有効だと Windows の DHCP が使っている。"
                 "変換器の IP アドレスが %s になっているかも確かめる" % (a.bind, a.port, e, a.bind))
    print("DHCP: %s:%d で待つ。配る範囲 %s(%d 個)、マスク %s、経路 %s、DNS %s、期間 %d 秒、静的 %d 件"
          % (a.bind, a.port, a.range, len(pool), a.mask, a.router or "配らない", a.dns or "配らない", a.lease, len(statics)),
          flush=True)
    bcast = cfg["broadcast"] or "255.255.255.255"
    while True:
        try:
            data, addr = sk.recvfrom(4096)
        except KeyboardInterrupt:
            break
        except OSError:
            continue
        try:
            r = srv.handle(parse_packet(data))
        except Exception as e:  # noqa: BLE001  壊れた要求は記録して捨てる
            print("壊れた要求 %s: %s" % (addr, e), flush=True)
            continue
        if r is None:
            continue
        pkt, _ = r
        req = parse_packet(data)
        if req["giaddr"] != "0.0.0.0":
            dest = (req["giaddr"], 67)
        elif req["ciaddr"] != "0.0.0.0" and req["type"] in (REQUEST, INFORM):
            dest = (req["ciaddr"], 68)
        else:
            dest = ("255.255.255.255", 68)
        try:
            sk.sendto(pkt, dest)
        except OSError:
            sk.sendto(pkt, (bcast, 68))


if __name__ == "__main__":
    main()

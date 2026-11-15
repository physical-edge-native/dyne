#!/usr/bin/env python3
"""ノードの時刻同期の基準になる、小さな NTP (ネットワーク時刻プロトコル) サーバ (2026-11-11)。

標準ライブラリだけで動く。Windows の時刻サービス (w32time) の設定に頼らずに、
PC を全ノードの時計の基準にするために使う。説明: docs/2026-11-11-timesync-ota.md

ノードは2通りでこのサーバに問い合わせる:
  1. ESP-IDF 標準の SNTP (簡易版 NTP)。システムの時計 (gettimeofday) を合わせる。UDP 123 番固定。
  2. ファーム独自の往復の時刻印の交換 (NTP と同じパケット)。往復時間を測ってずれを補正し、
     ずれ・往復時間・推定の誤差を MQTT で報告する。ポートは設定で変えられる。

基準の時計: 起動時の PC の時刻 + 高分解能カウンタ (perf_counter) の経過。
起動後に Windows が PC の時計を合わせ直しても、基準は跳ばない (全ノードが同じ基準にそろうことが目的。
PC の時計の絶対的な正しさは問わない)。--wall で PC の時計をそのまま使う。

使い方 (リポジトリのルートで):
  python scripts\\dyne_ntp_server.py --bind 0.0.0.0 --port 123
      123 番が使用中 (w32time が動いている) なら、管理者の PowerShell で `net stop w32time`
      してから起動するか、別のポート (例 12300) で起動してノードに port を設定注入する
      (その場合 SNTP は PC の w32time に問い合わせ、独自の交換だけがこのサーバを使う)。
  python scripts\\dyne_ntp_server.py --port 12300 --log ntp_log.csv   要求ごとに1行記録
  python scripts\\dyne_ntp_server.py --query 127.0.0.1:12300 -n 20     動作確認 (問い合わせ側)
"""
import argparse
import csv
import socket
import struct
import sys
import time

NTP_DELTA_S = 2208988800  # 1900-01-01 から 1970-01-01 までの秒数


class RefClock:
    """基準の時計 (Unix µs)。既定は「起動時の PC の時刻 + 高分解能カウンタの経過」"""

    def __init__(self, wall=False):
        self.wall = wall
        self.wall0 = time.time_ns()
        self.perf0 = time.perf_counter_ns()

    def now_us(self):
        if self.wall:
            return time.time_ns() // 1000
        return (self.wall0 + (time.perf_counter_ns() - self.perf0)) // 1000


def to_ntp(unix_us):
    sec, rem = divmod(unix_us, 1_000_000)
    frac = (rem << 32) // 1_000_000
    return ((sec + NTP_DELTA_S) << 32) | frac


def from_ntp(ntp):
    sec = ntp >> 32
    frac = ntp & 0xFFFFFFFF
    return (sec - NTP_DELTA_S) * 1_000_000 + ((frac * 1_000_000 + 0x80000000) >> 32)


def make_reply(req, t2_us, t3_us, ref_us):
    """NTP 応答 (48バイト)。依頼側の送信時刻の欄を、応答の依頼側の時刻の欄へそのまま写す"""
    vn = (req[0] >> 3) & 7
    if vn < 1 or vn > 4:
        vn = 4
    poll = req[2]
    li_vn_mode = (0 << 6) | (vn << 3) | 4  # うるう秒の指示なし、サーバモード
    stratum = 1
    precision = -20 & 0xFF  # 約 1µs
    root_delay = 0
    root_disp = 1  # 1/65536 秒
    refid = b"DYNE"
    orig = req[40:48]
    return (struct.pack("!BBBBII4s", li_vn_mode, stratum, poll, precision, root_delay, root_disp, refid)
            + struct.pack("!Q", to_ntp(ref_us)) + orig
            + struct.pack("!QQ", to_ntp(t2_us), to_ntp(t3_us)))


def serve(args):
    clk = RefClock(wall=args.wall)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.bind, args.port))
    except OSError as e:
        print(f"bind {args.bind}:{args.port} に失敗: {e}", file=sys.stderr)
        if args.port == 123:
            print("123 番は Windows の時刻サービス (w32time) が使っていることが多い。管理者の PowerShell で\n"
                  "  net stop w32time\nを実行するか、--port 12300 など別のポートで起動する。", file=sys.stderr)
        return 1
    ref_us = clk.now_us()
    print(f"dyne NTP server on {args.bind}:{args.port}  clock={'wall' if args.wall else 'anchored perf_counter'}"
          f"  start={time.strftime('%Y-%m-%d %H:%M:%S')}", flush=True)
    logf = open(args.log, "w", newline="") if args.log else None
    logw = csv.writer(logf) if logf else None
    if logw:
        logw.writerow(["client", "port", "version", "mode", "t2_us", "t3_us", "server_proc_us"])
    counts = {}
    last_print = time.monotonic()
    try:
        while True:
            try:
                req, addr = sock.recvfrom(512)
            except ConnectionResetError:
                continue  # Windows: 前回の送信先が閉じていると ICMP で例外になる。無視
            t2 = clk.now_us()
            if len(req) < 48:
                continue
            mode = req[0] & 7
            if mode != 3:  # 依頼者モード以外は無視
                continue
            # t3 は送信直前に取る。make_reply の中で取ると送信までの時間が短くなる
            t3 = clk.now_us()
            rep = make_reply(req, t2, t3, ref_us)
            sock.sendto(rep, addr)
            counts[addr[0]] = counts.get(addr[0], 0) + 1
            if logw:
                logw.writerow([addr[0], addr[1], (req[0] >> 3) & 7, mode, t2, t3, t3 - t2])
                logf.flush()
            if not args.quiet and time.monotonic() - last_print >= args.stats_every:
                last_print = time.monotonic()
                summary = " ".join(f"{ip}:{n}" for ip, n in sorted(counts.items()))
                print(f"[{time.strftime('%H:%M:%S')}] 要求数 {summary}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if logf:
            logf.close()
    return 0


def query(args):
    """問い合わせ側 (動作確認用)。往復時間とずれを表示する。ずれ = サーバの基準 - この PC の時計"""
    host, port = args.query.rsplit(":", 1)
    port = int(port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    ok = 0
    rtts, offs = [], []
    for i in range(args.n):
        t1 = time.time_ns() // 1000
        req = bytearray(48)
        req[0] = (4 << 3) | 3
        req[40:48] = struct.pack("!Q", to_ntp(t1))
        sock.sendto(bytes(req), (host, port))
        try:
            rep, _ = sock.recvfrom(512)
        except socket.timeout:
            print(f"#{i} 応答なし")
            continue
        t4 = time.time_ns() // 1000
        if len(rep) < 48 or (rep[0] & 7) != 4 or rep[1] == 0 or rep[24:32] != req[40:48]:
            print(f"#{i} 不正な応答")
            continue
        t2 = from_ntp(struct.unpack("!Q", rep[32:40])[0])
        t3 = from_ntp(struct.unpack("!Q", rep[40:48])[0])
        rtt = (t4 - t1) - (t3 - t2)
        off = ((t2 - t1) + (t3 - t4)) // 2
        rtts.append(rtt)
        offs.append(off)
        ok += 1
        print(f"#{i} rtt={rtt} us offset={off} us stratum={rep[1]} refid={rep[12:16]!r}")
        time.sleep(args.interval)
    if ok:
        print(f"応答 {ok}/{args.n}  往復時間 最小 {min(rtts)} us / 中央 {sorted(rtts)[len(rtts)//2]} us")
    return 0 if ok == args.n else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="0.0.0.0", help="待ち受けるアドレス (既定 0.0.0.0 = 全部)")
    ap.add_argument("--port", type=int, default=123, help="UDP ポート (既定 123。SNTP は 123 固定)")
    ap.add_argument("--wall", action="store_true", help="PC の時計をそのまま基準にする (既定は起動時に固定)")
    ap.add_argument("--log", help="要求ごとの記録の CSV")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--stats-every", type=float, default=30.0, help="要求数の表示の間隔 [s]")
    ap.add_argument("--query", help="問い合わせ側として動かす (host:port)")
    ap.add_argument("-n", type=int, default=10, help="--query の回数")
    ap.add_argument("--interval", type=float, default=0.05, help="--query の間隔 [s]")
    args = ap.parse_args()
    return query(args) if args.query else serve(args)


if __name__ == "__main__":
    sys.exit(main())

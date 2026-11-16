#!/usr/bin/env python3
"""Minimal MQTT 3.1.1 client and broker, standard library only (GitHub issue #4, 2026-11-16).

Why: the PC tools use paho-mqtt and a mosquitto broker, but CI (Linux / macOS / Windows) installs no Python
packages and runs no broker. This module lets the wave2d cluster flow run end to end against fake nodes
in CI (scripts/wave2d_rehearsal.py). It is not a general-purpose MQTT implementation:

  - Client: the subset of the paho-mqtt 2.x API that the Dyne tools use (Client, CallbackAPIVersion,
    connect, loop_start / loop_stop, subscribe, publish(...).wait_for_publish(), will_set, disconnect,
    on_connect / on_message with the VERSION2 signatures). QoS 0 and 1, retained messages, keepalive.
    Tools import paho first and fall back to this module (`import dyne_mqtt_lite as mqtt`).
  - Broker: one process, in-memory retained store, + and # wildcards, QoS 0 and 1 (no redelivery),
    last will on unexpected disconnect. Bind it to 127.0.0.1; it has no authentication.

For the hardware experiments use mosquitto (and paho, which the tools pick when it is installed).

Command line (a broker for a local rehearsal without mosquitto):
  python scripts/dyne_mqtt_lite.py broker [--host 127.0.0.1] [--port 1883] [--port-file F]
"""
import argparse
import itertools
import socket
import struct
import sys
import threading
import time

# ---- packet types ----
CONNECT, CONNACK, PUBLISH, PUBACK = 1, 2, 3, 4
SUBSCRIBE, SUBACK, UNSUBSCRIBE, UNSUBACK = 8, 9, 10, 11
PINGREQ, PINGRESP, DISCONNECT = 12, 13, 14


def _enc_len(n):
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def _enc_str(s):
    b = s.encode("utf-8") if isinstance(s, str) else bytes(s)
    return struct.pack("!H", len(b)) + b


def _packet(ptype, flags, body):
    return bytes([(ptype << 4) | flags]) + _enc_len(len(body)) + body


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return bytes(buf)


def _read_packet(sock):
    """Returns (type, flags, body). Raises ConnectionError on close, socket.timeout if nothing arrived."""
    h = sock.recv(1)
    if not h:
        raise ConnectionError("closed")
    old = sock.gettimeout()
    sock.settimeout(None)  # the first byte arrived: read the rest of the packet without a timeout
    try:
        mult, n = 1, 0
        while True:
            b = _recv_exact(sock, 1)[0]
            n += (b & 0x7F) * mult
            if not b & 0x80:
                break
            mult *= 128
        body = _recv_exact(sock, n) if n else b""
    finally:
        sock.settimeout(old)
    return h[0] >> 4, h[0] & 0x0F, body


def _publish_packet(topic, payload, qos, retain, pid):
    body = _enc_str(topic) + (struct.pack("!H", pid) if qos else b"") + payload
    return _packet(PUBLISH, (qos << 1) | (1 if retain else 0), body)


def _parse_publish(flags, body):
    qos = (flags >> 1) & 3
    tl = struct.unpack("!H", body[:2])[0]
    topic = body[2:2 + tl].decode("utf-8")
    pos = 2 + tl
    pid = None
    if qos:
        pid = struct.unpack("!H", body[pos:pos + 2])[0]
        pos += 2
    return topic, body[pos:], qos, bool(flags & 1), pid


def topic_matches(filt, topic):
    """MQTT topic filter matching (+ = one level, # = the rest)."""
    if topic.startswith("$") and not filt.startswith("$"):
        return False
    f, t = filt.split("/"), topic.split("/")
    for i, part in enumerate(f):
        if part == "#":
            return True
        if i >= len(t):
            return False
        if part != "+" and part != t[i]:
            return False
    return len(f) == len(t)


def pick(kind="auto"):
    """The MQTT client module to use: 'paho' (paho-mqtt), 'lite' (this module) or 'auto' (paho if installed)."""
    if kind in ("auto", "paho"):
        try:
            import paho.mqtt.client as m
            return m
        except ImportError:
            if kind == "paho":
                raise
    return sys.modules[__name__]


def _to_bytes(payload):
    if payload is None:
        return b""
    if isinstance(payload, (bytes, bytearray)):
        return bytes(payload)
    if isinstance(payload, str):
        return payload.encode("utf-8")
    return str(payload).encode("utf-8")


# ======================= client (paho-mqtt 2.x subset) =======================

class CallbackAPIVersion:
    VERSION1 = 1
    VERSION2 = 2


MQTT_ERR_SUCCESS = 0


class MQTTMessage:
    def __init__(self, topic, payload, qos, retain):
        self.topic = topic
        self.payload = payload
        self.qos = qos
        self.retain = retain


class MQTTMessageInfo:
    def __init__(self, mid):
        self.mid = mid
        self.rc = MQTT_ERR_SUCCESS
        self._ev = threading.Event()

    def is_published(self):
        return self._ev.is_set()

    def wait_for_publish(self, timeout=None):
        if not self._ev.wait(timeout if timeout is not None else 30.0):
            raise RuntimeError("MQTT publish not acknowledged")


class Client:
    def __init__(self, callback_api_version=CallbackAPIVersion.VERSION2, client_id="", userdata=None, **_kw):
        self._client_id = client_id or f"lite-{id(self):x}"
        self._userdata = userdata
        self.on_connect = None
        self.on_message = None
        self.on_disconnect = None
        self._sock = None
        self._wlock = threading.Lock()
        self._mid = itertools.count(1)
        self._pending = {}  # mid -> MQTTMessageInfo (QoS 1 waiting for PUBACK)
        self._thread = None
        self._running = False
        self._will = None
        self._keepalive = 60
        self._last_tx = 0.0

    def will_set(self, topic, payload=None, qos=0, retain=False):
        self._will = (topic, _to_bytes(payload), qos, retain)

    def connect(self, host, port=1883, keepalive=60):
        self._keepalive = keepalive
        s = socket.create_connection((host, port), timeout=10)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        flags = 0x02  # clean session
        payload = _enc_str(self._client_id)
        if self._will:
            wt, wp, wq, wr = self._will
            flags |= 0x04 | (wq << 3) | (0x20 if wr else 0)
            payload += _enc_str(wt) + struct.pack("!H", len(wp)) + wp
        body = _enc_str("MQTT") + bytes([4, flags]) + struct.pack("!H", keepalive) + payload
        s.sendall(_packet(CONNECT, 0, body))
        ptype, _, b = _read_packet(s)
        if ptype != CONNACK or len(b) < 2 or b[1] != 0:
            s.close()
            raise ConnectionError(f"MQTT connect refused ({b!r})")
        s.settimeout(1.0)
        self._sock = s
        self._last_tx = time.time()
        return 0

    def _send(self, data):
        with self._wlock:
            if self._sock is None:
                raise ConnectionError("not connected")
            self._sock.sendall(data)
            self._last_tx = time.time()

    def subscribe(self, topic, qos=0):
        subs = topic if isinstance(topic, list) else [(topic, qos)]
        mid = next(self._mid) % 65536 or 1
        body = struct.pack("!H", mid) + b"".join(_enc_str(t) + bytes([min(q, 1)]) for t, q in subs)
        self._send(_packet(SUBSCRIBE, 0x02, body))
        return (MQTT_ERR_SUCCESS, mid)

    def publish(self, topic, payload=None, qos=0, retain=False):
        qos = min(qos, 1)
        mid = next(self._mid) % 65536 or 1
        info = MQTTMessageInfo(mid)
        if qos:
            self._pending[mid] = info
        self._send(_publish_packet(topic, _to_bytes(payload), qos, retain, mid))
        if not qos:
            info._ev.set()
        return info

    def _loop(self):
        if self.on_connect:
            self.on_connect(self, self._userdata, {}, 0, None)
        while self._running:
            try:
                ptype, flags, body = _read_packet(self._sock)
            except socket.timeout:
                if time.time() - self._last_tx > self._keepalive / 2:
                    try:
                        self._send(_packet(PINGREQ, 0, b""))
                    except OSError:
                        break
                continue
            except (OSError, ConnectionError):
                break
            if ptype == PUBLISH:
                topic, payload, qos, retain, pid = _parse_publish(flags, body)
                if qos == 1:
                    try:
                        self._send(_packet(PUBACK, 0, struct.pack("!H", pid)))
                    except OSError:
                        break
                if self.on_message:
                    try:
                        self.on_message(self, self._userdata, MQTTMessage(topic, payload, qos, retain))
                    except Exception as e:  # keep the loop alive (paho logs and continues as well)
                        print(f"dyne_mqtt_lite: on_message raised {e!r}", file=sys.stderr)
            elif ptype == PUBACK:
                info = self._pending.pop(struct.unpack("!H", body[:2])[0], None)
                if info:
                    info._ev.set()
        if self._running and self.on_disconnect:
            self.on_disconnect(self, self._userdata, {}, 1, None)

    def loop_start(self):
        if self._thread:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def loop_stop(self):
        self._running = False
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=3)
        self._thread = None

    def disconnect(self):
        try:
            self._send(_packet(DISCONNECT, 0, b""))
        except OSError:
            pass
        self._running = False
        with self._wlock:
            if self._sock:
                try:
                    self._sock.close()
                except OSError:
                    pass
            self._sock = None


# ======================= broker =======================

class Broker:
    """In-process MQTT broker for tests. start() returns the port (port 0 = pick a free one)."""

    def __init__(self, host="127.0.0.1", port=0, verbose=False):
        self.host, self.port, self.verbose = host, port, verbose
        self._lock = threading.Lock()
        self._sessions = set()
        self._retained = {}  # topic -> (payload, qos)
        self._srv = None
        self._mid = itertools.count(1)

    def start(self):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(64)
        self.port = self._srv.getsockname()[1]
        threading.Thread(target=self._accept, daemon=True).start()
        return self.port

    def stop(self):
        try:
            self._srv.close()
        except OSError:
            pass
        with self._lock:
            sessions = list(self._sessions)
        for s in sessions:
            s.close()

    def _accept(self):
        while True:
            try:
                conn, _ = self._srv.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=_Session(self, conn).run, daemon=True).start()

    def route(self, topic, payload, qos, retain):
        if retain:
            with self._lock:
                if payload:
                    self._retained[topic] = (payload, qos)
                else:
                    self._retained.pop(topic, None)
        with self._lock:
            sessions = list(self._sessions)
        for s in sessions:
            s.deliver(topic, payload, qos, False)

    def retained_for(self, filt):
        with self._lock:
            return [(t, p, q) for t, (p, q) in self._retained.items() if topic_matches(filt, t)]


class _Session:
    def __init__(self, broker, conn):
        self.b, self.conn = broker, conn
        self.subs = {}
        self.will = None
        self.wlock = threading.Lock()
        self.alive = True

    def send(self, data):
        with self.wlock:
            self.conn.sendall(data)

    def deliver(self, topic, payload, qos, retain):
        q = -1
        for f, sq in list(self.subs.items()):
            if topic_matches(f, topic):
                q = max(q, sq)
        if q < 0:
            return
        q = min(q, qos)
        try:
            self.send(_publish_packet(topic, payload, q, retain, next(self.b._mid) % 65536 or 1))
        except OSError:
            self.close()

    def close(self):
        self.alive = False
        try:
            self.conn.close()
        except OSError:
            pass

    def run(self):
        clean = False
        try:
            ptype, _, body = _read_packet(self.conn)
            if ptype != CONNECT:
                return
            pos = 2 + struct.unpack("!H", body[:2])[0]
            flags = body[pos + 1]
            pos += 4
            cl = struct.unpack("!H", body[pos:pos + 2])[0]
            pos += 2 + cl
            if flags & 0x04:
                tl = struct.unpack("!H", body[pos:pos + 2])[0]
                wt = body[pos + 2:pos + 2 + tl].decode("utf-8")
                pos += 2 + tl
                pl = struct.unpack("!H", body[pos:pos + 2])[0]
                wp = body[pos + 2:pos + 2 + pl]
                self.will = (wt, wp, (flags >> 3) & 3, bool(flags & 0x20))
            self.send(_packet(CONNACK, 0, b"\x00\x00"))
            with self.b._lock:
                self.b._sessions.add(self)
            while self.alive:
                ptype, flags, body = _read_packet(self.conn)
                if ptype == PUBLISH:
                    topic, payload, qos, retain, pid = _parse_publish(flags, body)
                    if qos == 1:
                        self.send(_packet(PUBACK, 0, struct.pack("!H", pid)))
                    self.b.route(topic, payload, qos, retain)
                elif ptype == SUBSCRIBE:
                    pid = body[:2]
                    pos, granted, new = 2, bytearray(), []
                    while pos < len(body):
                        tl = struct.unpack("!H", body[pos:pos + 2])[0]
                        filt = body[pos + 2:pos + 2 + tl].decode("utf-8")
                        q = min(body[pos + 2 + tl], 1)
                        pos += 3 + tl
                        self.subs[filt] = q
                        granted.append(q)
                        new.append((filt, q))
                    self.send(_packet(SUBACK, 0, pid + bytes(granted)))
                    for filt, q in new:
                        for t, p, rq in self.b.retained_for(filt):
                            self.send(_publish_packet(t, p, min(q, rq), True, next(self.b._mid) % 65536 or 1))
                elif ptype == UNSUBSCRIBE:
                    pos = 2
                    while pos < len(body):
                        tl = struct.unpack("!H", body[pos:pos + 2])[0]
                        self.subs.pop(body[pos + 2:pos + 2 + tl].decode("utf-8"), None)
                        pos += 2 + tl
                    self.send(_packet(UNSUBACK, 0, body[:2]))
                elif ptype == PINGREQ:
                    self.send(_packet(PINGRESP, 0, b""))
                elif ptype == DISCONNECT:
                    clean = True
                    break
                # PUBACK from subscribers: nothing to do (no redelivery)
        except (OSError, ConnectionError, struct.error, IndexError):
            pass
        finally:
            with self.b._lock:
                self.b._sessions.discard(self)
            self.close()
            if not clean and self.will:
                self.b.route(*self.will)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("broker", help="run a broker until killed")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--port-file", help="write the port number here once listening")
    a = ap.parse_args()
    b = Broker(a.host, a.port)
    port = b.start()
    print(f"dyne_mqtt_lite broker listening on {a.host}:{port}", flush=True)
    if a.port_file:
        with open(a.port_file, "w", encoding="utf-8") as f:
            f.write(str(port))
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        b.stop()


if __name__ == "__main__":
    main()

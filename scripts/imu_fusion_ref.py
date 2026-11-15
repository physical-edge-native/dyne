# IMU の値を隣との通信だけで融合するアクター (crates/imu-fusion-actor) の参照モデルと、
# 実物の WASM を動かす包み、N ノードのネットワークを1ステップずつ進める実行器。
#
# 出所: Claude Code が 2026-11-13 に作成。記録は docs/2026-11-13-imu-fusion.md。
# - ImuFusionRef: Rust の lib.rs と同じ順序で同じ浮動小数点演算をする (ビット単位で揃える)。
#   送信/受信メールボックスのバイト列もワイヤ形式v2 (crates/dyne-wire) と同じに作る・読む
# - ImuFusionWasm: wasmtime で実物の .wasm を動かす (同じメソッド)
# - run_network: 標本の割り当て表に従って push → 送信箱 → 配送 → 受信箱 → step を繰り返す
# 定数の正本は crates/dyne-wire/src/abi.rs の imu モジュール (ここは写し)。
import math
import os
import struct

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
WASM = os.path.join(ROOT, 'crates', 'imu-fusion-actor', 'target', 'wasm32-unknown-unknown', 'release',
                    'imu_fusion_actor.wasm')

# ---- abi.rs の写し ----
CFG_EPOCH_STEPS, CFG_TIME_MODE, CFG_T0_US, CFG_TS_OFFSET_US, CFG_CF_ALPHA, CFG_GYRO_CAL_EPOCHS, CFG_CONSENSUS = range(7)
CONS_RESET, CONS_DYNAMIC = 0, 1
CFG_DEFAULT = [10.0, 0.0, 0.0, 0.0, 0.9, 0.0, 1.0]
TIME_STAMP, TIME_ARRIVAL = 0, 1
PER_POINT = 8
PUSH_OK, PUSH_LATE, PUSH_EARLY, PUSH_NOT_READY = 0, 1, 2, -1
G_EPOCH, G_FUSED_N, G_FUSED = 0, 1, 2
G_FUSED_ROLL, G_FUSED_PITCH = 8, 9
G_LOCAL_N, G_LOCAL = 10, 11
G_LOCAL_ROLL, G_LOCAL_PITCH = 17, 18
G_LATE, G_EARLY, G_MISSING, G_LEN = 19, 20, 21, 22
ANGLE_IDX = (G_FUSED_ROLL, G_FUSED_PITCH, G_LOCAL_ROLL, G_LOCAL_PITCH)
# field2d
FACE_W, FACE_E, FACE_S, FACE_N = 0, 1, 2, 3
BC_INTERFACE = 1
STEP_OK, STEP_MISSING_GHOST = 0, 1
INBOX_OK, INBOX_REJECTED, INBOX_MALFORMED = 0, 1, -1
# ワイヤ形式v2
KIND_IFACE_BAND = 1
QTY_INERTIAL = 3
RING = 4
XLEN = 7
SCRATCH_CAP = 512


# ---- libm (Rust の純Rust版、FreeBSD msun 由来) の atan / atan2 の写し ----
# アクターの相補フィルタの角度をビット単位で照合するため、Python の math.atan2 (C の実装。
# 最後の1桁が違いうる) ではなく、アクターが使う libm 0.2.16 と同じ手順で計算する。
_ATANHI = (4.63647609000806093515e-01, 7.85398163397448278999e-01,
           9.82793723247329054082e-01, 1.57079632679489655800e+00)
_ATANLO = (2.26987774529616870924e-17, 3.06161699786838301793e-17,
           1.39033110312309984516e-17, 6.12323399573676603587e-17)
_AT = (3.33333333333329318027e-01, -1.99999999998764832476e-01, 1.42857142725034663711e-01,
       -1.11111104054623557880e-01, 9.09088713343650656196e-02, -7.69187620504482999495e-02,
       6.66107313738753120669e-02, -5.83357013379057348645e-02, 4.97687799461593236017e-02,
       -3.65315727442169155270e-02, 1.62858201153657823623e-02)
_PI = 3.1415926535897931160E+00
_PI_LO = 1.2246467991473531772E-16


def _bits(x):
    return struct.unpack('<Q', struct.pack('<d', x))[0]


def libm_atan(x):
    hx = _bits(x) >> 32
    sign = hx >> 31
    ix = hx & 0x7fffffff
    if ix >= 0x44100000:
        if x != x:
            return x
        z = _ATANHI[3] + struct.unpack('<d', struct.pack('<Q', 0x0380_0000))[0]
        return -z if sign else z
    if ix < 0x3fdc0000:
        if ix < 0x3e400000:
            return x
        idx = -1
    else:
        x = abs(x)
        if ix < 0x3ff30000:
            if ix < 0x3fe60000:
                x = (2. * x - 1.) / (2. + x)
                idx = 0
            else:
                x = (x - 1.) / (x + 1.)
                idx = 1
        elif ix < 0x40038000:
            x = (x - 1.5) / (1. + 1.5 * x)
            idx = 2
        else:
            x = -1. / x
            idx = 3
    z = x * x
    w = z * z
    A = _AT
    s1 = z * (A[0] + w * (A[2] + w * (A[4] + w * (A[6] + w * (A[8] + w * A[10])))))
    s2 = w * (A[1] + w * (A[3] + w * (A[5] + w * (A[7] + w * A[9]))))
    if idx < 0:
        return x - x * (s1 + s2)
    z = _ATANHI[idx] - (x * (s1 + s2) - _ATANLO[idx] - x)
    return -z if sign else z


def libm_atan2(y, x):
    if x != x or y != y:
        return x + y
    bx, by = _bits(x), _bits(y)
    ix, lx = bx >> 32, bx & 0xffffffff
    iy, ly = by >> 32, by & 0xffffffff
    if ((ix - 0x3ff00000) & 0xffffffff) | lx == 0:
        return libm_atan(y)
    m = ((iy >> 31) & 1) | ((ix >> 30) & 2)
    ix &= 0x7fffffff
    iy &= 0x7fffffff
    if iy | ly == 0:
        return y if m in (0, 1) else (_PI if m == 2 else -_PI)
    if ix | lx == 0:
        return -_PI / 2.0 if m & 1 else _PI / 2.0
    if ix == 0x7ff00000:
        if iy == 0x7ff00000:
            return (_PI / 4.0, -_PI / 4.0, 3.0 * _PI / 4.0, -3.0 * _PI / 4.0)[m]
        return (0.0, -0.0, _PI, -_PI)[m]
    if ((ix + (64 << 20)) & 0xffffffff) < iy or iy == 0x7ff00000:
        return -_PI / 2.0 if m & 1 else _PI / 2.0
    if (m & 2) and ((iy + (64 << 20)) & 0xffffffff) < ix:
        z = 0.0
    else:
        z = libm_atan(abs(y / x))
    if m == 0:
        return z
    if m == 1:
        return -z
    if m == 2:
        return _PI - (z - _PI_LO)
    return (z - _PI_LO) - _PI


def _trunc_i64(v):
    """Rust の `f64 as i64` (0 へ丸める)"""
    return int(v)


class _Cf:
    def __init__(self):
        self.init = False
        self.roll = 0.0
        self.pitch = 0.0
        self.cal_n = 0.0
        self.gsum = [0.0, 0.0]

    def update(self, m, e_s, alpha, cal):
        ra = libm_atan2(m[1], m[2])
        pa = libm_atan2(-m[0], math.sqrt(m[1] * m[1] + m[2] * m[2]))
        if self.cal_n < cal:
            self.gsum[0] += m[3]
            self.gsum[1] += m[4]
            self.cal_n += 1.0
            self.roll = ra
            self.pitch = pa
            self.init = True
            return
        if self.cal_n > 0.0:
            bx, by = self.gsum[0] / self.cal_n, self.gsum[1] / self.cal_n
        else:
            bx, by = 0.0, 0.0
        if not self.init:
            self.roll = ra
            self.pitch = pa
            self.init = True
        else:
            self.roll = alpha * (self.roll + (m[3] - bx) * e_s) + (1.0 - alpha) * ra
            self.pitch = alpha * (self.pitch + (m[4] - by) * e_s) + (1.0 - alpha) * pa


def encode_band(src, step, vals):
    """枠つきの IFACE_BAND メッセージ1通 (count=1, per_point=len(vals))"""
    body = struct.pack('<BBBBIQQII', 2, KIND_IFACE_BAND, QTY_INERTIAL, 0, src, step, 0, 1, len(vals))
    body += struct.pack('<%dd' % len(vals), *vals)
    return struct.pack('<I', len(body)) + body


def split_frames(buf):
    """枠つきメッセージ列を (src, バイト列 (枠を含む)) の並びに分ける"""
    out = []
    pos = 0
    while pos + 4 <= len(buf):
        (mlen,) = struct.unpack_from('<I', buf, pos)
        if mlen == 0:
            break
        msg = buf[pos:pos + 4 + mlen]
        (src,) = struct.unpack_from('<I', msg, 8)
        out.append((src, msg))
        pos += 4 + mlen
    return out


class ImuFusionRef:
    """crates/imu-fusion-actor/src/lib.rs の Fusion と同じ動作 (Python の float は IEEE 倍精度)"""

    def __init__(self):
        self.kind = [0] * 4
        self.iface = [0] * 4
        self.cfg_set = {}
        self.ready = False

    def cfg(self, key, val):
        if 0 <= key < 7:
            self.cfg_set[key] = float(val)

    def _cfg_get(self, key):
        return self.cfg_set.get(key, CFG_DEFAULT[key])

    def set_face(self, face, kind, param):
        if 0 <= face < 4:
            self.kind[face] = kind
            self.iface[face] = param & 0xFFFFFFFF

    def init(self, dt):
        m = self._cfg_get(CFG_EPOCH_STEPS)
        if not (dt > 0.0) or not (m >= 1.0) or m > 1.0e6:
            self.ready = False
            return -1
        self.m = int(m)
        self.dt = dt
        self.dt_us = _trunc_i64(dt * 1.0e6 + 0.5)
        if self.dt_us < 1:
            self.ready = False
            return -1
        self.e_us = self.dt_us * self.m
        self.e_s = float(self.m) * dt
        self.t0 = _trunc_i64(self._cfg_get(CFG_T0_US))
        self.offset = _trunc_i64(self._cfg_get(CFG_TS_OFFSET_US))
        self.mode = int(self._cfg_get(CFG_TIME_MODE))
        self.alpha = self._cfg_get(CFG_CF_ALPHA)
        self.cal = self._cfg_get(CFG_GYRO_CAL_EPOCHS)
        self.deg = float(sum(1 for f in range(4) if self.kind[f] == BC_INTERFACE))
        self.step_no = 0
        self.acc = [[-1, [0.0] * XLEN] for _ in range(RING)]
        self.x = [0.0] * XLEN
        self.cur_epoch = -1
        self.cur_local = [0.0] * XLEN
        self.cons = int(self._cfg_get(CFG_CONSENSUS))
        self.cur_n = 0.0
        self.ghost = [[0.0] * PER_POINT for _ in range(4)]
        self.got = [False] * 4
        self.out_epoch = -1
        self.fused_n = 0.0
        self.fused = [0.0] * 6
        self.local_n = 0.0
        self.local = [0.0] * 6
        self.cf_fused = _Cf()
        self.cf_local = _Cf()
        self.late = self.early = self.missing = self.dropped = self.rejected = 0
        self.ready = True
        return 0

    def push(self, t_us, v):
        if not self.ready:
            return PUSH_NOT_READY
        open_e = self.step_no // self.m
        if self.mode == TIME_ARRIVAL:
            e = open_e
        else:
            e = (t_us + self.offset - self.t0 - 1) // self.e_us
        if e < open_e:
            self.late += 1
            return PUSH_LATE
        if e >= open_e + RING:
            self.early += 1
            return PUSH_EARLY
        slot = self.acc[e % RING]
        if slot[0] != e:
            slot[0] = e
            slot[1] = [0.0] * XLEN
        x = slot[1]
        x[0] += 1.0
        for k in range(6):
            x[k + 1] += v[k]
        return PUSH_OK

    def need_until(self):
        if not self.ready or self.mode == TIME_ARRIVAL or (self.step_no + 1) % self.m != 0:
            return 0
        return self.t0 + (self.step_no + 1) * self.dt_us - self.offset

    def outbox(self):
        vals = [self.deg] + self.x
        out = b''
        for f in range(4):
            if self.kind[f] == BC_INTERFACE:
                out += encode_band(self.iface[f], self.step_no, vals)
        return out

    def inbox(self, buf):
        res = INBOX_OK
        for src, msg in split_frames(buf):
            ver, kind, qty, _r, src2, step, _t, count, pp = struct.unpack_from('<BBBBIQQII', msg, 4)
            face = next((f for f in range(4) if self.kind[f] == BC_INTERFACE and self.iface[f] == src2), None)
            if kind != KIND_IFACE_BAND or face is None or step != self.step_no or count != 1 or pp != PER_POINT:
                self.rejected += 1
                res = INBOX_REJECTED
                continue
            self.ghost[face] = list(struct.unpack_from('<8d', msg, 4 + 32))
            self.got[face] = True
        return res

    def step(self):
        rc = STEP_OK
        delta = [0.0] * XLEN
        for f in range(4):
            if self.kind[f] != BC_INTERFACE:
                continue
            if not self.got[f]:
                self.missing += 1
                rc = STEP_MISSING_GHOST
                continue
            g = self.ghost[f]
            dj = g[0]
            dmax = self.deg if self.deg > dj else dj
            w = 1.0 / (1.0 + dmax)
            for k in range(XLEN):
                delta[k] += w * (g[k + 1] - self.x[k])
        for k in range(XLEN):
            self.x[k] = self.x[k] + delta[k]
        self.got = [False] * 4
        self.step_no += 1
        if self.step_no % self.m == 0:
            q = self.step_no // self.m
            if self.cur_epoch >= 0:
                self.out_epoch = self.cur_epoch
                self.fused_n = self.x[0]
                if self.x[0] > 0.0:
                    self.fused = [self.x[k + 1] / self.x[0] for k in range(6)]
                    self.cf_fused.update(self.fused, self.e_s, self.alpha, self.cal)
                self.local_n = self.cur_n
                if self.cur_local[0] > 0.0:
                    self.local = [self.cur_local[k + 1] / self.cur_local[0] for k in range(6)]
                    self.cf_local.update(self.local, self.e_s, self.alpha, self.cal)
            e = q - 1
            slot = self.acc[e % RING]
            zs = list(slot[1]) if slot[0] == e else [0.0] * XLEN
            z = [0.0] * XLEN
            if zs[0] > 0.0:
                z[0] = 1.0
                for k in range(6):
                    z[k + 1] = zs[k + 1] / zs[0]
            self.acc[e % RING] = [-1, [0.0] * XLEN]
            if self.cons == CONS_DYNAMIC:
                self.x = [self.x[k] + (z[k] - self.cur_local[k]) for k in range(XLEN)]
            else:
                self.x = list(z)
            self.cur_local = list(z)
            self.cur_n = zs[0]
            self.cur_epoch = e
        return rc

    def get_all(self):
        return ([float(self.out_epoch), self.fused_n] + list(self.fused) +
                [self.cf_fused.roll, self.cf_fused.pitch, self.local_n] + list(self.local) +
                [self.cf_local.roll, self.cf_local.pitch, float(self.late), float(self.early), float(self.missing)])


class ImuFusionWasm:
    """実物の imu_fusion_actor.wasm を wasmtime で動かす (ImuFusionRef と同じメソッド)"""

    _engine = None
    _module = None

    def __init__(self, path=WASM):
        import wasmtime
        if ImuFusionWasm._module is None:
            ImuFusionWasm._engine = wasmtime.Engine()
            ImuFusionWasm._module = wasmtime.Module.from_file(ImuFusionWasm._engine, path)
        self.store = wasmtime.Store(ImuFusionWasm._engine)
        inst = wasmtime.Instance(self.store, ImuFusionWasm._module, [])
        ex = inst.exports(self.store)
        self.mem = ex['memory']
        self.f = {k: ex[k] for k in ('dyne_cfg', 'dyne_set_face', 'dyne_init2d', 'dyne_imu_push',
                                     'dyne_imu_need_until_us', 'dyne_msg_buf', 'dyne_msg_cap', 'dyne_outbox_read',
                                     'dyne_inbox_write', 'dyne_step', 'dyne_get')}
        self.buf = self.f['dyne_msg_buf'](self.store)
        self.cap = self.f['dyne_msg_cap'](self.store)

    def cfg(self, key, val):
        self.f['dyne_cfg'](self.store, key, float(val))

    def set_face(self, face, kind, param):
        self.f['dyne_set_face'](self.store, face, kind, param)

    def init(self, dt):
        return self.f['dyne_init2d'](self.store, 1, 1, 0, 0, 0.0, 0.0, dt)

    def push(self, t_us, v):
        return self.f['dyne_imu_push'](self.store, int(t_us), *[float(a) for a in v])

    def need_until(self):
        return self.f['dyne_imu_need_until_us'](self.store)

    def outbox(self):
        n = self.f['dyne_outbox_read'](self.store, self.buf, self.cap)
        return bytes(self.mem.read(self.store, self.buf, self.buf + n))

    def inbox(self, data):
        self.mem.write(self.store, data, self.buf)
        return self.f['dyne_inbox_write'](self.store, self.buf, len(data))

    def step(self):
        return self.f['dyne_step'](self.store)

    def get_all(self):
        g = self.f['dyne_get']
        return [g(self.store, k) for k in range(G_LEN)]


# ---- つながり方 (隣の関係) ----

def topology(kind, n, rows=None):
    """[(ノードa, 面a, ノードb, 面b)] の並び。界面IDは並びの番号+1。
    kind = 'line' (直線)、'ring' (環)、'grid' (rows×(n/rows) の格子)"""
    links = []
    if n <= 1:
        return links
    if kind in ('line', 'ring'):
        for i in range(n - 1):
            links.append((i, FACE_E, i + 1, FACE_W))
        if kind == 'ring' and n >= 3:
            links.append((n - 1, FACE_E, 0, FACE_W))
    elif kind == 'grid':
        r = rows or 2
        c = n // r
        assert r * c == n
        for y in range(r):
            for x in range(c):
                i = y * c + x
                if x + 1 < c:
                    links.append((i, FACE_E, i + 1, FACE_W))
                if y + 1 < r:
                    links.append((i, FACE_N, i + c, FACE_S))
    else:
        raise ValueError(kind)
    return links


def metropolis_matrix(n, links):
    """メトロポリスの重みの行列 W (全ノードの x の更新 x ← W x)。収束の速さの見積もり用"""
    import numpy as np
    deg = [0] * n
    for a, _, b, _ in links:
        deg[a] += 1
        deg[b] += 1
    W = np.zeros((n, n))
    for a, _, b, _ in links:
        w = 1.0 / (1.0 + max(deg[a], deg[b]))
        W[a, b] += w
        W[b, a] += w
    for i in range(n):
        W[i, i] = 1.0 - W[i].sum()
    return W


def make_nodes(impl, n, links, cfg, dt):
    nodes = [ImuFusionRef() if impl == 'ref' else ImuFusionWasm() for _ in range(n)]
    for li, (a, fa, b, fb) in enumerate(links):
        nodes[a].set_face(fa, BC_INTERFACE, li + 1)
        nodes[b].set_face(fb, BC_INTERFACE, li + 1)
    for i, nd in enumerate(nodes):
        for k, v in cfg(i).items() if callable(cfg) else cfg.items():
            nd.cfg(k, v)
        assert nd.init(dt) == 0
    return nodes


def run_network(impl, n, links, cfg, dt, steps, pushes, record_every=None, check_msgs=None):
    """pushes[i] = [(呼び出し番号c, t_us, (ax..gz)), ...] (c の昇順)。呼び出し c の直前に入れる。
    記録: 区間の境目ごと (record_every = M) に全ノードの get_all。戻り値 {step: [[22値]*n]}"""
    nodes = make_nodes(impl, n, links, cfg, dt)
    peer = {}
    for li, (a, fa, b, fb) in enumerate(links):
        peer[(a, li + 1)] = b
        peer[(b, li + 1)] = a
    ptr = [0] * n
    rec = {}
    msglog = [] if check_msgs is not None else None
    for c in range(steps):
        for i, nd in enumerate(nodes):
            lst = pushes[i]
            p = ptr[i]
            while p < len(lst) and lst[p][0] <= c:
                nd.push(lst[p][1], lst[p][2])
                p += 1
            ptr[i] = p
        inbox = [b''] * n
        for i, nd in enumerate(nodes):
            out = nd.outbox()
            if msglog is not None and c < check_msgs:
                msglog.append(out)
            for src, msg in split_frames(out):
                j = peer[(i, src)]
                inbox[j] += msg
        for i, nd in enumerate(nodes):
            if inbox[i]:
                nd.inbox(inbox[i])
        for nd in nodes:
            nd.step()
        if record_every and (c + 1) % record_every == 0:
            rec[c + 1] = [nd.get_all() for nd in nodes]
    return rec, msglog

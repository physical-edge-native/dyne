//! imu-fusion-actor: IMU (加速度と角速度のセンサ) の値を、隣との通信だけで融合する WASMアクター
//! (2026-11-13)
//!
//! 設計と検証の記録: docs/2026-11-13-imu-fusion.md。差込口は 2次元の場のアクター
//! (heat2d-actor, wave2d-actor) と同じ一式 (dyne_cfg → dyne_set_face → dyne_init2d、
//! 送信/受信メールボックス、dyne_step、dyne_len/dyne_get) なので、実機の新経路
//! (node-esp32/main/node_v2.c) と同じ配送の規則でそのまま動く。足したのは2つ:
//!   - dyne_imu_push(t_us, ax, ay, az, gx, gy, gz): 自分の IMU の標本を1つ入れる
//!   - dyne_imu_need_until_us(): 次の dyne_step の前に、どの時刻までの標本を入れ終えるべきか
//!
//! ## 何をするか
//! 1. 区間 (時間窓。長さ E = M·dt) ごとに、自分の標本の数 n と和 Σa, Σg を集める
//!    (区間が閉じたら自分の平均にする。融合はノードごとに等しい重み)。
//!    区間への割り当ては2通り (dyne_cfg の CFG_TIME_MODE):
//!    - TIME_STAMP: 時刻印 t (+ CFG_TS_OFFSET_US) で決める。区間 e = (t0 + e·E, t0 + (e+1)·E]
//!    - TIME_ARRIVAL: 時刻印を使わない。前の区間を閉じた後に届いた標本は全て今の区間
//! 2. 区間 e が閉じたら (ステップ (e+1)·M)、自分の z = [1, 平均 ax, ay, az, gx, gy, gz]
//!    (標本が無ければ全て0) を
//!    合意の状態 x に入れ (CONS_RESET: x = z。CONS_DYNAMIC (既定): x += z − 前の区間の z)、
//!    合意平均 (隣どうしの平均を繰り返す方法) を M 回反復する。重みはメトロポリスの重み
//!    w_ij = 1 / (1 + max(d_i, d_j)) (d = 隣の数)。重みが対称なので全ノードの x の和は保たれ、
//!    各ノードの x は全ノードの平均に近づく。融合した平均 = (Σ の平均) / (n の平均)
//!    = 全ノードの標本の総和 / 総数 (標本数の重みつき平均)。
//! 3. ステップ (e+2)·M で区間 e の融合を確定し、相補フィルタで傾き (ロール・ピッチ) を更新する。
//!    比較用に、自分だけの平均と自分だけの相補フィルタも同じ区間について持つ。
//!
//! ## 計算の順序 (Python の参照モデル scripts/imu_sim.py の ImuFusionRef とビット単位で揃える)
//! - 標本の和: 入った順に n += 1.0、s[k] += v[k]
//! - 区間の境目: CONS_DYNAMIC は x[k] = x[k] + (z[k] − z_prev[k])
//! - 合意の反復: δ[k] = Σ_{面 W,E,S,N の順} w·(g[k] − x[k])、x[k] = x[k] + δ[k]
//! - 平均: x[k+1] / x[0]
//! - 相補フィルタ: 加速度からの角度 roll_a = atan2(ay, az)、pitch_a = atan2(−ax, √(ay²+az²))。
//!   最初は角度 = 加速度からの角度。以降 roll = α·(roll + (gx − bx)·E) + (1−α)·roll_a (pitch は gy)。
//!   bx = 最初の CFG_GYRO_CAL_EPOCHS 区間の gx の平均 (較正しないとき 0.0。x − 0.0 = x なので較正なしと同じ値)
//!   (atan2 は libm と Python の math.atan2 で最後の1桁がずれうる。照合は角度だけ許容差つき)
//!
//! no_std / 静的確保 / libm (純Rust) のみ依存。同一入力で全出力がビット一致する。

#![cfg_attr(target_arch = "wasm32", no_std)]

use dyne_wire as wire;
use dyne_wire::abi::field2d::{BC_INTERFACE, INBOX_MALFORMED, INBOX_OK, INBOX_REJECTED, STEP_MISSING_GHOST, STEP_OK};
use dyne_wire::abi::imu::*;

/// 区間のため置きの数 (今の区間から先へ R 区間ぶん受け付ける)
const RING: usize = 4;
/// 合意の状態の量の数 (n と6つの和)
const XLEN: usize = 7;
const PP: usize = PER_POINT as usize;
/// 4面 × (枠4 + ヘッダ24 + 8 + 8·8 = 100) + 終端印4 = 404
const SCRATCH_CAP: usize = 512;

/// dyne_cfg の設定 (既定値は読み出し時に補う。静的領域を全て0の初期値に保つため)
#[derive(Clone, Copy)]
struct Cfg {
    set: u32,
    v: [f64; 7],
}

impl Cfg {
    const ZERO: Cfg = Cfg { set: 0, v: [0.0; 7] };
    const DEFAULT: [f64; 7] = [10.0, TIME_STAMP as f64, 0.0, 0.0, 0.9, 0.0, CONS_DYNAMIC as f64];
    fn get(&self, key: u32) -> f64 {
        let k = key as usize;
        if (self.set >> k) & 1 == 1 {
            self.v[k]
        } else {
            Self::DEFAULT[k]
        }
    }
}

/// 区間1つぶんの標本の和
#[derive(Clone, Copy)]
struct Acc {
    /// この枠が持っている区間の番号 (使っていなければ −1)
    tag: i64,
    x: [f64; XLEN],
}

impl Acc {
    const EMPTY: Acc = Acc { tag: -1, x: [0.0; XLEN] };
}

/// 相補フィルタの状態。角速度のずれ (ゼロ点) の較正つき: 最初の cal 区間は板が静止している
/// 前提で角速度の平均を取り (その間の角度は加速度からの角度)、以降はそれを引いて積分する
#[derive(Clone, Copy)]
struct Cf {
    init: bool,
    roll: f64,
    pitch: f64,
    cal_n: f64,
    gsum: [f64; 2],
}

impl Cf {
    const ZERO: Cf = Cf { init: false, roll: 0.0, pitch: 0.0, cal_n: 0.0, gsum: [0.0; 2] };
    fn update(&mut self, m: &[f64; 6], e_s: f64, alpha: f64, cal: f64) {
        let ra = libm::atan2(m[1], m[2]);
        let pa = libm::atan2(-m[0], libm::sqrt(m[1] * m[1] + m[2] * m[2]));
        if self.cal_n < cal {
            self.gsum[0] += m[3];
            self.gsum[1] += m[4];
            self.cal_n += 1.0;
            self.roll = ra;
            self.pitch = pa;
            self.init = true;
            return;
        }
        let (bx, by) = if self.cal_n > 0.0 { (self.gsum[0] / self.cal_n, self.gsum[1] / self.cal_n) } else { (0.0, 0.0) };
        if !self.init {
            self.roll = ra;
            self.pitch = pa;
            self.init = true;
        } else {
            self.roll = alpha * (self.roll + (m[3] - bx) * e_s) + (1.0 - alpha) * ra;
            self.pitch = alpha * (self.pitch + (m[4] - by) * e_s) + (1.0 - alpha) * pa;
        }
    }
}

pub struct Fusion {
    ready: bool,
    /// 面ごとの種類 (BC_*) と界面ID
    kind: [u32; 4],
    iface: [u32; 4],
    cfg: Cfg,
    // init で決まる量
    m: u64,
    dt: f64,
    dt_us: i64,
    e_us: i64,
    e_s: f64,
    t0: i64,
    offset: i64,
    mode: u32,
    alpha: f64,
    cal: f64,
    deg: f64,
    // 状態
    step: u64,
    acc: [Acc; RING],
    /// 合意の状態 (x[0] = n の平均、x[1..7] = 和の平均)
    x: [f64; XLEN],
    /// 合意の途中の区間の番号 (−1 = なし) と、その区間の自分だけの和
    cur_epoch: i64,
    cur_local: [f64; XLEN],
    /// 合意の始め方 (CONS_*)
    cons: u32,
    /// 合意の途中の区間の自分の標本数
    cur_n: f64,
    ghost: [[f64; PP]; 4],
    got: [bool; 4],
    // 出力 (最後に確定した区間)
    out_epoch: i64,
    fused_n: f64,
    fused: [f64; 6],
    local_n: f64,
    local: [f64; 6],
    cf_fused: Cf,
    cf_local: Cf,
    // 計数
    late: u64,
    early: u64,
    missing: u64,
    dropped: u64,
    rejected: u64,
    scratch: [u8; SCRATCH_CAP],
    band: [u8; 8 * PP],
}

static mut S: Fusion = Fusion {
    ready: false,
    kind: [0; 4],
    iface: [0; 4],
    cfg: Cfg::ZERO,
    m: 0,
    dt: 0.0,
    dt_us: 0,
    e_us: 0,
    e_s: 0.0,
    t0: 0,
    offset: 0,
    mode: 0,
    alpha: 0.0,
    cal: 0.0,
    deg: 0.0,
    step: 0,
    acc: [Acc::EMPTY; RING],
    x: [0.0; XLEN],
    cur_epoch: -1,
    cur_local: [0.0; XLEN],
    cons: 0,
    cur_n: 0.0,
    ghost: [[0.0; PP]; 4],
    got: [false; 4],
    out_epoch: -1,
    fused_n: 0.0,
    fused: [0.0; 6],
    local_n: 0.0,
    local: [0.0; 6],
    cf_fused: Cf::ZERO,
    cf_local: Cf::ZERO,
    late: 0,
    early: 0,
    missing: 0,
    dropped: 0,
    rejected: 0,
    scratch: [0; SCRATCH_CAP],
    band: [0; 8 * PP],
};

#[allow(static_mut_refs)]
fn state() -> &'static mut Fusion {
    // 単一スレッド (WASM の1インスタンス) からのみ呼ばれる
    unsafe { &mut *core::ptr::addr_of_mut!(S) }
}

/// f64 の整数値を i64 へ (小数部は 0 へ丸める)。µs の設定値用
fn f2i(v: f64) -> i64 {
    v as i64
}

impl Fusion {
    fn set_face(&mut self, face: u32, kind: u32, param: u64) {
        if face < 4 {
            self.kind[face as usize] = kind;
            self.iface[face as usize] = param as u32;
        }
    }

    fn cfg(&mut self, key: u32, val: f64) {
        if (key as usize) < self.cfg.v.len() {
            self.cfg.v[key as usize] = val;
            self.cfg.set |= 1 << key;
        }
    }

    fn init(&mut self, dt: f64) -> i32 {
        let m = self.cfg.get(CFG_EPOCH_STEPS);
        if !(dt > 0.0) || !(m >= 1.0) || m > 1.0e6 {
            self.ready = false;
            return -1;
        }
        self.m = m as u64;
        self.dt = dt;
        self.dt_us = f2i(dt * 1.0e6 + 0.5);
        if self.dt_us < 1 {
            self.ready = false;
            return -1;
        }
        self.e_us = self.dt_us * self.m as i64;
        self.e_s = (self.m as f64) * dt;
        self.t0 = f2i(self.cfg.get(CFG_T0_US));
        self.offset = f2i(self.cfg.get(CFG_TS_OFFSET_US));
        self.mode = self.cfg.get(CFG_TIME_MODE) as u32;
        self.alpha = self.cfg.get(CFG_CF_ALPHA);
        self.cal = self.cfg.get(CFG_GYRO_CAL_EPOCHS);
        self.deg = (0..4).filter(|&f| self.kind[f] == BC_INTERFACE).count() as f64;
        self.step = 0;
        self.acc = [Acc::EMPTY; RING];
        self.x = [0.0; XLEN];
        self.cur_epoch = -1;
        self.cur_local = [0.0; XLEN];
        self.cons = self.cfg.get(CFG_CONSENSUS) as u32;
        self.cur_n = 0.0;
        self.ghost = [[0.0; PP]; 4];
        self.got = [false; 4];
        self.out_epoch = -1;
        self.fused_n = 0.0;
        self.fused = [0.0; 6];
        self.local_n = 0.0;
        self.local = [0.0; 6];
        self.cf_fused = Cf::ZERO;
        self.cf_local = Cf::ZERO;
        self.late = 0;
        self.early = 0;
        self.missing = 0;
        self.dropped = 0;
        self.rejected = 0;
        self.ready = true;
        0
    }

    /// 今標本を受け付けている区間 (ステップ e·M .. (e+1)·M−1 の間は区間 e)
    fn open_epoch(&self) -> i64 {
        (self.step / self.m) as i64
    }

    fn push(&mut self, t_us: i64, v: [f64; 6]) -> i32 {
        if !self.ready {
            return PUSH_NOT_READY;
        }
        let open = self.open_epoch();
        let e = if self.mode == TIME_ARRIVAL {
            open
        } else {
            // 区間 e = (t0 + e·E, t0 + (e+1)·E] → e = floor((t − t0 − 1) / E)
            (t_us.wrapping_add(self.offset) - self.t0 - 1).div_euclid(self.e_us)
        };
        if e < open {
            self.late += 1;
            return PUSH_LATE;
        }
        if e >= open + RING as i64 {
            self.early += 1;
            return PUSH_EARLY;
        }
        let slot = &mut self.acc[(e as usize) % RING];
        if slot.tag != e {
            *slot = Acc { tag: e, x: [0.0; XLEN] };
        }
        slot.x[0] += 1.0;
        for k in 0..6 {
            slot.x[k + 1] += v[k];
        }
        PUSH_OK
    }

    /// 次の dyne_step の前に入れ終えるべき標本の時刻 [µs] (TIME_STAMP のとき、次のステップで
    /// 区間が閉じるならその終わりの時刻。閉じないなら 0 = 待たなくてよい)。時刻印のずれ
    /// (CFG_TS_OFFSET_US) を足す前の、ノードの時計の値で返す
    fn need_until(&self) -> i64 {
        if !self.ready || self.mode == TIME_ARRIVAL || (self.step + 1) % self.m != 0 {
            return 0;
        }
        self.t0 + (self.step as i64 + 1) * self.dt_us - self.offset
    }

    fn outbox_read(&mut self, cap: usize) -> usize {
        let cap = cap.min(SCRATCH_CAP);
        let mut used = 0usize;
        let mut dropped = 0u64;
        self.band[0..8].copy_from_slice(&self.deg.to_bits().to_le_bytes());
        for k in 0..XLEN {
            self.band[8 * (k + 1)..8 * (k + 2)].copy_from_slice(&self.x[k].to_bits().to_le_bytes());
        }
        for face in 0..4 {
            if self.kind[face] != BC_INTERFACE {
                continue;
            }
            let hdr = wire::Header {
                kind: wire::KIND_IFACE_BAND,
                quantity: wire::QTY_INERTIAL,
                src: self.iface[face],
                step: self.step,
                t_us: 0,
            };
            let payload = wire::Payload::IfaceBand { data: &self.band, count: 1, per_point: PER_POINT };
            let mut w = wire::MailboxWriter::new(&mut self.scratch[used..cap]);
            let _ = w.push(&hdr, &payload);
            used += w.used();
            dropped += w.dropped;
        }
        if used + 4 <= cap {
            self.scratch[used..used + 4].copy_from_slice(&0u32.to_le_bytes());
        }
        self.dropped += dropped;
        used
    }

    fn inbox_write(&mut self, len: usize) -> i32 {
        let len = len.min(SCRATCH_CAP);
        let mut result = INBOX_OK;
        let mut pos = 0usize;
        while pos + 4 <= len {
            let mlen = u32::from_le_bytes(self.scratch[pos..pos + 4].try_into().unwrap()) as usize;
            if mlen == 0 {
                break;
            }
            if pos + 4 + mlen > len {
                return INBOX_MALFORMED;
            }
            let raw = pos + 4..pos + 4 + mlen;
            pos += 4 + mlen;
            let (hdr, payload) = match wire::decode(&self.scratch[raw]) {
                Ok((h, p, used)) if used == mlen => (h, p),
                _ => return INBOX_MALFORMED,
            };
            let wire::Payload::IfaceBand { data, count, per_point } = payload else {
                self.rejected += 1;
                result = INBOX_REJECTED;
                continue;
            };
            let face = (0..4).find(|&f| self.kind[f] == BC_INTERFACE && self.iface[f] == hdr.src);
            let ok = face.is_some() && hdr.step == self.step && count == 1 && per_point == PER_POINT;
            if !ok {
                self.rejected += 1;
                result = INBOX_REJECTED;
                continue;
            }
            let f = face.unwrap();
            for k in 0..PP {
                self.ghost[f][k] = wire::band_get(data, k);
            }
            self.got[f] = true;
        }
        result
    }

    fn step(&mut self) -> i32 {
        if !self.ready {
            return -1;
        }
        // 1. 合意の1反復 (メトロポリスの重み)
        let mut rc = STEP_OK;
        let mut delta = [0.0f64; XLEN];
        for face in 0..4 {
            if self.kind[face] != BC_INTERFACE {
                continue;
            }
            if !self.got[face] {
                // 届かなかった隣は、この反復では重み0 (和は保たれなくなる。計数して知らせる)
                self.missing += 1;
                rc = STEP_MISSING_GHOST;
                continue;
            }
            let g = &self.ghost[face];
            let dj = g[0];
            let dmax = if self.deg > dj { self.deg } else { dj };
            let w = 1.0 / (1.0 + dmax);
            for k in 0..XLEN {
                delta[k] += w * (g[k + 1] - self.x[k]);
            }
        }
        for k in 0..XLEN {
            self.x[k] = self.x[k] + delta[k];
        }
        self.got = [false; 4];
        self.step += 1;

        // 2. 区間の境目: 合意を終えた区間を確定し、閉じた区間の和で次の合意を始める
        if self.step % self.m == 0 {
            let q = (self.step / self.m) as i64;
            if self.cur_epoch >= 0 {
                self.out_epoch = self.cur_epoch;
                self.fused_n = self.x[0];
                if self.x[0] > 0.0 {
                    for k in 0..6 {
                        self.fused[k] = self.x[k + 1] / self.x[0];
                    }
                    let f = self.fused;
                    self.cf_fused.update(&f, self.e_s, self.alpha, self.cal);
                }
                self.local_n = self.cur_n;
                if self.cur_local[0] > 0.0 {
                    for k in 0..6 {
                        self.local[k] = self.cur_local[k + 1] / self.cur_local[0];
                    }
                    let l = self.local;
                    self.cf_local.update(&l, self.e_s, self.alpha, self.cal);
                }
            }
            let e = q - 1;
            let slot = &mut self.acc[(e as usize) % RING];
            let zs = if slot.tag == e { slot.x } else { [0.0; XLEN] };
            // ノードごとに等しい重み: z = [1, 自分の平均 (6)] (標本が無ければ全て0)。標本数で重みを
            // つけると、区間ごとの標本数の揺れ (20個/21個) とノードごとのゼロ点のずれの差が掛け合わさって
            // 融合した値が揺れる (シミュレーションで 8台の短時間の標準偏差が 1/√N より大きく出た原因)
            let mut z = [0.0; XLEN];
            if zs[0] > 0.0 {
                z[0] = 1.0;
                for k in 0..6 {
                    z[k + 1] = zs[k + 1] / zs[0];
                }
            }
            *slot = Acc::EMPTY;
            if self.cons == CONS_DYNAMIC {
                // 動的合意平均: 前の区間の自分の和との差だけを足す (最初の区間は前の和 = 0)
                for k in 0..XLEN {
                    self.x[k] = self.x[k] + (z[k] - self.cur_local[k]);
                }
            } else {
                self.x = z;
            }
            self.cur_local = z;
            self.cur_n = zs[0];
            self.cur_epoch = e;
        }
        rc
    }

    fn get(&self, k: u32) -> f64 {
        match k {
            G_EPOCH => self.out_epoch as f64,
            G_FUSED_N => self.fused_n,
            k if (G_FUSED..G_FUSED + 6).contains(&k) => self.fused[(k - G_FUSED) as usize],
            G_FUSED_ROLL => self.cf_fused.roll,
            G_FUSED_PITCH => self.cf_fused.pitch,
            G_LOCAL_N => self.local_n,
            k if (G_LOCAL..G_LOCAL + 6).contains(&k) => self.local[(k - G_LOCAL) as usize],
            G_LOCAL_ROLL => self.cf_local.roll,
            G_LOCAL_PITCH => self.cf_local.pitch,
            G_LATE => self.late as f64,
            G_EARLY => self.early as f64,
            G_MISSING => self.missing as f64,
            _ => 0.0,
        }
    }
}

// ---------- 差込口 (エクスポート) ----------

#[no_mangle]
pub extern "C" fn dyne_cfg(key: u32, val: f64) {
    state().cfg(key, val);
}

/// 面 (隣) の設定。kind = BC_INTERFACE の面が隣で、param の下位32ビットが界面ID。
/// それ以外の種類の面は無視する (隣なし)
#[no_mangle]
pub extern "C" fn dyne_set_face(face: u32, kind: u32, param: u64) {
    state().set_face(face, kind, param);
}

/// 2次元の場のアクターと同じ形の初期化。使うのは dt (合意の1反復の周期 [s]) だけ。
/// nx, ny, ix0, iy0, h, alpha は無視する。戻り値 0 = 成功、−1 = 設定が不正
#[no_mangle]
pub extern "C" fn dyne_init2d(_nx: u32, _ny: u32, _ix0: u32, _iy0: u32, _h: f64, _alpha: f64, dt: f64) -> i32 {
    state().init(dt)
}

/// IMU の標本を1つ入れる。t_us = 取得時刻 [µs] (同期した時刻)、加速度 [m/s²]、角速度 [rad/s]。
/// 戻り値は PUSH_*
#[no_mangle]
pub extern "C" fn dyne_imu_push(t_us: i64, ax: f64, ay: f64, az: f64, gx: f64, gy: f64, gz: f64) -> i32 {
    state().push(t_us, [ax, ay, az, gx, gy, gz])
}

#[no_mangle]
pub extern "C" fn dyne_imu_need_until_us() -> i64 {
    state().need_until()
}

#[no_mangle]
pub extern "C" fn dyne_msg_buf() -> u32 {
    core::ptr::addr_of!(state().scratch) as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_msg_cap() -> u32 {
    SCRATCH_CAP as u32
}

#[no_mangle]
pub extern "C" fn dyne_outbox_read(ptr: u32, cap: u32) -> u32 {
    let s = state();
    if ptr != core::ptr::addr_of!(s.scratch) as usize as u32 {
        return 0;
    }
    s.outbox_read(cap as usize) as u32
}

#[no_mangle]
pub extern "C" fn dyne_inbox_write(ptr: u32, len: u32) -> i32 {
    let s = state();
    if ptr != core::ptr::addr_of!(s.scratch) as usize as u32 || len as usize > SCRATCH_CAP {
        return INBOX_MALFORMED;
    }
    s.inbox_write(len as usize)
}

#[no_mangle]
pub extern "C" fn dyne_step() -> i32 {
    state().step()
}

#[no_mangle]
pub extern "C" fn dyne_len() -> u32 {
    G_LEN
}

#[no_mangle]
pub extern "C" fn dyne_get(k: u32) -> f64 {
    state().get(k)
}

#[no_mangle]
pub extern "C" fn dyne_steps_done() -> u64 {
    state().step
}

#[no_mangle]
pub extern "C" fn dyne_dropped() -> u64 {
    state().dropped
}

#[no_mangle]
pub extern "C" fn dyne_rejected() -> u64 {
    state().rejected
}

#[no_mangle]
pub extern "C" fn dyne_missing() -> u64 {
    state().missing
}

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    core::arch::wasm32::unreachable()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn new_node() -> Fusion {
        // 静的領域とは別に、試験ごとに新しい状態を作る
        let mut f = unsafe { core::ptr::read(core::ptr::addr_of!(S)) };
        f.ready = false;
        f
    }

    /// 1台 (隣なし) なら、融合 = 自分の平均。区間の割り当てと確定の時点を確かめる
    #[test]
    fn single_node_epochs() {
        let mut f = new_node();
        f.cfg(CFG_EPOCH_STEPS, 2.0);
        assert_eq!(f.init(0.01), 0); // dt = 10ms、区間 20ms
        // 区間0 = (0, 20000]、区間1 = (20000, 40000]
        assert_eq!(f.push(10_000, [1.0, 0.0, 9.0, 0.0, 0.0, 0.0]), PUSH_OK);
        assert_eq!(f.push(20_000, [3.0, 0.0, 9.0, 0.0, 0.0, 0.0]), PUSH_OK);
        assert_eq!(f.push(20_001, [7.0, 0.0, 9.0, 0.0, 0.0, 0.0]), PUSH_OK);
        assert_eq!(f.push(200_001, [0.0; 6]), PUSH_EARLY);
        assert_eq!(f.need_until(), 0);
        f.step(); // step 1
        assert_eq!(f.need_until(), 20_000);
        f.step(); // step 2: 区間0を閉じて合意を始める
        assert_eq!(f.push(5_000, [0.0; 6]), PUSH_LATE);
        assert_eq!(f.get(G_EPOCH), -1.0);
        f.step();
        f.step(); // step 4: 区間0を確定
        assert_eq!(f.get(G_EPOCH), 0.0);
        assert_eq!(f.get(G_FUSED_N), 1.0);
        assert_eq!(f.get(G_LOCAL_N), 2.0);
        assert_eq!(f.get(G_FUSED), 2.0);
        assert_eq!(f.get(G_LOCAL), 2.0);
        f.step();
        f.step(); // step 6: 区間1を確定
        assert_eq!(f.get(G_EPOCH), 1.0);
        assert_eq!(f.get(G_FUSED), 7.0);
        assert_eq!(f.get(G_LATE), 1.0);
        assert_eq!(f.get(G_EARLY), 1.0);
    }

    /// 届いた順の割り当てでは、時刻印に関係なく今の区間に入る
    #[test]
    fn arrival_mode_ignores_timestamps() {
        let mut f = new_node();
        f.cfg(CFG_EPOCH_STEPS, 1.0);
        f.cfg(CFG_TIME_MODE, TIME_ARRIVAL as f64);
        assert_eq!(f.init(0.02), 0);
        assert_eq!(f.push(999_999_999, [4.0, 0.0, 1.0, 0.0, 0.0, 0.0]), PUSH_OK);
        f.step();
        f.step();
        assert_eq!(f.get(G_EPOCH), 0.0);
        assert_eq!(f.get(G_FUSED), 4.0);
    }

    /// 2台 (界面1本でつながる) の合意平均: 1反復で平均に一致する (重み 1/2)
    #[test]
    fn two_nodes_consensus() {
        let mut a = new_node();
        let mut b = new_node();
        for f in [&mut a, &mut b] {
            f.cfg(CFG_EPOCH_STEPS, 1.0);
        }
        a.set_face(1, BC_INTERFACE, 5); // 東の面
        b.set_face(0, BC_INTERFACE, 5); // 西の面
        assert_eq!(a.init(0.02), 0);
        assert_eq!(b.init(0.02), 0);
        a.push(1, [2.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        b.push(1, [4.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        b.push(2, [6.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let exchange = |a: &mut Fusion, b: &mut Fusion| {
            let la = a.outbox_read(SCRATCH_CAP);
            let lb = b.outbox_read(SCRATCH_CAP);
            let ma = a.scratch;
            let mb = b.scratch;
            b.scratch[..la].copy_from_slice(&ma[..la]);
            a.scratch[..lb].copy_from_slice(&mb[..lb]);
            assert_eq!(a.inbox_write(lb), INBOX_OK);
            assert_eq!(b.inbox_write(la), INBOX_OK);
            assert_eq!(a.step(), STEP_OK);
            assert_eq!(b.step(), STEP_OK);
        };
        exchange(&mut a, &mut b); // step 1: 区間0を閉じる
        exchange(&mut a, &mut b); // step 2: 1反復して区間0を確定
        // ノードごとの平均 2 と 5 を等しい重みで → 3.5
        assert_eq!(a.get(G_FUSED), 3.5);
        assert_eq!(b.get(G_FUSED), 3.5);
        assert_eq!(a.get(G_FUSED_N), 1.0);
        assert_eq!(b.get(G_LOCAL_N), 2.0);
        assert_eq!(a.get(G_LOCAL), 2.0);
        assert_eq!(b.get(G_LOCAL), 5.0);
    }

    /// 直線につないだ3台が毎区間ちがう一定値を入れる。動的合意平均では区間をまたいで混ざり続け、
    /// 全ノードの融合が全体の平均に収束する。区間ごとに置き直す方式では M=1 反復ぶんしか混ざらない
    fn line3(cons: u32, epochs: usize) -> [f64; 3] {
        let mut n: [Fusion; 3] = [new_node(), new_node(), new_node()];
        n[0].set_face(1, BC_INTERFACE, 1);
        n[1].set_face(0, BC_INTERFACE, 1);
        n[1].set_face(1, BC_INTERFACE, 2);
        n[2].set_face(0, BC_INTERFACE, 2);
        for f in n.iter_mut() {
            f.cfg(CFG_EPOCH_STEPS, 1.0);
            f.cfg(CFG_TIME_MODE, TIME_ARRIVAL as f64);
            f.cfg(CFG_CONSENSUS, cons as f64);
            assert_eq!(f.init(0.02), 0);
        }
        let val = [1.0, 2.0, 6.0];
        for _ in 0..epochs {
            for (i, f) in n.iter_mut().enumerate() {
                f.push(0, [val[i], 0.0, 1.0, 0.0, 0.0, 0.0]);
            }
            let mut boxes = [[0u8; SCRATCH_CAP]; 3];
            let mut lens = [0usize; 3];
            for i in 0..3 {
                lens[i] = n[i].outbox_read(SCRATCH_CAP);
                boxes[i] = n[i].scratch;
            }
            // 配送: 界面ID 1 は 0⇔1、2 は 1⇔2
            let mut inbox: [std::vec::Vec<u8>; 3] = Default::default();
            for i in 0..3 {
                let mut pos = 0;
                while pos < lens[i] {
                    let mlen = u32::from_le_bytes(boxes[i][pos..pos + 4].try_into().unwrap()) as usize;
                    let src = u32::from_le_bytes(boxes[i][pos + 8..pos + 12].try_into().unwrap());
                    let j = match (i, src) { (0, 1) => 1, (1, 1) => 0, (1, 2) => 2, (2, 2) => 1, _ => unreachable!() };
                    inbox[j].extend_from_slice(&boxes[i][pos..pos + 4 + mlen]);
                    pos += 4 + mlen;
                }
            }
            for j in 0..3 {
                n[j].scratch[..inbox[j].len()].copy_from_slice(&inbox[j]);
                assert_eq!(n[j].inbox_write(inbox[j].len()), INBOX_OK);
                assert_eq!(n[j].step(), STEP_OK);
            }
        }
        [n[0].get(G_FUSED), n[1].get(G_FUSED), n[2].get(G_FUSED)]
    }

    #[test]
    fn dynamic_consensus_removes_constant_disagreement() {
        let d = line3(CONS_DYNAMIC, 200);
        for v in d {
            assert!((v - 3.0).abs() < 1e-12, "{:?}", d);
        }
        let r = line3(CONS_RESET, 200);
        assert!((r[0] - 3.0).abs() > 0.1, "{:?}", r);
    }
}

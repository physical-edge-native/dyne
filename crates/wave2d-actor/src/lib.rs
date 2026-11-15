//! wave2d-actor: 2次元の波動方程式の区画を解く WASMアクター (2026-11-11)
//!
//! 設計と検証の記録: docs/2026-11-11-wave2d.md。差込口・メールボックス・ゴースト点の
//! 枠つき配列は heat2d-actor と同じ形で、dyne-sim2d と実機の新経路 (node-esp32/main/node_v2.c)
//! がそのまま動かせる。
//!
//! ## 数値法
//!   u_tt = c² (u_xx + u_yy)、正方格子 (刻み h は x, y 共通)、上下左右の4点の差分
//!   (5点ラプラシアン) と時間の中心差分 (3時刻の陽解法、いわゆるリープフロッグ):
//!     u^{n+1} = 2u^n − u^{n−1} + ν²·((u_W − 2u + u_E) + (u_S − 2u + u_N))、ν = c·dt/h
//!   最初の1ステップだけは初速度0のテイラー展開の式 u^1 = u^0 + (ν²/2)·(同じラプラシアン)。
//!   安定条件 (無限領域のフォン・ノイマンの方法) は ν ≤ 1/√2。アクターは検査しない
//!   (限界をまたぐ掃引に使うため。dyne_courant で ν を返す)。
//!   状態は2時刻ぶん (u^n と u^{n−1})。u^{n+1} は u^{n−1} の配列に点ごとに上書きする
//!   (u^{n−1} はその点でしか使わないので安全)。
//!
//! ## 界面
//!   更新式が隣を参照するのは現在の時刻 u^n だけ (u^{n−1} は自分の点のみ)。よって界面で
//!   受け渡すのは熱拡散と同じく「現在の状態の自分の端の値」1列だけで、ゴースト点は1層で足りる。
//!
//! ## 外周の境界 (面ごと)
//!   値固定 (BC_DIRICHLET)、ゼロ勾配 (BC_ZERO_GRAD。ゴースト点は鏡映)、吸収境界
//!   (dyne_cfg の CFG_ABSORB のビットで選ぶ。1次のエングクイスト・マイダ条件 u_t = ±c·u_n を
//!   ムーアの方法で離散化: 端の点 u_0^{n+1} = u_1^n + k·(u_1^{n+1} − u_0^n)、k = (ν−1)/(ν+1)。
//!   吸収境界の面は、まずゼロ勾配として全点を更新した後に端の点だけをこの式で上書きする)。
//!   適用順は面の順 W, E, S, N (吸収境界)、その後に値固定 (角は後の面が勝つ)。
//!
//! no_std / 静的確保 / libm (純Rust) のみ依存。同一入力で全出力がビット一致する。

#![cfg_attr(target_arch = "wasm32", no_std)]

use dyne_wire as wire;
use dyne_wire::abi::field2d::*;
use dyne_wire::abi::wave2d::*;

// ---------- 容量 ----------

/// 枠つき配列の点数の上限 ((nx+2)·(ny+2) ≤ PAD_MAX) と1辺の点数の上限。
/// フル版 (シミュレータ用): 2配列で 640KB。縮小版 (実機用、線形メモリ64KB): 区画は1辺48点まで
#[cfg(not(feature = "small"))]
const PAD_MAX: usize = 40_000;
#[cfg(not(feature = "small"))]
const SIDE_MAX: usize = 196;
#[cfg(not(feature = "small"))]
const SCRATCH_CAP: usize = 8192;

#[cfg(feature = "small")]
const PAD_MAX: usize = 50 * 50;
#[cfg(feature = "small")]
const SIDE_MAX: usize = 48;
/// 4面 × (枠4 + ヘッダ24 + 8 + 8·48 = 420) + 終端印4 が入る大きさ
#[cfg(feature = "small")]
const SCRATCH_CAP: usize = 2048;

const PI: f64 = core::f64::consts::PI;

/// dyne_cfg の設定。大きな静的領域 S を全て0の初期値に保つ (0以外の初期値があると
/// 状態の配列ごと WASM のデータ節に載り、ファイルが数百KBになる) ため、既定値は
/// 「設定されていなければ既定」として読み出し時に補う
#[derive(Clone, Copy)]
struct Cfg {
    set: u32,
    v: [f64; 8],
}

impl Cfg {
    const ZERO: Cfg = Cfg { set: 0, v: [0.0; 8] };
    const DEFAULT: [f64; 8] = [IC_SIN_MODE as f64, 1.0, 1.0, 1.0, 0.5, 0.5, 0.05, 0.0];
    fn get(&self, key: u32) -> f64 {
        let k = key as usize;
        if (self.set >> k) & 1 == 1 {
            self.v[k]
        } else {
            Self::DEFAULT[k]
        }
    }
}

pub struct Field {
    nx: usize,
    ny: usize,
    ix0: u32,
    iy0: u32,
    h: f64,
    c: f64,
    dt: f64,
    /// ν² と ν²/2 (init で一度だけ計算)
    nu2: f64,
    half_nu2: f64,
    /// ムーアの係数 (ν−1)/(ν+1)
    kmur: f64,
    /// 面ごとの種類 (BC_*)
    kind: [u32; 4],
    /// 面ごとの付帯値: 界面なら界面ID、値固定なら値 (f64 のビット列)
    param: [u64; 4],
    /// 設定 (dyne_cfg。init をまたいで保持)
    cfg: Cfg,
    /// 枠つき配列 (幅 nx+2、高さ ny+2)。u = u^n、p = u^{n−1} (ステップ中に u^{n+1} で上書き)
    u: [f64; PAD_MAX],
    p: [f64; PAD_MAX],
    ghost: [[f64; SIDE_MAX]; 4],
    got: [bool; 4],
    step: u64,
    scratch: [u8; SCRATCH_CAP],
    edge: [u8; 8 * SIDE_MAX],
    dropped: u64,
    rejected: u64,
    missing: u64,
}

static mut S: Field = Field {
    nx: 0,
    ny: 0,
    ix0: 0,
    iy0: 0,
    h: 0.0,
    c: 0.0,
    dt: 0.0,
    nu2: 0.0,
    half_nu2: 0.0,
    kmur: 0.0,
    kind: [BC_DIRICHLET; 4],
    param: [0; 4],
    cfg: Cfg::ZERO,
    u: [0.0; PAD_MAX],
    p: [0.0; PAD_MAX],
    ghost: [[0.0; SIDE_MAX]; 4],
    got: [false; 4],
    step: 0,
    scratch: [0; SCRATCH_CAP],
    edge: [0; 8 * SIDE_MAX],
    dropped: 0,
    rejected: 0,
    missing: 0,
};

fn state() -> &'static mut Field {
    unsafe { &mut *core::ptr::addr_of_mut!(S) }
}

// ---------- 内部ロジック ----------

impl Field {
    #[inline]
    fn w(&self) -> usize {
        self.nx + 2
    }

    #[inline]
    fn at(&self, i: isize, j: isize) -> usize {
        ((j + 1) as usize) * self.w() + (i + 1) as usize
    }

    fn face_len(&self, face: usize) -> usize {
        if face == FACE_W as usize || face == FACE_E as usize {
            self.ny
        } else {
            self.nx
        }
    }

    fn edge_point(&self, face: usize, k: usize) -> (isize, isize) {
        let (nx, ny) = (self.nx as isize, self.ny as isize);
        let k = k as isize;
        match face as u32 {
            FACE_W => (0, k),
            FACE_E => (nx - 1, k),
            FACE_S => (k, 0),
            _ => (k, ny - 1),
        }
    }

    fn halo_point(&self, face: usize, k: usize) -> (isize, isize) {
        let (nx, ny) = (self.nx as isize, self.ny as isize);
        let k = k as isize;
        match face as u32 {
            FACE_W => (-1, k),
            FACE_E => (nx, k),
            FACE_S => (k, -1),
            _ => (k, ny),
        }
    }

    fn mirror_point(&self, face: usize, k: usize) -> (isize, isize) {
        let (nx, ny) = (self.nx as isize, self.ny as isize);
        let k = k as isize;
        match face as u32 {
            FACE_W => (1, k),
            FACE_E => (nx - 2, k),
            FACE_S => (k, 1),
            _ => (k, ny - 2),
        }
    }

    /// 吸収境界として扱う面か (外周の面で、CFG_ABSORB のビットが立っている)
    fn is_absorbing(&self, face: usize) -> bool {
        self.kind[face] != BC_INTERFACE && (self.cfg.get(CFG_ABSORB) as u32 >> face) & 1 == 1
    }

    fn set_face(&mut self, face: u32, kind: u32, param: u64) {
        if face < 4 {
            self.kind[face as usize] = kind;
            self.param[face as usize] = param;
        }
    }

    fn cfg(&mut self, key: u32, val: f64) {
        if (key as usize) < self.cfg.v.len() {
            self.cfg.v[key as usize] = val;
            self.cfg.set |= 1 << key;
        }
    }

    /// 初期条件 (大域の座標の関数。区画の切り方によらず同じ点は同じビットになる)
    fn initial(&self, x: f64, y: f64) -> f64 {
        let c = &self.cfg;
        let amp = c.get(CFG_AMP);
        let (m, n) = (c.get(CFG_MODE_M), c.get(CFG_MODE_N));
        match c.get(CFG_IC) as u32 {
            IC_GAUSS => {
                let (dx, dy) = (x - c.get(CFG_GX), y - c.get(CFG_GY));
                let sg = c.get(CFG_SIGMA);
                amp * libm::exp(-(dx * dx + dy * dy) / (2.0 * sg * sg))
            }
            IC_COS_MODE => amp * libm::cos(m * PI * x) * libm::cos(n * PI * y),
            _ => amp * libm::sin(m * PI * x) * libm::sin(n * PI * y),
        }
    }

    fn init(&mut self, nx: u32, ny: u32, ix0: u32, iy0: u32, h: f64, c: f64, dt: f64) -> i32 {
        let (nx, ny) = (nx as usize, ny as usize);
        if nx < 3 || ny < 3 || nx > SIDE_MAX || ny > SIDE_MAX || (nx + 2) * (ny + 2) > PAD_MAX {
            return -1;
        }
        if !(h > 0.0) || !(c > 0.0) || !(dt > 0.0) || (self.cfg.get(CFG_IC) as u32 == IC_GAUSS && !(self.cfg.get(CFG_SIGMA) > 0.0)) {
            return -2;
        }
        self.nx = nx;
        self.ny = ny;
        self.ix0 = ix0;
        self.iy0 = iy0;
        self.h = h;
        self.c = c;
        self.dt = dt;
        let nu = c * dt / h;
        self.nu2 = nu * nu;
        self.half_nu2 = 0.5 * self.nu2;
        self.kmur = (nu - 1.0) / (nu + 1.0);
        self.step = 0;
        self.got = [false; 4];
        self.ghost = [[0.0; SIDE_MAX]; 4];
        self.dropped = 0;
        self.rejected = 0;
        self.missing = 0;
        for v in self.u.iter_mut() {
            *v = 0.0;
        }
        for v in self.p.iter_mut() {
            *v = 0.0;
        }
        for j in 0..ny {
            let y = ((iy0 as usize + j) as f64) * h;
            for i in 0..nx {
                let x = ((ix0 as usize + i) as f64) * h;
                let k = self.at(i as isize, j as isize);
                self.u[k] = self.initial(x, y);
            }
        }
        self.apply_dirichlet();
        0
    }

    fn apply_dirichlet(&mut self) {
        for face in 0..4 {
            if self.kind[face] != BC_DIRICHLET || self.is_absorbing(face) {
                continue;
            }
            let v = f64::from_bits(self.param[face]);
            for k in 0..self.face_len(face) {
                let (i, j) = self.edge_point(face, k);
                let idx = self.at(i, j);
                self.u[idx] = v;
            }
        }
    }

    fn outbox_read(&mut self, cap: usize) -> usize {
        let cap = cap.min(SCRATCH_CAP);
        let mut used = 0usize;
        let mut dropped = 0u64;
        for face in 0..4 {
            if self.kind[face] != BC_INTERFACE {
                continue;
            }
            let n = self.face_len(face);
            for k in 0..n {
                let (i, j) = self.edge_point(face, k);
                let v = self.u[self.at(i, j)];
                self.edge[8 * k..8 * k + 8].copy_from_slice(&v.to_bits().to_le_bytes());
            }
            let hdr = wire::Header {
                kind: wire::KIND_IFACE_BAND,
                // 変位 (2026-11-13 に番号を追加。docs/2026-11-13-imu-fusion.md §6。受け手は quantity を見ないので
                // 場の値・ハッシュは変わらない)
                quantity: wire::QTY_DISPLACEMENT,
                src: self.param[face] as u32,
                step: self.step,
                t_us: 0,
            };
            let payload = wire::Payload::IfaceBand { data: &self.edge[..8 * n], count: n as u32, per_point: 1 };
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
            let (hdr, payload) = match wire::decode(&self.scratch[raw.clone()]) {
                Ok((h, p, used)) if used == mlen => (h, p),
                _ => return INBOX_MALFORMED,
            };
            let wire::Payload::IfaceBand { data, count, per_point } = payload else {
                self.rejected += 1;
                result = INBOX_REJECTED;
                continue;
            };
            let face = (0..4).find(|&f| self.kind[f] == BC_INTERFACE && self.param[f] as u32 == hdr.src);
            let ok = match face {
                Some(f) => hdr.step == self.step && count as usize == self.face_len(f) && per_point >= 1,
                None => false,
            };
            if !ok {
                self.rejected += 1;
                result = INBOX_REJECTED;
                continue;
            }
            let f = face.unwrap();
            let pp = per_point as usize;
            for k in 0..count as usize {
                self.ghost[f][k] = wire::band_get(data, k * pp);
            }
            self.got[f] = true;
        }
        result
    }

    /// 枠 (ゴースト点) を現在の時刻 u^n で埋める。吸収境界の面はゼロ勾配と同じ鏡映
    /// (端の点は後でムーアの式で上書きするので、その値は端以外に影響しない)
    fn fill_halo(&mut self) {
        for face in 0..4 {
            for k in 0..self.face_len(face) {
                let (hi, hj) = self.halo_point(face, k);
                let v = if self.kind[face] == BC_INTERFACE {
                    self.ghost[face][k]
                } else if self.kind[face] == BC_ZERO_GRAD || self.is_absorbing(face) {
                    let (mi, mj) = self.mirror_point(face, k);
                    self.u[self.at(mi, mj)]
                } else {
                    f64::from_bits(self.param[face])
                };
                let idx = self.at(hi, hj);
                self.u[idx] = v;
            }
        }
    }

    fn step(&mut self) -> i32 {
        let mut status = STEP_OK;
        for face in 0..4 {
            if self.kind[face] == BC_INTERFACE && !self.got[face] {
                status = STEP_MISSING_GHOST;
            }
        }
        if status != STEP_OK {
            self.missing += 1;
        }
        self.fill_halo();
        let w = self.w();
        let first = self.step == 0;
        let (nu2, half_nu2) = (self.nu2, self.half_nu2);
        for j in 0..self.ny {
            for i in 0..self.nx {
                let c = (j + 1) * w + (i + 1);
                let u = self.u[c];
                let lap = (self.u[c - 1] - 2.0 * u + self.u[c + 1]) + (self.u[c - w] - 2.0 * u + self.u[c + w]);
                self.p[c] = if first { u + half_nu2 * lap } else { 2.0 * u - self.p[c] + nu2 * lap };
            }
        }
        // 吸収境界 (ムーアの方法): 新しい内側の値 p[内] と古い値 u[端]・u[内] から端の点を決める
        for face in 0..4 {
            if !self.is_absorbing(face) {
                continue;
            }
            for k in 0..self.face_len(face) {
                let (ei, ej) = self.edge_point(face, k);
                let (mi, mj) = self.mirror_point(face, k);
                let (e, m) = (self.at(ei, ej), self.at(mi, mj));
                self.p[e] = self.u[m] + self.kmur * (self.p[m] - self.u[e]);
            }
        }
        core::mem::swap(&mut self.u, &mut self.p);
        self.apply_dirichlet();
        self.step += 1;
        self.got = [false; 4];
        status
    }

    fn get(&self, k: usize) -> f64 {
        if self.nx == 0 || k >= self.nx * self.ny {
            return f64::NAN;
        }
        let (i, j) = (k % self.nx, k / self.nx);
        self.u[self.at(i as isize, j as isize)]
    }

    fn get_prev(&self, k: usize) -> f64 {
        if self.nx == 0 || k >= self.nx * self.ny {
            return f64::NAN;
        }
        let (i, j) = (k % self.nx, k / self.nx);
        self.p[self.at(i as isize, j as isize)]
    }
}

/// ネイティブの検証用: 1つ前の時刻 u^{n−1} の格子点 k = j·nx + i の値
/// (エネルギーの計算に使う。WASMの差込口には出さない)
pub fn get_prev(k: u32) -> f64 {
    state().get_prev(k as usize)
}

/// 縮小版かどうかと容量 (PAD_MAX, SIDE_MAX, SCRATCH_CAP)。ネイティブの確認用
pub fn capacity() -> (usize, usize, usize) {
    (PAD_MAX, SIDE_MAX, SCRATCH_CAP)
}

// ---------- WASMエクスポート ----------

/// 設定 (dyne_init2d の前に呼ぶ。init をまたいで保持)。キーは dyne_wire::abi::wave2d
#[no_mangle]
pub extern "C" fn dyne_cfg(key: u32, val: f64) {
    state().cfg(key, val);
}

/// 面の設定 (dyne_init2d の前に呼ぶ)。heat2d-actor と同じ
#[no_mangle]
pub extern "C" fn dyne_set_face(face: u32, kind: u32, param: u64) {
    state().set_face(face, kind, param);
}

/// 区画の初期化。alpha は波の速さ c として解釈する。
/// 戻り値: 0=成功 / −1=点数が範囲外 / −2=パラメータ不正
#[no_mangle]
pub extern "C" fn dyne_init2d(nx: u32, ny: u32, ix0: u32, iy0: u32, h: f64, alpha: f64, dt: f64) -> i32 {
    state().init(nx, ny, ix0, iy0, h, alpha, dt)
}

/// クーラン数 ν = c·dt/h (init の後)。無限領域の安定条件は ν ≤ 1/√2
#[no_mangle]
pub extern "C" fn dyne_courant() -> f64 {
    let s = state();
    if s.nx == 0 {
        return f64::NAN;
    }
    s.c * s.dt / s.h
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
    let s = state();
    (s.nx * s.ny) as u32
}

#[no_mangle]
pub extern "C" fn dyne_nx() -> u32 {
    state().nx as u32
}

#[no_mangle]
pub extern "C" fn dyne_ny() -> u32 {
    state().ny as u32
}

/// 格子点 k = j·nx + i の現在の値 u^n
#[no_mangle]
pub extern "C" fn dyne_get(k: u32) -> f64 {
    state().get(k as usize)
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

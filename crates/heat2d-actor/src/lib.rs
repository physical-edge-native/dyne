//! heat2d-actor: 2次元熱拡散の区画を解く WASMアクター (2026-10-10)
//!
//! 設計書 docs/2026-10-10-order10-design.md の「2次元の土台」の検証用の足場。
//! 答えが一つに決まる2次元熱拡散を差分法で解き、分割解と単一ノード解の
//! ビット一致で界面の受け渡し (メールボックス方式+界面サンプル列) を検証する。
//! 温度と形状を組み合わせる目的のものではない (設計書 §2)。
//!
//! ## 数値法
//!   u_t = α (u_xx + u_yy)、正方格子 (刻み h は x, y 共通)、上下左右の4点を使う
//!   中心差分+時間前進オイラー (陽解法):
//!     u' = u + r·((u_W − 2u + u_E) + (u_S − 2u + u_N))、r = α·dt/h²
//!   安定条件 r ≤ 1/4 (ホスト側で検証する)。
//!   初期条件 u = sin(πx)·sin(πy)、x = (ix0+i)·h, y = (iy0+j)·h (整数の大域添字から
//!   計算するので、区画の切り方によらず同じ点は同じビットになる)。
//!
//! ## ビット一致の仕組み
//!   状態は周囲1点ぶん広げた配列 (ゴースト点の枠つき) に持つ。ステップの冒頭で
//!   枠を埋め (界面=隣から届いた値、ゼロ勾配=鏡映、値固定=その値)、全格子点を
//!   **同じ式・同じ演算順**で更新する。界面の枠には隣の区画の端の値がそのまま
//!   入るので、単一ノードで解いたときの隣の格子点と同じ値で同じ式を計算する
//!   (理論ノートT1: 境界条件ではなく、全体と同じ式を分担する)。角の点との交換は
//!   4点の式なので不要。
//!
//! ## 差込口 (共有定数は dyne_wire::abi::field2d)
//!   dyne_set_face(face, kind, iface_or_bits) … 面の設定 (dyne_init2d の前)
//!   dyne_init2d(nx, ny, ix0, iy0, h, alpha, dt) -> i32 … 区画の初期化
//!   dyne_msg_buf() / dyne_msg_cap() … 受け渡し用の作業領域 (ホストはここへ書く/ここから読む)
//!   dyne_outbox_read(ptr, cap) -> u32 … 送信メールボックス: 全界面ぶんの界面サンプル列
//!       (現在の状態の自分の端の値。1点あたり1量=値のみ) を枠つきで書き出す
//!   dyne_inbox_write(ptr, len) -> i32 … 受信メールボックス: 届いたメッセージ列を
//!       読み解き、宛先の界面のゴースト値にする (INBOX_*)
//!   dyne_step() -> i32 … 1ステップ進める (STEP_*)
//!   dyne_len / dyne_nx / dyne_ny / dyne_get(k) … 状態の観測 (k = j·nx + i)
//!   dyne_steps_done / dyne_dropped / dyne_rejected … 計数
//!
//! メールボックスの中のメッセージはワイヤ形式v2 (dyne-wire) の KIND_IFACE_BAND。
//! ホストはヘッダ (界面ID=src、ステップ) だけを見て配送し、中身は解釈しない。
//!
//! no_std / 静的確保 / libm (純Rust) のみ依存。同一入力で全出力がビット一致する。

#![cfg_attr(target_arch = "wasm32", no_std)]

use dyne_wire as wire;
use dyne_wire::abi::field2d::*;

// ---------- 容量 ----------

/// 枠つき配列の点数の上限 ((nx+2)·(ny+2) ≤ PAD_MAX)。シミュレータ用のフル版。
/// 実機向けの縮小版 (線形メモリ64KB) は実機対応の段で feature として足す
const PAD_MAX: usize = 40_000;
/// 1辺の格子点数の上限 (界面サンプル列の長さ)。196点で1通 24+8+8·196 = 1600バイト
const SIDE_MAX: usize = 196;
/// 受け渡し用の作業領域: 4面 × (枠4 + ヘッダ24 + 8 + 8·SIDE_MAX) が入る大きさ
const SCRATCH_CAP: usize = 8192;

const PI: f64 = core::f64::consts::PI;

pub struct Field {
    nx: usize,
    ny: usize,
    ix0: u32,
    iy0: u32,
    h: f64,
    alpha: f64,
    dt: f64,
    /// 面ごとの種類 (BC_*)
    kind: [u32; 4],
    /// 面ごとの付帯値: 界面なら界面ID、値固定なら値 (f64 のビット列)
    param: [u64; 4],
    /// 枠つき配列 (幅 nx+2、高さ ny+2)。u[(j+1)·(nx+2) + (i+1)] が格子点 (i, j)
    u: [f64; PAD_MAX],
    u_next: [f64; PAD_MAX],
    /// 面ごとのゴースト値 (隣から届いた界面サンプル列の値)
    ghost: [[f64; SIDE_MAX]; 4],
    /// 現ステップで受領済みの面
    got: [bool; 4],
    /// 完了したステップ数 (= 送信するメッセージのステップ番号)
    step: u64,
    scratch: [u8; SCRATCH_CAP],
    /// 界面サンプル列の組み立て用
    edge: [u8; 8 * SIDE_MAX],
    /// 送信メールボックスのあふれで破棄したメッセージ数
    dropped: u64,
    /// 受理しなかったメッセージ数 (界面ID・ステップ・点の数の不一致)
    rejected: u64,
    /// 未受領の界面を抱えたまま計算したステップ数
    missing: u64,
}

static mut S: Field = Field {
    nx: 0,
    ny: 0,
    ix0: 0,
    iy0: 0,
    h: 0.0,
    alpha: 0.0,
    dt: 0.0,
    kind: [BC_DIRICHLET; 4],
    param: [0; 4],
    u: [0.0; PAD_MAX],
    u_next: [0.0; PAD_MAX],
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

    /// 格子点 (i, j) の枠つき配列での添字 (i, j は −1..=n を取りうる)
    #[inline]
    fn at(&self, i: isize, j: isize) -> usize {
        ((j + 1) as usize) * self.w() + (i + 1) as usize
    }

    /// 面の長さ (界面サンプル列の点の数)
    fn face_len(&self, face: usize) -> usize {
        if face == FACE_W as usize || face == FACE_E as usize {
            self.ny
        } else {
            self.nx
        }
    }

    /// 面の k 番目の自分の格子点 (端の点)。南北は i 昇順、東西は j 昇順
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

    /// 面の k 番目の枠の点 (自分の端の点の外側1点)
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

    /// 鏡映の相手 (自分の端の点から内側へ1点)
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

    fn set_face(&mut self, face: u32, kind: u32, param: u64) {
        if face < 4 {
            self.kind[face as usize] = kind;
            self.param[face as usize] = param;
        }
    }

    fn init(&mut self, nx: u32, ny: u32, ix0: u32, iy0: u32, h: f64, alpha: f64, dt: f64) -> i32 {
        let (nx, ny) = (nx as usize, ny as usize);
        // 鏡映に内側1点が要るので各辺3点以上
        if nx < 3 || ny < 3 || nx > SIDE_MAX || ny > SIDE_MAX || (nx + 2) * (ny + 2) > PAD_MAX {
            return -1;
        }
        if !(h > 0.0) || !(alpha > 0.0) || !(dt > 0.0) {
            return -2;
        }
        self.nx = nx;
        self.ny = ny;
        self.ix0 = ix0;
        self.iy0 = iy0;
        self.h = h;
        self.alpha = alpha;
        self.dt = dt;
        self.step = 0;
        self.got = [false; 4];
        self.ghost = [[0.0; SIDE_MAX]; 4];
        self.dropped = 0;
        self.rejected = 0;
        self.missing = 0;
        for v in self.u.iter_mut() {
            *v = 0.0;
        }
        for j in 0..ny {
            let y = ((iy0 as usize + j) as f64) * h;
            let sy = libm::sin(PI * y);
            for i in 0..nx {
                let x = ((ix0 as usize + i) as f64) * h;
                let k = self.at(i as isize, j as isize);
                self.u[k] = libm::sin(PI * x) * sy;
            }
        }
        self.apply_dirichlet();
        0
    }

    /// 値固定の面の格子点を値に固定する (面の順 W, E, S, N。角は後の面が勝つ)
    fn apply_dirichlet(&mut self) {
        for face in 0..4 {
            if self.kind[face] != BC_DIRICHLET {
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

    /// 送信メールボックス: 全界面の自分の端の値を界面サンプル列として書き出す
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
                quantity: wire::QTY_TEMPERATURE,
                src: self.param[face] as u32,
                step: self.step,
                t_us: 0,
            };
            let payload = wire::Payload::IfaceBand {
                data: &self.edge[..8 * n],
                count: n as u32,
                per_point: 1,
            };
            let mut w = wire::MailboxWriter::new(&mut self.scratch[used..cap]);
            let _ = w.push(&hdr, &payload);
            used += w.used();
            dropped += w.dropped;
        }
        // 終端印 (len=0)。読み手は容量切れでも止まるので書けなければ省略
        if used + 4 <= cap {
            self.scratch[used..used + 4].copy_from_slice(&0u32.to_le_bytes());
        }
        self.dropped += dropped;
        used
    }

    /// 受信メールボックス: scratch[..len] のメッセージ列を読み解いてゴースト値にする
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
            let face = (0..4).find(|&f| {
                self.kind[f] == BC_INTERFACE && self.param[f] as u32 == hdr.src
            });
            let ok = match face {
                Some(f) => hdr.step == self.step && count as usize == self.face_len(f),
                None => false,
            };
            if !ok {
                self.rejected += 1;
                result = INBOX_REJECTED;
                continue;
            }
            let f = face.unwrap();
            // 1点あたり per_point 個の量のうち、先頭 (値) をゴースト値に使う
            let pp = per_point as usize;
            for k in 0..count as usize {
                self.ghost[f][k] = wire::band_get(data, k * pp);
            }
            self.got[f] = true;
        }
        result
    }

    /// 枠 (ゴースト点) を埋める
    fn fill_halo(&mut self) {
        for face in 0..4 {
            for k in 0..self.face_len(face) {
                let (hi, hj) = self.halo_point(face, k);
                let v = match self.kind[face] {
                    BC_INTERFACE => self.ghost[face][k],
                    BC_ZERO_GRAD => {
                        let (mi, mj) = self.mirror_point(face, k);
                        self.u[self.at(mi, mj)]
                    }
                    _ => f64::from_bits(self.param[face]),
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
        let r = self.alpha * self.dt / (self.h * self.h);
        let w = self.w();
        for j in 0..self.ny {
            for i in 0..self.nx {
                let c = (j + 1) * w + (i + 1);
                let u = self.u[c];
                let lap = (self.u[c - 1] - 2.0 * u + self.u[c + 1])
                    + (self.u[c - w] - 2.0 * u + self.u[c + w]);
                self.u_next[c] = u + r * lap;
            }
        }
        // Copy only the block's own points back (issue #96). Swapping the two whole arrays moved
        // 2 × PAD_MAX values per step whatever the block size, which made a small block cost about
        // 1.5 ms per step in the simulator. The halo is refilled by fill_halo before it is read, so
        // the values computed are the same bit for bit.
        for j in 0..self.ny {
            let row = (j + 1) * w + 1;
            self.u[row..row + self.nx].copy_from_slice(&self.u_next[row..row + self.nx]);
        }
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
}

// ---------- 初期場の差し替え口 (2026-11-08、他クレートのアクター向け) ----------

/// dyne_init2d の直後・最初のステップの前に、初期場を格子点ごとの値 f(x, y) で置き換える。
/// x = (ix0+i)·h, y = (iy0+j)·h は dyne_init2d と同じ式 (区画の切り方によらず同じ点は
/// 同じ入力になる)。置き換えた後、値固定の面を再び固定する。
/// heat2d-mlp-actor (外部で学習したモデルの予測を初期場にするアクター) が使う。
/// 本クレートの差込口の動作 (既存の経路の数値) は変えない。
/// 戻り値: 0=成功 / −1=未初期化または既にステップを進めた / −2=f が非有限の値を返した
pub fn replace_initial(mut f: impl FnMut(f64, f64) -> f64) -> i32 {
    let s = state();
    if s.nx == 0 || s.step != 0 {
        return -1;
    }
    let mut bad = false;
    for j in 0..s.ny {
        let y = ((s.iy0 as usize + j) as f64) * s.h;
        for i in 0..s.nx {
            let x = ((s.ix0 as usize + i) as f64) * s.h;
            let v = f(x, y);
            if !v.is_finite() {
                bad = true;
            }
            let k = s.at(i as isize, j as isize);
            s.u[k] = v;
        }
    }
    s.apply_dirichlet();
    if bad {
        -2
    } else {
        0
    }
}

// ---------- WASMエクスポート ----------

/// 面の設定 (dyne_init2d の前に呼ぶ。init をまたいで保持)。
/// kind=BC_INTERFACE なら param=界面ID、BC_DIRICHLET なら param=値の f64 ビット列
/// (f64::to_bits)、BC_ZERO_GRAD なら param は無視
#[no_mangle]
pub extern "C" fn dyne_set_face(face: u32, kind: u32, param: u64) {
    state().set_face(face, kind, param);
}

/// 区画の初期化: nx×ny 点、大域添字の原点 (ix0, iy0)、刻み h。
/// 戻り値: 0=成功 / −1=点数が範囲外 / −2=パラメータ不正
#[no_mangle]
pub extern "C" fn dyne_init2d(nx: u32, ny: u32, ix0: u32, iy0: u32, h: f64, alpha: f64, dt: f64) -> i32 {
    state().init(nx, ny, ix0, iy0, h, alpha, dt)
}

/// 受け渡し用の作業領域の先頭アドレス
#[no_mangle]
pub extern "C" fn dyne_msg_buf() -> u32 {
    core::ptr::addr_of!(state().scratch) as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_msg_cap() -> u32 {
    SCRATCH_CAP as u32
}

/// 送信メールボックス: ptr (= dyne_msg_buf) へ全界面ぶんのメッセージを枠つきで書き、
/// 使用バイト数を返す。cap に入らないメッセージは破棄+計数
#[no_mangle]
pub extern "C" fn dyne_outbox_read(ptr: u32, cap: u32) -> u32 {
    let s = state();
    if ptr != core::ptr::addr_of!(s.scratch) as usize as u32 {
        return 0;
    }
    s.outbox_read(cap as usize) as u32
}

/// 受信メールボックス: ptr (= dyne_msg_buf) に置かれた枠つきメッセージ列 (len バイト) を
/// 読み解く。戻り値は INBOX_*
#[no_mangle]
pub extern "C" fn dyne_inbox_write(ptr: u32, len: u32) -> i32 {
    let s = state();
    if ptr != core::ptr::addr_of!(s.scratch) as usize as u32 || len as usize > SCRATCH_CAP {
        return INBOX_MALFORMED;
    }
    s.inbox_write(len as usize)
}

/// 1ステップ進める。戻り値は STEP_*
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

/// 格子点 k = j·nx + i の値
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

/// 未受領の界面を抱えたまま計算したステップ数
#[no_mangle]
pub extern "C" fn dyne_missing() -> u64 {
    state().missing
}

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    core::arch::wasm32::unreachable()
}

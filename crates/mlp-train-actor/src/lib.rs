//! mlp-train-actor: 外部で学習した多層パーセプトロン (MLP、全結合層と活性化関数を
//! 積み重ねたNN) を読み込み、マイコン上でさらに学習 (追加学習) できる WASMアクター (2026-11-08)
//!
//! 推論だけの crates/mlp-actor は小さく保ち、学習の作業領域 (正規方程式の行列など) が
//! 要る機能はこちらに分けた。.dynm の検証・読み出し・活性化関数は mlp-core の部品を
//! そのまま使う (mlp-actor と共通)。
//!
//! ## 流れ
//!   1. ホストが .dynm を読み込み領域に書き、dyne_model_load(len) を呼ぶ。
//!      検証に通ると、重み・バイアスを書き換え可能なパラメータ配列 (f64) へ写す。
//!      f32 の重みは f64 に広げる (以後の学習は f64 で行う)。
//!   2. ホストが学習データ (入力 X を行優先で rows·n_in 個、目標 T を rows·n_out 個) を
//!      dyne_train_x_buf / dyne_train_t_buf に書き、dyne_train_set(rows) で確定する。
//!   3. dyne_train_lm(iters) で LM法 (レーベンバーグ・マルカート法、ガウス・ニュートン法に
//!      減衰項を足した非線形最小二乗法) を iters 回まわす。損失は平均二乗誤差
//!      L = (1/(rows·n_out)) Σ (y − t)²。
//!   4. 推論 (dyne_model_eval_batch) は常に現在のパラメータ配列で行う。読み込み直後は
//!      mlp-actor と同じ演算順なので、出力は mlp-actor とビット一致する。
//!
//! ## LM法の1反復 (crates/heat1d-pinn-actor の lm_step と同じ構成)
//!   残差 r_k = s·(y_k − t_k)、s = √(1/(rows·n_out))。ヤコビ行列 J (∂r/∂p) の行を
//!   1つずつ逆伝播で作り、JᵀJ と Jᵀr を積み上げる (J 全体は持たない)。
//!   (JᵀJ + λ·diag(JᵀJ)) δ = −Jᵀr をコレスキー分解で解き、損失が下がれば採用して λ を
//!   1/3 に、下がらなければ戻して λ を4倍にして再試行 (最大8回)。分解できなければ λ を10倍。
//!   和の順序はすべて固定なので、同じ入力なら結果はビット一致する。
//!
//! ## 差込口
//!   dyne_load_buf / dyne_load_cap / dyne_model_load(len) -> i32 (mlp_core::LOAD_*)
//!   dyne_in_buf / dyne_out_buf / dyne_io_cap / dyne_model_eval_batch(rows) -> i32
//!   dyne_model_n_in / dyne_model_n_out / dyne_model_n_params / dyne_param_max
//!   dyne_train_x_buf / dyne_train_t_buf / dyne_train_cap (各 f64 の個数)
//!   dyne_train_set(rows) -> i32 (TRAIN_*) / dyne_train_loss() -> f64
//!   dyne_train_lm(iters) -> f64 (反復後の損失。負 = 学習できない状態)
//!   dyne_train_param_max() … LM法で扱えるパラメータ数の上限
//!   dyne_param_get(k) -> f64 … 現在のパラメータ k / dyne_model_reset() … 読み込み時の値に戻す
//!
//! no_std / 静的確保 / libm のみ依存。

#![cfg_attr(target_arch = "wasm32", no_std)]

use mlp_core::{activate, new_mlp, Mlp, ACT_NONE, ACT_RELU, ACT_SIGMOID, ACT_TANH, LAYER_MAX, PARAM_MAX, WIDTH_MAX};

// ---------- 容量 ----------

/// LM法で扱えるパラメータ数の上限 (正規方程式の P×P 行列を2枚静的確保する)。
/// フル版 256 (幅16・隠れ1層の 49 個、幅12・隠れ2層の 193 個が入る)、縮小版 64
#[cfg(not(feature = "small"))]
pub const P_LM: usize = 256;
#[cfg(feature = "small")]
pub const P_LM: usize = 64;

/// 学習データの領域の容量 (入力・目標それぞれ f64 の個数)
#[cfg(not(feature = "small"))]
pub const DATA_CAP: usize = 8192;
#[cfg(feature = "small")]
pub const DATA_CAP: usize = 512;

/// 推論の入力・出力の領域の容量 (f64 の個数)
#[cfg(not(feature = "small"))]
pub const IO_CAP: usize = 1024;
#[cfg(feature = "small")]
pub const IO_CAP: usize = 256;

// dyne_train_set の戻り値
pub const TRAIN_OK: i32 = 0;
pub const TRAIN_NO_MODEL: i32 = -1;
pub const TRAIN_TOO_MANY: i32 = -2;
pub const TRAIN_NONFINITE: i32 = -3;
pub const TRAIN_EMPTY: i32 = -4;

// dyne_model_eval_batch の戻り値 (負のとき。mlp-actor と同じ値)
pub const EVAL_NO_MODEL: i32 = mlp_core::EVAL_NO_MODEL;
pub const EVAL_TOO_MANY: i32 = mlp_core::EVAL_TOO_MANY;

pub struct Trainer {
    mlp: Mlp,
    /// 書き換え可能なパラメータ (読み込み時に .dynm から写す)
    p: [f64; PARAM_MAX],
    n_params: usize,
    ready: bool,
    // ---- 推論・逆伝播の作業領域 ----
    /// 層 l の入力 (a[0] = ネットへの入力、a[l+1] = 層 l の出力)
    a: [[f64; WIDTH_MAX]; LAYER_MAX + 1],
    delta: [f64; WIDTH_MAX],
    delta_prev: [f64; WIDTH_MAX],
    /// ヤコビ行列の1行 (∂y_o/∂p)
    g: [f64; P_LM],
    // ---- 学習データ ----
    x: [f64; DATA_CAP],
    t: [f64; DATA_CAP],
    rows: usize,
    // ---- 推論の入出力 ----
    input: [f64; IO_CAP],
    output: [f64; IO_CAP],
    // ---- LM法の作業領域 ----
    lm_lambda: f64,
    lm_a: [f64; P_LM * P_LM],
    lm_l: [f64; P_LM * P_LM],
    lm_b: [f64; P_LM],
    lm_delta: [f64; P_LM],
    p_backup: [f64; P_LM],
    pub last_loss: f64,
}

pub const fn new_trainer() -> Trainer {
    Trainer {
        mlp: new_mlp(),
        p: [0.0; PARAM_MAX],
        n_params: 0,
        ready: false,
        a: [[0.0; WIDTH_MAX]; LAYER_MAX + 1],
        delta: [0.0; WIDTH_MAX],
        delta_prev: [0.0; WIDTH_MAX],
        g: [0.0; P_LM],
        x: [0.0; DATA_CAP],
        t: [0.0; DATA_CAP],
        rows: 0,
        input: [0.0; IO_CAP],
        output: [0.0; IO_CAP],
        // 0 で静的確保 (全体がゼロなら WASM のデータ節に載らず、ファイルが小さい)。
        // 読み込み時 (reset_params) に 1e-3 にする
        lm_lambda: 0.0,
        lm_a: [0.0; P_LM * P_LM],
        lm_l: [0.0; P_LM * P_LM],
        lm_b: [0.0; P_LM],
        lm_delta: [0.0; P_LM],
        p_backup: [0.0; P_LM],
        last_loss: 0.0,
    }
}

/// 活性化関数の微分 (前活性 z ではなく出力 a から求める。ReLU は a > 0 で 1)
#[inline]
fn act_deriv(act: u32, a: f64) -> f64 {
    match act {
        ACT_TANH => 1.0 - a * a,
        ACT_RELU => {
            if a > 0.0 {
                1.0
            } else {
                0.0
            }
        }
        ACT_SIGMOID => a * (1.0 - a),
        _ => 1.0,
    }
}

impl Trainer {
    /// 読み込み領域の len バイトを検証し、パラメータ配列へ写す (f32 は f64 に広げる)
    pub fn model_load(&mut self, len: usize) -> i32 {
        self.ready = false;
        self.rows = 0;
        let r = self.mlp.model_load(len);
        if r != mlp_core::LOAD_OK {
            return r;
        }
        self.n_params = self.mlp.n_params();
        self.reset_params();
        self.ready = true;
        r
    }

    /// パラメータを読み込み時の値に戻す
    pub fn reset_params(&mut self) {
        for k in 0..self.mlp.n_params() {
            self.p[k] = self.mlp.param(k);
        }
        self.lm_lambda = 1e-3;
    }

    fn n_in(&self) -> usize {
        self.mlp.n_in()
    }

    fn n_out(&self) -> usize {
        if self.ready {
            self.mlp.n_out()
        } else {
            0
        }
    }

    /// a[0] に置いた入力を順伝播する。各層の出力を a[l+1] に残す。
    /// 演算順は mlp-actor と同じ (acc=0 → 昇順に W·x → +b → 活性化)
    fn forward(&mut self) {
        for l in 0..self.mlp.n_layers() {
            let ly = self.mlp.layer(l);
            let b_off = ly.w_off + ly.n_in * ly.n_out;
            for j in 0..ly.n_out {
                let mut acc = 0.0f64;
                let w_row = ly.w_off + j * ly.n_in;
                for i in 0..ly.n_in {
                    acc += self.p[w_row + i] * self.a[l][i];
                }
                acc += self.p[b_off + j];
                self.a[l + 1][j] = activate(ly.act, acc);
            }
        }
    }

    /// 直前の forward の結果から、出力 o のパラメータ勾配 ∂y_o/∂p を g に書く
    fn backward(&mut self, o: usize) {
        let nl = self.mlp.n_layers();
        for k in 0..self.n_params {
            self.g[k] = 0.0;
        }
        let last = self.mlp.layer(nl - 1);
        for j in 0..last.n_out {
            self.delta[j] = 0.0;
        }
        self.delta[o] = act_deriv(last.act, self.a[nl][o]);
        for l in (0..nl).rev() {
            let ly = self.mlp.layer(l);
            let b_off = ly.w_off + ly.n_in * ly.n_out;
            for j in 0..ly.n_out {
                let d = self.delta[j];
                let w_row = ly.w_off + j * ly.n_in;
                for i in 0..ly.n_in {
                    self.g[w_row + i] = d * self.a[l][i];
                }
                self.g[b_off + j] = d;
            }
            if l > 0 {
                let prev_act = self.mlp.layer(l - 1).act;
                for i in 0..ly.n_in {
                    let mut s = 0.0f64;
                    for j in 0..ly.n_out {
                        s += self.p[ly.w_off + j * ly.n_in + i] * self.delta[j];
                    }
                    self.delta_prev[i] = s * act_deriv(prev_act, self.a[l][i]);
                }
                for i in 0..ly.n_in {
                    self.delta[i] = self.delta_prev[i];
                }
            }
        }
    }

    pub fn eval_batch(&mut self, rows: usize) -> i32 {
        if !self.ready {
            return EVAL_NO_MODEL;
        }
        let (n_in, n_out) = (self.n_in(), self.n_out());
        if rows * n_in > IO_CAP || rows * n_out > IO_CAP {
            return EVAL_TOO_MANY;
        }
        let nl = self.mlp.n_layers();
        for r in 0..rows {
            for i in 0..n_in {
                self.a[0][i] = self.input[r * n_in + i];
            }
            self.forward();
            for j in 0..n_out {
                self.output[r * n_out + j] = self.a[nl][j];
            }
        }
        n_out as i32
    }

    /// 学習データの行数を確定する
    pub fn train_set(&mut self, rows: usize) -> i32 {
        self.rows = 0;
        if !self.ready {
            return TRAIN_NO_MODEL;
        }
        if rows == 0 {
            return TRAIN_EMPTY;
        }
        let (n_in, n_out) = (self.n_in(), self.n_out());
        if rows * n_in > DATA_CAP || rows * n_out > DATA_CAP {
            return TRAIN_TOO_MANY;
        }
        if self.x[..rows * n_in].iter().chain(self.t[..rows * n_out].iter()).any(|v| !v.is_finite()) {
            return TRAIN_NONFINITE;
        }
        self.rows = rows;
        TRAIN_OK
    }

    #[inline]
    fn scale(&self) -> f64 {
        libm::sqrt(1.0 / ((self.rows * self.n_out()) as f64))
    }

    fn load_row(&mut self, r: usize) {
        let n_in = self.n_in();
        for i in 0..n_in {
            self.a[0][i] = self.x[r * n_in + i];
        }
    }

    /// 現在のパラメータでの損失 (平均二乗誤差)。学習データがなければ負
    pub fn loss(&mut self) -> f64 {
        if !self.ready || self.rows == 0 {
            return -1.0;
        }
        let s = self.scale();
        let (n_out, nl) = (self.n_out(), self.mlp.n_layers());
        let mut loss = 0.0;
        for r in 0..self.rows {
            self.load_row(r);
            self.forward();
            for o in 0..n_out {
                let res = s * (self.a[nl][o] - self.t[r * n_out + o]);
                loss += res * res;
            }
        }
        loss
    }

    /// LM法 1反復。戻り値: 反復後の損失
    fn lm_step(&mut self) -> f64 {
        let np = self.n_params;
        for k in 0..np * np {
            self.lm_a[k] = 0.0;
        }
        for k in 0..np {
            self.lm_b[k] = 0.0;
        }
        let s = self.scale();
        let (n_out, nl) = (self.n_out(), self.mlp.n_layers());
        let mut loss = 0.0;
        for r in 0..self.rows {
            self.load_row(r);
            self.forward();
            for o in 0..n_out {
                let res = s * (self.a[nl][o] - self.t[r * n_out + o]);
                loss += res * res;
                self.backward(o);
                for a in 0..np {
                    let ja = s * self.g[a];
                    if ja == 0.0 {
                        continue;
                    }
                    self.lm_b[a] += ja * res;
                    for b in 0..=a {
                        self.lm_a[a * np + b] += ja * (s * self.g[b]);
                    }
                }
            }
        }
        self.p_backup[..np].copy_from_slice(&self.p[..np]);
        for _try in 0..8 {
            let lam = self.lm_lambda;
            let mut ok = true;
            'outer: for a in 0..np {
                for b in 0..=a {
                    let mut sum = self.lm_a[a * np + b];
                    if a == b {
                        sum += lam * self.lm_a[a * np + a] + 1e-15;
                    }
                    for k in 0..b {
                        sum -= self.lm_l[a * np + k] * self.lm_l[b * np + k];
                    }
                    if a == b {
                        if sum <= 0.0 {
                            ok = false;
                            break 'outer;
                        }
                        self.lm_l[a * np + a] = libm::sqrt(sum);
                    } else {
                        self.lm_l[a * np + b] = sum / self.lm_l[b * np + b];
                    }
                }
            }
            if !ok {
                self.lm_lambda *= 10.0;
                continue;
            }
            for a in 0..np {
                let mut sum = -self.lm_b[a];
                for k in 0..a {
                    sum -= self.lm_l[a * np + k] * self.lm_delta[k];
                }
                self.lm_delta[a] = sum / self.lm_l[a * np + a];
            }
            for a in (0..np).rev() {
                let mut sum = self.lm_delta[a];
                for k in a + 1..np {
                    sum -= self.lm_l[k * np + a] * self.lm_delta[k];
                }
                self.lm_delta[a] = sum / self.lm_l[a * np + a];
            }
            for a in 0..np {
                self.p[a] = self.p_backup[a] + self.lm_delta[a];
            }
            let new_loss = self.loss();
            if new_loss < loss {
                self.lm_lambda = (self.lm_lambda / 3.0).max(1e-12);
                return new_loss;
            }
            self.p[..np].copy_from_slice(&self.p_backup[..np]);
            self.lm_lambda = (self.lm_lambda * 4.0).min(1e12);
        }
        loss
    }

    /// LM法を iters 回まわす。戻り値: 損失 (負 = 学習できない: 未読み込み・データなし・
    /// パラメータ数が P_LM を超える)
    pub fn train_lm(&mut self, iters: u32) -> f64 {
        if !self.ready || self.rows == 0 {
            return -1.0;
        }
        if self.n_params > P_LM {
            return -2.0;
        }
        let mut loss = self.loss();
        for _ in 0..iters {
            loss = self.lm_step();
            if loss < 1e-28 {
                break;
            }
        }
        self.last_loss = loss;
        loss
    }
}

// ---------- 差込口 ----------

static mut S: Trainer = new_trainer();

fn state() -> &'static mut Trainer {
    unsafe { &mut *core::ptr::addr_of_mut!(S) }
}

#[no_mangle]
pub extern "C" fn dyne_load_buf() -> u32 {
    state().mlp.load_buf_mut().as_ptr() as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_load_cap() -> u32 {
    mlp_core::LOAD_CAP as u32
}

/// .dynm を検証してパラメータ配列へ写す。戻り値は mlp_core::LOAD_*
#[no_mangle]
pub extern "C" fn dyne_model_load(len: u32) -> i32 {
    state().model_load(len as usize)
}

/// パラメータを読み込み時の値に戻す
#[no_mangle]
pub extern "C" fn dyne_model_reset() {
    state().reset_params();
}

#[no_mangle]
pub extern "C" fn dyne_in_buf() -> u32 {
    core::ptr::addr_of!(state().input) as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_out_buf() -> u32 {
    core::ptr::addr_of!(state().output) as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_io_cap() -> u32 {
    IO_CAP as u32
}

/// rows 組をまとめて推論する (現在のパラメータで)。戻り値: 出力の次元 / 負=EVAL_*
#[no_mangle]
pub extern "C" fn dyne_model_eval_batch(rows: u32) -> i32 {
    state().eval_batch(rows as usize)
}

#[no_mangle]
pub extern "C" fn dyne_model_n_in() -> u32 {
    state().n_in() as u32
}

#[no_mangle]
pub extern "C" fn dyne_model_n_out() -> u32 {
    state().n_out() as u32
}

#[no_mangle]
pub extern "C" fn dyne_model_n_params() -> u32 {
    let s = state();
    if s.ready { s.n_params as u32 } else { 0 }
}

#[no_mangle]
pub extern "C" fn dyne_param_max() -> u32 {
    PARAM_MAX as u32
}

#[no_mangle]
pub extern "C" fn dyne_train_param_max() -> u32 {
    P_LM as u32
}

#[no_mangle]
pub extern "C" fn dyne_train_x_buf() -> u32 {
    core::ptr::addr_of!(state().x) as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_train_t_buf() -> u32 {
    core::ptr::addr_of!(state().t) as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_train_cap() -> u32 {
    DATA_CAP as u32
}

/// 学習データ rows 行を確定する。戻り値は TRAIN_*
#[no_mangle]
pub extern "C" fn dyne_train_set(rows: u32) -> i32 {
    state().train_set(rows as usize)
}

#[no_mangle]
pub extern "C" fn dyne_train_loss() -> f64 {
    state().loss()
}

/// LM法を iters 回まわし、反復後の損失を返す (負 = 学習できない状態)
#[no_mangle]
pub extern "C" fn dyne_train_lm(iters: u32) -> f64 {
    state().train_lm(iters)
}

#[no_mangle]
pub extern "C" fn dyne_param_get(k: u32) -> f64 {
    let s = state();
    if (k as usize) < s.n_params { s.p[k as usize] } else { f64::NAN }
}

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    core::arch::wasm32::unreachable()
}

// 未使用の警告を避ける (ACT_NONE は act_deriv の既定の枝に当たる)
const _: u32 = ACT_NONE;

// ---------- 単体テスト (ネイティブ) ----------

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use mlp_core::{FLAG_F32, MAGIC, OP_DENSE, VERSION};
    use std::boxed::Box;
    use std::vec::Vec;

    fn build(n_in: u32, layers: &[(u32, u32, u32)], params: &[f64], f32w: bool) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&MAGIC);
        v.extend_from_slice(&VERSION.to_le_bytes());
        v.extend_from_slice(&(if f32w { FLAG_F32 } else { 0 }).to_le_bytes());
        v.extend_from_slice(&n_in.to_le_bytes());
        v.extend_from_slice(&(layers.len() as u32).to_le_bytes());
        v.extend_from_slice(&(params.len() as u32).to_le_bytes());
        v.extend_from_slice(&0u32.to_le_bytes());
        for &(i, o, a) in layers {
            for x in [OP_DENSE, i, o, a] {
                v.extend_from_slice(&x.to_le_bytes());
            }
        }
        for &p in params {
            if f32w {
                v.extend_from_slice(&(p as f32).to_le_bytes());
            } else {
                v.extend_from_slice(&p.to_le_bytes());
            }
        }
        v
    }

    fn net_params() -> Vec<f64> {
        net_params_n(25)
    }

    /// 決定論的な初期重み (n 個、[−1, 1))
    fn net_params_n(n: usize) -> Vec<f64> {
        let mut s = 0x1234_5678_9ABC_DEF1u64;
        (0..n)
            .map(|_| {
                s ^= s >> 12;
                s ^= s << 25;
                s ^= s >> 27;
                (((s.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64) / ((1u64 << 53) as f64)) * 2.0 - 1.0
            })
            .collect()
    }

    const LAYERS: [(u32, u32, u32); 2] = [(1, 8, ACT_TANH), (8, 1, ACT_NONE)];

    /// 状態は大きい (フル版で約1.6MB) のでスタックを経由せずヒープに直接作る。
    /// 全フィールドはゼロで有効な値 (f64/usize/u32/bool) なので、ゼロ埋め後に既定値を入れる
    fn boxed() -> Box<Trainer> {
        unsafe {
            let layout = std::alloc::Layout::new::<Trainer>();
            let ptr = std::alloc::alloc_zeroed(layout) as *mut Trainer;
            assert!(!ptr.is_null());
            let mut b = Box::from_raw(ptr);
            b.lm_lambda = 1e-3;
            b
        }
    }

    fn fresh(bytes: &[u8]) -> Box<Trainer> {
        let mut t = boxed();
        t.mlp.load_buf_mut()[..bytes.len()].copy_from_slice(bytes);
        assert_eq!(t.model_load(bytes.len()), 0);
        t
    }

    /// 目標: sin(πx)、x ∈ [−1, 1] の 21 点
    fn set_data(t: &mut Trainer) {
        for k in 0..21 {
            let x = -1.0 + 0.1 * k as f64;
            t.x[k] = x;
            t.t[k] = libm::sin(core::f64::consts::PI * x);
        }
        assert_eq!(t.train_set(21), TRAIN_OK);
    }

    #[test]
    fn f32_weights_widened_and_eval_matches_inference_actor() {
        let p = net_params();
        let bytes = build(1, &LAYERS, &p, true);
        let mut t = fresh(&bytes);
        for k in 0..p.len() {
            assert_eq!(t.p[k].to_bits(), ((p[k] as f32) as f64).to_bits());
        }
        // 推論専用アクターの部品 (同じ .dynm) とビット一致
        let mut m = Box::new(new_mlp());  // 約90KB なのでスタック経由でも収まる
        m.load_buf_mut()[..bytes.len()].copy_from_slice(&bytes);
        assert_eq!(m.model_load(bytes.len()), 0);
        for k in 0..50 {
            let x = -1.2 + 0.05 * k as f64;
            t.input[k] = x;
        }
        assert_eq!(t.eval_batch(50), 1);
        for k in 0..50 {
            let mut y = [0.0];
            m.eval_point(&[t.input[k]], &mut y);
            assert_eq!(t.output[k].to_bits(), y[0].to_bits());
        }
    }

    #[test]
    fn gradient_matches_finite_difference() {
        let _p = net_params();
        let mut t = fresh(&build(1, &[(1, 4, ACT_SIGMOID), (4, 3, ACT_TANH), (3, 2, ACT_NONE)], &net_params_n(31), false));
        t.a[0][0] = 0.37;
        t.forward();
        t.backward(1);
        let g: Vec<f64> = t.g[..t.n_params].to_vec();
        for k in 0..t.n_params {
            let h = 1e-6;
            let p0 = t.p[k];
            t.p[k] = p0 + h;
            t.a[0][0] = 0.37;
            t.forward();
            let yp = t.a[3][1];
            t.p[k] = p0 - h;
            t.forward();
            let ym = t.a[3][1];
            t.p[k] = p0;
            let fd = (yp - ym) / (2.0 * h);
            assert!((fd - g[k]).abs() < 1e-7, "k={k} 解析 {} 差分 {}", g[k], fd);
        }
    }

    #[test]
    fn lm_reduces_loss() {
        let mut t = fresh(&build(1, &LAYERS, &net_params(), false));
        set_data(&mut t);
        let l0 = t.loss();
        let l1 = t.train_lm(30);
        std::println!("LM 30反復: 損失 {l0:.3e} → {l1:.3e}");
        assert!(l1 < l0 * 1e-3, "{l0} → {l1}");
    }

    #[test]
    fn deterministic_bit_identical() {
        let bytes = build(1, &LAYERS, &net_params(), true);
        let run = || {
            let mut t = fresh(&bytes);
            set_data(&mut t);
            let l = t.train_lm(15);
            let p: Vec<u64> = t.p[..t.n_params].iter().map(|v| v.to_bits()).collect();
            (l.to_bits(), p)
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn refuses_without_data_or_model() {
        let mut t = boxed();
        assert_eq!(t.train_set(3), TRAIN_NO_MODEL);
        assert!(t.train_lm(1) < 0.0);
        let mut t = fresh(&build(1, &LAYERS, &net_params(), false));
        assert!(t.train_lm(1) < 0.0);
        t.t[0] = f64::NAN;
        assert_eq!(t.train_set(1), TRAIN_NONFINITE);
        // 読み込み時の値へ戻せる
        set_data(&mut t);
        let before = t.p[3];
        t.train_lm(3);
        assert_ne!(t.p[3].to_bits(), before.to_bits());
        t.reset_params();
        assert_eq!(t.p[3].to_bits(), before.to_bits());
    }
}

//! mlp-core: 多層パーセプトロン (MLP、全結合層と活性化関数を積み重ねたNN) の
//! .dynm 形式の検証・読み出し・推論の部品 (2026-11-08)
//!
//! 差込口 (#[no_mangle] の関数) もパニック時の処理も持たない、純粋な部品のクレート。
//! これを使う WASMアクター:
//!   crates/mlp-actor … 推論だけのアクター (差込口は下の「差込口」の節)
//!   crates/mlp-train-actor … 読み込んだ重みをマイコン上で追加学習するアクター
//!   crates/heat2d-mlp-actor … 2次元の熱拡散の初期場をこのモデルの予測で作るアクター
//!
//! PyTorch 等の重い実行環境で学習したモデルを ONNX (NNの計算グラフと重みの
//! 共通交換形式) で書き出し、scripts/onnx_import.py で「計算グラフ+重み」の
//! 小さな2進ファイル (.dynm) に変換したものを、そのまま読み込んで推論する。
//! マイコン側には ONNX の実行環境を置かない。
//!
//! ## .dynm 形式 第1版 (すべてリトルエンディアン)
//!   位置  大きさ  内容
//!   0     4       識別子 b"DYNM"
//!   4     2       版 (u16) = 1
//!   6     2       フラグ (u16)。bit0 = 1 なら重み・バイアスは f32、0 なら f64
//!   8     4       入力の次元 (u32)
//!   12    4       層の数 L (u32)
//!   16    4       パラメータの総数 (u32、全層の重み+バイアスの個数)
//!   20    4       予約 (u32) = 0
//!   24    16·L    層の記述: 層ごとに 演算 (u32、1=全結合)、入力次元 (u32)、
//!                 出力次元 (u32)、活性化 (u32、0=なし 1=tanh 2=ReLU 3=シグモイド)
//!   24+16L ...    層ごとに 重み W[出力][入力] (行優先) → バイアス b[出力]
//!   ファイルの長さは上の合計とちょうど一致しなければならない。
//!
//! ## 計算 (演算順を固定し、同一入力で全出力がビット一致する)
//!   y_j = act( (Σ_{i=0..in-1, 昇順} W[j][i]·x_i) + b_j )
//!   累積は f64。重みが f32 のときは f64 に広げてから掛ける。
//!   tanh・exp は libm (純Rust) を使う。
//!
//! ## 差込口 (crates/mlp-actor が出す)
//!   dyne_load_buf() / dyne_load_cap() … .dynm を書き込む領域とその容量
//!   dyne_model_load(len) -> i32 … 読み込んだ len バイトを検証して有効にする (LOAD_*)
//!   dyne_in_buf() / dyne_out_buf() / dyne_io_cap() … 入力・出力の f64 列の領域と
//!       その容量 (f64 の個数)
//!   dyne_model_eval(n_in) -> i32 … 1組の入力を推論し、出力の次元を返す (負=EVAL_*)
//!   dyne_model_eval_batch(rows) -> i32 … rows 組をまとめて推論 (入力は行優先で
//!       rows·n_in 個、出力も行優先で rows·n_out 個)。出力の次元を返す
//!   dyne_model_n_in / dyne_model_n_out / dyne_model_n_layers / dyne_model_n_params
//!   dyne_param_max() … パラメータ数の上限
//!
//! no_std / 静的確保 / libm のみ依存。
//! (2026-11-08 に mlp-actor から部品として切り出した。cdylib を含むクレートを依存先にすると
//!  依存先の側でもパニック時の処理が要り、複数のアクターで重複するため)

#![cfg_attr(target_arch = "wasm32", no_std)]

// ---------- 容量 ----------

/// パラメータ数 (重み+バイアス) の上限。フル版 8192、縮小版 (実機向け) 2048
#[cfg(not(feature = "small"))]
pub const PARAM_MAX: usize = 8192;
#[cfg(feature = "small")]
pub const PARAM_MAX: usize = 2048;

/// 層の数の上限
pub const LAYER_MAX: usize = 16;
/// 1層の幅 (入力・出力の次元) の上限
pub const WIDTH_MAX: usize = 256;
/// 入力・出力の領域の容量 (f64 の個数)
#[cfg(not(feature = "small"))]
pub const IO_CAP: usize = 1024;
#[cfg(feature = "small")]
pub const IO_CAP: usize = 256;

pub const HEADER_LEN: usize = 24;
pub const LAYER_DESC_LEN: usize = 16;
/// 読み込み領域の容量: 最大の層数と、全パラメータを f64 で持つ場合が入る大きさ
pub const LOAD_CAP: usize = HEADER_LEN + LAYER_DESC_LEN * LAYER_MAX + 8 * PARAM_MAX;

pub const MAGIC: [u8; 4] = *b"DYNM";
pub const VERSION: u16 = 1;
pub const FLAG_F32: u16 = 1;

pub const OP_DENSE: u32 = 1;
pub const ACT_NONE: u32 = 0;
pub const ACT_TANH: u32 = 1;
pub const ACT_RELU: u32 = 2;
pub const ACT_SIGMOID: u32 = 3;

// dyne_model_load の戻り値
pub const LOAD_OK: i32 = 0;
pub const LOAD_BAD_LEN: i32 = -1;
pub const LOAD_BAD_MAGIC: i32 = -2;
pub const LOAD_BAD_VERSION: i32 = -3;
pub const LOAD_BAD_FLAGS: i32 = -4;
pub const LOAD_BAD_LAYERS: i32 = -5;
pub const LOAD_BAD_SHAPE: i32 = -6;
pub const LOAD_BAD_OP: i32 = -7;
pub const LOAD_BAD_PARAMS: i32 = -8;
pub const LOAD_SIZE_MISMATCH: i32 = -9;
pub const LOAD_NONFINITE: i32 = -10;

// dyne_model_eval の戻り値 (負のとき)
pub const EVAL_NO_MODEL: i32 = -1;
pub const EVAL_BAD_INPUT: i32 = -2;
pub const EVAL_TOO_MANY: i32 = -3;

/// 層の記述 (他クレートのアクターからも読めるよう公開。学習用アクターが使う)
#[derive(Clone, Copy)]
pub struct Layer {
    pub n_in: usize,
    pub n_out: usize,
    pub act: u32,
    /// この層の重みの先頭 (パラメータの通し番号)。バイアスは w_off + n_in·n_out から
    pub w_off: usize,
}

const EMPTY_LAYER: Layer = Layer { n_in: 0, n_out: 0, act: 0, w_off: 0 };

pub struct Mlp {
    load: [u8; LOAD_CAP],
    /// 有効なモデルがあるか
    ready: bool,
    f32w: bool,
    n_in: usize,
    n_layers: usize,
    n_params: usize,
    /// 重みの先頭のバイト位置 (load の中)
    data_off: usize,
    layers: [Layer; LAYER_MAX],
    input: [f64; IO_CAP],
    output: [f64; IO_CAP],
    act_a: [f64; WIDTH_MAX],
    act_b: [f64; WIDTH_MAX],
}

pub const fn new_mlp() -> Mlp {
    Mlp {
        load: [0; LOAD_CAP],
        ready: false,
        f32w: false,
        n_in: 0,
        n_layers: 0,
        n_params: 0,
        data_off: 0,
        layers: [EMPTY_LAYER; LAYER_MAX],
        input: [0.0; IO_CAP],
        output: [0.0; IO_CAP],
        act_a: [0.0; WIDTH_MAX],
        act_b: [0.0; WIDTH_MAX],
    }
}


#[inline]
fn rd_u16(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}

#[inline]
fn rd_u32(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

/// 活性化関数 (学習用アクターも同じ実装を使い、推論結果をビット一致させる)
#[inline]
pub fn activate(act: u32, v: f64) -> f64 {
    match act {
        ACT_TANH => libm::tanh(v),
        ACT_RELU => {
            if v > 0.0 {
                v
            } else {
                0.0
            }
        }
        ACT_SIGMOID => 1.0 / (1.0 + libm::exp(-v)),
        _ => v,
    }
}

impl Mlp {
    /// パラメータ k (通し番号) の値
    #[inline]
    pub fn param(&self, k: usize) -> f64 {
        if self.f32w {
            let o = self.data_off + 4 * k;
            f32::from_le_bytes([self.load[o], self.load[o + 1], self.load[o + 2], self.load[o + 3]])
                as f64
        } else {
            let o = self.data_off + 8 * k;
            let mut a = [0u8; 8];
            a.copy_from_slice(&self.load[o..o + 8]);
            f64::from_le_bytes(a)
        }
    }

    /// load[..len] を検証して有効にする。失敗したら以前のモデルも無効になる
    pub fn model_load(&mut self, len: usize) -> i32 {
        self.ready = false;
        if len < HEADER_LEN || len > LOAD_CAP {
            return LOAD_BAD_LEN;
        }
        let b = &self.load[..len];
        if b[0..4] != MAGIC {
            return LOAD_BAD_MAGIC;
        }
        if rd_u16(b, 4) != VERSION {
            return LOAD_BAD_VERSION;
        }
        let flags = rd_u16(b, 6);
        if flags & !FLAG_F32 != 0 {
            return LOAD_BAD_FLAGS;
        }
        let f32w = flags & FLAG_F32 != 0;
        let n_in = rd_u32(b, 8) as usize;
        let n_layers = rd_u32(b, 12) as usize;
        let n_params = rd_u32(b, 16) as usize;
        if rd_u32(b, 20) != 0 {
            return LOAD_BAD_FLAGS;
        }
        if n_layers == 0 || n_layers > LAYER_MAX {
            return LOAD_BAD_LAYERS;
        }
        if n_in == 0 || n_in > WIDTH_MAX {
            return LOAD_BAD_SHAPE;
        }
        let data_off = HEADER_LEN + LAYER_DESC_LEN * n_layers;
        if len < data_off {
            return LOAD_BAD_LEN;
        }
        let mut layers = [EMPTY_LAYER; LAYER_MAX];
        let mut prev = n_in;
        let mut off = 0usize;
        for (l, slot) in layers.iter_mut().enumerate().take(n_layers) {
            let o = HEADER_LEN + LAYER_DESC_LEN * l;
            let op = rd_u32(b, o);
            let li = rd_u32(b, o + 4) as usize;
            let lo = rd_u32(b, o + 8) as usize;
            let act = rd_u32(b, o + 12);
            if op != OP_DENSE || act > ACT_SIGMOID {
                return LOAD_BAD_OP;
            }
            if li != prev || lo == 0 || lo > WIDTH_MAX {
                return LOAD_BAD_SHAPE;
            }
            *slot = Layer { n_in: li, n_out: lo, act, w_off: off };
            off += li * lo + lo;
            if off > PARAM_MAX {
                return LOAD_BAD_PARAMS;
            }
            prev = lo;
        }
        if off != n_params {
            return LOAD_BAD_PARAMS;
        }
        if prev > IO_CAP {
            return LOAD_BAD_SHAPE;
        }
        let elem = if f32w { 4 } else { 8 };
        if len != data_off + elem * n_params {
            return LOAD_SIZE_MISMATCH;
        }
        self.f32w = f32w;
        self.n_in = n_in;
        self.n_layers = n_layers;
        self.n_params = n_params;
        self.data_off = data_off;
        self.layers = layers;
        for k in 0..n_params {
            if !self.param(k).is_finite() {
                return LOAD_NONFINITE;
            }
        }
        self.ready = true;
        LOAD_OK
    }

    /// 読み込み領域 (ホストが .dynm を書き込む先)
    pub fn load_buf_mut(&mut self) -> &mut [u8; LOAD_CAP] {
        &mut self.load
    }

    /// 推論の入力の領域 (f64 が IO_CAP 個) の先頭
    pub fn input_ptr(&self) -> *const f64 {
        self.input.as_ptr()
    }

    /// 推論の出力の領域 (f64 が IO_CAP 個) の先頭
    pub fn output_ptr(&self) -> *const f64 {
        self.output.as_ptr()
    }

    pub fn is_ready(&self) -> bool {
        self.ready
    }

    pub fn n_in(&self) -> usize {
        if self.ready { self.n_in } else { 0 }
    }

    pub fn n_layers(&self) -> usize {
        if self.ready { self.n_layers } else { 0 }
    }

    pub fn n_params(&self) -> usize {
        if self.ready { self.n_params } else { 0 }
    }

    /// 層 l の記述
    pub fn layer(&self, l: usize) -> Layer {
        self.layers[l]
    }

    /// 1点の推論 (入出力の領域を使わない版)。x は n_in 個、y に n_out 個を書く。
    /// 演算順は eval_batch と同じ (結果はビット一致)。戻り値: 出力の次元 / 負=EVAL_*
    pub fn eval_point(&mut self, x: &[f64], y: &mut [f64]) -> i32 {
        if !self.ready {
            return EVAL_NO_MODEL;
        }
        if x.len() != self.n_in || y.len() < self.n_out() {
            return EVAL_BAD_INPUT;
        }
        self.act_a[..self.n_in].copy_from_slice(x);
        self.run_layers();
        let n_out = self.n_out();
        y[..n_out].copy_from_slice(&self.act_a[..n_out]);
        n_out as i32
    }

    /// act_a に置いた入力を全層に通す (結果は act_a に残る)
    fn run_layers(&mut self) {
        for l in 0..self.n_layers {
            let ly = self.layers[l];
            let b_off = ly.w_off + ly.n_in * ly.n_out;
            for j in 0..ly.n_out {
                let mut acc = 0.0f64;
                let w_row = ly.w_off + j * ly.n_in;
                for i in 0..ly.n_in {
                    acc += self.param(w_row + i) * self.act_a[i];
                }
                acc += self.param(b_off + j);
                self.act_b[j] = activate(ly.act, acc);
            }
            core::mem::swap(&mut self.act_a, &mut self.act_b);
        }
    }

    pub fn n_out(&self) -> usize {
        if self.ready {
            self.layers[self.n_layers - 1].n_out
        } else {
            0
        }
    }

    /// 1組の推論: x (n_in 個) → y (n_out 個)
    fn eval_one(&mut self, row: usize) {
        let n_in = self.n_in;
        let n_out = self.n_out();
        for i in 0..n_in {
            self.act_a[i] = self.input[row * n_in + i];
        }
        self.run_layers();
        for j in 0..n_out {
            self.output[row * n_out + j] = self.act_a[j];
        }
    }

    pub fn eval_batch(&mut self, rows: usize) -> i32 {
        if !self.ready {
            return EVAL_NO_MODEL;
        }
        if rows * self.n_in > IO_CAP || rows * self.n_out() > IO_CAP {
            return EVAL_TOO_MANY;
        }
        for r in 0..rows {
            self.eval_one(r);
        }
        self.n_out() as i32
    }
}

// ---------- 単体テスト (ネイティブ) ----------

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::boxed::Box;
    use std::vec::Vec;

    /// テスト用に .dynm を組み立てる。layers = (入力, 出力, 活性化)
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

    fn fresh(bytes: &[u8]) -> (Box<Mlp>, i32) {
        let mut m = Box::new(new_mlp());
        m.load[..bytes.len()].copy_from_slice(bytes);
        let r = m.model_load(bytes.len());
        (m, r)
    }

    // 2→2 (tanh) → 1 (なし)。手計算と照合する
    fn small_params() -> Vec<f64> {
        std::vec![
            0.5, -1.0, // W1 行0
            0.25, 2.0, // W1 行1
            0.1, -0.2, // b1
            1.5, -0.5, // W2
            0.3, // b2
        ]
    }

    #[test]
    fn hand_computed_f64() {
        let bytes = build(2, &[(2, 2, ACT_TANH), (2, 1, ACT_NONE)], &small_params(), false);
        let (mut m, r) = fresh(&bytes);
        assert_eq!(r, LOAD_OK);
        m.input[0] = 0.3;
        m.input[1] = -0.7;
        assert_eq!(m.eval_batch(1), 1);
        let h0 = libm::tanh((0.0 + 0.5 * 0.3 + -1.0 * -0.7) + 0.1);
        let h1 = libm::tanh((0.0 + 0.25 * 0.3 + 2.0 * -0.7) + -0.2);
        let y = (0.0 + 1.5 * h0 + -0.5 * h1) + 0.3;
        assert_eq!(m.output[0].to_bits(), y.to_bits());
    }

    #[test]
    fn f32_weights_and_batch() {
        let p = small_params();
        let bytes = build(2, &[(2, 2, ACT_RELU), (2, 1, ACT_SIGMOID)], &p, true);
        assert_eq!(bytes.len(), HEADER_LEN + 2 * LAYER_DESC_LEN + 4 * p.len());
        let (mut m, r) = fresh(&bytes);
        assert_eq!(r, LOAD_OK);
        for k in 0..3 {
            m.input[2 * k] = 0.1 * k as f64;
            m.input[2 * k + 1] = -0.2 * k as f64;
        }
        assert_eq!(m.eval_batch(3), 1);
        for k in 0..3 {
            let x = [0.1 * k as f64, -0.2 * k as f64];
            let q: Vec<f64> = p.iter().map(|&v| v as f32 as f64).collect();
            let relu = |v: f64| if v > 0.0 { v } else { 0.0 };
            let h0 = relu((0.0 + q[0] * x[0] + q[1] * x[1]) + q[4]);
            let h1 = relu((0.0 + q[2] * x[0] + q[3] * x[1]) + q[5]);
            let z = (0.0 + q[6] * h0 + q[7] * h1) + q[8];
            let y = 1.0 / (1.0 + libm::exp(-z));
            assert_eq!(m.output[k].to_bits(), y.to_bits());
        }
    }

    #[test]
    fn rejects_bad_files() {
        let p = small_params();
        let good = build(2, &[(2, 2, ACT_TANH), (2, 1, ACT_NONE)], &p, false);
        // 識別子
        let mut b = good.clone();
        b[0] = b'X';
        assert_eq!(fresh(&b).1, LOAD_BAD_MAGIC);
        // 版
        let mut b = good.clone();
        b[4] = 9;
        assert_eq!(fresh(&b).1, LOAD_BAD_VERSION);
        // 長さの不一致
        assert_eq!(fresh(&good[..good.len() - 1]).1, LOAD_SIZE_MISMATCH);
        // 層の次元がつながらない
        let b = build(2, &[(2, 2, ACT_TANH), (3, 1, ACT_NONE)], &p, false);
        assert_eq!(fresh(&b).1, LOAD_BAD_SHAPE);
        // 未知の活性化
        let b = build(2, &[(2, 2, 7), (2, 1, ACT_NONE)], &p, false);
        assert_eq!(fresh(&b).1, LOAD_BAD_OP);
        // パラメータ数の申告が違う
        let b = build(2, &[(2, 2, ACT_TANH), (2, 1, ACT_NONE)], &p[..8], false);
        assert_eq!(fresh(&b).1, LOAD_BAD_PARAMS);
        // 非有限の重み
        let mut q = p.clone();
        q[3] = f64::NAN;
        let b = build(2, &[(2, 2, ACT_TANH), (2, 1, ACT_NONE)], &q, false);
        assert_eq!(fresh(&b).1, LOAD_NONFINITE);
        // 上限を超えるパラメータ数
        let big = std::vec![0.0; 256 * 256 + 256];
        let b = build(256, &[(256, 256, ACT_TANH)], &big, false);
        let mut m = Box::new(new_mlp());
        let n = b.len().min(LOAD_CAP);
        m.load[..n].copy_from_slice(&b[..n]);
        assert!(m.model_load(b.len()) < 0);
    }

    #[test]
    fn eval_without_model() {
        let mut m = Box::new(new_mlp());
        assert_eq!(m.eval_batch(1), EVAL_NO_MODEL);
    }
}

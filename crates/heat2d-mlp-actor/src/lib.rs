//! heat2d-mlp-actor: 外部で学習したモデル (ONNX → .dynm) の予測を初期場にして、
//! 2次元熱拡散の区画を解く WASMアクター (2026-11-08)
//!
//! 「取り込んだ ONNX のモデルが、2次元の領域分割の1区画のアクターとして動く」ことを
//! 示す最小の構成。区画の計算・メールボックス方式の受け渡し・差込口はすべて
//! crates/heat2d-actor のもの (dyne_set_face / dyne_init2d / dyne_outbox_read /
//! dyne_inbox_write / dyne_step / dyne_get …) をそのまま出す。足したのは次の3つだけ:
//!
//!   dyne_load_buf() / dyne_load_cap() … .dynm を書き込む領域とその容量
//!   dyne_model_load(len) -> i32 … 読み込んだ .dynm を検証して有効にする (mlp_core::LOAD_*)。
//!       入力2次元 (x, y)・出力1次元のモデルだけを受け付ける (それ以外は −100)
//!   dyne_model_init_field() -> i32 … dyne_init2d の直後に呼ぶと、区画の全格子点の
//!       初期値を u(x, y) = モデル(x, y) に置き換える (値固定の面は再び固定)。
//!       0=成功 / −1=未初期化か既にステップを進めた / −2=非有限の値 / −3=モデルなし
//!
//! 格子点の座標 x = (ix0+i)·h, y = (iy0+j)·h は大域の整数添字から計算し、推論の演算順も
//! 固定なので、区画の切り方によらず同じ点は同じビットの初期値になる。よって分割解と
//! 単一ノード解のビット一致は、heat2d-actor と同じ理由で成り立つ。
//! モデルは各ノードが自分の区画の分だけ評価する (ホストは .dynm を配るだけで推論しない)。
//!
//! no_std / 静的確保 / libm のみ依存。

#![cfg_attr(target_arch = "wasm32", no_std)]

// heat2d-actor の差込口 (#[no_mangle]) とパニック時の処理をこのアクターに含める
pub use heat2d_actor;
use mlp_core::{new_mlp, Mlp, LOAD_CAP, LOAD_OK};

/// 入力・出力の次元が合わないモデル
pub const LOAD_BAD_IO: i32 = -100;
pub const INIT_NO_MODEL: i32 = -3;

static mut M: Mlp = new_mlp();

fn model() -> &'static mut Mlp {
    unsafe { &mut *core::ptr::addr_of_mut!(M) }
}

/// .dynm を検証して有効にし、(x, y) → u の形かを確かめる
pub fn model_load(m: &mut Mlp, len: usize) -> i32 {
    let r = m.model_load(len);
    if r != LOAD_OK {
        return r;
    }
    if m.n_in() != 2 || m.n_out() != 1 {
        // 形が違うモデルは無効にする (以前のモデルも残さない)
        let _ = m.model_load(0);
        return LOAD_BAD_IO;
    }
    LOAD_OK
}

/// 区画の初期場をモデルの予測で置き換える
pub fn init_field(m: &mut Mlp) -> i32 {
    if !m.is_ready() {
        return INIT_NO_MODEL;
    }
    heat2d_actor::replace_initial(|x, y| {
        let mut out = [0.0f64];
        m.eval_point(&[x, y], &mut out);
        out[0]
    })
}

#[no_mangle]
pub extern "C" fn dyne_load_buf() -> u32 {
    model().load_buf_mut().as_ptr() as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_load_cap() -> u32 {
    LOAD_CAP as u32
}

#[no_mangle]
pub extern "C" fn dyne_model_load(len: u32) -> i32 {
    model_load(model(), len as usize)
}

#[no_mangle]
pub extern "C" fn dyne_model_init_field() -> i32 {
    init_field(model())
}

// ---------- 単体テスト (ネイティブ) ----------

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use mlp_core::{ACT_NONE, ACT_TANH, FLAG_F32, MAGIC, OP_DENSE, VERSION};
    use std::boxed::Box;
    use std::vec::Vec;


    fn build(n_in: u32, layers: &[(u32, u32, u32)], params: &[f64]) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&MAGIC);
        v.extend_from_slice(&VERSION.to_le_bytes());
        v.extend_from_slice(&(0u16 & FLAG_F32).to_le_bytes());
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
            v.extend_from_slice(&p.to_le_bytes());
        }
        v
    }

    #[test]
    fn rejects_wrong_shape_and_requires_model() {
        let mut m = Box::new(new_mlp());
        assert_eq!(init_field(&mut m), INIT_NO_MODEL);
        // 1入力のモデルは拒否
        let b = build(1, &[(1, 1, ACT_NONE)], &[1.0, 0.0]);
        m.load_buf_mut()[..b.len()].copy_from_slice(&b);
        assert_eq!(model_load(&mut m, b.len()), LOAD_BAD_IO);
        assert!(!m.is_ready());
        // 2入力1出力は受理
        let b = build(2, &[(2, 2, ACT_TANH), (2, 1, ACT_NONE)], &[0.5, -1.0, 0.25, 2.0, 0.1, -0.2, 1.5, -0.5, 0.3]);
        m.load_buf_mut()[..b.len()].copy_from_slice(&b);
        assert_eq!(model_load(&mut m, b.len()), LOAD_OK);
    }
}

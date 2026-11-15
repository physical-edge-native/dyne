//! mlp-actor: 外部で学習した多層パーセプトロン (MLP、全結合層と活性化関数を
//! 積み重ねたNN) を実行する WASMアクター (2026-11-08)
//!
//! PyTorch 等の重い実行環境で学習したモデルを ONNX (NNの計算グラフと重みの
//! 共通交換形式) で書き出し、scripts/onnx_import.py で「計算グラフ+重み」の
//! 小さな2進ファイル (.dynm) に変換したものを、そのまま読み込んで推論する。
//! マイコン側には ONNX の実行環境を置かない。
//!
//! .dynm 形式・計算の順序・差込口の一覧は crates/mlp-core/src/lib.rs の冒頭にある。
//! 本クレートは mlp-core の部品に差込口 (#[no_mangle] の関数) を付けただけのもの。
//!
//! no_std / 静的確保 / libm のみ依存。

#![cfg_attr(target_arch = "wasm32", no_std)]

pub use mlp_core::*;

static mut S: Mlp = new_mlp();

fn state() -> &'static mut Mlp {
    unsafe { &mut *core::ptr::addr_of_mut!(S) }
}

// ---------- 差込口 ----------

#[no_mangle]
pub extern "C" fn dyne_load_buf() -> u32 {
    state().load_buf_mut().as_ptr() as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_load_cap() -> u32 {
    LOAD_CAP as u32
}

/// 読み込み領域に置かれた len バイトの .dynm を検証して有効にする。戻り値は LOAD_*
#[no_mangle]
pub extern "C" fn dyne_model_load(len: u32) -> i32 {
    state().model_load(len as usize)
}

#[no_mangle]
pub extern "C" fn dyne_in_buf() -> u32 {
    state().input_ptr() as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_out_buf() -> u32 {
    state().output_ptr() as usize as u32
}

/// 入力・出力の領域の容量 (f64 の個数)
#[no_mangle]
pub extern "C" fn dyne_io_cap() -> u32 {
    IO_CAP as u32
}

/// 1組の入力 (n_in 個、モデルの入力次元と一致すること) を推論する。
/// 戻り値: 出力の次元 / 負=EVAL_*
#[no_mangle]
pub extern "C" fn dyne_model_eval(n_in: u32) -> i32 {
    let s = state();
    if s.is_ready() && n_in as usize != s.n_in() {
        return EVAL_BAD_INPUT;
    }
    s.eval_batch(1)
}

/// rows 組をまとめて推論する。戻り値: 出力の次元 / 負=EVAL_*
#[no_mangle]
pub extern "C" fn dyne_model_eval_batch(rows: u32) -> i32 {
    state().eval_batch(rows as usize)
}

#[no_mangle]
pub extern "C" fn dyne_model_n_in() -> u32 {
    let s = state();
    s.n_in() as u32
}

#[no_mangle]
pub extern "C" fn dyne_model_n_out() -> u32 {
    state().n_out() as u32
}

#[no_mangle]
pub extern "C" fn dyne_model_n_layers() -> u32 {
    let s = state();
    s.n_layers() as u32
}

#[no_mangle]
pub extern "C" fn dyne_model_n_params() -> u32 {
    let s = state();
    s.n_params() as u32
}

#[no_mangle]
pub extern "C" fn dyne_param_max() -> u32 {
    PARAM_MAX as u32
}

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    core::arch::wasm32::unreachable()
}

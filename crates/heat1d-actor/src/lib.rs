//! heat1d-actor: 1次元熱伝導方程式のサブドメインを解く WASM アクター (M0)
//!
//! 設計方針 (docs A-3):
//! - アクターは純粋な計算体。通信・時刻・同期を一切知らない。
//! - 通信ロジックはホスト(ランタイムレイヤー)が「注入」する。すなわち、
//!   境界値の出し入れは下記のエクスポート関数を通じてホストが行う。
//! - `#![no_std]`・ヒープ割り当てなし。これは M1 でマイコン(ESP32,
//!   WAMR事前コンパイル)へそのまま持ち込むための制約を先取りしたもの。
//!
//! DyneSolver トレイト (仕様書 A-3) の M0 における C ABI 化:
//!   subdomain()     -> dyne_init(n, x0, x1, alpha, dt, bc_left, bc_right)
//!   step()          -> dyne_step()
//!   boundary_out()  -> dyne_boundary_value(face) / dyne_boundary_grad(face)
//!   (neighbors 入力) -> dyne_set_neighbor(face, value, grad)
//!   状態の観測       -> dyne_len() / dyne_get(i) / dyne_x(i)
//!   状態の書き込み   -> dyne_set(i, value)   (M-S1a追加)
//!
//! 数値法: 空間2次中心差分 + 時間前進オイラー (陽解法)。
//!   u_i^{n+1} = u_i^n + alpha*dt/h^2 * (u_{i-1}^n - 2 u_i^n + u_{i+1}^n)
//! 安定条件: alpha*dt/h^2 <= 1/2 (ホスト側で検証する)。

#![no_std]

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    // no_std のためプロセスを止める術がない。WASM では unreachable でトラップさせる。
    loop {}
}

/// 格子点数の上限 (静的確保。マイコン制約の先取りでヒープを使わない)
const N_MAX: usize = 4096;

/// 境界面 ID
const FACE_LEFT: u32 = 0;
const FACE_RIGHT: u32 = 1;

/// 境界条件タイプ
const BC_DIRICHLET_ZERO: u32 = 0; // u = 0 固定 (グローバル境界)
const BC_INTERFACE: u32 = 1; // 隣接ノードとの内部境界 (ゴースト点を使う)

struct State {
    /// u[0..n]: 担当格子点。u[0] と u[n-1] が自領域の端。
    u: [f64; N_MAX],
    u_next: [f64; N_MAX],
    /// ゴースト点 (隣接ノードの端の値)。face ごとに 1 点。
    ghost: [f64; 2],
    ghost_grad: [f64; 2],
    n: usize,
    h: f64,
    x0: f64,
    alpha: f64,
    dt: f64,
    bc: [u32; 2],
}

static mut STATE: State = State {
    u: [0.0; N_MAX],
    u_next: [0.0; N_MAX],
    ghost: [0.0; 2],
    ghost_grad: [0.0; 2],
    n: 0,
    h: 0.0,
    x0: 0.0,
    alpha: 0.0,
    dt: 0.0,
    bc: [BC_DIRICHLET_ZERO, BC_DIRICHLET_ZERO],
};

#[allow(static_mut_refs)]
fn st() -> &'static mut State {
    // M0 は単一インスタンス・単一スレッド前提 (WASM はシングルスレッド)。
    unsafe { &mut STATE }
}

/// 初期化。担当区間 [x0, x1] を n 点 (両端含む) で離散化し、
/// 初期条件 u(x,0) = sin(pi * x) を設定する。
/// 戻り値: 0 = 成功, 負 = エラー。
#[no_mangle]
pub extern "C" fn dyne_init(
    n: u32,
    x0: f64,
    x1: f64,
    alpha: f64,
    dt: f64,
    bc_left: u32,
    bc_right: u32,
) -> i32 {
    let s = st();
    let n = n as usize;
    if n < 3 || n > N_MAX {
        return -1;
    }
    if !(x1 > x0) || alpha <= 0.0 || dt <= 0.0 {
        return -2;
    }
    s.n = n;
    s.x0 = x0;
    s.h = (x1 - x0) / ((n - 1) as f64);
    s.alpha = alpha;
    s.dt = dt;
    s.bc = [bc_left, bc_right];
    s.ghost = [0.0; 2];
    s.ghost_grad = [0.0; 2];
    for i in 0..n {
        let x = x0 + (i as f64) * s.h;
        s.u[i] = sin_pi(x);
    }
    // グローバル境界は厳密に 0 (sin(0)=sin(pi)=0 だが丸めを消す)
    if bc_left == BC_DIRICHLET_ZERO {
        s.u[0] = 0.0;
    }
    if bc_right == BC_DIRICHLET_ZERO {
        s.u[n - 1] = 0.0;
    }
    0
}

/// 隣接ノードの境界値をホストが注入する。
#[no_mangle]
pub extern "C" fn dyne_set_neighbor(face: u32, value: f64, grad: f64) {
    let s = st();
    if face <= 1 {
        s.ghost[face as usize] = value;
        s.ghost_grad[face as usize] = grad;
    }
}

/// 自領域の端の物理量 (境界面の値)。
#[no_mangle]
pub extern "C" fn dyne_boundary_value(face: u32) -> f64 {
    let s = st();
    match face {
        FACE_LEFT => s.u[0],
        FACE_RIGHT => s.u[s.n - 1],
        _ => f64::NAN,
    }
}

/// 自領域の端の勾配 (片側差分)。プロトコル上 value と共に運ぶ。
#[no_mangle]
pub extern "C" fn dyne_boundary_grad(face: u32) -> f64 {
    let s = st();
    match face {
        FACE_LEFT => (s.u[1] - s.u[0]) / s.h,
        FACE_RIGHT => (s.u[s.n - 1] - s.u[s.n - 2]) / s.h,
        _ => f64::NAN,
    }
}

/// 1 ステップ進める。事前に必要な面へ dyne_set_neighbor 済みであること。
#[no_mangle]
pub extern "C" fn dyne_step() {
    let s = st();
    let r = s.alpha * s.dt / (s.h * s.h);
    let n = s.n;

    // 内部点
    for i in 1..n - 1 {
        s.u_next[i] = s.u[i] + r * (s.u[i - 1] - 2.0 * s.u[i] + s.u[i + 1]);
    }
    // 左端
    s.u_next[0] = match s.bc[0] {
        BC_INTERFACE => s.u[0] + r * (s.ghost[0] - 2.0 * s.u[0] + s.u[1]),
        _ => 0.0, // Dirichlet
    };
    // 右端
    s.u_next[n - 1] = match s.bc[1] {
        BC_INTERFACE => s.u[n - 1] + r * (s.u[n - 2] - 2.0 * s.u[n - 1] + s.ghost[1]),
        _ => 0.0,
    };

    core::mem::swap(&mut s.u, &mut s.u_next);
}

#[no_mangle]
pub extern "C" fn dyne_len() -> u32 {
    st().n as u32
}

#[no_mangle]
pub extern "C" fn dyne_get(i: u32) -> f64 {
    let s = st();
    if (i as usize) < s.n {
        s.u[i as usize]
    } else {
        f64::NAN
    }
}

/// 状態の書き込み (M-S1a追加: 状態移送つき復帰・引き継ぎ用)。
/// dyne_init 後に、ホストが保存済みの値を格子点 i へ書き戻す。
/// 初期条件を上書きするだけで、数値法・境界処理には一切影響しない。
/// これにより「離脱ノードの状態を別インスタンスへ引き渡す」経路が成立する。
#[no_mangle]
pub extern "C" fn dyne_set(i: u32, value: f64) {
    let s = st();
    if (i as usize) < s.n {
        s.u[i as usize] = value;
    }
}

#[no_mangle]
pub extern "C" fn dyne_x(i: u32) -> f64 {
    let s = st();
    s.x0 + (i as f64) * s.h
}

/// sin(pi * x) を no_std で計算する (初期条件専用)。
/// [0,1] を想定し、テイラー展開 (11次) で十分な精度 (~1e-12) を出す。
fn sin_pi(x: f64) -> f64 {
    const PI: f64 = core::f64::consts::PI;
    // sin の対称性で引数を [-pi/2, pi/2] へ簡約
    let mut t = PI * (x - libm_floor(x + 0.5)); // t in [-pi/2, pi/2] 相当
    // x in [0,1] なら t = pi*x - pi*round(x) だが、単純化のため直接:
    t = PI * x;
    // 引数簡約: sin(t) = sin(pi - t)
    if t > PI / 2.0 {
        t = PI - t;
    }
    let t2 = t * t;
    // テイラー: t - t^3/3! + t^5/5! - t^7/7! + t^9/9! - t^11/11!
    t * (1.0
        + t2 * (-1.0 / 6.0
            + t2 * (1.0 / 120.0
                + t2 * (-1.0 / 5040.0 + t2 * (1.0 / 362880.0 + t2 * (-1.0 / 39916800.0))))))
}

fn libm_floor(x: f64) -> f64 {
    let i = x as i64 as f64;
    if x < i { i - 1.0 } else { i }
}
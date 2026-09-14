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

/// 境界条件タイプ (dyne-core::actor の BC_* と一致させる)
const BC_DIRICHLET_ZERO: u32 = 0; // u = 値固定 (既定0。dyne_set_bc_value で変更可)
const BC_INTERFACE: u32 = 1; // 隣接ノードとの内部境界 (ゴースト点を使う)
// ---- F1 (発注3、2026-09-14): 未観測領域境界の設定情報化で追加 ----
const BC_ZERO_GRAD: u32 = 2; // ゼロ勾配 (断熱端)。ゴースト点は鏡映 u_{-1} = u_1
const BC_LINEAR_EXTRAP: u32 = 3; // 線形外挿ゴースト u_{-1} = 2u_0 − u_1 (端点は時間凍結する: 理論ノートT8)
const BC_TRANSPARENT: u32 = 4; // 半無限透過境界 (理論ノートT10)

/// 透過境界の指数和の項数上限 (dyne-core::transparent::SOE_M_MAX と一致させる)
const M_SOE: usize = 64;

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
    /// Dirichlet境界の値 (face ごと)。既定 0.0。init をまたいで保持される
    bc_value: [f64; 2],
    // ---- 透過境界 (BC_TRANSPARENT) の状態 ----
    // 係数 (ホストが dyne_transparent_coef で注入。init をまたいで保持される)
    t_decay: [f64; M_SOE],
    t_half: [f64; M_SOE],
    t_weight: [f64; M_SOE],
    t_m: usize,
    // 補助変数 (漏れバケツ)。**ノード内状態であり可搬状態 u[] には含めない**。
    // 担当替え (handover / ADD に伴う再init) では dyne_init でリセット =
    // 透過境界の再発動として扱う (理論ノートT10、モデルアーキテクチャノートE3注記)
    t_y: [[f64; M_SOE]; 2],
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
    bc_value: [0.0; 2],
    t_decay: [0.0; M_SOE],
    t_half: [0.0; M_SOE],
    t_weight: [0.0; M_SOE],
    t_m: 0,
    t_y: [[0.0; M_SOE]; 2],
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
    // 透過境界の補助変数はリセット (係数 t_decay/t_half/t_weight と bc_value は
    // init をまたいで保持)。担当替え時の再init = 透過境界の再発動 (理論ノートT10)
    s.t_y = [[0.0; M_SOE]; 2];
    for i in 0..n {
        let x = x0 + (i as f64) * s.h;
        s.u[i] = sin_pi(x);
    }
    // Dirichlet境界は厳密に境界値 (既定0。sin(0)=sin(pi)=0 だが丸めを消す)
    if bc_left == BC_DIRICHLET_ZERO {
        s.u[0] = s.bc_value[0];
    }
    if bc_right == BC_DIRICHLET_ZERO {
        s.u[n - 1] = s.bc_value[1];
    }
    0
}

/// Dirichlet境界の値を設定する (F1)。dyne_init の前に呼ぶ (init 時の端点の
/// 丸め落としに使われる)。既定 0.0 = 従来の u=0 固定端とビット同一。
#[no_mangle]
pub extern "C" fn dyne_set_bc_value(face: u32, value: f64) {
    let s = st();
    if face <= 1 {
        s.bc_value[face as usize] = value;
    }
}

/// 透過境界の指数和係数を1項ずつ注入する (F1)。ホストが dyne-core::transparent の
/// 構成 (パラメータ固定・libm) で計算した値をそのまま渡す。init をまたいで保持。
#[no_mangle]
pub extern "C" fn dyne_transparent_coef(j: u32, decay: f64, half: f64, weight: f64) {
    let s = st();
    let j = j as usize;
    if j < M_SOE {
        s.t_decay[j] = decay;
        s.t_half[j] = half;
        s.t_weight[j] = weight;
        if j + 1 > s.t_m {
            s.t_m = j + 1;
        }
    }
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

/// 透過境界の端点更新 (半陰的結合)。戻り値は端点の増分 Δ。
///
/// 陽的結合 (前ステップの境界変化 Δg から流束を作る) は、1/√t 核の t→0 の大きさ
/// ゆえに応答ゲイン ≈ (2/h)√(2αΔt/π) が r≈0.5 で 1 を超え、振動発散する
/// (2026-09-14 実測。理論ノートN10(ii) が予見した「記憶項による増幅」)。
/// そこで当ステップの境界変化 Δ 自身への応答だけを陰的に解く:
///   Δ·(1 + 2rh·S0) = r·(内側ステンシル) − 2rh·K_hist
///   K_hist = Σ_j w_j·(decay_j·y_j)  (過去の変化の寄与を1ステップ減衰させた値)
///   S0     = Σ_j w_j·half_j          (当ステップの変化への即時応答)
///   y_j ← decay_j·y_j + half_j·Δ    (漸化式そのもの: 導出 §5 と同一)
/// 半群性による漸化式は不変で、結合のみ半陰的。K=0 の極限は zero_grad の鏡映に
/// (分母 1+2rh·S0 の減衰を除き) 一致する。総和順は j 昇順の逐次加算に固定 (決定論)。
fn transparent_delta(s: &mut State, face: usize, stencil: f64, r: f64) -> f64 {
    let mut k_hist = 0.0;
    let mut s0 = 0.0;
    for j in 0..s.t_m {
        k_hist += s.t_weight[j] * (s.t_decay[j] * s.t_y[face][j]);
        s0 += s.t_weight[j] * s.t_half[j];
    }
    let c2 = 2.0 * r * s.h;
    let delta = (r * stencil - c2 * k_hist) / (1.0 + c2 * s0);
    for j in 0..s.t_m {
        s.t_y[face][j] = s.t_decay[j] * s.t_y[face][j] + s.t_half[j] * delta;
    }
    delta
}

/// 1 ステップ進める。事前に必要な面へ dyne_set_neighbor 済みであること。
///
/// 未観測領域境界 (F1) の離散化 (複合参照解と式・演算順を厳密に一致させること):
/// - zero_grad: 鏡映ゴースト u_{-1} = u_1 → u_next[0] = u_0 + r(u_1 − 2u_0 + u_1)
/// - linear_extrap: ゴースト u_{-1} = 2u_0 − u_1。端点の離散ラプラシアンが恒等的に
///   0 になり端点は時間凍結する (理論ノートT8。選択はホストが警告する)
/// - transparent: transparent_delta の半陰的結合 (上記)
#[no_mangle]
pub extern "C" fn dyne_step() {
    let s = st();
    let r = s.alpha * s.dt / (s.h * s.h);
    let n = s.n;

    // 端点を先に確定する (transparent は補助変数の更新を伴うため u_next 代入と分離)
    let left = match s.bc[0] {
        BC_INTERFACE => s.u[0] + r * (s.ghost[0] - 2.0 * s.u[0] + s.u[1]),
        BC_ZERO_GRAD => s.u[0] + r * (s.u[1] - 2.0 * s.u[0] + s.u[1]),
        BC_LINEAR_EXTRAP => {
            let gh = 2.0 * s.u[0] - s.u[1];
            s.u[0] + r * (gh - 2.0 * s.u[0] + s.u[1])
        }
        BC_TRANSPARENT => {
            let stencil = s.u[1] - 2.0 * s.u[0] + s.u[1];
            s.u[0] + transparent_delta(s, 0, stencil, r)
        }
        _ => s.bc_value[0], // Dirichlet (既定 0.0)
    };
    let right = match s.bc[1] {
        BC_INTERFACE => s.u[n - 1] + r * (s.u[n - 2] - 2.0 * s.u[n - 1] + s.ghost[1]),
        BC_ZERO_GRAD => s.u[n - 1] + r * (s.u[n - 2] - 2.0 * s.u[n - 1] + s.u[n - 2]),
        BC_LINEAR_EXTRAP => {
            let gh = 2.0 * s.u[n - 1] - s.u[n - 2];
            s.u[n - 1] + r * (s.u[n - 2] - 2.0 * s.u[n - 1] + gh)
        }
        BC_TRANSPARENT => {
            let stencil = s.u[n - 2] - 2.0 * s.u[n - 1] + s.u[n - 2];
            s.u[n - 1] + transparent_delta(s, 1, stencil, r)
        }
        _ => s.bc_value[1],
    };

    // 内部点
    for i in 1..n - 1 {
        s.u_next[i] = s.u[i] + r * (s.u[i - 1] - 2.0 * s.u[i] + s.u[i + 1]);
    }
    s.u_next[0] = left;
    s.u_next[n - 1] = right;

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
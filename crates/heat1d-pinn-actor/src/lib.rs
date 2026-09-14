//! heat1d-pinn-actor: 1次元熱伝導のサブドメインを「時刻離散化PINN」で解く WASM アクター (M-S1b)
//!
//! PINN (物理情報ニューラルネットワーク) = 物理方程式を満たすよう学習するNN。
//! 本アクターは B-16 設計書 4章の「座標入力の領域分解PINN型」を、シミュレータの
//! BSP ステップ同期 (1ステップ = dt だけ時間を進める) と両立させるため、
//! **時刻離散化 (Rothe法)** で構成する (2026-08-25 ユーザー決定):
//!
//! - NN は「現在時刻の u(x)」だけを表す小型MLP (入力 x → 出力 u、隠れ層 tanh)。
//! - 1ステップ = 前ステップの場 u_prev (格子点上の標本) を教師の一部として、
//!   後退オイラー型の残差  r(x) = [u(x) − u_prev(x)] − α·dt·u''(x)  と
//!   境界条件 (外部境界 u=0 / 隣接から届いた界面値) を損失に、Adam を K 回まわす。
//! - 状態の観測・移送は格子点上の標本 (u[]) で行う。差分法アクターと同じ C ABI
//!   (dyne_init / dyne_step / dyne_get / dyne_set …) をそのまま実装するので、
//!   シミュレータからは差分法アクターと差し替え可能 (プロトコル変更ゼロ)。
//!   重みは移送しない = 「場の標本を共通通貨とする知識の持ち運び」(B-16付録、
//!   ADD-PINN のウォームスタートと同じ考え方)。
//!
//! 追加のエクスポート (差分法アクターにはない。ホストは存在しなければ呼ばない):
//!   dyne_cfg(key, value)     … ハイパーパラメータ (dyne_init の前に呼ぶ)
//!   dyne_set_bc(face, bc)    … 境界条件の切り替え (再initせず重みを保つ)
//!   dyne_resize(width) -> i32… 隠れ層の幅変更 = GROW / SHRINK (機能保存)
//!   dyne_eval(i) -> f64      … 現在のNNを格子点 i で評価 (ステップを進めない)
//!
//! 微分の計算: 入力がスカラー x なので、u, u', u'' は前進モード (ジェット) で
//! 1回の順伝播で得る。損失のパラメータ勾配は、そのジェット計算を手書きで
//! 逆伝播する (外部ライブラリなし・ヒープなし・決定論)。
//! no_std / 静的確保 / libm (純Rust) のみ依存。

// wasm32 では no_std (ヒープなし・静的確保)。ネイティブでは同じソースを std 付きで
// ビルドし、単体テストと参照実装に使う (共有クレート方針)。
#![cfg_attr(target_arch = "wasm32", no_std)]

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

/// 格子点数の上限 (差分法アクターは4096。学習用の作業領域があるため小さめ)
pub const N_MAX: usize = 1024;
/// 隠れ層の幅の上限・隠れ層数の上限 (静的確保)
pub const W_MAX: usize = 32;
pub const L_MAX: usize = 3;
/// パラメータ総数 (上限構成で確保。実際に使うのは width/depth の範囲のみ)
pub const P_MAX: usize = W_MAX + L_MAX * W_MAX + (L_MAX - 1) * W_MAX * W_MAX + W_MAX + 1;
/// LM法 (レーベンバーグ・マルカート法) が扱える有効パラメータ数の上限。
/// 正規方程式の行列 (P×P) を静的確保する。パラメータ数 P = W + D·W + (D−1)·W² + W + 1
/// (W=幅, D=隠れ層数)。幅8×2層=97、幅12×2層=193、幅16×1層=49 は収まるが、
/// 幅16×2層=321 は収まらない (その構成は Adam/SGD に落ちる)。
pub const P_LM: usize = 256;

const FACE_LEFT: u32 = 0;
const FACE_RIGHT: u32 = 1;
const BC_DIRICHLET_ZERO: u32 = 0; // 値固定 (既定0。dyne_set_bc_value で変更可)
const BC_INTERFACE: u32 = 1;
// ---- F1 (発注3、2026-09-14): 未観測領域境界の設定情報化で追加。
// 学習型では境界損失の形で実装する ----
const BC_ZERO_GRAD: u32 = 2; // 端点で u' = 0 を損失に課す (断熱端)
const BC_LINEAR_EXTRAP: u32 = 3; // 端点で u'' = 0 を損失に課す (線形延長と等価)
// 透過境界: 差分法アクターの半陰的な端点更新式で u_prev から端点の値目標を作り、
// u(端) = 目標 を損失に課す (iface_mode=1 と同じ「差分法式で端点目標を作る」方式。
// 陽的な流束目標 u'=±K は 1/√t 核の即時応答の硬さで端点が振動することを実測し不採用)
const BC_TRANSPARENT: u32 = 4;

/// 透過境界の指数和の項数上限 (dyne-core::transparent::SOE_M_MAX と一致させる)
pub const M_SOE: usize = 64;

/// dyne_cfg のキー
pub const CFG_ITERS: u32 = 0; // 1ステップあたりの最適化反復回数 (既定 5)
pub const CFG_LR: u32 = 1; // 温め段階の Adam 学習率 (既定 1e-2)
pub const CFG_WARM: u32 = 2; // 初回ステップ前の温め反復回数 (既定 50、0 = 白紙)
pub const CFG_WIDTH: u32 = 3; // 隠れ層の幅 (既定 16、上限 32)
pub const CFG_DEPTH: u32 = 4; // 隠れ層の数 (既定 1、上限 3)
pub const CFG_LAMBDA_B: u32 = 5; // 境界・界面の損失の重み (既定 1.0)
pub const CFG_SEED: u32 = 6; // 重み初期化の乱数シード (既定 1)
pub const CFG_WARM_RESIZE: u32 = 7; // SHRINK 後の温め反復回数 (既定 50)
pub const CFG_LAMBDA_G: u32 = 8; // 界面勾配の一致項の重み (既定 0 = 使わない)
pub const CFG_OPT: u32 = 9; // 最適化器: 0=Adam, 1=SGD(慣性つき), 2=LM法 (既定 2)
pub const CFG_LR_STEP: u32 = 10; // 追従段階の学習率 (既定 0.5、SGD向け)
pub const CFG_MOMENTUM: u32 = 11; // SGDの慣性係数 (既定 0.9)
/// 界面条件の方式 (既定 1):
///  0 = ゴースト点一致: 隣の端点 (自領域の端から h 外側) で u = 隣の前ステップ値。
///      前ステップ値への拘束が1ステップ遅れの偏りを生み、多ノードで誤差が蓄積した。
///  1 = 陽解法端点値: 自分の端点の新しい値を、差分法アクターと同じ陽解法の式
///      u_end + r (ghost − 2 u_end + u_in) で作り、それに u(x_end) を合わせる。
///      使う情報は差分法と同一 (隣の前ステップの端点値のみ)。
pub const CFG_IFACE: u32 = 12;
/// 方程式残差の評価点の細分数 (既定 1 = 格子点のみ)。k のとき各格子間隔を k 等分し、
/// 内部の (k−1) 点を残差評価点に加える。その点の u_prev は前ステップ終了時の
/// NN を評価した値 (u_prev は「前ステップのNNが表す関数」とみなす)。
/// 1ノードあたりの格子点が少ないとき (16ノード分割で5点) の残差評価点不足を補う。
pub const CFG_SUB: u32 = 13;
pub const SUB_MAX: usize = 4;
pub const M_MAX: usize = N_MAX * (SUB_MAX - 1);

// ---- パラメータの平坦配列上の添字 ----
#[inline]
fn idx_in(j: usize) -> usize {
    j
}
#[inline]
fn idx_b(l: usize, j: usize) -> usize {
    W_MAX + l * W_MAX + j
}
#[inline]
fn idx_h(l: usize, j: usize, i: usize) -> usize {
    // 隠れ層 l (1..depth) のニューロン j が、前層のニューロン i から受ける重み
    W_MAX + L_MAX * W_MAX + (l - 1) * W_MAX * W_MAX + j * W_MAX + i
}
#[inline]
fn idx_out(j: usize) -> usize {
    W_MAX + L_MAX * W_MAX + (L_MAX - 1) * W_MAX * W_MAX + j
}
#[inline]
fn idx_c() -> usize {
    idx_out(W_MAX)
}

/// 決定論的乱数 (xorshift64*)
struct Rng(u64);
impl Rng {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    /// [-1, 1) の一様乱数
    fn sym(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64) / ((1u64 << 53) as f64) * 2.0 - 1.0
    }
}

/// 1点の順伝播で保存する中間量 (逆伝播用)。ジェット = (値, x微分, x二階微分)
struct Jet {
    z: [[f64; W_MAX]; L_MAX],
    t: [[f64; W_MAX]; L_MAX],
    z1: [[f64; W_MAX]; L_MAX],
    t1: [[f64; W_MAX]; L_MAX],
    z2: [[f64; W_MAX]; L_MAX],
    t2: [[f64; W_MAX]; L_MAX],
    y: f64,
    y1: f64,
    y2: f64,
}

pub struct Pinn {
    // ---- ハイパーパラメータ ----
    iters: u32,
    lr: f64,
    warm: u32,
    warm_resize: u32,
    width: usize,
    depth: usize,
    lambda_b: f64,
    lambda_g: f64,
    seed: u64,
    opt: u32,
    lr_step: f64,
    momentum: f64,
    iface_mode: u32,
    // ---- NN パラメータと Adam 状態 ----
    p: [f64; P_MAX],
    g: [f64; P_MAX],
    m: [f64; P_MAX],
    v: [f64; P_MAX],
    adam_t: u64,
    // ---- 物理状態 (格子点上の標本) ----
    u: [f64; N_MAX],
    u_prev: [f64; N_MAX],
    /// 細分点上の u_prev (sub>1 のときのみ使用)。添字 = i*(sub−1)+(k−1)
    u_prev_mid: [f64; M_MAX],
    /// u_prev_mid が現在の u_prev と整合しているか (dyne_set/init で偽になる)
    mid_valid: bool,
    sub: usize,
    ghost: [f64; 2],
    ghost_grad: [f64; 2],
    n: usize,
    h: f64,
    x0: f64,
    x1: f64,
    alpha: f64,
    dt: f64,
    bc: [u32; 2],
    /// Dirichlet境界の値 (face ごと)。既定 0.0。init をまたいで保持される
    bc_value: [f64; 2],
    // ---- 透過境界 (BC_TRANSPARENT) の状態。補助変数はノード内状態であり
    // 可搬状態 u[] には含めない。再init でリセット = 再発動 (理論ノートT10) ----
    t_decay: [f64; M_SOE],
    t_half: [f64; M_SOE],
    t_weight: [f64; M_SOE],
    t_m: usize,
    t_y: [[f64; M_SOE]; 2],
    /// 当ステップの透過境界の端点の値目標 (差分法の半陰的更新式で u_prev から作る)
    t_target: [f64; 2],
    /// 次の dyne_step の冒頭で行う温め反復回数 (dyne_init / SHRINK で予約)
    pending_warm: u32,
    jet: Jet,
    // ---- LM法の作業領域 ----
    lm_lambda: f64,
    lm_a: [f64; P_LM * P_LM],
    lm_l: [f64; P_LM * P_LM],
    lm_b: [f64; P_LM],
    lm_delta: [f64; P_LM],
    lm_act: [usize; P_LM],
    lm_na: usize,
    p_backup: [f64; P_MAX],
    /// 直近の最適化反復で得た損失 (計測用)
    pub last_loss: f64,
}

impl Pinn {
    pub const fn new() -> Self {
        Pinn {
            iters: 5,
            lr: 1e-2,
            warm: 50,
            warm_resize: 50,
            width: 16,
            depth: 1,
            lambda_b: 1.0,
            lambda_g: 0.0,
            seed: 1,
            opt: 2,
            lr_step: 0.5,
            momentum: 0.9,
            iface_mode: 1,
            p: [0.0; P_MAX],
            g: [0.0; P_MAX],
            m: [0.0; P_MAX],
            v: [0.0; P_MAX],
            adam_t: 0,
            u: [0.0; N_MAX],
            u_prev: [0.0; N_MAX],
            u_prev_mid: [0.0; M_MAX],
            mid_valid: false,
            sub: 1,
            ghost: [0.0; 2],
            ghost_grad: [0.0; 2],
            n: 0,
            h: 0.0,
            x0: 0.0,
            x1: 1.0,
            alpha: 0.0,
            dt: 0.0,
            bc: [BC_DIRICHLET_ZERO, BC_DIRICHLET_ZERO],
            bc_value: [0.0; 2],
            t_decay: [0.0; M_SOE],
            t_half: [0.0; M_SOE],
            t_weight: [0.0; M_SOE],
            t_m: 0,
            t_y: [[0.0; M_SOE]; 2],
            t_target: [0.0; 2],
            pending_warm: 0,
            lm_lambda: 1e-3,
            lm_a: [0.0; P_LM * P_LM],
            lm_l: [0.0; P_LM * P_LM],
            lm_b: [0.0; P_LM],
            lm_delta: [0.0; P_LM],
            lm_act: [0; P_LM],
            lm_na: 0,
            p_backup: [0.0; P_MAX],
            last_loss: 0.0,
            jet: Jet {
                z: [[0.0; W_MAX]; L_MAX],
                t: [[0.0; W_MAX]; L_MAX],
                z1: [[0.0; W_MAX]; L_MAX],
                t1: [[0.0; W_MAX]; L_MAX],
                z2: [[0.0; W_MAX]; L_MAX],
                t2: [[0.0; W_MAX]; L_MAX],
                y: 0.0,
                y1: 0.0,
                y2: 0.0,
            },
        }
    }

    pub fn cfg(&mut self, key: u32, value: f64) {
        match key {
            CFG_ITERS => self.iters = value as u32,
            CFG_LR => self.lr = value,
            CFG_WARM => self.warm = value as u32,
            CFG_WIDTH => self.width = (value as usize).clamp(1, W_MAX),
            CFG_DEPTH => self.depth = (value as usize).clamp(1, L_MAX),
            CFG_LAMBDA_B => self.lambda_b = value,
            CFG_SEED => self.seed = value as u64,
            CFG_WARM_RESIZE => self.warm_resize = value as u32,
            CFG_LAMBDA_G => self.lambda_g = value,
            CFG_OPT => self.opt = value as u32,
            CFG_LR_STEP => self.lr_step = value,
            CFG_MOMENTUM => self.momentum = value,
            CFG_IFACE => self.iface_mode = value as u32,
            CFG_SUB => self.sub = (value as usize).clamp(1, SUB_MAX),
            _ => {}
        }
    }

    fn rng(&self) -> Rng {
        let s = self.seed.wrapping_mul(0x9E3779B97F4A7C15)
            ^ self.x0.to_bits().rotate_left(17)
            ^ ((self.n as u64) << 40);
        Rng(if s == 0 { 0x1234_5678_9ABC_DEF1 } else { s })
    }

    /// 重み初期化 (Xavier一様)。全パラメータを上限幅まで初期化しておく。
    fn init_params(&mut self) {
        let mut rng = self.rng();
        let w = self.width as f64;
        let lim_in = libm::sqrt(6.0 / (1.0 + w));
        let lim_h = libm::sqrt(6.0 / (2.0 * w));
        let lim_out = libm::sqrt(6.0 / (w + 1.0));
        self.p = [0.0; P_MAX];
        for j in 0..self.width {
            self.p[idx_in(j)] = lim_in * rng.sym();
        }
        for l in 1..self.depth {
            for j in 0..self.width {
                for i in 0..self.width {
                    self.p[idx_h(l, j, i)] = lim_h * rng.sym();
                }
            }
        }
        for j in 0..self.width {
            self.p[idx_out(j)] = lim_out * rng.sym();
        }
        self.reset_adam();
    }

    fn reset_adam(&mut self) {
        self.m = [0.0; P_MAX];
        self.v = [0.0; P_MAX];
        self.adam_t = 0;
    }

    pub fn init(
        &mut self,
        n: u32,
        x0: f64,
        x1: f64,
        alpha: f64,
        dt: f64,
        bc_left: u32,
        bc_right: u32,
    ) -> i32 {
        let n = n as usize;
        if n < 3 || n > N_MAX {
            return -1;
        }
        if !(x1 > x0) || alpha <= 0.0 || dt <= 0.0 {
            return -2;
        }
        self.n = n;
        self.x0 = x0;
        self.x1 = x1;
        self.h = (x1 - x0) / ((n - 1) as f64);
        self.alpha = alpha;
        self.dt = dt;
        self.bc = [bc_left, bc_right];
        self.ghost = [0.0; 2];
        self.ghost_grad = [0.0; 2];
        // 透過境界の補助変数はリセット (係数と bc_value は init をまたいで保持)。
        // 担当替え時の再init = 透過境界の再発動 (理論ノートT10)
        self.t_y = [[0.0; M_SOE]; 2];
        self.t_target = [0.0; 2];
        for i in 0..n {
            let x = x0 + (i as f64) * self.h;
            self.u[i] = sin_pi(x);
        }
        if bc_left == BC_DIRICHLET_ZERO {
            self.u[0] = self.bc_value[0];
        }
        if bc_right == BC_DIRICHLET_ZERO {
            self.u[n - 1] = self.bc_value[1];
        }
        self.u_prev[..n].copy_from_slice(&self.u[..n]);
        self.mid_valid = false;
        self.init_params();
        self.pending_warm = self.warm;
        0
    }

    /// 入力の正規化: ξ = 2(x−x0)/(x1−x0) − 1 ∈ [−1,1]、dξ/dx = 2/(x1−x0)
    #[inline]
    fn xi(&self, x: f64) -> (f64, f64) {
        let sc = 2.0 / (self.x1 - self.x0);
        ((x - self.x0) * sc - 1.0, sc)
    }

    /// 順伝播 (ジェット付き)。結果は self.jet に残る。
    fn forward(&mut self, x: f64) {
        let (xi, sc) = self.xi(x);
        let (w, d) = (self.width, self.depth);
        let jet = &mut self.jet;
        for j in 0..w {
            let z = self.p[idx_in(j)] * xi + self.p[idx_b(0, j)];
            let z1 = self.p[idx_in(j)] * sc;
            let t = libm::tanh(z);
            let s = 1.0 - t * t;
            jet.z[0][j] = z;
            jet.z1[0][j] = z1;
            jet.z2[0][j] = 0.0;
            jet.t[0][j] = t;
            jet.t1[0][j] = s * z1;
            jet.t2[0][j] = -2.0 * t * s * z1 * z1;
        }
        for l in 1..d {
            for j in 0..w {
                let mut z = self.p[idx_b(l, j)];
                let mut z1 = 0.0;
                let mut z2 = 0.0;
                for i in 0..w {
                    let wij = self.p[idx_h(l, j, i)];
                    z += wij * jet.t[l - 1][i];
                    z1 += wij * jet.t1[l - 1][i];
                    z2 += wij * jet.t2[l - 1][i];
                }
                let t = libm::tanh(z);
                let s = 1.0 - t * t;
                jet.z[l][j] = z;
                jet.z1[l][j] = z1;
                jet.z2[l][j] = z2;
                jet.t[l][j] = t;
                jet.t1[l][j] = s * z1;
                jet.t2[l][j] = s * z2 - 2.0 * t * s * z1 * z1;
            }
        }
        let mut y = self.p[idx_c()];
        let mut y1 = 0.0;
        let mut y2 = 0.0;
        for j in 0..w {
            let wo = self.p[idx_out(j)];
            y += wo * jet.t[d - 1][j];
            y1 += wo * jet.t1[d - 1][j];
            y2 += wo * jet.t2[d - 1][j];
        }
        jet.y = y;
        jet.y1 = y1;
        jet.y2 = y2;
    }

    /// 逆伝播: 直前の forward(x) に対し、(∂L/∂y, ∂L/∂y', ∂L/∂y'') から
    /// ∂L/∂p を self.g に加算する。
    fn backward(&mut self, x: f64, yb: f64, yb1: f64, yb2: f64) {
        let (xi, sc) = self.xi(x);
        let (w, d) = (self.width, self.depth);
        let jet = &self.jet;
        let mut tb = [0.0f64; W_MAX];
        let mut tb1 = [0.0f64; W_MAX];
        let mut tb2 = [0.0f64; W_MAX];
        for j in 0..w {
            self.g[idx_out(j)] +=
                yb * jet.t[d - 1][j] + yb1 * jet.t1[d - 1][j] + yb2 * jet.t2[d - 1][j];
            let wo = self.p[idx_out(j)];
            tb[j] = yb * wo;
            tb1[j] = yb1 * wo;
            tb2[j] = yb2 * wo;
        }
        self.g[idx_c()] += yb;
        for l in (0..d).rev() {
            let mut ntb = [0.0f64; W_MAX];
            let mut ntb1 = [0.0f64; W_MAX];
            let mut ntb2 = [0.0f64; W_MAX];
            for j in 0..w {
                let t = jet.t[l][j];
                let s = 1.0 - t * t;
                let z1 = jet.z1[l][j];
                let z2 = jet.z2[l][j];
                // t = tanh(z), t1 = s z1, t2 = s z2 − 2 t s z1²  の各偏微分
                let zb = tb[j] * s
                    + tb1[j] * (-2.0 * t * s * z1)
                    + tb2[j] * (-2.0 * t * s * z2 - 2.0 * z1 * z1 * (s * s - 2.0 * t * t * s));
                let zb1 = tb1[j] * s + tb2[j] * (-4.0 * t * s * z1);
                let zb2 = tb2[j] * s;
                self.g[idx_b(l, j)] += zb;
                if l == 0 {
                    // z = w_in ξ + b, z1 = w_in sc, z2 = 0
                    self.g[idx_in(j)] += zb * xi + zb1 * sc;
                } else {
                    for i in 0..w {
                        let wij = self.p[idx_h(l, j, i)];
                        self.g[idx_h(l, j, i)] +=
                            zb * jet.t[l - 1][i] + zb1 * jet.t1[l - 1][i] + zb2 * jet.t2[l - 1][i];
                        ntb[i] += wij * zb;
                        ntb1[i] += wij * zb1;
                        ntb2[i] += wij * zb2;
                    }
                }
            }
            tb = ntb;
            tb1 = ntb1;
            tb2 = ntb2;
        }
    }

    #[inline]
    fn x_of(&self, i: usize) -> f64 {
        self.x0 + (i as f64) * self.h
    }

    /// 細分点の個数 (sub=1 なら 0)
    fn n_mid(&self) -> usize {
        (self.n - 1) * (self.sub - 1)
    }

    /// 細分点 m の座標
    fn x_mid(&self, m: usize) -> f64 {
        let k = self.sub - 1;
        let i = m / k;
        let j = m % k + 1;
        self.x0 + ((i * self.sub + j) as f64) * (self.h / (self.sub as f64))
    }

    /// 現在のNNを細分点で評価して u_prev_mid に書く
    fn fill_mid(&mut self) {
        for m in 0..self.n_mid() {
            let x = self.x_mid(m);
            self.forward(x);
            self.u_prev_mid[m] = self.jet.y;
        }
        self.mid_valid = true;
    }

    /// 損失と勾配 (self.g) を計算する。
    /// physics=true: 時刻離散化PINNの損失 (方程式残差 + 境界/界面条件)
    /// physics=false: 温め (u_prev への出力一致のみ)
    fn loss_and_grad(&mut self, physics: bool) -> f64 {
        self.g = [0.0; P_MAX];
        let n = self.n;
        let inv_n = 1.0 / (n as f64);
        let mut loss = 0.0;
        for i in 0..n {
            let x = self.x_of(i);
            self.forward(x);
            if physics {
                let r = (self.jet.y - self.u_prev[i]) - self.alpha * self.dt * self.jet.y2;
                loss += r * r * inv_n;
                self.backward(x, 2.0 * r * inv_n, 0.0, -2.0 * r * self.alpha * self.dt * inv_n);
            } else {
                let r = self.jet.y - self.u_prev[i];
                loss += r * r * inv_n;
                self.backward(x, 2.0 * r * inv_n, 0.0, 0.0);
            }
        }
        if physics {
            let nm = self.n_mid();
            let inv_t = 1.0 / ((n + nm) as f64);
            for m in 0..nm {
                let x = self.x_mid(m);
                self.forward(x);
                let r = (self.jet.y - self.u_prev_mid[m]) - self.alpha * self.dt * self.jet.y2;
                loss += r * r * inv_t;
                self.backward(x, 2.0 * r * inv_t, 0.0, -2.0 * r * self.alpha * self.dt * inv_t);
            }
            for face in 0..2 {
                let end = if face == 0 { self.x0 } else { self.x1 };
                // F1: 未観測領域境界は微分量への損失で課す (値の押し付けをしない)
                match self.bc[face] {
                    BC_ZERO_GRAD => {
                        self.forward(end);
                        let rg = self.jet.y1;
                        loss += self.lambda_b * rg * rg;
                        self.backward(end, 0.0, 2.0 * self.lambda_b * rg, 0.0);
                        continue;
                    }
                    BC_LINEAR_EXTRAP => {
                        self.forward(end);
                        let r2 = self.jet.y2;
                        loss += self.lambda_b * r2 * r2;
                        self.backward(end, 0.0, 0.0, 2.0 * self.lambda_b * r2);
                        continue;
                    }
                    BC_TRANSPARENT => {
                        let tgt = self.t_target[face];
                        self.forward(end);
                        let r = self.jet.y - tgt;
                        loss += self.lambda_b * r * r;
                        self.backward(end, 2.0 * self.lambda_b * r, 0.0, 0.0);
                        continue;
                    }
                    _ => {}
                }
                let (x, target, grad) = if self.bc[face] == BC_INTERFACE {
                    // 隣接ノードの端点 = 自領域の端から h だけ外側のゴースト点
                    let xg = if face == 0 { end - self.h } else { end + self.h };
                    (xg, self.ghost[face], Some(self.ghost_grad[face]))
                } else {
                    (end, self.bc_value[face], None)
                };
                self.forward(x);
                let r = self.jet.y - target;
                loss += self.lambda_b * r * r;
                let mut yb1 = 0.0;
                if let Some(gt) = grad {
                    if self.lambda_g != 0.0 {
                        let rg = self.jet.y1 - gt;
                        loss += self.lambda_g * rg * rg;
                        yb1 = 2.0 * self.lambda_g * rg;
                    }
                }
                self.backward(x, 2.0 * self.lambda_b * r, yb1, 0.0);
            }
        }
        loss
    }

    /// Adam 1回分の更新 (有効なパラメータのみ)
    fn adam_step(&mut self, lr: f64) {
        const B1: f64 = 0.9;
        const B2: f64 = 0.999;
        const EPS: f64 = 1e-8;
        self.adam_t += 1;
        let t = self.adam_t as f64;
        let c1 = 1.0 - libm::pow(B1, t);
        let c2 = 1.0 - libm::pow(B2, t);
        let (w, d) = (self.width, self.depth);
        let upd = |k: usize, p: &mut [f64; P_MAX], g: &[f64; P_MAX], m: &mut [f64; P_MAX], v: &mut [f64; P_MAX]| {
            m[k] = B1 * m[k] + (1.0 - B1) * g[k];
            v[k] = B2 * v[k] + (1.0 - B2) * g[k] * g[k];
            let mh = m[k] / c1;
            let vh = v[k] / c2;
            p[k] -= lr * mh / (libm::sqrt(vh) + EPS);
        };
        for j in 0..w {
            upd(idx_in(j), &mut self.p, &self.g, &mut self.m, &mut self.v);
            upd(idx_out(j), &mut self.p, &self.g, &mut self.m, &mut self.v);
            for l in 0..d {
                upd(idx_b(l, j), &mut self.p, &self.g, &mut self.m, &mut self.v);
            }
            for l in 1..d {
                for i in 0..w {
                    upd(idx_h(l, j, i), &mut self.p, &self.g, &mut self.m, &mut self.v);
                }
            }
        }
        upd(idx_c(), &mut self.p, &self.g, &mut self.m, &mut self.v);
    }

    /// 慣性つき勾配降下 1回分 (self.m を速度として流用)。勾配に比例して動くため、
    /// 最小値近傍では動きが止まる (時間追従の微小変化に向く)。
    fn sgd_step(&mut self, lr: f64) {
        let (w, d) = (self.width, self.depth);
        let mu = self.momentum;
        let upd = |k: usize, p: &mut [f64; P_MAX], g: &[f64; P_MAX], m: &mut [f64; P_MAX]| {
            m[k] = mu * m[k] + g[k];
            p[k] -= lr * m[k];
        };
        for j in 0..w {
            upd(idx_in(j), &mut self.p, &self.g, &mut self.m);
            upd(idx_out(j), &mut self.p, &self.g, &mut self.m);
            for l in 0..d {
                upd(idx_b(l, j), &mut self.p, &self.g, &mut self.m);
            }
            for l in 1..d {
                for i in 0..w {
                    upd(idx_h(l, j, i), &mut self.p, &self.g, &mut self.m);
                }
            }
        }
        upd(idx_c(), &mut self.p, &self.g, &mut self.m);
    }

    /// 有効パラメータの添字リストを作る (LM法用)。上限超過なら false。
    fn build_active(&mut self) -> bool {
        let (w, d) = (self.width, self.depth);
        let mut na = 0usize;
        let mut list = [0usize; P_LM];
        let mut push = |k: usize| -> bool {
            if na >= P_LM {
                return false;
            }
            list[na] = k;
            na += 1;
            true
        };
        for j in 0..w {
            if !push(idx_in(j)) {
                return false;
            }
            for l in 0..d {
                if !push(idx_b(l, j)) {
                    return false;
                }
            }
            for l in 1..d {
                for i in 0..w {
                    if !push(idx_h(l, j, i)) {
                        return false;
                    }
                }
            }
            if !push(idx_out(j)) {
                return false;
            }
        }
        if !push(idx_c()) {
            return false;
        }
        self.lm_act = list;
        self.lm_na = na;
        true
    }

    /// 残差 r と、その (y, y', y'') に関する偏微分 (重み付き) を列挙して f に渡す。
    /// physics=true: 方程式残差 + 境界/界面条件、false: 温め (標本一致のみ)。
    /// 損失 = Σ r² となるよう重みは残差側に平方根で掛ける。
    fn for_each_residual(
        &mut self,
        physics: bool,
        mut f: impl FnMut(&mut Self, f64, f64, f64, f64, f64),
    ) {
        let n = self.n;
        let nm = if physics { self.n_mid() } else { 0 };
        let s_n = libm::sqrt(1.0 / ((n + nm) as f64));
        for i in 0..n {
            let x = self.x_of(i);
            self.forward(x);
            if physics {
                let r = s_n * ((self.jet.y - self.u_prev[i]) - self.alpha * self.dt * self.jet.y2);
                let ad = -s_n * self.alpha * self.dt;
                f(self, x, r, s_n, 0.0, ad);
            } else {
                let r = s_n * (self.jet.y - self.u_prev[i]);
                f(self, x, r, s_n, 0.0, 0.0);
            }
        }
        for m in 0..nm {
            let x = self.x_mid(m);
            self.forward(x);
            let r = s_n * ((self.jet.y - self.u_prev_mid[m]) - self.alpha * self.dt * self.jet.y2);
            let ad = -s_n * self.alpha * self.dt;
            f(self, x, r, s_n, 0.0, ad);
        }
        if physics {
            let s_b = libm::sqrt(self.lambda_b);
            let s_g = libm::sqrt(self.lambda_g);
            for face in 0..2 {
                let end = if face == 0 { self.x0 } else { self.x1 };
                // F1: 未観測領域境界は微分量への残差で課す (値の押し付けをしない)
                match self.bc[face] {
                    BC_ZERO_GRAD => {
                        self.forward(end);
                        let rg = s_b * self.jet.y1;
                        f(self, end, rg, 0.0, s_b, 0.0);
                        continue;
                    }
                    BC_LINEAR_EXTRAP => {
                        self.forward(end);
                        let r2 = s_b * self.jet.y2;
                        f(self, end, r2, 0.0, 0.0, s_b);
                        continue;
                    }
                    BC_TRANSPARENT => {
                        let tgt = self.t_target[face];
                        self.forward(end);
                        let r = s_b * (self.jet.y - tgt);
                        f(self, end, r, s_b, 0.0, 0.0);
                        continue;
                    }
                    _ => {}
                }
                let (x, target, grad) = if self.bc[face] == BC_INTERFACE {
                    if self.iface_mode == 1 {
                        let (ie, ii) = if face == 0 { (0, 1) } else { (n - 1, n - 2) };
                        let rc = self.alpha * self.dt / (self.h * self.h);
                        let ue = self.u_prev[ie];
                        let tgt = ue + rc * (self.ghost[face] - 2.0 * ue + self.u_prev[ii]);
                        (end, tgt, None)
                    } else {
                        let xg = if face == 0 { end - self.h } else { end + self.h };
                        (xg, self.ghost[face], Some(self.ghost_grad[face]))
                    }
                } else {
                    (end, self.bc_value[face], None)
                };
                self.forward(x);
                let r = s_b * (self.jet.y - target);
                f(self, x, r, s_b, 0.0, 0.0);
                if let Some(gt) = grad {
                    if self.lambda_g != 0.0 {
                        self.forward(x);
                        let rg = s_g * (self.jet.y1 - gt);
                        f(self, x, rg, 0.0, s_g, 0.0);
                    }
                }
            }
        }
    }

    fn loss_only(&mut self, physics: bool) -> f64 {
        let mut loss = 0.0;
        self.for_each_residual(physics, |_, _, r, _, _, _| loss += r * r);
        loss
    }

    /// LM法 1反復: 正規方程式 (JᵀJ + λ diag(JᵀJ)) δ = −Jᵀr を解いて試行し、
    /// 損失が下がれば採用 (λ を減らす)、上がれば棄却 (λ を増やして再試行)。
    /// 戻り値: 反復後の損失。
    fn lm_step(&mut self, physics: bool) -> f64 {
        let na = self.lm_na;
        for k in 0..na * na {
            self.lm_a[k] = 0.0;
        }
        for k in 0..na {
            self.lm_b[k] = 0.0;
        }
        let mut loss = 0.0;
        // JᵀJ と Jᵀr を1点ずつ積み上げる (J 全体は保持しない)
        self.for_each_residual(physics, |slf, x, r, yb, yb1, yb2| {
            loss += r * r;
            slf.g = [0.0; P_MAX];
            slf.backward(x, yb, yb1, yb2);
            let na = slf.lm_na;
            for a in 0..na {
                let ra = slf.g[slf.lm_act[a]];
                if ra == 0.0 {
                    continue;
                }
                slf.lm_b[a] += ra * r;
                for b in 0..=a {
                    slf.lm_a[a * na + b] += ra * slf.g[slf.lm_act[b]];
                }
            }
        });
        self.p_backup = self.p;
        for _try in 0..8 {
            // 減衰つき正規方程式をコレスキー分解で解く (下三角のみ使用)
            let lam = self.lm_lambda;
            let mut ok = true;
            'outer: for a in 0..na {
                for b in 0..=a {
                    let mut sum = self.lm_a[a * na + b];
                    if a == b {
                        sum += lam * self.lm_a[a * na + a] + 1e-15;
                    }
                    for k in 0..b {
                        sum -= self.lm_l[a * na + k] * self.lm_l[b * na + k];
                    }
                    if a == b {
                        if sum <= 0.0 {
                            ok = false;
                            break 'outer;
                        }
                        self.lm_l[a * na + a] = libm::sqrt(sum);
                    } else {
                        self.lm_l[a * na + b] = sum / self.lm_l[b * na + b];
                    }
                }
            }
            if !ok {
                self.lm_lambda *= 10.0;
                continue;
            }
            // 前進代入 L y = −b、後退代入 Lᵀ δ = y
            for a in 0..na {
                let mut sum = -self.lm_b[a];
                for k in 0..a {
                    sum -= self.lm_l[a * na + k] * self.lm_delta[k];
                }
                self.lm_delta[a] = sum / self.lm_l[a * na + a];
            }
            for a in (0..na).rev() {
                let mut sum = self.lm_delta[a];
                for k in a + 1..na {
                    sum -= self.lm_l[k * na + a] * self.lm_delta[k];
                }
                self.lm_delta[a] = sum / self.lm_l[a * na + a];
            }
            for a in 0..na {
                let k = self.lm_act[a];
                self.p[k] = self.p_backup[k] + self.lm_delta[a];
            }
            let new_loss = self.loss_only(physics);
            if new_loss < loss {
                self.lm_lambda = (self.lm_lambda / 3.0).max(1e-12);
                return new_loss;
            }
            self.p = self.p_backup;
            self.lm_lambda = (self.lm_lambda * 4.0).min(1e12);
        }
        loss
    }

    /// 最適化を iters 回まわす (opt で手法選択)。戻り値: 最終損失。
    fn optimize(&mut self, physics: bool, iters: u32, lr: f64) -> f64 {
        let mut loss = 0.0;
        let use_lm = self.opt == 2 && self.build_active();
        for _ in 0..iters {
            if use_lm {
                loss = self.lm_step(physics);
                if loss < 1e-28 {
                    break;
                }
            } else {
                loss = self.loss_and_grad(physics);
                if self.opt == 1 {
                    self.sgd_step(lr);
                } else {
                    self.adam_step(lr);
                }
            }
        }
        self.last_loss = loss;
        loss
    }

    /// 1 ステップ進める (温めの予約があれば先に消化する)。
    pub fn step(&mut self) {
        let warm = self.pending_warm;
        self.pending_warm = 0;
        if warm > 0 {
            self.reset_adam();
            self.lm_lambda = 1e-3;
            self.optimize(false, warm, self.lr);
            // 温めの最適化器の状態は追従段階へ持ち込まない
            self.reset_adam();
        }
        if self.sub > 1 && !self.mid_valid {
            // 標本が外から書かれた直後 (init / dyne_set / SHRINK温め後): 温め済みの
            // NN を「前ステップの関数」とみなして細分点の u_prev を作る
            self.fill_mid();
        }
        // 透過境界 (F1): 差分法アクターの半陰的な端点更新式 (transparent_delta と
        // 同一の式・同じ総和順) で u_prev から当ステップの端点の値目標を作る。
        // iface_mode=1 の「差分法式で端点目標を作る」方式の透過境界版。
        // ここでは目標の導出のみ行い、補助変数 y は**まだ更新しない**——
        // 履歴更新は最適化後に NN が実際に出力した端点値で行う (下記。
        // 値目標化の等価性3条件の(i)、2026-09-15 理論確認)。
        let mut g_old = [0.0f64; 2];
        for face in 0..2 {
            if self.bc[face] == BC_TRANSPARENT {
                let (ie, ii) = if face == 0 { (0, 1) } else { (self.n - 1, self.n - 2) };
                let rc = self.alpha * self.dt / (self.h * self.h);
                let stencil = self.u_prev[ii] - 2.0 * self.u_prev[ie] + self.u_prev[ii];
                let mut k_hist = 0.0;
                let mut s0 = 0.0;
                for j in 0..self.t_m {
                    k_hist += self.t_weight[j] * (self.t_decay[j] * self.t_y[face][j]);
                    s0 += self.t_weight[j] * self.t_half[j];
                }
                let c2 = 2.0 * rc * self.h;
                let delta = (rc * stencil - c2 * k_hist) / (1.0 + c2 * s0);
                g_old[face] = self.u_prev[ie];
                self.t_target[face] = g_old[face] + delta;
            }
        }
        self.optimize(true, self.iters, self.lr_step);
        for i in 0..self.n {
            let x = self.x_of(i);
            self.forward(x);
            self.u[i] = self.jet.y;
        }
        // 透過境界の履歴更新: 目標値 g* ではなく、最適化後に NN が実際に出力した
        // 端点値の変化 Δ = u_new[端] − u_old[端] で補助変数を進める。目標値で
        // 更新すると「NNが目標に届かなかった分」が履歴に蓄積しドリフトを生む
        // (等価性3条件の(i))。
        for face in 0..2 {
            if self.bc[face] == BC_TRANSPARENT {
                let ie = if face == 0 { 0 } else { self.n - 1 };
                let dg = self.u[ie] - g_old[face];
                for j in 0..self.t_m {
                    self.t_y[face][j] = self.t_decay[j] * self.t_y[face][j] + self.t_half[j] * dg;
                }
            }
        }
        self.u_prev[..self.n].copy_from_slice(&self.u[..self.n]);
        if self.sub > 1 {
            self.fill_mid();
        }
    }

    /// 現在のNNを格子点 i で評価 (状態は変えない)
    pub fn eval(&mut self, i: usize) -> f64 {
        let x = self.x_of(i);
        self.forward(x);
        self.jet.y
    }

    /// 隠れ層の幅の変更 (GROW / SHRINK)。
    /// GROW: 既存重みは据え置き、新ニューロンへの出力重みと既存ニューロンへの
    ///       入力重みは 0 に置く → 関数はビット単位で不変 (厳密な機能保存)。
    ///       新ニューロン自身の入力側の重みは小さな乱数 (学習の対称性破り)。
    /// SHRINK: 末尾のニューロンを落とす (関数は変わる) → 直前の出力 (u_prev) への
    ///         温め (warm_resize 回) を次ステップ冒頭に予約して埋め合わせる。
    pub fn resize(&mut self, width: u32) -> i32 {
        let nw = width as usize;
        if nw < 1 || nw > W_MAX {
            return -1;
        }
        let ow = self.width;
        if nw == ow {
            return 0;
        }
        if nw > ow {
            let mut rng = self.rng();
            // 既に使ったシード列と重ならないよう幅を混ぜる
            rng.0 ^= (nw as u64).wrapping_mul(0xD1B54A32D192ED03);
            let lim_in = 0.1 * libm::sqrt(6.0 / (1.0 + nw as f64));
            let lim_h = 0.1 * libm::sqrt(6.0 / (2.0 * nw as f64));
            for j in ow..nw {
                self.p[idx_in(j)] = lim_in * rng.sym();
                for l in 0..self.depth {
                    self.p[idx_b(l, j)] = 0.0;
                }
                for l in 1..self.depth {
                    for i in 0..nw {
                        self.p[idx_h(l, j, i)] = lim_h * rng.sym();
                    }
                }
                self.p[idx_out(j)] = 0.0;
            }
            // 既存ニューロンが新ニューロンから受ける重みは 0 (機能保存)
            for l in 1..self.depth {
                for j in 0..ow {
                    for i in ow..nw {
                        self.p[idx_h(l, j, i)] = 0.0;
                    }
                }
            }
            for j in ow..nw {
                let ks = [idx_in(j), idx_out(j)];
                for k in ks {
                    self.m[k] = 0.0;
                    self.v[k] = 0.0;
                }
                for l in 0..self.depth {
                    self.m[idx_b(l, j)] = 0.0;
                    self.v[idx_b(l, j)] = 0.0;
                }
                for l in 1..self.depth {
                    for i in 0..nw {
                        self.m[idx_h(l, j, i)] = 0.0;
                        self.v[idx_h(l, j, i)] = 0.0;
                        self.m[idx_h(l, i, j)] = 0.0;
                        self.v[idx_h(l, i, j)] = 0.0;
                    }
                }
            }
            self.width = nw;
        } else {
            self.width = nw;
            self.pending_warm = self.warm_resize;
            self.mid_valid = false;
        }
        0
    }
}

// ---------- 単一インスタンス (WASM はシングルスレッド) ----------

static mut STATE: Pinn = Pinn::new();

#[allow(static_mut_refs)]
fn st() -> &'static mut Pinn {
    unsafe { &mut STATE }
}

#[no_mangle]
pub extern "C" fn dyne_cfg(key: u32, value: f64) {
    st().cfg(key, value);
}

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
    st().init(n, x0, x1, alpha, dt, bc_left, bc_right)
}

#[no_mangle]
pub extern "C" fn dyne_set_neighbor(face: u32, value: f64, grad: f64) {
    let s = st();
    if face <= 1 {
        s.ghost[face as usize] = value;
        s.ghost_grad[face as usize] = grad;
    }
}

#[no_mangle]
pub extern "C" fn dyne_set_bc(face: u32, bc: u32) {
    let s = st();
    if face <= 1 {
        let f = face as usize;
        if s.bc[f] != bc {
            // 境界条件が変わる面の透過境界の補助変数はリセットする
            // (次に BC_TRANSPARENT になったときは再発動から始まる)
            s.t_y[f] = [0.0; M_SOE];
            s.t_target[f] = 0.0;
        }
        s.bc[f] = bc;
    }
}

/// Dirichlet境界の値を設定する (F1)。dyne_init の前に呼ぶ。既定 0.0。
#[no_mangle]
pub extern "C" fn dyne_set_bc_value(face: u32, value: f64) {
    let s = st();
    if face <= 1 {
        s.bc_value[face as usize] = value;
    }
}

/// 透過境界の指数和係数を1項ずつ注入する (F1)。init をまたいで保持される。
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

#[no_mangle]
pub extern "C" fn dyne_boundary_value(face: u32) -> f64 {
    let s = st();
    match face {
        FACE_LEFT => s.u[0],
        FACE_RIGHT => s.u[s.n - 1],
        _ => f64::NAN,
    }
}

/// 端の勾配。差分法アクターと同じ片側差分 (界面残差センサーの比較可能性のため)
#[no_mangle]
pub extern "C" fn dyne_boundary_grad(face: u32) -> f64 {
    let s = st();
    match face {
        FACE_LEFT => (s.u[1] - s.u[0]) / s.h,
        FACE_RIGHT => (s.u[s.n - 1] - s.u[s.n - 2]) / s.h,
        _ => f64::NAN,
    }
}

#[no_mangle]
pub extern "C" fn dyne_step() {
    st().step();
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

/// 状態の書き込み (移送)。格子点の標本を上書きし、次ステップの u_prev にもする。
/// 重みはそのまま (dyne_init 直後なら乱数初期値 → 予約済みの温めで標本に合わせる)。
#[no_mangle]
pub extern "C" fn dyne_set(i: u32, value: f64) {
    let s = st();
    if (i as usize) < s.n {
        s.u[i as usize] = value;
        s.u_prev[i as usize] = value;
        s.mid_valid = false;
    }
}

#[no_mangle]
pub extern "C" fn dyne_x(i: u32) -> f64 {
    st().x_of(i as usize)
}

#[no_mangle]
pub extern "C" fn dyne_eval(i: u32) -> f64 {
    let s = st();
    if (i as usize) < s.n {
        s.eval(i as usize)
    } else {
        f64::NAN
    }
}

#[no_mangle]
pub extern "C" fn dyne_resize(width: u32) -> i32 {
    st().resize(width)
}

/// sin(pi x): 差分法アクターと同一のテイラー展開 (初期条件のビット一致のため)
pub fn sin_pi(x: f64) -> f64 {
    const PI: f64 = core::f64::consts::PI;
    let mut t = PI * x;
    if t > PI / 2.0 {
        t = PI - t;
    }
    let t2 = t * t;
    t * (1.0
        + t2 * (-1.0 / 6.0
            + t2 * (1.0 / 120.0
                + t2 * (-1.0 / 5040.0 + t2 * (1.0 / 362880.0 + t2 * (-1.0 / 39916800.0))))))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fresh(n: u32, bcl: u32, bcr: u32) -> Box<Pinn> {
        let mut p = Box::new(Pinn::new());
        assert_eq!(p.init(n, 0.25, 0.5, 1.0, 1e-4, bcl, bcr), 0);
        p
    }

    /// ジェット (u', u'') を中心差分と照合
    #[test]
    fn jets_match_finite_difference() {
        let mut p = fresh(9, BC_INTERFACE, BC_INTERFACE);
        p.width = 8;
        p.depth = 3;
        p.init_params();
        let x = 0.37;
        let e = 1e-5;
        p.forward(x);
        let (y, y1, y2) = (p.jet.y, p.jet.y1, p.jet.y2);
        p.forward(x + e);
        let yp = p.jet.y;
        p.forward(x - e);
        let ym = p.jet.y;
        let fd1 = (yp - ym) / (2.0 * e);
        let fd2 = (yp - 2.0 * y + ym) / (e * e);
        assert!((y1 - fd1).abs() < 1e-6, "y1={y1} fd={fd1}");
        assert!((y2 - fd2).abs() < 1e-3, "y2={y2} fd={fd2}");
    }

    /// 損失のパラメータ勾配を数値微分と照合 (物理損失、界面条件・勾配項込み)
    #[test]
    fn grad_matches_finite_difference() {
        let mut p = fresh(9, BC_INTERFACE, BC_DIRICHLET_ZERO);
        p.width = 6;
        p.depth = 3;
        p.lambda_g = 0.5;
        p.init_params();
        p.ghost = [0.3, 0.1];
        p.ghost_grad = [1.5, -2.0];
        // 残差項が非自明になるよう u_prev を少しずらす
        for i in 0..p.n {
            p.u_prev[i] *= 0.9;
        }
        p.loss_and_grad(true);
        let g = p.g;
        let e = 1e-6;
        let mut checked = 0;
        for k in 0..P_MAX {
            if g[k] == 0.0 {
                continue;
            }
            let orig = p.p[k];
            p.p[k] = orig + e;
            let lp = p.loss_and_grad(true);
            p.p[k] = orig - e;
            let lm = p.loss_and_grad(true);
            p.p[k] = orig;
            let fd = (lp - lm) / (2.0 * e);
            let tol = 1e-5 * (1.0 + fd.abs());
            assert!((g[k] - fd).abs() < tol, "k={k}: analytic={} fd={fd}", g[k]);
            checked += 1;
        }
        assert!(checked > 30, "checked={checked}");
    }

    /// GROW は関数をビット単位で保存する
    #[test]
    fn grow_preserves_function_bitwise() {
        let mut p = fresh(17, BC_DIRICHLET_ZERO, BC_INTERFACE);
        p.step();
        let before: Vec<f64> = (0..17).map(|i| p.eval(i)).collect();
        assert_eq!(p.resize(24), 0);
        let after: Vec<f64> = (0..17).map(|i| p.eval(i)).collect();
        assert_eq!(before, after);
    }

    /// 単一領域で解析解に近づくこと (温め + 数百ステップ)
    #[test]
    fn single_domain_tracks_analytic() {
        let mut p = Box::new(Pinn::new());
        assert_eq!(p.init(33, 0.0, 1.0, 1.0, 1e-4, 0, 0), 0);
        let steps = 300;
        for _ in 0..steps {
            p.step();
        }
        let t = 1e-4 * steps as f64;
        let decay = (-std::f64::consts::PI.powi(2) * t).exp();
        let mut linf: f64 = 0.0;
        for i in 0..33 {
            let x = i as f64 / 32.0;
            linf = linf.max((p.u[i] - (std::f64::consts::PI * x).sin() * decay).abs());
        }
        assert!(linf < 2e-3, "linf={linf}");
    }
}

#[cfg(test)]
mod diag {
    use super::*;
    #[test]
    #[ignore]
    fn warm_convergence() {
        for &(lr, w, d) in &[(0.0, 8, 2), (0.0, 16, 1), (0.0, 12, 2), (0.0, 32, 1)] {
            let mut p = Box::new(Pinn::new());
            p.lr = lr; p.width = w; p.depth = d; p.warm = 0;
            assert_eq!(p.init(33, 0.0, 1.0, 1.0, 1e-4, 0, 0), 0);
            let mut line = format!("lr={lr} w={w} d={d}:");
            for it in 1..=100 {
                let l = p.optimize(false, 1, lr);
                if it % 10 == 0 { line += &format!(" {it}:{l:.2e}"); }
            }
            println!("{line}");
        }
    }
    #[test]
    #[ignore]
    fn step_tracking() {
        for &(w, d, iters, mom) in &[(8usize, 2usize, 2u32, 0.0), (8, 2, 5, 0.0), (16, 1, 2, 0.0), (16, 1, 5, 0.0), (12, 2, 3, 0.0)] {
            let mut p = Box::new(Pinn::new());
            p.width = w; p.depth = d; p.iters = iters; p.momentum = mom; let (opt, lr) = (w, d);
            let warm = format!("opt={opt} mom={mom}");
            assert_eq!(p.init(33, 0.0, 1.0, 1.0, 1e-4, 0, 0), 0);
            let mut line = format!("lr={lr} iters={iters} warm={warm}:");
            for s in 1..=2000u32 {
                p.step();
                if s % 250 == 0 || s == 1 || s == 10 {
                    let t = 1e-4 * s as f64;
                    let decay = (-std::f64::consts::PI.powi(2) * t).exp();
                    let mut linf: f64 = 0.0;
                    for i in 0..33 { let x = i as f64 / 32.0; let e = (p.u[i] - (std::f64::consts::PI * x).sin() * decay).abs(); if e.is_nan() { linf = f64::NAN; break; } linf = linf.max(e); }
                    line += &format!(" {s}:{linf:.1e}");
                }
            }
            println!("{line}");
        }
    }
}

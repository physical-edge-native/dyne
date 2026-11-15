//! sdf2d-grid-actor: 測距による形状構成の格子型アクター (2026-10-10)
//!
//! 設計書 docs/2026-10-10-shape-grid-actor-design.md。自分の区画の格子点ごとに
//! 「最も近い表面までの符号つき距離」を持ち、測距の光線を統合し、隣と界面を
//! 交換しながら距離の場を更新する。差込口 (メールボックス方式・面の設定・
//! dyne_init2d) は heat2d-actor と同じで、ホスト (dyne-sim2d) は両者を同じ手順で動かす。
//!
//! ## 格子点ごとの状態
//!   d … 統合した切り詰め距離 (−μ〜+μ)。w … 統合の重み (0 = 未観測、上限 w_max)。
//!   φ … 距離の場 (界面で交換するのはこれだけ)。
//!   表面の近く (w>0 かつ |d|<μ) は φ = d (反復の起点)。それ以外は、アイコナール方程式
//!   |∇φ| = 1 の Godunov 型の風上差分で、上下左右の隣の |φ| から求める (Jacobi 反復:
//!   全点を前回の値だけから同時に更新)。符号は観測済みの裏側 (w>0 かつ d<0) なら負。
//!   外周 (界面でない面) の枠は「情報なし」= BIG。
//!
//! ## 光線の統合 (設計書 §1.4)
//!   光線 = 原点 o と終点 e (センサのあるノードが一度だけ計算し、全ノードが同じ値を使う)。
//!   r = |e−o|、v = (e−o)/r。標本点 s_k = k·δ (δ = h/2、k = 0, 1, …、r+μ まで) の
//!   最寄りの格子点 p に d_obs = clamp(r − (p−o)·v, −μ, μ) (格子点を光線へ投影した位置で
//!   測った距離) を重み1で加える。続く標本点が同じ格子点に落ちたら1回だけ数える。
//!   各ノードは最寄りの格子点が自分の区画にある標本点だけを処理する。
//!
//! ## 観測の中継 (設計書 §2、D9 案1)
//!   標本点の列を自分の区画に対する相対位置 (dx, dy ∈ {−1,0,1}) に分類し、
//!   自分の区画の最後の標本点 (自分の区画に一つも無ければ、来た面の側の最後の標本点) の
//!   次の標本点が出ていく先。x方向にはみ出していれば東西の面 (角では x 方向を優先)、
//!   そうでなければ南北の面へ渡す。取得ステップ s の光線は全ノードがステップ s+L で統合し、
//!   期限を過ぎて届いた光線は統合せずに数える (遅着)。
//!
//! ## 1ステップの中の順序 (設計書 §1.5。ビット一致の要)
//!   1. 距離の反復を1回 (前回の φ と枠のゴースト値)
//!   2. 今ステップに受け取った光線 (隣から・自分のセンサから) を取り込む:
//!      統合待ちに積み、区画を出ていくものは出ていく面の送信待ちに積む
//!   3. 期限が今ステップの光線を (取得ステップ, センサID, 区画番号) の昇順で統合する
//!   送信待ちの光線は次のステップの頭の送信メールボックスで、その面の界面サンプル列と
//!   一緒に (同じ界面ID・同じステップで) 出す。
//!
//! ## 古い観測を忘れる仕組み (2026-11-11、docs/2026-11-11-dynamic-shape.md)
//!   物が動いたときに古い形の跡を消し、新しい形を早く出すための設定 (既定ではどれも働かない)。
//!   - 重みの減衰 (CFG_DECAY = λ < 1): 手順2と3の間で全格子点の重みを w ← λ·w にする。
//!     見え続けている点は w が 1/(1−λ) 付近で釣り合い、見えなくなった点の重みは減っていく。
//!     CFG_W_FORGET > 0 なら、重みがそれ未満になった点を未観測 (w = 0, d = 0) に戻す。
//!   - 食い違いの検出 (CFG_DISAGREE = τ > 0): 統合のとき |観測 − d| > τ なら、統合済みの重みを
//!     CFG_DISAGREE_W まで下げてから加える (新しい観測を強く信じる)。
//!   どれも格子点ごとに閉じた計算で、順序も固定なので、分割解と単一ノード解のビット一致は保たれる。
//!
//! no_std / 静的確保 / libm (純Rust) のみ依存。同一入力で全出力がビット一致する。

#![cfg_attr(all(target_arch = "wasm32", not(feature = "std-math")), no_std)]

use dyne_wire as wire;
use dyne_wire::abi::field2d::*;
use dyne_wire::abi::sdf2d::*;
use dyne_wire::Ray;

// ---------- 容量 ----------
// フル版 (シミュレータ用) と実機用の縮小版 (--features small、線形メモリ1ページ=64KB)。
// 縮小版は配列の大きさと、計算結果を変えない省メモリの書き方 (下記) だけが違う。
// 置き場があふれない限り、出力はフル版とビット一致する (あふれは dyne_overflow で数える)。

#[cfg(not(feature = "small"))]
const PAD_MAX: usize = 40_000;
#[cfg(not(feature = "small"))]
const SIDE_MAX: usize = 196;
/// 枠なしの格子点の数の上限 (d・w)
#[cfg(not(feature = "small"))]
const CELL_MAX: usize = PAD_MAX;
const MAX_SENSORS: usize = 64;
/// 視野の中の印 (1点1ビット) の語数
const SEEN_WORDS: usize = (CELL_MAX + 31) / 32;
/// 1ステップに受け取れる光線 (隣から+自分のセンサから)
#[cfg(not(feature = "small"))]
const RECV_MAX: usize = 8192;
/// 統合待ちの光線
#[cfg(not(feature = "small"))]
const PEND_MAX: usize = 65536;
/// 面ごとの送信待ちの光線
#[cfg(not(feature = "small"))]
const FWD_MAX: usize = 4096;

// 縮小版: 6畳 (3.6m×2.7m) を刻み 2.5cm (145×109 点) で 4×4 または 5×3 に分けた区画
// (最大 37×28 / 29×37 点) が入る大きさ。光線は部屋全体で1ステップ 64 本 (8区画×8台) 程度を想定。
// 省メモリの書き方: (1) 距離の反復は φ の2枚目を持たず、前の行と今の行の古い値だけを
// 写して同じ式で上書きする (同じ値から同じ演算をするので結果は同じ)。(2) 統合待ちの光線に
// 期限を持たせず、取得ステップ + L で求める (取り込み時に入れていた値と同じ)。
// (3) 期限の来た光線を別の列へ取り出さず、統合待ちの列を安定に並べ替えて先頭から統合する
// (fuse_due の説明)。統合待ちの 304 本は、4×4 分割・測距8台で統合の遅れ L = 7 の間に
// 1ノードが抱える最大 (シミュレータで 234 本) に余裕を持たせた値。
#[cfg(feature = "small")]
const PAD_MAX: usize = 1210;
#[cfg(feature = "small")]
const SIDE_MAX: usize = 40;
#[cfg(feature = "small")]
const CELL_MAX: usize = 1080;
#[cfg(feature = "small")]
const RECV_MAX: usize = 64;
#[cfg(feature = "small")]
const PEND_MAX: usize = 304;
#[cfg(feature = "small")]
const FWD_MAX: usize = 24;
/// 受け渡し用の作業領域: 4面 × (界面サンプル列 + 光線束)
const SCRATCH_CAP: usize = 4 * ((4 + 32 + 8 * SIDE_MAX) + (4 + 32 + wire::RAY_LEN * FWD_MAX));

/// 場と光線の幾何の計算に使う実数の型。既定は f64 (倍精度)。`--features f32` で単精度にする
/// (2026-11-11、docs/2026-11-11-shape-speedup.md。ESP32 は単精度の演算器だけを持ち、倍精度は
/// ソフトウェアの関数呼び出しになる)。f32 版は f64 版と結果が違う (分割解と単一ノード解は、
/// どちらも f32 版で解けばビット一致する)。ワイヤ形式・光線・dyne_cfg・dyne_get は f64 のまま
#[cfg(not(feature = "f32"))]
pub type R = f64;
#[cfg(feature = "f32")]
pub type R = f32;

/// 情報なし (未観測で、どの表面からも距離が伝わっていない)
pub const BIG: R = 1.0e30;
/// 自分のセンサから作った光線の「来た面」
const FROM_SELF: usize = 4;

// ---------- 浮動小数点の基本関数 (2026-11-11、docs/2026-11-11-shape-speedup.md) ----------
// wasm では libm (純Rust) の floor・sqrt が整数演算の長い命令列になり、マイコンのインタプリタで
// 重い。
// - 格子点の添字に使う「floor(x) を整数にしたもの」を、整数への切り捨て (wasm の1命令) と
//   1回の比較で求める (floor_i)。floor は誤差のない演算で、どちらも同じ整数を返す (NaN は 0、
//   範囲外は i64 の端に飽和するのも `floor(x) as i64` と同じ)。結果のビット列は変わらない。
// - sqrt は、機能 std-math を付けると標準ライブラリの sqrt (wasm の sqrt 命令1つ) を使う。
//   IEEE 754 の sqrt は正しく丸めた値と定められ、libm の sqrt も正しく丸めるので、結果の
//   ビット列は同じ。wasm の sqrt 命令を直接使う関数は安定版の Rust では使えない
//   (core::arch::wasm32::f64_sqrt は不安定機能) ので、標準ライブラリを結合する
//   (wasm が約 15KB 大きくなり、線形メモリ1ページに入れるにはスタックを 4KB にする)。
mod fm {
    use super::R;

    #[cfg(not(feature = "f32"))]
    #[inline(always)]
    pub fn fabs(x: R) -> R {
        libm::fabs(x)
    }
    #[cfg(feature = "f32")]
    #[inline(always)]
    pub fn fabs(x: R) -> R {
        libm::fabsf(x)
    }

    #[cfg(all(target_arch = "wasm32", feature = "std-math"))]
    #[inline(always)]
    pub fn sqrt(x: R) -> R {
        x.sqrt()
    }
    #[cfg(all(not(all(target_arch = "wasm32", feature = "std-math")), not(feature = "f32")))]
    #[inline(always)]
    pub fn sqrt(x: R) -> R {
        libm::sqrt(x)
    }
    #[cfg(all(not(all(target_arch = "wasm32", feature = "std-math")), feature = "f32"))]
    #[inline(always)]
    pub fn sqrt(x: R) -> R {
        libm::sqrtf(x)
    }

    /// `floor(x) as i64` と同じ値 (R)
    #[inline(always)]
    pub fn floor_i(x: R) -> i64 {
        if x < i64::MIN as R {
            return i64::MIN;
        }
        let t = x as i64;
        if (t as R) > x { t - 1 } else { t }
    }
}

// ---------- 光線の幾何 (純粋関数。ネイティブの単体テストから直接呼ぶ) ----------

/// 区画と格子の幾何 (光線の標本点の分類に要るものだけ)
#[derive(Clone, Copy, Debug)]
pub struct Geom {
    pub x0: R,
    pub y0: R,
    pub h: R,
    pub ix0: i64,
    pub iy0: i64,
    pub nx: i64,
    pub ny: i64,
}

/// 光線の標本化に要る量 (原点・単位ベクトル・測った距離・標本点の数)
#[derive(Clone, Copy, Debug)]
pub struct Walk {
    pub ox: R,
    pub oy: R,
    pub vx: R,
    pub vy: R,
    pub r: R,
    /// 標本点 k = 0..n_samples
    pub n_samples: u32,
    pub delta: R,
    pub carve: bool,
}

/// 光線から標本化の量を作る。r が小さすぎる光線は None
pub fn walk_of(ray: &Ray, h: R, mu: R) -> Option<Walk> {
    // 光線はワイヤ形式どおり f64。計算の型 R に直してから使う (既定の f64 では何もしない)
    let (ox, oy) = (ray.ox as R, ray.oy as R);
    let dx = ray.ex as R - ox;
    let dy = ray.ey as R - oy;
    let r = fm::sqrt(dx * dx + dy * dy);
    if !(r > 1.0e-6) {
        return None;
    }
    let carve = ray.id & wire::RAY_FLAG_NO_TARGET != 0;
    let delta = 0.5 * h;
    // 標本点は r+μ まで (対象なしを空きとして刻む場合は r−μ まで)
    let reach = if carve { r - mu } else { r + mu };
    if !(reach > 0.0) {
        return None;
    }
    let n_samples = fm::floor_i(reach / delta).clamp(0, u32::MAX as i64) as u32 + 1;
    Some(Walk { ox, oy, vx: dx / r, vy: dy / r, r, n_samples, delta, carve })
}

/// 標本点 k の最寄りの格子点の大域添字
#[inline]
pub fn sample_cell(w: &Walk, g: &Geom, k: u32) -> (i64, i64, R) {
    let s = (k as R) * w.delta;
    let px = w.ox + w.vx * s;
    let py = w.oy + w.vy * s;
    let gi = fm::floor_i((px - g.x0) / g.h + 0.5);
    let gj = fm::floor_i((py - g.y0) / g.h + 0.5);
    (gi, gj, s)
}

/// 大域添字の、区画に対する相対位置 (dx, dy ∈ {−1, 0, 1})
#[inline]
pub fn rel(g: &Geom, gi: i64, gj: i64) -> (i64, i64) {
    let dx = if gi < g.ix0 { -1 } else if gi >= g.ix0 + g.nx { 1 } else { 0 };
    let dy = if gj < g.iy0 { -1 } else if gj >= g.iy0 + g.ny { 1 } else { 0 };
    (dx, dy)
}

/// 面の向き (相対位置)
fn face_offset(face: usize) -> (i64, i64) {
    match face as u32 {
        FACE_W => (-1, 0),
        FACE_E => (1, 0),
        FACE_S => (0, -1),
        _ => (0, 1),
    }
}

/// 標本点のうち、最寄りの格子点が区画に入りうるものの番号の範囲 [ka, kb] (無ければ None)。
/// 区画を格子1点ぶん広げた長方形と光線の交わりから求め、両端に標本点2つずつの余裕を持たせる。
/// 範囲の外の標本点は区画から h 以上離れているので、最寄りの格子点は必ず区画の外
/// (丸めの誤差は 1e-12 m 程度で、余白 h より桁違いに小さい)。標本点の位置と格子点の添字は
/// 範囲の中でも従来と同じ式で求めるので、範囲を使っても結果は変わらない
/// (2026-11-11、docs/2026-11-11-shape-speedup.md。光線1本の処理を「光線の全長」から
/// 「区画の中の長さ」に比例する量に減らす)
pub fn clip_range(w: &Walk, g: &Geom) -> Option<(u32, u32)> {
    let last = w.n_samples - 1;
    let mut s0 = 0.0 as R;
    let mut s1 = (last as R) * w.delta;
    let axes = [
        (w.ox, w.vx, g.x0 + ((g.ix0 as R) - 1.5) * g.h, g.x0 + (((g.ix0 + g.nx) as R) + 0.5) * g.h),
        (w.oy, w.vy, g.y0 + ((g.iy0 as R) - 1.5) * g.h, g.y0 + (((g.iy0 + g.ny) as R) + 0.5) * g.h),
    ];
    for (o, v, lo, hi) in axes {
        if fm::fabs(v) < 1.0e-12 {
            if o < lo || o > hi {
                return None;
            }
        } else {
            let a = (lo - o) / v;
            let b = (hi - o) / v;
            let (a, b) = if a < b { (a, b) } else { (b, a) };
            if a > s0 {
                s0 = a;
            }
            if b < s1 {
                s1 = b;
            }
        }
    }
    if !(s0 <= s1) {
        return None;
    }
    let ka = (fm::floor_i(s0 / w.delta) - 2).max(0);
    let kb = (fm::floor_i(s1 / w.delta) + 2).min(last as i64);
    if ka > kb {
        return None;
    }
    Some((ka as u32, kb as u32))
}

/// 光線の経路の判定: (自分の区画に標本点があるか, 出ていく面)。
/// from_face は光線が来た面 (自分のセンサなら FROM_SELF)。
/// 自分の区画に標本点があれば clip_range の範囲だけを後ろから調べる (最後の自分の標本点が
/// 決まれば出ていく面が決まる)。無ければ全標本点を調べる route_full (来た面の側の最後の
/// 標本点を探す) に任せる。どちらも route_full と同じ答え (単体テスト route_matches_full)
pub fn route(w: &Walk, g: &Geom, from_face: usize) -> (bool, Option<usize>) {
    if let Some((ka, kb)) = clip_range(w, g) {
        let mut k = kb;
        loop {
            let (gi, gj, _) = sample_cell(w, g, k);
            if rel(g, gi, gj) == (0, 0) {
                return (true, exit_face(w, g, k));
            }
            if k == ka {
                break;
            }
            k -= 1;
        }
    }
    route_full(w, g, from_face)
}

/// 標本点 k0 の次の標本点が出ていく面 (無ければ None)
fn exit_face(w: &Walk, g: &Geom, k0: u32) -> Option<usize> {
    if k0 + 1 >= w.n_samples {
        return None;
    }
    let (gi, gj, _) = sample_cell(w, g, k0 + 1);
    let (dx, dy) = rel(g, gi, gj);
    if dx < 0 {
        Some(FACE_W as usize)
    } else if dx > 0 {
        Some(FACE_E as usize)
    } else if dy < 0 {
        Some(FACE_S as usize)
    } else if dy > 0 {
        Some(FACE_N as usize)
    } else {
        None
    }
}

/// 光線の経路の判定 (全標本点を調べる従来の版)
pub fn route_full(w: &Walk, g: &Geom, from_face: usize) -> (bool, Option<usize>) {
    let mut last_own: Option<u32> = None;
    let mut last_from: Option<u32> = None;
    let from_off = if from_face < 4 { Some(face_offset(from_face)) } else { None };
    for k in 0..w.n_samples {
        let (gi, gj, _) = sample_cell(w, g, k);
        let off = rel(g, gi, gj);
        if off == (0, 0) {
            last_own = Some(k);
        } else if Some(off) == from_off && last_own.is_none() {
            last_from = Some(k);
        }
    }
    let owned = last_own.is_some();
    let Some(k0) = last_own.or(last_from) else {
        return (false, None);
    };
    if k0 + 1 >= w.n_samples {
        return (owned, None);
    }
    let (gi, gj, _) = sample_cell(w, g, k0 + 1);
    let (dx, dy) = rel(g, gi, gj);
    let face = if dx < 0 {
        Some(FACE_W as usize)
    } else if dx > 0 {
        Some(FACE_E as usize)
    } else if dy < 0 {
        Some(FACE_S as usize)
    } else if dy > 0 {
        Some(FACE_N as usize)
    } else {
        None
    };
    (owned, face)
}

/// 統合の順序の鍵: (取得ステップ, センサID, 区画番号)
#[inline]
fn ray_key(r: &Ray) -> (u32, u32) {
    (r.acq_step, r.id >> 8)
}

/// 小さな配列の挿入ソート (no_std・安定・決定論)
fn sort_rays(v: &mut [Ray]) {
    for i in 1..v.len() {
        let mut j = i;
        while j > 0 && ray_key(&v[j - 1]) > ray_key(&v[j]) {
            v.swap(j - 1, j);
            j -= 1;
        }
    }
}

// ---------- 状態 ----------

#[derive(Clone, Copy)]
struct Sensor {
    present: bool,
    x: f64,
    y: f64,
    yaw: f64,
}

#[cfg(not(feature = "small"))]
#[derive(Clone, Copy)]
struct Pending {
    ray: Ray,
    due: u64,
}

#[cfg(not(feature = "small"))]
impl Pending {
    const NONE: Pending = Pending { ray: NO_RAY, due: 0 };
    #[inline]
    fn new(ray: Ray, due: u64) -> Self {
        Pending { ray, due }
    }
    #[inline]
    fn due(&self, _delay: u64) -> u64 {
        self.due
    }
}

/// 縮小版: 期限は取得ステップ + L (取り込み時の due と同じ値) で求める
#[cfg(feature = "small")]
#[derive(Clone, Copy)]
struct Pending {
    ray: Ray,
}

#[cfg(feature = "small")]
impl Pending {
    const NONE: Pending = Pending { ray: NO_RAY };
    #[inline]
    fn new(ray: Ray, _due: u64) -> Self {
        Pending { ray }
    }
    #[inline]
    fn due(&self, delay: u64) -> u64 {
        self.ray.acq_step as u64 + delay
    }
}

const NO_RAY: Ray = Ray { ox: 0.0, oy: 0.0, ex: 0.0, ey: 0.0, acq_step: 0, id: 0 };

pub struct Grid {
    nx: usize,
    ny: usize,
    ix0: u32,
    iy0: u32,
    h: R,
    // 設定 (dyne_cfg。init をまたいで保持)
    defaults_set: bool,
    x0: R,
    y0: R,
    mu: R,
    w_max: R,
    delay: u64,
    range_def: u32,
    carve: bool,
    max_range: f64,
    fov: f64,
    /// 古い観測を忘れる仕組み (既定 1 / 0 / 0 / 1 = 働かない)
    decay: R,
    w_forget: R,
    dis_tau: R,
    dis_w: R,
    /// 距離の反復の間引き (n ステップに1回。既定 1 = 毎ステップ、docs/2026-11-11-shape-speedup.md)
    sweep_every: u64,
    zone_row: u32,
    sensor_sel: usize,
    sensors: [Sensor; MAX_SENSORS],
    kind: [u32; 4],
    param: [u64; 4],
    // 場
    /// 枠つき配列 (幅 nx+2、高さ ny+2)
    phi: [R; PAD_MAX],
    #[cfg(not(feature = "small"))]
    phi_next: [R; PAD_MAX],
    /// 縮小版の反復用: 前の行と今の行の古い φ (枠つきの幅)
    #[cfg(feature = "small")]
    row_prev: [R; SIDE_MAX + 2],
    #[cfg(feature = "small")]
    row_cur: [R; SIDE_MAX + 2],
    /// 枠なし (k = j·nx + i)
    d: [R; CELL_MAX],
    w: [R; CELL_MAX],
    ghost: [[R; SIDE_MAX]; 4],
    got: [bool; 4],
    step: u64,
    // 光線
    recv: [Ray; RECV_MAX],
    recv_from: [u8; RECV_MAX],
    n_recv: usize,
    pend: [Pending; PEND_MAX],
    n_pend: usize,
    fwd: [[Ray; FWD_MAX]; 4],
    n_fwd: [usize; 4],
    #[cfg(not(feature = "small"))]
    due_buf: [Ray; PEND_MAX],
    // 受け渡し
    scratch: [u8; SCRATCH_CAP],
    edge: [u8; 8 * SIDE_MAX],
    ray_bytes: [u8; wire::RAY_LEN * FWD_MAX],
    // 計数
    dropped: u64,
    rejected: u64,
    missing: u64,
    late: u64,
    overflow: u64,
    /// 減衰・忘却を視野の中 (そのステップに統合する光線が通る格子点) だけに効かせる (CFG_DECAY_IN_VIEW)
    decay_in_view: bool,
    /// 視野の中だけ減衰するとき、視野の外の格子点の減衰率 (CFG_DECAY_OUT、既定 1 = 保つ)
    decay_out: R,
    /// このステップに光線が通った格子点の印 (1点1ビット。decay_in_view のときだけ使う)
    seen: [u32; SEEN_WORDS],
}

static mut S: Grid = Grid {
    nx: 0,
    ny: 0,
    ix0: 0,
    iy0: 0,
    h: 0.0,
    x0: 0.0,
    y0: 0.0,
    // 既定値は ensure_defaults で入れる (静的領域を全て0にして、wasmのデータ節に
    // 1MB超の配列が載らないようにするため)
    defaults_set: false,
    mu: 0.0,
    w_max: 0.0,
    delay: 0,
    range_def: RANGE_DEF_RADIAL,
    carve: false,
    max_range: 0.0,
    fov: 0.0,
    decay: 0.0,
    w_forget: 0.0,
    dis_tau: 0.0,
    dis_w: 0.0,
    sweep_every: 0,
    zone_row: 0,
    sensor_sel: 0,
    sensors: [Sensor { present: false, x: 0.0, y: 0.0, yaw: 0.0 }; MAX_SENSORS],
    kind: [BC_DIRICHLET; 4],
    param: [0; 4],
    phi: [0.0; PAD_MAX], // init で BIG にする (静的領域を0にしてwasmのデータ節を小さく保つ)
    #[cfg(not(feature = "small"))]
    phi_next: [0.0; PAD_MAX],
    #[cfg(feature = "small")]
    row_prev: [0.0; SIDE_MAX + 2],
    #[cfg(feature = "small")]
    row_cur: [0.0; SIDE_MAX + 2],
    d: [0.0; CELL_MAX],
    w: [0.0; CELL_MAX],
    ghost: [[0.0; SIDE_MAX]; 4],
    got: [false; 4],
    step: 0,
    recv: [NO_RAY; RECV_MAX],
    recv_from: [0; RECV_MAX],
    n_recv: 0,
    pend: [Pending::NONE; PEND_MAX],
    n_pend: 0,
    fwd: [[NO_RAY; FWD_MAX]; 4],
    n_fwd: [0; 4],
    #[cfg(not(feature = "small"))]
    due_buf: [NO_RAY; PEND_MAX],
    scratch: [0; SCRATCH_CAP],
    edge: [0; 8 * SIDE_MAX],
    ray_bytes: [0; wire::RAY_LEN * FWD_MAX],
    dropped: 0,
    rejected: 0,
    missing: 0,
    late: 0,
    overflow: 0,
    decay_in_view: false,
    decay_out: 0.0,
    seen: [0; SEEN_WORDS],
};

fn state() -> &'static mut Grid {
    unsafe { &mut *core::ptr::addr_of_mut!(S) }
}

impl Grid {
    #[inline]
    fn wp(&self) -> usize {
        self.nx + 2
    }

    #[inline]
    fn at(&self, i: isize, j: isize) -> usize {
        ((j + 1) as usize) * self.wp() + (i + 1) as usize
    }

    fn geom(&self) -> Geom {
        Geom {
            x0: self.x0,
            y0: self.y0,
            h: self.h,
            ix0: self.ix0 as i64,
            iy0: self.iy0 as i64,
            nx: self.nx as i64,
            ny: self.ny as i64,
        }
    }

    fn face_len(&self, face: usize) -> usize {
        if face == FACE_W as usize || face == FACE_E as usize {
            self.ny
        } else {
            self.nx
        }
    }

    fn edge_point(&self, face: usize, k: usize) -> (isize, isize) {
        let (nx, ny, k) = (self.nx as isize, self.ny as isize, k as isize);
        match face as u32 {
            FACE_W => (0, k),
            FACE_E => (nx - 1, k),
            FACE_S => (k, 0),
            _ => (k, ny - 1),
        }
    }

    fn halo_point(&self, face: usize, k: usize) -> (isize, isize) {
        let (nx, ny, k) = (self.nx as isize, self.ny as isize, k as isize);
        match face as u32 {
            FACE_W => (-1, k),
            FACE_E => (nx, k),
            FACE_S => (k, -1),
            _ => (k, ny),
        }
    }

    fn ensure_defaults(&mut self) {
        if !self.defaults_set {
            self.defaults_set = true;
            self.mu = 0.15;
            self.w_max = 50.0;
            self.delay = 1;
            self.max_range = 4.0;
            self.fov = core::f64::consts::FRAC_PI_4; // 45°
            self.decay = 1.0;
            self.w_forget = 0.0;
            self.dis_tau = 0.0;
            self.dis_w = 1.0;
            self.sweep_every = 1;
            self.decay_out = 1.0;
        }
    }

    fn cfg(&mut self, key: u32, val: f64) {
        self.ensure_defaults();
        match key {
            CFG_ORIGIN_X => self.x0 = val as R,
            CFG_ORIGIN_Y => self.y0 = val as R,
            CFG_MU => self.mu = val as R,
            CFG_W_MAX => self.w_max = val as R,
            CFG_DELAY => self.delay = val as u64,
            CFG_RANGE_DEF => self.range_def = val as u32,
            CFG_CARVE_NO_TARGET => self.carve = val != 0.0,
            CFG_MAX_RANGE => self.max_range = val,
            CFG_FOV => self.fov = val,
            CFG_ZONE_ROW => self.zone_row = val as u32,
            CFG_DECAY => self.decay = val as R,
            CFG_W_FORGET => self.w_forget = val as R,
            CFG_DISAGREE => self.dis_tau = val as R,
            CFG_DISAGREE_W => self.dis_w = val as R,
            CFG_SWEEP_EVERY => self.sweep_every = val as u64,
            CFG_DECAY_IN_VIEW => self.decay_in_view = val != 0.0,
            CFG_DECAY_OUT => self.decay_out = val as R,
            CFG_SENSOR_SEL => {
                let id = val as usize;
                if id < MAX_SENSORS {
                    self.sensor_sel = id;
                    self.sensors[id].present = true;
                }
            }
            CFG_SENSOR_X => self.sensors[self.sensor_sel].x = val,
            CFG_SENSOR_Y => self.sensors[self.sensor_sel].y = val,
            CFG_SENSOR_YAW => self.sensors[self.sensor_sel].yaw = val,
            _ => {}
        }
    }

    fn init(&mut self, nx: u32, ny: u32, ix0: u32, iy0: u32, h: f64) -> i32 {
        self.ensure_defaults();
        let (nx, ny) = (nx as usize, ny as usize);
        if nx < 2 || ny < 2 || nx > SIDE_MAX || ny > SIDE_MAX || (nx + 2) * (ny + 2) > PAD_MAX || nx * ny > CELL_MAX {
            return -1;
        }
        if !(h > 0.0) || !(self.mu > 0.0) || !(self.w_max >= 1.0) || self.delay < 1 {
            return -2;
        }
        if !(self.decay > 0.0 && self.decay <= 1.0) || !(self.w_forget >= 0.0) || !(self.dis_tau >= 0.0) || !(self.dis_w >= 0.0) || self.sweep_every < 1
            || !(self.decay_out > 0.0 && self.decay_out <= 1.0)
        {
            return -2;
        }
        // 減衰させるなら忘却の閾値は λ 未満であること (1回観測した点 w=1 が次の統合の前に
        // 忘れられて、どの点も平均が育たなくなるのを防ぐ)
        if self.decay < 1.0 && self.w_forget >= self.decay {
            return -2;
        }
        self.nx = nx;
        self.ny = ny;
        self.ix0 = ix0;
        self.iy0 = iy0;
        self.h = h as R;
        // センサは自分の区画の中にあること (光線の経路の判定の前提)
        let g = self.geom();
        for s in self.sensors.iter() {
            if s.present {
                let gi = fm::floor_i((s.x as R - g.x0) / g.h + 0.5);
                let gj = fm::floor_i((s.y as R - g.y0) / g.h + 0.5);
                if rel(&g, gi, gj) != (0, 0) {
                    return -3;
                }
            }
        }
        for v in self.phi.iter_mut() {
            *v = BIG;
        }
        for k in 0..nx * ny {
            self.d[k] = 0.0;
            self.w[k] = 0.0;
        }
        self.ghost = [[BIG; SIDE_MAX]; 4];
        self.got = [false; 4];
        self.step = 0;
        self.n_recv = 0;
        self.n_pend = 0;
        self.n_fwd = [0; 4];
        self.dropped = 0;
        self.rejected = 0;
        self.missing = 0;
        self.late = 0;
        self.overflow = 0;
        0
    }

    // ---- 送信メールボックス ----

    fn outbox_read(&mut self, cap: usize) -> usize {
        let cap = cap.min(SCRATCH_CAP);
        let mut used = 0usize;
        let mut dropped = 0u64;
        for face in 0..4 {
            if self.kind[face] != BC_INTERFACE {
                continue;
            }
            let iface = self.param[face] as u32;
            let n = self.face_len(face);
            for k in 0..n {
                let (i, j) = self.edge_point(face, k);
                let v = self.phi[self.at(i, j)] as f64;
                self.edge[8 * k..8 * k + 8].copy_from_slice(&v.to_bits().to_le_bytes());
            }
            let hdr = wire::Header {
                kind: wire::KIND_IFACE_BAND,
                quantity: wire::QTY_NONE,
                src: iface,
                step: self.step,
                t_us: 0,
            };
            let pl = wire::Payload::IfaceBand { data: &self.edge[..8 * n], count: n as u32, per_point: 1 };
            let mut w = wire::MailboxWriter::new(&mut self.scratch[used..cap]);
            let _ = w.push(&hdr, &pl);
            used += w.used();
            dropped += w.dropped;
            // 送信待ちの光線 (順序を固定してから出す: メッセージのバイト列も決定論にする)
            let nf = self.n_fwd[face];
            if nf > 0 {
                sort_rays(&mut self.fwd[face][..nf]);
                for q in 0..nf {
                    wire::ray_put(&mut self.ray_bytes, q, &self.fwd[face][q]);
                }
                let hdr = wire::Header { kind: wire::KIND_RAYS, ..hdr };
                let pl = wire::Payload::Rays { data: &self.ray_bytes[..wire::RAY_LEN * nf], count: nf as u32 };
                let mut w = wire::MailboxWriter::new(&mut self.scratch[used..cap]);
                let _ = w.push(&hdr, &pl);
                used += w.used();
                dropped += w.dropped;
                self.n_fwd[face] = 0;
            }
        }
        if used + 4 <= cap {
            self.scratch[used..used + 4].copy_from_slice(&0u32.to_le_bytes());
        }
        self.dropped += dropped;
        used
    }

    // ---- 受信メールボックス ----

    fn push_recv(&mut self, ray: Ray, from: usize) {
        if self.n_recv < RECV_MAX {
            self.recv[self.n_recv] = ray;
            self.recv_from[self.n_recv] = from as u8;
            self.n_recv += 1;
        } else {
            self.overflow += 1;
        }
    }

    fn iface_face(&self, src: u32) -> Option<usize> {
        (0..4).find(|&f| self.kind[f] == BC_INTERFACE && self.param[f] as u32 == src)
    }

    /// 測距フレームを光線にする (センサのあるノードで一度だけ。原点と終点を確定する)
    fn frame_to_rays(&mut self, sensor: usize, zones: &[u8], count: u32, cols: u32) {
        let row = self.zone_row;
        if (row + 1) * cols > count {
            self.rejected += 1;
            return;
        }
        let s = self.sensors[sensor];
        for c in 0..cols {
            let (dist, st) = wire::range_zone(zones, (row * cols + c) as usize);
            let off = wire::zone_angle_offset(c, cols, self.fov);
            let th = s.yaw + off;
            let (vx, vy) = (libm::cos(th), libm::sin(th));
            let (r, flag) = if status_valid(st) && dist > 0.0 {
                let r = if self.range_def == RANGE_DEF_PERPENDICULAR { dist / libm::cos(off) } else { dist };
                (r, 0)
            } else if self.carve {
                (self.max_range, wire::RAY_FLAG_NO_TARGET)
            } else {
                continue;
            };
            let ray = Ray {
                ox: s.x,
                oy: s.y,
                ex: s.x + vx * r,
                ey: s.y + vy * r,
                acq_step: self.step as u32,
                id: ((sensor as u32) << 16) | (c << 8) | flag,
            };
            self.push_recv(ray, FROM_SELF);
        }
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
            let start = pos + 4;
            pos += 4 + mlen;
            // 作業領域から読みつつ状態を書き換えるため、一旦ヘッダと種別だけ確かめる
            let (hdr, kind_ok) = match wire::decode(&self.scratch[start..start + mlen]) {
                Ok((h, _, used)) if used == mlen => (h, true),
                _ => (wire::Header { kind: 0, quantity: 0, src: 0, step: 0, t_us: 0 }, false),
            };
            if !kind_ok {
                return INBOX_MALFORMED;
            }
            if hdr.step != self.step {
                self.rejected += 1;
                result = INBOX_REJECTED;
                continue;
            }
            match hdr.kind {
                wire::KIND_IFACE_BAND => {
                    let Some(f) = self.iface_face(hdr.src) else {
                        self.rejected += 1;
                        result = INBOX_REJECTED;
                        continue;
                    };
                    let n = self.face_len(f);
                    let (_, pl, _) = wire::decode(&self.scratch[start..start + mlen]).unwrap();
                    let wire::Payload::IfaceBand { data, count, per_point } = pl else { unreachable!() };
                    if count as usize != n {
                        self.rejected += 1;
                        result = INBOX_REJECTED;
                        continue;
                    }
                    let pp = per_point as usize;
                    for k in 0..n {
                        self.ghost[f][k] = wire::band_get(data, k * pp) as R;
                    }
                    self.got[f] = true;
                }
                wire::KIND_RAYS => {
                    let Some(f) = self.iface_face(hdr.src) else {
                        self.rejected += 1;
                        result = INBOX_REJECTED;
                        continue;
                    };
                    let (_, pl, _) = wire::decode(&self.scratch[start..start + mlen]).unwrap();
                    let wire::Payload::Rays { data, count } = pl else { unreachable!() };
                    for q in 0..count as usize {
                        let ray = wire::ray_get(data, q);
                        if self.n_recv < RECV_MAX {
                            self.recv[self.n_recv] = ray;
                            self.recv_from[self.n_recv] = f as u8;
                            self.n_recv += 1;
                        } else {
                            self.overflow += 1;
                        }
                    }
                }
                wire::KIND_RANGE_FRAME => {
                    let sensor = hdr.src as usize;
                    if sensor >= MAX_SENSORS || !self.sensors[sensor].present {
                        self.rejected += 1;
                        result = INBOX_REJECTED;
                        continue;
                    }
                    // 区画の値を作業領域の外へ写してから光線にする (作業領域の借用と状態の更新を分ける)
                    let (_, pl, _) = wire::decode(&self.scratch[start..start + mlen]).unwrap();
                    let wire::Payload::RangeFrame { data, count, cols } = pl else { unreachable!() };
                    let mut zones = [0u8; wire::ZONE_LEN * 64];
                    if count as usize > 64 {
                        self.rejected += 1;
                        result = INBOX_REJECTED;
                        continue;
                    }
                    zones[..data.len()].copy_from_slice(data);
                    self.frame_to_rays(sensor, &zones[..wire::ZONE_LEN * count as usize], count, cols);
                }
                _ => {
                    self.rejected += 1;
                    result = INBOX_REJECTED;
                }
            }
        }
        result
    }

    // ---- 1ステップ ----

    fn fill_halo(&mut self) {
        for face in 0..4 {
            for k in 0..self.face_len(face) {
                let v = if self.kind[face] == BC_INTERFACE { self.ghost[face][k] } else { BIG };
                let (hi, hj) = self.halo_point(face, k);
                let idx = self.at(hi, hj);
                self.phi[idx] = v;
            }
        }
    }

    /// 距離の反復 (Jacobi)
    #[cfg(not(feature = "small"))]
    fn sweep(&mut self) {
        self.fill_halo();
        let wp = self.wp();
        let h = self.h;
        let mu = self.mu;
        for j in 0..self.ny {
            for i in 0..self.nx {
                let c = (j + 1) * wp + (i + 1);
                let k = j * self.nx + i;
                let (d, w) = (self.d[k], self.w[k]);
                let v = if w > 0.0 && fm::fabs(d) < mu {
                    d
                } else {
                    let a = fmin(fm::fabs(self.phi[c - 1]), fm::fabs(self.phi[c + 1]));
                    let b = fmin(fm::fabs(self.phi[c - wp]), fm::fabs(self.phi[c + wp]));
                    let m = fmin(a, b);
                    let u = if m >= BIG {
                        BIG
                    } else if fm::fabs(a - b) >= h {
                        m + h
                    } else {
                        let t = a - b;
                        (a + b + fm::sqrt(2.0 * h * h - t * t)) * 0.5
                    };
                    if w > 0.0 && d < 0.0 { -u } else { u }
                };
                self.phi_next[c] = v;
            }
        }
        core::mem::swap(&mut self.phi, &mut self.phi_next);
    }

    /// 距離の反復 (Jacobi、縮小版)。フル版と同じ式・同じ入力値で、φ を行ごとに上書きする。
    /// 左右と下 (j−1) の古い値は写し (row_cur・row_prev) から、上 (j+1) はまだ書き換えて
    /// いない φ から読む
    #[cfg(feature = "small")]
    fn sweep(&mut self) {
        self.fill_halo();
        let wp = self.wp();
        let h = self.h;
        let mu = self.mu;
        // 下の枠の行 (j = −1) の古い値
        self.row_prev[..wp].copy_from_slice(&self.phi[..wp]);
        for j in 0..self.ny {
            let base = (j + 1) * wp;
            self.row_cur[..wp].copy_from_slice(&self.phi[base..base + wp]);
            for i in 0..self.nx {
                let c = base + (i + 1);
                let k = j * self.nx + i;
                let (d, w) = (self.d[k], self.w[k]);
                let v = if w > 0.0 && fm::fabs(d) < mu {
                    d
                } else {
                    let a = fmin(fm::fabs(self.row_cur[i]), fm::fabs(self.row_cur[i + 2]));
                    let b = fmin(fm::fabs(self.row_prev[i + 1]), fm::fabs(self.phi[c + wp]));
                    let m = fmin(a, b);
                    let u = if m >= BIG {
                        BIG
                    } else if fm::fabs(a - b) >= h {
                        m + h
                    } else {
                        let t = a - b;
                        (a + b + fm::sqrt(2.0 * h * h - t * t)) * 0.5
                    };
                    if w > 0.0 && d < 0.0 { -u } else { u }
                };
                self.phi[c] = v;
            }
            core::mem::swap(&mut self.row_prev, &mut self.row_cur);
        }
    }

    /// 受け取った光線の取り込み: 統合待ちと送信待ちへ
    fn intake(&mut self) {
        let g = self.geom();
        for q in 0..self.n_recv {
            let ray = self.recv[q];
            let from = self.recv_from[q] as usize;
            let due = ray.acq_step as u64 + self.delay;
            if due < self.step {
                self.late += 1;
                continue;
            }
            let Some(wk) = walk_of(&ray, self.h, self.mu) else { continue };
            let (owned, out) = route(&wk, &g, from);
            if owned {
                if self.n_pend < PEND_MAX {
                    self.pend[self.n_pend] = Pending::new(ray, due);
                    self.n_pend += 1;
                } else {
                    self.overflow += 1;
                }
            }
            if let Some(f) = out {
                if self.kind[f] == BC_INTERFACE {
                    if self.n_fwd[f] < FWD_MAX {
                        self.fwd[f][self.n_fwd[f]] = ray;
                        self.n_fwd[f] += 1;
                    } else {
                        self.overflow += 1;
                    }
                }
            }
        }
        self.n_recv = 0;
    }

    /// 重みの減衰と、重みの小さくなった点の忘却 (既定の設定では何もしない)
    fn decay_weights(&mut self) {
        if self.decay_in_view {
            // 視野の中だけ減衰する設定: ここでは「このステップに光線が通った」印を消すだけ
            // (減衰は fuse_ray が最初の統合の直前に行う)
            let words = (self.nx * self.ny + 31) / 32;
            for v in self.seen[..words].iter_mut() {
                *v = 0;
            }
            return;
        }
        if !(self.decay < 1.0) && !(self.w_forget > 0.0) {
            return;
        }
        let (lam, wf) = (self.decay, self.w_forget);
        for k in 0..self.nx * self.ny {
            let w = self.w[k] * lam;
            if w < wf {
                self.w[k] = 0.0;
                self.d[k] = 0.0;
            } else {
                self.w[k] = w;
            }
        }
    }

    /// 視野の中だけ減衰する設定で、このステップに光線が通らなかった格子点を λ_out で減衰・忘却する
    /// (CFG_DECAY_OUT < 1 のときだけ。統合の後に行う。格子点ごとに閉じた計算で順序も固定)
    fn decay_out_of_view(&mut self) {
        if !self.decay_in_view || !(self.decay_out < 1.0) {
            return;
        }
        let (lam, wf) = (self.decay_out, self.w_forget);
        for k in 0..self.nx * self.ny {
            if self.seen[k >> 5] & (1u32 << (k & 31)) != 0 {
                continue;
            }
            let w = self.w[k] * lam;
            if w < wf {
                self.w[k] = 0.0;
                self.d[k] = 0.0;
            } else {
                self.w[k] = w;
            }
        }
    }

    /// 期限が今ステップの光線を固定の順序で統合する
    #[cfg(not(feature = "small"))]
    fn fuse_due(&mut self) {
        let mut n_due = 0usize;
        let mut keep = 0usize;
        for q in 0..self.n_pend {
            let p = self.pend[q];
            if p.due(self.delay) == self.step {
                self.due_buf[n_due] = p.ray;
                n_due += 1;
            } else {
                self.pend[keep] = p;
                keep += 1;
            }
        }
        self.n_pend = keep;
        sort_rays(&mut self.due_buf[..n_due]);
        for q in 0..n_due {
            let ray = self.due_buf[q];
            self.fuse_ray(&ray);
        }
    }

    /// 縮小版: 統合待ちの列そのものを (取得ステップ, センサID, 区画番号) で安定に並べ替え、
    /// 先頭の期限が今ステップのものを統合して取り除く (作業用の2つ目の列を持たない)。
    /// 統合待ちには取得ステップが s−L 以上の光線しか無い (それより古いものは前のステップで
    /// 統合済み、遅着は取り込まない) ので、期限が今ステップの光線は鍵が最小で、並べ替えの先頭に
    /// 集まる。安定な並べ替えなので、同じ鍵の光線の順は取り込みの順のまま = フル版の
    /// 「取り出してから並べ替え」と同じ順に統合する
    #[cfg(feature = "small")]
    fn fuse_due(&mut self) {
        let n = self.n_pend;
        for i in 1..n {
            let mut j = i;
            while j > 0 && ray_key(&self.pend[j - 1].ray) > ray_key(&self.pend[j].ray) {
                self.pend.swap(j - 1, j);
                j -= 1;
            }
        }
        let mut n_due = 0usize;
        while n_due < n && self.pend[n_due].due(self.delay) == self.step {
            n_due += 1;
        }
        for q in 0..n_due {
            let ray = self.pend[q].ray;
            self.fuse_ray(&ray);
        }
        self.pend.copy_within(n_due..n, 0);
        self.n_pend = n - n_due;
    }

    /// 光線1本の統合 (視野の中だけの減衰の有無で命令列を分ける: 既定の設定の内側の繰り返しに
    /// 判定を足さないため)
    #[inline]
    fn fuse_ray(&mut self, ray: &Ray) {
        if self.decay_in_view {
            self.fuse_ray_impl::<true>(ray)
        } else {
            self.fuse_ray_impl::<false>(ray)
        }
    }

    fn fuse_ray_impl<const IN_VIEW: bool>(&mut self, ray: &Ray) {
        let g = self.geom();
        let mu = self.mu;
        let Some(wk) = walk_of(ray, self.h, mu) else { return };
        // 区画に入りうる標本点だけを調べる (clip_range。範囲の直前の標本点は区画の外なので、
        // 「続く標本点が同じ格子点なら1回だけ」の判定も全標本点を調べたときと同じになる)
        let Some((ka, kb)) = clip_range(&wk, &g) else { return };
        let (lam, wf, dis_tau, dis_w, w_max) = (self.decay, self.w_forget, self.dis_tau, self.dis_w, self.w_max);
        let mut prev: (i64, i64) = (i64::MIN, i64::MIN);
        for k in ka..=kb {
            let (gi, gj, _) = sample_cell(&wk, &g, k);
            // 同じ光線の続く標本点が同じ格子点に落ちたら1回だけ統合する
            if (gi, gj) == prev {
                continue;
            }
            prev = (gi, gj);
            if rel(&g, gi, gj) != (0, 0) {
                continue;
            }
            let li = (gi - g.ix0) as usize;
            let lj = (gj - g.iy0) as usize;
            let idx = lj * self.nx + li;
            // 観測値は格子点そのものを光線へ投影した位置で測る (標本点の位置で測ると、
            // 最寄りの格子点とのずれ (最大 h/√2) がそのまま誤差になるため)
            let gx = g.x0 + (gi as R) * g.h;
            let gy = g.y0 + (gj as R) * g.h;
            let along = (gx - wk.ox) * wk.vx + (gy - wk.oy) * wk.vy;
            let obs = if wk.carve { mu } else { clamp(wk.r - along, -mu, mu) };
            // 視野の中だけの減衰 (CFG_DECAY_IN_VIEW): このステップで最初に光線が通ったときに1回だけ、
            // decay_weights と同じ式で減衰・忘却してから統合する (統合の前に減衰する順序は従来と同じ)
            if IN_VIEW {
                let (wd, bit) = (idx >> 5, 1u32 << (idx & 31));
                if self.seen[wd] & bit == 0 {
                    self.seen[wd] |= bit;
                    let wv = self.w[idx] * lam;
                    if wv < wf {
                        self.w[idx] = 0.0;
                        self.d[idx] = 0.0;
                    } else {
                        self.w[idx] = wv;
                    }
                }
            }
            let mut w = self.w[idx];
            // 食い違いの検出 (既定 dis_tau = 0 では働かない)
            if dis_tau > 0.0 && w > dis_w && fm::fabs(obs - self.d[idx]) > dis_tau {
                w = dis_w;
            }
            self.d[idx] = (self.d[idx] * w + obs) / (w + 1.0);
            self.w[idx] = fmin(w + 1.0, w_max);
        }
    }

    fn step(&mut self) -> i32 {
        let status = self.step_part(0);
        self.step_part(1);
        self.step_part(2);
        status
    }

    /// 1ステップを3つに分けたもの (計測用に dyne_step_part からも呼ぶ。順に 0・1・2 を呼べば
    /// dyne_step と同じ)。0 = 界面の値の確認と距離の反復、1 = 光線の取り込み、
    /// 2 = 重みの減衰・統合・ステップを進める
    fn step_part(&mut self, part: u32) -> i32 {
        match part {
            0 => {
                let mut status = STEP_OK;
                for face in 0..4 {
                    if self.kind[face] == BC_INTERFACE && !self.got[face] {
                        status = STEP_MISSING_GHOST;
                    }
                }
                if status != STEP_OK {
                    self.missing += 1;
                }
                // 反復の間引き (CFG_SWEEP_EVERY = n): ステップ番号が n の倍数のときだけ反復する。
                // ステップ番号は全ノードで同じなので、分割解と単一ノード解で同じ判断になる
                if self.sweep_every <= 1 || self.step % self.sweep_every == 0 {
                    self.sweep();
                }
                status
            }
            1 => {
                self.intake();
                STEP_OK
            }
            _ => {
                self.decay_weights();
                self.fuse_due();
                self.decay_out_of_view();
                self.step += 1;
                self.got = [false; 4];
                STEP_OK
            }
        }
    }

    fn get(&self, k: usize) -> f64 {
        if self.nx == 0 || k >= self.nx * self.ny {
            return f64::NAN;
        }
        let (i, j) = (k % self.nx, k / self.nx);
        self.phi[self.at(i as isize, j as isize)] as f64
    }

    fn get_aux(&self, q: u32, k: usize) -> f64 {
        if self.nx == 0 || k >= self.nx * self.ny {
            return f64::NAN;
        }
        match q {
            AUX_D => self.d[k] as f64,
            AUX_W => self.w[k] as f64,
            _ => self.get(k),
        }
    }
}

#[inline]
fn fmin(a: R, b: R) -> R {
    if a < b { a } else { b }
}

#[inline]
fn clamp(v: R, lo: R, hi: R) -> R {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

// ---------- WASMエクスポート ----------

/// 設定の注入 (キーは dyne_wire::abi::sdf2d::CFG_*。dyne_init2d の前に呼ぶ)
#[no_mangle]
pub extern "C" fn dyne_cfg(key: u32, val: f64) {
    state().cfg(key, val);
}

/// 面の設定 (heat2d-actor と同じ。界面でない面は「情報なし」として扱う)
#[no_mangle]
pub extern "C" fn dyne_set_face(face: u32, kind: u32, param: u64) {
    let s = state();
    if face < 4 {
        s.kind[face as usize] = kind;
        s.param[face as usize] = param;
    }
}

/// 区画の初期化 (差込口は heat2d-actor と同じ。alpha・dt は使わない)。
/// 戻り値: 0=成功 / −1=点数が範囲外 / −2=設定不正 / −3=センサが区画の外
#[no_mangle]
pub extern "C" fn dyne_init2d(nx: u32, ny: u32, ix0: u32, iy0: u32, h: f64, _alpha: f64, _dt: f64) -> i32 {
    state().init(nx, ny, ix0, iy0, h)
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

/// 計測用: 1ステップを3つに分けて呼ぶ (0 = 界面の確認と距離の反復、1 = 光線の取り込み、
/// 2 = 減衰・統合・ステップを進める)。0・1・2 を順に呼べば dyne_step と同じ結果になる。
/// bench-c5 と recfuel が内訳を測るのに使う (2026-11-11、docs/2026-11-11-shape-speedup.md)
#[no_mangle]
pub extern "C" fn dyne_step_part(part: u32) -> i32 {
    state().step_part(part)
}

#[no_mangle]
pub extern "C" fn dyne_len() -> u32 {
    let s = state();
    (s.nx * s.ny) as u32
}

/// 格子点 k = j·nx + i の φ
#[no_mangle]
pub extern "C" fn dyne_get(k: u32) -> f64 {
    state().get(k as usize)
}

/// 格子点 k の補助の量 (AUX_D = 切り詰め距離 d、AUX_W = 重み w)
#[no_mangle]
pub extern "C" fn dyne_get_aux(q: u32, k: u32) -> f64 {
    state().get_aux(q, k as usize)
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

/// 期限を過ぎて届いた光線の数
#[no_mangle]
pub extern "C" fn dyne_late() -> u64 {
    state().late
}

/// 光線の置き場のあふれ (受信・統合待ち・送信待ち)
#[no_mangle]
pub extern "C" fn dyne_overflow() -> u64 {
    state().overflow
}

/// 統合待ちの光線の数
#[no_mangle]
pub extern "C" fn dyne_pending() -> u32 {
    state().n_pend as u32
}

/// 計測用 (縮小版のみ): 距離の反復だけを n 回行う。状態 φ を書き換えるので、
/// 結果の照合を済ませた後にだけ呼ぶこと (bench-c5 が反復と光線の処理の時間を分けるのに使う)
#[cfg(feature = "small")]
#[no_mangle]
pub extern "C" fn dyne_bench_sweep(n: u32) -> u32 {
    let s = state();
    for _ in 0..n {
        s.sweep();
    }
    n
}

#[cfg(all(target_arch = "wasm32", not(feature = "std-math")))]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    core::arch::wasm32::unreachable()
}

// ---------- ネイティブの単体テスト: 中継の経路 ----------

#[cfg(test)]
mod tests {
    use super::*;

    /// px×py に分けた区画の幾何
    fn blocks(nx: i64, ny: i64, px: i64, py: i64, h: f64) -> Vec<Geom> {
        let split = |n: i64, p: i64| -> Vec<(i64, i64)> {
            let (base, extra) = (n / p, n % p);
            let mut v = Vec::new();
            let mut idx = 0;
            for i in 0..p {
                let c = base + if i < extra { 1 } else { 0 };
                v.push((idx, c));
                idx += c;
            }
            v
        };
        let (xs, ys) = (split(nx, px), split(ny, py));
        let mut out = Vec::new();
        for by in 0..py as usize {
            for bx in 0..px as usize {
                out.push(Geom { x0: 0.0, y0: 0.0, h, ix0: xs[bx].0, iy0: ys[by].0, nx: xs[bx].1, ny: ys[by].1 });
            }
        }
        out
    }

    fn owner(bl: &[Geom], gi: i64, gj: i64) -> Option<usize> {
        bl.iter().position(|g| rel(g, gi, gj) == (0, 0))
    }

    /// 中継をたどって光線が届くノードの集合が、標本点を持つノードの集合と一致すること
    fn check_ray(bl: &[Geom], px: usize, ray: &Ray, h: f64, mu: f64) {
        let wk = walk_of(ray, h, mu).unwrap();
        let mut expect = vec![false; bl.len()];
        for k in 0..wk.n_samples {
            let (gi, gj, _) = sample_cell(&wk, &bl[0], k);
            if let Some(o) = owner(bl, gi, gj) {
                expect[o] = true;
            }
        }
        let (gi, gj, _) = sample_cell(&wk, &bl[0], 0);
        let mut node = owner(bl, gi, gj).expect("原点は格子の中");
        let mut from = FROM_SELF;
        let mut reached = vec![false; bl.len()];
        for _ in 0..(2 * bl.len()) {
            let (owned, out) = route(&wk, &bl[node], from);
            if owned {
                reached[node] = true;
            }
            let Some(f) = out else { break };
            let (bx, by) = ((node % px) as i64, (node / px) as i64);
            let (dx, dy) = face_offset(f);
            let (nbx, nby) = (bx + dx, by + dy);
            let py = (bl.len() / px) as i64;
            if nbx < 0 || nby < 0 || nbx >= px as i64 || nby >= py {
                break;
            }
            node = (nby * px as i64 + nbx) as usize;
            from = match f as u32 {
                FACE_W => FACE_E as usize,
                FACE_E => FACE_W as usize,
                FACE_S => FACE_N as usize,
                _ => FACE_S as usize,
            };
        }
        assert_eq!(reached, expect, "ray {:?}", ray);
    }

    #[test]
    fn relay_reaches_every_owner() {
        let h = 0.05;
        let mu = 0.15;
        for &(px, py) in &[(2usize, 2usize), (3, 3), (4, 2), (1, 3)] {
            let bl = blocks(81, 81, px as i64, py as i64, h);
            let mut seed = 0x1234_5678_9abc_def0u64;
            let mut rnd = || {
                seed ^= seed >> 12;
                seed ^= seed << 25;
                seed ^= seed >> 27;
                (seed.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64 / (1u64 << 53) as f64
            };
            for _ in 0..2000 {
                let (ox, oy) = (0.05 + 3.9 * rnd(), 0.05 + 3.9 * rnd());
                let th = 2.0 * core::f64::consts::PI * rnd();
                let r = 0.1 + 4.0 * rnd();
                let ray = Ray { ox, oy, ex: ox + r * th.cos(), ey: oy + r * th.sin(), acq_step: 0, id: 0 };
                check_ray(&bl, px, &ray, h, mu);
            }
            // 角をちょうど通る光線 (区画の境目の格子の対角線)
            let g = &bl[0];
            let cx = (g.ix0 + g.nx) as f64 * h - 0.5 * h;
            let cy = (g.iy0 + g.ny) as f64 * h - 0.5 * h;
            for &(ox, oy) in &[(0.3, 0.3), (cx - 0.7, cy - 0.7), (0.1, cy - 1.2)] {
                let (dx, dy) = (cx - ox, cy - oy);
                let ray = Ray { ox, oy, ex: ox + 2.0 * dx, ey: oy + 2.0 * dy, acq_step: 0, id: 0 };
                check_ray(&bl, px, &ray, h, mu);
            }
        }
    }

    /// clip_range を使う route が全標本点を調べる route_full と同じ答えを返し、範囲の外に
    /// 区画の標本点が無いこと (2026-11-11、docs/2026-11-11-shape-speedup.md)
    #[test]
    fn route_matches_full() {
        for &h in &[0.025, 0.05] {
            for &(px, py) in &[(4usize, 4usize), (5, 3), (3, 3), (1, 3)] {
                let bl = blocks((3.6 / h) as i64 + 1, (2.7 / h) as i64 + 1, px as i64, py as i64, h);
                let mut seed = 0x9e37_79b9_7f4a_7c15u64;
                let mut rnd = || {
                    seed ^= seed >> 12;
                    seed ^= seed << 25;
                    seed ^= seed >> 27;
                    (seed.wrapping_mul(0x2545F4914F6CDD1D) >> 11) as f64 / (1u64 << 53) as f64
                };
                for q in 0..3000 {
                    let (ox, oy) = (3.6 * rnd(), 2.7 * rnd());
                    // 軸に平行な光線と、区画の境目の上を通る光線も混ぜる
                    let th = match q % 5 {
                        0 => 0.5 * core::f64::consts::PI * ((q / 5) % 4) as f64,
                        _ => 2.0 * core::f64::consts::PI * rnd(),
                    };
                    let (ox, oy) = if q % 7 == 0 { (bl[1].ix0 as f64 * h - 0.5 * h, oy) } else { (ox, oy) };
                    let r = 0.05 + 4.0 * rnd();
                    let id = if q % 3 == 0 { wire::RAY_FLAG_NO_TARGET } else { 0 };
                    let ray = Ray { ox, oy, ex: ox + r * th.cos(), ey: oy + r * th.sin(), acq_step: 0, id };
                    let Some(wk) = walk_of(&ray, h, 0.15) else { continue };
                    for g in &bl {
                        for from in 0..5 {
                            assert_eq!(route(&wk, g, from), route_full(&wk, g, from), "ray {:?}", ray);
                        }
                        let clip = clip_range(&wk, g);
                        for k in 0..wk.n_samples {
                            let (gi, gj, _) = sample_cell(&wk, g, k);
                            if rel(g, gi, gj) == (0, 0) {
                                let (ka, kb) = clip.expect("区画に標本点があるのに範囲が空");
                                assert!(ka < k && k < kb || k == 0 && ka == 0 || k == wk.n_samples - 1 && kb == k,
                                        "標本点 {k} が範囲 {:?} の端 (余裕なし)", clip);
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn floor_i_matches_libm() {
        let vals = [0.0, -0.0, 0.5, -0.5, 1.0, -1.0, 2.999999999, -2.999999999, 1e15 + 0.5, -1e15 - 0.5,
                    4503599627370497.0, -4503599627370497.0, 1e19, -1e19, f64::NAN, f64::INFINITY, f64::NEG_INFINITY,
                    -9.223372036854775808e18, 9.223372036854775807e18, f64::MIN_POSITIVE, -f64::MIN_POSITIVE];
        for &v in &vals {
            assert_eq!(fm::floor_i(v), libm::floor(v) as i64, "{v}");
        }
    }

    #[test]
    fn sort_by_acq_then_sensor_zone() {
        let mk = |a: u32, s: u32, z: u32| Ray { acq_step: a, id: (s << 16) | (z << 8), ..NO_RAY };
        let mut v = [mk(2, 0, 0), mk(1, 1, 3), mk(1, 0, 7), mk(1, 1, 2)];
        sort_rays(&mut v);
        let keys: Vec<_> = v.iter().map(ray_key).collect();
        assert_eq!(keys, vec![(1, 7), (1, (1 << 8) | 2), (1, (1 << 8) | 3), (2, 0)]);
    }
}


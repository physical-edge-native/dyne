//! wave2d-actor のネイティブでの検証 (2026-11-11、docs/2026-11-11-wave2d.md)
//!
//! WASMにしたものと同じコード (同じ差込口の関数) をネイティブで直接呼び、1ノード (区画の
//! 分割なし) で次を調べる。分割解とのビット一致は dyne-sim2d で別に確かめる。
//!   conv   … 収束次数: 値固定0の正方形の定在波 (1,1) を ν を固定して h を半分ずつにし、
//!            t = 1 での解析解 cos(√2·π·c·t)·sin(πx)·sin(πy) との最大誤差を出す
//!   energy … 離散エネルギー (保存量) の時間変化 (値固定・ゼロ勾配・吸収境界)
//!   growth … クーラン数 ν ごとの最大振幅の増え方 (1ステップあたりの増幅率の実測)
//!   crc    … dyne-sim2d の単一ノード解との照合用に、全格子点の CRC32 を出す
//!
//! 実行: cargo run --release --example wave_study -- conv|energy|growth|crc [引数]
//! 出力は標準出力 (空白区切りの表)。

use dyne_wire::abi::field2d::*;
use dyne_wire::abi::wave2d::*;
use wave2d_actor as w;

struct Run {
    n: usize,
    h: f64,
    c: f64,
    dt: f64,
}

/// 全域1ノードで初期化する。outer: 外周の種類、absorb: 吸収境界の面のビット列
fn setup(n: usize, nu: f64, outer: u32, cfg: &[(u32, f64)]) -> Run {
    let h = 1.0 / ((n - 1) as f64);
    let c = 1.0;
    let dt = nu * h / c;
    for &(k, v) in cfg {
        w::dyne_cfg(k, v);
    }
    for f in 0..4 {
        w::dyne_set_face(f, outer, 0f64.to_bits());
    }
    assert_eq!(w::dyne_init2d(n as u32, n as u32, 0, 0, h, c, dt), 0);
    Run { n, h, c, dt }
}

fn reset_cfg() {
    // 既定値に戻すため、既定値を明示的に入れ直す
    for (k, v) in [
        (CFG_IC, IC_SIN_MODE as f64),
        (CFG_MODE_M, 1.0),
        (CFG_MODE_N, 1.0),
        (CFG_AMP, 1.0),
        (CFG_GX, 0.5),
        (CFG_GY, 0.5),
        (CFG_SIGMA, 0.05),
        (CFG_ABSORB, 0.0),
    ] {
        w::dyne_cfg(k, v);
    }
}

fn values(n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut u = vec![0.0; n * n];
    let mut p = vec![0.0; n * n];
    for k in 0..n * n {
        u[k] = w::dyne_get(k as u32);
        p[k] = w::get_prev(k as u32);
    }
    (u, p)
}

/// 離散エネルギー E^{n+1/2} = ½Σ w_p ((u^{n+1}−u^n)/dt)² + ½c²Σ_辺 w_e (Δu^{n+1})(Δu^n)/h²、×h²。
/// 重み w は台形則 (外周の点 1/2、角 1/4、外周に沿う辺 1/2)。ゼロ勾配 (鏡映) のとき
/// 更新行列がこの重みで対称になるので、リープフロッグで丸めを除いて厳密に保存される。
/// 値固定0では外周の点・外周に沿う辺の寄与が0なので、重みによらず同じ量になる。
fn energy(r: &Run, un: &[f64], uo: &[f64]) -> f64 {
    let n = r.n;
    let wt = |i: usize| if i == 0 || i == n - 1 { 0.5 } else { 1.0 };
    let mut kin = 0.0;
    let mut pot = 0.0;
    for j in 0..n {
        for i in 0..n {
            let k = j * n + i;
            let v = (un[k] - uo[k]) / r.dt;
            kin += wt(i) * wt(j) * v * v;
            if i + 1 < n {
                let a = un[k + 1] - un[k];
                let b = uo[k + 1] - uo[k];
                pot += wt(j) * a * b;
            }
            if j + 1 < n {
                let a = un[k + n] - un[k];
                let b = uo[k + n] - uo[k];
                pot += wt(i) * a * b;
            }
        }
    }
    0.5 * r.h * r.h * (kin + r.c * r.c * pot / (r.h * r.h))
}

fn crc32(d: &[u8]) -> u32 {
    let mut c: u32 = 0xFFFF_FFFF;
    for &b in d {
        c ^= b as u32;
        for _ in 0..8 {
            let m = (c & 1).wrapping_neg();
            c = (c >> 1) ^ (0xEDB8_8320 & m);
        }
    }
    !c
}

fn conv() {
    let pi = std::f64::consts::PI;
    println!("# conv: Dirichlet 0, IC sin(pi x) sin(pi y), zero initial velocity, nu = 0.5, t_end = 1");
    println!("# n h dt steps Linf_err ratio_to_previous");
    let mut prev: Option<f64> = None;
    for &n in &[21usize, 41, 81, 161] {
        reset_cfg();
        let r = setup(n, 0.5, BC_DIRICHLET, &[]);
        let steps = 2 * (n - 1); // t_end = steps·dt = 2(n−1)·0.5/(n−1) = 1
        for _ in 0..steps {
            w::dyne_step();
        }
        let t = steps as f64 * r.dt;
        let om = 2f64.sqrt() * pi * r.c;
        let mut e = 0.0f64;
        for j in 0..n {
            for i in 0..n {
                let (x, y) = (i as f64 * r.h, j as f64 * r.h);
                let ex = libm::cos(om * t) * libm::sin(pi * x) * libm::sin(pi * y);
                e = e.max((w::dyne_get((j * n + i) as u32) - ex).abs());
            }
        }
        let ratio = prev.map(|p| p / e).unwrap_or(f64::NAN);
        println!("{} {:.6} {:.6} {} {:.6e} {:.3}", n, r.h, r.dt, steps, e, ratio);
        prev = Some(e);
    }
}

fn energy_case(name: &str, outer: u32, cfg: &[(u32, f64)], steps: usize) {
    reset_cfg();
    let r = setup(41, 0.5, outer, cfg);
    let mut e0 = f64::NAN;
    let mut emin = f64::INFINITY;
    let mut emax = f64::NEG_INFINITY;
    let mut samples = Vec::new();
    for s in 1..=steps {
        w::dyne_step();
        let (un, uo) = values(r.n);
        let e = energy(&r, &un, &uo);
        if s == 1 {
            e0 = e;
        }
        emin = emin.min(e);
        emax = emax.max(e);
        if s == 1 || s % (steps / 8) == 0 {
            samples.push((s, e));
        }
    }
    let drift = (emax - emin) / e0;
    print!("{} E(1/2)={:.15e} max_rel_spread={:.3e} |", name, e0, drift);
    for (s, e) in samples {
        print!(" {}:{:.6e}", s, e / e0);
    }
    println!();
}

fn energy_all() {
    println!("# energy: n = 41, nu = 0.5, 2000 steps (t = 25). E normalised by E at the first half step");
    energy_case("dirichlet_sin11", BC_DIRICHLET, &[], 2000);
    energy_case(
        "dirichlet_gauss",
        BC_DIRICHLET,
        &[(CFG_IC, IC_GAUSS as f64), (CFG_GX, 0.3), (CFG_GY, 0.6), (CFG_SIGMA, 0.06)],
        2000,
    );
    energy_case("zerograd_cos11", BC_ZERO_GRAD, &[(CFG_IC, IC_COS_MODE as f64)], 2000);
    energy_case(
        "zerograd_gauss",
        BC_ZERO_GRAD,
        &[(CFG_IC, IC_GAUSS as f64), (CFG_GX, 0.3), (CFG_GY, 0.6), (CFG_SIGMA, 0.06)],
        2000,
    );
    energy_case(
        "absorb_gauss",
        BC_ZERO_GRAD,
        &[(CFG_IC, IC_GAUSS as f64), (CFG_SIGMA, 0.06), (CFG_ABSORB, 15.0)],
        400,
    );
}

fn growth(args: &[String]) {
    // growth [outer] [n] [steps] : 外周 dirichlet|zero_grad|absorb
    let outer_s = args.first().map(|s| s.as_str()).unwrap_or("dirichlet");
    let n: usize = args.get(1).map(|s| s.parse().unwrap()).unwrap_or(41);
    let steps: usize = args.get(2).map(|s| s.parse().unwrap()).unwrap_or(2000);
    let (outer, absorb) = match outer_s {
        "zero_grad" => (BC_ZERO_GRAD, 0.0),
        "absorb" => (BC_ZERO_GRAD, 15.0),
        _ => (BC_DIRICHLET, 0.0),
    };
    println!("# growth: outer {} n {} steps {}, IC gauss (0.3,0.6) sigma 0.06", outer_s, n, steps);
    println!("# nu max|u|_final measured_rate(last 200 steps, NaN if not growing)");
    let nus = [
        0.60, 0.65, 0.70, 0.705, 0.7071, 0.7072, 0.7074, 0.7076, 0.7077, 0.7078, 0.708, 0.709, 0.71, 0.72, 0.75,
    ];
    for &nu in &nus {
        reset_cfg();
        setup(
            n,
            nu,
            outer,
            &[(CFG_IC, IC_GAUSS as f64), (CFG_GX, 0.3), (CFG_GY, 0.6), (CFG_SIGMA, 0.06), (CFG_ABSORB, absorb)],
        );
        let mut amax = vec![0.0f64; steps + 1];
        for s in 1..=steps {
            w::dyne_step();
            let mut m = 0.0f64;
            for k in 0..n * n {
                let v = w::dyne_get(k as u32).abs();
                // NaN (あふれの後) は無限大として数える (f64::max は NaN を無視するため)
                m = if v.is_nan() { f64::INFINITY } else { m.max(v) };
            }
            amax[s] = m;
        }
        let (a1, a2) = (amax[steps - 200], amax[steps]);
        let rate = if a2.is_finite() && a2 > 1e3 { (a2 / a1).powf(1.0 / 200.0) } else { f64::NAN };
        println!("{} {:e} {:.6}", nu, amax[steps], rate);
    }
}

fn crc(args: &[String]) {
    // crc n dt steps [k=v ...] : 値固定0の外周、全域1ノード、c = 1 (dyne-sim2d の --dt と同じ値を渡す)
    let n: usize = args[0].parse().unwrap();
    let dt: f64 = args[1].parse().unwrap();
    let steps: usize = args[2].parse().unwrap();
    let cfg: Vec<(u32, f64)> = args[3..]
        .iter()
        .map(|a| {
            let (k, v) = a.split_once('=').unwrap();
            (k.parse().unwrap(), v.parse().unwrap())
        })
        .collect();
    reset_cfg();
    for &(k, v) in &cfg {
        w::dyne_cfg(k, v);
    }
    for f in 0..4 {
        w::dyne_set_face(f, BC_DIRICHLET, 0f64.to_bits());
    }
    let h = 1.0 / ((n - 1) as f64);
    assert_eq!(w::dyne_init2d(n as u32, n as u32, 0, 0, h, 1.0, dt), 0);
    for _ in 0..steps {
        w::dyne_step();
    }
    let mut b = Vec::new();
    for k in 0..n * n {
        b.extend_from_slice(&w::dyne_get(k as u32).to_bits().to_le_bytes());
    }
    println!("native single crc32 {:08x} (n {} dt {:e} steps {})", crc32(&b), n, dt, steps);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(|s| s.as_str()) {
        Some("conv") => conv(),
        Some("energy") => energy_all(),
        Some("growth") => growth(&args[1..]),
        Some("crc") => crc(&args[1..]),
        _ => {
            conv();
            energy_all();
        }
    }
}

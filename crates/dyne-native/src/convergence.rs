//! Convergence-order check of the wave solvers against analytic standing-wave solutions.
//!
//! Bit-exactness shows that splitting does not change the result; this module shows that the result
//! converges to the solution of the equation at the expected rate. The leapfrog scheme with the
//! five-point (2D) or seven-point (3D) Laplacian is second order in space and time, so at a fixed
//! Courant number the error at a fixed time should fall as `h²`.
//!
//! # Problems
//!
//! On the unit square (cube) with fixed value 0 on every face, `c = 1` and zero initial velocity:
//!
//! - 2D: `u = sin(πx)·sin(πy)·cos(ωt)`, `ω = π√2`;
//! - 3D: `u = sin(πx)·sin(πy)·sin(πz)·cos(ωt)`, `ω = π√3`.
//!
//! The grid has `N` points per axis (`h = 1/(N−1)`), the time step is `Δt = ν·h` for the Courant
//! number `ν`, the run takes `round(T/Δt)` steps, and the analytic solution is evaluated at the time
//! actually reached (`steps·Δt`). Analytic values use the pure-Rust `libm`, so that every OS computes
//! the same bits.
//!
//! # Error norms and observed order
//!
//! Over every grid point (boundary included), with `e` = numerical − analytic: the maximum norm
//! `max |e|` and the discrete L2 norm `sqrt(h^d · Σ e²)` (`d` the dimension; sums in point order).
//! The observed order between two grids is `ln(e_coarse / e_fine) / ln(h_coarse / h_fine)`; the fitted
//! order is the least-squares slope of `ln e` against `ln h`.

use crate::decomp::Decomposition;
use crate::patch::{run_single, PatchError};
use crate::{wave2d, wave3d};

const PI: f64 = core::f64::consts::PI;

/// The errors of one grid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Level {
    /// Number of grid points per axis.
    pub points: usize,
    /// Grid spacing `1/(points − 1)`.
    pub h: f64,
    /// Number of time steps.
    pub steps: u64,
    /// Time reached (`steps · Δt`), at which the analytic solution is evaluated.
    pub t: f64,
    /// Maximum norm of the error.
    pub max_error: f64,
    /// Discrete L2 norm of the error, `sqrt(h^d · Σ e²)`.
    pub l2_error: f64,
}

/// Which error norm to use for an observed order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Norm {
    /// [`Level::max_error`].
    Max,
    /// [`Level::l2_error`].
    L2,
}

impl Level {
    /// The error in norm `n`.
    pub fn error(&self, n: Norm) -> f64 {
        match n {
            Norm::Max => self.max_error,
            Norm::L2 => self.l2_error,
        }
    }
}

fn spacing_and_steps(points: usize, courant: f64, t_end: f64) -> Result<(f64, f64, u64), PatchError> {
    if points < 3 || !(courant > 0.0) || !(t_end > 0.0) {
        return Err(PatchError("need at least 3 points, a positive Courant number and a positive end time".into()));
    }
    let h = 1.0 / ((points - 1) as f64);
    let dt = courant * h;
    let steps = libm::round(t_end / dt) as u64;
    Ok((h, dt, steps))
}

fn norms(errors: impl Iterator<Item = f64>, cell: f64) -> (f64, f64) {
    let (mut max, mut sum) = (0.0f64, 0.0f64);
    for e in errors {
        max = max.max(e.abs());
        sum += e * e;
    }
    (max, libm::sqrt(cell * sum))
}

/// Errors of the 2D solver for the standing wave `sin(πx)·sin(πy)·cos(π√2·t)` on an `N × N` grid,
/// solved as one patch at Courant number `courant` up to time `t_end` (rounded to whole steps).
pub fn standing_wave_2d(points: usize, courant: f64, t_end: f64) -> Result<Level, PatchError> {
    let (h, dt, steps) = spacing_and_steps(points, courant, t_end)?;
    let p = wave2d::Wave2dProblem {
        points: [points; 2],
        h,
        c: 1.0,
        dt,
        initial: wave2d::Initial::SinMode { m: 1.0, n: 1.0, amp: 1.0 },
        outer: [wave2d::Outer::Fixed(0.0); 4],
    };
    let d = Decomposition::whole(p.points, wave2d::MIN_POINTS).map_err(|e| PatchError(e.0))?;
    let field = run_single(&mut wave2d::Wave2dPatch::new(&p, &d.blocks[0])?, steps)?;
    let t = (steps as f64) * dt;
    let time = libm::cos(PI * libm::sqrt(2.0) * t);
    let exact = |k: usize| {
        let (i, j) = (k % points, k / points);
        let (x, y) = ((i as f64) * h, (j as f64) * h);
        libm::sin(PI * x) * libm::sin(PI * y) * time
    };
    let (max_error, l2_error) = norms(field.iter().enumerate().map(|(k, &u)| u - exact(k)), h * h);
    Ok(Level { points, h, steps, t, max_error, l2_error })
}

/// Errors of the 3D solver for the standing wave `sin(πx)·sin(πy)·sin(πz)·cos(π√3·t)` on an
/// `N × N × N` grid, solved as one patch at Courant number `courant` up to time `t_end`.
pub fn standing_wave_3d(points: usize, courant: f64, t_end: f64) -> Result<Level, PatchError> {
    let (h, dt, steps) = spacing_and_steps(points, courant, t_end)?;
    let p = wave3d::Wave3dProblem {
        points: [points; 3],
        h,
        c: 1.0,
        dt,
        initial: wave3d::Initial::SinMode { m: 1.0, n: 1.0, l: 1.0, amp: 1.0 },
        outer: [wave3d::Outer::Fixed(0.0); 6],
    };
    let d = Decomposition::whole(p.points, wave3d::MIN_POINTS).map_err(|e| PatchError(e.0))?;
    let field = run_single(&mut wave3d::Wave3dPatch::new(&p, &d.blocks[0])?, steps)?;
    let t = (steps as f64) * dt;
    let time = libm::cos(PI * libm::sqrt(3.0) * t);
    let exact = |k: usize| {
        let (i, j, l) = (k % points, (k / points) % points, k / (points * points));
        let (x, y, z) = ((i as f64) * h, (j as f64) * h, (l as f64) * h);
        libm::sin(PI * x) * libm::sin(PI * y) * libm::sin(PI * z) * time
    };
    let (max_error, l2_error) = norms(field.iter().enumerate().map(|(k, &u)| u - exact(k)), h * h * h);
    Ok(Level { points, h, steps, t, max_error, l2_error })
}

/// Observed order between two grids: `ln(e_coarse / e_fine) / ln(h_coarse / h_fine)`.
pub fn pair_order(coarse: &Level, fine: &Level, n: Norm) -> f64 {
    libm::log(coarse.error(n) / fine.error(n)) / libm::log(coarse.h / fine.h)
}

/// Least-squares slope of `ln e` against `ln h` over `levels` (at least two).
pub fn fitted_order(levels: &[Level], n: Norm) -> f64 {
    let k = levels.len() as f64;
    let xs: Vec<f64> = levels.iter().map(|l| libm::log(l.h)).collect();
    let ys: Vec<f64> = levels.iter().map(|l| libm::log(l.error(n))).collect();
    let (mx, my) = (xs.iter().sum::<f64>() / k, ys.iter().sum::<f64>() / k);
    let (mut sxy, mut sxx) = (0.0, 0.0);
    for (x, y) in xs.iter().zip(&ys) {
        sxy += (x - mx) * (y - my);
        sxx += (x - mx) * (x - mx);
    }
    sxy / sxx
}

/// The accepted band of observed orders: `[2 − 0.1, 2 + 0.1]`.
///
/// Reason (stated before the check was run): for these standing waves the only error is the phase
/// error of the discrete frequency, `ω_h = ω (1 + δ)` with `δ = h² (ν² K⁴ − Σ k⁴) / (24 K²) + O(h⁴)`;
/// the higher-order terms move the observed order by an estimated 0.01 or less for `h ≤ 1/16`, and
/// rounding errors are many orders of magnitude below the smallest error measured. A band ten times
/// wider than that estimate still rejects a first-order error (order near 1).
pub const ORDER_BAND: (f64, f64) = (1.9, 2.1);

/// `true` if `order` lies in [`ORDER_BAND`].
pub fn order_in_band(order: f64) -> bool {
    order >= ORDER_BAND.0 && order <= ORDER_BAND.1
}

/// A convergence study: the levels and the observed orders, with the verdict over the levels used
/// for the check (`levels[checked_from..]`).
#[derive(Clone, Debug)]
pub struct Study {
    /// Grids from coarse to fine.
    pub levels: Vec<Level>,
    /// Index of the first level used for the check (coarser levels are reported only).
    pub checked_from: usize,
}

impl Study {
    /// Runs `solve(points, courant, t_end)` for every grid size in `points` (coarse to fine).
    pub fn run(
        solve: fn(usize, f64, f64) -> Result<Level, PatchError>,
        points: &[usize],
        checked_from: usize,
        courant: f64,
        t_end: f64,
    ) -> Result<Study, PatchError> {
        let levels = points.iter().map(|&n| solve(n, courant, t_end)).collect::<Result<Vec<_>, _>>()?;
        if levels.len() < checked_from + 2 {
            return Err(PatchError("need at least two checked levels".into()));
        }
        Ok(Study { levels, checked_from })
    }

    /// Every observed order used for the check: each pair of neighbouring checked levels and the
    /// least-squares fit over the checked levels, for both norms. Each entry: (what, order).
    pub fn checked_orders(&self) -> Vec<(String, f64)> {
        let c = &self.levels[self.checked_from..];
        let mut out = Vec::new();
        for n in [Norm::Max, Norm::L2] {
            for w in c.windows(2) {
                out.push((format!("{n:?} {}->{}", w[0].points, w[1].points), pair_order(&w[0], &w[1], n)));
            }
            out.push((format!("{n:?} fit {}..{}", c[0].points, c[c.len() - 1].points), fitted_order(c, n)));
        }
        out
    }

    /// `true` if every checked order lies in [`ORDER_BAND`].
    pub fn passes(&self) -> bool {
        self.checked_orders().iter().all(|(_, p)| order_in_band(*p))
    }

    /// The error table as Markdown (orders to the next coarser grid; `fit` rows for the checked levels).
    pub fn table(&self) -> String {
        let mut s = String::from(
            "| N | h | steps | t | max error | order (max) | L2 error | order (L2) | max error / h² |\n\
             |---:|---:|---:|---:|---:|---:|---:|---:|---:|\n",
        );
        for (i, l) in self.levels.iter().enumerate() {
            let (om, o2) = if i == 0 {
                ("-".to_string(), "-".to_string())
            } else {
                let p = &self.levels[i - 1];
                (format!("{:.4}", pair_order(p, l, Norm::Max)), format!("{:.4}", pair_order(p, l, Norm::L2)))
            };
            s += &format!(
                "| {} | 1/{} | {} | {} | {:.4e} | {om} | {:.4e} | {o2} | {:.4} |\n",
                l.points,
                l.points - 1,
                l.steps,
                l.t,
                l.max_error,
                l.l2_error,
                l.max_error / (l.h * l.h)
            );
        }
        let c = &self.levels[self.checked_from..];
        s += &format!(
            "| fit {}..{} | | | | | {:.4} | | {:.4} | |\n",
            c[0].points,
            c[c.len() - 1].points,
            fitted_order(c, Norm::Max),
            fitted_order(c, Norm::L2)
        );
        s
    }
}

/// Prints the description of `s`, its error table, every checked order and the verdict
/// (`PASS (order within [1.9, 2.1])` or `FAIL ...`); returns [`Study::passes`].
pub fn print_report(s: &Study) -> bool {
    let (lo, hi) = ORDER_BAND;
    println!(
        "standing wave, fixed value 0, Courant number 0.5, t = 0.5, checked from N = {}",
        s.levels[s.checked_from].points
    );
    println!();
    print!("{}", s.table());
    println!();
    for (what, p) in s.checked_orders() {
        println!("order {what}: {p:.4}{}", if order_in_band(p) { "" } else { "  (outside the band)" });
    }
    let ok = s.passes();
    if ok {
        println!("PASS (order within [{lo}, {hi}])");
    } else {
        println!("FAIL: an observed order is outside [{lo}, {hi}]");
    }
    ok
}

/// The 2D study of the check: N = 17, 33, 65, 129, 257, all checked, Courant number 0.5, T = 0.5.
pub fn study_2d() -> Result<Study, PatchError> {
    Study::run(standing_wave_2d, &[17, 33, 65, 129, 257], 0, 0.5, 0.5)
}

/// The 3D study of the check: N = 9, 17, 33, 65, checked from N = 17 (N = 9 is reported only, as
/// outside the asymptotic range), Courant number 0.5, T = 0.5.
pub fn study_3d() -> Result<Study, PatchError> {
    Study::run(standing_wave_3d, &[9, 17, 33, 65], 1, 0.5, 0.5)
}

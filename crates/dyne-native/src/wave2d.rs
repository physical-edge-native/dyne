//! The 2D wave equation on one patch: a native Rust solver with the same floating-point operations,
//! in the same order, as Dyne's WebAssembly wave solver (`wave2d-actor` in the Dyne repository,
//! described in Dyne's `docs/2026-11-11-wave2d.md`).
//!
//! Because every operation is the same IEEE 754 operation in the same order (Rust does not fuse
//! multiplications and additions unless asked to), this solver gives the same bits as Dyne's solver
//! running in WebAssembly, and a split run gives the same bits as the single-patch run.
//!
//! # Method
//!
//! `u_tt = c² (u_xx + u_yy)` on a square grid (spacing `h` on both axes), five-point Laplacian and the
//! three-level central difference in time (the leapfrog scheme):
//!
//! ```text
//! u^{n+1} = 2 u^n − u^{n−1} + ν² ((u_W − 2u + u_E) + (u_S − 2u + u_N)),   ν = c Δt / h
//! ```
//!
//! The first step starts from zero initial velocity: `u^1 = u^0 + (ν²/2) (same Laplacian)`.
//! The scheme is stable for `ν ≤ 1/√2` on an unbounded grid (von Neumann analysis); the solver does
//! not check this, so that sweeps across the limit stay possible ([`Wave2dProblem::courant`]).
//!
//! Only the current level `u^n` of a neighbour is used, so a patch needs one layer of ghost points and
//! exchanges one row of values per face and step ([`Wave2dPatch::new`], as Dyne's solver does).
//!
//! # Wider borders
//!
//! [`Wave2dPatch::with_halo`] keeps `k` layers of the neighbours' values and exchanges them once every
//! `k` steps (with the face and the diagonal neighbours), advancing the overlap redundantly; see
//! [`crate::halo`]. Every point still performs the same operations, so the bits do not change. Dyne's
//! WebAssembly solver supports only `k = 1`, so all patches of a run with `k > 1` must be native.
//!
//! # Outer boundaries (per face)
//!
//! - [`Outer::Fixed`]: the edge points are reset to a value after every step (Dirichlet condition).
//! - [`Outer::ZeroGradient`]: the ghost point mirrors the first interior point (Neumann condition).
//! - [`Outer::Absorbing`]: first-order Engquist–Majda condition discretised with Mur's method:
//!   `u_0^{n+1} = u_1^n + k (u_1^{n+1} − u_0^n)`, `k = (ν−1)/(ν+1)`. The face is first updated as a
//!   zero-gradient face, then its edge points are overwritten.
//!
//! Absorbing faces are applied in face order west, east, south, north, then fixed faces in the same
//! order (at a corner, the later face wins), exactly as in Dyne's solver.
//!
//! # 3D
//!
//! The patch interface ([`PatchSolver`]), the decomposition and the border messages do not depend on the
//! dimension. The 3D solver ([`crate::wave3d`]) is a type next to this one (seven-point Laplacian,
//! stability limit `ν ≤ 1/√3`, six faces whose values are sent as flat lists) and uses the same
//! operations as this solver for the x and y terms.

use crate::decomp::Block;
use crate::halo::Window;
use crate::patch::{PatchError, PatchSolver};
use dyne_wire::abi::field2d::{BC_DIRICHLET, BC_INTERFACE, BC_ZERO_GRAD};
use dyne_wire::abi::wave2d as keys;

/// Smallest number of points per axis of a patch (as in Dyne's solver).
pub const MIN_POINTS: usize = 3;

const PI: f64 = core::f64::consts::PI;

/// Initial displacement (the initial velocity is always zero). `x` and `y` are `i·h` and `j·h` for
/// the global point indices `i`, `j`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(missing_docs)] // the fields are the parameters in each variant's formula
pub enum Initial {
    /// `amp · sin(mπx) · sin(nπy)`: a standing wave on the unit square with fixed value 0.
    SinMode { m: f64, n: f64, amp: f64 },
    /// `amp · exp(−((x−x0)² + (y−y0)²) / (2σ²))`: a Gaussian bump.
    Gauss { x0: f64, y0: f64, sigma: f64, amp: f64 },
    /// `amp · cos(mπx) · cos(nπy)`: a standing wave on the unit square with zero gradient.
    CosMode { m: f64, n: f64, amp: f64 },
}

/// Condition on one face of the outer boundary.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Outer {
    /// Fixed value (Dirichlet condition).
    Fixed(f64),
    /// Zero gradient (Neumann condition, free edge).
    ZeroGradient,
    /// First-order absorbing boundary (Engquist–Majda condition, Mur's discretisation).
    Absorbing,
}

/// A 2D wave problem on a grid of `points[0] × points[1]` points with spacing `h`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wave2dProblem {
    /// Number of grid points along x and y.
    pub points: [usize; 2],
    /// Grid spacing (the same on both axes).
    pub h: f64,
    /// Wave speed.
    pub c: f64,
    /// Time step.
    pub dt: f64,
    /// Initial displacement.
    pub initial: Initial,
    /// Outer boundary per face: west, east, south, north (face `2·axis + side`).
    pub outer: [Outer; 4],
}

impl Wave2dProblem {
    /// Courant number `ν = c Δt / h` (stable for `ν ≤ 1/√2` on an unbounded grid).
    pub fn courant(&self) -> f64 {
        self.c * self.dt / self.h
    }

    /// The configuration calls (`dyne_cfg(key, value)`) that set up Dyne's WebAssembly solver for this
    /// problem (keys from `dyne_wire::abi::wave2d`).
    pub fn actor_cfg(&self) -> Vec<(u32, f64)> {
        let mut cfg = match self.initial {
            Initial::SinMode { m, n, amp } => vec![
                (keys::CFG_IC, keys::IC_SIN_MODE as f64),
                (keys::CFG_MODE_M, m),
                (keys::CFG_MODE_N, n),
                (keys::CFG_AMP, amp),
            ],
            Initial::Gauss { x0, y0, sigma, amp } => vec![
                (keys::CFG_IC, keys::IC_GAUSS as f64),
                (keys::CFG_GX, x0),
                (keys::CFG_GY, y0),
                (keys::CFG_SIGMA, sigma),
                (keys::CFG_AMP, amp),
            ],
            Initial::CosMode { m, n, amp } => vec![
                (keys::CFG_IC, keys::IC_COS_MODE as f64),
                (keys::CFG_MODE_M, m),
                (keys::CFG_MODE_N, n),
                (keys::CFG_AMP, amp),
            ],
        };
        let absorb: u32 = (0..4).filter(|&f| self.outer[f] == Outer::Absorbing).map(|f| 1 << f).sum();
        cfg.push((keys::CFG_ABSORB, absorb as f64));
        cfg
    }

    /// The face settings (`dyne_set_face(face, kind, param)`) of Dyne's WebAssembly solver for one
    /// block: an interface carries its id, a fixed face its value as `f64` bits.
    pub fn actor_faces(&self, b: &Block<2>) -> [(u32, u64); 4] {
        let mut out = [(0, 0); 4];
        for (f, o) in out.iter_mut().enumerate() {
            *o = match (b.faces[f], self.outer[f]) {
                (Some(i), _) => (BC_INTERFACE, i.id as u64),
                (None, Outer::Fixed(v)) => (BC_DIRICHLET, v.to_bits()),
                // An absorbing face is a zero-gradient face with its bit set in CFG_ABSORB.
                (None, Outer::ZeroGradient | Outer::Absorbing) => (BC_ZERO_GRAD, 0),
            };
        }
        out
    }

    /// Initial displacement at global point `(gi, gj)` (the same expression, in the same order, as
    /// Dyne's solver).
    fn initial_at(&self, gi: usize, gj: usize) -> f64 {
        let x = (gi as f64) * self.h;
        let y = (gj as f64) * self.h;
        match self.initial {
            Initial::Gauss { x0, y0, sigma, amp } => {
                let (dx, dy) = (x - x0, y - y0);
                amp * libm::exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma))
            }
            Initial::CosMode { m, n, amp } => amp * libm::cos(m * PI * x) * libm::cos(n * PI * y),
            Initial::SinMode { m, n, amp } => amp * libm::sin(m * PI * x) * libm::sin(n * PI * y),
        }
    }
}

/// The native 2D wave solver for one patch.
pub struct Wave2dPatch {
    nu2: f64,
    half_nu2: f64,
    kmur: f64,
    /// Layout, faces, neighbours and exchange schedule.
    win: Window<2>,
    /// Current level `u^n` with the overlap layers and one ring of ghost points.
    u: Vec<f64>,
    /// Previous level `u^{n−1}` (overwritten point by point with `u^{n+1}` during a step).
    p: Vec<f64>,
}

impl Wave2dPatch {
    /// Sets up the solver of block `b` of `problem` at step 0, exchanging one layer every step.
    pub fn new(problem: &Wave2dProblem, b: &Block<2>) -> Result<Self, PatchError> {
        Self::with_halo(problem, b, 1)
    }

    /// Sets up the solver of block `b` of `problem` at step 0, exchanging `halo` layers once every
    /// `halo` steps. Fails if the block has fewer than `halo` points along an axis on which it has a
    /// neighbour.
    pub fn with_halo(problem: &Wave2dProblem, b: &Block<2>, halo: usize) -> Result<Self, PatchError> {
        let [nx, ny] = b.size;
        if nx < MIN_POINTS || ny < MIN_POINTS {
            return Err(PatchError(format!("patch of {nx}×{ny} points; at least {MIN_POINTS} per axis needed")));
        }
        if !(problem.h > 0.0) || !(problem.c > 0.0) || !(problem.dt > 0.0) {
            return Err(PatchError("h, c and dt must be positive".into()));
        }
        if let Initial::Gauss { sigma, .. } = problem.initial {
            if !(sigma > 0.0) {
                return Err(PatchError("sigma must be positive".into()));
            }
        }
        let nu = problem.c * problem.dt / problem.h;
        let nu2 = nu * nu;
        let win = Window::new(b, &problem.outer, halo)?;
        let total = win.total();
        let mut s = Wave2dPatch {
            nu2,
            half_nu2: 0.5 * nu2,
            kmur: (nu - 1.0) / (nu + 1.0),
            win,
            u: vec![0.0; total],
            p: vec![0.0; total],
        };
        let [ix0, iy0] = b.origin;
        for j in 0..ny {
            for i in 0..nx {
                let k = s.win.at([i as isize, j as isize]);
                s.u[k] = problem.initial_at(ix0 + i, iy0 + j);
            }
        }
        let own = s.win.own();
        s.win.apply_fixed(&mut s.u, &own);
        Ok(s)
    }

    /// Number of steps done.
    pub fn steps_done(&self) -> u64 {
        self.win.step
    }

    /// Number of layers exchanged (one exchange every that many steps).
    pub fn halo(&self) -> usize {
        self.win.k
    }

    /// Update of the points of `region` (rows are independent; each point uses the same operations
    /// in the same order as Dyne's solver).
    fn update(&mut self, region: &crate::halo::Range<2>) {
        let [(x0, x1), (y0, y1)] = *region;
        let nx = (x1 - x0) as usize;
        let w = self.win.stride(1);
        let (nu2, half_nu2) = (self.nu2, self.half_nu2);
        let first = self.win.step == 0;
        for j in y0..y1 {
            let r = self.win.at([x0, j]);
            let south = &self.u[r - w..r - w + nx];
            let north = &self.u[r + w..r + w + nx];
            let west = &self.u[r - 1..r - 1 + nx];
            let mid = &self.u[r..r + nx];
            let east = &self.u[r + 1..r + 1 + nx];
            let out = &mut self.p[r..r + nx];
            if first {
                for i in 0..nx {
                    let u = mid[i];
                    let lap = (west[i] - 2.0 * u + east[i]) + (south[i] - 2.0 * u + north[i]);
                    out[i] = u + half_nu2 * lap;
                }
            } else {
                for i in 0..nx {
                    let u = mid[i];
                    let lap = (west[i] - 2.0 * u + east[i]) + (south[i] - 2.0 * u + north[i]);
                    out[i] = 2.0 * u - out[i] + nu2 * lap;
                }
            }
        }
    }
}

impl PatchSolver for Wave2dPatch {
    fn outbox(&mut self, out: &mut Vec<u8>) -> Result<(), PatchError> {
        self.win.outbox(&self.u, &self.p, out)
    }

    fn inbox(&mut self, framed: &[u8]) -> Result<(), PatchError> {
        self.win.inbox(&mut self.u, &mut self.p, framed)
    }

    fn step(&mut self) -> Result<(), PatchError> {
        self.win.check_ready()?;
        let region = self.win.region();
        self.win.fill_outer_ghosts(&mut self.u, &region);
        self.update(&region);
        self.win.apply_absorbing(&self.u, &mut self.p, self.kmur, &region);
        std::mem::swap(&mut self.u, &mut self.p);
        self.win.apply_fixed(&mut self.u, &region);
        self.win.advance();
        Ok(())
    }

    fn field(&mut self) -> Result<Vec<f64>, PatchError> {
        Ok(self.win.field(&self.u))
    }

    fn halo_width(&self) -> usize {
        self.win.k
    }
}

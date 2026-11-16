//! The 3D wave equation on one patch: a native Rust solver, the 3D extension of [`crate::wave2d`].
//!
//! Dyne has no 3D WebAssembly solver yet. The order of every floating-point operation is fixed and
//! written down below, so that a future 3D WebAssembly solver can be built to give the same bits and
//! be compared with this one, as the 2D solvers are. A split run gives the same bits as the
//! single-patch run because every point performs the same operations in the same order whatever
//! patch it belongs to (Rust does not fuse multiplications and additions unless asked to).
//!
//! # Method
//!
//! `u_tt = c² (u_xx + u_yy + u_zz)` on a cubic grid (spacing `h` on all axes), seven-point Laplacian
//! and the three-level central difference in time (the leapfrog scheme), `ν = c Δt / h`. The scheme is
//! stable for `ν ≤ 1/√3` on an unbounded grid (von Neumann analysis); the solver does not check this,
//! so that sweeps across the limit stay possible ([`Wave3dProblem::courant`]).
//!
//! # Fixed operation order (the contract for bit-exact comparison)
//!
//! With `u` the current level, `W, E` its neighbours along x (low, high), `S, N` along y and
//! `D, U` along z, every point computes, in this order and with these parentheses:
//!
//! ```text
//! lap = ((W − 2.0·u + E) + (S − 2.0·u + N)) + (D − 2.0·u + U)
//! first step (n = 0):  u^1     = u + half_nu2 · lap
//! later steps:         u^{n+1} = 2.0·u − u^{n−1} + nu2 · lap
//! ```
//!
//! where each bracket `(A − 2.0·u + B)` is `(A − (2.0·u)) + B`, and the coefficients are computed once
//! at set-up: `nu = c · dt / h`, `nu2 = nu · nu`, `half_nu2 = 0.5 · nu2`, `kmur = (nu − 1.0) / (nu + 1.0)`.
//! The x and y terms are exactly those of the 2D solver, so a 3D problem that does not vary along z
//! (with zero-gradient z faces) reproduces the 2D solver bit for bit in every z slice (tested).
//!
//! The first step starts from zero initial velocity. Initial values are computed with the pure-Rust
//! `libm` (identical bits on every platform), from `x = i·h`, `y = j·h`, `z = k·h` with the global
//! point indices `i, j, k` (see [`Initial`] for each expression and its order).
//!
//! # Outer boundaries (per face)
//!
//! - [`Outer::Fixed`]: the face points are reset to a value after every step (Dirichlet condition);
//!   the ghost points of the face hold the same value.
//! - [`Outer::ZeroGradient`]: the ghost point mirrors the first interior point (Neumann condition).
//! - [`Outer::Absorbing`]: first-order Engquist–Majda condition discretised with Mur's method:
//!   `u_0^{n+1} = u_1^n + kmur · (u_1^{n+1} − u_0^n)`. The face is first updated as a zero-gradient
//!   face, then its points are overwritten.
//!
//! One step is: fill the ghost points of the outer faces (faces 0 to 5 in order), update every point
//! (the formula above), apply the absorbing faces in face order 0 to 5, make the new level current,
//! then reset the fixed faces in face order 0 to 5. Where faces meet (edges and corners of the box),
//! the later face wins. This is the 2D order with the two z faces appended.
//!
//! # Faces and border messages
//!
//! Faces are numbered `2·axis + side` (x low 0, x high 1, y low 2, y high 3, z low 4, z high 5). Only
//! the six faces are exchanged (the seven-point stencil uses no edge or corner neighbour): each step,
//! a patch sends the 2D slab of values on each face that touches a neighbour as one IFACE_BAND
//! message (one `f64` per point). The slab is flattened in the block's point order: the lower of the
//! two in-face axes runs fastest (face x: y fastest, then z; face y: x, then z; face z: x, then y).

use crate::decomp::Block;
use crate::halo::Window;
use crate::patch::{PatchError, PatchSolver};

pub use crate::wave2d::Outer;

/// Smallest number of points per axis of a patch.
pub const MIN_POINTS: usize = 3;

const PI: f64 = core::f64::consts::PI;

/// Initial displacement (the initial velocity is always zero). `x`, `y`, `z` are `i·h`, `j·h`, `k·h`
/// for the global point indices; products are evaluated left to right as written.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Initial {
    /// `amp · sin(mπx) · sin(nπy) · sin(lπz)`: a standing wave in the unit cube with fixed value 0.
    SinMode { m: f64, n: f64, l: f64, amp: f64 },
    /// `amp · exp(−(dx² + dy² + dz²) / (2σ²))` with `dx = x − x0` etc. (sum left to right): a
    /// Gaussian bump.
    Gauss { x0: f64, y0: f64, z0: f64, sigma: f64, amp: f64 },
    /// `amp · cos(mπx) · cos(nπy) · cos(lπz)`: a standing wave in the unit cube with zero gradient.
    CosMode { m: f64, n: f64, l: f64, amp: f64 },
}

/// A 3D wave problem on a grid of `points[0] × points[1] × points[2]` points with spacing `h`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wave3dProblem {
    /// Number of grid points along x, y and z.
    pub points: [usize; 3],
    /// Grid spacing (the same on all axes).
    pub h: f64,
    /// Wave speed.
    pub c: f64,
    /// Time step.
    pub dt: f64,
    /// Initial displacement.
    pub initial: Initial,
    /// Outer boundary per face `2·axis + side`: x low, x high, y low, y high, z low, z high.
    pub outer: [Outer; 6],
}

impl Wave3dProblem {
    /// Courant number `ν = c Δt / h` (stable for `ν ≤ 1/√3` on an unbounded grid).
    pub fn courant(&self) -> f64 {
        self.c * self.dt / self.h
    }

    /// Initial displacement at global point `(gi, gj, gk)`.
    fn initial_at(&self, gi: usize, gj: usize, gk: usize) -> f64 {
        let x = (gi as f64) * self.h;
        let y = (gj as f64) * self.h;
        let z = (gk as f64) * self.h;
        match self.initial {
            Initial::Gauss { x0, y0, z0, sigma, amp } => {
                let (dx, dy, dz) = (x - x0, y - y0, z - z0);
                amp * libm::exp(-(dx * dx + dy * dy + dz * dz) / (2.0 * sigma * sigma))
            }
            Initial::CosMode { m, n, l, amp } => {
                amp * libm::cos(m * PI * x) * libm::cos(n * PI * y) * libm::cos(l * PI * z)
            }
            Initial::SinMode { m, n, l, amp } => {
                amp * libm::sin(m * PI * x) * libm::sin(n * PI * y) * libm::sin(l * PI * z)
            }
        }
    }
}

/// The native 3D wave solver for one patch.
pub struct Wave3dPatch {
    nu2: f64,
    half_nu2: f64,
    kmur: f64,
    /// Layout, faces, neighbours and exchange schedule.
    win: Window<3>,
    /// Current level `u^n` with the overlap layers and one layer of ghost points, x fastest.
    u: Vec<f64>,
    /// Previous level `u^{n−1}` (overwritten point by point with `u^{n+1}` during a step).
    p: Vec<f64>,
}

impl Wave3dPatch {
    /// Sets up the solver of block `b` of `problem` at step 0, exchanging one layer every step.
    pub fn new(problem: &Wave3dProblem, b: &Block<3>) -> Result<Self, PatchError> {
        Self::with_halo(problem, b, 1)
    }

    /// Sets up the solver of block `b` of `problem` at step 0, exchanging `halo` layers once every
    /// `halo` steps. Fails if the block has fewer than `halo` points along an axis on which it has a
    /// neighbour.
    pub fn with_halo(problem: &Wave3dProblem, b: &Block<3>, halo: usize) -> Result<Self, PatchError> {
        let n = b.size;
        if n.iter().any(|&k| k < MIN_POINTS) {
            return Err(PatchError(format!(
                "patch of {}×{}×{} points; at least {MIN_POINTS} per axis needed",
                n[0], n[1], n[2]
            )));
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
        let mut s = Wave3dPatch {
            nu2,
            half_nu2: 0.5 * nu2,
            kmur: (nu - 1.0) / (nu + 1.0),
            win,
            u: vec![0.0; total],
            p: vec![0.0; total],
        };
        let [ix0, iy0, iz0] = b.origin;
        for k in 0..n[2] {
            for j in 0..n[1] {
                for i in 0..n[0] {
                    let idx = s.win.at([i as isize, j as isize, k as isize]);
                    s.u[idx] = problem.initial_at(ix0 + i, iy0 + j, iz0 + k);
                }
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

    /// Update of the points of `region` (rows along x are independent; every point uses the
    /// operations of the module documentation, in that order).
    fn update(&mut self, region: &crate::halo::Range<3>) {
        let [(x0, x1), (y0, y1), (z0, z1)] = *region;
        let nx = (x1 - x0) as usize;
        let (w, plane) = (self.win.stride(1), self.win.stride(2));
        let (nu2, half_nu2) = (self.nu2, self.half_nu2);
        let first = self.win.step == 0;
        for k in z0..z1 {
            for j in y0..y1 {
                let row = self.win.at([x0, j, k]);
                let west = &self.u[row - 1..row - 1 + nx];
                let mid = &self.u[row..row + nx];
                let east = &self.u[row + 1..row + 1 + nx];
                let south = &self.u[row - w..row - w + nx];
                let north = &self.u[row + w..row + w + nx];
                let down = &self.u[row - plane..row - plane + nx];
                let up = &self.u[row + plane..row + plane + nx];
                let out = &mut self.p[row..row + nx];
                if first {
                    for i in 0..nx {
                        let u = mid[i];
                        let lap = ((west[i] - 2.0 * u + east[i]) + (south[i] - 2.0 * u + north[i]))
                            + (down[i] - 2.0 * u + up[i]);
                        out[i] = u + half_nu2 * lap;
                    }
                } else {
                    for i in 0..nx {
                        let u = mid[i];
                        let lap = ((west[i] - 2.0 * u + east[i]) + (south[i] - 2.0 * u + north[i]))
                            + (down[i] - 2.0 * u + up[i]);
                        out[i] = 2.0 * u - out[i] + nu2 * lap;
                    }
                }
            }
        }
    }
}

impl PatchSolver for Wave3dPatch {
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

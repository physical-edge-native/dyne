//! Tutorial actor: one region of the 1D equation `u_t = alpha * u_xx - K * u`
//! (heat conduction with linear decay), explicit Euler in time, second-order
//! central differences in space.
//!
//! It implements the minimal set of exports that the 1D simulator `dyne-sim`
//! requires. The host (simulator or device firmware) owns time, messages and
//! neighbours; the actor only computes. See `docs/en/tutorial.md`.
//!
//! Bit-for-bit equality between a split run and a single-solver run holds
//! because every grid point is updated by the same expression, evaluated in the
//! same order, whether its neighbour value comes from the actor's own array or
//! from a neighbouring device (through `dyne_set_neighbor`).

#![no_std]

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

/// Largest number of grid points per region (static memory, no heap).
const N_MAX: usize = 1024;
/// Decay rate K [1/s].
const K: f64 = 2.0;

/// Face numbers: 0 = left (smaller x), 1 = right.
const FACE_LEFT: u32 = 0;
const FACE_RIGHT: u32 = 1;
/// Boundary condition codes passed to `dyne_init` by the host.
const BC_DIRICHLET: u32 = 0; // value fixed at 0
const BC_INTERFACE: u32 = 1; // border shared with a neighbouring region

struct State {
    u: [f64; N_MAX],
    u_next: [f64; N_MAX],
    ghost: [f64; 2],
    n: usize,
    x0: f64,
    h: f64,
    alpha: f64,
    dt: f64,
    bc: [u32; 2],
}

static mut STATE: State = State {
    u: [0.0; N_MAX],
    u_next: [0.0; N_MAX],
    ghost: [0.0; 2],
    n: 0,
    x0: 0.0,
    h: 0.0,
    alpha: 0.0,
    dt: 0.0,
    bc: [BC_DIRICHLET; 2],
};

#[allow(static_mut_refs)]
fn st() -> &'static mut State {
    // WebAssembly modules here are single-threaded and hold one region each.
    unsafe { &mut STATE }
}

/// Set up the region `[x0, x1]` with `n` points (both ends included) and the
/// initial condition `u(x, 0) = 4 x (1 - x)`. Returns 0 on success.
#[no_mangle]
pub extern "C" fn dyne_init(n: u32, x0: f64, x1: f64, alpha: f64, dt: f64, bc_left: u32, bc_right: u32) -> i32 {
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
    for i in 0..n {
        let x = x0 + (i as f64) * s.h;
        s.u[i] = 4.0 * x * (1.0 - x);
    }
    if bc_left == BC_DIRICHLET {
        s.u[0] = 0.0;
    }
    if bc_right == BC_DIRICHLET {
        s.u[n - 1] = 0.0;
    }
    0
}

/// The update of one point from its left value `l`, its own value `c` and its
/// right value `r`. Every point, inside or at a border, uses this expression.
fn update(c: f64, l: f64, r: f64, k: f64, dt: f64) -> f64 {
    c + k * (l - 2.0 * c + r) - dt * K * c
}

/// Advance one time step. The host has already passed the neighbours' border
/// values with `dyne_set_neighbor` for every interface face.
#[no_mangle]
pub extern "C" fn dyne_step() {
    let s = st();
    let n = s.n;
    let k = s.alpha * s.dt / (s.h * s.h);
    let left = match s.bc[0] {
        BC_INTERFACE => update(s.u[0], s.ghost[0], s.u[1], k, s.dt),
        _ => 0.0,
    };
    let right = match s.bc[1] {
        BC_INTERFACE => update(s.u[n - 1], s.u[n - 2], s.ghost[1], k, s.dt),
        _ => 0.0,
    };
    for i in 1..n - 1 {
        s.u_next[i] = update(s.u[i], s.u[i - 1], s.u[i + 1], k, s.dt);
    }
    s.u_next[0] = left;
    s.u_next[n - 1] = right;
    core::mem::swap(&mut s.u, &mut s.u_next);
}

/// Receive the neighbour's border value (and gradient, unused here) for `face`.
#[no_mangle]
pub extern "C" fn dyne_set_neighbor(face: u32, value: f64, _grad: f64) {
    if face <= 1 {
        st().ghost[face as usize] = value;
    }
}

/// This region's value at `face`, sent to the neighbour on that side.
#[no_mangle]
pub extern "C" fn dyne_boundary_value(face: u32) -> f64 {
    let s = st();
    match face {
        FACE_LEFT => s.u[0],
        FACE_RIGHT => s.u[s.n - 1],
        _ => f64::NAN,
    }
}

/// One-sided gradient at `face` (carried with the value in each border message).
#[no_mangle]
pub extern "C" fn dyne_boundary_grad(face: u32) -> f64 {
    let s = st();
    match face {
        FACE_LEFT => (s.u[1] - s.u[0]) / s.h,
        FACE_RIGHT => (s.u[s.n - 1] - s.u[s.n - 2]) / s.h,
        _ => f64::NAN,
    }
}

/// Number of grid points of this region.
#[no_mangle]
pub extern "C" fn dyne_len() -> u32 {
    st().n as u32
}

/// Value at local point `i`.
#[no_mangle]
pub extern "C" fn dyne_get(i: u32) -> f64 {
    let s = st();
    if (i as usize) < s.n { s.u[i as usize] } else { f64::NAN }
}

/// Overwrite the value at local point `i` (used when a device takes over the
/// state of another one).
#[no_mangle]
pub extern "C" fn dyne_set(i: u32, value: f64) {
    let s = st();
    if (i as usize) < s.n {
        s.u[i as usize] = value;
    }
}

/// Position of local point `i`.
#[no_mangle]
pub extern "C" fn dyne_x(i: u32) -> f64 {
    let s = st();
    s.x0 + (i as f64) * s.h
}

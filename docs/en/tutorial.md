# Tutorial: write your own solver and check it in the simulator

In this tutorial you write a solver for a 1D equation that Dyne does not ship,
compile it to WebAssembly, and check in the simulator that splitting it across
devices does not change a single bit — also under message loss, delay and a
device being replaced while running. You then break the solver on purpose and
see the check catch it.

You need the [installation](installation.md) steps 1 and 3 done (Rust, the
`wasm32-unknown-unknown` target, and `dyne-sim` built). The finished solver is
in [`examples/decay1d-actor`](../../examples/decay1d-actor); CI builds it and
runs the commands of this page.

## The equation

Heat conduction with linear decay on the interval [0, 1]:

    u_t = alpha * u_xx - K * u,   u(0, t) = u(1, t) = 0,   u(x, 0) = 4 x (1 - x)

discretised with central differences in space and explicit Euler in time. With
`k = alpha * dt / h^2`, each point is updated from its left value `l`, its own
value `c` and its right value `r`:

    c_new = c + k * (l - 2c + r) - dt * K * c

## How the simulator uses a solver

Each simulated device loads its own copy of the WebAssembly module, which holds
one region of the grid. The host (the simulator, or the firmware on a device)
owns time and messages; the solver only computes. In every step the host:

1. reads the values at the solver's borders (`dyne_boundary_value`,
   `dyne_boundary_grad`) and sends them to the neighbours;
2. gives the solver the values received from the neighbours
   (`dyne_set_neighbor`);
3. calls `dyne_step`.

The simulator also runs one unsplit copy over the whole grid and, at the end,
compares every grid point of the split run with it, bit for bit.

The 1D simulator `dyne-sim` needs these exports:

| Export | Purpose |
|---|---|
| `dyne_init(n, x0, x1, alpha, dt, bc_left, bc_right) -> i32` | set up a region of `n` points on `[x0, x1]`; each face is `0` (value fixed at 0, outer edge) or `1` (shared with a neighbour); return 0 on success |
| `dyne_step()` | advance one time step |
| `dyne_set_neighbor(face, value, grad)` | the neighbour's border value (and gradient) on `face` (0 = left, 1 = right) |
| `dyne_boundary_value(face) -> f64`, `dyne_boundary_grad(face) -> f64` | this region's value and one-sided gradient at `face`, sent to that neighbour |
| `dyne_len() -> u32`, `dyne_get(i) -> f64`, `dyne_x(i) -> f64` | number of points, value and position of local point `i` |
| `dyne_set(i, value)` | overwrite local point `i` (used when a device takes over another's state) |

Optional exports add features: `dyne_observe` (pull a point towards a
measurement, for data assimilation), `dyne_cfg` (configuration values),
`dyne_set_bc_value` (non-zero fixed values) and others; see
`crates/heat1d-actor` and `crates/dyne-core/src/actor.rs`.

## Step 1: create the crate

```sh
mkdir -p examples/decay1d-actor/src examples/decay1d-actor/.cargo
```

`examples/decay1d-actor/Cargo.toml`:

```toml
[package]
name = "decay1d-actor"
version = "0.1.0"
edition = "2021"
publish = false

[lib]
crate-type = ["cdylib"]      # a WebAssembly module, not a Rust library

[profile.release]
panic = "abort"
opt-level = "z"
lto = true

[workspace]                  # built on its own, outside the main workspace
```

`examples/decay1d-actor/.cargo/config.toml` keeps the WebAssembly stack small.
Rust's default of 1 MB would not fit a microcontroller, and the simulator
rejects a module whose memory would not fit a device:

```toml
[target.wasm32-unknown-unknown]
rustflags = ["-C", "link-arg=-zstack-size=32768"]
```

## Step 2: write the solver

The solver is `no_std` (no operating system on a microcontroller) and keeps its
state in a static array (no heap). The full file is
[`examples/decay1d-actor/src/lib.rs`](../../examples/decay1d-actor/src/lib.rs);
the part that matters for bit-for-bit equality is this:

```rust
/// The update of one point from its left value `l`, its own value `c` and its
/// right value `r`. Every point, inside or at a border, uses this expression.
fn update(c: f64, l: f64, r: f64, k: f64, dt: f64) -> f64 {
    c + k * (l - 2.0 * c + r) - dt * K * c
}

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
```

A point at a border between two devices gets its outer neighbour from the other
device (`s.ghost`), but it goes through **the same expression in the same
order** as a point inside a region. Floating-point addition is not associative,
so this is what makes the split solution identical to the unsplit one.

## Step 3: build

```sh
(cd examples/decay1d-actor && cargo build --release --target wasm32-unknown-unknown)
```

The module is
`examples/decay1d-actor/target/wasm32-unknown-unknown/release/decay1d_actor.wasm`.

## Step 4: check it in the simulator

Four devices, no faults:

```sh
cargo run --release -p dyne-sim -- \
  --wasm examples/decay1d-actor/target/wasm32-unknown-unknown/release/decay1d_actor.wasm \
  --nodes 4 --points 65 --steps 2000
```

The end of the output:

```
bit-exact vs single: 65/65 points, max_diff=0.000e0
...
RESULT: PASS (bit-exact)
```

(The line `L2 vs analytic` compares with the analytic solution of the plain
heat equation, which the simulator knows; it does not apply to this equation
and can be ignored.)

Eight devices, 10% of messages lost, 5 ± 3 ms delay, and device 3 going silent
at 30 s of virtual time and replaced at 60 s by a new instance that takes over
its state:

```sh
cargo run --release -p dyne-sim -- \
  --wasm examples/decay1d-actor/target/wasm32-unknown-unknown/release/decay1d_actor.wasm \
  --nodes 8 --points 65 --steps 2000 --loss 0.1 --lat-ms 5 --lat-jitter-ms 3 --handover 3@30000:60000
```

```
[sim] handover applied: node 3 reincarnated at t=60000ms (state 8 pts carried)
...
RESULT: PASS (bit-exact)
```

Lost messages are sent again, so loss turns into waiting rather than error. The
run is deterministic: the same command prints the same output on every
operating system.

## Step 5: break it and watch the check catch it

Change the left border to add the same three terms in a different order:

```rust
BC_INTERFACE => s.u[0] + k * (s.u[1] - 2.0 * s.u[0] + s.ghost[0]) - s.dt * K * s.u[0],
```

Mathematically this is the same update. Rebuild and run the four-device check
again:

```
bit-exact vs single: 11/65 points, max_diff=2.776e-17
...
RESULT: FAIL
```

The difference is in the last bit (2.8e-17), but the check reports it, and the
simulator exits with a non-zero status. Undo the change before going on.

## Where to go from here

- **2D:** `crates/heat2d-actor` is the 2D counterpart, run by `dyne-sim2d`
  (faces 0..3 = west, east, south, north; the shared constants are in
  `dyne_wire::abi::field2d`). Its exports are described in its source.
- **Measurements:** add `dyne_observe(i, y, gamma) -> f64` to pull point `i`
  towards a measured value `y` (data assimilation by nudging); see
  `crates/heat1d-actor` and the `--nudge-gamma` option of `dyne-sim`.
- **Devices:** the same `.wasm` file runs on ESP32 boards; the
  [hardware guide](hardware.md) shows how solvers are deployed and checked
  against the simulator.

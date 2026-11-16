//! dyne-native-wave2d: the 2D wave equation split into patches, one thread per patch.
//!
//! ```text
//! dyne-native-wave2d check   [--grid 41x41] [--steps 400] [--splits 1x1,2x2,4x4,8x8,3x5]
//!                            [--initial gauss|sin|cos] [--outer fixed|zero-gradient|absorbing]
//!                            [--halo 1,2,...] [--wasm PATH]
//! dyne-native-wave2d scaling --mode strong [--grid 2048x2048] [--cores 1,2,4,8] [--steps 100] [--repeat 3]
//! dyne-native-wave2d scaling --mode weak   [--patch 512x512]  [--cores 1,2,4,8] [--steps 100] [--repeat 3]
//!                            [--halo K] [--spin-us N] [--wasm PATH] [--no-verify]
//! ```
//!
//! `check` solves one problem as a single patch and with each split, prints the CRC-32 of each
//! global field (the hash Dyne's simulators print) and `PASS (bit-exact)` or `FAIL`; the exit code is
//! 1 if any split differs.
//!
//! `scaling` measures the wall time per step of the step loop. Strong scaling keeps the grid fixed
//! and splits it into one patch per core; weak scaling keeps the patch size fixed and grows the grid
//! with the number of cores. Every timed run is also checked bit for bit against the single-patch
//! solve of the same grid (unless `--no-verify`). The problem is a standing wave
//! (`sin(πx)·sin(πy)`, fixed value 0, Courant number 0.5), chosen so that no subnormal numbers
//! appear (they are slow on most CPUs and would distort the timing).
//!
//! With `--wasm PATH`, patches run Dyne's WebAssembly wave solver (`wave2d-actor`) in the `wasmi`
//! interpreter instead of the native solver.
//!
//! `--halo K` sets the border width of the native solver: `K` layers of the neighbours' values,
//! exchanged once every `K` steps (default 1, one layer every step as in Dyne's solver;
//! `dyne_native::halo`). `check` accepts a list and checks every split with every width; a width
//! larger than a patch along a split axis is an error. Dyne's WebAssembly solver supports only 1.

use dyne_native::decomp::{Block, Decomposition};
use dyne_native::field_crc32;
use dyne_native::patch::{run_single, run_threaded, run_threaded_with, PatchError, PatchSolver, RunOptions};
use dyne_native::wasm2d::Wasm2dPatch;
use dyne_native::wave2d::{Initial, Outer, Wave2dPatch, Wave2dProblem, MIN_POINTS};
use std::collections::HashMap;
use std::process::ExitCode;
use std::time::{Duration, Instant};

fn usage() -> ! {
    eprintln!(
        "usage:\n  dyne-native-wave2d check [--grid NXxNY] [--steps N] [--splits PXxPY,...] [--initial gauss|sin|cos] \
         [--outer fixed|zero-gradient|absorbing] [--halo K,...] [--wasm PATH]\n  \
         dyne-native-wave2d scaling --mode strong|weak [--grid NXxNY] [--patch NXxNY] [--cores 1,2,...] [--steps N] \
         [--repeat N] [--halo K] [--spin-us N] [--wasm PATH] [--no-verify]"
    );
    std::process::exit(2)
}

fn pair(s: &str) -> [usize; 2] {
    let v: Vec<usize> = s.split('x').map(|t| t.parse().unwrap_or_else(|_| usage())).collect();
    if v.len() != 2 {
        usage()
    }
    [v[0], v[1]]
}

fn parse_args(args: &[String]) -> HashMap<String, String> {
    let mut m = HashMap::new();
    let mut i = 0;
    while i < args.len() {
        let k = args[i].strip_prefix("--").unwrap_or_else(|| usage()).to_string();
        if k == "no-verify" || k == "help" {
            m.insert(k, String::new());
            i += 1;
        } else {
            let v = args.get(i + 1).unwrap_or_else(|| usage()).clone();
            m.insert(k, v);
            i += 2;
        }
    }
    m
}

/// A problem in Dyne's convention: h = 1/(nx − 1), c = 1, Courant number 0.5.
fn problem(points: [usize; 2], initial: Initial, outer: Outer) -> Wave2dProblem {
    let h = 1.0 / ((points[0] - 1) as f64);
    Wave2dProblem { points, h, c: 1.0, dt: 0.5 * h, initial, outer: [outer; 4] }
}

/// Which patch solver to run.
enum Solver {
    Native,
    Wasm(wasmi::Engine, wasmi::Module),
}

/// Either solver behind one type, so that one runner serves both.
enum AnyPatch {
    Native(Wave2dPatch),
    Wasm(Box<Wasm2dPatch>),
}

impl PatchSolver for AnyPatch {
    fn outbox(&mut self, out: &mut Vec<u8>) -> Result<(), PatchError> {
        match self {
            AnyPatch::Native(s) => s.outbox(out),
            AnyPatch::Wasm(s) => s.outbox(out),
        }
    }
    fn inbox(&mut self, framed: &[u8]) -> Result<(), PatchError> {
        match self {
            AnyPatch::Native(s) => s.inbox(framed),
            AnyPatch::Wasm(s) => s.inbox(framed),
        }
    }
    fn step(&mut self) -> Result<(), PatchError> {
        match self {
            AnyPatch::Native(s) => s.step(),
            AnyPatch::Wasm(s) => s.step(),
        }
    }
    fn field(&mut self) -> Result<Vec<f64>, PatchError> {
        match self {
            AnyPatch::Native(s) => s.field(),
            AnyPatch::Wasm(s) => s.field(),
        }
    }
    fn halo_width(&self) -> usize {
        match self {
            AnyPatch::Native(s) => s.halo_width(),
            AnyPatch::Wasm(s) => s.halo_width(),
        }
    }
}

impl Solver {
    fn from_args(a: &HashMap<String, String>) -> Solver {
        match a.get("wasm") {
            None => Solver::Native,
            Some(path) => {
                let bytes = std::fs::read(path).unwrap_or_else(|e| {
                    eprintln!("cannot read {path}: {e}");
                    std::process::exit(2)
                });
                let engine = wasmi::Engine::default();
                let module = wasmi::Module::new(&engine, &bytes[..]).expect("valid WebAssembly module");
                Solver::Wasm(engine, module)
            }
        }
    }

    fn name(&self) -> &'static str {
        match self {
            Solver::Native => "native",
            Solver::Wasm(..) => "WebAssembly (wasmi)",
        }
    }

    fn make(&self, p: &Wave2dProblem, b: &Block<2>, halo: usize) -> Result<AnyPatch, PatchError> {
        Ok(match self {
            Solver::Native => AnyPatch::Native(Wave2dPatch::with_halo(p, b, halo)?),
            Solver::Wasm(..) if halo != 1 => {
                return Err(PatchError(format!(
                    "halo width {halo}: Dyne's WebAssembly wave solver exchanges one layer every step (halo 1 only)"
                )))
            }
            Solver::Wasm(e, m) => AnyPatch::Wasm(Box::new(Wasm2dPatch::new_wave(e, m, p, b)?)),
        })
    }

    fn single(&self, p: &Wave2dProblem, steps: u64) -> Result<Vec<f64>, PatchError> {
        let d = Decomposition::whole(p.points, MIN_POINTS).map_err(|e| PatchError(e.0))?;
        run_single(&mut self.make(p, &d.blocks[0], 1)?, steps)
    }
}

fn same_bits(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
}

fn check(a: &HashMap<String, String>) -> Result<bool, PatchError> {
    let points = pair(a.get("grid").map_or("41x41", |s| s));
    let steps: u64 = a.get("steps").map_or(Ok(400), |s| s.parse()).unwrap_or_else(|_| usage());
    let initial = match a.get("initial").map_or("gauss", |s| s) {
        "gauss" => Initial::Gauss { x0: 0.3, y0: 0.6, sigma: 0.06, amp: 1.0 },
        "sin" => Initial::SinMode { m: 1.0, n: 1.0, amp: 1.0 },
        "cos" => Initial::CosMode { m: 1.0, n: 1.0, amp: 1.0 },
        _ => usage(),
    };
    let outer = match a.get("outer").map_or("fixed", |s| s) {
        "fixed" => Outer::Fixed(0.0),
        "zero-gradient" => Outer::ZeroGradient,
        "absorbing" => Outer::Absorbing,
        _ => usage(),
    };
    let splits: Vec<[usize; 2]> = a.get("splits").map_or("1x1,2x2,4x4,8x8,3x5", |s| s).split(',').map(pair).collect();
    let halos = halo_list(a);
    let solver = Solver::from_args(a);
    let p = problem(points, initial, outer);
    println!(
        "dyne-native-wave2d check: grid {}x{}, {steps} steps, Courant number {}, {} solver",
        points[0],
        points[1],
        p.courant(),
        solver.name()
    );
    let reference = solver.single(&p, steps)?;
    println!("single patch      crc32 {:08x}", field_crc32(&reference));
    let mut ok = true;
    for &halo in &halos {
        for &parts in &splits {
            let d = Decomposition::new(points, parts, MIN_POINTS).map_err(|e| PatchError(e.0))?;
            let r = run_threaded(&d, steps, |b| solver.make(&p, b, halo))?;
            let pass = same_bits(&reference, &r.field);
            ok &= pass;
            println!(
                "split {:>2}x{:<2} ({:>2} threads) halo {halo} crc32 {:08x}  {}",
                parts[0],
                parts[1],
                d.blocks.len(),
                field_crc32(&r.field),
                if pass { "PASS (bit-exact)" } else { "FAIL" }
            );
        }
    }
    Ok(ok)
}

/// The `--halo` list (default 1).
fn halo_list(a: &HashMap<String, String>) -> Vec<usize> {
    a.get("halo").map_or("1", |s| s).split(',').map(|t| t.parse().unwrap_or_else(|_| usage())).collect()
}

/// `p` patches as `px × py` with `py` the largest divisor of `p` not above √p (px ≥ py).
fn factor(p: usize) -> [usize; 2] {
    let mut py = 1;
    for d in 1..=p {
        if d * d > p {
            break;
        }
        if p % d == 0 {
            py = d;
        }
    }
    [p / py, py]
}

fn scaling(a: &HashMap<String, String>) -> Result<bool, PatchError> {
    let mode = a.get("mode").map(|s| s.as_str()).unwrap_or_else(|| usage());
    let strong = match mode {
        "strong" => true,
        "weak" => false,
        _ => usage(),
    };
    let cores: Vec<usize> = a
        .get("cores")
        .map_or("1,2,4,8", |s| s)
        .split(',')
        .map(|t| t.parse().unwrap_or_else(|_| usage()))
        .collect();
    let steps: u64 = a.get("steps").map_or(Ok(100), |s| s.parse()).unwrap_or_else(|_| usage());
    let repeat: usize = a.get("repeat").map_or(Ok(3), |s| s.parse()).unwrap_or_else(|_| usage());
    let verify = !a.contains_key("no-verify");
    let spin_us: u64 = a.get("spin-us").map_or(Ok(0), |s| s.parse()).unwrap_or_else(|_| usage());
    let options = RunOptions { spin: Duration::from_micros(spin_us) };
    let halo: usize = a.get("halo").map_or(Ok(1), |s| s.parse()).unwrap_or_else(|_| usage());
    let solver = Solver::from_args(a);
    let grid = pair(a.get("grid").map_or("2048x2048", |s| s));
    let patch = pair(a.get("patch").map_or("512x512", |s| s));
    let initial = Initial::SinMode { m: 1.0, n: 1.0, amp: 1.0 };
    let logical = std::thread::available_parallelism().map_or(0, |n| n.get());

    if strong {
        println!(
            "strong scaling: grid {}x{}, {steps} steps, median of {repeat}, {} solver, halo {halo}, spin {spin_us} us, {logical} logical CPUs",
            grid[0],
            grid[1],
            solver.name()
        );
    } else {
        println!(
            "weak scaling: {}x{} points per patch, {steps} steps, median of {repeat}, {} solver, halo {halo}, spin {spin_us} us, {logical} logical CPUs",
            patch[0],
            patch[1],
            solver.name()
        );
    }
    println!();
    println!(
        "| cores | split | grid | points per patch (max) | ms per step | compute ms per step (mean / max patch) | speed-up | efficiency | bit-exact |"
    );
    println!("|---:|---:|---:|---:|---:|---:|---:|---:|:---:|");

    let mut ok = true;
    let mut t1: Option<f64> = None;
    let mut strong_reference: Option<Vec<f64>> = None;
    if strong && verify {
        strong_reference = Some(solver.single(&problem(grid, initial, Outer::Fixed(0.0)), steps)?);
    }
    for &c in &cores {
        let parts = factor(c);
        let points = if strong { grid } else { [patch[0] * parts[0], patch[1] * parts[1]] };
        let p = problem(points, initial, Outer::Fixed(0.0));
        let d = Decomposition::new(points, parts, MIN_POINTS).map_err(|e| PatchError(e.0))?;
        let mut runs = Vec::with_capacity(repeat);
        for _ in 0..repeat {
            let r = run_threaded_with(&d, steps, options, |b| solver.make(&p, b, halo))?;
            runs.push((r.wall.as_secs_f64() / steps as f64, r));
        }
        runs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let (t, median_run) = runs.swap_remove(runs.len() / 2);
        let compute: Vec<f64> = median_run.compute.iter().map(|c| c.as_secs_f64() / steps as f64).collect();
        let compute_mean = compute.iter().sum::<f64>() / compute.len() as f64;
        let compute_max = compute.iter().cloned().fold(0.0, f64::max);
        let field = median_run.field;
        let exact = if !verify {
            "-"
        } else {
            let reference = if strong { strong_reference.clone().unwrap() } else { solver.single(&p, steps)? };
            if same_bits(&reference, &field) {
                "yes"
            } else {
                ok = false;
                "NO"
            }
        };
        let base = *t1.get_or_insert(t * if strong { c as f64 } else { 1.0 });
        let (speedup, eff) = if strong {
            (base / t, base / (t * c as f64))
        } else {
            (base * c as f64 / t, base / t)
        };
        let max_patch = d.blocks.iter().map(|b| b.len()).max().unwrap();
        println!(
            "| {c} | {}x{} | {}x{} | {} | {:.3} | {:.3} / {:.3} | {:.2} | {:.0}% | {exact} |",
            parts[0],
            parts[1],
            points[0],
            points[1],
            max_patch,
            t * 1e3,
            compute_mean * 1e3,
            compute_max * 1e3,
            speedup,
            eff * 100.0
        );
    }
    // The same grid as the one-core row, as one patch stepped in a plain loop (no messages), on a
    // fresh thread like the workers.
    let p = problem(if strong { grid } else { patch }, initial, Outer::Fixed(0.0));
    let d = Decomposition::whole(p.points, MIN_POINTS).map_err(|e| PatchError(e.0))?;
    let mut serial = Vec::with_capacity(repeat);
    for _ in 0..repeat {
        let t = std::thread::scope(|sc| {
            sc.spawn(|| -> Result<f64, PatchError> {
                let mut s = solver.make(&p, &d.blocks[0], 1)?;
                let t0 = Instant::now();
                for _ in 0..steps {
                    s.step()?;
                }
                Ok(t0.elapsed().as_secs_f64() / steps as f64)
            })
            .join()
            .expect("serial run")
        })?;
        serial.push(t);
    }
    serial.sort_by(|a, b| a.partial_cmp(b).unwrap());
    println!();
    println!(
        "one patch in a plain step loop (no messages), {}x{}: {:.3} ms per step",
        p.points[0],
        p.points[1],
        serial[serial.len() / 2] * 1e3
    );
    Ok(ok)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(cmd) = args.first() else { usage() };
    let a = parse_args(&args[1..]);
    let r = match cmd.as_str() {
        "check" => check(&a),
        "scaling" => scaling(&a),
        _ => usage(),
    };
    match r {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => {
            println!("FAIL: a split differs from the single-patch solve");
            ExitCode::FAILURE
        }
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

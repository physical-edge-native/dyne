//! dyne-native-wave3d: the 3D wave equation split into patches, one thread per patch.
//!
//! ```text
//! dyne-native-wave3d check   [--grid 25x23x21] [--steps 100] [--splits 1x1x1,2x2x2,3x2x1,4x4x2]
//!                            [--initial gauss|sin|cos] [--outer fixed|zero-gradient|absorbing] [--halo 1,2,...]
//! dyne-native-wave3d scaling --mode strong [--grid 256x256x256] [--cores 1,2,4,8] [--steps 20] [--repeat 3]
//! dyne-native-wave3d scaling --mode weak   [--patch 64x64x64]   [--cores 1,2,4,8] [--steps 20] [--repeat 3]
//!                            [--halo K] [--spin-us N] [--no-verify]
//! dyne-native-wave3d convergence
//! ```
//!
//! `check` solves one problem as a single patch and with each split, prints the CRC-32 of each
//! global field (the field hash that Dyne's simulators print) and `PASS (bit-exact)` or `FAIL`; the
//! exit code is 1 if any split differs. The problem uses h = 1/(nx − 1), c = 1 and Courant number 0.5.
//!
//! `scaling` measures the wall time per step of the step loop. Strong scaling keeps the grid fixed and
//! splits it into one patch per core; weak scaling keeps the patch size fixed and grows the grid with
//! the number of cores. `p` cores are split as `px × py × pz` with `pz` the largest divisor of `p`
//! not above its cube root and `px ≥ py ≥ pz`. Every timed run is also checked bit for bit against
//! the single-patch solve of the same grid (unless `--no-verify`). The problem is a standing wave
//! (`sin(πx)·sin(πy)·sin(πz)`, fixed value 0, Courant number 0.5), chosen so that no subnormal
//! numbers appear (they are slow on most CPUs and would distort the timing).
//!
//! `--halo K` sets the border width: `K` layers of the neighbours' values (faces, edges and corners),
//! exchanged once every `K` steps (default 1: one layer of face values every step;
//! `dyne_native::halo`). `check` accepts a list and checks every split with every width; a width
//! larger than a patch along a split axis is an error.
//!
//! `convergence` runs the convergence-order check of the solver against an analytic standing wave
//! (`dyne_native::convergence`): it prints the error table and the observed orders, then
//! `PASS (order within [1.9, 2.1])` or `FAIL` (exit code 1).

use dyne_native::decomp::Decomposition;
use dyne_native::field_crc32;
use dyne_native::patch::{run_single, run_threaded, run_threaded_with, PatchError, PatchSolver, RunOptions};
use dyne_native::wave3d::{Initial, Outer, Wave3dPatch, Wave3dProblem, MIN_POINTS};
use std::collections::HashMap;
use std::process::ExitCode;
use std::time::{Duration, Instant};

fn usage() -> ! {
    eprintln!(
        "usage:\n  dyne-native-wave3d check [--grid NXxNYxNZ] [--steps N] [--splits PXxPYxPZ,...] [--initial gauss|sin|cos] \
         [--outer fixed|zero-gradient|absorbing] [--halo K,...]\n  \
         dyne-native-wave3d scaling --mode strong|weak [--grid NXxNYxNZ] [--patch NXxNYxNZ] [--cores 1,2,...] [--steps N] \
         [--repeat N] [--halo K] [--spin-us N] [--no-verify]\n  \
         dyne-native-wave3d convergence"
    );
    std::process::exit(2)
}

fn triple(s: &str) -> [usize; 3] {
    let v: Vec<usize> = s.split('x').map(|t| t.parse().unwrap_or_else(|_| usage())).collect();
    if v.len() != 3 {
        usage()
    }
    [v[0], v[1], v[2]]
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

/// h = 1/(nx − 1), c = 1, Courant number 0.5.
fn problem(points: [usize; 3], initial: Initial, outer: Outer) -> Wave3dProblem {
    let h = 1.0 / ((points[0] - 1) as f64);
    Wave3dProblem { points, h, c: 1.0, dt: 0.5 * h, initial, outer: [outer; 6] }
}

fn single(p: &Wave3dProblem, steps: u64) -> Result<Vec<f64>, PatchError> {
    let d = Decomposition::whole(p.points, MIN_POINTS).map_err(|e| PatchError(e.0))?;
    run_single(&mut Wave3dPatch::new(p, &d.blocks[0])?, steps)
}

fn same_bits(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
}

fn check(a: &HashMap<String, String>) -> Result<bool, PatchError> {
    let points = triple(a.get("grid").map_or("25x23x21", |s| s));
    let steps: u64 = a.get("steps").map_or(Ok(100), |s| s.parse()).unwrap_or_else(|_| usage());
    let initial = match a.get("initial").map_or("gauss", |s| s) {
        "gauss" => Initial::Gauss { x0: 0.3, y0: 0.6, z0: 0.45, sigma: 0.1, amp: 1.0 },
        "sin" => Initial::SinMode { m: 1.0, n: 1.0, l: 1.0, amp: 1.0 },
        "cos" => Initial::CosMode { m: 1.0, n: 1.0, l: 1.0, amp: 1.0 },
        _ => usage(),
    };
    let outer = match a.get("outer").map_or("fixed", |s| s) {
        "fixed" => Outer::Fixed(0.0),
        "zero-gradient" => Outer::ZeroGradient,
        "absorbing" => Outer::Absorbing,
        _ => usage(),
    };
    let splits: Vec<[usize; 3]> =
        a.get("splits").map_or("1x1x1,2x2x2,3x2x1,4x4x2", |s| s).split(',').map(triple).collect();
    let halos: Vec<usize> =
        a.get("halo").map_or("1", |s| s).split(',').map(|t| t.parse().unwrap_or_else(|_| usage())).collect();
    let p = problem(points, initial, outer);
    println!(
        "dyne-native-wave3d check: grid {}x{}x{}, {steps} steps, Courant number {}, native solver",
        points[0],
        points[1],
        points[2],
        p.courant()
    );
    let reference = single(&p, steps)?;
    println!("single patch         crc32 {:08x}", field_crc32(&reference));
    let mut ok = true;
    for &halo in &halos {
        for &parts in &splits {
            let d = Decomposition::new(points, parts, MIN_POINTS).map_err(|e| PatchError(e.0))?;
            let r = run_threaded(&d, steps, |b| Wave3dPatch::with_halo(&p, b, halo))?;
            let pass = same_bits(&reference, &r.field);
            ok &= pass;
            println!(
                "split {:>2}x{:>2}x{:<2} ({:>3} threads) halo {halo} crc32 {:08x}  {}",
                parts[0],
                parts[1],
                parts[2],
                d.blocks.len(),
                field_crc32(&r.field),
                if pass { "PASS (bit-exact)" } else { "FAIL" }
            );
        }
    }
    Ok(ok)
}

/// `p` patches as `px × py × pz`: `pz` the largest divisor of `p` with `pz³ ≤ p`, then the rest split
/// as `px × py` with `py` the largest divisor not above its square root (`px ≥ py ≥ pz`).
fn factor(p: usize) -> [usize; 3] {
    let mut pz = 1;
    for d in 1..=p {
        if d * d * d > p {
            break;
        }
        if p % d == 0 {
            pz = d;
        }
    }
    let q = p / pz;
    let mut py = 1;
    for d in 1..=q {
        if d * d > q {
            break;
        }
        if q % d == 0 {
            py = d;
        }
    }
    [q / py, py, pz]
}

fn scaling(a: &HashMap<String, String>) -> Result<bool, PatchError> {
    let strong = match a.get("mode").map(|s| s.as_str()).unwrap_or_else(|| usage()) {
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
    let steps: u64 = a.get("steps").map_or(Ok(20), |s| s.parse()).unwrap_or_else(|_| usage());
    let repeat: usize = a.get("repeat").map_or(Ok(3), |s| s.parse()).unwrap_or_else(|_| usage());
    let verify = !a.contains_key("no-verify");
    let spin_us: u64 = a.get("spin-us").map_or(Ok(0), |s| s.parse()).unwrap_or_else(|_| usage());
    let options = RunOptions { spin: Duration::from_micros(spin_us) };
    let halo: usize = a.get("halo").map_or(Ok(1), |s| s.parse()).unwrap_or_else(|_| usage());
    let grid = triple(a.get("grid").map_or("256x256x256", |s| s));
    let patch = triple(a.get("patch").map_or("64x64x64", |s| s));
    let initial = Initial::SinMode { m: 1.0, n: 1.0, l: 1.0, amp: 1.0 };
    let logical = std::thread::available_parallelism().map_or(0, |n| n.get());

    if strong {
        println!(
            "strong scaling: grid {}x{}x{}, {steps} steps, median of {repeat}, native solver, halo {halo}, spin {spin_us} us, {logical} logical CPUs",
            grid[0], grid[1], grid[2]
        );
    } else {
        println!(
            "weak scaling: {}x{}x{} points per patch, {steps} steps, median of {repeat}, native solver, halo {halo}, spin {spin_us} us, {logical} logical CPUs",
            patch[0], patch[1], patch[2]
        );
    }
    println!();
    println!(
        "| cores | split | grid | points per patch (max) | ms per step | compute ms per step (mean / max patch) | speed-up | efficiency | bit-exact |"
    );
    println!("|---:|---:|---:|---:|---:|---:|---:|---:|:---:|");

    let mut ok = true;
    let mut t1: Option<f64> = None;
    let strong_reference = if strong && verify {
        Some(single(&problem(grid, initial, Outer::Fixed(0.0)), steps)?)
    } else {
        None
    };
    for &c in &cores {
        let parts = factor(c);
        let points = if strong {
            grid
        } else {
            [patch[0] * parts[0], patch[1] * parts[1], patch[2] * parts[2]]
        };
        let p = problem(points, initial, Outer::Fixed(0.0));
        let d = Decomposition::new(points, parts, MIN_POINTS).map_err(|e| PatchError(e.0))?;
        let mut runs = Vec::with_capacity(repeat);
        for _ in 0..repeat {
            let r = run_threaded_with(&d, steps, options, |b| Wave3dPatch::with_halo(&p, b, halo))?;
            runs.push((r.wall.as_secs_f64() / steps as f64, r));
        }
        runs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let (t, median_run) = runs.swap_remove(runs.len() / 2);
        let compute: Vec<f64> = median_run.compute.iter().map(|c| c.as_secs_f64() / steps as f64).collect();
        let compute_mean = compute.iter().sum::<f64>() / compute.len() as f64;
        let compute_max = compute.iter().cloned().fold(0.0, f64::max);
        let exact = if !verify {
            "-"
        } else {
            let same = match &strong_reference {
                Some(r) => same_bits(r, &median_run.field),
                None => same_bits(&single(&p, steps)?, &median_run.field),
            };
            if same {
                "yes"
            } else {
                ok = false;
                "NO"
            }
        };
        let base = *t1.get_or_insert(t * if strong { c as f64 } else { 1.0 });
        let (speedup, eff) = if strong { (base / t, base / (t * c as f64)) } else { (base * c as f64 / t, base / t) };
        let max_patch = d.blocks.iter().map(|b| b.len()).max().unwrap();
        println!(
            "| {c} | {}x{}x{} | {}x{}x{} | {} | {:.3} | {:.3} / {:.3} | {:.2} | {:.0}% | {exact} |",
            parts[0],
            parts[1],
            parts[2],
            points[0],
            points[1],
            points[2],
            max_patch,
            t * 1e3,
            compute_mean * 1e3,
            compute_max * 1e3,
            speedup,
            eff * 100.0
        );
    }
    // The grid of the one-core row, as one patch stepped in a plain loop (no messages), on a fresh
    // thread like the workers.
    let p = problem(if strong { grid } else { patch }, initial, Outer::Fixed(0.0));
    let d = Decomposition::whole(p.points, MIN_POINTS).map_err(|e| PatchError(e.0))?;
    let mut serial = Vec::with_capacity(repeat);
    for _ in 0..repeat {
        let t = std::thread::scope(|sc| {
            sc.spawn(|| -> Result<f64, PatchError> {
                let mut s = Wave3dPatch::new(&p, &d.blocks[0])?;
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
        "one patch in a plain step loop (no messages), {}x{}x{}: {:.3} ms per step",
        p.points[0],
        p.points[1],
        p.points[2],
        serial[serial.len() / 2] * 1e3
    );
    Ok(ok)
}

/// The convergence-order check (see `dyne_native::convergence`); prints its own verdict.
fn convergence() -> Result<bool, PatchError> {
    let s = dyne_native::convergence::study_3d()?;
    print!("dyne-native-wave3d convergence: ");
    Ok(dyne_native::convergence::print_report(&s))
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(cmd) = args.first() else { usage() };
    let a = parse_args(&args[1..]);
    let r = match cmd.as_str() {
        "check" => check(&a),
        "scaling" => scaling(&a),
        "convergence" => convergence(),
        _ => usage(),
    };
    match r {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => {
            if cmd != "convergence" {
                println!("FAIL: a split differs from the single-patch solve");
            }
            ExitCode::FAILURE
        }
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

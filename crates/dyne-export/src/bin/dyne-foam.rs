//! dyne-foam: writes an OpenFOAM case from a Dyne field, reads it back, and imports a velocity field
//! computed by OpenFOAM.
//!
//! ```text
//! dyne-foam case --vti IN.vti --out DIR [--array NAME] [--diffusivity DT] [--velocity UX,UY,UZ]
//!                [--faces W,E,S,N[,B,T]] [--end-time S] [--delta-t S] [--write-interval S]
//! dyne-foam info DIR
//! dyne-foam velocity DIR TIME [--vti OUT.vti]
//! ```
//!
//! - `case` writes a case for `scalarTransportFoam` (see `dyne_export::foam`) from one array of a
//!   `.vti` file written by Dyne (default: the first array). A face is `fixed:VALUE`, `zero-gradient`
//!   or `inlet-outlet:VALUE`, in the order west, east, south, north (and bottom, top in 3D); the
//!   default is `zero-gradient` everywhere. Defaults: `--diffusivity 2.2e-5` (air, m²/s),
//!   `--velocity 0,0,0`, `--end-time 1`, `--delta-t 0.01`, `--write-interval 0.1`.
//! - `info` reads a case written by `case` (every file is checked against the others) and prints the
//!   grid and the CRC-32 of `T` (`dyne_export::field_crc32`, the field hash that Dyne's simulators
//!   print), so the exported field can be compared with the run that produced it.
//! - `velocity` reads `DIR/TIME/U` (ASCII, one vector per cell of the mesh exported by `case`) and
//!   prints the CRC-32 of each component; `--vti` writes the components as arrays `ux`, `uy`, `uz` of
//!   a `.vti` file on Dyne's grid.
//!
//! The exit code is 1 on any error, 2 on a usage error.

use dyne_export::field_crc32;
use dyne_export::foam::{import_velocity, Case, Face, Grid, Velocity};
use dyne_export::vti::ImageData;
use std::process::ExitCode;

const USAGE: &str = "usage:\n  dyne-foam case --vti IN.vti --out DIR [--array NAME] [--diffusivity DT] [--velocity UX,UY,UZ]\n                 [--faces W,E,S,N[,B,T]] [--end-time S] [--delta-t S] [--write-interval S]\n  dyne-foam info DIR\n  dyne-foam velocity DIR TIME [--vti OUT.vti]";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(|s| s.as_str()) {
        Some("case") => case(&args[1..]),
        Some("info") if args.len() == 2 => info(&args[1]),
        Some("velocity") if args.len() >= 3 => velocity(&args[1], &args[2], &args[3..]),
        _ => Err(Usage(String::new())),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(Usage(msg)) if !msg.starts_with("error: ") => {
            if !msg.is_empty() {
                eprintln!("{msg}");
            }
            eprintln!("{USAGE}");
            ExitCode::from(2)
        }
        Err(Usage(msg)) => {
            eprintln!("{msg}");
            ExitCode::FAILURE
        }
    }
}

/// A usage error, or (with the prefix "error: ") a failure.
struct Usage(String);

fn fail(e: impl std::fmt::Display) -> Usage {
    Usage(format!("error: {e}"))
}

/// `--key value` pairs; every key must be in `allowed`.
fn options<'a>(args: &'a [String], allowed: &[&str]) -> Result<Vec<(&'a str, &'a str)>, Usage> {
    if args.len() % 2 != 0 {
        return Err(Usage("options come in pairs: --key value".into()));
    }
    let mut out = Vec::new();
    for p in args.chunks(2) {
        if !allowed.contains(&p[0].as_str()) {
            return Err(Usage(format!("unknown option {}", p[0])));
        }
        out.push((p[0].as_str(), p[1].as_str()));
    }
    Ok(out)
}

fn number(key: &str, v: &str) -> Result<f64, Usage> {
    v.parse::<f64>().map_err(|_| Usage(format!("{key}: not a number: {v}")))
}

fn face(s: &str) -> Result<Face, Usage> {
    if s == "zero-gradient" {
        return Ok(Face::ZeroGradient);
    }
    if let Some(v) = s.strip_prefix("fixed:") {
        return Ok(Face::FixedValue(number("--faces", v)?));
    }
    if let Some(v) = s.strip_prefix("inlet-outlet:") {
        return Ok(Face::InletOutlet(number("--faces", v)?));
    }
    Err(Usage(format!("--faces: {s}: use fixed:VALUE, zero-gradient or inlet-outlet:VALUE")))
}

fn case(args: &[String]) -> Result<(), Usage> {
    let opts = options(
        args,
        &["--vti", "--out", "--array", "--diffusivity", "--velocity", "--faces", "--end-time", "--delta-t", "--write-interval"],
    )?;
    let get = |k: &str| opts.iter().find(|(key, _)| *key == k).map(|(_, v)| *v);
    let (Some(input), Some(out)) = (get("--vti"), get("--out")) else {
        return Err(Usage("case needs --vti and --out".into()));
    };
    let d = ImageData::read_vti(input).map_err(|e| fail(format!("{input}: {e}")))?;
    let array = match get("--array") {
        Some(name) => d.array(name).ok_or_else(|| fail(format!("{input}: no array {name}")))?,
        None => &d.arrays[0],
    };
    let grid = Grid { points: d.points, origin: d.origin, spacing: d.spacing };
    let diffusivity = get("--diffusivity").map(|v| number("--diffusivity", v)).transpose()?.unwrap_or(2.2e-5);
    let mut c = Case::new(grid, array.values.clone(), diffusivity);
    if let Some(v) = get("--velocity") {
        let u: Vec<f64> = v.split(',').map(|s| number("--velocity", s)).collect::<Result<_, _>>()?;
        let [x, y, z] = u[..] else { return Err(Usage("--velocity needs three numbers UX,UY,UZ".into())) };
        c.velocity = Velocity::Uniform([x, y, z]);
    }
    if let Some(v) = get("--faces") {
        c.faces = v.split(',').map(face).collect::<Result<_, _>>()?;
        let want = grid.face_names().len();
        if c.faces.len() != want {
            return Err(Usage(format!("--faces: {} given, this grid has {want} faces ({})", c.faces.len(), grid.face_names().join(","))));
        }
    }
    if let Some(v) = get("--end-time") {
        c.end_time = number("--end-time", v)?;
    }
    if let Some(v) = get("--delta-t") {
        c.delta_t = number("--delta-t", v)?;
    }
    if let Some(v) = get("--write-interval") {
        c.write_interval = number("--write-interval", v)?;
    }
    c.write(out).map_err(fail)?;
    println!(
        "{out}: {} case, {}x{}x{} cells, T from array {} (crc32 {:08x})",
        if grid.is_2d() { "2D" } else { "3D" },
        grid.points[0],
        grid.points[1],
        grid.points[2],
        array.name,
        field_crc32(&c.t)
    );
    Ok(())
}

fn info(dir: &str) -> Result<(), Usage> {
    let c = Case::read(dir).map_err(|e| fail(format!("{dir}: {e}")))?;
    let g = &c.grid;
    println!(
        "{dir}: {} case, {}x{}x{} cells, origin {:?}, spacing {:?}",
        if g.is_2d() { "2D" } else { "3D" },
        g.points[0],
        g.points[1],
        g.points[2],
        g.origin,
        g.spacing
    );
    println!("T: {} values, crc32 {:08x}", c.t.len(), field_crc32(&c.t));
    let faces: Vec<String> = g
        .face_names()
        .iter()
        .zip(&c.faces)
        .map(|(n, f)| match f {
            Face::FixedValue(v) => format!("{n} fixedValue {v:?}"),
            Face::ZeroGradient => format!("{n} zeroGradient"),
            Face::InletOutlet(v) => format!("{n} inletOutlet {v:?}"),
        })
        .collect();
    println!("faces: {}", faces.join(", "));
    match &c.velocity {
        Velocity::Uniform(u) => println!("U: uniform {u:?}"),
        Velocity::Cells(u) => print_components(u),
    }
    println!("DT {:?}, deltaT {:?}, endTime {:?}, writeInterval {:?}", c.diffusivity, c.delta_t, c.end_time, c.write_interval);
    Ok(())
}

fn components(u: &[[f64; 3]]) -> [Vec<f64>; 3] {
    std::array::from_fn(|k| u.iter().map(|v| v[k]).collect())
}

fn print_components(u: &[[f64; 3]]) {
    for (name, values) in ["ux", "uy", "uz"].iter().zip(components(u)) {
        println!("array {name}: {} values, crc32 {:08x}", values.len(), field_crc32(&values));
    }
}

fn velocity(dir: &str, time: &str, rest: &[String]) -> Result<(), Usage> {
    let opts = options(rest, &["--vti"])?;
    let (g, u) = import_velocity(dir, time).map_err(fail)?;
    println!("{dir}/{time}/U: {} vectors on the {}x{}x{} grid", u.len(), g.points[0], g.points[1], g.points[2]);
    print_components(&u);
    if let Some((_, path)) = opts.first() {
        let mut d = ImageData::new_3d(g.points, g.origin, g.spacing);
        let [x, y, z] = components(&u);
        d.add("ux", x).add("uy", y).add("uz", z);
        d.write_vti(path).map_err(|e| fail(format!("{path}: {e}")))?;
    }
    Ok(())
}

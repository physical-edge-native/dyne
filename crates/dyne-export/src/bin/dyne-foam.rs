//! dyne-foam: writes an OpenFOAM case from a Dyne field, reads it back, and imports a velocity field
//! computed by OpenFOAM.
//!
//! ```text
//! dyne-foam case (--vti IN.vti [--array NAME] | [--vti GRID.vti] --sensors READINGS.csv) --out DIR
//!                [--diffusivity DT] [--velocity UX,UY,UZ]
//!                [--faces W,E,S,N[,B,T]] [--end-time S] [--delta-t S] [--write-interval S]
//!                [--shape SHAPE.vti|SHAPE.csv [--shape-array NAME] [--weight-array NAME]
//!                 [--unobserved solid|fluid] [--min-weight W] [--walls FACE] [--regions largest|all]]
//! dyne-foam info DIR
//! dyne-foam velocity DIR TIME [--vti OUT.vti]
//! ```
//!
//! - `case` writes a case for `scalarTransportFoam` (see `dyne_export::foam`) from one array of a
//!   `.vti` file written by Dyne (default: the first array). A face is `fixed:VALUE`, `zero-gradient`
//!   or `inlet-outlet:VALUE`, in the order west, east, south, north (and bottom, top in 3D); the
//!   default is `zero-gradient` everywhere. Defaults: `--diffusivity 2.2e-5` (air, m²/s),
//!   `--velocity 0,0,0`, `--end-time 1`, `--delta-t 0.01`, `--write-interval 0.1`.
//! - `--sensors` (issue #92) builds the initial `T` from a few point readings instead of a Dyne field,
//!   by inverse-distance weighting (`dyne_export::sensors`: file format, method and what the result
//!   means). The grid is that of `--vti` if given, otherwise that of `--shape`. It prints, for every
//!   reading, the nearest grid point and a warning if it lies outside the grid or in a solid point.
//! - `--shape` (issue #91) leaves out the grid points inside walls and obstacles of the room shape
//!   (`dyne_export::walls`): a point is solid where the signed distance `--shape-array` (default `phi`)
//!   is negative, or where the weight `--weight-array` (default `w`) is `<= --min-weight` (default 0:
//!   not observed) unless `--unobserved fluid`. The shape must be on the grid of the field (same point
//!   counts, origin and spacing within 1e-6 m). By default only the largest connected fluid region is
//!   kept (`--regions all` keeps every region). The mesh is then written as `constant/polyMesh` (do not
//!   run `blockMesh`), and the faces towards solid points form the patch `walls` with the condition
//!   `--walls` (default `zero-gradient`). Without `--shape` the case is the same as before.
//! - `info` reads a case written by `case` (every file is checked against the others) and prints the
//!   grid and the CRC-32 of `T` (`dyne_export::field_crc32`, the field hash that Dyne's simulators
//!   print), so the exported field can be compared with the run that produced it. For a case with walls
//!   it also checks the mesh without OpenFOAM (`dyne_export::polymesh::PolyMesh::check`) and prints its
//!   counts.
//! - `velocity` reads `DIR/TIME/U` (ASCII, one vector per cell of the mesh exported by `case`) and
//!   prints the CRC-32 of each component; `--vti` writes the components as arrays `ux`, `uy`, `uz` of
//!   a `.vti` file on Dyne's grid.
//!
//! The exit code is 1 on any error, 2 on a usage error.

use dyne_export::field_crc32;
use dyne_export::foam::{import_velocity, Case, Face, Grid, Velocity};
use dyne_export::polymesh::{keep_largest_region, WALL_PATCH};
use dyne_export::sensors::{idw, nearest_point, read_csv, Reading};
use dyne_export::vti::ImageData;
use dyne_export::walls::{same_grid, Shape, SolidRule, CSV_TOLERANCE};
use std::process::ExitCode;

const USAGE: &str = "usage:\n  dyne-foam case (--vti IN.vti [--array NAME] | [--vti GRID.vti] --sensors READINGS.csv) --out DIR\n                 [--diffusivity DT] [--velocity UX,UY,UZ]\n                 [--faces W,E,S,N[,B,T]] [--end-time S] [--delta-t S] [--write-interval S]\n                 [--shape SHAPE.vti|SHAPE.csv [--shape-array NAME] [--weight-array NAME]\n                  [--unobserved solid|fluid] [--min-weight W] [--walls FACE] [--regions largest|all]]\n  dyne-foam info DIR\n  dyne-foam velocity DIR TIME [--vti OUT.vti]";

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
        &[
            "--vti",
            "--out",
            "--array",
            "--diffusivity",
            "--velocity",
            "--faces",
            "--end-time",
            "--delta-t",
            "--write-interval",
            "--shape",
            "--shape-array",
            "--weight-array",
            "--unobserved",
            "--min-weight",
            "--walls",
            "--regions",
            "--sensors",
        ],
    )?;
    let get = |k: &str| opts.iter().find(|(key, _)| *key == k).map(|(_, v)| *v);
    let Some(out) = get("--out") else { return Err(Usage("case needs --out".into())) };
    let shape = match get("--shape") {
        Some(path) => Some((path, Shape::read(path, get("--shape-array").unwrap_or("phi"), get("--weight-array").unwrap_or("w")).map_err(fail)?)),
        None => None,
    };
    let sensors = match get("--sensors") {
        Some(path) => {
            if get("--array").is_some() {
                return Err(Usage("--array and --sensors exclude each other (T comes from one of them)".into()));
            }
            let text = std::fs::read_to_string(path).map_err(|e| fail(format!("{path}: {e}")))?;
            Some((path, read_csv(&text).map_err(|e| fail(format!("{path}: {e}")))?))
        }
        None => None,
    };
    let (grid, t, source) = match (get("--vti"), &sensors, &shape) {
        (Some(input), _, _) => {
            let d = ImageData::read_vti(input).map_err(|e| fail(format!("{input}: {e}")))?;
            let grid = Grid { points: d.points, origin: d.origin, spacing: d.spacing };
            match &sensors {
                Some((path, rs)) => (grid, idw(&grid, rs), format!("{} readings of {path}", rs.len())),
                None => {
                    let array = match get("--array") {
                        Some(name) => d.array(name).ok_or_else(|| fail(format!("{input}: no array {name}")))?,
                        None => &d.arrays[0],
                    };
                    (grid, array.values.clone(), format!("array {}", array.name))
                }
            }
        }
        (None, Some((path, rs)), Some((_, sh))) => (sh.grid, idw(&sh.grid, rs), format!("{} readings of {path}", rs.len())),
        (None, Some(_), None) => return Err(Usage("--sensors needs a grid: give --vti or --shape".into())),
        (None, None, _) => return Err(Usage("case needs --vti or --sensors".into())),
    };
    let diffusivity = get("--diffusivity").map(|v| number("--diffusivity", v)).transpose()?.unwrap_or(2.2e-5);
    let mut c = Case::new(grid, t, diffusivity);
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
    let walls_note = match &shape {
        None => {
            for k in ["--shape-array", "--weight-array", "--unobserved", "--min-weight", "--walls", "--regions"] {
                if get(k).is_some() {
                    return Err(Usage(format!("{k} needs --shape")));
                }
            }
            None
        }
        Some((path, sh)) => Some(apply_shape(&mut c, path, sh, &get)?),
    };
    c.write(out).map_err(fail)?;
    println!(
        "{out}: {} case, {}x{}x{} cells, T from {source} (crc32 {:08x})",
        if grid.is_2d() { "2D" } else { "3D" },
        grid.points[0],
        grid.points[1],
        grid.points[2],
        field_crc32(&c.cell_values())
    );
    if let Some(note) = walls_note {
        println!("{note}");
        if matches!(c.velocity, Velocity::Uniform(u) if u != [0.0; 3]) {
            // Found with OpenFOAM on 2026-11-18: T grew without bound in such a run.
            println!(
                "WARNING: a uniform velocity flows into the walls (U is zero there), so the flux is not conserved and \
                 scalarTransportFoam can blow up; use --velocity 0,0,0 or a velocity field computed on this mesh"
            );
        }
    }
    if let Some((_, rs)) = &sensors {
        report_sensors(&c, rs);
    }
    Ok(())
}

/// Prints where every reading lies on the grid, and the range of the initial field.
fn report_sensors(c: &Case, rs: &[Reading]) {
    for (i, r) in rs.iter().enumerate() {
        let place = match nearest_point(&c.grid, r.pos) {
            None => "WARNING: outside the grid (still used for the weighting)".to_string(),
            Some(k) if c.solid.as_ref().is_some_and(|s| s[k]) => {
                format!("nearest grid point {k}, WARNING: inside a wall or obstacle (still used for the weighting)")
            }
            Some(k) => format!("nearest grid point {k}"),
        };
        println!("sensor {i}: ({:?}, {:?}, {:?}) T {:?}: {place}", r.pos[0], r.pos[1], r.pos[2], r.value);
    }
    let t = c.cell_values();
    let lo = t.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = t.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    println!("initial T (inverse-distance weighting): {} cells, min {lo:?}, max {hi:?}", t.len());
}

/// Reads the room shape, marks the solid points of `c` and sets the wall condition; returns a line
/// that says what was done.
fn apply_shape<'a>(c: &mut Case, path: &str, shape: &Shape, get: &dyn Fn(&str) -> Option<&'a str>) -> Result<String, Usage> {
    if !same_grid(&shape.grid, &c.grid, CSV_TOLERANCE) {
        return Err(fail(format!(
            "{path}: the shape is on a different grid ({:?} points, origin {:?}, spacing {:?}) than the field ({:?}, {:?}, {:?})",
            shape.grid.points, shape.grid.origin, shape.grid.spacing, c.grid.points, c.grid.origin, c.grid.spacing
        )));
    }
    let unobserved_solid = match get("--unobserved").unwrap_or("solid") {
        "solid" => true,
        "fluid" => false,
        v => return Err(Usage(format!("--unobserved: {v}: use solid or fluid"))),
    };
    let min_weight = get("--min-weight").map(|v| number("--min-weight", v)).transpose()?.unwrap_or(0.0);
    let mut solid = shape.solid(SolidRule { min_weight, unobserved_solid });
    let shape_solid = solid.iter().filter(|&&x| x).count();
    let dropped = match get("--regions").unwrap_or("largest") {
        "largest" => keep_largest_region(&c.grid, &mut solid),
        "all" => 0,
        v => return Err(Usage(format!("--regions: {v}: use largest or all"))),
    };
    if let Some(v) = get("--walls") {
        c.wall = face(v)?;
    }
    c.solid = Some(solid);
    let mesh = c.poly_mesh().map_err(fail)?.expect("walls given");
    let r = mesh.check().map_err(fail)?;
    Ok(format!(
        "walls from {path}: {shape_solid} solid points in the shape, {dropped} fluid points outside the largest region made solid; \
         {} cells, {} faces on patch {WALL_PATCH}, {} region(s)",
        r.cells,
        mesh.patch(WALL_PATCH).map_or(0, |p| p.len),
        r.regions
    ))
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
    let t = c.cell_values();
    println!("T: {} values, crc32 {:08x}", t.len(), field_crc32(&t));
    if let Some(mesh) = c.poly_mesh().map_err(fail)? {
        let r = mesh.check().map_err(fail)?;
        let patches: Vec<String> = mesh.patches.iter().map(|p| format!("{} {}", p.name, p.len)).collect();
        println!(
            "mesh (constant/polyMesh): {} cells, {} faces ({} internal), {} points, {} region(s); patch faces: {}",
            r.cells,
            r.faces,
            r.internal_faces,
            r.points,
            r.regions,
            patches.join(", ")
        );
    }
    let faces: Vec<String> = g
        .face_names()
        .iter()
        .zip(&c.faces)
        .map(|(n, f)| match f {
            Face::FixedValue(v) => format!("{n} fixedValue {v:?}"),
            Face::ZeroGradient => format!("{n} zeroGradient"),
            Face::InletOutlet(v) => format!("{n} inletOutlet {v:?}"),
        })
        .chain(c.solid.as_ref().map(|_| format!("{WALL_PATCH} {}", face_text(&c.wall))))
        .collect();
    println!("faces: {}", faces.join(", "));
    match &c.velocity {
        Velocity::Uniform(u) => println!("U: uniform {u:?}"),
        Velocity::Cells(u) => print_components(u),
    }
    println!("DT {:?}, deltaT {:?}, endTime {:?}, writeInterval {:?}", c.diffusivity, c.delta_t, c.end_time, c.write_interval);
    Ok(())
}

fn face_text(f: &Face) -> String {
    match f {
        Face::FixedValue(v) => format!("fixedValue {v:?}"),
        Face::ZeroGradient => "zeroGradient".into(),
        Face::InletOutlet(v) => format!("inletOutlet {v:?}"),
    }
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

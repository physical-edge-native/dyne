//! dyne-mjcf: writes the room shape computed by Dyne as an MJCF file (MuJoCo's model format) and the
//! free floor where a robot can start as a small JSON file.
//!
//! ```text
//! dyne-mjcf --shape SHAPE.vti|SHAPE.csv --out ROOM.xml [--spawn SPAWN.json]
//!           [--shape-array NAME] [--weight-array NAME] [--unobserved solid|free] [--min-weight W]
//!           [--height M] [--wall-thickness M] [--floor-margin M] [--clearance M] [--friction MU]
//! ```
//!
//! - `--shape` is the same input as `dyne-foam case --shape`: a `.vti` file of `dyne-sim2d` or a CSV
//!   shape file of `scripts/twin_live.py`. A grid point is solid where `--shape-array` (default `phi`)
//!   is negative, or where `--weight-array` (default `w`) is `<= --min-weight` (default 0: not
//!   observed) unless `--unobserved free`.
//! - `--spawn` defaults to `ROOM_spawn.json` next to `ROOM.xml`.
//! - Defaults: `--height 2.4`, `--wall-thickness 0.1`, `--floor-margin 0.5`, `--clearance 0`,
//!   `--friction 0.8`.
//!
//! Frame, units and the layout of both files: `dyne_export::mjcf`. The program prints the counts and
//! the CRC-32 of both files' bytes. The exit code is 1 on any error, 2 on a usage error.

use dyne_export::crc32;
use dyne_export::mjcf::{MjcfOptions, Room};
use dyne_export::walls::{Shape, SolidRule};
use std::process::ExitCode;

const USAGE: &str = "usage:\n  dyne-mjcf --shape SHAPE.vti|SHAPE.csv --out ROOM.xml [--spawn SPAWN.json]\n            [--shape-array NAME] [--weight-array NAME] [--unobserved solid|free] [--min-weight W]\n            [--height M] [--wall-thickness M] [--floor-margin M] [--clearance M] [--friction MU]";

const KEYS: [&str; 12] = [
    "--shape",
    "--out",
    "--spawn",
    "--shape-array",
    "--weight-array",
    "--unobserved",
    "--min-weight",
    "--height",
    "--wall-thickness",
    "--floor-margin",
    "--clearance",
    "--friction",
];

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err((code, msg)) => {
            eprintln!("{msg}");
            if code == 2 {
                eprintln!("{USAGE}");
            }
            ExitCode::from(code)
        }
    }
}

fn run(args: &[String]) -> Result<(), (u8, String)> {
    let usage = |m: String| (2u8, m);
    let fail = |m: String| (1u8, format!("error: {m}"));
    if args.len() % 2 != 0 {
        return Err(usage("options come in pairs: --key value".into()));
    }
    let mut opts: Vec<(&str, &str)> = Vec::new();
    for p in args.chunks(2) {
        if !KEYS.contains(&p[0].as_str()) {
            return Err(usage(format!("unknown option {}", p[0])));
        }
        opts.push((p[0].as_str(), p[1].as_str()));
    }
    let get = |k: &str| opts.iter().rev().find(|(key, _)| *key == k).map(|(_, v)| *v);
    let num = |k: &str, default: f64| -> Result<f64, (u8, String)> {
        match get(k) {
            Some(v) => v.parse::<f64>().map_err(|_| usage(format!("{k}: not a number: {v}"))),
            None => Ok(default),
        }
    };
    let (Some(shape_path), Some(out)) = (get("--shape"), get("--out")) else {
        return Err(usage("dyne-mjcf needs --shape and --out".into()));
    };
    let spawn_path = match get("--spawn") {
        Some(p) => p.to_string(),
        None => format!("{}_spawn.json", out.strip_suffix(".xml").unwrap_or(out)),
    };
    let unobserved_solid = match get("--unobserved").unwrap_or("solid") {
        "solid" => true,
        "free" => false,
        v => return Err(usage(format!("--unobserved: {v}: use solid or free"))),
    };
    let d = MjcfOptions::default();
    let options = MjcfOptions {
        rule: SolidRule { min_weight: num("--min-weight", 0.0)?, unobserved_solid },
        height: num("--height", d.height)?,
        wall_thickness: num("--wall-thickness", d.wall_thickness)?,
        floor_margin: num("--floor-margin", d.floor_margin)?,
        clearance: num("--clearance", d.clearance)?,
        friction: num("--friction", d.friction)?,
    };
    let shape = Shape::read(shape_path, get("--shape-array").unwrap_or("phi"), get("--weight-array").unwrap_or("w")).map_err(|e| fail(e.0))?;
    let room = Room::build(&shape, options).map_err(|e| fail(format!("{shape_path}: {}", e.0)))?;
    let xml = room.to_mjcf();
    let json = room.spawn_json();
    std::fs::write(out, &xml).map_err(|e| fail(format!("{out}: {e}")))?;
    std::fs::write(&spawn_path, &json).map_err(|e| fail(format!("{spawn_path}: {e}")))?;
    let g = room.grid;
    println!(
        "shape: {}x{} points, spacing {} x {} m, origin ({}, {}) m",
        g.points[0], g.points[1], g.spacing[0], g.spacing[1], g.origin[0], g.origin[1]
    );
    println!(
        "{out}: {} geoms (floor, 4 outer boxes, {} solid boxes, {} unobserved boxes), height {} m, crc32 {:08x}",
        room.geom_count(),
        room.solid.len(),
        room.unobserved.len(),
        options.height,
        crc32(xml.bytes())
    );
    println!(
        "{spawn_path}: {} spawn points ({:.3} m2) in {} rectangles, clearance {} m, crc32 {:08x}",
        room.spawn.points,
        room.spawn.points as f64 * g.spacing[0] * g.spacing[1],
        room.spawn.rects.len(),
        options.clearance,
        crc32(json.bytes())
    );
    Ok(())
}

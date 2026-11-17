//! dyne-vti: reads a `.vti` file written by Dyne and prints its grid and the CRC-32 of each array.
//!
//! ```text
//! dyne-vti info PATH
//! ```
//!
//! Output:
//!
//! ```text
//! PATH: 41x41x1 points, origin [0.0, 0.0, 0.0], spacing [0.025, 0.025, 0.025]
//! array u: 1681 values, crc32 18e44331
//! ```
//!
//! The CRC-32 is the field hash that Dyne's simulators and native solvers print
//! (`dyne_export::field_crc32`), so the printed value can be compared with the run that wrote the
//! file: equal values mean the field was written and read back bit for bit. The exit code is 1 if the
//! file cannot be read (it must have exactly the layout that `dyne_export::vti` writes).

use dyne_export::field_crc32;
use dyne_export::vti::ImageData;
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [cmd, path] = args.as_slice() else {
        eprintln!("usage: dyne-vti info PATH");
        return ExitCode::from(2);
    };
    if cmd != "info" {
        eprintln!("usage: dyne-vti info PATH");
        return ExitCode::from(2);
    }
    match ImageData::read_vti(path) {
        Ok(d) => {
            println!(
                "{path}: {}x{}x{} points, origin {:?}, spacing {:?}",
                d.points[0], d.points[1], d.points[2], d.origin, d.spacing
            );
            for a in &d.arrays {
                println!("array {}: {} values, crc32 {:08x}", a.name, a.values.len(), field_crc32(&a.values));
            }
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("error: {path}: {e}");
            ExitCode::FAILURE
        }
    }
}

//! Import of the synthetic OpenFOAM velocity field in `tests/fixtures/openfoam-velocity` (hand-written
//! in the layout that OpenFOAM writes; CI does not run OpenFOAM). The imported values must be exactly
//! the numbers in the file, in the order of Dyne's grid.

use dyne_export::foam::{import_velocity, Grid};
use std::path::PathBuf;

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/openfoam-velocity")
}

#[test]
fn imports_fixture_exactly() {
    let (g, u) = import_velocity(fixture(), "0").unwrap();
    assert_eq!(g, Grid::new_3d([3, 2, 2], [0.05; 3], [0.1; 3]));
    // The 17-digit decimals that OpenFOAM writes (writePrecision 17) read back to the shortest forms.
    let want: [[f64; 3]; 12] = [
        [0.1, 0.0, 0.0],
        [0.2, 0.01, 0.0],
        [0.3, -0.01, 0.0],
        [0.1, 0.0, 1e-5],
        [0.2, 0.05, -1e-5],
        [0.3, -0.0, 2.5e-17],
        [-0.025, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0 / 3.0, 2.0 / 3.0, -1.0 / 3.0],
        [1.5, -2.25, 0.125],
        [1e-300, -1e20, f64::from_bits(1)],
        [0.12345678901234568, -0.9876543210987654, 0.5],
    ];
    assert_eq!(u.len(), want.len());
    for (k, (a, b)) in u.iter().zip(&want).enumerate() {
        for c in 0..3 {
            assert_eq!(a[c].to_bits(), b[c].to_bits(), "cell {k} component {c}: {} vs {}", a[c], b[c]);
        }
    }
}

#[test]
fn wrong_time_or_grid_is_an_error() {
    assert!(import_velocity(fixture(), "1").is_err());
    // A grid with a different number of points than vectors in U is rejected.
    let d = std::env::temp_dir().join(format!("dyne-foam-fixture-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(d.join("system")).unwrap();
    std::fs::create_dir_all(d.join("0")).unwrap();
    let grid = std::fs::read_to_string(fixture().join("system/dyneGrid")).unwrap();
    std::fs::write(d.join("system/dyneGrid"), grid.replace("(3 2 2)", "(3 2 3)")).unwrap();
    std::fs::copy(fixture().join("0/U"), d.join("0/U")).unwrap();
    assert!(import_velocity(&d, "0").unwrap_err().0.contains("12 vectors"));
    std::fs::remove_dir_all(&d).unwrap();
}

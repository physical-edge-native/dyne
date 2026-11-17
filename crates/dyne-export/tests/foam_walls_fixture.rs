//! A small case with walls, compared byte for byte with `tests/fixtures/foam-walls-4x3` (issue #91).
//!
//! The fixture is a 4x3 grid (2D) with one solid point in the middle, so the patch `walls` has four
//! faces. It was written by this code and then checked with OpenFOAM (ESI openfoam2606: `checkMesh`
//! reports "Mesh OK.", `scalarTransportFoam` runs); CI compares the files without OpenFOAM, so any
//! change of the written mesh or fields shows up here. To write the fixture again after an intended
//! change, run the test with `DYNE_BLESS=1` and check the new files with OpenFOAM before committing.

use dyne_export::foam::{Case, Face, Grid};
use std::path::PathBuf;

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/foam-walls-4x3")
}

fn case() -> Case {
    let g = Grid::new_2d([4, 3], [0.05, 0.05], [0.1, 0.1]);
    let mut c = Case::new(g, (0..12).map(|k| 20.0 + 0.5 * k as f64).collect(), 2.2e-5);
    let mut solid = vec![false; 12];
    solid[5] = true; // grid point (1, 1)
    c.solid = Some(solid);
    c.wall = Face::FixedValue(18.5);
    c.faces = vec![Face::ZeroGradient, Face::FixedValue(25.0), Face::ZeroGradient, Face::ZeroGradient];
    c.end_time = 10.0;
    c.delta_t = 0.5;
    c.write_interval = 5.0;
    c
}

#[test]
fn walls_case_matches_fixture() {
    let c = case();
    if std::env::var_os("DYNE_BLESS").is_some() {
        c.write(fixture()).unwrap();
    }
    for (rel, text) in c.files().unwrap() {
        let want = std::fs::read_to_string(fixture().join(rel)).unwrap_or_else(|e| panic!("{rel}: {e}"));
        assert_eq!(text, want.replace("\r\n", "\n"), "{rel} differs from the fixture");
    }
    // The fixture reads back: 11 cells, T at the solid point is NaN, the rest exactly as written.
    let r = Case::read(fixture()).unwrap();
    assert_eq!(r.cell_values(), c.cell_values());
    assert!(r.t[5].is_nan());
    assert_eq!(r.wall, Face::FixedValue(18.5));
    let m = r.poly_mesh().unwrap().unwrap();
    let rep = m.check().unwrap();
    assert_eq!((rep.cells, rep.regions), (11, 1));
    assert_eq!(m.patch("walls").unwrap().len, 4);
    assert!(!fixture().join("system/blockMeshDict").exists());
}

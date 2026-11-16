//! The wave solvers converge to analytic standing-wave solutions at second order: every checked
//! observed order lies in `convergence::ORDER_BAND` (the band and its reasons are documented there).

use dyne_native::convergence::{order_in_band, study_2d, study_3d, Study, ORDER_BAND};

fn check(name: &str, s: &Study) {
    println!("{name}\n{}", s.table());
    for (what, p) in s.checked_orders() {
        assert!(order_in_band(p), "{name}: {what}: observed order {p} outside {ORDER_BAND:?}");
    }
    assert!(s.passes());
    // The errors fall at every refinement (also on the levels that are reported only).
    for w in s.levels.windows(2) {
        assert!(w[1].max_error < w[0].max_error && w[1].l2_error < w[0].l2_error, "{name}: error does not fall");
    }
}

#[test]
fn wave2d_is_second_order() {
    check("2D", &study_2d().unwrap());
}

#[test]
fn wave3d_is_second_order() {
    check("3D", &study_3d().unwrap());
}

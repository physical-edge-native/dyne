//! The 3D wave equation split into patches gives the same bits as the single-patch solve, and a 3D
//! problem that does not vary along z gives the same bits as the 2D solver in every z slice.

use dyne_native::decomp::Decomposition;
use dyne_native::field_crc32;
use dyne_native::patch::{run_single, run_threaded, PatchSolver};
use dyne_native::wave2d;
use dyne_native::wave3d::{Initial, Outer, Wave3dPatch, Wave3dProblem, MIN_POINTS};

const GAUSS: Initial = Initial::Gauss { x0: 0.3, y0: 0.6, z0: 0.45, sigma: 0.1, amp: 1.0 };

/// h = 1/(nx − 1), c = 1, dt = courant · h.
fn problem(points: [usize; 3], courant: f64, initial: Initial, outer: [Outer; 6]) -> Wave3dProblem {
    let h = 1.0 / ((points[0] - 1) as f64);
    Wave3dProblem { points, h, c: 1.0, dt: courant * h, initial, outer }
}

fn single(p: &Wave3dProblem, steps: u64) -> Vec<f64> {
    let d = Decomposition::whole(p.points, MIN_POINTS).unwrap();
    let mut s = Wave3dPatch::new(p, &d.blocks[0]).unwrap();
    run_single(&mut s, steps).unwrap()
}

fn split(p: &Wave3dProblem, parts: [usize; 3], steps: u64) -> Vec<f64> {
    let d = Decomposition::new(p.points, parts, MIN_POINTS).unwrap();
    run_threaded(&d, steps, |b| Wave3dPatch::new(p, b)).unwrap().field
}

fn assert_bits_equal(a: &[f64], b: &[f64], what: &str) {
    assert_eq!(a.len(), b.len(), "{what}: length");
    for (k, (x, y)) in a.iter().zip(b).enumerate() {
        assert_eq!(x.to_bits(), y.to_bits(), "{what}: point {k}: {x:e} vs {y:e}");
    }
}

const FIX0: [Outer; 6] = [Outer::Fixed(0.0); 6];
const ZG: [Outer; 6] = [Outer::ZeroGradient; 6];
const ABS: [Outer; 6] = [Outer::Absorbing; 6];
/// Every kind of face on one box: x low absorbing, x high fixed at 0.5, y low fixed at 0.5,
/// y high absorbing, z low zero gradient, z high absorbing.
const MIXED: [Outer; 6] = [
    Outer::Absorbing,
    Outer::Fixed(0.5),
    Outer::Fixed(0.5),
    Outer::Absorbing,
    Outer::ZeroGradient,
    Outer::Absorbing,
];

/// Split solutions match the single-patch solution bit for bit for even and uneven splits and every
/// kind of outer boundary, including a run beyond the stability limit.
#[test]
fn split_matches_single_patch_bit_for_bit() {
    let points = [25, 23, 21];
    let cases = [
        ("fixed", problem(points, 0.5, GAUSS, FIX0)),
        ("zero gradient", problem(points, 0.5, GAUSS, ZG)),
        ("absorbing", problem(points, 0.5, GAUSS, ABS)),
        ("mixed", problem(points, 0.5, GAUSS, MIXED)),
        // Courant number 0.6, above 1/√3 ≈ 0.577: the run diverges, identically in every split.
        ("unstable", problem(points, 0.6, GAUSS, FIX0)),
    ];
    let parts = [[1, 1, 1], [2, 2, 2], [3, 2, 1], [4, 4, 2], [1, 3, 5], [5, 1, 2], [1, 1, 7]];
    for (name, p) in cases {
        let reference = single(&p, 80);
        assert!(reference.iter().all(|v| v.is_finite()) || name == "unstable", "{name}: not finite");
        for pp in parts {
            let t = split(&p, pp, 80);
            assert_bits_equal(&reference, &t, &format!("{name} {}x{}x{}", pp[0], pp[1], pp[2]));
        }
    }
}

/// The unstable case really diverges (so the bit-exactness above covers growing values too).
#[test]
fn courant_above_the_limit_diverges() {
    let p = problem([25, 23, 21], 0.6, GAUSS, FIX0);
    let f = single(&p, 300);
    let max = f.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    assert!(!(max < 1e3), "max |u| = {max:e}");
}

/// The single-patch field hash of the check run by `scripts/ci-check.sh`, pinned so that the same
/// bits are required on every OS.
#[test]
fn field_hash_is_pinned() {
    let p = problem([25, 23, 21], 0.5, GAUSS, FIX0);
    let f = single(&p, 100);
    assert_eq!(field_crc32(&f), CI_CRC, "crc {:08x}", field_crc32(&f));
}

/// CRC-32 of the field of the 25×23×21 Gaussian bump, fixed value 0, Courant number 0.5, 100 steps.
const CI_CRC: u32 = 0x2ca2_6bb5;

/// A 3D problem that does not vary along z (initial cos(lπz) with l = 0, zero-gradient z faces)
/// gives, in every z slice, the same bits as the 2D solver: the x and y terms of the seven-point
/// Laplacian are computed exactly as in the 2D solver, and the z term is exactly zero.
#[test]
fn z_independent_problem_reproduces_the_2d_solver() {
    let (nx, ny, nz) = (31, 27, 6);
    let h = 1.0 / ((nx - 1) as f64);
    let dt = 0.5 * h;
    let outers2: [[wave2d::Outer; 4]; 3] = [
        [wave2d::Outer::Fixed(0.0); 4],
        [wave2d::Outer::ZeroGradient; 4],
        [wave2d::Outer::Absorbing, wave2d::Outer::Fixed(0.5), wave2d::Outer::Fixed(0.5), wave2d::Outer::Absorbing],
    ];
    for o in outers2 {
        let p2 = wave2d::Wave2dProblem {
            points: [nx, ny],
            h,
            c: 1.0,
            dt,
            initial: wave2d::Initial::CosMode { m: 1.0, n: 2.0, amp: 1.0 },
            outer: o,
        };
        let p3 = Wave3dProblem {
            points: [nx, ny, nz],
            h,
            c: 1.0,
            dt,
            initial: Initial::CosMode { m: 1.0, n: 2.0, l: 0.0, amp: 1.0 },
            outer: [o[0], o[1], o[2], o[3], Outer::ZeroGradient, Outer::ZeroGradient],
        };
        let d2 = Decomposition::whole(p2.points, wave2d::MIN_POINTS).unwrap();
        let f2 = run_single(&mut wave2d::Wave2dPatch::new(&p2, &d2.blocks[0]).unwrap(), 120).unwrap();
        let f3 = split(&p3, [2, 2, 2], 120);
        for k in 0..nz {
            assert_bits_equal(&f2, &f3[k * nx * ny..(k + 1) * nx * ny], &format!("{o:?}, slice z = {k}"));
        }
    }
}

/// The border messages are Dyne's IFACE_BAND messages: the interface id as sender, the step number,
/// and the 2D slab of the face as one list of values (one f64 per point).
#[test]
fn border_messages_carry_face_slabs() {
    use dyne_native::wire;
    let p = problem([25, 23, 21], 0.5, GAUSS, FIX0);
    let d = Decomposition::new(p.points, [2, 2, 2], MIN_POINTS).unwrap();
    let b = &d.blocks[0]; // low corner block: faces 1 (x high), 3 (y high), 5 (z high) are interfaces
    let mut s = Wave3dPatch::new(&p, b).unwrap();
    let mut out = Vec::new();
    s.outbox(&mut out).unwrap();
    let msgs: Vec<_> = wire::MailboxReader::new(&out).map(|m| m.unwrap()).collect();
    assert_eq!(msgs.len(), 3);
    let [nx, ny, nz] = b.size;
    for ((hdr, payload, _), (face, count)) in msgs.iter().zip([(1usize, ny * nz), (3, nx * nz), (5, nx * ny)]) {
        assert_eq!(hdr.kind, wire::KIND_IFACE_BAND);
        assert_eq!(hdr.quantity, wire::QTY_DISPLACEMENT);
        assert_eq!(hdr.src, b.faces[face].unwrap().id);
        assert_eq!(hdr.step, 0);
        let wire::Payload::IfaceBand { count: c, per_point, .. } = payload else { panic!("band expected") };
        assert_eq!((*c as usize, *per_point), (count, 1));
    }
}

/// A patch that is not given its neighbours' border values refuses to step.
#[test]
fn missing_border_values_are_an_error() {
    let p = problem([25, 23, 21], 0.5, GAUSS, FIX0);
    let d = Decomposition::new(p.points, [1, 1, 2], MIN_POINTS).unwrap();
    let mut s = Wave3dPatch::new(&p, &d.blocks[0]).unwrap();
    assert!(s.step().is_err());
}

/// Splits with wider borders (k layers exchanged once every k steps, with the face, edge and corner
/// neighbours) match the single-patch solution bit for bit, for every kind of outer boundary and
/// even and uneven splits; a width larger than a patch along a split axis is rejected.
#[test]
fn wide_borders_match_single_patch_bit_for_bit() {
    let points = [25, 23, 21];
    let cases = [
        ("fixed", problem(points, 0.5, GAUSS, FIX0)),
        ("zero gradient", problem(points, 0.5, GAUSS, ZG)),
        ("absorbing", problem(points, 0.5, GAUSS, ABS)),
        ("mixed", problem(points, 0.5, GAUSS, MIXED)),
        ("unstable", problem(points, 0.6, GAUSS, FIX0)),
    ];
    // [1, 1, 7] leaves 3 points per patch along z: rejected for k = 4.
    let parts = [[2, 2, 2], [3, 2, 1], [4, 4, 2], [1, 3, 5], [5, 1, 2], [1, 1, 7]];
    for (name, p) in cases {
        let reference = single(&p, 47);
        for k in [2, 3, 4] {
            for pp in parts {
                let d = Decomposition::new(p.points, pp, MIN_POINTS).unwrap();
                let too_wide = (0..3).any(|a| pp[a] > 1 && d.blocks.iter().any(|b| b.size[a] < k));
                let r = run_threaded(&d, 47, |b| Wave3dPatch::with_halo(&p, b, k));
                let what = format!("{name} {}x{}x{} halo {k}", pp[0], pp[1], pp[2]);
                if too_wide {
                    let e = r.err().unwrap_or_else(|| panic!("{what}: should be rejected"));
                    assert!(e.0.contains(&format!("halo width {k}")), "{what}: {e}");
                } else {
                    assert_bits_equal(&reference, &r.unwrap().field, &what);
                }
            }
        }
    }
}

/// With k = 2, the low corner block of a 2×2×2 split sends 7 messages at step 0: 3 faces, 3 edges
/// and 1 corner, each with the current level on k layers and the previous level on k − 1 layers.
#[test]
fn wide_border_messages_reach_edges_and_corners() {
    use dyne_native::wire;
    let p = problem([25, 23, 21], 0.5, GAUSS, FIX0);
    let d = Decomposition::new(p.points, [2, 2, 2], MIN_POINTS).unwrap();
    let b = &d.blocks[0];
    let mut s = Wave3dPatch::with_halo(&p, b, 2).unwrap();
    let mut out = Vec::new();
    s.outbox(&mut out).unwrap();
    let counts: Vec<usize> = wire::MailboxReader::new(&out)
        .map(|m| match m.unwrap().1 {
            wire::Payload::IfaceBand { count, .. } => count as usize,
            _ => panic!("band expected"),
        })
        .collect();
    let [nx, ny, nz] = b.size;
    // faces x, y, z; then the edges (x,y), (x,z), (y,z) and the corner, in base-3 offset order
    let expect = vec![3 * ny * nz, 3 * nx * nz, 3 * nx * ny, 5 * nz, 5 * ny, 5 * nx, 8 + 1];
    assert_eq!(counts, expect);
}

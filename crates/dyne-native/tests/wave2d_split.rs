//! The 2D wave equation split into patches gives the same bits as the single-patch solve, and the
//! native solver gives the same bits as Dyne's WebAssembly solver (through the field hashes that
//! Dyne recorded for its own regression scenarios).

use dyne_native::decomp::Decomposition;
use dyne_native::field_crc32;
use dyne_native::patch::{run_single, run_threaded, PatchSolver};
use dyne_native::wave2d::{Initial, Outer, Wave2dPatch, Wave2dProblem, MIN_POINTS};

/// The Gaussian bump of Dyne's wave scenarios: centre (0.3, 0.6), σ = 0.06.
const GAUSS: Initial = Initial::Gauss { x0: 0.3, y0: 0.6, sigma: 0.06, amp: 1.0 };
const SIN11: Initial = Initial::SinMode { m: 1.0, n: 1.0, amp: 1.0 };
const COS11: Initial = Initial::CosMode { m: 1.0, n: 1.0, amp: 1.0 };

/// A problem as Dyne's dyne-sim2d sets it up: h = 1/(nx − 1), c = 1.
fn problem(nx: usize, ny: usize, dt: f64, initial: Initial, outer: [Outer; 4]) -> Wave2dProblem {
    Wave2dProblem { points: [nx, ny], h: 1.0 / ((nx - 1) as f64), c: 1.0, dt, initial, outer }
}

fn single(p: &Wave2dProblem, steps: u64) -> Vec<f64> {
    let d = Decomposition::whole(p.points, MIN_POINTS).unwrap();
    let mut s = Wave2dPatch::new(p, &d.blocks[0]).unwrap();
    run_single(&mut s, steps).unwrap()
}

fn split(p: &Wave2dProblem, parts: [usize; 2], steps: u64) -> Vec<f64> {
    let d = Decomposition::new(p.points, parts, MIN_POINTS).unwrap();
    run_threaded(&d, steps, |b| Wave2dPatch::new(p, b)).unwrap().field
}

fn assert_bits_equal(a: &[f64], b: &[f64], what: &str) {
    assert_eq!(a.len(), b.len(), "{what}: length");
    for (k, (x, y)) in a.iter().zip(b).enumerate() {
        assert_eq!(x.to_bits(), y.to_bits(), "{what}: point {k}: {x:e} vs {y:e}");
    }
}

const FIX0: [Outer; 4] = [Outer::Fixed(0.0); 4];
const ZG: [Outer; 4] = [Outer::ZeroGradient; 4];
const ABS: [Outer; 4] = [Outer::Absorbing; 4];
/// West and north absorbing, east and south fixed at 0.5.
const MIXED: [Outer; 4] = [Outer::Absorbing, Outer::Fixed(0.5), Outer::Fixed(0.5), Outer::Absorbing];

/// Dyne's 2D wave regression scenarios (Dyne `scripts/check-wave2d.ps1`, results in Dyne
/// `docs/2026-11-11-wave2d.md` §3): problem, Dyne's partition, steps, and the CRC-32 of the global
/// field that Dyne's WebAssembly solver produced (split and single-node runs, which agreed).
/// The network faults of those scenarios (loss, delay, pauses) do not change the result.
fn dyne_scenarios() -> Vec<(&'static str, Wave2dProblem, [usize; 2], u64, u32)> {
    vec![
        ("dir_2x2", problem(41, 41, 0.0125, SIN11, FIX0), [2, 2], 400, 0x064e7eb0),
        ("gauss_pause", problem(41, 41, 0.0125, GAUSS, FIX0), [2, 2], 400, 0x18e44331),
        ("3x2_uneven", problem(40, 33, 0.01282051282051282, GAUSS, FIX0), [3, 2], 300, 0x3accd6ec),
        ("1x4_strip", problem(41, 41, 0.0125, GAUSS, FIX0), [1, 4], 300, 0x767c9cd4),
        ("4x4_pause", problem(61, 61, 0.008333333333333333, GAUSS, FIX0), [4, 4], 300, 0x42db3fda),
        ("zero_grad_cos", problem(41, 41, 0.0125, COS11, ZG), [2, 2], 400, 0xe7d9801a),
        ("zero_grad_gs", problem(41, 41, 0.0125, GAUSS, ZG), [3, 3], 400, 0xefa74d42),
        ("absorb_3x3", problem(45, 31, 0.011363636363636364, GAUSS, ABS), [3, 3], 300, 0xdbe2fcb8),
        ("mixed_dir05_WN", problem(41, 41, 0.0125, GAUSS, MIXED), [2, 3], 300, 0x356ed9e9),
        ("unstable_072", problem(41, 41, 0.018, GAUSS, FIX0), [2, 2], 400, 0x2fccdf00),
    ]
}

/// The native solver reproduces the field hashes of Dyne's WebAssembly solver, both as one patch
/// and split as in Dyne's scenario (including an uneven split and a run beyond the stability limit).
#[test]
fn native_solver_reproduces_dyne_field_hashes() {
    for (name, p, parts, steps, crc) in dyne_scenarios() {
        let s = single(&p, steps);
        assert_eq!(field_crc32(&s), crc, "{name}: single patch, crc {:08x}", field_crc32(&s));
        let t = split(&p, parts, steps);
        assert_bits_equal(&s, &t, name);
    }
}

/// Split solutions match the single-patch solution bit for bit for 1×1, 2×2, 4×4, 8×8 and uneven
/// splits, for every kind of outer boundary.
#[test]
fn split_matches_single_patch_bit_for_bit() {
    let cases = [
        ("fixed", problem(67, 59, 0.5 / 66.0, GAUSS, FIX0)),
        ("zero gradient", problem(67, 59, 0.5 / 66.0, GAUSS, ZG)),
        ("absorbing", problem(67, 59, 0.5 / 66.0, GAUSS, ABS)),
        ("mixed", problem(67, 59, 0.5 / 66.0, GAUSS, MIXED)),
        // Courant number 0.72, above 1/√2: the run diverges, identically in every split.
        ("unstable", problem(67, 59, 0.72 / 66.0, GAUSS, FIX0)),
    ];
    let parts = [[1, 1], [2, 2], [4, 4], [8, 8], [3, 5], [7, 2], [1, 6], [6, 1]];
    for (name, p) in cases {
        let reference = single(&p, 250);
        for pp in parts {
            let t = split(&p, pp, 250);
            assert_bits_equal(&reference, &t, &format!("{name} {}x{}", pp[0], pp[1]));
        }
    }
}

/// The border messages are Dyne's: wire format v2, kind IFACE_BAND, quantity displacement, the
/// interface id as sender, the step number, one f64 per point of the face.
#[test]
fn border_messages_use_dyne_wire_format() {
    use dyne_native::wire;
    let p = problem(41, 41, 0.0125, GAUSS, FIX0);
    let d = Decomposition::new(p.points, [2, 2], MIN_POINTS).unwrap();
    let b = &d.blocks[0]; // south-west block: east and north faces are interfaces
    let mut s = Wave2dPatch::new(&p, b).unwrap();
    let mut out = Vec::new();
    s.outbox(&mut out).unwrap();
    let msgs: Vec<_> = wire::MailboxReader::new(&out).map(|m| m.unwrap()).collect();
    assert_eq!(msgs.len(), 2);
    for ((hdr, payload, _), face) in msgs.iter().zip([1usize, 3]) {
        assert_eq!(hdr.kind, wire::KIND_IFACE_BAND);
        assert_eq!(hdr.quantity, wire::QTY_DISPLACEMENT);
        assert_eq!(hdr.src, b.faces[face].unwrap().id);
        assert_eq!(hdr.step, 0);
        let wire::Payload::IfaceBand { count, per_point, .. } = payload else { panic!("band expected") };
        assert_eq!((*count as usize, *per_point), (if face < 2 { b.size[1] } else { b.size[0] }, 1));
    }
}

/// A patch that is not given its neighbours' border values refuses to step, and a failing patch
/// stops a threaded run with an error instead of hanging it.
#[test]
fn missing_border_values_are_an_error() {
    let p = problem(41, 41, 0.0125, GAUSS, FIX0);
    let d = Decomposition::new(p.points, [2, 1], MIN_POINTS).unwrap();
    let mut s = Wave2dPatch::new(&p, &d.blocks[0]).unwrap();
    assert!(s.step().is_err());

    let r = run_threaded(&d, 10, |b| {
        if b.index == 1 {
            Err(dyne_native::patch::PatchError("refused on purpose".into()))
        } else {
            Wave2dPatch::new(&p, b)
        }
    });
    assert!(r.unwrap_err().0.contains("refused on purpose"));
}

/// Splits with wider borders (k layers, one exchange every k steps, also with diagonal neighbours)
/// match the single-patch solution bit for bit, for every kind of outer boundary, even and uneven
/// splits, and a step count that is not a multiple of k. A width larger than a patch along a split
/// axis is rejected with an error that names the halo width.
#[test]
fn wide_borders_match_single_patch_bit_for_bit() {
    let cases = [
        ("fixed", problem(67, 59, 0.5 / 66.0, GAUSS, FIX0)),
        ("zero gradient", problem(67, 59, 0.5 / 66.0, GAUSS, ZG)),
        ("absorbing", problem(67, 59, 0.5 / 66.0, GAUSS, ABS)),
        ("mixed", problem(67, 59, 0.5 / 66.0, GAUSS, MIXED)),
        ("unstable", problem(67, 59, 0.72 / 66.0, GAUSS, FIX0)),
    ];
    // 16x16 leaves 4 or 5 points per patch along x and 3 or 4 along y: rejected for k = 4.
    let parts = [[1, 1], [2, 2], [4, 4], [8, 8], [3, 5], [7, 2], [1, 6], [6, 1], [16, 16]];
    for (name, p) in cases {
        let reference = single(&p, 211);
        for k in [2, 3, 4] {
            for pp in parts {
                let d = Decomposition::new(p.points, pp, MIN_POINTS).unwrap();
                let too_wide = (0..2).any(|a| pp[a] > 1 && d.blocks.iter().any(|b| b.size[a] < k));
                let r = run_threaded(&d, 211, |b| Wave2dPatch::with_halo(&p, b, k));
                let what = format!("{name} {}x{} halo {k}", pp[0], pp[1]);
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

/// With k = 3, the south-west block of a 2×2 split sends three messages at step 0: east face
/// (3 layers of the current level and 2 of the previous one, per row), north face, and the
/// north-east corner (3×3 + 2×2 values), with the corner's own link id. Between exchanges there is
/// nothing to send.
#[test]
fn wide_border_messages_include_the_corner() {
    use dyne_native::wire;
    let p = problem(41, 41, 0.0125, GAUSS, FIX0);
    let d = Decomposition::new(p.points, [2, 2], MIN_POINTS).unwrap();
    let b = &d.blocks[0];
    let mut s = Wave2dPatch::with_halo(&p, b, 3).unwrap();
    assert_eq!(s.halo_width(), 3);
    let mut out = Vec::new();
    s.outbox(&mut out).unwrap();
    let msgs: Vec<_> = wire::MailboxReader::new(&out).map(|m| m.unwrap()).collect();
    let [nx, ny] = b.size;
    assert_eq!(b.diagonals.len(), 1);
    assert_eq!(b.diagonals[0].offset, [1, 1]);
    let expect = [(b.faces[1].unwrap().id, 5 * ny), (b.faces[3].unwrap().id, 5 * nx), (b.diagonals[0].id, 9 + 4)];
    assert_eq!(msgs.len(), expect.len());
    for ((hdr, payload, _), (id, count)) in msgs.iter().zip(expect) {
        assert_eq!((hdr.kind, hdr.src, hdr.step), (wire::KIND_IFACE_BAND, id, 0));
        let wire::Payload::IfaceBand { count: c, per_point, .. } = payload else { panic!("band expected") };
        assert_eq!((*c as usize, *per_point), (count, 1));
    }
    let whole = Decomposition::whole(p.points, MIN_POINTS).unwrap();
    let mut alone = Wave2dPatch::with_halo(&p, &whole.blocks[0], 3).unwrap();
    alone.step().unwrap();
    let mut out = Vec::new();
    assert!(alone.outbox(&mut out).is_err() && out.is_empty());
}

/// Patches with different border widths cannot run together.
#[test]
fn mixed_border_widths_are_an_error() {
    let p = problem(41, 41, 0.0125, GAUSS, FIX0);
    let d = Decomposition::new(p.points, [2, 2], MIN_POINTS).unwrap();
    let r = run_threaded(&d, 10, |b| Wave2dPatch::with_halo(&p, b, if b.index == 3 { 2 } else { 1 }));
    assert!(r.unwrap_err().0.contains("differs from the width"));
}

//! Bit-level regression of the wire format and the deterministic random numbers.
//!
//! The acceptance criterion for numerical results is bit-for-bit equality, so the border-exchange
//! format (`dyne-wire`) and the deterministic random numbers (`dyne-core`) that the native solvers
//! use must not change silently. These tests fail if either changes.

use dyne_native::sim::rng::Rng;
use dyne_native::wire::{
    band_get, Header, MailboxReader, MailboxWriter, Payload, HDR_LEN, KIND_IFACE_BAND,
    QTY_DISPLACEMENT,
};

/// A band of border values (the values a patch sends to its neighbour) survives a trip through a
/// mailbox bit for bit, including values whose bits are easy to lose (-0.0, a NaN with a payload,
/// a subnormal number).
#[test]
fn border_band_round_trips_bit_for_bit() {
    let values: [f64; 5] = [
        1.5,
        -0.0,
        f64::from_bits(0x7FF8_0000_0000_1234), // NaN with a payload
        f64::from_bits(1),                     // smallest subnormal
        -123.456e-300,
    ];
    let mut data = Vec::new();
    for v in &values {
        data.extend_from_slice(&v.to_le_bytes());
    }
    let hdr = Header { kind: KIND_IFACE_BAND, quantity: QTY_DISPLACEMENT, src: 7, step: 42, t_us: 1_000 };
    let payload = Payload::IfaceBand { data: &data, count: values.len() as u32, per_point: 1 };

    let mut buf = [0u8; 256];
    let mut w = MailboxWriter::new(&mut buf);
    w.push(&hdr, &payload).expect("message fits");
    let used = w.finish();
    // [u32 len] + 24-byte header + count u32 + per_point u32 + 5 values
    assert_eq!(used, 4 + HDR_LEN + 8 + 8 * values.len());

    let mut r = MailboxReader::new(&buf);
    let (h, p, _raw) = r.next().expect("one message").expect("decodes");
    assert_eq!(h, hdr);
    match p {
        Payload::IfaceBand { data, count, per_point } => {
            assert_eq!((count, per_point), (values.len() as u32, 1));
            for (i, v) in values.iter().enumerate() {
                assert_eq!(band_get(data, i).to_bits(), v.to_bits(), "value {i}");
            }
        }
        other => panic!("unexpected payload {other:?}"),
    }
    assert!(r.next().is_none(), "only one message");
}

/// Dyne's deterministic random numbers give the same sequence on every platform
/// (fixed vector shared with Dyne's own test).
#[test]
fn deterministic_rng_fixed_vector() {
    let mut r = Rng::new(42);
    assert_eq!(r.next_u64(), 6255019084209693600);
    assert_eq!(r.next_u64(), 14430073426741505498);
    assert_eq!(Rng::new(42).uniform().to_bits(), 0x3FD5B392ADDC66E8);
}

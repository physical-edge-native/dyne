//! The outer envelope for carrying border messages over a real radio link (UDP).
//!
//! Wire format v2 messages (24-byte header and payload) have neither a sequence number nor
//! an error-detecting code. Messages cannot be corrupted in the simulator's virtual
//! network, but they can on a radio link, so each UDP datagram wraps one delivery unit in
//! the envelope below. A delivery unit is the framed sequence of v2 messages
//! `[u32 len][msg]...` for one border and one step (without the terminating `len = 0`).
//! Hosts and firmware do not convert the contents: the bytes from the sender's outbox go
//! into the envelope as they are, and the receiver hands them to its inbox as they are.
//!
//! ## Envelope layout (little-endian)
//!
//! ```text
//! offset size field
//! 0      4    magic        = "DYN2" (bytes 0x44 0x59 0x4E 0x32)
//! 4      1    env_version  = 1
//! 5      1    flags        = 0 (reserved; anything else is invalid)
//! 6      2    reserved     = 0
//! 8      4    sender       u32 sending device id
//! 12     4    seq          u32 sequence number (sender adds 1 per step; a resend keeps it)
//! 16     4    iface        u32 border id (must equal the src of the messages inside)
//! 20     4    payload_len  u32 number of payload bytes N
//! 24     8    step         u64 step (must equal the step of the messages inside)
//! 32     N    payload      framed v2 messages (one or more)
//! 32+N   4    crc32        IEEE CRC-32 over bytes 0 .. 32+N−1
//! ```
//!
//! A datagram is at most 1472 bytes (Ethernet's 1500 minus 20 for IPv4 and 8 for UDP, so
//! it is not fragmented); the payload is therefore at most 1472 − 36 = 1436 bytes. A unit
//! that does not fit is **not sent but returned as an error** (splitting is not supported;
//! the caller counts or stops). Receivers discard longer datagrams unread.
//!
//! The `iface` and `step` fields repeat what is in the messages, so that a receiver can
//! route a datagram and tell old from new without decoding the payload; decoding checks
//! that both agree.

use crate::{decode, routed_by_iface, WireError};

/// Envelope magic bytes.
pub const ENV_MAGIC: [u8; 4] = *b"DYN2";
/// Envelope version.
pub const ENV_VERSION: u8 = 1;
/// Length of the envelope header.
pub const ENV_HDR_LEN: usize = 32;
/// Length of the trailing CRC-32.
pub const ENV_CRC_LEN: usize = 4;
/// Largest datagram (not fragmented on Ethernet).
pub const ENV_MAX_DATAGRAM: usize = 1472;
/// Largest payload.
pub const ENV_MAX_PAYLOAD: usize = ENV_MAX_DATAGRAM - ENV_HDR_LEN - ENV_CRC_LEN;

/// The variable fields of an envelope header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EnvHeader {
    /// Sending device id.
    pub sender: u32,
    /// Sequence number.
    pub seq: u32,
    /// Border id.
    pub iface: u32,
    /// Step.
    pub step: u64,
}

/// Envelope errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EnvError {
    /// Shorter than header and CRC, or `payload_len` does not match the actual length.
    Truncated,
    /// Larger than `ENV_MAX_DATAGRAM` / `ENV_MAX_PAYLOAD`.
    Oversize,
    /// Not enough room in the output buffer.
    Full,
    /// The magic bytes are not "DYN2".
    BadMagic,
    /// Wrong envelope version, or a reserved field is not 0.
    BadVersion,
    /// CRC-32 mismatch (corrupted on the link).
    BadCrc,
    /// The payload is not a valid delivery unit (empty, broken framing, not decodable as v2,
    /// a kind not routed by border, or iface/step not matching the envelope).
    BadUnit(Option<WireError>),
}

/// IEEE 802.3 CRC-32 (reflected, polynomial 0xEDB88320, initial value and final XOR
/// 0xFFFFFFFF); the same function as `dyne_crc32` of the firmware.
pub fn crc32(data: &[u8]) -> u32 {
    let mut c: u32 = 0xFFFF_FFFF;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            let m = (c & 1).wrapping_neg();
            c = (c >> 1) ^ (0xEDB8_8320 & m);
        }
    }
    !c
}

/// Check that `payload` (framed v2 messages) is a valid delivery unit for border `iface` and
/// `step`; return the number of messages.
pub fn check_unit(payload: &[u8], iface: u32, step: u64) -> Result<usize, EnvError> {
    let mut pos = 0usize;
    let mut n = 0usize;
    while pos < payload.len() {
        if pos + 4 > payload.len() {
            return Err(EnvError::BadUnit(Some(WireError::Truncated)));
        }
        let len = u32::from_le_bytes(payload[pos..pos + 4].try_into().unwrap()) as usize;
        let start = pos + 4;
        if len == 0 || start + len > payload.len() {
            return Err(EnvError::BadUnit(Some(WireError::Truncated)));
        }
        let (hdr, _, used) = decode(&payload[start..start + len]).map_err(|e| EnvError::BadUnit(Some(e)))?;
        if used != len {
            return Err(EnvError::BadUnit(Some(WireError::Truncated)));
        }
        if !routed_by_iface(hdr.kind) || hdr.src != iface || hdr.step != step {
            return Err(EnvError::BadUnit(None));
        }
        pos = start + len;
        n += 1;
    }
    if n == 0 {
        return Err(EnvError::BadUnit(None));
    }
    Ok(n)
}

/// Wrap a delivery unit in the envelope at the start of `buf` and return the datagram
/// length. On error nothing is written.
pub fn encode_envelope(buf: &mut [u8], hdr: &EnvHeader, payload: &[u8]) -> Result<usize, EnvError> {
    if payload.len() > ENV_MAX_PAYLOAD {
        return Err(EnvError::Oversize);
    }
    check_unit(payload, hdr.iface, hdr.step)?;
    let total = ENV_HDR_LEN + payload.len() + ENV_CRC_LEN;
    if buf.len() < total {
        return Err(EnvError::Full);
    }
    buf[0..4].copy_from_slice(&ENV_MAGIC);
    buf[4] = ENV_VERSION;
    buf[5] = 0;
    buf[6..8].copy_from_slice(&[0, 0]);
    buf[8..12].copy_from_slice(&hdr.sender.to_le_bytes());
    buf[12..16].copy_from_slice(&hdr.seq.to_le_bytes());
    buf[16..20].copy_from_slice(&hdr.iface.to_le_bytes());
    buf[20..24].copy_from_slice(&(payload.len() as u32).to_le_bytes());
    buf[24..32].copy_from_slice(&hdr.step.to_le_bytes());
    buf[ENV_HDR_LEN..ENV_HDR_LEN + payload.len()].copy_from_slice(payload);
    let body = ENV_HDR_LEN + payload.len();
    let crc = crc32(&buf[..body]);
    buf[body..body + 4].copy_from_slice(&crc.to_le_bytes());
    Ok(total)
}

/// Check one received datagram and return (header, payload). Checks in this order: length,
/// magic, length field, CRC, version and reserved fields, payload.
pub fn decode_envelope(dgram: &[u8]) -> Result<(EnvHeader, &[u8]), EnvError> {
    if dgram.len() > ENV_MAX_DATAGRAM {
        return Err(EnvError::Oversize);
    }
    if dgram.len() < ENV_HDR_LEN + ENV_CRC_LEN {
        return Err(EnvError::Truncated);
    }
    if dgram[0..4] != ENV_MAGIC {
        return Err(EnvError::BadMagic);
    }
    let plen = u32::from_le_bytes(dgram[20..24].try_into().unwrap()) as usize;
    if plen > ENV_MAX_PAYLOAD {
        return Err(EnvError::Oversize);
    }
    if ENV_HDR_LEN + plen + ENV_CRC_LEN != dgram.len() {
        return Err(EnvError::Truncated);
    }
    let body = ENV_HDR_LEN + plen;
    let crc = u32::from_le_bytes(dgram[body..body + 4].try_into().unwrap());
    if crc32(&dgram[..body]) != crc {
        return Err(EnvError::BadCrc);
    }
    if dgram[4] != ENV_VERSION || dgram[5] != 0 || dgram[6] != 0 || dgram[7] != 0 {
        return Err(EnvError::BadVersion);
    }
    let hdr = EnvHeader {
        sender: u32::from_le_bytes(dgram[8..12].try_into().unwrap()),
        seq: u32::from_le_bytes(dgram[12..16].try_into().unwrap()),
        iface: u32::from_le_bytes(dgram[16..20].try_into().unwrap()),
        step: u64::from_le_bytes(dgram[24..32].try_into().unwrap()),
    };
    let payload = &dgram[ENV_HDR_LEN..body];
    check_unit(payload, hdr.iface, hdr.step)?;
    Ok((hdr, payload))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{encode, Header, Payload, KIND_IFACE_BAND, KIND_POSITION, QTY_NONE, QTY_TEMPERATURE};

    /// 照合用の既知の配送単位: 界面5・ステップ42、値のみの界面サンプル列 [1.0, −2.5]
    fn golden_unit() -> Vec<u8> {
        let mut data = [0u8; 16];
        data[0..8].copy_from_slice(&1.0f64.to_bits().to_le_bytes());
        data[8..16].copy_from_slice(&(-2.5f64).to_bits().to_le_bytes());
        let hdr = Header { kind: KIND_IFACE_BAND, quantity: QTY_TEMPERATURE, src: 5, step: 42, t_us: 0 };
        let mut m = [0u8; 64];
        let n = encode(&mut m, &hdr, &Payload::IfaceBand { data: &data, count: 2, per_point: 1 }).unwrap();
        let mut u = Vec::new();
        u.extend_from_slice(&(n as u32).to_le_bytes());
        u.extend_from_slice(&m[..n]);
        u
    }

    const GOLDEN_HDR: EnvHeader = EnvHeader { sender: 3, seq: 7, iface: 5, step: 42 };

    fn golden_dgram() -> Vec<u8> {
        let u = golden_unit();
        let mut b = vec![0u8; ENV_MAX_DATAGRAM];
        let n = encode_envelope(&mut b, &GOLDEN_HDR, &u).unwrap();
        b.truncate(n);
        b
    }

    /// node-esp32/main/dyne_wire2_golden.h の照合用バイト列を読む (C側と同じ配列を
    /// 両方で検査し、二重定義の食い違いを検出する)
    fn c_golden() -> Vec<u8> {
        let src = include_str!("../../../node-esp32/main/dyne_wire2_golden.h");
        let start = src.find("DYNE_ENV_GOLDEN_BYTES").expect("配列の印が無い");
        let body = &src[start..];
        let open = body.find('{').unwrap();
        let close = body.find('}').unwrap();
        body[open + 1..close]
            .split(',')
            .map(|t| t.trim())
            .filter(|t| !t.is_empty())
            .map(|t| u8::from_str_radix(t.trim_start_matches("0x"), 16).unwrap())
            .collect()
    }

    #[test]
    fn golden_matches_c_header() {
        let g = golden_dgram();
        // 4 (枠) + 24 (v2ヘッダ) + 8 + 16 = 52 バイトの中身 + 外箱36
        assert_eq!(g.len(), 88);
        assert_eq!(&g[0..4], b"DYN2");
        let hex: Vec<String> = g.iter().map(|b| format!("0x{b:02X}")).collect();
        assert_eq!(c_golden(), g, "C側の照合用バイト列と食い違う。Rust側の値: {}", hex.join(","));
    }

    #[test]
    fn crc32_known_value() {
        // IEEE CRC32 の標準の確認値
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn roundtrip() {
        let g = golden_dgram();
        let (h, p) = decode_envelope(&g).unwrap();
        assert_eq!(h, GOLDEN_HDR);
        assert_eq!(p, &golden_unit()[..]);
    }

    #[test]
    fn detects_every_single_bit_flip() {
        // どの1ビットを反転させても受理しない (CRC32 は1ビット誤りを必ず検出する)
        let g = golden_dgram();
        for i in 0..g.len() {
            for bit in 0..8 {
                let mut c = g.clone();
                c[i] ^= 1 << bit;
                assert!(decode_envelope(&c).is_err(), "byte {i} bit {bit} の破損を見逃した");
            }
        }
        // 中身の破損は CRC で検出される (中身の検査より前)
        let mut c = g.clone();
        c[ENV_HDR_LEN + 40] ^= 0x10;
        assert_eq!(decode_envelope(&c), Err(EnvError::BadCrc));
    }

    #[test]
    fn rejects_truncation_and_bad_fields() {
        let g = golden_dgram();
        assert_eq!(decode_envelope(&g[..10]), Err(EnvError::Truncated));
        assert_eq!(decode_envelope(&g[..g.len() - 1]), Err(EnvError::Truncated));
        let mut c = g.clone();
        c[0] = b'X';
        assert_eq!(decode_envelope(&c), Err(EnvError::BadMagic));
        // 版を変えて CRC を付け直したものは版の不一致として拒否
        let mut c = g.clone();
        c[4] = 2;
        let body = c.len() - 4;
        let crc = crc32(&c[..body]);
        c[body..].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode_envelope(&c), Err(EnvError::BadVersion));
        // 外箱のステップと中身のステップの不一致 (CRC は正しい)
        let mut c = g.clone();
        c[24] = 43;
        let crc = crc32(&c[..body]);
        c[body..].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode_envelope(&c), Err(EnvError::BadUnit(None)));
    }

    #[test]
    fn oversize_and_capacity() {
        // 中身の上限ちょうどは送れて、1点増やすと送れない。
        // 値のみの界面サンプル列 n 点 = 枠4+ヘッダ24+8+8n。上限1436 → n=175 まで
        let mk = |n: usize| {
            let data = vec![0u8; 8 * n];
            let hdr = Header { kind: KIND_IFACE_BAND, quantity: QTY_NONE, src: 1, step: 0, t_us: 0 };
            let mut m = vec![0u8; 24 + 8 + 8 * n];
            let k = encode(&mut m, &hdr, &Payload::IfaceBand { data: &data, count: n as u32, per_point: 1 }).unwrap();
            let mut u = (k as u32).to_le_bytes().to_vec();
            u.extend_from_slice(&m[..k]);
            u
        };
        let h = EnvHeader { sender: 0, seq: 0, iface: 1, step: 0 };
        let mut b = vec![0u8; 2048];
        let n = encode_envelope(&mut b, &h, &mk(175)).unwrap();
        assert_eq!(n, ENV_MAX_DATAGRAM);
        assert!(decode_envelope(&b[..n]).is_ok());
        assert_eq!(encode_envelope(&mut b, &h, &mk(176)), Err(EnvError::Oversize));
        // 上限を超える長さのデータグラムは読まずに拒否
        assert_eq!(decode_envelope(&vec![0u8; ENV_MAX_DATAGRAM + 1]), Err(EnvError::Oversize));
        // 書き込み先が足りなければ何も書かない
        let mut small = vec![0xAAu8; 40];
        assert_eq!(encode_envelope(&mut small, &h, &mk(2)), Err(EnvError::Full));
        assert!(small.iter().all(|&x| x == 0xAA));
    }

    #[test]
    fn rejects_bad_units() {
        let h = EnvHeader { sender: 0, seq: 0, iface: 5, step: 42 };
        let mut b = vec![0u8; 256];
        // 空の配送単位
        assert_eq!(encode_envelope(&mut b, &h, &[]), Err(EnvError::BadUnit(None)));
        // 界面IDの不一致
        let h2 = EnvHeader { iface: 6, ..h };
        assert_eq!(encode_envelope(&mut b, &h2, &golden_unit()), Err(EnvError::BadUnit(None)));
        // 終端印 (len=0) は中身に含めない
        let mut u = golden_unit();
        u.extend_from_slice(&0u32.to_le_bytes());
        assert!(matches!(encode_envelope(&mut b, &h, &u), Err(EnvError::BadUnit(_))));
        // 界面で運ばない種別 (位置)
        let hp = Header { kind: KIND_POSITION, quantity: QTY_NONE, src: 5, step: 42, t_us: 0 };
        let mut m = [0u8; 64];
        let k = encode(&mut m, &hp, &Payload::Position { x: 0.0, y: 0.0, temp: 0.0, gx: 0.0, gy: 0.0 }).unwrap();
        let mut u = (k as u32).to_le_bytes().to_vec();
        u.extend_from_slice(&m[..k]);
        assert_eq!(encode_envelope(&mut b, &h, &u), Err(EnvError::BadUnit(None)));
        // 2通入りの配送単位は受理 (光線束+界面サンプル列 など)
        let mut u2 = golden_unit();
        u2.extend_from_slice(&golden_unit());
        let n = encode_envelope(&mut b, &h, &u2).unwrap();
        assert_eq!(check_unit(decode_envelope(&b[..n]).unwrap().1, 5, 42), Ok(2));
    }
}

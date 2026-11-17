//! dyne-wire: the single definition of Dyne's message format (wire format v2), `no_std`.
//!
//! Every host (the simulators, the PC tools, and the ESP32 firmware through a C copy that
//! is checked against this crate) and every solver ("actor") encodes and decodes border
//! messages with this crate, so the layout is written down once. The format carries the
//! values exchanged at the borders between regions, plus a few message kinds used by the
//! applications built on Dyne (flocking agents, range sensors).
//!
//! Wire format v2 has no sequence number and no checksum: on real radio links each
//! datagram is wrapped in the [`envelope`] (sequence number and CRC-32). The constants of
//! the solver interface are in [`abi`].
//!
//! ## Message layout (little-endian)
//!
//! ```text
//! offset size field
//! 0      1    version  = 2
//! 1      1    kind     (0=IFACE_SCALAR, 1=IFACE_BAND, 2=POSITION, 3=HEARSAY, 4=RANGE_FRAME, 5=RAYS)
//! 2      1    quantity (0=temperature, 1=flow, 2=displacement, 3=inertial, 255=unspecified)
//! 3      1    reserved = 0
//! 4      4    src      u32 (IFACE_* and RAYS: border id; POSITION, HEARSAY: agent id;
//!                           RANGE_FRAME: sensor id)
//! 8      8    step     u64 (logical time step)
//! 16     8    t_us     u64 (time stamp in microseconds; 0 if unused)
//! 24     -    payload  (depends on kind)
//! ```
//!
//! Payloads:
//! - `IFACE_SCALAR`: value f64, grad f64 (40 bytes in total): one value and its gradient
//!   at a 1D border.
//! - `IFACE_BAND`: count u32, per_point u32, then `count × per_point` f64: a border of
//!   `count` points with `per_point` quantities each, grouped by point (point 0 quantities
//!   0..per_point−1, then point 1, ...). Which quantities is agreed per solver type.
//!   `per_point = 0` is invalid.
//! - `POSITION`: x, y, temp, gx, gy, all f64 (64 bytes in total; flocking demo).
//! - `HEARSAY`: x, y, score f64, age_steps u64 (56 bytes in total; flocking demo).
//! - `RANGE_FRAME`: count u32, cols u32, then `count` zones of (distance f64 \[m\], sensor
//!   status u32, reserved u32): one frame of a multi-zone range sensor, from the host to
//!   the region that owns the sensor. Which row is used and which status counts as valid
//!   is decided by the solver.
//! - `RAYS`: count u32, reserved u32, then `count` rays of 40 bytes (see [`Ray`]): rays
//!   passed to a neighbouring region (room-shape reconstruction).
//!
//! ## Mailbox framing: fixed-size buffers
//!
//! Solvers that exchange messages receive and send them through two fixed-size byte
//! buffers, the inbox and the outbox. Messages are packed from the start as
//! `[u32 len][len bytes of message]` (len includes the header). The sequence ends at
//! `len = 0` or at the end of the buffer. A message that does not fit is **dropped and
//! counted** (the writer keeps its own counter); a message is never written partially.
//! See [`MailboxWriter`] and [`MailboxReader`].

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

pub mod abi;
/// The outer envelope for real radio links (UDP): sequence number and CRC-32 around v2 messages.
pub mod envelope;

/// Version byte of wire format v2.
pub const VERSION: u8 = 2;

/// Message kind: one value and its gradient at a 1D border.
pub const KIND_IFACE_SCALAR: u8 = 0;
/// Message kind: values at the points of a border (2D and 3D).
pub const KIND_IFACE_BAND: u8 = 1;
/// Message kind: position, temperature and gradient estimate of a flocking agent.
pub const KIND_POSITION: u8 = 2;
/// Message kind: the best place an agent has heard of (flocking demo).
pub const KIND_HEARSAY: u8 = 3;
/// Message kind: one frame of a multi-zone range sensor.
pub const KIND_RANGE_FRAME: u8 = 4;
/// Message kind: rays passed to a neighbouring region (room-shape reconstruction).
pub const KIND_RAYS: u8 = 5;

/// Whether messages of `kind` are routed by border id (the header's `src` is a border id).
/// Hosts and firmware bundle such messages per (border id, step) into one delivery unit
/// for the neighbour.
pub fn routed_by_iface(kind: u8) -> bool {
    matches!(kind, KIND_IFACE_SCALAR | KIND_IFACE_BAND | KIND_RAYS)
}

/// Flag in a ray's id: no target (nothing within the sensor's maximum range in that zone).
pub const RAY_FLAG_NO_TARGET: u32 = 1;

/// One ray of a `RAYS` message (40 bytes on the wire).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    /// Origin x \[m\].
    pub ox: f64,
    /// Origin y \[m\].
    pub oy: f64,
    /// End x \[m\].
    pub ex: f64,
    /// End y \[m\].
    pub ey: f64,
    /// Step at which the measurement was taken.
    pub acq_step: u32,
    /// Sensor id << 16 | zone << 8 | flags (`RAY_FLAG_*`).
    pub id: u32,
}

/// Bytes per ray.
pub const RAY_LEN: usize = 40;
/// Bytes per zone of a range frame.
pub const ZONE_LEN: usize = 16;

/// Angle \[rad\] of the centre of zone `c` (0..cols) from the sensor's forward direction,
/// counter-clockwise positive; zone 0 is the leftmost (positive angle). Shared by the
/// simulated sensor and the solvers.
pub fn zone_angle_offset(c: u32, cols: u32, fov_rad: f64) -> f64 {
    fov_rad * (((cols as f64) - 1.0) * 0.5 - c as f64) / cols as f64
}

/// Quantity: temperature.
pub const QTY_TEMPERATURE: u8 = 0;
/// Quantity: flow.
pub const QTY_FLOW: u8 = 1;
/// Quantity: displacement (the `u` of the wave equation; used by wave2d-actor).
pub const QTY_DISPLACEMENT: u8 = 2;
/// Quantity: inertial (acceleration and angular rate; used by imu-fusion-actor).
pub const QTY_INERTIAL: u8 = 3;
/// Quantity: unspecified.
pub const QTY_NONE: u8 = 255;

/// Header length in bytes.
pub const HDR_LEN: usize = 24;

/// The variable fields of a message header (the version byte is implied).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    /// Message kind (`KIND_*`).
    pub kind: u8,
    /// Physical quantity (`QTY_*`).
    pub quantity: u8,
    /// Border id for `IFACE_*` and `RAYS`; sender's agent id for `POSITION` and `HEARSAY`;
    /// sensor id for `RANGE_FRAME`.
    pub src: u32,
    /// Logical time step.
    pub step: u64,
    /// Time stamp in microseconds (virtual time in the simulators); 0 if unused.
    pub t_us: u64,
}

/// A decoded payload; variable-length data borrows from the buffer.
#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(missing_docs)] // the fields are described in each variant's documentation
pub enum Payload<'a> {
    /// One value and its gradient at a 1D border.
    IfaceScalar { value: f64, grad: f64 },
    /// Values at the points of a border: `data` holds `count × per_point` f64 as raw
    /// little-endian bytes (read with [`band_get`], independent of alignment).
    IfaceBand { data: &'a [u8], count: u32, per_point: u32 },
    /// Flocking demo: position, temperature and gradient estimate.
    Position { x: f64, y: f64, temp: f64, gx: f64, gy: f64 },
    /// Flocking demo: the best place the sender knows of, and how old that news is.
    Hearsay { x: f64, y: f64, score: f64, age_steps: u64 },
    /// Range frame: `data` holds `ZONE_LEN × count` bytes (read with [`range_zone`]);
    /// `cols` zones per row.
    RangeFrame { data: &'a [u8], count: u32, cols: u32 },
    /// Rays: `data` holds `RAY_LEN × count` bytes (read with [`ray_get`]).
    Rays { data: &'a [u8], count: u32 },
}

/// Errors of encoding and decoding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WireError {
    /// The buffer is too short, or the payload length does not match what is declared.
    Truncated,
    /// The version byte is not [`VERSION`].
    BadVersion,
    /// Unknown kind, or a header kind that does not match the payload.
    BadKind,
    /// Invalid field value (for example `per_point = 0`).
    BadField,
    /// Not enough room in the output buffer (the caller drops and counts the message).
    Full,
}

#[inline]
fn put_u32(buf: &mut [u8], off: usize, v: u32) {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
}
#[inline]
fn put_u64(buf: &mut [u8], off: usize, v: u64) {
    buf[off..off + 8].copy_from_slice(&v.to_le_bytes());
}
#[inline]
fn put_f64(buf: &mut [u8], off: usize, v: f64) {
    buf[off..off + 8].copy_from_slice(&v.to_bits().to_le_bytes());
}
#[inline]
fn get_u32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes(buf[off..off + 4].try_into().unwrap())
}
#[inline]
fn get_u64(buf: &[u8], off: usize) -> u64 {
    u64::from_le_bytes(buf[off..off + 8].try_into().unwrap())
}
#[inline]
fn get_f64(buf: &[u8], off: usize) -> f64 {
    f64::from_bits(get_u64(buf, off))
}

/// Payload length in bytes. For variable-length kinds it depends on `n`: the total number
/// of f64 (`count × per_point`) for `IFACE_BAND`, the number of zones for `RANGE_FRAME`,
/// the number of rays for `RAYS`. `None` for an unknown kind.
pub fn payload_len(kind: u8, n: u32) -> Option<usize> {
    match kind {
        KIND_IFACE_SCALAR => Some(16),
        KIND_IFACE_BAND => Some(8 + 8 * n as usize),
        KIND_RANGE_FRAME => Some(8 + ZONE_LEN * n as usize),
        KIND_RAYS => Some(8 + RAY_LEN * n as usize),
        KIND_POSITION => Some(40),
        KIND_HEARSAY => Some(32),
        _ => None,
    }
}

/// Encode one message at the start of `buf` and return the number of bytes written.
/// If it does not fit, return `Err(Full)` and **write nothing** (no partial writes).
pub fn encode(buf: &mut [u8], hdr: &Header, payload: &Payload) -> Result<usize, WireError> {
    let (kind_ok, plen) = match payload {
        Payload::IfaceScalar { .. } => (KIND_IFACE_SCALAR, 16),
        Payload::IfaceBand { data, count, per_point } => {
            if *per_point == 0 {
                return Err(WireError::BadField);
            }
            if data.len() != 8 * (*count as usize) * (*per_point as usize) {
                return Err(WireError::Truncated);
            }
            (KIND_IFACE_BAND, 8 + data.len())
        }
        Payload::Position { .. } => (KIND_POSITION, 40),
        Payload::Hearsay { .. } => (KIND_HEARSAY, 32),
        Payload::RangeFrame { data, count, cols } => {
            if *cols == 0 || *count % *cols != 0 {
                return Err(WireError::BadField);
            }
            if data.len() != ZONE_LEN * *count as usize {
                return Err(WireError::Truncated);
            }
            (KIND_RANGE_FRAME, 8 + data.len())
        }
        Payload::Rays { data, count } => {
            if data.len() != RAY_LEN * *count as usize {
                return Err(WireError::Truncated);
            }
            (KIND_RAYS, 8 + data.len())
        }
    };
    if hdr.kind != kind_ok {
        return Err(WireError::BadKind);
    }
    let total = HDR_LEN + plen;
    if buf.len() < total {
        return Err(WireError::Full);
    }
    buf[0] = VERSION;
    buf[1] = hdr.kind;
    buf[2] = hdr.quantity;
    buf[3] = 0;
    put_u32(buf, 4, hdr.src);
    put_u64(buf, 8, hdr.step);
    put_u64(buf, 16, hdr.t_us);
    let p = HDR_LEN;
    match payload {
        Payload::IfaceScalar { value, grad } => {
            put_f64(buf, p, *value);
            put_f64(buf, p + 8, *grad);
        }
        Payload::IfaceBand { data, count, per_point } => {
            put_u32(buf, p, *count);
            put_u32(buf, p + 4, *per_point);
            buf[p + 8..p + 8 + data.len()].copy_from_slice(data);
        }
        Payload::Position { x, y, temp, gx, gy } => {
            put_f64(buf, p, *x);
            put_f64(buf, p + 8, *y);
            put_f64(buf, p + 16, *temp);
            put_f64(buf, p + 24, *gx);
            put_f64(buf, p + 32, *gy);
        }
        Payload::Hearsay { x, y, score, age_steps } => {
            put_f64(buf, p, *x);
            put_f64(buf, p + 8, *y);
            put_f64(buf, p + 16, *score);
            put_u64(buf, p + 24, *age_steps);
        }
        Payload::RangeFrame { data, count, cols } => {
            put_u32(buf, p, *count);
            put_u32(buf, p + 4, *cols);
            buf[p + 8..p + 8 + data.len()].copy_from_slice(data);
        }
        Payload::Rays { data, count } => {
            put_u32(buf, p, *count);
            put_u32(buf, p + 4, 0);
            buf[p + 8..p + 8 + data.len()].copy_from_slice(data);
        }
    }
    Ok(total)
}

/// Decode the message at the start of `buf`; return (header, payload, bytes consumed).
pub fn decode(buf: &[u8]) -> Result<(Header, Payload<'_>, usize), WireError> {
    if buf.len() < HDR_LEN {
        return Err(WireError::Truncated);
    }
    if buf[0] != VERSION {
        return Err(WireError::BadVersion);
    }
    let kind = buf[1];
    let hdr = Header {
        kind,
        quantity: buf[2],
        src: get_u32(buf, 4),
        step: get_u64(buf, 8),
        t_us: get_u64(buf, 16),
    };
    let p = HDR_LEN;
    let (payload, plen) = match kind {
        KIND_IFACE_SCALAR => {
            if buf.len() < p + 16 {
                return Err(WireError::Truncated);
            }
            (
                Payload::IfaceScalar { value: get_f64(buf, p), grad: get_f64(buf, p + 8) },
                16,
            )
        }
        KIND_IFACE_BAND => {
            if buf.len() < p + 8 {
                return Err(WireError::Truncated);
            }
            let count = get_u32(buf, p);
            let per_point = get_u32(buf, p + 4);
            if per_point == 0 {
                return Err(WireError::BadField);
            }
            let plen = 8 + 8 * (count as usize) * (per_point as usize);
            if buf.len() < p + plen {
                return Err(WireError::Truncated);
            }
            (
                Payload::IfaceBand { data: &buf[p + 8..p + plen], count, per_point },
                plen,
            )
        }
        KIND_POSITION => {
            if buf.len() < p + 40 {
                return Err(WireError::Truncated);
            }
            (
                Payload::Position {
                    x: get_f64(buf, p),
                    y: get_f64(buf, p + 8),
                    temp: get_f64(buf, p + 16),
                    gx: get_f64(buf, p + 24),
                    gy: get_f64(buf, p + 32),
                },
                40,
            )
        }
        KIND_HEARSAY => {
            if buf.len() < p + 32 {
                return Err(WireError::Truncated);
            }
            (
                Payload::Hearsay {
                    x: get_f64(buf, p),
                    y: get_f64(buf, p + 8),
                    score: get_f64(buf, p + 16),
                    age_steps: get_u64(buf, p + 24),
                },
                32,
            )
        }
        KIND_RANGE_FRAME => {
            if buf.len() < p + 8 {
                return Err(WireError::Truncated);
            }
            let count = get_u32(buf, p);
            let cols = get_u32(buf, p + 4);
            if cols == 0 || count % cols != 0 {
                return Err(WireError::BadField);
            }
            let plen = 8 + ZONE_LEN * count as usize;
            if buf.len() < p + plen {
                return Err(WireError::Truncated);
            }
            (Payload::RangeFrame { data: &buf[p + 8..p + plen], count, cols }, plen)
        }
        KIND_RAYS => {
            if buf.len() < p + 8 {
                return Err(WireError::Truncated);
            }
            let count = get_u32(buf, p);
            let plen = 8 + RAY_LEN * count as usize;
            if buf.len() < p + plen {
                return Err(WireError::Truncated);
            }
            (Payload::Rays { data: &buf[p + 8..p + plen], count }, plen)
        }
        _ => return Err(WireError::BadKind),
    };
    Ok((hdr, payload, p + plen))
}

/// Read the `i`-th f64 of an `IFACE_BAND` payload (independent of alignment). Quantity `q`
/// of point `k` is at `i = k × per_point + q`.
pub fn band_get(data: &[u8], i: usize) -> f64 {
    get_f64(data, 8 * i)
}

/// Zone `i` of a range frame: (distance \[m\], sensor status).
pub fn range_zone(data: &[u8], i: usize) -> (f64, u32) {
    (get_f64(data, ZONE_LEN * i), get_u32(data, ZONE_LEN * i + 8))
}

/// Write zone `i` of a range frame (`data` is the `ZONE_LEN × count` area).
pub fn range_zone_put(data: &mut [u8], i: usize, dist_m: f64, status: u32) {
    let o = ZONE_LEN * i;
    put_f64(data, o, dist_m);
    put_u32(data, o + 8, status);
    put_u32(data, o + 12, 0);
}

/// Read ray `i` of a `RAYS` payload.
pub fn ray_get(data: &[u8], i: usize) -> Ray {
    let o = RAY_LEN * i;
    Ray {
        ox: get_f64(data, o),
        oy: get_f64(data, o + 8),
        ex: get_f64(data, o + 16),
        ey: get_f64(data, o + 24),
        acq_step: get_u32(data, o + 32),
        id: get_u32(data, o + 36),
    }
}

/// Write ray `i` of a `RAYS` payload (`data` is the `RAY_LEN × count` area).
pub fn ray_put(data: &mut [u8], i: usize, r: &Ray) {
    let o = RAY_LEN * i;
    put_f64(data, o, r.ox);
    put_f64(data, o + 8, r.oy);
    put_f64(data, o + 16, r.ex);
    put_f64(data, o + 24, r.ey);
    put_u32(data, o + 32, r.acq_step);
    put_u32(data, o + 36, r.id);
}

// ---------- 受信箱/送信箱の枠 (固定長バッファへの詰め込み) ----------

/// Writer that packs messages into a fixed-size buffer as `[u32 len][message]`.
/// A message that does not fit is **dropped and counted**.
pub struct MailboxWriter<'a> {
    buf: &'a mut [u8],
    used: usize,
    /// Number of messages dropped because they did not fit.
    pub dropped: u64,
}

impl<'a> MailboxWriter<'a> {
    /// Start writing at the beginning of `buf`.
    pub fn new(buf: &'a mut [u8]) -> Self {
        MailboxWriter { buf, used: 0, dropped: 0 }
    }

    /// Append one message. If it does not fit, drop it, add 1 to `dropped` and return
    /// `Err(Full)`.
    pub fn push(&mut self, hdr: &Header, payload: &Payload) -> Result<(), WireError> {
        let start = self.used + 4;
        if start > self.buf.len() {
            self.dropped += 1;
            return Err(WireError::Full);
        }
        match encode(&mut self.buf[start..], hdr, payload) {
            Ok(n) => {
                put_u32(self.buf, self.used, n as u32);
                self.used = start + n;
                Ok(())
            }
            Err(WireError::Full) => {
                self.dropped += 1;
                Err(WireError::Full)
            }
            Err(e) => Err(e),
        }
    }

    /// Bytes used so far (not counting the terminating `len = 0`).
    pub fn used(&self) -> usize {
        self.used
    }

    /// Write the terminating `len = 0` if there is room (optional: readers also stop at
    /// the end of the buffer). Returns the bytes used.
    pub fn finish(&mut self) -> usize {
        if self.used + 4 <= self.buf.len() {
            put_u32(self.buf, self.used, 0);
        }
        self.used
    }
}

/// Reader that iterates over the `[u32 len][message]` sequence of a fixed-size buffer.
pub struct MailboxReader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> MailboxReader<'a> {
    /// Start reading at the beginning of `buf`.
    pub fn new(buf: &'a [u8]) -> Self {
        MailboxReader { buf, pos: 0 }
    }
}

impl<'a> Iterator for MailboxReader<'a> {
    type Item = Result<(Header, Payload<'a>, &'a [u8]), WireError>;

    /// The next message, or `None` at the end (`len = 0` or end of buffer). The third
    /// element is the raw message bytes (for forwarding or keeping a copy).
    fn next(&mut self) -> Option<Self::Item> {
        if self.pos + 4 > self.buf.len() {
            return None;
        }
        let len = get_u32(self.buf, self.pos) as usize;
        if len == 0 {
            return None;
        }
        let start = self.pos + 4;
        if start + len > self.buf.len() {
            return Some(Err(WireError::Truncated));
        }
        let raw = &self.buf[start..start + len];
        self.pos = start + len;
        match decode(raw) {
            Ok((h, p, consumed)) if consumed == len => Some(Ok((h, p, raw))),
            Ok(_) => Some(Err(WireError::Truncated)),
            Err(e) => Some(Err(e)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_scalar() {
        let mut buf = [0u8; 64];
        let hdr = Header { kind: KIND_IFACE_SCALAR, quantity: QTY_TEMPERATURE, src: 7, step: 1234, t_us: 98_765 };
        let pl = Payload::IfaceScalar { value: 0.5, grad: -3.25 };
        let n = encode(&mut buf, &hdr, &pl).unwrap();
        assert_eq!(n, 40); // v1と同じ40バイト
        let (h2, p2, c) = decode(&buf).unwrap();
        assert_eq!(h2, hdr);
        assert_eq!(p2, pl);
        assert_eq!(c, 40);
    }

    #[test]
    fn roundtrip_band() {
        let mut buf = [0u8; 128];
        let vals = [1.0f64, 2.5, -3.0];
        let mut data = [0u8; 24];
        for (i, v) in vals.iter().enumerate() {
            data[8 * i..8 * i + 8].copy_from_slice(&v.to_bits().to_le_bytes());
        }
        let hdr = Header { kind: KIND_IFACE_BAND, quantity: QTY_TEMPERATURE, src: 3, step: 9, t_us: 0 };
        let n = encode(&mut buf, &hdr, &Payload::IfaceBand { data: &data, count: 3, per_point: 1 }).unwrap();
        assert_eq!(n, HDR_LEN + 8 + 24);
        let (h2, p2, _) = decode(&buf).unwrap();
        assert_eq!(h2, hdr);
        match p2 {
            Payload::IfaceBand { data, count, per_point } => {
                assert_eq!(count, 3);
                assert_eq!(per_point, 1);
                for (i, v) in vals.iter().enumerate() {
                    assert_eq!(band_get(data, i).to_bits(), v.to_bits());
                }
            }
            _ => panic!("kind mismatch"),
        }
    }

    #[test]
    fn band_per_point_layout_and_rejects_zero() {
        // 2点×(値,勾配) = f64×4。並びは点ごと: 点0の値,点0の勾配,点1の値,点1の勾配
        let vals = [10.0f64, -1.0, 20.0, -2.0];
        let mut data = [0u8; 32];
        for (i, v) in vals.iter().enumerate() {
            data[8 * i..8 * i + 8].copy_from_slice(&v.to_bits().to_le_bytes());
        }
        let hdr = Header { kind: KIND_IFACE_BAND, quantity: QTY_NONE, src: 1, step: 2, t_us: 0 };
        let mut buf = [0u8; 128];
        let n = encode(&mut buf, &hdr, &Payload::IfaceBand { data: &data, count: 2, per_point: 2 }).unwrap();
        assert_eq!(n, HDR_LEN + 8 + 32);
        assert_eq!(Some(n - HDR_LEN), payload_len(KIND_IFACE_BAND, 4));
        match decode(&buf[..n]).unwrap().1 {
            Payload::IfaceBand { data, count: 2, per_point: 2 } => {
                assert_eq!(band_get(data, 1 * 2 + 1), -2.0); // 点1の勾配
            }
            p => panic!("unexpected {:?}", p),
        }
        // データ長が count×per_point と合わない
        assert_eq!(
            encode(&mut buf, &hdr, &Payload::IfaceBand { data: &data, count: 3, per_point: 2 }),
            Err(WireError::Truncated)
        );
        // per_point=0 は送るのも受けるのも不正
        assert_eq!(
            encode(&mut buf, &hdr, &Payload::IfaceBand { data: &[], count: 0, per_point: 0 }),
            Err(WireError::BadField)
        );
        buf[HDR_LEN + 4..HDR_LEN + 8].copy_from_slice(&0u32.to_le_bytes());
        assert_eq!(decode(&buf[..n]).map(|_| ()), Err(WireError::BadField));
    }

    #[test]
    fn roundtrip_range_frame_and_rays() {
        let mut zones = [0u8; ZONE_LEN * 4];
        for i in 0..4 {
            range_zone_put(&mut zones, i, 0.5 + i as f64, if i == 2 { 255 } else { 5 });
        }
        let hdr = Header { kind: KIND_RANGE_FRAME, quantity: QTY_NONE, src: 3, step: 7, t_us: 11 };
        let mut buf = [0u8; 256];
        let n = encode(&mut buf, &hdr, &Payload::RangeFrame { data: &zones, count: 4, cols: 2 }).unwrap();
        assert_eq!(n, HDR_LEN + 8 + 64);
        match decode(&buf[..n]).unwrap() {
            (h, Payload::RangeFrame { data, count: 4, cols: 2 }, _) => {
                assert_eq!(h, hdr);
                assert_eq!(range_zone(data, 2), (2.5, 255));
                assert_eq!(range_zone(data, 3), (3.5, 5));
            }
            p => panic!("unexpected {:?}", p),
        }
        // cols が count を割り切らない
        assert_eq!(
            encode(&mut buf, &hdr, &Payload::RangeFrame { data: &zones, count: 4, cols: 3 }),
            Err(WireError::BadField)
        );

        let r0 = Ray { ox: 0.1, oy: 0.2, ex: 3.9, ey: 1.0, acq_step: 42, id: (1 << 16) | (5 << 8) };
        let r1 = Ray { id: r0.id | RAY_FLAG_NO_TARGET, ..r0 };
        let mut rays = [0u8; RAY_LEN * 2];
        ray_put(&mut rays, 0, &r0);
        ray_put(&mut rays, 1, &r1);
        let hdr = Header { kind: KIND_RAYS, quantity: QTY_NONE, src: 9, step: 8, t_us: 0 };
        let n = encode(&mut buf, &hdr, &Payload::Rays { data: &rays, count: 2 }).unwrap();
        assert_eq!(Some(n - HDR_LEN), payload_len(KIND_RAYS, 2));
        match decode(&buf[..n]).unwrap().1 {
            Payload::Rays { data, count: 2 } => {
                assert_eq!(ray_get(data, 0), r0);
                assert_eq!(ray_get(data, 1), r1);
            }
            p => panic!("unexpected {:?}", p),
        }
        assert!(routed_by_iface(KIND_RAYS) && routed_by_iface(KIND_IFACE_BAND));
        assert!(!routed_by_iface(KIND_RANGE_FRAME) && !routed_by_iface(KIND_POSITION));
    }

    #[test]
    fn zone_angles_symmetric_left_positive() {
        let fov = 0.8;
        assert!(zone_angle_offset(0, 8, fov) > 0.0);
        assert_eq!(zone_angle_offset(0, 8, fov), -zone_angle_offset(7, 8, fov));
        assert_eq!(zone_angle_offset(3, 8, fov), fov * 0.5 / 8.0);
    }

    #[test]
    fn roundtrip_position_hearsay() {
        let mut buf = [0u8; 128];
        let hp = Header { kind: KIND_POSITION, quantity: QTY_TEMPERATURE, src: 42, step: 5, t_us: 1 };
        let pp = Payload::Position { x: 0.1, y: 0.9, temp: 23.5, gx: -0.01, gy: 0.02 };
        let n = encode(&mut buf, &hp, &pp).unwrap();
        assert_eq!(n, 64);
        let (h2, p2, _) = decode(&buf).unwrap();
        assert_eq!((h2, p2), (hp, pp));

        let hh = Header { kind: KIND_HEARSAY, quantity: QTY_TEMPERATURE, src: 42, step: 5, t_us: 1 };
        let ph = Payload::Hearsay { x: 0.3, y: 0.4, score: 0.7, age_steps: 12 };
        let n = encode(&mut buf, &hh, &ph).unwrap();
        assert_eq!(n, 56);
        let (h2, p2, _) = decode(&buf).unwrap();
        assert_eq!((h2, p2), (hh, ph));
    }

    #[test]
    fn mailbox_overflow_discard_and_count() {
        // 40バイト+枠4バイト=44バイト/通。容量100 → 2通で88、3通目はあふれ
        let mut buf = [0u8; 100];
        let mut w = MailboxWriter::new(&mut buf);
        let hdr = Header { kind: KIND_IFACE_SCALAR, quantity: QTY_NONE, src: 0, step: 0, t_us: 0 };
        let pl = Payload::IfaceScalar { value: 1.0, grad: 2.0 };
        assert!(w.push(&hdr, &pl).is_ok());
        assert!(w.push(&hdr, &pl).is_ok());
        assert_eq!(w.push(&hdr, &pl), Err(WireError::Full));
        assert_eq!(w.dropped, 1);
        w.finish();
        drop(w);
        let msgs: Vec<_> = MailboxReader::new(&buf)
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        assert_eq!(msgs.len(), 2); // あふれた1通は存在しない (部分書き込みなし)
    }

    #[test]
    fn decode_rejects_bad_input() {
        assert_eq!(decode(&[0u8; 10]), Err(WireError::Truncated));
        let mut buf = [0u8; 40];
        buf[0] = 1; // 旧版
        assert_eq!(decode(&buf), Err(WireError::BadVersion));
        buf[0] = VERSION;
        buf[1] = 99;
        assert_eq!(decode(&buf), Err(WireError::BadKind));
    }

    /// 物理量種別の番号は重ならず、ヘッダの1バイトにそのまま載って往復する (2026-11-13 追加)
    #[test]
    fn quantity_codes_distinct_and_roundtrip() {
        let q = [QTY_TEMPERATURE, QTY_FLOW, QTY_DISPLACEMENT, QTY_INERTIAL, QTY_NONE];
        for i in 0..q.len() {
            for j in i + 1..q.len() {
                assert_ne!(q[i], q[j]);
            }
        }
        for &qq in &q {
            let hdr = Header { kind: KIND_IFACE_BAND, quantity: qq, src: 1, step: 3, t_us: 5 };
            let data = 1.5f64.to_le_bytes();
            let pl = Payload::IfaceBand { data: &data, count: 1, per_point: 1 };
            let mut buf = [0u8; 64];
            let n = encode(&mut buf, &hdr, &pl).unwrap();
            assert_eq!(buf[2], qq);
            let (h2, _, used) = decode(&buf[..n]).unwrap();
            assert_eq!(used, n);
            assert_eq!(h2.quantity, qq);
        }
    }
}

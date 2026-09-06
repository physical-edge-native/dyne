//! dyne-wire: ワイヤ形式v2の単一定義 (no_std)
//!
//! ロードマップ (docs/2026-09-01-roadmap.md 2.5) のワイヤ形式v2改訂
//! (時刻印・物理量種別・帯ペイロード) と、発注6 (群れ移植) で追加する
//! メッセージ種別 (位置・伝聞) を定義する。
//!
//! **二重定義の禁止 (2026-09-02 ユーザー決定)**: アクターの受信箱/送信箱
//! バッファ上のメッセージ形式は本クレートの規約と同一とする。ホスト側・
//! アクター側の双方が本クレートに依存し、レイアウトを別々に書かない。
//! (v1 = M0の40バイト形式は crates/dyne-proto に残る歴史的定義。v2は
//! IFACE_SCALAR がその意味論を包含する)
//!
//! ## メッセージレイアウト (リトルエンディアン)
//!
//! ```text
//! offset size field
//! 0      1    version  = 2
//! 1      1    kind     (0=IFACE_SCALAR, 1=IFACE_BAND, 2=POSITION, 3=HEARSAY)
//! 2      1    quantity (物理量種別: 0=温度, 1=流速, 255=指定なし)
//! 3      1    reserved = 0
//! 4      4    src      u32 (kind=IFACE_*: 界面ID / kind=POSITION,HEARSAY: 個体ID)
//! 8      8    step     u64 (論理ステップ)
//! 16     8    t_us     u64 (時刻印: 仮想時刻µs。未使用なら0)
//! 24     -    payload  (kind依存)
//! ```
//!
//! payload:
//! - IFACE_SCALAR: value f64, grad f64 (計40バイト。v1の40バイト意味論を包含)
//! - IFACE_BAND:   count u32, pad u32, f64×count (2次元界面=帯の先行定義)
//! - POSITION:     x f64, y f64, temp f64, gx f64, gy f64 (計64バイト)
//! - HEARSAY:      x f64, y f64, score f64, age_steps u64 (計56バイト)
//!
//! POSITION/HEARSAY のフィールドは現行 dyne-swarm が交換している量に合わせた
//! 初期定義。群れ実装で確定し、変更時は本コメントと設計書を併せて改訂する。
//!
//! ## 受信箱/送信箱の枠 (framing) — 固定長バッファ
//!
//! バッファは固定長。メッセージは先頭から `[u32 len][len バイトのメッセージ]` で
//! 連続に詰める (lenにヘッダを含む)。終端は len=0 または容量切れ。
//! **あふれ時の挙動 (2026-09-02 ユーザー決定)**: 入り切らないメッセージは
//! **破棄し、破棄計数を+1**する (書いた側が自分の計数を持つ。ホストは統計に
//! 報告し、アクターは自分の破棄数をエクスポートで返せるようにする)。
//! 部分書き込みは行わない (壊れたメッセージをバッファに残さない)。

#![cfg_attr(not(test), no_std)]

pub mod abi;

pub const VERSION: u8 = 2;

/// メッセージ種別
pub const KIND_IFACE_SCALAR: u8 = 0;
pub const KIND_IFACE_BAND: u8 = 1;
pub const KIND_POSITION: u8 = 2;
pub const KIND_HEARSAY: u8 = 3;

/// 物理量種別
pub const QTY_TEMPERATURE: u8 = 0;
pub const QTY_FLOW: u8 = 1;
pub const QTY_NONE: u8 = 255;

/// ヘッダ長 (バイト)
pub const HDR_LEN: usize = 24;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub kind: u8,
    pub quantity: u8,
    /// kind=IFACE_*: 界面ID / kind=POSITION,HEARSAY: 送信元の個体ID
    pub src: u32,
    pub step: u64,
    /// 時刻印 (仮想時刻µs。未使用なら0)
    pub t_us: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Payload<'a> {
    /// 界面スカラー交換 (値, 勾配)
    IfaceScalar { value: f64, grad: f64 },
    /// 帯 (2次元界面の先行定義)。f64列はアラインメント非依存で読む
    IfaceBand { data: &'a [u8], count: u32 },
    /// 群れ: 位置・温度・勾配推定
    Position { x: f64, y: f64, temp: f64, gx: f64, gy: f64 },
    /// 群れ: 伝聞 (自分が知る最良地点)
    Hearsay { x: f64, y: f64, score: f64, age_steps: u64 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WireError {
    /// バッファが短すぎる / ペイロード長が宣言と合わない
    Truncated,
    /// version 不一致
    BadVersion,
    /// 未知の kind
    BadKind,
    /// 書き込み先の容量不足 (呼び出し側は破棄+計数する)
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

/// ペイロード長 (バイト)。IFACE_BAND は count 依存
pub fn payload_len(kind: u8, band_count: u32) -> Option<usize> {
    match kind {
        KIND_IFACE_SCALAR => Some(16),
        KIND_IFACE_BAND => Some(8 + 8 * band_count as usize),
        KIND_POSITION => Some(40),
        KIND_HEARSAY => Some(32),
        _ => None,
    }
}

/// 1メッセージを buf 先頭へエンコードし、書いたバイト数を返す。
/// 容量不足なら Err(Full) を返し、**buf には一切書かない** (部分書き込み禁止)。
pub fn encode(buf: &mut [u8], hdr: &Header, payload: &Payload) -> Result<usize, WireError> {
    let (kind_ok, plen) = match payload {
        Payload::IfaceScalar { .. } => (KIND_IFACE_SCALAR, 16),
        Payload::IfaceBand { data, count } => {
            if data.len() != 8 * *count as usize {
                return Err(WireError::Truncated);
            }
            (KIND_IFACE_BAND, 8 + data.len())
        }
        Payload::Position { .. } => (KIND_POSITION, 40),
        Payload::Hearsay { .. } => (KIND_HEARSAY, 32),
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
        Payload::IfaceBand { data, count } => {
            put_u32(buf, p, *count);
            put_u32(buf, p + 4, 0);
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
    }
    Ok(total)
}

/// buf 先頭の1メッセージをデコードし、(ヘッダ, ペイロード, 消費バイト数) を返す
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
            let plen = 8 + 8 * count as usize;
            if buf.len() < p + plen {
                return Err(WireError::Truncated);
            }
            (
                Payload::IfaceBand { data: &buf[p + 8..p + plen], count },
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
        _ => return Err(WireError::BadKind),
    };
    Ok((hdr, payload, p + plen))
}

/// 帯ペイロードの i 番目の f64 を読む (アラインメント非依存)
pub fn band_get(data: &[u8], i: usize) -> f64 {
    get_f64(data, 8 * i)
}

// ---------- 受信箱/送信箱の枠 (固定長バッファへの詰め込み) ----------

/// 固定長バッファへ `[u32 len][メッセージ]` 形式でメッセージを詰める書き手。
/// あふれたメッセージは**破棄+計数** (2026-09-02 ユーザー決定の規約)。
pub struct MailboxWriter<'a> {
    buf: &'a mut [u8],
    used: usize,
    /// あふれで破棄したメッセージ数
    pub dropped: u64,
}

impl<'a> MailboxWriter<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        MailboxWriter { buf, used: 0, dropped: 0 }
    }

    /// 1メッセージを追記する。容量不足なら破棄して dropped を+1し Err(Full)。
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

    /// 詰め終わったバイト数 (終端の len=0 は含まない)
    pub fn used(&self) -> usize {
        self.used
    }

    /// 終端印 (len=0) を書けるなら書く。読み手は容量切れでも停止するので省略可
    pub fn finish(&mut self) -> usize {
        if self.used + 4 <= self.buf.len() {
            put_u32(self.buf, self.used, 0);
        }
        self.used
    }
}

/// 固定長バッファから `[u32 len][メッセージ]` 列を順に読む読み手
pub struct MailboxReader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> MailboxReader<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        MailboxReader { buf, pos: 0 }
    }
}

impl<'a> Iterator for MailboxReader<'a> {
    type Item = Result<(Header, Payload<'a>, &'a [u8]), WireError>;

    /// 次のメッセージ。終端 (len=0 または容量切れ) で None。
    /// Item の第3要素は生のメッセージバイト列 (転送・控えの保存用)
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
        let n = encode(&mut buf, &hdr, &Payload::IfaceBand { data: &data, count: 3 }).unwrap();
        assert_eq!(n, HDR_LEN + 8 + 24);
        let (h2, p2, _) = decode(&buf).unwrap();
        assert_eq!(h2, hdr);
        match p2 {
            Payload::IfaceBand { data, count } => {
                assert_eq!(count, 3);
                for (i, v) in vals.iter().enumerate() {
                    assert_eq!(band_get(data, i).to_bits(), v.to_bits());
                }
            }
            _ => panic!("kind mismatch"),
        }
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
}

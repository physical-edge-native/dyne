//! dyne-wire: ワイヤ形式v2の単一定義 (no_std)
//!
//! ロードマップ (docs/2026-09-01-roadmap.md 2.5) のワイヤ形式v2改訂
//! (時刻印・物理量種別・界面サンプル列ペイロード) と、発注6 (群れ移植) で追加する
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
//! - IFACE_BAND:   count u32, per_point u32, f64×(count×per_point)
//!   (界面サンプル列。count=界面上の点の数、per_point=1点あたりの量の数。
//!   並びは点ごとにまとめる: 点0の量0..per_point−1、点1の…。何の量を入れるかは
//!   アクターの型ごとの取り決め。per_point=0 は不正。2026-10-10 設計書D4で
//!   旧 pad 欄を per_point に充てた。それまで本種別の使用箇所は無い)
//! - POSITION:     x f64, y f64, temp f64, gx f64, gy f64 (計64バイト)
//! - HEARSAY:      x f64, y f64, score f64, age_steps u64 (計56バイト)
//! - RANGE_FRAME:  count u32, cols u32, count×(距離 f64 [m], 状態値 u32, 予備 u32)
//!   (測距フレーム。ファーム/シミュレータのホスト→センサのあるノード。src=センサID。
//!   状態値はセンサの値そのまま。行の選び方・有効の判定はアクターがする。2026-10-10)
//! - RAYS:         count u32, 予備 u32, count×40バイトの光線
//!   (光線束。ノード→隣のノード。src=界面ID。光線 = 原点 x,y f64、終点 x,y f64、
//!   取得ステップ u32、識別 u32 (センサID<<16 | 区画番号<<8 | 印)。2026-10-10、
//!   設計書 docs/2026-10-10-shape-grid-actor-design.md §3)
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
pub const KIND_RANGE_FRAME: u8 = 4;
pub const KIND_RAYS: u8 = 5;

/// 界面IDで運ぶ種別か (ヘッダの src が界面ID)。ホスト・ファームは、この種別の
/// メッセージを (界面ID, ステップ) ごとに1つの配送単位にまとめて隣へ運ぶ
pub fn routed_by_iface(kind: u8) -> bool {
    matches!(kind, KIND_IFACE_SCALAR | KIND_IFACE_BAND | KIND_RAYS)
}

/// 光線の識別欄の印: 対象なし (最大距離の内側に何も無かった区画)
pub const RAY_FLAG_NO_TARGET: u32 = 1;

/// 光線1本 (光線束の要素)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    pub ox: f64,
    pub oy: f64,
    pub ex: f64,
    pub ey: f64,
    pub acq_step: u32,
    /// センサID<<16 | 区画番号<<8 | 印 (RAY_FLAG_*)
    pub id: u32,
}

/// 光線1本のバイト数
pub const RAY_LEN: usize = 40;
/// 測距フレームの1区画のバイト数
pub const ZONE_LEN: usize = 16;

/// 区画 c (0..cols) の中心方向の、センサの正面からの角度 [rad] (反時計回りが正)。
/// 区画0が左端 (正の角度)。シミュレータの模擬センサとアクターが共有する単一定義
pub fn zone_angle_offset(c: u32, cols: u32, fov_rad: f64) -> f64 {
    fov_rad * (((cols as f64) - 1.0) * 0.5 - c as f64) / cols as f64
}

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
    /// 界面サンプル列。data は f64×(count×per_point) の生バイト列
    /// (アラインメント非依存で読む。i 番目は band_get)
    IfaceBand { data: &'a [u8], count: u32, per_point: u32 },
    /// 群れ: 位置・温度・勾配推定
    Position { x: f64, y: f64, temp: f64, gx: f64, gy: f64 },
    /// 群れ: 伝聞 (自分が知る最良地点)
    Hearsay { x: f64, y: f64, score: f64, age_steps: u64 },
    /// 測距フレーム。data は ZONE_LEN×count の生バイト列 (i 番目は range_zone)
    RangeFrame { data: &'a [u8], count: u32, cols: u32 },
    /// 光線束。data は RAY_LEN×count の生バイト列 (i 番目は ray_get)
    Rays { data: &'a [u8], count: u32 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WireError {
    /// バッファが短すぎる / ペイロード長が宣言と合わない
    Truncated,
    /// version 不一致
    BadVersion,
    /// 未知の kind
    BadKind,
    /// 欄の値が不正 (例: 界面サンプル列の per_point=0)
    BadField,
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

/// ペイロード長 (バイト)。可変長の種別は n に依存: IFACE_BAND は f64 の総数
/// (count×per_point)、RANGE_FRAME は区画数、RAYS は光線の数
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

/// 1メッセージを buf 先頭へエンコードし、書いたバイト数を返す。
/// 容量不足なら Err(Full) を返し、**buf には一切書かない** (部分書き込み禁止)。
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

/// 界面サンプル列の i 番目の f64 を読む (アラインメント非依存。点 k の量 q は i = k×per_point+q)
pub fn band_get(data: &[u8], i: usize) -> f64 {
    get_f64(data, 8 * i)
}

/// 測距フレームの i 番目の区画: (距離 [m], 状態値)
pub fn range_zone(data: &[u8], i: usize) -> (f64, u32) {
    (get_f64(data, ZONE_LEN * i), get_u32(data, ZONE_LEN * i + 8))
}

/// 測距フレームの i 番目の区画を書く (data は ZONE_LEN×count の領域)
pub fn range_zone_put(data: &mut [u8], i: usize, dist_m: f64, status: u32) {
    let o = ZONE_LEN * i;
    put_f64(data, o, dist_m);
    put_u32(data, o + 8, status);
    put_u32(data, o + 12, 0);
}

/// 光線束の i 番目の光線を読む
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

/// 光線束の i 番目に光線を書く (data は RAY_LEN×count の領域)
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
}

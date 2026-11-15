//! 実機の無線 (UDP) で運ぶための外箱 (2026-11-08)
//!
//! 設計書 docs/2026-10-10-order10-design.md の D5 (実機のメッセージ形式) の決定
//! 「実機もワイヤ形式v2にそろえ、v2の外側に通番とCRCを付ける」の実装。
//! 本モジュールの説明は docs/2026-11-08-firmware-wire-v2.md。
//!
//! v2 のメッセージそのもの (ヘッダ24バイト+ペイロード) には通番も誤り検出符号も
//! 無い。シミュレータの仮想ネットでは壊れたメッセージは起きないが、実機の無線では
//! 起きうるので、UDPの1データグラムを次の外箱で包む。中身は「1つの界面・1つの
//! ステップぶんの配送単位」= 枠つき v2 メッセージ列 `[u32 len][msg]...`
//! (終端の len=0 は含めない)。ホスト・ファームは中身を変換せず、アクターの
//! 送信メールボックスから出たバイト列をそのまま入れ、受信側はそのまま受信
//! メールボックスへ渡す。
//!
//! ## 外箱のレイアウト (リトルエンディアン)
//!
//! ```text
//! offset size field
//! 0      4    magic        = "DYN2" (バイト列 0x44 0x59 0x4E 0x32)
//! 4      1    env_version  = 1
//! 5      1    flags        = 0 (予約。0以外は不正)
//! 6      2    reserved     = 0
//! 8      4    sender       u32 送信元ノードID
//! 12     4    seq          u32 通番 (送信元がステップごとに+1。再送は同じ値)
//! 16     4    iface        u32 界面ID (中身のメッセージの src と一致すること)
//! 20     4    payload_len  u32 中身のバイト数 N
//! 24     8    step         u64 ステップ (中身のメッセージの step と一致すること)
//! 32     N    payload      枠つき v2 メッセージ列 (1通以上)
//! 32+N   4    crc32        IEEE CRC32 (offset 0 から 32+N−1 まで。既存の実機形式と同じ式)
//! ```
//!
//! 1データグラムの上限は 1472 バイト (イーサネットの1500バイトから IPv4 20 と
//! UDP 8 を引いた、分割されずに届く大きさ)。よって中身の上限は 1472−36 = 1436 バイト。
//! 入り切らない配送単位は**送らずに誤りとして返す** (分割送信は未対応。呼び出し側が
//! 計数・停止する)。受信側は上限を超える長さのデータグラムを読まずに捨てる。
//!
//! 外箱の欄 iface/step は中身のヘッダと重複するが、受け手が中身を解読せずに
//! 配送先と新旧を判定できるようにするため置く。デコードでは両者の一致を検査する。

use crate::{decode, routed_by_iface, WireError};

/// 外箱の識別子
pub const ENV_MAGIC: [u8; 4] = *b"DYN2";
/// 外箱の版
pub const ENV_VERSION: u8 = 1;
/// 外箱の前置部の長さ
pub const ENV_HDR_LEN: usize = 32;
/// 末尾の CRC32 の長さ
pub const ENV_CRC_LEN: usize = 4;
/// 1データグラムの上限 (分割されない大きさ)
pub const ENV_MAX_DATAGRAM: usize = 1472;
/// 中身の上限
pub const ENV_MAX_PAYLOAD: usize = ENV_MAX_DATAGRAM - ENV_HDR_LEN - ENV_CRC_LEN;

/// 外箱の前置部の可変欄
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EnvHeader {
    /// 送信元ノードID
    pub sender: u32,
    /// 通番
    pub seq: u32,
    /// 界面ID
    pub iface: u32,
    /// ステップ
    pub step: u64,
}

/// 外箱の誤り
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EnvError {
    /// 前置部+CRCに満たない / payload_len が実際の長さと合わない
    Truncated,
    /// 上限 (ENV_MAX_DATAGRAM / ENV_MAX_PAYLOAD) を超える
    Oversize,
    /// 書き込み先の容量不足
    Full,
    /// 識別子が "DYN2" でない
    BadMagic,
    /// 外箱の版の不一致、または予約欄が0でない
    BadVersion,
    /// CRC32 の不一致 (無線上での破損)
    BadCrc,
    /// 中身が配送単位として不正 (空、枠の不整合、v2として解読できない、
    /// 界面で運ばない種別、外箱の iface/step と中身の不一致)
    BadUnit(Option<WireError>),
}

/// IEEE 802.3 の CRC32 (反射、多項式 0xEDB88320、初期値・最終反転 0xFFFFFFFF)。
/// node-esp32 の dyne_crc32・crates/dyne-proto の crc32 と同じ式
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

/// 中身 (枠つき v2 メッセージ列) が iface/step の配送単位として正しいかを確かめる
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

/// 配送単位を外箱に包んで buf 先頭へ書き、データグラムのバイト数を返す。
/// 誤りのときは buf に一切書かない
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

/// 1データグラム (受信した全バイト) を検査して (前置部, 中身) を返す。
/// 検査の順: 長さ → 識別子 → 長さ欄 → CRC → 版・予約欄 → 中身
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

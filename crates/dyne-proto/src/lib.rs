//! dyne-proto: Boundary Sync Protocol のワイヤ形式 (M0版)
//!
//! docs/2026-08-04-dyne-spec-review.md A-4 のメッセージスキーマに従う。
//! 1メッセージ = 1境界面 × 1論理ステップ。壁時計は使わない。
//!
//! ワイヤレイアウト (リトルエンディアン, 固定長 40 バイト):
//!   magic:   u32  = 0x44594E30 ("DYN0")
//!   face_id: u16
//!   _pad:    u16  = 0
//!   step:    u64  論理タイムステップ
//!   seq:     u32  再送検出用の通し番号
//!   value:   f64  境界の物理量 (M0では温度)
//!   grad:    f64  境界の勾配 (M0では dT/dx)
//!   crc:     u32  上記全フィールドの CRC32 (IEEE)

pub const MAGIC: u32 = 0x4459_4E30;
pub const WIRE_LEN: usize = 40;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundaryMsg {
    pub face_id: u16,
    pub step: u64,
    pub seq: u32,
    pub value: f64,
    pub grad: f64,
}

impl BoundaryMsg {
    pub fn encode(&self) -> [u8; WIRE_LEN] {
        let mut b = [0u8; WIRE_LEN];
        b[0..4].copy_from_slice(&MAGIC.to_le_bytes());
        b[4..6].copy_from_slice(&self.face_id.to_le_bytes());
        // b[6..8] padding = 0
        b[8..16].copy_from_slice(&self.step.to_le_bytes());
        b[16..20].copy_from_slice(&self.seq.to_le_bytes());
        b[20..28].copy_from_slice(&self.value.to_le_bytes());
        b[28..36].copy_from_slice(&self.grad.to_le_bytes());
        let crc = crc32(&b[0..36]);
        b[36..40].copy_from_slice(&crc.to_le_bytes());
        b
    }

    pub fn decode(b: &[u8]) -> Result<Self, DecodeError> {
        if b.len() != WIRE_LEN {
            return Err(DecodeError::Length);
        }
        let magic = u32::from_le_bytes(b[0..4].try_into().unwrap());
        if magic != MAGIC {
            return Err(DecodeError::Magic);
        }
        let crc_expect = u32::from_le_bytes(b[36..40].try_into().unwrap());
        if crc32(&b[0..36]) != crc_expect {
            return Err(DecodeError::Crc);
        }
        Ok(BoundaryMsg {
            face_id: u16::from_le_bytes(b[4..6].try_into().unwrap()),
            step: u64::from_le_bytes(b[8..16].try_into().unwrap()),
            seq: u32::from_le_bytes(b[16..20].try_into().unwrap()),
            value: f64::from_le_bytes(b[20..28].try_into().unwrap()),
            grad: f64::from_le_bytes(b[28..36].try_into().unwrap()),
        })
    }
}

#[derive(Debug, PartialEq)]
pub enum DecodeError {
    Length,
    Magic,
    Crc,
}

/// CRC32 (IEEE 802.3, 反転あり)。依存を増やさないため自前実装。
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let m = BoundaryMsg { face_id: 1, step: 42, seq: 7, value: 0.5, grad: -3.25 };
        let b = m.encode();
        assert_eq!(BoundaryMsg::decode(&b).unwrap(), m);
    }

    #[test]
    fn corrupt_is_rejected() {
        let m = BoundaryMsg { face_id: 0, step: 1, seq: 1, value: 1.0, grad: 0.0 };
        let mut b = m.encode();
        b[21] ^= 0xFF;
        assert_eq!(BoundaryMsg::decode(&b), Err(DecodeError::Crc));
    }
}

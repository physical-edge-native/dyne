//! 仮想ネットワーク: 損失・遅延ジッタ・一時分断 (dyne-sim 従来実装の移設)。
//!
//! 乱数の消費順 (ビット互換の要): 分断判定 (乱数を消費しない) → 損失判定
//! (uniform を1回、loss>0 のときのみ) → 遅延ジッタ (range_u64 を1回)。
//! この順序を変更すると全回帰のビット一致が壊れる。

use crate::rng::Rng;

pub struct VirtNet {
    pub loss: f64,
    pub lat_us: u64,
    pub lat_jitter_us: u64,
    /// (split k, start_us, end_us): 期間中、境目 k をまたぐ通信を遮断
    /// (1次元チェーンでは「ノードk-1|k間」。判定は from<k と to<k の異同)
    pub partitions: Vec<(usize, u64, u64)>,
    pub rng: Rng,
}

impl VirtNet {
    pub fn new(
        loss: f64,
        lat_us: u64,
        lat_jitter_us: u64,
        partitions: Vec<(usize, u64, u64)>,
        seed: u64,
    ) -> Self {
        VirtNet { loss, lat_us, lat_jitter_us, partitions, rng: Rng::new(seed) }
    }

    /// 時刻 t に from→to が分断で遮られているか
    pub fn partition_blocks(&self, from: usize, to: usize, t: u64) -> bool {
        self.partitions
            .iter()
            .any(|&(k, a, b)| t >= a && t < b && ((from < k) != (to < k)))
    }

    /// 配送判定: None = 喪失 (分断または確率的ロス)、Some(t) = 配送時刻。
    /// 配送時刻は now + max(遅延+ジッタ, 1µs)。
    pub fn transit(&mut self, from: usize, to: usize, now: u64) -> Option<u64> {
        if self.partition_blocks(from, to, now) {
            return None;
        }
        if self.loss > 0.0 && self.rng.uniform() < self.loss {
            return None;
        }
        let lat = self.lat_us + self.rng.range_u64(self.lat_jitter_us);
        Some(now + lat.max(1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn partition_blocks_crossing_only() {
        let net = VirtNet::new(0.0, 0, 0, vec![(4, 100, 200)], 42);
        assert!(net.partition_blocks(3, 4, 150));
        assert!(net.partition_blocks(4, 3, 150));
        assert!(!net.partition_blocks(2, 3, 150)); // 境目をまたがない
        assert!(!net.partition_blocks(3, 4, 250)); // 期間外
    }

    #[test]
    fn transit_no_loss_min_latency() {
        let mut net = VirtNet::new(0.0, 0, 0, Vec::new(), 42);
        // 遅延0でも最低1µs (従来実装と同一)
        assert_eq!(net.transit(0, 1, 1000), Some(1001));
    }
}

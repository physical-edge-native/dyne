//! Virtual network: message loss, delay jitter and temporary partitions.
//!
//! Order in which random numbers are drawn (needed for determinism): partition check (no
//! random number), then loss (one `uniform`, only if `loss > 0`), then delay jitter (one
//! `range_u64`). Changing this order changes every regression result.

use crate::rng::Rng;

/// The virtual network between simulated devices.
pub struct VirtNet {
    /// Probability that a message is lost.
    pub loss: f64,
    /// Base delay \[µs\].
    pub lat_us: u64,
    /// Delay jitter \[µs\]: a uniform integer in 0..=2·jitter is added.
    pub lat_jitter_us: u64,
    /// Partitions (split k, start_us, end_us): during \[start, end), messages between devices
    /// below k and devices at or above k are blocked (in a 1D chain: between k−1 and k).
    pub partitions: Vec<(usize, u64, u64)>,
    /// The random generator of the network.
    pub rng: Rng,
}

impl VirtNet {
    /// A network with the given loss, delay, jitter, partitions and seed.
    pub fn new(
        loss: f64,
        lat_us: u64,
        lat_jitter_us: u64,
        partitions: Vec<(usize, u64, u64)>,
        seed: u64,
    ) -> Self {
        VirtNet { loss, lat_us, lat_jitter_us, partitions, rng: Rng::new(seed) }
    }

    /// Whether a message from `from` to `to` at time `t` is blocked by a partition.
    pub fn partition_blocks(&self, from: usize, to: usize, t: u64) -> bool {
        self.partitions
            .iter()
            .any(|&(k, a, b)| t >= a && t < b && ((from < k) != (to < k)))
    }

    /// Decide the fate of a message sent at `now`: `None` if lost (partition or random loss),
    /// otherwise `Some(arrival time)` = now + max(delay + jitter, 1 µs).
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

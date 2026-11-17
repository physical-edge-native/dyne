//! Deterministic random numbers (xorshift64*) and the rule for deriving per-device seeds.
//! Integer arithmetic only, so every platform produces the same sequence.

/// xorshift64* generator: the same sequence on every platform for a given seed.
pub struct Rng(u64);

impl Rng {
    /// A generator with `seed` (0 is replaced by 1).
    pub fn new(seed: u64) -> Self {
        Rng(seed.max(1))
    }
    /// The next 64-bit value.
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    /// Uniform value in \[0, 1) with 53-bit resolution.
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Uniform integer in 0..=2·hi (used for delay jitter).
    pub fn range_u64(&mut self, hi: u64) -> u64 {
        if hi == 0 { 0 } else { self.next_u64() % (hi * 2 + 1) }
    }
}

/// One step of splitmix64 (only for deriving seeds).
fn splitmix64(x: u64) -> u64 {
    let mut z = x.wrapping_add(0x9E3779B97F4A7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    z ^ (z >> 31)
}

/// Seed of device (or agent) `id`, derived from a global seed by a fixed rule:
///
/// ```text
/// seed_i = splitmix64( global_seed XOR (id+1) * 0x9E3779B97F4A7C15 )
/// (a result of 0 is replaced by 0x9E3779B97F4A7C15)
/// ```
///
/// - Integer arithmetic only; the same on every platform.
/// - `id + 1` keeps id 0 from passing the global seed through unchanged.
/// - splitmix64 removes the correlation between seeds of nearby ids.
/// - The derived seed is used with [`Rng::new`] or handed to a solver with `dyne_cfg`.
pub fn derive_seed(global_seed: u64, id: u64) -> u64 {
    let s = splitmix64(global_seed ^ (id.wrapping_add(1)).wrapping_mul(0x9E3779B97F4A7C15));
    if s == 0 { 0x9E3779B97F4A7C15 } else { s }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rng_bit_compat_first_values() {
        // 従来実装 (dyne-sim) と同一系列であることの固定ベクトル
        let mut r = Rng::new(42);
        assert_eq!(r.next_u64(), 6255019084209693600);
        assert_eq!(r.next_u64(), 14430073426741505498);
        let mut r2 = Rng::new(42);
        let u = r2.uniform();
        assert!((0.0..1.0).contains(&u));
        assert_eq!(u.to_bits(), 0x3FD5B392ADDC66E8);
    }

    #[test]
    fn derive_seed_deterministic_and_distinct() {
        let a = derive_seed(42, 0);
        let b = derive_seed(42, 1);
        let c = derive_seed(43, 0);
        assert_eq!(a, derive_seed(42, 0));
        assert_ne!(a, b);
        assert_ne!(a, c);
        assert_ne!(a, 0);
    }
}

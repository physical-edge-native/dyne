//! 決定論的乱数 (xorshift64*) と個体シードの導出規則。整数演算のみ。
//!
//! `Rng` は dyne-sim 従来実装の移設 (ビット互換。実装を変更してはならない)。

/// xorshift64*。シード固定で全プラットフォーム同一系列 (整数演算のみ)
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed.max(1))
    }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    /// [0,1) の一様乱数 (53ビット精度)
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// 0..=2*hi の一様整数 (遅延ジッタ用。従来実装と同一の意味論)
    pub fn range_u64(&mut self, hi: u64) -> u64 {
        if hi == 0 { 0 } else { self.next_u64() % (hi * 2 + 1) }
    }
}

/// splitmix64 の1ステップ (シード導出専用。Rng本体とは独立)
fn splitmix64(x: u64) -> u64 {
    let mut z = x.wrapping_add(0x9E3779B97F4A7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    z ^ (z >> 31)
}

/// 個体シードの導出規則 (2026-09-02 ユーザー決定の固定式):
///
/// ```text
/// seed_i = splitmix64( global_seed XOR (id+1) * 0x9E3779B97F4A7C15 )
/// (結果が0なら 0x9E3779B97F4A7C15 に置き換える)
/// ```
///
/// - 整数演算のみ。全プラットフォームで同一。
/// - id+1 とするのは id=0 で global_seed がそのまま通るのを避けるため。
/// - splitmix64 を挟むのは、近い id 同士のシードの相関 (xorshift系の弱点) を
///   切るため (慣用的な組み合わせ)。
/// - 導出されたシードを [`Rng::new`] に渡してアクターへ dyne_cfg で注入する。
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

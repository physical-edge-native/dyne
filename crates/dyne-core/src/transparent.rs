//! 半無限透過境界 (transparent) の指数和係数 (発注3 F1、2026-09-14)。
//!
//! 理論: docs/2026-09-13-transparent-boundary-derivation.md (理論ノートT10)。
//! 未観測領域を「境界の先に同じ材質が無限に続く」と想定すると、境界の流束は
//! 自ノードの境界温度の履歴のみから決まる (Dirichlet-to-Neumann写像、1/√t核の
//! 畳み込み)。1/√t 核を指数和で近似し、補助変数 (漏れバケツ) M個の漸化式
//!
//! ```text
//! y_j^{n+1} = e^{-λ_j Δt} · y_j^n + e^{-λ_j Δt/2} · Δg^n
//! 境界勾配 K^n = (1/√(πα)) Σ_j w_j y_j^n
//! ```
//!
//! で毎ステップ O(M) にする (導出 §5)。本モジュールは節点 λ_j と重み w_j の
//! 構成 (導出 §4: 対数軸の台形則) を行い、アクターへ渡す係数
//! (decay_j, half_j, weight_j) を返す。
//!
//! 決定論の規律 (検収仕様 2026-09-14):
//! - 構成パラメータは下記の定数に**固定** (実行ごとに変えない)。
//! - 数学関数は libm (純Rust) を使い、OS間でもビット一致を保つ。
//! - 総和順は j 昇順の逐次加算に固定する (アクター側・複合参照解側とも
//!   この順で Σ w_j y_j を計算すること)。
//!
//! 補助変数はノード内状態であり、可搬状態 u[] (モデルアーキテクチャノートE3)
//! には含めない。担当替え (handover / ADD に伴う再init) 時はリセット =
//! 透過境界の再発動として扱う (理論ノートT10、E3注記、2026-09-14決定)。

/// 対数軸の台形則の刻み。導出 §6 の表より、この値で M=32・核の相対誤差 7.7e-4。
pub const SOE_H: f64 = 0.6;
/// 下限側の裕度: λ_min = 1/(C1·T_REF)。大きいほど長時間側の打ち切り誤差が減る。
pub const SOE_C1: f64 = 1e9;
/// 上限側の裕度: λ_max = C2/Δt。短時間側 (t=Δt) の精度を決める。
pub const SOE_C2: f64 = 2e4;
/// 核近似の設計時間範囲の上限 [仮想秒]。既定シナリオ (2000ステップ×Δt=1e-4) の
/// T=0.2。これより長い実行でも、λ_min が 1/(C1·T_REF) と極小のため打ち切り誤差は
/// t=4·T_REF で相対 1e-4 未満に留まる (レポートの長時間実行 (iii) で実測確認)。
pub const SOE_T_REF: f64 = 0.2;
/// 項数の上限 (アクター側の静的確保と揃える)
pub const SOE_M_MAX: usize = 64;

/// 透過境界の指数和係数。j 昇順 (λ 昇順 = 遅い忘却が先)。
pub struct TransparentCoef {
    /// e^{-λ_j Δt} (補助変数の1ステップ減衰)
    pub decay: Vec<f64>,
    /// e^{-λ_j Δt/2} (当ステップの境界温度変化 Δg の中点補正)
    pub half: Vec<f64>,
    /// w_j / √(πα) (勾配への重み。1/√(πα) を畳み込み済み)
    pub weight: Vec<f64>,
}

/// 節点・重みの構成 (導出 §4・付録と同一の式)。
///   x_j = x_lo + j·h  (x_j < x_hi の範囲、j=0,1,…)
///   λ_j = e^{2 x_j},  w_j = (2/√π)·h·e^{x_j}
/// 既定 (Δt=1e-4、h=SOE_H=0.6) で M=32。
pub fn coefficients(alpha: f64, dt: f64) -> TransparentCoef {
    coefficients_with(alpha, dt, SOE_H)
}

/// 刻み h_soe を指定した構成 (実験用。導出 §6 の表: h=0.6→M=32、h=0.3→M=64)。
/// 既定運用は coefficients (h=SOE_H) を使うこと。
pub fn coefficients_with(alpha: f64, dt: f64, h_soe: f64) -> TransparentCoef {
    assert!(alpha > 0.0 && dt > 0.0 && h_soe > 0.0);
    let x_lo = 0.5 * libm::log(1.0 / (SOE_C1 * SOE_T_REF));
    let x_hi = 0.5 * libm::log(SOE_C2 / dt);
    let kfac = 1.0 / libm::sqrt(core::f64::consts::PI * alpha);
    let wfac = (2.0 / libm::sqrt(core::f64::consts::PI)) * h_soe;
    let mut c = TransparentCoef { decay: Vec::new(), half: Vec::new(), weight: Vec::new() };
    let mut j = 0usize;
    loop {
        let x = x_lo + (j as f64) * h_soe;
        if x >= x_hi {
            break;
        }
        let lam = libm::exp(2.0 * x);
        c.decay.push(libm::exp(-lam * dt));
        c.half.push(libm::exp(-lam * (0.5 * dt)));
        c.weight.push(wfac * libm::exp(x) * kfac);
        j += 1;
        assert!(j <= SOE_M_MAX, "transparent: 指数和の項数が上限 {SOE_M_MAX} を超えた (dt が小さすぎる)");
    }
    assert!(c.decay.len() >= 2, "transparent: 指数和の項数が少なすぎる (dt が大きすぎる)");
    c
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 既定シナリオ (Δt=1e-4) で項数が M=32 (導出 §6 の表の h=0.6 の行) になること
    #[test]
    fn m_is_32_for_default_dt() {
        let c = coefficients(1.0, 1e-4);
        assert_eq!(c.decay.len(), 32);
    }

    /// 核近似 Σ w_j e^{-λ_j t} ≈ 1/√t が設計範囲 [Δt, 2·T_REF] で相対誤差 1e-3 未満
    /// (導出 §6: h=0.6 で 7.7e-4)。重みには 1/√(πα) が畳み込まれているので戻して比較。
    #[test]
    fn kernel_accuracy() {
        let dt = 1e-4;
        let alpha = 1.0;
        let c = coefficients(alpha, dt);
        let x_lo = 0.5 * libm::log(1.0 / (SOE_C1 * SOE_T_REF));
        let kfac = 1.0 / libm::sqrt(core::f64::consts::PI * alpha);
        // λ_j を再構成して核を直接評価する
        let lams: Vec<f64> = (0..c.decay.len())
            .map(|j| libm::exp(2.0 * (x_lo + (j as f64) * SOE_H)))
            .collect();
        let mut t = dt;
        while t <= 2.0 * SOE_T_REF {
            let mut s = 0.0;
            for (j, lam) in lams.iter().enumerate() {
                s += (c.weight[j] / kfac) * libm::exp(-lam * t);
            }
            let exact = 1.0 / libm::sqrt(t);
            let rel = ((s - exact) / exact).abs();
            assert!(rel < 1e-3, "t={t:.3e}: rel={rel:.3e}");
            t *= 1.5;
        }
    }

    /// 補助変数の漸化式が直接畳み込み (O(n²)) と一致すること (導出 付録の移植)。
    /// g(t)=sin(20t)、2000ステップで相対差 (最大) < 1e-3 (M=48時 9.1e-6、M=32では緩む)。
    #[test]
    fn recurrence_matches_direct_convolution() {
        let dt = 1e-4;
        let n = 2000usize;
        let c = coefficients(1.0, dt);
        let m = c.decay.len();
        let g: Vec<f64> = (0..=n).map(|k| libm::sin(20.0 * (k as f64) * dt)).collect();
        let dg: Vec<f64> = (0..n).map(|k| g[k + 1] - g[k]).collect();
        // 直接畳み込み: F[k-1] = Σ_{j<k} dg[j] / √((k-j-0.5)Δt)
        let mut f_direct = vec![0.0f64; n];
        for k in 1..=n {
            let mut s = 0.0;
            for j in 0..k {
                s += dg[j] / libm::sqrt(((k - j) as f64 - 0.5) * dt);
            }
            f_direct[k - 1] = s;
        }
        // 漸化式 (重みから 1/√(πα)·(π)^{...} を戻す: weight = (2/√π)h e^x / √(πα)、
        // ここでは α=1 なので √π を掛ければ w_j = (2/√π)h e^x に戻る)
        let kfac = 1.0 / libm::sqrt(core::f64::consts::PI);
        let mut y = vec![0.0f64; m];
        let mut max_rel = 0.0f64;
        let fmax = f_direct.iter().fold(0.0f64, |a, &b| a.max(b.abs()));
        for k in 0..n {
            let mut s = 0.0;
            for j in 0..m {
                y[j] = c.decay[j] * y[j] + c.half[j] * dg[k];
                s += (c.weight[j] / kfac) * y[j];
            }
            max_rel = max_rel.max((s - f_direct[k]).abs() / fmax);
        }
        assert!(max_rel < 1e-3, "max_rel={max_rel:.3e}");
    }
}

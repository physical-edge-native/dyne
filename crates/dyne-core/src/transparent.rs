//! Coefficients for a transparent face: a boundary through which heat leaves as if the
//! same material continued without end beyond it.
//!
//! For the 1D heat equation, the flux through such a boundary depends only on the history
//! of the boundary temperature (a convolution with a 1/√t kernel). The kernel is
//! approximated by a sum of M exponentials, so that M auxiliary variables updated by
//!
//! ```text
//! y_j^{n+1} = e^{-λ_j Δt} · y_j^n + e^{-λ_j Δt/2} · Δg^n
//! boundary gradient K^n = (1/√(πα)) Σ_j w_j y_j^n
//! ```
//!
//! give the flux in O(M) per step. This module computes the nodes λ_j and weights w_j
//! (trapezoidal rule on a logarithmic axis) and returns the coefficients (decay_j, half_j,
//! weight_j) that the host passes to the solver.
//!
//! Determinism: the construction parameters are the fixed constants below; mathematical
//! functions come from pure-Rust `libm`; sums run over j in increasing order (solvers and
//! references must use the same order). The auxiliary variables belong to a device and
//! are reset when a region is handed to another device. Derivation (Japanese):
//! `docs/2026-09-13-transparent-boundary-derivation.md`.

/// Step of the trapezoidal rule on the logarithmic axis (M = 32 terms, relative kernel error
/// 7.7e-4 for the default scenario).
pub const SOE_H: f64 = 0.6;
/// Margin at the low end: λ_min = 1/(C1·T_REF); larger values reduce the truncation error
/// at long times.
pub const SOE_C1: f64 = 1e9;
/// Margin at the high end: λ_max = C2/Δt; sets the accuracy at short times (t = Δt).
pub const SOE_C2: f64 = 2e4;
/// Upper end \[virtual seconds\] of the time range the kernel approximation is designed for
/// (T = 0.2 for the default scenario of 2000 steps × Δt = 1e-4). Longer runs stay below a
/// relative truncation error of 1e-4 at t = 4·T_REF because λ_min is tiny.
pub const SOE_T_REF: f64 = 0.2;
/// Largest number of terms (matches the static arrays in the solvers).
pub const SOE_M_MAX: usize = 64;

/// Coefficients of a transparent face, in increasing j (increasing λ: slow forgetting first).
pub struct TransparentCoef {
    /// e^{-λ_j Δt}: decay of an auxiliary variable per step.
    pub decay: Vec<f64>,
    /// e^{-λ_j Δt/2}: midpoint factor for this step's change of boundary temperature Δg.
    pub half: Vec<f64>,
    /// w_j / √(πα): weight in the gradient (with 1/√(πα) folded in).
    pub weight: Vec<f64>,
}

/// Nodes and weights for diffusivity `alpha` and time step `dt`:
/// x_j = x_lo + j·h (while x_j < x_hi), λ_j = e^{2 x_j}, w_j = (2/√π)·h·e^{x_j}.
/// M = 32 for the defaults (Δt = 1e-4, h = [`SOE_H`] = 0.6).
pub fn coefficients(alpha: f64, dt: f64) -> TransparentCoef {
    coefficients_with(alpha, dt, SOE_H)
}

/// Like [`coefficients`] with a chosen step `h_soe` (for experiments: h = 0.6 gives M = 32,
/// h = 0.3 gives M = 64). Normal use is [`coefficients`].
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

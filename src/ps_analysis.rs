//! PS encoder analysis front end — ISO/IEC 14496-3:2009 Annex 8.C.6.2
//! over the §8.6.4.3 hybrid filterbank.
//!
//! The parametric stereo encoder observes the stereo input in the
//! same sub-subband domain the decoder mixes in: each channel's
//! 64-band complex QMF analysis ([`crate::sbr_qmf::EncoderAnalysisQmf`])
//! runs through the decoder-side [`PsHybrid`] filterbank (both
//! §8.6.4.3 configurations — 71 channels for 10/20 stereo bands, 91
//! for 34), on the Annex 8.A.3 `Xinput` framing (32 slots plus the
//! 6 look-ahead slots the 13-tap prototypes consume). The stereo cues
//! are then measured per stereo band `b` and parameter region from
//! the band energies and cross-correlations of Annex 8.C.6.2:
//!
//! ```text
//! e_l(b) = Σ_k Σ_n |l(k,n)|² + ε        e_r(b) = Σ_k Σ_n |r(k,n)|² + ε
//! e_R(b) = Σ_k Σ_n l(k,n)·r*(k,n) + ε   e_O(b) = Σ_k Σ_n l(k,n)·m*(k,n) + ε
//! iid(b) = 10·log10(e_l/e_r)   ρ(b) = e_R/√(e_l·e_r)
//! ipd(b) = ∠e_R                opd(b) = ∠e_O
//! ```
//!
//! with `m = (l + r)/2` (Annex 8.C.6.1) and `ε = 1e-10`. The
//! summation ranges over `k` are the Annex 8.C.6.2 Tables 8.C.2
//! (20 bands, 71 channels) and 8.C.3 (34 bands, 91 channels): they
//! cover only the positive-frequency sub-subbands of the split QMF
//! bands — the negative-frequency mirrors that the decoder's Table
//! 8.48/8.49 `b(k)` maps (conjugated) onto a stereo band are left out
//! of the estimate. The 10-band ranges are the union of each pair of
//! 20-band ranges (Table 8.45 applied additionally, as 8.C.6.2
//! prescribes).
//!
//! All truth from ISO/IEC 14496-3:2009 subpart 8 (Annexes 8.A / 8.C)
//! staged under `docs/audio/aac/`.

use crate::ps_hybrid::{HybridConfig, PsHybrid, LOOKAHEAD, NUM_QMF_SLOTS};
use crate::sbr_qmf::Complex;
use crate::{Error, Result};

/// `ε` of Annex 8.C.6.2.
pub const EPSILON: f64 = 1e-10;

/// Table 8.C.2 — summation range (inclusive hybrid channel bounds) per
/// stereo band, 20-band configuration.
const RANGES_20: [(usize, usize); 20] = [
    (2, 2),
    (3, 3),
    (4, 4),
    (5, 5),
    (6, 6),
    (7, 7),
    (8, 8),
    (9, 9),
    (10, 10),
    (11, 11),
    (12, 12),
    (13, 13),
    (14, 14),
    (15, 15),
    (16, 17),
    (18, 20),
    (21, 24),
    (25, 29),
    (30, 41),
    (42, 70),
];

/// Table 8.C.2 folded through Table 8.45 — the 10-band ranges are
/// the union of each 20-band pair.
const RANGES_10: [(usize, usize); 10] = [
    (2, 3),
    (4, 5),
    (6, 7),
    (8, 9),
    (10, 11),
    (12, 13),
    (14, 15),
    (16, 20),
    (21, 29),
    (30, 70),
];

/// Table 8.C.3 — summation range per stereo band, 34-band
/// configuration (91 hybrid channels).
const RANGES_34: [(usize, usize); 34] = [
    (0, 0),
    (1, 1),
    (2, 2),
    (3, 3),
    (4, 4),
    (5, 5),
    (16, 16),
    (17, 17),
    (18, 18),
    (19, 19),
    (20, 20),
    (21, 21),
    (26, 26),
    (27, 27),
    (28, 28),
    (29, 29),
    (32, 32),
    (33, 33),
    (34, 34),
    (35, 35),
    (36, 36),
    (37, 37),
    (38, 39),
    (40, 41),
    (42, 43),
    (44, 45),
    (46, 47),
    (48, 50),
    (51, 53),
    (54, 56),
    (57, 59),
    (60, 63),
    (64, 67),
    (68, 90),
];

/// The Annex 8.C.6.2 summation ranges for a parameter count of 10,
/// 20 or 34 (inclusive `(k_lo, k_hi)` per stereo band).
///
/// Returns [`Error::PsDataInvalid`] for any other count.
pub fn estimation_ranges(n_pars: usize) -> Result<&'static [(usize, usize)]> {
    match n_pars {
        10 => Ok(&RANGES_10),
        20 => Ok(&RANGES_20),
        34 => Ok(&RANGES_34),
        _ => Err(Error::PsDataInvalid),
    }
}

/// The hybrid configuration a parameter count lives in (Table 8.44:
/// 34 parameters need the 91-channel bank, 10 / 20 the 71-channel one).
#[must_use]
pub fn hybrid_config_for(n_pars: usize) -> HybridConfig {
    if n_pars == 34 {
        HybridConfig::Bands34
    } else {
        HybridConfig::Bands1020
    }
}

/// One stereo frame of both channels in the hybrid domain
/// (`NUM_QMF_SLOTS` rows of `nr_bands()` channels each).
#[derive(Debug, Clone, PartialEq)]
pub struct HybridStereo {
    /// Left channel `l(k, n)`.
    pub l: Vec<Vec<Complex>>,
    /// Right channel `r(k, n)`.
    pub r: Vec<Vec<Complex>>,
}

/// The stereo analysis front end: one [`PsHybrid`] per input channel,
/// threaded across frames.
#[derive(Debug, Clone)]
pub struct PsAnalysis {
    hybrid_l: PsHybrid,
    hybrid_r: PsHybrid,
}

impl PsAnalysis {
    /// A fresh front end in `config`.
    #[must_use]
    pub fn new(config: HybridConfig) -> Self {
        PsAnalysis {
            hybrid_l: PsHybrid::new(config),
            hybrid_r: PsHybrid::new(config),
        }
    }

    /// The active hybrid configuration.
    #[must_use]
    pub fn config(&self) -> HybridConfig {
        self.hybrid_l.config()
    }

    /// Switch configuration (resets both filter histories, as the
    /// decoder does on a Table 8.47 band-count switch).
    pub fn reset(&mut self, config: HybridConfig) {
        self.hybrid_l.reset(config);
        self.hybrid_r.reset(config);
    }

    /// Hybrid-analyse one stereo frame. `l` / `r` are the Annex 8.A.3
    /// `Xinput` matrices of each channel: at least
    /// `NUM_QMF_SLOTS + LOOKAHEAD` 64-band QMF slots.
    pub fn analyze(&mut self, l: &[[Complex; 64]], r: &[[Complex; 64]]) -> Result<HybridStereo> {
        if l.len() < NUM_QMF_SLOTS + LOOKAHEAD || r.len() < NUM_QMF_SLOTS + LOOKAHEAD {
            return Err(Error::PsDataInvalid);
        }
        Ok(HybridStereo {
            l: self.hybrid_l.analyze(l)?,
            r: self.hybrid_r.analyze(r)?,
        })
    }
}

/// The Annex 8.C.6.2 excitations of one stereo band over one
/// parameter region.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BandStats {
    /// `e_l(b)` — left excitation.
    pub el: f64,
    /// `e_r(b)` — right excitation.
    pub er: f64,
    /// `e_R(b)` — left × conj(right) cross excitation.
    pub e_lr: Complex,
    /// `e_O(b)` — left × conj(mono) cross excitation.
    pub e_lm: Complex,
}

impl Default for BandStats {
    fn default() -> Self {
        BandStats {
            el: EPSILON,
            er: EPSILON,
            e_lr: Complex::new(EPSILON, 0.0),
            e_lm: Complex::new(EPSILON, 0.0),
        }
    }
}

impl BandStats {
    /// `iid(b) = 10·log10(e_l / e_r)` in dB.
    #[must_use]
    pub fn iid_db(&self) -> f64 {
        10.0 * (self.el / self.er).log10()
    }

    /// The real-valued coherence `Re(e_R) / √(e_l·e_r)` — what
    /// mixing procedure Ra reproduces (the decoder's output
    /// cross-correlation equals `ρ` exactly for `d ⟂ s`); ranges
    /// over `[−1, 1]`, anti-phase content landing at `−1`.
    #[must_use]
    pub fn icc_real(&self) -> f64 {
        (self.e_lr.re / (self.el * self.er).sqrt()).clamp(-1.0, 1.0)
    }

    /// The magnitude coherence `|e_R| / √(e_l·e_r)` in `[0, 1]` —
    /// the phase-independent coherence used with IPD/OPD (procedure
    /// Rb), where the phase relation travels separately.
    #[must_use]
    pub fn icc_magnitude(&self) -> f64 {
        (self.e_lr.norm_sqr().sqrt() / (self.el * self.er).sqrt()).clamp(0.0, 1.0)
    }

    /// `ipd(b) = ∠e_R` (radians, four-quadrant).
    #[must_use]
    pub fn ipd(&self) -> f64 {
        self.e_lr.im.atan2(self.e_lr.re)
    }

    /// `opd(b) = ∠e_O` (radians, four-quadrant).
    #[must_use]
    pub fn opd(&self) -> f64 {
        self.e_lm.im.atan2(self.e_lm.re)
    }

    /// Total excitation `e_l + e_r` (the band's weight).
    #[must_use]
    pub fn energy(&self) -> f64 {
        self.el + self.er
    }
}

/// Annex 8.C.6.2 band statistics of `s` over slots `lo..=hi` for
/// `n_pars` (10 / 20 / 34) stereo bands.
///
/// `s` must be in the hybrid configuration the parameter count needs
/// ([`hybrid_config_for`]); the summation ranges are Tables
/// 8.C.2 / 8.C.3. Errors on a shape mismatch.
pub fn band_stats(s: &HybridStereo, n_pars: usize, lo: usize, hi: usize) -> Result<Vec<BandStats>> {
    let ranges = estimation_ranges(n_pars)?;
    let nr = hybrid_config_for(n_pars).nr_bands();
    if hi >= s.l.len() || hi >= s.r.len() || lo > hi {
        return Err(Error::PsDataInvalid);
    }
    if s.l.iter().chain(s.r.iter()).any(|row| row.len() != nr) {
        return Err(Error::PsDataInvalid);
    }
    let mut out = Vec::with_capacity(ranges.len());
    for &(k_lo, k_hi) in ranges {
        let mut st = BandStats::default();
        for n in lo..=hi {
            for k in k_lo..=k_hi {
                let l = s.l[n][k];
                let r = s.r[n][k];
                let m = (l + r) * 0.5;
                st.el += l.norm_sqr();
                st.er += r.norm_sqr();
                st.e_lr += l * r.conj();
                st.e_lm += l * m.conj();
            }
        }
        out.push(st);
    }
    Ok(out)
}

/// Per-slot total excitation `Σ_k |l|² + |r|²` over all hybrid
/// channels — the transient detector's input.
#[must_use]
pub fn slot_energies(s: &HybridStereo) -> Vec<f64> {
    s.l.iter()
        .zip(s.r.iter())
        .map(|(l, r)| {
            l.iter().map(|c| c.norm_sqr()).sum::<f64>()
                + r.iter().map(|c| c.norm_sqr()).sum::<f64>()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ps_map::{conjugate_flags, parameter_map};

    /// Every estimation range lies inside a single stereo band of the
    /// decoder's `b(k)` map and never touches a conjugate channel: the
    /// encoder measures exactly the sub-subbands the decoder will
    /// drive with that band's cues.
    #[test]
    fn ranges_agree_with_decoder_band_maps() {
        for n in [20usize, 34] {
            let cfg = hybrid_config_for(n);
            let b_k = parameter_map(cfg);
            let conj = conjugate_flags(cfg);
            let ranges = estimation_ranges(n).unwrap();
            assert_eq!(ranges.len(), n);
            for (b, &(lo, hi)) in ranges.iter().enumerate() {
                assert!(
                    lo <= hi && hi < cfg.nr_bands(),
                    "band {b} range {lo}..={hi}"
                );
                for (k, &bk) in b_k.iter().enumerate().take(hi + 1).skip(lo) {
                    assert_eq!(usize::from(bk), b, "{n} bands: channel {k} maps elsewhere");
                    assert!(
                        !conj.contains(&k),
                        "{n} bands: conjugate channel {k} in range"
                    );
                }
            }
            // Ranges are disjoint and ascending.
            for w in ranges.windows(2) {
                assert!(w[0].1 < w[1].0);
            }
        }
        // 10-band ranges are the pairwise unions of the 20-band ones.
        let r10 = estimation_ranges(10).unwrap();
        let r20 = estimation_ranges(20).unwrap();
        for (b, &(lo, hi)) in r10.iter().enumerate() {
            assert_eq!(lo, r20[2 * b].0);
            assert_eq!(hi, r20[2 * b + 1].1);
        }
        assert!(estimation_ranges(12).is_err());
    }

    fn frame(f: impl Fn(usize, usize) -> Complex) -> Vec<[Complex; 64]> {
        (0..NUM_QMF_SLOTS + LOOKAHEAD)
            .map(|n| {
                let mut s = [Complex::default(); 64];
                for (k, c) in s.iter_mut().enumerate() {
                    *c = f(n, k);
                }
                s
            })
            .collect()
    }

    fn noise(seed: u64) -> impl Fn(usize, usize) -> Complex {
        move |n, k| {
            let mut h = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add((n * 64 + k) as u64 + 1);
            h ^= h >> 33;
            h = h.wrapping_mul(0xff51afd7ed558ccd);
            h ^= h >> 33;
            let re = (h & 0xFFFF) as f64 / 65535.0 - 0.5;
            let im = ((h >> 16) & 0xFFFF) as f64 / 65535.0 - 0.5;
            Complex::new(re, im)
        }
    }

    /// Identical channels: IID 0 dB, ICC 1, IPD = OPD = 0 in every
    /// band of every configuration.
    #[test]
    fn identical_channels_are_centred_and_coherent() {
        for n in [10usize, 20, 34] {
            let mut fe = PsAnalysis::new(hybrid_config_for(n));
            let sig = noise(3);
            let x = frame(&sig);
            // Warm the hybrid history with one frame, then measure.
            fe.analyze(&x, &x).unwrap();
            let s = fe.analyze(&x, &x).unwrap();
            let st = band_stats(&s, n, 0, NUM_QMF_SLOTS - 1).unwrap();
            assert_eq!(st.len(), n);
            for (b, v) in st.iter().enumerate() {
                assert!(
                    v.iid_db().abs() < 1e-9,
                    "{n} bands: band {b} iid {}",
                    v.iid_db()
                );
                assert!((v.icc_real() - 1.0).abs() < 1e-9);
                assert!((v.icc_magnitude() - 1.0).abs() < 1e-9);
                assert!(v.ipd().abs() < 1e-9);
                assert!(v.opd().abs() < 1e-9);
            }
        }
    }

    /// A right channel scaled by −6.02 dB and delayed in phase: IID
    /// tracks the level ratio, IPD the rotation, the magnitude
    /// coherence stays 1 while the real coherence follows cos(φ).
    #[test]
    fn scaled_rotated_right_channel() {
        let mut fe = PsAnalysis::new(HybridConfig::Bands1020);
        let sig = noise(9);
        let phi = core::f64::consts::FRAC_PI_3;
        let rot = Complex::new(phi.cos(), phi.sin());
        let l = frame(&sig);
        let r = frame(|n, k| sig(n, k) * rot * 0.5);
        fe.analyze(&l, &r).unwrap();
        let s = fe.analyze(&l, &r).unwrap();
        let st = band_stats(&s, 20, 0, NUM_QMF_SLOTS - 1).unwrap();
        for v in &st {
            assert!((v.iid_db() - 6.0206).abs() < 1e-3, "iid {}", v.iid_db());
            assert!((v.icc_magnitude() - 1.0).abs() < 1e-9);
            assert!((v.icc_real() - phi.cos()).abs() < 1e-9);
            // l·r* = |l|²·conj(rot)/2 → ∠ = −φ.
            assert!((v.ipd() + phi).abs() < 1e-9, "ipd {}", v.ipd());
        }
    }

    /// Independent channels of equal level: ICC near zero, IID near
    /// 0 dB (statistically), in the wide upper bands.
    #[test]
    fn independent_channels_are_incoherent() {
        let mut fe = PsAnalysis::new(HybridConfig::Bands1020);
        let l = frame(noise(21));
        let r = frame(noise(22));
        fe.analyze(&l, &r).unwrap();
        let s = fe.analyze(&l, &r).unwrap();
        let st = band_stats(&s, 20, 0, NUM_QMF_SLOTS - 1).unwrap();
        // Band 19 sums 29 channels × 32 slots — a solid estimate.
        let v = &st[19];
        assert!(v.icc_real().abs() < 0.1, "icc {}", v.icc_real());
        assert!(v.iid_db().abs() < 1.0, "iid {}", v.iid_db());
    }

    /// A short region and a shape mismatch are rejected.
    #[test]
    fn shape_checks() {
        let mut fe = PsAnalysis::new(HybridConfig::Bands1020);
        let x = frame(noise(1));
        let s = fe.analyze(&x, &x).unwrap();
        assert!(band_stats(&s, 34, 0, 31).is_err());
        assert!(band_stats(&s, 20, 5, 4).is_err());
        assert!(band_stats(&s, 20, 0, 32).is_err());
        let short: Vec<[Complex; 64]> = x[..NUM_QMF_SLOTS].to_vec();
        assert!(fe.analyze(&short, &x).is_err());
        assert_eq!(slot_energies(&s).len(), NUM_QMF_SLOTS);
    }
}

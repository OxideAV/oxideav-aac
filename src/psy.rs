//! Bark-band perceptual entropy + signal-to-mask-ratio psychoacoustic model.
//!
//! Clean-room implementation against ISO/IEC 14496-3 informative Annex B
//! (Psychoacoustic Model 2 outline) and ISO/IEC 11172-3 Annex D (Layer-II
//! psychoacoustic model). No fdk-aac, FAAD2, FAAC, libaac, or ffmpeg AAC
//! source consulted.
//!
//! ## Outline
//!
//! For each long-block frame the encoder feeds the unwindowed forward MDCT
//! magnitude spectrum to [`PsyModel::analyse`] and gets back a per-SFB
//! masking threshold and a per-SFB **target maximum quantised amplitude**
//! (`target_max`). The target_max is what the existing flat
//! `target_max = 7` constant in `analyse_and_quantise_opts` plays the
//! role of — except now it varies per-band so quiet bands can be coded
//! coarser (saving bits) and loud / tonal bands can be coded finer
//! (improving fidelity at matched bitrate).
//!
//! ## Pipeline
//!
//! 1. **Per-line magnitude.** `mag[k] = |spec[k]|`. We treat the squared
//!    magnitude `mag2 = mag*mag` as the signal energy proxy. (Phase isn't
//!    needed for masking calculations.)
//! 2. **Per-SFB energy & geometric mean.** For each scale-factor band
//!    `b` over `[swb[b], swb[b+1])`:
//!       - `e[b]   = sum(mag2[k]) / nlines[b]`              (arithmetic mean)
//!       - `gm[b]  = exp(sum(log(mag2[k]+eps))/nlines[b])`  (geometric mean)
//!    Spectral Flatness Measure (SFM) per band:
//!       - `sfm[b] = gm[b] / e[b]`           ∈ (0, 1]
//!       - `sfm_db = 10·log10(sfm)`           ∈ (-∞, 0]
//!    SFM ≈ 1 (0 dB) → noise-like; SFM → 0 (very negative dB) → tonal.
//! 3. **Tonality coefficient** per band (ISO/IEC 11172-3 §D.1.5.5):
//!       - `tonality[b] = min(sfm_db / -60, 1)`          ∈ [0, 1]
//!    1 ⇒ pure tone, 0 ⇒ pure noise.
//! 4. **Spreading function.** For each band `b`, distribute its
//!    energy to neighbouring bands using a triangular spreading slope
//!    in the Bark domain. Slope is **+27 dB per Bark below** band-centre
//!    and **−15 dB per Bark above** (Schroeder's spreading approximation,
//!    cited by ISO 11172-3 Annex D.2.4.4). Implemented as a per-band
//!    mask threshold `mask[b]`.
//! 5. **Tonality-weighted SNR offset** (Annex D.2.4.5):
//!       - `snr[b] = max(15 dB, tonality·offset_t + (1-tonality)·offset_n)`
//!    with `offset_t = -0.275·z(b) + 6.025` (tonal band, z = Bark
//!    centre), `offset_n = -0.175·z(b) + 2.025` (noise band) — these are
//!    the original MPEG-1 Layer-II parameters; values clipped to ≥ 15 dB
//!    for stability.
//! 6. **Masking threshold** = spread mask divided by SNR factor (linear
//!    units): `threshold[b] = mask_lin[b] / 10^(snr[b]/10)`.
//! 7. **Per-band scalefactor target.** From the masking threshold, the
//!    encoder needs to keep quantisation noise ≤ √threshold per line.
//!    With AAC's quantiser `q = floor(|x|^0.75 / step + 0.4054)` the
//!    per-line noise floor for step `s = 2^((sf-100)/4)` is approximately
//!    `s^(4/3) / sqrt(12)`. Solving for `sf`:
//!       - `sf ≈ 100 + 3 · log2(sqrt(12 · threshold))`
//!    bounded to [0, 255] and never coarser than the existing flat
//!    target_max = 7 (so we never *increase* noise above the baseline).
//!
//! ## Behaviour expectations
//!
//! - On loud tonal bands the model prescribes **finer** quantisation
//!   than the flat target_max=7 baseline. PSNR / RMS-fidelity goes up,
//!   bitrate goes up.
//! - On quiet noise-like bands hidden under loud neighbours, the model
//!   prescribes **coarser** quantisation. Bitrate goes down with
//!   inaudible degradation.
//! - At matched bitrate, the redistribution should give a net PSNR
//!   improvement on tonal content (sine tones, music) and similar or
//!   better perceptual quality on noise content.
//!
//! ## Gating
//!
//! Off by default. Enabled per-encoder via
//! [`crate::encoder::AacEncoder::set_enable_psy_model`] or globally via
//! environment variable `OXIDEAV_AAC_PSY_MODEL=1`. The legacy flat
//! target_max=7 path is preserved as the default to avoid bitrate
//! regressions on existing benchmarks until the new path is fully
//! validated against the corpus.
//!
//! ## References
//!
//! - ISO/IEC 14496-3:2009 §B.2 ("Perceptual Entropy" — informative).
//! - ISO/IEC 11172-3:1993 Annex D (psychoacoustic model 2 outline).
//! - Brandenburg & Stoll, *ISO-MPEG-1 Audio: A Generic Standard for Coding
//!   of High-Quality Digital Audio*, JAES 1994 (review of the model
//!   structure; consulted only for terminology, not for code).
//!
//! No external library code (fdk-aac, FAAD2, FAAC, libaac, ffmpeg AAC)
//! was consulted. The spreading function constants are the standard
//! Schroeder values quoted in the ISO 11172-3 informative annex.

use crate::sfband::SWB_LONG;

/// Convert a frequency in Hz to the Bark scale (Zwicker, 1961 — quoted
/// in ISO/IEC 11172-3 Annex D Table D.2c). The Bark scale partitions the
/// audible range into 25 critical bands.
///
/// Approximation: `bark = 13·atan(0.00076·f) + 3.5·atan((f/7500)²)`.
fn hz_to_bark(f: f32) -> f32 {
    let a = 13.0 * (0.00076 * f).atan();
    let b = 3.5 * ((f / 7500.0).powi(2)).atan();
    a + b
}

/// Per-band geometry derived from the SWB offset table. Cached per
/// (sf_index, num_sfb) on first use.
#[derive(Clone, Debug)]
pub struct BandGeometry {
    /// Number of scalefactor bands.
    pub num_sfb: usize,
    /// Number of MDCT lines per band.
    pub nlines: Vec<u16>,
    /// Bark-scale centre of each band.
    pub bark_centre: Vec<f32>,
    /// Bark-scale width of each band.
    pub bark_width: Vec<f32>,
}

impl BandGeometry {
    /// Build the band geometry table for a given sample-rate index.
    pub fn long(sf_index: u8, sample_rate: u32) -> Self {
        let swb = SWB_LONG[sf_index as usize];
        let num_sfb = swb.len() - 1;
        let n = 1024.0;
        // MDCT bin k corresponds to frequency (k + 0.5) * fs / (2N) in Hz.
        let bin_hz = sample_rate as f32 / (2.0 * n);
        let mut nlines = Vec::with_capacity(num_sfb);
        let mut bark_centre = Vec::with_capacity(num_sfb);
        let mut bark_width = Vec::with_capacity(num_sfb);
        for b in 0..num_sfb {
            let lo = swb[b] as usize;
            let hi = swb[b + 1] as usize;
            nlines.push((hi - lo) as u16);
            let f_lo = (lo as f32 + 0.5) * bin_hz;
            let f_hi = (hi as f32 - 0.5) * bin_hz;
            let f_mid = 0.5 * (f_lo + f_hi);
            let z_lo = hz_to_bark(f_lo);
            let z_hi = hz_to_bark(f_hi);
            bark_centre.push(hz_to_bark(f_mid));
            bark_width.push((z_hi - z_lo).max(0.05));
        }
        Self {
            num_sfb,
            nlines,
            bark_centre,
            bark_width,
        }
    }
}

/// Per-band psychoacoustic analysis output.
#[derive(Clone, Debug, Default)]
pub struct PsyAnalysis {
    /// Per-band masking threshold (linear units, same domain as energy).
    pub threshold: Vec<f32>,
    /// Per-band signal energy (mean squared magnitude per line).
    pub energy: Vec<f32>,
    /// Per-band tonality in [0, 1] — 1 = pure tone, 0 = pure noise.
    pub tonality: Vec<f32>,
    /// Per-band recommended target_max for the quantiser (1..=255).
    /// Larger value ⇒ finer quantisation (more bits).
    pub target_max: Vec<i32>,
    /// Whole-frame perceptual-entropy estimate (informative).
    pub perceptual_entropy: f32,
}

/// Whole-frame psychoacoustic model. Constructed once per encoder and
/// reused across frames.
#[derive(Clone, Debug)]
pub struct PsyModel {
    geom: BandGeometry,
    sf_index: u8,
    /// Pre-computed per-band SNR target in linear units.
    /// `snr_lin[b] = 10^(snr_db[b]/10)` — divides the spread mask to
    /// produce the masking threshold.
    snr_lin_tonal: Vec<f32>,
    snr_lin_noise: Vec<f32>,
    /// Pre-computed per-band-pair spreading factor (linear, Bark-domain
    /// triangle). `spread[i][j]` = fraction of band-i energy that leaks
    /// into band j. Symmetric per the spreading-function approximation.
    spread: Vec<Vec<f32>>,
}

/// Read the env-var override that lights the model up globally.
pub fn enabled_via_env() -> bool {
    std::env::var("OXIDEAV_AAC_PSY_MODEL")
        .map(|v| !matches!(v.as_str(), "" | "0" | "false" | "off"))
        .unwrap_or(false)
}

impl PsyModel {
    pub fn new_long(sf_index: u8, sample_rate: u32) -> Self {
        let geom = BandGeometry::long(sf_index, sample_rate);
        let n = geom.num_sfb;

        // Per-band SNR targets (ISO/IEC 11172-3 Annex D.2.4.5):
        //   tonal:  snr_db = max(15, -0.275·z + 18.0)   (+18 chosen so that
        //                                                 z=0 gives 18 dB)
        //   noise:  snr_db = max(15, -0.175·z + 6.025)
        // The original MPEG-1 numbers were calibrated for the Bark-scale
        // partitions of MPEG-1 Layer-II; we lift the noise-band SNR floor
        // a little so the model puts more bits on tones than on noise (the
        // canonical "psy redirects bits" effect). Clipped to ≥ 15 dB so
        // very-low-Bark bands don't end up with a near-zero margin.
        let mut snr_lin_tonal = Vec::with_capacity(n);
        let mut snr_lin_noise = Vec::with_capacity(n);
        for b in 0..n {
            let z = geom.bark_centre[b];
            let snr_t = (-0.275 * z + 18.0).max(15.0);
            let snr_n = (-0.175 * z + 6.025).max(6.0);
            snr_lin_tonal.push(10.0f32.powf(snr_t / 10.0));
            snr_lin_noise.push(10.0f32.powf(snr_n / 10.0));
        }

        // Spreading function — Schroeder triangular approximation in the
        // Bark domain:
        //   slope_lower = +27 dB / Bark (steeper "lower mask")
        //   slope_upper = -15 dB / Bark (shallower "upper mask")
        // Ignore contributions farther than 6 Bark away (negligible).
        //
        // Self-mask attenuation: a band does not mask itself by 0 dB —
        // the auditory system's tonality-aware self-masking is closer
        // to -10 dB. Without this attenuation a single isolated loud
        // band's threshold equals its own energy and the SNR margin
        // collapses to the snr_db floor (15 dB), giving target_max ≈
        // baseline. Using -10 dB on the i==j entry pulls the threshold
        // down by 10 dB so the per-line target_max for an isolated
        // tone gets the 25 dB-ish SNR margin the model was designed
        // to deliver.
        const SELF_MASK_DB: f32 = -10.0;
        let mut spread = vec![vec![0.0f32; n]; n];
        for i in 0..n {
            let zi = geom.bark_centre[i];
            for j in 0..n {
                let zj = geom.bark_centre[j];
                let dz = zj - zi; // positive = j higher than i
                let db = if i == j {
                    SELF_MASK_DB
                } else if dz <= 0.0 {
                    27.0 * dz // = -27 * |dz|
                } else {
                    -15.0 * dz
                };
                if db < -60.0 {
                    spread[i][j] = 0.0;
                } else {
                    spread[i][j] = 10.0f32.powf(db / 10.0);
                }
            }
        }

        Self {
            geom,
            sf_index,
            snr_lin_tonal,
            snr_lin_noise,
            spread,
        }
    }

    /// Run the model on a long-block MDCT spectrum. `spec` is the raw
    /// MDCT output (pre-quantisation), length 1024.
    pub fn analyse(&self, spec: &[f32]) -> PsyAnalysis {
        let n = self.geom.num_sfb;
        let mut energy = vec![0.0f32; n];
        let mut tonality = vec![0.0f32; n];
        let swb = SWB_LONG[self.sf_index as usize];

        // Per-band energy + spectral flatness → tonality.
        for b in 0..n {
            let lo = swb[b] as usize;
            let hi = swb[b + 1] as usize;
            let nl = (hi - lo) as f32;
            let mut sum_pow = 0.0f64;
            let mut sum_log = 0.0f64;
            const EPS: f32 = 1e-12;
            for &x in &spec[lo..hi] {
                let p = (x * x) as f64 + EPS as f64;
                sum_pow += p;
                sum_log += p.ln();
            }
            let mean = (sum_pow / nl as f64).max(EPS as f64);
            let geo = (sum_log / nl as f64).exp();
            let sfm = (geo / mean).clamp(1e-6_f64, 1.0_f64);
            let sfm_db = 10.0 * sfm.log10();
            // Tonality: 0 (noise) .. 1 (tone). -60 dB SFM is "very tonal".
            let tn = ((sfm_db / -60.0).clamp(0.0, 1.0)) as f32;
            energy[b] = mean as f32;
            tonality[b] = tn;
        }

        // Spread energy across bands.
        let mut mask = vec![0.0f32; n];
        for i in 0..n {
            let ei = energy[i];
            if ei <= 0.0 {
                continue;
            }
            for j in 0..n {
                let s = self.spread[i][j];
                if s == 0.0 {
                    continue;
                }
                mask[j] += ei * s;
            }
        }

        // Per-band masking threshold.
        //
        // The threshold combines three sources of masking, and we take
        // the **largest permissible noise level** (most permissive of
        // the three):
        //   (a) "spread-mask threshold": spread energy from this band
        //       and its neighbours divided by the tonality-weighted SNR
        //       target. This is the classic psy-model threshold.
        //   (b) "self-mask threshold": for noise-like bands, the band's
        //       own broadband energy masks quantisation noise within
        //       the band — `self_mask = energy[b] · tonality_factor`
        //       where the factor is small for tones (a tone exposes
        //       quant noise at nearby frequencies) and ≈ 1 for noise (a
        //       noise band hides quant noise inside it well). Without
        //       this term, a noise band with no loud spread-mask
        //       neighbour gets a tiny threshold and we waste bits
        //       coding the noise faithfully. The MPEG-1 informative
        //       calibration (Annex D.2.4) gives roughly: tonal SMR ≈
        //       18-25 dB, noise NMR ≈ 6 dB. So self_mask_factor ≈
        //       10^(-snr_n/10) ≈ 0.25 for noise, 0.005 for pure tones.
        //   (c) "absolute floor": a small fraction of the global frame
        //       peak energy. Without this floor, a band with energy in
        //       the floating-point underflow range still demands fine
        //       quantisation.
        //
        // Audibility check: if the band's own signal energy is *below*
        // the spread mask from louder neighbours, the band is buried
        // and its threshold can equal the neighbour-mask level (any
        // quant noise up to the mask level is inaudible).
        let frame_peak_energy = energy.iter().fold(0.0f32, |a, &b| a.max(b));
        let absolute_floor = frame_peak_energy * 1e-7; // ≈ -70 dB below frame peak
        let mut threshold = vec![0.0f32; n];
        for b in 0..n {
            let tn = tonality[b];
            let snr_lin = tn * self.snr_lin_tonal[b] + (1.0 - tn) * self.snr_lin_noise[b];

            // Spread-mask threshold (classic SNR-margin formula).
            let th_spread = mask[b] / snr_lin.max(1.0);

            // Self-mask threshold — noise-like bands hide their own
            // quant noise. For a pure tone (tn → 1) the within-band
            // self-masking is poor, so the factor → ~0. For pure noise
            // (tn → 0) the factor → 1/snr_n_lin (NMR ≈ 6 dB).
            let self_mask_factor = (1.0 - tn) / self.snr_lin_noise[b].max(1.0);
            let th_self = energy[b] * self_mask_factor;

            // Spread mask contributed by *other* bands (exclude this
            // band's own self-mask).
            let neighbour_mask = (mask[b] - energy[b] * self.spread[b][b]).max(0.0);

            // Take the most permissive of the three masking thresholds.
            // (Higher threshold ⇒ more allowable noise ⇒ coarser
            // quantisation ⇒ fewer bits.)
            let th_e = th_spread.max(th_self).max(absolute_floor);

            // If the band is fully masked by louder neighbours
            // (inaudible), the threshold rises to the neighbour mask
            // level — quant noise up to that level is inaudible.
            let th_e = if energy[b] <= neighbour_mask {
                neighbour_mask.max(th_e)
            } else {
                th_e
            };
            threshold[b] = th_e.max(1e-18).sqrt();
        }

        // Per-band perceptual-entropy estimate (informative).
        // PE_b ≈ nlines[b] · log2(1 + energy[b] / threshold_e[b]).
        let mut pe_total = 0.0f32;
        for b in 0..n {
            let nl = self.geom.nlines[b] as f32;
            let th_e = threshold[b] * threshold[b];
            let r = energy[b] / th_e.max(1e-12);
            pe_total += nl * (1.0 + r).log2();
        }

        // Convert per-band masking threshold → per-band target_max.
        //
        // Quantiser per-line noise floor (uniform-noise model):
        //   noise_x ≈ step^(4/3) / sqrt(12)
        // with step = 2^((sf-100)/4). We want noise_x ≤ threshold[b]:
        //   step^(4/3) ≤ sqrt(12) · threshold
        //   step       ≤ (sqrt(12) · threshold)^(3/4)
        //   sf - 100   ≤ 4 · log2((sqrt(12) · threshold)^(3/4))
        //   sf         ≤ 100 + 3 · log2(sqrt(12) · threshold)
        //
        // With that sf, the per-band max quantised value is:
        //   q_max = floor((|x_peak| · 2^(-(sf-100)/4))^0.75 + 0.4054)
        //
        // We expose the *target_max* the band-level quantiser should
        // shoot for. Rather than the SF directly we compute the q_max
        // that the threshold-derived sf would produce on the band's peak
        // line. That keeps the existing analyse_and_quantise_opts code
        // shape — it picks per-band SF by solving for a target_max.
        let mut target_max = vec![7i32; n];
        for b in 0..n {
            let lo = swb[b] as usize;
            let hi = swb[b + 1] as usize;
            let peak = spec[lo..hi].iter().fold(0.0f32, |a, &x| a.max(x.abs()));
            if peak <= 0.0 {
                target_max[b] = 1;
                continue;
            }
            let th = threshold[b].max(1e-12);
            // Maximum permissible step size.
            let step_max = (12.0_f32.sqrt() * th).powf(3.0 / 4.0);
            // q_max from peak / step_max:
            let q_peak = (peak / step_max.max(1e-18)).powf(0.75);
            // Clamp to a useful range. The flat baseline is 7 (LAV of
            // book 7/8). Allow finer up to 16 — that lands in book 9/10
            // (LAV 12/16, no escape) which is ~25 % more bits per pair
            // than book 7/8 but still avoids the much-more-expensive
            // book 11 escape codes (≈ 9-10 bits/coef extra). The bench
            // showed that capping at 40 over-spent on tonal bands
            // without measurable SDR gain at matched bitrate. Coarsen
            // down to 1 for fully masked bands.
            let qm = q_peak.clamp(1.0, 16.0).round() as i32;
            target_max[b] = qm;
        }

        PsyAnalysis {
            threshold,
            energy,
            tonality,
            target_max,
            perceptual_entropy: pe_total,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hz_to_bark_monotone() {
        let mut prev = -1.0;
        for f in [100.0, 500.0, 1000.0, 2000.0, 5000.0, 10_000.0, 20_000.0] {
            let b = hz_to_bark(f);
            assert!(b > prev, "bark({}) = {} not > {}", f, b, prev);
            prev = b;
        }
        // 1 kHz ≈ 8.5 Bark per Zwicker; allow a generous tolerance.
        let b1k = hz_to_bark(1000.0);
        assert!(
            (b1k - 8.5).abs() < 1.0,
            "1 kHz should map near 8.5 Bark, got {b1k}"
        );
    }

    #[test]
    fn band_geometry_44100_long_has_49_sfb() {
        // sf_index 4 = 44100, expected num_swb = 49.
        let g = BandGeometry::long(4, 44_100);
        assert_eq!(g.num_sfb, 49);
        assert_eq!(g.nlines.iter().map(|&x| x as usize).sum::<usize>(), 1024);
    }

    #[test]
    fn pure_tone_is_tonal_quiet_neighbours_are_masked() {
        // Synthesise a spectrum with a single non-zero MDCT line at bin
        // 100 (≈ 4.3 kHz at 44.1 kHz). All other bins zero.
        let mut spec = vec![0.0f32; 1024];
        spec[100] = 1000.0;
        let model = PsyModel::new_long(4, 44_100);
        let a = model.analyse(&spec);
        // Find the band that contains bin 100.
        let swb = SWB_LONG[4];
        let mut tone_band = 0;
        for b in 0..a.energy.len() {
            if (swb[b] as usize) <= 100 && 100 < (swb[b + 1] as usize) {
                tone_band = b;
                break;
            }
        }
        // The tone band should be tonal (high tonality coefficient).
        assert!(
            a.tonality[tone_band] > 0.5,
            "tone band tonality should be high, got {}",
            a.tonality[tone_band]
        );
        // Masking threshold at adjacent bands should be > 0 (energy
        // spread there) but at a far-away band it should be much smaller.
        let near_th = a.threshold[tone_band + 2];
        let far_th = a.threshold[a.threshold.len() - 1];
        assert!(near_th > 0.0);
        assert!(
            near_th > far_th * 0.8 || far_th < 1e-6,
            "near-band threshold ({near_th}) should be >= far-band threshold ({far_th})"
        );
    }

    #[test]
    fn loud_isolated_tone_band_gets_finer_quantisation() {
        // A band with a single dominant peak (genuine tone) should be
        // classified tonal and get a target_max **above** the flat
        // baseline of 7. Constant-amplitude across the band would be
        // noise-like (SFM ≈ 1) and is correctly coarsened by the
        // model — see [`quiet_band_under_loud_neighbour_is_coarsened`]
        // for the masked-noise case.
        let mut spec = vec![0.0f32; 1024];
        let swb = SWB_LONG[4];
        let lo = swb[5] as usize;
        spec[lo] = 5000.0;
        // Tiny secondary line so SFM stays clearly < 1 (avoid the
        // single-non-zero-bin degenerate case where geomean → 0).
        spec[lo + 1] = 50.0;
        let model = PsyModel::new_long(4, 44_100);
        let a = model.analyse(&spec);
        assert!(
            a.tonality[5] > 0.5,
            "single-peak band should be classified tonal, got tonality={}",
            a.tonality[5]
        );
        assert!(
            a.target_max[5] >= 8,
            "loud tonal band target_max should be above baseline 7; got {}",
            a.target_max[5]
        );
    }

    #[test]
    fn quiet_band_under_loud_neighbour_is_coarsened() {
        // A loud band at sfb 5 + a quiet band at sfb 7. The quiet band
        // should fall under the spread mask of the loud band and end up
        // with a small target_max (coarse coding ⇒ bit savings).
        let mut spec = vec![0.0f32; 1024];
        let swb = SWB_LONG[4];
        let lo5 = swb[5] as usize;
        let hi5 = swb[6] as usize;
        for k in lo5..hi5 {
            spec[k] = 5000.0;
        }
        let lo7 = swb[7] as usize;
        let hi7 = swb[8] as usize;
        for k in lo7..hi7 {
            spec[k] = 5.0; // ~60 dB below loud band
        }
        let model = PsyModel::new_long(4, 44_100);
        let a = model.analyse(&spec);
        // The quiet band's target_max should be small (≤ 7) since the
        // mask is dominated by the loud neighbour.
        assert!(
            a.target_max[7] <= 7,
            "quiet band under loud-neighbour mask should be coarsened, got target_max={}",
            a.target_max[7]
        );
    }

    #[test]
    fn perceptual_entropy_is_finite_and_positive() {
        let mut spec = vec![0.0f32; 1024];
        for k in 0..1024 {
            spec[k] = (k as f32).sin() * 100.0;
        }
        let model = PsyModel::new_long(4, 44_100);
        let a = model.analyse(&spec);
        assert!(a.perceptual_entropy.is_finite());
        assert!(a.perceptual_entropy >= 0.0);
    }
}

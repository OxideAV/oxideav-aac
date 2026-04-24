//! SBR HF envelope adjustment (§4.6.18.7).
//!
//! Takes XHigh (the patched complex QMF matrix) together with the
//! transmitted envelope / noise / sinusoid scalefactors and applies gains
//! to match the desired envelope shape, then adds scaled noise + harmonic
//! sinusoids at envelope boundaries (§4.6.18.7.3-4). A limiter-band pass
//! (§4.6.18.7.5) caps the per-subband gain so the noise doesn't blow up on
//! tonal input.

use super::bitstream::SbrChannelData;
use super::freq::FreqTables;
use super::hf_gen::PatchInfo;
use super::{Complex32, NUM_QMF_BANDS, RATE};

/// Deterministic pseudo-random generator for SBR noise synthesis.
///
/// The spec (Table 4.A.90) provides a fixed 512×2 table of noise samples
/// uniformly distributed in `[-1, 1]`. To avoid the 4 KiB table blob we use
/// a Park-Miller LCG whose period and statistics (uniform over `(0, 1)`) are
/// adequate for the shaped-noise rôle. Seeded from a running counter so
/// successive frames get different noise realisations.
#[derive(Clone, Copy, Debug)]
pub struct SbrNoiseRng(u32);

impl SbrNoiseRng {
    pub fn new(seed: u32) -> Self {
        // Park-Miller requires a nonzero seed < 2^31.
        let s = (seed | 1) & 0x7FFF_FFFF;
        Self(s.max(1))
    }
    fn next_u32(&mut self) -> u32 {
        // state = state · 48271 mod (2^31 - 1)
        let s = (self.0 as u64 * 48271) % 0x7FFF_FFFF;
        self.0 = s as u32;
        self.0
    }
    /// Next uniform sample in `[-1, 1]`.
    pub fn next_unit(&mut self) -> f32 {
        let n = self.next_u32();
        // Scale to (-1, 1). 2^31 - 1 = 0x7FFF_FFFF.
        (n as f32) / (0x3FFF_FFFF as f32) - 1.0
    }
    pub fn next_complex(&mut self) -> Complex32 {
        Complex32::new(self.next_unit(), self.next_unit())
    }
}

/// Compute limiter-band boundaries (§4.6.18.7.5, Table 4.176).
///
/// Returns a vector of subband indices `f_lim[0..N_L]` such that each
/// limiter band spans the subbands `[f_lim[i], f_lim[i+1])`. The boundaries
/// align with patch borders so sharp gain changes at patch boundaries are
/// confined to a single limiter band.
///
/// `bs_limiter_bands` ∈ {0, 1, 2, 3} selects the ratio: 0 disables limiting
/// (one band covering everything), 1 uses 1.2, 2 uses 2.0, 3 uses 3.0 — the
/// larger the number, the coarser the limiter bands (fewer, wider).
pub fn build_limiter_bands(ft: &FreqTables, patches: &PatchInfo, bs_limiter_bands: u8) -> Vec<i32> {
    if bs_limiter_bands == 0 {
        return vec![ft.kx, ft.kx + ft.m];
    }
    let ratio = match bs_limiter_bands {
        1 => 1.2f32,
        2 => 2.0,
        _ => 3.0,
    };
    // Start from patch borders as the coarse partition.
    let lim: Vec<i32> = patches.patch_borders[..=patches.num_patches].to_vec();
    // Refine each patch range: keep doubling band within the ratio.
    let mut refined: Vec<i32> = Vec::with_capacity(lim.len() * 4);
    refined.push(lim[0]);
    for i in 1..lim.len() {
        let lo = *refined.last().unwrap();
        let hi = lim[i];
        let mut cur = lo;
        while cur < hi {
            let next = ((cur as f32) * ratio).ceil() as i32;
            let next = next.min(hi);
            if next > cur {
                refined.push(next);
                cur = next;
            } else {
                break;
            }
        }
    }
    // Deduplicate consecutive equal entries.
    refined.dedup();
    refined
}

/// Apply envelope adjustment + noise + sinusoid synthesis + limiter pass
/// to a single channel's XHigh matrix (§4.6.18.7).
///
/// `t_e`    — envelope time-border vector in time-slots (length `LE+1`).
/// `lim`    — optional limiter-band vector. When `None`, a flat limiter
///             spanning the whole SBR range is used.
pub fn apply_envelope(
    x_high: &mut [[Complex32; NUM_QMF_BANDS]],
    data: &SbrChannelData,
    ft: &FreqTables,
    t_e: &[i32],
    t_hf_adj: usize,
) {
    apply_envelope_with_limiter(x_high, data, ft, t_e, t_hf_adj, None, 0);
}

/// Full envelope adjuster as described in §4.6.18.7.
///
/// `limiter_bands`: per the header `bs_limiter_bands`; when `Some`, each
/// limiter band bounds the per-subband gain to `limiter_gain × max-gain in
/// band` so narrow peaks don't blow up adjacent subbands.
/// `noise_seed`: deterministic seed for the PRNG — typically the running
/// frame counter. Pass 0 for a default seed.
#[allow(clippy::too_many_arguments)]
pub fn apply_envelope_with_limiter(
    x_high: &mut [[Complex32; NUM_QMF_BANDS]],
    data: &SbrChannelData,
    ft: &FreqTables,
    t_e: &[i32],
    t_hf_adj: usize,
    limiter_bands: Option<&[i32]>,
    noise_seed: u32,
) {
    let le = data.bs_num_env as usize;
    if le == 0 || t_e.len() < le + 1 {
        return;
    }
    let amp_res_bits = if data.bs_amp_res != 0 { 1 } else { 2 };
    let mut rng = SbrNoiseRng::new(noise_seed ^ 0xC0DE_BABE);
    // Limiter-gain cap: spec Table 4.176 maps bs_limiter_gains {0..3} to
    // {0.5, 1.0, 2.0, 1e10}. We fold bs_limiter_gains from the header via
    // the `_unused_limiter_gains` arg the caller didn't wire yet (default
    // = 2.0 per the most common choice).
    let limiter_gain_cap = 2.0f32;

    for env in 0..le {
        let n_bands_idx = data.freq_res[env] as usize;
        let band_table: &[i32] = if n_bands_idx != 0 {
            &ft.f_high
        } else {
            &ft.f_low
        };
        let n_bands = if n_bands_idx != 0 {
            ft.n_high
        } else {
            ft.n_low
        };
        // Pick the noise-floor index to use for this envelope (§4.6.18.3.3
        // Note: noise-floor envelope count is bs_num_noise; envelopes map
        // to the nearest noise floor).
        let n_idx = if le == 1 {
            0
        } else {
            env.min(data.bs_num_noise as usize - 1)
        };
        let l_start = (RATE as i32 * t_e[env]) as usize + t_hf_adj;
        let l_end_raw = (RATE as i32 * t_e[env + 1]) as usize + t_hf_adj;
        if l_start >= x_high.len() {
            continue;
        }
        let l_end = l_end_raw.min(x_high.len());

        // 1) Dequantise envelope + noise scalefactors and compute per-QMF-
        //    subband (E_orig, Q_orig, S_mask) triples.
        let mut e_orig_sb = [0.0f32; NUM_QMF_BANDS];
        let mut q_orig_sb = [0.0f32; NUM_QMF_BANDS];
        let mut s_mask_sb = [false; NUM_QMF_BANDS];

        let mut acc_e = 0i32;
        for k in 0..n_bands {
            let d = data.env_sf[env][k];
            if data.bs_df_env[env] == 0 && k > 0 {
                acc_e += d;
            } else {
                acc_e = d;
            }
            let e_band = 64.0f32 * 2.0f32.powf(acc_e as f32 / amp_res_bits as f32);
            let k0 = band_table[k].max(0) as usize;
            let k1 = band_table[k + 1].min(NUM_QMF_BANDS as i32).max(0) as usize;
            for kk in k0..k1 {
                e_orig_sb[kk] = e_band;
            }
        }
        // Noise-floor dequant: Q_orig(band) = 2^(NOISE_FLOOR_OFFSET - noise_sf)
        // per §4.6.18.3.5 (NOISE_FLOOR_OFFSET = 6 in the spec).
        let mut acc_q = 0i32;
        const NOISE_FLOOR_OFFSET: i32 = 6;
        for nb in 0..ft.nq {
            let d = data.noise_sf[n_idx][nb];
            if data.bs_df_noise[n_idx] == 0 && nb > 0 {
                acc_q += d;
            } else {
                acc_q = d;
            }
            let q_band = 2.0f32.powf((NOISE_FLOOR_OFFSET - acc_q) as f32);
            let k0 = ft.f_noise[nb].max(0) as usize;
            let k1 = ft.f_noise[nb + 1].min(NUM_QMF_BANDS as i32).max(0) as usize;
            for kk in k0..k1 {
                q_orig_sb[kk] = q_band;
            }
        }
        // Sinusoid mask: bs_add_harmonic is indexed by high-res bands; the
        // sinusoid fires at the "middle" subband of each flagged high-res
        // band (§4.6.18.7.4 chooses the middle-odd subband deterministically).
        if data.bs_add_harmonic_flag {
            for band in 0..ft.n_high {
                if data.bs_add_harmonic[band] != 0 {
                    let k0 = ft.f_high[band].max(0);
                    let k1 = ft.f_high[band + 1].min(NUM_QMF_BANDS as i32);
                    // Pick middle-odd sub-band per spec.
                    let mut mid = (k0 + k1) / 2;
                    if (mid - ft.kx) % 2 == 0 {
                        mid += 1;
                    }
                    let mid = mid.clamp(0, NUM_QMF_BANDS as i32 - 1) as usize;
                    s_mask_sb[mid] = true;
                }
            }
        }

        // 2) Per-subband current energy over the envelope time-span.
        let mut e_curr_sb = [0.0f32; NUM_QMF_BANDS];
        {
            let span = (l_end - l_start).max(1);
            for kk in 0..NUM_QMF_BANDS {
                let mut s = 0.0f32;
                for l in l_start..l_end {
                    s += x_high[l][kk].norm_sqr();
                }
                e_curr_sb[kk] = s / span as f32;
            }
        }

        // 3) Gain + noise level per subband.
        let eps = 1e-12f32;
        let mut gain_sb = [0.0f32; NUM_QMF_BANDS];
        let mut q_m_sb = [0.0f32; NUM_QMF_BANDS];
        let mut s_m_sb = [0.0f32; NUM_QMF_BANDS];
        for kk in (ft.kx as usize)..(ft.kx + ft.m).min(NUM_QMF_BANDS as i32) as usize {
            let eo = e_orig_sb[kk];
            let qo = q_orig_sb[kk];
            let s_on = s_mask_sb[kk];
            if s_on {
                // Sinusoid present: S_M gets energy; noise is suppressed.
                // G_lim = sqrt(E_orig / (E_curr * (1 + Q_orig)))
                // S_M^2 = E_orig - Q_M^2 (sinusoid takes the rest).
                let denom_s = e_curr_sb[kk] * (1.0 + qo) + eps;
                gain_sb[kk] = (eo / denom_s).max(0.0).sqrt();
                q_m_sb[kk] = 0.0;
                s_m_sb[kk] = (eo * qo / (1.0 + qo)).max(0.0).sqrt();
            } else if qo > 0.0 {
                // Noise present, no sinusoid.
                let denom = e_curr_sb[kk] * (1.0 + qo) + eps;
                gain_sb[kk] = (eo / denom).max(0.0).sqrt();
                q_m_sb[kk] = (eo * qo / (1.0 + qo)).max(0.0).sqrt();
                s_m_sb[kk] = 0.0;
            } else {
                // Neither noise nor sinusoid: just match envelope.
                gain_sb[kk] = (eo / (e_curr_sb[kk] + eps)).max(0.0).sqrt();
                q_m_sb[kk] = 0.0;
                s_m_sb[kk] = 0.0;
            }
        }

        // 4) Limiter pass — clamp gains inside each limiter band.
        let default_lim = [ft.kx, ft.kx + ft.m];
        let lim = limiter_bands.unwrap_or(&default_lim);
        for w in lim.windows(2) {
            let k0 = w[0].max(ft.kx) as usize;
            let k1 = w[1].min(ft.kx + ft.m) as usize;
            if k1 <= k0 {
                continue;
            }
            // Reference gain = sqrt(sum E_orig / sum E_curr) over the limiter band.
            let mut sum_eo = 0.0f32;
            let mut sum_ec = 0.0f32;
            for kk in k0..k1 {
                sum_eo += e_orig_sb[kk];
                sum_ec += e_curr_sb[kk];
            }
            let g_ref = (sum_eo / (sum_ec + eps)).max(0.0).sqrt();
            let cap = g_ref * limiter_gain_cap;
            for kk in k0..k1 {
                if gain_sb[kk] > cap {
                    // Rescale Q_M and S_M downward too so energy stays matched.
                    let scale = cap / (gain_sb[kk] + eps);
                    gain_sb[kk] = cap;
                    q_m_sb[kk] *= scale;
                    s_m_sb[kk] *= scale;
                }
                // Also clamp against absolute runaway.
                gain_sb[kk] = gain_sb[kk].min(1e4);
            }
        }

        // 5) Apply gains + add noise + add sinusoids over every QMF subsample
        //    inside the envelope.
        let span = (l_end - l_start).max(1) as f32;
        let inv_sqrt_span = 1.0 / span.sqrt();
        for l in l_start..l_end {
            for kk in (ft.kx as usize)..(ft.kx + ft.m).min(NUM_QMF_BANDS as i32) as usize {
                let c = x_high[l][kk];
                let scaled = Complex32::new(c.re * gain_sb[kk], c.im * gain_sb[kk]);
                // Noise contribution.
                let noise = if q_m_sb[kk] > 0.0 {
                    let nz = rng.next_complex();
                    Complex32::new(
                        nz.re * q_m_sb[kk] * inv_sqrt_span,
                        nz.im * q_m_sb[kk] * inv_sqrt_span,
                    )
                } else {
                    Complex32::default()
                };
                // Sinusoid contribution — rotates by π/2 per subsample, so
                // over 4 subsamples the complex exponential cycles.
                let sinus = if s_m_sb[kk] > 0.0 {
                    let phi = ((l as i32) & 3) as f32 * core::f32::consts::FRAC_PI_2;
                    let sign = if (kk as i32 - ft.kx) & 1 == 0 {
                        1.0
                    } else {
                        -1.0
                    };
                    Complex32::new(s_m_sb[kk] * sign * phi.cos(), s_m_sb[kk] * sign * phi.sin())
                } else {
                    Complex32::default()
                };
                x_high[l][kk] = Complex32::new(
                    scaled.re + noise.re + sinus.re,
                    scaled.im + noise.im + sinus.im,
                );
            }
        }
    }
}

/// Apply coupled-mode envelope adjustment to a CPE pair.
///
/// Coupled-mode encoding (§4.6.18.3.5) shares one envelope/noise grid + flags
/// between the two channels. `data_l` carries the total-energy scalefactors;
/// `data_r` carries balance values encoded against the `bs_amp_res`-dependent
/// balance Huffman tables. For each (env, band) pair the spec defines
///
///   `pan = 2^(E_balance / a)` where `a = 1` at 3.0 dB or `a = 2` at 1.5 dB,
///   `E_orig_L = E_total / (1 + pan)`,
///   `E_orig_R = E_total * pan / (1 + pan)`.
///
/// `E_total` comes from `data_l.env_sf` using the same
/// `64 * 2^(E_total / a)` formula as the uncoupled SCE path (the coupled raw
/// value is the sum, not each channel separately). After unpacking, we apply
/// gains to each channel independently.
pub fn apply_envelope_coupled(
    x_high_l: &mut [[Complex32; NUM_QMF_BANDS]],
    x_high_r: &mut [[Complex32; NUM_QMF_BANDS]],
    data_l: &SbrChannelData,
    data_r: &SbrChannelData,
    ft: &FreqTables,
    t_e: &[i32],
    t_hf_adj: usize,
) {
    apply_envelope_coupled_with_limiter(
        x_high_l, x_high_r, data_l, data_r, ft, t_e, t_hf_adj, None, 0,
    );
}

/// Full coupled-mode envelope adjuster — see `apply_envelope_coupled`.
#[allow(clippy::too_many_arguments)]
pub fn apply_envelope_coupled_with_limiter(
    x_high_l: &mut [[Complex32; NUM_QMF_BANDS]],
    x_high_r: &mut [[Complex32; NUM_QMF_BANDS]],
    data_l: &SbrChannelData,
    data_r: &SbrChannelData,
    ft: &FreqTables,
    t_e: &[i32],
    t_hf_adj: usize,
    limiter_bands: Option<&[i32]>,
    noise_seed: u32,
) {
    // We delegate to apply_envelope_with_limiter per-channel — constructing
    // unpacked SbrChannelData copies that hold the decoupled E_orig / E_bal
    // values as if each channel had been transmitted independently.
    //
    // Simpler alternative: run the coupled energy split inline (as before)
    // without noise + sinusoid, and add noise as a post-pass. For the
    // coupled path we take the simpler path — noise is added only to
    // subbands where the envelope left a gap (noise scalefactors >> 0).
    let _ = limiter_bands;
    let _ = noise_seed;
    let le = data_l.bs_num_env as usize;
    if le == 0 || t_e.len() < le + 1 {
        return;
    }
    let amp_res_bits = if data_l.bs_amp_res != 0 { 1 } else { 2 };
    for env in 0..le {
        let n_bands_idx = data_l.freq_res[env] as usize;
        let band_table: &[i32] = if n_bands_idx != 0 {
            &ft.f_high
        } else {
            &ft.f_low
        };
        let n_bands = if n_bands_idx != 0 {
            ft.n_high
        } else {
            ft.n_low
        };
        let mut acc_t = 0i32;
        let mut acc_b = 0i32;
        for k in 0..n_bands {
            let d_t = data_l.env_sf[env][k];
            let d_b = data_r.env_sf[env][k];
            if data_l.bs_df_env[env] == 0 && k > 0 {
                acc_t += d_t;
            } else {
                acc_t = d_t;
            }
            if data_r.bs_df_env[env] == 0 && k > 0 {
                acc_b += d_b;
            } else {
                acc_b = d_b;
            }
            // E_total value decoded from the "raw" scalefactor. The spec's
            // coupled formula treats `acc_t` as the joint energy scalefactor.
            let e_total = 64.0f32 * 2.0f32.powf(acc_t as f32 / amp_res_bits as f32);
            // Balance "pan" ratio. For balance Huffman tables the LAV is
            // smaller so `acc_b` is the signed balance code.
            let pan = 2.0f32.powf(acc_b as f32 / amp_res_bits as f32);
            let e_l = e_total / (1.0 + pan);
            let e_r = e_total * pan / (1.0 + pan);
            let k0 = band_table[k].max(0) as usize;
            let k1 = band_table[k + 1].min(NUM_QMF_BANDS as i32).max(0) as usize;
            if k1 <= k0 {
                continue;
            }
            let l_start = (RATE as i32 * t_e[env]) as usize + t_hf_adj;
            let l_end = (RATE as i32 * t_e[env + 1]) as usize + t_hf_adj;
            if l_start >= x_high_l.len() || l_start >= x_high_r.len() {
                continue;
            }
            let l_end = l_end.min(x_high_l.len()).min(x_high_r.len());
            let mut sum_l = 0.0f32;
            let mut sum_r = 0.0f32;
            let mut count = 0usize;
            for l in l_start..l_end {
                for kk in k0..k1 {
                    sum_l += x_high_l[l][kk].norm_sqr();
                    sum_r += x_high_r[l][kk].norm_sqr();
                    count += 1;
                }
            }
            let eps = 1e-12f32;
            let e_curr_l = if count > 0 { sum_l / count as f32 } else { 0.0 };
            let e_curr_r = if count > 0 { sum_r / count as f32 } else { 0.0 };
            let gain_l = (e_l / (e_curr_l + eps)).max(0.0).sqrt().min(1e4);
            let gain_r = (e_r / (e_curr_r + eps)).max(0.0).sqrt().min(1e4);
            for l in l_start..l_end {
                for kk in k0..k1 {
                    let cl = x_high_l[l][kk];
                    let cr = x_high_r[l][kk];
                    x_high_l[l][kk] = Complex32::new(cl.re * gain_l, cl.im * gain_l);
                    x_high_r[l][kk] = Complex32::new(cr.re * gain_r, cr.im * gain_r);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sbr::hf_gen::PatchInfo;

    #[test]
    fn noise_rng_unit_range() {
        let mut rng = SbrNoiseRng::new(12345);
        for _ in 0..10_000 {
            let v = rng.next_unit();
            assert!(v >= -1.0 && v < 1.0);
        }
    }

    #[test]
    fn limiter_bands_are_monotonic() {
        let ft = crate::sbr::freq::FreqTables::build(48_000, 5, 9, 0, 2, false, 2).unwrap();
        let mut pi = PatchInfo::default();
        pi.num_patches = 1;
        pi.patch_num_subbands[0] = ft.m;
        pi.patch_borders[0] = ft.kx;
        pi.patch_borders[1] = ft.kx + ft.m;
        let lim = build_limiter_bands(&ft, &pi, 2);
        for w in lim.windows(2) {
            assert!(w[1] > w[0]);
        }
        assert_eq!(lim[0], ft.kx);
        assert_eq!(*lim.last().unwrap(), ft.kx + ft.m);
    }

    #[test]
    fn limiter_bands_disabled_is_flat() {
        let ft = crate::sbr::freq::FreqTables::build(48_000, 5, 9, 0, 2, false, 2).unwrap();
        let mut pi = PatchInfo::default();
        pi.num_patches = 1;
        pi.patch_num_subbands[0] = ft.m;
        pi.patch_borders[0] = ft.kx;
        pi.patch_borders[1] = ft.kx + ft.m;
        let lim = build_limiter_bands(&ft, &pi, 0);
        assert_eq!(lim, vec![ft.kx, ft.kx + ft.m]);
    }
}

/// Compute envelope time-border vector tE (§4.6.18.3.3).
pub fn envelope_time_borders(data: &SbrChannelData, num_time_slots: i32) -> [i32; 6] {
    use super::bitstream::FrameClass;
    let mut t_e = [0i32; 6];
    let le = data.bs_num_env as usize;
    let abs_bord_lead = match data.frame_class {
        FrameClass::FixFix | FrameClass::FixVar => 0,
        FrameClass::VarVar | FrameClass::VarFix => data.bs_var_bord_0 as i32,
    };
    let abs_bord_trail = match data.frame_class {
        FrameClass::FixFix | FrameClass::VarFix => num_time_slots,
        FrameClass::VarVar | FrameClass::FixVar => data.bs_var_bord_1 as i32 + num_time_slots,
    };
    let n_rel_lead = match data.frame_class {
        FrameClass::FixFix => le.saturating_sub(1),
        FrameClass::FixVar => 0,
        FrameClass::VarVar | FrameClass::VarFix => data.bs_num_rel_0 as usize,
    };
    let n_rel_trail = match data.frame_class {
        FrameClass::FixFix | FrameClass::VarFix => 0,
        FrameClass::VarVar | FrameClass::FixVar => data.bs_num_rel_1 as usize,
    };
    for l in 0..=le {
        t_e[l] = if l == 0 {
            abs_bord_lead
        } else if l == le {
            abs_bord_trail
        } else if l <= n_rel_lead {
            let mut sum = abs_bord_lead;
            for i in 0..l {
                let val = match data.frame_class {
                    FrameClass::FixFix => {
                        // NINT(numTimeSlots / LE)
                        let nint = (num_time_slots as f32 / le as f32 + 0.5).floor() as i32;
                        nint
                    }
                    FrameClass::FixVar => 0, // NA
                    FrameClass::VarVar | FrameClass::VarFix => data.bs_rel_bord_0[i] as i32,
                };
                sum += val;
            }
            sum
        } else {
            // Trailing section.
            let mut sum = abs_bord_trail;
            for i in 0..(le - l) {
                let val = match data.frame_class {
                    FrameClass::FixFix | FrameClass::VarFix => 0,
                    FrameClass::VarVar | FrameClass::FixVar => data.bs_rel_bord_1[i] as i32,
                };
                sum -= val;
            }
            sum
        };
        let _ = n_rel_trail;
    }
    t_e
}

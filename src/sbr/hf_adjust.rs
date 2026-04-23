//! SBR HF envelope adjustment (§4.6.18.7).
//!
//! Takes XHigh (the patched complex QMF matrix) together with the
//! transmitted envelope / noise / sinusoid scalefactors and applies gains
//! to match the desired envelope shape.

use super::bitstream::SbrChannelData;
use super::freq::FreqTables;
use super::{Complex32, NUM_QMF_BANDS, RATE};

/// Apply simple envelope adjustment: for each SBR envelope band, scale
/// XHigh subbands so that their total energy matches the transmitted
/// envelope scalefactor.
///
/// This implements the core of §4.6.18.7 at reduced fidelity — it dequants
/// the envelope scalefactors, averages the current XHigh energy per band,
/// and applies a gain. Noise/sinusoid addition is omitted here; the
/// aggregate result is still bandwidth-extended audio.
///
/// `t_e` is the envelope time-border vector in time-slots (length LE+1).
/// `x_high` indexes QMF subsamples then QMF subbands.
pub fn apply_envelope(
    x_high: &mut [[Complex32; NUM_QMF_BANDS]],
    data: &SbrChannelData,
    ft: &FreqTables,
    t_e: &[i32],
    t_hf_adj: usize,
) {
    let le = data.bs_num_env as usize;
    if le == 0 || t_e.len() < le + 1 {
        return;
    }
    let amp_res_bits = if data.bs_amp_res != 0 { 1 } else { 2 }; // a=1 or a=2
    // Dequantise accumulated envelope scalefactors and apply band by band.
    for env in 0..le {
        let n_bands_idx = data.freq_res[env] as usize;
        let band_table: &[i32] = if n_bands_idx != 0 {
            &ft.f_high
        } else {
            &ft.f_low
        };
        let n_bands = if n_bands_idx != 0 { ft.n_high } else { ft.n_low };
        // Dequantiser: accumulate deltas (simple freq-direction case).
        // Spec §4.6.18.3.5: E_orig = 64 * 2^(E(k,l)/a)
        let mut acc_e = 0i32;
        for k in 0..n_bands {
            let d = data.env_sf[env][k];
            // For freq-delta mode (bs_df_env=0) the first value is
            // absolute and subsequent bands accumulate. For time-delta
            // mode we don't have the previous frame's state here, so we
            // treat deltas as absolutes — this degrades envelope tracking
            // but still produces plausible high-band audio.
            if data.bs_df_env[env] == 0 && k > 0 {
                acc_e += d;
            } else {
                acc_e = d;
            }
            let e_orig = 64.0f32 * 2.0f32.powf(acc_e as f32 / amp_res_bits as f32);
            // Compute current-envelope energy: average |X_High|^2 over
            // all subbands inside this band and all subsamples inside the
            // envelope time-span.
            let k0 = band_table[k].max(0) as usize;
            let k1 = band_table[k + 1].min(NUM_QMF_BANDS as i32).max(0) as usize;
            if k1 <= k0 {
                continue;
            }
            let l_start = (RATE as i32 * t_e[env]) as usize + t_hf_adj;
            let l_end = (RATE as i32 * t_e[env + 1]) as usize + t_hf_adj;
            if l_start >= x_high.len() {
                continue;
            }
            let l_end = l_end.min(x_high.len());
            let mut sum = 0.0f32;
            let mut count = 0usize;
            for l in l_start..l_end {
                for kk in k0..k1 {
                    sum += x_high[l][kk].norm_sqr();
                    count += 1;
                }
            }
            let e_curr = if count > 0 { sum / count as f32 } else { 0.0 };
            let eps = 1e-12f32;
            let gain_sq = e_orig / (e_curr + eps);
            // Cap gain to avoid blowing up noise — spec §4.6.18.7.5 uses a
            // limiter; we clamp at 1e4.
            let gain = gain_sq.max(0.0).sqrt().min(1e4);
            for l in l_start..l_end {
                for kk in k0..k1 {
                    let c = x_high[l][kk];
                    x_high[l][kk] = Complex32::new(c.re * gain, c.im * gain);
                }
            }
        }
    }
}

/// Compute envelope time-border vector tE (§4.6.18.3.3).
pub fn envelope_time_borders(
    data: &SbrChannelData,
    num_time_slots: i32,
) -> [i32; 6] {
    use super::bitstream::FrameClass;
    let mut t_e = [0i32; 6];
    let le = data.bs_num_env as usize;
    let abs_bord_lead = match data.frame_class {
        FrameClass::FixFix | FrameClass::FixVar => 0,
        FrameClass::VarVar | FrameClass::VarFix => data.bs_var_bord_0 as i32,
    };
    let abs_bord_trail = match data.frame_class {
        FrameClass::FixFix | FrameClass::VarFix => num_time_slots,
        FrameClass::VarVar | FrameClass::FixVar => {
            data.bs_var_bord_1 as i32 + num_time_slots
        }
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

//! SBR High-Frequency generation (§4.6.18.6).
//!
//! Produces `XHigh[k][l]` from `XLow[k][l]` by patching low-band subband
//! samples up into the high-band region `kx..kx+M`.

use super::freq::FreqTables;
use super::{Complex32, NUM_QMF_BANDS};
use oxideav_core::{Error, Result};

/// Inverse-filtering bandwidth array element for each noise-floor band.
pub type BwArray = [f32; 5];

/// Patching result. Stores how many subbands each patch covers, and the
/// start-subband each patch copies from. Required for the limiter table.
#[derive(Clone, Debug, Default)]
pub struct PatchInfo {
    pub num_patches: usize,
    pub patch_num_subbands: [i32; 6],
    pub patch_start_subband: [i32; 6],
    pub patch_borders: [i32; 7],
}

/// Construct the patches per §4.6.18.6.3 Figure 4.48.
pub fn build_patches(ft: &FreqTables, fs_sbr: u32) -> Result<PatchInfo> {
    let mut info = PatchInfo::default();
    let mut msb = ft.k0;
    let mut usb = ft.kx;
    let mut num_patches = 0usize;

    // goalSb = NINT( 2.048e6 / Fs )
    let goal_sb = nint(2_048_000.0f32 / fs_sbr as f32);
    let mut k = if goal_sb < ft.kx + ft.m {
        // Find first i such that fMaster[i] >= goalSb, record k = i+1 of
        // the last i where fMaster[i] < goalSb.
        let mut last_lt: usize = 0;
        for (i, &v) in ft.f_master.iter().enumerate() {
            if v < goal_sb {
                last_lt = i + 1;
            } else {
                break;
            }
        }
        last_lt
    } else {
        ft.n_master
    };

    let mut j = k;
    let mut patch_counter = 0usize;
    // Cap the iterations — build_patches is O(numPatches) and the spec
    // caps numPatches at 5. Each outer pass adds at most one patch;
    // bounding the outer loop at ~NMaster * 2 is a safe upper bound.
    let max_outer = ft.n_master * 4 + 16;
    let mut iter_guard = 0usize;
    loop {
        iter_guard += 1;
        if iter_guard > max_outer {
            // Didn't converge — abort, caller will treat as "no patches" and
            // fall back to low-band-only output.
            break;
        }
        let mut sb: i32;
        // Decrement j until condition met.
        let mut inner_guard = 0usize;
        loop {
            inner_guard += 1;
            if inner_guard > ft.n_master + 2 {
                break;
            }
            sb = ft.f_master[j];
            let odd = ((sb - 2 + ft.k0).rem_euclid(2)).abs();
            if sb <= (ft.k0 - 1 + msb - odd) {
                break;
            }
            if j == 0 {
                break;
            }
            j -= 1;
        }
        sb = ft.f_master[j];
        let odd = ((sb - 2 + ft.k0).rem_euclid(2)).abs();
        let pns = (sb - usb).max(0);
        let pss = ft.k0 - odd - pns;
        if pns > 0 {
            if patch_counter >= 6 {
                return Err(Error::invalid("SBR: too many patches"));
            }
            info.patch_num_subbands[patch_counter] = pns;
            info.patch_start_subband[patch_counter] = pss;
            patch_counter += 1;
            usb = sb;
            msb = sb;
        } else {
            msb = ft.kx;
        }
        if k < ft.f_master.len() && ft.f_master[k] - sb < 3 {
            k = ft.n_master;
        }
        if sb == ft.kx + ft.m {
            break;
        }
        // If we haven't advanced at all (sb hasn't changed), force progress.
        if pns == 0 && j == 0 {
            break;
        }
    }
    if patch_counter > 1 && info.patch_num_subbands[patch_counter - 1] < 3 {
        patch_counter -= 1;
    }
    num_patches = patch_counter;
    info.num_patches = num_patches;
    // Patch borders for the limiter-band construction.
    info.patch_borders[0] = ft.kx;
    for i in 1..=num_patches {
        info.patch_borders[i] = info.patch_borders[i - 1] + info.patch_num_subbands[i - 1];
    }
    Ok(info)
}

/// Copy-up patching of `XLow` into `XHigh` (no inverse filtering yet —
/// bwArray terms are folded in by `apply_hf_generation`).
///
/// `x_low[k][l]`: analysis-bank output, 32 subbands × `num_subsamples` slots.
/// `x_high[k][l]`: 64 synthesis-bank subbands × `num_subsamples` slots. Only
/// subbands `kx..kx+M` are filled; below-kx are unchanged (they carry the
/// verbatim low-band signal).
pub fn apply_hf_generation(
    x_low: &[[Complex32; 32]],
    x_high: &mut [[Complex32; NUM_QMF_BANDS]],
    patches: &PatchInfo,
    ft: &FreqTables,
    bw: &BwArray,
    alpha0: &[Complex32; 32],
    alpha1: &[Complex32; 32],
    t_hf_adj: usize,
    t_e0: usize,
    t_elast: usize,
) {
    let num_slots = x_high.len();
    // First, copy the low-band subbands verbatim (k < kx).
    for l in 0..num_slots.min(x_low.len()) {
        for k in 0..(ft.kx as usize).min(32).min(NUM_QMF_BANDS) {
            x_high[l][k] = x_low[l][k];
        }
    }
    // Then patch high-band subbands.
    for i in 0..patches.num_patches {
        for x in 0..patches.patch_num_subbands[i] as usize {
            let k_dst_i =
                ft.kx as usize + patches.patch_borders[i] as usize - ft.kx as usize + x;
            let p = patches.patch_start_subband[i] as usize + x;
            if k_dst_i >= NUM_QMF_BANDS || p >= 32 {
                continue;
            }
            // Noise-band index g(k_dst) determined by fTableNoise.
            let g = noise_band_index(ft, k_dst_i as i32);
            let bw_g = bw[g.min(bw.len() - 1)];
            let bw_g2 = bw_g * bw_g;
            // Iterate across QMF subsamples in the SBR frame range:
            //   l in [RATE * t_e[0], RATE * t_e[LE])
            // but we process the full num_slots (caller provides the slice
            // aligned to the HFAdj frame origin).
            let start = (super::RATE * t_e0).min(num_slots);
            let end = (super::RATE * t_elast).min(num_slots);
            for l in start..end {
                // X_High[k,l+t_HFAdj] = X_Low[p,l+t_HFAdj]
                //   + bw * alpha0[p] * X_Low[p,l-1+t_HFAdj]
                //   + bw^2 * alpha1[p] * X_Low[p,l-2+t_HFAdj]
                let l_src = l + t_hf_adj;
                if l_src >= x_low.len() {
                    continue;
                }
                let x0 = x_low[l_src][p];
                let x1 = if l_src >= 1 { x_low[l_src - 1][p] } else { Complex32::default() };
                let x2 = if l_src >= 2 { x_low[l_src - 2][p] } else { Complex32::default() };
                let term1 = alpha0[p].scale(bw_g) * x1;
                let term2 = alpha1[p].scale(bw_g2) * x2;
                x_high[l_src][k_dst_i] = x0 + term1 + term2;
            }
        }
    }
}

fn noise_band_index(ft: &FreqTables, k: i32) -> usize {
    // g(k): fTableNoise[g(k)] <= k < fTableNoise[g(k)+1]
    for (i, w) in ft.f_noise.windows(2).enumerate() {
        if k >= w[0] && k < w[1] {
            return i;
        }
    }
    0
}

/// Compute newBw from inverse-filtering mode per Table 4.175.
pub fn new_bw(prev_mode: u8, cur_mode: u8) -> f32 {
    const TABLE: [[f32; 4]; 4] = [
        // prev \ cur = Off, Low, Intermediate, Strong
        [0.0, 0.6, 0.9, 0.98],         // prev = Off
        [0.6, 0.75, 0.9, 0.98],        // prev = Low
        [0.0, 0.75, 0.9, 0.98],        // prev = Intermediate
        [0.0, 0.75, 0.9, 0.98],        // prev = Strong
    ];
    let p = (prev_mode as usize).min(3);
    let c = (cur_mode as usize).min(3);
    TABLE[p][c]
}

/// Update bwArray following §4.6.18.6.2 formulas.
pub fn update_bw(
    prev_bw: &BwArray,
    prev_modes: &[u8; 5],
    cur_modes: &[u8; 5],
    nq: usize,
) -> BwArray {
    let mut out = [0.0f32; 5];
    for i in 0..nq.min(5) {
        let new_bw_i = new_bw(prev_modes[i], cur_modes[i]);
        let temp_bw = if new_bw_i < prev_bw[i] {
            0.75 * new_bw_i + 0.25 * prev_bw[i]
        } else {
            0.90625 * new_bw_i + 0.09375 * prev_bw[i]
        };
        out[i] = if temp_bw < 0.015625 { 0.0 } else { temp_bw };
    }
    out
}

fn nint(x: f32) -> i32 {
    if x >= 0.0 {
        (x + 0.5).floor() as i32
    } else {
        -((-x + 0.5).floor() as i32)
    }
}

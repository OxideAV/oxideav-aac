//! Perceptual Noise Substitution (PNS) — ISO/IEC 14496-3 §4.6.13.
//!
//! PNS replaces a scalefactor band's coefficients with pseudo-random noise
//! scaled to the band's stored energy. Detection: the band's Huffman codebook
//! index equals `NOISE_HCB` (13). The scalefactor for that band encodes the
//! RMS energy per spectral line as an exponent.
//!
//! Stereo handling (§4.6.13.5): when a CPE's `ms_mask_present == 1` AND the
//! `ms_used[g,sfb]` bit is set for a noise band, both channels share the same
//! random sequence (correlated noise). Otherwise each channel draws
//! independently.
//!
//! PRNG: AAC does not mandate a specific PRNG, only the resulting band
//! energy. We use a Park-Miller LCG with a per-decoder seed (matches FAAD2).

use crate::ics::{
    group_starts, IcsInfo, SectionData, INTENSITY_HCB, INTENSITY_HCB2, NOISE_HCB, SPEC_LEN,
    ZERO_HCB,
};
use crate::sfband::{SWB_LONG, SWB_SHORT};

/// Stateful Park-Miller LCG used to draw random spectral lines for PNS.
#[derive(Clone, Debug)]
pub struct PnsRng {
    state: u32,
}

impl PnsRng {
    pub fn new() -> Self {
        // Non-zero seed; matches FAAD2's default.
        Self { state: 1 }
    }

    pub fn with_seed(seed: u32) -> Self {
        Self {
            state: if seed == 0 { 1 } else { seed },
        }
    }

    /// Advance and return a float in roughly `[-1, 1]`.
    /// Uses the Park-Miller minimal standard LCG then rescales.
    #[inline]
    pub fn next_float(&mut self) -> f32 {
        // Park-Miller: state = state * 16807 mod (2^31 - 1), using 64-bit math.
        let s = (self.state as u64) * 16807;
        let s = (s & 0x7FFFFFFF) + (s >> 31);
        let s = (s & 0x7FFFFFFF) + (s >> 31);
        self.state = s as u32;
        // Map to [-1, 1).
        (self.state as f32) * (2.0 / 2147483647.0) - 1.0
    }
}

impl Default for PnsRng {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute the PNS scaling factor for a scalefactor value.
/// Per §4.6.13.3 the stored scalefactor is an energy-domain value; each
/// spectral line is drawn unit-normal then multiplied by
/// `sqrt(band_energy / band_length)`. We fold the 2^(sf/4) convention into
/// a single gain: `gain = 2^(sf/4 - 14.5)` — the `-14.5` offset is the
/// common reference-impl calibration (FAAD2) that makes PNS output match
/// coefficient-quantised bands at the same scalefactor.
#[inline]
pub fn pns_gain(sf: i32) -> f32 {
    let exp = sf as f32 * 0.25 - 14.5;
    2.0f32.powf(exp)
}

/// Apply PNS to a long-window spectrum. For each sfb where the codebook is
/// `NOISE_HCB`, fill the band with random samples scaled to the energy
/// implied by `sf[sfb]`.
pub fn apply_pns_long(
    spec: &mut [f32; SPEC_LEN],
    info: &IcsInfo,
    sec: &SectionData,
    sf: &[i32],
    rng: &mut PnsRng,
    correlated: Option<&mut [f32; SPEC_LEN]>,
    ms_noise_used: Option<&[bool]>,
) {
    let swb = SWB_LONG[info.sf_index as usize];
    let max_sfb = info.max_sfb as usize;
    let mut shared: Option<&mut [f32; SPEC_LEN]> = correlated;
    for sfb in 0..max_sfb {
        if sec.sfb_cb[sfb] != NOISE_HCB {
            continue;
        }
        let start = swb[sfb] as usize;
        let end = swb[sfb + 1] as usize;
        let gain = pns_gain(sf[sfb]);
        let share = ms_noise_used.map(|m| m[sfb]).unwrap_or(false);
        for s in start..end {
            let v = rng.next_float() * gain;
            spec[s] = v;
            if share {
                if let Some(other) = shared.as_deref_mut() {
                    other[s] = v;
                }
            } else if let Some(other) = shared.as_deref_mut() {
                // Independent noise for the other channel.
                other[s] = rng.next_float() * gain;
            }
        }
    }
}

/// Apply PNS to an eight-short-window spectrum (grouped layout). Noise bands
/// are filled per sub-window.
pub fn apply_pns_short(
    spec: &mut [f32; SPEC_LEN],
    info: &IcsInfo,
    sec: &SectionData,
    sf: &[i32],
    rng: &mut PnsRng,
) {
    let swb = SWB_SHORT[info.sf_index as usize];
    let max_sfb = info.max_sfb as usize;
    let groups = info.num_window_groups as usize;
    let starts = group_starts(info);
    for g in 0..groups {
        let group_len = info.window_group_length[g] as usize;
        let win_start_offset = starts[g] * 128;
        for sfb in 0..max_sfb {
            if sec.sfb_cb[g * max_sfb + sfb] != NOISE_HCB {
                continue;
            }
            let band_start = swb[sfb] as usize;
            let band_end = swb[sfb + 1] as usize;
            let gain = pns_gain(sf[g * max_sfb + sfb]);
            for w in 0..group_len {
                for j in band_start..band_end {
                    let dst = win_start_offset + w * 128 + j;
                    spec[dst] = rng.next_float() * gain;
                }
            }
        }
    }
}

/// Return true if any band of either channel uses NOISE_HCB and the ms_used
/// flag is set — i.e. the two channels should get correlated noise for that
/// band. (§4.6.13.5.)
pub fn any_correlated_noise(sec0: &SectionData, ms_used: &[bool]) -> bool {
    // We only check channel 0's codebook because in a CPE with common_window
    // both channels share section data section-by-section; when ms_used[sfb]
    // is set and cb == NOISE_HCB, the noise is correlated across channels.
    for (i, used) in ms_used.iter().enumerate() {
        if *used && sec0.sfb_cb.get(i) == Some(&NOISE_HCB) {
            return true;
        }
    }
    false
}

/// Returns an `ms_used` mask restricted to NOISE_HCB bands — useful when the
/// caller wants to apply correlated noise only on the bands that actually are
/// PNS, leaving regular M/S handling for other bands.
pub fn noise_correlation_mask(sec: &SectionData, ms_used: &[bool]) -> Vec<bool> {
    ms_used
        .iter()
        .enumerate()
        .map(|(i, u)| *u && sec.sfb_cb.get(i) == Some(&NOISE_HCB))
        .collect()
}

/// Convenience: is `cb` a noise/intensity/zero codebook that skips standard
/// coefficient decoding?
#[inline]
pub fn is_special_cb(cb: u8) -> bool {
    matches!(cb, ZERO_HCB | NOISE_HCB | INTENSITY_HCB | INTENSITY_HCB2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prng_is_deterministic() {
        let mut a = PnsRng::with_seed(42);
        let mut b = PnsRng::with_seed(42);
        for _ in 0..100 {
            assert_eq!(a.next_float(), b.next_float());
        }
    }

    #[test]
    fn prng_changes_with_seed() {
        let mut a = PnsRng::with_seed(42);
        let mut b = PnsRng::with_seed(43);
        assert_ne!(a.next_float(), b.next_float());
    }

    #[test]
    fn pns_gain_doubles_per_four_sf() {
        let g0 = pns_gain(0);
        let g4 = pns_gain(4);
        assert!((g4 / g0 - 2.0).abs() < 1e-4);
    }

    #[test]
    fn apply_pns_long_fills_noise_band() {
        // Build an IcsInfo for 44.1kHz long, max_sfb=5.
        let info = IcsInfo {
            sf_index: 4,
            max_sfb: 5,
            num_window_groups: 1,
            window_group_length: [1, 0, 0, 0, 0, 0, 0, 0],
            ..Default::default()
        };
        // Section data: all zero except sfb 2 which is noise.
        let mut sec = SectionData {
            sfb_cb: vec![0, 0, NOISE_HCB, 0, 0],
        };
        // Mark bands zeroed so we can detect fill.
        let _ = &mut sec;
        let sf = vec![0, 0, 120, 0, 0];
        let mut spec = [0.0f32; SPEC_LEN];
        let mut rng = PnsRng::with_seed(7);
        apply_pns_long(&mut spec, &info, &sec, &sf, &mut rng, None, None);
        let swb = SWB_LONG[4];
        let start = swb[2] as usize;
        let end = swb[3] as usize;
        let mut nonzero = 0;
        for s in start..end {
            if spec[s].abs() > 0.0 {
                nonzero += 1;
            }
        }
        assert!(nonzero > 0, "noise band should be filled");
        // Non-noise bands should stay zero.
        for s in 0..start {
            assert_eq!(spec[s], 0.0);
        }
        for s in end..SPEC_LEN {
            assert_eq!(spec[s], 0.0);
        }
    }

    #[test]
    fn apply_pns_long_correlated_mode() {
        let info = IcsInfo {
            sf_index: 4,
            max_sfb: 3,
            num_window_groups: 1,
            window_group_length: [1, 0, 0, 0, 0, 0, 0, 0],
            ..Default::default()
        };
        let sec = SectionData {
            sfb_cb: vec![NOISE_HCB, NOISE_HCB, NOISE_HCB],
        };
        let sf = vec![100, 100, 100];
        // ms_used marks sfb 0 + 2 for correlation; sfb 1 independent.
        let ms = vec![true, false, true];
        let mut spec0 = [0.0f32; SPEC_LEN];
        let mut spec1 = [0.0f32; SPEC_LEN];
        let mut rng = PnsRng::with_seed(9);
        apply_pns_long(
            &mut spec0,
            &info,
            &sec,
            &sf,
            &mut rng,
            Some(&mut spec1),
            Some(&ms),
        );
        let swb = SWB_LONG[4];
        // sfb 0 range: should match.
        for s in (swb[0] as usize)..(swb[1] as usize) {
            assert_eq!(spec0[s], spec1[s], "sfb0 should be correlated");
        }
        // sfb 1 range: independent — we just require they differ somewhere.
        let mut differs = false;
        for s in (swb[1] as usize)..(swb[2] as usize) {
            if spec0[s] != spec1[s] {
                differs = true;
                break;
            }
        }
        assert!(differs, "sfb1 should be independent noise");
        // sfb 2 range: correlated again.
        for s in (swb[2] as usize)..(swb[3] as usize) {
            assert_eq!(spec0[s], spec1[s], "sfb2 should be correlated");
        }
    }
}

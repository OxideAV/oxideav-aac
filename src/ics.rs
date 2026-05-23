//! Individual Channel Stream (ICS) — ISO/IEC 14496-3 §4.6.2.
//!
//! Decodes ics_info, scalefactor data, spectral data, and runs inverse
//! quantisation + scalefactor reconstruction. This is the bulk of an AAC
//! channel pair decode — both the SCE (mono) and CPE (stereo) elements
//! call into this module.

use oxideav_core::{Error, Result};

use crate::huffman::{decode_scalefactor_delta, decode_spectral, spectral_book};
use crate::sfband::{num_swb_long, num_swb_short, SWB_LONG, SWB_SHORT};
use crate::syntax::{WindowSequence, WindowShape};
use oxideav_core::bits::BitReader;

/// Maximum scalefactor bands per group (long-window worst case).
pub const MAX_SFB: usize = 51;
/// Number of spectral coefficients in a single AAC frame (long block).
pub const SPEC_LEN: usize = 1024;
/// Special codebook IDs.
pub const ZERO_HCB: u8 = 0;
pub const FIRST_PAIR_HCB: u8 = 5;
pub const ESC_HCB: u8 = 11;
pub const NOISE_HCB: u8 = 13;
pub const INTENSITY_HCB2: u8 = 14;
pub const INTENSITY_HCB: u8 = 15;

#[derive(Clone, Debug, Default)]
pub struct IcsInfo {
    pub window_sequence: WindowSequence,
    pub window_shape: WindowShape,
    pub max_sfb: u8,
    pub num_window_groups: u8,
    /// `window_group_length[g]` — number of windows in group `g` (sum=8 for short, =1 for long).
    pub window_group_length: [u8; 8],
    /// `scale_factor_grouping` raw bits (only meaningful for short-windows).
    pub scale_factor_grouping: u8,
    pub predictor_data_present: bool,
    pub sf_index: u8,
}

impl IcsInfo {
    pub fn num_swb(&self) -> usize {
        if self.window_sequence.is_eight_short() {
            num_swb_short(self.sf_index)
        } else {
            num_swb_long(self.sf_index)
        }
    }
}

/// LTP coefficient table — ISO/IEC 14496-3 §4.6.7.2 Table 4.98. The 3-bit
/// `ltp_coef` index selects the single-tap predictor gain applied to the
/// lagged time-domain history.
pub const LTP_COEF: [f32; 8] = [
    0.570_829, 0.696_616, 0.813_004, 0.911_304, 0.984_900, 1.067_894, 1.194_601, 1.369_533,
];

/// Parsed `ltp_data()` for AAC-LD (objectType 23) — ISO/IEC 14496-3
/// §4.6.7 / Table 4.49 (the `AudioObjectType == ER AAC LD` branch).
///
/// LD is long-window-only, so only the per-sfb `ltp_long_used` flags are
/// present (no `ltp_short_used` / `ltp_short_lag`). The lag is a 10-bit
/// value (range 0..1023); when `ltp_lag_update == 0` the previous frame's
/// lag is reused, which the caller resolves via the per-channel
/// `ltp_prev_lag` state before constructing this struct.
#[derive(Clone, Debug, Default)]
pub struct LtpData {
    /// Resolved long-term-prediction lag in samples (0..1023 for LD).
    pub lag: u16,
    /// Predictor gain from [`LTP_COEF`] (the resolved `ltp_coef` table value).
    pub coef: f32,
    /// `ltp_long_used[sfb]` — one flag per coded scalefactor band.
    pub long_used: Vec<bool>,
}

/// Parse the AAC-LD variant of `ltp_data()` (ISO/IEC 14496-3 §4.6.7,
/// Table 4.49, `AudioObjectType == ER AAC LD` branch):
///
/// ```text
/// ltp_lag_update                            1   uimsbf
/// if (ltp_lag_update) ltp_lag               10  uimsbf
/// else                ltp_lag = ltp_prev_lag
/// ltp_coef                                  3   uimsbf
/// for (sfb = 0; sfb < max_sfb; sfb++)
///     ltp_long_used[sfb]                    1   uimsbf
/// ```
///
/// `prev_lag` carries the channel's previous-frame lag for the
/// `ltp_lag_update == 0` reuse case; it is updated in place to the
/// resolved lag so the next frame can reuse it again.
pub fn parse_ltp_data_ld(
    br: &mut BitReader<'_>,
    max_sfb: usize,
    prev_lag: &mut u16,
) -> Result<LtpData> {
    let lag = if br.read_bit()? {
        br.read_u32(10)? as u16
    } else {
        *prev_lag
    };
    *prev_lag = lag;
    let coef_idx = br.read_u32(3)? as usize;
    let coef = LTP_COEF[coef_idx];
    let mut long_used = vec![false; max_sfb];
    for slot in long_used.iter_mut() {
        *slot = br.read_bit()?;
    }
    Ok(LtpData {
        lag,
        coef,
        long_used,
    })
}

/// Maximum number of scalefactor bands over which long-term prediction can
/// be signalled in a non-LD long window — ISO/IEC 14496-3 §4.6.7.3. The
/// `ltp_long_used[]` flags are read for `min(max_sfb, MAX_LTP_LONG_SFB)`
/// bands; bands beyond this index are never predicted regardless of
/// `max_sfb` (they hold no `ltp_long_used` bit in the bitstream).
pub const MAX_LTP_LONG_SFB: usize = 40;

/// Parsed `ltp_data()` for the non-LD audio object types (AAC-LTP /
/// AOT 4, and the AAC-Scalable / ER variants that share the same syntax) —
/// ISO/IEC 14496-3 §4.6.7 / Table 4.49 (the `else` branch, i.e.
/// `AudioObjectType != ER AAC LD`).
///
/// Unlike the LD branch (see [`LtpData`]) the lag is an 11-bit value
/// (range 0..2047) transmitted unconditionally — there is no
/// `ltp_lag_update` previous-frame reuse. For long windows the per-sfb
/// `long_used` flags select which bands receive the predicted spectrum;
/// for short windows the per-window `short_used` / `short_lag` fields
/// drive the per-subframe predictor instead (§4.6.7.3 short-window path).
#[derive(Clone, Debug, Default)]
pub struct LtpDataNonLd {
    /// 11-bit long-term-prediction lag in samples (0..2047). For short
    /// windows the effective per-window lag is `lag + short_lag[w]`.
    pub lag: u16,
    /// Predictor gain from [`LTP_COEF`] (the resolved `ltp_coef` table
    /// value). The same coefficient applies to all short windows in the
    /// frame (§4.6.7.2).
    pub coef: f32,
    /// `ltp_long_used[sfb]` — one flag per coded scalefactor band, read
    /// for `min(max_sfb, MAX_LTP_LONG_SFB)` bands on long windows. Empty
    /// when the frame uses short windows.
    pub long_used: Vec<bool>,
    /// `ltp_short_used[w]` — one flag per window on short blocks. Empty on
    /// long windows.
    pub short_used: Vec<bool>,
    /// Per-window relative lag offset (`ltp_short_lag[w]`, range −8..7) for
    /// short blocks; 0 where `ltp_short_lag_present[w] == 0` or
    /// `ltp_short_used[w] == 0`. Empty on long windows. The effective
    /// per-window read offset is `lag + short_lag[w]` (§4.6.7.3).
    pub short_lag: Vec<i8>,
}

/// Parse the non-LD variant of `ltp_data()` (ISO/IEC 14496-3 §4.6.7,
/// Table 4.49, `AudioObjectType != ER AAC LD` branch):
///
/// ```text
/// ltp_lag                                       11  uimsbf
/// ltp_coef                                      3   uimsbf
/// if (window_sequence == EIGHT_SHORT_SEQUENCE) {
///     for (w = 0; w < num_windows; w++) {
///         ltp_short_used[w]                     1   uimsbf
///         if (ltp_short_used[w]) {
///             ltp_short_lag_present[w]          1   uimsbf
///             if (ltp_short_lag_present[w])
///                 ltp_short_lag[w]              4   uimsbf
///         }
///     }
/// } else {
///     for (sfb = 0; sfb < max_sfb; sfb++)
///         ltp_long_used[sfb]                    1   uimsbf
/// }
/// ```
///
/// `ltp_short_lag[w]` is a 4-bit field decoded as a signed relative delay
/// from −8 to 7 (§4.6.7.2: *"relative delay … from −8 to 7"*), i.e.
/// `short_lag = ltp_short_lag - 8`. `is_short` selects the short-window
/// path and `num_windows` is the per-group window count (8 for an
/// EIGHT_SHORT block). `max_sfb` bounds the long-window flag count and is
/// additionally clamped to [`MAX_LTP_LONG_SFB`] per §4.6.7.3.
pub fn parse_ltp_data(
    br: &mut BitReader<'_>,
    is_short: bool,
    num_windows: usize,
    max_sfb: usize,
) -> Result<LtpDataNonLd> {
    let lag = br.read_u32(11)? as u16;
    let coef_idx = br.read_u32(3)? as usize;
    let coef = LTP_COEF[coef_idx];
    let mut long_used = Vec::new();
    let mut short_used = Vec::new();
    let mut short_lag = Vec::new();
    if is_short {
        short_used = vec![false; num_windows];
        short_lag = vec![0i8; num_windows];
        for w in 0..num_windows {
            short_used[w] = br.read_bit()?;
            if short_used[w] {
                let short_lag_present = br.read_bit()?;
                if short_lag_present {
                    // 4-bit field, signed relative delay −8..7.
                    short_lag[w] = br.read_u32(4)? as i8 - 8;
                }
            }
        }
    } else {
        let count = max_sfb.min(MAX_LTP_LONG_SFB);
        long_used = vec![false; count];
        for slot in long_used.iter_mut() {
            *slot = br.read_bit()?;
        }
    }
    Ok(LtpDataNonLd {
        lag,
        coef,
        long_used,
        short_used,
        short_lag,
    })
}

/// LD-specific variant of [`parse_ics_info`] — ISO/IEC 14496-3 §4.6.17.
///
/// AAC-LD always uses long blocks (no `EIGHT_SHORT_SEQUENCE`), uses the
/// LD SWB table at the given frame length, and replaces the
/// AAC-Main backward predictor's `predictor_data` block with the LD
/// long-term-prediction (LTP) side-info stream.
///
/// Per ISO/IEC 14496-3 Table 4.6, when `predictor_data_present == 1` and
/// `audioObjectType != 1` the body is the LTP path:
///
/// ```text
/// ltp_data_present                          1   uimsbf
/// if (ltp_data_present) ltp_data();
/// if (common_window) {
///     ltp_data_present                      1   uimsbf  (second channel)
///     if (ltp_data_present) ltp_data();
/// }
/// ```
///
/// `swb_offsets` is the LD-specific scalefactor-band offset table for the
/// (sf_index, frame_length) pair (see `ld_eld::swb_ld_for`). The
/// resulting `IcsInfo::max_sfb` is clamped to fit inside the table.
///
/// `common_window` selects whether the shared ics_info carries a second
/// channel's `ltp_data` (the CPE common_window case). `prev_lags` carries
/// the per-channel previous-frame LTP lag for the `ltp_lag_update == 0`
/// reuse case and is updated in place. The returned `[Option<LtpData>; 2]`
/// holds the parsed LTP side info for channel 0 (and channel 1 when
/// `common_window`); entries are `None` when LTP is disabled for that
/// channel this frame.
pub fn parse_ics_info_ld(
    br: &mut BitReader<'_>,
    sf_index: u8,
    swb_offsets: &[u16],
    common_window: bool,
    prev_lags: &mut [u16; 2],
) -> Result<(IcsInfo, [Option<LtpData>; 2])> {
    let _ics_reserved_bit = br.read_bit()?;
    let window_sequence = WindowSequence::from_u32(br.read_u32(2)?);
    let window_shape = WindowShape::from_bit(br.read_u32(1)?);
    // §4.6.17.2.2: block switching is disabled in the low-delay codec.
    // Only `OnlyLong` is conformant. A non-zero window_sequence in an
    // LD bitstream is malformed; reject up-front so we never try to
    // decode 8 short windows through the 512-sample LD filterbank.
    if !matches!(window_sequence, WindowSequence::OnlyLong) {
        return Err(Error::invalid(
            "AAC-LD: ics_info.window_sequence must be OnlyLong (§4.6.17.2.2)",
        ));
    }
    let max_sfb_raw = br.read_u32(6)? as u8;
    let num_swb = swb_offsets.len().saturating_sub(1);
    let max_sfb = (max_sfb_raw as usize).min(num_swb) as u8;
    let predictor_data_present = br.read_bit()?;
    let mut ltp: [Option<LtpData>; 2] = [None, None];
    if predictor_data_present {
        // §4.6.7 / Table 4.49 — LTP side info. The per-sfb
        // `ltp_long_used` flags are bounded by the (clamped) max_sfb so a
        // non-conformant max_sfb can never read past the SWB table.
        let ltp_data_present = br.read_bit()?;
        if ltp_data_present {
            ltp[0] = Some(parse_ltp_data_ld(br, max_sfb as usize, &mut prev_lags[0])?);
        }
        if common_window {
            let ltp_data_present_1 = br.read_bit()?;
            if ltp_data_present_1 {
                ltp[1] = Some(parse_ltp_data_ld(br, max_sfb as usize, &mut prev_lags[1])?);
            }
        }
    }
    let mut info = IcsInfo {
        window_sequence,
        window_shape,
        sf_index,
        max_sfb,
        predictor_data_present,
        num_window_groups: 1,
        window_group_length: [1u8, 0, 0, 0, 0, 0, 0, 0],
        ..Default::default()
    };
    info.window_group_length[0] = 1;
    Ok((info, ltp))
}

pub fn parse_ics_info(br: &mut BitReader<'_>, sf_index: u8) -> Result<IcsInfo> {
    let _ics_reserved_bit = br.read_bit()?;
    let window_sequence = WindowSequence::from_u32(br.read_u32(2)?);
    let window_shape = WindowShape::from_bit(br.read_u32(1)?);
    let mut info = IcsInfo {
        window_sequence,
        window_shape,
        sf_index,
        ..Default::default()
    };

    if window_sequence.is_eight_short() {
        info.max_sfb = br.read_u32(4)? as u8;
        info.scale_factor_grouping = br.read_u32(7)? as u8;
        // Bit 6 of grouping = group with window 1, bit 5 = win 2, ..., bit 0 = win 7.
        // Window 0 always starts a new group.
        let mut groups = 1u8;
        let mut lengths = [1u8; 8];
        let mut cur_len = 1u8;
        for w in 1..8 {
            let bit = (info.scale_factor_grouping >> (6 - (w - 1))) & 1;
            if bit == 1 {
                cur_len += 1;
            } else {
                lengths[(groups - 1) as usize] = cur_len;
                groups += 1;
                cur_len = 1;
            }
        }
        lengths[(groups - 1) as usize] = cur_len;
        info.num_window_groups = groups;
        info.window_group_length = lengths;
    } else {
        info.max_sfb = br.read_u32(6)? as u8;
        info.predictor_data_present = br.read_bit()?;
        if info.predictor_data_present {
            // ISO/IEC 14496-3 §4.5.2.1: AAC-LC streams must keep
            // `predictor_data_present == 0` (prediction is reserved
            // for AAC-Main / AAC-LTP). Workspace task #759 follow-up
            // surfaced an 88.2 kHz / 5.1 ADTS frame where the bit is
            // set; libavcodec tolerates it and emits PCM. Mirror
            // that behaviour by consuming the predictor_data() block
            // exactly so the bit-reader stays aligned, then ignore
            // the parsed prediction info (we don't run the Main/LTP
            // predictor synthesis in the LC decoder). Per Table 4.55:
            //   predictor_reset (1)
            //   if (predictor_reset)
            //       predictor_reset_group_number (5)
            //   for (sfb = 0; sfb < min(max_sfb, MAX_PRED_SFB); sfb++)
            //       prediction_used[sfb] (1)
            // MAX_PRED_SFB depends on sf_index; per Table 4.74,
            // max_pred_sfb = 33 for sf_index 0..=4 and decreases for
            // lower sample rates. We bound by max_sfb for safety.
            let predictor_reset = br.read_bit()?;
            if predictor_reset {
                let _predictor_reset_group_number = br.read_u32(5)?;
            }
            // Consume one bit per coded scalefactor band — bound by
            // max_sfb (already clamped to num_swb above) so we never
            // read past the spec-allowed max_pred_sfb of 40.
            let max_pred_sfb = (info.max_sfb as usize).min(40);
            for _ in 0..max_pred_sfb {
                let _prediction_used = br.read_bit()?;
            }
        }
        info.num_window_groups = 1;
        info.window_group_length[0] = 1;
    }

    // Per ISO/IEC 14496-3 Table 4.110, max_sfb is bounded by
    // num_swb(sample_rate, window_shape). The bit-field allows up to 63
    // (long) / 15 (short), but the SWB tables only contain
    // num_swb(sf_index)+1 offsets. A non-conformant stream that codes
    // max_sfb beyond this bound would later index past
    // SWB_LONG[sf_index] / SWB_SHORT[sf_index] in the spectrum-decode
    // path.
    //
    // Workspace task #744 added a hard reject here. Task #759 then
    // revealed a follow-up case: an 88.2 kHz / 5.1 ADTS run where the
    // *second* element (a CPE per-channel ICS) codes max_sfb=60 with
    // sf_index=1 (num_swb_long = 41). libavcodec emits a frame for
    // this stream — it tolerates the overrun by clamping max_sfb to
    // num_swb, which leaves the unreachable upper bands silent. We
    // mirror that behaviour so any libavcodec-accepted ADTS frame
    // also produces output here. Rejecting outright traps the same
    // class of malformed-but-tolerated streams that motivated the
    // gain_control tolerance fix in `decode_ics`.
    let num_swb = info.num_swb();
    if info.max_sfb as usize > num_swb {
        info.max_sfb = num_swb as u8;
    }

    Ok(info)
}

/// Section data: codebook + length-in-sfbs per (group, section).
#[derive(Clone, Debug, Default)]
pub struct SectionData {
    /// `sfb_cb[g * max_sfb + sfb]` — codebook index for that band/group.
    pub sfb_cb: Vec<u8>,
}

impl SectionData {
    pub fn empty() -> Self {
        Self::default()
    }
}

pub fn parse_section_data(br: &mut BitReader<'_>, info: &IcsInfo) -> Result<SectionData> {
    let max_sfb = info.max_sfb as usize;
    let groups = info.num_window_groups as usize;
    let sect_esc_val = if info.window_sequence.is_eight_short() {
        (1u32 << 3) - 1
    } else {
        (1u32 << 5) - 1
    };
    let sect_bits = if info.window_sequence.is_eight_short() {
        3
    } else {
        5
    };

    let mut sfb_cb = vec![0u8; groups * max_sfb];
    for g in 0..groups {
        let mut k = 0usize;
        while k < max_sfb {
            let cb = br.read_u32(4)? as u8;
            let mut len = 0u32;
            loop {
                let inc = br.read_u32(sect_bits)?;
                len += inc;
                if inc != sect_esc_val {
                    break;
                }
            }
            for _ in 0..len {
                if k >= max_sfb {
                    return Err(Error::invalid("AAC: section overruns max_sfb"));
                }
                sfb_cb[g * max_sfb + k] = cb;
                k += 1;
            }
        }
    }
    Ok(SectionData { sfb_cb })
}

/// Decode scalefactor data — fills `sf[g * max_sfb + sfb]` with quantised
/// scalefactors per ISO §4.6.2.3.
///
/// `global_gain` is the seed; deltas are decoded from the SF Huffman code.
/// IS bands and noise (PNS) bands use a separate accumulator/seed each.
pub fn parse_scalefactors(
    br: &mut BitReader<'_>,
    info: &IcsInfo,
    sec: &SectionData,
    global_gain: u8,
) -> Result<Vec<i32>> {
    let max_sfb = info.max_sfb as usize;
    let groups = info.num_window_groups as usize;
    let mut sf = vec![0i32; groups * max_sfb];

    let mut g_gain: i32 = global_gain as i32;
    let mut g_noise: i32 = global_gain as i32 - 90; // PNS seed (spec §4.6.2.3)
    let mut g_is: i32 = 0; // Intensity seed

    let mut noise_seed_set = false;

    for g in 0..groups {
        for sfb in 0..max_sfb {
            let cb = sec.sfb_cb[g * max_sfb + sfb];
            match cb {
                ZERO_HCB => sf[g * max_sfb + sfb] = 0,
                INTENSITY_HCB | INTENSITY_HCB2 => {
                    let d = decode_scalefactor_delta(br)?;
                    g_is += d;
                    sf[g * max_sfb + sfb] = g_is;
                }
                NOISE_HCB => {
                    if !noise_seed_set {
                        // First PNS band — read 9-bit dpcm_noise_nrg seed.
                        let raw = br.read_u32(9)? as i32;
                        g_noise += raw - 256;
                        noise_seed_set = true;
                    } else {
                        let d = decode_scalefactor_delta(br)?;
                        g_noise += d;
                    }
                    sf[g * max_sfb + sfb] = g_noise;
                }
                _ => {
                    let d = decode_scalefactor_delta(br)?;
                    g_gain += d;
                    sf[g * max_sfb + sfb] = g_gain;
                }
            }
        }
    }
    Ok(sf)
}

/// Group/window helpers.
pub fn group_starts(info: &IcsInfo) -> Vec<usize> {
    // Cumulative window starts for each group. Long block: [0, 1].
    let mut starts = Vec::with_capacity(info.num_window_groups as usize + 1);
    starts.push(0);
    let mut cum = 0usize;
    for g in 0..info.num_window_groups as usize {
        cum += info.window_group_length[g] as usize;
        starts.push(cum);
    }
    starts
}

/// Decode spectral data into `coef[1024]`. Implements §4.6.4 +§4.6.6 + §4.6.7
/// (Huffman → quantised spectrum → inverse-quant cubic root → scalefactor).
pub fn decode_spectrum_long(
    br: &mut BitReader<'_>,
    info: &IcsInfo,
    sec: &SectionData,
    sf: &[i32],
    coef: &mut [f32; SPEC_LEN],
) -> Result<()> {
    let swb_offsets = SWB_LONG[info.sf_index as usize];
    decode_spectrum_long_with_swb(br, info, sec, sf, swb_offsets, coef, SPEC_LEN)
}

/// Generalised long-spectrum decoder: same algorithm as
/// [`decode_spectrum_long`], but with an explicit SWB offset table and
/// explicit total spectrum length. Used by AAC-LD (objectType 23) where
/// the spectrum is 512 (or 480) coefficients long with its own
/// scalefactor-band layout (§4.5.4 Tables 4.137-4.156).
///
/// `coef` is a fixed-size 1024-coefficient buffer; the LD path zero-fills
/// the unused tail so the rest of the decoder pipeline (TNS / PNS / IS /
/// IMDCT) sees a consistent layout. The IMDCT helper only reads the first
/// `spec_len` slots.
pub fn decode_spectrum_long_with_swb(
    br: &mut BitReader<'_>,
    info: &IcsInfo,
    sec: &SectionData,
    sf: &[i32],
    swb_offsets: &[u16],
    coef: &mut [f32; SPEC_LEN],
    spec_len: usize,
) -> Result<()> {
    let max_sfb = info.max_sfb as usize;
    if max_sfb >= swb_offsets.len() {
        return Err(Error::invalid(
            "AAC: max_sfb exceeds SWB table length (LD/LC spectrum decode)",
        ));
    }
    for sfb in 0..max_sfb {
        let cb = sec.sfb_cb[sfb];
        let start = swb_offsets[sfb] as usize;
        let end = swb_offsets[sfb + 1] as usize;
        if cb == ZERO_HCB {
            for s in start..end {
                coef[s] = 0.0;
            }
            continue;
        }
        if cb == NOISE_HCB || cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
            // PNS / IS — handled separately. Leave zeros in spectrum for now.
            for s in start..end {
                coef[s] = 0.0;
            }
            continue;
        }
        let book = spectral_book(cb)?;
        let dim = book.dim as usize;
        let scale = sf_to_gain(sf[sfb]);
        let mut s = start;
        while s < end {
            let vals = decode_spectral(br, book)?;
            for k in 0..dim {
                if s + k >= end {
                    break;
                }
                let q = vals[k] as f32;
                coef[s + k] = inv_quant(q) * scale;
            }
            s += dim;
        }
    }
    // Fill any unused trailing bands with zero (within the spectrum
    // range) plus zero the unused tail past `spec_len`.
    let last = swb_offsets[max_sfb] as usize;
    for s in last..spec_len.min(SPEC_LEN) {
        coef[s] = 0.0;
    }
    for s in spec_len.min(SPEC_LEN)..SPEC_LEN {
        coef[s] = 0.0;
    }
    Ok(())
}

/// Decode short-window grouped spectral data into `coef[1024]`. The grouped
/// layout is: for each group (g) of length L_g, the L_g sub-windows of 128
/// coefs are interleaved per-sfb.
pub fn decode_spectrum_short(
    br: &mut BitReader<'_>,
    info: &IcsInfo,
    sec: &SectionData,
    sf: &[i32],
    coef: &mut [f32; SPEC_LEN],
) -> Result<()> {
    let swb_offsets = SWB_SHORT[info.sf_index as usize];
    let max_sfb = info.max_sfb as usize;
    let groups = info.num_window_groups as usize;
    let starts = group_starts(info);

    // Zero the entire spectrum first; bands not coded stay zero.
    for c in coef.iter_mut() {
        *c = 0.0;
    }

    for g in 0..groups {
        let group_len = info.window_group_length[g] as usize;
        let win_start_offset = starts[g] * 128;
        for sfb in 0..max_sfb {
            let cb = sec.sfb_cb[g * max_sfb + sfb];
            let band_start = swb_offsets[sfb] as usize;
            let band_end = swb_offsets[sfb + 1] as usize;
            let band_len = band_end - band_start;
            // grouped layout: L_g sub-windows interleaved by sfb.
            // Order: w=0, sfb's range; w=1, sfb's range; ...
            // Memory: write into coef positions `(win_start_offset + w*128 + band_start) ..`
            if cb == ZERO_HCB || cb == NOISE_HCB || cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
                // All zero (we'll fill IS/PNS later).
                continue;
            }
            let book = spectral_book(cb)?;
            let dim = book.dim as usize;
            let scale = sf_to_gain(sf[g * max_sfb + sfb]);
            // Decode group_len * band_len coefficients sequentially.
            let mut local = vec![0f32; group_len * band_len];
            let mut s = 0;
            while s < group_len * band_len {
                let vals = decode_spectral(br, book)?;
                for k in 0..dim {
                    if s + k < local.len() {
                        local[s + k] = inv_quant(vals[k] as f32) * scale;
                    }
                }
                s += dim;
            }
            // De-interleave into coef.
            for w in 0..group_len {
                for j in 0..band_len {
                    let dst = win_start_offset + w * 128 + band_start + j;
                    let src = w * band_len + j;
                    coef[dst] = local[src];
                }
            }
        }
    }
    Ok(())
}

/// Inverse quantisation: x = sign(q) * |q|^(4/3).
#[inline]
pub fn inv_quant(q: f32) -> f32 {
    if q == 0.0 {
        0.0
    } else {
        let a = q.abs();
        let p = a.powf(4.0 / 3.0);
        if q < 0.0 {
            -p
        } else {
            p
        }
    }
}

/// Convert a scalefactor (-100..150 typical) to a linear gain.
/// gain = 2^((sf - 100)/4) per ISO §4.6.2.3.
#[inline]
pub fn sf_to_gain(sf: i32) -> f32 {
    let exp = (sf - 100) as f32 / 4.0;
    2.0f32.powf(exp)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inv_quant_basic() {
        assert_eq!(inv_quant(0.0), 0.0);
        let v = inv_quant(8.0);
        // 8^(4/3) = 16
        assert!((v - 16.0).abs() < 1e-3);
        let v = inv_quant(-1.0);
        assert!((v + 1.0).abs() < 1e-6);
    }

    #[test]
    fn sf_to_gain_unity() {
        // sf=100 -> gain=1.
        assert!((sf_to_gain(100) - 1.0).abs() < 1e-6);
        // sf=104 -> gain=2.
        assert!((sf_to_gain(104) - 2.0).abs() < 1e-3);
    }

    /// `parse_ltp_data` (non-LD long-window branch) consumes Table 4.49's
    /// `else` path exactly: `ltp_lag`(11) + `ltp_coef`(3) + per-sfb
    /// `ltp_long_used`(1). Note the 11-bit lag (range 0..2047) and the
    /// absence of the LD-only `ltp_lag_update` reuse bit.
    #[test]
    fn parse_ltp_data_long_roundtrip() {
        use oxideav_core::bits::BitWriter;
        let mut bw = BitWriter::new();
        bw.write_u32(1500, 11); // ltp_lag (> 1023 — only reachable with 11 bits)
        bw.write_u32(6, 3); // ltp_coef index 6
                            // 5 long_used flags: 1,0,1,1,0
        for b in [true, false, true, true, false] {
            bw.write_bit(b);
        }
        let bytes = bw.into_bytes();
        let mut br = BitReader::new(&bytes);
        let ltp = parse_ltp_data(&mut br, false, 1, 5).unwrap();
        assert_eq!(ltp.lag, 1500);
        assert_eq!(ltp.coef, LTP_COEF[6]);
        assert_eq!(ltp.long_used, vec![true, false, true, true, false]);
        assert!(ltp.short_used.is_empty());
        assert!(ltp.short_lag.is_empty());
    }

    /// The long-window flag count is clamped to `MAX_LTP_LONG_SFB` (40)
    /// even when `max_sfb` exceeds it — §4.6.7.3. Bands beyond index 40
    /// carry no `ltp_long_used` bit.
    #[test]
    fn parse_ltp_data_long_clamps_to_max_ltp_sfb() {
        use oxideav_core::bits::BitWriter;
        let mut bw = BitWriter::new();
        bw.write_u32(0, 11);
        bw.write_u32(0, 3);
        // Write exactly MAX_LTP_LONG_SFB used bits (all true) plus a sentinel.
        for _ in 0..MAX_LTP_LONG_SFB {
            bw.write_bit(true);
        }
        bw.write_bit(false); // sentinel that must NOT be consumed
        let bytes = bw.into_bytes();
        let mut br = BitReader::new(&bytes);
        // max_sfb = 51 (worst case) but only 40 flags should be read.
        let ltp = parse_ltp_data(&mut br, false, 1, 51).unwrap();
        assert_eq!(ltp.long_used.len(), MAX_LTP_LONG_SFB);
        assert!(ltp.long_used.iter().all(|&b| b));
        // The sentinel bit is still unread — confirm by reading it now.
        assert!(!br.read_bit().unwrap());
    }

    /// `parse_ltp_data` (short-window branch) walks the per-window
    /// `ltp_short_used` / `ltp_short_lag_present` / `ltp_short_lag` nest of
    /// Table 4.49 and decodes the 4-bit `ltp_short_lag` as a signed
    /// relative delay (−8..7 = field − 8).
    #[test]
    fn parse_ltp_data_short_roundtrip() {
        use oxideav_core::bits::BitWriter;
        let mut bw = BitWriter::new();
        bw.write_u32(42, 11); // ltp_lag
        bw.write_u32(2, 3); // ltp_coef
                            // 3 windows:
                            //  w0: used=1, lag_present=1, short_lag field=12 (-> +4)
                            //  w1: used=0
                            //  w2: used=1, lag_present=0  (short_lag stays 0)
        bw.write_bit(true); // w0 used
        bw.write_bit(true); // w0 lag_present
        bw.write_u32(12, 4); // w0 short_lag field (12-8 = +4)
        bw.write_bit(false); // w1 used
        bw.write_bit(true); // w2 used
        bw.write_bit(false); // w2 lag_present = 0
        let bytes = bw.into_bytes();
        let mut br = BitReader::new(&bytes);
        let ltp = parse_ltp_data(&mut br, true, 3, 0).unwrap();
        assert_eq!(ltp.lag, 42);
        assert_eq!(ltp.coef, LTP_COEF[2]);
        assert_eq!(ltp.short_used, vec![true, false, true]);
        assert_eq!(ltp.short_lag, vec![4i8, 0, 0]);
        assert!(ltp.long_used.is_empty());
    }
}

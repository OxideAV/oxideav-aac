//! Temporal Noise Shaping (TNS) — ISO/IEC 14496-3 §4.6.9.
//!
//! TNS is an optional AAC-LC tool that applies an all-pole LPC filter in the
//! frequency domain to shape coding-noise in the time domain. The encoder
//! forward-filters spectral lines with a predictor; the decoder runs the
//! inverse (all-pole) filter.
//!
//! Bit layout per window (§4.6.9.1, Table 4.52):
//!   * `n_filt`          — 2 bits long / 1 bit short (number of filters in this window)
//!   * for each filter:
//!       - `coef_res`     — 1 bit (once per window, precedes filters)
//!       - `length`       — 6 bits long / 4 bits short
//!       - `order`        — 5 bits long / 3 bits short
//!       - (if order > 0)
//!           - `direction`   — 1 bit
//!           - `coef_compress` — 1 bit
//!           - `coef[]`    — `coef_bits = 3 + coef_res - coef_compress` bits each
//!
//! Coefficient dequantisation (§4.6.9.3) converts each raw code to a reflection
//! coefficient in `(-1, 1)`, then Levinson-style recursion builds LPC
//! coefficients which the decoder applies with an all-pole IIR. Per the
//! spec convention `direction == 0` means **upward** (low → high index,
//! `inc = +1`) and `direction == 1` means **downward** (high → low,
//! starting at `end-1` with `inc = -1`).
//!
//! Band extent (§4.6.9.3 `tns_decode_frame`): the iteration variable
//! `bottom` is initialised to `num_swb` (NOT `min(TNS_MAX_BANDS, max_sfb)`).
//! On each filter, `top := bottom; bottom := max(top - length, 0)`, and
//! the filter's start/end bin indices are
//! `swb_offset[min(bottom|top, TNS_MAX_BANDS, max_sfb)]`. The cap is
//! applied to the indices, not to `bottom`.

use oxideav_core::{Error, Result};

use crate::ics::{group_starts, IcsInfo, SPEC_LEN};
use crate::sfband::SWB_SHORT;
use crate::syntax::WindowSequence;
use oxideav_core::bits::BitReader;

/// Max TNS filter order for AAC-LC (§4.6.9, Table 4.136): 12 long / 7 short.
pub const TNS_MAX_ORDER_LONG: u8 = 12;
pub const TNS_MAX_ORDER_SHORT: u8 = 7;
/// Max TNS filters per window: 3 long / 1 short.
pub const TNS_MAX_FILT_LONG: u8 = 3;
pub const TNS_MAX_FILT_SHORT: u8 = 1;

/// Default TNS `tns_max_sfb` table — §4.6.9.1, Table 4.139. One entry per
/// sampling-frequency-index. For AAC-LC, short windows use a fixed value of 14.
/// These values are the *long-window* max sfb covered by TNS by default.
pub const TNS_MAX_BANDS_LONG: [u8; 13] = [
    31, // 96000
    31, // 88200
    34, // 64000
    40, // 48000
    42, // 44100
    51, // 32000
    46, // 24000
    46, // 22050
    42, // 16000
    42, // 12000
    42, // 11025
    39, // 8000
    39, // 7350
];
pub const TNS_MAX_BANDS_SHORT: u8 = 14;

/// Parsed TNS filter (single filter of a window).
#[derive(Clone, Debug, Default)]
pub struct TnsFilter {
    pub length: u8,
    pub order: u8,
    /// Per §4.6.9.3: 0 = upward (low → high index), 1 = downward (high → low).
    pub direction: u8,
    pub coef_compress: u8,
    /// Raw 3- or 4-bit coefficient codes. Up to `order` entries.
    pub coef_raw: [u8; TNS_MAX_ORDER_LONG as usize],
}

/// Per-window TNS data (up to 3 filters long / 1 filter short).
#[derive(Clone, Debug, Default)]
pub struct TnsWindow {
    pub n_filt: u8,
    /// Per-window coefficient resolution (0 => 3-bit, 1 => 4-bit).
    pub coef_res: u8,
    pub filters: [TnsFilter; TNS_MAX_FILT_LONG as usize],
}

/// Parsed tns_data for a whole channel ICS. Indexed by window (for long blocks
/// `num_windows == 1`; for eight-short, 8 sub-windows — note some streams use
/// `num_window_groups`, but tns is per-sub-window not per-group).
#[derive(Clone, Debug, Default)]
pub struct TnsData {
    pub num_windows: u8,
    pub windows: [TnsWindow; 8],
}

/// Parse `tns_data()` per §4.6.9.1. Called when `tns_data_present == 1`.
pub fn parse_tns_data(
    br: &mut BitReader<'_>,
    window_sequence: WindowSequence,
    num_windows: u8,
) -> Result<TnsData> {
    let (n_filt_bits, length_bits, order_bits) = if window_sequence.is_eight_short() {
        (1u32, 4u32, 3u32)
    } else {
        (2u32, 6u32, 5u32)
    };
    let max_order = if window_sequence.is_eight_short() {
        TNS_MAX_ORDER_SHORT
    } else {
        TNS_MAX_ORDER_LONG
    };

    let mut tns = TnsData {
        num_windows,
        ..Default::default()
    };
    for w in 0..num_windows as usize {
        let n_filt = br.read_u32(n_filt_bits)? as u8;
        tns.windows[w].n_filt = n_filt;
        if n_filt == 0 {
            continue;
        }
        let coef_res = br.read_u32(1)? as u8;
        tns.windows[w].coef_res = coef_res;
        for f in 0..n_filt as usize {
            let length = br.read_u32(length_bits)? as u8;
            let order = br.read_u32(order_bits)? as u8;
            let mut filt = TnsFilter {
                length,
                order,
                ..Default::default()
            };
            if order > 0 {
                if order > max_order {
                    // Out-of-spec stream. Consume the expected bits to stay
                    // aligned but clamp order so we treat this filter as a
                    // no-op.
                    return Err(Error::invalid("AAC: TNS order exceeds max"));
                }
                filt.direction = br.read_u32(1)? as u8;
                filt.coef_compress = br.read_u32(1)? as u8;
                let coef_bits: u32 = 3 + coef_res as u32 - filt.coef_compress as u32;
                for o in 0..order as usize {
                    filt.coef_raw[o] = br.read_u32(coef_bits)? as u8;
                }
            }
            tns.windows[w].filters[f] = filt;
        }
    }
    Ok(tns)
}

/// Dequantise raw TNS coefficient codes into reflection coefficients.
/// ISO §4.6.9.3, function `tns_decode_coef`.
///
/// Spec pseudo-code (re-stated):
/// ```text
///   coef_res_bits = coef_res + 3            // 3 or 4
///   coef_res2     = coef_res_bits - coef_compress
///   s_mask        = sgn_mask[coef_res2-2]   // 0x2/0x4/0x8
///   n_mask        = neg_mask[coef_res2-2]   // ~0x3/~0x7/~0xf
///   tmp           = (code & s_mask) ? code | n_mask : code   // sign-extend
///                                                            // in transmitted-width
///   iqfac         = ((1 << (coef_res_bits-1)) - 0.5) / (PI/2)   // for tmp >= 0
///   iqfac_m       = ((1 << (coef_res_bits-1)) + 0.5) / (PI/2)   // for tmp <  0
///   parcor        = sin( tmp / (tmp >= 0 ? iqfac : iqfac_m) )
/// ```
///
/// Important: the divisor `iqfac` uses the **uncompressed** width
/// `coef_res_bits`, NOT `coef_res2`. With `coef_compress = 1` the magnitude
/// of `tmp` is halved (shorter transmission word) but the divisor is
/// unchanged — i.e. compression coarsens the quantiser step rather than
/// rescaling, and we must not "re-inflate" `tmp` by left-shifting it.
fn dequant_tns_coef(code: u8, coef_res: u8, coef_compress: u8) -> f32 {
    // Sign-extend within the transmitted bit width (coef_res2).
    let coef_res2 = 3 + coef_res - coef_compress;
    let sign_bit = 1u8 << (coef_res2 - 1);
    let tmp: i32 = if (code & sign_bit) != 0 {
        // Negative: extend sign by ORing in 1s above the transmitted MSB.
        let mask = !((1u8 << coef_res2).wrapping_sub(1));
        ((code | mask) as i8) as i32
    } else {
        code as i32
    };

    // iqfac uses the *uncompressed* coef_res_bits = coef_res + 3.
    let coef_res_bits = (3 + coef_res) as u32;
    let two_pow = (1u32 << (coef_res_bits - 1)) as f32; // 4 (res=0) or 8 (res=1)
    let half_pi = std::f32::consts::FRAC_PI_2;
    let iqfac = (two_pow - 0.5) / half_pi;
    let iqfac_m = (two_pow + 0.5) / half_pi;
    let denom = if tmp >= 0 { iqfac } else { iqfac_m };
    (tmp as f32 / denom).sin()
}

/// Convert parcor (reflection) coefficients to LPC coefficients using the
/// step-up recursion (§4.6.9.3 box "tns_decode_coef"). Output length = order.
fn parcor_to_lpc(parcor: &[f32], lpc: &mut [f32]) {
    let order = parcor.len();
    // tmp workspace for iterative build.
    let mut work = [0.0f32; TNS_MAX_ORDER_LONG as usize];
    for m in 0..order {
        let mut tmp = [0.0f32; TNS_MAX_ORDER_LONG as usize];
        for i in 0..m {
            tmp[i] = work[i] + parcor[m] * work[m - 1 - i];
        }
        for i in 0..m {
            work[i] = tmp[i];
        }
        work[m] = parcor[m];
    }
    for i in 0..order {
        lpc[i] = work[i];
    }
}

/// Apply a single TNS all-pole IIR filter in place to the spectral range
/// `[start, start + length)`. `lpc[0..order]` holds the LPC coefficients.
///
/// Per ISO/IEC 14496-3 §4.6.9.3, `direction == 0` is **upward** (low → high
/// index, `inc = +1`) and `direction == 1` is **downward** (high → low,
/// starting at `end-1` with `inc = -1`).
///
/// Bug history: an earlier version had these directions swapped (running
/// upward on direction=1 and downward on direction=0), which caused real
/// streams to apply the filter the wrong way around its taps and produce
/// large IMDCT-scale amplification on TNS-active CPE frames — visible in
/// e.g. solana-ad.mp4 frame 3852 where a swapped 9th-order direction-0
/// (spec: upward) filter ran top-down and turned a normal high-amplitude
/// transient into a saturated -32768 sample run.
fn apply_tns_filter_range(
    spec: &mut [f32; SPEC_LEN],
    start: usize,
    length: usize,
    lpc: &[f32],
    direction: u8,
) {
    let order = lpc.len();
    if order == 0 || length == 0 {
        return;
    }
    let end = (start + length).min(SPEC_LEN);
    if end <= start {
        return;
    }
    let n = end - start;
    let mut state = [0.0f32; TNS_MAX_ORDER_LONG as usize];

    if direction == 1 {
        // Downward: process indices end-1, end-2, ..., start.
        for i in 0..n {
            let idx = end - 1 - i;
            let mut y = spec[idx];
            for k in 0..order.min(i) {
                y -= lpc[k] * state[k];
            }
            // shift state
            for k in (1..order).rev() {
                state[k] = state[k - 1];
            }
            state[0] = y;
            spec[idx] = y;
        }
    } else {
        // Upward (direction == 0): low → high.
        for i in 0..n {
            let idx = start + i;
            let mut y = spec[idx];
            for k in 0..order.min(i) {
                y -= lpc[k] * state[k];
            }
            for k in (1..order).rev() {
                state[k] = state[k - 1];
            }
            state[0] = y;
            spec[idx] = y;
        }
    }
}

/// Test/internal-use re-export so sibling modules (`tns_analyse`) can call
/// the decoder's filter directly to verify encoder/decoder inversion.
#[doc(hidden)]
pub fn _apply_tns_filter_range_for_test(
    spec: &mut [f32; SPEC_LEN],
    start: usize,
    length: usize,
    lpc: &[f32],
    direction: u8,
) {
    apply_tns_filter_range(spec, start, length, lpc, direction);
}

/// Resolve the TNS start band for a window. ISO §4.6.9.1 — the span begins at
/// `tns_max_sfb` (see Table 4.139) scaled downward by the filter's `length`,
/// but most impls simply cap `tns_max_sfb` to `max_sfb` and run the filter
/// from the top band downward. We use the spec's default table.
pub fn tns_max_bands(window_sequence: WindowSequence, sf_index: u8) -> u8 {
    if window_sequence.is_eight_short() {
        TNS_MAX_BANDS_SHORT
    } else {
        TNS_MAX_BANDS_LONG
            .get(sf_index as usize)
            .copied()
            .unwrap_or(39)
    }
}

/// Run TNS on the spectrum for one ICS. Call this AFTER dequantisation and
/// scalefactor application, BEFORE M/S and IMDCT.
///
/// `max_sfb` is the ICS's max_sfb; `swb_offsets` gives band offsets (long or
/// short — for short blocks the caller passes per-sub-window slicing).
///
/// Per ISO/IEC 14496-3 §4.6.9.3 (`tns_decode_frame` pseudo-code):
/// ```text
///   bottom = num_swb;
///   for (f = 0; f < n_filt[w]; f++) {
///       top    = bottom;
///       bottom = max(top - length[w][f], 0);
///       ...
///       start  = swb_offset[min(bottom, TNS_MAX_BANDS, max_sfb)];
///       end    = swb_offset[min(top,    TNS_MAX_BANDS, max_sfb)];
///       ...
///   }
/// ```
/// Note that the **band-index cap** to `TNS_MAX_BANDS` and `max_sfb` is
/// applied to `start` / `end` AFTER computing them as raw (potentially
/// `> TNS_MAX_BANDS`) values. Initial `top = num_swb`, NOT `TNS_MAX_BANDS`.
///
/// Bug history: an earlier version pre-clamped `top` to
/// `min(TNS_MAX_BANDS, max_sfb)` before subtracting `length`, so for
/// 44.1 kHz long (TNS_MAX_BANDS = 42) a length-25 filter covered SFBs
/// 17..42 instead of the spec's 24..42 — i.e. 7 extra low-frequency
/// SFBs that contain the bulk of the spectrum's energy. The IIR filter
/// ran on the wrong region, amplifying low-band magnitudes ~2× and
/// producing PCM peaks that exceeded i16 range (e.g. solana-ad.mp4
/// frame 2566).
pub fn apply_tns_long(
    spec: &mut [f32; SPEC_LEN],
    tns: &TnsData,
    sf_index: u8,
    max_sfb: u8,
    swb_offsets: &[u16],
) {
    if tns.num_windows == 0 {
        return;
    }
    let win = &tns.windows[0];
    if win.n_filt == 0 {
        return;
    }
    let tns_max = tns_max_bands(WindowSequence::OnlyLong, sf_index) as usize;
    // num_swb = swb_offsets.len() - 1 (table is in 0..=num_swb form).
    let num_swb = swb_offsets.len().saturating_sub(1);
    // Spec: initial `bottom = num_swb`; then on each filter: top=bottom,
    // bottom = max(top - length, 0); start/end use `min(top|bottom,
    // TNS_MAX_BANDS, max_sfb)`.
    let mut bottom = num_swb;
    let cap = tns_max.min(max_sfb as usize);
    for f in 0..win.n_filt as usize {
        let filt = &win.filters[f];
        let top = bottom;
        let length = filt.length as usize;
        bottom = top.saturating_sub(length);
        if filt.order == 0 || length == 0 {
            // Spec lets length/order be zero — just advance bottom.
            continue;
        }
        let s_idx = bottom.min(cap);
        let e_idx = top.min(cap);
        if s_idx >= swb_offsets.len() || e_idx >= swb_offsets.len() {
            break;
        }
        let start = swb_offsets[s_idx] as usize;
        let end = swb_offsets[e_idx] as usize;
        if end <= start {
            continue;
        }

        // Dequantise coef_raw -> parcor -> lpc.
        let order = filt.order as usize;
        let mut parcor = [0.0f32; TNS_MAX_ORDER_LONG as usize];
        for o in 0..order {
            parcor[o] = dequant_tns_coef(filt.coef_raw[o], win.coef_res, filt.coef_compress);
        }
        let mut lpc = [0.0f32; TNS_MAX_ORDER_LONG as usize];
        parcor_to_lpc(&parcor[..order], &mut lpc[..order]);

        apply_tns_filter_range(spec, start, end - start, &lpc[..order], filt.direction);
    }
}

/// Apply TNS filtering to an eight-short-window spectrum.
///
/// Layout: `decode_spectrum_short` writes samples de-interleaved back to
/// per-sub-window order so the output array has 8 contiguous 128-sample
/// sub-windows at offsets `0, 128, 256, ..., 896`. TNS is specified
/// per sub-window (not per group) with `num_windows == 8`, and each
/// sub-window's filters run on its own 128-sample chunk using
/// `SWB_SHORT` band offsets.
///
/// Filters are ordered top-down within a sub-window — same semantics as
/// the long-block case but bounded by `TNS_MAX_BANDS_SHORT = 14`.
pub fn apply_tns_short(
    spec: &mut [f32; SPEC_LEN],
    tns: &TnsData,
    sf_index: u8,
    max_sfb: u8,
    info: &IcsInfo,
) {
    let swb = SWB_SHORT[sf_index as usize];
    let cap = tns_max_bands(WindowSequence::EightShort, sf_index).min(max_sfb) as usize;
    // Spec §4.6.9.3: initial `bottom = num_swb`, NOT the TNS_MAX_BANDS cap.
    let num_swb = swb.len().saturating_sub(1);
    let starts = group_starts(info);
    for g in 0..info.num_window_groups as usize {
        let group_len = info.window_group_length[g] as usize;
        let win_start_offset = starts[g] * 128;
        for w in 0..group_len {
            let subwin = starts[g] + w;
            if (subwin as u8) >= tns.num_windows {
                break;
            }
            let tns_win = &tns.windows[subwin];
            if tns_win.n_filt == 0 {
                continue;
            }
            let sub_base = win_start_offset + w * 128;
            let mut bottom = num_swb;
            for f in 0..tns_win.n_filt as usize {
                let filt = &tns_win.filters[f];
                let top = bottom;
                let length = filt.length as usize;
                bottom = top.saturating_sub(length);
                if filt.order == 0 || length == 0 {
                    continue;
                }
                let s_idx = bottom.min(cap);
                let e_idx = top.min(cap);
                if s_idx >= swb.len() || e_idx >= swb.len() {
                    break;
                }
                let start = sub_base + swb[s_idx] as usize;
                let end = sub_base + swb[e_idx] as usize;
                if end <= start {
                    continue;
                }
                let order = filt.order as usize;
                let mut parcor = [0.0f32; TNS_MAX_ORDER_LONG as usize];
                for o in 0..order {
                    parcor[o] =
                        dequant_tns_coef(filt.coef_raw[o], tns_win.coef_res, filt.coef_compress);
                }
                let mut lpc = [0.0f32; TNS_MAX_ORDER_LONG as usize];
                parcor_to_lpc(&parcor[..order], &mut lpc[..order]);
                apply_tns_filter_range(spec, start, end - start, &lpc[..order], filt.direction);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    #[test]
    fn dequant_tns_coef_sign() {
        // 4-bit, compress=0, code=0 => 0.
        let v = dequant_tns_coef(0, 1, 0);
        assert!(v.abs() < 1e-6);
        // 4-bit, compress=0, code=1 (positive 1) => sin(1 / iqfac) > 0
        // where iqfac = (8 - 0.5) / (π/2). With 0 < arg < π/2 the result
        // is strictly in (0, 1).
        let v = dequant_tns_coef(1, 1, 0);
        assert!(v > 0.0 && v < 1.0);
        // 4-bit, compress=0, code=0b1111 (= -1) => sin(-1 / iqfac_m) < 0.
        let v = dequant_tns_coef(0b1111, 1, 0);
        assert!(v < 0.0 && v > -1.0);
    }

    /// Spec-exact (§4.6.9.3) asymmetric `iqfac/iqfac_m` formula. Pin the
    /// exact value for a few representative codes so a future regression
    /// is caught immediately.
    ///
    /// Bug history: an earlier impl used `sin(iq * π / N)` with `N=9` for
    /// `coef_res=0` and `N=17` for `coef_res=1` — i.e. always the
    /// negative-branch denominator. Spec requires asymmetric divisors:
    /// for `coef_res=1` (B=4), `iqfac = (8 − 0.5) / (π/2) ≈ 4.7746` for
    /// `tmp ≥ 0` and `iqfac_m = (8 + 0.5) / (π/2) ≈ 5.4113` for `tmp < 0`.
    #[test]
    fn dequant_tns_coef_spec_exact_values() {
        let half_pi = std::f32::consts::FRAC_PI_2;
        // coef_res=1, coef_compress=0
        let iqfac_4 = (8.0 - 0.5) / half_pi;
        let iqfac_m_4 = (8.0 + 0.5) / half_pi;

        // code=4 => positive tmp=4. parcor = sin(4 / iqfac).
        let want = (4.0_f32 / iqfac_4).sin();
        let got = dequant_tns_coef(4, 1, 0);
        assert!(
            (got - want).abs() < 1e-6,
            "code=4: want {} got {}",
            want,
            got
        );

        // code=0b1111 (sign-extended -1) => negative tmp=-1. parcor =
        // sin(-1 / iqfac_m).
        let want = (-1.0_f32 / iqfac_m_4).sin();
        let got = dequant_tns_coef(0b1111, 1, 0);
        assert!(
            (got - want).abs() < 1e-6,
            "code=-1: want {} got {}",
            want,
            got
        );

        // coef_compress=1 with coef_res=1: transmitted width is 3 bits but
        // iqfac still uses the uncompressed coef_res_bits=4. So the
        // divisor is unchanged, and the inflated-magnitude approach
        // (left-shifting tmp) would have given a 2× too-large value.
        //
        // code=0b011 (sign-extended +3) => sin(3 / iqfac_4).
        let want = (3.0_f32 / iqfac_4).sin();
        let got = dequant_tns_coef(0b011, 1, 1);
        assert!(
            (got - want).abs() < 1e-6,
            "compress=1 code=3: want {} got {}",
            want,
            got
        );

        // Buggy "left-shift then divide by 17" form would produce
        // sin(6 * π / 17) ≈ 0.9009. Confirm we no longer match that.
        let buggy = (6.0_f32 * std::f32::consts::PI / 17.0).sin();
        assert!(
            (got - buggy).abs() > 0.05,
            "still computing buggy sin(2*tmp*π/17)?"
        );
    }

    #[test]
    fn parcor_to_lpc_order1() {
        // For order 1, lpc[0] == parcor[0].
        let parcor = [0.5f32];
        let mut lpc = [0.0f32; 1];
        parcor_to_lpc(&parcor, &mut lpc);
        assert!((lpc[0] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn parse_tns_data_long_filter() {
        // Build a minimal long-window TNS payload:
        //   n_filt=1 (2 bits), coef_res=1 (1 bit),
        //   length=3 (6 bits), order=2 (5 bits), direction=1 (1 bit),
        //   coef_compress=0 (1 bit), coef0=0b0001, coef1=0b0010 (4 bits each).
        let mut w = BitWriter::new();
        w.write_u32(1, 2);
        w.write_u32(1, 1);
        w.write_u32(3, 6);
        w.write_u32(2, 5);
        w.write_u32(1, 1);
        w.write_u32(0, 1);
        w.write_u32(0b0001, 4);
        w.write_u32(0b0010, 4);
        let bytes = w.finish();
        let mut br = BitReader::new(&bytes);
        let tns = parse_tns_data(&mut br, WindowSequence::OnlyLong, 1).unwrap();
        assert_eq!(tns.windows[0].n_filt, 1);
        assert_eq!(tns.windows[0].coef_res, 1);
        assert_eq!(tns.windows[0].filters[0].length, 3);
        assert_eq!(tns.windows[0].filters[0].order, 2);
        assert_eq!(tns.windows[0].filters[0].direction, 1);
        assert_eq!(tns.windows[0].filters[0].coef_compress, 0);
        assert_eq!(tns.windows[0].filters[0].coef_raw[0], 0b0001);
        assert_eq!(tns.windows[0].filters[0].coef_raw[1], 0b0010);
    }

    #[test]
    fn tns_filter_order1_forward_matches_allpole() {
        // Order 1 all-pole: y[n] = x[n] - a * y[n-1].
        // Set up a constant input; after filter, DC response = 1/(1+a).
        // Spec §4.6.9.3: direction=0 → upward (low → high), inc=+1.
        let mut spec = [0.0f32; SPEC_LEN];
        for s in spec.iter_mut().take(32).skip(10) {
            *s = 1.0;
        }
        let a = 0.3f32;
        let lpc = [a];
        apply_tns_filter_range(&mut spec, 10, 22, &lpc, 0);
        // First output: x[0] = 1.0 (no state yet).
        assert!((spec[10] - 1.0).abs() < 1e-6);
        // Second: 1.0 - 0.3 * 1.0 = 0.7
        assert!((spec[11] - 0.7).abs() < 1e-6);
        // Third: 1.0 - 0.3 * 0.7 = 0.79
        assert!((spec[12] - 0.79).abs() < 1e-4);
    }

    #[test]
    fn apply_tns_long_does_not_panic_empty() {
        let mut spec = [0.0f32; SPEC_LEN];
        let tns = TnsData::default();
        // swb_offsets for 44.1 kHz long.
        let swb: Vec<u16> = (0..=49).map(|i| (i * 20) as u16).collect();
        apply_tns_long(&mut spec, &tns, 4, 49, &swb);
    }

    #[test]
    fn apply_tns_short_filters_only_target_subwindow() {
        // Build an 8-short ICS with the default "8 groups of 1 window each"
        // grouping. Put a constant signal in sub-windows 0 and 3; attach a
        // TNS filter only to sub-window 0; verify sub-window 0 is IIR-
        // filtered and sub-window 3 is untouched.
        let mut info = IcsInfo {
            sf_index: 4,
            window_sequence: WindowSequence::EightShort,
            max_sfb: 14,
            num_window_groups: 8,
            ..IcsInfo::default()
        };
        for g in 0..8 {
            info.window_group_length[g] = 1;
        }
        let mut spec = [0.0f32; SPEC_LEN];
        for k in 0..128 {
            spec[k] = 1.0; // sub-window 0
            spec[3 * 128 + k] = 1.0; // sub-window 3
        }

        // Build TNS data: one filter on sub-window 0, order=1, direction=0
        // (upward = low→high, per spec §4.6.9.3), coef_res=1, coef_compress=0,
        // coef_raw[0] encodes ≈0.5 after dequant. With the spec's asymmetric
        // formula and `coef_res=1`, raw=3 → sin(3 / iqfac) where
        // iqfac = (8-0.5)/(π/2) = 4.7746 → sin(0.6283) ≈ 0.587.
        let mut tns = TnsData {
            num_windows: 8,
            ..TnsData::default()
        };
        tns.windows[0].n_filt = 1;
        tns.windows[0].coef_res = 1;
        tns.windows[0].filters[0] = TnsFilter {
            length: 14,
            order: 1,
            direction: 0,
            coef_compress: 0,
            coef_raw: {
                let mut c = [0u8; TNS_MAX_ORDER_LONG as usize];
                c[0] = 3;
                c
            },
        };

        apply_tns_short(&mut spec, &tns, info.sf_index, info.max_sfb, &info);

        // Sub-window 3 must still be the constant input.
        for k in 0..128 {
            assert_eq!(spec[3 * 128 + k], 1.0, "sub-window 3 was modified at k={k}");
        }
        // Sub-window 0 must show the first-order IIR response: first sample
        // stays 1.0, subsequent samples decay toward 1/(1+a).
        // The filter runs over bands [0..14) which for 44.1 kHz short covers
        // the first chunk of sub-window 0.
        let swb = SWB_SHORT[info.sf_index as usize];
        let filter_end = swb[14] as usize;
        assert!((spec[0] - 1.0).abs() < 1e-6);
        // Past the filter range, samples remain untouched.
        if filter_end < 128 {
            assert_eq!(spec[filter_end], 1.0);
        }
        // Within the filter range, sample 1 should have decayed.
        if filter_end > 1 {
            assert!(spec[1] < 1.0, "sub-window 0 sample 1 was not filtered");
        }
    }

    /// Pin the spec direction convention (§4.6.9.3): direction=0 is
    /// upward, direction=1 is downward.
    ///
    /// Bug history: the decoder ran direction=0 top-down and direction=1
    /// bottom-up — exactly the opposite of the spec. With real-content
    /// streams that emit direction=0 9th-order TNS filters (e.g.
    /// solana-ad.mp4 frame 3852) the swapped direction caused the IIR
    /// filter to amplify the bottom of the spectrum instead of running
    /// in the spec-prescribed direction, producing PCM peaks that
    /// saturated to ±32k.
    #[test]
    fn tns_direction_zero_runs_low_to_high() {
        // First-order all-pole; impulse at the start of the range. With
        // direction=0 (upward), only samples to the *right* of the impulse
        // pick up state from it. Samples to the left are untouched.
        let mut spec = [0.0f32; SPEC_LEN];
        spec[100] = 1.0;
        let lpc = [0.5f32];
        apply_tns_filter_range(&mut spec, 100, 10, &lpc, 0);
        // First output: x[0] (no state yet) = 1.0.
        assert!((spec[100] - 1.0).abs() < 1e-6);
        // Subsequent: y[1] = 0 - 0.5 * y[0] = -0.5; y[2] = 0 - 0.5*-0.5 = 0.25
        assert!((spec[101] - (-0.5)).abs() < 1e-6, "got {}", spec[101]);
        assert!((spec[102] - 0.25).abs() < 1e-6, "got {}", spec[102]);
    }

    #[test]
    fn tns_direction_one_runs_high_to_low() {
        // First-order all-pole, direction=1 (downward): processing starts
        // at end-1 and walks back toward start.
        let mut spec = [0.0f32; SPEC_LEN];
        spec[109] = 1.0; // end-1 of [100..110)
        let lpc = [0.5f32];
        apply_tns_filter_range(&mut spec, 100, 10, &lpc, 1);
        // First processed sample (idx=109): y = 1.0.
        assert!((spec[109] - 1.0).abs() < 1e-6);
        // Next (idx=108): y = 0 - 0.5*1.0 = -0.5
        assert!((spec[108] - (-0.5)).abs() < 1e-6, "got {}", spec[108]);
        assert!((spec[107] - 0.25).abs() < 1e-6, "got {}", spec[107]);
    }

    /// Pin the spec band-extent semantics (§4.6.9.3): the initial value
    /// of `bottom` is `num_swb`, NOT `min(TNS_MAX_BANDS, max_sfb)`. The
    /// `TNS_MAX_BANDS` cap is applied to start/end indices AFTER the
    /// length subtraction, not to `bottom` itself.
    ///
    /// Bug history: an earlier version pre-clamped `top` to
    /// `min(TNS_MAX_BANDS, max_sfb)`, so a length-25 filter on 44.1 kHz
    /// long (TNS_MAX_BANDS=42, num_swb=49) covered SFBs 17..42 instead
    /// of the spec's 24..42 — i.e. 7 extra low-frequency SFBs that
    /// hold the bulk of the spectrum's energy. The wrong filter
    /// region multiplied the IMDCT input on real-content streams,
    /// producing the audible spike artefacts on solana-ad.mp4.
    #[test]
    fn tns_long_band_extent_matches_spec() {
        // 44.1 kHz long window. num_swb=49, TNS_MAX_BANDS_LONG[4]=42.
        // Filter length=25 should cover SFBs 24..42 per spec
        // (top=49→42, bottom=49-25=24). Pre-fix code computed bottom=42-25=17
        // and ran the filter over SFBs 17..42 — testing the SFB boundary
        // pins the corrected behaviour.
        let info = IcsInfo {
            sf_index: 4,
            window_sequence: WindowSequence::OnlyLong,
            max_sfb: 49,
            num_window_groups: 1,
            window_group_length: [1, 0, 0, 0, 0, 0, 0, 0],
            ..IcsInfo::default()
        };
        let swb = crate::sfband::SWB_LONG[info.sf_index as usize];
        // Build a spectrum that's all 1.0 within SFBs 17..49.
        let mut spec = [0.0f32; SPEC_LEN];
        for k in (swb[17] as usize)..(swb[49] as usize) {
            spec[k] = 1.0;
        }
        // Snapshot the lower portion (SFBs 17..24) — these MUST remain
        // unchanged with the spec-correct band-extent semantics.
        let snapshot_start = swb[17] as usize;
        let snapshot_end = swb[24] as usize;
        let pre: Vec<f32> = spec[snapshot_start..snapshot_end].to_vec();

        // Construct a TNS data with a length=25 long-window filter that
        // would (with the buggy code) reach down into SFBs 17..24.
        let mut tns = TnsData {
            num_windows: 1,
            ..TnsData::default()
        };
        tns.windows[0].n_filt = 1;
        tns.windows[0].coef_res = 1;
        tns.windows[0].filters[0] = TnsFilter {
            length: 25,
            order: 4,
            direction: 0,
            coef_compress: 0,
            coef_raw: {
                let mut c = [0u8; TNS_MAX_ORDER_LONG as usize];
                c[0] = 6; // moderately strong parcor
                c[1] = 4;
                c[2] = 2;
                c[3] = 1;
                c
            },
        };

        apply_tns_long(&mut spec, &tns, info.sf_index, info.max_sfb, swb);

        // SFBs 17..24 must still be all-1.0 — TNS only operates on 24..42.
        for (i, &want) in pre.iter().enumerate() {
            let got = spec[snapshot_start + i];
            assert!(
                (got - want).abs() < 1e-6,
                "SFB <24 mutated: idx={} got={} want={}",
                snapshot_start + i,
                got,
                want
            );
        }
        // SFBs 24..42 must have been touched (some sample changed).
        let mut any_changed = false;
        for k in (swb[24] as usize)..(swb[42] as usize) {
            if (spec[k] - 1.0).abs() > 1e-6 {
                any_changed = true;
                break;
            }
        }
        assert!(any_changed, "TNS did not modify any SFB in 24..42");
    }
}

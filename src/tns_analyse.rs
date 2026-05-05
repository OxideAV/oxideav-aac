//! Encoder-side TNS (Temporal Noise Shaping) analysis — ISO/IEC 14496-3 §4.6.9.
//!
//! On the encoder side TNS is an *analysis* step: we fit an LPC predictor to
//! the MDCT coefficient vector, apply the forward (analysis / all-zero) filter
//! in place to flatten the coefficient envelope, and write the filter's
//! reflection coefficients into the bitstream. The decoder then runs the
//! complementary all-pole synthesis filter to restore the envelope, which
//! reshapes the quantisation noise to follow the signal envelope in time —
//! very effective on percussive / transient material.
//!
//! Workflow:
//!   1. `levinson` — autocorrelation-based Levinson-Durbin yields standard
//!      LPC coefficients `a[1..=order]` and reflection coefficients
//!      `k[1..=order]`, along with a prediction error ratio.
//!   2. Prediction gain `g = 1 / error_ratio`; we gate on `g >= GAIN_THRESHOLD`.
//!      `1.4 dB ≈ 10^(1.4/10) ≈ 1.38` — below that TNS isn't worth the bits.
//!   3. `a_std -> decoder_lpc`: the decoder's filter is parameterised as
//!      `y[n] = x[n] - sum_k lpc[k] * y[n-1-k]` (an all-pole IIR). To match
//!      `H(z) = 1/A(z)` with `A(z) = 1 - sum_k a_std[k] z^{-k}` we need
//!      `decoder_lpc[i] = -a_std[i+1]`.
//!   4. `lpc_to_parcor` — step-down recursion on `decoder_lpc` produces the
//!      parcor values to quantise into the bitstream.
//!   5. Quantise each parcor to 4 bits (`coef_res = 1`, `coef_compress = 0`
//!      — spec: the signed raw value `iq` satisfies `parcor = sin(iq * π / 17)`,
//!      so `iq = round(asin(parcor) * 17 / π)` clipped to `[-8, 7]`).
//!   6. Re-dequantise to get the *decoder's view* of the parcor, rebuild the
//!      matching `decoder_lpc`, and apply the forward filter
//!      `e[n] = x[n] + sum_k decoder_lpc[k] * x[n-1-k]` to the spectrum.
//!      (Why decoder_lpc and not a_std: after requantisation the values drift,
//!      so we re-derive consistency.)

use crate::ics::SPEC_LEN;
use crate::sfband::{SWB_LONG, SWB_SHORT};
use crate::tns::{TNS_MAX_BANDS_SHORT, TNS_MAX_ORDER_LONG, TNS_MAX_ORDER_SHORT};

/// Maximum LPC order used by the encoder's TNS (long windows). The actual
/// order is chosen adaptively per-frame by [`select_tns_order`] — louder,
/// more tonal frames benefit from higher orders (up to this cap) while
/// noise-like or low-energy frames settle at lower orders to avoid spending
/// bits on uninformative coefficients.
pub const TNS_ENC_ORDER_MAX: usize = 8;
/// Minimum LPC order for the long-window TNS path.
pub const TNS_ENC_ORDER_MIN: usize = 2;
/// Legacy constant kept for callers that haven't been updated; points at
/// the cap so existing tests are unaffected.
pub const TNS_ENC_ORDER: usize = TNS_ENC_ORDER_MAX;

/// LPC order used by the encoder's TNS on 128-coefficient short
/// sub-windows. Kept below [`TNS_MAX_ORDER_SHORT`] (= 7) — order 4 hits
/// the same cost/benefit sweet spot as the long-window path and uses
/// the same 4-bit (coef_res=1) parcor quantisation.
pub const TNS_ENC_ORDER_SHORT: usize = 4;

/// Minimum prediction gain (= 1 / levinson_error_ratio) required to apply TNS.
/// `1.4 dB ≈ 10^(1.4/10) ≈ 1.3804` — below that the bits spent signalling TNS
/// outweigh the quantisation savings.
///
/// For signals whose spectral flatness is high (noise-like), this threshold is
/// raised by [`adaptive_tns_threshold`] so TNS does not fire on bands where the
/// LPC filter flattens the noise envelope but saves essentially no bits (the
/// noise quantises well without flattening). For highly tonal signals the
/// threshold is used as-is so even a modest prediction gain enables TNS.
pub const TNS_GAIN_THRESHOLD: f32 = 1.3804;

/// Select the LPC filter order for a long-block TNS analysis pass.
///
/// Strategy (clean-room, calibrated against ISO/IEC 14496-3 Annex B):
/// - Compute the spectral flatness measure (SFM) of the band: `geomean / arithmean`
///   of squared magnitudes. SFM ≈ 1 → noise-like (low order helps less).
///   SFM ≈ 0 → tonal (higher order extracts more redundancy).
/// - High-amplitude, tonal bands: use up to `TNS_ENC_ORDER_MAX`.
/// - Low-amplitude or noisy bands: floor at `TNS_ENC_ORDER_MIN`.
/// - Cap at `min(band_len - 1, TNS_ENC_ORDER_MAX, TNS_MAX_ORDER_LONG)`.
pub fn select_tns_order(band: &[f32], max_allowed: usize) -> usize {
    if band.len() < 3 {
        return 1;
    }
    // Spectral flatness of the input band (linear power ratios).
    let mut sum_sq = 0.0f64;
    let mut sum_log = 0.0f64;
    const EPS: f64 = 1e-20;
    for &x in band {
        let p = (x as f64 * x as f64 + EPS).max(EPS);
        sum_sq += p;
        sum_log += p.ln();
    }
    let n = band.len() as f64;
    let mean = sum_sq / n;
    let geomean = (sum_log / n).exp();
    // SFM ∈ (0, 1]. 0 = pure tone (use max order), 1 = white noise (use min).
    let sfm = (geomean / mean).clamp(1e-9, 1.0) as f32;
    // Map SFM → order: linear interpolation between min and max.
    //   sfm ≈ 0 (tonal)  → max order
    //   sfm ≈ 1 (noise)  → min order
    // With a tonality-bias: prefer higher orders unless clearly noise-like.
    let order_f =
        TNS_ENC_ORDER_MAX as f32 - (TNS_ENC_ORDER_MAX - TNS_ENC_ORDER_MIN) as f32 * sfm.powf(0.5);
    let order = (order_f.round() as usize)
        .clamp(TNS_ENC_ORDER_MIN, TNS_ENC_ORDER_MAX)
        .min(max_allowed)
        .min(TNS_MAX_ORDER_LONG as usize);
    order.max(1)
}

/// Adaptive TNS gain threshold that rises for noise-like inputs.
///
/// On noise-like bands (high SFM) the LPC filter makes only marginal
/// prediction gains — the autocorrelation is small and the filter order
/// rarely helps. We raise the threshold to `~1.8` for pure noise so TNS
/// only fires when the prediction gain is meaningful. For tonal content
/// (low SFM) we keep the standard `TNS_GAIN_THRESHOLD`.
pub fn adaptive_tns_threshold(band: &[f32]) -> f32 {
    if band.len() < 3 {
        return TNS_GAIN_THRESHOLD;
    }
    let mut sum_sq = 0.0f64;
    let mut sum_log = 0.0f64;
    const EPS: f64 = 1e-20;
    for &x in band {
        let p = (x as f64 * x as f64 + EPS).max(EPS);
        sum_sq += p;
        sum_log += p.ln();
    }
    let n = band.len() as f64;
    let mean = sum_sq / n;
    let geomean = (sum_log / n).exp();
    let sfm = (geomean / mean).clamp(1e-9, 1.0) as f32;
    // Linearly blend from TNS_GAIN_THRESHOLD (tonal) to 1.8 (noise) as SFM
    // goes from 0 to 1. The 1.8 cap means noise bands with prediction gain
    // ≤ 2.6 dB don't fire TNS.
    TNS_GAIN_THRESHOLD + (1.8 - TNS_GAIN_THRESHOLD) * sfm.min(1.0)
}

/// Coefficient resolution written into the TNS filter (4-bit parcor codes).
pub const TNS_ENC_COEF_RES: u8 = 1;
/// Coefficient-compress flag (0 => full 4-bit range).
pub const TNS_ENC_COEF_COMPRESS: u8 = 0;
/// Filter direction (0 = high-to-low, i.e. the filter runs from the top of
/// the band downward). For the encoder we pick the simple default.
pub const TNS_ENC_DIRECTION: u8 = 0;

/// Parameters emitted by a successful TNS analysis — everything the encoder
/// needs to serialise the `tns_data()` payload.
#[derive(Clone, Debug)]
pub struct TnsEncFilter {
    /// Length in SFBs covered by the filter.
    pub length_sfb: u8,
    /// Filter order (0 => no filter).
    pub order: u8,
    /// Direction bit (0 high→low, 1 low→high). We always emit 0.
    pub direction: u8,
    /// Coefficient-compress (always 0 for this encoder).
    pub coef_compress: u8,
    /// Raw parcor codes, one per order. Valid range per `coef_res`:
    ///   res=0 (3-bit): [-4..=3], stored 2's-complement in 3 bits
    ///   res=1 (4-bit): [-8..=7], stored 2's-complement in 4 bits
    pub coef_raw: [u8; TNS_MAX_ORDER_LONG as usize],
}

/// Autocorrelation-based Levinson-Durbin LPC analysis.
///
/// Returns `(lpc_a[1..=order], reflection_k[1..=order], error_ratio)` where
/// `error_ratio = E_order / E_0` (values in (0, 1] for well-conditioned input;
/// closer to 0 means better prediction). Length of both returned slices is
/// `order`.
pub fn levinson(samples: &[f32], order: usize) -> (Vec<f32>, Vec<f32>, f32) {
    assert!(order >= 1);
    assert!(samples.len() > order);
    // Autocorrelation r[0..=order].
    let mut r = vec![0.0f64; order + 1];
    for lag in 0..=order {
        let mut acc = 0.0f64;
        for n in lag..samples.len() {
            acc += samples[n] as f64 * samples[n - lag] as f64;
        }
        r[lag] = acc;
    }
    if r[0] <= 0.0 {
        return (vec![0.0; order], vec![0.0; order], 1.0);
    }
    // Classical Levinson-Durbin recursion.
    let mut a = vec![0.0f64; order + 1]; // a[0] = 1 conceptually; we store 1..=order
    let mut k = vec![0.0f64; order + 1];
    let mut e = r[0];
    for m in 1..=order {
        // Compute reflection coefficient.
        let mut num = r[m];
        for i in 1..m {
            num += a[i] * r[m - i];
        }
        let km = if e.abs() > 1e-20 { -num / e } else { 0.0 };
        // Update prediction coefficients.
        let mut new_a = a.clone();
        new_a[m] = km;
        for i in 1..m {
            new_a[i] = a[i] + km * a[m - i];
        }
        a = new_a;
        k[m] = km;
        e *= 1.0 - km * km;
        if e <= 0.0 {
            // Numerical instability — bail out with what we have.
            e = 0.0;
            break;
        }
    }
    let ratio = (e / r[0]).max(0.0) as f32;
    // Convention mismatch: classical Levinson here models the signal as
    // `x[n] = -sum_k a[k] * x[n-k] + e[n]`, so the *standard* LPC coefficients
    // used by most textbooks are `a_std[k] = -a[k]` (predictor form
    // `x_pred[n] = sum_k a_std[k] * x[n-k]`). Convert before returning.
    let lpc_std: Vec<f32> = (1..=order).map(|i| -a[i] as f32).collect();
    let refl: Vec<f32> = (1..=order).map(|i| k[i] as f32).collect();
    (lpc_std, refl, ratio)
}

/// Step-down recursion: given target LPC coefficients in the decoder's
/// convention (i.e. the values that `tns::parcor_to_lpc` should produce),
/// recover the parcor values to signal.
pub fn lpc_to_parcor(lpc: &[f32]) -> Vec<f32> {
    let order = lpc.len();
    let mut work: Vec<f32> = lpc.to_vec();
    let mut parcor = vec![0.0f32; order];
    for m in (0..order).rev() {
        let km = work[m];
        parcor[m] = km;
        if m == 0 {
            break;
        }
        let denom = 1.0 - km * km;
        if denom.abs() < 1e-12 {
            // Near-unstable pole; leave lower-order values zero.
            break;
        }
        let mut tmp = vec![0.0f32; m];
        for i in 0..m {
            tmp[i] = (work[i] - km * work[m - 1 - i]) / denom;
        }
        for i in 0..m {
            work[i] = tmp[i];
        }
    }
    parcor
}

/// Quantise one parcor (reflection) coefficient into the raw code stored in
/// the bitstream.
///
/// Spec (§4.6.9.3, `tns_decode_coef`): with `coef_res = R ∈ {0,1}` and
/// `coef_compress = 0`, the raw code `iq` is a `(R+3)`-bit signed integer.
/// The dequantised parcor is asymmetric:
/// ```text
///   iqfac   = ((1<<(R+2)) - 0.5) / (π/2)     // used when iq ≥ 0
///   iqfac_m = ((1<<(R+2)) + 0.5) / (π/2)     // used when iq <  0
///   parcor  = sin( iq / (iq>=0 ? iqfac : iqfac_m) )
/// ```
/// We invert that here. The asymmetric divisor means the inverse must use
/// `iqfac` for positive `p` and `iqfac_m` for negative `p`.
pub fn quantise_parcor(p: f32, coef_res: u8) -> u8 {
    let (low, high) = if coef_res == 0 {
        (-4i32, 3i32)
    } else {
        (-8i32, 7i32)
    };
    let two_pow = (1u32 << (coef_res as u32 + 2)) as f32; // 4 (R=0) or 8 (R=1)
    let half_pi = std::f32::consts::FRAC_PI_2;
    let iqfac = (two_pow - 0.5) / half_pi;
    let iqfac_m = (two_pow + 0.5) / half_pi;
    let p_clamped = p.clamp(-0.999, 0.999);
    let asin_p = p_clamped.asin();
    let iq_f = if asin_p >= 0.0 {
        asin_p * iqfac
    } else {
        asin_p * iqfac_m
    };
    let iq = iq_f.round() as i32;
    let iq_clamped = iq.clamp(low, high);
    // 2's-complement within `coef_bits = 3 + coef_res` bits.
    let coef_bits = 3 + coef_res as u32;
    let mask = (1u32 << coef_bits) - 1;
    (iq_clamped as u32 & mask) as u8
}

/// Dequantise a raw parcor code back to a float — used by the encoder to
/// re-align the forward filter with the decoder's view of the quantised
/// filter. Mirror of [`crate::tns`]'s `dequant_tns_coef` for `coef_compress = 0`.
pub fn dequantise_parcor(code: u8, coef_res: u8) -> f32 {
    let coef_bits = (3 + coef_res) as u32;
    let sign_bit = 1u8 << (coef_bits - 1);
    let iq = if code & sign_bit != 0 {
        (code as i32) - (1i32 << coef_bits)
    } else {
        code as i32
    };
    let two_pow = (1u32 << (coef_res as u32 + 2)) as f32; // 4 or 8
    let half_pi = std::f32::consts::FRAC_PI_2;
    let iqfac = (two_pow - 0.5) / half_pi;
    let iqfac_m = (two_pow + 0.5) / half_pi;
    let denom = if iq >= 0 { iqfac } else { iqfac_m };
    (iq as f32 / denom).sin()
}

/// Given parcor values, produce the LPC coefficients in the decoder's
/// convention (matches `tns::parcor_to_lpc`).
pub fn parcor_to_decoder_lpc(parcor: &[f32]) -> Vec<f32> {
    let order = parcor.len();
    let mut work = vec![0.0f32; order];
    for m in 0..order {
        let mut tmp = vec![0.0f32; m];
        for i in 0..m {
            tmp[i] = work[i] + parcor[m] * work[m - 1 - i];
        }
        for i in 0..m {
            work[i] = tmp[i];
        }
        work[m] = parcor[m];
    }
    work
}

/// Apply the forward TNS analysis filter (all-zero) in place over
/// `spec[start..end]`. Mirror-image of the decoder's all-pole synthesis
/// filter in `tns::apply_tns_filter_range`:
///
///   decoder (all-pole):    y[n] = x[n] - sum_k lpc[k] * y[n-1-k]
///   encoder (all-zero):    y[n] = x[n] + sum_k lpc[k] * x[n-1-k]
///
/// So feeding the encoder's output through the decoder yields the original
/// spectrum (modulo quantisation of the parcors).
///
/// `direction` follows the spec convention (§4.6.9.3): 0 = upward (low →
/// high), 1 = downward (high → low). MUST match the decoder convention in
/// [`crate::tns::apply_tns_filter_range`].
pub fn apply_forward_tns(
    spec: &mut [f32; SPEC_LEN],
    start: usize,
    length: usize,
    decoder_lpc: &[f32],
    direction: u8,
) {
    let order = decoder_lpc.len();
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
        // Downward: process top-down.
        for i in 0..n {
            let idx = end - 1 - i;
            let x = spec[idx];
            let mut y = x;
            for k in 0..order.min(i) {
                y += decoder_lpc[k] * state[k];
            }
            // state = history of input x (FIR on past inputs).
            for k in (1..order).rev() {
                state[k] = state[k - 1];
            }
            state[0] = x;
            spec[idx] = y;
        }
    } else {
        // Upward (direction == 0): low → high.
        for i in 0..n {
            let idx = start + i;
            let x = spec[idx];
            let mut y = x;
            for k in 0..order.min(i) {
                y += decoder_lpc[k] * state[k];
            }
            for k in (1..order).rev() {
                state[k] = state[k - 1];
            }
            state[0] = x;
            spec[idx] = y;
        }
    }
}

/// High-level entry point used by the encoder. Runs analysis on the current
/// MDCT coefficients for a long window; if TNS is worth applying, modifies
/// `spec` in place (applies forward filter) and returns filter parameters
/// ready for the bitstream emitter. Returns `None` if TNS should not be
/// signalled.
///
/// The filter covers SFBs `[0, length_sfb)` starting at the *top* of the
/// TNS-allowed range (`min(max_sfb, tns_max_bands)`). The decoder unwinds
/// filters from top downward, matching this layout.
pub fn analyse_long(spec: &mut [f32; SPEC_LEN], sf_index: u8, max_sfb: u8) -> Option<TnsEncFilter> {
    let swb = SWB_LONG[sf_index as usize];
    if max_sfb == 0 {
        return None;
    }
    // Work out the TNS span: from sfb 0 up to `top` (exclusive).
    let tns_cap = crate::tns::tns_max_bands(crate::syntax::WindowSequence::OnlyLong, sf_index);
    let top = (max_sfb as usize).min(tns_cap as usize);
    if top < 2 {
        return None;
    }
    let start_bin = swb[0] as usize;
    let end_bin = swb[top] as usize;
    if end_bin <= start_bin {
        return None;
    }
    let n = end_bin - start_bin;
    if n < 2 {
        return None;
    }
    // Levinson on the spectral-coefficient sequence in-place order.
    let window_slice: &[f32] = &spec[start_bin..end_bin];
    // Adaptively select order based on spectral flatness of the band.
    let order = select_tns_order(window_slice, n - 1);
    if order == 0 {
        return None;
    }
    let (_lpc_std, refl, err_ratio) = levinson(window_slice, order);
    if err_ratio >= 1.0 {
        return None;
    }
    let gain = 1.0 / err_ratio.max(1e-9);
    // Use an adaptive threshold that is higher for noise-like inputs.
    let threshold = adaptive_tns_threshold(window_slice);
    if gain < threshold {
        return None;
    }
    // Convert reflection coefficients (standard Levinson output) into the
    // parcor values the decoder's `parcor_to_lpc` uses. The decoder's
    // convention lpc[i] = -a_std[i+1] means the parcor we emit is
    // `parcor = -refl` — but the recursion is not linear in sign so we
    // do a step-down on the negated lpc vector for correctness.
    let lpc_std: Vec<f32> = _lpc_std;
    let decoder_lpc_target: Vec<f32> = lpc_std.iter().map(|&v| -v).collect();
    let parcor_target = lpc_to_parcor(&decoder_lpc_target);
    // Quantise + dequantise each parcor, then rebuild the decoder_lpc from
    // the quantised view so the forward filter we apply is the exact inverse
    // of what the decoder will do.
    let mut coef_raw = [0u8; TNS_MAX_ORDER_LONG as usize];
    let mut parcor_q = vec![0.0f32; order];
    let _ = refl;
    for (i, &p) in parcor_target.iter().enumerate() {
        let code = quantise_parcor(p, TNS_ENC_COEF_RES);
        coef_raw[i] = code;
        parcor_q[i] = dequantise_parcor(code, TNS_ENC_COEF_RES);
    }
    let decoder_lpc_q = parcor_to_decoder_lpc(&parcor_q);
    // Apply the forward filter using the quantised decoder_lpc.
    apply_forward_tns(
        spec,
        start_bin,
        end_bin - start_bin,
        &decoder_lpc_q,
        TNS_ENC_DIRECTION,
    );
    Some(TnsEncFilter {
        length_sfb: top as u8,
        order: order as u8,
        direction: TNS_ENC_DIRECTION,
        coef_compress: TNS_ENC_COEF_COMPRESS,
        coef_raw,
    })
}

/// Short-window TNS analysis. Runs on a single 128-coefficient
/// sub-window at offset `sub_window * 128` inside a 1024-coefficient
/// short-block spectrum. Mirror of [`analyse_long`] with three
/// short-specific caps:
///   * `length_sfb` is capped at `min(max_sfb, TNS_MAX_BANDS_SHORT)`
///     (= 14 for AAC-LC, per §4.6.9.1 / Table 4.139).
///   * `order` is capped at `TNS_ENC_ORDER_SHORT` (4) and bounded by
///     [`TNS_MAX_ORDER_SHORT`] (= 7) per spec.
///   * Filter `length_sfb` field is emitted as 4 bits, so must fit in
///     `[0, 15]`.
///
/// On success, mutates `spec[sub_base..sub_base + swb[length_sfb]]` in
/// place with the forward (all-zero) filter and returns the filter
/// params to serialise. Returns `None` when TNS wouldn't help (gain
/// below [`TNS_GAIN_THRESHOLD`], degenerate autocorrelation, etc.) —
/// callers then emit `n_filt = 0` for the sub-window.
pub fn analyse_short(
    spec: &mut [f32; SPEC_LEN],
    sub_window: usize,
    sf_index: u8,
    max_sfb: u8,
) -> Option<TnsEncFilter> {
    debug_assert!(sub_window < 8);
    let swb = SWB_SHORT[sf_index as usize];
    if max_sfb == 0 {
        return None;
    }
    let tns_cap = (TNS_MAX_BANDS_SHORT as usize).min(max_sfb as usize);
    // length_sfb is written as 4 bits → must fit in [0, 15].
    let top = tns_cap.min(15);
    if top < 2 {
        return None;
    }
    let sub_base = sub_window * 128;
    let start_bin = sub_base + swb[0] as usize;
    let end_bin = sub_base + swb[top] as usize;
    if end_bin <= start_bin || end_bin > SPEC_LEN {
        return None;
    }
    let n = end_bin - start_bin;
    let order = TNS_ENC_ORDER_SHORT
        .min(TNS_MAX_ORDER_SHORT as usize)
        .min(n.saturating_sub(1));
    if order == 0 {
        return None;
    }
    let window_slice: &[f32] = &spec[start_bin..end_bin];
    let (lpc_std, _refl, err_ratio) = levinson(window_slice, order);
    if err_ratio >= 1.0 {
        return None;
    }
    let gain = 1.0 / err_ratio.max(1e-9);
    let threshold = adaptive_tns_threshold(window_slice);
    if gain < threshold {
        return None;
    }
    let decoder_lpc_target: Vec<f32> = lpc_std.iter().map(|&v| -v).collect();
    let parcor_target = lpc_to_parcor(&decoder_lpc_target);
    let mut coef_raw = [0u8; TNS_MAX_ORDER_LONG as usize];
    let mut parcor_q = vec![0.0f32; order];
    for (i, &p) in parcor_target.iter().enumerate() {
        let code = quantise_parcor(p, TNS_ENC_COEF_RES);
        coef_raw[i] = code;
        parcor_q[i] = dequantise_parcor(code, TNS_ENC_COEF_RES);
    }
    let decoder_lpc_q = parcor_to_decoder_lpc(&parcor_q);
    apply_forward_tns(
        spec,
        start_bin,
        end_bin - start_bin,
        &decoder_lpc_q,
        TNS_ENC_DIRECTION,
    );
    Some(TnsEncFilter {
        length_sfb: top as u8,
        order: order as u8,
        direction: TNS_ENC_DIRECTION,
        coef_compress: TNS_ENC_COEF_COMPRESS,
        coef_raw,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levinson_order1_sine() {
        // Highly-correlated signal: 1, 2, 1, 2, ... -> Levinson should find
        // a strong reflection coefficient.
        let x: Vec<f32> = (0..64)
            .map(|i| (i as f32 * std::f32::consts::PI / 8.0).sin())
            .collect();
        let (lpc, refl, err) = levinson(&x, 4);
        assert_eq!(lpc.len(), 4);
        assert_eq!(refl.len(), 4);
        // Prediction error ratio should be strictly < 1 on a pure tone.
        assert!(err < 0.5, "err={err}");
        // LPC of a pure sinusoid has strong second-order term.
        let nonzero = lpc.iter().any(|&v| v.abs() > 1e-3);
        assert!(nonzero);
    }

    #[test]
    fn quantise_parcor_roundtrip_4bit() {
        for &p in &[-0.9f32, -0.5, -0.1, 0.0, 0.1, 0.5, 0.9] {
            let code = quantise_parcor(p, 1);
            let p2 = dequantise_parcor(code, 1);
            // 4-bit parcor step size is sin(pi/17) ≈ 0.184 — roundtrip
            // should be within that.
            assert!((p - p2).abs() < 0.2, "p={p} p2={p2}");
        }
    }

    #[test]
    fn lpc_to_parcor_roundtrip() {
        // Start with a parcor vector, expand to LPC (decoder convention),
        // then step-down back to parcors — should round-trip exactly.
        let parcor = [0.4f32, -0.25, 0.1, -0.3];
        let lpc = parcor_to_decoder_lpc(&parcor);
        let p2 = lpc_to_parcor(&lpc);
        for i in 0..4 {
            assert!((parcor[i] - p2[i]).abs() < 1e-5, "mismatch at {i}");
        }
    }

    #[test]
    fn forward_then_decoder_reverses() {
        // Apply the encoder's forward filter and then the decoder's reverse
        // — output should match the input (quantisation aside).
        let mut spec = [0.0f32; SPEC_LEN];
        for (i, s) in spec.iter_mut().enumerate().take(100).skip(10) {
            *s = ((i as f32) * 0.3).sin();
        }
        let orig = spec;
        let parcor = [0.3f32, -0.2, 0.1, -0.15];
        let dlpc = parcor_to_decoder_lpc(&parcor);
        apply_forward_tns(&mut spec, 10, 90, &dlpc, 0);
        // Now run the decoder's all-pole IIR.
        crate::tns::_apply_tns_filter_range_for_test(&mut spec, 10, 90, &dlpc, 0);
        for i in 10..100 {
            assert!(
                (spec[i] - orig[i]).abs() < 1e-4,
                "mismatch at {i}: {} vs {}",
                spec[i],
                orig[i]
            );
        }
    }
}

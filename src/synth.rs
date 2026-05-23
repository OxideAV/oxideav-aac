//! Per-channel post-quantisation processing — windowing, IMDCT, overlap-add.
//!
//! ISO/IEC 14496-3 §4.6.11 and §4.6.18.
//!
//! Each channel keeps a 1024-sample overlap history (`prev`). Each frame:
//! 1. Run IMDCT (long: 1024→2048; short: 8 × 128→256).
//! 2. Apply the appropriate window across the full 2N IMDCT output.
//! 3. PCM = `prev + windowed_first_half`; new `prev` = `windowed_second_half`.

use crate::ics::LtpDataNonLd;
use crate::imdct::{imdct_long, imdct_short, LONG_INPUT, SHORT_INPUT};
use crate::syntax::{WindowSequence, WindowShape};
use crate::window::{
    build_long_window_full, kbd_short, long_window_for_shape, sine_short, LONG_LEN, SHORT_LEN,
};

pub const FRAME_LEN: usize = 1024;

/// Number of time-domain history samples retained per channel for non-LD
/// AAC-LTP — ISO/IEC 14496-3 §4.6.7.3. The predictor reads
/// `x_rec(i − M − ltp_lag)` for `i` in `0..N` (`N = 2·FRAME_LEN = 2048`,
/// `M = 0`) and `ltp_lag` up to 2047, so the deepest read into past output
/// is `i = −1 − 2047 = −2048`. Retaining 2048 samples covers every lag.
pub const LTP_HISTORY_LEN: usize = 2048;

/// Per-channel running state for IMDCT overlap.
#[derive(Clone, Debug)]
pub struct ChannelState {
    /// Right (already-windowed) half of last frame's IMDCT output, ready to overlap.
    pub prev: Vec<f32>,
    /// Window shape (W bit) of the previous block — determines the previous
    /// block's right-side window for asymmetric long_start/long_stop blocks.
    pub prev_shape: WindowShape,
    /// Window sequence of the previous block.
    pub prev_seq: WindowSequence,
    /// Non-LD AAC-LTP (§4.6.7) time-domain history — the previously
    /// reconstructed decoder output, newest sample last. `time_hist[len−1]`
    /// is the sample at `x_rec(−1)`. Sized to [`LTP_HISTORY_LEN`] so the
    /// deepest `x_rec(−1 − ltp_lag)` read (lag ≤ 2047) is covered.
    pub ltp_hist: Vec<f32>,
}

impl ChannelState {
    pub fn new() -> Self {
        Self {
            prev: vec![0.0; FRAME_LEN],
            prev_shape: WindowShape::Sine,
            prev_seq: WindowSequence::OnlyLong,
            ltp_hist: vec![0.0; LTP_HISTORY_LEN],
        }
    }

    /// Append `pcm` (this frame's reconstructed decoder output) to the
    /// non-LD AAC-LTP history, dropping the oldest samples so the buffer
    /// length stays at [`LTP_HISTORY_LEN`]. Newest sample ends up last.
    pub fn push_ltp_history(&mut self, pcm: &[f32]) {
        let n = pcm.len();
        if n >= LTP_HISTORY_LEN {
            self.ltp_hist.copy_from_slice(&pcm[n - LTP_HISTORY_LEN..]);
            return;
        }
        self.ltp_hist.copy_within(n.., 0);
        let tail = LTP_HISTORY_LEN - n;
        self.ltp_hist[tail..].copy_from_slice(pcm);
    }
}

impl Default for ChannelState {
    fn default() -> Self {
        Self::new()
    }
}

fn short_window(shape: WindowShape) -> &'static [f32] {
    match shape {
        WindowShape::Sine => sine_short(),
        WindowShape::Kbd => kbd_short(),
    }
}

/// Run IMDCT on `spec` (1024 spectral coefs) according to `seq`/`shape`,
/// applying windowing and overlap-add against `state.prev`. Writes 1024
/// PCM samples into `pcm` and updates `state` for the next frame.
pub fn imdct_and_overlap(
    spec: &[f32; 1024],
    seq: WindowSequence,
    shape: WindowShape,
    state: &mut ChannelState,
    pcm: &mut [f32; FRAME_LEN],
) {
    let n = LONG_LEN;
    if !matches!(seq, WindowSequence::EightShort) {
        let mut tmp = vec![0.0f32; 2 * LONG_INPUT];
        imdct_long(spec, &mut tmp);
        let win = build_long_window_full(seq, shape, state.prev_shape);
        // Apply window.
        for i in 0..(2 * n) {
            tmp[i] *= win[i];
        }
        // PCM = prev + tmp[0..N]
        for i in 0..n {
            pcm[i] = state.prev[i] + tmp[i];
        }
        // New overlap = tmp[N..2N]
        for i in 0..n {
            state.prev[i] = tmp[n + i];
        }
    } else {
        // Eight-short sequence — 8 sub-windows of 128 spectral coefs each,
        // each IMDCT'd to 256 samples then windowed and overlapped per ISO Fig. 4.24.
        let mut shorts = [[0.0f32; 2 * SHORT_INPUT]; 8];
        for w in 0..8 {
            let s = w * SHORT_INPUT;
            let chunk: &[f32] = &spec[s..s + SHORT_INPUT];
            imdct_short(chunk, &mut shorts[w]);
        }
        let cur_short = short_window(shape);
        let prev_short = short_window(state.prev_shape);
        // Window each sub-window. Sub-window 0 uses `prev_short` for its
        // rising side (left), `cur_short` for its falling side (right).
        // Sub-windows 1..7 use `cur_short` on both sides.
        for n_ in 0..SHORT_LEN {
            shorts[0][n_] *= prev_short[n_];
            shorts[0][SHORT_LEN + n_] *= cur_short[SHORT_LEN - 1 - n_];
        }
        for w in 1..8 {
            for n_ in 0..SHORT_LEN {
                shorts[w][n_] *= cur_short[n_];
                shorts[w][SHORT_LEN + n_] *= cur_short[SHORT_LEN - 1 - n_];
            }
        }
        // PCM layout per ISO Fig. 4.24:
        //   pcm[0..448]    = state.prev[0..448]
        //   pcm[448..576]  = state.prev[448..576] + s[0][0..128]
        //   pcm[576..704]  = s[0][128..256]      + s[1][0..128]
        //   pcm[704..832]  = s[1][128..256]      + s[2][0..128]
        //   pcm[832..960]  = s[2][128..256]      + s[3][0..128]
        //   pcm[960..1024] = s[3][128..192]
        for i in 0..448 {
            pcm[i] = state.prev[i];
        }
        for i in 0..SHORT_LEN {
            pcm[448 + i] = state.prev[448 + i] + shorts[0][i];
            pcm[576 + i] = shorts[0][SHORT_LEN + i] + shorts[1][i];
            pcm[704 + i] = shorts[1][SHORT_LEN + i] + shorts[2][i];
            pcm[832 + i] = shorts[2][SHORT_LEN + i] + shorts[3][i];
        }
        for i in 0..64 {
            pcm[960 + i] = shorts[3][SHORT_LEN + i];
        }
        // Build new overlap (= s[3][192..256]+s[4][0..64], etc.):
        //   prev[0..64]    = s[3][192..256] + s[4][0..64]
        //   prev[64..192]  = s[4][64..192]  + s[5][0..128]
        //   prev[192..320] = s[5][128..256] + s[6][0..128]
        //   prev[320..448] = s[6][128..256] + s[7][0..128]
        //   prev[448..576] = s[7][128..256]
        //   prev[576..1024]= 0
        for i in 0..64 {
            state.prev[i] = shorts[3][SHORT_LEN + 64 + i] + shorts[4][i];
        }
        for i in 0..SHORT_LEN {
            state.prev[64 + i] = shorts[4][SHORT_LEN + i] + shorts[5][i];
            state.prev[192 + i] = shorts[5][SHORT_LEN + i] + shorts[6][i];
            state.prev[320 + i] = shorts[6][SHORT_LEN + i] + shorts[7][i];
            state.prev[448 + i] = shorts[7][SHORT_LEN + i];
        }
        for i in 576..LONG_LEN {
            state.prev[i] = 0.0;
        }
    }
    state.prev_shape = shape;
    state.prev_seq = seq;
}

/// Apply non-LD AAC-LTP long-term prediction (AOT 4 and the non-LD ER /
/// scalable variants) to one channel's residual spectrum, in place —
/// ISO/IEC 14496-3 §4.6.7.3 (long-window path).
///
/// `spec` holds the dequantised residual `Y_rec` ([`FRAME_LEN`]
/// coefficients). `state.ltp_hist` is the previously reconstructed decoder
/// output (`x_rec(i < 0)`) and `state.prev` is the last aliased half
/// window from the previous frame's IMDCT (`x_rec(0 … N/2−1)`); both must
/// reflect the state *before* this frame's IMDCT.
///
/// The predictor is a single-tap IIR in the time domain:
///
/// ```text
/// x_est(i) = ltp_coef · x_rec(i − M − ltp_lag),   i = 0 … N−1
/// ```
///
/// with `N = 2·FRAME_LEN = 2048` and **`M = 0`** for the non-LD object
/// types (contrast `M = N/2` for ER AAC LD — §4.6.7.3). The
/// reconstructed-sample buffer `x_rec` is arranged so that
/// `x_rec(0 … N/2−1)` is the aliased IMDCT half (`state.prev`),
/// `x_rec(N/2 … N−1)` is zero, and `x_rec(i < 0)` is the prior decoder
/// output (`state.ltp_hist`, newest last). The predicted time signal is
/// windowed with the long analysis window (`window_shape`-selected,
/// using `prev_shape` on the rising half so the predicted spectrum is in
/// the same MDCT domain as `Y_rec`) and forward-MDCT'd; the resulting
/// `X_est` is added to `Y_rec` on every scalefactor band whose
/// `long_used` flag is set, except bands flagged in `skip_bands` (PNS / IS,
/// which take precedence over prediction per §4.6.7.4.2).
///
/// Only long windows are handled here; short-window LTP (the per-window
/// `short_used` / `short_lag` path) is a separate decode step.
pub fn apply_ltp(
    spec: &mut [f32],
    state: &ChannelState,
    ltp: &LtpDataNonLd,
    shape: WindowShape,
    prev_shape: WindowShape,
    swb: &[u16],
    skip_bands: &[bool],
) {
    use crate::mdct::mdct_long;

    let n = 2 * FRAME_LEN; // transform-window length N = 2048
    let m = 0usize; // M = 0 for the non-LD object types
    let lag = ltp.lag as usize;

    // Build x_est(i) = coef · x_rec(i − M − lag) for i = 0 … N−1.
    // x_rec index j: j in [0, N/2) -> state.prev[j]; j in [N/2, N) -> 0;
    // j < 0 -> ltp_hist (ltp_hist[len−1] is the sample at j = −1).
    let hist = &state.ltp_hist;
    let hist_len = hist.len();
    let mut x_est = vec![0.0f32; n];
    for (i, slot) in x_est.iter_mut().enumerate() {
        let j = i as i64 - m as i64 - lag as i64;
        let sample = if j < 0 {
            let idx = hist_len as i64 + j;
            if idx >= 0 {
                hist[idx as usize]
            } else {
                0.0
            }
        } else if (j as usize) < FRAME_LEN {
            state.prev[j as usize]
        } else {
            0.0
        };
        *slot = ltp.coef * sample;
    }

    // Analysis filterbank: window x_est with the long window (rising half
    // governed by `prev_shape`, falling half by `shape` — exactly the
    // window the IMDCT/overlap path applied so X_est lands in the same
    // MDCT domain as Y_rec), then forward-MDCT to X_est.
    let prev_w = long_window_for_shape(prev_shape);
    let cur_w = long_window_for_shape(shape);
    let mut windowed = vec![0.0f32; n];
    for i in 0..FRAME_LEN {
        windowed[i] = x_est[i] * prev_w[i];
        windowed[FRAME_LEN + i] = x_est[FRAME_LEN + i] * cur_w[FRAME_LEN - 1 - i];
    }
    let mut x_est_spec = vec![0.0f32; FRAME_LEN];
    mdct_long(&windowed, &mut x_est_spec);

    // X_rec = X_est + Y_rec on bands where ltp_long_used is set and the
    // band is not a PNS / IS band.
    let num_swb = swb.len().saturating_sub(1);
    for sfb in 0..ltp.long_used.len().min(num_swb) {
        if !ltp.long_used[sfb] {
            continue;
        }
        if skip_bands.get(sfb).copied().unwrap_or(false) {
            continue;
        }
        let start = swb[sfb] as usize;
        let end = (swb[sfb + 1] as usize).min(FRAME_LEN);
        for k in start..end {
            spec[k] += x_est_spec[k];
        }
    }
}

#[cfg(test)]
mod ltp_tests {
    use super::*;
    use crate::ics::LtpDataNonLd;
    use crate::mdct::mdct_long;
    use crate::window::long_window_for_shape;

    /// `push_ltp_history` keeps the newest output sample last and drops the
    /// oldest, holding the buffer at [`LTP_HISTORY_LEN`].
    #[test]
    fn push_ltp_history_orders_newest_last() {
        let mut st = ChannelState::new();
        // First push: a ramp of FRAME_LEN samples.
        let first: Vec<f32> = (0..FRAME_LEN).map(|i| i as f32).collect();
        st.push_ltp_history(&first);
        assert_eq!(st.ltp_hist.len(), LTP_HISTORY_LEN);
        // Newest sample is the last element.
        assert_eq!(st.ltp_hist[LTP_HISTORY_LEN - 1], (FRAME_LEN - 1) as f32);
        // The first half is still zero (history started empty).
        assert_eq!(st.ltp_hist[0], 0.0);

        // Second push of FRAME_LEN: the previous block slides toward index 0.
        let second: Vec<f32> = (0..FRAME_LEN).map(|i| 1000.0 + i as f32).collect();
        st.push_ltp_history(&second);
        assert_eq!(
            st.ltp_hist[LTP_HISTORY_LEN - 1],
            1000.0 + (FRAME_LEN - 1) as f32
        );
        // The first push now occupies the lower half.
        assert_eq!(st.ltp_hist[0], 0.0); // still part of the original ramp's start
        assert_eq!(
            st.ltp_hist[LTP_HISTORY_LEN - FRAME_LEN - 1],
            (FRAME_LEN - 1) as f32
        );
    }

    /// A push of more than [`LTP_HISTORY_LEN`] samples keeps only the tail.
    #[test]
    fn push_ltp_history_truncates_overlong_push() {
        let mut st = ChannelState::new();
        let big: Vec<f32> = (0..LTP_HISTORY_LEN + 100).map(|i| i as f32).collect();
        st.push_ltp_history(&big);
        assert_eq!(st.ltp_hist.len(), LTP_HISTORY_LEN);
        assert_eq!(st.ltp_hist[0], 100.0);
        assert_eq!(
            st.ltp_hist[LTP_HISTORY_LEN - 1],
            (LTP_HISTORY_LEN + 99) as f32
        );
    }

    /// Headline arithmetic check for the non-LD long-window predictor.
    ///
    /// With `M = 0`, `N = 2048` and history filled so that
    /// `x_rec(i − lag)` is a known signal, `apply_ltp` must add exactly the
    /// forward long-MDCT of the (long-windowed) `coef · x_rec(i − lag)` to
    /// the residual on enabled bands — and nothing on disabled bands.
    #[test]
    fn apply_ltp_predicts_lagged_history_through_filterbank() {
        let shape = WindowShape::Sine;
        let lag = 600usize;
        let coef = crate::ics::LTP_COEF[4];

        // Put a recognisable low-frequency tone into the history so the
        // predicted signal has energy in the low SWBs.
        let mut st = ChannelState::new();
        for (j, h) in st.ltp_hist.iter_mut().enumerate() {
            *h = (j as f32 * 0.013).sin();
        }
        // `prev` (aliased IMDCT half, x_rec(0..N/2)) stays zero here so the
        // entire prediction window draws from history (j = i − lag < 0 for
        // i < lag, and i in [lag, N) reads history/prev/zero). Keep it zero
        // to isolate the history-read arithmetic.
        // (state.prev already zero from ::new())

        // Reconstruct the reference x_est(i) = coef · x_rec(i − lag).
        let n = 2 * FRAME_LEN;
        let hist_len = st.ltp_hist.len();
        let mut x_est = vec![0.0f32; n];
        for (i, slot) in x_est.iter_mut().enumerate() {
            let j = i as i64 - lag as i64;
            let sample = if j < 0 {
                let idx = hist_len as i64 + j;
                if idx >= 0 {
                    st.ltp_hist[idx as usize]
                } else {
                    0.0
                }
            } else if (j as usize) < FRAME_LEN {
                st.prev[j as usize]
            } else {
                0.0
            };
            *slot = coef * sample;
        }
        // Window + forward MDCT -> reference X_est.
        let w = long_window_for_shape(shape);
        let mut windowed = vec![0.0f32; n];
        for i in 0..FRAME_LEN {
            windowed[i] = x_est[i] * w[i];
            windowed[FRAME_LEN + i] = x_est[FRAME_LEN + i] * w[FRAME_LEN - 1 - i];
        }
        let mut ref_x_est_spec = vec![0.0f32; FRAME_LEN];
        mdct_long(&windowed, &mut ref_x_est_spec);

        // Two SWBs: [0,16) enabled, [16,32) disabled.
        let swb = [0u16, 16, 32];
        let ltp = LtpDataNonLd {
            lag: lag as u16,
            coef,
            long_used: vec![true, false],
            short_used: Vec::new(),
            short_lag: Vec::new(),
        };
        let skip = vec![false, false];

        // Residual Y_rec = constant so we can isolate the X_est add.
        let mut spec = vec![1.0f32; FRAME_LEN];
        apply_ltp(&mut spec, &st, &ltp, shape, shape, &swb, &skip);

        // Enabled band [0,16): spec == 1 + X_est.
        for k in 0..16 {
            assert!(
                (spec[k] - (1.0 + ref_x_est_spec[k])).abs() < 1e-4,
                "enabled band k={k}: got {} want {}",
                spec[k],
                1.0 + ref_x_est_spec[k]
            );
        }
        // Disabled band [16,32): spec untouched (== 1).
        for k in 16..32 {
            assert!(
                (spec[k] - 1.0).abs() < 1e-6,
                "disabled band k={k} changed: {}",
                spec[k]
            );
        }
    }

    /// PNS / IS bands (flagged in `skip_bands`) are never predicted even
    /// when their `ltp_long_used` flag is set — §4.6.7.4.2.
    #[test]
    fn apply_ltp_skips_pns_is_bands() {
        let st = {
            let mut s = ChannelState::new();
            for (j, h) in s.ltp_hist.iter_mut().enumerate() {
                *h = (j as f32 * 0.02).cos();
            }
            s
        };
        let swb = [0u16, 16, 32];
        let ltp = LtpDataNonLd {
            lag: 300,
            coef: crate::ics::LTP_COEF[3],
            long_used: vec![true, true],
            short_used: Vec::new(),
            short_lag: Vec::new(),
        };
        // Band 0 enabled, band 1 marked PNS/IS (skip).
        let skip = vec![false, true];
        let mut spec = vec![2.0f32; FRAME_LEN];
        apply_ltp(
            &mut spec,
            &st,
            &ltp,
            WindowShape::Sine,
            WindowShape::Sine,
            &swb,
            &skip,
        );
        // Skipped band must be exactly the original residual.
        for k in 16..32 {
            assert!(
                (spec[k] - 2.0).abs() < 1e-6,
                "skipped band k={k} changed: {}",
                spec[k]
            );
        }
        // Enabled band 0 must have moved (prediction added).
        let moved = (0..16).any(|k| (spec[k] - 2.0).abs() > 1e-5);
        assert!(moved, "enabled band 0 received no prediction");
    }
}

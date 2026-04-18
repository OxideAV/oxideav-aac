//! Forward MDCT — the analysis counterpart to [`crate::imdct`].
//!
//! Mirrors the same direct (O(N²)) form. The IMDCT formula (eq. 4.A.43) is
//!   x[n] = 2/N · Σ_{k=0..N/2-1} spec[k] · cos((2π/N)(n+n0)(k+0.5))
//! so the forward transform that round-trips with overlap-add is
//!   spec[k] = Σ_{n=0..N-1} x[n] · cos((2π/N)(n+n0)(k+0.5))
//! with the (2/N) sitting entirely on the inverse side. With a
//! Princen-Bradley sine (or KBD) window applied symmetrically the OLA of
//! consecutive blocks reproduces the input exactly.

use std::f64::consts::PI;
use std::sync::OnceLock;

use crate::imdct::{LONG_INPUT, SHORT_INPUT};

/// Cosine table cached per `input_n`. `tbl[k * (2*input_n) + n]` —
/// inner loop iterates over `n` for a fixed `k`.
struct CosTable {
    tbl: Vec<f32>,
    n_in: usize,
}

impl CosTable {
    fn new(input_n: usize) -> Self {
        let n_total = 2 * input_n;
        let n0 = (input_n as f64 + 1.0) / 2.0;
        let mut tbl = vec![0.0f32; n_total * input_n];
        for k in 0..input_n {
            for n in 0..n_total {
                let arg = (2.0 * PI / n_total as f64) * (n as f64 + n0) * (k as f64 + 0.5);
                tbl[k * n_total + n] = arg.cos() as f32;
            }
        }
        Self { tbl, n_in: input_n }
    }

    #[inline]
    fn row(&self, k: usize) -> &[f32] {
        let n_total = 2 * self.n_in;
        &self.tbl[k * n_total..(k + 1) * n_total]
    }
}

static LONG_COS: OnceLock<CosTable> = OnceLock::new();
static SHORT_COS: OnceLock<CosTable> = OnceLock::new();

fn long_cos() -> &'static CosTable {
    LONG_COS.get_or_init(|| CosTable::new(LONG_INPUT))
}

fn short_cos() -> &'static CosTable {
    SHORT_COS.get_or_init(|| CosTable::new(SHORT_INPUT))
}

fn mdct_direct(time: &[f32], spec: &mut [f32], cos: &CosTable, input_n: usize) {
    let n_total = 2 * input_n;
    debug_assert_eq!(time.len(), n_total);
    debug_assert!(spec.len() >= input_n);
    // Unscaled forward — combined with the existing 2/N inverse scale,
    // sine windows with the partition-of-unity property give exact TDAC
    // OLA reconstruction. Empirically (see `diagnose_alias_map_short`):
    //   IMDCT(MDCT(δ_n))[m] = δ_n[m]              for non-aliased m
    //                        - δ[L-1-n]            for first half (n<L)
    //                        + δ[3L-1-n]           for second half
    // The minus/plus structure makes the cross-block aliases cancel
    // exactly when the next/previous block contributes the right
    // mirror-windowed terms.
    for k in 0..input_n {
        let row = cos.row(k);
        let mut acc = 0.0f32;
        for n in 0..n_total {
            acc += time[n] * row[n];
        }
        spec[k] = acc;
    }
}

/// Long-block MDCT (2048 in, 1024 out).
pub fn mdct_long(time: &[f32], spec: &mut [f32]) {
    mdct_direct(time, spec, long_cos(), LONG_INPUT);
}

/// Short-block MDCT (256 in, 128 out).
pub fn mdct_short(time: &[f32], spec: &mut [f32]) {
    mdct_direct(time, spec, short_cos(), SHORT_INPUT);
}

/// Compute the 1024-coefficient spectrum for an EightShort block.
///
/// `block` is a 2*LONG_LEN (= 2048) sample time-domain region = previous
/// frame's overlap (length LONG_LEN, unwindowed) concatenated with the
/// current frame's 1024 new samples (unwindowed).
///
/// The 8 overlapping short sub-windows sit at offsets
/// `{448 + w*128}` for `w = 0..8`, each consuming 256 samples. Each
/// sub-window is multiplied by a 256-sample window whose first half is
/// `prev_short` for w==0 (to TDAC against the preceding long block)
/// and `cur_short` for w >= 1, and whose second half is always
/// `cur_short` (mirrored to form the falling slope). The 128-coefficient
/// MDCT outputs are concatenated in natural (non-grouped) per-sub-window
/// order — `spec[w*128 .. (w+1)*128]`.
pub fn mdct_short_eightshort(
    block: &[f32],
    cur_shape: crate::syntax::WindowShape,
    prev_shape: crate::syntax::WindowShape,
    spec: &mut [f32; 1024],
) {
    assert!(block.len() >= 2 * crate::window::LONG_LEN);
    let cur_short = crate::window::short_window_for_shape(cur_shape);
    let prev_short = crate::window::short_window_for_shape(prev_shape);
    const SUB_LEN: usize = crate::window::SHORT_LEN; // 128
    const SUB_INPUT: usize = 2 * SUB_LEN; // 256
    let mut sub_time = [0.0f32; SUB_INPUT];
    for w in 0..8 {
        let base = 448 + w * SUB_LEN;
        let rising = if w == 0 { prev_short } else { cur_short };
        for i in 0..SUB_LEN {
            sub_time[i] = block[base + i] * rising[i];
        }
        for i in 0..SUB_LEN {
            sub_time[SUB_LEN + i] = block[base + SUB_LEN + i] * cur_short[SUB_LEN - 1 - i];
        }
        let mut sub_spec = [0.0f32; SUB_LEN];
        mdct_short(&sub_time, &mut sub_spec);
        let dst_base = w * SUB_LEN;
        spec[dst_base..dst_base + SUB_LEN].copy_from_slice(&sub_spec);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imdct::{imdct_long, imdct_short};
    use crate::window::{sine_long, sine_short, LONG_LEN, SHORT_LEN};

    /// MDCT(IMDCT(δ_k)) = 2·δ_k under the (forward 1.0 · inverse 2/N) pair.
    /// The factor-of-2 reflects the basis inner product Σ A[n,k]² = N
    /// over n in [0, 2L). It self-cancels under proper TDAC OLA — see
    /// `long_round_trip_with_sine_ola`.
    #[test]
    fn round_trip_unit_basis_long_gives_2x() {
        let input_n = LONG_INPUT;
        let mut spec = vec![0.0f32; input_n];
        spec[5] = 1.0;
        let mut time = vec![0.0f32; 2 * input_n];
        imdct_long(&spec, &mut time);
        let mut spec2 = vec![0.0f32; input_n];
        mdct_long(&time, &mut spec2);
        let on = spec2[5];
        let off: f32 = (0..input_n)
            .filter(|&k| k != 5)
            .map(|k| spec2[k].abs())
            .sum::<f32>()
            / (input_n as f32 - 1.0);
        assert!((on - 2.0).abs() < 0.05, "on bin = {on}, want 2");
        assert!(off < 0.05, "off energy {off}");
    }

    /// Verify that windowed MDCT followed by windowed IMDCT recovers the
    /// input under sine/sine OLA across two consecutive blocks.
    #[test]
    fn long_round_trip_with_sine_ola() {
        let n = LONG_LEN;
        let n2 = 2 * n;
        // Construct a known sequence: x[i] = sin(2*pi*5*i / N).
        let total = 3 * n;
        let mut x = vec![0.0f32; total];
        for i in 0..total {
            x[i] = (2.0 * PI * 5.0 * i as f64 / n as f64).sin() as f32;
        }
        let win = sine_long();
        // Block 0: time samples 0..2N, windowed with sine.
        let mut t0 = vec![0.0f32; n2];
        // Block 1: time samples N..3N (advances N; overlaps half).
        let mut t1 = vec![0.0f32; n2];
        for i in 0..n {
            t0[i] = x[i] * win[i];
            t0[n + i] = x[n + i] * win[n - 1 - i];
            t1[i] = x[n + i] * win[i];
            t1[n + i] = x[2 * n + i] * win[n - 1 - i];
        }
        let mut s0 = vec![0.0f32; n];
        let mut s1 = vec![0.0f32; n];
        mdct_long(&t0, &mut s0);
        mdct_long(&t1, &mut s1);

        let mut o0 = vec![0.0f32; n2];
        let mut o1 = vec![0.0f32; n2];
        imdct_long(&s0, &mut o0);
        imdct_long(&s1, &mut o1);
        // Re-window.
        for i in 0..n {
            o0[i] *= win[i];
            o0[n + i] *= win[n - 1 - i];
            o1[i] *= win[i];
            o1[n + i] *= win[n - 1 - i];
        }
        // Reconstruct samples N..2N as o0[N..2N] + o1[0..N].
        let mut max_err = 0.0f32;
        for i in 0..n {
            let recon = o0[n + i] + o1[i];
            let want = x[n + i];
            let err = (recon - want).abs();
            if err > max_err {
                max_err = err;
            }
        }
        assert!(max_err < 1e-3, "round-trip max err {max_err}");
    }

    /// Encoder-side EightShort MDCT followed by the decoder-side synth
    /// path must reconstruct the overlap region of a known input signal.
    /// This verifies mdct_short_eightshort is TDAC-correct against the
    /// existing short-window synthesis in synth.rs.
    #[test]
    fn eightshort_round_trip_overlap_region() {
        use crate::imdct::imdct_short;
        use crate::syntax::WindowShape;
        use crate::window::{short_window_for_shape, SHORT_LEN};

        let n = LONG_LEN;
        // Input signal: known sine spanning 3 frames.
        let total = 3 * n;
        let mut x = vec![0.0f32; total];
        for i in 0..total {
            x[i] = (2.0 * PI * 7.0 * i as f64 / n as f64).sin() as f32;
        }

        // Build a 2N-sample block = samples [0, 2N). Run the encoder-side
        // EightShort MDCT on it.
        let mut block = vec![0.0f32; 2 * n];
        block[..2 * n].copy_from_slice(&x[..2 * n]);
        let mut spec = [0.0f32; 1024];
        mdct_short_eightshort(&block, WindowShape::Sine, WindowShape::Sine, &mut spec);

        // Now run 8 × 128-sample IMDCTs, apply the short windows, and
        // overlap-add per the EightShort layout documented in synth.rs.
        // The overlap-add reconstructs samples in [448, 1600) of the
        // 2N-sample block, corresponding to input samples [448, 1600).
        let mut shorts = [[0.0f32; 2 * SHORT_LEN]; 8];
        for w in 0..8 {
            let chunk = &spec[w * SHORT_LEN..(w + 1) * SHORT_LEN];
            imdct_short(chunk, &mut shorts[w]);
        }
        let cur_short = short_window_for_shape(WindowShape::Sine);
        let prev_short = short_window_for_shape(WindowShape::Sine);
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
        // Reconstruct samples [576, 1472) by overlapping adjacent sub-windows.
        let mut recon = vec![0.0f32; 2 * n];
        for i in 0..SHORT_LEN {
            recon[448 + i] += shorts[0][i];
            recon[576 + i] += shorts[0][SHORT_LEN + i] + shorts[1][i];
            recon[704 + i] += shorts[1][SHORT_LEN + i] + shorts[2][i];
            recon[832 + i] += shorts[2][SHORT_LEN + i] + shorts[3][i];
            recon[960 + i] += shorts[3][SHORT_LEN + i] + shorts[4][i];
            recon[1088 + i] += shorts[4][SHORT_LEN + i] + shorts[5][i];
            recon[1216 + i] += shorts[5][SHORT_LEN + i] + shorts[6][i];
            recon[1344 + i] += shorts[6][SHORT_LEN + i] + shorts[7][i];
            recon[1472 + i] += shorts[7][SHORT_LEN + i];
        }
        // The stable overlap region where two sub-windows sum (TDAC valid)
        // is [576, 1472). Check reconstruction there. MDCT/IMDCT round-trip
        // under the sine-window OLA scheme gives back the input directly
        // (no scaling) because windowed analysis + windowed synthesis +
        // sin²+cos² overlap cancels the factor-of-2 IMDCT amplification.
        let mut max_err = 0.0f32;
        for i in 576..1472 {
            let got = recon[i];
            let want = x[i];
            let err = (got - want).abs();
            if err > max_err {
                max_err = err;
            }
        }
        assert!(max_err < 5e-3, "eight-short OLA max err {max_err}");
    }

    #[test]
    fn short_round_trip_dc() {
        // Constant DC signal through MDCT/IMDCT/OLA should be near-constant
        // in the overlap region.
        let n = SHORT_LEN;
        let n2 = 2 * n;
        let win = sine_short();
        let mut t0 = vec![0.0f32; n2];
        let mut t1 = vec![0.0f32; n2];
        for i in 0..n {
            t0[i] = win[i]; // x[i] = 1.0; block 0 covers x[0..2N]
            t0[n + i] = win[n - 1 - i];
            t1[i] = win[i]; // block 1 covers x[N..3N]
            t1[n + i] = win[n - 1 - i];
        }
        let mut s0 = vec![0.0f32; n];
        let mut s1 = vec![0.0f32; n];
        mdct_short(&t0, &mut s0);
        mdct_short(&t1, &mut s1);
        let mut o0 = vec![0.0f32; n2];
        let mut o1 = vec![0.0f32; n2];
        imdct_short(&s0, &mut o0);
        imdct_short(&s1, &mut o1);
        for i in 0..n {
            o0[i] *= win[i];
            o0[n + i] *= win[n - 1 - i];
            o1[i] *= win[i];
            o1[n + i] *= win[n - 1 - i];
        }
        // x[N+n] = 1.0 ; reconstruction comes from the right half of o0
        // plus the left half of o1.
        for i in 0..n {
            let recon = o0[n + i] + o1[i];
            assert!((recon - 1.0).abs() < 1e-3, "short OLA at {i}: {recon}");
        }
    }
}

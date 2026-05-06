//! IMDCT (Inverse Modified DCT) for AAC — ISO/IEC 14496-3 §4.6.18.
//!
//! AAC IMDCT formula (eq. 4.A.43):
//!   x_{i,n} = 2/N * sum_{k=0}^{N/2 - 1} spec[k] *
//!            cos( (2π / N) * (n + n0) * (k + 0.5) )
//! for n = 0..N where N = 2048 (long) or 256 (short) and
//! n0 = (N/2 + 1) / 2.
//!
//! This file implements a direct (O(N²)) form for clarity. For 1024 long
//! blocks per frame the cost is ~2M ops/block — fine for our acceptance
//! bar (1 second of audio). A radix-2 split could replace this later.

use std::f64::consts::PI;
use std::sync::OnceLock;

/// Long IMDCT input length (= 1024); output length is 2N = 2048.
pub const LONG_INPUT: usize = 1024;
/// Short IMDCT input length (= 128); output length is 256.
pub const SHORT_INPUT: usize = 128;
/// AAC-LD / AAC-ELD long-window IMDCT input length (= 512); output is 1024.
///
/// ISO/IEC 14496-3 §4.6.18 — LD's filterbank uses a 512-sample (or 480-
/// sample) IMDCT instead of the 2048-sample one used by AAC-LC. Output
/// is `2 * input_n` samples = 1024 for the 512-input case (480-input
/// case yields 960; see `LD_480_INPUT`). Same direct cosine kernel as
/// long/short — see `imdct_n`.
pub const LD_512_INPUT: usize = 512;
/// AAC-LD 480-sample IMDCT input length; output is 960 samples.
pub const LD_480_INPUT: usize = 480;

/// Cache of cosine tables, one per N.
struct CosTable {
    /// `tbl[n][k]` = cos((2π/N)(n+n0)(k+0.5)) — but we lay it out flat.
    tbl: Vec<f32>,
    n: usize,
}

impl CosTable {
    fn new(input_n: usize) -> Self {
        // Per ISO/IEC 14496-3 eq. 4.A.43 the IMDCT uses N = 2*input_n and
        // n0 = (N/2 + 1)/2 = (input_n + 1) / 2.
        let n0 = (input_n as f64 + 1.0) / 2.0;
        let mut tbl = vec![0.0f32; (2 * input_n) * input_n];
        for n in 0..(2 * input_n) {
            for k in 0..input_n {
                let arg = (2.0 * PI / (2.0 * input_n as f64)) * (n as f64 + n0) * (k as f64 + 0.5);
                tbl[n * input_n + k] = arg.cos() as f32;
            }
        }
        Self { tbl, n: input_n }
    }

    fn get(&self, n: usize, k: usize) -> f32 {
        self.tbl[n * self.n + k]
    }
}

static LONG_COS: OnceLock<CosTable> = OnceLock::new();
static SHORT_COS: OnceLock<CosTable> = OnceLock::new();
static LD_512_COS: OnceLock<CosTable> = OnceLock::new();
static LD_480_COS: OnceLock<CosTable> = OnceLock::new();

fn long_cos() -> &'static CosTable {
    LONG_COS.get_or_init(|| CosTable::new(LONG_INPUT))
}

fn short_cos() -> &'static CosTable {
    SHORT_COS.get_or_init(|| CosTable::new(SHORT_INPUT))
}

fn ld_512_cos() -> &'static CosTable {
    LD_512_COS.get_or_init(|| CosTable::new(LD_512_INPUT))
}

fn ld_480_cos() -> &'static CosTable {
    LD_480_COS.get_or_init(|| CosTable::new(LD_480_INPUT))
}

/// Compute the IMDCT of `spec[0..input_n]` into `out[0..2*input_n]`.
///
/// Per ISO/IEC 14496-3 §4.6.11.3.1:
///
/// ```text
/// x[i][n] = (2/N) * Σ_{k=0}^{N/2-1} spec[i][k] * cos((2π/N)(n+n0)(k+½))
/// ```
///
/// where `N` is the *window* length = `2 * input_n`. The spec-prescribed
/// scale factor would be `2/N = 1/input_n`, but this implementation pairs
/// an unscaled forward MDCT (see [`crate::mdct`]) with a `2/input_n` scale
/// here, which yields exact TDAC round-trip at gain 2. The missing factor
/// of 2 relative to the spec formula is compensated at the PCM-output
/// stage by scaling down when converting to the S16 sample format.
fn imdct_direct(spec: &[f32], out: &mut [f32], cos: &CosTable, input_n: usize) {
    debug_assert_eq!(spec.len(), input_n);
    debug_assert!(out.len() >= 2 * input_n);
    let scale = 2.0f32 / (input_n as f32);
    for n in 0..(2 * input_n) {
        let mut acc = 0.0f32;
        for k in 0..input_n {
            acc += spec[k] * cos.get(n, k);
        }
        out[n] = scale * acc;
    }
}

/// Long-block IMDCT (1024 in, 2048 out).
pub fn imdct_long(spec: &[f32], out: &mut [f32]) {
    imdct_direct(spec, out, long_cos(), LONG_INPUT);
}

/// Short-block IMDCT (128 in, 256 out).
pub fn imdct_short(spec: &[f32], out: &mut [f32]) {
    imdct_direct(spec, out, short_cos(), SHORT_INPUT);
}

/// AAC-LD 512-sample IMDCT (512 in, 1024 out).
///
/// Same direct cosine kernel as `imdct_long`, just sized to N/2 = 512
/// per ISO/IEC 14496-3 §4.6.18. Used by both AAC-LD (objectType 23)
/// and AAC-ELD (objectType 39) when `frame_length_flag = 0`.
pub fn imdct_ld_512(spec: &[f32], out: &mut [f32]) {
    imdct_direct(spec, out, ld_512_cos(), LD_512_INPUT);
}

/// AAC-LD 480-sample IMDCT (480 in, 960 out) for broadcast LD/ELD profiles
/// where `frame_length_flag = 1`.
pub fn imdct_ld_480(spec: &[f32], out: &mut [f32]) {
    imdct_direct(spec, out, ld_480_cos(), LD_480_INPUT);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imdct_short_runs() {
        let spec = [0.0f32; SHORT_INPUT];
        let mut out = [0.0f32; 2 * SHORT_INPUT];
        imdct_short(&spec, &mut out);
        for v in out {
            assert_eq!(v, 0.0);
        }
    }

    #[test]
    fn imdct_long_dc_bin() {
        // A single non-zero spectral coefficient at k=0 should produce a
        // pattern proportional to cos((2π/N)(n+n0)(0.5)).
        let mut spec = [0.0f32; LONG_INPUT];
        spec[0] = 1.0;
        let mut out = vec![0.0f32; 2 * LONG_INPUT];
        imdct_long(&spec, &mut out);
        // Sum of magnitudes shouldn't be zero.
        let mag: f32 = out.iter().map(|&v| v.abs()).sum();
        assert!(mag > 0.0);
    }

    /// Sanity: the LD-512 IMDCT runs and produces 2N=1024 samples.
    #[test]
    fn imdct_ld_512_runs() {
        let spec = [0.0f32; LD_512_INPUT];
        let mut out = vec![0.0f32; 2 * LD_512_INPUT];
        imdct_ld_512(&spec, &mut out);
        for v in &out {
            assert_eq!(*v, 0.0);
        }
    }

    /// Sanity: the LD-480 IMDCT runs and produces 960 samples.
    #[test]
    fn imdct_ld_480_runs() {
        let spec = [0.0f32; LD_480_INPUT];
        let mut out = vec![0.0f32; 2 * LD_480_INPUT];
        imdct_ld_480(&spec, &mut out);
        for v in &out {
            assert_eq!(*v, 0.0);
        }
    }

    /// LD-512 IMDCT(δ_0) must produce a non-zero pattern (DC bin smear).
    #[test]
    fn imdct_ld_512_dc_bin_nonzero() {
        let mut spec = [0.0f32; LD_512_INPUT];
        spec[0] = 1.0;
        let mut out = vec![0.0f32; 2 * LD_512_INPUT];
        imdct_ld_512(&spec, &mut out);
        let mag: f32 = out.iter().map(|&v| v.abs()).sum();
        assert!(mag > 0.0, "LD-512 DC bin should produce non-zero output");
    }

    /// LD-480 IMDCT(δ_k) for a few different bins.
    #[test]
    fn imdct_ld_480_random_bins_nonzero() {
        for k in [0usize, 7, 60, 239, 479] {
            let mut spec = [0.0f32; LD_480_INPUT];
            spec[k] = 1.0;
            let mut out = vec![0.0f32; 2 * LD_480_INPUT];
            imdct_ld_480(&spec, &mut out);
            let mag: f32 = out.iter().map(|&v| v.abs()).sum();
            assert!(mag > 0.0, "LD-480 IMDCT(δ_{k}) magnitude is zero");
        }
    }
}

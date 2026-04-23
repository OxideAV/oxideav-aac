//! 64-channel complex QMF analysis and synthesis filterbanks for SBR.
//!
//! ISO/IEC 14496-3 §4.6.18.4.1 (analysis) and §4.6.18.4.2 (synthesis).
//!
//! The analysis filterbank takes 32 time-domain input samples per call and
//! outputs a column of 64 complex-valued subband samples (2× oversampled —
//! we'd normally get 32 real subbands from a 32-ch bank, but the complex
//! modulation produces 64 complex subbands).
//!
//! Actually per the spec: the analysis bank is 32-channel but with complex
//! exponential modulation, producing W[k][l] for k = 0..32 and one
//! subsample l per call. Wait — re-reading: "32 new subband samples" per
//! loop. So the ANALYSIS bank is 32-subband complex. The SYNTHESIS bank at
//! 2× output rate is 64-subband.
//!
//! That matches: AAC core outputs 1024 samples; analysis bank runs
//! 1024/32 = 32 times, producing a 32×32 complex matrix W[k=0..31][l=0..31].
//! SBR extends this to 64 subbands by filling subbands 32..63 from the HF
//! generator. Synthesis bank runs at 2× rate, producing 2048 output samples
//! from a 64×32 complex matrix X[k=0..63][l=0..31].
//!
//! For the 960-sample variant, numTimeSlots=15 so analysis runs 30 times.

use super::tables::QMF_C;
use super::Complex32;

/// Number of QMF subbands in the analysis bank.
pub const ANALYSIS_BANDS: usize = 32;
/// Number of QMF subbands in the (full-rate) synthesis bank.
pub const SYNTHESIS_BANDS: usize = 64;

/// State for the 32-channel analysis QMF bank.
///
/// Holds a 320-sample history buffer into which new samples are shifted.
#[derive(Clone, Debug)]
pub struct QmfAnalysis {
    x: [f32; 320],
}

impl QmfAnalysis {
    pub fn new() -> Self {
        Self { x: [0.0; 320] }
    }

    /// Push 32 time-domain samples and produce one column of 32 complex
    /// subband samples (`out[k]` = W[k][l] for the current subsample l).
    pub fn process(&mut self, input: &[f32; 32], out: &mut [Complex32; ANALYSIS_BANDS]) {
        // Shift by 32 positions — oldest 32 discarded.
        for n in (32..320).rev() {
            self.x[n] = self.x[n - 32];
        }
        // Insert 32 newest samples at positions 0..31 (position 0 is newest).
        // Spec's flowchart writes x[31] = s[0], x[30] = s[1], ... so that
        // after shifting, x[0] is the most recent sample. We match by
        // reversing: x[n] = input[31-n] for n = 0..31.
        for n in 0..32 {
            self.x[n] = input[31 - n];
        }
        // z[n] = x[n] * c[2n] for n = 0..319.
        let mut z = [0.0f32; 320];
        for n in 0..320 {
            z[n] = self.x[n] * QMF_C[2 * n];
        }
        // u[n] = sum_j=0..4 z[n + j*64] for n = 0..63.
        let mut u = [0.0f32; 64];
        for n in 0..64 {
            u[n] = z[n];
            for j in 1..=4 {
                u[n] += z[n + j * 64];
            }
        }
        // W[k] = sum_{n=0..63} u[n] * 2 * exp(i * pi/64 * (k + 0.5) * (2n - 0.5))
        // Precompute trig for efficiency.
        for k in 0..ANALYSIS_BANDS {
            let kf = k as f32 + 0.5;
            let mut re = 0.0f32;
            let mut im = 0.0f32;
            for n in 0..64 {
                let ang = core::f32::consts::PI / 64.0 * kf * (2.0 * n as f32 - 0.5);
                re += u[n] * 2.0 * ang.cos();
                im += u[n] * 2.0 * ang.sin();
            }
            out[k] = Complex32::new(re, im);
        }
    }
}

impl Default for QmfAnalysis {
    fn default() -> Self {
        Self::new()
    }
}

/// State for the 64-channel synthesis QMF bank.
///
/// Holds a 1280-sample v buffer.
#[derive(Clone, Debug)]
pub struct QmfSynthesis {
    v: [f32; 1280],
}

impl QmfSynthesis {
    pub fn new() -> Self {
        Self { v: [0.0; 1280] }
    }

    /// Consume one column of 64 complex subband samples
    /// (`input[k]` = X[k][l]) and produce 64 real output samples.
    pub fn process(&mut self, input: &[Complex32; SYNTHESIS_BANDS], output: &mut [f32; 64]) {
        // Shift v by 128.
        for n in (128..1280).rev() {
            self.v[n] = self.v[n - 128];
        }
        // Write 128 new samples into positions 0..127 via the N modulation.
        // v[n] = Re( sum_k X[k][l]/64 * exp(i * pi/128 * (k+0.5) * (2n - 255)) )
        // for n = 0..127.
        for n in 0..128 {
            let mut re = 0.0f32;
            for k in 0..SYNTHESIS_BANDS {
                let kf = k as f32 + 0.5;
                let ang = core::f32::consts::PI / 128.0 * kf * (2.0 * n as f32 - 255.0);
                // Re(X * (cos+isin)) = X.re*cos - X.im*sin
                let x = input[k];
                re += (x.re * ang.cos() - x.im * ang.sin()) / 64.0;
            }
            self.v[n] = re;
        }
        // g[128n + k] = v[256n + k], g[128n + 64 + k] = v[256n + 192 + k] for n=0..4, k=0..63.
        let mut g = [0.0f32; 640];
        for n in 0..5 {
            for k in 0..64 {
                g[128 * n + k] = self.v[256 * n + k];
                g[128 * n + 64 + k] = self.v[256 * n + 192 + k];
            }
        }
        // w[n] = g[n] * c[n]
        let mut w = [0.0f32; 640];
        for n in 0..640 {
            w[n] = g[n] * QMF_C[n];
        }
        // output[k] = sum_{n=0..9} w[64*n + k] for k=0..63
        for k in 0..64 {
            let mut s = w[k];
            for n in 1..=9 {
                s += w[64 * n + k];
            }
            output[k] = s;
        }
    }
}

impl Default for QmfSynthesis {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn qmf_analysis_zero_input() {
        let mut qa = QmfAnalysis::new();
        let inp = [0.0f32; 32];
        let mut out = [Complex32::new(0.0, 0.0); ANALYSIS_BANDS];
        qa.process(&inp, &mut out);
        for k in 0..ANALYSIS_BANDS {
            assert!(out[k].re.abs() < 1e-6);
            assert!(out[k].im.abs() < 1e-6);
        }
    }

    #[test]
    fn qmf_synthesis_zero_input() {
        let mut qs = QmfSynthesis::new();
        let inp = [Complex32::new(0.0, 0.0); SYNTHESIS_BANDS];
        let mut out = [0.0f32; 64];
        qs.process(&inp, &mut out);
        for k in 0..64 {
            assert!(out[k].abs() < 1e-6);
        }
    }

    /// Verify a DC input produces energy only in subband 0 (approximately).
    #[test]
    fn qmf_analysis_dc_goes_to_band0() {
        let mut qa = QmfAnalysis::new();
        let mut out = [Complex32::new(0.0, 0.0); ANALYSIS_BANDS];
        // Push DC into the bank many times to fill state.
        let dc = [1.0f32; 32];
        for _ in 0..20 {
            qa.process(&dc, &mut out);
        }
        // After the bank is steady-state, subband 0 should have nonzero
        // magnitude and upper subbands should be much smaller.
        let e0 = out[0].re * out[0].re + out[0].im * out[0].im;
        let e_mid = out[15].re * out[15].re + out[15].im * out[15].im;
        assert!(e0 > e_mid * 10.0, "e0={e0} vs e_mid={e_mid}");
    }
}

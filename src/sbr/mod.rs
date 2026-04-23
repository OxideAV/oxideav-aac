//! Spectral Band Replication (SBR) — ISO/IEC 14496-3 §4.6.18.
//!
//! HE-AACv1 extends AAC-LC by transmitting a low-band AAC-LC signal along
//! with compact "envelope" metadata describing the high-band spectral
//! energy. The SBR decoder:
//!
//! 1. Analyses the low-band PCM through a 32-subband complex QMF bank,
//!    producing `XLow[k=0..31][l=0..31]`.
//! 2. Generates high-band subbands by patching low-band subbands up into
//!    positions `kx..kx+M`, yielding `XHigh[k][l]`.
//! 3. Adjusts the HF envelope to match the transmitted scalefactors,
//!    optionally adding noise and sinusoids.
//! 4. Runs the result through a 64-subband complex synthesis QMF bank at
//!    2× the core sample rate, producing PCM output at twice the input rate.
//!
//! This module implements a mono HE-AACv1 decode path. CPE / PS
//! (HE-AACv2) are not implemented.

pub mod bitstream;
pub mod decode;
pub mod freq;
pub mod hf_adjust;
pub mod hf_gen;
pub mod qmf;
pub mod tables;

/// Minimal complex-32 type to avoid adding `num-complex` as a dependency.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Complex32 {
    pub re: f32,
    pub im: f32,
}

impl Complex32 {
    #[inline]
    pub const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }
    #[inline]
    pub fn norm_sqr(self) -> f32 {
        self.re * self.re + self.im * self.im
    }
    #[inline]
    pub fn conj(self) -> Self {
        Self::new(self.re, -self.im)
    }
    #[inline]
    pub fn scale(self, s: f32) -> Self {
        Self::new(self.re * s, self.im * s)
    }
}

impl core::ops::Add for Complex32 {
    type Output = Complex32;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl core::ops::Sub for Complex32 {
    type Output = Complex32;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl core::ops::Mul for Complex32 {
    type Output = Complex32;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.re * rhs.re - self.im * rhs.im,
            self.re * rhs.im + self.im * rhs.re,
        )
    }
}

impl core::ops::AddAssign for Complex32 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.re += rhs.re;
        self.im += rhs.im;
    }
}

/// Number of QMF time-slots per SBR frame when the AAC core uses 1024-sample
/// frames. This is the dominant AAC-LC configuration.
pub const NUM_TIME_SLOTS_1024: usize = 16;
/// Number of QMF time-slots per SBR frame for the 960-sample AAC core.
pub const NUM_TIME_SLOTS_960: usize = 15;
/// QMF subsamples per time-slot (§4.6.18.2.5, `RATE = 2`).
pub const RATE: usize = 2;
/// Number of QMF subbands (synthesis bank).
pub const NUM_QMF_BANDS: usize = 64;
/// Offset from the envelope-adjuster frame origin (§4.6.18.7.1). Used when
/// accessing XHigh / storing Y; the encoder places data shifted by this
/// number of QMF subsamples.
pub const T_HF_ADJ: usize = 2;
/// Offset from the HF-generator frame origin (§4.6.18.6).
pub const T_HF_GEN: usize = 8;

/// SBR sample-rate mapping, §4.6.18 Table 4.82: the SBR internal sample
/// rate is twice the AAC core rate, except for certain low-bitrate rates
/// that get mapped upward.
pub fn sbr_internal_rate(core_rate: u32) -> u32 {
    // Per Table 4.82 most rates double straight up; some explicit mappings:
    match core_rate {
        // 8 / 11.025 / 12 / 16 / 22.05 / 24 / 32 / 44.1 / 48 kHz — double.
        r if r > 0 => r * 2,
        _ => 0,
    }
}

//! Gapless-playback metadata for AAC streams (encoder delay + end-of-file
//! padding tracking).
//!
//! ## Why this exists
//!
//! AAC's MDCT/IMDCT pipeline introduces an unavoidable *encoder delay*:
//! the very first decoded sample reconstructed from the very first
//! `raw_data_block()` corresponds to a moment in time that is **earlier**
//! than the start of the input PCM. The reason is the time-domain aliasing
//! cancellation (TDAC) overlap-add structure (ISO/IEC 14496-3 §4.6.11):
//!
//! - Frame 0 is windowed against an all-zero "previous overlap", so its
//!   left half decodes to silence and its right half is the first 1024
//!   real input samples.
//! - The encoder must therefore prime by emitting one more frame at flush
//!   time than the number of input frames, just to spill out the last
//!   real samples held in the overlap buffer.
//!
//! Apple's iTunes encoder convention adopted in the early 2000s (and now
//! the de-facto standard for both ADTS and MP4-wrapped AAC) is:
//!
//! | Profile     | Encoder delay (high-rate samples) |
//! |-------------|------------------------------------|
//! | AAC-LC      | 2112                               |
//! | HE-AAC v1/v2| 2624                               |
//!
//! The 2112-sample LC value breaks down as 2 × 1024 (one frame of MDCT
//! priming + one frame of overlap-add reconstruction lag) plus 64 samples
//! of decoder-internal latency. The HE-AAC value of 2624 adds 512 samples
//! for the SBR analysis/synthesis QMF chain (the QMF is a 64-band complex
//! polyphase bank with its own warm-up).
//!
//! At the **end** of the file the encoder emits one trailing silence-padded
//! `raw_data_block()` so the OLA can flush the held overlap. The number of
//! padding samples is therefore:
//!
//! ```text
//!     padding = frames_emitted * 1024 - encoder_delay - input_samples
//! ```
//!
//! ## Carrier
//!
//! There are two well-documented carriers for the (delay, padding,
//! valid-samples) triple:
//!
//! 1. **MP4 / ISOBMFF** — `edts/elst` EditListBox (§8.6.6) skipping the
//!    first `encoder_delay` media-time samples and ending the edit at
//!    `valid_samples`. Decoders that respect `elst` (ffmpeg, AVFoundation)
//!    automatically trim the priming and padding.
//! 2. **iTunSMPB** — Apple iTunes-style metadata atom (or ID3 user-text
//!    frame in non-MP4 containers). The payload is a fixed-format ASCII
//!    string of 12 hex words documented at
//!    <https://wiki.hydrogenaud.io/index.php?title=Apple_iTunes_Encoder_Delay_Sample_Tag>.
//!
//! oxideav-aac does **not** itself emit either carrier — they live in the
//! container layer (oxideav-mp4 for `edts/elst`, an ID3v2 wrapper for
//! iTunSMPB). What this module provides is the precise (delay, padding,
//! valid-samples) values the wrapping container needs, plus a formatter
//! for the iTunSMPB string so callers don't have to know the layout.
//!
//! ## API
//!
//! [`GaplessInfo`] is the shared container; [`GaplessInfo::format_itunsmpb`]
//! emits the canonical Apple-format string. The encoder side updates the
//! triple as samples flow through and exposes it via
//! `AacEncoder::gapless_info()` / `HeAacMonoEncoder::gapless_info()` etc.

/// Encoder delay (in high-rate samples) for AAC-LC. Apple iTunes convention.
///
/// Breakdown: 2 × 1024 (MDCT priming + OLA lag) + 64 sample decoder slack.
/// This is the value Apple's iTunes encoder writes to iTunSMPB for AAC-LC
/// streams and that QuickTime's decoder trims when consuming them.
pub const ENCODER_DELAY_AAC_LC: u32 = 2112;

/// Encoder delay for HE-AAC v1 (SBR) and v2 (PS) at the **high** sample
/// rate (= 2 × core rate). Apple iTunes convention. Adds 512 samples on
/// top of [`ENCODER_DELAY_AAC_LC`] to account for the SBR QMF analysis +
/// synthesis warm-up (64-band complex polyphase bank, 320-coefficient
/// prototype filter).
pub const ENCODER_DELAY_HE_AAC: u32 = 2624;

/// Gapless-playback metadata for one encoded AAC track.
///
/// Carries the three numbers a container needs to round-trip a stream
/// sample-accurately:
///
/// - [`encoder_delay`](Self::encoder_delay) — samples to skip at the
///   start (primer / lookahead).
/// - [`padding_samples`](Self::padding_samples) — silence samples at the
///   end after the last real input sample.
/// - [`valid_samples`](Self::valid_samples) — number of *real* input
///   samples that were encoded (used for the iTunSMPB "original sample
///   count" word and for sizing an MP4 `elst` segment_duration).
///
/// All three are in the **same time base** as the encoded stream's
/// sample rate (i.e. high rate for HE-AAC, core rate for AAC-LC).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GaplessInfo {
    pub encoder_delay: u32,
    pub padding_samples: u32,
    pub valid_samples: u64,
}

impl GaplessInfo {
    /// Build a triple from raw counts.
    pub fn new(encoder_delay: u32, padding_samples: u32, valid_samples: u64) -> Self {
        Self {
            encoder_delay,
            padding_samples,
            valid_samples,
        }
    }

    /// Compute padding given (frames_emitted, frame_len, input_samples).
    ///
    /// Returns the canonical
    /// `frames_emitted * frame_len - encoder_delay - input_samples`
    /// value, saturating at zero. This is the formulation used by both
    /// the iTunes / Apple encoder and FFmpeg's libfdk_aac wrapper.
    pub fn compute_padding(
        encoder_delay: u32,
        frames_emitted: u64,
        frame_len: u32,
        input_samples: u64,
    ) -> u32 {
        let total = frames_emitted.saturating_mul(frame_len as u64);
        let real = (encoder_delay as u64).saturating_add(input_samples);
        total.saturating_sub(real).min(u32::MAX as u64) as u32
    }

    /// Format as an Apple iTunSMPB tag string.
    ///
    /// Layout (from the Hydrogenaudio wiki entry, well-documented public
    /// knowledge): twelve space-separated hexadecimal words. The first
    /// four are the only ones any spec-conforming reader checks; the
    /// remaining eight are reserved / vendor-specific zero-fill that
    /// iTunes always writes and most parsers ignore.
    ///
    /// ```text
    ///   word  width  meaning
    ///   ----  -----  -------
    ///   0     8 hex  always 0x00000000 (reserved / version)
    ///   1     8 hex  encoder_delay (priming samples)
    ///   2     8 hex  padding_samples
    ///   3     16 hex valid_samples (original sample count, 64-bit)
    ///   4-11  8 hex  zero-fill (reserved)
    /// ```
    ///
    /// The leading-space convention iTunes itself uses is preserved so
    /// byte-for-byte round-trips against an iTunes-tagged source compare
    /// equal.
    pub fn format_itunsmpb(&self) -> String {
        format!(
            " {:08X} {:08X} {:08X} {:016X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X}",
            0u32,
            self.encoder_delay,
            self.padding_samples,
            self.valid_samples,
            0u32,
            0u32,
            0u32,
            0u32,
            0u32,
            0u32,
            0u32,
            0u32,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lc_delay_value() {
        assert_eq!(ENCODER_DELAY_AAC_LC, 2112);
        assert_eq!(format!("{:08X}", ENCODER_DELAY_AAC_LC), "00000840");
    }

    #[test]
    fn he_aac_delay_value() {
        assert_eq!(ENCODER_DELAY_HE_AAC, 2624);
        assert_eq!(format!("{:08X}", ENCODER_DELAY_HE_AAC), "00000A40");
    }

    #[test]
    fn padding_compute_basic() {
        // 10 frames * 1024 = 10240 emitted samples. Delay 2112 + 7000
        // input = 9112 valid bytes; padding = 10240 - 9112 = 1128.
        let p = GaplessInfo::compute_padding(2112, 10, 1024, 7000);
        assert_eq!(p, 1128);
    }

    #[test]
    fn padding_saturates_when_overflow() {
        // Tail flush emitted nothing extra (degenerate but possible
        // path): emitted < delay + input → saturate to 0 instead of
        // wrapping around.
        let p = GaplessInfo::compute_padding(2112, 5, 1024, 10000);
        assert_eq!(p, 0);
    }

    #[test]
    fn itunsmpb_format_matches_apple_spec() {
        let info = GaplessInfo::new(2112, 420, 44_100);
        let s = info.format_itunsmpb();
        // Leading space, 12 hex words. Compare the prefix (the first
        // four meaningful words) byte-for-byte.
        let expected_prefix = " 00000000 00000840 000001A4 000000000000AC44";
        assert!(
            s.starts_with(expected_prefix),
            "iTunSMPB string must start with: {expected_prefix}\ngot: {s}"
        );
        // Twelve hex words separated by single spaces, plus the leading
        // space — total 13 spaces.
        assert_eq!(s.matches(' ').count(), 12);
    }

    #[test]
    fn itunsmpb_he_aac_example() {
        // HE-AAC at 48 kHz, 1 second of input, ~17 frames * 1024 = 17408
        // emitted high-rate samples; delay 2624 + 48000 input = 50624;
        // padding = 0 (saturated, since flush-tail isn't accounted in
        // this raw arithmetic). Just verify the format prints both fields.
        let info = GaplessInfo::new(ENCODER_DELAY_HE_AAC, 1056, 48_000);
        let s = info.format_itunsmpb();
        assert!(s.contains("00000A40"), "{s}");
        assert!(s.contains("00000420"), "{s}"); // 1056 == 0x420
        assert!(s.contains("000000000000BB80"), "{s}"); // 48000 == 0xBB80
    }
}

//! AAC-LC decoder and encoder — ISO/IEC 14496-3 / ISO 13818-7.
//!
//! Decoder implements:
//! - ADTS frame header parser (§1.A.2 / 13818-7 §6.2)
//! - AudioSpecificConfig parser (§1.6.2.1)
//! - Huffman codebooks 1-11 + scalefactor (§4.A.1)
//! - SCE / CPE / LFE syntax (§4.6.2 / §4.6.3 / §4.6.10)
//! - Program Config Element (PCE) parser — §4.5.2.1
//! - Channel configurations 1..=7 (mono through 7.1, up to 8 output channels)
//! - Section data + scalefactor reconstruction (§4.6.2.3)
//! - Inverse quantisation x = sign·|q|^(4/3) (§4.6.6)
//! - M/S stereo (§4.6.13)
//! - Intensity stereo — IS (§4.6.8.2.3)
//! - IMDCT 2048/256 with sine + KBD windows + overlap-add (§4.6.18 / §4.6.11)
//! - LongStart / LongStop / EightShort window sequences
//! - TNS synthesis for long AND short windows (§4.6.9)
//! - PNS / perceptual noise substitution (§4.6.13)
//! - Pulse data (§4.6.5) — long-window; conformant-bitstream check
//! - Fill / DSE elements
//!
//! Encoder implements:
//! - AAC-LC long-block output, channel configurations 1..=7
//!   (mono / stereo / 3.0 / 4.0 / 5.0 / 5.1 / 7.1) via SCE + CPE + LFE
//!   element sequencing per §1.6.3
//! - TNS forward filtering on SCE long and short windows
//! - M/S stereo per band in CPE
//! - ADTS wrap with single raw_data_block per frame
//! - PNS + intensity-stereo with detection: long-window SCE + CPE
//!   common-window bands above 4 kHz run `classify_pns_band`
//!   (peak-to-RMS test) and `classify_is_band` (L/R correlation +
//!   energy-ratio test); emission plumbing writes cb 13 / 14 / 15 on
//!   the affected bands with matching scalefactor-stream accumulators
//! - Pulse data: up to 4 outlier quantised coefficients per frame are
//!   moved into `pulse_data()` so the residual fits a cheaper Huffman
//!   codebook. Amplitudes are capped to `|residual| - 1` so the
//!   decoder's sign-from-residual rule reproduces the original sign
//! - Short-block encoder — opt-in via
//!   [`encoder::AacEncoder::set_enable_short_blocks`]. Runs a
//!   per-channel [`transient::TransientDetector`] with a 1-frame
//!   lookahead; transitions through
//!   `OnlyLong → LongStart → EightShort → LongStop → OnlyLong` based on
//!   attack detection. CPE pairs unify their transient flag so both
//!   channels share a window sequence; short CPE drops common_window
//!   (per-channel ics_info, no M/S). LFE is long-only per §4.6.10.
//!   The default encoder path still emits only OnlyLong windows — the
//!   short-block mode is off until the caller opts in, because the
//!   transient-driven path has higher bitrate on tonal content.
//!
//! HE-AACv1 (SBR):
//! - SBR bitstream parsing (§4.6.18, Tables 4.62-4.74) — sbr_header,
//!   sbr_single_channel_element, sbr_grid, sbr_dtdf, sbr_invf,
//!   sbr_envelope, sbr_noise, sbr_sinusoidal_coding.
//! - SBR Huffman tables (Annex 4.A.6.1 — all 10 tables).
//! - 32-channel analysis / 64-channel complex synthesis QMF banks
//!   (§4.6.18.4.1-2, Table 4.A.89 coefficients transcribed from the spec).
//! - Frequency band tables (§4.6.18.3.2): fMaster (bark + linear),
//!   fTableHigh, fTableLow, fTableNoise.
//! - HF generator — copy-up patching with bwArray inverse filtering
//!   (§4.6.18.6). Alpha coefficients default to zero (reduced-quality
//!   patching; full covariance-method LPC is not yet wired in).
//! - HF adjuster — envelope-gain application per-band so the SBR range
//!   tracks the transmitted envelope (§4.6.18.7). Noise and sinusoid
//!   insertion are not yet applied.
//! - Synthesis produces PCM at twice the core sample rate, making mono
//!   HE-AACv1 decode sample-rate-doubled.
//!
//! Not implemented (returns `Error::Unsupported` or stubbed to zeros):
//! - Gain control (§4.6.12)
//! - CCE elements (parsed / emitted as unsupported)
//! - HE-AAC PS / CPE-coupled SBR — return Unsupported when detected
//! - SBR noise/sinusoid insertion, full covariance-method HF LPC,
//!   limiter-band energy compensation (§4.6.18.7.4-5) — simplified
//!   envelope-only adjustment is used instead, sufficient for
//!   audible-bandwidth doubling.
//! - Main / SSR / LTP profiles (§4.6.7-8) — only AAC-LC accepted
//! - VBR rate control
//! - Encoder short-window PNS / IS (short-block path emits them gated
//!   off — decode round-trips but bitrate is loose on percussive
//!   content)

#![allow(
    dead_code,
    clippy::needless_range_loop,
    clippy::unnecessary_cast,
    clippy::doc_lazy_continuation,
    clippy::doc_overindented_list_items,
    clippy::manual_memcpy,
    clippy::too_many_arguments,
    clippy::if_same_then_else
)]

pub mod adts;
pub mod asc;
pub mod decoder;
pub mod encoder;
pub mod huffman;
pub mod huffman_tables;
pub mod ics;
pub mod imdct;
pub mod mdct;
pub mod pce;
pub mod pns;
pub mod pulse;
pub mod sbr;
pub mod sfband;
pub mod syntax;
pub mod synth;
pub mod tns;
pub mod tns_analyse;
pub mod transient;
pub mod window;

use oxideav_codec::{CodecInfo, CodecRegistry, Decoder, Encoder};
use oxideav_core::{CodecCapabilities, CodecId, CodecParameters, CodecTag, Result};

pub const CODEC_ID_STR: &str = "aac";

pub fn register(reg: &mut CodecRegistry) {
    let cid = CodecId::new(CODEC_ID_STR);
    let dec_caps = CodecCapabilities::audio("aac_sw")
        .with_lossy(true)
        .with_intra_only(true)
        // Decoder handles AAC-LC channel configurations 1..=7 (up to 8
        // channels for config 7 / 7.1). Config 0 (PCE-defined) also works
        // as long as the PCE-implied channel count fits.
        .with_max_channels(8)
        .with_max_sample_rate(96_000);
    // AVI / WAVEFORMATEX tags — several historical wFormatTag values
    // have been stamped on AAC streams in the wild:
    //   0x00FF: MPEG-2 AAC "raw" ADTS (the common one).
    //   0x706D: ASCII "mp" — libavformat / ffmpeg lineage.
    //   0x4143: ASCII "AC" — seen in some AAC-LC AVI exports.
    //   0xA106: MPEG-4 AAC (Sony lineage).
    reg.register(
        CodecInfo::new(cid.clone())
            .capabilities(dec_caps)
            .decoder(make_decoder)
            .tags([
                CodecTag::wave_format(0x00FF),
                CodecTag::wave_format(0x706D),
                CodecTag::wave_format(0x4143),
                CodecTag::wave_format(0xA106),
            ]),
    );
    let enc_caps = CodecCapabilities::audio("aac_sw")
        .with_lossy(true)
        .with_intra_only(true)
        // Encoder now orchestrates SCE / CPE / LFE elements for channel
        // configurations 1..=7 (1, 2, 3, 4, 5, 6, 8 channels). 7-channel
        // layouts have no standard configuration and are rejected.
        .with_max_channels(8)
        .with_max_sample_rate(48_000);
    reg.register(
        CodecInfo::new(cid)
            .capabilities(enc_caps)
            .encoder(make_encoder),
    );
}

fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    decoder::make_decoder(params)
}

fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    encoder::make_encoder(params)
}

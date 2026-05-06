//! AAC-LC decoder and encoder — ISO/IEC 14496-3 / ISO 13818-7.
//!
//! Decoder implements:
//! - ADTS frame header parser (§1.A.2 / 13818-7 §6.2)
//! - LOAS AudioSyncStream framing + LATM AudioMuxElement demultiplex
//!   (§1.7.2 / §1.7.3) — single-program, single-layer, frameLengthType=0
//!   subset; multi-program / scalable / CELP / HVXC layouts return
//!   `Error::Unsupported`
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
//! - **Encoder** (mono and stereo): `sbr::encode::SbrEncoder` +
//!   `he_aac_encoder::HeAacMonoEncoder` produce a core AAC-LC SCE +
//!   FIL/EXT_SBR_DATA stream from 2×-rate mono PCM. The stereo path
//!   (`sbr::encode::SbrStereoEncoder` + `HeAacStereoEncoder`) emits a
//!   CPE in **independent coupling** (`bs_coupling=0`, §4.6.18.3.5)
//!   with one envelope+noise block per channel sharing the same SBR
//!   header. Pipeline (per channel for stereo): downsample (31-tap
//!   half-band FIR), 64-band complex analysis QMF, per-high-res-band
//!   envelope energy → 1.5 dB scalefactor, flat noise floor, FIXFIX-
//!   num_env=1 grid, freq-delta Huffman-coded envelope + noise. FIL
//!   element spliced before ID_END inside the raw_data_block; ffmpeg
//!   accepts both mono and stereo streams and decodes to 2× sample
//!   rate. Independent coupling is a strict superset of coupled mode
//!   so we always emit it on round 1; balance-mode encoding can land
//!   later without breaking the bitstream.
//! - SBR bitstream parsing (§4.6.18, Tables 4.62-4.74) — sbr_header,
//!   sbr_single_channel_element, sbr_channel_pair_element (both
//!   bs_coupling modes), sbr_grid, sbr_dtdf, sbr_invf, sbr_envelope,
//!   sbr_noise, sbr_sinusoidal_coding. Balance-mode Huffman tables
//!   wired in for coupled CPE.
//! - SBR Huffman tables (Annex 4.A.6.1 — all 10 tables).
//! - 32-channel analysis / 64-channel complex synthesis QMF banks
//!   (§4.6.18.4.1-2, Table 4.A.89 coefficients transcribed from the spec).
//! - Frequency band tables (§4.6.18.3.2): fMaster (bark + linear),
//!   fTableHigh, fTableLow, fTableNoise.
//! - HF generator — copy-up patching with bwArray inverse filtering
//!   and covariance-method alpha0/alpha1 LPC fit per-subband
//!   (§4.6.18.6, stability-clipped at |alpha|^2 >= 16).
//! - HF adjuster — envelope / noise / sinusoid synthesis + limiter-band
//!   gain compensation (§4.6.18.7). Park-Miller PRNG drives noise.
//!   Sinusoid injected at the mid-odd subband of each flagged high-res
//!   band with a π/2-per-subsample phase.
//! - Synthesis produces PCM at twice the core sample rate. Stereo CPE
//!   pairs (coupled and independent) both decode to 2× rate stereo.
//!
//! HE-AACv2 (Parametric Stereo):
//! - **Encoder**: `he_aac_encoder::HeAacV2Encoder` produces a mono SBR
//!   SCE stream with a no-op PS extension (`bs_extension_id =
//!   EXTENSION_ID_PS = 2`, §4.6.18 Table 4.112) carrying the identity-
//!   stereo `ps_data()` payload (IID = 0 dB, ICC = 1 in all 10 bands,
//!   §8.6.4 Table 8.9). `SbrEncoder::set_emit_ps(true)` lights up the PS
//!   payload on existing mono SBR pipelines without other changes. The
//!   bitstream is HE-AACv2-shaped: ffmpeg accepts it, recognises PS,
//!   upmixes the mono SBR back to stereo at 2× the core sample rate,
//!   and reconstructs `L = R = downmix(input)`. Real per-band IID/ICC
//!   analysis (i.e. preserving the original stereo image rather than
//!   collapsing it to mono) is the next layer above this scaffolding.
//! - `sbr::ps` module parses ps_data() carried in the SBR extended_data
//!   block (bs_extension_id = 2) — IID + ICC envelopes with coarse /
//!   fine resolution.
//! - Simplified time-domain upmix: panning per average IID, short-delay
//!   decorrelator contribution per average ICC. Mono-SBR streams with
//!   PS produce stereo output at 2× sample rate.
//! - IPD/OPD, allpass-chain decorrelator, and true QMF-domain upmix are
//!   not yet implemented.
//!
//! AAC-LD / AAC-ELD / USAC scaffolding:
//! - LD/ELD AudioSpecificConfig is parsed end-to-end (`asc.ld_config` /
//!   `asc.eld_config`); 480- and 512-sample MDCT/IMDCT kernels +
//!   sine half-windows + `LdChannelState`-driven overlap-add filterbank
//!   (`crate::ld_eld::imdct_and_overlap_ld`) are TDAC-validated against
//!   sine inputs at both frame sizes (max error < 5e-3). The ER
//!   raw_data_block decoder + LD-SBR / ELD low-overlap window paths are
//!   not yet implemented; `make_decoder` returns `Error::Unsupported`
//!   for AOT 23 / 39 with a message pointing to the available kernels.
//! - USAC / xHE-AAC (AOT 42) `UsacConfig` scaffold parse:
//!   `crate::usac::parse_usac_config` captures `usacSamplingFrequencyIndex`
//!   (incl. 24-bit explicit escape), `coreSbrFrameLengthIndex`,
//!   `channelConfigurationIndex`, and the first `usacElementType`
//!   (SCE/CPE/LFE/Ext, ISO/IEC 23003-3 Table 9). Frame decode of USAC
//!   is not implemented.
//!
//! Not implemented (returns `Error::Unsupported` or stubbed to zeros):
//! - Gain control (§4.6.12)
//! - CCE elements (parsed / emitted as unsupported)
//! - Main / SSR / LTP profiles (§4.6.7-8) — only AAC-LC accepted
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
pub mod gapless;
pub mod he_aac_encoder;
pub mod huffman;
pub mod huffman_tables;
pub mod ics;
pub mod imdct;
pub mod latm;
pub mod ld_eld;
pub mod loas;
pub mod mdct;
pub mod pce;
pub mod pns;
pub mod psy;
pub mod pulse;
pub mod sbr;
pub mod sfband;
pub mod syntax;
pub mod synth;
pub mod tns;
pub mod tns_analyse;
pub mod transient;
pub mod usac;
pub mod window;

use oxideav_core::{CodecCapabilities, CodecId, CodecParameters, CodecTag, Result};
use oxideav_core::{CodecInfo, CodecRegistry, Decoder, Encoder};

pub const CODEC_ID_STR: &str = "aac";

pub fn register_codecs(reg: &mut CodecRegistry) {
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

/// Unified registration entry point — installs AAC into the codec
/// sub-registry of the supplied [`oxideav_core::RuntimeContext`].
///
/// Also wired into [`oxideav_meta::register_all`] via the
/// [`oxideav_core::register!`] macro below.
pub fn register(ctx: &mut oxideav_core::RuntimeContext) {
    register_codecs(&mut ctx.codecs);
}

oxideav_core::register!("aac", register);

fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    decoder::make_decoder(params)
}

fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    encoder::make_encoder(params)
}

#[cfg(test)]
mod register_tests {
    use super::*;

    #[test]
    fn register_via_runtime_context_installs_codec_factory() {
        let mut ctx = oxideav_core::RuntimeContext::new();
        register(&mut ctx);
        let id = CodecId::new(CODEC_ID_STR);
        assert!(
            ctx.codecs.has_decoder(&id),
            "decoder factory not installed via RuntimeContext"
        );
        assert!(
            ctx.codecs.has_encoder(&id),
            "encoder factory not installed via RuntimeContext"
        );
    }
}

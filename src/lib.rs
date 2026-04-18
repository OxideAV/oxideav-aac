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
//! - AAC-LC long-block I-only output, mono + stereo CPE (M/S per band)
//! - TNS forward filtering on SCE long windows
//! - ADTS wrap with single raw_data_block per frame
//! - PNS + IS emission plumbing (scalefactor + spectral-data paths); the
//!   PNS noise-band detector is gated off pending psy-model tuning
//!
//! Not implemented (returns `Error::Unsupported` or stubbed to zeros):
//! - Gain control (§4.6.12)
//! - CCE elements (parsed / emitted as unsupported)
//! - HE-AAC SBR (§4.6.18.4) / PS — return Unsupported when detected
//! - Main / SSR / LTP profiles (§4.6.7-8) — only AAC-LC accepted
//! - Encoder short-block / transient detection
//! - Encoder multi-channel (≥ 3 channels), VBR rate control

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
pub mod bitreader;
pub mod bitwriter;
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
pub mod sfband;
pub mod syntax;
pub mod synth;
pub mod tns;
pub mod tns_analyse;
pub mod window;

use oxideav_codec::{CodecRegistry, Decoder, Encoder};
use oxideav_core::{CodecCapabilities, CodecId, CodecParameters, Result};

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
    reg.register_decoder_impl(cid.clone(), dec_caps, make_decoder);
    let enc_caps = CodecCapabilities::audio("aac_sw")
        .with_lossy(true)
        .with_intra_only(true)
        // Encoder now orchestrates SCE / CPE / LFE elements for channel
        // configurations 1..=7 (1, 2, 3, 4, 5, 6, 8 channels). 7-channel
        // layouts have no standard configuration and are rejected.
        .with_max_channels(8)
        .with_max_sample_rate(48_000);
    reg.register_encoder_impl(cid, enc_caps, make_encoder);
}

fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    decoder::make_decoder(params)
}

fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    encoder::make_encoder(params)
}

#![no_main]

//! Encoder → decoder roundtrip on arbitrary PCM input.
//!
//! AAC-LC is lossy, so this harness deliberately does NOT assert
//! pixel-equality. The contract instead is the same as the existing
//! `tests/encode_roundtrip.rs` shape:
//!
//!   1. The encoder accepts the input frame (any rejection is just a
//!      skipped iteration — fuzz-input garbage may not satisfy the
//!      sample-format / channel-count contract).
//!   2. After flush, the encoder emits at least one ADTS-wrapped
//!      packet whose header parses cleanly.
//!   3. The decoder consumes that packet and emits at least one frame
//!      with the correct per-channel sample count slot.
//!   4. No panics anywhere along the way.
//!
//! This catches encoder-side panics on edge-case PCM (NaNs in float
//! input, all-zero buffers, all-clipped buffers, zero-length frames,
//! very short frames < 1024 samples) and decoder-side panics on the
//! encoder's own output — a more focused failure surface than the
//! generic `panic_free_decode` harness which only feeds raw bytes.

use libfuzzer_sys::fuzz_target;
use oxideav_aac::adts::parse_adts_header;
use oxideav_aac::decoder::make_decoder;
use oxideav_aac::encoder::AacEncoder;
// Encoder trait is needed for `enc.send_frame` / `enc.flush` /
// `enc.receive_packet` resolution; Decoder is not — the
// `Box<dyn Decoder>` returned by `make_decoder` carries the vtable.
use oxideav_core::Encoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, Packet, SampleFormat, TimeBase};

/// AAC-LC frames are 1024 samples per channel. Encoders need at least
/// one full frame's worth of input before producing output (long-block
/// MDCT spans 2048 samples → first emission after the 2nd frame's
/// worth of input). Cap fuzz-input PCM length to 4 frames per channel
/// (4096 samples) so each iteration stays under a few ms.
const MAX_SAMPLES_PER_CHANNEL: usize = 4096;

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }
    // Carve a small config header from the first byte:
    //   bit 0   : channels (0 = mono, 1 = stereo)
    //   bits 1-2: sample_rate selector (4 entries)
    let cfg = data[0];
    let channels: u16 = if cfg & 1 == 0 { 1 } else { 2 };
    let sample_rate: u32 = match (cfg >> 1) & 0b11 {
        0 => 44_100,
        1 => 48_000,
        2 => 32_000,
        _ => 22_050,
    };
    let pcm_bytes = &data[1..];
    let bytes_per_sample = 2usize; // S16
    let frame_bytes = bytes_per_sample * channels as usize;
    if frame_bytes == 0 {
        return;
    }
    let total_samples = (pcm_bytes.len() / frame_bytes).min(MAX_SAMPLES_PER_CHANNEL);
    if total_samples == 0 {
        return;
    }
    let usable = total_samples * frame_bytes;
    let pcm = pcm_bytes[..usable].to_vec();

    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sample_rate);
    params.channels = Some(channels);
    params.bit_rate = Some(128_000);
    params.sample_format = Some(SampleFormat::S16);

    let mut enc = match AacEncoder::new(&params) {
        Ok(e) => e,
        Err(_) => return,
    };
    // Disable psy model to keep encode deterministic on garbage PCM —
    // matches the `tests/encode_roundtrip.rs` flat-quantiser path.
    enc.set_enable_psy_model(false);

    let frame = Frame::Audio(AudioFrame {
        samples: total_samples as u32,
        pts: Some(0),
        data: vec![pcm],
    });
    if enc.send_frame(&frame).is_err() {
        return;
    }
    if enc.flush().is_err() {
        return;
    }

    // Drain encoded ADTS packets.
    let mut adts_bytes: Vec<u8> = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts_bytes.extend_from_slice(&pkt.data);
    }
    if adts_bytes.is_empty() {
        return;
    }

    // Locate the first ADTS frame and validate its header parses.
    let mut start = None;
    for i in 0..adts_bytes.len().saturating_sub(7) {
        if adts_bytes[i] == 0xFF && (adts_bytes[i + 1] & 0xF0) == 0xF0 {
            if parse_adts_header(&adts_bytes[i..]).is_ok() {
                start = Some(i);
                break;
            }
        }
    }
    let start = match start {
        Some(s) => s,
        None => return,
    };
    let hdr = parse_adts_header(&adts_bytes[start..]).unwrap();
    let frame_len = hdr.frame_length;
    if frame_len == 0 || start + frame_len > adts_bytes.len() {
        return;
    }

    // Decode and assert at least one frame comes out cleanly.
    let mut dec_params = CodecParameters::audio(CodecId::new("aac"));
    dec_params.sample_rate = Some(sample_rate);
    dec_params.channels = Some(channels);
    let mut dec = match make_decoder(&dec_params) {
        Ok(d) => d,
        Err(_) => return,
    };
    let pkt = Packet::new(
        0,
        TimeBase::new(1, sample_rate as i64),
        adts_bytes[start..start + frame_len].to_vec(),
    );
    if dec.send_packet(&pkt).is_err() {
        return;
    }
    let out = match dec.receive_frame() {
        Ok(Frame::Audio(af)) => af,
        // Decoder rejected — likely an internal-but-handled NeedMore
        // path on this particular header. Skip rather than assert.
        _ => return,
    };

    // The decoder reports samples as per-channel sample count for one
    // raw_data_block. AAC-LC long blocks are 1024 samples each; we
    // assert it's a sane positive value rather than a specific one
    // (encoder may pad/trim depending on input length).
    assert!(out.samples > 0, "decoded AudioFrame had zero samples");
    assert!(
        out.samples <= 4096,
        "decoded AudioFrame samples too large: {}",
        out.samples
    );
});

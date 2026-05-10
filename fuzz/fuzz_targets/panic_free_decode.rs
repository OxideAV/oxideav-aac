#![no_main]

//! Panic-free decode of arbitrary bytes.
//!
//! Contract: feeding any byte string to the AAC decoder must return a
//! `Result` (Ok or Err) — never panic, never abort, never recurse to
//! stack overflow. The decoder may legitimately reject the input
//! (`Err(Error::Invalid)` / `Err(Error::Unsupported)` / etc.) — that's
//! fine. Only crashes count as fuzz hits.
//!
//! We try **two** entry points per input so we cover both decode paths:
//!   1. ADTS-wrapped packet decode (the streaming entrypoint, which
//!      auto-configures the decoder from the first ADTS header).
//!   2. raw_data_block decode against a pre-configured decoder built
//!      from a synthetic AudioSpecificConfig — exercises the inner
//!      element / IMDCT / overlap path on bitstreams that aren't ADTS.
//!
//! Both paths are wrapped in `std::panic::catch_unwind`-free code: a
//! libfuzzer-sys harness already converts unhandled panics into the
//! libFuzzer "crash" outcome, which is exactly the failure mode we
//! want to detect. We don't suppress them.

use libfuzzer_sys::fuzz_target;
use oxideav_aac::decoder::make_decoder;
use oxideav_core::{CodecId, CodecParameters, Packet, TimeBase};

fuzz_target!(|data: &[u8]| {
    // -------- Path 1: ADTS / LATM / LOAS streaming decode -----------
    // No extradata → decoder bootstraps from the first ADTS frame.
    {
        let params = CodecParameters::audio(CodecId::new("aac"));
        if let Ok(mut dec) = make_decoder(&params) {
            let pkt = Packet::new(0, TimeBase::new(1, 44100), data.to_vec());
            // Discard return — only crashes matter.
            let _ = dec.send_packet(&pkt);
            // Drain at most a handful of frames to keep the iter fast
            // but still catch state-machine bugs that surface on the
            // 2nd/3rd receive_frame call.
            for _ in 0..4 {
                if dec.receive_frame().is_err() {
                    break;
                }
            }
            let _ = dec.flush();
            for _ in 0..4 {
                if dec.receive_frame().is_err() {
                    break;
                }
            }
        }
    }

    // -------- Path 2: raw_data_block via synthetic ASC -------------
    // Build a 2-byte AudioSpecificConfig for AAC-LC / 44.1 kHz / stereo
    // (object_type=2, sf_index=4, channel_config=2):
    //   bits  high → low
    //     5: AOT          = 2  (AAC-LC)
    //     4: sf_index     = 4  (44100 Hz)
    //     4: channel_cfg  = 2  (stereo)
    //     3: GASpecific   = 0
    //   → 16 bits = 0x12 0x10
    {
        let mut params = CodecParameters::audio(CodecId::new("aac"));
        params.sample_rate = Some(44_100);
        params.channels = Some(2);
        params.extradata = vec![0x12, 0x10];
        if let Ok(mut dec) = make_decoder(&params) {
            let pkt = Packet::new(0, TimeBase::new(1, 44100), data.to_vec());
            let _ = dec.send_packet(&pkt);
            for _ in 0..4 {
                if dec.receive_frame().is_err() {
                    break;
                }
            }
            let _ = dec.flush();
        }
    }

    // -------- Path 3: ADTS header parser in isolation --------------
    // Calling parse_adts_header on each candidate sync-word offset
    // exercises the bit-reader on every prefix length, which is a
    // hot path for malformed streams. Returns Result, never panics.
    {
        for i in 0..data.len().saturating_sub(7) {
            if data[i] == 0xFF && (data[i + 1] & 0xF0) == 0xF0 {
                let _ = oxideav_aac::adts::parse_adts_header(&data[i..]);
            }
        }
    }

    // -------- Path 4: AudioSpecificConfig parser in isolation ------
    {
        let _ = oxideav_aac::asc::parse_asc(data);
    }
});

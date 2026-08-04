//! Deterministic corruption battery over the r437 encoder paths —
//! multichannel (5.1 / 7.1) element plans, grouped `EIGHT_SHORT`
//! frames and short-frame M/S masks.
//!
//! The decoder's contract on hostile input is errors, never panics:
//! every bit flip and every truncation of a self-produced stream must
//! come back as `Ok` (the flip landed somewhere inert) or a clean
//! `Err`.

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder, FRAME_LEN};

/// A transient-bearing multichannel signal (canonical interleave).
fn burst_signal(n: usize, channels: usize) -> Vec<i16> {
    let mut out = Vec::with_capacity(n * channels);
    for i in 0..n {
        let t = i as f64;
        let burst = (FRAME_LEN + 200..FRAME_LEN + 700).contains(&i);
        for c in 0..channels {
            let w = 0.03 + 0.04 * c as f64;
            let v = 6000.0 * (w * t).sin()
                + if burst {
                    15000.0 * (0.55 * t).sin()
                } else {
                    0.0
                };
            out.push(v.round() as i16);
        }
    }
    out
}

fn encode(channels: usize) -> Vec<u8> {
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: channels as u8,
        bitrate: 64_000 * channels as u32,
    })
    .unwrap();
    enc.encode_all(&burst_signal(3 * FRAME_LEN, channels))
        .unwrap()
}

/// Flip a bounded, evenly spread sweep of single bits across the
/// stream (a golden-ratio bit offset per step visits every region
/// and bit lane); decode must never panic.
fn bit_flip_battery(stream: &[u8], flips: usize) {
    let bits = stream.len() * 8;
    let mut decoded_ok = 0usize;
    let mut errored = 0usize;
    for i in 0..flips {
        // Low-discrepancy walk over the bit positions.
        let flip = (i.wrapping_mul(0x9E37_79B9) >> 8) % bits;
        let mut corrupt = stream.to_vec();
        corrupt[flip / 8] ^= 0x80 >> (flip % 8);
        let mut dec = StreamDecoder::new();
        match dec.decode_all(&corrupt) {
            Ok(_) => decoded_ok += 1,
            Err(_) => errored += 1,
        }
    }
    // Sanity: the battery actually exercised both outcomes (an
    // all-Ok run would mean the flips landed nowhere meaningful).
    assert_eq!(decoded_ok + errored, flips);
    assert!(errored > 0, "no flip ever surfaced an error");
}

/// A spread of truncation lengths (byte granularity, both frame
/// boundaries and mid-frame cuts) must decode without panicking.
fn truncation_battery(stream: &[u8], cuts: usize) {
    for i in 0..cuts {
        let len = (i.wrapping_mul(0x9E37_79B9) >> 8) % stream.len();
        let mut dec = StreamDecoder::new();
        let _ = dec.decode_all(&stream[..len]);
    }
}

#[test]
fn stereo_short_window_stream_survives_bit_flips() {
    let stream = encode(2);
    bit_flip_battery(&stream, 60);
    truncation_battery(&stream, 20);
}

#[test]
fn five_one_stream_survives_bit_flips() {
    let stream = encode(6);
    bit_flip_battery(&stream, 40);
    truncation_battery(&stream, 12);
}

#[test]
fn seven_one_stream_survives_bit_flips() {
    let stream = encode(8);
    bit_flip_battery(&stream, 30);
    truncation_battery(&stream, 10);
}

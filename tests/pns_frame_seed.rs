//! The decoder's opt-in per-frame PNS seed
//! ([`StreamDecoder::set_pns_frame_seed`]): every channel element's
//! §4.6.13.3 noise generator restarts at each frame from the frame's
//! own bytes, so a stream coded with perceptual noise substitution
//! decodes the same from any frame on, however much of the stream the
//! decoder saw before. That is what lets a stream be decoded in pieces
//! (a seek, or segments converted one at a time) and joined without a
//! change at the joins.
//!
//! The streams are the crate's own encoder's, PNS on, over white noise:
//! most bands above the lowest few are noise-coded.

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, FRAME_LEN, StreamEncoder};

/// Frames per test stream.
const FRAMES: usize = 24;

/// `n` interleaved frames of `channels` channels of white noise:
/// independent per channel, or the same noise on every channel.
fn noise_pcm(n: usize, channels: usize, same: bool) -> Vec<i16> {
    let mut states: Vec<u32> = (0..channels as u32)
        .map(|c| {
            if same {
                0x1357_9bdf
            } else {
                0x1357_9bdf ^ (c + 1).wrapping_mul(0x9e37_79b9)
            }
        })
        .collect();
    let mut out = Vec::with_capacity(n * channels);
    for _ in 0..n {
        for state in &mut states {
            *state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            out.push(((*state as i32) as f64 / 2_147_483_648.0 * 6_000.0) as i16);
        }
    }
    out
}

/// The ADTS frames of `pcm` encoded with PNS on.
fn pns_frames(pcm: &[i16], channels: usize) -> Vec<Vec<u8>> {
    let mut enc = StreamEncoder::new(EncoderConfig::new(
        48_000,
        channels as u8,
        48_000 * channels as u32,
    ))
    .unwrap();
    enc.set_pns(true);
    enc.set_bandwidth(None);
    let stream = enc.encode_all(pcm).unwrap();
    let mut frames = Vec::new();
    let mut at = 0;
    while at < stream.len() {
        let (header, _) = AdtsHeader::parse(&stream[at..]).unwrap();
        let len = header.aac_frame_length as usize;
        frames.push(stream[at..at + len].to_vec());
        at += len;
    }
    frames
}

/// Each frame's 16-bit and float output, decoding `frames` with a fresh
/// decoder.
fn decode(frames: &[Vec<u8>], per_frame_seed: bool) -> Vec<(Vec<i16>, Vec<f64>)> {
    let mut dec = StreamDecoder::new();
    dec.set_pns_frame_seed(per_frame_seed);
    dec.set_float_pcm_capture(true);
    frames
        .iter()
        .map(|frame| {
            let pcm = dec.decode_adts_frame(frame).unwrap().pcm;
            (pcm, dec.last_pcm_f64().unwrap().to_vec())
        })
        .collect()
}

/// For each start frame `k`, whether a decoder started at `k` gives
/// every frame from `k + 1` on (its first frame lacks the overlap of the
/// one before) exactly as the decoder that ran from the stream's start.
fn joins(frames: &[Vec<u8>], per_frame_seed: bool) -> Vec<bool> {
    let whole = decode(frames, per_frame_seed);
    [3, 8, 15]
        .iter()
        .map(|&k| decode(&frames[k..], per_frame_seed)[1..] == whole[k + 1..])
        .collect()
}

#[test]
fn a_decoder_started_mid_stream_joins_the_continuous_one_exactly() {
    // Mono (one SCE), stereo (a CPE) and 5.1 (SCE, two CPEs, LFE).
    for channels in [1, 2, 6] {
        let frames = pns_frames(&noise_pcm(FRAMES * FRAME_LEN, channels, false), channels);
        assert_eq!(joins(&frames, true), [true; 3], "{channels} channels");
        // The control: with one generator run on from frame to frame, a
        // decoder started elsewhere draws other noise, so the streams
        // do carry noise-coded bands.
        assert_eq!(joins(&frames, false), [false; 3], "{channels} channels");
    }
}

/// Each element of a frame starts its generator from its own state:
/// with the same noise on all six channels, both channel pairs and the
/// center code the same energies, yet decode to unrelated noise.
#[test]
fn the_elements_of_one_frame_draw_unrelated_noise() {
    let frames = pns_frames(&noise_pcm(FRAMES * FRAME_LEN, 6, true), 6);
    let decoded = decode(&frames, true);
    let channel = |c: usize| -> Vec<f64> {
        decoded[2..]
            .iter()
            .flat_map(|(_, pcm)| pcm.iter().skip(c).step_by(6).copied())
            .collect()
    };
    // Canonical 5.1: L R C LFE Ls Rs; L and Ls are the two pairs' first
    // channels, C the single channel element.
    let (left, center, surround) = (channel(0), channel(2), channel(4));
    for (a, b, names) in [
        (&left, &surround, "L and Ls"),
        (&left, &center, "L and C"),
        (&center, &surround, "C and Ls"),
    ] {
        assert!(a != b, "{names} decode identical noise");
    }
}

/// Successive frames draw unrelated noise: a frame's seed comes from its
/// bytes, which differ from frame to frame. One seed for every frame
/// would repeat the noise each 1024 samples, a buzz at the frame rate,
/// which shows as correlation at a lag of one frame.
#[test]
fn successive_frames_draw_unrelated_noise() {
    let frames = pns_frames(&noise_pcm(FRAMES * FRAME_LEN, 1, false), 1);
    let decoded: Vec<f64> = decode(&frames, true)[2..]
        .iter()
        .flat_map(|(_, pcm)| pcm.iter().copied())
        .collect();
    let lag = FRAME_LEN;
    let n = decoded.len() - lag;
    let (mut cross, mut a2, mut b2) = (0.0, 0.0, 0.0);
    for i in 0..n {
        let (a, b) = (decoded[i], decoded[i + lag]);
        cross += a * b;
        a2 += a * a;
        b2 += b * b;
    }
    let correlation = cross / (a2 * b2).sqrt();
    assert!(
        correlation.abs() < 0.05,
        "one-frame lag correlation {correlation}"
    );
}

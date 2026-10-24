//! The opt-in "sine-shaped block before the first"
//! (`StreamDecoder::set_sine_before_first_block`) against the §4.6.11.3.2
//! default, on a stream whose first block is KBD-shaped.

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};

/// ADTS frames of a stereo tone stream, starting mid-stream (frame 4) so
/// the first decoded block carries signal.
fn mid_stream_frames() -> Vec<u8> {
    let n = 12 * 1024;
    let pcm: Vec<i16> = (0..n * 2)
        .map(|i| {
            let t = (i / 2) as f64 / 44_100.0;
            let f = if i % 2 == 0 { 440.0 } else { 660.0 };
            (8000.0 * (2.0 * std::f64::consts::PI * f * t).sin()) as i16
        })
        .collect();
    let stream = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: 2,
        bitrate: 128_000,
    })
    .unwrap()
    .encode_all(&pcm)
    .unwrap();
    let mut pos = 0;
    for _ in 0..4 {
        let len = (usize::from(stream[pos + 3] & 3) << 11)
            | (usize::from(stream[pos + 4]) << 3)
            | (usize::from(stream[pos + 5]) >> 5);
        pos += len;
    }
    stream[pos..].to_vec()
}

/// Set the first frame's `window_shape` bit (a raw field: no other bit
/// moves). The first element must be a common-window CPE, whose
/// `ics_info()` follows `id_syn_ele`, `element_instance_tag` and
/// `common_window` directly.
fn first_block_kbd(frames: &[u8]) -> Vec<u8> {
    let mut d = frames.to_vec();
    let header = if d[1] & 1 == 1 { 7 } else { 9 };
    let bit = |d: &[u8], o: usize| (d[o / 8] >> (7 - o % 8)) & 1;
    let base = header * 8;
    assert_eq!(
        (bit(&d, base) << 2) | (bit(&d, base + 1) << 1) | bit(&d, base + 2),
        1,
        "first element is a CPE"
    );
    assert_eq!(bit(&d, base + 7), 1, "common_window");
    // ics_reserved_bit (1) + window_sequence (2), then window_shape.
    let shape = base + 8 + 3;
    assert_eq!(bit(&d, shape), 0, "encoder writes sine blocks");
    d[shape / 8] |= 1 << (7 - shape % 8);
    d
}

fn decode(stream: &[u8], sine_before_first: bool) -> Vec<Vec<i16>> {
    let mut dec = StreamDecoder::new();
    dec.set_sine_before_first_block(sine_before_first);
    dec.decode_all(stream)
        .unwrap()
        .into_iter()
        .map(|f| f.pcm)
        .collect()
}

#[test]
fn a_kbd_first_block_differs_only_in_its_first_frame() {
    let stream = first_block_kbd(&mid_stream_frames());
    let spec = decode(&stream, false);
    let compat = decode(&stream, true);
    assert_eq!(spec.len(), compat.len());
    assert_ne!(spec[0], compat[0], "the first frame's left half differs");
    assert_eq!(
        spec[1..],
        compat[1..],
        "later blocks follow the coded shapes"
    );
}

#[test]
fn a_sine_first_block_is_unaffected() {
    let stream = mid_stream_frames();
    assert_eq!(decode(&stream, false), decode(&stream, true));
}

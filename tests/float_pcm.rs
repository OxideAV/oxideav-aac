//! The opt-in float PCM view (StreamDecoder::set_float_pcm_capture /
//! last_pcm_f64): the last frame before i16 rounding.

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};

fn tone(n: usize, channels: usize) -> Vec<i16> {
    (0..n * channels)
        .map(|i| {
            let t = (i / channels) as f64 / 44_100.0;
            (8000.0 * (2.0 * std::f64::consts::PI * (440.0 + 110.0 * (i % channels) as f64) * t).sin()) as i16
        })
        .collect()
}

#[test]
fn float_view_is_the_frame_before_rounding() {
    let stream = StreamEncoder::new(EncoderConfig { sample_rate: 44_100, channels: 2, bitrate: 128_000 })
        .unwrap()
        .encode_all(&tone(8 * 1024, 2))
        .unwrap();
    let mut dec = StreamDecoder::new();
    assert!(dec.last_pcm_f64().is_none(), "off by default");
    dec.set_float_pcm_capture(true);
    let frames = dec.decode_all(&stream).unwrap();
    let last = frames.last().unwrap();
    let pcm = dec.last_pcm_f64().unwrap();
    assert_eq!(pcm.len(), last.pcm.len());
    for (&f, &i) in pcm.iter().zip(&last.pcm) {
        let rounded = (f * 32768.0).round().clamp(-32768.0, 32767.0) as i16;
        assert!((i32::from(rounded) - i32::from(i)).abs() <= 1, "{f} vs {i}");
    }
    assert!(pcm.iter().any(|&f| f != 0.0));
    dec.set_float_pcm_capture(false);
    assert!(dec.last_pcm_f64().is_none());
}

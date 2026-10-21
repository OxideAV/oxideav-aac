//! The opt-in core-only decode (StreamDecoder::set_core_only).

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::he_aac_encoder::{HeAacConfig, HeAacEncoder};

#[test]
fn an_he_aac_stream_decodes_its_core_only() {
    let n = 8 * 2048;
    let pcm: Vec<i16> = (0..n * 2)
        .map(|i| {
            let t = (i / 2) as f64 / 44_100.0;
            (5000.0 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin()
                + 2000.0 * (2.0 * std::f64::consts::PI * 12_000.0 * t).sin()) as i16
        })
        .collect();
    let stream = HeAacEncoder::new(HeAacConfig::new(44_100, 2, 48_000)).unwrap().encode_all(&pcm).unwrap();
    let full = StreamDecoder::new().decode_all(&stream).unwrap();
    let mut dec = StreamDecoder::new();
    dec.set_core_only(true);
    let core = dec.decode_all(&stream).unwrap();
    assert_eq!(full.len(), core.len());
    let (f, c) = (full.last().unwrap(), core.last().unwrap());
    assert_eq!(f.pcm.len(), 2 * 1024 * 2, "SBR doubles the rate");
    assert_eq!(c.pcm.len(), 1024 * 2, "the core alone");
    assert_eq!(c.sample_rate * 2, f.sample_rate);
}

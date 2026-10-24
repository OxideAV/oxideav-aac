//! The opt-in declaration channel order (StreamDecoder::set_declaration_order).

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};

#[test]
fn five_one_in_element_order() {
    let n = 6 * 1024;
    let pcm: Vec<i16> = (0..n * 6)
        .map(|i| {
            let (t, c) = ((i / 6) as f64 / 48_000.0, i % 6);
            (6000.0 * (2.0 * std::f64::consts::PI * (200.0 + 300.0 * c as f64) * t).sin()) as i16
        })
        .collect();
    let stream = StreamEncoder::new(EncoderConfig {
        sample_rate: 48_000,
        channels: 6,
        bitrate: 384_000,
    })
    .unwrap()
    .encode_all(&pcm)
    .unwrap();
    let canonical = StreamDecoder::new().decode_all(&stream).unwrap();
    let mut dec = StreamDecoder::new();
    dec.set_declaration_order(true);
    let declared = dec.decode_all(&stream).unwrap();
    assert_eq!(canonical.len(), declared.len());
    // canonical L R C LFE Ls Rs; channelConfiguration 6 declares C, L R, Ls Rs, LFE
    const FROM: [usize; 6] = [2, 0, 1, 4, 5, 3];
    for (c, d) in canonical.iter().zip(&declared) {
        assert_eq!(d.channels, 6);
        for (cf, df) in c
            .pcm
            .as_chunks::<6>()
            .0
            .iter()
            .zip(d.pcm.as_chunks::<6>().0)
        {
            for k in 0..6 {
                assert_eq!(df[k], cf[FROM[k]]);
            }
        }
    }
}

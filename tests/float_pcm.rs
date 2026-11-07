//! The opt-in float PCM view (StreamDecoder::set_float_pcm_capture /
//! last_pcm_f64): the last frame before i16 rounding.

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};

fn tone(n: usize, channels: usize) -> Vec<i16> {
    (0..n * channels)
        .map(|i| {
            let t = (i / channels) as f64 / 44_100.0;
            (8000.0
                * (2.0 * std::f64::consts::PI * (440.0 + 110.0 * (i % channels) as f64) * t).sin())
                as i16
        })
        .collect()
}

#[test]
fn float_view_is_the_frame_before_rounding() {
    let stream = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: 2,
        bitrate: 128_000,
    })
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

/// A configuration change (an ADTS splice to another channel count)
/// restarts the stream but keeps float capture on; the ER entry point
/// fills the view too.
#[test]
fn float_view_survives_a_configuration_change_and_covers_er() {
    let encode = |channels: u8| {
        StreamEncoder::new(EncoderConfig {
            sample_rate: 44_100,
            channels,
            bitrate: 64_000 * u32::from(channels),
        })
        .unwrap()
        .encode_all(&tone(4 * 1024, usize::from(channels)))
        .unwrap()
    };
    let mut spliced = encode(2);
    spliced.extend_from_slice(&encode(1));
    let mut dec = StreamDecoder::new();
    dec.set_float_pcm_capture(true);
    let frames = dec.decode_all(&spliced).unwrap();
    let last = frames.last().unwrap();
    assert_eq!(last.channels, 1);
    let pcm = dec
        .last_pcm_f64()
        .expect("capture stays on across the splice");
    assert_eq!(pcm.len(), last.pcm.len());

    // ER AAC LD through decode_er_raw_data_block.
    let mut enc = StreamEncoder::new_ld(
        EncoderConfig {
            sample_rate: 48_000,
            channels: 2,
            bitrate: 128_000,
        },
        512,
    )
    .unwrap();
    let aus = enc.encode_all_access_units(&tone(4 * 512, 2)).unwrap();
    let mut dec = StreamDecoder::new();
    dec.set_frame_family(oxideav_aac::swb_offset::FrameFamily::Ld512);
    dec.set_float_pcm_capture(true);
    let mut last = None;
    for au in &aus {
        last = Some(
            dec.decode_er_raw_data_block(23, 3, 48_000, 2, Default::default(), au)
                .unwrap(),
        );
    }
    let last = last.unwrap();
    let pcm = dec.last_pcm_f64().unwrap();
    assert_eq!(pcm.len(), last.pcm.len());
    for (&f, &i) in pcm.iter().zip(&last.pcm) {
        let rounded = (f * 32768.0).round().clamp(-32768.0, 32767.0) as i16;
        assert!((i32::from(rounded) - i32::from(i)).abs() <= 1, "{f} vs {i}");
    }
}

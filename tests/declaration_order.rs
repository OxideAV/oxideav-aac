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

/// A `channelConfiguration == 0` block of two all-zero SCEs (tags 0, 1).
fn two_sce_block() -> Vec<u8> {
    let mut w = oxideav_core::bits::BitWriter::new();
    for tag in 0..2 {
        w.write_u32(0, 3); // ID_SCE
        w.write_u32(tag, 4); // element_instance_tag
        w.write_u32(100, 8); // global_gain
        w.write_bit(false); // ics_reserved_bit
        w.write_u32(0, 2); // ONLY_LONG_SEQUENCE
        w.write_bit(false); // window_shape
        w.write_u32(0, 6); // max_sfb
        w.write_bit(false); // predictor_data_present
        w.write_bit(false); // pulse_data_present
        w.write_bit(false); // tns_data_present
        w.write_bit(false); // gain_control_data_present
    }
    w.write_u32(7, 3); // ID_END
    w.finish()
}

#[test]
fn config_zero_without_a_pce_keeps_element_order() {
    let block = two_sce_block();
    let plain = StreamDecoder::new()
        .decode_raw_data_block(2, 4, 44_100, 0, 1, &block)
        .unwrap();
    let mut dec = StreamDecoder::new();
    dec.set_declaration_order(true);
    let declared = dec
        .decode_raw_data_block(2, 4, 44_100, 0, 1, &block)
        .unwrap();
    assert_eq!(declared.channels, 2);
    assert_eq!(declared.pcm, plain.pcm);
}

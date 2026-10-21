//! The opt-in rejection of coupling channel elements
//! (StreamDecoder::set_reject_coupling).

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};
use oxideav_core::bits::BitWriter;

#[test]
fn a_block_with_a_coupling_element_fails() {
    let mut w = BitWriter::new();
    w.write_u32(2, 3); // ID_CCE
    w.write_u32(0, 4); // element_instance_tag
    for _ in 0..8 {
        w.write_u32(0, 8);
    }
    let block = w.finish();
    let mut dec = StreamDecoder::new();
    dec.set_reject_coupling(true);
    let err = dec.decode_raw_data_block(2, 4, 44_100, 2, 1, &block).unwrap_err();
    assert!(format!("{err:?}").contains("UnsupportedElementSkip(2)"), "{err:?}");
}

#[test]
fn a_stream_without_coupling_still_decodes() {
    let pcm: Vec<i16> = (0..4 * 1024 * 2).map(|i| ((i * 37) % 2000) as i16 - 1000).collect();
    let stream = StreamEncoder::new(EncoderConfig { sample_rate: 44_100, channels: 2, bitrate: 128_000 })
        .unwrap()
        .encode_all(&pcm)
        .unwrap();
    let plain = StreamDecoder::new().decode_all(&stream).unwrap();
    let mut dec = StreamDecoder::new();
    dec.set_reject_coupling(true);
    let rejecting = dec.decode_all(&stream).unwrap();
    assert_eq!(plain.len(), rejecting.len());
    for (a, b) in plain.iter().zip(&rejecting) {
        assert_eq!(a.pcm, b.pcm);
    }
}

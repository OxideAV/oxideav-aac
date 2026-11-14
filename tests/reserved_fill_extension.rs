//! A fill element (ISO/IEC 14496-3 §4.4.2.7) carrying an `extension_payload()` whose `extension_type` the decoder has
//! no body layout for must be skipped, not refused. Table 4.51's `default` branch reads `8 * (cnt - 1) + 4`
//! `other_bits` and moves on; Table 4.59 reserves every value but a handful, and `EXT_DATA_ELEMENT` is ancillary data.
//! A stream carrying one decodes to the same samples as the stream without it.

use oxideav_aac::codec_decoder::make_decoder;
use oxideav_aac::codec_encoder::make_encoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Decoder, Encoder, Frame, Packet, TimeBase,
};

fn params(rate: u32, channels: u16, extradata: Vec<u8>) -> CodecParameters {
    let mut p = CodecParameters::audio(CodecId::new("aac"));
    p.sample_rate = Some(rate);
    p.channels = Some(channels);
    p.extradata = extradata;
    p
}

/// Raw access units (ADTS headers stripped) of a short two-channel encode, and the stream's AudioSpecificConfig.
fn access_units() -> (Vec<u8>, Vec<Vec<u8>>) {
    let (rate, channels) = (48_000u32, 2u16);
    let mut pcm = Vec::new();
    for i in 0..rate as usize / 4 {
        let t = i as f64 / f64::from(rate);
        for c in 0..channels {
            let v = 0.3 * (std::f64::consts::TAU * (440.0 + 110.0 * f64::from(c)) * t).sin();
            pcm.push((v * 32767.0) as i16);
        }
    }
    let mut enc: Box<dyn Encoder> = make_encoder(&params(rate, channels, Vec::new())).unwrap();
    let extradata = enc.output_params().extradata.clone();
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: (pcm.len() / channels as usize) as u32,
        pts: Some(0),
        data: vec![pcm.iter().flat_map(|s| s.to_le_bytes()).collect()],
    }))
    .unwrap();
    enc.flush().unwrap();
    let mut units = Vec::new();
    while let Ok(packet) = enc.receive_packet() {
        let (header, offset) = oxideav_aac::adts::AdtsHeader::parse(&packet.data).unwrap();
        assert_eq!(header.aac_frame_length as usize, packet.data.len());
        units.push(packet.data[offset..].to_vec());
    }
    assert!(units.len() > 3);
    (extradata, units)
}

fn push_bits(out: &mut Vec<u8>, used: &mut usize, value: u32, width: usize) {
    for shift in (0..width).rev() {
        if (*used).is_multiple_of(8) {
            out.push(0);
        }
        *out.last_mut().unwrap() |= (((value >> shift) & 1) as u8) << (7 - *used % 8);
        *used += 1;
    }
}

/// `unit` with a fill element of `payload_bytes` bytes (`extension_type` nibble plus `8 * (payload_bytes - 1) + 4`
/// bits taken from `body`, repeating) ahead of its first element.
fn with_fill(unit: &[u8], extension_type: u32, payload_bytes: u32, body: &[u8]) -> Vec<u8> {
    assert!((1..15).contains(&payload_bytes));
    let (mut out, mut used) = (Vec::new(), 0);
    push_bits(&mut out, &mut used, 0b110, 3);
    push_bits(&mut out, &mut used, payload_bytes, 4);
    push_bits(&mut out, &mut used, extension_type, 4);
    for index in 0..(8 * (payload_bytes - 1) + 4) as usize {
        let bit = (body[(index / 8) % body.len()] >> (7 - index % 8)) & 1;
        push_bits(&mut out, &mut used, u32::from(bit), 1);
    }
    for byte in unit {
        push_bits(&mut out, &mut used, u32::from(*byte), 8);
    }
    out
}

fn decode(extradata: &[u8], units: &[Vec<u8>]) -> Vec<u8> {
    let mut decoder: Box<dyn Decoder> =
        make_decoder(&params(48_000, 2, extradata.to_vec())).unwrap();
    let mut out = Vec::new();
    for (index, unit) in units.iter().enumerate() {
        let mut packet = Packet::new(0, TimeBase::new(1, 48_000), unit.clone());
        packet.pts = Some(index as i64 * 1024);
        decoder.send_packet(&packet).unwrap();
        while let Ok(Frame::Audio(audio)) = decoder.receive_frame() {
            out.extend_from_slice(&audio.data[0]);
        }
    }
    out
}

fn expect_same_samples(extension_type: u32, payload_bytes: u32, body: &[u8]) {
    let (extradata, units) = access_units();
    let expected = decode(&extradata, &units);
    assert!(!expected.is_empty());
    let filled: Vec<Vec<u8>> = units
        .iter()
        .map(|unit| with_fill(unit, extension_type, payload_bytes, body))
        .collect();
    assert_eq!(
        decode(&extradata, &filled),
        expected,
        "extension_type {extension_type:#x}, {payload_bytes} bytes"
    );
}

/// Table 4.59 reserves every `extension_type` but 0, 1, 0xb and (13818-7 Table 40) 0xd, 0xe.
#[test]
fn reserved_extension_types_are_skipped() {
    for extension_type in [0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xc, 0xf] {
        expect_same_samples(extension_type, 4, &[0xa5, 0x3c, 0xff, 0x00]);
    }
}

/// A payload of one byte is the `extension_type` nibble and four `other_bits`.
#[test]
fn a_reserved_extension_type_with_the_shortest_payload_is_skipped() {
    expect_same_samples(0xa, 1, &[0xff]);
}

/// The widest payload a four-bit `count` carries without the escape.
#[test]
fn a_reserved_extension_type_with_a_long_payload_is_skipped() {
    expect_same_samples(0xa, 14, &[0x12, 0x34, 0x56, 0x78, 0x9a]);
}

/// `EXT_DATA_ELEMENT` (0x2), whatever its `data_element_version`: version 0 is `ANC_DATA`, others have no layout.
#[test]
fn a_data_element_extension_is_skipped_for_any_version() {
    for version in 0..16u8 {
        expect_same_samples(0x2, 4, &[version << 4 | 0x5, 0x01, 0x02, 0x03]);
    }
}

/// `EXT_FILL_DATA` (0x1) "must be" a zero nibble and 0xa5 bytes; a stream that breaks it is not one to refuse.
#[test]
fn a_fill_data_extension_off_its_pattern_is_skipped() {
    expect_same_samples(0x1, 4, &[0x10, 0x00, 0x11, 0x22]);
    expect_same_samples(0x1, 4, &[0x0a, 0x5a, 0x5a, 0x5a]);
}

/// The pattern the standard asks of an encoder still decodes (no regression).
#[test]
fn a_fill_data_extension_on_its_pattern_is_skipped() {
    expect_same_samples(0x1, 4, &[0x0a, 0x5a, 0x5a, 0x5a]);
}

/// A reserved type with its payload cut short by the end of the block is still an error: the bits are not there.
#[test]
fn a_reserved_extension_payload_the_block_cannot_hold_is_refused() {
    let (extradata, units) = access_units();
    let mut decoder: Box<dyn Decoder> = make_decoder(&params(48_000, 2, extradata)).unwrap();
    // ID_FIL, count 14, extension_type 0xa, then only the byte-aligned end of the block.
    let truncated = vec![0b1101_1101, 0b0100_0000];
    let mut packet = Packet::new(0, TimeBase::new(1, 48_000), truncated);
    packet.pts = Some(0);
    let _ = units;
    let sent = decoder.send_packet(&packet);
    let got = sent.and_then(|()| decoder.receive_frame());
    assert!(got.is_err());
}

//! ADTS `number_of_raw_data_blocks_in_frame > 0` (multi-RDB) decode.
//!
//! ISO/IEC 13818-7 §6.2, Table 5: an `adts_frame()` may multiplex up to
//! four `raw_data_block()`s. The common case (and the only one most
//! encoders emit) is a single block, but the bitstream legitimately
//! allows 2..4 (e.g. "VLB " audio in NSV, some DTV multiplexers). These
//! tests synthesise multi-RDB frames by wrapping
//! several of our own encoder's single-RDB payloads and confirm the
//! decoder drains one `Frame::Audio` per block, byte-identical to decoding
//! the single-RDB frames individually.
//!
//! Both transport variants are covered:
//!   * `protection_absent == 1` (no CRC) — blocks back-to-back, located
//!     by post-END byte alignment (Tables 6 & 7 empty).
//!   * `protection_absent == 0` (CRC) — `adts_header_error_check()`
//!     carries `raw_data_block_position[i]` pointers + a header CRC, and
//!     each block is followed by a per-block CRC.

use oxideav_aac::adts::{parse_adts_header, ADTS_HEADER_NO_CRC};
use oxideav_core::bits::BitWriter;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame, Packet, TimeBase};

fn pcm_sine_mono(freq: f32, sr: u32, samples: usize, amp: f32) -> Vec<u8> {
    let mut out = Vec::with_capacity(samples * 2);
    for i in 0..samples {
        let t = i as f32 / sr as f32;
        let v = (2.0 * std::f32::consts::PI * freq * t).sin() * amp;
        let s = (v * 32767.0) as i16;
        out.extend_from_slice(&s.to_le_bytes());
    }
    out
}

/// Encode `pcm` (interleaved S16 mono) into a stream of ADTS frames.
fn encode_adts(pcm: Vec<u8>, sr: u32) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(1);
    params.bit_rate = Some(64_000);
    let mut enc = oxideav_aac::encoder::AacEncoder::new(&params).expect("make encoder");
    enc.set_enable_psy_model(false);
    let total_samples = pcm.len() / 2;
    let frame = Frame::Audio(AudioFrame {
        samples: total_samples as u32,
        pts: Some(0),
        data: vec![pcm],
    });
    enc.send_frame(&frame).expect("send_frame");
    enc.flush().expect("flush");
    let mut out = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        out.extend_from_slice(&p.data);
    }
    out
}

/// Split a stream of single-RDB ADTS frames into `(header, raw_data_block)`
/// byte slices.
fn split_frames(bytes: &[u8]) -> Vec<(Vec<u8>, Vec<u8>)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + ADTS_HEADER_NO_CRC <= bytes.len() {
        if bytes[i] != 0xFF || (bytes[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        let h = match parse_adts_header(&bytes[i..]) {
            Ok(h) => h,
            Err(_) => {
                i += 1;
                continue;
            }
        };
        if h.frame_length == 0 || i + h.frame_length > bytes.len() {
            break;
        }
        let hl = h.header_length();
        out.push((
            bytes[i..i + hl].to_vec(),
            bytes[i + hl..i + h.frame_length].to_vec(),
        ));
        i += h.frame_length;
    }
    out
}

/// Build a 7-byte (no-CRC) ADTS fixed+variable header for AAC-LC mono,
/// with the supplied `frame_length` and `n_blocks_minus_one`.
fn build_adts_header(sr_idx: u8, frame_length: usize, n_blocks_minus_one: u8) -> Vec<u8> {
    build_adts_header_inner(sr_idx, frame_length, n_blocks_minus_one, true)
}

fn build_adts_header_inner(
    sr_idx: u8,
    frame_length: usize,
    n_blocks_minus_one: u8,
    protection_absent: bool,
) -> Vec<u8> {
    let mut w = BitWriter::new();
    w.write_u32(0xFFF, 12); // syncword
    w.write_u32(0, 1); // id (MPEG-4)
    w.write_u32(0, 2); // layer
    w.write_u32(protection_absent as u32, 1); // protection_absent
    w.write_u32(1, 2); // profile (1 = LC, object_type - 1)
    w.write_u32(sr_idx as u32, 4); // sampling_freq_index
    w.write_u32(0, 1); // private_bit
    w.write_u32(1, 3); // channel_configuration (mono)
    w.write_u32(0, 1); // original_copy
    w.write_u32(0, 1); // home
    w.write_u32(0, 1); // copyright_id_bit
    w.write_u32(0, 1); // copyright_id_start
    w.write_u32(frame_length as u32, 13); // aac_frame_length
    w.write_u32(0x7FF, 11); // adts_buffer_fullness (VBR sentinel)
    w.write_u32(n_blocks_minus_one as u32, 2); // number_of_raw_data_blocks - 1
    w.finish()
}

/// Reference: feed each raw_data_block as its own single-RDB packet into
/// ONE decoder, collecting per-frame PCM. Sharing one decoder mirrors the
/// IMDCT overlap-add state evolution of the multi-RDB drain, so per-frame
/// PCM is directly comparable.
fn decode_blocks_sequential(blocks: &[&Vec<u8>], sr: u32) -> Vec<Vec<u8>> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(1);
    // Mark configured via a one-byte ASC so the raw (no-sync) path is taken.
    // AAC-LC mono ASC = audioObjectType 2, sf_idx, chan_cfg 1.
    params.extradata = asc_lc_mono(sr);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make dec");
    let tb = TimeBase::new(1, sr as i64);
    let mut out = Vec::new();
    for rdb in blocks {
        let pkt = Packet::new(0, tb, rdb.to_vec());
        dec.send_packet(&pkt).unwrap();
        match dec.receive_frame() {
            Ok(Frame::Audio(af)) => out.push(af.data[0].clone()),
            other => panic!("unexpected: {other:?}"),
        }
    }
    out
}

/// Minimal AudioSpecificConfig for AAC-LC mono at `sr`.
fn asc_lc_mono(sr: u32) -> Vec<u8> {
    let sr_idx = sr_index(sr);
    let mut w = BitWriter::new();
    w.write_u32(2, 5); // audioObjectType = AAC-LC
    w.write_u32(sr_idx as u32, 4); // samplingFrequencyIndex
    w.write_u32(1, 4); // channelConfiguration = mono
                       // GASpecificConfig: frameLengthFlag(0) dependsOnCoreCoder(0) extensionFlag(0)
    w.write_u32(0, 3);
    w.finish()
}

fn sr_index(sr: u32) -> u8 {
    const RATES: [u32; 13] = [
        96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350,
    ];
    RATES.iter().position(|&r| r == sr).unwrap() as u8
}

/// Decode a multi-RDB ADTS frame: send one packet, drain N+1 frames.
fn decode_multi(frame: &[u8], sr: u32, expected_blocks: usize) -> Vec<Vec<u8>> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(1);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make dec");
    let tb = TimeBase::new(1, sr as i64);
    let pkt = Packet::new(0, tb, frame.to_vec());
    dec.send_packet(&pkt).unwrap();
    let mut out = Vec::new();
    for _ in 0..expected_blocks {
        match dec.receive_frame() {
            Ok(Frame::Audio(af)) => out.push(af.data[0].clone()),
            other => panic!("expected {expected_blocks} frames, got error/short: {other:?}"),
        }
    }
    out
}

#[test]
fn multi_rdb_no_crc_two_blocks() {
    let sr = 44_100;
    let sr_idx = sr_index(sr);
    // Encode 3 frames; we only need the first two raw_data_blocks.
    let pcm = pcm_sine_mono(440.0, sr, 1024 * 3, 0.3);
    let stream = encode_adts(pcm, sr);
    let frames = split_frames(&stream);
    assert!(frames.len() >= 2, "need at least 2 encoded frames");
    let rdb0 = &frames[0].1;
    let rdb1 = &frames[1].1;

    // Reference: decode the two blocks sequentially through one decoder.
    let refs = decode_blocks_sequential(&[rdb0, rdb1], sr);

    // Build a single multi-RDB ADTS frame: header + rdb0 + rdb1, no CRC.
    let frame_length = ADTS_HEADER_NO_CRC + rdb0.len() + rdb1.len();
    let mut frame = build_adts_header(sr_idx, frame_length, 1);
    frame.extend_from_slice(rdb0);
    frame.extend_from_slice(rdb1);

    let got = decode_multi(&frame, sr, 2);
    assert_eq!(got.len(), 2);
    assert_eq!(
        got[0], refs[0],
        "block 0 PCM diverges from single-RDB decode"
    );
    assert_eq!(
        got[1], refs[1],
        "block 1 PCM diverges from single-RDB decode"
    );
}

#[test]
fn multi_rdb_no_crc_four_blocks() {
    let sr = 48_000;
    let sr_idx = sr_index(sr);
    let pcm = pcm_sine_mono(660.0, sr, 1024 * 5, 0.25);
    let stream = encode_adts(pcm, sr);
    let frames = split_frames(&stream);
    assert!(frames.len() >= 4, "need at least 4 encoded frames");
    let blocks: Vec<&Vec<u8>> = frames[..4].iter().map(|(_, r)| r).collect();

    let refs = decode_blocks_sequential(&blocks, sr);

    let total: usize = blocks.iter().map(|b| b.len()).sum();
    let frame_length = ADTS_HEADER_NO_CRC + total;
    let mut frame = build_adts_header(sr_idx, frame_length, 3); // 4 blocks
    for b in &blocks {
        frame.extend_from_slice(b);
    }

    let got = decode_multi(&frame, sr, 4);
    assert_eq!(got.len(), 4);
    for (i, (g, r)) in got.iter().zip(refs.iter()).enumerate() {
        assert_eq!(g, r, "block {i} PCM diverges from single-RDB decode");
    }
}

#[test]
fn multi_rdb_with_crc_three_blocks() {
    let sr = 44_100;
    let sr_idx = sr_index(sr);
    let pcm = pcm_sine_mono(880.0, sr, 1024 * 4, 0.2);
    let stream = encode_adts(pcm, sr);
    let frames = split_frames(&stream);
    assert!(frames.len() >= 3, "need at least 3 encoded frames");
    let blocks: Vec<&Vec<u8>> = frames[..3].iter().map(|(_, r)| r).collect();
    let n = blocks.len();
    let n_minus_one = (n - 1) as u8;

    let refs = decode_blocks_sequential(&blocks, sr);

    // CRC-present layout:
    //   [7-byte header]
    //   [adts_header_error_check: (n-1) * 16-bit positions + 16-bit crc]
    //   [rdb0][16-bit crc][rdb1][16-bit crc][rdb2][16-bit crc]
    // raw_data_block_position[i] = byte offset of rdb(i) from rdb0 start.
    // Compute byte positions (each block is followed by a 2-byte CRC).
    let mut positions = Vec::with_capacity(n - 1);
    let mut acc = 0usize;
    for (i, b) in blocks.iter().enumerate() {
        if i == 0 {
            acc += b.len() + 2; // rdb0 + its crc
        } else {
            positions.push(acc);
            acc += b.len() + 2;
        }
    }
    // Total frame length: header(7) + header_error_check(n*2) + sum(block+crc).
    let blocks_with_crc: usize = blocks.iter().map(|b| b.len() + 2).sum();
    let frame_length = 7 + n * 2 + blocks_with_crc;

    let mut frame = build_adts_header_inner(sr_idx, frame_length, n_minus_one, false);
    // adts_header_error_check: positions then header CRC (dummy 0).
    let mut ec = BitWriter::new();
    for &p in &positions {
        ec.write_u32(p as u32, 16);
    }
    ec.write_u32(0, 16); // header crc_check (not validated)
    frame.extend_from_slice(&ec.finish());
    // Blocks each followed by a 2-byte per-block CRC (dummy 0).
    for b in &blocks {
        frame.extend_from_slice(b);
        frame.extend_from_slice(&[0u8, 0u8]);
    }

    let got = decode_multi(&frame, sr, n);
    assert_eq!(got.len(), n);
    for (i, (g, r)) in got.iter().zip(refs.iter()).enumerate() {
        assert_eq!(g, r, "block {i} PCM diverges from single-RDB decode");
    }
}

#[test]
fn single_rdb_still_decodes() {
    // Regression: number_of_raw_data_blocks_in_frame == 0 path unchanged.
    let sr = 44_100;
    let pcm = pcm_sine_mono(440.0, sr, 1024 * 2, 0.3);
    let stream = encode_adts(pcm, sr);
    let frames = split_frames(&stream);
    assert!(!frames.is_empty());
    // Re-wrap frame 0 as a normal single-RDB ADTS frame and decode it.
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(1);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make dec");
    let tb = TimeBase::new(1, sr as i64);
    let mut hdr_and_block = frames[0].0.clone();
    hdr_and_block.extend_from_slice(&frames[0].1);
    let pkt = Packet::new(0, tb, hdr_and_block);
    dec.send_packet(&pkt).unwrap();
    assert!(matches!(dec.receive_frame(), Ok(Frame::Audio(_))));
}

//! ADTS `error_check()` CRC verification end to end — ISO/IEC
//! 13818-7:2004 §8.1.1 region selection + the ISO/IEC 11172-3
//! §2.4.3.1 CRC-16 (via 13818-7 §8.1.1.2), wired through
//! `StreamDecoder::decode_adts_frame` / `decode_all`.
//!
//! Streams are produced by the crate's own encoder (or the bit-exact
//! writers for the CPE / multi-RDB shapes), CRC-protected with
//! `adts_crc::protect_adts_stream`, and decoded: a protected stream
//! must decode to the exact PCM of its unprotected original, a
//! corrupted *covered* bit must surface `Error::AdtsCrcMismatch`, and
//! a corrupted *uncovered* bit (fill-element body, or an element bit
//! past the 192-bit protection window) must leave the CRC check
//! satisfied.

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::adts_crc::{
    adts_header_crc, adts_rdb_crc, adts_single_crc, collect_block_regions, protect_adts_stream,
};
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

const AOT_LC: u8 = 2;
const FS_INDEX: u8 = 4; // 44.1 kHz

fn tone_pcm(samples: usize) -> Vec<i16> {
    (0..samples)
        .map(|i| (7000.0 * (0.021 * i as f64).sin()) as i16)
        .collect()
}

fn encode_tone(frames: usize) -> Vec<u8> {
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44100,
        channels: 1,
        bitrate: 96_000,
    })
    .unwrap();
    enc.encode_all(&tone_pcm(1024 * frames)).unwrap()
}

fn long_ics_info(max_sfb: u8) -> IcsInfo {
    IcsInfo {
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb,
        scale_factor_grouping: None,
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: NUM_SWB_LONG_WINDOW[FS_INDEX as usize],
    }
}

/// A minimal long-window channel: codebook-1 sections over
/// `0..max_sfb`, flat scalefactors, constant quantized spectrum.
fn make_channel(max_sfb: u8, global_gain: u8) -> (IcsBody, SpectralData) {
    let sfb_cb = vec![vec![1u8; max_sfb as usize]];
    let sections = vec![vec![Section {
        codebook: 1,
        start: 0,
        end: max_sfb,
    }]];
    let entries: Vec<ScaleFactorEntry> = (0..max_sfb).map(|_| ScaleFactorEntry::Dpcm(0)).collect();
    let body = IcsBody {
        global_gain,
        ics_info: Some(long_ics_info(max_sfb)),
        section_data: SectionData { sections, sfb_cb },
        scale_factor_data: ScaleFactorData {
            entries: vec![entries],
        },
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    let offsets = oxideav_aac::swb_offset::long_window_offsets(FS_INDEX).unwrap();
    let active = usize::from(offsets[max_sfb as usize]);
    let mut coeffs = vec![0i32; 1024];
    coeffs[..active].fill(1);
    let spectral = SpectralData {
        x_quant: vec![coeffs],
    };
    (body, spectral)
}

fn push_sce(fa: &mut FrameAssembler, tag: u8, body: &IcsBody, spectral: &SpectralData) {
    fa.push_channel_header(IdSynEle::Sce, tag).unwrap();
    let mut bw = BitWriter::new();
    body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
    let ics = body.ics_info.as_ref().unwrap();
    spectral
        .write(&mut bw, ics, &body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
}

/// Wrap one raw_data_block payload in a `protection_absent == 1`
/// mono-config ADTS header.
fn wrap_adts(payload: &[u8], channel_configuration: u8) -> Vec<u8> {
    let header = AdtsHeader {
        mpeg_version_mpeg2: false,
        protection_absent: true,
        profile: AOT_LC - 1,
        sampling_frequency_index: FS_INDEX,
        channel_configuration,
        aac_frame_length: (7 + payload.len()) as u16,
        adts_buffer_fullness: 0x7FF,
        number_of_raw_data_blocks_in_frame: 1,
    };
    let mut out = header.write().unwrap().to_vec();
    out.extend_from_slice(payload);
    out
}

#[test]
fn protected_stream_decodes_identically() {
    let plain = encode_tone(8);
    let protected = protect_adts_stream(&plain).unwrap();
    assert!(protected.len() > plain.len());

    // Every protected frame signals the CRC and grew by 2 bytes.
    let (h, off) = AdtsHeader::parse(&protected).unwrap();
    assert!(!h.protection_absent);
    assert_eq!(off, 9);

    let base = StreamDecoder::new().decode_all(&plain).unwrap();
    let checked = StreamDecoder::new().decode_all(&protected).unwrap();
    assert_eq!(base.len(), checked.len());
    for (a, b) in base.iter().zip(checked.iter()) {
        assert_eq!(a.pcm, b.pcm);
        assert_eq!(a.sample_rate, b.sample_rate);
    }
    assert!(base.iter().any(|f| f.pcm.iter().any(|&s| s != 0)));
}

#[test]
fn corrupt_covered_element_bit_is_rejected() {
    let protected = protect_adts_stream(&encode_tone(3)).unwrap();
    // Payload byte 1 carries global_gain bits of the first channel
    // element — inside the §8.1.1.1 first-192-bit window, and
    // structurally inert (the element still parses).
    let mut bad = protected.clone();
    bad[9 + 1] ^= 0x10;
    assert!(matches!(
        StreamDecoder::new().decode_all(&bad),
        Err(Error::AdtsCrcMismatch)
    ));
}

#[test]
fn corrupt_crc_field_is_rejected() {
    let protected = protect_adts_stream(&encode_tone(3)).unwrap();
    let mut bad = protected.clone();
    bad[8] ^= 0x01; // low byte of crc_check
    assert!(matches!(
        StreamDecoder::new().decode_all(&bad),
        Err(Error::AdtsCrcMismatch)
    ));
}

#[test]
fn corrupt_header_bit_is_rejected() {
    let protected = protect_adts_stream(&encode_tone(3)).unwrap();
    // adts_buffer_fullness (a covered variable-header field) rides in
    // bytes 5..6; flipping a bit keeps the header structurally valid
    // but must break the CRC.
    let mut bad = protected.clone();
    bad[5] ^= 0x02;
    assert!(matches!(
        StreamDecoder::new().decode_all(&bad),
        Err(Error::AdtsCrcMismatch)
    ));
}

#[test]
fn fill_element_bits_are_not_covered() {
    // [SCE, FIL, END]: the FIL body (EXT_FILL other_bits) is outside
    // every §8.1.1.1 protected region — corrupting it must pass the
    // CRC check and decode to the same PCM.
    let (body, spectral) = make_channel(8, 150);
    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, 0, &body, &spectral);
    // EXT_FILL: type nibble 0000 + 8*(cnt-1)+4 other_bits. The body
    // starts 7 bits (3-bit id + 4-bit count, cnt < 15) after the
    // current assembler position.
    let fil_body_bit = fa.bit_position() + 7;
    fa.push_fill(&[0x0A, 0xAA, 0xAA, 0xAA, 0xAA]).unwrap();
    let payload = fa.push_end();
    let frame = wrap_adts(&payload, 1);
    let protected = protect_adts_stream(&frame).unwrap();

    let base = StreamDecoder::new().decode_all(&protected).unwrap();

    // Flip an other_bits bit (20 bits into the FIL body, past the
    // type nibble). Payload starts at frame byte 9 once protected.
    let bit = 9 * 8 + fil_body_bit + 20;
    let mut tweaked = protected.clone();
    tweaked[(bit / 8) as usize] ^= 0x80 >> (bit % 8);
    let redecoded = StreamDecoder::new().decode_all(&tweaked).unwrap();
    assert_eq!(base[0].pcm, redecoded[0].pcm);
}

#[test]
fn region_windows_cap_at_192_bits() {
    // CRC-computation level: flipping a payload bit inside the
    // element's first-192-bit window changes the checksum; flipping a
    // bit past both the 192-bit window and the second-ICS 128-bit
    // window leaves it unchanged.
    let (l, ls) = make_channel(20, 152); // long left ICS (> 192 bits)
    let (r, rs) = make_channel(16, 148); // right ICS longer than 136 bits
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Cpe, 0).unwrap();
    let mut bw = BitWriter::new();
    bw.write_bit(false); // common_window = 0
    for (body, spectral) in [(&l, &ls), (&r, &rs)] {
        body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
        let ics = body.ics_info.as_ref().unwrap();
        spectral
            .write(&mut bw, ics, &body.section_data, FS_INDEX)
            .unwrap();
    }
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    let payload = fa.push_end();
    let frame = wrap_adts(&payload, 2);

    let mut reader = BitReader::new(&payload);
    let regions = collect_block_regions(&mut reader, AOT_LC, FS_INDEX).unwrap();
    assert_eq!(
        regions.len(),
        2,
        "CPE yields the 192 + second-ICS-128 regions"
    );
    let cpe = regions[0];
    let ics2 = regions[1];
    assert_eq!(cpe.pad_to, Some(192));
    assert_eq!(ics2.pad_to, Some(128));
    assert!(
        ics2.start_bit > cpe.start_bit + 192,
        "left ICS must outrun the 192-bit window for this test"
    );

    let flip = |payload: &[u8], bit: u64| -> Vec<u8> {
        let mut p = payload.to_vec();
        p[(bit / 8) as usize] ^= 0x80 >> (bit % 8);
        p
    };
    let base = adts_single_crc(&frame[..7], &payload, &regions);
    // Inside the first-192 window.
    let in_window = flip(&payload, cpe.start_bit + 100);
    assert_ne!(adts_single_crc(&frame[..7], &in_window, &regions), base);
    // Inside the second-ICS 128-bit window.
    let in_ics2 = flip(&payload, ics2.start_bit + 40);
    assert_ne!(adts_single_crc(&frame[..7], &in_ics2, &regions), base);
    // Past the 192-bit window but before the second ICS: uncovered.
    let uncovered = flip(&payload, cpe.start_bit + 192 + 8);
    assert!(cpe.start_bit + 200 < ics2.start_bit);
    assert_eq!(adts_single_crc(&frame[..7], &uncovered, &regions), base);
    // Past the second ICS's 128-bit window: uncovered.
    let past_ics2 = flip(&payload, ics2.start_bit + 128 + 8);
    assert!(ics2.start_bit + 136 < ics2.end_bit);
    assert_eq!(adts_single_crc(&frame[..7], &past_ics2, &regions), base);
}

#[test]
fn cpe_second_ics_corruption_is_rejected_end_to_end() {
    // Stereo encoder output: the right channel's bits live in the
    // second-ICS region; corrupt its global_gain (structurally inert)
    // and expect the CRC gate to fire.
    let pcm: Vec<i16> = (0..1024 * 3 * 2)
        .map(|i| {
            let n = (i / 2) as f64;
            let ch = (i % 2) as f64;
            (6000.0 * (0.017 * n + 0.4 * ch).sin()) as i16
        })
        .collect();
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44100,
        channels: 2,
        bitrate: 128_000,
    })
    .unwrap();
    let plain = enc.encode_all(&pcm).unwrap();
    let protected = protect_adts_stream(&plain).unwrap();

    // Find the second frame's second-ICS region and flip a bit inside
    // its first 8 bits (the right channel's global_gain).
    let (h0, _) = AdtsHeader::parse(&protected).unwrap();
    let f1 = h0.aac_frame_length as usize;
    let (h1, off1) = AdtsHeader::parse(&protected[f1..]).unwrap();
    assert!(!h1.protection_absent);
    let payload = &protected[f1 + off1..f1 + h1.aac_frame_length as usize];
    let mut reader = BitReader::new(payload);
    let regions = collect_block_regions(&mut reader, AOT_LC, FS_INDEX).unwrap();
    let ics2 = regions
        .iter()
        .find(|r| r.pad_to == Some(128))
        .expect("stereo frame carries a CPE second-ICS region");
    let bit = ics2.start_bit + 3;
    let mut bad = protected.clone();
    bad[f1 + off1 + (bit / 8) as usize] ^= 0x80 >> (bit % 8);
    assert!(matches!(
        StreamDecoder::new().decode_all(&bad),
        Err(Error::AdtsCrcMismatch)
    ));
}

/// Hand-built two-raw-data-block frame with the Table 1.A.9 header
/// CRC (positions + headers) and one Table 1.A.10 CRC per block.
fn build_multi_rdb_frame() -> Vec<u8> {
    let (b0, s0) = make_channel(8, 150);
    let (b1, s1) = make_channel(10, 148);
    let blocks: Vec<Vec<u8>> = [(b0, s0), (b1, s1)]
        .iter()
        .map(|(b, s)| {
            let mut fa = FrameAssembler::new();
            push_sce(&mut fa, 0, b, s);
            fa.push_end()
        })
        .collect();

    // payload = block0 ++ crc0 ++ block1 ++ crc1 (each block ends
    // byte-aligned after END). CRCs are computed over the final
    // payload layout — the CRC slots themselves are outside every
    // region, so a two-pass assemble-then-patch is exact.
    let mut payload = Vec::new();
    let mut crc_slots = Vec::new();
    for block in &blocks {
        payload.extend_from_slice(block);
        crc_slots.push(payload.len());
        payload.extend_from_slice(&[0, 0]);
    }

    // raw_data_block_position: byte offset of block 1 within the
    // payload region (the one reading of the field verification does
    // not depend on — the value is covered, not interpreted).
    let positions = vec![(blocks[0].len() + 2) as u16];

    let header = AdtsHeader {
        mpeg_version_mpeg2: false,
        protection_absent: false,
        profile: AOT_LC - 1,
        sampling_frequency_index: FS_INDEX,
        channel_configuration: 1,
        aac_frame_length: (7 + 2 * positions.len() + 2 + payload.len()) as u16,
        adts_buffer_fullness: 0x7FF,
        number_of_raw_data_blocks_in_frame: 2,
    };
    let h = header.write().unwrap();

    // Per-block CRCs over the final payload bit positions. Region
    // collection first (the zeroed CRC slots are outside every
    // region, so parsing and the checksums are unaffected by the
    // later patch), then patch each slot.
    let per_block_regions: Vec<_> = {
        let mut reader = BitReader::new(&payload);
        crc_slots
            .iter()
            .map(|_| {
                let regions = collect_block_regions(&mut reader, AOT_LC, FS_INDEX).unwrap();
                reader.read_u32(16).unwrap(); // skip the CRC slot
                regions
            })
            .collect()
    };
    for (slot, regions) in crc_slots.iter().zip(&per_block_regions) {
        let crc = adts_rdb_crc(&payload, regions);
        payload[*slot..slot + 2].copy_from_slice(&crc.to_be_bytes());
    }

    let mut frame = h.to_vec();
    for p in &positions {
        frame.extend_from_slice(&p.to_be_bytes());
    }
    frame.extend_from_slice(&adts_header_crc(&h, &positions).to_be_bytes());
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn multi_rdb_frame_verifies_and_decodes() {
    let frame = build_multi_rdb_frame();
    let decoded = StreamDecoder::new().decode_adts_frame(&frame).unwrap();
    // Two mono raw data blocks = two consecutive 1024-sample hops of
    // the same single-channel program.
    assert_eq!(decoded.channels, 1);
    assert_eq!(decoded.pcm.len(), 2 * 1024);
    assert!(decoded.pcm.iter().any(|&s| s != 0));
}

#[test]
fn multi_rdb_corrupt_position_is_rejected() {
    let mut frame = build_multi_rdb_frame();
    frame[7] ^= 0x01; // raw_data_block_position high byte
    assert!(matches!(
        StreamDecoder::new().decode_adts_frame(&frame),
        Err(Error::AdtsCrcMismatch)
    ));
}

/// Deterministic mutation battery: the CRC-frame walk (region
/// collection parses untrusted element bodies *before* the checksum
/// comparison) must never panic — every mutated input decodes or
/// errors cleanly.
#[test]
fn mutated_protected_frames_never_panic() {
    let single = protect_adts_stream(&encode_tone(2)).unwrap();
    let multi = build_multi_rdb_frame();
    let mut state = 0x9E37_79B9u32;
    let mut rng = move || {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        state
    };
    for base in [&single[..], &multi[..]] {
        for _ in 0..400 {
            let mut bad = base.to_vec();
            // 1..=3 byte mutations anywhere in the frame.
            for _ in 0..=(rng() % 3) {
                let idx = (rng() as usize) % bad.len();
                bad[idx] ^= (rng() % 255 + 1) as u8;
            }
            let _ = StreamDecoder::new().decode_adts_frame(&bad);
            let _ = StreamDecoder::new().decode_all(&bad);
        }
        // Truncations at every prefix length.
        for len in 0..base.len() {
            let _ = StreamDecoder::new().decode_adts_frame(&base[..len]);
        }
    }
}

#[test]
fn multi_rdb_corrupt_second_block_is_rejected() {
    let frame = build_multi_rdb_frame();
    // Find the second block's global_gain: walk block 0 + its CRC.
    let payload_start = 7 + 2 + 2;
    let payload = &frame[payload_start..];
    let mut reader = BitReader::new(payload);
    collect_block_regions(&mut reader, AOT_LC, FS_INDEX).unwrap();
    reader.read_u32(16).unwrap();
    let block1_start = (reader.bit_position() / 8) as usize;
    let mut bad = frame.clone();
    // Bit 8 of block 1 (inside global_gain, after id + tag + 1).
    bad[payload_start + block1_start + 1] ^= 0x20;
    assert!(matches!(
        StreamDecoder::new().decode_adts_frame(&bad),
        Err(Error::AdtsCrcMismatch)
    ));
}

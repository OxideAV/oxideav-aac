//! EPAudioSyncStream / EPMuxElement transport (§1.7.2.1 Table 1.37 /
//! §1.7.3.1 Table 1.40): a writer-assembled error-protected LOAS
//! stream — BCH(36,18) sync headers, majority-protected flags,
//! Golay-protected config length, Table 1.59-protected
//! ErrorProtectionSpecificConfig, and the ep_frame()-coded
//! AudioMuxElement — decodes to PCM byte-identical to the equivalent
//! plain LOAS stream, and survives correctable channel errors.

use oxideav_aac::ep_config::{EpClass, EpPredefinedSet, ErrorProtectionSpecificConfig};
use oxideav_aac::ep_frame::{EpFrameCodec, EpFrameData};
use oxideav_aac::latm::{ep_sync_header_parity, LoasDecoder};
use oxideav_core::bits::{BitReader, BitWriter};

/// AAC-LC 44.1 kHz mono ASC (16 bits).
fn write_aac_lc_asc(w: &mut BitWriter) {
    w.write_u32(2, 5); // AOT LC
    w.write_u32(4, 4); // fsIndex
    w.write_u32(1, 4); // chanConfig
    w.write_bit(false);
    w.write_bit(false);
    w.write_bit(false);
}

/// A tiny mono SCE raw_data_block with small book-1 spectra.
fn tiny_raw_data_block() -> Vec<u8> {
    use oxideav_aac::ics_body::IcsBody;
    use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
    use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
    use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
    use oxideav_aac::section_data::{Section, SectionData};
    use oxideav_aac::spectral_data::SpectralData;
    let ics = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 4,
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
        num_swb: NUM_SWB_LONG_WINDOW[4],
    };
    let offsets = oxideav_aac::swb_offset::long_window_offsets(4).unwrap();
    let mut coeffs = vec![0i32; 1024];
    let mut seed = 0x1357u32;
    for sfb in 0..4usize {
        let (a, b) = (usize::from(offsets[sfb]), usize::from(offsets[sfb + 1]));
        for c in coeffs[a..b].iter_mut() {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            *c = ((seed >> 8) % 3) as i32 - 1;
        }
    }
    let body = IcsBody {
        global_gain: 140,
        ics_info: Some(ics.clone()),
        section_data: SectionData {
            sections: vec![(0..4)
                .map(|sfb| Section {
                    codebook: 1,
                    start: sfb as u8,
                    end: sfb as u8 + 1,
                })
                .collect()],
            sfb_cb: vec![vec![1, 1, 1, 1]],
        },
        scale_factor_data: ScaleFactorData {
            entries: vec![vec![ScaleFactorEntry::Dpcm(0); 4]],
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
    let spectral = SpectralData {
        x_quant: vec![coeffs],
    };
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let mut bw = BitWriter::new();
    body.write(&mut bw, 2, 4, false).unwrap();
    spectral
        .write(&mut bw, &ics, &body.section_data, 4)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    fa.push_end()
}

/// The plain AudioMuxElement(1) bytes for one payload (first frame
/// carries the StreamMuxConfig).
fn audio_mux_element(payload: &[u8], first: bool) -> Vec<u8> {
    let mut w = BitWriter::new();
    if first {
        w.write_bit(false); // useSameStreamMux = 0
        w.write_bit(false); // audioMuxVersion
        w.write_bit(true); // allStreamsSameTimeFraming
        w.write_u32(0, 6); // numSubFrames
        w.write_u32(0, 4); // numProgram
        w.write_u32(0, 3); // numLayer
        write_aac_lc_asc(&mut w);
        w.write_u32(0, 3); // frameLengthType
        w.write_u32(0xFF, 8); // latmBufferFullness
        w.write_bit(false); // otherDataPresent
        w.write_bit(false); // crcCheckPresent
    } else {
        w.write_bit(true); // useSameStreamMux
    }
    let mut rem = payload.len();
    while rem >= 255 {
        w.write_u32(255, 8);
        rem -= 255;
    }
    w.write_u32(rem as u32, 8);
    for &b in payload {
        w.write_u32(u32::from(b), 8);
    }
    w.finish()
}

/// One until-the-end SRCPC class (rate 8/16, CRC16) protecting the
/// whole AudioMuxElement.
fn ep_config() -> ErrorProtectionSpecificConfig {
    ErrorProtectionSpecificConfig {
        interleave_type: 0,
        bit_stuffing: 1,
        number_of_concatenated_frame: 1,
        sets: vec![EpPredefinedSet {
            classes: vec![EpClass {
                length_escape: true,
                rate_escape: false,
                crclen_escape: false,
                concatenate_flag: false,
                fec_type: 0,
                termination_switch: Some(true),
                interleave_switch: None,
                class_optional: false,
                number_of_bits_for_length: Some(0), // until the end
                class_length: None,
                class_rate: Some(8), // 8/16
                class_crclen: Some(16),
            }],
            class_reordered_output: false,
            class_output_order: Vec::new(),
        }],
        header_protection: false,
        header_rate: None,
        header_crclen: None,
    }
}

/// Assemble one EPMuxElement(1,1) around an AudioMuxElement.
fn ep_mux_element(au: &[u8], first: bool, codec: &EpFrameCodec) -> Vec<u8> {
    let mut w = BitWriter::new();
    // epUsePreviousMuxConfig + 2-bit repetition.
    let use_prev = !first;
    w.write_bit(use_prev);
    w.write_bit(use_prev);
    w.write_bit(use_prev);
    if first {
        // ErrorProtectionSpecificConfig bits.
        let mut cw = BitWriter::new();
        ep_config().write(&mut cw).unwrap();
        let cfg_bit_len = cw.bit_position() as usize;
        let cfg_bytes = cw.finish();
        let cfg_bits: Vec<bool> = (0..cfg_bit_len)
            .map(|i| cfg_bytes[i / 8] & (0x80 >> (i % 8)) != 0)
            .collect();
        // epSpecificConfigLength (10, in bits) + Golay(23,12) parity.
        let len_bits: Vec<bool> = (0..10)
            .rev()
            .map(|i| cfg_bit_len & (1usize << i) != 0)
            .collect();
        for &b in &len_bits {
            w.write_bit(b);
        }
        for b in oxideav_aac::ep_fec::header_fec_encode(&len_bits).unwrap() {
            w.write_bit(b);
        }
        for &b in &cfg_bits {
            w.write_bit(b);
        }
        for b in oxideav_aac::ep_fec::header_fec_encode(&cfg_bits).unwrap() {
            w.write_bit(b);
        }
    }
    // ByteAlign().
    let pos = w.bit_position();
    for _ in 0..((8 - (pos % 8)) % 8) {
        w.write_bit(false);
    }
    let mut out = w.finish();
    // EPAudioMuxElement = ep_frame over the AudioMuxElement bits.
    let au_bits: Vec<bool> = (0..au.len() * 8)
        .map(|i| au[i / 8] & (0x80 >> (i % 8)) != 0)
        .collect();
    let frame = EpFrameData {
        choice_of_pred: 0,
        classes: vec![au_bits],
        rate_codes: vec![None],
        crc_codes: vec![None],
    };
    out.extend_from_slice(&codec.encode(&frame).unwrap());
    out
}

/// Wrap EPMuxElements into an EPAudioSyncStream.
fn ep_sync_stream(elements: &[Vec<u8>]) -> Vec<u8> {
    let mut out = Vec::new();
    for (i, el) in elements.iter().enumerate() {
        let mut w = BitWriter::new();
        w.write_u32(0x4DE1, 16);
        w.write_u32(0, 4); // futureUse
        w.write_u32(el.len() as u32, 13);
        w.write_u32((i % 32) as u32, 5); // frameCounter
        w.write_u32(ep_sync_header_parity(el.len() as u16, (i % 32) as u8), 18);
        out.extend_from_slice(&w.finish());
        out.extend_from_slice(el);
    }
    out
}

/// The equivalent plain LOAS stream.
fn plain_loas(aus: &[Vec<u8>]) -> Vec<u8> {
    let mut out = Vec::new();
    for au in aus {
        let mut fw = BitWriter::new();
        fw.write_u32(0x2B7, 11);
        fw.write_u32(au.len() as u32, 13);
        out.extend_from_slice(&fw.finish());
        out.extend_from_slice(au);
    }
    out
}

#[test]
fn ep_transport_matches_plain_loas() {
    let rdb = tiny_raw_data_block();
    let aus: Vec<Vec<u8>> = (0..3).map(|i| audio_mux_element(&rdb, i == 0)).collect();
    let codec = EpFrameCodec::new(ep_config()).unwrap();
    let elements: Vec<Vec<u8>> = aus
        .iter()
        .enumerate()
        .map(|(i, au)| ep_mux_element(au, i == 0, &codec))
        .collect();
    let ep_stream = ep_sync_stream(&elements);
    let plain = plain_loas(&aus);

    let mut ep_dec = LoasDecoder::new();
    let ep_frames = ep_dec.decode_all_ep(&ep_stream).unwrap();
    let mut plain_dec = LoasDecoder::new();
    let plain_frames = plain_dec.decode_all(&plain).unwrap();
    assert_eq!(ep_frames.len(), 3);
    assert_eq!(ep_frames.len(), plain_frames.len());
    for (i, (a, b)) in ep_frames.iter().zip(&plain_frames).enumerate() {
        assert_eq!(a.pcm, b.pcm, "frame {i}");
        assert_eq!(a.channels, b.channels, "frame {i}");
        assert!(a.pcm.iter().any(|&s| s != 0), "frame {i} silent");
    }

    // Docs-corpus fixture: the deterministic EP-protected LOAS
    // stream plus this crate's own decode.
    let mut pcm = Vec::new();
    for f in &ep_frames {
        pcm.extend_from_slice(&f.pcm);
    }
    stage_or_pin(
        "aac-ep-writer-loas",
        "input.eploas",
        &ep_stream,
        &pcm,
        1,
        44_100,
    );
}

#[test]
fn ep_transport_corrects_channel_errors() {
    let rdb = tiny_raw_data_block();
    let aus: Vec<Vec<u8>> = (0..2).map(|i| audio_mux_element(&rdb, i == 0)).collect();
    let codec = EpFrameCodec::new(ep_config()).unwrap();
    let elements: Vec<Vec<u8>> = aus
        .iter()
        .enumerate()
        .map(|(i, au)| ep_mux_element(au, i == 0, &codec))
        .collect();
    let mut ep_stream = ep_sync_stream(&elements);

    // Flip a bit inside the second element's payload region (the
    // SRCPC 8/16 corrects it).
    let second_start = 7 + elements[0].len() + 7; // header + el0 + header
    ep_stream[second_start + elements[1].len() / 2] ^= 0x08;

    let mut ep_dec = LoasDecoder::new();
    let ep_frames = ep_dec.decode_all_ep(&ep_stream).unwrap();
    let mut plain_dec = LoasDecoder::new();
    let plain_frames = plain_dec.decode_all(&plain_loas(&aus)).unwrap();
    for (a, b) in ep_frames.iter().zip(&plain_frames) {
        assert_eq!(a.pcm, b.pcm);
    }
}

#[test]
fn ep_sync_header_parity_detects_corruption() {
    let rdb = tiny_raw_data_block();
    let au = audio_mux_element(&rdb, true);
    let codec = EpFrameCodec::new(ep_config()).unwrap();
    let el = ep_mux_element(&au, true, &codec);
    let mut stream = ep_sync_stream(&[el]);
    // Corrupt audioMuxLengthBytes (byte 2-3 carry it).
    stream[3] ^= 0x40;
    let mut dec = LoasDecoder::new();
    assert!(dec.decode_all_ep(&stream).is_err());
}

#[test]
fn ep_mux_battery_never_panics() {
    let rdb = tiny_raw_data_block();
    let au = audio_mux_element(&rdb, true);
    let codec = EpFrameCodec::new(ep_config()).unwrap();
    let el = ep_mux_element(&au, true, &codec);
    let stream = ep_sync_stream(&[el]);
    for bit in (0..stream.len() * 8).step_by(3) {
        let mut mutated = stream.clone();
        mutated[bit / 8] ^= 0x80 >> (bit % 8);
        let mut dec = LoasDecoder::new();
        let _ = dec.decode_all_ep(&mutated);
    }
    for keep in 0..stream.len() {
        let mut dec = LoasDecoder::new();
        let _ = dec.decode_all_ep(&stream[..keep]);
    }
    // BitReader sanity: the pristine stream still decodes.
    let mut dec = LoasDecoder::new();
    assert_eq!(dec.decode_all_ep(&stream).unwrap().len(), 1);
    let _ = BitReader::new(&stream);
}

// ==================================================================
// Docs-corpus staging (mirrors docs_family_fixtures.rs)
// ==================================================================

fn fixtures_root() -> std::path::PathBuf {
    std::path::PathBuf::from("../../docs/audio/aac/fixtures")
}

fn stage_enabled() -> bool {
    std::env::var_os("OXIDEAV_AAC_STAGE_FIXTURES").is_some()
}

fn wav_bytes(pcm: &[i16], channels: u16, sample_rate: u32) -> Vec<u8> {
    let data_len = (pcm.len() * 2) as u32;
    let byte_rate = sample_rate * u32::from(channels) * 2;
    let block_align = channels * 2;
    let mut out = Vec::with_capacity(44 + pcm.len() * 2);
    out.extend_from_slice(b"RIFF");
    out.extend_from_slice(&(36 + data_len).to_le_bytes());
    out.extend_from_slice(b"WAVEfmt ");
    out.extend_from_slice(&16u32.to_le_bytes());
    out.extend_from_slice(&1u16.to_le_bytes());
    out.extend_from_slice(&channels.to_le_bytes());
    out.extend_from_slice(&sample_rate.to_le_bytes());
    out.extend_from_slice(&byte_rate.to_le_bytes());
    out.extend_from_slice(&block_align.to_le_bytes());
    out.extend_from_slice(&16u16.to_le_bytes());
    out.extend_from_slice(b"data");
    out.extend_from_slice(&data_len.to_le_bytes());
    for &s in pcm {
        out.extend_from_slice(&s.to_le_bytes());
    }
    out
}

fn stage_or_pin(name: &str, input_name: &str, input: &[u8], pcm: &[i16], ch: u16, rate: u32) {
    use std::fs;
    let dir = fixtures_root().join(name);
    let input_path = dir.join(input_name);
    let wav_path = dir.join("expected.wav");
    let wav = wav_bytes(pcm, ch, rate);
    if stage_enabled() {
        fs::create_dir_all(&dir).unwrap();
        fs::write(&input_path, input).unwrap();
        fs::write(&wav_path, &wav).unwrap();
        eprintln!("staged {}", dir.display());
        return;
    }
    match (fs::read(&input_path), fs::read(&wav_path)) {
        (Ok(staged_in), Ok(staged_wav)) => {
            assert_eq!(staged_in, input, "{name}: staged {input_name} drifted");
            assert_eq!(staged_wav, wav, "{name}: staged expected.wav drifted");
        }
        _ => eprintln!("skip: {} not staged yet", dir.display()),
    }
}

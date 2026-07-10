//! End-to-end ER AAC LC decode — ISO/IEC 14496-3 §4.4.2.3 Table 4.19
//! `er_raw_data_block()` through the stream drivers.
//!
//! Builds ER payloads with the crate's own writers (the ER
//! section-data branch, the HCR `reordered_spectral_data()` encoder)
//! and pins the ER decode against the equivalent non-resilient
//! `raw_data_block()` decode of the *same* spectra: the resilience
//! tools change the wire coding, not the reconstruction, so the PCM
//! must be bit-identical.

use oxideav_aac::asc::AacResilienceFlags;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::hcr_decode::encode_reordered_spectral_data;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::latm::LoasDecoder;
use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_core::bits::BitWriter;

const FS_INDEX: u8 = 4; // 44.1 kHz
const SAMPLE_RATE: u32 = 44100;
const AOT_LC: u8 = 2;
const AOT_ER_LC: u8 = 17;

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

/// A single-group long-window channel over `sfb_cbs` (one section per
/// band) with bounded pseudo-random spectrum.
fn make_channel(sfb_cbs: &[u8], global_gain: u8, seed: u32) -> (IcsBody, SpectralData) {
    let max_sfb = sfb_cbs.len() as u8;
    let sections = vec![sfb_cbs
        .iter()
        .enumerate()
        .map(|(sfb, &cb)| Section {
            codebook: cb,
            start: sfb as u8,
            end: sfb as u8 + 1,
        })
        .collect()];
    let entries: Vec<ScaleFactorEntry> = sfb_cbs
        .iter()
        .filter(|&&cb| cb != 0)
        .map(|_| ScaleFactorEntry::Dpcm(0))
        .collect();
    let body = IcsBody {
        global_gain,
        ics_info: Some(long_ics_info(max_sfb)),
        section_data: SectionData {
            sections,
            sfb_cb: vec![sfb_cbs.to_vec()],
        },
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
    let mut coeffs = vec![0i32; 1024];
    let mut state = seed;
    let mut prand = |max: i32| {
        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        ((state >> 8) % (2 * max + 1) as u32) as i32 - max
    };
    for (sfb, &cb) in sfb_cbs.iter().enumerate() {
        let max = match cb {
            0 | 13 | 14 | 15 => 0,
            1 | 2 => 1,
            3 | 4 => 2,
            5 | 6 => 4,
            7 | 8 => 7,
            9 | 10 => 12,
            _ => 30, // book 11 / virtual codebooks: exercise escapes
        };
        if max > 0 {
            let (a, b) = (usize::from(offsets[sfb]), usize::from(offsets[sfb + 1]));
            for c in coeffs[a..b].iter_mut() {
                *c = prand(max);
            }
        }
    }
    let spectral = SpectralData {
        x_quant: vec![coeffs],
    };
    (body, spectral)
}

/// Serialize one ER SCE (tag + ER body + spectrum) into `bw`.
///
/// `resilience` selects the wire branches: the ER section form
/// (`SectionData::write_er`), and the HCR lengths + reordered payload
/// instead of `spectral_data()`.
fn write_er_sce(
    bw: &mut BitWriter,
    tag: u8,
    body: &IcsBody,
    spectral: &SpectralData,
    resilience: AacResilienceFlags,
) {
    bw.write_u32(u32::from(tag), 4);
    let ics = body.ics_info.as_ref().unwrap();
    bw.write_u32(u32::from(body.global_gain), 8);
    ics.write(bw, AOT_ER_LC, FS_INDEX, false).unwrap();
    if resilience.section_data {
        body.section_data
            .write_er(bw, ics.window_sequence, ics.max_sfb)
            .unwrap();
    } else {
        body.section_data
            .write(bw, ics.window_sequence, ics.max_sfb)
            .unwrap();
    }
    assert!(
        !resilience.scalefactor_data,
        "test helper writes the non-RVLC scalefactor branch"
    );
    body.scale_factor_data
        .write(bw, &body.section_data.sfb_cb)
        .unwrap();
    // pulse_data_present / tns_data_present / gain_control_data_present.
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    if resilience.spectral_data {
        let (payload, len_bits, longest) =
            encode_reordered_spectral_data(spectral, ics, &body.section_data, FS_INDEX).unwrap();
        bw.write_u32(u32::from(len_bits), 14);
        bw.write_u32(u32::from(longest), 6);
        for i in 0..usize::from(len_bits) {
            bw.write_bit(payload[i / 8] & (0x80 >> (i % 8)) != 0);
        }
    } else {
        spectral
            .write(bw, ics, &body.section_data, FS_INDEX)
            .unwrap();
    }
}

/// The equivalent non-ER `raw_data_block()` ([SCE, END]).
fn non_er_block(body: &IcsBody, spectral: &SpectralData) -> Vec<u8> {
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let mut bw = BitWriter::new();
    body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
    let ics = body.ics_info.as_ref().unwrap();
    spectral
        .write(&mut bw, ics, &body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    fa.push_end()
}

/// HCR-coded spectrum through the ER driver reconstructs the exact
/// same PCM as the plain `spectral_data()` coding through the non-ER
/// driver — over several frames so the per-slot filterbank state
/// threads identically.
#[test]
fn er_spectral_resilience_matches_plain_decode() {
    let (body, spectral) = make_channel(&[1, 3, 5, 7, 9, 11, 11, 2], 150, 0xE11);
    let resilience = AacResilienceFlags {
        section_data: false,
        scalefactor_data: false,
        spectral_data: true,
    };
    let mut bw = BitWriter::new();
    write_er_sce(&mut bw, 0, &body, &spectral, resilience);
    let er_payload = bw.finish();
    let plain = non_er_block(&body, &spectral);

    let mut er_dec = StreamDecoder::new();
    let mut plain_dec = StreamDecoder::new();
    for frame in 0..3 {
        let er = er_dec
            .decode_er_raw_data_block(AOT_ER_LC, FS_INDEX, SAMPLE_RATE, 1, resilience, &er_payload)
            .unwrap();
        let base = plain_dec
            .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 1, 1, &plain)
            .unwrap();
        assert_eq!(er.channels, 1);
        assert_eq!(er.pcm.len(), 1024);
        assert_eq!(er.pcm, base.pcm, "frame {frame}");
        assert!(er.pcm.iter().any(|&s| s != 0), "frame {frame} silent");
    }
}

/// The ER section-data branch with a §4.6.16.4 virtual codebook plus
/// HCR: a VCB band decodes exactly like the same spectrum under the
/// plain ESC book 11.
#[test]
fn er_section_resilience_with_virtual_codebook() {
    // ER stream: band 4 coded with VCB 18; reference: same band as 11.
    let (er_body, spectral) = make_channel(&[1, 3, 5, 7, 18, 2], 150, 0x5EC7);
    let (mut ref_body, _) = make_channel(&[1, 3, 5, 7, 11, 2], 150, 0x5EC7);
    ref_body.global_gain = er_body.global_gain;
    let resilience = AacResilienceFlags {
        section_data: true,
        scalefactor_data: false,
        spectral_data: true,
    };
    let mut bw = BitWriter::new();
    write_er_sce(&mut bw, 0, &er_body, &spectral, resilience);
    let er_payload = bw.finish();
    let plain = non_er_block(&ref_body, &spectral);

    let mut er_dec = StreamDecoder::new();
    let mut plain_dec = StreamDecoder::new();
    for frame in 0..2 {
        let er = er_dec
            .decode_er_raw_data_block(AOT_ER_LC, FS_INDEX, SAMPLE_RATE, 1, resilience, &er_payload)
            .unwrap();
        let base = plain_dec
            .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 1, 1, &plain)
            .unwrap();
        assert_eq!(er.pcm, base.pcm, "frame {frame}");
    }
}

/// A stereo ER CPE (`common_window == 1`, config 2) with HCR spectra
/// matches the equivalent non-ER shared-window CPE decode.
#[test]
fn er_cpe_common_window_matches_plain_decode() {
    let (left_body, left_spec) = make_channel(&[1, 3, 5, 7, 9, 11], 148, 0xA);
    let (right_body, right_spec) = make_channel(&[1, 3, 5, 7, 9, 11], 152, 0xB);
    let ics = left_body.ics_info.clone().unwrap();
    let resilience = AacResilienceFlags {
        section_data: false,
        scalefactor_data: false,
        spectral_data: true,
    };

    // ER payload: tag, common_window=1, shared ics, ms_mask=00, then
    // per channel: ER body (no inline ics) + HCR lengths + payload.
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_bit(true); // common_window
    ics.write(&mut bw, AOT_ER_LC, FS_INDEX, true).unwrap();
    bw.write_u32(0, 2); // ms_mask_present = 00
    for (body, spectral) in [(&left_body, &left_spec), (&right_body, &right_spec)] {
        bw.write_u32(u32::from(body.global_gain), 8);
        body.section_data
            .write(&mut bw, ics.window_sequence, ics.max_sfb)
            .unwrap();
        body.scale_factor_data
            .write(&mut bw, &body.section_data.sfb_cb)
            .unwrap();
        bw.write_bit(false); // pulse
        bw.write_bit(false); // tns
        bw.write_bit(false); // gain control
        let (payload, len_bits, longest) =
            encode_reordered_spectral_data(spectral, &ics, &body.section_data, FS_INDEX).unwrap();
        bw.write_u32(u32::from(len_bits), 14);
        bw.write_u32(u32::from(longest), 6);
        for i in 0..usize::from(len_bits) {
            bw.write_bit(payload[i / 8] & (0x80 >> (i % 8)) != 0);
        }
    }
    let er_payload = bw.finish();

    // Non-ER reference: shared-window CPE in a raw_data_block.
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Cpe, 0).unwrap();
    let mut bw = BitWriter::new();
    bw.write_bit(true); // common_window
    ics.write(&mut bw, AOT_LC, FS_INDEX, true).unwrap();
    bw.write_u32(0, 2); // ms_mask_present = 00
    for (body, spectral) in [(&left_body, &left_spec), (&right_body, &right_spec)] {
        body.write_with_ics_info(&mut bw, &ics, AOT_LC, false)
            .unwrap();
        spectral
            .write(&mut bw, &ics, &body.section_data, FS_INDEX)
            .unwrap();
    }
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    let plain = fa.push_end();

    let mut er_dec = StreamDecoder::new();
    let mut plain_dec = StreamDecoder::new();
    for frame in 0..2 {
        let er = er_dec
            .decode_er_raw_data_block(AOT_ER_LC, FS_INDEX, SAMPLE_RATE, 2, resilience, &er_payload)
            .unwrap();
        let base = plain_dec
            .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 2, 1, &plain)
            .unwrap();
        assert_eq!(er.channels, 2);
        assert_eq!(er.pcm, base.pcm, "frame {frame}");
    }
}

/// LOAS/LATM end to end: an AOT-17 ASC with the spectral resilience
/// flag routes the payload through `er_raw_data_block()` and decodes
/// identically to the direct ER entry point.
#[test]
fn loas_er_stream_decodes() {
    let (body, spectral) = make_channel(&[1, 3, 5, 7, 9, 11], 150, 0x10A5);
    let resilience = AacResilienceFlags {
        section_data: false,
        scalefactor_data: false,
        spectral_data: true,
    };
    let mut bw = BitWriter::new();
    write_er_sce(&mut bw, 0, &body, &spectral, resilience);
    let er_payload = bw.finish();

    // ER AAC LC mono 44.1 kHz ASC: AOT 17, fsIdx 4, chanConfig 1;
    // GASpecificConfig frameLengthFlag=0 dependsOnCoreCoder=0
    // extensionFlag=1; AOT-17 resilience triplet (0,0,1);
    // extensionFlag3=0; epConfig=0.
    let write_asc = |w: &mut BitWriter| {
        w.write_u32(17, 5);
        w.write_u32(u32::from(FS_INDEX), 4);
        w.write_u32(1, 4);
        w.write_bit(false); // frameLengthFlag
        w.write_bit(false); // dependsOnCoreCoder
        w.write_bit(true); // extensionFlag
        w.write_bit(false); // aacSectionDataResilienceFlag
        w.write_bit(false); // aacScalefactorDataResilienceFlag
        w.write_bit(true); // aacSpectralDataResilienceFlag
        w.write_bit(false); // extensionFlag3
        w.write_u32(0, 2); // epConfig
    };

    // Minimal LOAS wrap: one sync frame with an inline StreamMuxConfig
    // (audioMuxVersion 0, one program / layer, frameLengthType 0).
    let mut w = BitWriter::new();
    w.write_bit(false); // useSameStreamMux
    w.write_bit(false); // audioMuxVersion = 0
    w.write_bit(true); // allStreamsSameTimeFraming
    w.write_u32(0, 6); // numSubFrames
    w.write_u32(0, 4); // numProgram
    w.write_u32(0, 3); // numLayer
    write_asc(&mut w);
    w.write_u32(0, 3); // frameLengthType = 0
    w.write_u32(0xFF, 8); // latmBufferFullness
    w.write_bit(false); // otherDataPresent
    w.write_bit(false); // crcCheckPresent
                        // PayloadLengthInfo: MuxSlotLengthBytes with 255-escapes.
    let mut rem = er_payload.len();
    while rem >= 255 {
        w.write_u32(255, 8);
        rem -= 255;
    }
    w.write_u32(rem as u32, 8);
    for &b in &er_payload {
        w.write_u32(u32::from(b), 8);
    }
    let element = w.finish();
    let mut fw = BitWriter::new();
    fw.write_u32(0x2B7, 11);
    fw.write_u32(element.len() as u32, 13);
    for &b in &element {
        fw.write_u32(u32::from(b), 8);
    }
    let loas = fw.finish();

    let mut loas_dec = LoasDecoder::new();
    let frames = loas_dec.decode_all(&loas).unwrap();
    assert_eq!(frames.len(), 1);
    let mut direct = StreamDecoder::new();
    let expect = direct
        .decode_er_raw_data_block(AOT_ER_LC, FS_INDEX, SAMPLE_RATE, 1, resilience, &er_payload)
        .unwrap();
    assert_eq!(frames[0].pcm, expect.pcm);
    assert_eq!(frames[0].sample_rate, SAMPLE_RATE);
    assert!(frames[0].pcm.iter().any(|&s| s != 0));
}

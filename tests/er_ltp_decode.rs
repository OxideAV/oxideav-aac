//! End-to-end ER AAC LTP (AOT 19) decode — ISO/IEC 14496-3 §4.4.2.3
//! Table 4.19 `er_raw_data_block()` carrying the §4.6.7 Long Term
//! Prediction tool.
//!
//! AOT 19 is AOT 4's LTP semantics (Table 4.55 non-LD `ltp_data()`
//! branch: 11-bit lag, `M = 0`, per-sfb `ltp_long_used`) over the ER
//! top-level payload, optionally combined with the §4.4.6 resilience
//! branches. The pins here are exact: the resilience wire changes the
//! coding, not the reconstruction, and the LTP `x_rec` history rules
//! are shared with the non-ER walk — so an AOT-19 decode must be
//! bit-identical to the equivalent AOT-4 `raw_data_block()` decode of
//! the same spectra + LTP side info, frame after frame.

use oxideav_aac::asc::AacResilienceFlags;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::hcr_decode::encode_reordered_spectral_data;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, LtpData, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::latm::LoasDecoder;
use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_core::bits::BitWriter;

const FS_INDEX: u8 = 4; // 44.1 kHz
const SAMPLE_RATE: u32 = 44100;
const AOT_LTP: u8 = 4;
const AOT_ER_LC: u8 = 17;
const AOT_ER_LTP: u8 = 19;

const NO_RESILIENCE: AacResilienceFlags = AacResilienceFlags {
    section_data: false,
    scalefactor_data: false,
    spectral_data: false,
};
const HCR_RESILIENCE: AacResilienceFlags = AacResilienceFlags {
    section_data: false,
    scalefactor_data: false,
    spectral_data: true,
};

fn long_ics_info(max_sfb: u8) -> IcsInfo {
    IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
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

/// The Table 4.55 non-LD `ltp_data()` record: 11-bit `lag`, 3-bit
/// `coef`, `long_used` over `min(max_sfb, MAX_LTP_LONG_SFB)` bands.
fn ltp_record(lag: u16, coef: u8, max_sfb: u8) -> LtpData {
    LtpData {
        lag_update: None,
        lag: Some(lag),
        coef,
        long_used: vec![true; usize::from(max_sfb)],
        short: None,
    }
}

/// Install LTP side info on a channel's `ics_info` (the SCE / CPE
/// channel-0 subtree; `pair` fills the `common_window` second slot).
fn with_ltp(ics: &mut IcsInfo, own: LtpData, pair: Option<LtpData>) {
    ics.predictor_data_present = true;
    ics.ltp_data_present = true;
    ics.ltp_data = Some(own);
    if let Some(p) = pair {
        ics.ltp_data_present_pair = Some(true);
        ics.ltp_data_pair = Some(p);
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
            _ => 30,
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

/// Serialize one ER SCE (tag + body + spectrum) into `bw` under `aot`.
/// The spectrum rides `reordered_spectral_data()` when
/// `resilience.spectral_data` is set, plain `spectral_data()` else.
fn write_er_sce(
    bw: &mut BitWriter,
    tag: u8,
    aot: u8,
    body: &IcsBody,
    spectral: &SpectralData,
    resilience: AacResilienceFlags,
) {
    bw.write_u32(u32::from(tag), 4);
    let ics = body.ics_info.as_ref().unwrap();
    bw.write_u32(u32::from(body.global_gain), 8);
    ics.write(bw, aot, FS_INDEX, false).unwrap();
    body.section_data
        .write(bw, ics.window_sequence, ics.max_sfb)
        .unwrap();
    body.scale_factor_data
        .write(bw, &body.section_data.sfb_cb)
        .unwrap();
    bw.write_bit(false); // pulse_data_present
    bw.write_bit(false); // tns_data_present
    bw.write_bit(false); // gain_control_data_present
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

/// The equivalent non-ER `raw_data_block()` ([SCE, END]) under `aot`.
fn non_er_sce_block(aot: u8, body: &IcsBody, spectral: &SpectralData) -> Vec<u8> {
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let mut bw = BitWriter::new();
    body.write(&mut bw, aot, FS_INDEX, false).unwrap();
    let ics = body.ics_info.as_ref().unwrap();
    spectral
        .write(&mut bw, ics, &body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    fa.push_end()
}

/// An LTP-active AOT-19 SCE stream decodes bit-identically to the
/// AOT-4 `raw_data_block()` decode of the same spectra + LTP side
/// info — over several frames, so the §4.6.7.3 `x_rec` history (zeros
/// on frame 0, real output after) threads identically through the
/// per-element decoder slots.
#[test]
fn er_ltp_stream_matches_non_er_ltp_decode() {
    let (mut body, spectral) = make_channel(&[1, 3, 5, 7, 9, 11, 11, 2], 150, 0x17B);
    let max_sfb = body.ics_info.as_ref().unwrap().max_sfb;
    with_ltp(
        body.ics_info.as_mut().unwrap(),
        ltp_record(1536, 7, max_sfb),
        None,
    );

    let mut bw = BitWriter::new();
    write_er_sce(&mut bw, 0, AOT_ER_LTP, &body, &spectral, NO_RESILIENCE);
    let er_payload = bw.finish();
    let plain = non_er_sce_block(AOT_LTP, &body, &spectral);

    let mut er_dec = StreamDecoder::new();
    let mut plain_dec = StreamDecoder::new();
    for frame in 0..4 {
        let er = er_dec
            .decode_er_raw_data_block(
                AOT_ER_LTP,
                FS_INDEX,
                SAMPLE_RATE,
                1,
                NO_RESILIENCE,
                &er_payload,
            )
            .unwrap();
        let base = plain_dec
            .decode_raw_data_block(AOT_LTP, FS_INDEX, SAMPLE_RATE, 1, 1, &plain)
            .unwrap();
        assert_eq!(er.channels, 1);
        assert_eq!(er.pcm, base.pcm, "frame {frame}");
        assert!(er.pcm.iter().any(|&s| s != 0), "frame {frame} silent");
    }
    // The pin is only meaningful if LTP actually contributed: a
    // second run with the prediction bands disabled must diverge
    // once the history is non-zero.
    let mut no_ltp_body = body.clone();
    let ics = no_ltp_body.ics_info.as_mut().unwrap();
    ics.predictor_data_present = false;
    ics.ltp_data_present = false;
    ics.ltp_data = None;
    let mut bw = BitWriter::new();
    write_er_sce(
        &mut bw,
        0,
        AOT_ER_LTP,
        &no_ltp_body,
        &spectral,
        NO_RESILIENCE,
    );
    let off_payload = bw.finish();
    let mut off_dec = StreamDecoder::new();
    let mut on_dec = StreamDecoder::new();
    let mut diverged = false;
    for _ in 0..4 {
        let off = off_dec
            .decode_er_raw_data_block(
                AOT_ER_LTP,
                FS_INDEX,
                SAMPLE_RATE,
                1,
                NO_RESILIENCE,
                &off_payload,
            )
            .unwrap();
        let on = on_dec
            .decode_er_raw_data_block(
                AOT_ER_LTP,
                FS_INDEX,
                SAMPLE_RATE,
                1,
                NO_RESILIENCE,
                &er_payload,
            )
            .unwrap();
        diverged |= off.pcm != on.pcm;
    }
    assert!(diverged, "LTP side info never changed the reconstruction");
}

/// LTP composed with the §4.6.16.3 HCR spectral-resilience wire: the
/// reordered spectrum decodes to the same coefficients, so the
/// LTP-active reconstruction still matches the plain AOT-4 decode.
#[test]
fn er_ltp_with_hcr_spectrum_matches_plain_decode() {
    let (mut body, spectral) = make_channel(&[1, 3, 5, 7, 9, 11], 149, 0x11C2);
    let max_sfb = body.ics_info.as_ref().unwrap().max_sfb;
    with_ltp(
        body.ics_info.as_mut().unwrap(),
        ltp_record(1100, 5, max_sfb),
        None,
    );

    let mut bw = BitWriter::new();
    write_er_sce(&mut bw, 0, AOT_ER_LTP, &body, &spectral, HCR_RESILIENCE);
    let er_payload = bw.finish();
    let plain = non_er_sce_block(AOT_LTP, &body, &spectral);

    let mut er_dec = StreamDecoder::new();
    let mut plain_dec = StreamDecoder::new();
    for frame in 0..3 {
        let er = er_dec
            .decode_er_raw_data_block(
                AOT_ER_LTP,
                FS_INDEX,
                SAMPLE_RATE,
                1,
                HCR_RESILIENCE,
                &er_payload,
            )
            .unwrap();
        let base = plain_dec
            .decode_raw_data_block(AOT_LTP, FS_INDEX, SAMPLE_RATE, 1, 1, &plain)
            .unwrap();
        assert_eq!(er.pcm, base.pcm, "frame {frame}");
    }
}

/// With `predictor_data_present == 0` the AOT-19 wire is identical to
/// AOT 17's, and so is the reconstruction.
#[test]
fn er_ltp_without_ltp_matches_er_lc() {
    let (body, spectral) = make_channel(&[1, 3, 5, 7, 9, 11], 150, 0xD0C5);
    let mut bw = BitWriter::new();
    write_er_sce(&mut bw, 0, AOT_ER_LTP, &body, &spectral, HCR_RESILIENCE);
    let payload = bw.finish();

    let mut ltp_dec = StreamDecoder::new();
    let mut lc_dec = StreamDecoder::new();
    for frame in 0..2 {
        let a = ltp_dec
            .decode_er_raw_data_block(
                AOT_ER_LTP,
                FS_INDEX,
                SAMPLE_RATE,
                1,
                HCR_RESILIENCE,
                &payload,
            )
            .unwrap();
        let b = lc_dec
            .decode_er_raw_data_block(
                AOT_ER_LC,
                FS_INDEX,
                SAMPLE_RATE,
                1,
                HCR_RESILIENCE,
                &payload,
            )
            .unwrap();
        assert_eq!(a.pcm, b.pcm, "frame {frame}");
    }
}

/// A `common_window` CPE with independent LTP on both channels (the
/// Table 4.6 `ltp_data` + second-`ltp_data_present` pair subtree)
/// matches the non-ER AOT-4 shared-window CPE decode.
#[test]
fn er_ltp_cpe_common_window_pair_matches_plain_decode() {
    let (left_body, left_spec) = make_channel(&[1, 3, 5, 7, 9, 11], 148, 0xA1);
    let (right_body, right_spec) = make_channel(&[1, 3, 5, 7, 9, 11], 152, 0xB2);
    let mut ics = left_body.ics_info.clone().unwrap();
    let max_sfb = ics.max_sfb;
    with_ltp(
        &mut ics,
        ltp_record(1024, 6, max_sfb),
        Some(ltp_record(900, 3, max_sfb)),
    );

    // ER payload: tag, common_window=1, shared ics (with both LTP
    // records), ms_mask=00, then per channel: body-sans-ics + HCR
    // lengths + reordered payload.
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4);
    bw.write_bit(true); // common_window
    ics.write(&mut bw, AOT_ER_LTP, FS_INDEX, true).unwrap();
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

    // Non-ER AOT-4 reference: shared-window CPE in a raw_data_block.
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Cpe, 0).unwrap();
    let mut bw = BitWriter::new();
    bw.write_bit(true); // common_window
    ics.write(&mut bw, AOT_LTP, FS_INDEX, true).unwrap();
    bw.write_u32(0, 2); // ms_mask_present = 00
    for (body, spectral) in [(&left_body, &left_spec), (&right_body, &right_spec)] {
        body.write_with_ics_info(&mut bw, &ics, AOT_LTP, false)
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
    for frame in 0..3 {
        let er = er_dec
            .decode_er_raw_data_block(
                AOT_ER_LTP,
                FS_INDEX,
                SAMPLE_RATE,
                2,
                HCR_RESILIENCE,
                &er_payload,
            )
            .unwrap();
        let base = plain_dec
            .decode_raw_data_block(AOT_LTP, FS_INDEX, SAMPLE_RATE, 2, 1, &plain)
            .unwrap();
        assert_eq!(er.channels, 2);
        assert_eq!(er.pcm, base.pcm, "frame {frame}");
    }
}

/// LOAS/LATM end to end: an AOT-19 ASC routes the payload through
/// `er_raw_data_block()`; a two-sync-frame stream matches the direct
/// entry point frame for frame (the LTP history threads through the
/// per-stream decoder).
#[test]
fn loas_er_ltp_stream_decodes() {
    let (mut body, spectral) = make_channel(&[1, 3, 5, 7, 9, 11], 150, 0x10A5);
    let max_sfb = body.ics_info.as_ref().unwrap().max_sfb;
    with_ltp(
        body.ics_info.as_mut().unwrap(),
        ltp_record(1400, 7, max_sfb),
        None,
    );
    let mut bw = BitWriter::new();
    write_er_sce(&mut bw, 0, AOT_ER_LTP, &body, &spectral, HCR_RESILIENCE);
    let er_payload = bw.finish();

    // ER AAC LTP mono 44.1 kHz ASC: AOT 19, fsIdx 4, chanConfig 1;
    // GASpecificConfig frameLengthFlag=0 dependsOnCoreCoder=0
    // extensionFlag=1; resilience triplet (0,0,1); extensionFlag3=0;
    // epConfig=0.
    let write_asc = |w: &mut BitWriter| {
        w.write_u32(u32::from(AOT_ER_LTP), 5);
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

    // Two LOAS sync frames: the first with an inline StreamMuxConfig,
    // the second inheriting it via useSameStreamMux.
    let mut loas = Vec::new();
    for first in [true, false] {
        let mut w = BitWriter::new();
        if first {
            w.write_bit(false); // useSameStreamMux = 0 (inline config)
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
        } else {
            w.write_bit(true); // useSameStreamMux = 1
        }
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
        loas.extend_from_slice(&fw.finish());
    }

    let mut loas_dec = LoasDecoder::new();
    let frames = loas_dec.decode_all(&loas).unwrap();
    assert_eq!(frames.len(), 2);
    let mut direct = StreamDecoder::new();
    for (i, frame) in frames.iter().enumerate() {
        let expect = direct
            .decode_er_raw_data_block(
                AOT_ER_LTP,
                FS_INDEX,
                SAMPLE_RATE,
                1,
                HCR_RESILIENCE,
                &er_payload,
            )
            .unwrap();
        assert_eq!(frame.pcm, expect.pcm, "sync frame {i}");
        assert_eq!(frame.sample_rate, SAMPLE_RATE);
        assert!(frame.pcm.iter().any(|&s| s != 0));
    }
}

/// AOT 19 runs the 1024/960 families only — a decoder configured for
/// an LD family must reject it (the family/AOT cross-check), and the
/// still-unrouted ER AOT (20, scalable) stays rejected.
#[test]
fn er_ltp_family_and_aot_guards() {
    let (body, spectral) = make_channel(&[1, 3, 5], 140, 0x9);
    let mut bw = BitWriter::new();
    write_er_sce(&mut bw, 0, AOT_ER_LTP, &body, &spectral, NO_RESILIENCE);
    let payload = bw.finish();

    let mut ld_dec = StreamDecoder::new();
    ld_dec.set_frame_family(oxideav_aac::swb_offset::FrameFamily::Ld512);
    assert!(ld_dec
        .decode_er_raw_data_block(
            AOT_ER_LTP,
            FS_INDEX,
            SAMPLE_RATE,
            1,
            NO_RESILIENCE,
            &payload
        )
        .is_err());

    let mut dec = StreamDecoder::new();
    assert!(dec
        .decode_er_raw_data_block(20, FS_INDEX, SAMPLE_RATE, 1, NO_RESILIENCE, &payload)
        .is_err());
}

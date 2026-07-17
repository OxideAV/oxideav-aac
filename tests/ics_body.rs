//! Self-roundtrip tests for [`oxideav_aac::ics_body::IcsBody`] — the
//! Table 4.50 `individual_channel_stream()` body walker that composes
//! the existing per-tool parsers / writers (global_gain, ics_info,
//! section_data, scale_factor_data, optional pulse_data / tns_data /
//! gain_control_data) into a single channel-element body parse /
//! write cycle, stopping just before `spectral_data()`.

use oxideav_aac::gain_control_data::{GainAdjust, GainBand, GainControlData, GainWindow};
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape};
use oxideav_aac::pulse_data::{Pulse, PulseData};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::tns_data::{TnsData, TnsFilter, TnsWindow};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

// ===========================================================================
// Test helpers
// ===========================================================================

/// Build a minimal long-window `IcsInfo` for an AAC-LC frame.
fn make_lc_long_ics_info(max_sfb: u8, num_swb: u8) -> IcsInfo {
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
        num_swb,
    }
}

/// Build an eight-short `IcsInfo` (single group covering all 8 windows,
/// scale_factor_grouping mask of 0x7f i.e. all bits set so windows
/// 1..=7 merge into the same group as window 0).
fn make_lc_eight_short_ics_info(max_sfb: u8, num_swb: u8) -> IcsInfo {
    IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::EightShort,
        window_shape: WindowShape::Sine,
        max_sfb,
        scale_factor_grouping: Some(0x7f),
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 8,
        num_window_groups: 1,
        window_group_length: vec![8],
        num_swb,
    }
}

/// Build a one-section `SectionData` covering all `max_sfb` bands of
/// every group with codebook `cb`.
fn make_uniform_section_data(num_groups: u8, max_sfb: u8, cb: u8) -> SectionData {
    let mut sections = Vec::with_capacity(num_groups as usize);
    let mut sfb_cb = Vec::with_capacity(num_groups as usize);
    for _ in 0..num_groups {
        sections.push(vec![Section {
            codebook: cb,
            start: 0,
            end: max_sfb,
        }]);
        sfb_cb.push(vec![cb; max_sfb as usize]);
    }
    SectionData { sections, sfb_cb }
}

/// Build a `ScaleFactorData` whose entries are all DPCM zero (delta 0)
/// for every non-zero band of every group. PNS / intensity branches
/// are not exercised here — that's the scale_factor_data tests'
/// remit.
fn make_zero_dpcm_scale_factor_data(sfb_cb: &[Vec<u8>]) -> ScaleFactorData {
    let mut entries = Vec::with_capacity(sfb_cb.len());
    for group in sfb_cb {
        let count = group.iter().filter(|&&cb| cb != 0).count();
        entries.push(vec![ScaleFactorEntry::Dpcm(0); count]);
    }
    ScaleFactorData { entries }
}

/// Encode → parse → equality cycle for the inline-ics_info path.
///
/// `body.spectral_data_bit_offset` is ignored on input (the value
/// here is "what the parser derives"); the helper compares everything
/// else field-by-field, then asserts that a re-write of the parsed
/// body produces the same bit-buffer.
fn roundtrip(body: &IcsBody, aot: u8, fs_index: u8) -> u64 {
    let mut bw = BitWriter::new();
    body.write(&mut bw, aot, fs_index, false)
        .expect("encode succeeds");
    let bits_written = bw.bit_position();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = IcsBody::parse(&mut br, aot, fs_index, false)
        .expect("parse of self-encoded bitstream succeeds");
    assert_body_eq_ignoring_offset(&parsed, body);
    assert_eq!(
        parsed.spectral_data_bit_offset, bits_written,
        "spectral_data_bit_offset must equal the bits the writer emitted"
    );
    assert_eq!(
        br.bit_position(),
        bits_written,
        "parser consumed a different number of bits than the writer emitted"
    );

    // Re-encode the parsed body and assert byte-identical re-emission.
    let mut bw2 = BitWriter::new();
    parsed
        .write(&mut bw2, aot, fs_index, false)
        .expect("re-encode succeeds");
    assert_eq!(bw2.bit_position(), bits_written);
    assert_eq!(bw2.finish(), buf);
    bits_written
}

/// Encode → parse → equality cycle for the shared-ics_info path.
fn roundtrip_with_ics_info(body: &IcsBody, ics: &IcsInfo, aot: u8) -> u64 {
    let mut bw = BitWriter::new();
    body.write_with_ics_info(&mut bw, ics, aot, false)
        .expect("encode succeeds");
    let bits_written = bw.bit_position();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = IcsBody::parse_with_ics_info(&mut br, ics, aot, false)
        .expect("parse of self-encoded bitstream succeeds");
    assert_body_eq_ignoring_offset(&parsed, body);
    assert_eq!(
        parsed.spectral_data_bit_offset, bits_written,
        "spectral_data_bit_offset must equal the bits the writer emitted"
    );
    assert_eq!(
        br.bit_position(),
        bits_written,
        "parser consumed a different number of bits than the writer emitted"
    );

    let mut bw2 = BitWriter::new();
    parsed
        .write_with_ics_info(&mut bw2, ics, aot, false)
        .expect("re-encode succeeds");
    assert_eq!(bw2.bit_position(), bits_written);
    assert_eq!(bw2.finish(), buf);
    bits_written
}

/// Compare every field of an [`IcsBody`] except
/// `spectral_data_bit_offset` (which is parser-derived and is
/// expected to equal the writer's bit count, asserted separately).
fn assert_body_eq_ignoring_offset(parsed: &IcsBody, expected: &IcsBody) {
    assert_eq!(parsed.global_gain, expected.global_gain, "global_gain");
    assert_eq!(parsed.ics_info, expected.ics_info, "ics_info");
    assert_eq!(parsed.section_data, expected.section_data, "section_data");
    assert_eq!(
        parsed.scale_factor_data, expected.scale_factor_data,
        "scale_factor_data"
    );
    assert_eq!(
        parsed.pulse_data_present, expected.pulse_data_present,
        "pulse_data_present"
    );
    assert_eq!(parsed.pulse_data, expected.pulse_data, "pulse_data");
    assert_eq!(
        parsed.tns_data_present, expected.tns_data_present,
        "tns_data_present"
    );
    assert_eq!(parsed.tns_data, expected.tns_data, "tns_data");
    assert_eq!(
        parsed.gain_control_data_present, expected.gain_control_data_present,
        "gain_control_data_present"
    );
    assert_eq!(
        parsed.gain_control_data, expected.gain_control_data,
        "gain_control_data"
    );
}

// ===========================================================================
// Minimal SCE / LC long-window body (no tools dispatched)
// ===========================================================================

#[test]
fn minimal_lc_long_body_roundtrips() {
    // SCE-style: AOT 2 (LC), 44.1 kHz, max_sfb = 5, all bands silent
    // (codebook ZERO_HCB) so scale_factor_data is empty. None of the
    // three optional tools are present.
    let ics = make_lc_long_ics_info(5, 49);
    let sd = make_uniform_section_data(1, 5, 0); // all ZERO_HCB
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let body = IcsBody {
        global_gain: 128,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0, // computed by parser
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    // Spectral-data bit offset is recomputed from the parsed stream;
    // build the expected by parsing first and then asserting the
    // round-trip equals what we get back.
    let mut bw = BitWriter::new();
    body.write(&mut bw, 2, 4, false).expect("encode succeeds");
    let bits_written = bw.bit_position();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = IcsBody::parse(&mut br, 2, 4, false).expect("parse succeeds");
    assert_eq!(parsed.global_gain, 128);
    assert!(!parsed.pulse_data_present);
    assert!(!parsed.tns_data_present);
    assert!(!parsed.gain_control_data_present);
    assert_eq!(parsed.pulse_data, None);
    assert_eq!(parsed.tns_data, None);
    assert_eq!(parsed.gain_control_data, None);
    assert_eq!(parsed.spectral_data_bit_offset, bits_written);
    assert_eq!(br.bit_position(), bits_written);

    // Re-write the parsed body and verify bit-for-bit equality.
    let mut bw2 = BitWriter::new();
    parsed
        .write(&mut bw2, 2, 4, false)
        .expect("encode succeeds");
    assert_eq!(bw2.bit_position(), bits_written);
    assert_eq!(bw2.finish(), buf);
}

#[test]
fn lc_long_body_with_one_active_band_roundtrips() {
    // Codebook 4 (QUAD-unsigned) for the single band; scale_factor_data
    // emits one DPCM entry per group.
    let ics = make_lc_long_ics_info(1, 49);
    let sd = make_uniform_section_data(1, 1, 4);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let body = IcsBody {
        global_gain: 200,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
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
    // Self-roundtrip via parser-derived spectral_data_bit_offset.
    let mut bw = BitWriter::new();
    body.write(&mut bw, 2, 4, false).expect("encode succeeds");
    let bits = bw.bit_position();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = IcsBody::parse(&mut br, 2, 4, false).expect("parse succeeds");
    assert_eq!(parsed.global_gain, 200);
    assert_eq!(parsed.section_data.sections[0][0].codebook, 4);
    assert_eq!(parsed.spectral_data_bit_offset, bits);
}

// ===========================================================================
// Pulse-data dispatch
// ===========================================================================

#[test]
fn lc_long_body_with_pulse_data_roundtrips() {
    let ics = make_lc_long_ics_info(5, 49);
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let pd = PulseData {
        pulse_start_sfb: 0,
        pulses: vec![Pulse { offset: 3, amp: 1 }, Pulse { offset: 7, amp: 2 }],
    };
    let body = IcsBody {
        global_gain: 100,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
        pulse_data_present: true,
        pulse_data: Some(pd),
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    roundtrip(&body, 2, 4);
}

#[test]
fn pulse_data_on_eight_short_rejected_by_writer() {
    // Table 4.50 Note 1: pulse_data is illegal on EIGHT_SHORT_SEQUENCE.
    let ics = make_lc_eight_short_ics_info(5, 14);
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let pd = PulseData {
        pulse_start_sfb: 0,
        pulses: vec![Pulse { offset: 3, amp: 1 }],
    };
    let body = IcsBody {
        global_gain: 0,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
        pulse_data_present: true,
        pulse_data: Some(pd),
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    let mut bw = BitWriter::new();
    let err = body
        .write(&mut bw, 2, 4, false)
        .expect_err("writer must reject pulse_data on EIGHT_SHORT");
    assert_eq!(err, Error::PulseDataEncodeInvalid);
}

#[test]
fn pulse_data_slot_populated_without_dispatch_bit_rejected() {
    let ics = make_lc_long_ics_info(5, 49);
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let pd = PulseData {
        pulse_start_sfb: 0,
        pulses: vec![Pulse { offset: 3, amp: 1 }],
    };
    let body = IcsBody {
        global_gain: 0,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
        pulse_data_present: false,
        pulse_data: Some(pd), // slot set while dispatch bit is clear
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    let mut bw = BitWriter::new();
    let err = body
        .write(&mut bw, 2, 4, false)
        .expect_err("writer must reject populated pulse_data slot without dispatch bit");
    assert_eq!(err, Error::PulseDataEncodeInvalid);
}

// ===========================================================================
// TNS-data dispatch
// ===========================================================================

#[test]
fn lc_long_body_with_tns_data_roundtrips() {
    let ics = make_lc_long_ics_info(5, 49);
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    // Single window, single filter, order 2.
    let tns = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                length: 5,
                order: 2,
                direction: false,
                coef_compress: false,
                coef: vec![1, 2],
            }],
        }],
    };
    let body = IcsBody {
        global_gain: 80,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: true,
        tns_data: Some(tns),
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    roundtrip(&body, 2, 4);
}

#[test]
fn lc_eight_short_body_with_tns_data_roundtrips() {
    // EIGHT_SHORT: tns_data has 8 windows, each with a smaller n_filt
    // field width. Even an all-zero filter list per window round-trips.
    let ics = make_lc_eight_short_ics_info(5, 14);
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let tns = TnsData {
        windows: (0..8)
            .map(|_| TnsWindow {
                coef_res: false,
                filters: vec![],
            })
            .collect(),
    };
    let body = IcsBody {
        global_gain: 64,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: true,
        tns_data: Some(tns),
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    roundtrip(&body, 2, 4);
}

// ===========================================================================
// Gain-control dispatch (AOT 3 SSR only)
// ===========================================================================

#[test]
fn ssr_long_body_with_gain_control_roundtrips() {
    // AOT 3 (SSR), only AOT permitted to carry gain_control_data.
    let ics = make_lc_long_ics_info(5, 49);
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    // max_band = 1 means one band slot, one window per LONG sequence.
    let gc = GainControlData {
        max_band: 1,
        bands: vec![GainBand {
            windows: vec![GainWindow {
                adjustments: vec![GainAdjust {
                    alevcode: 3,
                    aloccode: 7, // 5-bit field for OnlyLong wd=0
                }],
            }],
        }],
    };
    let body = IcsBody {
        global_gain: 50,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: true,
        gain_control_data: Some(gc),
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    roundtrip(&body, 3, 4);
}

#[test]
fn gain_control_on_non_ssr_aot_rejected_by_writer() {
    let ics = make_lc_long_ics_info(5, 49);
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let gc = GainControlData {
        max_band: 0,
        bands: vec![],
    };
    let body = IcsBody {
        global_gain: 50,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: true,
        gain_control_data: Some(gc),
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    let mut bw = BitWriter::new();
    // AOT 2 (LC) — not 3 (SSR) — must be rejected.
    let err = body
        .write(&mut bw, 2, 4, false)
        .expect_err("writer must reject gain_control_data on non-SSR AOT");
    assert_eq!(err, Error::GainControlDataEncodeInvalid);
}

// ===========================================================================
// Shared-ics_info path (CPE common_window form)
// ===========================================================================

#[test]
fn cpe_shared_ics_info_path_roundtrips() {
    // Caller holds the shared IcsInfo outside the body. The body
    // itself carries ics_info == None.
    let ics = make_lc_long_ics_info(5, 49);
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let body = IcsBody {
        global_gain: 90,
        ics_info: None,
        section_data: sd,
        scale_factor_data: sf,
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
    roundtrip_with_ics_info(&body, &ics, 2);
}

#[test]
fn write_inline_with_missing_ics_info_rejected() {
    // The inline `write` requires Self::ics_info to be Some.
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let body = IcsBody {
        global_gain: 0,
        ics_info: None,
        section_data: sd,
        scale_factor_data: sf,
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
    let mut bw = BitWriter::new();
    let err = body
        .write(&mut bw, 2, 4, false)
        .expect_err("writer must reject missing inline IcsInfo");
    assert_eq!(err, Error::IcsInfoEncodeInvalid);
}

// ===========================================================================
// scale_flag rejection (scalable AAC, AOT 6 — not yet supported)
// ===========================================================================

#[test]
fn scale_flag_true_rejected_on_parse() {
    let mut bw = BitWriter::new();
    bw.write_u32(0xFF, 8); // arbitrary global_gain
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let err =
        IcsBody::parse(&mut br, 6, 4, true).expect_err("parser must reject scale_flag == true");
    assert_eq!(err, Error::NotImplemented);
}

#[test]
fn scale_flag_true_rejected_on_write() {
    let ics = make_lc_long_ics_info(5, 49);
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let body = IcsBody {
        global_gain: 0,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
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
    let mut bw = BitWriter::new();
    let err = body
        .write(&mut bw, 6, 4, true)
        .expect_err("writer must reject scale_flag == true");
    assert_eq!(err, Error::NotImplemented);
}

// ===========================================================================
// All-tools-dispatched stress test (SSR LC long, pulse + tns + gain)
// ===========================================================================

#[test]
fn ssr_long_body_with_all_tools_roundtrips() {
    let ics = make_lc_long_ics_info(5, 49);
    let sd = make_uniform_section_data(1, 5, 0);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let pd = PulseData {
        pulse_start_sfb: 0,
        pulses: vec![Pulse { offset: 4, amp: 3 }],
    };
    let tns = TnsData {
        windows: vec![TnsWindow {
            coef_res: true,
            filters: vec![TnsFilter {
                length: 1,
                order: 1,
                direction: true,
                coef_compress: false,
                coef: vec![5],
            }],
        }],
    };
    let gc = GainControlData {
        max_band: 1,
        bands: vec![GainBand {
            windows: vec![GainWindow {
                adjustments: vec![],
            }],
        }],
    };
    let body = IcsBody {
        global_gain: 137,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
        pulse_data_present: true,
        pulse_data: Some(pd),
        tns_data_present: true,
        tns_data: Some(tns),
        gain_control_data_present: true,
        gain_control_data: Some(gc),
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    roundtrip(&body, 3, 4);
}

// ===========================================================================
// Spectral-data bit offset accuracy
// ===========================================================================

#[test]
fn spectral_data_bit_offset_matches_writer_position() {
    // Build a non-trivial body, encode it, parse it back, and verify
    // the surfaced spectral_data_bit_offset equals exactly the number
    // of bits the writer emitted (since the trailing spectral_data is
    // not written).
    let ics = make_lc_long_ics_info(3, 49);
    let sd = make_uniform_section_data(1, 3, 4);
    let sf = make_zero_dpcm_scale_factor_data(&sd.sfb_cb);
    let tns = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![],
        }],
    };
    let body = IcsBody {
        global_gain: 200,
        ics_info: Some(ics),
        section_data: sd,
        scale_factor_data: sf,
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: true,
        tns_data: Some(tns),
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    let mut bw = BitWriter::new();
    body.write(&mut bw, 2, 4, false).expect("encode succeeds");
    let bits = bw.bit_position();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = IcsBody::parse(&mut br, 2, 4, false).expect("parse succeeds");
    assert_eq!(parsed.spectral_data_bit_offset, bits);
    assert_eq!(br.bit_position(), bits);
}

// ===========================================================================
// Error-resilient ICS body (Table 4.50 ER branch) — section_data() ER,
// scale_factor_data() RVLC, and the spectral-resilience length fields.
// ===========================================================================

use oxideav_aac::asc::AacResilienceFlags;
use oxideav_aac::scale_factor_data::ErScaleFactorData;

/// ER AAC LC object type (AOTs 17/19/20/23 carry the resilience triplet).
const AOT_ER_AAC_LC: u8 = 17;
const FS_44100_IDX: u8 = 4;

/// Build a one-section ER scale_factor_data() carrying a single DPCM
/// scalefactor band so the body parses end to end.
fn make_er_sf(global_gain: u8) -> ErScaleFactorData {
    ErScaleFactorData {
        sf_concealment: false,
        rev_global_gain: global_gain,
        data: ScaleFactorData {
            entries: vec![vec![ScaleFactorEntry::Dpcm(0)]],
        },
        dpcm_is_last_position: None,
        dpcm_noise_last_position: None,
    }
}

#[test]
fn er_ics_body_full_resilience_round_geometry() {
    let ics = make_lc_long_ics_info(1, 1);
    let resilience = AacResilienceFlags {
        section_data: true,
        scalefactor_data: true,
        spectral_data: true,
    };

    // section_data: one ER section, cb 4 over the single band (escape
    // coding for cb < 11).
    let section = SectionData {
        sections: vec![vec![Section {
            codebook: 4,
            start: 0,
            end: 1,
        }]],
        sfb_cb: vec![vec![4]],
    };
    let er_sf = make_er_sf(0);

    let mut bw = BitWriter::new();
    bw.write_u32(0, 8); // global_gain
    ics.write(&mut bw, AOT_ER_AAC_LC, FS_44100_IDX, false)
        .unwrap();
    section
        .write_er(&mut bw, WindowSequence::OnlyLong, 1)
        .unwrap();
    er_sf
        .write(&mut bw, &section.sfb_cb, WindowSequence::OnlyLong)
        .unwrap();
    bw.write_u32(0, 1); // pulse_data_present
    bw.write_u32(0, 1); // tns_data_present
    bw.write_u32(0, 1); // gain_control_data_present
    bw.write_u32(123, 14); // length_of_reordered_spectral_data
    bw.write_u32(17, 6); // length_of_longest_codeword
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let body = IcsBody::parse_er(&mut br, AOT_ER_AAC_LC, FS_44100_IDX, false, resilience).unwrap();

    assert!(body.er_scale_factor_data.is_some());
    assert_eq!(body.reordered_spectral_lengths, Some((123, 17)));
    // The ER section preserved the 5-bit codebook value.
    assert_eq!(body.section_data.sections[0][0].codebook, 4);
    // The RVLC reconstruction mirrors into the shared scale_factor_data.
    assert_eq!(
        body.scale_factor_data.entries,
        body.er_scale_factor_data.as_ref().unwrap().data.entries
    );
}

#[test]
fn er_ics_body_scalefactor_only_no_spectral_lengths() {
    // Only aacScalefactorDataResilienceFlag set: section_data is the
    // ordinary 4-bit branch, no spectral length fields.
    let ics = make_lc_long_ics_info(1, 1);
    let resilience = AacResilienceFlags {
        section_data: false,
        scalefactor_data: true,
        spectral_data: false,
    };
    let section = SectionData {
        sections: vec![vec![Section {
            codebook: 4,
            start: 0,
            end: 1,
        }]],
        sfb_cb: vec![vec![4]],
    };
    let er_sf = make_er_sf(0);

    let mut bw = BitWriter::new();
    bw.write_u32(0, 8);
    ics.write(&mut bw, AOT_ER_AAC_LC, FS_44100_IDX, false)
        .unwrap();
    section.write(&mut bw, WindowSequence::OnlyLong, 1).unwrap(); // non-ER section
    er_sf
        .write(&mut bw, &section.sfb_cb, WindowSequence::OnlyLong)
        .unwrap();
    bw.write_u32(0, 1);
    bw.write_u32(0, 1);
    bw.write_u32(0, 1);
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let body = IcsBody::parse_er(&mut br, AOT_ER_AAC_LC, FS_44100_IDX, false, resilience).unwrap();
    assert!(body.er_scale_factor_data.is_some());
    assert_eq!(body.reordered_spectral_lengths, None);
}

#[test]
fn er_ics_body_shared_info_cpe_form() {
    // parse_with_ics_info_er uses the caller's ics_info for geometry.
    let ics = make_lc_long_ics_info(1, 1);
    let resilience = AacResilienceFlags {
        section_data: true,
        scalefactor_data: true,
        spectral_data: true,
    };
    let section = SectionData {
        sections: vec![vec![Section {
            codebook: 4,
            start: 0,
            end: 1,
        }]],
        sfb_cb: vec![vec![4]],
    };
    let er_sf = make_er_sf(0);

    let mut bw = BitWriter::new();
    bw.write_u32(0, 8); // global_gain (no inline ics_info on the shared form)
    section
        .write_er(&mut bw, WindowSequence::OnlyLong, 1)
        .unwrap();
    er_sf
        .write(&mut bw, &section.sfb_cb, WindowSequence::OnlyLong)
        .unwrap();
    bw.write_u32(0, 1);
    bw.write_u32(0, 1);
    bw.write_u32(0, 1);
    bw.write_u32(50, 14);
    bw.write_u32(9, 6);
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let body =
        IcsBody::parse_with_ics_info_er(&mut br, &ics, AOT_ER_AAC_LC, false, resilience).unwrap();
    assert!(body.ics_info.is_none());
    assert_eq!(body.reordered_spectral_lengths, Some((50, 9)));
}

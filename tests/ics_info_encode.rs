//! Self-roundtrip tests for [`oxideav_aac::ics_info::IcsInfo::write`]
//! — the AAC crate's second encoder primitive (ISO/IEC 14496-3
//! §4.4.6 Table 4.6 / Table 4.55).
//!
//! Each test builds an `IcsInfo` (either in-memory or by parsing a
//! synthetic bit-stream constructed with `BitWriter`), runs
//! `IcsInfo::write` into a fresh `BitWriter`, parses the resulting
//! buffer back, and asserts structural equality plus matching bit
//! count. The inputs are the
//! spec-defined wire field widths (Table 4.6: `ics_reserved_bit` 1 +
//! `window_sequence` 2 + `window_shape` 1 + `max_sfb` 4 / 6 + the
//! per-branch tail) and the §4.6.7.2 / Table 4.55 LTP body.

use oxideav_aac::ics_info::{
    write_ltp_data, IcsInfo, LtpData, PredictorData, WindowSequence, WindowShape,
};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// Encode → parse → equality cycle. Returns the bit position the
/// writer reached (which the parser must consume exactly).
fn roundtrip(info: &IcsInfo, aot: u8, fs_index: u8, common_window: bool) -> u64 {
    let mut bw = BitWriter::new();
    info.write(&mut bw, aot, fs_index, common_window)
        .expect("encode succeeds");
    let bits_written = bw.bit_position();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = IcsInfo::parse(&mut br, aot, fs_index, common_window)
        .expect("parse of self-encoded bitstream succeeds");
    assert_eq!(&parsed, info, "round-trip changes IcsInfo");
    assert_eq!(
        br.bit_position(),
        bits_written,
        "parser consumed a different number of bits than the writer emitted"
    );
    bits_written
}

// ---- Long branch ----

#[test]
fn long_lc_no_predictor_roundtrips() {
    // AOT 2 (LC), 44.1 kHz, max_sfb = 49, predictor_data_present = 0.
    // Wire bits: 1 (reserved) + 2 (ws) + 1 (shape) + 6 (max_sfb) + 1 (pred) = 11.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 49,
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
        num_swb: 49,
    };
    let bits = roundtrip(&info, 2, 4, false);
    assert_eq!(bits, 11);
}

#[test]
fn long_lc_kbd_window_shape_roundtrips() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Kbd,
        max_sfb: 40,
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
        num_swb: 49, // 44.1 kHz long
    };
    roundtrip(&info, 2, 4, false);
}

#[test]
fn long_start_no_predictor_roundtrips() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::LongStart,
        window_shape: WindowShape::Sine,
        max_sfb: 30,
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
        num_swb: 49,
    };
    roundtrip(&info, 2, 4, false);
}

#[test]
fn long_stop_with_reserved_bit_set_roundtrips() {
    // Parser surfaces ics_reserved_bit verbatim even when non-zero;
    // the writer must round-trip the same wire value.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: true,
        window_sequence: WindowSequence::LongStop,
        window_shape: WindowShape::Sine,
        max_sfb: 49,
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
        num_swb: 49,
    };
    roundtrip(&info, 2, 4, false);
}

#[test]
fn max_sfb_zero_long_roundtrips() {
    // max_sfb == 0 is permitted; downstream tools (section_data) just
    // emit empty sections per group.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 0,
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
        num_swb: 49,
    };
    let bits = roundtrip(&info, 2, 4, false);
    assert_eq!(bits, 11); // 1 + 2 + 1 + 6 + 1
}

// ---- EIGHT_SHORT branch ----

#[test]
fn eight_short_no_grouping_roundtrips() {
    // EIGHT_SHORT_SEQUENCE: max_sfb 4 bits, scale_factor_grouping 7 bits.
    // grouping = 0 → eight separate window groups (each window alone).
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::EightShort,
        window_shape: WindowShape::Sine,
        max_sfb: 12,
        scale_factor_grouping: Some(0b000_0000),
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 8,
        num_window_groups: 8,
        window_group_length: vec![1; 8],
        num_swb: 14, // 44.1 kHz short
    };
    // Bits: 1 + 2 + 1 + 4 + 7 = 15.
    let bits = roundtrip(&info, 2, 4, false);
    assert_eq!(bits, 15);
}

#[test]
fn eight_short_all_grouped_roundtrips() {
    // grouping = 0b111_1111 → all eight windows merge into a single
    // group (every bit 6-i merges window i+1).
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::EightShort,
        window_shape: WindowShape::Sine,
        max_sfb: 8,
        scale_factor_grouping: Some(0b111_1111),
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 8,
        num_window_groups: 1,
        window_group_length: vec![8],
        num_swb: 14,
    };
    roundtrip(&info, 2, 4, false);
}

#[test]
fn eight_short_mixed_grouping_roundtrips() {
    // grouping = 0b101_1010 → groups [1, 1+1, 1, 1+1, 1+1]? Let the
    // parser do the derivation work — the round-trip is what we care
    // about. We only need to set num_windows / num_window_groups /
    // window_group_length to whatever the parser would derive, since
    // PartialEq compares them too.
    let mask = 0b101_1010u8;
    // Manually derive the groups using the spec rule (bit 6-i for
    // i in 0..7).
    let mut groups: Vec<u8> = vec![1];
    for i in 0..7u32 {
        let bit = (mask >> (6 - i as u8)) & 1;
        if bit == 0 {
            groups.push(1);
        } else {
            *groups.last_mut().unwrap() += 1;
        }
    }
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::EightShort,
        window_shape: WindowShape::Kbd,
        max_sfb: 14,
        scale_factor_grouping: Some(mask),
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 8,
        num_window_groups: groups.len() as u8,
        window_group_length: groups,
        num_swb: 14,
    };
    roundtrip(&info, 2, 4, false);
}

// ---- Main predictor branch (AOT 1) ----

#[test]
fn main_predictor_no_reset_roundtrips() {
    // AOT 1, 44.1 kHz: PRED_SFB_MAX = 40, max_sfb = 30 → 30 prediction bits.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 30,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: Some(PredictorData {
            reset: false,
            reset_group_number: None,
            // Alternating bits gives a non-trivial round-trip target.
            prediction_used: (0..30).map(|i| i % 2 == 0).collect(),
        }),
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    // Bits: 1 + 2 + 1 + 6 + 1 + 1 (reset=0) + 30 = 42.
    let bits = roundtrip(&info, 1, 4, false);
    assert_eq!(bits, 42);
}

#[test]
fn main_predictor_with_reset_roundtrips() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 10,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: Some(PredictorData {
            reset: true,
            reset_group_number: Some(15),
            prediction_used: vec![true; 10],
        }),
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    // Bits: 1 + 2 + 1 + 6 + 1 + 1 (reset=1) + 5 (reset_group) + 10 = 27.
    let bits = roundtrip(&info, 1, 4, false);
    assert_eq!(bits, 27);
}

#[test]
fn main_predictor_max_sfb_caps_at_pred_sfb_max_roundtrips() {
    // AOT 1, 96 kHz (fs_index 0): PRED_SFB_MAX = 33; max_sfb = 41 →
    // 33 prediction bits (capped).
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 41,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: Some(PredictorData {
            reset: false,
            reset_group_number: None,
            prediction_used: vec![true; 33], // capped
        }),
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 41,
    };
    // Bits: 1 + 2 + 1 + 6 + 1 + 1 + 33 = 45.
    let bits = roundtrip(&info, 1, 0, false);
    assert_eq!(bits, 45);
}

// ---- LTP branch (AOT 4 / 19 / etc.) ----

#[test]
fn ltp_long_roundtrips() {
    // AOT 4 (LTP), long sequence: 11-bit lag, 3-bit coef, max_sfb
    // bits of ltp_long_used.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 20,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(LtpData {
            lag_update: None,
            lag: Some(1234),
            coef: 5,
            long_used: (0..20).map(|i| i % 3 == 0).collect(),
            short: None,
        }),
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    // Bits: 1 + 2 + 1 + 6 + 1 + 11 + 3 + 20 = 45.
    let bits = roundtrip(&info, 4, 4, false);
    assert_eq!(bits, 45);
}

#[test]
fn ltp_long_caps_at_max_ltp_long_sfb_roundtrips() {
    // max_sfb = 49 (44.1 kHz long max) > MAX_LTP_LONG_SFB (40) →
    // long_used.len() should be 40.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 49,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(LtpData {
            lag_update: None,
            lag: Some(0x7ff),
            coef: 7,
            long_used: vec![true; 40],
            short: None,
        }),
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    // Bits: 1 + 2 + 1 + 6 + 1 + 11 + 3 + 40 = 65.
    let bits = roundtrip(&info, 4, 4, false);
    assert_eq!(bits, 65);
}

#[test]
fn ltp_short_no_long_used_roundtrips() {
    // Non-LD LTP + EIGHT_SHORT_SEQUENCE: lag (11) + coef (3) only,
    // NO long_used[] loop per Table 4.55.
    //
    // BUT — Table 4.6 only enters the LTP branch when
    // `predictor_data_present == 1`, which is only legal on the LONG
    // branch (the EIGHT_SHORT branch in Table 4.6 doesn't carry the
    // `predictor_data_present` bit at all). So a conforming AAC
    // stream never has LTP on a short window. We exercise
    // `write_ltp_data` directly with a constructed LtpData to cover
    // the short-window omission of `long_used`.
    let ltp = LtpData {
        lag_update: None,
        lag: Some(0x123),
        coef: 2,
        long_used: vec![],
        short: None,
    };
    let mut bw = BitWriter::new();
    write_ltp_data(&mut bw, &ltp, 4, WindowSequence::EightShort, 14).unwrap();
    assert_eq!(bw.bit_position(), 14); // 11 + 3
    let buf = bw.finish();
    // Spot-check the wire layout. ltp_lag = 0x123 = 291, encoded in
    // 11 bits is 001_0010_0011; ltp_coef = 010 (3 bits). Packed MSB
    // first, padded to a byte boundary with two trailing zeros:
    //   bit  0..7 = 0010_0100 = 0x24
    //   bit  8..13 (lag tail + coef) = 011_010
    //   bit 14,15 = 00 (pad)
    //   byte 1    = 0110_1000 = 0x68
    assert_eq!(buf.len(), 2);
    assert_eq!(buf[0], 0b0010_0100);
    assert_eq!(buf[1], 0b0110_1000);
}

#[test]
fn ltp_er_aac_ld_with_lag_update_roundtrips() {
    // AOT 23 (ER AAC LD): lag_update (1) + lag (10) + coef (3) +
    // long_used[max_sfb].
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 15,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(LtpData {
            lag_update: Some(true),
            lag: Some(0x3ff),
            coef: 4,
            long_used: vec![
                false, true, false, true, false, true, false, true, false, true, false, true,
                false, true, false,
            ],
            short: None,
        }),
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    // Bits: 1 + 2 + 1 + 6 + 1 + 1 (lag_update=1) + 10 + 3 + 15 = 40.
    let bits = roundtrip(&info, 23, 4, false);
    assert_eq!(bits, 40);
}

#[test]
fn ltp_er_aac_ld_without_lag_update_roundtrips() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 8,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(LtpData {
            lag_update: Some(false),
            lag: None,
            coef: 0,
            long_used: vec![true; 8],
            short: None,
        }),
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    // Bits: 1 + 2 + 1 + 6 + 1 + 1 (lag_update=0, no lag) + 3 + 8 = 23.
    let bits = roundtrip(&info, 23, 4, false);
    assert_eq!(bits, 23);
}

#[test]
fn ltp_common_window_paired_roundtrips() {
    // CPE common_window == true: two consecutive ltp_data() bodies.
    let primary = LtpData {
        lag_update: None,
        lag: Some(0x010),
        coef: 1,
        long_used: vec![true; 5],
        short: None,
    };
    let pair = LtpData {
        lag_update: None,
        lag: Some(0x020),
        coef: 2,
        long_used: vec![false; 5],
        short: None,
    };
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(primary),
        ltp_data_present_pair: Some(true),
        ltp_data_pair: Some(pair),
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    // Bits: 1 + 2 + 1 + 6 + 1 + (11+3+5) + 1 (pair bit) + (11+3+5) = 50.
    let bits = roundtrip(&info, 4, 4, true);
    assert_eq!(bits, 50);
}

#[test]
fn ltp_common_window_pair_absent_roundtrips() {
    // common_window == true, ltp_data_present_pair == Some(false):
    // the 1-bit pair flag is still written, no second ltp_data.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(LtpData {
            lag_update: None,
            lag: Some(0),
            coef: 0,
            long_used: vec![false; 5],
            short: None,
        }),
        ltp_data_present_pair: Some(false),
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    // Bits: 1 + 2 + 1 + 6 + 1 + (11+3+5) + 1 = 31.
    let bits = roundtrip(&info, 4, 4, true);
    assert_eq!(bits, 31);
}

// ---- Encoder rejection branches ----

#[test]
fn write_rejects_max_sfb_overflow_long() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 64, // > 6-bit field
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
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 2, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_max_sfb_overflow_short() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::EightShort,
        window_shape: WindowShape::Sine,
        max_sfb: 16, // > 4-bit field
        scale_factor_grouping: Some(0),
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 8,
        num_window_groups: 8,
        window_group_length: vec![1; 8],
        num_swb: 14,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 2, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_eight_short_without_grouping() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::EightShort,
        window_shape: WindowShape::Sine,
        max_sfb: 8,
        scale_factor_grouping: None, // missing for EIGHT_SHORT
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 8,
        num_window_groups: 8,
        window_group_length: vec![1; 8],
        num_swb: 14,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 2, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_long_with_grouping() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 49,
        scale_factor_grouping: Some(0x55), // not legal on long
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 2, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_grouping_overflow() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::EightShort,
        window_shape: WindowShape::Sine,
        max_sfb: 8,
        scale_factor_grouping: Some(0x80), // > 7-bit field
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 8,
        num_window_groups: 8,
        window_group_length: vec![1; 8],
        num_swb: 14,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 2, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_predictor_data_on_short() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::EightShort,
        window_shape: WindowShape::Sine,
        max_sfb: 8,
        scale_factor_grouping: Some(0),
        predictor_data_present: true, // Table 4.6 omits this on EIGHT_SHORT
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 8,
        num_window_groups: 8,
        window_group_length: vec![1; 8],
        num_swb: 14,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 2, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_predictor_data_with_wrong_aot() {
    // AOT 2 (LC) must not own a PredictorData (Main is AOT 1).
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: Some(PredictorData {
            reset: false,
            reset_group_number: None,
            prediction_used: vec![false; 5],
        }),
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 2, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_ltp_data_on_main_aot() {
    // AOT 1 must not own an LtpData (LTP is for AOT != 1).
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(LtpData {
            lag_update: None,
            lag: Some(0),
            coef: 0,
            long_used: vec![false; 5],
            short: None,
        }),
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 1, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_predictor_with_inconsistent_reset() {
    // reset == true but reset_group_number is None.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: Some(PredictorData {
            reset: true,
            reset_group_number: None, // mismatch
            prediction_used: vec![false; 5],
        }),
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 1, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_prediction_used_wrong_length() {
    // max_sfb=10 → expect 10 prediction bits; we supply 5.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 10,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: Some(PredictorData {
            reset: false,
            reset_group_number: None,
            prediction_used: vec![false; 5],
        }),
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 1, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_pair_slot_without_common_window() {
    // common_window == false but ltp_data_present_pair == Some(_):
    // Table 4.6 does not emit the pair bit, so the slot must be None.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(LtpData {
            lag_update: None,
            lag: Some(0),
            coef: 0,
            long_used: vec![false; 5],
            short: None,
        }),
        ltp_data_present_pair: Some(false),
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 4, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_stale_data_on_no_predictor_branch() {
    // predictor_data_present == false but ltp_data is populated —
    // the writer would silently drop the body, breaking round-trip.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
        scale_factor_grouping: None,
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: Some(LtpData {
            lag_update: None,
            lag: Some(0),
            coef: 0,
            long_used: vec![false; 5],
            short: None,
        }),
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 4, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_ltp_coef_overflow() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(LtpData {
            lag_update: None,
            lag: Some(0),
            coef: 8, // > 3-bit field
            long_used: vec![false; 5],
            short: None,
        }),
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 4, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_ltp_long_used_wrong_length() {
    // max_sfb=5 but long_used has 7 entries.
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(LtpData {
            lag_update: None,
            lag: Some(0),
            coef: 0,
            long_used: vec![false; 7],
            short: None,
        }),
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 4, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_ltp_ld_missing_lag_update() {
    // AOT 23 (LD) requires lag_update == Some(_).
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
        scale_factor_grouping: None,
        predictor_data_present: true,
        predictor_data: None,
        ltp_data_present: true,
        ltp_data: Some(LtpData {
            lag_update: None, // missing for LD
            lag: Some(0),
            coef: 0,
            long_used: vec![false; 5],
            short: None,
        }),
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 23, 4, false),
        Err(Error::IcsInfoEncodeInvalid)
    );
}

#[test]
fn write_rejects_invalid_fs_index() {
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 5,
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
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        info.write(&mut bw, 2, 12, false), // fs_index == 12 is out of range
        Err(Error::IcsInfoEncodeInvalid)
    );
}

// ---- Hand-pinned wire layout ----

#[test]
fn long_lc_wire_layout_pin() {
    // Minimal LC long sequence:
    //   ics_reserved_bit = 0
    //   window_sequence  = 00
    //   window_shape     = 0
    //   max_sfb          = 011001 (= 25, 6 bits)
    //   predictor_data_present = 0
    // Bit string: 0 00 0 011001 0 = 0000_0110_010x  (11 bits, x = pad)
    //           = 0x06 0x40
    let info = IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 25,
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
        num_swb: 49,
    };
    let mut bw = BitWriter::new();
    info.write(&mut bw, 2, 4, false).unwrap();
    assert_eq!(bw.bit_position(), 11);
    let buf = bw.finish();
    assert_eq!(buf.len(), 2);
    assert_eq!(buf[0], 0b0000_0110);
    assert_eq!(buf[1], 0b0100_0000);

    // Round-trip via the parser too.
    let mut br = BitReader::new(&buf);
    let parsed = IcsInfo::parse(&mut br, 2, 4, false).unwrap();
    assert_eq!(parsed, info);
    assert_eq!(br.bit_position(), 11);
}

// ---- ISO/IEC 14496-3:2001 short-window LTP branch ----

#[test]
fn ltp_2001_short_windows_roundtrip() {
    use oxideav_aac::ics_info::{
        parse_ltp_data_edition, write_ltp_data_edition, LtpEdition, LtpShortWindow,
        SHORT_WINDOWS_PER_FRAME,
    };

    // A representative mix: unused windows, used-without-lag, and
    // used-with-lag covering the signed extremes -8, -1, 0, 7.
    let short: Vec<LtpShortWindow> = vec![
        LtpShortWindow {
            used: false,
            lag_present: false,
            lag: 0,
        },
        LtpShortWindow {
            used: true,
            lag_present: false,
            lag: 0,
        },
        LtpShortWindow {
            used: true,
            lag_present: true,
            lag: -8,
        },
        LtpShortWindow {
            used: true,
            lag_present: true,
            lag: 7,
        },
        LtpShortWindow {
            used: true,
            lag_present: true,
            lag: -1,
        },
        LtpShortWindow {
            used: true,
            lag_present: true,
            lag: 0,
        },
        LtpShortWindow {
            used: false,
            lag_present: false,
            lag: 0,
        },
        LtpShortWindow {
            used: true,
            lag_present: true,
            lag: 3,
        },
    ];
    assert_eq!(short.len(), SHORT_WINDOWS_PER_FRAME);
    let ltp = LtpData {
        lag_update: None,
        lag: Some(700),
        coef: 4,
        long_used: vec![],
        short: Some(short),
    };
    let mut bw = BitWriter::new();
    write_ltp_data_edition(
        &mut bw,
        &ltp,
        4,
        WindowSequence::EightShort,
        14,
        LtpEdition::Iso2001,
    )
    .unwrap();
    // Bits: lag 11 + coef 3 + per-window (1 / 2 / 6):
    // w0: 1, w1: 2, w2..w5: 6 each, w6: 1, w7: 6 = 34; total 48.
    assert_eq!(bw.bit_position(), 48);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = parse_ltp_data_edition(
        &mut br,
        4,
        WindowSequence::EightShort,
        14,
        LtpEdition::Iso2001,
    )
    .unwrap();
    assert_eq!(parsed, ltp);
    assert_eq!(br.bit_position(), 48);
}

#[test]
fn ltp_2001_short_lag_is_twos_complement_on_the_wire() {
    use oxideav_aac::ics_info::{parse_ltp_data_edition, LtpEdition};

    // Hand-pack: lag = 0 (11 bits), coef = 0 (3 bits), then
    // window 0: used=1, lag_present=1, lag bits 1111 (= -1 two's
    // complement), then 7 unused windows (single 0 bits).
    let mut bw = BitWriter::new();
    bw.write_u32(0, 11);
    bw.write_u32(0, 3);
    bw.write_bit(true); // used
    bw.write_bit(true); // lag_present
    bw.write_u32(0b1111, 4); // -1
    for _ in 0..7 {
        bw.write_bit(false);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = parse_ltp_data_edition(
        &mut br,
        2,
        WindowSequence::EightShort,
        10,
        LtpEdition::Iso2001,
    )
    .unwrap();
    let short = parsed.short.as_ref().unwrap();
    assert_eq!(short[0].lag, -1);
    assert!(short[0].used && short[0].lag_present);
    assert!(short[1..].iter().all(|w| !w.used));
}

#[test]
fn ltp_2009_short_branch_reads_nothing_after_coef() {
    use oxideav_aac::ics_info::{parse_ltp_data_edition, LtpEdition};

    let mut bw = BitWriter::new();
    bw.write_u32(123, 11);
    bw.write_u32(5, 3);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = parse_ltp_data_edition(
        &mut br,
        2,
        WindowSequence::EightShort,
        10,
        LtpEdition::Iso2009,
    )
    .unwrap();
    assert_eq!(br.bit_position(), 14);
    assert_eq!(parsed.short, None);
    assert!(parsed.long_used.is_empty());
}

#[test]
fn ltp_2001_write_rejects_inconsistent_short_records() {
    use oxideav_aac::ics_info::{write_ltp_data_edition, LtpEdition, LtpShortWindow};

    let base = |w: LtpShortWindow| {
        let mut v = vec![
            LtpShortWindow {
                used: false,
                lag_present: false,
                lag: 0,
            };
            8
        ];
        v[0] = w;
        LtpData {
            lag_update: None,
            lag: Some(1),
            coef: 0,
            long_used: vec![],
            short: Some(v),
        }
    };
    let bad = [
        // unused window carrying a lag_present flag
        LtpShortWindow {
            used: false,
            lag_present: true,
            lag: 0,
        },
        // absent lag but nonzero value
        LtpShortWindow {
            used: true,
            lag_present: false,
            lag: 3,
        },
    ];
    for w in bad {
        let mut bw = BitWriter::new();
        let err = write_ltp_data_edition(
            &mut bw,
            &base(w),
            2,
            WindowSequence::EightShort,
            10,
            LtpEdition::Iso2001,
        )
        .unwrap_err();
        assert!(matches!(err, Error::IcsInfoEncodeInvalid));
    }

    // Wrong vector length.
    let mut short_7 = vec![
        LtpShortWindow {
            used: false,
            lag_present: false,
            lag: 0,
        };
        7
    ];
    short_7.truncate(7);
    let ltp = LtpData {
        lag_update: None,
        lag: Some(1),
        coef: 0,
        long_used: vec![],
        short: Some(short_7),
    };
    let mut bw = BitWriter::new();
    assert!(matches!(
        write_ltp_data_edition(
            &mut bw,
            &ltp,
            2,
            WindowSequence::EightShort,
            10,
            LtpEdition::Iso2001
        ),
        Err(Error::IcsInfoEncodeInvalid)
    ));

    // A 2009-edition write must reject any `short` payload.
    let ltp_2009 = LtpData {
        lag_update: None,
        lag: Some(1),
        coef: 0,
        long_used: vec![],
        short: Some(vec![
            LtpShortWindow {
                used: false,
                lag_present: false,
                lag: 0,
            };
            8
        ]),
    };
    let mut bw = BitWriter::new();
    assert!(matches!(
        write_ltp_data_edition(
            &mut bw,
            &ltp_2009,
            2,
            WindowSequence::EightShort,
            10,
            LtpEdition::Iso2009
        ),
        Err(Error::IcsInfoEncodeInvalid)
    ));
}

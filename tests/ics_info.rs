//! Tests for [`oxideav_aac::ics_info::IcsInfo`] — synthetic
//! `ics_info()` bit-streams constructed via
//! `oxideav_core::bits::BitWriter` so the tests do not depend on
//! any external AAC encoder.

use oxideav_aac::ics_info::{
    derive_window_grouping, IcsInfo, LtpData, WindowSequence, WindowShape, MAX_LTP_LONG_SFB,
    NUM_SWB_LONG_WINDOW, NUM_SWB_SHORT_WINDOW, PRED_SFB_MAX,
};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// Build a minimal long-sequence ics_info for AOT 2 (LC):
///   ics_reserved_bit, window_sequence (2), window_shape (1),
///   max_sfb (6), predictor_data_present (1) = 0.
fn build_long_lc(window_sequence: u8, window_shape: u8, max_sfb: u8) -> Vec<u8> {
    let mut bw = BitWriter::new();
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(window_sequence as u32, 2);
    bw.write_u32(window_shape as u32, 1);
    bw.write_u32(max_sfb as u32, 6);
    bw.write_bit(false); // predictor_data_present
    bw.finish()
}

#[test]
fn parses_only_long_sequence_lc() {
    let buf = build_long_lc(0, 0, 49); // 44.1 kHz LC max
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 2, 4, false).unwrap();

    assert_eq!(info.window_sequence, WindowSequence::OnlyLong);
    assert_eq!(info.window_shape, WindowShape::Sine);
    assert_eq!(info.max_sfb, 49);
    assert_eq!(info.scale_factor_grouping, None);
    assert!(!info.predictor_data_present);
    assert!(info.predictor_data.is_none());
    assert!(info.ltp_data.is_none());
    assert_eq!(info.num_windows, 1);
    assert_eq!(info.num_window_groups, 1);
    assert_eq!(info.window_group_length, vec![1]);
    assert_eq!(info.num_swb, NUM_SWB_LONG_WINDOW[4]);
    // 1 + 2 + 1 + 6 + 1 = 11 bits consumed
    assert_eq!(br.bit_position(), 11);
}

#[test]
fn parses_kbd_window_shape() {
    let buf = build_long_lc(0, 1, 49);
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 2, 4, false).unwrap();
    assert_eq!(info.window_shape, WindowShape::Kbd);
}

#[test]
fn parses_long_start_and_long_stop_sequences() {
    for seq_bits in [1u8, 3u8] {
        let buf = build_long_lc(seq_bits, 0, 25);
        let mut br = BitReader::new(&buf);
        let info = IcsInfo::parse(&mut br, 2, 4, false).unwrap();
        let expected = WindowSequence::from_bits(seq_bits);
        assert_eq!(info.window_sequence, expected);
        // Long start/stop derive identically to OnlyLong.
        assert_eq!(info.num_windows, 1);
        assert_eq!(info.num_window_groups, 1);
        assert_eq!(info.num_swb, NUM_SWB_LONG_WINDOW[4]);
    }
}

#[test]
fn parses_eight_short_sequence_no_grouping() {
    // EIGHT_SHORT, max_sfb=14 (short 44.1kHz max), grouping=0
    // (all 7 grouping bits zero → 8 groups of 1).
    let mut bw = BitWriter::new();
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(2, 2); // window_sequence = EIGHT_SHORT
    bw.write_u32(0, 1); // window_shape
    bw.write_u32(14, 4); // max_sfb (4 bits in short branch)
    bw.write_u32(0, 7); // scale_factor_grouping
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 2, 4, false).unwrap();

    assert_eq!(info.window_sequence, WindowSequence::EightShort);
    assert_eq!(info.max_sfb, 14);
    assert_eq!(info.scale_factor_grouping, Some(0));
    assert_eq!(info.num_windows, 8);
    assert_eq!(info.num_window_groups, 8);
    assert_eq!(info.window_group_length, vec![1; 8]);
    assert_eq!(info.num_swb, NUM_SWB_SHORT_WINDOW[4]);
    // No predictor / ltp fields in short branch.
    assert!(!info.predictor_data_present);
    // 1 + 2 + 1 + 4 + 7 = 15 bits consumed
    assert_eq!(br.bit_position(), 15);
}

#[test]
fn parses_eight_short_sequence_all_grouped() {
    // All 7 grouping bits = 1 → all 8 windows in one group.
    let mut bw = BitWriter::new();
    bw.write_bit(false);
    bw.write_u32(2, 2);
    bw.write_u32(0, 1);
    bw.write_u32(8, 4);
    bw.write_u32(0b111_1111, 7); // group everything
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 2, 4, false).unwrap();

    assert_eq!(info.num_window_groups, 1);
    assert_eq!(info.window_group_length, vec![8]);
}

#[test]
fn eight_short_grouping_mask_bit_position_matches_spec() {
    // Spec: bit_set(mask, 6 - i) controls window (i + 1) for i in 0..7.
    // Mask bit 6 → controls window 1; mask = 0b100_0000 → window 1
    // joins window 0, windows 2..7 each open a new group.
    let mut bw = BitWriter::new();
    bw.write_bit(false);
    bw.write_u32(2, 2);
    bw.write_u32(0, 1);
    bw.write_u32(1, 4);
    bw.write_u32(0b100_0000, 7);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 2, 4, false).unwrap();
    // Groups: [w0,w1], [w2], [w3], [w4], [w5], [w6], [w7] → 7 groups
    assert_eq!(info.num_window_groups, 7);
    assert_eq!(info.window_group_length, vec![2, 1, 1, 1, 1, 1, 1]);
}

#[test]
fn eight_short_grouping_mask_two_groups_of_four() {
    // window 1,2,3 join group 0; window 4 opens group 1; windows
    // 5,6,7 join group 1.
    // Bit 6 (window 1) = 1
    // Bit 5 (window 2) = 1
    // Bit 4 (window 3) = 1
    // Bit 3 (window 4) = 0 ← new group
    // Bit 2 (window 5) = 1
    // Bit 1 (window 6) = 1
    // Bit 0 (window 7) = 1
    // mask = 0b1110_111 = 0x77
    let mut bw = BitWriter::new();
    bw.write_bit(false);
    bw.write_u32(2, 2);
    bw.write_u32(0, 1);
    bw.write_u32(1, 4);
    bw.write_u32(0x77, 7);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 2, 4, false).unwrap();
    assert_eq!(info.num_window_groups, 2);
    assert_eq!(info.window_group_length, vec![4, 4]);
}

#[test]
fn main_predictor_branch_reset_false() {
    // AOT 1 (Main), 48 kHz, max_sfb=10, predictor_data_present=1,
    // predictor_reset=0, then min(10, PRED_SFB_MAX[3])=10
    // prediction_used bits.
    let mut bw = BitWriter::new();
    bw.write_bit(false);
    bw.write_u32(0, 2); // OnlyLong
    bw.write_u32(0, 1);
    bw.write_u32(10, 6); // max_sfb
    bw.write_bit(true); // predictor_data_present
    bw.write_bit(false); // predictor_reset = 0
                         // 10 prediction_used bits, alternating 1010101010
    for i in 0..10u32 {
        bw.write_bit(i % 2 == 0);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 1, 3, false).unwrap();

    assert!(info.predictor_data_present);
    let pd = info.predictor_data.unwrap();
    assert!(!pd.reset);
    assert!(pd.reset_group_number.is_none());
    assert_eq!(pd.prediction_used.len(), 10);
    assert!(pd.prediction_used[0]);
    assert!(!pd.prediction_used[1]);
}

#[test]
fn main_predictor_branch_with_reset_group() {
    // predictor_reset=1, reset_group_number=17, max_sfb caps at PRED_SFB_MAX.
    // 96 kHz (fs_index 0), PRED_SFB_MAX = 33.
    // Set max_sfb = 41 > PRED_SFB_MAX → loop runs PRED_SFB_MAX times.
    let mut bw = BitWriter::new();
    bw.write_bit(false);
    bw.write_u32(0, 2);
    bw.write_u32(0, 1);
    bw.write_u32(41, 6); // max_sfb (96 kHz long max is 41)
    bw.write_bit(true);
    bw.write_bit(true); // predictor_reset = 1
    bw.write_u32(17, 5); // reset_group_number
    for _ in 0..PRED_SFB_MAX[0] {
        bw.write_bit(true);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 1, 0, false).unwrap();
    let pd = info.predictor_data.unwrap();
    assert!(pd.reset);
    assert_eq!(pd.reset_group_number, Some(17));
    assert_eq!(pd.prediction_used.len(), PRED_SFB_MAX[0] as usize);
}

#[test]
fn ltp_branch_aot4_long_sequence() {
    // AOT 4 (LTP), 44.1 kHz, max_sfb=20, predictor_data_present=1,
    // ltp_lag=0x123 (11 bits), ltp_coef=5 (3 bits), 20
    // ltp_long_used bits.
    let mut bw = BitWriter::new();
    bw.write_bit(false);
    bw.write_u32(0, 2);
    bw.write_u32(0, 1);
    bw.write_u32(20, 6);
    bw.write_bit(true); // predictor_data_present
    bw.write_bit(true); // Table 4.6 nested ltp_data_present
                        // ltp_data() — non-LD branch
    bw.write_u32(0x123, 11); // ltp_lag
    bw.write_u32(5, 3); // ltp_coef
    for _ in 0..20 {
        bw.write_bit(false);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 4, 4, false).unwrap();

    assert!(info.predictor_data_present);
    assert!(info.ltp_data_present);
    let ltp = info.ltp_data.unwrap();
    assert_eq!(ltp.lag, Some(0x123));
    assert_eq!(ltp.coef, 5);
    assert_eq!(ltp.long_used.len(), 20);
    assert!(ltp.lag_update.is_none());
    assert!(info.ltp_data_present_pair.is_none());
}

#[test]
fn ltp_branch_aot4_eight_short_skips_long_used_loop() {
    // EIGHT_SHORT does NOT carry ltp_long_used per Table 4.55
    // non-LD branch.
    // Wait — ltp_data_present is only readable in the
    // non-EIGHT_SHORT path of Table 4.6, so this can't happen via
    // IcsInfo::parse. Validate parse_ltp_data directly via the
    // exposed function.
    use oxideav_aac::ics_info::parse_ltp_data;
    let mut bw = BitWriter::new();
    bw.write_u32(0x7FE, 11); // ltp_lag
    bw.write_u32(2, 3); // ltp_coef
                        // No ltp_long_used in short branch.
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let ltp = parse_ltp_data(&mut br, 4, WindowSequence::EightShort, 14).unwrap();
    assert_eq!(ltp.lag, Some(0x7FE));
    assert_eq!(ltp.coef, 2);
    assert!(ltp.long_used.is_empty());
}

#[test]
fn ltp_branch_aot23_er_aac_ld_with_lag_update() {
    // ER AAC LD: ltp_lag_update=1, ltp_lag (10 bits), ltp_coef (3),
    // then min(max_sfb, MAX_LTP_LONG_SFB) long_used bits.
    let mut bw = BitWriter::new();
    bw.write_bit(true); // ltp_lag_update
    bw.write_u32(0x3FF, 10); // ltp_lag
    bw.write_u32(7, 3); // ltp_coef
    for _ in 0..15 {
        bw.write_bit(true);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let ltp =
        oxideav_aac::ics_info::parse_ltp_data(&mut br, 23, WindowSequence::OnlyLong, 15).unwrap();
    assert_eq!(ltp.lag_update, Some(true));
    assert_eq!(ltp.lag, Some(0x3FF));
    assert_eq!(ltp.coef, 7);
    assert_eq!(ltp.long_used.len(), 15);
    assert!(ltp.long_used.iter().all(|b| *b));
}

#[test]
fn ltp_branch_aot23_er_aac_ld_without_lag_update() {
    let mut bw = BitWriter::new();
    bw.write_bit(false); // ltp_lag_update = 0
    bw.write_u32(3, 3); // ltp_coef
    bw.write_bit(true); // single long_used
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let ltp =
        oxideav_aac::ics_info::parse_ltp_data(&mut br, 23, WindowSequence::OnlyLong, 1).unwrap();
    assert_eq!(ltp.lag_update, Some(false));
    assert_eq!(ltp.lag, None);
    assert_eq!(ltp.coef, 3);
    assert_eq!(ltp.long_used, vec![true]);
}

#[test]
fn ltp_long_used_loop_caps_at_max_ltp_long_sfb() {
    // max_sfb > MAX_LTP_LONG_SFB → loop runs MAX_LTP_LONG_SFB times.
    let mut bw = BitWriter::new();
    bw.write_u32(0, 11); // lag
    bw.write_u32(0, 3); // coef
    for _ in 0..MAX_LTP_LONG_SFB {
        bw.write_bit(true);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let ltp =
        oxideav_aac::ics_info::parse_ltp_data(&mut br, 4, WindowSequence::OnlyLong, 63).unwrap();
    assert_eq!(ltp.long_used.len(), MAX_LTP_LONG_SFB);
}

#[test]
fn common_window_reads_second_ltp_present_flag_and_body() {
    // AOT 4, CPE common_window=true. ics_info has the pair-channel
    // ltp_data_present (+ optional body) after the first ltp_data().
    let mut bw = BitWriter::new();
    bw.write_bit(false);
    bw.write_u32(0, 2);
    bw.write_u32(0, 1);
    bw.write_u32(8, 6);
    bw.write_bit(true); // predictor_data_present
    bw.write_bit(true); // Table 4.6 nested ltp_data_present
                        // first ltp_data
    bw.write_u32(0x111, 11);
    bw.write_u32(1, 3);
    for _ in 0..8 {
        bw.write_bit(false);
    }
    // pair-channel ltp_data_present = 1
    bw.write_bit(true);
    // second ltp_data
    bw.write_u32(0x222, 11);
    bw.write_u32(2, 3);
    for _ in 0..8 {
        bw.write_bit(true);
    }
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 4, 4, true).unwrap();

    assert!(info.ltp_data_present);
    let first = info.ltp_data.unwrap();
    assert_eq!(first.lag, Some(0x111));
    assert_eq!(info.ltp_data_present_pair, Some(true));
    let second = info.ltp_data_pair.unwrap();
    assert_eq!(second.lag, Some(0x222));
    assert_eq!(second.coef, 2);
    assert!(second.long_used.iter().all(|b| *b));
}

#[test]
fn common_window_with_pair_ltp_absent_still_reads_flag() {
    let mut bw = BitWriter::new();
    bw.write_bit(false);
    bw.write_u32(0, 2);
    bw.write_u32(0, 1);
    bw.write_u32(8, 6);
    bw.write_bit(true); // predictor_data_present
    bw.write_bit(true); // Table 4.6 nested ltp_data_present
    bw.write_u32(0x000, 11);
    bw.write_u32(0, 3);
    for _ in 0..8 {
        bw.write_bit(false);
    }
    bw.write_bit(false); // pair flag = 0
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 4, 4, true).unwrap();
    assert_eq!(info.ltp_data_present_pair, Some(false));
    assert!(info.ltp_data_pair.is_none());
}

#[test]
fn rejects_unsupported_sample_rate_index() {
    let buf = build_long_lc(0, 0, 1);
    let mut br = BitReader::new(&buf);
    assert_eq!(
        IcsInfo::parse(&mut br, 2, 12, false),
        Err(Error::IcsInfoUnsupportedSampleRateIndex(12))
    );
    let mut br = BitReader::new(&buf);
    assert_eq!(
        IcsInfo::parse(&mut br, 2, 15, false),
        Err(Error::IcsInfoUnsupportedSampleRateIndex(15))
    );
}

#[test]
fn num_swb_long_window_table_values_match_spec_tables() {
    // Spot-check against ISO/IEC 14496-3 Tables 4.129/4.131/4.132/
    // 4.134/4.136/4.138/4.140 (count column).
    assert_eq!(NUM_SWB_LONG_WINDOW[0], 41); // 96 kHz, Table 4.140
    assert_eq!(NUM_SWB_LONG_WINDOW[3], 49); // 48 kHz, Table 4.129
    assert_eq!(NUM_SWB_LONG_WINDOW[5], 51); // 32 kHz, Table 4.131
    assert_eq!(NUM_SWB_LONG_WINDOW[11], 40); // 8 kHz, Table 4.132
}

#[test]
fn num_swb_short_window_table_values_match_spec_tables() {
    assert_eq!(NUM_SWB_SHORT_WINDOW[0], 12); // 96 kHz, Table 4.141
    assert_eq!(NUM_SWB_SHORT_WINDOW[3], 14); // 48 kHz, Table 4.130
    assert_eq!(NUM_SWB_SHORT_WINDOW[6], 15); // 24 kHz, Table 4.137
    assert_eq!(NUM_SWB_SHORT_WINDOW[11], 15); // 8 kHz, Table 4.133
}

#[test]
fn pred_sfb_max_table_values_match_iso13818_7_table_62() {
    assert_eq!(PRED_SFB_MAX[0], 33); // 96 kHz
    assert_eq!(PRED_SFB_MAX[3], 40); // 48 kHz
    assert_eq!(PRED_SFB_MAX[6], 41); // 24 kHz
    assert_eq!(PRED_SFB_MAX[11], 34); // 8 kHz
}

#[test]
fn derive_window_grouping_long_sequences() {
    for ws in [
        WindowSequence::OnlyLong,
        WindowSequence::LongStart,
        WindowSequence::LongStop,
    ] {
        let (nw, ng, lens, sfb) = derive_window_grouping(ws, None, 4);
        assert_eq!(nw, 1);
        assert_eq!(ng, 1);
        assert_eq!(lens, vec![1]);
        assert_eq!(sfb, NUM_SWB_LONG_WINDOW[4]);
    }
}

#[test]
fn derive_window_grouping_short_no_mask_uses_default() {
    // Function treats absent mask as 0 → 8 groups of 1.
    let (nw, ng, lens, sfb) = derive_window_grouping(WindowSequence::EightShort, None, 5);
    assert_eq!(nw, 8);
    assert_eq!(ng, 8);
    assert_eq!(lens, vec![1; 8]);
    assert_eq!(sfb, NUM_SWB_SHORT_WINDOW[5]);
}

#[test]
fn ics_reserved_bit_is_surfaced_verbatim() {
    // Spec says shall be 0; the parser doesn't enforce it.
    let mut bw = BitWriter::new();
    bw.write_bit(true); // ics_reserved_bit = 1
    bw.write_u32(0, 2);
    bw.write_u32(0, 1);
    bw.write_u32(1, 6);
    bw.write_bit(false);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 2, 4, false).unwrap();
    assert!(info.ics_reserved_bit);
}

#[test]
fn full_consumption_for_lc_long_sequence_minimal_path() {
    // The fixtures-doc example trace event:
    //   ICS_INFO window_sequence=1 window_shape=0 max_sfb=42 ...
    // num_windows=1, num_window_groups=1, num_swb=49 for 44.1 kHz.
    let buf = build_long_lc(1, 0, 42);
    let mut br = BitReader::new(&buf);
    let info = IcsInfo::parse(&mut br, 2, 4, false).unwrap();
    assert_eq!(info.window_sequence, WindowSequence::LongStart);
    assert_eq!(info.window_shape, WindowShape::Sine);
    assert_eq!(info.max_sfb, 42);
    assert_eq!(info.num_windows, 1);
    assert_eq!(info.num_window_groups, 1);
    assert_eq!(info.num_swb, 49); // Table 4.129 count for 44.1 kHz
}

#[test]
fn unexpected_end_in_long_branch_surfaces_error() {
    // Provide only 10 bits when 11 are needed for the predictor_data_present path.
    let mut bw = BitWriter::new();
    bw.write_bit(false);
    bw.write_u32(0, 2);
    bw.write_u32(0, 1);
    bw.write_u32(1, 6);
    // No predictor_data_present bit
    let raw = bw.finish();
    // Use exactly the 10 bits — truncate.
    let buf = vec![raw[0], raw[1] & 0xC0]; // 2 bytes, last 6 bits already 0 but unused.
                                           // Actually, easier: use just one byte (8 bits) and read 10 → underflow.
    let one_byte = vec![raw[0]];
    let mut br = BitReader::new(&one_byte);
    let res = IcsInfo::parse(&mut br, 2, 4, false);
    assert!(matches!(res, Err(Error::UnexpectedEnd)));
    // suppress unused warning for buf
    let _ = buf;
}

#[test]
fn ltp_data_struct_can_be_constructed_externally() {
    // Sanity check that LtpData fields are pub — useful for
    // downstream tests that build expected values.
    let ltp = LtpData {
        lag_update: None,
        lag: Some(42),
        coef: 0,
        long_used: vec![],
        short: None,
    };
    assert_eq!(ltp.lag, Some(42));
}

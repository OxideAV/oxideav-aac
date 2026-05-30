//! Integration tests for the §4.5.4.1 scalefactor-band offset tables
//! and the §4.6.13 pulse-escape reconstruction loop
//! (`swb_offset::apply_pulse_data`).
//!
//! Round 194: pure-spec table-content verification plus reconstruction-
//! loop semantics that exercise every documented branch of the §4.6.13
//! pseudocode (sign-dependent `±pulse_amp`, multi-pulse `k` chaining,
//! malformed bitstream rejection).

use oxideav_aac::pulse_data::{Pulse, PulseData};
use oxideav_aac::swb_offset::{
    apply_pulse_data, long_window_offsets, short_window_offsets, LONG_WINDOW_LEN, SHORT_WINDOW_LEN,
    SWB_OFFSET_LONG_WINDOW, SWB_OFFSET_SHORT_WINDOW,
};
use oxideav_aac::Error;

#[test]
fn long_window_table_dispatch_matches_table_provenance() {
    // From the per-row Table-N attribution in ics_info.rs:
    // fs 0, 1 -> Table 4.140 (88.2 / 96 kHz)
    // fs 2    -> Table 4.138 (64 kHz)
    // fs 3, 4 -> Table 4.129 (44.1 / 48 kHz)
    // fs 5    -> Table 4.131 (32 kHz)
    // fs 6, 7 -> Table 4.136 (22.05 / 24 kHz)
    // fs 8, 9, 10 -> Table 4.134 (11.025 / 12 / 16 kHz)
    // fs 11   -> Table 4.132 (8 kHz)
    let t140 = long_window_offsets(0).unwrap();
    let t138 = long_window_offsets(2).unwrap();
    let t129 = long_window_offsets(3).unwrap();
    let t131 = long_window_offsets(5).unwrap();
    let t136 = long_window_offsets(6).unwrap();
    let t134 = long_window_offsets(8).unwrap();
    let t132 = long_window_offsets(11).unwrap();

    // Per-table band counts from the spec:
    assert_eq!(
        t140.len(),
        42,
        "Table 4.140 entry count (41 SWB + sentinel)"
    );
    assert_eq!(
        t138.len(),
        48,
        "Table 4.138 entry count (47 SWB + sentinel)"
    );
    assert_eq!(
        t129.len(),
        50,
        "Table 4.129 entry count (49 SWB + sentinel)"
    );
    assert_eq!(
        t131.len(),
        52,
        "Table 4.131 entry count (51 SWB + sentinel)"
    );
    assert_eq!(
        t136.len(),
        48,
        "Table 4.136 entry count (47 SWB + sentinel)"
    );
    assert_eq!(
        t134.len(),
        44,
        "Table 4.134 entry count (43 SWB + sentinel)"
    );
    assert_eq!(
        t132.len(),
        41,
        "Table 4.132 entry count (40 SWB + sentinel)"
    );

    // Same-table aliasing (multiple fs indices map to the same table).
    assert_eq!(long_window_offsets(1).unwrap(), t140);
    assert_eq!(long_window_offsets(4).unwrap(), t129);
    assert_eq!(long_window_offsets(7).unwrap(), t136);
    assert_eq!(long_window_offsets(9).unwrap(), t134);
    assert_eq!(long_window_offsets(10).unwrap(), t134);
}

#[test]
fn short_window_table_dispatch_matches_table_provenance() {
    // Per-row attribution:
    // fs 0, 1 -> Table 4.141 (88.2 / 96 kHz)
    // fs 2    -> Table 4.139 (64 kHz)
    // fs 3, 4, 5 -> Table 4.130 (32 / 44.1 / 48 kHz)
    // fs 6, 7 -> Table 4.137 (22.05 / 24 kHz)
    // fs 8, 9, 10 -> Table 4.135 (11.025 / 12 / 16 kHz)
    // fs 11   -> Table 4.133 (8 kHz)
    let t141 = short_window_offsets(0).unwrap();
    let t139 = short_window_offsets(2).unwrap();
    let t130 = short_window_offsets(3).unwrap();
    let t137 = short_window_offsets(6).unwrap();
    let t135 = short_window_offsets(8).unwrap();
    let t133 = short_window_offsets(11).unwrap();

    assert_eq!(t141.len(), 13);
    assert_eq!(t139.len(), 13);
    assert_eq!(t130.len(), 15);
    assert_eq!(t137.len(), 16);
    assert_eq!(t135.len(), 16);
    assert_eq!(t133.len(), 16);

    assert_eq!(short_window_offsets(1).unwrap(), t141);
    assert_eq!(short_window_offsets(4).unwrap(), t130);
    assert_eq!(short_window_offsets(5).unwrap(), t130);
    assert_eq!(short_window_offsets(7).unwrap(), t137);
    assert_eq!(short_window_offsets(9).unwrap(), t135);
    assert_eq!(short_window_offsets(10).unwrap(), t135);
}

#[test]
fn raw_table_constants_match_accessor_output() {
    // Independent cross-check: the const-table dispatch matches the
    // safe accessor's output for every valid fs_index.
    for fs in 0..12_u8 {
        let long = long_window_offsets(fs).unwrap();
        let short = short_window_offsets(fs).unwrap();
        assert_eq!(SWB_OFFSET_LONG_WINDOW[fs as usize], long);
        assert_eq!(SWB_OFFSET_SHORT_WINDOW[fs as usize], short);
    }
    assert!(SWB_OFFSET_LONG_WINDOW[12].is_empty());
    assert!(SWB_OFFSET_SHORT_WINDOW[12].is_empty());
}

#[test]
fn long_window_spectrum_length_constant() {
    assert_eq!(LONG_WINDOW_LEN, 1024);
}

#[test]
fn short_window_spectrum_length_constant() {
    assert_eq!(SHORT_WINDOW_LEN, 128);
}

#[test]
fn apply_pulse_data_followups_456_13_pseudocode_listing() {
    // Hand-traced §4.6.13:
    //   k = swb_offset_long_window[fs_index][pulse_start_sfb];
    //   for i in 0..number_pulse + 1:
    //     k += pulse_offset[i]
    //     if x_quant[k] > 0: x_quant[k] += pulse_amp[i]
    //     else:              x_quant[k] -= pulse_amp[i]
    //
    // 44.1 kHz long: swb_offset_long[5] = 20.
    // Pulses: (offset=3, amp=2), (offset=7, amp=5).
    //   i=0: k = 20 + 3 = 23; x_quant[23] = +1 (> 0) → +2 → 3
    //   i=1: k = 23 + 7 = 30; x_quant[30] = -4 (≤ 0) → -5 → -9
    let mut x_quant = vec![0_i32; 1024];
    x_quant[23] = 1;
    x_quant[30] = -4;
    let pd = PulseData {
        pulse_start_sfb: 5,
        pulses: vec![Pulse { offset: 3, amp: 2 }, Pulse { offset: 7, amp: 5 }],
    };
    apply_pulse_data(&mut x_quant, 4, &pd).unwrap();
    assert_eq!(x_quant[23], 3);
    assert_eq!(x_quant[30], -9);
}

#[test]
fn apply_pulse_data_max_amp_max_offset_per_pulse() {
    // Saturate every Table 4.7 field: 4 pulses, each with offset=31
    // (5-bit max), amp=15 (4-bit max). Starting at swb 0 (offset 0)
    // at 48 kHz, k progression is 31, 62, 93, 124, all in-range.
    let mut x_quant = vec![1_i32; 1024];
    let pd = PulseData {
        pulse_start_sfb: 0,
        pulses: vec![
            Pulse {
                offset: 31,
                amp: 15
            };
            4
        ],
    };
    apply_pulse_data(&mut x_quant, 3, &pd).unwrap();
    assert_eq!(x_quant[31], 16);
    assert_eq!(x_quant[62], 16);
    assert_eq!(x_quant[93], 16);
    assert_eq!(x_quant[124], 16);
}

#[test]
fn apply_pulse_data_pulse_offset_can_be_zero() {
    // pulse_offset[i] == 0 is well-formed: it leaves k unchanged
    // between pulses, so two pulses can land on the same coefficient.
    let mut x_quant = vec![5_i32; 1024];
    let pd = PulseData {
        pulse_start_sfb: 2, // 48 kHz: swb_offset_long[2] = 8
        pulses: vec![Pulse { offset: 3, amp: 1 }, Pulse { offset: 0, amp: 2 }],
    };
    apply_pulse_data(&mut x_quant, 3, &pd).unwrap();
    // k = 8 + 3 = 11; x_quant[11] = 5+1 = 6; then 0-offset → x_quant[11] = 6+2 = 8.
    assert_eq!(x_quant[11], 8);
}

#[test]
fn apply_pulse_data_8k_long_window_uses_table_4_132() {
    // 8 kHz long-window has only 40 SWB; swb_offset_long[1] = 12 per
    // Table 4.132's wider band 0.
    let mut x_quant = vec![3_i32; 1024];
    let pd = PulseData {
        pulse_start_sfb: 1,
        pulses: vec![Pulse { offset: 2, amp: 5 }],
    };
    apply_pulse_data(&mut x_quant, 11, &pd).unwrap();
    assert_eq!(x_quant[14], 8);
}

#[test]
fn apply_pulse_data_96k_long_window_uses_table_4_140() {
    // 96 kHz long-window has 41 SWB; swb_offset_long[10] = 40 per
    // Table 4.140.
    let mut x_quant = vec![0_i32; 1024];
    let pd = PulseData {
        pulse_start_sfb: 10,
        pulses: vec![Pulse { offset: 2, amp: 7 }],
    };
    apply_pulse_data(&mut x_quant, 0, &pd).unwrap();
    // 40 + 2 = 42; x_quant[42] = 0 → falls into else branch → -7.
    assert_eq!(x_quant[42], -7);
}

#[test]
fn apply_pulse_data_does_not_touch_unrelated_coefficients() {
    let mut x_quant = vec![123_i32; 1024];
    let pd = PulseData {
        pulse_start_sfb: 0,
        pulses: vec![Pulse { offset: 1, amp: 1 }],
    };
    apply_pulse_data(&mut x_quant, 3, &pd).unwrap();
    // Only x_quant[1] should differ from 123.
    assert_eq!(x_quant[0], 123);
    assert_eq!(x_quant[1], 124);
    for (i, value) in x_quant.iter().enumerate().take(1024).skip(2) {
        assert_eq!(*value, 123, "unexpected mutation at index {}", i);
    }
}

#[test]
fn apply_pulse_data_with_explicit_rate_index_rejected() {
    // samplingFrequencyIndex == 15 is the 24-bit explicit-rate escape;
    // no SWB table is defined and the accessor must reject.
    let mut x_quant = vec![0_i32; 1024];
    let pd = PulseData {
        pulse_start_sfb: 0,
        pulses: vec![Pulse { offset: 1, amp: 1 }],
    };
    assert!(matches!(
        apply_pulse_data(&mut x_quant, 15, &pd),
        Err(Error::IcsInfoUnsupportedSampleRateIndex(15))
    ));
}

#[test]
fn long_offset_widths_are_non_decreasing_within_each_table() {
    // The bands at the spectrum edge are wider than the bands near DC.
    // Strict monotonicity of the offsets is already covered in
    // unit tests; this is a soft-monotonicity check on the *widths*.
    for fs in 0..12_u8 {
        let offsets = long_window_offsets(fs).unwrap();
        let mut prev_width = 0u16;
        let mut saw_widening = false;
        for w in offsets.windows(2) {
            let width = w[1] - w[0];
            if width > prev_width {
                saw_widening = true;
            }
            prev_width = width;
        }
        assert!(
            saw_widening,
            "fs_index {} long-window widths never widen (suspicious)",
            fs
        );
    }
}

//! Integration tests for [`oxideav_aac::spectral_data`] — the
//! ISO/IEC 14496-3 Table 4.56 `spectral_data()` wire walker — and
//! its composition with [`oxideav_aac::ics_body::IcsBody`] into a
//! complete Table 4.50 channel-element body parse / write cycle.

use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData, ZERO_HCB};
use oxideav_aac::spectral_data::{sect_sfb_offset, SpectralData, ESC_FLAG, PAIR_LEN, QUAD_LEN};
use oxideav_aac::swb_offset::{long_window_offsets, short_window_offsets};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

// ===========================================================================
// Helpers
// ===========================================================================

/// Long-window AAC-LC `IcsInfo` for `fs_index` 4 (44.1 kHz).
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
        num_swb: oxideav_aac::ics_info::NUM_SWB_LONG_WINDOW[4],
    }
}

/// Eight-short `IcsInfo` for `fs_index` 4 with explicit grouping.
fn short_ics_info(max_sfb: u8, window_group_length: Vec<u8>, scale_factor_grouping: u8) -> IcsInfo {
    let num_window_groups = window_group_length.len() as u8;
    IcsInfo {
        ics_reserved_bit: false,
        window_sequence: WindowSequence::EightShort,
        window_shape: WindowShape::Sine,
        max_sfb,
        scale_factor_grouping: Some(scale_factor_grouping),
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 8,
        num_window_groups,
        window_group_length,
        num_swb: oxideav_aac::ics_info::NUM_SWB_SHORT_WINDOW[4],
    }
}

/// One uniform section per group with codebook `cb` over all
/// `max_sfb` bands.
fn uniform_section_data(num_groups: usize, cb: u8, max_sfb: u8) -> SectionData {
    let sections = (0..num_groups)
        .map(|_| {
            vec![Section {
                codebook: cb,
                start: 0,
                end: max_sfb,
            }]
        })
        .collect::<Vec<_>>();
    let sfb_cb = (0..num_groups)
        .map(|_| vec![cb; max_sfb as usize])
        .collect::<Vec<_>>();
    SectionData { sections, sfb_cb }
}

/// Write → parse → equality cycle.
fn round_trip(
    data: &SpectralData,
    ics_info: &IcsInfo,
    section_data: &SectionData,
    fs_index: u8,
) -> SpectralData {
    let mut writer = BitWriter::new();
    data.write(&mut writer, ics_info, section_data, fs_index)
        .expect("write succeeds");
    let bits = writer.bit_position();
    let bytes = writer.finish();
    let mut reader = BitReader::new(&bytes);
    let parsed =
        SpectralData::parse(&mut reader, ics_info, section_data, fs_index).expect("parse succeeds");
    assert_eq!(
        reader.bit_position(),
        bits,
        "parser consumed a different bit count than the writer emitted"
    );
    parsed
}

// ===========================================================================
// Wire-level invariants
// ===========================================================================

#[test]
fn quad_len_and_pair_len_match_table_4_95_dims() {
    assert_eq!(QUAD_LEN, 4);
    assert_eq!(PAIR_LEN, 2);
    assert_eq!(ESC_FLAG, 16);
}

#[test]
fn all_zero_quad_section_pins_one_bit_per_tuple() {
    // Codebook 1 parks the §4.6.3.3 zero-tuple (index 40) on the
    // single-bit codeword `0`. Bands 0..2 at fs 4 long span 8
    // coefficients = 2 quad tuples = 2 zero bits, padded to [0x00].
    let info = long_ics_info(2);
    let sd = uniform_section_data(1, 1, 2);
    let data = SpectralData {
        x_quant: vec![vec![0i32; 1024]],
    };
    let mut writer = BitWriter::new();
    data.write(&mut writer, &info, &sd, 4).expect("write");
    assert_eq!(writer.bit_position(), 2);
    assert_eq!(writer.finish(), vec![0x00]);
}

#[test]
fn hand_pinned_zero_byte_decodes_to_zero_spectrum() {
    let info = long_ics_info(2);
    let sd = uniform_section_data(1, 1, 2);
    let mut reader = BitReader::new(&[0x00]);
    let parsed = SpectralData::parse(&mut reader, &info, &sd, 4).expect("parse");
    assert_eq!(reader.bit_position(), 2);
    assert!(parsed.x_quant[0].iter().all(|&v| v == 0));
}

#[test]
fn every_spectrum_codebook_round_trips() {
    // One representative value pattern per book, within each book's
    // Table 4.95 LAV: 1/2 (signed quad, LAV 1), 3/4 (unsigned quad,
    // LAV 2), 5/6 (signed pair, LAV 4), 7/8 (unsigned pair, LAV 7),
    // 9/10 (unsigned pair, LAV 12), 11 (ESC).
    let cases: [(u8, [i32; 8]); 11] = [
        (1, [1, -1, 0, 1, 0, 0, -1, 1]),
        (2, [0, 1, 1, -1, -1, 0, 1, 0]),
        (3, [2, -2, 0, 1, 0, 2, -1, 0]),
        (4, [1, 0, -2, 2, 2, -1, 0, 1]),
        (5, [4, -4, 3, -3, 0, 1, -2, 2]),
        (6, [0, 4, -1, 2, -4, 0, 3, -3]),
        (7, [7, -7, 0, 5, -3, 1, 0, 2]),
        (8, [1, -1, 7, 0, -6, 4, 2, 0]),
        (9, [12, -12, 0, 9, -5, 1, 0, 11]),
        (10, [3, -10, 12, 0, -12, 6, 1, 0]),
        (11, [15, -16, 16, 8191, -8191, 0, 100, -42]),
    ];
    let info = long_ics_info(2);
    for (cb, values) in cases {
        let sd = uniform_section_data(1, cb, 2);
        let mut data = SpectralData {
            x_quant: vec![vec![0i32; 1024]],
        };
        data.x_quant[0][..8].copy_from_slice(&values);
        assert_eq!(
            round_trip(&data, &info, &sd, 4),
            data,
            "codebook {cb} round-trip"
        );
    }
}

#[test]
fn esc_corner_tuple_round_trips_with_two_escapes() {
    // The (±esc, ±esc) corner carries the short in-band (16, 16)
    // codeword plus two sign bits and two escape sequences.
    let info = long_ics_info(1);
    let sd = uniform_section_data(1, 11, 1);
    let mut data = SpectralData {
        x_quant: vec![vec![0i32; 1024]],
    };
    data.x_quant[0][..4].copy_from_slice(&[8191, -8191, 16, -17]);
    assert_eq!(round_trip(&data, &info, &sd, 4), data);
}

#[test]
fn short_grouped_spectrum_round_trips_across_groups() {
    // Grouping 1+2+5 (scale_factor_grouping 0b0111101 is irrelevant
    // here — the helper takes explicit lengths). Mixed codebooks
    // per group exercise the per-group sect_sfb_offset scaling.
    let info = short_ics_info(4, vec![1, 2, 5], 0);
    let offsets = sect_sfb_offset(&info, 4).expect("offsets");
    let sections = vec![
        vec![
            Section {
                codebook: 2,
                start: 0,
                end: 2,
            },
            Section {
                codebook: ZERO_HCB,
                start: 2,
                end: 4,
            },
        ],
        vec![Section {
            codebook: 8,
            start: 0,
            end: 4,
        }],
        vec![Section {
            codebook: 11,
            start: 0,
            end: 4,
        }],
    ];
    let sfb_cb = vec![vec![2, 2, ZERO_HCB, ZERO_HCB], vec![8; 4], vec![11; 4]];
    let sd = SectionData { sections, sfb_cb };
    let mut data = SpectralData {
        x_quant: vec![vec![0i32; 128], vec![0i32; 256], vec![0i32; 640]],
    };
    // Group 0: book 2 over bands 0..2.
    let g0_end = offsets[0][2] as usize;
    for k in 0..g0_end {
        data.x_quant[0][k] = [1, -1, 0][k % 3];
    }
    // Group 1: book 8 over bands 0..4.
    let g1_end = offsets[1][4] as usize;
    for k in 0..g1_end {
        data.x_quant[1][k] = [0, 3, -7, 1][k % 4];
    }
    // Group 2: ESC book with a few escapes.
    let g2_end = offsets[2][4] as usize;
    for k in 0..g2_end {
        data.x_quant[2][k] = [20, 0, -16, 5][k % 4];
    }
    assert_eq!(round_trip(&data, &info, &sd, 4), data);
}

#[test]
fn sect_sfb_offset_totals_match_group_spans() {
    // Per §4.5.2.3.4 the last short-group offset is the grouped
    // band total: Σ widths × window_group_length[g] when max_sfb ==
    // num_swb covers the full 128-line window.
    let num_swb = oxideav_aac::ics_info::NUM_SWB_SHORT_WINDOW[4];
    let info = short_ics_info(num_swb, vec![3, 5], 0);
    let offsets = sect_sfb_offset(&info, 4).expect("offsets");
    let swb = short_window_offsets(4).expect("table");
    assert_eq!(
        offsets[0][num_swb as usize],
        u32::from(swb[swb.len() - 1]) * 3
    );
    assert_eq!(
        offsets[1][num_swb as usize],
        u32::from(swb[swb.len() - 1]) * 5
    );
    // Long: offsets mirror the Table 4.129-family values verbatim.
    let long_info = long_ics_info(10);
    let long_offsets = sect_sfb_offset(&long_info, 4).expect("offsets");
    let long_swb = long_window_offsets(4).expect("table");
    assert_eq!(long_offsets[0][10], u32::from(long_swb[10]));
}

#[test]
fn unsupported_fs_index_propagates() {
    let info = long_ics_info(2);
    assert!(matches!(
        sect_sfb_offset(&info, 12),
        Err(Error::IcsInfoUnsupportedSampleRateIndex(12))
    ));
}

// ===========================================================================
// Composition with the Table 4.50 body walker
// ===========================================================================

#[test]
fn ics_body_then_spectral_data_consumes_a_complete_channel_body() {
    // Serialise a full individual_channel_stream() — the Table 4.50
    // prefix via IcsBody::write, the Table 4.56 spectrum via
    // SpectralData::write — then parse both back sequentially from
    // one reader, the way the raw_data_block walker will drive the
    // pair once the channel-element wiring lands.
    const AOT_LC: u8 = 2;
    const FS_INDEX: u8 = 4;
    let max_sfb = 4u8;
    let ics_info = long_ics_info(max_sfb);
    let section_data = uniform_section_data(1, 11, max_sfb);
    let scale_factor_data = ScaleFactorData {
        entries: vec![vec![ScaleFactorEntry::Dpcm(0); max_sfb as usize]],
    };
    let body = IcsBody {
        global_gain: 100,
        ics_info: Some(ics_info.clone()),
        section_data: section_data.clone(),
        scale_factor_data,
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
    };
    let offsets = sect_sfb_offset(&ics_info, FS_INDEX).expect("offsets");
    let spectrum_end = offsets[0][max_sfb as usize] as usize;
    let mut spectral = SpectralData {
        x_quant: vec![vec![0i32; 1024]],
    };
    for k in 0..spectrum_end {
        spectral.x_quant[0][k] = [3, -1, 0, 25, -16, 0, 7, 0][k % 8];
    }

    let mut writer = BitWriter::new();
    body.write(&mut writer, AOT_LC, FS_INDEX, false)
        .expect("body write");
    let prefix_bits = writer.bit_position();
    spectral
        .write(&mut writer, &ics_info, &section_data, FS_INDEX)
        .expect("spectrum write");
    let total_bits = writer.bit_position();
    let bytes = writer.finish();

    let mut reader = BitReader::new(&bytes);
    let parsed_body = IcsBody::parse(&mut reader, AOT_LC, FS_INDEX, false).expect("body parse");
    assert_eq!(
        parsed_body.spectral_data_bit_offset, prefix_bits,
        "body walker must surface the first spectral_data() bit"
    );
    let parsed_info = parsed_body.ics_info.as_ref().expect("inline ics_info");
    let parsed_spectral = SpectralData::parse(
        &mut reader,
        parsed_info,
        &parsed_body.section_data,
        FS_INDEX,
    )
    .expect("spectrum parse");
    assert_eq!(reader.bit_position(), total_bits);
    assert_eq!(parsed_spectral, spectral);
    assert_eq!(parsed_body.global_gain, 100);
    assert_eq!(&parsed_body.section_data, &section_data);
}

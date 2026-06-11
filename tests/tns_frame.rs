//! End-to-end tests for [`oxideav_aac::tns_frame::tns_decode_frame`]
//! — the ISO/IEC 14496-3 §4.6.9.3 per-frame TNS orchestration.
//!
//! These tests run the *full wire path*: a [`TnsData`] structure is
//! serialised through [`TnsData::write`] into a fresh [`BitWriter`],
//! parsed back with [`TnsData::parse`], and the parsed block drives
//! [`tns_decode_frame`] over a synthetic dequantised spectrum. The
//! result is checked against a manual composition of the §4.6.9.3
//! building blocks ([`tns_decode_coef_to_lpc`] + [`tns_ar_filter`])
//! over the band regions the spec's
//! `min(band, TNS_MAX_BANDS, max_sfb)` arithmetic selects.
//!
//! No external reference decoder is invoked; the only truth is the
//! §4.6.9.3 pseudocode plus the Table 4.102 / 4.103 caps and the
//! Table 4.140 / 4.141 `swb_offset` tables already in the crate.

use oxideav_aac::ics_info::WindowSequence;
use oxideav_aac::swb_offset::{long_window_offsets, short_window_offsets};
use oxideav_aac::tns_coef::{tns_ar_filter, tns_decode_coef_to_lpc};
use oxideav_aac::tns_data::{TnsData, TnsFilter, TnsWindow};
use oxideav_aac::tns_frame::tns_decode_frame;
use oxideav_aac::tns_max::{tns_max_bands, AOT_AAC_LC};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// 48 kHz: long window has 49 SWBs, short has 14.
const FS_48K: u8 = 3;

/// Deterministic non-trivial spectrum.
fn spectrum(len: usize) -> Vec<f64> {
    (0..len)
        .map(|i| ((i * 37 + 11) % 101) as f64 * 0.125 - 6.0)
        .collect()
}

/// Round-trip a [`TnsData`] through the Table 4.54 wire format so the
/// orchestrator consumes a parser-produced structure.
fn wire_roundtrip(tns: &TnsData, seq: WindowSequence) -> TnsData {
    let mut writer = BitWriter::new();
    tns.write(&mut writer, seq).unwrap();
    let bytes = writer.into_bytes();
    let mut reader = BitReader::new(&bytes);
    TnsData::parse(&mut reader, seq).unwrap()
}

#[test]
fn wire_parsed_long_filter_matches_manual_composition() {
    let offsets = long_window_offsets(FS_48K).unwrap();
    let num_swb = offsets.len() - 1; // 49
    let cap = tns_max_bands(AOT_AAC_LC, WindowSequence::OnlyLong, FS_48K).unwrap() as usize; // 40

    let built = TnsData {
        windows: vec![TnsWindow {
            coef_res: true, // coef_res_bits = 4
            filters: vec![TnsFilter {
                length: 20,
                order: 4,
                direction: false,
                coef_compress: false,
                coef: vec![2, 13, 5, 8],
            }],
        }],
    };
    let parsed = wire_roundtrip(&built, WindowSequence::OnlyLong);
    assert_eq!(parsed, built);

    let mut spec = spectrum(1024);
    let mut want = spec.clone();

    // Manual §4.6.9.3 composition: bottom = 49 - 20 = 29, top = 49 →
    // clamped to min(·, 40, 49).
    let start = offsets[29.min(cap)] as usize;
    let end = offsets[num_swb.min(cap)] as usize;
    let lpc = tns_decode_coef_to_lpc(4, 0, &[2, 13, 5, 8]).unwrap();
    tns_ar_filter(&mut want, start, end - start, 1, &lpc).unwrap();

    tns_decode_frame(
        &mut spec,
        &parsed,
        WindowSequence::OnlyLong,
        num_swb as u8,
        AOT_AAC_LC,
        FS_48K,
    )
    .unwrap();
    assert_eq!(spec, want);
    assert_ne!(spec[start..end], spectrum(1024)[start..end]);
}

#[test]
fn wire_parsed_downward_compressed_filter_matches_manual_composition() {
    // coef_res = 0 (3-bit), coef_compress = 1 → 2-bit wire
    // magnitudes; direction = downward.
    let offsets = long_window_offsets(FS_48K).unwrap();
    let num_swb = offsets.len() - 1;
    let cap = tns_max_bands(AOT_AAC_LC, WindowSequence::OnlyLong, FS_48K).unwrap() as usize;

    let built = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                length: 18,
                order: 2,
                direction: true,
                coef_compress: true,
                coef: vec![1, 3],
            }],
        }],
    };
    let parsed = wire_roundtrip(&built, WindowSequence::OnlyLong);

    let mut spec = spectrum(1024);
    let mut want = spec.clone();

    let start = offsets[(num_swb - 18).min(cap)] as usize;
    let end = offsets[num_swb.min(cap)] as usize;
    let lpc = tns_decode_coef_to_lpc(3, 1, &[1, 3]).unwrap();
    tns_ar_filter(&mut want, end - 1, end - start, -1, &lpc).unwrap();

    tns_decode_frame(
        &mut spec,
        &parsed,
        WindowSequence::OnlyLong,
        num_swb as u8,
        AOT_AAC_LC,
        FS_48K,
    )
    .unwrap();
    assert_eq!(spec, want);
}

#[test]
fn wire_parsed_short_sequence_filters_each_window_independently() {
    // Filters on windows 1 and 6 with different coefficients; the
    // remaining 6 windows must be untouched and each filtered window
    // must match its own manual composition.
    let offsets = short_window_offsets(FS_48K).unwrap();
    let num_swb = offsets.len() - 1; // 14
    let cap = tns_max_bands(AOT_AAC_LC, WindowSequence::EightShort, FS_48K).unwrap() as usize; // 14

    let mut windows: Vec<TnsWindow> = (0..8)
        .map(|_| TnsWindow {
            coef_res: false,
            filters: vec![],
        })
        .collect();
    windows[1] = TnsWindow {
        coef_res: false,
        filters: vec![TnsFilter {
            length: 14,
            order: 1,
            direction: false,
            coef_compress: false,
            coef: vec![6],
        }],
    };
    windows[6] = TnsWindow {
        coef_res: false,
        filters: vec![TnsFilter {
            length: 9,
            order: 2,
            direction: true,
            coef_compress: false,
            coef: vec![7, 2],
        }],
    };
    let parsed = wire_roundtrip(&TnsData { windows }, WindowSequence::EightShort);

    let mut spec = spectrum(1024);
    let before = spec.clone();
    tns_decode_frame(
        &mut spec,
        &parsed,
        WindowSequence::EightShort,
        num_swb as u8,
        AOT_AAC_LC,
        FS_48K,
    )
    .unwrap();

    for w in [0_usize, 2, 3, 4, 5, 7] {
        assert_eq!(
            spec[w * 128..(w + 1) * 128],
            before[w * 128..(w + 1) * 128],
            "window {w} must be untouched"
        );
    }

    // Window 1: full-range upward order-1.
    let mut want1 = before[128..256].to_vec();
    let end1 = offsets[num_swb.min(cap)] as usize;
    let lpc1 = tns_decode_coef_to_lpc(3, 0, &[6]).unwrap();
    tns_ar_filter(&mut want1, 0, end1, 1, &lpc1).unwrap();
    assert_eq!(spec[128..256], want1);

    // Window 6: top-9-bands downward order-2.
    let mut want6 = before[6 * 128..7 * 128].to_vec();
    let s6 = offsets[(num_swb - 9).min(cap)] as usize;
    let e6 = offsets[num_swb.min(cap)] as usize;
    let lpc6 = tns_decode_coef_to_lpc(3, 0, &[7, 2]).unwrap();
    tns_ar_filter(&mut want6, e6 - 1, e6 - s6, -1, &lpc6).unwrap();
    assert_eq!(spec[6 * 128..7 * 128], want6);
}

#[test]
fn max_sfb_limits_the_filtered_band_range() {
    // max_sfb = 30 < TNS_MAX_BANDS = 40: nothing at or above
    // swb_offset[30] may change.
    let offsets = long_window_offsets(FS_48K).unwrap();
    let max_sfb = 30_u8;
    let boundary = offsets[max_sfb as usize] as usize;

    let built = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                length: 40,
                order: 1,
                direction: false,
                coef_compress: false,
                coef: vec![5],
            }],
        }],
    };
    let parsed = wire_roundtrip(&built, WindowSequence::OnlyLong);

    let mut spec = spectrum(1024);
    let before = spec.clone();
    tns_decode_frame(
        &mut spec,
        &parsed,
        WindowSequence::OnlyLong,
        max_sfb,
        AOT_AAC_LC,
        FS_48K,
    )
    .unwrap();
    assert_eq!(spec[boundary..], before[boundary..]);
    // bottom = 49 - 40 = 9 < max_sfb → the band-9..30 region was
    // filtered.
    let start = offsets[9] as usize;
    assert_ne!(spec[start..boundary], before[start..boundary]);
}

#[test]
fn frame_level_validation_errors_surface() {
    let tns = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![],
        }],
    };
    // Wrong spectrum length.
    let mut short_spec = spectrum(100);
    assert!(matches!(
        tns_decode_frame(
            &mut short_spec,
            &tns,
            WindowSequence::OnlyLong,
            49,
            AOT_AAC_LC,
            FS_48K
        ),
        Err(Error::TnsFrameInvalid)
    ));
    // Window-count / window-sequence disagreement.
    let mut spec = spectrum(1024);
    assert!(matches!(
        tns_decode_frame(
            &mut spec,
            &tns,
            WindowSequence::EightShort,
            14,
            AOT_AAC_LC,
            FS_48K
        ),
        Err(Error::TnsFrameInvalid)
    ));
    // fs_index without swb_offset coverage.
    assert!(matches!(
        tns_decode_frame(
            &mut spec,
            &tns,
            WindowSequence::OnlyLong,
            49,
            AOT_AAC_LC,
            12
        ),
        Err(Error::IcsInfoUnsupportedSampleRateIndex(12))
    ));
}

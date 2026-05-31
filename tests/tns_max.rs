//! Integration tests for the `tns_max` lookup module —
//! ISO/IEC 14496-3 §4.6.9.4 (Tables 4.102 / 4.103) and
//! §4.6.17.2.5 (Tables 4.119 / 4.120) clamp tables.
//!
//! The unit suite in `src/tns_max.rs` covers every row of every
//! table and each `Error` branch; this integration suite focuses on
//! consumer-facing scenarios — pairing the clamps with the
//! `ics_info` `max_sfb` cap and the `swb_offset` table sizes so the
//! eventual TNS reconstruction loop has a verifiable lockstep.

use oxideav_aac::ics_info::{WindowSequence, NUM_SWB_LONG_WINDOW, NUM_SWB_SHORT_WINDOW};
use oxideav_aac::swb_offset::{long_window_offsets, short_window_offsets};
use oxideav_aac::tns_max::{
    clamp_tns_band, clamp_tns_order, tns_max_bands, tns_max_bands_ld_480, tns_max_bands_ld_512,
    tns_max_order, AOT_AAC_LC, AOT_AAC_MAIN, AOT_AAC_SSR, AOT_ER_AAC_LD,
};
use oxideav_aac::Error;

#[test]
fn tns_max_bands_never_exceeds_num_swb_for_any_aot_and_fs() {
    // §4.6.9.3 takes `min(top, TNS_MAX_BANDS, max_sfb)` and uses it as
    // an index into the SWB offset table. For the clamp to be safe
    // the TNS_MAX_BANDS cap must not point past the last real
    // scalefactor band of the matching swb_offset table — i.e.
    // TNS_MAX_BANDS <= num_swb for that (window, fs) pair.
    for fs in 0..=11_u8 {
        let num_swb_long = NUM_SWB_LONG_WINDOW[fs as usize];
        let num_swb_short = NUM_SWB_SHORT_WINDOW[fs as usize];
        for aot in [AOT_AAC_MAIN, AOT_AAC_LC, AOT_AAC_SSR, 4, 17, 19, 22] {
            let long_cap = tns_max_bands(aot, WindowSequence::OnlyLong, fs).unwrap();
            assert!(
                long_cap <= num_swb_long,
                "AOT {aot} fs {fs}: long TNS_MAX_BANDS = {long_cap} > num_swb_long {num_swb_long}",
            );
            let short_cap = tns_max_bands(aot, WindowSequence::EightShort, fs).unwrap();
            assert!(
                short_cap <= num_swb_short,
                "AOT {aot} fs {fs}: short TNS_MAX_BANDS = {short_cap} > num_swb_short {num_swb_short}",
            );
        }
    }
}

#[test]
fn clamped_band_indexes_swb_offset_inside_spectrum() {
    // For every rate and AOT, taking the band clamp and indexing the
    // long_window_offsets slice must land on a valid offset (never
    // past the slice's last element) — that's the entire reason the
    // clamp exists.
    for fs in 0..=11_u8 {
        let offsets = long_window_offsets(fs).unwrap();
        for aot in [AOT_AAC_LC, AOT_AAC_SSR, AOT_AAC_MAIN] {
            for raw_band in [0_u8, 7, 14, 21, 28, 35, 42, 49, 56, 63] {
                // max_sfb of 49 is the upper bound for any fs (Table
                // 4.129's 49 long-window SWB at 44.1/48 kHz). With the
                // three-way `min`, the clamp must never exceed 49 nor
                // the SWB count for `fs`.
                let clamped =
                    clamp_tns_band(raw_band, 49, aot, WindowSequence::OnlyLong, fs).unwrap();
                assert!(
                    (clamped as usize) < offsets.len(),
                    "AOT {aot} fs {fs} raw_band {raw_band} → clamped {clamped} out of swb_offset len {}",
                    offsets.len(),
                );
            }
        }
    }
}

#[test]
fn clamped_short_band_indexes_short_swb_offset_inside_spectrum() {
    for fs in 0..=11_u8 {
        let offsets = short_window_offsets(fs).unwrap();
        for aot in [AOT_AAC_LC, AOT_AAC_SSR] {
            for raw_band in [0_u8, 4, 8, 14, 31] {
                let clamped =
                    clamp_tns_band(raw_band, 14, aot, WindowSequence::EightShort, fs).unwrap();
                assert!(
                    (clamped as usize) < offsets.len(),
                    "AOT {aot} fs {fs} raw_band {raw_band} → clamped {clamped} out of short swb_offset len {}",
                    offsets.len(),
                );
            }
        }
    }
}

#[test]
fn tns_max_order_caps_within_filter_field_width() {
    // Per Table 4.155 the wire `order` field is 5 bits in non-short
    // windows (max = 31) and 3 bits in short windows (max = 7). The
    // TNS_MAX_ORDER cap must always be at most the matching field's
    // max — otherwise the cap couldn't actually fire on the wire.
    for fs in 0..=12_u8 {
        for aot in [AOT_AAC_MAIN, AOT_AAC_LC, AOT_AAC_SSR, 4, 17, 22, 23] {
            let long_cap = tns_max_order(aot, WindowSequence::OnlyLong, fs).unwrap();
            assert!(long_cap <= 31, "AOT {aot} fs {fs} long cap > 5-bit field");

            let short_cap = tns_max_order(aot, WindowSequence::EightShort, fs).unwrap();
            assert!(short_cap <= 7, "AOT {aot} fs {fs} short cap > 3-bit field");
        }
    }
}

#[test]
fn clamp_chain_handles_44100_lc_realistic_frame() {
    // Realistic AAC-LC 44.1 kHz frame: fs_index = 4, max_sfb = 40
    // (well below the 49 num_swb_long ceiling for this rate). A wire
    // bottom = 0, top = 40 filter should clamp to (0, 40) — neither
    // the TNS_MAX_BANDS (42) nor max_sfb (40) trims it.
    let start = clamp_tns_band(0, 40, AOT_AAC_LC, WindowSequence::OnlyLong, 4).unwrap();
    let end = clamp_tns_band(40, 40, AOT_AAC_LC, WindowSequence::OnlyLong, 4).unwrap();
    assert_eq!(start, 0);
    assert_eq!(end, 40);

    // Wire order = 12 with TNS_MAX_ORDER (LC long) = 12 leaves the
    // order intact.
    let order = clamp_tns_order(12, AOT_AAC_LC, WindowSequence::OnlyLong, 4).unwrap();
    assert_eq!(order, 12);

    // An over-spec'd wire order of 17 (impossible for AAC LC but
    // possible if a future ER variant runs through the same clamp)
    // is brought back to 12.
    let order = clamp_tns_order(17, AOT_AAC_LC, WindowSequence::OnlyLong, 4).unwrap();
    assert_eq!(order, 12);
}

#[test]
fn clamp_chain_handles_96000_ssr_realistic_frame() {
    // AAC SSR at 96 kHz: TNS_MAX_BANDS for long-PQF = 28. A wire
    // top = 40 (impossible but the table-defined cap range) clamps
    // to min(40, 28, 49) = 28; max_sfb = 41 (the 96 kHz long swb
    // count); 49 is the AAC LC ceiling so the SSR clamp must use the
    // PQF column.
    let end = clamp_tns_band(40, 41, AOT_AAC_SSR, WindowSequence::OnlyLong, 0).unwrap();
    assert_eq!(end, 28);

    // SSR long order cap = 12 (same as LC).
    let order = clamp_tns_order(20, AOT_AAC_SSR, WindowSequence::OnlyLong, 0).unwrap();
    assert_eq!(order, 12);
}

#[test]
fn clamp_chain_handles_main_high_rate_long_filter() {
    // AAC Main at 96 kHz: TNS_MAX_ORDER (long) = 20. Wire order of
    // 24 (5-bit field allows up to 31) clamps to 20.
    let order = clamp_tns_order(24, AOT_AAC_MAIN, WindowSequence::OnlyLong, 0).unwrap();
    assert_eq!(order, 20);
}

#[test]
fn ld_tables_diverge_at_24_22_khz_between_480_and_512() {
    // The whole point of Tables 4.119 / 4.120 existing as two
    // separate tables is that the per-(fs, frame-size) caps diverge.
    // At fs 6 (24 kHz) and fs 7 (22.05 kHz) the 480-sample frame
    // caps at 30, while the 512-sample frame caps at 31.
    assert_eq!(tns_max_bands_ld_480(6).unwrap(), 30);
    assert_eq!(tns_max_bands_ld_512(6).unwrap(), 31);
    assert_eq!(tns_max_bands_ld_480(7).unwrap(), 30);
    assert_eq!(tns_max_bands_ld_512(7).unwrap(), 31);

    // At fs 3..=5 the two tables agree (31 / 32 / 37 each).
    for fs in 3..=5_u8 {
        let a = tns_max_bands_ld_480(fs).unwrap();
        let b = tns_max_bands_ld_512(fs).unwrap();
        assert_eq!(a, b, "fs {fs} should match between 480 and 512");
    }
}

#[test]
fn er_aac_ld_aot_constant_matches_table_4_104_row() {
    // Sanity that the public AOT constant matches the ISO/IEC
    // 14496-3 Table 1.16 row for ER AAC LD (decimal 23).
    assert_eq!(AOT_ER_AAC_LD, 23);
}

#[test]
fn error_path_propagates_through_clamp_helpers() {
    // Every clamp helper short-circuits with the unsupported
    // sample-rate error its underlying lookup raises. The integration
    // surface returns the same `Error` variant for the order and
    // bands paths.
    assert!(matches!(
        clamp_tns_order(5, AOT_AAC_LC, WindowSequence::OnlyLong, 13),
        Err(Error::IcsInfoUnsupportedSampleRateIndex(13))
    ));
    assert!(matches!(
        clamp_tns_band(5, 49, AOT_AAC_LC, WindowSequence::OnlyLong, 12),
        Err(Error::IcsInfoUnsupportedSampleRateIndex(12))
    ));
    assert!(matches!(
        tns_max_bands_ld_480(8),
        Err(Error::IcsInfoUnsupportedSampleRateIndex(8))
    ));
    assert!(matches!(
        tns_max_bands_ld_512(0),
        Err(Error::IcsInfoUnsupportedSampleRateIndex(0))
    ));
}

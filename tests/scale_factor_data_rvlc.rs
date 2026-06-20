//! Integration tests for the error-resilient `scale_factor_data()`
//! RVLC branch — ISO/IEC 14496-3 Table 4.53 / §4.6.16.2
//! (`aacScalefactorDataResilienceFlag == 1`).
//!
//! These exercise the public [`oxideav_aac::scale_factor_data::ErScaleFactorData`]
//! parse / write pair and the [`oxideav_aac::rvlc`] codebook primitives
//! through their crate-public API, complementing the in-module unit
//! tests. Each test builds an [`ErScaleFactorData`] in memory, writes
//! it against a matching `sfb_cb` codebook map, re-parses the byte
//! buffer, and asserts:
//!
//! 1. structural recovery (parsed == original);
//! 2. the §4.6.2.3.2 forward-decode invariant — RVLC and Huffman
//!    streams carrying the same DPCM deltas accumulate to identical
//!    absolute scalefactors;
//! 3. the §4.6.16.2.1 error-detection behaviour (forbidden
//!    codewords / length-field desync are rejected in-band).
//!
//! The only normative inputs are Tables 4.166 / 4.167 / 4.168
//! (transcribed verbatim in `src/rvlc.rs`) and the Table 4.53 RVLC
//! wire layout.

use oxideav_aac::ics_info::WindowSequence;
use oxideav_aac::rvlc::{
    rvlc_decode, rvlc_encode, rvlc_esc_decode, rvlc_esc_encode, RVLC_ESC_FLAG,
};
use oxideav_aac::scale_factor_data::{
    accumulate, ErScaleFactorData, ScaleFactorData, ScaleFactorEntry,
};
use oxideav_aac::section_data::{INTENSITY_HCB2, NOISE_HCB};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// Write `block` against `sfb_cb` / `window_sequence`, re-parse, and
/// assert structural equality.
fn roundtrip(block: &ErScaleFactorData, sfb_cb: &[Vec<u8>], ws: WindowSequence) {
    let mut w = BitWriter::new();
    block.write(&mut w, sfb_cb, ws).expect("ER write");
    let bytes = w.finish();
    let mut r = BitReader::new(&bytes);
    let parsed = ErScaleFactorData::parse(&mut r, sfb_cb, ws).expect("ER parse");
    assert_eq!(&parsed, block, "ER block did not round-trip");
}

#[test]
fn rvlc_primitives_roundtrip_through_public_api() {
    for value in -7i8..=7 {
        let (len, cw) = rvlc_encode(value).unwrap();
        let mut w = BitWriter::new();
        w.write_u32(cw, u32::from(len));
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(rvlc_decode(&mut r).unwrap(), value);
    }
    for mag in 0u8..=53 {
        let (len, cw) = rvlc_esc_encode(mag).unwrap();
        let mut w = BitWriter::new();
        w.write_u32(cw, u32::from(len));
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(rvlc_esc_decode(&mut r).unwrap(), mag);
    }
    assert_eq!(RVLC_ESC_FLAG, 7);
}

#[test]
fn er_spectrum_only_roundtrips() {
    let sfb_cb = vec![vec![2u8, 2, 2], vec![4u8, 4]];
    let block = ErScaleFactorData {
        sf_concealment: true,
        rev_global_gain: 144,
        data: ScaleFactorData {
            entries: vec![
                vec![
                    ScaleFactorEntry::Dpcm(0),
                    ScaleFactorEntry::Dpcm(6),
                    ScaleFactorEntry::Dpcm(-6),
                ],
                vec![ScaleFactorEntry::Dpcm(1), ScaleFactorEntry::Dpcm(-1)],
            ],
        },
        dpcm_is_last_position: None,
        dpcm_noise_last_position: None,
    };
    roundtrip(&block, &sfb_cb, WindowSequence::OnlyLong);
}

#[test]
fn er_escapes_span_full_dpcm_range() {
    // Deltas at the extremes of the -60..=+60 DPCM range force the
    // base-±7 + Table 4.168 escape split, including the longest
    // (20-bit) escape codewords near magnitude 53.
    let sfb_cb = vec![vec![3u8, 3, 3, 3]];
    let block = ErScaleFactorData {
        sf_concealment: false,
        rev_global_gain: 60,
        data: ScaleFactorData {
            entries: vec![vec![
                ScaleFactorEntry::Dpcm(7),
                ScaleFactorEntry::Dpcm(-7),
                ScaleFactorEntry::Dpcm(60),
                ScaleFactorEntry::Dpcm(-60),
            ]],
        },
        dpcm_is_last_position: None,
        dpcm_noise_last_position: None,
    };
    roundtrip(&block, &sfb_cb, WindowSequence::EightShort);
}

#[test]
fn er_intensity_and_pns_seeds_roundtrip() {
    let sfb_cb = vec![vec![2u8, INTENSITY_HCB2, NOISE_HCB, NOISE_HCB]];
    let block = ErScaleFactorData {
        sf_concealment: true,
        rev_global_gain: 110,
        data: ScaleFactorData {
            entries: vec![vec![
                ScaleFactorEntry::Dpcm(-2),
                ScaleFactorEntry::Intensity(8), // intensity escape (+7 + 1)
                ScaleFactorEntry::NoisePcm(0x0ff),
                ScaleFactorEntry::NoiseDpcm(-4),
            ]],
        },
        dpcm_is_last_position: Some(-9), // intensity-last escape (-7 - 2)
        dpcm_noise_last_position: Some(0x1ff),
    };
    roundtrip(&block, &sfb_cb, WindowSequence::OnlyLong);
}

#[test]
fn er_forward_decode_equals_huffman_path() {
    // The §4.6.2.3.2 headline invariant, end to end through the
    // public accumulate() API.
    let sfb_cb = vec![vec![2u8, 2, 2, 2, 2]];
    let global_gain = 128u8;
    let entries = vec![vec![
        ScaleFactorEntry::Dpcm(0),
        ScaleFactorEntry::Dpcm(4),
        ScaleFactorEntry::Dpcm(-40), // escape on the RVLC side
        ScaleFactorEntry::Dpcm(6),
        ScaleFactorEntry::Dpcm(-3),
    ]];

    // Huffman branch.
    let huff = ScaleFactorData {
        entries: entries.clone(),
    };
    let mut hw = BitWriter::new();
    huff.write(&mut hw, &sfb_cb).unwrap();
    let hbytes = hw.finish();
    let mut hr = BitReader::new(&hbytes);
    let hparsed = ScaleFactorData::parse(&mut hr, &sfb_cb).unwrap();
    let habs = accumulate(&hparsed, &sfb_cb, global_gain).unwrap();

    // RVLC branch.
    let er = ErScaleFactorData {
        sf_concealment: false,
        rev_global_gain: 0,
        data: ScaleFactorData { entries },
        dpcm_is_last_position: None,
        dpcm_noise_last_position: None,
    };
    let mut ew = BitWriter::new();
    er.write(&mut ew, &sfb_cb, WindowSequence::OnlyLong)
        .unwrap();
    let ebytes = ew.finish();
    let mut er_reader = BitReader::new(&ebytes);
    let eparsed =
        ErScaleFactorData::parse(&mut er_reader, &sfb_cb, WindowSequence::OnlyLong).unwrap();
    let eabs = accumulate(&eparsed.data, &sfb_cb, global_gain).unwrap();

    assert_eq!(
        habs, eabs,
        "RVLC forward decode must equal the Huffman path"
    );
}

#[test]
fn er_forbidden_codeword_is_rejected() {
    // Hand-assemble a header + one forbidden RVLC codeword (Table
    // 4.167, 6-bit 0b110010).
    let mut w = BitWriter::new();
    w.write_u32(0, 1); // sf_concealment
    w.write_u32(0, 8); // rev_global_gain
    w.write_u32(6, 9); // length_of_rvlc_sf = 6 bits
    w.write_u32(0b110010, 6);
    let bytes = w.finish();
    let sfb_cb = vec![vec![2u8]];
    let mut r = BitReader::new(&bytes);
    assert!(matches!(
        ErScaleFactorData::parse(&mut r, &sfb_cb, WindowSequence::OnlyLong),
        Err(Error::RvlcForbiddenCodeword)
    ));
}

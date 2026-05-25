//! Tests for [`oxideav_aac::asc::AudioSpecificConfig`] — synthetic
//! ASC buffers constructed via `oxideav_core::bits::BitWriter` so
//! the tests do not depend on any external AAC encoder.

use oxideav_aac::asc::{AudioSpecificConfig, FrameLength};
use oxideav_aac::Error;
use oxideav_core::bits::BitWriter;

/// Build a minimal AAC-LC ASC: outer AOT = 2, channel config, frame
/// length flag, no PCE. Returns the byte buffer.
fn build_lc_asc(sampling_frequency_index: u8, channel_configuration: u8) -> Vec<u8> {
    let mut bw = BitWriter::new();
    bw.write_u32(2, 5); // audioObjectType = AAC LC
    bw.write_u32(sampling_frequency_index as u32, 4);
    bw.write_u32(channel_configuration as u32, 4);
    // GASpecificConfig
    bw.write_bit(false); // frameLengthFlag = 0 → 1024
    bw.write_bit(false); // dependsOnCoreCoder = 0
    bw.write_bit(false); // extensionFlag = 0
    bw.finish()
}

#[test]
fn parses_aac_lc_stereo_44100() {
    let buf = build_lc_asc(4, 2);
    let (asc, bits) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.outer_aot, 2);
    assert_eq!(asc.aot, 2);
    assert_eq!(asc.sampling_frequency_index, 4);
    assert_eq!(asc.sample_rate, 44100);
    assert_eq!(asc.channel_configuration, 2);
    assert_eq!(asc.channel_count(), 2);
    assert!(!asc.sbr_present);
    assert!(!asc.ps_present);
    assert!(asc.extension_sampling_frequency_index.is_none());
    assert_eq!(asc.ga_body.frame_length, FrameLength::Long1024);
    assert!(!asc.ga_body.depends_on_core_coder);
    assert!(asc.ga_body.core_coder_delay.is_none());
    assert!(!asc.ga_body.extension_flag);
    assert!(asc.ga_body.pce.is_none());
    // 5+4+4+1+1+1 = 16 bits = 2 bytes
    assert_eq!(bits, 16);
}

#[test]
fn parses_mpeg4_main_with_core_coder_delay() {
    // AOT 1 (Main), 48 kHz, mono, dependsOnCoreCoder = 1.
    let mut bw = BitWriter::new();
    bw.write_u32(1, 5); // AOT = Main
    bw.write_u32(3, 4); // sfi = 48000
    bw.write_u32(1, 4); // mono
    bw.write_bit(false); // frameLengthFlag = 0
    bw.write_bit(true); // dependsOnCoreCoder = 1
    bw.write_u32(0x1234, 14); // coreCoderDelay
    bw.write_bit(false); // extensionFlag = 0
    let buf = bw.finish();

    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.aot, 1);
    assert_eq!(asc.sample_rate, 48000);
    assert!(asc.ga_body.depends_on_core_coder);
    assert_eq!(asc.ga_body.core_coder_delay, Some(0x1234));
}

#[test]
fn frame_length_flag_selects_960() {
    let mut bw = BitWriter::new();
    bw.write_u32(2, 5);
    bw.write_u32(4, 4);
    bw.write_u32(2, 4);
    bw.write_bit(true); // frameLengthFlag = 1 → 960
    bw.write_bit(false);
    bw.write_bit(false);
    let buf = bw.finish();
    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.ga_body.frame_length, FrameLength::Long960);
    assert_eq!(asc.ga_body.frame_length.samples(), 960);
}

#[test]
fn aot_escape_31_unlocks_extended_range() {
    // audioObjectType escape: base = 31, ext = 11 → AOT 43 (not
    // defined as a GA AOT, so the body dispatch rejects with
    // UnsupportedAot — but the escape-path read should succeed and
    // surface the right AOT in the error).
    let mut bw = BitWriter::new();
    bw.write_u32(31, 5); // escape
    bw.write_u32(11, 6); // ext = 11 → AOT = 32 + 11 = 43
    bw.write_u32(4, 4);
    bw.write_u32(2, 4);
    let buf = bw.finish();
    assert_eq!(
        AudioSpecificConfig::parse(&buf),
        Err(Error::UnsupportedAot(43))
    );
}

#[test]
fn rejects_reserved_sampling_frequency_index() {
    let buf = build_lc_asc(13, 2);
    assert_eq!(
        AudioSpecificConfig::parse(&buf),
        Err(Error::AdtsReservedSampleRateIndex)
    );
}

#[test]
fn parses_24bit_escape_sample_rate() {
    // sampling_frequency_index = 0xf → 24-bit explicit sample rate.
    let mut bw = BitWriter::new();
    bw.write_u32(2, 5); // AOT = LC
    bw.write_u32(0xf, 4); // sfi escape
    bw.write_u32(192_000, 24); // explicit rate
    bw.write_u32(2, 4); // stereo
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    let buf = bw.finish();

    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.sampling_frequency_index, 0xf);
    assert_eq!(asc.sample_rate, 192_000);
}

#[test]
fn parses_he_aac_v1_explicit_sbr() {
    // Outer AOT = 5 (SBR). Inner AOT = 2 (AAC LC).
    let mut bw = BitWriter::new();
    bw.write_u32(5, 5); // outer audioObjectType = SBR
    bw.write_u32(7, 4); // core sfi = 22050
    bw.write_u32(2, 4); // chan_cfg = stereo
    bw.write_u32(4, 4); // extension_sampling_frequency_index = 44100
    bw.write_u32(2, 5); // inner audioObjectType = LC
                        // GASpecificConfig (inner)
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    let buf = bw.finish();

    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.outer_aot, 5);
    assert_eq!(asc.aot, 2);
    assert!(asc.sbr_present);
    assert!(!asc.ps_present);
    assert_eq!(asc.sample_rate, 22050);
    assert_eq!(asc.extension_sample_rate, Some(44100));
    assert_eq!(asc.extension_sampling_frequency_index, Some(4));
}

#[test]
fn parses_he_aac_v2_explicit_ps() {
    // Outer AOT = 29 (PS). Inner AOT = 2 (AAC LC). Core mono ↑ SBR
    // stereo at 32 kHz via PS.
    let mut bw = BitWriter::new();
    bw.write_u32(29, 5); // outer = PS
    bw.write_u32(8, 4); // core sfi = 16000
    bw.write_u32(1, 4); // chan_cfg = mono
    bw.write_u32(5, 4); // extension sfi = 32000
    bw.write_u32(2, 5); // inner aot = LC
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    let buf = bw.finish();

    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.outer_aot, 29);
    assert_eq!(asc.aot, 2);
    assert!(asc.sbr_present);
    assert!(asc.ps_present);
    assert_eq!(asc.sample_rate, 16000);
    assert_eq!(asc.extension_sample_rate, Some(32000));
}

#[test]
fn channel_configuration_zero_requires_inline_pce() {
    // ASC with chan_cfg = 0 — GASpecificConfig must carry an inline
    // PCE. Build a minimal PCE (all element counts zero, no comment
    // field) to satisfy the requirement.
    let mut bw = BitWriter::new();
    bw.write_u32(2, 5); // AOT = LC
    bw.write_u32(4, 4); // sfi = 44100
    bw.write_u32(0, 4); // chan_cfg = 0 (PCE-defined)
                        // GASpecificConfig start
    bw.write_bit(false); // frameLengthFlag
    bw.write_bit(false); // dependsOnCoreCoder
    bw.write_bit(false); // extensionFlag
                         // Inline PCE
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_u32(1, 2); // object_type = LC
    bw.write_u32(4, 4); // sampling_frequency_index
    bw.write_u32(0, 4); // n_front
    bw.write_u32(0, 4); // n_side
    bw.write_u32(0, 4); // n_back
    bw.write_u32(0, 2); // n_lfe
    bw.write_u32(0, 3); // n_assoc
    bw.write_u32(0, 4); // n_cc
    bw.write_bit(false); // mono_mixdown_present
    bw.write_bit(false); // stereo_mixdown_present
    bw.write_bit(false); // matrix_mixdown_present
    bw.align_to_byte_zero(); // PCE Note 1 byte_alignment() — ASC starts at bit 0
    bw.write_u32(0, 8); // comment_field_bytes = 0
    let buf = bw.finish();

    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.channel_configuration, 0);
    assert!(asc.ga_body.pce.is_some());
    let pce = asc.ga_body.pce.as_ref().unwrap();
    assert_eq!(pce.object_type, 1);
    assert_eq!(pce.sampling_frequency_index, 4);
    assert_eq!(asc.channel_count(), 0); // empty PCE → 0 channels
}

#[test]
fn channel_count_resolves_table_1_19() {
    let cases = [(1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 8)];
    for (chan_cfg, want) in cases {
        let buf = build_lc_asc(4, chan_cfg);
        let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
        assert_eq!(
            asc.channel_count(),
            want,
            "channel_configuration = {}",
            chan_cfg
        );
    }
}

#[test]
fn unknown_aot_returns_unsupported_aot() {
    // AOT 8 (CELP) — not GA, should be rejected.
    let mut bw = BitWriter::new();
    bw.write_u32(8, 5);
    bw.write_u32(4, 4);
    bw.write_u32(2, 4);
    let buf = bw.finish();
    assert_eq!(
        AudioSpecificConfig::parse(&buf),
        Err(Error::UnsupportedAot(8))
    );
}

#[test]
fn rejects_truncated_asc() {
    let buf = vec![0x12]; // 1 byte — can't even hold the 9-bit (5+4) AOT+sfi pair completely.
    assert_eq!(AudioSpecificConfig::parse(&buf), Err(Error::UnexpectedEnd));
}

#[test]
fn scalable_aot_6_has_layer_nr() {
    // AOT 6 (AAC scalable) — GA body adds a 3-bit layerNr.
    let mut bw = BitWriter::new();
    bw.write_u32(6, 5);
    bw.write_u32(4, 4);
    bw.write_u32(2, 4);
    bw.write_bit(false); // frameLengthFlag
    bw.write_bit(false); // dependsOnCoreCoder
    bw.write_bit(false); // extensionFlag (AOT 6 → must be 0)
    bw.write_u32(5, 3); // layerNr
    let buf = bw.finish();

    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.aot, 6);
    assert_eq!(asc.ga_body.layer_nr, Some(5));
}

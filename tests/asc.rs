//! Tests for [`oxideav_aac::asc::AudioSpecificConfig`] — synthetic
//! ASC buffers constructed via `oxideav_core::bits::BitWriter` so
//! the tests do not depend on any external AAC encoder.

use oxideav_aac::asc::{AacResilienceFlags, AudioSpecificConfig, BsacLayerSpec, FrameLength};
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

// ---------------------------------------------------------------------
// Round 177 — GASpecificConfig extensionFlag body + epConfig coverage.
// ---------------------------------------------------------------------

#[test]
fn lc_aac_default_has_no_extension_body_and_no_ep_config() {
    // Regression: AOT 2 (LC) has extension_flag = 0 (shall be 0 per
    // Table 4.1 for AOTs 1, 2, 3, 4, 6, 7) and is not in the
    // EP_CONFIG_AOTS list, so both new fields stay `None`.
    let buf = build_lc_asc(4, 2);
    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert!(asc.ga_body.extension_body.is_none());
    assert_eq!(asc.ep_config, None);
}

/// Build an ER AOT 17 (ER AAC LC) ASC with the resilience triplet
/// + extensionFlag3 = 0 + epConfig.
fn build_er_aac_lc_asc(
    sfi: u8,
    chan_cfg: u8,
    section: bool,
    scalefactor: bool,
    spectral: bool,
    ep_config: u32,
) -> Vec<u8> {
    let mut bw = BitWriter::new();
    bw.write_u32(17, 5); // outer AOT = ER AAC LC
    bw.write_u32(sfi as u32, 4);
    bw.write_u32(chan_cfg as u32, 4);
    // GASpecificConfig
    bw.write_bit(false); // frameLengthFlag
    bw.write_bit(false); // dependsOnCoreCoder
    bw.write_bit(true); // extensionFlag = 1 (shall be 1 for ER AOTs)
                        // Extension body: no numOfSubFrame (AOT != 22),
                        // resilience triplet for AOTs ∈ {17,19,20,23}.
    bw.write_bit(section);
    bw.write_bit(scalefactor);
    bw.write_bit(spectral);
    bw.write_bit(false); // extensionFlag3
                         // Tail epConfig — 2 bits.
    bw.write_u32(ep_config, 2);
    bw.finish()
}

#[test]
fn er_aac_lc_parses_resilience_triplet_and_ep_config() {
    let buf = build_er_aac_lc_asc(4, 2, true, false, true, 0);
    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.outer_aot, 17);
    assert_eq!(asc.aot, 17);
    assert!(asc.ga_body.extension_flag);
    let body = asc.ga_body.extension_body.as_ref().unwrap();
    assert_eq!(
        body.resilience,
        Some(AacResilienceFlags {
            section_data: true,
            scalefactor_data: false,
            spectral_data: true,
        })
    );
    assert!(body.bsac_layer.is_none());
    assert!(!body.extension_flag3);
    assert_eq!(asc.ep_config, Some(0));
}

#[test]
fn er_aac_lc_ep_config_one_is_accepted() {
    // epConfig == 1 routes via the EP class mapping table without an
    // inline ErrorProtectionSpecificConfig body.
    let buf = build_er_aac_lc_asc(4, 2, false, false, false, 1);
    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.ep_config, Some(1));
}

#[test]
fn er_aac_lc_ep_config_two_is_rejected() {
    let buf = build_er_aac_lc_asc(4, 2, false, false, false, 2);
    assert_eq!(
        AudioSpecificConfig::parse(&buf),
        Err(Error::UnsupportedEpConfig(2))
    );
}

#[test]
fn er_aac_lc_ep_config_three_is_rejected() {
    let buf = build_er_aac_lc_asc(4, 2, false, false, false, 3);
    assert_eq!(
        AudioSpecificConfig::parse(&buf),
        Err(Error::UnsupportedEpConfig(3))
    );
}

#[test]
fn er_bsac_aot_22_has_num_of_sub_frame_and_layer_length() {
    // AOT 22 (ER BSAC) — extension body adds 5-bit numOfSubFrame +
    // 11-bit layer_length BEFORE the resilience triplet is *not*
    // emitted (Table 4.1 gates the resilience triplet on AOTs
    // {17, 19, 20, 23} which does not include 22).
    let mut bw = BitWriter::new();
    bw.write_u32(22, 5); // outer AOT = ER BSAC
    bw.write_u32(4, 4); // sfi = 44100
    bw.write_u32(0, 4); // chan_cfg = 0 (BSAC carries an inline PCE)
                        // GASpecificConfig
    bw.write_bit(false); // frameLengthFlag
    bw.write_bit(false); // dependsOnCoreCoder
    bw.write_bit(true); // extensionFlag = 1
                        // Inline PCE because chan_cfg == 0.
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_u32(1, 2); // object_type
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
    bw.align_to_byte_zero(); // PCE byte_alignment()
    bw.write_u32(0, 8); // comment_field_bytes
                        // Extension body:
    bw.write_u32(7, 5); // numOfSubFrame
    bw.write_u32(0x123, 11); // layer_length
    bw.write_bit(false); // extensionFlag3
                         // epConfig = 0.
    bw.write_u32(0, 2);
    let buf = bw.finish();

    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    let body = asc.ga_body.extension_body.as_ref().unwrap();
    assert_eq!(
        body.bsac_layer,
        Some(BsacLayerSpec {
            num_of_sub_frame: 7,
            layer_length: 0x123,
        })
    );
    // AOT 22 is NOT in the resilience triplet list.
    assert!(body.resilience.is_none());
    assert!(!body.extension_flag3);
    assert_eq!(asc.ep_config, Some(0));
}

#[test]
fn er_aac_ld_aot_23_emits_resilience_triplet_only() {
    // AOT 23 (ER AAC LD) — extension body emits the resilience
    // triplet but NOT the numOfSubFrame pair (which is AOT 22 only).
    let mut bw = BitWriter::new();
    bw.write_u32(23, 5);
    bw.write_u32(4, 4);
    bw.write_u32(1, 4);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(true); // extensionFlag = 1
    bw.write_bit(true); // section
    bw.write_bit(true); // scalefactor
    bw.write_bit(false); // spectral
    bw.write_bit(false); // extensionFlag3
    bw.write_u32(0, 2); // epConfig
    let buf = bw.finish();

    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    let body = asc.ga_body.extension_body.as_ref().unwrap();
    assert!(body.bsac_layer.is_none());
    assert_eq!(
        body.resilience,
        Some(AacResilienceFlags {
            section_data: true,
            scalefactor_data: true,
            spectral_data: false,
        })
    );
    assert_eq!(asc.ep_config, Some(0));
}

#[test]
fn er_aac_scalable_aot_20_emits_both_layer_nr_and_resilience() {
    // AOT 20 sits in BOTH the layerNr list (AOTs {6, 20}) and the
    // resilience-triplet list ({17, 19, 20, 23}). Verifies the
    // ordering: layerNr is BEFORE the extension body in Table 4.1.
    let mut bw = BitWriter::new();
    bw.write_u32(20, 5);
    bw.write_u32(4, 4);
    bw.write_u32(2, 4);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(true); // extensionFlag = 1
    bw.write_u32(3, 3); // layerNr
                        // Extension body resilience triplet.
    bw.write_bit(false); // section
    bw.write_bit(true); // scalefactor
    bw.write_bit(false); // spectral
    bw.write_bit(false); // extensionFlag3
    bw.write_u32(1, 2); // epConfig
    let buf = bw.finish();

    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(asc.ga_body.layer_nr, Some(3));
    let body = asc.ga_body.extension_body.as_ref().unwrap();
    assert!(body.bsac_layer.is_none());
    assert_eq!(
        body.resilience,
        Some(AacResilienceFlags {
            section_data: false,
            scalefactor_data: true,
            spectral_data: false,
        })
    );
    assert_eq!(asc.ep_config, Some(1));
}

#[test]
fn er_aac_extension_flag3_set_is_rejected() {
    // extensionFlag3 == 1 means "tbd in version 3" — the body
    // layout is undefined, so the parser cannot continue.
    let mut bw = BitWriter::new();
    bw.write_u32(17, 5);
    bw.write_u32(4, 4);
    bw.write_u32(2, 4);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(true);
    bw.write_bit(false); // section
    bw.write_bit(false); // scalefactor
    bw.write_bit(false); // spectral
    bw.write_bit(true); // extensionFlag3 = 1 → reject
    let buf = bw.finish();

    assert_eq!(
        AudioSpecificConfig::parse(&buf),
        Err(Error::UnsupportedAscExtensionFlag3)
    );
}

#[test]
fn er_aac_lc_truncated_in_resilience_body_is_unexpected_end() {
    // Build a complete frame, then truncate the buffer mid-resilience.
    // Layout reaches bit 16 (AOT/sfi/chan/3 GA flags) — we cut to a
    // single byte so only the 5-bit AOT + 3 bits of sfi survive.
    let mut bw = BitWriter::new();
    bw.write_u32(17, 5);
    bw.write_u32(4, 4);
    bw.write_u32(2, 4);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(true); // extensionFlag
    bw.write_bit(true); // section
    bw.write_bit(true); // scalefactor
    bw.write_bit(true); // spectral
    bw.write_bit(false); // extensionFlag3
    bw.write_u32(0, 2); // epConfig
    let mut buf = bw.finish();
    buf.truncate(2); // chops out everything past bit 16 — mid extension body.

    assert_eq!(AudioSpecificConfig::parse(&buf), Err(Error::UnexpectedEnd));
}

#[test]
fn er_aac_lc_truncated_at_ep_config_is_unexpected_end() {
    // Build a complete frame, truncate at the byte boundary that
    // sits *before* the epConfig field. The frame normally takes 22
    // bits → 3 bytes; truncating to 2 bytes leaves the parser with
    // only the first 16 bits (AOT/sfi/chan + 3 GA flags). All
    // subsequent reads fail at the body bit-reader.
    let mut bw = BitWriter::new();
    bw.write_u32(17, 5);
    bw.write_u32(4, 4);
    bw.write_u32(2, 4);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(true);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_u32(0, 2);
    let mut buf = bw.finish();
    buf.truncate(2);

    assert_eq!(AudioSpecificConfig::parse(&buf), Err(Error::UnexpectedEnd));
}

#[test]
fn er_twinvq_aot_21_has_ep_config_but_no_extension_subfields() {
    // AOT 21 (ER TwinVQ) is in EP_CONFIG_AOTS but NOT in either
    // {17, 19, 20, 23} (resilience) or {22} (BSAC), so the
    // extension body collapses to extensionFlag3 + epConfig only.
    let mut bw = BitWriter::new();
    bw.write_u32(21, 5);
    bw.write_u32(4, 4);
    bw.write_u32(2, 4);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(true); // extensionFlag = 1
    bw.write_bit(false); // extensionFlag3
    bw.write_u32(0, 2); // epConfig
    let buf = bw.finish();

    let (asc, _) = AudioSpecificConfig::parse(&buf).unwrap();
    let body = asc.ga_body.extension_body.as_ref().unwrap();
    assert!(body.bsac_layer.is_none());
    assert!(body.resilience.is_none());
    assert!(!body.extension_flag3);
    assert_eq!(asc.ep_config, Some(0));
}

#[test]
fn er_aac_lc_bit_position_advances_through_extension_and_ep_config() {
    // Sanity that the parser consumes the right number of bits.
    // Layout: 5 + 4 + 4 (= 13) AOT/sfi/chan_cfg
    //       + 1 + 1 + 1 (= 16) frameLengthFlag/dependsOn/extensionFlag
    //       + 1 + 1 + 1 (= 19) resilience triplet
    //       + 1       (= 20) extensionFlag3
    //       + 2       (= 22) epConfig
    let buf = build_er_aac_lc_asc(4, 2, false, false, false, 0);
    let (_asc, bits) = AudioSpecificConfig::parse(&buf).unwrap();
    assert_eq!(bits, 22);
}

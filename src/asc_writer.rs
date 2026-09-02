//! `AudioSpecificConfig()` writer — ISO/IEC 14496-3 Table 1.15 written
//! forward for the configurations this crate's encoders produce.
//!
//! The inverse of [`crate::asc::AudioSpecificConfig::parse`] for the
//! GA object types with a plain `GASpecificConfig` (Table 4.1:
//! `frameLengthFlag = 0`, `dependsOnCoreCoder = 0`,
//! `extensionFlag = 0`), plus the two HE-AAC v1 signalling forms of
//! §1.6.5 / §1.6.6:
//!
//! * **Backward compatible** — the AAC-LC ASC followed by the Table
//!   1.15 trailer `syncExtensionType (11) = 0x2b7`,
//!   `extensionAudioObjectType (5) = 5`, `sbrPresentFlag (1) = 1`,
//!   `extensionSamplingFrequencyIndex (4)` (+ the 24-bit escape when
//!   the rate is not a Table 1.18 index). A decoder unaware of SBR
//!   stops after `GASpecificConfig()` and plays the core.
//! * **Hierarchical** — `audioObjectType = 5` first, then
//!   `extensionSamplingFrequencyIndex`, then the core
//!   `audioObjectType = 2` and its `GASpecificConfig()`.
//!
//! and the two HE-AAC v2 forms of §1.6.6 ([`he_aac_v2_asc`]): the
//! backward-compatible SBR trailer followed by `syncExtensionType
//! 0x548` + `psPresentFlag`, or the hierarchical `audioObjectType =
//! 29` wrapper.
//!
//! Every writer pads to a whole byte with zero bits (the ASC is carried
//! as bytes in MP4 `esds` / LATM `StreamMuxConfig`).

use oxideav_core::bits::BitWriter;

use crate::adts::ADTS_SAMPLE_RATES_HZ;

/// The Table 1.18 `samplingFrequencyIndex` for `rate`, or `0xf` with
/// the 24-bit escape value.
fn write_sampling_frequency(w: &mut BitWriter, rate: u32) {
    match ADTS_SAMPLE_RATES_HZ.iter().position(|&r| r == rate) {
        Some(i) => w.write_u32(i as u32, 4),
        None => {
            w.write_u32(0xf, 4);
            w.write_u32(rate & 0x00ff_ffff, 24);
        }
    }
}

/// `GetAudioObjectType()`: 5 bits, or `31` + 6-bit escape for
/// `aot ≥ 32`.
fn write_aot(w: &mut BitWriter, aot: u8) {
    if aot < 32 {
        w.write_u32(u32::from(aot), 5);
    } else {
        w.write_u32(31, 5);
        w.write_u32(u32::from(aot - 32), 6);
    }
}

/// The plain `GASpecificConfig()` (Table 4.1) for a 1024-line,
/// non-scalable, extension-free object type.
fn write_ga_specific_config(w: &mut BitWriter) {
    w.write_bit(false); // frameLengthFlag = 0 (1024)
    w.write_bit(false); // dependsOnCoreCoder = 0
    w.write_bit(false); // extensionFlag = 0
}

/// The AAC-LC (`audioObjectType = 2`) `AudioSpecificConfig` for
/// `sample_rate` / `channel_configuration` (Table 1.19 value).
pub fn aac_lc_asc(sample_rate: u32, channel_configuration: u8) -> Vec<u8> {
    let mut w = BitWriter::new();
    write_aot(&mut w, 2);
    write_sampling_frequency(&mut w, sample_rate);
    w.write_u32(u32::from(channel_configuration & 0xf), 4);
    write_ga_specific_config(&mut w);
    w.align_to_byte_zero();
    w.finish()
}

/// The HE-AAC v1 `AudioSpecificConfig`: AAC-LC core at `core_rate`
/// with SBR output at `sbr_rate`, in the backward-compatible
/// (`hierarchical == false`) or hierarchical form.
pub fn he_aac_v1_asc(
    core_rate: u32,
    sbr_rate: u32,
    channel_configuration: u8,
    hierarchical: bool,
) -> Vec<u8> {
    let mut w = BitWriter::new();
    if hierarchical {
        write_aot(&mut w, 5);
        write_sampling_frequency(&mut w, core_rate);
        w.write_u32(u32::from(channel_configuration & 0xf), 4);
        write_sampling_frequency(&mut w, sbr_rate);
        write_aot(&mut w, 2);
        write_ga_specific_config(&mut w);
    } else {
        write_aot(&mut w, 2);
        write_sampling_frequency(&mut w, core_rate);
        w.write_u32(u32::from(channel_configuration & 0xf), 4);
        write_ga_specific_config(&mut w);
        w.write_u32(u32::from(crate::asc::SYNC_EXTENSION_TYPE_SBR), 11);
        write_aot(&mut w, 5);
        w.write_bit(true); // sbrPresentFlag
        write_sampling_frequency(&mut w, sbr_rate);
    }
    w.align_to_byte_zero();
    w.finish()
}

/// The HE-AAC v2 `AudioSpecificConfig` (§1.6.6): a single-channel
/// AAC-LC core at `core_rate`, SBR output at `sbr_rate` and the
/// parametric stereo tool, in the backward-compatible form
/// (`hierarchical == false`: the AAC-LC ASC, the `0x2b7` SBR trailer
/// with `sbrPresentFlag = 1`, then `syncExtensionType 0x548` +
/// `psPresentFlag = 1` — 49 bits) or the hierarchical form
/// (`audioObjectType = 29` up front, `extensionSamplingFrequencyIndex`,
/// the core `audioObjectType = 2` — 25 bits; the form LATM with
/// `audioMuxVersion == 0` requires, since it conveys no ASC length).
///
/// `channelConfiguration` is always 1: it describes the underlying
/// AAC stream, the PS tool producing the two output channels.
pub fn he_aac_v2_asc(core_rate: u32, sbr_rate: u32, hierarchical: bool) -> Vec<u8> {
    let mut w = BitWriter::new();
    if hierarchical {
        write_aot(&mut w, 29);
        write_sampling_frequency(&mut w, core_rate);
        w.write_u32(1, 4);
        write_sampling_frequency(&mut w, sbr_rate);
        write_aot(&mut w, 2);
        write_ga_specific_config(&mut w);
    } else {
        write_aot(&mut w, 2);
        write_sampling_frequency(&mut w, core_rate);
        w.write_u32(1, 4);
        write_ga_specific_config(&mut w);
        w.write_u32(u32::from(crate::asc::SYNC_EXTENSION_TYPE_SBR), 11);
        write_aot(&mut w, 5);
        w.write_bit(true); // sbrPresentFlag
        write_sampling_frequency(&mut w, sbr_rate);
        w.write_u32(u32::from(crate::asc::SYNC_EXTENSION_TYPE_PS), 11);
        w.write_bit(true); // psPresentFlag
    }
    w.align_to_byte_zero();
    w.finish()
}

/// The exact syntax bit count of an ASC this module writes (for
/// length-prefixed carriers such as [`crate::latm_writer`]): the rates
/// must be Table 1.18 indices (an escaped rate adds 24 bits each).
#[must_use]
pub fn asc_bits(sbr: bool, ps: bool, hierarchical: bool) -> u32 {
    match (sbr, ps, hierarchical) {
        (false, _, _) => 16,
        (true, _, true) => 25,
        (true, false, false) => 37,
        (true, true, false) => 49,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::asc::AudioSpecificConfig;

    #[test]
    fn he_aac_v2_ascs_signal_ps_both_ways() {
        // Backward compatible: AAC-LC + SBR trailer + PS trailer.
        let bytes = he_aac_v2_asc(24_000, 48_000, false);
        assert_eq!(bytes.len(), 7);
        let (asc, _) = AudioSpecificConfig::parse(&bytes).unwrap();
        assert_eq!(asc.outer_aot, 2);
        assert_eq!(asc.aot, 2);
        assert_eq!(asc.channel_configuration, 1);
        assert_eq!(asc.sample_rate, 24_000);
        let probe = asc.trailing_sbr_probe.expect("0x2b7 trailer");
        assert!(probe.sbr_present_flag);
        assert_eq!(probe.extension_sample_rate, Some(48_000));
        assert_eq!(probe.ps_present_flag, Some(true));
        assert!(asc.ps_present);
        // Hierarchical: AOT 29 wrapper.
        let bytes = he_aac_v2_asc(22_050, 44_100, true);
        assert_eq!(bytes.len(), 4);
        let (asc, _) = AudioSpecificConfig::parse(&bytes).unwrap();
        assert_eq!(asc.outer_aot, 29);
        assert_eq!(asc.aot, 2);
        assert!(asc.sbr_present);
        assert!(asc.ps_present);
        assert_eq!(asc.sample_rate, 22_050);
        assert_eq!(asc.extension_sample_rate, Some(44_100));
        assert_eq!(asc.channel_configuration, 1);
        // Bit counts.
        assert_eq!(asc_bits(false, false, false), 16);
        assert_eq!(asc_bits(true, false, true), 25);
        assert_eq!(asc_bits(true, false, false), 37);
        assert_eq!(asc_bits(true, true, false), 49);
        assert_eq!(asc_bits(true, true, true), 25);
    }

    #[test]
    fn aac_lc_asc_is_the_canonical_two_bytes() {
        // 44.1 kHz stereo AAC-LC: 0x12 0x10.
        assert_eq!(aac_lc_asc(44_100, 2), vec![0x12, 0x10]);
        let (asc, _) = AudioSpecificConfig::parse(&aac_lc_asc(48_000, 1)).unwrap();
        assert_eq!(asc.aot, 2);
        assert_eq!(asc.sample_rate, 48_000);
        assert_eq!(asc.channel_configuration, 1);
        assert!(!asc.sbr_present);
    }

    #[test]
    fn backward_compatible_he_asc_parses_with_sbr_probe() {
        let bytes = he_aac_v1_asc(22_050, 44_100, 2, false);
        assert_eq!(bytes.len(), 5);
        let (asc, _) = AudioSpecificConfig::parse(&bytes).unwrap();
        assert_eq!(asc.outer_aot, 2);
        assert_eq!(asc.aot, 2);
        assert_eq!(asc.sample_rate, 22_050);
        let probe = asc.trailing_sbr_probe.expect("0x2b7 trailer");
        assert_eq!(probe.extension_audio_object_type, 5);
        assert!(probe.sbr_present_flag);
        assert_eq!(probe.extension_sample_rate, Some(44_100));
    }

    #[test]
    fn hierarchical_he_asc_parses_as_aot_5() {
        let bytes = he_aac_v1_asc(24_000, 48_000, 1, true);
        let (asc, _) = AudioSpecificConfig::parse(&bytes).unwrap();
        assert_eq!(asc.outer_aot, 5);
        assert_eq!(asc.aot, 2);
        assert!(asc.sbr_present);
        assert_eq!(asc.sample_rate, 24_000);
        assert_eq!(asc.extension_sample_rate, Some(48_000));
        assert_eq!(asc.channel_configuration, 1);
    }

    #[test]
    fn escaped_rates_round_trip() {
        let bytes = he_aac_v1_asc(22_050, 44_100 + 2, 2, true);
        let (asc, _) = AudioSpecificConfig::parse(&bytes).unwrap();
        assert_eq!(asc.extension_sample_rate, Some(44_102));
    }
}

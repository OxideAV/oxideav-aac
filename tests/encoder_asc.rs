//! Acceptance test for `*Encoder::audio_specific_config()`.
//!
//! Each encoder variant exposes an `audio_specific_config()` helper
//! that returns the ASC blob a downstream MP4 muxer needs in `esds`
//! (or a DASH manifest needs in the `codecs` parameter). This test
//! round-trips the ASC bytes through `parse_asc` and asserts the
//! parsed view matches the encoder's runtime configuration: the
//! profile flag (LC vs HE-AAC vs HE-AAC v2), sample-rate pair, and
//! channel count.

use oxideav_aac::asc::parse_asc;
use oxideav_aac::encoder::AacEncoder;
use oxideav_aac::he_aac_encoder::{HeAacMonoEncoder, HeAacStereoEncoder, HeAacV2Encoder};
use oxideav_core::{CodecId, CodecParameters};

fn lc_params(sr: u32, ch: u16) -> CodecParameters {
    let mut p = CodecParameters::audio(CodecId::new("aac"));
    p.sample_rate = Some(sr);
    p.channels = Some(ch);
    p.bit_rate = Some(128_000);
    p
}

#[test]
fn aac_lc_encoder_emits_round_trippable_asc() {
    let enc = AacEncoder::new(&lc_params(44_100, 2)).expect("ctor");
    let asc_bytes = enc.audio_specific_config();
    assert_eq!(asc_bytes.len(), 2);
    let asc = parse_asc(&asc_bytes).expect("parse");
    assert_eq!(asc.object_type, oxideav_aac::syntax::AOT_AAC_LC);
    assert_eq!(asc.sampling_frequency, 44_100);
    assert_eq!(asc.channel_configuration, 2);
    assert!(!asc.sbr_present);
    assert!(!asc.ps_present);
}

#[test]
fn aac_lc_encoder_asc_picks_up_5_1() {
    let enc = AacEncoder::new(&lc_params(48_000, 6)).expect("ctor");
    let asc = parse_asc(&enc.audio_specific_config()).expect("parse");
    assert_eq!(
        asc.channel_configuration, 6,
        "5.1 → channel_configuration=6"
    );
    assert_eq!(asc.channel_count(), 6);
}

#[test]
fn aac_lc_encoder_asc_picks_up_7_1() {
    let enc = AacEncoder::new(&lc_params(48_000, 8)).expect("ctor");
    let asc = parse_asc(&enc.audio_specific_config()).expect("parse");
    assert_eq!(
        asc.channel_configuration, 7,
        "7.1 → channel_configuration=7"
    );
    assert_eq!(asc.channel_count(), 8);
}

#[test]
fn he_aac_mono_encoder_emits_explicit_aot5_asc() {
    let mut p = lc_params(48_000, 1); // high rate (= 2 * core)
    p.bit_rate = Some(32_000);
    let enc = HeAacMonoEncoder::new(&p).expect("ctor");
    let asc_bytes = enc.audio_specific_config();
    let asc = parse_asc(&asc_bytes).expect("parse");
    assert!(asc.sbr_present, "explicit AOT_SBR must surface sbr_present");
    assert!(!asc.ps_present);
    assert_eq!(asc.ext_sampling_frequency, Some(48_000));
    assert_eq!(asc.channel_configuration, 1);
}

#[test]
fn he_aac_stereo_encoder_emits_explicit_aot5_asc() {
    let mut p = lc_params(48_000, 2);
    p.bit_rate = Some(64_000);
    let enc = HeAacStereoEncoder::new(&p).expect("ctor");
    let asc = parse_asc(&enc.audio_specific_config()).expect("parse");
    assert!(asc.sbr_present);
    assert!(!asc.ps_present);
    assert_eq!(asc.ext_sampling_frequency, Some(48_000));
    assert_eq!(asc.channel_configuration, 2);
}

#[test]
fn he_aac_v2_encoder_emits_explicit_aot29_asc() {
    let mut p = lc_params(44_100, 2); // input is stereo, downmixed to mono SBR
    p.bit_rate = Some(24_000);
    let enc = HeAacV2Encoder::new(&p).expect("ctor");
    let asc = parse_asc(&enc.audio_specific_config()).expect("parse");
    assert!(asc.sbr_present);
    assert!(asc.ps_present, "PS-bearing encoder must surface ps_present");
    assert_eq!(asc.ext_sampling_frequency, Some(44_100));
    assert_eq!(
        asc.channel_configuration, 1,
        "PS payload mandates mono inner core (channel_configuration=1)"
    );
}

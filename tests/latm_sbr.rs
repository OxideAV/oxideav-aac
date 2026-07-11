//! HE-AAC v1 over LATM/LOAS — the §1.7.3 transport must carry an
//! SBR-signalling `AudioSpecificConfig` (explicit AOT-5 hierarchical
//! wrapper *or* implicit AAC-LC-only signalling) straight into the
//! shared §4.6.18 SBR back-end, byte-identical to the bare ADTS path.
//!
//! The tests re-multiplex the staged HE-AAC v1 ADTS fixture: each
//! `raw_data_block()` payload is lifted out of its ADTS frame and
//! wrapped in a LOAS `AudioSyncStream()` frame (11-bit `0x2B7` sync +
//! 13-bit `audioMuxLengthBytes` + `AudioMuxElement(1)`). The first
//! element carries the inline `StreamMuxConfig`; every later element
//! sets `useSameStreamMux`. Because the payload bits are identical,
//! [`LoasDecoder`] must reproduce the [`StreamDecoder`] ADTS output
//! *exactly* — the transport adds no arithmetic of its own — so the
//! comparison is byte-equality, not an RMS bound.
//!
//! Skips (logged, success) when `docs/` is absent (standalone CI).

use std::fs;
use std::path::PathBuf;

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::latm::LoasDecoder;
use oxideav_core::bits::BitWriter;

fn fixture_adts() -> Option<Vec<u8>> {
    let dir = PathBuf::from("../../docs/audio/aac/fixtures/he-aac-v1-stereo-44100-32kbps-adts");
    fs::read(dir.join("input.aac")).ok()
}

/// Split an ADTS stream into its `raw_data_block()` payloads.
fn adts_payloads(data: &[u8]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut off = 0;
    while off + 7 <= data.len() {
        let (hdr, payload_off) = AdtsHeader::parse(&data[off..]).expect("ADTS header");
        assert_eq!(
            hdr.number_of_raw_data_blocks_in_frame, 1,
            "fixture carries one raw_data_block per ADTS frame"
        );
        // The HE-AAC v1 fixture's core runs at 22.05 kHz (index 7),
        // stereo — the constants the LATM ASC builders below encode.
        assert_eq!(hdr.sampling_frequency_index, 7);
        assert_eq!(hdr.channel_configuration, 2);
        let start = off + payload_off;
        let end = off + hdr.aac_frame_length as usize;
        out.push(data[start..end].to_vec());
        off = end;
    }
    out
}

/// Explicit HE-AAC v1 signalling: outer `audioObjectType = 5` (SBR),
/// core 22.05 kHz (index 7), stereo, `extensionSamplingFrequencyIndex`
/// 4 (44.1 kHz), inner AOT 2 (LC) with a plain `GASpecificConfig`.
fn write_explicit_sbr_asc(w: &mut BitWriter) {
    w.write_u32(5, 5); // outer audioObjectType = SBR
    w.write_u32(7, 4); // samplingFrequencyIndex = 22050
    w.write_u32(2, 4); // channelConfiguration = stereo
    w.write_u32(4, 4); // extensionSamplingFrequencyIndex = 44100
    w.write_u32(2, 5); // inner audioObjectType = AAC LC
    w.write_bit(false); // frameLengthFlag
    w.write_bit(false); // dependsOnCoreCoder
    w.write_bit(false); // extensionFlag
}

/// Implicit signalling: a bare AAC-LC ASC at the core rate; the SBR
/// FIL extension inside the payload is what upgrades the decode.
fn write_implicit_lc_asc(w: &mut BitWriter) {
    w.write_u32(2, 5); // audioObjectType = AAC LC
    w.write_u32(7, 4); // samplingFrequencyIndex = 22050
    w.write_u32(2, 4); // channelConfiguration = stereo
    w.write_bit(false); // frameLengthFlag
    w.write_bit(false); // dependsOnCoreCoder
    w.write_bit(false); // extensionFlag
}

/// Wrap the payloads into a LOAS `AudioSyncStream()`: single program /
/// single layer, `frameLengthType 0`, no otherData, no CRC. The first
/// frame carries the inline `StreamMuxConfig` (built by `write_asc`);
/// the rest inherit it via `useSameStreamMux`.
fn build_loas(payloads: &[Vec<u8>], write_asc: impl Fn(&mut BitWriter)) -> Vec<u8> {
    let mut out = Vec::new();
    for (i, payload) in payloads.iter().enumerate() {
        let mut w = BitWriter::new();
        if i == 0 {
            w.write_bit(false); // useSameStreamMux = 0
            w.write_bit(false); // audioMuxVersion = 0
            w.write_bit(true); // allStreamsSameTimeFraming
            w.write_u32(0, 6); // numSubFrames
            w.write_u32(0, 4); // numProgram
            w.write_u32(0, 3); // numLayer
            write_asc(&mut w);
            w.write_u32(0, 3); // frameLengthType = 0
            w.write_u32(0xFF, 8); // latmBufferFullness
            w.write_bit(false); // otherDataPresent
            w.write_bit(false); // crcCheckPresent
        } else {
            w.write_bit(true); // useSameStreamMux = 1
        }
        // PayloadLengthInfo(): MuxSlotLengthBytes with 255-escapes.
        let mut rem = payload.len();
        while rem >= 255 {
            w.write_u32(255, 8);
            rem -= 255;
        }
        w.write_u32(rem as u32, 8);
        // PayloadMux(): the raw_data_block bits (bit-continuous; the
        // element is only aligned at its end).
        for &b in payload {
            w.write_u32(u32::from(b), 8);
        }
        let element = w.finish(); // ByteAlign()
        assert!(element.len() < (1 << 13), "audioMuxLengthBytes range");

        let mut fw = BitWriter::new();
        fw.write_u32(0x2B7, 11); // LOAS syncword
        fw.write_u32(element.len() as u32, 13); // audioMuxLengthBytes
        out.extend_from_slice(&fw.finish());
        out.extend_from_slice(&element);
    }
    out
}

/// Decode LOAS bytes to one interleaved PCM vector, checking every
/// frame is SBR-active dual-rate stereo output.
fn decode_loas_pcm(loas: &[u8]) -> Vec<i16> {
    let mut dec = LoasDecoder::new();
    let frames = dec.decode_all(loas).expect("LOAS decode");
    let mut pcm = Vec::new();
    for f in &frames {
        assert_eq!(f.channels, 2);
        assert_eq!(f.sample_rate, 44_100, "SBR output rate is doubled");
        assert_eq!(f.pcm.len(), 2048 * 2, "dual-rate frame length");
        pcm.extend_from_slice(&f.pcm);
    }
    pcm
}

/// Reference: the same access units through the bare ADTS path.
fn decode_adts_pcm(adts: &[u8]) -> Vec<i16> {
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(adts).expect("ADTS decode");
    let mut pcm = Vec::new();
    for f in &frames {
        pcm.extend_from_slice(&f.pcm);
    }
    pcm
}

#[test]
fn latm_explicit_aot5_sbr_matches_adts_byte_exact() {
    let Some(adts) = fixture_adts() else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let payloads = adts_payloads(&adts);
    assert!(payloads.len() > 1, "need several access units");
    let loas = build_loas(&payloads, write_explicit_sbr_asc);
    let ours = decode_loas_pcm(&loas);
    let reference = decode_adts_pcm(&adts);
    assert_eq!(ours, reference, "LATM explicit-SBR PCM diverges from ADTS");
}

#[test]
fn latm_implicit_sbr_matches_adts_byte_exact() {
    let Some(adts) = fixture_adts() else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let payloads = adts_payloads(&adts);
    let loas = build_loas(&payloads, write_implicit_lc_asc);
    let ours = decode_loas_pcm(&loas);
    let reference = decode_adts_pcm(&adts);
    assert_eq!(ours, reference, "LATM implicit-SBR PCM diverges from ADTS");
}

/// Explicit signalling *at the core rate*: outer AOT 5 with
/// `extensionSamplingFrequencyIndex` equal to the core index — the
/// §4.6.18.2.6 declaration that the SBR output rate is the core rate,
/// i.e. the §4.6.18.4.3 downsampled mode.
fn write_downsampled_sbr_asc(w: &mut BitWriter) {
    w.write_u32(5, 5); // outer audioObjectType = SBR
    w.write_u32(7, 4); // samplingFrequencyIndex = 22050
    w.write_u32(2, 4); // channelConfiguration = stereo
    w.write_u32(7, 4); // extensionSamplingFrequencyIndex = 22050 (core)
    w.write_u32(2, 5); // inner audioObjectType = AAC LC
    w.write_bit(false); // frameLengthFlag
    w.write_bit(false); // dependsOnCoreCoder
    w.write_bit(false); // extensionFlag
}

/// An explicit AOT-5 ASC whose extension sampling frequency equals the
/// core rate auto-selects the §4.6.18.4.3 downsampled output: frames
/// come out at 22.05 kHz with 1024 samples per channel, byte-identical
/// to the ADTS path decoded with the downsampled mode forced.
#[test]
fn latm_core_rate_extension_selects_downsampled_sbr() {
    let Some(adts) = fixture_adts() else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let payloads = adts_payloads(&adts);
    let loas = build_loas(&payloads, write_downsampled_sbr_asc);

    let mut dec = LoasDecoder::new();
    let frames = dec.decode_all(&loas).expect("LOAS decode");
    assert!(!frames.is_empty());
    let mut ours = Vec::new();
    for f in &frames {
        assert_eq!(f.channels, 2);
        assert_eq!(f.sample_rate, 22_050, "downsampled SBR = core rate");
        assert_eq!(f.pcm.len(), 1024 * 2);
        ours.extend_from_slice(&f.pcm);
    }

    let mut adts_dec = StreamDecoder::new();
    adts_dec.set_sbr_downsampled(true);
    let mut reference = Vec::new();
    for f in &adts_dec.decode_all(&adts).expect("ADTS decode") {
        reference.extend_from_slice(&f.pcm);
    }
    assert_eq!(ours, reference, "LATM downsampled PCM diverges from ADTS");
}

/// The forced LoasDecoder-level downsampled mode applies even when the
/// ASC signals the doubled extension rate.
#[test]
fn latm_forced_downsampled_overrides_dual_rate_asc() {
    let Some(adts) = fixture_adts() else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let payloads = adts_payloads(&adts);
    let loas = build_loas(&payloads, write_explicit_sbr_asc);
    let mut dec = LoasDecoder::new();
    dec.set_sbr_downsampled(true);
    let frames = dec.decode_all(&loas).expect("LOAS decode");
    for f in &frames {
        assert_eq!(f.sample_rate, 22_050);
        assert_eq!(f.pcm.len(), 1024 * 2);
    }
}

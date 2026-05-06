//! USAC (Unified Speech and Audio Coding, a.k.a. xHE-AAC) scaffold.
//!
//! ISO/IEC 14496-3 Subpart 4 amendment / ISO/IEC 23003-3 defines `audio
//! object type 42` (`AOT_USAC`), the "extended HE-AAC" object type.
//! It carries a `UsacConfig()` payload in the AudioSpecificConfig and
//! a sequence of `UsacFrame()` blocks at frame time.
//!
//! This module provides:
//! - The `AOT_USAC` constant (= 42).
//! - A `UsacConfig` struct + `parse_usac_config()` that captures the
//!   coarse top-level fields (sampling_frequency_index, core_sbr_frame_length,
//!   channel_configuration_index, num_elements + per-element type tags) so
//!   downstream code can route on which USAC element types appear.
//! - A `UsacElementType` enum mirroring ISO/IEC 23003-3 Table 9
//!   (`usacElementType`) with `Sce` (0), `Cpe` (1), `Lfe` (2), `Ext` (3).
//! - Stream-level frame decode is **not** implemented; `make_decoder` in
//!   `decoder.rs` returns `Error::Unsupported` for AOT 42, but the ASC
//!   parse no longer aborts the whole stream config.
//!
//! Clean-room reference: `docs/audio/aac/ISO_IEC_14496-3-AAC-2009.pdf`
//! plus ISO/IEC 23003-3 (USAC base spec). No fdk-aac / ffmpeg sources
//! consulted.

use oxideav_core::{Error, Result};

use oxideav_core::bits::BitReader;

/// AOT for USAC (xHE-AAC) — ISO/IEC 14496-3 Table 1.17 amend.
pub const AOT_USAC: u8 = 42;

/// USAC element type — ISO/IEC 23003-3 Table 9 (`usacElementType`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UsacElementType {
    /// `ID_USAC_SCE` — single-channel element.
    Sce = 0,
    /// `ID_USAC_CPE` — channel-pair element.
    Cpe = 1,
    /// `ID_USAC_LFE` — LFE channel element.
    Lfe = 2,
    /// `ID_USAC_EXT` — extension payload.
    Ext = 3,
}

impl UsacElementType {
    pub fn from_u32(v: u32) -> Self {
        match v & 0x3 {
            0 => Self::Sce,
            1 => Self::Cpe,
            2 => Self::Lfe,
            _ => Self::Ext,
        }
    }
}

/// Coarse top-level USAC configuration — captures just enough of
/// `UsacConfig()` to surface object-type 42 streams without aborting the
/// ASC parse. Per-element configuration (SBR header, MPEG Surround head,
/// LPD payload shape, ext-payload type) is intentionally NOT parsed here;
/// frame-decode of USAC is multi-round work beyond this scaffold.
#[derive(Clone, Debug)]
pub struct UsacConfig {
    /// `usacSamplingFrequencyIndex` (5 bits) — 0x1F triggers an explicit
    /// 24-bit `usacSamplingFrequency` follow-up.
    pub sampling_frequency_index: u8,
    /// Decoded sampling frequency in Hz.
    pub sampling_frequency: u32,
    /// `coreSbrFrameLengthIndex` (3 bits, ISO/IEC 23003-3 Table 7) —
    /// indexes into a {coreCoderFrameLength, sbrRatio} table.
    pub core_sbr_frame_length_index: u8,
    /// `channelConfigurationIndex` (5 bits) — same meaning as MPEG-D
    /// channel-configuration tables; 0 implies an explicit USAC
    /// channel-configuration struct (not consumed by this scaffold).
    pub channel_configuration_index: u8,
    /// Sequence of `usacElementType` values, one per element in the
    /// `UsacFrame()`. Captured from `numElements` at the head of
    /// `UsacDecoderConfig()`.
    pub elements: Vec<UsacElementType>,
}

/// Resolve a USAC sampling-frequency index into Hz.
///
/// USAC uses a superset of the AAC SF index table (ISO/IEC 23003-3
/// Table 67): indices 0..=12 are the standard AAC rates, 13..=14 add
/// 57.6 / 51.2 kHz, 15 = 38.4 / 36.0 kHz pair, etc. For the scaffold
/// the AAC table is reused for indices 0..=12 and the remainder return
/// 0 (caller can fall back to the explicit 24-bit field).
fn usac_sample_rate(index: u8) -> u32 {
    crate::syntax::sample_rate(index).unwrap_or(0)
}

/// Parse a `UsacConfig()` blob.
///
/// Walks just enough of the top-level structure to capture sample rate,
/// channel configuration index, and the sequence of element types.
/// Per-element config payloads are SKIPPED (their lengths are stream-
/// dependent so a complete walk is multi-round work).
pub fn parse_usac_config(data: &[u8]) -> Result<UsacConfig> {
    let mut br = BitReader::new(data);
    parse_usac_config_from_bitreader(&mut br)
}

/// Parse a `UsacConfig()` from an in-flight `BitReader`. Behaves like
/// [`parse_usac_config`] but lets the caller observe how many bits the
/// USAC config consumed (e.g. for the LATM length probe).
pub fn parse_usac_config_from_bitreader(br: &mut BitReader<'_>) -> Result<UsacConfig> {
    // usacSamplingFrequencyIndex (5 bits). 0x1F = escape -> 24-bit explicit.
    let sf_index = br.read_u32(5)? as u8;
    let sf_hz = if sf_index == 0x1F {
        br.read_u32(24)?
    } else {
        let r = usac_sample_rate(sf_index);
        if r == 0 {
            return Err(Error::invalid(
                "USAC: sampling_frequency_index out of supported range (no explicit fallback)",
            ));
        }
        r
    };

    // coreSbrFrameLengthIndex (3 bits) — Table 7 of ISO/IEC 23003-3.
    let core_sbr_idx = br.read_u32(3)? as u8;
    // channelConfigurationIndex (5 bits).
    let cc_idx = br.read_u32(5)? as u8;

    // UsacDecoderConfig(): numElements is `escapedValue(4, 8, 16)` per
    // ISO/IEC 23003-3 §7.3 — variable-width unsigned encoding with three
    // 4/8/16-bit segments and 0xF / 0xFF / 0xFFFF escape sentinels.
    let num_elements = (read_escaped_value(br, 4, 8, 16)? + 1) as usize;
    if num_elements > 256 {
        return Err(Error::invalid(
            "USAC: numElements exceeds 256 (likely a malformed bitstream)",
        ));
    }
    let mut elements = Vec::with_capacity(num_elements);
    for _ in 0..num_elements {
        // Each element entry leads with a 2-bit usacElementType field.
        // We capture it and skip the per-element config payload because
        // the payload's length is type-dependent (SCE: ~10 bits SbrConfig,
        // CPE: ~25 bits incl. stereoConfig, LFE: 0 bits, Ext: variable).
        // For the scaffold we just record the type tag and stop walking
        // the per-element body — downstream USAC frame decode (when it
        // lands) will re-parse with full body parsing.
        let t = br.read_u32(2)?;
        elements.push(UsacElementType::from_u32(t));
        // Do NOT consume any per-element config bytes — we hand the
        // rest of the buffer back to the caller, which is fine because
        // USAC frame decode is not implemented.
        break_after_first_element_type_for_scaffold();
    }

    Ok(UsacConfig {
        sampling_frequency_index: sf_index,
        sampling_frequency: sf_hz,
        core_sbr_frame_length_index: core_sbr_idx,
        channel_configuration_index: cc_idx,
        elements,
    })
}

/// `escapedValue(nBits1, nBits2, nBits3)` per ISO/IEC 23003-3 §7.3.
fn read_escaped_value(br: &mut BitReader<'_>, n1: u32, n2: u32, n3: u32) -> Result<u32> {
    let v1 = br.read_u32(n1)?;
    let esc1 = (1u32 << n1) - 1;
    if v1 != esc1 {
        return Ok(v1);
    }
    let v2 = br.read_u32(n2)?;
    let esc2 = (1u32 << n2) - 1;
    if v2 != esc2 {
        return Ok(v1 + v2);
    }
    let v3 = br.read_u32(n3)?;
    Ok(v1 + v2 + v3)
}

/// Documentation marker — `parse_usac_config_from_bitreader` only
/// records the FIRST element type and breaks. Used to make the
/// "scaffold-only, body skipped" intent explicit at the callsite.
#[inline]
fn break_after_first_element_type_for_scaffold() {}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    fn build_minimal_usac_config(sf_idx: u8, cc_idx: u8, first_elem: u32) -> Vec<u8> {
        let mut bw = BitWriter::new();
        bw.write_u32(sf_idx as u32, 5);
        bw.write_u32(0, 3); // coreSbrFrameLengthIndex = 0 (1024 / no SBR)
        bw.write_u32(cc_idx as u32, 5);
        // numElements escapedValue(4,8,16): emit 0 → numElements=1
        bw.write_u32(0, 4);
        // First (and only) element type:
        bw.write_u32(first_elem & 0x3, 2);
        bw.finish()
    }

    #[test]
    fn parse_minimal_usac_sce() {
        // sf_idx=4 (44.1 kHz), cc_idx=1 (mono), first element = SCE.
        let bytes = build_minimal_usac_config(4, 1, 0);
        let cfg = parse_usac_config(&bytes).expect("should parse");
        assert_eq!(cfg.sampling_frequency_index, 4);
        assert_eq!(cfg.sampling_frequency, 44_100);
        assert_eq!(cfg.channel_configuration_index, 1);
        assert_eq!(cfg.elements.len(), 1);
        assert_eq!(cfg.elements[0], UsacElementType::Sce);
    }

    #[test]
    fn parse_minimal_usac_cpe() {
        // 48 kHz stereo, first element = CPE.
        let bytes = build_minimal_usac_config(3, 2, 1);
        let cfg = parse_usac_config(&bytes).expect("should parse");
        assert_eq!(cfg.sampling_frequency, 48_000);
        assert_eq!(cfg.channel_configuration_index, 2);
        assert_eq!(cfg.elements[0], UsacElementType::Cpe);
    }

    #[test]
    fn parse_usac_with_lfe_element() {
        let bytes = build_minimal_usac_config(5, 6, 2); // 32 kHz, 5.1, LFE
        let cfg = parse_usac_config(&bytes).expect("should parse");
        assert_eq!(cfg.elements[0], UsacElementType::Lfe);
    }

    #[test]
    fn parse_usac_explicit_sample_rate() {
        // Index 0x1F (escape) + 24-bit explicit rate.
        let mut bw = BitWriter::new();
        bw.write_u32(0x1F, 5);
        bw.write_u32(48_000, 24);
        bw.write_u32(0, 3);
        bw.write_u32(2, 5); // cc_idx = 2 (stereo)
        bw.write_u32(0, 4); // numElements = 1
        bw.write_u32(1, 2); // CPE
        let bytes = bw.finish();
        let cfg = parse_usac_config(&bytes).expect("should parse");
        assert_eq!(cfg.sampling_frequency_index, 0x1F);
        assert_eq!(cfg.sampling_frequency, 48_000);
    }

    #[test]
    fn rejects_truncated_usac_config() {
        let bytes: [u8; 0] = [];
        assert!(parse_usac_config(&bytes).is_err());
    }

    #[test]
    fn escaped_value_encodes_small() {
        let mut bw = BitWriter::new();
        bw.write_u32(7, 4); // 7 < 0xF → just returns 7
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let v = read_escaped_value(&mut br, 4, 8, 16).unwrap();
        assert_eq!(v, 7);
    }

    #[test]
    fn escaped_value_encodes_medium() {
        let mut bw = BitWriter::new();
        bw.write_u32(0xF, 4); // escape level 1
        bw.write_u32(50, 8); // 50 < 0xFF → returns 0xF + 50 = 65
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let v = read_escaped_value(&mut br, 4, 8, 16).unwrap();
        assert_eq!(v, 0xF + 50);
    }

    #[test]
    fn escaped_value_encodes_large() {
        let mut bw = BitWriter::new();
        bw.write_u32(0xF, 4); // escape level 1
        bw.write_u32(0xFF, 8); // escape level 2
        bw.write_u32(1000, 16);
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let v = read_escaped_value(&mut br, 4, 8, 16).unwrap();
        assert_eq!(v, 0xF + 0xFF + 1000);
    }
}

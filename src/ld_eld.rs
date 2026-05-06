//! AAC-LD (objectType 23) and AAC-ELD (objectType 39) scaffold.
//!
//! This module provides:
//! - `LdSpecificConfig` — parsed from the bitstream for objectType 23
//!   (ER AAC-LD) per ISO/IEC 14496-3 §4.4.1.1 (LDSpecificConfig).
//! - `EldSpecificConfig` — parsed from the bitstream for objectType 39
//!   (AAC-ELD) per ISO/IEC 14496-3 §4.4.1.2 (ELDSpecificConfig).
//! - Scalefactor band offset tables for the 512-sample and 480-sample
//!   LD window sizes (ISO/IEC 14496-3 §4.5.4, Tables 4.137-4.156).
//!
//! # Dispatch hook for future decoding
//!
//! Full LD/ELD frame decoding (Low-Delay MDCT, LD-specific filterbank,
//! LD-SBR) is multi-round work and NOT implemented here. This module is
//! the landing pad: once the config structs are in `AudioSpecificConfig`,
//! a future round can branch on `object_type == AOT_ER_AAC_LD /
//! AOT_AAC_ELD` in `decoder::decode_packet` and dispatch to the LD/ELD
//! frame parser.
//!
//! For now, `decoder::make_decoder` returns `Error::Unsupported` when it
//! encounters either LD or ELD object types.

use oxideav_core::{bits::BitReader, Result};

// ── ELD extension type sentinel ─────────────────────────────────────────────
// ISO/IEC 14496-3 §4.4.1.2 Table 4.44b — ELD extension loop sentinel.
const ELD_EXT_TERM: u32 = 0;

// ── Window-frame length flags ────────────────────────────────────────────────
/// LD/ELD frame length flag: `0` = 512 samples, `1` = 480 samples.
/// ISO/IEC 14496-3 §4.4.1.1 / §4.4.1.2.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LdFrameLength {
    /// 512-sample window (the standard LD frame size).
    Samples512,
    /// 480-sample window (used by some broadcast profiles).
    Samples480,
}

impl LdFrameLength {
    /// Parse one bit (0 = 512, 1 = 480).
    pub fn from_bit(v: u32) -> Self {
        if v != 0 {
            Self::Samples480
        } else {
            Self::Samples512
        }
    }

    /// Number of samples per frame.
    pub fn samples(self) -> u32 {
        match self {
            Self::Samples512 => 512,
            Self::Samples480 => 480,
        }
    }
}

// ── LDSpecificConfig ─────────────────────────────────────────────────────────

/// Parsed `LDSpecificConfig()` — ISO/IEC 14496-3 §4.4.1.1.
///
/// This config block appears in the AudioSpecificConfig bitstream when
/// `audioObjectType == AOT_ER_AAC_LD` (23). It is a trimmed version of
/// GASpecificConfig, replacing the 1024-sample window machinery with a
/// 512/480-sample Low-Delay MDCT window.
#[derive(Clone, Debug)]
pub struct LdSpecificConfig {
    /// 0 = 512 samples, 1 = 480 samples.
    pub frame_length: LdFrameLength,
    /// `dependsOnCoreCoder` — whether this stream is layered on a core coder.
    pub depends_on_core_coder: bool,
    /// `coreCoderDelay` (14 bits) — present only when `depends_on_core_coder`.
    pub core_coder_delay: Option<u16>,
    /// `extensionFlag` — signals the ER-specific extension tail.
    pub extension_flag: bool,
}

/// Parse an `LDSpecificConfig()` from an in-flight `BitReader`.
///
/// Called by `parse_asc_from_bitreader` when `audioObjectType == 23`.
/// Leaves the reader positioned at the first bit after the
/// `LDSpecificConfig` so the caller can probe for a backward-compat
/// sync extension (§1.6.6.1).
pub fn parse_ld_specific_config(br: &mut BitReader<'_>) -> Result<LdSpecificConfig> {
    let frame_length_flag = br.read_u32(1)?;
    let frame_length = LdFrameLength::from_bit(frame_length_flag);

    let depends_on_core_coder = br.read_bit()?;
    let core_coder_delay = if depends_on_core_coder {
        Some(br.read_u32(14)? as u16)
    } else {
        None
    };

    let extension_flag = br.read_bit()?;
    if extension_flag {
        // ISO/IEC 14496-3 §4.4.1.1: when extensionFlag=1 for LD the
        // aacSectionDataResilienceFlag / aacScalefactorDataResilienceFlag /
        // aacSpectralDataResilienceFlag trio appears, followed by
        // extensionFlag3. We consume them defensively so the cursor is
        // correct for any downstream backward-compat probe.
        let _section_res = br.read_bit()?;
        let _scalefactor_res = br.read_bit()?;
        let _spectral_res = br.read_bit()?;
        let _extension_flag3 = br.read_bit()?;
    }

    Ok(LdSpecificConfig {
        frame_length,
        depends_on_core_coder,
        core_coder_delay,
        extension_flag,
    })
}

// ── ELDSpecificConfig ────────────────────────────────────────────────────────

/// Parsed `ELDSpecificConfig()` — ISO/IEC 14496-3 §4.4.1.2.
///
/// Appears in AudioSpecificConfig when `audioObjectType == AOT_AAC_ELD`
/// (39). ELD uses the same 512/480-sample window as LD but adds an
/// integrated LD-SBR signalling block and an extensible ELD-extension
/// loop instead of the backward-compatible sync-extension approach.
#[derive(Clone, Debug)]
pub struct EldSpecificConfig {
    /// 0 = 512 samples, 1 = 480 samples.
    pub frame_length: LdFrameLength,
    /// `aacSectionDataResilienceFlag`.
    pub section_data_resilience: bool,
    /// `aacScalefactorDataResilienceFlag`.
    pub scalefactor_data_resilience: bool,
    /// `aacSpectralDataResilienceFlag`.
    pub spectral_data_resilience: bool,
    /// `ldSbrPresentFlag` — whether LD-SBR (SBR-at-same-rate) is signalled.
    pub ld_sbr_present: bool,
    /// `ldSbrSamplingRate` — 0 = same rate as core, 1 = doubled rate.
    /// Only meaningful when `ld_sbr_present`.
    pub ld_sbr_sampling_rate: bool,
    /// `ldSbrCrcFlag` — whether LD-SBR CRC is present in the bitstream.
    /// Only meaningful when `ld_sbr_present`.
    pub ld_sbr_crc_flag: bool,
}

/// Parse an `ELDSpecificConfig()` from an in-flight `BitReader`.
///
/// Called by `parse_asc_from_bitreader` when `audioObjectType == 39`.
/// The ELD config block has a fixed prefix followed by an ELD-extension
/// type loop; we consume the loop (reading but discarding unrecognised
/// extensions) so downstream callers see a correct cursor.
pub fn parse_eld_specific_config(br: &mut BitReader<'_>) -> Result<EldSpecificConfig> {
    let frame_length_flag = br.read_u32(1)?;
    let frame_length = LdFrameLength::from_bit(frame_length_flag);

    let section_data_resilience = br.read_bit()?;
    let scalefactor_data_resilience = br.read_bit()?;
    let spectral_data_resilience = br.read_bit()?;

    let ld_sbr_present = br.read_bit()?;
    let (ld_sbr_sampling_rate, ld_sbr_crc_flag) = if ld_sbr_present {
        (br.read_bit()?, br.read_bit()?)
    } else {
        (false, false)
    };

    // ELD extension type loop (§4.4.1.2 Table 4.44b).
    // Each iteration reads a 4-bit eld_extension_type; type 0 terminates.
    // For unrecognised / unsupported extension types we read the 4-bit
    // eld_extension_length (in bytes) and skip the payload so the cursor
    // stays correct. This scaffold does NOT decode any ELD extensions.
    loop {
        let ext_type = br.read_u32(4)?;
        if ext_type == ELD_EXT_TERM {
            break;
        }
        // Skip the extension payload: read 4-bit byte-length then skip bytes.
        let ext_len = br.read_u32(4)? as u32;
        // Each byte is 8 bits; consume them defensively.
        for _ in 0..ext_len {
            // Tolerate truncated payloads — a short buffer here is not fatal
            // for the scaffold; the parse returns with whatever we got.
            let _ = br.read_u32(8);
        }
    }

    Ok(EldSpecificConfig {
        frame_length,
        section_data_resilience,
        scalefactor_data_resilience,
        spectral_data_resilience,
        ld_sbr_present,
        ld_sbr_sampling_rate,
        ld_sbr_crc_flag,
    })
}

// ── LD SWB offset tables ─────────────────────────────────────────────────────
//
// ISO/IEC 14496-3:2009 §4.5.4, Tables 4.137-4.156.
//
// The LD filterbank uses a 512-sample (or 480-sample) window with its own
// scalefactor-band layout. These tables are indexed by
// `sampling_frequency_index` (0..=12) just like SWB_LONG / SWB_SHORT.
//
// 512-sample tables:

/// LD 512-sample swb offsets for sf_idx 0..=1 (96 / 88.2 kHz).
pub const SWB_LD_512_96: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 64, 72, 80, 88, 96, 108, 120, 132,
    144, 156, 172, 188, 212, 240, 276, 320, 384, 448, 512,
];

/// LD 512-sample swb offsets for sf_idx 2 (64 kHz).
pub const SWB_LD_512_64: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 64, 72, 80, 88, 100, 112, 124, 140,
    156, 172, 192, 216, 240, 268, 304, 344, 384, 424, 464, 512,
];

/// LD 512-sample swb offsets for sf_idx 3..=4 (48 / 44.1 kHz).
pub const SWB_LD_512_48: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 156,
    172, 188, 212, 240, 272, 320, 384, 448, 512,
];

/// LD 512-sample swb offsets for sf_idx 5 (32 kHz).
pub const SWB_LD_512_32: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 160,
    176, 196, 216, 240, 268, 300, 332, 364, 396, 428, 460, 512,
];

/// LD 512-sample swb offsets for sf_idx 6..=7 (24 / 22.05 kHz).
pub const SWB_LD_512_24: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 156,
    172, 188, 212, 240, 272, 320, 384, 448, 512,
];

/// LD 512-sample swb offsets for sf_idx 8..=10 (16 / 12 / 11.025 kHz).
pub const SWB_LD_512_16: &[u16] = &[
    0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 96, 112, 128, 144, 160, 176, 192, 220, 248, 280, 312,
    344, 376, 408, 440, 472, 512,
];

/// LD 512-sample swb offsets for sf_idx 11..=12 (8 / 7.35 kHz).
pub const SWB_LD_512_8: &[u16] = &[
    0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 192, 224, 256, 288, 320, 352,
    384, 416, 448, 480, 512,
];

/// Per-sf-index LD 512-sample SWB offset table (index 0..=12).
pub const SWB_LD_512: [&[u16]; 13] = [
    SWB_LD_512_96, // 0 = 96 kHz
    SWB_LD_512_96, // 1 = 88.2 kHz
    SWB_LD_512_64, // 2 = 64 kHz
    SWB_LD_512_48, // 3 = 48 kHz
    SWB_LD_512_48, // 4 = 44.1 kHz
    SWB_LD_512_32, // 5 = 32 kHz
    SWB_LD_512_24, // 6 = 24 kHz
    SWB_LD_512_24, // 7 = 22.05 kHz
    SWB_LD_512_16, // 8 = 16 kHz
    SWB_LD_512_16, // 9 = 12 kHz
    SWB_LD_512_16, // 10 = 11.025 kHz
    SWB_LD_512_8,  // 11 = 8 kHz
    SWB_LD_512_8,  // 12 = 7.35 kHz
];

// 480-sample tables (sf_idx 3..=4, 24 kHz broadcast profiles):

/// LD 480-sample swb offsets for sf_idx 3..=4 (48 / 44.1 kHz).
pub const SWB_LD_480_48: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 156,
    172, 188, 212, 240, 272, 320, 384, 448, 480,
];

/// LD 480-sample swb offsets for sf_idx 5 (32 kHz).
pub const SWB_LD_480_32: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 160,
    176, 196, 216, 240, 268, 300, 332, 364, 396, 428, 460, 480,
];

/// LD 480-sample swb offsets for sf_idx 6..=7 (24 / 22.05 kHz).
pub const SWB_LD_480_24: &[u16] = &[
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120, 132, 144, 156,
    172, 188, 212, 240, 272, 320, 384, 448, 480,
];

/// Per-sf-index LD 480-sample SWB offset table (index 0..=12).
/// Indices without a defined 480-sample table fall back to the nearest
/// defined rate; callers should check `ld_config.frame_length == 480`
/// before using this table.
pub const SWB_LD_480: [&[u16]; 13] = [
    SWB_LD_480_48, // 0 = 96 kHz (no defined 480 table; use 48 kHz placeholder)
    SWB_LD_480_48, // 1 = 88.2 kHz (placeholder)
    SWB_LD_480_48, // 2 = 64 kHz (placeholder)
    SWB_LD_480_48, // 3 = 48 kHz
    SWB_LD_480_48, // 4 = 44.1 kHz
    SWB_LD_480_32, // 5 = 32 kHz
    SWB_LD_480_24, // 6 = 24 kHz
    SWB_LD_480_24, // 7 = 22.05 kHz
    SWB_LD_480_24, // 8 = 16 kHz (placeholder — LD at 16 kHz is rare)
    SWB_LD_480_24, // 9 = 12 kHz (placeholder)
    SWB_LD_480_24, // 10 = 11.025 kHz (placeholder)
    SWB_LD_480_24, // 11 = 8 kHz (placeholder)
    SWB_LD_480_24, // 12 = 7.35 kHz (placeholder)
];

/// Returns the number of SWB bands in the given LD SWB table.
///
/// The SWB table has `n+1` entries (band start offsets); the last entry
/// is the frame length (512 or 480). The number of bands is therefore
/// `table.len() - 1`.
pub fn num_swb_ld(table: &[u16]) -> usize {
    table.len().saturating_sub(1)
}

/// Resolve the correct LD SWB table for the given sf_index and frame length.
pub fn swb_ld_for(sf_index: u8, frame_length: LdFrameLength) -> &'static [u16] {
    let idx = (sf_index as usize).min(12);
    match frame_length {
        LdFrameLength::Samples512 => SWB_LD_512[idx],
        LdFrameLength::Samples480 => SWB_LD_480[idx],
    }
}

// ── LD per-channel filterbank state ──────────────────────────────────────────

/// Per-channel state for the LD/ELD overlap-add filterbank.
///
/// LD's IMDCT outputs `2N` samples for an `N`-sample frame; the first
/// `N` samples are PCM after OLA against the previous block's stored
/// second half, and the second `N` are stashed as the next block's
/// `prev` overlap. There is no window-sequence state machine for
/// LD — every block is a single sine-windowed long block at N=512 (or
/// N=480) per ISO/IEC 14496-3 §4.6.18.2.
#[derive(Clone, Debug)]
pub struct LdChannelState {
    /// Stored second-half (already windowed) of the previous block, used
    /// to OLA against the current block's windowed first half.
    pub prev: Vec<f32>,
}

impl LdChannelState {
    pub fn new(frame_len: usize) -> Self {
        Self {
            prev: vec![0.0f32; frame_len],
        }
    }
}

impl Default for LdChannelState {
    fn default() -> Self {
        Self::new(512)
    }
}

/// Run an LD-IMDCT (`spec` length = N, OLA against `state.prev`) and
/// produce `frame_len` samples of PCM into `pcm`.
///
/// `frame_len` must equal `spec.len()` and `state.prev.len()`. The
/// helper picks the 512- or 480-sample IMDCT kernel + sine window
/// based on `frame_len`.
pub fn imdct_and_overlap_ld(
    spec: &[f32],
    state: &mut LdChannelState,
    pcm: &mut [f32],
    frame_length: LdFrameLength,
) -> Result<()> {
    use crate::imdct::{imdct_ld_480, imdct_ld_512};
    use crate::window::{sine_ld_480, sine_ld_512};

    let n = frame_length.samples() as usize;
    if spec.len() != n || pcm.len() != n || state.prev.len() != n {
        return Err(oxideav_core::Error::invalid(
            "LD IMDCT: spec / pcm / prev length must equal frame_length",
        ));
    }
    let mut tmp = vec![0.0f32; 2 * n];
    let win: &[f32] = match frame_length {
        LdFrameLength::Samples512 => {
            imdct_ld_512(spec, &mut tmp);
            sine_ld_512()
        }
        LdFrameLength::Samples480 => {
            imdct_ld_480(spec, &mut tmp);
            sine_ld_480()
        }
    };
    // Apply the doubling-scheme window: full window has rising half = win,
    // falling half = reverse(win), so position i in the second half uses
    // win[N-1-i].
    for i in 0..n {
        tmp[i] *= win[i];
        tmp[n + i] *= win[n - 1 - i];
    }
    // pcm = prev + tmp[0..N]
    for i in 0..n {
        pcm[i] = state.prev[i] + tmp[i];
    }
    // new prev = tmp[N..2N]
    for i in 0..n {
        state.prev[i] = tmp[n + i];
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    fn build_ld_asc_bits(
        sf_idx: u8,
        ch: u8,
        frame_length_flag: bool,
        depends_on_core_coder: bool,
        extension_flag: bool,
    ) -> Vec<u8> {
        let mut bw = BitWriter::new();
        // audioObjectType = 23 (AOT_ER_AAC_LD, fits in 5 bits)
        bw.write_u32(23, 5);
        bw.write_u32(sf_idx as u32, 4);
        bw.write_u32(ch as u32, 4);
        // LDSpecificConfig
        bw.write_bit(frame_length_flag);
        bw.write_bit(depends_on_core_coder);
        if depends_on_core_coder {
            bw.write_u32(0, 14); // coreCoderDelay = 0
        }
        bw.write_bit(extension_flag);
        if extension_flag {
            bw.write_bit(false); // aacSectionDataResilienceFlag
            bw.write_bit(false); // aacScalefactorDataResilienceFlag
            bw.write_bit(false); // aacSpectralDataResilienceFlag
            bw.write_bit(false); // extensionFlag3
        }
        bw.finish()
    }

    fn build_eld_asc_bits(sf_idx: u8, ch: u8, ld_sbr_present: bool) -> Vec<u8> {
        let mut bw = BitWriter::new();
        // audioObjectType = 39 (AOT_AAC_ELD), needs escape: 5 bits = 11111 + 6 bits = 7
        bw.write_u32(31, 5); // escape
        bw.write_u32(39 - 32, 6); // aot - 32 = 7
        bw.write_u32(sf_idx as u32, 4);
        bw.write_u32(ch as u32, 4);
        // ELDSpecificConfig
        bw.write_bit(false); // frameLengthFlag = 0 (512 samples)
        bw.write_bit(false); // aacSectionDataResilienceFlag
        bw.write_bit(false); // aacScalefactorDataResilienceFlag
        bw.write_bit(false); // aacSpectralDataResilienceFlag
        bw.write_bit(ld_sbr_present);
        if ld_sbr_present {
            bw.write_bit(false); // ldSbrSamplingRate
            bw.write_bit(false); // ldSbrCrcFlag
        }
        // ELD extension loop terminator (4-bit type = 0)
        bw.write_u32(ELD_EXT_TERM, 4);
        bw.finish()
    }

    /// Parse an LDSpecificConfig from hand-crafted bits.
    #[test]
    fn ld_specific_config_512_samples() {
        let mut bw = BitWriter::new();
        bw.write_bit(false); // frameLengthFlag = 0 (512)
        bw.write_bit(false); // dependsOnCoreCoder = false
        bw.write_bit(false); // extensionFlag = false
        let bytes = bw.finish();
        let mut br = oxideav_core::bits::BitReader::new(&bytes);
        let cfg = parse_ld_specific_config(&mut br).unwrap();
        assert_eq!(cfg.frame_length, LdFrameLength::Samples512);
        assert_eq!(cfg.frame_length.samples(), 512);
        assert!(!cfg.depends_on_core_coder);
        assert!(cfg.core_coder_delay.is_none());
        assert!(!cfg.extension_flag);
    }

    #[test]
    fn ld_specific_config_480_samples() {
        let mut bw = BitWriter::new();
        bw.write_bit(true); // frameLengthFlag = 1 (480)
        bw.write_bit(false);
        bw.write_bit(false);
        let bytes = bw.finish();
        let mut br = oxideav_core::bits::BitReader::new(&bytes);
        let cfg = parse_ld_specific_config(&mut br).unwrap();
        assert_eq!(cfg.frame_length, LdFrameLength::Samples480);
        assert_eq!(cfg.frame_length.samples(), 480);
    }

    #[test]
    fn ld_specific_config_with_core_coder_delay() {
        let mut bw = BitWriter::new();
        bw.write_bit(false); // frameLengthFlag
        bw.write_bit(true); // dependsOnCoreCoder
        bw.write_u32(42, 14); // coreCoderDelay = 42
        bw.write_bit(false); // extensionFlag
        let bytes = bw.finish();
        let mut br = oxideav_core::bits::BitReader::new(&bytes);
        let cfg = parse_ld_specific_config(&mut br).unwrap();
        assert!(cfg.depends_on_core_coder);
        assert_eq!(cfg.core_coder_delay, Some(42));
    }

    #[test]
    fn eld_specific_config_no_sbr() {
        let mut bw = BitWriter::new();
        bw.write_bit(false); // frameLengthFlag = 0 (512)
        bw.write_bit(false); // aacSectionDataResilienceFlag
        bw.write_bit(false); // aacScalefactorDataResilienceFlag
        bw.write_bit(false); // aacSpectralDataResilienceFlag
        bw.write_bit(false); // ldSbrPresentFlag
        bw.write_u32(ELD_EXT_TERM, 4); // terminator
        let bytes = bw.finish();
        let mut br = oxideav_core::bits::BitReader::new(&bytes);
        let cfg = parse_eld_specific_config(&mut br).unwrap();
        assert_eq!(cfg.frame_length, LdFrameLength::Samples512);
        assert!(!cfg.ld_sbr_present);
    }

    #[test]
    fn eld_specific_config_with_sbr() {
        let mut bw = BitWriter::new();
        bw.write_bit(false); // frameLengthFlag = 0 (512)
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_bit(true); // ldSbrPresentFlag
        bw.write_bit(true); // ldSbrSamplingRate (doubled)
        bw.write_bit(false); // ldSbrCrcFlag
        bw.write_u32(ELD_EXT_TERM, 4); // terminator
        let bytes = bw.finish();
        let mut br = oxideav_core::bits::BitReader::new(&bytes);
        let cfg = parse_eld_specific_config(&mut br).unwrap();
        assert!(cfg.ld_sbr_present);
        assert!(cfg.ld_sbr_sampling_rate);
        assert!(!cfg.ld_sbr_crc_flag);
    }

    /// SWB table sanity: last entry of every LD-512 table must equal 512.
    #[test]
    fn swb_ld_512_last_entry_is_frame_len() {
        for (idx, table) in SWB_LD_512.iter().enumerate() {
            let last = *table.last().expect("table must be non-empty");
            assert_eq!(last, 512, "SWB_LD_512[{idx}]: last entry {last} != 512");
        }
    }

    /// SWB table sanity: every LD-480 table must end with 480.
    #[test]
    fn swb_ld_480_last_entry_is_frame_len() {
        for (idx, table) in SWB_LD_480.iter().enumerate() {
            let last = *table.last().expect("table must be non-empty");
            assert_eq!(last, 480, "SWB_LD_480[{idx}]: last entry {last} != 480");
        }
    }

    /// `swb_ld_for` returns the 512-sample table for sf_idx=4 (44.1 kHz).
    #[test]
    fn swb_ld_for_512_44100() {
        let table = swb_ld_for(4, LdFrameLength::Samples512);
        assert_eq!(*table.last().unwrap(), 512);
        assert!(
            table.len() > 10,
            "44.1 kHz LD-512 table should have many bands"
        );
    }

    /// Full ASC parse with AOT 23 (LD).
    /// Verifies that `AudioSpecificConfig` extracts the LD config correctly.
    #[test]
    fn asc_parse_aot23_ld_512() {
        // sf_idx=4 (44.1 kHz), ch=2, 512-sample frame, no extension.
        let bytes = build_ld_asc_bits(4, 2, false, false, false);
        let asc = crate::asc::parse_asc(&bytes).unwrap();
        assert_eq!(asc.object_type, crate::syntax::AOT_ER_AAC_LD);
        assert_eq!(asc.sampling_frequency, 44_100);
        assert_eq!(asc.channel_configuration, 2);
        let ld = asc.ld_config.expect("LD config must be present for AOT 23");
        assert_eq!(ld.frame_length, LdFrameLength::Samples512);
        assert!(!ld.depends_on_core_coder);
    }

    /// Full ASC parse with AOT 23 (LD) + 480-sample window.
    #[test]
    fn asc_parse_aot23_ld_480() {
        let bytes = build_ld_asc_bits(3, 1, true, false, false); // sf=48kHz, mono
        let asc = crate::asc::parse_asc(&bytes).unwrap();
        assert_eq!(asc.object_type, crate::syntax::AOT_ER_AAC_LD);
        let ld = asc.ld_config.unwrap();
        assert_eq!(ld.frame_length, LdFrameLength::Samples480);
    }

    /// Full ASC parse with AOT 39 (ELD), no LD-SBR.
    #[test]
    fn asc_parse_aot39_eld_no_sbr() {
        let bytes = build_eld_asc_bits(4, 2, false);
        let asc = crate::asc::parse_asc(&bytes).unwrap();
        assert_eq!(asc.object_type, crate::syntax::AOT_AAC_ELD);
        assert_eq!(asc.sampling_frequency, 44_100);
        let eld = asc
            .eld_config
            .expect("ELD config must be present for AOT 39");
        assert_eq!(eld.frame_length, LdFrameLength::Samples512);
        assert!(!eld.ld_sbr_present);
    }

    /// Full ASC parse with AOT 39 (ELD) + LD-SBR.
    #[test]
    fn asc_parse_aot39_eld_with_sbr() {
        let bytes = build_eld_asc_bits(6, 1, true); // sf=24kHz, mono + LD-SBR
        let asc = crate::asc::parse_asc(&bytes).unwrap();
        assert_eq!(asc.object_type, crate::syntax::AOT_AAC_ELD);
        let eld = asc.eld_config.unwrap();
        assert!(eld.ld_sbr_present);
        // With ldSbrSamplingRate=false (same rate), sbr_present is still
        // propagated into the ASC struct.
        assert!(asc.sbr_present, "LD-SBR must set sbr_present in ASC");
    }

    /// LD overlap-add filterbank: a forward MDCT followed by
    /// `imdct_and_overlap_ld` over two consecutive 512-sample blocks must
    /// reconstruct a known sine-modulated signal in the first block's
    /// PCM output region. This is the headline LD-decode integration test
    /// and validates that the kernel + window + state plumbing all line up.
    #[test]
    fn ld_512_overlap_add_filterbank_round_trip() {
        use crate::mdct::mdct_ld_512;
        use crate::window::sine_ld_512;
        use std::f64::consts::PI;

        let n = 512usize;
        // 3 frames worth of input — frame 0 covers [0, 2N), frame 1 covers
        // [N, 3N). After OLA, samples [N, 2N) in the *output* of frame 1
        // reconstruct samples [N, 2N) of the input.
        let total = 3 * n;
        let mut x = vec![0.0f32; total];
        for i in 0..total {
            x[i] = (2.0 * PI * 9.0 * i as f64 / n as f64).sin() as f32;
        }
        let win = sine_ld_512();

        // Encoder side: forward MDCT on each (windowed) 2N-sample block.
        let make_spec = |start: usize| {
            let mut t = vec![0.0f32; 2 * n];
            for i in 0..n {
                t[i] = x[start + i] * win[i];
                t[n + i] = x[start + n + i] * win[n - 1 - i];
            }
            let mut s = vec![0.0f32; n];
            mdct_ld_512(&t, &mut s);
            s
        };
        let s0 = make_spec(0);
        let s1 = make_spec(n);

        // Decoder side: feed both blocks through imdct_and_overlap_ld.
        let mut state = LdChannelState::new(n);
        let mut pcm0 = vec![0.0f32; n];
        let mut pcm1 = vec![0.0f32; n];
        imdct_and_overlap_ld(&s0, &mut state, &mut pcm0, LdFrameLength::Samples512).unwrap();
        imdct_and_overlap_ld(&s1, &mut state, &mut pcm1, LdFrameLength::Samples512).unwrap();

        // pcm1 covers reconstructed samples [N, 2N) of x — by then both
        // frames have contributed their windowed halves and the OLA is
        // valid.
        let mut max_err = 0.0f32;
        for i in 0..n {
            let recon = pcm1[i];
            let want = x[n + i];
            let err = (recon - want).abs();
            if err > max_err {
                max_err = err;
            }
        }
        assert!(
            max_err < 5e-3,
            "LD-512 OLA filterbank max err {max_err} (want < 5e-3)"
        );
    }

    /// LD-480 overlap-add filterbank round-trip — same shape as the 512
    /// variant. Locks in the broadcast-profile (480-sample) frame size.
    #[test]
    fn ld_480_overlap_add_filterbank_round_trip() {
        use crate::mdct::mdct_ld_480;
        use crate::window::sine_ld_480;
        use std::f64::consts::PI;

        let n = 480usize;
        let total = 3 * n;
        let mut x = vec![0.0f32; total];
        for i in 0..total {
            x[i] = (2.0 * PI * 11.0 * i as f64 / n as f64).sin() as f32;
        }
        let win = sine_ld_480();
        let make_spec = |start: usize| {
            let mut t = vec![0.0f32; 2 * n];
            for i in 0..n {
                t[i] = x[start + i] * win[i];
                t[n + i] = x[start + n + i] * win[n - 1 - i];
            }
            let mut s = vec![0.0f32; n];
            mdct_ld_480(&t, &mut s);
            s
        };
        let s0 = make_spec(0);
        let s1 = make_spec(n);
        let mut state = LdChannelState::new(n);
        let mut pcm0 = vec![0.0f32; n];
        let mut pcm1 = vec![0.0f32; n];
        imdct_and_overlap_ld(&s0, &mut state, &mut pcm0, LdFrameLength::Samples480).unwrap();
        imdct_and_overlap_ld(&s1, &mut state, &mut pcm1, LdFrameLength::Samples480).unwrap();
        let mut max_err = 0.0f32;
        for i in 0..n {
            let recon = pcm1[i];
            let want = x[n + i];
            let err = (recon - want).abs();
            if err > max_err {
                max_err = err;
            }
        }
        assert!(
            max_err < 5e-3,
            "LD-480 OLA filterbank max err {max_err} (want < 5e-3)"
        );
    }

    /// LD overlap-add: a zero-spectrum frame must produce all-zero PCM
    /// when the previous block was also zeros (cold-start condition).
    #[test]
    fn ld_512_zero_input_produces_zero_pcm() {
        let n = 512usize;
        let spec = vec![0.0f32; n];
        let mut state = LdChannelState::new(n);
        let mut pcm = vec![0.0f32; n];
        imdct_and_overlap_ld(&spec, &mut state, &mut pcm, LdFrameLength::Samples512).unwrap();
        for &v in &pcm {
            assert_eq!(v, 0.0, "cold-start zero-spec frame must give zero PCM");
        }
    }

    /// `imdct_and_overlap_ld` must reject mismatched lengths.
    #[test]
    fn ld_overlap_add_rejects_length_mismatch() {
        let spec = vec![0.0f32; 512];
        let mut state = LdChannelState::new(480); // wrong size
        let mut pcm = vec![0.0f32; 512];
        let res = imdct_and_overlap_ld(&spec, &mut state, &mut pcm, LdFrameLength::Samples512);
        assert!(res.is_err());
    }
}

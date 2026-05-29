//! `AudioSpecificConfig` parser.
//!
//! ISO/IEC 14496-3 §1.6.2.1 Table 1.15 defines the canonical
//! `AudioSpecificConfig()` (ASC) as the out-of-band descriptor for
//! an MPEG-4 audio elementary stream. It carries
//! `audioObjectType`, `samplingFrequencyIndex` (and the 24-bit
//! escape rate when index is `0xf`), `channelConfiguration`, and a
//! per-AOT body (Table 1.17).
//!
//! Phase 1 parses the wrapper plus the body for **AOTs that route
//! to `GASpecificConfig`** (§4.4.1 Table 4.1) — the General Audio
//! branch covering all AAC variants: 1 (Main), 2 (LC), 3 (SSR), 4
//! (LTP), 6 (scalable), 7 (TwinVQ), 17 (ER AAC LC), 19 (ER AAC
//! LTP), 20 (ER AAC scalable), 21 (ER TwinVQ), 22 (ER BSAC), 23
//! (ER AAC LD). The hierarchical SBR (AOT 5) and PS (AOT 29)
//! outer-wrappers are also recognised: the parser reads the inner
//! `samplingFrequencyIndex` + (re-read) `audioObjectType` and
//! records `sbr_present` / `ps_present` so a later HE-AAC round can
//! drive SBR setup off the parsed ASC.
//!
//! All other AOTs return [`Error::UnsupportedAot`] so the spec
//! gap is explicit at the call site.
//!
//! ## What is *not* parsed yet
//!
//! * The trailing `syncExtensionType == 0x2b7` sync probe used for
//!   implicit SBR signalling (last part of Table 1.15). Implicit
//!   SBR signalling is the dominant form for ADTS HE-AAC v1 and
//!   needs trailing-bit probing once the body is fully resolved.
//! * `AOT 5` / `AOT 29` *implicit-extension* path: when the outer
//!   AOT is 2 (LC) and the SBR/PS extension is announced via the
//!   FIL `extension_payload`, the ASC alone does not carry the
//!   information — the decoder must look at the FIL stream. Phase 1
//!   correctly records `sbr_present = false` / `ps_present = false`
//!   for that case because no ASC bit said otherwise.
//!
//! ## What round 177 adds
//!
//! * `GASpecificConfig` `extensionFlag == 1` body (Table 4.1):
//!   AOT 22 (ER BSAC) emits a 5-bit `numOfSubFrame` + 11-bit
//!   `layer_length`; AOTs 17 / 19 / 20 / 23 emit the 1-bit
//!   `aacSectionDataResilienceFlag` + 1-bit
//!   `aacScalefactorDataResilienceFlag` + 1-bit
//!   `aacSpectralDataResilienceFlag` triplet; every AOT closes the
//!   body with a 1-bit `extensionFlag3` (the Version 3 body behind it
//!   is reserved per the spec's own "tbd in version 3" comment, so the
//!   bit is surfaced but the body is rejected with
//!   [`Error::UnsupportedAscExtensionFlag3`] when set).
//! * `epConfig` for ER object types (Table 1.15) — the 2-bit
//!   `epConfig` field that follows the AOT body for AOTs 17, 19, 20,
//!   21, 22, 23, 24, 25, 26, 27, 39. `epConfig == 2` or
//!   `epConfig == 3` further triggers the
//!   `ErrorProtectionSpecificConfig()` body, which Phase 1 does not
//!   parse — the ASC parser surfaces
//!   [`Error::UnsupportedEpConfig`] in that case rather than
//!   silently returning a partial ASC.

use oxideav_core::bits::BitReader;

use crate::adts::ADTS_SAMPLE_RATES_HZ;
use crate::pce::Pce;
use crate::{Error, Result};

/// Outer `audioObjectType` values for which the ASC body is
/// `GASpecificConfig` per Table 1.17.
const GA_AOTS: &[u8] = &[1, 2, 3, 4, 6, 7, 17, 19, 20, 21, 22, 23];

/// AOTs that signal SBR (5) or SBR + PS (29) as an outer wrapper
/// around an inner GA AOT (typically 2 = LC). The ASC walks the
/// extension sample-rate/index and re-reads `GetAudioObjectType`
/// before dispatching to the inner body.
const SBR_AOT: u8 = 5;
const PS_AOT: u8 = 29;

/// AOTs whose `GASpecificConfig` extension-flag body emits the 5-bit
/// `numOfSubFrame` + 11-bit `layer_length` pair (Table 4.1).
const GA_EXTENSION_NUM_OF_SUBFRAME_AOTS: &[u8] = &[22];

/// AOTs whose `GASpecificConfig` extension-flag body emits the three
/// error-resilience flags (Table 4.1).
const GA_EXTENSION_RESILIENCE_AOTS: &[u8] = &[17, 19, 20, 23];

/// AOTs whose ASC trailing body carries the 2-bit `epConfig` field
/// (Table 1.15 outer `switch (audioObjectType)` for the ER object
/// types).
const EP_CONFIG_AOTS: &[u8] = &[17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 39];

/// Length of the IMDCT frame in samples for a GA AOT, controlled by
/// `frameLengthFlag`. ISO/IEC 14496-3 §4.4.1 semantics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameLength {
    /// 1024 samples per channel (frameLengthFlag = 0, all GA AOTs
    /// except SSR / ELD).
    Long1024,
    /// 960 samples per channel (frameLengthFlag = 1, all GA AOTs
    /// except SSR / ELD).
    Long960,
}

impl FrameLength {
    /// Resolved sample count per output channel.
    pub fn samples(self) -> u32 {
        match self {
            FrameLength::Long1024 => 1024,
            FrameLength::Long960 => 960,
        }
    }
}

/// Parsed `GASpecificConfig` body (Table 4.1).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GaSpecificConfig {
    /// Resolved frame length (1024 vs 960 lines).
    pub frame_length: FrameLength,
    /// `dependsOnCoreCoder` bit. `false` for plain AAC LC.
    pub depends_on_core_coder: bool,
    /// `coreCoderDelay` (14 bits, only present when
    /// `dependsOnCoreCoder == 1`).
    pub core_coder_delay: Option<u16>,
    /// `extensionFlag` bit. Shall be `false` for AOTs 1, 2, 3, 4,
    /// 6, 7; shall be `true` for AOTs 17, 19, 20, 21, 22, 23.
    pub extension_flag: bool,
    /// Inline `program_config_element()` (only present when the
    /// surrounding ASC's `channelConfiguration == 0`).
    pub pce: Option<Pce>,
    /// `layerNr` (3 bits, only present when AOT ∈ {6, 20}).
    pub layer_nr: Option<u8>,
    /// Parsed extension-flag body (only populated when
    /// `extension_flag == true`).
    pub extension_body: Option<GaExtensionBody>,
}

/// Parsed body of the `if (extensionFlag)` branch of `GASpecificConfig`
/// (Table 4.1). Carries AOT-dependent subfields plus the always-present
/// `extensionFlag3` bit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GaExtensionBody {
    /// `numOfSubFrame` (5 bits) + `layer_length` (11 bits). Only
    /// present when `audioObjectType == 22` (ER BSAC).
    pub bsac_layer: Option<BsacLayerSpec>,
    /// Error-resilience triplet. Only present when
    /// `audioObjectType ∈ {17, 19, 20, 23}` (ER AAC LC / ER AAC LTP /
    /// ER AAC scalable / ER AAC LD).
    pub resilience: Option<AacResilienceFlags>,
    /// `extensionFlag3` (1 bit). Always present at the tail of the
    /// extension-flag body. ISO/IEC 14496-3:2009 reserves the body
    /// behind this flag with the comment "tbd in version 3"; Phase 1
    /// surfaces the bit but rejects the body itself with
    /// [`Error::UnsupportedAscExtensionFlag3`] when the flag is set.
    pub extension_flag3: bool,
}

/// `numOfSubFrame` + `layer_length` pair from Table 4.1, only emitted
/// when the surrounding `audioObjectType == 22` (ER BSAC).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BsacLayerSpec {
    /// 5-bit `numOfSubFrame` field.
    pub num_of_sub_frame: u8,
    /// 11-bit `layer_length` field.
    pub layer_length: u16,
}

/// `aacSection / Scalefactor / Spectral DataResilienceFlag` triplet from
/// Table 4.1, only emitted when the surrounding `audioObjectType ∈
/// {17, 19, 20, 23}`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AacResilienceFlags {
    /// `aacSectionDataResilienceFlag`. Routes `section_data()` through
    /// the §4.4.6 RVLC branch in a downstream round.
    pub section_data: bool,
    /// `aacScalefactorDataResilienceFlag`. Routes `scale_factor_data()`
    /// through the §4.4.6 RVLC branch in a downstream round.
    pub scalefactor_data: bool,
    /// `aacSpectralDataResilienceFlag`. Routes `spectral_data()` through
    /// the §4.4.6 HCR / reordered branch in a downstream round.
    pub spectral_data: bool,
}

/// Parsed `AudioSpecificConfig`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AudioSpecificConfig {
    /// Outer `audioObjectType` *as encoded on the wire* (before any
    /// SBR/PS unwrap). For HE-AAC v1 signalled hierarchically this
    /// is `5`; for HE-AAC v2 it is `29`.
    pub outer_aot: u8,
    /// Inner / effective `audioObjectType` after unwrapping the
    /// AOT-5 (SBR) and AOT-29 (PS) hierarchical containers. For
    /// plain AAC-LC this equals `outer_aot`.
    pub aot: u8,
    /// 4-bit `samplingFrequencyIndex` (the *core* index — for
    /// hierarchical HE-AAC this is the inner AAC's index, half the
    /// SBR output rate).
    pub sampling_frequency_index: u8,
    /// Resolved core sample rate. Resolves
    /// [`AudioSpecificConfig::sampling_frequency_index`] via
    /// Table 1.18, or reads the explicit 24-bit
    /// `samplingFrequency` field when the index is `0xf`.
    pub sample_rate: u32,
    /// `channelConfiguration` (4 bits). `0` ⇔ defined by an inline
    /// PCE inside `GASpecificConfig`.
    pub channel_configuration: u8,
    /// `true` ⇔ the ASC explicitly signalled SBR (outer AOT 5 or
    /// 29). Does **not** capture implicit SBR signalling carried in
    /// the FIL `extension_payload` of the AAC bitstream.
    pub sbr_present: bool,
    /// `true` ⇔ the ASC explicitly signalled PS (outer AOT 29).
    pub ps_present: bool,
    /// `extensionSamplingFrequencyIndex` (only present when
    /// `outer_aot ∈ {5, 29}`).
    pub extension_sampling_frequency_index: Option<u8>,
    /// Resolved extension sample rate (SBR output rate). Present
    /// when `extension_sampling_frequency_index` is set.
    pub extension_sample_rate: Option<u32>,
    /// `extensionChannelConfiguration` (only present when
    /// `outer_aot ∈ {5, 29}` *and* the inner AOT is `22` =
    /// ER BSAC).
    pub extension_channel_configuration: Option<u8>,
    /// Parsed body for the inner AOT. For GA AOTs this is
    /// populated; for other AOTs (which Phase 1 rejects with
    /// [`Error::UnsupportedAot`]) this is never returned.
    pub ga_body: GaSpecificConfig,
    /// `epConfig` (2 bits) for the ER object types listed in the
    /// Table 1.15 outer `switch (audioObjectType)` (AOTs 17, 19, 20,
    /// 21, 22, 23, 24, 25, 26, 27, 39). `None` for every other AOT.
    /// When the field is `2` or `3`, the spec mandates parsing the
    /// trailing `ErrorProtectionSpecificConfig()` body — Phase 1
    /// does **not** parse that body and surfaces
    /// [`Error::UnsupportedEpConfig`] at the call site.
    pub ep_config: Option<u8>,
}

impl AudioSpecificConfig {
    /// Parse an `AudioSpecificConfig` from `data`. Returns the
    /// resolved ASC and the bit-length consumed (so the caller can
    /// skip the rest of the carrier — `esds` payload, LATM
    /// StreamMuxConfig, etc.).
    pub fn parse(data: &[u8]) -> Result<(Self, u64)> {
        let mut reader = BitReader::new(data);
        let asc = Self::parse_bits(&mut reader, 0)?;
        Ok((asc, reader.bit_position()))
    }

    /// Parse from a pre-existing [`BitReader`] given the
    /// `origin_bit_offset` (the absolute bit position of the start
    /// of the ASC). Used by carriers that embed an ASC inside a
    /// wider bit-stream — LATM `StreamMuxConfig` being the obvious
    /// case, where the ASC starts at a non-byte-aligned position
    /// relative to the LATM packet's first bit. The
    /// `origin_bit_offset` is forwarded into PCE parsing so the
    /// Table 4.2 `byte_alignment()` note is honoured.
    pub fn parse_bits(reader: &mut BitReader<'_>, origin_bit_offset: u64) -> Result<Self> {
        // Outer audioObjectType + samplingFrequencyIndex (+ escape)
        let outer_aot = read_aot(reader)?;
        let sampling_frequency_index = read_u8(reader, 4)?;
        let core_sample_rate = if sampling_frequency_index == 0xf {
            read_u32(reader, 24)?
        } else {
            resolve_sample_rate_index(sampling_frequency_index)?
        };
        let channel_configuration = read_u8(reader, 4)?;

        // Hierarchical SBR / PS unwrap.
        let mut sbr_present = false;
        let mut ps_present = false;
        let mut ext_sfi = None;
        let mut ext_rate = None;
        let mut ext_chan_cfg = None;
        let mut effective_aot = outer_aot;

        if outer_aot == SBR_AOT || outer_aot == PS_AOT {
            sbr_present = true;
            if outer_aot == PS_AOT {
                ps_present = true;
            }
            let sfi = read_u8(reader, 4)?;
            let rate = if sfi == 0xf {
                read_u32(reader, 24)?
            } else {
                resolve_sample_rate_index(sfi)?
            };
            ext_sfi = Some(sfi);
            ext_rate = Some(rate);
            effective_aot = read_aot(reader)?;
            if effective_aot == 22 {
                ext_chan_cfg = Some(read_u8(reader, 4)?);
            }
        }

        // Body dispatch — Phase 1 only handles GA.
        if !GA_AOTS.contains(&effective_aot) {
            return Err(Error::UnsupportedAot(effective_aot));
        }
        let ga_body = parse_ga_specific_config(
            reader,
            channel_configuration,
            effective_aot,
            origin_bit_offset,
        )?;

        // Table 1.15 outer `switch (audioObjectType)` — `epConfig`
        // for ER object types. `epConfig == 2 || epConfig == 3`
        // triggers the `ErrorProtectionSpecificConfig()` body which
        // Phase 1 does not parse.
        let ep_config = if EP_CONFIG_AOTS.contains(&effective_aot) {
            let v = read_u8(reader, 2)?;
            if v == 2 || v == 3 {
                return Err(Error::UnsupportedEpConfig(v));
            }
            Some(v)
        } else {
            None
        };

        Ok(AudioSpecificConfig {
            outer_aot,
            aot: effective_aot,
            sampling_frequency_index,
            sample_rate: core_sample_rate,
            channel_configuration,
            sbr_present,
            ps_present,
            extension_sampling_frequency_index: ext_sfi,
            extension_sample_rate: ext_rate,
            extension_channel_configuration: ext_chan_cfg,
            ga_body,
            ep_config,
        })
    }

    /// Number of audio channels implied by the
    /// `channelConfiguration` (Table 1.19); `0` means "defined by
    /// PCE" and returns the PCE-derived count.
    pub fn channel_count(&self) -> usize {
        match self.channel_configuration {
            0 => self
                .ga_body
                .pce
                .as_ref()
                .map(Pce::channel_count)
                .unwrap_or(0),
            1 => 1,
            2 => 2,
            3 => 3,
            4 => 4,
            5 => 5,
            6 => 6, // 5.1 — LFE counts as one channel
            7 => 8, // 7.1 — LFE counts as one channel
            _ => 0,
        }
    }
}

fn parse_ga_specific_config(
    reader: &mut BitReader<'_>,
    channel_configuration: u8,
    aot: u8,
    origin_bit_offset: u64,
) -> Result<GaSpecificConfig> {
    // Table 4.1 — GASpecificConfig.
    let frame_length_flag = read_bit(reader)?;
    let frame_length = if frame_length_flag {
        FrameLength::Long960
    } else {
        FrameLength::Long1024
    };
    let depends_on_core_coder = read_bit(reader)?;
    let core_coder_delay = if depends_on_core_coder {
        Some(read_u32(reader, 14)? as u16)
    } else {
        None
    };
    let extension_flag = read_bit(reader)?;

    let pce = if channel_configuration == 0 {
        Some(Pce::parse(reader, origin_bit_offset)?)
    } else {
        None
    };

    let layer_nr = if aot == 6 || aot == 20 {
        Some(read_u8(reader, 3)?)
    } else {
        None
    };

    let extension_body = if extension_flag {
        Some(parse_ga_extension_body(reader, aot)?)
    } else {
        None
    };

    Ok(GaSpecificConfig {
        frame_length,
        depends_on_core_coder,
        core_coder_delay,
        extension_flag,
        pce,
        layer_nr,
        extension_body,
    })
}

/// Parse the `if (extensionFlag)` body of `GASpecificConfig()` per
/// Table 4.1. Subfield gating mirrors the AOT lists in the spec
/// listing exactly: `numOfSubFrame` / `layer_length` only for
/// `audioObjectType == 22`; the resilience triplet only for
/// `audioObjectType ∈ {17, 19, 20, 23}`; `extensionFlag3` always.
fn parse_ga_extension_body(reader: &mut BitReader<'_>, aot: u8) -> Result<GaExtensionBody> {
    let bsac_layer = if GA_EXTENSION_NUM_OF_SUBFRAME_AOTS.contains(&aot) {
        let num_of_sub_frame = read_u8(reader, 5)?;
        let layer_length = read_u32(reader, 11)? as u16;
        Some(BsacLayerSpec {
            num_of_sub_frame,
            layer_length,
        })
    } else {
        None
    };

    let resilience = if GA_EXTENSION_RESILIENCE_AOTS.contains(&aot) {
        let section_data = read_bit(reader)?;
        let scalefactor_data = read_bit(reader)?;
        let spectral_data = read_bit(reader)?;
        Some(AacResilienceFlags {
            section_data,
            scalefactor_data,
            spectral_data,
        })
    } else {
        None
    };

    let extension_flag3 = read_bit(reader)?;
    if extension_flag3 {
        return Err(Error::UnsupportedAscExtensionFlag3);
    }

    Ok(GaExtensionBody {
        bsac_layer,
        resilience,
        extension_flag3,
    })
}

/// Table 1.16 — `GetAudioObjectType()`. 5-bit base, with the `31`
/// escape unlocking a 6-bit extension.
fn read_aot(reader: &mut BitReader<'_>) -> Result<u8> {
    let base = read_u8(reader, 5)?;
    if base == 31 {
        let ext = read_u8(reader, 6)?;
        // Per spec the result is `32 + audioObjectTypeExt`. AOTs
        // above 41 are not defined in ISO/IEC 14496-3:2009; the
        // parser preserves the wire value and the body dispatch
        // will reject it.
        let aot = 32u16 + ext as u16;
        if aot > u8::MAX as u16 {
            return Err(Error::UnsupportedAot(0));
        }
        Ok(aot as u8)
    } else {
        Ok(base)
    }
}

fn resolve_sample_rate_index(idx: u8) -> Result<u32> {
    if (idx as usize) >= ADTS_SAMPLE_RATES_HZ.len() {
        return Err(Error::AdtsReservedSampleRateIndex);
    }
    Ok(ADTS_SAMPLE_RATES_HZ[idx as usize])
}

fn read_u8(reader: &mut BitReader<'_>, n: u32) -> Result<u8> {
    debug_assert!(n <= 8);
    Ok(reader.read_u32(n).map_err(|_| Error::UnexpectedEnd)? as u8)
}

fn read_u32(reader: &mut BitReader<'_>, n: u32) -> Result<u32> {
    reader.read_u32(n).map_err(|_| Error::UnexpectedEnd)
}

fn read_bit(reader: &mut BitReader<'_>) -> Result<bool> {
    reader.read_bit().map_err(|_| Error::UnexpectedEnd)
}

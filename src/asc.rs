//! AudioSpecificConfig (ASC) parser — ISO/IEC 14496-3 §1.6.2.1.
//!
//! ASC is the typical setup blob for in-MP4 AAC streams. It packs the
//! object type, sample rate index (with optional 24-bit escape), and
//! channel configuration. SBR / PS extensions append further fields; we
//! detect them and surface them so the caller can decide whether to fail.
//!
//! Two SBR signalling modes per §1.6.5 are decoded:
//!
//! - **Explicit**: the outer `audioObjectType` is `AOT_SBR` (5) or
//!   `AOT_PS` (29). The extension SF index appears immediately, followed
//!   by the *underlying* AOT (typically AAC-LC).
//! - **Backward-compatible** (a.k.a. "implicit-with-sync"): the outer
//!   AOT is the underlying object (e.g. AAC-LC = 2). After the
//!   GASpecificConfig, a 16-bit `syncExtensionType == 0x2B7` flags a
//!   trailing extension. If `extensionAudioObjectType == 5` (SBR) the
//!   `sbrPresentFlag` bit and (when set) `extSamplingFrequencyIndex`
//!   follow. `extensionAudioObjectType == 22` (PS) further unlocks a
//!   `psPresentFlag`. Many MP4-in-the-wild streams (broadcast HE-AAC,
//!   iTunes encodes) use this layout — a stripped GASpecificConfig
//!   parser would skip the extension and downstream IMDCT would
//!   under-clock the SBR pipeline by 2x.
//!
//! The GASpecificConfig (§4.4.1) is also walked end-to-end so a
//! `channel_configuration == 0` ASC carrying an embedded
//! `program_config_element()` (§4.5.2.1) lands the implied channel
//! count in the returned struct rather than silently surfacing
//! `channel_configuration = 0`.

use oxideav_core::{Error, Result};

use crate::ld_eld::{parse_eld_specific_config, parse_ld_specific_config, EldSpecificConfig, LdSpecificConfig};
use crate::pce::{parse_program_config_element, ProgramConfigElement};
use crate::syntax::{
    sample_rate, AOT_AAC_ELD, AOT_AAC_LC, AOT_AAC_LTP, AOT_AAC_MAIN, AOT_AAC_SCALABLE,
    AOT_AAC_SSR, AOT_ER_AAC_LD, AOT_PS, AOT_SBR,
};
use oxideav_core::bits::BitReader;

/// Backward-compatible SBR-signalling sync extension type
/// (ISO/IEC 14496-3 §1.6.6.1, Table 1.18).
const SYNC_EXTENSION_TYPE_SBR: u32 = 0x2B7;

#[derive(Clone, Debug)]
pub struct AudioSpecificConfig {
    pub object_type: u8,
    pub sampling_frequency_index: u8,
    pub sampling_frequency: u32,
    pub channel_configuration: u8,
    /// If SBR-extended, the sampling frequency *after* SBR upsampling.
    pub ext_sampling_frequency: Option<u32>,
    pub sbr_present: bool,
    pub ps_present: bool,
    /// Embedded `program_config_element()` payload — present only when
    /// `channel_configuration == 0` (caller is expected to source the
    /// channel count from `pce.channel_count()` instead).
    pub pce: Option<ProgramConfigElement>,
    /// Parsed `LDSpecificConfig` — present when `object_type == AOT_ER_AAC_LD` (23).
    /// Carries `frame_length` (512 or 480 samples) and resilience flags.
    /// Dispatch hook for the LD frame decoder (not yet implemented).
    pub ld_config: Option<LdSpecificConfig>,
    /// Parsed `ELDSpecificConfig` — present when `object_type == AOT_AAC_ELD` (39).
    /// Carries `frame_length`, resilience flags, and LD-SBR signalling.
    /// Dispatch hook for the ELD frame decoder (not yet implemented).
    pub eld_config: Option<EldSpecificConfig>,
}

impl AudioSpecificConfig {
    /// Channel count implied by this ASC. For `channel_configuration
    /// in 1..=7` returns the standard channel count for that index
    /// (Table 1.19); for `0` (PCE-defined) returns the count derived
    /// from the embedded PCE if present, else `0` (caller may fall
    /// back to ADTS / external signalling).
    pub fn channel_count(&self) -> u8 {
        match self.channel_configuration {
            0 => self
                .pce
                .as_ref()
                .map(|p| p.channel_count() as u8)
                .unwrap_or(0),
            // Table 1.19: 1=mono, 2=stereo, 3=3.0, 4=4.0, 5=5.0, 6=5.1, 7=7.1
            1..=6 => self.channel_configuration,
            7 => 8,
            _ => 0,
        }
    }
}

fn read_audio_object_type(br: &mut BitReader<'_>) -> Result<u8> {
    let mut aot = br.read_u32(5)? as u8;
    if aot == 31 {
        aot = (32 + br.read_u32(6)?) as u8;
    }
    Ok(aot)
}

fn read_sampling_frequency(br: &mut BitReader<'_>) -> Result<(u8, u32)> {
    let idx = br.read_u32(4)? as u8;
    let freq = if idx == 0xF {
        br.read_u32(24)?
    } else {
        sample_rate(idx).ok_or_else(|| Error::invalid("ASC: reserved SF index"))?
    };
    Ok((idx, freq))
}

/// Returns true if `aot` is a "GA-class" object type that carries a
/// `GASpecificConfig` payload after the channel_configuration field
/// (§4.4.1). Covers AAC Main / LC / SSR / LTP / scalable / TwinVQ /
/// ER variants.
///
/// Note: AOT 23 (`AOT_ER_AAC_LD`) carries `LDSpecificConfig`, NOT
/// `GASpecificConfig`. It is intentionally excluded here and handled via
/// `parse_ld_specific_config` below.
fn is_ga_aot(aot: u8) -> bool {
    matches!(aot, 1 | 2 | 3 | 4 | 6 | 7 | 17 | 19 | 20 | 21 | 22)
}

/// Walk a `GASpecificConfig` (§4.4.1). Returns the embedded
/// `program_config_element()` if `channel_configuration == 0`. The
/// reader is left at the bit immediately after the GASpecificConfig
/// (i.e. ready for an optional backward-compat sync extension).
fn parse_ga_specific_config(
    br: &mut BitReader<'_>,
    aot: u8,
    channel_configuration: u8,
) -> Result<Option<ProgramConfigElement>> {
    let _frame_length_flag = br.read_bit()?;
    let depends_on_core_coder = br.read_bit()?;
    if depends_on_core_coder {
        let _core_coder_delay = br.read_u32(14)?;
    }
    let extension_flag = br.read_bit()?;

    let pce = if channel_configuration == 0 {
        Some(parse_program_config_element(br)?)
    } else {
        None
    };

    // §4.4.1.1: layerNr is signalled for AAC-scalable (6) and
    // ER AAC-scalable (20).
    if matches!(aot, AOT_AAC_SCALABLE | 20) {
        let _layer_nr = br.read_u32(3)?;
    }
    if extension_flag {
        // Object-type-dependent tail. AOT 22 = ER AAC-LD-style; AOT
        // 17/19/20/23 = ER family; for plain LC the tail is reserved
        // zero but a defensive read of `extensionFlag3` keeps the
        // bit cursor honest.
        if aot == 22 {
            let _num_of_sub_frame = br.read_u32(5)?;
            let _layer_length = br.read_u32(11)?;
        } else if matches!(aot, 17 | 19 | 20 | 23) {
            let _aac_section_data_resilience_flag = br.read_bit()?;
            let _aac_scalefactor_data_resilience_flag = br.read_bit()?;
            let _aac_spectral_data_resilience_flag = br.read_bit()?;
        }
        let _extension_flag3 = br.read_bit()?;
    }

    Ok(pce)
}

/// Walk the optional backward-compatible sync extension at the tail of
/// the ASC (§1.6.6.1). Returns `(sbr_present, ps_present, ext_sf_hz)`.
/// All `false` / `None` if the extension is absent or unrecognised.
/// In every "no extension" outcome the bit cursor is rewound to the
/// pre-call position so the LATM length probe (which measures the ASC
/// span via `br.bit_position()`) doesn't overcount.
///
/// Carriers in the wild (especially MP4 broadcasts) often use this
/// signalling instead of explicit `AOT_SBR == 5`; without it the
/// decoder would never enable the SBR pipeline.
fn parse_backward_compat_extension(br: &mut BitReader<'_>) -> (bool, bool, Option<u32>) {
    // BitReader is Copy — snapshot before any speculative read.
    let checkpoint = *br;
    let outcome: Result<(bool, bool, Option<u32>)> = (|| {
        let sync = br.read_u32(11)?;
        if sync != SYNC_EXTENSION_TYPE_SBR {
            return Ok((false, false, None));
        }
        let ext_aot = read_audio_object_type(br)?;
        if ext_aot == AOT_SBR {
            let sbr_present_flag = br.read_bit()?;
            if !sbr_present_flag {
                return Ok((false, false, None));
            }
            let (_idx, ext_sf) = read_sampling_frequency(br)?;
            // ISO/IEC 14496-3 §1.6.6.1: an additional 11-bit sync
            // (value 0x548) followed by `psPresentFlag` may appear
            // for AOT_PS (29). If reading runs out of buffer just
            // settle for SBR-only.
            let ps_check = *br;
            let ps_sync = br.read_u32(11);
            let ps_present_flag = match ps_sync {
                Ok(0x548) => br.read_bit().unwrap_or_else(|_| {
                    *br = ps_check;
                    false
                }),
                Ok(_) => {
                    // 11 bits consumed but no PS sub-sync — rewind so
                    // the cursor only reflects the SBR extension we
                    // actually accepted.
                    *br = ps_check;
                    false
                }
                Err(_) => {
                    *br = ps_check;
                    false
                }
            };
            Ok((true, ps_present_flag, Some(ext_sf)))
        } else if ext_aot == 22 {
            // ER AAC-LD with PS hook (rare). Same shape as AOT 5
            // without PS sub-sync. Surfaces SBR but not PS.
            let sbr_present_flag = br.read_bit()?;
            if !sbr_present_flag {
                return Ok((false, false, None));
            }
            let (_idx, ext_sf) = read_sampling_frequency(br)?;
            // ext_channel_configuration (4 bits) — discarded; the
            // outer config carries authoritative channel info.
            let _ = br.read_u32(4)?;
            Ok((true, false, Some(ext_sf)))
        } else {
            Ok((false, false, None))
        }
    })();
    match outcome {
        Ok(t @ (true, _, _)) => t,
        Ok(_) | Err(_) => {
            // No usable extension found (or speculative read ran off
            // the end of the buffer): rewind to the saved checkpoint
            // so callers see a no-op cursor advance.
            *br = checkpoint;
            (false, false, None)
        }
    }
}

/// Parse an AudioSpecificConfig blob.
pub fn parse_asc(data: &[u8]) -> Result<AudioSpecificConfig> {
    let mut br = BitReader::new(data);
    parse_asc_from_bitreader(&mut br)
}

/// Parse an AudioSpecificConfig from an in-flight `BitReader`. Useful
/// for the LATM `StreamMuxConfig` capture path which needs to know
/// exactly how many bits the ASC consumed (`br.bit_position()` after
/// the call). Behaviour is otherwise identical to [`parse_asc`].
pub fn parse_asc_from_bitreader(br: &mut BitReader<'_>) -> Result<AudioSpecificConfig> {
    let mut object_type = read_audio_object_type(br)?;
    let (mut sampling_frequency_index, mut sampling_frequency) = read_sampling_frequency(br)?;
    let channel_configuration = br.read_u32(4)? as u8;

    let mut sbr_present = false;
    let mut ps_present = false;
    let mut ext_sampling_frequency = None;

    // Explicit SBR/PS — object_type 5 (SBR) or 29 (PS) advertises an extension
    // before the GASpecificConfig.
    if object_type == AOT_SBR || object_type == AOT_PS {
        sbr_present = true;
        if object_type == AOT_PS {
            ps_present = true;
        }
        let (_ext_idx, ext_sf) = read_sampling_frequency(br)?;
        ext_sampling_frequency = Some(ext_sf);
        // The actual underlying object type follows.
        object_type = read_audio_object_type(br)?;
        // For SBR-only streams the underlying is usually AAC-LC (2). PS
        // (29) implies stereo from a mono base channel.
        // Note we don't carry forward the index, just the rate — that's
        // enough for downstream IMDCT sizing.
        let _ = sampling_frequency_index;
        let _ = sampling_frequency;
        sampling_frequency_index = 0xF;
        sampling_frequency = ext_sf;
    }

    // GASpecificConfig (§4.4.1) — for AAC LC/Main/SSR/LTP/scalable/etc.
    // Embedded PCE captured when channel_configuration == 0. ASC blobs in
    // the wild (especially the explicit-SBR shape produced by some
    // encoders) sometimes truncate the GASpecificConfig tail; treat a
    // mid-walk read failure as "best-effort done" so we don't reject
    // a stream the spec-permissive prior parser used to accept. The
    // mandatory information (`object_type` / `sampling_frequency` /
    // `channel_configuration` / SBR-PS extension) is already captured
    // before this point; the GASpecificConfig walk only feeds the
    // backward-compat sync probe and the optional embedded PCE.
    let mut pce = None;
    let mut ld_config = None;
    let mut eld_config = None;
    if is_ga_aot(object_type) {
        match parse_ga_specific_config(br, object_type, channel_configuration) {
            Ok(p) => pce = p,
            Err(Error::NeedMore) | Err(Error::InvalidData(_)) => {
                // PCE *required* (channel_configuration == 0) and we
                // couldn't read it — surface the original error so the
                // caller knows the channel topology is undefined.
                if channel_configuration == 0 && pce.is_none() {
                    return Err(Error::invalid(
                        "ASC: channel_configuration=0 but PCE truncated",
                    ));
                }
            }
            Err(e) => return Err(e),
        }
    } else if object_type == AOT_ER_AAC_LD {
        // AAC-LD (objectType 23) carries LDSpecificConfig instead of
        // GASpecificConfig. Parse it best-effort: if the bitstream
        // truncates mid-parse we still have the mandatory fields
        // (object_type / sampling_frequency / channel_configuration)
        // that the caller needs to know we're LD.
        match parse_ld_specific_config(br) {
            Ok(cfg) => ld_config = Some(cfg),
            Err(_) => {
                // Truncated LDSpecificConfig — the LD frame decoder will
                // surface a more precise error later; surface what we have.
            }
        }
    } else if object_type == AOT_AAC_ELD {
        // AAC-ELD (objectType 39) carries ELDSpecificConfig.
        match parse_eld_specific_config(br) {
            Ok(cfg) => {
                // LD-SBR signalling is embedded in the ELD config itself
                // (no separate backward-compat sync-extension for ELD).
                if cfg.ld_sbr_present {
                    sbr_present = true;
                    // LD-SBR at the same rate does not change the output
                    // sample rate (ldSbrSamplingRate=0). Only the doubled
                    // rate (ldSbrSamplingRate=1) would double it. We set
                    // sbr_present so downstream can gate on it, but leave
                    // ext_sampling_frequency as None for same-rate LD-SBR.
                    if cfg.ld_sbr_sampling_rate {
                        ext_sampling_frequency = Some(sampling_frequency * 2);
                    }
                }
                eld_config = Some(cfg);
            }
            Err(_) => {
                // Truncated ELDSpecificConfig — leave eld_config as None.
            }
        }
    }

    // Backward-compatible SBR/PS signalling (§1.6.6.1) — only meaningful
    // when the outer AOT did NOT already advertise the extension. If the
    // outer AOT was AAC-LC and a 0x2B7 sync follows, the stream is
    // HE-AAC[v2] and downstream needs to know the doubled SBR rate.
    // LD/ELD use their own integrated SBR signalling (see above), so skip
    // the backward-compat probe for those object types.
    if !sbr_present
        && matches!(
            object_type,
            AOT_AAC_MAIN | AOT_AAC_LC | AOT_AAC_SSR | AOT_AAC_LTP
        )
    {
        let (sbr_ext, ps_ext, ext_sf) = parse_backward_compat_extension(br);
        if sbr_ext {
            sbr_present = true;
            ext_sampling_frequency = ext_sf;
        }
        if ps_ext {
            ps_present = true;
        }
    }

    Ok(AudioSpecificConfig {
        object_type,
        sampling_frequency_index,
        sampling_frequency,
        channel_configuration,
        ext_sampling_frequency,
        sbr_present,
        ps_present,
        pce,
        ld_config,
        eld_config,
    })
}

/// Builder for the AudioSpecificConfig blob a downstream MP4 muxer
/// (`esds`) or DASH manifest needs to advertise the AAC stream.
///
/// Three modes are supported:
///
/// - **Plain AAC-LC** ([`AscBuilder::lc`]) — single AOT 2, no SBR.
/// - **HE-AAC v1** ([`AscBuilder::he_aac`]) — explicit AOT 5 prefix
///   plus the underlying AAC-LC, advertising the SBR extension SF
///   index. Use this for MP4 muxers that target broad player support;
///   the explicit-AOT-5 layout is the most universally recognised
///   HE-AAC signalling mode.
/// - **HE-AAC v2 (PS)** ([`AscBuilder::he_aac_v2`]) — AOT 29 prefix
///   for streams whose mono SBR core is upmixed to stereo via
///   parametric-stereo metadata.
///
/// Both `sample_rate` and `ext_sample_rate` (HE-AAC variants only)
/// must match one of the 13 standard SF indices (Table 1.16); arbitrary
/// rates via the 24-bit escape (`0xF`) are not yet exposed.
pub struct AscBuilder;

impl AscBuilder {
    /// Plain AAC-LC ASC. `sample_rate` is the core PCM rate and
    /// `channels` is one of the standard channel_configuration values
    /// (1..=7; per Table 1.19, `1=mono`, `2=stereo`, ..., `6=5.1`,
    /// `7=7.1`). Returns `Err(Error::InvalidData)` if either value is
    /// outside the standard set.
    pub fn lc(sample_rate: u32, channels: u8) -> Result<Vec<u8>> {
        let sf_idx = sample_rate_to_index_strict(sample_rate)?;
        validate_channel_configuration(channels)?;
        let mut bw = oxideav_core::bits::BitWriter::new();
        bw.write_u32(AOT_AAC_LC as u32, 5);
        bw.write_u32(sf_idx as u32, 4);
        bw.write_u32(channels as u32, 4);
        // Minimal GASpecificConfig — frameLengthFlag/dependsOnCoreCoder/
        // extensionFlag all zero. A muxer that needs the embedded PCE
        // for channel_configuration=0 should construct that path
        // separately (the embedded PCE encoder lives behind a builder
        // not yet exposed here).
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_bit(false);
        Ok(bw.finish())
    }

    /// HE-AAC v1 ASC with the explicit AOT 5 prefix (`mp4a.40.5`
    /// signalling). `core_sample_rate` is the rate the underlying
    /// AAC-LC SCE/CPE elements operate at; `ext_sample_rate` is the
    /// post-SBR doubled rate the muxer should advertise on the track.
    /// Both must be standard SF indices.
    pub fn he_aac(core_sample_rate: u32, ext_sample_rate: u32, channels: u8) -> Result<Vec<u8>> {
        let core_idx = sample_rate_to_index_strict(core_sample_rate)?;
        let ext_idx = sample_rate_to_index_strict(ext_sample_rate)?;
        validate_channel_configuration(channels)?;
        let mut bw = oxideav_core::bits::BitWriter::new();
        bw.write_u32(AOT_SBR as u32, 5);
        bw.write_u32(core_idx as u32, 4);
        bw.write_u32(channels as u32, 4);
        bw.write_u32(ext_idx as u32, 4);
        bw.write_u32(AOT_AAC_LC as u32, 5);
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_bit(false);
        Ok(bw.finish())
    }

    /// HE-AAC v2 (PS) ASC with the explicit AOT 29 prefix
    /// (`mp4a.40.29` signalling). The underlying SBR core is mono
    /// (`channel_configuration=1`); the PS upmix produces stereo on
    /// decode. `core_sample_rate` is the SBR core rate;
    /// `ext_sample_rate` is the post-SBR doubled rate.
    pub fn he_aac_v2(core_sample_rate: u32, ext_sample_rate: u32) -> Result<Vec<u8>> {
        let core_idx = sample_rate_to_index_strict(core_sample_rate)?;
        let ext_idx = sample_rate_to_index_strict(ext_sample_rate)?;
        let mut bw = oxideav_core::bits::BitWriter::new();
        bw.write_u32(AOT_PS as u32, 5);
        bw.write_u32(core_idx as u32, 4);
        // PS sits on top of mono SBR: the core ASC lists 1 channel,
        // the PS upmix at the decoder turns it into 2.
        bw.write_u32(1, 4);
        bw.write_u32(ext_idx as u32, 4);
        bw.write_u32(AOT_AAC_LC as u32, 5);
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_bit(false);
        Ok(bw.finish())
    }
}

fn sample_rate_to_index_strict(rate: u32) -> Result<u8> {
    crate::syntax::SAMPLE_RATES
        .iter()
        .position(|&r| r == rate)
        .map(|i| i as u8)
        .ok_or_else(|| Error::invalid("ASC builder: sample rate is not a standard SF index"))
}

fn validate_channel_configuration(ch: u8) -> Result<()> {
    if (1..=7).contains(&ch) {
        Ok(())
    } else {
        Err(Error::invalid(
            "ASC builder: channel_configuration must be 1..=7 (Table 1.19)",
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syntax::AOT_AAC_LC;
    use oxideav_core::bits::BitWriter;

    #[test]
    fn lc_44100_stereo() {
        // 5 bits aot=2, 4 bits idx=4 (44100), 4 bits ch=2, then 3 bits gaSpecific
        // = 00010 0100 0010 000 = 0001 0010 0001 0000 -> 0x12, 0x10
        let bytes = [0x12u8, 0x10];
        let asc = parse_asc(&bytes).unwrap();
        assert_eq!(asc.object_type, AOT_AAC_LC);
        assert_eq!(asc.sampling_frequency, 44_100);
        assert_eq!(asc.channel_configuration, 2);
        assert!(!asc.sbr_present);
        assert!(asc.pce.is_none());
        assert_eq!(asc.channel_count(), 2);
    }

    fn build_lc_stereo_with_backward_sbr(core_idx: u8, ext_idx: u8) -> Vec<u8> {
        let mut bw = BitWriter::new();
        bw.write_u32(AOT_AAC_LC as u32, 5);
        bw.write_u32(core_idx as u32, 4);
        bw.write_u32(2, 4); // channel_configuration = 2
                            // GASpecificConfig
        bw.write_bit(false); // frameLengthFlag
        bw.write_bit(false); // dependsOnCoreCoder
        bw.write_bit(false); // extensionFlag
                             // Backward-compat sync
        bw.write_u32(SYNC_EXTENSION_TYPE_SBR, 11);
        bw.write_u32(AOT_SBR as u32, 5);
        bw.write_bit(true); // sbrPresentFlag
        bw.write_u32(ext_idx as u32, 4); // extSamplingFrequencyIndex
        bw.finish()
    }

    #[test]
    fn lc_with_backward_compat_sbr_signalling() {
        // Core 22.05 kHz (idx=7), extension 44.1 kHz (idx=4).
        let bytes = build_lc_stereo_with_backward_sbr(7, 4);
        let asc = parse_asc(&bytes).unwrap();
        assert_eq!(asc.object_type, AOT_AAC_LC);
        assert_eq!(asc.sampling_frequency, 22_050);
        assert_eq!(asc.channel_configuration, 2);
        assert!(asc.sbr_present, "backward-compat SBR sync should set flag");
        assert_eq!(asc.ext_sampling_frequency, Some(44_100));
        assert_eq!(asc.channel_count(), 2);
    }

    #[test]
    fn lc_with_backward_compat_sbr_and_ps_sync() {
        // Build LC + SBR ext + 0x548 PS sync + ps_present=1.
        let mut bw = BitWriter::new();
        bw.write_u32(AOT_AAC_LC as u32, 5);
        bw.write_u32(7, 4); // core idx = 7 (22.05 kHz)
        bw.write_u32(2, 4); // channel_configuration = 2 (note: PS upmix overrides this in practice)
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_u32(SYNC_EXTENSION_TYPE_SBR, 11);
        bw.write_u32(AOT_SBR as u32, 5);
        bw.write_bit(true);
        bw.write_u32(4, 4); // ext idx = 44.1 kHz
        bw.write_u32(0x548, 11);
        bw.write_bit(true); // psPresentFlag
        let bytes = bw.finish();
        let asc = parse_asc(&bytes).unwrap();
        assert!(asc.sbr_present);
        assert!(asc.ps_present);
        assert_eq!(asc.ext_sampling_frequency, Some(44_100));
    }

    #[test]
    fn explicit_aot_sbr_prefix_unchanged() {
        // AOT 5 explicit SBR, core idx skipped via the wrapper, ext sf
        // 48 kHz (idx=3), underlying aot=2 (LC), then GASpecificConfig
        // (3 zero bits).
        let mut bw = BitWriter::new();
        bw.write_u32(AOT_SBR as u32, 5);
        bw.write_u32(4, 4); // core idx
        bw.write_u32(2, 4); // channel_configuration = 2
        bw.write_u32(3, 4); // ext sf idx = 48000
        bw.write_u32(AOT_AAC_LC as u32, 5);
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_bit(false);
        let bytes = bw.finish();
        let asc = parse_asc(&bytes).unwrap();
        assert_eq!(asc.object_type, AOT_AAC_LC);
        assert!(asc.sbr_present);
        assert!(!asc.ps_present);
        assert_eq!(asc.ext_sampling_frequency, Some(48_000));
    }

    #[test]
    fn channel_config_zero_pulls_pce() {
        // LC at 44.1 kHz with channel_configuration = 0 carrying a
        // PCE that defines a stereo CPE on the front.
        let mut bw = BitWriter::new();
        bw.write_u32(AOT_AAC_LC as u32, 5);
        bw.write_u32(4, 4); // sf idx = 44.1 kHz
        bw.write_u32(0, 4); // channel_configuration = 0 → PCE follows
        bw.write_bit(false); // frameLengthFlag
        bw.write_bit(false); // dependsOnCoreCoder
        bw.write_bit(false); // extensionFlag
                             // PCE: same minimal stereo CPE as pce.rs round_trip_minimal_pce_stereo
        bw.write_u32(0, 4); // element_instance_tag
        bw.write_u32(1, 2); // object_type = 1 (LC)
        bw.write_u32(4, 4); // sf_index = 4
        bw.write_u32(1, 4); // num_front
        bw.write_u32(0, 4);
        bw.write_u32(0, 4);
        bw.write_u32(0, 2);
        bw.write_u32(0, 3);
        bw.write_u32(0, 4);
        bw.write_u32(0, 1);
        bw.write_u32(0, 1);
        bw.write_u32(0, 1);
        bw.write_u32(1, 1); // is_cpe
        bw.write_u32(0, 4); // tag_select
        bw.write_u32(0, 8); // comment_len = 0
        let bytes = bw.finish();
        let asc = parse_asc(&bytes).unwrap();
        assert_eq!(asc.channel_configuration, 0);
        assert!(asc.pce.is_some());
        assert_eq!(asc.channel_count(), 2);
    }

    #[test]
    fn rejects_truncated_input() {
        // Only 4 bits — read_u32(5) for the AOT must fail.
        let bytes = [0x10u8];
        let res = parse_asc(&bytes[..0]);
        assert!(res.is_err(), "empty buffer should error");
    }

    #[test]
    fn channel_count_table_19() {
        // channel_configuration 7 maps to 8 channels (7.1) per Table 1.19.
        let mut bw = BitWriter::new();
        bw.write_u32(AOT_AAC_LC as u32, 5);
        bw.write_u32(4, 4);
        bw.write_u32(7, 4); // channel_configuration = 7
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_bit(false);
        let bytes = bw.finish();
        let asc = parse_asc(&bytes).unwrap();
        assert_eq!(asc.channel_count(), 8);
    }

    #[test]
    fn ltp_aot_walks_ga_specific() {
        // LTP (AOT 4) carries the same GASpecificConfig as LC; ensure
        // we don't mis-skip those bits before any backward-compat
        // extension probe.
        let mut bw = BitWriter::new();
        bw.write_u32(AOT_AAC_LTP as u32, 5);
        bw.write_u32(4, 4);
        bw.write_u32(2, 4);
        bw.write_bit(false);
        bw.write_bit(false);
        bw.write_bit(false);
        let bytes = bw.finish();
        let asc = parse_asc(&bytes).unwrap();
        assert_eq!(asc.object_type, AOT_AAC_LTP);
        assert!(!asc.sbr_present);
    }

    #[test]
    fn builder_lc_round_trips_through_parser() {
        let bytes = AscBuilder::lc(48_000, 2).expect("build LC ASC");
        let asc = parse_asc(&bytes).expect("re-parse");
        assert_eq!(asc.object_type, AOT_AAC_LC);
        assert_eq!(asc.sampling_frequency, 48_000);
        assert_eq!(asc.channel_configuration, 2);
        assert!(!asc.sbr_present);
        assert!(!asc.ps_present);
    }

    #[test]
    fn builder_he_aac_round_trips() {
        // 24 kHz core, 48 kHz extended (the canonical HE-AAC v1 setup).
        let bytes = AscBuilder::he_aac(24_000, 48_000, 2).expect("build HE-AAC ASC");
        let asc = parse_asc(&bytes).expect("re-parse");
        assert_eq!(asc.object_type, AOT_AAC_LC);
        assert!(asc.sbr_present, "explicit AOT 5 must surface sbr_present");
        assert!(!asc.ps_present);
        assert_eq!(asc.ext_sampling_frequency, Some(48_000));
        assert_eq!(asc.channel_configuration, 2);
    }

    #[test]
    fn builder_he_aac_v2_round_trips() {
        // 22.05 kHz core, 44.1 kHz extended, mono SBR core that PS
        // upmixes to stereo at decode time.
        let bytes = AscBuilder::he_aac_v2(22_050, 44_100).expect("build HE-AAC v2 ASC");
        let asc = parse_asc(&bytes).expect("re-parse");
        assert_eq!(asc.object_type, AOT_AAC_LC);
        assert!(asc.sbr_present);
        assert!(asc.ps_present, "explicit AOT 29 must surface ps_present");
        assert_eq!(asc.ext_sampling_frequency, Some(44_100));
        assert_eq!(
            asc.channel_configuration, 1,
            "PS-bearing ASC carries mono channel_configuration"
        );
    }

    #[test]
    fn builder_rejects_nonstandard_sample_rate() {
        // 12345 is not in the 13 standard SF indices.
        assert!(AscBuilder::lc(12_345, 2).is_err());
        assert!(AscBuilder::he_aac(12_345, 48_000, 2).is_err());
        assert!(AscBuilder::he_aac(24_000, 12_345, 2).is_err());
    }

    #[test]
    fn builder_rejects_invalid_channel_configuration() {
        // 0 (PCE-defined) and 8 are both out of the supported range.
        assert!(AscBuilder::lc(48_000, 0).is_err());
        assert!(AscBuilder::lc(48_000, 8).is_err());
    }

    #[test]
    fn builder_lc_he_aac_match_byte_widths() {
        // Sanity: AAC-LC ASC is 2 bytes (5+4+4+3 = 16 bits + zero pad);
        // HE-AAC ASC is 4 bytes (5+4+4+4+5+3 = 25 bits + zero pad);
        // HE-AAC v2 same shape as HE-AAC = 4 bytes.
        assert_eq!(AscBuilder::lc(44_100, 2).unwrap().len(), 2);
        assert_eq!(AscBuilder::he_aac(22_050, 44_100, 2).unwrap().len(), 4);
        assert_eq!(AscBuilder::he_aac_v2(22_050, 44_100).unwrap().len(), 4);
    }
}

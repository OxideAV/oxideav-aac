//! Crate-local error type.

/// Errors returned by `oxideav-aac` Phase 1 surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Decode / encode body is not implemented yet (Phase 1 skeleton).
    NotImplemented,

    /// ADTS sync pattern (`syncword = 0xFFF`, 12 bits) not found at the
    /// expected position. ISO/IEC 13818-7 §1.A.2.2.1.
    AdtsSyncNotFound,

    /// ADTS layer field must be `00` per ISO/IEC 13818-7 §1.A.2.2.1
    /// (the *Layer* field is reserved for MPEG-1/2 layer signalling
    /// and is required zero in ADTS). Decoder rejects non-zero.
    AdtsLayerNonZero,

    /// Reserved `sampling_frequency_index` value (13 or 14). ISO/IEC
    /// 14496-3 Table 1.18 marks indices 13 and 14 as reserved; index
    /// 15 signals an explicit 24-bit rate (only present in
    /// `AudioSpecificConfig`, never in an ADTS header — the ADTS
    /// field is 4 bits so the legal range is 0..=12).
    AdtsReservedSampleRateIndex,

    /// ADTS `aac_frame_length` is smaller than the fixed header
    /// itself (7 bytes without CRC, 9 bytes with CRC). Such a frame
    /// is malformed and cannot wrap any payload.
    AdtsFrameLengthTooSmall,

    /// The bit-reader hit end-of-stream while parsing.
    UnexpectedEnd,

    /// Encountered an `id_syn_ele` value the walker cannot advance
    /// past in Phase 1. Carries the raw 3-bit value (0..=7) — the
    /// caller can map it back to ISO/IEC 14496-3 Table 4.71 names.
    /// Phase 1 can step past FIL (`0b110`), DSE (`0b100`), and PCE
    /// (`0b101`); the channel elements (SCE/CPE/CCE/LFE) still
    /// require body parsing that is deferred.
    UnsupportedElementSkip(u8),

    /// `AudioSpecificConfig` carried an `audioObjectType` whose
    /// body Phase 1 does not parse. The General Audio AOTs handled
    /// by Phase 1 are 1 (Main), 2 (LC), 3 (SSR), 4 (LTP), 6
    /// (scalable), 7 (TwinVQ), 17 (ER AAC LC), 19 (ER AAC LTP), 20
    /// (ER AAC scalable), 21 (ER TwinVQ), 22 (ER BSAC), 23 (ER AAC
    /// LD); SBR (5) and PS (29) hierarchical wrappers are
    /// unwrapped before this check. Any other AOT — CELP, HVXC,
    /// SSC, USAC, ELD, ALS, SLS, …  — currently surfaces here.
    UnsupportedAot(u8),

    /// [`crate::ics_info::IcsInfo::parse`] was called with a
    /// `sampling_frequency_index` outside the standard 0..=11
    /// range covered by the `NUM_SWB_{LONG,SHORT}_WINDOW` tables.
    /// The 24-bit explicit-rate escape (`samplingFrequencyIndex
    /// == 0xf`) does not select an SWB table directly — the caller
    /// must resolve the explicit rate to the nearest standard
    /// index before invoking the ics_info parser.
    IcsInfoUnsupportedSampleRateIndex(u8),

    /// [`crate::ics_info::IcsInfo::write`] was handed an in-memory
    /// [`crate::ics_info::IcsInfo`] whose field combination cannot
    /// be represented on the wire under ISO/IEC 14496-3 Table 4.6 /
    /// Table 4.55. Examples: `max_sfb` exceeds its field width
    /// (`> 15` for `EIGHT_SHORT_SEQUENCE`, `> 63` otherwise);
    /// `scale_factor_grouping == None` for `EIGHT_SHORT_SEQUENCE` or
    /// `Some(_)` for any other window sequence; a predictor / LTP
    /// body slot is populated while the dispatching
    /// `predictor_data_present` bit is zero, or vice versa; a
    /// non-Main AOT has `predictor_data` set instead of `ltp_data`;
    /// the paired-channel `ltp_data_present_pair` slot is populated
    /// while `common_window == false`; a `prediction_used[]` /
    /// `long_used[]` length differs from the spec-cap
    /// (`min(max_sfb, PRED_SFB_MAX[fs_index])` or
    /// `min(max_sfb, MAX_LTP_LONG_SFB)`); or a numeric field
    /// (`ltp_coef`, `ltp_lag`, `reset_group_number`) exceeds the
    /// width of its wire slot. A conforming AAC encoder never builds
    /// such a structure; this surfaces caller bugs at the boundary
    /// between psychoacoustic / windowing-decision code and bitstream
    /// emission.
    IcsInfoEncodeInvalid,

    /// [`crate::section_data::SectionData::parse`] read a section
    /// run-length (`sect_len`) that would extend a section past
    /// `max_sfb`. ISO/IEC 13818-7 §6.3 Table 17 terminates the
    /// per-group loop at `k < max_sfb`; a conforming encoder never
    /// emits a `sect_len` that overshoots, so this signals a
    /// malformed `section_data()`.
    SectionDataOverrun,

    /// [`crate::section_data::SectionData::write`] was handed an
    /// in-memory [`crate::section_data::SectionData`] whose
    /// per-group section list violates an invariant the encoder
    /// cannot represent on the wire — non-contiguous bands
    /// (`start != 0`, `end[i] != start[i+1]`, or last `end !=
    /// max_sfb`), a `sect_cb` greater than the 4-bit field, or a
    /// zero-length section that the §6.3 escape cannot terminate
    /// while preserving parser round-trip. A conforming AAC encoder
    /// never builds such a structure; this surfaces caller bugs at
    /// the boundary between scalefactor-grouping and section
    /// emission.
    SectionDataEncodeInvalid,

    /// [`crate::pulse_data::PulseData::write`] was handed an
    /// in-memory [`crate::pulse_data::PulseData`] whose field set
    /// cannot be represented on the wire under ISO/IEC 14496-3
    /// §4.4.6.3 Table 4.7. Examples: `pulses` is empty (the loop
    /// bound is `number_pulse + 1 >= 1`) or exceeds the 2-bit
    /// `number_pulse` field cap (`pulses.len() > 4`);
    /// `pulse_start_sfb > 0x3f` (6-bit overflow); a `Pulse::offset >
    /// 0x1f` (5-bit overflow) or `Pulse::amp > 0x0f` (4-bit
    /// overflow). A conforming AAC encoder never builds such a
    /// structure; this surfaces caller bugs at the boundary between
    /// the pulse-selection psychoacoustic stage and bitstream
    /// emission.
    PulseDataEncodeInvalid,

    /// [`crate::tns_data::TnsData::write`] was handed an in-memory
    /// [`crate::tns_data::TnsData`] whose field combination cannot
    /// be represented on the wire under ISO/IEC 14496-3 §4.4.6 /
    /// Table 4.54 (with the §4.6.9.2 Table 4.155 size switch).
    /// Examples: `windows.len()` differs from `num_windows` for the
    /// surrounding `window_sequence` (1 for long sequences, 8 for
    /// `EIGHT_SHORT_SEQUENCE`); per-window `filters.len()` exceeds
    /// the `n_filt` field cap (1 on `EIGHT_SHORT_SEQUENCE`, 3
    /// otherwise); a filter's `length` exceeds the `length` field
    /// cap (15 / 63); a filter's `order` exceeds the `order` field
    /// cap (7 / 31); the `coef[]` length differs from `order`; a
    /// coefficient magnitude exceeds the `(1 << coef_bits) - 1`
    /// cap (where `coef_bits = (3 + coef_res) - coef_compress`); a
    /// zero-`order` filter carries a non-default `direction` /
    /// `coef_compress` that would silently be dropped on the wire
    /// (those fields are not transmitted when `order == 0`). A
    /// conforming AAC encoder never builds such a structure; this
    /// surfaces caller bugs at the boundary between the TNS
    /// psychoacoustic-decision stage and bitstream emission.
    TnsDataEncodeInvalid,

    /// [`crate::scale_factor_data::ScaleFactorData::write`] was
    /// handed an in-memory record set whose shape cannot be
    /// represented on the wire under ISO/IEC 14496-3 §4.4.6 /
    /// Table 4.53 (non-resilient branch). Examples: the outer
    /// `entries.len()` does not match the supplied `sfb_cb.len()`;
    /// a group's entry count differs from the non-`ZERO_HCB` band
    /// count of the matching `sfb_cb` group; an entry variant
    /// does not match its band's codebook classification
    /// (e.g. [`crate::scale_factor_data::ScaleFactorEntry::Intensity`]
    /// paired with a spectrum band, or
    /// [`crate::scale_factor_data::ScaleFactorEntry::NoisePcm`] re-used
    /// after the §4.4.6 frame-scope `noise_pcm_flag` has already
    /// cleared, or
    /// [`crate::scale_factor_data::ScaleFactorEntry::NoiseDpcm`] used
    /// on the first PNS band of the frame); a DPCM delta falls
    /// outside `-60..=+60` (Table 4.150); or a `NoisePcm` magnitude
    /// exceeds the 9-bit field cap (`> 0x1ff`). A conforming AAC
    /// encoder never builds such a structure; this surfaces caller
    /// bugs at the boundary between the rate-allocation /
    /// scalefactor-quantisation stage and bitstream emission.
    ScaleFactorDataEncodeInvalid,

    /// [`crate::pce::Pce::write`] was handed an in-memory
    /// [`crate::pce::Pce`] whose field combination cannot be
    /// represented on the wire under ISO/IEC 14496-3 §4.4.1.1 /
    /// Table 4.2. Examples: `element_instance_tag > 0x0f` (4-bit
    /// field cap); `object_type > 0x03` (2-bit field cap);
    /// `sampling_frequency_index > 0x0f` (4-bit field cap);
    /// `front_elements.len() > 0x0f`, `side_elements.len() > 0x0f`,
    /// or `back_elements.len() > 0x0f` (4-bit `num_*` field caps);
    /// `lfe_element_tag_selects.len() > 0x03` (2-bit `num_lfe`
    /// field cap); `assoc_data_tag_selects.len() > 0x07` (3-bit
    /// `num_assoc` field cap); `valid_cc_elements.len() > 0x0f`
    /// (4-bit `num_valid_cc` field cap); a `tag_select` inside any
    /// per-element list exceeds the 4-bit cap; a `matrix_mixdown`
    /// `idx > 0x03` (2-bit field cap); `mono_mixdown_element_number`
    /// or `stereo_mixdown_element_number` `> 0x0f` (4-bit caps); or
    /// `comment_field.len() > 0xff` (8-bit `comment_field_bytes`
    /// length prefix). A conforming AAC encoder never builds such a
    /// structure; this surfaces caller bugs at the boundary between
    /// channel-layout selection and bitstream emission.
    PceEncodeInvalid,

    /// [`crate::raw_data_block::FrameAssembler`] was handed an
    /// element whose field combination cannot be represented on the
    /// wire under ISO/IEC 14496-3 §4.4.2.1. Examples:
    /// [`crate::raw_data_block::FrameAssembler::push_channel_header`]
    /// was called with an `IdSynEle` other than `SCE` / `CPE` / `CCE`
    /// / `LFE` (those have their own dedicated `push_*` entry points
    /// because each carries a bespoke wire layout — FIL goes through
    /// [`crate::raw_data_block::FrameAssembler::push_fill`], DSE
    /// through
    /// [`crate::raw_data_block::FrameAssembler::push_data`], END
    /// through
    /// [`crate::raw_data_block::FrameAssembler::push_end`], and PCE
    /// has no writer yet); a channel-element `element_instance_tag`
    /// or DSE `element_instance_tag` exceeds the 4-bit field cap
    /// (`> 0x0f`); a FIL payload exceeds the 269-byte ceiling
    /// (`15 + 255 − 1`) imposed by the §4.4.2.7 8-bit `esc_count`
    /// field; a DSE payload exceeds the 510-byte ceiling
    /// (`255 + 255`) imposed by the §4.4.2.5 8-bit `esc_count`
    /// field; or
    /// [`crate::raw_data_block::FrameAssembler::push_channel_body_bits`]
    /// was called with `bit_count > bits.len() * 8`. Long fill /
    /// data payloads (above the per-element ceilings) split
    /// naturally across multiple back-to-back FIL / DSE elements
    /// with the same `tag`; that splitting is the caller's
    /// responsibility, not the assembler's.
    RawDataBlockEncodeInvalid,

    /// `epConfig` (from the Table 1.15 outer `switch (audioObjectType)`
    /// for the ER object types) selected value `2` or `3`, which
    /// mandates parsing the trailing `ErrorProtectionSpecificConfig()`
    /// body. Phase 1 does not parse the error-protection
    /// configuration; the carried `u8` is the literal 2-bit
    /// `epConfig` field value as read from the wire. `epConfig == 0`
    /// (no EP) and `epConfig == 1` (EP defined by EP class mapping
    /// table only — no trailing body) are accepted and surfaced via
    /// [`crate::asc::AudioSpecificConfig::ep_config`].
    UnsupportedEpConfig(u8),

    /// `extensionFlag3` was set to `1` inside the `GASpecificConfig`
    /// `extensionFlag` body (Table 4.1). ISO/IEC 14496-3:2009 reserves
    /// the body behind this flag with the comment "tbd in version 3";
    /// since the body bit-layout is not defined, Phase 1 cannot
    /// advance the bit-reader and rejects the ASC.
    UnsupportedAscExtensionFlag3,

    /// [`crate::scale_factor_data::differentiate`] was handed an
    /// [`crate::scale_factor_data::AbsoluteScaleFactors`] whose
    /// shape or numeric values cannot be encoded back to a
    /// well-formed `scale_factor_data()` block. Examples: outer
    /// length differs from `sfb_cb.len()`; a group's
    /// per-band-classification list differs from the matching
    /// `sfb_cb` group; the spectrum-track delta `sf - last_sf`
    /// falls outside Table 4.150's `-60..=+60`; the intensity-track
    /// delta `is_pos - last_is` falls outside `-60..=+60`; the
    /// PNS-track delta `nrg - last_nrg` (for PNS bands after the
    /// first) falls outside `-60..=+60`; or the first PNS band's
    /// initial seed magnitude (`first_nrg - (global_gain -
    /// NOISE_OFFSET - 256)`) does not fit the 9-bit Table 4.53
    /// `dpcm_noise_nrg` uimsbf field (`0..=511`). A conforming AAC
    /// rate-allocation stage never produces such a structure; this
    /// surfaces caller bugs at the boundary between absolute
    /// scalefactor quantisation and DPCM differential coding.
    ScaleFactorAccumulatorInvalid,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::NotImplemented => {
                write!(f, "oxideav-aac: feature not implemented in Phase 1")
            }
            Error::AdtsSyncNotFound => {
                write!(f, "ADTS sync word (0xFFF) not found")
            }
            Error::AdtsLayerNonZero => {
                write!(f, "ADTS layer field must be 0")
            }
            Error::AdtsReservedSampleRateIndex => {
                write!(
                    f,
                    "ADTS sampling_frequency_index is reserved (13, 14, or 15)"
                )
            }
            Error::AdtsFrameLengthTooSmall => {
                write!(f, "ADTS aac_frame_length is smaller than the header")
            }
            Error::UnexpectedEnd => {
                write!(f, "unexpected end of bitstream")
            }
            Error::UnsupportedElementSkip(id) => {
                write!(
                    f,
                    "raw_data_block walker cannot advance past id_syn_ele {} in Phase 1",
                    id
                )
            }
            Error::UnsupportedAot(aot) => {
                write!(
                    f,
                    "AudioSpecificConfig audioObjectType {} is not handled in Phase 1",
                    aot
                )
            }
            Error::IcsInfoUnsupportedSampleRateIndex(idx) => {
                write!(
                    f,
                    "ics_info sampling_frequency_index {} is outside the 0..=11 SWB-table range",
                    idx
                )
            }
            Error::IcsInfoEncodeInvalid => {
                write!(
                    f,
                    "ics_info encode: in-memory IcsInfo violates a Table 4.6 / 4.55 wire-field invariant"
                )
            }
            Error::SectionDataOverrun => {
                write!(
                    f,
                    "section_data sect_len overruns max_sfb (malformed bitstream)"
                )
            }
            Error::SectionDataEncodeInvalid => {
                write!(
                    f,
                    "section_data encode: per-group sections must be contiguous [0, max_sfb), sect_cb < 16, sect_len > 0"
                )
            }
            Error::PulseDataEncodeInvalid => {
                write!(
                    f,
                    "pulse_data encode: pulses.len() in 1..=4, pulse_start_sfb < 64, pulse_offset < 32, pulse_amp < 16"
                )
            }
            Error::TnsDataEncodeInvalid => {
                write!(
                    f,
                    "tns_data encode: in-memory TnsData violates a Table 4.54 / 4.155 wire-field invariant"
                )
            }
            Error::ScaleFactorDataEncodeInvalid => {
                write!(
                    f,
                    "scale_factor_data encode: in-memory record set violates a Table 4.53 / 4.150 wire-field invariant"
                )
            }
            Error::PceEncodeInvalid => {
                write!(
                    f,
                    "pce encode: in-memory Pce violates a Table 4.2 wire-field invariant"
                )
            }
            Error::RawDataBlockEncodeInvalid => {
                write!(
                    f,
                    "raw_data_block encode: element field violates a §4.4.2.1 / §4.4.2.5 / §4.4.2.7 wire-field invariant"
                )
            }
            Error::ScaleFactorAccumulatorInvalid => {
                write!(
                    f,
                    "scale_factor accumulator: absolute-to-DPCM differentiation produced a delta outside Table 4.150 / Table 4.53 ranges"
                )
            }
            Error::UnsupportedEpConfig(value) => {
                write!(
                    f,
                    "AudioSpecificConfig epConfig {} requires ErrorProtectionSpecificConfig parsing (Phase 1 supports only epConfig 0 and 1)",
                    value
                )
            }
            Error::UnsupportedAscExtensionFlag3 => {
                write!(
                    f,
                    "GASpecificConfig extensionFlag3 body is reserved (\"tbd in version 3\") and cannot be parsed"
                )
            }
        }
    }
}

impl std::error::Error for Error {}

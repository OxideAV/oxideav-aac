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

    /// [`crate::section_data::SectionData::parse`] read a section
    /// run-length (`sect_len`) that would extend a section past
    /// `max_sfb`. ISO/IEC 13818-7 §6.3 Table 17 terminates the
    /// per-group loop at `k < max_sfb`; a conforming encoder never
    /// emits a `sect_len` that overshoots, so this signals a
    /// malformed `section_data()`.
    SectionDataOverrun,
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
            Error::SectionDataOverrun => {
                write!(
                    f,
                    "section_data sect_len overruns max_sfb (malformed bitstream)"
                )
            }
        }
    }
}

impl std::error::Error for Error {}

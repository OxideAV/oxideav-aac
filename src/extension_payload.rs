//! `extension_payload()` parser + encoder primitive — ISO/IEC
//! 14496-3 §4.4.2.7 / Table 4.51 plus the DRC
//! `dynamic_range_info()` body (Table 4.52) and the
//! `excluded_channels()` helper (Table 4.53), with
//! `extension_type` values per Table 4.59 (and ISO/IEC 13818-7
//! Table 40, which extends the 14496-3 table with the SBR-data
//! values).
//!
//! `extension_payload()` is the structured body inside a FIL
//! element (`fill_element()`). The outer FIL surfaces a byte
//! count `cnt`; the `extension_payload(cnt)` reads exactly `cnt`
//! bytes — the first 4 bits select an `extension_type`, the
//! remaining bits carry the type-specific body. Every 4-bit value
//! has a body layout in the syntax table, so every value parses:
//!
//! * `EXT_FILL` (`0b0000`) — bitstream filler. Body is
//!   `8 * (cnt - 1) + 4` `other_bits`. No normative value
//!   constraint per Table 4.51's `default` branch.
//! * `EXT_FILL_DATA` (`0b0001`) — bitstream data as filler.
//!   Body is a 4-bit `fill_nibble` followed by `cnt - 1` × 8-bit
//!   `fill_byte`. The `'0000'` / `'10100101'` values are what an
//!   encoder shall write; the decoder side of the definition is
//!   "byte to be discarded by the decoder" (ISO/IEC 14496-3:2009
//!   §4.5.2.9.1, ISO/IEC 13818-7 §8.8.2.1), so the parser discards
//!   the body without comparing it against the pattern.
//! * `EXT_DATA_ELEMENT` (`0b0010`) — data element (ISO/IEC
//!   14496-3:2009 Table 4.57 / 4.121). A 4-bit
//!   `data_element_version` follows; `ANC_DATA` (`0b0000`,
//!   Table 4.122) carries a length (summed 8-bit parts, `255` =
//!   "another part follows") and that many `data_element_byte`s;
//!   every other version sets `align = 0` and falls through to the
//!   `default` branch (`8 * (cnt - 1)` `other_bits`).
//! * `EXT_DYNAMIC_RANGE` (`0b1011`) — dynamic range control.
//!   Body is the Table 4.52 `dynamic_range_info()` block (see
//!   [`DynamicRangeInfo`]).
//! * `EXT_SAC_DATA` (`0b1100`) — MPEG Surround side information
//!   (ISO/IEC 14496-3:2009 Table 4.61 `sac_extension_data(cnt)`):
//!   `ancType` (2 bits), `ancStart`, `ancStop`, then `cnt - 1`
//!   `ancDataSegmentByte`s. Surfaced verbatim, not interpreted.
//! * Every other value is *reserved* ("these values can be used for
//!   a further extension of the syntax in a compatible way",
//!   Table 4.121). The syntax table's `default` branch covers them:
//!   `8 * (cnt - 1) + 4` `other_bits`, "bits to be discarded by the
//!   decoder", returning `cnt`. They parse as
//!   [`ExtensionPayload::Reserved`].
//!
//! The SBR-data extension types defined by ISO/IEC 13818-7 Table 40
//! are surfaced as [`Error::UnsupportedExtensionSbr`] by the default
//! [`ExtensionPayload::parse`] (so the byte-exact AAC-LC decode path
//! stays untouched). The dedicated [`ExtensionPayload::parse_with_sbr`]
//! entry instead routes them into the §4.4.2.8
//! [`crate::sbr_extension::SbrExtensionData`] side-info walker (the SBR
//! back-end DSP is still not applied):
//!
//! * `EXT_SBR_DATA` (`0b1101`).
//! * `EXT_SBR_DATA_CRC` (`0b1110`).
//!
//! ## Why a parser / writer pair, and why now
//!
//! The Phase 1 `raw_data_block()` walker (round 121) recognises
//! FIL but skips its payload bytes opaque. Round 160's
//! `FrameAssembler::push_fill` accepts an opaque payload byte
//! slice. Neither side decodes or encodes the structured
//! `extension_payload()` body — and the FIL element is where the
//! DRC metadata (per-band gain factors), encoder-identifier fill
//! bytes, and the SBR enhancement bytes ride. This module is
//! the §4.4.2.7 wire-level decode/encode for the three non-SBR
//! extension types whose body layouts are fully specified by
//! fixed-width fields (no Huffman, no spectral context). The
//! intent is that downstream rounds plug this module into
//! `FrameAssembler::push_fill` /
//! `Walker::next_element` to surface a typed `extension_payload`
//! per FIL element.
//!
//! ## Returned byte count
//!
//! Per Table 4.51, `extension_payload()` returns the byte count
//! it consumed. Table 4.52's `dynamic_range_info()` returns its
//! own byte count starting from `n = 1` (the leading byte
//! containing the 4-bit `extension_type` nibble plus four of the
//! body's "presence" flags); each subsequent 8-bit-wide field set
//! is `n++`. [`ExtensionPayload::parse`] and [`ExtensionPayload::write`]
//! both expose this byte count via the returned
//! [`ExtensionPayload::bytes_consumed`] / [`ExtensionPayload::byte_length`]
//! accessors.
//!
//! ## What this module does *not* cover
//!
//! * No application of the DRC `(dyn_rng_sgn, dyn_rng_ctl)` gain
//!   factors to the reconstructed audio. §4.5.2.13 specifies the
//!   companding curve; this module surfaces the raw fields only.
//! * No semantic validation of `pce_instance_tag` against the
//!   surrounding PCE (the surrounding PCE may not be known at
//!   parse time — e.g. when the DRC FIL precedes the PCE in
//!   independent-program multiplexes).
//! * The SBR-data extension types (Table 40
//!   `EXT_SBR_DATA` / `EXT_SBR_DATA_CRC`) are surfaced as
//!   [`Error::UnsupportedExtensionSbr`] — their bodies are the
//!   `sbr_extension_data()` syntax which needs the QMF / patching
//!   back-end. This module's writer / parser deliberately does
//!   *not* consume bits for these types so a future SBR round can
//!   take over without a wire-format incompatibility.

use oxideav_core::bits::{BitReader, BitWriter};

use crate::{Error, Result};

/// Width in bits of the wire `extension_type` field. ISO/IEC
/// 14496-3 Table 4.51.
pub const EXTENSION_TYPE_BITS: u32 = 4;

/// Symbolic `extension_type` values per ISO/IEC 14496-3 Table 4.59
/// plus the ISO/IEC 13818-7 Table 40 SBR-data extensions.
///
/// Every variant maps to a single 4-bit wire value; the raw value
/// is exposed via [`ExtensionType::as_u8`] for round-tripping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ExtensionType {
    /// `EXT_FILL` (`0b0000`) — bitstream filler.
    Fill,
    /// `EXT_FILL_DATA` (`0b0001`) — bitstream data as filler.
    /// Normative payload: 4-bit `fill_nibble == 0b0000` followed
    /// by `cnt - 1` × 8-bit `fill_byte == 0b10100101`.
    FillData,
    /// `EXT_DYNAMIC_RANGE` (`0b1011`) — dynamic range control.
    /// Body is the Table 4.52 `dynamic_range_info()` block.
    DynamicRange,
    /// `EXT_SBR_DATA` (`0b1101`) — SBR enhancement (ISO/IEC
    /// 13818-7 Table 40). This crate does not parse the
    /// `sbr_extension_data()` body yet.
    SbrData,
    /// `EXT_SBR_DATA_CRC` (`0b1110`) — SBR enhancement with CRC
    /// (ISO/IEC 13818-7 Table 40). This crate does not parse the
    /// `sbr_extension_data()` body yet.
    SbrDataCrc,
    /// `EXT_DATA_ELEMENT` (`0b0010`) — data element (ISO/IEC
    /// 14496-3:2009 Table 4.121).
    DataElement,
    /// `EXT_SAC_DATA` (`0b1100`) — MPEG Surround side information
    /// (ISO/IEC 14496-3:2009 Table 4.121).
    SacData,
    /// A reserved value ("can be used for a further extension of the
    /// syntax in a compatible way"). Carries the 4-bit wire value.
    /// The syntax table's `default` branch gives these the same body
    /// as `EXT_FILL`: `8 * (cnt - 1) + 4` discarded `other_bits`.
    Reserved(u8),
}

impl ExtensionType {
    /// Map a 4-bit wire value (`0..=15`) to the corresponding
    /// [`ExtensionType`], or surface a structural error.
    ///
    /// Returns:
    ///
    /// * [`Error::UnsupportedExtensionSbr`] for `0b1101`
    ///   (`EXT_SBR_DATA`) and `0b1110` (`EXT_SBR_DATA_CRC`) — the
    ///   bodies are the SBR `sbr_extension_data()` syntax, which
    ///   only [`ExtensionPayload::parse_with_sbr`] walks.
    /// * [`ExtensionType::Reserved`] for every value Table 4.121
    ///   lists as reserved — these have a body layout (the syntax
    ///   table's `default` branch) and are not an error.
    /// * [`Error::UnsupportedExtensionType`] only for a `value`
    ///   above `0x0f`, which is not a 4-bit field value at all.
    pub fn from_bits(value: u8) -> Result<Self> {
        match value {
            0b1101 | 0b1110 => Err(Error::UnsupportedExtensionSbr(value)),
            other => Self::from_bits_allow_sbr(other),
        }
    }

    /// Like [`Self::from_bits`] but maps the two SBR wire values to
    /// their [`ExtensionType`] variants instead of an error, so the
    /// [`ExtensionPayload::parse_with_sbr`] entry can dispatch them into
    /// the SBR side-info walker.
    pub fn from_bits_allow_sbr(value: u8) -> Result<Self> {
        match value {
            0b0000 => Ok(ExtensionType::Fill),
            0b0001 => Ok(ExtensionType::FillData),
            0b0010 => Ok(ExtensionType::DataElement),
            0b1011 => Ok(ExtensionType::DynamicRange),
            0b1100 => Ok(ExtensionType::SacData),
            0b1101 => Ok(ExtensionType::SbrData),
            0b1110 => Ok(ExtensionType::SbrDataCrc),
            other if other <= 0x0f => Ok(ExtensionType::Reserved(other)),
            other => Err(Error::UnsupportedExtensionType(other)),
        }
    }

    /// Whether Table 4.121 lists this value as reserved.
    pub fn is_reserved(self) -> bool {
        matches!(self, ExtensionType::Reserved(_))
    }

    /// Convert back to the 4-bit wire value used by Table 4.51.
    pub fn as_u8(self) -> u8 {
        match self {
            ExtensionType::Fill => 0b0000,
            ExtensionType::FillData => 0b0001,
            ExtensionType::DynamicRange => 0b1011,
            ExtensionType::SbrData => 0b1101,
            ExtensionType::SbrDataCrc => 0b1110,
            ExtensionType::DataElement => 0b0010,
            ExtensionType::SacData => 0b1100,
            ExtensionType::Reserved(value) => value & 0x0f,
        }
    }
}

/// Normative `fill_byte` literal per ISO/IEC 14496-3 §4.4.2.7 /
/// Table 4.51 (`must be '10100101'`). Surfaced as a public constant
/// so callers and tests can refer to the same magic value. This is
/// the value a writer emits; the parser discards the byte unread.
pub const FILL_DATA_BYTE: u8 = 0b1010_0101;

/// Normative `fill_nibble` literal per ISO/IEC 14496-3 §4.4.2.7 /
/// Table 4.51 (`must be '0000'`). Writer-side value, as above.
pub const FILL_DATA_NIBBLE: u8 = 0b0000;

/// `data_element_version` value of an ancillary data element
/// (`ANC_DATA`, ISO/IEC 14496-3:2009 Table 4.122). Every other
/// value is reserved.
pub const DATA_ELEMENT_VERSION_ANC_DATA: u8 = 0b0000;

/// Parsed `extension_payload()` body (Table 4.51 dispatch).
///
/// The body always carries the byte count it consumed (the `n`
/// returned by Table 4.51) so the surrounding FIL `cnt` can be
/// decremented in lockstep with the spec.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum ExtensionPayload {
    /// `EXT_FILL` — opaque filler. Carries the raw `8 * (cnt - 1) + 4`
    /// "other_bits" packed MSB-first into a byte vector. The last
    /// byte's low 4 bits are unused if `cnt > 0` (since the body
    /// is not a whole number of bytes).
    Fill {
        /// Total bytes consumed by this `extension_payload`,
        /// including the 4-bit `extension_type` nibble (so the
        /// useful body is `8 * (cnt - 1) + 4` bits).
        cnt: u32,
        /// `other_bits` packed MSB-first. Empty when `cnt == 1`
        /// (a 4-bit-only EXT_FILL whose body is 4 unused bits).
        other_bits: Vec<u8>,
    },
    /// `EXT_FILL_DATA` — pattern filler. Carries the byte count
    /// (FIL `cnt`) so the body length is implicit. The body is
    /// "to be discarded by the decoder": the parser does not keep it
    /// and does not compare it with the pattern an encoder shall
    /// write; the writer emits that pattern.
    FillData {
        /// Total bytes consumed (the FIL `cnt`).
        cnt: u32,
    },
    /// `EXT_DYNAMIC_RANGE` — DRC metadata per Table 4.52.
    DynamicRange(DynamicRangeInfo),
    /// `EXT_DATA_ELEMENT` with `data_element_version == ANC_DATA`
    /// — an ancillary data element. Occupies
    /// `1 + (data.len() / 255 + 1) + data.len()` bytes, which may be
    /// fewer than the FIL `cnt` (the fill element's loop then reads
    /// the next `extension_payload()`).
    AncillaryData {
        /// The `data_element_byte`s, in wire order.
        data: Vec<u8>,
    },
    /// `EXT_DATA_ELEMENT` with a reserved `data_element_version`
    /// (`1..=15`): `align = 0`, then the `default` branch's
    /// `8 * (cnt - 1)` `other_bits`. Occupies
    /// `1 + other_bytes.len()` bytes.
    DataElementReserved {
        /// The 4-bit `data_element_version` (never `ANC_DATA`).
        version: u8,
        /// The `cnt - 1` bytes of `other_bits`.
        other_bytes: Vec<u8>,
    },
    /// `EXT_SAC_DATA` — `sac_extension_data(cnt)` (ISO/IEC
    /// 14496-3:2009 Table 4.61), carried verbatim. Occupies
    /// `1 + data.len()` bytes.
    SacData {
        /// `ancType` — 2 bits.
        anc_type: u8,
        /// `ancStart`.
        anc_start: bool,
        /// `ancStop`.
        anc_stop: bool,
        /// The `cnt - 1` `ancDataSegmentByte`s.
        data: Vec<u8>,
    },
    /// A reserved `extension_type`: the syntax table's `default`
    /// branch, same shape as [`ExtensionPayload::Fill`].
    Reserved {
        /// The 4-bit `extension_type` as read from the wire.
        extension_type: u8,
        /// Total bytes consumed (the FIL `cnt`).
        cnt: u32,
        /// The `8 * (cnt - 1) + 4` `other_bits`, packed MSB-first
        /// (the last byte's low 4 bits are unused).
        other_bits: Vec<u8>,
    },
}

/// The result of [`ExtensionPayload::parse_with_sbr`]: either a standard
/// (non-SBR) extension payload, or a decoded SBR side-info element.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum ExtensionPayloadOrSbr {
    /// A non-SBR extension payload (`EXT_FILL` / `EXT_FILL_DATA` /
    /// `EXT_DYNAMIC_RANGE`).
    Payload(ExtensionPayload),
    /// A decoded `sbr_extension_data()` (`EXT_SBR_DATA` /
    /// `EXT_SBR_DATA_CRC`). Boxed because the SBR side-info element is
    /// much larger than the other variants.
    Sbr(Box<crate::sbr_extension::SbrExtensionData>),
    /// An SBR payload received **before any `sbr_header()`** — the
    /// stream opens with `bs_header_flag == 0` payloads and no header
    /// has been threaded yet. Per ISO/IEC 14496-3:2009 §4.5.2.8.1
    /// ("As long as no SBR header part is present, the SBR decoder
    /// performs upsampling and delay adjustment only") the `sbr_data()`
    /// body cannot be parsed (its band tables come from the missing
    /// header), so the payload is skipped whole; the caller should run
    /// the §4.6.18.5 pure-upsampling path for the covered element. The
    /// ISO/IEC 14496-26 `al_sbr_{e,i}_32_*` conformance vectors open
    /// this way.
    ///
    /// `crc` / `crc_region` carry the `EXT_SBR_DATA_CRC` checksum and
    /// its covered bit range (everything after the 10-bit CRC field up
    /// to the end of the fill payload, per the §4.5.2.8.1 coverage
    /// statement — with no parsed `sbr_data()` the `bs_fill_bits`
    /// boundary is unknowable, and the whole-payload region is the
    /// normative coverage); `None` for the plain `EXT_SBR_DATA` type.
    SbrPreHeader {
        /// Transmitted `bs_sbr_crc_bits`, when the CRC variant.
        crc: Option<u16>,
        /// Covered `[start, end)` bit range in the parse buffer.
        crc_region: Option<(u64, u64)>,
    },
}

/// Parsed `dynamic_range_info()` body (Table 4.52). All fields are
/// surfaced verbatim from the wire; the §4.5.2.13 companding curve
/// that maps `(dyn_rng_sgn, dyn_rng_ctl)` pairs to dB attenuations
/// is *not* applied here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DynamicRangeInfo {
    /// Optional PCE element-tag selector. `Some((tag, reserved))`
    /// when `pce_tag_present == 1`. Both fields are 4 bits.
    pub pce_tag: Option<PceTagFields>,
    /// Optional excluded-channels list. `Some(_)` when
    /// `excluded_chns_present == 1`.
    pub excluded_channels: Option<ExcludedChannels>,
    /// Optional per-band partitioning. `Some(_)` when
    /// `drc_bands_present == 1`. When `None`, the spec sets
    /// `drc_num_bands = 1` and there is a single
    /// `(dyn_rng_sgn[0], dyn_rng_ctl[0])` pair below.
    pub drc_bands: Option<DrcBands>,
    /// Optional 7-bit `prog_ref_level` reference level
    /// (`Some((level, reserved))` when `prog_ref_level_present
    /// == 1`). `reserved` is the trailing 1-bit reserved field.
    pub prog_ref_level: Option<ProgRefLevelFields>,
    /// Per-band `(dyn_rng_sgn, dyn_rng_ctl)` records, in wire
    /// order. Length equals the resolved `drc_num_bands`
    /// (`drc_bands.is_none()` ⇒ 1; otherwise
    /// `1 + drc_bands.band_incr`).
    pub bands: Vec<DrcBandRecord>,
}

/// 4-bit `pce_instance_tag` + 4-bit `drc_tag_reserved_bits` pair
/// per Table 4.52.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PceTagFields {
    /// `pce_instance_tag` — selects the surrounding PCE this DRC
    /// applies to. 4 bits.
    pub pce_instance_tag: u8,
    /// `drc_tag_reserved_bits` — 4 bits, value not constrained by
    /// the spec.
    pub reserved: u8,
}

/// `excluded_channels()` body (Table 4.53). Carries the resolved
/// `exclude_mask[]` bits packed MSB-first into a `Vec<bool>`. The
/// wire length is implied by the trailing
/// `additional_excluded_chns[n-1] == 0` flag — every 7
/// `exclude_mask` bits are followed by a 1-bit continuation flag,
/// repeating until the continuation flag reads 0.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExcludedChannels {
    /// `exclude_mask[i]` for `i = 0..(7 * n_groups)`, where
    /// `n_groups` is the number of 8-bit-wide groups consumed.
    pub exclude_mask: Vec<bool>,
}

/// `drc_band_incr` + `drc_bands_reserved_bits` + `drc_band_top[]`
/// payload per Table 4.52.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DrcBands {
    /// `drc_band_incr` — 4 bits. Resolved
    /// `drc_num_bands = 1 + drc_band_incr`.
    pub band_incr: u8,
    /// `drc_bands_reserved_bits` — 4 bits, value not constrained
    /// by the spec.
    pub reserved: u8,
    /// `drc_band_top[i]` — 8 bits per band. Length equals
    /// `1 + band_incr`.
    pub band_top: Vec<u8>,
}

/// 7-bit `prog_ref_level` + 1-bit `prog_ref_level_reserved_bits`
/// pair per Table 4.52.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProgRefLevelFields {
    /// `prog_ref_level` — 7 bits. Reference level for downstream
    /// loudness normalisation.
    pub level: u8,
    /// `prog_ref_level_reserved_bits` — 1 bit.
    pub reserved: bool,
}

/// Per-band `(dyn_rng_sgn, dyn_rng_ctl)` pair per Table 4.52.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DrcBandRecord {
    /// `dyn_rng_sgn[i]` — 1-bit sign. `true` ⇒ negative gain (cut).
    pub dyn_rng_sgn: bool,
    /// `dyn_rng_ctl[i]` — 7-bit magnitude in 0.25 dB steps per
    /// §4.5.2.13. Surfaced verbatim here.
    pub dyn_rng_ctl: u8,
}

impl ExtensionPayload {
    /// Parse an `extension_payload(cnt)` from `reader`.
    ///
    /// `cnt` is the FIL element's payload byte count after the
    /// §4.4.2.7 `esc_count` escape resolution (the same value the
    /// existing [`crate::raw_data_block::Walker`] computes via
    /// `read_fill_count`). `cnt == 0` is rejected as
    /// [`Error::ExtensionPayloadInvalid`] — Table 4.51's
    /// `extension_type` field itself is 4 bits, so a zero-byte FIL
    /// has no room for it.
    ///
    /// Every `extension_type` other than the two SBR values parses
    /// (the SBR values surface [`Error::UnsupportedExtensionSbr`]
    /// here; see [`Self::parse_with_sbr`]). A body that runs past
    /// the end of the reader's buffer is [`Error::UnexpectedEnd`];
    /// one whose own length exceeds `cnt` is
    /// [`Error::ExtensionPayloadInvalid`].
    pub fn parse(reader: &mut BitReader<'_>, cnt: u32) -> Result<Self> {
        if cnt == 0 {
            return Err(Error::ExtensionPayloadInvalid);
        }
        let raw = read_u8(reader, EXTENSION_TYPE_BITS)?;
        parse_body(reader, ExtensionType::from_bits(raw)?, cnt)
    }

    /// Parse an `extension_payload(cnt)`, routing the two SBR extension
    /// types (`EXT_SBR_DATA` / `EXT_SBR_DATA_CRC`) into the
    /// [`crate::sbr_extension::SbrExtensionData`] side-info walker rather
    /// than rejecting them.
    ///
    /// Unlike [`Self::parse`] (which surfaces
    /// [`Error::UnsupportedExtensionSbr`] for the SBR types so the
    /// byte-exact AAC-LC decode path stays untouched), this entry decodes
    /// the SBR bitstream side info: the §4.4.2.8 `sbr_extension_data()`
    /// header + element framing keyed off the surrounding channel
    /// element. The SBR back-end DSP (QMF / HF patching / envelope
    /// adjustment) is still not applied — this only recovers the decoded
    /// side info.
    ///
    /// * `id_aac` — the AAC core element this FIL follows
    ///   ([`crate::raw_data_block::IdSynEle::Sce`] / `Cpe`); selects the
    ///   single- vs pair-element `sbr_data()` dispatch.
    /// * `fs_sbr` — the SBR internal sample rate (twice the core rate).
    /// * `prev_header` — the threaded previous `sbr_header()` for the
    ///   `bs_header_flag == 0` reuse path (`None` on the first payload).
    ///
    /// A non-SBR extension type returns
    /// [`ExtensionPayloadOrSbr::Payload`] with the same body
    /// [`Self::parse`] would produce.
    pub fn parse_with_sbr(
        reader: &mut BitReader<'_>,
        cnt: u32,
        id_aac: crate::raw_data_block::IdSynEle,
        fs_sbr: u32,
        prev_header: Option<crate::sbr_header::SbrHeader>,
    ) -> Result<ExtensionPayloadOrSbr> {
        if cnt == 0 {
            return Err(Error::ExtensionPayloadInvalid);
        }
        let nibble_start = reader.bit_position();
        let raw = read_u8(reader, EXTENSION_TYPE_BITS)?;
        let ty = ExtensionType::from_bits_allow_sbr(raw)?;
        match ty {
            ExtensionType::Fill
            | ExtensionType::FillData
            | ExtensionType::DataElement
            | ExtensionType::DynamicRange
            | ExtensionType::SacData
            | ExtensionType::Reserved(_) => {
                Ok(ExtensionPayloadOrSbr::Payload(parse_body(reader, ty, cnt)?))
            }
            ExtensionType::SbrData | ExtensionType::SbrDataCrc => {
                let crc_flag = ty == ExtensionType::SbrDataCrc;
                if prev_header.is_none() {
                    // Peek the CRC field + bs_header_flag without
                    // committing: a header-less payload before the
                    // first sbr_header() cannot be parsed (§4.5.2.8.1
                    // — upsampling and delay adjustment only), so the
                    // payload is skipped whole with its CRC surfaced.
                    let crc = if crc_flag {
                        Some(reader.read_u32(10).map_err(|_| Error::UnexpectedEnd)? as u16)
                    } else {
                        None
                    };
                    let region_start = reader.bit_position();
                    let header_flag = reader.read_bit().map_err(|_| Error::UnexpectedEnd)?;
                    if !header_flag {
                        let end = nibble_start + u64::from(cnt) * 8;
                        let mut pos = reader.bit_position();
                        while pos < end {
                            let step = (end - pos).min(32) as u32;
                            reader.read_u32(step).map_err(|_| Error::UnexpectedEnd)?;
                            pos += u64::from(step);
                        }
                        return Ok(ExtensionPayloadOrSbr::SbrPreHeader {
                            crc,
                            crc_region: crc.map(|_| (region_start, end)),
                        });
                    }
                    // A header is present after all: re-parse through
                    // the normal path from the header flag onward.
                    let sbr = crate::sbr_extension::SbrExtensionData::parse_after_prefix(
                        reader,
                        id_aac,
                        crc,
                        nibble_start,
                        fs_sbr,
                        Some(cnt),
                        None,
                    )?;
                    return Ok(ExtensionPayloadOrSbr::Sbr(Box::new(sbr)));
                }
                let sbr = crate::sbr_extension::SbrExtensionData::parse(
                    reader,
                    id_aac,
                    crc_flag,
                    fs_sbr,
                    Some(cnt),
                    prev_header,
                )?;
                Ok(ExtensionPayloadOrSbr::Sbr(Box::new(sbr)))
            }
        }
    }

    /// Encode an `extension_payload()` body onto `writer` — the
    /// bit-exact inverse of [`ExtensionPayload::parse`].
    ///
    /// Returns the byte count consumed (matching Table 4.51's
    /// returned `n`). Surfaces caller-side field violations as
    /// [`Error::ExtensionPayloadInvalid`].
    pub fn write(&self, writer: &mut BitWriter) -> Result<u32> {
        match self {
            ExtensionPayload::Fill { cnt, other_bits } => write_fill(writer, *cnt, other_bits),
            ExtensionPayload::FillData { cnt } => write_fill_data(writer, *cnt),
            ExtensionPayload::DynamicRange(drc) => write_dynamic_range(writer, drc),
            ExtensionPayload::AncillaryData { data } => write_ancillary_data(writer, data),
            ExtensionPayload::DataElementReserved {
                version,
                other_bytes,
            } => write_data_element_reserved(writer, *version, other_bytes),
            ExtensionPayload::SacData {
                anc_type,
                anc_start,
                anc_stop,
                data,
            } => write_sac_data(writer, *anc_type, *anc_start, *anc_stop, data),
            ExtensionPayload::Reserved {
                extension_type,
                cnt,
                other_bits,
            } => {
                if !matches!(
                    ExtensionType::from_bits_allow_sbr(*extension_type),
                    Ok(ExtensionType::Reserved(_))
                ) {
                    return Err(Error::ExtensionPayloadInvalid);
                }
                write_other_bits(writer, *extension_type, *cnt, other_bits)
            }
        }
    }

    /// Total byte count this `extension_payload` consumed on the
    /// wire — Table 4.51's returned `n`.
    pub fn byte_length(&self) -> u32 {
        match self {
            ExtensionPayload::Fill { cnt, .. } => *cnt,
            ExtensionPayload::FillData { cnt } => *cnt,
            ExtensionPayload::DynamicRange(drc) => drc.byte_length(),
            ExtensionPayload::AncillaryData { data } => ancillary_data_byte_length(data.len()),
            ExtensionPayload::DataElementReserved { other_bytes, .. } => {
                byte_count(other_bytes.len()).saturating_add(1)
            }
            ExtensionPayload::SacData { data, .. } => byte_count(data.len()).saturating_add(1),
            ExtensionPayload::Reserved { cnt, .. } => *cnt,
        }
    }
}

/// Table 4.51 / 4.57 dispatch on an already-read `extension_type`
/// (never one of the SBR values — both entries route those away).
fn parse_body(reader: &mut BitReader<'_>, ty: ExtensionType, cnt: u32) -> Result<ExtensionPayload> {
    match ty {
        ExtensionType::Fill => {
            let other_bits = read_other_bits(reader, cnt)?;
            Ok(ExtensionPayload::Fill { cnt, other_bits })
        }
        ExtensionType::Reserved(extension_type) => {
            let other_bits = read_other_bits(reader, cnt)?;
            Ok(ExtensionPayload::Reserved {
                extension_type,
                cnt,
                other_bits,
            })
        }
        ExtensionType::FillData => parse_fill_data(reader, cnt),
        ExtensionType::DataElement => parse_data_element(reader, cnt),
        ExtensionType::DynamicRange => parse_dynamic_range(reader, cnt),
        ExtensionType::SacData => parse_sac_data(reader, cnt),
        ExtensionType::SbrData | ExtensionType::SbrDataCrc => {
            Err(Error::UnsupportedExtensionSbr(ty.as_u8()))
        }
    }
}

fn byte_count(len: usize) -> u32 {
    u32::try_from(len).unwrap_or(u32::MAX)
}

/// Bytes an `ANC_DATA` element of `len` data bytes occupies: the
/// `extension_type` + `data_element_version` byte, `loopCounter`
/// length parts, and the data (Table 4.57's
/// `dataElementLength + loopCounter + 1`).
fn ancillary_data_byte_length(len: usize) -> u32 {
    byte_count(len)
        .saturating_add(byte_count(len / 255 + 1))
        .saturating_add(1)
}

impl DynamicRangeInfo {
    /// Byte count this DRC body consumes — Table 4.52's returned
    /// `n`, including the 4-bit `extension_type` nibble that the
    /// outer `extension_payload()` writes immediately before the
    /// DRC body.
    pub fn byte_length(&self) -> u32 {
        // Start from n = 1 (the leading byte containing the 4-bit
        // extension_type + 4 presence flags).
        let mut n: u32 = 1;
        if self.pce_tag.is_some() {
            n += 1;
        }
        if let Some(ex) = &self.excluded_channels {
            // Each group is 7 mask bits + 1 continuation bit = 1 byte.
            n += excluded_group_count(ex.exclude_mask.len()) as u32;
        }
        if let Some(b) = &self.drc_bands {
            // drc_band_incr + reserved = 1 byte, then 1 byte per
            // drc_band_top entry.
            n += 1 + b.band_top.len() as u32;
        }
        if self.prog_ref_level.is_some() {
            n += 1;
        }
        // 1 byte per (dyn_rng_sgn + dyn_rng_ctl).
        n += self.bands.len() as u32;
        n
    }

    /// Resolved `drc_num_bands` per Table 4.52. Always equals
    /// `bands.len()`.
    pub fn num_bands(&self) -> usize {
        self.bands.len()
    }
}

// ===================================================================
// EXT_FILL parser / writer
// ===================================================================

/// The `default` branch of the syntax table with `align = 4`
/// (`EXT_FILL` and every reserved `extension_type`):
///
/// ```text
/// for (i = 0; i < 8*(cnt-1) + align; i++) other_bits[i];
/// return cnt;
/// ```
///
/// The loop counts the bits after the `extension_type` nibble, so
/// the payload occupies `4 + 8*(cnt-1) + 4 = 8 * cnt` bits.
fn read_other_bits(reader: &mut BitReader<'_>, cnt: u32) -> Result<Vec<u8>> {
    read_packed_bits(reader, other_bits_len(cnt)?)
}

fn other_bits_len(cnt: u32) -> Result<u32> {
    8u32.checked_mul(cnt.saturating_sub(1))
        .ok_or(Error::ExtensionPayloadInvalid)?
        .checked_add(4)
        .ok_or(Error::ExtensionPayloadInvalid)
}

fn write_fill(writer: &mut BitWriter, cnt: u32, other_bits: &[u8]) -> Result<u32> {
    write_other_bits(writer, ExtensionType::Fill.as_u8(), cnt, other_bits)
}

fn write_other_bits(
    writer: &mut BitWriter,
    extension_type: u8,
    cnt: u32,
    other_bits: &[u8],
) -> Result<u32> {
    if cnt == 0 {
        return Err(Error::ExtensionPayloadInvalid);
    }
    let body_bits = other_bits_len(cnt)?;
    let expected_bytes = (body_bits as usize).div_ceil(8);
    if other_bits.len() != expected_bytes {
        return Err(Error::ExtensionPayloadInvalid);
    }
    writer.write_u32(u32::from(extension_type), EXTENSION_TYPE_BITS);
    write_packed_bits(writer, other_bits, body_bits)?;
    Ok(cnt)
}

// ===================================================================
// EXT_FILL_DATA parser / writer
// ===================================================================

fn parse_fill_data(reader: &mut BitReader<'_>, cnt: u32) -> Result<ExtensionPayload> {
    // Table 4.51:
    //   fill_nibble;                 4 bits  /* must be '0000' */
    //   for (i = 0; i < cnt - 1; i++)
    //       fill_byte[i];            8 bits  /* must be '10100101' */
    //
    // The "must be" values bind the encoder ("shall be set to").
    // For the decoder the same definition reads "byte to be
    // discarded by the decoder" (ISO/IEC 14496-3:2009 §4.5.2.9.1;
    // ISO/IEC 13818-7 §8.8.2.1), and the pattern exists for clock
    // recovery on self-clocked channels, not as a validity check —
    // so the body is skipped unexamined.
    read_u8(reader, 4)?;
    for _ in 0..cnt.saturating_sub(1) {
        read_u8(reader, 8)?;
    }
    Ok(ExtensionPayload::FillData { cnt })
}

fn write_fill_data(writer: &mut BitWriter, cnt: u32) -> Result<u32> {
    if cnt == 0 {
        return Err(Error::ExtensionPayloadInvalid);
    }
    writer.write_u32(ExtensionType::FillData.as_u8() as u32, EXTENSION_TYPE_BITS);
    writer.write_u32(FILL_DATA_NIBBLE as u32, 4);
    let body_bytes = cnt.saturating_sub(1) as usize;
    for _ in 0..body_bytes {
        writer.write_u32(FILL_DATA_BYTE as u32, 8);
    }
    Ok(cnt)
}

// ===================================================================
// EXT_DATA_ELEMENT parser / writer
// ===================================================================

fn parse_data_element(reader: &mut BitReader<'_>, cnt: u32) -> Result<ExtensionPayload> {
    // ISO/IEC 14496-3:2009 Table 4.57:
    //   data_element_version;                      4 bits
    //   switch (data_element_version) {
    //   case ANC_DATA:
    //       loopCounter = 0; dataElementLength = 0;
    //       do {
    //           dataElementLengthPart;             8 bits
    //           dataElementLength += dataElementLengthPart;
    //           loopCounter++;
    //       } while (dataElementLengthPart == 255);
    //       for (i = 0; i < dataElementLength; i++)
    //           data_element_byte[i];              8 bits
    //       return (dataElementLength + loopCounter + 1);
    //   default:
    //       align = 0;
    //   }
    //   /* falls into the outer default branch */
    //   for (i = 0; i < 8*(cnt-1) + align; i++) other_bits[i];
    //   return cnt;
    let version = read_u8(reader, 4)?;
    if version != DATA_ELEMENT_VERSION_ANC_DATA {
        let mut other_bytes = Vec::new();
        for _ in 0..cnt.saturating_sub(1) {
            other_bytes.push(read_u8(reader, 8)?);
        }
        return Ok(ExtensionPayload::DataElementReserved {
            version,
            other_bytes,
        });
    }
    // Bytes left in the enclosing fill element after the
    // type/version byte. The element's own length may not exceed
    // it: the fill element's `cnt -= extension_payload(cnt)` loop
    // has no meaning for a payload longer than `cnt`.
    let mut budget = cnt - 1;
    let mut length = 0u32;
    loop {
        if budget == 0 {
            return Err(Error::ExtensionPayloadInvalid);
        }
        budget -= 1;
        let part = read_u8(reader, 8)?;
        length = length.saturating_add(u32::from(part));
        if part != 255 {
            break;
        }
    }
    if length > budget {
        return Err(Error::ExtensionPayloadInvalid);
    }
    let mut data = Vec::with_capacity(length as usize);
    for _ in 0..length {
        data.push(read_u8(reader, 8)?);
    }
    Ok(ExtensionPayload::AncillaryData { data })
}

fn write_ancillary_data(writer: &mut BitWriter, data: &[u8]) -> Result<u32> {
    if u32::try_from(data.len()).is_err() {
        return Err(Error::ExtensionPayloadInvalid);
    }
    writer.write_u32(
        u32::from(ExtensionType::DataElement.as_u8()),
        EXTENSION_TYPE_BITS,
    );
    writer.write_u32(u32::from(DATA_ELEMENT_VERSION_ANC_DATA), 4);
    for _ in 0..data.len() / 255 {
        writer.write_u32(255, 8);
    }
    writer.write_u32((data.len() % 255) as u32, 8);
    for &b in data {
        writer.write_u32(u32::from(b), 8);
    }
    Ok(ancillary_data_byte_length(data.len()))
}

fn write_data_element_reserved(
    writer: &mut BitWriter,
    version: u8,
    other_bytes: &[u8],
) -> Result<u32> {
    if version == DATA_ELEMENT_VERSION_ANC_DATA
        || version > 0x0f
        || u32::try_from(other_bytes.len()).is_err()
    {
        return Err(Error::ExtensionPayloadInvalid);
    }
    writer.write_u32(
        u32::from(ExtensionType::DataElement.as_u8()),
        EXTENSION_TYPE_BITS,
    );
    writer.write_u32(u32::from(version), 4);
    for &b in other_bytes {
        writer.write_u32(u32::from(b), 8);
    }
    Ok(byte_count(other_bytes.len()).saturating_add(1))
}

// ===================================================================
// EXT_SAC_DATA parser / writer
// ===================================================================

fn parse_sac_data(reader: &mut BitReader<'_>, cnt: u32) -> Result<ExtensionPayload> {
    // ISO/IEC 14496-3:2009 Table 4.61:
    //   ancType;                                   2 bits
    //   ancStart;                                  1 bit
    //   ancStop;                                   1 bit
    //   for (i = 0; i < cnt-1; i++)
    //       ancDataSegmentByte[i];                 8 bits
    //   return (cnt);
    let anc_type = read_u8(reader, 2)?;
    let anc_start = read_bit(reader)?;
    let anc_stop = read_bit(reader)?;
    let mut data = Vec::new();
    for _ in 0..cnt.saturating_sub(1) {
        data.push(read_u8(reader, 8)?);
    }
    Ok(ExtensionPayload::SacData {
        anc_type,
        anc_start,
        anc_stop,
        data,
    })
}

fn write_sac_data(
    writer: &mut BitWriter,
    anc_type: u8,
    anc_start: bool,
    anc_stop: bool,
    data: &[u8],
) -> Result<u32> {
    if anc_type > 0b11 || u32::try_from(data.len()).is_err() {
        return Err(Error::ExtensionPayloadInvalid);
    }
    writer.write_u32(
        u32::from(ExtensionType::SacData.as_u8()),
        EXTENSION_TYPE_BITS,
    );
    writer.write_u32(u32::from(anc_type), 2);
    writer.write_bit(anc_start);
    writer.write_bit(anc_stop);
    for &b in data {
        writer.write_u32(u32::from(b), 8);
    }
    Ok(byte_count(data.len()).saturating_add(1))
}

// ===================================================================
// EXT_DYNAMIC_RANGE parser / writer
// ===================================================================

fn parse_dynamic_range(reader: &mut BitReader<'_>, cnt: u32) -> Result<ExtensionPayload> {
    let pce_tag_present = read_bit(reader)?;
    let pce_tag = if pce_tag_present {
        let pce_instance_tag = read_u8(reader, 4)?;
        let reserved = read_u8(reader, 4)?;
        Some(PceTagFields {
            pce_instance_tag,
            reserved,
        })
    } else {
        None
    };

    let excluded_chns_present = read_bit(reader)?;
    let excluded_channels = if excluded_chns_present {
        Some(parse_excluded_channels(reader)?)
    } else {
        None
    };

    let drc_bands_present = read_bit(reader)?;
    let drc_bands = if drc_bands_present {
        let band_incr = read_u8(reader, 4)?;
        let reserved = read_u8(reader, 4)?;
        let num_bands = 1usize + band_incr as usize;
        let mut band_top = Vec::with_capacity(num_bands);
        for _ in 0..num_bands {
            band_top.push(read_u8(reader, 8)?);
        }
        Some(DrcBands {
            band_incr,
            reserved,
            band_top,
        })
    } else {
        None
    };

    let prog_ref_level_present = read_bit(reader)?;
    let prog_ref_level = if prog_ref_level_present {
        let level = read_u8(reader, 7)?;
        let reserved = read_bit(reader)?;
        Some(ProgRefLevelFields { level, reserved })
    } else {
        None
    };

    let num_bands = drc_bands
        .as_ref()
        .map(|b| 1 + b.band_incr as usize)
        .unwrap_or(1);
    let mut bands = Vec::with_capacity(num_bands);
    for _ in 0..num_bands {
        let dyn_rng_sgn = read_bit(reader)?;
        let dyn_rng_ctl = read_u8(reader, 7)?;
        bands.push(DrcBandRecord {
            dyn_rng_sgn,
            dyn_rng_ctl,
        });
    }

    let drc = DynamicRangeInfo {
        pce_tag,
        excluded_channels,
        drc_bands,
        prog_ref_level,
        bands,
    };
    if drc.byte_length() != cnt {
        // The dispatching FIL `cnt` and the derived Table 4.52 `n`
        // must agree byte-for-byte — Table 4.52 normatively
        // returns the byte count to the caller. A mismatch
        // indicates a malformed bitstream.
        return Err(Error::ExtensionPayloadInvalid);
    }
    Ok(ExtensionPayload::DynamicRange(drc))
}

fn write_dynamic_range(writer: &mut BitWriter, drc: &DynamicRangeInfo) -> Result<u32> {
    // Caller-side invariant checks (every numeric field cap from
    // Table 4.52).
    if let Some(p) = &drc.pce_tag
        && (p.pce_instance_tag > 0x0f || p.reserved > 0x0f)
    {
        return Err(Error::ExtensionPayloadInvalid);
    }
    if let Some(b) = &drc.drc_bands {
        if b.band_incr > 0x0f || b.reserved > 0x0f {
            return Err(Error::ExtensionPayloadInvalid);
        }
        if b.band_top.len() != 1 + b.band_incr as usize {
            return Err(Error::ExtensionPayloadInvalid);
        }
    }
    if let Some(p) = &drc.prog_ref_level
        && p.level > 0x7f
    {
        return Err(Error::ExtensionPayloadInvalid);
    }
    let expected_bands = drc
        .drc_bands
        .as_ref()
        .map(|b| 1 + b.band_incr as usize)
        .unwrap_or(1);
    if drc.bands.len() != expected_bands {
        return Err(Error::ExtensionPayloadInvalid);
    }
    for r in &drc.bands {
        if r.dyn_rng_ctl > 0x7f {
            return Err(Error::ExtensionPayloadInvalid);
        }
    }

    writer.write_u32(
        ExtensionType::DynamicRange.as_u8() as u32,
        EXTENSION_TYPE_BITS,
    );

    writer.write_bit(drc.pce_tag.is_some());
    if let Some(p) = &drc.pce_tag {
        writer.write_u32(p.pce_instance_tag as u32, 4);
        writer.write_u32(p.reserved as u32, 4);
    }

    writer.write_bit(drc.excluded_channels.is_some());
    if let Some(ex) = &drc.excluded_channels {
        write_excluded_channels(writer, ex)?;
    }

    writer.write_bit(drc.drc_bands.is_some());
    if let Some(b) = &drc.drc_bands {
        writer.write_u32(b.band_incr as u32, 4);
        writer.write_u32(b.reserved as u32, 4);
        for &top in &b.band_top {
            writer.write_u32(top as u32, 8);
        }
    }

    writer.write_bit(drc.prog_ref_level.is_some());
    if let Some(p) = &drc.prog_ref_level {
        writer.write_u32(p.level as u32, 7);
        writer.write_bit(p.reserved);
    }

    for r in &drc.bands {
        writer.write_bit(r.dyn_rng_sgn);
        writer.write_u32(r.dyn_rng_ctl as u32, 7);
    }

    Ok(drc.byte_length())
}

// ===================================================================
// excluded_channels() helper (Table 4.53)
// ===================================================================

fn parse_excluded_channels(reader: &mut BitReader<'_>) -> Result<ExcludedChannels> {
    // Table 4.53: each iteration reads 7 exclude_mask bits + 1
    // additional_excluded_chns continuation flag = 1 byte. Stop
    // when the continuation flag reads 0.
    let mut exclude_mask = Vec::new();
    loop {
        for _ in 0..7 {
            exclude_mask.push(read_bit(reader)?);
        }
        let cont = read_bit(reader)?;
        if !cont {
            break;
        }
    }
    Ok(ExcludedChannels { exclude_mask })
}

fn write_excluded_channels(writer: &mut BitWriter, ex: &ExcludedChannels) -> Result<()> {
    if ex.exclude_mask.is_empty() || !ex.exclude_mask.len().is_multiple_of(7) {
        // Table 4.53 emits exclude_mask bits in fixed groups of 7;
        // any non-multiple-of-7 length cannot round-trip through
        // [`parse_excluded_channels`].
        return Err(Error::ExtensionPayloadInvalid);
    }
    let groups = ex.exclude_mask.len() / 7;
    for g in 0..groups {
        for i in 0..7 {
            writer.write_bit(ex.exclude_mask[g * 7 + i]);
        }
        // The continuation flag is 1 for every group except the
        // last, which carries 0 to terminate.
        let last = g + 1 == groups;
        writer.write_bit(!last);
    }
    Ok(())
}

/// Resolved byte count for an `excluded_channels()` body carrying
/// the given total `exclude_mask` bit count. Exposed so callers can
/// pre-size `cnt` without round-tripping through
/// [`DynamicRangeInfo::byte_length`].
pub fn excluded_group_count(exclude_mask_len: usize) -> usize {
    // The spec emits 7-bit groups; the byte count equals the group
    // count (each group is 7 mask bits + 1 continuation bit).
    exclude_mask_len.div_ceil(7)
}

// ===================================================================
// Helpers
// ===================================================================

fn read_packed_bits(reader: &mut BitReader<'_>, n_bits: u32) -> Result<Vec<u8>> {
    let n_bytes = (n_bits as usize).div_ceil(8);
    let mut out = vec![0u8; n_bytes];
    let mut remaining = n_bits;
    let mut idx = 0;
    while remaining >= 8 {
        out[idx] = read_u8(reader, 8)?;
        idx += 1;
        remaining -= 8;
    }
    if remaining > 0 {
        // Pack the trailing partial byte into the top bits of the
        // last output byte.
        let partial = read_u8(reader, remaining)?;
        out[idx] = partial << (8 - remaining);
    }
    Ok(out)
}

fn write_packed_bits(writer: &mut BitWriter, bytes: &[u8], n_bits: u32) -> Result<()> {
    let mut remaining = n_bits;
    let mut idx = 0;
    while remaining >= 8 {
        writer.write_u32(bytes[idx] as u32, 8);
        idx += 1;
        remaining -= 8;
    }
    if remaining > 0 {
        // The trailing partial byte stores its bits in the top
        // `remaining` bits; recover them with a right-shift.
        let partial = bytes[idx] >> (8 - remaining);
        writer.write_u32(partial as u32, remaining);
    }
    Ok(())
}

fn read_u8(reader: &mut BitReader<'_>, n: u32) -> Result<u8> {
    debug_assert!(n <= 8);
    Ok(reader.read_u32(n).map_err(|_| Error::UnexpectedEnd)? as u8)
}

fn read_bit(reader: &mut BitReader<'_>) -> Result<bool> {
    reader.read_bit().map_err(|_| Error::UnexpectedEnd)
}

//! `program_config_element()` parser.
//!
//! ISO/IEC 14496-3 §4.4.1.1 Table 4.2 (identical to ISO/IEC 13818-7
//! §8.5 Table 25 modulo the field rename `profile` → `object_type`).
//! A PCE describes a custom channel layout — element ordering, per-
//! element CPE/SCE selection, mix-down hints, and a free-form
//! comment field. It is emitted either:
//!
//! * **As the first element of a `raw_data_block()`** (`id_syn_ele ==
//!   PCE`), when the `channelConfiguration` is one of 1..=7 *and* the
//!   encoder wants to override the implicit element layout.
//! * **Inline in [`AudioSpecificConfig`](crate::asc::AudioSpecificConfig)**
//!   when `channelConfiguration == 0`. In that case the PCE has no
//!   surrounding `id_syn_ele` prefix and the byte-alignment Note 1
//!   on Table 4.2 applies *relative to the start of the
//!   `AudioSpecificConfig`*, not to the absolute byte position in
//!   the bitstream.
//!
//! Phase 1 retains the entire PCE structure verbatim (every wire
//! field is preserved) so a later round can validate channel layouts
//! and matrix mix-down semantics without re-parsing.
//!
//! ## Byte alignment
//!
//! The `byte_alignment()` call inside Table 4.2 follows the final
//! `valid_cc_element_tag_select[i]` loop. The position to align *to*
//! depends on the call site:
//!
//! * **Standalone PCE in `raw_data_block()`** ⇒ align to the next
//!   absolute byte boundary of the bit-reader.
//! * **PCE inline in `AudioSpecificConfig`** ⇒ align to the next byte
//!   boundary *relative to the start of the ASC*. Since the ASC
//!   itself usually starts on a byte boundary in the carrying
//!   container (`esds` payload, LATM `StreamMuxConfig`, etc.), the
//!   two definitions usually coincide; they differ only when the
//!   ASC was started at a non-zero bit offset inside a larger
//!   bit-stream. [`Pce::parse`] takes a `relative_origin_bit`
//!   parameter that the caller passes when the ASC origin is not at
//!   the bit-reader's current zero — see
//!   [`AudioSpecificConfig`](crate::asc::AudioSpecificConfig) for
//!   the ASC-origin handling.

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// One entry in a per-element list (`front_element_*`, `side_*`,
/// `back_*`) of a PCE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ElementSelect {
    /// `true` ⇔ the element at this slot is a CPE (channel-pair
    /// element); `false` ⇔ SCE (single-channel element). Matches the
    /// `*_element_is_cpe[i]` wire bit.
    pub is_cpe: bool,
    /// 4-bit `*_element_tag_select[i]` — the
    /// `element_instance_tag` value the matching SCE/CPE will carry
    /// inside the `raw_data_block()`.
    pub tag_select: u8,
}

/// One entry in the `valid_cc_element_*` list of a PCE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CcElementSelect {
    /// `true` ⇔ the coupling channel element is independently
    /// switched (`cc_element_is_ind_sw[i] == 1`).
    pub is_ind_sw: bool,
    /// 4-bit `valid_cc_element_tag_select[i]` — the tag the matching
    /// CCE will carry inside the `raw_data_block()`.
    pub tag_select: u8,
}

/// Parsed `program_config_element()` (Table 4.2).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pce {
    /// 4-bit `element_instance_tag`.
    pub element_instance_tag: u8,
    /// 2-bit `object_type` (ISO/IEC 14496-3) — synonym of `profile`
    /// in ISO/IEC 13818-7. `0` = Main, `1` = LC, `2` = SSR, `3` =
    /// LTP. Note this is **the same scheme as ADTS** (one less than
    /// the `audioObjectType` defined by Table 1.16).
    pub object_type: u8,
    /// 4-bit `sampling_frequency_index` (Table 1.18). The PCE is
    /// allowed to override the surrounding context's
    /// `samplingFrequencyIndex`; in practice the wire value usually
    /// matches the ASC / ADTS value.
    pub sampling_frequency_index: u8,
    /// `num_front_channel_elements` × [`ElementSelect`].
    pub front_elements: Vec<ElementSelect>,
    /// `num_side_channel_elements` × [`ElementSelect`].
    pub side_elements: Vec<ElementSelect>,
    /// `num_back_channel_elements` × [`ElementSelect`].
    pub back_elements: Vec<ElementSelect>,
    /// `num_lfe_channel_elements` × `lfe_element_tag_select[i]`
    /// (4-bit each).
    pub lfe_element_tag_selects: Vec<u8>,
    /// `num_assoc_data_elements` × `assoc_data_element_tag_select[i]`
    /// (4-bit each).
    pub assoc_data_tag_selects: Vec<u8>,
    /// `num_valid_cc_elements` × [`CcElementSelect`].
    pub valid_cc_elements: Vec<CcElementSelect>,
    /// `mono_mixdown_element_number` (4 bits) if
    /// `mono_mixdown_present == 1`.
    pub mono_mixdown_element_number: Option<u8>,
    /// `stereo_mixdown_element_number` (4 bits) if
    /// `stereo_mixdown_present == 1`.
    pub stereo_mixdown_element_number: Option<u8>,
    /// `(matrix_mixdown_idx, pseudo_surround_enable)` if
    /// `matrix_mixdown_idx_present == 1`.
    pub matrix_mixdown: Option<(u8, bool)>,
    /// `comment_field_bytes` raw bytes (after the `byte_alignment()`
    /// and the 8-bit `comment_field_bytes` length prefix).
    pub comment_field: Vec<u8>,
}

impl Pce {
    /// Parse a PCE starting at the current bit-reader position. The
    /// `byte_alignment()` clause inside Table 4.2 will align the
    /// reader to the next byte boundary whose *absolute* bit
    /// position is a multiple of 8 plus `origin_bit_offset`. Pass
    /// `0` for a standalone PCE inside a `raw_data_block()`; pass
    /// the ASC origin bit-position for a PCE inline in
    /// `AudioSpecificConfig` (see module docs).
    pub fn parse(reader: &mut BitReader<'_>, origin_bit_offset: u64) -> Result<Self> {
        // 4 + 2 + 4 = 10 bits of header
        let element_instance_tag = read_u8(reader, 4)?;
        let object_type = read_u8(reader, 2)?;
        let sampling_frequency_index = read_u8(reader, 4)?;

        // Element counts.
        let n_front = read_u8(reader, 4)? as usize;
        let n_side = read_u8(reader, 4)? as usize;
        let n_back = read_u8(reader, 4)? as usize;
        let n_lfe = read_u8(reader, 2)? as usize;
        let n_assoc = read_u8(reader, 3)? as usize;
        let n_cc = read_u8(reader, 4)? as usize;

        // Mix-down presence + bodies.
        let mono_mixdown_present = read_bit(reader)?;
        let mono_mixdown_element_number = if mono_mixdown_present {
            Some(read_u8(reader, 4)?)
        } else {
            None
        };
        let stereo_mixdown_present = read_bit(reader)?;
        let stereo_mixdown_element_number = if stereo_mixdown_present {
            Some(read_u8(reader, 4)?)
        } else {
            None
        };
        let matrix_mixdown_idx_present = read_bit(reader)?;
        let matrix_mixdown = if matrix_mixdown_idx_present {
            let idx = read_u8(reader, 2)?;
            let pseudo = read_bit(reader)?;
            Some((idx, pseudo))
        } else {
            None
        };

        // Element lists.
        let front_elements = read_element_selects(reader, n_front)?;
        let side_elements = read_element_selects(reader, n_side)?;
        let back_elements = read_element_selects(reader, n_back)?;

        let mut lfe_element_tag_selects = Vec::with_capacity(n_lfe);
        for _ in 0..n_lfe {
            lfe_element_tag_selects.push(read_u8(reader, 4)?);
        }
        let mut assoc_data_tag_selects = Vec::with_capacity(n_assoc);
        for _ in 0..n_assoc {
            assoc_data_tag_selects.push(read_u8(reader, 4)?);
        }
        let mut valid_cc_elements = Vec::with_capacity(n_cc);
        for _ in 0..n_cc {
            let is_ind_sw = read_bit(reader)?;
            let tag_select = read_u8(reader, 4)?;
            valid_cc_elements.push(CcElementSelect {
                is_ind_sw,
                tag_select,
            });
        }

        // §4.4.1.1 Note 1: byte_alignment() relative to the PCE's
        // origin reference. The "next byte boundary" is determined
        // by the absolute reader position minus `origin_bit_offset`
        // — when the offset is 0, this collapses to the standard
        // `align_to_byte()`.
        align_relative_to_origin(reader, origin_bit_offset)?;

        let comment_field_bytes = read_u8(reader, 8)? as usize;
        let mut comment_field = Vec::with_capacity(comment_field_bytes);
        for _ in 0..comment_field_bytes {
            comment_field.push(read_u8(reader, 8)?);
        }

        Ok(Pce {
            element_instance_tag,
            object_type,
            sampling_frequency_index,
            front_elements,
            side_elements,
            back_elements,
            lfe_element_tag_selects,
            assoc_data_tag_selects,
            valid_cc_elements,
            mono_mixdown_element_number,
            stereo_mixdown_element_number,
            matrix_mixdown,
            comment_field,
        })
    }

    /// Total channel count implied by this PCE — sums one channel
    /// for each SCE entry and two channels for each CPE entry across
    /// front/side/back lists, plus one channel per LFE entry. (CCEs
    /// are coupling buses and do not contribute to the output
    /// channel count.)
    pub fn channel_count(&self) -> usize {
        let count_list = |list: &[ElementSelect]| -> usize {
            list.iter().map(|e| if e.is_cpe { 2 } else { 1 }).sum()
        };
        count_list(&self.front_elements)
            + count_list(&self.side_elements)
            + count_list(&self.back_elements)
            + self.lfe_element_tag_selects.len()
    }
}

fn read_element_selects(reader: &mut BitReader<'_>, n: usize) -> Result<Vec<ElementSelect>> {
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let is_cpe = read_bit(reader)?;
        let tag_select = read_u8(reader, 4)?;
        out.push(ElementSelect { is_cpe, tag_select });
    }
    Ok(out)
}

fn read_u8(reader: &mut BitReader<'_>, n: u32) -> Result<u8> {
    debug_assert!(n <= 8);
    Ok(reader.read_u32(n).map_err(|_| Error::UnexpectedEnd)? as u8)
}

fn read_bit(reader: &mut BitReader<'_>) -> Result<bool> {
    reader.read_bit().map_err(|_| Error::UnexpectedEnd)
}

/// Align the reader to the next byte boundary measured from
/// `origin_bit_offset`. Equivalent to `align_to_byte()` when
/// `origin_bit_offset == 0`.
fn align_relative_to_origin(reader: &mut BitReader<'_>, origin_bit_offset: u64) -> Result<()> {
    let cur = reader.bit_position();
    let from_origin = cur.saturating_sub(origin_bit_offset);
    let pad = (8 - (from_origin % 8)) % 8;
    if pad == 0 {
        return Ok(());
    }
    reader.skip(pad as u32).map_err(|_| Error::UnexpectedEnd)?;
    Ok(())
}

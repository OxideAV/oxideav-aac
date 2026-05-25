//! `section_data()` parser — ISO/IEC 14496-3 §4.4.6 / ISO/IEC
//! 13818-7 §6.3 Table 17.
//!
//! `section_data()` is the second tool inside
//! `individual_channel_stream()` (after `global_gain` and
//! `ics_info()`, before `scale_factor_data()`). It assigns one
//! Huffman codebook (`sect_cb`) to each *run* of scalefactor bands
//! (a "section") within each window group, using run-length coding
//! with an escape mechanism for sections longer than the field can
//! hold in one increment.
//!
//! This parser depends only on values already produced by
//! [`crate::ics_info::IcsInfo`]:
//!
//! * `num_window_groups` — the outer loop bound.
//! * `max_sfb` — the inner loop terminator (`while (k < max_sfb)`).
//! * `window_sequence == EIGHT_SHORT_SEQUENCE` — selects the
//!   3-bit (`sect_esc_val = 7`) versus 5-bit (`sect_esc_val = 31`)
//!   `sect_len_incr` field width.
//!
//! Crucially it carries **no Huffman codebook of its own**: every
//! field is fixed-width (`sect_cb` is 4 bits, `sect_len_incr` is
//! 3 or 5 bits), so the parser is a pure bit-walker. The Huffman
//! codebooks the `sect_cb` values *select* (the spectrum books 1-11
//! plus the scalefactor book) are consumed by later tools
//! (`scale_factor_data()`, `spectral_data()`), not here.
//!
//! ## Run-length escape coding (Table 17)
//!
//! For each window group `g`, starting at scalefactor band `k = 0`:
//!
//! 1. Read `sect_cb[g][i]` (4 bits).
//! 2. Set `sect_len = 0`. Read `sect_len_incr` (3 or 5 bits).
//!    While the value read equals `sect_esc_val`, add `sect_esc_val`
//!    to `sect_len` and read the next `sect_len_incr`. When a
//!    non-escape value is read, add it to `sect_len` and stop.
//! 3. The section covers bands `[k, k + sect_len)`. Record
//!    `sect_start[g][i] = k`, `sect_end[g][i] = k + sect_len`, and
//!    `sfb_cb[g][sfb] = sect_cb[g][i]` for every band in the run.
//! 4. Advance `k += sect_len`, `i += 1`. Repeat while `k < max_sfb`.
//!
//! `num_sec[g]` is the final value of `i` for the group.
//!
//! ## What is *not* in this round
//!
//! * No Huffman decode. The codebook indices are surfaced verbatim;
//!   the spectrum / scalefactor decoders consume them later.
//! * No `is_intensity()` / PNS classification. The
//!   [`Codebook`] enum exposes the semantic role of each value
//!   (`Intensity`, `IntensityInPhase`, `Noise`, `Esc`, …) for the
//!   benefit of `scale_factor_data()` / `spectral_data()`, but
//!   `section_data()` itself only records the raw `u8`.
//! * No validation that `sfb_cb` is fully populated to `max_sfb` in
//!   pathological streams — the parser surfaces a
//!   [`Error::SectionDataOverrun`] when a section would extend past
//!   `max_sfb` (which a conforming encoder never emits) and
//!   otherwise trusts the run lengths.

use oxideav_core::bits::BitReader;

use crate::ics_info::WindowSequence;
use crate::{Error, Result};

/// `ZERO_HCB` — section carries neither scalefactor nor spectral
/// data; the band is silent. ISO/IEC 13818-7 §9.2.2 / §11.3.2.
pub const ZERO_HCB: u8 = 0;

/// `FIRST_PAIR_HCB` — the first codebook whose dimension is 2
/// (a 2-tuple); books `< FIRST_PAIR_HCB` are 4-tuple (QUAD) books.
/// ISO/IEC 13818-7 §9.2.2.
pub const FIRST_PAIR_HCB: u8 = 5;

/// `ESC_HCB` — the spectrum escape codebook (book 11). Values whose
/// magnitude reaches the LAV use the §9.3 escape sequence for the
/// actual coefficient. ISO/IEC 13818-7 §9.2.2.
pub const ESC_HCB: u8 = 11;

/// `NOISE_HCB` — Perceptual Noise Substitution codebook (value 13).
/// An MPEG-4 extension (ISO/IEC 14496-3; the base ISO/IEC 13818-7
/// Table 59 marks value 13 *reserved* and adds PNS in its Annex B
/// Table B.1 extended `scale_factor_data()`). When a band's
/// `sfb_cb == NOISE_HCB` the band is noise-filled and its
/// "scalefactor" position carries the PNS energy delta instead.
pub const NOISE_HCB: u8 = 13;

/// `INTENSITY_HCB2` — out-of-phase intensity-stereo codebook
/// (value 14). ISO/IEC 13818-7 §9.2.2 / Table 59.
pub const INTENSITY_HCB2: u8 = 14;

/// `INTENSITY_HCB` — in-phase intensity-stereo codebook (value 15).
/// ISO/IEC 13818-7 §9.2.2 / Table 59.
pub const INTENSITY_HCB: u8 = 15;

/// Semantic classification of a 4-bit `sect_cb` value, per ISO/IEC
/// 13818-7 Table 59 (extended by the MPEG-4 PNS codebook 13).
///
/// `section_data()` records the raw `u8` in [`Section::codebook`];
/// this enum is a *view* over that value so downstream tools
/// (`scale_factor_data()` for the `is_intensity` / PNS branch,
/// `spectral_data()` for the dimension / signed / escape branch)
/// can dispatch without re-deriving the classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Codebook {
    /// `0` — `ZERO_HCB`: silent band, no scalefactor, no spectrum.
    Zero,
    /// `1..=4` — 4-tuple (QUAD) spectrum book. `signed` is `false`
    /// for books 1-2 (`unsigned_cb == 0`) and `true` for 3-4.
    Quad {
        /// Codebook number (1..=4).
        number: u8,
        /// `true` ⇔ the book is *unsigned* (`unsigned_cb[i] == 1`).
        unsigned: bool,
    },
    /// `5..=10` — 2-tuple (PAIR) spectrum book.
    Pair {
        /// Codebook number (5..=10).
        number: u8,
        /// `true` ⇔ the book is *unsigned* (`unsigned_cb[i] == 1`).
        unsigned: bool,
    },
    /// `11` — `ESC_HCB`: 2-tuple unsigned escape book.
    Esc,
    /// `12` — reserved (ISO/IEC 13818-7 Table 59).
    Reserved12,
    /// `13` — `NOISE_HCB`: Perceptual Noise Substitution (MPEG-4).
    Noise,
    /// `14` — `INTENSITY_HCB2`: out-of-phase intensity stereo.
    IntensityOutOfPhase,
    /// `15` — `INTENSITY_HCB`: in-phase intensity stereo.
    IntensityInPhase,
}

impl Codebook {
    /// Classify a raw 4-bit `sect_cb` value (0..=15).
    ///
    /// `unsigned_cb[]` per ISO/IEC 13818-7 Table 59: books 1, 2 are
    /// signed (`unsigned == false`); books 3, 4, 5*, 6*, 7, 8, 9,
    /// 10, 11 are unsigned. (*Books 5 and 6 are 2-tuple signed in
    /// Table 59 — see the per-number mapping below.)
    pub fn from_value(value: u8) -> Self {
        match value & 0x0f {
            0 => Codebook::Zero,
            // QUAD books (dimension 4): 1, 2 signed; 3, 4 unsigned.
            n @ 1..=4 => Codebook::Quad {
                number: n,
                unsigned: matches!(n, 3 | 4),
            },
            // PAIR books (dimension 2): 5, 6 signed; 7, 8, 9, 10
            // unsigned.
            n @ 5..=10 => Codebook::Pair {
                number: n,
                unsigned: matches!(n, 7..=10),
            },
            11 => Codebook::Esc,
            12 => Codebook::Reserved12,
            13 => Codebook::Noise,
            14 => Codebook::IntensityOutOfPhase,
            15 => Codebook::IntensityInPhase,
            _ => unreachable!("masked to 0..=15"),
        }
    }

    /// `true` ⇔ this codebook is an intensity-stereo book
    /// (`INTENSITY_HCB` or `INTENSITY_HCB2`). Mirrors the spec
    /// `is_intensity()` helper used by `scale_factor_data()`.
    pub fn is_intensity(self) -> bool {
        matches!(
            self,
            Codebook::IntensityInPhase | Codebook::IntensityOutOfPhase
        )
    }

    /// `true` ⇔ this is the PNS noise codebook (`NOISE_HCB`).
    pub fn is_noise(self) -> bool {
        matches!(self, Codebook::Noise)
    }

    /// `true` ⇔ this is `ZERO_HCB` (band carries no data).
    pub fn is_zero(self) -> bool {
        matches!(self, Codebook::Zero)
    }
}

/// One contiguous run of scalefactor bands sharing a codebook, as
/// produced by Table 17. `start`/`end` are scalefactor-band indices
/// (`end` is one past the last band, matching `sect_end`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Section {
    /// `sect_cb[g][i]` — the raw 4-bit codebook value for this run.
    pub codebook: u8,
    /// `sect_start[g][i]` — first scalefactor band in the section.
    pub start: u8,
    /// `sect_end[g][i]` — one past the last band (`start +
    /// sect_len`).
    pub end: u8,
}

impl Section {
    /// Length of the section in scalefactor bands (`sect_len`).
    pub fn len(self) -> u8 {
        self.end - self.start
    }

    /// `true` ⇔ the section spans zero bands. A conforming encoder
    /// never emits a zero-length section, but the accessor is
    /// provided so the `clippy::len_without_is_empty` lint is
    /// satisfied and callers can defensively check.
    pub fn is_empty(self) -> bool {
        self.end == self.start
    }

    /// Semantic [`Codebook`] classification of [`Self::codebook`].
    pub fn codebook_kind(self) -> Codebook {
        Codebook::from_value(self.codebook)
    }
}

/// Parsed `section_data()` for one `individual_channel_stream()`.
///
/// The per-group section lists plus the flattened `sfb_cb[g][sfb]`
/// map are surfaced; `scale_factor_data()` (next round) consumes
/// `sfb_cb` to decide which bands carry a transmitted scalefactor.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SectionData {
    /// `sect[g]` — the ordered sections of window group `g`. The
    /// outer index runs `0..num_window_groups`; `sect[g].len()` is
    /// `num_sec[g]`.
    pub sections: Vec<Vec<Section>>,
    /// `sfb_cb[g][sfb]` — the codebook assigned to scalefactor band
    /// `sfb` of group `g`, for `sfb in 0..max_sfb`. Flattened per
    /// group; the outer index runs `0..num_window_groups`.
    pub sfb_cb: Vec<Vec<u8>>,
}

impl SectionData {
    /// Parse a `section_data()` from the bit-reader.
    ///
    /// * `reader` — positioned immediately after `ics_info()` (well,
    ///   after `global_gain` + `ics_info()` in the full ICS, but
    ///   `section_data()` starts right where the caller leaves the
    ///   reader).
    /// * `window_sequence` — from the surrounding `ics_info()`;
    ///   selects the 3-bit vs 5-bit `sect_len_incr` field.
    /// * `num_window_groups` — from the surrounding `ics_info()`
    ///   derivations (`1` for long sequences).
    /// * `max_sfb` — from the surrounding `ics_info()`.
    ///
    /// Returns [`Error::SectionDataOverrun`] if a section run would
    /// extend past `max_sfb` (non-conforming stream), and
    /// [`Error::UnexpectedEnd`] on bit-reader underflow.
    pub fn parse(
        reader: &mut BitReader<'_>,
        window_sequence: WindowSequence,
        num_window_groups: u8,
        max_sfb: u8,
    ) -> Result<Self> {
        // Table 17: sect_esc_val and sect_len_incr field width.
        let (sect_esc_val, len_bits) = if window_sequence.is_eight_short() {
            ((1u32 << 3) - 1, 3u32) // 7, 3-bit field
        } else {
            ((1u32 << 5) - 1, 5u32) // 31, 5-bit field
        };

        let mut sections: Vec<Vec<Section>> = Vec::with_capacity(num_window_groups as usize);
        let mut sfb_cb: Vec<Vec<u8>> = Vec::with_capacity(num_window_groups as usize);

        for _g in 0..num_window_groups {
            let mut group_sections: Vec<Section> = Vec::new();
            let mut group_sfb_cb: Vec<u8> = vec![ZERO_HCB; max_sfb as usize];

            let mut k: u32 = 0;
            let max = max_sfb as u32;
            while k < max {
                let sect_cb = read_u8(reader, 4)?;

                // sect_len accumulation with escape coding.
                let mut sect_len: u32 = 0;
                loop {
                    let incr = reader
                        .read_u32(len_bits)
                        .map_err(|_| Error::UnexpectedEnd)?;
                    if incr == sect_esc_val {
                        sect_len += sect_esc_val;
                        // Re-read another sect_len_incr.
                        continue;
                    }
                    sect_len += incr;
                    break;
                }

                let start = k;
                let end = k + sect_len;
                if end > max {
                    return Err(Error::SectionDataOverrun);
                }
                for sfb in start..end {
                    group_sfb_cb[sfb as usize] = sect_cb;
                }
                group_sections.push(Section {
                    codebook: sect_cb,
                    start: start as u8,
                    end: end as u8,
                });
                k = end;
            }

            sections.push(group_sections);
            sfb_cb.push(group_sfb_cb);
        }

        Ok(SectionData { sections, sfb_cb })
    }

    /// `num_sec[g]` — number of sections in window group `g`.
    /// Returns `0` for an out-of-range group index.
    pub fn num_sec(&self, group: usize) -> usize {
        self.sections.get(group).map_or(0, Vec::len)
    }
}

fn read_u8(reader: &mut BitReader<'_>, n: u32) -> Result<u8> {
    debug_assert!(n <= 8);
    Ok(reader.read_u32(n).map_err(|_| Error::UnexpectedEnd)? as u8)
}

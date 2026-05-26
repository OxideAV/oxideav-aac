//! `scale_factor_data()` parser + encoder primitive — ISO/IEC 14496-3
//! §4.4.6 / Table 4.53 (non-resilient branch) plus §4.6.3 / Table 4.A.1
//! ("Scalefactor Huffman Codebook" — codebook 12).
//!
//! `scale_factor_data()` is the third tool inside
//! `individual_channel_stream()` (after `global_gain` and
//! `section_data()`, before `pulse_data_present` /
//! `pulse_data()`). For every `(g, sfb)` whose
//! [`section_data`](crate::section_data) classifier picked a non-zero
//! codebook, this tool emits one differentially-coded value (a DPCM
//! delta in the range `-60..=+60`) using the 121-entry Table 4.A.1
//! Huffman codebook. The exception is the **first** Perceptual Noise
//! Substitution (PNS) band of the frame, whose energy delta is sent
//! as a literal 9-bit signed value — every subsequent PNS band falls
//! back to the Huffman path.
//!
//! ## Wire layout (Table 4.53, non-resilient branch)
//!
//! ```text
//! scale_factor_data() {
//!     noise_pcm_flag = 1
//!     for (g = 0; g < num_window_groups; g++) {
//!         for (sfb = 0; sfb < max_sfb; sfb++) {
//!             if (sfb_cb[g][sfb] != ZERO_HCB) {
//!                 if (is_intensity(g, sfb)) {
//!                     hcod_sf[dpcm_is_position[g][sfb]];   1..19 bits
//!                 } else if (is_noise(g, sfb)) {
//!                     if (noise_pcm_flag) {
//!                         noise_pcm_flag = 0
//!                         dpcm_noise_nrg[g][sfb];          9 bits (PCM)
//!                     } else {
//!                         hcod_sf[dpcm_noise_nrg[g][sfb]]; 1..19 bits
//!                     }
//!                 } else {
//!                     hcod_sf[dpcm_sf[g][sfb]];            1..19 bits
//!                 }
//!             }
//!         }
//!     }
//! }
//! ```
//!
//! Three observations the parser and writer both rely on:
//!
//! 1. The outer `(g, sfb)` traversal is **driven by**
//!    [`section_data::SectionData::sfb_cb`](crate::section_data::SectionData::sfb_cb)
//!    — the parser must already know which bands carry a value before
//!    it can decide between "skip", "Huffman value", or "9-bit PCM
//!    energy". The wire stream carries no per-band header that would
//!    let it self-synchronise.
//! 2. The DPCM range is `-60..=+60` (Table 4.150). The Huffman
//!    codebook (Table 4.A.1) has 121 entries indexed `0..=120`; an
//!    `index_offset` of `-60` recovers the signed delta. The codeword
//!    for index 60 (delta 0) is the single bit `0`.
//! 3. `noise_pcm_flag` is **frame-scoped** (not group-scoped): it
//!    starts at `1` at the top of `scale_factor_data()` and clears the
//!    first time a PNS band is emitted, regardless of which window
//!    group or scalefactor band that is.
//!
//! ## What this module covers
//!
//! * [`ScaleFactorData::parse`] — read a non-resilient Table 4.53
//!   block given the surrounding `sfb_cb[g][sfb]` map. Surfaces the
//!   raw transmitted `dpcm_sf` / `dpcm_is_position` deltas and the
//!   `dpcm_noise_nrg` magnitudes verbatim, without applying the
//!   §4.6.2.3.2 running-`last_sf` accumulator (which is a decoder-only
//!   step that needs `global_gain` as the start value).
//! * [`ScaleFactorData::write`] — the inverse: serialise a
//!   [`ScaleFactorData`] bit-for-bit. Surfaces caller-side structural
//!   bugs (delta out of range, PCM energy out of range, missing /
//!   surplus per-band entry versus the `sfb_cb` map) as
//!   [`Error::ScaleFactorDataEncodeInvalid`].
//! * [`hcod_sf_encode`] / [`hcod_sf_decode`] — public Table 4.A.1
//!   accessors for callers (Auditor harnesses, fixture cross-checks)
//!   that need the codebook directly without going through the full
//!   `scale_factor_data()` driver.
//!
//! ## What this module does *not* cover
//!
//! * The §4.6.2.3.2 `last_sf = global_gain` accumulator — that is a
//!   decoder-side step that converts the transmitted DPCM deltas into
//!   absolute `sf[g][sfb]` values in the `0..=255` range. The
//!   accumulator needs `global_gain` and is symmetrically applied at
//!   encode-time before this module is invoked.
//! * The §4.4.6 error-resilient branch (`aacScalefactorDataResilienceFlag
//!   == 1` → RVLC with `rev_global_gain`, `length_of_rvlc_sf`,
//!   `sf_concealment`, `length_of_rvlc_escapes`, etc.) — the in-memory
//!   structure here is the non-resilient flavour. ER AAC-LD / scalable
//!   profiles that flip the resilience flag will need a sibling
//!   `scale_factor_data_rvlc()` module.
//! * The §4.6.13 `dpcm_noise_nrg` reconstruction (running-energy
//!   accumulator that mirrors the scalefactor `last_sf` accumulator
//!   but uses the first 9-bit literal as the seed) — same reasoning:
//!   the wire record is what this module exchanges with the parser;
//!   the seeded-running-difference unwind belongs in the per-AOT
//!   decoder.

use oxideav_core::bits::{BitReader, BitWriter};

use crate::section_data::{INTENSITY_HCB, INTENSITY_HCB2, NOISE_HCB, ZERO_HCB};
use crate::{Error, Result};

// =============================================================================
// Table 4.A.1 — Scalefactor Huffman Codebook (codebook 12)
// =============================================================================
//
// Per Table 4.150, the codebook covers indices 0..=120 with
// `index_offset = -60`, producing DPCM values in `-60..=+60`. The
// table is reproduced verbatim from ISO/IEC 14496-3 §4.A.1 / Table
// 4.A.1 with every length / codeword cross-checked against the
// 13818-7 §11.3.2 / Table 11.3 listing (the two specifications carry
// the same table for backwards bitstream compatibility).
//
// Format: `(length_in_bits, codeword_value)`. Codewords are stored
// right-aligned (the MSB of the wire codeword sits at bit
// `length - 1`), exactly as the Table 4.A.1 hexadecimal column
// presents them.

/// `index_offset` for the scalefactor codebook per Table 4.150
/// (`-60`, surfaced as a signed type because the DPCM range is
/// `-60..=+60`).
pub const SF_INDEX_OFFSET: i8 = -60;

/// `dpcm_noise_nrg` PCM seed width — Table 4.53 `dpcm_noise_nrg`
/// row (9 bits, `uimsbf` in the spec which the §4.6.13 decoder
/// re-interprets as a signed 9-bit delta).
pub const NOISE_PCM_BITS: u32 = 9;

/// Number of entries in Table 4.A.1 (`121`, indices `0..=120`).
pub const HCOD_SF_NUM_ENTRIES: usize = 121;

/// Maximum codeword length emitted by Table 4.A.1 (19 bits).
pub const HCOD_SF_MAX_LEN: u32 = 19;

/// Table 4.A.1 — `(length_in_bits, codeword)` per index `0..=120`.
///
/// Codewords are right-aligned within the `u32`. To emit one bit-for-
/// bit, write `codeword` as `length` bits MSB-first.
const HCOD_SF: [(u8, u32); HCOD_SF_NUM_ENTRIES] = [
    (18, 0x3ffe8), // 0
    (18, 0x3ffe6), // 1
    (18, 0x3ffe7), // 2
    (18, 0x3ffe5), // 3
    (19, 0x7fff5), // 4
    (19, 0x7fff1), // 5
    (19, 0x7ffed), // 6
    (19, 0x7fff6), // 7
    (19, 0x7ffee), // 8
    (19, 0x7ffef), // 9
    (19, 0x7fff0), // 10
    (19, 0x7fffc), // 11
    (19, 0x7fffd), // 12
    (19, 0x7ffff), // 13
    (19, 0x7fffe), // 14
    (19, 0x7fff7), // 15
    (19, 0x7fff8), // 16
    (19, 0x7fffb), // 17
    (19, 0x7fff9), // 18
    (18, 0x3ffe4), // 19
    (19, 0x7fffa), // 20
    (18, 0x3ffe3), // 21
    (17, 0x1ffef), // 22
    (17, 0x1fff0), // 23
    (16, 0x0fff5), // 24
    (17, 0x1ffee), // 25
    (16, 0x0fff2), // 26
    (16, 0x0fff3), // 27
    (16, 0x0fff4), // 28
    (16, 0x0fff1), // 29
    (15, 0x07ff6), // 30
    (15, 0x07ff7), // 31
    (14, 0x03ff9), // 32
    (14, 0x03ff5), // 33
    (14, 0x03ff7), // 34
    (14, 0x03ff3), // 35
    (14, 0x03ff6), // 36
    (14, 0x03ff2), // 37
    (13, 0x01ff7), // 38
    (13, 0x01ff5), // 39
    (12, 0x00ff9), // 40
    (12, 0x00ff7), // 41
    (12, 0x00ff6), // 42
    (11, 0x007f9), // 43
    (12, 0x00ff4), // 44
    (11, 0x007f8), // 45
    (10, 0x003f9), // 46
    (10, 0x003f7), // 47
    (10, 0x003f5), // 48
    (9, 0x001f8),  // 49
    (9, 0x001f7),  // 50
    (8, 0x000fa),  // 51
    (8, 0x000f8),  // 52
    (8, 0x000f6),  // 53
    (7, 0x00079),  // 54
    (6, 0x0003a),  // 55
    (6, 0x00038),  // 56
    (5, 0x0001a),  // 57
    (4, 0x0000b),  // 58
    (3, 0x00004),  // 59
    (1, 0x00000),  // 60 — delta 0, single bit `0`
    (4, 0x0000a),  // 61
    (4, 0x0000c),  // 62
    (5, 0x0001b),  // 63
    (6, 0x00039),  // 64
    (6, 0x0003b),  // 65
    (7, 0x00078),  // 66
    (7, 0x0007a),  // 67
    (8, 0x000f7),  // 68
    (8, 0x000f9),  // 69
    (9, 0x001f6),  // 70
    (9, 0x001f9),  // 71
    (10, 0x003f4), // 72
    (10, 0x003f6), // 73
    (10, 0x003f8), // 74
    (11, 0x007f5), // 75
    (11, 0x007f4), // 76
    (11, 0x007f6), // 77
    (11, 0x007f7), // 78
    (12, 0x00ff5), // 79
    (12, 0x00ff8), // 80
    (13, 0x01ff4), // 81
    (13, 0x01ff6), // 82
    (13, 0x01ff8), // 83
    (14, 0x03ff8), // 84
    (14, 0x03ff4), // 85
    (16, 0x0fff0), // 86
    (15, 0x07ff4), // 87
    (16, 0x0fff6), // 88
    (15, 0x07ff5), // 89
    (18, 0x3ffe2), // 90
    (19, 0x7ffd9), // 91
    (19, 0x7ffda), // 92
    (19, 0x7ffdb), // 93
    (19, 0x7ffdc), // 94
    (19, 0x7ffdd), // 95
    (19, 0x7ffde), // 96
    (19, 0x7ffd8), // 97
    (19, 0x7ffd2), // 98
    (19, 0x7ffd3), // 99
    (19, 0x7ffd4), // 100
    (19, 0x7ffd5), // 101
    (19, 0x7ffd6), // 102
    (19, 0x7fff2), // 103
    (19, 0x7ffdf), // 104
    (19, 0x7ffe7), // 105
    (19, 0x7ffe8), // 106
    (19, 0x7ffe9), // 107
    (19, 0x7ffea), // 108
    (19, 0x7ffeb), // 109
    (19, 0x7ffe6), // 110
    (19, 0x7ffe0), // 111
    (19, 0x7ffe1), // 112
    (19, 0x7ffe2), // 113
    (19, 0x7ffe3), // 114
    (19, 0x7ffe4), // 115
    (19, 0x7ffe5), // 116
    (19, 0x7ffd7), // 117
    (19, 0x7ffec), // 118
    (19, 0x7fff4), // 119
    (19, 0x7fff3), // 120
];

/// Encode a signed DPCM delta in `-60..=+60` to the wire Huffman
/// codeword for Table 4.A.1.
///
/// Returns `(length_in_bits, codeword)` with `codeword` right-aligned
/// in the `u32` (MSB at bit `length - 1`). Out-of-range `dpcm`
/// produces [`Error::ScaleFactorDataEncodeInvalid`].
///
/// The inverse of [`hcod_sf_decode`].
pub fn hcod_sf_encode(dpcm: i8) -> Result<(u8, u32)> {
    let idx = (dpcm as i32) - (SF_INDEX_OFFSET as i32);
    if !(0..HCOD_SF_NUM_ENTRIES as i32).contains(&idx) {
        return Err(Error::ScaleFactorDataEncodeInvalid);
    }
    Ok(HCOD_SF[idx as usize])
}

/// Decode one Table 4.A.1 Huffman codeword from `reader`, returning
/// the signed DPCM delta in `-60..=+60`.
///
/// The decoder is a straight prefix-match: read one bit at a time,
/// look it up in a flat table. The table is small (121 entries, max
/// length 19 bits) so a single linear scan per bit-extend is
/// sufficient and avoids the cost / complexity of a multi-level
/// lookup acceleration table. Returns [`Error::UnexpectedEnd`] on
/// reader underflow.
///
/// The codebook is a **complete** prefix code (Kraft equality:
/// `Σ 2^(19-L_i) = 2^19`), so every fully-read 19-bit sequence is
/// guaranteed to match some entry — the bottom of the loop is
/// unreachable provided `reader` produces 19 bits without
/// underflowing. A purely-defensive `unreachable!()` guards the
/// loop fall-through; it has been verified at compile-time as
/// dead code by the [`hcod_sf_decode_is_complete`](#) regression
/// test that exhaustively walks all `2^19` 19-bit prefixes.
pub fn hcod_sf_decode(reader: &mut BitReader<'_>) -> Result<i8> {
    let mut acc: u32 = 0;
    for len in 1..=HCOD_SF_MAX_LEN {
        let bit = reader.read_u32(1).map_err(|_| Error::UnexpectedEnd)?;
        acc = (acc << 1) | bit;
        // Linear scan: cost is bounded by HCOD_SF_NUM_ENTRIES * 19.
        for (idx, &(entry_len, entry_cw)) in HCOD_SF.iter().enumerate() {
            if u32::from(entry_len) == len && entry_cw == acc {
                return Ok((idx as i8) + SF_INDEX_OFFSET);
            }
        }
    }
    // Unreachable: the codebook is a complete prefix code over
    // 19 bits (Kraft equality = 524288), so the inner loop must
    // hit for at least one `len <= 19`. The guard is here so the
    // compiler doesn't infer a non-`!` return path.
    unreachable!("HCOD_SF is a complete 19-bit prefix code; the 19-bit walk must match");
}

// =============================================================================
// Per-band record
// =============================================================================

/// One transmitted per-band record.
///
/// The variant is selected by [`crate::section_data::SectionData::sfb_cb`]:
/// `Dpcm` for ordinary spectrum books (1..=11, plus PNS book 13
/// after the first), `Intensity` for books 14 / 15, `NoisePcm` for
/// the **first** PNS band of the frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScaleFactorEntry {
    /// `hcod_sf[dpcm_sf[g][sfb]]` — Huffman DPCM delta for a band
    /// whose codebook is a non-zero spectrum book (1..=11).
    Dpcm(i8),
    /// `hcod_sf[dpcm_is_position[g][sfb]]` — Huffman DPCM delta for
    /// an intensity-stereo band (codebook 14 or 15).
    Intensity(i8),
    /// `dpcm_noise_nrg[g][sfb]` 9-bit PCM seed — emitted **only**
    /// for the first PNS band (codebook 13) of the frame. The value
    /// is the raw 9-bit wire bits (the §4.6.13 reconstruction
    /// converts the unsigned wire pattern to a signed `-256..=+255`
    /// energy delta).
    NoisePcm(u16),
    /// `hcod_sf[dpcm_noise_nrg[g][sfb]]` — Huffman DPCM delta for a
    /// PNS band after the first.
    NoiseDpcm(i8),
}

/// Parsed `scale_factor_data()` payload (non-resilient branch).
///
/// `entries` is grouped per window group: `entries[g][i]` is the
/// `i`-th transmitted per-band record for group `g`, in wire
/// (low-frequency-first) order. The mapping back to scalefactor
/// bands is recovered by walking
/// [`SectionData::sfb_cb`](crate::section_data::SectionData::sfb_cb)
/// and skipping `ZERO_HCB` bands — the same walk the parser
/// performed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScaleFactorData {
    /// `entries[g]` — the per-band records of window group `g` in
    /// wire order. `entries.len()` equals `sfb_cb.len()`
    /// (`num_window_groups`).
    pub entries: Vec<Vec<ScaleFactorEntry>>,
}

impl ScaleFactorData {
    /// Parse a non-resilient `scale_factor_data()` from `reader`.
    ///
    /// * `reader` — positioned at the first bit of the
    ///   `scale_factor_data()` block (immediately after
    ///   `section_data()`).
    /// * `sfb_cb` — the per-`(g, sfb)` codebook map produced by
    ///   [`section_data::SectionData::parse`](crate::section_data::SectionData::parse).
    ///   Outer length is `num_window_groups`; each inner slice is
    ///   `max_sfb` entries.
    ///
    /// Returns [`Error::UnexpectedEnd`] on reader underflow. The
    /// codebook is a complete 19-bit prefix code so a fully-read
    /// Huffman value is guaranteed to match an entry.
    pub fn parse(reader: &mut BitReader<'_>, sfb_cb: &[Vec<u8>]) -> Result<Self> {
        let mut noise_pcm_flag = true;
        let mut entries: Vec<Vec<ScaleFactorEntry>> = Vec::with_capacity(sfb_cb.len());
        for group in sfb_cb {
            let mut group_entries: Vec<ScaleFactorEntry> = Vec::new();
            for &cb in group {
                if cb == ZERO_HCB {
                    continue;
                }
                let entry = if is_intensity(cb) {
                    let dpcm = hcod_sf_decode(reader)?;
                    ScaleFactorEntry::Intensity(dpcm)
                } else if is_noise(cb) {
                    if noise_pcm_flag {
                        noise_pcm_flag = false;
                        let pcm = reader
                            .read_u32(NOISE_PCM_BITS)
                            .map_err(|_| Error::UnexpectedEnd)?
                            as u16;
                        ScaleFactorEntry::NoisePcm(pcm)
                    } else {
                        let dpcm = hcod_sf_decode(reader)?;
                        ScaleFactorEntry::NoiseDpcm(dpcm)
                    }
                } else {
                    let dpcm = hcod_sf_decode(reader)?;
                    ScaleFactorEntry::Dpcm(dpcm)
                };
                group_entries.push(entry);
            }
            entries.push(group_entries);
        }
        Ok(ScaleFactorData { entries })
    }

    /// Encode `scale_factor_data()` onto `writer`, the inverse of
    /// [`ScaleFactorData::parse`].
    ///
    /// * `writer` — receives the bit-exact Table 4.53 stream.
    /// * `sfb_cb` — the same codebook map the matching parse call
    ///   would receive. Drives the variant the writer expects at
    ///   each band.
    ///
    /// Returns [`Error::ScaleFactorDataEncodeInvalid`] if:
    ///
    /// * `self.entries.len()` does not equal `sfb_cb.len()`.
    /// * A group's `entries` count does not match the number of
    ///   non-zero-codebook bands in the matching `sfb_cb` group.
    /// * The variant at index `i` does not match the codebook
    ///   classification of the `i`-th non-zero band
    ///   (e.g. [`ScaleFactorEntry::Intensity`] paired with a
    ///   spectrum book, or [`ScaleFactorEntry::NoisePcm`] paired
    ///   with a non-PNS band, or — for the second PNS band onward —
    ///   [`ScaleFactorEntry::NoisePcm`] re-used after
    ///   `noise_pcm_flag` has cleared).
    /// * A `Dpcm` / `Intensity` / `NoiseDpcm` delta falls outside
    ///   `-60..=+60`.
    /// * A `NoisePcm` value exceeds the 9-bit field cap
    ///   (`> 0x1ff`).
    pub fn write(&self, writer: &mut BitWriter, sfb_cb: &[Vec<u8>]) -> Result<()> {
        if self.entries.len() != sfb_cb.len() {
            return Err(Error::ScaleFactorDataEncodeInvalid);
        }
        let mut noise_pcm_flag = true;
        for (group_entries, group_cb) in self.entries.iter().zip(sfb_cb.iter()) {
            // Walk both in lockstep: the entries list and the
            // non-zero subsequence of sfb_cb must match position-by-
            // position. Surfacing a mismatch is the same error
            // regardless of cause (length vs variant mismatch).
            let mut entry_iter = group_entries.iter();
            for &cb in group_cb {
                if cb == ZERO_HCB {
                    continue;
                }
                let entry = entry_iter
                    .next()
                    .ok_or(Error::ScaleFactorDataEncodeInvalid)?;
                match (entry, cb) {
                    (ScaleFactorEntry::Intensity(dpcm), cb) if is_intensity(cb) => {
                        let (len, cw) = hcod_sf_encode(*dpcm)?;
                        writer.write_u32(cw, u32::from(len));
                    }
                    (ScaleFactorEntry::NoisePcm(pcm), cb) if is_noise(cb) => {
                        if !noise_pcm_flag {
                            // PNS seed already consumed earlier;
                            // a second NoisePcm is wire-illegal.
                            return Err(Error::ScaleFactorDataEncodeInvalid);
                        }
                        if u32::from(*pcm) >= (1u32 << NOISE_PCM_BITS) {
                            return Err(Error::ScaleFactorDataEncodeInvalid);
                        }
                        noise_pcm_flag = false;
                        writer.write_u32(u32::from(*pcm), NOISE_PCM_BITS);
                    }
                    (ScaleFactorEntry::NoiseDpcm(dpcm), cb) if is_noise(cb) => {
                        if noise_pcm_flag {
                            // First PNS band of the frame must use
                            // the 9-bit PCM seed, not the Huffman
                            // delta — caller skipped the seed.
                            return Err(Error::ScaleFactorDataEncodeInvalid);
                        }
                        let (len, cw) = hcod_sf_encode(*dpcm)?;
                        writer.write_u32(cw, u32::from(len));
                    }
                    (ScaleFactorEntry::Dpcm(dpcm), cb) if !is_intensity(cb) && !is_noise(cb) => {
                        let (len, cw) = hcod_sf_encode(*dpcm)?;
                        writer.write_u32(cw, u32::from(len));
                    }
                    _ => return Err(Error::ScaleFactorDataEncodeInvalid),
                }
            }
            // Extra entries beyond the non-zero codebook subsequence
            // would silently shift the wire layout — reject.
            if entry_iter.next().is_some() {
                return Err(Error::ScaleFactorDataEncodeInvalid);
            }
        }
        Ok(())
    }
}

/// Internal: `cb` is an intensity codebook (14 or 15).
fn is_intensity(cb: u8) -> bool {
    cb == INTENSITY_HCB || cb == INTENSITY_HCB2
}

/// Internal: `cb` is the PNS codebook (13).
fn is_noise(cb: u8) -> bool {
    cb == NOISE_HCB
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Spot-check a handful of Table 4.A.1 rows the way the spec
    /// presents them: `index 60 → 1 bit, codeword 0`; `index 59 →
    /// 3 bits, codeword 4`; `index 61 → 4 bits, codeword 0xa`.
    #[test]
    fn hcod_sf_table_known_rows() {
        assert_eq!(HCOD_SF[60], (1, 0x0));
        assert_eq!(HCOD_SF[59], (3, 0x4));
        assert_eq!(HCOD_SF[61], (4, 0xa));
        assert_eq!(HCOD_SF[0], (18, 0x3ffe8));
        assert_eq!(HCOD_SF[120], (19, 0x7fff3));
    }

    /// The codebook is prefix-free (no codeword is a prefix of any
    /// other). Verified once here as a regression guard against typos
    /// in the Table 4.A.1 transcription above.
    #[test]
    fn hcod_sf_table_is_prefix_free() {
        for (i, &(li, vi)) in HCOD_SF.iter().enumerate() {
            for (j, &(lj, vj)) in HCOD_SF.iter().enumerate() {
                if i == j || lj < li {
                    continue;
                }
                let lo = u32::from(lj - li);
                let prefix = vj >> lo;
                assert_ne!(
                    prefix, vi,
                    "entry {} (L={}, v={:x}) is prefix of entry {} (L={}, v={:x})",
                    i, li, vi, j, lj, vj
                );
            }
        }
    }

    /// `index_offset = -60`: encoding `dpcm = 0` selects index 60,
    /// the single-bit `0` codeword.
    #[test]
    fn encode_dpcm_zero_is_single_bit() {
        let (len, cw) = hcod_sf_encode(0).unwrap();
        assert_eq!(len, 1);
        assert_eq!(cw, 0);
    }

    /// Boundary values: `-60` and `+60` are the endpoints of the
    /// DPCM range; anything outside is rejected.
    #[test]
    fn encode_dpcm_boundaries() {
        assert!(hcod_sf_encode(-60).is_ok());
        assert!(hcod_sf_encode(60).is_ok());
        assert_eq!(
            hcod_sf_encode(-61),
            Err(Error::ScaleFactorDataEncodeInvalid)
        );
        assert_eq!(hcod_sf_encode(61), Err(Error::ScaleFactorDataEncodeInvalid));
    }

    /// Every entry of the table round-trips: encode then decode
    /// recovers the original DPCM value.
    #[test]
    fn hcod_sf_roundtrip_every_entry() {
        for dpcm in -60i8..=60 {
            let (len, cw) = hcod_sf_encode(dpcm).unwrap();
            let mut bw = BitWriter::new();
            bw.write_u32(cw, u32::from(len));
            let bits_written = bw.bit_position();
            let buf = bw.finish();
            let mut br = BitReader::new(&buf);
            let recovered = hcod_sf_decode(&mut br).unwrap();
            assert_eq!(recovered, dpcm);
            // Reader must consume exactly `len` bits.
            assert_eq!(br.bit_position(), bits_written);
        }
    }
}

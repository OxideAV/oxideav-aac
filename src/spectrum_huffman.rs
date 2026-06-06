//! Spectrum Huffman codebook **wire** layer — ISO/IEC 14496-3
//! §4.6.3 + Annex 4.A (Tables 4.A.2 … 4.A.12).
//!
//! Round 213 landed [`crate::spectral_codebook`] — the §4.6.3.3 index ↔
//! n-tuple translation, the §4.6.3 sign-bit fix-up, and the codebook-11
//! ESC sequence. That module does **not** carry the Huffman codeword
//! tables themselves: it operates on the *index* the wire bitstream
//! decodes to, leaving the codeword ↔ index mapping for this module
//! to own.
//!
//! Round 219 landed the first of the eleven spectrum Huffman
//! codebooks — **Table 4.A.2, "Spectrum Huffman Codebook 1"**. Round
//! 226 added the second — **Table 4.A.3, "Spectrum Huffman Codebook
//! 2"**. Round 231 added the third — **Table 4.A.4, "Spectrum Huffman
//! Codebook 3"**. Round 234 added the fourth — **Table 4.A.5, "Spectrum
//! Huffman Codebook 4"**. Round 238 added the fifth — **Table 4.A.6,
//! "Spectrum Huffman Codebook 5"** — the first **pair** (`dim = 2`)
//! book and the first book to widen its codewords to 13 bits. Round
//! 241 adds the sixth — **Table 4.A.7, "Spectrum Huffman Codebook
//! 6"** — the second pair book, sharing the Codebook 5 Table 4.95
//! row shape (`signed`, `dim = 2`, `LAV = 4`) but tightening the
//! codeword ceiling back down to 11 bits.
//! Codebooks 1 and 2 share the same Table 4.95
//! row shape (`signed`, `dim = 4`, `LAV = 1` → `3^4 = 81` entries
//! indexed `0..=80`); Codebooks 3 and 4 share the unsigned dim-4
//! shape (Table 4.95 rows 3 and 4 both: `unsigned_cb = 1`, `dim = 4`,
//! `LAV = 2` → `3^4 = 81` entries indexed `0..=80`, with sign bits
//! following the Huffman codeword for every non-zero coefficient per
//! §4.6.3.3); Codebooks 5 and 6 share the signed pair shape
//! (Table 4.95 rows 5 and 6 both: `unsigned_cb = 0`, `dim = 2`,
//! `LAV = 4` → `(2 * 4 + 1)^2 = 9^2 = 81` entries indexed `0..=80`,
//! each tuple coefficient in `-4..=+4`, signed-book so no sign-bit
//! suffix is required after the codeword). Codebooks 7..=11
//! (Tables 4.A.8 … 4.A.12) reuse the same module shape and will be
//! added one per future round; the traits and dispatch surface here
//! are intentionally codebook-agnostic so the per-codebook additions
//! land as a single static-table transcription each.
//!
//! ## Codebook 1 invariants (Table 4.A.2)
//!
//! | property               | value     | source                       |
//! |------------------------|-----------|------------------------------|
//! | dimension              | 4         | Table 4.95 row 1, column 3   |
//! | `unsigned_cb`          | 0 (signed)| Table 4.95 row 1, column 2   |
//! | LAV                    | 1         | Table 4.95 row 1, column 4   |
//! | entry count            | `3^4 = 81`| `(2 * 1 + 1)^4` per §4.6.3.3 |
//! | maximum codeword length| 11 bits   | Table 4.A.2 column 2 maximum |
//! | shortest codeword      | 1 bit     | Table 4.A.2 row 40 (index 40)|
//! | shortest codeword value| `0`       | Table 4.A.2 row 40           |
//! | Kraft equality         | 2048 = 2¹¹| see [`hcod1_is_complete`]    |
//!
//! Index 40 is `(w, x, y, z) = (0, 0, 0, 0)` per §4.6.3.3 — the
//! zero-tuple gets the single-bit codeword because zero-tuples are
//! the modal spectrum n-tuple in any non-silent frame.
//!
//! ## Wire representation in memory
//!
//! Codewords are stored right-aligned within a `u16`: the MSB of the
//! wire codeword sits at bit `length − 1`, the LSB at bit `0`. To emit
//! bit-for-bit, [`hcod1_encode`] returns `(length, codeword)` and the
//! caller passes them straight to
//! [`oxideav_core::bits::BitWriter::write_u32`].
//!
//! ## Codebook 2 invariants (Table 4.A.3)
//!
//! | property               | value     | source                       |
//! |------------------------|-----------|------------------------------|
//! | dimension              | 4         | Table 4.95 row 2, column 3   |
//! | `unsigned_cb`          | 0 (signed)| Table 4.95 row 2, column 2   |
//! | LAV                    | 1         | Table 4.95 row 2, column 4   |
//! | entry count            | `3^4 = 81`| `(2 * 1 + 1)^4` per §4.6.3.3 |
//! | maximum codeword length| 9 bits    | Table 4.A.3 column 2 maximum |
//! | shortest codeword      | 3 bits    | Table 4.A.3 row 40 (index 40)|
//! | shortest codeword value| `0`       | Table 4.A.3 row 40           |
//! | Kraft equality         | 512 = 2⁹  | see [`hcod2_is_complete`]    |
//!
//! Codebook 2 covers the same `3^4 = 81` signed 4-tuple universe as
//! Codebook 1, with each coefficient in `(-1, 0, +1)`. The encoder
//! chooses between the two books per-section based on
//! `section_data()`'s `sect_cb` field; the choice reflects which book
//! gives the shorter overall bit count for the section's tuple
//! statistics. Index 40 is `(w, x, y, z) = (0, 0, 0, 0)` in both
//! books; in Codebook 2 it carries the 3-bit codeword `0b000`
//! (vs the single bit `0` in Codebook 1).
//!
//! ## Codebook 3 invariants (Table 4.A.4)
//!
//! | property               | value      | source                       |
//! |------------------------|------------|------------------------------|
//! | dimension              | 4          | Table 4.95 row 3, column 3   |
//! | `unsigned_cb`          | 1 (unsigned)| Table 4.95 row 3, column 2  |
//! | LAV                    | 2          | Table 4.95 row 3, column 4   |
//! | entry count            | `3^4 = 81` | `(2 + 1)^4` per §4.6.3.3     |
//! | maximum codeword length| 16 bits    | Table 4.A.4 column 2 maximum |
//! | shortest codeword      | 1 bit      | Table 4.A.4 row 0 (index 0)  |
//! | shortest codeword value| `0`        | Table 4.A.4 row 0            |
//! | Kraft equality         | 65536 = 2¹⁶| see [`hcod3_is_complete`]    |
//!
//! Codebook 3 is the first *unsigned* spectrum book: the Huffman
//! codeword conveys the magnitude n-tuple (each coefficient in
//! `0..=LAV = 0..=2`) and each non-zero coefficient is followed by a
//! single sign bit per §4.6.3.3 (the sign bits travel in
//! low-frequency-first order: `w`, `x`, `y`, `z`). The zero-tuple
//! `(0, 0, 0, 0)` is at *index 0* (not 40 as in the signed books)
//! because the unsigned modulus-3 polynomial puts all-zero at the
//! origin; it carries the single bit codeword `0`. The §4.6.3.3
//! sign-bit suffix is exposed by
//! [`apply_sign_bits`](crate::spectral_codebook::apply_sign_bits) /
//! [`derive_sign_bits`](crate::spectral_codebook::derive_sign_bits)
//! and is *not* part of the Huffman codeword itself — this module's
//! `hcod3_encode` / `hcod3_decode` cover the codeword only.
//!
//! ## Codebook 4 invariants (Table 4.A.5)
//!
//! | property               | value      | source                       |
//! |------------------------|------------|------------------------------|
//! | dimension              | 4          | Table 4.95 row 4, column 3   |
//! | `unsigned_cb`          | 1 (unsigned)| Table 4.95 row 4, column 2  |
//! | LAV                    | 2          | Table 4.95 row 4, column 4   |
//! | entry count            | `3^4 = 81` | `(2 + 1)^4` per §4.6.3.3     |
//! | maximum codeword length| 12 bits    | Table 4.A.5 column 2 maximum |
//! | shortest codeword      | 4 bits     | Table 4.A.5 row 40 (index 40)|
//! | shortest codeword value| `0`        | Table 4.A.5 row 40           |
//! | Kraft equality         | 4096 = 2¹² | see [`hcod4_is_complete`]    |
//!
//! Codebook 4 shares Codebook 3's unsigned dim-4 LAV-2 tuple universe
//! (Table 4.95 row 4 is identical to row 3 except for the `Codebook
//! listed in Table` column) but uses a different per-row Huffman
//! length tuning for a different encoder target-statistics. Where
//! Codebook 3 puts the zero-tuple at index 0 with a single-bit
//! codeword and lets the magnitude-2 tuples climb to a 16-bit
//! maximum, Codebook 4 puts the zero-tuple at the *same* §4.6.3.3
//! polynomial position (index 0 maps the unsigned `(0, 0, 0, 0)`
//! tuple via the `((w*3 + x)*3 + y)*3 + z` evaluation with no offset)
//! — but the codeword assignment lifts the zero-tuple to a 4-bit
//! codeword (`0b0111`) and parks the *shortest* codeword (4 bits
//! `0b0000`) at **index 40** instead. The maximum codeword length is
//! **12 bits** (vs 16 for Codebook 3), and two distinct rows reach
//! that length: index 62 (`0xfff`) and index 74 (`0xffe`). The
//! shorter overall code length distribution makes Codebook 4 a
//! better fit for sections whose magnitude statistics are flatter
//! across the `(0, 0, 0, 0) .. (2, 2, 2, 2)` range than Codebook 3's
//! zero-heavy target. The §4.6.3.3 sign-bit suffix is again exposed
//! by [`apply_sign_bits`](crate::spectral_codebook::apply_sign_bits)
//! / [`derive_sign_bits`](crate::spectral_codebook::derive_sign_bits)
//! and is *not* part of the Huffman codeword itself — this module's
//! `hcod4_encode` / `hcod4_decode` cover the codeword only.
//!
//! ## Codebook 5 invariants (Table 4.A.6)
//!
//! | property               | value      | source                       |
//! |------------------------|------------|------------------------------|
//! | dimension              | 2 (pair)   | Table 4.95 row 5, column 3   |
//! | `unsigned_cb`          | 0 (signed) | Table 4.95 row 5, column 2   |
//! | LAV                    | 4          | Table 4.95 row 5, column 4   |
//! | entry count            | `9^2 = 81` | `(2 * 4 + 1)^2` per §4.6.3.3 |
//! | maximum codeword length| 13 bits    | Table 4.A.6 column 2 maximum |
//! | shortest codeword      | 1 bit      | Table 4.A.6 row 40 (index 40)|
//! | shortest codeword value| `0`        | Table 4.A.6 row 40           |
//! | Kraft equality         | 8192 = 2¹³ | see [`hcod5_is_complete`]    |
//!
//! Codebook 5 is the first **pair** book — the §4.6.3.3 translation
//! consumes two coefficients per Huffman codeword (`(y, z)`) rather
//! than four (`(w, x, y, z)`) — and the first book to widen the
//! per-coefficient quantised range to `-4..=+4` (LAV = 4). The pair
//! universe stays at 81 entries because `(2 * 4 + 1)^2 = 9^2 = 81`
//! coincides with the dim-4 LAV-1 / LAV-2 universes of Codebooks
//! 1..=4. Index 40 carries the §4.6.3.3 zero-tuple `(0, 0)` — the
//! `(modulus = 9, offset = 4)` polynomial evaluation puts the
//! origin at the centre of the index range, not at the edges as in
//! the unsigned books (Codebooks 3 and 4 placed `(0, 0, 0, 0)` at
//! index 0). The shortest codeword (1 bit `0`) parks at index 40
//! — the same zero-tuple position as Codebook 1 (whose dim-4 origin
//! also lands at the row-40 centre via the same signed-book
//! polynomial). The maximum codeword length is **13 bits** — one
//! more than Codebook 4's 12-bit ceiling and three less than
//! Codebook 3's 16-bit reach — and exactly four rows occupy the
//! 13-bit ceiling: indices 0, 8, 72, and 80 (the four corners
//! `(-4, -4)`, `(-4, +4)`, `(+4, -4)`, `(+4, +4)` of the
//! `9 × 9` signed pair lattice). Because Codebook 5 is **signed**,
//! the §4.6.3.3 sign-bit suffix is *not* emitted after the
//! codeword — every coefficient's sign is baked into the index
//! itself via the `offset = LAV = 4` shift.
//!
//! ## Codebook 6 invariants (Table 4.A.7)
//!
//! | property               | value      | source                       |
//! |------------------------|------------|------------------------------|
//! | dimension              | 2 (pair)   | Table 4.95 row 6, column 3   |
//! | `unsigned_cb`          | 0 (signed) | Table 4.95 row 6, column 2   |
//! | LAV                    | 4          | Table 4.95 row 6, column 4   |
//! | entry count            | `9^2 = 81` | `(2 * 4 + 1)^2` per §4.6.3.3 |
//! | maximum codeword length| 11 bits    | Table 4.A.7 column 2 maximum |
//! | shortest codeword      | 4 bits     | Table 4.A.7 row 40 (index 40)|
//! | shortest codeword value| `0`        | Table 4.A.7 row 40           |
//! | Kraft equality         | 2048 = 2¹¹| see [`hcod6_is_complete`]    |
//!
//! Codebook 6 shares Codebook 5's signed pair tuple universe
//! (Table 4.95 row 6 is identical to row 5 except for the `Codebook
//! listed in Table` column) but uses a different per-row Huffman
//! length tuning. Where Codebook 5 parks the single bit `0` at
//! index 40 and lets the four lattice corners reach a 13-bit
//! ceiling, Codebook 6 lifts the zero-tuple at index 40 to a 4-bit
//! `0b0000` and pulls the ceiling back to **11 bits**. Exactly four
//! rows reach the 11-bit ceiling: indices 0 (`0x7fe`), 8 (`0x7fd`),
//! 72 (`0x7ff`), and 80 (`0x7fc`) — the four `(±4, ±4)` corners of
//! the `9 × 9` signed pair lattice, the same four corner positions
//! Codebook 5 also pinned to its 13-bit ceiling. The shorter,
//! flatter codeword distribution makes Codebook 6 a better fit
//! for sections whose magnitude statistics put more weight in the
//! `(±1, ±1) .. (±3, ±3)` interior than Codebook 5's
//! more-zero-tuple-heavy target. The encoder chooses between the
//! two books per-section via `section_data()`'s `sect_cb` field;
//! the §4.6.3.3 sign bits remain inside the index for both books
//! because both are signed (`unsigned_cb = 0`).
//!
//! * The §4.6.3.3 index → n-tuple translation. That sits in
//!   [`crate::spectral_codebook::decode_index_to_tuple`] /
//!   [`crate::spectral_codebook::encode_tuple_to_index`].
//! * The ESC sequence (codebook 11 and the extension books 16..=31).
//!   That sits in [`crate::spectral_codebook::decode_esc_value`] /
//!   [`crate::spectral_codebook::encode_esc_value`].
//! * The §4.6.3 sign-bit suffix for unsigned codebooks. Codebook 1
//!   is *signed* so no sign bits follow the codeword; the
//!   [`apply_sign_bits`](crate::spectral_codebook::apply_sign_bits)
//!   path is exercised by unsigned codebooks (3, 4, 7..=11, 16..=31).
//! * The `spectral_data()` driver that loops over scalefactor bands
//!   and dispatches per-band onto the codebook chosen by
//!   `section_data()`. That driver will land once codebooks 2..=11
//!   are in place.

use oxideav_core::bits::{BitReader, BitWriter};

use crate::{Error, Result};

// =============================================================================
// Table 4.A.2 — Spectrum Huffman Codebook 1
// =============================================================================
//
// 81 entries, indices 0..=80. Each row is `(length_in_bits,
// codeword)` with `codeword` right-aligned in a `u16` (MSB of the wire
// codeword at bit `length − 1`). Reproduced verbatim from ISO/IEC
// 14496-3:2001(E) §4.A.1 Table 4.A.2 (page 193).
//
// The codebook is a complete prefix code: Σᵢ 2^(11 − Lᵢ) = 2048 = 2¹¹.
// This is exhaustively verified at compile time by the
// `hcod1_is_complete` regression test (which walks every 11-bit
// prefix and asserts each maps to exactly one index).

/// Number of entries in Table 4.A.2 (`81`, indices `0..=80`).
pub const HCOD1_NUM_ENTRIES: usize = 81;

/// Maximum codeword length emitted by Table 4.A.2 (11 bits).
pub const HCOD1_MAX_LEN: u32 = 11;

/// Table 4.A.2 — `(length_in_bits, codeword)` per index `0..=80`.
///
/// Codewords are right-aligned within the `u16`. To emit one
/// bit-for-bit, write `codeword` as `length` bits MSB-first.
const HCOD1: [(u8, u16); HCOD1_NUM_ENTRIES] = [
    (11, 0x7f8), // 0
    (9, 0x1f1),  // 1
    (11, 0x7fd), // 2
    (10, 0x3f5), // 3
    (7, 0x68),   // 4
    (10, 0x3f0), // 5
    (11, 0x7f7), // 6
    (9, 0x1ec),  // 7
    (11, 0x7f5), // 8
    (10, 0x3f1), // 9
    (7, 0x72),   // 10
    (10, 0x3f4), // 11
    (7, 0x74),   // 12
    (5, 0x11),   // 13
    (7, 0x76),   // 14
    (9, 0x1eb),  // 15
    (7, 0x6c),   // 16
    (10, 0x3f6), // 17
    (11, 0x7fc), // 18
    (9, 0x1e1),  // 19
    (11, 0x7f1), // 20
    (9, 0x1f0),  // 21
    (7, 0x61),   // 22
    (9, 0x1f6),  // 23
    (11, 0x7f2), // 24
    (9, 0x1ea),  // 25
    (11, 0x7fb), // 26
    (9, 0x1f2),  // 27
    (7, 0x69),   // 28
    (9, 0x1ed),  // 29
    (7, 0x77),   // 30
    (5, 0x17),   // 31
    (7, 0x6f),   // 32
    (9, 0x1e6),  // 33
    (7, 0x64),   // 34
    (9, 0x1e5),  // 35
    (7, 0x67),   // 36
    (5, 0x15),   // 37
    (7, 0x62),   // 38
    (5, 0x12),   // 39
    (1, 0x000),  // 40 — zero-tuple, single bit `0`
    (5, 0x14),   // 41
    (7, 0x65),   // 42
    (5, 0x16),   // 43
    (7, 0x6d),   // 44
    (9, 0x1e9),  // 45
    (7, 0x63),   // 46
    (9, 0x1e4),  // 47
    (7, 0x6b),   // 48
    (5, 0x13),   // 49
    (7, 0x71),   // 50
    (9, 0x1e3),  // 51
    (7, 0x70),   // 52
    (9, 0x1f3),  // 53
    (11, 0x7fe), // 54
    (9, 0x1e7),  // 55
    (11, 0x7f3), // 56
    (9, 0x1ef),  // 57
    (7, 0x60),   // 58
    (9, 0x1ee),  // 59
    (11, 0x7f0), // 60
    (9, 0x1e2),  // 61
    (11, 0x7fa), // 62
    (10, 0x3f3), // 63
    (7, 0x6a),   // 64
    (9, 0x1e8),  // 65
    (7, 0x75),   // 66
    (5, 0x10),   // 67
    (7, 0x73),   // 68
    (9, 0x1f4),  // 69
    (7, 0x6e),   // 70
    (10, 0x3f7), // 71
    (11, 0x7f6), // 72
    (9, 0x1e0),  // 73
    (11, 0x7f9), // 74
    (10, 0x3f2), // 75
    (7, 0x66),   // 76
    (9, 0x1f5),  // 77
    (11, 0x7ff), // 78
    (9, 0x1f7),  // 79
    (11, 0x7f4), // 80
];

/// Encode a Codebook 1 codeword index (`0..=80`) to the wire Huffman
/// codeword from Table 4.A.2.
///
/// Returns `(length_in_bits, codeword)` with `codeword` right-aligned
/// in the `u16` (MSB at bit `length − 1`). Out-of-range `idx`
/// produces [`Error::SpectralCodebookIndexOutOfRange`]; the legal
/// range is `0..=80` (the 81-entry `3^4` enumeration of every legal
/// signed 4-tuple with each coefficient in `-1..=+1`).
///
/// The inverse of [`hcod1_decode`].
pub fn hcod1_encode(idx: u32) -> Result<(u8, u16)> {
    let entry = HCOD1
        .get(idx as usize)
        .ok_or(Error::SpectralCodebookIndexOutOfRange(1))?;
    Ok(*entry)
}

/// Decode one Codebook 1 Huffman codeword from `reader`, returning
/// the codeword index in `0..=80`.
///
/// The decoder is a straight prefix-match: read one bit at a time
/// (MSB-first), look it up in a flat 81-entry table. The table is
/// small (max codeword length 11 bits, 81 entries) so a single
/// linear scan per bit-extend is cheaper than the storage and
/// build-time cost of a multi-level lookup acceleration table.
/// Returns [`Error::UnexpectedEnd`] on reader underflow.
///
/// The codebook is a **complete** prefix code over 11 bits (Kraft
/// equality `Σᵢ 2^(11 − Lᵢ) = 2048 = 2¹¹`), so any 11-bit prefix
/// fully read from `reader` is guaranteed to match exactly one
/// entry — the bottom of the loop is unreachable when `reader`
/// produces 11 bits without underflowing. A purely defensive
/// `unreachable!()` guards the loop fall-through; it is verified
/// dead by the `hcod1_is_complete` regression test that exhaustively
/// walks all `2¹¹` 11-bit prefixes.
pub fn hcod1_decode(reader: &mut BitReader<'_>) -> Result<u32> {
    let mut acc: u32 = 0;
    for len in 1..=HCOD1_MAX_LEN {
        let bit = reader.read_u32(1).map_err(|_| Error::UnexpectedEnd)?;
        acc = (acc << 1) | bit;
        for (idx, &(entry_len, entry_cw)) in HCOD1.iter().enumerate() {
            if u32::from(entry_len) == len && u32::from(entry_cw) == acc {
                return Ok(idx as u32);
            }
        }
    }
    // Unreachable: HCOD1 is a complete 11-bit prefix code. The
    // `hcod1_is_complete` regression test verifies every 11-bit
    // prefix maps to exactly one entry.
    unreachable!("HCOD1 is a complete 11-bit prefix code; the 11-bit walk must match");
}

/// Write a Codebook 1 codeword to `writer` by index.
///
/// Convenience over `hcod1_encode` + manual `write_u32`. Returns
/// [`Error::SpectralCodebookIndexOutOfRange`] for `idx > 80`.
pub fn hcod1_write(writer: &mut BitWriter, idx: u32) -> Result<()> {
    let (len, cw) = hcod1_encode(idx)?;
    writer.write_u32(u32::from(cw), u32::from(len));
    Ok(())
}

// =============================================================================
// Table 4.A.3 — Spectrum Huffman Codebook 2
// =============================================================================
//
// 81 entries, indices 0..=80. Each row is `(length_in_bits,
// codeword)` with `codeword` right-aligned in a `u16` (MSB of the wire
// codeword at bit `length − 1`). Transcribed verbatim from ISO/IEC
// 14496-3:2001(E) §4.A.1 Table 4.A.3 (page 194).
//
// The codebook is a complete prefix code: Σᵢ 2^(9 − Lᵢ) = 512 = 2⁹.
// This is exhaustively verified by the `hcod2_is_complete` regression
// test (which walks every 9-bit prefix and asserts each maps to
// exactly one index).
//
// The signed-tuple universe is identical to Codebook 1's (3^4 = 81
// signed 4-tuples with each element in `-1..=+1`); the §4.6.3.3 index
// translation in [`crate::spectral_codebook`] is reused as-is.

/// Number of entries in Table 4.A.3 (`81`, indices `0..=80`).
pub const HCOD2_NUM_ENTRIES: usize = 81;

/// Maximum codeword length emitted by Table 4.A.3 (9 bits).
pub const HCOD2_MAX_LEN: u32 = 9;

/// Table 4.A.3 — `(length_in_bits, codeword)` per index `0..=80`.
///
/// Codewords are right-aligned within the `u16`. To emit one
/// bit-for-bit, write `codeword` as `length` bits MSB-first.
const HCOD2: [(u8, u16); HCOD2_NUM_ENTRIES] = [
    (9, 0x1f3), // 0
    (7, 0x6f),  // 1
    (9, 0x1fd), // 2
    (8, 0xeb),  // 3
    (6, 0x23),  // 4
    (8, 0xea),  // 5
    (9, 0x1f7), // 6
    (8, 0xe8),  // 7
    (9, 0x1fa), // 8
    (8, 0xf2),  // 9
    (6, 0x2d),  // 10
    (7, 0x70),  // 11
    (6, 0x20),  // 12
    (5, 0x06),  // 13
    (6, 0x2b),  // 14
    (7, 0x6e),  // 15
    (6, 0x28),  // 16
    (8, 0xe9),  // 17
    (9, 0x1f9), // 18
    (7, 0x66),  // 19
    (8, 0xf8),  // 20
    (8, 0xe7),  // 21
    (6, 0x1b),  // 22
    (8, 0xf1),  // 23
    (9, 0x1f4), // 24
    (7, 0x6b),  // 25
    (9, 0x1f5), // 26
    (8, 0xec),  // 27
    (6, 0x2a),  // 28
    (7, 0x6c),  // 29
    (6, 0x2c),  // 30
    (5, 0x0a),  // 31
    (6, 0x27),  // 32
    (7, 0x67),  // 33
    (6, 0x1a),  // 34
    (8, 0xf5),  // 35
    (6, 0x24),  // 36
    (5, 0x08),  // 37
    (6, 0x1f),  // 38
    (5, 0x09),  // 39
    (3, 0x000), // 40 — zero-tuple, 3-bit codeword `0`
    (5, 0x07),  // 41
    (6, 0x1d),  // 42
    (5, 0x0b),  // 43
    (6, 0x30),  // 44
    (8, 0xef),  // 45
    (6, 0x1c),  // 46
    (7, 0x64),  // 47
    (6, 0x1e),  // 48
    (5, 0x0c),  // 49
    (6, 0x29),  // 50
    (8, 0xf3),  // 51
    (6, 0x2f),  // 52
    (8, 0xf0),  // 53
    (9, 0x1fc), // 54
    (7, 0x71),  // 55
    (9, 0x1f2), // 56
    (8, 0xf4),  // 57
    (6, 0x21),  // 58
    (8, 0xe6),  // 59
    (8, 0xf7),  // 60
    (7, 0x68),  // 61
    (9, 0x1f8), // 62
    (8, 0xee),  // 63
    (6, 0x22),  // 64
    (7, 0x65),  // 65
    (6, 0x31),  // 66
    (4, 0x02),  // 67
    (6, 0x26),  // 68
    (8, 0xed),  // 69
    (6, 0x25),  // 70
    (7, 0x6a),  // 71
    (9, 0x1fb), // 72
    (7, 0x72),  // 73
    (9, 0x1fe), // 74
    (7, 0x69),  // 75
    (6, 0x2e),  // 76
    (8, 0xf6),  // 77
    (9, 0x1ff), // 78
    (7, 0x6d),  // 79
    (9, 0x1f6), // 80
];

/// Encode a Codebook 2 codeword index (`0..=80`) to the wire Huffman
/// codeword from Table 4.A.3.
///
/// Returns `(length_in_bits, codeword)` with `codeword` right-aligned
/// in the `u16` (MSB at bit `length − 1`). Out-of-range `idx`
/// produces [`Error::SpectralCodebookIndexOutOfRange`] carrying the
/// codebook number `2`; the legal range is `0..=80` (the 81-entry
/// `3^4` enumeration of every legal signed 4-tuple with each
/// coefficient in `-1..=+1` — the same universe as Codebook 1).
///
/// The inverse of [`hcod2_decode`].
pub fn hcod2_encode(idx: u32) -> Result<(u8, u16)> {
    let entry = HCOD2
        .get(idx as usize)
        .ok_or(Error::SpectralCodebookIndexOutOfRange(2))?;
    Ok(*entry)
}

/// Decode one Codebook 2 Huffman codeword from `reader`, returning
/// the codeword index in `0..=80`.
///
/// The decoder is a straight prefix-match: read one bit at a time
/// (MSB-first), look it up in a flat 81-entry table. The table is
/// small (max codeword length 9 bits, 81 entries) so a single
/// linear scan per bit-extend is cheaper than the storage and
/// build-time cost of a multi-level lookup acceleration table.
/// Returns [`Error::UnexpectedEnd`] on reader underflow.
///
/// The codebook is a **complete** prefix code over 9 bits (Kraft
/// equality `Σᵢ 2^(9 − Lᵢ) = 512 = 2⁹`), so any 9-bit prefix fully
/// read from `reader` is guaranteed to match exactly one entry — the
/// bottom of the loop is unreachable when `reader` produces 9 bits
/// without underflowing. A purely defensive `unreachable!()` guards
/// the loop fall-through; it is verified dead by the
/// `hcod2_is_complete` regression test that exhaustively walks all
/// `2⁹` 9-bit prefixes.
pub fn hcod2_decode(reader: &mut BitReader<'_>) -> Result<u32> {
    let mut acc: u32 = 0;
    for len in 1..=HCOD2_MAX_LEN {
        let bit = reader.read_u32(1).map_err(|_| Error::UnexpectedEnd)?;
        acc = (acc << 1) | bit;
        for (idx, &(entry_len, entry_cw)) in HCOD2.iter().enumerate() {
            if u32::from(entry_len) == len && u32::from(entry_cw) == acc {
                return Ok(idx as u32);
            }
        }
    }
    // Unreachable: HCOD2 is a complete 9-bit prefix code. The
    // `hcod2_is_complete` regression test verifies every 9-bit
    // prefix maps to exactly one entry.
    unreachable!("HCOD2 is a complete 9-bit prefix code; the 9-bit walk must match");
}

/// Write a Codebook 2 codeword to `writer` by index.
///
/// Convenience over `hcod2_encode` + manual `write_u32`. Returns
/// [`Error::SpectralCodebookIndexOutOfRange`] for `idx > 80`.
pub fn hcod2_write(writer: &mut BitWriter, idx: u32) -> Result<()> {
    let (len, cw) = hcod2_encode(idx)?;
    writer.write_u32(u32::from(cw), u32::from(len));
    Ok(())
}

// =============================================================================
// Table 4.A.4 — Spectrum Huffman Codebook 3
// =============================================================================
//
// 81 entries, indices 0..=80. Each row is `(length_in_bits,
// codeword)` with `codeword` right-aligned in a `u16` (MSB of the
// wire codeword at bit `length − 1`). Transcribed verbatim from
// ISO/IEC 14496-3:2009(E) §4.A.1 Table 4.A.4.
//
// The codebook is a complete prefix code: Σᵢ 2^(16 − Lᵢ) = 65536 = 2¹⁶.
// This is exhaustively verified by the `hcod3_is_complete` regression
// test (which walks every 16-bit prefix and asserts each maps to
// exactly one index).
//
// Codebook 3 is the first *unsigned* spectrum book: each tuple
// coefficient is a non-negative magnitude in `0..=LAV = 0..=2`, and
// the §4.6.3.3 sign-bit suffix carries the sign of each non-zero
// coefficient outside the Huffman codeword. The §4.6.3.3 index ↔
// 4-tuple translation lives in
// [`crate::spectral_codebook::decode_index_to_tuple`] /
// [`crate::spectral_codebook::encode_tuple_to_index`]; the sign-bit
// suffix lives in
// [`crate::spectral_codebook::apply_sign_bits`] /
// [`crate::spectral_codebook::derive_sign_bits`].

/// Number of entries in Table 4.A.4 (`81`, indices `0..=80`).
pub const HCOD3_NUM_ENTRIES: usize = 81;

/// Maximum codeword length emitted by Table 4.A.4 (16 bits).
pub const HCOD3_MAX_LEN: u32 = 16;

/// Table 4.A.4 — `(length_in_bits, codeword)` per index `0..=80`.
///
/// Codewords are right-aligned within the `u16`. To emit one
/// bit-for-bit, write `codeword` as `length` bits MSB-first.
const HCOD3: [(u8, u16); HCOD3_NUM_ENTRIES] = [
    (1, 0x0000),  // 0 — zero-tuple, single bit `0`
    (4, 0x0009),  // 1
    (8, 0x00ef),  // 2
    (4, 0x000b),  // 3
    (5, 0x0019),  // 4
    (8, 0x00f0),  // 5
    (9, 0x01eb),  // 6
    (9, 0x01e6),  // 7
    (10, 0x03f2), // 8
    (4, 0x000a),  // 9
    (6, 0x0035),  // 10
    (9, 0x01ef),  // 11
    (6, 0x0034),  // 12
    (6, 0x0037),  // 13
    (9, 0x01e9),  // 14
    (9, 0x01ed),  // 15
    (9, 0x01e7),  // 16
    (10, 0x03f3), // 17
    (9, 0x01ee),  // 18
    (10, 0x03ed), // 19
    (13, 0x1ffa), // 20
    (9, 0x01ec),  // 21
    (9, 0x01f2),  // 22
    (11, 0x07f9), // 23
    (11, 0x07f8), // 24
    (10, 0x03f8), // 25
    (12, 0x0ff8), // 26
    (4, 0x0008),  // 27
    (6, 0x0038),  // 28
    (10, 0x03f6), // 29
    (6, 0x0036),  // 30
    (7, 0x0075),  // 31
    (10, 0x03f1), // 32
    (10, 0x03eb), // 33
    (10, 0x03ec), // 34
    (12, 0x0ff4), // 35
    (5, 0x0018),  // 36
    (7, 0x0076),  // 37
    (11, 0x07f4), // 38
    (6, 0x0039),  // 39
    (7, 0x0074),  // 40
    (10, 0x03ef), // 41
    (9, 0x01f3),  // 42
    (9, 0x01f4),  // 43
    (11, 0x07f6), // 44
    (9, 0x01e8),  // 45
    (10, 0x03ea), // 46
    (13, 0x1ffc), // 47
    (8, 0x00f2),  // 48
    (9, 0x01f1),  // 49
    (12, 0x0ffb), // 50
    (10, 0x03f5), // 51
    (11, 0x07f3), // 52
    (12, 0x0ffc), // 53
    (8, 0x00ee),  // 54
    (10, 0x03f7), // 55
    (15, 0x7ffe), // 56
    (9, 0x01f0),  // 57
    (11, 0x07f5), // 58
    (15, 0x7ffd), // 59
    (13, 0x1ffb), // 60
    (14, 0x3ffa), // 61
    (16, 0xffff), // 62
    (8, 0x00f1),  // 63
    (10, 0x03f0), // 64
    (14, 0x3ffc), // 65
    (9, 0x01ea),  // 66
    (10, 0x03ee), // 67
    (14, 0x3ffb), // 68
    (12, 0x0ff6), // 69
    (12, 0x0ffa), // 70
    (15, 0x7ffc), // 71
    (11, 0x07f2), // 72
    (12, 0x0ff5), // 73
    (16, 0xfffe), // 74
    (10, 0x03f4), // 75
    (11, 0x07f7), // 76
    (15, 0x7ffb), // 77
    (12, 0x0ff7), // 78
    (12, 0x0ff9), // 79
    (15, 0x7ffa), // 80
];

/// Encode a Codebook 3 codeword index (`0..=80`) to the wire Huffman
/// codeword from Table 4.A.4.
///
/// Returns `(length_in_bits, codeword)` with `codeword` right-aligned
/// in the `u16` (MSB at bit `length − 1`). Out-of-range `idx`
/// produces [`Error::SpectralCodebookIndexOutOfRange`] carrying the
/// codebook number `3`; the legal range is `0..=80` (the 81-entry
/// `3^4` enumeration of every legal unsigned 4-tuple with each
/// coefficient in `0..=LAV = 0..=2`).
///
/// The inverse of [`hcod3_decode`]. The sign-bit suffix for each
/// non-zero coefficient is *not* part of the returned codeword — the
/// caller emits sign bits separately per
/// [`crate::spectral_codebook::derive_sign_bits`].
pub fn hcod3_encode(idx: u32) -> Result<(u8, u16)> {
    let entry = HCOD3
        .get(idx as usize)
        .ok_or(Error::SpectralCodebookIndexOutOfRange(3))?;
    Ok(*entry)
}

/// Decode one Codebook 3 Huffman codeword from `reader`, returning
/// the codeword index in `0..=80`.
///
/// The decoder is a straight prefix-match: read one bit at a time
/// (MSB-first), look it up in a flat 81-entry table. The table is
/// small (max codeword length 16 bits, 81 entries) so a single
/// linear scan per bit-extend is cheaper than the storage and
/// build-time cost of a multi-level lookup acceleration table.
/// Returns [`Error::UnexpectedEnd`] on reader underflow.
///
/// The codebook is a **complete** prefix code over 16 bits (Kraft
/// equality `Σᵢ 2^(16 − Lᵢ) = 65536 = 2¹⁶`), so any 16-bit prefix
/// fully read from `reader` is guaranteed to match exactly one
/// entry — the bottom of the loop is unreachable when `reader`
/// produces 16 bits without underflowing. A purely defensive
/// `unreachable!()` guards the loop fall-through; it is verified
/// dead by the `hcod3_is_complete` regression test that exhaustively
/// walks all `2¹⁶` 16-bit prefixes.
///
/// The sign-bit suffix for non-zero coefficients is *not* consumed
/// here — the caller pairs the returned index with the §4.6.3.3
/// translation and then reads exactly one sign bit per non-zero
/// coefficient in low-frequency-first order.
pub fn hcod3_decode(reader: &mut BitReader<'_>) -> Result<u32> {
    let mut acc: u32 = 0;
    for len in 1..=HCOD3_MAX_LEN {
        let bit = reader.read_u32(1).map_err(|_| Error::UnexpectedEnd)?;
        acc = (acc << 1) | bit;
        for (idx, &(entry_len, entry_cw)) in HCOD3.iter().enumerate() {
            if u32::from(entry_len) == len && u32::from(entry_cw) == acc {
                return Ok(idx as u32);
            }
        }
    }
    // Unreachable: HCOD3 is a complete 16-bit prefix code. The
    // `hcod3_is_complete` regression test verifies every 16-bit
    // prefix maps to exactly one entry.
    unreachable!("HCOD3 is a complete 16-bit prefix code; the 16-bit walk must match");
}

/// Write a Codebook 3 codeword to `writer` by index.
///
/// Convenience over `hcod3_encode` + manual `write_u32`. Returns
/// [`Error::SpectralCodebookIndexOutOfRange`] for `idx > 80`. The
/// caller is responsible for emitting the §4.6.3.3 sign bits for
/// every non-zero coefficient after this call.
pub fn hcod3_write(writer: &mut BitWriter, idx: u32) -> Result<()> {
    let (len, cw) = hcod3_encode(idx)?;
    writer.write_u32(u32::from(cw), u32::from(len));
    Ok(())
}

// =============================================================================
// Table 4.A.5 — Spectrum Huffman Codebook 4
// =============================================================================
//
// 81 entries, indices 0..=80. Each row is `(length_in_bits,
// codeword)` with `codeword` right-aligned in a `u16` (MSB of the
// wire codeword at bit `length − 1`). Transcribed verbatim from
// ISO/IEC 14496-3:2001(E) §4.A.1 Table 4.A.5.
//
// The codebook is a complete prefix code: Σᵢ 2^(12 − Lᵢ) = 4096 = 2¹².
// This is exhaustively verified by the `hcod4_is_complete` regression
// test (which walks every 12-bit prefix and asserts each maps to
// exactly one index).
//
// Codebook 4 shares Codebook 3's unsigned dim-4 LAV-2 tuple universe
// (Table 4.95 row 4 = row 3 except for the source-table column);
// the §4.6.3.3 index ↔ 4-tuple translation in
// [`crate::spectral_codebook`] is reused as-is. The §4.6.3.3 sign-bit
// suffix lives in [`crate::spectral_codebook::apply_sign_bits`] /
// [`crate::spectral_codebook::derive_sign_bits`].

/// Number of entries in Table 4.A.5 (`81`, indices `0..=80`).
pub const HCOD4_NUM_ENTRIES: usize = 81;

/// Maximum codeword length emitted by Table 4.A.5 (12 bits).
pub const HCOD4_MAX_LEN: u32 = 12;

/// Table 4.A.5 — `(length_in_bits, codeword)` per index `0..=80`.
///
/// Codewords are right-aligned within the `u16`. To emit one
/// bit-for-bit, write `codeword` as `length` bits MSB-first.
const HCOD4: [(u8, u16); HCOD4_NUM_ENTRIES] = [
    (4, 0x007),  // 0
    (5, 0x016),  // 1
    (8, 0x0f6),  // 2
    (5, 0x018),  // 3
    (4, 0x008),  // 4
    (8, 0x0ef),  // 5
    (9, 0x1ef),  // 6
    (8, 0x0f3),  // 7
    (11, 0x7f8), // 8
    (5, 0x019),  // 9
    (5, 0x017),  // 10
    (8, 0x0ed),  // 11
    (5, 0x015),  // 12
    (4, 0x001),  // 13
    (8, 0x0e2),  // 14
    (8, 0x0f0),  // 15
    (7, 0x070),  // 16
    (10, 0x3f0), // 17
    (9, 0x1ee),  // 18
    (8, 0x0f1),  // 19
    (11, 0x7fa), // 20
    (8, 0x0ee),  // 21
    (8, 0x0e4),  // 22
    (10, 0x3f2), // 23
    (11, 0x7f6), // 24
    (10, 0x3ef), // 25
    (11, 0x7fd), // 26
    (4, 0x005),  // 27
    (5, 0x014),  // 28
    (8, 0x0f2),  // 29
    (4, 0x009),  // 30
    (4, 0x004),  // 31
    (8, 0x0e5),  // 32
    (8, 0x0f4),  // 33
    (8, 0x0e8),  // 34
    (10, 0x3f4), // 35
    (4, 0x006),  // 36
    (4, 0x002),  // 37
    (8, 0x0e7),  // 38
    (4, 0x003),  // 39
    (4, 0x000),  // 40 — shortest codeword in Codebook 4
    (7, 0x06b),  // 41
    (8, 0x0e3),  // 42
    (7, 0x069),  // 43
    (9, 0x1f3),  // 44
    (8, 0x0eb),  // 45
    (8, 0x0e6),  // 46
    (10, 0x3f6), // 47
    (7, 0x06e),  // 48
    (7, 0x06a),  // 49
    (9, 0x1f4),  // 50
    (10, 0x3ec), // 51
    (9, 0x1f0),  // 52
    (10, 0x3f9), // 53
    (8, 0x0f5),  // 54
    (8, 0x0ec),  // 55
    (11, 0x7fb), // 56
    (8, 0x0ea),  // 57
    (7, 0x06f),  // 58
    (10, 0x3f7), // 59
    (11, 0x7f9), // 60
    (10, 0x3f3), // 61
    (12, 0xfff), // 62
    (8, 0x0e9),  // 63
    (7, 0x06d),  // 64
    (10, 0x3f8), // 65
    (7, 0x06c),  // 66
    (7, 0x068),  // 67
    (9, 0x1f5),  // 68
    (10, 0x3ee), // 69
    (9, 0x1f2),  // 70
    (11, 0x7f4), // 71
    (11, 0x7f7), // 72
    (10, 0x3f1), // 73
    (12, 0xffe), // 74
    (10, 0x3ed), // 75
    (9, 0x1f1),  // 76
    (11, 0x7f5), // 77
    (11, 0x7fe), // 78
    (10, 0x3f5), // 79
    (11, 0x7fc), // 80
];

/// Encode a Codebook 4 codeword index (`0..=80`) to the wire Huffman
/// codeword from Table 4.A.5.
///
/// Returns `(length_in_bits, codeword)` with `codeword` right-aligned
/// in the `u16` (MSB at bit `length − 1`). Out-of-range `idx`
/// produces [`Error::SpectralCodebookIndexOutOfRange`] carrying the
/// codebook number `4`; the legal range is `0..=80` (the 81-entry
/// `3^4` enumeration of every legal unsigned 4-tuple with each
/// coefficient in `0..=LAV = 0..=2` — the same universe as Codebook
/// 3).
///
/// The inverse of [`hcod4_decode`]. The sign-bit suffix for each
/// non-zero coefficient is *not* part of the returned codeword — the
/// caller emits sign bits separately per
/// [`crate::spectral_codebook::derive_sign_bits`].
pub fn hcod4_encode(idx: u32) -> Result<(u8, u16)> {
    let entry = HCOD4
        .get(idx as usize)
        .ok_or(Error::SpectralCodebookIndexOutOfRange(4))?;
    Ok(*entry)
}

/// Decode one Codebook 4 Huffman codeword from `reader`, returning
/// the codeword index in `0..=80`.
///
/// The decoder is a straight prefix-match: read one bit at a time
/// (MSB-first), look it up in a flat 81-entry table. The table is
/// small (max codeword length 12 bits, 81 entries) so a single
/// linear scan per bit-extend is cheaper than the storage and
/// build-time cost of a multi-level lookup acceleration table.
/// Returns [`Error::UnexpectedEnd`] on reader underflow.
///
/// The codebook is a **complete** prefix code over 12 bits (Kraft
/// equality `Σᵢ 2^(12 − Lᵢ) = 4096 = 2¹²`), so any 12-bit prefix
/// fully read from `reader` is guaranteed to match exactly one
/// entry — the bottom of the loop is unreachable when `reader`
/// produces 12 bits without underflowing. A purely defensive
/// `unreachable!()` guards the loop fall-through; it is verified
/// dead by the `hcod4_is_complete` regression test that exhaustively
/// walks all `2¹²` 12-bit prefixes.
///
/// The sign-bit suffix for non-zero coefficients is *not* consumed
/// here — the caller pairs the returned index with the §4.6.3.3
/// translation and then reads exactly one sign bit per non-zero
/// coefficient in low-frequency-first order.
pub fn hcod4_decode(reader: &mut BitReader<'_>) -> Result<u32> {
    let mut acc: u32 = 0;
    for len in 1..=HCOD4_MAX_LEN {
        let bit = reader.read_u32(1).map_err(|_| Error::UnexpectedEnd)?;
        acc = (acc << 1) | bit;
        for (idx, &(entry_len, entry_cw)) in HCOD4.iter().enumerate() {
            if u32::from(entry_len) == len && u32::from(entry_cw) == acc {
                return Ok(idx as u32);
            }
        }
    }
    // Unreachable: HCOD4 is a complete 12-bit prefix code. The
    // `hcod4_is_complete` regression test verifies every 12-bit
    // prefix maps to exactly one entry.
    unreachable!("HCOD4 is a complete 12-bit prefix code; the 12-bit walk must match");
}

/// Write a Codebook 4 codeword to `writer` by index.
///
/// Convenience over `hcod4_encode` + manual `write_u32`. Returns
/// [`Error::SpectralCodebookIndexOutOfRange`] for `idx > 80`. The
/// caller is responsible for emitting the §4.6.3.3 sign bits for
/// every non-zero coefficient after this call.
pub fn hcod4_write(writer: &mut BitWriter, idx: u32) -> Result<()> {
    let (len, cw) = hcod4_encode(idx)?;
    writer.write_u32(u32::from(cw), u32::from(len));
    Ok(())
}

// =============================================================================
// Table 4.A.6 — Spectrum Huffman Codebook 5
// =============================================================================
//
// 81 entries, indices 0..=80. Each row is `(length_in_bits,
// codeword)` with `codeword` right-aligned in a `u16` (MSB of the
// wire codeword at bit `length − 1`). Transcribed verbatim from
// ISO/IEC 14496-3:2001(E) §4.A.1 Table 4.A.6.
//
// The codebook is a complete prefix code: Σᵢ 2^(13 − Lᵢ) = 8192 = 2¹³.
// This is exhaustively verified by the `hcod5_is_complete` regression
// test (which walks every 13-bit prefix and asserts each maps to
// exactly one index).
//
// Codebook 5 is the first **pair** spectrum book (Table 4.95 row 5:
// `unsigned_cb = 0`, `dim = 2`, `LAV = 4`). Per §4.6.3.3 the
// index↔tuple translation evaluates `idx = (y + LAV) * 9 + (z + LAV)`
// so the signed pair lattice spans `(-4, -4) .. (+4, +4)` and the
// zero-tuple `(0, 0)` lands at the centre row index 40. The
// [`crate::spectral_codebook`] §4.6.3.3 dispatcher already handles
// the dim=2 path; this module owns only the codeword wire layer.
// Because Codebook 5 is signed, no sign-bit suffix follows the
// codeword on the wire — the index alone fully specifies the
// signed pair.

/// Number of entries in Table 4.A.6 (`81`, indices `0..=80`).
pub const HCOD5_NUM_ENTRIES: usize = 81;

/// Maximum codeword length emitted by Table 4.A.6 (13 bits).
pub const HCOD5_MAX_LEN: u32 = 13;

/// Table 4.A.6 — `(length_in_bits, codeword)` per index `0..=80`.
///
/// Codewords are right-aligned within the `u16`. To emit one
/// bit-for-bit, write `codeword` as `length` bits MSB-first.
const HCOD5: [(u8, u16); HCOD5_NUM_ENTRIES] = [
    (13, 0x1fff), // 0  — (y, z) = (-4, -4); one of the four 13-bit corners
    (12, 0xff7),  // 1
    (11, 0x7f4),  // 2
    (11, 0x7e8),  // 3
    (10, 0x3f1),  // 4
    (11, 0x7ee),  // 5
    (11, 0x7f9),  // 6
    (12, 0xff8),  // 7
    (13, 0x1ffd), // 8  — (y, z) = (-4, +4); 13-bit corner
    (12, 0xffd),  // 9
    (11, 0x7f1),  // 10
    (10, 0x3e8),  // 11
    (9, 0x1e8),   // 12
    (8, 0xf0),    // 13
    (9, 0x1ec),   // 14
    (10, 0x3ee),  // 15
    (11, 0x7f2),  // 16
    (12, 0xffa),  // 17
    (12, 0xff4),  // 18
    (10, 0x3ef),  // 19
    (9, 0x1f2),   // 20
    (8, 0xe8),    // 21
    (7, 0x70),    // 22
    (8, 0xec),    // 23
    (9, 0x1f0),   // 24
    (10, 0x3ea),  // 25
    (11, 0x7f3),  // 26
    (11, 0x7eb),  // 27
    (9, 0x1eb),   // 28
    (8, 0xea),    // 29
    (5, 0x1a),    // 30
    (4, 0x8),     // 31
    (5, 0x19),    // 32
    (8, 0xee),    // 33
    (9, 0x1ef),   // 34
    (11, 0x7ed),  // 35
    (10, 0x3f0),  // 36
    (8, 0xf2),    // 37
    (7, 0x73),    // 38
    (4, 0xb),     // 39
    (1, 0x0),     // 40 — (y, z) = (0, 0); single-bit zero codeword
    (4, 0xa),     // 41
    (7, 0x71),    // 42
    (8, 0xf3),    // 43
    (11, 0x7e9),  // 44
    (11, 0x7ef),  // 45
    (9, 0x1ee),   // 46
    (8, 0xef),    // 47
    (5, 0x18),    // 48
    (4, 0x9),     // 49
    (5, 0x1b),    // 50
    (8, 0xeb),    // 51
    (9, 0x1e9),   // 52
    (11, 0x7ec),  // 53
    (11, 0x7f6),  // 54
    (10, 0x3eb),  // 55
    (9, 0x1f3),   // 56
    (8, 0xed),    // 57
    (7, 0x72),    // 58
    (8, 0xe9),    // 59
    (9, 0x1f1),   // 60
    (10, 0x3ed),  // 61
    (11, 0x7f7),  // 62
    (12, 0xff6),  // 63
    (11, 0x7f0),  // 64
    (10, 0x3e9),  // 65
    (9, 0x1ed),   // 66
    (8, 0xf1),    // 67
    (9, 0x1ea),   // 68
    (10, 0x3ec),  // 69
    (11, 0x7f8),  // 70
    (12, 0xff9),  // 71
    (13, 0x1ffc), // 72 — (y, z) = (+4, -4); 13-bit corner
    (12, 0xffc),  // 73
    (12, 0xff5),  // 74
    (11, 0x7ea),  // 75
    (10, 0x3f3),  // 76
    (10, 0x3f2),  // 77
    (11, 0x7f5),  // 78
    (12, 0xffb),  // 79
    (13, 0x1ffe), // 80 — (y, z) = (+4, +4); 13-bit corner
];

/// Encode a Codebook 5 codeword index (`0..=80`) to the wire Huffman
/// codeword from Table 4.A.6.
///
/// Returns `(length_in_bits, codeword)` with `codeword` right-aligned
/// in the `u16` (MSB at bit `length − 1`). Out-of-range `idx`
/// produces [`Error::SpectralCodebookIndexOutOfRange`] carrying the
/// codebook number `5`; the legal range is `0..=80` (the 81-entry
/// `9^2` enumeration of every legal signed 2-tuple with each
/// coefficient in `-LAV..=+LAV = -4..=+4`).
///
/// The inverse of [`hcod5_decode`]. Because Codebook 5 is signed,
/// no sign-bit suffix follows the codeword on the wire — the
/// `offset = LAV = 4` shift inside the §4.6.3.3 translation already
/// encodes every coefficient's sign into the index.
pub fn hcod5_encode(idx: u32) -> Result<(u8, u16)> {
    let entry = HCOD5
        .get(idx as usize)
        .ok_or(Error::SpectralCodebookIndexOutOfRange(5))?;
    Ok(*entry)
}

/// Decode one Codebook 5 Huffman codeword from `reader`, returning
/// the codeword index in `0..=80`.
///
/// The decoder is a straight prefix-match: read one bit at a time
/// (MSB-first), look it up in a flat 81-entry table. The table is
/// small (max codeword length 13 bits, 81 entries) so a single
/// linear scan per bit-extend is cheaper than the storage and
/// build-time cost of a multi-level lookup acceleration table.
/// Returns [`Error::UnexpectedEnd`] on reader underflow.
///
/// The codebook is a **complete** prefix code over 13 bits (Kraft
/// equality `Σᵢ 2^(13 − Lᵢ) = 8192 = 2¹³`), so any 13-bit prefix
/// fully read from `reader` is guaranteed to match exactly one
/// entry — the bottom of the loop is unreachable when `reader`
/// produces 13 bits without underflowing. A purely defensive
/// `unreachable!()` guards the loop fall-through; it is verified
/// dead by the `hcod5_is_complete` regression test that exhaustively
/// walks all `2¹³` 13-bit prefixes.
///
/// No sign-bit suffix is read here — Codebook 5 is signed, so every
/// coefficient's sign is already baked into the index via the
/// `offset = LAV = 4` §4.6.3.3 polynomial.
pub fn hcod5_decode(reader: &mut BitReader<'_>) -> Result<u32> {
    let mut acc: u32 = 0;
    for len in 1..=HCOD5_MAX_LEN {
        let bit = reader.read_u32(1).map_err(|_| Error::UnexpectedEnd)?;
        acc = (acc << 1) | bit;
        for (idx, &(entry_len, entry_cw)) in HCOD5.iter().enumerate() {
            if u32::from(entry_len) == len && u32::from(entry_cw) == acc {
                return Ok(idx as u32);
            }
        }
    }
    // Unreachable: HCOD5 is a complete 13-bit prefix code. The
    // `hcod5_is_complete` regression test verifies every 13-bit
    // prefix maps to exactly one entry.
    unreachable!("HCOD5 is a complete 13-bit prefix code; the 13-bit walk must match");
}

/// Write a Codebook 5 codeword to `writer` by index.
///
/// Convenience over `hcod5_encode` + manual `write_u32`. Returns
/// [`Error::SpectralCodebookIndexOutOfRange`] for `idx > 80`. No
/// sign bits follow on the wire (Codebook 5 is signed).
pub fn hcod5_write(writer: &mut BitWriter, idx: u32) -> Result<()> {
    let (len, cw) = hcod5_encode(idx)?;
    writer.write_u32(u32::from(cw), u32::from(len));
    Ok(())
}

// =============================================================================
// Table 4.A.7 — Spectrum Huffman Codebook 6
// =============================================================================
//
// 81 entries, indices 0..=80. Each row is `(length_in_bits,
// codeword)` with `codeword` right-aligned in a `u16` (MSB of the wire
// codeword at bit `length − 1`). Transcribed verbatim from ISO/IEC
// 14496-3:2001(E) §4.A.1 Table 4.A.7.
//
// The codebook is a complete prefix code: Σᵢ 2^(11 − Lᵢ) = 2048 = 2¹¹.
// This is exhaustively verified by the `hcod6_is_complete` regression
// test (which walks every 11-bit prefix and asserts each maps to
// exactly one index).
//
// Codebook 6 is the second signed pair spectrum book (Table 4.95 row 6:
// `unsigned_cb = 0`, `dim = 2`, `LAV = 4` → `9^2 = 81` entries, each
// coefficient in `-4..=+4`). The §4.6.3.3 polynomial places the
// zero-tuple `(0, 0)` at the centre of the index range (index 40);
// the four `(±4, ±4)` lattice corners sit at indices 0, 8, 72, 80.
// Because Codebook 6 is signed, no sign-bit suffix follows the
// codeword on the wire.

/// Number of entries in Table 4.A.7 (`81`, indices `0..=80`).
pub const HCOD6_NUM_ENTRIES: usize = 81;

/// Maximum codeword length emitted by Table 4.A.7 (11 bits).
pub const HCOD6_MAX_LEN: u32 = 11;

/// Table 4.A.7 — `(length_in_bits, codeword)` per index `0..=80`.
///
/// Codewords are right-aligned within the `u16`. To emit one
/// bit-for-bit, write `codeword` as `length` bits MSB-first.
const HCOD6: [(u8, u16); HCOD6_NUM_ENTRIES] = [
    (11, 0x7fe), // 0  — (y, z) = (-4, -4)
    (10, 0x3fd), // 1
    (9, 0x1f1),  // 2
    (9, 0x1eb),  // 3
    (9, 0x1f4),  // 4
    (9, 0x1ea),  // 5
    (9, 0x1f0),  // 6
    (10, 0x3fc), // 7
    (11, 0x7fd), // 8  — (y, z) = (-4, +4)
    (10, 0x3f6), // 9
    (9, 0x1e5),  // 10
    (8, 0xea),   // 11
    (7, 0x6c),   // 12
    (7, 0x71),   // 13
    (7, 0x68),   // 14
    (8, 0xf0),   // 15
    (9, 0x1e6),  // 16
    (10, 0x3f7), // 17
    (9, 0x1f3),  // 18
    (8, 0xef),   // 19
    (6, 0x32),   // 20
    (6, 0x27),   // 21
    (6, 0x28),   // 22
    (6, 0x26),   // 23
    (6, 0x31),   // 24
    (8, 0xeb),   // 25
    (9, 0x1f7),  // 26
    (9, 0x1e8),  // 27
    (7, 0x6f),   // 28
    (6, 0x2e),   // 29
    (4, 0x8),    // 30
    (4, 0x4),    // 31
    (4, 0x6),    // 32
    (6, 0x29),   // 33
    (7, 0x6b),   // 34
    (9, 0x1ee),  // 35
    (9, 0x1ef),  // 36
    (7, 0x72),   // 37
    (6, 0x2d),   // 38
    (4, 0x2),    // 39
    (4, 0x0),    // 40 — zero-tuple (y, z) = (0, 0), 4-bit `0b0000`
    (4, 0x3),    // 41
    (6, 0x2f),   // 42
    (7, 0x73),   // 43
    (9, 0x1fa),  // 44
    (9, 0x1e7),  // 45
    (7, 0x6e),   // 46
    (6, 0x2b),   // 47
    (4, 0x7),    // 48
    (4, 0x1),    // 49
    (4, 0x5),    // 50
    (6, 0x2c),   // 51
    (7, 0x6d),   // 52
    (9, 0x1ec),  // 53
    (9, 0x1f9),  // 54
    (8, 0xee),   // 55
    (6, 0x30),   // 56
    (6, 0x24),   // 57
    (6, 0x2a),   // 58
    (6, 0x25),   // 59
    (6, 0x33),   // 60
    (8, 0xec),   // 61
    (9, 0x1f2),  // 62
    (10, 0x3f8), // 63
    (9, 0x1e4),  // 64
    (8, 0xed),   // 65
    (7, 0x6a),   // 66
    (7, 0x70),   // 67
    (7, 0x69),   // 68
    (7, 0x74),   // 69
    (8, 0xf1),   // 70
    (10, 0x3fa), // 71
    (11, 0x7ff), // 72 — (y, z) = (+4, -4)
    (10, 0x3f9), // 73
    (9, 0x1f6),  // 74
    (9, 0x1ed),  // 75
    (9, 0x1f8),  // 76
    (9, 0x1e9),  // 77
    (9, 0x1f5),  // 78
    (10, 0x3fb), // 79
    (11, 0x7fc), // 80 — (y, z) = (+4, +4)
];

/// Encode a Codebook 6 codeword index (`0..=80`) to the wire Huffman
/// codeword from Table 4.A.7.
///
/// Returns `(length_in_bits, codeword)` with `codeword` right-aligned
/// in the `u16` (MSB at bit `length − 1`). Out-of-range `idx`
/// produces [`Error::SpectralCodebookIndexOutOfRange`]; the legal
/// range is `0..=80` (the 81-entry `9^2` enumeration of every legal
/// signed pair with each coefficient in `-4..=+4`).
///
/// The inverse of [`hcod6_decode`]. Because Codebook 6 is signed,
/// each tuple coefficient's sign is already encoded in the index via
/// the §4.6.3.3 `offset = LAV = 4` shift — no sign-bit suffix is
/// emitted after the codeword.
pub fn hcod6_encode(idx: u32) -> Result<(u8, u16)> {
    let entry = HCOD6
        .get(idx as usize)
        .ok_or(Error::SpectralCodebookIndexOutOfRange(6))?;
    Ok(*entry)
}

/// Decode one Codebook 6 Huffman codeword from `reader`, returning
/// the codeword index in `0..=80`.
///
/// The decoder is a straight prefix-match: read one bit at a time
/// (MSB-first), look it up in a flat 81-entry table. The table is
/// small (max codeword length 11 bits, 81 entries) so a single
/// linear scan per bit-extend is cheaper than the storage and
/// build-time cost of a multi-level lookup acceleration table.
/// Returns [`Error::UnexpectedEnd`] on reader underflow.
///
/// The codebook is a **complete** prefix code over 11 bits (Kraft
/// equality `Σᵢ 2^(11 − Lᵢ) = 2048 = 2¹¹`), so any 11-bit prefix
/// fully read from `reader` is guaranteed to match exactly one
/// entry — the bottom of the loop is unreachable when `reader`
/// produces 11 bits without underflowing. A purely defensive
/// `unreachable!()` guards the loop fall-through; it is verified
/// dead by the `hcod6_is_complete` regression test that exhaustively
/// walks all `2¹¹` 11-bit prefixes.
///
/// No sign-bit suffix is read here — Codebook 6 is signed, so every
/// `(y, z)` pair carries its sign inside the §4.6.3.3 index via the
/// `offset = LAV = 4` shift.
pub fn hcod6_decode(reader: &mut BitReader<'_>) -> Result<u32> {
    let mut acc: u32 = 0;
    for len in 1..=HCOD6_MAX_LEN {
        let bit = reader.read_u32(1).map_err(|_| Error::UnexpectedEnd)?;
        acc = (acc << 1) | bit;
        for (idx, &(entry_len, entry_cw)) in HCOD6.iter().enumerate() {
            if u32::from(entry_len) == len && u32::from(entry_cw) == acc {
                return Ok(idx as u32);
            }
        }
    }
    // Unreachable: HCOD6 is a complete 11-bit prefix code. The
    // `hcod6_is_complete` regression test verifies every 11-bit
    // prefix maps to exactly one entry.
    unreachable!("HCOD6 is a complete 11-bit prefix code; the 11-bit walk must match");
}

/// Write a Codebook 6 codeword to `writer` by index.
///
/// Convenience over `hcod6_encode` + manual `write_u32`. Returns
/// [`Error::SpectralCodebookIndexOutOfRange`] for `idx > 80`. No
/// sign bits follow on the wire (Codebook 6 is signed).
pub fn hcod6_write(writer: &mut BitWriter, idx: u32) -> Result<()> {
    let (len, cw) = hcod6_encode(idx)?;
    writer.write_u32(u32::from(cw), u32::from(len));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // -------------------------------------------------------------------
    // Table-shape invariants
    // -------------------------------------------------------------------

    #[test]
    fn hcod1_has_exactly_81_entries() {
        // 3^4 = 81 (signed LAV=1 → mod = 2*1+1 = 3, dim = 4).
        assert_eq!(HCOD1.len(), HCOD1_NUM_ENTRIES);
        assert_eq!(HCOD1_NUM_ENTRIES, 81);
    }

    #[test]
    fn hcod1_max_length_is_11_bits() {
        let max = HCOD1.iter().map(|&(len, _)| len).max().unwrap();
        assert_eq!(u32::from(max), HCOD1_MAX_LEN);
        assert_eq!(HCOD1_MAX_LEN, 11);
    }

    #[test]
    fn hcod1_min_length_is_one_bit_at_index_40() {
        // The zero-tuple (w, x, y, z) = (0, 0, 0, 0) at index 40
        // gets the single bit `0`. Every other index has length >= 5.
        for (idx, &(len, cw)) in HCOD1.iter().enumerate() {
            if idx == 40 {
                assert_eq!(len, 1, "index 40 must be 1-bit");
                assert_eq!(cw, 0, "index 40 codeword must be `0`");
            } else {
                assert!(
                    len >= 5,
                    "every non-zero-tuple index must have length >= 5; idx={} len={}",
                    idx,
                    len
                );
            }
        }
    }

    #[test]
    fn hcod1_codewords_fit_their_declared_length() {
        for (idx, &(len, cw)) in HCOD1.iter().enumerate() {
            let max = if len == 0 { 0 } else { (1u32 << len) - 1 };
            assert!(
                u32::from(cw) <= max,
                "idx={}: codeword {:#x} does not fit {} bits",
                idx,
                cw,
                len
            );
        }
    }

    // -------------------------------------------------------------------
    // Kraft equality / completeness
    // -------------------------------------------------------------------

    #[test]
    fn hcod1_kraft_sum_is_two_to_the_eleven() {
        // Σᵢ 2^(L_max − Lᵢ) must equal 2^L_max for a complete code.
        let lmax = HCOD1_MAX_LEN;
        let mut sum: u64 = 0;
        for &(len, _) in &HCOD1 {
            sum += 1u64 << (lmax - u32::from(len));
        }
        assert_eq!(sum, 1u64 << lmax, "Kraft equality failed");
        assert_eq!(sum, 2048);
    }

    #[test]
    fn hcod1_is_complete() {
        // Walk every 11-bit prefix, decode it via the same path the
        // production decoder uses, and confirm every prefix yields
        // exactly one entry. Bonus: confirm the decoded index round-
        // trips back to the same codeword via `hcod1_encode`.
        for prefix in 0u32..(1u32 << HCOD1_MAX_LEN) {
            let bytes = [(prefix >> 3) as u8, ((prefix & 0x7) << 5) as u8];
            let mut br = BitReader::new(&bytes);
            let idx = hcod1_decode(&mut br).expect("11-bit prefix must decode");
            let (len, cw) = hcod1_encode(idx).expect("decoded index must round-trip");
            // The decoded codeword should match the leading `len` bits
            // of `prefix`.
            let lead = prefix >> (HCOD1_MAX_LEN - u32::from(len));
            assert_eq!(
                u32::from(cw),
                lead,
                "round-trip prefix={:#05x} idx={} len={} cw={:#x}",
                prefix,
                idx,
                len,
                cw
            );
        }
    }

    // -------------------------------------------------------------------
    // Encoder API
    // -------------------------------------------------------------------

    #[test]
    fn encode_zero_tuple_is_single_zero_bit() {
        // Index 40 = the zero 4-tuple → 1-bit `0` codeword.
        let (len, cw) = hcod1_encode(40).unwrap();
        assert_eq!(len, 1);
        assert_eq!(cw, 0);
    }

    #[test]
    fn encode_first_entry_matches_table() {
        // Spec PDF Table 4.A.2 row 0: length 11, codeword 0x7f8.
        let (len, cw) = hcod1_encode(0).unwrap();
        assert_eq!(len, 11);
        assert_eq!(cw, 0x7f8);
    }

    #[test]
    fn encode_last_entry_matches_table() {
        // Spec PDF Table 4.A.2 row 80: length 11, codeword 0x7f4.
        let (len, cw) = hcod1_encode(80).unwrap();
        assert_eq!(len, 11);
        assert_eq!(cw, 0x7f4);
    }

    #[test]
    fn encode_rejects_out_of_range_index() {
        assert!(matches!(
            hcod1_encode(81),
            Err(Error::SpectralCodebookIndexOutOfRange(1))
        ));
        assert!(matches!(
            hcod1_encode(0xffff_ffff),
            Err(Error::SpectralCodebookIndexOutOfRange(1))
        ));
    }

    // -------------------------------------------------------------------
    // Decoder API
    // -------------------------------------------------------------------

    #[test]
    fn decode_single_zero_bit_yields_index_40() {
        // One byte starting with `0` followed by anything → idx 40.
        let bytes = [0b0111_1111u8];
        let mut br = BitReader::new(&bytes);
        let idx = hcod1_decode(&mut br).unwrap();
        assert_eq!(idx, 40);
        // Only one bit consumed; the remaining 7 are untouched.
        assert_eq!(br.bit_position(), 1);
    }

    #[test]
    fn decode_first_entry_round_trip() {
        // Index 0 → length 11, codeword 0x7f8 = 0b111_1111_1000.
        // Pack into 2 bytes left-aligned: 0xff, 0x00.
        let bytes = [0xff, 0x00];
        let mut br = BitReader::new(&bytes);
        let idx = hcod1_decode(&mut br).unwrap();
        assert_eq!(idx, 0);
        assert_eq!(br.bit_position(), 11);
    }

    #[test]
    fn decode_propagates_unexpected_end() {
        let bytes: [u8; 0] = [];
        let mut br = BitReader::new(&bytes);
        assert_eq!(hcod1_decode(&mut br), Err(Error::UnexpectedEnd));
    }

    // -------------------------------------------------------------------
    // Writer API
    // -------------------------------------------------------------------

    #[test]
    fn write_then_decode_round_trips_every_index() {
        for idx in 0..HCOD1_NUM_ENTRIES as u32 {
            let mut w = BitWriter::new();
            hcod1_write(&mut w, idx).unwrap();
            // Pad to byte boundary if needed so BitReader can consume.
            let (len, _) = hcod1_encode(idx).unwrap();
            let mut w2 = w;
            let pad = (8 - (u32::from(len) % 8)) % 8;
            if pad > 0 {
                w2.write_u32(0, pad);
            }
            let bytes = w2.into_bytes();
            let mut br = BitReader::new(&bytes);
            let decoded = hcod1_decode(&mut br).unwrap();
            assert_eq!(
                decoded, idx,
                "round-trip mismatch at idx={} (encoded as {} bits)",
                idx, len
            );
        }
    }

    #[test]
    fn write_rejects_out_of_range_index() {
        let mut w = BitWriter::new();
        assert!(matches!(
            hcod1_write(&mut w, 81),
            Err(Error::SpectralCodebookIndexOutOfRange(1))
        ));
    }

    // -------------------------------------------------------------------
    // Codebook 2 — Table 4.A.3
    // -------------------------------------------------------------------

    #[test]
    fn hcod2_has_exactly_81_entries() {
        // 3^4 = 81 (signed LAV=1 → mod = 2*1+1 = 3, dim = 4) — same
        // tuple universe as Codebook 1.
        assert_eq!(HCOD2.len(), HCOD2_NUM_ENTRIES);
        assert_eq!(HCOD2_NUM_ENTRIES, 81);
    }

    #[test]
    fn hcod2_max_length_is_9_bits() {
        let max = HCOD2.iter().map(|&(len, _)| len).max().unwrap();
        assert_eq!(u32::from(max), HCOD2_MAX_LEN);
        assert_eq!(HCOD2_MAX_LEN, 9);
    }

    #[test]
    fn hcod2_min_length_is_three_bits_at_index_40() {
        // The zero-tuple (w, x, y, z) = (0, 0, 0, 0) at index 40
        // gets a 3-bit codeword `0b000` (vs the 1-bit `0` of
        // Codebook 1). Every other index has length >= 4.
        for (idx, &(len, cw)) in HCOD2.iter().enumerate() {
            if idx == 40 {
                assert_eq!(len, 3, "index 40 must be 3-bit");
                assert_eq!(cw, 0, "index 40 codeword must be `0`");
            } else {
                assert!(
                    len >= 4,
                    "every non-zero-tuple index must have length >= 4; idx={} len={}",
                    idx,
                    len
                );
            }
        }
    }

    #[test]
    fn hcod2_codewords_fit_their_declared_length() {
        for (idx, &(len, cw)) in HCOD2.iter().enumerate() {
            let max = if len == 0 { 0 } else { (1u32 << len) - 1 };
            assert!(
                u32::from(cw) <= max,
                "idx={}: codeword {:#x} does not fit {} bits",
                idx,
                cw,
                len
            );
        }
    }

    #[test]
    fn hcod2_kraft_sum_is_two_to_the_nine() {
        // Σᵢ 2^(L_max − Lᵢ) must equal 2^L_max for a complete code.
        let lmax = HCOD2_MAX_LEN;
        let mut sum: u64 = 0;
        for &(len, _) in &HCOD2 {
            sum += 1u64 << (lmax - u32::from(len));
        }
        assert_eq!(sum, 1u64 << lmax, "Kraft equality failed");
        assert_eq!(sum, 512);
    }

    #[test]
    fn hcod2_is_complete() {
        // Walk every 9-bit prefix, decode it via the production
        // decoder, and confirm every prefix yields exactly one entry.
        // Bonus: confirm the decoded index round-trips back to the
        // same codeword via `hcod2_encode`.
        for prefix in 0u32..(1u32 << HCOD2_MAX_LEN) {
            // Pack `prefix` (9 bits) left-aligned into two bytes:
            // [bits 8..1] [bit 0 << 7 | rest].
            let bytes = [(prefix >> 1) as u8, ((prefix & 0x1) << 7) as u8];
            let mut br = BitReader::new(&bytes);
            let idx = hcod2_decode(&mut br).expect("9-bit prefix must decode");
            let (len, cw) = hcod2_encode(idx).expect("decoded index must round-trip");
            // The decoded codeword should match the leading `len` bits
            // of `prefix`.
            let lead = prefix >> (HCOD2_MAX_LEN - u32::from(len));
            assert_eq!(
                u32::from(cw),
                lead,
                "round-trip prefix={:#05x} idx={} len={} cw={:#x}",
                prefix,
                idx,
                len,
                cw
            );
        }
    }

    #[test]
    fn encode_zero_tuple_is_three_zero_bits_in_codebook_2() {
        // Index 40 = the zero 4-tuple → 3-bit `000` codeword.
        let (len, cw) = hcod2_encode(40).unwrap();
        assert_eq!(len, 3);
        assert_eq!(cw, 0);
    }

    #[test]
    fn hcod2_encode_first_entry_matches_table() {
        // Spec PDF Table 4.A.3 row 0: length 9, codeword 0x1f3.
        let (len, cw) = hcod2_encode(0).unwrap();
        assert_eq!(len, 9);
        assert_eq!(cw, 0x1f3);
    }

    #[test]
    fn hcod2_encode_last_entry_matches_table() {
        // Spec PDF Table 4.A.3 row 80: length 9, codeword 0x1f6.
        let (len, cw) = hcod2_encode(80).unwrap();
        assert_eq!(len, 9);
        assert_eq!(cw, 0x1f6);
    }

    #[test]
    fn hcod2_encode_rejects_out_of_range_index() {
        assert!(matches!(
            hcod2_encode(81),
            Err(Error::SpectralCodebookIndexOutOfRange(2))
        ));
        assert!(matches!(
            hcod2_encode(0xffff_ffff),
            Err(Error::SpectralCodebookIndexOutOfRange(2))
        ));
    }

    #[test]
    fn hcod2_decode_three_zero_bits_yields_index_40() {
        // Three leading `0` bits → idx 40. Remaining 5 bits untouched.
        let bytes = [0b0001_1111u8];
        let mut br = BitReader::new(&bytes);
        let idx = hcod2_decode(&mut br).unwrap();
        assert_eq!(idx, 40);
        assert_eq!(br.bit_position(), 3);
    }

    #[test]
    fn hcod2_decode_first_entry_round_trip() {
        // Index 0 → length 9, codeword 0x1f3 = 0b1_1111_0011.
        // Pack into 2 bytes left-aligned: 0xf9, 0x80.
        // 0x1f3 << 7 = 0xf980 (16-bit big-endian).
        let bytes = [0xf9, 0x80];
        let mut br = BitReader::new(&bytes);
        let idx = hcod2_decode(&mut br).unwrap();
        assert_eq!(idx, 0);
        assert_eq!(br.bit_position(), 9);
    }

    #[test]
    fn hcod2_decode_propagates_unexpected_end() {
        let bytes: [u8; 0] = [];
        let mut br = BitReader::new(&bytes);
        assert_eq!(hcod2_decode(&mut br), Err(Error::UnexpectedEnd));
    }

    #[test]
    fn hcod2_write_then_decode_round_trips_every_index() {
        for idx in 0..HCOD2_NUM_ENTRIES as u32 {
            let mut w = BitWriter::new();
            hcod2_write(&mut w, idx).unwrap();
            let (len, _) = hcod2_encode(idx).unwrap();
            let mut w2 = w;
            let pad = (8 - (u32::from(len) % 8)) % 8;
            if pad > 0 {
                w2.write_u32(0, pad);
            }
            let bytes = w2.into_bytes();
            let mut br = BitReader::new(&bytes);
            let decoded = hcod2_decode(&mut br).unwrap();
            assert_eq!(
                decoded, idx,
                "round-trip mismatch at idx={} (encoded as {} bits)",
                idx, len
            );
        }
    }

    #[test]
    fn hcod2_write_rejects_out_of_range_index() {
        let mut w = BitWriter::new();
        assert!(matches!(
            hcod2_write(&mut w, 81),
            Err(Error::SpectralCodebookIndexOutOfRange(2))
        ));
    }

    // -------------------------------------------------------------------
    // Cross-check: Codebooks 1 and 2 share the same tuple universe but
    // never share a codeword for the same index (different lengths
    // and codewords for index 40 — 1 bit `0` vs 3 bits `0b000`).
    // -------------------------------------------------------------------

    #[test]
    fn codebook_1_and_2_disagree_on_zero_tuple_codeword_length() {
        let (l1, _) = hcod1_encode(40).unwrap();
        let (l2, _) = hcod2_encode(40).unwrap();
        // Both books carry the zero-tuple at index 40 but use
        // different codeword lengths: 1 bit for Codebook 1, 3 bits
        // for Codebook 2.
        assert_eq!(l1, 1);
        assert_eq!(l2, 3);
        assert_ne!(l1, l2);
    }

    // -------------------------------------------------------------------
    // Codebook 3 — Table 4.A.4
    // -------------------------------------------------------------------

    #[test]
    fn hcod3_has_exactly_81_entries() {
        // 3^4 = 81 (unsigned LAV=2 → mod = lav+1 = 3, dim = 4).
        assert_eq!(HCOD3.len(), HCOD3_NUM_ENTRIES);
        assert_eq!(HCOD3_NUM_ENTRIES, 81);
    }

    #[test]
    fn hcod3_max_length_is_16_bits() {
        let max = HCOD3.iter().map(|&(len, _)| len).max().unwrap();
        assert_eq!(u32::from(max), HCOD3_MAX_LEN);
        assert_eq!(HCOD3_MAX_LEN, 16);
    }

    #[test]
    fn hcod3_min_length_is_one_bit_at_index_0() {
        // Unsigned books put the all-zero magnitude n-tuple at
        // index 0 (vs index 40 for the signed books); it carries the
        // single bit `0`. Every other index has length >= 4.
        for (idx, &(len, cw)) in HCOD3.iter().enumerate() {
            if idx == 0 {
                assert_eq!(len, 1, "index 0 must be 1-bit");
                assert_eq!(cw, 0, "index 0 codeword must be `0`");
            } else {
                assert!(
                    len >= 4,
                    "every non-zero-tuple index must have length >= 4; idx={} len={}",
                    idx,
                    len
                );
            }
        }
    }

    #[test]
    fn hcod3_codewords_fit_their_declared_length() {
        for (idx, &(len, cw)) in HCOD3.iter().enumerate() {
            let max = if len == 0 { 0 } else { (1u32 << len) - 1 };
            assert!(
                u32::from(cw) <= max,
                "idx={}: codeword {:#x} does not fit {} bits",
                idx,
                cw,
                len
            );
        }
    }

    #[test]
    fn hcod3_kraft_sum_is_two_to_the_sixteen() {
        // Σᵢ 2^(L_max − Lᵢ) must equal 2^L_max for a complete code.
        let lmax = HCOD3_MAX_LEN;
        let mut sum: u64 = 0;
        for &(len, _) in &HCOD3 {
            sum += 1u64 << (lmax - u32::from(len));
        }
        assert_eq!(sum, 1u64 << lmax, "Kraft equality failed");
        assert_eq!(sum, 65536);
    }

    #[test]
    fn hcod3_is_complete() {
        // Walk every 16-bit prefix, decode it via the production
        // decoder, and confirm every prefix yields exactly one entry.
        // Bonus: confirm the decoded index round-trips back to the
        // same codeword via `hcod3_encode`.
        for prefix in 0u32..(1u32 << HCOD3_MAX_LEN) {
            // `prefix` already fits in 16 bits: pack left-aligned
            // into two bytes (high byte first).
            let bytes = [(prefix >> 8) as u8, (prefix & 0xff) as u8];
            let mut br = BitReader::new(&bytes);
            let idx = hcod3_decode(&mut br).expect("16-bit prefix must decode");
            let (len, cw) = hcod3_encode(idx).expect("decoded index must round-trip");
            // The decoded codeword should match the leading `len` bits
            // of `prefix`.
            let lead = prefix >> (HCOD3_MAX_LEN - u32::from(len));
            assert_eq!(
                u32::from(cw),
                lead,
                "round-trip prefix={:#06x} idx={} len={} cw={:#x}",
                prefix,
                idx,
                len,
                cw
            );
        }
    }

    #[test]
    fn hcod3_encode_zero_tuple_is_single_zero_bit() {
        // Index 0 = the zero 4-tuple `(0, 0, 0, 0)` in the unsigned
        // book → 1-bit `0` codeword.
        let (len, cw) = hcod3_encode(0).unwrap();
        assert_eq!(len, 1);
        assert_eq!(cw, 0);
    }

    #[test]
    fn hcod3_encode_last_entry_matches_table() {
        // Spec PDF Table 4.A.4 row 80: length 15, codeword 0x7ffa.
        let (len, cw) = hcod3_encode(80).unwrap();
        assert_eq!(len, 15);
        assert_eq!(cw, 0x7ffa);
    }

    #[test]
    fn hcod3_encode_index_62_is_the_only_full_16_bit_codeword_0xffff() {
        // Spec PDF Table 4.A.4 row 62: length 16, codeword 0xffff
        // (the all-ones 16-bit pattern). Verify by spot-check that
        // this is the unique row with codeword 0xffff.
        let (len, cw) = hcod3_encode(62).unwrap();
        assert_eq!(len, 16);
        assert_eq!(cw, 0xffff);
        let count_matching = HCOD3.iter().filter(|&&(_, c)| c == 0xffff).count();
        assert_eq!(count_matching, 1);
    }

    #[test]
    fn hcod3_encode_rejects_out_of_range_index() {
        assert!(matches!(
            hcod3_encode(81),
            Err(Error::SpectralCodebookIndexOutOfRange(3))
        ));
        assert!(matches!(
            hcod3_encode(0xffff_ffff),
            Err(Error::SpectralCodebookIndexOutOfRange(3))
        ));
    }

    #[test]
    fn hcod3_decode_single_zero_bit_yields_index_0() {
        // Leading `0` bit → idx 0 (the unsigned book's zero-tuple).
        let bytes = [0b0111_1111u8];
        let mut br = BitReader::new(&bytes);
        let idx = hcod3_decode(&mut br).unwrap();
        assert_eq!(idx, 0);
        // Only one bit consumed; the remaining 7 are untouched.
        assert_eq!(br.bit_position(), 1);
    }

    #[test]
    fn hcod3_decode_full_16_bit_codeword_round_trips() {
        // Index 62 → length 16, codeword 0xffff. Pack as two bytes.
        let bytes = [0xff, 0xff];
        let mut br = BitReader::new(&bytes);
        let idx = hcod3_decode(&mut br).unwrap();
        assert_eq!(idx, 62);
        assert_eq!(br.bit_position(), 16);
    }

    #[test]
    fn hcod3_decode_propagates_unexpected_end() {
        let bytes: [u8; 0] = [];
        let mut br = BitReader::new(&bytes);
        assert_eq!(hcod3_decode(&mut br), Err(Error::UnexpectedEnd));
    }

    #[test]
    fn hcod3_write_then_decode_round_trips_every_index() {
        for idx in 0..HCOD3_NUM_ENTRIES as u32 {
            let mut w = BitWriter::new();
            hcod3_write(&mut w, idx).unwrap();
            let (len, _) = hcod3_encode(idx).unwrap();
            let mut w2 = w;
            let pad = (8 - (u32::from(len) % 8)) % 8;
            if pad > 0 {
                w2.write_u32(0, pad);
            }
            let bytes = w2.into_bytes();
            let mut br = BitReader::new(&bytes);
            let decoded = hcod3_decode(&mut br).unwrap();
            assert_eq!(
                decoded, idx,
                "round-trip mismatch at idx={} (encoded as {} bits)",
                idx, len
            );
        }
    }

    #[test]
    fn hcod3_write_rejects_out_of_range_index() {
        let mut w = BitWriter::new();
        assert!(matches!(
            hcod3_write(&mut w, 81),
            Err(Error::SpectralCodebookIndexOutOfRange(3))
        ));
    }

    // -------------------------------------------------------------------
    // Cross-check: Codebook 3 zero-tuple sits at a different index
    // than Codebooks 1 / 2 because unsigned books use a different
    // index origin from signed books.
    // -------------------------------------------------------------------

    #[test]
    fn codebook_3_zero_tuple_lives_at_index_zero_not_forty() {
        // The zero magnitude 4-tuple `(0, 0, 0, 0)`:
        //   - signed book (mod = 3, offset = LAV = 1): polynomial
        //     evaluates to (0+1)*27 + (0+1)*9 + (0+1)*3 + (0+1) = 40.
        //   - unsigned book (mod = 3, offset = 0): polynomial
        //     evaluates to (0)*27 + (0)*9 + (0)*3 + (0) = 0.
        // So the zero-tuple lives at index 40 in HCOD1 / HCOD2 and
        // at index 0 in HCOD3. Both still carry a 1-bit codeword in
        // their respective books (Codebook 1 + 3); Codebook 2 trades
        // the 1-bit zero-tuple for a 3-bit one to free up the short
        // codes for the non-zero tuples its target statistics prefer.
        let (l1, cw1) = hcod1_encode(40).unwrap();
        let (l3, cw3) = hcod3_encode(0).unwrap();
        assert_eq!(l1, 1);
        assert_eq!(cw1, 0);
        assert_eq!(l3, 1);
        assert_eq!(cw3, 0);
    }

    // -------------------------------------------------------------------
    // Codebook 4 — Table 4.A.5
    // -------------------------------------------------------------------

    #[test]
    fn hcod4_has_exactly_81_entries() {
        // 3^4 = 81 (unsigned LAV=2 → mod = lav+1 = 3, dim = 4) — same
        // tuple universe as Codebook 3.
        assert_eq!(HCOD4.len(), HCOD4_NUM_ENTRIES);
        assert_eq!(HCOD4_NUM_ENTRIES, 81);
    }

    #[test]
    fn hcod4_max_length_is_12_bits() {
        let max = HCOD4.iter().map(|&(len, _)| len).max().unwrap();
        assert_eq!(u32::from(max), HCOD4_MAX_LEN);
        assert_eq!(HCOD4_MAX_LEN, 12);
    }

    #[test]
    fn hcod4_min_length_is_four_bits_at_index_40() {
        // The shortest codeword in Codebook 4 is 4 bits, parked at
        // index 40 with the all-zero pattern `0b0000`. Every other
        // index has length >= 4 (Codebook 4's distribution has a
        // dense 4-bit head: indices 0, 4, 13, 27, 30, 31, 36, 37, 39,
        // 40 all share length 4).
        let (len_40, cw_40) = (HCOD4[40].0, HCOD4[40].1);
        assert_eq!(len_40, 4, "index 40 must be 4-bit");
        assert_eq!(cw_40, 0, "index 40 codeword must be `0b0000`");
        for (idx, &(len, _)) in HCOD4.iter().enumerate() {
            assert!(
                len >= 4,
                "every index must have length >= 4; idx={} len={}",
                idx,
                len
            );
        }
    }

    #[test]
    fn hcod4_codewords_fit_their_declared_length() {
        for (idx, &(len, cw)) in HCOD4.iter().enumerate() {
            let max = if len == 0 { 0 } else { (1u32 << len) - 1 };
            assert!(
                u32::from(cw) <= max,
                "idx={}: codeword {:#x} does not fit {} bits",
                idx,
                cw,
                len
            );
        }
    }

    #[test]
    fn hcod4_kraft_sum_is_two_to_the_twelve() {
        // Σᵢ 2^(L_max − Lᵢ) must equal 2^L_max for a complete code.
        let lmax = HCOD4_MAX_LEN;
        let mut sum: u64 = 0;
        for &(len, _) in &HCOD4 {
            sum += 1u64 << (lmax - u32::from(len));
        }
        assert_eq!(sum, 1u64 << lmax, "Kraft equality failed");
        assert_eq!(sum, 4096);
    }

    #[test]
    fn hcod4_is_complete() {
        // Walk every 12-bit prefix, decode it via the production
        // decoder, and confirm every prefix yields exactly one entry.
        // Bonus: confirm the decoded index round-trips back to the
        // same codeword via `hcod4_encode`.
        for prefix in 0u32..(1u32 << HCOD4_MAX_LEN) {
            // Pack `prefix` (12 bits) left-aligned into two bytes:
            // high byte = bits 11..4, low byte = (bits 3..0) << 4.
            let bytes = [(prefix >> 4) as u8, ((prefix & 0xf) << 4) as u8];
            let mut br = BitReader::new(&bytes);
            let idx = hcod4_decode(&mut br).expect("12-bit prefix must decode");
            let (len, cw) = hcod4_encode(idx).expect("decoded index must round-trip");
            // The decoded codeword should match the leading `len` bits
            // of `prefix`.
            let lead = prefix >> (HCOD4_MAX_LEN - u32::from(len));
            assert_eq!(
                u32::from(cw),
                lead,
                "round-trip prefix={:#05x} idx={} len={} cw={:#x}",
                prefix,
                idx,
                len,
                cw
            );
        }
    }

    #[test]
    fn hcod4_encode_index_40_is_4_bit_zero_codeword() {
        // Spec PDF Table 4.A.5 row 40: length 4, codeword 0 (the
        // shortest codeword in the table).
        let (len, cw) = hcod4_encode(40).unwrap();
        assert_eq!(len, 4);
        assert_eq!(cw, 0);
    }

    #[test]
    fn hcod4_encode_first_entry_matches_table() {
        // Spec PDF Table 4.A.5 row 0: length 4, codeword 0x7.
        let (len, cw) = hcod4_encode(0).unwrap();
        assert_eq!(len, 4);
        assert_eq!(cw, 0x7);
    }

    #[test]
    fn hcod4_encode_last_entry_matches_table() {
        // Spec PDF Table 4.A.5 row 80: length 11, codeword 0x7fc.
        let (len, cw) = hcod4_encode(80).unwrap();
        assert_eq!(len, 11);
        assert_eq!(cw, 0x7fc);
    }

    #[test]
    fn hcod4_encode_indices_62_and_74_are_the_full_12_bit_codewords() {
        // Spec PDF Table 4.A.5 row 62: length 12, codeword 0xfff.
        // Spec PDF Table 4.A.5 row 74: length 12, codeword 0xffe.
        // These are the only two 12-bit rows in Codebook 4.
        let (len_62, cw_62) = hcod4_encode(62).unwrap();
        assert_eq!((len_62, cw_62), (12, 0xfff));
        let (len_74, cw_74) = hcod4_encode(74).unwrap();
        assert_eq!((len_74, cw_74), (12, 0xffe));
        let count_12_bit = HCOD4.iter().filter(|&&(l, _)| l == 12).count();
        assert_eq!(count_12_bit, 2);
    }

    #[test]
    fn hcod4_encode_rejects_out_of_range_index() {
        assert!(matches!(
            hcod4_encode(81),
            Err(Error::SpectralCodebookIndexOutOfRange(4))
        ));
        assert!(matches!(
            hcod4_encode(0xffff_ffff),
            Err(Error::SpectralCodebookIndexOutOfRange(4))
        ));
    }

    #[test]
    fn hcod4_decode_four_zero_bits_yields_index_40() {
        // Leading `0b0000` → idx 40 (Codebook 4's shortest codeword).
        // Remaining 4 bits of the byte untouched.
        let bytes = [0b0000_1111u8];
        let mut br = BitReader::new(&bytes);
        let idx = hcod4_decode(&mut br).unwrap();
        assert_eq!(idx, 40);
        assert_eq!(br.bit_position(), 4);
    }

    #[test]
    fn hcod4_decode_full_12_bit_codeword_round_trips_index_62() {
        // Index 62 → length 12, codeword 0xfff = 0b1111_1111_1111.
        // Pack left-aligned into 2 bytes: 0xff, 0xf0.
        let bytes = [0xff, 0xf0];
        let mut br = BitReader::new(&bytes);
        let idx = hcod4_decode(&mut br).unwrap();
        assert_eq!(idx, 62);
        assert_eq!(br.bit_position(), 12);
    }

    #[test]
    fn hcod4_decode_propagates_unexpected_end() {
        let bytes: [u8; 0] = [];
        let mut br = BitReader::new(&bytes);
        assert_eq!(hcod4_decode(&mut br), Err(Error::UnexpectedEnd));
    }

    #[test]
    fn hcod4_write_then_decode_round_trips_every_index() {
        for idx in 0..HCOD4_NUM_ENTRIES as u32 {
            let mut w = BitWriter::new();
            hcod4_write(&mut w, idx).unwrap();
            let (len, _) = hcod4_encode(idx).unwrap();
            let mut w2 = w;
            let pad = (8 - (u32::from(len) % 8)) % 8;
            if pad > 0 {
                w2.write_u32(0, pad);
            }
            let bytes = w2.into_bytes();
            let mut br = BitReader::new(&bytes);
            let decoded = hcod4_decode(&mut br).unwrap();
            assert_eq!(
                decoded, idx,
                "round-trip mismatch at idx={} (encoded as {} bits)",
                idx, len
            );
        }
    }

    #[test]
    fn hcod4_write_rejects_out_of_range_index() {
        let mut w = BitWriter::new();
        assert!(matches!(
            hcod4_write(&mut w, 81),
            Err(Error::SpectralCodebookIndexOutOfRange(4))
        ));
    }

    // -------------------------------------------------------------------
    // Cross-check: Codebook 3 and Codebook 4 share the unsigned dim-4
    // LAV-2 tuple universe (same Table 4.95 row shape) but assign
    // different codewords for the same tuple — Codebook 3 gives the
    // zero-tuple the single-bit codeword `0`; Codebook 4 lifts it to
    // a 4-bit `0b0111` and parks the 4-bit `0b0000` shortest at
    // index 40 instead.
    // -------------------------------------------------------------------

    #[test]
    fn codebook_3_and_4_disagree_on_zero_tuple_codeword() {
        let (l3, cw3) = hcod3_encode(0).unwrap();
        let (l4, cw4) = hcod4_encode(0).unwrap();
        assert_eq!((l3, cw3), (1, 0));
        assert_eq!((l4, cw4), (4, 0x7));
        // Codebook 4's shortest codeword sits at a different index
        // (40) with a different value (`0b0000`).
        let (l40, cw40) = hcod4_encode(40).unwrap();
        assert_eq!((l40, cw40), (4, 0));
    }

    // -------------------------------------------------------------------
    // Codebook 5 (Table 4.A.6) — signed dim-2 LAV-4 pair book
    // -------------------------------------------------------------------

    #[test]
    fn hcod5_has_exactly_81_entries() {
        // 9^2 = 81 (signed LAV=4 → mod = 2*4+1 = 9, dim = 2).
        assert_eq!(HCOD5.len(), HCOD5_NUM_ENTRIES);
        assert_eq!(HCOD5_NUM_ENTRIES, 81);
    }

    #[test]
    fn hcod5_max_length_is_13_bits() {
        let max = HCOD5.iter().map(|&(len, _)| len).max().unwrap();
        assert_eq!(u32::from(max), HCOD5_MAX_LEN);
        assert_eq!(HCOD5_MAX_LEN, 13);
    }

    #[test]
    fn hcod5_min_length_is_one_bit_at_index_40() {
        // The shortest codeword in Codebook 5 is the single bit `0`
        // at index 40 — the §4.6.3.3 zero-tuple `(0, 0)` for a
        // signed pair book with LAV = 4 lands at the centre of the
        // index range, not at the edges.
        let (len_40, cw_40) = (HCOD5[40].0, HCOD5[40].1);
        assert_eq!(len_40, 1, "index 40 must be 1-bit");
        assert_eq!(cw_40, 0, "index 40 codeword must be `0`");
        let count_1_bit = HCOD5.iter().filter(|&&(l, _)| l == 1).count();
        assert_eq!(count_1_bit, 1, "exactly one 1-bit codeword");
    }

    #[test]
    fn hcod5_codewords_fit_their_declared_length() {
        for (idx, &(len, cw)) in HCOD5.iter().enumerate() {
            let max = if len == 0 { 0 } else { (1u32 << len) - 1 };
            assert!(
                u32::from(cw) <= max,
                "idx={}: codeword {:#x} does not fit {} bits",
                idx,
                cw,
                len
            );
        }
    }

    #[test]
    fn hcod5_kraft_sum_is_two_to_the_thirteen() {
        // Σᵢ 2^(L_max − Lᵢ) must equal 2^L_max for a complete code.
        let lmax = HCOD5_MAX_LEN;
        let mut sum: u64 = 0;
        for &(len, _) in &HCOD5 {
            sum += 1u64 << (lmax - u32::from(len));
        }
        assert_eq!(sum, 1u64 << lmax, "Kraft equality failed");
        assert_eq!(sum, 8192);
    }

    #[test]
    fn hcod5_is_complete() {
        // Walk every 13-bit prefix, decode it via the production
        // decoder, and confirm every prefix yields exactly one entry.
        // Bonus: confirm the decoded index round-trips back to the
        // same codeword via `hcod5_encode`.
        for prefix in 0u32..(1u32 << HCOD5_MAX_LEN) {
            // Pack `prefix` (13 bits) left-aligned into two bytes:
            // high byte = bits 12..5, low byte = (bits 4..0) << 3.
            let bytes = [(prefix >> 5) as u8, ((prefix & 0x1f) << 3) as u8];
            let mut br = BitReader::new(&bytes);
            let idx = hcod5_decode(&mut br).expect("13-bit prefix must decode");
            let (len, cw) = hcod5_encode(idx).expect("decoded index must round-trip");
            // The decoded codeword should match the leading `len` bits
            // of `prefix`.
            let lead = prefix >> (HCOD5_MAX_LEN - u32::from(len));
            assert_eq!(
                u32::from(cw),
                lead,
                "round-trip prefix={:#06x} idx={} len={} cw={:#x}",
                prefix,
                idx,
                len,
                cw
            );
        }
    }

    #[test]
    fn hcod5_encode_index_40_is_single_zero_bit() {
        // Spec PDF Table 4.A.6 row 40: length 1, codeword 0 — the
        // §4.6.3.3 zero-tuple `(0, 0)`.
        let (len, cw) = hcod5_encode(40).unwrap();
        assert_eq!(len, 1);
        assert_eq!(cw, 0);
    }

    #[test]
    fn hcod5_encode_first_entry_matches_table() {
        // Spec PDF Table 4.A.6 row 0: length 13, codeword 0x1fff —
        // the lower-left corner `(-4, -4)` of the signed pair lattice.
        let (len, cw) = hcod5_encode(0).unwrap();
        assert_eq!(len, 13);
        assert_eq!(cw, 0x1fff);
    }

    #[test]
    fn hcod5_encode_last_entry_matches_table() {
        // Spec PDF Table 4.A.6 row 80: length 13, codeword 0x1ffe —
        // the upper-right corner `(+4, +4)` of the signed pair lattice.
        let (len, cw) = hcod5_encode(80).unwrap();
        assert_eq!(len, 13);
        assert_eq!(cw, 0x1ffe);
    }

    #[test]
    fn hcod5_encode_four_13_bit_rows_are_the_lattice_corners() {
        // The four 13-bit codewords sit at indices 0, 8, 72, 80 — the
        // four `(±4, ±4)` corners of the signed `9 × 9` pair lattice.
        let expected = [
            (0u32, 0x1fffu16),  // (-4, -4)
            (8u32, 0x1ffdu16),  // (-4, +4)
            (72u32, 0x1ffcu16), // (+4, -4)
            (80u32, 0x1ffeu16), // (+4, +4)
        ];
        let observed: Vec<_> = HCOD5
            .iter()
            .enumerate()
            .filter_map(|(i, &(l, cw))| if l == 13 { Some((i as u32, cw)) } else { None })
            .collect();
        assert_eq!(observed.len(), 4);
        for (e, o) in expected.iter().zip(observed.iter()) {
            assert_eq!(*e, *o, "expected {:?} got {:?}", e, o);
        }
    }

    #[test]
    fn hcod5_encode_rejects_out_of_range_index() {
        assert!(matches!(
            hcod5_encode(81),
            Err(Error::SpectralCodebookIndexOutOfRange(5))
        ));
        assert!(matches!(
            hcod5_encode(0xffff_ffff),
            Err(Error::SpectralCodebookIndexOutOfRange(5))
        ));
    }

    #[test]
    fn hcod5_decode_single_zero_bit_yields_index_40() {
        // Leading bit `0` → idx 40 (the zero-tuple `(0, 0)`).
        // Remaining 7 bits of the byte untouched.
        let bytes = [0b0111_1111u8];
        let mut br = BitReader::new(&bytes);
        let idx = hcod5_decode(&mut br).unwrap();
        assert_eq!(idx, 40);
        assert_eq!(br.bit_position(), 1);
    }

    #[test]
    fn hcod5_decode_full_13_bit_codeword_round_trips_index_0() {
        // Index 0 → length 13, codeword 0x1fff = 0b1_1111_1111_1111.
        // Pack left-aligned into 2 bytes: 0xff, 0xf8.
        let bytes = [0xff, 0xf8];
        let mut br = BitReader::new(&bytes);
        let idx = hcod5_decode(&mut br).unwrap();
        assert_eq!(idx, 0);
        assert_eq!(br.bit_position(), 13);
    }

    #[test]
    fn hcod5_decode_propagates_unexpected_end() {
        let bytes: [u8; 0] = [];
        let mut br = BitReader::new(&bytes);
        assert_eq!(hcod5_decode(&mut br), Err(Error::UnexpectedEnd));
    }

    #[test]
    fn hcod5_write_then_decode_round_trips_every_index() {
        for idx in 0..HCOD5_NUM_ENTRIES as u32 {
            let mut w = BitWriter::new();
            hcod5_write(&mut w, idx).unwrap();
            let (len, _) = hcod5_encode(idx).unwrap();
            let mut w2 = w;
            let pad = (8 - (u32::from(len) % 8)) % 8;
            if pad > 0 {
                w2.write_u32(0, pad);
            }
            let bytes = w2.into_bytes();
            let mut br = BitReader::new(&bytes);
            let decoded = hcod5_decode(&mut br).unwrap();
            assert_eq!(
                decoded, idx,
                "round-trip mismatch at idx={} (encoded as {} bits)",
                idx, len
            );
        }
    }

    #[test]
    fn hcod5_write_rejects_out_of_range_index() {
        let mut w = BitWriter::new();
        assert!(matches!(
            hcod5_write(&mut w, 81),
            Err(Error::SpectralCodebookIndexOutOfRange(5))
        ));
    }

    // -------------------------------------------------------------------
    // Codebook 6 (Table 4.A.7) — signed dim-2 LAV-4 pair book
    // -------------------------------------------------------------------

    #[test]
    fn hcod6_has_exactly_81_entries() {
        // 9^2 = 81 (signed LAV=4 → mod = 2*4+1 = 9, dim = 2) — same
        // tuple universe as Codebook 5.
        assert_eq!(HCOD6.len(), HCOD6_NUM_ENTRIES);
        assert_eq!(HCOD6_NUM_ENTRIES, 81);
    }

    #[test]
    fn hcod6_max_length_is_11_bits() {
        let max = HCOD6.iter().map(|&(len, _)| len).max().unwrap();
        assert_eq!(u32::from(max), HCOD6_MAX_LEN);
        assert_eq!(HCOD6_MAX_LEN, 11);
    }

    #[test]
    fn hcod6_min_length_is_four_bits_at_index_40() {
        // The shortest codeword in Codebook 6 is 4 bits, parked at
        // index 40 (the §4.6.3.3 zero-tuple `(0, 0)` for a signed
        // pair book with LAV=4) with the all-zero pattern `0b0000`.
        // Every other index has length >= 4 (Codebook 6's
        // distribution has a dense 4-bit head: indices 30, 31, 32,
        // 39, 40, 41, 48, 49, 50 all share length 4).
        let (len_40, cw_40) = (HCOD6[40].0, HCOD6[40].1);
        assert_eq!(len_40, 4, "index 40 must be 4-bit");
        assert_eq!(cw_40, 0, "index 40 codeword must be `0b0000`");
        for (idx, &(len, _)) in HCOD6.iter().enumerate() {
            assert!(
                len >= 4,
                "every index must have length >= 4; idx={} len={}",
                idx,
                len
            );
        }
    }

    #[test]
    fn hcod6_codewords_fit_their_declared_length() {
        for (idx, &(len, cw)) in HCOD6.iter().enumerate() {
            let max = if len == 0 { 0 } else { (1u32 << len) - 1 };
            assert!(
                u32::from(cw) <= max,
                "idx={}: codeword {:#x} does not fit {} bits",
                idx,
                cw,
                len
            );
        }
    }

    #[test]
    fn hcod6_kraft_sum_is_two_to_the_eleven() {
        // Σᵢ 2^(L_max − Lᵢ) must equal 2^L_max for a complete code.
        let lmax = HCOD6_MAX_LEN;
        let mut sum: u64 = 0;
        for &(len, _) in &HCOD6 {
            sum += 1u64 << (lmax - u32::from(len));
        }
        assert_eq!(sum, 1u64 << lmax, "Kraft equality failed");
        assert_eq!(sum, 2048);
    }

    #[test]
    fn hcod6_is_complete() {
        // Walk every 11-bit prefix, decode it via the production
        // decoder, and confirm every prefix yields exactly one entry.
        // Bonus: confirm the decoded index round-trips back to the
        // same codeword via `hcod6_encode`.
        for prefix in 0u32..(1u32 << HCOD6_MAX_LEN) {
            // Pack `prefix` (11 bits) left-aligned into two bytes:
            // high byte = bits 10..3, low byte = (bits 2..0) << 5.
            let bytes = [(prefix >> 3) as u8, ((prefix & 0x7) << 5) as u8];
            let mut br = BitReader::new(&bytes);
            let idx = hcod6_decode(&mut br).expect("11-bit prefix must decode");
            let (len, cw) = hcod6_encode(idx).expect("decoded index must round-trip");
            // The decoded codeword should match the leading `len` bits
            // of `prefix`.
            let lead = prefix >> (HCOD6_MAX_LEN - u32::from(len));
            assert_eq!(
                u32::from(cw),
                lead,
                "round-trip prefix={:#05x} idx={} len={} cw={:#x}",
                prefix,
                idx,
                len,
                cw
            );
        }
    }

    #[test]
    fn hcod6_encode_index_40_is_4_bit_zero_codeword() {
        // Spec PDF Table 4.A.7 row 40: length 4, codeword 0 — the
        // §4.6.3.3 zero-tuple `(0, 0)`.
        let (len, cw) = hcod6_encode(40).unwrap();
        assert_eq!(len, 4);
        assert_eq!(cw, 0);
    }

    #[test]
    fn hcod6_encode_first_entry_matches_table() {
        // Spec PDF Table 4.A.7 row 0: length 11, codeword 0x7fe —
        // the lower-left corner `(-4, -4)` of the signed pair lattice.
        let (len, cw) = hcod6_encode(0).unwrap();
        assert_eq!(len, 11);
        assert_eq!(cw, 0x7fe);
    }

    #[test]
    fn hcod6_encode_last_entry_matches_table() {
        // Spec PDF Table 4.A.7 row 80: length 11, codeword 0x7fc —
        // the upper-right corner `(+4, +4)` of the signed pair lattice.
        let (len, cw) = hcod6_encode(80).unwrap();
        assert_eq!(len, 11);
        assert_eq!(cw, 0x7fc);
    }

    #[test]
    fn hcod6_encode_four_11_bit_rows_are_the_lattice_corners() {
        // The four 11-bit codewords sit at indices 0, 8, 72, 80 — the
        // four `(±4, ±4)` corners of the signed `9 × 9` pair lattice.
        let expected = [
            (0u32, 0x7feu16),  // (-4, -4)
            (8u32, 0x7fdu16),  // (-4, +4)
            (72u32, 0x7ffu16), // (+4, -4)
            (80u32, 0x7fcu16), // (+4, +4)
        ];
        let observed: Vec<_> = HCOD6
            .iter()
            .enumerate()
            .filter_map(|(i, &(l, cw))| if l == 11 { Some((i as u32, cw)) } else { None })
            .collect();
        assert_eq!(observed.len(), 4);
        for (e, o) in expected.iter().zip(observed.iter()) {
            assert_eq!(*e, *o, "expected {:?} got {:?}", e, o);
        }
    }

    #[test]
    fn hcod6_encode_rejects_out_of_range_index() {
        assert!(matches!(
            hcod6_encode(81),
            Err(Error::SpectralCodebookIndexOutOfRange(6))
        ));
        assert!(matches!(
            hcod6_encode(0xffff_ffff),
            Err(Error::SpectralCodebookIndexOutOfRange(6))
        ));
    }

    #[test]
    fn hcod6_decode_four_zero_bits_yields_index_40() {
        // Leading `0b0000` → idx 40 (the zero-tuple `(0, 0)`).
        // Remaining 4 bits of the byte untouched.
        let bytes = [0b0000_1111u8];
        let mut br = BitReader::new(&bytes);
        let idx = hcod6_decode(&mut br).unwrap();
        assert_eq!(idx, 40);
        assert_eq!(br.bit_position(), 4);
    }

    #[test]
    fn hcod6_decode_full_11_bit_codeword_round_trips_index_72() {
        // Index 72 → length 11, codeword 0x7ff = 0b111_1111_1111.
        // Pack left-aligned into 2 bytes: 0xff, 0xe0.
        let bytes = [0xff, 0xe0];
        let mut br = BitReader::new(&bytes);
        let idx = hcod6_decode(&mut br).unwrap();
        assert_eq!(idx, 72);
        assert_eq!(br.bit_position(), 11);
    }

    #[test]
    fn hcod6_decode_propagates_unexpected_end() {
        let bytes: [u8; 0] = [];
        let mut br = BitReader::new(&bytes);
        assert_eq!(hcod6_decode(&mut br), Err(Error::UnexpectedEnd));
    }

    #[test]
    fn hcod6_write_then_decode_round_trips_every_index() {
        for idx in 0..HCOD6_NUM_ENTRIES as u32 {
            let mut w = BitWriter::new();
            hcod6_write(&mut w, idx).unwrap();
            let (len, _) = hcod6_encode(idx).unwrap();
            let mut w2 = w;
            let pad = (8 - (u32::from(len) % 8)) % 8;
            if pad > 0 {
                w2.write_u32(0, pad);
            }
            let bytes = w2.into_bytes();
            let mut br = BitReader::new(&bytes);
            let decoded = hcod6_decode(&mut br).unwrap();
            assert_eq!(
                decoded, idx,
                "round-trip mismatch at idx={} (encoded as {} bits)",
                idx, len
            );
        }
    }

    #[test]
    fn hcod6_write_rejects_out_of_range_index() {
        let mut w = BitWriter::new();
        assert!(matches!(
            hcod6_write(&mut w, 81),
            Err(Error::SpectralCodebookIndexOutOfRange(6))
        ));
    }

    // -------------------------------------------------------------------
    // Cross-check: Codebooks 5 and 6 share the signed pair tuple
    // universe (Table 4.95 rows 5 and 6 are identical except for the
    // `Codebook listed in Table` column) but assign different codewords
    // for the same tuple — Codebook 5 gives the zero-tuple the single-
    // bit codeword `0`; Codebook 6 lifts it to a 4-bit `0b0000` and
    // pulls the ceiling back from 13 down to 11 bits.
    // -------------------------------------------------------------------

    #[test]
    fn codebook_5_and_6_disagree_on_zero_tuple_codeword() {
        let (l5, cw5) = hcod5_encode(40).unwrap();
        let (l6, cw6) = hcod6_encode(40).unwrap();
        assert_eq!((l5, cw5), (1, 0));
        assert_eq!((l6, cw6), (4, 0));
    }

    #[test]
    fn codebook_5_and_6_agree_on_lattice_corner_indices() {
        // Both books pin the four (±4, ±4) lattice corners to their
        // respective maximum-length codewords — Codebook 5 at 13 bits,
        // Codebook 6 at 11 bits — but at the same four index positions.
        let corners: Vec<usize> = [0, 8, 72, 80].to_vec();
        let cb5_max_idx: Vec<usize> = HCOD5
            .iter()
            .enumerate()
            .filter_map(|(i, &(l, _))| {
                if u32::from(l) == HCOD5_MAX_LEN {
                    Some(i)
                } else {
                    None
                }
            })
            .collect();
        let cb6_max_idx: Vec<usize> = HCOD6
            .iter()
            .enumerate()
            .filter_map(|(i, &(l, _))| {
                if u32::from(l) == HCOD6_MAX_LEN {
                    Some(i)
                } else {
                    None
                }
            })
            .collect();
        assert_eq!(cb5_max_idx, corners);
        assert_eq!(cb6_max_idx, corners);
    }
}

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
//! 226 adds the second — **Table 4.A.3, "Spectrum Huffman Codebook
//! 2"**. Both books share the same Table 4.95 row shape (`signed`,
//! `dim = 4`, `LAV = 1` → `3^4 = 81` entries indexed `0..=80`); only
//! the per-row Huffman lengths differ to reflect the encoder's choice
//! between two normative spectrum-statistics tunings of the same
//! tuple universe. Codebooks 3..=11 (Tables 4.A.4 … 4.A.12) reuse the
//! same module shape and will be added one per future round; the
//! traits and dispatch surface here are intentionally codebook-agnostic
//! so the per-codebook additions land as a single static-table
//! transcription each.
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
//! ## What this module does *not* cover
//!
//! * Codebooks 3..=11 (Tables 4.A.4 … 4.A.12). Each will land as a
//!   separate dense static table plus reuse of the same encode /
//!   decode shape exposed here.
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
}

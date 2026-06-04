//! Integration tests for `oxideav_aac::spectrum_huffman`.
//!
//! Cross-checks Codebooks 1 (Table 4.A.2), 2 (Table 4.A.3), 3
//! (Table 4.A.4), and 4 (Table 4.A.5) wire round-trip against the
//! §4.6.3.3 index ↔ n-tuple translation already covered by
//! [`oxideav_aac::spectral_codebook`].

use oxideav_aac::{
    spectral_codebook::{
        decode_index_to_tuple, derive_sign_bits, encode_tuple_to_index, table_4_95,
    },
    spectrum_huffman::{
        hcod1_decode, hcod1_encode, hcod1_write, hcod2_decode, hcod2_encode, hcod2_write,
        hcod3_decode, hcod3_encode, hcod3_write, hcod4_decode, hcod4_encode, hcod4_write,
        HCOD1_MAX_LEN, HCOD1_NUM_ENTRIES, HCOD2_MAX_LEN, HCOD2_NUM_ENTRIES, HCOD3_MAX_LEN,
        HCOD3_NUM_ENTRIES, HCOD4_MAX_LEN, HCOD4_NUM_ENTRIES,
    },
    Error,
};
use oxideav_core::bits::{BitReader, BitWriter};

// =============================================================================
// Per-row spec-PDF spot checks (Table 4.A.2)
// =============================================================================
//
// Each spot row is taken from ISO/IEC 14496-3:2001(E) §4.A.1 Table
// 4.A.2 (page 193) verbatim. The full 81-row table-shape and Kraft
// completeness is covered by the unit-tests; this module locks the
// per-row hexadecimal column at integration-test scope so a future
// rebuild against the spec PDF immediately catches any mismatch.

#[test]
fn table_4_a_2_row_0_matches_spec() {
    // Row 0: length 11, codeword 0x7f8.
    let (len, cw) = hcod1_encode(0).unwrap();
    assert_eq!((len, cw), (11, 0x7f8));
}

#[test]
fn table_4_a_2_row_40_matches_spec() {
    // Row 40 (zero-tuple): length 1, codeword 0.
    let (len, cw) = hcod1_encode(40).unwrap();
    assert_eq!((len, cw), (1, 0));
}

#[test]
fn table_4_a_2_row_67_matches_spec() {
    // Row 67: length 5, codeword 0x10.
    let (len, cw) = hcod1_encode(67).unwrap();
    assert_eq!((len, cw), (5, 0x10));
}

#[test]
fn table_4_a_2_row_31_matches_spec() {
    // Row 31: length 5, codeword 0x17. (The largest 5-bit codeword.)
    let (len, cw) = hcod1_encode(31).unwrap();
    assert_eq!((len, cw), (5, 0x17));
}

#[test]
fn table_4_a_2_row_80_matches_spec() {
    // Row 80 (last): length 11, codeword 0x7f4.
    let (len, cw) = hcod1_encode(80).unwrap();
    assert_eq!((len, cw), (11, 0x7f4));
}

#[test]
fn table_4_a_2_row_54_matches_spec() {
    // Row 54: length 11, codeword 0x7fe. Spot-check from the
    // middle of the table to catch a column-swap regression.
    let (len, cw) = hcod1_encode(54).unwrap();
    assert_eq!((len, cw), (11, 0x7fe));
}

// =============================================================================
// Codebook-shape cross-check vs Table 4.95
// =============================================================================

#[test]
fn codebook_1_table_4_95_row_matches_table_4_a_2_size() {
    // Row 1 of Table 4.95: signed, dim=4, lav=1 → 3^4 = 81 entries.
    let row = table_4_95(1).unwrap();
    assert_eq!(row.unsigned, Some(false));
    assert_eq!(row.dimension, Some(4));
    assert_eq!(row.lav, Some(1));
    assert_eq!(row.huffman_table, Some(2));

    let lav = row.lav.unwrap();
    let dim = row.dimension.unwrap();
    let modulus = 2 * lav + 1; // signed
    let count: u32 = modulus.pow(u32::from(dim));
    assert_eq!(count as usize, HCOD1_NUM_ENTRIES);
}

// =============================================================================
// Round-trip every entry through the wire writer + reader
// =============================================================================

#[test]
fn every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD1_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod1_write(&mut w, idx).unwrap();
        // Pad to a byte boundary so BitReader has a full byte to read.
        let bits_written = w.bit_position();
        let pad = (8 - (bits_written % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod1_decode(&mut br).unwrap();
        assert_eq!(
            got, idx,
            "round-trip mismatch at idx={} ({} bits written)",
            idx, bits_written
        );
    }
}

// =============================================================================
// Cross-check: decoded index → §4.6.3.3 tuple → re-encoded index
// =============================================================================

#[test]
fn every_index_translates_through_spectral_codebook_round_trip() {
    // For each index 0..=80, run the §4.6.3.3 decode → tuple → encode
    // path. The dim-4 signed range is `-1..=+1` per Table 4.95, so
    // every codebook-1 index has a legal tuple representation.
    for idx in 0..HCOD1_NUM_ENTRIES as u32 {
        let tup = decode_index_to_tuple(1, idx).expect("legal codebook-1 index");
        // §4.6.3.3: every tuple element must be in `-1..=+1`.
        for (k, &v) in tup.iter().take(4).enumerate() {
            assert!(
                (-1..=1).contains(&v),
                "idx={} k={} tuple value {} outside ±LAV (1)",
                idx,
                k,
                v
            );
        }
        let round = encode_tuple_to_index(1, &tup[..4]).expect("tuple round-trip");
        assert_eq!(round, idx, "index round-trip failed for idx={}", idx);
    }
}

// =============================================================================
// Wire-bit precision: pin a few hand-built byte sequences
// =============================================================================

#[test]
fn write_index_40_then_align_yields_zero_byte() {
    // Index 40 is the single bit `0`. Pad 7 zero bits → byte 0x00.
    let mut w = BitWriter::new();
    hcod1_write(&mut w, 40).unwrap();
    w.write_u32(0, 7);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x00]);
}

#[test]
fn write_index_0_yields_11_bit_codeword_0x7f8() {
    // Codeword 0x7f8 = 0b111_1111_1000, MSB-first.
    let mut w = BitWriter::new();
    hcod1_write(&mut w, 0).unwrap();
    // Pad 5 zero bits → 16-bit boundary.
    w.write_u32(0, 5);
    let bytes = w.into_bytes();
    // 0b1111_1111_0000_0000 = 0xFF, 0x00.
    assert_eq!(bytes, vec![0xff, 0x00]);
}

#[test]
fn write_two_indices_yields_concatenated_bitstream() {
    // Index 40 (1 bit `0`) followed by index 0 (11 bits `0x7f8`)
    // packs to: 0_111_1111_1000 = 12 bits. Pad 4 zero bits to a
    // 16-bit boundary: 0111_1111_1000_0000 = 0x7f, 0x80.
    let mut w = BitWriter::new();
    hcod1_write(&mut w, 40).unwrap();
    hcod1_write(&mut w, 0).unwrap();
    w.write_u32(0, 4);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x7f, 0x80]);

    // Inverse round-trip: read back both indices.
    let mut br = BitReader::new(&bytes);
    assert_eq!(hcod1_decode(&mut br).unwrap(), 40);
    assert_eq!(hcod1_decode(&mut br).unwrap(), 0);
}

// =============================================================================
// Decoder reads only as many bits as the codeword needs
// =============================================================================

#[test]
fn decoder_consumes_exactly_codeword_length_bits() {
    for idx in 0..HCOD1_NUM_ENTRIES as u32 {
        let (len, _) = hcod1_encode(idx).unwrap();
        let mut w = BitWriter::new();
        hcod1_write(&mut w, idx).unwrap();
        // Pad to a byte boundary so BitReader has a full byte to read.
        let pad = (8 - (w.bit_position() % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let _ = hcod1_decode(&mut br).unwrap();
        assert_eq!(
            br.bit_position(),
            u64::from(len),
            "idx={}: decoder consumed {} bits, expected {}",
            idx,
            br.bit_position(),
            len
        );
    }
}

// =============================================================================
// Rejection branches
// =============================================================================

#[test]
fn encode_rejects_indices_at_and_above_81() {
    for bad in [81u32, 82, 100, 1000, u32::MAX] {
        assert!(matches!(
            hcod1_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(1))
        ));
    }
}

#[test]
fn decoder_returns_unexpected_end_on_truncation() {
    // Index 0 needs 11 bits. Provide only 1 byte (8 bits) → underflow.
    let bytes = [0xffu8];
    let mut br = BitReader::new(&bytes);
    // We've read 8 ones; the next bit_read should fail.
    let err = hcod1_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

// =============================================================================
// HCOD1_MAX_LEN constant is the actual max
// =============================================================================

#[test]
fn hcod1_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD1_NUM_ENTRIES as u32 {
        let (len, _) = hcod1_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD1_MAX_LEN);
}

// =============================================================================
// Codebook 2 — Table 4.A.3 per-row spec-PDF spot checks
// =============================================================================
//
// Each spot row is taken from ISO/IEC 14496-3:2001(E) §4.A.1 Table
// 4.A.3 (page 194) verbatim. The full 81-row table-shape and Kraft
// completeness are covered by the unit tests; this module locks the
// per-row hexadecimal column at integration-test scope.

#[test]
fn table_4_a_3_row_0_matches_spec() {
    // Row 0: length 9, codeword 0x1f3.
    let (len, cw) = hcod2_encode(0).unwrap();
    assert_eq!((len, cw), (9, 0x1f3));
}

#[test]
fn table_4_a_3_row_13_matches_spec() {
    // Row 13: length 5, codeword 0x06 — a low-frequency 5-bit slot.
    let (len, cw) = hcod2_encode(13).unwrap();
    assert_eq!((len, cw), (5, 0x06));
}

#[test]
fn table_4_a_3_row_40_matches_spec() {
    // Row 40 (zero-tuple): length 3, codeword 0.
    let (len, cw) = hcod2_encode(40).unwrap();
    assert_eq!((len, cw), (3, 0));
}

#[test]
fn table_4_a_3_row_67_matches_spec() {
    // Row 67: length 4, codeword 0x02 — the shortest non-zero-tuple
    // codeword in the book.
    let (len, cw) = hcod2_encode(67).unwrap();
    assert_eq!((len, cw), (4, 0x02));
}

#[test]
fn table_4_a_3_row_78_matches_spec() {
    // Row 78: length 9, codeword 0x1ff — the largest 9-bit codeword.
    let (len, cw) = hcod2_encode(78).unwrap();
    assert_eq!((len, cw), (9, 0x1ff));
}

#[test]
fn table_4_a_3_row_80_matches_spec() {
    // Row 80 (last): length 9, codeword 0x1f6.
    let (len, cw) = hcod2_encode(80).unwrap();
    assert_eq!((len, cw), (9, 0x1f6));
}

// =============================================================================
// Codebook-shape cross-check vs Table 4.95 row 2
// =============================================================================

#[test]
fn codebook_2_table_4_95_row_matches_table_4_a_3_size() {
    // Row 2 of Table 4.95: signed, dim=4, lav=1 → 3^4 = 81 entries.
    let row = table_4_95(2).unwrap();
    assert_eq!(row.unsigned, Some(false));
    assert_eq!(row.dimension, Some(4));
    assert_eq!(row.lav, Some(1));
    assert_eq!(row.huffman_table, Some(3));

    let lav = row.lav.unwrap();
    let dim = row.dimension.unwrap();
    let modulus = 2 * lav + 1; // signed
    let count: u32 = modulus.pow(u32::from(dim));
    assert_eq!(count as usize, HCOD2_NUM_ENTRIES);
}

// =============================================================================
// Round-trip every entry through the Codebook 2 wire writer + reader
// =============================================================================

#[test]
fn hcod2_every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD2_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod2_write(&mut w, idx).unwrap();
        let bits_written = w.bit_position();
        let pad = (8 - (bits_written % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod2_decode(&mut br).unwrap();
        assert_eq!(
            got, idx,
            "round-trip mismatch at idx={} ({} bits written)",
            idx, bits_written
        );
    }
}

// =============================================================================
// Cross-check: decoded index → §4.6.3.3 tuple → re-encoded index
// (Codebook 2 shares Codebook 1's tuple universe)
// =============================================================================

#[test]
fn hcod2_every_index_translates_through_spectral_codebook_round_trip() {
    // For each index 0..=80, run the §4.6.3.3 decode → tuple → encode
    // path against codebook 2. The dim-4 signed range is `-1..=+1`
    // per Table 4.95 row 2, so every index has a legal tuple
    // representation. The translation layer in `spectral_codebook`
    // does not depend on which Huffman table assigned the index, so
    // the same tuple universe is shared with codebook 1.
    for idx in 0..HCOD2_NUM_ENTRIES as u32 {
        let tup = decode_index_to_tuple(2, idx).expect("legal codebook-2 index");
        for (k, &v) in tup.iter().take(4).enumerate() {
            assert!(
                (-1..=1).contains(&v),
                "idx={} k={} tuple value {} outside ±LAV (1)",
                idx,
                k,
                v
            );
        }
        let round = encode_tuple_to_index(2, &tup[..4]).expect("tuple round-trip");
        assert_eq!(round, idx, "index round-trip failed for idx={}", idx);
    }
}

#[test]
fn hcod1_and_hcod2_share_the_same_tuple_for_each_index() {
    // The §4.6.3.3 translation is codebook-shape driven, not codeword
    // driven. Codebooks 1 and 2 have the same row in Table 4.95
    // (signed dim=4 LAV=1), so for every shared `idx` they map to
    // the same `(w, x, y, z)` tuple. The Huffman codewords differ
    // but the spectral content is identical.
    for idx in 0..HCOD1_NUM_ENTRIES as u32 {
        let t1 = decode_index_to_tuple(1, idx).unwrap();
        let t2 = decode_index_to_tuple(2, idx).unwrap();
        assert_eq!(t1, t2, "tuple disagreement at idx={}", idx);
    }
}

// =============================================================================
// Wire-bit precision: hand-built byte sequences for Codebook 2
// =============================================================================

#[test]
fn hcod2_write_index_40_then_align_yields_zero_byte() {
    // Index 40 is 3 bits `000`. Pad 5 zero bits → byte 0x00.
    let mut w = BitWriter::new();
    hcod2_write(&mut w, 40).unwrap();
    w.write_u32(0, 5);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x00]);
}

#[test]
fn hcod2_write_index_0_yields_9_bit_codeword_0x1f3() {
    // Codeword 0x1f3 = 0b1_1111_0011, MSB-first.
    let mut w = BitWriter::new();
    hcod2_write(&mut w, 0).unwrap();
    // Pad 7 zero bits → 16-bit boundary.
    w.write_u32(0, 7);
    let bytes = w.into_bytes();
    // 0b1111_1001_1000_0000 = 0xf9, 0x80.
    assert_eq!(bytes, vec![0xf9, 0x80]);
}

#[test]
fn hcod2_write_two_indices_yields_concatenated_bitstream() {
    // Index 40 (3 bits `000`) followed by index 0 (9 bits `0x1f3`)
    // packs to: 000_1_1111_0011 = 12 bits. Pad 4 zero bits to a
    // 16-bit boundary: 0001_1111_0011_0000 = 0x1f, 0x30.
    let mut w = BitWriter::new();
    hcod2_write(&mut w, 40).unwrap();
    hcod2_write(&mut w, 0).unwrap();
    w.write_u32(0, 4);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x1f, 0x30]);

    let mut br = BitReader::new(&bytes);
    assert_eq!(hcod2_decode(&mut br).unwrap(), 40);
    assert_eq!(hcod2_decode(&mut br).unwrap(), 0);
}

// =============================================================================
// Decoder reads only as many bits as the codeword needs
// =============================================================================

#[test]
fn hcod2_decoder_consumes_exactly_codeword_length_bits() {
    for idx in 0..HCOD2_NUM_ENTRIES as u32 {
        let (len, _) = hcod2_encode(idx).unwrap();
        let mut w = BitWriter::new();
        hcod2_write(&mut w, idx).unwrap();
        let pad = (8 - (w.bit_position() % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let _ = hcod2_decode(&mut br).unwrap();
        assert_eq!(
            br.bit_position(),
            u64::from(len),
            "idx={}: decoder consumed {} bits, expected {}",
            idx,
            br.bit_position(),
            len
        );
    }
}

// =============================================================================
// Rejection branches
// =============================================================================

#[test]
fn hcod2_encode_rejects_indices_at_and_above_81() {
    for bad in [81u32, 82, 100, 1000, u32::MAX] {
        assert!(matches!(
            hcod2_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(2))
        ));
    }
}

#[test]
fn hcod2_decoder_returns_unexpected_end_on_truncation() {
    // Codebook 2 needs at least 3 bits for the shortest codeword
    // (index 40). Provide zero bytes → underflow immediately.
    let bytes: [u8; 0] = [];
    let mut br = BitReader::new(&bytes);
    let err = hcod2_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

#[test]
fn hcod2_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD2_NUM_ENTRIES as u32 {
        let (len, _) = hcod2_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD2_MAX_LEN);
}

// =============================================================================
// Codebook 3 — Table 4.A.4 per-row spec-PDF spot checks
// =============================================================================
//
// Each spot row is taken from ISO/IEC 14496-3:2009(E) §4.A.1 Table
// 4.A.4 verbatim. The full 81-row table-shape and Kraft completeness
// are covered by the unit tests; this module locks the per-row
// hexadecimal column at integration-test scope.

#[test]
fn table_4_a_4_row_0_matches_spec() {
    // Row 0 (the unsigned book's zero-tuple): length 1, codeword 0.
    let (len, cw) = hcod3_encode(0).unwrap();
    assert_eq!((len, cw), (1, 0));
}

#[test]
fn table_4_a_4_row_1_matches_spec() {
    // Row 1: length 4, codeword 0x9.
    let (len, cw) = hcod3_encode(1).unwrap();
    assert_eq!((len, cw), (4, 0x9));
}

#[test]
fn table_4_a_4_row_27_matches_spec() {
    // Row 27: length 4, codeword 0x8 — a low-frequency 4-bit slot.
    let (len, cw) = hcod3_encode(27).unwrap();
    assert_eq!((len, cw), (4, 0x8));
}

#[test]
fn table_4_a_4_row_40_matches_spec() {
    // Row 40: length 7, codeword 0x74 — a mid-table 7-bit slot.
    let (len, cw) = hcod3_encode(40).unwrap();
    assert_eq!((len, cw), (7, 0x74));
}

#[test]
fn table_4_a_4_row_62_matches_spec() {
    // Row 62: length 16, codeword 0xffff — the full 16-bit pattern.
    let (len, cw) = hcod3_encode(62).unwrap();
    assert_eq!((len, cw), (16, 0xffff));
}

#[test]
fn table_4_a_4_row_74_matches_spec() {
    // Row 74: length 16, codeword 0xfffe — the second of the two
    // 16-bit rows.
    let (len, cw) = hcod3_encode(74).unwrap();
    assert_eq!((len, cw), (16, 0xfffe));
}

#[test]
fn table_4_a_4_row_80_matches_spec() {
    // Row 80 (last): length 15, codeword 0x7ffa.
    let (len, cw) = hcod3_encode(80).unwrap();
    assert_eq!((len, cw), (15, 0x7ffa));
}

// =============================================================================
// Codebook-shape cross-check vs Table 4.95 row 3
// =============================================================================

#[test]
fn codebook_3_table_4_95_row_matches_table_4_a_4_size() {
    // Row 3 of Table 4.95: unsigned, dim=4, lav=2 → 3^4 = 81 entries.
    let row = table_4_95(3).unwrap();
    assert_eq!(row.unsigned, Some(true));
    assert_eq!(row.dimension, Some(4));
    assert_eq!(row.lav, Some(2));
    assert_eq!(row.huffman_table, Some(4));

    let lav = row.lav.unwrap();
    let dim = row.dimension.unwrap();
    let modulus = lav + 1; // unsigned
    let count: u32 = modulus.pow(u32::from(dim));
    assert_eq!(count as usize, HCOD3_NUM_ENTRIES);
}

// =============================================================================
// Round-trip every entry through the Codebook 3 wire writer + reader
// =============================================================================

#[test]
fn hcod3_every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD3_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod3_write(&mut w, idx).unwrap();
        let bits_written = w.bit_position();
        let pad = (8 - (bits_written % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod3_decode(&mut br).unwrap();
        assert_eq!(
            got, idx,
            "round-trip mismatch at idx={} ({} bits written)",
            idx, bits_written
        );
    }
}

// =============================================================================
// Cross-check: decoded index → §4.6.3.3 tuple → re-encoded index
// (Codebook 3 is unsigned: every tuple element lies in `0..=LAV = 0..=2`)
// =============================================================================

#[test]
fn hcod3_every_index_translates_through_spectral_codebook_round_trip() {
    // For each index 0..=80, run the §4.6.3.3 decode → tuple → encode
    // path against codebook 3. The dim-4 unsigned range is `0..=2`
    // per Table 4.95 row 3, so every index has a legal tuple
    // representation. The translation layer in `spectral_codebook`
    // dispatches on the row's `unsigned` flag.
    for idx in 0..HCOD3_NUM_ENTRIES as u32 {
        let tup = decode_index_to_tuple(3, idx).expect("legal codebook-3 index");
        for (k, &v) in tup.iter().take(4).enumerate() {
            assert!(
                (0..=2).contains(&v),
                "idx={} k={} tuple value {} outside [0, LAV=2]",
                idx,
                k,
                v
            );
        }
        let round = encode_tuple_to_index(3, &tup[..4]).expect("tuple round-trip");
        assert_eq!(round, idx, "index round-trip failed for idx={}", idx);
    }
}

#[test]
fn hcod3_unsigned_book_puts_zero_tuple_at_index_zero() {
    // The §4.6.3.3 translation polynomial is `((w * mod) + x) * mod
    // + y) * mod + z` with `mod = LAV + 1 = 3` for unsigned books
    // and `offset = 0`. So `(0, 0, 0, 0)` evaluates to index 0.
    // Cross-check against `decode_index_to_tuple`.
    let tup = decode_index_to_tuple(3, 0).unwrap();
    assert_eq!(tup[..4], [0, 0, 0, 0]);
    // And the largest magnitude tuple `(2, 2, 2, 2)` lands at the
    // last index (3^4 - 1 = 80).
    let tup_last = decode_index_to_tuple(3, 80).unwrap();
    assert_eq!(tup_last[..4], [2, 2, 2, 2]);
}

// =============================================================================
// Cross-check: Codebook 3 sign bits — every non-zero coefficient gets
// one sign bit (the codeword itself carries magnitudes only).
// =============================================================================

#[test]
fn hcod3_zero_tuple_emits_zero_sign_bits() {
    // The zero-tuple has no non-zero coefficients → derive_sign_bits
    // returns an empty Vec.
    let tup = decode_index_to_tuple(3, 0).unwrap();
    let signs = derive_sign_bits(3, &tup[..4]).unwrap();
    assert!(signs.is_empty());
}

#[test]
fn hcod3_full_magnitude_tuple_emits_four_sign_bits() {
    // The `(2, 2, 2, 2)` magnitude tuple has all four coefficients
    // non-zero → derive_sign_bits returns exactly four bits.
    let tup = decode_index_to_tuple(3, 80).unwrap();
    let signs = derive_sign_bits(3, &tup[..4]).unwrap();
    assert_eq!(signs.len(), 4);
}

#[test]
fn hcod3_sign_bit_count_equals_nonzero_count_for_every_index() {
    // For every codebook-3 index, the number of §4.6.3.3 sign bits
    // (one per non-zero coefficient) equals the count of non-zero
    // entries in the decoded magnitude tuple.
    for idx in 0..HCOD3_NUM_ENTRIES as u32 {
        let tup = decode_index_to_tuple(3, idx).unwrap();
        let signs = derive_sign_bits(3, &tup[..4]).unwrap();
        let nonzero = tup.iter().take(4).filter(|&&v| v != 0).count();
        assert_eq!(
            signs.len(),
            nonzero,
            "idx={}: sign bits {} != non-zero count {}",
            idx,
            signs.len(),
            nonzero
        );
    }
}

// =============================================================================
// Wire-bit precision: hand-built byte sequences for Codebook 3
// =============================================================================

#[test]
fn hcod3_write_index_0_then_align_yields_zero_byte() {
    // Index 0 is the single bit `0`. Pad 7 zero bits → byte 0x00.
    let mut w = BitWriter::new();
    hcod3_write(&mut w, 0).unwrap();
    w.write_u32(0, 7);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x00]);
}

#[test]
fn hcod3_write_index_62_yields_full_16_bit_codeword_0xffff() {
    // Codeword 0xffff = 0b1111_1111_1111_1111, MSB-first.
    let mut w = BitWriter::new();
    hcod3_write(&mut w, 62).unwrap();
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0xff, 0xff]);
}

#[test]
fn hcod3_write_two_indices_yields_concatenated_bitstream() {
    // Index 0 (1 bit `0`) followed by index 1 (4 bits `0x9 = 0b1001`)
    // packs to: 0_1001 = 5 bits. Pad 3 zero bits to a byte boundary:
    // 0100_1000 = 0x48.
    let mut w = BitWriter::new();
    hcod3_write(&mut w, 0).unwrap();
    hcod3_write(&mut w, 1).unwrap();
    w.write_u32(0, 3);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x48]);

    let mut br = BitReader::new(&bytes);
    assert_eq!(hcod3_decode(&mut br).unwrap(), 0);
    assert_eq!(hcod3_decode(&mut br).unwrap(), 1);
}

// =============================================================================
// Decoder reads only as many bits as the codeword needs
// =============================================================================

#[test]
fn hcod3_decoder_consumes_exactly_codeword_length_bits() {
    for idx in 0..HCOD3_NUM_ENTRIES as u32 {
        let (len, _) = hcod3_encode(idx).unwrap();
        let mut w = BitWriter::new();
        hcod3_write(&mut w, idx).unwrap();
        let pad = (8 - (w.bit_position() % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let _ = hcod3_decode(&mut br).unwrap();
        assert_eq!(
            br.bit_position(),
            u64::from(len),
            "idx={}: decoder consumed {} bits, expected {}",
            idx,
            br.bit_position(),
            len
        );
    }
}

// =============================================================================
// Rejection branches
// =============================================================================

#[test]
fn hcod3_encode_rejects_indices_at_and_above_81() {
    for bad in [81u32, 82, 100, 1000, u32::MAX] {
        assert!(matches!(
            hcod3_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(3))
        ));
    }
}

#[test]
fn hcod3_decoder_returns_unexpected_end_on_truncation() {
    // Index 62 needs a full 16 bits. Provide one byte (8 bits) of
    // all-ones — the loop drives down to the 9th bit-read which
    // underflows.
    let bytes = [0xffu8];
    let mut br = BitReader::new(&bytes);
    let err = hcod3_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

#[test]
fn hcod3_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD3_NUM_ENTRIES as u32 {
        let (len, _) = hcod3_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD3_MAX_LEN);
}

// =============================================================================
// Cross-check: Codebooks 1 / 2 (signed dim=4 LAV=1) and Codebook 3
// (unsigned dim=4 LAV=2) cover *different* tuple universes despite
// the matching cardinality of 81.
// =============================================================================

#[test]
fn codebook_3_has_a_different_tuple_universe_than_codebook_1() {
    // Both books enumerate 81 tuples but the value-ranges differ:
    // Codebook 1 (signed LAV=1) lives in `(-1, 0, +1)^4`;
    // Codebook 3 (unsigned LAV=2) lives in `(0, 1, 2)^4`. Any
    // codebook-1 tuple containing a negative coefficient must NOT
    // round-trip when re-encoded with Codebook 3 — and vice-versa
    // for codebook-3 tuples containing a `2` entry.
    let neg_tup = [-1i32, 0, 0, 0];
    assert!(matches!(
        encode_tuple_to_index(3, &neg_tup),
        Err(Error::SpectralCodebookTupleOutOfRange(3))
    ));
    let big_tup = [2i32, 0, 0, 0];
    assert!(matches!(
        encode_tuple_to_index(1, &big_tup),
        Err(Error::SpectralCodebookTupleOutOfRange(1))
    ));
}

// =============================================================================
// Codebook 4 — Table 4.A.5 per-row spec-PDF spot checks
// =============================================================================
//
// Each spot row is taken from ISO/IEC 14496-3:2001(E) §4.A.1 Table
// 4.A.5 verbatim. The full 81-row table-shape and Kraft completeness
// are covered by the unit tests; this module locks the per-row
// hexadecimal column at integration-test scope.

#[test]
fn table_4_a_5_row_0_matches_spec() {
    // Row 0: length 4, codeword 0x7.
    let (len, cw) = hcod4_encode(0).unwrap();
    assert_eq!((len, cw), (4, 0x7));
}

#[test]
fn table_4_a_5_row_13_matches_spec() {
    // Row 13: length 4, codeword 0x1 — a low-frequency 4-bit slot.
    let (len, cw) = hcod4_encode(13).unwrap();
    assert_eq!((len, cw), (4, 0x1));
}

#[test]
fn table_4_a_5_row_27_matches_spec() {
    // Row 27: length 4, codeword 0x5.
    let (len, cw) = hcod4_encode(27).unwrap();
    assert_eq!((len, cw), (4, 0x5));
}

#[test]
fn table_4_a_5_row_40_matches_spec() {
    // Row 40 (shortest codeword): length 4, codeword 0x0 (`0b0000`).
    let (len, cw) = hcod4_encode(40).unwrap();
    assert_eq!((len, cw), (4, 0));
}

#[test]
fn table_4_a_5_row_62_matches_spec() {
    // Row 62: length 12, codeword 0xfff — one of the two 12-bit rows.
    let (len, cw) = hcod4_encode(62).unwrap();
    assert_eq!((len, cw), (12, 0xfff));
}

#[test]
fn table_4_a_5_row_74_matches_spec() {
    // Row 74: length 12, codeword 0xffe — the other 12-bit row.
    let (len, cw) = hcod4_encode(74).unwrap();
    assert_eq!((len, cw), (12, 0xffe));
}

#[test]
fn table_4_a_5_row_80_matches_spec() {
    // Row 80 (last): length 11, codeword 0x7fc.
    let (len, cw) = hcod4_encode(80).unwrap();
    assert_eq!((len, cw), (11, 0x7fc));
}

// =============================================================================
// Codebook-shape cross-check vs Table 4.95 row 4
// =============================================================================

#[test]
fn codebook_4_table_4_95_row_matches_table_4_a_5_size() {
    // Row 4 of Table 4.95: unsigned, dim=4, lav=2 → 3^4 = 81 entries.
    let row = table_4_95(4).unwrap();
    assert_eq!(row.unsigned, Some(true));
    assert_eq!(row.dimension, Some(4));
    assert_eq!(row.lav, Some(2));
    assert_eq!(row.huffman_table, Some(5));

    let lav = row.lav.unwrap();
    let dim = row.dimension.unwrap();
    let modulus = lav + 1; // unsigned
    let count: u32 = modulus.pow(u32::from(dim));
    assert_eq!(count as usize, HCOD4_NUM_ENTRIES);
}

// =============================================================================
// Round-trip every entry through the Codebook 4 wire writer + reader
// =============================================================================

#[test]
fn hcod4_every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD4_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod4_write(&mut w, idx).unwrap();
        let bits_written = w.bit_position();
        let pad = (8 - (bits_written % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod4_decode(&mut br).unwrap();
        assert_eq!(
            got, idx,
            "round-trip mismatch at idx={} ({} bits written)",
            idx, bits_written
        );
    }
}

// =============================================================================
// Cross-check: decoded index → §4.6.3.3 tuple → re-encoded index
// (Codebook 4 is unsigned dim-4: every tuple element lies in `0..=LAV
// = 0..=2`). Codebook 4 shares Codebook 3's Table 4.95 row shape, so
// the §4.6.3.3 translation maps every index to the same tuple in both
// books — only the codeword assignment differs.
// =============================================================================

#[test]
fn hcod4_every_index_translates_through_spectral_codebook_round_trip() {
    for idx in 0..HCOD4_NUM_ENTRIES as u32 {
        let tup = decode_index_to_tuple(4, idx).expect("legal codebook-4 index");
        for (k, &v) in tup.iter().take(4).enumerate() {
            assert!(
                (0..=2).contains(&v),
                "idx={} k={} tuple value {} outside [0, LAV=2]",
                idx,
                k,
                v
            );
        }
        let round = encode_tuple_to_index(4, &tup[..4]).expect("tuple round-trip");
        assert_eq!(round, idx, "index round-trip failed for idx={}", idx);
    }
}

#[test]
fn hcod3_and_hcod4_map_every_index_to_the_same_tuple() {
    // Both books share Table 4.95's unsigned dim-4 LAV-2 row shape,
    // so the §4.6.3.3 translation is identical even though the
    // Huffman codeword assignments differ.
    for idx in 0..HCOD4_NUM_ENTRIES as u32 {
        let t3 = decode_index_to_tuple(3, idx).unwrap();
        let t4 = decode_index_to_tuple(4, idx).unwrap();
        assert_eq!(
            t3[..4],
            t4[..4],
            "Codebook 3 / Codebook 4 disagree at idx={}",
            idx
        );
    }
}

#[test]
fn hcod4_unsigned_book_puts_zero_tuple_at_index_zero() {
    // The §4.6.3.3 translation polynomial puts `(0, 0, 0, 0)` at
    // index 0 for any unsigned book; the magnitude-2 tuple
    // `(2, 2, 2, 2)` lands at index 80 (3^4 - 1).
    let tup = decode_index_to_tuple(4, 0).unwrap();
    assert_eq!(tup[..4], [0, 0, 0, 0]);
    let tup_last = decode_index_to_tuple(4, 80).unwrap();
    assert_eq!(tup_last[..4], [2, 2, 2, 2]);
}

// =============================================================================
// Cross-check: Codebook 4 sign bits — every non-zero coefficient gets
// one sign bit (the codeword itself carries magnitudes only).
// =============================================================================

#[test]
fn hcod4_zero_tuple_emits_zero_sign_bits() {
    let tup = decode_index_to_tuple(4, 0).unwrap();
    let signs = derive_sign_bits(4, &tup[..4]).unwrap();
    assert!(signs.is_empty());
}

#[test]
fn hcod4_full_magnitude_tuple_emits_four_sign_bits() {
    let tup = decode_index_to_tuple(4, 80).unwrap();
    let signs = derive_sign_bits(4, &tup[..4]).unwrap();
    assert_eq!(signs.len(), 4);
}

#[test]
fn hcod4_sign_bit_count_equals_nonzero_count_for_every_index() {
    for idx in 0..HCOD4_NUM_ENTRIES as u32 {
        let tup = decode_index_to_tuple(4, idx).unwrap();
        let signs = derive_sign_bits(4, &tup[..4]).unwrap();
        let nonzero = tup.iter().take(4).filter(|&&v| v != 0).count();
        assert_eq!(
            signs.len(),
            nonzero,
            "idx={}: sign bits {} != non-zero count {}",
            idx,
            signs.len(),
            nonzero
        );
    }
}

// =============================================================================
// Wire-bit precision: hand-built byte sequences for Codebook 4
// =============================================================================

#[test]
fn hcod4_write_index_40_then_align_yields_zero_byte() {
    // Index 40 is 4 bits `0b0000`. Pad 4 zero bits → byte 0x00.
    let mut w = BitWriter::new();
    hcod4_write(&mut w, 40).unwrap();
    w.write_u32(0, 4);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x00]);
}

#[test]
fn hcod4_write_index_62_yields_full_12_bit_codeword_0xfff() {
    // Codeword 0xfff = 0b1111_1111_1111, MSB-first. Pack into 12
    // bits + 4 zero pad bits → 0xff, 0xf0.
    let mut w = BitWriter::new();
    hcod4_write(&mut w, 62).unwrap();
    w.write_u32(0, 4);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0xff, 0xf0]);
}

#[test]
fn hcod4_write_two_indices_yields_concatenated_bitstream() {
    // Index 40 (4 bits `0000`) followed by index 0 (4 bits `0111`)
    // packs to: 0000_0111 = 0x07.
    let mut w = BitWriter::new();
    hcod4_write(&mut w, 40).unwrap();
    hcod4_write(&mut w, 0).unwrap();
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x07]);

    let mut br = BitReader::new(&bytes);
    assert_eq!(hcod4_decode(&mut br).unwrap(), 40);
    assert_eq!(hcod4_decode(&mut br).unwrap(), 0);
}

// =============================================================================
// Decoder reads only as many bits as the codeword needs
// =============================================================================

#[test]
fn hcod4_decoder_consumes_exactly_codeword_length_bits() {
    for idx in 0..HCOD4_NUM_ENTRIES as u32 {
        let (len, _) = hcod4_encode(idx).unwrap();
        let mut w = BitWriter::new();
        hcod4_write(&mut w, idx).unwrap();
        let pad = (8 - (w.bit_position() % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let _ = hcod4_decode(&mut br).unwrap();
        assert_eq!(
            br.bit_position(),
            u64::from(len),
            "idx={}: decoder consumed {} bits, expected {}",
            idx,
            br.bit_position(),
            len
        );
    }
}

// =============================================================================
// Rejection branches
// =============================================================================

#[test]
fn hcod4_encode_rejects_indices_at_and_above_81() {
    for bad in [81u32, 82, 100, 1000, u32::MAX] {
        assert!(matches!(
            hcod4_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(4))
        ));
    }
}

#[test]
fn hcod4_decoder_returns_unexpected_end_on_truncation() {
    // Codebook 4 needs at least 4 bits for the shortest codeword
    // (index 40). Provide zero bytes → immediate underflow.
    let bytes: [u8; 0] = [];
    let mut br = BitReader::new(&bytes);
    let err = hcod4_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

#[test]
fn hcod4_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD4_NUM_ENTRIES as u32 {
        let (len, _) = hcod4_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD4_MAX_LEN);
}

// =============================================================================
// Cross-check: Codebook 3 / Codebook 4 share a tuple universe but use
// different codeword assignments (the four-tuple universe is identical
// per Table 4.95 rows 3 and 4, but Codebook 3 maxes out at 16 bits
// while Codebook 4 maxes out at 12 bits).
// =============================================================================

#[test]
fn codebook_3_and_4_have_the_same_tuple_universe_but_distinct_max_codeword_lengths() {
    assert_eq!(HCOD3_NUM_ENTRIES, HCOD4_NUM_ENTRIES);
    assert_eq!(HCOD3_NUM_ENTRIES, 81);
    assert_ne!(HCOD3_MAX_LEN, HCOD4_MAX_LEN);
    assert_eq!(HCOD3_MAX_LEN, 16);
    assert_eq!(HCOD4_MAX_LEN, 12);
}

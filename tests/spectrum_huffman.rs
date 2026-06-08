//! Integration tests for `oxideav_aac::spectrum_huffman`.
//!
//! Cross-checks Codebooks 1 (Table 4.A.2), 2 (Table 4.A.3), 3
//! (Table 4.A.4), 4 (Table 4.A.5), 5 (Table 4.A.6), 6 (Table 4.A.7),
//! 7 (Table 4.A.8), 8 (Table 4.A.9), 9 (Table 4.A.10), 10
//! (Table 4.A.11), and 11 (Table 4.A.12, the ESC book) wire
//! round-trip against the §4.6.3.3 index ↔ n-tuple translation
//! already covered by [`oxideav_aac::spectral_codebook`].

use oxideav_aac::{
    spectral_codebook::{
        decode_index_to_tuple, derive_sign_bits, encode_tuple_to_index, table_4_95,
    },
    spectrum_huffman::{
        hcod10_decode, hcod10_encode, hcod10_write, hcod11_decode, hcod11_encode, hcod11_write,
        hcod1_decode, hcod1_encode, hcod1_write, hcod2_decode, hcod2_encode, hcod2_write,
        hcod3_decode, hcod3_encode, hcod3_write, hcod4_decode, hcod4_encode, hcod4_write,
        hcod5_decode, hcod5_encode, hcod5_write, hcod6_decode, hcod6_encode, hcod6_write,
        hcod7_decode, hcod7_encode, hcod7_write, hcod8_decode, hcod8_encode, hcod8_write,
        hcod9_decode, hcod9_encode, hcod9_write, HCOD10_MAX_LEN, HCOD10_NUM_ENTRIES,
        HCOD11_MAX_LEN, HCOD11_NUM_ENTRIES, HCOD1_MAX_LEN, HCOD1_NUM_ENTRIES, HCOD2_MAX_LEN,
        HCOD2_NUM_ENTRIES, HCOD3_MAX_LEN, HCOD3_NUM_ENTRIES, HCOD4_MAX_LEN, HCOD4_NUM_ENTRIES,
        HCOD5_MAX_LEN, HCOD5_NUM_ENTRIES, HCOD6_MAX_LEN, HCOD6_NUM_ENTRIES, HCOD7_MAX_LEN,
        HCOD7_NUM_ENTRIES, HCOD8_MAX_LEN, HCOD8_NUM_ENTRIES, HCOD9_MAX_LEN, HCOD9_NUM_ENTRIES,
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

// =============================================================================
// Per-row spec-PDF spot checks (Table 4.A.6 — Codebook 5)
// =============================================================================
//
// Each spot row is taken from ISO/IEC 14496-3:2001(E) §4.A.1 Table
// 4.A.6 (pages 196–197) verbatim. The four 13-bit corners
// `(0, 8, 72, 80)`, the single 1-bit row at index 40 (the §4.6.3.3
// zero-tuple `(0, 0)`), and three interior rows (`13`, `31`, `41`)
// are spot-checked.

#[test]
fn table_4_a_6_row_0_matches_spec() {
    // Row 0 — (y, z) = (-4, -4): length 13, codeword 0x1fff.
    let (len, cw) = hcod5_encode(0).unwrap();
    assert_eq!((len, cw), (13, 0x1fff));
}

#[test]
fn table_4_a_6_row_8_matches_spec() {
    // Row 8 — (y, z) = (-4, +4): length 13, codeword 0x1ffd.
    let (len, cw) = hcod5_encode(8).unwrap();
    assert_eq!((len, cw), (13, 0x1ffd));
}

#[test]
fn table_4_a_6_row_13_matches_spec() {
    // Row 13 — (y, z) = (-3, 0): length 8, codeword 0xf0.
    let (len, cw) = hcod5_encode(13).unwrap();
    assert_eq!((len, cw), (8, 0xf0));
}

#[test]
fn table_4_a_6_row_31_matches_spec() {
    // Row 31 — (y, z) = (-1, -4): length 4, codeword 0x8.
    let (len, cw) = hcod5_encode(31).unwrap();
    assert_eq!((len, cw), (4, 0x8));
}

#[test]
fn table_4_a_6_row_40_matches_spec() {
    // Row 40 (zero-tuple): length 1, codeword 0.
    let (len, cw) = hcod5_encode(40).unwrap();
    assert_eq!((len, cw), (1, 0));
}

#[test]
fn table_4_a_6_row_41_matches_spec() {
    // Row 41 — (y, z) = (0, +1): length 4, codeword 0xa.
    let (len, cw) = hcod5_encode(41).unwrap();
    assert_eq!((len, cw), (4, 0xa));
}

#[test]
fn table_4_a_6_row_72_matches_spec() {
    // Row 72 — (y, z) = (+4, -4): length 13, codeword 0x1ffc.
    let (len, cw) = hcod5_encode(72).unwrap();
    assert_eq!((len, cw), (13, 0x1ffc));
}

#[test]
fn table_4_a_6_row_80_matches_spec() {
    // Row 80 — (y, z) = (+4, +4): length 13, codeword 0x1ffe.
    let (len, cw) = hcod5_encode(80).unwrap();
    assert_eq!((len, cw), (13, 0x1ffe));
}

// =============================================================================
// Codebook-shape cross-check vs Table 4.95 row 5
// =============================================================================

#[test]
fn codebook_5_table_4_95_row_matches_table_4_a_6_size() {
    // Row 5 of Table 4.95: signed, dim=2, lav=4 → (2*4+1)^2 = 81.
    let row = table_4_95(5).unwrap();
    assert_eq!(row.unsigned, Some(false));
    assert_eq!(row.dimension, Some(2));
    assert_eq!(row.lav, Some(4));
    assert_eq!(row.huffman_table, Some(6));

    let lav = row.lav.unwrap();
    let dim = row.dimension.unwrap();
    let modulus = 2 * lav + 1; // signed
    let count: u32 = modulus.pow(u32::from(dim));
    assert_eq!(count as usize, HCOD5_NUM_ENTRIES);
}

// =============================================================================
// Round-trip every entry through the Codebook 5 wire writer + reader
// =============================================================================

#[test]
fn hcod5_every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD5_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod5_write(&mut w, idx).unwrap();
        let bits_written = w.bit_position();
        let pad = (8 - (bits_written % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod5_decode(&mut br).unwrap();
        assert_eq!(
            got, idx,
            "round-trip mismatch at idx={} ({} bits written)",
            idx, bits_written
        );
    }
}

// =============================================================================
// Cross-check: decoded index → §4.6.3.3 tuple → re-encoded index.
// Codebook 5 is signed dim-2 LAV-4: every tuple element lies in
// `-4..=+4`. Tuple lives in slots `[0..2]` ([y, z]).
// =============================================================================

#[test]
fn hcod5_every_index_translates_through_spectral_codebook_round_trip() {
    for idx in 0..HCOD5_NUM_ENTRIES as u32 {
        let tup = decode_index_to_tuple(5, idx).expect("legal codebook-5 index");
        for (k, &v) in tup.iter().take(2).enumerate() {
            assert!(
                (-4..=4).contains(&v),
                "idx={} k={} tuple value {} outside [-LAV, +LAV] = [-4, +4]",
                idx,
                k,
                v
            );
        }
        let round = encode_tuple_to_index(5, &tup[..2]).expect("tuple round-trip");
        assert_eq!(round, idx, "index round-trip failed for idx={}", idx);
    }
}

#[test]
fn hcod5_signed_pair_book_puts_zero_tuple_at_index_40() {
    // The §4.6.3.3 translation polynomial puts `(0, 0)` at the
    // centre row (index 40) for a signed pair book with LAV = 4:
    // `idx = (y + 4) * 9 + (z + 4) = 4 * 9 + 4 = 40`.
    let tup = decode_index_to_tuple(5, 40).unwrap();
    assert_eq!(tup[..2], [0, 0]);
    // The four corners: 0, 8, 72, 80.
    assert_eq!(decode_index_to_tuple(5, 0).unwrap()[..2], [-4, -4]);
    assert_eq!(decode_index_to_tuple(5, 8).unwrap()[..2], [-4, 4]);
    assert_eq!(decode_index_to_tuple(5, 72).unwrap()[..2], [4, -4]);
    assert_eq!(decode_index_to_tuple(5, 80).unwrap()[..2], [4, 4]);
}

// =============================================================================
// Sign-bit cross-check: signed Codebook 5 emits zero sign bits — the
// codeword + index alone fully specifies the signed pair (the
// `offset = LAV = 4` shift inside the §4.6.3.3 polynomial bakes the
// sign into the index).
// =============================================================================

#[test]
fn hcod5_emits_zero_sign_bits_for_every_index() {
    for idx in 0..HCOD5_NUM_ENTRIES as u32 {
        let tup = decode_index_to_tuple(5, idx).unwrap();
        let signs = derive_sign_bits(5, &tup[..2]).unwrap();
        assert!(
            signs.is_empty(),
            "idx={}: signed book must emit no sign bits, got {} bits",
            idx,
            signs.len()
        );
    }
}

// =============================================================================
// Wire-bit precision: hand-built byte sequences for Codebook 5
// =============================================================================

#[test]
fn hcod5_write_index_40_then_align_yields_zero_byte() {
    // Index 40 is 1 bit `0`. Pad 7 zero bits → byte 0x00.
    let mut w = BitWriter::new();
    hcod5_write(&mut w, 40).unwrap();
    w.write_u32(0, 7);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x00]);
}

#[test]
fn hcod5_write_index_0_yields_full_13_bit_codeword_0x1fff() {
    // Codeword 0x1fff = 13 bits of all-ones, MSB-first. Pack into 13
    // bits + 3 zero pad bits → 0xff, 0xf8.
    let mut w = BitWriter::new();
    hcod5_write(&mut w, 0).unwrap();
    w.write_u32(0, 3);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0xff, 0xf8]);
}

#[test]
fn hcod5_write_two_indices_yields_concatenated_bitstream() {
    // Index 40 (1 bit `0`) followed by index 40 again (1 bit `0`).
    // Padded out to byte boundary with zeros: 0000_0000 = 0x00.
    // But the more interesting concatenation: index 40 (1 bit `0`)
    // followed by index 41 (4 bits `1010`) packs to 0_1010_xxx = 0x50
    // when right-padded with three zero bits.
    let mut w = BitWriter::new();
    hcod5_write(&mut w, 40).unwrap();
    hcod5_write(&mut w, 41).unwrap();
    w.write_u32(0, 3);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0b0101_0000]);

    let mut br = BitReader::new(&bytes);
    assert_eq!(hcod5_decode(&mut br).unwrap(), 40);
    assert_eq!(hcod5_decode(&mut br).unwrap(), 41);
}

// =============================================================================
// Decoder reads only as many bits as the codeword needs
// =============================================================================

#[test]
fn hcod5_decoder_consumes_exactly_codeword_length_bits() {
    for idx in 0..HCOD5_NUM_ENTRIES as u32 {
        let (len, _) = hcod5_encode(idx).unwrap();
        let mut w = BitWriter::new();
        hcod5_write(&mut w, idx).unwrap();
        let pad = (8 - (w.bit_position() % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let _ = hcod5_decode(&mut br).unwrap();
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
fn hcod5_encode_rejects_indices_at_and_above_81() {
    for bad in [81u32, 82, 100, 1000, u32::MAX] {
        assert!(matches!(
            hcod5_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(5))
        ));
    }
}

#[test]
fn hcod5_decoder_returns_unexpected_end_on_truncation() {
    let bytes: [u8; 0] = [];
    let mut br = BitReader::new(&bytes);
    let err = hcod5_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

#[test]
fn hcod5_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD5_NUM_ENTRIES as u32 {
        let (len, _) = hcod5_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD5_MAX_LEN);
}

// =============================================================================
// Cross-check: Codebooks 1..=4 share dim-4 (quad) shape; Codebook 5
// is the first pair (dim-2) book — the tuple universes are disjoint
// in the dim-arity dimension even though both happen to have 81
// entries (`3^4 = 9^2 = 81`).
// =============================================================================

#[test]
fn codebook_5_is_the_first_pair_book_with_dim_2_distinct_from_codebooks_1_through_4() {
    let row5 = table_4_95(5).unwrap();
    assert_eq!(row5.dimension, Some(2));
    for cb in 1u8..=4 {
        let row = table_4_95(cb).unwrap();
        assert_eq!(
            row.dimension,
            Some(4),
            "Codebooks 1..=4 are dim-4 (quad); Codebook 5 is dim-2 (pair)"
        );
    }
    // The 81-entry coincidence is meaningful: 3^4 (dim-4 with mod 3)
    // and 9^2 (dim-2 with mod 9) both produce 81 codeword positions.
    assert_eq!(HCOD5_NUM_ENTRIES, HCOD4_NUM_ENTRIES);
    assert_eq!(HCOD5_NUM_ENTRIES, HCOD3_NUM_ENTRIES);
    assert_eq!(HCOD5_NUM_ENTRIES, HCOD2_NUM_ENTRIES);
    assert_eq!(HCOD5_NUM_ENTRIES, HCOD1_NUM_ENTRIES);
}

// =============================================================================
// Per-row spec-PDF spot checks (Table 4.A.7 — Codebook 6)
// =============================================================================
//
// Each spot row is taken from ISO/IEC 14496-3:2001(E) §4.A.1 Table
// 4.A.7 (pages 197–198) verbatim. The four 11-bit corners
// `(0, 8, 72, 80)`, the 4-bit row at index 40 (the §4.6.3.3
// zero-tuple `(0, 0)`), and three interior rows (`13`, `31`, `41`)
// are spot-checked.

#[test]
fn table_4_a_7_row_0_matches_spec() {
    // Row 0 — (y, z) = (-4, -4): length 11, codeword 0x7fe.
    let (len, cw) = hcod6_encode(0).unwrap();
    assert_eq!((len, cw), (11, 0x7fe));
}

#[test]
fn table_4_a_7_row_8_matches_spec() {
    // Row 8 — (y, z) = (-4, +4): length 11, codeword 0x7fd.
    let (len, cw) = hcod6_encode(8).unwrap();
    assert_eq!((len, cw), (11, 0x7fd));
}

#[test]
fn table_4_a_7_row_13_matches_spec() {
    // Row 13 — (y, z) = (-3, 0): length 7, codeword 0x71.
    let (len, cw) = hcod6_encode(13).unwrap();
    assert_eq!((len, cw), (7, 0x71));
}

#[test]
fn table_4_a_7_row_31_matches_spec() {
    // Row 31 — (y, z) = (-1, -4): length 4, codeword 0x4.
    let (len, cw) = hcod6_encode(31).unwrap();
    assert_eq!((len, cw), (4, 0x4));
}

#[test]
fn table_4_a_7_row_40_matches_spec() {
    // Row 40 (zero-tuple): length 4, codeword 0.
    let (len, cw) = hcod6_encode(40).unwrap();
    assert_eq!((len, cw), (4, 0));
}

#[test]
fn table_4_a_7_row_41_matches_spec() {
    // Row 41 — (y, z) = (0, +1): length 4, codeword 0x3.
    let (len, cw) = hcod6_encode(41).unwrap();
    assert_eq!((len, cw), (4, 0x3));
}

#[test]
fn table_4_a_7_row_72_matches_spec() {
    // Row 72 — (y, z) = (+4, -4): length 11, codeword 0x7ff.
    let (len, cw) = hcod6_encode(72).unwrap();
    assert_eq!((len, cw), (11, 0x7ff));
}

#[test]
fn table_4_a_7_row_80_matches_spec() {
    // Row 80 — (y, z) = (+4, +4): length 11, codeword 0x7fc.
    let (len, cw) = hcod6_encode(80).unwrap();
    assert_eq!((len, cw), (11, 0x7fc));
}

// =============================================================================
// Codebook-shape cross-check vs Table 4.95 row 6
// =============================================================================

#[test]
fn codebook_6_table_4_95_row_matches_table_4_a_7_size() {
    // Row 6 of Table 4.95: signed, dim=2, lav=4 → (2*4+1)^2 = 81.
    let row = table_4_95(6).unwrap();
    assert_eq!(row.unsigned, Some(false));
    assert_eq!(row.dimension, Some(2));
    assert_eq!(row.lav, Some(4));
    assert_eq!(row.huffman_table, Some(7));

    let lav = row.lav.unwrap();
    let dim = row.dimension.unwrap();
    let modulus = 2 * lav + 1; // signed
    let count: u32 = modulus.pow(u32::from(dim));
    assert_eq!(count as usize, HCOD6_NUM_ENTRIES);
}

// =============================================================================
// Round-trip every entry through the Codebook 6 wire writer + reader
// =============================================================================

#[test]
fn hcod6_every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD6_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod6_write(&mut w, idx).unwrap();
        let bits_written = w.bit_position();
        let pad = (8 - (bits_written % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod6_decode(&mut br).unwrap();
        assert_eq!(
            got, idx,
            "round-trip mismatch at idx={} (wrote {} bits)",
            idx, bits_written
        );
    }
}

// =============================================================================
// §4.6.3.3 translation cross-check: every Codebook 6 index translates
// to a legal signed pair tuple and round-trips back. Codebook 6 is
// signed dim-2 LAV-4: every tuple element lies in `-4..=+4`.
// =============================================================================

#[test]
fn hcod6_every_index_translates_through_spectral_codebook_round_trip() {
    for idx in 0..HCOD6_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(6, idx).expect("idx -> tuple");
        // dim == 2 → only the first two slots are meaningful.
        for &c in &tuple[..2] {
            assert!(
                (-4..=4).contains(&c),
                "tuple coefficient {} out of [-4, 4]",
                c
            );
        }
        let back = encode_tuple_to_index(6, &tuple[..2]).expect("tuple -> idx");
        assert_eq!(back, idx);
    }
}

#[test]
fn hcod6_signed_pair_book_puts_zero_tuple_at_index_40() {
    // Index 40 → zero-tuple `(0, 0)`. Confirmed via the §4.6.3.3
    // decode (modulus 9, offset 4: `idx = 4*9 + 4 = 40` ↔ `y = 0,
    // z = 0`).
    let tuple = decode_index_to_tuple(6, 40).unwrap();
    assert_eq!(tuple[0], 0);
    assert_eq!(tuple[1], 0);
    // And the wire codeword is the shortest in the table.
    let (len, cw) = hcod6_encode(40).unwrap();
    assert_eq!((len, cw), (4, 0));
}

// =============================================================================
// Sign-bit cross-check: signed Codebook 6 emits zero sign bits — the
// signedness is baked into the §4.6.3.3 index via the LAV-4 offset.
// =============================================================================

#[test]
fn hcod6_emits_zero_sign_bits_for_every_index() {
    for idx in 0..HCOD6_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(6, idx).unwrap();
        let signs = derive_sign_bits(6, &tuple[..2]).unwrap();
        assert!(
            signs.is_empty(),
            "Codebook 6 is signed; sign-bit suffix must be empty (idx={})",
            idx
        );
    }
}

// =============================================================================
// Wire-bit precision: hand-built byte sequences for Codebook 6
// =============================================================================

#[test]
fn hcod6_write_index_40_then_align_yields_zero_byte() {
    let mut w = BitWriter::new();
    hcod6_write(&mut w, 40).unwrap();
    // Row 40 is 4 bits `0b0000`; pad 4 zero bits to reach a byte.
    w.write_u32(0, 4);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x00]);
}

#[test]
fn hcod6_write_index_72_yields_full_11_bit_codeword_0x7ff() {
    let mut w = BitWriter::new();
    hcod6_write(&mut w, 72).unwrap();
    // Row 72 is 11 bits `0x7ff` = `0b111_1111_1111` → pad 5 zero
    // bits → bytes `0xff, 0xe0`.
    w.write_u32(0, 5);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0xff, 0xe0]);
}

#[test]
fn hcod6_write_two_indices_yields_concatenated_bitstream() {
    let mut w = BitWriter::new();
    // Index 40 (4 bits `0b0000`) followed by index 41 (4 bits
    // `0b0011`) packs into a single byte `0b0000_0011` = `0x03`.
    hcod6_write(&mut w, 40).unwrap();
    hcod6_write(&mut w, 41).unwrap();
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x03]);
    let mut br = BitReader::new(&bytes);
    assert_eq!(hcod6_decode(&mut br).unwrap(), 40);
    assert_eq!(hcod6_decode(&mut br).unwrap(), 41);
}

// =============================================================================
// Bit-consumption invariant for Codebook 6: each decode consumes
// exactly its declared codeword length.
// =============================================================================

#[test]
fn hcod6_decoder_consumes_exactly_codeword_length_bits() {
    for idx in 0..HCOD6_NUM_ENTRIES as u32 {
        let (len, _) = hcod6_encode(idx).unwrap();
        let mut w = BitWriter::new();
        hcod6_write(&mut w, idx).unwrap();
        let pad = (8 - (u32::from(len) % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let _ = hcod6_decode(&mut br).unwrap();
        assert_eq!(
            br.bit_position() as u32,
            u32::from(len),
            "idx={}: decoder consumed {} bits, expected {}",
            idx,
            br.bit_position(),
            len
        );
    }
}

// =============================================================================
// Rejection branches for Codebook 6
// =============================================================================

#[test]
fn hcod6_encode_rejects_indices_at_and_above_81() {
    for bad in [81u32, 82, 100, 1000, u32::MAX] {
        assert!(matches!(
            hcod6_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(6))
        ));
    }
}

#[test]
fn hcod6_decoder_returns_unexpected_end_on_truncation() {
    let bytes: [u8; 0] = [];
    let mut br = BitReader::new(&bytes);
    let err = hcod6_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

#[test]
fn hcod6_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD6_NUM_ENTRIES as u32 {
        let (len, _) = hcod6_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD6_MAX_LEN);
}

// =============================================================================
// Cross-check: Codebooks 5 and 6 share the signed dim-2 LAV-4 pair
// tuple universe (Table 4.95 rows 5 and 6 are identical except for
// the `Codebook listed in Table` column) but assign different
// codewords for the same `(y, z)` tuple.
// =============================================================================

#[test]
fn codebook_6_is_the_second_signed_pair_book_with_dim_2() {
    let row5 = table_4_95(5).unwrap();
    let row6 = table_4_95(6).unwrap();
    // Same shape — both signed, dim-2, lav-4.
    assert_eq!(row5.unsigned, row6.unsigned);
    assert_eq!(row5.dimension, row6.dimension);
    assert_eq!(row5.lav, row6.lav);
    // Different table column though.
    assert_eq!(row5.huffman_table, Some(6));
    assert_eq!(row6.huffman_table, Some(7));
    // 81-entry universe shared.
    assert_eq!(HCOD6_NUM_ENTRIES, HCOD5_NUM_ENTRIES);
}

#[test]
fn codebook_5_and_6_share_lattice_corner_index_positions() {
    // Both books pin the four (±4, ±4) lattice corners to their
    // respective max-length codewords (Codebook 5 → 13 bits,
    // Codebook 6 → 11 bits) at the same indices 0, 8, 72, 80.
    for &corner in &[0u32, 8, 72, 80] {
        let (l5, _) = hcod5_encode(corner).unwrap();
        let (l6, _) = hcod6_encode(corner).unwrap();
        assert_eq!(u32::from(l5), HCOD5_MAX_LEN);
        assert_eq!(u32::from(l6), HCOD6_MAX_LEN);
    }
}

// =============================================================================
// Per-row spec-PDF spot checks (Table 4.A.8)
// =============================================================================
//
// Each spot row is taken from ISO/IEC 14496-3:2001(E) §4.A.1 Table
// 4.A.8 (page 198) verbatim. The 1-bit row at index 0 (the §4.6.3.3
// zero-tuple `(0, 0)`), the dim-2 boundary at index 8 (first y=1
// row), the far corner at index 63 ((y, z) = (7, 7)), and five
// interior rows (`1`, `9`, `16`, `40`, `55`) are spot-checked.

#[test]
fn table_4_a_8_row_0_matches_spec() {
    // Row 0 — (y, z) = (0, 0): length 1, codeword 0.
    let (len, cw) = hcod7_encode(0).unwrap();
    assert_eq!((len, cw), (1, 0));
}

#[test]
fn table_4_a_8_row_1_matches_spec() {
    // Row 1 — (y, z) = (0, 1): length 3, codeword 0x5.
    let (len, cw) = hcod7_encode(1).unwrap();
    assert_eq!((len, cw), (3, 0x5));
}

#[test]
fn table_4_a_8_row_8_matches_spec() {
    // Row 8 — (y, z) = (1, 0): length 3, codeword 0x4.
    let (len, cw) = hcod7_encode(8).unwrap();
    assert_eq!((len, cw), (3, 0x4));
}

#[test]
fn table_4_a_8_row_9_matches_spec() {
    // Row 9 — (y, z) = (1, 1): length 4, codeword 0xc.
    let (len, cw) = hcod7_encode(9).unwrap();
    assert_eq!((len, cw), (4, 0xc));
}

#[test]
fn table_4_a_8_row_16_matches_spec() {
    // Row 16 — (y, z) = (2, 0): length 6, codeword 0x36.
    let (len, cw) = hcod7_encode(16).unwrap();
    assert_eq!((len, cw), (6, 0x36));
}

#[test]
fn table_4_a_8_row_40_matches_spec() {
    // Row 40 — (y, z) = (5, 0): length 9, codeword 0x1ed.
    let (len, cw) = hcod7_encode(40).unwrap();
    assert_eq!((len, cw), (9, 0x1ed));
}

#[test]
fn table_4_a_8_row_55_matches_spec() {
    // Row 55 — (y, z) = (6, 7): length 12, codeword 0xffe.
    let (len, cw) = hcod7_encode(55).unwrap();
    assert_eq!((len, cw), (12, 0xffe));
}

#[test]
fn table_4_a_8_row_63_matches_spec() {
    // Row 63 — (y, z) = (7, 7): length 12, codeword 0xfff.
    let (len, cw) = hcod7_encode(63).unwrap();
    assert_eq!((len, cw), (12, 0xfff));
}

// =============================================================================
// Codebook-shape cross-check vs Table 4.95 row 7
// =============================================================================

#[test]
fn codebook_7_table_4_95_row_matches_table_4_a_8_size() {
    // Row 7 of Table 4.95: unsigned, dim=2, lav=7 → (7+1)^2 = 64.
    let row = table_4_95(7).unwrap();
    assert_eq!(row.unsigned, Some(true));
    assert_eq!(row.dimension, Some(2));
    assert_eq!(row.lav, Some(7));
    assert_eq!(row.huffman_table, Some(8));

    let lav = row.lav.unwrap();
    let dim = row.dimension.unwrap();
    let modulus = lav + 1; // unsigned
    let count: u32 = modulus.pow(u32::from(dim));
    assert_eq!(count as usize, HCOD7_NUM_ENTRIES);
}

// =============================================================================
// Round-trip every entry through the Codebook 7 wire writer + reader
// =============================================================================

#[test]
fn hcod7_every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD7_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod7_write(&mut w, idx).unwrap();
        let bits_written = w.bit_position();
        let pad = (8 - (bits_written % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod7_decode(&mut br).unwrap();
        assert_eq!(
            got, idx,
            "round-trip mismatch at idx={} (wrote {} bits)",
            idx, bits_written
        );
    }
}

// =============================================================================
// §4.6.3.3 translation cross-check: every Codebook 7 index translates
// to a legal unsigned pair tuple and round-trips back. Codebook 7 is
// unsigned dim-2 LAV-7: every tuple element lies in `0..=7`.
// =============================================================================

#[test]
fn hcod7_every_index_translates_through_spectral_codebook_round_trip() {
    for idx in 0..HCOD7_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(7, idx).expect("idx -> tuple");
        // dim == 2 → only the first two slots are meaningful.
        for &c in &tuple[..2] {
            assert!(
                (0..=7).contains(&c),
                "tuple coefficient {} outside [0, 7]",
                c
            );
        }
        let back = encode_tuple_to_index(7, &tuple[..2]).expect("tuple -> idx");
        assert_eq!(back, idx);
    }
}

#[test]
fn hcod7_unsigned_pair_book_puts_zero_tuple_at_index_0() {
    // Index 0 → zero-tuple `(0, 0)`. Confirmed via the §4.6.3.3
    // decode (modulus 8, offset 0: `idx = 0*8 + 0 = 0` ↔ `y = 0,
    // z = 0`).
    let tuple = decode_index_to_tuple(7, 0).unwrap();
    assert_eq!(tuple[0], 0);
    assert_eq!(tuple[1], 0);
    // And the wire codeword is the shortest in the table.
    let (len, cw) = hcod7_encode(0).unwrap();
    assert_eq!((len, cw), (1, 0));
}

#[test]
fn hcod7_far_corner_lives_at_index_63() {
    // Index 63 → (y, z) = (7, 7) via `idx = 7*8 + 7 = 63`. The
    // longest codeword shares the table ceiling with three other
    // 12-bit rows (54, 55, 62).
    let tuple = decode_index_to_tuple(7, 63).unwrap();
    assert_eq!(tuple[0], 7);
    assert_eq!(tuple[1], 7);
    let (len, cw) = hcod7_encode(63).unwrap();
    assert_eq!((len, cw), (12, 0xfff));
}

// =============================================================================
// Sign-bit cross-check: unsigned Codebook 7 emits one sign bit per
// non-zero coefficient via the §4.6.3.3 suffix. The suffix lives
// outside the Huffman codeword.
// =============================================================================

#[test]
fn hcod7_emits_one_sign_bit_per_nonzero_coefficient() {
    for idx in 0..HCOD7_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(7, idx).unwrap();
        let signs = derive_sign_bits(7, &tuple[..2]).unwrap();
        let expected = tuple[..2].iter().filter(|&&c| c != 0).count();
        assert_eq!(
            signs.len(),
            expected,
            "idx={}: tuple {:?} expected {} sign bits, got {}",
            idx,
            &tuple[..2],
            expected,
            signs.len()
        );
        // Every sign bit from the unsigned-magnitude path is `false`
        // (positive) because `decode_index_to_tuple` returns
        // magnitudes in `0..=lav` for unsigned books.
        for s in &signs {
            assert!(
                !*s,
                "Codebook 7 derive_sign_bits on a positive tuple yields false bits"
            );
        }
    }
}

#[test]
fn hcod7_zero_tuple_emits_zero_sign_bits() {
    // The zero-tuple `(0, 0)` at index 0 carries two zero coefficients,
    // so the §4.6.3.3 sign-bit suffix is empty (the suffix emits one
    // bit per non-zero coefficient, low-frequency-first).
    let tuple = decode_index_to_tuple(7, 0).unwrap();
    let signs = derive_sign_bits(7, &tuple[..2]).unwrap();
    assert!(signs.is_empty());
}

#[test]
fn hcod7_max_tuple_emits_two_sign_bits() {
    // Index 63 (`(7, 7)`) has two non-zero coefficients → two sign
    // bits in the §4.6.3.3 suffix.
    let tuple = decode_index_to_tuple(7, 63).unwrap();
    let signs = derive_sign_bits(7, &tuple[..2]).unwrap();
    assert_eq!(signs.len(), 2);
}

// =============================================================================
// Wire-bit precision: hand-built byte sequences for Codebook 7
// =============================================================================

#[test]
fn hcod7_write_index_0_then_align_yields_zero_byte() {
    let mut w = BitWriter::new();
    hcod7_write(&mut w, 0).unwrap();
    // Row 0 is 1 bit `0`; pad 7 zero bits to reach a byte.
    w.write_u32(0, 7);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x00]);
}

#[test]
fn hcod7_write_index_63_yields_full_12_bit_codeword_0xfff() {
    let mut w = BitWriter::new();
    hcod7_write(&mut w, 63).unwrap();
    // Row 63 is 12 bits `0xfff` = `0b1111_1111_1111` → pad 4 zero
    // bits → bytes `0xff, 0xf0`.
    w.write_u32(0, 4);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0xff, 0xf0]);
}

#[test]
fn hcod7_write_two_indices_yields_concatenated_bitstream() {
    let mut w = BitWriter::new();
    // Index 0 (1 bit `0`) followed by index 8 (3 bits `0b100`)
    // followed by index 9 (4 bits `0b1100`) packs into a single byte
    // `0b0100_1100` = `0x4c`.
    hcod7_write(&mut w, 0).unwrap();
    hcod7_write(&mut w, 8).unwrap();
    hcod7_write(&mut w, 9).unwrap();
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x4c]);
    let mut br = BitReader::new(&bytes);
    assert_eq!(hcod7_decode(&mut br).unwrap(), 0);
    assert_eq!(hcod7_decode(&mut br).unwrap(), 8);
    assert_eq!(hcod7_decode(&mut br).unwrap(), 9);
}

// =============================================================================
// Bit-consumption invariant for Codebook 7: each decode consumes
// exactly its declared codeword length.
// =============================================================================

#[test]
fn hcod7_decoder_consumes_exactly_codeword_length_bits() {
    for idx in 0..HCOD7_NUM_ENTRIES as u32 {
        let (len, _) = hcod7_encode(idx).unwrap();
        let mut w = BitWriter::new();
        hcod7_write(&mut w, idx).unwrap();
        let pad = (8 - (u32::from(len) % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let _ = hcod7_decode(&mut br).unwrap();
        assert_eq!(
            br.bit_position() as u32,
            u32::from(len),
            "idx={}: decoder consumed {} bits, expected {}",
            idx,
            br.bit_position(),
            len
        );
    }
}

// =============================================================================
// Rejection branches for Codebook 7
// =============================================================================

#[test]
fn hcod7_encode_rejects_indices_at_and_above_64() {
    for bad in [64u32, 65, 80, 1000, u32::MAX] {
        assert!(matches!(
            hcod7_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(7))
        ));
    }
}

#[test]
fn hcod7_decoder_returns_unexpected_end_on_truncation() {
    let bytes: [u8; 0] = [];
    let mut br = BitReader::new(&bytes);
    let err = hcod7_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

#[test]
fn hcod7_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD7_NUM_ENTRIES as u32 {
        let (len, _) = hcod7_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD7_MAX_LEN);
}

// =============================================================================
// Cross-check: Codebook 7 is the first unsigned **pair** book — a
// shape change from the 81-entry universe shared by Codebooks 1..=6.
// Codebook 3 was the first unsigned book overall (dim-4); Codebook 7
// is the first unsigned pair book (dim-2), so both share the "index 0
// is the zero-tuple with the shortest codeword" head placement that
// the §4.6.3.3 unsigned polynomial dictates.
// =============================================================================

#[test]
fn codebook_7_is_the_first_unsigned_pair_book_with_dim_2() {
    let row6 = table_4_95(6).unwrap();
    let row7 = table_4_95(7).unwrap();
    // Codebook 6 is signed dim-2 LAV-4; Codebook 7 is unsigned
    // dim-2 LAV-7. Both dim-2, distinct signedness and range.
    assert_eq!(row6.unsigned, Some(false));
    assert_eq!(row7.unsigned, Some(true));
    assert_eq!(row6.dimension, row7.dimension);
    assert_eq!(row6.lav, Some(4));
    assert_eq!(row7.lav, Some(7));
    // Universe-size shift: 81 → 64.
    assert_eq!(HCOD6_NUM_ENTRIES, 81);
    assert_eq!(HCOD7_NUM_ENTRIES, 64);
}

#[test]
fn codebook_3_and_7_share_zero_tuple_at_index_0() {
    // Both unsigned books place the §4.6.3.3 zero-tuple at index 0
    // with the single-bit `0` codeword. Codebook 3 uses dim-4; Codebook
    // 7 uses dim-2 — the shape differs, the head placement matches.
    let (l3, cw3) = hcod3_encode(0).unwrap();
    let (l7, cw7) = hcod7_encode(0).unwrap();
    assert_eq!((l3, cw3), (1, 0));
    assert_eq!((l7, cw7), (1, 0));
}

#[test]
fn codebook_7_universe_is_64_distinct_unsigned_pairs() {
    // Every legal (y, z) pair with y, z in 0..=7 must map to a
    // distinct index, and every index must map back to a distinct
    // pair. This is the §4.6.3.3 bijection at dim-2 LAV-7.
    let mut seen = std::collections::HashSet::new();
    for y in 0..=7i32 {
        for z in 0..=7i32 {
            let idx = encode_tuple_to_index(7, &[y, z]).unwrap();
            assert!(idx < HCOD7_NUM_ENTRIES as u32);
            assert!(
                seen.insert(idx),
                "duplicate idx={} for tuple ({}, {})",
                idx,
                y,
                z
            );
            let back = decode_index_to_tuple(7, idx).unwrap();
            assert_eq!(back[0], y);
            assert_eq!(back[1], z);
        }
    }
    assert_eq!(seen.len(), HCOD7_NUM_ENTRIES);
}

// =============================================================================
// Per-row spec-PDF spot checks (Table 4.A.9)
// =============================================================================
//
// Each spot row is taken from ISO/IEC 14496-3:2001(E) §4.A.1 Table
// 4.A.9 (page 198) verbatim. The 5-bit row at index 0 (the §4.6.3.3
// zero-tuple `(0, 0)` lifted off the shortest slot), the 3-bit row at
// index 9 ((y, z) = (1, 1), the shortest codeword in this book), the
// dim-2 boundary at index 8 (first y=1 row), the far corner at index
// 63 ((y, z) = (7, 7)), and four interior rows (`1`, `7`, `40`, `55`)
// are spot-checked.

#[test]
fn table_4_a_9_row_0_matches_spec() {
    // Row 0 — (y, z) = (0, 0): length 5, codeword 0xe.
    let (len, cw) = hcod8_encode(0).unwrap();
    assert_eq!((len, cw), (5, 0xe));
}

#[test]
fn table_4_a_9_row_1_matches_spec() {
    // Row 1 — (y, z) = (0, 1): length 4, codeword 0x5.
    let (len, cw) = hcod8_encode(1).unwrap();
    assert_eq!((len, cw), (4, 0x5));
}

#[test]
fn table_4_a_9_row_7_matches_spec() {
    // Row 7 — (y, z) = (0, 7): length 10, codeword 0x3fe.
    let (len, cw) = hcod8_encode(7).unwrap();
    assert_eq!((len, cw), (10, 0x3fe));
}

#[test]
fn table_4_a_9_row_8_matches_spec() {
    // Row 8 — (y, z) = (1, 0): length 4, codeword 0x3.
    let (len, cw) = hcod8_encode(8).unwrap();
    assert_eq!((len, cw), (4, 0x3));
}

#[test]
fn table_4_a_9_row_9_matches_spec() {
    // Row 9 — (y, z) = (1, 1): length 3, codeword 0 — the shortest
    // codeword in Codebook 8 (the zero-tuple gives way to the
    // statistically more common `(1, 1)` interior tuple).
    let (len, cw) = hcod8_encode(9).unwrap();
    assert_eq!((len, cw), (3, 0));
}

#[test]
fn table_4_a_9_row_40_matches_spec() {
    // Row 40 — (y, z) = (5, 0): length 8, codeword 0xef.
    let (len, cw) = hcod8_encode(40).unwrap();
    assert_eq!((len, cw), (8, 0xef));
}

#[test]
fn table_4_a_9_row_55_matches_spec() {
    // Row 55 — (y, z) = (6, 7): length 9, codeword 0x1fd.
    let (len, cw) = hcod8_encode(55).unwrap();
    assert_eq!((len, cw), (9, 0x1fd));
}

#[test]
fn table_4_a_9_row_63_matches_spec() {
    // Row 63 — (y, z) = (7, 7): length 10, codeword 0x3ff.
    let (len, cw) = hcod8_encode(63).unwrap();
    assert_eq!((len, cw), (10, 0x3ff));
}

// =============================================================================
// Codebook-shape cross-check vs Table 4.95 row 8
// =============================================================================

#[test]
fn codebook_8_table_4_95_row_matches_table_4_a_9_size() {
    // Row 8 of Table 4.95: unsigned, dim=2, lav=7 → (7+1)^2 = 64.
    let row = table_4_95(8).unwrap();
    assert_eq!(row.unsigned, Some(true));
    assert_eq!(row.dimension, Some(2));
    assert_eq!(row.lav, Some(7));
    assert_eq!(row.huffman_table, Some(9));

    let lav = row.lav.unwrap();
    let dim = row.dimension.unwrap();
    let modulus = lav + 1; // unsigned
    let count: u32 = modulus.pow(u32::from(dim));
    assert_eq!(count as usize, HCOD8_NUM_ENTRIES);
}

// =============================================================================
// Round-trip every entry through the Codebook 8 wire writer + reader
// =============================================================================

#[test]
fn hcod8_every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD8_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod8_write(&mut w, idx).unwrap();
        let bits_written = w.bit_position();
        let pad = (8 - (bits_written % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad as u32);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod8_decode(&mut br).unwrap();
        assert_eq!(
            got, idx,
            "round-trip mismatch at idx={} (wrote {} bits)",
            idx, bits_written
        );
    }
}

// =============================================================================
// §4.6.3.3 translation cross-check: every Codebook 8 index translates
// to a legal unsigned pair tuple and round-trips back. Codebook 8 is
// unsigned dim-2 LAV-7 (same universe as Codebook 7): every tuple
// element lies in `0..=7`.
// =============================================================================

#[test]
fn hcod8_every_index_translates_through_spectral_codebook_round_trip() {
    for idx in 0..HCOD8_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(8, idx).expect("idx -> tuple");
        // dim == 2 → only the first two slots are meaningful.
        for &c in &tuple[..2] {
            assert!(
                (0..=7).contains(&c),
                "tuple coefficient {} outside [0, 7]",
                c
            );
        }
        let back = encode_tuple_to_index(8, &tuple[..2]).expect("tuple -> idx");
        assert_eq!(back, idx);
    }
}

#[test]
fn hcod8_zero_tuple_at_index_0_carries_5_bit_codeword() {
    // Index 0 → zero-tuple `(0, 0)` per the §4.6.3.3 unsigned dim-2
    // polynomial (modulus 8, offset 0: `idx = 0*8 + 0 = 0` ↔ `y = 0,
    // z = 0`). Codebook 8 lifts the zero-tuple off the shortest slot
    // — the codeword is 5 bits long.
    let tuple = decode_index_to_tuple(8, 0).unwrap();
    assert_eq!(tuple[0], 0);
    assert_eq!(tuple[1], 0);
    let (len, cw) = hcod8_encode(0).unwrap();
    assert_eq!((len, cw), (5, 0xe));
}

#[test]
fn hcod8_interior_tuple_1_1_holds_shortest_codeword() {
    // Index 9 → (y, z) = (1, 1) via `idx = 1*8 + 1 = 9`. The 3-bit
    // codeword `0` is the shortest in Codebook 8, signalling that the
    // book's target statistics put more weight on the `(1, 1)` interior
    // than the `(0, 0)` zero-tuple corner Codebook 7 optimises for.
    let tuple = decode_index_to_tuple(8, 9).unwrap();
    assert_eq!(tuple[0], 1);
    assert_eq!(tuple[1], 1);
    let (len, cw) = hcod8_encode(9).unwrap();
    assert_eq!((len, cw), (3, 0));
}

#[test]
fn hcod8_far_corner_lives_at_index_63() {
    // Index 63 → (y, z) = (7, 7) via `idx = 7*8 + 7 = 63`. The
    // longest codeword shares the table ceiling with three other
    // 10-bit rows (7, 47, 56).
    let tuple = decode_index_to_tuple(8, 63).unwrap();
    assert_eq!(tuple[0], 7);
    assert_eq!(tuple[1], 7);
    let (len, cw) = hcod8_encode(63).unwrap();
    assert_eq!((len, cw), (10, 0x3ff));
}

// =============================================================================
// Sign-bit cross-check: unsigned Codebook 8 emits one sign bit per
// non-zero coefficient via the §4.6.3.3 suffix. The suffix lives
// outside the Huffman codeword.
// =============================================================================

#[test]
fn hcod8_emits_one_sign_bit_per_nonzero_coefficient() {
    for idx in 0..HCOD8_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(8, idx).unwrap();
        let signs = derive_sign_bits(8, &tuple[..2]).unwrap();
        let expected = tuple[..2].iter().filter(|&&c| c != 0).count();
        assert_eq!(
            signs.len(),
            expected,
            "idx={}: tuple {:?} expected {} sign bits, got {}",
            idx,
            &tuple[..2],
            expected,
            signs.len()
        );
        // Every sign bit from the unsigned-magnitude path is `false`
        // (positive) because `decode_index_to_tuple` returns
        // magnitudes in `0..=lav` for unsigned books.
        for s in &signs {
            assert!(
                !*s,
                "Codebook 8 derive_sign_bits on a positive tuple yields false bits"
            );
        }
    }
}

#[test]
fn hcod8_zero_tuple_emits_zero_sign_bits() {
    // The zero-tuple `(0, 0)` at index 0 carries two zero coefficients,
    // so the §4.6.3.3 sign-bit suffix is empty (the suffix emits one
    // bit per non-zero coefficient, low-frequency-first).
    let tuple = decode_index_to_tuple(8, 0).unwrap();
    let signs = derive_sign_bits(8, &tuple[..2]).unwrap();
    assert!(signs.is_empty());
}

#[test]
fn hcod8_max_tuple_emits_two_sign_bits() {
    // Index 63 (`(7, 7)`) has two non-zero coefficients → two sign
    // bits in the §4.6.3.3 suffix.
    let tuple = decode_index_to_tuple(8, 63).unwrap();
    let signs = derive_sign_bits(8, &tuple[..2]).unwrap();
    assert_eq!(signs.len(), 2);
}

// =============================================================================
// Wire-bit precision: hand-built byte sequences for Codebook 8
// =============================================================================

#[test]
fn hcod8_write_index_9_then_align_yields_zero_byte() {
    let mut w = BitWriter::new();
    hcod8_write(&mut w, 9).unwrap();
    // Row 9 is 3 bits `0b000`; pad 5 zero bits to reach a byte.
    w.write_u32(0, 5);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x00]);
}

#[test]
fn hcod8_write_index_63_yields_full_10_bit_codeword_0x3ff() {
    let mut w = BitWriter::new();
    hcod8_write(&mut w, 63).unwrap();
    // Row 63 is 10 bits `0x3ff` = `0b11_1111_1111` → pad 6 zero bits
    // → bytes `0xff, 0xc0`.
    w.write_u32(0, 6);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0xff, 0xc0]);
}

#[test]
fn hcod8_write_three_indices_yields_concatenated_bitstream() {
    let mut w = BitWriter::new();
    // Index 9 (3 bits `0b000`), then index 8 (4 bits `0b0011`),
    // then index 0 (5 bits `0b01110`). Total 12 bits:
    // `0b000_0011_01110` → byte split `0b0000_0110 0b1110_xxxx`
    // = `0x06, 0xe0` after padding the trailing four bits with zero.
    hcod8_write(&mut w, 9).unwrap();
    hcod8_write(&mut w, 8).unwrap();
    hcod8_write(&mut w, 0).unwrap();
    w.write_u32(0, 4);
    let bytes = w.into_bytes();
    assert_eq!(bytes, vec![0x06, 0xe0]);
    let mut br = BitReader::new(&bytes);
    assert_eq!(hcod8_decode(&mut br).unwrap(), 9);
    assert_eq!(hcod8_decode(&mut br).unwrap(), 8);
    assert_eq!(hcod8_decode(&mut br).unwrap(), 0);
}

// =============================================================================
// Bit-consumption invariant for Codebook 8: each decode consumes
// exactly its declared codeword length.
// =============================================================================

#[test]
fn hcod8_decoder_consumes_exactly_codeword_length_bits() {
    for idx in 0..HCOD8_NUM_ENTRIES as u32 {
        let (len, _) = hcod8_encode(idx).unwrap();
        let mut w = BitWriter::new();
        hcod8_write(&mut w, idx).unwrap();
        let pad = (8 - (u32::from(len) % 8)) % 8;
        if pad > 0 {
            w.write_u32(0, pad);
        }
        let bytes = w.into_bytes();
        let mut br = BitReader::new(&bytes);
        let _ = hcod8_decode(&mut br).unwrap();
        assert_eq!(
            br.bit_position() as u32,
            u32::from(len),
            "idx={}: decoder consumed {} bits, expected {}",
            idx,
            br.bit_position(),
            len
        );
    }
}

// =============================================================================
// Rejection branches for Codebook 8
// =============================================================================

#[test]
fn hcod8_encode_rejects_indices_at_and_above_64() {
    for bad in [64u32, 65, 80, 1000, u32::MAX] {
        assert!(matches!(
            hcod8_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(8))
        ));
    }
}

#[test]
fn hcod8_decoder_returns_unexpected_end_on_truncation() {
    let bytes: [u8; 0] = [];
    let mut br = BitReader::new(&bytes);
    let err = hcod8_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

#[test]
fn hcod8_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD8_NUM_ENTRIES as u32 {
        let (len, _) = hcod8_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD8_MAX_LEN);
}

// =============================================================================
// Cross-check: Codebook 8 is the second unsigned **pair** book — it
// shares Codebook 7's `unsigned = 1`, `dim = 2`, `LAV = 7` shape but
// retunes the per-row codeword lengths. The encoder chooses between
// the two per-section via `section_data()`'s `sect_cb` field.
// =============================================================================

#[test]
fn codebook_8_is_the_second_unsigned_pair_book_with_dim_2() {
    let row7 = table_4_95(7).unwrap();
    let row8 = table_4_95(8).unwrap();
    // Same shape — both unsigned dim-2 LAV-7.
    assert_eq!(row7.unsigned, row8.unsigned);
    assert_eq!(row7.dimension, row8.dimension);
    assert_eq!(row7.lav, row8.lav);
    // Different table column though.
    assert_eq!(row7.huffman_table, Some(8));
    assert_eq!(row8.huffman_table, Some(9));
    // 64-entry universe shared.
    assert_eq!(HCOD8_NUM_ENTRIES, HCOD7_NUM_ENTRIES);
}

#[test]
fn codebook_8_universe_is_64_distinct_unsigned_pairs() {
    // Every legal (y, z) pair with y, z in 0..=7 must map to a
    // distinct index, and every index must map back to a distinct
    // pair. This is the §4.6.3.3 bijection at dim-2 LAV-7 — the same
    // bijection Codebook 7 walks; the wire codewords differ.
    let mut seen = std::collections::HashSet::new();
    for y in 0..=7i32 {
        for z in 0..=7i32 {
            let idx = encode_tuple_to_index(8, &[y, z]).unwrap();
            assert!(idx < HCOD8_NUM_ENTRIES as u32);
            assert!(
                seen.insert(idx),
                "duplicate idx={} for tuple ({}, {})",
                idx,
                y,
                z
            );
            let back = decode_index_to_tuple(8, idx).unwrap();
            assert_eq!(back[0], y);
            assert_eq!(back[1], z);
        }
    }
    assert_eq!(seen.len(), HCOD8_NUM_ENTRIES);
}

#[test]
fn codebook_7_and_8_disagree_on_zero_tuple_codeword() {
    // Codebook 7: zero-tuple `(0, 0)` at index 0 takes the 1-bit
    // shortest slot.
    let (l7, cw7) = hcod7_encode(0).unwrap();
    assert_eq!((l7, cw7), (1, 0));
    // Codebook 8: zero-tuple at the same index 0 carries a 5-bit
    // codeword 0xe — the shortest slot has moved to index 9.
    let (l8, cw8) = hcod8_encode(0).unwrap();
    assert_eq!((l8, cw8), (5, 0xe));
}

#[test]
fn codebook_7_and_8_share_far_corner_index_position() {
    // Both books pin `(7, 7)` at index 63 (the §4.6.3.3 unsigned
    // polynomial puts the far corner at the highest index). Only
    // the codeword length / value differs: Codebook 7 → 12-bit
    // 0xfff; Codebook 8 → 10-bit 0x3ff.
    let (l7, cw7) = hcod7_encode(63).unwrap();
    let (l8, cw8) = hcod8_encode(63).unwrap();
    assert_eq!((l7, cw7), (12, 0xfff));
    assert_eq!((l8, cw8), (10, 0x3ff));
}

// =============================================================================
// Per-row spec-PDF spot checks (Table 4.A.10)
// =============================================================================
//
// Each spot row is taken from ISO/IEC 14496-3:2009(E) §4.A.1 Table
// 4.A.10 verbatim. The full 169-row table-shape and Kraft completeness
// is covered by the unit-tests; this module locks the per-row
// hexadecimal column at integration-test scope.

#[test]
fn table_4_a_10_row_0_matches_spec() {
    // Row 0: length 1, codeword 0 (the §4.6.3.3 zero-tuple `(0, 0)`).
    let (len, cw) = hcod9_encode(0).unwrap();
    assert_eq!((len, cw), (1, 0));
}

#[test]
fn table_4_a_10_row_1_matches_spec() {
    // Row 1: length 3, codeword 0x5.
    let (len, cw) = hcod9_encode(1).unwrap();
    assert_eq!((len, cw), (3, 0x5));
}

#[test]
fn table_4_a_10_row_13_matches_spec() {
    // Row 13: length 3, codeword 0x4. The other 3-bit row in the
    // table.
    let (len, cw) = hcod9_encode(13).unwrap();
    assert_eq!((len, cw), (3, 0x4));
}

#[test]
fn table_4_a_10_row_14_matches_spec() {
    // Row 14: length 4, codeword 0xc. `(y, z) = (1, 1)` since
    // idx = 1 * 13 + 1 = 14.
    let (len, cw) = hcod9_encode(14).unwrap();
    assert_eq!((len, cw), (4, 0xc));
}

#[test]
fn table_4_a_10_row_142_matches_spec() {
    // Row 142: length 15, codeword 0x7ffc — the first of the four
    // rows that reach the table's maximum codeword length.
    let (len, cw) = hcod9_encode(142).unwrap();
    assert_eq!((len, cw), (15, 0x7ffc));
}

#[test]
fn table_4_a_10_row_154_matches_spec() {
    // Row 154: length 15, codeword 0x7ffd.
    let (len, cw) = hcod9_encode(154).unwrap();
    assert_eq!((len, cw), (15, 0x7ffd));
}

#[test]
fn table_4_a_10_row_155_matches_spec() {
    // Row 155: length 15, codeword 0x7ffe.
    let (len, cw) = hcod9_encode(155).unwrap();
    assert_eq!((len, cw), (15, 0x7ffe));
}

#[test]
fn table_4_a_10_row_168_matches_spec() {
    // Row 168: length 15, codeword 0x7fff — the far corner
    // `(12, 12)` of the unsigned `13 × 13` pair lattice.
    let (len, cw) = hcod9_encode(168).unwrap();
    assert_eq!((len, cw), (15, 0x7fff));
}

// =============================================================================
// Table 4.95 row 9 ↔ Table 4.A.10 size cross-check.
// =============================================================================

#[test]
fn table_4_95_row_9_declares_unsigned_dim_2_lav_12() {
    let row = table_4_95(9).unwrap();
    assert_eq!(row.unsigned, Some(true));
    assert_eq!(row.dimension, Some(2));
    assert_eq!(row.lav, Some(12));
    assert_eq!(row.huffman_table, Some(10));
    assert!(row.esc_threshold.is_none());
    // 169 entries follow from (LAV + 1)^dim = 13^2.
    assert_eq!(HCOD9_NUM_ENTRIES, 169);
}

// =============================================================================
// Full writer → reader round-trip for every legal index, with the
// §4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check threaded
// through `spectral_codebook::decode_index_to_tuple` /
// `spectral_codebook::encode_tuple_to_index`.
// =============================================================================

#[test]
fn hcod9_every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD9_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod9_write(&mut w, idx).unwrap();
        let (len, _) = hcod9_encode(idx).unwrap();
        let pad = (8 - (u32::from(len) % 8)) % 8;
        let mut w2 = w;
        if pad > 0 {
            w2.write_u32(0, pad);
        }
        let bytes = w2.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod9_decode(&mut br).unwrap();
        assert_eq!(got, idx, "round-trip mismatch at idx={}", idx);
        assert_eq!(
            br.bit_position(),
            u64::from(len),
            "bit consumption mismatch at idx={}",
            idx
        );
    }
}

#[test]
fn hcod9_every_index_translates_through_spectral_codebook_round_trip() {
    for idx in 0..HCOD9_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(9, idx).unwrap();
        // dim = 2: only the first two slots carry the pair.
        let y = tuple[0];
        let z = tuple[1];
        assert!((0..=12).contains(&y), "y out of range at idx={}", idx);
        assert!((0..=12).contains(&z), "z out of range at idx={}", idx);
        let back = encode_tuple_to_index(9, &[y, z]).unwrap();
        assert_eq!(back, idx, "tuple round-trip mismatch at idx={}", idx);
    }
}

#[test]
fn hcod9_zero_tuple_at_index_0_carries_1_bit_codeword() {
    // The §4.6.3.3 unsigned polynomial idx = y * 13 + z places
    // `(0, 0)` at index 0; Codebook 9 hands it the 1-bit `0`
    // codeword — the shortest possible slot.
    let tuple = decode_index_to_tuple(9, 0).unwrap();
    assert_eq!(tuple[0], 0);
    assert_eq!(tuple[1], 0);
    let (len, cw) = hcod9_encode(0).unwrap();
    assert_eq!((len, cw), (1, 0));
}

#[test]
fn hcod9_far_corner_lives_at_index_168() {
    // `(12, 12)` at index 12 * 13 + 12 = 168.
    let tuple = decode_index_to_tuple(9, 168).unwrap();
    assert_eq!(tuple[0], 12);
    assert_eq!(tuple[1], 12);
    let (len, cw) = hcod9_encode(168).unwrap();
    assert_eq!((len, cw), (15, 0x7fff));
}

// =============================================================================
// Sign-bit cross-check: Codebook 9 is unsigned, so the §4.6.3.3
// sign-bit suffix follows the Huffman codeword for every non-zero
// coefficient. The sign-bit count must match the count of non-zero
// pair coefficients for every legal index.
// =============================================================================

#[test]
fn hcod9_emits_one_sign_bit_per_nonzero_coefficient() {
    for idx in 0..HCOD9_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(9, idx).unwrap();
        // dim = 2: only the first two slots carry the pair.
        let pair = [tuple[0], tuple[1], 0, 0];
        let signs = derive_sign_bits(9, &pair).unwrap();
        let nonzero_count = pair[..2].iter().filter(|&&v| v != 0).count();
        assert_eq!(
            signs.len(),
            nonzero_count,
            "sign-bit count mismatch at idx={} pair={:?}",
            idx,
            &pair[..2]
        );
    }
}

#[test]
fn hcod9_zero_tuple_emits_zero_sign_bits() {
    // The zero-tuple `(0, 0)` has no non-zero coefficient — no
    // sign bits follow.
    let tuple = decode_index_to_tuple(9, 0).unwrap();
    let signs = derive_sign_bits(9, &tuple).unwrap();
    assert!(signs.is_empty());
}

#[test]
fn hcod9_max_tuple_emits_two_sign_bits() {
    // `(12, 12)` at index 168 — both coefficients are non-zero so
    // two sign bits follow the Huffman codeword.
    let tuple = decode_index_to_tuple(9, 168).unwrap();
    let signs = derive_sign_bits(9, &tuple).unwrap();
    assert_eq!(signs.len(), 2);
}

// =============================================================================
// Hand-pinned byte sequences — these lock the exact wire bits the
// decoder must accept.
// =============================================================================

#[test]
fn hcod9_write_index_0_then_align_yields_zero_byte() {
    // Index 0 → 1-bit `0`. Padding to a byte boundary with zeros
    // produces 0x00.
    let mut w = BitWriter::new();
    hcod9_write(&mut w, 0).unwrap();
    w.write_u32(0, 7); // pad to byte boundary
    assert_eq!(w.into_bytes(), vec![0x00]);
}

#[test]
fn hcod9_decode_full_15_bit_far_corner_codeword() {
    // Index 168 → 15-bit 0x7fff packed left-aligned: high byte =
    // 0xff (bits 14..7), low byte = (0x7f << 1) = 0xfe (bits 6..0
    // shifted into the top of the low byte).
    let bytes = [0xff, 0xfe];
    let mut br = BitReader::new(&bytes);
    let idx = hcod9_decode(&mut br).unwrap();
    assert_eq!(idx, 168);
    assert_eq!(br.bit_position(), 15u64);
}

// =============================================================================
// Rejection paths.
// =============================================================================

#[test]
fn hcod9_encode_rejects_indices_at_and_above_169() {
    for bad in [169u32, 170, 200, 1000, u32::MAX] {
        assert!(matches!(
            hcod9_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(9))
        ));
    }
}

#[test]
fn hcod9_decoder_returns_unexpected_end_on_truncation() {
    let bytes: [u8; 0] = [];
    let mut br = BitReader::new(&bytes);
    let err = hcod9_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

#[test]
fn hcod9_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD9_NUM_ENTRIES as u32 {
        let (len, _) = hcod9_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD9_MAX_LEN);
}

// =============================================================================
// Cross-check: Codebook 9 widens the unsigned pair universe from
// `8 × 8 = 64` entries (Codebooks 7 / 8 with `LAV = 7`) to
// `13 × 13 = 169` entries (`LAV = 12`). The encoder chooses Codebook
// 9 when a section's magnitude statistics need range Codebooks 7 / 8
// cannot reach without ESC.
// =============================================================================

#[test]
fn codebook_9_is_the_first_expanded_lav_unsigned_pair_book() {
    let row7 = table_4_95(7).unwrap();
    let row8 = table_4_95(8).unwrap();
    let row9 = table_4_95(9).unwrap();
    // 7, 8, 9 are all unsigned pair books.
    for row in [row7, row8, row9] {
        assert_eq!(row.unsigned, Some(true));
        assert_eq!(row.dimension, Some(2));
    }
    // Books 7 / 8 share LAV = 7; book 9 widens to LAV = 12.
    assert_eq!(row7.lav, Some(7));
    assert_eq!(row8.lav, Some(7));
    assert_eq!(row9.lav, Some(12));
    // Different Annex 4.A table for each book.
    assert_eq!(row7.huffman_table, Some(8));
    assert_eq!(row8.huffman_table, Some(9));
    assert_eq!(row9.huffman_table, Some(10));
}

#[test]
fn codebook_9_universe_is_169_distinct_unsigned_pairs() {
    // Every legal `(y, z)` pair with `y, z` in `0..=12` must map to a
    // distinct index, and every index must map back to a distinct
    // pair. This is the §4.6.3.3 bijection at dim-2 LAV-12.
    let mut seen = std::collections::HashSet::new();
    for y in 0..=12i32 {
        for z in 0..=12i32 {
            let idx = encode_tuple_to_index(9, &[y, z]).unwrap();
            assert!(idx < HCOD9_NUM_ENTRIES as u32);
            assert!(
                seen.insert(idx),
                "duplicate idx={} for tuple ({}, {})",
                idx,
                y,
                z
            );
            let back = decode_index_to_tuple(9, idx).unwrap();
            assert_eq!(back[0], y);
            assert_eq!(back[1], z);
        }
    }
    assert_eq!(seen.len(), HCOD9_NUM_ENTRIES);
}

#[test]
fn codebook_9_ceiling_is_5_bits_wider_than_codebook_8() {
    // Codebook 8's ceiling is 10 bits; Codebook 9's ceiling is 15
    // bits — a 5-bit jump that reflects the `169 / 64 ≈ 2.6×`
    // universe expansion.
    assert_eq!(HCOD8_MAX_LEN, 10);
    assert_eq!(HCOD9_MAX_LEN, 15);
    assert_eq!(HCOD9_MAX_LEN - HCOD8_MAX_LEN, 5);
}

#[test]
fn codebook_9_zero_tuple_shares_codebook_7_head_placement() {
    // Codebook 7 parks `(0, 0)` at index 0 with the 1-bit `0`
    // codeword; Codebook 9 does the same. Codebook 8 lifts the
    // zero-tuple off the 1-bit slot (5 bits at index 0) and migrates
    // the shortest codeword to the (1, 1) interior at index 9.
    let (l7, cw7) = hcod7_encode(0).unwrap();
    let (l8, cw8) = hcod8_encode(0).unwrap();
    let (l9, cw9) = hcod9_encode(0).unwrap();
    assert_eq!((l7, cw7), (1, 0));
    assert_eq!((l8, cw8), (5, 0xe));
    assert_eq!((l9, cw9), (1, 0));
}

// =============================================================================
// Per-row spec-PDF spot checks (Table 4.A.11)
// =============================================================================
//
// Each spot row is taken from ISO/IEC 14496-3:2009(E) §4.A.1 Table
// 4.A.11 verbatim. The full 169-row table-shape and Kraft completeness
// is covered by the unit-tests; this module locks the per-row
// hexadecimal column at integration-test scope.

#[test]
fn table_4_a_11_row_0_matches_spec() {
    // Row 0: length 6, codeword 0x22 — the §4.6.3.3 zero-tuple
    // `(0, 0)`. Unlike Codebook 9, Codebook 10 does NOT place the
    // 1-bit codeword here.
    let (len, cw) = hcod10_encode(0).unwrap();
    assert_eq!((len, cw), (6, 0x22));
}

#[test]
fn table_4_a_11_row_1_matches_spec() {
    // Row 1: length 5, codeword 0x8.
    let (len, cw) = hcod10_encode(1).unwrap();
    assert_eq!((len, cw), (5, 0x8));
}

#[test]
fn table_4_a_11_row_14_matches_spec() {
    // Row 14: length 4, codeword 0 — the shortest slot of Codebook
    // 10, sitting on the interior `(y, z) = (1, 1)` since
    // idx = 1 * 13 + 1 = 14.
    let (len, cw) = hcod10_encode(14).unwrap();
    assert_eq!((len, cw), (4, 0));
}

#[test]
fn table_4_a_11_row_15_matches_spec() {
    // Row 15: length 4, codeword 0x1. The second 4-bit row.
    let (len, cw) = hcod10_encode(15).unwrap();
    assert_eq!((len, cw), (4, 0x1));
}

#[test]
fn table_4_a_11_row_27_matches_spec() {
    // Row 27: length 4, codeword 0x2. The third (and last) 4-bit row.
    let (len, cw) = hcod10_encode(27).unwrap();
    assert_eq!((len, cw), (4, 0x2));
}

#[test]
fn table_4_a_11_row_12_matches_spec() {
    // Row 12: length 12, codeword 0xffd — the first of the eight
    // 12-bit rows.
    let (len, cw) = hcod10_encode(12).unwrap();
    assert_eq!((len, cw), (12, 0xffd));
}

#[test]
fn table_4_a_11_row_129_matches_spec() {
    // Row 129: length 12, codeword 0xffa.
    let (len, cw) = hcod10_encode(129).unwrap();
    assert_eq!((len, cw), (12, 0xffa));
}

#[test]
fn table_4_a_11_row_142_matches_spec() {
    // Row 142: length 12, codeword 0xff9.
    let (len, cw) = hcod10_encode(142).unwrap();
    assert_eq!((len, cw), (12, 0xff9));
}

#[test]
fn table_4_a_11_row_155_matches_spec() {
    // Row 155: length 12, codeword 0xffb.
    let (len, cw) = hcod10_encode(155).unwrap();
    assert_eq!((len, cw), (12, 0xffb));
}

#[test]
fn table_4_a_11_row_168_matches_spec() {
    // Row 168: length 12, codeword 0xfff — the far corner `(12, 12)`
    // of the unsigned `13 × 13` pair lattice.
    let (len, cw) = hcod10_encode(168).unwrap();
    assert_eq!((len, cw), (12, 0xfff));
}

// =============================================================================
// Table 4.95 row 10 ↔ Table 4.A.11 size cross-check.
// =============================================================================

#[test]
fn table_4_95_row_10_declares_unsigned_dim_2_lav_12() {
    let row = table_4_95(10).unwrap();
    assert_eq!(row.unsigned, Some(true));
    assert_eq!(row.dimension, Some(2));
    assert_eq!(row.lav, Some(12));
    assert_eq!(row.huffman_table, Some(11));
    assert!(row.esc_threshold.is_none());
    // 169 entries follow from (LAV + 1)^dim = 13^2.
    assert_eq!(HCOD10_NUM_ENTRIES, 169);
}

// =============================================================================
// Full writer → reader round-trip for every legal index, with the
// §4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check threaded
// through `spectral_codebook::decode_index_to_tuple` /
// `spectral_codebook::encode_tuple_to_index`.
// =============================================================================

#[test]
fn hcod10_every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD10_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod10_write(&mut w, idx).unwrap();
        let (len, _) = hcod10_encode(idx).unwrap();
        let pad = (8 - (u32::from(len) % 8)) % 8;
        let mut w2 = w;
        if pad > 0 {
            w2.write_u32(0, pad);
        }
        let bytes = w2.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod10_decode(&mut br).unwrap();
        assert_eq!(got, idx, "round-trip mismatch at idx={}", idx);
        assert_eq!(
            br.bit_position(),
            u64::from(len),
            "bit consumption mismatch at idx={}",
            idx
        );
    }
}

#[test]
fn hcod10_every_index_translates_through_spectral_codebook_round_trip() {
    for idx in 0..HCOD10_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(10, idx).unwrap();
        // dim = 2: only the first two slots carry the pair.
        let y = tuple[0];
        let z = tuple[1];
        assert!((0..=12).contains(&y), "y out of range at idx={}", idx);
        assert!((0..=12).contains(&z), "z out of range at idx={}", idx);
        let back = encode_tuple_to_index(10, &[y, z]).unwrap();
        assert_eq!(back, idx, "tuple round-trip mismatch at idx={}", idx);
    }
}

#[test]
fn hcod10_zero_tuple_at_index_0_carries_six_bit_codeword() {
    // The §4.6.3.3 unsigned polynomial idx = y * 13 + z places
    // `(0, 0)` at index 0; Codebook 10 hands it the 6-bit codeword
    // 0x22 — the zero-tuple is NOT in the shortest slot here.
    let tuple = decode_index_to_tuple(10, 0).unwrap();
    assert_eq!(tuple[0], 0);
    assert_eq!(tuple[1], 0);
    let (len, cw) = hcod10_encode(0).unwrap();
    assert_eq!((len, cw), (6, 0x22));
}

#[test]
fn hcod10_shortest_codeword_sits_on_interior_one_one_tuple() {
    // The shortest Codebook 10 codeword (4 bits, 0b0000) is at
    // index 14, which the §4.6.3.3 unsigned polynomial decodes as
    // the interior pair `(1, 1)` (1 * 13 + 1 = 14).
    let tuple = decode_index_to_tuple(10, 14).unwrap();
    assert_eq!(tuple[0], 1);
    assert_eq!(tuple[1], 1);
    let (len, cw) = hcod10_encode(14).unwrap();
    assert_eq!((len, cw), (4, 0));
}

#[test]
fn hcod10_far_corner_lives_at_index_168() {
    // `(12, 12)` at index 12 * 13 + 12 = 168.
    let tuple = decode_index_to_tuple(10, 168).unwrap();
    assert_eq!(tuple[0], 12);
    assert_eq!(tuple[1], 12);
    let (len, cw) = hcod10_encode(168).unwrap();
    assert_eq!((len, cw), (12, 0xfff));
}

// =============================================================================
// Sign-bit cross-check: Codebook 10 is unsigned, so the §4.6.3.3
// sign-bit suffix follows the Huffman codeword for every non-zero
// coefficient. The sign-bit count must match the count of non-zero
// pair coefficients for every legal index.
// =============================================================================

#[test]
fn hcod10_emits_one_sign_bit_per_nonzero_coefficient() {
    for idx in 0..HCOD10_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(10, idx).unwrap();
        // dim = 2: only the first two slots carry the pair.
        let pair = [tuple[0], tuple[1], 0, 0];
        let signs = derive_sign_bits(10, &pair).unwrap();
        let nonzero_count = pair[..2].iter().filter(|&&v| v != 0).count();
        assert_eq!(
            signs.len(),
            nonzero_count,
            "sign-bit count mismatch at idx={} pair={:?}",
            idx,
            &pair[..2]
        );
    }
}

#[test]
fn hcod10_zero_tuple_emits_zero_sign_bits() {
    // The zero-tuple `(0, 0)` has no non-zero coefficient — no
    // sign bits follow.
    let tuple = decode_index_to_tuple(10, 0).unwrap();
    let signs = derive_sign_bits(10, &tuple).unwrap();
    assert!(signs.is_empty());
}

#[test]
fn hcod10_max_tuple_emits_two_sign_bits() {
    // `(12, 12)` at index 168 — both coefficients are non-zero so
    // two sign bits follow the Huffman codeword.
    let tuple = decode_index_to_tuple(10, 168).unwrap();
    let signs = derive_sign_bits(10, &tuple).unwrap();
    assert_eq!(signs.len(), 2);
}

// =============================================================================
// Hand-pinned byte sequences — these lock the exact wire bits the
// decoder must accept.
// =============================================================================

#[test]
fn hcod10_write_index_14_then_align_yields_low_nibble_zero() {
    // Index 14 → 4-bit `0`. Padding to a byte boundary with zeros
    // produces 0x00.
    let mut w = BitWriter::new();
    hcod10_write(&mut w, 14).unwrap();
    w.write_u32(0, 4); // pad to byte boundary
    assert_eq!(w.into_bytes(), vec![0x00]);
}

#[test]
fn hcod10_decode_full_12_bit_far_corner_codeword() {
    // Index 168 → 12-bit 0xfff packed left-aligned: high byte =
    // 0xff (bits 11..4), low byte = (0xf << 4) = 0xf0 (bits 3..0
    // shifted into the top of the low byte).
    let bytes = [0xff, 0xf0];
    let mut br = BitReader::new(&bytes);
    let idx = hcod10_decode(&mut br).unwrap();
    assert_eq!(idx, 168);
    assert_eq!(br.bit_position(), 12u64);
}

// =============================================================================
// Rejection paths.
// =============================================================================

#[test]
fn hcod10_encode_rejects_indices_at_and_above_169() {
    for bad in [169u32, 170, 200, 1000, u32::MAX] {
        assert!(matches!(
            hcod10_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(10))
        ));
    }
}

#[test]
fn hcod10_decoder_returns_unexpected_end_on_truncation() {
    let bytes: [u8; 0] = [];
    let mut br = BitReader::new(&bytes);
    let err = hcod10_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

#[test]
fn hcod10_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD10_NUM_ENTRIES as u32 {
        let (len, _) = hcod10_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD10_MAX_LEN);
}

// =============================================================================
// Cross-check: Codebook 10 shares Codebook 9's `13 × 13 = 169`-entry
// unsigned pair universe (both Table 4.95 rows declare LAV = 12)
// but uses a different codeword distribution. Codebook 10 trades the
// 1-bit head Codebook 9 gives the zero-tuple for a 6-bit codeword
// and migrates the 4-bit shortest slot onto the interior `(1, 1)`
// tuple at index 14 — the same head-displacement pattern Codebook 8
// uses relative to Codebook 7 but at the wider LAV = 12 universe.
// =============================================================================

#[test]
fn codebook_10_is_the_second_expanded_lav_unsigned_pair_book() {
    let row9 = table_4_95(9).unwrap();
    let row10 = table_4_95(10).unwrap();
    // Both books are unsigned pair books with LAV = 12.
    for row in [row9, row10] {
        assert_eq!(row.unsigned, Some(true));
        assert_eq!(row.dimension, Some(2));
        assert_eq!(row.lav, Some(12));
    }
    // Different Annex 4.A table for each book.
    assert_eq!(row9.huffman_table, Some(10));
    assert_eq!(row10.huffman_table, Some(11));
}

#[test]
fn codebook_10_universe_is_169_distinct_unsigned_pairs() {
    // Every legal `(y, z)` pair with `y, z` in `0..=12` must map to a
    // distinct index, and every index must map back to a distinct
    // pair. This is the §4.6.3.3 bijection at dim-2 LAV-12, same as
    // Codebook 9.
    let mut seen = std::collections::HashSet::new();
    for y in 0..=12i32 {
        for z in 0..=12i32 {
            let idx = encode_tuple_to_index(10, &[y, z]).unwrap();
            assert!(idx < HCOD10_NUM_ENTRIES as u32);
            assert!(
                seen.insert(idx),
                "duplicate idx={} for tuple ({}, {})",
                idx,
                y,
                z
            );
            let back = decode_index_to_tuple(10, idx).unwrap();
            assert_eq!(back[0], y);
            assert_eq!(back[1], z);
        }
    }
    assert_eq!(seen.len(), HCOD10_NUM_ENTRIES);
}

#[test]
fn codebook_10_ceiling_is_3_bits_below_codebook_9() {
    // Codebook 9's ceiling is 15 bits; Codebook 10's ceiling is
    // 12 bits — a 3-bit pull-down reflecting the flatter
    // codeword distribution.
    assert_eq!(HCOD9_MAX_LEN, 15);
    assert_eq!(HCOD10_MAX_LEN, 12);
    assert_eq!(HCOD9_MAX_LEN - HCOD10_MAX_LEN, 3);
}

#[test]
fn codebook_10_zero_tuple_displaced_from_codebook_9_head_placement() {
    // Codebook 9 parks `(0, 0)` at index 0 with the 1-bit `0`
    // codeword; Codebook 10 keeps the same index but bumps the
    // codeword to 6 bits and lifts the 4-bit shortest slot onto
    // the interior `(1, 1)` tuple at index 14 — mirroring
    // Codebook 8's relationship to Codebook 7 at LAV = 7.
    let (l9, cw9) = hcod9_encode(0).unwrap();
    let (l10_0, cw10_0) = hcod10_encode(0).unwrap();
    let (l10_14, cw10_14) = hcod10_encode(14).unwrap();
    assert_eq!((l9, cw9), (1, 0));
    assert_eq!((l10_0, cw10_0), (6, 0x22));
    assert_eq!((l10_14, cw10_14), (4, 0));
}

// =============================================================================
// Per-row spec-PDF spot checks (Table 4.A.12).
// =============================================================================

#[test]
fn table_4_a_12_row_0_matches_spec() {
    // Row 0: length 4, codeword 0 — the §4.6.3.3 zero-tuple `(0, 0)`.
    // Codebook 11 parks the zero-tuple in the shortest 4-bit slot,
    // matching the head placement of Codebooks 7 and 9 but at the
    // wider 4-bit floor required by the 289-entry universe.
    let (len, cw) = hcod11_encode(0).unwrap();
    assert_eq!((len, cw), (4, 0));
}

#[test]
fn table_4_a_12_row_1_matches_spec() {
    // Row 1: length 5, codeword 0x6.
    let (len, cw) = hcod11_encode(1).unwrap();
    assert_eq!((len, cw), (5, 0x6));
}

#[test]
fn table_4_a_12_row_12_matches_spec() {
    // Row 12: length 12, codeword 0xffb — the first of the six
    // 12-bit rows.
    let (len, cw) = hcod11_encode(12).unwrap();
    assert_eq!((len, cw), (12, 0xffb));
}

#[test]
fn table_4_a_12_row_14_matches_spec() {
    let (len, cw) = hcod11_encode(14).unwrap();
    assert_eq!((len, cw), (12, 0xffa));
}

#[test]
fn table_4_a_12_row_15_matches_spec() {
    let (len, cw) = hcod11_encode(15).unwrap();
    assert_eq!((len, cw), (12, 0xffe));
}

#[test]
fn table_4_a_12_row_16_matches_spec() {
    // Row 16: length 10, codeword 0x38e — the §4.6.3.3 half-ESC
    // tuple (0, 16) (y = 0, z at the escape flag).
    let (len, cw) = hcod11_encode(16).unwrap();
    assert_eq!((len, cw), (10, 0x38e));
}

#[test]
fn table_4_a_12_row_18_matches_spec() {
    // Row 18: length 4, codeword 0x1 — the interior pair (1, 1)
    // (the second 4-bit slot).
    let (len, cw) = hcod11_encode(18).unwrap();
    assert_eq!((len, cw), (4, 0x1));
}

#[test]
fn table_4_a_12_row_255_matches_spec() {
    let (len, cw) = hcod11_encode(255).unwrap();
    assert_eq!((len, cw), (12, 0xffd));
}

#[test]
fn table_4_a_12_row_269_matches_spec() {
    let (len, cw) = hcod11_encode(269).unwrap();
    assert_eq!((len, cw), (12, 0xffc));
}

#[test]
fn table_4_a_12_row_270_matches_spec() {
    let (len, cw) = hcod11_encode(270).unwrap();
    assert_eq!((len, cw), (12, 0xfff));
}

#[test]
fn table_4_a_12_row_272_matches_spec() {
    // Row 272: length 9, codeword 0x1c2 — the §4.6.3.3 half-ESC
    // tuple (16, 0).
    let (len, cw) = hcod11_encode(272).unwrap();
    assert_eq!((len, cw), (9, 0x1c2));
}

#[test]
fn table_4_a_12_row_288_matches_spec() {
    // Row 288: length 5, codeword 0x4 — the far corner (16, 16) of
    // the 17×17 lattice. Both coefficients carry the ESC flag.
    let (len, cw) = hcod11_encode(288).unwrap();
    assert_eq!((len, cw), (5, 0x4));
}

// =============================================================================
// Table 4.95 row 11 ↔ Table 4.A.12 size cross-check.
// =============================================================================

#[test]
fn table_4_95_row_11_declares_unsigned_dim_2_lav_16_esc_8191() {
    let row = table_4_95(11).unwrap();
    assert_eq!(row.unsigned, Some(true));
    assert_eq!(row.dimension, Some(2));
    assert_eq!(row.lav, Some(16));
    assert_eq!(row.huffman_table, Some(12));
    assert_eq!(row.esc_threshold, Some(8191));
    assert!(row.has_esc());
    // 289 entries follow from (LAV + 1)^dim = 17^2.
    assert_eq!(HCOD11_NUM_ENTRIES, 289);
}

// =============================================================================
// Full writer → reader round-trip for every legal index, with the
// §4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check threaded
// through `spectral_codebook::decode_index_to_tuple` /
// `spectral_codebook::encode_tuple_to_index`.
// =============================================================================

#[test]
fn hcod11_every_index_round_trips_writer_to_reader() {
    for idx in 0..HCOD11_NUM_ENTRIES as u32 {
        let mut w = BitWriter::new();
        hcod11_write(&mut w, idx).unwrap();
        let (len, _) = hcod11_encode(idx).unwrap();
        let pad = (8 - (u32::from(len) % 8)) % 8;
        let mut w2 = w;
        if pad > 0 {
            w2.write_u32(0, pad);
        }
        let bytes = w2.into_bytes();
        let mut br = BitReader::new(&bytes);
        let got = hcod11_decode(&mut br).unwrap();
        assert_eq!(got, idx, "round-trip mismatch at idx={idx}");
        assert_eq!(
            br.bit_position(),
            u64::from(len),
            "bit consumption mismatch at idx={idx}",
        );
    }
}

#[test]
fn hcod11_every_index_translates_through_spectral_codebook_round_trip() {
    for idx in 0..HCOD11_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(11, idx).unwrap();
        // dim = 2: only the first two slots carry the pair.
        let y = tuple[0];
        let z = tuple[1];
        assert!((0..=16).contains(&y), "y out of range at idx={idx}");
        assert!((0..=16).contains(&z), "z out of range at idx={idx}");
        let back = encode_tuple_to_index(11, &[y, z]).unwrap();
        assert_eq!(back, idx, "tuple round-trip mismatch at idx={idx}");
    }
}

#[test]
fn hcod11_zero_tuple_at_index_0_carries_four_bit_codeword() {
    // The §4.6.3.3 unsigned polynomial idx = y * 17 + z places
    // `(0, 0)` at index 0; Codebook 11 hands it the shortest 4-bit
    // codeword `0b0000`. Matches the head-placement Codebooks 7
    // and 9 use, modulo the 4-bit floor.
    let tuple = decode_index_to_tuple(11, 0).unwrap();
    assert_eq!(tuple[0], 0);
    assert_eq!(tuple[1], 0);
    let (len, cw) = hcod11_encode(0).unwrap();
    assert_eq!((len, cw), (4, 0));
}

#[test]
fn hcod11_interior_one_one_at_index_18_carries_four_bit_codeword() {
    // 1 * 17 + 1 = 18. The interior (1, 1) tuple shares the 4-bit
    // floor with the zero-tuple, with codeword 0b0001.
    let tuple = decode_index_to_tuple(11, 18).unwrap();
    assert_eq!(tuple[0], 1);
    assert_eq!(tuple[1], 1);
    let (len, cw) = hcod11_encode(18).unwrap();
    assert_eq!((len, cw), (4, 0x1));
}

#[test]
fn hcod11_far_corner_at_index_288_carries_five_bit_codeword() {
    // (16, 16) at 16 * 17 + 16 = 288.
    let tuple = decode_index_to_tuple(11, 288).unwrap();
    assert_eq!(tuple[0], 16);
    assert_eq!(tuple[1], 16);
    let (len, cw) = hcod11_encode(288).unwrap();
    assert_eq!((len, cw), (5, 0x4));
}

#[test]
fn hcod11_half_esc_indices_decode_to_half_esc_tuples() {
    // (0, 16) at idx 16; (16, 0) at idx 272.
    let t16 = decode_index_to_tuple(11, 16).unwrap();
    assert_eq!(t16[0], 0);
    assert_eq!(t16[1], 16);
    let t272 = decode_index_to_tuple(11, 272).unwrap();
    assert_eq!(t272[0], 16);
    assert_eq!(t272[1], 0);
}

// =============================================================================
// Sign-bit cross-check: Codebook 11 is unsigned, so the §4.6.3.3
// sign-bit suffix follows the Huffman codeword for every non-zero
// coefficient. The sign-bit count must match the count of non-zero
// pair coefficients for every legal index (the §4.6.3.3 escape flag
// at value 16 is non-zero and so produces a sign bit too — the bit
// signs the ESC-reconstructed magnitude rather than the literal 16).
// =============================================================================

#[test]
fn hcod11_emits_one_sign_bit_per_nonzero_coefficient() {
    for idx in 0..HCOD11_NUM_ENTRIES as u32 {
        let tuple = decode_index_to_tuple(11, idx).unwrap();
        let pair = [tuple[0], tuple[1], 0, 0];
        let signs = derive_sign_bits(11, &pair).unwrap();
        let nonzero_count = pair[..2].iter().filter(|&&v| v != 0).count();
        assert_eq!(
            signs.len(),
            nonzero_count,
            "sign-bit count mismatch at idx={idx} pair={:?}",
            &pair[..2],
        );
    }
}

#[test]
fn hcod11_zero_tuple_emits_zero_sign_bits() {
    let tuple = decode_index_to_tuple(11, 0).unwrap();
    let signs = derive_sign_bits(11, &tuple).unwrap();
    assert!(signs.is_empty());
}

#[test]
fn hcod11_far_corner_emits_two_sign_bits() {
    // (16, 16) — both coefficients are non-zero (ESC-flagged) so
    // two sign bits follow the Huffman codeword, ahead of the two
    // escape sequences.
    let tuple = decode_index_to_tuple(11, 288).unwrap();
    let signs = derive_sign_bits(11, &tuple).unwrap();
    assert_eq!(signs.len(), 2);
}

// =============================================================================
// Hand-pinned byte sequences — these lock the exact wire bits the
// decoder must accept.
// =============================================================================

#[test]
fn hcod11_write_index_0_then_align_yields_low_nibble_zero() {
    // Index 0 → 4-bit `0`. Padding to a byte boundary with zeros
    // produces 0x00.
    let mut w = BitWriter::new();
    hcod11_write(&mut w, 0).unwrap();
    w.write_u32(0, 4); // pad to byte boundary
    assert_eq!(w.into_bytes(), vec![0x00]);
}

#[test]
fn hcod11_decode_full_12_bit_far_codeword_at_index_270() {
    // Index 270 → 12-bit 0xfff packed left-aligned: high byte =
    // 0xff (bits 11..4), low byte = (0xf << 4) = 0xf0 (bits 3..0
    // in the high nibble of the low byte).
    let bytes = [0xffu8, 0xf0];
    let mut br = BitReader::new(&bytes);
    let idx = hcod11_decode(&mut br).unwrap();
    assert_eq!(idx, 270);
    assert_eq!(br.bit_position(), 12u64);
}

// =============================================================================
// Rejection paths.
// =============================================================================

#[test]
fn hcod11_encode_rejects_indices_at_and_above_289() {
    for bad in [289u32, 290, 300, 1000, u32::MAX] {
        assert!(matches!(
            hcod11_encode(bad),
            Err(Error::SpectralCodebookIndexOutOfRange(11))
        ));
    }
}

#[test]
fn hcod11_decoder_returns_unexpected_end_on_truncation() {
    let bytes: [u8; 0] = [];
    let mut br = BitReader::new(&bytes);
    let err = hcod11_decode(&mut br).unwrap_err();
    assert_eq!(err, Error::UnexpectedEnd);
}

#[test]
fn hcod11_max_len_constant_matches_table_data() {
    let mut observed_max = 0u32;
    for idx in 0..HCOD11_NUM_ENTRIES as u32 {
        let (len, _) = hcod11_encode(idx).unwrap();
        observed_max = observed_max.max(u32::from(len));
    }
    assert_eq!(observed_max, HCOD11_MAX_LEN);
}

// =============================================================================
// Cross-check against Codebook 10. Codebook 10 is the second
// `LAV = 12` unsigned pair (169 entries, 12-bit ceiling); Codebook
// 11 widens the universe to `LAV = 16` (289 entries, also a 12-bit
// ceiling). The codeword ceiling is unchanged because Codebook 11
// pushes its tail distribution out of the Huffman table and into
// the §4.6.3.3 ESC sequence.
// =============================================================================

#[test]
fn codebook_11_universe_is_289_distinct_unsigned_pairs() {
    let mut seen = std::collections::HashSet::new();
    for y in 0..=16i32 {
        for z in 0..=16i32 {
            let idx = encode_tuple_to_index(11, &[y, z]).unwrap();
            assert!(idx < HCOD11_NUM_ENTRIES as u32);
            assert!(seen.insert(idx), "duplicate idx={idx} for tuple ({y}, {z})",);
            let back = decode_index_to_tuple(11, idx).unwrap();
            assert_eq!(back[0], y);
            assert_eq!(back[1], z);
        }
    }
    assert_eq!(seen.len(), HCOD11_NUM_ENTRIES);
}

#[test]
fn codebook_11_ceiling_matches_codebook_10_ceiling() {
    assert_eq!(HCOD10_MAX_LEN, HCOD11_MAX_LEN);
    assert_eq!(HCOD11_MAX_LEN, 12);
}

#[test]
fn codebook_11_is_the_only_esc_spectrum_book() {
    let row10 = table_4_95(10).unwrap();
    let row11 = table_4_95(11).unwrap();
    assert!(!row10.has_esc());
    assert!(row11.has_esc());
    assert_eq!(row11.esc_threshold, Some(8191));
}

#[test]
fn codebook_11_universe_doubles_codebook_10_universe_minus_57() {
    // 289 - 169 = 120 extra rows. The 120 extras are the ESC-border
    // rows where y == 16 or z == 16: that's exactly
    // 17 + 17 - 1 = 33 rows per row pair, computed across the
    // expanded lattice. Concrete count: rows with y in 13..=16 OR
    // z in 13..=16 in the wider 17×17 universe minus the 0 rows
    // those positions occupy in the smaller 13×13.
    assert_eq!(HCOD11_NUM_ENTRIES - HCOD10_NUM_ENTRIES, 120);
}

// =============================================================================
// Cross-check: every legal §4.6.3.3 pair `(y, z)` with `y, z` in
// `0..=15` (the in-band magnitudes, no ESC) has a unique Huffman
// codeword. The pairs with `y == 16` or `z == 16` are the half-ESC
// and full-ESC tuples — they too have a unique Huffman codeword,
// but the wire layout extends with one or two `escape_sequence`s.
// =============================================================================

#[test]
fn hcod11_in_band_indices_are_disjoint_from_esc_border_indices() {
    use std::collections::HashSet;
    let mut in_band: HashSet<u32> = HashSet::new();
    let mut esc_border: HashSet<u32> = HashSet::new();
    for y in 0..=16i32 {
        for z in 0..=16i32 {
            let idx = encode_tuple_to_index(11, &[y, z]).unwrap();
            if y == 16 || z == 16 {
                assert!(esc_border.insert(idx));
            } else {
                assert!(in_band.insert(idx));
            }
        }
    }
    assert!(in_band.is_disjoint(&esc_border));
    assert_eq!(in_band.len(), 16 * 16); // 256 in-band pairs
    assert_eq!(esc_border.len(), 17 * 17 - 16 * 16); // 33 ESC-border pairs
    assert_eq!(in_band.len() + esc_border.len(), HCOD11_NUM_ENTRIES);
}

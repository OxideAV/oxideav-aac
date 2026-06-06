//! Integration tests for `oxideav_aac::spectrum_huffman`.
//!
//! Cross-checks Codebooks 1 (Table 4.A.2), 2 (Table 4.A.3), 3
//! (Table 4.A.4), 4 (Table 4.A.5), 5 (Table 4.A.6), 6 (Table 4.A.7),
//! and 7 (Table 4.A.8) wire round-trip against the §4.6.3.3 index ↔
//! n-tuple translation already covered by
//! [`oxideav_aac::spectral_codebook`].

use oxideav_aac::{
    spectral_codebook::{
        decode_index_to_tuple, derive_sign_bits, encode_tuple_to_index, table_4_95,
    },
    spectrum_huffman::{
        hcod1_decode, hcod1_encode, hcod1_write, hcod2_decode, hcod2_encode, hcod2_write,
        hcod3_decode, hcod3_encode, hcod3_write, hcod4_decode, hcod4_encode, hcod4_write,
        hcod5_decode, hcod5_encode, hcod5_write, hcod6_decode, hcod6_encode, hcod6_write,
        hcod7_decode, hcod7_encode, hcod7_write, HCOD1_MAX_LEN, HCOD1_NUM_ENTRIES, HCOD2_MAX_LEN,
        HCOD2_NUM_ENTRIES, HCOD3_MAX_LEN, HCOD3_NUM_ENTRIES, HCOD4_MAX_LEN, HCOD4_NUM_ENTRIES,
        HCOD5_MAX_LEN, HCOD5_NUM_ENTRIES, HCOD6_MAX_LEN, HCOD6_NUM_ENTRIES, HCOD7_MAX_LEN,
        HCOD7_NUM_ENTRIES,
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

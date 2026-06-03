//! Integration tests for `oxideav_aac::spectrum_huffman`.
//!
//! Cross-checks Codebooks 1 (Table 4.A.2) and 2 (Table 4.A.3) wire
//! round-trip against the §4.6.3.3 index ↔ n-tuple translation
//! already covered by [`oxideav_aac::spectral_codebook`].

use oxideav_aac::{
    spectral_codebook::{decode_index_to_tuple, encode_tuple_to_index, table_4_95},
    spectrum_huffman::{
        hcod1_decode, hcod1_encode, hcod1_write, hcod2_decode, hcod2_encode, hcod2_write,
        HCOD1_MAX_LEN, HCOD1_NUM_ENTRIES, HCOD2_MAX_LEN, HCOD2_NUM_ENTRIES,
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

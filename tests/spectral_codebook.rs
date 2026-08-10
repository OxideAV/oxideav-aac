//! Integration tests for [`oxideav_aac::spectral_codebook`].
//!
//! Covers ISO/IEC 14496-3 §4.6.3 / Table 4.95 Spectrum Huffman
//! codebook parameter accessors, the §4.6.3.3 index → n-tuple
//! translation, the §4.6.3.3 sign-bit fix-up, and the §4.6.3.3 ESC
//! sequence handling.

use oxideav_aac::spectral_codebook::{
    apply_sign_bits, decode_esc_value, decode_index_to_tuple, derive_sign_bits, encode_esc_value,
    encode_tuple_to_index, table_4_95, Table495Row, MAX_QUANT, TABLE_4_95,
};
use oxideav_aac::Error;

// ---------------------------------------------------------------------
// Table 4.95 row-by-row exact-match
// ---------------------------------------------------------------------

#[test]
fn table_4_95_row_count_is_32() {
    assert_eq!(TABLE_4_95.len(), 32);
}

#[test]
fn table_4_95_row_0_is_zero_hcb() {
    let row = TABLE_4_95[0];
    assert_eq!(row.unsigned, None);
    assert_eq!(row.dimension, None);
    assert_eq!(row.lav, Some(0));
    assert_eq!(row.esc_threshold, None);
    assert_eq!(row.huffman_table, None);
}

#[test]
fn table_4_95_quad_books_1_through_4() {
    // Books 1, 2: signed (unsigned_cb = 0), dim 4, LAV 1.
    for cb in [1, 2] {
        let row = TABLE_4_95[cb];
        assert_eq!(row.unsigned, Some(false), "cb {}", cb);
        assert_eq!(row.dimension, Some(4), "cb {}", cb);
        assert_eq!(row.lav, Some(1), "cb {}", cb);
        assert_eq!(row.esc_threshold, None, "cb {}", cb);
    }
    // Books 3, 4: unsigned, dim 4, LAV 2.
    for cb in [3, 4] {
        let row = TABLE_4_95[cb];
        assert_eq!(row.unsigned, Some(true), "cb {}", cb);
        assert_eq!(row.dimension, Some(4), "cb {}", cb);
        assert_eq!(row.lav, Some(2), "cb {}", cb);
        assert_eq!(row.esc_threshold, None, "cb {}", cb);
    }
}

#[test]
fn table_4_95_pair_books_5_through_10() {
    // Books 5, 6: signed (unsigned_cb = 0), dim 2, LAV 4.
    for cb in [5, 6] {
        let row = TABLE_4_95[cb];
        assert_eq!(row.unsigned, Some(false));
        assert_eq!(row.dimension, Some(2));
        assert_eq!(row.lav, Some(4));
        assert_eq!(row.esc_threshold, None);
    }
    // Books 7, 8: unsigned, dim 2, LAV 7.
    for cb in [7, 8] {
        let row = TABLE_4_95[cb];
        assert_eq!(row.unsigned, Some(true));
        assert_eq!(row.dimension, Some(2));
        assert_eq!(row.lav, Some(7));
        assert_eq!(row.esc_threshold, None);
    }
    // Books 9, 10: unsigned, dim 2, LAV 12.
    for cb in [9, 10] {
        let row = TABLE_4_95[cb];
        assert_eq!(row.unsigned, Some(true));
        assert_eq!(row.dimension, Some(2));
        assert_eq!(row.lav, Some(12));
        assert_eq!(row.esc_threshold, None);
    }
}

#[test]
fn table_4_95_row_11_is_esc_book() {
    let row = TABLE_4_95[11];
    assert_eq!(row.unsigned, Some(true));
    assert_eq!(row.dimension, Some(2));
    assert_eq!(row.lav, Some(16));
    assert_eq!(row.esc_threshold, Some(8191));
    assert_eq!(row.huffman_table, Some(12));
}

#[test]
fn table_4_95_rows_12_through_15_are_nonspectral() {
    for (cb, row) in TABLE_4_95.iter().enumerate().take(16).skip(12) {
        assert_eq!(row.unsigned, None, "cb {}", cb);
        assert_eq!(row.dimension, None, "cb {}", cb);
        assert_eq!(row.lav, None, "cb {}", cb);
        assert_eq!(row.esc_threshold, None, "cb {}", cb);
        assert_eq!(row.huffman_table, None, "cb {}", cb);
    }
}

#[test]
fn table_4_95_extension_books_16_through_31_have_increasing_esc_thresholds() {
    let expected = [
        15u32, 31, 47, 63, 95, 127, 159, 191, 223, 255, 319, 383, 511, 767, 1023, 2047,
    ];
    for (i, &thr) in expected.iter().enumerate() {
        let cb = 16 + i;
        let row = TABLE_4_95[cb];
        assert_eq!(row.unsigned, Some(true), "cb {}", cb);
        assert_eq!(row.dimension, Some(2), "cb {}", cb);
        assert_eq!(row.lav, Some(16), "cb {}", cb);
        assert_eq!(row.esc_threshold, Some(thr), "cb {}", cb);
        assert_eq!(row.huffman_table, Some(12), "cb {}", cb);
    }
}

#[test]
fn table_4_95_accessor_rejects_out_of_range_codebook() {
    assert_eq!(
        table_4_95(32).unwrap_err(),
        Error::SpectralCodebookOutOfRange(32)
    );
    assert_eq!(
        table_4_95(255).unwrap_err(),
        Error::SpectralCodebookOutOfRange(255)
    );
}

#[test]
fn table_4_95_accessor_returns_row_for_legal_codebook() {
    let row = table_4_95(7).unwrap();
    assert_eq!(row.dimension, Some(2));
    assert_eq!(row.lav, Some(7));
    assert_eq!(row.unsigned, Some(true));
}

// ---------------------------------------------------------------------
// §4.6.3.3 index → tuple translation
// ---------------------------------------------------------------------

#[test]
fn decode_index_to_tuple_codebook_1_quad_signed_lav1() {
    // Codebook 1: signed, dim 4, LAV 1. modulus = 2*1+1 = 3, offset = 1.
    // idx = 0 should decode to all-negatives at -lav, i.e. (-1, -1, -1, -1).
    let out = decode_index_to_tuple(1, 0).unwrap();
    assert_eq!(out, [-1, -1, -1, -1]);
    // idx = mod^3 + mod^2 + mod + 1 + offset adjustments — round-trip path.
    let centre = 1 + 3 + 9 + 27; // idx that yields (0, 0, 0, 0)
    let out = decode_index_to_tuple(1, centre).unwrap();
    assert_eq!(out, [0, 0, 0, 0]);
    // idx = mod^4 - 1 = 80: should be (+1, +1, +1, +1).
    let out = decode_index_to_tuple(1, 80).unwrap();
    assert_eq!(out, [1, 1, 1, 1]);
}

#[test]
fn decode_index_to_tuple_codebook_3_quad_unsigned_lav2() {
    // Codebook 3: unsigned, dim 4, LAV 2. modulus = 3, offset = 0.
    // idx = 0 → (0, 0, 0, 0); idx = mod^4 - 1 = 80 → (2, 2, 2, 2).
    let out = decode_index_to_tuple(3, 0).unwrap();
    assert_eq!(out, [0, 0, 0, 0]);
    let out = decode_index_to_tuple(3, 80).unwrap();
    assert_eq!(out, [2, 2, 2, 2]);
    // idx = 1 → (0, 0, 0, 1).
    let out = decode_index_to_tuple(3, 1).unwrap();
    assert_eq!(out, [0, 0, 0, 1]);
    // idx = mod = 3 → (0, 0, 1, 0).
    let out = decode_index_to_tuple(3, 3).unwrap();
    assert_eq!(out, [0, 0, 1, 0]);
    // idx = mod^3 = 27 → (1, 0, 0, 0).
    let out = decode_index_to_tuple(3, 27).unwrap();
    assert_eq!(out, [1, 0, 0, 0]);
}

#[test]
fn decode_index_to_tuple_codebook_5_pair_signed_lav4() {
    // Codebook 5: signed, dim 2, LAV 4. modulus = 9, offset = 4.
    // idx = 0 → (-4, -4); idx = mod^2 - 1 = 80 → (+4, +4).
    let out = decode_index_to_tuple(5, 0).unwrap();
    assert_eq!(out[0..2], [-4, -4]);
    let out = decode_index_to_tuple(5, 80).unwrap();
    assert_eq!(out[0..2], [4, 4]);
    // idx = 4 + 4*9 = 40 → (0, 0) (centre).
    let out = decode_index_to_tuple(5, 40).unwrap();
    assert_eq!(out[0..2], [0, 0]);
}

#[test]
fn decode_index_to_tuple_codebook_7_pair_unsigned_lav7() {
    // Codebook 7: unsigned, dim 2, LAV 7. modulus = 8, offset = 0.
    // idx = 0 → (0, 0); idx = mod^2 - 1 = 63 → (7, 7).
    let out = decode_index_to_tuple(7, 0).unwrap();
    assert_eq!(out[0..2], [0, 0]);
    let out = decode_index_to_tuple(7, 63).unwrap();
    assert_eq!(out[0..2], [7, 7]);
    // idx = 1 → (0, 1); idx = 8 → (1, 0).
    assert_eq!(decode_index_to_tuple(7, 1).unwrap()[0..2], [0, 1]);
    assert_eq!(decode_index_to_tuple(7, 8).unwrap()[0..2], [1, 0]);
}

#[test]
fn decode_index_to_tuple_codebook_11_esc_unsigned_lav16() {
    // Codebook 11: unsigned, dim 2, LAV 16. modulus = 17.
    let out = decode_index_to_tuple(11, 0).unwrap();
    assert_eq!(out[0..2], [0, 0]);
    let out = decode_index_to_tuple(11, 17 * 17 - 1).unwrap();
    assert_eq!(out[0..2], [16, 16]);
    // The 16-magnitude is the LAV cap; an actual stream would
    // follow up with the ESC sequence for any value at that cap.
    // The index translation itself stops at the cap.
}

#[test]
fn decode_index_to_tuple_zero_hcb_rejects() {
    assert_eq!(
        decode_index_to_tuple(0, 0).unwrap_err(),
        Error::SpectralCodebookHasNoTuple(0)
    );
}

#[test]
fn decode_index_to_tuple_nonspectral_books_reject() {
    for cb in [12u8, 13, 14, 15] {
        assert_eq!(
            decode_index_to_tuple(cb, 0).unwrap_err(),
            Error::SpectralCodebookHasNoTuple(cb)
        );
    }
}

#[test]
fn decode_index_to_tuple_out_of_range_index_rejects() {
    // Codebook 3: modulus = 3, dim = 4 → idx_max = 81.
    assert_eq!(
        decode_index_to_tuple(3, 81).unwrap_err(),
        Error::SpectralCodebookIndexOutOfRange(3)
    );
    // Codebook 7: modulus = 8, dim = 2 → idx_max = 64.
    assert_eq!(
        decode_index_to_tuple(7, 64).unwrap_err(),
        Error::SpectralCodebookIndexOutOfRange(7)
    );
}

#[test]
fn decode_index_to_tuple_out_of_range_codebook_rejects() {
    assert_eq!(
        decode_index_to_tuple(32, 0).unwrap_err(),
        Error::SpectralCodebookOutOfRange(32)
    );
}

// ---------------------------------------------------------------------
// encode_tuple_to_index (inverse) + roundtrip
// ---------------------------------------------------------------------

#[test]
fn encode_tuple_to_index_quad_signed_roundtrip_codebook_1() {
    // Walk every (w, x, y, z) tuple in -1..=+1 (3^4 = 81 tuples).
    for w in -1..=1 {
        for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    let tuple = [w, x, y, z];
                    let idx = encode_tuple_to_index(1, &tuple).unwrap();
                    let decoded = decode_index_to_tuple(1, idx).unwrap();
                    assert_eq!(decoded, tuple, "tuple {:?} idx {}", tuple, idx);
                }
            }
        }
    }
}

#[test]
fn encode_tuple_to_index_quad_unsigned_roundtrip_codebook_3() {
    // Walk every (w, x, y, z) tuple in 0..=2 (3^4 = 81 tuples).
    for w in 0..=2 {
        for x in 0..=2 {
            for y in 0..=2 {
                for z in 0..=2 {
                    let tuple = [w, x, y, z];
                    let idx = encode_tuple_to_index(3, &tuple).unwrap();
                    let decoded = decode_index_to_tuple(3, idx).unwrap();
                    assert_eq!(decoded, tuple, "tuple {:?} idx {}", tuple, idx);
                }
            }
        }
    }
}

#[test]
fn encode_tuple_to_index_pair_signed_roundtrip_codebook_5() {
    for y in -4..=4 {
        for z in -4..=4 {
            let tuple = [y, z, 0, 0];
            let idx = encode_tuple_to_index(5, &tuple).unwrap();
            let decoded = decode_index_to_tuple(5, idx).unwrap();
            assert_eq!(decoded[0], y);
            assert_eq!(decoded[1], z);
        }
    }
}

#[test]
fn encode_tuple_to_index_pair_unsigned_roundtrip_codebook_7() {
    for y in 0..=7 {
        for z in 0..=7 {
            let tuple = [y, z, 0, 0];
            let idx = encode_tuple_to_index(7, &tuple).unwrap();
            let decoded = decode_index_to_tuple(7, idx).unwrap();
            assert_eq!(decoded[0], y);
            assert_eq!(decoded[1], z);
        }
    }
}

#[test]
fn encode_tuple_to_index_rejects_out_of_lav_signed() {
    // Codebook 1: signed LAV 1, so (-1, 0, 0, 0) is valid but (-2,
    // 0, 0, 0) is not.
    assert!(encode_tuple_to_index(1, &[-1, 0, 0, 0]).is_ok());
    assert_eq!(
        encode_tuple_to_index(1, &[-2, 0, 0, 0]).unwrap_err(),
        Error::SpectralCodebookTupleOutOfRange(1)
    );
    assert_eq!(
        encode_tuple_to_index(1, &[0, 0, 0, 2]).unwrap_err(),
        Error::SpectralCodebookTupleOutOfRange(1)
    );
}

#[test]
fn encode_tuple_to_index_rejects_out_of_lav_unsigned() {
    // Codebook 3: unsigned LAV 2.
    assert!(encode_tuple_to_index(3, &[2, 0, 0, 0]).is_ok());
    assert_eq!(
        encode_tuple_to_index(3, &[3, 0, 0, 0]).unwrap_err(),
        Error::SpectralCodebookTupleOutOfRange(3)
    );
    assert_eq!(
        encode_tuple_to_index(3, &[0, 0, 0, -1]).unwrap_err(),
        Error::SpectralCodebookTupleOutOfRange(3)
    );
}

#[test]
fn encode_tuple_to_index_rejects_short_tuple() {
    assert_eq!(
        encode_tuple_to_index(1, &[0]).unwrap_err(),
        Error::SpectralCodebookTupleOutOfRange(1)
    );
    assert_eq!(
        encode_tuple_to_index(5, &[]).unwrap_err(),
        Error::SpectralCodebookTupleOutOfRange(5)
    );
}

#[test]
fn encode_tuple_to_index_rejects_nonspectral_codebook() {
    for cb in [0u8, 12, 13, 14, 15] {
        assert_eq!(
            encode_tuple_to_index(cb, &[0, 0, 0, 0]).unwrap_err(),
            Error::SpectralCodebookHasNoTuple(cb)
        );
    }
}

// ---------------------------------------------------------------------
// Sign-bit handling
// ---------------------------------------------------------------------

#[test]
fn apply_sign_bits_unsigned_codebook_3_all_nonzero() {
    // Codebook 3: unsigned, dim 4. Tuple (1, 2, 1, 2): 4 non-zero
    // → 4 sign bits.
    let tuple = [1, 2, 1, 2];
    let result = apply_sign_bits(3, tuple, &[true, false, true, false]).unwrap();
    assert_eq!(result, [-1, 2, -1, 2]);
}

#[test]
fn apply_sign_bits_unsigned_codebook_3_with_zeros() {
    // Tuple (1, 0, 2, 0): only 2 non-zero → 2 sign bits.
    let tuple = [1, 0, 2, 0];
    let result = apply_sign_bits(3, tuple, &[false, true]).unwrap();
    assert_eq!(result, [1, 0, -2, 0]);
}

#[test]
fn apply_sign_bits_signed_codebook_1_takes_no_bits() {
    // Codebook 1: signed; sign already in idx, no sign bits emitted.
    let tuple = [-1, 0, 1, -1];
    let result = apply_sign_bits(1, tuple, &[]).unwrap();
    assert_eq!(result, tuple);
}

#[test]
fn apply_sign_bits_signed_codebook_rejects_nonempty_signs() {
    assert_eq!(
        apply_sign_bits(1, [1, 0, 0, 0], &[true]).unwrap_err(),
        Error::SpectralCodebookSignBitsMismatch(1)
    );
}

#[test]
fn apply_sign_bits_unsigned_rejects_wrong_signs_length() {
    // 2 non-zero coefficients, but only 1 sign bit.
    assert_eq!(
        apply_sign_bits(3, [1, 0, 2, 0], &[true]).unwrap_err(),
        Error::SpectralCodebookSignBitsMismatch(3)
    );
    // 2 non-zero coefficients, but 3 sign bits.
    assert_eq!(
        apply_sign_bits(3, [1, 0, 2, 0], &[true, false, true]).unwrap_err(),
        Error::SpectralCodebookSignBitsMismatch(3)
    );
}

#[test]
fn derive_sign_bits_roundtrip_codebook_3() {
    let tuple = [-1, 0, 2, -2];
    let bits = derive_sign_bits(3, &tuple).unwrap();
    assert_eq!(bits, vec![true, false, true]);
    // Re-apply: drop signs from the source, then re-attach.
    let magnitudes = [
        tuple[0].abs(),
        tuple[1].abs(),
        tuple[2].abs(),
        tuple[3].abs(),
    ];
    let recovered = apply_sign_bits(3, magnitudes, &bits).unwrap();
    assert_eq!(recovered, tuple);
}

#[test]
fn derive_sign_bits_signed_codebook_returns_empty() {
    let bits = derive_sign_bits(1, &[-1, 0, 1, -1]).unwrap();
    assert!(bits.is_empty());
}

#[test]
fn derive_sign_bits_all_zero_tuple_returns_empty_bits() {
    let bits = derive_sign_bits(3, &[0, 0, 0, 0]).unwrap();
    assert!(bits.is_empty());
}

// ---------------------------------------------------------------------
// ESC sequence handling (§4.6.3.3 codebook 11)
// ---------------------------------------------------------------------

#[test]
fn encode_esc_value_then_decode_roundtrip_powers_of_two() {
    // Test each prefix_len: value = 2^(N + 4) yields prefix N, word 0.
    for n in 0..=8u32 {
        let value = 1u32 << (n + 4);
        let (prefix, word) = encode_esc_value(value).unwrap();
        assert_eq!(prefix, n, "value 2^{}", n + 4);
        assert_eq!(word, 0, "value 2^{}", n + 4);
        let decoded = decode_esc_value(prefix, word).unwrap();
        assert_eq!(decoded, value);
    }
}

#[test]
fn encode_esc_value_at_lav_boundary() {
    // value = 16 → 2^4 + 0 → prefix 0, word 0.
    assert_eq!(encode_esc_value(16).unwrap(), (0, 0));
    // value = 17 → 2^4 + 1 → prefix 0, word 1.
    assert_eq!(encode_esc_value(17).unwrap(), (0, 1));
    // value = 31 → 2^4 + 15 → prefix 0, word 15.
    assert_eq!(encode_esc_value(31).unwrap(), (0, 15));
    // value = 32 → 2^5 + 0 → prefix 1, word 0.
    assert_eq!(encode_esc_value(32).unwrap(), (1, 0));
}

#[test]
fn encode_esc_value_at_max_quant_boundary() {
    // value = MAX_QUANT = 8191 → 2^12 + 4095 → prefix 8, word 4095.
    let (prefix, word) = encode_esc_value(MAX_QUANT as u32).unwrap();
    assert_eq!(prefix, 8);
    assert_eq!(word, 4095);
    let decoded = decode_esc_value(prefix, word).unwrap();
    assert_eq!(decoded, MAX_QUANT as u32);
}

#[test]
fn encode_esc_value_rejects_in_band_magnitudes() {
    for v in 0..16 {
        assert_eq!(
            encode_esc_value(v).unwrap_err(),
            Error::SpectralCodebookEscOutOfRange
        );
    }
}

#[test]
fn encode_esc_value_rejects_overflow() {
    assert_eq!(
        encode_esc_value(8192).unwrap_err(),
        Error::SpectralCodebookEscOutOfRange
    );
}

#[test]
fn decode_esc_value_rejects_prefix_len_overflow() {
    // The decoder-side bound is N <= 24 (a hostile all-ones run must
    // still terminate and the magnitude stay in i32 range); N == 25
    // rejects.
    assert_eq!(
        decode_esc_value(25, 0).unwrap_err(),
        Error::SpectralCodebookEscOutOfRange
    );
}

#[test]
fn decode_esc_value_rejects_escape_word_overflow() {
    // prefix_len = 0 → word_bits = 4 → max word = 15.
    assert_eq!(
        decode_esc_value(0, 16).unwrap_err(),
        Error::SpectralCodebookEscOutOfRange
    );
    // prefix_len = 4 → word_bits = 8 → max word = 255.
    assert_eq!(
        decode_esc_value(4, 256).unwrap_err(),
        Error::SpectralCodebookEscOutOfRange
    );
}

#[test]
fn decode_esc_value_accepts_magnitudes_above_max_quant() {
    // §4.6.2's 8191 cap binds the encoder, not the decoder: the
    // ISO/IEC 14496-26 ER AAC LD conformance vectors transmit N == 9
    // escapes (er_ad1103np_22_ep0 AU 508 carries magnitude 9283 =
    // 2^13 + 1091) and their reference waveforms decode them.
    assert_eq!(decode_esc_value(9, 0).unwrap(), 8192);
    assert_eq!(decode_esc_value(9, 1091).unwrap(), 9283);
    // er_ad1103np_24_ep0 AU 1551: N = 15, word = 259678 -> 783966.
    assert_eq!(decode_esc_value(15, 259_678).unwrap(), 783_966);
    // The encoder-side inverse still refuses to emit them.
    assert_eq!(
        encode_esc_value(8192).unwrap_err(),
        Error::SpectralCodebookEscOutOfRange
    );
}

// ---------------------------------------------------------------------
// Row helper accessors
// ---------------------------------------------------------------------

#[test]
fn table_4_95_row_is_unsigned_helper() {
    let r1: Table495Row = TABLE_4_95[1];
    assert!(!r1.is_unsigned()); // signed codebook
    let r3: Table495Row = TABLE_4_95[3];
    assert!(r3.is_unsigned()); // unsigned codebook
    let r12: Table495Row = TABLE_4_95[12];
    assert!(!r12.is_unsigned()); // reserved (None → false)
}

#[test]
fn table_4_95_row_has_esc_helper() {
    for (cb, row) in TABLE_4_95.iter().enumerate().take(11).skip(1) {
        assert!(!row.has_esc(), "cb {}", cb);
    }
    assert!(TABLE_4_95[11].has_esc());
    for (cb, row) in TABLE_4_95.iter().enumerate().skip(16) {
        assert!(row.has_esc(), "cb {}", cb);
    }
}

// ---------------------------------------------------------------------
// Cross-check Table 4.95 LAV against the §4.6.1.3 MAX_QUANT cap.
// ---------------------------------------------------------------------

#[test]
fn max_quant_matches_codebook_11_esc_threshold() {
    // §4.6.1.3: max allowed absolute amplitude for x_quant is 8191.
    // Codebook 11 ESC threshold is also 8191; the codebook is the
    // *only* spectrum book that can represent values up to MAX_QUANT.
    assert_eq!(MAX_QUANT as u32, TABLE_4_95[11].esc_threshold.unwrap());
}

#[test]
fn no_in_band_lav_exceeds_max_quant() {
    for row in TABLE_4_95.iter() {
        if let Some(lav) = row.lav {
            assert!(lav as i32 <= MAX_QUANT, "in-band LAV exceeds MAX_QUANT");
        }
    }
}

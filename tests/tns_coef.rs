//! Integration tests for the §4.6.9.3 / §C.6 TNS coefficient
//! inverse-quantisation primitives.

use oxideav_aac::tns_coef::{
    iqfac, iqfac_m, lpc_step_up, pack_coef, sign_extend_coef, tns_decode_coef,
    tns_decode_coef_to_lpc, tns_encode_coef,
};
use oxideav_aac::Error;

// ---- iqfac vs iqfac_m ----

#[test]
fn iqfac_m_minus_iqfac_equals_one_over_half_pi() {
    // iqfac_m - iqfac = ((1<<(n-1))+0.5)/(π/2) - ((1<<(n-1))-0.5)/(π/2)
    //                 = 1.0 / (π/2) = 2/π — invariant of every legal n.
    let expected = 1.0_f64 / (core::f64::consts::PI / 2.0);
    for n in [3_u32, 4] {
        let diff = iqfac_m(n).unwrap() - iqfac(n).unwrap();
        assert!(
            (diff - expected).abs() < 1e-15,
            "n={n} diff={diff} want={expected}",
        );
    }
}

// ---- sign extend + pack round-trips (full enumeration) ----

#[test]
fn sign_extend_pack_round_trips_every_width_every_value() {
    for coef_res2 in 2_u32..=4 {
        let half = 1i32 << (coef_res2 - 1);
        for value in -half..half {
            let packed = pack_coef(value, coef_res2).unwrap();
            // The wire value must fit the field.
            assert!(packed < (1u32 << coef_res2));
            // And re-extend to the same signed value.
            assert_eq!(sign_extend_coef(packed, coef_res2).unwrap(), value);
        }
    }
}

// ---- decode is a strict subset of [-1, 1] ----

#[test]
fn decoded_parcor_magnitudes_never_exceed_one() {
    // For every legal (coef_res_bits, coef_compress) combination, walk
    // every wire value the field admits and confirm |sin(...)| <= 1.
    // This is mathematically guaranteed but worth a regression check
    // because the iqfac branch chooses between two divisors and a
    // formula slip could push outside.
    for coef_res_bits in [3_u32, 4] {
        for coef_compress in [0_u32, 1] {
            let coef_res2 = coef_res_bits - coef_compress;
            let max = 1u32 << coef_res2;
            for wire in 0..max {
                let parcor = tns_decode_coef(coef_res_bits, coef_compress, &[wire]).unwrap();
                assert_eq!(parcor.len(), 1);
                assert!(
                    parcor[0].abs() <= 1.0,
                    "(rb={coef_res_bits},c={coef_compress},w={wire}) ⇒ {}",
                    parcor[0],
                );
            }
        }
    }
}

// ---- full round-trip surface across every config ----

#[test]
fn every_legal_wire_pattern_round_trips_through_decode_then_encode() {
    for coef_res_bits in [3_u32, 4] {
        for coef_compress in [0_u32, 1] {
            let coef_res2 = coef_res_bits - coef_compress;
            let max = 1u32 << coef_res2;
            for wire in 0..max {
                let parcor = tns_decode_coef(coef_res_bits, coef_compress, &[wire]).unwrap();
                let back = tns_encode_coef(coef_res_bits, coef_compress, &parcor).unwrap();
                assert_eq!(
                    back,
                    vec![wire],
                    "(rb={coef_res_bits},c={coef_compress},w={wire}) decode→encode",
                );
            }
        }
    }
}

// ---- multi-coefficient batches survive together ----

#[test]
fn batch_of_mixed_signs_round_trips_in_order() {
    // A realistic per-filter coef[] mixes positive and negative
    // PARCOR indices. Confirm the per-coefficient branch selection
    // works element-wise (the iqfac vs iqfac_m branch is keyed on the
    // *current* coefficient's sign, not the batch).
    let wire = vec![0_u32, 1, 7, 8, 0xF, 3, 5, 0xC];
    let parcor = tns_decode_coef(4, 0, &wire).unwrap();
    let back = tns_encode_coef(4, 0, &parcor).unwrap();
    assert_eq!(back, wire);
}

// ---- compressed-mode field width is independent of iqfac ----

#[test]
fn coef_compress_only_shrinks_field_not_iqfac() {
    // Two different (coef_res_bits, coef_compress) pairs yield the
    // same coef_res2=3 sign-extend window: (3, 0) and (4, 1). They
    // share the same field width but use *different* iqfac scaling
    // (3-bit iqfac vs 4-bit iqfac), so the decoded PARCOR magnitudes
    // differ for the same wire value.
    for wire in 0_u32..8 {
        let parcor_rb3 = tns_decode_coef(3, 0, &[wire]).unwrap()[0];
        let parcor_rb4 = tns_decode_coef(4, 1, &[wire]).unwrap()[0];
        // Same field width ⇒ same sign-extended index.
        let extended = sign_extend_coef(wire, 3).unwrap();
        if extended == 0 {
            // Both sin(0) = 0, no contrast to test.
            assert_eq!(parcor_rb3, parcor_rb4);
            continue;
        }
        // Different iqfac ⇒ different magnitudes (4-bit iqfac is
        // ~2.14× the 3-bit iqfac, so the rb=4 decode lands ~half-way
        // closer to zero).
        assert_ne!(
            parcor_rb3, parcor_rb4,
            "wire {wire} should differ between rb=3 and rb=4",
        );
        // Both stay in [-1, 1].
        assert!(parcor_rb3.abs() <= 1.0);
        assert!(parcor_rb4.abs() <= 1.0);
    }
}

// ---- decode → step-up LPC chain matches manual composition ----

#[test]
fn decode_to_lpc_matches_manual_composition() {
    let wire = [3_u32, 5, 0xF, 0, 7];
    let want = lpc_step_up(&tns_decode_coef(4, 0, &wire).unwrap());
    let got = tns_decode_coef_to_lpc(4, 0, &wire).unwrap();
    assert_eq!(got.len(), wire.len() + 1);
    assert_eq!(got, want);
    // The first slot is always 1.0.
    assert!((got[0] - 1.0).abs() < 1e-15);
}

// ---- step-up preserves PARCOR stability for |k| < 1 ----

#[test]
fn step_up_stable_filter_has_minimum_phase_lpc() {
    // A PARCOR sequence with |k_i| < 1 produces a stable
    // (minimum-phase) all-pole filter. We don't compute the roots —
    // instead we sanity-check the §4.6.9.3 invariant `a[0] = 1` and
    // that no LPC coefficient blows up (a single check at order 8
    // catches the egregious cases).
    let parcor = vec![0.4, -0.3, 0.2, -0.1, 0.05, -0.025, 0.0125, -0.00625];
    let a = lpc_step_up(&parcor);
    assert_eq!(a.len(), parcor.len() + 1);
    assert!((a[0] - 1.0).abs() < 1e-15);
    for (i, &v) in a.iter().enumerate() {
        assert!(
            v.is_finite() && v.abs() < 1e3,
            "a[{i}] = {v} is not finite or unreasonably large",
        );
    }
    // a[last] equals the last PARCOR by the spec's `a[m] = tmp2[m-1]`.
    assert!((a[parcor.len()] - parcor[parcor.len() - 1]).abs() < 1e-15);
}

// ---- failure modes surface a single canonical Error ----

#[test]
fn invalid_args_surface_tns_coef_out_of_range() {
    assert!(matches!(iqfac(0), Err(Error::TnsCoefOutOfRange)));
    assert!(matches!(iqfac_m(2), Err(Error::TnsCoefOutOfRange)));
    assert!(matches!(
        sign_extend_coef(0, 5),
        Err(Error::TnsCoefOutOfRange)
    ));
    assert!(matches!(pack_coef(0, 5), Err(Error::TnsCoefOutOfRange)));
    assert!(matches!(
        tns_decode_coef(5, 0, &[0]),
        Err(Error::TnsCoefOutOfRange)
    ));
    assert!(matches!(
        tns_decode_coef(4, 0, &[16]),
        Err(Error::TnsCoefOutOfRange)
    ));
    assert!(matches!(
        tns_encode_coef(4, 0, &[1.5]),
        Err(Error::TnsCoefOutOfRange)
    ));
    assert!(matches!(
        tns_decode_coef_to_lpc(0, 0, &[0]),
        Err(Error::TnsCoefOutOfRange)
    ));
}

// ---- a `tns_data::TnsFilter` wire batch decodes correctly ----

#[test]
fn decodes_realistic_filter_order_8_at_coef_res_1() {
    // Realistic AAC LC at coef_res = 1 (coef_res_bits = 4),
    // coef_compress = 0 (coef_res2 = 4). The first PARCOR after a
    // typical 8th-order TNS Levinson run sits near the field edge
    // (high-magnitude reflection), and successive coefficients decay
    // toward zero. We supply a hand-crafted wire pattern and check
    // that the decoded magnitudes monotonically decay.
    let wire = [6_u32, 5, 4, 3, 2, 1, 1, 0];
    let parcor = tns_decode_coef(4, 0, &wire).unwrap();
    assert_eq!(parcor.len(), 8);
    // All positive (every wire value sign-extends to a non-negative
    // index, then the sin() preserves the sign).
    for &v in &parcor {
        assert!(v >= 0.0);
    }
    // Strictly decreasing through index 5 (the wire 1, 1 pair lands
    // on the same magnitude).
    for i in 0..5 {
        assert!(
            parcor[i] > parcor[i + 1],
            "expected decay at i={i}: {} > {}",
            parcor[i],
            parcor[i + 1],
        );
    }
    // Trailing zero wire decodes to exactly 0.
    assert!(parcor[7].abs() < 1e-15);
}

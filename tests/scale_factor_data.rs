//! Self-roundtrip + structural tests for
//! [`oxideav_aac::scale_factor_data`] — ISO/IEC 14496-3 §4.4.6 /
//! Table 4.53 (non-resilient branch) plus the §4.6.3 / Table 4.A.1
//! scalefactor Huffman codebook (codebook 12).
//!
//! Each test builds a [`ScaleFactorData`] in memory, runs
//! [`ScaleFactorData::write`] into a fresh [`BitWriter`] with a
//! matching `sfb_cb` codebook map, then feeds the resulting byte
//! buffer through [`ScaleFactorData::parse`] driven by the same
//! `sfb_cb` map and asserts:
//!
//! 1. The parsed structure equals the original (bit-exact structural
//!    recovery).
//! 2. The parser's terminal bit-position matches the writer's
//!    bit-position (no over- or under-read).
//!
//! The only inputs are the
//! 121-entry Table 4.A.1 codebook (transcribed verbatim in
//! `src/scale_factor_data.rs`), the `index_offset = -60` from Table
//! 4.150, and the `noise_pcm_flag` frame-scope behaviour from Table
//! 4.53.

use oxideav_aac::scale_factor_data::{
    hcod_sf_decode, hcod_sf_encode, ScaleFactorData, ScaleFactorEntry, NOISE_PCM_BITS,
};
use oxideav_aac::section_data::{INTENSITY_HCB, INTENSITY_HCB2, NOISE_HCB, ZERO_HCB};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// Encode `sfd` against `sfb_cb`, parse the result back, and assert
/// structural + bit-position parity. Returns the encoded bytes and
/// the writer's terminal bit position so individual tests can extra-
/// check sizing / known-prefix invariants.
fn roundtrip(sfd: &ScaleFactorData, sfb_cb: &[Vec<u8>]) -> (Vec<u8>, u64) {
    let mut bw = BitWriter::new();
    sfd.write(&mut bw, sfb_cb).expect("write succeeds");
    let bits_written = bw.bit_position();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed =
        ScaleFactorData::parse(&mut br, sfb_cb).expect("parse of self-encoded stream succeeds");
    assert_eq!(&parsed, sfd, "round-trip changes structure");
    assert_eq!(
        br.bit_position(),
        bits_written,
        "parser consumed a different number of bits than the writer emitted"
    );
    (buf, bits_written)
}

// ---- Trivial / boundary cases ----

/// Empty: zero groups → zero entries, zero output bits.
#[test]
fn empty_codebook_map_roundtrips() {
    let sfb_cb: Vec<Vec<u8>> = vec![];
    let sfd = ScaleFactorData {
        entries: Vec::new(),
    };
    let (buf, bits) = roundtrip(&sfd, &sfb_cb);
    assert_eq!(bits, 0);
    assert!(buf.is_empty());
}

/// One long-window group where every band is ZERO_HCB → no per-band
/// records at all. Even with `max_sfb = 50` the encoder writes zero
/// bits because there is nothing to emit.
#[test]
fn all_zero_hcb_emits_no_bits() {
    let sfb_cb = vec![vec![ZERO_HCB; 50]];
    let sfd = ScaleFactorData {
        entries: vec![Vec::new()],
    };
    let (buf, bits) = roundtrip(&sfd, &sfb_cb);
    assert_eq!(bits, 0);
    assert!(buf.is_empty());
}

/// Single band with codebook 1 (a non-zero spectrum book) and
/// `dpcm_sf = 0` → emits exactly one bit (the `0` codeword for index
/// 60 in Table 4.A.1).
#[test]
fn single_band_dpcm_zero_emits_one_bit() {
    let sfb_cb = vec![vec![1u8]];
    let sfd = ScaleFactorData {
        entries: vec![vec![ScaleFactorEntry::Dpcm(0)]],
    };
    let (_buf, bits) = roundtrip(&sfd, &sfb_cb);
    assert_eq!(bits, 1);
}

/// Single intensity-stereo band → uses the same `hcod_sf[]` codebook.
/// `dpcm_is_position = 0` → 1 bit.
#[test]
fn single_intensity_band_dpcm_zero_emits_one_bit() {
    for cb in [INTENSITY_HCB, INTENSITY_HCB2] {
        let sfb_cb = vec![vec![cb]];
        let sfd = ScaleFactorData {
            entries: vec![vec![ScaleFactorEntry::Intensity(0)]],
        };
        let (_buf, bits) = roundtrip(&sfd, &sfb_cb);
        assert_eq!(bits, 1, "intensity cb={} produced {} bits", cb, bits);
    }
}

/// First PNS band of the frame is a 9-bit literal seed; second is a
/// Huffman delta. With `dpcm = 0` the second band collapses to 1
/// bit, so the total is `9 + 1 = 10` bits.
#[test]
fn first_pns_uses_pcm_seed_subsequent_uses_huffman() {
    let sfb_cb = vec![vec![NOISE_HCB, NOISE_HCB]];
    let sfd = ScaleFactorData {
        entries: vec![vec![
            ScaleFactorEntry::NoisePcm(0x123),
            ScaleFactorEntry::NoiseDpcm(0),
        ]],
    };
    let (_buf, bits) = roundtrip(&sfd, &sfb_cb);
    assert_eq!(bits, u64::from(NOISE_PCM_BITS) + 1);
}

/// `noise_pcm_flag` is **frame**-scoped: a PNS band in window group 0
/// consumes the seed even if group 1's first PNS band is the
/// numerically-first PNS hit on the wire. Verifies the flag does
/// not reset between groups.
#[test]
fn noise_pcm_flag_is_frame_scoped_across_groups() {
    let sfb_cb = vec![
        // group 0: one PNS band (consumes the seed)
        vec![NOISE_HCB],
        // group 1: one PNS band (must Huffman-encode, the seed is gone)
        vec![NOISE_HCB],
    ];
    let sfd = ScaleFactorData {
        entries: vec![
            vec![ScaleFactorEntry::NoisePcm(0x55)],
            vec![ScaleFactorEntry::NoiseDpcm(-3)],
        ],
    };
    let (_buf, bits) = roundtrip(&sfd, &sfb_cb);
    // First group: 9-bit PCM seed.
    // Second group: hcod_sf for index 57 (dpcm = -3) → 5 bits.
    let (cw_len, _) = hcod_sf_encode(-3).unwrap();
    assert_eq!(bits, u64::from(NOISE_PCM_BITS) + u64::from(cw_len));
}

// ---- Coverage across all DPCM values ----

/// Walk every legal DPCM value `-60..=+60` as a one-band group; the
/// emitted bit count must match `HCOD_SF[idx].0` from Table 4.A.1.
#[test]
fn every_dpcm_value_roundtrips_one_band_per_group() {
    for dpcm in -60i8..=60 {
        let sfb_cb = vec![vec![3u8]]; // codebook 3 (signed QUAD), non-zero
        let sfd = ScaleFactorData {
            entries: vec![vec![ScaleFactorEntry::Dpcm(dpcm)]],
        };
        let (_buf, bits) = roundtrip(&sfd, &sfb_cb);
        let (len, _) = hcod_sf_encode(dpcm).unwrap();
        assert_eq!(bits, u64::from(len), "dpcm={} bits mismatch", dpcm);
    }
}

// ---- Mixed-content frame ----

/// One window group with a representative mix: ZERO_HCB skip,
/// spectrum band, intensity band, ZERO_HCB skip, PNS-first,
/// PNS-second, intensity-out-of-phase. Validates that the parser
/// resynchronises across ZERO_HCB gaps and dispatches between the
/// four entry variants correctly.
#[test]
fn mixed_frame_one_group_roundtrips() {
    let sfb_cb = vec![vec![
        ZERO_HCB,
        4, // spectrum
        INTENSITY_HCB,
        ZERO_HCB,
        NOISE_HCB,
        NOISE_HCB,
        INTENSITY_HCB2,
        7, // spectrum
    ]];
    let sfd = ScaleFactorData {
        entries: vec![vec![
            ScaleFactorEntry::Dpcm(5),
            ScaleFactorEntry::Intensity(-12),
            ScaleFactorEntry::NoisePcm(0x1ff), // max value
            ScaleFactorEntry::NoiseDpcm(7),
            ScaleFactorEntry::Intensity(-1),
            ScaleFactorEntry::Dpcm(-60), // extreme negative
        ]],
    };
    let _ = roundtrip(&sfd, &sfb_cb);
}

/// EIGHT_SHORT-style frame: 8 window groups, each with a single
/// non-zero band, exercising the per-group iteration of
/// `scale_factor_data()` against a non-trivial `num_window_groups`
/// (the parser does not need to know `num_window_groups` separately
/// — it walks `sfb_cb.len()`).
#[test]
fn eight_groups_each_one_band_roundtrips() {
    let sfb_cb: Vec<Vec<u8>> = (0..8).map(|_| vec![2u8]).collect();
    let entries: Vec<Vec<ScaleFactorEntry>> = (0..8)
        .map(|i| vec![ScaleFactorEntry::Dpcm((i as i8) - 4)])
        .collect();
    let sfd = ScaleFactorData { entries };
    let _ = roundtrip(&sfd, &sfb_cb);
}

// ---- Wire-format spot checks ----

/// Verify that `dpcm = 0` emits the single-bit `0` codeword and that
/// the parser bit-position lands exactly on the next bit. The wire
/// byte after a single `0` bit followed by writer pad is `0x00`.
#[test]
fn single_zero_bit_wire_shape() {
    let sfb_cb = vec![vec![1u8]];
    let sfd = ScaleFactorData {
        entries: vec![vec![ScaleFactorEntry::Dpcm(0)]],
    };
    let mut bw = BitWriter::new();
    sfd.write(&mut bw, &sfb_cb).unwrap();
    assert_eq!(bw.bit_position(), 1);
    let buf = bw.finish();
    assert_eq!(buf.as_slice(), &[0x00]); // padded to a byte
}

/// `dpcm_noise_nrg = 0x1AB` (a known 9-bit pattern) is emitted as
/// the first PNS band. The wire layout starts with `0b110_1010_11_`,
/// i.e. `0xD5` + `0b1`-then-pad.
#[test]
fn first_pns_pcm_wire_shape_is_msb_first() {
    let sfb_cb = vec![vec![NOISE_HCB]];
    let sfd = ScaleFactorData {
        entries: vec![vec![ScaleFactorEntry::NoisePcm(0x1AB)]],
    };
    let mut bw = BitWriter::new();
    sfd.write(&mut bw, &sfb_cb).unwrap();
    assert_eq!(bw.bit_position(), u64::from(NOISE_PCM_BITS));
    let buf = bw.finish();
    // 0x1AB = 0b1_1010_1011. MSB-first packing into two bytes:
    //   first byte: bits 8..1 = 1101_0101 = 0xD5
    //   second byte: bit 0 = 1, then 7 zero pad bits = 0x80
    assert_eq!(buf.as_slice(), &[0xD5, 0x80]);
}

// ---- Negative-path tests (the writer's structural sieve) ----

/// `entries.len()` must equal `sfb_cb.len()`.
#[test]
fn write_rejects_outer_length_mismatch() {
    let sfb_cb = vec![vec![1u8], vec![1u8]];
    let sfd = ScaleFactorData {
        entries: vec![vec![ScaleFactorEntry::Dpcm(0)]], // missing second group
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        sfd.write(&mut bw, &sfb_cb),
        Err(Error::ScaleFactorDataEncodeInvalid)
    );
}

/// More entries than non-zero bands.
#[test]
fn write_rejects_extra_entries() {
    let sfb_cb = vec![vec![1u8, ZERO_HCB]];
    let sfd = ScaleFactorData {
        entries: vec![vec![
            ScaleFactorEntry::Dpcm(0),
            ScaleFactorEntry::Dpcm(0), // surplus
        ]],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        sfd.write(&mut bw, &sfb_cb),
        Err(Error::ScaleFactorDataEncodeInvalid)
    );
}

/// Fewer entries than non-zero bands.
#[test]
fn write_rejects_missing_entries() {
    let sfb_cb = vec![vec![1u8, 2u8]];
    let sfd = ScaleFactorData {
        entries: vec![vec![ScaleFactorEntry::Dpcm(0)]],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        sfd.write(&mut bw, &sfb_cb),
        Err(Error::ScaleFactorDataEncodeInvalid)
    );
}

/// Intensity variant paired with a spectrum band.
#[test]
fn write_rejects_intensity_variant_on_spectrum_band() {
    let sfb_cb = vec![vec![3u8]];
    let sfd = ScaleFactorData {
        entries: vec![vec![ScaleFactorEntry::Intensity(0)]],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        sfd.write(&mut bw, &sfb_cb),
        Err(Error::ScaleFactorDataEncodeInvalid)
    );
}

/// `NoisePcm` paired with a non-PNS band.
#[test]
fn write_rejects_noisepcm_on_non_pns_band() {
    let sfb_cb = vec![vec![3u8]];
    let sfd = ScaleFactorData {
        entries: vec![vec![ScaleFactorEntry::NoisePcm(0)]],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        sfd.write(&mut bw, &sfb_cb),
        Err(Error::ScaleFactorDataEncodeInvalid)
    );
}

/// `NoiseDpcm` as the **first** PNS band — illegal; the first PNS
/// band of the frame must carry the 9-bit PCM seed.
#[test]
fn write_rejects_noisedpcm_as_first_pns() {
    let sfb_cb = vec![vec![NOISE_HCB]];
    let sfd = ScaleFactorData {
        entries: vec![vec![ScaleFactorEntry::NoiseDpcm(0)]],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        sfd.write(&mut bw, &sfb_cb),
        Err(Error::ScaleFactorDataEncodeInvalid)
    );
}

/// Second `NoisePcm` in the same frame — illegal; the wire bit is
/// frame-scoped and can fire only once.
#[test]
fn write_rejects_second_noisepcm_in_frame() {
    let sfb_cb = vec![vec![NOISE_HCB, NOISE_HCB]];
    let sfd = ScaleFactorData {
        entries: vec![vec![
            ScaleFactorEntry::NoisePcm(0x10),
            ScaleFactorEntry::NoisePcm(0x20), // illegal — flag cleared
        ]],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        sfd.write(&mut bw, &sfb_cb),
        Err(Error::ScaleFactorDataEncodeInvalid)
    );
}

/// DPCM delta outside `-60..=+60`.
#[test]
fn write_rejects_dpcm_out_of_range() {
    for bad in [-61i8, 61, 100, -100] {
        let sfb_cb = vec![vec![1u8]];
        let sfd = ScaleFactorData {
            entries: vec![vec![ScaleFactorEntry::Dpcm(bad)]],
        };
        let mut bw = BitWriter::new();
        assert_eq!(
            sfd.write(&mut bw, &sfb_cb),
            Err(Error::ScaleFactorDataEncodeInvalid),
            "dpcm={} should be rejected",
            bad
        );
    }
}

/// `NoisePcm` magnitude exceeds the 9-bit field cap (0x1FF).
#[test]
fn write_rejects_noisepcm_overflow() {
    let sfb_cb = vec![vec![NOISE_HCB]];
    let sfd = ScaleFactorData {
        entries: vec![vec![ScaleFactorEntry::NoisePcm(0x200)]],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        sfd.write(&mut bw, &sfb_cb),
        Err(Error::ScaleFactorDataEncodeInvalid)
    );
}

// ---- Decoder-only paths ----

/// A truncated stream (1 bit short of a 9-bit PCM seed) surfaces
/// [`Error::UnexpectedEnd`] from the parser. Verifies the
/// `read_u32` error path on the `dpcm_noise_nrg` PCM branch.
#[test]
fn parse_underflow_on_noise_pcm() {
    let sfb_cb = vec![vec![NOISE_HCB]];
    // Only 8 bits of input, but the PCM seed needs 9.
    let buf = [0xFFu8];
    let mut br = BitReader::new(&buf);
    let result = ScaleFactorData::parse(&mut br, &sfb_cb);
    assert_eq!(result, Err(Error::UnexpectedEnd));
}

/// Exhaustive completeness check: walk every 19-bit prefix and
/// verify `hcod_sf_decode` succeeds for all of them. The Table 4.A.1
/// codebook satisfies Kraft equality (`Σ 2^(19-L) = 2^19`), which
/// is a stronger guarantee than the prefix-free check in the
/// module's embedded test — it confirms the codebook is *complete*,
/// not just well-formed. Combined with the prefix-free property
/// (proved in the module unit tests) this means every fully-read
/// 19-bit sequence decodes to exactly one Table 4.A.1 entry, which
/// is what the `unreachable!()` at the bottom of
/// [`hcod_sf_decode`] relies on.
#[test]
fn hcod_sf_decode_is_complete_over_19_bits() {
    for v in 0u32..(1u32 << 19) {
        let mut bw = BitWriter::new();
        bw.write_u32(v, 19);
        let buf = bw.finish();
        let mut br = BitReader::new(&buf);
        // Every 19-bit prefix decodes successfully.
        hcod_sf_decode(&mut br).expect("complete codebook covers every 19-bit prefix");
    }
}

/// The first non-trivial entries (`HCOD_SF[59] = (3, 4)`, etc.)
/// reach the decoder correctly when invoked directly without the
/// `scale_factor_data()` driver.
#[test]
fn hcod_sf_decode_recovers_known_short_codewords() {
    // Codeword for index 59 (dpcm = -1) is 3 bits 0b100 = 4.
    let mut bw = BitWriter::new();
    let (len, cw) = hcod_sf_encode(-1).unwrap();
    assert_eq!((len, cw), (3, 4));
    bw.write_u32(cw, u32::from(len));
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    assert_eq!(hcod_sf_decode(&mut br).unwrap(), -1);
}

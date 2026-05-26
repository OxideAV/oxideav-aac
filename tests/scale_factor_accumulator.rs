//! Round-trip + structural tests for the §4.6.2.3.2 / §4.6.8.1.4 /
//! §4.6.13 DPCM accumulators added in round 152 — the
//! [`accumulate`](oxideav_aac::scale_factor_data::accumulate) /
//! [`differentiate`](oxideav_aac::scale_factor_data::differentiate)
//! pair that converts between transmitted DPCM deltas and absolute
//! per-band quantities.
//!
//! The three-track model implemented here:
//!
//! 1. **Spectrum scalefactors** (codebooks 1..=11): seed
//!    `last_sf = global_gain`, range `0..=255`.
//! 2. **Intensity stereo positions** (codebooks 14, 15): seed
//!    `last_is = 0`.
//! 3. **PNS noise energies** (codebook 13): seed `last_nrg =
//!    global_gain - NOISE_OFFSET - 256`. First PNS band carries a
//!    9-bit unsigned literal added directly to `last_nrg`;
//!    subsequent PNS bands carry Huffman deltas in `-60..=+60`.
//!
//! The §4.6.2.3.2 illustrative pseudocode that would single-track
//! everything is deliberately not implemented — the surrounding
//! §4.6.8.1.4 / §4.6.13 prose-level "done separately" wording
//! takes precedence (see `scale_factor_data.rs` module docstring).

use oxideav_aac::scale_factor_data::{
    accumulate, differentiate, AbsoluteScaleFactorEntry, AbsoluteScaleFactors, ScaleFactorData,
    ScaleFactorEntry, NOISE_OFFSET,
};
use oxideav_aac::section_data::{INTENSITY_HCB, INTENSITY_HCB2, NOISE_HCB, ZERO_HCB};
use oxideav_aac::Error;

/// Helper: build an [`AbsoluteScaleFactors`] from a flat list of
/// per-group inner vectors.
fn abs(entries: Vec<Vec<AbsoluteScaleFactorEntry>>) -> AbsoluteScaleFactors {
    AbsoluteScaleFactors { entries }
}

/// Helper: build a [`ScaleFactorData`] from a flat list of per-group
/// inner vectors.
fn sfd(entries: Vec<Vec<ScaleFactorEntry>>) -> ScaleFactorData {
    ScaleFactorData { entries }
}

/// `accumulate ∘ differentiate == identity` on every well-formed
/// `abs` (the encoder→DPCM→decoder loop is bit-exact).
fn accumulate_differentiate_roundtrip(
    abs_in: &AbsoluteScaleFactors,
    sfb_cb: &[Vec<u8>],
    global_gain: u8,
) {
    let sfd = differentiate(abs_in, sfb_cb, global_gain).expect("differentiate succeeds");
    let abs_out = accumulate(&sfd, sfb_cb, global_gain).expect("accumulate succeeds");
    assert_eq!(
        &abs_out, abs_in,
        "accumulate ∘ differentiate is not identity"
    );
}

/// `differentiate ∘ accumulate == identity` on every well-formed
/// `sfd` (the decoder→absolute→encoder loop is bit-exact).
fn differentiate_accumulate_roundtrip(
    sfd_in: &ScaleFactorData,
    sfb_cb: &[Vec<u8>],
    global_gain: u8,
) {
    let abs = accumulate(sfd_in, sfb_cb, global_gain).expect("accumulate succeeds");
    let sfd_out = differentiate(&abs, sfb_cb, global_gain).expect("differentiate succeeds");
    assert_eq!(
        &sfd_out, sfd_in,
        "differentiate ∘ accumulate is not identity"
    );
}

// =============================================================================
// Trivial paths
// =============================================================================

#[test]
fn empty_sfb_cb_yields_empty_absolute() {
    let sfb_cb: Vec<Vec<u8>> = vec![];
    let abs_in = abs(vec![]);
    accumulate_differentiate_roundtrip(&abs_in, &sfb_cb, 100);
}

#[test]
fn all_zero_hcb_yields_no_entries() {
    let sfb_cb = vec![vec![ZERO_HCB, ZERO_HCB, ZERO_HCB]];
    let abs_in = abs(vec![vec![]]);
    accumulate_differentiate_roundtrip(&abs_in, &sfb_cb, 100);
}

#[test]
fn empty_per_group_lists_roundtrip() {
    // 4 window groups, every band ZERO_HCB.
    let sfb_cb = vec![vec![ZERO_HCB; 5]; 4];
    let abs_in = abs(vec![vec![]; 4]);
    accumulate_differentiate_roundtrip(&abs_in, &sfb_cb, 50);
}

// =============================================================================
// Spectrum track only
// =============================================================================

#[test]
fn single_spectrum_band_zero_delta_at_global_gain() {
    // sf = global_gain → dpcm = 0 (the single-bit `0` codeword).
    let sfb_cb = vec![vec![3u8]]; // codebook 3 (spectrum)
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::Sf(100)]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 100).unwrap();
    assert_eq!(sfd.entries[0][0], ScaleFactorEntry::Dpcm(0));
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 100);
}

#[test]
fn spectrum_track_ascending_deltas() {
    // global_gain = 100; sf values 100, 105, 110, 115 → deltas 0, 5, 5, 5.
    let sfb_cb = vec![vec![1u8, 2, 3, 4]];
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::Sf(100),
        AbsoluteScaleFactorEntry::Sf(105),
        AbsoluteScaleFactorEntry::Sf(110),
        AbsoluteScaleFactorEntry::Sf(115),
    ]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 100).unwrap();
    assert_eq!(
        sfd.entries[0],
        vec![
            ScaleFactorEntry::Dpcm(0),
            ScaleFactorEntry::Dpcm(5),
            ScaleFactorEntry::Dpcm(5),
            ScaleFactorEntry::Dpcm(5),
        ]
    );
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 100);
}

#[test]
fn spectrum_track_boundary_deltas() {
    // ±60 are the table-4.150 boundaries; verify both are reachable.
    let sfb_cb = vec![vec![1u8, 1, 1]];
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::Sf(100), // delta 0
        AbsoluteScaleFactorEntry::Sf(160), // delta +60
        AbsoluteScaleFactorEntry::Sf(100), // delta -60
    ]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 100).unwrap();
    assert_eq!(
        sfd.entries[0],
        vec![
            ScaleFactorEntry::Dpcm(0),
            ScaleFactorEntry::Dpcm(60),
            ScaleFactorEntry::Dpcm(-60),
        ]
    );
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 100);
}

#[test]
fn spectrum_track_sf_at_clip_boundaries() {
    // `sf` must stay in `0..=255` per §4.6.2.3.2 Note; verify both
    // endpoints are reachable.
    let sfb_cb = vec![vec![1u8, 1]];
    // global_gain=60; abs sf go 0, then 60 (delta -60, then +60).
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::Sf(0),
        AbsoluteScaleFactorEntry::Sf(60),
    ]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 60).unwrap();
    assert_eq!(
        sfd.entries[0],
        vec![ScaleFactorEntry::Dpcm(-60), ScaleFactorEntry::Dpcm(60)]
    );
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 60);

    // Upper boundary 255.
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::Sf(255),
        AbsoluteScaleFactorEntry::Sf(255),
    ]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 195).unwrap();
    assert_eq!(
        sfd.entries[0],
        vec![ScaleFactorEntry::Dpcm(60), ScaleFactorEntry::Dpcm(0)]
    );
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 195);
}

// =============================================================================
// Intensity track only
// =============================================================================

#[test]
fn intensity_track_starts_at_zero() {
    // `last_is = 0` initially, independent of `global_gain`.
    let sfb_cb = vec![vec![INTENSITY_HCB]];
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::IsPos(0)]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 200).unwrap();
    assert_eq!(sfd.entries[0][0], ScaleFactorEntry::Intensity(0));
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 200);
}

#[test]
fn intensity_track_independent_of_global_gain() {
    // Same intensity-position sequence under two different
    // global_gain values produces identical wire DPCM deltas.
    let sfb_cb = vec![vec![INTENSITY_HCB, INTENSITY_HCB2]];
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::IsPos(5),
        AbsoluteScaleFactorEntry::IsPos(-3),
    ]]);
    let sfd_a = differentiate(&abs_in, &sfb_cb, 50).unwrap();
    let sfd_b = differentiate(&abs_in, &sfb_cb, 200).unwrap();
    assert_eq!(sfd_a, sfd_b, "intensity track ignores global_gain");
    assert_eq!(
        sfd_a.entries[0],
        vec![
            ScaleFactorEntry::Intensity(5),
            ScaleFactorEntry::Intensity(-8),
        ]
    );
}

#[test]
fn intensity_track_boundary_deltas() {
    // ±60 are still the bounds — intensity uses the same Table 4.A.1
    // codebook so the delta range is identical to the spectrum track.
    let sfb_cb = vec![vec![INTENSITY_HCB, INTENSITY_HCB, INTENSITY_HCB]];
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::IsPos(0),  // delta 0
        AbsoluteScaleFactorEntry::IsPos(60), // delta +60
        AbsoluteScaleFactorEntry::IsPos(0),  // delta -60
    ]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 100).unwrap();
    assert_eq!(
        sfd.entries[0],
        vec![
            ScaleFactorEntry::Intensity(0),
            ScaleFactorEntry::Intensity(60),
            ScaleFactorEntry::Intensity(-60),
        ]
    );
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 100);
}

// =============================================================================
// PNS noise energy track
// =============================================================================

#[test]
fn pns_first_band_uses_pcm_seed() {
    // Seed = global_gain - NOISE_OFFSET - 256 = 100 - 90 - 256 = -246.
    // Pick abs nrg = -246 + 200 = -46 → 9-bit PCM literal = 200.
    let sfb_cb = vec![vec![NOISE_HCB]];
    let seed = i32::from(100u8) - NOISE_OFFSET - 256;
    assert_eq!(seed, -246);
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::NoiseNrg(-46)]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 100).unwrap();
    assert_eq!(sfd.entries[0][0], ScaleFactorEntry::NoisePcm(200));
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 100);
}

#[test]
fn pns_subsequent_band_uses_huffman_delta() {
    let sfb_cb = vec![vec![NOISE_HCB, NOISE_HCB, NOISE_HCB]];
    // global_gain = 200 → seed = -146. abs values -146, -100, -110.
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::NoiseNrg(-146),
        AbsoluteScaleFactorEntry::NoiseNrg(-100),
        AbsoluteScaleFactorEntry::NoiseNrg(-110),
    ]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 200).unwrap();
    assert_eq!(
        sfd.entries[0],
        vec![
            ScaleFactorEntry::NoisePcm(0),    // delta 0 from seed
            ScaleFactorEntry::NoiseDpcm(46),  // -100 - (-146) = +46
            ScaleFactorEntry::NoiseDpcm(-10), // -110 - (-100) = -10
        ]
    );
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 200);
}

#[test]
fn pns_noise_pcm_flag_is_frame_scoped() {
    // First PNS band lives in group 0; second PNS band lives in
    // group 1. The seed-vs-Huffman choice depends on whether ANY
    // PNS band fired earlier in the frame — not on the group.
    let sfb_cb = vec![
        vec![NOISE_HCB], // group 0: first PNS band of the frame
        vec![NOISE_HCB], // group 1: second PNS band of the frame
    ];
    // seed = 100 - 90 - 256 = -246. abs values -200, -195.
    let abs_in = abs(vec![
        vec![AbsoluteScaleFactorEntry::NoiseNrg(-200)],
        vec![AbsoluteScaleFactorEntry::NoiseNrg(-195)],
    ]);
    let sfd = differentiate(&abs_in, &sfb_cb, 100).unwrap();
    // Group 0: first PNS band → 9-bit PCM seed, delta = -200 - (-246) = 46.
    assert_eq!(sfd.entries[0][0], ScaleFactorEntry::NoisePcm(46));
    // Group 1: second PNS band → Huffman delta = -195 - (-200) = +5.
    assert_eq!(sfd.entries[1][0], ScaleFactorEntry::NoiseDpcm(5));
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 100);
}

#[test]
fn pns_first_seed_at_9bit_boundaries() {
    // seed = global_gain - 346; verify seed magnitudes at both
    // boundaries of the 9-bit `uimsbf` (0 and 511) round-trip.
    let sfb_cb = vec![vec![NOISE_HCB]];
    // Min seed magnitude: delta = 0 → abs = seed itself.
    let seed = i32::from(0u8) - NOISE_OFFSET - 256;
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::NoiseNrg(seed)]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 0).unwrap();
    assert_eq!(sfd.entries[0][0], ScaleFactorEntry::NoisePcm(0));
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 0);

    // Max seed magnitude: delta = 511 → abs = seed + 511.
    let seed = i32::from(0u8) - NOISE_OFFSET - 256;
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::NoiseNrg(seed + 511)]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 0).unwrap();
    assert_eq!(sfd.entries[0][0], ScaleFactorEntry::NoisePcm(511));
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 0);
}

// =============================================================================
// Three-track independence (interleaved bands)
// =============================================================================

#[test]
fn tracks_run_independently_when_interleaved() {
    // sfb_cb interleaves spectrum, intensity, PNS bands. Each track
    // accumulates only its own deltas — the spectrum track does not
    // see the intensity / PNS deltas, etc.
    //
    // global_gain = 100:
    //  - last_sf  = 100
    //  - last_is  = 0
    //  - last_nrg = -246
    //
    // sequence:
    //   spectrum sf=110 (sf delta +10)
    //   intensity is=5  (is delta +5)
    //   spectrum sf=115 (sf delta +5, NOT seeing the is delta)
    //   PNS first nrg=-200 (9-bit literal seed delta = 46)
    //   intensity is=10 (is delta +5, NOT seeing the spectrum/PNS)
    //   PNS subsequent nrg=-190 (Huffman delta +10)
    //   spectrum sf=120 (sf delta +5)
    let sfb_cb = vec![vec![
        3u8,            // spectrum
        INTENSITY_HCB,  // intensity
        3,              // spectrum
        NOISE_HCB,      // PNS first
        INTENSITY_HCB2, // intensity
        NOISE_HCB,      // PNS subsequent
        3,              // spectrum
    ]];
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::Sf(110),
        AbsoluteScaleFactorEntry::IsPos(5),
        AbsoluteScaleFactorEntry::Sf(115),
        AbsoluteScaleFactorEntry::NoiseNrg(-200),
        AbsoluteScaleFactorEntry::IsPos(10),
        AbsoluteScaleFactorEntry::NoiseNrg(-190),
        AbsoluteScaleFactorEntry::Sf(120),
    ]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 100).unwrap();
    assert_eq!(
        sfd.entries[0],
        vec![
            ScaleFactorEntry::Dpcm(10),      // 110 - 100
            ScaleFactorEntry::Intensity(5),  // 5 - 0
            ScaleFactorEntry::Dpcm(5),       // 115 - 110 (spectrum track only)
            ScaleFactorEntry::NoisePcm(46),  // -200 - (-246) = 46 (9-bit seed)
            ScaleFactorEntry::Intensity(5),  // 10 - 5 (intensity track only)
            ScaleFactorEntry::NoiseDpcm(10), // -190 - (-200) = 10
            ScaleFactorEntry::Dpcm(5),       // 120 - 115 (spectrum track only)
        ]
    );
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 100);
}

#[test]
fn tracks_skip_zero_hcb_bands() {
    // Zero-HCB bands carry no record; the accumulator must not
    // mistake the gap for a delta.
    let sfb_cb = vec![vec![ZERO_HCB, 3u8, ZERO_HCB, 4, ZERO_HCB]];
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::Sf(100), // first non-ZERO band
        AbsoluteScaleFactorEntry::Sf(105), // second non-ZERO band
    ]]);
    let sfd = differentiate(&abs_in, &sfb_cb, 100).unwrap();
    assert_eq!(
        sfd.entries[0],
        vec![ScaleFactorEntry::Dpcm(0), ScaleFactorEntry::Dpcm(5)]
    );
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 100);
}

#[test]
fn tracks_persist_across_window_groups() {
    // Spectrum / intensity / PNS accumulators are FRAME-scoped, not
    // group-scoped. A spectrum band in group 1 differentiates
    // against the LAST spectrum band of group 0.
    let sfb_cb = vec![
        vec![3u8, INTENSITY_HCB], // group 0
        vec![3, INTENSITY_HCB],   // group 1
    ];
    // gg=100 → last_sf=100, last_is=0.
    let abs_in = abs(vec![
        vec![
            AbsoluteScaleFactorEntry::Sf(105),  // sf delta +5; new last_sf=105
            AbsoluteScaleFactorEntry::IsPos(3), // is delta +3; new last_is=3
        ],
        vec![
            AbsoluteScaleFactorEntry::Sf(108),  // sf delta +3 (against 105)
            AbsoluteScaleFactorEntry::IsPos(7), // is delta +4 (against 3)
        ],
    ]);
    let sfd = differentiate(&abs_in, &sfb_cb, 100).unwrap();
    assert_eq!(
        sfd.entries[0],
        vec![ScaleFactorEntry::Dpcm(5), ScaleFactorEntry::Intensity(3)]
    );
    assert_eq!(
        sfd.entries[1],
        vec![ScaleFactorEntry::Dpcm(3), ScaleFactorEntry::Intensity(4)]
    );
    differentiate_accumulate_roundtrip(&sfd, &sfb_cb, 100);
}

// =============================================================================
// Spec-pseudocode lockstep: trace `accumulate()` against the
// §4.6.2.3.2 / §4.6.8.1.4 / §4.6.13 pseudocode directly.
// =============================================================================

#[test]
fn spec_pseudocode_lockstep_spectrum_only() {
    // Mirror the §4.6.2.3.2 pseudocode with hand-walked state.
    // dpcm_sf deltas: +5, +5, -3, +2.
    let sfb_cb = vec![vec![1u8, 2, 3, 4]];
    let sfd = sfd(vec![vec![
        ScaleFactorEntry::Dpcm(5),
        ScaleFactorEntry::Dpcm(5),
        ScaleFactorEntry::Dpcm(-3),
        ScaleFactorEntry::Dpcm(2),
    ]]);
    let abs = accumulate(&sfd, &sfb_cb, 100).unwrap();
    // last_sf := 100; then 100+5=105, 105+5=110, 110-3=107, 107+2=109.
    assert_eq!(
        abs.entries[0],
        vec![
            AbsoluteScaleFactorEntry::Sf(105),
            AbsoluteScaleFactorEntry::Sf(110),
            AbsoluteScaleFactorEntry::Sf(107),
            AbsoluteScaleFactorEntry::Sf(109),
        ]
    );
}

#[test]
fn spec_pseudocode_lockstep_pns_first_then_subsequent() {
    // §4.6.13 first PNS band: nrg = (gg - NOISE_OFFSET - 256) +
    // dpcm_noise_nrg[first]. Subsequent: nrg += huffman delta.
    let sfb_cb = vec![vec![NOISE_HCB, NOISE_HCB, NOISE_HCB]];
    let sfd = sfd(vec![vec![
        ScaleFactorEntry::NoisePcm(300), // first PNS, literal 300
        ScaleFactorEntry::NoiseDpcm(7),
        ScaleFactorEntry::NoiseDpcm(-12),
    ]]);
    let abs = accumulate(&sfd, &sfb_cb, 150).unwrap();
    // seed = 150 - 90 - 256 = -196.
    // first  : -196 + 300 = 104
    // second : 104 + 7    = 111
    // third  : 111 - 12   = 99
    assert_eq!(
        abs.entries[0],
        vec![
            AbsoluteScaleFactorEntry::NoiseNrg(104),
            AbsoluteScaleFactorEntry::NoiseNrg(111),
            AbsoluteScaleFactorEntry::NoiseNrg(99),
        ]
    );
}

// =============================================================================
// Rejection: encoder-side invariant violations
// =============================================================================

#[test]
fn differentiate_rejects_spectrum_delta_overflow() {
    // sf 100 → 200 is a +100 delta, exceeds +60 cap.
    let sfb_cb = vec![vec![1u8, 1]];
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::Sf(100),
        AbsoluteScaleFactorEntry::Sf(200),
    ]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn differentiate_rejects_spectrum_delta_underflow() {
    // sf 200 → 100 is a -100 delta, exceeds -60 cap.
    let sfb_cb = vec![vec![1u8, 1]];
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::Sf(200),
        AbsoluteScaleFactorEntry::Sf(100),
    ]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 200),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn differentiate_rejects_intensity_delta_overflow() {
    let sfb_cb = vec![vec![INTENSITY_HCB, INTENSITY_HCB]];
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::IsPos(0),
        AbsoluteScaleFactorEntry::IsPos(100),
    ]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn differentiate_rejects_pns_first_seed_overflow() {
    // First PNS abs nrg = seed + 600 → 9-bit field cannot hold 600.
    let sfb_cb = vec![vec![NOISE_HCB]];
    let seed = i32::from(100u8) - NOISE_OFFSET - 256;
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::NoiseNrg(seed + 600)]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn differentiate_rejects_pns_first_seed_negative() {
    // 9-bit `uimsbf` field is unsigned 0..=511; abs nrg < seed
    // requires a negative delta which the wire cannot represent.
    let sfb_cb = vec![vec![NOISE_HCB]];
    let seed = i32::from(100u8) - NOISE_OFFSET - 256;
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::NoiseNrg(seed - 1)]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn differentiate_rejects_pns_subsequent_delta_overflow() {
    let sfb_cb = vec![vec![NOISE_HCB, NOISE_HCB]];
    let seed = i32::from(100u8) - NOISE_OFFSET - 256;
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::NoiseNrg(seed),
        AbsoluteScaleFactorEntry::NoiseNrg(seed + 100), // delta +100
    ]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn differentiate_rejects_variant_codebook_mismatch() {
    // Spectrum-track abs entry paired with an intensity band.
    let sfb_cb = vec![vec![INTENSITY_HCB]];
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::Sf(100)]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );

    // Intensity entry on a spectrum band.
    let sfb_cb = vec![vec![3u8]];
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::IsPos(0)]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );

    // NoiseNrg on a spectrum band.
    let sfb_cb = vec![vec![3u8]];
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::NoiseNrg(0)]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn differentiate_rejects_outer_length_mismatch() {
    let sfb_cb = vec![vec![3u8], vec![3]];
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::Sf(100)]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn differentiate_rejects_inner_count_mismatch() {
    // Group has 2 non-ZERO_HCB bands but abs supplies only 1 entry.
    let sfb_cb = vec![vec![3u8, 4]];
    let abs_in = abs(vec![vec![AbsoluteScaleFactorEntry::Sf(100)]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );

    // Group has 1 non-ZERO_HCB band but abs supplies 2 entries.
    let sfb_cb = vec![vec![3u8]];
    let abs_in = abs(vec![vec![
        AbsoluteScaleFactorEntry::Sf(100),
        AbsoluteScaleFactorEntry::Sf(105),
    ]]);
    assert_eq!(
        differentiate(&abs_in, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

// =============================================================================
// Rejection: decoder-side invariant violations
// =============================================================================

#[test]
fn accumulate_rejects_sf_track_underflow() {
    // dpcm 0 against global_gain=0 stays at 0 (OK), then dpcm -1
    // would underflow to -1 (out of range).
    let sfb_cb = vec![vec![1u8, 1]];
    let sfd = sfd(vec![vec![
        ScaleFactorEntry::Dpcm(0),
        ScaleFactorEntry::Dpcm(-1),
    ]]);
    assert_eq!(
        accumulate(&sfd, &sfb_cb, 0),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn accumulate_rejects_sf_track_overflow() {
    // global_gain 255 + dpcm +1 = 256 (out of 0..=255).
    let sfb_cb = vec![vec![1u8]];
    let sfd = sfd(vec![vec![ScaleFactorEntry::Dpcm(1)]]);
    assert_eq!(
        accumulate(&sfd, &sfb_cb, 255),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn accumulate_rejects_outer_length_mismatch() {
    let sfb_cb = vec![vec![1u8], vec![1u8]];
    let sfd = sfd(vec![vec![ScaleFactorEntry::Dpcm(0)]]);
    assert_eq!(
        accumulate(&sfd, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn accumulate_rejects_inner_count_mismatch() {
    // Group has 2 non-ZERO_HCB bands but sfd has 1 entry.
    let sfb_cb = vec![vec![1u8, 1]];
    let sfd = sfd(vec![vec![ScaleFactorEntry::Dpcm(0)]]);
    assert_eq!(
        accumulate(&sfd, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn accumulate_rejects_variant_codebook_mismatch() {
    // Intensity entry on a spectrum band.
    let sfb_cb = vec![vec![3u8]];
    let sfd = sfd(vec![vec![ScaleFactorEntry::Intensity(0)]]);
    assert_eq!(
        accumulate(&sfd, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn accumulate_rejects_pns_pcm_after_flag_cleared() {
    // First PNS band sets the flag; a second NoisePcm in the same
    // frame is a wire-illegal record set.
    let sfb_cb = vec![vec![NOISE_HCB, NOISE_HCB]];
    let sfd = sfd(vec![vec![
        ScaleFactorEntry::NoisePcm(0),
        ScaleFactorEntry::NoisePcm(0),
    ]]);
    assert_eq!(
        accumulate(&sfd, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

#[test]
fn accumulate_rejects_pns_huffman_before_flag_cleared() {
    // First PNS band must use the 9-bit PCM seed.
    let sfb_cb = vec![vec![NOISE_HCB]];
    let sfd = sfd(vec![vec![ScaleFactorEntry::NoiseDpcm(0)]]);
    assert_eq!(
        accumulate(&sfd, &sfb_cb, 100),
        Err(Error::ScaleFactorAccumulatorInvalid)
    );
}

// =============================================================================
// Composite: a realistic-shaped frame round-trips
// =============================================================================

#[test]
fn composite_eight_short_frame_three_track_roundtrip() {
    // 8 window groups, varied band content per group.
    let sfb_cb: Vec<Vec<u8>> = vec![
        // group 0: spectrum + ZERO + intensity
        vec![3, ZERO_HCB, INTENSITY_HCB],
        // group 1: spectrum
        vec![4, 5],
        // group 2: PNS first + ZERO + spectrum
        vec![NOISE_HCB, ZERO_HCB, 6],
        // group 3: intensity + PNS subsequent
        vec![INTENSITY_HCB2, NOISE_HCB],
        // group 4: empty
        vec![ZERO_HCB, ZERO_HCB],
        // group 5: spectrum + intensity + spectrum
        vec![7, INTENSITY_HCB, 8],
        // group 6: PNS subsequent + intensity
        vec![NOISE_HCB, INTENSITY_HCB],
        // group 7: spectrum
        vec![9],
    ];
    // Build a manually-tracked absolute sequence so we know all
    // deltas fit `-60..=+60`.
    let abs_in = abs(vec![
        vec![
            AbsoluteScaleFactorEntry::Sf(120), // sf delta +20
            AbsoluteScaleFactorEntry::IsPos(3),
        ],
        vec![
            AbsoluteScaleFactorEntry::Sf(110), // sf delta -10
            AbsoluteScaleFactorEntry::Sf(115), // sf delta +5
        ],
        vec![
            AbsoluteScaleFactorEntry::NoiseNrg(-200), // first PNS, seed delta = 46
            AbsoluteScaleFactorEntry::Sf(118),        // sf delta +3
        ],
        vec![
            AbsoluteScaleFactorEntry::IsPos(8),       // is delta +5
            AbsoluteScaleFactorEntry::NoiseNrg(-195), // PNS delta +5
        ],
        vec![],
        vec![
            AbsoluteScaleFactorEntry::Sf(120),   // sf delta +2
            AbsoluteScaleFactorEntry::IsPos(15), // is delta +7
            AbsoluteScaleFactorEntry::Sf(125),   // sf delta +5
        ],
        vec![
            AbsoluteScaleFactorEntry::NoiseNrg(-190), // PNS delta +5
            AbsoluteScaleFactorEntry::IsPos(13),      // is delta -2
        ],
        vec![
            AbsoluteScaleFactorEntry::Sf(130), // sf delta +5
        ],
    ]);
    let global_gain = 100;
    accumulate_differentiate_roundtrip(&abs_in, &sfb_cb, global_gain);

    // Sanity: the second PNS band of the frame is `NoiseDpcm`, not
    // `NoisePcm` — verify the flag-clear semantics in a realistic
    // multi-group layout.
    let sfd = differentiate(&abs_in, &sfb_cb, global_gain).unwrap();
    assert!(matches!(sfd.entries[2][0], ScaleFactorEntry::NoisePcm(_)));
    assert!(matches!(sfd.entries[3][1], ScaleFactorEntry::NoiseDpcm(_)));
    assert!(matches!(sfd.entries[6][0], ScaleFactorEntry::NoiseDpcm(_)));
}

#[test]
fn noise_offset_constant_is_ninety() {
    // §4.6.13 fixes NOISE_OFFSET at 90; verify the constant.
    assert_eq!(NOISE_OFFSET, 90);
}

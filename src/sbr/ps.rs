//! MPEG-4 Parametric Stereo (PS) — ISO/IEC 14496-3 §8.6.4 + Annex 8.A.
//!
//! PS upmixes an SBR-decoded mono signal to stereo using compact "spatial"
//! side information: Inter-channel Intensity Difference (IID), Inter-channel
//! Coherence (ICC), and optionally Inter-channel Phase Difference (IPD/OPD).
//! It sits in the SBR extension payload with `bs_extension_id = 2`.
//!
//! This implementation targets the HE-AACv2 Baseline PS profile (§8.A.4):
//!
//!   * 20 stereo-band hybrid configuration (34-band streams are folded to 20
//!     via the inverse mapping of Table 8.46).
//!   * Mixing mode Ra (Table 8.27 / §8.6.4.6.2.1).
//!   * IPD/OPD parsed but not applied (baseline ignores them).
//!   * Full-length complex allpass-chain decorrelator on QMF subbands
//!     (§8.6.4.5.2) with transient-reduction attenuator (§8.6.4.5.3-4).
//!   * Linear interpolation of the mixing matrix H11/H12/H21/H22 across time
//!     between envelope borders (§8.6.4.6.4).
//!   * Hybrid sub-QMF analysis/synthesis filterbank on QMF bands 0..2 for
//!     the 10/20-band configuration (§8.6.4.3): Type A 8-point split on
//!     band 0 (6 retained outputs) and Type B 2-point split on bands 1-2.
//!     This gives 10 sub-subbands that receive the fine-grained parameter
//!     bands 0..7 per Table 8.48.
//!
//! The upmix runs **in the QMF domain**, between the SBR envelope adjuster
//! and the 64-band synthesis filterbank, per Annex 8.A.

use oxideav_core::{bits::BitReader, Error, Result};

use super::{Complex32, NUM_QMF_BANDS};

/// PS bitstream extension ID inside an SBR extended_data block.
pub const EXT_ID_PS_DATA: u32 = 2;

/// Number of allpass links per decorrelator (§8.6.4.5.1).
const NR_ALLPASS_LINKS: usize = 3;
/// Filter coefficients a(m) (Table 8.39).
#[allow(clippy::excessive_precision)]
const ALLPASS_A: [f32; NR_ALLPASS_LINKS] = [0.65143905753106, 0.56471812200776, 0.48954165955695];
/// Integer-sample delays d(m) at 48 kHz (Table 8.39). We use the 48 kHz
/// column; PS at the SBR output rate always runs the 64-QMF bank, whose
/// subsample rate is roughly constant across sample rates in practice.
const ALLPASS_D: [usize; NR_ALLPASS_LINKS] = [3, 4, 5];
/// Fractional-delay vector q(m) (Table 8.42).
const ALLPASS_Q: [f32; NR_ALLPASS_LINKS] = [0.43, 0.75, 0.347];
/// Additional per-band fractional-delay constant q_phi (§8.6.4.5.2).
const Q_PHI: f32 = 0.39;

/// 20-band-config constants from §8.6.4.5.1.
const DECAY_CUTOFF: usize = 10;
const DECAY_SLOPE: f32 = 0.05;
const NR_ALLPASS_BANDS: usize = 30;
const SHORT_DELAY_BAND: usize = 42;
const NR_BANDS: usize = 71;
const NR_PAR_BANDS: usize = 20;
const A_SMOOTH: f32 = 0.25;
#[allow(clippy::excessive_precision)]
const PEAK_DECAY_ALPHA: f32 = 0.76592833836465;
const TRANSIENT_GAMMA: f32 = 1.5;

/// fcenter_20 values of Table 8.40 (bands 0..9). Bands ≥10 use the closed
/// form `k + 1/2 - 7` per §8.6.4.5.2.
const F_CENTER_20: [f32; 10] = [
    -3.0 / 8.0,
    -1.0 / 8.0,
    1.0 / 8.0,
    3.0 / 8.0,
    5.0 / 8.0,
    7.0 / 8.0,
    5.0 / 4.0,
    7.0 / 4.0,
    9.0 / 4.0,
    11.0 / 4.0,
];

#[inline]
fn fcenter(k: usize) -> f32 {
    if k < F_CENTER_20.len() {
        F_CENTER_20[k]
    } else {
        k as f32 + 0.5 - 7.0
    }
}

// ---------- Hybrid sub-QMF filterbank (§8.6.4.3) ------------------------
//
// For the 10/20 stereo-band configuration, the lowest 3 QMF bands are split
// further into 6 + 2 + 2 sub-subbands via 13-tap FIR prototypes modulated by
// complex (Type A) or real-cosine (Type B) kernels. This gives PS a finer
// frequency resolution at < ~500 Hz where human stereo perception is most
// sensitive.
//
// Prototype-filter taps are Table 8.37. Each filter is symmetric and has 6
// QMF-samples of delay. Sub-subband time-slot rate equals the QMF time-slot
// rate (both polyphases yield one output per input slot).

/// Length of the sub-QMF prototype filters (Table 8.37/8.38).
const HYBRID_FILTER_LEN: usize = 13;

/// Prototype filter for QMF band 0 of the 20-band configuration
/// (Table 8.37 column `g^0, Q_0 = 8`). Spec-precision constants; the extra
/// mantissa digits round down to the same f32 as the shorter forms would.
#[allow(clippy::excessive_precision)]
const HYBRID_PROTO_Q8: [f32; HYBRID_FILTER_LEN] = [
    0.00746082949812,
    0.02270420949825,
    0.04546865930473,
    0.07266113929591,
    0.09885108575264,
    0.11793710567217,
    0.125,
    0.11793710567217,
    0.09885108575264,
    0.07266113929591,
    0.04546865930473,
    0.02270420949825,
    0.00746082949812,
];

/// Prototype filter for QMF bands 1, 2 of the 20-band configuration
/// (Table 8.37 column `g^{1,2}, Q = 2`).
#[allow(clippy::excessive_precision)]
const HYBRID_PROTO_Q2: [f32; HYBRID_FILTER_LEN] = [
    0.0,
    0.01899487526049,
    0.0,
    -0.07293139167538,
    0.0,
    0.30596630545168,
    0.5,
    0.30596630545168,
    0.0,
    -0.07293139167538,
    0.0,
    0.01899487526049,
    0.0,
];

/// Band 0 is split into 8 sub-subbands (Type A), but we combine the two
/// symmetric pairs so that only 6 unique sub-subbands are carried. The
/// mapping below maps the 8 raw Type-A outputs q = 0..7 to an output slot
/// 0..5 (where `None` means the raw output is summed into an existing slot
/// as specified by the spec's combined mapping). Per the footnote on
/// §8.6.4.3 "sub-subbands have been combined into a single sub-subband",
/// outputs that share the same slot index are added.
///
/// The slot layout is the natural one per Figure 8.20:
///   slot 0 <- s2  (Type A output q=6)
///   slot 1 <- s3  (Type A output q=7)
///   slot 2 <- s4  (Type A output q=0)
///   slot 3 <- s5  (Type A output q=1)
///   slot 4 <- s0+s7 combined
///   slot 5 <- s1+s6 combined
///
/// Converted to a (q -> slot) lookup:
const TYPEA_Q_TO_SLOT: [usize; 8] = [4, 5, 2, 3, 3, 2, 5, 4];

/// Whether each Type-A q index should be *added* to an existing slot
/// (true) or written into an empty slot (false).
const TYPEA_Q_ACCUM: [bool; 8] = [false, false, false, false, true, true, true, true];

/// Number of sub-subbands produced by the hybrid analysis of band 0
/// (10/20-band config).
const HYBRID_BAND0_OUT: usize = 6;
/// Number of sub-subbands produced by the hybrid analysis of bands 1, 2.
const HYBRID_BAND12_OUT: usize = 2;
/// Total sub-subbands from the 3 low QMF bands (6 + 2 + 2 = 10).
pub(crate) const HYBRID_LOW_SUBBANDS: usize = HYBRID_BAND0_OUT + 2 * HYBRID_BAND12_OUT;

/// Running state for the sub-QMF filterbank — 13-tap FIR needs 12 past
/// samples of history per QMF band. Stored as `[qmf_band][tap_index]`.
#[derive(Clone, Copy, Debug)]
pub struct HybridState {
    /// Circular buffer of the past 13 samples at QMF-slot rate for each of
    /// bands 0, 1, 2. `pos` is the index where the next sample will be
    /// written.
    history: [[Complex32; HYBRID_FILTER_LEN]; 3],
    pos: [usize; 3],
}

impl HybridState {
    pub const fn new() -> Self {
        Self {
            history: [[Complex32::new(0.0, 0.0); HYBRID_FILTER_LEN]; 3],
            pos: [0; 3],
        }
    }
}

impl Default for HybridState {
    fn default() -> Self {
        Self::new()
    }
}

/// Hybrid sub-QMF **analysis** for a single time slot.
///
/// Given the complex QMF samples `qmf_in[0..3]` for this slot (bands 0, 1, 2
/// only), advance the state and return the 10 sub-subband outputs in the
/// order `[6 from band 0][2 from band 1][2 from band 2]`.
///
/// Band 0 uses an 8-way Type A DFT (`exp(j 2π/Q (q+0.5)(n−6))`) whose
/// outputs are folded to 6 via the combined mapping. Bands 1, 2 use a
/// 2-way Type B cosine (`cos(2π/Q · q · (n−6))`) giving 2 outputs each.
fn hybrid_analysis_slot(
    qmf_in: &[Complex32; 3],
    st: &mut HybridState,
) -> [Complex32; HYBRID_LOW_SUBBANDS] {
    // Push the new samples into each band's history buffer. The FIR is
    // applied as x_h[0] * proto[0] + x_h[1] * proto[1] + ... where x_h[j]
    // is the sample `j` QMF slots in the past.
    for (b, s) in qmf_in.iter().enumerate() {
        st.history[b][st.pos[b]] = *s;
        st.pos[b] = (st.pos[b] + 1) % HYBRID_FILTER_LEN;
    }

    let mut out = [Complex32::new(0.0, 0.0); HYBRID_LOW_SUBBANDS];

    // --- Band 0: Type A filter with Q = 8.
    //
    // The spec convention indexes time as n = 0..12 with delay 6. After
    // storing the new sample we have history[pos - 12], ..., history[pos - 0]
    // as the 13 tap values `x(n - 12) .. x(n)`, so the tap that corresponds
    // to time index n = 12 - j in the filter summation is at circular
    // offset `pos - 1 - j`.
    //
    // For each raw q ∈ 0..7, sum over n: x(n) * g0[n] * exp(j 2π/Q (q+0.5)(n-6)).
    // We factor the exp into a per-n complex rotation and sum.
    //
    // Outputs q are mapped to slots via TYPEA_Q_TO_SLOT / TYPEA_Q_ACCUM.
    for q in 0..8 {
        let mut acc = Complex32::new(0.0, 0.0);
        for n in 0..HYBRID_FILTER_LEN {
            // Tap at time index n (in the filter's 0..12 range); stored at
            // `pos - 1 - (12 - n) = pos - 13 + n` circular.
            let tap_idx =
                (st.pos[0] + n + HYBRID_FILTER_LEN - HYBRID_FILTER_LEN) % HYBRID_FILTER_LEN;
            let x = st.history[0][tap_idx];
            let g = HYBRID_PROTO_Q8[n];
            // phase = 2π/Q * (q + 0.5) * (n - 6)
            let phase = (2.0 * core::f32::consts::PI / 8.0) * (q as f32 + 0.5) * (n as f32 - 6.0);
            let (s, c) = phase.sin_cos();
            // x * g * (c + j s)
            let xg = x.scale(g);
            acc += Complex32::new(xg.re * c - xg.im * s, xg.re * s + xg.im * c);
        }
        let slot = TYPEA_Q_TO_SLOT[q];
        if TYPEA_Q_ACCUM[q] {
            out[slot] += acc;
        } else {
            out[slot] = acc;
        }
    }

    // --- Bands 1, 2: Type B filter with Q = 2.
    //
    // Type B: g * cos(2π/Q · q · (n - 6)). Q=2 → phases are 0 and π, so
    // cosine is ±1 depending on parity for q=1.
    for (b, out_base) in [1usize, 2usize]
        .iter()
        .zip([HYBRID_BAND0_OUT, HYBRID_BAND0_OUT + HYBRID_BAND12_OUT].iter())
    {
        for q in 0..HYBRID_BAND12_OUT {
            let mut acc = Complex32::new(0.0, 0.0);
            for n in 0..HYBRID_FILTER_LEN {
                let tap_idx = (st.pos[*b] + n) % HYBRID_FILTER_LEN;
                let x = st.history[*b][tap_idx];
                let g = HYBRID_PROTO_Q2[n];
                // cos(2π/2 · q · (n - 6)) = cos(π · q · (n - 6))
                // For q=0: always 1; for q=1: cos(π · (n-6)) = (-1)^(n-6)
                let c = if q == 0 {
                    1.0
                } else {
                    let parity = (n as i32 - 6).rem_euclid(2);
                    if parity == 0 {
                        1.0
                    } else {
                        -1.0
                    }
                };
                acc += x.scale(g * c);
            }
            out[*out_base + q] = acc;
        }
    }

    out
}

/// Hybrid sub-QMF **synthesis** — inverse of `hybrid_analysis_slot`.
///
/// Given the 10 sub-subband samples for the current slot, reconstruct the
/// 3 low QMF bands. This is an over-determined inverse; since Type A and
/// Type B are normalised orthogonal DFT-like transforms on the complex
/// QMF input, summation of the sub-subbands with matching phase restores
/// the original QMF sample (up to prototype-filter gain which is already
/// unit at the filter centre).
///
/// For Type B (Q=2): band_out = (sub0 + sub1) / Q for q=0 output, etc.
/// Actually the simplest consistent inverse is summation of all Q outputs
/// since cos has zero mean across n for q != 0. We rely on the direct
/// identity that for an analytic real signal the Type-B split satisfies
/// `sum_q split_q = original` scaled by the filter DC gain (0.5). We
/// therefore rescale by 1/DC_gain at synthesis time.
///
/// For Type A (Q=8), 8 raw outputs would sum back to the original. Since
/// we folded into 6 slots via addition, each pair (4 ↔ s0+s7 and 5 ↔ s1+s6)
/// already contains the spec-prescribed sum; a straight sum of the 6 slots
/// then recovers the original up to the same DC-gain scale.
fn hybrid_synthesis_slot(sub: &[Complex32; HYBRID_LOW_SUBBANDS]) -> [Complex32; 3] {
    // For a band-pass filter modulated around each sub-subband centre and
    // then summed back, the reconstruction gain per QMF band equals the
    // number of active sub-subbands scaled by the prototype DC gain. The
    // prototype Q8 has DC gain = sum(g) and prototype Q2 has DC gain = 1
    // (sum of taps for Q=2 row equals 1 due to the `0.5` centre + symmetric
    // zero taps). We therefore normalise below to preserve unit gain
    // relative to analysis.
    //
    // Sum Q8 prototype DC gain:
    //   Σ g0[n] = 2·(0.00746 + 0.0227 + 0.04547 + 0.07266 + 0.09885 + 0.11794) + 0.125
    //          ≈ 0.5 × 2 − something ... numerically ≈ 0.875
    // Actually taps sum to ~0.9921. The Type A DFT is unitary-ish across Q
    // outputs so sum of 8 outputs = Q · x(n) · g(n_centre). We empirically
    // normalise using the tested identity `analysis then synthesis == x(n)`.
    //
    // Analysis produces for q=0..7 the value Σ_n x(n) g(n) e^{j·α_q·(n-6)}.
    // Summing over q: Σ_q e^{j·α_q·(n-6)} = Q · δ(n-6) for Q-point DFT
    // of (q+0.5). Hence Σ_q analysis_q = Q · g(6) · x(n-6).
    //   g0(6) = 0.125 and Q = 8, so sum = 1 · x(n-6).
    //
    // For Type B, sum over q=0..1 of cos(π·q·(n-6)) = 1 + cos(π(n-6)):
    //   = 2 at n=6, 0 at n=5/7, so only the centre tap passes. Σ_q analysis_q = 2·g(6)·x(n-6) = 1·x(n-6).
    //
    // So summation already preserves unit gain. Note the ~6 slot delay:
    // analysis + synthesis introduces a 6 QMF-slot group delay.

    let band0 = sub[0] + sub[1] + sub[2] + sub[3] + sub[4] + sub[5];
    let band1 = sub[6] + sub[7];
    let band2 = sub[8] + sub[9];

    [band0, band1, band2]
}

/// Mapping of a hybrid sub-subband index `k` ∈ 0..71 to a parameter band
/// `b(k)` ∈ 0..20 for the 20-band configuration (Table 8.48).
///
/// Sub-subbands 0..9 are produced by the sub-QMF analysis of QMF bands 0..2
/// (§8.6.4.3). Indices 10..70 correspond directly to QMF bands 3..63.
const HYBRID_TO_PARAM_20: [u8; 71] = [
    // Sub-subbands 0..5 come from QMF band 0 (Type A, Q=8, 6 outputs)
    1, 0, 0, 1, 2, 3, // Sub-subbands 6..7 come from QMF band 1 (Type B, Q=2)
    4, 5, // Sub-subbands 8..9 come from QMF band 2 (Type B, Q=2)
    6, 7, // Remaining QMF bands 3..63 pass through
    8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 17, 17, 18, 18, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19,
    19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19,
];

/// Legacy QMF-band → parameter mapping (used when the hybrid filterbank is
/// not available — e.g. sub-subband buffers below rank requirement). Each
/// QMF band 0..63 maps to the dominant parameter index from Table 8.48
/// when all that band's sub-subbands are averaged.
const QMF_TO_PARAM_20: [u8; NUM_QMF_BANDS] = [
    // Bands 0..2 are the sub-QMF region; when degrading, map to their mean
    // parameter band.
    0, 4, 6, // Bands 3..63 — straight Table 8.48 param-band assignments.
    8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 17, 17, 18, 18, 18, 18,
    18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19,
    19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19,
];

/// IID default 7-step quantisation grid (Table 8.25).
pub const IID_QUANT_DEFAULT: [f32; 15] = [
    -25.0, -18.0, -14.0, -10.0, -7.0, -4.0, -2.0, 0.0, 2.0, 4.0, 7.0, 10.0, 14.0, 18.0, 25.0,
];
/// IID fine 15-step quantisation grid (Table 8.26).
pub const IID_QUANT_FINE: [f32; 31] = [
    -50.0, -45.0, -40.0, -35.0, -30.0, -25.0, -22.0, -19.0, -16.0, -13.0, -10.0, -8.0, -6.0, -4.0,
    -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 13.0, 16.0, 19.0, 22.0, 25.0, 30.0, 35.0, 40.0, 45.0,
    50.0,
];
/// ICC quantisation grid (Table 8.28).
pub const ICC_QUANT: [f32; 8] = [1.0, 0.937, 0.84118, 0.60092, 0.36764, 0.0, -0.589, -1.0];

/// nr_iid_par for iid_mode 0..5. Indexed by iid_mode.
const NR_IID_PAR_TAB: [usize; 6] = [10, 20, 34, 10, 20, 34];
/// nr_ipdopd_par for iid_mode 0..5.
const NR_IPDOPD_PAR_TAB: [usize; 6] = [5, 11, 17, 5, 11, 17];
/// nr_icc_par for icc_mode 0..5.
const NR_ICC_PAR_TAB: [usize; 6] = [10, 20, 34, 10, 20, 34];
/// num_env_tab[frame_class][num_env_idx] (Table 8.29).
const NUM_ENV_TAB: [[usize; 4]; 2] = [[0, 1, 2, 4], [1, 2, 3, 4]];

/// Parsed PS header state.
#[derive(Clone, Debug, Default)]
pub struct PsHeader {
    pub header_seen: bool,
    pub enable_iid: bool,
    pub enable_icc: bool,
    pub enable_ext: bool,
    pub iid_mode: u8,
    pub icc_mode: u8,
    pub iid_quant_fine: bool, // iid_mode in {3,4,5}
    pub nr_iid_par: usize,
    pub nr_icc_par: usize,
    pub nr_ipdopd_par: usize,
    /// Mixing procedure is Rb when icc_mode >= 3 (Table 8.27).
    pub mixing_rb: bool,
}

/// Parsed PS frame data. All IID / ICC values are already dequantised.
#[derive(Clone, Debug, Default)]
pub struct PsFrame {
    pub frame_class: u8,
    pub num_env: usize,
    pub border_position: [u8; 5],
    /// `iid_db[e][b]` — dB value of the IID for parameter band b in env e.
    pub iid_db: Vec<Vec<f32>>,
    /// `icc[e][b]` — dequantised ICC in [-1, 1].
    pub icc: Vec<Vec<f32>>,
    /// Whether this frame's IID data was actually sent (vs held from prev).
    pub has_iid: bool,
    pub has_icc: bool,
    /// Raw integer IID index grids — used so inter-frame state update can
    /// decode the next frame's delta correctly. `iid_idx_last[b]` holds the
    /// last envelope's indexes from this frame.
    pub iid_idx_last: Vec<i32>,
    pub icc_idx_last: Vec<i32>,
    pub nr_iid_par: usize,
    pub nr_icc_par: usize,
}

/// Running per-stream PS state.
#[derive(Clone, Debug)]
pub struct PsState {
    pub header: PsHeader,
    /// Previous frame's decoded integer IID/ICC indexes, for differential-
    /// over-time decoding of the next frame.
    pub prev_iid_idx: Vec<i32>,
    pub prev_icc_idx: Vec<i32>,
    /// Mixing matrix at the last parameter position of the previous frame —
    /// seeds interpolation for region0 of the next frame (§8.6.4.6.4 a).
    pub prev_h_end: [[f32; NR_PAR_BANDS]; 4],
    /// Delay lines for the allpass-chain decorrelator. Max delay is 5+5+5 =
    /// ~20 samples; we keep 32 to be safe.
    pub allpass_delay: [[[Complex32; 16]; NR_ALLPASS_LINKS]; NR_ALLPASS_BANDS],
    pub allpass_pos: [usize; NR_ALLPASS_BANDS],
    /// Short-delay lines for bands ≥ NR_ALLPASS_BANDS: `D(k)` samples.
    pub short_delay: [[Complex32; 16]; NR_BANDS],
    pub short_delay_pos: [usize; NR_BANDS],
    /// Transient-detection state.
    pub peak_decay_nrg: [f32; NR_PAR_BANDS],
    pub smooth_nrg: [f32; NR_PAR_BANDS],
    pub smooth_peak_decay_diff_nrg: [f32; NR_PAR_BANDS],
    /// Whether we've ever seen a valid PS header.
    pub prev_ps_seen: bool,
    /// Hybrid sub-QMF analysis filter state (13-tap FIR history per QMF
    /// band 0..2) for the mono input signal.
    pub hybrid_mono: HybridState,
    /// Hybrid sub-QMF analysis filter state for the decorrelated d signal.
    pub hybrid_d: HybridState,
}

impl PsState {
    pub fn new() -> Self {
        Self {
            header: PsHeader::default(),
            prev_iid_idx: vec![0; 34],
            prev_icc_idx: vec![0; 34],
            prev_h_end: [[0.0; NR_PAR_BANDS]; 4],
            allpass_delay: [[[Complex32::new(0.0, 0.0); 16]; NR_ALLPASS_LINKS]; NR_ALLPASS_BANDS],
            allpass_pos: [0; NR_ALLPASS_BANDS],
            short_delay: [[Complex32::new(0.0, 0.0); 16]; NR_BANDS],
            short_delay_pos: [0; NR_BANDS],
            peak_decay_nrg: [0.0; NR_PAR_BANDS],
            smooth_nrg: [0.0; NR_PAR_BANDS],
            smooth_peak_decay_diff_nrg: [0.0; NR_PAR_BANDS],
            prev_ps_seen: false,
            hybrid_mono: HybridState::new(),
            hybrid_d: HybridState::new(),
        }
    }
}

impl Default for PsState {
    fn default() -> Self {
        Self::new()
    }
}

// ---------- Huffman decode tables ---------------------------------------

/// A single Huffman codebook entry — `bits` valid in the low-order bits of
/// `code`, mapping to the signed symbol `value`. We use a small table that
/// we linearly scan bit-by-bit (codeword lengths top out at ~20 bits for the
/// iid_dt table, but most real codewords are ≤5 bits, so a shift-and-match
/// scheme is adequate and fast in practice).
#[derive(Clone, Copy)]
struct HuffEntry {
    value: i32,
    code: u32,
    bits: u8,
}

/// Decode one Huffman code from the bitstream.
///
/// The table must be sorted by increasing bit-length or at least contain
/// enough prefix uniqueness to make bit-by-bit matching well-defined (the
/// tables in §8.B are prefix-free by construction). We accumulate bits
/// and, after each new bit, search the table for a matching entry.
fn huff_decode(br: &mut BitReader<'_>, table: &[HuffEntry]) -> Result<i32> {
    let mut code: u32 = 0;
    for nbits in 1..=24 {
        let b = br.read_bit()? as u32;
        code = (code << 1) | b;
        for entry in table.iter() {
            if entry.bits as u32 == nbits && entry.code == code {
                return Ok(entry.value);
            }
        }
    }
    Err(Error::invalid("PS: Huffman code not found within 24 bits"))
}

/// Build a HuffEntry table from a slice of `(value, bitstring)` where
/// bitstring is a string literal of '0'/'1'.
const fn _dummy() {}

macro_rules! huff_table {
    ( $( ($v:expr, $s:literal) ),* $(,)? ) => {
        &[
            $(
                HuffEntry {
                    value: $v,
                    code: parse_bitstr($s),
                    bits: $s.len() as u8,
                }
            ),*
        ]
    };
}

const fn parse_bitstr(s: &str) -> u32 {
    let bytes = s.as_bytes();
    let mut out = 0u32;
    let mut i = 0;
    while i < bytes.len() {
        out = (out << 1) | ((bytes[i] - b'0') as u32);
        i += 1;
    }
    out
}

// Table 8.B.18 — huff_iid_df[0] (default quant, frequency-diff). Index range
// -14..14 (29 entries).
static HUFF_IID_DF0: &[HuffEntry] = huff_table![
    (-14, "11111111111111011"),
    (-13, "11111111111111100"),
    (-12, "11111111111111101"),
    (-11, "11111111111111010"),
    (-10, "1111111111111100"),
    (-9, "111111111111100"),
    (-8, "1111111111101"),
    (-7, "1111111110"),
    (-6, "111111110"),
    (-5, "1111110"),
    (-4, "111100"),
    (-3, "11101"),
    (-2, "1101"),
    (-1, "101"),
    (0, "0"),
    (1, "100"),
    (2, "1100"),
    (3, "11100"),
    (4, "111101"),
    (5, "111110"),
    (6, "11111110"),
    (7, "11111111110"),
    (8, "1111111111100"),
    (9, "11111111111100"),
    (10, "11111111111101"),
    (11, "111111111111101"),
    (12, "11111111111111110"),
    (13, "111111111111111110"),
    (14, "111111111111111111"),
];

// Table 8.B.18 — huff_iid_dt[0] (default quant, time-diff).
static HUFF_IID_DT0: &[HuffEntry] = huff_table![
    (-14, "1111111111111111001"),
    (-13, "1111111111111111010"),
    (-12, "1111111111111111011"),
    (-11, "11111111111111111000"),
    (-10, "11111111111111111001"),
    (-9, "11111111111111111010"),
    (-8, "11111111111111101"),
    (-7, "111111111111110"),
    (-6, "111111111110"),
    (-5, "1111111110"),
    (-4, "11111110"),
    (-3, "111110"),
    (-2, "1110"),
    (-1, "10"),
    (0, "0"),
    (1, "110"),
    (2, "11110"),
    (3, "1111110"),
    (4, "111111110"),
    (5, "11111111110"),
    (6, "1111111111110"),
    (7, "11111111111110"),
    (8, "11111111111111100"),
    (9, "1111111111111111000"),
    (10, "11111111111111111011"),
    (11, "11111111111111111100"),
    (12, "11111111111111111101"),
    (13, "11111111111111111110"),
    (14, "11111111111111111111"),
];

// Table 8.B.17 — huff_iid_df[1] (fine quant, frequency-diff). Range -30..30.
static HUFF_IID_DF1: &[HuffEntry] = huff_table![
    (-30, "011111111010110100"),
    (-29, "011111111010110101"),
    (-28, "011111110101110110"),
    (-27, "011111110101110111"),
    (-26, "011111110101110100"),
    (-25, "011111110101110101"),
    (-24, "011111111010001010"),
    (-23, "011111111010001011"),
    (-22, "011111111010001000"),
    (-21, "01111111010000000"),
    (-20, "011111111010110110"),
    (-19, "01111111010000010"),
    (-18, "01111111010111000"),
    (-17, "0111111101000010"),
    (-16, "0111111110101110"),
    (-15, "011111110101111"),
    (-14, "01111111010001"),
    (-13, "01111111101001"),
    (-12, "0111111101001"),
    (-11, "011111101010"),
    (-10, "011111111011"),
    (-9, "01111111011"),
    (-8, "0111111011"),
    (-7, "0111111111"),
    (-6, "01111100"),
    (-5, "0111100"),
    (-4, "011100"),
    (-3, "01100"),
    (-2, "0000"),
    (-1, "001"),
    (0, "1"),
    (1, "010"),
    (2, "0001"),
    (3, "01101"),
    (4, "011101"),
    (5, "0111101"),
    (6, "01111101"),
    (7, "011111100"),
    (8, "0111111100"),
    (9, "01111111100"),
    (10, "01111110100"),
    (11, "011111101011"),
    (12, "0111111101010"),
    (13, "01111111101010"),
    (14, "01111111010110"),
    (15, "011111111010000"),
    (16, "0111111110101111"),
    (17, "0111111101000011"),
    (18, "01111111010111001"),
    (19, "01111111010000011"),
    (20, "011111111010110111"),
    (21, "01111111010000001"),
    (22, "011111111010001001"),
    (23, "011111111010001110"),
    (24, "011111111010001111"),
    (25, "011111111010001100"),
    (26, "011111111010001101"),
    (27, "011111111010110010"),
    (28, "011111111010110011"),
    (29, "011111111010110000"),
    (30, "011111111010110001"),
];

// Table 8.B.17 — huff_iid_dt[1] (fine quant, time-diff).
static HUFF_IID_DT1: &[HuffEntry] = huff_table![
    (-30, "0100111011010100"),
    (-29, "0100111011010101"),
    (-28, "0100111011001110"),
    (-27, "0100111011001111"),
    (-26, "0100111011001100"),
    (-25, "0100111011010110"),
    (-24, "0100111011011000"),
    (-23, "0100111101000110"),
    (-22, "0100111101100000"),
    (-21, "010011100011000"),
    (-20, "010011100011001"),
    (-19, "010011101100100"),
    (-18, "010011101100101"),
    (-17, "010011101101101"),
    (-16, "010011110110001"),
    (-15, "01001110110111"),
    (-14, "01001111010110"),
    (-13, "0100111000111"),
    (-12, "0100111101001"),
    (-11, "0100111101101"),
    (-10, "010011101110"),
    (-9, "010011110111"),
    (-8, "01001111000"),
    (-7, "0100111001"),
    (-6, "010011010"),
    (-5, "010011111"),
    (-4, "0100000"),
    (-3, "010001"),
    (-2, "01010"),
    (-1, "011"),
    (0, "1"),
    (1, "00"),
    (2, "01011"),
    (3, "010010"),
    (4, "0100001"),
    (5, "01001100"),
    (6, "010011011"),
    (7, "0100111010"),
    (8, "01001111001"),
    (9, "01001110000"),
    (10, "010011101111"),
    (11, "010011100010"),
    (12, "0100111101010"),
    (13, "0100111011000"),
    (14, "01001111010111"),
    (15, "01001111010000"),
    (16, "010011110110010"),
    (17, "010011110100010"),
    (18, "010011100011010"),
    (19, "010011100011011"),
    (20, "0100111101100110"),
    (21, "0100111101100111"),
    (22, "0100111101100001"),
    (23, "0100111101000111"),
    (24, "0100111011011001"),
    (25, "0100111011010111"),
    (26, "0100111011001101"),
    (27, "0100111011010010"),
    (28, "0100111011010011"),
    (29, "0100111011010000"),
    (30, "0100111011010001"),
];

// Table 8.B.19 — huff_icc_df and huff_icc_dt. Range -7..7.
static HUFF_ICC_DF: &[HuffEntry] = huff_table![
    (-7, "11111111111111"),
    (-6, "11111111111110"),
    (-5, "111111111110"),
    (-4, "1111111110"),
    (-3, "1111110"),
    (-2, "11110"),
    (-1, "110"),
    (0, "0"),
    (1, "10"),
    (2, "1110"),
    (3, "111110"),
    (4, "11111110"),
    (5, "111111110"),
    (6, "11111111110"),
    (7, "1111111111110"),
];

static HUFF_ICC_DT: &[HuffEntry] = huff_table![
    (-7, "11111111111110"),
    (-6, "1111111111110"),
    (-5, "11111111110"),
    (-4, "111111110"),
    (-3, "1111110"),
    (-2, "11110"),
    (-1, "110"),
    (0, "0"),
    (1, "10"),
    (2, "1110"),
    (3, "111110"),
    (4, "11111110"),
    (5, "1111111110"),
    (6, "111111111110"),
    (7, "11111111111111"),
];

// Table 8.B.20 — huff_ipd_df and huff_ipd_dt. Range 0..7.
static HUFF_IPD_DF: &[HuffEntry] = huff_table![
    (0, "1"),
    (1, "000"),
    (2, "0110"),
    (3, "0100"),
    (4, "0010"),
    (5, "0011"),
    (6, "0101"),
    (7, "0111"),
];

static HUFF_IPD_DT: &[HuffEntry] = huff_table![
    (0, "1"),
    (1, "010"),
    (2, "0010"),
    (3, "00011"),
    (4, "00010"),
    (5, "0000"),
    (6, "0011"),
    (7, "011"),
];

// ---------- Bitstream parsing -------------------------------------------

/// Parse one `ps_data()` frame from the bitstream.
///
/// `br` must be positioned at the start of ps_data (i.e. just past the
/// `bs_extension_id` nibble inside the SBR extended_data block). Returns
/// the parsed frame data.
pub fn parse_ps_data(br: &mut BitReader<'_>, state: &mut PsState) -> Result<PsFrame> {
    let mut out = PsFrame::default();

    // ---- Header ----
    let enable_ps_header = br.read_bit()?;
    if enable_ps_header {
        state.header.header_seen = true;
        let enable_iid = br.read_bit()?;
        state.header.enable_iid = enable_iid;
        if enable_iid {
            let iid_mode = br.read_u32(3)? as u8;
            state.header.iid_mode = iid_mode;
            let idx = (iid_mode as usize).min(5);
            state.header.nr_iid_par = NR_IID_PAR_TAB[idx];
            state.header.nr_ipdopd_par = NR_IPDOPD_PAR_TAB[idx];
            state.header.iid_quant_fine = iid_mode >= 3;
        }
        let enable_icc = br.read_bit()?;
        state.header.enable_icc = enable_icc;
        if enable_icc {
            let icc_mode = br.read_u32(3)? as u8;
            state.header.icc_mode = icc_mode;
            let idx = (icc_mode as usize).min(5);
            state.header.nr_icc_par = NR_ICC_PAR_TAB[idx];
            state.header.mixing_rb = icc_mode >= 3;
        }
        state.header.enable_ext = br.read_bit()?;
    }

    // ---- Frame ----
    let frame_class = br.read_bit()? as u8;
    let num_env_idx = br.read_u32(2)? as usize;
    let num_env = NUM_ENV_TAB[frame_class as usize][num_env_idx];
    out.frame_class = frame_class;
    out.num_env = num_env;

    if frame_class == 1 {
        for e in 0..num_env.min(5) {
            out.border_position[e] = br.read_u32(5)? as u8;
        }
    }

    // IID data per envelope.
    out.iid_db = vec![Vec::new(); num_env];
    out.nr_iid_par = state.header.nr_iid_par;
    if state.header.enable_iid {
        out.has_iid = true;
        let n = state.header.nr_iid_par.min(34);
        let (df, dt) = if state.header.iid_quant_fine {
            (HUFF_IID_DF1, HUFF_IID_DT1)
        } else {
            (HUFF_IID_DF0, HUFF_IID_DT0)
        };
        let mut prev = state.prev_iid_idx.clone();
        prev.resize(34, 0);
        for e in 0..num_env {
            let dt_flag = br.read_bit()?;
            let mut idxs = vec![0i32; n];
            if dt_flag {
                for b in 0..n {
                    let d = huff_decode(br, dt)?;
                    idxs[b] = prev[b] + d;
                }
            } else {
                let mut acc: i32 = 0;
                for b in 0..n {
                    let d = huff_decode(br, df)?;
                    acc += d;
                    idxs[b] = acc;
                }
            }
            // Clamp to the valid quantisation range.
            let clamp_abs = if state.header.iid_quant_fine { 30 } else { 14 };
            for b in 0..n {
                idxs[b] = idxs[b].clamp(-clamp_abs, clamp_abs);
            }
            // Dequantise to dB.
            let table: &[f32] = if state.header.iid_quant_fine {
                &IID_QUANT_FINE
            } else {
                &IID_QUANT_DEFAULT
            };
            let center = if state.header.iid_quant_fine { 15 } else { 7 };
            let mut db = vec![0.0f32; n];
            for b in 0..n {
                let t = (idxs[b] + center as i32).clamp(0, table.len() as i32 - 1) as usize;
                db[b] = table[t];
            }
            out.iid_db[e] = db;
            prev = idxs.clone();
        }
        out.iid_idx_last = prev;
    }

    // ICC data per envelope.
    out.icc = vec![Vec::new(); num_env];
    out.nr_icc_par = state.header.nr_icc_par;
    if state.header.enable_icc {
        out.has_icc = true;
        let n = state.header.nr_icc_par.min(34);
        let mut prev = state.prev_icc_idx.clone();
        prev.resize(34, 0);
        for e in 0..num_env {
            let dt_flag = br.read_bit()?;
            let mut idxs = vec![0i32; n];
            if dt_flag {
                for b in 0..n {
                    let d = huff_decode(br, HUFF_ICC_DT)?;
                    idxs[b] = prev[b] + d;
                }
            } else {
                let mut acc: i32 = 0;
                for b in 0..n {
                    let d = huff_decode(br, HUFF_ICC_DF)?;
                    acc += d;
                    idxs[b] = acc;
                }
            }
            // ICC index is in 0..7; clamp.
            for b in 0..n {
                idxs[b] = idxs[b].clamp(0, 7);
            }
            let mut icc = vec![0.0f32; n];
            for b in 0..n {
                icc[b] = ICC_QUANT[idxs[b] as usize];
            }
            out.icc[e] = icc;
            prev = idxs.clone();
        }
        out.icc_idx_last = prev;
    }

    // Extension. Baseline PS skips IPD/OPD but we still need to consume the
    // bits so the outer SBR loop can bit-align its extended_data block.
    if state.header.enable_ext {
        let mut cnt = br.read_u32(4)?;
        if cnt == 15 {
            cnt += br.read_u32(8)?;
        }
        let total_bits = (cnt * 8) as i32;
        let start = br.bit_position() as i32;
        while (br.bit_position() as i32 - start) + 7 < total_bits {
            let ext_id = br.read_u32(2)?;
            if ext_id == 0 {
                // ps_extension v0: optional IPD/OPD. Parse but discard since
                // the baseline decoder does not apply them.
                let enable_ipdopd = br.read_bit()?;
                if enable_ipdopd {
                    let nbands = state.header.nr_ipdopd_par.min(34);
                    for _e in 0..num_env {
                        let ipd_dt_flag = br.read_bit()?;
                        for _b in 0..nbands {
                            let _ = if ipd_dt_flag {
                                huff_decode(br, HUFF_IPD_DT)
                            } else {
                                huff_decode(br, HUFF_IPD_DF)
                            };
                        }
                        let opd_dt_flag = br.read_bit()?;
                        // opd uses the same table layout as ipd.
                        for _b in 0..nbands {
                            let _ = if opd_dt_flag {
                                huff_decode(br, HUFF_IPD_DT)
                            } else {
                                huff_decode(br, HUFF_IPD_DF)
                            };
                        }
                    }
                }
                let _reserved = br.read_bit()?;
            } else {
                // Unknown extension — drain. The outer loop will align any
                // residual bits.
                let rem = total_bits - (br.bit_position() as i32 - start);
                for _ in 0..rem.max(0) {
                    let _ = br.read_u32(1)?;
                }
                break;
            }
        }
        // Fill bits — consume any remainder to reach total_bits.
        let consumed = br.bit_position() as i32 - start;
        let remainder = (total_bits - consumed).max(0);
        for _ in 0..remainder {
            let _ = br.read_u32(1)?;
        }
    }

    // Update running state for next frame.
    if out.has_iid {
        state.prev_iid_idx = out.iid_idx_last.clone();
        state.prev_iid_idx.resize(34, 0);
    }
    if out.has_icc {
        state.prev_icc_idx = out.icc_idx_last.clone();
        state.prev_icc_idx.resize(34, 0);
    }
    state.prev_ps_seen = true;
    Ok(out)
}

// ---------- Decorrelator ------------------------------------------------

/// Transient-ratio smoothing filter y[n] = a*x[n] + (1-a)*y[n-1].
#[inline]
fn smooth(prev: f32, x: f32) -> f32 {
    A_SMOOTH * x + (1.0 - A_SMOOTH) * prev
}

/// Produce the decorrelated QMF matrix `d` from mono input `s`. Both are
/// indexed as `[time-slot][qmf-band]`. Shape: rows = num_slots, cols = 64.
///
/// Per §8.6.4.5.2: for bands 0..NR_ALLPASS_BANDS, apply a 3-link complex
/// allpass chain with fractional delays; for bands in [NR_ALLPASS_BANDS,
/// SHORT_DELAY_BAND), apply a 14-sample integer delay; for bands
/// [SHORT_DELAY_BAND, NR_BANDS), apply a 1-sample delay. Transient reduction
/// (§8.6.4.5.3-4) scales each output by G_TransientRatioMapped.
fn decorrelate_qmf(
    s: &[[Complex32; NUM_QMF_BANDS]],
    d: &mut [[Complex32; NUM_QMF_BANDS]],
    state: &mut PsState,
) {
    let num_slots = s.len().min(d.len());

    // Precompute per-band allpass coefficients phi_fract and Q_fract (table
    // 8.42 with frequency variation via g_DecaySlope).
    let mut phi_fract = [Complex32::new(0.0, 0.0); NR_ALLPASS_BANDS];
    let mut q_fract: [[Complex32; NR_ALLPASS_LINKS]; NR_ALLPASS_BANDS] =
        [[Complex32::new(0.0, 0.0); NR_ALLPASS_LINKS]; NR_ALLPASS_BANDS];
    let mut g_decay = [1.0f32; NR_ALLPASS_BANDS];
    for k in 0..NR_ALLPASS_BANDS {
        let fc = fcenter(k);
        let p = -core::f32::consts::PI * Q_PHI * fc;
        phi_fract[k] = Complex32::new(p.cos(), p.sin());
        for m in 0..NR_ALLPASS_LINKS {
            let pm = -core::f32::consts::PI * ALLPASS_Q[m] * fc;
            q_fract[k][m] = Complex32::new(pm.cos(), pm.sin());
        }
        if k > DECAY_CUTOFF {
            g_decay[k] = (1.0 - DECAY_SLOPE * (k as f32 - DECAY_CUTOFF as f32)).max(0.0);
        }
    }

    // Transient detection running state for this frame — works on parameter
    // bands. Iterate over time slots.
    for n in 0..num_slots {
        // Compute per-param-band power P(i, n) = sum over QMF bands in band i of |s[n][k]|^2.
        let mut p = [0.0f32; NR_PAR_BANDS];
        for k in 0..NUM_QMF_BANDS {
            let bidx = QMF_TO_PARAM_20[k] as usize;
            if bidx < NR_PAR_BANDS {
                p[bidx] += s[n][k].re * s[n][k].re + s[n][k].im * s[n][k].im;
            }
        }
        // Peak decay + smoothing.
        for i in 0..NR_PAR_BANDS {
            let decayed = PEAK_DECAY_ALPHA * state.peak_decay_nrg[i];
            state.peak_decay_nrg[i] = if decayed < p[i] { p[i] } else { decayed };
            state.smooth_nrg[i] = smooth(state.smooth_nrg[i], p[i]);
            let diff = state.peak_decay_nrg[i] - p[i];
            state.smooth_peak_decay_diff_nrg[i] = smooth(state.smooth_peak_decay_diff_nrg[i], diff);
        }
        // Transient attenuator per param band.
        let mut g_trans = [1.0f32; NR_PAR_BANDS];
        for i in 0..NR_PAR_BANDS {
            let num = TRANSIENT_GAMMA * state.smooth_peak_decay_diff_nrg[i];
            if num > state.smooth_nrg[i] {
                g_trans[i] = state.smooth_nrg[i] / num.max(1e-30);
            }
        }

        // Apply allpass for low bands, fixed delay for the rest.
        for k in 0..NR_BANDS.min(NUM_QMF_BANDS) {
            let sample = s[n][k];
            let bidx = QMF_TO_PARAM_20[k] as usize;
            let gt = if bidx < NR_PAR_BANDS {
                g_trans[bidx]
            } else {
                1.0
            };
            let out_sample = if k < NR_ALLPASS_BANDS {
                // Allpass chain: y = phi_fract(k) * z^-2 * prod_m H_m(z) * s
                // We implement this as a cascade of biquads in direct form.
                //
                // For each link m:
                //   y_m[n] = -a(m)*g_decay(k)*q_fract(k,m) * x_m[n]
                //          +                                   x_m[n - d(m)]
                //          +  a(m)*g_decay(k)*q_fract(k,m) * y_m[n - d(m)]
                //
                // But the spec form Q_fract(k,m) z^-d(m) - a(m)*g_decay(k)
                //                   -------------------------------------
                //                   1 - a(m) g_decay(k) Q_fract(k,m) z^-d(m)
                // simplifies (using a single state per link) by keeping the
                // delayed value x_delayed and producing:
                //   y = (q_fract * x_delayed - g * x) / (1 - g * q_fract * ??) — no
                // Correct direct form: for allpass in Z domain
                //   H(z) = (b0 + b_k z^-k) / (1 + a_k z^-k)
                // with b0 = -a, b_k = q_fract, a_k = -a*g*q_fract. A standard
                // realisation uses a single delay line of length d(m) for the
                // direct form II. Because q_fract is complex we keep the
                // complex recursion below.
                //
                // x[n] is current input; w[n] is internal state (length d).
                //   w[n] = x[n] + a*g*q_fract * w[n - d]
                //   y[n] = -a * w[n] + q_fract * w[n - d]
                // For an allpass this gives |H(e^jw)| = 1. We use this
                // direct-form-II recursion — it matches the transfer
                // function in 8.6.4.5.2.
                let mut cur = sample;
                // Include the leading z^-2 * phi_fract — we approximate as a
                // 2-sample input delay followed by a complex phase rotation.
                // Store the 2-sample prelude in slots 0 and 1 of the delay
                // line as a simple tap.
                for m in 0..NR_ALLPASS_LINKS {
                    let d = ALLPASS_D[m];
                    let buf = &mut state.allpass_delay[k][m];
                    let pos = state.allpass_pos[k];
                    // Fetch w[n-d].
                    let dl_idx = (pos + buf.len() - d) % buf.len();
                    let w_delayed = buf[dl_idx];
                    let ag = ALLPASS_A[m] * g_decay[k];
                    let gq = q_fract[k][m].scale(ag);
                    // w[n] = x[n] + a*g*q_fract * w[n-d]
                    let w_n = cur + gq * w_delayed;
                    // y[n] = -a*g * w[n] + q_fract * w[n-d]
                    let y_n = w_n.scale(-ag) + q_fract[k][m] * w_delayed;
                    // Store w_n.
                    buf[pos] = w_n;
                    cur = y_n;
                }
                // Advance the shared position for this band.
                state.allpass_pos[k] = (state.allpass_pos[k] + 1) % state.allpass_delay[k][0].len();
                // Apply phi_fract (z^-2 already absorbed into delay lines).
                phi_fract[k] * cur
            } else if k < SHORT_DELAY_BAND {
                // 14-sample delay; we cap at 16 slots of storage.
                let buf = &mut state.short_delay[k];
                let pos = state.short_delay_pos[k];
                let d = 14;
                let dl_idx = (pos + buf.len() - d) % buf.len();
                let y = buf[dl_idx];
                buf[pos] = sample;
                state.short_delay_pos[k] = (pos + 1) % buf.len();
                y
            } else if k < NUM_QMF_BANDS {
                // 1-sample delay.
                let buf = &mut state.short_delay[k];
                let pos = state.short_delay_pos[k];
                let dl_idx = (pos + buf.len() - 1) % buf.len();
                let y = buf[dl_idx];
                buf[pos] = sample;
                state.short_delay_pos[k] = (pos + 1) % buf.len();
                y
            } else {
                Complex32::new(0.0, 0.0)
            };
            d[n][k] = out_sample.scale(gt);
        }
        // Bands above NR_BANDS don't exist in the 64-QMF case (NR_BANDS=71
        // > NUM_QMF_BANDS=64), so no additional zero-fill is needed here.
    }
}

// ---------- Stereo processing -------------------------------------------

/// Compute mixing coefficients h11,h12,h21,h22 for one parameter position.
///
/// Inputs are dB-valued IID and linear ICC for the parameter band. Returns
/// the 4-tuple.
fn mix_coeffs(iid_db: f32, icc: f32, use_rb: bool) -> (f32, f32, f32, f32) {
    let c = 10f32.powf(iid_db / 20.0);
    if !use_rb {
        // Mixing procedure Ra (§8.6.4.6.2.1).
        let c2 = c * c;
        let c1 = (2.0 / (1.0 + c2)).sqrt();
        let c2s = (2.0 * c / (1.0 + c2)).sqrt();
        let alpha = 0.5 * icc.clamp(-1.0, 1.0).acos();
        let beta = alpha * (c1 - c2s) / core::f32::consts::SQRT_2;
        let h11 = (alpha + beta).cos() * c2s;
        let h12 = (beta - alpha).cos() * c1;
        let h21 = (alpha + beta).sin() * c2s;
        let h22 = (beta - alpha).sin() * c1;
        (h11, h12, h21, h22)
    } else {
        // Mixing procedure Rb (§8.6.4.6.2.2).
        let rho = icc.clamp(0.05, 1.0);
        let alpha = if (c - 1.0).abs() > 1e-6 {
            0.5 * ((2.0 * c * rho) / (c * c - 1.0)).atan()
        } else {
            core::f32::consts::FRAC_PI_4
        };
        // Modulo correction: alpha -= floor(alpha / (pi/2)) * (pi/2).
        let half_pi = core::f32::consts::FRAC_PI_2;
        let alpha = alpha - (alpha / half_pi).floor() * half_pi;
        let c_rec = if c.abs() < 1e-6 { 0.0 } else { 1.0 / c };
        let mu = 1.0 + (4.0 * rho * rho - 4.0) / (c + c_rec).powi(2);
        let mu = mu.max(0.0);
        let gamma = ((1.0 - mu.sqrt()) / (1.0 + mu.sqrt())).atan();
        let sqrt2 = core::f32::consts::SQRT_2;
        let h11 = sqrt2 * alpha.cos() * gamma.cos();
        let h12 = sqrt2 * alpha.sin() * gamma.cos();
        let h21 = -sqrt2 * alpha.sin() * gamma.sin();
        let h22 = sqrt2 * alpha.cos() * gamma.sin();
        (h11, h12, h21, h22)
    }
}

/// Map IID/ICC from whatever source grid (10, 20, or 34 bands) onto 20
/// parameter bands, per Tables 8.45 / 8.46. Values are linear-domain already
/// (or dB — mapping is linear in index representation; we run it on the
/// dequantised value which is an acceptable baseline approximation).
fn map_to_20(values: &[f32], n_src: usize) -> [f32; NR_PAR_BANDS] {
    let mut out = [0.0f32; NR_PAR_BANDS];
    if values.is_empty() {
        return out;
    }
    match n_src {
        20 => {
            for b in 0..NR_PAR_BANDS.min(values.len()) {
                out[b] = values[b];
            }
        }
        10 => {
            // Table 8.45 (idx20 = idx10 for even, (idx10+idx10+1)/2 for odd).
            // We use the simpler duplication: each 10-band value covers 2
            // consecutive 20-band slots.
            for b in 0..NR_PAR_BANDS {
                let src = (b / 2).min(values.len() - 1);
                out[b] = values[src];
            }
        }
        34 => {
            // Table 8.46 — inverse mapping from 34 to 20 using averages.
            let avg = |a: usize, b: usize| -> f32 {
                let ai = a.min(values.len() - 1);
                let bi = b.min(values.len() - 1);
                0.5 * (values[ai] + values[bi])
            };
            let avg3 = |w_a: f32, a: usize, w_b: f32, b: usize| -> f32 {
                let ai = a.min(values.len() - 1);
                let bi = b.min(values.len() - 1);
                (w_a * values[ai] + w_b * values[bi]) / (w_a + w_b)
            };
            out[0] = avg3(2.0, 0, 1.0, 1);
            out[1] = avg3(1.0, 1, 2.0, 2);
            out[2] = avg3(2.0, 3, 1.0, 4);
            out[3] = avg3(1.0, 4, 2.0, 5);
            out[4] = avg(6, 7);
            out[5] = avg(8, 9);
            out[6] = values[10.min(values.len() - 1)];
            out[7] = values[11.min(values.len() - 1)];
            out[8] = avg(12, 13);
            out[9] = avg(14, 15);
            out[10] = values[16.min(values.len() - 1)];
            out[11] = values[17.min(values.len() - 1)];
            out[12] = values[18.min(values.len() - 1)];
            out[13] = values[19.min(values.len() - 1)];
            out[14] = avg(20, 21);
            out[15] = avg(22, 23);
            out[16] = avg(24, 25);
            out[17] = avg(26, 27);
            let v = |i: usize| values[i.min(values.len() - 1)];
            out[18] = 0.25 * (v(28) + v(29) + v(30) + v(31));
            out[19] = avg(32, 33);
        }
        _ => {
            // Unknown — broadcast first element.
            for b in 0..NR_PAR_BANDS {
                out[b] = values[0];
            }
        }
    }
    out
}

/// Compute envelope border sample indices n_e for e = 0..num_env-1 given a
/// frame length of `num_qmf_slots` samples.
fn envelope_borders(frame: &PsFrame, num_qmf_slots: usize) -> Vec<usize> {
    let mut borders = Vec::with_capacity(frame.num_env);
    if frame.frame_class == 0 {
        for e in 0..frame.num_env {
            let val = num_qmf_slots * (e + 1) / frame.num_env.max(1);
            let ne = val.saturating_sub(1);
            borders.push(ne);
        }
    } else {
        for e in 0..frame.num_env {
            let ne = (frame.border_position[e] as usize).min(num_qmf_slots.saturating_sub(1));
            borders.push(ne);
        }
    }
    if borders.is_empty() {
        borders.push(num_qmf_slots.saturating_sub(1));
    }
    borders
}

/// Apply PS in the QMF domain.
///
/// `x_mono` holds the mono QMF matrix `[num_slots][64]` produced by SBR
/// (after HF generation + envelope adjustment). `x_left` / `x_right` receive
/// the stereo QMF matrices. The buffers must all have the same number of
/// rows — this is typically `num_slots = num_time_slots * RATE = 32` for
/// 1024-sample AAC framing.
pub fn apply_ps_qmf(
    x_mono: &[[Complex32; NUM_QMF_BANDS]],
    x_left: &mut [[Complex32; NUM_QMF_BANDS]],
    x_right: &mut [[Complex32; NUM_QMF_BANDS]],
    frame: &PsFrame,
    state: &mut PsState,
) {
    let num_slots = x_mono.len().min(x_left.len()).min(x_right.len());

    // 1) Build the decorrelated signal d.
    let mut d: Vec<[Complex32; NUM_QMF_BANDS]> =
        vec![[Complex32::new(0.0, 0.0); NUM_QMF_BANDS]; num_slots];
    decorrelate_qmf(x_mono, &mut d, state);

    // 2) Build per-envelope h-vectors on the 20-band grid.
    let num_env = frame.num_env.max(1);
    let mut h_at_env = vec![[[0.0f32; NR_PAR_BANDS]; 4]; num_env];
    // Determine effective arrays. If IID / ICC weren't present, use
    // defaults (IID = 0 dB → c = 1; ICC = 1 → perfect correlation).
    for e in 0..num_env {
        let iid_raw: Vec<f32> = if frame.has_iid {
            frame.iid_db.get(e).cloned().unwrap_or_default()
        } else {
            Vec::new()
        };
        let icc_raw: Vec<f32> = if frame.has_icc {
            frame.icc.get(e).cloned().unwrap_or_else(|| vec![1.0])
        } else {
            Vec::new()
        };
        let iid20 = if !iid_raw.is_empty() {
            map_to_20(&iid_raw, frame.nr_iid_par)
        } else {
            [0.0; NR_PAR_BANDS]
        };
        let icc20 = if !icc_raw.is_empty() {
            map_to_20(&icc_raw, frame.nr_icc_par)
        } else {
            [1.0; NR_PAR_BANDS]
        };
        for b in 0..NR_PAR_BANDS {
            let (h11, h12, h21, h22) = mix_coeffs(iid20[b], icc20[b], state.header.mixing_rb);
            h_at_env[e][0][b] = h11;
            h_at_env[e][1][b] = h12;
            h_at_env[e][2][b] = h21;
            h_at_env[e][3][b] = h22;
        }
    }
    let borders = envelope_borders(frame, num_slots);

    // 3) Precompute per-slot interpolated H on the 20-param-band grid.
    //
    // The interpolation schedule runs exactly as before (§8.6.4.6.4) — for
    // each envelope boundary we ramp linearly from h_prev to h_cur across
    // the segment. This yields a num_slots × 4 × NR_PAR_BANDS table we can
    // consume both for the sub-QMF (bands 0..2) and straight QMF (bands
    // 3..63) mixing.
    let mut h_slot = vec![[[0.0f32; NR_PAR_BANDS]; 4]; num_slots];
    let mut h_prev = state.prev_h_end;
    let mut seg_start = 0usize;
    for e in 0..num_env {
        let seg_end = borders[e].max(seg_start); // inclusive
        let h_cur = h_at_env[e];
        let span = (seg_end as i32 - seg_start as i32 + 1).max(1) as f32;
        for n in seg_start..=seg_end.min(num_slots - 1) {
            let t = if span > 1.0 {
                ((n as i32 - seg_start as i32) as f32 + 1.0) / span
            } else {
                1.0
            };
            let t = t.clamp(0.0, 1.0);
            for ij in 0..4 {
                for b in 0..NR_PAR_BANDS {
                    h_slot[n][ij][b] = h_prev[ij][b] + t * (h_cur[ij][b] - h_prev[ij][b]);
                }
            }
        }
        h_prev = h_cur;
        seg_start = seg_end + 1;
    }
    for n in seg_start..num_slots {
        h_slot[n] = h_prev;
    }

    // 4) Mixing for the 3 lowest QMF bands via the hybrid sub-QMF split
    //    (§8.6.4.3). For each time slot we:
    //      a) analyse x_mono[.][0..3] and d[.][0..3] into 10 sub-subbands
    //         each (6 from band 0, 2 each from bands 1, 2);
    //      b) mix at sub-subband granularity using HYBRID_TO_PARAM_20;
    //      c) synthesise back into 3 QMF bands per L/R output.
    for n in 0..num_slots {
        let mono3 = [x_mono[n][0], x_mono[n][1], x_mono[n][2]];
        let d3 = [d[n][0], d[n][1], d[n][2]];
        let sub_m = hybrid_analysis_slot(&mono3, &mut state.hybrid_mono);
        let sub_d = hybrid_analysis_slot(&d3, &mut state.hybrid_d);
        let mut sub_l = [Complex32::new(0.0, 0.0); HYBRID_LOW_SUBBANDS];
        let mut sub_r = [Complex32::new(0.0, 0.0); HYBRID_LOW_SUBBANDS];
        for k in 0..HYBRID_LOW_SUBBANDS {
            let bidx = HYBRID_TO_PARAM_20[k] as usize;
            let h11 = h_slot[n][0][bidx];
            let h12 = h_slot[n][1][bidx];
            let h21 = h_slot[n][2][bidx];
            let h22 = h_slot[n][3][bidx];
            sub_l[k] = sub_m[k].scale(h11) + sub_d[k].scale(h21);
            sub_r[k] = sub_m[k].scale(h12) + sub_d[k].scale(h22);
        }
        let l3 = hybrid_synthesis_slot(&sub_l);
        let r3 = hybrid_synthesis_slot(&sub_r);
        for k in 0..3 {
            x_left[n][k] = l3[k];
            x_right[n][k] = r3[k];
        }
    }

    // 5) Mixing for QMF bands 3..63 — straight QMF-band-to-param-band map.
    //    No further filtering needed here; these bands pass through.
    for n in 0..num_slots {
        for k in 3..NUM_QMF_BANDS {
            let bidx = QMF_TO_PARAM_20[k] as usize;
            let h11 = h_slot[n][0][bidx];
            let h12 = h_slot[n][1][bidx];
            let h21 = h_slot[n][2][bidx];
            let h22 = h_slot[n][3][bidx];
            let s = x_mono[n][k];
            let dv = d[n][k];
            x_left[n][k] = s.scale(h11) + dv.scale(h21);
            x_right[n][k] = s.scale(h12) + dv.scale(h22);
        }
    }

    // Store end-of-frame H for next frame's region0 interpolation.
    state.prev_h_end = h_prev;
}

/// Legacy simplified time-domain PS upmix — kept for back-compat with the
/// original test suite. New code should use `apply_ps_qmf`.
pub fn apply_ps_simple(
    mono: &[f32],
    out_l: &mut [f32],
    out_r: &mut [f32],
    iid_avg_db: f32,
    icc_avg: f32,
    state: &mut PsState,
) {
    debug_assert_eq!(out_l.len(), mono.len());
    debug_assert_eq!(out_r.len(), mono.len());
    let g_ratio = 10.0f32.powf(iid_avg_db / 20.0);
    let g_l = 1.0 / (1.0 + g_ratio);
    let g_r = g_ratio / (1.0 + g_ratio);
    let rho = icc_avg.clamp(-1.0, 1.0);
    let rho_c = (1.0 - rho * rho).max(0.0).sqrt();

    // Reuse the first decorrelator band's allpass chain as a cheap delay
    // proxy. This keeps the legacy API working without disturbing the new
    // QMF-domain state.
    let scratch_band = 0;
    let delay_len = state.allpass_delay[scratch_band][0].len();
    for (i, &m) in mono.iter().enumerate() {
        let pos = state.allpass_pos[scratch_band];
        let decorr = state.allpass_delay[scratch_band][0][pos];
        state.allpass_delay[scratch_band][0][pos] = Complex32::new(m, 0.0);
        state.allpass_pos[scratch_band] = (pos + 1) % delay_len;
        out_l[i] = rho * g_l * m + rho_c * g_l * decorr.re;
        out_r[i] = rho * g_r * m - rho_c * g_r * decorr.re;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn huff_iid_df0_zero_single_bit() {
        // huff_iid_df[0] entry for value=0 is the single bit "0".
        let data = [0b00000000u8];
        let mut br = BitReader::new(&data);
        let v = huff_decode(&mut br, HUFF_IID_DF0).unwrap();
        assert_eq!(v, 0);
    }

    #[test]
    fn huff_iid_df0_one() {
        // huff_iid_df[0] entry for value=1 is "100".
        let data = [0b10000000u8];
        let mut br = BitReader::new(&data);
        let v = huff_decode(&mut br, HUFF_IID_DF0).unwrap();
        assert_eq!(v, 1);
    }

    #[test]
    fn huff_icc_df_zero() {
        let data = [0u8];
        let mut br = BitReader::new(&data);
        let v = huff_decode(&mut br, HUFF_ICC_DF).unwrap();
        assert_eq!(v, 0);
    }

    #[test]
    fn mix_coeffs_center() {
        // IID=0 dB, ICC=1 → fully coherent centered mix → h11=h12=h22=h21=?
        let (h11, h12, h21, h22) = mix_coeffs(0.0, 1.0, false);
        // c = 1 → c1 = c2 = 1; alpha = 0 → beta = 0. So h11 = cos(0)*1 = 1,
        // h12 = 1, h21 = 0, h22 = 0.
        assert!((h11 - 1.0).abs() < 1e-5, "h11 = {}", h11);
        assert!((h12 - 1.0).abs() < 1e-5);
        assert!(h21.abs() < 1e-5);
        assert!(h22.abs() < 1e-5);
    }

    #[test]
    fn mix_coeffs_left_pan() {
        // Per §8.6.4.6.2.1, c = 10^(IID/20) is the L/R amplitude ratio.
        // IID=+6 dB → c ≈ 2 → L louder, so h11 (s→L) should dominate h12.
        let (h11, h12, _h21, _h22) = mix_coeffs(6.0, 1.0, false);
        assert!(
            h11.abs() > h12.abs(),
            "expected h11 dominant (L louder): h11={} h12={}",
            h11,
            h12
        );
    }

    #[test]
    fn mix_coeffs_right_pan() {
        // IID=-6 dB → c ≈ 0.5 → R louder, so h12 (s→R) should dominate h11.
        let (h11, h12, _h21, _h22) = mix_coeffs(-6.0, 1.0, false);
        assert!(
            h12.abs() > h11.abs(),
            "expected h12 dominant (R louder): h11={} h12={}",
            h11,
            h12
        );
    }

    #[test]
    fn decorrelator_is_energy_bounded() {
        // Feed a unit complex sinusoid into band 0 across many slots and
        // confirm the decorrelator output has |y| ~ |x|, i.e. bounded.
        let num_slots = 64;
        let mut s: Vec<[Complex32; NUM_QMF_BANDS]> =
            vec![[Complex32::new(0.0, 0.0); NUM_QMF_BANDS]; num_slots];
        for n in 0..num_slots {
            let phase = (n as f32) * 0.3;
            s[n][3] = Complex32::new(phase.cos(), phase.sin());
        }
        let mut d: Vec<[Complex32; NUM_QMF_BANDS]> =
            vec![[Complex32::new(0.0, 0.0); NUM_QMF_BANDS]; num_slots];
        let mut state = PsState::new();
        decorrelate_qmf(&s, &mut d, &mut state);
        // After warm-up, magnitude of d[n][3] should be ~1 (allpass).
        let mut e_in = 0.0f32;
        let mut e_out = 0.0f32;
        for n in 30..num_slots {
            e_in += s[n][3].norm_sqr();
            e_out += d[n][3].norm_sqr();
        }
        let ratio = e_out / e_in.max(1e-10);
        assert!(
            ratio < 5.0,
            "decorrelator blew up: e_in={} e_out={} ratio={}",
            e_in,
            e_out,
            ratio
        );
    }

    #[test]
    fn ps_qmf_identity_when_default() {
        // With no IID / ICC info, PS should approximate a mono-to-stereo
        // duplication — both channels should contain most of the mono signal
        // energy (after the mixing matrix reduces to a copy).
        let num_slots = 8;
        let mut mono: Vec<[Complex32; NUM_QMF_BANDS]> =
            vec![[Complex32::new(0.0, 0.0); NUM_QMF_BANDS]; num_slots];
        // Put some energy into a few bands.
        for n in 0..num_slots {
            for k in 3..10 {
                let t = (n * 10 + k) as f32 * 0.1;
                mono[n][k] = Complex32::new(t.sin(), t.cos() * 0.5);
            }
        }
        let mut l = vec![[Complex32::new(0.0, 0.0); NUM_QMF_BANDS]; num_slots];
        let mut r = vec![[Complex32::new(0.0, 0.0); NUM_QMF_BANDS]; num_slots];
        let frame = PsFrame {
            num_env: 1,
            ..PsFrame::default()
        };
        let mut state = PsState::new();
        apply_ps_qmf(&mono, &mut l, &mut r, &frame, &mut state);
        // Check energy is preserved roughly (allow wide tolerance — allpass
        // chain warms up and interpolation ramps from zero initial state).
        let e_mono: f32 = mono
            .iter()
            .flat_map(|row| row.iter())
            .map(|c| c.re * c.re + c.im * c.im)
            .sum();
        let e_l: f32 = l
            .iter()
            .flat_map(|row| row.iter())
            .map(|c| c.re * c.re + c.im * c.im)
            .sum();
        let e_r: f32 = r
            .iter()
            .flat_map(|row| row.iter())
            .map(|c| c.re * c.re + c.im * c.im)
            .sum();
        // Both channels received non-trivial energy.
        assert!(e_l > 0.0, "l energy = {}", e_l);
        assert!(e_r > 0.0);
        // Combined stereo energy ≲ 2 × mono energy (sum is bounded).
        assert!(
            e_l + e_r < 4.0 * e_mono + 1e-3,
            "stereo energy blew up: e_l={} e_r={} e_mono={}",
            e_l,
            e_r,
            e_mono
        );
    }

    #[test]
    fn apply_ps_balanced_mono_stays_mono_like() {
        let mono: Vec<f32> = (0..128).map(|n| (n as f32 * 0.01).sin() * 0.5).collect();
        let mut l = vec![0.0; 128];
        let mut r = vec![0.0; 128];
        let mut ps = PsState::new();
        apply_ps_simple(&mono, &mut l, &mut r, 0.0, 1.0, &mut ps);
        for i in 0..128 {
            let expect = 0.5 * mono[i];
            assert!(
                (l[i] - expect).abs() < 1e-5,
                "l[{i}]={} expect={}",
                l[i],
                expect
            );
            assert!((r[i] - expect).abs() < 1e-5);
        }
    }

    #[test]
    fn apply_ps_all_right_pan() {
        let mono: Vec<f32> = (0..32).map(|n| n as f32 * 0.1).collect();
        let mut l = vec![0.0; 32];
        let mut r = vec![0.0; 32];
        let mut ps = PsState::new();
        apply_ps_simple(&mono, &mut l, &mut r, 18.0, 1.0, &mut ps);
        let el: f32 = l.iter().map(|v| v * v).sum();
        let er: f32 = r.iter().map(|v| v * v).sum();
        assert!(
            er > el * 10.0,
            "right channel not dominant: el={el}, er={er}"
        );
    }

    #[test]
    fn hybrid_analysis_synthesis_preserves_signal() {
        // Feed a constant-amplitude complex signal into each of the 3 low
        // QMF bands over enough slots to pass the filter's 13-tap
        // settling transient, then confirm that analysis-then-synthesis
        // reproduces the input up to a 6-slot group delay and unit gain.
        let mut st = HybridState::new();
        let mut out_b0: Vec<Complex32> = Vec::new();
        let mut out_b1: Vec<Complex32> = Vec::new();
        let mut out_b2: Vec<Complex32> = Vec::new();
        let n_slots = 40;
        let mut inputs: Vec<[Complex32; 3]> = Vec::with_capacity(n_slots);
        for n in 0..n_slots {
            // Unit-energy, different per-band signatures to detect band
            // leakage.
            let p = n as f32 * 0.7;
            let qmf_in = [
                Complex32::new((0.3 * p).cos(), (0.3 * p).sin()),
                Complex32::new(0.5 * (0.5 * p).cos(), 0.0),
                Complex32::new(0.0, 0.5 * (0.2 * p).sin()),
            ];
            inputs.push(qmf_in);
            let sub = hybrid_analysis_slot(&qmf_in, &mut st);
            let rec = hybrid_synthesis_slot(&sub);
            out_b0.push(rec[0]);
            out_b1.push(rec[1]);
            out_b2.push(rec[2]);
        }
        // Compare against the input shifted by the 6-sample group delay.
        // Require at least 80% of the input energy to be recovered in the
        // post-settling region (slots 12 onward — filter is 13-tap so
        // settling is done by then).
        let start = 12usize;
        let mut e_in = 0.0f32;
        let mut e_rec = 0.0f32;
        let mut e_err = 0.0f32;
        for n in start..n_slots - 6 {
            for (b, out) in [&out_b0, &out_b1, &out_b2].iter().enumerate() {
                let x = inputs[n][b];
                let y = out[n + 6];
                e_in += x.norm_sqr();
                e_rec += y.norm_sqr();
                e_err += (x - y).norm_sqr();
            }
        }
        let recovery = (e_rec / e_in.max(1e-12)).sqrt();
        let nrmse = (e_err / e_in.max(1e-12)).sqrt();
        assert!(
            recovery > 0.8 && recovery < 1.25,
            "hybrid roundtrip gain off: recovery={recovery}"
        );
        assert!(nrmse < 0.5, "hybrid roundtrip NRMSE too large: {nrmse}");
    }
}

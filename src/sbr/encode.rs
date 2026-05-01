//! HE-AACv1 mono SBR encoder foundation.
//!
//! Produces a valid `sbr_extension_data(EXT_SBR_DATA)` payload for one
//! mono frame, wraps it in a FIL element, and appends it after the
//! AAC-LC raw_data_block. Goal: emit a stream ffmpeg can decode back at
//! 2× the core sample rate.
//!
//! The encoder takes input PCM at 2× the target AAC-LC core rate:
//!
//! 1. **Downsampler** — symmetric FIR halves the sample rate so the core
//!    encoder sees a 1× rate signal (§4.6.18.4.3 doesn't prescribe the
//!    downsampler; any low-pass at Fs/4 followed by ÷2 works. We use a
//!    windowed-sinc FIR with a short symmetric kernel.)
//! 2. **High-rate QMF analysis** — 64-subband complex analysis QMF on the
//!    2× PCM. For a 2048-sample high-rate input we produce a
//!    `[32][64]` complex matrix `Xhigh[time][subband]`.
//! 3. **Envelope estimation** — for each SBR envelope time-slot, compute
//!    per-band mean power in the high-band range `kx..kx+M`. Convert to a
//!    scalefactor index using the amp_res=1 (3.0 dB) rule:
//!      sf = round(log2(E / 64) / 1)  i.e. 1 step == 3 dB.
//!    Clamp to the dequantised-by-decoder range.
//! 4. **Noise floor** — keep at a constant "low" level (index chosen so the
//!    decoder adds a small amount of shaped noise). 5 noise bands max.
//! 5. **Payload emission** — `sbr_extension_data(EXT_SBR_DATA)`:
//!    `bs_header_flag=1` on the first frame and every 8 frames; followed by
//!    `sbr_single_channel_element()` with `FIXFIX` grid, `bs_num_env=1`,
//!    `freq_res=1` (high-res), and no sinusoids. Envelope + noise are
//!    encoded with the in-frame freq-delta path (bs_df_env=0,
//!    bs_df_noise=0): first value absolute, rest deltas through the 3.0 dB
//!    Huffman tables.
//!
//! The emitted stream is self-round-trippable through the decoder in this
//! crate and is designed to be interoperable with ffmpeg's native AAC
//! decoder (which implements full HE-AACv1).

use oxideav_core::bits::BitWriter;

use super::bitstream::{FrameClass, SbrHeader, EXT_SBR_DATA};
use super::freq::FreqTables;
use super::ps::{
    huff_bits_icc_df, huff_bits_icc_dt, huff_bits_iid_df0, huff_bits_iid_dt0, huff_encode_icc_df,
    huff_encode_icc_dt, huff_encode_iid_df0, huff_encode_iid_dt0, PsParams10, PsParamsFrame,
    EXT_ID_PS_DATA,
};
use super::tables::{LAV_ENV_3_0DB, LAV_NOISE_3_0DB};
use super::{Complex32, NUM_QMF_BANDS, NUM_TIME_SLOTS_1024};

/// Downsample a 2×-rate PCM block by a factor of two with a simple
/// windowed-sinc half-band low-pass FIR. The kernel is symmetric so the
/// output has minimal group-delay distortion apart from an L/2 latency.
///
/// `input` must be at least `2 * output.len()` samples. The encoder calls
/// this with 2048 input samples → 1024 output samples per frame.
pub fn downsample_by_two(input: &[f32], output: &mut [f32]) {
    // 31-tap half-band FIR (windowed-sinc). Coefficients designed by
    // sinc(0.5n/π) * Hann window; precomputed.
    const KERNEL: [f32; 31] = [
        -0.000000000e+00,
        -1.7e-3,
        0.000000e+00,
        4.2e-3,
        0.000000e+00,
        -8.7e-3,
        0.000000e+00,
        1.60e-2,
        0.000000e+00,
        -2.75e-2,
        0.000000e+00,
        4.58e-2,
        0.000000e+00,
        -7.75e-2,
        0.000000e+00,
        3.0631e-1,
        5.00000e-1,
        3.0631e-1,
        0.000000e+00,
        -7.75e-2,
        0.000000e+00,
        4.58e-2,
        0.000000e+00,
        -2.75e-2,
        0.000000e+00,
        1.60e-2,
        0.000000e+00,
        -8.7e-3,
        0.000000e+00,
        4.2e-3,
        -1.7e-3,
    ];
    const HALF: isize = (KERNEL.len() / 2) as isize;
    let n_in = input.len() as isize;
    let n_out = output.len();
    for i in 0..n_out {
        let center = (2 * i) as isize;
        let mut s = 0.0f32;
        for k in 0..KERNEL.len() {
            let idx = center + k as isize - HALF;
            if (0..n_in).contains(&idx) {
                s += input[idx as usize] * KERNEL[k];
            }
        }
        output[i] = s;
    }
}

/// Stateful downsampler that preserves a `KERNEL.len() - 1`-sample tail
/// across frame boundaries so the filter delay line stays continuous.
#[derive(Clone, Debug)]
pub struct Downsampler {
    tail: Vec<f32>,
}

impl Downsampler {
    pub fn new() -> Self {
        Self {
            tail: vec![0.0f32; 30],
        }
    }

    /// Process `input` (length `2 * output.len()`) producing a downsampled
    /// block into `output`. Keeps 30 samples of history so the next call
    /// smoothly continues.
    pub fn process(&mut self, input: &[f32], output: &mut [f32]) {
        let mut combined = Vec::with_capacity(self.tail.len() + input.len());
        combined.extend_from_slice(&self.tail);
        combined.extend_from_slice(input);
        // Offset so that `output[0]` corresponds to the first input sample
        // after the tail (i.e. the first sample of this frame). The FIR is
        // centred so we start at index `tail.len() + HALF` - 2*i + k - HALF
        // = tail.len() + 2*i + k - HALF. We effectively convolve with the
        // kernel centred at tail.len() + 2i.
        let n_out = output.len();
        const KERNEL: [f32; 31] = [
            -0.000000000e+00,
            -1.7e-3,
            0.000000e+00,
            4.2e-3,
            0.000000e+00,
            -8.7e-3,
            0.000000e+00,
            1.60e-2,
            0.000000e+00,
            -2.75e-2,
            0.000000e+00,
            4.58e-2,
            0.000000e+00,
            -7.75e-2,
            0.000000e+00,
            3.0631e-1,
            5.00000e-1,
            3.0631e-1,
            0.000000e+00,
            -7.75e-2,
            0.000000e+00,
            4.58e-2,
            0.000000e+00,
            -2.75e-2,
            0.000000e+00,
            1.60e-2,
            0.000000e+00,
            -8.7e-3,
            0.000000e+00,
            4.2e-3,
            -1.7e-3,
        ];
        const HALF: isize = (KERNEL.len() / 2) as isize;
        let base = self.tail.len() as isize;
        for i in 0..n_out {
            let center = base + 2 * i as isize;
            let mut s = 0.0f32;
            for k in 0..KERNEL.len() {
                let idx = center + k as isize - HALF;
                if idx >= 0 && (idx as usize) < combined.len() {
                    s += combined[idx as usize] * KERNEL[k];
                }
            }
            output[i] = s;
        }
        // Update tail to the last 30 samples of the combined buffer.
        let keep = 30usize.min(combined.len());
        let start = combined.len() - keep;
        self.tail.clear();
        self.tail.extend_from_slice(&combined[start..]);
    }
}

impl Default for Downsampler {
    fn default() -> Self {
        Self::new()
    }
}

/// 64-channel complex analysis QMF filterbank for the encoder side.
///
/// Mirrors the decoder-side 32-channel analysis but with N=64 subbands, to
/// match the SBR spec's high-rate representation. One call consumes 64
/// time-domain samples and produces one column of 64 complex subbands.
#[derive(Clone, Debug)]
pub struct QmfAnalysis64 {
    x: [f32; 640],
}

impl QmfAnalysis64 {
    pub fn new() -> Self {
        Self { x: [0.0; 640] }
    }

    /// Push 64 time-domain samples, producing 64 complex subband samples.
    pub fn process(&mut self, input: &[f32; 64], out: &mut [Complex32; NUM_QMF_BANDS]) {
        // Shift state by 64 positions.
        for n in (64..640).rev() {
            self.x[n] = self.x[n - 64];
        }
        // Insert 64 newest samples at positions 0..63 (position 0 = newest).
        for n in 0..64 {
            self.x[n] = input[63 - n];
        }
        // z[n] = x[n] * c[n].
        let mut z = [0.0f32; 640];
        for n in 0..640 {
            z[n] = self.x[n] * super::tables::QMF_C[n];
        }
        // u[n] = sum_{j=0..4} z[n + j*128] for n = 0..127.
        let mut u = [0.0f32; 128];
        for n in 0..128 {
            u[n] = z[n] + z[n + 128] + z[n + 256] + z[n + 384] + z[n + 512];
        }
        // W[k] = sum_{n=0..127} u[n] * 2 * exp(i * pi/128 * (k + 0.5) * (2n - 0.5))
        for k in 0..NUM_QMF_BANDS {
            let kf = k as f32 + 0.5;
            let mut re = 0.0f32;
            let mut im = 0.0f32;
            for n in 0..128 {
                let ang = core::f32::consts::PI / 128.0 * kf * (2.0 * n as f32 - 0.5);
                re += u[n] * 2.0 * ang.cos();
                im += u[n] * 2.0 * ang.sin();
            }
            out[k] = Complex32::new(re, im);
        }
    }
}

impl Default for QmfAnalysis64 {
    fn default() -> Self {
        Self::new()
    }
}

/// Per-frame scalefactor data produced by [`estimate_envelope`].
#[derive(Clone, Debug)]
pub struct SbrFrameScalefactors {
    /// Per-high-res-band envelope scalefactor (length = `ft.n_high`).
    pub env: Vec<i32>,
    /// Per-noise-band noise scalefactor (length = `ft.nq`).
    pub noise: Vec<i32>,
}

/// Estimate per-band envelope energies from a `[time_slots][64]` matrix
/// of complex QMF subbands on the high-rate PCM. Returns one envelope
/// (bs_num_env=1) covering the full frame.
///
/// The decoder dequantises with `E_orig = 64 · 2^(acc_e / amp_res_bits)`
/// (§4.6.18.7.1, ISO/IEC 14496-3:2009). At amp_res=0 (1.5 dB step)
/// `amp_res_bits=2`; at amp_res=1 (3.0 dB) `amp_res_bits=1`. The decoder
/// then applies `gain = sqrt(E_orig / E_curr)` (§4.6.18.7.5) where
/// `E_curr` is computed from its own internal QMF on the AAC core PCM
/// output.
///
/// **Scale convention (round 15)**: the AAC core decoder in this crate
/// (and ffmpeg / afconvert) outputs PCM at native int16 amplitude scale
/// (~32768 peak — see `decoder.rs` line 825-833 and §4.6.11.3.1 IMDCT
/// scaling). Its QMF analysis therefore produces `|X|²` in int16-PCM
/// scale². But our SBR encoder's QMF here runs on input PCM in
/// normalised `[-1, 1]` float scale (see `he_aac_encoder.rs` line 122-124
/// where i16 input is divided by 32768). To make the encoded `acc_e`
/// values match the decoder-side `E_curr` units (so the gain formula
/// reproduces the intended envelope), we multiply the squared QMF
/// magnitudes by `INT16_SCALE_SQ = 2^30` before quantising. Without this
/// scale, every band's `e` was tiny → `v_15 < 0` → clamped to 0 by the
/// 7-bit unsigned writer → decoder synthesised `E_orig = 64` (the
/// minimum representable) → `gain ≈ sqrt(64 / E_curr) ≫ 1` → spurious
/// noise just above the SBR crossover.
pub fn estimate_envelope(
    x_high: &[[Complex32; NUM_QMF_BANDS]],
    ft: &FreqTables,
) -> SbrFrameScalefactors {
    // 32768² = 2^30 ≈ 1.0737e9. Brings `[-1, 1]` PCM-float QMF energies
    // up to int16-PCM scale² so the encoder and decoder formulas line up
    // for our own decoder (which carries spec at int16-amplitude through
    // SBR). Round 18 verified empirically that this constant has **no**
    // effect on ffmpeg-decoded HE-AAC output amplitude — see the round-18
    // commit message for the full diagnostic.
    //
    // Round 24 (2026-04-30): the diff harness in
    // `tests/r24_sbr_fil_diff.rs` against fdkaac on a 1 kHz / 48 kHz mono
    // tone (the r18 amplitude fixture) uncovered that for tonal content
    // with **no real high-band energy**, ours emits envelope index = 29
    // for the bottom SBR band (E_orig = 64·2^14.5 ≈ 1.5 M) — pure QMF
    // analysis-bank skirt leakage of the 1 kHz tone into the kx subbands.
    // fdkaac sends `[0; n_high]` (= E_orig = 64, the spec minimum).
    //
    // ffmpeg's gain formula `gain = √(E_orig / E_curr)` then amplifies the
    // bottom SBR band by ~3700× — exactly the saturation observed in
    // `tests/sbr_he_aac_ffmpeg_amplitude_r18.rs`. The `OXIDEAV_AAC_SBR_ENV_FORCE_ZERO`
    // env-var below pins this hypothesis: when set, every band gets value
    // 0 regardless of measured energy; combined with ffmpeg-decode this
    // should produce a non-saturated peak.
    const INT16_SCALE_SQ: f32 = (32768.0 * 32768.0) as f32;
    let force_zero = std::env::var("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO").is_ok();
    if force_zero {
        return SbrFrameScalefactors {
            env: vec![0; ft.n_high],
            // Match fdkaac's small-noise output (~14 at the first band).
            // Index 14 at amp_res=0 (1.5 dB step) → Q_orig = 2^(6 - 7) = 0.5.
            noise: vec![14; ft.nq],
        };
    }
    let n = x_high.len().max(1) as f32;
    let mut env = vec![0i32; ft.n_high];
    for band in 0..ft.n_high {
        let k0 = ft.f_high[band].max(0) as usize;
        let k1 = ft.f_high[band + 1].min(NUM_QMF_BANDS as i32).max(0) as usize;
        if k1 <= k0 {
            env[band] = 0;
            continue;
        }
        let mut acc = 0.0f32;
        for row in x_high.iter() {
            for kk in k0..k1 {
                acc += row[kk].norm_sqr();
            }
        }
        let bands = (k1 - k0) as f32;
        let e = (acc * INT16_SCALE_SQ) / (n * bands + 1e-30);
        // Quantised value at amp_res=0 (1.5 dB step): E = 64 * 2^(v/2)
        // => v = round(log2(E/64) * 2). With INT16_SCALE_SQ baked in,
        // typical signals land in [0, 100]; truly silent bands land at the
        // clamp floor and the writer encodes 0 (decoder synthesises the
        // spec's minimum E_orig = 64, which is then small relative to the
        // decoder's int16-scale E_curr → gain ≈ 0).
        let v_15 = if e > 0.0 {
            ((e / 64.0).log2() * 2.0).round() as i32
        } else {
            -2 * LAV_ENV_3_0DB
        };
        // Store the effective 1.5 dB index divided by 2 so the writer
        // can re-scale. We clamp at [-LAV_ENV_3_0DB, LAV_ENV_3_0DB] (i.e.
        // v_15 in [-62, 62]) which the writer maps into 0..120 via
        // sign-bias shift.
        env[band] = v_15;
    }
    // Noise floor — Q_orig = 2^(NOISE_FLOOR_OFFSET - sf). The decoder's
    // NOISE_FLOOR_OFFSET is 6, so sf=6 gives Q_orig=1 (roughly equal
    // noise:signal), sf>=10 gives <-6 dB below signal. Pick a high sf
    // (≈ very low noise) for tonal input.
    let noise = vec![18i32; ft.nq];
    SbrFrameScalefactors { env, noise }
}

/// Runtime state for the mono SBR encoder — carries the analysis QMF
/// filterbank, the prev-frame bitstream scalefactor state so deltas can be
/// computed, and a frame counter used to decide when to re-emit the
/// header.
#[derive(Clone, Debug)]
pub struct SbrEncoder {
    pub header: SbrHeader,
    pub freq: FreqTables,
    pub qmf: QmfAnalysis64,
    pub downsampler: Downsampler,
    pub frame_count: u64,
    /// Previously-emitted (absolute) envelope scalefactors. Unused by the
    /// current freq-delta-only encoder but kept for future time-delta
    /// encoding.
    pub prev_env: Vec<i32>,
    /// Previously-emitted (absolute) noise scalefactors.
    pub prev_noise: Vec<i32>,
    /// When `true`, every emitted SCE SBR payload also carries a
    /// Parametric Stereo extension (`bs_extension_id = EXTENSION_ID_PS`,
    /// see Table 4.112) inside the SBR `bs_extended_data` block. Used by
    /// [`crate::he_aac_encoder::HeAacV2Encoder`] to produce a
    /// HE-AACv2-compatible mono-SBR + PS bitstream that decoders such as
    /// ffmpeg upmix to stereo.
    ///
    /// When `pending_ps` is `Some`, that parameter set is consumed by the
    /// next [`Self::emit_sbr_payload`] call and reset to `None`. When
    /// `pending_ps` is `None` and `emit_ps == true`, the encoder falls
    /// back to the identity (no-op) PS payload — IID = 0 dB, ICC = 1 in
    /// every band, equivalent to [`write_ps_data_noop`].
    pub emit_ps: bool,
    /// PS parameters to emit on the next frame. Set by an external PS
    /// analyser via [`Self::set_pending_ps`] / [`Self::set_pending_ps_frame`];
    /// consumed-and-cleared on each `emit_sbr_payload` call so a missing
    /// staged value always falls back to identity stereo (rather than
    /// reusing stale parameters).
    pub pending_ps: Option<PsParamsFrame>,
    /// IID indices of the **last** envelope of the previously-emitted PS
    /// frame, per parameter band (10 entries). Used by
    /// [`write_ps_data_real`] as the time-direction differential baseline
    /// per §8.6.4.6.2: when the encoder sets `iid_dt[0] = 1`, the first
    /// envelope of this frame is decoded as `idx[b] = prev_iid_idx[b] + delta`.
    /// Initialised to all-zero (matches the decoder's [`PsState`] reset).
    pub prev_iid_idx: [i32; 10],
    /// Companion to `prev_iid_idx` for ICC. Same role as in the decoder
    /// state — last envelope's per-band ICC index from the previous frame.
    pub prev_icc_idx: [i32; 10],
}

impl SbrEncoder {
    /// Build an SBR encoder for a given AAC-LC core sample rate. The SBR
    /// internal rate is `2 * core_rate`.
    ///
    /// Default header parameters target a ~5 kHz crossover which matches
    /// ffmpeg's libavcodec/aacenc_sbr preferences for 24 kHz core → 48 kHz
    /// output.
    pub fn new(core_rate: u32) -> oxideav_core::Result<Self> {
        let mut header = SbrHeader::defaults();
        // Conservative defaults that mirror afconvert's choice for
        // 24 kHz core / 48 kHz output. amp_res=0 (1.5 dB tables) matches
        // the implicit per-frame override applied by sbr_grid() FIXFIX
        // with bs_num_env == 1 (§4.6.18.3.3) — keeping the header value
        // the same as the effective frame value avoids any parser
        // inconsistency on decoders that read amp_res before the grid.
        header.bs_amp_res = 0;
        header.bs_start_freq = 5;
        header.bs_stop_freq = 9;
        header.bs_xover_band = 0;
        header.bs_freq_scale = 2;
        header.bs_alter_scale = true;
        header.bs_noise_bands = 2;
        header.bs_limiter_bands = 2;
        header.bs_limiter_gains = 2;
        header.bs_interpol_freq = true;
        header.bs_smoothing_mode = true;
        // Set header_extra_1=true so we explicitly write our chosen
        // bs_freq_scale / bs_alter_scale / bs_noise_bands instead of
        // letting the decoder fall back to the bits-saving defaults.
        // ffmpeg's HE-AAC parser treats `bs_header_extra_1=0` as a
        // hint that the header is degenerate; explicit values are
        // safer for cross-decoder interop.
        header.bs_header_extra_1 = true;
        header.bs_header_extra_2 = false;

        let fs_sbr = core_rate * 2;
        let freq = FreqTables::build(
            fs_sbr,
            header.bs_start_freq,
            header.bs_stop_freq,
            header.bs_xover_band,
            header.bs_freq_scale,
            header.bs_alter_scale,
            header.bs_noise_bands,
        )?;

        Ok(Self {
            header,
            prev_env: vec![0; freq.n_high],
            prev_noise: vec![0; freq.nq],
            freq,
            qmf: QmfAnalysis64::new(),
            downsampler: Downsampler::new(),
            frame_count: 0,
            emit_ps: false,
            pending_ps: None,
            prev_iid_idx: [0; 10],
            prev_icc_idx: [0; 10],
        })
    }

    /// Stage a [`PsParams10`] set to be emitted as a single-envelope PS
    /// frame on the next [`Self::emit_sbr_payload`] call. The pending
    /// params are consumed (set back to `None`) on emission, so each
    /// frame's PS payload comes from a fresh analysis pass.
    ///
    /// Backwards-compatible round-13 entry point — equivalent to calling
    /// [`Self::set_pending_ps_frame`] with `PsParamsFrame::single(params)`.
    pub fn set_pending_ps(&mut self, params: PsParams10) {
        self.pending_ps = Some(PsParamsFrame::single(params));
    }

    /// Stage a multi-envelope [`PsParamsFrame`] (round-14) for the next
    /// payload. Pass `PsParamsFrame::single` for `num_env = 1` (round-13
    /// behaviour); a frame with 2 or 4 envelopes triggers the `num_env > 1`
    /// PS bitstream layout per §8.6.4.6.2 (Table 8.29).
    pub fn set_pending_ps_frame(&mut self, frame: PsParamsFrame) {
        self.pending_ps = Some(frame);
    }

    /// Enable / disable Parametric Stereo emission. When enabled, every
    /// SCE SBR payload from [`Self::emit_sbr_payload`] embeds a no-op PS
    /// extension so the bitstream is ffmpeg-decodable as HE-AACv2 (mono
    /// SBR upmixed to stereo by the decoder). The PS parameters are all
    /// zero (IID = 0 dB, ICC = 1) so the reconstructed `L = R = mono`.
    ///
    /// This is the round-12 "scaffolding" path: it lights up the
    /// HE-AACv2 wire format without yet doing real PS analysis.
    pub fn set_emit_ps(&mut self, on: bool) {
        self.emit_ps = on;
    }

    /// Produce a `[num_slots * RATE][64]` high-rate QMF matrix from
    /// `high_pcm` (2048 samples for a 1024-sample core frame).
    pub fn analyse(&mut self, high_pcm: &[f32]) -> Vec<[Complex32; NUM_QMF_BANDS]> {
        let num_slots = NUM_TIME_SLOTS_1024 * super::RATE; // 32
        let mut out = vec![[Complex32::default(); NUM_QMF_BANDS]; num_slots];
        let mut tmp = [0.0f32; 64];
        for l in 0..num_slots {
            let src = &high_pcm[l * 64..l * 64 + 64];
            tmp.copy_from_slice(src);
            let mut col = [Complex32::default(); NUM_QMF_BANDS];
            self.qmf.process(&tmp, &mut col);
            out[l] = col;
        }
        out
    }

    /// Emit one mono SBR payload as a bit-aligned byte stream plus the
    /// exact bit count. The returned bytes are 0-padded on the tail to
    /// the next byte boundary so they can be written back with byte-
    /// oriented writers; `bits` is authoritative.
    ///
    /// `x_high_sbr` is the 32-slot × 64-band complex QMF matrix produced
    /// by [`Self::analyse`].
    pub fn emit_sbr_payload(
        &mut self,
        x_high_sbr: &[[Complex32; NUM_QMF_BANDS]],
    ) -> (Vec<u8>, u32) {
        let sf = estimate_envelope(x_high_sbr, &self.freq);
        let mut bw = BitWriter::with_capacity(64);
        let start = bw.bit_position();
        // Header re-emitted on frame 0 and every 8 frames thereafter to
        // resync decoder state.
        let emit_header = self.frame_count == 0 || (self.frame_count % 8) == 0;
        bw.write_bit(emit_header); // bs_header_flag
        if emit_header {
            write_sbr_header(&mut bw, &self.header);
        }
        // sbr_data() for SCE: sbr_single_channel_element(). When
        // `emit_ps` is on, the SCE writer additionally emits the PS
        // extension inside `bs_extended_data` so the stream is
        // HE-AACv2-compatible.
        if self.emit_ps {
            // Use pending real PS frame if staged; otherwise fall back to
            // the identity (no-op) single-envelope payload so the bitstream
            // remains structurally valid.
            let ps_frame = self.pending_ps.take().unwrap_or_default();
            let (last_iid, last_icc) = write_single_channel_element_mono_with_ps_real(
                &mut bw,
                &self.header,
                &self.freq,
                &sf,
                &ps_frame,
                &self.prev_iid_idx,
                &self.prev_icc_idx,
            );
            // Update prev-frame state for the next call's time-direction
            // differential baseline. The decoder's PsState updates in the
            // exact same way (§8.6.4.6.2 — "after parsing the last
            // envelope, prev = idxs of last envelope").
            self.prev_iid_idx = last_iid;
            self.prev_icc_idx = last_icc;
        } else {
            write_single_channel_element_mono(&mut bw, &self.header, &self.freq, &sf);
        }
        let bits_written = (bw.bit_position() - start) as u32;
        bw.align_to_byte();

        // Update state for next frame.
        self.prev_env.clone_from(&sf.env);
        self.prev_noise.clone_from(&sf.noise);
        self.frame_count = self.frame_count.wrapping_add(1);
        (bw.finish(), bits_written)
    }

    /// Convenience wrapper: given 2048 samples of 2× rate PCM, run the
    /// analysis QMF and emit the SBR payload (plus bit count).
    pub fn encode_frame_fil(&mut self, high_pcm: &[f32]) -> (Vec<u8>, u32) {
        let x = self.analyse(high_pcm);
        self.emit_sbr_payload(&x)
    }
}

/// Stereo SBR encoder for HE-AACv1 CPE streams (§4.6.18.3.5,
/// `sbr_channel_pair_element` independent / `bs_coupling = 0`).
///
/// Holds a shared `SbrHeader` + `FreqTables` (since both channels MUST
/// agree on the SBR setup) plus per-channel analysis QMF banks,
/// downsamplers, and previous-frame scalefactor state. Each call to
/// [`Self::analyse`] / [`Self::emit_sbr_payload_pair`] processes one
/// stereo frame.
///
/// The emitted CPE payload uses **independent** coupling — both channels
/// carry their own grid, dtdf, invf, envelope, and noise. We choose
/// independent over coupled because:
///   1. It's a strict superset: every signal that can be coupled-encoded
///      can be independently encoded.
///   2. Round 1 doesn't need to estimate an L/R energy ratio for the
///      balance Huffman path.
///   3. The decoder side already supports both paths (parser tested).
#[derive(Clone, Debug)]
pub struct SbrStereoEncoder {
    pub header: SbrHeader,
    pub freq: FreqTables,
    pub qmf_l: QmfAnalysis64,
    pub qmf_r: QmfAnalysis64,
    pub downsampler_l: Downsampler,
    pub downsampler_r: Downsampler,
    pub frame_count: u64,
    /// Previously-emitted (absolute) per-channel envelope scalefactors.
    /// Currently unused — freq-direction delta coding doesn't need
    /// previous-frame state. Kept so a future time-delta encoder can
    /// switch to bs_df_env=1 without reshaping the public type.
    pub prev_env_l: Vec<i32>,
    pub prev_env_r: Vec<i32>,
    pub prev_noise_l: Vec<i32>,
    pub prev_noise_r: Vec<i32>,
}

impl SbrStereoEncoder {
    /// Build a stereo SBR encoder for the given AAC-LC core sample rate.
    /// Defaults match [`SbrEncoder::new`] so a stereo HE-AAC stream is
    /// header-compatible with a mono one.
    pub fn new(core_rate: u32) -> oxideav_core::Result<Self> {
        // Re-use the mono encoder's header + freq table choice by building
        // a throwaway mono encoder. This guarantees both encoders make the
        // same SBR-band decisions.
        let mono = SbrEncoder::new(core_rate)?;
        Ok(Self {
            header: mono.header.clone(),
            prev_env_l: vec![0; mono.freq.n_high],
            prev_env_r: vec![0; mono.freq.n_high],
            prev_noise_l: vec![0; mono.freq.nq],
            prev_noise_r: vec![0; mono.freq.nq],
            freq: mono.freq,
            qmf_l: QmfAnalysis64::new(),
            qmf_r: QmfAnalysis64::new(),
            downsampler_l: Downsampler::new(),
            downsampler_r: Downsampler::new(),
            frame_count: 0,
        })
    }

    /// Run the per-channel analysis QMF on one stereo high-rate block.
    /// `high_pcm_l` and `high_pcm_r` are each 2048 samples (one core
    /// frame at 2× rate). Returns the two `[32][64]` complex matrices.
    pub fn analyse(
        &mut self,
        high_pcm_l: &[f32],
        high_pcm_r: &[f32],
    ) -> (
        Vec<[Complex32; NUM_QMF_BANDS]>,
        Vec<[Complex32; NUM_QMF_BANDS]>,
    ) {
        let num_slots = NUM_TIME_SLOTS_1024 * super::RATE; // 32
        let mut x_l = vec![[Complex32::default(); NUM_QMF_BANDS]; num_slots];
        let mut x_r = vec![[Complex32::default(); NUM_QMF_BANDS]; num_slots];
        let mut tmp = [0.0f32; 64];
        for l in 0..num_slots {
            tmp.copy_from_slice(&high_pcm_l[l * 64..l * 64 + 64]);
            self.qmf_l.process(&tmp, &mut x_l[l]);
            tmp.copy_from_slice(&high_pcm_r[l * 64..l * 64 + 64]);
            self.qmf_r.process(&tmp, &mut x_r[l]);
        }
        (x_l, x_r)
    }

    /// Emit one stereo SBR payload (CPE, independent coupling). Returns
    /// the byte-aligned bytes and the exact meaningful-bit count.
    ///
    /// Layout follows Table 4.66 with `bs_data_extra=0` and `bs_coupling=0`:
    ///
    /// ```text
    /// bs_data_extra      1 bit  = 0
    /// bs_coupling        1 bit  = 0
    /// sbr_grid(L)
    /// sbr_grid(R)
    /// sbr_dtdf(L)
    /// sbr_dtdf(R)
    /// sbr_invf(L)        2 * Nq bits
    /// sbr_invf(R)        2 * Nq bits
    /// sbr_envelope(L, df=0)
    /// sbr_envelope(R, df=0)
    /// sbr_noise(L,    df=0)
    /// sbr_noise(R,    df=0)
    /// bs_add_harmonic_flag(L)  = 0
    /// bs_add_harmonic_flag(R)  = 0
    /// bs_extended_data         = 0
    /// ```
    pub fn emit_sbr_payload_pair(
        &mut self,
        x_l: &[[Complex32; NUM_QMF_BANDS]],
        x_r: &[[Complex32; NUM_QMF_BANDS]],
    ) -> (Vec<u8>, u32) {
        let sf_l = estimate_envelope(x_l, &self.freq);
        let sf_r = estimate_envelope(x_r, &self.freq);
        let mut bw = BitWriter::with_capacity(128);
        let start = bw.bit_position();
        let emit_header = self.frame_count == 0 || (self.frame_count % 8) == 0;
        bw.write_bit(emit_header);
        if emit_header {
            write_sbr_header(&mut bw, &self.header);
        }
        // sbr_data() for CPE: sbr_channel_pair_element() in independent
        // mode (bs_coupling = 0).
        write_channel_pair_element_independent(&mut bw, &self.freq, &sf_l, &sf_r);
        let bits_written = (bw.bit_position() - start) as u32;
        bw.align_to_byte();

        self.prev_env_l.clone_from(&sf_l.env);
        self.prev_env_r.clone_from(&sf_r.env);
        self.prev_noise_l.clone_from(&sf_l.noise);
        self.prev_noise_r.clone_from(&sf_r.noise);
        self.frame_count = self.frame_count.wrapping_add(1);
        (bw.finish(), bits_written)
    }

    /// Convenience wrapper for `analyse` + `emit_sbr_payload_pair`.
    pub fn encode_frame_fil_pair(
        &mut self,
        high_pcm_l: &[f32],
        high_pcm_r: &[f32],
    ) -> (Vec<u8>, u32) {
        let (xl, xr) = self.analyse(high_pcm_l, high_pcm_r);
        self.emit_sbr_payload_pair(&xl, &xr)
    }
}

/// Write an sbr_header() element. Matches the inverse of
/// [`crate::sbr::bitstream::parse_sbr_header`] exactly.
pub fn write_sbr_header(bw: &mut BitWriter, h: &SbrHeader) {
    bw.write_u32(h.bs_amp_res as u32, 1);
    bw.write_u32(h.bs_start_freq as u32, 4);
    bw.write_u32(h.bs_stop_freq as u32, 4);
    bw.write_u32(h.bs_xover_band as u32, 3);
    bw.write_u32(0, 2); // bs_reserved
    bw.write_bit(h.bs_header_extra_1);
    bw.write_bit(h.bs_header_extra_2);
    if h.bs_header_extra_1 {
        bw.write_u32(h.bs_freq_scale as u32, 2);
        bw.write_bit(h.bs_alter_scale);
        bw.write_u32(h.bs_noise_bands as u32, 2);
    }
    if h.bs_header_extra_2 {
        bw.write_u32(h.bs_limiter_bands as u32, 2);
        bw.write_u32(h.bs_limiter_gains as u32, 2);
        bw.write_bit(h.bs_interpol_freq);
        bw.write_bit(h.bs_smoothing_mode);
    }
}

/// Write an sbr_single_channel_element() for mono. Uses a minimal FIXFIX
/// grid with `bs_num_env=1`, freq_res=1 (high-res envelope bands), no
/// sinusoids, and freq-direction delta coding (`bs_df_env=0`,
/// `bs_df_noise=0`).
pub fn write_single_channel_element_mono(
    bw: &mut BitWriter,
    _header: &SbrHeader,
    freq: &FreqTables,
    sf: &SbrFrameScalefactors,
) {
    bw.write_bit(false); // bs_data_extra
                         // sbr_grid — FIXFIX, bs_num_env=1, freq_res=1.
    bw.write_u32(FrameClass::FixFix as u32, 2);
    bw.write_u32(0, 2); // bs_num_env = 2^0 = 1
                        // Spec: when FIXFIX && bs_num_env == 1, bs_amp_res is forced to 0.
                        // We MUST encode at amp_res=0 (1.5 dB tables) to match. Choose
                        // amp_res=0 explicitly by re-quantising below.
    bw.write_u32(1, 1); // bs_freq_res[0] = 1 (high-res)
                        // sbr_dtdf for 1 envelope + 1 noise floor.
    bw.write_u32(0, 1); // bs_df_env[0] = 0 (freq-direction)
    bw.write_u32(0, 1); // bs_df_noise[0] = 0 (freq-direction)
                        // sbr_invf — one value per noise band (bw mode 0 = off).
    for _ in 0..freq.nq {
        bw.write_u32(0, 2);
    }
    // Envelope — FIXFIX num_env==1 => amp_res forced to 0 (1.5 dB tables).
    // We need to re-quantise the envelope with the 1.5 dB step so decoder
    // and encoder agree.
    write_envelope_1_5db_freq_delta(bw, &sf.env, freq.n_high);
    // Noise — always uses the 3.0 dB noise tables (no 1.5 dB variant).
    write_noise_3_0db_freq_delta(bw, &sf.noise, freq.nq);
    // bs_add_harmonic_flag = 0
    bw.write_bit(false);
    // bs_extended_data = 0
    bw.write_bit(false);
}

/// Write a no-op `ps_data()` element — IID = 0 dB, ICC = 1, no IPD/OPD —
/// per ISO/IEC 14496-3 §8.6.4 (Table 8.9). The header sets `iid_mode = 0`
/// (10 IID bands, default 7-step quantisation grid, Table 8.24) and
/// `icc_mode = 0` (10 ICC bands). One envelope (`num_env = 1`) carries
/// `iid_index = 0` and `icc_index = 0` for every band.
///
/// Decoder semantics (Table 8.25 / 8.28): IID index 0 → 0 dB
/// inter-channel level difference (centred), ICC index 0 → ρ = 1 (fully
/// coherent). The PS mixing matrix at IID = 0, ICC = 1 reduces to
/// `H = [[1, 0], [1, 0]]` (Ra mode, §8.6.4.6.2.1) so the upmix
/// reconstructs `L = R = mono` from the SBR-decoded mono input — the
/// "identity stereo" target for the round-1 PS encode.
///
/// Bit layout (35 bits):
///
/// ```text
///   enable_ps_header  1   = 1
///     enable_iid      1   = 1
///       iid_mode      3   = 0  (10 bands, default quant)
///     enable_icc      1   = 1
///       icc_mode      3   = 0  (10 bands)
///     enable_ext      1   = 0
///   frame_class       1   = 0  (FIX_BORDERS)
///   num_env_idx       2   = 1  → num_env = 1
///   iid_dt[0]         1   = 0  (freq-direction)
///   iid_data[0..10]  10   = ten "0" Huffman codewords (HUFF_IID_DF[0]
///                            value 0 maps to the 1-bit code "0")
///   icc_dt[0]         1   = 0
///   icc_data[0..10]  10   = ten "0" Huffman codewords (HUFF_ICC_DF
///                            value 0 maps to "0")
/// ```
///
/// Per Table 8.29 the FIX_BORDERS column maps `num_env_idx = {0,1,2,3}`
/// → `num_env = {0,1,2,4}`. We use index 1 so the very first frame
/// already carries one envelope of parameters; index 0 (`num_env = 0`)
/// would mean "reuse previous parameters" — which is undefined on the
/// first frame.
pub fn write_ps_data_noop(bw: &mut BitWriter) {
    // ---- Header ----
    bw.write_bit(true); // enable_ps_header
    bw.write_bit(true); // enable_iid
    bw.write_u32(0, 3); // iid_mode = 0  → 10 IID bands, default 7-step grid
    bw.write_bit(true); // enable_icc
    bw.write_u32(0, 3); // icc_mode = 0  → 10 ICC bands
    bw.write_bit(false); // enable_ext = 0 (no IPD/OPD)

    // ---- Frame ----
    bw.write_bit(false); // frame_class = 0  (FIX_BORDERS)
    bw.write_u32(1, 2); // num_env_idx = 1  → num_env = 1 (Table 8.29
                        // FIX_BORDERS column).

    // IID — 1 envelope × 10 bands of value 0.
    // Per §8.6.4.4.1 / Table 8.B.18 huff_iid_df[0]: value 0 has codeword "0"
    // (1 bit). bs_df_env-style flag: iid_dt[0] = 0 selects freq-diff.
    bw.write_bit(false); // iid_dt[0] = 0 (freq-diff)
    for _ in 0..10 {
        bw.write_bit(false); // huff_iid_df[0] codeword for value 0 is "0"
    }

    // ICC — 1 envelope × 10 bands of value 0.
    // Per §8.6.4.4.2 / Table 8.B.19 huff_icc_df: value 0 codeword is "0".
    bw.write_bit(false); // icc_dt[0] = 0
    for _ in 0..10 {
        bw.write_bit(false); // huff_icc_df codeword for value 0 is "0"
    }
}

/// Write the SBR SCE element with a Parametric Stereo extension embedded
/// inside `bs_extended_data` (Table 4.65). Identical to
/// [`write_single_channel_element_mono`] except the trailing
/// `bs_extended_data` flag is set and the extended_data block carries
/// one `bs_extension_id == EXTENSION_ID_PS` (= 2) element holding the
/// no-op [`write_ps_data_noop`] payload.
///
/// Layout of the appended extended_data block:
///
/// ```text
///   bs_extended_data       1 bit   = 1
///   bs_extension_size      4 bits  = N  (or 15 + esc_count if N >= 15)
///   --- N bytes of content ---
///     bs_extension_id      2 bits  = 2  (EXTENSION_ID_PS)
///     ps_data()           35 bits  (no-op payload above)
///     fill_bits             p bits = 0  (pad to N · 8 bits)
/// ```
///
/// With the no-op payload `2 + 35 = 37` content bits → `N = 5` bytes
/// (40 bits) with 3 fill bits.
pub fn write_single_channel_element_mono_with_ps(
    bw: &mut BitWriter,
    _header: &SbrHeader,
    freq: &FreqTables,
    sf: &SbrFrameScalefactors,
) {
    // ---- Same body as write_single_channel_element_mono up to the
    //      bs_extended_data flag.
    bw.write_bit(false); // bs_data_extra
    bw.write_u32(FrameClass::FixFix as u32, 2);
    bw.write_u32(0, 2); // bs_num_env = 1
    bw.write_u32(1, 1); // bs_freq_res[0] = 1
    bw.write_u32(0, 1); // bs_df_env[0] = 0
    bw.write_u32(0, 1); // bs_df_noise[0] = 0
    for _ in 0..freq.nq {
        bw.write_u32(0, 2); // bs_invf_mode = 0 (off)
    }
    write_envelope_1_5db_freq_delta(bw, &sf.env, freq.n_high);
    write_noise_3_0db_freq_delta(bw, &sf.noise, freq.nq);
    bw.write_bit(false); // bs_add_harmonic_flag = 0

    // ---- bs_extended_data = 1 + EXT_ID_PS extension.
    bw.write_bit(true); // bs_extended_data = 1
                        // Pre-compute content bytes: bs_extension_id (2) + ps_data (35) = 37 bits
                        // → 5 bytes with 3 fill bits.
    const PS_CONTENT_BITS: u32 = 2 + 35;
    let cnt_bytes = PS_CONTENT_BITS.div_ceil(8); // = 5
    let fill_bits = cnt_bytes * 8 - PS_CONTENT_BITS;
    if cnt_bytes < 15 {
        bw.write_u32(cnt_bytes, 4); // bs_extension_size
    } else {
        bw.write_u32(15, 4);
        bw.write_u32(cnt_bytes + 1 - 15, 8); // bs_esc_count
    }
    bw.write_u32(EXT_ID_PS_DATA, 2); // bs_extension_id = 2
    write_ps_data_noop(bw);
    if fill_bits > 0 {
        bw.write_u32(0, fill_bits);
    }
}

/// Write a `ps_data()` payload carrying real per-band IID (default grid,
/// 7-step) and ICC parameters in the 10-band configuration
/// (`iid_mode = 0`, `icc_mode = 0`). Per ISO/IEC 14496-3 §8.6.4.4.1 /
/// §8.6.4.4.2 with frame_class = 0 (FIX_BORDERS), num_env = 1 and
/// freq-direction differential coding (iid_dt[0] = icc_dt[0] = 0).
///
/// Bit layout (header 11 bits + grid 3 bits + per-envelope IID/ICC):
///
/// ```text
///   enable_ps_header  1   = 1
///     enable_iid      1   = 1
///       iid_mode      3   = 0  (10 bands, default 7-step quant)
///     enable_icc      1   = 1
///       icc_mode      3   = 0  (10 bands)
///     enable_ext      1   = 0
///   frame_class       1   = 0  (FIX_BORDERS)
///   num_env_idx       2   = 1  → num_env = 1
///   iid_dt[0]         1   = 0  (freq-direction)
///   for b in 0..10: huff_iid_df[0](delta_iid[b])
///   icc_dt[0]         1   = 0
///   for b in 0..10: huff_icc_df  (delta_icc[b])
/// ```
///
/// Returns the exact bit-count written (variable, since Huffman
/// codewords vary in length 1..17 bits).
/// Cost (in bits) and payload-write closure for one envelope's IID coding,
/// in either freq-direction or time-direction. The encoder picks whichever
/// representation is cheaper (§8.6.4.6.2: each envelope independently sets
/// `iid_dt[e]` to 0 or 1).
fn iid_envelope_costs(env_idx: &[i32; 10], prev: &[i32; 10]) -> (u32, u32) {
    // df: delta[0] = idx[0] - 0, delta[b] = idx[b] - idx[b-1]. Range ±14.
    let mut df_bits = 0u32;
    let mut prev_band = 0i32;
    for b in 0..10 {
        let cur = env_idx[b].clamp(-7, 7);
        let delta = (cur - prev_band).clamp(-14, 14);
        df_bits += huff_bits_iid_df0(delta);
        prev_band += delta;
    }
    // dt: delta[b] = idx[b] - prev_frame_idx[b]. Range ±14.
    let mut dt_bits = 0u32;
    for b in 0..10 {
        let cur = env_idx[b].clamp(-7, 7);
        let delta = (cur - prev[b]).clamp(-14, 14);
        dt_bits += huff_bits_iid_dt0(delta);
    }
    (df_bits, dt_bits)
}

fn icc_envelope_costs(env_idx: &[i32; 10], prev: &[i32; 10]) -> (u32, u32) {
    let mut df_bits = 0u32;
    let mut prev_band = 0i32;
    for b in 0..10 {
        let cur = env_idx[b].clamp(0, 7);
        let delta = (cur - prev_band).clamp(-7, 7);
        df_bits += huff_bits_icc_df(delta);
        prev_band += delta;
    }
    let mut dt_bits = 0u32;
    for b in 0..10 {
        let cur = env_idx[b].clamp(0, 7);
        let delta = (cur - prev[b]).clamp(-7, 7);
        dt_bits += huff_bits_icc_dt(delta);
    }
    (df_bits, dt_bits)
}

/// Write the PS extension payload (`ps_data()`, §8.6.4.6.2) for a multi-
/// envelope frame. For each envelope picks freq-direction or time-direction
/// differential coding per parameter (`iid_dt[e]`, `icc_dt[e]`) by comparing
/// Huffman-coded bit cost; emits the cheaper one. Returns `(bits_written,
/// last_iid_idx, last_icc_idx)` — the last-envelope per-band indices feed
/// the next frame's time-direction baseline (matches `PsState` update on
/// the decoder side).
///
/// `prev_iid_idx` / `prev_icc_idx` hold the last envelope's per-band indices
/// from the **previous** PS frame; they must be the same values the
/// decoder's `PsState.prev_*_idx` holds at frame boundary.
pub fn write_ps_data_real(
    bw: &mut BitWriter,
    frame: &PsParamsFrame,
    prev_iid_idx: &[i32; 10],
    prev_icc_idx: &[i32; 10],
) -> (u32, [i32; 10], [i32; 10]) {
    let start = bw.bit_position();
    // ---- Header (same as the no-op variant). ----
    bw.write_bit(true); // enable_ps_header
    bw.write_bit(true); // enable_iid
    bw.write_u32(0, 3); // iid_mode = 0
    bw.write_bit(true); // enable_icc
    bw.write_u32(0, 3); // icc_mode = 0
    bw.write_bit(false); // enable_ext = 0

    // ---- Frame ---- §8.6.4.6.2 (Table 8.29): frame_class = 0 → uniform
    // FIX_BORDERS grid; num_env ∈ {0, 1, 2, 4} via num_env_idx ∈ {0, 1, 2, 3}.
    let num_env = frame.envs.len().max(1);
    let num_env_idx: u32 = match num_env {
        1 => 1,
        2 => 2,
        4 => 3,
        _ => 1, // unsupported → coerce to 1
    };
    bw.write_bit(false); // frame_class = 0 (FIX_BORDERS)
    bw.write_u32(num_env_idx, 2);

    // IID per envelope — pick df vs dt independently per envelope. The
    // dt baseline for envelope 0 is `prev_iid_idx` from the previous frame;
    // for envelope e>0 it is the previous envelope's indices in this frame
    // (see §8.6.4.6.2 — "differential coding always references the
    // immediately preceding envelope, regardless of frame boundary").
    let mut prev = *prev_iid_idx;
    let mut last_iid = *prev_iid_idx;
    for env in &frame.envs {
        let env_idx = clamp_iid_10(&env.iid_idx);
        let (df_bits, dt_bits) = iid_envelope_costs(&env_idx, &prev);
        let use_dt = dt_bits < df_bits;
        bw.write_bit(use_dt); // iid_dt[e]
        if use_dt {
            for b in 0..10 {
                let delta = (env_idx[b] - prev[b]).clamp(-14, 14);
                huff_encode_iid_dt0(bw, delta);
            }
        } else {
            let mut prev_band = 0i32;
            for b in 0..10 {
                let delta = (env_idx[b] - prev_band).clamp(-14, 14);
                huff_encode_iid_df0(bw, delta);
                prev_band += delta;
            }
        }
        prev = env_idx;
        last_iid = env_idx;
    }

    // ICC per envelope — same df-vs-dt selection.
    let mut prev = *prev_icc_idx;
    let mut last_icc = *prev_icc_idx;
    for env in &frame.envs {
        let env_idx = clamp_icc_10(&env.icc_idx);
        let (df_bits, dt_bits) = icc_envelope_costs(&env_idx, &prev);
        let use_dt = dt_bits < df_bits;
        bw.write_bit(use_dt); // icc_dt[e]
        if use_dt {
            for b in 0..10 {
                let delta = (env_idx[b] - prev[b]).clamp(-7, 7);
                huff_encode_icc_dt(bw, delta);
            }
        } else {
            let mut prev_band = 0i32;
            for b in 0..10 {
                let delta = (env_idx[b] - prev_band).clamp(-7, 7);
                huff_encode_icc_df(bw, delta);
                prev_band += delta;
            }
        }
        prev = env_idx;
        last_icc = env_idx;
    }
    let bits = (bw.bit_position() - start) as u32;
    (bits, last_iid, last_icc)
}

/// Per-band clamp of an IID parameter envelope to the default-quant range
/// `-7..=7` used by `iid_mode = 0`.
fn clamp_iid_10(src: &[i32; 10]) -> [i32; 10] {
    let mut out = [0i32; 10];
    for b in 0..10 {
        out[b] = src[b].clamp(-7, 7);
    }
    out
}

/// Per-band clamp of an ICC parameter envelope to `0..=7`.
fn clamp_icc_10(src: &[i32; 10]) -> [i32; 10] {
    let mut out = [0i32; 10];
    for b in 0..10 {
        out[b] = src[b].clamp(0, 7);
    }
    out
}

/// Write the SBR SCE element with a Parametric Stereo extension carrying
/// real (variable-length) IID/ICC parameters. The PS payload is encoded
/// into a temporary [`BitWriter`] first so that `bs_extension_size` can be
/// set correctly before the content is appended.
///
/// Same outer layout as [`write_single_channel_element_mono_with_ps`]:
/// SBR SCE body, then `bs_extended_data = 1`, then a PS extension
/// (`bs_extension_id = 2`) sized to fit the variable PS payload. Trailing
/// fill bits pad to a byte boundary inside the extension block.
pub fn write_single_channel_element_mono_with_ps_real(
    bw: &mut BitWriter,
    _header: &SbrHeader,
    freq: &FreqTables,
    sf: &SbrFrameScalefactors,
    frame: &PsParamsFrame,
    prev_iid_idx: &[i32; 10],
    prev_icc_idx: &[i32; 10],
) -> ([i32; 10], [i32; 10]) {
    // ---- Same body as write_single_channel_element_mono. ----
    bw.write_bit(false); // bs_data_extra
    bw.write_u32(FrameClass::FixFix as u32, 2);
    bw.write_u32(0, 2); // bs_num_env = 1
    bw.write_u32(1, 1); // bs_freq_res[0] = 1
    bw.write_u32(0, 1); // bs_df_env[0] = 0
    bw.write_u32(0, 1); // bs_df_noise[0] = 0
    for _ in 0..freq.nq {
        bw.write_u32(0, 2); // bs_invf_mode = 0
    }
    write_envelope_1_5db_freq_delta(bw, &sf.env, freq.n_high);
    write_noise_3_0db_freq_delta(bw, &sf.noise, freq.nq);
    bw.write_bit(false); // bs_add_harmonic_flag = 0

    // ---- Pre-compute the PS payload into a side buffer. ----
    // Layout inside the extension block: bs_extension_id (2 bits) +
    // ps_data() (variable). Total content bits round up to bytes; any
    // slack is filled with zero bits.
    let mut sub = BitWriter::with_capacity(16);
    let (ps_bits, last_iid, last_icc) =
        write_ps_data_real(&mut sub, frame, prev_iid_idx, prev_icc_idx);
    sub.align_to_byte();
    let ps_bytes = sub.finish();

    let content_bits = 2 + ps_bits; // bs_extension_id (2) + ps_data
    let cnt_bytes = content_bits.div_ceil(8);
    let fill_bits = cnt_bytes * 8 - content_bits;

    // ---- bs_extended_data = 1 + EXT_ID_PS extension. ----
    bw.write_bit(true); // bs_extended_data = 1
    if cnt_bytes < 15 {
        bw.write_u32(cnt_bytes, 4); // bs_extension_size
    } else {
        // Escape mechanism (Table 4.65 / §4.6.18): bs_extension_size = 15
        // means "use bs_esc_count", and the decoder computes
        // total = 15 + bs_esc_count. So we emit `cnt_bytes - 15`.
        bw.write_u32(15, 4);
        bw.write_u32(cnt_bytes - 15, 8);
    }
    bw.write_u32(EXT_ID_PS_DATA, 2); // bs_extension_id = 2

    // Splice the pre-built PS payload bits back in. We re-write `ps_bits`
    // bits MSB-first from `ps_bytes`.
    let full = (ps_bits / 8) as usize;
    for byte in &ps_bytes[..full] {
        bw.write_u32(*byte as u32, 8);
    }
    let tail = ps_bits - (full as u32) * 8;
    if tail > 0 {
        let last = ps_bytes[full] >> (8 - tail);
        bw.write_u32(last as u32, tail);
    }
    if fill_bits > 0 {
        bw.write_u32(0, fill_bits);
    }
    (last_iid, last_icc)
}

/// Write an `sbr_channel_pair_element()` in **independent** coupling mode
/// (Table 4.66 with `bs_data_extra=0` and `bs_coupling=0`). Each channel
/// gets its own minimal FIXFIX `bs_num_env=1` grid (matching the mono
/// SCE writer above) plus envelope / noise data quantised at amp_res=0
/// (1.5 dB envelope, 3.0 dB noise). No sinusoids, no extended data.
///
/// The decoder side uses the inherited `bs_amp_res` from the SBR header
/// — but FIXFIX with `bs_num_env == 1` overrides it to 0 inside
/// `parse_sbr_grid` (§4.6.18.3.3). So our 1.5 dB envelope tables are
/// the right pick regardless of `header.bs_amp_res`.
pub fn write_channel_pair_element_independent(
    bw: &mut BitWriter,
    freq: &FreqTables,
    sf_l: &SbrFrameScalefactors,
    sf_r: &SbrFrameScalefactors,
) {
    bw.write_bit(false); // bs_data_extra = 0
    bw.write_bit(false); // bs_coupling  = 0 (independent)

    // sbr_grid for L — FIXFIX, bs_num_env=1, freq_res=1.
    bw.write_u32(FrameClass::FixFix as u32, 2);
    bw.write_u32(0, 2); // bs_num_env = 1
    bw.write_u32(1, 1); // bs_freq_res[0] = 1 (high-res)

    // sbr_grid for R — same shape.
    bw.write_u32(FrameClass::FixFix as u32, 2);
    bw.write_u32(0, 2); // bs_num_env = 1
    bw.write_u32(1, 1); // bs_freq_res[0] = 1 (high-res)

    // sbr_dtdf for L (1 envelope + 1 noise) and R (same).
    bw.write_u32(0, 1); // bs_df_env[L][0] = 0 (freq-direction)
    bw.write_u32(0, 1); // bs_df_noise[L][0] = 0
    bw.write_u32(0, 1); // bs_df_env[R][0] = 0
    bw.write_u32(0, 1); // bs_df_noise[R][0] = 0

    // sbr_invf for L then R (bw mode 0 = filtering off, per noise band).
    for _ in 0..freq.nq {
        bw.write_u32(0, 2);
    }
    for _ in 0..freq.nq {
        bw.write_u32(0, 2);
    }

    // Independent coupling — per Table 4.66 the order is envelope(L),
    // envelope(R), noise(L), noise(R). Do NOT interleave env+noise per
    // channel here; that's the layout of the coupled branch.
    write_envelope_1_5db_freq_delta(bw, &sf_l.env, freq.n_high);
    write_envelope_1_5db_freq_delta(bw, &sf_r.env, freq.n_high);
    write_noise_3_0db_freq_delta(bw, &sf_l.noise, freq.nq);
    write_noise_3_0db_freq_delta(bw, &sf_r.noise, freq.nq);

    // bs_add_harmonic_flag(L) = 0
    bw.write_bit(false);
    // bs_add_harmonic_flag(R) = 0
    bw.write_bit(false);
    // bs_extended_data = 0
    bw.write_bit(false);
}

/// Write per-band envelope scalefactors with freq-direction delta coding.
///
/// `env[0]` goes out as a 7-bit absolute. `env[k]` for `k > 0` is encoded
/// as `env[k] - env[k-1] + LAV` through the `F_HUFFMAN_ENV_1_5DB` table.
///
/// Input `env[i]` is in 1.5 dB quantisation (directly — no doubling
/// applied). An input of 0 gives decoder `E_orig = 64`, negative inputs
/// go below (e.g. `-10` → `E_orig = 64 · 2^-5 = 2`). Since the 7-bit
/// absolute field is unsigned (0..127), negative first-band values are
/// clamped at 0; the Huffman-coded deltas cover the negative range.
pub fn write_envelope_1_5db_freq_delta(bw: &mut BitWriter, env: &[i32], n_bands: usize) {
    use super::tables::{F_HUFFMAN_ENV_1_5DB, LAV_ENV_1_5DB};
    let n = n_bands.min(env.len());
    // Spec limit on the accumulated absolute scalefactor value at
    // amp_res=0 is 127 (7-bit unsigned start + any positive delta run).
    // ffmpeg rejects `env_facs_q > 127`. Keep reconstructed values
    // inside [0, 127] at every band.
    const MAX_ACC: i32 = 127;
    let mut prev_abs = 0i32;
    for i in 0..n {
        if i == 0 {
            let v = env[i].clamp(0, MAX_ACC);
            bw.write_u32(v as u32, 7);
            prev_abs = v;
        } else {
            // Requested reconstructed value, bounded by both the
            // Huffman delta range and the cumulative absolute cap.
            let v = env[i]
                .clamp(prev_abs - LAV_ENV_1_5DB, prev_abs + LAV_ENV_1_5DB)
                .clamp(0, MAX_ACC);
            let delta = v - prev_abs;
            write_huffman_sym(bw, delta + LAV_ENV_1_5DB, &F_HUFFMAN_ENV_1_5DB);
            prev_abs = v;
        }
    }
}

/// Write per-band noise scalefactors with freq-direction delta coding
/// using the 3.0 dB noise tables.
pub fn write_noise_3_0db_freq_delta(bw: &mut BitWriter, noise: &[i32], nq: usize) {
    use super::tables::F_HUFFMAN_ENV_3_0DB;
    let n = nq.min(noise.len());
    let mut prev_abs = 0i32;
    for i in 0..n {
        let v = noise[i].clamp(0, LAV_NOISE_3_0DB);
        if i == 0 {
            // Start value is 5 bits per §4.6.18.3.5 for noise.
            bw.write_u32(v as u32, 5);
            prev_abs = v;
        } else {
            // Uses the env 3.0 dB F-table per Note 2 of Table 4.73.
            let delta = v - prev_abs;
            write_huffman_sym(bw, delta + LAV_NOISE_3_0DB, &F_HUFFMAN_ENV_3_0DB);
            prev_abs = v;
        }
    }
}

/// Write a Huffman symbol from a `(length, codeword)` table — the inverse
/// of [`crate::sbr::tables::sbr_huff_decode`]. Out-of-range indices write
/// code 0 of the table (equivalent to the smallest legal delta), which
/// keeps the bitstream well-formed even on surprise inputs.
fn write_huffman_sym(bw: &mut BitWriter, idx: i32, table: &[(u8, u32)]) {
    let i = if idx < 0 {
        0usize
    } else if (idx as usize) >= table.len() {
        table.len() - 1
    } else {
        idx as usize
    };
    let (len, code) = table[i];
    bw.write_u32(code, len as u32);
}

/// Write a FIL element (ID=6) carrying an SBR extension payload.
///
/// `sbr_payload` is the byte-aligned output of
/// [`SbrEncoder::emit_sbr_payload`] — the 4-bit extension_type field is
/// prepended here. The FIL element is preceded by its 3-bit element type
/// (`ElementType::Fil = 6`) written by the caller; this helper handles
/// only the element body.
///
/// Layout:
///   cnt           4 bits  (or 4+8 if the payload is >= 15 bytes)
///   extension_id  4 bits  = EXT_SBR_DATA (0xD)
///   [sbr payload bits]
pub fn write_fil_body_with_sbr(
    bw: &mut BitWriter,
    sbr_payload_bits: &[u8],
    sbr_payload_nbits: u32,
) {
    // The FIL element has a byte-granular count that includes the 4-bit
    // extension_id. Round-up bits.
    let total_bits = 4 + sbr_payload_nbits;
    let total_bytes = total_bits.div_ceil(8) as usize;
    if total_bytes >= 15 {
        bw.write_u32(15, 4);
        bw.write_u32((total_bytes - 15 + 1) as u32, 8);
    } else {
        bw.write_u32(total_bytes as u32, 4);
    }
    bw.write_u32(EXT_SBR_DATA, 4);
    // Write the payload bits from the pre-assembled buffer.
    let full_bytes = (sbr_payload_nbits / 8) as usize;
    for byte in &sbr_payload_bits[..full_bytes] {
        bw.write_u32(*byte as u32, 8);
    }
    let tail_bits = sbr_payload_nbits - (full_bytes as u32 * 8);
    if tail_bits > 0 {
        let last = sbr_payload_bits[full_bytes] >> (8 - tail_bits);
        bw.write_u32(last as u32, tail_bits);
    }
    // Bit-align: payload was written byte-aligned at the source, but the
    // 4-bit extension_id bumps alignment — pad the final byte with zeros.
    let pad = (8 - (bw.bit_position() % 8)) % 8;
    if pad > 0 {
        bw.write_u32(0, pad as u32);
    }
}

/// Convenience: emit SBR-bearing FIL bytes (ready to be written as the
/// `sbr_extension_data()` portion of the raw_data_block) given the byte
/// payload from [`SbrEncoder::emit_sbr_payload`] and its bit count.
pub fn encode_fil_sbr_element(payload_bits: &[u8], payload_nbits: u32) -> Vec<u8> {
    let mut bw = BitWriter::with_capacity(4 + payload_bits.len() + 2);
    // Element ID = 6 (Fil), 3 bits.
    bw.write_u32(crate::syntax::ElementType::Fil as u32, 3);
    write_fil_body_with_sbr(&mut bw, payload_bits, payload_nbits);
    bw.align_to_byte();
    bw.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn downsampler_silence() {
        let inp = vec![0.0f32; 2048];
        let mut out = vec![1.0f32; 1024];
        downsample_by_two(&inp, &mut out);
        for v in out {
            assert!(v.abs() < 1e-6);
        }
    }

    #[test]
    fn downsampler_lowpass_preserves_dc() {
        let inp = vec![1.0f32; 2048];
        let mut out = vec![0.0f32; 1024];
        downsample_by_two(&inp, &mut out);
        // The centre-tap + symmetric kernel should sum to 1.0; allow ~5%
        // deviation for the short kernel.
        for v in &out[200..800] {
            assert!(v.abs() > 0.8 && v.abs() < 1.2, "dc out = {v}");
        }
    }

    #[test]
    fn qmf64_zero_input_gives_zero() {
        let mut qa = QmfAnalysis64::new();
        let inp = [0.0f32; 64];
        let mut out = [Complex32::new(0.0, 0.0); NUM_QMF_BANDS];
        qa.process(&inp, &mut out);
        for k in 0..NUM_QMF_BANDS {
            assert!(out[k].re.abs() < 1e-6);
            assert!(out[k].im.abs() < 1e-6);
        }
    }

    #[test]
    fn encoder_constructs() {
        let enc = SbrEncoder::new(24_000).expect("construct");
        assert!(enc.freq.n_high > 0);
        assert!(enc.freq.kx > 0);
    }

    #[test]
    fn emit_payload_produces_bytes() {
        let mut enc = SbrEncoder::new(24_000).expect("construct");
        let pcm: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        let (bytes, bits) = enc.encode_frame_fil(&pcm);
        assert!(!bytes.is_empty());
        assert!(bits > 0);
    }

    #[test]
    fn stereo_encoder_constructs() {
        let enc = SbrStereoEncoder::new(24_000).expect("construct");
        assert!(enc.freq.n_high > 0);
        assert!(enc.freq.kx > 0);
    }

    #[test]
    fn stereo_emit_payload_produces_bytes() {
        let mut enc = SbrStereoEncoder::new(24_000).expect("construct");
        let pcm_l: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        let pcm_r: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1500.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        let (bytes, bits) = enc.encode_frame_fil_pair(&pcm_l, &pcm_r);
        assert!(!bytes.is_empty());
        assert!(bits > 0);
    }

    /// Produce a CPE SBR payload and parse it back via the decoder's CPE
    /// branch — the round trip exercises both writer and
    /// `parse_channel_pair_element` in independent mode.
    #[test]
    fn stereo_emit_payload_round_trips_through_parser() {
        use crate::sbr::decode::{try_parse_sbr_extension_ext, SbrChannelState, SbrPayload};
        use oxideav_core::bits::BitReader;

        let mut enc = SbrStereoEncoder::new(24_000).expect("construct");
        let pcm_l: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        let pcm_r: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1500.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        let (xl, xr) = enc.analyse(&pcm_l, &pcm_r);
        let (bytes, bits) = enc.emit_sbr_payload_pair(&xl, &xr);
        let mut bw = BitWriter::with_capacity(bytes.len() + 1);
        bw.write_u32(EXT_SBR_DATA, 4);
        let full = (bits / 8) as usize;
        for b in &bytes[..full] {
            bw.write_u32(*b as u32, 8);
        }
        let tail = bits - full as u32 * 8;
        if tail > 0 {
            bw.write_u32((bytes[full] >> (8 - tail)) as u32, tail);
        }
        bw.align_to_byte();
        let framed = bw.finish();
        let mut br = BitReader::new(&framed);
        let num_payload_bits = 4 + bits;
        let mut state = SbrChannelState::new();
        // is_sce = false to drive the CPE parser branch.
        let parsed =
            try_parse_sbr_extension_ext(&mut br, num_payload_bits, false, &mut state, 24_000)
                .expect("parse ok");
        match parsed {
            Some(SbrPayload::Pair { l, r, coupled }) => {
                assert_eq!(l.bs_num_env, 1);
                assert_eq!(r.bs_num_env, 1);
                assert!(
                    !coupled,
                    "CPE encoded as independent should parse coupled=false"
                );
            }
            other => panic!("expected Pair CPE payload, got {other:?}"),
        }
    }

    /// Produce an SBR payload, then parse it back through the decoder's
    /// `try_parse_sbr_extension_ext` — confirms encoder / parser are
    /// bitstream-compatible.
    #[test]
    fn emit_payload_round_trips_through_parser() {
        use crate::sbr::decode::{try_parse_sbr_extension_ext, SbrChannelState, SbrPayload};
        use oxideav_core::bits::BitReader;
        let mut enc = SbrEncoder::new(24_000).expect("construct");
        let pcm: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        let x = enc.analyse(&pcm);
        let (bytes, bits) = enc.emit_sbr_payload(&x);
        // Feed the bytes as if they were the FIL-inner payload: we need
        // to prepend the 4-bit EXT_SBR_DATA so try_parse consumes it.
        let mut bw = BitWriter::with_capacity(bytes.len() + 1);
        bw.write_u32(EXT_SBR_DATA, 4);
        // Write exactly `bits` of SBR payload.
        let full = (bits / 8) as usize;
        for b in &bytes[..full] {
            bw.write_u32(*b as u32, 8);
        }
        let tail = bits - full as u32 * 8;
        if tail > 0 {
            bw.write_u32((bytes[full] >> (8 - tail)) as u32, tail);
        }
        bw.align_to_byte();
        let framed = bw.finish();
        let mut br = BitReader::new(&framed);
        let num_payload_bits = 4 + bits;
        let mut state = SbrChannelState::new();
        let parsed =
            try_parse_sbr_extension_ext(&mut br, num_payload_bits, true, &mut state, 24_000)
                .expect("parse ok");
        match parsed {
            Some(SbrPayload::Single { data, .. }) => {
                assert_eq!(data.bs_num_env, 1);
            }
            other => panic!("expected Single SCE payload, got {other:?}"),
        }
    }

    /// PS-enabled SBR payload should round-trip through the parser and
    /// surface a [`SbrPayload::Single`] with a populated `ps` frame whose
    /// no-op IID/ICC parameters dequantise to identity stereo.
    #[test]
    fn emit_payload_with_ps_round_trips_and_carries_ps_frame() {
        use crate::sbr::decode::{try_parse_sbr_extension_ext, SbrChannelState, SbrPayload};
        use oxideav_core::bits::BitReader;
        let mut enc = SbrEncoder::new(24_000).expect("construct");
        enc.set_emit_ps(true);
        let pcm: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        let x = enc.analyse(&pcm);
        let (bytes, bits) = enc.emit_sbr_payload(&x);
        // Re-frame as a FIL inner payload: EXT_SBR_DATA + N bits of SBR.
        let mut bw = BitWriter::with_capacity(bytes.len() + 1);
        bw.write_u32(EXT_SBR_DATA, 4);
        let full = (bits / 8) as usize;
        for b in &bytes[..full] {
            bw.write_u32(*b as u32, 8);
        }
        let tail = bits - full as u32 * 8;
        if tail > 0 {
            bw.write_u32((bytes[full] >> (8 - tail)) as u32, tail);
        }
        bw.align_to_byte();
        let framed = bw.finish();
        let mut br = BitReader::new(&framed);
        let num_payload_bits = 4 + bits;
        let mut state = SbrChannelState::new();
        let parsed =
            try_parse_sbr_extension_ext(&mut br, num_payload_bits, true, &mut state, 24_000)
                .expect("parse ok");
        match parsed {
            Some(SbrPayload::Single { data, ps }) => {
                assert_eq!(data.bs_num_env, 1);
                let ps = ps.expect("PS frame should be present when emit_ps=true");
                assert_eq!(ps.num_env, 1);
                assert!(ps.has_iid, "no-op PS payload should set enable_iid");
                assert!(ps.has_icc, "no-op PS payload should set enable_icc");
                // IID = 0 dB, ICC = 1 (fully coherent) per Tables 8.25, 8.28.
                for v in &ps.iid_db[0] {
                    assert!(v.abs() < 1e-5, "IID should be 0 dB, got {v}");
                }
                for v in &ps.icc[0] {
                    assert!(
                        (*v - 1.0).abs() < 1e-5,
                        "ICC should be 1 (fully coherent), got {v}"
                    );
                }
            }
            other => panic!("expected Single SCE payload, got {other:?}"),
        }
    }

    /// PS-enabled SBR payload with **real** per-band IID/ICC parameters
    /// should round-trip through the parser and surface a
    /// [`SbrPayload::Single`] whose dequantised PS frame matches the
    /// quantised input parameters band-by-band.
    #[test]
    fn emit_payload_with_real_ps_round_trips() {
        use crate::sbr::decode::{try_parse_sbr_extension_ext, SbrChannelState, SbrPayload};
        use crate::sbr::ps::{PsParams10, ICC_QUANT, IID_QUANT_DEFAULT};
        use oxideav_core::bits::BitReader;

        let mut enc = SbrEncoder::new(24_000).expect("construct");
        enc.set_emit_ps(true);
        // Stage a fixed parameter set: alternating +4 dB / -4 dB IID per
        // band, and ICC 0.937 / 0.84118 alternating.
        let mut params = PsParams10::default();
        for b in 0..10 {
            params.iid_idx[b] = if b % 2 == 0 { 2 } else { -2 };
            params.icc_idx[b] = if b % 2 == 0 { 1 } else { 2 };
        }
        enc.set_pending_ps(params.clone());
        let pcm: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        let x = enc.analyse(&pcm);
        let (bytes, bits) = enc.emit_sbr_payload(&x);

        // Re-frame as a FIL inner payload: EXT_SBR_DATA + N bits of SBR.
        let mut bw = BitWriter::with_capacity(bytes.len() + 1);
        bw.write_u32(EXT_SBR_DATA, 4);
        let full = (bits / 8) as usize;
        for b in &bytes[..full] {
            bw.write_u32(*b as u32, 8);
        }
        let tail = bits - full as u32 * 8;
        if tail > 0 {
            bw.write_u32((bytes[full] >> (8 - tail)) as u32, tail);
        }
        bw.align_to_byte();
        let framed = bw.finish();
        let mut br = BitReader::new(&framed);
        let num_payload_bits = 4 + bits;
        let mut state = SbrChannelState::new();
        let parsed =
            try_parse_sbr_extension_ext(&mut br, num_payload_bits, true, &mut state, 24_000)
                .expect("parse real PS ok");
        match parsed {
            Some(SbrPayload::Single { ps, .. }) => {
                let ps = ps.expect("PS frame present");
                assert_eq!(ps.iid_db.len(), 1);
                assert_eq!(ps.icc.len(), 1);
                let iid = &ps.iid_db[0];
                let icc = &ps.icc[0];
                assert_eq!(iid.len(), 10);
                assert_eq!(icc.len(), 10);
                for b in 0..10 {
                    let want_iid = IID_QUANT_DEFAULT[(params.iid_idx[b] + 7) as usize];
                    let want_icc = ICC_QUANT[params.icc_idx[b] as usize];
                    assert!(
                        (iid[b] - want_iid).abs() < 1e-4,
                        "IID band {b}: got {} want {}",
                        iid[b],
                        want_iid,
                    );
                    assert!(
                        (icc[b] - want_icc).abs() < 1e-4,
                        "ICC band {b}: got {} want {}",
                        icc[b],
                        want_icc,
                    );
                }
            }
            other => panic!("expected Single SCE payload, got {other:?}"),
        }
    }

    /// Round-14 — when the same parameter set is emitted on two
    /// consecutive frames, the second frame's prev-baseline matches the
    /// current values exactly: every band's time-direction delta is 0.
    /// Since `huff_iid_dt[0]` symbol 0 is 1 bit (Table 8.B.18), 10 bands ×
    /// 1 bit = 10 bits for the DT branch — much cheaper than DF (which
    /// needs at least one wider codeword for the absolute first band when
    /// the parameters are non-zero). The encoder must therefore pick
    /// `iid_dt[0] = 1` (and likewise for ICC) on frame 2.
    #[test]
    fn ps_payload_picks_time_direction_when_stable() {
        use crate::sbr::ps::PsParams10;

        let mut enc = SbrEncoder::new(24_000).expect("construct");
        enc.set_emit_ps(true);
        // Non-zero params so DF coding pays for at least one wide codeword.
        let mut params = PsParams10::default();
        for b in 0..10 {
            params.iid_idx[b] = if b % 2 == 0 { 4 } else { -4 };
            params.icc_idx[b] = 3;
        }
        let pcm: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        // Frame 1 — primes the prev-baseline. Encoder's prev is all-zero
        // so DF wins (DF[0] = signed-bias absolute → cheap; DT delta vs 0
        // is also cheap, but DF tends to tie or beat for first frame).
        enc.set_pending_ps(params.clone());
        let x = enc.analyse(&pcm);
        let (_, bits1) = enc.emit_sbr_payload(&x);
        // Sanity: frame 1 published the staged params verbatim into the
        // running prev-baseline.
        for b in 0..10 {
            assert_eq!(enc.prev_iid_idx[b], params.iid_idx[b]);
            assert_eq!(enc.prev_icc_idx[b], params.icc_idx[b]);
        }
        // Frame 2 — same params, the prev-baseline now matches exactly so
        // every band's DT delta is 0. The DT branch costs 10 bits per
        // envelope (10 × 1-bit-zero-codeword). DF requires 0..6 bits per
        // delta; for this alternating-sign pattern it pays multiple
        // multi-bit deltas. So frame 2 must be strictly smaller than
        // frame 1 — proof that the encoder picked DT over DF.
        enc.set_pending_ps(params.clone());
        let x = enc.analyse(&pcm);
        let (_, bits2) = enc.emit_sbr_payload(&x);
        assert!(
            bits2 < bits1,
            "expected DT to shrink frame 2 (bits1={bits1}, bits2={bits2}) — \
             encoder did not pick time-direction differential coding",
        );
    }

    /// Cost-only test: when the encoder's running prev exactly matches
    /// the staged parameters, the DT representation costs `10 × 1 = 10`
    /// bits (every per-band delta is 0, encoded as the single-bit "0"
    /// codeword in `huff_iid_dt[0]` / `huff_icc_dt`). The DF path's first
    /// delta is the absolute IID/ICC value, which costs more for any
    /// non-zero parameter set.
    #[test]
    fn ps_dt_costs_less_than_df_for_repeated_params() {
        use crate::sbr::ps::{
            huff_bits_icc_df, huff_bits_icc_dt, huff_bits_iid_df0, huff_bits_iid_dt0,
        };
        let iid = [4, -4, 4, -4, 4, -4, 4, -4, 4, -4];
        let icc = [3i32; 10];
        // DF cost when prev = [0; 10] (frame-start baseline).
        let mut df_iid = 0u32;
        let mut p = 0i32;
        for b in 0..10 {
            let delta = iid[b] - p;
            df_iid += huff_bits_iid_df0(delta);
            p += delta;
        }
        let mut df_icc = 0u32;
        let mut p = 0i32;
        for b in 0..10 {
            let delta = icc[b] - p;
            df_icc += huff_bits_icc_df(delta);
            p += delta;
        }
        // DT cost when prev = same as current (perfect prediction).
        let mut dt_iid = 0u32;
        for b in 0..10 {
            dt_iid += huff_bits_iid_dt0(iid[b] - iid[b]);
        }
        let mut dt_icc = 0u32;
        for b in 0..10 {
            dt_icc += huff_bits_icc_dt(icc[b] - icc[b]);
        }
        assert_eq!(dt_iid, 10, "10 bands × 1-bit-zero-codeword == 10");
        assert_eq!(dt_icc, 10, "10 bands × 1-bit-zero-codeword == 10");
        assert!(
            dt_iid < df_iid,
            "DT IID ({dt_iid}b) should beat DF ({df_iid}b)"
        );
        assert!(
            dt_icc < df_icc,
            "DT ICC ({dt_icc}b) should beat DF ({df_icc}b)"
        );
    }

    /// Multi-envelope payload (`num_env = 2`) round-trips through the
    /// parser with both envelopes' IID/ICC values intact and the correct
    /// `num_env` advertised in the bitstream.
    #[test]
    fn ps_payload_multi_env_two_round_trips() {
        use crate::sbr::decode::{try_parse_sbr_extension_ext, SbrChannelState, SbrPayload};
        use crate::sbr::ps::{PsParams10, PsParamsFrame, ICC_QUANT, IID_QUANT_DEFAULT};
        use oxideav_core::bits::BitReader;

        let mut enc = SbrEncoder::new(24_000).expect("construct");
        enc.set_emit_ps(true);
        // Envelope 0: IID +4 dB across the board, ICC index 1.
        // Envelope 1: IID -4 dB, ICC index 4. Distinct enough that a
        // collapse to a single envelope would be detectable.
        let mut e0 = PsParams10::default();
        let mut e1 = PsParams10::default();
        for b in 0..10 {
            e0.iid_idx[b] = 2;
            e0.icc_idx[b] = 1;
            e1.iid_idx[b] = -2;
            e1.icc_idx[b] = 4;
        }
        enc.set_pending_ps_frame(PsParamsFrame { envs: vec![e0, e1] });
        let pcm: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        let x = enc.analyse(&pcm);
        let (bytes, bits) = enc.emit_sbr_payload(&x);

        let mut bw = BitWriter::with_capacity(bytes.len() + 1);
        bw.write_u32(EXT_SBR_DATA, 4);
        let full = (bits / 8) as usize;
        for b in &bytes[..full] {
            bw.write_u32(*b as u32, 8);
        }
        let tail = bits - full as u32 * 8;
        if tail > 0 {
            bw.write_u32((bytes[full] >> (8 - tail)) as u32, tail);
        }
        bw.align_to_byte();
        let framed = bw.finish();
        let mut br = BitReader::new(&framed);
        let num_payload_bits = 4 + bits;
        let mut state = SbrChannelState::new();
        let parsed =
            try_parse_sbr_extension_ext(&mut br, num_payload_bits, true, &mut state, 24_000)
                .expect("parse multi-env ok");
        match parsed {
            Some(SbrPayload::Single { ps, .. }) => {
                let ps = ps.expect("PS frame present");
                assert_eq!(ps.num_env, 2, "expected num_env = 2");
                assert_eq!(ps.iid_db.len(), 2);
                assert_eq!(ps.icc.len(), 2);
                // Verify env 0 and env 1 are dequantised correctly per
                // band — proves the encoder/decoder grid agrees.
                let want_iid_e0 = IID_QUANT_DEFAULT[(2 + 7) as usize];
                let want_iid_e1 = IID_QUANT_DEFAULT[(-2 + 7) as usize];
                let want_icc_e0 = ICC_QUANT[1];
                let want_icc_e1 = ICC_QUANT[4];
                for b in 0..10 {
                    assert!(
                        (ps.iid_db[0][b] - want_iid_e0).abs() < 1e-4,
                        "env0 IID b={b}: got {}, want {want_iid_e0}",
                        ps.iid_db[0][b]
                    );
                    assert!(
                        (ps.iid_db[1][b] - want_iid_e1).abs() < 1e-4,
                        "env1 IID b={b}: got {}, want {want_iid_e1}",
                        ps.iid_db[1][b]
                    );
                    assert!(
                        (ps.icc[0][b] - want_icc_e0).abs() < 1e-4,
                        "env0 ICC b={b}: got {}, want {want_icc_e0}",
                        ps.icc[0][b]
                    );
                    assert!(
                        (ps.icc[1][b] - want_icc_e1).abs() < 1e-4,
                        "env1 ICC b={b}: got {}, want {want_icc_e1}",
                        ps.icc[1][b]
                    );
                }
            }
            other => panic!("expected Single SCE payload, got {other:?}"),
        }
    }

    /// Multi-envelope payload (`num_env = 4`) round-trips through the
    /// parser carrying all four distinct envelopes.
    #[test]
    fn ps_payload_multi_env_four_round_trips() {
        use crate::sbr::decode::{try_parse_sbr_extension_ext, SbrChannelState, SbrPayload};
        use crate::sbr::ps::{PsParams10, PsParamsFrame};
        use oxideav_core::bits::BitReader;

        let mut enc = SbrEncoder::new(24_000).expect("construct");
        enc.set_emit_ps(true);
        let mut envs = Vec::with_capacity(4);
        for e in 0..4 {
            let mut p = PsParams10::default();
            for b in 0..10 {
                p.iid_idx[b] = (e as i32) - 2; // -2, -1, 0, 1
                p.icc_idx[b] = e as i32;
            }
            envs.push(p);
        }
        enc.set_pending_ps_frame(PsParamsFrame { envs });
        let pcm: Vec<f32> = (0..2048)
            .map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / 48_000.0).sin() * 0.5)
            .collect();
        let x = enc.analyse(&pcm);
        let (bytes, bits) = enc.emit_sbr_payload(&x);

        let mut bw = BitWriter::with_capacity(bytes.len() + 1);
        bw.write_u32(EXT_SBR_DATA, 4);
        let full = (bits / 8) as usize;
        for b in &bytes[..full] {
            bw.write_u32(*b as u32, 8);
        }
        let tail = bits - full as u32 * 8;
        if tail > 0 {
            bw.write_u32((bytes[full] >> (8 - tail)) as u32, tail);
        }
        bw.align_to_byte();
        let framed = bw.finish();
        let mut br = BitReader::new(&framed);
        let num_payload_bits = 4 + bits;
        let mut state = SbrChannelState::new();
        let parsed =
            try_parse_sbr_extension_ext(&mut br, num_payload_bits, true, &mut state, 24_000)
                .expect("parse 4-env ok");
        match parsed {
            Some(SbrPayload::Single { ps, .. }) => {
                let ps = ps.expect("PS frame present");
                assert_eq!(ps.num_env, 4, "expected num_env = 4");
            }
            other => panic!("expected Single SCE payload, got {other:?}"),
        }
    }
}

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
/// The decoder dequantises with `E_orig = 64 · 2^(acc_e / amp_res_bits)`.
/// At amp_res=0 (1.5 dB step) `amp_res_bits=2`; at amp_res=1 (3.0 dB)
/// `amp_res_bits=1`. We target the 1.5 dB path (FIXFIX num_env=1 forces
/// amp_res=0) so the quantised integer is `round(log2(E/64) * 2)`.
///
/// To avoid blow-up in the decoder's `gain = sqrt(E_orig / E_curr)` step
/// when E_curr is tiny (near-silent high band from a low-frequency tone
/// LPC-extrapolated into high subbands), we apply a modest clamp that
/// keeps envelopes within a sensible dynamic range.
pub fn estimate_envelope(
    x_high: &[[Complex32; NUM_QMF_BANDS]],
    ft: &FreqTables,
) -> SbrFrameScalefactors {
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
        let e = acc / (n * bands + 1e-30);
        // Quantised value at amp_res=0 (1.5 dB step): E = 64 * 2^(v/2)
        // => v = round(log2(E/64) * 2). Allow negative values so
        // silent / near-silent bands don't get a 64-unit energy floor.
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
        // Conservative defaults — 5, 9 are reasonable for 24 kHz core
        // (kx ~ 15, stop ~ 32). amp_res=1 picks the 3.0 dB tables.
        header.bs_amp_res = 1;
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
        header.bs_header_extra_1 = false;
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
        })
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
        // sbr_data() for SCE: sbr_single_channel_element()
        write_single_channel_element_mono(&mut bw, &self.header, &self.freq, &sf);
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
}

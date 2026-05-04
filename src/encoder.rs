//! AAC-LC CBR encoder — ISO/IEC 14496-3 §4.5.2.1.
//!
//! This is a baseline long-only encoder. It mirrors the inverse structure
//! already present in `imdct.rs` / `synth.rs`:
//!
//! 1. Buffer input PCM, keeping a 1024-sample overlap window from the
//!    previous frame.
//! 2. Apply the same sine window used on decode (`window::sine_long`).
//! 3. Run forward MDCT (`mdct::mdct_long`) to produce 1024 spectral
//!    coefficients.
//! 4. For CPE (stereo) channels, consider M/S stereo per scalefactor band
//!    — choose whichever of (L,R) vs (M,S) costs fewer bits.
//! 5. Flat-quantise per scalefactor band: pick one global scalefactor per
//!    band so the largest quantised magnitude stays in the usable range
//!    for codebook 11 (the escape book, LAV=16 plus escape).
//! 6. For each band, pick the cheapest Huffman codebook among 0 (all-zero)
//!    and 1-11 whose LAV is compatible. Merge runs of the same codebook
//!    across bands into a single section.
//! 7. Encode scalefactor deltas via the scalefactor Huffman codebook.
//! 8. Write the SCE or CPE element, then `ID_END`, and pad to byte
//!    boundary.
//! 9. Wrap in an ADTS header (single raw_data_block).
//!
//! The round-trip acceptance bar is ffmpeg's AAC decoder + our own decoder
//! reporting a Goertzel ratio >= 50× at the source tone frequency on a
//! 1-second synthesised sine.
//!
//! Partially implemented:
//!   - TNS analysis + filter emission for SCE (mono) long windows. See
//!     `tns_analyse.rs`. CPE (stereo) leaves TNS off for now because the
//!     M/S per-band decision would need to take the TNS-flattened spectrum
//!     into account, which isn't wired yet.
//!
//! Not implemented (deferred): PNS (encoder side), intensity stereo, pulse
//! data, short-block/transient detection, gain control.

use std::collections::VecDeque;

use oxideav_core::Encoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, MediaType, Packet, Result, SampleFormat,
    TimeBase,
};

use crate::huffman_tables::{
    BOOK10_BITS, BOOK10_CODES, BOOK11_BITS, BOOK11_CODES, BOOK1_BITS, BOOK1_CODES, BOOK2_BITS,
    BOOK2_CODES, BOOK3_BITS, BOOK3_CODES, BOOK4_BITS, BOOK4_CODES, BOOK5_BITS, BOOK5_CODES,
    BOOK6_BITS, BOOK6_CODES, BOOK7_BITS, BOOK7_CODES, BOOK8_BITS, BOOK8_CODES, BOOK9_BITS,
    BOOK9_CODES, SCALEFACTOR_BITS, SCALEFACTOR_CODES,
};
use crate::ics::SPEC_LEN;
use crate::ics::{INTENSITY_HCB, INTENSITY_HCB2, NOISE_HCB};
use crate::mdct::mdct_long;
use crate::psy::PsyModel;
use crate::sfband::{SWB_LONG, SWB_SHORT};
use crate::syntax::{ElementType, WindowSequence, WindowShape, AOT_AAC_LC, SAMPLE_RATES};
use crate::tns_analyse::{analyse_long as tns_analyse_long, TnsEncFilter};
use crate::transient::TransientDetector;
use crate::window::{build_long_window_full, sine_long};

/// Process-global psy override read once on first encoder construction
/// (env-var `OXIDEAV_AAC_PSY_MODEL=1`). Per-encoder
/// [`AacEncoder::set_enable_psy_model`] takes precedence over the env
/// flag but the default initial value tracks it.
fn psy_default_enabled() -> bool {
    crate::psy::enabled_via_env()
}

thread_local! {
    /// Per-thread psy override for the helpers that don't carry an
    /// `&AacEncoder`. Cheap path: when the encoder enters
    /// [`AacEncoder::emit_payload`] it sets this for the duration of
    /// the call; the helpers (`analyse_and_quantise_opts`,
    /// `analyse_cpe`) read it. Not pretty but keeps the public API
    /// stable.
    static PSY_OVERRIDE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Run a closure with the per-thread psy flag set, restoring the
/// previous value on drop.
fn with_psy<F, R>(on: bool, f: F) -> R
where
    F: FnOnce() -> R,
{
    let prev = PSY_OVERRIDE.with(|c| c.replace(on));
    let r = f();
    PSY_OVERRIDE.with(|c| c.set(prev));
    r
}

fn psy_active() -> bool {
    PSY_OVERRIDE.with(|c| c.get()) || psy_default_enabled()
}
use oxideav_core::bits::BitWriter;

/// MDCT length (long block).
const FRAME_LEN: usize = 1024;
/// Full windowed block length (= 2*FRAME_LEN).
const BLOCK_LEN: usize = 2 * FRAME_LEN;

/// Magic number in the AAC quantizer rounding rule (§4.6.6):
/// `ix = floor(|x_scaled|^(3/4) + MAGIC_NUMBER)`.
const QUANT_MAGIC: f32 = 0.4054;

/// Forward-MDCT scale that pairs with our `2/input_n` IMDCT to give a
/// **unity-RMS** round-trip per ISO/IEC 14496-3 §4.6.11.3.1.
///
/// Derivation (clean-room, against the 14496-3:2009 spec only):
///
/// 1. The spec IMDCT formula is `x[n] = (2/N) · Σ spec[k] · cos((2π/N)
///    (n+n0)(k+½))` with `N = 2 · input_n`. Our implementation uses
///    `2/input_n = 4/N`, i.e. **2× the spec scale**, compensated at the
///    PCM-output stage by the `* 0.5` in `decoder.rs`.
/// 2. With unscaled forward MDCT (`Σ x · cos(...)`) and the spec inverse
///    `2/N`, sine-windowed TDAC OLA reconstructs **0.5× the input** —
///    verified empirically. To produce a unity-RMS round-trip a spec-
///    compliant encoder must scale its emitted spectrum by 2× so the
///    spec decoder's `2/N · Σ` lands on s16-range output (per §4.5.2.3.6:
///    "the integer part of the output of the IMDCT can be used directly
///    as a 16-bit PCM audio output").
/// 3. Our encoder normalises S16 input as `s/32768` (range `[-1, 1]`).
///    To bring that to an s16-range spectrum we need a forward scale of
///    `2 × 32768 = 65_536`.
///
/// **Round-19 RMS interop measurement** (sine 440 Hz, amp 0.3, expected
/// RMS `0.3 · 32767 / √2 ≈ 6951`; see `tests/lc_rms_interop_r19.rs`):
///
/// | direction                       | RMS  | ratio |
/// |---------------------------------|------|-------|
/// | ours-encode → ours-decode       | 6650 | 0.96× |
/// | ours-encode → ffmpeg-decode     | 6650 | 0.96× |
/// | ffmpeg-encode → ours-decode     | 6950 | 1.00× |
/// | ffmpeg-encode → ffmpeg-decode   | 6950 | 1.00× |
///
/// All four within ±5 % of unity — pipeline is spec-correct. Note that
/// the **peak** ratios diverge (1.79× for ffmpeg→ours) because ffmpeg's
/// AAC encoder fills HF bands with PNS-coded noise (`codebook 13`,
/// §4.6.13); our spec-correct decoder reconstructs that noise, which
/// rides additively on the sine peak. PNS is non-deterministic per
/// frame, so peak-ratio is not a meaningful interop metric for tonal-
/// with-noise content. RMS is.
///
/// **Round-23 audit (2026-04-30)**: the r22 thesis that this constant
/// should drop 16× to `4096` was tested directly and refuted. With
/// `MDCT_FORWARD_SCALE = 4096`:
///
/// - LC RMS at 44.1 kHz / 440 Hz drops to ratio **0.060** (16× too
///   quiet — the test's ±5 % tol fails immediately).
/// - HE-AAC SBR amplitude only drops from peak 32 768 → 25 287, still
///   saturated.
/// - Pure AAC-LC at 24 kHz / 1 kHz **mono** with current SCALE = 65 536
///   produces ffmpeg-decoded peak 10 930 / RMS 6 955 — within ±5 % of
///   the input. Pure stereo at 24 kHz / (1k+2k) likewise lands within
///   ±5 % per channel. r22's "pure-LC-saturates" claim mixed up the
///   HE-AAC code path (which carries an SBR FIL extension) with pure
///   LC. r22's "RMS lands on target at SCALE = 4 096" reading was a
///   methodological error — a clipped square-wave has RMS ≈ 30 000;
///   reducing SCALE 16× simply lowers input below the clipping
///   threshold and the RMS *passes through* the target on its way to
///   silence (verified: SCALE = 2 048 → RMS 1 891, SCALE = 1 024 →
///   RMS 947).
///
/// Conclusion: 65 536 is the correct value. The HE-AAC ffmpeg-interop
/// gap lives in the SBR FIL extension itself, not the LC core.
const MDCT_FORWARD_SCALE: f32 = 65536.0;

/// Largest un-escaped amplitude supported by book 11.
const ESC_LAV: i32 = 16;

pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    Ok(Box::new(AacEncoder::new(params)?))
}

/// Per-channel state for the short-block encoder state machine. One
/// instance per input channel, carried across frames.
#[derive(Clone, Debug, Default)]
struct ChannelShortState {
    /// Per-channel transient detector. Consumes the raw (unwindowed) new
    /// samples of each frame.
    transient: TransientDetector,
    /// 1-frame lookahead buffer: the spectrum and WindowSequence decided
    /// in the previous `emit_block` call, awaiting emission on the next
    /// call. `None` before the first frame has been processed or after a
    /// [`drain_held`]-style flush.
    held: Option<HeldBlock>,
}

#[derive(Clone, Debug)]
struct HeldBlock {
    /// 1024 forward-MDCT coefficients (either long-block or 8 × 128
    /// short-block layout, depending on `seq`).
    spec: Vec<f32>,
    seq: WindowSequence,
}

pub struct AacEncoder {
    codec_id: CodecId,
    out_params: CodecParameters,
    time_base: TimeBase,
    channels: u16,
    /// AAC `channel_configuration` (§1.6.3) matching `channels`. Written to
    /// every ADTS header and used to look up the SCE/CPE/LFE element
    /// sequence.
    channel_configuration: u8,
    sample_rate: u32,
    sf_index: u8,
    bitrate: u64,
    /// Input PCM sample format expected on incoming frames. Defaults to
    /// `S16` when the input params don't specify one (matches the
    /// historical default). The encoder accepts `S16` and `F32`.
    input_sample_format: SampleFormat,
    /// Per-channel running PCM buffer (float in [-1, 1]).
    input_buf: Vec<Vec<f32>>,
    /// Per-channel overlap from the previous block's right half.
    overlap: Vec<Vec<f32>>,
    output_queue: VecDeque<Packet>,
    pts: i64,
    flushed: bool,
    /// When true, `emit_block` runs the 1-frame-lookahead state machine
    /// and may emit OnlyLong / LongStart / EightShort / LongStop frames
    /// driven by transient detection. When false (default) every frame is
    /// emitted as OnlyLong via the long-only path.
    enable_short_blocks: bool,
    /// Per-channel short-block state; only populated when
    /// `enable_short_blocks` is true.
    short_state: Vec<ChannelShortState>,
    /// Optional SBR payload bits to splice in as a FIL element on the
    /// *next* raw_data_block, before `ID_END`. When `Some`, the encoder
    /// writes the Fil element header (3-bit ID + 4-bit cnt [or 4+8],
    /// 4-bit EXT_SBR_DATA) followed by the payload bytes. Consumed
    /// (taken) each call — the HE-AAC wrapper must set it before every
    /// frame.
    pending_sbr_fil: Option<SbrFilBits>,
    /// Total raw input samples (per-channel) accepted so far. Used by
    /// [`Self::gapless_info`] / [`Self::valid_samples`] so a downstream
    /// MP4 muxer or iTunSMPB writer can express "trim N priming samples
    /// off the front and M padding samples off the tail".
    total_input_samples: u64,
    /// Encoder-side count of `raw_data_block`s emitted so far. Increments
    /// on every [`Self::emit_payload`] — combined with `FRAME_LEN` this
    /// gives the total *emitted* sample count (which always exceeds
    /// `total_input_samples + encoder_delay` by the OLA flush tail; the
    /// difference is the padding count).
    frames_emitted: u64,
    /// Latched encoder-delay value reported by [`Self::encoder_delay`].
    /// Defaults to [`crate::gapless::ENCODER_DELAY_AAC_LC`] (2112 samples,
    /// the Apple iTunes convention for AAC-LC). The HE-AAC wrappers
    /// override this to [`crate::gapless::ENCODER_DELAY_HE_AAC`] (2624
    /// samples) via [`Self::set_encoder_delay`] so a single inner-encoder
    /// instance can serve either profile without callers having to
    /// special-case which convention to read.
    encoder_delay: u32,
    /// When true, the encoder uses the Bark-band PE/SMR psychoacoustic
    /// model from [`crate::psy`] to drive per-band scalefactor
    /// allocation, instead of the legacy flat `target_max = 7` rule.
    ///
    /// Defaults to the value of env var `OXIDEAV_AAC_PSY_MODEL` (set ⇒
    /// on, unset/0/off ⇒ off). Per-encoder override:
    /// [`Self::set_enable_psy_model`].
    enable_psy_model: bool,
    /// When true, [`Self::flush_final`] skips the gapless-padding loop
    /// that emits extra silence frames to reach the
    /// `frames_emitted * FRAME_LEN >= encoder_delay + valid_samples`
    /// invariant. The HE-AAC wrappers set this so they can manage their
    /// own SBR-FIL-aware silence-frame emission (each tail frame in the
    /// SBR path needs a freshly-analysed FIL element staged or ffmpeg
    /// trips `"No quantized data read for sbr_dequant"`). Standalone
    /// AAC-LC encoders leave it false so the gapless invariant holds.
    skip_gapless_padding: bool,
}

/// Staged SBR FIL payload: raw byte-aligned SBR bytes (produced by
/// [`crate::sbr::encode::SbrEncoder::emit_sbr_payload`]) plus the exact
/// number of bits they contain.
#[derive(Clone, Debug)]
pub struct SbrFilBits {
    /// Byte-aligned SBR payload. The `bits` field is authoritative — the
    /// final byte may have tail zero-padding.
    pub bytes: Vec<u8>,
    /// Number of *bits* of meaningful SBR data in `bytes` (the
    /// byte-alignment padding is not counted).
    pub bits: u32,
}

impl AacEncoder {
    /// Construct an unboxed AAC-LC encoder from a set of
    /// [`CodecParameters`]. Used by [`make_encoder`] (which boxes the
    /// result into a `dyn Encoder`); callers that need to reach
    /// through-type extensions like [`Self::set_enable_short_blocks`]
    /// can instantiate directly.
    pub fn new(params: &CodecParameters) -> Result<Self> {
        let channels = params
            .channels
            .ok_or_else(|| Error::invalid("AAC encoder: channels required"))?;
        // Map input channel count → AAC channel_configuration (§1.6.3).
        // Supported: 1, 2, 3, 4, 5, 6, 8 (configs 1..=7). 7 channels
        // has no standard config so it's rejected.
        let channel_configuration: u8 = match channels {
            1 => 1,
            2 => 2,
            3 => 3,
            4 => 4,
            5 => 5,
            6 => 6,
            8 => 7,
            _ => {
                return Err(Error::unsupported(format!(
                    "AAC encoder: {channels}-channel layout has no standard channel_configuration",
                )));
            }
        };
        let sample_rate = params
            .sample_rate
            .ok_or_else(|| Error::invalid("AAC encoder: sample_rate required"))?;
        let sf_index = SAMPLE_RATES
            .iter()
            .position(|&r| r == sample_rate)
            .ok_or_else(|| {
                Error::unsupported(format!(
                    "AAC encoder: sample rate {sample_rate} not supported"
                ))
            })? as u8;

        let bitrate = params.bit_rate.unwrap_or(128_000).max(16_000);

        let mut out_params = CodecParameters::audio(CodecId::new(crate::CODEC_ID_STR));
        out_params.media_type = MediaType::Audio;
        out_params.channels = Some(channels);
        out_params.sample_rate = Some(sample_rate);
        out_params.sample_format = Some(SampleFormat::S16);
        out_params.bit_rate = Some(bitrate);

        let n_ch = channels as usize;
        let input_sample_format = params.sample_format.unwrap_or(SampleFormat::S16);
        Ok(AacEncoder {
            codec_id: CodecId::new(crate::CODEC_ID_STR),
            out_params,
            time_base: TimeBase::new(1, sample_rate as i64),
            channels,
            channel_configuration,
            sample_rate,
            sf_index,
            bitrate,
            input_sample_format,
            input_buf: vec![Vec::with_capacity(BLOCK_LEN * 2); n_ch],
            overlap: vec![vec![0.0f32; FRAME_LEN]; n_ch],
            output_queue: VecDeque::new(),
            pts: 0,
            flushed: false,
            enable_short_blocks: false,
            short_state: (0..n_ch).map(|_| ChannelShortState::default()).collect(),
            pending_sbr_fil: None,
            total_input_samples: 0,
            frames_emitted: 0,
            encoder_delay: crate::gapless::ENCODER_DELAY_AAC_LC,
            enable_psy_model: psy_default_enabled(),
            skip_gapless_padding: false,
        })
    }

    /// Disable (or re-enable) the flush-time gapless-padding silence
    /// loop. The HE-AAC wrappers call this with `true` because each
    /// silence tail frame they emit must carry its own pre-staged SBR
    /// FIL element (otherwise ffmpeg's `ff_aac_sbr_apply` trips
    /// `"No quantized data read for sbr_dequant"` on the trailing frame).
    /// Standalone AAC-LC users leave the default `false`.
    pub fn set_skip_gapless_padding(&mut self, skip: bool) {
        self.skip_gapless_padding = skip;
    }

    /// Build the AudioSpecificConfig blob a downstream MP4 muxer needs
    /// to advertise this stream in its `esds` box (or that a DASH
    /// manifest needs to inline as a base64 `codecs` parameter). The
    /// returned bytes describe the AAC-LC core configuration the
    /// encoder is currently emitting; HE-AAC wrappers
    /// ([`crate::he_aac_encoder::HeAacMonoEncoder`] etc.) carry their
    /// own `audio_specific_config()` that prepends the explicit
    /// AOT-5/AOT-29 SBR/PS extension prefix.
    pub fn audio_specific_config(&self) -> Vec<u8> {
        // `Self::new` already validated channel_configuration ∈ 1..=7
        // and sample_rate against the standard SF index table; the
        // builder cannot fail here.
        crate::asc::AscBuilder::lc(self.sample_rate, self.channel_configuration)
            .expect("encoder constructor pre-validates ASC builder inputs")
    }

    /// Override the reported encoder-delay value. The HE-AAC wrappers
    /// call this with [`crate::gapless::ENCODER_DELAY_HE_AAC`] so the
    /// inner [`AacEncoder`] reports the SBR-aware figure (2624 samples
    /// at the high rate) rather than the AAC-LC default (2112). The
    /// MDCT priming behaviour itself is unchanged — this only affects
    /// what [`Self::gapless_info`] / [`Self::iTunSMPB_string`] report
    /// to downstream container code.
    pub fn set_encoder_delay(&mut self, samples: u32) {
        self.encoder_delay = samples;
    }

    /// Encoder delay in samples: how many priming samples a downstream
    /// player should skip before the first "real" sample of the source
    /// PCM appears. Defaults to 2112 (AAC-LC) and is bumped to 2624 by
    /// the HE-AAC wrappers. See [`crate::gapless`] for the derivation.
    pub fn encoder_delay(&self) -> u32 {
        self.encoder_delay
    }

    /// Number of input samples (per channel) accepted via `send_frame`
    /// to date. Authoritative for the iTunSMPB "valid samples" word and
    /// for the `segment_duration` of an MP4 `elst` edit segment.
    pub fn valid_samples(&self) -> u64 {
        self.total_input_samples
    }

    /// Number of `raw_data_block`s emitted so far. One frame == 1024
    /// PCM samples per channel.
    pub fn frames_emitted(&self) -> u64 {
        self.frames_emitted
    }

    /// Padding samples at the tail (i.e.
    /// `frames_emitted * 1024 - encoder_delay - valid_samples`,
    /// saturated at zero). Only meaningful **after** [`Encoder::flush`]
    /// — before flush the encoder hasn't decided on the final frame
    /// count and the residual buffer has not yet been padded out.
    pub fn padding_samples(&self) -> u32 {
        crate::gapless::GaplessInfo::compute_padding(
            self.encoder_delay,
            self.frames_emitted,
            FRAME_LEN as u32,
            self.total_input_samples,
        )
    }

    /// Bundle (encoder_delay, padding_samples, valid_samples) into a
    /// single [`crate::gapless::GaplessInfo`] suitable for handing to
    /// an MP4 `edts/elst` writer or an iTunSMPB tag emitter.
    pub fn gapless_info(&self) -> crate::gapless::GaplessInfo {
        crate::gapless::GaplessInfo::new(
            self.encoder_delay,
            self.padding_samples(),
            self.total_input_samples,
        )
    }

    /// Convenience: format the gapless triple as an Apple iTunSMPB
    /// metadata-tag string. Container code that wants ID3v2 carriage
    /// emits this string as the value of a `TXXX` user-text frame with
    /// description `"iTunSMPB"`; MP4 ilst carriage uses the same value
    /// inside an `----` named atom (`com.apple.iTunes:iTunSMPB`).
    #[allow(non_snake_case)]
    pub fn iTunSMPB_string(&self) -> String {
        self.gapless_info().format_itunsmpb()
    }

    /// Stage an SBR payload (as produced by the HE-AACv1 encoder) so the
    /// *next* `raw_data_block` carries it as a FIL / EXT_SBR_DATA
    /// element inserted before `ID_END`. One-shot — consumed on the next
    /// frame emission.
    ///
    /// Use [`crate::sbr::encode::SbrEncoder`] to produce the bytes and
    /// call this before feeding the matching low-band PCM frame.
    pub fn stage_sbr_fil(&mut self, sbr: SbrFilBits) {
        self.pending_sbr_fil = Some(sbr);
    }

    /// Enable (or disable) the short-block encoder state machine. Off by
    /// default — every frame is emitted as `OnlyLong`. When on, the
    /// encoder runs a per-channel [`TransientDetector`] against each new
    /// frame and may transition through
    /// `OnlyLong → LongStart → EightShort → LongStop → OnlyLong` in
    /// response to attacks, localising them with 128-sample sub-windows
    /// and suppressing long-window pre-echo.
    ///
    /// Enabling adds one frame of encoder latency (the lookahead buffer).
    /// If turned on mid-stream after frames have already been emitted,
    /// the per-channel held spectrum is cleared so the state machine
    /// starts fresh.
    pub fn set_enable_short_blocks(&mut self, on: bool) {
        self.enable_short_blocks = on;
        if !on {
            // Drop any held state so the long-only path resumes cleanly.
            for s in self.short_state.iter_mut() {
                s.held = None;
            }
        }
    }

    /// Enable (or disable) the Bark-band PE / SMR psychoacoustic model
    /// from [`crate::psy`] for per-band scalefactor allocation. Off by
    /// default unless `OXIDEAV_AAC_PSY_MODEL` is set in the environment
    /// at construction time.
    ///
    /// On tonal content this typically improves PSNR / RMS-fidelity at
    /// matched bitrate by redistributing quantisation precision: tonal
    /// loud bands get finer steps, quiet bands hidden under loud
    /// neighbours get coarser steps. The exact bitrate impact depends on
    /// the input signal — bench against your fixtures before flipping
    /// it on for a production encoder.
    ///
    /// See [`crate::psy`] for the model description and ISO clause
    /// references.
    pub fn set_enable_psy_model(&mut self, on: bool) {
        self.enable_psy_model = on;
    }

    /// Read the current psy-model toggle value.
    pub fn enable_psy_model(&self) -> bool {
        self.enable_psy_model
    }

    fn push_audio_frame(&mut self, frame: &AudioFrame) -> Result<()> {
        let n = frame.samples as usize;
        if n == 0 {
            return Ok(());
        }
        let plane = frame
            .data
            .first()
            .ok_or_else(|| Error::invalid("AAC encoder: frame missing data plane"))?;
        match self.input_sample_format {
            SampleFormat::S16 => {
                let stride = self.channels as usize * 2;
                if plane.len() < n * stride {
                    return Err(Error::invalid("AAC encoder: S16 frame too short"));
                }
                for i in 0..n {
                    for ch in 0..self.channels as usize {
                        let off = i * stride + ch * 2;
                        let s = i16::from_le_bytes([plane[off], plane[off + 1]]);
                        self.input_buf[ch].push(s as f32 / 32768.0);
                    }
                }
            }
            SampleFormat::F32 => {
                let stride = self.channels as usize * 4;
                if plane.len() < n * stride {
                    return Err(Error::invalid("AAC encoder: F32 frame too short"));
                }
                for i in 0..n {
                    for ch in 0..self.channels as usize {
                        let off = i * stride + ch * 4;
                        let v = f32::from_le_bytes([
                            plane[off],
                            plane[off + 1],
                            plane[off + 2],
                            plane[off + 3],
                        ]);
                        self.input_buf[ch].push(v);
                    }
                }
            }
            other => {
                return Err(Error::unsupported(format!(
                    "AAC encoder: input sample format {other:?} not supported"
                )));
            }
        }
        // Track the **per-channel** input-sample count for gapless
        // metadata. The buffer's per-channel length grows by `n`
        // regardless of the channel count (we push n samples per
        // channel), so the total counter advances by `n` not `n *
        // channels`.
        self.total_input_samples = self.total_input_samples.saturating_add(n as u64);
        Ok(())
    }

    /// Emit one or more AAC frames while we have a full FRAME_LEN of new
    /// samples buffered.
    fn drain_blocks(&mut self) -> Result<()> {
        while self.input_buf[0].len() >= FRAME_LEN {
            self.emit_block(false)?;
        }
        Ok(())
    }

    /// Emit a final (possibly silence-padded) block when flushing.
    fn flush_final(&mut self) -> Result<()> {
        // Drain whatever full blocks remain.
        self.drain_blocks()?;
        // If there are any leftover samples, pad to FRAME_LEN with zeros
        // and emit one more block so the decoder's overlap-add produces
        // the last samples.
        if self.input_buf[0].is_empty() {
            // Even with no pending samples, emit a silence-block tail so
            // the decoder's first-frame latency is flushed out.
            for ch in 0..self.channels as usize {
                self.input_buf[ch].resize(FRAME_LEN, 0.0);
            }
            self.emit_block(true)?;
        } else {
            for ch in 0..self.channels as usize {
                if self.input_buf[ch].len() < FRAME_LEN {
                    self.input_buf[ch].resize(FRAME_LEN, 0.0);
                }
            }
            self.emit_block(true)?;
        }
        // The short-block path adds one more frame of latency (the
        // held lookahead buffer). Drain it so the last real frame is
        // actually emitted.
        if self.enable_short_blocks {
            self.drain_held()?;
        }
        // ===== Gapless padding tuning (task #169) =====
        //
        // The Apple iTunSMPB convention demands the bitstream invariant
        //
        //     frames_emitted * FRAME_LEN >= encoder_delay + valid_samples
        //
        // so the (delay, padding, valid_samples) triple stays internally
        // consistent: a player skipping `encoder_delay` priming samples
        // and trimming `padding_samples` off the tail must land on
        // exactly `valid_samples` of real PCM. Emit additional silence
        // frames here until the invariant holds (rounding up to the next
        // packet boundary). This keeps the per-stream tail honest and
        // — crucially for concatenated playback — gives downstream
        // muxers a frame-boundary-aligned padding count to record in
        // an `edts/elst` segment_duration or an iTunSMPB tail-pad word.
        //
        // The HE-AAC wrappers gate this loop off with
        // `set_skip_gapless_padding(true)` because every tail frame they
        // emit must carry a freshly-staged SBR FIL element; the wrapper
        // manages its own padding-frame loop and stages an SBR FIL on
        // each iteration.
        if !self.skip_gapless_padding {
            let need = (self.encoder_delay as u64).saturating_add(self.total_input_samples);
            let mut emitted_samples = self.frames_emitted.saturating_mul(FRAME_LEN as u64);
            while emitted_samples < need {
                for ch in 0..self.channels as usize {
                    self.input_buf[ch].clear();
                    self.input_buf[ch].resize(FRAME_LEN, 0.0);
                }
                self.emit_block(true)?;
                if self.enable_short_blocks {
                    // The short-block path holds one frame back; drain it
                    // so the silence frame actually emerges from the
                    // encoder. Without this the loop never advances.
                    self.drain_held()?;
                }
                emitted_samples = self.frames_emitted.saturating_mul(FRAME_LEN as u64);
            }
        }
        Ok(())
    }

    fn emit_block(&mut self, is_last: bool) -> Result<()> {
        let psy = self.enable_psy_model;
        with_psy(psy, || {
            if self.enable_short_blocks {
                self.emit_block_short(is_last)
            } else {
                self.emit_block_long(is_last)
            }
        })
    }

    /// Long-only emit path — every frame is OnlyLong. Simpler and used
    /// whenever `enable_short_blocks` is false.
    fn emit_block_long(&mut self, _is_last: bool) -> Result<()> {
        let n_ch = self.channels as usize;
        // Build the 2N windowed block per channel:
        //   first_half = overlap[ch]      (was saved after last block)
        //   second_half = next FRAME_LEN samples from input_buf (window'd)
        let mut blocks: Vec<Vec<f32>> = vec![vec![0.0; BLOCK_LEN]; n_ch];
        let win = sine_long();
        for ch in 0..n_ch {
            for i in 0..FRAME_LEN {
                blocks[ch][i] = self.overlap[ch][i] * win[i];
            }
            // Pull next FRAME_LEN samples.
            for i in 0..FRAME_LEN {
                let sample = self.input_buf[ch][i];
                blocks[ch][FRAME_LEN + i] = sample * win[FRAME_LEN - 1 - i];
            }
        }
        // Update overlap to the *unwindowed* upcoming-new samples so the
        // next block's first_half * win(rising) matches the just-emitted
        // second_half * win(falling) and OLA reconstructs the input.
        for ch in 0..n_ch {
            let new_overlap: Vec<f32> = self.input_buf[ch][..FRAME_LEN].to_vec();
            self.overlap[ch] = new_overlap;
            self.input_buf[ch].drain(..FRAME_LEN);
        }

        // Forward MDCT per channel. ffmpeg's AAC encoder applies a 32768
        // scale on the forward MDCT (to match the int16 input range —
        // see `aacenc.c::dsp_init`). We do the same so the spectrum
        // values land in the range the standard inverse-quantisation
        // expects.
        let mut specs: Vec<Vec<f32>> = vec![vec![0.0; FRAME_LEN]; n_ch];
        for ch in 0..n_ch {
            mdct_long(&blocks[ch], &mut specs[ch]);
            for v in specs[ch].iter_mut() {
                *v *= MDCT_FORWARD_SCALE;
            }
        }

        // Frame header + raw_data_block.
        let payload = self.encode_raw_data_block(&specs)?;
        self.emit_payload(payload)
    }

    /// Short-block emit path. 1-frame lookahead: analyse the new samples,
    /// decide the current frame's `WindowSequence` from the state table,
    /// compute its spectrum with the appropriate MDCT/windowing path,
    /// emit the previously-held spectrum, then stash the current
    /// spectrum as the new hold.
    fn emit_block_short(&mut self, _is_last: bool) -> Result<()> {
        let n_ch = self.channels as usize;
        let element_seq = element_sequence(self.channel_configuration);

        // Step 1 — per-channel transient detection on the new samples.
        let mut transients = vec![false; n_ch];
        for ch in 0..n_ch {
            transients[ch] = self.short_state[ch]
                .transient
                .analyse(&self.input_buf[ch][..FRAME_LEN]);
        }

        // Step 2 — reduce per element: CPE pairs OR their transient
        // flags so both channels share a unified decision; LFE is
        // long-only per §4.6.10 so its transient is forced false.
        let mut elem_transients = vec![false; n_ch];
        let mut ch_idx = 0usize;
        for &el in element_seq {
            match el {
                AacElement::Sce => {
                    elem_transients[ch_idx] = transients[ch_idx];
                    ch_idx += 1;
                }
                AacElement::Cpe => {
                    let t = transients[ch_idx] || transients[ch_idx + 1];
                    elem_transients[ch_idx] = t;
                    elem_transients[ch_idx + 1] = t;
                    ch_idx += 2;
                }
                AacElement::Lfe => {
                    elem_transients[ch_idx] = false;
                    ch_idx += 1;
                }
            }
        }

        // Step 3 — compute cur_seq per channel from the state table.
        // prev_seq is the held spectrum's seq (the one we're ABOUT to
        // emit); on the first call it defaults to OnlyLong.
        let mut cur_seqs = vec![WindowSequence::OnlyLong; n_ch];
        let mut ch_idx = 0usize;
        for &el in element_seq {
            match el {
                AacElement::Sce => {
                    let prev = self.short_state[ch_idx]
                        .held
                        .as_ref()
                        .map(|h| h.seq)
                        .unwrap_or(WindowSequence::OnlyLong);
                    cur_seqs[ch_idx] = next_window_seq(prev, elem_transients[ch_idx]);
                    ch_idx += 1;
                }
                AacElement::Cpe => {
                    let prev = self.short_state[ch_idx]
                        .held
                        .as_ref()
                        .map(|h| h.seq)
                        .unwrap_or(WindowSequence::OnlyLong);
                    let next = next_window_seq(prev, elem_transients[ch_idx]);
                    cur_seqs[ch_idx] = next;
                    cur_seqs[ch_idx + 1] = next;
                    ch_idx += 2;
                }
                AacElement::Lfe => {
                    cur_seqs[ch_idx] = WindowSequence::OnlyLong;
                    ch_idx += 1;
                }
            }
        }

        // Step 4 — window + forward-MDCT per channel. For long
        // sequences we use `build_long_window_full` so the asymmetric
        // LongStart / LongStop shapes are applied correctly. For
        // EightShort we use `mdct_short_eightshort` which internally
        // applies per-sub-window sine windowing with w=0's rising edge
        // coming from the prev shape.
        let shape = WindowShape::Sine;
        let mut specs: Vec<Vec<f32>> = vec![vec![0.0f32; FRAME_LEN]; n_ch];
        for ch in 0..n_ch {
            let cur_seq = cur_seqs[ch];
            // Assemble the 2N-sample unwindowed block (= prev overlap |
            // new samples).
            let mut block = vec![0.0f32; BLOCK_LEN];
            for i in 0..FRAME_LEN {
                block[i] = self.overlap[ch][i];
                block[FRAME_LEN + i] = self.input_buf[ch][i];
            }
            if cur_seq == WindowSequence::EightShort {
                let mut short_spec = [0.0f32; 1024];
                crate::mdct::mdct_short_eightshort(&block, shape, shape, &mut short_spec);
                for v in short_spec.iter_mut() {
                    *v *= MDCT_FORWARD_SCALE;
                }
                specs[ch].copy_from_slice(&short_spec);
            } else {
                let window = build_long_window_full(cur_seq, shape, shape);
                for i in 0..BLOCK_LEN {
                    block[i] *= window[i];
                }
                mdct_long(&block, &mut specs[ch]);
                for v in specs[ch].iter_mut() {
                    *v *= MDCT_FORWARD_SCALE;
                }
            }
        }

        // Step 5 — update overlap to the new (unwindowed) samples and
        // drain them from the input buffer.
        for ch in 0..n_ch {
            self.overlap[ch] = self.input_buf[ch][..FRAME_LEN].to_vec();
            self.input_buf[ch].drain(..FRAME_LEN);
        }

        // Step 6 — emit the previously-held frame (if any).
        if self.short_state[0].held.is_some() {
            self.emit_held_frame()?;
        }

        // Step 7 — save (specs, cur_seqs) as the new held.
        for ch in 0..n_ch {
            self.short_state[ch].held = Some(HeldBlock {
                spec: std::mem::take(&mut specs[ch]),
                seq: cur_seqs[ch],
            });
        }

        Ok(())
    }

    /// Emit the currently-held (1-frame-deferred) frame as an ADTS
    /// packet. Called from `emit_block_short` when a new frame arrives
    /// and again from `drain_held` on flush.
    fn emit_held_frame(&mut self) -> Result<()> {
        let n_ch = self.channels as usize;
        let mut specs: Vec<Vec<f32>> = Vec::with_capacity(n_ch);
        let mut seqs: Vec<WindowSequence> = Vec::with_capacity(n_ch);
        for ch in 0..n_ch {
            let h = self.short_state[ch]
                .held
                .as_ref()
                .ok_or_else(|| Error::other("AAC encoder: held frame missing"))?;
            specs.push(h.spec.clone());
            seqs.push(h.seq);
        }
        let payload = self.encode_raw_data_block_seq(&specs, &seqs)?;
        self.emit_payload(payload)
    }

    /// Drain any remaining held frame at flush time.
    fn drain_held(&mut self) -> Result<()> {
        if self.short_state.is_empty() || self.short_state[0].held.is_none() {
            return Ok(());
        }
        self.emit_held_frame()?;
        for s in self.short_state.iter_mut() {
            s.held = None;
        }
        Ok(())
    }

    /// Wrap a raw_data_block payload in ADTS and push onto the output
    /// queue, advancing `pts` by one frame.
    fn emit_payload(&mut self, payload: Vec<u8>) -> Result<()> {
        let samples_per_frame = FRAME_LEN as u32;
        let mut adts_frame =
            build_adts_frame(self.sf_index, self.channel_configuration, payload.len());
        adts_frame.extend_from_slice(&payload);
        let pkt = Packet::new(0, self.time_base, adts_frame).with_pts(self.pts);
        self.pts += samples_per_frame as i64;
        self.output_queue.push_back(pkt);
        // Tally the emitted-frame count for the gapless-padding maths
        // (see [`Self::padding_samples`]). One ADTS frame == one
        // raw_data_block == FRAME_LEN samples per channel.
        self.frames_emitted = self.frames_emitted.saturating_add(1);
        Ok(())
    }

    fn encode_raw_data_block(&mut self, specs: &[Vec<f32>]) -> Result<Vec<u8>> {
        let all_long = vec![WindowSequence::OnlyLong; specs.len()];
        self.encode_raw_data_block_seq(specs, &all_long)
    }

    /// Build one raw_data_block with per-channel [`WindowSequence`]. When
    /// `seqs[ch]` is `EightShort`, that channel's spectrum is emitted via
    /// the short-window writer (`write_single_ics_short`) and CPE pairs
    /// fall back to `common_window = 0` — each channel carries its own
    /// `ics_info`, M/S is off. Non-EightShort sequences (OnlyLong /
    /// LongStart / LongStop) go through the existing long writers with
    /// common-window M/S intact.
    fn encode_raw_data_block_seq(
        &mut self,
        specs: &[Vec<f32>],
        seqs: &[WindowSequence],
    ) -> Result<Vec<u8>> {
        let mut bw = BitWriter::with_capacity(1024);
        let element_seq = element_sequence(self.channel_configuration);
        if element_seq.is_empty() {
            return Err(Error::unsupported(format!(
                "AAC encoder: channel_configuration={} unsupported",
                self.channel_configuration
            )));
        }
        let mut ch_idx: usize = 0;
        let mut sce_tag: u8 = 0;
        let mut cpe_tag: u8 = 0;
        let mut lfe_tag: u8 = 0;
        for &el in element_seq {
            match el {
                AacElement::Sce => {
                    bw.write_u32(ElementType::Sce as u32, 3);
                    bw.write_u32(sce_tag as u32, 4);
                    write_single_ics_any(&mut bw, &specs[ch_idx], self.sf_index, seqs[ch_idx])?;
                    ch_idx += 1;
                    sce_tag += 1;
                }
                AacElement::Cpe => {
                    bw.write_u32(ElementType::Cpe as u32, 3);
                    bw.write_u32(cpe_tag as u32, 4);
                    // Short-window CPE: drop common_window so each channel
                    // carries its own ics_info; long-window CPE keeps the
                    // existing common_window=1 + per-band M/S path.
                    let l_seq = seqs[ch_idx];
                    let r_seq = seqs[ch_idx + 1];
                    debug_assert_eq!(l_seq, r_seq, "CPE channels must share seq");
                    if l_seq == WindowSequence::EightShort {
                        bw.write_bit(false); // common_window = 0
                        write_single_ics_any(&mut bw, &specs[ch_idx], self.sf_index, l_seq)?;
                        write_single_ics_any(&mut bw, &specs[ch_idx + 1], self.sf_index, r_seq)?;
                    } else {
                        write_cpe(
                            &mut bw,
                            &specs[ch_idx],
                            &specs[ch_idx + 1],
                            self.sf_index,
                            l_seq,
                        )?;
                    }
                    ch_idx += 2;
                    cpe_tag += 1;
                }
                AacElement::Lfe => {
                    bw.write_u32(ElementType::Lfe as u32, 3);
                    bw.write_u32(lfe_tag as u32, 4);
                    debug_assert!(
                        !matches!(seqs[ch_idx], WindowSequence::EightShort),
                        "LFE is long-only per §4.6.10",
                    );
                    write_single_ics_any(&mut bw, &specs[ch_idx], self.sf_index, seqs[ch_idx])?;
                    ch_idx += 1;
                    lfe_tag += 1;
                }
            }
        }
        // Optional FIL/SBR element — one-shot, consumed here. Writes the
        // 3-bit Fil element id, then the FIL payload (length field +
        // 4-bit extension_id + SBR payload bits).
        if let Some(sbr) = self.pending_sbr_fil.take() {
            write_fil_sbr_element(&mut bw, &sbr);
        }
        // ID_END
        bw.write_u32(ElementType::End as u32, 3);
        bw.align_to_byte();
        Ok(bw.finish())
    }
}

/// Write a Fil / EXT_SBR_DATA element body into `bw`. Layout:
///   3 bits  element id   = ElementType::Fil (6)
///   4 bits  cnt          (or 4 + 8 bits when total_bytes >= 15)
///   4 bits  extension_id = EXT_SBR_DATA (0xD)
///   N bits  SBR payload
///
/// The cnt field counts *bytes* covering the 4-bit extension_id plus
/// the SBR payload — the FIL element in raw_data_block is byte-oriented
/// so the SBR payload itself must be bit-stuffed with zeros up to a byte
/// boundary. `sbr.bits` is the exact count of meaningful bits; any
/// trailing zero-bits inside the final byte of `sbr.bytes` are inert
/// padding that the decoder consumes and discards.
fn write_fil_sbr_element(bw: &mut BitWriter, sbr: &SbrFilBits) {
    // Element id — Fil (6). Writes 3 bits at the current bit position.
    bw.write_u32(ElementType::Fil as u32, 3);
    // `total_bits_content` = 4 (extension_id) + sbr.bits bits of SBR
    // payload. The FIL `cnt` field counts *bytes* of content, so pad
    // the content to a multiple of 8 bits (independently of the
    // surrounding byte alignment — cnt is measured in bits relative to
    // the end of the cnt field, not the start of the raw_data_block).
    let content_bits = 4 + sbr.bits;
    let pad_bits = (8 - (content_bits % 8)) % 8;
    let total_bytes = (content_bits + pad_bits) / 8;
    if total_bytes < 15 {
        bw.write_u32(total_bytes, 4);
    } else {
        bw.write_u32(15, 4);
        // "cnt == 15 means the real count is cnt + esc - 1".
        bw.write_u32(total_bytes + 1 - 15, 8);
    }
    // 4-bit extension_id.
    bw.write_u32(crate::sbr::bitstream::EXT_SBR_DATA, 4);
    // SBR payload bits — exactly sbr.bits bits.
    let full = (sbr.bits / 8) as usize;
    for byte in &sbr.bytes[..full.min(sbr.bytes.len())] {
        bw.write_u32(*byte as u32, 8);
    }
    let tail = sbr.bits - (full as u32 * 8);
    if tail > 0 && full < sbr.bytes.len() {
        let last = sbr.bytes[full] >> (8 - tail);
        bw.write_u32(last as u32, tail);
    }
    // Content-bit pad — this is part of the declared `cnt` byte count so
    // the decoder will consume it as part of the SBR payload. The spec
    // allows this because `sbr_extension()` includes bs_fill_bits up to
    // the declared length.
    if pad_bits > 0 {
        bw.write_u32(0, pad_bits);
    }
}

/// SCE / LFE writer that dispatches between long (`write_single_ics`) and
/// short (`write_single_ics_short`) based on `seq`. For short sequences
/// the spectrum is expected to be laid out as 8 × 128 coefficients in
/// sub-window-major order (what [`mdct::mdct_short_eightshort`] produces).
fn write_single_ics_any(
    bw: &mut BitWriter,
    spec: &[f32],
    sf_index: u8,
    seq: WindowSequence,
) -> Result<()> {
    if seq == WindowSequence::EightShort {
        let spec_arr: &[f32; 1024] = spec
            .try_into()
            .map_err(|_| Error::invalid("AAC encoder: EightShort spec must be 1024 coeffs"))?;
        let ics = analyse_and_quantise_short(spec_arr, sf_index)?;
        write_single_ics_short(bw, &ics)
    } else {
        write_single_ics(bw, spec, sf_index, seq, false)
    }
}

#[derive(Clone, Copy)]
enum AacElement {
    Sce,
    Cpe,
    Lfe,
}

/// State-machine transition for the short-block encoder:
///
/// ```text
///   OnlyLong  + transient  → LongStart    // prepare for EightShort
///   OnlyLong  + no-trans   → OnlyLong
///   LongStart (any)        → EightShort   // LongStart always commits
///   EightShort + transient → EightShort   // keep resolving
///   EightShort + no-trans  → LongStop     // decay back
///   LongStop  (any)        → OnlyLong
/// ```
///
/// All five transitions are spec-legal — each `{right_edge}.{left_edge}`
/// boundary TDAC-cancels: OnlyLong⇄OnlyLong and LongStop⇄OnlyLong share
/// the full-length sine/KBD curve; OnlyLong⇄LongStart, LongStart⇄
/// EightShort, EightShort⇄EightShort, EightShort⇄LongStop chain through
/// 128-sample short-window edges.
fn next_window_seq(prev: WindowSequence, transient: bool) -> WindowSequence {
    match prev {
        WindowSequence::OnlyLong if transient => WindowSequence::LongStart,
        WindowSequence::OnlyLong => WindowSequence::OnlyLong,
        WindowSequence::LongStart => WindowSequence::EightShort,
        WindowSequence::EightShort if transient => WindowSequence::EightShort,
        WindowSequence::EightShort => WindowSequence::LongStop,
        WindowSequence::LongStop => WindowSequence::OnlyLong,
    }
}

/// Element sequence for each AAC `channel_configuration` (§1.6.3). Listed
/// in the exact order they must appear in the raw_data_block — the
/// encoder's channel vector is sliced by these positions: SCE / LFE
/// take one channel, CPE takes two.
fn element_sequence(channel_configuration: u8) -> &'static [AacElement] {
    match channel_configuration {
        1 => &[AacElement::Sce],
        2 => &[AacElement::Cpe],
        3 => &[AacElement::Sce, AacElement::Cpe],
        4 => &[AacElement::Sce, AacElement::Cpe, AacElement::Sce],
        5 => &[AacElement::Sce, AacElement::Cpe, AacElement::Cpe],
        6 => &[
            AacElement::Sce,
            AacElement::Cpe,
            AacElement::Cpe,
            AacElement::Lfe,
        ],
        7 => &[
            AacElement::Sce,
            AacElement::Cpe,
            AacElement::Cpe,
            AacElement::Cpe,
            AacElement::Lfe,
        ],
        _ => &[],
    }
}

impl Encoder for AacEncoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.out_params
    }

    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        if self.flushed {
            return Err(Error::other(
                "AAC encoder: flushed, cannot accept more frames",
            ));
        }
        match frame {
            Frame::Audio(af) => {
                self.push_audio_frame(af)?;
                self.drain_blocks()
            }
            _ => Err(Error::invalid("AAC encoder: expected audio frame")),
        }
    }

    fn receive_packet(&mut self) -> Result<Packet> {
        match self.output_queue.pop_front() {
            Some(p) => Ok(p),
            None => {
                if self.flushed {
                    Err(Error::Eof)
                } else {
                    Err(Error::NeedMore)
                }
            }
        }
    }

    fn flush(&mut self) -> Result<()> {
        if self.flushed {
            return Ok(());
        }
        self.flush_final()?;
        self.flushed = true;
        Ok(())
    }
}

// ==================== ADTS framing ====================

/// Build the 7-byte ADTS header for the given payload length.
fn build_adts_frame(sf_index: u8, channel_configuration: u8, payload_len: usize) -> Vec<u8> {
    let frame_length = payload_len + 7; // no CRC
    assert!(frame_length < (1 << 13));
    let mut hdr = [0u8; 7];
    // syncword 0xFFF
    hdr[0] = 0xFF;
    hdr[1] = 0xF0;
    // ID = 0 (MPEG-4), layer = 00, protection_absent = 1.
    hdr[1] |= 0b0001;
    // profile (AAC-LC) = 1 (0-based, so stored as 2-1=1)
    // sampling_frequency_index (4 bits), private_bit=0, channel_configuration (3 bits)
    let profile = AOT_AAC_LC - 1; // = 1
    hdr[2] = (profile << 6) | ((sf_index & 0x0F) << 2) | ((channel_configuration >> 2) & 0x01);
    hdr[3] = ((channel_configuration & 0x03) << 6) | ((frame_length >> 11) as u8 & 0x03);
    hdr[4] = ((frame_length >> 3) & 0xFF) as u8;
    // buffer_fullness = 0x7FF (variable)
    hdr[5] = (((frame_length & 0x07) << 5) as u8) | 0b11111;
    hdr[6] = 0b11111100; // remaining 6 fullness bits + number_of_raw_blocks=0
    hdr.to_vec()
}

// ==================== SCE / CPE writers ====================

fn write_single_ics(
    bw: &mut BitWriter,
    spec: &[f32],
    sf_index: u8,
    seq: WindowSequence,
    _in_cpe: bool,
) -> Result<()> {
    // global_gain (8 bits) — set later. For now write a placeholder.
    // Design: we need to pick scalefactors first so we know the gain, then
    // write the full ICS in one pass. We encode everything into a temp
    // structure and emit at the end.
    let ics = analyse_and_quantise(spec, sf_index)?;
    write_ics(bw, &ics, seq, false)?;
    Ok(())
}

fn write_cpe(
    bw: &mut BitWriter,
    spec_l: &[f32],
    spec_r: &[f32],
    sf_index: u8,
    seq: WindowSequence,
) -> Result<()> {
    // Decide M/S stereo per band. We build M/S spectra, then try both
    // representations and pick whichever needs fewer bits overall.
    let (ms_used, ics_l, ics_r) = analyse_cpe(spec_l, spec_r, sf_index)?;

    bw.write_bit(true); // common_window — share ics_info between the channels
                        // The shared ics_info uses ch0's max_sfb (which equals ch1's after the
                        // pad-to-max-sfb done in analyse_cpe).
    write_ics_info(bw, &ics_l.info, seq);
    let any_ms = ms_used.iter().any(|&b| b);
    if any_ms {
        bw.write_u32(1, 2); // ms_mask_present = 1 (explicit per-band mask)
                            // For long blocks num_window_groups = 1, so the mask is written
                            // as max_sfb consecutive bits.
        for sfb in 0..ics_l.info.max_sfb as usize {
            bw.write_bit(ms_used.get(sfb).copied().unwrap_or(false));
        }
    } else {
        bw.write_u32(0, 2); // ms_mask_present = 0
    }
    // Per-channel individual_channel_stream (no ics_info because common):
    //   global_gain (8) | section_data | scalefactor_data |
    //   pulse_data_present (1) | tns_data_present (1) |
    //   gain_control_data_present (1) | spectral_data
    write_ics_body(bw, &ics_l)?;
    write_ics_body(bw, &ics_r)?;
    Ok(())
}

// ==================== ICS analysis & writing ====================

#[derive(Clone, Debug)]
struct Ics {
    info: IcsInfoEnc,
    /// Per-band global scalefactor (8-bit int — first band is absolute,
    /// subsequent bands are deltas on encode).
    sfs: Vec<i32>,
    /// Per-band chosen codebook (0..=11 spectral, 13 PNS, 14/15 IS).
    cbs: Vec<u8>,
    /// Per-band quantised coefficients laid out in band order.
    q_bands: Vec<Vec<i32>>,
    /// Global gain value (first non-ZERO band's scalefactor, 8-bit).
    global_gain: u8,
    /// TNS filter parameters (at most one filter per window for the
    /// current encoder). `None` => `tns_data_present = 0`.
    tns: Option<TnsEncFilter>,
    /// Optional pulse_data() record (§4.6.5). When populated, the
    /// affected q_bands already have the pulse amplitudes subtracted so
    /// the sum of residual + pulse reconstructs the original quantised
    /// coefficient. `None` => `pulse_data_present = 0`.
    pulse: Option<PulseRecord>,
}

/// Up to 4 pulses per frame per §4.6.5.
const MAX_PULSES_ENC: usize = 4;

/// Pulse-data record owned by `Ics`. Offsets form a cumulative chain
/// rooted at `swb[pulse_start_sfb]`: pulse 0 sits at
/// `swb[pulse_start_sfb] + pulse_offset[0]`, pulse i+1 at the previous
/// absolute position plus `pulse_offset[i+1]`. Each offset fits in 5
/// bits and each amp in 4.
#[derive(Clone, Debug)]
struct PulseRecord {
    pulse_start_sfb: u8,
    number_pulse: u8,
    pulse_offset: [u8; MAX_PULSES_ENC],
    pulse_amp: [u8; MAX_PULSES_ENC],
}

#[derive(Clone, Debug)]
struct IcsInfoEnc {
    max_sfb: u8,
    sf_index: u8,
}

fn analyse_and_quantise(spec: &[f32], sf_index: u8) -> Result<Ics> {
    analyse_and_quantise_opts(spec, sf_index, true)
}

fn analyse_and_quantise_opts(spec: &[f32], sf_index: u8, use_tns: bool) -> Result<Ics> {
    let swb = SWB_LONG[sf_index as usize];
    let total_sfb = swb.len() - 1;

    // Compute the highest band carrying significant energy. This sets
    // `max_sfb`; we can stop quantising past it. We use a relative
    // threshold (1/8000 of peak) so trivial leakage from the MDCT
    // doesn't pull `max_sfb` out to the Nyquist boundary on every frame.
    let global_peak = spec.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
    let threshold = (global_peak * 1e-4).max(1e-3);
    let mut max_band_active = 0usize;
    for sfb in 0..total_sfb {
        let start = swb[sfb] as usize;
        let end = swb[sfb + 1] as usize;
        let mx = spec[start..end].iter().fold(0.0f32, |a, &b| a.max(b.abs()));
        if mx > threshold {
            max_band_active = sfb + 1;
        }
    }
    let max_sfb = max_band_active.max(1).min(total_sfb);

    // Optional psychoacoustic-model pass — produces per-band target_max
    // overrides that drive finer quantisation on tonal/loud bands and
    // coarser quantisation on bands hidden under spread masks.
    let psy_target: Option<Vec<i32>> = if psy_active() {
        let sr = SAMPLE_RATES
            .get(sf_index as usize)
            .copied()
            .unwrap_or(44_100);
        let model = PsyModel::new_long(sf_index, sr);
        let analysis = model.analyse(spec);
        Some(analysis.target_max)
    } else {
        None
    };

    // Copy the spectrum into a fixed-size array so the TNS analyser can
    // apply its forward filter in place. If TNS is gated off, `spec_tns`
    // is identical to the input.
    let mut spec_tns = [0.0f32; SPEC_LEN];
    let copy_len = spec.len().min(SPEC_LEN);
    spec_tns[..copy_len].copy_from_slice(&spec[..copy_len]);
    let tns = if use_tns {
        tns_analyse_long(&mut spec_tns, sf_index, max_sfb as u8)
    } else {
        None
    };
    // Work with the TNS-flattened spectrum from this point on. If TNS was
    // not applied, spec_tns == spec.
    let spec: &[f32] = &spec_tns[..];

    // Pick per-band scalefactor so the largest quantised magnitude lands
    // in a useful range. The flat baseline target_max ≈ 7 lets smaller
    // bands use the cheaper books 7-8 (LAV 7) and pushes loud bands
    // onto book 9/10/11.
    //
    // When the psy model is active, target_max becomes per-band: tonal
    // / loud bands get a larger value (finer step → more bits, more
    // fidelity), bands buried under spread masks get a smaller value
    // (coarser step → fewer bits).
    let baseline_target_max = 7i32;
    let mut sfs = vec![0i32; max_sfb];
    let mut q_bands: Vec<Vec<i32>> = Vec::with_capacity(max_sfb);
    for sfb in 0..max_sfb {
        let start = swb[sfb] as usize;
        let end = swb[sfb + 1] as usize;
        let band = &spec[start..end];
        let max_abs = band.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
        if max_abs <= threshold {
            // Zero band.
            sfs[sfb] = 0; // treated as absent — cb=0
            q_bands.push(vec![0i32; end - start]);
            continue;
        }
        let target_max = match psy_target.as_ref().and_then(|t| t.get(sfb).copied()) {
            Some(t) => t.clamp(1, 16),
            None => baseline_target_max,
        };
        // Find the smallest scalefactor that makes ceil((|max|/2^((sf-100)/4))^(3/4))
        // <= target_max. Solve: 2^((sf-100)/4) >= (max_abs / target_max^(4/3))
        // => sf >= 100 + 4 * log2(max_abs / target_max^(4/3))
        let tgt_inv = (target_max as f32).powf(4.0 / 3.0);
        let ratio = max_abs / tgt_inv;
        let sf_f = 100.0 + 4.0 * ratio.log2();
        let mut sf = sf_f.ceil() as i32;
        sf = sf.clamp(0, 255);
        // Quantise with this sf; if any coefficient lands above ESC_LAV,
        // bump sf and retry (rare path).
        let (q, ok) = quantise_band(band, sf);
        if ok {
            sfs[sfb] = sf;
            q_bands.push(q);
        } else {
            let mut sf2 = sf + 1;
            let final_q;
            loop {
                let (q2, ok2) = quantise_band(band, sf2);
                if ok2 || sf2 >= 255 {
                    final_q = q2;
                    break;
                }
                sf2 += 1;
            }
            sfs[sfb] = sf2;
            q_bands.push(final_q);
        }
    }

    // Pick codebook per band. PNS is enabled: `classify_pns_band`
    // gates on (a) peak-to-RMS ≤ 2.8 (noise-like), (b) band length
    // ≥ 4, (c) non-trivial energy, and (d) band center frequency
    // ≥ PNS_MIN_HZ so we never flip tonal LF bands to noise. The
    // emission machinery (codebook 13 section, 9-bit PNS seed, DPCM
    // deltas for subsequent PNS bands, write_spectral_data skip) is
    // all in place.
    let sample_rate = SAMPLE_RATES
        .get(sf_index as usize)
        .copied()
        .unwrap_or(44_100);
    let mut cbs = vec![0u8; max_sfb];
    let pns_off = pns_disabled_by_env();
    for sfb in 0..max_sfb {
        let q = &q_bands[sfb];
        cbs[sfb] = if q.iter().all(|&x| x == 0) {
            0
        } else if !pns_off && pns_eligible_band(swb, sfb, sample_rate) {
            if let Some(pns_sf) = classify_pns_band(spec, swb[sfb] as usize, swb[sfb + 1] as usize)
            {
                sfs[sfb] = pns_sf;
                NOISE_HCB
            } else {
                best_codebook_for_band(q)
            }
        } else {
            best_codebook_for_band(q)
        };
    }

    // Pulse data extraction: pull up to 4 outlier peaks out of bands
    // where a single coefficient sits far above its neighbours. After
    // subtracting the pulse amplitude, the residual band usually slots
    // into a cheaper Huffman codebook, so we re-pick codebooks on the
    // affected bands below.
    let pulse = extract_pulse_record(swb, max_sfb, &mut q_bands, &mut cbs);

    // Re-anchor global_gain now that PNS / IS / pulse have settled:
    // pick the first *regular-codebook* band's sf so g_gain's first
    // delta on decode is zero.
    let mut gg: i32 = 100;
    for sfb in 0..max_sfb {
        let cb = cbs[sfb];
        if cb == 0 || cb == NOISE_HCB || cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
            continue;
        }
        gg = sfs[sfb];
        break;
    }
    let gg_clamped = gg.clamp(0, 255) as u8;

    Ok(Ics {
        info: IcsInfoEnc {
            max_sfb: max_sfb as u8,
            sf_index,
        },
        sfs,
        cbs,
        q_bands,
        global_gain: gg_clamped,
        tns,
        pulse,
    })
}

fn quantise_band(band: &[f32], sf: i32) -> (Vec<i32>, bool) {
    let inv_gain = 2.0f32.powf(-(sf as f32 - 100.0) / 4.0);
    let mut out = Vec::with_capacity(band.len());
    let mut ok = true;
    for &x in band {
        if x == 0.0 {
            out.push(0);
            continue;
        }
        let scaled = x * inv_gain;
        let q_abs = scaled.abs().powf(3.0 / 4.0) + QUANT_MAGIC;
        let q = q_abs.floor() as i32;
        let signed = if scaled < 0.0 { -q } else { q };
        if signed.abs() > 8191 {
            ok = false; // beyond the 13-bit amplitude escape range
        }
        out.push(signed);
    }
    // Also mark failure if max unsigned abs > ESC_LAV and escape is
    // impossible (it's always possible via book 11, but the amplitude
    // field tops out at 13 bits — handled above).
    (out, ok)
}

/// For a given vector of quantised coefficients (length = band size),
/// return the codebook index (1..=11) that minimises total Huffman bits.
/// Encoder-side EightShort ICS. Mirrors [`Ics`] but with arrays sized
/// `num_groups * max_sfb` instead of `max_sfb`. The simplest emission
/// uses `num_groups = 8` with `window_group_length = [1; 8]` — every
/// sub-window is its own group. That produces a slightly-larger
/// bitstream than a grouped encoder but avoids the scale_factor_grouping
/// heuristic.
#[derive(Clone, Debug)]
struct IcsShort {
    sf_index: u8,
    max_sfb: u8,
    num_groups: u8,
    window_group_length: [u8; 8],
    /// `scale_factor_grouping` byte (7 meaningful bits, MSB-first), with
    /// the convention described in `ics::parse_ics_info`.
    scale_factor_grouping: u8,
    /// Per-(group, sfb) scalefactor. Indexed `g * max_sfb + sfb`.
    sfs: Vec<i32>,
    /// Per-(group, sfb) codebook.
    cbs: Vec<u8>,
    /// Per-(group, sfb) quantised coefficient run (length =
    /// `group_len * band_len`, laid out as w=0,..w=group_len-1
    /// concatenated across the band).
    q_bands: Vec<Vec<i32>>,
    global_gain: u8,
    /// Per-sub-window TNS filter (at most one filter per sub-window —
    /// `TNS_MAX_FILT_SHORT` = 1 per §4.6.9.1). `None` for sub-windows
    /// where TNS didn't pass the gain threshold; `Some` ones are
    /// written into the `tns_data()` block as `n_filt = 1`.
    tns: [Option<TnsEncFilter>; 8],
}

/// Analyse a 1024-coefficient EightShort spectrum (laid out as 8
/// contiguous 128-coefficient sub-windows) into an [`IcsShort`] ready
/// for bitstream emission.
///
/// Uses `num_groups = 8` (no grouping) — each sub-window gets its own
/// group for scalefactor purposes.
fn analyse_and_quantise_short(spec: &[f32; 1024], sf_index: u8) -> Result<IcsShort> {
    analyse_and_quantise_short_opts(spec, sf_index, true)
}

fn analyse_and_quantise_short_opts(
    spec: &[f32; 1024],
    sf_index: u8,
    use_tns: bool,
) -> Result<IcsShort> {
    let swb = SWB_SHORT[sf_index as usize];
    let total_sfb = swb.len() - 1;
    const N_WINDOWS: usize = 8;

    // Find the highest active sfb across all sub-windows, using the same
    // relative-threshold rule the long analyser uses.
    let global_peak = spec.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
    let threshold = (global_peak * 1e-4).max(1e-3);
    let mut max_band_active = 0usize;
    for w in 0..N_WINDOWS {
        for sfb in 0..total_sfb {
            let base = w * 128;
            let start = base + swb[sfb] as usize;
            let end = base + swb[sfb + 1] as usize;
            let mx = spec[start..end].iter().fold(0.0f32, |a, &b| a.max(b.abs()));
            if mx > threshold && sfb + 1 > max_band_active {
                max_band_active = sfb + 1;
            }
        }
    }
    // max_sfb is 4 bits → capped at 15.
    let max_sfb = max_band_active.max(1).min(total_sfb).min(15);

    // Run TNS on a mutable copy of the spectrum laid out as
    // `[f32; SPEC_LEN]` so `analyse_short` / the decoder-compat filter
    // runs on matching buffer shapes. Quantisation then proceeds on
    // `spec_tns` — with TNS off, `spec_tns == spec`.
    let mut spec_tns = [0.0f32; SPEC_LEN];
    spec_tns[..1024].copy_from_slice(spec);
    let mut tns: [Option<TnsEncFilter>; 8] = Default::default();
    if use_tns {
        for w in 0..N_WINDOWS {
            tns[w] = crate::tns_analyse::analyse_short(&mut spec_tns, w, sf_index, max_sfb as u8);
        }
    }
    // From here on everything quantises off the TNS-flattened view.
    let spec: &[f32] = &spec_tns[..1024];

    // Quantise each (group, sfb).
    let target_max = 7i32;
    let mut sfs = vec![0i32; N_WINDOWS * max_sfb];
    let mut cbs = vec![0u8; N_WINDOWS * max_sfb];
    let mut q_bands: Vec<Vec<i32>> = Vec::with_capacity(N_WINDOWS * max_sfb);

    for g in 0..N_WINDOWS {
        let base = g * 128;
        for sfb in 0..max_sfb {
            let start = base + swb[sfb] as usize;
            let end = base + swb[sfb + 1] as usize;
            let band = &spec[start..end];
            let max_abs = band.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
            let idx = g * max_sfb + sfb;
            if max_abs <= threshold {
                sfs[idx] = 0;
                q_bands.push(vec![0i32; end - start]);
                continue;
            }
            // Pick scalefactor that keeps |q| ≤ target_max.
            let tgt_inv = (target_max as f32).powf(4.0 / 3.0);
            let ratio = max_abs / tgt_inv;
            let sf_f = 100.0 + 4.0 * ratio.log2();
            let mut sf = sf_f.ceil() as i32;
            sf = sf.clamp(0, 255);
            let (q, ok) = quantise_band(band, sf);
            if ok {
                sfs[idx] = sf;
                q_bands.push(q);
            } else {
                let mut sf2 = sf + 1;
                let final_q;
                loop {
                    let (q2, ok2) = quantise_band(band, sf2);
                    if ok2 || sf2 >= 255 {
                        final_q = q2;
                        break;
                    }
                    sf2 += 1;
                }
                sfs[idx] = sf2;
                q_bands.push(final_q);
            }
        }
    }

    // Codebook per (group, sfb). Short-window codebook selection uses
    // the same "smallest book whose LAV fits" heuristic as long.
    for g in 0..N_WINDOWS {
        for sfb in 0..max_sfb {
            let idx = g * max_sfb + sfb;
            cbs[idx] = if q_bands[idx].iter().all(|&x| x == 0) {
                0
            } else {
                best_codebook_for_band(&q_bands[idx])
            };
        }
    }

    // Anchor global_gain on the first regular-codebook band.
    let mut gg: i32 = 100;
    for g in 0..N_WINDOWS {
        for sfb in 0..max_sfb {
            let idx = g * max_sfb + sfb;
            let cb = cbs[idx];
            if cb != 0 && cb != NOISE_HCB && cb != INTENSITY_HCB && cb != INTENSITY_HCB2 {
                gg = sfs[idx];
                break;
            }
        }
        if gg != 100 {
            break;
        }
    }
    let global_gain = gg.clamp(0, 255) as u8;

    Ok(IcsShort {
        sf_index,
        max_sfb: max_sfb as u8,
        num_groups: N_WINDOWS as u8,
        window_group_length: [1; N_WINDOWS],
        scale_factor_grouping: 0, // all 8 windows are separate groups
        sfs,
        cbs,
        q_bands,
        global_gain,
        tns,
    })
}

/// Minimum center frequency (Hz) above which PNS and intensity-stereo
/// are allowed to fire. Below ~4 kHz, the ear is far more sensitive to
/// phase / tonality and both tools degrade quality; above it, phase
/// information is already irrelevant to the human auditory system, so
/// substituting shaped noise for Huffman-coded tones is a clean win.
const PNS_IS_MIN_HZ: f32 = 4_000.0;

/// Test/debug knob: when the environment variable `OXIDEAV_AAC_DISABLE_PNS`
/// is set (any value), the encoder will not classify any band as `NOISE_HCB`
/// — every non-zero band falls back to a regular Huffman codebook. Used by
/// `tests/encode_pns_savings.rs` to A/B-compare frame sizes with PNS on vs
/// off on a noise-rich fixture, demonstrating the bit savings PNS buys.
/// Default behaviour (env var unset) leaves PNS classification fully
/// active; this knob never changes the bitstream of regular runs.
fn pns_disabled_by_env() -> bool {
    std::env::var("OXIDEAV_AAC_DISABLE_PNS").is_ok()
}

/// True if scalefactor band `sfb` at `sample_rate` sits entirely above
/// `PNS_IS_MIN_HZ`. Used as a hard gate on PNS and intensity-stereo —
/// we never want to flip a tonal LF band to either of those tools.
fn pns_eligible_band(swb: &[u16], sfb: usize, sample_rate: u32) -> bool {
    let nyquist = sample_rate as f32 * 0.5;
    let bins_per_hz = SPEC_LEN as f32 / nyquist;
    let start = swb[sfb] as f32;
    let center_hz = (start + swb[sfb + 1] as f32) * 0.5 / bins_per_hz;
    center_hz >= PNS_IS_MIN_HZ
}

/// Classify a scalefactor band as IS-codable if `R ≈ scale·L` holds to
/// good precision across the band. Returns `(is_position, sign_flip)`
/// where `is_position` encodes the magnitude scale via
/// `|scale| = 2^(-is_position/4)` and `sign_flip` is true when scale is
/// negative (consumed by the decoder as the MS-bit sign repurpose).
///
/// Criteria (conservative):
///  * `||L|| > 1e-3` (otherwise the scale is ill-defined — zero band).
///  * `|<L, R>| / (||L||·||R||) > 0.95` — L and R are near-colinear.
///  * |is_position| ≤ 60 so the SF-Huffman delta fits.
fn classify_is_band(
    l: &[f32],
    r: &[f32],
    band_start: usize,
    band_end: usize,
) -> Option<(i32, bool)> {
    if band_end <= band_start || band_end > l.len() || band_end > r.len() {
        return None;
    }
    let lb = &l[band_start..band_end];
    let rb = &r[band_start..band_end];
    let mut sum_ll = 0.0f32;
    let mut sum_rr = 0.0f32;
    let mut sum_lr = 0.0f32;
    for i in 0..lb.len() {
        sum_ll += lb[i] * lb[i];
        sum_rr += rb[i] * rb[i];
        sum_lr += lb[i] * rb[i];
    }
    if sum_ll < 1e-8 || sum_rr < 1e-8 {
        return None;
    }
    let corr = sum_lr / (sum_ll * sum_rr).sqrt();
    if corr.abs() < 0.95 {
        return None;
    }
    let scale = sum_lr / sum_ll; // least-squares R ≈ scale·L
    let mag = scale.abs();
    if mag < 1e-4 {
        return None;
    }
    let is_pos = (-4.0 * mag.log2()).round() as i32;
    if !(-60..=60).contains(&is_pos) {
        return None;
    }
    Some((is_pos, scale < 0.0))
}

/// Classify a scalefactor band as PNS (noise-like) if it passes all three
/// conservative tests:
///
///  1. Band is long enough (≥ 4 samples) to give a stable energy estimate.
///  2. Peak-to-RMS ratio is ≤ 2.8 — a well-behaved noise band has most
///     samples near its RMS magnitude; a tonal band has a small number
///     of samples much higher than its RMS.
///  3. Band has non-trivial average energy (> 1e-8 per sample) — silent
///     bands stay on codebook 0 where they cost nothing anyway.
///
/// Returns the PNS scalefactor that reproduces the band's energy on
/// decode, or `None` if the band is not a noise candidate.
///
/// The PNS decoder synthesises each spectral line as `uniform(-1, 1) *
/// gain` where `gain = 2^(sf/4 - 14.5)`. The expected energy per line is
/// `gain² / 3` (variance of uniform on [-1, 1)). Invert:
///   `gain = sqrt(3 · energy_per_line)`
///   `sf   = 4·(log₂(gain) + 14.5) = 2·log₂(3·energy_per_line) + 58`.
fn classify_pns_band(spec: &[f32], band_start: usize, band_end: usize) -> Option<i32> {
    let band = &spec[band_start..band_end];
    let len = band.len();
    if len < 4 {
        return None;
    }
    let mut sum_sq = 0.0f32;
    let mut peak = 0.0f32;
    for &x in band {
        sum_sq += x * x;
        let a = x.abs();
        if a > peak {
            peak = a;
        }
    }
    let rms = (sum_sq / len as f32).sqrt();
    if rms < 1e-4 {
        return None;
    }
    let peak_rms = peak / rms;
    if peak_rms > 2.8 {
        return None; // tonal band — leave it on a regular codebook.
    }
    let energy_per_line = sum_sq / len as f32;
    let sf_f = 2.0 * (3.0 * energy_per_line).log2() + 58.0;
    let sf = sf_f.round() as i32;
    Some(sf.clamp(-100, 200))
}

/// Minimum unsigned outlier magnitude (in quantised units) to be worth
/// moving into pulse_data, and minimum peak-to-second gap before we
/// call it an outlier. Values below this aren't worth the header
/// overhead — the surrounding book already encodes them cheaply.
const PULSE_MIN_MAG: i32 = 6;

/// Try to extract 1..=4 outlier coefficients into a single
/// `PulseRecord`. On success, the chosen `q_bands[sfb]` entries have
/// been decremented (preserving sign) so that decoder's pulse-add step
/// reconstructs the original value. The affected bands then get their
/// codebook re-picked so we actually cash in the bit savings.
///
/// Constraints:
///  - pulse_offset is 5-bit (0..=31) and cumulative from the previous
///    pulse's absolute position (or from `swb[pulse_start_sfb]` for the
///    first pulse).
///  - pulse_amp is 4-bit (1..=15).
///  - The amp is capped at `|residual| - 1` so the residual keeps the
///    original sign — the decoder takes the sign from the residual and
///    re-adds amp, so a zero residual would flip a negative peak to
///    positive (phase-inversion disaster on tonal content).
fn extract_pulse_record(
    swb: &[u16],
    max_sfb: usize,
    q_bands: &mut [Vec<i32>],
    cbs: &mut [u8],
) -> Option<PulseRecord> {
    // Candidates: (sfb, local_offset_in_band, signed_q_value).
    let mut candidates: Vec<(usize, u8, i32)> = Vec::new();
    for sfb in 0..max_sfb {
        let cb = cbs[sfb];
        if cb == 0 || cb == NOISE_HCB || cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
            continue;
        }
        let q = &q_bands[sfb];
        if q.is_empty() {
            continue;
        }
        let mut max_i = 0usize;
        let mut max_v = 0i32;
        let mut second_v = 0i32;
        for (i, &v) in q.iter().enumerate() {
            let a = v.abs();
            if a > max_v {
                second_v = max_v;
                max_v = a;
                max_i = i;
            } else if a > second_v {
                second_v = a;
            }
        }
        if max_v < PULSE_MIN_MAG || (max_v - second_v) < PULSE_MIN_MAG {
            continue;
        }
        if max_i >= 32 {
            continue; // doesn't fit in the 5-bit local offset.
        }
        candidates.push((sfb, max_i as u8, q[max_i]));
    }
    if candidates.is_empty() {
        return None;
    }
    candidates.truncate(MAX_PULSES_ENC);
    // Compute absolute positions for each candidate.
    let abs_positions: Vec<usize> = candidates
        .iter()
        .map(|&(sfb, off, _)| swb[sfb] as usize + off as usize)
        .collect();
    // Subtract amplitudes, capped to preserve sign.
    let mut amps = [0u8; MAX_PULSES_ENC];
    for (i, &(sfb, off, v)) in candidates.iter().enumerate() {
        let local = off as usize;
        let band = &mut q_bands[sfb];
        if local >= band.len() {
            continue;
        }
        let max_amp = v.abs() - 1;
        if max_amp <= 0 {
            amps[i] = 0;
            continue;
        }
        let amp = max_amp.min(15);
        amps[i] = amp as u8;
        if v > 0 {
            band[local] = v - amp;
        } else {
            band[local] = v + amp;
        }
    }
    // Compact: drop amp=0 entries and re-encode the cumulative offsets.
    let live: Vec<(usize, u8)> = abs_positions
        .iter()
        .zip(amps.iter())
        .filter(|(_, &a)| a > 0)
        .map(|(&p, &a)| (p, a))
        .collect();
    if live.is_empty() {
        return None;
    }
    let first_abs = live[0].0;
    let mut start_sfb = 0usize;
    while start_sfb + 1 < max_sfb && (first_abs as u16) >= swb[start_sfb + 1] {
        start_sfb += 1;
    }
    let first_offset = first_abs - swb[start_sfb] as usize;
    if first_offset >= 32 || start_sfb > 63 {
        return None;
    }
    let mut pulse_offset = [0u8; MAX_PULSES_ENC];
    let mut pulse_amp = [0u8; MAX_PULSES_ENC];
    pulse_offset[0] = first_offset as u8;
    pulse_amp[0] = live[0].1;
    let mut prev_abs = first_abs;
    let mut count = 1usize;
    for (pos, a) in live.into_iter().skip(1) {
        let delta = pos - prev_abs;
        if delta == 0 || delta >= 32 || count >= MAX_PULSES_ENC {
            continue;
        }
        pulse_offset[count] = delta as u8;
        pulse_amp[count] = a;
        prev_abs = pos;
        count += 1;
    }
    // Re-pick codebooks on every affected (regular-coded) band — the
    // peak just dropped so a cheaper book may fit.
    for sfb in 0..max_sfb {
        let cb = cbs[sfb];
        if cb == 0 || cb == NOISE_HCB || cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
            continue;
        }
        let q = &q_bands[sfb];
        if q.iter().all(|&x| x == 0) {
            cbs[sfb] = 0;
        } else {
            cbs[sfb] = best_codebook_for_band(q);
        }
    }
    Some(PulseRecord {
        pulse_start_sfb: start_sfb as u8,
        number_pulse: count as u8,
        pulse_offset,
        pulse_amp,
    })
}

fn best_codebook_for_band(q: &[i32]) -> u8 {
    let mut best_cb = 11u8;
    let mut best_bits = u64::MAX;
    for cb in 1u8..=11 {
        if let Some(bits) = try_encode_bits(q, cb) {
            if bits < best_bits {
                best_bits = bits;
                best_cb = cb;
            }
        }
    }
    best_cb
}

/// Compute (without writing) the bit cost of encoding `q` under codebook
/// `cb`. Returns `None` if any element exceeds the codebook's LAV with no
/// escape capability.
fn try_encode_bits(q: &[i32], cb: u8) -> Option<u64> {
    let book = encoder_book(cb);
    let dim = book.dim as usize;
    if q.len() % dim != 0 {
        // Bands aren't necessarily multiples of 2/4; this shouldn't
        // happen for AAC-LC (SWB_LONG bands are all multiples of 4),
        // but guard anyway.
        return None;
    }
    let lav = book.lav as i32;
    let mut total_bits = 0u64;
    let mut i = 0;
    while i < q.len() {
        let (idx, extra_bits, ok) = pack_tuple_index(&q[i..i + dim], book, lav);
        if !ok {
            return None;
        }
        total_bits += book.bits[idx] as u64 + extra_bits;
        i += dim;
    }
    Some(total_bits)
}

/// Write the bits for `q` under `cb` to `bw`.
fn write_band_bits(bw: &mut BitWriter, q: &[i32], cb: u8) {
    let book = encoder_book(cb);
    let dim = book.dim as usize;
    let lav = book.lav as i32;
    let mut i = 0;
    while i < q.len() {
        let (idx, _extra_bits, ok) = pack_tuple_index(&q[i..i + dim], book, lav);
        debug_assert!(ok);
        // Huffman codeword.
        bw.write_u32(book.codes[idx] as u32, book.bits[idx] as u32);
        // Unsigned books: append sign bits for non-zero coefficients.
        if !book.signed {
            for &v in &q[i..i + dim] {
                if v != 0 {
                    bw.write_bit(v < 0);
                }
            }
        }
        // Book 11 escape.
        if book.escape {
            for &v in &q[i..i + dim] {
                if v.abs() >= ESC_LAV {
                    write_escape_amp(bw, v.unsigned_abs());
                }
            }
        }
        i += dim;
    }
}

/// Compute the Huffman symbol index for a tuple of `dim` coefficients
/// under `book`. For escape books (11), clamp to ±16 in the index; the
/// caller emits the escape amplitude separately.
///
/// Returns (index, extra-bits-needed-for-escape-amp, ok).
fn pack_tuple_index(tuple: &[i32], book: &EncBook, lav: i32) -> (usize, u64, bool) {
    let dim = book.dim as usize;
    if book.signed {
        // Digits in [-lav, lav], 2*lav+1 possibilities per position.
        let modulo = 2 * lav + 1;
        let mut idx = 0i32;
        for &v in &tuple[..dim] {
            if v < -lav || v > lav {
                return (0, 0, false);
            }
            idx = idx * modulo + (v + lav);
        }
        (idx as usize, 0, true)
    } else {
        // Unsigned: digits in [0, lav]; sign is carried separately.
        let modulo = lav + 1;
        let mut idx = 0i32;
        let mut extra = 0u64;
        for &v in &tuple[..dim] {
            let mut a = v.abs();
            if book.escape {
                if a >= ESC_LAV {
                    extra += escape_amp_bits(a as u32) as u64;
                    a = ESC_LAV;
                }
            } else if a > lav {
                return (0, 0, false);
            }
            idx = idx * modulo + a;
        }
        (idx as usize, extra, true)
    }
}

/// Number of bits used by the escape amplitude code for value `a`. The
/// escape code is a unary-prefix `1..1 0` of length `prefix` ones plus a
/// terminating zero, followed by `prefix + 4` raw bits. For an amplitude
/// `a`, `prefix = floor(log2(a)) - 4`.
fn escape_amp_bits(a: u32) -> u32 {
    // a must be >= 16 (= ESC_LAV).
    let top = 31 - a.leading_zeros(); // floor(log2(a))
    let prefix = top.saturating_sub(4);
    // unary prefix (prefix ones) + terminator zero (1 bit) + prefix+4 raw bits
    prefix + 1 + prefix + 4
}

/// Emit the escape-amplitude code for absolute value `a` (expects a >= 16).
fn write_escape_amp(bw: &mut BitWriter, a: u32) {
    let top = 31 - a.leading_zeros();
    let prefix = top.saturating_sub(4);
    for _ in 0..prefix {
        bw.write_bit(true);
    }
    bw.write_bit(false);
    let raw = a & ((1u32 << (prefix + 4)) - 1);
    bw.write_u32(raw, prefix + 4);
}

// ==================== ICS bitstream writers ====================

/// Write ics_info for any long-style window sequence (OnlyLong, LongStart,
/// LongStop). EightShort uses [`write_ics_info_short`] which has a
/// different layout (4-bit max_sfb + 7-bit scale_factor_grouping).
fn write_ics_info(bw: &mut BitWriter, info: &IcsInfoEnc, seq: WindowSequence) {
    debug_assert!(!matches!(seq, WindowSequence::EightShort));
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(seq as u32, 2); // window_sequence
    bw.write_u32(0, 1); // window_shape = sine
    bw.write_u32(info.max_sfb as u32, 6);
    bw.write_bit(false); // predictor_data_present
}

/// Write the full SCE individual_channel_stream payload. The SCE caller
/// has already emitted element_instance_tag (4 bits). Layout:
///   global_gain (8) | ics_info | body
fn write_ics(bw: &mut BitWriter, ics: &Ics, seq: WindowSequence, _in_cpe: bool) -> Result<()> {
    bw.write_u32(ics.global_gain as u32, 8);
    write_ics_info(bw, &ics.info, seq);
    write_ics_body_no_global_gain(bw, ics)
}

/// Write per-channel CPE body (used inside a CPE with common_window=1).
/// Layout (per spec individual_channel_stream when ics_info is shared):
///   global_gain (8) | section_data | scale_factor_data |
///   pulse_data_present (1) | tns_data_present (1) | gain_control_present (1) |
///   spectral_data.
fn write_ics_body(bw: &mut BitWriter, ics: &Ics) -> Result<()> {
    bw.write_u32(ics.global_gain as u32, 8);
    write_ics_body_no_global_gain(bw, ics)
}

fn write_ics_body_no_global_gain(bw: &mut BitWriter, ics: &Ics) -> Result<()> {
    let _ = ics.info.sf_index;
    write_section_data(bw, ics);
    write_scalefactors(bw, ics)?;
    let pulse_present = ics.pulse.is_some();
    bw.write_bit(pulse_present);
    if let Some(ref pd) = ics.pulse {
        write_pulse_data(bw, pd);
    }
    let tns_present = ics.tns.is_some();
    bw.write_bit(tns_present);
    if let Some(ref f) = ics.tns {
        write_tns_data_long(bw, f);
    }
    bw.write_bit(false); // gain_control_data_present
    write_spectral_data(bw, ics);
    Ok(())
}

/// Serialise `pulse_data()` per §4.6.5. Matches the bit layout consumed
/// by `pulse::parse_pulse_data`.
fn write_pulse_data(bw: &mut BitWriter, pd: &PulseRecord) {
    let n = pd.number_pulse.clamp(1, MAX_PULSES_ENC as u8);
    bw.write_u32((n - 1) as u32, 2);
    bw.write_u32(pd.pulse_start_sfb as u32, 6);
    for i in 0..n as usize {
        bw.write_u32(pd.pulse_offset[i] as u32, 5);
        bw.write_u32(pd.pulse_amp[i] as u32, 4);
    }
}

/// Serialise `tns_data()` for a single long window with one filter. Matches
/// the bit layout consumed by `tns::parse_tns_data` — see `src/tns.rs`.
fn write_tns_data_long(bw: &mut BitWriter, filt: &TnsEncFilter) {
    // Long window: n_filt is 2 bits.
    bw.write_u32(1, 2); // n_filt = 1
    bw.write_u32(crate::tns_analyse::TNS_ENC_COEF_RES as u32, 1); // coef_res
                                                                  // length (6 bits) + order (5 bits).
    bw.write_u32(filt.length_sfb as u32, 6);
    bw.write_u32(filt.order as u32, 5);
    if filt.order > 0 {
        bw.write_u32(filt.direction as u32, 1);
        bw.write_u32(filt.coef_compress as u32, 1);
        let coef_bits: u32 =
            3 + crate::tns_analyse::TNS_ENC_COEF_RES as u32 - filt.coef_compress as u32;
        for o in 0..filt.order as usize {
            bw.write_u32(
                filt.coef_raw[o] as u32 & ((1u32 << coef_bits) - 1),
                coef_bits,
            );
        }
    }
}

fn write_section_data(bw: &mut BitWriter, ics: &Ics) {
    let max_sfb = ics.info.max_sfb as usize;
    if max_sfb == 0 {
        return;
    }
    // Long-only: sect_bits = 5, sect_esc_val = 31.
    let sect_bits: u32 = 5;
    let sect_esc_val: u32 = (1 << sect_bits) - 1;

    let mut k = 0usize;
    while k < max_sfb {
        let cb = ics.cbs[k];
        let mut run = 1usize;
        while k + run < max_sfb && ics.cbs[k + run] == cb {
            run += 1;
        }
        bw.write_u32(cb as u32, 4);
        // Write the length in `sect_bits` chunks; each chunk == sect_esc_val
        // means "more bits follow", and the terminating chunk is < sect_esc_val.
        let mut remaining = run as u32;
        while remaining >= sect_esc_val {
            bw.write_u32(sect_esc_val, sect_bits);
            remaining -= sect_esc_val;
        }
        bw.write_u32(remaining, sect_bits);
        k += run;
    }
}

fn write_scalefactors(bw: &mut BitWriter, ics: &Ics) -> Result<()> {
    let max_sfb = ics.info.max_sfb as usize;
    // Walk bands in decode order; for each non-ZERO band emit the
    // scalefactor delta via the scalefactor Huffman codebook. The
    // decoder seeds the three accumulators as:
    //   g_gain  = global_gain        (regular-codebook bands)
    //   g_noise = global_gain - 90   (NOISE_HCB bands)
    //   g_is    = 0                  (INTENSITY_HCB / INTENSITY_HCB2 bands)
    // We emit deltas against the appropriate accumulator, and the first
    // PNS band uses a 9-bit raw `dpcm_noise_nrg` seed instead of a SF-
    // Huffman delta.
    let mut cur: i32 = ics.global_gain as i32;
    let mut g_noise: i32 = ics.global_gain as i32 - 90;
    let mut g_is: i32 = 0;
    let mut noise_seed_emitted = false;
    for sfb in 0..max_sfb {
        let cb = ics.cbs[sfb];
        if cb == 0 {
            continue;
        }
        let target = ics.sfs[sfb];
        if cb == NOISE_HCB {
            if !noise_seed_emitted {
                // 9-bit raw `dpcm_noise_nrg`: g_noise_after = g_noise_before + raw - 256.
                //   raw = target - (global_gain - 90) + 256
                //       = target - global_gain + 346
                let raw = target - ics.global_gain as i32 + 346;
                let raw_c = raw.clamp(0, 511);
                bw.write_u32(raw_c as u32, 9);
                g_noise = (ics.global_gain as i32 - 90) + (raw_c - 256);
                noise_seed_emitted = true;
            } else {
                let delta = (target - g_noise).clamp(-60, 60);
                g_noise += delta;
                let idx = (delta + 60) as usize;
                bw.write_u32(SCALEFACTOR_CODES[idx] as u32, SCALEFACTOR_BITS[idx] as u32);
            }
        } else if cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
            let delta = (target - g_is).clamp(-60, 60);
            g_is += delta;
            let idx = (delta + 60) as usize;
            bw.write_u32(SCALEFACTOR_CODES[idx] as u32, SCALEFACTOR_BITS[idx] as u32);
        } else {
            let delta = (target - cur).clamp(-60, 60);
            cur += delta;
            let idx = (delta + 60) as usize;
            bw.write_u32(SCALEFACTOR_CODES[idx] as u32, SCALEFACTOR_BITS[idx] as u32);
        }
    }
    Ok(())
}

// ==================== EightShort emission helpers ====================

/// Write ics_info for an EightShort ICS. Layout per §4.6.11 /
/// `parse_ics_info`:
///
///   reserved(1=0) | window_sequence(2=2) | window_shape(1)
///   | max_sfb(4)   | scale_factor_grouping(7)
fn write_ics_info_short(bw: &mut BitWriter, ics: &IcsShort) {
    bw.write_u32(0, 1); // ics_reserved_bit
    bw.write_u32(2, 2); // window_sequence = EightShort
    bw.write_u32(0, 1); // window_shape = 0 (sine) — matches the short MDCT path
    bw.write_u32(ics.max_sfb as u32, 4);
    bw.write_u32(ics.scale_factor_grouping as u32, 7);
}

/// Write section_data for an EightShort ICS. `sect_bits = 3`,
/// `sect_esc_val = 7`. Sections are independent per group.
fn write_section_data_short(bw: &mut BitWriter, ics: &IcsShort) {
    let max_sfb = ics.max_sfb as usize;
    let groups = ics.num_groups as usize;
    if max_sfb == 0 {
        return;
    }
    let sect_bits: u32 = 3;
    let sect_esc_val: u32 = (1 << sect_bits) - 1;
    for g in 0..groups {
        let base = g * max_sfb;
        let mut k = 0usize;
        while k < max_sfb {
            let cb = ics.cbs[base + k];
            let mut run = 1usize;
            while k + run < max_sfb && ics.cbs[base + k + run] == cb {
                run += 1;
            }
            bw.write_u32(cb as u32, 4);
            let mut remaining = run as u32;
            while remaining >= sect_esc_val {
                bw.write_u32(sect_esc_val, sect_bits);
                remaining -= sect_esc_val;
            }
            bw.write_u32(remaining, sect_bits);
            k += run;
        }
    }
}

/// Write grouped scalefactors for an EightShort ICS. Walks (g, sfb) in
/// the same order the decoder's `parse_scalefactors` consumes.
fn write_scalefactors_short(bw: &mut BitWriter, ics: &IcsShort) -> Result<()> {
    let max_sfb = ics.max_sfb as usize;
    let groups = ics.num_groups as usize;
    let mut cur: i32 = ics.global_gain as i32;
    let mut g_noise: i32 = ics.global_gain as i32 - 90;
    let mut g_is: i32 = 0;
    let mut noise_seed_emitted = false;
    for g in 0..groups {
        for sfb in 0..max_sfb {
            let idx = g * max_sfb + sfb;
            let cb = ics.cbs[idx];
            if cb == 0 {
                continue;
            }
            let target = ics.sfs[idx];
            if cb == NOISE_HCB {
                if !noise_seed_emitted {
                    let raw = target - ics.global_gain as i32 + 346;
                    let raw_c = raw.clamp(0, 511);
                    bw.write_u32(raw_c as u32, 9);
                    g_noise = (ics.global_gain as i32 - 90) + (raw_c - 256);
                    noise_seed_emitted = true;
                } else {
                    let delta = (target - g_noise).clamp(-60, 60);
                    g_noise += delta;
                    let i = (delta + 60) as usize;
                    bw.write_u32(SCALEFACTOR_CODES[i] as u32, SCALEFACTOR_BITS[i] as u32);
                }
            } else if cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
                let delta = (target - g_is).clamp(-60, 60);
                g_is += delta;
                let i = (delta + 60) as usize;
                bw.write_u32(SCALEFACTOR_CODES[i] as u32, SCALEFACTOR_BITS[i] as u32);
            } else {
                let delta = (target - cur).clamp(-60, 60);
                cur += delta;
                let i = (delta + 60) as usize;
                bw.write_u32(SCALEFACTOR_CODES[i] as u32, SCALEFACTOR_BITS[i] as u32);
            }
        }
    }
    Ok(())
}

/// Write grouped spectral data for an EightShort ICS. Each (group, sfb)
/// emits its `q_bands` entry via the band's codebook; zero / PNS / IS
/// codebooks carry no coefficient bits.
fn write_spectral_data_short(bw: &mut BitWriter, ics: &IcsShort) {
    let max_sfb = ics.max_sfb as usize;
    let groups = ics.num_groups as usize;
    for g in 0..groups {
        for sfb in 0..max_sfb {
            let idx = g * max_sfb + sfb;
            let cb = ics.cbs[idx];
            if cb == 0 || cb == NOISE_HCB || cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
                continue;
            }
            write_band_bits(bw, &ics.q_bands[idx], cb);
        }
    }
}

/// Serialise `tns_data()` for an EightShort ICS — one 1-bit `n_filt`
/// flag per sub-window, then (if present) the single short-window
/// filter per §4.6.9.1. Bit layout is the dual of
/// [`crate::tns::parse_tns_data`]'s EightShort branch.
fn write_tns_data_short(bw: &mut BitWriter, tns: &[Option<TnsEncFilter>; 8]) {
    for filt_opt in tns.iter() {
        match filt_opt {
            None => bw.write_u32(0, 1), // n_filt = 0
            Some(filt) => {
                bw.write_u32(1, 1); // n_filt = 1
                                    // coef_res — once per window before the filter list.
                bw.write_u32(crate::tns_analyse::TNS_ENC_COEF_RES as u32, 1);
                // length (4 bits) + order (3 bits) per filter.
                bw.write_u32(filt.length_sfb as u32, 4);
                bw.write_u32(filt.order as u32, 3);
                if filt.order > 0 {
                    bw.write_u32(filt.direction as u32, 1);
                    bw.write_u32(filt.coef_compress as u32, 1);
                    let coef_bits: u32 =
                        3 + crate::tns_analyse::TNS_ENC_COEF_RES as u32 - filt.coef_compress as u32;
                    for o in 0..filt.order as usize {
                        bw.write_u32(
                            filt.coef_raw[o] as u32 & ((1u32 << coef_bits) - 1),
                            coef_bits,
                        );
                    }
                }
            }
        }
    }
}

/// Write an entire SCE-style EightShort individual_channel_stream:
/// global_gain + ics_info + section + scalefactors + pulse=0 + tns +
/// gain_control=0 + spectral_data. Used for both SCE and LFE elements.
#[allow(dead_code)]
fn write_single_ics_short(bw: &mut BitWriter, ics: &IcsShort) -> Result<()> {
    bw.write_u32(ics.global_gain as u32, 8);
    write_ics_info_short(bw, ics);
    write_section_data_short(bw, ics);
    write_scalefactors_short(bw, ics)?;
    bw.write_u32(0, 1); // pulse_data_present — always 0 for short (spec forbids it)
    let tns_present = ics.tns.iter().any(|f| f.is_some());
    bw.write_u32(tns_present as u32, 1);
    if tns_present {
        write_tns_data_short(bw, &ics.tns);
    }
    bw.write_u32(0, 1); // gain_control_data_present
    write_spectral_data_short(bw, ics);
    Ok(())
}

fn write_spectral_data(bw: &mut BitWriter, ics: &Ics) {
    let max_sfb = ics.info.max_sfb as usize;
    for sfb in 0..max_sfb {
        let cb = ics.cbs[sfb];
        // Skip bands whose codebook doesn't carry coefficient bits:
        //   * cb 0       → zero band.
        //   * cb 13      → NOISE (PNS) — decoder synthesises the band.
        //   * cb 14 / 15 → INTENSITY — decoder derives it from ch 0.
        if cb == 0 || cb == NOISE_HCB || cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
            continue;
        }
        write_band_bits(bw, &ics.q_bands[sfb], cb);
    }
}

// ==================== CPE analysis ====================

/// Choose M/S stereo per band and return per-channel ICS.
///
/// For common-window CPE both channels MUST share the same ics_info
/// (window_sequence, max_sfb, etc.). We therefore pad both per-channel
/// ICS structures to a single unified max_sfb after analysis.
fn analyse_cpe(l: &[f32], r: &[f32], sf_index: u8) -> Result<(Vec<bool>, Ics, Ics)> {
    // Quantise L/R and M/S independently; pick the cheaper one per band.
    // NOTE: TNS is disabled for CPE in this first-cut encoder because a
    // single TNS filter must span the whole spectrum, while per-band M/S
    // decisions can take quantised coefficients from multiple source ICSs.
    // Re-enabling TNS here requires running analysis on L/R directly, then
    // computing M/S from the flattened coefficients and picking per-band —
    // left as future work.
    let ics_l_alone = analyse_and_quantise_opts(l, sf_index, false)?;
    let ics_r_alone = analyse_and_quantise_opts(r, sf_index, false)?;
    let mut m = vec![0.0f32; l.len()];
    let mut s = vec![0.0f32; l.len()];
    for i in 0..l.len() {
        m[i] = (l[i] + r[i]) * 0.5;
        s[i] = (l[i] - r[i]) * 0.5;
    }
    let ics_m = analyse_and_quantise_opts(&m, sf_index, false)?;
    let ics_s = analyse_and_quantise_opts(&s, sf_index, false)?;

    let max_sfb_lr = ics_l_alone.info.max_sfb.max(ics_r_alone.info.max_sfb);
    let max_sfb_ms = ics_m.info.max_sfb.max(ics_s.info.max_sfb);
    let max_sfb = max_sfb_lr.max(max_sfb_ms) as usize;

    let cost_lr: Vec<u64> = (0..max_sfb)
        .map(|sfb| band_bit_cost(sfb, &ics_l_alone) + band_bit_cost(sfb, &ics_r_alone))
        .collect();
    let cost_ms: Vec<u64> = (0..max_sfb)
        .map(|sfb| band_bit_cost(sfb, &ics_m) + band_bit_cost(sfb, &ics_s))
        .collect();
    // Per-band decision: pick the cheapest representation.
    //
    //   LR:   ch0 = L quantised, ch1 = R quantised.
    //   MS:   ch0 = M quantised, ch1 = S quantised.
    //   IS:   ch0 = L quantised, ch1 is synthesised on decode as
    //         sign·2^(-is_position/4)·spec[L], so it only costs a SF-
    //         Huffman delta plus the 4-bit codebook in section data.
    //
    // IS eligibility (`classify_is_band`) requires a strong L↔R correlation
    // in the band; we also gate on the band centre sitting above
    // `PNS_IS_MIN_HZ` so a tonal LF image doesn't get IS-swapped. The
    // cost model charges ~10 bits of overhead for the SF-Huffman delta
    // plus the 4-bit section codebook.
    let sample_rate = SAMPLE_RATES
        .get(sf_index as usize)
        .copied()
        .unwrap_or(44_100);
    let swb = SWB_LONG[sf_index as usize];
    let mut ms_used = vec![false; max_sfb];
    let mut is_used = vec![false; max_sfb];
    let mut is_sign = vec![false; max_sfb]; // repurposed ms_used bit for IS
    let mut is_position = vec![0i32; max_sfb];

    for sfb in 0..max_sfb {
        let mut best_cost = cost_lr[sfb];
        let mut choice: u8 = 0; // 0 = LR, 1 = MS, 2 = IS
        if cost_ms[sfb] < best_cost {
            best_cost = cost_ms[sfb];
            choice = 1;
        }
        if pns_eligible_band(swb, sfb, sample_rate) {
            if let Some((pos, sign)) =
                classify_is_band(l, r, swb[sfb] as usize, swb[sfb + 1] as usize)
            {
                let is_cost = band_bit_cost(sfb, &ics_l_alone).saturating_add(10);
                if is_cost < best_cost {
                    best_cost = is_cost;
                    choice = 2;
                    is_position[sfb] = pos;
                    is_sign[sfb] = sign;
                }
            }
        }
        let _ = best_cost;
        match choice {
            0 => {}
            1 => ms_used[sfb] = true,
            2 => {
                is_used[sfb] = true;
                // ms_used on an IS band is repurposed as the sign flip.
                if is_sign[sfb] {
                    ms_used[sfb] = true;
                }
            }
            _ => unreachable!(),
        }
    }

    let mut ch0 = empty_ics(max_sfb, sf_index);
    let mut ch1 = empty_ics(max_sfb, sf_index);
    for sfb in 0..max_sfb {
        if is_used[sfb] {
            // Channel 0 keeps the L spectrum; channel 1 is IS-coded.
            copy_band(&mut ch0, sfb, &ics_l_alone, sfb);
            let band_len = (swb[sfb + 1] - swb[sfb]) as usize;
            ch1.cbs[sfb] = INTENSITY_HCB;
            ch1.sfs[sfb] = is_position[sfb];
            ch1.q_bands[sfb] = vec![0i32; band_len];
        } else if ms_used[sfb] {
            copy_band(&mut ch0, sfb, &ics_m, sfb);
            copy_band(&mut ch1, sfb, &ics_s, sfb);
        } else {
            copy_band(&mut ch0, sfb, &ics_l_alone, sfb);
            copy_band(&mut ch1, sfb, &ics_r_alone, sfb);
        }
    }
    // Common-window CPE: both channels MUST share ics_info.max_sfb.
    // Anchor global_gain on each channel without trimming max_sfb.
    finalize_ics_keep_max_sfb(&mut ch0);
    finalize_ics_keep_max_sfb(&mut ch1);

    Ok((ms_used, ch0, ch1))
}

fn band_bit_cost(sfb: usize, ics: &Ics) -> u64 {
    if sfb >= ics.info.max_sfb as usize {
        return 0;
    }
    let cb = ics.cbs[sfb];
    if cb == 0 {
        return 0;
    }
    if cb == NOISE_HCB || cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
        // PNS / IS bands carry no Huffman coefficients — the only cost
        // is the SF-Huffman delta written in `write_scalefactors` (~6
        // bits average for a small delta, plus the 4-bit section-data
        // codebook). Use 10 bits flat as a stand-in.
        return 10;
    }
    try_encode_bits(&ics.q_bands[sfb], cb).unwrap_or(u64::MAX / 4)
}

fn empty_ics(max_sfb: usize, sf_index: u8) -> Ics {
    let swb = SWB_LONG[sf_index as usize];
    let mut q_bands = Vec::with_capacity(max_sfb);
    for sfb in 0..max_sfb {
        let len = (swb[sfb + 1] - swb[sfb]) as usize;
        q_bands.push(vec![0i32; len]);
    }
    Ics {
        info: IcsInfoEnc {
            max_sfb: max_sfb as u8,
            sf_index,
        },
        sfs: vec![0; max_sfb],
        cbs: vec![0; max_sfb],
        q_bands,
        global_gain: 100,
        tns: None,
        pulse: None,
    }
}

fn copy_band(dst: &mut Ics, dst_sfb: usize, src: &Ics, src_sfb: usize) {
    if src_sfb >= src.info.max_sfb as usize {
        // Zero band — leave dst defaults.
        return;
    }
    dst.sfs[dst_sfb] = src.sfs[src_sfb];
    dst.cbs[dst_sfb] = src.cbs[src_sfb];
    dst.q_bands[dst_sfb] = src.q_bands[src_sfb].clone();
}

fn finalize_ics(ics: &mut Ics) {
    // Trim trailing zero bands from max_sfb.
    let mut max_sfb = ics.info.max_sfb as usize;
    while max_sfb > 0 && ics.cbs[max_sfb - 1] == 0 {
        max_sfb -= 1;
    }
    ics.info.max_sfb = max_sfb as u8;
    ics.sfs.truncate(max_sfb);
    ics.cbs.truncate(max_sfb);
    ics.q_bands.truncate(max_sfb);
    finalize_ics_keep_max_sfb(ics);
}

/// Pick global_gain (= first non-zero band's scalefactor) without trimming
/// `max_sfb` — used for CPE common-window where both channels must share
/// the same band count.
fn finalize_ics_keep_max_sfb(ics: &mut Ics) {
    let mut gg = 100i32;
    for sfb in 0..ics.info.max_sfb as usize {
        if ics.cbs[sfb] != 0 {
            gg = ics.sfs[sfb];
            break;
        }
    }
    ics.global_gain = gg.clamp(0, 255) as u8;
}

// ==================== Encoder-side Huffman helpers ====================

struct EncBook {
    dim: u8,
    lav: u8,
    signed: bool,
    escape: bool,
    codes: &'static [u16],
    bits: &'static [u8],
}

static BOOK1_E: EncBook = EncBook {
    dim: 4,
    lav: 1,
    signed: true,
    escape: false,
    codes: BOOK1_CODES,
    bits: BOOK1_BITS,
};
static BOOK2_E: EncBook = EncBook {
    dim: 4,
    lav: 1,
    signed: true,
    escape: false,
    codes: BOOK2_CODES,
    bits: BOOK2_BITS,
};
static BOOK3_E: EncBook = EncBook {
    dim: 4,
    lav: 2,
    signed: false,
    escape: false,
    codes: BOOK3_CODES,
    bits: BOOK3_BITS,
};
static BOOK4_E: EncBook = EncBook {
    dim: 4,
    lav: 2,
    signed: false,
    escape: false,
    codes: BOOK4_CODES,
    bits: BOOK4_BITS,
};
static BOOK5_E: EncBook = EncBook {
    dim: 2,
    lav: 4,
    signed: true,
    escape: false,
    codes: BOOK5_CODES,
    bits: BOOK5_BITS,
};
static BOOK6_E: EncBook = EncBook {
    dim: 2,
    lav: 4,
    signed: true,
    escape: false,
    codes: BOOK6_CODES,
    bits: BOOK6_BITS,
};
static BOOK7_E: EncBook = EncBook {
    dim: 2,
    lav: 7,
    signed: false,
    escape: false,
    codes: BOOK7_CODES,
    bits: BOOK7_BITS,
};
static BOOK8_E: EncBook = EncBook {
    dim: 2,
    lav: 7,
    signed: false,
    escape: false,
    codes: BOOK8_CODES,
    bits: BOOK8_BITS,
};
static BOOK9_E: EncBook = EncBook {
    dim: 2,
    lav: 12,
    signed: false,
    escape: false,
    codes: BOOK9_CODES,
    bits: BOOK9_BITS,
};
static BOOK10_E: EncBook = EncBook {
    dim: 2,
    lav: 12,
    signed: false,
    escape: false,
    codes: BOOK10_CODES,
    bits: BOOK10_BITS,
};
static BOOK11_E: EncBook = EncBook {
    dim: 2,
    lav: 16,
    signed: false,
    escape: true,
    codes: BOOK11_CODES,
    bits: BOOK11_BITS,
};

fn encoder_book(cb: u8) -> &'static EncBook {
    match cb {
        1 => &BOOK1_E,
        2 => &BOOK2_E,
        3 => &BOOK3_E,
        4 => &BOOK4_E,
        5 => &BOOK5_E,
        6 => &BOOK6_E,
        7 => &BOOK7_E,
        8 => &BOOK8_E,
        9 => &BOOK9_E,
        10 => &BOOK10_E,
        11 => &BOOK11_E,
        _ => panic!("encoder_book: invalid codebook {cb}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode an EightShort SCE frame using the short-window helpers,
    /// wrap it in an ADTS header, and confirm the existing AAC-LC
    /// decoder parses it end-to-end without error. This validates the
    /// bit-level contract between write_single_ics_short and the
    /// decoder's ics/section/scalefactor/spectrum paths.
    #[test]
    fn short_ics_roundtrip_through_decoder() {
        use crate::adts::parse_adts_header;
        use crate::syntax::ElementType;
        #[allow(unused_imports)]
        use oxideav_core::Decoder;
        use oxideav_core::{CodecId, CodecParameters, Packet, TimeBase};

        // Build a spectrum with content in sub-windows 2 and 5.
        let mut spec = [0.0f32; 1024];
        for k in 2 * 128..2 * 128 + 32 {
            spec[k] = 200.0;
        }
        for k in 5 * 128..5 * 128 + 16 {
            spec[k] = -300.0;
        }
        let sf_index = 4u8; // 44.1 kHz
        let ics = analyse_and_quantise_short(&spec, sf_index).unwrap();

        let mut bw = BitWriter::new();
        bw.write_u32(ElementType::Sce as u32, 3);
        bw.write_u32(0, 4); // instance_tag
        write_single_ics_short(&mut bw, &ics).unwrap();
        bw.write_u32(ElementType::End as u32, 3);
        bw.align_to_byte();
        let payload = bw.finish();

        let adts = build_adts_frame(sf_index, 1, payload.len());
        let mut frame = adts;
        frame.extend_from_slice(&payload);

        // Sanity: ADTS header parses cleanly.
        let hdr = parse_adts_header(&frame).expect("ADTS parse");
        assert_eq!(hdr.frame_length, frame.len());

        // Feed to decoder.
        let mut params = CodecParameters::audio(CodecId::new("aac"));
        params.sample_rate = Some(44_100);
        params.channels = Some(1);
        let mut dec = crate::decoder::make_decoder(&params).expect("decoder");
        let tb = TimeBase::new(1, 44_100);
        let pkt = Packet::new(0, tb, frame);
        dec.send_packet(&pkt).expect("send_packet");
        match dec.receive_frame() {
            Ok(oxideav_core::Frame::Audio(af)) => {
                assert_eq!(af.samples, 1024);
            }
            Ok(other) => panic!("unexpected frame variant: {other:?}"),
            Err(e) => panic!("decoder rejected EightShort SCE: {e}"),
        }
    }

    /// A highly-correlated per-sub-window spectrum triggers TNS; the
    /// emitted bitstream carries `tns_data_present = 1` and parses
    /// through the decoder end-to-end. This is the positive-path
    /// counterpart to `short_ics_roundtrip_through_decoder` (which
    /// uses a sparse spectrum that does not trigger TNS).
    #[test]
    fn short_tns_roundtrip_through_decoder() {
        use crate::adts::parse_adts_header;
        use crate::syntax::ElementType;
        use oxideav_core::{CodecId, CodecParameters, Packet, TimeBase};

        // Build a spectrum where each 128-coef sub-window carries a
        // smoothly-correlated envelope (envelope's AR structure is
        // what makes Levinson find a useful predictor). Use a strong
        // first-order IIR-like decay so TNS can fit an order-4 model.
        let sf_index = 4u8; // 44.1 kHz
        let mut spec = [0.0f32; 1024];
        for w in 0..8 {
            let base = w * 128;
            let mut cur = 800.0f32;
            for k in 0..128 {
                spec[base + k] = cur;
                // Fake AR(1)-ish decay with small sign flips so the
                // coefficients span a real dynamic range.
                cur = cur * 0.92 - 60.0 * if k & 7 == 0 { 1.0 } else { 0.0 };
            }
        }
        let ics = analyse_and_quantise_short(&spec, sf_index).unwrap();
        // At least one sub-window should have a TNS filter attached.
        let tns_count = ics.tns.iter().filter(|f| f.is_some()).count();
        assert!(
            tns_count > 0,
            "AR-correlated spectrum should trigger TNS in ≥ 1 sub-window",
        );

        // Build the raw_data_block and wrap it in ADTS.
        let mut bw = BitWriter::new();
        bw.write_u32(ElementType::Sce as u32, 3);
        bw.write_u32(0, 4);
        write_single_ics_short(&mut bw, &ics).unwrap();
        bw.write_u32(ElementType::End as u32, 3);
        bw.align_to_byte();
        let payload = bw.finish();
        let adts = build_adts_frame(sf_index, 1, payload.len());
        let mut frame = adts;
        frame.extend_from_slice(&payload);

        // Sanity: ADTS parses cleanly.
        let hdr = parse_adts_header(&frame).expect("ADTS parse");
        assert_eq!(hdr.frame_length, frame.len());

        // Decoder round-trip: must accept the frame without error.
        let mut params = CodecParameters::audio(CodecId::new("aac"));
        params.sample_rate = Some(44_100);
        params.channels = Some(1);
        let mut dec = crate::decoder::make_decoder(&params).expect("decoder");
        let tb = TimeBase::new(1, 44_100);
        let pkt = Packet::new(0, tb, frame);
        dec.send_packet(&pkt).expect("send_packet");
        match dec.receive_frame() {
            Ok(oxideav_core::Frame::Audio(af)) => {
                assert_eq!(af.samples, 1024);
            }
            Ok(other) => panic!("unexpected frame variant: {other:?}"),
            Err(e) => panic!("decoder rejected TNS-present EightShort SCE: {e}"),
        }
    }

    #[test]
    fn analyse_short_silent_spectrum() {
        let spec = [0.0f32; 1024];
        let ics = analyse_and_quantise_short(&spec, 4).unwrap();
        assert_eq!(ics.num_groups, 8);
        assert_eq!(ics.window_group_length, [1; 8]);
        assert_eq!(ics.scale_factor_grouping, 0);
        assert_eq!(ics.max_sfb, 1); // clamped to ≥ 1 by analyser
                                    // Every group+sfb should be a zero band (cb = 0).
        for &cb in ics.cbs.iter() {
            assert_eq!(cb, 0);
        }
    }

    #[test]
    fn analyse_short_tone_in_one_subwindow() {
        // Place a concentrated tone in sub-window 3; expect max_sfb > 1 and
        // at least one non-zero codebook in group 3's bands.
        let mut spec = [0.0f32; 1024];
        // Sub-window 3 starts at index 3*128 = 384. Put energy in bands
        // 2..=4 of that window.
        for k in 384 + 8..384 + 16 {
            spec[k] = 500.0;
        }
        let ics = analyse_and_quantise_short(&spec, 4).unwrap();
        assert_eq!(ics.num_groups, 8);
        assert!(ics.max_sfb >= 2, "max_sfb={}", ics.max_sfb);
        // Group 3 should carry the non-zero bands.
        let max_sfb = ics.max_sfb as usize;
        let g3_nonzero = (0..max_sfb).any(|sfb| ics.cbs[3 * max_sfb + sfb] != 0);
        assert!(g3_nonzero, "group 3 should have a non-zero band");
        // Other groups are all zeros (cb = 0).
        for g in [0, 1, 2, 4, 5, 6, 7] {
            for sfb in 0..max_sfb {
                assert_eq!(
                    ics.cbs[g * max_sfb + sfb],
                    0,
                    "group {g} sfb {sfb} should be empty"
                );
            }
        }
    }

    #[test]
    fn build_adts_header_fields() {
        let frame = build_adts_frame(4, 1, 100); // 44.1 kHz, mono, 100-byte payload
        assert_eq!(frame.len(), 7);
        assert_eq!(frame[0], 0xFF);
        assert!((frame[1] & 0xF0) == 0xF0);
        // frame_length = 107
        let flen = (((frame[3] & 0x3) as usize) << 11)
            | ((frame[4] as usize) << 3)
            | ((frame[5] >> 5) as usize);
        assert_eq!(flen, 107);
    }

    #[test]
    fn escape_amp_bits_exact() {
        // a = 16 → prefix = 0, bits = 0 + 1 + 0 + 4 = 5
        assert_eq!(escape_amp_bits(16), 5);
        // a = 32 → top = 5, prefix = 1, bits = 1 + 1 + 1 + 4 = 7
        assert_eq!(escape_amp_bits(32), 7);
        // a = 8191 → top = 12, prefix = 8, bits = 8 + 1 + 8 + 4 = 21
        assert_eq!(escape_amp_bits(8191), 21);
    }

    #[test]
    fn scalefactor_zero_delta_is_one_bit() {
        // delta=0 is SCALEFACTOR_CODES[60]=0x00 with 1 bit.
        assert_eq!(SCALEFACTOR_CODES[60], 0);
        assert_eq!(SCALEFACTOR_BITS[60], 1);
    }

    #[test]
    fn sf_huffman_roundtrip() {
        use crate::huffman::decode_scalefactor_delta;
        use oxideav_core::bits::BitReader;
        // Write a series of deltas via the encoder's SF writer logic and
        // verify the decoder reads them back unchanged.
        let deltas: Vec<i32> = (-30..=30).step_by(3).collect();
        let mut bw = BitWriter::new();
        for &d in &deltas {
            let idx = (d + 60) as usize;
            bw.write_u32(SCALEFACTOR_CODES[idx] as u32, SCALEFACTOR_BITS[idx] as u32);
        }
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        for &expect in &deltas {
            let got = decode_scalefactor_delta(&mut br).unwrap();
            assert_eq!(got, expect, "SF roundtrip mismatch");
        }
    }

    #[test]
    fn spectral_book_roundtrip_book8() {
        use crate::huffman::{decode_spectral, BOOK8};
        use oxideav_core::bits::BitReader;
        // Encode a few unsigned (lav 7) pairs and verify decode.
        let pairs = [(3i32, -5i32), (7, 0), (0, 0), (-2, -7), (1, 1)];
        let mut bw = BitWriter::new();
        for &(a, b) in &pairs {
            let q = [a, b];
            write_band_bits(&mut bw, &q, 8);
        }
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        for &(want_a, want_b) in &pairs {
            let v = decode_spectral(&mut br, &BOOK8).unwrap();
            assert_eq!(v[0] as i32, want_a, "book8 A mismatch");
            assert_eq!(v[1] as i32, want_b, "book8 B mismatch");
        }
    }

    #[test]
    fn book7_index_layout() {
        use crate::huffman::{decode_spectral, BOOK7};
        use oxideav_core::bits::BitReader;
        // Book 7 (dim=2, lav=7, unsigned, no escape): index = i*8 + j.
        // Try (1, 0) by setting q = [1, 0].
        let mut bw = BitWriter::new();
        write_band_bits(&mut bw, &[1, 0], 7);
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let v = decode_spectral(&mut br, &BOOK7).unwrap();
        assert_eq!(v[0] as i32, 1);
        assert_eq!(v[1] as i32, 0);
    }

    #[test]
    fn spectral_book_roundtrip_book11_with_escape() {
        use crate::huffman::{decode_spectral, BOOK11};
        use oxideav_core::bits::BitReader;
        let pairs = [(3i32, -5i32), (16, 0), (-32, 12), (100, -200), (1, 1)];
        let mut bw = BitWriter::new();
        for &(a, b) in &pairs {
            let q = [a, b];
            write_band_bits(&mut bw, &q, 11);
        }
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        for &(want_a, want_b) in &pairs {
            let v = decode_spectral(&mut br, &BOOK11).unwrap();
            assert_eq!(v[0] as i32, want_a, "book11 A mismatch");
            assert_eq!(v[1] as i32, want_b, "book11 B mismatch");
        }
    }

    #[test]
    fn next_window_seq_transitions() {
        // Validate the 6 entries of the state table.
        assert_eq!(
            next_window_seq(WindowSequence::OnlyLong, true),
            WindowSequence::LongStart,
        );
        assert_eq!(
            next_window_seq(WindowSequence::OnlyLong, false),
            WindowSequence::OnlyLong,
        );
        assert_eq!(
            next_window_seq(WindowSequence::LongStart, false),
            WindowSequence::EightShort,
        );
        assert_eq!(
            next_window_seq(WindowSequence::LongStart, true),
            WindowSequence::EightShort,
        );
        assert_eq!(
            next_window_seq(WindowSequence::EightShort, true),
            WindowSequence::EightShort,
        );
        assert_eq!(
            next_window_seq(WindowSequence::EightShort, false),
            WindowSequence::LongStop,
        );
        assert_eq!(
            next_window_seq(WindowSequence::LongStop, true),
            WindowSequence::OnlyLong,
        );
        assert_eq!(
            next_window_seq(WindowSequence::LongStop, false),
            WindowSequence::OnlyLong,
        );
    }

    /// Percussive-signal round-trip: a click train drives the encoder's
    /// lookahead + transient detector through the full
    /// `OnlyLong → LongStart → EightShort → LongStop → OnlyLong`
    /// cycle. Verify that (a) every emitted packet parses through the
    /// decoder end-to-end, (b) at least one transition actually
    /// happened (we see LongStart AND EightShort AND LongStop in the
    /// emitted window-sequence history), and (c) the decoded output
    /// stays bounded — no catastrophic pre-echo spikes in the pre-attack
    /// samples.
    #[test]
    fn short_block_percussive_round_trip() {
        use oxideav_core::bits::BitReader;
        #[allow(unused_imports)]
        use oxideav_core::Decoder;
        use oxideav_core::{CodecId, CodecParameters, Packet, TimeBase};

        let sr = 44_100u32;
        // Build a 10-frame (10 * 1024 = 10 240 samples) click train:
        // low-level white-ish jitter with sharp attacks at samples 4096
        // and 8192 (≈ start of frames 4 and 8).
        let total_samples = 10 * FRAME_LEN;
        let mut pcm_f32 = vec![0.0f32; total_samples];
        for (i, s) in pcm_f32.iter_mut().enumerate() {
            // Deterministic pseudo-noise — a handful of dB below attacks.
            let n = ((i as u32).wrapping_mul(1103515245).wrapping_add(12345)) >> 16;
            *s = ((n as f32 / 65536.0) - 0.5) * 0.01;
        }
        // Attacks: 16 full-amplitude samples per attack.
        for &pos in &[4096usize, 8192usize] {
            for k in 0..16 {
                pcm_f32[pos + k] = 0.9 * if k & 1 == 0 { 1.0 } else { -1.0 };
            }
        }
        // Convert to interleaved S16 LE.
        let mut pcm_s16: Vec<u8> = Vec::with_capacity(total_samples * 2);
        for &x in &pcm_f32 {
            let s = (x * 32767.0).clamp(-32768.0, 32767.0) as i16;
            pcm_s16.extend_from_slice(&s.to_le_bytes());
        }

        // Construct the encoder directly so we can flip the short-block
        // feature flag (not exposed via the Encoder trait).
        let mut params = CodecParameters::audio(CodecId::new(crate::CODEC_ID_STR));
        params.sample_rate = Some(sr);
        params.channels = Some(1);
        let mut enc = AacEncoder::new(&params).expect("make encoder");
        enc.set_enable_short_blocks(true);

        let frame = Frame::Audio(AudioFrame {
            samples: total_samples as u32,
            pts: None,
            data: vec![pcm_s16],
        });
        enc.send_frame(&frame).unwrap();
        enc.flush().unwrap();

        // Drain emitted packets.
        let mut pkts: Vec<Packet> = Vec::new();
        loop {
            match enc.receive_packet() {
                Ok(p) => pkts.push(p),
                Err(oxideav_core::Error::Eof) => break,
                Err(oxideav_core::Error::NeedMore) => break,
                Err(e) => panic!("encoder: {e}"),
            }
        }
        // With 10 input frames + 1 long-path silence tail + 1 short-path
        // held drain, we expect at least 10 packets; upper bound ≈ 12.
        assert!(
            pkts.len() >= 10,
            "emitted {} packets, expected >= 10",
            pkts.len()
        );

        // Inspect each packet's window_sequence by parsing the first
        // SCE element's ics_info. The ADTS header is 7 bytes; the
        // raw_data_block then reads:
        //   id_syn_ele (3) | elem_instance_tag (4)
        //   SCE: global_gain (8) | ics_info: reserved (1) | window_sequence (2) | ...
        // We only need to read the first 3 + 4 + 8 + 1 + 2 = 18 bits.
        // Counters indexed by `seq as u8` (OnlyLong=0 .. LongStop=3).
        let mut seen = [0usize; 4];
        let mut seq_history: Vec<WindowSequence> = Vec::new();
        for pkt in &pkts {
            assert!(pkt.data.len() > 7, "ADTS frame too short");
            let payload = &pkt.data[7..];
            let mut br = BitReader::new(payload);
            let syn_ele = br.read_u32(3).unwrap();
            assert_eq!(
                syn_ele,
                ElementType::Sce as u32,
                "expected SCE element, got id={syn_ele}"
            );
            let _tag = br.read_u32(4).unwrap();
            let _global_gain = br.read_u32(8).unwrap();
            let _reserved = br.read_bit().unwrap();
            let ws = WindowSequence::from_u32(br.read_u32(2).unwrap());
            seen[ws as usize] += 1;
            seq_history.push(ws);
        }

        // The state machine must have traversed the short-block cycle
        // at least once. Every transient path goes through all four
        // sequences.
        assert!(
            seen[WindowSequence::LongStart as usize] > 0,
            "never saw LongStart. history = {seq_history:?}",
        );
        assert!(
            seen[WindowSequence::EightShort as usize] > 0,
            "never saw EightShort. history = {seq_history:?}",
        );
        assert!(
            seen[WindowSequence::LongStop as usize] > 0,
            "never saw LongStop. history = {seq_history:?}",
        );
        assert!(
            seen[WindowSequence::OnlyLong as usize] > 0,
            "never saw OnlyLong. history = {seq_history:?}",
        );

        // Decoder round-trip: all packets must parse without error.
        let mut dec = crate::decoder::make_decoder(&params).expect("decoder");
        let tb = TimeBase::new(1, sr as i64);
        let mut decoded = Vec::new();
        for pkt in pkts.iter() {
            let p2 = Packet::new(0, tb, pkt.data.clone());
            dec.send_packet(&p2)
                .unwrap_or_else(|e| panic!("decoder rejected a short-block packet: {e}"));
            loop {
                match dec.receive_frame() {
                    Ok(oxideav_core::Frame::Audio(af)) => {
                        assert_eq!(af.samples, 1024);
                        // Grab the first plane (S16 interleaved — 1 ch
                        // so 2 bytes per sample). Append to decoded.
                        let plane = &af.data[0];
                        for k in 0..af.samples as usize {
                            let s = i16::from_le_bytes([plane[k * 2], plane[k * 2 + 1]]);
                            decoded.push(s as f32 / 32768.0);
                        }
                    }
                    Ok(other) => panic!("unexpected frame variant: {other:?}"),
                    Err(oxideav_core::Error::NeedMore) => break,
                    Err(e) => panic!("decoder receive_frame: {e}"),
                }
            }
        }
        // Flush the decoder in case it has one frame of residual delay.
        let _ = dec.flush();
        loop {
            match dec.receive_frame() {
                Ok(oxideav_core::Frame::Audio(af)) => {
                    let plane = &af.data[0];
                    for k in 0..af.samples as usize {
                        let s = i16::from_le_bytes([plane[k * 2], plane[k * 2 + 1]]);
                        decoded.push(s as f32 / 32768.0);
                    }
                }
                Ok(_) => break,
                Err(_) => break,
            }
        }

        // Sanity: decoded is non-empty and at least as long as the
        // original input (encoder + decoder each add ≈ 1 frame of
        // latency; we fed 10 frames, should see ≥ 10 frames out).
        assert!(
            decoded.len() >= 10 * FRAME_LEN,
            "decoded only {} samples, expected ≥ {}",
            decoded.len(),
            10 * FRAME_LEN,
        );

        // Bounded pre-echo check: the decoder is lossy, so we can't
        // expect bit-exact output. We just verify nothing exploded —
        // every decoded sample fits in [-1.5, 1.5]. This catches NaNs,
        // infs, and runaway quantisation errors that would happen if
        // the window transitions produced inconsistent overlap.
        for (i, &s) in decoded.iter().enumerate() {
            assert!(
                s.is_finite() && s.abs() < 1.5,
                "decoded sample {i} out of range: {s}",
            );
        }
    }

    #[test]
    fn encoder_smoke_mono() {
        let mut params = CodecParameters::audio(CodecId::new(crate::CODEC_ID_STR));
        params.sample_rate = Some(44_100);
        params.channels = Some(1);
        let mut enc = make_encoder(&params).expect("make encoder");
        // Feed 2048 samples of a 440 Hz sine.
        let mut pcm = Vec::with_capacity(2048 * 2);
        for i in 0..2048 {
            let v = (2.0 * std::f32::consts::PI * 440.0 * i as f32 / 44_100.0).sin();
            let s = (v * 0.5 * 32767.0) as i16;
            pcm.extend_from_slice(&s.to_le_bytes());
        }
        let frame = Frame::Audio(AudioFrame {
            samples: 2048,
            pts: None,
            data: vec![pcm],
        });
        enc.send_frame(&frame).unwrap();
        let pkt1 = enc.receive_packet().unwrap();
        assert!(pkt1.data.len() >= 7);
        let pkt2 = enc.receive_packet().unwrap();
        assert!(pkt2.data.len() >= 7);
        // Both should be ADTS-framed.
        assert_eq!(pkt1.data[0], 0xFF);
        assert_eq!(pkt2.data[0], 0xFF);
    }
}

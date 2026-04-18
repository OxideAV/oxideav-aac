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

use oxideav_codec::Encoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, MediaType, Packet, Result, SampleFormat,
    TimeBase,
};

use crate::bitwriter::BitWriter;
use crate::ics::{INTENSITY_HCB, INTENSITY_HCB2, NOISE_HCB};
use crate::huffman_tables::{
    BOOK10_BITS, BOOK10_CODES, BOOK11_BITS, BOOK11_CODES, BOOK1_BITS, BOOK1_CODES, BOOK2_BITS,
    BOOK2_CODES, BOOK3_BITS, BOOK3_CODES, BOOK4_BITS, BOOK4_CODES, BOOK5_BITS, BOOK5_CODES,
    BOOK6_BITS, BOOK6_CODES, BOOK7_BITS, BOOK7_CODES, BOOK8_BITS, BOOK8_CODES, BOOK9_BITS,
    BOOK9_CODES, SCALEFACTOR_BITS, SCALEFACTOR_CODES,
};
use crate::ics::SPEC_LEN;
use crate::mdct::mdct_long;
use crate::sfband::{SWB_LONG, SWB_SHORT};
use crate::syntax::{ElementType, WindowSequence, WindowShape, AOT_AAC_LC, SAMPLE_RATES};
use crate::tns_analyse::{analyse_long as tns_analyse_long, TnsEncFilter};
use crate::transient::TransientDetector;
use crate::window::{build_long_window_full, sine_long};

/// MDCT length (long block).
const FRAME_LEN: usize = 1024;
/// Full windowed block length (= 2*FRAME_LEN).
const BLOCK_LEN: usize = 2 * FRAME_LEN;

/// Magic number in the AAC quantizer rounding rule (§4.6.6):
/// `ix = floor(|x_scaled|^(3/4) + MAGIC_NUMBER)`.
const QUANT_MAGIC: f32 = 0.4054;

/// Forward-MDCT scale matching ffmpeg's AAC encoder convention
/// (`aacenc.c::dsp_init`: `float scale = 32768.0f`). Without this scale
/// the spectrum values come out ~5 orders of magnitude below what AAC
/// inverse quantisation expects, and reference decoders fall back to
/// near-silent output.
const MDCT_FORWARD_SCALE: f32 = 32768.0;

/// Largest un-escaped amplitude supported by book 11.
const ESC_LAV: i32 = 16;

pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    let channels = params
        .channels
        .ok_or_else(|| Error::invalid("AAC encoder: channels required"))?;
    // Map input channel count → AAC channel_configuration (§1.6.3).
    // Supported: 1, 2, 3, 4, 5, 6, 8 (configs 1..=7). 7 channels has no
    // standard config so it's rejected.
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
    Ok(Box::new(AacEncoder {
        codec_id: CodecId::new(crate::CODEC_ID_STR),
        out_params,
        time_base: TimeBase::new(1, sample_rate as i64),
        channels,
        channel_configuration,
        sample_rate,
        sf_index,
        bitrate,
        input_buf: vec![Vec::with_capacity(BLOCK_LEN * 2); n_ch],
        overlap: vec![vec![0.0f32; FRAME_LEN]; n_ch],
        output_queue: VecDeque::new(),
        pts: 0,
        flushed: false,
        enable_short_blocks: false,
        short_state: (0..n_ch).map(|_| ChannelShortState::default()).collect(),
    }))
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
}

impl AacEncoder {
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

    fn push_audio_frame(&mut self, frame: &AudioFrame) -> Result<()> {
        if frame.channels != self.channels {
            return Err(Error::invalid(format!(
                "AAC encoder: expected {} channels, got {}",
                self.channels, frame.channels
            )));
        }
        let n = frame.samples as usize;
        if n == 0 {
            return Ok(());
        }
        let plane = frame
            .data
            .first()
            .ok_or_else(|| Error::invalid("AAC encoder: frame missing data plane"))?;
        match frame.format {
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
        Ok(())
    }

    fn emit_block(&mut self, is_last: bool) -> Result<()> {
        if self.enable_short_blocks {
            self.emit_block_short(is_last)
        } else {
            self.emit_block_long(is_last)
        }
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
        Ok(())
    }

    fn encode_raw_data_block(&self, specs: &[Vec<f32>]) -> Result<Vec<u8>> {
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
        &self,
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
                        write_single_ics_any(
                            &mut bw,
                            &specs[ch_idx + 1],
                            self.sf_index,
                            r_seq,
                        )?;
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
        // ID_END
        bw.write_u32(ElementType::End as u32, 3);
        bw.align_to_byte();
        Ok(bw.finish())
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
    /// Per-band chosen codebook (0..=11).
    cbs: Vec<u8>,
    /// Per-band quantised coefficients laid out in band order.
    q_bands: Vec<Vec<i32>>,
    /// Global gain value (first non-ZERO band's scalefactor, 8-bit).
    global_gain: u8,
    /// TNS filter parameters (at most one filter per window for the
    /// current encoder). `None` => `tns_data_present = 0`.
    tns: Option<TnsEncFilter>,
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
    // in a useful range. We aim for `target_max ≈ 7` so smaller bands
    // can use the cheaper books 7-8 (LAV 7) instead of always falling
    // back to book 11. Loud bands will still drift higher and end up on
    // book 9/10/11 — that's fine.
    let target_max = 7i32;
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

    // Pick codebook per band. The `classify_pns_band` noise-band
    // detector is wired but gated OFF by default: a loose heuristic
    // regresses round-trip tests on tone-heavy content, and tuning the
    // threshold needs a psy-model that doesn't exist yet. The emission
    // machinery (codebook 13 section, 9-bit PNS seed, DPCM deltas for
    // subsequent PNS bands, write_spectral_data skip) is in place, so
    // once the detector is tuned we just toggle this flag.
    const ENABLE_PNS: bool = false;
    let mut cbs = vec![0u8; max_sfb];
    for sfb in 0..max_sfb {
        let q = &q_bands[sfb];
        cbs[sfb] = if q.iter().all(|&x| x == 0) {
            0
        } else if ENABLE_PNS {
            if let Some(pns_sf) =
                classify_pns_band(spec, swb[sfb] as usize, swb[sfb + 1] as usize)
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

    // Global gain: first *regular-codebook* band's scalefactor. The
    // decoder seeds `g_gain = global_gain`, `g_noise = global_gain - 90`,
    // `g_is = 0`. Only cbs 1..=11 use the g_gain stream, so we anchor
    // global_gain on the first such band — not the first PNS (cb 13) or
    // IS (cb 14/15) band.
    let mut gg: i32 = 100;
    for sfb in 0..max_sfb {
        let cb = cbs[sfb];
        if cb != 0 && cb != NOISE_HCB && cb != INTENSITY_HCB && cb != INTENSITY_HCB2 {
            gg = sfs[sfb];
            break;
        }
    }
    let gg_clamped = gg.clamp(0, 255) as u8;

    // Re-anchor sfs so that the *first non-zero band's* scalefactor = gg
    // and subsequent non-zero bands carry deltas on top of each previous
    // non-zero band. The decoder uses `g_gain = global_gain`, then for
    // every band with cb != ZERO it adds a delta. Zero bands don't read
    // a delta. So we just need the non-zero-band scalefactors. Zero-band
    // scalefactors are never written.

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
}

/// Analyse a 1024-coefficient EightShort spectrum (laid out as 8
/// contiguous 128-coefficient sub-windows) into an [`IcsShort`] ready
/// for bitstream emission.
///
/// Uses `num_groups = 8` (no grouping) — each sub-window gets its own
/// group for scalefactor purposes.
fn analyse_and_quantise_short(spec: &[f32; 1024], sf_index: u8) -> Result<IcsShort> {
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
    })
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
    bw.write_bit(false); // pulse_data_present
    let tns_present = ics.tns.is_some();
    bw.write_bit(tns_present);
    if let Some(ref f) = ics.tns {
        write_tns_data_long(bw, f);
    }
    bw.write_bit(false); // gain_control_data_present
    write_spectral_data(bw, ics);
    Ok(())
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
                bw.write_u32(
                    SCALEFACTOR_CODES[idx] as u32,
                    SCALEFACTOR_BITS[idx] as u32,
                );
            }
        } else if cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
            let delta = (target - g_is).clamp(-60, 60);
            g_is += delta;
            let idx = (delta + 60) as usize;
            bw.write_u32(
                SCALEFACTOR_CODES[idx] as u32,
                SCALEFACTOR_BITS[idx] as u32,
            );
        } else {
            let delta = (target - cur).clamp(-60, 60);
            cur += delta;
            let idx = (delta + 60) as usize;
            bw.write_u32(
                SCALEFACTOR_CODES[idx] as u32,
                SCALEFACTOR_BITS[idx] as u32,
            );
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

/// Write an entire SCE-style EightShort individual_channel_stream:
/// global_gain + ics_info + section + scalefactors + pulse=0 + tns=0 +
/// gain_control=0 + spectral_data. Used for both SCE and LFE elements.
#[allow(dead_code)]
fn write_single_ics_short(bw: &mut BitWriter, ics: &IcsShort) -> Result<()> {
    bw.write_u32(ics.global_gain as u32, 8);
    write_ics_info_short(bw, ics);
    write_section_data_short(bw, ics);
    write_scalefactors_short(bw, ics)?;
    bw.write_u32(0, 1); // pulse_data_present — always 0 for short (spec forbids it)
    bw.write_u32(0, 1); // tns_data_present   — short-window encoder TNS deferred
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
    // in the band; when the correlation is weak IS would reconstruct the
    // wrong ch1 and the error would blow up. Gated behind `ENABLE_IS`
    // until we have a psy-acoustic bit-allocation model to drive it; the
    // emission path is fully in place.
    const ENABLE_IS: bool = false;
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
        if ENABLE_IS {
            if let Some((pos, sign)) = classify_is_band(
                l,
                r,
                swb[sfb] as usize,
                swb[sfb + 1] as usize,
            ) {
                // IS cost ≈ L-band cost + ~10 bits overhead for the
                // scalefactor delta + codebook bits in section_data.
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
        use oxideav_codec::Decoder;
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
                assert_eq!(af.channels, 1);
                assert_eq!(af.samples, 1024);
            }
            Ok(other) => panic!("unexpected frame variant: {other:?}"),
            Err(e) => panic!("decoder rejected EightShort SCE: {e}"),
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
        use crate::bitreader::BitReader;
        use crate::huffman::decode_scalefactor_delta;
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
        use crate::bitreader::BitReader;
        use crate::huffman::{decode_spectral, BOOK8};
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
        use crate::bitreader::BitReader;
        use crate::huffman::{decode_spectral, BOOK7};
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
        use crate::bitreader::BitReader;
        use crate::huffman::{decode_spectral, BOOK11};
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
            format: SampleFormat::S16,
            channels: 1,
            sample_rate: 44_100,
            samples: 2048,
            pts: None,
            time_base: TimeBase::new(1, 44_100),
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

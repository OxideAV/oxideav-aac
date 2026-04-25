//! HE-AACv1 mono / stereo encoder wrappers.
//!
//! Wraps [`crate::encoder::AacEncoder`] (configured for 1- or 2-channel)
//! and [`crate::sbr::encode::SbrEncoder`] / [`SbrStereoEncoder`] to
//! produce a combined AAC-LC core + SBR stream. The caller feeds PCM at
//! **2×** the target AAC-LC sample rate; the wrapper:
//!
//! 1. Downsamples the 2× PCM by two into the inner AAC-LC encoder.
//! 2. Runs the 64-channel complex analysis QMF (one per channel for
//!    stereo) on the original 2× PCM and builds the SBR scalefactor
//!    payload (SCE for mono, CPE in independent coupling for stereo).
//! 3. Stages the SBR payload as a FIL / EXT_SBR_DATA element on the
//!    inner encoder so the next-emitted raw_data_block carries it
//!    before `ID_END`.
//! 4. Returns the ADTS-wrapped packets from the inner encoder.
//!
//! The ADTS header reports the AAC-LC sample rate — the doubled output
//! rate is signalled *implicitly* (the decoder sees SBR in the FIL
//! element and applies the 2× doubler). This is the standard HE-AAC
//! ADTS encapsulation.
//!
//! Mono + stereo + core rates that are in the AAC-LC sample-rate table
//! are supported (typically 24 kHz → 48 kHz doubled output, or 22.05 kHz
//! → 44.1 kHz).

use oxideav_core::Encoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, MediaType, Packet, Result, SampleFormat,
};

use crate::encoder::{AacEncoder, SbrFilBits};
use crate::sbr::encode::{Downsampler, QmfAnalysis64, SbrEncoder, SbrStereoEncoder};
use crate::sbr::ps::{analyse_ps_params_10_multi_env, detect_num_env};
use crate::sbr::{Complex32, NUM_QMF_BANDS, NUM_TIME_SLOTS_1024};

/// Number of high-rate input samples per encoded frame — one AAC core
/// frame of 1024 core samples corresponds to 2048 high-rate samples.
const HIGH_RATE_FRAME: usize = 2048;

/// HE-AACv1 mono encoder.
pub struct HeAacMonoEncoder {
    inner: AacEncoder,
    sbr: SbrEncoder,
    downsampler: Downsampler,
    /// Buffer of 2×-rate PCM samples pending encode.
    high_pcm: Vec<f32>,
    flushed: bool,
    /// Core (AAC-LC) sample rate.
    core_rate: u32,
    /// Output sample rate — 2 × core_rate, exposed via `output_params`
    /// so callers can route the downstream sink correctly.
    out_rate: u32,
    out_params: CodecParameters,
}

impl HeAacMonoEncoder {
    /// Build an HE-AACv1 mono encoder.
    ///
    /// `params.sample_rate` must be the *high* rate (2× the AAC-LC
    /// core). `params.channels` must be 1. Valid high rates follow
    /// Table 4.82 — effectively `24_000`, `32_000`, `44_100`, `48_000`
    /// (with core at 12 / 16 / 22.05 / 24 kHz respectively). 48 kHz
    /// input produces a 24 kHz AAC-LC core.
    pub fn new(params: &CodecParameters) -> Result<Self> {
        let channels = params.channels.unwrap_or(1);
        if channels != 1 {
            return Err(Error::unsupported(
                "HE-AACv1 mono encoder: only 1 channel supported",
            ));
        }
        let out_rate = params
            .sample_rate
            .ok_or_else(|| Error::invalid("HE-AAC encoder: sample_rate required"))?;
        if out_rate % 2 != 0 {
            return Err(Error::invalid(
                "HE-AAC encoder: input sample rate must be even (2x core)",
            ));
        }
        let core_rate = out_rate / 2;
        // Inner AAC-LC encoder runs at the core rate.
        let mut inner_params = params.clone();
        inner_params.sample_rate = Some(core_rate);
        inner_params.channels = Some(1);
        let inner = AacEncoder::new(&inner_params)?;
        let sbr = SbrEncoder::new(core_rate)?;
        let mut out_params = params.clone();
        out_params.media_type = MediaType::Audio;
        out_params.sample_format = Some(SampleFormat::S16);
        out_params.channels = Some(1);
        out_params.sample_rate = Some(out_rate);
        Ok(Self {
            inner,
            sbr,
            downsampler: Downsampler::new(),
            high_pcm: Vec::with_capacity(HIGH_RATE_FRAME * 2),
            flushed: false,
            core_rate,
            out_rate,
            out_params,
        })
    }

    fn push_audio(&mut self, frame: &AudioFrame) -> Result<()> {
        if frame.channels != 1 {
            return Err(Error::invalid("HE-AAC encoder: expected 1-channel input"));
        }
        let n = frame.samples as usize;
        if n == 0 {
            return Ok(());
        }
        let plane = frame
            .data
            .first()
            .ok_or_else(|| Error::invalid("HE-AAC encoder: missing data plane"))?;
        match frame.format {
            SampleFormat::S16 => {
                let stride = 2usize;
                if plane.len() < n * stride {
                    return Err(Error::invalid("HE-AAC encoder: S16 frame too short"));
                }
                for i in 0..n {
                    let s = i16::from_le_bytes([plane[i * 2], plane[i * 2 + 1]]);
                    self.high_pcm.push(s as f32 / 32768.0);
                }
            }
            SampleFormat::F32 => {
                let stride = 4usize;
                if plane.len() < n * stride {
                    return Err(Error::invalid("HE-AAC encoder: F32 frame too short"));
                }
                for i in 0..n {
                    let off = i * 4;
                    let v = f32::from_le_bytes([
                        plane[off],
                        plane[off + 1],
                        plane[off + 2],
                        plane[off + 3],
                    ]);
                    self.high_pcm.push(v);
                }
            }
            other => {
                return Err(Error::unsupported(format!(
                    "HE-AAC encoder: sample format {other:?} not supported"
                )));
            }
        }
        Ok(())
    }

    fn drain_frames(&mut self) -> Result<()> {
        while self.high_pcm.len() >= HIGH_RATE_FRAME {
            self.emit_frame(false)?;
        }
        Ok(())
    }

    fn emit_frame(&mut self, flush: bool) -> Result<()> {
        let have = self.high_pcm.len();
        if have == 0 {
            return Ok(());
        }
        let to_take = have.min(HIGH_RATE_FRAME);
        let mut block = vec![0.0f32; HIGH_RATE_FRAME];
        block[..to_take].copy_from_slice(&self.high_pcm[..to_take]);
        self.high_pcm.drain(..to_take);

        // 1) Analyse QMF on the high-rate block and emit SBR payload.
        let x_high = self.sbr.analyse(&block);
        let (sbr_bytes, bits) = self.sbr.emit_sbr_payload(&x_high);
        self.inner.stage_sbr_fil(SbrFilBits {
            bytes: sbr_bytes,
            bits,
        });

        // 2) Downsample 2× → 1× into the inner AAC-LC encoder.
        let mut low = vec![0.0f32; HIGH_RATE_FRAME / 2];
        self.downsampler.process(&block, &mut low);
        // Feed as one AudioFrame (F32) to the inner encoder.
        let mut bytes = Vec::with_capacity(low.len() * 4);
        for v in &low {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        let af = AudioFrame {
            format: SampleFormat::F32,
            channels: 1,
            sample_rate: self.core_rate,
            samples: low.len() as u32,
            pts: None,
            time_base: oxideav_core::TimeBase::new(1, self.core_rate as i64),
            data: vec![bytes],
        };
        self.inner.send_frame(&Frame::Audio(af))?;
        if flush {
            self.inner.flush()?;
        }
        Ok(())
    }

    /// Return the output sample rate (= 2 × core rate).
    pub fn output_sample_rate(&self) -> u32 {
        self.out_rate
    }
}

impl Encoder for HeAacMonoEncoder {
    fn codec_id(&self) -> &CodecId {
        self.inner.codec_id()
    }

    fn output_params(&self) -> &CodecParameters {
        &self.out_params
    }

    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        if self.flushed {
            return Err(Error::other(
                "HE-AAC encoder: flushed, cannot accept more frames",
            ));
        }
        match frame {
            Frame::Audio(af) => {
                self.push_audio(af)?;
                self.drain_frames()
            }
            _ => Err(Error::invalid("HE-AAC encoder: expected audio frame")),
        }
    }

    fn receive_packet(&mut self) -> Result<Packet> {
        self.inner.receive_packet()
    }

    fn flush(&mut self) -> Result<()> {
        if self.flushed {
            return Ok(());
        }
        if !self.high_pcm.is_empty() {
            // Pad last partial frame with silence and emit.
            if self.high_pcm.len() < HIGH_RATE_FRAME {
                self.high_pcm.resize(HIGH_RATE_FRAME, 0.0);
            }
            self.emit_frame(true)?;
        } else {
            // Still flush the inner encoder.
            self.inner.flush()?;
        }
        self.flushed = true;
        Ok(())
    }
}

/// HE-AACv1 stereo encoder.
///
/// Same pipeline as [`HeAacMonoEncoder`] but with a 2-channel inner
/// encoder (CPE) and an [`SbrStereoEncoder`] producing a CPE-shaped
/// SBR payload (independent coupling, see
/// [`crate::sbr::encode::write_channel_pair_element_independent`]).
///
/// Input is interleaved stereo PCM at the high (output) rate. The
/// wrapper splits it into two channel planes, downsamples each, feeds
/// the resulting low-band stereo into the AAC-LC encoder as one
/// interleaved F32 frame, and analyses each channel through its own
/// 64-band QMF for the SBR payload.
pub struct HeAacStereoEncoder {
    inner: AacEncoder,
    sbr: SbrStereoEncoder,
    downsampler_l: Downsampler,
    downsampler_r: Downsampler,
    /// Per-channel buffer of 2×-rate PCM samples pending encode.
    high_pcm_l: Vec<f32>,
    high_pcm_r: Vec<f32>,
    flushed: bool,
    core_rate: u32,
    out_rate: u32,
    out_params: CodecParameters,
}

impl HeAacStereoEncoder {
    /// Build an HE-AACv1 stereo encoder.
    ///
    /// `params.sample_rate` must be the *high* rate (2× the AAC-LC
    /// core). `params.channels` must be 2.
    pub fn new(params: &CodecParameters) -> Result<Self> {
        let channels = params.channels.unwrap_or(2);
        if channels != 2 {
            return Err(Error::unsupported(
                "HE-AACv1 stereo encoder: only 2 channels supported",
            ));
        }
        let out_rate = params
            .sample_rate
            .ok_or_else(|| Error::invalid("HE-AAC encoder: sample_rate required"))?;
        if out_rate % 2 != 0 {
            return Err(Error::invalid(
                "HE-AAC encoder: input sample rate must be even (2x core)",
            ));
        }
        let core_rate = out_rate / 2;
        let mut inner_params = params.clone();
        inner_params.sample_rate = Some(core_rate);
        inner_params.channels = Some(2);
        let inner = AacEncoder::new(&inner_params)?;
        let sbr = SbrStereoEncoder::new(core_rate)?;
        let mut out_params = params.clone();
        out_params.media_type = MediaType::Audio;
        out_params.sample_format = Some(SampleFormat::S16);
        out_params.channels = Some(2);
        out_params.sample_rate = Some(out_rate);
        Ok(Self {
            inner,
            sbr,
            downsampler_l: Downsampler::new(),
            downsampler_r: Downsampler::new(),
            high_pcm_l: Vec::with_capacity(HIGH_RATE_FRAME * 2),
            high_pcm_r: Vec::with_capacity(HIGH_RATE_FRAME * 2),
            flushed: false,
            core_rate,
            out_rate,
            out_params,
        })
    }

    fn push_audio(&mut self, frame: &AudioFrame) -> Result<()> {
        if frame.channels != 2 {
            return Err(Error::invalid(
                "HE-AACv1 stereo encoder: expected 2-channel input",
            ));
        }
        let n = frame.samples as usize;
        if n == 0 {
            return Ok(());
        }
        let plane = frame
            .data
            .first()
            .ok_or_else(|| Error::invalid("HE-AAC encoder: missing data plane"))?;
        match frame.format {
            SampleFormat::S16 => {
                let stride = 4usize; // 2 channels * 2 bytes
                if plane.len() < n * stride {
                    return Err(Error::invalid("HE-AAC encoder: S16 frame too short"));
                }
                for i in 0..n {
                    let off = i * stride;
                    let l = i16::from_le_bytes([plane[off], plane[off + 1]]);
                    let r = i16::from_le_bytes([plane[off + 2], plane[off + 3]]);
                    self.high_pcm_l.push(l as f32 / 32768.0);
                    self.high_pcm_r.push(r as f32 / 32768.0);
                }
            }
            SampleFormat::F32 => {
                let stride = 8usize; // 2 channels * 4 bytes
                if plane.len() < n * stride {
                    return Err(Error::invalid("HE-AAC encoder: F32 frame too short"));
                }
                for i in 0..n {
                    let off = i * stride;
                    let l = f32::from_le_bytes([
                        plane[off],
                        plane[off + 1],
                        plane[off + 2],
                        plane[off + 3],
                    ]);
                    let r = f32::from_le_bytes([
                        plane[off + 4],
                        plane[off + 5],
                        plane[off + 6],
                        plane[off + 7],
                    ]);
                    self.high_pcm_l.push(l);
                    self.high_pcm_r.push(r);
                }
            }
            other => {
                return Err(Error::unsupported(format!(
                    "HE-AAC encoder: sample format {other:?} not supported"
                )));
            }
        }
        Ok(())
    }

    fn drain_frames(&mut self) -> Result<()> {
        while self.high_pcm_l.len() >= HIGH_RATE_FRAME && self.high_pcm_r.len() >= HIGH_RATE_FRAME {
            self.emit_frame(false)?;
        }
        Ok(())
    }

    fn emit_frame(&mut self, flush: bool) -> Result<()> {
        let have = self.high_pcm_l.len().min(self.high_pcm_r.len());
        if have == 0 {
            return Ok(());
        }
        let to_take = have.min(HIGH_RATE_FRAME);
        let mut block_l = vec![0.0f32; HIGH_RATE_FRAME];
        let mut block_r = vec![0.0f32; HIGH_RATE_FRAME];
        block_l[..to_take].copy_from_slice(&self.high_pcm_l[..to_take]);
        block_r[..to_take].copy_from_slice(&self.high_pcm_r[..to_take]);
        self.high_pcm_l.drain(..to_take);
        self.high_pcm_r.drain(..to_take);

        // 1) Per-channel analysis QMF + emit one CPE-shaped SBR payload.
        let (xl, xr) = self.sbr.analyse(&block_l, &block_r);
        let (sbr_bytes, bits) = self.sbr.emit_sbr_payload_pair(&xl, &xr);
        self.inner.stage_sbr_fil(SbrFilBits {
            bytes: sbr_bytes,
            bits,
        });

        // 2) Per-channel downsample 2× → 1×, then re-interleave for the
        //    inner stereo AAC-LC encoder.
        let mut low_l = vec![0.0f32; HIGH_RATE_FRAME / 2];
        let mut low_r = vec![0.0f32; HIGH_RATE_FRAME / 2];
        self.downsampler_l.process(&block_l, &mut low_l);
        self.downsampler_r.process(&block_r, &mut low_r);
        let n_low = low_l.len();
        let mut bytes = Vec::with_capacity(n_low * 8); // 2 channels × 4 bytes (F32)
        for i in 0..n_low {
            bytes.extend_from_slice(&low_l[i].to_le_bytes());
            bytes.extend_from_slice(&low_r[i].to_le_bytes());
        }
        let af = AudioFrame {
            format: SampleFormat::F32,
            channels: 2,
            sample_rate: self.core_rate,
            samples: n_low as u32,
            pts: None,
            time_base: oxideav_core::TimeBase::new(1, self.core_rate as i64),
            data: vec![bytes],
        };
        self.inner.send_frame(&Frame::Audio(af))?;
        if flush {
            self.inner.flush()?;
        }
        Ok(())
    }

    /// Return the output sample rate (= 2 × core rate).
    pub fn output_sample_rate(&self) -> u32 {
        self.out_rate
    }
}

impl Encoder for HeAacStereoEncoder {
    fn codec_id(&self) -> &CodecId {
        self.inner.codec_id()
    }

    fn output_params(&self) -> &CodecParameters {
        &self.out_params
    }

    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        if self.flushed {
            return Err(Error::other(
                "HE-AAC stereo encoder: flushed, cannot accept more frames",
            ));
        }
        match frame {
            Frame::Audio(af) => {
                self.push_audio(af)?;
                self.drain_frames()
            }
            _ => Err(Error::invalid("HE-AAC encoder: expected audio frame")),
        }
    }

    fn receive_packet(&mut self) -> Result<Packet> {
        self.inner.receive_packet()
    }

    fn flush(&mut self) -> Result<()> {
        if self.flushed {
            return Ok(());
        }
        let have = self.high_pcm_l.len().min(self.high_pcm_r.len());
        if have > 0 {
            // Pad partial last frame with silence and emit.
            if self.high_pcm_l.len() < HIGH_RATE_FRAME {
                self.high_pcm_l.resize(HIGH_RATE_FRAME, 0.0);
            }
            if self.high_pcm_r.len() < HIGH_RATE_FRAME {
                self.high_pcm_r.resize(HIGH_RATE_FRAME, 0.0);
            }
            self.emit_frame(true)?;
        } else {
            self.inner.flush()?;
        }
        self.flushed = true;
        Ok(())
    }
}

/// HE-AACv2 (mono SBR + Parametric Stereo) encoder.
///
/// Pipeline (ISO/IEC 14496-3 §1.6.5 / §8.6.4):
///
/// ```text
///   2x-rate stereo PCM in
///       ↓ keep L, R separate
///   ┌─────────────┬───────────────────────────────────┐
///   │             │                                   │
///   │             ↓ per-channel 64-band complex QMF   │
///   │           Xl[t,k], Xr[t,k]  (32 × 64)           │
///   │             ↓ analyse_ps_params_10              │
///   │           PsParams10 (IID + ICC, 10 bands)      │
///   │             ↓ stage on SbrEncoder               │
///   │                                                 │
///   ↓ downmix L+R → mono                              │
///   2x-rate mono PCM                                  │
///       ↓ HeAacMonoEncoder pipeline                   │
///       ↓   - downsample 2x → 1x for AAC-LC core SCE  │
///       ↓   - SBR mono analysis + envelope payload    │
///       ↓   - SBR FIL element with EXT_ID_PS carrying │
///       ↓     the real PsParams10 IID/ICC bands       │
///   ADTS frames out — decoder runs SBR on the single  │
///   core channel, then PS upmixes to stereo using the │
///   per-band IID (level) and ICC (coherence) hints.   │
/// ```
///
/// **Real PS analysis (round 13):** per QMF band b, we compute over each
/// frame's worth of QMF samples:
/// ```text
///   P_L  = Σ |L(t,k)|^2,  P_R = Σ |R(t,k)|^2,  P_LR = Σ L · conj(R)
///   IID_dB(b) = 10·log10(P_L / P_R)               → quant. Table 8.25
///   ICC(b)    = |P_LR| / sqrt(P_L · P_R)          → quant. Table 8.28
/// ```
/// QMF bands are aggregated to 10 PS parameter bands per Table 8.45 /
/// 8.48 (folded from the 20-band default mapping). This preserves the
/// stereo image — a 1 kHz tone on L only / 2 kHz on R only emerges from
/// the decoder with the correct per-band level split, instead of
/// collapsing to mono as in the round-12 identity-stereo path.
///
/// `params.sample_rate` must be the *high* rate (2x core rate, e.g.
/// 44 100 or 48 000 Hz). `params.channels` must be 2 (stereo input).
pub struct HeAacV2Encoder {
    inner: AacEncoder,
    sbr: SbrEncoder,
    downsampler: Downsampler,
    /// Per-channel QMF analysis banks for PS parameter extraction. The
    /// SBR mono encoder (`sbr`) maintains its own QMF bank for the
    /// downmixed signal — these are L/R-specific and do NOT feed the
    /// SBR envelope.
    qmf_l: QmfAnalysis64,
    qmf_r: QmfAnalysis64,
    /// Per-channel buffers of 2x-rate PCM samples pending encode.
    high_pcm_l: Vec<f32>,
    high_pcm_r: Vec<f32>,
    flushed: bool,
    core_rate: u32,
    out_rate: u32,
    out_params: CodecParameters,
}

impl HeAacV2Encoder {
    /// Build an HE-AACv2 encoder. The encoder takes 2-channel PCM at
    /// `params.sample_rate` (the high rate, 2x the AAC-LC core),
    /// downmixes to mono, runs the SBR + PS bitstream path, and emits
    /// ADTS frames whose decoder output is stereo at the high rate.
    pub fn new(params: &CodecParameters) -> Result<Self> {
        let channels = params.channels.unwrap_or(2);
        if channels != 2 {
            return Err(Error::unsupported(
                "HE-AACv2 encoder: input must be 2 channels (downmixed to mono internally)",
            ));
        }
        let out_rate = params
            .sample_rate
            .ok_or_else(|| Error::invalid("HE-AAC encoder: sample_rate required"))?;
        if out_rate % 2 != 0 {
            return Err(Error::invalid(
                "HE-AAC encoder: input sample rate must be even (2x core)",
            ));
        }
        let core_rate = out_rate / 2;
        // The inner AAC-LC core is **mono** SCE — PS upmixes the single
        // SBR channel to two output channels at the decoder. ADTS
        // therefore advertises channel_configuration = 1.
        let mut inner_params = params.clone();
        inner_params.sample_rate = Some(core_rate);
        inner_params.channels = Some(1);
        let inner = AacEncoder::new(&inner_params)?;
        let mut sbr = SbrEncoder::new(core_rate)?;
        sbr.set_emit_ps(true);
        // Output params — we advertise 2 output channels even though the
        // ADTS stream is mono SCE, because the decoder upmixes via PS.
        let mut out_params = params.clone();
        out_params.media_type = MediaType::Audio;
        out_params.sample_format = Some(SampleFormat::S16);
        out_params.channels = Some(2);
        out_params.sample_rate = Some(out_rate);
        Ok(Self {
            inner,
            sbr,
            downsampler: Downsampler::new(),
            qmf_l: QmfAnalysis64::new(),
            qmf_r: QmfAnalysis64::new(),
            high_pcm_l: Vec::with_capacity(HIGH_RATE_FRAME * 2),
            high_pcm_r: Vec::with_capacity(HIGH_RATE_FRAME * 2),
            flushed: false,
            core_rate,
            out_rate,
            out_params,
        })
    }

    fn push_audio(&mut self, frame: &AudioFrame) -> Result<()> {
        if frame.channels != 2 {
            return Err(Error::invalid("HE-AACv2 encoder: expected 2-channel input"));
        }
        let n = frame.samples as usize;
        if n == 0 {
            return Ok(());
        }
        let plane = frame
            .data
            .first()
            .ok_or_else(|| Error::invalid("HE-AAC encoder: missing data plane"))?;
        match frame.format {
            SampleFormat::S16 => {
                let stride = 4usize; // 2 channels * 2 bytes
                if plane.len() < n * stride {
                    return Err(Error::invalid("HE-AAC encoder: S16 frame too short"));
                }
                for i in 0..n {
                    let off = i * stride;
                    let l = i16::from_le_bytes([plane[off], plane[off + 1]]) as f32 / 32768.0;
                    let r = i16::from_le_bytes([plane[off + 2], plane[off + 3]]) as f32 / 32768.0;
                    self.high_pcm_l.push(l);
                    self.high_pcm_r.push(r);
                }
            }
            SampleFormat::F32 => {
                let stride = 8usize;
                if plane.len() < n * stride {
                    return Err(Error::invalid("HE-AAC encoder: F32 frame too short"));
                }
                for i in 0..n {
                    let off = i * stride;
                    let l = f32::from_le_bytes([
                        plane[off],
                        plane[off + 1],
                        plane[off + 2],
                        plane[off + 3],
                    ]);
                    let r = f32::from_le_bytes([
                        plane[off + 4],
                        plane[off + 5],
                        plane[off + 6],
                        plane[off + 7],
                    ]);
                    self.high_pcm_l.push(l);
                    self.high_pcm_r.push(r);
                }
            }
            other => {
                return Err(Error::unsupported(format!(
                    "HE-AACv2 encoder: sample format {other:?} not supported"
                )));
            }
        }
        Ok(())
    }

    fn drain_frames(&mut self) -> Result<()> {
        while self.high_pcm_l.len() >= HIGH_RATE_FRAME && self.high_pcm_r.len() >= HIGH_RATE_FRAME {
            self.emit_frame(false)?;
        }
        Ok(())
    }

    /// Run the per-channel high-rate QMF analysis on one frame's worth of
    /// L / R PCM. Returns the two `[NUM_TIME_SLOTS_1024 * RATE][64]`
    /// complex matrices used for PS analysis.
    fn analyse_lr(
        &mut self,
        l: &[f32],
        r: &[f32],
    ) -> (
        Vec<[Complex32; NUM_QMF_BANDS]>,
        Vec<[Complex32; NUM_QMF_BANDS]>,
    ) {
        let num_slots = NUM_TIME_SLOTS_1024 * crate::sbr::RATE; // 32
        let mut x_l = vec![[Complex32::default(); NUM_QMF_BANDS]; num_slots];
        let mut x_r = vec![[Complex32::default(); NUM_QMF_BANDS]; num_slots];
        let mut tmp = [0.0f32; 64];
        for slot in 0..num_slots {
            tmp.copy_from_slice(&l[slot * 64..slot * 64 + 64]);
            self.qmf_l.process(&tmp, &mut x_l[slot]);
            tmp.copy_from_slice(&r[slot * 64..slot * 64 + 64]);
            self.qmf_r.process(&tmp, &mut x_r[slot]);
        }
        (x_l, x_r)
    }

    fn emit_frame(&mut self, flush: bool) -> Result<()> {
        let have = self.high_pcm_l.len().min(self.high_pcm_r.len());
        if have == 0 {
            return Ok(());
        }
        let to_take = have.min(HIGH_RATE_FRAME);
        let mut block_l = vec![0.0f32; HIGH_RATE_FRAME];
        let mut block_r = vec![0.0f32; HIGH_RATE_FRAME];
        block_l[..to_take].copy_from_slice(&self.high_pcm_l[..to_take]);
        block_r[..to_take].copy_from_slice(&self.high_pcm_r[..to_take]);
        self.high_pcm_l.drain(..to_take);
        self.high_pcm_r.drain(..to_take);

        // 1) Per-channel QMF analysis → real PS parameters.
        //    Round-14: detect transients on the QMF energy profile to pick
        //    `num_env ∈ {1, 2, 4}` (§8.6.4.6.2 / Table 8.29). Stationary
        //    content stays at the round-13 single-envelope path.
        let (x_l, x_r) = self.analyse_lr(&block_l, &block_r);
        let num_env = detect_num_env(&x_l, &x_r);
        let ps_frame = analyse_ps_params_10_multi_env(&x_l, &x_r, num_env);
        self.sbr.set_pending_ps_frame(ps_frame);

        // 2) Downmix to mono (L+R averaged with 0.5 attenuation) for the
        //    SBR mono analysis path. The SBR scalefactors describe the
        //    high-band energy of the *mono* signal; the PS upmix at the
        //    decoder then redistributes that energy to L vs R according
        //    to IID and adjusts coherence according to ICC.
        let mut block_mono = vec![0.0f32; HIGH_RATE_FRAME];
        for i in 0..HIGH_RATE_FRAME {
            block_mono[i] = 0.5 * (block_l[i] + block_r[i]);
        }

        // 3) SBR analysis on the mono signal + emit SCE+PS payload.
        let x_high = self.sbr.analyse(&block_mono);
        let (sbr_bytes, bits) = self.sbr.emit_sbr_payload(&x_high);
        self.inner.stage_sbr_fil(SbrFilBits {
            bytes: sbr_bytes,
            bits,
        });

        // 4) Downsample 2x → 1x for the AAC-LC core (mono SCE).
        let mut low = vec![0.0f32; HIGH_RATE_FRAME / 2];
        self.downsampler.process(&block_mono, &mut low);
        let mut bytes = Vec::with_capacity(low.len() * 4);
        for v in &low {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        let af = AudioFrame {
            format: SampleFormat::F32,
            channels: 1,
            sample_rate: self.core_rate,
            samples: low.len() as u32,
            pts: None,
            time_base: oxideav_core::TimeBase::new(1, self.core_rate as i64),
            data: vec![bytes],
        };
        self.inner.send_frame(&Frame::Audio(af))?;
        if flush {
            self.inner.flush()?;
        }
        Ok(())
    }

    /// Output sample rate (= 2 × core rate).
    pub fn output_sample_rate(&self) -> u32 {
        self.out_rate
    }
}

impl Encoder for HeAacV2Encoder {
    fn codec_id(&self) -> &CodecId {
        self.inner.codec_id()
    }

    fn output_params(&self) -> &CodecParameters {
        &self.out_params
    }

    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        if self.flushed {
            return Err(Error::other(
                "HE-AACv2 encoder: flushed, cannot accept more frames",
            ));
        }
        match frame {
            Frame::Audio(af) => {
                self.push_audio(af)?;
                self.drain_frames()
            }
            _ => Err(Error::invalid("HE-AACv2 encoder: expected audio frame")),
        }
    }

    fn receive_packet(&mut self) -> Result<Packet> {
        self.inner.receive_packet()
    }

    fn flush(&mut self) -> Result<()> {
        if self.flushed {
            return Ok(());
        }
        let have = self.high_pcm_l.len().min(self.high_pcm_r.len());
        if have > 0 {
            if self.high_pcm_l.len() < HIGH_RATE_FRAME {
                self.high_pcm_l.resize(HIGH_RATE_FRAME, 0.0);
            }
            if self.high_pcm_r.len() < HIGH_RATE_FRAME {
                self.high_pcm_r.resize(HIGH_RATE_FRAME, 0.0);
            }
            self.emit_frame(true)?;
        } else {
            self.inner.flush()?;
        }
        self.flushed = true;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::TimeBase;

    #[test]
    fn he_aac_encoder_emits_adts_frames() {
        let mut params = CodecParameters::audio(CodecId::new("aac"));
        params.sample_rate = Some(48_000);
        params.channels = Some(1);
        params.bit_rate = Some(32_000);
        let mut enc = HeAacMonoEncoder::new(&params).expect("construct");
        // 1 kHz sine at 48 kHz for 200 ms.
        let sr = 48_000u32;
        let secs = 0.2;
        let n = (sr as f32 * secs) as usize;
        let mut bytes = Vec::with_capacity(n * 2);
        for i in 0..n {
            let t = i as f32 / sr as f32;
            let v = (2.0 * std::f32::consts::PI * 1000.0 * t).sin() * 0.3;
            let s = (v * 32767.0) as i16;
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        let af = AudioFrame {
            format: SampleFormat::S16,
            channels: 1,
            sample_rate: sr,
            samples: n as u32,
            pts: Some(0),
            time_base: TimeBase::new(1, sr as i64),
            data: vec![bytes],
        };
        enc.send_frame(&Frame::Audio(af)).expect("send_frame");
        enc.flush().expect("flush");
        let mut total = 0;
        while let Ok(pkt) = enc.receive_packet() {
            assert!(pkt.data.len() > 7);
            // ADTS syncword.
            assert_eq!(pkt.data[0], 0xFF);
            assert_eq!(pkt.data[1] & 0xF0, 0xF0);
            total += 1;
        }
        assert!(total >= 3, "expected >= 3 HE-AAC frames, got {total}");
    }

    #[test]
    fn he_aac_stereo_encoder_emits_adts_frames() {
        let mut params = CodecParameters::audio(CodecId::new("aac"));
        params.sample_rate = Some(48_000);
        params.channels = Some(2);
        params.bit_rate = Some(64_000);
        let mut enc = HeAacStereoEncoder::new(&params).expect("construct");
        // L = 1 kHz, R = 1500 Hz at 48 kHz for 200 ms.
        let sr = 48_000u32;
        let secs = 0.2;
        let n = (sr as f32 * secs) as usize;
        let mut bytes = Vec::with_capacity(n * 4);
        for i in 0..n {
            let t = i as f32 / sr as f32;
            let l = (2.0 * std::f32::consts::PI * 1000.0 * t).sin() * 0.3;
            let r = (2.0 * std::f32::consts::PI * 1500.0 * t).sin() * 0.3;
            let sl = (l * 32767.0) as i16;
            let sr_s = (r * 32767.0) as i16;
            bytes.extend_from_slice(&sl.to_le_bytes());
            bytes.extend_from_slice(&sr_s.to_le_bytes());
        }
        let af = AudioFrame {
            format: SampleFormat::S16,
            channels: 2,
            sample_rate: sr,
            samples: n as u32,
            pts: Some(0),
            time_base: TimeBase::new(1, sr as i64),
            data: vec![bytes],
        };
        enc.send_frame(&Frame::Audio(af)).expect("send_frame");
        enc.flush().expect("flush");
        let mut total = 0;
        while let Ok(pkt) = enc.receive_packet() {
            assert!(pkt.data.len() > 7);
            assert_eq!(pkt.data[0], 0xFF);
            assert_eq!(pkt.data[1] & 0xF0, 0xF0);
            // ADTS channel_configuration nibble — for 2 channels this is 2.
            // Layout: byte[2] low bit = (cc >> 2) & 1; byte[3] top 2 bits = cc & 0x3.
            let cc_hi = (pkt.data[2] & 0x01) << 2;
            let cc_lo = (pkt.data[3] >> 6) & 0x03;
            assert_eq!(cc_hi | cc_lo, 2, "expected channel_configuration=2");
            total += 1;
        }
        assert!(
            total >= 3,
            "expected >= 3 HE-AAC stereo frames, got {total}"
        );
    }
}

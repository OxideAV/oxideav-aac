//! HE-AACv1 mono encoder wrapper.
//!
//! Wraps [`crate::encoder::AacEncoder`] (configured for 1-channel) and
//! [`crate::sbr::encode::SbrEncoder`] to produce a combined AAC-LC core +
//! SBR stream. The caller feeds PCM at **2×** the target AAC-LC sample
//! rate; the wrapper:
//!
//! 1. Downsamples the 2× PCM by two into the inner AAC-LC encoder.
//! 2. Runs the 64-channel complex analysis QMF on the original 2× PCM
//!    and builds the SBR scalefactor payload.
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
//! Only mono + core rates that are in the AAC-LC sample-rate table are
//! supported (typically 24 kHz → 48 kHz doubled output, or 22.05 kHz →
//! 44.1 kHz).

use oxideav_core::Encoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, MediaType, Packet, Result, SampleFormat,
};

use crate::encoder::{AacEncoder, SbrFilBits};
use crate::sbr::encode::{Downsampler, SbrEncoder};

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
}

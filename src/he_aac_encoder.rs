//! HE-AAC v1 encoder — AAC-LC core at half rate + the §4.6.18 SBR tool,
//! Annex 4.B.18.1 (Figure 4.B.15) written forward.
//!
//! [`HeAacEncoder`] takes full-rate interleaved S16 PCM in hops of
//! `2 × 1024` samples per channel and emits one ADTS frame per hop:
//!
//! 1. **Downsampler** — a linear-phase FIR low-pass (windowed sinc,
//!    [`FIR_TAPS`] taps) cut at the SBR crossover frequency, decimated
//!    2:1, feeds the [`crate::encoder::StreamEncoder`] AAC-LC core at
//!    `fs/2`. The core therefore spends its bits below the crossover
//!    only — the decoder discards core content above `k_x` anyway
//!    (`XLow(k) = 0` for `k ≥ k_x`).
//! 2. **Analysis QMF bank** — the full-rate input, delayed by the FIR's
//!    group delay so it stays sample-aligned with the core signal, runs
//!    through the 64-band [`crate::sbr_qmf::EncoderAnalysisQmf`]
//!    continuously; a ring of the most recent columns is sliced per
//!    frame into the [`crate::sbr_encoder::SBR_ENC_COLS`] window the
//!    parameter encoder expects (see the alignment note there).
//! 3. **SBR parameter encoder** — one [`crate::sbr_encoder::SbrEncoder`]
//!    per channel element (SCE or CPE) yields the `EXT_SBR_DATA` fill
//!    payload for the frame.
//! 4. **Core encode with fills** — the core encoder assembles the
//!    `raw_data_block()` with the SBR `fill_element()` right after its
//!    channel element, charging the payload against the frame budget,
//!    and wraps it in the ADTS header carrying the *core* rate and the
//!    LC profile — the implicit SBR signalling of §4.5.2.8.1 / Annex
//!    1.A, which any HE-AAC decoder (this crate's included) detects
//!    from the fill element itself.
//!
//! ## Timing
//!
//! The core encoder's decoded frame `f` reconstructs its input hop
//! `f − 1`; the SBR payload in ADTS frame `f` accordingly describes the
//! analysis columns of hop `f − 1` (with the 8-slot lead-in the decoder
//! takes from the previous block and the lookahead a variable trailing
//! border may reach into hop `f`). The complete encode → decode path
//! delays the audio by one hop plus the decoder's SBR synchronisation
//! offset and the filterbank pair's group delay; the round-trip tests
//! measure it (about 1.1 hops at the output rate) rather than assume
//! it.
//!
//! ## Explicit signalling
//!
//! [`HeAacEncoder::audio_specific_config`] returns the Table 1.15
//! `AudioSpecificConfig` for MP4 / LATM carriage in either the
//! backward-compatible form (AAC-LC + trailing `syncExtensionType
//! 0x2b7` SBR extension) or the hierarchical form
//! (`audioObjectType = 5`); [`crate::latm_writer`] wraps the ADTS
//! payloads into LOAS/LATM with that configuration.

use std::collections::VecDeque;

use crate::adts::ADTS_SAMPLE_RATES_HZ;
use crate::encoder::{EncoderConfig, StreamEncoder, FRAME_LEN};
use crate::sbr_encoder::{SbrEncoder, SbrEncoderConfig, SbrFrame, RATE, SBR_ENC_COLS, T_HF_GEN};
use crate::sbr_qmf::{Complex, EncoderAnalysisQmf};
use crate::{Error, Result};

/// Full-rate samples per encoder hop per channel (`2 × 1024`).
pub const HE_FRAME_LEN: usize = 2 * FRAME_LEN;

/// Taps of the downsampling low-pass (odd, so the group delay
/// `(FIR_TAPS − 1) / 2` is a whole number of full-rate samples — and
/// even, so it is a whole number of core samples too).
pub const FIR_TAPS: usize = 65;

/// The FIR group delay in full-rate samples.
pub const FIR_DELAY: usize = (FIR_TAPS - 1) / 2;

/// Configuration for [`HeAacEncoder`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeAacConfig {
    /// Output (SBR) sample rate in Hz. Its half must be a Table 1.18
    /// ADTS rate with a §4.5.4 band table, and the rate itself one of
    /// the §4.6.18.3.2.1 SBR rates: 16 000 … 96 000 Hz.
    pub sample_rate: u32,
    /// `1` (mono, SCE) or `2` (stereo, CPE).
    pub channels: u8,
    /// Target total bitrate in bits/second (core + SBR side info).
    pub bitrate: u32,
    /// SBR crossover frequency in Hz (`None` = chosen from the
    /// bitrate per channel).
    pub crossover_hz: Option<f64>,
    /// Emit `EXT_SBR_DATA_CRC` payloads.
    pub sbr_crc: bool,
    /// Enable the `bs_add_harmonic` sinusoid detector.
    pub add_harmonic: bool,
    /// Code a stereo pair with `bs_coupling` (level + balance).
    pub coupling: bool,
    /// Transmit `sbr_header()` every N frames.
    pub header_interval: u32,
    /// `bs_interpol_freq` — see
    /// [`crate::sbr_encoder::SbrEncoderConfig::interpol_freq`].
    pub interpol_freq: bool,
}

impl HeAacConfig {
    /// A configuration at `sample_rate` / `channels` / `bitrate` with
    /// every SBR option at its default (crossover from the bitrate, no
    /// CRC, no add-harmonic, independent stereo, header every 8
    /// frames).
    pub fn new(sample_rate: u32, channels: u8, bitrate: u32) -> Self {
        HeAacConfig {
            sample_rate,
            channels,
            bitrate,
            crossover_hz: None,
            sbr_crc: false,
            add_harmonic: true,
            coupling: false,
            header_interval: 8,
            interpol_freq: true,
        }
    }

    /// The crossover frequency in force: the explicit one, else a
    /// bitrate-driven choice — `0.115·fs·√(bps_per_channel / 24 000)`
    /// (≈ 4.1 kHz at 16 kbps/ch and 5.9 kHz at 32 kbps/ch for 44.1 kHz
    /// output), kept within `[0.09·fs, 0.22·fs]` so the core always
    /// carries a band below it and the SBR range is never empty.
    pub fn crossover(&self) -> f64 {
        let fs = f64::from(self.sample_rate);
        self.crossover_hz.unwrap_or_else(|| {
            let per_ch = f64::from(self.bitrate) / f64::from(self.channels.max(1));
            (0.115 * fs * (per_ch / 24_000.0).sqrt()).clamp(0.09 * fs, 0.22 * fs)
        })
    }

    /// The SBR stop frequency: `min(0.36·fs, 16.5 kHz)` — the upper
    /// end of the reconstructed band.
    pub fn stop_hz(&self) -> f64 {
        (0.36 * f64::from(self.sample_rate)).min(16_500.0)
    }
}

/// Windowed-sinc low-pass FIR with cutoff `fc` (cycles per sample,
/// `0 < fc < 0.5`), [`FIR_TAPS`] taps, unity DC gain.
fn design_lowpass(fc: f64) -> Vec<f64> {
    let n = FIR_TAPS;
    let m = (n - 1) as f64 / 2.0;
    let mut h: Vec<f64> = (0..n)
        .map(|i| {
            let x = i as f64 - m;
            let sinc = if x == 0.0 {
                2.0 * fc
            } else {
                (2.0 * core::f64::consts::PI * fc * x).sin() / (core::f64::consts::PI * x)
            };
            // Blackman window.
            let w = 0.42 - 0.5 * (2.0 * core::f64::consts::PI * i as f64 / (n - 1) as f64).cos()
                + 0.08 * (4.0 * core::f64::consts::PI * i as f64 / (n - 1) as f64).cos();
            sinc * w
        })
        .collect();
    let sum: f64 = h.iter().sum();
    for v in h.iter_mut() {
        *v /= sum;
    }
    h
}

/// Per-channel front-end state.
#[derive(Debug, Clone)]
struct ChannelFrontEnd {
    /// The last `FIR_TAPS − 1` full-rate input samples (FIR history).
    fir_hist: Vec<f64>,
    /// The last `FIR_DELAY` input samples not yet fed to the QMF bank
    /// (the delay line aligning the analysis with the core signal).
    qmf_delay: Vec<f64>,
    bank: EncoderAnalysisQmf,
    /// The most recent analysis columns, newest last.
    cols: VecDeque<[Complex; 64]>,
}

/// Columns kept per channel: the frame window starts
/// `RATE·32 + 2·8 + …` behind the newest column — see
/// [`HeAacEncoder::frame_window`].
const RING_COLS: usize = 96;

impl ChannelFrontEnd {
    fn new() -> Self {
        let mut cols = VecDeque::with_capacity(RING_COLS + 32);
        for _ in 0..RING_COLS {
            cols.push_back([Complex::default(); 64]);
        }
        ChannelFrontEnd {
            fir_hist: vec![0.0; FIR_TAPS - 1],
            qmf_delay: vec![0.0; FIR_DELAY],
            bank: EncoderAnalysisQmf::new(),
            cols,
        }
    }
}

/// The HE-AAC v1 (AAC-LC + SBR) streaming encoder.
#[derive(Debug, Clone)]
pub struct HeAacEncoder {
    config: HeAacConfig,
    core: StreamEncoder,
    sbr: SbrEncoder,
    fir: Vec<f64>,
    fe: Vec<ChannelFrontEnd>,
    frames: u64,
    /// The last SBR frame's diagnostics.
    last_sbr: Option<SbrFrame>,
}

impl HeAacEncoder {
    /// Build an encoder for `config`.
    ///
    /// Errors with [`Error::EncoderInvalidConfig`] when the rate has no
    /// half-rate ADTS core / SBR table, the channel count is not 1 or
    /// 2, or the bitrate is 0.
    pub fn new(config: HeAacConfig) -> Result<Self> {
        if !(1..=2).contains(&config.channels) || config.bitrate == 0 {
            return Err(Error::EncoderInvalidConfig);
        }
        if config.sample_rate % 2 != 0 {
            return Err(Error::EncoderInvalidConfig);
        }
        let core_rate = config.sample_rate / 2;
        if !ADTS_SAMPLE_RATES_HZ.contains(&core_rate) {
            return Err(Error::EncoderInvalidConfig);
        }
        let mut sbr_cfg = SbrEncoderConfig::new(
            config.sample_rate,
            usize::from(config.channels),
            config.crossover(),
            config.stop_hz(),
        )?;
        sbr_cfg.crc = config.sbr_crc;
        sbr_cfg.add_harmonic = config.add_harmonic;
        sbr_cfg.coupling = config.coupling;
        sbr_cfg.header_interval = config.header_interval;
        sbr_cfg.interpol_freq = config.interpol_freq;
        let sbr = SbrEncoder::new(sbr_cfg)?;
        // The core is band-limited to the actual SBR start (k_x), which
        // the header rounding may have moved from the requested
        // crossover.
        let k_x = f64::from(sbr.bands().k_x);
        let fs = f64::from(config.sample_rate);
        let xo_hz = (k_x * fs / 128.0).min(0.225 * fs);
        let fir = design_lowpass(xo_hz / fs);
        let core = StreamEncoder::new(EncoderConfig {
            sample_rate: core_rate,
            channels: config.channels,
            bitrate: config.bitrate,
        })?;
        let n = usize::from(config.channels);
        Ok(HeAacEncoder {
            config,
            core,
            sbr,
            fir,
            fe: (0..n).map(|_| ChannelFrontEnd::new()).collect(),
            frames: 0,
            last_sbr: None,
        })
    }

    /// The configuration.
    pub fn config(&self) -> &HeAacConfig {
        &self.config
    }

    /// The SBR parameter encoder (band tables, header).
    pub fn sbr(&self) -> &SbrEncoder {
        &self.sbr
    }

    /// The core encoder's ADTS sample rate (`sample_rate / 2`).
    pub fn core_sample_rate(&self) -> u32 {
        self.config.sample_rate / 2
    }

    /// Diagnostics of the most recently encoded SBR frame.
    pub fn last_sbr_frame(&self) -> Option<&SbrFrame> {
        self.last_sbr.as_ref()
    }

    /// Run the front end over one channel's full-rate hop: returns the
    /// decimated core samples and pushes 32 analysis columns.
    fn front_end(&mut self, c: usize, hop: &[f64]) -> Vec<f64> {
        let taps = self.fir.len();
        let fe = &mut self.fe[c];
        // Contiguous [history | hop] for the FIR.
        let mut buf = Vec::with_capacity(taps - 1 + hop.len());
        buf.extend_from_slice(&fe.fir_hist);
        buf.extend_from_slice(hop);
        // y[m] = Σ_j h[j] · x[2m − j] over the causal window (the
        // output sample m at index 2m + taps − 1 in `buf`).
        let mut core = Vec::with_capacity(hop.len() / 2);
        for m in 0..hop.len() / 2 {
            let end = 2 * m + taps - 1;
            let mut acc = 0.0;
            for (j, &h) in self.fir.iter().enumerate() {
                acc += h * buf[end - j];
            }
            core.push(acc);
        }
        let keep = buf.len() - (taps - 1);
        fe.fir_hist.clear();
        fe.fir_hist.extend_from_slice(&buf[keep..]);

        // Delayed input into the analysis bank.
        let mut delayed = Vec::with_capacity(hop.len());
        delayed.extend_from_slice(&fe.qmf_delay);
        delayed.extend_from_slice(&hop[..hop.len() - FIR_DELAY]);
        fe.qmf_delay.clear();
        fe.qmf_delay
            .extend_from_slice(&hop[hop.len() - FIR_DELAY..]);
        for slot in delayed.chunks_exact(64) {
            let x = fe.bank.push_slot(slot).expect("64-sample slot");
            fe.cols.push_back(x);
            if fe.cols.len() > RING_COLS {
                fe.cols.pop_front();
            }
        }
        core
    }

    /// The [`SBR_ENC_COLS`] analysis columns for the frame being
    /// emitted: column 0 is core slot `−tHFGen` of the core block the
    /// decoder pairs with this ADTS frame (hop `f − 1`), i.e. it starts
    /// `32 + 32 + 8 = 72` columns before the newest pushed column.
    fn frame_window(&self, c: usize) -> Vec<[Complex; 64]> {
        let cols = &self.fe[c].cols;
        let newest = cols.len();
        let start = newest - (2 * RATE * 16 + T_HF_GEN);
        (start..start + SBR_ENC_COLS).map(|i| cols[i]).collect()
    }

    /// Encode one hop of `2048 × channels` interleaved full-rate
    /// samples (a shorter tail is zero-padded) into one ADTS frame.
    pub fn encode_frame(&mut self, interleaved: &[i16]) -> Result<Vec<u8>> {
        let ch = usize::from(self.config.channels);
        if interleaved.len() > HE_FRAME_LEN * ch || interleaved.len() % ch != 0 {
            return Err(Error::EncoderInvalidConfig);
        }
        let mut hops: Vec<Vec<f64>> = vec![vec![0.0; HE_FRAME_LEN]; ch];
        for (j, hop) in hops.iter_mut().enumerate() {
            for (n, v) in hop.iter_mut().take(interleaved.len() / ch).enumerate() {
                *v = f64::from(interleaved[n * ch + j]);
            }
        }
        self.encode_hops(&hops)
    }

    fn encode_hops(&mut self, hops: &[Vec<f64>]) -> Result<Vec<u8>> {
        let ch = hops.len();
        // Front end: decimated core PCM + fresh analysis columns.
        let mut core_pcm: Vec<Vec<f64>> = Vec::with_capacity(ch);
        for (c, hop) in hops.iter().enumerate() {
            core_pcm.push(self.front_end(c, hop));
        }
        // SBR side info for the core block this frame decodes to.
        let windows: Vec<Vec<[Complex; 64]>> = (0..ch).map(|c| self.frame_window(c)).collect();
        let refs: Vec<&[[Complex; 64]]> = windows.iter().map(|w| w.as_slice()).collect();
        let sbr_frame = self.sbr.encode_frame(&refs)?;
        let fills = vec![sbr_frame.payload.clone()];
        self.last_sbr = Some(sbr_frame);
        // Core encode (interleaved S16 on the ±32768 axis).
        let mut inter = Vec::with_capacity(FRAME_LEN * ch);
        for n in 0..FRAME_LEN {
            for pcm in core_pcm.iter() {
                inter.push(crate::pcm::to_s16(pcm[n]));
            }
        }
        self.frames += 1;
        self.core.encode_frame_with_fills(&inter, &fills)
    }

    /// Flush: one trailing frame covering the final core overlap (and
    /// the SBR data for the last real hop).
    pub fn finish(&mut self) -> Result<Vec<u8>> {
        let ch = usize::from(self.config.channels);
        let zeros: Vec<Vec<f64>> = vec![vec![0.0; HE_FRAME_LEN]; ch];
        self.encode_hops(&zeros)
    }

    /// One-shot: encode a whole interleaved buffer to a complete ADTS
    /// stream (`⌈n / 2048⌉ + 1` frames including the flush).
    pub fn encode_all(&mut self, interleaved: &[i16]) -> Result<Vec<u8>> {
        let ch = usize::from(self.config.channels);
        if interleaved.len() % ch != 0 {
            return Err(Error::EncoderInvalidConfig);
        }
        let mut out = Vec::new();
        let hop = HE_FRAME_LEN * ch;
        let mut chunks = interleaved.chunks(hop);
        let first = chunks.next().unwrap_or(&[]);
        out.extend_from_slice(&self.encode_frame(first)?);
        for chunk in chunks {
            out.extend_from_slice(&self.encode_frame(chunk)?);
        }
        out.extend_from_slice(&self.finish()?);
        Ok(out)
    }

    /// The Table 1.15 `AudioSpecificConfig` describing this stream.
    ///
    /// * `hierarchical == false` — backward-compatible signalling: the
    ///   AAC-LC ASC (`audioObjectType = 2`, the core rate, the channel
    ///   configuration, `GASpecificConfig`) followed by the
    ///   `syncExtensionType = 0x2b7` / `extensionAudioObjectType = 5`
    ///   / `sbrPresentFlag = 1` / `extensionSamplingFrequencyIndex`
    ///   trailer. A legacy AAC-LC decoder ignores the trailer and
    ///   plays the core at half rate.
    /// * `hierarchical == true` — `audioObjectType = 5` up front with
    ///   `extensionSamplingFrequencyIndex` (the SBR rate) and the core
    ///   `audioObjectType = 2` behind it.
    pub fn audio_specific_config(&self, hierarchical: bool) -> Vec<u8> {
        crate::asc_writer::he_aac_v1_asc(
            self.core_sample_rate(),
            self.config.sample_rate,
            self.config.channels,
            hierarchical,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_validation() {
        assert!(HeAacEncoder::new(HeAacConfig::new(44_100, 1, 32_000)).is_ok());
        assert!(HeAacEncoder::new(HeAacConfig::new(48_000, 2, 64_000)).is_ok());
        assert!(HeAacEncoder::new(HeAacConfig::new(44_100, 3, 32_000)).is_err());
        assert!(HeAacEncoder::new(HeAacConfig::new(44_100, 1, 0)).is_err());
        // 44 055 / 2 is not an ADTS rate; 8 000 has no SBR table.
        assert!(HeAacEncoder::new(HeAacConfig::new(44_055, 1, 32_000)).is_err());
        assert!(HeAacEncoder::new(HeAacConfig::new(8_000, 1, 32_000)).is_err());
    }

    #[test]
    fn crossover_tracks_bitrate() {
        let lo = HeAacConfig::new(44_100, 2, 32_000).crossover();
        let hi = HeAacConfig::new(44_100, 2, 128_000).crossover();
        assert!(lo < hi);
        assert!(lo >= 0.09 * 44_100.0 && hi <= 0.22 * 44_100.0);
        let explicit = HeAacConfig {
            crossover_hz: Some(7_000.0),
            ..HeAacConfig::new(44_100, 2, 32_000)
        };
        assert_eq!(explicit.crossover(), 7_000.0);
    }

    #[test]
    fn lowpass_is_unity_dc_and_attenuates_above_cutoff() {
        let h = design_lowpass(0.125);
        assert!((h.iter().sum::<f64>() - 1.0).abs() < 1e-12);
        // Frequency response at DC, passband, stopband.
        let resp = |f: f64| -> f64 {
            let (mut re, mut im) = (0.0, 0.0);
            for (i, &v) in h.iter().enumerate() {
                let a = 2.0 * core::f64::consts::PI * f * i as f64;
                re += v * a.cos();
                im -= v * a.sin();
            }
            (re * re + im * im).sqrt()
        };
        assert!((resp(0.02) - 1.0).abs() < 0.02);
        assert!(resp(0.25) < 0.01, "stopband {}", resp(0.25));
    }

    /// The emitted frames are LC-profile ADTS at the core rate with an
    /// SBR fill element the crate's own decoder detects: it emits
    /// 2048-sample frames at the full rate.
    #[test]
    fn frames_decode_as_sbr_active_at_full_rate() {
        let mut enc = HeAacEncoder::new(HeAacConfig::new(44_100, 1, 40_000)).unwrap();
        let n = 4 * HE_FRAME_LEN;
        let pcm: Vec<i16> = (0..n)
            .map(|i| (6000.0 * (0.02 * i as f64).sin() + 3000.0 * (0.9 * i as f64).sin()) as i16)
            .collect();
        let stream = enc.encode_all(&pcm).unwrap();
        let (h, _) = crate::adts::AdtsHeader::parse(&stream).unwrap();
        assert_eq!(h.sample_rate(), 22_050);
        assert_eq!(h.profile, 1);
        let mut dec = crate::decode::StreamDecoder::new();
        let frames = dec.decode_all(&stream).unwrap();
        assert_eq!(frames.len(), 5);
        assert!(frames.iter().all(|f| f.sample_rate == 44_100));
        assert!(frames.iter().all(|f| f.pcm.len() == HE_FRAME_LEN));
        assert!(enc.last_sbr_frame().is_some());
    }
}

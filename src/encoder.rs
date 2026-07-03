//! End-to-end AAC-LC encoder — ISO/IEC 14496-3 §4.5/§4.6 written
//! forward.
//!
//! This module drives the crate's Phase-2 bit-exact wire writers
//! ([`crate::ics_body::IcsBody::write`],
//! [`crate::section_data::SectionData::write`],
//! [`crate::scale_factor_data::ScaleFactorData::write`],
//! [`crate::spectral_data::SpectralData::write`],
//! [`crate::raw_data_block::FrameAssembler`],
//! [`crate::adts::AdtsHeader::write`]) from PCM input, producing an
//! ADTS stream the crate's own [`crate::decode::StreamDecoder`]
//! round-trips.
//!
//! ## What the wire format fixes vs. what the encoder chooses
//!
//! ISO/IEC 14496-3 normatively defines the *decoder*: the §4.6.2
//! inverse quantizer `x = sign(q)·|q|^(4/3)·2^(0.25·(sf−100))`, the
//! §4.6.3 noiseless coding, and the §4.6.11 filterbank. Everything on
//! the analysis side — the psychoacoustic model, the
//! scalefactor/quantizer search, the codebook choice — is an encoder
//! degree of freedom; any choice that yields conforming syntax is a
//! conforming encoder. The choices here are deliberately simple and
//! fully derived from the normative decoder equations:
//!
//! * **Window decision** — every frame is `ONLY_LONG_SEQUENCE` with
//!   the §4.6.11.3.2 sine shape. (Block switching is a quality
//!   refinement, not a conformance requirement: a long-window-only
//!   stream is valid AAC-LC.)
//! * **Analysis filterbank** — the §4.6.11.3.1 forward MDCT (the
//!   transform whose windowed overlap-add against the decoder's IMDCT
//!   is unity — the same [`crate::filterbank::forward_mdct`] the
//!   §4.6.7 LTP loop uses). Frame `f` transforms input samples
//!   `[f·1024 − 1024, f·1024 + 1024)`; the leading frame is primed
//!   with zeros, giving the standard 1024-sample encoder delay.
//! * **Quantizer** — the exact inverse of §4.6.2:
//!   `q = round((|x| / 2^(0.25·(sf−100)))^(3/4))`, with
//!   round-half-away-from-zero (the §1.3 `NINT` convention).
//! * **Psychoacoustics-lite** — a masking-spread rule: each
//!   scalefactor band's `sf` is chosen so the band's *peak*
//!   coefficient quantizes to a target magnitude
//!   `M_b = M · (peak_b / peak_frame)^½` (`sf = 100 + 4·log2(peak_b)
//!   − (16/3)·log2(M_b)`, the inversion of the dequant gain ladder).
//!   The square-root spread interpolates between constant-SNR
//!   (every band equally precise relative to itself — wasteful on
//!   the leakage skirts of tonal signals) and a flat noise floor
//!   (all precision on the loudest band): a band 40 dB below the
//!   frame peak is quantized ~20 dB more coarsely, and a band whose
//!   target falls below one quantizer step is culled to `ZERO_HCB`
//!   outright — a first-order simultaneous-masking model.
//! * **Rate loop** — an outer loop adds a uniform offset to every
//!   band's scalefactor (coarsening all quantizers by 1.5 dB per
//!   step, the §4.6.2.3.3 quarter-step ladder ×2) until the assembled
//!   frame fits the per-frame byte budget derived from the requested
//!   bitrate.
//! * **Codebook choice** — the smallest Table 4.95 codebook whose LAV
//!   covers the band's max |q| (`1 / 3 / 5 / 7 / 9 / 11`); adjacent
//!   bands with equal codebooks merge into one section.
//! * **Stereo** — a CPE with `common_window == 1` (one shared
//!   `ics_info()`), `ms_mask_present == 0`, and the two channels
//!   coded independently. No M/S, no intensity, no TNS, no PNS on the
//!   encode side yet.
//!
//! ## Conformance envelope
//!
//! The assembled frame respects the wire-format hard limits:
//! scalefactors clamp to `0..=255` (8-bit `global_gain` seed) with
//! consecutive DPCM deltas in `−60..=+60` (Table 4.A.1's codeword
//! range), quantized magnitudes cap at
//! [`crate::spectral_codebook::MAX_QUANT`] (8191, the §4.6.3.3 ESC
//! ceiling), and `aac_frame_length` stays within its 13-bit field.

use crate::adts::{AdtsHeader, ADTS_HEADER_BYTES_NO_CRC, ADTS_SAMPLE_RATES_HZ};
use crate::filterbank::{forward_mdct, long_only_window};
use crate::ics_body::IcsBody;
use crate::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use crate::raw_data_block::{FrameAssembler, IdSynEle};
use crate::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use crate::section_data::{Section, SectionData, ZERO_HCB};
use crate::spectral_codebook::MAX_QUANT;
use crate::spectral_data::SpectralData;
use crate::swb_offset::{long_window_offsets, LONG_WINDOW_LEN};
use crate::{Error, Result};

use oxideav_core::bits::BitWriter;

/// Historical direct-factory endpoint (the crate convention's
/// `<crate>::encoder::make_encoder` path) — re-exported from
/// [`crate::codec_encoder`].
pub use crate::codec_encoder::make_encoder;

/// Samples per channel per AAC frame (the 1024-line transform
/// family this crate implements).
pub const FRAME_LEN: usize = LONG_WINDOW_LEN as usize;

/// The long transform length `N = 2048`.
const LONG_TRANSFORM_LEN: usize = 2 * FRAME_LEN;

/// §4.6.2.3.3 `SF_OFFSET` — the scalefactor of unit gain.
const SF_OFFSET: i32 = 100;

/// Table 4.53 DPCM delta bound (the Table 4.A.1 codeword range).
const MAX_SF_DELTA: i32 = 60;

/// Target magnitude the *loudest* band's peak coefficient quantizes
/// to before the rate loop engages; quieter bands scale down with
/// the square-root masking spread.
const TARGET_PEAK_MAG: f64 = 42.0;

/// Masking-spread exponent: a band's target magnitude is
/// `TARGET_PEAK_MAG · (peak_b / peak_frame)^SPREAD`. `0` would be
/// constant-SNR, `1` a flat noise floor; `½` splits the difference.
const SPREAD: f64 = 0.5;

/// Cull threshold: a band whose spread target magnitude falls below
/// this fraction of one quantizer step carries no audible content
/// relative to the frame and is sent as `ZERO_HCB`.
const MIN_TARGET_MAG: f64 = 0.7;

/// Upper bound on rate-loop iterations. Each iteration coarsens
/// every quantizer by 3 dB (sf offset +4), so 48 iterations span
/// ~144 dB — beyond that the frame is all-zero anyway.
const MAX_RATE_ITERATIONS: usize = 48;

/// Deepest refinement the rate loop applies when a frame comes in
/// under budget: −32 scalefactors ≈ 24 dB of extra precision
/// (magnitudes ×2^6 over the [`TARGET_PEAK_MAG`] baseline).
const MAX_REFINE_OFFSET: i32 = 32;

/// Configuration for [`StreamEncoder`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderConfig {
    /// Output sample rate in Hz. Must be one of the ISO/IEC 14496-3
    /// Table 1.18 rates expressible in an ADTS header (index 0..=12).
    pub sample_rate: u32,
    /// Channel count: `1` (one SCE) or `2` (one CPE).
    pub channels: u8,
    /// Target bitrate in bits/second. Drives the per-frame byte
    /// budget of the rate loop. The output is not strictly CBR — each
    /// frame independently fits its budget — but averages at or below
    /// this rate.
    pub bitrate: u32,
}

impl EncoderConfig {
    /// Resolve `sample_rate` to its Table 1.18
    /// `sampling_frequency_index`. Index 12 (7350 Hz) is excluded:
    /// the §4.5.4 scalefactor-band tables only cover indices
    /// 0..=11 (7350 Hz content conventionally ships under index 11,
    /// see the staged corpus notes).
    fn fs_index(&self) -> Result<u8> {
        ADTS_SAMPLE_RATES_HZ
            .iter()
            .position(|&r| r == self.sample_rate)
            .filter(|&i| i < NUM_SWB_LONG_WINDOW.len())
            .map(|i| i as u8)
            .ok_or(Error::EncoderInvalidConfig)
    }

    /// Per-frame payload byte budget from the bitrate:
    /// `bitrate · 1024 / sample_rate` bits, minus the 7-byte ADTS
    /// header, floored at a minimum that always allows a syntactically
    /// valid (silent) frame.
    fn frame_budget_bytes(&self) -> usize {
        let bits = (self.bitrate as u64 * FRAME_LEN as u64) / self.sample_rate.max(1) as u64;
        let bytes = (bits / 8) as usize;
        bytes.saturating_sub(ADTS_HEADER_BYTES_NO_CRC).max(16)
    }
}

/// A streaming AAC-LC encoder producing one ADTS frame per
/// 1024-sample input hop.
///
/// Feed interleaved `i16` PCM via [`StreamEncoder::encode_all`] (one
/// shot), or drive [`StreamEncoder::encode_frame`] hop by hop and
/// finish with [`StreamEncoder::finish`] to flush the analysis
/// overlap. The encoder delay is exactly [`FRAME_LEN`] samples: the
/// first decoded frame of the round-trip is silence, and decoded
/// frame `f ≥ 1` reconstructs input hop `f − 1`.
#[derive(Debug, Clone)]
pub struct StreamEncoder {
    config: EncoderConfig,
    fs_index: u8,
    /// Per-channel previous input hop (the left half of the next
    /// analysis window), [`FRAME_LEN`] samples each. Starts all-zero
    /// (the priming frame).
    history: Vec<Vec<f64>>,
    /// The 2048-sample sine `ONLY_LONG` analysis window.
    window: Vec<f64>,
}

impl StreamEncoder {
    /// Build an encoder for `config`.
    ///
    /// Errors with [`Error::EncoderInvalidConfig`] when the sample
    /// rate is not a Table 1.18 ADTS rate, the channel count is not
    /// 1 or 2, or the bitrate is 0.
    pub fn new(config: EncoderConfig) -> Result<Self> {
        let fs_index = config.fs_index()?;
        if !(1..=2).contains(&config.channels) || config.bitrate == 0 {
            return Err(Error::EncoderInvalidConfig);
        }
        Ok(Self {
            config,
            fs_index,
            history: vec![vec![0.0; FRAME_LEN]; config.channels as usize],
            window: long_only_window(WindowShape::Sine, WindowShape::Sine),
        })
    }

    /// The configuration this encoder was built with.
    pub fn config(&self) -> &EncoderConfig {
        &self.config
    }

    /// Encode one 1024-sample-per-channel hop of interleaved `i16`
    /// PCM into one complete ADTS frame.
    ///
    /// `interleaved` must hold at most `1024 × channels` samples; a
    /// shorter slice (the stream tail) is zero-padded. The analysis
    /// window spans the previous hop and this one, so the emitted
    /// frame carries the overlap-add contribution of both.
    pub fn encode_frame(&mut self, interleaved: &[i16]) -> Result<Vec<u8>> {
        let ch = self.config.channels as usize;
        if interleaved.len() > FRAME_LEN * ch || interleaved.len() % ch != 0 {
            return Err(Error::EncoderInvalidConfig);
        }
        // De-interleave onto the ±32768 axis the §4.6.11 output
        // contract uses (no rescaling — the decoder's PCM stage
        // rounds these values back to i16 directly).
        let mut cur: Vec<Vec<f64>> = vec![vec![0.0; FRAME_LEN]; ch];
        for (i, &s) in interleaved.iter().enumerate() {
            cur[i % ch][i / ch] = f64::from(s);
        }
        let frame = self.encode_hop(&cur)?;
        self.history = cur;
        Ok(frame)
    }

    /// Flush the final analysis overlap: emits one trailing ADTS
    /// frame whose window covers the last real hop and a zero hop.
    pub fn finish(&mut self) -> Result<Vec<u8>> {
        let ch = self.config.channels as usize;
        let zeros: Vec<Vec<f64>> = vec![vec![0.0; FRAME_LEN]; ch];
        let frame = self.encode_hop(&zeros)?;
        self.history = zeros;
        Ok(frame)
    }

    /// One-shot convenience: encode a whole interleaved `i16` buffer
    /// to a complete ADTS stream (`⌈n/1024⌉ + 1` frames — the `+1`
    /// is the [`StreamEncoder::finish`] flush).
    pub fn encode_all(&mut self, interleaved: &[i16]) -> Result<Vec<u8>> {
        let ch = self.config.channels as usize;
        if interleaved.len() % ch != 0 {
            return Err(Error::EncoderInvalidConfig);
        }
        let mut out = Vec::new();
        let hop = FRAME_LEN * ch;
        let mut chunks = interleaved.chunks(hop);
        // Always emit at least one content frame (an empty input
        // yields one silent frame + the flush frame).
        let first = chunks.next().unwrap_or(&[]);
        out.extend_from_slice(&self.encode_frame(first)?);
        for chunk in chunks {
            out.extend_from_slice(&self.encode_frame(chunk)?);
        }
        out.extend_from_slice(&self.finish()?);
        Ok(out)
    }

    /// Window `[history | cur]`, transform, quantize under the rate
    /// loop, and wrap the raw data block in an ADTS header.
    fn encode_hop(&mut self, cur: &[Vec<f64>]) -> Result<Vec<u8>> {
        let ch = self.config.channels as usize;
        // Per-channel forward MDCT.
        let mut spectra: Vec<Vec<f64>> = Vec::with_capacity(ch);
        for (hist, chan) in self.history.iter().zip(cur.iter()) {
            let mut z = vec![0.0f64; LONG_TRANSFORM_LEN];
            for m in 0..FRAME_LEN {
                z[m] = hist[m] * self.window[m];
                z[FRAME_LEN + m] = chan[m] * self.window[FRAME_LEN + m];
            }
            spectra.push(forward_mdct(&z, LONG_TRANSFORM_LEN));
        }

        // Rate loop: uniform scalefactor offset in ±4 steps (3 dB
        // per step on the §4.6.2.3.3 quarter-step ladder). Coarsen
        // until the raw data block fits the budget; when it already
        // fits, refine (spend the remaining budget on precision) as
        // long as the finer frame still fits, down to
        // `-MAX_REFINE_OFFSET`.
        let budget = self.config.frame_budget_bytes();
        let mut sf_offset = 0i32;
        let mut raw_block = self.assemble_raw_block(&spectra, sf_offset)?;
        let mut iterations = 0usize;
        if raw_block.len() > budget {
            while raw_block.len() > budget && iterations < MAX_RATE_ITERATIONS {
                sf_offset += 4;
                raw_block = self.assemble_raw_block(&spectra, sf_offset)?;
                iterations += 1;
            }
        } else {
            while sf_offset > -MAX_REFINE_OFFSET && iterations < MAX_RATE_ITERATIONS {
                let finer = self.assemble_raw_block(&spectra, sf_offset - 4)?;
                if finer.len() > budget {
                    break;
                }
                sf_offset -= 4;
                raw_block = finer;
                iterations += 1;
            }
        }
        // Fine pass: the ±4 ladder can leave up to 3 scalefactors of
        // precision unspent at the budget boundary (a full −4 step
        // un-zeros a whole swath of near-threshold coefficients at
        // once). Try the intermediate offsets, finest first.
        if raw_block.len() <= budget && sf_offset > -MAX_REFINE_OFFSET {
            for fine in [3i32, 2, 1] {
                let cand = self.assemble_raw_block(&spectra, sf_offset - fine)?;
                if cand.len() <= budget {
                    raw_block = cand;
                    break;
                }
            }
        }

        // ADTS wrap. aac_frame_length is 13 bits; the budget floor
        // (16 bytes) and MAX_RATE_ITERATIONS guarantee headroom for
        // every realistic configuration, but validate regardless.
        let frame_len = ADTS_HEADER_BYTES_NO_CRC + raw_block.len();
        if frame_len >= (1 << 13) {
            return Err(Error::EncoderFrameOverflow);
        }
        let header = AdtsHeader {
            mpeg_version_mpeg2: false,
            protection_absent: true,
            profile: 1, // AAC LC: profile_ObjectType = AOT − 1 = 1
            sampling_frequency_index: self.fs_index,
            channel_configuration: self.config.channels,
            aac_frame_length: frame_len as u16,
            adts_buffer_fullness: 0x7FF, // VBR sentinel
            number_of_raw_data_blocks_in_frame: 1,
        };
        let mut out = Vec::with_capacity(frame_len);
        out.extend_from_slice(&header.write()?);
        out.extend_from_slice(&raw_block);
        Ok(out)
    }

    /// Assemble one `raw_data_block()` (SCE for mono, one
    /// `common_window` CPE for stereo, then END) at the given
    /// rate-loop scalefactor offset.
    fn assemble_raw_block(&self, spectra: &[Vec<f64>], sf_offset: i32) -> Result<Vec<u8>> {
        let mut channels = Vec::with_capacity(spectra.len());
        for spec in spectra {
            channels.push(quantize_channel(spec, self.fs_index, sf_offset)?);
        }

        let mut asm = FrameAssembler::new();
        let mut body_bits = BitWriter::new();
        match channels.as_slice() {
            [mono] => {
                asm.push_channel_header(IdSynEle::Sce, 0)?;
                mono.body.write(&mut body_bits, 2, self.fs_index, false)?;
                mono.spectral.write(
                    &mut body_bits,
                    &mono.info,
                    &mono.body.section_data,
                    self.fs_index,
                )?;
            }
            [left, right] => {
                asm.push_channel_header(IdSynEle::Cpe, 0)?;
                // §4.4.2.3: common_window = 1, one shared ics_info,
                // ms_mask_present = 0 (channels coded independently).
                body_bits.write_bit(true);
                left.info.write(&mut body_bits, 2, self.fs_index, true)?;
                body_bits.write_u32(0, 2);
                for chan in [left, right] {
                    chan.body
                        .write_with_ics_info(&mut body_bits, &chan.info, 2, false)?;
                    chan.spectral.write(
                        &mut body_bits,
                        &chan.info,
                        &chan.body.section_data,
                        self.fs_index,
                    )?;
                }
            }
            _ => return Err(Error::EncoderInvalidConfig),
        }
        let nbits = body_bits.bit_position();
        let body = body_bits.finish();
        asm.push_channel_body_bits(&body, nbits)?;
        Ok(asm.push_end())
    }
}

/// One quantized channel, ready for wire assembly.
struct QuantizedChannel {
    info: IcsInfo,
    body: IcsBody,
    spectral: SpectralData,
}

/// §4.6.2 forward quantizer for one coefficient at scalefactor `sf`:
/// `q = sign(x) · NINT((|x| · 2^(−0.25·(sf−100)))^(3/4))`, the exact
/// inverse of the normative `|q|^(4/3) · 2^(0.25·(sf−100))`
/// (round-half-away-from-zero per §1.3 `NINT`).
fn quantize_coef(x: f64, sf: i32) -> i32 {
    let gain = (0.25 * f64::from(sf - SF_OFFSET)).exp2();
    let mag = (x.abs() / gain).powf(0.75).round();
    let mag = mag.min(f64::from(MAX_QUANT)) as i32;
    if x < 0.0 {
        -mag
    } else {
        mag
    }
}

/// The masking-spread scalefactor for a band whose peak coefficient
/// is `peak` in a frame whose loudest band peaks at `frame_peak`:
/// solve `(peak / 2^(0.25·(sf−100)))^(3/4) = M_b` for `sf` with
/// `M_b = TARGET_PEAK_MAG · (peak/frame_peak)^SPREAD`, i.e.
/// `sf = 100 + 4·log2(peak) − (16/3)·log2(M_b)`.
///
/// Returns `None` when the band's spread target falls below
/// [`MIN_TARGET_MAG`] — such a band quantizes to silence anyway and
/// is culled to `ZERO_HCB` by the caller.
fn band_scalefactor(peak: f64, frame_peak: f64, sf_offset: i32) -> Option<i32> {
    if peak <= 0.0 || frame_peak <= 0.0 {
        return None;
    }
    let target = TARGET_PEAK_MAG * (peak / frame_peak).powf(SPREAD);
    if target < MIN_TARGET_MAG {
        return None;
    }
    let sf = f64::from(SF_OFFSET) + 4.0 * peak.log2() - (16.0 / 3.0) * target.log2();
    Some((sf.round() as i32 + sf_offset).clamp(0, 255))
}

/// Smallest Table 4.95 spectrum codebook whose LAV covers `qmax`.
/// `1`/`3` are the 4-tuple books (LAV 1 / 2), `5`/`7`/`9` the pair
/// books (LAV 4 / 7 / 12), `11` the ESC book.
fn codebook_for(qmax: i32) -> u8 {
    match qmax {
        0 => ZERO_HCB,
        1 => 1,
        2 => 3,
        3..=4 => 5,
        5..=7 => 7,
        8..=12 => 9,
        _ => 11,
    }
}

/// Quantize one channel's 1024-line spectrum into a complete
/// `individual_channel_stream()` record set.
fn quantize_channel(spec: &[f64], fs_index: u8, sf_offset: i32) -> Result<QuantizedChannel> {
    debug_assert_eq!(spec.len(), FRAME_LEN);
    let offsets = long_window_offsets(fs_index)?;
    let num_swb = NUM_SWB_LONG_WINDOW[fs_index as usize] as usize;

    // Pass 1: per-band scalefactor from the constant-SNR rule, with
    // the DPCM ±60 clamp applied against the previous *coded* band.
    // Pass 2 below re-quantizes with the clamped sf and derives the
    // codebook; a band whose coefficients all quantize to zero
    // becomes ZERO_HCB and drops out of the scalefactor track.
    let mut x_quant = vec![0i32; FRAME_LEN];
    let mut sfb_cb = vec![ZERO_HCB; num_swb];
    let mut sfs: Vec<Option<i32>> = vec![None; num_swb];
    let mut prev_sf: Option<i32> = None;
    let frame_peak = spec.iter().fold(0.0f64, |m, &v| m.max(v.abs()));
    for sfb in 0..num_swb {
        let start = offsets[sfb] as usize;
        let end = offsets[sfb + 1] as usize;
        let peak = spec[start..end].iter().fold(0.0f64, |m, &v| m.max(v.abs()));
        let Some(mut sf) = band_scalefactor(peak, frame_peak, sf_offset) else {
            continue; // culled: below the frame's masking floor
        };
        if let Some(p) = prev_sf {
            sf = sf.clamp(p - MAX_SF_DELTA, p + MAX_SF_DELTA).clamp(0, 255);
        }
        // Raise sf until the band's peak fits the ESC ceiling (a
        // +4 step scales magnitudes by 2^(-3/4)).
        let mut qmax = quantize_coef(peak, sf).abs();
        while qmax >= MAX_QUANT && sf < 255 {
            sf = (sf + 4).min(255);
            qmax = quantize_coef(peak, sf).abs();
        }
        if qmax == 0 {
            continue; // all-zero band → ZERO_HCB, no scalefactor
        }
        let mut band_max = 0i32;
        for k in start..end {
            let q = quantize_coef(spec[k], sf);
            x_quant[k] = q;
            band_max = band_max.max(q.abs());
        }
        if band_max == 0 {
            continue;
        }
        sfb_cb[sfb] = codebook_for(band_max);
        sfs[sfb] = Some(sf);
        prev_sf = Some(sf);
    }

    // Scalefactor track: global_gain seeds the DPCM at the first
    // coded band; every subsequent coded band transmits its delta.
    let mut entries: Vec<ScaleFactorEntry> = Vec::new();
    let mut global_gain: Option<i32> = None;
    let mut last = 0i32;
    for sf in sfs.iter().copied().flatten() {
        match global_gain {
            None => {
                global_gain = Some(sf);
                entries.push(ScaleFactorEntry::Dpcm(0));
            }
            Some(_) => {
                let delta = sf - last;
                debug_assert!((-MAX_SF_DELTA..=MAX_SF_DELTA).contains(&delta));
                entries.push(ScaleFactorEntry::Dpcm(delta as i8));
            }
        }
        last = sf;
    }
    let global_gain = global_gain.unwrap_or(SF_OFFSET) as u8;

    // Sections: merge adjacent equal codebooks.
    let mut sections: Vec<Section> = Vec::new();
    for (sfb, &cb) in sfb_cb.iter().enumerate() {
        match sections.last_mut() {
            Some(s) if s.codebook == cb => s.end = (sfb + 1) as u8,
            _ => sections.push(Section {
                codebook: cb,
                start: sfb as u8,
                end: (sfb + 1) as u8,
            }),
        }
    }

    let info = IcsInfo {
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: num_swb as u8,
        scale_factor_grouping: None,
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: num_swb as u8,
    };
    let section_data = SectionData {
        sections: vec![sections],
        sfb_cb: vec![sfb_cb],
    };
    let scale_factor_data = ScaleFactorData {
        entries: vec![entries],
    };
    let body = IcsBody {
        global_gain,
        ics_info: Some(info.clone()),
        section_data,
        scale_factor_data,
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    let spectral = SpectralData {
        x_quant: vec![x_quant],
    };
    Ok(QuantizedChannel {
        info,
        body,
        spectral,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dequant::{inverse_quantize, scale_factor_gain};

    #[test]
    fn quantize_coef_inverts_dequant_within_rounding() {
        // For any q and sf, dequantizing then re-quantizing recovers
        // q exactly (the quantizer is the exact inverse map).
        for sf in [40i32, 100, 156, 200] {
            for q in [-8190i32, -1000, -12, -1, 0, 1, 7, 40, 999, 8190] {
                let x = inverse_quantize(q) * scale_factor_gain(sf as u8);
                assert_eq!(quantize_coef(x, sf), q, "sf={sf} q={q}");
            }
        }
    }

    #[test]
    fn band_scalefactor_hits_target_magnitude() {
        // A frame-loudest peak quantized with its own
        // band_scalefactor lands within rounding of TARGET_PEAK_MAG.
        for peak in [1.0f64, 100.0, 3.2e4, 6.7e7] {
            let sf = band_scalefactor(peak, peak, 0).expect("loudest band never culls");
            let q = quantize_coef(peak, sf).abs();
            let lo = (TARGET_PEAK_MAG / 2.0_f64.powf(0.375)).floor() as i32;
            let hi = (TARGET_PEAK_MAG * 2.0_f64.powf(0.375)).ceil() as i32;
            assert!(
                (lo..=hi).contains(&q),
                "peak={peak} sf={sf} q={q} not in [{lo},{hi}]"
            );
        }
    }

    #[test]
    fn band_scalefactor_spreads_and_culls() {
        let frame_peak = 1.0e6f64;
        // A band 40 dB down gets a ~20 dB smaller target: the target
        // is 42·(10^-2)^0.5 = 4.2, so its peak quantizes to ~4.
        let sf = band_scalefactor(frame_peak * 1e-2, frame_peak, 0).unwrap();
        let q = quantize_coef(frame_peak * 1e-2, sf).abs();
        assert!((2..=8).contains(&q), "spread target off: q={q}");
        // A band ~90 dB down is culled outright (target < 0.7).
        assert_eq!(band_scalefactor(frame_peak * 3.2e-5, frame_peak, 0), None);
        // Zero-peak bands cull.
        assert_eq!(band_scalefactor(0.0, frame_peak, 0), None);
    }

    #[test]
    fn codebook_selection_covers_table_4_95_lavs() {
        assert_eq!(codebook_for(0), ZERO_HCB);
        assert_eq!(codebook_for(1), 1);
        assert_eq!(codebook_for(2), 3);
        assert_eq!(codebook_for(4), 5);
        assert_eq!(codebook_for(7), 7);
        assert_eq!(codebook_for(12), 9);
        assert_eq!(codebook_for(13), 11);
        assert_eq!(codebook_for(8191), 11);
    }

    #[test]
    fn config_rejects_bad_parameters() {
        assert!(StreamEncoder::new(EncoderConfig {
            sample_rate: 44_100,
            channels: 1,
            bitrate: 64_000,
        })
        .is_ok());
        assert!(matches!(
            StreamEncoder::new(EncoderConfig {
                sample_rate: 44_056, // not a Table 1.18 rate
                channels: 1,
                bitrate: 64_000,
            }),
            Err(Error::EncoderInvalidConfig)
        ));
        assert!(matches!(
            StreamEncoder::new(EncoderConfig {
                sample_rate: 44_100,
                channels: 3,
                bitrate: 64_000,
            }),
            Err(Error::EncoderInvalidConfig)
        ));
        assert!(matches!(
            StreamEncoder::new(EncoderConfig {
                sample_rate: 44_100,
                channels: 1,
                bitrate: 0,
            }),
            Err(Error::EncoderInvalidConfig)
        ));
    }

    #[test]
    fn silent_input_yields_valid_minimal_frames() {
        let mut enc = StreamEncoder::new(EncoderConfig {
            sample_rate: 44_100,
            channels: 1,
            bitrate: 64_000,
        })
        .unwrap();
        let stream = enc.encode_all(&[0i16; FRAME_LEN]).unwrap();
        // Two frames (content + flush), each parseable.
        let (h0, off) = AdtsHeader::parse(&stream).unwrap();
        assert_eq!(h0.channel_configuration, 1);
        assert_eq!(off, ADTS_HEADER_BYTES_NO_CRC);
        let second = &stream[h0.aac_frame_length as usize..];
        let (h1, _) = AdtsHeader::parse(second).unwrap();
        assert_eq!(
            h0.aac_frame_length as usize + h1.aac_frame_length as usize,
            stream.len()
        );
    }
}

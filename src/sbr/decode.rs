//! End-to-end SBR decoding for a single channel.
//!
//! Wires the bitstream parser, frequency-band tables, QMF analysis /
//! synthesis banks, HF generator, and HF adjuster together to turn
//!
//! 1) 1024 low-band PCM samples from the AAC-LC core, and
//! 2) a parsed SBR payload
//!
//! into 2048 output PCM samples at twice the sample rate.

use super::bitstream::{
    parse_single_channel_element, parse_sbr_header, SbrChannelData, SbrHeader, EXT_SBR_DATA,
    EXT_SBR_DATA_CRC,
};
use super::freq::FreqTables;
use super::hf_adjust::{apply_envelope, envelope_time_borders};
use super::hf_gen::{apply_hf_generation, build_patches, update_bw, BwArray, PatchInfo};
use super::qmf::{QmfAnalysis, QmfSynthesis, ANALYSIS_BANDS};
use super::{Complex32, NUM_QMF_BANDS, NUM_TIME_SLOTS_1024, RATE};

use oxideav_core::{bits::BitReader, Error, Result};

/// Per-channel running state for the SBR decoder.
pub struct SbrChannelState {
    pub header: SbrHeader,
    pub freq: Option<FreqTables>,
    pub patches: Option<PatchInfo>,
    pub qmf_analysis: QmfAnalysis,
    pub qmf_synthesis: QmfSynthesis,
    /// Previous frame's XLow tail — covers the HFAdj/HFGen lookahead.
    pub x_low_tail: Vec<[Complex32; ANALYSIS_BANDS]>,
    pub bw_array: BwArray,
    pub prev_invf_modes: [u8; 5],
    pub header_seen: bool,
    /// Frame counter — tracks whether this is the first SBR frame.
    pub frame_count: u64,
}

impl SbrChannelState {
    pub fn new() -> Self {
        Self {
            header: SbrHeader::defaults(),
            freq: None,
            patches: None,
            qmf_analysis: QmfAnalysis::new(),
            qmf_synthesis: QmfSynthesis::new(),
            x_low_tail: vec![[Complex32::default(); ANALYSIS_BANDS]; 8],
            bw_array: [0.0; 5],
            prev_invf_modes: [0; 5],
            header_seen: false,
            frame_count: 0,
        }
    }
}

impl Default for SbrChannelState {
    fn default() -> Self {
        Self::new()
    }
}

/// Attempt to recognise an SBR extension payload in the given FIL buffer.
///
/// `fil_bytes` is the raw payload of a `fill_element` (NOT including the
/// 4-bit count header — that is, the bits starting with the
/// extension_type nibble). Returns `Ok(Some(..))` with the parsed data if
/// it really was an SBR payload, `Ok(None)` if the extension_type was
/// something else, or `Err` on parse failure.
pub fn try_parse_sbr_extension(
    br: &mut BitReader<'_>,
    num_payload_bits: u32,
    is_sce: bool,
    state: &mut SbrChannelState,
    fs_core: u32,
) -> Result<Option<SbrChannelData>> {
    if num_payload_bits < 4 {
        return Ok(None);
    }
    let start_pos = br.bit_position();
    let ext_type = br.read_u32(4)?;
    if ext_type != EXT_SBR_DATA && ext_type != EXT_SBR_DATA_CRC {
        // Rewind: caller expected we'd consume everything, so rewind is
        // not possible with this BitReader — return None and let caller
        // skip the remaining bits. We've consumed 4 bits here.
        let remaining = num_payload_bits - 4;
        for _ in 0..remaining {
            let _ = br.read_u32(1)?;
        }
        return Ok(None);
    }
    let crc_flag = ext_type == EXT_SBR_DATA_CRC;
    if crc_flag {
        let _crc = br.read_u32(10)?;
    }
    let bs_header_flag = br.read_bit()?;
    if bs_header_flag {
        parse_sbr_header(br, &mut state.header)?;
        state.header_seen = true;
        // (Re)build freq tables on header change.
        let fs_sbr = fs_core * 2;
        state.freq = Some(FreqTables::build(
            fs_sbr,
            state.header.bs_start_freq,
            state.header.bs_stop_freq,
            state.header.bs_xover_band,
            state.header.bs_freq_scale,
            state.header.bs_alter_scale,
            state.header.bs_noise_bands,
        )?);
        if let Some(ft) = &state.freq {
            state.patches = Some(build_patches(ft, fs_sbr)?);
        }
    }
    if !state.header_seen {
        // Skip remaining bits — we can't decode without a header.
        let consumed = (br.bit_position() - start_pos) as u32;
        let remaining = num_payload_bits.saturating_sub(consumed);
        for _ in 0..remaining {
            let _ = br.read_u32(1)?;
        }
        return Ok(None);
    }
    let ft = state.freq.as_ref().ok_or_else(|| {
        Error::invalid("SBR: missing freq tables when decoding SBR data")
    })?;
    let (num_env_bands_lo, num_env_bands_hi) = (ft.n_low, ft.n_high);
    let num_noise_bands = ft.nq;
    let num_high_res = ft.n_high;
    let mut data = SbrChannelData {
        bs_amp_res: state.header.bs_amp_res,
        ..SbrChannelData::default()
    };
    if is_sce {
        parse_single_channel_element(
            br,
            &mut data,
            num_noise_bands,
            [num_env_bands_lo, num_env_bands_hi],
            num_high_res,
        )?;
    } else {
        // CPE / stereo HE-AACv1 not supported yet — mark and return early.
        return Err(Error::unsupported(
            "SBR: channel_pair_element in SBR payload not implemented",
        ));
    }
    // Bit-align: skip any remaining bits inside the payload.
    let consumed = (br.bit_position() - start_pos) as u32;
    let remaining = num_payload_bits.saturating_sub(consumed);
    for _ in 0..remaining {
        let _ = br.read_u32(1)?;
    }
    Ok(Some(data))
}

/// Run the full SBR decode on one frame of 1024 mono PCM samples.
///
/// `pcm_in` is the low-band PCM from the AAC-LC core, length 1024.
/// `sbr_data` is the parsed per-channel SBR data for this frame.
/// `output` receives 2048 PCM samples at 2× sample rate.
///
/// The output is normalised to roughly the same amplitude range as the
/// input (QMF gain is compensated inside the synthesis bank).
pub fn decode_sbr_frame(
    pcm_in: &[f32],
    sbr_data: &SbrChannelData,
    state: &mut SbrChannelState,
    output: &mut [f32],
) -> Result<()> {
    let ft = state.freq.as_ref().ok_or_else(|| {
        Error::invalid("SBR: decode_sbr_frame called before header parse")
    })?;
    let patches = state.patches.as_ref().ok_or_else(|| {
        Error::invalid("SBR: decode_sbr_frame called before patch construction")
    })?;
    if pcm_in.len() < 1024 || output.len() < 2048 {
        return Err(Error::invalid(
            "SBR: decode_sbr_frame requires 1024 input / 2048 output PCM",
        ));
    }
    let num_time_slots = NUM_TIME_SLOTS_1024 as i32;
    let num_slots = NUM_TIME_SLOTS_1024 * RATE; // 32 QMF subsamples per frame

    // 1) Analysis QMF — 32 time samples produce one column of 32 subbands.
    //    Build x_low: [subsample][subband]. We need tail from previous
    //    frame for HF generator lookahead (needs l-2 and l-1).
    let mut x_low: Vec<[Complex32; ANALYSIS_BANDS]> = vec![
        [Complex32::default(); ANALYSIS_BANDS];
        state.x_low_tail.len() + num_slots
    ];
    for (i, row) in state.x_low_tail.iter().enumerate() {
        x_low[i] = *row;
    }
    let tail_len = state.x_low_tail.len();
    let mut tmp_in = [0.0f32; 32];
    for l in 0..num_slots {
        tmp_in.copy_from_slice(&pcm_in[l * 32..l * 32 + 32]);
        let mut col = [Complex32::default(); ANALYSIS_BANDS];
        state.qmf_analysis.process(&tmp_in, &mut col);
        x_low[tail_len + l] = col;
    }

    // 2) Update bwArray from invf_mode — use previous frame's modes.
    let cur_modes: [u8; 5] = {
        let mut m = [0u8; 5];
        for i in 0..ft.nq.min(5) {
            m[i] = sbr_data.bs_invf_mode[i];
        }
        m
    };
    let bw = update_bw(&state.bw_array, &state.prev_invf_modes, &cur_modes, ft.nq);

    // 3) HF generator — produce XHigh from XLow. Alpha coefficients
    //    default to zero (simplified — covariance-method HF LPC not
    //    implemented). Copy-up alone already reconstructs the spectrum
    //    shape.
    let alpha0 = [Complex32::default(); 32];
    let alpha1 = [Complex32::default(); 32];
    let mut x_high: Vec<[Complex32; NUM_QMF_BANDS]> = vec![
        [Complex32::default(); NUM_QMF_BANDS];
        x_low.len()
    ];
    apply_hf_generation(
        &x_low,
        &mut x_high,
        patches,
        ft,
        &bw,
        &alpha0,
        &alpha1,
        super::T_HF_ADJ,
        0,
        num_time_slots as usize,
    );

    // 4) HF adjuster — apply envelope gains.
    let t_e = envelope_time_borders(sbr_data, num_time_slots);
    apply_envelope(
        &mut x_high,
        sbr_data,
        ft,
        &t_e,
        super::T_HF_ADJ,
    );

    // 5) Synthesis QMF — 64 complex subbands × one subsample → 64 PCM
    //    samples. Total output is num_slots * 64 = 2048.
    let mut out64 = [0.0f32; 64];
    for l in 0..num_slots {
        // Use the slot aligned past the tail — output the newly-decoded
        // range.
        let src = &x_high[tail_len + l];
        state.qmf_synthesis.process(src, &mut out64);
        output[l * 64..l * 64 + 64].copy_from_slice(&out64);
    }

    // 6) Carry forward for next frame.
    for (i, row) in x_low.iter().rev().take(tail_len).rev().enumerate() {
        state.x_low_tail[i] = *row;
    }
    state.bw_array = bw;
    state.prev_invf_modes = cur_modes;
    state.frame_count = state.frame_count.wrapping_add(1);
    Ok(())
}

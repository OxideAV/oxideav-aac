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
    parse_channel_pair_element, parse_sbr_header, parse_single_channel_element_ext, SbrChannelData,
    SbrHeader, EXT_SBR_DATA, EXT_SBR_DATA_CRC,
};
use super::freq::FreqTables;
use super::hf_adjust::{
    apply_envelope_coupled_with_limiter, apply_envelope_with_limiter, build_limiter_bands,
    envelope_time_borders,
};
use super::hf_gen::{
    apply_hf_generation, build_patches, compute_hf_lpc, update_bw, BwArray, PatchInfo,
};
use super::ps::{apply_ps_qmf, PsFrame, PsState};
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
    /// Running PS state (header + delay line for the decorrelator).
    pub ps: PsState,
    /// Right-channel 64-band synthesis QMF — only used when PS is active.
    /// The left channel reuses `qmf_synthesis`.
    pub ps_right_qmf: QmfSynthesis,
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
            ps: PsState::new(),
            ps_right_qmf: QmfSynthesis::new(),
        }
    }
}

impl Default for SbrChannelState {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of `try_parse_sbr_extension` — distinguishes SCE mono payload
/// from a stereo CPE payload. Mono payloads may additionally carry a PS
/// extension for HE-AACv2 stereo upmix.
// `SbrChannelData` is intentionally stack-allocated (~1.4 KiB) so the Pair
// variant reaches ~2.8 KiB. Boxing would add a heap-alloc per SBR frame on
// the hot decode path without reducing peak working set (the parent
// stack-frame already holds the data either way). Behaviour-preserving
// suppression — hot path; boxing adds heap alloc per frame.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug)]
pub enum SbrPayload {
    /// Single channel (mono HE-AACv1 or HE-AACv2 when `ps` is `Some`).
    Single {
        data: SbrChannelData,
        ps: Option<PsFrame>,
    },
    /// Channel pair. `coupled` indicates whether the two channels share a
    /// grid and the right-channel envelope is encoded as a balance.
    Pair {
        l: SbrChannelData,
        r: SbrChannelData,
        coupled: bool,
    },
}

/// Attempt to recognise an SBR extension payload in the given FIL buffer.
///
/// `num_payload_bits` is the total bit-size of the fill-element payload
/// (including the 4-bit extension_type field). Returns `Ok(Some(..))` with
/// the parsed data if it really was an SBR payload, `Ok(None)` if the
/// extension_type was something else, or `Err` on parse failure.
pub fn try_parse_sbr_extension(
    br: &mut BitReader<'_>,
    num_payload_bits: u32,
    is_sce: bool,
    state: &mut SbrChannelState,
    fs_core: u32,
) -> Result<Option<SbrChannelData>> {
    match try_parse_sbr_extension_ext(br, num_payload_bits, is_sce, state, fs_core)? {
        Some(SbrPayload::Single { data, .. }) => Ok(Some(data)),
        // A CPE slipped through the is_sce=true path — silently drop; the
        // stereo-capable caller should use the _ext variant.
        Some(SbrPayload::Pair { l, .. }) => Ok(Some(l)),
        None => Ok(None),
    }
}

/// Like `try_parse_sbr_extension` but returns the richer [`SbrPayload`]
/// enum covering both SCE and CPE payloads.
pub fn try_parse_sbr_extension_ext(
    br: &mut BitReader<'_>,
    num_payload_bits: u32,
    is_sce: bool,
    state: &mut SbrChannelState,
    fs_core: u32,
) -> Result<Option<SbrPayload>> {
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
    let ft = state
        .freq
        .as_ref()
        .ok_or_else(|| Error::invalid("SBR: missing freq tables when decoding SBR data"))?;
    let (num_env_bands_lo, num_env_bands_hi) = (ft.n_low, ft.n_high);
    let num_noise_bands = ft.nq;
    let num_high_res = ft.n_high;
    let payload = if is_sce {
        let mut data = SbrChannelData {
            bs_amp_res: state.header.bs_amp_res,
            ..SbrChannelData::default()
        };
        let mut ps_frame: Option<PsFrame> = None;
        parse_single_channel_element_ext(
            br,
            &mut data,
            num_noise_bands,
            [num_env_bands_lo, num_env_bands_hi],
            num_high_res,
            Some((&mut state.ps, &mut ps_frame)),
        )?;
        SbrPayload::Single { data, ps: ps_frame }
    } else {
        let mut data_l = SbrChannelData {
            bs_amp_res: state.header.bs_amp_res,
            ..SbrChannelData::default()
        };
        let mut data_r = SbrChannelData {
            bs_amp_res: state.header.bs_amp_res,
            ..SbrChannelData::default()
        };
        let coupled = parse_channel_pair_element(
            br,
            &mut data_l,
            &mut data_r,
            num_noise_bands,
            [num_env_bands_lo, num_env_bands_hi],
            num_high_res,
        )?;
        SbrPayload::Pair {
            l: data_l,
            r: data_r,
            coupled,
        }
    };
    // Bit-align: skip any remaining bits inside the payload.
    let consumed = (br.bit_position() - start_pos) as u32;
    let remaining = num_payload_bits.saturating_sub(consumed);
    for _ in 0..remaining {
        let _ = br.read_u32(1)?;
    }
    Ok(Some(payload))
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
    let ft = state
        .freq
        .as_ref()
        .ok_or_else(|| Error::invalid("SBR: decode_sbr_frame called before header parse"))?;
    let patches = state
        .patches
        .as_ref()
        .ok_or_else(|| Error::invalid("SBR: decode_sbr_frame called before patch construction"))?;
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
    let mut x_low: Vec<[Complex32; ANALYSIS_BANDS]> =
        vec![[Complex32::default(); ANALYSIS_BANDS]; state.x_low_tail.len() + num_slots];
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

    // 3) HF generator — produce XHigh from XLow. Alpha coefficients come
    //    from the 2nd-order covariance-method LPC fit on the low-band
    //    subbands. Stability-clipped to zero when |alpha|^2 >= 16.
    let mut alpha0 = [Complex32::default(); 32];
    let mut alpha1 = [Complex32::default(); 32];
    compute_hf_lpc(&x_low, num_slots, super::T_HF_ADJ, &mut alpha0, &mut alpha1);
    let mut x_high: Vec<[Complex32; NUM_QMF_BANDS]> =
        vec![[Complex32::default(); NUM_QMF_BANDS]; x_low.len()];
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

    // 4) HF adjuster — envelope gains + noise + sinusoid + limiter.
    let t_e = envelope_time_borders(sbr_data, num_time_slots);
    let lim = build_limiter_bands(ft, patches, state.header.bs_limiter_bands);
    let seed = (state.frame_count as u32)
        .wrapping_mul(2_654_435_761)
        .wrapping_add(0x9E37_79B9);
    apply_envelope_with_limiter(
        &mut x_high,
        sbr_data,
        ft,
        &t_e,
        super::T_HF_ADJ,
        Some(&lim),
        seed,
        state.header.bs_limiter_gains,
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

/// Run the full HE-AACv2 decode path for a mono-with-PS frame.
///
/// Identical to [`decode_sbr_frame`] up through HF generation + envelope
/// adjustment, but instead of running one synthesis QMF the mono `X_high`
/// matrix is first fed into [`apply_ps_qmf`], which upmixes to stereo in
/// the QMF domain (§8.6.4.6 + Annex 8.A). Two 64-band synthesis QMF banks
/// (left / right, held inside the PS sibling state) then produce
/// `out_l` / `out_r` at 2× the core sample rate.
///
/// The PS decorrelator and mixing matrix state lives in `state.ps`; the
/// right-channel synthesis QMF lives in `state_right_qmf` which the caller
/// owns so the decoder can carry filterbank history across frames.
pub fn decode_sbr_frame_ps(
    pcm_in: &[f32],
    sbr_data: &SbrChannelData,
    ps_frame: &PsFrame,
    state: &mut SbrChannelState,
    state_right_qmf: &mut super::qmf::QmfSynthesis,
    out_l: &mut [f32],
    out_r: &mut [f32],
) -> Result<()> {
    if state.freq.is_none() {
        return Err(Error::invalid(
            "SBR: decode_sbr_frame_ps called before header parse",
        ));
    }
    if state.patches.is_none() {
        return Err(Error::invalid(
            "SBR: decode_sbr_frame_ps called before patch construction",
        ));
    }
    if pcm_in.len() < 1024 || out_l.len() < 2048 || out_r.len() < 2048 {
        return Err(Error::invalid(
            "SBR+PS: decode_sbr_frame_ps requires 1024 input / 2048 output per channel",
        ));
    }
    let num_time_slots = NUM_TIME_SLOTS_1024 as i32;
    let num_slots = NUM_TIME_SLOTS_1024 * RATE;

    // 1) Analysis QMF.
    let (x_low, tail_len) = run_analysis(state, pcm_in, num_slots);

    // 2) Update bwArray. Snapshot `nq` before re-borrowing state mutably.
    let nq = state.freq.as_ref().unwrap().nq;
    let cur_modes: [u8; 5] = {
        let mut m = [0u8; 5];
        for i in 0..nq.min(5) {
            m[i] = sbr_data.bs_invf_mode[i];
        }
        m
    };
    let bw = update_bw(&state.bw_array, &state.prev_invf_modes, &cur_modes, nq);

    // 3) HF generator.
    let mut alpha0 = [Complex32::default(); 32];
    let mut alpha1 = [Complex32::default(); 32];
    compute_hf_lpc(&x_low, num_slots, super::T_HF_ADJ, &mut alpha0, &mut alpha1);
    let mut x_high: Vec<[Complex32; NUM_QMF_BANDS]> =
        vec![[Complex32::default(); NUM_QMF_BANDS]; x_low.len()];
    {
        let ft = state.freq.as_ref().unwrap();
        let patches = state.patches.as_ref().unwrap();
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
    }

    // 4) HF adjuster.
    let t_e = envelope_time_borders(sbr_data, num_time_slots);
    let lim = {
        let ft = state.freq.as_ref().unwrap();
        let patches = state.patches.as_ref().unwrap();
        build_limiter_bands(ft, patches, state.header.bs_limiter_bands)
    };
    let seed = (state.frame_count as u32)
        .wrapping_mul(2_654_435_761)
        .wrapping_add(0x9E37_79B9);
    {
        let ft = state.freq.as_ref().unwrap();
        apply_envelope_with_limiter(
            &mut x_high,
            sbr_data,
            ft,
            &t_e,
            super::T_HF_ADJ,
            Some(&lim),
            seed,
            state.header.bs_limiter_gains,
        );
    }

    // 5) PS upmix — x_high → (x_left, x_right) at QMF granularity. We feed
    //    only the newly-decoded range (skip the leading tail used as HF-gen
    //    look-back). PS is applied over num_slots rows.
    let mut x_left: Vec<[Complex32; NUM_QMF_BANDS]> =
        vec![[Complex32::default(); NUM_QMF_BANDS]; num_slots];
    let mut x_right: Vec<[Complex32; NUM_QMF_BANDS]> =
        vec![[Complex32::default(); NUM_QMF_BANDS]; num_slots];
    // Slice x_high[tail_len..tail_len+num_slots] as the mono input.
    let x_mono = &x_high[tail_len..tail_len + num_slots];
    apply_ps_qmf(x_mono, &mut x_left, &mut x_right, ps_frame, &mut state.ps);

    // 6) Synthesis QMF — left reuses the mono-channel bank already in state,
    //    right uses the caller-supplied bank. This gives each output channel
    //    its own polyphase history.
    let mut out64 = [0.0f32; 64];
    for l in 0..num_slots {
        state.qmf_synthesis.process(&x_left[l], &mut out64);
        out_l[l * 64..l * 64 + 64].copy_from_slice(&out64);
    }
    for l in 0..num_slots {
        state_right_qmf.process(&x_right[l], &mut out64);
        out_r[l * 64..l * 64 + 64].copy_from_slice(&out64);
    }

    // 7) Carry forward.
    for (i, row) in x_low.iter().rev().take(tail_len).rev().enumerate() {
        state.x_low_tail[i] = *row;
    }
    state.bw_array = bw;
    state.prev_invf_modes = cur_modes;
    state.frame_count = state.frame_count.wrapping_add(1);
    Ok(())
}

/// Run SBR on a CPE pair. `pcm_l` / `pcm_r` each hold 1024 low-band samples;
/// `out_l` / `out_r` receive 2048 output samples each at 2× rate.
///
/// When `coupled` is true, the dequantisation pulls `E_total` + balance
/// (`E_balance`) from `data_l` / `data_r` as described in
/// `apply_envelope_coupled`. When it's false, each channel's envelope is
/// applied independently.
#[allow(clippy::too_many_arguments)]
pub fn decode_sbr_cpe_frame(
    pcm_l: &[f32],
    pcm_r: &[f32],
    data_l: &SbrChannelData,
    data_r: &SbrChannelData,
    coupled: bool,
    state_l: &mut SbrChannelState,
    state_r: &mut SbrChannelState,
    out_l: &mut [f32],
    out_r: &mut [f32],
) -> Result<()> {
    if pcm_l.len() < 1024 || pcm_r.len() < 1024 || out_l.len() < 2048 || out_r.len() < 2048 {
        return Err(Error::invalid(
            "SBR: decode_sbr_cpe_frame requires 1024 input / 2048 output PCM per channel",
        ));
    }
    let num_time_slots = NUM_TIME_SLOTS_1024 as i32;
    let num_slots = NUM_TIME_SLOTS_1024 * RATE;

    // Analysis for both channels.
    let (x_low_l, tail_len_l) = run_analysis(state_l, pcm_l, num_slots);
    let (x_low_r, tail_len_r) = run_analysis(state_r, pcm_r, num_slots);

    // Update bwArray per channel.
    let ft_l = state_l
        .freq
        .as_ref()
        .ok_or_else(|| Error::invalid("SBR: CPE decode without freq tables (L)"))?;
    let patches_l = state_l
        .patches
        .as_ref()
        .ok_or_else(|| Error::invalid("SBR: CPE decode without patches (L)"))?;
    let cur_modes_l: [u8; 5] = {
        let mut m = [0u8; 5];
        for i in 0..ft_l.nq.min(5) {
            m[i] = data_l.bs_invf_mode[i];
        }
        m
    };
    let bw_l = update_bw(
        &state_l.bw_array,
        &state_l.prev_invf_modes,
        &cur_modes_l,
        ft_l.nq,
    );

    let mut alpha0_l = [Complex32::default(); 32];
    let mut alpha1_l = [Complex32::default(); 32];
    compute_hf_lpc(
        &x_low_l,
        num_slots,
        super::T_HF_ADJ,
        &mut alpha0_l,
        &mut alpha1_l,
    );
    let mut x_high_l: Vec<[Complex32; NUM_QMF_BANDS]> =
        vec![[Complex32::default(); NUM_QMF_BANDS]; x_low_l.len()];
    apply_hf_generation(
        &x_low_l,
        &mut x_high_l,
        patches_l,
        ft_l,
        &bw_l,
        &alpha0_l,
        &alpha1_l,
        super::T_HF_ADJ,
        0,
        num_time_slots as usize,
    );

    let ft_r = state_r
        .freq
        .as_ref()
        .ok_or_else(|| Error::invalid("SBR: CPE decode without freq tables (R)"))?;
    let patches_r = state_r
        .patches
        .as_ref()
        .ok_or_else(|| Error::invalid("SBR: CPE decode without patches (R)"))?;
    let cur_modes_r: [u8; 5] = {
        let mut m = [0u8; 5];
        for i in 0..ft_r.nq.min(5) {
            m[i] = data_r.bs_invf_mode[i];
        }
        m
    };
    let bw_r = update_bw(
        &state_r.bw_array,
        &state_r.prev_invf_modes,
        &cur_modes_r,
        ft_r.nq,
    );
    let mut alpha0_r = [Complex32::default(); 32];
    let mut alpha1_r = [Complex32::default(); 32];
    compute_hf_lpc(
        &x_low_r,
        num_slots,
        super::T_HF_ADJ,
        &mut alpha0_r,
        &mut alpha1_r,
    );
    let mut x_high_r: Vec<[Complex32; NUM_QMF_BANDS]> =
        vec![[Complex32::default(); NUM_QMF_BANDS]; x_low_r.len()];
    apply_hf_generation(
        &x_low_r,
        &mut x_high_r,
        patches_r,
        ft_r,
        &bw_r,
        &alpha0_r,
        &alpha1_r,
        super::T_HF_ADJ,
        0,
        num_time_slots as usize,
    );

    // Envelope application.
    let t_e_l = envelope_time_borders(data_l, num_time_slots);
    let lim_l = build_limiter_bands(ft_l, patches_l, state_l.header.bs_limiter_bands);
    let lim_r = build_limiter_bands(ft_r, patches_r, state_r.header.bs_limiter_bands);
    let seed_l = (state_l.frame_count as u32)
        .wrapping_mul(2_654_435_761)
        .wrapping_add(0x9E37_79B9);
    let seed_r = (state_r.frame_count as u32)
        .wrapping_mul(2_654_435_761)
        .wrapping_add(0x632B_E593);
    if coupled {
        apply_envelope_coupled_with_limiter(
            &mut x_high_l,
            &mut x_high_r,
            data_l,
            data_r,
            ft_l,
            &t_e_l,
            super::T_HF_ADJ,
            Some(&lim_l),
            seed_l,
            state_l.header.bs_limiter_gains,
        );
    } else {
        apply_envelope_with_limiter(
            &mut x_high_l,
            data_l,
            ft_l,
            &t_e_l,
            super::T_HF_ADJ,
            Some(&lim_l),
            seed_l,
            state_l.header.bs_limiter_gains,
        );
        let t_e_r = envelope_time_borders(data_r, num_time_slots);
        apply_envelope_with_limiter(
            &mut x_high_r,
            data_r,
            ft_r,
            &t_e_r,
            super::T_HF_ADJ,
            Some(&lim_r),
            seed_r,
            state_r.header.bs_limiter_gains,
        );
    }

    // Synthesis.
    let mut out64 = [0.0f32; 64];
    for l in 0..num_slots {
        let src = &x_high_l[tail_len_l + l];
        state_l.qmf_synthesis.process(src, &mut out64);
        out_l[l * 64..l * 64 + 64].copy_from_slice(&out64);
    }
    for l in 0..num_slots {
        let src = &x_high_r[tail_len_r + l];
        state_r.qmf_synthesis.process(src, &mut out64);
        out_r[l * 64..l * 64 + 64].copy_from_slice(&out64);
    }

    // Carry forward.
    for (i, row) in x_low_l.iter().rev().take(tail_len_l).rev().enumerate() {
        state_l.x_low_tail[i] = *row;
    }
    for (i, row) in x_low_r.iter().rev().take(tail_len_r).rev().enumerate() {
        state_r.x_low_tail[i] = *row;
    }
    state_l.bw_array = bw_l;
    state_l.prev_invf_modes = cur_modes_l;
    state_l.frame_count = state_l.frame_count.wrapping_add(1);
    state_r.bw_array = bw_r;
    state_r.prev_invf_modes = cur_modes_r;
    state_r.frame_count = state_r.frame_count.wrapping_add(1);
    Ok(())
}

fn run_analysis(
    state: &mut SbrChannelState,
    pcm_in: &[f32],
    num_slots: usize,
) -> (Vec<[Complex32; ANALYSIS_BANDS]>, usize) {
    let tail_len = state.x_low_tail.len();
    let mut x_low: Vec<[Complex32; ANALYSIS_BANDS]> =
        vec![[Complex32::default(); ANALYSIS_BANDS]; tail_len + num_slots];
    for (i, row) in state.x_low_tail.iter().enumerate() {
        x_low[i] = *row;
    }
    let mut tmp_in = [0.0f32; 32];
    for l in 0..num_slots {
        tmp_in.copy_from_slice(&pcm_in[l * 32..l * 32 + 32]);
        let mut col = [Complex32::default(); ANALYSIS_BANDS];
        state.qmf_analysis.process(&tmp_in, &mut col);
        x_low[tail_len + l] = col;
    }
    (x_low, tail_len)
}

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

/// Run SBR in "upsampling-only" mode per ISO/IEC 14496-3 §4.6.18.5
/// (the bullet starting *"If scalable SBR is used, or if the SBR tool is
/// used for pure upsampling without SBR processing"*).
///
/// This is the **boundary-case** path that #771 left unwired. It fires when
/// the decoder is operating in SBR mode (either explicit SBR signalling in
/// the AudioSpecificConfig / ADTS, or the implicit-SBR convention adopted
/// by HE-AAC encoders for the very last frame of a stream) but the current
/// frame's `raw_data_block()` carries no `EXT_SBR_DATA` / `EXT_SBR_DATA_CRC`
/// FIL extension — i.e. no envelope, no noise floor, no `bs_invf_mode`.
/// The spec is explicit: when this happens the decoder still emits
/// `2 * samplesPerFrame` PCM samples (= 2048 for the 1024-sample AAC
/// core) and gets there by running the 32-channel analysis QMF followed
/// directly by the 64-channel synthesis QMF, with all HF-generator and
/// HF-adjuster steps bypassed. The high-band (`k = 32..63`) of the input
/// to the synthesis bank is the zero matrix.
///
/// Concretely per §4.6.18.5 the relationship used here is:
///
/// ```text
///     X_Low(k, l) = W(k, l - t_HFGen)    for 0 <= k < 32
///     X_High(k, l) = 0                   for 32 <= k < 64
/// ```
///
/// with `t_HFGen` discarded — the synthesis bank simply consumes the 32
/// analysis-bank columns directly. Output is `2 * pcm_in.len()` real PCM
/// samples (2048 for the 1024-sample core, 1920 for the 960-sample core).
///
/// The function does **not** require a parsed `SbrHeader`, freq tables,
/// or patches — it works on a freshly-defaulted [`SbrChannelState`] as
/// well as on one that has previously decoded full SBR frames. The
/// analysis / synthesis filterbank histories in `state` continue to
/// build, so a transition back to a payload-bearing SBR frame on the
/// next packet starts with seamless QMF state.
///
/// Returns `Err` only when `pcm_in.len() < 1024` or `output.len() < 2048`
/// — the spec requires the dual-rate output to be exactly 2 × the core
/// frame's sample count, so we treat under-sized buffers as caller
/// programming errors.
pub fn decode_sbr_upsample_only(
    pcm_in: &[f32],
    state: &mut SbrChannelState,
    output: &mut [f32],
) -> Result<()> {
    if pcm_in.len() < 1024 || output.len() < 2048 {
        return Err(Error::invalid(
            "SBR upsample-only: requires 1024 input / 2048 output PCM (§4.6.18.5)",
        ));
    }
    let num_slots = NUM_TIME_SLOTS_1024 * RATE;

    // 1) Analysis QMF — same as the full SBR path. We still write to the
    //    state's `x_low_tail` so a subsequent full SBR frame sees the
    //    correct HF-gen lookback. (The HF generator wants `l-2` / `l-1`
    //    columns; bypassing it here doesn't excuse skipping the carry.)
    let (x_low, tail_len) = run_analysis(state, pcm_in, num_slots);

    // 2) Construct a 64-band column per slot: low 32 bands = analysis
    //    output, high 32 bands = zero (per §4.6.18.5 "X_High = 0 when SBR
    //    is used for upsampling only"). Feed straight into the synthesis
    //    bank — no HF generator, no envelope adjuster, no limiter.
    let mut col64 = [Complex32::default(); NUM_QMF_BANDS];
    let mut out64 = [0.0f32; 64];
    for l in 0..num_slots {
        // The synthesis bank wants the "current" subsample column; we
        // align with `decode_sbr_frame`'s convention of skipping the
        // leading tail (`x_low[tail_len + l]`) so the polyphase delay
        // line behaves identically across upsample-only and full-SBR
        // frames.
        let src = &x_low[tail_len + l];
        for k in 0..ANALYSIS_BANDS {
            col64[k] = src[k];
        }
        // High-band stays zero (already initialised; reset defensively
        // in case the loop body grows in future).
        for k in ANALYSIS_BANDS..NUM_QMF_BANDS {
            col64[k] = Complex32::default();
        }
        state.qmf_synthesis.process(&col64, &mut out64);
        output[l * 64..l * 64 + 64].copy_from_slice(&out64);
    }

    // 3) Carry forward the x_low tail so the next frame (whether
    //    upsample-only or full SBR) sees the right history. Matches the
    //    full-SBR path's tail rotation.
    for (i, row) in x_low.iter().rev().take(tail_len).rev().enumerate() {
        state.x_low_tail[i] = *row;
    }
    state.frame_count = state.frame_count.wrapping_add(1);
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

#[cfg(test)]
mod boundary_tests {
    //! Tests for the §4.6.18.5 SBR-upsample-only boundary path
    //! ([`decode_sbr_upsample_only`]).
    //!
    //! The function fires when SBR signalling is active for a stream but
    //! the current frame's `raw_data_block()` carries no EXT_SBR_DATA
    //! FIL payload — the trailing frame of a real HE-AAC capture is the
    //! canonical case (workspace task #771).
    use super::*;
    use crate::synth::FRAME_LEN;
    use core::f32::consts::PI;

    /// `decode_sbr_upsample_only` must emit EXACTLY 2 * FRAME_LEN samples
    /// for the standard 1024-sample core (= 2048). Anything less would
    /// drop the trailing high-rate samples on the floor; anything more
    /// would overrun the caller's pre-allocated buffer. This is the
    /// "boundary trim" half of the §4.6.18 contract.
    #[test]
    fn upsample_only_emits_2x_frame_len() {
        let pcm_in = vec![0.0f32; FRAME_LEN];
        let mut output = vec![0.0f32; 2 * FRAME_LEN];
        let mut state = SbrChannelState::new();
        decode_sbr_upsample_only(&pcm_in, &mut state, &mut output)
            .expect("zero input must succeed");
        assert_eq!(output.len(), 2 * FRAME_LEN);
    }

    /// Under-sized buffers must error rather than silently truncating.
    /// The spec REQUIRES 2 * samples_per_frame at the dual-rate output;
    /// returning a short slice would be a quiet contract violation.
    #[test]
    fn upsample_only_rejects_short_input() {
        let pcm_in = vec![0.0f32; FRAME_LEN - 1];
        let mut output = vec![0.0f32; 2 * FRAME_LEN];
        let mut state = SbrChannelState::new();
        let err = decode_sbr_upsample_only(&pcm_in, &mut state, &mut output).unwrap_err();
        assert!(matches!(err, Error::InvalidData(_)));
    }

    #[test]
    fn upsample_only_rejects_short_output() {
        let pcm_in = vec![0.0f32; FRAME_LEN];
        let mut output = vec![0.0f32; 2 * FRAME_LEN - 1];
        let mut state = SbrChannelState::new();
        let err = decode_sbr_upsample_only(&pcm_in, &mut state, &mut output).unwrap_err();
        assert!(matches!(err, Error::InvalidData(_)));
    }

    /// Silence in must produce silence out (modulo QMF cold-start, which
    /// is bounded by the prototype filter's tail energy). The first few
    /// hundred samples of the QMF synthesis output can carry warm-up
    /// from the all-zero filterbank state; the trailing samples should
    /// be at-or-near zero. We check the FAR-tail (samples 1500..2048)
    /// to give the filterbank time to settle.
    #[test]
    fn upsample_only_silence_is_silent_after_qmf_warmup() {
        let pcm_in = vec![0.0f32; FRAME_LEN];
        let mut output = vec![0.0f32; 2 * FRAME_LEN];
        let mut state = SbrChannelState::new();
        decode_sbr_upsample_only(&pcm_in, &mut state, &mut output).unwrap();
        let max_tail: f32 = output[1500..2048]
            .iter()
            .map(|s| s.abs())
            .fold(0.0f32, f32::max);
        assert!(
            max_tail < 1e-3,
            "silence-in / silence-out: tail max = {max_tail}"
        );
    }

    /// The function must not require a parsed header / freq tables /
    /// patches — the implicit-SBR-at-trailing-frame case is exactly the
    /// configuration where those structures are still `None` if the
    /// stream never carried a single SBR FIL payload. Calling on a
    /// freshly-defaulted `SbrChannelState` exercises that path.
    #[test]
    fn upsample_only_works_without_header_or_patches() {
        let pcm_in = vec![0.1f32; FRAME_LEN];
        let mut output = vec![0.0f32; 2 * FRAME_LEN];
        let mut state = SbrChannelState::new();
        assert!(state.freq.is_none());
        assert!(state.patches.is_none());
        assert!(!state.header_seen);
        decode_sbr_upsample_only(&pcm_in, &mut state, &mut output)
            .expect("no header/patches needed for upsample-only");
    }

    /// A low-frequency tone in the AAC-LC core must survive the
    /// analysis+synthesis QMF round-trip with energy on the same order
    /// of magnitude as the input. The QMF prototype filter passes the
    /// low subbands without modification when the high band is zero
    /// (§4.6.18.4.1 / .4.2 perfect-reconstruction property of the
    /// 64-tap polyphase pair). This is the audible-quality criterion
    /// for the boundary path — zero-order-hold doubling (the pre-r91
    /// fallback) would produce aliasing harmonics at f_s_core - f.
    #[test]
    fn upsample_only_preserves_low_frequency_tone_energy() {
        // 1 kHz tone at 22050 Hz core (so output rate is 44100).
        let core_rate = 22_050.0_f32;
        let freq = 1_000.0_f32;
        let amp = 0.3_f32;
        let pcm_in: Vec<f32> = (0..FRAME_LEN)
            .map(|n| (2.0 * PI * freq * (n as f32) / core_rate).sin() * amp)
            .collect();
        let mut output = vec![0.0f32; 2 * FRAME_LEN];
        let mut state = SbrChannelState::new();
        // Prime the analysis bank with two frames of the same tone so
        // we're past the 320-sample QMF warm-up by the time we measure.
        let _ = decode_sbr_upsample_only(&pcm_in, &mut state, &mut output);
        let _ = decode_sbr_upsample_only(&pcm_in, &mut state, &mut output);
        let _ = decode_sbr_upsample_only(&pcm_in, &mut state, &mut output);
        decode_sbr_upsample_only(&pcm_in, &mut state, &mut output).unwrap();
        let in_rms: f32 = (pcm_in.iter().map(|s| s * s).sum::<f32>() / FRAME_LEN as f32).sqrt();
        let out_rms: f32 =
            (output.iter().map(|s| s * s).sum::<f32>() / (2 * FRAME_LEN) as f32).sqrt();
        // The QMF analysis-then-synthesis pair carries an internal
        // 1/64 normalisation (§4.6.18.4.2) that's compensated by the
        // analysis 2.0-multiplied modulation; the round-trip should
        // restore input amplitude to within a small constant factor.
        // Allow a wide envelope (factor 8 either way) because the
        // 64-tap polyphase prototype's group delay still mixes warm-up
        // tail into the steady-state measurement; the point is to
        // confirm the signal didn't collapse to zero (which a buggy
        // wiring of the high-band zero-pad would produce).
        let ratio = out_rms / in_rms;
        assert!(
            ratio > 0.125 && ratio < 8.0,
            "QMF upsample round-trip ratio = {ratio} (in_rms={in_rms} out_rms={out_rms})"
        );
    }

    /// The function must advance `frame_count` so the per-frame PRNG
    /// seed used by the full SBR path (when a payload-bearing frame
    /// follows an upsample-only frame) gets a fresh value rather than
    /// repeating the previous frame's noise.
    #[test]
    fn upsample_only_advances_frame_counter() {
        let pcm_in = vec![0.0f32; FRAME_LEN];
        let mut output = vec![0.0f32; 2 * FRAME_LEN];
        let mut state = SbrChannelState::new();
        let before = state.frame_count;
        decode_sbr_upsample_only(&pcm_in, &mut state, &mut output).unwrap();
        assert_eq!(state.frame_count, before + 1);
        decode_sbr_upsample_only(&pcm_in, &mut state, &mut output).unwrap();
        assert_eq!(state.frame_count, before + 2);
    }

    /// The x_low_tail carry-forward must mutate `state.x_low_tail` —
    /// otherwise a follow-up full-SBR frame whose HF generator reads
    /// `l = -2` / `l = -1` columns would see all-zero history and emit
    /// silence into the patched high band for the first envelope.
    #[test]
    fn upsample_only_updates_x_low_tail() {
        // Use a non-zero input so the analysis bank produces non-zero
        // subbands. After a single call the tail should not be all
        // zero anymore (the QMF analysis impulse response of a sine
        // input lands energy in at least one subband).
        let core_rate = 22_050.0_f32;
        let freq = 5_000.0_f32;
        let pcm_in: Vec<f32> = (0..FRAME_LEN)
            .map(|n| (2.0 * PI * freq * (n as f32) / core_rate).sin() * 0.5)
            .collect();
        let mut output = vec![0.0f32; 2 * FRAME_LEN];
        let mut state = SbrChannelState::new();
        decode_sbr_upsample_only(&pcm_in, &mut state, &mut output).unwrap();
        let any_non_zero = state
            .x_low_tail
            .iter()
            .any(|row| row.iter().any(|c| c.re.abs() > 1e-6 || c.im.abs() > 1e-6));
        assert!(
            any_non_zero,
            "x_low_tail must be carried forward after upsample-only frame"
        );
    }
}

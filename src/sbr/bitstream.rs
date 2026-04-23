//! SBR bitstream parsing — ISO/IEC 14496-3 Tables 4.62–4.74.

use oxideav_core::{bits::BitReader, Error, Result};

use super::tables::{
    sbr_huff_decode, F_HUFFMAN_ENV_1_5DB, F_HUFFMAN_ENV_3_0DB, F_HUFFMAN_ENV_BAL_1_5DB,
    F_HUFFMAN_ENV_BAL_3_0DB, LAV_ENV_1_5DB, LAV_ENV_3_0DB, LAV_ENV_BAL_1_5DB, LAV_ENV_BAL_3_0DB,
    LAV_NOISE_3_0DB, LAV_NOISE_BAL_3_0DB, T_HUFFMAN_ENV_1_5DB, T_HUFFMAN_ENV_3_0DB,
    T_HUFFMAN_ENV_BAL_1_5DB, T_HUFFMAN_ENV_BAL_3_0DB, T_HUFFMAN_NOISE_3_0DB,
    T_HUFFMAN_NOISE_BAL_3_0DB,
};

pub const MAX_SBR_ENVELOPES: usize = 5;
pub const MAX_NOISE_FLOORS: usize = 2;
pub const MAX_ENV_BANDS: usize = 64;
pub const MAX_NOISE_BANDS: usize = 5;

/// Frame class — Table 4.69.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FrameClass {
    #[default]
    FixFix = 0,
    FixVar = 1,
    VarFix = 2,
    VarVar = 3,
}

impl FrameClass {
    pub fn from_u32(v: u32) -> Self {
        match v {
            1 => Self::FixVar,
            2 => Self::VarFix,
            3 => Self::VarVar,
            _ => Self::FixFix,
        }
    }
}

/// Extension types inside the SBR payload, Table 4.121.
pub const EXT_SBR_DATA: u32 = 0xD;
pub const EXT_SBR_DATA_CRC: u32 = 0xE;

/// Parsed SBR header (§4.6.18.3 Table 4.63).
#[derive(Clone, Debug, Default)]
pub struct SbrHeader {
    pub bs_amp_res: u8,
    pub bs_start_freq: u8,
    pub bs_stop_freq: u8,
    pub bs_xover_band: u8,
    pub bs_header_extra_1: bool,
    pub bs_header_extra_2: bool,
    pub bs_freq_scale: u8,
    pub bs_alter_scale: bool,
    pub bs_noise_bands: u8,
    pub bs_limiter_bands: u8,
    pub bs_limiter_gains: u8,
    pub bs_interpol_freq: bool,
    pub bs_smoothing_mode: bool,
}

impl SbrHeader {
    pub fn defaults() -> Self {
        // Defaults per Note 3 of Table 4.63.
        Self {
            bs_amp_res: 1,
            bs_start_freq: 0,
            bs_stop_freq: 0,
            bs_xover_band: 0,
            bs_header_extra_1: false,
            bs_header_extra_2: false,
            bs_freq_scale: 2,
            bs_alter_scale: true,
            bs_noise_bands: 2,
            bs_limiter_bands: 2,
            bs_limiter_gains: 2,
            bs_interpol_freq: true,
            bs_smoothing_mode: true,
        }
    }
}

/// Parse an sbr_header() bitstream element (Table 4.63).
///
/// Mutates `hdr` in place — the caller keeps a running header between
/// frames because headers can be omitted on frames where nothing changes.
pub fn parse_sbr_header(br: &mut BitReader<'_>, hdr: &mut SbrHeader) -> Result<()> {
    hdr.bs_amp_res = br.read_u32(1)? as u8;
    hdr.bs_start_freq = br.read_u32(4)? as u8;
    hdr.bs_stop_freq = br.read_u32(4)? as u8;
    hdr.bs_xover_band = br.read_u32(3)? as u8;
    let _bs_reserved = br.read_u32(2)?;
    hdr.bs_header_extra_1 = br.read_bit()?;
    hdr.bs_header_extra_2 = br.read_bit()?;

    if hdr.bs_header_extra_1 {
        hdr.bs_freq_scale = br.read_u32(2)? as u8;
        hdr.bs_alter_scale = br.read_bit()?;
        hdr.bs_noise_bands = br.read_u32(2)? as u8;
    } else {
        hdr.bs_freq_scale = 2;
        hdr.bs_alter_scale = true;
        hdr.bs_noise_bands = 2;
    }

    if hdr.bs_header_extra_2 {
        hdr.bs_limiter_bands = br.read_u32(2)? as u8;
        hdr.bs_limiter_gains = br.read_u32(2)? as u8;
        hdr.bs_interpol_freq = br.read_bit()?;
        hdr.bs_smoothing_mode = br.read_bit()?;
    } else {
        hdr.bs_limiter_bands = 2;
        hdr.bs_limiter_gains = 2;
        hdr.bs_interpol_freq = true;
        hdr.bs_smoothing_mode = true;
    }

    Ok(())
}

/// Per-channel SBR bitstream state parsed from one sbr_single_channel_element
/// (Table 4.65).
#[derive(Clone, Debug)]
pub struct SbrChannelData {
    pub frame_class: FrameClass,
    pub bs_num_env: u8,
    pub bs_num_noise: u8,
    pub bs_amp_res: u8, // effective amp_res for this frame (see grid FIXFIX carve-out)
    /// Freq-res flag per envelope: 0 = LO (fTableLow), 1 = HI (fTableHigh).
    pub freq_res: [u8; MAX_SBR_ENVELOPES],
    pub bs_var_bord_0: u8,
    pub bs_var_bord_1: u8,
    pub bs_num_rel_0: u8,
    pub bs_num_rel_1: u8,
    pub bs_rel_bord_0: [u8; 3],
    pub bs_rel_bord_1: [u8; 3],
    pub bs_pointer: u8,
    /// Delta-coding flag per envelope (0 = freq, 1 = time).
    pub bs_df_env: [u8; MAX_SBR_ENVELOPES],
    /// Delta-coding flag per noise floor (0 = freq, 1 = time).
    pub bs_df_noise: [u8; MAX_NOISE_FLOORS],
    /// Inverse-filtering mode per noise-band (Table 4.71).
    pub bs_invf_mode: [u8; MAX_NOISE_BANDS],
    pub bs_add_harmonic_flag: bool,
    /// Per-high-res band sine-addition flags.
    pub bs_add_harmonic: [u8; MAX_ENV_BANDS],
    /// Delta-decoded envelope scalefactors per (env, band).
    pub env_sf: [[i32; MAX_ENV_BANDS]; MAX_SBR_ENVELOPES],
    /// Delta-decoded noise scalefactors per (noise_floor, band).
    pub noise_sf: [[i32; MAX_NOISE_BANDS]; MAX_NOISE_FLOORS],
    /// When `true` this channel carries the *balance* side of a coupled CPE
    /// pair (Table 4.68 coupling-mode envelope / noise values, §4.6.18.3.5).
    /// Envelope and noise dequantisation is different for balance data.
    pub bs_coupling_balance: bool,
}

impl Default for SbrChannelData {
    fn default() -> Self {
        Self {
            frame_class: FrameClass::default(),
            bs_num_env: 0,
            bs_num_noise: 0,
            bs_amp_res: 0,
            freq_res: [0; MAX_SBR_ENVELOPES],
            bs_var_bord_0: 0,
            bs_var_bord_1: 0,
            bs_num_rel_0: 0,
            bs_num_rel_1: 0,
            bs_rel_bord_0: [0; 3],
            bs_rel_bord_1: [0; 3],
            bs_pointer: 0,
            bs_df_env: [0; MAX_SBR_ENVELOPES],
            bs_df_noise: [0; MAX_NOISE_FLOORS],
            bs_invf_mode: [0; MAX_NOISE_BANDS],
            bs_add_harmonic_flag: false,
            bs_add_harmonic: [0; MAX_ENV_BANDS],
            env_sf: [[0; MAX_ENV_BANDS]; MAX_SBR_ENVELOPES],
            noise_sf: [[0; MAX_NOISE_BANDS]; MAX_NOISE_FLOORS],
            bs_coupling_balance: false,
        }
    }
}

/// Parse sbr_grid() for one channel (Table 4.69).
pub fn parse_sbr_grid(br: &mut BitReader<'_>, data: &mut SbrChannelData) -> Result<()> {
    data.frame_class = FrameClass::from_u32(br.read_u32(2)?);
    match data.frame_class {
        FrameClass::FixFix => {
            let tmp = br.read_u32(2)?;
            data.bs_num_env = 1 << tmp; // 1, 2, 4, or 8
            if data.bs_num_env as usize > MAX_SBR_ENVELOPES {
                // Spec caps at 4 for FIXFIX, 5 for VARVAR.
                return Err(Error::invalid(
                    "SBR: bs_num_env exceeds FIXFIX maximum",
                ));
            }
            // bs_amp_res forced to 0 when bs_num_env == 1 in FIXFIX.
            if data.bs_num_env == 1 {
                data.bs_amp_res = 0;
            }
            let f0 = br.read_u32(1)? as u8;
            for env in 0..data.bs_num_env as usize {
                data.freq_res[env] = f0;
            }
        }
        FrameClass::FixVar => {
            data.bs_var_bord_1 = br.read_u32(2)? as u8;
            data.bs_num_rel_1 = br.read_u32(2)? as u8;
            data.bs_num_env = data.bs_num_rel_1 + 1;
            for rel in 0..data.bs_num_rel_1 as usize {
                data.bs_rel_bord_1[rel] = 2 * (br.read_u32(2)? as u8) + 2;
            }
            let ptr_bits = ceil_log2(data.bs_num_env as u32 + 1);
            data.bs_pointer = br.read_u32(ptr_bits)? as u8;
            for env in 0..data.bs_num_env as usize {
                // Indexed reverse per spec: bs_freq_res[bs_num_env-1-env].
                let idx = data.bs_num_env as usize - 1 - env;
                data.freq_res[idx] = br.read_u32(1)? as u8;
            }
        }
        FrameClass::VarFix => {
            data.bs_var_bord_0 = br.read_u32(2)? as u8;
            data.bs_num_rel_0 = br.read_u32(2)? as u8;
            data.bs_num_env = data.bs_num_rel_0 + 1;
            for rel in 0..data.bs_num_rel_0 as usize {
                data.bs_rel_bord_0[rel] = 2 * (br.read_u32(2)? as u8) + 2;
            }
            let ptr_bits = ceil_log2(data.bs_num_env as u32 + 1);
            data.bs_pointer = br.read_u32(ptr_bits)? as u8;
            for env in 0..data.bs_num_env as usize {
                data.freq_res[env] = br.read_u32(1)? as u8;
            }
        }
        FrameClass::VarVar => {
            data.bs_var_bord_0 = br.read_u32(2)? as u8;
            data.bs_var_bord_1 = br.read_u32(2)? as u8;
            data.bs_num_rel_0 = br.read_u32(2)? as u8;
            data.bs_num_rel_1 = br.read_u32(2)? as u8;
            data.bs_num_env = data.bs_num_rel_0 + data.bs_num_rel_1 + 1;
            if data.bs_num_env as usize > MAX_SBR_ENVELOPES {
                return Err(Error::invalid(
                    "SBR: bs_num_env exceeds VARVAR maximum",
                ));
            }
            for rel in 0..data.bs_num_rel_0 as usize {
                data.bs_rel_bord_0[rel] = 2 * (br.read_u32(2)? as u8) + 2;
            }
            for rel in 0..data.bs_num_rel_1 as usize {
                data.bs_rel_bord_1[rel] = 2 * (br.read_u32(2)? as u8) + 2;
            }
            let ptr_bits = ceil_log2(data.bs_num_env as u32 + 1);
            data.bs_pointer = br.read_u32(ptr_bits)? as u8;
            for env in 0..data.bs_num_env as usize {
                data.freq_res[env] = br.read_u32(1)? as u8;
            }
        }
    }

    data.bs_num_noise = if data.bs_num_env > 1 { 2 } else { 1 };
    Ok(())
}

fn ceil_log2(v: u32) -> u32 {
    let v = v.max(1);
    32 - (v - 1).leading_zeros()
}

/// Parse sbr_dtdf() — Table 4.70.
pub fn parse_sbr_dtdf(br: &mut BitReader<'_>, data: &mut SbrChannelData) -> Result<()> {
    for env in 0..data.bs_num_env as usize {
        data.bs_df_env[env] = br.read_u32(1)? as u8;
    }
    for n in 0..data.bs_num_noise as usize {
        data.bs_df_noise[n] = br.read_u32(1)? as u8;
    }
    Ok(())
}

/// Parse sbr_invf() — Table 4.71.
pub fn parse_sbr_invf(
    br: &mut BitReader<'_>,
    data: &mut SbrChannelData,
    num_noise_bands: usize,
) -> Result<()> {
    for n in 0..num_noise_bands.min(MAX_NOISE_BANDS) {
        data.bs_invf_mode[n] = br.read_u32(2)? as u8;
    }
    Ok(())
}

/// Parse sbr_envelope() for a single channel (Table 4.72).
///
/// `num_env_bands` is an array giving the number of bands for each
/// frequency resolution (LO/HI): `num_env_bands[0]` = `NLow`,
/// `num_env_bands[1]` = `NHigh`.
///
/// When `data.bs_coupling_balance` is set, the balance-mode Huffman tables
/// and "balance" start-value bit count are used instead (§4.6.18.3.5).
pub fn parse_sbr_envelope(
    br: &mut BitReader<'_>,
    data: &mut SbrChannelData,
    num_env_bands: [usize; 2],
) -> Result<()> {
    // Pick Huffman tables by amp_res and by coupling-balance flag. Balance
    // tables encode the *difference* between the two coupled channels; they
    // have smaller LAV and a smaller start-value field (5 bits vs 6/7).
    let (t_huff, f_huff, start_bits, lav) = match (data.bs_amp_res != 0, data.bs_coupling_balance) {
        (true, false) => (
            &T_HUFFMAN_ENV_3_0DB[..],
            &F_HUFFMAN_ENV_3_0DB[..],
            6u32,
            LAV_ENV_3_0DB,
        ),
        (false, false) => (
            &T_HUFFMAN_ENV_1_5DB[..],
            &F_HUFFMAN_ENV_1_5DB[..],
            7u32,
            LAV_ENV_1_5DB,
        ),
        (true, true) => (
            &T_HUFFMAN_ENV_BAL_3_0DB[..],
            &F_HUFFMAN_ENV_BAL_3_0DB[..],
            5u32,
            LAV_ENV_BAL_3_0DB,
        ),
        (false, true) => (
            &T_HUFFMAN_ENV_BAL_1_5DB[..],
            &F_HUFFMAN_ENV_BAL_1_5DB[..],
            6u32,
            LAV_ENV_BAL_1_5DB,
        ),
    };

    for env in 0..data.bs_num_env as usize {
        let n = num_env_bands[data.freq_res[env] as usize];
        if n > MAX_ENV_BANDS {
            return Err(Error::invalid("SBR: too many envelope bands"));
        }
        if data.bs_df_env[env] == 0 {
            // Freq-direction delta: first value is absolute, rest are deltas.
            data.env_sf[env][0] = br.read_u32(start_bits)? as i32;
            for band in 1..n {
                let sym = sbr_huff_decode(br, f_huff)? as i32;
                data.env_sf[env][band] = sym - lav;
            }
        } else {
            // Time-direction delta: all bands are deltas from prev frame (or
            // prev envelope inside the frame). We store raw deltas here; the
            // dequantiser accumulates.
            for band in 0..n {
                let sym = sbr_huff_decode(br, t_huff)? as i32;
                data.env_sf[env][band] = sym - lav;
            }
        }
    }
    Ok(())
}

/// Parse sbr_noise() for a single channel (Table 4.73).
///
/// Uses the balance-mode Huffman tables when `data.bs_coupling_balance` is
/// set.
pub fn parse_sbr_noise(
    br: &mut BitReader<'_>,
    data: &mut SbrChannelData,
    num_noise_bands: usize,
) -> Result<()> {
    let (t_huff, f_huff, start_bits, lav) = if data.bs_coupling_balance {
        (
            &T_HUFFMAN_NOISE_BAL_3_0DB[..],
            // Balance noise uses the BAL 3.0 dB env f-table (Note 2 of
            // Table 4.73: noise f-table == env f-table at the same res).
            &F_HUFFMAN_ENV_BAL_3_0DB[..],
            5u32,
            LAV_NOISE_BAL_3_0DB,
        )
    } else {
        (
            &T_HUFFMAN_NOISE_3_0DB[..],
            &F_HUFFMAN_ENV_3_0DB[..], // Note 2: noise f-table == env f-table at 3.0dB.
            5u32,
            LAV_NOISE_3_0DB,
        )
    };
    for n in 0..data.bs_num_noise as usize {
        if data.bs_df_noise[n] == 0 {
            data.noise_sf[n][0] = br.read_u32(start_bits)? as i32;
            for band in 1..num_noise_bands {
                let sym = sbr_huff_decode(br, f_huff)? as i32;
                data.noise_sf[n][band] = sym - lav;
            }
        } else {
            for band in 0..num_noise_bands {
                let sym = sbr_huff_decode(br, t_huff)? as i32;
                data.noise_sf[n][band] = sym - lav;
            }
        }
    }
    Ok(())
}

/// Parse sbr_sinusoidal_coding() — Table 4.74.
pub fn parse_sbr_sinusoidal(
    br: &mut BitReader<'_>,
    data: &mut SbrChannelData,
    num_high_res: usize,
) -> Result<()> {
    for n in 0..num_high_res.min(MAX_ENV_BANDS) {
        data.bs_add_harmonic[n] = br.read_u32(1)? as u8;
    }
    Ok(())
}

/// Parse sbr_single_channel_element() — Table 4.65.
///
/// Returns the number of bits consumed from `br`.
#[allow(clippy::too_many_arguments)]
pub fn parse_single_channel_element(
    br: &mut BitReader<'_>,
    data: &mut SbrChannelData,
    num_noise_bands: usize,
    num_env_bands: [usize; 2],
    num_high_res: usize,
) -> Result<()> {
    parse_single_channel_element_ext(br, data, num_noise_bands, num_env_bands, num_high_res, None)
}

/// Parse sbr_single_channel_element() and optionally capture a PS
/// extended_data payload (bs_extension_id = 2). When `ps_state` is `Some`,
/// any `ps_data()` payload found in the extension is parsed via
/// `ps::parse_ps_data` and returned via `*ps_out` if provided.
#[allow(clippy::too_many_arguments)]
pub fn parse_single_channel_element_ext(
    br: &mut BitReader<'_>,
    data: &mut SbrChannelData,
    num_noise_bands: usize,
    num_env_bands: [usize; 2],
    num_high_res: usize,
    ps_capture: Option<(&mut super::ps::PsState, &mut Option<super::ps::PsFrame>)>,
) -> Result<()> {
    let bs_data_extra = br.read_bit()?;
    if bs_data_extra {
        let _bs_reserved = br.read_u32(4)?;
    }
    parse_sbr_grid(br, data)?;
    parse_sbr_dtdf(br, data)?;
    parse_sbr_invf(br, data, num_noise_bands)?;
    parse_sbr_envelope(br, data, num_env_bands)?;
    parse_sbr_noise(br, data, num_noise_bands)?;
    data.bs_add_harmonic_flag = br.read_bit()?;
    if data.bs_add_harmonic_flag {
        parse_sbr_sinusoidal(br, data, num_high_res)?;
    }
    let bs_extended_data = br.read_bit()?;
    if bs_extended_data {
        let mut cnt = br.read_u32(4)?;
        if cnt == 15 {
            cnt += br.read_u32(8)?;
        }
        let total_ext_bits = 8 * cnt as u32;
        // Each extension element: 2-bit bs_extension_id + data bits. The
        // spec allows multiple back-to-back elements within total_ext_bits.
        let start_bits = br.bit_position() as u32;
        let mut maybe_ps = ps_capture;
        // Try to read a single extension element — the spec allows a
        // 2-bit ext_id up front followed by its data. Multiple back-to-
        // back elements may follow but most real streams carry one
        // (either SBR-internal or PS).
        if total_ext_bits >= 2 {
            let ext_id = br.read_u32(2)?;
            if ext_id == super::ps::EXT_ID_PS_DATA as u32 && maybe_ps.is_some() {
                let (state, out_slot) = maybe_ps.take().unwrap();
                let frame = super::ps::parse_ps_data(br, state)?;
                *out_slot = Some(frame);
            }
        }
        // Drain any remaining bits to match the declared total.
        let actual = (br.bit_position() as u32).saturating_sub(start_bits);
        if actual < total_ext_bits {
            let remaining = total_ext_bits - actual;
            for _ in 0..remaining {
                let _ = br.read_u32(1)?;
            }
        }
    }
    Ok(())
}

/// Parse `sbr_channel_pair_element()` — Table 4.66.
///
/// A CPE can be either
///   * **coupled**    (`bs_coupling = 1`): the two channels share time/freq
///     grid, df flags and inverse-filter modes. `data_l` carries the
///     *shared* envelope/noise values; `data_r` carries balance values
///     (with `bs_coupling_balance = true`).
///   * **independent** (`bs_coupling = 0`): each channel carries its own
///     grid + data, just like back-to-back single_channel_elements.
///
/// Returns `Ok(coupling)` so callers can apply the coupled-mode gain at
/// envelope-application time. Bit-alignment and any trailing extension data
/// are consumed by the caller that invoked this function.
#[allow(clippy::too_many_arguments)]
pub fn parse_channel_pair_element(
    br: &mut BitReader<'_>,
    data_l: &mut SbrChannelData,
    data_r: &mut SbrChannelData,
    num_noise_bands: usize,
    num_env_bands: [usize; 2],
    num_high_res: usize,
) -> Result<bool> {
    let bs_data_extra = br.read_bit()?;
    if bs_data_extra {
        let _reserved_l = br.read_u32(4)?;
        let _reserved_r = br.read_u32(4)?;
    }
    let bs_coupling = br.read_bit()?;
    if bs_coupling {
        parse_sbr_grid(br, data_l)?;
        // Right-channel inherits every grid parameter from left.
        copy_grid(data_l, data_r);
        parse_sbr_dtdf(br, data_l)?;
        parse_sbr_dtdf(br, data_r)?;
        parse_sbr_invf(br, data_l, num_noise_bands)?;
        for n in 0..num_noise_bands.min(MAX_NOISE_BANDS) {
            data_r.bs_invf_mode[n] = data_l.bs_invf_mode[n];
        }
        // Left carries the raw envelope/noise, right carries the balance.
        data_l.bs_coupling_balance = false;
        data_r.bs_coupling_balance = true;
        data_r.bs_amp_res = data_l.bs_amp_res;
        parse_sbr_envelope(br, data_l, num_env_bands)?;
        parse_sbr_noise(br, data_l, num_noise_bands)?;
        parse_sbr_envelope(br, data_r, num_env_bands)?;
        parse_sbr_noise(br, data_r, num_noise_bands)?;
        data_l.bs_add_harmonic_flag = br.read_bit()?;
        if data_l.bs_add_harmonic_flag {
            parse_sbr_sinusoidal(br, data_l, num_high_res)?;
        }
        data_r.bs_add_harmonic_flag = br.read_bit()?;
        if data_r.bs_add_harmonic_flag {
            parse_sbr_sinusoidal(br, data_r, num_high_res)?;
        }
    } else {
        parse_sbr_grid(br, data_l)?;
        parse_sbr_grid(br, data_r)?;
        parse_sbr_dtdf(br, data_l)?;
        parse_sbr_dtdf(br, data_r)?;
        parse_sbr_invf(br, data_l, num_noise_bands)?;
        parse_sbr_invf(br, data_r, num_noise_bands)?;
        data_l.bs_coupling_balance = false;
        data_r.bs_coupling_balance = false;
        parse_sbr_envelope(br, data_l, num_env_bands)?;
        parse_sbr_noise(br, data_l, num_noise_bands)?;
        parse_sbr_envelope(br, data_r, num_env_bands)?;
        parse_sbr_noise(br, data_r, num_noise_bands)?;
        data_l.bs_add_harmonic_flag = br.read_bit()?;
        if data_l.bs_add_harmonic_flag {
            parse_sbr_sinusoidal(br, data_l, num_high_res)?;
        }
        data_r.bs_add_harmonic_flag = br.read_bit()?;
        if data_r.bs_add_harmonic_flag {
            parse_sbr_sinusoidal(br, data_r, num_high_res)?;
        }
    }
    // Mirror the extended-data handling of the SCE parser at the tail — the
    // shared sbr_extension() logic lives with the caller.
    let bs_extended_data = br.read_bit()?;
    if bs_extended_data {
        let mut cnt = br.read_u32(4)?;
        if cnt == 15 {
            cnt += br.read_u32(8)?;
        }
        let num_bits = 8 * cnt as i64;
        for _ in 0..num_bits {
            br.read_u32(1)?;
        }
    }
    Ok(bs_coupling)
}

fn copy_grid(src: &SbrChannelData, dst: &mut SbrChannelData) {
    dst.frame_class = src.frame_class;
    dst.bs_num_env = src.bs_num_env;
    dst.bs_num_noise = src.bs_num_noise;
    dst.bs_amp_res = src.bs_amp_res;
    dst.freq_res = src.freq_res;
    dst.bs_var_bord_0 = src.bs_var_bord_0;
    dst.bs_var_bord_1 = src.bs_var_bord_1;
    dst.bs_num_rel_0 = src.bs_num_rel_0;
    dst.bs_num_rel_1 = src.bs_num_rel_1;
    dst.bs_rel_bord_0 = src.bs_rel_bord_0;
    dst.bs_rel_bord_1 = src.bs_rel_bord_1;
    dst.bs_pointer = src.bs_pointer;
}


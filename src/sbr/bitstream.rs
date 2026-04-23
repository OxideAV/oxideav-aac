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

/// Parse sbr_envelope() for a single channel (Table 4.72, `bs_coupling = 0`).
///
/// `num_env_bands` is an array giving the number of bands for each
/// frequency resolution (LO/HI): `num_env_bands[0]` = `NLow`,
/// `num_env_bands[1]` = `NHigh`.
pub fn parse_sbr_envelope(
    br: &mut BitReader<'_>,
    data: &mut SbrChannelData,
    num_env_bands: [usize; 2],
) -> Result<()> {
    // Pick Huffman tables by amp_res.
    let (t_huff, f_huff, start_bits) = if data.bs_amp_res != 0 {
        (
            &T_HUFFMAN_ENV_3_0DB[..],
            &F_HUFFMAN_ENV_3_0DB[..],
            6u32,
        )
    } else {
        (
            &T_HUFFMAN_ENV_1_5DB[..],
            &F_HUFFMAN_ENV_1_5DB[..],
            7u32,
        )
    };
    let lav = if data.bs_amp_res != 0 {
        LAV_ENV_3_0DB
    } else {
        LAV_ENV_1_5DB
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

/// Parse sbr_noise() for a single channel (Table 4.73, bs_coupling=0).
pub fn parse_sbr_noise(
    br: &mut BitReader<'_>,
    data: &mut SbrChannelData,
    num_noise_bands: usize,
) -> Result<()> {
    let (t_huff, f_huff) = (
        &T_HUFFMAN_NOISE_3_0DB[..],
        &F_HUFFMAN_ENV_3_0DB[..], // Note 2: noise f-table == env f-table at 3.0dB.
    );
    let lav = LAV_NOISE_3_0DB;
    for n in 0..data.bs_num_noise as usize {
        if data.bs_df_noise[n] == 0 {
            data.noise_sf[n][0] = br.read_u32(5)? as i32;
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
        let num_bits = 8 * cnt as i64;
        // Skip the extension blob — we don't handle MPS or PS here.
        for _ in 0..num_bits {
            br.read_u32(1)?;
        }
    }
    Ok(())
}

/// Unused in SCE decode path but kept for parse symmetry.
#[allow(dead_code)]
pub fn parse_channel_pair_element(
    _br: &mut BitReader<'_>,
    _data_l: &mut SbrChannelData,
    _data_r: &mut SbrChannelData,
    _num_noise_bands: usize,
    _num_env_bands: [usize; 2],
    _num_high_res: usize,
) -> Result<bool> {
    // CPE + coupling/stereo are not supported yet — surface Unsupported.
    Err(Error::unsupported(
        "SBR: channel_pair_element not implemented (mono HE-AACv1 only)",
    ))
}


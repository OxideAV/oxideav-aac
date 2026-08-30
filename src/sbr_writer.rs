//! SBR bitstream writer — ISO/IEC 14496-3 §4.4.2.8, Tables 4.62–4.74,
//! written forward.
//!
//! The exact inverse of the decode-side parsers
//! ([`crate::sbr_extension`], [`crate::sbr_header`],
//! [`crate::sbr_grid`], [`crate::sbr_envelope`],
//! [`crate::sbr_element`]): every function here serialises the same
//! side-info structs those parsers produce, in the same field order,
//! so a written payload reparses to the identical structs (the
//! round-trip tests pin that). The pieces:
//!
//! * [`write_header`] — `sbr_header()` (Table 4.63) with the two
//!   optional extra blocks emitted exactly when their flags are set.
//! * [`write_grid`] / [`write_dtdf`] / [`write_invf`] — Tables
//!   4.69–4.71. FIXFIX carries `log2(bs_num_env)` in its 2-bit
//!   field (so `bs_num_env ∈ {1, 2, 4, 8}`, and §4.B.18.3 restricts
//!   an encoder to `≤ 4`); FIXVAR transmits its frequency-resolution
//!   flags in reverse envelope order; the variable classes write
//!   `bs_pointer` with `ceil(log2(bs_num_env + 1))` bits.
//! * [`sbr_huff_enc`] — the Annex 4.A.6.1 codebook lookup (table
//!   index `value + LAV` → `(length, codeword)`), the inverse of
//!   [`crate::sbr_huffman::sbr_huff_dec`].
//! * [`write_envelope`] / [`write_noise`] — Tables 4.72 / 4.73: the
//!   absolute start value (7/6/5-bit per the Table 4.72 width rule,
//!   always 5 for noise) followed by frequency-direction deltas, or
//!   all-time-direction deltas, with the codebook pair selected by
//!   the `(coupling, ch, amp_res)` context.
//! * [`write_single_element`] / [`write_pair_element`] — Tables 4.65
//!   / 4.66 including both coupling layouts and the
//!   `sbr_sinusoidal_coding()` blocks; the extended-data block is
//!   written when the element carries one.
//! * [`build_extension_payload`] — the whole `extension_payload()`
//!   body for a `fill_element()`: the `EXT_SBR_DATA` /
//!   `EXT_SBR_DATA_CRC` type nibble, the optional 10-bit
//!   `bs_sbr_crc_bits`, `bs_header_flag`, the header, `sbr_data()`,
//!   and the `bs_fill_bits` that pad to `cnt` whole bytes. The CRC
//!   is the §4.5.2.8.1 10-bit checksum over every bit after the CRC
//!   field through the end of the padded payload (the coverage the
//!   staged `aac-crc-regions.md` documents and the decoder verifies).
//!
//! The values handed in are the *raw* wire values (`bs_data_env` /
//! `bs_data_noise` deltas, already quantised and delta-coded by the
//! §4.B.18.6 / §4.B.18.7 stages in [`crate::sbr_encoder`]); this
//! module only serialises.

use oxideav_core::bits::BitWriter;

use crate::extension_payload::ExtensionType;
use crate::raw_data_block::IdSynEle;
use crate::sbr_element::SbrElement;
use crate::sbr_envelope::{SbrEnvelopeData, SbrNoiseData};
use crate::sbr_extension::SBR_CRC_BITS;
use crate::sbr_freq_bands::HiLoTables;
use crate::sbr_grid::{FrameClass, SbrDtdf, SbrGrid, SbrInvf};
use crate::sbr_header::SbrHeader;
use crate::sbr_huffman::{env_tables, noise_tables, SbrHuffContext};
use crate::{Error, Result};

/// Write `sbr_header()` (Table 4.63). The extra blocks are emitted
/// exactly when `header_extra_1` / `header_extra_2` are set; when a
/// flag is clear the corresponding fields must already hold their
/// Table 4.63 Note 3 defaults for the decoder to see the same values
/// (the caller's responsibility — [`crate::sbr_encoder`] builds
/// headers that way).
pub fn write_header(w: &mut BitWriter, h: &SbrHeader) {
    w.write_bit(h.amp_res);
    w.write_u32(u32::from(h.start_freq), 4);
    w.write_u32(u32::from(h.stop_freq), 4);
    w.write_u32(u32::from(h.xover_band), 3);
    w.write_u32(u32::from(h.reserved), 2);
    w.write_bit(h.header_extra_1);
    w.write_bit(h.header_extra_2);
    if h.header_extra_1 {
        w.write_u32(u32::from(h.freq_scale), 2);
        w.write_bit(h.alter_scale);
        w.write_u32(u32::from(h.noise_bands), 2);
    }
    if h.header_extra_2 {
        w.write_u32(u32::from(h.limiter_bands), 2);
        w.write_u32(u32::from(h.limiter_gains), 2);
        w.write_bit(h.interpol_freq);
        w.write_bit(h.smoothing_mode);
    }
}

/// `ptr_bits = ceil(log2(num_env + 1))` (Table 4.69 Note 2).
fn ptr_bits(num_env: usize) -> u32 {
    let n = (num_env + 1) as u32;
    if n <= 1 {
        0
    } else {
        let floor_log2 = 31 - n.leading_zeros();
        if n.is_power_of_two() {
            floor_log2
        } else {
            floor_log2 + 1
        }
    }
}

/// Write `sbr_grid()` (Table 4.69).
///
/// Validates the grid against the syntax: FIXFIX needs
/// `num_env ∈ {1, 2, 4, 8}` and one shared frequency-resolution flag;
/// the variable classes need `rel_bord_*` lengths consistent with
/// `num_env` (`num_env − 1` for FIXVAR / VARFIX, `num_rel_0 +
/// num_rel_1 + 1 == num_env` for VARVAR, each raw value ≤ 3),
/// `var_bord_* ≤ 3`, `num_env ≤ 5`, and `pointer ≤ num_env`.
/// Violations surface as [`Error::SbrGridInvalid`].
pub fn write_grid(w: &mut BitWriter, g: &SbrGrid) -> Result<()> {
    if g.num_env == 0 || g.num_env > crate::sbr_grid::SBR_MAX_NUM_ENV {
        return Err(Error::SbrGridInvalid);
    }
    if g.freq_res.len() != g.num_env || g.var_bord_0 > 3 || g.var_bord_1 > 3 {
        return Err(Error::SbrGridInvalid);
    }
    if g.rel_bord_0
        .iter()
        .chain(g.rel_bord_1.iter())
        .any(|&r| r > 3)
    {
        return Err(Error::SbrGridInvalid);
    }
    if g.pointer as usize > g.num_env {
        return Err(Error::SbrGridInvalid);
    }
    w.write_u32(g.frame_class.to_bits(), 2);
    match g.frame_class {
        FrameClass::FixFix => {
            let raw = match g.num_env {
                1 => 0,
                2 => 1,
                4 => 2,
                _ => return Err(Error::SbrGridInvalid),
            };
            if g.freq_res.iter().any(|&f| f != g.freq_res[0]) {
                return Err(Error::SbrGridInvalid);
            }
            w.write_u32(raw, 2);
            w.write_bit(g.freq_res[0]);
        }
        FrameClass::FixVar => {
            if g.rel_bord_1.len() + 1 != g.num_env || g.num_env > 4 {
                return Err(Error::SbrGridInvalid);
            }
            w.write_u32(u32::from(g.var_bord_1), 2);
            w.write_u32((g.num_env - 1) as u32, 2);
            for &r in &g.rel_bord_1 {
                w.write_u32(u32::from(r), 2);
            }
            w.write_u32(g.pointer, ptr_bits(g.num_env));
            // Reverse order: bs_freq_res[num_env - 1 - env].
            for env in 0..g.num_env {
                w.write_bit(g.freq_res[g.num_env - 1 - env]);
            }
        }
        FrameClass::VarFix => {
            if g.rel_bord_0.len() + 1 != g.num_env || g.num_env > 4 {
                return Err(Error::SbrGridInvalid);
            }
            w.write_u32(u32::from(g.var_bord_0), 2);
            w.write_u32((g.num_env - 1) as u32, 2);
            for &r in &g.rel_bord_0 {
                w.write_u32(u32::from(r), 2);
            }
            w.write_u32(g.pointer, ptr_bits(g.num_env));
            for &f in &g.freq_res {
                w.write_bit(f);
            }
        }
        FrameClass::VarVar => {
            if g.rel_bord_0.len() + g.rel_bord_1.len() + 1 != g.num_env
                || g.rel_bord_0.len() > 3
                || g.rel_bord_1.len() > 3
            {
                return Err(Error::SbrGridInvalid);
            }
            w.write_u32(u32::from(g.var_bord_0), 2);
            w.write_u32(u32::from(g.var_bord_1), 2);
            w.write_u32(g.rel_bord_0.len() as u32, 2);
            w.write_u32(g.rel_bord_1.len() as u32, 2);
            for &r in &g.rel_bord_0 {
                w.write_u32(u32::from(r), 2);
            }
            for &r in &g.rel_bord_1 {
                w.write_u32(u32::from(r), 2);
            }
            w.write_u32(g.pointer, ptr_bits(g.num_env));
            for &f in &g.freq_res {
                w.write_bit(f);
            }
        }
    }
    Ok(())
}

/// Write `sbr_dtdf()` (Table 4.70).
pub fn write_dtdf(w: &mut BitWriter, d: &SbrDtdf) {
    for &f in &d.df_env {
        w.write_bit(f);
    }
    for &f in &d.df_noise {
        w.write_bit(f);
    }
}

/// Write `sbr_invf()` (Table 4.71): one 2-bit mode per noise band.
pub fn write_invf(w: &mut BitWriter, i: &SbrInvf) -> Result<()> {
    for &m in &i.invf_mode {
        if m > 3 {
            return Err(Error::SbrGridInvalid);
        }
        w.write_u32(u32::from(m), 2);
    }
    Ok(())
}

/// The inverse of [`crate::sbr_huffman::sbr_huff_dec`]: emit the
/// codeword for the signed delta `value` from `table` (whose entry
/// `value + lav` is `(length, codeword)`).
///
/// Returns [`Error::SbrHuffInvalid`] when `|value| > lav` — the
/// codebooks only span `[-LAV, LAV]`; the quantisation stage clamps
/// deltas before they reach the writer.
pub fn sbr_huff_enc(w: &mut BitWriter, table: &[(u8, u32)], lav: i32, value: i32) -> Result<()> {
    let idx = value + lav;
    if idx < 0 || idx as usize >= table.len() {
        return Err(Error::SbrHuffInvalid);
    }
    let (len, code) = table[idx as usize];
    w.write_u32(code, u32::from(len));
    Ok(())
}

/// The Table 4.72 start-value width for an envelope.
fn env_start_bits(coupling: bool, ch: bool, amp_res: bool) -> u32 {
    if coupling && ch {
        if amp_res {
            5
        } else {
            6
        }
    } else if amp_res {
        6
    } else {
        7
    }
}

/// Write `sbr_envelope()` (Table 4.72) — the inverse of
/// [`SbrEnvelopeData::parse`]. `amp_res` is the *effective* amplitude
/// resolution (after the single-envelope FIXFIX override).
///
/// A row whose length disagrees with the grid's per-envelope band
/// count, a start value that does not fit its field, or a delta
/// outside the codebook range surfaces as [`Error::SbrHuffInvalid`].
#[allow(clippy::too_many_arguments)]
pub fn write_envelope(
    w: &mut BitWriter,
    env: &SbrEnvelopeData,
    grid: &SbrGrid,
    dtdf: &SbrDtdf,
    bands: &HiLoTables,
    coupling: bool,
    ch: bool,
    amp_res: bool,
) -> Result<()> {
    let ctx = SbrHuffContext {
        coupling,
        ch,
        amp_res,
    };
    let ((t_huff, t_lav), (f_huff, f_lav)) = env_tables(ctx);
    let start_bits = env_start_bits(coupling, ch, amp_res);
    if env.data.len() != grid.num_env || dtdf.df_env.len() != grid.num_env {
        return Err(Error::SbrHuffInvalid);
    }
    for (l, row) in env.data.iter().enumerate() {
        let n = if grid.freq_res[l] {
            bands.n_high()
        } else {
            bands.n_low()
        };
        if row.len() != n {
            return Err(Error::SbrHuffInvalid);
        }
        if !dtdf.df_env[l] {
            let start = row[0];
            if start < 0 || start >= (1 << start_bits) {
                return Err(Error::SbrHuffInvalid);
            }
            w.write_u32(start as u32, start_bits);
            for &d in &row[1..] {
                sbr_huff_enc(w, f_huff, f_lav, d)?;
            }
        } else {
            for &d in row {
                sbr_huff_enc(w, t_huff, t_lav, d)?;
            }
        }
    }
    Ok(())
}

/// Write `sbr_noise()` (Table 4.73) — the inverse of
/// [`SbrNoiseData::parse`]. The start value is always a 5-bit field.
#[allow(clippy::too_many_arguments)]
pub fn write_noise(
    w: &mut BitWriter,
    noise: &SbrNoiseData,
    grid: &SbrGrid,
    dtdf: &SbrDtdf,
    num_noise_bands: usize,
    coupling: bool,
    ch: bool,
    amp_res: bool,
) -> Result<()> {
    let ctx = SbrHuffContext {
        coupling,
        ch,
        amp_res,
    };
    let ((t_huff, t_lav), (f_huff, f_lav)) = noise_tables(ctx);
    if noise.data.len() != grid.num_noise || dtdf.df_noise.len() != grid.num_noise {
        return Err(Error::SbrHuffInvalid);
    }
    for (l, row) in noise.data.iter().enumerate() {
        if row.len() != num_noise_bands {
            return Err(Error::SbrHuffInvalid);
        }
        if !dtdf.df_noise[l] {
            let start = row[0];
            if !(0..32).contains(&start) {
                return Err(Error::SbrHuffInvalid);
            }
            w.write_u32(start as u32, 5);
            for &d in &row[1..] {
                sbr_huff_enc(w, f_huff, f_lav, d)?;
            }
        } else {
            for &d in row {
                sbr_huff_enc(w, t_huff, t_lav, d)?;
            }
        }
    }
    Ok(())
}

/// `bs_add_harmonic_flag` + `sbr_sinusoidal_coding()` (Table 4.74):
/// the flag is set exactly when the channel carries any harmonic
/// flags, in which case all `NHigh` are written.
fn write_harmonics(w: &mut BitWriter, add_harmonic: &[bool], n_high: usize) -> Result<()> {
    if add_harmonic.is_empty() {
        w.write_bit(false);
        return Ok(());
    }
    if add_harmonic.len() != n_high {
        return Err(Error::SbrGridInvalid);
    }
    w.write_bit(true);
    for &f in add_harmonic {
        w.write_bit(f);
    }
    Ok(())
}

/// The `if (bs_extended_data) { … }` block shared by Tables 4.65 /
/// 4.66: `bs_extension_size` (+ escape), then the extension id and
/// body, padded with `bs_fill_bits` to the declared byte count.
fn write_extended_data(w: &mut BitWriter, ext: Option<&crate::sbr_element::SbrExtension>) {
    match ext {
        None => w.write_bit(false),
        Some(e) => {
            w.write_bit(true);
            // 2-bit id + body bits, rounded up to whole bytes.
            let body_bits = 2 + 8 * e.data.len();
            let cnt = body_bits.div_ceil(8);
            if cnt >= 15 {
                w.write_u32(15, 4);
                w.write_u32((cnt - 15) as u32, 8);
            } else {
                w.write_u32(cnt as u32, 4);
            }
            let mut written = 0usize;
            if cnt > 0 {
                w.write_u32(u32::from(e.id), 2);
                written += 2;
                for &b in &e.data {
                    w.write_u32(u32::from(b), 8);
                    written += 8;
                }
            }
            let pad = 8 * cnt - written;
            if pad > 0 {
                w.write_u32(0, pad as u32);
            }
        }
    }
}

/// Write `sbr_single_channel_element()` (Table 4.65) — the inverse of
/// [`SbrElement::parse_single`]. `amp_res` is the header
/// `bs_amp_res`; the single-envelope FIXFIX override is applied here
/// exactly as the parser does.
pub fn write_single_element(
    w: &mut BitWriter,
    el: &SbrElement,
    bands: &HiLoTables,
    amp_res: bool,
) -> Result<()> {
    let [ch] = el.channels.as_slice() else {
        return Err(Error::SbrGridInvalid);
    };
    if el.coupling {
        return Err(Error::SbrGridInvalid);
    }
    w.write_bit(false); // bs_data_extra
    write_grid(w, &ch.grid)?;
    write_dtdf(w, &ch.dtdf);
    if ch.invf.invf_mode.len() != bands.n_q() {
        return Err(Error::SbrGridInvalid);
    }
    write_invf(w, &ch.invf)?;
    let eff_amp = amp_res && !ch.grid.amp_res_override;
    write_envelope(
        w,
        &ch.envelope,
        &ch.grid,
        &ch.dtdf,
        bands,
        false,
        false,
        eff_amp,
    )?;
    write_noise(
        w,
        &ch.noise,
        &ch.grid,
        &ch.dtdf,
        bands.n_q(),
        false,
        false,
        eff_amp,
    )?;
    write_harmonics(w, &ch.add_harmonic, bands.n_high())?;
    write_extended_data(w, el.extension.as_ref());
    Ok(())
}

/// Write `sbr_channel_pair_element()` (Table 4.66) — the inverse of
/// [`SbrElement::parse_pair`], in either coupling layout. For a
/// coupled pair both channels must share the grid (the parser stores
/// the one transmitted grid in both channel slots) and the second
/// channel's `invf` is not transmitted.
pub fn write_pair_element(
    w: &mut BitWriter,
    el: &SbrElement,
    bands: &HiLoTables,
    amp_res: bool,
) -> Result<()> {
    let [c0, c1] = el.channels.as_slice() else {
        return Err(Error::SbrGridInvalid);
    };
    let n_q = bands.n_q();
    w.write_bit(false); // bs_data_extra
    w.write_bit(el.coupling);
    if el.coupling {
        if c0.grid != c1.grid || c0.invf.invf_mode.len() != n_q {
            return Err(Error::SbrGridInvalid);
        }
        write_grid(w, &c0.grid)?;
        write_dtdf(w, &c0.dtdf);
        write_dtdf(w, &c1.dtdf);
        write_invf(w, &c0.invf)?;
        let eff = amp_res && !c0.grid.amp_res_override;
        write_envelope(w, &c0.envelope, &c0.grid, &c0.dtdf, bands, true, false, eff)?;
        write_noise(w, &c0.noise, &c0.grid, &c0.dtdf, n_q, true, false, eff)?;
        write_envelope(w, &c1.envelope, &c0.grid, &c1.dtdf, bands, true, true, eff)?;
        write_noise(w, &c1.noise, &c0.grid, &c1.dtdf, n_q, true, true, eff)?;
    } else {
        if c0.invf.invf_mode.len() != n_q || c1.invf.invf_mode.len() != n_q {
            return Err(Error::SbrGridInvalid);
        }
        write_grid(w, &c0.grid)?;
        write_grid(w, &c1.grid)?;
        write_dtdf(w, &c0.dtdf);
        write_dtdf(w, &c1.dtdf);
        write_invf(w, &c0.invf)?;
        write_invf(w, &c1.invf)?;
        let eff0 = amp_res && !c0.grid.amp_res_override;
        let eff1 = amp_res && !c1.grid.amp_res_override;
        write_envelope(
            w,
            &c0.envelope,
            &c0.grid,
            &c0.dtdf,
            bands,
            false,
            false,
            eff0,
        )?;
        write_envelope(
            w,
            &c1.envelope,
            &c1.grid,
            &c1.dtdf,
            bands,
            false,
            true,
            eff1,
        )?;
        write_noise(w, &c0.noise, &c0.grid, &c0.dtdf, n_q, false, false, eff0)?;
        write_noise(w, &c1.noise, &c1.grid, &c1.dtdf, n_q, false, true, eff1)?;
    }
    write_harmonics(w, &c0.add_harmonic, bands.n_high())?;
    write_harmonics(w, &c1.add_harmonic, bands.n_high())?;
    write_extended_data(w, el.extension.as_ref());
    Ok(())
}

/// Build a complete `extension_payload()` body carrying
/// `sbr_extension_data()` (Table 4.62) for a `fill_element()`: the
/// returned bytes are exactly the `cnt` bytes
/// [`crate::raw_data_block::FrameAssembler::push_fill`] emits.
///
/// * `id_aac` — the core element the payload extends
///   ([`IdSynEle::Sce`] → single-channel element, [`IdSynEle::Cpe`]
///   → channel pair); anything else is rejected.
/// * `header` — `Some` transmits `bs_header_flag = 1` and the header;
///   `None` writes `bs_header_flag = 0` (header reuse). `active` is
///   the header in force (the one transmitted, or the last one), whose
///   `bs_amp_res` and band geometry size `sbr_data()`.
/// * `crc` — emit `EXT_SBR_DATA_CRC` with the 10-bit
///   `bs_sbr_crc_bits` computed over every bit after the CRC field to
///   the end of the padded payload (§4.5.2.8.1), else `EXT_SBR_DATA`.
pub fn build_extension_payload(
    id_aac: IdSynEle,
    header: Option<&SbrHeader>,
    active: &SbrHeader,
    element: &SbrElement,
    bands: &HiLoTables,
    crc: bool,
) -> Result<Vec<u8>> {
    let mut w = BitWriter::new();
    let ty = if crc {
        ExtensionType::SbrDataCrc
    } else {
        ExtensionType::SbrData
    };
    w.write_u32(u32::from(ty.as_u8()), 4);
    if crc {
        w.write_u32(0, SBR_CRC_BITS); // placeholder, patched below
    }
    match header {
        Some(h) => {
            w.write_bit(true);
            write_header(&mut w, h);
        }
        None => w.write_bit(false),
    }
    match id_aac {
        IdSynEle::Sce => write_single_element(&mut w, element, bands, active.amp_res)?,
        IdSynEle::Cpe => write_pair_element(&mut w, element, bands, active.amp_res)?,
        _ => return Err(Error::SbrGridInvalid),
    }
    // bs_fill_bits: num_align_bits = (8·cnt − 4 − num_sbr_bits) % 8,
    // i.e. pad to the next whole byte.
    w.align_to_byte_zero();
    let mut bytes = w.finish();
    if crc {
        let end = bytes.len() as u64 * 8;
        let start = 4 + u64::from(SBR_CRC_BITS);
        let value = crate::adts_crc::sbr_crc(&bytes, start, end);
        // Patch the 10 bits at positions 4..14 (MSB first).
        for i in 0..SBR_CRC_BITS {
            let bit = (value >> (SBR_CRC_BITS - 1 - i)) & 1;
            let pos = 4 + i as usize;
            let byte = pos / 8;
            let shift = 7 - (pos % 8);
            bytes[byte] = (bytes[byte] & !(1 << shift)) | ((bit as u8) << shift);
        }
    }
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sbr_element::SbrChannel;
    use crate::sbr_extension::SbrExtensionData;
    use crate::sbr_huffman::{
        F_HUFFMAN_ENV_1_5DB, F_HUFFMAN_ENV_3_0DB, F_HUFFMAN_ENV_BAL_1_5DB, F_HUFFMAN_ENV_BAL_3_0DB,
        T_HUFFMAN_ENV_1_5DB, T_HUFFMAN_ENV_3_0DB, T_HUFFMAN_ENV_BAL_1_5DB, T_HUFFMAN_ENV_BAL_3_0DB,
        T_HUFFMAN_NOISE_3_0DB, T_HUFFMAN_NOISE_BAL_3_0DB,
    };
    use oxideav_core::bits::BitReader;

    fn header() -> SbrHeader {
        SbrHeader {
            amp_res: true,
            start_freq: 5,
            stop_freq: 5,
            xover_band: 0,
            reserved: 0,
            header_extra_1: false,
            header_extra_2: false,
            freq_scale: 2,
            alter_scale: true,
            noise_bands: 2,
            limiter_bands: 2,
            limiter_gains: 2,
            interpol_freq: true,
            smoothing_mode: true,
        }
    }

    fn bands(h: &SbrHeader) -> HiLoTables {
        h.derive_bands(44_100).unwrap()
    }

    /// Every codeword of every table round-trips through the writer
    /// and the decoder's `sbr_huff_dec`.
    #[test]
    fn huffman_tables_round_trip() {
        let tables: [(&[(u8, u32)], i32); 10] = [
            (&T_HUFFMAN_ENV_1_5DB, 60),
            (&F_HUFFMAN_ENV_1_5DB, 60),
            (&T_HUFFMAN_ENV_BAL_1_5DB, 24),
            (&F_HUFFMAN_ENV_BAL_1_5DB, 24),
            (&T_HUFFMAN_ENV_3_0DB, 31),
            (&F_HUFFMAN_ENV_3_0DB, 31),
            (&T_HUFFMAN_ENV_BAL_3_0DB, 12),
            (&F_HUFFMAN_ENV_BAL_3_0DB, 12),
            (&T_HUFFMAN_NOISE_3_0DB, 31),
            (&T_HUFFMAN_NOISE_BAL_3_0DB, 12),
        ];
        for (table, lav) in tables {
            let mut w = BitWriter::new();
            for v in -lav..=lav {
                sbr_huff_enc(&mut w, table, lav, v).unwrap();
            }
            assert!(sbr_huff_enc(&mut w, table, lav, lav + 1).is_err());
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            for v in -lav..=lav {
                assert_eq!(
                    crate::sbr_huffman::sbr_huff_dec(&mut r, table, lav).unwrap(),
                    v
                );
            }
        }
    }

    fn fixfix_channel(bands: &HiLoTables, num_env: usize, high: bool) -> SbrChannel {
        let grid = SbrGrid {
            frame_class: FrameClass::FixFix,
            num_env,
            num_noise: if num_env > 1 { 2 } else { 1 },
            freq_res: vec![high; num_env],
            var_bord_0: 0,
            var_bord_1: 0,
            rel_bord_0: vec![],
            rel_bord_1: vec![],
            pointer: 0,
            amp_res_override: num_env == 1,
        };
        let n = if high { bands.n_high() } else { bands.n_low() };
        let dtdf = SbrDtdf {
            df_env: (0..num_env).map(|l| l % 2 == 1).collect(),
            df_noise: vec![false; grid.num_noise],
        };
        let envelope = SbrEnvelopeData {
            data: (0..num_env)
                .map(|l| {
                    (0..n)
                        .map(|k| {
                            if k == 0 && !dtdf.df_env[l] {
                                40 + l as i32
                            } else {
                                (k as i32 % 5) - 2
                            }
                        })
                        .collect()
                })
                .collect(),
        };
        let noise = SbrNoiseData {
            data: (0..grid.num_noise)
                .map(|_| {
                    (0..bands.n_q())
                        .map(|k| if k == 0 { 12 } else { 1 })
                        .collect()
                })
                .collect(),
        };
        SbrChannel {
            grid,
            dtdf,
            invf: SbrInvf {
                invf_mode: (0..bands.n_q()).map(|k| (k % 4) as u8).collect(),
            },
            envelope,
            noise,
            add_harmonic: Vec::new(),
        }
    }

    #[test]
    fn single_element_payload_round_trips_with_and_without_crc() {
        let h = header();
        let b = bands(&h);
        for crc in [false, true] {
            for (num_env, high) in [(1usize, true), (2, false), (4, true)] {
                let mut ch = fixfix_channel(&b, num_env, high);
                if num_env == 4 {
                    ch.add_harmonic = (0..b.n_high()).map(|k| k % 3 == 0).collect();
                }
                let el = SbrElement {
                    coupling: false,
                    channels: vec![ch],
                    extension: None,
                };
                let bytes =
                    build_extension_payload(IdSynEle::Sce, Some(&h), &h, &el, &b, crc).unwrap();
                let mut r = BitReader::new(&bytes);
                assert_eq!(r.read_u32(4).unwrap(), if crc { 14 } else { 13 });
                let parsed = SbrExtensionData::parse(
                    &mut r,
                    IdSynEle::Sce,
                    crc,
                    44_100,
                    Some(bytes.len() as u32),
                    None,
                )
                .unwrap();
                assert!(parsed.header_present);
                assert_eq!(parsed.header, h);
                assert_eq!(parsed.element, el);
                parsed.verify_crc(&bytes).unwrap();
                assert_eq!(r.bit_position(), bytes.len() as u64 * 8);
            }
        }
    }

    #[test]
    fn header_reuse_and_variable_classes_round_trip() {
        let h = header();
        let b = bands(&h);
        let mut ch0 = fixfix_channel(&b, 2, true);
        // FIXVAR with 3 envelopes, mixed resolution.
        ch0.grid = SbrGrid {
            frame_class: FrameClass::FixVar,
            num_env: 3,
            num_noise: 2,
            freq_res: vec![true, false, true],
            var_bord_0: 0,
            var_bord_1: 2,
            rel_bord_0: vec![],
            rel_bord_1: vec![1, 0],
            pointer: 2,
            amp_res_override: false,
        };
        ch0.dtdf = SbrDtdf {
            df_env: vec![false, true, false],
            df_noise: vec![false, true],
        };
        ch0.envelope = SbrEnvelopeData {
            data: vec![
                vec![30; b.n_high()],
                vec![-1; b.n_low()],
                vec![20; b.n_high()],
            ],
        };
        ch0.noise = SbrNoiseData {
            data: vec![vec![5; b.n_q()], vec![0; b.n_q()]],
        };
        let mut ch1 = fixfix_channel(&b, 4, false);
        ch1.grid = SbrGrid {
            frame_class: FrameClass::VarVar,
            num_env: 4,
            num_noise: 2,
            freq_res: vec![false, false, true, true],
            var_bord_0: 1,
            var_bord_1: 3,
            rel_bord_0: vec![2],
            rel_bord_1: vec![0, 3],
            pointer: 3,
            amp_res_override: false,
        };
        ch1.dtdf = SbrDtdf {
            df_env: vec![false, true, true, false],
            df_noise: vec![true, false],
        };
        ch1.envelope = SbrEnvelopeData {
            data: vec![
                vec![10; b.n_low()],
                vec![2; b.n_low()],
                vec![-3; b.n_high()],
                vec![15; b.n_high()],
            ],
        };
        ch1.noise = SbrNoiseData {
            data: vec![vec![-2; b.n_q()], vec![7; b.n_q()]],
        };
        let el = SbrElement {
            coupling: false,
            channels: vec![ch0, ch1],
            extension: Some(crate::sbr_element::SbrExtension {
                id: 1,
                data: vec![0xAB, 0xCD],
            }),
        };
        let bytes = build_extension_payload(IdSynEle::Cpe, None, &h, &el, &b, true).unwrap();
        let mut r = BitReader::new(&bytes);
        r.read_u32(4).unwrap();
        let parsed = SbrExtensionData::parse(
            &mut r,
            IdSynEle::Cpe,
            true,
            44_100,
            Some(bytes.len() as u32),
            Some(h),
        )
        .unwrap();
        assert!(!parsed.header_present);
        assert_eq!(parsed.element.channels, el.channels);
        // The parser re-packs the extension body bit-exactly and
        // zero-pads it to whole bytes, so the written bytes are a
        // prefix of the captured ones.
        let ext = parsed.element.extension.as_ref().unwrap();
        assert_eq!(ext.id, 1);
        assert!(ext.data.starts_with(&[0xAB, 0xCD]));
        parsed.verify_crc(&bytes).unwrap();
        // Any single flipped payload bit after the CRC field breaks it.
        let mut bad = bytes.clone();
        bad[3] ^= 0x10;
        let mut r = BitReader::new(&bad);
        r.read_u32(4).unwrap();
        if let Ok(p) = SbrExtensionData::parse(
            &mut r,
            IdSynEle::Cpe,
            true,
            44_100,
            Some(bad.len() as u32),
            Some(h),
        ) {
            assert!(p.verify_crc(&bad).is_err());
        }
    }

    #[test]
    fn coupled_pair_round_trips() {
        let mut h = header();
        h.header_extra_1 = true;
        h.header_extra_2 = true;
        h.freq_scale = 1;
        h.noise_bands = 1;
        h.limiter_gains = 1;
        let b = bands(&h);
        let c0 = fixfix_channel(&b, 2, true);
        let mut c1 = fixfix_channel(&b, 2, true);
        c1.invf = SbrInvf {
            invf_mode: Vec::new(),
        };
        // Balance channel: even deltas within the balance LAV.
        for row in c1.envelope.data.iter_mut() {
            for (k, v) in row.iter_mut().enumerate() {
                *v = if k == 0 { 12 } else { 2 };
            }
        }
        for row in c1.noise.data.iter_mut() {
            for (k, v) in row.iter_mut().enumerate() {
                *v = if k == 0 { 12 } else { -2 };
            }
        }
        let el = SbrElement {
            coupling: true,
            channels: vec![c0, c1],
            extension: None,
        };
        let bytes = build_extension_payload(IdSynEle::Cpe, Some(&h), &h, &el, &b, false).unwrap();
        let mut r = BitReader::new(&bytes);
        r.read_u32(4).unwrap();
        let parsed = SbrExtensionData::parse(
            &mut r,
            IdSynEle::Cpe,
            false,
            44_100,
            Some(bytes.len() as u32),
            None,
        )
        .unwrap();
        assert_eq!(parsed.header, h);
        assert_eq!(parsed.element, el);
    }

    #[test]
    fn writer_rejects_malformed_grids_and_deltas() {
        let h = header();
        let b = bands(&h);
        let mut w = BitWriter::new();
        let mut g = fixfix_channel(&b, 2, true).grid;
        g.num_env = 3; // not a power of two for FIXFIX
        g.freq_res = vec![true; 3];
        assert!(matches!(write_grid(&mut w, &g), Err(Error::SbrGridInvalid)));
        g.frame_class = FrameClass::FixVar;
        g.rel_bord_1 = vec![1]; // needs num_env - 1 = 2 entries
        assert!(matches!(write_grid(&mut w, &g), Err(Error::SbrGridInvalid)));
        let mut ch = fixfix_channel(&b, 1, true);
        ch.envelope.data[0][0] = 200; // exceeds the 7-bit start field
        let el = SbrElement {
            coupling: false,
            channels: vec![ch],
            extension: None,
        };
        assert!(matches!(
            build_extension_payload(IdSynEle::Sce, Some(&h), &h, &el, &b, false),
            Err(Error::SbrHuffInvalid)
        ));
        assert!(matches!(
            build_extension_payload(IdSynEle::Lfe, Some(&h), &h, &el, &b, false),
            Err(Error::SbrGridInvalid)
        ));
    }
}

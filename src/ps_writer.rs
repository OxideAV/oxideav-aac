//! `ps_data()` bitstream writer — ISO/IEC 14496-3:2009 §8.4.2 Tables
//! 8.9–8.14 written forward.
//!
//! The exact inverse of [`crate::ps_data::PsData::parse`]: every field
//! of a [`PsData`] is serialised in Table 8.9 order — the optional
//! `enable_ps_header` configuration block, `frame_class` /
//! `num_env_idx` (Table 8.29 inverted), the VAR_BORDERS
//! `border_position` list, the per-envelope `iid_dt` / `icc_dt` flags
//! with their Huffman-coded delta rows, and the byte-counted
//! extension layer (`ps_extension_size` + escape, `ps_extension_id =
//! 0`, `enable_ipdopd`, the IPD/OPD rows, `reserved_ps`, `fill_bits`)
//! — so a written payload reparses to the identical struct (pinned
//! by the round-trip tests).
//!
//! [`ps_huff_enc`] is the Annex 8.B codebook lookup (table index
//! `delta + LAV` → `(length, codeword)`), the inverse of
//! [`crate::ps_huffman::ps_huff_dec`]; [`ps_huff_bits`] prices a delta
//! without writing it, which is what the encoder's time-versus-
//! frequency differential election measures.
//!
//! The extension byte count is chosen as `⌈bits / 8⌉` so that fewer
//! than 8 fill bits follow the `ps_extension()` body — the Table 8.9
//! `while (num_bits_left > 7)` loop must not find a second
//! `ps_extension_id` inside the padding.
//!
//! All truth from ISO/IEC 14496-3:2009 subpart 8 staged under
//! `docs/audio/aac/`.

use oxideav_core::bits::BitWriter;

use crate::ps_data::{PsConfig, PsData};
use crate::ps_huffman::{
    HUFF_ICC_DF, HUFF_ICC_DT, HUFF_IID_DF, HUFF_IID_DT, HUFF_IID_FINE_DF, HUFF_IID_FINE_DT,
    HUFF_IPD_DF, HUFF_IPD_DT, HUFF_OPD_DF, HUFF_OPD_DT,
};
use crate::{Error, Result};

/// The `(length, codeword)` entry for `value` in `table` (index
/// `value + lav`), or [`Error::PsDataInvalid`] when the delta is
/// outside the codebook.
#[inline]
fn lookup(table: &[(u8, u32)], lav: i32, value: i32) -> Result<(u8, u32)> {
    let idx = value + lav;
    if idx < 0 {
        return Err(Error::PsDataInvalid);
    }
    table.get(idx as usize).copied().ok_or(Error::PsDataInvalid)
}

/// Write the Annex 8.B codeword for `value` (a signed DPCM delta;
/// index `value + lav` into `table`).
pub fn ps_huff_enc(w: &mut BitWriter, table: &[(u8, u32)], lav: i32, value: i32) -> Result<()> {
    let (len, code) = lookup(table, lav, value)?;
    w.write_u32(code, u32::from(len));
    Ok(())
}

/// The codeword length in bits of `value` in `table`, or `None` when
/// the delta is outside the codebook.
#[must_use]
pub fn ps_huff_bits(table: &[(u8, u32)], lav: i32, value: i32) -> Option<u32> {
    lookup(table, lav, value)
        .ok()
        .map(|(len, _)| u32::from(len))
}

/// The `(table, lav)` pair for an IID delta row on `config`'s grid in
/// the time (`dt = true`) or frequency direction.
#[must_use]
pub fn iid_table(config: &PsConfig, dt: bool) -> (&'static [(u8, u32)], i32) {
    match (config.iid_quant_fine(), dt) {
        (false, false) => (&HUFF_IID_DF, 14),
        (false, true) => (&HUFF_IID_DT, 14),
        (true, false) => (&HUFF_IID_FINE_DF, 30),
        (true, true) => (&HUFF_IID_FINE_DT, 30),
    }
}

/// The `(table, lav)` pair for an ICC delta row.
#[must_use]
pub fn icc_table(dt: bool) -> (&'static [(u8, u32)], i32) {
    if dt {
        (&HUFF_ICC_DT, 7)
    } else {
        (&HUFF_ICC_DF, 7)
    }
}

/// The `(table, lav)` pair for an IPD delta row (raw modulo-8 index
/// deltas, `lav = 0`).
#[must_use]
pub fn ipd_table(dt: bool) -> (&'static [(u8, u32)], i32) {
    if dt {
        (&HUFF_IPD_DT, 0)
    } else {
        (&HUFF_IPD_DF, 0)
    }
}

/// The `(table, lav)` pair for an OPD delta row.
#[must_use]
pub fn opd_table(dt: bool) -> (&'static [(u8, u32)], i32) {
    if dt {
        (&HUFF_OPD_DT, 0)
    } else {
        (&HUFF_OPD_DF, 0)
    }
}

/// Bits of one delta row under `(table, lav)`, or `None` if any delta
/// is outside the codebook.
#[must_use]
pub fn row_bits(table: &[(u8, u32)], lav: i32, row: &[i32]) -> Option<u32> {
    row.iter()
        .try_fold(0u32, |acc, &d| ps_huff_bits(table, lav, d).map(|b| acc + b))
}

/// Table 8.29 inverted: `num_env_idx` for `(frame_class, num_env)`.
fn num_env_idx(frame_class: bool, num_env: usize) -> Result<u32> {
    let idx = if frame_class {
        match num_env {
            1 => 0,
            2 => 1,
            3 => 2,
            4 => 3,
            _ => return Err(Error::PsDataInvalid),
        }
    } else {
        match num_env {
            0 => 0,
            1 => 1,
            2 => 2,
            4 => 3,
            _ => return Err(Error::PsDataInvalid),
        }
    };
    Ok(idx)
}

/// Write the delta rows of one parameter kind: per envelope the
/// `*_dt[e]` flag then `nr_par` codewords.
fn write_rows(
    w: &mut BitWriter,
    dt: &[bool],
    rows: &[Vec<i32>],
    nr_par: usize,
    num_env: usize,
    table_for: impl Fn(bool) -> (&'static [(u8, u32)], i32),
) -> Result<()> {
    if dt.len() != num_env || rows.len() != num_env {
        return Err(Error::PsDataInvalid);
    }
    for e in 0..num_env {
        if rows[e].len() != nr_par {
            return Err(Error::PsDataInvalid);
        }
        w.write_bit(dt[e]);
        let (table, lav) = table_for(dt[e]);
        for &d in &rows[e] {
            ps_huff_enc(w, table, lav, d)?;
        }
    }
    Ok(())
}

/// Serialise one `ps_data()` element (Table 8.9) into `w`.
///
/// Validates the shape: `num_env` must be a Table 8.29 value for the
/// frame class, VAR_BORDERS needs one `border_position` (≤ 31) per
/// envelope, every enabled parameter kind needs `num_env` rows of
/// the configured width, IPD/OPD rows are only written when
/// `config.enable_ext && enable_ipdopd` (and `enable_ipdopd` needs
/// `enable_iid`, §8.5.2), and every delta must sit inside its
/// codebook. Anything else is [`Error::PsDataInvalid`].
pub fn write_ps_data(w: &mut BitWriter, ps: &PsData) -> Result<()> {
    let config = &ps.config;
    if config.iid_mode > 5 || config.icc_mode > 5 {
        return Err(Error::PsDataInvalid);
    }
    w.write_bit(ps.header_present);
    if ps.header_present {
        w.write_bit(config.enable_iid);
        if config.enable_iid {
            w.write_u32(u32::from(config.iid_mode), 3);
        }
        w.write_bit(config.enable_icc);
        if config.enable_icc {
            w.write_u32(u32::from(config.icc_mode), 3);
        }
        w.write_bit(config.enable_ext);
    }

    w.write_bit(ps.frame_class);
    w.write_u32(num_env_idx(ps.frame_class, ps.num_env)?, 2);
    if ps.frame_class {
        if ps.border_position.len() != ps.num_env {
            return Err(Error::PsDataInvalid);
        }
        for &b in &ps.border_position {
            if b > 31 {
                return Err(Error::PsDataInvalid);
            }
            w.write_u32(u32::from(b), 5);
        }
    }

    if config.enable_iid {
        write_rows(
            w,
            &ps.iid_dt,
            &ps.iid_deltas,
            config.nr_iid_par(),
            ps.num_env,
            |dt| iid_table(config, dt),
        )?;
    }
    if config.enable_icc {
        write_rows(
            w,
            &ps.icc_dt,
            &ps.icc_deltas,
            config.nr_icc_par(),
            ps.num_env,
            icc_table,
        )?;
    }

    if config.enable_ext {
        if ps.enable_ipdopd && !config.enable_iid {
            return Err(Error::PsDataInvalid);
        }
        // Build the ps_extension(0) body first to size it.
        let mut body = BitWriter::new();
        body.write_u32(0, 2); // ps_extension_id = 0
        body.write_bit(ps.enable_ipdopd);
        if ps.enable_ipdopd {
            let nr = config.nr_ipdopd_par();
            if ps.ipd_dt.len() != ps.num_env
                || ps.opd_dt.len() != ps.num_env
                || ps.ipd_deltas.len() != ps.num_env
                || ps.opd_deltas.len() != ps.num_env
            {
                return Err(Error::PsDataInvalid);
            }
            for e in 0..ps.num_env {
                if ps.ipd_deltas[e].len() != nr || ps.opd_deltas[e].len() != nr {
                    return Err(Error::PsDataInvalid);
                }
                body.write_bit(ps.ipd_dt[e]);
                let (t, lav) = ipd_table(ps.ipd_dt[e]);
                for &d in &ps.ipd_deltas[e] {
                    ps_huff_enc(&mut body, t, lav, d)?;
                }
                body.write_bit(ps.opd_dt[e]);
                let (t, lav) = opd_table(ps.opd_dt[e]);
                for &d in &ps.opd_deltas[e] {
                    ps_huff_enc(&mut body, t, lav, d)?;
                }
            }
        }
        body.write_bit(false); // reserved_ps
        let bits = body.bit_position();
        // ⌈bits/8⌉ bytes: fewer than 8 fill bits remain, so the
        // parser's extension loop ends after this one block.
        let cnt = bits.div_ceil(8) as u32;
        if cnt >= 15 {
            w.write_u32(15, 4);
            w.write_u32(cnt - 15, 8);
        } else {
            w.write_u32(cnt, 4);
        }
        let padded = body.finish(); // zero-padded to whole bytes
        for &b in padded.iter().take(cnt as usize) {
            w.write_u32(u32::from(b), 8);
        }
    } else if ps.enable_ipdopd {
        return Err(Error::PsDataInvalid);
    }
    Ok(())
}

/// Serialise one `ps_data()` element to bytes (zero-padded to a whole
/// byte, as the `sbr_extension()` carrier pads with `bs_fill_bits`).
pub fn ps_data_bytes(ps: &PsData) -> Result<Vec<u8>> {
    let mut w = BitWriter::new();
    write_ps_data(&mut w, ps)?;
    w.align_to_byte_zero();
    Ok(w.finish())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ps_data::PsIndexState;
    use crate::ps_huffman::ps_huff_dec;
    use oxideav_core::bits::BitReader;

    /// Every codebook: encode → decode is the identity over the whole
    /// index range, and the priced length matches the written bits.
    #[test]
    fn huffman_encode_decode_identity_all_tables() {
        let tables: [(&[(u8, u32)], i32); 10] = [
            (&HUFF_IID_DF, 14),
            (&HUFF_IID_DT, 14),
            (&HUFF_IID_FINE_DF, 30),
            (&HUFF_IID_FINE_DT, 30),
            (&HUFF_ICC_DF, 7),
            (&HUFF_ICC_DT, 7),
            (&HUFF_IPD_DF, 0),
            (&HUFF_IPD_DT, 0),
            (&HUFF_OPD_DF, 0),
            (&HUFF_OPD_DT, 0),
        ];
        for (table, lav) in tables {
            let n = table.len() as i32;
            for value in -lav..(n - lav) {
                let mut w = BitWriter::new();
                ps_huff_enc(&mut w, table, lav, value).unwrap();
                let bits = w.bit_position();
                assert_eq!(Some(bits as u32), ps_huff_bits(table, lav, value));
                let bytes = w.finish();
                let mut r = BitReader::new(&bytes);
                assert_eq!(ps_huff_dec(&mut r, table, lav).unwrap(), value);
                assert_eq!(r.bit_position(), bits);
            }
            assert!(ps_huff_enc(&mut BitWriter::new(), table, lav, n - lav).is_err());
            assert!(ps_huff_enc(&mut BitWriter::new(), table, lav, -lav - 1).is_err());
            assert_eq!(ps_huff_bits(table, lav, n), None);
        }
    }

    fn config(iid_mode: u8, icc_mode: u8, ext: bool) -> PsConfig {
        PsConfig {
            enable_iid: true,
            iid_mode,
            enable_icc: true,
            icc_mode,
            enable_ext: ext,
        }
    }

    /// Deterministic delta rows inside the codebook ranges.
    fn rows(num_env: usize, nr: usize, span: i32, seed: i32) -> Vec<Vec<i32>> {
        (0..num_env)
            .map(|e| {
                (0..nr)
                    .map(|b| ((b as i32 * 7 + e as i32 * 3 + seed) % (2 * span + 1)) - span)
                    .collect()
            })
            .collect()
    }

    fn round_trip(ps: &PsData, prev: Option<&PsConfig>) -> PsData {
        let bytes = ps_data_bytes(ps).unwrap();
        let mut r = BitReader::new(&bytes);
        let back = PsData::parse(&mut r, prev).unwrap().expect("decodable");
        // Only fill bits may follow.
        assert!(bytes.len() * 8 - (r.bit_position() as usize) < 8);
        back
    }

    /// Header'd FIX_BORDERS elements on every IID / ICC mode
    /// combination, with both coding directions, reparse identically.
    #[test]
    fn fix_borders_all_modes_round_trip() {
        for iid_mode in 0..=5u8 {
            for icc_mode in 0..=5u8 {
                for num_env in [1usize, 2, 4] {
                    let cfg = config(iid_mode, icc_mode, false);
                    let span = if cfg.iid_quant_fine() { 8 } else { 4 };
                    let ps = PsData {
                        header_present: true,
                        config: cfg,
                        frame_class: false,
                        num_env,
                        border_position: Vec::new(),
                        iid_dt: (0..num_env).map(|e| e % 2 == 1).collect(),
                        iid_deltas: rows(num_env, cfg.nr_iid_par(), span, 1),
                        icc_dt: (0..num_env).map(|e| e % 2 == 0).collect(),
                        icc_deltas: rows(num_env, cfg.nr_icc_par(), 2, 2),
                        enable_ipdopd: false,
                        ipd_dt: Vec::new(),
                        ipd_deltas: Vec::new(),
                        opd_dt: Vec::new(),
                        opd_deltas: Vec::new(),
                    };
                    assert_eq!(round_trip(&ps, None), ps);
                }
            }
        }
    }

    /// VAR_BORDERS with the IPD/OPD extension layer: the byte-counted
    /// block, the escape-free size field and the fill bits reparse.
    #[test]
    fn var_borders_with_phase_extension_round_trip() {
        for iid_mode in [0u8, 1, 2, 5] {
            for num_env in 1..=4usize {
                let cfg = config(iid_mode, 3, true);
                let nr = cfg.nr_ipdopd_par();
                let ps = PsData {
                    header_present: true,
                    config: cfg,
                    frame_class: true,
                    num_env,
                    border_position: (0..num_env).map(|e| (8 * e + 7) as u8).collect(),
                    iid_dt: vec![false; num_env],
                    // Alternating ±1 keeps the frequency accumulation
                    // inside the ±7 index range for the resolve below.
                    iid_deltas: (0..num_env)
                        .map(|_| {
                            (0..cfg.nr_iid_par())
                                .map(|b| if b % 2 == 0 { 1 } else { -1 })
                                .collect()
                        })
                        .collect(),
                    icc_dt: vec![true; num_env],
                    icc_deltas: (0..num_env)
                        .map(|_| (0..cfg.nr_icc_par()).map(|b| (b % 2) as i32).collect())
                        .collect(),
                    enable_ipdopd: true,
                    ipd_dt: (0..num_env).map(|e| e % 2 == 0).collect(),
                    ipd_deltas: (0..num_env)
                        .map(|e| (0..nr).map(|b| ((b + e) % 8) as i32).collect())
                        .collect(),
                    opd_dt: (0..num_env).map(|e| e % 2 == 1).collect(),
                    opd_deltas: (0..num_env)
                        .map(|e| (0..nr).map(|b| ((3 * b + e) % 8) as i32).collect())
                        .collect(),
                };
                let back = round_trip(&ps, None);
                assert_eq!(back, ps, "iid_mode {iid_mode} num_env {num_env}");
                // The decoder resolves the phases modulo 8 without
                // complaint.
                let mut st = PsIndexState::default();
                let idx = back.resolve(&mut st).unwrap();
                assert_eq!(idx.ipd.len(), num_env);
            }
        }
    }

    /// Headerless elements inherit the configuration; the `num_env
    /// == 0` hold element is four bits.
    #[test]
    fn headerless_and_hold_elements() {
        let cfg = config(1, 1, false);
        let hold = PsData {
            header_present: false,
            config: cfg,
            frame_class: false,
            num_env: 0,
            border_position: Vec::new(),
            iid_dt: Vec::new(),
            iid_deltas: Vec::new(),
            icc_dt: Vec::new(),
            icc_deltas: Vec::new(),
            enable_ipdopd: false,
            ipd_dt: Vec::new(),
            ipd_deltas: Vec::new(),
            opd_dt: Vec::new(),
            opd_deltas: Vec::new(),
        };
        let mut w = BitWriter::new();
        write_ps_data(&mut w, &hold).unwrap();
        // enable_ps_header + frame_class + num_env_idx.
        assert_eq!(w.bit_position(), 4);
        assert_eq!(round_trip(&hold, Some(&cfg)), hold);
        // Extension enabled but no IPD/OPD this frame: one body byte.
        let cfg_ext = config(1, 1, true);
        let mut none = hold.clone();
        none.config = cfg_ext;
        none.header_present = true;
        let mut w = BitWriter::new();
        write_ps_data(&mut w, &none).unwrap();
        // 1 + (1+3+1+3+1) + 1 + 2 + 4 + 8 = 25 bits.
        assert_eq!(w.bit_position(), 25);
        assert_eq!(round_trip(&none, None), none);
    }

    /// Shape violations are rejected rather than written.
    #[test]
    fn invalid_shapes_rejected() {
        let cfg = config(0, 0, false);
        let base = PsData {
            header_present: true,
            config: cfg,
            frame_class: false,
            num_env: 1,
            border_position: Vec::new(),
            iid_dt: vec![false],
            iid_deltas: vec![vec![0; 10]],
            icc_dt: vec![false],
            icc_deltas: vec![vec![0; 10]],
            enable_ipdopd: false,
            ipd_dt: Vec::new(),
            ipd_deltas: Vec::new(),
            opd_dt: Vec::new(),
            opd_deltas: Vec::new(),
        };
        assert!(ps_data_bytes(&base).is_ok());
        // FIX with 3 envelopes has no Table 8.29 index.
        let mut bad = base.clone();
        bad.num_env = 3;
        bad.iid_dt = vec![false; 3];
        bad.iid_deltas = vec![vec![0; 10]; 3];
        bad.icc_dt = vec![false; 3];
        bad.icc_deltas = vec![vec![0; 10]; 3];
        assert!(ps_data_bytes(&bad).is_err());
        // VAR needs borders ≤ 31.
        let mut bad = base.clone();
        bad.frame_class = true;
        bad.border_position = vec![32];
        assert!(ps_data_bytes(&bad).is_err());
        // Wrong row width.
        let mut bad = base.clone();
        bad.iid_deltas = vec![vec![0; 20]];
        assert!(ps_data_bytes(&bad).is_err());
        // Delta outside the coarse codebook (±14).
        let mut bad = base.clone();
        bad.iid_deltas = vec![{
            let mut v = vec![0; 10];
            v[0] = 15;
            v
        }];
        assert!(ps_data_bytes(&bad).is_err());
        // IPD/OPD without the extension layer.
        let mut bad = base.clone();
        bad.enable_ipdopd = true;
        assert!(ps_data_bytes(&bad).is_err());
        // Reserved modes.
        let mut bad = base;
        bad.config.iid_mode = 6;
        assert!(ps_data_bytes(&bad).is_err());
    }

    /// `row_bits` prices exactly what the writer emits.
    #[test]
    fn row_bits_matches_written_length() {
        let cfg = config(4, 1, false);
        let row: Vec<i32> = (0..20i32).map(|b| (b % 5) - 2).collect();
        for dt in [false, true] {
            let (t, lav) = iid_table(&cfg, dt);
            let mut w = BitWriter::new();
            for &d in &row {
                ps_huff_enc(&mut w, t, lav, d).unwrap();
            }
            assert_eq!(row_bits(t, lav, &row), Some(w.bit_position() as u32));
        }
        assert_eq!(row_bits(&HUFF_ICC_DF, 7, &[8]), None);
    }
}

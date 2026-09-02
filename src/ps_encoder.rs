//! Parametric Stereo encoder — ISO/IEC 14496-3:2009 Annex 8.C.6
//! written forward against the normative §8.6.4 decoder this crate
//! carries (the HE-AAC v2 stereo tool of Annex 8.A).
//!
//! One [`PsEncoder`] turns the 64-band QMF analysis of a stereo input
//! into one `ps_data()` payload per SBR frame:
//!
//! 1. **Analysis** ([`crate::ps_analysis`]) — both channels through
//!    the §8.6.4.3 hybrid filterbank on the Annex 8.A.3 `Xinput`
//!    framing (32 slots + 6 look-ahead), the Table 8.44 configuration
//!    following the parameter count (71 channels for 10/20 bands, 91
//!    for 34).
//! 2. **Parameter positions** (§8.6.4.4 / Table 8.29) — a transient
//!    detector over the per-slot excitation elects VAR_BORDERS on an
//!    attack whose stereo cues differ from the preceding region: the
//!    borders `[t−1, t, 31]` place the pre-attack cues at `t−1`, jump
//!    to the post-attack cues at `t` and hold them (the decoder's
//!    §8.6.4.6.4 interpolation between two identical parameter sets
//!    is flat). Otherwise FIX_BORDERS with 1, 2 or 4 envelopes,
//!    chosen by how far the quarter-frame cues stray from the coarser
//!    estimate; a frame whose single quantised parameter set repeats
//!    the previous one is sent as the four-bit `num_env = 0` hold.
//! 3. **Estimation** (Annex 8.C.6.2) — per envelope region and stereo
//!    band the excitations `e_l, e_r, e_R, e_O` give
//!    `iid = 10·log10(e_l/e_r)`, `ρ` (the real coherence for mixing
//!    procedure Ra — exactly the cross-correlation the decoder's
//!    `l = H11·s + H21·d` output reproduces — or the magnitude
//!    coherence when the phase layer carries the angle), and the
//!    IPD / OPD angles.
//! 4. **Quantisation** (Annex 8.C.6.3–8.C.6.5) — nearest entry of the
//!    Table 8.25 / 8.26 IID grid, the Table 8.28 ICC grid and the
//!    Table 8.31 `π/4` phase ladder (index 8 folding to 0).
//! 5. **Differential coding** (§8.5.2) — every envelope's row of each
//!    parameter kind is coded over frequency or over time (against
//!    the previous envelope, or the previous frame's last envelope
//!    through a decoder-mirrored [`PsIndexState`]) — whichever prices
//!    fewer Annex 8.B codebook bits. Envelope 0 of a header frame is
//!    always frequency-coded so a decoder joining there (§8.6.5.1)
//!    resolves the same indices; phase deltas run modulo 8.
//! 6. **Bitstream** ([`crate::ps_writer`]) — `enable_ps_header` every
//!    [`PsEncoderConfig::header_interval`] frames, the extension
//!    layer only when phase coding is configured.
//!
//! The closed loop is literal: after assembling the element the
//! encoder resolves it with [`PsData::resolve`] against its own copy
//! of the decoder state and asserts the decoder lands on the intended
//! indices.
//!
//! All truth from ISO/IEC 14496-3:2009 subpart 8 (Annexes 8.A / 8.C)
//! staged under `docs/audio/aac/`.

use crate::ps_analysis::{band_stats, hybrid_config_for, slot_energies, BandStats, PsAnalysis};
use crate::ps_data::{PsConfig, PsData, PsIndexState, PsIndices};
use crate::ps_hybrid::NUM_QMF_SLOTS;
use crate::ps_stereo::{ICC_RHO, IID_DB_COARSE, IID_DB_FINE};
use crate::ps_writer::{icc_table, iid_table, ipd_table, opd_table, row_bits, write_ps_data};
use crate::sbr_qmf::Complex;
use crate::{Error, Result};

/// The stereo parameter band count (Tables 8.24 / 8.27 `nr_*_par`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PsBands {
    /// 10 parameters (`iid_mode` / `icc_mode` 0 or 3).
    Ten,
    /// 20 parameters (modes 1 / 4) — the default.
    #[default]
    Twenty,
    /// 34 parameters (modes 2 / 5; the 91-channel hybrid bank).
    ThirtyFour,
}

impl PsBands {
    /// The parameter count.
    #[must_use]
    pub fn count(self) -> usize {
        match self {
            PsBands::Ten => 10,
            PsBands::Twenty => 20,
            PsBands::ThirtyFour => 34,
        }
    }

    fn mode(self) -> u8 {
        match self {
            PsBands::Ten => 0,
            PsBands::Twenty => 1,
            PsBands::ThirtyFour => 2,
        }
    }
}

/// Configuration of one [`PsEncoder`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PsEncoderConfig {
    /// Stereo band count for IID and ICC.
    pub bands: PsBands,
    /// Quantise IID on the fine Table 8.26 grid (`iid_mode + 3`).
    pub fine_iid: bool,
    /// Transmit ICC (`enable_icc`). Off leaves the decoder at ρ = 1
    /// (pure intensity panning).
    pub icc: bool,
    /// Code the phase layer: `enable_ext` with IPD/OPD in every
    /// frame, ICC as the magnitude coherence and mixing procedure Rb
    /// (`icc_mode + 3`). Off codes the real coherence for procedure
    /// Ra, which reproduces anti-phase content through `ρ < 0`.
    pub phase: bool,
    /// Transmit the `enable_ps_header` block every N frames (and
    /// always in the first). `0` behaves as `1`.
    pub header_interval: u32,
    /// Elect VAR_BORDERS on cue-changing transients; off keeps
    /// FIX_BORDERS grids only.
    pub variable_borders: bool,
}

impl Default for PsEncoderConfig {
    fn default() -> Self {
        PsEncoderConfig {
            bands: PsBands::Twenty,
            fine_iid: false,
            icc: true,
            phase: false,
            header_interval: 8,
            variable_borders: true,
        }
    }
}

impl PsEncoderConfig {
    /// The Table 8.9 header block this configuration transmits.
    #[must_use]
    pub fn ps_config(&self) -> PsConfig {
        PsConfig {
            enable_iid: true,
            iid_mode: self.bands.mode() + if self.fine_iid { 3 } else { 0 },
            enable_icc: self.icc,
            icc_mode: self.bands.mode() + if self.phase { 3 } else { 0 },
            enable_ext: self.phase,
        }
    }
}

/// IID tolerance (dB) under which two regions count as sharing a
/// parameter set — the Table 8.25 step around 0 dB.
const IID_TOL_DB: f64 = 2.0;
/// ICC tolerance under the same rule (about one Table 8.28 step in
/// the coherent half of the grid).
const ICC_TOL: f64 = 0.25;
/// Bands quieter than this fraction of the frame's loudest band are
/// ignored by the region-similarity test.
const QUIET_BAND_RATIO: f64 = 1e-3;

/// One encoded frame with its diagnostics.
#[derive(Debug, Clone, PartialEq)]
pub struct PsFrame {
    /// The `ps_data()` bytes (zero-padded to a whole byte) — the
    /// `sbr_extension()` body behind `bs_extension_id =
    /// EXTENSION_ID_PS`.
    pub payload: Vec<u8>,
    /// The element as the decoder will parse it.
    pub data: PsData,
    /// The absolute indices the decoder resolves (empty rows for a
    /// hold frame).
    pub indices: PsIndices,
    /// Parameter positions `n_e` (slot of each envelope's border).
    pub borders: Vec<usize>,
    /// Whether `enable_ps_header` was set.
    pub header_sent: bool,
    /// The attack slot that elected VAR_BORDERS, if any.
    pub transient: Option<usize>,
    /// Per-envelope band statistics the parameters were quantised
    /// from (one row per envelope region; a hold frame reports its
    /// single region).
    pub stats: Vec<Vec<BandStats>>,
    /// `ps_data()` bits before byte padding.
    pub bits: u32,
}

/// The Annex 8.C.6 parametric stereo encoder.
#[derive(Debug, Clone)]
pub struct PsEncoder {
    cfg: PsEncoderConfig,
    ps_config: PsConfig,
    analysis: PsAnalysis,
    frames: u64,
    /// The decoder's cross-frame differential state, mirrored.
    state: PsIndexState,
}

impl PsEncoder {
    /// Build an encoder for `cfg`.
    pub fn new(cfg: PsEncoderConfig) -> Result<Self> {
        let ps_config = cfg.ps_config();
        Ok(PsEncoder {
            cfg,
            ps_config,
            analysis: PsAnalysis::new(hybrid_config_for(cfg.bands.count())),
            frames: 0,
            state: PsIndexState::default(),
        })
    }

    /// The configuration.
    #[must_use]
    pub fn config(&self) -> &PsEncoderConfig {
        &self.cfg
    }

    /// The transmitted `ps_data()` header configuration.
    #[must_use]
    pub fn ps_config(&self) -> PsConfig {
        self.ps_config
    }

    /// Frames encoded so far.
    #[must_use]
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Whether the next [`encode_frame`](Self::encode_frame) transmits
    /// the header block.
    #[must_use]
    pub fn next_header_due(&self) -> bool {
        self.frames % u64::from(self.cfg.header_interval.max(1)) == 0
    }

    /// Encode one stereo frame. `l` / `r` are each channel's Annex
    /// 8.A.3 `Xinput` matrix: 38 slots of 64 QMF bands (32 frame
    /// slots + 6 look-ahead; the tail needs only the split bands).
    /// The header cadence follows
    /// [`PsEncoderConfig::header_interval`].
    pub fn encode_frame(&mut self, l: &[[Complex; 64]], r: &[[Complex; 64]]) -> Result<PsFrame> {
        let header = self.next_header_due();
        self.encode_frame_with_header(l, r, header)
    }

    /// [`encode_frame`](Self::encode_frame) with an explicit header
    /// decision (the first frame always carries one).
    pub fn encode_frame_with_header(
        &mut self,
        l: &[[Complex; 64]],
        r: &[[Complex; 64]],
        header: bool,
    ) -> Result<PsFrame> {
        let header = header || self.frames == 0;
        let n_pars = self.cfg.bands.count();
        let hyb = self.analysis.analyze(l, r)?;

        // Parameter positions.
        let energies = slot_energies(&hyb);
        let whole = band_stats(&hyb, n_pars, 0, NUM_QMF_SLOTS - 1)?;
        let attack = detect_attack(&energies);
        let mut transient = None;
        // (estimation region lo, hi, parameter position n_e).
        let (frame_class, regions): (bool, Vec<(usize, usize, usize)>) = match attack {
            Some(t) if self.cfg.variable_borders => {
                let pre = band_stats(&hyb, n_pars, 0, t - 1)?;
                let post = band_stats(&hyb, n_pars, t, NUM_QMF_SLOTS - 1)?;
                if cues_close(&pre, &post, self.cfg.phase) {
                    (
                        false,
                        fixed_regions(elect_fixed(&hyb, n_pars, &whole, self.cfg.phase)?),
                    )
                } else {
                    transient = Some(t);
                    if t == NUM_QMF_SLOTS - 1 {
                        (true, vec![(0, t - 1, t - 1), (t, t, t)])
                    } else {
                        (
                            true,
                            vec![
                                (0, t - 1, t - 1),
                                (t, NUM_QMF_SLOTS - 1, t),
                                (t, NUM_QMF_SLOTS - 1, NUM_QMF_SLOTS - 1),
                            ],
                        )
                    }
                }
            }
            _ => (
                false,
                fixed_regions(elect_fixed(&hyb, n_pars, &whole, self.cfg.phase)?),
            ),
        };
        let borders: Vec<usize> = regions.iter().map(|&(_, _, n_e)| n_e).collect();

        // Estimation + quantisation per envelope.
        let mut stats = Vec::with_capacity(regions.len());
        let mut iid_abs = Vec::with_capacity(regions.len());
        let mut icc_abs = Vec::with_capacity(regions.len());
        let mut ipd_abs = Vec::with_capacity(regions.len());
        let mut opd_abs = Vec::with_capacity(regions.len());
        let nr_phase = self.ps_config.nr_ipdopd_par();
        for &(lo, hi, _) in &regions {
            let st = if (lo, hi) == (0, NUM_QMF_SLOTS - 1) {
                whole.clone()
            } else {
                band_stats(&hyb, n_pars, lo, hi)?
            };
            iid_abs.push(
                st.iter()
                    .map(|s| quantise_iid(s.iid_db(), self.cfg.fine_iid))
                    .collect::<Vec<i32>>(),
            );
            if self.cfg.icc {
                icc_abs.push(
                    st.iter()
                        .map(|s| {
                            quantise_icc(if self.cfg.phase {
                                s.icc_magnitude()
                            } else {
                                s.icc_real()
                            })
                        })
                        .collect::<Vec<i32>>(),
                );
            }
            if self.cfg.phase {
                ipd_abs.push(
                    st.iter()
                        .take(nr_phase)
                        .map(|s| quantise_phase(s.ipd()))
                        .collect::<Vec<i32>>(),
                );
                opd_abs.push(
                    st.iter()
                        .take(nr_phase)
                        .map(|s| quantise_phase(s.opd()))
                        .collect::<Vec<i32>>(),
                );
            }
            stats.push(st);
        }

        // Hold: a single parameter set identical to the decoder's
        // current one needs no new data.
        let hold = !header
            && !frame_class
            && regions.len() == 1
            && iid_abs[0] == self.state.iid
            && (!self.cfg.icc || icc_abs[0] == self.state.icc)
            && (!self.cfg.phase || (ipd_abs[0] == self.state.ipd && opd_abs[0] == self.state.opd));

        let num_env = if hold { 0 } else { regions.len() };
        let mut data = PsData {
            header_present: header,
            config: self.ps_config,
            frame_class,
            num_env,
            border_position: if frame_class {
                borders.iter().map(|&b| b as u8).collect()
            } else {
                Vec::new()
            },
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

        if !hold {
            let cfg = self.ps_config;
            let (dt, deltas) = code_kind(&iid_abs, &self.state.iid, header, false, |t| {
                iid_table(&cfg, t)
            })?;
            data.iid_dt = dt;
            data.iid_deltas = deltas;
            if self.cfg.icc {
                let (dt, deltas) = code_kind(&icc_abs, &self.state.icc, header, false, icc_table)?;
                data.icc_dt = dt;
                data.icc_deltas = deltas;
            }
            if self.cfg.phase {
                data.enable_ipdopd = true;
                let (dt, deltas) = code_kind(&ipd_abs, &self.state.ipd, header, true, ipd_table)?;
                data.ipd_dt = dt;
                data.ipd_deltas = deltas;
                let (dt, deltas) = code_kind(&opd_abs, &self.state.opd, header, true, opd_table)?;
                data.opd_dt = dt;
                data.opd_deltas = deltas;
            }
        }

        // Closed loop: the decoder's own resolution must land on the
        // intended indices and evolve the mirrored state.
        let mut state = self.state.clone();
        let indices = data.resolve(&mut state)?;
        if !hold {
            let intended = PsIndices {
                iid: iid_abs,
                icc: icc_abs,
                ipd: ipd_abs,
                opd: opd_abs,
            };
            if indices != intended {
                return Err(Error::PsDataInvalid);
            }
        }
        self.state = state;

        let mut w = oxideav_core::bits::BitWriter::new();
        write_ps_data(&mut w, &data)?;
        let bits = w.bit_position() as u32;
        w.align_to_byte_zero();
        let payload = w.finish();

        self.frames += 1;
        Ok(PsFrame {
            payload,
            data,
            indices,
            borders,
            header_sent: header,
            transient,
            stats,
            bits,
        })
    }
}

/// The attack slot of a frame: the first slot at least 8× the mean of
/// the preceding four, carrying a tenth of the frame's peak and a
/// non-trivial absolute level.
fn detect_attack(e: &[f64]) -> Option<usize> {
    let peak = e.iter().cloned().fold(0.0f64, f64::max);
    (1..NUM_QMF_SLOTS).find(|&t| {
        let lo = t.saturating_sub(4);
        let prev = e[lo..t].iter().sum::<f64>() / (t - lo) as f64;
        e[t] > 8.0 * prev + 1e-9 && e[t] > 0.1 * peak && e[t] > 1.0
    })
}

/// FIX_BORDERS regions for `n` envelopes: `[32e/n, 32(e+1)/n − 1]`,
/// the parameter position at the region's last slot.
fn fixed_regions(n: usize) -> Vec<(usize, usize, usize)> {
    (0..n)
        .map(|e| {
            let hi = NUM_QMF_SLOTS * (e + 1) / n - 1;
            (NUM_QMF_SLOTS * e / n, hi, hi)
        })
        .collect()
}

/// Elect 1, 2 or 4 fixed envelopes from how far the quarter-frame cues
/// stray from the coarser estimates.
fn elect_fixed(
    hyb: &crate::ps_analysis::HybridStereo,
    n_pars: usize,
    whole: &[BandStats],
    phase: bool,
) -> Result<usize> {
    let quarters: Vec<Vec<BandStats>> = fixed_regions(4)
        .into_iter()
        .map(|(lo, hi, _)| band_stats(hyb, n_pars, lo, hi))
        .collect::<Result<_>>()?;
    if quarters.iter().all(|q| cues_close(q, whole, phase)) {
        return Ok(1);
    }
    let halves: Vec<Vec<BandStats>> = fixed_regions(2)
        .into_iter()
        .map(|(lo, hi, _)| band_stats(hyb, n_pars, lo, hi))
        .collect::<Result<_>>()?;
    let two = cues_close(&quarters[0], &halves[0], phase)
        && cues_close(&quarters[1], &halves[0], phase)
        && cues_close(&quarters[2], &halves[1], phase)
        && cues_close(&quarters[3], &halves[1], phase);
    Ok(if two { 2 } else { 4 })
}

/// Whether two regions' cues agree within one quantiser step in every
/// band that carries energy (relative to the loudest band of either).
fn cues_close(a: &[BandStats], b: &[BandStats], phase: bool) -> bool {
    let peak = a
        .iter()
        .chain(b.iter())
        .map(BandStats::energy)
        .fold(0.0f64, f64::max);
    let floor = peak * QUIET_BAND_RATIO;
    a.iter().zip(b.iter()).all(|(x, y)| {
        if x.energy() < floor && y.energy() < floor {
            return true;
        }
        let icc_x = if phase {
            x.icc_magnitude()
        } else {
            x.icc_real()
        };
        let icc_y = if phase {
            y.icc_magnitude()
        } else {
            y.icc_real()
        };
        (x.iid_db() - y.iid_db()).abs() <= IID_TOL_DB && (icc_x - icc_y).abs() <= ICC_TOL
    })
}

/// Annex 8.C.6.3 — nearest Table 8.25 / 8.26 index.
#[must_use]
pub fn quantise_iid(iid_db: f64, fine: bool) -> i32 {
    let grid: &[f64] = if fine { &IID_DB_FINE } else { &IID_DB_COARSE };
    let bound = (grid.len() / 2) as i32;
    let v = if iid_db.is_finite() {
        iid_db
    } else if iid_db > 0.0 {
        grid[grid.len() - 1]
    } else {
        grid[0]
    };
    let mut best = 0usize;
    let mut best_d = f64::INFINITY;
    for (i, &g) in grid.iter().enumerate() {
        let d = (v - g).abs();
        if d < best_d {
            best_d = d;
            best = i;
        }
    }
    best as i32 - bound
}

/// Annex 8.C.6.4 — nearest Table 8.28 index.
#[must_use]
pub fn quantise_icc(rho: f64) -> i32 {
    let v = if rho.is_finite() { rho } else { 1.0 };
    let mut best = 0usize;
    let mut best_d = f64::INFINITY;
    for (i, &g) in ICC_RHO.iter().enumerate() {
        let d = (v - g).abs();
        if d < best_d {
            best_d = d;
            best = i;
        }
    }
    best as i32
}

/// Annex 8.C.6.5 — nearest Table 8.31 `π/4` step, index 8 folded to 0.
#[must_use]
pub fn quantise_phase(rad: f64) -> i32 {
    if !rad.is_finite() {
        return 0;
    }
    let steps = (rad / core::f64::consts::FRAC_PI_4).round() as i64;
    steps.rem_euclid(8) as i32
}

/// Differential-code one parameter kind's absolute index rows: per
/// envelope the cheaper of frequency / time coding by measured
/// codebook bits. `force_freq0` pins envelope 0 to frequency coding
/// (header frames); `modulo8` selects the phase ladder's wrap-around
/// deltas. Returns `(dt flags, delta rows)`.
fn code_kind(
    rows: &[Vec<i32>],
    prev_state: &[i32],
    force_freq0: bool,
    modulo8: bool,
    table_for: impl Fn(bool) -> (&'static [(u8, u32)], i32),
) -> Result<(Vec<bool>, Vec<Vec<i32>>)> {
    let wrap = |d: i32| if modulo8 { d.rem_euclid(8) } else { d };
    let mut dts = Vec::with_capacity(rows.len());
    let mut out = Vec::with_capacity(rows.len());
    for (e, row) in rows.iter().enumerate() {
        let nr = row.len();
        // Frequency direction: band 0 against 0, then band b − 1.
        let mut freq = Vec::with_capacity(nr);
        let mut prev = 0i32;
        for &a in row {
            freq.push(wrap(a - prev));
            prev = a;
        }
        // Time direction: the previous envelope, or the decoder's
        // carried state (zeros on a width mismatch — §8.5.2 mode
        // change), mirroring `PsData::resolve`.
        let reference: Vec<i32> = if e > 0 {
            rows[e - 1].clone()
        } else if prev_state.len() == nr {
            prev_state.to_vec()
        } else {
            vec![0; nr]
        };
        let time: Vec<i32> = row
            .iter()
            .zip(reference.iter())
            .map(|(&a, &p)| wrap(a - p))
            .collect();
        let (tf, lf) = table_for(false);
        let (tt, lt) = table_for(true);
        let cost_f = row_bits(tf, lf, &freq);
        let cost_t = if e == 0 && force_freq0 {
            None
        } else {
            row_bits(tt, lt, &time)
        };
        match (cost_f, cost_t) {
            (Some(f), Some(t)) if t < f => {
                dts.push(true);
                out.push(time);
            }
            (Some(_), _) => {
                dts.push(false);
                out.push(freq);
            }
            (None, Some(_)) => {
                dts.push(true);
                out.push(time);
            }
            (None, None) => return Err(Error::PsDataInvalid),
        }
    }
    Ok((dts, out))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ps_hybrid::LOOKAHEAD;
    use oxideav_core::bits::BitReader;

    fn frame(f: impl Fn(usize, usize) -> Complex) -> Vec<[Complex; 64]> {
        (0..NUM_QMF_SLOTS + LOOKAHEAD)
            .map(|n| {
                let mut s = [Complex::default(); 64];
                for (k, c) in s.iter_mut().enumerate() {
                    *c = f(n, k);
                }
                s
            })
            .collect()
    }

    fn noise(seed: u64) -> impl Fn(usize, usize) -> Complex {
        move |n, k| {
            let mut h = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add((n * 64 + k) as u64 + 1);
            h ^= h >> 33;
            h = h.wrapping_mul(0xff51afd7ed558ccd);
            h ^= h >> 33;
            let re = (h & 0xFFFF) as f64 / 65535.0 - 0.5;
            let im = ((h >> 16) & 0xFFFF) as f64 / 65535.0 - 0.5;
            Complex::new(re * 1000.0, im * 1000.0)
        }
    }

    #[test]
    fn quantisers_hit_the_grids() {
        assert_eq!(quantise_iid(0.0, false), 0);
        assert_eq!(quantise_iid(2.9, false), 1);
        assert_eq!(quantise_iid(3.1, false), 2);
        assert_eq!(quantise_iid(-30.0, false), -7);
        assert_eq!(quantise_iid(f64::INFINITY, false), 7);
        assert_eq!(quantise_iid(f64::NEG_INFINITY, true), -15);
        assert_eq!(quantise_iid(23.0, true), 9); // 22 dB
        assert_eq!(quantise_icc(1.0), 0);
        assert_eq!(quantise_icc(0.9), 1);
        assert_eq!(quantise_icc(0.0), 5);
        assert_eq!(quantise_icc(-0.7), 6);
        assert_eq!(quantise_icc(-0.8), 7);
        assert_eq!(quantise_icc(-1.0), 7);
        assert_eq!(quantise_phase(0.0), 0);
        assert_eq!(quantise_phase(core::f64::consts::FRAC_PI_2), 2);
        assert_eq!(quantise_phase(-core::f64::consts::FRAC_PI_4), 7);
        assert_eq!(quantise_phase(core::f64::consts::PI), 4);
        assert_eq!(quantise_phase(2.0 * core::f64::consts::PI), 0);
    }

    /// The default configuration on a hard-left/right-panned frame:
    /// header'd first element, the decoder resolves ±25 dB IIDs, and
    /// a repeated frame becomes a hold element.
    #[test]
    fn panned_input_codes_extreme_iid_then_holds() {
        let mut enc = PsEncoder::new(PsEncoderConfig::default()).unwrap();
        let sig = noise(5);
        let l = frame(&sig);
        let r = frame(|n, k| sig(n, k) * 0.01); // −40 dB
        let f0 = enc.encode_frame(&l, &r).unwrap();
        assert!(f0.header_sent);
        assert!(f0.data.header_present);
        assert_eq!(f0.data.config, PsEncoderConfig::default().ps_config());
        assert!(!f0.data.frame_class);
        assert!(f0.data.num_env >= 1);
        assert!(!f0.data.iid_dt[0], "header frame env 0 must be freq-coded");
        let last = f0.indices.iid.last().unwrap();
        assert!(last.iter().all(|&i| i == 7), "{last:?}");
        assert!(f0.indices.icc.last().unwrap().iter().all(|&i| i == 0));
        // The payload reparses to the same element.
        let mut rd = BitReader::new(&f0.payload);
        let back = PsData::parse(&mut rd, None).unwrap().unwrap();
        assert_eq!(back, f0.data);
        // Same content again: hold.
        let f1 = enc.encode_frame(&l, &r).unwrap();
        assert!(!f1.header_sent);
        assert_eq!(f1.data.num_env, 0);
        assert_eq!(f1.bits, 4);
        assert!(f1.indices.iid.is_empty());
    }

    /// Every band configuration and both grids: the decoder-side
    /// resolution of the written payload reproduces the encoder's
    /// indices across a run of frames with changing cues (so both
    /// coding directions get exercised), including the phase layer.
    #[test]
    fn closed_loop_across_configurations() {
        for bands in [PsBands::Ten, PsBands::Twenty, PsBands::ThirtyFour] {
            for (fine, phase) in [(false, false), (true, false), (false, true), (true, true)] {
                let cfg = PsEncoderConfig {
                    bands,
                    fine_iid: fine,
                    phase,
                    header_interval: 3,
                    ..PsEncoderConfig::default()
                };
                let mut enc = PsEncoder::new(cfg).unwrap();
                let mut dec_cfg = None;
                let mut dec_state = PsIndexState::default();
                let mut saw_time = false;
                for f in 0..7u64 {
                    let a = noise(100 + f);
                    let b = noise(200 + f);
                    let g = 0.2 + 0.15 * f as f64;
                    let rot = Complex::new((0.4 * f as f64).cos(), (0.4 * f as f64).sin());
                    let l = frame(|n, k| a(n, k) + b(n, k) * 0.3);
                    let r = frame(|n, k| (a(n, k) * g + b(n, k) * 0.7) * rot);
                    let fr = enc.encode_frame(&l, &r).unwrap();
                    assert_eq!(fr.header_sent, f % 3 == 0);
                    let mut rd = BitReader::new(&fr.payload);
                    let back = PsData::parse(&mut rd, dec_cfg.as_ref()).unwrap().unwrap();
                    dec_cfg = Some(back.config);
                    assert_eq!(back, fr.data);
                    let idx = back.resolve(&mut dec_state).unwrap();
                    assert_eq!(
                        idx, fr.indices,
                        "{bands:?} fine={fine} phase={phase} frame {f}"
                    );
                    saw_time |= fr.data.iid_dt.iter().any(|&d| d);
                    assert_eq!(fr.data.enable_ipdopd, phase);
                    if phase && fr.data.num_env > 0 {
                        assert_eq!(fr.indices.ipd[0].len(), cfg.ps_config().nr_ipdopd_par());
                    }
                }
                assert!(saw_time, "{bands:?}: time coding never elected");
            }
        }
    }

    /// An attack mid-frame whose stereo image flips elects
    /// VAR_BORDERS at `[t−1, t, 31]`, with the pre- and post-attack
    /// cues on either side.
    #[test]
    fn cue_flipping_attack_elects_variable_borders() {
        let mut enc = PsEncoder::new(PsEncoderConfig::default()).unwrap();
        let sig = noise(77);
        let t = 20usize;
        // Quiet left-only, then a loud right-only burst from slot t.
        let l = frame(|n, k| {
            if n < t {
                sig(n, k) * 0.05
            } else {
                Complex::default()
            }
        });
        let r = frame(|n, k| if n < t { Complex::default() } else { sig(n, k) });
        let fr = enc.encode_frame(&l, &r).unwrap();
        assert_eq!(fr.transient, Some(t), "{:?}", fr.borders);
        assert!(fr.data.frame_class);
        assert_eq!(fr.borders, vec![t - 1, t, NUM_QMF_SLOTS - 1]);
        assert_eq!(fr.data.border_position, vec![(t - 1) as u8, t as u8, 31]);
        assert_eq!(fr.data.num_env, 3);
        // The pre-attack region is hard left in the unsplit bands;
        // the split low bands see the burst's 6-slot filter
        // pre-ringing (the hybrid bank's time resolution), so only
        // b ≥ 8 (hybrid channels ≥ 10) are pinned.
        assert!(
            fr.indices.iid[0][8..].iter().all(|&i| i == 7),
            "{:?}",
            fr.indices.iid[0]
        );
        assert!(fr.indices.iid[1].iter().all(|&i| i == -7));
        assert_eq!(fr.indices.iid[1], fr.indices.iid[2]);
        // The hold envelope is all-zero time deltas → time-coded.
        assert!(fr.data.iid_dt[2]);
        // With variable borders disabled the same frame is FIX.
        let mut enc = PsEncoder::new(PsEncoderConfig {
            variable_borders: false,
            ..PsEncoderConfig::default()
        })
        .unwrap();
        let fr = enc.encode_frame(&l, &r).unwrap();
        assert!(!fr.data.frame_class);
        assert!(fr.transient.is_none());
    }

    /// An attack that does not move the stereo image stays on fixed
    /// borders (the level jump is the core/SBR encoder's business).
    #[test]
    fn level_only_attack_keeps_fixed_borders() {
        let mut enc = PsEncoder::new(PsEncoderConfig::default()).unwrap();
        let sig = noise(78);
        let l = frame(|n, k| if n < 16 { sig(n, k) * 0.05 } else { sig(n, k) });
        let r = frame(|n, k| {
            if n < 16 {
                sig(n, k) * 0.025
            } else {
                sig(n, k) * 0.5
            }
        });
        let fr = enc.encode_frame(&l, &r).unwrap();
        assert!(fr.transient.is_none());
        assert!(!fr.data.frame_class);
        assert_eq!(fr.data.num_env, 1);
    }

    /// A short look-ahead is rejected.
    #[test]
    fn short_input_rejected() {
        let mut enc = PsEncoder::new(PsEncoderConfig::default()).unwrap();
        let x = vec![[Complex::default(); 64]; NUM_QMF_SLOTS];
        assert!(enc.encode_frame(&x, &x).is_err());
    }
}

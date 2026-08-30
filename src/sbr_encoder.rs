//! SBR encoder — the parameter-estimation half of the HE-AAC v1
//! encoder, ISO/IEC 14496-3 Annex 4.B.18 (informative encoder
//! description) written forward against the normative §4.6.18 decoder
//! this crate already carries.
//!
//! One [`SbrEncoder`] serves one core channel element (an SCE or a
//! CPE) and turns the 64-band analysis of the full-rate input
//! ([`crate::sbr_qmf::EncoderAnalysisQmf`]) into one
//! `sbr_extension_data()` fill payload per core frame:
//!
//! 1. **Time / frequency grid** (§4.B.18.3) — a transient detector over
//!    the SBR-range energy per QMF slot elects the frame class: a
//!    stationary frame is one `FIXFIX` envelope, a slowly varying one
//!    two, and an attack inside the frame becomes a `FIXVAR` /
//!    `VARFIX` / `VARVAR` grid whose variable border lands on the
//!    attack slot (with the border constraints of §4.6.18.3.3 —
//!    strictly increasing borders inside `[0, numTimeSlots + 8]`, and
//!    the leading border of the next frame meeting this frame's
//!    trailing one). High frequency resolution is used for ≤ 2
//!    envelopes, low for more.
//! 2. **Envelope estimation** (§4.B.18.4) — the mean of `|X(k,l)|²`
//!    over each envelope's time span and frequency band, on the band
//!    table of the envelope's resolution.
//! 3. **Noise floor and inverse filtering** (§4.B.18.5) — a tonality
//!    measure (second-order forward-prediction gain per subband, the
//!    same covariance formulation the decoder's HF generator uses)
//!    compares the original high band with the low band the decoder's
//!    patching will copy there. The ratio of the two tonalities picks
//!    the `bs_invf_mode` level; the noise-to-tonal energy the copy
//!    lacks after whitening becomes the noise floor `Q`. A strong tonal
//!    component that the patch cannot deliver can additionally be
//!    flagged as `bs_add_harmonic` (opt-in).
//! 4. **Quantisation** (§4.B.18.6) — `E_Q = INT(a·max(log2(E/64), 0)
//!    + 0.5)` (`a = 2` for 1.5 dB, `1` for 3 dB; a single-envelope
//!    FIXFIX frame is always 1.5 dB) and
//!    `Q_Q = INT(NOISE_FLOOR_OFFSET − log2(Q) + 0.5)`, clamped to
//!    `[0, 30]`.
//! 5. **Delta coding** (§4.B.18.7) — each envelope / noise floor is
//!    coded in whichever direction (frequency, or time against the
//!    previous envelope with the `i(k)` resolution remap) measures
//!    fewer Huffman bits, closed-loop: deltas are clamped to the
//!    codebook LAV and the *reconstructed* values (exactly what the
//!    decoder's §4.6.18.3.5 reconstruction yields) feed the next
//!    reference. The first envelope and noise floor of a frame that
//!    transmits `sbr_header()` are always frequency-coded, so a decoder
//!    joining at any header frame (a §4.6.18.3.3 reset) decodes
//!    correctly.
//! 6. **Bitstream** — [`crate::sbr_writer::build_extension_payload`]
//!    serialises the result with a header every
//!    [`SbrEncoderConfig::header_interval`] frames.
//!
//! ## Slot alignment
//!
//! The decoder assembles `XLow` from `tHFGen = 8` slots of the previous
//! core block followed by the 32 slots of the current one, and its
//! output frame reads columns `tHFAdj = 2 ..` of that buffer (§4.6.18
//! "Synchronization and timing"): envelope `l` therefore covers
//! `XLow` columns `[RATE·tE(l) + tHFAdj, RATE·tE(l+1) + tHFAdj)`. The
//! caller hands [`SbrEncoder::encode_frame`] exactly
//! [`SBR_ENC_COLS`] analysis columns per channel in that same
//! coordinate system: column `c` is core slot `c − 8` of the core
//! block the SBR payload rides with (a negative slot is the tail of
//! the previous block). [`crate::he_aac_encoder`] keeps the running
//! analysis and slices this window per frame.

use crate::raw_data_block::IdSynEle;
use crate::sbr_element::{SbrChannel, SbrElement};
use crate::sbr_envelope::{SbrEnvelopeData, SbrNoiseData};
use crate::sbr_freq_bands::{k0, k2, master_table, HiLoTables};
use crate::sbr_grid::{FrameClass, SbrDtdf, SbrGrid, SbrInvf};
use crate::sbr_header::SbrHeader;
use crate::sbr_hf_gen::{build_patches, Patches};
use crate::sbr_huffman::{env_tables, noise_tables, SbrHuffContext};
use crate::sbr_limiter::limiter_table;
use crate::sbr_qmf::Complex;
use crate::sbr_reconstruct::{ref_band, EnvelopeScalefactors, NoiseScalefactors};
use crate::sbr_time_grid::derive_time_grid;
use crate::sbr_writer::build_extension_payload;
use crate::{Error, Result};

/// `numTimeSlots` for the 1024-sample core frame (§4.6.18.2.6).
pub const NUM_TIME_SLOTS: usize = 16;
/// `RATE = 2` — QMF columns per SBR time slot.
pub const RATE: usize = 2;
/// `tHFAdj = 2` — the envelope adjuster's column offset into `XLow`.
pub const T_HF_ADJ: usize = 2;
/// `tHFGen = 8` — the slots of the previous core block that lead the
/// decoder's `XLow` buffer.
pub const T_HF_GEN: usize = 8;
/// Analysis columns [`SbrEncoder::encode_frame`] consumes per channel:
/// the furthest trailing border `tE(LE) = numTimeSlots + 8` maps to
/// column `RATE·24 + tHFAdj = 50`.
pub const SBR_ENC_COLS: usize = RATE * (NUM_TIME_SLOTS + 8) + T_HF_ADJ;

/// `NOISE_FLOOR_OFFSET = 6` (§4.6.18.2.5).
const NOISE_FLOOR_OFFSET: f64 = 6.0;

/// Configuration of one [`SbrEncoder`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SbrEncoderConfig {
    /// The SBR internal rate (the output rate; twice the core rate).
    /// Must be one of the §4.6.18.3.2.1 tabulated rates.
    pub fs_sbr: u32,
    /// `1` (SCE) or `2` (CPE).
    pub channels: usize,
    /// `bs_start_freq` (0..=15) — see [`pick_start_freq`].
    pub start_freq: u8,
    /// `bs_stop_freq` (0..=15) — see [`pick_stop_freq`].
    pub stop_freq: u8,
    /// `bs_xover_band` — the master-table band where the SBR range
    /// begins (`0` = at `k0`).
    pub xover_band: u8,
    /// `bs_freq_scale` (0..=3, Table 4.105).
    pub freq_scale: u8,
    /// `bs_alter_scale` (Table 4.106).
    pub alter_scale: bool,
    /// `bs_noise_bands` (0..=3, Table 4.107).
    pub noise_bands: u8,
    /// `bs_amp_res`: `true` = 3.0 dB envelope steps, `false` = 1.5 dB.
    pub amp_res: bool,
    /// Emit `EXT_SBR_DATA_CRC` payloads (else plain `EXT_SBR_DATA`).
    pub crc: bool,
    /// Transmit `sbr_header()` in every N-th frame (and always in the
    /// first). `0` behaves as `1` (every frame).
    pub header_interval: u32,
    /// Flag missing strong tonal components as `bs_add_harmonic`
    /// (§4.B.18.5 last paragraph). Off by default: the decoder's
    /// sinusoid is a plain tone at the band centre and is only ever an
    /// improvement on sparse, strongly harmonic material.
    pub add_harmonic: bool,
    /// Allow the variable frame classes (FIXVAR / VARFIX / VARVAR) on
    /// transients; when `false` a transient frame is a four-envelope
    /// FIXFIX grid instead.
    pub variable_borders: bool,
    /// `bs_coupling` for a channel pair (§4.B.18.4 / §4.B.18.6 joint
    /// level + balance coding). Ignored for a single channel.
    pub coupling: bool,
    /// `bs_interpol_freq` (Table 4.110): `true` maps the envelope
    /// energy onto every QMF subband of a band and lets the decoder's
    /// §4.6.18.7.5 limiter cap each subband's gain at 3 dB above the
    /// limiter-band average; `false` matches the *band* energy with
    /// one gain per band, preserving the copied source's shape inside
    /// it. The second reproduces band energies exactly even when the
    /// patch source is far more peaked than the original (a tonal low
    /// band under a dense high band), where the per-subband gains
    /// would be limited away.
    pub interpol_freq: bool,
    /// `bs_limiter_gains` (Table 4.109): 0 = −3 dB, 1 = 0 dB,
    /// 2 = 3 dB, 3 = no limit.
    pub limiter_gains: u8,
}

impl SbrEncoderConfig {
    /// A configuration for `fs_sbr` / `channels` with the SBR range
    /// spanning `[crossover_hz, stop_hz]` (rounded to the nearest
    /// reachable `bs_start_freq` / `bs_stop_freq`), 3 dB resolution,
    /// the Table 4.63 default band densities, a header every 8 frames,
    /// no CRC.
    pub fn new(fs_sbr: u32, channels: usize, crossover_hz: f64, stop_hz: f64) -> Result<Self> {
        if channels == 0 || channels > 2 {
            return Err(Error::EncoderInvalidConfig);
        }
        let start_freq = pick_start_freq(fs_sbr, crossover_hz)?;
        let stop_freq = pick_stop_freq(fs_sbr, start_freq, stop_hz)?;
        Ok(SbrEncoderConfig {
            fs_sbr,
            channels,
            start_freq,
            stop_freq,
            xover_band: 0,
            freq_scale: crate::sbr_header::DEFAULT_FREQ_SCALE,
            alter_scale: crate::sbr_header::DEFAULT_ALTER_SCALE,
            noise_bands: crate::sbr_header::DEFAULT_NOISE_BANDS,
            amp_res: true,
            crc: false,
            header_interval: 8,
            add_harmonic: true,
            variable_borders: true,
            coupling: false,
            interpol_freq: crate::sbr_header::DEFAULT_INTERPOL_FREQ,
            limiter_gains: crate::sbr_header::DEFAULT_LIMITER_GAINS,
        })
    }

    /// The `sbr_header()` this configuration transmits. The extra
    /// blocks are only emitted when a field departs from its Table
    /// 4.63 default; the limiter-band and smoothing fields stay at
    /// their defaults.
    pub fn header(&self) -> SbrHeader {
        let header_extra_1 = self.freq_scale != crate::sbr_header::DEFAULT_FREQ_SCALE
            || self.alter_scale != crate::sbr_header::DEFAULT_ALTER_SCALE
            || self.noise_bands != crate::sbr_header::DEFAULT_NOISE_BANDS;
        let header_extra_2 = self.interpol_freq != crate::sbr_header::DEFAULT_INTERPOL_FREQ
            || self.limiter_gains != crate::sbr_header::DEFAULT_LIMITER_GAINS;
        SbrHeader {
            amp_res: self.amp_res,
            start_freq: self.start_freq,
            stop_freq: self.stop_freq,
            xover_band: self.xover_band,
            reserved: 0,
            header_extra_1,
            header_extra_2,
            freq_scale: self.freq_scale,
            alter_scale: self.alter_scale,
            noise_bands: self.noise_bands,
            limiter_bands: crate::sbr_header::DEFAULT_LIMITER_BANDS,
            limiter_gains: self.limiter_gains.min(3),
            interpol_freq: self.interpol_freq,
            smoothing_mode: crate::sbr_header::DEFAULT_SMOOTHING_MODE,
        }
    }
}

/// The QMF subband width in Hz at the SBR rate: `fs_sbr / 128`.
fn band_hz(fs_sbr: u32) -> f64 {
    f64::from(fs_sbr) / 128.0
}

/// The `bs_start_freq` whose `k0` (§4.6.18.3.2.1) lies closest to
/// `crossover_hz`.
pub fn pick_start_freq(fs_sbr: u32, crossover_hz: f64) -> Result<u8> {
    let target = crossover_hz / band_hz(fs_sbr);
    let mut best = None::<(f64, u8)>;
    for idx in 0..16u8 {
        let k = f64::from(k0(fs_sbr, idx)?);
        let d = (k - target).abs();
        if best.map_or(true, |(bd, _)| d < bd) {
            best = Some((d, idx));
        }
    }
    best.map(|(_, i)| i).ok_or(Error::EncoderInvalidConfig)
}

/// The `bs_stop_freq` whose `k2` lies closest to `stop_hz` (never
/// below `k0 + 1`, and never above 64 subbands).
pub fn pick_stop_freq(fs_sbr: u32, start_freq: u8, stop_hz: f64) -> Result<u8> {
    let k0v = k0(fs_sbr, start_freq)?;
    let target = (stop_hz / band_hz(fs_sbr)).min(64.0);
    let mut best = None::<(f64, u8)>;
    for idx in 0..16u8 {
        let k = k2(fs_sbr, idx, k0v)?;
        if k <= k0v {
            continue;
        }
        let d = (f64::from(k) - target).abs();
        if best.map_or(true, |(bd, _)| d < bd) {
            best = Some((d, idx));
        }
    }
    best.map(|(_, i)| i).ok_or(Error::EncoderInvalidConfig)
}

/// Per-channel cross-frame coding state.
#[derive(Debug, Clone, Default)]
struct ChannelState {
    /// The decoder's reconstruction of the previous frame's envelopes
    /// (`E'_Q`, with `r'`) — the time-delta reference for `l = 0`.
    prev_env: Option<EnvelopeScalefactors>,
    /// The previous frame's reconstructed noise floors.
    prev_noise: Option<NoiseScalefactors>,
    /// `tE'(LE')` — the previous frame's trailing border (the next
    /// frame's leading border must meet it).
    t_e_last_prev: i32,
}

/// Per-frame diagnostics of one channel, returned alongside the
/// payload for tests and tuning.
#[derive(Debug, Clone, PartialEq)]
pub struct SbrChannelReport {
    /// The elected grid's envelope borders `tE(0..=LE)` in slots.
    pub t_e: Vec<i32>,
    /// Estimated linear envelope energies `E(k, l)` per envelope.
    pub energy: Vec<Vec<f64>>,
    /// The decoder-side reconstruction of the coded envelopes `E_Q`.
    pub eq: Vec<Vec<i32>>,
    /// The decoder-side reconstruction of the coded noise floors.
    pub qq: Vec<Vec<i32>>,
    /// Estimated linear noise-floor ratios `Q(k, l)`.
    pub q: Vec<Vec<f64>>,
    /// Whether a transient was detected in this frame.
    pub transient: bool,
}

/// One frame's output.
#[derive(Debug, Clone, PartialEq)]
pub struct SbrFrame {
    /// The complete `extension_payload()` bytes for a `fill_element()`
    /// following the core channel element.
    pub payload: Vec<u8>,
    /// Whether this payload carried `sbr_header()`.
    pub header_sent: bool,
    /// The element as the decoder will parse it.
    pub element: SbrElement,
    /// Per-channel diagnostics.
    pub reports: Vec<SbrChannelReport>,
}

/// The SBR parameter encoder for one core channel element.
#[derive(Debug, Clone)]
pub struct SbrEncoder {
    cfg: SbrEncoderConfig,
    header: SbrHeader,
    bands: HiLoTables,
    patches: Patches,
    /// `fTableLim` — the decoder's limiter bands for this header.
    f_table_lim: Vec<i32>,
    frames: u64,
    ch: Vec<ChannelState>,
}

/// Energy summary of one time span of one subband.
#[derive(Debug, Clone, Copy, Default)]
struct Tonality {
    /// Total energy `Σ|x|²`.
    total: f64,
    /// Second-order forward-prediction residual energy (the
    /// "noise-like" part).
    noise: f64,
}

impl Tonality {
    fn tonal(&self) -> f64 {
        (self.total - self.noise).max(0.0)
    }
}

/// Second-order covariance-method prediction over `x[c0..c1]`: the
/// residual energy of the best `x[n] ≈ −α0·x[n−1] − α1·x[n−2]`
/// predictor (the §4.6.18.6.2 normal equations, solved here for the
/// error energy rather than the coefficients).
fn tonality(x: &[[Complex; 64]], k: usize, c0: usize, c1: usize) -> Tonality {
    let total: f64 = (c0..c1).map(|c| x[c][k].norm_sqr()).sum();
    if c1 < c0 + 4 {
        // Too short for a meaningful two-tap fit: treat as noise.
        return Tonality {
            total,
            noise: total,
        };
    }
    let phi = |i: usize, j: usize| -> Complex {
        let mut acc = Complex::default();
        for n in (c0 + 2)..c1 {
            acc += x[n - i][k] * x[n - j][k].conj();
        }
        acc
    };
    let p11 = phi(1, 1).re;
    let p22 = phi(2, 2).re;
    let p12 = phi(1, 2);
    let p01 = phi(0, 1);
    let p02 = phi(0, 2);
    // One-tap fit (always well-posed for a non-zero signal); a pure
    // complex exponential is rank-one and only this fit resolves it.
    let mut err = f64::INFINITY;
    if p11 > 0.0 {
        let a0 = p01 * (-1.0 / p11);
        let mut e1 = 0.0;
        for n in (c0 + 2)..c1 {
            let r = x[n][k] + a0 * x[n - 1][k];
            e1 += r.norm_sqr();
        }
        err = e1;
    }
    // Two-tap fit — the normal equations of
    // Σ|x[n] + a0·x[n−1] + a1·x[n−2]|²:
    //   p11·a0 + p12*·a1 = −p01,   p12·a0 + p22·a1 = −p02.
    let d = p11 * p22 - p12.norm_sqr();
    if d > 1e-9 * (p11 * p22).max(1e-300) {
        let a0 = (p12.conj() * p02 - p01 * p22) * (1.0 / d);
        let a1 = (p12 * p01 - p02 * p11) * (1.0 / d);
        let mut e2 = 0.0;
        for n in (c0 + 2)..c1 {
            let r = x[n][k] + a0 * x[n - 1][k] + a1 * x[n - 2][k];
            e2 += r.norm_sqr();
        }
        err = err.min(e2);
    }
    if !err.is_finite() {
        return Tonality {
            total,
            noise: total,
        };
    }
    // The first two columns are not predicted; count them as noise so
    // a short span cannot look perfectly tonal.
    let head: f64 = (c0..c0 + 2).map(|c| x[c][k].norm_sqr()).sum();
    Tonality {
        total,
        noise: (err + head).min(total),
    }
}

/// Aggregate tonality over a subband range.
fn band_tonality(x: &[[Complex; 64]], k_lo: usize, k_hi: usize, c0: usize, c1: usize) -> Tonality {
    let mut acc = Tonality::default();
    for k in k_lo..k_hi {
        let t = tonality(x, k, c0, c1);
        acc.total += t.total;
        acc.noise += t.noise;
    }
    acc
}

impl SbrEncoder {
    /// Build an encoder; derives the band tables and the patch layout
    /// from the configuration's header exactly as the decoder will.
    pub fn new(cfg: SbrEncoderConfig) -> Result<Self> {
        if cfg.channels == 0 || cfg.channels > 2 {
            return Err(Error::EncoderInvalidConfig);
        }
        let header = cfg.header();
        let k0v = k0(cfg.fs_sbr, header.start_freq)?;
        let k2v = k2(cfg.fs_sbr, header.stop_freq, k0v)?;
        let f_master = master_table(k0v, k2v, header.freq_scale, header.alter_scale)?;
        let bands = HiLoTables::derive(&f_master, header.xover_band, header.noise_bands)?;
        let patches = build_patches(&f_master, k0v, bands.k_x, bands.m, cfg.fs_sbr)?;
        let f_table_lim = limiter_table(&bands, &patches.borders(bands.k_x), header.limiter_bands)?;
        Ok(SbrEncoder {
            cfg,
            header,
            bands,
            patches,
            f_table_lim,
            frames: 0,
            ch: vec![ChannelState::default(); cfg.channels],
        })
    }

    /// The transmitted header.
    pub fn header(&self) -> &SbrHeader {
        &self.header
    }

    /// The derived band tables.
    pub fn bands(&self) -> &HiLoTables {
        &self.bands
    }

    /// The configuration.
    pub fn config(&self) -> &SbrEncoderConfig {
        &self.cfg
    }

    /// The source subband the decoder's patching copies into high
    /// subband `k` (§4.6.18.6.3).
    fn source_band(&self, k: usize) -> usize {
        let borders = self.patches.borders(self.bands.k_x);
        for i in 0..self.patches.num_patches() {
            let lo = borders[i] as usize;
            let hi = borders[i + 1] as usize;
            if k >= lo && k < hi {
                return self.patches.start[i] + (k - lo);
            }
        }
        // Outside the patched range (never for k in [kx, kx+M)).
        k.min(31)
    }

    /// Encode one frame. `x[ch]` holds [`SBR_ENC_COLS`] analysis
    /// columns per channel (see the module docs for the alignment).
    pub fn encode_frame(&mut self, x: &[&[[Complex; 64]]]) -> Result<SbrFrame> {
        if x.len() != self.cfg.channels || x.iter().any(|c| c.len() < SBR_ENC_COLS) {
            return Err(Error::SbrQmfInvalid);
        }
        let interval = self.cfg.header_interval.max(1) as u64;
        let header_sent = self.frames % interval == 0;
        let reset = self.frames == 0;
        let id_aac = if self.cfg.channels == 1 {
            IdSynEle::Sce
        } else {
            IdSynEle::Cpe
        };

        let coupling = self.cfg.channels == 2 && self.cfg.coupling;
        let mut channels = Vec::with_capacity(self.cfg.channels);
        let mut reports = Vec::with_capacity(self.cfg.channels);
        if coupling {
            let (chs, reps) = self.encode_coupled(x[0], x[1], header_sent, reset)?;
            channels = chs;
            reports = reps;
        } else {
            for (c, xc) in x.iter().enumerate() {
                let (ch, rep) = self.encode_channel(c, xc, header_sent, reset)?;
                channels.push(ch);
                reports.push(rep);
            }
        }
        let element = SbrElement {
            coupling,
            channels,
            extension: None,
        };
        let payload = build_extension_payload(
            id_aac,
            header_sent.then_some(&self.header),
            &self.header,
            &element,
            &self.bands,
            self.cfg.crc,
        )?;
        self.frames += 1;
        Ok(SbrFrame {
            payload,
            header_sent,
            element,
            reports,
        })
    }

    /// Elect the frame grid for one channel from its analysis columns.
    fn elect_grid(&self, x: &[[Complex; 64]], t_e_last_prev: i32) -> (SbrGrid, bool) {
        let kx = self.bands.k_x as usize;
        let k_end = (self.bands.k_x + self.bands.m) as usize;
        // SBR-range energy per SBR time slot (RATE columns each) over
        // the nominal frame [tHFAdj, tHFAdj + 32) plus the lookahead
        // the variable trailing border can reach.
        let n_slots = NUM_TIME_SLOTS + 8;
        let mut e = vec![0.0f64; n_slots];
        for (s, es) in e.iter_mut().enumerate() {
            for r in 0..RATE {
                let c = T_HF_ADJ + RATE * s + r;
                *es += (kx..k_end).map(|k| x[c][k].norm_sqr()).sum::<f64>();
            }
        }
        let peak = e.iter().cloned().fold(0.0f64, f64::max);
        // Attack: a slot at least 8× the mean of the preceding four
        // and carrying a non-negligible share of the frame's peak.
        let mut attack: Option<usize> = None;
        for s in 4..NUM_TIME_SLOTS {
            let prev = e[s - 4..s].iter().sum::<f64>() / 4.0;
            if e[s] > 8.0 * prev + 1e-9 && e[s] > 0.1 * peak && e[s] > 64.0 {
                attack = Some(s);
                break;
            }
        }
        let transient = attack.is_some();

        // The leading border must meet the previous trailing one; a
        // previous VAR trailing border beyond numTimeSlots becomes
        // this frame's bs_var_bord_0.
        let lead = (t_e_last_prev - NUM_TIME_SLOTS as i32).clamp(0, 3) as u8;

        let grid = match attack {
            Some(s) if self.cfg.variable_borders => variable_grid(s, lead),
            Some(_) if lead == 0 => fixfix_grid(4, false),
            Some(_) => varfix_plain(lead, 4),
            None => {
                // Stationary vs slowly varying: compare the two halves.
                let first: f64 = e[..NUM_TIME_SLOTS / 2].iter().sum();
                let second: f64 = e[NUM_TIME_SLOTS / 2..NUM_TIME_SLOTS].iter().sum();
                let ratio = (first + 1.0) / (second + 1.0);
                let n = if (0.5..=2.0).contains(&ratio) { 1 } else { 2 };
                if lead == 0 {
                    fixfix_grid(n, true)
                } else {
                    varfix_plain(lead, n)
                }
            }
        };
        // A grid the decoder would reject falls back to the safe one.
        match derive_time_grid(&grid, NUM_TIME_SLOTS as i32) {
            Ok(_) => (grid, transient),
            Err(_) => (
                fixfix_grid(if transient { 4 } else { 1 }, !transient),
                transient,
            ),
        }
    }

    /// Envelope energies (§4.B.18.4) for `grid` over `x`.
    fn estimate_envelopes(
        &self,
        x: &[[Complex; 64]],
        grid: &SbrGrid,
        t_e: &[i32],
    ) -> Vec<Vec<f64>> {
        (0..grid.num_env)
            .map(|l| {
                let table = if grid.freq_res[l] {
                    &self.bands.f_table_high
                } else {
                    &self.bands.f_table_low
                };
                let c0 = RATE * t_e[l] as usize + T_HF_ADJ;
                let c1 = RATE * t_e[l + 1] as usize + T_HF_ADJ;
                table
                    .windows(2)
                    .map(|w| {
                        let (kl, kh) = (w[0] as usize, w[1] as usize);
                        let acc: f64 = x[c0..c1]
                            .iter()
                            .map(|col| col[kl..kh].iter().map(|v| v.norm_sqr()).sum::<f64>())
                            .sum();
                        acc / ((c1 - c0) * (kh - kl)) as f64
                    })
                    .collect()
            })
            .collect()
    }

    /// Analysis-by-synthesis of the decoder's §4.6.18.7.5 limiter over
    /// columns `[c0, c1)`: per noise band, the fraction of the target
    /// energy that the copied source cannot deliver once each
    /// subband's gain is capped at `limGain` above its limiter band's
    /// average gain (`G_max = √(ΣE_orig / ΣE_curr) · limGain`).
    ///
    /// `E_orig` is taken as the decoder will map it — the envelope
    /// band mean spread over the band's subbands (`bs_interpol_freq =
    /// 1`), or the same value with one gain per envelope band
    /// (`bs_interpol_freq = 0`, where only the band's total matters);
    /// `E_curr` is the energy of each subband's patch source.
    fn limiter_deficit(
        &self,
        x: &[[Complex; 64]],
        c0: usize,
        c1: usize,
        harm: &[bool],
    ) -> Vec<f64> {
        let kx = self.bands.k_x as usize;
        let k_end = (self.bands.k_x + self.bands.m) as usize;
        let span = (c1 - c0).max(1) as f64;
        let energy =
            |k: usize| -> f64 { x[c0..c1].iter().map(|col| col[k].norm_sqr()).sum::<f64>() / span };
        // Per-subband target (envelope-band mean) and source energies.
        let mut e_orig = vec![0.0f64; 64];
        let mut e_curr = vec![0.0f64; 64];
        for w in self.bands.f_table_high.windows(2) {
            let (lo, hi) = (w[0] as usize, w[1] as usize);
            let mean = (lo..hi).map(energy).sum::<f64>() / (hi - lo) as f64;
            let src_mean =
                (lo..hi).map(|k| energy(self.source_band(k))).sum::<f64>() / (hi - lo) as f64;
            for k in lo..hi {
                e_orig[k] = mean;
                e_curr[k] = if self.cfg.interpol_freq {
                    energy(self.source_band(k))
                } else {
                    src_mean
                };
            }
        }
        // Delivered energy per subband under the limiter cap.
        let lim_gain_sq =
            crate::sbr_env_adjust::LIM_GAIN[usize::from(self.header.limiter_gains)].powi(2);
        let mut delivered = vec![0.0f64; 64];
        for w in self.f_table_lim.windows(2) {
            let lo = (w[0] as usize).max(kx);
            let hi = (w[1] as usize).min(k_end);
            if hi <= lo {
                continue;
            }
            let num: f64 = 1e-12 + e_orig[lo..hi].iter().sum::<f64>();
            let den: f64 = 1e-12 + e_curr[lo..hi].iter().sum::<f64>();
            let g_max_sq = (num / den) * lim_gain_sq;
            for k in lo..hi {
                let g_sq = e_orig[k] / (1e-12 + e_curr[k]);
                delivered[k] = e_curr[k] * g_sq.min(g_max_sq);
            }
        }
        // A band carrying a bs_add_harmonic sinusoid is delivered
        // coherently — it contributes no deficit.
        if !harm.is_empty() {
            for (p, &flag) in harm.iter().enumerate() {
                if flag {
                    let lo = self.bands.f_table_high[p] as usize;
                    let hi = self.bands.f_table_high[p + 1] as usize;
                    delivered[lo..hi].copy_from_slice(&e_orig[lo..hi]);
                }
            }
        }
        self.bands
            .f_table_noise
            .windows(2)
            .map(|w| {
                let (lo, hi) = (w[0] as usize, w[1] as usize);
                let target: f64 = e_orig[lo..hi].iter().sum();
                let got: f64 = delivered[lo..hi].iter().sum();
                if target <= 0.0 {
                    0.0
                } else {
                    (1.0 - got / target).max(0.0)
                }
            })
            .collect()
    }

    /// Noise floors `Q`, inverse-filtering modes and add-harmonic
    /// flags (§4.B.18.5) for the noise-floor spans `t_q`.
    fn estimate_noise(
        &self,
        x: &[[Complex; 64]],
        t_q: &[i32],
        t_e: &[i32],
        freq_res: &[bool],
    ) -> (Vec<Vec<f64>>, Vec<u8>, Vec<bool>) {
        let nq = self.bands.n_q();
        let mut q = Vec::with_capacity(t_q.len() - 1);
        let mut invf = vec![0u8; nq];
        // Tonality of original vs. patched source per noise band,
        // measured over the whole frame span (extended 6 columns back
        // for a stable two-tap fit) — the invf mode is per frame.
        let frame_c0 = (RATE * t_e[0] as usize + T_HF_ADJ).saturating_sub(6);
        let frame_c1 = RATE * t_e[t_e.len() - 1] as usize + T_HF_ADJ;
        let mut band_ratio = vec![(0.0f64, 0.0f64); nq];
        for n in 0..nq {
            let kl = self.bands.f_table_noise[n] as usize;
            let kh = self.bands.f_table_noise[n + 1] as usize;
            let orig = band_tonality(x, kl, kh, frame_c0, frame_c1);
            let mut src = Tonality::default();
            for k in kl..kh {
                let t = tonality(x, self.source_band(k), frame_c0, frame_c1);
                src.total += t.total;
                src.noise += t.noise;
            }
            let eps = 1e-9;
            let r_o = orig.noise / (orig.tonal() + eps * orig.total.max(1.0));
            let r_s = src.noise / (src.tonal() + eps * src.total.max(1.0));
            band_ratio[n] = (r_o, r_s);
            // Tonality-ratio in dB: how much more tonal the copy is
            // than the original decides the whitening level.
            let db = 10.0 * ((r_o + 1e-6) / (r_s + 1e-6)).log10();
            invf[n] = if orig.total <= 64.0 * (kh - kl) as f64 || db < 3.0 {
                0
            } else if db < 9.0 {
                1
            } else if db < 15.0 {
                2
            } else {
                3
            };
        }
        // §4.B.18.5 last paragraph: a strong tonal component the HF
        // generator cannot deliver is coded as a sinusoid
        // (bs_add_harmonic) — the coherent injection reproduces a
        // peaked band without the broadband leak a huge noise floor
        // would spray into its neighbours. Detector: the original
        // band is tonal-dominant, clearly peaked above the frame's
        // median band level, and its patch source carries no
        // comparable tone. Only expressible when the frame's last
        // envelope uses the high-resolution table (the flags index
        // NHigh bands).
        let mut harm = Vec::new();
        if self.cfg.add_harmonic && *freq_res.last().unwrap_or(&false) {
            let n_high = self.bands.n_high();
            let mut level: Vec<f64> = Vec::with_capacity(n_high);
            let mut tone: Vec<(f64, f64)> = Vec::with_capacity(n_high);
            for p in 0..n_high {
                let kl = self.bands.f_table_high[p] as usize;
                let kh = self.bands.f_table_high[p + 1] as usize;
                let orig = band_tonality(x, kl, kh, frame_c0, frame_c1);
                let mut src = Tonality::default();
                for k in kl..kh {
                    let t = tonality(x, self.source_band(k), frame_c0, frame_c1);
                    src.total += t.total;
                    src.noise += t.noise;
                }
                level.push(orig.total / (kh - kl) as f64);
                tone.push((
                    orig.tonal() / (orig.noise + 1e-9),
                    src.tonal() / (src.noise + 1e-9),
                ));
            }
            let mut sorted = level.clone();
            sorted.sort_by(f64::total_cmp);
            let median = sorted[sorted.len() / 2];
            harm = (0..n_high)
                .map(|p| {
                    let kl = self.bands.f_table_high[p] as usize;
                    let kh = self.bands.f_table_high[p + 1] as usize;
                    let (o_ratio, s_ratio) = tone[p];
                    level[p] > 64.0 * (kh - kl) as f64
                        && level[p] > 4.0 * median
                        && o_ratio > 5.0
                        && s_ratio < o_ratio / 2.0
                })
                .collect();
            if !harm.iter().any(|&f| f) {
                harm.clear();
            }
        }
        for l in 0..t_q.len() - 1 {
            let c0 = RATE * t_q[l] as usize + T_HF_ADJ;
            let c1 = RATE * t_q[l + 1] as usize + T_HF_ADJ;
            let deficit = self.limiter_deficit(x, c0, c1, &harm);
            let mut row = Vec::with_capacity(nq);
            for n in 0..nq {
                let kl = self.bands.f_table_noise[n] as usize;
                let kh = self.bands.f_table_noise[n + 1] as usize;
                let (r_o, r_s) = band_ratio[n];
                // Whitening raises the copy's noise-to-tonal ratio;
                // model the four levels as ×1 / ×2 / ×4 / ×8.
                let r_s_eff = r_s * f64::from(1u32 << invf[n]);
                // Within this floor's span, scale the frame-wide
                // ratio by the span's own noisiness so a quiet tail
                // after an attack does not inherit the attack's mix.
                let span = band_tonality(x, kl, kh, c0.saturating_sub(4), c1);
                let r_span = span.noise / (span.tonal() + 1e-9 * span.total.max(1.0));
                let r_o = if span.total > 0.0 {
                    r_o.min(r_span.max(r_o * 0.25))
                } else {
                    r_o
                };
                let q_ton = if r_o > r_s_eff {
                    (r_o - r_s_eff) / (1.0 + r_s_eff)
                } else {
                    0.0
                };
                // Energy the decoder's limiter will refuse to deliver
                // through the copied source must come from the noise
                // floor instead: noise fraction Q/(1+Q) ≥ deficit.
                let d = deficit[n].clamp(0.0, 0.98);
                let q_def = d / (1.0 - d);
                row.push(q_ton.max(q_def).clamp(2f64.powi(-24), 64.0));
            }
            q.push(row);
        }
        (q, invf, harm)
    }

    /// Estimate, quantise and delta-code one uncoupled channel.
    fn encode_channel(
        &mut self,
        c: usize,
        x: &[[Complex; 64]],
        header_sent: bool,
        reset: bool,
    ) -> Result<(SbrChannel, SbrChannelReport)> {
        let (grid, transient) = self.elect_grid(x, self.ch[c].t_e_last_prev);
        let tg = derive_time_grid(&grid, NUM_TIME_SLOTS as i32)?;
        let eff_amp = self.header.amp_res && !grid.amp_res_override;
        let energy = self.estimate_envelopes(x, &grid, &tg.t_e);
        let (q, invf_mode, harm) = self.estimate_noise(x, &tg.t_q, &tg.t_e, &grid.freq_res);

        let qq_target = quantise_noise(&q);
        let mut comp_energy = energy.clone();
        compensate_noise_loss(
            &mut comp_energy,
            &qq_target,
            &tg.t_e,
            &tg.t_q,
            &grid.freq_res,
            &self.bands,
        );
        let eq_target = quantise_envelopes(&comp_energy, eff_amp);

        let prev_env = if reset {
            None
        } else {
            self.ch[c].prev_env.as_ref()
        };
        let prev_noise = if reset {
            None
        } else {
            self.ch[c].prev_noise.as_ref()
        };
        let force_freq_first = header_sent || reset;
        let ctx = SbrHuffContext {
            coupling: false,
            ch: c == 1,
            amp_res: eff_amp,
        };
        let (env_data, df_env) = code_envelopes(
            &eq_target,
            &grid,
            &self.bands,
            ctx,
            prev_env,
            force_freq_first,
            1,
        );
        let (noise_data, df_noise) =
            code_noise(&qq_target, &grid, ctx, prev_noise, force_freq_first, 1);
        let dtdf = SbrDtdf { df_env, df_noise };
        let envelope = SbrEnvelopeData { data: env_data };
        let noise = SbrNoiseData { data: noise_data };

        // Decoder-side reconstruction becomes the next reference.
        let rec_env = EnvelopeScalefactors::reconstruct(
            &envelope,
            &grid,
            &dtdf,
            &self.bands,
            false,
            c == 1,
            prev_env,
        )?;
        let rec_noise = NoiseScalefactors::reconstruct(
            &noise,
            &grid,
            &dtdf,
            self.bands.n_q(),
            false,
            c == 1,
            prev_noise,
        )?;
        let report = SbrChannelReport {
            t_e: tg.t_e.clone(),
            energy,
            eq: rec_env.eq.clone(),
            qq: rec_noise.q.clone(),
            q,
            transient,
        };
        let st = &mut self.ch[c];
        st.prev_env = Some(rec_env);
        st.prev_noise = Some(rec_noise);
        st.t_e_last_prev = tg.t_e[tg.t_e.len() - 1];
        Ok((
            SbrChannel {
                grid,
                dtdf,
                invf: SbrInvf { invf_mode },
                envelope,
                noise,
                add_harmonic: harm,
            },
            report,
        ))
    }

    /// Estimate, quantise and delta-code a coupled channel pair
    /// (§4.B.18.4 / §4.B.18.6 level + balance): one shared grid from
    /// the summed energy, channel 0 carrying the pair average and
    /// channel 1 the left/right ratio quantised to even values around
    /// `panOffset`.
    fn encode_coupled(
        &mut self,
        xl: &[[Complex; 64]],
        xr: &[[Complex; 64]],
        header_sent: bool,
        reset: bool,
    ) -> Result<(Vec<SbrChannel>, Vec<SbrChannelReport>)> {
        // Shared grid from the pair sum.
        let sum: Vec<[Complex; 64]> = xl
            .iter()
            .zip(xr.iter())
            .map(|(a, b)| {
                let mut s = [Complex::default(); 64];
                for k in 0..64 {
                    s[k] = a[k] + b[k];
                }
                s
            })
            .collect();
        let (grid, transient) = self.elect_grid(&sum, self.ch[0].t_e_last_prev);
        let tg = derive_time_grid(&grid, NUM_TIME_SLOTS as i32)?;
        let eff_amp = self.header.amp_res && !grid.amp_res_override;
        let e_l = self.estimate_envelopes(xl, &grid, &tg.t_e);
        let e_r = self.estimate_envelopes(xr, &grid, &tg.t_e);
        let (q_l, invf_l, harm_l) = self.estimate_noise(xl, &tg.t_q, &tg.t_e, &grid.freq_res);
        let (q_r, invf_r, harm_r) = self.estimate_noise(xr, &tg.t_q, &tg.t_e, &grid.freq_res);
        // One invf vector is transmitted for the pair: the stronger
        // whitening of the two.
        let invf_mode: Vec<u8> = invf_l
            .iter()
            .zip(invf_r.iter())
            .map(|(&a, &b)| a.max(b))
            .collect();

        // Level = (L + R) / 2 (§4.B.18.4 E_Left), balance = L / R.
        let a = if eff_amp { 1.0 } else { 2.0 };
        let pan_e = crate::sbr_dequant::pan_offset(eff_amp) as i32;
        let eps = 1e-9;
        let mut e_level = Vec::with_capacity(e_l.len());
        let mut e_bal = Vec::with_capacity(e_l.len());
        for (rl, rr) in e_l.iter().zip(e_r.iter()) {
            let mut lv = Vec::with_capacity(rl.len());
            let mut bv = Vec::with_capacity(rl.len());
            for (&l, &r) in rl.iter().zip(rr.iter()) {
                lv.push((l + r) / 2.0);
                let ratio = (eps + l) / (eps + r);
                let raw = (a * ratio.log2() + 0.5).floor() as i32 + pan_e;
                // Even values only, within [0, 2·panOffset].
                let even = ((raw + 1) / 2 * 2).clamp(0, 2 * pan_e);
                bv.push(even);
            }
            e_level.push(lv);
            e_bal.push(bv);
        }
        let pan_q = crate::sbr_dequant::pan_offset(true) as i32; // panOffset(1) = 12
        let mut q_level = Vec::with_capacity(q_l.len());
        let mut q_bal = Vec::with_capacity(q_l.len());
        for (rl, rr) in q_l.iter().zip(q_r.iter()) {
            let mut lv = Vec::with_capacity(rl.len());
            let mut bv = Vec::with_capacity(rl.len());
            for (&l, &r) in rl.iter().zip(rr.iter()) {
                lv.push((l + r) / 2.0);
                let raw = ((l / r).log2() + 0.5).floor() as i32 + pan_q;
                bv.push(((raw + 1) / 2 * 2).clamp(0, 2 * pan_q));
            }
            q_level.push(lv);
            q_bal.push(bv);
        }
        let qq_level = quantise_noise(&q_level);
        let mut comp_level = e_level.clone();
        compensate_noise_loss(
            &mut comp_level,
            &qq_level,
            &tg.t_e,
            &tg.t_q,
            &grid.freq_res,
            &self.bands,
        );
        let eq_level = quantise_envelopes(&comp_level, eff_amp);

        let force_freq_first = header_sent || reset;
        let mut out_ch = Vec::with_capacity(2);
        let mut reports = Vec::with_capacity(2);
        for (c, (eq_t, qq_t)) in [(eq_level, qq_level), (e_bal, q_bal)]
            .into_iter()
            .enumerate()
        {
            let prev_env = if reset {
                None
            } else {
                self.ch[c].prev_env.as_ref()
            };
            let prev_noise = if reset {
                None
            } else {
                self.ch[c].prev_noise.as_ref()
            };
            let ctx = SbrHuffContext {
                coupling: true,
                ch: c == 1,
                amp_res: eff_amp,
            };
            // The balance channel's deltas are halved on the wire
            // (δ = 0.5) and doubled back by the decoder.
            let step = if c == 1 { 2 } else { 1 };
            let (env_data, df_env) = code_envelopes(
                &eq_t,
                &grid,
                &self.bands,
                ctx,
                prev_env,
                force_freq_first,
                step,
            );
            let (noise_data, df_noise) =
                code_noise(&qq_t, &grid, ctx, prev_noise, force_freq_first, step);
            let dtdf = SbrDtdf { df_env, df_noise };
            let envelope = SbrEnvelopeData { data: env_data };
            let noise = SbrNoiseData { data: noise_data };
            let rec_env = EnvelopeScalefactors::reconstruct(
                &envelope,
                &grid,
                &dtdf,
                &self.bands,
                true,
                c == 1,
                prev_env,
            )?;
            let rec_noise = NoiseScalefactors::reconstruct(
                &noise,
                &grid,
                &dtdf,
                self.bands.n_q(),
                true,
                c == 1,
                prev_noise,
            )?;
            reports.push(SbrChannelReport {
                t_e: tg.t_e.clone(),
                energy: if c == 0 { e_l.clone() } else { e_r.clone() },
                eq: rec_env.eq.clone(),
                qq: rec_noise.q.clone(),
                q: if c == 0 { q_l.clone() } else { q_r.clone() },
                transient,
            });
            let st = &mut self.ch[c];
            st.prev_env = Some(rec_env);
            st.prev_noise = Some(rec_noise);
            st.t_e_last_prev = tg.t_e[tg.t_e.len() - 1];
            out_ch.push(SbrChannel {
                grid: grid.clone(),
                dtdf,
                invf: SbrInvf {
                    invf_mode: if c == 0 {
                        invf_mode.clone()
                    } else {
                        Vec::new()
                    },
                },
                envelope,
                noise,
                add_harmonic: if c == 0 {
                    harm_l.clone()
                } else {
                    harm_r.clone()
                },
            });
        }
        Ok((out_ch, reports))
    }
}

/// A FIXFIX grid with `num_env` envelopes at one resolution.
fn fixfix_grid(num_env: usize, high: bool) -> SbrGrid {
    SbrGrid {
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
    }
}

/// A VARFIX grid whose leading border `lead` meets the previous
/// frame's late trailing border, with `num_env` envelopes spread as
/// evenly as the even-length relative borders allow, no transient.
fn varfix_plain(lead: u8, num_env: usize) -> SbrGrid {
    let span = NUM_TIME_SLOTS as i32 - i32::from(lead);
    let mut rel = Vec::new();
    let mut used = 0;
    for _ in 1..num_env {
        // Remaining envelopes share the rest; each explicit one is an
        // even length 2..=8 leaving ≥ 1 slot for the implicit last.
        let remaining = num_env - rel.len();
        let len = ((span - used) / remaining as i32).clamp(2, 8);
        let len = len - (len % 2);
        if used + len >= span {
            break;
        }
        rel.push(((len - 2) / 2) as u8);
        used += len;
    }
    let n = rel.len() + 1;
    SbrGrid {
        frame_class: FrameClass::VarFix,
        num_env: n,
        num_noise: if n > 1 { 2 } else { 1 },
        freq_res: vec![n <= 2; n],
        var_bord_0: lead,
        var_bord_1: 0,
        rel_bord_0: rel,
        rel_bord_1: vec![],
        pointer: 0,
        amp_res_override: false,
    }
}

/// §4.B.18.3 variable-border grid for an attack at slot `s` (in
/// `4..NUM_TIME_SLOTS`) with leading border `lead`: a border lands
/// on the attack (rounded down to the even-length grid the
/// `2·bs_rel_bord + 2` syntax can express), a short envelope isolates
/// it, and `bs_pointer` marks it as the transient envelope `lA`, which
/// also splits the two noise floors there (Tables 4.174 / 4.176).
///
/// * `lead == 0` → FIXVAR: the segments after the attack are coded
///   from the trailing border (`bs_rel_bord_1`, last first); the
///   trailing border extends past the frame (`bs_var_bord_1`) when
///   the attack is too late to leave a whole envelope behind it.
/// * `lead > 0` → VARFIX: segments coded from the leading border
///   (`bs_rel_bord_0`); the last envelope absorbs the remainder.
fn variable_grid(s: usize, lead: u8) -> SbrGrid {
    let s = s as i32;
    let lead_i = i32::from(lead);
    if lead == 0 {
        // Even attack border ≥ 4.
        let sp = s - (s % 2);
        // Trailing border: nominal, or extended so the tail is ≥ 2.
        let t = (NUM_TIME_SLOTS as i32).max(sp + 4);
        let rem = t - sp;
        // Even segments after the attack, each 2..=8.
        let segs: Vec<i32> = if rem <= 2 {
            vec![rem]
        } else if rem <= 10 {
            vec![2, rem - 2]
        } else {
            vec![2, 8, rem - 10]
        };
        let num_env = segs.len() + 1;
        let rel_bord_1: Vec<u8> = segs.iter().rev().map(|&l| ((l - 2) / 2) as u8).collect();
        // The attack envelope is index 1 (after the implicit lead-in).
        let l_a = 1u32;
        SbrGrid {
            frame_class: FrameClass::FixVar,
            num_env,
            num_noise: 2,
            freq_res: vec![false; num_env],
            var_bord_0: 0,
            var_bord_1: (t - NUM_TIME_SLOTS as i32) as u8,
            rel_bord_0: vec![],
            rel_bord_1,
            pointer: num_env as u32 + 1 - l_a,
            amp_res_override: false,
        }
    } else {
        // Attack border with the lead's parity, at least `lead + 2`.
        let mut sp = s - ((s - lead_i).rem_euclid(2));
        if sp < lead_i + 2 {
            sp = lead_i + 2;
        }
        let first = sp - lead_i; // even, ≥ 2
        let mut rel: Vec<i32> = if first <= 8 {
            vec![first]
        } else {
            vec![8, first - 8]
        };
        // Short attack envelope, then the implicit tail (≥ 1 slot).
        if sp + 2 < NUM_TIME_SLOTS as i32 {
            rel.push(2);
        }
        let l_a = rel.len() as u32 - 1 + u32::from(sp + 2 < NUM_TIME_SLOTS as i32);
        let num_env = rel.len() + 1;
        let l_a = l_a.min(num_env as u32 - 1).max(1);
        SbrGrid {
            frame_class: FrameClass::VarFix,
            num_env,
            num_noise: 2,
            freq_res: vec![false; num_env],
            var_bord_0: lead,
            var_bord_1: 0,
            rel_bord_0: rel.iter().map(|&l| ((l - 2) / 2) as u8).collect(),
            rel_bord_1: vec![],
            pointer: l_a + 1,
            amp_res_override: false,
        }
    }
}

/// Pre-compensate the envelope energies for the noise path's synthesis
/// loss. The §4.6.18.7.6 assembly injects the noise component as
/// independent complex values (Table 4.A.91); unlike the
/// analysis-coherent patched signal, only about half of that nominal
/// `|X|²` survives the real-output synthesis bank (the non-analytic
/// half cancels — measured on this crate's own filterbank pair). A
/// band reconstructed with noise fraction `f = Q/(1+Q)` therefore
/// lands at `(1 − f) + f/2` of its envelope value; scaling the
/// transmitted envelope by the inverse `(1 + Q)/(1 + Q/2)` restores
/// the band energy the decoder actually delivers.
fn compensate_noise_loss(
    energy: &mut [Vec<f64>],
    qq: &[Vec<i32>],
    t_e: &[i32],
    t_q: &[i32],
    freq_res: &[bool],
    bands: &HiLoTables,
) {
    for (l, row) in energy.iter_mut().enumerate() {
        // The noise floor whose span contains this envelope's middle.
        let mid = (t_e[l] + t_e[l + 1]) / 2;
        let mut fl = 0usize;
        for i in 0..t_q.len() - 1 {
            if mid >= t_q[i] && mid < t_q[i + 1] {
                fl = i;
            }
        }
        let table = if freq_res[l] {
            &bands.f_table_high
        } else {
            &bands.f_table_low
        };
        for (p, e) in row.iter_mut().enumerate() {
            let centre = (table[p] + table[p + 1]) / 2;
            let mut nb = 0usize;
            for i in 0..bands.f_table_noise.len() - 1 {
                if centre >= bands.f_table_noise[i] && centre < bands.f_table_noise[i + 1] {
                    nb = i;
                }
            }
            let q = 2f64.powi(6 - qq[fl].get(nb).copied().unwrap_or(30));
            *e *= (1.0 + q) / (1.0 + q / 2.0);
        }
    }
}

/// §4.B.18.6 envelope quantisation: `E_Q = INT(a·max(log2(E/64), 0)
/// + 0.5)`.
pub fn quantise_envelopes(energy: &[Vec<f64>], amp_res: bool) -> Vec<Vec<i32>> {
    let a = if amp_res { 1.0 } else { 2.0 };
    let max = if amp_res { 63 } else { 127 };
    energy
        .iter()
        .map(|row| {
            row.iter()
                .map(|&e| {
                    let v = if e > 0.0 {
                        (e / 64.0).log2().max(0.0)
                    } else {
                        0.0
                    };
                    ((a * v + 0.5).floor() as i32).clamp(0, max)
                })
                .collect()
        })
        .collect()
}

/// §4.B.18.6 noise-floor quantisation: `Q_Q = INT(NOISE_FLOOR_OFFSET −
/// log2(Q) + 0.5)` limited to `[0, 30]`.
pub fn quantise_noise(q: &[Vec<f64>]) -> Vec<Vec<i32>> {
    q.iter()
        .map(|row| {
            row.iter()
                .map(|&v| {
                    ((NOISE_FLOOR_OFFSET - v.max(1e-30).log2() + 0.5).floor() as i32).clamp(0, 30)
                })
                .collect()
        })
        .collect()
}

/// Codeword length of `delta` in `table`, or `None` when it exceeds
/// the LAV.
fn code_len(table: &[(u8, u32)], lav: i32, delta: i32) -> Option<u32> {
    let idx = delta + lav;
    if idx < 0 || idx as usize >= table.len() {
        None
    } else {
        Some(u32::from(table[idx as usize].0))
    }
}

/// Delta-code one channel's envelopes (§4.B.18.7), choosing the
/// cheaper direction per envelope closed-loop. `step` is `1/δ` (`2`
/// for the coupled balance channel, whose reconstructed values are
/// `2 × wire`). Returns the raw `bs_data_env` rows and `bs_df_env`.
fn code_envelopes(
    target: &[Vec<i32>],
    grid: &SbrGrid,
    bands: &HiLoTables,
    ctx: SbrHuffContext,
    prev: Option<&EnvelopeScalefactors>,
    force_freq_first: bool,
    step: i32,
) -> (Vec<Vec<i32>>, Vec<bool>) {
    let ((t_huff, t_lav), (f_huff, f_lav)) = env_tables(ctx);
    let start_bits = if ctx.coupling && ctx.ch {
        if ctx.amp_res {
            5
        } else {
            6
        }
    } else if ctx.amp_res {
        6
    } else {
        7
    };
    let start_max = (1i32 << start_bits) - 1;
    let mut data = Vec::with_capacity(grid.num_env);
    let mut df = Vec::with_capacity(grid.num_env);
    // Reconstructed rows of this frame so far (decoder view).
    let mut rec: Vec<Vec<i32>> = Vec::with_capacity(grid.num_env);
    for (l, row) in target.iter().enumerate() {
        let cur_high = grid.freq_res[l];
        let n = row.len();
        // Frequency direction.
        let mut f_raw = Vec::with_capacity(n);
        let mut f_rec = Vec::with_capacity(n);
        let mut f_bits = start_bits;
        let start = (row[0] / step).clamp(0, start_max);
        f_raw.push(start);
        f_rec.push(start * step);
        for k in 1..n {
            let want = row[k] - f_rec[k - 1];
            let d = (want / step).clamp(-f_lav, f_lav);
            f_bits += code_len(f_huff, f_lav, d).unwrap_or(0);
            f_raw.push(d);
            f_rec.push(f_rec[k - 1] + d * step);
        }
        // Time direction.
        let reference: Option<(Vec<i32>, bool)> = if l >= 1 {
            Some((rec[l - 1].clone(), grid.freq_res[l - 1]))
        } else if let Some(p) = prev {
            let last = p.eq.len().saturating_sub(1);
            p.eq.get(last)
                .map(|r| (r.clone(), *p.freq_res.get(last).unwrap_or(&cur_high)))
        } else {
            None
        };
        let time = reference.and_then(|(prev_row, prev_high)| {
            if l == 0 && force_freq_first {
                return None;
            }
            let mut t_raw = Vec::with_capacity(n);
            let mut t_rec = Vec::with_capacity(n);
            let mut t_bits = 0u32;
            for (k, &target_k) in row.iter().enumerate() {
                let g = ref_band(bands, &prev_row, cur_high, prev_high, k);
                let d = ((target_k - g) / step).clamp(-t_lav, t_lav);
                t_bits += code_len(t_huff, t_lav, d)?;
                t_raw.push(d);
                t_rec.push(g + d * step);
            }
            Some((t_raw, t_rec, t_bits))
        });
        // Prefer the direction with fewer bits; on a tie, frequency
        // (no dependence on the previous frame).
        let f_err: i64 = row
            .iter()
            .zip(f_rec.iter())
            .map(|(&a, &b)| i64::from((a - b).abs()))
            .sum();
        match time {
            Some((t_raw, t_rec, t_bits)) => {
                let t_err: i64 = row
                    .iter()
                    .zip(t_rec.iter())
                    .map(|(&a, &b)| i64::from((a - b).abs()))
                    .sum();
                if t_err < f_err || (t_err == f_err && t_bits < f_bits) {
                    data.push(t_raw);
                    df.push(true);
                    rec.push(t_rec);
                } else {
                    data.push(f_raw);
                    df.push(false);
                    rec.push(f_rec);
                }
            }
            None => {
                data.push(f_raw);
                df.push(false);
                rec.push(f_rec);
            }
        }
    }
    (data, df)
}

/// Delta-code one channel's noise floors — the noise analogue of
/// [`code_envelopes`] (one resolution, 5-bit start value).
fn code_noise(
    target: &[Vec<i32>],
    grid: &SbrGrid,
    ctx: SbrHuffContext,
    prev: Option<&NoiseScalefactors>,
    force_freq_first: bool,
    step: i32,
) -> (Vec<Vec<i32>>, Vec<bool>) {
    let ((t_huff, t_lav), (f_huff, f_lav)) = noise_tables(ctx);
    let mut data = Vec::with_capacity(grid.num_noise);
    let mut df = Vec::with_capacity(grid.num_noise);
    let mut rec: Vec<Vec<i32>> = Vec::with_capacity(grid.num_noise);
    for (l, row) in target.iter().enumerate() {
        let n = row.len();
        let mut f_raw = Vec::with_capacity(n);
        let mut f_rec = Vec::with_capacity(n);
        let mut f_bits = 5u32;
        let start = (row[0] / step).clamp(0, 31);
        f_raw.push(start);
        f_rec.push(start * step);
        for k in 1..n {
            let d = ((row[k] - f_rec[k - 1]) / step).clamp(-f_lav, f_lav);
            f_bits += code_len(f_huff, f_lav, d).unwrap_or(0);
            f_raw.push(d);
            f_rec.push(f_rec[k - 1] + d * step);
        }
        let reference: Option<Vec<i32>> = if l >= 1 {
            Some(rec[l - 1].clone())
        } else if let Some(p) = prev {
            p.q.last().cloned()
        } else {
            None
        };
        let time = reference.and_then(|prev_row| {
            if (l == 0 && force_freq_first) || prev_row.len() != n {
                return None;
            }
            let mut t_raw = Vec::with_capacity(n);
            let mut t_rec = Vec::with_capacity(n);
            let mut t_bits = 0u32;
            for k in 0..n {
                let d = ((row[k] - prev_row[k]) / step).clamp(-t_lav, t_lav);
                t_bits += code_len(t_huff, t_lav, d)?;
                t_raw.push(d);
                t_rec.push(prev_row[k] + d * step);
            }
            Some((t_raw, t_rec, t_bits))
        });
        let f_err: i64 = row
            .iter()
            .zip(f_rec.iter())
            .map(|(&a, &b)| i64::from((a - b).abs()))
            .sum();
        match time {
            Some((t_raw, t_rec, t_bits)) => {
                let t_err: i64 = row
                    .iter()
                    .zip(t_rec.iter())
                    .map(|(&a, &b)| i64::from((a - b).abs()))
                    .sum();
                if t_err < f_err || (t_err == f_err && t_bits < f_bits) {
                    data.push(t_raw);
                    df.push(true);
                    rec.push(t_rec);
                } else {
                    data.push(f_raw);
                    df.push(false);
                    rec.push(f_rec);
                }
            }
            None => {
                data.push(f_raw);
                df.push(false);
                rec.push(f_rec);
            }
        }
    }
    (data, df)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sbr_extension::SbrExtensionData;
    use crate::sbr_qmf::EncoderAnalysisQmf;
    use oxideav_core::bits::BitReader;

    /// Analyse a synthetic full-rate signal into `n_cols` columns.
    fn analyse(signal: impl Fn(usize) -> f64, n_cols: usize) -> Vec<[Complex; 64]> {
        let mut bank = EncoderAnalysisQmf::new();
        let mut cols = Vec::with_capacity(n_cols);
        // Prime the bank so the first returned column is steady.
        let prime = 10;
        for c in 0..n_cols + prime {
            let slot: Vec<f64> = (0..64).map(|i| signal(c * 64 + i)).collect();
            let x = bank.push_slot(&slot).unwrap();
            if c >= prime {
                cols.push(x);
            }
        }
        cols
    }

    fn tone(band: f64, amp: f64) -> impl Fn(usize) -> f64 {
        move |t| amp * (2.0 * core::f64::consts::PI * band / 128.0 * t as f64).cos()
    }

    #[test]
    fn config_picks_reachable_band_edges() {
        let cfg = SbrEncoderConfig::new(44_100, 1, 6_000.0, 16_000.0).unwrap();
        let enc = SbrEncoder::new(cfg).unwrap();
        let kx = enc.bands().k_x;
        let hz = f64::from(kx) * 44_100.0 / 128.0;
        assert!((hz - 6_000.0).abs() < 1_500.0, "k_x {kx} → {hz} Hz");
        let stop = enc.bands().k_x + enc.bands().m;
        let stop_hz = f64::from(stop) * 44_100.0 / 128.0;
        assert!(
            (stop_hz - 16_000.0).abs() < 1_500.0,
            "stop {stop} → {stop_hz} Hz"
        );
        assert!(SbrEncoderConfig::new(44_100, 3, 6_000.0, 16_000.0).is_err());
    }

    #[test]
    fn quantiser_matches_spec_formulas() {
        // E = 64·2^10 → log2 = 10 → 3 dB: 10, 1.5 dB: 20.
        let e = vec![vec![64.0 * 1024.0, 10.0, 0.0]];
        assert_eq!(quantise_envelopes(&e, true), vec![vec![10, 0, 0]]);
        assert_eq!(quantise_envelopes(&e, false), vec![vec![20, 0, 0]]);
        // Q = 2^-4 → 6 + 4 = 10; Q = 64 → 0; Q tiny → 30.
        let q = vec![vec![2f64.powi(-4), 64.0, 1e-20]];
        assert_eq!(quantise_noise(&q), vec![vec![10, 0, 30]]);
    }

    /// A stationary tone in the SBR range: single-envelope FIXFIX,
    /// the envelope's band holding the tone reads the tone energy the
    /// decoder will reproduce (E_Orig = 64·2^(E_Q/a) within one
    /// quantiser step of the estimate), and the payload reparses.
    #[test]
    fn stationary_tone_codes_one_envelope() {
        let cfg = SbrEncoderConfig::new(44_100, 1, 6_000.0, 16_000.0).unwrap();
        let mut enc = SbrEncoder::new(cfg).unwrap();
        let cols = analyse(tone(30.5, 2000.0), SBR_ENC_COLS);
        let frame = enc.encode_frame(&[&cols]).unwrap();
        assert!(frame.header_sent);
        let ch = &frame.element.channels[0];
        assert_eq!(ch.grid.frame_class, FrameClass::FixFix);
        assert_eq!(ch.grid.num_env, 1);
        assert!(!frame.reports[0].transient);
        // Band containing subband 30 on the high table.
        let b = enc.bands();
        let p = (0..b.n_high())
            .find(|&p| b.f_table_high[p] <= 30 && 30 < b.f_table_high[p + 1])
            .unwrap();
        let e = frame.reports[0].energy[0][p];
        let eq = frame.reports[0].eq[0][p];
        // Single-envelope FIXFIX → 1.5 dB steps (a = 2). The wire
        // value carries the noise-path compensation on top of the
        // estimate: undo it from the coded noise floor before
        // comparing.
        let centre = (b.f_table_high[p] + b.f_table_high[p + 1]) / 2;
        let nb = (0..b.n_q())
            .find(|&n| b.f_table_noise[n] <= centre && centre < b.f_table_noise[n + 1])
            .unwrap();
        let qdec = 2f64.powi(6 - frame.reports[0].qq[0][nb]);
        let comp = (1.0 + qdec) / (1.0 + qdec / 2.0);
        let e_orig = 64.0 * 2f64.powf(f64::from(eq) / 2.0) / comp;
        let db = 10.0 * (e_orig / e).log10();
        assert!(
            db.abs() < 1.6,
            "envelope error {db} dB (E {e}, E_Q {eq}, comp {comp})"
        );
        // Reparse through the decoder-side walker.
        let mut r = BitReader::new(&frame.payload);
        r.read_u32(4).unwrap();
        let parsed = SbrExtensionData::parse(
            &mut r,
            IdSynEle::Sce,
            false,
            44_100,
            Some(frame.payload.len() as u32),
            None,
        )
        .unwrap();
        assert_eq!(parsed.element, frame.element);
        assert_eq!(parsed.header, *enc.header());

        // Second frame: no header, time-direction coding allowed and
        // chosen for the unchanged envelope (zero deltas).
        let frame2 = enc.encode_frame(&[&cols]).unwrap();
        assert!(!frame2.header_sent);
        let ch2 = &frame2.element.channels[0];
        assert!(ch2.dtdf.df_env[0]);
        assert!(ch2.envelope.data[0].iter().all(|&d| d == 0));
        assert_eq!(frame2.reports[0].eq, frame.reports[0].eq);
    }

    /// An attack in the SBR range mid-frame elects a variable grid
    /// whose border lands on the attack slot, and the envelope before
    /// the attack is far quieter than the one after it.
    #[test]
    fn transient_elects_variable_border_on_the_attack() {
        let cfg = SbrEncoderConfig::new(44_100, 1, 6_000.0, 16_000.0).unwrap();
        let mut enc = SbrEncoder::new(cfg).unwrap();
        // Silence, then a loud HF tone from slot 9 of the nominal
        // frame (column 2 + 2·9 = 20 → sample 20·64 in the analysed
        // window; the analysis primes 10 columns first).
        let onset = (10 + 20) * 64;
        let cols = analyse(
            move |t| {
                if t >= onset {
                    tone(40.5, 3000.0)(t)
                } else {
                    0.0
                }
            },
            SBR_ENC_COLS,
        );
        let frame = enc.encode_frame(&[&cols]).unwrap();
        let rep = &frame.reports[0];
        assert!(rep.transient);
        let ch = &frame.element.channels[0];
        assert!(ch.grid.num_env >= 2, "{:?}", ch.grid);
        // The filterbank's group delay moves the onset a few columns
        // later than the input sample index; the border lands on the
        // even slot at or before the detected attack.
        let l_attack = rep
            .t_e
            .iter()
            .position(|&b| (8..=12).contains(&b))
            .unwrap_or_else(|| panic!("borders {:?}", rep.t_e));
        let before: f64 = rep.energy[l_attack - 1].iter().sum();
        let after: f64 = rep.energy[l_attack].iter().sum();
        assert!(
            after > 1e3 * before.max(1.0),
            "before {before} after {after}"
        );
        // Reparse.
        let mut r = BitReader::new(&frame.payload);
        r.read_u32(4).unwrap();
        let parsed = SbrExtensionData::parse(
            &mut r,
            IdSynEle::Sce,
            false,
            44_100,
            Some(frame.payload.len() as u32),
            None,
        )
        .unwrap();
        assert_eq!(parsed.element, frame.element);
        // The next frame's leading border meets this trailing one.
        let frame2 = enc.encode_frame(&[&cols]).unwrap();
        assert_eq!(frame2.reports[0].t_e[0], rep.t_e[rep.t_e.len() - 1] - 16);
    }

    /// Noise floor: white noise in the SBR range whose patch source is
    /// a pure tone gets a high noise floor and strong inverse
    /// filtering; a tone copied from a tone gets neither.
    #[test]
    fn noise_floor_and_invf_follow_tonality_mismatch() {
        let cfg = SbrEncoderConfig::new(44_100, 1, 6_000.0, 16_000.0).unwrap();
        let mut enc = SbrEncoder::new(cfg).unwrap();
        let kx = enc.bands().k_x as usize;
        let src = enc.source_band(kx + 1) as f64 + 0.5;
        // Low band: tone in the source subband; high band: noise.
        let mut seed = 0x1234_5678u32;
        let mut noise = vec![0.0f64; 64 * (SBR_ENC_COLS + 12)];
        for v in noise.iter_mut() {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *v = (f64::from(seed >> 8) / f64::from(1u32 << 24) - 0.5) * 2000.0;
        }
        let cols = analyse(move |t| tone(src, 3000.0)(t) + noise[t] * 1.0, SBR_ENC_COLS);
        let frame = enc.encode_frame(&[&cols]).unwrap();
        let ch = &frame.element.channels[0];
        // The noise band fed by the tone subband copies a tonal
        // source into a noisy original: strong inverse filtering and
        // a real noise floor there. (Bands whose source is itself the
        // broadband noise need neither.)
        assert!(ch.invf.invf_mode[0] >= 2, "{:?}", ch.invf);
        let qq = &frame.reports[0].qq[0];
        assert!(qq[0] < 30, "noise floors {qq:?}");

        // Tone copied from a tone: the copy is as tonal as the
        // original → invf off, noise floor at the minimum.
        let mut enc2 = SbrEncoder::new(cfg).unwrap();
        let high = (kx + 1) as f64 + 0.5;
        let cols2 = analyse(
            move |t| tone(src, 3000.0)(t) + tone(high, 3000.0)(t),
            SBR_ENC_COLS,
        );
        let frame2 = enc2.encode_frame(&[&cols2]).unwrap();
        let ch2 = &frame2.element.channels[0];
        assert_eq!(ch2.invf.invf_mode[0], 0, "{:?}", ch2.invf);
        assert_eq!(frame2.reports[0].qq[0][0], 30);
    }

    /// Coupled pair: the balance channel codes even values around
    /// panOffset and the decoder's dequantisation recovers the
    /// left/right energies.
    #[test]
    fn coupled_pair_round_trips_left_right_levels() {
        let mut cfg = SbrEncoderConfig::new(44_100, 2, 6_000.0, 16_000.0).unwrap();
        cfg.coupling = true;
        let mut enc = SbrEncoder::new(cfg).unwrap();
        let l = analyse(tone(30.5, 2000.0), SBR_ENC_COLS);
        let r = analyse(tone(30.5, 500.0), SBR_ENC_COLS);
        let frame = enc.encode_frame(&[&l, &r]).unwrap();
        assert!(frame.element.coupling);
        let c1 = &frame.element.channels[1];
        assert!(c1.invf.invf_mode.is_empty());
        // Decoder-side dequantisation.
        let e0 = EnvelopeScalefactors {
            eq: frame.reports[0].eq.clone(),
            freq_res: c1.grid.freq_res.clone(),
        };
        let e1 = EnvelopeScalefactors {
            eq: frame.reports[1].eq.clone(),
            freq_res: c1.grid.freq_res.clone(),
        };
        let n0 = NoiseScalefactors {
            q: frame.reports[0].qq.clone(),
        };
        let n1 = NoiseScalefactors {
            q: frame.reports[1].qq.clone(),
        };
        let eff_amp = enc.header().amp_res && !c1.grid.amp_res_override;
        let (left, right) = crate::sbr_dequant::dequant_coupled(&e0, &n0, &e1, &n1, eff_amp);
        let b = enc.bands();
        let p = (0..b.n_high())
            .find(|&p| b.f_table_high[p] <= 30 && 30 < b.f_table_high[p + 1])
            .unwrap();
        let el = frame.reports[0].energy[0][p];
        let er = frame.reports[1].energy[0][p];
        let dl = 10.0 * (left.e_orig[0][p] / el).log10();
        let dr = 10.0 * (right.e_orig[0][p] / er).log10();
        assert!(dl.abs() < 2.0 && dr.abs() < 2.0, "L {dl} dB, R {dr} dB");
        // Reparse.
        let mut rd = BitReader::new(&frame.payload);
        rd.read_u32(4).unwrap();
        let parsed = SbrExtensionData::parse(
            &mut rd,
            IdSynEle::Cpe,
            false,
            44_100,
            Some(frame.payload.len() as u32),
            None,
        )
        .unwrap();
        assert_eq!(parsed.element, frame.element);
    }
}

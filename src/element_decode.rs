//! Channel-element decode driver — the §4.6 block-order chain that
//! turns a parsed `single_channel_element()` (SCE / LFE) or
//! `channel_pair_element()` (CPE) into PCM-domain samples.
//!
//! Every per-tool reconstruction primitive landed in earlier rounds;
//! what was missing was the element-level glue that runs them in the
//! ISO/IEC 14496-3 §4.6 block order and carries the per-channel
//! filterbank overlap state across frames. This module is that glue.
//!
//! ## §4.6 block order
//!
//! For a single channel the per-channel chain is (§4.6, Figure 4.1 /
//! the "Decoder block diagram"):
//!
//! 1. **Noiseless decoding** — `spectral_data()` (Table 4.56), already
//!    parsed into [`crate::spectral_data::SpectralData`].
//! 2. **Pulse fix-up** (§4.6.3.3) — fold the `±pulse_amp` corrections
//!    into the quantised spectrum (long windows only, Table 4.50
//!    Note 1).
//! 3. **Inverse quantisation** (§4.6.1.3) + **scalefactor application**
//!    (§4.6.2.3.3) — [`crate::dequant::rescale_spectrum`] over the
//!    §4.6.2.3.2-accumulated absolute scalefactors.
//! 4. **De-interleave** (§4.6.3.3 `quant_to_spec()`) — group-interleaved
//!    transmission order → window-major `spec[w][k]`
//!    ([`crate::decoded_spectrum::quant_to_spec`]).
//! 5. **Joint stereo / noise** (CPE only, §4.6.8 / §4.6.13) — M/S
//!    de-matrix (§4.6.8.1), then intensity stereo (§4.6.8.2), then PNS
//!    (§4.6.13). The spec applies these *before* TNS (§4.6.13.5: noise
//!    is injected prior to the TNS step) and on the de-interleaved
//!    pre-TNS spectrum, which is exactly the
//!    [`crate::ms_stereo::ChannelPairSpectra`] /
//!    [`crate::intensity_stereo::IntensityPairSpectra`] /
//!    [`crate::pns::PnsChannel`] contract.
//! 6. **TNS** (§4.6.9) — [`crate::tns_frame::tns_decode_frame`] in
//!    place on the window-major spectrum.
//! 7. **Filterbank** (§4.6.11) — IMDCT + window + inter-frame
//!    overlap-add ([`crate::filterbank::Filterbank::synthesize`]),
//!    emitting `LONG_WINDOW_LEN` (1024) PCM samples per channel per
//!    frame.
//!
//! Because the joint-stereo / noise tools (step 5) sit *between*
//! `quant_to_spec()` and TNS, the CPE path cannot reuse the
//! single-channel [`crate::decoded_spectrum::decode_channel_spectrum`]
//! (which runs TNS internally at the end of its own chain). This module
//! therefore composes the finer-grained primitives directly:
//! [`reconstruct_pre_pair`] runs steps 2–4 for one channel, the pair
//! tools run on both pre-TNS spectra, then [`finish_channel`] runs
//! steps 6–7 per channel.
//!
//! ## Scope
//!
//! * The Main-profile predictor (§4.6.7) and LTP (§4.6.6) are not
//!   applied — `ics_info()` surfaces their side-info but the
//!   prediction loops are a later tool, so a stream that flips those
//!   bits decodes as if prediction were off (the spec's behaviour when
//!   `predictor_data_present == 0`).
//! * The SSR (AOT 3) gain-control ladder (§4.6.12) is parsed but not
//!   applied; this driver targets the LC / Main / LTP filterbank.
//! * PNS output is RNG-defined per §4.6.13.3 (only the per-band L2 norm
//!   is spec-determined); the driver uses the default
//!   [`crate::pns::gen_rand_vector`] LCG, seeded once per decoder so the
//!   noise is reproducible across a decode run.

use crate::decoded_spectrum::quant_to_spec;
use crate::dequant::rescale_spectrum;
use crate::filterbank::Filterbank;
use crate::ics_body::IcsBody;
use crate::ics_info::IcsInfo;
use crate::intensity_stereo::{apply_intensity_stereo, IntensityPairSpectra};
use crate::ms_stereo::{apply_ms_stereo, ChannelPairSpectra, MsMaskPresent};
use crate::pns::{apply_pns, apply_pns_pair, gen_rand_vector, PnsChannel};
use crate::scale_factor_data::{accumulate, AbsoluteScaleFactorEntry, AbsoluteScaleFactors};
use crate::section_data::ZERO_HCB;
use crate::spectral_data::SpectralData;
use crate::swb_offset::apply_pulse_data;
use crate::tns_frame::tns_decode_frame;
use crate::{Error, Result};

/// One channel's parsed Table 4.50 body plus its Table 4.56 spectrum,
/// bundled so the element driver can take them by reference.
#[derive(Debug)]
pub struct ChannelInput<'a> {
    /// The parsed `individual_channel_stream()` body
    /// ([`IcsBody::parse`] / [`IcsBody::parse_with_ics_info`]).
    pub body: &'a IcsBody,
    /// The channel's `ics_info()`. For an SCE / LFE or a non-shared
    /// CPE this is `body.ics_info`; for a `common_window == 1` CPE this
    /// is the shared `ics_info` the caller parsed once.
    pub ics_info: &'a IcsInfo,
    /// The channel's parsed `spectral_data()`
    /// ([`SpectralData::parse`]).
    pub spectral: &'a SpectralData,
}

/// Expand a wire-order [`AbsoluteScaleFactors`] into the band-indexed
/// `track[g][sfb]` layout (size `num_window_groups × max_sfb`) the
/// §4.6.8.2 / §4.6.13 synthesis passes consume.
///
/// `accumulate()` returns one record per non-`ZERO_HCB` band in
/// wire (low-frequency-first) order; the joint-stereo / noise tools
/// instead index by `(g, sfb)`. This walks `sfb_cb[g][sfb]` in lock-step
/// with the wire records and scatters the requested track value into the
/// `(g, sfb)` slot, leaving non-matching bands at `default`.
///
/// `pick` maps an [`AbsoluteScaleFactorEntry`] to the track value of
/// interest (`is_pos` or `noise_nrg`), or `None` for a record that
/// belongs to a different track (in which case the slot stays
/// `default`).
fn band_indexed_track<F>(
    abs: &AbsoluteScaleFactors,
    sfb_cb: &[Vec<u8>],
    max_sfb: usize,
    default: i32,
    pick: F,
) -> Result<Vec<Vec<i32>>>
where
    F: Fn(&AbsoluteScaleFactorEntry) -> Option<i32>,
{
    if abs.entries.len() != sfb_cb.len() {
        return Err(Error::ElementDecodeInvalid);
    }
    let mut out: Vec<Vec<i32>> = Vec::with_capacity(sfb_cb.len());
    for (group_records, group_cb) in abs.entries.iter().zip(sfb_cb.iter()) {
        if group_cb.len() < max_sfb {
            return Err(Error::ElementDecodeInvalid);
        }
        let mut row = vec![default; max_sfb];
        let mut rec = group_records.iter();
        for (sfb, &cb) in group_cb.iter().enumerate() {
            if cb == ZERO_HCB {
                continue;
            }
            // Every non-ZERO_HCB band consumes exactly one wire record,
            // in lock-step with the accumulate() walk.
            let entry = rec.next().ok_or(Error::ElementDecodeInvalid)?;
            if sfb < max_sfb {
                if let Some(v) = pick(entry) {
                    row[sfb] = v;
                }
            }
        }
        out.push(row);
    }
    Ok(out)
}

/// Band-indexed `is_pos[g][sfb]` (§4.6.8.1.4), default `0` on
/// non-intensity bands.
fn is_pos_table(
    abs: &AbsoluteScaleFactors,
    sfb_cb: &[Vec<u8>],
    max_sfb: usize,
) -> Result<Vec<Vec<i32>>> {
    band_indexed_track(abs, sfb_cb, max_sfb, 0, |e| match e {
        AbsoluteScaleFactorEntry::IsPos(p) => Some(i32::from(*p)),
        _ => None,
    })
}

/// Band-indexed `noise_nrg[g][sfb]` (§4.6.13.3), default `0` on
/// non-noise bands.
fn noise_nrg_table(
    abs: &AbsoluteScaleFactors,
    sfb_cb: &[Vec<u8>],
    max_sfb: usize,
) -> Result<Vec<Vec<i32>>> {
    band_indexed_track(abs, sfb_cb, max_sfb, 0, |e| match e {
        AbsoluteScaleFactorEntry::NoiseNrg(n) => Some(*n),
        _ => None,
    })
}

/// Run §4.6 steps 2–4 for one channel: pulse fix-up → scalefactor
/// accumulation → inverse quantisation + rescaling → `quant_to_spec()`.
///
/// Returns the window-major **pre-TNS** spectrum (the joint-stereo /
/// noise tools' input) alongside the accumulated absolute scalefactors
/// (so the caller can derive the band-indexed `is_pos` / `noise_nrg`
/// tracks without re-running the accumulator).
fn reconstruct_pre_pair(
    ch: &ChannelInput<'_>,
    fs_index: u8,
) -> Result<(Vec<f64>, AbsoluteScaleFactors)> {
    // 2. §4.6.3.3 pulse fix-up on the quantised spectrum (long windows
    //    only — the parser already rejects pulse on EIGHT_SHORT, and a
    //    long sequence has exactly one group).
    let x_quant: SpectralData = if let Some(pd) = &ch.body.pulse_data {
        let mut patched = ch.spectral.clone();
        let group0 = patched.x_quant.first_mut().ok_or(Error::DequantInvalid)?;
        apply_pulse_data(group0, fs_index, pd)?;
        patched
    } else {
        ch.spectral.clone()
    };

    // 3a. §4.6.2.3.2 scalefactor accumulation.
    let abs = accumulate(
        &ch.body.scale_factor_data,
        &ch.body.section_data.sfb_cb,
        ch.body.global_gain,
    )?;

    // 3b. §4.6.1.3 + §4.6.2.3.3 inverse quantisation + rescaling.
    let rescaled = rescale_spectrum(
        &x_quant,
        &abs,
        &ch.body.section_data.sfb_cb,
        ch.ics_info,
        fs_index,
    )?;

    // 4. §4.6.3.3 quant_to_spec() de-interleaving.
    let spec = quant_to_spec(&rescaled, ch.ics_info, fs_index)?;
    Ok((spec, abs))
}

/// Run §4.6 steps 6–7 for one channel: TNS (§4.6.9) in place, then the
/// §4.6.11 filterbank to PCM, advancing `fb`'s overlap state.
fn finish_channel(
    spec: &mut [f64],
    body: &IcsBody,
    ics_info: &IcsInfo,
    aot: u8,
    fs_index: u8,
    fb: &mut Filterbank,
) -> Result<Vec<f64>> {
    if let Some(tns) = &body.tns_data {
        tns_decode_frame(
            spec,
            tns,
            ics_info.window_sequence,
            ics_info.max_sfb,
            aot,
            fs_index,
        )?;
    }
    fb.synthesize(spec, ics_info)
}

/// The shared `channel_pair_element()` joint-stereo header (Table 4.4)
/// the caller reads after `common_window`.
///
/// Only meaningful when `common_window == 1`. For
/// `common_window == 0` both channels carry their own `ics_info()` and
/// no M/S mask is transmitted, so the joint-stereo tools do not run.
#[derive(Debug, Clone)]
pub struct CpeJointStereo {
    /// Decoded `ms_mask_present` (§4.6.8.1.1, Table 4.4): `00`
    /// all-zeros, `01` per-band `ms_used` mask, `10` all-ones; `11` is
    /// reserved (the caller rejects it before constructing this).
    pub ms_mask_present: MsMaskPresent,
    /// `ms_used[g][sfb]` when `ms_mask_present == 01`; empty otherwise.
    /// Each row must cover `max_sfb`.
    pub ms_used: Vec<Vec<bool>>,
}

impl Default for CpeJointStereo {
    /// The `common_window == 0` / no-joint-stereo default: all-zeros
    /// M/S mask (an identity de-matrix) and no per-band `ms_used`.
    fn default() -> Self {
        CpeJointStereo {
            ms_mask_present: MsMaskPresent::AllZeros,
            ms_used: Vec::new(),
        }
    }
}

/// Stateful per-element decoder: holds one [`Filterbank`] per channel
/// slot (so the inter-frame overlap-add tail and previous-block window
/// shape persist across frames) and the PNS generator state.
///
/// Construct one [`ElementDecoder`] per channel element of the stream
/// (one for an SCE / LFE, one for a CPE) and call [`Self::decode_sce`]
/// / [`Self::decode_cpe`] once per frame.
#[derive(Debug, Clone)]
pub struct ElementDecoder {
    /// Per-channel filterbanks. `[0]` for the SCE / LFE or the CPE's
    /// first channel; `[1]` for the CPE's second channel.
    filterbanks: [Filterbank; 2],
    /// §4.6.13.3 default generator state, advanced across every noise
    /// band of every frame so the noise is reproducible per decode run.
    pns_state: u32,
}

impl Default for ElementDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl ElementDecoder {
    /// A fresh element decoder with zeroed filterbank overlap and a
    /// fixed PNS generator seed.
    pub fn new() -> Self {
        ElementDecoder {
            filterbanks: [Filterbank::new(), Filterbank::new()],
            // Any non-zero seed yields a non-degenerate sequence; the
            // §4.6.13.3 normalisation makes the per-band energy
            // independent of the seed, so this choice only fixes the
            // (spec-undefined) per-coefficient phase.
            pns_state: 0x0001_2345,
        }
    }

    /// A fresh element decoder with an explicit PNS generator seed.
    /// Per §4.6.13.3 the seed only affects the noise *phase*, not the
    /// (spec-determined) per-band energy.
    pub fn with_pns_seed(seed: u32) -> Self {
        ElementDecoder {
            filterbanks: [Filterbank::new(), Filterbank::new()],
            pns_state: seed,
        }
    }

    /// Decode one single-channel element (SCE) or LFE channel to PCM.
    ///
    /// Runs the full §4.6 single-channel chain (pulse → dequant →
    /// `quant_to_spec()` → PNS → TNS → filterbank). M/S and intensity
    /// stereo are channel-*pair* tools and do not apply to an SCE; PNS
    /// (§4.6.13) does, so a single-channel noise band is synthesised
    /// here.
    ///
    /// Returns `LONG_WINDOW_LEN` (1024) PCM-domain samples for the
    /// frame.
    pub fn decode_sce(&mut self, ch: &ChannelInput<'_>, aot: u8, fs_index: u8) -> Result<Vec<f64>> {
        let (mut spec, abs) = reconstruct_pre_pair(ch, fs_index)?;
        let max_sfb = ch.ics_info.max_sfb as usize;

        // §4.6.13 PNS on the single channel (no pair correlation).
        let noise_nrg = noise_nrg_table(&abs, &ch.body.section_data.sfb_cb, max_sfb)?;
        let state = &mut self.pns_state;
        let mut pns_chan = PnsChannel {
            spec: &mut spec,
            sfb_cb: &ch.body.section_data.sfb_cb,
            noise_nrg: &noise_nrg,
        };
        apply_pns(&mut pns_chan, ch.ics_info, fs_index, |out| {
            gen_rand_vector(out, state)
        })?;

        finish_channel(
            &mut spec,
            ch.body,
            ch.ics_info,
            aot,
            fs_index,
            &mut self.filterbanks[0],
        )
    }

    /// Decode one channel-pair element (CPE) to a `(left, right)` pair
    /// of PCM frames.
    ///
    /// * `left` / `right` — the two channels' parsed bodies + spectra.
    ///   For the shared-info form both [`ChannelInput::ics_info`] point
    ///   at the same shared `ics_info`.
    /// * `joint` — the Table 4.4 joint-stereo header
    ///   ([`CpeJointStereo`]); pass [`CpeJointStereo::default`] (mask
    ///   all-zeros, no `ms_used`) for a `common_window == 0` pair, where
    ///   no joint-stereo tools run.
    ///
    /// Runs the full §4.6 chain with the joint-stereo / noise tools in
    /// block order: per-channel pulse → dequant → `quant_to_spec()`,
    /// then M/S (§4.6.8.1) → intensity (§4.6.8.2) → PNS (§4.6.13) on the
    /// pre-TNS pair, then per-channel TNS (§4.6.9) → filterbank
    /// (§4.6.11).
    ///
    /// Both channels must share `window_sequence` (the `common_window`
    /// geometry the §4.6.8 tools require) when any joint-stereo tool is
    /// active; a mismatch surfaces as [`Error::ElementDecodeInvalid`].
    pub fn decode_cpe(
        &mut self,
        left: &ChannelInput<'_>,
        right: &ChannelInput<'_>,
        joint: &CpeJointStereo,
        aot: u8,
        fs_index: u8,
    ) -> Result<(Vec<f64>, Vec<f64>)> {
        // The §4.6.8 joint-stereo tools de-matrix the two channels
        // band-for-band, so they require a shared window geometry. The
        // shared-info CPE form guarantees this; reject a mismatch the
        // non-shared form might present.
        if left.ics_info.window_sequence != right.ics_info.window_sequence
            || left.ics_info.num_window_groups != right.ics_info.num_window_groups
            || left.ics_info.window_group_length != right.ics_info.window_group_length
        {
            return Err(Error::ElementDecodeInvalid);
        }
        // The joint-stereo geometry keys off the shared (here: left)
        // ics_info's max_sfb; the pair tools validate both channels'
        // sfb_cb against it.
        let geom = left.ics_info;
        let max_sfb = geom.max_sfb as usize;

        let (mut left_spec, left_abs) = reconstruct_pre_pair(left, fs_index)?;
        let (mut right_spec, right_abs) = reconstruct_pre_pair(right, fs_index)?;

        // §4.6.8.1 M/S de-matrix (suppressed on intensity / noise bands
        // by apply_ms_stereo itself).
        let ms_used_slice: &[Vec<bool>] = if joint.ms_mask_present == MsMaskPresent::Mask {
            validate_ms_used(&joint.ms_used, geom)?;
            &joint.ms_used
        } else {
            &[]
        };
        {
            let mut pair = ChannelPairSpectra {
                left: &mut left_spec,
                right: &mut right_spec,
                left_sfb_cb: &left.body.section_data.sfb_cb,
                right_sfb_cb: &right.body.section_data.sfb_cb,
            };
            apply_ms_stereo(
                &mut pair,
                joint.ms_mask_present,
                ms_used_slice,
                geom,
                fs_index,
            )?;
        }

        // §4.6.8.2 intensity stereo: right derived from left on
        // intensity bands. invert_intensity reads the per-band M/S mask
        // only when ms_mask_present == 01 (Mask).
        let right_is_pos = is_pos_table(&right_abs, &right.body.section_data.sfb_cb, max_sfb)?;
        let is_mask = joint.ms_mask_present == MsMaskPresent::Mask;
        {
            let mut pair = IntensityPairSpectra {
                left: &left_spec,
                right: &mut right_spec,
                right_sfb_cb: &right.body.section_data.sfb_cb,
                is_pos: &right_is_pos,
            };
            apply_intensity_stereo(&mut pair, is_mask, ms_used_slice, geom, fs_index)?;
        }

        // §4.6.13 PNS with the shared-vector correlation rule. PNS and
        // M/S are mutually exclusive per band (§4.6.13.5), so a noise
        // band was skipped by the M/S de-matrix above; here it is filled.
        let left_nrg = noise_nrg_table(&left_abs, &left.body.section_data.sfb_cb, max_sfb)?;
        let right_nrg = noise_nrg_table(&right_abs, &right.body.section_data.sfb_cb, max_sfb)?;
        let all_shared = joint.ms_mask_present == MsMaskPresent::AllOnes;
        {
            let mut left_chan = PnsChannel {
                spec: &mut left_spec,
                sfb_cb: &left.body.section_data.sfb_cb,
                noise_nrg: &left_nrg,
            };
            let mut right_chan = PnsChannel {
                spec: &mut right_spec,
                sfb_cb: &right.body.section_data.sfb_cb,
                noise_nrg: &right_nrg,
            };
            let state = &mut self.pns_state;
            apply_pns_pair(
                &mut left_chan,
                &mut right_chan,
                is_mask,
                all_shared,
                ms_used_slice,
                geom,
                fs_index,
                |out| gen_rand_vector(out, state),
            )?;
        }

        // §4.6.9 TNS + §4.6.11 filterbank, per channel.
        let out_left = finish_channel(
            &mut left_spec,
            left.body,
            left.ics_info,
            aot,
            fs_index,
            &mut self.filterbanks[0],
        )?;
        let out_right = finish_channel(
            &mut right_spec,
            right.body,
            right.ics_info,
            aot,
            fs_index,
            &mut self.filterbanks[1],
        )?;
        Ok((out_left, out_right))
    }
}

/// Validate that an `ms_used[g][sfb]` mask covers
/// `num_window_groups × max_sfb`. The pair tools re-check this, but
/// surfacing the element-level [`Error::ElementDecodeInvalid`] gives the
/// caller a single, element-scoped failure mode.
fn validate_ms_used(ms_used: &[Vec<bool>], ics_info: &IcsInfo) -> Result<()> {
    let num_groups = ics_info.num_window_groups as usize;
    let max_sfb = ics_info.max_sfb as usize;
    if ms_used.len() != num_groups {
        return Err(Error::ElementDecodeInvalid);
    }
    for row in ms_used {
        if row.len() < max_sfb {
            return Err(Error::ElementDecodeInvalid);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ics_info::{WindowSequence, WindowShape};
    use crate::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
    use crate::section_data::{Section, SectionData, INTENSITY_HCB, NOISE_HCB};

    // ---- band-indexed track expansion ----

    fn sfb_cb_one_group(cbs: &[u8]) -> Vec<Vec<u8>> {
        vec![cbs.to_vec()]
    }

    #[test]
    fn band_indexed_track_scatters_by_wire_order() {
        // Group 0: bands [ZERO, INTENSITY_HCB, NOISE_HCB, spectrum=2].
        // Wire records skip ZERO; so records are
        // [IsPos, NoiseNrg, Sf] for sfb 1, 2, 3.
        let sfb_cb = sfb_cb_one_group(&[ZERO_HCB, INTENSITY_HCB, NOISE_HCB, 2]);
        let abs = AbsoluteScaleFactors {
            entries: vec![vec![
                AbsoluteScaleFactorEntry::IsPos(7),
                AbsoluteScaleFactorEntry::NoiseNrg(42),
                AbsoluteScaleFactorEntry::Sf(120),
            ]],
        };
        let is_pos = is_pos_table(&abs, &sfb_cb, 4).unwrap();
        assert_eq!(is_pos[0], vec![0, 7, 0, 0]);
        let nrg = noise_nrg_table(&abs, &sfb_cb, 4).unwrap();
        assert_eq!(nrg[0], vec![0, 0, 42, 0]);
    }

    #[test]
    fn band_indexed_track_rejects_record_shortfall() {
        // Two non-ZERO bands but only one wire record.
        let sfb_cb = sfb_cb_one_group(&[INTENSITY_HCB, NOISE_HCB]);
        let abs = AbsoluteScaleFactors {
            entries: vec![vec![AbsoluteScaleFactorEntry::IsPos(1)]],
        };
        assert!(matches!(
            is_pos_table(&abs, &sfb_cb, 2),
            Err(Error::ElementDecodeInvalid)
        ));
    }

    #[test]
    fn band_indexed_track_rejects_group_count_mismatch() {
        let sfb_cb = vec![vec![2u8], vec![2u8]];
        let abs = AbsoluteScaleFactors {
            entries: vec![vec![AbsoluteScaleFactorEntry::Sf(100)]],
        };
        assert!(matches!(
            noise_nrg_table(&abs, &sfb_cb, 1),
            Err(Error::ElementDecodeInvalid)
        ));
    }

    // ---- end-to-end element decode ----

    fn long_ics_info(max_sfb: u8) -> IcsInfo {
        IcsInfo {
            ics_reserved_bit: false,
            window_sequence: WindowSequence::OnlyLong,
            window_shape: WindowShape::Sine,
            max_sfb,
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
            num_swb: crate::ics_info::NUM_SWB_LONG_WINDOW[4],
        }
    }

    /// Build a minimal single-group long-window channel body whose
    /// `section_data` assigns codebook `cb` to bands `0..max_sfb` and
    /// whose `scale_factor_data` carries one DPCM record per non-ZERO
    /// band. No pulse / TNS / gain-control tools.
    fn make_body(max_sfb: u8, cb: u8, sf_deltas: &[i16]) -> IcsBody {
        let sfb_cb = vec![vec![cb; max_sfb as usize]];
        let sections = vec![vec![Section {
            codebook: cb,
            start: 0,
            end: max_sfb,
        }]];
        let section_data = SectionData { sections, sfb_cb };
        // For a NOISE_HCB / INTENSITY band the record variant differs;
        // make_body is only used with spectrum books (Dpcm) and the
        // single-noise-band case below, where the first record is the
        // 9-bit PNS PCM seed.
        let entries: Vec<ScaleFactorEntry> = if cb == NOISE_HCB {
            // The first noise band of the frame carries the 9-bit PCM
            // seed; later noise bands carry Huffman DPCM deltas.
            sf_deltas
                .iter()
                .enumerate()
                .map(|(i, &d)| {
                    if i == 0 {
                        ScaleFactorEntry::NoisePcm(d as u16)
                    } else {
                        ScaleFactorEntry::NoiseDpcm(d as i8)
                    }
                })
                .collect()
        } else {
            sf_deltas
                .iter()
                .map(|&d| ScaleFactorEntry::Dpcm(d as i8))
                .collect()
        };
        let scale_factor_data = ScaleFactorData {
            entries: vec![entries],
        };
        IcsBody {
            global_gain: 100,
            ics_info: Some(long_ics_info(max_sfb)),
            section_data,
            scale_factor_data,
            pulse_data_present: false,
            pulse_data: None,
            tns_data_present: false,
            tns_data: None,
            gain_control_data_present: false,
            gain_control_data: None,
            spectral_data_bit_offset: 0,
        }
    }

    /// A spectral-data block with `value` in every coefficient of bands
    /// `0..max_sfb` (long window, fs_index 4: bands are 4 wide at the
    /// low end). Just fills the full 1024-coefficient group buffer.
    fn make_spectral(value: i32) -> SpectralData {
        SpectralData {
            x_quant: vec![vec![value; 1024]],
        }
    }

    #[test]
    fn decode_sce_produces_finite_pcm() {
        let body = make_body(4, 2, &[0, 0, 0, 0]);
        let ics = body.ics_info.clone().unwrap();
        let spectral = make_spectral(3);
        let ch = ChannelInput {
            body: &body,
            ics_info: &ics,
            spectral: &spectral,
        };
        let mut dec = ElementDecoder::new();
        let pcm = dec.decode_sce(&ch, 2, 4).unwrap();
        assert_eq!(pcm.len(), 1024);
        assert!(pcm.iter().all(|v| v.is_finite()));
        // The first frame overlaps against a zero tail, so the right
        // half of the windowed block is folded into the next frame.
        // A constant non-zero spectrum yields non-silent PCM.
        assert!(pcm.iter().any(|&v| v != 0.0));
    }

    #[test]
    fn decode_sce_overlap_couples_frames() {
        let body = make_body(4, 2, &[0, 0, 0, 0]);
        let ics = body.ics_info.clone().unwrap();
        let spectral = make_spectral(3);
        let ch = ChannelInput {
            body: &body,
            ics_info: &ics,
            spectral: &spectral,
        };
        let mut dec = ElementDecoder::new();
        let f0 = dec.decode_sce(&ch, 2, 4).unwrap();
        let f1 = dec.decode_sce(&ch, 2, 4).unwrap();
        // The second frame carries the first frame's overlap tail, so
        // for identical input the two frames differ only by the
        // (now non-zero) overlap contribution at frame start.
        assert_ne!(f0, f1);
    }

    #[test]
    fn decode_cpe_ms_reconstructs_left_right() {
        // common_window: shared ics_info. Channel 0 = mid, channel 1 =
        // side; ms_mask_present = all-ones (10). With a constant
        // spectrum m, s the de-matrix gives l = m + s, r = m - s.
        let left_body = make_body(4, 2, &[0, 0, 0, 0]);
        let right_body = make_body(4, 2, &[0, 0, 0, 0]);
        let ics = left_body.ics_info.clone().unwrap();
        let left_spec = make_spectral(5);
        let right_spec = make_spectral(2);
        let left = ChannelInput {
            body: &left_body,
            ics_info: &ics,
            spectral: &left_spec,
        };
        let right = ChannelInput {
            body: &right_body,
            ics_info: &ics,
            spectral: &right_spec,
        };
        let joint = CpeJointStereo {
            ms_mask_present: MsMaskPresent::AllOnes,
            ms_used: vec![],
        };
        let mut dec = ElementDecoder::new();
        let (l, r) = dec.decode_cpe(&left, &right, &joint, 2, 4).unwrap();
        assert_eq!(l.len(), 1024);
        assert_eq!(r.len(), 1024);
        assert!(l.iter().all(|v| v.is_finite()));
        assert!(r.iter().all(|v| v.is_finite()));
        // The reconstructed channels differ (l = m+s, r = m-s with
        // s != 0), so the PCM frames are not identical.
        assert_ne!(l, r);
    }

    #[test]
    fn decode_cpe_mask_off_is_independent_channels() {
        // ms_mask_present = all-zeros: M/S is a no-op, each channel
        // passes through independently.
        let left_body = make_body(4, 2, &[0, 0, 0, 0]);
        let right_body = make_body(4, 2, &[0, 0, 0, 0]);
        let ics = left_body.ics_info.clone().unwrap();
        let same = make_spectral(4);
        let left = ChannelInput {
            body: &left_body,
            ics_info: &ics,
            spectral: &same,
        };
        let right = ChannelInput {
            body: &right_body,
            ics_info: &ics,
            spectral: &same,
        };
        let joint = CpeJointStereo::default();
        let mut dec = ElementDecoder::new();
        let (l, r) = dec.decode_cpe(&left, &right, &joint, 2, 4).unwrap();
        // Identical input, identical (independent) filterbanks → equal.
        assert_eq!(l, r);
    }

    #[test]
    fn decode_cpe_rejects_window_sequence_mismatch() {
        let left_body = make_body(4, 2, &[0, 0, 0, 0]);
        let mut right_body = make_body(4, 2, &[0, 0, 0, 0]);
        // Give the right channel a different window sequence.
        let mut right_ics = right_body.ics_info.clone().unwrap();
        right_ics.window_sequence = WindowSequence::LongStop;
        right_body.ics_info = Some(right_ics.clone());
        let left_ics = left_body.ics_info.clone().unwrap();
        let left_spec = make_spectral(1);
        let right_spec = make_spectral(1);
        let left = ChannelInput {
            body: &left_body,
            ics_info: &left_ics,
            spectral: &left_spec,
        };
        let right = ChannelInput {
            body: &right_body,
            ics_info: &right_ics,
            spectral: &right_spec,
        };
        let joint = CpeJointStereo::default();
        let mut dec = ElementDecoder::new();
        assert!(matches!(
            dec.decode_cpe(&left, &right, &joint, 2, 4),
            Err(Error::ElementDecodeInvalid)
        ));
    }

    #[test]
    fn decode_sce_synthesizes_noise_band() {
        // A NOISE_HCB band carries no spectrum (silence on entry); PNS
        // fills it to the §4.6.13.3 target norm. With one noise band the
        // decoded PCM must be non-silent.
        let body = make_body(4, NOISE_HCB, &[10, 0, 0, 0]);
        let ics = body.ics_info.clone().unwrap();
        // Noise bands carry no x_quant (spectrum-less); leave zeros.
        let spectral = make_spectral(0);
        let ch = ChannelInput {
            body: &body,
            ics_info: &ics,
            spectral: &spectral,
        };
        let mut dec = ElementDecoder::new();
        let pcm = dec.decode_sce(&ch, 2, 4).unwrap();
        assert!(pcm.iter().all(|v| v.is_finite()));
        assert!(
            pcm.iter().any(|&v| v != 0.0),
            "PNS-filled noise band should produce non-silent PCM"
        );
    }
}

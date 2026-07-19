//! Scalable AAC decode — ISO/IEC 14496-3 §4.4.2.2 / §4.5.2.2 through
//! [`oxideav_aac::scalable::ScalableDecoder`].
//!
//! Streams are writer-assembled with the crate's own writers
//! ([`ScalableFrame::write`]) and pinned against internal invariants
//! the spec defines exactly:
//!
//! * a single-layer scalable program is the same coding as a plain
//!   SCE / common-window CPE `raw_data_block()` (§4.5.2.2.2 — the
//!   scalable syntax merely hoists `ics_info()` / TNS into the
//!   header), so the PCM must be **bit-identical** to the
//!   [`StreamDecoder`] decode of the equivalent stream;
//! * SIAQ (§4.5.2.2.4) adds dequantized spectra, so an extension
//!   layer of all-`ZERO_HCB` bands must not change one PCM bit, and
//!   a two-layer decode must equal a reference built by summing the
//!   two dequantized spectra through one filterbank;
//! * the §4.6.14.2.1 FSS / M/S merge at the mono→stereo transition
//!   follows `L/R += 2·M''` (diff bit 0) / `M = M'' + M'` exactly;
//! * Tables 4.91–4.93 invalid tool combinations surface
//!   [`Error::ScalableLayerCombination`].

use oxideav_aac::asc::AacResilienceFlags;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::ms_stereo::MsMaskPresent;
use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
use oxideav_aac::scalable::{
    ScalableChannel, ScalableConfig, ScalableDecoder, ScalableFrame, ScalableLayer,
};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::swb_offset::FrameFamily;
use oxideav_aac::Error;
use oxideav_core::bits::BitWriter;

const FS_INDEX: u8 = 4; // 44.1 kHz
const SAMPLE_RATE: u32 = 44100;
const AOT_SCALABLE: u8 = 6;
const AOT_LC: u8 = 2;

fn long_ics_info(max_sfb: u8) -> IcsInfo {
    IcsInfo {
        family: FrameFamily::Lc1024,
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
        num_swb: NUM_SWB_LONG_WINDOW[FS_INDEX as usize],
    }
}

/// One long-window channel: per-band codebooks (one section per band)
/// with a bounded pseudo-random spectrum, plus per-noise-band DPCM 0
/// energies.
fn make_channel(sfb_cbs: &[u8], global_gain: u8, seed: u32) -> ScalableChannel {
    let sections = vec![sfb_cbs
        .iter()
        .enumerate()
        .map(|(sfb, &cb)| Section {
            codebook: cb,
            start: sfb as u8,
            end: sfb as u8 + 1,
        })
        .collect()];
    let mut first_noise = true;
    let entries: Vec<ScaleFactorEntry> = sfb_cbs
        .iter()
        .filter(|&&cb| cb != 0)
        .map(|&cb| match cb {
            13 => {
                if first_noise {
                    first_noise = false;
                    ScaleFactorEntry::NoisePcm(256)
                } else {
                    ScaleFactorEntry::NoiseDpcm(0)
                }
            }
            14 | 15 => ScaleFactorEntry::Intensity(0),
            _ => ScaleFactorEntry::Dpcm(0),
        })
        .collect();
    let body = IcsBody {
        global_gain,
        ics_info: None,
        section_data: SectionData {
            sections,
            sfb_cb: vec![sfb_cbs.to_vec()],
        },
        scale_factor_data: ScaleFactorData {
            entries: vec![entries],
        },
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    let offsets = oxideav_aac::swb_offset::long_window_offsets(FS_INDEX).unwrap();
    let mut coeffs = vec![0i32; 1024];
    let mut state = seed;
    let mut prand = |max: i32| {
        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        ((state >> 8) % (2 * max + 1) as u32) as i32 - max
    };
    for (sfb, &cb) in sfb_cbs.iter().enumerate() {
        let max = match cb {
            0 | 13 | 14 | 15 => 0,
            1 | 2 => 1,
            3 | 4 => 2,
            5 | 6 => 4,
            7 | 8 => 7,
            9 | 10 => 12,
            _ => 30,
        };
        if max > 0 {
            let (a, b) = (usize::from(offsets[sfb]), usize::from(offsets[sfb + 1]));
            for c in coeffs[a..b].iter_mut() {
                *c = prand(max);
            }
        }
    }
    ScalableChannel {
        body,
        spectral: SpectralData {
            x_quant: vec![coeffs],
        },
    }
}

fn mono_cfg(n_layers: usize) -> ScalableConfig {
    ScalableConfig {
        aot: AOT_SCALABLE,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family: FrameFamily::Lc1024,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![false; n_layers],
    }
}

fn layer(ics: IcsInfo, channels: Vec<ScalableChannel>) -> ScalableLayer {
    let n = channels.len();
    ScalableLayer {
        ics,
        ms_mask_present: MsMaskPresent::AllZeros,
        ms_used_new: Vec::new(),
        tns: vec![None; n],
        ltp: vec![None; n],
        diff_lr_long: Vec::new(),
        diff_lr_short: vec![None; n],
        channels,
    }
}

/// The equivalent non-scalable `raw_data_block()` ([SCE, END]) for a
/// mono channel.
fn sce_block(chan: &ScalableChannel, ics: &IcsInfo) -> Vec<u8> {
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let mut bw = BitWriter::new();
    let mut body = chan.body.clone();
    body.ics_info = Some(ics.clone());
    body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
    chan.spectral
        .write(&mut bw, ics, &chan.body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    fa.push_end()
}

/// The equivalent non-scalable common-window CPE `raw_data_block()`.
fn cpe_block(
    left: &ScalableChannel,
    right: &ScalableChannel,
    ics: &IcsInfo,
    ms_mask_present: MsMaskPresent,
    ms_used: &[Vec<bool>],
) -> Vec<u8> {
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Cpe, 0).unwrap();
    let mut bw = BitWriter::new();
    bw.write_bit(true); // common_window
    ics.write(&mut bw, AOT_LC, FS_INDEX, true).unwrap();
    bw.write_u32(u32::from(ms_mask_present.to_bits()), 2);
    if ms_mask_present == MsMaskPresent::Mask {
        for row in ms_used {
            for &b in row.iter().take(usize::from(ics.max_sfb)) {
                bw.write_bit(b);
            }
        }
    }
    for chan in [left, right] {
        chan.body
            .write_with_ics_info(&mut bw, ics, AOT_LC, false)
            .unwrap();
        chan.spectral
            .write(&mut bw, ics, &chan.body.section_data, FS_INDEX)
            .unwrap();
    }
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    fa.push_end()
}

/// A single-layer mono scalable program decodes bit-identically to
/// the plain SCE coding of the same body + spectrum (§4.5.2.2.2),
/// across frames (shared filterbank state).
#[test]
fn single_layer_mono_matches_sce() {
    let cfg = mono_cfg(1);
    let ics = long_ics_info(8);
    let chan = make_channel(&[1, 3, 5, 7, 9, 11, 11, 2], 150, 0xA11);
    let frame = ScalableFrame {
        layers: vec![layer(ics.clone(), vec![chan.clone()])],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 8,
        max_mono_sfb: 8,
    };
    let payloads = frame.write(&cfg).unwrap();
    let plain = sce_block(&chan, &ics);

    let mut dec = ScalableDecoder::new(cfg).unwrap();
    let mut reference = StreamDecoder::new();
    for f in 0..3 {
        let got = dec
            .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let want = reference
            .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 1, 1, &plain)
            .unwrap();
        assert_eq!(got.channels, 1);
        assert_eq!(got.sample_rate, SAMPLE_RATE);
        assert_eq!(got.pcm, want.pcm, "frame {f}");
        assert!(got.pcm.iter().any(|&s| s != 0), "frame {f} silent");
    }
}

/// A single-layer stereo scalable program with a per-band M/S mask
/// decodes bit-identically to the equivalent common-window CPE.
#[test]
fn single_layer_stereo_matches_cpe() {
    let mut cfg = mono_cfg(1);
    cfg.layer_stereo = vec![true];
    let ics = long_ics_info(8);
    let left = make_channel(&[1, 3, 5, 7, 9, 11, 2, 2], 148, 0xB22);
    let right = make_channel(&[2, 4, 6, 8, 10, 11, 1, 1], 152, 0xC33);
    let ms_used = vec![vec![true, false, true, false, true, false, true, false]];
    let frame = ScalableFrame {
        layers: vec![ScalableLayer {
            ics: ics.clone(),
            ms_mask_present: MsMaskPresent::Mask,
            ms_used_new: ms_used.clone(),
            tns: vec![None, None],
            ltp: vec![None, None],
            diff_lr_long: Vec::new(),
            diff_lr_short: vec![None, None],
            channels: vec![left.clone(), right.clone()],
        }],
        ms_used: ms_used.clone(),
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 8,
        max_mono_sfb: 0,
    };
    let payloads = frame.write(&cfg).unwrap();
    let plain = cpe_block(&left, &right, &ics, MsMaskPresent::Mask, &ms_used);

    let mut dec = ScalableDecoder::new(cfg).unwrap();
    let mut reference = StreamDecoder::new();
    for f in 0..3 {
        let got = dec
            .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let want = reference
            .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 2, 1, &plain)
            .unwrap();
        assert_eq!(got.channels, 2);
        assert_eq!(got.pcm, want.pcm, "frame {f}");
        assert!(got.pcm.iter().any(|&s| s != 0), "frame {f} silent");
    }
}

/// An all-`ZERO_HCB` extension layer adds nothing: the two-layer
/// decode is bit-identical to the single-layer decode.
#[test]
fn zero_extension_layer_is_transparent() {
    let ics = long_ics_info(8);
    let chan = make_channel(&[1, 3, 5, 7, 9, 11, 11, 2], 150, 0xD44);
    let zero = make_channel(&[0, 0, 0, 0, 0, 0, 0, 0], 100, 0);

    let frame1 = ScalableFrame {
        layers: vec![layer(ics.clone(), vec![chan.clone()])],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 8,
        max_mono_sfb: 8,
    };
    let frame2 = ScalableFrame {
        layers: vec![
            layer(ics.clone(), vec![chan.clone()]),
            layer(ics.clone(), vec![zero]),
        ],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 8,
        max_mono_sfb: 8,
    };
    let p1 = frame1.write(&mono_cfg(1)).unwrap();
    let p2 = frame2.write(&mono_cfg(2)).unwrap();

    let mut d1 = ScalableDecoder::new(mono_cfg(1)).unwrap();
    let mut d2 = ScalableDecoder::new(mono_cfg(2)).unwrap();
    for f in 0..3 {
        let a = d1
            .decode_frame(&p1.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let b = d2
            .decode_frame(&p2.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        assert_eq!(a.pcm, b.pcm, "frame {f}");
    }
}

/// Two mono layers with real content: the SIAQ output equals the sum
/// of the two dequantized spectra through one filterbank — built
/// here from the crate's own reconstruction primitives.
#[test]
fn two_mono_layers_sum_spectra() {
    let ics = long_ics_info(6);
    let l0 = make_channel(&[1, 3, 5, 7, 9, 11], 150, 0xE55);
    let l1 = make_channel(&[2, 4, 6, 8, 10, 11], 146, 0xF66);

    let frame = ScalableFrame {
        layers: vec![
            layer(ics.clone(), vec![l0.clone()]),
            layer(ics.clone(), vec![l1.clone()]),
        ],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 6,
        max_mono_sfb: 6,
    };
    let payloads = frame.write(&mono_cfg(2)).unwrap();

    // Reference: dequantize both layers with the crate primitives and
    // run one filterbank over the summed spectrum.
    let reconstruct = |chan: &ScalableChannel| -> Vec<f64> {
        let abs = oxideav_aac::scale_factor_data::accumulate(
            &chan.body.scale_factor_data,
            &chan.body.section_data.sfb_cb,
            chan.body.global_gain,
        )
        .unwrap();
        let rescaled = oxideav_aac::dequant::rescale_spectrum(
            &chan.spectral,
            &abs,
            &chan.body.section_data.sfb_cb,
            &ics,
            FS_INDEX,
        )
        .unwrap();
        oxideav_aac::decoded_spectrum::quant_to_spec(&rescaled, &ics, FS_INDEX).unwrap()
    };
    let s0 = reconstruct(&l0);
    let s1 = reconstruct(&l1);
    let summed: Vec<f64> = s0.iter().zip(&s1).map(|(a, b)| a + b).collect();
    let mut fb = oxideav_aac::filterbank::Filterbank::new();
    let mut dec = ScalableDecoder::new(mono_cfg(2)).unwrap();
    for f in 0..3 {
        let want_f64 = fb.synthesize(&summed, &ics).unwrap();
        let want = oxideav_aac::pcm::channel_to_s16(&want_f64);
        let got = dec
            .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        assert_eq!(got.pcm, want, "frame {f}");
    }
}

/// Tables 4.91: a plain-coded mono band overwritten with PNS in a
/// higher layer is an invalid combination.
#[test]
fn plain_then_pns_is_invalid() {
    let ics = long_ics_info(4);
    let l0 = make_channel(&[1, 3, 5, 7], 150, 0x111);
    let l1 = make_channel(&[2, 13, 6, 8], 146, 0x222);
    let frame = ScalableFrame {
        layers: vec![layer(ics.clone(), vec![l0]), layer(ics.clone(), vec![l1])],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 4,
        max_mono_sfb: 4,
    };
    let payloads = frame.write(&mono_cfg(2)).unwrap();
    let mut dec = ScalableDecoder::new(mono_cfg(2)).unwrap();
    let err = dec
        .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
        .unwrap_err();
    assert_eq!(err, Error::ScalableLayerCombination);
}

/// §4.6.13.6: a lower-layer PNS band survives an all-zero higher
/// layer (noise still generated) but is cancelled by non-zero
/// higher-layer content (the decode then equals the same stream with
/// the noise band never sent).
#[test]
fn pns_survival_rules() {
    let ics = long_ics_info(4);
    // Layer 0: band 1 is PNS. Layer 1: band 1 all-zero (ZERO_HCB).
    let l0 = make_channel(&[1, 13, 5, 7], 150, 0x333);
    let l1_zero = make_channel(&[2, 0, 6, 8], 146, 0x444);
    let frame_zero = ScalableFrame {
        layers: vec![
            layer(ics.clone(), vec![l0.clone()]),
            layer(ics.clone(), vec![l1_zero]),
        ],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 4,
        max_mono_sfb: 4,
    };
    let payloads = frame_zero.write(&mono_cfg(2)).unwrap();
    let mut dec = ScalableDecoder::new(mono_cfg(2)).unwrap();
    let with_noise = dec
        .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
        .unwrap();

    // Same layers but the noise band silenced in layer 0: the outputs
    // must differ (the surviving noise band carries energy).
    let l0_silent = make_channel(&[1, 0, 5, 7], 150, 0x333);
    let frame_silent = ScalableFrame {
        layers: vec![
            layer(ics.clone(), vec![l0_silent.clone()]),
            layer(ics.clone(), vec![make_channel(&[2, 0, 6, 8], 146, 0x444)]),
        ],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 4,
        max_mono_sfb: 4,
    };
    let p_silent = frame_silent.write(&mono_cfg(2)).unwrap();
    let mut dec2 = ScalableDecoder::new(mono_cfg(2)).unwrap();
    let without_noise = dec2
        .decode_frame(&p_silent.iter().map(Vec::as_slice).collect::<Vec<_>>())
        .unwrap();
    assert_ne!(with_noise.pcm, without_noise.pcm, "noise band was dropped");

    // Layer 1 carries real content in the band: the noise is
    // cancelled, so the decode equals the same stream with layer 0's
    // band never sent.
    let l1_content = make_channel(&[2, 4, 6, 8], 146, 0x555);
    let frame_cancel = ScalableFrame {
        layers: vec![
            layer(ics.clone(), vec![l0]),
            layer(ics.clone(), vec![l1_content.clone()]),
        ],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 4,
        max_mono_sfb: 4,
    };
    let frame_ref = ScalableFrame {
        layers: vec![
            layer(ics.clone(), vec![l0_silent]),
            layer(ics.clone(), vec![l1_content]),
        ],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 4,
        max_mono_sfb: 4,
    };
    let p_cancel = frame_cancel.write(&mono_cfg(2)).unwrap();
    let p_ref = frame_ref.write(&mono_cfg(2)).unwrap();
    let mut da = ScalableDecoder::new(mono_cfg(2)).unwrap();
    let mut db = ScalableDecoder::new(mono_cfg(2)).unwrap();
    let a = da
        .decode_frame(&p_cancel.iter().map(Vec::as_slice).collect::<Vec<_>>())
        .unwrap();
    let b = db
        .decode_frame(&p_ref.iter().map(Vec::as_slice).collect::<Vec<_>>())
        .unwrap();
    assert_eq!(a.pcm, b.pcm);
}

/// The §4.6.14.2.1 mono→stereo FSS: with the L diff bit 0 and the R
/// bit 1 on a plain band, `L = L' + 2·M''` and `R = R'`; an M/S band
/// takes `M = M'' + M'`. Pinned against a reference assembled from
/// the crate's reconstruction primitives.
#[test]
fn mono_stereo_fss_merge() {
    let ics = long_ics_info(4);
    let mono = make_channel(&[1, 3, 5, 7], 150, 0x666);
    let left = make_channel(&[2, 4, 6, 8], 148, 0x777);
    let right = make_channel(&[1, 3, 5, 7], 149, 0x888);
    // Band 2 is M/S-coded in the stereo layer; the other bands are
    // plain L/R with diff bits L=0 (merge) / R=1 (keep).
    let ms_used = vec![vec![false, false, true, false]];
    let cfg = ScalableConfig {
        aot: AOT_SCALABLE,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family: FrameFamily::Lc1024,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![false, true],
    };
    let diff_l = vec![false, false, false]; // bands 0, 1, 3 (band 2 is ms)
    let diff_r = vec![true, true, true];
    let mut diff_lr_long: [Vec<Option<bool>>; 2] = [vec![None; 4], vec![None; 4]];
    for (i, sfb) in [0usize, 1, 3].iter().enumerate() {
        diff_lr_long[0][*sfb] = Some(diff_l[i]);
        diff_lr_long[1][*sfb] = Some(diff_r[i]);
    }
    let frame = ScalableFrame {
        layers: vec![
            layer(ics.clone(), vec![mono.clone()]),
            ScalableLayer {
                ics: ics.clone(),
                ms_mask_present: MsMaskPresent::Mask,
                ms_used_new: ms_used.clone(),
                tns: vec![None, None],
                ltp: vec![None, None],
                diff_lr_long: vec![diff_l.clone(), diff_r.clone()],
                diff_lr_short: vec![None, None],
                channels: vec![left.clone(), right.clone()],
            },
        ],
        ms_used: ms_used.clone(),
        diff_lr_long,
        diff_lr_short: [None, None],
        max_total_sfb: 4,
        max_mono_sfb: 4,
    };
    let payloads = frame.write(&cfg).unwrap();

    // Reference: manual §4.5.2.2.4 combination.
    let reconstruct = |chan: &ScalableChannel| -> Vec<f64> {
        let abs = oxideav_aac::scale_factor_data::accumulate(
            &chan.body.scale_factor_data,
            &chan.body.section_data.sfb_cb,
            chan.body.global_gain,
        )
        .unwrap();
        let rescaled = oxideav_aac::dequant::rescale_spectrum(
            &chan.spectral,
            &abs,
            &chan.body.section_data.sfb_cb,
            &ics,
            FS_INDEX,
        )
        .unwrap();
        oxideav_aac::decoded_spectrum::quant_to_spec(&rescaled, &ics, FS_INDEX).unwrap()
    };
    let m = reconstruct(&mono);
    let lp = reconstruct(&left);
    let rp = reconstruct(&right);
    let offsets = oxideav_aac::swb_offset::long_window_offsets(FS_INDEX).unwrap();
    let mut l_spec = vec![0.0f64; 1024];
    let mut r_spec = vec![0.0f64; 1024];
    for sfb in 0..4usize {
        let (a, b) = (usize::from(offsets[sfb]), usize::from(offsets[sfb + 1]));
        if sfb == 2 {
            // M/S band: m = M'' + M', s = S; butterfly l = m+s, r = m-s.
            for i in a..b {
                let mm = m[i] + lp[i];
                let ss = rp[i];
                l_spec[i] = mm + ss;
                r_spec[i] = mm - ss;
            }
        } else {
            for i in a..b {
                l_spec[i] = lp[i] + 2.0 * m[i]; // diff bit 0
                r_spec[i] = rp[i]; // diff bit 1
            }
        }
    }
    let mut fb_l = oxideav_aac::filterbank::Filterbank::new();
    let mut fb_r = oxideav_aac::filterbank::Filterbank::new();
    let mut dec = ScalableDecoder::new(cfg).unwrap();
    for f in 0..3 {
        let want_l = oxideav_aac::pcm::channel_to_s16(&fb_l.synthesize(&l_spec, &ics).unwrap());
        let want_r = oxideav_aac::pcm::channel_to_s16(&fb_r.synthesize(&r_spec, &ics).unwrap());
        let got = dec
            .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        assert_eq!(got.channels, 2);
        let got_l: Vec<i16> = got.pcm.iter().step_by(2).copied().collect();
        let got_r: Vec<i16> = got.pcm.iter().skip(1).step_by(2).copied().collect();
        assert_eq!(got_l, want_l, "frame {f} left");
        assert_eq!(got_r, want_r, "frame {f} right");
    }
}

/// Frame parse ↔ write round-trip over a mixed mono/stereo layer
/// stack: re-emitting the parsed frame reproduces the payload bytes.
#[test]
fn frame_write_parse_roundtrip() {
    let ics = long_ics_info(4);
    let cfg = ScalableConfig {
        aot: AOT_SCALABLE,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family: FrameFamily::Lc1024,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![false, true],
    };
    let ms_used = vec![vec![false, true, false, true]];
    let frame = ScalableFrame {
        layers: vec![
            layer(ics.clone(), vec![make_channel(&[1, 3, 5, 7], 150, 0x999)]),
            ScalableLayer {
                ics: ics.clone(),
                ms_mask_present: MsMaskPresent::Mask,
                ms_used_new: ms_used.clone(),
                tns: vec![None, None],
                ltp: vec![None, None],
                diff_lr_long: vec![vec![true, false], vec![false, true]],
                diff_lr_short: vec![None, None],
                channels: vec![
                    make_channel(&[2, 4, 6, 8], 148, 0xAAA),
                    make_channel(&[1, 3, 5, 7], 149, 0xBBB),
                ],
            },
        ],
        ms_used: ms_used.clone(),
        diff_lr_long: [
            vec![Some(true), None, Some(false)],
            vec![Some(false), None, Some(true)],
        ],
        diff_lr_short: [None, None],
        max_total_sfb: 4,
        max_mono_sfb: 4,
    };
    let payloads = frame.write(&cfg).unwrap();
    let parsed = ScalableFrame::parse(
        &cfg,
        &payloads.iter().map(Vec::as_slice).collect::<Vec<_>>(),
    )
    .unwrap();
    assert_eq!(parsed.ms_used, frame.ms_used);
    assert_eq!(parsed.max_total_sfb, 4);
    assert_eq!(parsed.max_mono_sfb, 4);
    assert_eq!(parsed.diff_lr_long, frame.diff_lr_long);
    let rewritten = parsed.write(&cfg).unwrap();
    assert_eq!(rewritten, payloads);
}

/// Configuration validation: a mono layer after a stereo layer, an
/// oversized stack and a payload-count mismatch are rejected.
#[test]
fn config_validation() {
    let mut cfg = mono_cfg(2);
    cfg.layer_stereo = vec![true, false];
    assert_eq!(
        ScalableDecoder::new(cfg).unwrap_err(),
        Error::ScalableInvalid
    );

    let cfg9 = mono_cfg(9);
    assert_eq!(
        ScalableDecoder::new(cfg9).unwrap_err(),
        Error::ScalableInvalid
    );

    let cfg = mono_cfg(2);
    let mut dec = ScalableDecoder::new(cfg).unwrap();
    // One payload for a two-layer config.
    assert_eq!(
        dec.decode_frame(&[&[0u8; 8][..]]).unwrap_err(),
        Error::ScalableInvalid
    );
}

//! Scalable AAC per-tool layering — the Table 4.92 stereo-stereo
//! rules, the §4.6.9.5 / Table 4.158 serial TNS layout, §4.6.7.5
//! base-layer LTP, the `EIGHT_SHORT` and 960-line geometries.

use oxideav_aac::asc::AacResilienceFlags;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{
    derive_window_grouping_family, IcsInfo, LtpData, WindowSequence, WindowShape,
    NUM_SWB_LONG_WINDOW,
};
use oxideav_aac::ms_stereo::MsMaskPresent;
use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
use oxideav_aac::scalable::{
    ScalableChannel, ScalableConfig, ScalableDecoder, ScalableFrame, ScalableLayer,
};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::swb_offset::FrameFamily;
use oxideav_aac::tns_data::{TnsData, TnsFilter, TnsWindow};
use oxideav_aac::Error;
use oxideav_core::bits::BitWriter;

const FS_INDEX: u8 = 4; // 44.1 kHz
const SAMPLE_RATE: u32 = 44100;
const AOT_SCALABLE: u8 = 6;
const AOT_LC: u8 = 2;
const AOT_LTP: u8 = 4;

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

fn make_channel_family(
    family: FrameFamily,
    ics: &IcsInfo,
    sfb_cbs_per_group: &[Vec<u8>],
    global_gain: u8,
    seed: u32,
) -> ScalableChannel {
    let num_groups = usize::from(ics.num_window_groups);
    assert_eq!(sfb_cbs_per_group.len(), num_groups);
    let mut sections = Vec::with_capacity(num_groups);
    let mut entries_all = Vec::with_capacity(num_groups);
    for cbs in sfb_cbs_per_group {
        let secs: Vec<Section> = cbs
            .iter()
            .enumerate()
            .map(|(sfb, &cb)| Section {
                codebook: cb,
                start: sfb as u8,
                end: sfb as u8 + 1,
            })
            .collect();
        sections.push(secs);
        let mut first_noise = true;
        let entries: Vec<ScaleFactorEntry> = cbs
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
        entries_all.push(entries);
    }
    let body = IcsBody {
        global_gain,
        ics_info: None,
        section_data: SectionData {
            sections,
            sfb_cb: sfb_cbs_per_group.to_vec(),
        },
        scale_factor_data: ScaleFactorData {
            entries: entries_all,
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
    let offsets: &[u16] = ics.swb_offsets(FS_INDEX).unwrap();
    let window_len = if ics.window_sequence.is_eight_short() {
        family.short_window_len().unwrap()
    } else {
        family.frame_len()
    };
    let mut state = seed;
    let mut prand = |max: i32| {
        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        ((state >> 8) % (2 * max + 1) as u32) as i32 - max
    };
    let mut x_quant = Vec::with_capacity(num_groups);
    for (g, cbs) in sfb_cbs_per_group.iter().enumerate() {
        let wgl = usize::from(ics.window_group_length[g]);
        let mut buf = vec![0i32; wgl * window_len];
        // §4.5.2.3.5 group-interleaved layout: per band, wgl runs of
        // the band width.
        let mut pos = 0usize;
        for (sfb, &cb) in cbs.iter().enumerate() {
            let width = usize::from(offsets[sfb + 1] - offsets[sfb]);
            let max = match cb {
                0 | 13 | 14 | 15 => 0,
                1 | 2 => 1,
                3 | 4 => 2,
                5 | 6 => 4,
                7 | 8 => 7,
                9 | 10 => 12,
                _ => 30,
            };
            for _w in 0..wgl {
                for i in 0..width {
                    if max > 0 {
                        buf[pos + i] = prand(max);
                    }
                }
                pos += width;
            }
        }
        x_quant.push(buf);
    }
    ScalableChannel {
        body,
        spectral: SpectralData { x_quant },
    }
}

fn make_channel(sfb_cbs: &[u8], global_gain: u8, seed: u32) -> ScalableChannel {
    let ics = long_ics_info(sfb_cbs.len() as u8);
    make_channel_family(
        FrameFamily::Lc1024,
        &ics,
        &[sfb_cbs.to_vec()],
        global_gain,
        seed,
    )
}

fn stereo_cfg(n_layers: usize) -> ScalableConfig {
    ScalableConfig {
        aot: AOT_SCALABLE,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family: FrameFamily::Lc1024,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![true; n_layers],
    }
}

fn plain_layer(ics: IcsInfo, channels: Vec<ScalableChannel>) -> ScalableLayer {
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

fn stereo_frame(layers: Vec<ScalableLayer>, max_sfb: u8) -> ScalableFrame {
    ScalableFrame {
        layers,
        ms_used: vec![vec![false; usize::from(max_sfb)]],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: max_sfb,
        max_mono_sfb: 0,
    }
}

fn reconstruct(chan: &ScalableChannel, ics: &IcsInfo) -> Vec<f64> {
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
        ics,
        FS_INDEX,
    )
    .unwrap();
    oxideav_aac::decoded_spectrum::quant_to_spec(&rescaled, ics, FS_INDEX).unwrap()
}

/// Table 4.92 IS → IS: the left/mid channel sums across layers, the
/// positions come from the highest layer; IS → plain replaces the
/// band with the highest layer's content.
#[test]
fn stereo_intensity_layering() {
    let ics = long_ics_info(4);
    // Band 3 is intensity in both layers; the position of layer 1
    // (Intensity DPCM 0 → position 0) applies to the summed left.
    let l0_l = make_channel(&[1, 3, 5, 7], 150, 0x10);
    let l0_r = make_channel(&[2, 4, 6, 15], 149, 0x20);
    let l1_l = make_channel(&[2, 4, 6, 8], 148, 0x30);
    let l1_r = make_channel(&[1, 3, 5, 15], 147, 0x40);
    let frame = stereo_frame(
        vec![
            plain_layer(ics.clone(), vec![l0_l.clone(), l0_r.clone()]),
            plain_layer(ics.clone(), vec![l1_l.clone(), l1_r.clone()]),
        ],
        4,
    );
    let cfg = stereo_cfg(2);
    let payloads = frame.write(&cfg).unwrap();

    // Reference: bands 0..3 sum per channel; band 3 sums on the left
    // and mirrors onto the right with scale 0.5^(0.25·0) = 1 (in
    // phase, position 0).
    let s0l = reconstruct(&l0_l, &ics);
    let s0r = reconstruct(&l0_r, &ics);
    let s1l = reconstruct(&l1_l, &ics);
    let s1r = reconstruct(&l1_r, &ics);
    let offsets = oxideav_aac::swb_offset::long_window_offsets(FS_INDEX).unwrap();
    let mut l_spec = vec![0.0f64; 1024];
    let mut r_spec = vec![0.0f64; 1024];
    for sfb in 0..4usize {
        let (a, b) = (usize::from(offsets[sfb]), usize::from(offsets[sfb + 1]));
        for i in a..b {
            l_spec[i] = s0l[i] + s1l[i];
            r_spec[i] = if sfb == 3 {
                l_spec[i] // in-phase intensity, position 0
            } else {
                s0r[i] + s1r[i]
            };
        }
    }
    let mut fb_l = oxideav_aac::filterbank::Filterbank::new();
    let mut fb_r = oxideav_aac::filterbank::Filterbank::new();
    let mut dec = ScalableDecoder::new(cfg).unwrap();
    for f in 0..2 {
        let want_l = oxideav_aac::pcm::channel_to_s16(&fb_l.synthesize(&l_spec, &ics).unwrap());
        let want_r = oxideav_aac::pcm::channel_to_s16(&fb_r.synthesize(&r_spec, &ics).unwrap());
        let got = dec
            .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let got_l: Vec<i16> = got.pcm.iter().step_by(2).copied().collect();
        let got_r: Vec<i16> = got.pcm.iter().skip(1).step_by(2).copied().collect();
        assert_eq!(got_l, want_l, "frame {f} left");
        assert_eq!(got_r, want_r, "frame {f} right");
    }

    // IS → plain: layer 1 replaces band 3 outright.
    let l1_r_plain = make_channel(&[1, 3, 5, 7], 147, 0x50);
    let frame2 = stereo_frame(
        vec![
            plain_layer(ics.clone(), vec![l0_l.clone(), l0_r.clone()]),
            plain_layer(ics.clone(), vec![l1_l.clone(), l1_r_plain.clone()]),
        ],
        4,
    );
    let cfg2 = stereo_cfg(2);
    let payloads2 = frame2.write(&cfg2).unwrap();
    let s1r_p = reconstruct(&l1_r_plain, &ics);
    let mut l_spec2 = vec![0.0f64; 1024];
    let mut r_spec2 = vec![0.0f64; 1024];
    for sfb in 0..4usize {
        let (a, b) = (usize::from(offsets[sfb]), usize::from(offsets[sfb + 1]));
        for i in a..b {
            if sfb == 3 {
                // Intensity → No Tool: layer N+1 only.
                l_spec2[i] = s1l[i];
                r_spec2[i] = s1r_p[i];
            } else {
                l_spec2[i] = s0l[i] + s1l[i];
                r_spec2[i] = s0r[i] + s1r_p[i];
            }
        }
    }
    let mut fb_l2 = oxideav_aac::filterbank::Filterbank::new();
    let mut fb_r2 = oxideav_aac::filterbank::Filterbank::new();
    let mut dec2 = ScalableDecoder::new(cfg2).unwrap();
    let want_l = oxideav_aac::pcm::channel_to_s16(&fb_l2.synthesize(&l_spec2, &ics).unwrap());
    let want_r = oxideav_aac::pcm::channel_to_s16(&fb_r2.synthesize(&r_spec2, &ics).unwrap());
    let got = dec2
        .decode_frame(&payloads2.iter().map(Vec::as_slice).collect::<Vec<_>>())
        .unwrap();
    let got_l: Vec<i16> = got.pcm.iter().step_by(2).copied().collect();
    let got_r: Vec<i16> = got.pcm.iter().skip(1).step_by(2).copied().collect();
    assert_eq!(got_l, want_l);
    assert_eq!(got_r, want_r);
}

/// Table 4.92: intensity over a plain-coded stereo band is invalid.
#[test]
fn plain_then_intensity_is_invalid() {
    let ics = long_ics_info(4);
    let frame = stereo_frame(
        vec![
            plain_layer(
                ics.clone(),
                vec![
                    make_channel(&[1, 3, 5, 7], 150, 0x60),
                    make_channel(&[2, 4, 6, 8], 149, 0x70),
                ],
            ),
            plain_layer(
                ics.clone(),
                vec![
                    make_channel(&[2, 4, 6, 8], 148, 0x80),
                    make_channel(&[1, 3, 5, 15], 147, 0x90),
                ],
            ),
        ],
        4,
    );
    let cfg = stereo_cfg(2);
    let payloads = frame.write(&cfg).unwrap();
    let mut dec = ScalableDecoder::new(cfg).unwrap();
    assert_eq!(
        dec.decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap_err(),
        Error::ScalableLayerCombination
    );
}

/// A single-layer mono scalable stream with TNS in the header decodes
/// bit-identically to the plain SCE carrying the same `tns_data()` in
/// its body (§4.6.9.5: the filter data merely moves into the header).
#[test]
fn header_tns_matches_sce_tns() {
    let ics = long_ics_info(8);
    let chan = make_channel(&[1, 3, 5, 7, 9, 11, 11, 2], 150, 0xA0);
    let tns = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                length: 4,
                order: 2,
                direction: false,
                coef_compress: false,
                coef: vec![3, 5],
            }],
        }],
    };
    let cfg = ScalableConfig {
        aot: AOT_SCALABLE,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family: FrameFamily::Lc1024,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![false],
    };
    let frame = ScalableFrame {
        layers: vec![ScalableLayer {
            ics: ics.clone(),
            ms_mask_present: MsMaskPresent::AllZeros,
            ms_used_new: Vec::new(),
            tns: vec![Some(tns.clone())],
            ltp: vec![None],
            diff_lr_long: Vec::new(),
            diff_lr_short: vec![None],
            channels: vec![chan.clone()],
        }],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 8,
        max_mono_sfb: 8,
    };
    let payloads = frame.write(&cfg).unwrap();

    // Reference: plain SCE with the same TNS in the body.
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let mut bw = BitWriter::new();
    let mut body = chan.body.clone();
    body.ics_info = Some(ics.clone());
    body.tns_data_present = true;
    body.tns_data = Some(tns);
    body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
    chan.spectral
        .write(&mut bw, &ics, &chan.body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    let plain = fa.push_end();

    let mut dec = ScalableDecoder::new(cfg).unwrap();
    let mut reference = StreamDecoder::new();
    for f in 0..3 {
        let got = dec
            .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let want = reference
            .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 1, 1, &plain)
            .unwrap();
        assert_eq!(got.pcm, want.pcm, "frame {f}");
    }
}

/// §4.6.9.5 / Table 4.158 serial TNS: with both an M filter (mono
/// layer) and an L filter (stereo layer) present, the M filter runs
/// first when the L filter stays above the mono boundary and is
/// skipped when the L filter reaches below it; a channel without its
/// own filter takes the M filter.
#[test]
fn serial_tns_layout() {
    let ics_m = long_ics_info(4);
    let ics_s = long_ics_info(8);
    let mono = make_channel(&[1, 3, 5, 7], 150, 0xB0);
    let left = make_channel(&[2, 4, 6, 8, 9, 10, 11, 2], 148, 0xC0);
    let right = make_channel(&[1, 3, 5, 7, 10, 9, 2, 1], 149, 0xD0);
    let tns_m = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                length: 3,
                order: 2,
                direction: false,
                coef_compress: false,
                coef: vec![2, 6],
            }],
        }],
    };
    // L filter covering bands 4..8 — lower boundary 4 >= max_mono 4,
    // so the M filter is NOT overridden.
    let tns_l = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                length: 4,
                order: 1,
                direction: false,
                coef_compress: false,
                coef: vec![5],
            }],
        }],
    };
    let cfg = ScalableConfig {
        aot: AOT_SCALABLE,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family: FrameFamily::Lc1024,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![false, true],
    };
    // All bands L/R-coded, every diff bit 0 (merge everywhere) so the
    // mono content reaches both channels.
    let diff_bits = vec![false, false, false, false];
    let diff_lr_long: [Vec<Option<bool>>; 2] = [vec![Some(false); 4], vec![Some(false); 4]];
    let build = |tns_stereo: [Option<TnsData>; 2]| -> ScalableFrame {
        ScalableFrame {
            layers: vec![
                ScalableLayer {
                    ics: ics_m.clone(),
                    ms_mask_present: MsMaskPresent::AllZeros,
                    ms_used_new: Vec::new(),
                    tns: vec![Some(tns_m.clone())],
                    ltp: vec![None],
                    diff_lr_long: Vec::new(),
                    diff_lr_short: vec![None],
                    channels: vec![mono.clone()],
                },
                ScalableLayer {
                    ics: ics_s.clone(),
                    ms_mask_present: MsMaskPresent::AllZeros,
                    ms_used_new: Vec::new(),
                    tns: tns_stereo.to_vec(),
                    ltp: vec![None, None],
                    diff_lr_long: vec![diff_bits.clone(), diff_bits.clone()],
                    diff_lr_short: vec![None, None],
                    channels: vec![left.clone(), right.clone()],
                },
            ],
            ms_used: vec![vec![false; 8]],
            diff_lr_long: diff_lr_long.clone(),
            diff_lr_short: [None, None],
            max_total_sfb: 8,
            max_mono_sfb: 4,
        }
    };
    let frame = build([Some(tns_l.clone()), None]);
    let payloads = frame.write(&cfg).unwrap();

    // Reference: combined spectra, then the Table 4.158 serial layout
    // via the crate's own TNS primitive.
    let m = reconstruct(&mono, &ics_m);
    let lp = reconstruct(&left, &ics_s);
    let rp = reconstruct(&right, &ics_s);
    let offsets = oxideav_aac::swb_offset::long_window_offsets(FS_INDEX).unwrap();
    let mut l_spec = vec![0.0f64; 1024];
    let mut r_spec = vec![0.0f64; 1024];
    for sfb in 0..8usize {
        let (a, b) = (usize::from(offsets[sfb]), usize::from(offsets[sfb + 1]));
        for i in a..b {
            l_spec[i] = lp[i];
            r_spec[i] = rp[i];
            if sfb < 4 {
                l_spec[i] += 2.0 * m[i];
                r_spec[i] += 2.0 * m[i];
            }
        }
    }
    // L: M filter (boundary rule holds) then L filter. R: M filter.
    oxideav_aac::tns_frame::tns_decode_frame_ics(
        &mut l_spec,
        &tns_m,
        &ics_m,
        AOT_SCALABLE,
        FS_INDEX,
    )
    .unwrap();
    oxideav_aac::tns_frame::tns_decode_frame_ics(
        &mut l_spec,
        &tns_l,
        &ics_s,
        AOT_SCALABLE,
        FS_INDEX,
    )
    .unwrap();
    oxideav_aac::tns_frame::tns_decode_frame_ics(
        &mut r_spec,
        &tns_m,
        &ics_m,
        AOT_SCALABLE,
        FS_INDEX,
    )
    .unwrap();
    let mut fb_l = oxideav_aac::filterbank::Filterbank::new();
    let mut fb_r = oxideav_aac::filterbank::Filterbank::new();
    let want_l = oxideav_aac::pcm::channel_to_s16(&fb_l.synthesize(&l_spec, &ics_s).unwrap());
    let want_r = oxideav_aac::pcm::channel_to_s16(&fb_r.synthesize(&r_spec, &ics_s).unwrap());
    let mut dec = ScalableDecoder::new(cfg.clone()).unwrap();
    let got = dec
        .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
        .unwrap();
    let got_l: Vec<i16> = got.pcm.iter().step_by(2).copied().collect();
    let got_r: Vec<i16> = got.pcm.iter().skip(1).step_by(2).copied().collect();
    assert_eq!(got_l, want_l, "serial M→L left");
    assert_eq!(got_r, want_r, "M-only right");

    // Override case: an L filter reaching below the mono boundary
    // (length 6 → lower boundary 2 < 4) suppresses the M filter.
    let tns_l_deep = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                length: 6,
                order: 1,
                direction: false,
                coef_compress: false,
                coef: vec![5],
            }],
        }],
    };
    let frame2 = build([Some(tns_l_deep.clone()), None]);
    let payloads2 = frame2.write(&cfg).unwrap();
    let mut l_spec2 = vec![0.0f64; 1024];
    let mut r_spec2 = vec![0.0f64; 1024];
    for sfb in 0..8usize {
        let (a, b) = (usize::from(offsets[sfb]), usize::from(offsets[sfb + 1]));
        for i in a..b {
            l_spec2[i] = lp[i];
            r_spec2[i] = rp[i];
            if sfb < 4 {
                l_spec2[i] += 2.0 * m[i];
                r_spec2[i] += 2.0 * m[i];
            }
        }
    }
    // L: only the deep L filter. R: M filter.
    oxideav_aac::tns_frame::tns_decode_frame_ics(
        &mut l_spec2,
        &tns_l_deep,
        &ics_s,
        AOT_SCALABLE,
        FS_INDEX,
    )
    .unwrap();
    oxideav_aac::tns_frame::tns_decode_frame_ics(
        &mut r_spec2,
        &tns_m,
        &ics_m,
        AOT_SCALABLE,
        FS_INDEX,
    )
    .unwrap();
    let mut fb_l2 = oxideav_aac::filterbank::Filterbank::new();
    let mut fb_r2 = oxideav_aac::filterbank::Filterbank::new();
    let want_l2 = oxideav_aac::pcm::channel_to_s16(&fb_l2.synthesize(&l_spec2, &ics_s).unwrap());
    let want_r2 = oxideav_aac::pcm::channel_to_s16(&fb_r2.synthesize(&r_spec2, &ics_s).unwrap());
    let mut dec2 = ScalableDecoder::new(cfg).unwrap();
    let got2 = dec2
        .decode_frame(&payloads2.iter().map(Vec::as_slice).collect::<Vec<_>>())
        .unwrap();
    let got_l2: Vec<i16> = got2.pcm.iter().step_by(2).copied().collect();
    let got_r2: Vec<i16> = got2.pcm.iter().skip(1).step_by(2).copied().collect();
    assert_eq!(got_l2, want_l2, "override left");
    assert_eq!(got_r2, want_r2, "override right");
}

/// §4.6.7.5: a single-layer mono scalable stream with LTP decodes
/// bit-identically to the AAC-LTP (AOT 4) SCE carrying the same
/// `ltp_data()` — and a two-layer stream whose extension is all-zero
/// matches the single-layer decode (the base history is the base
/// output).
#[test]
fn base_layer_ltp() {
    let ics = long_ics_info(8);
    let chan = make_channel(&[1, 3, 5, 7, 9, 11, 11, 2], 150, 0xE0);
    let ltp = LtpData {
        lag_update: None,
        lag: Some(700),
        coef: 4,
        long_used: vec![true; 8],
        short: None,
    };
    let cfg1 = ScalableConfig {
        aot: AOT_SCALABLE,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family: FrameFamily::Lc1024,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![false],
    };
    let mk_frame = |extension: Option<ScalableChannel>| -> ScalableFrame {
        let mut layers = vec![ScalableLayer {
            ics: ics.clone(),
            ms_mask_present: MsMaskPresent::AllZeros,
            ms_used_new: Vec::new(),
            tns: vec![None],
            ltp: vec![Some(ltp.clone())],
            diff_lr_long: Vec::new(),
            diff_lr_short: vec![None],
            channels: vec![chan.clone()],
        }];
        if let Some(ext) = extension {
            layers.push(plain_layer(ics.clone(), vec![ext]));
        }
        ScalableFrame {
            layers,
            ms_used: vec![Vec::new()],
            diff_lr_long: [Vec::new(), Vec::new()],
            diff_lr_short: [None, None],
            max_total_sfb: 8,
            max_mono_sfb: 8,
        }
    };
    let p1 = mk_frame(None).write(&cfg1).unwrap();

    // Reference: AOT-4 SCE with the LTP body inside ics_info.
    let mut ics_ltp = ics.clone();
    ics_ltp.predictor_data_present = true;
    ics_ltp.ltp_data_present = true;
    ics_ltp.ltp_data = Some(ltp.clone());
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let mut bw = BitWriter::new();
    let mut body = chan.body.clone();
    body.ics_info = Some(ics_ltp.clone());
    body.write(&mut bw, AOT_LTP, FS_INDEX, false).unwrap();
    chan.spectral
        .write(&mut bw, &ics_ltp, &chan.body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    let plain = fa.push_end();

    let mut dec1 = ScalableDecoder::new(cfg1).unwrap();
    let mut reference = StreamDecoder::new();
    for f in 0..4 {
        let got = dec1
            .decode_frame(&p1.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let want = reference
            .decode_raw_data_block(AOT_LTP, FS_INDEX, SAMPLE_RATE, 1, 1, &plain)
            .unwrap();
        assert_eq!(got.pcm, want.pcm, "frame {f}");
    }

    // Two layers, extension all-zero: identical to the single-layer
    // decode (the LTP history is fed by the base output either way).
    let cfg2 = ScalableConfig {
        layer_stereo: vec![false, false],
        ..ScalableDecoder::new(ScalableConfig {
            aot: AOT_SCALABLE,
            fs_index: FS_INDEX,
            sample_rate: SAMPLE_RATE,
            family: FrameFamily::Lc1024,
            resilience: AacResilienceFlags::default(),
            layer_stereo: vec![false],
        })
        .unwrap()
        .config()
        .clone()
    };
    let zero = make_channel(&[0, 0, 0, 0, 0, 0, 0, 0], 100, 0);
    let p2 = mk_frame(Some(zero)).write(&cfg2).unwrap();
    let mut dec_a = ScalableDecoder::new(cfg2.clone()).unwrap();
    let mut dec_b = ScalableDecoder::new(ScalableConfig {
        layer_stereo: vec![false],
        ..cfg2.clone()
    })
    .unwrap();
    for f in 0..4 {
        let a = dec_a
            .decode_frame(&p2.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let b = dec_b
            .decode_frame(&p1.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        assert_eq!(a.pcm, b.pcm, "frame {f}");
    }
}

/// §4.6.7.5 history isolation: with a content-bearing extension
/// layer, the LTP prediction must follow the *base* output only — the
/// full-output difference between two streams differing only in the
/// extension layer equals the filterbank image of the extension
/// spectra difference (no prediction feedback).
#[test]
fn ltp_history_uses_base_output_only() {
    let ics = long_ics_info(8);
    let base = make_channel(&[1, 3, 5, 7, 9, 11, 11, 2], 150, 0xF0);
    let ext_a = make_channel(&[2, 4, 6, 8, 10, 11, 1, 1], 146, 0x101);
    let ext_b = make_channel(&[2, 4, 6, 8, 10, 11, 1, 1], 146, 0x202);
    let ltp = LtpData {
        lag_update: None,
        lag: Some(512),
        coef: 5,
        long_used: vec![true; 8],
        short: None,
    };
    let cfg = ScalableConfig {
        aot: AOT_SCALABLE,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family: FrameFamily::Lc1024,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![false, false],
    };
    let mk = |ext: &ScalableChannel| -> Vec<Vec<u8>> {
        ScalableFrame {
            layers: vec![
                ScalableLayer {
                    ics: ics.clone(),
                    ms_mask_present: MsMaskPresent::AllZeros,
                    ms_used_new: Vec::new(),
                    tns: vec![None],
                    ltp: vec![Some(ltp.clone())],
                    diff_lr_long: Vec::new(),
                    diff_lr_short: vec![None],
                    channels: vec![base.clone()],
                },
                plain_layer(ics.clone(), vec![ext.clone()]),
            ],
            ms_used: vec![Vec::new()],
            diff_lr_long: [Vec::new(), Vec::new()],
            diff_lr_short: [None, None],
            max_total_sfb: 8,
            max_mono_sfb: 8,
        }
        .write(&cfg)
        .unwrap()
    };
    let pa = mk(&ext_a);
    let pb = mk(&ext_b);

    // Expected difference per frame: filterbank(ext_a − ext_b) — the
    // same every frame (steady input).
    let sa = reconstruct(&ext_a, &ics);
    let sb = reconstruct(&ext_b, &ics);
    let diff_spec: Vec<f64> = sa.iter().zip(&sb).map(|(x, y)| x - y).collect();
    let mut fb = oxideav_aac::filterbank::Filterbank::new();

    let mut da = ScalableDecoder::new(cfg.clone()).unwrap();
    let mut db = ScalableDecoder::new(cfg).unwrap();
    for f in 0..4 {
        let want_diff = fb.synthesize(&diff_spec, &ics).unwrap();
        let a = da
            .decode_frame_channels(&pa.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let b = db
            .decode_frame_channels(&pb.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let peak = want_diff.iter().fold(0.0f64, |m, v| m.max(v.abs())) + 1.0;
        for i in 0..1024 {
            let got_diff = a[0][i] - b[0][i];
            assert!(
                (got_diff - want_diff[i]).abs() < 1e-6 * peak,
                "frame {f} sample {i}: LTP history leaked the extension layer"
            );
        }
    }
}

/// `EIGHT_SHORT` geometry: a grouped short-window single-layer stereo
/// stream decodes bit-identically to the equivalent common-window CPE.
#[test]
fn short_window_single_layer_matches_cpe() {
    let family = FrameFamily::Lc1024;
    let sfg = 0b0110101; // window groups from the 7-bit grouping field
    let (num_windows, num_window_groups, window_group_length, num_swb) =
        derive_window_grouping_family(family, WindowSequence::EightShort, Some(sfg), FS_INDEX)
            .unwrap();
    let ics = IcsInfo {
        family,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::EightShort,
        window_shape: WindowShape::Sine,
        max_sfb: 4,
        scale_factor_grouping: Some(sfg),
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows,
        num_window_groups,
        window_group_length,
        num_swb,
    };
    let groups = usize::from(ics.num_window_groups);
    let cbs: Vec<Vec<u8>> = (0..groups)
        .map(|g| match g % 3 {
            0 => vec![1, 3, 5, 7],
            1 => vec![2, 4, 6, 8],
            _ => vec![9, 10, 11, 1],
        })
        .collect();
    let left = make_channel_family(family, &ics, &cbs, 150, 0x303);
    let right = make_channel_family(family, &ics, &cbs, 149, 0x404);
    let ms_used: Vec<Vec<bool>> = (0..groups)
        .map(|g| vec![g % 2 == 0, false, true, false])
        .collect();
    let cfg = ScalableConfig {
        aot: AOT_SCALABLE,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![true],
    };
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
        max_total_sfb: 4,
        max_mono_sfb: 0,
    };
    let payloads = frame.write(&cfg).unwrap();

    // Reference CPE.
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Cpe, 0).unwrap();
    let mut bw = BitWriter::new();
    bw.write_bit(true);
    ics.write(&mut bw, AOT_LC, FS_INDEX, true).unwrap();
    bw.write_u32(1, 2); // ms_mask_present = 1
    for row in &ms_used {
        for &b in row {
            bw.write_bit(b);
        }
    }
    for chan in [&left, &right] {
        chan.body
            .write_with_ics_info(&mut bw, &ics, AOT_LC, false)
            .unwrap();
        chan.spectral
            .write(&mut bw, &ics, &chan.body.section_data, FS_INDEX)
            .unwrap();
    }
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    let plain = fa.push_end();

    let mut dec = ScalableDecoder::new(cfg).unwrap();
    let mut reference = StreamDecoder::new();
    for f in 0..3 {
        let got = dec
            .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let want = reference
            .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 2, 1, &plain)
            .unwrap();
        assert_eq!(got.pcm, want.pcm, "frame {f}");
    }
}

/// The 960-line family (`frameLengthFlag == 1`): a single-layer mono
/// program decodes bit-identically to the same body through the
/// 960-line `StreamDecoder`.
#[test]
fn family_960_single_layer() {
    let family = FrameFamily::Lc960;
    let (num_windows, num_window_groups, window_group_length, num_swb) =
        derive_window_grouping_family(family, WindowSequence::OnlyLong, None, FS_INDEX).unwrap();
    let ics = IcsInfo {
        family,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb: 8,
        scale_factor_grouping: None,
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows,
        num_window_groups,
        window_group_length,
        num_swb,
    };
    let chan = make_channel_family(family, &ics, &[vec![1, 3, 5, 7, 9, 11, 11, 2]], 150, 0x505);
    let cfg = ScalableConfig {
        aot: AOT_SCALABLE,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![false],
    };
    let frame = ScalableFrame {
        layers: vec![ScalableLayer {
            ics: ics.clone(),
            ms_mask_present: MsMaskPresent::AllZeros,
            ms_used_new: Vec::new(),
            tns: vec![None],
            ltp: vec![None],
            diff_lr_long: Vec::new(),
            diff_lr_short: vec![None],
            channels: vec![chan.clone()],
        }],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 8,
        max_mono_sfb: 8,
    };
    let payloads = frame.write(&cfg).unwrap();

    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let mut bw = BitWriter::new();
    let mut body = chan.body.clone();
    body.ics_info = Some(ics.clone());
    body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
    chan.spectral
        .write(&mut bw, &ics, &chan.body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    let plain = fa.push_end();

    let mut dec = ScalableDecoder::new(cfg).unwrap();
    let mut reference = StreamDecoder::new();
    reference.set_frame_family(family);
    for f in 0..3 {
        let got = dec
            .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let want = reference
            .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 1, 1, &plain)
            .unwrap();
        assert_eq!(got.pcm.len(), 960, "frame {f}");
        assert_eq!(got.pcm, want.pcm, "frame {f}");
    }
}

//! ER AAC scalable (AOT 20) — §4.5.2.4: the scalable element syntax
//! is unchanged for error resilience; the three §4.4.6 resilience
//! branches (5-bit `sect_cb` section data, RVLC scalefactors, HCR
//! reordered spectra) change only the wire coding of each
//! `individual_channel_stream(1,1)`. The ER decode of a layer stack
//! must therefore be bit-identical to the AOT-6 decode of the same
//! spectra.

use oxideav_aac::asc::AacResilienceFlags;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::ms_stereo::MsMaskPresent;
use oxideav_aac::scalable::{
    ScalableChannel, ScalableConfig, ScalableDecoder, ScalableFrame, ScalableLayer,
};
use oxideav_aac::scale_factor_data::{ErScaleFactorData, ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::swb_offset::FrameFamily;

const FS_INDEX: u8 = 4;
const SAMPLE_RATE: u32 = 44100;

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

/// A long-window channel with plain spectrum-book bands only (the
/// RVLC track keeps every DPCM delta 0, so `rev_global_gain` is the
/// forward `global_gain`).
fn make_channel(sfb_cbs: &[u8], global_gain: u8, seed: u32, er: bool) -> ScalableChannel {
    let sections = vec![sfb_cbs
        .iter()
        .enumerate()
        .map(|(sfb, &cb)| Section {
            codebook: cb,
            start: sfb as u8,
            end: sfb as u8 + 1,
        })
        .collect()];
    let entries: Vec<ScaleFactorEntry> = sfb_cbs
        .iter()
        .filter(|&&cb| cb != 0)
        .map(|_| ScaleFactorEntry::Dpcm(0))
        .collect();
    let sfd = ScaleFactorData {
        entries: vec![entries],
    };
    let er_sfd = er.then(|| ErScaleFactorData {
        sf_concealment: false,
        rev_global_gain: global_gain,
        data: sfd.clone(),
        dpcm_is_last_position: None,
        dpcm_noise_last_position: None,
    });
    let body = IcsBody {
        global_gain,
        ics_info: None,
        section_data: SectionData {
            sections,
            sfb_cb: vec![sfb_cbs.to_vec()],
        },
        scale_factor_data: sfd,
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: er_sfd,
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
            0 => 0,
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

/// The HCR length fields the writer's re-encode must match are
/// computed from the spectrum itself.
fn install_hcr_lengths(chan: &mut ScalableChannel, ics: &IcsInfo) {
    let (_, len, longest) = oxideav_aac::hcr_decode::encode_reordered_spectral_data(
        &chan.spectral,
        ics,
        &chan.body.section_data,
        FS_INDEX,
    )
    .unwrap();
    chan.body.reordered_spectral_lengths = Some((len, longest));
}

fn cfg(aot: u8, resilience: AacResilienceFlags, layer_stereo: Vec<bool>) -> ScalableConfig {
    ScalableConfig {
        aot,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family: FrameFamily::Lc1024,
        resilience,
        layer_stereo,
    }
}

fn frame(layers: Vec<ScalableLayer>, ms_used: Vec<Vec<bool>>, max_sfb: u8) -> ScalableFrame {
    ScalableFrame {
        layers,
        ms_used,
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: max_sfb,
        max_mono_sfb: 0,
    }
}

/// A two-layer stereo AOT-20 stack under the full resilience triplet
/// (ER sections + RVLC scalefactors + HCR spectra) decodes
/// bit-identically to the AOT-6 stack carrying the same spectra.
#[test]
fn er_triplet_matches_plain_scalable() {
    let resilience = AacResilienceFlags {
        section_data: true,
        scalefactor_data: true,
        spectral_data: true,
    };
    let ics = long_ics_info(6);
    let cbs_l0_l: &[u8] = &[1, 3, 5, 7, 9, 11];
    let cbs_l0_r: &[u8] = &[2, 4, 6, 8, 10, 11];
    let cbs_l1_l: &[u8] = &[2, 4, 6, 8, 9, 2];
    let cbs_l1_r: &[u8] = &[1, 3, 5, 7, 10, 1];
    let ms_used = vec![vec![true, false, false, true, false, true]];

    let build = |er: bool| -> (ScalableConfig, Vec<Vec<u8>>) {
        let res = if er {
            resilience
        } else {
            AacResilienceFlags::default()
        };
        let c = cfg(if er { 20 } else { 6 }, res, vec![true, true]);
        let mk = |cbs: &[u8], gg: u8, seed: u32| {
            let mut ch = make_channel(cbs, gg, seed, er);
            if er {
                install_hcr_lengths(&mut ch, &ics);
            }
            ch
        };
        let f = frame(
            vec![
                ScalableLayer {
                    ics: ics.clone(),
                    ms_mask_present: MsMaskPresent::Mask,
                    ms_used_new: ms_used.clone(),
                    tns: vec![None, None],
                    ltp: vec![None, None],
                    diff_lr_long: Vec::new(),
                    diff_lr_short: vec![None, None],
                    channels: vec![mk(cbs_l0_l, 150, 0x11), mk(cbs_l0_r, 149, 0x22)],
                },
                ScalableLayer {
                    ics: ics.clone(),
                    ms_mask_present: MsMaskPresent::AllZeros,
                    ms_used_new: Vec::new(),
                    tns: vec![None, None],
                    ltp: vec![None, None],
                    diff_lr_long: Vec::new(),
                    diff_lr_short: vec![None, None],
                    channels: vec![mk(cbs_l1_l, 148, 0x33), mk(cbs_l1_r, 147, 0x44)],
                },
            ],
            ms_used.clone(),
            6,
        );
        let payloads = f.write(&c).unwrap();
        (c, payloads)
    };

    let (cfg_er, p_er) = build(true);
    let (cfg_plain, p_plain) = build(false);
    // The ER wire coding must actually differ.
    assert_ne!(p_er, p_plain);

    let mut dec_er = ScalableDecoder::new(cfg_er).unwrap();
    let mut dec_plain = ScalableDecoder::new(cfg_plain).unwrap();
    for f in 0..3 {
        let a = dec_er
            .decode_frame(&p_er.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        let b = dec_plain
            .decode_frame(&p_plain.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        assert_eq!(a.channels, 2);
        assert_eq!(a.pcm, b.pcm, "frame {f}");
        assert!(a.pcm.iter().any(|&s| s != 0), "frame {f} silent");
    }
}

/// The ER round-trip: an AOT-20 frame under the triplet re-emits
/// byte-identically through parse → write.
#[test]
fn er_frame_roundtrip() {
    let resilience = AacResilienceFlags {
        section_data: true,
        scalefactor_data: true,
        spectral_data: true,
    };
    let ics = long_ics_info(5);
    let c = cfg(20, resilience, vec![false]);
    let mut ch = make_channel(&[1, 3, 7, 9, 11], 150, 0x55, true);
    install_hcr_lengths(&mut ch, &ics);
    let f = ScalableFrame {
        layers: vec![ScalableLayer {
            ics: ics.clone(),
            ms_mask_present: MsMaskPresent::AllZeros,
            ms_used_new: Vec::new(),
            tns: vec![None],
            ltp: vec![None],
            diff_lr_long: Vec::new(),
            diff_lr_short: vec![None],
            channels: vec![ch],
        }],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 5,
        max_mono_sfb: 5,
    };
    let payloads = f.write(&c).unwrap();
    let parsed =
        ScalableFrame::parse(&c, &payloads.iter().map(Vec::as_slice).collect::<Vec<_>>()).unwrap();
    let rewritten = parsed.write(&c).unwrap();
    assert_eq!(rewritten, payloads);
}

/// The mutation battery over the ER wire form: every single-bit flip
/// of the AOT-20 payload is Ok or a typed Err, never a panic.
#[test]
fn er_bit_flip_battery() {
    let resilience = AacResilienceFlags {
        section_data: true,
        scalefactor_data: true,
        spectral_data: true,
    };
    let ics = long_ics_info(5);
    let c = cfg(20, resilience, vec![false]);
    let mut ch = make_channel(&[1, 3, 7, 9, 11], 150, 0x66, true);
    install_hcr_lengths(&mut ch, &ics);
    let f = ScalableFrame {
        layers: vec![ScalableLayer {
            ics: ics.clone(),
            ms_mask_present: MsMaskPresent::AllZeros,
            ms_used_new: Vec::new(),
            tns: vec![None],
            ltp: vec![None],
            diff_lr_long: Vec::new(),
            diff_lr_short: vec![None],
            channels: vec![ch],
        }],
        ms_used: vec![Vec::new()],
        diff_lr_long: [Vec::new(), Vec::new()],
        diff_lr_short: [None, None],
        max_total_sfb: 5,
        max_mono_sfb: 5,
    };
    let payloads = f.write(&c).unwrap();
    let mut errors = 0usize;
    for bit in 0..payloads[0].len() * 8 {
        let mut mutated = payloads.clone();
        mutated[0][bit / 8] ^= 0x80 >> (bit % 8);
        let mut d = ScalableDecoder::new(c.clone()).unwrap();
        let m: Vec<&[u8]> = mutated.iter().map(Vec::as_slice).collect();
        if d.decode_frame(&m).is_err() {
            errors += 1;
        }
    }
    assert!(errors > 0);
}

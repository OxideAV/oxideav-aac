//! End-to-end §4.6.8.3.3 coupling-channel decode — ISO/IEC 14496-3
//! Table 4.8 / `decode_coupling_channel()`.
//!
//! Builds raw data blocks with the crate's own bit-exact writers (SCE
//! / CPE bodies + a `coupling_channel_element()`), decodes them through
//! the public `StreamDecoder`, and pins the coupling against the
//! *linearity* of the §4.6.11 filterbank: with no TNS anywhere, adding
//! `cc_gain ×` the embedded-SCE spectrum onto a target before its
//! filterbank must equal adding `cc_gain ×` the standalone decode of
//! the embedded SCE in the PCM domain,
//!
//! ```text
//! decode([target, CCE]) == decode([target]) + cc_gain · decode([embedded])
//! ```
//!
//! within the ±1 LSB each of the three s16 roundings contributes. The
//! same identity validates the independently-switched (time-domain)
//! path, since the CCE's own filterbank starts from the same zero
//! state as the standalone decode.

use oxideav_aac::cce::{CoupledTarget, CouplingGains, CouplingHeader, GainList};
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_core::bits::BitWriter;

const AOT_LC: u8 = 2;
const FS_INDEX: u8 = 4; // 44.1 kHz
const SAMPLE_RATE: u32 = 44100;

fn long_ics_info(max_sfb: u8) -> IcsInfo {
    IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
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

/// A minimal long-window channel: codebook-1 sections over
/// `0..max_sfb`, flat scalefactors, and a small constant quantized
/// spectrum `value` in every coefficient of the active bands.
fn make_channel(max_sfb: u8, value: i32, global_gain: u8) -> (IcsBody, SpectralData) {
    let sfb_cb = vec![vec![1u8; max_sfb as usize]];
    let sections = vec![vec![Section {
        codebook: 1,
        start: 0,
        end: max_sfb,
    }]];
    let entries: Vec<ScaleFactorEntry> = (0..max_sfb).map(|_| ScaleFactorEntry::Dpcm(0)).collect();
    let body = IcsBody {
        global_gain,
        ics_info: Some(long_ics_info(max_sfb)),
        section_data: SectionData { sections, sfb_cb },
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
    // Codebook 1 carries quads in [-1, 1]: keep |value| <= 1. Only the
    // transmitted bands (0..max_sfb) may carry non-zero coefficients.
    let offsets = oxideav_aac::swb_offset::long_window_offsets(FS_INDEX).unwrap();
    let active = usize::from(offsets[max_sfb as usize]);
    let mut coeffs = vec![0i32; 1024];
    coeffs[..active].fill(value);
    let spectral = SpectralData {
        x_quant: vec![coeffs],
    };
    (body, spectral)
}

/// Append one SCE (tag) to the assembler.
fn push_sce(fa: &mut FrameAssembler, tag: u8, body: &IcsBody, spectral: &SpectralData) {
    fa.push_channel_header(IdSynEle::Sce, tag).unwrap();
    let mut bw = BitWriter::new();
    body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
    let ics = body.ics_info.as_ref().unwrap();
    spectral
        .write(&mut bw, ics, &body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
}

/// Append one non-shared CPE (tag) with two independent channels.
fn push_cpe(
    fa: &mut FrameAssembler,
    tag: u8,
    left: (&IcsBody, &SpectralData),
    right: (&IcsBody, &SpectralData),
) {
    fa.push_channel_header(IdSynEle::Cpe, tag).unwrap();
    let mut bw = BitWriter::new();
    bw.write_bit(false); // common_window = 0
    for (body, spectral) in [left, right] {
        body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
        let ics = body.ics_info.as_ref().unwrap();
        spectral
            .write(&mut bw, ics, &body.section_data, FS_INDEX)
            .unwrap();
    }
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
}

/// Append one CCE with the given header/gains and embedded channel.
fn push_cce(
    fa: &mut FrameAssembler,
    tag: u8,
    header: &CouplingHeader,
    body: &IcsBody,
    spectral: &SpectralData,
    gains: &CouplingGains,
) {
    fa.push_channel_header(IdSynEle::Cce, tag).unwrap();
    let mut bw = BitWriter::new();
    header.write(&mut bw).unwrap();
    body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
    let ics = body.ics_info.as_ref().unwrap();
    spectral
        .write(&mut bw, ics, &body.section_data, FS_INDEX)
        .unwrap();
    gains
        .write(&mut bw, header, &body.section_data.sfb_cb)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
}

fn finish_block(fa: FrameAssembler) -> Vec<u8> {
    fa.push_end()
}

/// Decode `frames` copies of `payload` through a fresh decoder,
/// returning per-frame interleaved PCM.
fn decode_frames(payload: &[u8], frames: usize) -> Vec<Vec<i16>> {
    let mut dec = StreamDecoder::new();
    (0..frames)
        .map(|_| {
            let f = dec
                .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 0, 1, payload)
                .unwrap();
            f.pcm
        })
        .collect()
}

/// `got ≈ a + gain·b` within the stacked s16 rounding slack.
fn assert_linear_sum(got: &[i16], a: &[i16], b: &[i16], gain: f64, what: &str) {
    assert_eq!(got.len(), a.len());
    assert_eq!(got.len(), b.len());
    let mut max_err = 0i32;
    for i in 0..got.len() {
        let expect = f64::from(a[i]) + gain * f64::from(b[i]);
        let err = (f64::from(got[i]) - expect).abs().round() as i32;
        max_err = max_err.max(err);
    }
    assert!(max_err <= 2, "{what}: max deviation {max_err} LSB");
}

/// A CCE with one SCE target couples in natural scaling (the implicit
/// list 0): `decode([SCE, CCE]) == decode([SCE]) + decode([embedded])`.
/// Runs several frames so the filterbank overlap state of the coupled
/// and standalone decodes stays in lockstep.
#[test]
fn dependently_switched_cce_couples_onto_sce_in_natural_scaling() {
    let (target_body, target_spec) = make_channel(8, 1, 156);
    let (emb_body, emb_spec) = make_channel(12, 1, 152);

    let header = CouplingHeader {
        ind_sw_cce_flag: false,
        num_coupled_elements: 0,
        targets: vec![CoupledTarget {
            is_cpe: false,
            tag_select: 0,
            cc_l: false,
            cc_r: false,
        }],
        cc_domain: false,
        gain_element_sign: false,
        gain_element_scale: 0,
        num_gain_element_lists: 1,
    };
    let gains = CouplingGains {
        cc_scale: 1.090_507_732_665_257_7,
        gain_element_sign: false,
        lists: vec![],
    };

    // [SCE0, CCE] — CCE after its target (the two-pass walk must not
    // care about element order).
    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, 0, &target_body, &target_spec);
    push_cce(&mut fa, 0, &header, &emb_body, &emb_spec, &gains);
    let coupled = finish_block(fa);

    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, 0, &target_body, &target_spec);
    let base = finish_block(fa);

    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, 0, &emb_body, &emb_spec);
    let standalone_emb = finish_block(fa);

    let got = decode_frames(&coupled, 3);
    let a = decode_frames(&base, 3);
    let b = decode_frames(&standalone_emb, 3);
    for f in 0..3 {
        assert_linear_sum(&got[f], &a[f], &b[f], 1.0, &format!("frame {f}"));
    }
    // The coupling actually did something.
    assert_ne!(got[1], a[1]);
}

/// Two SCE targets: the first couples with the implicit natural list,
/// the second with a transmitted `common_gain_element` (cc_scale = 2,
/// gain 1 ⇒ cc_gain = 2).
#[test]
fn cce_gain_list_scales_second_target() {
    let (t0_body, t0_spec) = make_channel(8, 1, 156);
    let (t1_body, t1_spec) = make_channel(6, 1, 154);
    let (emb_body, emb_spec) = make_channel(12, 1, 150);

    let header = CouplingHeader {
        ind_sw_cce_flag: false,
        num_coupled_elements: 1,
        targets: vec![
            CoupledTarget {
                is_cpe: false,
                tag_select: 0,
                cc_l: false,
                cc_r: false,
            },
            CoupledTarget {
                is_cpe: false,
                tag_select: 1,
                cc_l: false,
                cc_r: false,
            },
        ],
        cc_domain: true, // after-TNS domain (no TNS here, same result)
        gain_element_sign: false,
        gain_element_scale: 3, // cc_scale = 2
        num_gain_element_lists: 2,
    };
    let gains = CouplingGains {
        cc_scale: 2.0,
        gain_element_sign: false,
        // cc_gain = 2^(−(−1)) = 2 (conformance-settled negated
        // exponent; the am05 vectors carry the same −1 form).
        lists: vec![GainList::Common(-1)],
    };

    let mut fa = FrameAssembler::new();
    push_cce(&mut fa, 3, &header, &emb_body, &emb_spec, &gains);
    push_sce(&mut fa, 0, &t0_body, &t0_spec);
    push_sce(&mut fa, 1, &t1_body, &t1_spec);
    let coupled = finish_block(fa);

    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, 0, &t0_body, &t0_spec);
    push_sce(&mut fa, 1, &t1_body, &t1_spec);
    let base = finish_block(fa);

    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, 0, &emb_body, &emb_spec);
    let standalone_emb = finish_block(fa);

    let got = decode_frames(&coupled, 2);
    let ab = decode_frames(&base, 2);
    let b = decode_frames(&standalone_emb, 2);
    for f in 0..2 {
        // Interleaved [ch0, ch1] per time index.
        let got0: Vec<i16> = got[f].iter().copied().step_by(2).collect();
        let got1: Vec<i16> = got[f].iter().copied().skip(1).step_by(2).collect();
        let a0: Vec<i16> = ab[f].iter().copied().step_by(2).collect();
        let a1: Vec<i16> = ab[f].iter().copied().skip(1).step_by(2).collect();
        assert_linear_sum(&got0, &a0, &b[f], 1.0, &format!("frame {f} target 0"));
        assert_linear_sum(&got1, &a1, &b[f], 2.0, &format!("frame {f} target 1"));
    }
}

/// An independently switched CCE is decoded to the time domain through
/// its own filterbank and added there — the same linearity identity
/// holds, exercised over several frames so the CCE's own overlap
/// state matters.
#[test]
fn independently_switched_cce_couples_in_time_domain() {
    let (target_body, target_spec) = make_channel(8, 1, 156);
    let (emb_body, emb_spec) = make_channel(10, 1, 152);

    let header = CouplingHeader {
        ind_sw_cce_flag: true,
        num_coupled_elements: 0,
        targets: vec![CoupledTarget {
            is_cpe: false,
            tag_select: 0,
            cc_l: false,
            cc_r: false,
        }],
        cc_domain: false,
        gain_element_sign: false,
        gain_element_scale: 0,
        num_gain_element_lists: 1,
    };
    let gains = CouplingGains {
        cc_scale: 1.090_507_732_665_257_7,
        gain_element_sign: false,
        lists: vec![],
    };

    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, 0, &target_body, &target_spec);
    push_cce(&mut fa, 0, &header, &emb_body, &emb_spec, &gains);
    let coupled = finish_block(fa);

    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, 0, &target_body, &target_spec);
    let base = finish_block(fa);

    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, 0, &emb_body, &emb_spec);
    let standalone_emb = finish_block(fa);

    let got = decode_frames(&coupled, 3);
    let a = decode_frames(&base, 3);
    let b = decode_frames(&standalone_emb, 3);
    for f in 0..3 {
        assert_linear_sum(&got[f], &a[f], &b[f], 1.0, &format!("frame {f}"));
    }
    assert_ne!(got[2], a[2]);
}

/// A CPE target with `cc_l = 1, cc_r = 0`: only the left channel picks
/// up the coupling (as the first — and only — list consumer it takes
/// the implicit natural-scaling list 0, so `num_gain_element_lists ==
/// 1` and no list is transmitted); the right channel is bit-identical
/// to the un-coupled decode.
#[test]
fn cce_cpe_target_left_only() {
    let (l_body, l_spec) = make_channel(8, 1, 156);
    let (r_body, r_spec) = make_channel(8, 1, 154);
    let (emb_body, emb_spec) = make_channel(12, 1, 150);

    let header = CouplingHeader {
        ind_sw_cce_flag: false,
        num_coupled_elements: 0,
        targets: vec![CoupledTarget {
            is_cpe: true,
            tag_select: 0,
            cc_l: true,
            cc_r: false,
        }],
        cc_domain: false,
        gain_element_sign: false,
        gain_element_scale: 0,
        num_gain_element_lists: 1,
    };
    let gains = CouplingGains {
        cc_scale: 1.090_507_732_665_257_7,
        gain_element_sign: false,
        lists: vec![],
    };

    let mut fa = FrameAssembler::new();
    push_cpe(&mut fa, 0, (&l_body, &l_spec), (&r_body, &r_spec));
    push_cce(&mut fa, 0, &header, &emb_body, &emb_spec, &gains);
    let coupled = finish_block(fa);

    let mut fa = FrameAssembler::new();
    push_cpe(&mut fa, 0, (&l_body, &l_spec), (&r_body, &r_spec));
    let base = finish_block(fa);

    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, 0, &emb_body, &emb_spec);
    let standalone_emb = finish_block(fa);

    let got = decode_frames(&coupled, 2);
    let ab = decode_frames(&base, 2);
    let b = decode_frames(&standalone_emb, 2);
    for f in 0..2 {
        let got_l: Vec<i16> = got[f].iter().copied().step_by(2).collect();
        let got_r: Vec<i16> = got[f].iter().copied().skip(1).step_by(2).collect();
        let a_l: Vec<i16> = ab[f].iter().copied().step_by(2).collect();
        let a_r: Vec<i16> = ab[f].iter().copied().skip(1).step_by(2).collect();
        assert_linear_sum(&got_l, &a_l, &b[f], 1.0, &format!("frame {f} left"));
        assert_eq!(got_r, a_r, "frame {f}: right channel must be untouched");
        assert_ne!(got_l, a_l, "frame {f}: left channel must be coupled");
    }
}

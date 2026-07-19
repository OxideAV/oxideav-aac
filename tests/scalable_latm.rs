//! LATM/LOAS transport of scalable AAC programs (§1.7 + §4.4.2.2):
//! a two-layer AOT-6 program (mono base + stereo extension) rides two
//! `streamID`s of one program; [`LoasDecoder`] collects the layer
//! stack per access unit and decodes it through the §4.5.2.2
//! combination — byte-identical to feeding the layer payloads to
//! [`ScalableDecoder`] directly.

use oxideav_aac::asc::AacResilienceFlags;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::latm::LoasDecoder;
use oxideav_aac::ms_stereo::MsMaskPresent;
use oxideav_aac::scalable::{
    ScalableChannel, ScalableConfig, ScalableDecoder, ScalableFrame, ScalableLayer,
};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::swb_offset::FrameFamily;
use oxideav_core::bits::BitWriter;

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
    let entries: Vec<ScaleFactorEntry> = sfb_cbs
        .iter()
        .filter(|&&cb| cb != 0)
        .map(|_| ScaleFactorEntry::Dpcm(0))
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

/// Write an AOT-6 ASC: AOT(5) / fsIndex(4) / chanConfig(4) /
/// GASpecificConfig {frameLengthFlag=0, dependsOnCoreCoder=0,
/// extensionFlag=0, layerNr(3)}.
fn write_scalable_asc(w: &mut BitWriter, chan_config: u8, layer_nr: u8) {
    w.write_u32(6, 5);
    w.write_u32(u32::from(FS_INDEX), 4);
    w.write_u32(u32::from(chan_config), 4);
    w.write_bit(false); // frameLengthFlag
    w.write_bit(false); // dependsOnCoreCoder
    w.write_bit(false); // extensionFlag
    w.write_u32(u32::from(layer_nr), 3);
}

/// One LOAS frame per access unit: a two-layer program
/// (`numLayer == 1`) with per-layer ASCs, `frameLengthType == 0`.
fn build_loas_two_layer(per_frame_payloads: &[[Vec<u8>; 2]]) -> Vec<u8> {
    let mut out = Vec::new();
    for (i, payloads) in per_frame_payloads.iter().enumerate() {
        let mut w = BitWriter::new();
        if i == 0 {
            w.write_bit(false); // useSameStreamMux = 0
            w.write_bit(false); // audioMuxVersion = 0
            w.write_bit(true); // allStreamsSameTimeFraming
            w.write_u32(0, 6); // numSubFrames
            w.write_u32(0, 4); // numProgram
            w.write_u32(1, 3); // numLayer = 1 -> two layers
                               // layer 0: mono base (no useSameConfig bit on (0,0)).
            write_scalable_asc(&mut w, 1, 0);
            w.write_u32(0, 3); // frameLengthType = 0
            w.write_u32(0xFF, 8); // latmBufferFullness
                                  // layer 1: stereo extension.
            w.write_bit(false); // useSameConfig = 0
            write_scalable_asc(&mut w, 2, 1);
            w.write_u32(0, 3); // frameLengthType = 0
            w.write_u32(0xFF, 8); // latmBufferFullness
            w.write_bit(false); // otherDataPresent
            w.write_bit(false); // crcCheckPresent
        } else {
            w.write_bit(true); // useSameStreamMux = 1
        }
        // PayloadLengthInfo() for both streams, then PayloadMux().
        for payload in payloads {
            let mut rem = payload.len();
            while rem >= 255 {
                w.write_u32(255, 8);
                rem -= 255;
            }
            w.write_u32(rem as u32, 8);
        }
        for payload in payloads {
            for &b in payload {
                w.write_u32(u32::from(b), 8);
            }
        }
        let element = w.finish();
        assert!(element.len() < (1 << 13));
        let mut fw = BitWriter::new();
        fw.write_u32(0x2B7, 11);
        fw.write_u32(element.len() as u32, 13);
        out.extend_from_slice(&fw.finish());
        out.extend_from_slice(&element);
    }
    out
}

#[test]
fn loas_two_layer_scalable_program() {
    let ics_m = long_ics_info(4);
    let ics_s = long_ics_info(6);
    let cfg = ScalableConfig {
        aot: 6,
        fs_index: FS_INDEX,
        sample_rate: SAMPLE_RATE,
        family: FrameFamily::Lc1024,
        resilience: AacResilienceFlags::default(),
        layer_stereo: vec![false, true],
    };
    let ms_used = vec![vec![false, true, false, true, false, false]];
    let frame = ScalableFrame {
        layers: vec![
            ScalableLayer {
                ics: ics_m.clone(),
                ms_mask_present: MsMaskPresent::AllZeros,
                ms_used_new: Vec::new(),
                tns: vec![None],
                ltp: vec![None],
                diff_lr_long: Vec::new(),
                diff_lr_short: vec![None],
                channels: vec![make_channel(&[1, 3, 5, 7], 150, 0x77)],
            },
            ScalableLayer {
                ics: ics_s.clone(),
                ms_mask_present: MsMaskPresent::Mask,
                ms_used_new: ms_used.clone(),
                tns: vec![None, None],
                ltp: vec![None, None],
                diff_lr_long: vec![vec![false, true], vec![true, false]],
                diff_lr_short: vec![None, None],
                channels: vec![
                    make_channel(&[2, 4, 6, 8, 9, 10], 148, 0x88),
                    make_channel(&[1, 3, 5, 7, 11, 2], 149, 0x99),
                ],
            },
        ],
        ms_used: ms_used.clone(),
        diff_lr_long: [
            vec![Some(false), None, Some(true)],
            vec![Some(true), None, Some(false)],
        ],
        diff_lr_short: [None, None],
        max_total_sfb: 6,
        max_mono_sfb: 4,
    };
    let payloads = frame.write(&cfg).unwrap();
    let per_frame: Vec<[Vec<u8>; 2]> = (0..3)
        .map(|_| [payloads[0].clone(), payloads[1].clone()])
        .collect();
    let loas = build_loas_two_layer(&per_frame);

    let mut loas_dec = LoasDecoder::new();
    let frames = loas_dec.decode_all(&loas).unwrap();
    assert_eq!(frames.len(), 3, "one combined frame per access unit");

    let mut direct = ScalableDecoder::new(cfg).unwrap();
    for (f, got) in frames.iter().enumerate() {
        let want = direct
            .decode_frame(&payloads.iter().map(Vec::as_slice).collect::<Vec<_>>())
            .unwrap();
        assert_eq!(got.channels, 2, "frame {f}");
        assert_eq!(got.sample_rate, SAMPLE_RATE, "frame {f}");
        assert_eq!(got.pcm, want.pcm, "frame {f}");
        assert!(got.pcm.iter().any(|&s| s != 0), "frame {f} silent");
    }
}

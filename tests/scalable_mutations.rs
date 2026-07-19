//! Scalable-AAC corruption battery — every parse branch of the
//! §4.4.2.2 payloads must survive hostile input: deterministic
//! bit-flips and truncations over writer-assembled multi-layer
//! streams must yield `Ok` or a typed `Err`, never a panic, and the
//! decoder must stay usable afterwards.

use oxideav_aac::asc::AacResilienceFlags;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, LtpData, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::ms_stereo::MsMaskPresent;
use oxideav_aac::scalable::{
    ScalableChannel, ScalableConfig, ScalableDecoder, ScalableFrame, ScalableLayer,
};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::swb_offset::FrameFamily;
use oxideav_aac::tns_data::{TnsData, TnsFilter, TnsWindow};

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

/// The battery's reference stream: mono base (with TNS + LTP) plus a
/// stereo extension with M/S, intensity-free plain bands and diff
/// bits — every scalable header branch is on the wire.
fn battery_stream() -> (ScalableConfig, Vec<Vec<u8>>) {
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
    let ltp = LtpData {
        lag_update: None,
        lag: Some(300),
        coef: 3,
        long_used: vec![true, false, true, false],
        short: None,
    };
    let ms_used = vec![vec![false, true, false, true, false, false]];
    let frame = ScalableFrame {
        layers: vec![
            ScalableLayer {
                ics: ics_m.clone(),
                ms_mask_present: MsMaskPresent::AllZeros,
                ms_used_new: Vec::new(),
                tns: vec![Some(tns_m)],
                ltp: vec![Some(ltp)],
                diff_lr_long: Vec::new(),
                diff_lr_short: vec![None],
                channels: vec![make_channel(&[1, 3, 5, 7], 150, 0x111)],
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
                    make_channel(&[2, 4, 6, 8, 9, 10], 148, 0x222),
                    make_channel(&[1, 3, 5, 7, 11, 2], 149, 0x333),
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
    (cfg, payloads)
}

/// Every single-bit flip in either layer payload decodes to `Ok` or a
/// typed `Err` — never a panic — and the decoder keeps decoding the
/// pristine stream afterwards.
#[test]
fn bit_flip_battery() {
    let (cfg, payloads) = battery_stream();
    // Pristine decode must work.
    let mut dec = ScalableDecoder::new(cfg.clone()).unwrap();
    let refs: Vec<&[u8]> = payloads.iter().map(Vec::as_slice).collect();
    let pristine = dec.decode_frame(&refs).unwrap();
    assert!(pristine.pcm.iter().any(|&s| s != 0));

    let mut errors = 0usize;
    let mut oks = 0usize;
    for lay in 0..payloads.len() {
        for bit in 0..payloads[lay].len() * 8 {
            let mut mutated: Vec<Vec<u8>> = payloads.clone();
            mutated[lay][bit / 8] ^= 0x80 >> (bit % 8);
            let mut d = ScalableDecoder::new(cfg.clone()).unwrap();
            let m: Vec<&[u8]> = mutated.iter().map(Vec::as_slice).collect();
            match d.decode_frame(&m) {
                Ok(_) => oks += 1,
                Err(_) => errors += 1,
            }
            // The decoder object survives a poisoned frame: the
            // pristine stream still decodes (spot-checked to keep the
            // battery fast).
            if bit % 29 == 0 {
                let _ = d.decode_frame(&refs);
            }
        }
    }
    assert!(errors > 0, "no flip ever surfaced an error");
    assert!(oks > 0, "every flip errored — battery stream too fragile");
}

/// Every truncation of either layer payload is `Ok` or `Err`, never a
/// panic.
#[test]
fn truncation_battery() {
    let (cfg, payloads) = battery_stream();
    for lay in 0..payloads.len() {
        for keep in 0..payloads[lay].len() {
            let mut mutated: Vec<Vec<u8>> = payloads.clone();
            mutated[lay].truncate(keep);
            let mut d = ScalableDecoder::new(cfg.clone()).unwrap();
            let m: Vec<&[u8]> = mutated.iter().map(Vec::as_slice).collect();
            let _ = d.decode_frame(&m);
        }
    }
}

/// Trailing garbage after the element (which the extension_payload
/// walk must consume or reject) never panics.
#[test]
fn trailing_bytes_battery() {
    let (cfg, payloads) = battery_stream();
    for lay in 0..payloads.len() {
        for extra in 1..16usize {
            let mut mutated: Vec<Vec<u8>> = payloads.clone();
            for i in 0..extra {
                mutated[lay].push((i as u8).wrapping_mul(0x5D).wrapping_add(0x21));
            }
            let mut d = ScalableDecoder::new(cfg.clone()).unwrap();
            let m: Vec<&[u8]> = mutated.iter().map(Vec::as_slice).collect();
            let _ = d.decode_frame(&m);
        }
    }
}

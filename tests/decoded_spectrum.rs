//! Integration tests for [`oxideav_aac::decoded_spectrum`] — the
//! per-channel parse → dequant → scalefactor → TNS pipeline stage
//! (§4.6.1.3 / §4.6.2.3.3 / §4.6.3.3 / §4.6.9) — pinning the staged
//! composition against manual per-tool invocations and hand-derived
//! spec-formula vectors.

use oxideav_aac::decoded_spectrum::{decode_channel_spectrum, quant_to_spec};
use oxideav_aac::dequant::{inverse_quantize, rescale_spectrum, scale_factor_gain};
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape};
use oxideav_aac::scale_factor_data::{accumulate, ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData, ZERO_HCB};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::tns_data::{TnsData, TnsFilter, TnsWindow};
use oxideav_aac::tns_frame::tns_decode_frame;
use oxideav_aac::tns_max::AOT_AAC_LC;

/// 44.1 kHz.
const FS: u8 = 4;

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
        num_swb: oxideav_aac::ics_info::NUM_SWB_LONG_WINDOW[FS as usize],
    }
}

fn one_section(codebook: u8, max_sfb: u8) -> SectionData {
    SectionData {
        sections: vec![vec![Section {
            codebook,
            start: 0,
            end: max_sfb,
        }]],
        sfb_cb: vec![vec![codebook; max_sfb as usize]],
    }
}

/// Assemble a minimal long-window `IcsBody` around the given tools.
fn body(
    global_gain: u8,
    ics_info: IcsInfo,
    section_data: SectionData,
    entries: Vec<ScaleFactorEntry>,
    pulse_data: Option<oxideav_aac::pulse_data::PulseData>,
    tns_data: Option<TnsData>,
) -> IcsBody {
    IcsBody {
        global_gain,
        ics_info: Some(ics_info),
        section_data,
        scale_factor_data: ScaleFactorData {
            entries: vec![entries],
        },
        pulse_data_present: pulse_data.is_some(),
        pulse_data,
        tns_data_present: tns_data.is_some(),
        tns_data,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    }
}

// ===========================================================================
// Hand-derived spec-formula vectors
// ===========================================================================

/// §4.6.1.3 + §4.6.2.3.3 worked end-to-end by hand: x_quant = 8 at
/// sf = 104 must come out as 8^(4/3) · 2^(0.25·4) = 16 · 2 = 32.
#[test]
fn pipeline_pins_hand_derived_dequant_values() {
    let info = long_ics_info(2);
    let sd = one_section(11, 2);
    let mut x_quant = vec![0i32; 1024];
    // fs 4 long: band 0 = coefficients 0..4, band 1 = 4..8.
    x_quant[..8].copy_from_slice(&[8, -1, 0, 27, -8, 1, 64, 0]);
    let spectral = SpectralData {
        x_quant: vec![x_quant],
    };
    // global_gain 104 (band 0 sf = 104, gain 2), band 1 dpcm -8
    // (sf = 96, gain 0.5).
    let b = body(
        104,
        info.clone(),
        sd,
        vec![ScaleFactorEntry::Dpcm(0), ScaleFactorEntry::Dpcm(-8)],
        None,
        None,
    );
    let spec = decode_channel_spectrum(&b, &info, &spectral, AOT_AAC_LC, FS).unwrap();
    assert_eq!(spec.len(), 1024);
    // Band 0 at gain 2: 8^(4/3)=16, 1, 0, 27^(4/3)=81.
    assert_eq!(spec[..4], [32.0, -2.0, 0.0, 162.0]);
    // Band 1 at gain 0.5: 8^(4/3)=16, 1, 64^(4/3)=256, 0.
    assert_eq!(spec[4..8], [-8.0, 0.5, 128.0, 0.0]);
    assert!(spec[8..].iter().all(|&v| v == 0.0));
}

/// The pipeline output equals the manual stage-by-stage composition
/// accumulate → rescale_spectrum → quant_to_spec on a tool-free body.
#[test]
fn pipeline_matches_manual_composition_without_tools() {
    let info = long_ics_info(4);
    let sd = SectionData {
        sections: vec![vec![
            Section {
                codebook: 2,
                start: 0,
                end: 2,
            },
            Section {
                codebook: ZERO_HCB,
                start: 2,
                end: 3,
            },
            Section {
                codebook: 11,
                start: 3,
                end: 4,
            },
        ]],
        sfb_cb: vec![vec![2, 2, ZERO_HCB, 11]],
    };
    let mut x_quant = vec![0i32; 1024];
    x_quant[..8].copy_from_slice(&[1, -1, 0, 1, -1, 1, 0, -1]);
    x_quant[12..16].copy_from_slice(&[100, -20, 0, 3]);
    let spectral = SpectralData {
        x_quant: vec![x_quant],
    };
    let entries = vec![
        ScaleFactorEntry::Dpcm(2),
        ScaleFactorEntry::Dpcm(-5),
        ScaleFactorEntry::Dpcm(10),
    ];
    let b = body(120, info.clone(), sd.clone(), entries, None, None);

    let abs = accumulate(&b.scale_factor_data, &sd.sfb_cb, b.global_gain).unwrap();
    let rescaled = rescale_spectrum(&spectral, &abs, &sd.sfb_cb, &info, FS).unwrap();
    let want = quant_to_spec(&rescaled, &info, FS).unwrap();

    let got = decode_channel_spectrum(&b, &info, &spectral, AOT_AAC_LC, FS).unwrap();
    assert_eq!(got, want);
    // Sanity: some energy was produced.
    assert!(got.iter().any(|&v| v != 0.0));
}

// ===========================================================================
// Pulse fix-up ordering
// ===========================================================================

/// §4.6.3.3: the pulse correction applies to the *quantised*
/// spectrum, before inverse quantization. x_quant = 1 plus
/// pulse_amp 7 must dequantise as 8^(4/3) = 16, not as
/// 1^(4/3) + 7^(4/3).
#[test]
fn pulse_fixup_applies_before_inverse_quantization() {
    let info = long_ics_info(1);
    let sd = one_section(11, 1);
    let mut x_quant = vec![0i32; 1024];
    x_quant[0] = 1;
    let spectral = SpectralData {
        x_quant: vec![x_quant],
    };
    let pulse = oxideav_aac::pulse_data::PulseData {
        pulse_start_sfb: 0,
        pulses: vec![oxideav_aac::pulse_data::Pulse { offset: 0, amp: 7 }],
    };
    let b = body(
        100,
        info.clone(),
        sd,
        vec![ScaleFactorEntry::Dpcm(0)],
        Some(pulse),
        None,
    );
    let spec = decode_channel_spectrum(&b, &info, &spectral, AOT_AAC_LC, FS).unwrap();
    assert_eq!(spec[0], 16.0);
    // The input SpectralData must not have been mutated.
    assert_eq!(spectral.x_quant[0][0], 1);
}

// ===========================================================================
// TNS stage
// ===========================================================================

/// With tns_data present the pipeline output equals the manual
/// composition with a trailing tns_decode_frame pass.
#[test]
fn tns_stage_matches_manual_tns_decode_frame() {
    let info = long_ics_info(10);
    let sd = one_section(2, 10);
    let mut x_quant = vec![0i32; 1024];
    for (i, slot) in x_quant.iter_mut().take(40).enumerate() {
        *slot = match i % 3 {
            0 => 1,
            1 => -1,
            _ => 0,
        };
    }
    let spectral = SpectralData {
        x_quant: vec![x_quant],
    };
    let entries: Vec<ScaleFactorEntry> = (0..10).map(|_| ScaleFactorEntry::Dpcm(0)).collect();
    let tns = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                // §4.6.9.3 regions count down from num_swb (49 at fs
                // 4 long); a 49-band length keeps the region alive
                // after the max_sfb = 10 clamp (0..swb_offset[10]).
                length: 49,
                order: 2,
                direction: false,
                coef_compress: false,
                coef: vec![1, 6],
            }],
        }],
    };

    let without = body(110, info.clone(), sd.clone(), entries.clone(), None, None);
    let mut want = decode_channel_spectrum(&without, &info, &spectral, AOT_AAC_LC, FS).unwrap();
    tns_decode_frame(
        &mut want,
        &tns,
        info.window_sequence,
        info.max_sfb,
        AOT_AAC_LC,
        FS,
    )
    .unwrap();

    let with = body(110, info.clone(), sd, entries, None, Some(tns));
    let got = decode_channel_spectrum(&with, &info, &spectral, AOT_AAC_LC, FS).unwrap();
    assert_eq!(got, want);
    // The TNS pass genuinely changed the spectrum.
    let plain = decode_channel_spectrum(&without, &info, &spectral, AOT_AAC_LC, FS).unwrap();
    assert_ne!(got, plain);
}

// ===========================================================================
// Standalone formula spot checks (direct API)
// ===========================================================================

#[test]
fn formula_spot_checks() {
    // §4.6.1.3: Sign(x)·|x|^(4/3).
    assert_eq!(inverse_quantize(-64), -256.0);
    // §4.6.2.3.3: 2^(0.25·(sf − 100)).
    assert_eq!(scale_factor_gain(100), 1.0);
    assert_eq!(scale_factor_gain(104) * scale_factor_gain(96), 1.0);
}

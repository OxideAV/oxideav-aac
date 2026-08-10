//! Integration tests for [`oxideav_aac::cce`] — the
//! `coupling_channel_element()` (ISO/IEC 14496-3 §4.6.8.3 / Table 4.8)
//! whole-element parse, exercising the composition of the coupling
//! header, the embedded `individual_channel_stream(0,0)` body + spectrum,
//! and the trailing gain lists against an assembled bitstream.

use oxideav_aac::cce::{
    CoupledTarget, CouplingChannelElement, CouplingGains, CouplingHeader, GainList,
};
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::tns_max::AOT_AAC_LC;
use oxideav_core::bits::{BitReader, BitWriter};

/// 44.1 kHz.
const FS: u8 = 4;

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
        num_swb: oxideav_aac::ics_info::NUM_SWB_LONG_WINDOW[FS as usize],
    }
}

/// Build a minimal long-window embedded-SCE body + spectrum: `max_sfb`
/// bands of codebook 2 (signed-pair, dimension 4), small in-range
/// coefficients, one scalefactor entry per coded band.
fn embedded_sce(max_sfb: u8) -> (IcsBody, IcsInfo, SpectralData) {
    let info = long_ics_info(max_sfb);
    let sd = SectionData {
        sections: vec![vec![Section {
            codebook: 2,
            start: 0,
            end: max_sfb,
        }]],
        sfb_cb: vec![vec![2u8; max_sfb as usize]],
    };
    let mut x_quant = vec![0i32; 1024];
    // fs=4 long: each band is 4 coefficients; codebook 2 (signed pair)
    // bounds each value to |v| <= 1. Fill only the coded bands.
    let coded = 4 * max_sfb as usize;
    for (i, c) in x_quant[..coded].iter_mut().enumerate() {
        *c = [1, -1, 0, 1][i % 4];
    }
    let spectral = SpectralData {
        x_quant: vec![x_quant],
    };
    let entries: Vec<ScaleFactorEntry> = (0..max_sfb).map(|_| ScaleFactorEntry::Dpcm(0)).collect();
    let body = IcsBody {
        global_gain: 100,
        ics_info: Some(info.clone()),
        section_data: sd,
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
    (body, info, spectral)
}

/// Assemble a whole `coupling_channel_element()` bitstream from its
/// parts and confirm `CouplingChannelElement::parse` recovers each field
/// and consumes exactly the written number of bits.
#[test]
fn whole_cce_round_trips_through_parse() {
    let (body, info, spectral) = embedded_sce(1);

    // A single SCE target with a common-gain list (one transmitted list
    // beyond the implicit list 0).
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
        cc_domain: false,
        gain_element_sign: false,
        gain_element_scale: 2,
        num_gain_element_lists: 2,
    };
    let gains = CouplingGains {
        cc_scale: oxideav_aac::cce::CC_SCALE_TABLE[2],
        gain_element_sign: false,
        lists: vec![GainList::Common(5)],
    };

    let mut writer = BitWriter::new();
    writer.write_u32(7, 4); // element_instance_tag
    header.write(&mut writer).unwrap();
    body.write(&mut writer, AOT_AAC_LC, FS, false).unwrap();
    spectral
        .write(&mut writer, &info, &body.section_data, FS)
        .unwrap();
    gains
        .write(&mut writer, &header, &body.section_data.sfb_cb)
        .unwrap();
    let bits_written = writer.bit_position();
    let bytes = writer.into_bytes();

    let mut reader = BitReader::new(&bytes);
    let cce = CouplingChannelElement::parse(&mut reader, AOT_AAC_LC, FS).unwrap();

    assert_eq!(cce.element_instance_tag, 7);
    assert_eq!(cce.header, header);
    assert_eq!(cce.gains.lists, gains.lists);
    assert_eq!(cce.ics_info.max_sfb, 1);
    assert_eq!(cce.body.global_gain, 100);
    // max_sfb = 1 ⇒ one coded band of 4 coefficients.
    assert_eq!(cce.spectral.x_quant[0][..4], [1, -1, 0, 1]);
    // The parse must consume exactly the bits the assembler emitted.
    assert_eq!(reader.bit_position(), bits_written);
}

/// A CPE target with both `cc_l` and `cc_r` set produces two
/// transmitted gain lists (Table 4.153 split lists); the whole element
/// still round-trips.
#[test]
fn cce_with_split_cpe_lists_round_trips() {
    let (body, info, spectral) = embedded_sce(2);

    let header = CouplingHeader {
        ind_sw_cce_flag: false,
        num_coupled_elements: 0,
        targets: vec![CoupledTarget {
            is_cpe: true,
            tag_select: 2,
            cc_l: true,
            cc_r: true,
        }],
        cc_domain: true,
        gain_element_sign: true,
        gain_element_scale: 1,
        num_gain_element_lists: 2,
    };
    // num_gain_element_lists = 2 ⇒ the Table 4.8 loop `c = 1 ..< 2`
    // transmits exactly one gain list; list 0 is the implicit
    // natural-scaling target.
    let gains = CouplingGains {
        cc_scale: oxideav_aac::cce::CC_SCALE_TABLE[1],
        gain_element_sign: true,
        lists: vec![GainList::Common(-2)],
    };

    let mut writer = BitWriter::new();
    writer.write_u32(0, 4); // element_instance_tag
    header.write(&mut writer).unwrap();
    body.write(&mut writer, AOT_AAC_LC, FS, false).unwrap();
    spectral
        .write(&mut writer, &info, &body.section_data, FS)
        .unwrap();
    gains
        .write(&mut writer, &header, &body.section_data.sfb_cb)
        .unwrap();
    let bits_written = writer.bit_position();
    let bytes = writer.into_bytes();

    let mut reader = BitReader::new(&bytes);
    let cce = CouplingChannelElement::parse(&mut reader, AOT_AAC_LC, FS).unwrap();

    assert_eq!(cce.header.num_gain_element_lists, 2);
    assert_eq!(cce.gains.lists.len(), 1);
    assert_eq!(cce.gains.lists, gains.lists);
    assert_eq!(reader.bit_position(), bits_written);

    // Spot-check the §4.6.8.3.3 cc_gain on the transmitted list
    // (raw = -2: with the sign bit, cc_sign = 1 - 2*(0) = 1, gain =
    // -2>>1 = -1 => cc_scale^(−(−1)) = cc_scale under the
    // conformance-settled negated exponent).
    let g = cce.gains.cc_gain(1, 0, 0).unwrap();
    let expect = oxideav_aac::cce::CC_SCALE_TABLE[1];
    assert!((g - expect).abs() < 1e-12, "cc_gain={g} expect={expect}");
}

/// A `raw_data_block()` of SCE + CCE + END decodes the SCE channel and
/// fully consumes (skips) the CCE — confirming a CCE-bearing stream no
/// longer aborts the whole frame, and that the SCE output is identical to
/// the same SCE decoded without the trailing CCE.
#[test]
fn raw_data_block_with_cce_decodes_sce_and_skips_cce() {
    let (sce_body, sce_info, sce_spectral) = embedded_sce(2);

    // Build the SCE channel-element body bits (the part after the
    // 3-bit id_syn_ele + 4-bit instance tag the walker reads).
    let write_sce = |w: &mut BitWriter| {
        sce_body.write(w, AOT_AAC_LC, FS, false).unwrap();
        sce_spectral
            .write(w, &sce_info, &sce_body.section_data, FS)
            .unwrap();
    };

    // The embedded CCE (header + embedded SCE + a common-gain list).
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
        num_gain_element_lists: 1, // single SCE target, no transmitted list
    };
    let gains = CouplingGains {
        cc_scale: oxideav_aac::cce::CC_SCALE_TABLE[0],
        gain_element_sign: false,
        lists: vec![],
    };
    let (cce_body, cce_info, cce_spectral) = embedded_sce(1);

    let mut w = BitWriter::new();
    // SCE: id_syn_ele = 0b000, element_instance_tag = 0.
    w.write_u32(0, 3);
    w.write_u32(0, 4);
    write_sce(&mut w);
    // CCE: id_syn_ele = 0b010, then the whole Table 4.8 element.
    w.write_u32(2, 3);
    w.write_u32(0, 4); // element_instance_tag
    header.write(&mut w).unwrap();
    cce_body.write(&mut w, AOT_AAC_LC, FS, false).unwrap();
    cce_spectral
        .write(&mut w, &cce_info, &cce_body.section_data, FS)
        .unwrap();
    gains
        .write(&mut w, &header, &cce_body.section_data.sfb_cb)
        .unwrap();
    // END: id_syn_ele = 0b111, then byte align.
    w.write_u32(7, 3);
    w.align_to_byte();
    let payload = w.into_bytes();

    // The SCE channel must be byte-identical to the same SCE decoded
    // without the trailing CCE.
    let mut w2 = BitWriter::new();
    w2.write_u32(0, 3);
    w2.write_u32(0, 4);
    write_sce(&mut w2);
    w2.write_u32(7, 3);
    w2.align_to_byte();
    let sce_only = w2.into_bytes();
    let mut dec2 = StreamDecoder::new();
    let frame2 = dec2
        .decode_raw_data_block(AOT_AAC_LC, FS, 44100, 1, 1, &sce_only)
        .expect("SCE-only block decodes");

    let mut dec = StreamDecoder::new();
    let frame = dec
        .decode_raw_data_block(AOT_AAC_LC, FS, 44100, 1, 1, &payload)
        .expect("SCE+CCE block decodes");
    assert_eq!(frame.channels, 1, "the CCE contributes no output channel");
    assert_eq!(frame.pcm.len(), 1024);
    assert_eq!(frame.pcm, frame2.pcm, "CCE skip must not perturb the SCE");
}

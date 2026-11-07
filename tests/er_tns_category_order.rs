//! ER `tns_data()` placement — ISO/IEC 14496-3 §4.5.2.4.2.
//!
//! The error-resilient payload stores each `individual_channel_stream()`
//! by error sensitivity category (Table 4.95 "channel dependent
//! information … dependency structure according to category numbers",
//! Table 4.148 assignment), the logical order kept inside each category
//! instance. `tns_data()` is category 3 while `tns_data_present`,
//! `gain_control_data_present` and the HCR length fields are category 1,
//! so on the ER wire the TNS record sits after the whole category-1 run
//! and directly before the category-4 spectral payload — at the literal
//! Table 4.155 widths (2 / 6 / 5 on every long window, LD included).
//!
//! The pins:
//!
//! * an ER AAC LC (AOT 17) stream carrying real TNS filters decodes
//!   bit-identically to the AAC-LC `raw_data_block()` of the same
//!   spectra + filters (the TNS tool itself is AOT-independent), with
//!   and without the HCR spectral resilience branch;
//! * ER AAC LD 512 / 480 frames with filters only the 6-bit `length` /
//!   5-bit `order` / 2-bit `n_filt` widths can express round-trip
//!   through [`IcsBody::write_er`] / [`IcsBody::parse_er_family`] and
//!   change the decoded PCM (the filter is applied);
//! * the syntax-order placement (TNS between the tool flags) is *not*
//!   what the decoder reads.
//!
//! Black-box corroboration (r483): ER AAC LD 512 streams produced by
//! two independent black-box encoders (`fdkaac -p 23`, `afconvert -d
//! aacl`) at 22.05–48 kHz, mono and stereo, decode within 1 LSB of the
//! `afconvert` decode on every access unit, TNS-bearing ones included.
//! With `OXIDEAV_AAC_PROBE_OUT` set the LD streams built here are
//! written there as length-prefixed access units for the same check.

use oxideav_aac::asc::AacResilienceFlags;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::hcr_decode::encode_reordered_spectral_data;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, derive_window_grouping_family};
use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::swb_offset::{FrameFamily, long_window_offsets_family};
use oxideav_aac::tns_data::{TnsData, TnsFilter, TnsWindow};
use oxideav_core::bits::{BitReader, BitWriter};

const FS_INDEX: u8 = 3; // 48 kHz
const SAMPLE_RATE: u32 = 48_000;
const AOT_LC: u8 = 2;
const AOT_ER_LC: u8 = 17;
const AOT_ER_LD: u8 = 23;

const NO_RESILIENCE: AacResilienceFlags = AacResilienceFlags {
    section_data: false,
    scalefactor_data: false,
    spectral_data: false,
};
const HCR_RESILIENCE: AacResilienceFlags = AacResilienceFlags {
    section_data: false,
    scalefactor_data: false,
    spectral_data: true,
};

fn long_ics(family: FrameFamily, max_sfb: u8, shape: WindowShape) -> IcsInfo {
    let (num_windows, num_window_groups, window_group_length, num_swb) =
        derive_window_grouping_family(family, WindowSequence::OnlyLong, None, FS_INDEX).unwrap();
    IcsInfo {
        family,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: shape,
        max_sfb,
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
    }
}

/// One long-window channel over per-band codebooks `cbs` with a
/// deterministic pseudo-random spectrum.
fn channel(ics: &IcsInfo, cbs: &[u8], seed: u32, tns: Option<TnsData>) -> (IcsBody, SpectralData) {
    assert_eq!(cbs.len(), usize::from(ics.max_sfb));
    let sections = vec![
        cbs.iter()
            .enumerate()
            .map(|(sfb, &cb)| Section {
                codebook: cb,
                start: sfb as u8,
                end: sfb as u8 + 1,
            })
            .collect(),
    ];
    let entries = vec![
        cbs.iter()
            .filter(|&&cb| cb != 0)
            .map(|_| ScaleFactorEntry::Dpcm(0))
            .collect(),
    ];
    let body = IcsBody {
        global_gain: 140,
        ics_info: Some(ics.clone()),
        section_data: SectionData {
            sections,
            sfb_cb: vec![cbs.to_vec()],
        },
        scale_factor_data: ScaleFactorData { entries },
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: tns.is_some(),
        tns_data: tns,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    let offsets = long_window_offsets_family(ics.family, FS_INDEX).unwrap();
    let mut state = seed;
    let mut prand = |max: i32| {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        ((state >> 8) % (2 * max + 1) as u32) as i32 - max
    };
    let mut coeffs = vec![0i32; ics.family.frame_len()];
    for (sfb, &cb) in cbs.iter().enumerate() {
        let max = match cb {
            0 => 0,
            1 | 2 => 1,
            3 | 4 => 2,
            5 | 6 => 4,
            7 | 8 => 7,
            9 | 10 => 12,
            _ => 25,
        };
        for c in &mut coeffs[usize::from(offsets[sfb])..usize::from(offsets[sfb + 1])] {
            *c = prand(max);
        }
    }
    (
        body,
        SpectralData {
            x_quant: vec![coeffs],
        },
    )
}

/// Two filters, a top filter longer than 15 bands and an order above
/// 7 — only the 2 / 6 / 5-bit widths can carry it.
fn wide_tns() -> TnsData {
    TnsData {
        windows: vec![TnsWindow {
            coef_res: true,
            filters: vec![
                TnsFilter {
                    length: 20,
                    order: 9,
                    direction: false,
                    coef_compress: false,
                    coef: vec![9, 4, 13, 2, 0, 1, 15, 3, 1],
                },
                TnsFilter {
                    length: 6,
                    order: 3,
                    direction: true,
                    coef_compress: true,
                    coef: vec![3, 1, 6],
                },
            ],
        }],
    }
}

/// An ER single-channel access unit (Table 4.19: 4-bit tag + ER body
/// in category order + the spectral payload).
fn er_sce(aot: u8, body: &IcsBody, spectral: &SpectralData, res: AacResilienceFlags) -> Vec<u8> {
    let ics = body.ics_info.as_ref().unwrap();
    let mut body = body.clone();
    let mut hcr = None;
    if res.spectral_data {
        let (payload, len_bits, longest) =
            encode_reordered_spectral_data(spectral, ics, &body.section_data, FS_INDEX).unwrap();
        body.reordered_spectral_lengths = Some((len_bits, longest));
        hcr = Some((payload, len_bits));
    }
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4);
    body.write_er(&mut bw, aot, FS_INDEX, res).unwrap();
    match hcr {
        Some((payload, len_bits)) => {
            for i in 0..usize::from(len_bits) {
                bw.write_bit(payload[i / 8] & (0x80 >> (i % 8)) != 0);
            }
        }
        None => spectral
            .write(&mut bw, ics, &body.section_data, FS_INDEX)
            .unwrap(),
    }
    bw.finish()
}

/// The equivalent AAC-LC `raw_data_block()` ([SCE, END]).
fn lc_sce(body: &IcsBody, spectral: &SpectralData) -> Vec<u8> {
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let mut bw = BitWriter::new();
    body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
    let ics = body.ics_info.as_ref().unwrap();
    spectral
        .write(&mut bw, ics, &body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    fa.push_end()
}

const CBS: &[u8] = &[
    11, 9, 7, 5, 3, 1, 2, 4, 6, 8, 10, 11, 7, 5, 3, 1, 1, 2, 3, 4, 5, 6,
];

#[test]
fn er_lc_tns_decodes_like_lc_tns() {
    for res in [NO_RESILIENCE, HCR_RESILIENCE] {
        let mut er = StreamDecoder::new();
        let mut lc = StreamDecoder::new();
        for f in 0..4u32 {
            let ics = long_ics(FrameFamily::Lc1024, CBS.len() as u8, WindowShape::Sine);
            let tns = (f % 2 == 1).then(wide_tns);
            let (body, spectral) = channel(&ics, CBS, 0x7100 + f, tns);
            let a = er
                .decode_er_raw_data_block(
                    AOT_ER_LC,
                    FS_INDEX,
                    SAMPLE_RATE,
                    1,
                    res,
                    &er_sce(AOT_ER_LC, &body, &spectral, res),
                )
                .unwrap();
            let b = lc
                .decode_raw_data_block(
                    AOT_LC,
                    FS_INDEX,
                    SAMPLE_RATE,
                    1,
                    1,
                    &lc_sce(&body, &spectral),
                )
                .unwrap();
            assert_eq!(a.pcm, b.pcm, "frame {f} ({res:?}): ER vs LC TNS decode");
        }
    }
}

fn ld_stream(family: FrameFamily, with_tns: bool) -> Vec<Vec<u8>> {
    let num_swb = long_ics(family, 0, WindowShape::Sine).num_swb;
    let cbs = &CBS[..CBS.len().min(usize::from(num_swb))];
    (0..5u32)
        .map(|f| {
            let shape = if f == 2 {
                WindowShape::Kbd
            } else {
                WindowShape::Sine
            };
            let ics = long_ics(family, cbs.len() as u8, shape);
            let tns = (with_tns && f >= 1).then(wide_tns);
            let (body, spectral) = channel(&ics, cbs, 0x2d00 + f, tns);
            er_sce(AOT_ER_LD, &body, &spectral, NO_RESILIENCE)
        })
        .collect()
}

fn decode_ld(family: FrameFamily, aus: &[Vec<u8>]) -> Vec<i16> {
    let mut dec = StreamDecoder::new();
    dec.set_frame_family(family);
    let mut out = Vec::new();
    for au in aus {
        let f = dec
            .decode_er_raw_data_block(AOT_ER_LD, FS_INDEX, SAMPLE_RATE, 1, NO_RESILIENCE, au)
            .unwrap();
        assert_eq!(f.pcm.len(), family.frame_len());
        out.extend_from_slice(&f.pcm);
    }
    out
}

#[test]
fn ld_tns_filters_round_trip_and_apply() {
    for family in [FrameFamily::Ld512, FrameFamily::Ld480] {
        let aus = ld_stream(family, true);
        // The record parses back at the category-3 position with the
        // literal widths.
        for au in &aus[1..] {
            let mut r = BitReader::new(au);
            r.read_u32(4).unwrap();
            let body =
                IcsBody::parse_er_family(&mut r, family, AOT_ER_LD, FS_INDEX, false, NO_RESILIENCE)
                    .unwrap();
            assert_eq!(body.tns_data, Some(wide_tns()), "{family:?}");
        }
        let with = decode_ld(family, &aus);
        let without = decode_ld(family, &ld_stream(family, false));
        let n = family.frame_len();
        assert_eq!(with[..n], without[..n], "{family:?}: TNS-free first frame");
        assert_ne!(with[n..], without[n..], "{family:?}: TNS must apply");
        if let Some(dir) = std::env::var_os("OXIDEAV_AAC_PROBE_OUT") {
            let mut o = Vec::new();
            for au in &aus {
                o.extend_from_slice(&(au.len() as u32).to_be_bytes());
                o.extend_from_slice(au);
            }
            let dir = std::path::PathBuf::from(dir);
            std::fs::write(dir.join(format!("{family:?}.aus")), o).unwrap();
            let pcm: Vec<u8> = with.iter().flat_map(|s| s.to_le_bytes()).collect();
            std::fs::write(dir.join(format!("{family:?}.s16")), pcm).unwrap();
        }
    }
}

#[test]
fn syntax_order_tns_placement_is_not_the_er_wire() {
    // Write the TNS body between the tool flags (the non-ER Table 4.50
    // order) on an LD frame: the ER parse must not recover the filter.
    let family = FrameFamily::Ld512;
    let ics = long_ics(family, CBS.len() as u8, WindowShape::Sine);
    let (body, spectral) = channel(&ics, CBS, 0x3300, Some(wide_tns()));
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4);
    body.write(&mut bw, AOT_ER_LD, FS_INDEX, false).unwrap();
    spectral
        .write(&mut bw, &ics, &body.section_data, FS_INDEX)
        .unwrap();
    let au = bw.finish();
    let mut r = BitReader::new(&au);
    r.read_u32(4).unwrap();
    let parsed =
        IcsBody::parse_er_family(&mut r, family, AOT_ER_LD, FS_INDEX, false, NO_RESILIENCE);
    assert!(parsed.map_or(true, |b| b.tns_data != Some(wide_tns())));
}

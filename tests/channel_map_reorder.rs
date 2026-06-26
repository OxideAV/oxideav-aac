//! End-to-end validation of the Table 1.19 default-config channel
//! reorder (ISO/IEC 14496-3 §1.6.3.5) as applied by
//! [`oxideav_aac::decode::StreamDecoder`].
//!
//! The unit tests in `src/channel_map.rs` pin the permutation tables;
//! these tests pin the *driver wiring* — that
//! `decode_raw_data_block(..., channel_configuration, ...)` actually
//! permutes the decoded element-order channels into the canonical
//! interleaved layout, and that an unmapped configuration leaves them in
//! element order.
//!
//! A frame is assembled from N independent single-channel elements
//! (SCE / LFE), each given a distinct `global_gain` so its decoded PCM is
//! distinguishable from the others. The same payload is decoded twice —
//! once with `channelConfiguration == 0` (no reorder, element order
//! preserved) and once with the default `channelConfiguration` under test
//! — and the reordered output is checked to be exactly the element-order
//! output permuted by [`oxideav_aac::channel_map::reorder_permutation`].
//! Because the reorder operates on the decoded per-channel buffers and is
//! independent of which syntactic element produced each channel, using
//! single-channel elements is sufficient to exercise the full driver path
//! without hand-assembling a `channel_pair_element()`.

use oxideav_aac::channel_map::{reorder_channels, reorder_permutation};
use oxideav_aac::decode::{DecodedFrame, StreamDecoder, FRAME_LEN};
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape};
use oxideav_aac::raw_data_block::IdSynEle;
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::tns_max::AOT_AAC_LC;
use oxideav_core::bits::BitWriter;

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

/// A minimal long-window single-channel body + spectrum with a caller-set
/// `global_gain` (so each element decodes to a distinct PCM signal).
fn mono_element(global_gain: u8) -> (IcsBody, IcsInfo, SpectralData) {
    let max_sfb = 2u8;
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
    let coded = 4 * max_sfb as usize;
    for (i, c) in x_quant[..coded].iter_mut().enumerate() {
        *c = [1, -1, 0, 1][i % 4];
    }
    let spectral = SpectralData {
        x_quant: vec![x_quant],
    };
    let entries: Vec<ScaleFactorEntry> = (0..max_sfb).map(|_| ScaleFactorEntry::Dpcm(0)).collect();
    let body = IcsBody {
        global_gain,
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

fn id_bits(kind: IdSynEle) -> u32 {
    match kind {
        IdSynEle::Sce => 0,
        IdSynEle::Cpe => 1,
        IdSynEle::Cce => 2,
        IdSynEle::Lfe => 3,
        IdSynEle::Dse => 4,
        IdSynEle::Pce => 5,
        IdSynEle::Fil => 6,
        IdSynEle::End => 7,
    }
}

/// Assemble a `raw_data_block()` of single-channel elements: one element
/// per `(kind, tag, global_gain)` triple, then END + byte align.
fn assemble(elements: &[(IdSynEle, u8, u8)]) -> Vec<u8> {
    let mut w = BitWriter::new();
    for &(kind, tag, gain) in elements {
        let (body, info, spectral) = mono_element(gain);
        w.write_u32(id_bits(kind), 3);
        w.write_u32(u32::from(tag), 4);
        body.write(&mut w, AOT_AAC_LC, FS, false).unwrap();
        spectral
            .write(&mut w, &info, &body.section_data, FS)
            .unwrap();
    }
    w.write_u32(7, 3); // END
    w.align_to_byte();
    w.into_bytes()
}

/// De-interleave a [`DecodedFrame`] into per-channel s16 buffers.
fn deinterleave(frame: &DecodedFrame) -> Vec<Vec<i16>> {
    let ch = frame.channels;
    assert_eq!(frame.pcm.len(), FRAME_LEN * ch, "frame length geometry");
    (0..ch)
        .map(|c| frame.pcm.iter().skip(c).step_by(ch).copied().collect())
        .collect()
}

/// Re-interleave per-channel buffers (the inverse of [`deinterleave`]).
fn interleave(channels: &[Vec<i16>]) -> Vec<i16> {
    if channels.is_empty() {
        return Vec::new();
    }
    let frame_len = channels[0].len();
    let mut out = Vec::with_capacity(frame_len * channels.len());
    for n in 0..frame_len {
        for c in channels {
            out.push(c[n]);
        }
    }
    out
}

/// Decode the same payload with `channelConfiguration == 0` (element
/// order) and with `cfg`, and confirm the driver's reordered interleaved
/// PCM equals the element-order channels permuted by
/// [`channel_map::reorder_channels`] — i.e. the decode driver applies
/// exactly the library permutation. This holds independently of whether
/// the individual element signals are mutually distinguishable.
fn assert_reorder_matches(cfg: u8, elements: &[(IdSynEle, u8, u8)]) {
    let payload = assemble(elements);

    let mut dec0 = StreamDecoder::new();
    let element_order = dec0
        .decode_raw_data_block(AOT_AAC_LC, FS, 44100, 0, 1, &payload)
        .expect("element-order decode");

    let mut dec = StreamDecoder::new();
    let reordered = dec
        .decode_raw_data_block(AOT_AAC_LC, FS, 44100, cfg, 1, &payload)
        .expect("reordered decode");

    assert_eq!(element_order.channels, elements.len());
    assert_eq!(reordered.channels, elements.len());

    // Apply the library permutation to the element-order channels and
    // re-interleave; that is exactly what the driver must produce.
    let elem_ch = deinterleave(&element_order);
    let expected = interleave(&reorder_channels(cfg, elem_ch.clone()));
    assert_eq!(
        reordered.pcm, expected,
        "cfg {cfg}: driver output must equal reorder_channels of element order"
    );

    // And the permutation must be the non-trivial Table 1.19 one (guards
    // against a silently-skipped reorder for the multichannel configs).
    let perm = reorder_permutation(cfg).expect("cfg has a permutation");
    assert_eq!(perm.len(), elements.len());

    // The per-channel signals must be mutually distinguishable, so the
    // equivalence check above is not vacuous: a wrong permutation would
    // actually move an observably-different signal into the wrong slot.
    for i in 1..elem_ch.len() {
        assert_ne!(
            elem_ch[0], elem_ch[i],
            "elements 0 and {i} decoded identically; bump the gains apart"
        );
    }
}

#[test]
fn surround51_reorders_six_channels_into_canonical_order() {
    // cfg 6 element layout is SCE, CPE, CPE, LFE (= 6 channels). The
    // reorder operates on the decoded channel buffers, so six distinct
    // single-channel elements exercise the same driver path. Distinct
    // global_gains keep the six signals separable.
    let elements = [
        (IdSynEle::Sce, 0, 156),
        (IdSynEle::Sce, 1, 161),
        (IdSynEle::Sce, 2, 166),
        (IdSynEle::Sce, 3, 171),
        (IdSynEle::Sce, 4, 176),
        (IdSynEle::Lfe, 0, 181),
    ];
    assert_reorder_matches(6, &elements);
}

#[test]
fn surround50_reorders_five_channels() {
    let elements = [
        (IdSynEle::Sce, 0, 158),
        (IdSynEle::Sce, 1, 163),
        (IdSynEle::Sce, 2, 168),
        (IdSynEle::Sce, 3, 173),
        (IdSynEle::Sce, 4, 178),
    ];
    assert_reorder_matches(5, &elements);
}

#[test]
fn surround30_reorders_three_channels() {
    let elements = [
        (IdSynEle::Sce, 0, 160),
        (IdSynEle::Sce, 1, 168),
        (IdSynEle::Sce, 2, 176),
    ];
    assert_reorder_matches(3, &elements);
}

#[test]
fn stereo_is_unchanged_by_reorder() {
    // cfg 2 is an identity permutation: the reordered output must be
    // byte-identical to the element-order output.
    let elements = [(IdSynEle::Sce, 0, 100), (IdSynEle::Sce, 1, 120)];
    let payload = assemble(&elements);

    let mut dec0 = StreamDecoder::new();
    let a = dec0
        .decode_raw_data_block(AOT_AAC_LC, FS, 44100, 0, 1, &payload)
        .unwrap();
    let mut dec2 = StreamDecoder::new();
    let b = dec2
        .decode_raw_data_block(AOT_AAC_LC, FS, 44100, 2, 1, &payload)
        .unwrap();
    assert_eq!(a.pcm, b.pcm, "stereo reorder must be a no-op");
}

#[test]
fn config_zero_leaves_six_channels_in_element_order() {
    // A PCE-defined (cfg 0) layout is not reordered: cfg 0 must equal the
    // raw element order (trivially true, but pins the contract).
    let elements = [
        (IdSynEle::Sce, 0, 156),
        (IdSynEle::Sce, 1, 161),
        (IdSynEle::Sce, 2, 166),
        (IdSynEle::Sce, 3, 171),
        (IdSynEle::Sce, 4, 176),
        (IdSynEle::Lfe, 0, 181),
    ];
    let payload = assemble(&elements);
    let mut dec = StreamDecoder::new();
    let frame = dec
        .decode_raw_data_block(AOT_AAC_LC, FS, 44100, 0, 1, &payload)
        .unwrap();
    assert_eq!(frame.channels, 6);
    // cfg 0 has no permutation; reorder_channels is the identity, so the
    // driver output equals the un-permuted element-order interleave.
    let chans = deinterleave(&frame);
    // The six signals stay separable and in their emitted order.
    for i in 1..chans.len() {
        assert_ne!(chans[0], chans[i], "elements 0 and {i} not separable");
    }
    let passthrough = interleave(&reorder_channels(0, chans));
    assert_eq!(frame.pcm, passthrough);
    assert_eq!(reorder_permutation(0), None);
}

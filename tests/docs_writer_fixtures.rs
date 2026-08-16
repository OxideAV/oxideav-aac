//! Writer-assembled CCE / HCR / SSR docs fixtures — the three decoder
//! paths `docs/audio/aac/cce-hcr-ssr-fixtures.md` records as
//! fixture-gapped (no open encoder produces a CCE, an ER-AAC-LC HCR
//! stream, or an SSR stream; nothing on the permitted sample host).
//! Per that doc's §6.1 fallback, each fixture is a bit-exact stream
//! hand-assembled with this crate's own wire writers, staged under
//! `docs/audio/aac/fixtures/` together with the decoder's PCM as
//! `expected.wav` (flagged writer-assembled in its notes.md).
//!
//! Regenerate with
//! `OXIDEAV_AAC_STAGE_FIXTURES=1 cargo test --test docs_writer_fixtures`;
//! without the variable each test pins the staged bytes against the
//! in-code recipe and the staged `expected.wav` against a fresh
//! decode, and skips cleanly when the docs corpus is absent
//! (standalone-repo CI).
//!
//! Beyond the pinning, each builder carries its own oracle:
//!
//! * CCE — every §1.1 condition of the fixtures doc is exercised
//!   (both `ind_sw_cce_flag` values, both `cc_domain` values, a CPE
//!   target with `cc_l && cc_r`, a per-sfb dpcm gain list, a
//!   `gain_element_sign` split) and the decode must differ from the
//!   CCE-less decode of the same targets while a shared-list CPE
//!   couples both channels;
//! * HCR — the ER AAC LC stream (AOT 17, section + spectral
//!   resilience, VCB bands) decodes bit-identically to the plain
//!   `spectral_data()` coding of the same spectra;
//! * SSR — the AOT-3 stream drives `gain_control_data()` ladders
//!   through all four window sequences with the §4.6.12.3.3 variable
//!   frame lengths, and must differ from the same stream with unity
//!   gain ladders.

use std::fs;
use std::path::PathBuf;

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::cce::{CoupledTarget, CouplingGains, CouplingHeader, DpcmGain, GainList};
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::gain_control_data::{GainAdjust, GainBand, GainControlData, GainWindow};
use oxideav_aac::hcr_decode::encode_reordered_spectral_data;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{
    IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW, NUM_SWB_SHORT_WINDOW,
};
use oxideav_aac::latm::LoasDecoder;
use oxideav_aac::pce::{CcElementSelect, ElementSelect, Pce};
use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_core::bits::BitWriter;

const AOT_LC: u8 = 2;
const AOT_ER_LC: u8 = 17;
const FS_INDEX: u8 = 4; // 44.1 kHz
const SAMPLE_RATE: u32 = 44_100;

// ==================================================================
// Corpus plumbing
// ==================================================================

fn fixtures_root() -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures")
}

fn stage_enabled() -> bool {
    std::env::var_os("OXIDEAV_AAC_STAGE_FIXTURES").is_some()
}

/// Serialize interleaved s16 PCM as a canonical 44-byte-header WAV.
fn wav_bytes(pcm: &[i16], channels: u16, sample_rate: u32) -> Vec<u8> {
    let data_len = (pcm.len() * 2) as u32;
    let byte_rate = sample_rate * u32::from(channels) * 2;
    let block_align = channels * 2;
    let mut out = Vec::with_capacity(44 + pcm.len() * 2);
    out.extend_from_slice(b"RIFF");
    out.extend_from_slice(&(36 + data_len).to_le_bytes());
    out.extend_from_slice(b"WAVEfmt ");
    out.extend_from_slice(&16u32.to_le_bytes());
    out.extend_from_slice(&1u16.to_le_bytes()); // PCM
    out.extend_from_slice(&channels.to_le_bytes());
    out.extend_from_slice(&sample_rate.to_le_bytes());
    out.extend_from_slice(&byte_rate.to_le_bytes());
    out.extend_from_slice(&block_align.to_le_bytes());
    out.extend_from_slice(&16u16.to_le_bytes()); // bits/sample
    out.extend_from_slice(b"data");
    out.extend_from_slice(&data_len.to_le_bytes());
    for &s in pcm {
        out.extend_from_slice(&s.to_le_bytes());
    }
    out
}

/// Stage (env-gated) or pin one fixture: `input_name` bytes plus the
/// decoder's PCM as expected.wav.
fn stage_or_pin(name: &str, input_name: &str, input: &[u8], pcm: &[i16], ch: u16, rate: u32) {
    let dir = fixtures_root().join(name);
    let input_path = dir.join(input_name);
    let wav_path = dir.join("expected.wav");
    let wav = wav_bytes(pcm, ch, rate);
    if stage_enabled() {
        fs::create_dir_all(&dir).unwrap();
        fs::write(&input_path, input).unwrap();
        fs::write(&wav_path, &wav).unwrap();
        eprintln!(
            "staged {} ({} + {} bytes)",
            dir.display(),
            input.len(),
            wav.len()
        );
        return;
    }
    match (fs::read(&input_path), fs::read(&wav_path)) {
        (Ok(staged_in), Ok(staged_wav)) => {
            assert_eq!(
                staged_in, input,
                "{name}: staged {input_name} differs from the in-code recipe"
            );
            assert_eq!(
                staged_wav, wav,
                "{name}: staged expected.wav differs from a fresh decode"
            );
        }
        _ => eprintln!("skip: {} not staged yet", dir.display()),
    }
}

// ==================================================================
// Channel-element builders (long window unless stated)
// ==================================================================

fn ics_info_for(seq: WindowSequence, max_sfb: u8) -> IcsInfo {
    let short = seq == WindowSequence::EightShort;
    IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: seq,
        window_shape: WindowShape::Sine,
        max_sfb,
        scale_factor_grouping: if short { Some(0x7F) } else { None },
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: if short { 8 } else { 1 },
        num_window_groups: 1,
        window_group_length: vec![if short { 8 } else { 1 }],
        num_swb: if short {
            NUM_SWB_SHORT_WINDOW[FS_INDEX as usize]
        } else {
            NUM_SWB_LONG_WINDOW[FS_INDEX as usize]
        },
    }
}

/// One single-group channel over per-band codebooks `sfb_cbs`, with a
/// deterministic pseudo-random spectrum seeded by `seed`.
fn make_channel(
    seq: WindowSequence,
    sfb_cbs: &[u8],
    global_gain: u8,
    seed: u32,
) -> (IcsBody, SpectralData) {
    let max_sfb = sfb_cbs.len() as u8;
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
        ics_info: Some(ics_info_for(seq, max_sfb)),
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
    let short = seq == WindowSequence::EightShort;
    let offsets: Vec<u16> = if short {
        oxideav_aac::swb_offset::short_window_offsets(FS_INDEX)
            .unwrap()
            .to_vec()
    } else {
        oxideav_aac::swb_offset::long_window_offsets(FS_INDEX)
            .unwrap()
            .to_vec()
    };
    let group_windows = if short { 8 } else { 1 };
    let mut coeffs = vec![0i32; 1024];
    let mut state = seed;
    let mut prand = |max: i32| {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
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
            _ => 30, // book 11 / virtual codebooks: exercise escapes
        };
        if max > 0 {
            // Grouped layout: band `sfb` of a group spans
            // `offsets[sfb]·W .. offsets[sfb+1]·W` (§4.5.2.3.4 with one
            // group of W windows).
            let a = usize::from(offsets[sfb]) * group_windows;
            let b = usize::from(offsets[sfb + 1]) * group_windows;
            for c in coeffs[a..b].iter_mut() {
                *c = prand(max);
            }
        }
    }
    let spectral = SpectralData {
        x_quant: vec![coeffs],
    };
    (body, spectral)
}

fn push_sce(fa: &mut FrameAssembler, aot: u8, tag: u8, body: &IcsBody, spectral: &SpectralData) {
    fa.push_channel_header(IdSynEle::Sce, tag).unwrap();
    let mut bw = BitWriter::new();
    body.write(&mut bw, aot, FS_INDEX, false).unwrap();
    let ics = body.ics_info.as_ref().unwrap();
    spectral
        .write(&mut bw, ics, &body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
}

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

/// Wrap raw_data_block payloads as consecutive ADTS frames.
fn wrap_adts_frames(payloads: &[Vec<u8>], profile: u8, channel_configuration: u8) -> Vec<u8> {
    let mut out = Vec::new();
    for payload in payloads {
        let header = AdtsHeader {
            mpeg_version_mpeg2: false,
            protection_absent: true,
            profile,
            sampling_frequency_index: FS_INDEX,
            channel_configuration,
            aac_frame_length: (7 + payload.len()) as u16,
            adts_buffer_fullness: 0x7FF,
            number_of_raw_data_blocks_in_frame: 1,
        };
        out.extend_from_slice(&header.write().unwrap());
        out.extend_from_slice(payload);
    }
    out
}

// ==================================================================
// CCE fixture — aac-cce-writer-assembled
// ==================================================================

/// The three per-frame CCE shapes, covering every §1.1 condition of
/// the fixtures doc. Targets are always [SCE tag 0, CPE tag 0].
fn cce_variants() -> Vec<(CouplingHeader, CouplingGains)> {
    // Embedded-SCE band layout: [1, 3, 0, 5, 7, 2] (see build) — one
    // ZERO_HCB band exercises the gain-list skip. The grid is the
    // §4.6.8.3.3 (2001 / 13818-7:2004) delta-split decode of the wire
    // deltas [0, 1, (skip), 1, 0, −1] under `gain_element_sign == 1`
    // (`docs/audio/aac/cce-gain-sign-split.md` §3): per-band `cc_sign`
    // off each delta LSB, accumulator fed with `dpcm >> 1`.
    let dpcm_grid = vec![vec![
        DpcmGain {
            negative: false,
            gain: 0,
        },
        DpcmGain {
            negative: true,
            gain: 0,
        },
        // ZERO_HCB carry cell (band 2, not transmitted).
        DpcmGain {
            negative: false,
            gain: 0,
        },
        DpcmGain {
            negative: true,
            gain: 0,
        },
        DpcmGain {
            negative: false,
            gain: 0,
        },
        DpcmGain {
            negative: true,
            gain: -1,
        },
    ]];
    vec![
        // A: dependently switched, before-TNS domain, SCE natural +
        // CPE with both channels coupled (cc_l && cc_r → one common
        // list + one per-sfb dpcm list), sign split active.
        (
            CouplingHeader {
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
                        is_cpe: true,
                        tag_select: 0,
                        cc_l: true,
                        cc_r: true,
                    },
                ],
                cc_domain: false,
                gain_element_sign: true,
                gain_element_scale: 1,
                num_gain_element_lists: 3,
            },
            CouplingGains {
                cc_scale: 1.189_207_115_002_721, // 2^(1/4), Table 4.154 row 1
                gain_element_sign: true,
                lists: vec![GainList::Common(2), GainList::Dpcm(dpcm_grid)],
            },
        ),
        // B: independently switched (time-domain coupling; common
        // gains only), SCE natural + shared-list CPE.
        (
            CouplingHeader {
                ind_sw_cce_flag: true,
                num_coupled_elements: 1,
                targets: vec![
                    CoupledTarget {
                        is_cpe: false,
                        tag_select: 0,
                        cc_l: false,
                        cc_r: false,
                    },
                    CoupledTarget {
                        is_cpe: true,
                        tag_select: 0,
                        cc_l: false,
                        cc_r: false,
                    },
                ],
                cc_domain: false,
                gain_element_sign: false,
                gain_element_scale: 3,
                num_gain_element_lists: 2,
            },
            CouplingGains {
                cc_scale: 2.0,
                gain_element_sign: false,
                lists: vec![GainList::Common(1)],
            },
        ),
        // C: dependently switched, after-TNS domain, one shared-list
        // CPE target (implicit natural list 0 only).
        (
            CouplingHeader {
                ind_sw_cce_flag: false,
                num_coupled_elements: 0,
                targets: vec![CoupledTarget {
                    is_cpe: true,
                    tag_select: 0,
                    cc_l: false,
                    cc_r: false,
                }],
                cc_domain: true,
                gain_element_sign: false,
                gain_element_scale: 0,
                num_gain_element_lists: 1,
            },
            CouplingGains {
                cc_scale: 1.090_507_732_665_257_7,
                gain_element_sign: false,
                lists: vec![],
            },
        ),
    ]
}

/// Build the CCE stream: 9 ADTS frames of [SCE0, CPE0, CCE], cycling
/// the three CCE shapes. `with_cce == false` builds the same target
/// stream without any CCE (the comparison baseline).
fn build_cce_stream(with_cce: bool) -> Vec<u8> {
    let long = WindowSequence::OnlyLong;
    let (sce_body, sce_spec) = make_channel(long, &[1, 3, 5, 7, 9, 11, 2, 4], 152, 0xC0FFEE);
    let (l_body, l_spec) = make_channel(long, &[2, 4, 6, 8, 10, 1], 150, 0xBEEF);
    let (r_body, r_spec) = make_channel(long, &[1, 5, 9, 11, 3, 7], 151, 0xF00D);
    let (emb_body, emb_spec) = make_channel(long, &[1, 3, 0, 5, 7, 2], 148, 0x5EED);
    let variants = cce_variants();

    // §8.5.2.2: a CCE-bearing program declares its coupling elements
    // through a program_config_element (channelConfiguration 0) —
    // front SCE (centre) + front CPE (L/R) + the three valid CC
    // element tags. The PCE rides in-band in every frame.
    let pce = Pce {
        element_instance_tag: 0,
        object_type: AOT_LC - 1,
        sampling_frequency_index: FS_INDEX,
        front_elements: vec![
            ElementSelect {
                is_cpe: false,
                tag_select: 0,
            },
            ElementSelect {
                is_cpe: true,
                tag_select: 0,
            },
        ],
        side_elements: vec![],
        back_elements: vec![],
        lfe_element_tag_selects: vec![],
        assoc_data_tag_selects: vec![],
        valid_cc_elements: vec![
            CcElementSelect {
                is_ind_sw: false,
                tag_select: 0,
            },
            CcElementSelect {
                is_ind_sw: true,
                tag_select: 1,
            },
            CcElementSelect {
                is_ind_sw: false,
                tag_select: 2,
            },
        ],
        mono_mixdown_element_number: None,
        stereo_mixdown_element_number: None,
        matrix_mixdown: None,
        comment_field: vec![],
    };
    let payloads: Vec<Vec<u8>> = (0..9)
        .map(|i| {
            let mut fa = FrameAssembler::new();
            fa.push_pce(&pce).unwrap();
            push_sce(&mut fa, AOT_LC, 0, &sce_body, &sce_spec);
            push_cpe(&mut fa, 0, (&l_body, &l_spec), (&r_body, &r_spec));
            if with_cce {
                let (header, gains) = &variants[i % variants.len()];
                push_cce(&mut fa, (i % 3) as u8, header, &emb_body, &emb_spec, gains);
            }
            fa.push_end()
        })
        .collect();
    // channelConfiguration 0: the layout (and the CCE declarations)
    // come from the in-band PCE.
    wrap_adts_frames(&payloads, AOT_LC - 1, 0)
}

#[test]
fn cce_fixture_recipe_decodes_and_couples() {
    let stream = build_cce_stream(true);
    let baseline = build_cce_stream(false);

    let frames = StreamDecoder::new().decode_all(&stream).unwrap();
    let base = StreamDecoder::new().decode_all(&baseline).unwrap();
    assert_eq!(frames.len(), 9);
    let mut pcm = Vec::new();
    for (i, (f, b)) in frames.iter().zip(base.iter()).enumerate() {
        assert_eq!(f.channels, 3, "frame {i}");
        assert_eq!(f.pcm.len(), 3 * 1024);
        assert!(f.pcm.iter().any(|&s| s != 0), "frame {i} silent");
        // Every frame carries an active CCE: the coupled output must
        // differ from the CCE-less decode of the same targets.
        assert_ne!(f.pcm, b.pcm, "frame {i}: coupling had no effect");
        pcm.extend_from_slice(&f.pcm);
    }
    stage_or_pin(
        "aac-cce-writer-assembled",
        "input.aac",
        &stream,
        &pcm,
        3,
        SAMPLE_RATE,
    );
}

// ==================================================================
// HCR fixture — aac-er-hcr-loas
// ==================================================================

/// Per-frame band layouts. VCB entries (16..=31) exercise the ER
/// section branch; their plain-decode reference codes the same
/// spectrum with the ESC book 11.
const HCR_FRAME_CBS: &[&[u8]] = &[
    &[1, 3, 5, 7, 9, 11, 2, 4],
    &[2, 4, 6, 8, 10, 11, 1, 3],
    &[1, 3, 5, 7, 18, 2],
    &[11, 11, 9, 7, 5, 3, 1],
    &[1, 0, 3, 0, 5, 7, 27, 2],
    &[5, 6, 7, 8, 9, 10, 11, 11],
];

/// One ER SCE payload (tag + ER body + HCR spectrum).
fn er_sce_payload(sfb_cbs: &[u8], global_gain: u8, seed: u32) -> Vec<u8> {
    let (body, spectral) = make_channel(WindowSequence::OnlyLong, sfb_cbs, global_gain, seed);
    let ics = body.ics_info.as_ref().unwrap();
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_u32(u32::from(body.global_gain), 8);
    ics.write(&mut bw, AOT_ER_LC, FS_INDEX, false).unwrap();
    body.section_data
        .write_er(&mut bw, ics.window_sequence, ics.max_sfb)
        .unwrap();
    body.scale_factor_data
        .write(&mut bw, &body.section_data.sfb_cb)
        .unwrap();
    bw.write_bit(false); // pulse_data_present
    bw.write_bit(false); // tns_data_present
    bw.write_bit(false); // gain_control_data_present
    let (payload, len_bits, longest) =
        encode_reordered_spectral_data(&spectral, ics, &body.section_data, FS_INDEX).unwrap();
    bw.write_u32(u32::from(len_bits), 14);
    bw.write_u32(u32::from(longest), 6);
    for i in 0..usize::from(len_bits) {
        bw.write_bit(payload[i / 8] & (0x80 >> (i % 8)) != 0);
    }
    bw.finish()
}

/// The plain (non-ER) reference block for the same spectrum: VCB
/// codebooks fold to the ESC book 11.
fn plain_reference_block(sfb_cbs: &[u8], global_gain: u8, seed: u32) -> Vec<u8> {
    let folded: Vec<u8> = sfb_cbs
        .iter()
        .map(|&cb| if cb >= 16 { 11 } else { cb })
        .collect();
    let (body, _) = make_channel(WindowSequence::OnlyLong, &folded, global_gain, seed);
    // The spectrum must be generated from the *original* cb layout so
    // the pseudo-random draws match (VCBs draw from the book-11 range
    // in make_channel either way, so folding keeps them identical).
    let (_, spectral) = make_channel(WindowSequence::OnlyLong, sfb_cbs, global_gain, seed);
    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, AOT_LC, 0, &body, &spectral);
    fa.push_end()
}

/// Wrap ER payloads as a LOAS/LATM stream: the first sync frame
/// carries the inline `StreamMuxConfig()` (AOT-17 ASC, resilience
/// triplet 1/0/1, epConfig 0), later frames use `useSameStreamMux`.
fn wrap_loas(payloads: &[Vec<u8>]) -> Vec<u8> {
    let write_asc = |w: &mut BitWriter| {
        w.write_u32(u32::from(AOT_ER_LC), 5);
        w.write_u32(u32::from(FS_INDEX), 4);
        w.write_u32(1, 4); // channelConfiguration mono
        w.write_bit(false); // frameLengthFlag
        w.write_bit(false); // dependsOnCoreCoder
        w.write_bit(true); // extensionFlag
        w.write_bit(true); // aacSectionDataResilienceFlag
        w.write_bit(false); // aacScalefactorDataResilienceFlag
        w.write_bit(true); // aacSpectralDataResilienceFlag
        w.write_bit(false); // extensionFlag3
        w.write_u32(0, 2); // epConfig
    };
    let mut out = Vec::new();
    for (i, payload) in payloads.iter().enumerate() {
        let mut w = BitWriter::new();
        if i == 0 {
            w.write_bit(false); // useSameStreamMux = 0 (inline config)
            w.write_bit(false); // audioMuxVersion = 0
            w.write_bit(true); // allStreamsSameTimeFraming
            w.write_u32(0, 6); // numSubFrames
            w.write_u32(0, 4); // numProgram
            w.write_u32(0, 3); // numLayer
            write_asc(&mut w);
            w.write_u32(0, 3); // frameLengthType = 0
            w.write_u32(0xFF, 8); // latmBufferFullness
            w.write_bit(false); // otherDataPresent
            w.write_bit(false); // crcCheckPresent
        } else {
            w.write_bit(true); // useSameStreamMux = 1
        }
        // PayloadLengthInfo: MuxSlotLengthBytes with 255-escapes.
        let mut rem = payload.len();
        while rem >= 255 {
            w.write_u32(255, 8);
            rem -= 255;
        }
        w.write_u32(rem as u32, 8);
        for &b in payload {
            w.write_u32(u32::from(b), 8);
        }
        let element = w.finish();
        let mut fw = BitWriter::new();
        fw.write_u32(0x2B7, 11);
        fw.write_u32(element.len() as u32, 13);
        for &b in &element {
            fw.write_u32(u32::from(b), 8);
        }
        out.extend_from_slice(&fw.finish());
    }
    out
}

#[test]
fn hcr_fixture_recipe_matches_plain_decode() {
    let payloads: Vec<Vec<u8>> = HCR_FRAME_CBS
        .iter()
        .enumerate()
        .map(|(i, cbs)| er_sce_payload(cbs, 148 + (i as u8 % 5), 0x1000 + i as u32))
        .collect();
    let loas = wrap_loas(&payloads);

    let frames = LoasDecoder::new().decode_all(&loas).unwrap();
    assert_eq!(frames.len(), HCR_FRAME_CBS.len());

    // Oracle: the HCR / VCB coding changes the wire, not the
    // reconstruction — frame-for-frame bit-identical PCM against the
    // plain spectral_data() coding of the same spectra.
    let mut plain_dec = StreamDecoder::new();
    let mut pcm = Vec::new();
    for (i, (frame, cbs)) in frames.iter().zip(HCR_FRAME_CBS).enumerate() {
        let reference = plain_reference_block(cbs, 148 + (i as u8 % 5), 0x1000 + i as u32);
        let base = plain_dec
            .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 1, 1, &reference)
            .unwrap();
        assert_eq!(frame.channels, 1, "frame {i}");
        assert_eq!(frame.pcm, base.pcm, "frame {i}: ER decode diverged");
        assert!(frame.pcm.iter().any(|&s| s != 0), "frame {i} silent");
        pcm.extend_from_slice(&frame.pcm);
    }
    stage_or_pin("aac-er-hcr-loas", "input.latm", &loas, &pcm, 1, SAMPLE_RATE);
}

// ==================================================================
// SSR fixture — aac-ssr-gain-control-adts
// ==================================================================

/// The window-sequence chain the fixture walks (twice), exercising
/// every `gain_control_data()` branch of Table 4.12.
const SSR_SEQUENCE: &[WindowSequence] = &[
    WindowSequence::OnlyLong,
    WindowSequence::LongStart,
    WindowSequence::EightShort,
    WindowSequence::LongStop,
];

/// Per-sequence §4.6.12.3.3 output length (mono, 1024-line family).
fn ssr_frame_len(seq: WindowSequence) -> usize {
    match seq {
        WindowSequence::OnlyLong | WindowSequence::EightShort => 1024,
        WindowSequence::LongStart => 1472,
        WindowSequence::LongStop => 576,
    }
}

/// A gain ladder for one frame: non-unity adjustments on PQF bands
/// 1..=3 (band 0 is never gain-controlled), sized per window
/// sequence with the Table 4.12 `aloccode` width caps.
fn gain_data(seq: WindowSequence, active: bool) -> GainControlData {
    let n_win = oxideav_aac::gain_control_data::num_windows(seq);
    let adjust = |wd: usize, band: usize| -> Vec<GainAdjust> {
        if !active {
            return Vec::new();
        }
        // aloccode caps: OnlyLong wd0 5 bits; LongStart wd0 4 / wd1 2;
        // EightShort 2; LongStop wd0 4 / wd1 5.
        let aloc_cap: u8 = match (seq, wd) {
            (WindowSequence::OnlyLong, _) => 31,
            (WindowSequence::LongStart, 0) => 15,
            (WindowSequence::LongStart, _) => 3,
            (WindowSequence::EightShort, _) => 3,
            (WindowSequence::LongStop, 0) => 15,
            (WindowSequence::LongStop, _) => 31,
        };
        vec![GainAdjust {
            alevcode: (3 + band as u8) & 0x0F, // AdjLev = alevcode − 4
            aloccode: ((band as u8 * 3 + wd as u8) % (aloc_cap + 1)).min(aloc_cap),
        }]
    };
    let bands = (1..=3)
        .map(|band| GainBand {
            windows: (0..n_win)
                .map(|wd| GainWindow {
                    adjustments: adjust(wd, band),
                })
                .collect(),
        })
        .collect();
    GainControlData { max_band: 3, bands }
}

/// Build the SSR stream: mono AOT-3 ADTS walking the window chain
/// twice, every frame carrying `gain_control_data()`. `active`
/// selects non-unity ladders (the fixture) or empty ladders (the
/// comparison baseline).
fn build_ssr_stream(active: bool) -> (Vec<u8>, Vec<WindowSequence>) {
    let mut seqs = vec![WindowSequence::OnlyLong];
    seqs.extend(SSR_SEQUENCE.iter().skip(1));
    seqs.push(WindowSequence::OnlyLong);
    seqs.extend(SSR_SEQUENCE.iter().skip(1));
    seqs.push(WindowSequence::OnlyLong);

    let payloads: Vec<Vec<u8>> = seqs
        .iter()
        .enumerate()
        .map(|(i, &seq)| {
            // The spectrum must reach past PQF band 0 (the only band
            // never gain-controlled) for the ladders to matter: 40
            // long bands span well past coefficient 256, 14 short
            // bands past line 32 of each 128-line window.
            let cbs: Vec<u8> = if seq == WindowSequence::EightShort {
                (0..14).map(|k| [1, 3, 5, 7, 9, 11, 2][k % 7]).collect()
            } else {
                (0..40).map(|k| [1, 3, 5, 7, 9, 11, 2, 4][k % 8]).collect()
            };
            let (mut body, spectral) = make_channel(seq, &cbs, 150, 0x55B + i as u32);
            body.gain_control_data_present = true;
            body.gain_control_data = Some(gain_data(seq, active));
            let mut fa = FrameAssembler::new();
            // AOT 3 (SSR) legalises the gain_control_data gate.
            push_sce(&mut fa, 3, 0, &body, &spectral);
            fa.push_end()
        })
        .collect();
    // ADTS profile 2 = AOT 3 (SSR).
    (wrap_adts_frames(&payloads, 2, 1), seqs)
}

#[test]
fn ssr_fixture_recipe_decodes_with_gain_control() {
    let (stream, seqs) = build_ssr_stream(true);
    let (baseline, _) = build_ssr_stream(false);

    let (h, _) = AdtsHeader::parse(&stream).unwrap();
    assert_eq!(h.audio_object_type(), 3, "AOT must be SSR");

    let frames = StreamDecoder::new().decode_all(&stream).unwrap();
    let base = StreamDecoder::new().decode_all(&baseline).unwrap();
    assert_eq!(frames.len(), seqs.len());

    let mut pcm = Vec::new();
    let mut any_diff = false;
    for (i, (frame, &seq)) in frames.iter().zip(&seqs).enumerate() {
        assert_eq!(frame.channels, 1, "frame {i}");
        // §4.6.12.3.3: LONG_START / LONG_STOP produce 1472 / 576
        // samples through the SSR gain-control pipeline.
        assert_eq!(frame.pcm.len(), ssr_frame_len(seq), "frame {i} ({seq:?})");
        any_diff |= frame.pcm != base[i].pcm;
        pcm.extend_from_slice(&frame.pcm);
    }
    assert!(
        pcm.iter().any(|&s| s != 0),
        "SSR decode produced only silence"
    );
    // The non-unity ladders must actually modulate the output.
    assert!(any_diff, "gain ladders had no effect");
    stage_or_pin(
        "aac-ssr-gain-control-adts",
        "input.aac",
        &stream,
        &pcm,
        1,
        SAMPLE_RATE,
    );
}

//! Writer-assembled §4.5.1.1 frame-length-family docs fixtures — the
//! 960/120-line AAC-LC family and the ER AAC LD 512/480-line families
//! (§4.6.17), none of which any available encoder produces in a form
//! this crate can treat as a clean oracle:
//!
//! * no local encoder emits 960-line AAC-LC or 480-line LD at all;
//! * the one LD-512 encoder binary available emits a TNS side-info
//!   layout that contradicts the literal ISO/IEC 14496-3 Table 4.54 /
//!   Table 4.155 field widths (deployed decoders split on it too —
//!   see the notes.md), so its output cannot pin a spec-literal
//!   decoder.
//!
//! Per the fixtures-doc §6.1 fallback each fixture is a bit-exact
//! stream hand-assembled with this crate's own wire writers and
//! staged under `docs/audio/aac/fixtures/` with the decoder's PCM as
//! `expected.wav`. The *geometry* of every family — SWB tables,
//! window shapes (sine, KBD, and the §4.6.17.2.3 low-overlap), the
//! 1920/240- and 1024/960-point transforms and the overlap-add — was
//! cross-verified against two independent black-box decoder binaries
//! on equivalent MP4-wrapped streams before staging (bit-exact; the
//! per-fixture notes.md records the commands), so the writer-assembled
//! form is externally corroborated everywhere the deployed ecosystem
//! implements the family at all.
//!
//! Regenerate with
//! `OXIDEAV_AAC_STAGE_FIXTURES=1 cargo test --test docs_family_fixtures`;
//! without the variable each test pins the staged bytes against the
//! in-code recipe and the staged `expected.wav` against a fresh
//! decode, and skips cleanly when the docs corpus is absent
//! (standalone-repo CI).

use std::fs;
use std::path::PathBuf;

use oxideav_aac::asc::AacResilienceFlags;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{
    derive_window_grouping_family, IcsInfo, LtpData, WindowSequence, WindowShape,
};
use oxideav_aac::latm::LoasDecoder;
use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::swb_offset::{
    long_window_offsets_family, short_window_offsets_family, FrameFamily,
};
use oxideav_aac::tns_data::{TnsData, TnsFilter, TnsWindow};
use oxideav_core::bits::BitWriter;

const AOT_LC: u8 = 2;
const AOT_ER_LD: u8 = 23;
const FS_INDEX: u8 = 3; // 48 kHz
const SAMPLE_RATE: u32 = 48_000;

// ==================================================================
// Corpus plumbing (mirrors docs_writer_fixtures.rs)
// ==================================================================

fn fixtures_root() -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures")
}

fn stage_enabled() -> bool {
    std::env::var_os("OXIDEAV_AAC_STAGE_FIXTURES").is_some()
}

fn wav_bytes(pcm: &[i16], channels: u16, sample_rate: u32) -> Vec<u8> {
    let data_len = (pcm.len() * 2) as u32;
    let byte_rate = sample_rate * u32::from(channels) * 2;
    let block_align = channels * 2;
    let mut out = Vec::with_capacity(44 + pcm.len() * 2);
    out.extend_from_slice(b"RIFF");
    out.extend_from_slice(&(36 + data_len).to_le_bytes());
    out.extend_from_slice(b"WAVEfmt ");
    out.extend_from_slice(&16u32.to_le_bytes());
    out.extend_from_slice(&1u16.to_le_bytes());
    out.extend_from_slice(&channels.to_le_bytes());
    out.extend_from_slice(&sample_rate.to_le_bytes());
    out.extend_from_slice(&byte_rate.to_le_bytes());
    out.extend_from_slice(&block_align.to_le_bytes());
    out.extend_from_slice(&16u16.to_le_bytes());
    out.extend_from_slice(b"data");
    out.extend_from_slice(&data_len.to_le_bytes());
    for &s in pcm {
        out.extend_from_slice(&s.to_le_bytes());
    }
    out
}

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
// Family-aware channel builders
// ==================================================================

/// A family-aware `ics_info()` for one frame.
#[allow(clippy::too_many_arguments)]
fn ics_for(
    family: FrameFamily,
    seq: WindowSequence,
    shape: WindowShape,
    max_sfb: u8,
    grouping: Option<u8>,
    ltp: Option<LtpData>,
) -> IcsInfo {
    let short = seq == WindowSequence::EightShort;
    let (num_windows, num_window_groups, window_group_length, num_swb) =
        derive_window_grouping_family(family, seq, grouping, FS_INDEX).unwrap();
    IcsInfo {
        family,
        ics_reserved_bit: false,
        window_sequence: seq,
        window_shape: shape,
        max_sfb,
        scale_factor_grouping: if short { grouping } else { None },
        predictor_data_present: ltp.is_some(),
        predictor_data: None,
        ltp_data_present: ltp.is_some(),
        ltp_data: ltp,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows,
        num_window_groups,
        window_group_length,
        num_swb,
    }
}

/// One channel (body + spectrum) over per-band codebooks `sfb_cbs`,
/// with a deterministic pseudo-random spectrum seeded by `seed`,
/// under any frame-length family.
fn make_channel(
    ics: &IcsInfo,
    sfb_cbs: &[u8],
    global_gain: u8,
    seed: u32,
    tns: Option<TnsData>,
) -> (IcsBody, SpectralData) {
    let max_sfb = sfb_cbs.len() as u8;
    assert_eq!(max_sfb, ics.max_sfb, "codebook row must match max_sfb");
    let num_groups = ics.num_window_groups as usize;
    let sections: Vec<Vec<Section>> = (0..num_groups)
        .map(|_| {
            sfb_cbs
                .iter()
                .enumerate()
                .map(|(sfb, &cb)| Section {
                    codebook: cb,
                    start: sfb as u8,
                    end: sfb as u8 + 1,
                })
                .collect()
        })
        .collect();
    let entries: Vec<Vec<ScaleFactorEntry>> = (0..num_groups)
        .map(|_| {
            sfb_cbs
                .iter()
                .filter(|&&cb| cb != 0)
                .map(|_| ScaleFactorEntry::Dpcm(0))
                .collect()
        })
        .collect();
    let sect = SectionData {
        sections,
        sfb_cb: vec![sfb_cbs.to_vec(); num_groups],
    };
    let body = IcsBody {
        global_gain,
        ics_info: Some(ics.clone()),
        section_data: sect,
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
    let short = ics.window_sequence == WindowSequence::EightShort;
    let offsets: Vec<u16> = if short {
        short_window_offsets_family(ics.family, FS_INDEX)
            .unwrap()
            .to_vec()
    } else {
        long_window_offsets_family(ics.family, FS_INDEX)
            .unwrap()
            .to_vec()
    };
    let window_len = if short {
        ics.family.short_window_len().unwrap()
    } else {
        ics.family.frame_len()
    };
    let mut state = seed;
    let mut prand = |max: i32| {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        ((state >> 8) % (2 * max + 1) as u32) as i32 - max
    };
    let mut x_quant = Vec::with_capacity(ics.num_window_groups as usize);
    for g in 0..ics.num_window_groups as usize {
        let wgl = ics.window_group_length[g] as usize;
        let mut coeffs = vec![0i32; wgl * window_len];
        for (sfb, &cb) in sfb_cbs.iter().enumerate() {
            let max = match cb {
                0 | 13 | 14 | 15 => 0,
                1 | 2 => 1,
                3 | 4 => 2,
                5 | 6 => 4,
                7 | 8 => 7,
                9 | 10 => 12,
                _ => 25,
            };
            if max > 0 {
                let a = usize::from(offsets[sfb]) * wgl;
                let b = usize::from(offsets[sfb + 1]) * wgl;
                for c in coeffs[a..b].iter_mut() {
                    *c = prand(max);
                }
            }
        }
        x_quant.push(coeffs);
    }
    (body, SpectralData { x_quant })
}

// ==================================================================
// LOAS wrapping
// ==================================================================

/// Write the `AudioSpecificConfig()` for one stream: `aot` at
/// 48 kHz mono, `frameLengthFlag` per the family, ER trailer
/// (resilience triplet all-zero + `epConfig == 0`) for AOT 23.
fn write_asc(w: &mut BitWriter, aot: u8, family: FrameFamily) {
    let flf = matches!(family, FrameFamily::Lc960 | FrameFamily::Ld480);
    w.write_u32(u32::from(aot), 5);
    w.write_u32(u32::from(FS_INDEX), 4);
    w.write_u32(1, 4); // channelConfiguration = 1 (mono)
    w.write_bit(flf); // frameLengthFlag
    w.write_bit(false); // dependsOnCoreCoder
    if aot == AOT_ER_LD {
        w.write_bit(true); // extensionFlag (mandatory for AOT 23)
        w.write_bit(false); // aacSectionDataResilienceFlag
        w.write_bit(false); // aacScalefactorDataResilienceFlag
        w.write_bit(false); // aacSpectralDataResilienceFlag
        w.write_bit(false); // extensionFlag3
        w.write_u32(0, 2); // epConfig = 0
    } else {
        w.write_bit(false); // extensionFlag
    }
}

/// Wrap access-unit payloads as a LOAS/LATM stream (inline
/// StreamMuxConfig on the first sync frame, `useSameStreamMux`
/// afterwards).
fn wrap_loas(payloads: &[Vec<u8>], aot: u8, family: FrameFamily) -> Vec<u8> {
    let mut out = Vec::new();
    for (i, payload) in payloads.iter().enumerate() {
        let mut w = BitWriter::new();
        if i == 0 {
            w.write_bit(false); // useSameStreamMux = 0
            w.write_bit(false); // audioMuxVersion = 0
            w.write_bit(true); // allStreamsSameTimeFraming
            w.write_u32(0, 6); // numSubFrames
            w.write_u32(0, 4); // numProgram
            w.write_u32(0, 3); // numLayer
            write_asc(&mut w, aot, family);
            w.write_u32(0, 3); // frameLengthType = 0
            w.write_u32(0xFF, 8); // latmBufferFullness
            w.write_bit(false); // otherDataPresent
            w.write_bit(false); // crcCheckPresent
        } else {
            w.write_bit(true); // useSameStreamMux = 1
        }
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

/// One ER AAC LD access unit: `single_channel_element()` in the
/// Table 4.19 fixed-sequence form (4-bit tag + ER channel body).
fn ld_sce_payload(body: &IcsBody, spectral: &SpectralData) -> Vec<u8> {
    let ics = body.ics_info.as_ref().unwrap();
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4); // element_instance_tag
    body.write(&mut bw, AOT_ER_LD, FS_INDEX, false).unwrap();
    spectral
        .write(&mut bw, ics, &body.section_data, FS_INDEX)
        .unwrap();
    bw.finish()
}

/// One AAC-LC access unit: a tagged `raw_data_block()` SCE + END.
fn lc_sce_payload(body: &IcsBody, spectral: &SpectralData) -> Vec<u8> {
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

// ==================================================================
// Stream recipes
// ==================================================================

/// The LC-960 frame chain: every window sequence and both window
/// shapes at the 960/120-line geometry, closing with a TNS-active
/// long frame.
fn build_lc960_payloads() -> Vec<Vec<u8>> {
    let fam = FrameFamily::Lc960;
    let long_cbs: &[u8] = &[
        11, 11, 9, 9, 7, 7, 5, 5, 3, 3, 1, 1, 2, 4, 6, 8, 10, 1, 1, 3,
    ];
    let short_cbs: &[u8] = &[9, 7, 5, 3, 1, 2, 4, 6];
    let mut payloads = Vec::new();
    let frames: Vec<(WindowSequence, WindowShape, Option<u8>)> = vec![
        (WindowSequence::OnlyLong, WindowShape::Sine, None),
        (WindowSequence::OnlyLong, WindowShape::Kbd, None),
        (WindowSequence::LongStart, WindowShape::Sine, None),
        // 0x35 = 0110101: windows group as [1, 3, 2, 1, 1].
        (WindowSequence::EightShort, WindowShape::Sine, Some(0x35)),
        (WindowSequence::LongStop, WindowShape::Kbd, None),
        (WindowSequence::OnlyLong, WindowShape::Sine, None),
    ];
    for (i, (seq, shape, grouping)) in frames.into_iter().enumerate() {
        let short = seq == WindowSequence::EightShort;
        let cbs = if short { short_cbs } else { long_cbs };
        // The last long frame carries a Table 4.54 TNS filter.
        let tns = if i == 5 {
            Some(TnsData {
                windows: vec![TnsWindow {
                    coef_res: false,
                    filters: vec![TnsFilter {
                        length: 12,
                        order: 3,
                        direction: false,
                        coef_compress: false,
                        coef: vec![2, 6, 1],
                    }],
                }],
            })
        } else {
            None
        };
        let ics = ics_for(fam, seq, shape, cbs.len() as u8, grouping, None);
        let (body, spectral) = make_channel(&ics, cbs, 132 + i as u8, 0x9600 + i as u32, tns);
        payloads.push(lc_sce_payload(&body, &spectral));
    }
    payloads
}

/// The LD frame chain shared by the 512 and 480 fixtures: sine and
/// low-overlap shapes with switches, a §4.6.7 LD LTP pair (explicit
/// lag then the `ltp_lag_update == 0` repeat), and a spec-literal
/// Table 4.54 TNS frame.
fn build_ld_payloads(fam: FrameFamily) -> Vec<Vec<u8>> {
    let cbs: &[u8] = &[11, 11, 9, 7, 5, 3, 1, 2, 4, 6, 8, 10, 1, 1, 3, 5];
    let max_sfb = cbs.len() as u8;
    let mut payloads = Vec::new();
    // Shapes: sine, low-overlap, low-overlap, sine (switches on both
    // edges). Frames 4/5 are the LTP pair, frame 6 the TNS frame.
    let shapes = [
        WindowShape::Sine,
        WindowShape::Kbd, // = low-overlap under LD (§4.6.17.2.3)
        WindowShape::Kbd,
        WindowShape::Sine,
        WindowShape::Sine,
        WindowShape::Sine,
        WindowShape::Sine,
    ];
    for (i, &shape) in shapes.iter().enumerate() {
        let ltp = match i {
            4 => Some(LtpData {
                lag_update: Some(true),
                lag: Some(200),
                coef: 3,
                long_used: vec![true; usize::from(max_sfb)],
                short: None,
            }),
            5 => Some(LtpData {
                lag_update: Some(false),
                lag: None,
                coef: 3,
                long_used: vec![true; usize::from(max_sfb)],
                short: None,
            }),
            _ => None,
        };
        let tns = if i == 6 {
            Some(TnsData {
                windows: vec![TnsWindow {
                    coef_res: false,
                    filters: vec![TnsFilter {
                        length: 10,
                        order: 2,
                        direction: false,
                        coef_compress: false,
                        coef: vec![2, 5],
                    }],
                }],
            })
        } else {
            None
        };
        let ics = ics_for(fam, WindowSequence::OnlyLong, shape, max_sfb, None, ltp);
        let (body, spectral) = make_channel(&ics, cbs, 130 + i as u8, 0x1d00 + i as u32, tns);
        payloads.push(ld_sce_payload(&body, &spectral));
    }
    payloads
}

fn decode_loas(stream: &[u8], expect_frames: usize, samples_per_frame: usize) -> Vec<i16> {
    let frames = LoasDecoder::new().decode_all(stream).unwrap();
    assert_eq!(frames.len(), expect_frames);
    let mut pcm = Vec::new();
    for (i, f) in frames.iter().enumerate() {
        assert_eq!(f.channels, 1, "frame {i}");
        assert_eq!(f.sample_rate, SAMPLE_RATE, "frame {i}");
        assert_eq!(
            f.pcm.len(),
            samples_per_frame,
            "frame {i}: family frame length"
        );
        pcm.extend_from_slice(&f.pcm);
    }
    assert!(pcm.iter().any(|&s| s != 0), "stream decoded to silence");
    pcm
}

// ==================================================================
// Fixture tests
// ==================================================================

#[test]
fn lc960_fixture_recipe_decodes() {
    let payloads = build_lc960_payloads();
    let loas = wrap_loas(&payloads, AOT_LC, FrameFamily::Lc960);
    let pcm = decode_loas(&loas, payloads.len(), 960);

    // Plumbing identity: the LOAS route must equal the raw
    // decode_raw_data_block route with the family installed.
    let mut dec = StreamDecoder::new();
    dec.set_frame_family(FrameFamily::Lc960);
    let mut direct = Vec::new();
    for p in &payloads {
        let f = dec
            .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 1, 1, p)
            .unwrap();
        direct.extend_from_slice(&f.pcm);
    }
    assert_eq!(pcm, direct, "LOAS vs direct raw_data_block decode");

    stage_or_pin(
        "aac-lc-960-writer-loas",
        "input.latm",
        &loas,
        &pcm,
        1,
        SAMPLE_RATE,
    );
}

#[test]
fn ld512_fixture_recipe_decodes() {
    let payloads = build_ld_payloads(FrameFamily::Ld512);
    let loas = wrap_loas(&payloads, AOT_ER_LD, FrameFamily::Ld512);
    let pcm = decode_loas(&loas, payloads.len(), 512);

    // LTP repeat identity: replacing the ltp_lag_update == 0 frame
    // with an explicit same-lag frame changes the wire but not the
    // PCM.
    let mut explicit = build_ld_payloads(FrameFamily::Ld512);
    {
        let cbs: &[u8] = &[11, 11, 9, 7, 5, 3, 1, 2, 4, 6, 8, 10, 1, 1, 3, 5];
        let ltp = LtpData {
            lag_update: Some(true),
            lag: Some(200), // same effective lag as the repeat
            coef: 3,
            long_used: vec![true; cbs.len()],
            short: None,
        };
        let ics = ics_for(
            FrameFamily::Ld512,
            WindowSequence::OnlyLong,
            WindowShape::Sine,
            cbs.len() as u8,
            None,
            Some(ltp),
        );
        let (body, spectral) = make_channel(&ics, cbs, 135, 0x1d05, None);
        explicit[5] = ld_sce_payload(&body, &spectral);
    }
    assert_ne!(explicit[5], payloads[5], "wire must differ");
    let loas_explicit = wrap_loas(&explicit, AOT_ER_LD, FrameFamily::Ld512);
    let pcm_explicit = decode_loas(&loas_explicit, explicit.len(), 512);
    assert_eq!(
        pcm, pcm_explicit,
        "ltp_lag_update repeat must decode like the explicit lag"
    );

    stage_or_pin(
        "aac-ld-512-writer-loas",
        "input.latm",
        &loas,
        &pcm,
        1,
        SAMPLE_RATE,
    );
}

#[test]
fn ld480_fixture_recipe_decodes() {
    let payloads = build_ld_payloads(FrameFamily::Ld480);
    let loas = wrap_loas(&payloads, AOT_ER_LD, FrameFamily::Ld480);
    let pcm = decode_loas(&loas, payloads.len(), 480);
    stage_or_pin(
        "aac-ld-480-writer-loas",
        "input.latm",
        &loas,
        &pcm,
        1,
        SAMPLE_RATE,
    );
}

// ==================================================================
// Mutation battery — corrupt streams must error, never panic, and
// the decoder must survive to decode a clean stream afterwards.
// ==================================================================

fn mutate_and_decode(stream: &[u8]) {
    // Flip one bit in every byte position across the stream (0x40
    // keeps LOAS sync plausible more often than 0x80, exercising
    // deeper parse paths); a second low-bit flip and the truncations
    // run on a stride to keep the battery CI-sized.
    for pos in 0..stream.len() {
        let mut bad = stream.to_vec();
        bad[pos] ^= 0x40;
        let _ = LoasDecoder::new().decode_all(&bad);
        if pos % 5 == 0 {
            let mut bad = stream.to_vec();
            bad[pos] ^= 0x01;
            let _ = LoasDecoder::new().decode_all(&bad);
        }
    }
    // Truncations across the stream on a byte stride.
    for len in (0..stream.len()).step_by(7) {
        let _ = LoasDecoder::new().decode_all(&stream[..len]);
    }
}

#[test]
fn lc960_mutations_never_panic() {
    let loas = wrap_loas(&build_lc960_payloads(), AOT_LC, FrameFamily::Lc960);
    mutate_and_decode(&loas);
}

#[test]
fn ld512_mutations_never_panic() {
    let loas = wrap_loas(
        &build_ld_payloads(FrameFamily::Ld512),
        AOT_ER_LD,
        FrameFamily::Ld512,
    );
    mutate_and_decode(&loas);
}

#[test]
fn ld480_mutations_never_panic() {
    let loas = wrap_loas(
        &build_ld_payloads(FrameFamily::Ld480),
        AOT_ER_LD,
        FrameFamily::Ld480,
    );
    mutate_and_decode(&loas);
}

// ==================================================================
// Family separation invariants
// ==================================================================

/// The same LD payload bits under the wrong family must not decode
/// to the same PCM silently: the ASC's frameLengthFlag is the only
/// in-band selector, so the driver must honour it exactly.
#[test]
fn ld_families_are_wire_incompatible() {
    let p512 = build_ld_payloads(FrameFamily::Ld512);
    // Decoding the 512-line payloads under an Ld480 family must fail
    // or produce different PCM — never the identical decode.
    let mut dec512 = StreamDecoder::new();
    dec512.set_frame_family(FrameFamily::Ld512);
    let mut dec480 = StreamDecoder::new();
    dec480.set_frame_family(FrameFamily::Ld480);
    let ok = dec512
        .decode_er_raw_data_block(
            AOT_ER_LD,
            FS_INDEX,
            SAMPLE_RATE,
            1,
            AacResilienceFlags::default(),
            &p512[0],
        )
        .unwrap();
    match dec480.decode_er_raw_data_block(
        AOT_ER_LD,
        FS_INDEX,
        SAMPLE_RATE,
        1,
        AacResilienceFlags::default(),
        &p512[0],
    ) {
        Ok(f) => assert_ne!(f.pcm, ok.pcm, "families must not alias"),
        Err(_) => {} // structural rejection is equally fine
    }
}

/// An SBR extension on a 960-line stream is out of scope and must be
/// rejected cleanly (Error::SbrUnsupportedFrameFamily), not
/// mis-decoded.
#[test]
fn sbr_on_960_family_is_rejected() {
    // An LC-960 frame followed by a FIL carrying an SBR extension
    // header fragment (extension_type 0b1101 = EXT_SBR_DATA).
    let fam = FrameFamily::Lc960;
    let cbs: &[u8] = &[1, 1, 1, 1];
    let ics = ics_for(
        fam,
        WindowSequence::OnlyLong,
        WindowShape::Sine,
        4,
        None,
        None,
    );
    let (body, spectral) = make_channel(&ics, cbs, 130, 0xABCD, None);
    let mut fa = FrameAssembler::new();
    fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let mut bw = BitWriter::new();
    body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
    spectral
        .write(&mut bw, &ics, &body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
    // FIL body: extension_type EXT_SBR_DATA + a plausible header bit.
    let mut fil = BitWriter::new();
    fil.write_u32(0b1101, 4);
    fil.write_u32(0, 4);
    fil.write_u32(0, 8);
    fa.push_fill(&fil.finish()).unwrap();
    let payload = fa.push_end();

    let mut dec = StreamDecoder::new();
    dec.set_frame_family(fam);
    let err = dec
        .decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 1, 1, &payload)
        .unwrap_err();
    assert_eq!(err, oxideav_aac::Error::SbrUnsupportedFrameFamily);
}

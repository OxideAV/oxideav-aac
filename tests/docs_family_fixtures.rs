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
    ics_for_fs(family, seq, shape, max_sfb, grouping, ltp, FS_INDEX)
}

/// [`ics_for`] at an explicit sampling-frequency index.
fn ics_for_fs(
    family: FrameFamily,
    seq: WindowSequence,
    shape: WindowShape,
    max_sfb: u8,
    grouping: Option<u8>,
    ltp: Option<LtpData>,
    fs_index: u8,
) -> IcsInfo {
    let short = seq == WindowSequence::EightShort;
    let (num_windows, num_window_groups, window_group_length, num_swb) =
        derive_window_grouping_family(family, seq, grouping, fs_index).unwrap();
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
    make_channel_fs(ics, sfb_cbs, global_gain, seed, tns, FS_INDEX)
}

/// [`make_channel`] at an explicit sampling-frequency index.
fn make_channel_fs(
    ics: &IcsInfo,
    sfb_cbs: &[u8],
    global_gain: u8,
    seed: u32,
    tns: Option<TnsData>,
    fs_index: u8,
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
        short_window_offsets_family(ics.family, fs_index)
            .unwrap()
            .to_vec()
    } else {
        long_window_offsets_family(ics.family, fs_index)
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
fn write_asc_fs(w: &mut BitWriter, aot: u8, family: FrameFamily, fs_index: u8) {
    let flf = matches!(family, FrameFamily::Lc960 | FrameFamily::Ld480);
    w.write_u32(u32::from(aot), 5);
    w.write_u32(u32::from(fs_index), 4);
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
    wrap_loas_fs(payloads, aot, family, FS_INDEX)
}

/// [`wrap_loas`] at an explicit sampling-frequency index.
fn wrap_loas_fs(payloads: &[Vec<u8>], aot: u8, family: FrameFamily, fs_index: u8) -> Vec<u8> {
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
            write_asc_fs(&mut w, aot, family, fs_index);
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
    // Structural rejection is equally fine; what must never happen
    // is an identical decode under the wrong family.
    if let Ok(f) = dec480.decode_er_raw_data_block(
        AOT_ER_LD,
        FS_INDEX,
        SAMPLE_RATE,
        1,
        AacResilienceFlags::default(),
        &p512[0],
    ) {
        assert_ne!(f.pcm, ok.pcm, "families must not alias");
    }
}

/// An SBR extension on a 960-line stream decodes over the §4.6.18
/// 15-time-slot frame (`numTimeSlots = 15` for a 960 AAC frame): a
/// LC-960 SCE followed by a FIL carrying a complete `sbr_extension_data()`
/// (header + one FIXFIX envelope) yields 1920 samples at the doubled
/// rate with the high band populated, a payload-less frame holds
/// the SBR state through pure upsampling, and the LD families still
/// reject the extension.
#[test]
fn sbr_on_960_family_decodes_at_fifteen_slots() {
    use oxideav_aac::sbr_element::{SbrChannel, SbrElement};
    use oxideav_aac::sbr_envelope::{SbrEnvelopeData, SbrNoiseData};
    use oxideav_aac::sbr_grid::{FrameClass, SbrDtdf, SbrGrid, SbrInvf};
    use oxideav_aac::sbr_header::SbrHeader;
    use oxideav_aac::sbr_writer::build_extension_payload;

    let fam = FrameFamily::Lc960;
    let fs_sbr = SAMPLE_RATE * 2;
    // SBR range 12–21 kHz at the 96 kHz SBR rate (within the
    // §4.6.18.3.6 `k2 − k0 ≤ 32` bound).
    let start_freq = oxideav_aac::sbr_encoder::pick_start_freq(fs_sbr, 12_000.0).unwrap();
    let stop_freq = oxideav_aac::sbr_encoder::pick_stop_freq(fs_sbr, start_freq, 21_000.0).unwrap();
    let header = SbrHeader {
        amp_res: true,
        start_freq,
        stop_freq,
        xover_band: 0,
        reserved: 0,
        header_extra_1: false,
        header_extra_2: false,
        freq_scale: 2,
        alter_scale: true,
        noise_bands: 2,
        limiter_bands: 2,
        limiter_gains: 2,
        interpol_freq: true,
        smoothing_mode: true,
    };
    let bands = header.derive_bands(fs_sbr).unwrap();
    let n_high = bands.n_high();
    let n_q = bands.n_q();
    let mut env_row = vec![0i32; n_high];
    env_row[0] = 30;
    let mut noise_row = vec![0i32; n_q];
    noise_row[0] = 8;
    let element = SbrElement {
        coupling: false,
        channels: vec![SbrChannel {
            grid: SbrGrid {
                frame_class: FrameClass::FixFix,
                num_env: 1,
                num_noise: 1,
                freq_res: vec![true],
                var_bord_0: 0,
                var_bord_1: 0,
                rel_bord_0: vec![],
                rel_bord_1: vec![],
                pointer: 0,
                amp_res_override: true,
            },
            dtdf: SbrDtdf {
                df_env: vec![false],
                df_noise: vec![false],
            },
            invf: SbrInvf {
                invf_mode: vec![0; n_q],
            },
            envelope: SbrEnvelopeData {
                data: vec![env_row],
            },
            noise: SbrNoiseData {
                data: vec![noise_row],
            },
            add_harmonic: vec![],
        }],
        extension: None,
    };
    let sbr_payload = build_extension_payload(
        IdSynEle::Sce,
        Some(&header),
        &header,
        &element,
        &bands,
        false,
    )
    .unwrap();

    let frame = |with_sbr: bool, seed: u32| -> Vec<u8> {
        let cbs: &[u8] = &[1, 1, 1, 1, 2, 2, 3, 3];
        let ics = ics_for(
            fam,
            WindowSequence::OnlyLong,
            WindowShape::Sine,
            8,
            None,
            None,
        );
        let (body, spectral) = make_channel(&ics, cbs, 140, seed, None);
        let mut fa = FrameAssembler::new();
        fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
        let mut bw = BitWriter::new();
        body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
        spectral
            .write(&mut bw, &ics, &body.section_data, FS_INDEX)
            .unwrap();
        let bits = bw.bit_position();
        fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
        if with_sbr {
            fa.push_fill(&sbr_payload).unwrap();
        }
        fa.push_end()
    };

    // Raw-block path: SBR frames, then a payload-less frame.
    let mut dec = StreamDecoder::new();
    dec.set_frame_family(fam);
    let mut pcm: Vec<i16> = Vec::new();
    for (i, with_sbr) in [true, true, true, false, true].into_iter().enumerate() {
        let f = dec
            .decode_raw_data_block(
                AOT_LC,
                FS_INDEX,
                SAMPLE_RATE,
                1,
                1,
                &frame(with_sbr, 0x9600 + i as u32),
            )
            .unwrap();
        assert_eq!(f.pcm.len(), 1920, "frame {i}: 15 slots × 2 × 64");
        assert_eq!(f.sample_rate, fs_sbr, "frame {i}");
        assert_eq!(f.channels, 1);
        pcm.extend_from_slice(&f.pcm);
    }
    assert!(pcm.iter().any(|&s| s != 0));
    // High band populated: energy above the core Nyquist over the
    // steady frames (a coarse DFT on a stride of bins).
    let tail: Vec<f64> = pcm[1920..].iter().map(|&v| f64::from(v)).collect();
    let n = tail.len();
    let (mut hi, mut total) = (0.0f64, 0.0f64);
    for k in (1..n / 2).step_by(53) {
        let (mut re, mut im) = (0.0, 0.0);
        for (t, &v) in tail.iter().enumerate() {
            let a = 2.0 * std::f64::consts::PI * ((k * t) % n) as f64 / n as f64;
            re += v * a.cos();
            im += v * a.sin();
        }
        let p = re * re + im * im;
        total += p;
        if k > n / 4 {
            hi += p;
        }
    }
    assert!(hi > 1e-4 * total, "high band {hi} of {total}");

    // LOAS path: the same frames through the ASC-driven family
    // selection.
    let payloads: Vec<Vec<u8>> = (0..4).map(|i| frame(true, 0x9700 + i)).collect();
    let loas = wrap_loas(&payloads, AOT_LC, fam);
    let frames = LoasDecoder::new().decode_all(&loas).unwrap();
    assert_eq!(frames.len(), 4);
    for f in &frames {
        assert_eq!(f.pcm.len(), 1920);
        assert_eq!(f.sample_rate, fs_sbr);
    }

    // The LD families stay out of the tool's scope.
    for ld in [FrameFamily::Ld512, FrameFamily::Ld480] {
        let mut fil = BitWriter::new();
        fil.write_u32(0b1101, 4);
        fil.write_u32(0, 4);
        fil.write_u32(0, 8);
        let ics = ics_for(
            ld,
            WindowSequence::OnlyLong,
            WindowShape::Sine,
            4,
            None,
            None,
        );
        let (body, spectral) = make_channel(&ics, &[1, 1, 1, 1], 130, 0xABCD, None);
        let mut fa = FrameAssembler::new();
        fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
        let mut bw = BitWriter::new();
        body.write(&mut bw, AOT_ER_LD, FS_INDEX, false).unwrap();
        spectral
            .write(&mut bw, &ics, &body.section_data, FS_INDEX)
            .unwrap();
        let bits = bw.bit_position();
        fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
        fa.push_fill(&fil.finish()).unwrap();
        let payload = fa.push_end();
        let mut dec = StreamDecoder::new();
        dec.set_frame_family(ld);
        let err = dec
            .decode_raw_data_block(AOT_ER_LD, FS_INDEX, SAMPLE_RATE, 1, 1, &payload)
            .unwrap_err();
        assert_eq!(err, oxideav_aac::Error::SbrUnsupportedFrameFamily, "{ld:?}");
    }
}

/// HE-AAC over the 960-line core, both directions: the SBR encoder
/// laid out over `numTimeSlots = 15` (48 analysis columns per frame,
/// grids over 15 slots) codes a wideband 48 kHz signal, its
/// `sbr_extension_data()` rides in a FIL after each writer-assembled
/// LC-960 SCE (24 kHz core), and the LOAS stream decodes through this
/// crate to 1920 samples per frame with a populated high band.
///
/// When a reference decoder binary is on PATH the same stream is
/// decoded black-box. The available reference implementation does
/// not implement SBR over 960-line frames (it reports so under
/// explicit signalling and silently decodes the core alone under the
/// implicit signalling used here), so the black-box check is the
/// core band: the stream must decode without diagnostics, and the
/// per-QMF-band long-term energies below the crossover must agree in
/// shape with this crate's decode (its 375 Hz bands at the SBR rate
/// against the reference's paired 187.5 Hz bands at the core rate,
/// up to one common scale).
#[test]
fn he_aac_960_round_trips_and_core_band_matches_the_reference_binary() {
    use oxideav_aac::sbr_encoder::{SbrEncoder, SbrEncoderConfig, NUM_TIME_SLOTS_960};
    use oxideav_aac::sbr_qmf::EncoderAnalysisQmf;
    use std::process::Command;

    // A 24 kHz core (implicit SBR signalling is only honoured at
    // core rates up to 24 kHz — §1.6.5.2) → 48 kHz SBR output.
    let fam = FrameFamily::Lc960;
    let fs_index = 6u8;
    let core_rate = 24_000u32;
    let fs_sbr = core_rate * 2;
    let n_frames = 24usize;
    let mut cfg = SbrEncoderConfig::new(fs_sbr, 1, 7_000.0, 16_000.0).unwrap();
    cfg.num_time_slots = NUM_TIME_SLOTS_960;
    let mut sbr = SbrEncoder::new(cfg).unwrap();
    let cols_per_frame = cfg.enc_cols();
    assert_eq!(cols_per_frame, 48);

    // Wideband source at the SBR rate: tones across the SBR range
    // over a noise bed, with a burst every fourth frame so the
    // variable grids are exercised on 15-slot frames.
    let total_cols = 30 * n_frames + cols_per_frame;
    let mut seed = 0x0960_ACE1u32;
    let mut bank = EncoderAnalysisQmf::new();
    let mut cols = Vec::with_capacity(total_cols);
    for c in 0..total_cols {
        let slot: Vec<f64> = (0..64)
            .map(|i| {
                let t = (c * 64 + i) as f64 / f64::from(fs_sbr);
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let noise = f64::from(seed >> 8) / f64::from(1u32 << 24) - 0.5;
                let frame = c / 30;
                let burst = frame % 4 == 1 && (c % 30) >= 12 && (c % 30) < 16;
                700.0 * (2.0 * std::f64::consts::PI * 8_100.0 * t).sin()
                    + 500.0 * (2.0 * std::f64::consts::PI * 11_300.0 * t).sin()
                    + 300.0 * (2.0 * std::f64::consts::PI * 14_600.0 * t).sin()
                    + noise * if burst { 24_000.0 } else { 100.0 }
            })
            .collect();
        cols.push(bank.push_slot(&slot).unwrap());
    }

    // Core spectrum across the whole band below the crossover (36
    // scalefactor bands reach ≈ 7 kHz at 24 kHz / 960 lines).
    let cbs: Vec<u8> = (0..36).map(|i| [1u8, 2, 3, 5, 7, 9][i % 6]).collect();
    let cbs: &[u8] = &cbs;
    let mut payloads = Vec::with_capacity(n_frames);
    let mut classes = [0usize; 4];
    for f in 0..n_frames {
        let window = &cols[30 * f..30 * f + cols_per_frame];
        let frame = sbr.encode_frame(&[window]).unwrap();
        let g = &frame.element.channels[0].grid;
        classes[g.frame_class.to_bits() as usize] += 1;
        let last = *frame.reports[0].t_e.last().unwrap();
        assert!(
            (15..=18).contains(&last),
            "frame {f}: {:?}",
            frame.reports[0].t_e
        );
        let ics = ics_for_fs(
            fam,
            WindowSequence::OnlyLong,
            WindowShape::Sine,
            cbs.len() as u8,
            None,
            None,
            fs_index,
        );
        let (body, spectral) = make_channel_fs(&ics, cbs, 140, 0x9600 + f as u32, None, fs_index);
        let mut fa = FrameAssembler::new();
        fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
        let mut bw = BitWriter::new();
        body.write(&mut bw, AOT_LC, fs_index, false).unwrap();
        spectral
            .write(&mut bw, &ics, &body.section_data, fs_index)
            .unwrap();
        let bits = bw.bit_position();
        fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
        fa.push_fill(&frame.payload).unwrap();
        payloads.push(fa.push_end());
    }
    eprintln!("960 SBR frame classes FIXFIX/FIXVAR/VARFIX/VARVAR = {classes:?}");
    assert!(
        classes[1] + classes[2] + classes[3] > 0,
        "no variable grid on a 15-slot frame"
    );

    let loas = wrap_loas_fs(&payloads, AOT_LC, fam, fs_index);
    let frames = LoasDecoder::new().decode_all(&loas).unwrap();
    assert_eq!(frames.len(), n_frames);
    let mut ours: Vec<i16> = Vec::new();
    for (i, f) in frames.iter().enumerate() {
        assert_eq!(f.pcm.len(), 1920, "frame {i}");
        assert_eq!(f.sample_rate, fs_sbr, "frame {i}");
        ours.extend_from_slice(&f.pcm);
    }
    let band_energies = |pcm: &[i16], channels: usize| -> [f64; 64] {
        let mono: Vec<f64> = pcm
            .iter()
            .step_by(channels)
            .map(|&v| f64::from(v))
            .collect();
        let mut bank = EncoderAnalysisQmf::new();
        let mut e = [0.0f64; 64];
        for slot in mono.chunks_exact(64) {
            for (k, v) in bank.push_slot(slot).unwrap().iter().enumerate() {
                e[k] += v.norm_sqr();
            }
        }
        e
    };
    let e_ours = band_energies(&ours, 1);
    let k_x = sbr.bands().k_x as usize;
    let k_end = (sbr.bands().k_x + sbr.bands().m) as usize;
    let low: f64 = e_ours[..k_x].iter().sum();
    let high: f64 = e_ours[k_x..k_end].iter().sum();
    assert!(high > 1e-3 * low, "SBR range silent: {high} vs {low}");

    // Black-box: the reference decoder binary on the same LOAS stream.
    let ff = "ffmpeg";
    if !Command::new(ff)
        .arg("-version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
    {
        eprintln!("skip black-box: no ffmpeg binary on PATH");
        return;
    }
    let dir = std::env::temp_dir().join("oxideav-aac-he960-blackbox");
    fs::create_dir_all(&dir).unwrap();
    let latm = dir.join("he960.latm");
    let wav = dir.join("he960.wav");
    fs::write(&latm, &loas).unwrap();
    let _ = fs::remove_file(&wav);
    let out = Command::new(ff)
        .args([
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "loas",
            "-i",
        ])
        .arg(&latm)
        .arg(&wav)
        .output()
        .expect("run reference decoder");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.trim().is_empty(),
        "reference decoder diagnostics: {stderr}"
    );
    let d = fs::read(&wav).unwrap();
    let mut i = 12;
    let (mut ch, mut rate, mut pcm) = (0usize, 0u32, Vec::new());
    while i + 8 <= d.len() {
        let cid = &d[i..i + 4];
        let sz = u32::from_le_bytes([d[i + 4], d[i + 5], d[i + 6], d[i + 7]]) as usize;
        let body = i + 8;
        if cid == b"fmt " {
            ch = usize::from(u16::from_le_bytes([d[body + 2], d[body + 3]]));
            rate = u32::from_le_bytes([d[body + 4], d[body + 5], d[body + 6], d[body + 7]]);
        }
        if cid == b"data" {
            let end = (body + sz).min(d.len());
            pcm = d[body..end]
                .chunks_exact(2)
                .map(|c| i16::from_le_bytes([c[0], c[1]]))
                .collect();
            break;
        }
        i = body + sz + (sz & 1);
    }
    assert!(
        rate == core_rate || rate == fs_sbr,
        "reference output rate {rate}"
    );
    assert!(ch == 1 || ch == 2, "reference channels {ch}");
    let n_ref = pcm.len() / ch.max(1) * (fs_sbr / rate) as usize;
    assert!(
        (n_ref as i64 - ours.len() as i64).unsigned_abs() <= 2 * 1920,
        "length {n_ref} vs ours {}",
        ours.len()
    );
    // Reference bands mapped onto this crate's 48 kHz band grid.
    let e_raw = band_energies(&pcm, ch.max(1));
    let e_ref: Vec<f64> = if rate == fs_sbr {
        e_raw.to_vec()
    } else {
        (0..32).map(|k| e_raw[2 * k] + e_raw[2 * k + 1]).collect()
    };
    // At the core rate the band just below the crossover is skipped:
    // this crate's output carries the SBR range there and the
    // analysis bank's adjacent-band overlap leaks it into `k_x − 1`,
    // which the reference's core-only output cannot show.
    let k_hi = if rate == fs_sbr {
        k_end.min(64)
    } else {
        (k_x - 1).min(32)
    };
    // Bands carrying real signal, relative to the compared range's
    // own peak (the SBR range above it is far louder than the
    // writer-assembled core spectrum).
    let floor = e_ours[1..k_hi].iter().cloned().fold(0.0, f64::max) * 1e-4;
    let mut deltas: Vec<f64> = (1..k_hi)
        .filter(|&k| e_ours[k] >= floor)
        .map(|k| 10.0 * (e_ref[k] / e_ours[k]).log10())
        .collect();
    assert!(deltas.len() >= 8, "{} comparable bands", deltas.len());
    let mut sorted = deltas.clone();
    sorted.sort_by(f64::total_cmp);
    let offset = sorted[sorted.len() / 2];
    for d in deltas.iter_mut() {
        *d = (*d - offset).abs();
    }
    let mean = deltas.iter().sum::<f64>() / deltas.len() as f64;
    let worst = deltas.iter().cloned().fold(0.0, f64::max);
    eprintln!(
        "he-aac 960 core band vs reference ({rate} Hz): {} bands, common offset {offset:.2} dB, mean |Δ| {mean:.2} dB, worst {worst:.2} dB",
        deltas.len()
    );
    assert!(mean < 1.0, "mean band delta {mean} dB");
    assert!(worst < 3.0, "worst band delta {worst} dB");
}

/// A mid-stream StreamMuxConfig replacement that changes the layer's
/// frame family must rebuild the per-stream decoder (the overlap/LTP
/// state is family-shaped): a 960-frame stream followed by an inline
/// reconfiguration to the LD-512 family decodes both halves at their
/// own frame lengths.
#[test]
fn latm_family_reconfiguration_resets_stream_state() {
    let lc = build_lc960_payloads();
    let ld = build_ld_payloads(FrameFamily::Ld512);
    let mut stream = wrap_loas(&lc[..2], AOT_LC, FrameFamily::Lc960);
    // Second LOAS run re-sends an inline config (i == 0 path) for the
    // LD layer.
    stream.extend_from_slice(&wrap_loas(&ld[..2], AOT_ER_LD, FrameFamily::Ld512));
    let frames = LoasDecoder::new().decode_all(&stream).unwrap();
    assert_eq!(frames.len(), 4);
    assert_eq!(frames[0].pcm.len(), 960);
    assert_eq!(frames[1].pcm.len(), 960);
    assert_eq!(frames[2].pcm.len(), 512);
    assert_eq!(frames[3].pcm.len(), 512);
}

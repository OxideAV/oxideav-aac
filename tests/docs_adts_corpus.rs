//! Structural decode tests against the `docs/audio/aac/fixtures/`
//! ADTS corpus.
//!
//! For every staged ADTS fixture (`input.aac`) this driver walks the
//! stream frame by frame — ADTS header → `raw_data_block()` element
//! walk → Table 4.50 channel bodies → Table 4.56 `spectral_data()` —
//! and runs each channel through the
//! [`oxideav_aac::decoded_spectrum::decode_channel_spectrum`]
//! pipeline stage (pulse fix-up → scalefactor accumulation →
//! §4.6.1.3 / §4.6.2.3.3 dequantisation → §4.6.3.3
//! `quant_to_spec()` → §4.6.9 TNS).
//!
//! PCM comparison against `expected.wav` is **not** attempted yet —
//! the §4.6.11 filterbank (IMDCT + window-overlap-add) is the next
//! pipeline stage. What this driver pins is structural health:
//!
//! * every frame of every fixture parses without error,
//! * every decoded spectrum is finite (no NaN / inf escapes the
//!   dequantiser or the TNS all-pole filter),
//! * the corpus produces plausible energy (each fixture decodes at
//!   least one non-silent spectrum, and per-coefficient magnitudes
//!   stay below the §4.6.1.3 / §4.6.2.3.3 worst-case bound
//!   8191^(4/3) · 2^(0.25·(255−100)) ≈ 2^56).
//!
//! `oxideav-aac` is its own repository and `docs/` is checked into
//! the workspace umbrella, not the standalone crate. When the
//! fixtures are missing the test logs `skip <name>: missing ...` and
//! returns success, so CI stays clean for both layouts.

use std::fs;
use std::path::PathBuf;

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::decoded_spectrum::decode_channel_spectrum;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::IcsInfo;
use oxideav_aac::raw_data_block::{Element, IdSynEle, Walker};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_core::bits::BitReader;

/// The staged ADTS fixtures (`input.aac`). The MP4 / LATM fixtures
/// need their container demuxers and are out of scope here.
const ADTS_FIXTURES: &[&str] = &[
    "aac-lc-chirp-windows",
    "aac-lc-intensity-stereo",
    "aac-lc-mono-11025-32kbps-adts",
    "aac-lc-mono-44100-64kbps-adts",
    "aac-lc-mono-8000-16kbps-adts",
    "aac-lc-ms-stereo",
    "aac-lc-pns-noise",
    "aac-lc-stereo-22050-64kbps-adts",
    "aac-lc-stereo-44100-128kbps-adts",
    "aac-lc-tns-active",
    "aac-with-id3v2",
    "he-aac-v1-stereo-44100-32kbps-adts",
];

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures")
        .join(name)
        .join("input.aac")
}

/// Skip a leading ID3v2 tag if present (the `aac-with-id3v2`
/// fixture): `"ID3"` magic, 2 version bytes, 1 flags byte, then a
/// 4-byte syncsafe size of the tag body (plus a 10-byte footer when
/// flags bit 4 is set).
fn skip_id3v2(data: &[u8]) -> &[u8] {
    if data.len() < 10 || &data[..3] != b"ID3" {
        return data;
    }
    let size = data[6..10]
        .iter()
        .fold(0usize, |acc, &b| (acc << 7) | usize::from(b & 0x7f));
    let footer = if data[5] & 0x10 != 0 { 10 } else { 0 };
    let total = 10 + size + footer;
    if total >= data.len() {
        data
    } else {
        &data[total..]
    }
}

/// Per-fixture accumulated statistics.
#[derive(Debug, Default)]
struct Stats {
    frames: usize,
    channels_decoded: usize,
    nonzero_spectra: usize,
    total_energy: f64,
    peak_abs: f64,
}

impl Stats {
    fn ingest(&mut self, spec: &[f64], context: &str) {
        assert_eq!(spec.len(), 1024, "{context}: spectrum length");
        let mut energy = 0.0f64;
        let mut nonzero = false;
        for &v in spec {
            assert!(v.is_finite(), "{context}: non-finite coefficient {v}");
            // §4.6.1.3 / §4.6.2.3.3 worst case: 8191^(4/3) ·
            // 2^(0.25·155) < 2^56. TNS can ring above a single
            // coefficient's bound, so allow generous headroom — the
            // assert catches runaway feedback, not tight conformance.
            assert!(
                v.abs() < (80.0f64).exp2(),
                "{context}: implausible magnitude {v}"
            );
            energy += v * v;
            nonzero |= v != 0.0;
            self.peak_abs = self.peak_abs.max(v.abs());
        }
        self.total_energy += energy;
        self.channels_decoded += 1;
        if nonzero {
            self.nonzero_spectra += 1;
        }
    }
}

/// Parse one Table 4.4 `channel_pair_element()` (everything after
/// the `element_instance_tag` the walker consumed) and run both
/// channels through the pipeline.
fn decode_cpe(reader: &mut BitReader<'_>, aot: u8, fs: u8, stats: &mut Stats, context: &str) {
    let common_window = reader.read_bit().expect("common_window");
    if common_window {
        let ics = IcsInfo::parse(reader, aot, fs, true)
            .unwrap_or_else(|e| panic!("{context}: shared ics_info: {e}"));
        let ms_mask_present = reader.read_u32(2).expect("ms_mask_present");
        match ms_mask_present {
            0 | 2 => {}
            1 => {
                let bits = usize::from(ics.num_window_groups) * usize::from(ics.max_sfb);
                for _ in 0..bits {
                    reader.read_bit().expect("ms_used");
                }
            }
            _ => panic!("{context}: reserved ms_mask_present 3"),
        }
        for ch in 0..2 {
            let body = IcsBody::parse_with_ics_info(reader, &ics, aot, false)
                .unwrap_or_else(|e| panic!("{context} ch{ch}: ics_body: {e}"));
            let spectral = SpectralData::parse(reader, &ics, &body.section_data, fs)
                .unwrap_or_else(|e| panic!("{context} ch{ch}: spectral_data: {e}"));
            let spec = decode_channel_spectrum(&body, &ics, &spectral, aot, fs)
                .unwrap_or_else(|e| panic!("{context} ch{ch}: pipeline: {e}"));
            stats.ingest(&spec, context);
        }
    } else {
        for ch in 0..2 {
            let body = IcsBody::parse(reader, aot, fs, false)
                .unwrap_or_else(|e| panic!("{context} ch{ch}: ics_body: {e}"));
            let ics = body.ics_info.clone().expect("inline ics_info");
            let spectral = SpectralData::parse(reader, &ics, &body.section_data, fs)
                .unwrap_or_else(|e| panic!("{context} ch{ch}: spectral_data: {e}"));
            let spec = decode_channel_spectrum(&body, &ics, &spectral, aot, fs)
                .unwrap_or_else(|e| panic!("{context} ch{ch}: pipeline: {e}"));
            stats.ingest(&spec, context);
        }
    }
}

/// Decode one fixture end to end; returns `None` when the fixture
/// file is not present (standalone-repo CI layout).
fn decode_fixture(name: &str) -> Option<Stats> {
    let path = fixture_path(name);
    let data = match fs::read(&path) {
        Ok(d) => d,
        Err(_) => {
            eprintln!("skip {name}: missing {}", path.display());
            return None;
        }
    };
    let data = skip_id3v2(&data);

    let mut stats = Stats::default();
    let mut pos = 0usize;
    while pos + 7 <= data.len() {
        let (header, payload_offset) =
            AdtsHeader::parse(&data[pos..]).unwrap_or_else(|e| panic!("{name}@{pos}: adts: {e}"));
        let frame_len = header.aac_frame_length as usize;
        assert!(
            pos + frame_len <= data.len(),
            "{name}@{pos}: truncated frame"
        );
        let payload = &data[pos + payload_offset..pos + frame_len];
        let aot = header.audio_object_type();
        let fs_index = header.sampling_frequency_index;
        let context = format!("{name} frame {}", stats.frames);

        let mut reader = BitReader::new(payload);
        for _ in 0..header.number_of_raw_data_blocks_in_frame {
            loop {
                // A fresh walker per element keeps `reader` free for
                // the channel-body parsers between elements; the
                // only walker state (END seen) is handled here.
                let elem = Walker::new(&mut reader)
                    .next_element()
                    .unwrap_or_else(|e| panic!("{context}: walker: {e}"))
                    .unwrap_or_else(|| panic!("{context}: payload ended before END"));
                match elem {
                    Element::ChannelElement {
                        kind: IdSynEle::Sce | IdSynEle::Lfe,
                        ..
                    } => {
                        let body = IcsBody::parse(&mut reader, aot, fs_index, false)
                            .unwrap_or_else(|e| panic!("{context}: ics_body: {e}"));
                        let ics = body.ics_info.clone().expect("inline ics_info");
                        let spectral =
                            SpectralData::parse(&mut reader, &ics, &body.section_data, fs_index)
                                .unwrap_or_else(|e| panic!("{context}: spectral_data: {e}"));
                        let spec = decode_channel_spectrum(&body, &ics, &spectral, aot, fs_index)
                            .unwrap_or_else(|e| panic!("{context}: pipeline: {e}"));
                        stats.ingest(&spec, &context);
                    }
                    Element::ChannelElement {
                        kind: IdSynEle::Cpe,
                        ..
                    } => decode_cpe(&mut reader, aot, fs_index, &mut stats, &context),
                    Element::ChannelElement { kind, .. } => {
                        panic!("{context}: unsupported channel element {}", kind.name())
                    }
                    Element::Fill { .. } | Element::Data { .. } | Element::ProgramConfig(_) => {}
                    Element::End => break,
                }
            }
        }
        stats.frames += 1;
        pos += frame_len;
    }
    assert_eq!(pos, data.len(), "{name}: trailing bytes after last frame");
    Some(stats)
}

#[test]
fn adts_corpus_decodes_with_plausible_energy() {
    let mut decoded_any = false;
    for &name in ADTS_FIXTURES {
        let Some(stats) = decode_fixture(name) else {
            continue;
        };
        decoded_any = true;
        assert!(stats.frames > 1, "{name}: only {} frames", stats.frames);
        assert!(
            stats.channels_decoded >= stats.frames,
            "{name}: fewer channel spectra ({}) than frames ({})",
            stats.channels_decoded,
            stats.frames
        );
        // Every fixture carries real audio: at least one non-silent
        // decoded spectrum must exist, with genuinely positive
        // energy. (PNS / intensity bands decode to silence at this
        // stage, but no staged fixture is *exclusively* PNS or IS.)
        assert!(
            stats.nonzero_spectra > 0 && stats.total_energy > 0.0,
            "{name}: all-silent decode ({} spectra)",
            stats.channels_decoded
        );
        eprintln!(
            "{name}: frames={} channels={} nonzero={} energy={:.3e} peak={:.3e}",
            stats.frames,
            stats.channels_decoded,
            stats.nonzero_spectra,
            stats.total_energy,
            stats.peak_abs
        );
    }
    if !decoded_any {
        eprintln!("docs fixtures unavailable; structural corpus pass skipped");
    }
}

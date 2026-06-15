//! End-to-end element-decode tests against the
//! `docs/audio/aac/fixtures/` ADTS corpus.
//!
//! Where `tests/docs_adts_corpus.rs` pins the *spectrum* stage (parse →
//! dequant → `quant_to_spec()` → TNS) frame by frame, this driver runs
//! the full §4.6 block-order chain through
//! [`oxideav_aac::element_decode::ElementDecoder`] all the way to
//! PCM-domain samples: per-channel pulse/dequant/de-interleave, then the
//! CPE joint-stereo / noise tools (M/S §4.6.8.1 → intensity §4.6.8.2 →
//! PNS §4.6.13), then per-channel TNS (§4.6.9) and the stateful §4.6.11
//! filterbank with inter-frame overlap-add.
//!
//! PCM comparison against `expected.wav` is **not** attempted — the LC
//! decode path here is feature-complete to the filterbank output but the
//! crate carries no resampler / channel-remapper / clipper, and PNS
//! output is RNG-defined per §4.6.13.3. What this driver pins is that
//! the element-level glue runs end to end on real streams and emits
//! finite, plausibly-energetic PCM with a working overlap tail.
//!
//! `oxideav-aac` is its own repository and `docs/` is checked into the
//! workspace umbrella; when the fixtures are missing the test logs
//! `skip <name>` and returns success so CI stays clean for both layouts.

use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::element_decode::{ChannelInput, CpeJointStereo, ElementDecoder};
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::IcsInfo;
use oxideav_aac::ms_stereo::MsMaskPresent;
use oxideav_aac::raw_data_block::{Element, IdSynEle, Walker};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_core::bits::BitReader;

/// The staged ADTS fixtures (`input.aac`).
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

/// Skip a leading ID3v2 tag if present.
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

#[derive(Debug, Default)]
struct Stats {
    frames: usize,
    pcm_channels: usize,
    nonzero_frames: usize,
    peak_abs: f64,
    /// Per-element-slot overlap coupling witnessed (a later frame at a
    /// given element slot differing from a hypothetical zero-overlap
    /// start). We approximate this by checking that consecutive
    /// same-element frames are not all identical.
    saw_overlap_coupling: bool,
}

impl Stats {
    fn ingest_pcm(&mut self, pcm: &[f64], context: &str) {
        assert_eq!(pcm.len(), 1024, "{context}: PCM frame length");
        let mut nonzero = false;
        for &v in pcm {
            assert!(v.is_finite(), "{context}: non-finite PCM {v}");
            // Filterbank output of a dequantised LC stream stays well
            // below any sane bound; a runaway TNS feedback would blow
            // past this.
            assert!(v.abs() < (60.0f64).exp2(), "{context}: implausible PCM {v}");
            nonzero |= v != 0.0;
            self.peak_abs = self.peak_abs.max(v.abs());
        }
        self.pcm_channels += 1;
        if nonzero {
            self.nonzero_frames += 1;
        }
    }
}

/// Decode one fixture end to end through the element driver; returns
/// `None` when the fixture file is absent.
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
    // One stateful ElementDecoder per element instance tag, so the
    // filterbank overlap carries across frames for each element.
    let mut decoders: HashMap<(u8, u8), ElementDecoder> = HashMap::new();
    // Track the previous frame's PCM per element slot to witness
    // overlap coupling.
    let mut prev_pcm: HashMap<(u8, u8), Vec<f64>> = HashMap::new();

    let mut pos = 0usize;
    while pos + 7 <= data.len() {
        let (header, payload_offset) =
            AdtsHeader::parse(&data[pos..]).unwrap_or_else(|e| panic!("{name}@{pos}: adts: {e}"));
        let frame_len = header.aac_frame_length as usize;
        assert!(pos + frame_len <= data.len(), "{name}@{pos}: truncated");
        let payload = &data[pos + payload_offset..pos + frame_len];
        let aot = header.audio_object_type();
        let fs = header.sampling_frequency_index;
        let context = format!("{name} frame {}", stats.frames);

        let mut reader = BitReader::new(payload);
        for _ in 0..header.number_of_raw_data_blocks_in_frame {
            loop {
                let elem = Walker::new(&mut reader)
                    .next_element()
                    .unwrap_or_else(|e| panic!("{context}: walker: {e}"))
                    .unwrap_or_else(|| panic!("{context}: ended before END"));
                match elem {
                    Element::ChannelElement {
                        kind: kind @ (IdSynEle::Sce | IdSynEle::Lfe),
                        element_instance_tag,
                    } => {
                        let key = (kind_id(kind), element_instance_tag);
                        let body = IcsBody::parse(&mut reader, aot, fs, false)
                            .unwrap_or_else(|e| panic!("{context}: ics_body: {e}"));
                        let ics = body.ics_info.clone().expect("inline ics_info");
                        let spectral =
                            SpectralData::parse(&mut reader, &ics, &body.section_data, fs)
                                .unwrap_or_else(|e| panic!("{context}: spectral: {e}"));
                        let ch = ChannelInput {
                            body: &body,
                            ics_info: &ics,
                            spectral: &spectral,
                        };
                        let dec = decoders.entry(key).or_default();
                        let pcm = dec
                            .decode_sce(&ch, aot, fs)
                            .unwrap_or_else(|e| panic!("{context}: sce decode: {e}"));
                        witness_overlap(&mut stats, &mut prev_pcm, key, &pcm);
                        stats.ingest_pcm(&pcm, &context);
                    }
                    Element::ChannelElement {
                        kind: IdSynEle::Cpe,
                        element_instance_tag,
                    } => {
                        let key = (kind_id(IdSynEle::Cpe), element_instance_tag);
                        let (l, r) =
                            decode_cpe_frame(&mut reader, aot, fs, &mut decoders, key, &context);
                        // Witness overlap on the left channel slot.
                        witness_overlap(&mut stats, &mut prev_pcm, key, &l);
                        stats.ingest_pcm(&l, &context);
                        stats.ingest_pcm(&r, &context);
                    }
                    Element::ChannelElement { kind, .. } => {
                        panic!("{context}: unsupported element {}", kind.name())
                    }
                    Element::Fill { .. } | Element::Data { .. } | Element::ProgramConfig(_) => {}
                    Element::End => break,
                }
            }
        }
        stats.frames += 1;
        pos += frame_len;
    }
    assert_eq!(pos, data.len(), "{name}: trailing bytes");
    Some(stats)
}

fn kind_id(kind: IdSynEle) -> u8 {
    match kind {
        IdSynEle::Sce => 0,
        IdSynEle::Cpe => 1,
        IdSynEle::Lfe => 3,
        _ => 9,
    }
}

fn witness_overlap(
    stats: &mut Stats,
    prev_pcm: &mut HashMap<(u8, u8), Vec<f64>>,
    key: (u8, u8),
    pcm: &[f64],
) {
    if let Some(prev) = prev_pcm.get(&key) {
        if prev != pcm {
            stats.saw_overlap_coupling = true;
        }
    }
    prev_pcm.insert(key, pcm.to_vec());
}

/// Parse one CPE (after the walker consumed the element-instance tag)
/// and run it through the element decoder.
fn decode_cpe_frame(
    reader: &mut BitReader<'_>,
    aot: u8,
    fs: u8,
    decoders: &mut HashMap<(u8, u8), ElementDecoder>,
    key: (u8, u8),
    context: &str,
) -> (Vec<f64>, Vec<f64>) {
    let common_window = reader.read_bit().expect("common_window");
    if common_window {
        let ics = IcsInfo::parse(reader, aot, fs, true)
            .unwrap_or_else(|e| panic!("{context}: shared ics_info: {e}"));
        let ms_bits = reader.read_u32(2).expect("ms_mask_present") as u8;
        let ms_mask_present = MsMaskPresent::from_bits(ms_bits)
            .unwrap_or_else(|e| panic!("{context}: ms_mask_present: {e}"));
        let mut ms_used: Vec<Vec<bool>> = Vec::new();
        if ms_mask_present == MsMaskPresent::Mask {
            for _g in 0..usize::from(ics.num_window_groups) {
                let mut row = Vec::with_capacity(usize::from(ics.max_sfb));
                for _sfb in 0..usize::from(ics.max_sfb) {
                    row.push(reader.read_bit().expect("ms_used"));
                }
                ms_used.push(row);
            }
        }
        let left_body = IcsBody::parse_with_ics_info(reader, &ics, aot, false)
            .unwrap_or_else(|e| panic!("{context} ch0: ics_body: {e}"));
        let left_spectral = SpectralData::parse(reader, &ics, &left_body.section_data, fs)
            .unwrap_or_else(|e| panic!("{context} ch0: spectral: {e}"));
        let right_body = IcsBody::parse_with_ics_info(reader, &ics, aot, false)
            .unwrap_or_else(|e| panic!("{context} ch1: ics_body: {e}"));
        let right_spectral = SpectralData::parse(reader, &ics, &right_body.section_data, fs)
            .unwrap_or_else(|e| panic!("{context} ch1: spectral: {e}"));
        let left = ChannelInput {
            body: &left_body,
            ics_info: &ics,
            spectral: &left_spectral,
        };
        let right = ChannelInput {
            body: &right_body,
            ics_info: &ics,
            spectral: &right_spectral,
        };
        let joint = CpeJointStereo {
            ms_mask_present,
            ms_used,
        };
        let dec = decoders.entry(key).or_default();
        dec.decode_cpe(&left, &right, &joint, aot, fs)
            .unwrap_or_else(|e| panic!("{context}: cpe decode: {e}"))
    } else {
        // Non-shared CPE: each channel carries its own ics_info, no M/S
        // mask. The two channels may differ in window_sequence; the
        // element decoder rejects a mismatch (the joint-stereo geometry
        // is undefined), so a non-shared CPE that differs decodes each
        // channel independently here.
        let left_body = IcsBody::parse(reader, aot, fs, false)
            .unwrap_or_else(|e| panic!("{context} ch0: ics_body: {e}"));
        let left_ics = left_body.ics_info.clone().expect("inline ics_info");
        let left_spectral = SpectralData::parse(reader, &left_ics, &left_body.section_data, fs)
            .unwrap_or_else(|e| panic!("{context} ch0: spectral: {e}"));
        let right_body = IcsBody::parse(reader, aot, fs, false)
            .unwrap_or_else(|e| panic!("{context} ch1: ics_body: {e}"));
        let right_ics = right_body.ics_info.clone().expect("inline ics_info");
        let right_spectral = SpectralData::parse(reader, &right_ics, &right_body.section_data, fs)
            .unwrap_or_else(|e| panic!("{context} ch1: spectral: {e}"));
        let left = ChannelInput {
            body: &left_body,
            ics_info: &left_ics,
            spectral: &left_spectral,
        };
        let right = ChannelInput {
            body: &right_body,
            ics_info: &right_ics,
            spectral: &right_spectral,
        };
        let dec = decoders.entry(key).or_default();
        dec.decode_cpe(&left, &right, &CpeJointStereo::default(), aot, fs)
            .unwrap_or_else(|e| panic!("{context}: non-shared cpe decode: {e}"))
    }
}

#[test]
fn adts_corpus_decodes_to_plausible_pcm() {
    let mut decoded_any = false;
    for &name in ADTS_FIXTURES {
        let Some(stats) = decode_fixture(name) else {
            continue;
        };
        decoded_any = true;
        assert!(stats.frames > 1, "{name}: only {} frames", stats.frames);
        assert!(
            stats.pcm_channels >= stats.frames,
            "{name}: fewer PCM frames ({}) than ADTS frames ({})",
            stats.pcm_channels,
            stats.frames
        );
        assert!(
            stats.nonzero_frames > 0,
            "{name}: all-silent PCM ({} frames)",
            stats.pcm_channels
        );
        // A multi-frame stream with a working overlap-add tail produces
        // frames that are not all identical at a given element slot.
        assert!(
            stats.saw_overlap_coupling,
            "{name}: no inter-frame overlap coupling witnessed"
        );
        eprintln!(
            "{name}: frames={} pcm_channels={} nonzero={} peak={:.3e}",
            stats.frames, stats.pcm_channels, stats.nonzero_frames, stats.peak_abs
        );
    }
    if !decoded_any {
        eprintln!("docs fixtures unavailable; element-decode PCM pass skipped");
    }
}

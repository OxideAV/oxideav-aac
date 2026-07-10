//! CRC-bearing docs fixtures — the `EXT_SBR_DATA_CRC` (type 14) and
//! ADTS `error_check()` streams the encoder corpus cannot produce
//! (per `docs/audio/aac/cce-hcr-ssr-fixtures.md` §4/§6: no available
//! encoder emits either form; the documented closure path is a
//! deterministic rewrite of an existing staged fixture).
//!
//! Two derived fixtures are staged under `docs/audio/aac/fixtures/`:
//!
//! * `aac-lc-adts-crc` — `aac-lc-stereo-44100-128kbps-adts` with every
//!   frame rewritten to `protection_absent == 0` and a Table 1.A.8
//!   `crc_check` computed over the ISO/IEC 13818-7:2004 §8.1.1.1
//!   region (via `adts_crc::protect_adts_stream`).
//! * `he-aac-v1-sbrcrc-adts` — `he-aac-v1-stereo-44100-32kbps-adts`
//!   with every SBR fill extension flipped from `EXT_SBR_DATA` (13)
//!   to `EXT_SBR_DATA_CRC` (14): a 10-bit `G10` CRC (zero init) over
//!   the §4.4.2.8.1 coverage region is inserted ahead of the
//!   unchanged SBR payload bits and the fill length is re-derived.
//!
//! Both rewrites are *pure protection additions*: this driver pins
//! that each derived stream decodes bit-identically to its source
//! fixture, and that the staged bytes equal the in-code rewrite (the
//! recipe is the test itself; regenerate with
//! `OXIDEAV_AAC_STAGE_FIXTURES=1 cargo test --test docs_crc_fixtures`).
//! When the docs corpus is not checked out (standalone-repo CI) every
//! test logs a skip and passes.

use std::fs;
use std::path::PathBuf;

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::adts_crc::{protect_adts_stream, sbr_crc};
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::extension_payload::{ExtensionPayload, ExtensionPayloadOrSbr};
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::IcsInfo;
use oxideav_aac::raw_data_block::{Element, FrameAssembler, IdSynEle, Walker};
use oxideav_aac::sbr_header::SbrHeader;
use oxideav_aac::spectral_data::SpectralData;
use oxideav_core::bits::{BitReader, BitWriter};

fn fixtures_root() -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures")
}

fn stage_enabled() -> bool {
    std::env::var_os("OXIDEAV_AAC_STAGE_FIXTURES").is_some()
}

/// Extract the bit range `[start, end)` of `data` into an MSB-packed
/// byte buffer.
fn extract_bits(data: &[u8], start: u64, end: u64) -> Vec<u8> {
    let n = (end - start) as usize;
    let mut out = vec![0u8; n.div_ceil(8)];
    for i in 0..n {
        let pos = start + i as u64;
        if data[(pos / 8) as usize] & (0x80 >> (pos % 8)) != 0 {
            out[i / 8] |= 0x80 >> (i % 8);
        }
    }
    out
}

/// Append the bit range `[start, end)` of `data` to a channel-element
/// body in the assembler.
fn copy_channel_body(fa: &mut FrameAssembler, data: &[u8], start: u64, end: u64) {
    let bits = extract_bits(data, start, end);
    fa.push_channel_body_bits(&bits, end - start).unwrap();
}

/// Parse-only walk of one CPE body (after the walker consumed the
/// instance tag), advancing `reader` to the element end.
fn skip_cpe(reader: &mut BitReader<'_>, aot: u8, fs: u8) {
    let common_window = reader.read_bit().unwrap();
    if common_window {
        let ics = IcsInfo::parse(reader, aot, fs, true).unwrap();
        let ms_mask_present = reader.read_u32(2).unwrap();
        if ms_mask_present == 1 {
            for _ in 0..usize::from(ics.num_window_groups) * usize::from(ics.max_sfb) {
                reader.read_bit().unwrap();
            }
        }
        for _ in 0..2 {
            let body = IcsBody::parse_with_ics_info(reader, &ics, aot, false).unwrap();
            SpectralData::parse(reader, &ics, &body.section_data, fs).unwrap();
        }
    } else {
        for _ in 0..2 {
            let body = IcsBody::parse(reader, aot, fs, false).unwrap();
            let ics = body.ics_info.clone().unwrap();
            SpectralData::parse(reader, &ics, &body.section_data, fs).unwrap();
        }
    }
}

/// Rewrite every `EXT_SBR_DATA` (type 13) fill extension of an ADTS
/// HE-AAC stream into `EXT_SBR_DATA_CRC` (type 14): the extension
/// nibble becomes `1110`, a correctly computed 10-bit `G10` CRC (zero
/// init, over the unchanged SBR payload bits — exactly the
/// §4.4.2.8.1 coverage region) is inserted, and the fill byte count /
/// `bs_fill_bits` alignment and the ADTS `aac_frame_length` are
/// re-derived. Everything else is copied bit-verbatim.
fn rewrite_sbr_to_crc(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() + 128);
    let mut prev_header: Option<SbrHeader> = None;
    let mut pos = 0usize;
    while pos + 7 <= data.len() {
        let (header, payload_offset) = AdtsHeader::parse(&data[pos..]).unwrap();
        assert!(header.protection_absent, "source fixture carries no CRC");
        let frame_len = header.aac_frame_length as usize;
        let payload = &data[pos + payload_offset..pos + frame_len];
        let aot = header.audio_object_type();
        let fs = header.sampling_frequency_index;
        let fs_sbr = header.sample_rate() * 2;

        let mut reader = BitReader::new(payload);
        let mut fa = FrameAssembler::new();
        let mut last_kind: Option<IdSynEle> = None;
        while let Some(elem) = Walker::new(&mut reader).next_element_keep_fill().unwrap() {
            match elem {
                Element::ChannelElement {
                    kind: kind @ (IdSynEle::Sce | IdSynEle::Lfe),
                    element_instance_tag,
                } => {
                    let body_start = reader.bit_position();
                    let body = IcsBody::parse(&mut reader, aot, fs, false).unwrap();
                    let ics = body.ics_info.clone().unwrap();
                    SpectralData::parse(&mut reader, &ics, &body.section_data, fs).unwrap();
                    fa.push_channel_header(kind, element_instance_tag).unwrap();
                    copy_channel_body(&mut fa, payload, body_start, reader.bit_position());
                    last_kind = Some(kind);
                }
                Element::ChannelElement {
                    kind: IdSynEle::Cpe,
                    element_instance_tag,
                } => {
                    let body_start = reader.bit_position();
                    skip_cpe(&mut reader, aot, fs);
                    fa.push_channel_header(IdSynEle::Cpe, element_instance_tag)
                        .unwrap();
                    copy_channel_body(&mut fa, payload, body_start, reader.bit_position());
                    last_kind = Some(IdSynEle::Cpe);
                }
                Element::ChannelElement { kind, .. } => {
                    panic!("unexpected channel element {} in source", kind.name())
                }
                Element::Fill { payload_bytes } => {
                    let body_start = reader.bit_position();
                    if payload_bytes == 0 {
                        // Zero-count FIL: no extension_payload body.
                        fa.push_fill(&[]).unwrap();
                        continue;
                    }
                    if reader.peek_u32(4).unwrap() != 0b1101 {
                        // Non-SBR FIL: copy its cnt bytes verbatim,
                        // uninterpreted.
                        reader.skip(payload_bytes * 8).unwrap();
                        let body = extract_bits(
                            payload,
                            body_start,
                            body_start + u64::from(payload_bytes) * 8,
                        );
                        fa.push_fill(&body).unwrap();
                        continue;
                    }
                    let id_aac = last_kind.expect("SBR FIL follows a channel element");
                    let ExtensionPayloadOrSbr::Sbr(ext) = ExtensionPayload::parse_with_sbr(
                        &mut reader,
                        payload_bytes,
                        id_aac,
                        fs_sbr,
                        prev_header,
                    )
                    .unwrap() else {
                        unreachable!("type nibble 13 always dispatches to the SBR walker");
                    };
                    assert!(ext.crc.is_none(), "source SBR is the type-13 form");
                    prev_header = Some(ext.header);
                    // Coverage region: the num_sbr_bits after the type
                    // nibble (no CRC on the type-13 wire).
                    let sbr_start = body_start + 4;
                    let sbr_end = sbr_start + ext.num_sbr_bits;
                    let crc = sbr_crc(payload, sbr_start, sbr_end);
                    let mut w = BitWriter::new();
                    w.write_u32(0b1110, 4); // EXT_SBR_DATA_CRC
                    w.write_u32(u32::from(crc), 10); // bs_sbr_crc_bits
                    for i in sbr_start..sbr_end {
                        let bit = payload[(i / 8) as usize] & (0x80 >> (i % 8)) != 0;
                        w.write_bit(bit);
                    }
                    w.align_to_byte_zero(); // bs_fill_bits
                    fa.push_fill(&w.finish()).unwrap();
                }
                Element::Data { .. } | Element::ProgramConfig(_) => {
                    panic!("unexpected DSE/PCE in source fixture")
                }
                Element::End => break,
            }
        }
        let new_payload = fa.push_end();

        // Patch the 7 header bytes: only aac_frame_length changes.
        let new_len = (7 + new_payload.len()) as u16;
        assert!(new_len < (1 << 13));
        let mut h = [0u8; 7];
        h.copy_from_slice(&data[pos..pos + 7]);
        h[3] = (h[3] & 0xFC) | ((new_len >> 11) as u8 & 0x03);
        h[4] = (new_len >> 3) as u8;
        h[5] = (h[5] & 0x1F) | (((new_len & 0x07) as u8) << 5);
        out.extend_from_slice(&h);
        out.extend_from_slice(&new_payload);
        pos += frame_len;
    }
    out
}

/// Load a source fixture's `input.aac`, or log a skip.
fn source_input(name: &str) -> Option<Vec<u8>> {
    let path = fixtures_root().join(name).join("input.aac");
    match fs::read(&path) {
        Ok(d) => Some(d),
        Err(_) => {
            eprintln!("skip: missing {}", path.display());
            None
        }
    }
}

/// Stage (env-gated) or pin the derived fixture bytes, then check the
/// bit-identical-decode property against the source stream.
fn stage_or_pin(name: &str, derived: &[u8], source: &[u8]) {
    let dir = fixtures_root().join(name);
    let input = dir.join("input.aac");
    if stage_enabled() {
        fs::create_dir_all(&dir).unwrap();
        fs::write(&input, derived).unwrap();
        eprintln!("staged {} ({} bytes)", input.display(), derived.len());
    } else if let Ok(staged) = fs::read(&input) {
        assert_eq!(
            staged, derived,
            "{name}: staged input.aac differs from the in-code rewrite"
        );
    } else {
        eprintln!("skip: {} not staged yet", input.display());
    }

    // Protection must be transparent: identical PCM, frame for frame.
    let base = StreamDecoder::new().decode_all(source).unwrap();
    let checked = StreamDecoder::new().decode_all(derived).unwrap();
    assert_eq!(base.len(), checked.len());
    for (a, b) in base.iter().zip(checked.iter()) {
        assert_eq!(a.pcm, b.pcm);
        assert_eq!(a.channels, b.channels);
        assert_eq!(a.sample_rate, b.sample_rate);
    }
    assert!(base.iter().any(|f| f.pcm.iter().any(|&s| s != 0)));
}

#[test]
fn adts_crc_fixture_matches_rewrite_and_decodes_identically() {
    let Some(source) = source_input("aac-lc-stereo-44100-128kbps-adts") else {
        return;
    };
    let derived = protect_adts_stream(&source).unwrap();
    let (h, off) = AdtsHeader::parse(&derived).unwrap();
    assert!(!h.protection_absent);
    assert_eq!(off, 9);
    stage_or_pin("aac-lc-adts-crc", &derived, &source);
}

#[test]
fn sbr_crc_fixture_matches_rewrite_and_decodes_identically() {
    let Some(source) = source_input("he-aac-v1-stereo-44100-32kbps-adts") else {
        return;
    };
    let derived = rewrite_sbr_to_crc(&source);
    // The rewrite produced dual-rate SBR output (22.05 kHz core →
    // 44.1 kHz) with verified CRCs on every frame (decode_all would
    // reject any mismatch).
    let frames = StreamDecoder::new().decode_all(&derived).unwrap();
    assert!(frames.iter().any(|f| f.sample_rate == 44_100));
    stage_or_pin("he-aac-v1-sbrcrc-adts", &derived, &source);
}

#[test]
fn sbr_crc_fixture_corruption_is_rejected() {
    // The derived stream's CRC gate actually fires: flip a bit inside
    // the first frame's SBR payload region (after the FIL's CRC
    // field) and expect the decode to fail.
    let Some(source) = source_input("he-aac-v1-stereo-44100-32kbps-adts") else {
        return;
    };
    let derived = rewrite_sbr_to_crc(&source);

    // Find the first frame's FIL body: walk elements until Fill.
    let (header, payload_offset) = AdtsHeader::parse(&derived).unwrap();
    let payload = &derived[payload_offset..header.aac_frame_length as usize];
    let mut reader = BitReader::new(payload);
    let aot = header.audio_object_type();
    let fs = header.sampling_frequency_index;
    let fil_body_bit = loop {
        match Walker::new(&mut reader).next_element_keep_fill().unwrap() {
            Some(Element::ChannelElement {
                kind: IdSynEle::Cpe,
                ..
            }) => skip_cpe(&mut reader, aot, fs),
            Some(Element::ChannelElement {
                kind: IdSynEle::Sce | IdSynEle::Lfe,
                ..
            }) => {
                let body = IcsBody::parse(&mut reader, aot, fs, false).unwrap();
                let ics = body.ics_info.clone().unwrap();
                SpectralData::parse(&mut reader, &ics, &body.section_data, fs).unwrap();
            }
            Some(Element::Fill { payload_bytes }) => {
                if reader.peek_u32(4).unwrap() == 0b1110 {
                    break reader.bit_position();
                }
                reader.skip(payload_bytes * 8).unwrap();
            }
            Some(_) => {}
            None => panic!("no type-14 FIL in first frame"),
        }
    };
    // extension_type(4) + bs_sbr_crc_bits(10) + bs_header_flag(1) +
    // bs_amp_res(1) + start(4) + stop(4) + xover(3) → bs_reserved at
    // +27: covered by the CRC but structurally inert (read, ignored),
    // so the failure is unambiguously the CRC gate.
    let bit = u64::from(payload_offset as u32) * 8 + fil_body_bit + 27;
    let mut bad = derived.clone();
    bad[(bit / 8) as usize] ^= 0x80 >> (bit % 8);
    assert!(matches!(
        StreamDecoder::new().decode_all(&bad),
        Err(oxideav_aac::Error::SbrCrcMismatch)
    ));
}

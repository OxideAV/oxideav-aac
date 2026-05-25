//! Tests for the [`oxideav_aac::raw_data_block::Walker`] — synthetic
//! `raw_data_block()` payloads constructed via
//! `oxideav_core::bits::BitWriter` so the tests do not depend on any
//! external AAC encoder.

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::raw_data_block::{Element, IdSynEle, Walker};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// Append a SCE header (id 0b000 + 4-bit element_instance_tag).
fn write_sce(bw: &mut BitWriter, tag: u8) {
    bw.write_u32(IdSynEle::Sce as u32, 3);
    bw.write_u32(tag as u32, 4);
}

/// Append a FIL element with zero payload bytes (id 0b110 + 4-bit
/// count, no esc_count, no payload).
fn write_fil_empty(bw: &mut BitWriter) {
    bw.write_u32(IdSynEle::Fil as u32, 3);
    bw.write_u32(0, 4);
}

/// Append a FIL element with `n` payload bytes (n < 15).
fn write_fil_short(bw: &mut BitWriter, n: u32) {
    assert!(n < 15);
    bw.write_u32(IdSynEle::Fil as u32, 3);
    bw.write_u32(n, 4);
    for _ in 0..n {
        bw.write_u32(0xAA, 8);
    }
}

/// Append a FIL element using the 8-bit `esc_count` escape (count == 15).
fn write_fil_escape(bw: &mut BitWriter, esc_count: u8) {
    bw.write_u32(IdSynEle::Fil as u32, 3);
    bw.write_u32(15, 4);
    bw.write_u32(esc_count as u32, 8);
    // resulting payload = esc_count + 15 - 1
    let n = (esc_count as u32) + 15 - 1;
    for _ in 0..n {
        bw.write_u32(0x55, 8);
    }
}

/// Append a DSE element with `n` payload bytes (n < 255) and the
/// requested byte_align flag.
fn write_dse(bw: &mut BitWriter, tag: u8, byte_align: bool, n: u8) {
    bw.write_u32(IdSynEle::Dse as u32, 3);
    bw.write_u32(tag as u32, 4);
    bw.write_bit(byte_align);
    bw.write_u32(n as u32, 8);
    if byte_align {
        bw.align_to_byte_zero();
    }
    for _ in 0..n {
        bw.write_u32(0xCC, 8);
    }
}

/// Append the END terminator and byte-align.
fn write_end(bw: &mut BitWriter) {
    bw.write_u32(IdSynEle::End as u32, 3);
    bw.align_to_byte_zero();
}

#[test]
fn walks_sce_fil_end_in_order_and_stops_cleanly() {
    // Hand-crafted SCE-FIL-END raw_data_block:
    //   SCE (id=000) + element_instance_tag=0000
    //   FIL (id=110) + count=0000 (zero payload bytes)
    //   END (id=111) + byte-align
    let mut bw = BitWriter::new();
    write_sce(&mut bw, 0);
    write_fil_empty(&mut bw);
    write_end(&mut bw);
    let payload = bw.finish();

    let mut br = BitReader::new(&payload);
    let mut walker = Walker::new(&mut br);

    let first = walker.next_element().unwrap().unwrap();
    assert_eq!(
        first,
        Element::ChannelElement {
            kind: IdSynEle::Sce,
            element_instance_tag: 0,
        }
    );

    let second = walker.next_element().unwrap().unwrap();
    assert_eq!(second, Element::Fill { payload_bytes: 0 });

    let third = walker.next_element().unwrap().unwrap();
    assert_eq!(third, Element::End);

    assert!(walker.is_finished());
    // Further calls after END must report end-of-block, not error.
    assert_eq!(walker.next_element().unwrap(), None);
    assert_eq!(walker.next_element().unwrap(), None);
}

#[test]
fn walks_through_adts_header_and_raw_data_block() {
    // Build SCE-FIL-END payload, then prefix with an ADTS header
    // whose `aac_frame_length` matches the resulting total.
    let mut payload_bw = BitWriter::new();
    write_sce(&mut payload_bw, 5);
    write_fil_empty(&mut payload_bw);
    write_end(&mut payload_bw);
    let payload = payload_bw.finish();

    // Build header with frame_length = 7 + payload.len()
    let frame_length = (7 + payload.len()) as u16;
    let mut hdr_bw = BitWriter::new();
    hdr_bw.write_u32(0xFFF, 12); // sync
    hdr_bw.write_bit(false); // ID = MPEG-4
    hdr_bw.write_u32(0, 2); // layer
    hdr_bw.write_bit(true); // protection_absent
    hdr_bw.write_u32(1, 2); // profile = LC
    hdr_bw.write_u32(4, 4); // sfi = 44100
    hdr_bw.write_bit(false); // private
    hdr_bw.write_u32(1, 3); // chan_cfg = mono
    hdr_bw.write_u32(0, 4); // orig/home/copy/copy_start
    hdr_bw.write_u32(frame_length as u32, 13);
    hdr_bw.write_u32(0x7FF, 11);
    hdr_bw.write_u32(0, 2); // nrdb = 1
    let mut frame = hdr_bw.finish();
    frame.extend_from_slice(&payload);

    let (h, offset) = AdtsHeader::parse(&frame).unwrap();
    assert_eq!(offset, 7);
    assert_eq!(h.aac_frame_length as usize, frame.len());

    let mut br = BitReader::new(&frame[offset..]);
    let mut walker = Walker::new(&mut br);
    let kinds: Vec<_> = std::iter::from_fn(|| walker.next_element().transpose())
        .collect::<Result<Vec<_>, _>>()
        .unwrap();
    assert_eq!(kinds.len(), 3);
    match &kinds[0] {
        Element::ChannelElement {
            kind,
            element_instance_tag,
        } => {
            assert_eq!(*kind, IdSynEle::Sce);
            assert_eq!(*element_instance_tag, 5);
        }
        other => panic!("expected SCE channel element, got {:?}", other),
    }
    assert_eq!(kinds[1], Element::Fill { payload_bytes: 0 });
    assert_eq!(kinds[2], Element::End);
}

#[test]
fn fill_short_count_skips_payload_bytes() {
    let mut bw = BitWriter::new();
    write_fil_short(&mut bw, 5);
    write_end(&mut bw);
    let payload = bw.finish();

    let mut br = BitReader::new(&payload);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill { payload_bytes: 5 })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn fill_escape_count_decodes_per_spec() {
    // esc_count = 1 ⇒ payload bytes = 1 + 15 - 1 = 15
    let mut bw = BitWriter::new();
    write_fil_escape(&mut bw, 1);
    write_end(&mut bw);
    let payload = bw.finish();

    let mut br = BitReader::new(&payload);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill { payload_bytes: 15 })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn dse_with_byte_align_skips_payload() {
    let mut bw = BitWriter::new();
    write_dse(&mut bw, 3, true, 7);
    write_end(&mut bw);
    let payload = bw.finish();

    let mut br = BitReader::new(&payload);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Data {
            element_instance_tag: 3,
            byte_align_flag: true,
            payload_bytes: 7,
        })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn dse_without_byte_align_skips_payload() {
    let mut bw = BitWriter::new();
    write_dse(&mut bw, 0, false, 4);
    write_end(&mut bw);
    let payload = bw.finish();

    let mut br = BitReader::new(&payload);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Data {
            element_instance_tag: 0,
            byte_align_flag: false,
            payload_bytes: 4,
        })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn cpe_emits_channel_element_with_tag() {
    let mut bw = BitWriter::new();
    bw.write_u32(IdSynEle::Cpe as u32, 3);
    bw.write_u32(7, 4);
    bw.write_u32(IdSynEle::End as u32, 3);
    bw.align_to_byte_zero();
    let payload = bw.finish();

    let mut br = BitReader::new(&payload);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::ChannelElement {
            kind: IdSynEle::Cpe,
            element_instance_tag: 7,
        })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn lfe_and_cce_recognised() {
    for id in [IdSynEle::Lfe, IdSynEle::Cce] {
        let mut bw = BitWriter::new();
        bw.write_u32(id as u32, 3);
        bw.write_u32(2, 4);
        bw.write_u32(IdSynEle::End as u32, 3);
        bw.align_to_byte_zero();
        let payload = bw.finish();
        let mut br = BitReader::new(&payload);
        let mut walker = Walker::new(&mut br);
        assert_eq!(
            walker.next_element().unwrap(),
            Some(Element::ChannelElement {
                kind: id,
                element_instance_tag: 2,
            })
        );
        assert_eq!(walker.next_element().unwrap(), Some(Element::End));
    }
}

#[test]
fn pce_is_parsed_in_raw_data_block() {
    // Construct a minimal PCE-bearing raw_data_block: PCE element
    // (id=0b101) followed by an END terminator. PCE body uses the
    // simplest legal contents — element_instance_tag=3, object_type=1
    // (LC), sampling_frequency_index=4 (44.1 kHz), every per-channel
    // list empty, no mixdown, no comment field.
    let mut bw = BitWriter::new();
    bw.write_u32(IdSynEle::Pce as u32, 3);
    // PCE body
    bw.write_u32(3, 4); // element_instance_tag
    bw.write_u32(1, 2); // object_type = LC
    bw.write_u32(4, 4); // sampling_frequency_index = 44100
    bw.write_u32(0, 4); // num_front_channel_elements
    bw.write_u32(0, 4); // num_side_channel_elements
    bw.write_u32(0, 4); // num_back_channel_elements
    bw.write_u32(0, 2); // num_lfe_channel_elements
    bw.write_u32(0, 3); // num_assoc_data_elements
    bw.write_u32(0, 4); // num_valid_cc_elements
    bw.write_bit(false); // mono_mixdown_present
    bw.write_bit(false); // stereo_mixdown_present
    bw.write_bit(false); // matrix_mixdown_idx_present
                         // (all per-element loops are empty)
    bw.align_to_byte_zero(); // PCE Note 1 byte_alignment()
    bw.write_u32(0, 8); // comment_field_bytes = 0
    write_end(&mut bw);
    let payload = bw.finish();

    let mut br = BitReader::new(&payload);
    let mut walker = Walker::new(&mut br);
    match walker.next_element().unwrap().unwrap() {
        Element::ProgramConfig(pce) => {
            assert_eq!(pce.element_instance_tag, 3);
            assert_eq!(pce.object_type, 1);
            assert_eq!(pce.sampling_frequency_index, 4);
            assert!(pce.front_elements.is_empty());
            assert!(pce.side_elements.is_empty());
            assert!(pce.back_elements.is_empty());
            assert!(pce.lfe_element_tag_selects.is_empty());
            assert!(pce.comment_field.is_empty());
            assert_eq!(pce.channel_count(), 0);
        }
        other => panic!("expected PCE element, got {:?}", other),
    }
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn id_syn_ele_names_match_spec_table() {
    assert_eq!(IdSynEle::from_bits(0).name(), "SCE");
    assert_eq!(IdSynEle::from_bits(1).name(), "CPE");
    assert_eq!(IdSynEle::from_bits(2).name(), "CCE");
    assert_eq!(IdSynEle::from_bits(3).name(), "LFE");
    assert_eq!(IdSynEle::from_bits(4).name(), "DSE");
    assert_eq!(IdSynEle::from_bits(5).name(), "PCE");
    assert_eq!(IdSynEle::from_bits(6).name(), "FIL");
    assert_eq!(IdSynEle::from_bits(7).name(), "END");
}

#[test]
fn truncated_payload_reports_unexpected_end() {
    // A SCE header (3+4 = 7 bits) requires at least 7 bits of input.
    // Provide just 3 bits worth of buffer (1 byte but only 3 read).
    let mut bw = BitWriter::new();
    bw.write_u32(IdSynEle::Sce as u32, 3);
    // Don't pad: BitWriter will pad anyway, but slice it down.
    let buf = bw.finish();
    let truncated = &buf[..0];
    let mut br = BitReader::new(truncated);
    let mut walker = Walker::new(&mut br);
    assert_eq!(walker.next_element(), Err(Error::UnexpectedEnd));
}

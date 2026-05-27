//! Tests for the [`oxideav_aac::raw_data_block::FrameAssembler`] —
//! round-160 encoder primitive, the bit-exact inverse of the
//! Phase 1 [`oxideav_aac::raw_data_block::Walker`].
//!
//! Every test builds a frame via `FrameAssembler`, feeds the result
//! back through `Walker`, and asserts that the round-trip preserves
//! both element shape and byte position. A handful of hand-pinned
//! wire-layout assertions cover the per-element bit layouts directly.

use oxideav_aac::pce::{CcElementSelect, ElementSelect, Pce};
use oxideav_aac::raw_data_block::{Element, FrameAssembler, IdSynEle, Walker};
use oxideav_aac::Error;
use oxideav_core::bits::BitReader;

// -------------------------------------------------------------------
// END-only frame
// -------------------------------------------------------------------

#[test]
fn end_only_frame_is_one_byte() {
    // END (0b111) + 5 bits of pad-zero == single byte 0b11100000.
    let frame = FrameAssembler::new().push_end();
    assert_eq!(frame, vec![0b1110_0000]);

    // Round-trip through Walker.
    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
    assert!(walker.is_finished());
    assert_eq!(walker.next_element().unwrap(), None);
}

#[test]
fn empty_assembler_bit_position_is_zero() {
    let asm = FrameAssembler::new();
    assert_eq!(asm.bit_position(), 0);
}

#[test]
fn with_capacity_works_like_new() {
    let frame = FrameAssembler::with_capacity(64).push_end();
    assert_eq!(frame, vec![0b1110_0000]);
}

#[test]
fn default_works_like_new() {
    let frame = FrameAssembler::default().push_end();
    assert_eq!(frame, vec![0b1110_0000]);
}

// -------------------------------------------------------------------
// Channel-header element
// -------------------------------------------------------------------

#[test]
fn sce_header_then_end_roundtrips() {
    let mut asm = FrameAssembler::new();
    asm.push_channel_header(IdSynEle::Sce, 0x0a).unwrap();
    let frame = asm.push_end();

    // Wire: id=000, tag=1010, end=111, pad.
    //   bits = 000 1010 111 then pad-zero to byte:
    //   = 0001 0101 11 00 0000 == 0x15 0xC0  ???
    // Let's compute carefully:
    //   bit  0..2  = 0b000 (SCE id)
    //   bit  3..6  = 0b1010 (tag = 0x0a)
    //   bit  7..9  = 0b111 (END id)
    //   bit 10..15 = 0b000000 (pad)
    // = byte0 = bits 0..7  = 0000 1010 = 0x0A
    //   byte1 = bits 8..15 = 1110 0000 = 0xE0
    // wait — bit-7 belongs to byte0 not byte1.
    //   byte0 = bits 0..7 (MSB-first): 0 0 0 1 0 1 0 1 = 0x15
    //   byte1 = bits 8..15:            1 1 0 0 0 0 0 0 = 0xC0
    assert_eq!(frame, vec![0x15, 0xC0]);

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::ChannelElement {
            kind: IdSynEle::Sce,
            element_instance_tag: 0x0a,
        })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn cpe_cce_lfe_headers_all_roundtrip() {
    for (kind, tag) in [
        (IdSynEle::Cpe, 0),
        (IdSynEle::Cce, 7),
        (IdSynEle::Lfe, 0x0f),
    ] {
        let mut asm = FrameAssembler::new();
        asm.push_channel_header(kind, tag).unwrap();
        let frame = asm.push_end();
        let mut br = BitReader::new(&frame);
        let mut walker = Walker::new(&mut br);
        assert_eq!(
            walker.next_element().unwrap(),
            Some(Element::ChannelElement {
                kind,
                element_instance_tag: tag,
            })
        );
        assert_eq!(walker.next_element().unwrap(), Some(Element::End));
    }
}

#[test]
fn push_channel_header_rejects_non_channel_kind() {
    for bad in [IdSynEle::Dse, IdSynEle::Pce, IdSynEle::Fil, IdSynEle::End] {
        let mut asm = FrameAssembler::new();
        let err = asm.push_channel_header(bad, 0).unwrap_err();
        assert_eq!(err, Error::RawDataBlockEncodeInvalid);
    }
}

#[test]
fn push_channel_header_rejects_overflowing_tag() {
    let mut asm = FrameAssembler::new();
    let err = asm.push_channel_header(IdSynEle::Sce, 0x10).unwrap_err();
    assert_eq!(err, Error::RawDataBlockEncodeInvalid);
}

// -------------------------------------------------------------------
// Channel-body bit append
// -------------------------------------------------------------------

#[test]
fn push_channel_body_bits_whole_bytes_then_partial() {
    // Push a SCE header, then a 12-bit body (0xAB_C, MSB-first ⇒
    // bytes [0xAB, 0xC0]), then END.
    let mut asm = FrameAssembler::new();
    asm.push_channel_header(IdSynEle::Sce, 0).unwrap();
    // 12 bits "1010 1011 1100" from byte slice [0xAB, 0xC0].
    asm.push_channel_body_bits(&[0xAB, 0xC0], 12).unwrap();
    let frame = asm.push_end();

    // Manual layout: SCE id (3) + tag (4) + body (12) + END (3) + pad
    //   = 22 bits → 24 bits with pad.
    //   id=000, tag=0000, body=1010 1011 1100, end=111, pad=00.
    //   byte0 = 0 0 0 0 0 0 0 1   bits 0..7  → 0x01
    //   byte1 = 0 1 0 1 0 1 1 1   bits 8..15 → 0x57
    //   byte2 = 1 0 0 1 1 1 0 0   bits 16..23 → 0x9C
    // Let me re-trace:
    //   bit  0..2  = id (SCE)        = 000
    //   bit  3..6  = tag = 0          = 0000
    //   bit  7..18 = body (12 bits)   = 1010 1011 1100
    //   bit 19..21 = end (3 bits)     = 111
    //   bit 22..23 = pad              = 00
    // Pack MSB-first into bytes of 8:
    //   byte0 = bits 0..7  = 0000 0001 = 0x01
    //   byte1 = bits 8..15 = 0101 0111 = 0x57
    //   byte2 = bits 16..23= 1001 1100 = 0x9C
    assert_eq!(frame, vec![0x01, 0x57, 0x9C]);

    // Round-trip the channel header through Walker.
    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::ChannelElement {
            kind: IdSynEle::Sce,
            element_instance_tag: 0,
        })
    );
    // Skip the 12-bit body.
    br.skip(12).unwrap();
    let mut walker = Walker::new(&mut br);
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn push_channel_body_bits_exact_multiple_of_eight() {
    let mut asm = FrameAssembler::new();
    asm.push_channel_header(IdSynEle::Cpe, 1).unwrap();
    asm.push_channel_body_bits(&[0xDE, 0xAD, 0xBE, 0xEF], 32)
        .unwrap();
    let frame = asm.push_end();
    // Roundtrip header + skip 32 bits + END.
    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::ChannelElement {
            kind: IdSynEle::Cpe,
            element_instance_tag: 1,
        })
    );
    br.skip(32).unwrap();
    let mut walker = Walker::new(&mut br);
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn push_channel_body_bits_zero_count_is_noop() {
    let mut asm = FrameAssembler::new();
    asm.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let pos_before = asm.bit_position();
    asm.push_channel_body_bits(&[], 0).unwrap();
    let pos_after = asm.bit_position();
    assert_eq!(pos_before, pos_after);
    let _ = asm.push_end();
}

#[test]
fn push_channel_body_bits_rejects_overflow_count() {
    let mut asm = FrameAssembler::new();
    asm.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let err = asm.push_channel_body_bits(&[0x00], 9).unwrap_err();
    assert_eq!(err, Error::RawDataBlockEncodeInvalid);
}

// -------------------------------------------------------------------
// FIL element
// -------------------------------------------------------------------

#[test]
fn fil_empty_roundtrips() {
    let mut asm = FrameAssembler::new();
    asm.push_fill(&[]).unwrap();
    let frame = asm.push_end();
    // FIL id=110 + count=0000 + END=111 + pad
    //   bits = 1100000111 + 6 pad = 16 bits = 2 bytes
    //   byte0 = 1100 0001 = 0xC1
    //   byte1 = 1100 0000 = 0xC0
    assert_eq!(frame, vec![0xC1, 0xC0]);

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill { payload_bytes: 0 })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn fil_short_payload_roundtrips() {
    for n in 0usize..15 {
        let payload: Vec<u8> = (0..n).map(|i| 0x80 + (i as u8)).collect();
        let mut asm = FrameAssembler::new();
        asm.push_fill(&payload).unwrap();
        let frame = asm.push_end();

        let mut br = BitReader::new(&frame);
        let mut walker = Walker::new(&mut br);
        assert_eq!(
            walker.next_element().unwrap(),
            Some(Element::Fill {
                payload_bytes: n as u32,
            })
        );
        assert_eq!(walker.next_element().unwrap(), Some(Element::End));
    }
}

#[test]
fn fil_at_escape_boundary_15_uses_esc_count_2() {
    // payload_bytes == 15 ⇒ esc_count = 15 - 14 = 1
    let payload = vec![0xAAu8; 15];
    let mut asm = FrameAssembler::new();
    asm.push_fill(&payload).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill { payload_bytes: 15 })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn fil_escape_boundary_payload_16_uses_esc_count_2() {
    // payload_bytes == 16 ⇒ esc_count = 16 - 14 = 2
    let payload = vec![0x5Au8; 16];
    let mut asm = FrameAssembler::new();
    asm.push_fill(&payload).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill { payload_bytes: 16 })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn fil_max_payload_269_roundtrips() {
    // 15 + 255 - 1 = 269 ⇒ esc_count = 255
    let payload: Vec<u8> = (0..269u32).map(|i| i as u8).collect();
    let mut asm = FrameAssembler::new();
    asm.push_fill(&payload).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill { payload_bytes: 269 })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn fil_rejects_payload_above_269() {
    let payload = vec![0u8; 270];
    let mut asm = FrameAssembler::new();
    let err = asm.push_fill(&payload).unwrap_err();
    assert_eq!(err, Error::RawDataBlockEncodeInvalid);
}

// -------------------------------------------------------------------
// DSE element
// -------------------------------------------------------------------

#[test]
fn dse_short_payload_no_align_roundtrips() {
    let payload = vec![0x11, 0x22, 0x33];
    let mut asm = FrameAssembler::new();
    asm.push_data(0x05, false, &payload).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Data {
            element_instance_tag: 0x05,
            byte_align_flag: false,
            payload_bytes: 3,
        })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn dse_with_byte_align_flag_pads_before_payload() {
    // DSE wire (no escape) = id(3) + tag(4) + flag(1) + count(8) = 16 bits.
    // Already on a byte boundary, so byte_align is a no-op for the
    // round-trip path; assert it still consumes correctly.
    let payload = vec![0xDE, 0xAD];
    let mut asm = FrameAssembler::new();
    asm.push_data(0x00, true, &payload).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Data {
            element_instance_tag: 0x00,
            byte_align_flag: true,
            payload_bytes: 2,
        })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn dse_with_byte_align_after_offset_pads_correctly() {
    // Push a SCE header (7 bits) first to force the DSE header to
    // start on a non-byte-aligned boundary; the DSE byte-align flag
    // must then pad before the payload.
    let mut asm = FrameAssembler::new();
    asm.push_channel_header(IdSynEle::Sce, 0).unwrap();
    asm.push_data(0x01, true, &[0xC3]).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::ChannelElement {
            kind: IdSynEle::Sce,
            element_instance_tag: 0,
        })
    );
    let dse = walker.next_element().unwrap().unwrap();
    assert_eq!(
        dse,
        Element::Data {
            element_instance_tag: 0x01,
            byte_align_flag: true,
            payload_bytes: 1,
        }
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn dse_escape_boundary_payload_255_uses_esc_count_0() {
    // payload_bytes == 255 ⇒ esc_count = 0
    let payload = vec![0x77u8; 255];
    let mut asm = FrameAssembler::new();
    asm.push_data(0x02, false, &payload).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Data {
            element_instance_tag: 0x02,
            byte_align_flag: false,
            payload_bytes: 255,
        })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn dse_max_payload_510_roundtrips() {
    // 255 + 255 = 510 ⇒ esc_count = 255
    let payload: Vec<u8> = (0..510u32).map(|i| (i & 0xFF) as u8).collect();
    let mut asm = FrameAssembler::new();
    asm.push_data(0x03, false, &payload).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Data {
            element_instance_tag: 0x03,
            byte_align_flag: false,
            payload_bytes: 510,
        })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn dse_rejects_payload_above_510() {
    let payload = vec![0u8; 511];
    let mut asm = FrameAssembler::new();
    let err = asm.push_data(0, false, &payload).unwrap_err();
    assert_eq!(err, Error::RawDataBlockEncodeInvalid);
}

#[test]
fn dse_rejects_overflowing_tag() {
    let mut asm = FrameAssembler::new();
    let err = asm.push_data(0x10, false, &[]).unwrap_err();
    assert_eq!(err, Error::RawDataBlockEncodeInvalid);
}

// -------------------------------------------------------------------
// Composite frames — multi-element ordering
// -------------------------------------------------------------------

#[test]
fn sce_fil_end_composite_matches_walker_test() {
    // Mirror tests/raw_data_block.rs's
    // `walks_sce_fil_end_in_order_and_stops_cleanly` from the encoder
    // side: build SCE(tag=0)+FIL(empty)+END and walk it back.
    let mut asm = FrameAssembler::new();
    asm.push_channel_header(IdSynEle::Sce, 0).unwrap();
    asm.push_fill(&[]).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::ChannelElement {
            kind: IdSynEle::Sce,
            element_instance_tag: 0,
        })
    );
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill { payload_bytes: 0 })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
    assert_eq!(walker.next_element().unwrap(), None);
}

#[test]
fn cpe_dse_fil_end_composite_roundtrips() {
    let mut asm = FrameAssembler::new();
    asm.push_channel_header(IdSynEle::Cpe, 3).unwrap();
    // Push a 13-bit synthetic CPE body (1-bit common_window=0 + 12
    // pad — just stand-in bytes since no real body encoder yet).
    asm.push_channel_body_bits(&[0x00, 0x00], 13).unwrap();
    asm.push_data(0x0a, false, &[0xCA, 0xFE]).unwrap();
    asm.push_fill(&[0xFF; 5]).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::ChannelElement {
            kind: IdSynEle::Cpe,
            element_instance_tag: 3,
        })
    );
    // Step over the 13-bit body that the walker doesn't parse.
    br.skip(13).unwrap();
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Data {
            element_instance_tag: 0x0a,
            byte_align_flag: false,
            payload_bytes: 2,
        })
    );
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill { payload_bytes: 5 })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn multiple_fill_elements_roundtrip() {
    // Splitting a large fill across two FIL elements is the
    // documented strategy when payload > 269.
    let mut asm = FrameAssembler::new();
    asm.push_fill(&[0xAA; 269]).unwrap();
    asm.push_fill(&[0xBB; 50]).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill { payload_bytes: 269 })
    );
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill { payload_bytes: 50 })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

// -------------------------------------------------------------------
// bit_position tracking
// -------------------------------------------------------------------

#[test]
fn bit_position_grows_with_each_push() {
    let mut asm = FrameAssembler::new();
    assert_eq!(asm.bit_position(), 0);
    asm.push_channel_header(IdSynEle::Sce, 0).unwrap();
    assert_eq!(asm.bit_position(), 7);
    asm.push_fill(&[]).unwrap(); // id(3) + count(4) = 7
    assert_eq!(asm.bit_position(), 14);
    asm.push_fill(&[0xAA]).unwrap(); // id(3) + count(4) + 8 = 15
    assert_eq!(asm.bit_position(), 29);
}

// -------------------------------------------------------------------
// END byte-alignment guarantee
// -------------------------------------------------------------------

#[test]
fn end_byte_aligns_after_unaligned_body() {
    // Force a mid-byte position (3 + 4 = 7 bits) then END, and check
    // the result is exactly two bytes (END(3) + 6 bits pad).
    let mut asm = FrameAssembler::new();
    asm.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let frame = asm.push_end();
    assert_eq!(frame.len(), 2);
    // The last byte's low bits must be zeros (pad).
    let last_byte = *frame.last().unwrap();
    // bit_position so far: 7 (SCE+tag) + 3 (END) = 10 bits ⇒ 6 pad
    // bits zero-filled. Top 2 bits of byte1 are the bottom 2 of END,
    // the next 6 are pad.
    assert_eq!(last_byte & 0b0011_1111, 0);
}

// -------------------------------------------------------------------
// PCE element (round 165) — push_pce + Walker round-trip
// -------------------------------------------------------------------

/// Helper: build a small but non-trivial PCE for use in the
/// `push_pce` round-trip tests.
fn pce_5_1_fixture() -> Pce {
    Pce {
        element_instance_tag: 0x05,
        object_type: 1,              // LC
        sampling_frequency_index: 3, // 48000
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
        back_elements: vec![ElementSelect {
            is_cpe: true,
            tag_select: 1,
        }],
        lfe_element_tag_selects: vec![0],
        assoc_data_tag_selects: vec![],
        valid_cc_elements: vec![],
        mono_mixdown_element_number: None,
        stereo_mixdown_element_number: None,
        matrix_mixdown: None,
        comment_field: b"clean-room".to_vec(),
    }
}

#[test]
fn push_pce_then_end_roundtrips_through_walker() {
    let pce = pce_5_1_fixture();
    let mut asm = FrameAssembler::new();
    asm.push_pce(&pce).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    match walker.next_element().unwrap() {
        Some(Element::ProgramConfig(round)) => assert_eq!(round, pce),
        other => panic!("expected ProgramConfig, got {:?}", other),
    }
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
    assert_eq!(walker.next_element().unwrap(), None);
}

#[test]
fn push_pce_first_byte_starts_with_pce_id_syn_ele() {
    let pce = pce_5_1_fixture();
    let mut asm = FrameAssembler::new();
    asm.push_pce(&pce).unwrap();
    let frame = asm.push_end();
    // PCE id_syn_ele = 0b101 — the top three bits of byte 0 must
    // therefore be `101`.
    assert_eq!(frame[0] >> 5, 0b101, "leading bits = {:03b}", frame[0] >> 5);
}

#[test]
fn push_pce_after_channel_header_roundtrips() {
    // Place an SCE channel header before the PCE so the PCE's byte
    // alignment lands at a non-trivial offset relative to frame
    // start; the walker must still see both elements in order.
    let pce = pce_5_1_fixture();
    let mut asm = FrameAssembler::new();
    asm.push_channel_header(IdSynEle::Sce, 0x0a).unwrap();
    // Empty channel body (the walker doesn't parse the body, so 0
    // bits of body is acceptable for this PCE-position test).
    asm.push_pce(&pce).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::ChannelElement {
            kind: IdSynEle::Sce,
            element_instance_tag: 0x0a,
        })
    );
    match walker.next_element().unwrap() {
        Some(Element::ProgramConfig(round)) => assert_eq!(round, pce),
        other => panic!("expected ProgramConfig, got {:?}", other),
    }
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn push_pce_then_fill_then_end_roundtrips() {
    let pce = pce_5_1_fixture();
    let payload = vec![0xDE, 0xAD, 0xBE, 0xEF];
    let mut asm = FrameAssembler::new();
    asm.push_pce(&pce).unwrap();
    asm.push_fill(&payload).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    match walker.next_element().unwrap() {
        Some(Element::ProgramConfig(round)) => assert_eq!(round, pce),
        other => panic!("expected ProgramConfig, got {:?}", other),
    }
    assert_eq!(
        walker.next_element().unwrap(),
        Some(Element::Fill {
            payload_bytes: payload.len() as u32,
        })
    );
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn push_pce_with_all_mixdowns_and_cc_roundtrips() {
    let pce = Pce {
        element_instance_tag: 0,
        object_type: 1,
        sampling_frequency_index: 4,
        front_elements: vec![ElementSelect {
            is_cpe: true,
            tag_select: 0,
        }],
        side_elements: vec![ElementSelect {
            is_cpe: false,
            tag_select: 1,
        }],
        back_elements: vec![ElementSelect {
            is_cpe: true,
            tag_select: 2,
        }],
        lfe_element_tag_selects: vec![3],
        assoc_data_tag_selects: vec![4, 5],
        valid_cc_elements: vec![
            CcElementSelect {
                is_ind_sw: false,
                tag_select: 6,
            },
            CcElementSelect {
                is_ind_sw: true,
                tag_select: 7,
            },
        ],
        mono_mixdown_element_number: Some(0xa),
        stereo_mixdown_element_number: Some(0xb),
        matrix_mixdown: Some((2, true)),
        comment_field: vec![],
    };
    let mut asm = FrameAssembler::new();
    asm.push_pce(&pce).unwrap();
    let frame = asm.push_end();

    let mut br = BitReader::new(&frame);
    let mut walker = Walker::new(&mut br);
    match walker.next_element().unwrap() {
        Some(Element::ProgramConfig(round)) => assert_eq!(round, pce),
        other => panic!("expected ProgramConfig, got {:?}", other),
    }
    assert_eq!(walker.next_element().unwrap(), Some(Element::End));
}

#[test]
fn push_pce_propagates_encode_invalid_for_overflowing_tag() {
    let mut pce = pce_5_1_fixture();
    pce.element_instance_tag = 0x10; // 4-bit cap is 0x0f
    let mut asm = FrameAssembler::new();
    assert_eq!(asm.push_pce(&pce), Err(Error::PceEncodeInvalid));
}

#[test]
fn push_pce_propagates_encode_invalid_for_oversized_comment() {
    let mut pce = pce_5_1_fixture();
    pce.comment_field = vec![0; 256]; // 8-bit length cap is 255
    let mut asm = FrameAssembler::new();
    assert_eq!(asm.push_pce(&pce), Err(Error::PceEncodeInvalid));
}

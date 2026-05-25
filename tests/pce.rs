//! Tests for [`oxideav_aac::pce::Pce`] — `program_config_element()`
//! synthesised via `oxideav_core::bits::BitWriter`.

use oxideav_aac::pce::{CcElementSelect, ElementSelect, Pce};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// Append a 4-bit `*_element_tag_select` after a 1-bit
/// `*_element_is_cpe` flag.
fn write_element(bw: &mut BitWriter, is_cpe: bool, tag: u8) {
    bw.write_bit(is_cpe);
    bw.write_u32(tag as u32, 4);
}

#[test]
fn parses_empty_pce_with_zero_comment() {
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_u32(1, 2); // object_type = LC
    bw.write_u32(4, 4); // sampling_frequency_index = 44100
    bw.write_u32(0, 4); // n_front
    bw.write_u32(0, 4); // n_side
    bw.write_u32(0, 4); // n_back
    bw.write_u32(0, 2); // n_lfe
    bw.write_u32(0, 3); // n_assoc
    bw.write_u32(0, 4); // n_cc
    bw.write_bit(false); // mono_mixdown_present
    bw.write_bit(false); // stereo_mixdown_present
    bw.write_bit(false); // matrix_mixdown_idx_present
    bw.align_to_byte_zero(); // byte_alignment()
    bw.write_u32(0, 8); // comment_field_bytes
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let pce = Pce::parse(&mut br, 0).unwrap();
    assert_eq!(pce.element_instance_tag, 0);
    assert_eq!(pce.object_type, 1);
    assert_eq!(pce.sampling_frequency_index, 4);
    assert!(pce.front_elements.is_empty());
    assert!(pce.side_elements.is_empty());
    assert!(pce.back_elements.is_empty());
    assert!(pce.lfe_element_tag_selects.is_empty());
    assert!(pce.assoc_data_tag_selects.is_empty());
    assert!(pce.valid_cc_elements.is_empty());
    assert!(pce.mono_mixdown_element_number.is_none());
    assert!(pce.stereo_mixdown_element_number.is_none());
    assert!(pce.matrix_mixdown.is_none());
    assert!(pce.comment_field.is_empty());
    assert_eq!(pce.channel_count(), 0);
}

#[test]
fn parses_5_1_layout_sce_cpe_cpe_lfe() {
    // Standard 5.1 layout as a PCE:
    //   front: SCE (centre) + CPE (L/R)
    //   side: empty
    //   back: CPE (Ls/Rs)
    //   lfe: 1
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4); // tag
    bw.write_u32(1, 2); // LC
    bw.write_u32(3, 4); // sfi = 48000
    bw.write_u32(2, 4); // n_front = 2
    bw.write_u32(0, 4); // n_side
    bw.write_u32(1, 4); // n_back = 1
    bw.write_u32(1, 2); // n_lfe = 1
    bw.write_u32(0, 3); // n_assoc
    bw.write_u32(0, 4); // n_cc
    bw.write_bit(false); // mono_mixdown
    bw.write_bit(false); // stereo_mixdown
    bw.write_bit(false); // matrix_mixdown
    write_element(&mut bw, false, 0); // front[0] = SCE tag=0
    write_element(&mut bw, true, 0); // front[1] = CPE tag=0
    write_element(&mut bw, true, 1); // back[0] = CPE tag=1
    bw.write_u32(0, 4); // lfe[0] tag = 0
    bw.align_to_byte_zero();
    bw.write_u32(0, 8);
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let pce = Pce::parse(&mut br, 0).unwrap();
    assert_eq!(pce.front_elements.len(), 2);
    assert_eq!(
        pce.front_elements[0],
        ElementSelect {
            is_cpe: false,
            tag_select: 0
        }
    );
    assert_eq!(
        pce.front_elements[1],
        ElementSelect {
            is_cpe: true,
            tag_select: 0
        }
    );
    assert_eq!(pce.back_elements.len(), 1);
    assert_eq!(pce.lfe_element_tag_selects, vec![0]);
    // channel count: 1 (SCE) + 2 (CPE) + 2 (CPE) + 1 (LFE) = 6
    assert_eq!(pce.channel_count(), 6);
}

#[test]
fn matrix_mixdown_body_is_parsed() {
    let mut bw = BitWriter::new();
    bw.write_u32(7, 4); // tag
    bw.write_u32(1, 2);
    bw.write_u32(4, 4);
    bw.write_u32(0, 4);
    bw.write_u32(0, 4);
    bw.write_u32(0, 4);
    bw.write_u32(0, 2);
    bw.write_u32(0, 3);
    bw.write_u32(0, 4);
    bw.write_bit(true); // mono_mixdown_present
    bw.write_u32(2, 4); // mono_mixdown_element_number = 2
    bw.write_bit(true); // stereo_mixdown_present
    bw.write_u32(3, 4); // stereo_mixdown_element_number = 3
    bw.write_bit(true); // matrix_mixdown_idx_present
    bw.write_u32(1, 2); // matrix_mixdown_idx = 1
    bw.write_bit(true); // pseudo_surround_enable = true
    bw.align_to_byte_zero();
    bw.write_u32(0, 8);
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let pce = Pce::parse(&mut br, 0).unwrap();
    assert_eq!(pce.element_instance_tag, 7);
    assert_eq!(pce.mono_mixdown_element_number, Some(2));
    assert_eq!(pce.stereo_mixdown_element_number, Some(3));
    assert_eq!(pce.matrix_mixdown, Some((1, true)));
}

#[test]
fn comment_field_round_trips() {
    let comment: &[u8] = b"AAC fixture";
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4);
    bw.write_u32(1, 2);
    bw.write_u32(4, 4);
    bw.write_u32(0, 4);
    bw.write_u32(0, 4);
    bw.write_u32(0, 4);
    bw.write_u32(0, 2);
    bw.write_u32(0, 3);
    bw.write_u32(0, 4);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.align_to_byte_zero();
    bw.write_u32(comment.len() as u32, 8);
    for &b in comment {
        bw.write_u32(b as u32, 8);
    }
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let pce = Pce::parse(&mut br, 0).unwrap();
    assert_eq!(pce.comment_field, comment);
}

#[test]
fn cc_elements_round_trip() {
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4); // tag
    bw.write_u32(1, 2);
    bw.write_u32(4, 4);
    bw.write_u32(0, 4); // n_front
    bw.write_u32(0, 4); // n_side
    bw.write_u32(0, 4); // n_back
    bw.write_u32(0, 2); // n_lfe
    bw.write_u32(0, 3); // n_assoc
    bw.write_u32(2, 4); // n_cc = 2
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    // cc[0]
    bw.write_bit(true); // is_ind_sw
    bw.write_u32(5, 4); // tag = 5
                        // cc[1]
    bw.write_bit(false);
    bw.write_u32(6, 4);
    bw.align_to_byte_zero();
    bw.write_u32(0, 8);
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let pce = Pce::parse(&mut br, 0).unwrap();
    assert_eq!(
        pce.valid_cc_elements,
        vec![
            CcElementSelect {
                is_ind_sw: true,
                tag_select: 5,
            },
            CcElementSelect {
                is_ind_sw: false,
                tag_select: 6,
            },
        ]
    );
    // CCEs are coupling buses — they don't contribute to output channels.
    assert_eq!(pce.channel_count(), 0);
}

#[test]
fn byte_alignment_relative_to_origin() {
    // Place 3 bits of leading padding before the PCE, then mark
    // `origin_bit_offset = 3` so the Table 4.2 byte_alignment()
    // aligns relative to the PCE start (bit 3) rather than the
    // absolute bit position. Without origin honouring, the
    // alignment would be off by 3 bits and the comment-length byte
    // would be misread.
    let mut bw = BitWriter::new();
    bw.write_u32(0b101, 3); // padding bits before ASC-relative origin
                            // PCE body — empty layout, no comment:
    bw.write_u32(0, 4); // tag
    bw.write_u32(1, 2); // object_type
    bw.write_u32(4, 4); // sfi
    bw.write_u32(0, 4); // n_front
    bw.write_u32(0, 4); // n_side
    bw.write_u32(0, 4); // n_back
    bw.write_u32(0, 2); // n_lfe
    bw.write_u32(0, 3); // n_assoc
    bw.write_u32(0, 4); // n_cc
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    // After the above, we're at bit 3 + 4+2+4+4+4+4+2+3+4+3 = 3 + 34 = 37.
    // 37 - origin(3) = 34. Pad 6 bits to reach the next byte-from-origin
    // boundary (bit 40 absolute, bit 37 from origin → pad 0; recompute:
    // (8 - (34 % 8)) % 8 = (8 - 2) % 8 = 6 padding bits needed).
    bw.write_u32(0, 6); // explicit padding to match expected align
    bw.write_u32(0, 8); // comment_field_bytes
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    // Skip the leading 3 bits — the bit-reader now sits at bit 3
    // (the PCE start) but the absolute bit_position is 3, matching
    // origin_bit_offset = 3.
    br.skip(3).unwrap();
    let pce = Pce::parse(&mut br, 3).unwrap();
    assert_eq!(pce.element_instance_tag, 0);
    assert!(pce.comment_field.is_empty());
}

#[test]
fn truncated_pce_returns_unexpected_end() {
    let buf = vec![0u8; 1]; // 8 bits — not enough for even the header
    let mut br = BitReader::new(&buf);
    assert_eq!(Pce::parse(&mut br, 0), Err(Error::UnexpectedEnd));
}

//! Tests for [`oxideav_aac::section_data::SectionData`] — synthetic
//! `section_data()` bit-streams constructed via
//! `oxideav_core::bits::BitWriter`, so the tests pin ISO/IEC 13818-7
//! §6.3 Table 17 (= ISO/IEC 14496-3 §4.4.6) behaviour without any
//! external AAC encoder.

use oxideav_aac::ics_info::WindowSequence;
use oxideav_aac::section_data::{
    Codebook, Section, SectionData, ESC_HCB, FIRST_PAIR_HCB, INTENSITY_HCB, INTENSITY_HCB2,
    NOISE_HCB, ZERO_HCB,
};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// Long-sequence section: `sect_cb` (4 bits) + `sect_len_incr`
/// (5 bits) repeated. Helper writes one (cb, len) pair where `len`
/// is small enough to fit without an escape.
fn write_long_section(bw: &mut BitWriter, cb: u8, len: u8) {
    bw.write_u32(cb as u32, 4);
    bw.write_u32(len as u32, 5);
}

#[test]
fn single_long_section_covers_all_bands() {
    // One section, codebook 4, covering all 5 scalefactor bands.
    let mut bw = BitWriter::new();
    write_long_section(&mut bw, 4, 5);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let sd = SectionData::parse(&mut br, WindowSequence::OnlyLong, 1, 5).unwrap();
    assert_eq!(sd.num_sec(0), 1);
    assert_eq!(
        sd.sections[0][0],
        Section {
            codebook: 4,
            start: 0,
            end: 5,
        }
    );
    assert_eq!(sd.sfb_cb[0], vec![4, 4, 4, 4, 4]);
    // 4 + 5 = 9 bits consumed.
    assert_eq!(br.bit_position(), 9);
}

#[test]
fn two_long_sections_concatenate() {
    // Section 0: cb=11 (ESC) bands 0..3; section 1: cb=2 bands 3..7.
    let mut bw = BitWriter::new();
    write_long_section(&mut bw, 11, 3);
    write_long_section(&mut bw, 2, 4);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let sd = SectionData::parse(&mut br, WindowSequence::OnlyLong, 1, 7).unwrap();
    assert_eq!(sd.num_sec(0), 2);
    assert_eq!(sd.sections[0][0].start, 0);
    assert_eq!(sd.sections[0][0].end, 3);
    assert_eq!(sd.sections[0][0].codebook, 11);
    assert_eq!(sd.sections[0][1].start, 3);
    assert_eq!(sd.sections[0][1].end, 7);
    assert_eq!(sd.sections[0][1].codebook, 2);
    assert_eq!(sd.sfb_cb[0], vec![11, 11, 11, 2, 2, 2, 2]);
}

#[test]
fn long_section_escape_coding_extends_length() {
    // Long sect_esc_val = 31. A section of 35 bands needs one
    // escape (31) + a 4 increment: sect_len = 31 + 4 = 35.
    let mut bw = BitWriter::new();
    bw.write_u32(6, 4); // sect_cb
    bw.write_u32(31, 5); // sect_len_incr == sect_esc_val (escape)
    bw.write_u32(4, 5); // final increment
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let sd = SectionData::parse(&mut br, WindowSequence::OnlyLong, 1, 35).unwrap();
    assert_eq!(sd.num_sec(0), 1);
    assert_eq!(sd.sections[0][0].len(), 35);
    assert_eq!(sd.sections[0][0].end, 35);
    assert!(sd.sfb_cb[0].iter().all(|&cb| cb == 6));
    // 4 + 5 + 5 = 14 bits consumed.
    assert_eq!(br.bit_position(), 14);
}

#[test]
fn long_section_double_escape() {
    // 31 + 31 + 1 = 63 bands → two escapes then a final 1.
    let mut bw = BitWriter::new();
    bw.write_u32(1, 4);
    bw.write_u32(31, 5);
    bw.write_u32(31, 5);
    bw.write_u32(1, 5);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let sd = SectionData::parse(&mut br, WindowSequence::OnlyLong, 1, 63).unwrap();
    assert_eq!(sd.sections[0][0].len(), 63);
    assert_eq!(br.bit_position(), 4 + 5 + 5 + 5);
}

#[test]
fn eight_short_uses_three_bit_increment_and_esc_7() {
    // EIGHT_SHORT: sect_esc_val = 7, sect_len_incr is 3 bits.
    // One group of 8 windows (after grouping), max_sfb=10.
    // Section of 10 bands needs an escape (7) + 3: 7 + 3 = 10.
    let mut bw = BitWriter::new();
    bw.write_u32(5, 4); // sect_cb
    bw.write_u32(7, 3); // escape (== sect_esc_val)
    bw.write_u32(3, 3); // final increment
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let sd = SectionData::parse(&mut br, WindowSequence::EightShort, 1, 10).unwrap();
    assert_eq!(sd.sections[0][0].len(), 10);
    assert_eq!(sd.sections[0][0].codebook, 5);
    // 4 + 3 + 3 = 10 bits consumed.
    assert_eq!(br.bit_position(), 10);
}

#[test]
fn eight_short_short_section_no_escape() {
    // A 3-band section fits in a single 3-bit increment (< 7).
    let mut bw = BitWriter::new();
    bw.write_u32(2, 4);
    bw.write_u32(3, 3);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let sd = SectionData::parse(&mut br, WindowSequence::EightShort, 1, 3).unwrap();
    assert_eq!(sd.sections[0][0].len(), 3);
    assert_eq!(br.bit_position(), 7);
}

#[test]
fn multiple_window_groups_parsed_independently() {
    // Two window groups, each with its own section list, max_sfb=4.
    let mut bw = BitWriter::new();
    // Group 0: one section cb=3, len 4 (3-bit incr in short branch).
    bw.write_u32(3, 4);
    bw.write_u32(4, 3);
    // Group 1: two sections cb=7 len 1 then cb=0 len 3.
    bw.write_u32(7, 4);
    bw.write_u32(1, 3);
    bw.write_u32(0, 4);
    bw.write_u32(3, 3);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let sd = SectionData::parse(&mut br, WindowSequence::EightShort, 2, 4).unwrap();
    assert_eq!(sd.sections.len(), 2);
    assert_eq!(sd.num_sec(0), 1);
    assert_eq!(sd.num_sec(1), 2);
    assert_eq!(sd.sfb_cb[0], vec![3, 3, 3, 3]);
    assert_eq!(sd.sfb_cb[1], vec![7, 0, 0, 0]);
}

#[test]
fn zero_max_sfb_yields_no_sections() {
    // max_sfb == 0 → the while loop never runs; num_sec[g] == 0.
    let mut bw = BitWriter::new();
    bw.write_u32(0, 1); // arbitrary padding so finish() has bytes
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let sd = SectionData::parse(&mut br, WindowSequence::OnlyLong, 1, 0).unwrap();
    assert_eq!(sd.num_sec(0), 0);
    assert!(sd.sections[0].is_empty());
    assert!(sd.sfb_cb[0].is_empty());
    // No bits consumed by the (empty) loop.
    assert_eq!(br.bit_position(), 0);
}

#[test]
fn overrun_section_length_is_rejected() {
    // max_sfb=3 but the section claims 5 bands → overrun error.
    let mut bw = BitWriter::new();
    bw.write_u32(4, 4);
    bw.write_u32(5, 5); // sect_len = 5 > max_sfb=3
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let res = SectionData::parse(&mut br, WindowSequence::OnlyLong, 1, 3);
    assert_eq!(res, Err(Error::SectionDataOverrun));
}

#[test]
fn unexpected_end_surfaces_error() {
    // Provide 4 bits (sect_cb) but no room for the 5-bit increment.
    let mut bw = BitWriter::new();
    bw.write_u32(3, 4);
    let buf = bw.finish();
    // finish() byte-pads to 1 byte (8 bits): 4 used, 4 pad-zero.
    // A 5-bit read after the 4-bit cb succeeds (reads pad bits) so
    // truncate to exactly the 4-bit prefix to force underflow.
    let truncated: Vec<u8> = Vec::new();
    let mut br = BitReader::new(&truncated);
    let res = SectionData::parse(&mut br, WindowSequence::OnlyLong, 1, 5);
    assert_eq!(res, Err(Error::UnexpectedEnd));
    let _ = buf;
}

// ---- Codebook classification (ISO/IEC 13818-7 Table 59 + MPEG-4 PNS) ----

#[test]
fn codebook_constants_match_iso13818_7_section_9_2_2() {
    assert_eq!(ZERO_HCB, 0);
    assert_eq!(FIRST_PAIR_HCB, 5);
    assert_eq!(ESC_HCB, 11);
    assert_eq!(NOISE_HCB, 13);
    assert_eq!(INTENSITY_HCB2, 14);
    assert_eq!(INTENSITY_HCB, 15);
}

#[test]
fn codebook_classification_zero() {
    let cb = Codebook::from_value(0);
    assert_eq!(cb, Codebook::Zero);
    assert!(cb.is_zero());
    assert!(!cb.is_intensity());
    assert!(!cb.is_noise());
}

#[test]
fn codebook_quad_signed_unsigned_matches_table_59() {
    // Table 59 unsigned_cb[]: book 1,2 signed; 3,4 unsigned.
    assert_eq!(
        Codebook::from_value(1),
        Codebook::Quad {
            number: 1,
            unsigned: false
        }
    );
    assert_eq!(
        Codebook::from_value(2),
        Codebook::Quad {
            number: 2,
            unsigned: false
        }
    );
    assert_eq!(
        Codebook::from_value(3),
        Codebook::Quad {
            number: 3,
            unsigned: true
        }
    );
    assert_eq!(
        Codebook::from_value(4),
        Codebook::Quad {
            number: 4,
            unsigned: true
        }
    );
}

#[test]
fn codebook_pair_signed_unsigned_matches_table_59() {
    // Table 59 unsigned_cb[]: book 5,6 signed; 7,8,9,10 unsigned.
    assert_eq!(
        Codebook::from_value(5),
        Codebook::Pair {
            number: 5,
            unsigned: false
        }
    );
    assert_eq!(
        Codebook::from_value(6),
        Codebook::Pair {
            number: 6,
            unsigned: false
        }
    );
    for n in 7..=10u8 {
        assert_eq!(
            Codebook::from_value(n),
            Codebook::Pair {
                number: n,
                unsigned: true
            }
        );
    }
}

#[test]
fn codebook_esc_reserved_noise_intensity() {
    assert_eq!(Codebook::from_value(11), Codebook::Esc);
    assert_eq!(Codebook::from_value(12), Codebook::Reserved12);
    let noise = Codebook::from_value(13);
    assert_eq!(noise, Codebook::Noise);
    assert!(noise.is_noise());
    let out = Codebook::from_value(14);
    assert_eq!(out, Codebook::IntensityOutOfPhase);
    assert!(out.is_intensity());
    let inp = Codebook::from_value(15);
    assert_eq!(inp, Codebook::IntensityInPhase);
    assert!(inp.is_intensity());
}

#[test]
fn section_helpers_len_and_kind() {
    let s = Section {
        codebook: 15,
        start: 10,
        end: 14,
    };
    assert_eq!(s.len(), 4);
    assert!(!s.is_empty());
    assert_eq!(s.codebook_kind(), Codebook::IntensityInPhase);
    assert!(s.codebook_kind().is_intensity());

    let empty = Section {
        codebook: 0,
        start: 3,
        end: 3,
    };
    assert!(empty.is_empty());
    assert_eq!(empty.len(), 0);
}

#[test]
fn fixtures_doc_section_trace_shape() {
    // The fixtures-doc reference trace (4.1) shows the leading
    // sections of a 44.1 kHz LC CPE:
    //   SECTION group=0 codebook=11 sect_end=7
    //   SECTION group=0 codebook=10 sect_end=22
    //   SECTION group=0 codebook=6  sect_end=26
    // Reconstruct a section_data() carrying exactly those three runs
    // (long branch, 5-bit increments; lengths 7, 15, 4) and confirm
    // the parser reproduces the sect_end column.
    let mut bw = BitWriter::new();
    write_long_section(&mut bw, 11, 7); // bands 0..7
    write_long_section(&mut bw, 10, 15); // bands 7..22
    write_long_section(&mut bw, 6, 4); // bands 22..26
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);

    let sd = SectionData::parse(&mut br, WindowSequence::OnlyLong, 1, 26).unwrap();
    assert_eq!(sd.num_sec(0), 3);
    assert_eq!(sd.sections[0][0].end, 7);
    assert_eq!(sd.sections[0][1].end, 22);
    assert_eq!(sd.sections[0][2].end, 26);
    assert_eq!(sd.sections[0][0].codebook, 11);
    assert_eq!(sd.sections[0][1].codebook, 10);
    assert_eq!(sd.sections[0][2].codebook, 6);
}

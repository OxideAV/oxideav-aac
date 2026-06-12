//! Self-roundtrip tests for [`oxideav_aac::section_data::SectionData::write`]
//! — the AAC crate's first encoder primitive (ISO/IEC 14496-3
//! §4.4.6 / ISO/IEC 13818-7 §6.3 Table 17).
//!
//! Each test builds a [`SectionData`] in memory, runs
//! [`SectionData::write`] into a fresh [`BitWriter`], then feeds the
//! resulting byte buffer through [`SectionData::parse`] and asserts
//! bit-exact recovery (same sections, same `sfb_cb` map, same byte
//! buffer down to the bit position the parser consumed).
//!
//! Pure encoder ↔ parser symmetry: the only inputs are the spec-defined wire field
//! widths (4-bit `sect_cb` + 3- or 5-bit `sect_len_incr`) and the
//! §6.3 escape mechanism (`sect_esc_val == 7` or `31`).

use oxideav_aac::ics_info::WindowSequence;
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// Helper: build a `SectionData` from a list of (codebook, length)
/// runs per group and the corresponding `max_sfb`.
fn make_section_data(groups: &[&[(u8, u8)]], max_sfb: u8) -> SectionData {
    let mut sections = Vec::with_capacity(groups.len());
    let mut sfb_cb = Vec::with_capacity(groups.len());
    for group in groups {
        let mut k: u8 = 0;
        let mut group_sections = Vec::with_capacity(group.len());
        let mut group_sfb_cb = vec![0u8; max_sfb as usize];
        for &(cb, len) in *group {
            let start = k;
            let end = k + len;
            for sfb in start..end {
                group_sfb_cb[sfb as usize] = cb;
            }
            group_sections.push(Section {
                codebook: cb,
                start,
                end,
            });
            k = end;
        }
        sections.push(group_sections);
        sfb_cb.push(group_sfb_cb);
    }
    SectionData { sections, sfb_cb }
}

/// Run a write → parse → equality cycle and return the encoded byte
/// length and bit-position the parser consumed.
fn roundtrip(sd: &SectionData, ws: WindowSequence, num_groups: u8, max_sfb: u8) -> (Vec<u8>, u64) {
    let mut bw = BitWriter::new();
    sd.write(&mut bw, ws, max_sfb).expect("encode succeeds");
    let bits_written = bw.bit_position();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = SectionData::parse(&mut br, ws, num_groups, max_sfb)
        .expect("parse of self-encoded bitstream succeeds");
    assert_eq!(&parsed, sd, "round-trip changes structure");
    assert_eq!(
        br.bit_position(),
        bits_written,
        "parser consumed a different number of bits than the writer emitted"
    );
    (buf, bits_written)
}

// ---- Long branch (5-bit sect_len_incr, sect_esc_val == 31) ----

#[test]
fn long_single_section_no_escape_roundtrips() {
    // 5-band run, codebook 4; sect_len < 31 → one 5-bit increment.
    let sd = make_section_data(&[&[(4, 5)]], 5);
    let (_, bits) = roundtrip(&sd, WindowSequence::OnlyLong, 1, 5);
    // 4 (sect_cb) + 5 (sect_len_incr=5) = 9 bits.
    assert_eq!(bits, 9);
}

#[test]
fn long_multiple_sections_no_escape_roundtrips() {
    // Three sections covering bands 0..7, 7..22, 22..26 — the
    // fixtures-doc reference shape from the parser tests.
    let sd = make_section_data(&[&[(11, 7), (10, 15), (6, 4)]], 26);
    let (_, bits) = roundtrip(&sd, WindowSequence::OnlyLong, 1, 26);
    // 3 × (4 + 5) = 27 bits.
    assert_eq!(bits, 27);
}

#[test]
fn long_single_escape_roundtrips() {
    // 35 bands → emit 31 (escape) + 4 (final).
    let sd = make_section_data(&[&[(6, 35)]], 35);
    let (_, bits) = roundtrip(&sd, WindowSequence::OnlyLong, 1, 35);
    // 4 (sect_cb) + 5 (esc) + 5 (4) = 14 bits.
    assert_eq!(bits, 14);
}

#[test]
fn long_double_escape_roundtrips() {
    // 63 bands → emit 31, 31, 1.
    let sd = make_section_data(&[&[(1, 63)]], 63);
    let (_, bits) = roundtrip(&sd, WindowSequence::OnlyLong, 1, 63);
    // 4 + 5 + 5 + 5 = 19 bits.
    assert_eq!(bits, 19);
}

#[test]
fn long_sect_len_exact_multiple_of_esc_emits_trailing_zero() {
    // sect_len == 31 must be encoded as 31 (escape) + 0 (terminator),
    // NOT a single 31 — otherwise the parser's `while incr ==
    // sect_esc_val` loop would not break and would try to read past
    // the end of the section.
    let sd = make_section_data(&[&[(2, 31)]], 31);
    let (buf, bits) = roundtrip(&sd, WindowSequence::OnlyLong, 1, 31);
    // 4 (sect_cb=2) + 5 (esc=31) + 5 (terminator=0) = 14 bits.
    assert_eq!(bits, 14);

    // Spot-check the literal bit layout: 0010 11111 00000 = 14 bits,
    // padded with two zero bits across two bytes:
    //   byte 0 = 0010_1111 = 0x2F
    //   byte 1 = 1000_0000 = 0x80 (top bit is the 14th, rest is pad).
    assert_eq!(buf.len(), 2);
    assert_eq!(buf[0], 0b0010_1111);
    assert_eq!(buf[1], 0b1000_0000);
}

#[test]
fn long_sect_len_62_double_exact_multiple_roundtrips() {
    // 62 = 31 + 31 → escape, escape, then terminator 0.
    let sd = make_section_data(&[&[(7, 62)]], 62);
    let (_, bits) = roundtrip(&sd, WindowSequence::OnlyLong, 1, 62);
    assert_eq!(bits, 4 + 5 + 5 + 5);
}

// ---- EIGHT_SHORT branch (3-bit sect_len_incr, sect_esc_val == 7) ----

#[test]
fn short_single_section_no_escape_roundtrips() {
    let sd = make_section_data(&[&[(3, 4)]], 4);
    let (_, bits) = roundtrip(&sd, WindowSequence::EightShort, 1, 4);
    // 4 (sect_cb) + 3 (sect_len_incr=4) = 7 bits.
    assert_eq!(bits, 7);
}

#[test]
fn short_single_escape_roundtrips() {
    // 10 bands → emit 7 (escape) + 3.
    let sd = make_section_data(&[&[(5, 10)]], 10);
    let (_, bits) = roundtrip(&sd, WindowSequence::EightShort, 1, 10);
    // 4 + 3 + 3 = 10 bits.
    assert_eq!(bits, 10);
}

#[test]
fn short_sect_len_exact_multiple_of_esc_emits_trailing_zero() {
    // sect_len == 7 → 7 (escape) + 0 (terminator).
    let sd = make_section_data(&[&[(2, 7)]], 7);
    let (_, bits) = roundtrip(&sd, WindowSequence::EightShort, 1, 7);
    // 4 + 3 + 3 = 10 bits.
    assert_eq!(bits, 10);
}

// ---- Multi-window-group ----

#[test]
fn multi_window_group_short_roundtrips() {
    // Group 0: one 4-band section (cb=3). Group 1: two sections
    // (cb=7 len=1, cb=0 len=3).
    let sd = make_section_data(&[&[(3, 4)], &[(7, 1), (0, 3)]], 4);
    let (_, bits) = roundtrip(&sd, WindowSequence::EightShort, 2, 4);
    // g0: 4 + 3 = 7; g1: (4 + 3) + (4 + 3) = 14; total = 21 bits.
    assert_eq!(bits, 21);
}

#[test]
fn multi_group_long_with_intensity_and_noise_roundtrips() {
    // Group 0: pair codebook spans, then intensity stereo + PNS.
    // Realistic-shape sect_cb sequence for a 24-band CPE.
    let g0: &[(u8, u8)] = &[
        (11, 8), // ESC_HCB
        (7, 8),  // pair book
        (15, 4), // intensity in-phase
        (13, 4), // noise (PNS)
    ];
    let g1: &[(u8, u8)] = &[
        (11, 4),
        (10, 8),
        (14, 4), // intensity out-of-phase
        (0, 8),  // ZERO_HCB
    ];
    let sd = make_section_data(&[g0, g1], 24);
    roundtrip(&sd, WindowSequence::OnlyLong, 2, 24);
}

// ---- Zero / empty edge cases ----

#[test]
fn empty_section_list_when_max_sfb_zero_roundtrips() {
    // Spec: when max_sfb == 0 the `while k < max_sfb` loop never
    // runs, so num_sec[g] == 0. Encoder mirrors that — emit nothing.
    let sd = SectionData {
        sections: vec![vec![]],
        sfb_cb: vec![vec![]],
    };
    let mut bw = BitWriter::new();
    sd.write(&mut bw, WindowSequence::OnlyLong, 0).unwrap();
    assert_eq!(bw.bit_position(), 0);
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = SectionData::parse(&mut br, WindowSequence::OnlyLong, 1, 0).unwrap();
    assert_eq!(parsed, sd);
    assert_eq!(br.bit_position(), 0);
}

// ---- Encoder input validation ----

#[test]
fn write_rejects_non_contiguous_start() {
    // First section starts at 2, not 0.
    let sd = SectionData {
        sections: vec![vec![Section {
            codebook: 4,
            start: 2,
            end: 5,
        }]],
        sfb_cb: vec![vec![0, 0, 4, 4, 4]],
    };
    let mut bw = BitWriter::new();
    let res = sd.write(&mut bw, WindowSequence::OnlyLong, 5);
    assert_eq!(res, Err(Error::SectionDataEncodeInvalid));
}

#[test]
fn write_rejects_short_of_max_sfb() {
    let sd = SectionData {
        sections: vec![vec![Section {
            codebook: 4,
            start: 0,
            end: 3,
        }]],
        sfb_cb: vec![vec![4, 4, 4, 0, 0]],
    };
    let mut bw = BitWriter::new();
    let res = sd.write(&mut bw, WindowSequence::OnlyLong, 5);
    assert_eq!(res, Err(Error::SectionDataEncodeInvalid));
}

#[test]
fn write_rejects_gap_between_sections() {
    let sd = SectionData {
        sections: vec![vec![
            Section {
                codebook: 1,
                start: 0,
                end: 2,
            },
            Section {
                codebook: 2,
                start: 3, // gap: previous ended at 2
                end: 5,
            },
        ]],
        sfb_cb: vec![vec![1, 1, 0, 2, 2]],
    };
    let mut bw = BitWriter::new();
    let res = sd.write(&mut bw, WindowSequence::OnlyLong, 5);
    assert_eq!(res, Err(Error::SectionDataEncodeInvalid));
}

#[test]
fn write_rejects_zero_length_section() {
    let sd = SectionData {
        sections: vec![vec![
            Section {
                codebook: 1,
                start: 0,
                end: 0, // empty
            },
            Section {
                codebook: 2,
                start: 0,
                end: 5,
            },
        ]],
        sfb_cb: vec![vec![2, 2, 2, 2, 2]],
    };
    let mut bw = BitWriter::new();
    let res = sd.write(&mut bw, WindowSequence::OnlyLong, 5);
    assert_eq!(res, Err(Error::SectionDataEncodeInvalid));
}

#[test]
fn write_rejects_oversized_codebook_value() {
    let sd = SectionData {
        sections: vec![vec![Section {
            codebook: 0x10, // > 4-bit field
            start: 0,
            end: 3,
        }]],
        sfb_cb: vec![vec![0x10, 0x10, 0x10]],
    };
    let mut bw = BitWriter::new();
    let res = sd.write(&mut bw, WindowSequence::OnlyLong, 3);
    assert_eq!(res, Err(Error::SectionDataEncodeInvalid));
}

// ---- Bit-exact pin against a hand-computed wire layout ----

#[test]
fn long_two_section_wire_layout_pin() {
    // Reproduce the existing parser test
    // `two_long_sections_concatenate` from the encode side: sections
    // cb=11 len=3 and cb=2 len=4 over 7 bands. The wire layout is:
    //   1011 00011  0010 00100
    //   ↑cb=11 ↑l=3 ↑cb=2 ↑l=4 = 18 bits → pad to 3 bytes.
    let sd = make_section_data(&[&[(11, 3), (2, 4)]], 7);
    let mut bw = BitWriter::new();
    sd.write(&mut bw, WindowSequence::OnlyLong, 7).unwrap();
    assert_eq!(bw.bit_position(), 4 + 5 + 4 + 5);
    let buf = bw.finish();
    assert_eq!(buf.len(), 3);
    // 1011_0001 1001_0001 00xx_xxxx  (xx = byte-pad zeros)
    assert_eq!(buf[0], 0b1011_0001);
    assert_eq!(buf[1], 0b1001_0001);
    assert_eq!(buf[2], 0b0000_0000);

    // Parser-roundtrip check.
    let mut br = BitReader::new(&buf);
    let parsed = SectionData::parse(&mut br, WindowSequence::OnlyLong, 1, 7).unwrap();
    assert_eq!(parsed, sd);
    assert_eq!(br.bit_position(), 18);
}

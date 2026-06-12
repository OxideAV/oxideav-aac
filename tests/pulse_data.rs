//! Self-roundtrip tests for [`oxideav_aac::pulse_data`] —
//! ISO/IEC 14496-3 §4.4.6.3 / Table 4.7 `pulse_data()` parser
//! **and** encoder primitive (the AAC crate's third encode-side
//! syntax-element writer).
//!
//! Every test builds a [`PulseData`] in memory, runs [`PulseData::write`]
//! into a fresh [`BitWriter`], then feeds the resulting byte buffer
//! through [`PulseData::parse`] and asserts:
//!
//! 1. The parsed structure equals the original (bit-exact structural
//!    recovery).
//! 2. The parser's terminal bit-position matches the writer's
//!    bit-position (no over- or under-read).
//!
//! The only inputs are the
//! spec-defined wire field widths (2 + 6 + (5+4) × (1..=4) bits) and
//! the `number_pulse + 1` loop bound semantics from Table 4.7.

use oxideav_aac::pulse_data::{Pulse, PulseData, MAX_PULSES, PULSE_AMP_BITS, PULSE_OFFSET_BITS};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// Convenience: build a `PulseData` from a `(start_sfb, pulses)`
/// tuple. The pulses arg is a slice of `(offset, amp)` pairs in wire
/// order.
fn make_pulse_data(start_sfb: u8, pulses: &[(u8, u8)]) -> PulseData {
    PulseData {
        pulse_start_sfb: start_sfb,
        pulses: pulses
            .iter()
            .copied()
            .map(|(offset, amp)| Pulse { offset, amp })
            .collect(),
    }
}

/// Run a write → parse → equality cycle and return the encoded byte
/// length and the writer's terminal bit-position.
fn roundtrip(pd: &PulseData) -> (Vec<u8>, u64) {
    let mut bw = BitWriter::new();
    pd.write(&mut bw).expect("encode succeeds");
    let bits_written = bw.bit_position();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = PulseData::parse(&mut br).expect("parse of self-encoded bitstream succeeds");
    assert_eq!(&parsed, pd, "round-trip changes structure");
    assert_eq!(
        br.bit_position(),
        bits_written,
        "parser consumed a different number of bits than the writer emitted"
    );
    (buf, bits_written)
}

// ---- Single-pulse smallest legal block ----

#[test]
fn single_pulse_smallest_roundtrips() {
    // number_pulse = 0, pulse_start_sfb = 0, one pulse (offset=0, amp=0).
    let pd = make_pulse_data(0, &[(0, 0)]);
    let (_, bits) = roundtrip(&pd);
    // 2 (number_pulse) + 6 (start_sfb) + 5 (offset) + 4 (amp) = 17 bits.
    assert_eq!(bits, 17);
}

#[test]
fn single_pulse_with_nonzero_fields_roundtrips() {
    let pd = make_pulse_data(5, &[(7, 3)]);
    let (_, bits) = roundtrip(&pd);
    assert_eq!(bits, 17);
}

// ---- Multi-pulse blocks (every legal pulse count 2..=MAX_PULSES) ----

#[test]
fn two_pulse_roundtrips() {
    let pd = make_pulse_data(12, &[(4, 2), (11, 5)]);
    let (_, bits) = roundtrip(&pd);
    // 2 + 6 + 2 × (5 + 4) = 26 bits.
    assert_eq!(bits, 26);
}

#[test]
fn three_pulse_roundtrips() {
    let pd = make_pulse_data(33, &[(1, 1), (2, 2), (3, 3)]);
    let (_, bits) = roundtrip(&pd);
    // 2 + 6 + 3 × 9 = 35 bits.
    assert_eq!(bits, 35);
}

#[test]
fn four_pulse_max_roundtrips() {
    // MAX_PULSES == 4, achieved by number_pulse == 3 on the wire.
    let pd = make_pulse_data(15, &[(0, 0), (10, 5), (20, 8), (31, 15)]);
    let (_, bits) = roundtrip(&pd);
    // 2 + 6 + 4 × 9 = 44 bits.
    assert_eq!(bits, 44);
    assert_eq!(pd.number_pulse(), 3);
    assert_eq!(pd.pulses.len(), MAX_PULSES);
}

// ---- Wire-field boundary values ----

#[test]
fn pulse_start_sfb_at_max_roundtrips() {
    // pulse_start_sfb = 0x3f (top of the 6-bit field).
    let pd = make_pulse_data(0x3f, &[(0, 0)]);
    roundtrip(&pd);
}

#[test]
fn pulse_offset_at_max_roundtrips() {
    // pulse_offset = 0x1f (top of the 5-bit field).
    let pd = make_pulse_data(7, &[(0x1f, 0)]);
    roundtrip(&pd);
}

#[test]
fn pulse_amp_at_max_roundtrips() {
    // pulse_amp = 0x0f (top of the 4-bit field).
    let pd = make_pulse_data(7, &[(0, 0x0f)]);
    roundtrip(&pd);
}

#[test]
fn all_fields_at_max_roundtrips() {
    let pd = make_pulse_data(
        0x3f,
        &[(0x1f, 0x0f), (0x1f, 0x0f), (0x1f, 0x0f), (0x1f, 0x0f)],
    );
    let (buf, bits) = roundtrip(&pd);
    assert_eq!(bits, 44);
    // Expected literal layout (44 bits, MSB-first into bytes):
    //   number_pulse=11  start_sfb=111111  [offset=11111 amp=1111]×4
    //   = 11 111111 (11111 1111)×4
    //   = 11_111111_11111_1111_11111_1111_11111_1111_11111_1111
    //   Byte 0: 1111_1111 = 0xFF (bits 0..7: nn ssssss = 11_111111)
    //   Byte 1: 1111_1111 = 0xFF (bits 8..15: 11111_111 = offset[0] + amp[0] top 3 bits)
    //   Byte 2: 1111_1111 = 0xFF (bits 16..23: 1_11111_11 = amp[0] low + offset[1] + amp[1] top 2)
    //   Byte 3: 1111_1111 = 0xFF
    //   Byte 4: 1111_1111 = 0xFF
    //   Byte 5: 1111_0000 = 0xF0 (44 bits used → 4 trailing pad zeros)
    assert_eq!(buf.len(), 6);
    for b in &buf[..5] {
        assert_eq!(*b, 0xFF);
    }
    assert_eq!(buf[5], 0xF0);
}

// ---- Hand-pinned wire layout ----

#[test]
fn single_pulse_wire_layout_pin() {
    // number_pulse = 0 (1 pulse), pulse_start_sfb = 0x2a = 0b101010,
    // pulse_offset[0] = 0x15 = 0b10101, pulse_amp[0] = 0x05 = 0b0101.
    //
    // Wire: 00 101010 10101 0101 = 17 bits.
    //   byte 0 = 00_101010 = 0x2A
    //   byte 1 = 10101_010 = 0xAA (top 5 = offset, bottom 3 = amp high bits 3..1)
    //   byte 2 = 1_0000000 = 0x80 (top 1 = amp bit 0, remainder pad)
    let pd = make_pulse_data(0x2a, &[(0x15, 0x05)]);
    let mut bw = BitWriter::new();
    pd.write(&mut bw).unwrap();
    assert_eq!(bw.bit_position(), 17);
    let buf = bw.finish();
    assert_eq!(buf.len(), 3);
    assert_eq!(buf[0], 0x2A);
    assert_eq!(buf[1], 0xAA);
    assert_eq!(buf[2], 0x80);

    let mut br = BitReader::new(&buf);
    let parsed = PulseData::parse(&mut br).unwrap();
    assert_eq!(parsed, pd);
    assert_eq!(br.bit_position(), 17);
}

#[test]
fn realistic_two_pulse_wire_layout_pin() {
    // A semi-realistic pulse_data() for a 44.1 kHz LC stream:
    // pulse_start_sfb = 32 (mid-band), two pulses at offsets 3 and
    // 17 with amplitudes 2 and 4.
    //
    // Wire layout (26 bits):
    //   01 100000 00011 0010 10001 0100
    //   ^number_pulse=1
    //      ^start_sfb=32 = 0b100000
    //              ^offset[0]=3 = 0b00011
    //                    ^amp[0]=2 = 0b0010
    //                         ^offset[1]=17 = 0b10001
    //                               ^amp[1]=4 = 0b0100
    //   Concatenated bits: 0110_0000_0001_1001_0100_0101_00
    //   Byte 0: 0110_0000 = 0x60
    //   Byte 1: 0001_1001 = 0x19
    //   Byte 2: 0100_0101 = 0x45
    //   Byte 3: 0000_0000 = 0x00 (top 2 bits = trailing "00" of amp[1],
    //                            bottom 6 = pad zeros)
    let pd = make_pulse_data(32, &[(3, 2), (17, 4)]);
    let mut bw = BitWriter::new();
    pd.write(&mut bw).unwrap();
    assert_eq!(bw.bit_position(), 26);
    let buf = bw.finish();
    assert_eq!(buf.len(), 4);
    assert_eq!(buf[0], 0x60);
    assert_eq!(buf[1], 0x19);
    assert_eq!(buf[2], 0x45);
    assert_eq!(buf[3], 0x00);

    let mut br = BitReader::new(&buf);
    let parsed = PulseData::parse(&mut br).unwrap();
    assert_eq!(parsed, pd);
    assert_eq!(br.bit_position(), 26);
}

// ---- Accessor + constant sanity ----

#[test]
fn number_pulse_accessor_matches_wire_value() {
    for n in 1..=MAX_PULSES {
        let pulses: Vec<(u8, u8)> = (0..n).map(|i| (i as u8, i as u8)).collect();
        let pd = make_pulse_data(0, &pulses);
        assert_eq!(pd.number_pulse() as usize, n - 1);
    }
}

#[test]
fn wire_field_width_constants_match_spec() {
    // Table 4.7: pulse_offset is 5 bits, pulse_amp is 4 bits,
    // number_pulse is 2 bits → MAX_PULSES = 4.
    assert_eq!(PULSE_OFFSET_BITS, 5);
    assert_eq!(PULSE_AMP_BITS, 4);
    assert_eq!(MAX_PULSES, 4);
}

// ---- Parser failure modes ----

#[test]
fn parse_unexpected_end_in_header() {
    // Only 7 bits provided (need 8 minimum for number_pulse +
    // pulse_start_sfb).
    let buf = [0b1010_1010];
    let mut br = BitReader::new(&buf[..]);
    // Burn 1 bit so only 7 remain.
    br.read_u32(1).unwrap();
    assert_eq!(PulseData::parse(&mut br), Err(Error::UnexpectedEnd));
}

#[test]
fn parse_unexpected_end_in_pulse_loop() {
    // 8 bits is enough for the header (2 + 6) but not for any pulse
    // (needs 9 more bits per pulse).
    let buf = [0x00];
    let mut br = BitReader::new(&buf[..]);
    assert_eq!(PulseData::parse(&mut br), Err(Error::UnexpectedEnd));
}

// ---- Encoder input validation ----

#[test]
fn write_rejects_empty_pulses() {
    let pd = PulseData {
        pulse_start_sfb: 0,
        pulses: vec![],
    };
    let mut bw = BitWriter::new();
    assert_eq!(pd.write(&mut bw), Err(Error::PulseDataEncodeInvalid));
}

#[test]
fn write_rejects_too_many_pulses() {
    // 5 pulses — exceeds the 2-bit number_pulse cap (max 4).
    let pd = make_pulse_data(0, &[(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]);
    let mut bw = BitWriter::new();
    assert_eq!(pd.write(&mut bw), Err(Error::PulseDataEncodeInvalid));
}

#[test]
fn write_rejects_oversized_pulse_start_sfb() {
    let pd = make_pulse_data(0x40, &[(0, 0)]); // 0x40 > 0x3f
    let mut bw = BitWriter::new();
    assert_eq!(pd.write(&mut bw), Err(Error::PulseDataEncodeInvalid));
}

#[test]
fn write_rejects_oversized_pulse_offset() {
    let pd = make_pulse_data(0, &[(0x20, 0)]); // 0x20 > 0x1f
    let mut bw = BitWriter::new();
    assert_eq!(pd.write(&mut bw), Err(Error::PulseDataEncodeInvalid));
}

#[test]
fn write_rejects_oversized_pulse_amp() {
    let pd = make_pulse_data(0, &[(0, 0x10)]); // 0x10 > 0x0f
    let mut bw = BitWriter::new();
    assert_eq!(pd.write(&mut bw), Err(Error::PulseDataEncodeInvalid));
}

#[test]
fn write_rejects_oversized_field_in_middle_pulse() {
    // First and last pulses are fine; middle has a bad amp.
    let pd = make_pulse_data(0, &[(0, 0), (5, 0x10), (10, 5)]);
    let mut bw = BitWriter::new();
    assert_eq!(pd.write(&mut bw), Err(Error::PulseDataEncodeInvalid));
}

// ---- Multi-instance interleaving (no shared state) ----

#[test]
fn back_to_back_pulse_blocks_roundtrip() {
    // Encode two PulseData blocks into the same buffer with no
    // separator; parse them back in order. Validates that the writer
    // and parser don't leak any inter-block state.
    let pd1 = make_pulse_data(5, &[(1, 1), (2, 2)]);
    let pd2 = make_pulse_data(40, &[(31, 15), (0, 0), (15, 7), (1, 2)]);

    let mut bw = BitWriter::new();
    pd1.write(&mut bw).unwrap();
    let bits_after_first = bw.bit_position();
    pd2.write(&mut bw).unwrap();
    let total_bits = bw.bit_position();
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let parsed1 = PulseData::parse(&mut br).unwrap();
    assert_eq!(br.bit_position(), bits_after_first);
    assert_eq!(parsed1, pd1);

    let parsed2 = PulseData::parse(&mut br).unwrap();
    assert_eq!(br.bit_position(), total_bits);
    assert_eq!(parsed2, pd2);
}

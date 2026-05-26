//! Self-roundtrip tests for [`oxideav_aac::tns_data`] — ISO/IEC
//! 14496-3 §4.4.6 / Table 4.54 `tns_data()` parser **and** encoder
//! primitive (the AAC crate's fourth encode-side syntax-element
//! writer).
//!
//! Every test builds a [`TnsData`] in memory, runs [`TnsData::write`]
//! into a fresh [`BitWriter`], then feeds the resulting byte buffer
//! through [`TnsData::parse`] and asserts:
//!
//! 1. The parsed structure equals the original (bit-exact structural
//!    recovery, including per-window `coef_res`).
//! 2. The parser's terminal bit-position matches the writer's
//!    bit-position (no over- or under-read).
//!
//! No external reference encoder is invoked; the only inputs are the
//! spec-defined wire field widths from Table 4.54 + Table 4.155 + the
//! §4.6.9.3 `coef_bits = (3 + coef_res) - coef_compress` rule.
//!
//! For zero-filter windows the parser does **not** consume a
//! `coef_res` bit, so the post-roundtrip structure stores
//! `coef_res = false` regardless of what the caller supplied — the
//! tests pin `coef_res` to `false` in that case to keep the
//! equality check meaningful.

use oxideav_aac::ics_info::WindowSequence;
use oxideav_aac::tns_data::{
    coef_bits, field_widths, num_windows, TnsData, TnsFilter, TnsWindow, COEF_COMPRESS_BITS,
    COEF_RES_BITS, DIRECTION_BITS, LENGTH_BITS_LONG, LENGTH_BITS_SHORT, N_FILT_BITS_LONG,
    N_FILT_BITS_SHORT, ORDER_BITS_LONG, ORDER_BITS_SHORT,
};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

// ----------------------------------------------------------------------------
// Builders
// ----------------------------------------------------------------------------

fn empty_long_window() -> TnsWindow {
    TnsWindow {
        coef_res: false,
        filters: Vec::new(),
    }
}

fn empty_long() -> TnsData {
    TnsData {
        windows: vec![empty_long_window()],
    }
}

fn empty_short() -> TnsData {
    TnsData {
        windows: (0..8).map(|_| empty_long_window()).collect(),
    }
}

fn filter(length: u8, order: u8, direction: bool, coef_compress: bool, coef: &[u8]) -> TnsFilter {
    TnsFilter {
        length,
        order,
        direction,
        coef_compress,
        coef: coef.to_vec(),
    }
}

fn zero_order_filter(length: u8) -> TnsFilter {
    TnsFilter {
        length,
        order: 0,
        direction: false,
        coef_compress: false,
        coef: Vec::new(),
    }
}

// ----------------------------------------------------------------------------
// Roundtrip helper
// ----------------------------------------------------------------------------

fn roundtrip(td: &TnsData, seq: WindowSequence) -> (Vec<u8>, u64) {
    let mut bw = BitWriter::new();
    td.write(&mut bw, seq).expect("encode succeeds");
    let bits_written = bw.bit_position();
    let buf = bw.finish();
    let mut br = BitReader::new(&buf);
    let parsed = TnsData::parse(&mut br, seq).expect("parse of self-encoded bitstream succeeds");
    assert_eq!(&parsed, td, "round-trip changes structure");
    assert_eq!(
        br.bit_position(),
        bits_written,
        "parser consumed a different number of bits than the writer emitted"
    );
    (buf, bits_written)
}

// ----------------------------------------------------------------------------
// Empty / no-filter cases
// ----------------------------------------------------------------------------

#[test]
fn empty_long_roundtrips() {
    let td = empty_long();
    let (_, bits) = roundtrip(&td, WindowSequence::OnlyLong);
    // 1 window × 2-bit n_filt = 2 bits.
    assert_eq!(bits, 2);
}

#[test]
fn empty_long_start_roundtrips() {
    let td = empty_long();
    let (_, bits) = roundtrip(&td, WindowSequence::LongStart);
    assert_eq!(bits, 2);
}

#[test]
fn empty_long_stop_roundtrips() {
    let td = empty_long();
    let (_, bits) = roundtrip(&td, WindowSequence::LongStop);
    assert_eq!(bits, 2);
}

#[test]
fn empty_short_roundtrips() {
    let td = empty_short();
    let (_, bits) = roundtrip(&td, WindowSequence::EightShort);
    // 8 windows × 1-bit n_filt = 8 bits.
    assert_eq!(bits, 8);
}

// ----------------------------------------------------------------------------
// Single-filter long-window cases (every `coef_res` × `coef_compress`)
// ----------------------------------------------------------------------------

#[test]
fn single_long_filter_coef_res0_compress0_roundtrips() {
    // coef_res=0, compress=0 → coef_bits = 3 - 0 = 3, max coef = 7.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![filter(10, 4, false, false, &[1, 2, 3, 4])],
        }],
    };
    let (_, bits) = roundtrip(&td, WindowSequence::OnlyLong);
    // n_filt(2) + coef_res(1) + length(6) + order(5) + dir(1) + cc(1)
    //   + 4 × coef_bits(3) = 2 + 1 + 6 + 5 + 1 + 1 + 12 = 28 bits.
    assert_eq!(bits, 28);
}

#[test]
fn single_long_filter_coef_res0_compress1_roundtrips() {
    // coef_res=0, compress=1 → coef_bits = 3 - 1 = 2, max coef = 3.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![filter(20, 3, true, true, &[0, 1, 3])],
        }],
    };
    let (_, bits) = roundtrip(&td, WindowSequence::OnlyLong);
    // 2 + 1 + 6 + 5 + 1 + 1 + 3*2 = 22.
    assert_eq!(bits, 22);
}

#[test]
fn single_long_filter_coef_res1_compress0_roundtrips() {
    // coef_res=1, compress=0 → coef_bits = 4 - 0 = 4, max coef = 15.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: true,
            filters: vec![filter(
                40,
                12,
                false,
                false,
                &[15, 0, 7, 8, 1, 2, 3, 14, 13, 9, 6, 10],
            )],
        }],
    };
    let (_, bits) = roundtrip(&td, WindowSequence::OnlyLong);
    // 2 + 1 + 6 + 5 + 1 + 1 + 12*4 = 64.
    assert_eq!(bits, 64);
}

#[test]
fn single_long_filter_coef_res1_compress1_roundtrips() {
    // coef_res=1, compress=1 → coef_bits = 4 - 1 = 3, max coef = 7.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: true,
            filters: vec![filter(
                63,
                20,
                true,
                true,
                &[7, 7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 7, 0, 7],
            )],
        }],
    };
    let (_, bits) = roundtrip(&td, WindowSequence::OnlyLong);
    // 2 + 1 + 6 + 5 + 1 + 1 + 20*3 = 76.
    assert_eq!(bits, 76);
}

// ----------------------------------------------------------------------------
// Long-window order=0 corner case (no coef payload)
// ----------------------------------------------------------------------------

#[test]
fn long_filter_order_zero_skips_coef_payload() {
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: true,
            filters: vec![zero_order_filter(7)],
        }],
    };
    let (_, bits) = roundtrip(&td, WindowSequence::OnlyLong);
    // n_filt(2) + coef_res(1) + length(6) + order(5) = 14 bits;
    // no direction / compress / coef on the wire.
    assert_eq!(bits, 14);
}

#[test]
fn long_filter_mix_zero_and_nonzero_order_roundtrips() {
    // Three filters; only the middle one carries a coef payload.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![
                zero_order_filter(5),
                filter(11, 6, false, true, &[0, 1, 2, 3, 0, 1]),
                zero_order_filter(15),
            ],
        }],
    };
    let (_, bits) = roundtrip(&td, WindowSequence::OnlyLong);
    // n_filt(2) + coef_res(1)
    // + 3 × (length(6) + order(5)) = 33
    // + middle filter dir(1) + cc(1) + 6 × coef_bits(2) = 14
    // total = 2 + 1 + 33 + 14 = 50.
    assert_eq!(bits, 50);
}

// ----------------------------------------------------------------------------
// Multi-filter long-window cases (every legal n_filt count 0..=3)
// ----------------------------------------------------------------------------

#[test]
fn three_filters_long_roundtrips() {
    // n_filt = 3 (max for 2-bit n_filt field).
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: true,
            filters: vec![
                filter(8, 2, false, false, &[1, 2]),
                filter(12, 5, true, false, &[7, 8, 9, 10, 11]),
                filter(16, 3, false, true, &[1, 2, 3]),
            ],
        }],
    };
    let (_, bits) = roundtrip(&td, WindowSequence::OnlyLong);
    // n_filt(2) + coef_res(1)
    // f0: length(6) + order(5) + dir(1) + cc(1) + 2×4 = 21
    // f1: 6 + 5 + 1 + 1 + 5×4 = 33
    // f2: 6 + 5 + 1 + 1 + 3×3 = 22
    // total = 2 + 1 + 21 + 33 + 22 = 79.
    assert_eq!(bits, 79);
}

// ----------------------------------------------------------------------------
// EIGHT_SHORT cases
// ----------------------------------------------------------------------------

#[test]
fn short_one_filter_per_window_roundtrips() {
    // One zero-order filter on each of 8 windows.
    let mut td = empty_short();
    for w in td.windows.iter_mut() {
        w.coef_res = false;
        w.filters = vec![zero_order_filter(2)];
    }
    let (_, bits) = roundtrip(&td, WindowSequence::EightShort);
    // per window: n_filt(1) + coef_res(1) + length(4) + order(3) = 9
    // × 8 windows = 72.
    assert_eq!(bits, 72);
}

#[test]
fn short_max_field_widths_roundtrip() {
    // Eight windows, each carrying one filter with max-permitted
    // short-window widths: length=15 (4 bits), order=7 (3 bits),
    // 7 × 4-bit coef.
    let mut td = empty_short();
    for w in td.windows.iter_mut() {
        w.coef_res = true;
        w.filters = vec![filter(15, 7, true, false, &[15, 14, 13, 12, 11, 10, 9])];
    }
    let (_, bits) = roundtrip(&td, WindowSequence::EightShort);
    // per window: 1 + 1 + 4 + 3 + 1 + 1 + 7*4 = 39
    // × 8 = 312.
    assert_eq!(bits, 312);
}

#[test]
fn short_heterogeneous_per_window_roundtrips() {
    // Mix: some windows filterless, some single-filter; vary
    // coef_res, coef_compress, direction, and order across windows.
    let mut td = empty_short();
    td.windows[0] = TnsWindow {
        coef_res: false,
        filters: vec![],
    };
    td.windows[1] = TnsWindow {
        coef_res: false,
        filters: vec![zero_order_filter(3)],
    };
    td.windows[2] = TnsWindow {
        coef_res: false,
        filters: vec![filter(4, 2, false, false, &[1, 2])],
    };
    td.windows[3] = TnsWindow {
        coef_res: true,
        filters: vec![filter(7, 5, true, true, &[1, 2, 3, 4, 5])],
    };
    td.windows[4] = TnsWindow {
        coef_res: true,
        filters: vec![filter(15, 7, false, false, &[15, 0, 7, 8, 1, 14, 9])],
    };
    // 5, 6, 7 stay empty.
    let (_, _) = roundtrip(&td, WindowSequence::EightShort);
}

// ----------------------------------------------------------------------------
// Hand-pinned wire-layout assertion
// ----------------------------------------------------------------------------

#[test]
fn long_minimal_single_filter_wire_layout_pin() {
    // OnlyLong, one window, one filter with order=1:
    //   n_filt = 1                  → 2 bits = 0b01
    //   coef_res = 0                → 1 bit  = 0b0
    //   length = 0b001011 (=11)     → 6 bits
    //   order = 0b00001 (=1)        → 5 bits
    //   direction = 1               → 1 bit
    //   coef_compress = 0           → 1 bit
    //   coef[0] = 0b101 (=5)        → 3 bits (coef_bits = 3-0 = 3)
    //
    // Bit stream (MSB-first, 19 bits total):
    //   01 0 001011 00001 1 0 101
    //
    // Byte 0 (bits 0..7):  0_1_0_0_0_1_0_1 = 0b01000101 = 0x45
    // Byte 1 (bits 8..15): 1_0_0_0_0_1_1_0 = 0b10000110 = 0x86
    // Byte 2 (bits 16..23): 1_0_1_pad pad pad pad pad
    //                       = 0b10100000 = 0xA0
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![filter(11, 1, true, false, &[5])],
        }],
    };
    let mut bw = BitWriter::new();
    td.write(&mut bw, WindowSequence::OnlyLong).unwrap();
    assert_eq!(bw.bit_position(), 19);
    let buf = bw.finish();
    assert_eq!(buf.len(), 3);
    assert_eq!(buf[0], 0x45);
    assert_eq!(buf[1], 0x86);
    assert_eq!(buf[2], 0xA0);

    let mut br = BitReader::new(&buf);
    let parsed = TnsData::parse(&mut br, WindowSequence::OnlyLong).unwrap();
    assert_eq!(parsed, td);
    assert_eq!(br.bit_position(), 19);
}

#[test]
fn long_zero_filter_wire_layout_pin() {
    // OnlyLong, one window, zero filters:
    //   n_filt = 0 → 2 bits = 0b00, no coef_res.
    // Wire = 2 bits, byte 0 = 0b00_000000 = 0x00.
    let td = empty_long();
    let mut bw = BitWriter::new();
    td.write(&mut bw, WindowSequence::OnlyLong).unwrap();
    assert_eq!(bw.bit_position(), 2);
    let buf = bw.finish();
    assert_eq!(buf.len(), 1);
    assert_eq!(buf[0], 0x00);
}

#[test]
fn short_zero_filter_all_windows_wire_layout_pin() {
    // EightShort, eight windows, each with n_filt=0:
    //   8 × 0b0 = 0b00000000 = 0x00, exactly one byte.
    let td = empty_short();
    let mut bw = BitWriter::new();
    td.write(&mut bw, WindowSequence::EightShort).unwrap();
    assert_eq!(bw.bit_position(), 8);
    let buf = bw.finish();
    assert_eq!(buf.len(), 1);
    assert_eq!(buf[0], 0x00);
}

// ----------------------------------------------------------------------------
// Field-width / helper sanity
// ----------------------------------------------------------------------------

#[test]
fn field_width_constants_match_table_4_155() {
    assert_eq!(N_FILT_BITS_SHORT, 1);
    assert_eq!(N_FILT_BITS_LONG, 2);
    assert_eq!(LENGTH_BITS_SHORT, 4);
    assert_eq!(LENGTH_BITS_LONG, 6);
    assert_eq!(ORDER_BITS_SHORT, 3);
    assert_eq!(ORDER_BITS_LONG, 5);
    assert_eq!(COEF_RES_BITS, 1);
    assert_eq!(DIRECTION_BITS, 1);
    assert_eq!(COEF_COMPRESS_BITS, 1);
}

#[test]
fn field_widths_dispatch_on_window_sequence() {
    assert_eq!(
        field_widths(WindowSequence::OnlyLong),
        (N_FILT_BITS_LONG, LENGTH_BITS_LONG, ORDER_BITS_LONG)
    );
    assert_eq!(
        field_widths(WindowSequence::LongStart),
        (N_FILT_BITS_LONG, LENGTH_BITS_LONG, ORDER_BITS_LONG)
    );
    assert_eq!(
        field_widths(WindowSequence::LongStop),
        (N_FILT_BITS_LONG, LENGTH_BITS_LONG, ORDER_BITS_LONG)
    );
    assert_eq!(
        field_widths(WindowSequence::EightShort),
        (N_FILT_BITS_SHORT, LENGTH_BITS_SHORT, ORDER_BITS_SHORT)
    );
}

#[test]
fn num_windows_matches_4_5_2_3_4() {
    assert_eq!(num_windows(WindowSequence::OnlyLong), 1);
    assert_eq!(num_windows(WindowSequence::LongStart), 1);
    assert_eq!(num_windows(WindowSequence::LongStop), 1);
    assert_eq!(num_windows(WindowSequence::EightShort), 8);
}

#[test]
fn coef_bits_dispatch_matches_4_6_9_3() {
    // (coef_res, coef_compress) → coef_bits
    assert_eq!(coef_bits(false, false), 3); // res=3, compress=0
    assert_eq!(coef_bits(false, true), 2); // res=3, compress=1
    assert_eq!(coef_bits(true, false), 4); // res=4, compress=0
    assert_eq!(coef_bits(true, true), 3); // res=4, compress=1
}

// ----------------------------------------------------------------------------
// Parser failure modes
// ----------------------------------------------------------------------------

#[test]
fn parse_unexpected_end_at_n_filt() {
    // Empty buffer; can't read the first n_filt (2 bits for long).
    let buf: [u8; 0] = [];
    let mut br = BitReader::new(&buf[..]);
    assert_eq!(
        TnsData::parse(&mut br, WindowSequence::OnlyLong),
        Err(Error::UnexpectedEnd)
    );
}

#[test]
fn parse_unexpected_end_inside_filter() {
    // Header bits say n_filt=1 + coef_res=0, but no length/order
    // bits follow.
    let buf = [0b0100_0000_u8];
    let mut br = BitReader::new(&buf[..]);
    assert_eq!(
        TnsData::parse(&mut br, WindowSequence::OnlyLong),
        Err(Error::UnexpectedEnd)
    );
}

// ----------------------------------------------------------------------------
// Encoder validation branches
// ----------------------------------------------------------------------------

#[test]
fn write_rejects_wrong_window_count_for_long() {
    // OnlyLong demands exactly 1 window.
    let td = empty_short(); // 8 windows
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::OnlyLong),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_wrong_window_count_for_short() {
    let td = empty_long(); // 1 window
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::EightShort),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_too_many_filters_long() {
    // 4 filters → 2-bit n_filt field overflows (max=3 for long).
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![zero_order_filter(1); 4],
        }],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::OnlyLong),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_too_many_filters_short() {
    // 2 filters → 1-bit n_filt field overflows (max=1 for short).
    let mut td = empty_short();
    td.windows[0].filters = vec![zero_order_filter(1); 2];
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::EightShort),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_length_overflow_long() {
    // length=64 exceeds the 6-bit length field (max=63) on long windows.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![zero_order_filter(64)],
        }],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::OnlyLong),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_length_overflow_short() {
    // length=16 exceeds the 4-bit length field (max=15) on short windows.
    let mut td = empty_short();
    td.windows[0].filters = vec![zero_order_filter(16)];
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::EightShort),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_order_overflow_long() {
    // order=32 exceeds the 5-bit order field (max=31) on long windows.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![filter(1, 32, false, false, &[0; 32])],
        }],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::OnlyLong),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_order_overflow_short() {
    // order=8 exceeds the 3-bit order field (max=7) on short windows.
    let mut td = empty_short();
    td.windows[0].filters = vec![filter(1, 8, false, false, &[0; 8])];
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::EightShort),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_coef_len_mismatch() {
    // order=3 but coef has 4 entries.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![filter(1, 3, false, false, &[0, 1, 2, 3])],
        }],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::OnlyLong),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_coef_value_overflow_compressed() {
    // coef_res=1, compress=1 → coef_bits=3, max coef=7. Value 8 is invalid.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: true,
            filters: vec![filter(1, 3, false, true, &[0, 8, 0])],
        }],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::OnlyLong),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_coef_value_overflow_uncompressed_3bit() {
    // coef_res=0, compress=0 → coef_bits=3, max coef=7. Value 8 is invalid.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![filter(1, 1, false, false, &[8])],
        }],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::OnlyLong),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_coef_value_overflow_uncompressed_4bit() {
    // coef_res=1, compress=0 → coef_bits=4, max coef=15. Value 16 is invalid.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: true,
            filters: vec![filter(1, 1, false, false, &[16])],
        }],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::OnlyLong),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_zero_order_with_direction_set() {
    // order=0 but direction=true → would silently drop on wire.
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                length: 1,
                order: 0,
                direction: true,
                coef_compress: false,
                coef: Vec::new(),
            }],
        }],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::OnlyLong),
        Err(Error::TnsDataEncodeInvalid)
    );
}

#[test]
fn write_rejects_zero_order_with_compress_set() {
    let td = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![TnsFilter {
                length: 1,
                order: 0,
                direction: false,
                coef_compress: true,
                coef: Vec::new(),
            }],
        }],
    };
    let mut bw = BitWriter::new();
    assert_eq!(
        td.write(&mut bw, WindowSequence::OnlyLong),
        Err(Error::TnsDataEncodeInvalid)
    );
}

// ----------------------------------------------------------------------------
// Back-to-back interleaving (no shared state)
// ----------------------------------------------------------------------------

#[test]
fn back_to_back_tns_blocks_roundtrip() {
    // Two TnsData blocks (same window_sequence) into one buffer with
    // no separator; parse them back in order. Validates that the
    // writer / parser don't carry inter-block state.
    let td1 = TnsData {
        windows: vec![TnsWindow {
            coef_res: false,
            filters: vec![filter(7, 2, true, false, &[1, 2])],
        }],
    };
    let td2 = TnsData {
        windows: vec![TnsWindow {
            coef_res: true,
            filters: vec![
                zero_order_filter(3),
                filter(11, 4, false, true, &[1, 2, 3, 4]),
            ],
        }],
    };

    let mut bw = BitWriter::new();
    td1.write(&mut bw, WindowSequence::OnlyLong).unwrap();
    let bits_after_first = bw.bit_position();
    td2.write(&mut bw, WindowSequence::OnlyLong).unwrap();
    let total_bits = bw.bit_position();
    let buf = bw.finish();

    let mut br = BitReader::new(&buf);
    let parsed1 = TnsData::parse(&mut br, WindowSequence::OnlyLong).unwrap();
    assert_eq!(br.bit_position(), bits_after_first);
    assert_eq!(parsed1, td1);

    let parsed2 = TnsData::parse(&mut br, WindowSequence::OnlyLong).unwrap();
    assert_eq!(br.bit_position(), total_bits);
    assert_eq!(parsed2, td2);
}

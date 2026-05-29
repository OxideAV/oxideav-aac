//! Integration tests for `gain_control_data()` — ISO/IEC 14496-3
//! §4.4.6.5 / Table 4.12.
//!
//! The fixtures hand-roll a few Table 4.12 layouts (one per
//! `window_sequence`) and assert that `parse` decodes them as the
//! expected in-memory structure, and that `write` re-emits the same
//! bytes. A handful of caller-side overflow cases on the writer
//! exercise the [`Error::GainControlDataEncodeInvalid`] surface.

use oxideav_aac::gain_control_data::{
    aloccode_bits, num_windows, GainAdjust, GainBand, GainControlData, GainWindow, ADJUST_NUM_BITS,
    ALEVCODE_BITS, MAX_BAND_BITS,
};
use oxideav_aac::ics_info::WindowSequence;
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

/// `num_windows` table per Table 4.12.
#[test]
fn num_windows_matches_table_4_12() {
    assert_eq!(num_windows(WindowSequence::OnlyLong), 1);
    assert_eq!(num_windows(WindowSequence::LongStart), 2);
    assert_eq!(num_windows(WindowSequence::EightShort), 8);
    assert_eq!(num_windows(WindowSequence::LongStop), 2);
}

/// `aloccode_bits` table per Table 4.12.
#[test]
fn aloccode_bits_matches_table_4_12() {
    // ONLY_LONG: 5 bits at wd=0.
    assert_eq!(aloccode_bits(WindowSequence::OnlyLong, 0), 5);

    // LONG_START: 4 at wd=0, 2 at wd=1.
    assert_eq!(aloccode_bits(WindowSequence::LongStart, 0), 4);
    assert_eq!(aloccode_bits(WindowSequence::LongStart, 1), 2);

    // EIGHT_SHORT: 2 at every wd in 0..8.
    for wd in 0..8 {
        assert_eq!(aloccode_bits(WindowSequence::EightShort, wd), 2);
    }
    // Out-of-range wd collapses to 0.
    assert_eq!(aloccode_bits(WindowSequence::EightShort, 8), 0);

    // LONG_STOP: 4 at wd=0, 5 at wd=1.
    assert_eq!(aloccode_bits(WindowSequence::LongStop, 0), 4);
    assert_eq!(aloccode_bits(WindowSequence::LongStop, 1), 5);
}

/// Field-width constants pin Table 4.12.
#[test]
fn field_width_constants() {
    assert_eq!(MAX_BAND_BITS, 2);
    assert_eq!(ADJUST_NUM_BITS, 3);
    assert_eq!(ALEVCODE_BITS, 4);
}

/// `max_band == 0` collapses to a bare 2-bit zero.
#[test]
fn empty_gain_control_data_only_long() {
    let bytes = [0b00_000000u8];
    let mut reader = BitReader::new(&bytes);
    let parsed = GainControlData::parse(&mut reader, WindowSequence::OnlyLong).expect("parse");
    assert_eq!(parsed.max_band, 0);
    assert!(parsed.bands.is_empty());

    // Round-trip.
    let mut writer = BitWriter::new();
    parsed
        .write(&mut writer, WindowSequence::OnlyLong)
        .expect("write");
    let out = writer.finish();
    // The 2-bit field is `00` at the MSB; the rest of the byte is
    // zero-pad from the writer flush.
    assert_eq!(out.len(), 1);
    assert_eq!(out[0] & 0b1100_0000, 0);
}

/// `OnlyLong` with `max_band = 1`, single adjustment.
#[test]
fn only_long_single_band_single_adjust_roundtrip() {
    let gcd = GainControlData {
        max_band: 1,
        bands: vec![GainBand {
            windows: vec![GainWindow {
                adjustments: vec![GainAdjust {
                    alevcode: 0b1010,
                    aloccode: 0b10101,
                }],
            }],
        }],
    };

    let mut writer = BitWriter::new();
    gcd.write(&mut writer, WindowSequence::OnlyLong)
        .expect("write");
    let out = writer.finish();

    // Expected bit stream:
    //   max_band       = 01           (2 bits)
    //   adjust_num     = 001          (3 bits, value 1)
    //   alevcode       = 1010         (4 bits)
    //   aloccode       = 10101        (5 bits)
    // Total = 14 bits → flushed to 16 bits = 2 bytes:
    //   01 001 1010 10101 00 = 0100_1101 0101_0100
    assert_eq!(out, vec![0b0100_1101, 0b0101_0100]);

    let mut reader = BitReader::new(&out);
    let parsed = GainControlData::parse(&mut reader, WindowSequence::OnlyLong).expect("parse");
    assert_eq!(parsed, gcd);
}

/// `LongStart` with `max_band = 2`, mixed adjustment counts; exercises
/// the 4-vs-2 split for `aloccode` widths.
#[test]
fn long_start_two_bands_mixed_widths_roundtrip() {
    let gcd = GainControlData {
        max_band: 2,
        bands: vec![
            // bd = 1
            GainBand {
                windows: vec![
                    // wd = 0: aloccode 4 bits
                    GainWindow {
                        adjustments: vec![GainAdjust {
                            alevcode: 0b0001,
                            aloccode: 0b1100,
                        }],
                    },
                    // wd = 1: aloccode 2 bits
                    GainWindow {
                        adjustments: vec![GainAdjust {
                            alevcode: 0b0010,
                            aloccode: 0b11,
                        }],
                    },
                ],
            },
            // bd = 2 — both windows empty
            GainBand {
                windows: vec![
                    GainWindow {
                        adjustments: vec![],
                    },
                    GainWindow {
                        adjustments: vec![],
                    },
                ],
            },
        ],
    };

    let mut writer = BitWriter::new();
    gcd.write(&mut writer, WindowSequence::LongStart)
        .expect("write");
    let out = writer.finish();

    let mut reader = BitReader::new(&out);
    let parsed = GainControlData::parse(&mut reader, WindowSequence::LongStart).expect("parse");
    assert_eq!(parsed, gcd);
}

/// `EightShort` exercises the 8-window loop; every `aloccode` is 2 bits.
#[test]
fn eight_short_all_windows_max_adjustments_roundtrip() {
    let mut windows = Vec::with_capacity(8);
    for wd in 0..8u8 {
        // alternate adjustment counts 0/1/2/3 across wd.
        let n = (wd as usize) % 4;
        let adjustments: Vec<GainAdjust> = (0..n)
            .map(|i| GainAdjust {
                alevcode: (i as u8) & 0x0f,
                aloccode: wd & 0b11,
            })
            .collect();
        windows.push(GainWindow { adjustments });
    }
    let gcd = GainControlData {
        max_band: 1,
        bands: vec![GainBand { windows }],
    };

    let mut writer = BitWriter::new();
    gcd.write(&mut writer, WindowSequence::EightShort)
        .expect("write");
    let out = writer.finish();

    let mut reader = BitReader::new(&out);
    let parsed = GainControlData::parse(&mut reader, WindowSequence::EightShort).expect("parse");
    assert_eq!(parsed, gcd);
}

/// `LongStop` exercises the 4-then-5 split for `aloccode` widths.
#[test]
fn long_stop_aloccode_5_bits_at_wd1_roundtrip() {
    let gcd = GainControlData {
        max_band: 1,
        bands: vec![GainBand {
            windows: vec![
                // wd = 0 — aloccode 4 bits
                GainWindow {
                    adjustments: vec![GainAdjust {
                        alevcode: 0b1111,
                        aloccode: 0b1111,
                    }],
                },
                // wd = 1 — aloccode 5 bits
                GainWindow {
                    adjustments: vec![GainAdjust {
                        alevcode: 0b0011,
                        aloccode: 0b11111,
                    }],
                },
            ],
        }],
    };

    let mut writer = BitWriter::new();
    gcd.write(&mut writer, WindowSequence::LongStop)
        .expect("write");
    let out = writer.finish();

    let mut reader = BitReader::new(&out);
    let parsed = GainControlData::parse(&mut reader, WindowSequence::LongStop).expect("parse");
    assert_eq!(parsed, gcd);
}

/// Max-everything: `max_band = 3`, every per-`(bd, wd)` slot holds
/// 7 adjustments (the 3-bit `adjust_num` cap), every field at its
/// per-slot maximum.
#[test]
fn max_everything_eight_short_roundtrip() {
    let mut bands = Vec::with_capacity(3);
    for _bd in 1..=3u8 {
        let mut windows = Vec::with_capacity(8);
        for _wd in 0..8u8 {
            let adjustments: Vec<GainAdjust> = (0..7u8)
                .map(|_| GainAdjust {
                    alevcode: 0b1111,
                    aloccode: 0b11,
                })
                .collect();
            windows.push(GainWindow { adjustments });
        }
        bands.push(GainBand { windows });
    }
    let gcd = GainControlData { max_band: 3, bands };

    let mut writer = BitWriter::new();
    gcd.write(&mut writer, WindowSequence::EightShort)
        .expect("write");
    let out = writer.finish();

    let mut reader = BitReader::new(&out);
    let parsed = GainControlData::parse(&mut reader, WindowSequence::EightShort).expect("parse");
    assert_eq!(parsed, gcd);
}

/// `max_band > 0x03` is rejected.
#[test]
fn write_rejects_max_band_overflow() {
    let gcd = GainControlData {
        max_band: 4, // 2-bit cap is 3.
        bands: vec![],
    };
    let mut writer = BitWriter::new();
    assert_eq!(
        gcd.write(&mut writer, WindowSequence::OnlyLong),
        Err(Error::GainControlDataEncodeInvalid)
    );
}

/// `bands.len() != max_band` is rejected.
#[test]
fn write_rejects_band_count_mismatch() {
    let gcd = GainControlData {
        max_band: 2,
        bands: vec![GainBand {
            windows: vec![GainWindow {
                adjustments: vec![],
            }],
        }],
    };
    let mut writer = BitWriter::new();
    assert_eq!(
        gcd.write(&mut writer, WindowSequence::OnlyLong),
        Err(Error::GainControlDataEncodeInvalid)
    );
}

/// `windows.len()` not matching `num_windows(window_sequence)` is rejected.
#[test]
fn write_rejects_window_count_mismatch_for_long_start() {
    // LongStart expects 2 windows; we supply 1.
    let gcd = GainControlData {
        max_band: 1,
        bands: vec![GainBand {
            windows: vec![GainWindow {
                adjustments: vec![],
            }],
        }],
    };
    let mut writer = BitWriter::new();
    assert_eq!(
        gcd.write(&mut writer, WindowSequence::LongStart),
        Err(Error::GainControlDataEncodeInvalid)
    );
}

/// `adjustments.len() > 7` overflows the 3-bit `adjust_num` field.
#[test]
fn write_rejects_adjust_num_overflow() {
    let adjustments: Vec<GainAdjust> = (0..8u8)
        .map(|_| GainAdjust {
            alevcode: 0,
            aloccode: 0,
        })
        .collect();
    let gcd = GainControlData {
        max_band: 1,
        bands: vec![GainBand {
            windows: vec![GainWindow { adjustments }],
        }],
    };
    let mut writer = BitWriter::new();
    assert_eq!(
        gcd.write(&mut writer, WindowSequence::OnlyLong),
        Err(Error::GainControlDataEncodeInvalid)
    );
}

/// `alevcode > 0x0f` overflows the 4-bit field.
#[test]
fn write_rejects_alevcode_overflow() {
    let gcd = GainControlData {
        max_band: 1,
        bands: vec![GainBand {
            windows: vec![GainWindow {
                adjustments: vec![GainAdjust {
                    alevcode: 0x10,
                    aloccode: 0,
                }],
            }],
        }],
    };
    let mut writer = BitWriter::new();
    assert_eq!(
        gcd.write(&mut writer, WindowSequence::OnlyLong),
        Err(Error::GainControlDataEncodeInvalid)
    );
}

/// `aloccode` overflow at the `EightShort` (2-bit) slot.
#[test]
fn write_rejects_aloccode_overflow_eight_short() {
    let mut windows = Vec::with_capacity(8);
    for _ in 0..8 {
        windows.push(GainWindow {
            adjustments: vec![],
        });
    }
    windows[0].adjustments.push(GainAdjust {
        alevcode: 0,
        aloccode: 0b100, // exceeds 2-bit cap (0..=3).
    });
    let gcd = GainControlData {
        max_band: 1,
        bands: vec![GainBand { windows }],
    };
    let mut writer = BitWriter::new();
    assert_eq!(
        gcd.write(&mut writer, WindowSequence::EightShort),
        Err(Error::GainControlDataEncodeInvalid)
    );
}

/// `aloccode` overflow at the `LongStart` `wd = 1` (2-bit) slot.
#[test]
fn write_rejects_aloccode_overflow_long_start_wd1() {
    let gcd = GainControlData {
        max_band: 1,
        bands: vec![GainBand {
            windows: vec![
                GainWindow {
                    adjustments: vec![],
                },
                GainWindow {
                    adjustments: vec![GainAdjust {
                        alevcode: 0,
                        aloccode: 0b100, // exceeds 2-bit cap.
                    }],
                },
            ],
        }],
    };
    let mut writer = BitWriter::new();
    assert_eq!(
        gcd.write(&mut writer, WindowSequence::LongStart),
        Err(Error::GainControlDataEncodeInvalid)
    );
}

/// `aloccode` overflow at the `OnlyLong` 5-bit slot (cap is 31).
#[test]
fn write_rejects_aloccode_overflow_only_long() {
    let gcd = GainControlData {
        max_band: 1,
        bands: vec![GainBand {
            windows: vec![GainWindow {
                adjustments: vec![GainAdjust {
                    alevcode: 0,
                    aloccode: 0b100000, // 32, exceeds 5-bit cap.
                }],
            }],
        }],
    };
    let mut writer = BitWriter::new();
    assert_eq!(
        gcd.write(&mut writer, WindowSequence::OnlyLong),
        Err(Error::GainControlDataEncodeInvalid)
    );
}

/// `parse` surfaces `UnexpectedEnd` when the bitstream runs out mid-record.
#[test]
fn parse_unexpected_end_during_alevcode() {
    // max_band = 1, then one byte of zero-padding only — adjust_num
    // reads the top 3 bits of byte 1 as 0, but if we then claim
    // 1 adjustment, alevcode/aloccode demand 9 more bits we don't have.
    //
    // Construct: max_band=01 (2b), adjust_num=001 (3b, value 1),
    // then truncate.
    let bytes = [0b01_001_000u8]; // 8 bits only
    let mut reader = BitReader::new(&bytes);
    let res = GainControlData::parse(&mut reader, WindowSequence::OnlyLong);
    assert_eq!(res, Err(Error::UnexpectedEnd));
}

/// Hand-pinned wire-layout: the `max_band == 0` block is exactly
/// 2 bits, zero-padded to one byte by the writer flush.
#[test]
fn empty_block_byte_layout() {
    let gcd = GainControlData::default();
    let mut writer = BitWriter::new();
    gcd.write(&mut writer, WindowSequence::OnlyLong)
        .expect("write");
    let out = writer.finish();
    assert_eq!(out, vec![0u8]);
}

/// Parser ignores trailing bits past the body — the surrounding
/// `individual_channel_stream()` is responsible for stepping any
/// subsequent fields after `gain_control_data()`.
#[test]
fn parse_does_not_consume_trailing_bits() {
    // max_band=0 (2 bits) then 6 arbitrary bits we expect untouched.
    let bytes = [0b00_111010u8, 0b1010_1100];
    let mut reader = BitReader::new(&bytes);
    let parsed = GainControlData::parse(&mut reader, WindowSequence::OnlyLong).expect("parse");
    assert_eq!(parsed.max_band, 0);
    // The next 6 bits should still read as 0b111010.
    let trailing = reader.read_u32(6).expect("trailing bits");
    assert_eq!(trailing, 0b111010);
}

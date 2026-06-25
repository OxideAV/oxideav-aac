//! Integration tests for the §4.4.2.7 / Table 4.51 / Table 4.52
//! / Table 4.53 `extension_payload()` parser + encoder primitive.
//!
//! Each test pairs a hand-pinned wire layout against the in-memory
//! [`ExtensionPayload`] / [`DynamicRangeInfo`] structure, exercising
//! the bit-exact inverse property of
//! [`ExtensionPayload::parse`] / [`ExtensionPayload::write`].

use oxideav_aac::extension_payload::{
    excluded_group_count, DrcBandRecord, DrcBands, DynamicRangeInfo, ExcludedChannels,
    ExtensionPayload, ExtensionType, PceTagFields, ProgRefLevelFields, FILL_DATA_BYTE,
    FILL_DATA_NIBBLE,
};
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

// ---------------------------------------------------------------
// ExtensionType — Table 4.59 + Table 40 dispatch
// ---------------------------------------------------------------

#[test]
fn extension_type_from_bits_known_values() {
    assert_eq!(
        ExtensionType::from_bits(0b0000).unwrap(),
        ExtensionType::Fill
    );
    assert_eq!(
        ExtensionType::from_bits(0b0001).unwrap(),
        ExtensionType::FillData
    );
    assert_eq!(
        ExtensionType::from_bits(0b1011).unwrap(),
        ExtensionType::DynamicRange
    );
}

#[test]
fn extension_type_as_u8_round_trip() {
    for raw in [0b0000u8, 0b0001, 0b1011] {
        let ty = ExtensionType::from_bits(raw).unwrap();
        assert_eq!(ty.as_u8(), raw);
    }
    // SBR variants round-trip the value even though `from_bits`
    // surfaces them as an error — the enum variant exists for the
    // future SBR back-end to construct directly.
    assert_eq!(ExtensionType::SbrData.as_u8(), 0b1101);
    assert_eq!(ExtensionType::SbrDataCrc.as_u8(), 0b1110);
}

#[test]
fn extension_type_sbr_rejected() {
    assert_eq!(
        ExtensionType::from_bits(0b1101),
        Err(Error::UnsupportedExtensionSbr(0b1101))
    );
    assert_eq!(
        ExtensionType::from_bits(0b1110),
        Err(Error::UnsupportedExtensionSbr(0b1110))
    );
}

#[test]
fn extension_type_reserved_rejected() {
    // Every 4-bit value except 0/1/0xb/0xd/0xe is reserved.
    for raw in [
        0b0010u8, 0b0011, 0b0100, 0b0101, 0b0110, 0b0111, 0b1000, 0b1001, 0b1010, 0b1100, 0b1111,
    ] {
        assert_eq!(
            ExtensionType::from_bits(raw),
            Err(Error::UnsupportedExtensionType(raw))
        );
    }
}

// ---------------------------------------------------------------
// EXT_FILL — Table 4.51 default branch
// ---------------------------------------------------------------

#[test]
fn ext_fill_cnt_eq_1_only_nibble_is_unused() {
    // cnt == 1 ⇒ body_bits = 8 * 0 + 4 = 4. Total bits on wire is
    // 4 (extension_type) + 4 (other_bits) = 8.
    let payload = ExtensionPayload::Fill {
        cnt: 1,
        other_bits: vec![0b1010_0000],
    };
    let mut w = BitWriter::new();
    let n = payload.write(&mut w).unwrap();
    assert_eq!(n, 1);
    let bytes = w.finish();
    assert_eq!(bytes.len(), 1);
    // High nibble = 0b0000 (EXT_FILL), low nibble = 0b1010 (other_bits top 4).
    assert_eq!(bytes[0], 0b0000_1010);

    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 1).unwrap();
    assert_eq!(round, payload);
    assert_eq!(round.byte_length(), 1);
}

#[test]
fn ext_fill_cnt_eq_3_roundtrip() {
    // cnt == 3 ⇒ body_bits = 8 * 2 + 4 = 20.
    // Pack 20 bits MSB-first into 3 bytes (last byte's low 4 bits
    // are unused).
    let mut other = vec![0xAB, 0xCD, 0xE0];
    other[2] &= 0xF0; // keep only the top 4 bits of the trailing partial byte
    let payload = ExtensionPayload::Fill {
        cnt: 3,
        other_bits: other.clone(),
    };

    let mut w = BitWriter::new();
    let n = payload.write(&mut w).unwrap();
    assert_eq!(n, 3);
    let bytes = w.finish();
    // 4 + 20 = 24 bits = 3 bytes.
    assert_eq!(bytes.len(), 3);

    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 3).unwrap();
    assert_eq!(round, payload);
}

#[test]
fn ext_fill_writer_rejects_wrong_other_bits_length() {
    // cnt == 2 ⇒ body_bits = 12 ⇒ expected 2 bytes.
    let payload = ExtensionPayload::Fill {
        cnt: 2,
        other_bits: vec![0xAA], // one byte short
    };
    let mut w = BitWriter::new();
    assert_eq!(payload.write(&mut w), Err(Error::ExtensionPayloadInvalid));
}

#[test]
fn ext_fill_writer_rejects_cnt_zero() {
    let payload = ExtensionPayload::Fill {
        cnt: 0,
        other_bits: vec![],
    };
    let mut w = BitWriter::new();
    assert_eq!(payload.write(&mut w), Err(Error::ExtensionPayloadInvalid));
}

#[test]
fn ext_fill_parser_rejects_cnt_zero() {
    let bytes = [0x00];
    let mut r = BitReader::new(&bytes);
    assert_eq!(
        ExtensionPayload::parse(&mut r, 0),
        Err(Error::ExtensionPayloadInvalid)
    );
}

// ---------------------------------------------------------------
// EXT_FILL_DATA — Table 4.51 normative pattern
// ---------------------------------------------------------------

#[test]
fn ext_fill_data_cnt_eq_3_roundtrip() {
    // cnt == 3 ⇒ extension_type (4 bits) + fill_nibble (4 bits) +
    // 2 × fill_byte = 24 bits = 3 bytes.
    let payload = ExtensionPayload::FillData { cnt: 3 };
    let mut w = BitWriter::new();
    let n = payload.write(&mut w).unwrap();
    assert_eq!(n, 3);
    let bytes = w.finish();
    assert_eq!(bytes.len(), 3);
    // [extension_type:4 = 0b0001 | fill_nibble:4 = 0b0000]
    //   = 0b0001_0000 = 0x10
    // [fill_byte = 0xA5]
    // [fill_byte = 0xA5]
    assert_eq!(bytes, [0x10, FILL_DATA_BYTE, FILL_DATA_BYTE]);

    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 3).unwrap();
    assert_eq!(round, payload);
}

#[test]
fn ext_fill_data_cnt_eq_1_no_payload() {
    // cnt == 1 ⇒ extension_type (4 bits) + fill_nibble (4 bits) =
    // 8 bits = 1 byte. The 0-iteration `cnt - 1` loop emits no
    // fill_byte values.
    let payload = ExtensionPayload::FillData { cnt: 1 };
    let mut w = BitWriter::new();
    let n = payload.write(&mut w).unwrap();
    assert_eq!(n, 1);
    let bytes = w.finish();
    assert_eq!(bytes, [0x10]);
    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 1).unwrap();
    assert_eq!(round, payload);
}

#[test]
fn ext_fill_data_parser_rejects_non_zero_nibble() {
    // type=0b0001, nibble=0b0001 (illegal) ⇒ first byte 0b0001_0001 = 0x11.
    let bytes = [0x11, FILL_DATA_BYTE, FILL_DATA_BYTE];
    let mut r = BitReader::new(&bytes);
    assert_eq!(
        ExtensionPayload::parse(&mut r, 3),
        Err(Error::ExtensionPayloadInvalid)
    );
}

#[test]
fn ext_fill_data_parser_rejects_non_a5_byte() {
    // type=1, nibble=0, then one 0xA5 then one 0x00 (illegal).
    let bytes = [0x10, FILL_DATA_BYTE, 0x00];
    let mut r = BitReader::new(&bytes);
    assert_eq!(
        ExtensionPayload::parse(&mut r, 3),
        Err(Error::ExtensionPayloadInvalid)
    );
}

#[test]
fn ext_fill_data_constants_pinned() {
    // Sanity: the normative values match the spec quotation.
    assert_eq!(FILL_DATA_NIBBLE, 0b0000);
    assert_eq!(FILL_DATA_BYTE, 0b1010_0101);
}

// ---------------------------------------------------------------
// EXT_DYNAMIC_RANGE — Table 4.52 (minimal body)
// ---------------------------------------------------------------

#[test]
fn ext_dynamic_range_minimal_body_one_byte_overhead() {
    // No optional sections, drc_num_bands defaults to 1, so the
    // body is: [type:4 | pce_present:1 | excl_present:1 |
    // bands_present:1 | prog_present:1] then [dyn_rng_sgn:1 |
    // dyn_rng_ctl:7] = 8 + 8 = 16 bits = 2 bytes.
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: None,
        prog_ref_level: None,
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0x42,
        }],
    };
    assert_eq!(drc.byte_length(), 2);
    let payload = ExtensionPayload::DynamicRange(drc.clone());

    let mut w = BitWriter::new();
    let n = payload.write(&mut w).unwrap();
    assert_eq!(n, 2);
    let bytes = w.finish();
    assert_eq!(bytes.len(), 2);
    // First byte: 0b1011 (type) | 0b0000 (all four presence flags off) = 0xB0.
    assert_eq!(bytes[0], 0xB0);
    // Second byte: 0b0 (sgn=false) | 0b1000010 (0x42) = 0x42.
    assert_eq!(bytes[1], 0x42);

    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 2).unwrap();
    assert_eq!(round, payload);
}

#[test]
fn ext_dynamic_range_with_pce_tag_roundtrip() {
    let drc = DynamicRangeInfo {
        pce_tag: Some(PceTagFields {
            pce_instance_tag: 0x05,
            reserved: 0x0a,
        }),
        excluded_channels: None,
        drc_bands: None,
        prog_ref_level: None,
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: true,
            dyn_rng_ctl: 0x7f,
        }],
    };
    assert_eq!(drc.byte_length(), 3);

    let mut w = BitWriter::new();
    let n = ExtensionPayload::DynamicRange(drc.clone())
        .write(&mut w)
        .unwrap();
    assert_eq!(n, 3);
    let bytes = w.finish();

    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 3).unwrap();
    assert_eq!(round, ExtensionPayload::DynamicRange(drc));
}

#[test]
fn ext_dynamic_range_with_drc_bands_roundtrip() {
    // band_incr = 2 ⇒ drc_num_bands = 3.
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: Some(DrcBands {
            band_incr: 2,
            reserved: 0,
            band_top: vec![0x10, 0x20, 0x30],
        }),
        prog_ref_level: None,
        bands: vec![
            DrcBandRecord {
                dyn_rng_sgn: false,
                dyn_rng_ctl: 0x01,
            },
            DrcBandRecord {
                dyn_rng_sgn: true,
                dyn_rng_ctl: 0x02,
            },
            DrcBandRecord {
                dyn_rng_sgn: false,
                dyn_rng_ctl: 0x03,
            },
        ],
    };
    // n = 1 (leading) + 1 (band_incr+reserved) + 3 (band_top) + 3
    // (per-band records) = 8.
    assert_eq!(drc.byte_length(), 8);

    let mut w = BitWriter::new();
    let n = ExtensionPayload::DynamicRange(drc.clone())
        .write(&mut w)
        .unwrap();
    assert_eq!(n, 8);
    let bytes = w.finish();
    assert_eq!(bytes.len(), 8);

    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 8).unwrap();
    assert_eq!(round, ExtensionPayload::DynamicRange(drc));
}

#[test]
fn ext_dynamic_range_with_prog_ref_level_roundtrip() {
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: None,
        prog_ref_level: Some(ProgRefLevelFields {
            level: 0x55,
            reserved: true,
        }),
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0x10,
        }],
    };
    assert_eq!(drc.byte_length(), 3);

    let mut w = BitWriter::new();
    ExtensionPayload::DynamicRange(drc.clone())
        .write(&mut w)
        .unwrap();
    let bytes = w.finish();

    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 3).unwrap();
    assert_eq!(round, ExtensionPayload::DynamicRange(drc));
}

#[test]
fn ext_dynamic_range_with_excluded_channels_roundtrip() {
    // exclude_mask of 14 bits = 2 groups = 2 bytes consumed by
    // excluded_channels().
    let mut mask = vec![false; 14];
    mask[0] = true;
    mask[7] = true;
    mask[13] = true;
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: Some(ExcludedChannels {
            exclude_mask: mask.clone(),
        }),
        drc_bands: None,
        prog_ref_level: None,
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0,
        }],
    };
    // n = 1 (leading) + 2 (excluded_channels groups) + 1 (band) = 4.
    assert_eq!(drc.byte_length(), 4);

    let mut w = BitWriter::new();
    let n = ExtensionPayload::DynamicRange(drc.clone())
        .write(&mut w)
        .unwrap();
    assert_eq!(n, 4);
    let bytes = w.finish();
    assert_eq!(bytes.len(), 4);

    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 4).unwrap();
    assert_eq!(round, ExtensionPayload::DynamicRange(drc));
}

#[test]
fn ext_dynamic_range_full_combination_roundtrip() {
    // Every optional present, band_incr = 1 ⇒ 2 bands, 7-bit
    // excluded_chans = 1 group.
    let drc = DynamicRangeInfo {
        pce_tag: Some(PceTagFields {
            pce_instance_tag: 0x0f,
            reserved: 0x05,
        }),
        excluded_channels: Some(ExcludedChannels {
            exclude_mask: vec![true, false, true, false, true, false, true],
        }),
        drc_bands: Some(DrcBands {
            band_incr: 1,
            reserved: 0x07,
            band_top: vec![0x40, 0x80],
        }),
        prog_ref_level: Some(ProgRefLevelFields {
            level: 0x33,
            reserved: false,
        }),
        bands: vec![
            DrcBandRecord {
                dyn_rng_sgn: true,
                dyn_rng_ctl: 0x11,
            },
            DrcBandRecord {
                dyn_rng_sgn: false,
                dyn_rng_ctl: 0x22,
            },
        ],
    };
    // n = 1 + 1 (pce_tag) + 1 (1 excluded group) + 1 (band_incr+reserved)
    //     + 2 (band_top) + 1 (prog_ref_level) + 2 (bands)
    //   = 9.
    assert_eq!(drc.byte_length(), 9);

    let mut w = BitWriter::new();
    let n = ExtensionPayload::DynamicRange(drc.clone())
        .write(&mut w)
        .unwrap();
    assert_eq!(n, 9);
    let bytes = w.finish();
    assert_eq!(bytes.len(), 9);

    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 9).unwrap();
    assert_eq!(round, ExtensionPayload::DynamicRange(drc));
}

#[test]
fn ext_dynamic_range_writer_rejects_band_top_length_mismatch() {
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: Some(DrcBands {
            band_incr: 2,
            reserved: 0,
            band_top: vec![0x10, 0x20], // expected length 3 (1 + band_incr)
        }),
        prog_ref_level: None,
        bands: vec![
            DrcBandRecord {
                dyn_rng_sgn: false,
                dyn_rng_ctl: 0,
            },
            DrcBandRecord {
                dyn_rng_sgn: false,
                dyn_rng_ctl: 0,
            },
            DrcBandRecord {
                dyn_rng_sgn: false,
                dyn_rng_ctl: 0,
            },
        ],
    };
    let mut w = BitWriter::new();
    assert_eq!(
        ExtensionPayload::DynamicRange(drc).write(&mut w),
        Err(Error::ExtensionPayloadInvalid)
    );
}

#[test]
fn ext_dynamic_range_writer_rejects_bands_len_mismatch() {
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: Some(DrcBands {
            band_incr: 1,
            reserved: 0,
            band_top: vec![0x10, 0x20],
        }),
        prog_ref_level: None,
        // band_incr says 2 bands but only 1 record supplied.
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0,
        }],
    };
    let mut w = BitWriter::new();
    assert_eq!(
        ExtensionPayload::DynamicRange(drc).write(&mut w),
        Err(Error::ExtensionPayloadInvalid)
    );
}

#[test]
fn ext_dynamic_range_writer_rejects_field_overflow() {
    // dyn_rng_ctl > 0x7f.
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: None,
        prog_ref_level: None,
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0x80,
        }],
    };
    let mut w = BitWriter::new();
    assert_eq!(
        ExtensionPayload::DynamicRange(drc).write(&mut w),
        Err(Error::ExtensionPayloadInvalid)
    );

    // pce_instance_tag > 0x0f.
    let drc = DynamicRangeInfo {
        pce_tag: Some(PceTagFields {
            pce_instance_tag: 0x10,
            reserved: 0,
        }),
        excluded_channels: None,
        drc_bands: None,
        prog_ref_level: None,
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0,
        }],
    };
    let mut w = BitWriter::new();
    assert_eq!(
        ExtensionPayload::DynamicRange(drc).write(&mut w),
        Err(Error::ExtensionPayloadInvalid)
    );

    // prog_ref_level > 0x7f.
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: None,
        prog_ref_level: Some(ProgRefLevelFields {
            level: 0x80,
            reserved: false,
        }),
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0,
        }],
    };
    let mut w = BitWriter::new();
    assert_eq!(
        ExtensionPayload::DynamicRange(drc).write(&mut w),
        Err(Error::ExtensionPayloadInvalid)
    );
}

#[test]
fn ext_dynamic_range_writer_rejects_bad_excluded_channels_length() {
    let drc = DynamicRangeInfo {
        pce_tag: None,
        // 6 bits is not a multiple of 7 — can't round-trip.
        excluded_channels: Some(ExcludedChannels {
            exclude_mask: vec![false; 6],
        }),
        drc_bands: None,
        prog_ref_level: None,
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0,
        }],
    };
    let mut w = BitWriter::new();
    assert_eq!(
        ExtensionPayload::DynamicRange(drc).write(&mut w),
        Err(Error::ExtensionPayloadInvalid)
    );
}

#[test]
fn ext_dynamic_range_parser_rejects_cnt_mismatch() {
    // Build a valid byte stream for a 2-byte DRC body and then
    // claim cnt = 3. The parser should detect the mismatch
    // because Table 4.52's derived `n` is 2.
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: None,
        prog_ref_level: None,
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0,
        }],
    };
    let mut w = BitWriter::new();
    ExtensionPayload::DynamicRange(drc).write(&mut w).unwrap();
    // Append a stray byte so the parser doesn't run out of bits
    // before the n-vs-cnt check.
    let mut bytes = w.finish();
    bytes.push(0x00);
    let mut r = BitReader::new(&bytes);
    assert_eq!(
        ExtensionPayload::parse(&mut r, 3),
        Err(Error::ExtensionPayloadInvalid)
    );
}

// ---------------------------------------------------------------
// excluded_channels() helper coverage
// ---------------------------------------------------------------

#[test]
fn excluded_group_count_table() {
    assert_eq!(excluded_group_count(0), 0);
    assert_eq!(excluded_group_count(1), 1);
    assert_eq!(excluded_group_count(7), 1);
    assert_eq!(excluded_group_count(8), 2);
    assert_eq!(excluded_group_count(14), 2);
    assert_eq!(excluded_group_count(15), 3);
}

#[test]
fn excluded_channels_extension_continuation_bit() {
    // Two-group exclude_mask: the writer must emit
    // continuation=1 on the first group and continuation=0 on the
    // second. Round-trip via the DRC wrapper to keep the test
    // honest about wire framing.
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: Some(ExcludedChannels {
            // 14 bits ⇒ two 7-bit groups.
            exclude_mask: vec![
                true, false, false, false, false, false, false, false, false, false, false, false,
                false, true,
            ],
        }),
        drc_bands: None,
        prog_ref_level: None,
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0,
        }],
    };
    assert_eq!(drc.byte_length(), 4);

    let mut w = BitWriter::new();
    ExtensionPayload::DynamicRange(drc.clone())
        .write(&mut w)
        .unwrap();
    let bytes = w.finish();

    // Hand-decode (note Table 4.52 inlines each presence flag's
    // body inside the flag's branch, so the four presence flags do
    // NOT all fall in byte 0).
    //
    // byte[0] = type[3..0]=1011 | pce_present=0 | excl_present=1 |
    //           exclude_mask[0]=1 | exclude_mask[1]=0
    //         = 0b1011_0110 = 0xB6.
    assert_eq!(bytes[0], 0xB6);
    // byte[1] = exclude_mask[2..7]=00000 | cont1=1 |
    //           exclude_mask[7..9]=00
    //         = 0b0000_0100 = 0x04.
    assert_eq!(bytes[1], 0x04);
    // byte[2] = exclude_mask[9..13]=0000 | exclude_mask[13]=1 |
    //           cont2=0 | drc_bands_present=0 |
    //           prog_ref_level_present=0
    //         = 0b0000_1000 = 0x08.
    assert_eq!(bytes[2], 0x08);
    // byte[3] = the single (dyn_rng_sgn=0, dyn_rng_ctl=0) band
    //           record = 0b0_0000000 = 0x00.
    assert_eq!(bytes[3], 0x00);

    let mut r = BitReader::new(&bytes);
    let round = ExtensionPayload::parse(&mut r, 4).unwrap();
    assert_eq!(round, ExtensionPayload::DynamicRange(drc));
}

// ---------------------------------------------------------------
// Wire-format dispatch on extension_type
// ---------------------------------------------------------------

#[test]
fn parser_dispatch_sbr_extension_type_surfaced() {
    // Byte 0: [type:4=1101, padding:4=0000] = 0xD0.
    let bytes = [0xD0u8];
    let mut r = BitReader::new(&bytes);
    assert_eq!(
        ExtensionPayload::parse(&mut r, 1),
        Err(Error::UnsupportedExtensionSbr(0b1101))
    );
}

#[test]
fn parser_dispatch_sbr_crc_extension_type_surfaced() {
    // Byte 0: [type:4=1110, padding:4=0000] = 0xE0.
    let bytes = [0xE0u8];
    let mut r = BitReader::new(&bytes);
    assert_eq!(
        ExtensionPayload::parse(&mut r, 1),
        Err(Error::UnsupportedExtensionSbr(0b1110))
    );
}

#[test]
fn parser_dispatch_reserved_extension_type_surfaced() {
    // Byte 0: [type:4=0010, padding:4=0000] = 0x20.
    let bytes = [0x20u8];
    let mut r = BitReader::new(&bytes);
    assert_eq!(
        ExtensionPayload::parse(&mut r, 1),
        Err(Error::UnsupportedExtensionType(0b0010))
    );
}

// ---------------------------------------------------------------
// Bit-reader truncation surfaces UnexpectedEnd
// ---------------------------------------------------------------

#[test]
fn ext_fill_data_truncation_surfaces_unexpected_end() {
    // cnt = 3 expects 24 bits; supply only 16.
    let bytes = [0x10, FILL_DATA_BYTE];
    let mut r = BitReader::new(&bytes);
    assert_eq!(
        ExtensionPayload::parse(&mut r, 3),
        Err(Error::UnexpectedEnd)
    );
}

#[test]
fn ext_dynamic_range_truncation_surfaces_unexpected_end() {
    // Truncated DRC body — header + presence flags only, no band record.
    let bytes = [0xB0u8];
    let mut r = BitReader::new(&bytes);
    assert_eq!(
        ExtensionPayload::parse(&mut r, 2),
        Err(Error::UnexpectedEnd)
    );
}

#[test]
fn ext_fill_truncation_surfaces_unexpected_end() {
    // cnt = 3 expects 24 bits; supply only 8.
    let bytes = [0x00u8];
    let mut r = BitReader::new(&bytes);
    assert_eq!(
        ExtensionPayload::parse(&mut r, 3),
        Err(Error::UnexpectedEnd)
    );
}

// ---------------------------------------------------------------
// byte_length / num_bands accessors
// ---------------------------------------------------------------

#[test]
fn extension_payload_byte_length_matches_wire() {
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: None,
        prog_ref_level: None,
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0,
        }],
    };
    let payload = ExtensionPayload::DynamicRange(drc);
    let mut w = BitWriter::new();
    let n = payload.write(&mut w).unwrap();
    assert_eq!(n, payload.byte_length());
    let bytes = w.finish();
    assert_eq!(bytes.len(), n as usize);
}

#[test]
fn dynamic_range_info_num_bands_tracks_drc_bands() {
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: Some(DrcBands {
            band_incr: 3,
            reserved: 0,
            band_top: vec![0x10, 0x20, 0x30, 0x40],
        }),
        prog_ref_level: None,
        bands: vec![
            DrcBandRecord {
                dyn_rng_sgn: false,
                dyn_rng_ctl: 0,
            };
            4
        ],
    };
    assert_eq!(drc.num_bands(), 4);
}

// ---------------------------------------------------------------
// Trailing-bit non-consumption
// ---------------------------------------------------------------

#[test]
fn parser_does_not_overread_past_drc_body() {
    let drc = DynamicRangeInfo {
        pce_tag: None,
        excluded_channels: None,
        drc_bands: None,
        prog_ref_level: None,
        bands: vec![DrcBandRecord {
            dyn_rng_sgn: false,
            dyn_rng_ctl: 0x5a,
        }],
    };
    let mut w = BitWriter::new();
    ExtensionPayload::DynamicRange(drc.clone())
        .write(&mut w)
        .unwrap();
    let mut bytes = w.finish();
    let n = bytes.len();
    // Append a stray sentinel byte. The parser must stop exactly
    // at the body boundary and leave the sentinel readable.
    bytes.push(0xA5);
    let mut r = BitReader::new(&bytes);
    let _ = ExtensionPayload::parse(&mut r, n as u32).unwrap();
    // Reader should be sitting right at bit-position 8 * n.
    assert_eq!(r.bit_position(), 8 * n as u64);
    let trailing = r.read_u32(8).unwrap() as u8;
    assert_eq!(trailing, 0xA5);
}

// ---------------------------------------------------------------
// parse_with_sbr — §4.4.2.8 sbr_extension_data() routing
// ---------------------------------------------------------------

use oxideav_aac::extension_payload::ExtensionPayloadOrSbr;
use oxideav_aac::raw_data_block::IdSynEle;
use oxideav_aac::sbr_freq_bands::HiLoTables;
use oxideav_aac::sbr_grid::FrameClass;
use oxideav_aac::sbr_header::SbrHeader;
use oxideav_aac::sbr_huffman::{env_tables, noise_tables, SbrHuffContext};

const FS_SBR: u32 = 88_200;

/// Write an `sbr_header()` with explicit extra-1 params (deterministic
/// band geometry); extra-2 absent.
fn write_sbr_header(w: &mut BitWriter, amp_res: bool) {
    w.write_bit(amp_res); // bs_amp_res
    w.write_u32(5, 4); // bs_start_freq
    w.write_u32(0, 4); // bs_stop_freq
    w.write_u32(1, 3); // bs_xover_band
    w.write_u32(0, 2); // bs_reserved
    w.write_bit(true); // bs_header_extra_1
    w.write_bit(false); // bs_header_extra_2
    w.write_u32(0, 2); // bs_freq_scale
    w.write_bit(false); // bs_alter_scale
    w.write_u32(2, 2); // bs_noise_bands
}

fn header_bands() -> HiLoTables {
    let mut w = BitWriter::new();
    write_sbr_header(&mut w, false);
    let bytes = w.finish();
    let mut r = BitReader::new(&bytes);
    SbrHeader::parse(&mut r)
        .unwrap()
        .derive_bands(FS_SBR)
        .unwrap()
}

fn write_minimal_sce(w: &mut BitWriter, bands: &HiLoTables) {
    let n_high = bands.n_high();
    let n_q = bands.n_q();
    w.write_bit(false); // bs_data_extra
    w.write_u32(FrameClass::FixFix.to_bits(), 2);
    w.write_u32(0, 2); // 1 env
    w.write_bit(true); // freq_res high
    w.write_bit(false); // df_env
    w.write_bit(false); // df_noise
    for _ in 0..n_q {
        w.write_u32(1, 2);
    }
    let (_, (f_huff, f_lav)) = env_tables(SbrHuffContext {
        coupling: false,
        ch: false,
        amp_res: false,
    });
    w.write_u32(33, 7);
    for i in 1..n_high {
        let (len, code) = f_huff[(i + f_lav as usize) % f_huff.len()];
        w.write_u32(code, len as u32);
    }
    let (_, (nf, nfl)) = noise_tables(SbrHuffContext {
        coupling: false,
        ch: false,
        amp_res: false,
    });
    w.write_u32(10, 5);
    for i in 1..n_q {
        let (len, code) = nf[(i + nfl as usize) % nf.len()];
        w.write_u32(code, len as u32);
    }
    w.write_bit(false); // add_harmonic_flag
    w.write_bit(false); // extended_data
}

#[test]
fn parse_with_sbr_routes_ext_sbr_data() {
    // FIL body: extension_type = EXT_SBR_DATA (0b1101), then
    // bs_header_flag + sbr_header() + sbr_single_channel_element().
    let bands = header_bands();
    let mut w = BitWriter::new();
    w.write_u32(0b1101, 4); // extension_type = EXT_SBR_DATA
    w.write_bit(true); // bs_header_flag
    write_sbr_header(&mut w, true);
    write_minimal_sce(&mut w, &bands);
    let mut bytes = w.finish();
    let cnt = (bytes.len() + 1) as u32; // one trailing fill byte
    bytes.push(0x00);

    let mut r = BitReader::new(&bytes);
    let out = ExtensionPayload::parse_with_sbr(&mut r, cnt, IdSynEle::Sce, FS_SBR, None).unwrap();
    match out {
        ExtensionPayloadOrSbr::Sbr(sbr) => {
            assert!(sbr.header_present);
            assert_eq!(sbr.header.start_freq, 5);
            assert_eq!(sbr.element.channels.len(), 1);
            assert_eq!(sbr.element.channels[0].envelope.data[0][0], 33);
        }
        other => panic!("expected Sbr, got {other:?}"),
    }
}

#[test]
fn parse_with_sbr_passes_through_fill() {
    // A non-SBR extension type still produces a standard payload.
    let mut w = BitWriter::new();
    w.write_u32(0b0000, 4); // EXT_FILL
    w.write_u32(0, 4); // 4 other_bits (cnt == 1)
    let bytes = w.finish();
    let mut r = BitReader::new(&bytes);
    let out = ExtensionPayload::parse_with_sbr(&mut r, 1, IdSynEle::Sce, FS_SBR, None).unwrap();
    match out {
        ExtensionPayloadOrSbr::Payload(ExtensionPayload::Fill { cnt, .. }) => {
            assert_eq!(cnt, 1);
        }
        other => panic!("expected Fill payload, got {other:?}"),
    }
}

#[test]
fn parse_with_sbr_crc_variant_reads_crc() {
    let bands = header_bands();
    let mut w = BitWriter::new();
    w.write_u32(0b1110, 4); // EXT_SBR_DATA_CRC
    w.write_u32(0x1B3, 10); // bs_sbr_crc_bits
    w.write_bit(true);
    write_sbr_header(&mut w, true);
    write_minimal_sce(&mut w, &bands);
    let mut bytes = w.finish();
    let cnt = (bytes.len() + 1) as u32;
    bytes.push(0x00);
    let mut r = BitReader::new(&bytes);
    let out = ExtensionPayload::parse_with_sbr(&mut r, cnt, IdSynEle::Sce, FS_SBR, None).unwrap();
    match out {
        ExtensionPayloadOrSbr::Sbr(sbr) => assert_eq!(sbr.crc, Some(0x1B3)),
        other => panic!("expected Sbr, got {other:?}"),
    }
}

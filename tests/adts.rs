//! Tests for [`oxideav_aac::adts::AdtsHeader`] — synthetic ADTS
//! headers constructed via `oxideav_core::bits::BitWriter` so the
//! tests do not depend on any external AAC encoder.

use oxideav_aac::adts::{AdtsHeader, ADTS_HEADER_BYTES_NO_CRC, ADTS_HEADER_BYTES_WITH_CRC};
use oxideav_aac::Error;
use oxideav_core::bits::BitWriter;

/// Build a 7- or 9-byte ADTS header with the given fields. All
/// other reserved-ish fields are zero.
#[allow(clippy::too_many_arguments)]
fn build_header(
    mpeg_version_mpeg2: bool,
    protection_absent: bool,
    profile: u8,
    sampling_frequency_index: u8,
    channel_configuration: u8,
    aac_frame_length: u16,
    adts_buffer_fullness: u16,
    raw_blocks_minus_one: u8,
) -> Vec<u8> {
    let mut bw = BitWriter::new();
    // syncword
    bw.write_u32(0xFFF, 12);
    // ID
    bw.write_bit(mpeg_version_mpeg2);
    // layer (2)
    bw.write_u32(0, 2);
    // protection_absent
    bw.write_bit(protection_absent);
    // profile (2)
    bw.write_u32(profile as u32, 2);
    // sampling_frequency_index (4)
    bw.write_u32(sampling_frequency_index as u32, 4);
    // private_bit
    bw.write_bit(false);
    // channel_configuration (3)
    bw.write_u32(channel_configuration as u32, 3);
    // original_copy, home, copyright_id_bit, copyright_id_start (4×1)
    bw.write_u32(0, 4);
    // aac_frame_length (13)
    bw.write_u32(aac_frame_length as u32, 13);
    // adts_buffer_fullness (11)
    bw.write_u32(adts_buffer_fullness as u32, 11);
    // number_of_raw_data_blocks_in_frame (2) — wire field is N-1
    bw.write_u32(raw_blocks_minus_one as u32, 2);
    if !protection_absent {
        // 16-bit CRC slot — zero in the synthetic builder. Phase 1
        // does not validate the CRC.
        bw.write_u32(0, 16);
    }
    bw.finish()
}

#[test]
fn parses_aac_lc_mono_44100_no_crc_no_padding() {
    // Mirror of `aac-lc-mono-44100-64kbps-adts` first frame: AAC-LC
    // (profile 1), 44.1 kHz (sfi 4), mono (chan_cfg 1), 287-byte
    // frame, buffer fullness 0x7FF, 1 RDB.
    let header = build_header(false, true, 1, 4, 1, 287, 0x7FF, 0);
    let (h, offset) = AdtsHeader::parse(&header).unwrap();
    assert_eq!(offset, ADTS_HEADER_BYTES_NO_CRC);
    assert!(!h.mpeg_version_mpeg2);
    assert!(h.protection_absent);
    assert_eq!(h.profile, 1);
    assert_eq!(h.audio_object_type(), 2); // AAC LC
    assert_eq!(h.sampling_frequency_index, 4);
    assert_eq!(h.sample_rate(), 44100);
    assert_eq!(h.channel_configuration, 1);
    assert_eq!(h.aac_frame_length, 287);
    assert_eq!(h.adts_buffer_fullness, 0x7FF);
    assert_eq!(h.number_of_raw_data_blocks_in_frame, 1);
    assert_eq!(h.payload_len(), 287 - 7);
}

#[test]
fn parses_with_crc_sets_offset_to_nine() {
    let header = build_header(false, false, 1, 4, 2, 484, 0x7FF, 0);
    let (h, offset) = AdtsHeader::parse(&header).unwrap();
    assert_eq!(offset, ADTS_HEADER_BYTES_WITH_CRC);
    assert!(!h.protection_absent);
    assert_eq!(h.payload_len(), 484 - 9);
}

#[test]
fn sample_rate_resolves_all_legal_indices() {
    let expected = [
        96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350,
    ];
    for (i, want) in expected.iter().enumerate() {
        let header = build_header(false, true, 1, i as u8, 2, 100, 0x7FF, 0);
        let (h, _) = AdtsHeader::parse(&header).unwrap();
        assert_eq!(h.sample_rate(), *want, "sfi={}", i);
    }
}

#[test]
fn rejects_missing_sync() {
    let mut data = build_header(false, true, 1, 4, 1, 287, 0x7FF, 0);
    // Corrupt the sync — flip the leading byte.
    data[0] = 0x00;
    assert_eq!(AdtsHeader::parse(&data), Err(Error::AdtsSyncNotFound));
}

#[test]
fn rejects_partial_sync_word() {
    // 0xFF E... — first 12 bits are 0xFFE not 0xFFF.
    let mut data = build_header(false, true, 1, 4, 1, 287, 0x7FF, 0);
    data[1] = 0xE0 | (data[1] & 0x0F);
    assert_eq!(AdtsHeader::parse(&data), Err(Error::AdtsSyncNotFound));
}

#[test]
fn rejects_reserved_sampling_frequency_index_13() {
    let header = build_header(false, true, 1, 13, 1, 287, 0x7FF, 0);
    assert_eq!(
        AdtsHeader::parse(&header),
        Err(Error::AdtsReservedSampleRateIndex)
    );
}

#[test]
fn rejects_reserved_sampling_frequency_index_15() {
    let header = build_header(false, true, 1, 15, 1, 287, 0x7FF, 0);
    assert_eq!(
        AdtsHeader::parse(&header),
        Err(Error::AdtsReservedSampleRateIndex)
    );
}

#[test]
fn rejects_non_zero_layer() {
    // Hand-roll a header with layer = 0b01 so the legal-builder
    // helper doesn't have to grow a knob for an invalid path.
    let mut bw = BitWriter::new();
    bw.write_u32(0xFFF, 12);
    bw.write_bit(false); // ID = MPEG-4
    bw.write_u32(0b01, 2); // layer (illegal)
    bw.write_bit(true); // protection_absent
    bw.write_u32(1, 2); // profile = LC
    bw.write_u32(4, 4); // sfi = 44100
    bw.write_bit(false); // private
    bw.write_u32(1, 3); // chan_cfg = mono
    bw.write_u32(0, 4); // orig/home/copy/copy_start
    bw.write_u32(287, 13);
    bw.write_u32(0x7FF, 11);
    bw.write_u32(0, 2);
    let data = bw.finish();
    assert_eq!(AdtsHeader::parse(&data), Err(Error::AdtsLayerNonZero));
}

#[test]
fn rejects_frame_length_smaller_than_header() {
    // protection_absent ⇒ header is 7 bytes; a frame length of 6 is
    // structurally impossible.
    let header = build_header(false, true, 1, 4, 1, 6, 0x7FF, 0);
    assert_eq!(
        AdtsHeader::parse(&header),
        Err(Error::AdtsFrameLengthTooSmall)
    );
}

#[test]
fn rejects_truncated_input() {
    let header = build_header(false, true, 1, 4, 1, 287, 0x7FF, 0);
    assert_eq!(AdtsHeader::parse(&header[..6]), Err(Error::UnexpectedEnd));
}

#[test]
fn raw_blocks_field_decoded_as_n_plus_one() {
    for wire in 0..4u8 {
        let header = build_header(false, true, 1, 4, 1, 100, 0x7FF, wire);
        let (h, _) = AdtsHeader::parse(&header).unwrap();
        assert_eq!(h.number_of_raw_data_blocks_in_frame, wire + 1);
    }
}

#[test]
fn mpeg2_id_bit_round_trips() {
    let header = build_header(true, true, 1, 4, 1, 100, 0x7FF, 0);
    let (h, _) = AdtsHeader::parse(&header).unwrap();
    assert!(h.mpeg_version_mpeg2);
}

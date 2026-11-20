//! `extension_payload()` parser ↔ writer round trip under arbitrary
//! input.
//!
//! The first two bytes choose the fill element's `cnt`; the rest is
//! the payload buffer (which may be shorter than `cnt` — the
//! truncated case). Whatever parses must
//!
//! * report a length of `1..=cnt` bytes and have consumed exactly
//!   that many bytes of the reader,
//! * re-serialise to the same length and reparse to an equal value
//!   that serialises to the same bytes (the `EXT_FILL_DATA` body is
//!   discarded by the parser, so the comparison is made on the
//!   writer's output, not on the input).
//!
//! The SBR-aware entry sees the same bytes under both channel-element
//! kinds. Neither path may panic.

#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_aac::extension_payload::ExtensionPayload;
use oxideav_aac::raw_data_block::IdSynEle;
use oxideav_core::bits::{BitReader, BitWriter};

fuzz_target!(|data: &[u8]| {
    if data.len() < 2 {
        return;
    }
    // Up to twice the largest fill element (269 bytes), zero included.
    let cnt = u32::from(u16::from_be_bytes([data[0], data[1]])) % 540;
    let body = &data[2..];

    let mut reader = BitReader::new(body);
    if let Ok(payload) = ExtensionPayload::parse(&mut reader, cnt) {
        let n = payload.byte_length();
        assert!((1..=cnt).contains(&n), "length {n} outside 1..={cnt}");
        assert_eq!(reader.bit_position(), u64::from(n) * 8);

        let mut writer = BitWriter::new();
        let written = payload
            .write(&mut writer)
            .expect("parsed payload must be writable");
        assert_eq!(written, n);
        let bytes = writer.finish();
        assert_eq!(bytes.len() as u32, n);

        let mut reader = BitReader::new(&bytes);
        let again = ExtensionPayload::parse(&mut reader, n).expect("written payload must parse");
        assert_eq!(again, payload);
        assert_eq!(reader.bit_position(), u64::from(n) * 8);
        let mut writer = BitWriter::new();
        again.write(&mut writer).unwrap();
        assert_eq!(writer.finish(), bytes);
    }

    for id_aac in [IdSynEle::Sce, IdSynEle::Cpe] {
        let mut reader = BitReader::new(body);
        let _ = ExtensionPayload::parse_with_sbr(&mut reader, cnt, id_aac, 48_000, None);
    }
});

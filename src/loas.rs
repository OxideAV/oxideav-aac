//! LOAS AudioSyncStream framing — ISO/IEC 14496-3 §1.7.2.
//!
//! LOAS wraps `audio_mux_element()` payloads with an 11-bit syncword
//! (`0x2B7`) and a 13-bit `audioMuxLengthBytes` field so they can be
//! carried over byte-stream transports (broadcast TS, file). The
//! actual demultiplex is delegated to [`crate::latm::AudioMuxElement`].
//!
//! Wire layout (24-bit fixed header):
//!
//! ```text
//! syncword              11 bits (= 0x2B7)
//! audioMuxLengthBytes   13 bits
//! audio_mux_element(1)  audioMuxLengthBytes * 8 bits
//! ```

use oxideav_core::{Error, Result};

/// LOAS syncword (11 MSB-first bits) — `0b010 1011 0111`.
pub const LOAS_SYNCWORD: u16 = 0x2B7;

/// Fixed LOAS header size in bytes (24 bits).
pub const LOAS_HEADER_BYTES: usize = 3;

/// One LOAS `AudioSyncStream` frame: the framing header bookkeeping
/// plus the byte-aligned payload that should be fed to
/// [`crate::latm::AudioMuxElement::parse`].
#[derive(Clone, Debug)]
pub struct LoasFrame<'a> {
    /// `audioMuxLengthBytes` from the header.
    pub audio_mux_length_bytes: usize,
    /// Pointer into the source buffer at the start of the payload —
    /// caller passes this to `AudioMuxElement::parse`.
    pub payload: &'a [u8],
    /// Total bytes consumed from the source buffer (header + payload).
    pub frame_length: usize,
}

/// Parse a single LOAS `AudioSyncStream` frame anchored at the start of
/// `data`. The caller is responsible for syncword scanning if `data`
/// might begin in the middle of a frame — see [`find_loas_sync`].
pub fn parse_loas_frame(data: &[u8]) -> Result<LoasFrame<'_>> {
    if data.len() < LOAS_HEADER_BYTES {
        return Err(Error::NeedMore);
    }
    // Bits 0..11 of the first 24 bits = syncword.
    let header24 = ((data[0] as u32) << 16) | ((data[1] as u32) << 8) | (data[2] as u32);
    let sync = (header24 >> 13) & 0x7FF;
    if sync as u16 != LOAS_SYNCWORD {
        return Err(Error::invalid("LOAS: missing 0x2B7 syncword"));
    }
    let audio_mux_length_bytes = (header24 & 0x1FFF) as usize;
    let frame_length = LOAS_HEADER_BYTES + audio_mux_length_bytes;
    if data.len() < frame_length {
        return Err(Error::NeedMore);
    }
    let payload = &data[LOAS_HEADER_BYTES..frame_length];
    Ok(LoasFrame {
        audio_mux_length_bytes,
        payload,
        frame_length,
    })
}

/// Scan `data` for the first occurrence of the 11-bit LOAS syncword
/// `0x2B7`. Returns the byte offset of the candidate header. The
/// syncword is required to be byte-aligned in LOAS — this matches the
/// behaviour of every reference parser and the wire format described
/// in §1.7.2 (the audioSyncStream is delivered on a byte boundary).
pub fn find_loas_sync(data: &[u8]) -> Option<usize> {
    let mut i = 0;
    while i + 1 < data.len() {
        // Sync bits 10..3 = 0x56, bits 2..0 = top 3 of bits 7..5 of next
        // byte = 0b101 (since 0x2B7 = 0b010 1011 0111).
        // Equivalently: data[i] == 0x56 && (data[i+1] & 0xE0) == 0xE0.
        if data[i] == 0x56 && (data[i + 1] & 0xE0) == 0xE0 {
            return Some(i);
        }
        i += 1;
    }
    None
}

/// Iterator over the LOAS frames in a byte buffer. Skips any leading
/// non-sync bytes; stops at the first malformed frame.
pub struct LoasFrameIter<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> LoasFrameIter<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }
}

impl<'a> Iterator for LoasFrameIter<'a> {
    type Item = (usize, LoasFrame<'a>);

    fn next(&mut self) -> Option<Self::Item> {
        // Scan for the next syncword from `pos`.
        let rel = find_loas_sync(&self.data[self.pos..])?;
        let abs = self.pos + rel;
        let frame = parse_loas_frame(&self.data[abs..]).ok()?;
        let consumed = frame.frame_length;
        self.pos = abs + consumed;
        Some((abs, frame))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_loas_frame(payload: &[u8]) -> Vec<u8> {
        // Build a 3-byte header carrying audioMuxLengthBytes = payload.len().
        let n = payload.len() as u32;
        assert!(n < (1 << 13), "payload too long for 13-bit length");
        let header_bits: u32 = (LOAS_SYNCWORD as u32) << 13 | n;
        let mut out = vec![
            ((header_bits >> 16) & 0xFF) as u8,
            ((header_bits >> 8) & 0xFF) as u8,
            (header_bits & 0xFF) as u8,
        ];
        out.extend_from_slice(payload);
        out
    }

    #[test]
    fn parses_header_and_payload() {
        let payload = vec![0xAA; 17];
        let frame = build_loas_frame(&payload);
        let parsed = parse_loas_frame(&frame).expect("parse");
        assert_eq!(parsed.audio_mux_length_bytes, 17);
        assert_eq!(parsed.payload, &payload[..]);
        assert_eq!(parsed.frame_length, 3 + 17);
    }

    #[test]
    fn rejects_bad_sync() {
        let bad = [0u8; 4];
        assert!(matches!(
            parse_loas_frame(&bad),
            Err(oxideav_core::Error::InvalidData(_))
        ));
    }

    #[test]
    fn need_more_when_payload_short() {
        let mut frame = build_loas_frame(&[0xAA; 17]);
        frame.truncate(5); // header + 2 bytes of payload, missing 15
        assert!(matches!(
            parse_loas_frame(&frame),
            Err(oxideav_core::Error::NeedMore)
        ));
    }

    #[test]
    fn finds_aligned_sync() {
        let mut buf = vec![0u8, 0u8];
        buf.extend(build_loas_frame(&[0x01, 0x02]));
        assert_eq!(find_loas_sync(&buf), Some(2));
    }

    #[test]
    fn iter_skips_garbage_and_walks_frames() {
        let mut buf = vec![0xCC; 4];
        buf.extend(build_loas_frame(&[0xAA; 4]));
        buf.extend(build_loas_frame(&[0xBB; 6]));
        let frames: Vec<_> = LoasFrameIter::new(&buf).collect();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].1.payload, &[0xAA; 4]);
        assert_eq!(frames[1].1.payload, &[0xBB; 6]);
    }
}

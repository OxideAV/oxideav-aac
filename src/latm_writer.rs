//! LOAS / LATM writer — ISO/IEC 14496-3 §1.7 written forward for the
//! streams this crate's encoders produce.
//!
//! [`LoasWriter`] emits `AudioSyncStream()` frames (Table 1.36: the
//! 11-bit `0x2B7` syncword + 13-bit `audioMuxLengthBytes` + a
//! byte-aligned `AudioMuxElement(1)`), one per access unit:
//!
//! * The multiplex is the single-program / single-layer /
//!   `allStreamsSameTimeFraming` shape with `frameLengthType = 0`
//!   (byte-escape `MuxSlotLengthBytes`), `numSubFrames = 0`.
//! * `StreamMuxConfig()` is written with **`audioMuxVersion = 1`** so
//!   the `AudioSpecificConfig` is length-prefixed (`ascLen =
//!   LatmGetValue()`): an ASC whose syntax does not fill whole bytes —
//!   the backward-compatible HE-AAC form with its 21-bit
//!   `syncExtensionType 0x2b7` trailer in particular — stays
//!   unambiguous, and a parser applies the §1.6.5 implicit-SBR probe
//!   over the declared length exactly as
//!   [`crate::latm::StreamMuxConfig::parse`] does.
//! * The config is transmitted in the first frame and repeated every
//!   [`LoasWriter::mux_interval`] frames; the frames between carry
//!   `useSameStreamMux = 1`.
//!
//! The payload of each frame is one `raw_data_block()` — for the ADTS
//! encoders in this crate, [`strip_adts`] converts their output into
//! the per-frame payloads (LATM carries the configuration out of the
//! ADTS header's reach, which is also how the *explicit* SBR
//! signalling of §1.6.5 travels).

use oxideav_core::bits::BitWriter;

use crate::{Error, Result};

/// The 11-bit LOAS `AudioSyncStream` syncword.
pub const LOAS_SYNCWORD: u32 = 0x2B7;

/// `LatmGetValue()` inverse (Table 1.43): `bytesForValue` (2 bits) then
/// `value` in that many + 1 bytes, big-endian.
fn write_latm_value(w: &mut BitWriter, value: u32) {
    let bytes = match value {
        0..=0xFF => 1u32,
        0x100..=0xFFFF => 2,
        0x1_0000..=0xFF_FFFF => 3,
        _ => 4,
    };
    w.write_u32(bytes - 1, 2);
    for i in (0..bytes).rev() {
        w.write_u32((value >> (8 * i)) & 0xFF, 8);
    }
}

/// A LOAS (`AudioSyncStream`) writer for a single-layer AAC stream.
#[derive(Debug, Clone)]
pub struct LoasWriter {
    /// The `AudioSpecificConfig` bits (from [`crate::asc_writer`]).
    asc: Vec<u8>,
    /// The exact ASC bit count (the bytes may carry alignment padding).
    asc_bits: u32,
    /// Transmit `StreamMuxConfig` in every N-th frame (and the first).
    pub mux_interval: u32,
    frames: u64,
}

impl LoasWriter {
    /// A writer for the given ASC. `asc_bits` is the exact syntax bit
    /// count (`asc.len() * 8` when the ASC happens to be whole bytes;
    /// [`crate::asc_writer`] returns byte-padded buffers whose bit
    /// counts are 16 for AAC-LC, 25 for hierarchical HE-AAC and 37 for
    /// the backward-compatible form).
    pub fn new(asc: Vec<u8>, asc_bits: u32, mux_interval: u32) -> Result<Self> {
        if asc.is_empty() || asc_bits == 0 || asc_bits as usize > asc.len() * 8 {
            return Err(Error::EncoderInvalidConfig);
        }
        Ok(LoasWriter {
            asc,
            asc_bits,
            mux_interval: mux_interval.max(1),
            frames: 0,
        })
    }

    /// The Table 1.42 `StreamMuxConfig()` for this stream.
    fn write_stream_mux_config(&self, w: &mut BitWriter) {
        w.write_bit(true); // audioMuxVersion = 1
        w.write_bit(false); // audioMuxVersionA = 0
        write_latm_value(w, 0xFF); // taraBufferFullness (unconstrained)
        w.write_bit(true); // allStreamsSameTimeFraming
        w.write_u32(0, 6); // numSubFrames = 0
        w.write_u32(0, 4); // numProgram = 0
        w.write_u32(0, 3); // numLayer = 0
                           // ascLen = LatmGetValue(); AudioSpecificConfig(); fillBits.
        write_latm_value(w, self.asc_bits);
        let mut left = self.asc_bits as usize;
        for &b in &self.asc {
            let take = left.min(8);
            if take == 0 {
                break;
            }
            w.write_u32(u32::from(b) >> (8 - take), take as u32);
            left -= take;
        }
        w.write_u32(0, 3); // frameLengthType = 0
        w.write_u32(0xFF, 8); // latmBufferFullness (unconstrained)
        w.write_bit(false); // otherDataPresent
        w.write_bit(false); // crcCheckPresent
    }

    /// Wrap one `raw_data_block()` payload into a complete LOAS sync
    /// frame (`0x2B7` + length + `AudioMuxElement(1)`).
    pub fn write_frame(&mut self, payload: &[u8]) -> Result<Vec<u8>> {
        let mut w = BitWriter::new();
        let send_config = self.frames % u64::from(self.mux_interval) == 0;
        w.write_bit(!send_config); // useSameStreamMux
        if send_config {
            self.write_stream_mux_config(&mut w);
        }
        // PayloadLengthInfo(): MuxSlotLengthBytes, 255-escaped.
        let mut n = payload.len();
        while n >= 255 {
            w.write_u32(255, 8);
            n -= 255;
        }
        w.write_u32(n as u32, 8);
        // PayloadMux().
        for &b in payload {
            w.write_u32(u32::from(b), 8);
        }
        w.align_to_byte_zero();
        let element = w.finish();
        if element.len() >= 1 << 13 {
            return Err(Error::EncoderFrameOverflow);
        }
        self.frames += 1;

        let mut out = BitWriter::with_capacity(element.len() + 3);
        out.write_u32(LOAS_SYNCWORD, 11);
        out.write_u32(element.len() as u32, 13);
        out.write_bytes(&element);
        Ok(out.finish())
    }

    /// Convert a whole ADTS stream into a LOAS stream: every ADTS
    /// frame's `raw_data_block()` becomes one sync frame.
    pub fn wrap_adts_stream(&mut self, adts: &[u8]) -> Result<Vec<u8>> {
        let mut out = Vec::with_capacity(adts.len() + adts.len() / 32 + 64);
        for (_, payload) in AdtsPayloads::new(adts) {
            out.extend_from_slice(&self.write_frame(payload)?);
        }
        Ok(out)
    }
}

/// Iterator over the `(header, raw_data_block)` pairs of an ADTS
/// stream ([`strip_adts`] is the collecting convenience).
#[derive(Debug)]
pub struct AdtsPayloads<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> AdtsPayloads<'a> {
    /// Walk `data` from its first ADTS header.
    pub fn new(data: &'a [u8]) -> Self {
        AdtsPayloads { data, pos: 0 }
    }
}

impl<'a> Iterator for AdtsPayloads<'a> {
    type Item = (crate::adts::AdtsHeader, &'a [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        let rest = &self.data[self.pos.min(self.data.len())..];
        let (header, header_len) = crate::adts::AdtsHeader::parse(rest).ok()?;
        let frame_len = usize::from(header.aac_frame_length);
        if frame_len <= header_len || frame_len > rest.len() {
            return None;
        }
        let payload = &rest[header_len..frame_len];
        self.pos += frame_len;
        Some((header, payload))
    }
}

/// The `raw_data_block()` payloads of an ADTS stream.
pub fn strip_adts(adts: &[u8]) -> Vec<Vec<u8>> {
    AdtsPayloads::new(adts).map(|(_, p)| p.to_vec()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::latm::AudioSyncStream;

    #[test]
    fn latm_value_widths() {
        let mut w = BitWriter::new();
        write_latm_value(&mut w, 0x25);
        write_latm_value(&mut w, 0x1234);
        let bytes = w.finish();
        let mut r = oxideav_core::bits::BitReader::new(&bytes);
        assert_eq!(crate::latm::latm_get_value(&mut r).unwrap(), 0x25);
        assert_eq!(crate::latm::latm_get_value(&mut r).unwrap(), 0x1234);
    }

    /// A LOAS stream written around AAC-LC payloads parses back:
    /// config in frame 0, `useSameStreamMux` after, payload bytes
    /// intact, ASC fields as written.
    #[test]
    fn loas_frames_round_trip_through_the_parser() {
        let asc = crate::asc_writer::aac_lc_asc(48_000, 2);
        let mut wtr = LoasWriter::new(asc, 16, 4).unwrap();
        let payloads: Vec<Vec<u8>> = (0..6u8)
            .map(|i| {
                (0..40 + usize::from(i) * 100)
                    .map(|j| (j as u8) ^ i)
                    .collect()
            })
            .collect();
        let mut stream = Vec::new();
        for p in &payloads {
            stream.extend_from_slice(&wtr.write_frame(p).unwrap());
        }
        let mut walker = AudioSyncStream::new(&stream);
        for (i, expect) in payloads.iter().enumerate() {
            let frame = walker.next_frame().unwrap().expect("frame");
            assert_eq!(frame.element.use_same_stream_mux, i % 4 != 0);
            let cfg = &frame.element.config;
            assert_eq!(cfg.audio_mux_version, 1);
            let asc = &cfg.layers[0].effective_asc;
            assert_eq!(asc.aot, 2);
            assert_eq!(asc.sample_rate, 48_000);
            assert_eq!(frame.element.payloads.len(), 1);
            assert_eq!(&frame.element.payloads[0].data, expect);
        }
        assert!(walker.next_frame().unwrap().is_none());
    }

    /// The backward-compatible HE-AAC ASC (37 bits, padded to 5 bytes)
    /// survives the length-prefixed muxVersion-1 carriage: the parser
    /// resolves the SBR probe and reports the doubled rate.
    #[test]
    fn he_aac_asc_with_trailer_is_probed_in_latm() {
        let asc = crate::asc_writer::he_aac_v1_asc(22_050, 44_100, 2, false);
        let mut wtr = LoasWriter::new(asc, 37, 8).unwrap();
        let stream = wtr.write_frame(&[0xAA; 25]).unwrap();
        let mut walker = AudioSyncStream::new(&stream);
        let frame = walker.next_frame().unwrap().expect("frame");
        let asc = &frame.element.config.layers[0].effective_asc;
        assert_eq!(asc.aot, 2);
        assert_eq!(asc.sample_rate, 22_050);
        let probe = asc.trailing_sbr_probe.as_ref().expect("SBR probe");
        assert!(probe.sbr_present_flag);
        assert_eq!(probe.extension_sample_rate, Some(44_100));
    }

    #[test]
    fn strip_adts_extracts_raw_blocks() {
        let mut enc = crate::encoder::StreamEncoder::new(crate::encoder::EncoderConfig {
            sample_rate: 44_100,
            channels: 1,
            bitrate: 64_000,
        })
        .unwrap();
        let pcm: Vec<i16> = (0..3 * 1024).map(|i| ((i * 7) % 251) as i16).collect();
        let adts = enc.encode_all(&pcm).unwrap();
        let payloads = strip_adts(&adts);
        assert_eq!(payloads.len(), 4);
        let total: usize = payloads.iter().map(|p| p.len() + 7).sum();
        assert_eq!(total, adts.len());
    }
}

//! LATM (Low-overhead MPEG-4 Audio Transport Multiplex) syntax parser —
//! ISO/IEC 14496-3 §1.7.3.
//!
//! LATM defines a multiplex layer that wraps one or more AAC sub-streams
//! together with their `StreamMuxConfig` (which itself embeds an
//! `AudioSpecificConfig`). The actual AAC `raw_data_block`s (or other
//! payload — CELP, HVXC, etc.) live in `PayloadMux()` blocks; their
//! lengths are signalled by `PayloadLengthInfo()`.
//!
//! Only the AAC-payload (`frameLengthType == 0`) common case is handled
//! by [`AudioMuxElement::parse`]. Other frame-length types (CELP / HVXC
//! tables, fixed-length frames) are surfaced as `Error::Unsupported`.
//!
//! The wire format for the most-common single-program / single-layer /
//! `audioMuxVersion=0` shape is laid out as:
//!
//! ```text
//! audio_mux_element(muxConfigPresent=1) {
//!     useSameStreamMux                1 bit
//!     if (!useSameStreamMux)
//!         StreamMuxConfig()
//!     // frameLengthType=0:
//!     PayloadLengthInfo()             // 8-bit increments terminated by !=0xFF
//!     PayloadMux()                    // MuxSlotLengthBytes bytes (byte-aligned
//!                                     // when allStreamsSameTimeFraming=1)
//!     // otherData / crc tail
//!     byte_alignment
//! }
//! ```

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::asc::{parse_asc, AudioSpecificConfig};

/// One AAC access unit extracted from a LATM `audio_mux_element`,
/// together with the `AudioSpecificConfig` that should be used to
/// decode it.
#[derive(Clone, Debug)]
pub struct LatmFrame {
    /// AAC `raw_data_block` payload bytes. Already byte-aligned and
    /// stripped of any LATM framing.
    pub payload: Vec<u8>,
    /// Stream config in effect for this payload. When the bitstream
    /// re-uses the previous frame's config (`useSameStreamMux=1`) the
    /// caller's stored config is echoed here.
    pub asc: AudioSpecificConfig,
}

/// Persistent LATM parser state. Tracks the most-recently-decoded
/// `StreamMuxConfig` so that frames marked `useSameStreamMux=1` can
/// reuse it without re-reading the wire bytes.
#[derive(Clone, Debug, Default)]
pub struct LatmContext {
    /// AudioSpecificConfig bytes captured from the last
    /// `StreamMuxConfig` (so we can re-parse / re-export them).
    pub asc_bytes: Vec<u8>,
    /// Cached parse of `asc_bytes`.
    pub asc: Option<AudioSpecificConfig>,
    /// `frameLengthType` from the most-recent layer 0 (only `0` is
    /// supported by the decoder path; we still record it for diagnosis).
    pub frame_length_type: u8,
    /// `audioMuxVersion` from the most-recent StreamMuxConfig.
    pub audio_mux_version: u8,
    /// `audioMuxVersionA` — must be 0 in the conforming subset of
    /// 14496-3 (sub-clause 1.7.3.1 reserves 1 for ISO future use).
    pub audio_mux_version_a: u8,
    /// `numSubFrames + 1` from the most-recent StreamMuxConfig — how
    /// many AAC payloads are packed into each `audio_mux_element`.
    pub num_sub_frames: u8,
}

impl LatmContext {
    pub fn new() -> Self {
        Self::default()
    }

    /// True iff a `StreamMuxConfig` has been parsed at least once and
    /// the cached `asc` is ready to be used by `useSameStreamMux=1`
    /// frames.
    pub fn is_configured(&self) -> bool {
        self.asc.is_some()
    }
}

/// Parse the value emitted by `LatmGetValue()` (§1.7.3.1):
/// 2-bit length-of-bytes prefix N, followed by `(N+1) * 8` bits of value.
fn read_latm_get_value(br: &mut BitReader<'_>) -> Result<u32> {
    let bytes_for_value = br.read_u32(2)? as usize;
    let mut value: u32 = 0;
    for _ in 0..=bytes_for_value {
        value = (value << 8) | br.read_u32(8)?;
    }
    Ok(value)
}

/// Parse `StreamMuxConfig()` (§1.7.3.1) — single-program / single-layer
/// AAC subset. Returns the captured `AudioSpecificConfig` bytes plus
/// the parsed view, and updates `ctx` in place.
///
/// On entry `br` is positioned just past `useSameStreamMux=0`.
fn parse_stream_mux_config(br: &mut BitReader<'_>, ctx: &mut LatmContext) -> Result<()> {
    let audio_mux_version = br.read_u32(1)? as u8;
    let audio_mux_version_a = if audio_mux_version == 1 {
        br.read_u32(1)? as u8
    } else {
        0
    };
    if audio_mux_version_a != 0 {
        return Err(Error::unsupported(
            "LATM: audioMuxVersionA != 0 reserved for future ISO use",
        ));
    }
    ctx.audio_mux_version = audio_mux_version;
    ctx.audio_mux_version_a = audio_mux_version_a;

    if audio_mux_version == 1 {
        // taraBufferFullness — discard.
        let _ = read_latm_get_value(br)?;
    }

    let _all_streams_same_time_framing = br.read_bit()?;
    let num_sub_frames = br.read_u32(6)? as u8;
    let num_program = br.read_u32(4)? as u8;
    if num_program != 0 {
        // Two or more programs is permitted by the spec, but extremely
        // rare in the wild and would multiply the per-frame
        // bookkeeping; reject it explicitly so the caller knows.
        return Err(Error::unsupported(
            "LATM: numProgram > 0 (multi-program StreamMuxConfig)",
        ));
    }
    let num_layer = br.read_u32(3)? as u8;
    if num_layer != 0 {
        return Err(Error::unsupported(
            "LATM: numLayer > 0 (scalable layered config)",
        ));
    }
    ctx.num_sub_frames = num_sub_frames + 1;

    // First (and only) layer of the first (and only) program. Per the
    // spec there is no `useSameConfig` flag for the very first layer.
    // The AudioSpecificConfig follows.
    let asc_bytes = if audio_mux_version == 0 {
        // 14496-3 §1.7.3.1 says: "For audioMuxVersion==0, the
        // AudioSpecificConfig is read directly from the bitstream and
        // its length is given by the next-occurring syntactic element
        // (frameLengthType, etc.)." We need to know how many bits the
        // ASC consumes — `parse_asc` reads from a byte buffer, so we
        // must materialise it. We capture the current bit position,
        // read the ASC into a temporary, and then re-read the bits
        // we just consumed back into a Vec<u8>.
        capture_asc_inline(br)?
    } else {
        // audioMuxVersion==1: explicit length in bits (LatmGetValue),
        // then exactly that many bits of ASC.
        let asc_len_bits = read_latm_get_value(br)? as usize;
        capture_asc_explicit(br, asc_len_bits)?
    };

    // Re-parse the captured ASC bytes for the public view.
    let asc = parse_asc(&asc_bytes)?;
    ctx.asc_bytes = asc_bytes;
    ctx.asc = Some(asc);

    // frameLengthType — only 0 (AAC payload, byte-length signalled by
    // PayloadLengthInfo) is supported.
    let frame_length_type = br.read_u32(3)? as u8;
    ctx.frame_length_type = frame_length_type;
    match frame_length_type {
        0 => {
            // latmBufferFullness — 8 bits, ignored.
            let _ = br.read_u32(8)?;
            // For non-scalable AAC the spec stops here for this layer;
            // there is no `coreFrameOffset` field unless the underlying
            // object type is AAC scalable / CELP / HVXC.
        }
        1 => {
            // frameLength: 9 bits, units of (256/8) raw_data_block samples.
            // CELP / HVXC don't have raw_data_block tail and we don't
            // implement those decoders anyway.
            return Err(Error::unsupported(
                "LATM: frameLengthType=1 (fixed-length frames) not supported",
            ));
        }
        3..=5 => {
            return Err(Error::unsupported(
                "LATM: frameLengthType in {3,4,5} (CELP) not supported",
            ));
        }
        6 | 7 => {
            return Err(Error::unsupported(
                "LATM: frameLengthType in {6,7} (HVXC) not supported",
            ));
        }
        _ => {
            return Err(Error::unsupported("LATM: reserved frameLengthType value"));
        }
    }

    let other_data_present = br.read_bit()?;
    if other_data_present {
        if audio_mux_version == 1 {
            let _ = read_latm_get_value(br)?;
        } else {
            // 8-bit increments, gated by an `other_data_len_esc` flag at
            // each step (sub-clause 1.7.3.1).
            loop {
                let esc = br.read_bit()?;
                let _ = br.read_u32(8)?;
                if !esc {
                    break;
                }
            }
        }
    }
    let crc_check_present = br.read_bit()?;
    if crc_check_present {
        let _ = br.read_u32(8)?;
    }
    Ok(())
}

/// Capture an `AudioSpecificConfig` whose length is implicit (defined
/// by its own internal syntax). We rewind the reader to the ASC start,
/// parse it once to learn the consumed bit count, then materialise that
/// many bits back into a fresh `Vec<u8>`.
fn capture_asc_inline(br: &mut BitReader<'_>) -> Result<Vec<u8>> {
    let start_bit = br.bit_position();
    // Probe-parse: we re-construct a fresh BitReader rooted at the
    // *original* buffer so we can measure how many bits the ASC
    // consumes without disturbing `br`.
    //
    // The `BitReader` doesn't expose the underlying slice, so we drain
    // the live `br` byte-by-byte into a scratch buffer aligned to its
    // current bit boundary, then run `parse_asc` over that scratch and
    // measure how far it got. We pessimistically grab 64 bytes which is
    // more than enough for any realistic ASC (an SBR/PS-extended ASC
    // tops out under 16 bytes).
    //
    // To stay zero-copy on the live reader we use a snapshot:
    let snapshot = *br;
    let scratch = drain_aligned_bytes(snapshot, 64)?;
    let mut probe = BitReader::new(&scratch);
    parse_asc_into(&mut probe)?;
    let consumed_bits = probe.bit_position() as usize;

    // Now read exactly `consumed_bits` from the live `br` into a Vec<u8>.
    let captured = read_bits_into_bytes(br, consumed_bits)?;
    debug_assert_eq!(br.bit_position() - start_bit, consumed_bits as u64);
    Ok(captured)
}

fn capture_asc_explicit(br: &mut BitReader<'_>, asc_len_bits: usize) -> Result<Vec<u8>> {
    read_bits_into_bytes(br, asc_len_bits)
}

/// Drain up to `n` bytes from a snapshot of `br`, padding with zero
/// where the underlying buffer ends. Used to feed `parse_asc`'s probe.
fn drain_aligned_bytes(mut br: BitReader<'_>, n: usize) -> Result<Vec<u8>> {
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        match br.read_u32(8) {
            Ok(b) => out.push(b as u8),
            Err(_) => out.push(0),
        }
    }
    Ok(out)
}

/// Read exactly `n` bits from `br` and pack them into a `Vec<u8>`,
/// MSB-first within each byte. The trailing byte is zero-padded if
/// `n` is not a multiple of 8.
fn read_bits_into_bytes(br: &mut BitReader<'_>, n: usize) -> Result<Vec<u8>> {
    let mut out = vec![0u8; n.div_ceil(8)];
    let full_bytes = n / 8;
    let tail_bits = (n % 8) as u32;
    for byte in out.iter_mut().take(full_bytes) {
        *byte = br.read_u32(8)? as u8;
    }
    if tail_bits != 0 {
        let v = br.read_u32(tail_bits)? as u8;
        out[full_bytes] = v << (8 - tail_bits);
    }
    Ok(out)
}

/// Mirror of [`crate::asc::parse_asc`] that operates on a `BitReader`
/// (so we can measure how many bits it consumed). Returning the parsed
/// view is unnecessary here — the caller only cares about the bit
/// position afterwards — but we surface it so the helper stays useful.
fn parse_asc_into(br: &mut BitReader<'_>) -> Result<()> {
    use crate::syntax::{sample_rate, AOT_PS, AOT_SBR};
    fn read_aot(br: &mut BitReader<'_>) -> Result<u8> {
        let mut aot = br.read_u32(5)? as u8;
        if aot == 31 {
            aot = (32 + br.read_u32(6)?) as u8;
        }
        Ok(aot)
    }
    fn read_sf(br: &mut BitReader<'_>) -> Result<u32> {
        let idx = br.read_u32(4)? as u8;
        if idx == 0xF {
            br.read_u32(24)
        } else {
            sample_rate(idx).ok_or_else(|| Error::invalid("LATM ASC: reserved SF index"))
        }
    }
    let mut aot = read_aot(br)?;
    let _sf = read_sf(br)?;
    let _ch_cfg = br.read_u32(4)?;
    if aot == AOT_SBR || aot == AOT_PS {
        let _ext_sf = read_sf(br)?;
        aot = read_aot(br)?;
    }
    // GASpecificConfig (§4.4.1): for AAC-LC the next bits are
    //   frameLengthFlag           1
    //   dependsOnCoreCoder        1
    //   if (depends) coreCoderDelay  14
    //   extensionFlag             1
    //   if (channel_config==0) program_config_element()  (we don't model)
    //   if (extensionFlag) ... (object-type dependent)
    // We model the cheap tail to track the ASC bit-length faithfully.
    if matches!(aot, 1 | 2 | 3 | 4 | 6 | 7 | 17 | 19 | 20 | 21 | 22 | 23) {
        let _frame_length_flag = br.read_bit()?;
        let depends_on_core = br.read_bit()?;
        if depends_on_core {
            let _core_coder_delay = br.read_u32(14)?;
        }
        let extension_flag = br.read_bit()?;
        // channel_config==0 case omitted — LATM streams in the wild
        // always carry an explicit channel config.
        // §4.4.1.1: layerNr is signalled for AAC-scalable (6) and ER
        // AAC-scalable (20).
        if matches!(aot, 6 | 20) {
            let _layer_nr = br.read_u32(3)?;
        }
        if extension_flag {
            // §4.4.1.1: extensionFlag tail is object-type-dependent.
            //   AOT 22:        numOfSubFrame(5) + layer_length(11)
            //   AOTs 17/19/20/23: 3 resilience flag bits
            //   then extensionFlag3 (1 bit).
            // For pure AAC-LC (AOT 2) the spec says extensionFlag is
            // reserved-zero, but defensively we still read the
            // extensionFlag3 trailer if it ever shows up.
            if aot == 22 {
                let _ = br.read_u32(5)?;
                let _ = br.read_u32(11)?;
            } else if matches!(aot, 17 | 19 | 20 | 23) {
                let _ = br.read_u32(3)?;
            }
            let _extension_flag3 = br.read_bit()?;
        }
    }
    Ok(())
}

/// Result of one `audio_mux_element` parse — zero or more AAC access
/// units (one per sub-frame), each tagged with the active ASC.
#[derive(Clone, Debug)]
pub struct AudioMuxElement {
    pub frames: Vec<LatmFrame>,
}

impl AudioMuxElement {
    /// Parse one `audio_mux_element` from `data`. `data` must be the
    /// payload of exactly one LOAS frame (`audioMuxLengthBytes` bytes
    /// captured by `LoasFrame::parse`); on success the returned
    /// `AudioMuxElement` carries one `LatmFrame` per sub-frame.
    ///
    /// `ctx` is updated with any new `StreamMuxConfig` and re-used by
    /// later calls when the bitstream sets `useSameStreamMux=1`.
    pub fn parse(data: &[u8], ctx: &mut LatmContext) -> Result<Self> {
        let mut br = BitReader::new(data);
        // muxConfigPresent is fixed to 1 for AudioSyncStream (LOAS) per
        // §1.7.2 / §1.7.3.
        let use_same_stream_mux = br.read_bit()?;
        if !use_same_stream_mux {
            parse_stream_mux_config(&mut br, ctx)?;
        }
        if !ctx.is_configured() {
            return Err(Error::invalid(
                "LATM: useSameStreamMux=1 before any StreamMuxConfig",
            ));
        }
        if ctx.audio_mux_version_a != 0 {
            return Err(Error::unsupported(
                "LATM: audioMuxVersionA!=0 not supported",
            ));
        }
        if ctx.frame_length_type != 0 {
            return Err(Error::unsupported(
                "LATM: only frameLengthType=0 (AAC payload) supported",
            ));
        }

        let mut frames = Vec::with_capacity(ctx.num_sub_frames as usize);
        for _sub in 0..ctx.num_sub_frames {
            // PayloadLengthInfo() — 8-bit increments terminated by the
            // first chunk that is NOT 0xFF.
            let mut mux_slot_length_bytes: usize = 0;
            loop {
                let chunk = br.read_u32(8)? as usize;
                mux_slot_length_bytes += chunk;
                if chunk != 255 {
                    break;
                }
            }
            // PayloadMux — `mux_slot_length_bytes` bytes of AAC payload.
            // The bits we read here are not necessarily byte-aligned
            // because the StreamMuxConfig before us doesn't have to land
            // on a byte boundary. We read bit-by-bit through u32(8).
            let mut payload = Vec::with_capacity(mux_slot_length_bytes);
            for _ in 0..mux_slot_length_bytes {
                payload.push(br.read_u32(8)? as u8);
            }
            frames.push(LatmFrame {
                payload,
                asc: ctx.asc.clone().unwrap(),
            });
        }
        // otherDataPresent / CRC tail belong to the StreamMuxConfig, not
        // here — anything left in `data` is the LOAS-level byte-alignment
        // padding and we ignore it.
        Ok(AudioMuxElement { frames })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    fn build_minimal_audio_mux_element() -> Vec<u8> {
        let mut bw = BitWriter::new();
        // useSameStreamMux=0 → expect StreamMuxConfig
        bw.write_bit(false);
        // audioMuxVersion=0
        bw.write_bit(false);
        // allStreamsSameTimeFraming=1
        bw.write_bit(true);
        // numSubFrames=0 (one sub-frame)
        bw.write_u32(0, 6);
        // numProgram=0
        bw.write_u32(0, 4);
        // numLayer=0
        bw.write_u32(0, 3);
        // ASC: AAC-LC, 44100 (idx=4), stereo, frameLengthFlag=0,
        // dependsOnCoreCoder=0, extensionFlag=0
        bw.write_u32(2, 5); // AOT
        bw.write_u32(4, 4); // SF index
        bw.write_u32(2, 4); // channel config
        bw.write_bit(false); // frameLengthFlag
        bw.write_bit(false); // dependsOnCoreCoder
        bw.write_bit(false); // extensionFlag
                             // frameLengthType=0
        bw.write_u32(0, 3);
        // latmBufferFullness=0xFF
        bw.write_u32(0xFF, 8);
        // otherDataPresent=0
        bw.write_bit(false);
        // crcCheckPresent=0
        bw.write_bit(false);
        // PayloadLengthInfo: one byte =5 → MuxSlotLengthBytes=5
        bw.write_u32(5, 8);
        // PayloadMux: 5 dummy bytes
        for b in [0xDE, 0xAD, 0xBE, 0xEF, 0x42] {
            bw.write_u32(b, 8);
        }
        bw.align_to_byte();
        bw.finish()
    }

    #[test]
    fn parses_minimal_lc_stereo() {
        let data = build_minimal_audio_mux_element();
        let mut ctx = LatmContext::new();
        let ame = AudioMuxElement::parse(&data, &mut ctx).expect("parse");
        assert!(ctx.is_configured());
        assert_eq!(ctx.num_sub_frames, 1);
        assert_eq!(ame.frames.len(), 1);
        let f = &ame.frames[0];
        assert_eq!(f.asc.object_type, crate::syntax::AOT_AAC_LC);
        assert_eq!(f.asc.sampling_frequency, 44_100);
        assert_eq!(f.asc.channel_configuration, 2);
        assert_eq!(f.payload, vec![0xDE, 0xAD, 0xBE, 0xEF, 0x42]);
    }

    #[test]
    fn second_frame_uses_same_stream_mux() {
        // First frame establishes config; second frame sets
        // useSameStreamMux=1 and just carries a payload.
        let first = build_minimal_audio_mux_element();
        let mut ctx = LatmContext::new();
        AudioMuxElement::parse(&first, &mut ctx).expect("first");

        let mut bw = BitWriter::new();
        bw.write_bit(true); // useSameStreamMux
        bw.write_u32(3, 8); // PayloadLengthInfo => 3 bytes
        for b in [0x11, 0x22, 0x33] {
            bw.write_u32(b, 8);
        }
        bw.align_to_byte();
        let second = bw.finish();
        let ame = AudioMuxElement::parse(&second, &mut ctx).expect("second");
        assert_eq!(ame.frames.len(), 1);
        assert_eq!(ame.frames[0].payload, vec![0x11, 0x22, 0x33]);
        assert_eq!(ame.frames[0].asc.sampling_frequency, 44_100);
    }

    #[test]
    fn rejects_use_same_before_any_config() {
        let mut bw = BitWriter::new();
        bw.write_bit(true); // useSameStreamMux=1, no config seen yet
        bw.align_to_byte();
        let data = bw.finish();
        let mut ctx = LatmContext::new();
        assert!(AudioMuxElement::parse(&data, &mut ctx).is_err());
    }
}

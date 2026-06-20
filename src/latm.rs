//! LATM / LOAS transport framing — ISO/IEC 14496-3 §1.7.
//!
//! LATM (Low-overhead MPEG-4 Audio Transport Multiplex) is the
//! multiplex layer that packs one or more MPEG-4 Audio payloads plus
//! their [`AudioSpecificConfig`] (ASC) into a single multiplexed
//! element ([`AudioMuxElement`], §1.7.3.1 Table 1.41). LOAS
//! (Low Overhead Audio Stream) is the synchronization layer above it
//! ([`AudioSyncStream`], §1.7.2.1 Table 1.36), which prefixes each
//! multiplexed element with a `0x2B7` syncword and a 13-bit byte
//! length so the multiplex can be recovered from a transmission
//! channel that carries no framing of its own.
//!
//! This module decodes the transport structure end to end for the AAC
//! case — the configuration ([`StreamMuxConfig`], Table 1.42), the
//! per-subframe payload lengths ([`PayloadLengthInfo`], Table 1.44),
//! and the multiplexed AAC access units ([`PayloadMux`], Table 1.45) —
//! and hands the recovered raw-data-block byte slices to the
//! [`crate::decode::StreamDecoder`] / [`crate::raw_data_block`] layer.
//!
//! ## Scope
//!
//! The decode path supports the configurations that carry AAC:
//! `audioMuxVersion ∈ {0, 1}` (the `audioMuxVersion == 1`
//! `taraBufferFullness` / per-ASC length-prefix extensions are parsed),
//! `allStreamsSameTimeFraming` in both states, and the per-layer
//! `frameLengthType` values `0` (variable-length, byte count carried
//! in `PayloadLengthInfo()`) and `1` (fixed `frameLength` bits in
//! `StreamMuxConfig()`). The CELP (`3`/`4`/`5`) and HVXC (`6`/`7`)
//! frame-length-table-indexed types are surfaced as
//! [`Error::LatmUnsupportedFrameLengthType`] — they index frame-length
//! tables for object types this AAC-focused crate does not decode. The
//! `audioMuxVersionA == 1` reserved branch is
//! [`Error::LatmAudioMuxVersionAReserved`]. The `EPMuxElement()`
//! error-protected variant (Table 1.40) and the
//! `EPAudioSyncStream()` FEC header (Table 1.37) are parsed at the
//! framing level but the EP-tool payload de-interleave is out of
//! scope.

use crate::asc::AudioSpecificConfig;
use crate::crc;
use crate::{Error, Result};
use oxideav_core::bits::BitReader;

/// §1.7.2.1 Table 1.36 `AudioSyncStream()` syncword (`0x2B7`, 11 bits).
pub const AUDIO_SYNC_STREAM_SYNCWORD: u32 = 0x2B7;

/// §1.7.2.1 Table 1.37 `EPAudioSyncStream()` syncword (`0x4DE1`,
/// 16 bits).
pub const EP_AUDIO_SYNC_STREAM_SYNCWORD: u32 = 0x4DE1;

/// §1.7.2.2.1: "The maximum byte-distance between two syncwords is
/// 8192 bytes", encoded in the 13-bit `audioMuxLengthBytes` field.
pub const MAX_AUDIO_MUX_LENGTH_BYTES: u32 = (1 << 13) - 1;

/// §1.7.3 signalling caps: `numProgram` is 4-bit (max program index
/// 15), `numLayer` is 3-bit (max layer index 7), `streamIndx` is
/// 4-bit (max 15 streams), `numChunk` is 4-bit.
const MAX_PROGRAM_INDEX: u32 = 15;
const MAX_LAYER_INDEX: u32 = 7;
const MAX_STREAM_COUNT: usize = 16;

/// One decoded scalable layer of a [`StreamMuxConfig`] program.
///
/// Mirrors the per-`streamID[prog][lay]` state the Table 1.42 loop
/// builds: the parsed [`AudioSpecificConfig`] (or `None` when
/// `useSameConfig` pointed at an earlier layer's config), the
/// `frameLengthType`, and the framing parameter that type selects
/// (`latmBufferFullness` for type 0, `frameLength` bits for type 1).
#[derive(Debug, Clone)]
pub struct LayerConfig {
    /// `progSIndx` — the program this layer belongs to.
    pub prog: u8,
    /// `laySIndx` — the layer index within the program.
    pub lay: u8,
    /// `streamID[prog][lay]` — the flat stream counter assigned in
    /// transmission order.
    pub stream_id: u8,
    /// The layer's [`AudioSpecificConfig`]. `None` ⇔ `useSameConfig`
    /// was set, meaning "apply the ASC most recently transmitted in a
    /// previous layer or program" (§1.7.3.2.3). [`StreamMuxConfig`]
    /// resolves this into [`LayerConfig::effective_asc`] on parse, so
    /// callers always have a concrete config there.
    pub asc: Option<AudioSpecificConfig>,
    /// The effective ASC after resolving `useSameConfig` back to the
    /// most recently transmitted config. Always populated.
    pub effective_asc: AudioSpecificConfig,
    /// `frameLengthType[streamID]` (§1.7.3.1 Table 1.42).
    pub frame_length_type: u8,
    /// `latmBufferFullness[streamID]` — present (8-bit) only for
    /// `frameLengthType == 0`.
    pub latm_buffer_fullness: Option<u8>,
    /// `coreFrameOffset` — present (6-bit) only for
    /// `frameLengthType == 0`, `!allStreamsSameTimeFraming`, and a
    /// CELP-core / AAC-enhancement layer pairing.
    pub core_frame_offset: Option<u8>,
    /// `frameLength[streamID]` — present (9-bit) only for
    /// `frameLengthType == 1`. The fixed payload length is
    /// `(frameLength + 20) * 8` bits per §1.7.3.2.3.
    pub frame_length: Option<u16>,
}

impl LayerConfig {
    /// §1.7.3.2.3: for `frameLengthType == 1` the fixed payload bit
    /// length is `(frameLength + 20) * 8`. Returns `None` for every
    /// other frame-length type (their length is carried in
    /// `PayloadLengthInfo()` or is table-indexed).
    pub fn fixed_payload_bits(&self) -> Option<u32> {
        if self.frame_length_type == 1 {
            self.frame_length
                .map(|fl| (u32::from(fl) + 20).saturating_mul(8))
        } else {
            None
        }
    }
}

/// Decoded `StreamMuxConfig()` — ISO/IEC 14496-3 §1.7.3.1 Table 1.42.
///
/// Carries the whole multiplex configuration: the version flags, the
/// time-framing mode, the per-program / per-layer [`LayerConfig`]
/// table, the `otherData` length, and the optional `crcCheckSum`.
#[derive(Debug, Clone)]
pub struct StreamMuxConfig {
    /// `audioMuxVersion` (1 bit).
    pub audio_mux_version: u8,
    /// `audioMuxVersionA` (1 bit; `0` unless `audioMuxVersion == 1`
    /// signalled it). A `1` here is the reserved `/* tbd */` branch,
    /// rejected on parse.
    pub audio_mux_version_a: u8,
    /// `taraBufferFullness` — present only for `audioMuxVersion == 1`.
    pub tara_buffer_fullness: Option<u32>,
    /// `allStreamsSameTimeFraming` (1 bit).
    pub all_streams_same_time_framing: bool,
    /// `numSubFrames` (6 bits). `numSubFrames + 1` PayloadMux frames
    /// are multiplexed.
    pub num_sub_frames: u8,
    /// `numProgram` (4 bits). `numProgram + 1` programs.
    pub num_program: u8,
    /// `numLayer[prog]` (3 bits) for each program — `num_layer[p] + 1`
    /// layers in program `p`.
    pub num_layer: Vec<u8>,
    /// The flat per-stream layer table, in transmission order.
    pub layers: Vec<LayerConfig>,
    /// `otherDataPresent` (1 bit).
    pub other_data_present: bool,
    /// `otherDataLenBits` — the decoded length of the trailing
    /// `otherData` field (in bits). `0` when `!otherDataPresent`.
    pub other_data_len_bits: u32,
    /// `crcCheckPresent` (1 bit).
    pub crc_check_present: bool,
    /// `crcCheckSum` (8 bits) when present.
    pub crc_check_sum: Option<u8>,
}

impl StreamMuxConfig {
    /// `streamID[prog][lay]` lookup, mirroring the Table 1.42
    /// `streamID` assignment (`prog`-major, `lay`-minor flat counter).
    pub fn stream_id(&self, prog: u8, lay: u8) -> Option<u8> {
        self.layers
            .iter()
            .find(|l| l.prog == prog && l.lay == lay)
            .map(|l| l.stream_id)
    }

    /// The [`LayerConfig`] for a given flat `streamID`.
    pub fn layer(&self, stream_id: u8) -> Option<&LayerConfig> {
        self.layers.iter().find(|l| l.stream_id == stream_id)
    }

    /// Parse a `StreamMuxConfig()` from `reader` (Table 1.42).
    ///
    /// `data` is the byte slice that backs `reader` (the same slice it
    /// was constructed over); it is used only to re-read the config
    /// prefix for CRC recomputation when `crcCheckPresent` is set.
    ///
    /// The reader is positioned at the `audioMuxVersion` bit and is
    /// advanced to the bit after the configuration (the `crcCheckSum`,
    /// or the last config bit when no CRC is present). The optional
    /// `crcCheckSum` is recomputed against the configuration prefix and
    /// validated; a mismatch is [`Error::LatmCrcMismatch`].
    pub fn parse(reader: &mut BitReader<'_>, data: &[u8]) -> Result<Self> {
        let start_bit = reader.bit_position();

        let audio_mux_version = read_u8(reader, 1)?;
        let audio_mux_version_a = if audio_mux_version == 1 {
            read_u8(reader, 1)?
        } else {
            0
        };

        if audio_mux_version_a != 0 {
            // The Table 1.42 `else { /* tbd */ }` branch — no defined
            // syntax.
            return Err(Error::LatmAudioMuxVersionAReserved);
        }

        let tara_buffer_fullness = if audio_mux_version == 1 {
            Some(latm_get_value(reader)?)
        } else {
            None
        };

        let all_streams_same_time_framing = read_bit(reader)?;
        let num_sub_frames = read_u8(reader, 6)?;
        let num_program = read_u8(reader, 4)?;
        if u32::from(num_program) > MAX_PROGRAM_INDEX {
            return Err(Error::LatmConfigOutOfRange);
        }

        let mut num_layer: Vec<u8> = Vec::with_capacity(usize::from(num_program) + 1);
        let mut layers: Vec<LayerConfig> = Vec::new();
        // The "most recently transmitted" ASC, threaded across layers
        // for `useSameConfig` resolution (§1.7.3.2.3).
        let mut last_asc: Option<AudioSpecificConfig> = None;
        let mut stream_cnt: u32 = 0;

        for prog in 0..=u32::from(num_program) {
            let n_layer = read_u8(reader, 3)?;
            if u32::from(n_layer) > MAX_LAYER_INDEX {
                return Err(Error::LatmConfigOutOfRange);
            }
            num_layer.push(n_layer);

            for lay in 0..=u32::from(n_layer) {
                if stream_cnt as usize >= MAX_STREAM_COUNT {
                    return Err(Error::LatmConfigOutOfRange);
                }
                let stream_id = stream_cnt as u8;
                stream_cnt += 1;

                // useSameConfig — never present for the (0,0) layer.
                let use_same_config = if prog == 0 && lay == 0 {
                    false
                } else {
                    read_bit(reader)?
                };

                let asc = if use_same_config {
                    None
                } else if audio_mux_version == 0 {
                    // audioMuxVersion == 0: the ASC has no explicit
                    // length prefix; it is parsed in place and its
                    // bit-length is implied by the ASC syntax.
                    let asc = AudioSpecificConfig::parse_bits(reader, start_bit)?;
                    Some(asc)
                } else {
                    // audioMuxVersion == 1: `ascLen = LatmGetValue();
                    // ascLen -= AudioSpecificConfig(); fillBits(ascLen)`.
                    // The ASC is length-prefixed, so we know the exact
                    // bit bound and can apply the §1.6.5 trailing
                    // implicit-SBR probe.
                    let asc_len = latm_get_value(reader)?;
                    let asc_start = reader.bit_position();
                    let asc = AudioSpecificConfig::parse_bits_bounded(
                        reader,
                        asc_start,
                        u64::from(asc_len),
                    )?;
                    let consumed = reader.bit_position().saturating_sub(asc_start);
                    // fillBits = ascLen - (bits the ASC consumed).
                    let fill = u64::from(asc_len).saturating_sub(consumed);
                    if fill > 0 {
                        skip_bits(reader, fill)?;
                    }
                    Some(asc)
                };

                // Resolve useSameConfig into a concrete effective ASC.
                let effective_asc = if let Some(a) = &asc {
                    last_asc = Some(a.clone());
                    a.clone()
                } else {
                    last_asc.clone().ok_or(Error::LatmNoPreviousMuxConfig)?
                };

                let frame_length_type = read_u8(reader, 3)?;
                let mut latm_buffer_fullness = None;
                let mut core_frame_offset = None;
                let mut frame_length = None;

                match frame_length_type {
                    0 => {
                        latm_buffer_fullness = Some(read_u8(reader, 8)?);
                        if !all_streams_same_time_framing {
                            // The CELP-core / AAC-enhancement pairing
                            // (§1.7.3.1 Table 1.42): AOT 6/20 (AAC SSR
                            // / ER AAC Scalable) layered above AOT 8/24
                            // (CELP / ER CELP).
                            let this_aot = effective_asc.aot;
                            let prev_aot = layers.last().map(|l| l.effective_asc.aot);
                            let pairs = (this_aot == 6 || this_aot == 20)
                                && matches!(prev_aot, Some(8) | Some(24));
                            if pairs {
                                core_frame_offset = Some(read_u8(reader, 6)?);
                            }
                        }
                    }
                    1 => {
                        frame_length = Some(read_u16(reader, 9)?);
                    }
                    other => {
                        // `2` is reserved; `3`/`4`/`5` are CELP and
                        // `6`/`7` are HVXC, all table-indexed framing
                        // this AAC-focused decoder does not carry.
                        return Err(Error::LatmUnsupportedFrameLengthType(other));
                    }
                }

                layers.push(LayerConfig {
                    prog: prog as u8,
                    lay: lay as u8,
                    stream_id,
                    asc,
                    effective_asc,
                    frame_length_type,
                    latm_buffer_fullness,
                    core_frame_offset,
                    frame_length,
                });
            }
        }

        // otherDataPresent / otherDataLenBits.
        let other_data_present = read_bit(reader)?;
        let other_data_len_bits = if other_data_present {
            if audio_mux_version == 1 {
                latm_get_value(reader)?
            } else {
                // do { otherDataLenBits *= 256; esc; tmp(8);
                // otherDataLenBits += tmp; } while (esc);
                let mut acc: u32 = 0;
                loop {
                    acc = acc.wrapping_mul(256);
                    let esc = read_bit(reader)?;
                    let tmp = read_u8(reader, 8)?;
                    acc = acc.wrapping_add(u32::from(tmp));
                    if !esc {
                        break;
                    }
                }
                acc
            }
        } else {
            0
        };

        // crcCheckPresent / crcCheckSum. The CRC covers the whole
        // StreamMuxConfig() from `audioMuxVersion` up to but excluding
        // crcCheckPresent — capture that prefix before reading the
        // flag.
        let crc_end_bit = reader.bit_position();
        let crc_check_present = read_bit(reader)?;
        let crc_check_sum = if crc_check_present {
            let sum = read_u8(reader, 8)?;
            // Recompute over the config prefix and validate.
            let prefix = read_back_bits(data, start_bit, crc_end_bit)?;
            let expected = crc::stream_mux_config_crc(&prefix);
            if expected != sum {
                return Err(Error::LatmCrcMismatch);
            }
            Some(sum)
        } else {
            None
        };

        Ok(StreamMuxConfig {
            audio_mux_version,
            audio_mux_version_a,
            tara_buffer_fullness,
            all_streams_same_time_framing,
            num_sub_frames,
            num_program,
            num_layer,
            layers,
            other_data_present,
            other_data_len_bits,
            crc_check_present,
            crc_check_sum,
        })
    }
}

/// §1.7.3 signalling cap: `numChunk` is 4-bit (max chunk index 15).
const MAX_NUM_CHUNK_INDEX: u32 = 15;

/// One recovered MPEG-4 Audio payload from a [`PayloadMux`] — the raw
/// access-unit bytes for a single `(subframe, prog, lay)` slot. For an
/// AAC layer these bytes are the §4.4.2.1 `raw_data_block()` that the
/// [`crate::decode::StreamDecoder`] / [`crate::raw_data_block`] layer
/// consumes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MuxPayload {
    /// Subframe index (`0 ..= numSubFrames`).
    pub sub_frame: u8,
    /// `prog` — the program this payload belongs to.
    pub prog: u8,
    /// `lay` — the layer within the program.
    pub lay: u8,
    /// `streamID[prog][lay]`.
    pub stream_id: u8,
    /// The raw payload bytes (one complete access unit for
    /// `frameLengthType == 0`).
    pub data: Vec<u8>,
}

/// `MuxSlotLengthBytes[streamID]` decoded for one payload slot of a
/// [`PayloadLengthInfo`] (Table 1.44). For `frameLengthType == 0` this
/// is the running 8-bit-escape byte count; the bit length for
/// `frameLengthType == 1` comes from the layer's fixed `frameLength`.
#[derive(Debug, Clone, Copy)]
struct SlotLength {
    prog: u8,
    lay: u8,
    stream_id: u8,
    /// Payload length in **bits**. For type-0 this is `bytes * 8`; for
    /// type-1 it is `(frameLength + 20) * 8`.
    bits: u32,
}

/// Decoded `AudioMuxElement()` — ISO/IEC 14496-3 §1.7.3.1 Table 1.41.
///
/// Holds the (possibly inherited) [`StreamMuxConfig`] and the recovered
/// per-subframe payloads. Parsing supports `audioMuxVersionA == 0`
/// (the only defined branch) and `allStreamsSameTimeFraming` in both
/// states; non-same-time-framing uses the `numChunk` chunk layout of
/// Tables 1.44 / 1.45.
#[derive(Debug, Clone)]
pub struct AudioMuxElement {
    /// `useSameStreamMux` (only present when `muxConfigPresent`). When
    /// `true`, [`AudioMuxElement::config`] was inherited from the
    /// previous element rather than parsed here.
    pub use_same_stream_mux: bool,
    /// The active multiplex configuration for this element.
    pub config: StreamMuxConfig,
    /// The recovered payloads in transmission order.
    pub payloads: Vec<MuxPayload>,
}

impl AudioMuxElement {
    /// Parse an `AudioMuxElement()` (Table 1.41) from `reader`.
    ///
    /// `data` is the byte slice backing `reader` (forwarded to
    /// [`StreamMuxConfig::parse`] for CRC recomputation).
    /// `mux_config_present` is the `muxConfigPresent` flag the calling
    /// layer supplies (LOAS [`AudioSyncStream`] passes `1`; an
    /// out-of-band-configured transport passes `0`). `prev_config` is
    /// the configuration decoded on the previous element, used when
    /// `useSameStreamMux` is set or when `muxConfigPresent == 0`.
    pub fn parse(
        reader: &mut BitReader<'_>,
        data: &[u8],
        mux_config_present: bool,
        prev_config: Option<&StreamMuxConfig>,
    ) -> Result<Self> {
        let (use_same_stream_mux, config) = if mux_config_present {
            let use_same = read_bit(reader)?;
            if use_same {
                let cfg = prev_config.cloned().ok_or(Error::LatmNoPreviousMuxConfig)?;
                (true, cfg)
            } else {
                (false, StreamMuxConfig::parse(reader, data)?)
            }
        } else {
            // Out-of-band StreamMuxConfig(): apply the previous one.
            let cfg = prev_config.cloned().ok_or(Error::LatmNoPreviousMuxConfig)?;
            (false, cfg)
        };

        if config.audio_mux_version_a != 0 {
            return Err(Error::LatmAudioMuxVersionAReserved);
        }

        let mut payloads = Vec::new();
        for sub_frame in 0..=u32::from(config.num_sub_frames) {
            let slots = payload_length_info(reader, &config)?;
            payload_mux(reader, &config, sub_frame as u8, &slots, &mut payloads)?;
        }

        // otherData: skip otherDataLenBits bits.
        if config.other_data_present {
            skip_bits(reader, u64::from(config.other_data_len_bits))?;
        }

        // ByteAlign().
        reader.align_to_byte();

        Ok(AudioMuxElement {
            use_same_stream_mux,
            config,
            payloads,
        })
    }
}

/// `PayloadLengthInfo()` — §1.7.3.1 Table 1.44. Returns the decoded
/// per-slot payload bit-lengths in the order `PayloadMux()` will emit
/// them.
fn payload_length_info(
    reader: &mut BitReader<'_>,
    config: &StreamMuxConfig,
) -> Result<Vec<SlotLength>> {
    let mut slots = Vec::new();
    if config.all_streams_same_time_framing {
        for prog in 0..=u32::from(config.num_program) {
            let n_layer = config.num_layer[prog as usize];
            for lay in 0..=u32::from(n_layer) {
                let stream_id = config
                    .stream_id(prog as u8, lay as u8)
                    .ok_or(Error::LatmConfigOutOfRange)?;
                let layer = config.layer(stream_id).ok_or(Error::LatmConfigOutOfRange)?;
                let bits = slot_bits(reader, layer)?;
                slots.push(SlotLength {
                    prog: prog as u8,
                    lay: lay as u8,
                    stream_id,
                    bits,
                });
            }
        }
    } else {
        let num_chunk = read_u8(reader, 4)?;
        if u32::from(num_chunk) > MAX_NUM_CHUNK_INDEX {
            return Err(Error::LatmConfigOutOfRange);
        }
        for _ in 0..=u32::from(num_chunk) {
            let stream_indx = read_u8(reader, 4)?;
            let layer = config
                .layer(stream_indx)
                .ok_or(Error::LatmConfigOutOfRange)?;
            let prog = layer.prog;
            let lay = layer.lay;
            let stream_id = layer.stream_id;
            let frame_length_type = layer.frame_length_type;
            let bits = slot_bits(reader, layer)?;
            // For frameLengthType == 0 in the chunk layout the spec
            // appends an AuEndFlag bit after MuxSlotLengthBytes.
            if frame_length_type == 0 {
                let _au_end_flag = read_bit(reader)?;
            }
            slots.push(SlotLength {
                prog,
                lay,
                stream_id,
                bits,
            });
        }
    }
    Ok(slots)
}

/// Decode the payload bit-length for one slot per its
/// `frameLengthType` (Table 1.44 inner body): the 8-bit-escape running
/// `MuxSlotLengthBytes` for type 0, or the fixed `(frameLength+20)*8`
/// for type 1. CELP/HVXC `MuxSlotLengthCoded` table indices are out of
/// scope and were already rejected when the config was parsed.
fn slot_bits(reader: &mut BitReader<'_>, layer: &LayerConfig) -> Result<u32> {
    match layer.frame_length_type {
        0 => {
            let mut bytes: u32 = 0;
            loop {
                let tmp = read_u8(reader, 8)?;
                bytes = bytes.wrapping_add(u32::from(tmp));
                if tmp != 255 {
                    break;
                }
            }
            Ok(bytes.saturating_mul(8))
        }
        1 => layer
            .fixed_payload_bits()
            .ok_or(Error::LatmConfigOutOfRange),
        other => Err(Error::LatmUnsupportedFrameLengthType(other)),
    }
}

/// `PayloadMux()` — §1.7.3.1 Table 1.45. Reads each slot's payload
/// bytes in the same order `PayloadLengthInfo()` emitted them, pushing
/// one [`MuxPayload`] per slot. Payloads are byte-extracted; the spec
/// guarantees `frameLengthType == 0` payloads are an integer number of
/// bytes, and `AudioMuxElement()` byte-aligns the reader at each
/// subframe boundary in the common AAC case.
fn payload_mux(
    reader: &mut BitReader<'_>,
    config: &StreamMuxConfig,
    sub_frame: u8,
    slots: &[SlotLength],
    out: &mut Vec<MuxPayload>,
) -> Result<()> {
    // Walk in the order PayloadLengthInfo built the slots, which is the
    // same program/layer (or chunk) order PayloadMux uses.
    let _ = config;
    for slot in slots {
        let data = read_payload_bytes(reader, slot.bits)?;
        out.push(MuxPayload {
            sub_frame,
            prog: slot.prog,
            lay: slot.lay,
            stream_id: slot.stream_id,
            data,
        });
    }
    Ok(())
}

/// Read `bits` bits of payload as a byte vector. The common AAC case
/// (`frameLengthType == 0`, byte-aligned reader) is a fast `read_bytes`
/// path; a non-byte-multiple length or non-aligned reader falls back to
/// bit-by-bit assembly (MSB-first), with the trailing partial byte
/// left-justified.
fn read_payload_bytes(reader: &mut BitReader<'_>, bits: u32) -> Result<Vec<u8>> {
    if bits % 8 == 0 && reader.is_byte_aligned() {
        let n = (bits / 8) as usize;
        return reader.read_bytes(n).map_err(|_| Error::UnexpectedEnd);
    }
    let full = bits / 8;
    let rem = bits % 8;
    let mut out = Vec::with_capacity((full + u32::from(rem != 0)) as usize);
    for _ in 0..full {
        out.push(read_u8(reader, 8)?);
    }
    if rem > 0 {
        let v = read_u8(reader, rem)?;
        out.push(v << (8 - rem));
    }
    Ok(out)
}

/// §1.7.3.1 Table 1.43 `LatmGetValue()`: a variable-length unsigned
/// integer carried as `bytesForValue` (2 bits) followed by
/// `bytesForValue + 1` bytes, big-endian.
pub fn latm_get_value(reader: &mut BitReader<'_>) -> Result<u32> {
    let bytes_for_value = read_u8(reader, 2)?;
    let mut value: u32 = 0;
    for _ in 0..=u32::from(bytes_for_value) {
        value = value.wrapping_mul(256);
        let byte = read_u8(reader, 8)?;
        value = value.wrapping_add(u32::from(byte));
    }
    Ok(value)
}

// ---- bit helpers -----------------------------------------------------

fn read_u8(reader: &mut BitReader<'_>, n: u32) -> Result<u8> {
    Ok(reader.read_u32(n).map_err(|_| Error::UnexpectedEnd)? as u8)
}

fn read_u16(reader: &mut BitReader<'_>, n: u32) -> Result<u16> {
    Ok(reader.read_u32(n).map_err(|_| Error::UnexpectedEnd)? as u16)
}

fn read_bit(reader: &mut BitReader<'_>) -> Result<bool> {
    reader.read_bit().map_err(|_| Error::UnexpectedEnd)
}

fn skip_bits(reader: &mut BitReader<'_>, n: u64) -> Result<()> {
    // BitReader::skip takes a u32; chunk for safety on large fill runs.
    let mut remaining = n;
    while remaining > 0 {
        let chunk = remaining.min(u64::from(u32::MAX)) as u32;
        reader.skip(chunk).map_err(|_| Error::UnexpectedEnd)?;
        remaining -= u64::from(chunk);
    }
    Ok(())
}

/// Re-read the bits of an already-consumed `[from_bit, to_bit)` range
/// of `data` as a `Vec<bool>` in MSB-first transmission order, for CRC
/// recomputation. A fresh reader is created over the backing buffer so
/// the original reader's position is untouched.
fn read_back_bits(data: &[u8], from_bit: u64, to_bit: u64) -> Result<Vec<bool>> {
    debug_assert!(to_bit >= from_bit);
    let count = (to_bit - from_bit) as usize;
    let mut scratch = BitReader::new(data);
    skip_bits(&mut scratch, from_bit)?;
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        out.push(scratch.read_bit().map_err(|_| Error::UnexpectedEnd)?);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// AAC-LC, 44.1 kHz (samplingFrequencyIndex 4), stereo
    /// (channelConfiguration 2): AOT=2 (5 bits `00010`), freqIdx=4
    /// (`0100`), chanConfig=2 (`0010`), then GASpecificConfig
    /// `frameLengthFlag=0 dependsOnCoreCoder=0 extensionFlag=0`
    /// (`000`). 16 bits total = `0x12 0x10`.
    const AAC_LC_ASC: [u8; 2] = [0x12, 0x10];

    /// Append the §1.7.3 AAC-LC ASC bit-for-bit into `w`.
    fn write_aac_lc_asc(w: &mut BitWriter) {
        // 16 bits, MSB-first, exactly as AAC_LC_ASC encodes.
        w.write_u32(u32::from(u16::from_be_bytes(AAC_LC_ASC)), 16);
    }

    #[test]
    fn latm_get_value_single_byte() {
        // bytesForValue = 0 -> one byte. value = 0xFF.
        let mut w = BitWriter::new();
        w.write_u32(0, 2); // bytesForValue
        w.write_u32(0xFF, 8);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(latm_get_value(&mut r).unwrap(), 0xFF);
    }

    #[test]
    fn latm_get_value_multi_byte() {
        // bytesForValue = 2 -> three bytes, big-endian: 0x010203.
        let mut w = BitWriter::new();
        w.write_u32(2, 2);
        w.write_u32(0x01, 8);
        w.write_u32(0x02, 8);
        w.write_u32(0x03, 8);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(latm_get_value(&mut r).unwrap(), 0x01_02_03);
    }

    /// Build a minimal `audioMuxVersion == 0` AAC-LC StreamMuxConfig:
    /// one program, one layer, allStreamsSameTimeFraming,
    /// frameLengthType 0, latmBufferFullness 0xFF, no otherData, no
    /// CRC.
    fn build_min_smc() -> Vec<u8> {
        let mut w = BitWriter::new();
        w.write_bit(false); // audioMuxVersion = 0
        w.write_bit(true); // allStreamsSameTimeFraming = 1
        w.write_u32(0, 6); // numSubFrames = 0
        w.write_u32(0, 4); // numProgram = 0
        w.write_u32(0, 3); // numLayer = 0
                           // (prog 0, lay 0): no useSameConfig bit; ASC inline.
        write_aac_lc_asc(&mut w);
        w.write_u32(0, 3); // frameLengthType = 0
        w.write_u32(0xFF, 8); // latmBufferFullness = 0xFF
        w.write_bit(false); // otherDataPresent = 0
        w.write_bit(false); // crcCheckPresent = 0
        w.finish()
    }

    #[test]
    fn stream_mux_config_minimal_aac_lc() {
        let bytes = build_min_smc();
        let mut r = BitReader::new(&bytes);
        let smc = StreamMuxConfig::parse(&mut r, &bytes).unwrap();
        assert_eq!(smc.audio_mux_version, 0);
        assert_eq!(smc.audio_mux_version_a, 0);
        assert!(smc.all_streams_same_time_framing);
        assert_eq!(smc.num_sub_frames, 0);
        assert_eq!(smc.num_program, 0);
        assert_eq!(smc.num_layer, vec![0]);
        assert_eq!(smc.layers.len(), 1);
        let lay = &smc.layers[0];
        assert_eq!(lay.stream_id, 0);
        assert_eq!(lay.frame_length_type, 0);
        assert_eq!(lay.latm_buffer_fullness, Some(0xFF));
        assert_eq!(lay.effective_asc.aot, 2);
        assert_eq!(lay.effective_asc.sampling_frequency_index, 4);
        assert_eq!(lay.effective_asc.channel_configuration, 2);
        assert!(!smc.other_data_present);
        assert!(!smc.crc_check_present);
        assert_eq!(smc.stream_id(0, 0), Some(0));
    }

    /// Push the low `n` bits of `v` (MSB-first) onto a bool vector,
    /// mirroring `BitWriter::write_u32` so the test can hold the config
    /// prefix as bits for an independent CRC recomputation.
    fn push_bits(out: &mut Vec<bool>, v: u32, n: u32) {
        for i in (0..n).rev() {
            out.push((v >> i) & 1 == 1);
        }
    }

    #[test]
    fn stream_mux_config_with_valid_crc() {
        // Build the config prefix as a bit vector, compute its CRC, then
        // emit prefix + crcCheckPresent + crcCheckSum.
        let mut prefix: Vec<bool> = Vec::new();
        push_bits(&mut prefix, 0, 1); // audioMuxVersion = 0
        push_bits(&mut prefix, 1, 1); // allStreamsSameTimeFraming
        push_bits(&mut prefix, 0, 6); // numSubFrames
        push_bits(&mut prefix, 0, 4); // numProgram
        push_bits(&mut prefix, 0, 3); // numLayer
        push_bits(&mut prefix, u32::from(u16::from_be_bytes(AAC_LC_ASC)), 16);
        push_bits(&mut prefix, 0, 3); // frameLengthType
        push_bits(&mut prefix, 0xFF, 8); // latmBufferFullness
        push_bits(&mut prefix, 0, 1); // otherDataPresent
        let sum = crc::stream_mux_config_crc(&prefix);

        let mut w = BitWriter::new();
        for &b in &prefix {
            w.write_bit(b);
        }
        w.write_bit(true); // crcCheckPresent
        w.write_u32(u32::from(sum), 8); // crcCheckSum
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        let smc = StreamMuxConfig::parse(&mut r, &bytes).unwrap();
        assert!(smc.crc_check_present);
        assert_eq!(smc.crc_check_sum, Some(sum));
    }

    #[test]
    fn stream_mux_config_bad_crc_rejected() {
        let mut w = BitWriter::new();
        w.write_bit(false);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0, 4);
        w.write_u32(0, 3);
        write_aac_lc_asc(&mut w);
        w.write_u32(0, 3);
        w.write_u32(0xFF, 8);
        w.write_bit(false);
        w.write_bit(true); // crcCheckPresent
        w.write_u32(0x00, 8); // deliberately wrong crcCheckSum
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert!(matches!(
            StreamMuxConfig::parse(&mut r, &bytes),
            Err(Error::LatmCrcMismatch)
        ));
    }

    #[test]
    fn stream_mux_config_two_layers_use_same_config() {
        // One program, two layers; the second layer sets
        // useSameConfig, so it must inherit the first layer's ASC.
        let mut w = BitWriter::new();
        w.write_bit(false); // audioMuxVersion = 0
        w.write_bit(true); // allStreamsSameTimeFraming
        w.write_u32(0, 6); // numSubFrames
        w.write_u32(0, 4); // numProgram = 0
        w.write_u32(1, 3); // numLayer = 1 -> two layers
                           // layer 0: no useSameConfig bit; inline ASC.
        write_aac_lc_asc(&mut w);
        w.write_u32(0, 3); // frameLengthType 0
        w.write_u32(0xFF, 8); // latmBufferFullness
                              // layer 1: useSameConfig = 1.
        w.write_bit(true); // useSameConfig
        w.write_u32(0, 3); // frameLengthType 0
        w.write_u32(0xFF, 8); // latmBufferFullness
        w.write_bit(false); // otherDataPresent
        w.write_bit(false); // crcCheckPresent
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        let smc = StreamMuxConfig::parse(&mut r, &bytes).unwrap();
        assert_eq!(smc.layers.len(), 2);
        assert!(smc.layers[0].asc.is_some());
        assert!(smc.layers[1].asc.is_none());
        // The inherited effective ASC matches the first layer.
        assert_eq!(
            smc.layers[1].effective_asc.aot,
            smc.layers[0].effective_asc.aot
        );
        assert_eq!(smc.stream_id(0, 1), Some(1));
    }

    #[test]
    fn stream_mux_config_unsupported_frame_length_type() {
        // frameLengthType = 3 (CELP) must be rejected.
        let mut w = BitWriter::new();
        w.write_bit(false);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0, 4);
        w.write_u32(0, 3);
        write_aac_lc_asc(&mut w);
        w.write_u32(3, 3); // frameLengthType = 3 (CELP)
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert!(matches!(
            StreamMuxConfig::parse(&mut r, &bytes),
            Err(Error::LatmUnsupportedFrameLengthType(3))
        ));
    }

    #[test]
    fn stream_mux_config_version1_reserved_a_rejected() {
        // audioMuxVersion = 1, audioMuxVersionA = 1 -> reserved.
        let mut w = BitWriter::new();
        w.write_bit(true); // audioMuxVersion = 1
        w.write_bit(true); // audioMuxVersionA = 1
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert!(matches!(
            StreamMuxConfig::parse(&mut r, &bytes),
            Err(Error::LatmAudioMuxVersionAReserved)
        ));
    }

    #[test]
    fn stream_mux_config_frame_length_type1_fixed_bits() {
        // frameLengthType = 1, frameLength = 100 -> (100+20)*8 bits.
        let mut w = BitWriter::new();
        w.write_bit(false);
        w.write_bit(true);
        w.write_u32(0, 6);
        w.write_u32(0, 4);
        w.write_u32(0, 3);
        write_aac_lc_asc(&mut w);
        w.write_u32(1, 3); // frameLengthType = 1
        w.write_u32(100, 9); // frameLength = 100
        w.write_bit(false); // otherDataPresent
        w.write_bit(false); // crcCheckPresent
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        let smc = StreamMuxConfig::parse(&mut r, &bytes).unwrap();
        let lay = &smc.layers[0];
        assert_eq!(lay.frame_length_type, 1);
        assert_eq!(lay.frame_length, Some(100));
        assert_eq!(lay.fixed_payload_bits(), Some((100 + 20) * 8));
    }

    /// Write the minimal `audioMuxVersion == 0` AAC-LC StreamMuxConfig
    /// (one prog, one layer, frameLengthType 0, no CRC) into `w`
    /// without finishing — for embedding inside an AudioMuxElement.
    fn write_min_smc_into(w: &mut BitWriter) {
        w.write_bit(false); // audioMuxVersion = 0
        w.write_bit(true); // allStreamsSameTimeFraming
        w.write_u32(0, 6); // numSubFrames = 0
        w.write_u32(0, 4); // numProgram = 0
        w.write_u32(0, 3); // numLayer = 0
        write_aac_lc_asc(w);
        w.write_u32(0, 3); // frameLengthType = 0
        w.write_u32(0xFF, 8); // latmBufferFullness
        w.write_bit(false); // otherDataPresent
        w.write_bit(false); // crcCheckPresent
    }

    #[test]
    fn audio_mux_element_in_band_single_payload() {
        // muxConfigPresent=1, useSameStreamMux=0, inline minimal SMC,
        // one subframe carrying a 4-byte payload.
        let payload: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
        let mut w = BitWriter::new();
        w.write_bit(false); // useSameStreamMux = 0
        write_min_smc_into(&mut w);
        // PayloadLengthInfo: MuxSlotLengthBytes = 4 (single byte, < 255).
        w.write_u32(4, 8);
        // PayloadMux: 4 payload bytes.
        for &b in &payload {
            w.write_byte(b);
        }
        // otherDataPresent was 0; ByteAlign() pads.
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        let ame = AudioMuxElement::parse(&mut r, &bytes, true, None).unwrap();
        assert!(!ame.use_same_stream_mux);
        assert_eq!(ame.payloads.len(), 1);
        let p = &ame.payloads[0];
        assert_eq!(p.sub_frame, 0);
        assert_eq!(p.prog, 0);
        assert_eq!(p.lay, 0);
        assert_eq!(p.stream_id, 0);
        assert_eq!(p.data, payload.to_vec());
    }

    #[test]
    fn audio_mux_element_escape_length() {
        // MuxSlotLengthBytes with one 0xFF escape: 255 + 3 = 258 bytes.
        let len = 258usize;
        let payload: Vec<u8> = (0..len).map(|i| (i & 0xFF) as u8).collect();
        let mut w = BitWriter::new();
        w.write_bit(false); // useSameStreamMux
        write_min_smc_into(&mut w);
        w.write_u32(255, 8); // escape
        w.write_u32(3, 8); // + 3 = 258
        for &b in &payload {
            w.write_byte(b);
        }
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        let ame = AudioMuxElement::parse(&mut r, &bytes, true, None).unwrap();
        assert_eq!(ame.payloads.len(), 1);
        assert_eq!(ame.payloads[0].data, payload);
    }

    #[test]
    fn audio_mux_element_use_same_stream_mux_inherits() {
        // First element carries the config; second sets
        // useSameStreamMux and inherits it.
        let mut w0 = BitWriter::new();
        w0.write_bit(false); // useSameStreamMux = 0
        write_min_smc_into(&mut w0);
        w0.write_u32(2, 8); // 2-byte payload
        w0.write_byte(0x11);
        w0.write_byte(0x22);
        let bytes0 = w0.finish();
        let mut r0 = BitReader::new(&bytes0);
        let first = AudioMuxElement::parse(&mut r0, &bytes0, true, None).unwrap();

        let mut w1 = BitWriter::new();
        w1.write_bit(true); // useSameStreamMux = 1
        w1.write_u32(3, 8); // 3-byte payload
        w1.write_byte(0xAA);
        w1.write_byte(0xBB);
        w1.write_byte(0xCC);
        let bytes1 = w1.finish();
        let mut r1 = BitReader::new(&bytes1);
        let second = AudioMuxElement::parse(&mut r1, &bytes1, true, Some(&first.config)).unwrap();
        assert!(second.use_same_stream_mux);
        assert_eq!(second.payloads.len(), 1);
        assert_eq!(second.payloads[0].data, vec![0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn audio_mux_element_use_same_without_prev_rejected() {
        let mut w = BitWriter::new();
        w.write_bit(true); // useSameStreamMux = 1, but no prev config
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert!(matches!(
            AudioMuxElement::parse(&mut r, &bytes, true, None),
            Err(Error::LatmNoPreviousMuxConfig)
        ));
    }

    #[test]
    fn audio_mux_element_multiple_subframes() {
        // numSubFrames = 1 -> two PayloadMux frames, each a separate
        // PayloadLengthInfo + payload.
        let mut w = BitWriter::new();
        w.write_bit(false); // useSameStreamMux
                            // StreamMuxConfig with numSubFrames = 1.
        w.write_bit(false); // audioMuxVersion = 0
        w.write_bit(true); // allStreamsSameTimeFraming
        w.write_u32(1, 6); // numSubFrames = 1
        w.write_u32(0, 4); // numProgram = 0
        w.write_u32(0, 3); // numLayer = 0
        write_aac_lc_asc(&mut w);
        w.write_u32(0, 3); // frameLengthType = 0
        w.write_u32(0xFF, 8); // latmBufferFullness
        w.write_bit(false); // otherDataPresent
        w.write_bit(false); // crcCheckPresent
                            // subframe 0: 2 bytes.
        w.write_u32(2, 8);
        w.write_byte(0x01);
        w.write_byte(0x02);
        // subframe 1: 1 byte.
        w.write_u32(1, 8);
        w.write_byte(0x03);
        let bytes = w.finish();

        let mut r = BitReader::new(&bytes);
        let ame = AudioMuxElement::parse(&mut r, &bytes, true, None).unwrap();
        assert_eq!(ame.payloads.len(), 2);
        assert_eq!(ame.payloads[0].sub_frame, 0);
        assert_eq!(ame.payloads[0].data, vec![0x01, 0x02]);
        assert_eq!(ame.payloads[1].sub_frame, 1);
        assert_eq!(ame.payloads[1].data, vec![0x03]);
    }
}

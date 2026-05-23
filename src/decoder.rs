//! Top-level AAC-LC packet decoder. Wires the ADTS / ASC parser, the SCE /
//! CPE element decoders, and the IMDCT/overlap state into a single
//! `oxideav_core::Decoder` impl.
//!
//! ISO/IEC 14496-3 §4.5.2.1 (raw_data_block) and §4.6.

use oxideav_core::Decoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, Packet, Result, SampleFormat, TimeBase,
};

use crate::adts::parse_adts_header;
use crate::asc::parse_asc;
use crate::ics::{
    decode_spectrum_long, decode_spectrum_long_with_swb, decode_spectrum_short, parse_ics_info,
    parse_ics_info_ld, parse_scalefactors, parse_section_data, IcsInfo, LtpData, SectionData,
    INTENSITY_HCB, INTENSITY_HCB2, NOISE_HCB, SPEC_LEN, ZERO_HCB,
};
use crate::latm::{AudioMuxElement, LatmContext};
use crate::ld_eld::{
    apply_ltp_ld, imdct_and_overlap_ld, swb_ld_for, LdChannelState, LdFrameLength,
};
use crate::loas::parse_loas_frame;
use crate::pns::{apply_pns_long, apply_pns_short, PnsRng};
use crate::pulse::{apply_pulse_long, parse_pulse_data, PulseData};
use crate::sbr::bitstream::SbrChannelData;
use crate::sbr::decode::{
    decode_sbr_cpe_frame, decode_sbr_frame, decode_sbr_frame_ps, decode_sbr_upsample_only,
    try_parse_sbr_extension_ext, SbrChannelState, SbrPayload,
};
use crate::sbr::ps::{apply_ps_simple, PsFrame};
use crate::sfband::{SWB_LONG, SWB_SHORT};
use crate::syntax::{
    ElementType, WindowSequence, AOT_AAC_ELD, AOT_AAC_LC, AOT_ER_AAC_LD, AOT_USAC,
};
use crate::synth::{imdct_and_overlap, ChannelState, FRAME_LEN};
use crate::tns::{apply_tns_long, apply_tns_short, parse_tns_data, TnsData};
use oxideav_core::bits::BitReader;

pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    // Figure out the stream config. Two paths:
    //   (a) extradata holds an AudioSpecificConfig (MP4 path).
    //   (b) ADTS — config will come from the first packet's ADTS header.
    let (sf_index, channels, object_type, sbr_present, ps_explicit, ld_frame_length) =
        if !params.extradata.is_empty() {
            let asc = parse_asc(&params.extradata)?;
            // ELD and USAC frame decode remain gated until subsequent
            // rounds — only AOT 2 (AAC-LC) and AOT 23 (AAC-LD) reach the
            // raw_data_block decoder.
            if asc.object_type == AOT_AAC_ELD {
                return Err(Error::unsupported(
                    "AAC: AAC-ELD (objectType 39) frame decode not yet implemented \
                     (LD-SBR + low-overlap window pending; AudioSpecificConfig parse and \
                     LD-size kernels available)",
                ));
            }
            if asc.object_type == AOT_USAC {
                return Err(Error::unsupported(
                    "AAC: USAC / xHE-AAC (objectType 42) frame decode not yet implemented \
                     (UsacConfig scaffold parse available via crate::usac::parse_usac_config)",
                ));
            }
            if asc.object_type != AOT_AAC_LC && asc.object_type != AOT_ER_AAC_LD {
                return Err(Error::unsupported(
                    "AAC: only AAC-LC (objectType 2) and AAC-LD (objectType 23) supported",
                ));
            }
            let sf_idx = sample_rate_to_index(asc.sampling_frequency)
                .unwrap_or(asc.sampling_frequency_index);
            // Reject reserved (13/14) and explicit-rate escape (15)
            // sf indices — they would OOB the SWB tables in the
            // long-/short-window decode path. Mirrors the same
            // guard in `decode_packet` for the ADTS path.
            if (sf_idx as usize) >= crate::syntax::SAMPLE_RATES.len() {
                return Err(Error::invalid(
                    "AAC: AudioSpecificConfig sampling_frequency_index out of table",
                ));
            }
            // AAC-LD is restricted to channelConfiguration ∈ {1, 2} in
            // this round (§4.4.2.3 Table 4.19 also defines 3..=7, but
            // those layouts add multiple SCE/CPE/LFE elements per
            // er_raw_data_block which is a larger wiring step deferred
            // to round-N+1). Reject up-front so callers see the
            // limitation rather than a corrupted decode.
            let ld_frame_length = if asc.object_type == AOT_ER_AAC_LD {
                if !(1..=2).contains(&asc.channel_configuration) {
                    return Err(Error::unsupported(
                        "AAC-LD: only channelConfiguration 1 (mono) and 2 (stereo) \
                         supported in this round (multichannel LD deferred)",
                    ));
                }
                // SBR is not part of AAC-LD itself — that's AAC-ELD's
                // territory. If the ASC somehow advertised SBR on an LD
                // stream, surface that as unsupported rather than
                // silently dropping the doubling.
                if asc.sbr_present {
                    return Err(Error::unsupported(
                        "AAC-LD with SBR (LD-SBR) is an AAC-ELD feature, \
                         not implemented under objectType 23",
                    ));
                }
                Some(
                    asc.ld_config
                        .as_ref()
                        .map(|c| c.frame_length)
                        .unwrap_or(LdFrameLength::Samples512),
                )
            } else {
                None
            };
            (
                sf_idx,
                asc.channel_configuration,
                asc.object_type,
                asc.sbr_present,
                asc.ps_present,
                ld_frame_length,
            )
        } else {
            // Will be filled in after seeing the first ADTS frame.
            (0xFF, 0, 0, false, false, None)
        };

    // Per-channel LD overlap-add state — allocated up-front for the 8
    // possible channel slots so the LD decode path can reuse the
    // multi-slot pattern of the LC decoder.
    let ld_n = ld_frame_length.map(|f| f.samples() as usize).unwrap_or(512);
    let ld_chans: Vec<LdChannelState> = (0..8).map(|_| LdChannelState::new(ld_n)).collect();

    Ok(Box::new(AacDecoder {
        codec_id: params.codec_id.clone(),
        time_base: TimeBase::new(1, params.sample_rate.unwrap_or(44_100) as i64),
        pending: None,
        latm_pending: Vec::new(),
        adts_rdb_pending: Vec::new(),
        adts_rdb_remaining: 0,
        latm_ctx: LatmContext::new(),
        eof: false,
        sf_index,
        channels,
        object_type,
        chans: vec![ChannelState::new(); 8],
        ld_chans,
        ld_frame_length,
        configured: !params.extradata.is_empty(),
        pns_rng: PnsRng::new(),
        sbr_explicit: sbr_present,
        ps_explicit,
        sbr_state: (0..8).map(|_| SbrChannelState::new()).collect(),
        sbr_data: vec![None; 8],
        sbr_pair: vec![None; 8],
        sbr_ps: vec![None; 8],
    }))
}

fn sample_rate_to_index(sr: u32) -> Option<u8> {
    use crate::syntax::SAMPLE_RATES;
    SAMPLE_RATES.iter().position(|&r| r == sr).map(|i| i as u8)
}

struct AacDecoder {
    codec_id: CodecId,
    time_base: TimeBase,
    pending: Option<Packet>,
    /// Extra raw_data_block payloads queued from a LATM AudioMuxElement
    /// that carried more than one sub-frame per LOAS audio_sync_stream.
    /// `decode_packet` drains this before touching `pending`.
    latm_pending: Vec<Vec<u8>>,
    /// Trailing bytes of an ADTS frame whose
    /// `number_of_raw_data_blocks_in_frame` field (ISO/IEC 13818-7
    /// §6.2, Table 5) packs more than one `raw_data_block()`. This holds
    /// the byte range starting at the *next* undecoded raw_data_block.
    /// `receive_frame` decodes one block from the head of this slice per
    /// call, then re-slices the remainder until `adts_rdb_remaining`
    /// drops to zero — so a 2..4-RDB ADTS frame produces 2..4 output
    /// `Frame::Audio`s. Drained before `latm_pending`.
    adts_rdb_pending: Vec<Vec<u8>>,
    /// Count of raw_data_blocks still to be drained from
    /// `adts_rdb_pending` (the multi-RDB ADTS tail). Zero outside a
    /// multi-RDB frame.
    adts_rdb_remaining: usize,
    /// Persistent LATM/LOAS demux state — caches the last
    /// `StreamMuxConfig` so frames marked `useSameStreamMux=1` reuse it.
    latm_ctx: LatmContext,
    eof: bool,
    sf_index: u8,
    channels: u8,
    object_type: u8,
    chans: Vec<ChannelState>,
    /// Per-channel AAC-LD overlap-add state. Only used when
    /// `object_type == AOT_ER_AAC_LD`; the LC/HE-AAC paths leave this
    /// untouched. Sized to 8 slots to mirror the LC layout.
    ld_chans: Vec<LdChannelState>,
    /// LD frame length (512 or 480 samples) derived from the
    /// `LdSpecificConfig` in the ASC. `None` outside AAC-LD streams.
    ld_frame_length: Option<LdFrameLength>,
    configured: bool,
    pns_rng: PnsRng,
    /// HE-AAC: extradata explicitly signalled SBR. When set we expect every
    /// frame to carry SBR extension payloads and we double the output
    /// sample rate.
    sbr_explicit: bool,
    /// HE-AACv2: extradata advertised PS (AOT_PS == 29). We'll also flip
    /// this on-the-fly when a PS extension appears inside the SBR payload.
    ps_explicit: bool,
    /// Per-SCE-slot SBR state. Only indices corresponding to SCE / LFE
    /// elements are used in the mono path.
    sbr_state: Vec<SbrChannelState>,
    /// Per-SCE-slot SBR parsed data for the *current* frame (filled by the
    /// FIL-element handler, consumed at frame finalise).
    sbr_data: Vec<Option<SbrChannelData>>,
    /// For each channel-pair start index `c`, if `sbr_pair[c]` is `Some`
    /// the SBR payload attached to the CPE at slot `c..c+2` had stereo data
    /// and the second channel's parsed data + coupling flag are here.
    sbr_pair: Vec<Option<(SbrChannelData, bool)>>,
    /// PS (HE-AACv2) frame data per SBR mono slot.
    sbr_ps: Vec<Option<PsFrame>>,
}

impl Decoder for AacDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        if self.pending.is_some()
            || !self.latm_pending.is_empty()
            || !self.adts_rdb_pending.is_empty()
        {
            return Err(Error::other(
                "AAC decoder: receive_frame must be called before sending another packet",
            ));
        }
        // LOAS / LATM dispatch: if the packet carries the LOAS sync
        // word, demultiplex it into one or more raw_data_block payloads
        // up-front and queue them on `latm_pending` so subsequent
        // `receive_frame` calls drain them in order.
        let data = &packet.data;
        if data.len() >= 2 && data[0] == 0x56 && (data[1] & 0xE0) == 0xE0 {
            // Walk every LOAS frame in the packet (a transport packet
            // can pack several).
            let mut offset = 0usize;
            while offset < data.len() {
                if offset + 2 >= data.len() {
                    break;
                }
                if data[offset] != 0x56 || (data[offset + 1] & 0xE0) != 0xE0 {
                    offset += 1;
                    continue;
                }
                let frame = match parse_loas_frame(&data[offset..]) {
                    Ok(f) => f,
                    Err(Error::NeedMore) => break,
                    Err(e) => return Err(e),
                };
                let consumed = frame.frame_length;
                let ame = AudioMuxElement::parse(frame.payload, &mut self.latm_ctx)?;
                // Snapshot the ASC into the decoder's stream config —
                // sample rate / channel count come from the cached ASC.
                if let Some(asc) = self.latm_ctx.asc.as_ref() {
                    if asc.object_type != AOT_AAC_LC {
                        return Err(Error::unsupported(
                            "AAC: LOAS/LATM advertises non-LC profile",
                        ));
                    }
                    self.sf_index = sample_rate_to_index(asc.sampling_frequency)
                        .unwrap_or(asc.sampling_frequency_index);
                    self.channels = asc.channel_configuration;
                    self.object_type = asc.object_type;
                    self.configured = true;
                    self.time_base = TimeBase::new(1, asc.sampling_frequency as i64);
                    self.sbr_explicit = asc.sbr_present;
                    self.ps_explicit = asc.ps_present;
                }
                for f in ame.frames {
                    self.latm_pending.push(f.payload);
                }
                offset += consumed;
            }
            if self.latm_pending.is_empty() {
                return Err(Error::invalid("LOAS packet yielded no AAC sub-frames"));
            }
            // The first sub-frame inherits the original packet's pts /
            // stream_index so downstream timing stays put. Subsequent
            // sub-frames are stored without packet metadata; the time
            // base stays the same and the caller can offset by 1024
            // samples per sub-frame if it cares.
            let first = self.latm_pending.remove(0);
            let mut head = packet.clone();
            head.data = first;
            self.pending = Some(head);
            return Ok(());
        }
        self.pending = Some(packet.clone());
        Ok(())
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        // Drain queued LATM sub-frames (already demultiplexed at
        // send_packet time) before falling back to `pending`.
        if let Some(pkt) = self.pending.take() {
            return self.decode_packet(&pkt);
        }
        // Drain trailing ADTS raw_data_blocks (ISO/IEC 13818-7 §6.2,
        // `number_of_raw_data_blocks_in_frame > 0`). These were split off
        // the multi-RDB ADTS frame in `decode_packet`; each is a bare
        // raw_data_block payload, decoded via the no-sync (MP4-like)
        // branch. The decoder is already `configured` from the ADTS
        // header that carried them.
        if !self.adts_rdb_pending.is_empty() {
            let payload = self.adts_rdb_pending.remove(0);
            let pkt = Packet::new(0, self.time_base, payload);
            return self.decode_packet(&pkt);
        }
        if !self.latm_pending.is_empty() {
            let payload = self.latm_pending.remove(0);
            // Reuse the time_base set when the LOAS frame was parsed.
            let pkt = Packet::new(0, self.time_base, payload);
            return self.decode_packet(&pkt);
        }
        if self.eof {
            Err(Error::Eof)
        } else {
            Err(Error::NeedMore)
        }
    }

    fn flush(&mut self) -> Result<()> {
        self.eof = true;
        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        // Per-channel IMDCT overlap-add state (`ChannelState`) is the
        // only real DSP carry-over — wiping it guarantees the first
        // frame after a seek won't OLA against pre-seek samples.
        // Stream-level config (sf_index, channels, object_type,
        // configured, time_base) stays put; `configured` reflects that
        // we've seen either ASC extradata or an ADTS header and shouldn't
        // lose that just because the position changed.
        for ch in self.chans.iter_mut() {
            *ch = ChannelState::new();
        }
        let ld_n = self
            .ld_frame_length
            .map(|f| f.samples() as usize)
            .unwrap_or(512);
        for ch in self.ld_chans.iter_mut() {
            *ch = LdChannelState::new(ld_n);
        }
        self.pending = None;
        self.latm_pending.clear();
        self.adts_rdb_pending.clear();
        self.adts_rdb_remaining = 0;
        // Drop the cached LATM StreamMuxConfig — after a seek the next
        // LOAS frame is required to either re-emit a config or be
        // already configured at the muxer's choice. The conservative
        // thing is to start over and let the next frame re-establish.
        self.latm_ctx = LatmContext::new();
        self.eof = false;
        Ok(())
    }
}

/// Expected PCM-channel count produced by a raw_data_block given its
/// `channel_configuration` (§1.6.3, Table 1.19). Config 0 means the
/// topology is defined by a PCE and must be computed at runtime.
fn expected_channels(config: u8) -> Option<usize> {
    match config {
        1 => Some(1),
        2 => Some(2),
        3 => Some(3),
        4 => Some(4),
        5 => Some(5),
        6 => Some(6),
        7 => Some(8),
        _ => None,
    }
}

impl AacDecoder {
    fn decode_packet(&mut self, pkt: &Packet) -> Result<Frame> {
        // AAC-LD (objectType 23, ER_AAC_LD) takes a separate path —
        // er_raw_data_block() per §4.4.2.3 Table 4.19 omits the
        // element-type ID bits and uses a 512/480-sample IMDCT. ADTS
        // does not carry LD (LD is MP4 / LATM / LOAS), so we never
        // confuse the two paths.
        if self.object_type == AOT_ER_AAC_LD {
            return self.decode_packet_ld(pkt);
        }
        // Detect ADTS by syncword. Otherwise treat as raw_data_block (e.g. MP4).
        let data = &pkt.data;
        let (payload_offset, frame_end) =
            if data.len() >= 7 && data[0] == 0xFF && (data[1] & 0xF0) == 0xF0 {
                let hdr = parse_adts_header(data)?;
                if hdr.object_type != AOT_AAC_LC {
                    return Err(Error::unsupported(
                        "AAC: ADTS header advertises non-LC profile",
                    ));
                }
                // ISO/IEC 14496-3 Table 1.16: sf_index 0..=12 covers
                // 96 kHz down to 7350 Hz; 13/14 are reserved and 15
                // is the explicit-rate escape value (we don't support
                // arbitrary rates outside the table). Reject up front
                // so downstream `SWB_LONG[sf_index]` / `SWB_SHORT[..]`
                // table lookups never see an OOB index.
                if hdr.sampling_freq_index >= crate::syntax::SAMPLE_RATES.len() as u8 {
                    return Err(Error::invalid(
                        "ADTS: reserved/escape sampling_frequency_index",
                    ));
                }
                self.sf_index = hdr.sampling_freq_index;
                self.channels = hdr.channel_configuration;
                self.object_type = hdr.object_type;
                self.configured = true;
                self.time_base = TimeBase::new(1, hdr.sample_rate().unwrap_or(44_100) as i64);
                // Workspace task #760: a 7-byte packet that codes
                // `protection_absent == 0` (CRC follows) advertises a
                // 9-byte header but the packet itself is too short to
                // carry it. `parse_adts_header` only reads 7 bytes, so
                // it succeeds; without the bounds check below
                // `data[payload_offset..frame_end]` panics with
                // "range start index 9 out of range for slice of
                // length 7" because `payload_offset = 9 > data.len()`.
                // Reject up-front per ISO/IEC 14496-3 §1.A.2 — a frame
                // shorter than its declared header is malformed.
                let header_len = hdr.header_length();
                if data.len() < header_len || hdr.frame_length < header_len {
                    return Err(Error::invalid(
                        "ADTS: packet shorter than declared header length",
                    ));
                }
                let frame_end = hdr.frame_length.min(data.len());
                // ISO/IEC 13818-7 §6.2, Table 5 — adts_frame() multiplexes
                // `number_of_raw_data_blocks_in_frame + 1` raw_data_block()s.
                // The common case is 0 (one block); when > 0 the trailing
                // blocks are split off here and drained one-per-frame.
                let n_extra = hdr.number_of_raw_blocks_minus_one as usize;
                if n_extra > 0 {
                    if hdr.protection_absent {
                        // No CRC ⇒ no adts_header_error_check, no per-block
                        // CRCs (Tables 6 & 7 are empty when protection_absent
                        // == 1). raw_data_block()s sit back-to-back, each
                        // byte-aligned (§4.4.2.1 byte_alignment() after END).
                        // There are no position pointers, so the boundaries
                        // are discovered by parsing each block: rdb0 decodes
                        // from `header_len`, and the post-END byte position
                        // (captured below the element loop) locates rdb1, and
                        // so on via `adts_rdb_remaining`.
                        self.adts_rdb_remaining = n_extra;
                        (header_len, frame_end)
                    } else {
                        // CRC present ⇒ adts_header_error_check (Table 6)
                        // carries `raw_data_block_position[i]` (16 bits each,
                        // i = 1..=n_extra) followed by a 16-bit crc_check.
                        // The fixed+variable header is 7 bytes; the position
                        // array + header CRC adds `(n_extra + 1) * 2` bytes
                        // before the first raw_data_block. Each trailing
                        // block is followed by a 2-byte adts_raw_data_block_
                        // error_check CRC (Table 7), but the position pointers
                        // (offsets from rdb0's start) already skip past them,
                        // so we split purely on the pointers and ignore the
                        // CRC bytes (we do not validate CRC anywhere).
                        let hdr_ec_bytes = (n_extra + 1) * 2;
                        let rdb0_off = 7 + hdr_ec_bytes;
                        if frame_end < rdb0_off {
                            return Err(Error::invalid(
                                "ADTS: multi-RDB frame shorter than header error check",
                            ));
                        }
                        // Read the position pointers from the header error
                        // check region (immediately after the 7-byte header).
                        let mut pos_reader = BitReader::with_position(data, 7);
                        let mut positions = Vec::with_capacity(n_extra);
                        for _ in 0..n_extra {
                            positions.push(pos_reader.read_u32(16)? as usize);
                        }
                        // Split blocks 1..=n_extra into the pending queue.
                        // positions[i] is rdb(i+1)'s byte offset from rdb0's
                        // start. The final block runs to frame_end.
                        for i in 0..n_extra {
                            let start = rdb0_off + positions[i];
                            let end = if i + 1 < n_extra {
                                rdb0_off + positions[i + 1]
                            } else {
                                frame_end
                            };
                            if start > frame_end || end > frame_end || start > end {
                                return Err(Error::invalid(
                                    "ADTS: raw_data_block_position out of range",
                                ));
                            }
                            self.adts_rdb_pending.push(data[start..end].to_vec());
                        }
                        // rdb0 spans from rdb0_off up to the first pointer.
                        let rdb0_end = rdb0_off + positions[0];
                        if rdb0_end > frame_end {
                            return Err(Error::invalid(
                                "ADTS: raw_data_block_position[1] out of range",
                            ));
                        }
                        // Pre-split path: everything queued, no recursion.
                        self.adts_rdb_remaining = 0;
                        (rdb0_off, rdb0_end)
                    }
                } else {
                    (header_len, frame_end)
                }
            } else {
                if !self.configured {
                    return Err(Error::invalid(
                        "AAC: first packet has no ADTS sync and no extradata config",
                    ));
                }
                (0, data.len())
            };
        let payload = &data[payload_offset..frame_end];

        // Decode raw_data_block.
        let mut br = BitReader::new(payload);
        // Up to 8 output channels (7.1). Individual SCE / CPE / LFE
        // elements write to the slot at the running `got_channels` index
        // and advance it by 1 (SCE / LFE) or 2 (CPE).
        let mut pcm: Vec<Vec<f32>> = (0..8).map(|_| vec![0.0; FRAME_LEN]).collect();
        let mut got_channels: usize = 0;
        // Track the index & kind of the most recent SCE / CPE / LFE so
        // fill_element can route SBR extension payloads to the right slot
        // per Note 1 of Table 4.57.
        let mut last_elem_start: usize = 0;
        let mut last_elem_is_cpe: bool = false;

        // libavcodec is tolerant to non-conformant trailing elements:
        // workspace task #759 surfaced an 88.2 kHz / 5.1 ADTS frame
        // where ffmpeg emits the leading SCE output even though a
        // follow-up CPE has out-of-range max_sfb. Mirror that
        // behaviour by tracking whether *any* full element decoded
        // before a parse error and, if so, returning the partial
        // PCM rather than bailing the whole frame.
        let mut element_loop_err: Option<Error> = None;
        'elements: loop {
            let id = match br.read_u32(3) {
                Ok(v) => v,
                Err(e) => {
                    element_loop_err = Some(e);
                    break 'elements;
                }
            };
            let kind = ElementType::from_id(id);
            // Inner match wrapped in a Result-returning closure so any
            // inner `?` failure on a malformed sub-element falls back
            // to "stop here, emit what we have" rather than discarding
            // already-decoded SCE/CPE/LFE channels.
            let iter_result: Result<bool> = (|| -> Result<bool> {
                match kind {
                    ElementType::Sce => {
                        last_elem_start = got_channels;
                        last_elem_is_cpe = false;
                        let _instance_tag = br.read_u32(4)?;
                        let mut spec = [0.0f32; SPEC_LEN];
                        let (info, sf, sec, tns, pulse) =
                            decode_ics(&mut br, self.sf_index, false)?;
                        fill_spectrum(&mut br, &info, &sec, &sf, &mut spec)?;
                        // Pulse data: §4.6.5 — applied to long-window spectrum
                        // before PNS / TNS.
                        if let Some(pd) = pulse.as_ref() {
                            // §4.6.5.2: pulse_data is only valid for long windows.
                            // A stream with pulse_data_present=1 on EIGHT_SHORT is
                            // non-conformant.
                            if info.window_sequence == WindowSequence::EightShort {
                                return Err(Error::invalid(
                                "AAC: pulse_data_present=1 with EIGHT_SHORT window (non-conformant)",
                            ));
                            }
                            apply_pulse_long(&mut spec, pd, self.sf_index, info.max_sfb, &sf)?;
                        }
                        // PNS first: fill NOISE_HCB bands with shaped noise.
                        if info.window_sequence == WindowSequence::EightShort {
                            apply_pns_short(&mut spec, &info, &sec, &sf, &mut self.pns_rng);
                        } else {
                            apply_pns_long(
                                &mut spec,
                                &info,
                                &sec,
                                &sf,
                                &mut self.pns_rng,
                                None,
                                None,
                            );
                        }
                        // TNS after PNS/IS but before IMDCT (§4.6.9.2).
                        if let Some(tns) = tns.as_ref() {
                            if info.window_sequence == WindowSequence::EightShort {
                                apply_tns_short(&mut spec, tns, self.sf_index, info.max_sfb, &info);
                            } else {
                                let swb = SWB_LONG[self.sf_index as usize];
                                apply_tns_long(&mut spec, tns, self.sf_index, info.max_sfb, swb);
                            }
                        }
                        if got_channels >= pcm.len() {
                            return Err(Error::invalid(
                                "AAC: more decoded channels than 8 slots permit",
                            ));
                        }
                        let mut channel_pcm = [0.0f32; FRAME_LEN];
                        imdct_and_overlap(
                            &spec,
                            info.window_sequence,
                            info.window_shape,
                            &mut self.chans[got_channels],
                            &mut channel_pcm,
                        );
                        pcm[got_channels].copy_from_slice(&channel_pcm);
                        got_channels += 1;
                    }
                    ElementType::Cpe => {
                        last_elem_start = got_channels;
                        last_elem_is_cpe = true;
                        let _instance_tag = br.read_u32(4)?;
                        let common_window = br.read_bit()?;
                        if common_window {
                            // Shared ICS info, then ms_mask flags.
                            let info = parse_ics_info(&mut br, self.sf_index)?;
                            let ms_mask_present = br.read_u32(2)? as u8;
                            let max_sfb = info.max_sfb as usize;
                            let groups = info.num_window_groups as usize;
                            let mut ms_used = vec![false; groups * max_sfb];
                            match ms_mask_present {
                                0 => { /* ms not used */ }
                                1 => {
                                    for i in 0..groups * max_sfb {
                                        ms_used[i] = br.read_bit()?;
                                    }
                                }
                                2 => {
                                    for i in 0..groups * max_sfb {
                                        ms_used[i] = true;
                                    }
                                }
                                _ => return Err(Error::invalid("AAC: reserved ms_mask_present=3")),
                            }
                            let mut spec = [[0.0f32; SPEC_LEN]; 2];
                            let mut secs: [SectionData; 2] = Default::default();
                            let mut sfs: [Vec<i32>; 2] = Default::default();
                            let infos: [IcsInfo; 2] = [info.clone(), info.clone()];
                            let mut tns_all: [Option<TnsData>; 2] = [None, None];
                            let mut pulse_all: [Option<PulseData>; 2] = [None, None];
                            for ch in 0..2 {
                                let gg = br.read_u32(8)? as u8;
                                let sec = parse_section_data(&mut br, &infos[ch])?;
                                let sf = parse_scalefactors(&mut br, &infos[ch], &sec, gg)?;
                                let pulse_present = br.read_bit()?;
                                if pulse_present {
                                    pulse_all[ch] = Some(parse_pulse_data(&mut br)?);
                                }
                                let tns_present = br.read_bit()?;
                                if tns_present {
                                    let n_windows = if infos[ch].window_sequence.is_eight_short() {
                                        8
                                    } else {
                                        1
                                    };
                                    tns_all[ch] = Some(parse_tns_data(
                                        &mut br,
                                        infos[ch].window_sequence,
                                        n_windows,
                                    )?);
                                }
                                // See `decode_ics` for the rationale on
                                // tolerating gain_control_data_present in the
                                // common-window CPE path.
                                let _gain_control = br.read_bit()?;
                                secs[ch] = sec;
                                sfs[ch] = sf;
                                fill_spectrum(
                                    &mut br,
                                    &infos[ch],
                                    &secs[ch],
                                    &sfs[ch],
                                    &mut spec[ch],
                                )?;
                                if let Some(pd) = pulse_all[ch].as_ref() {
                                    if infos[ch].window_sequence == WindowSequence::EightShort {
                                        return Err(Error::invalid(
                                        "AAC: pulse_data_present=1 with EIGHT_SHORT window (non-conformant)",
                                    ));
                                    }
                                    apply_pulse_long(
                                        &mut spec[ch],
                                        pd,
                                        self.sf_index,
                                        infos[ch].max_sfb,
                                        &sfs[ch],
                                    )?;
                                }
                            }
                            // PNS: fill NOISE_HCB bands. For correlated-noise bands
                            // (ms_used set on a noise sfb) both channels share the
                            // same random sequence.
                            if infos[0].window_sequence == WindowSequence::EightShort {
                                apply_pns_short(
                                    &mut spec[0],
                                    &infos[0],
                                    &secs[0],
                                    &sfs[0],
                                    &mut self.pns_rng,
                                );
                                apply_pns_short(
                                    &mut spec[1],
                                    &infos[1],
                                    &secs[1],
                                    &sfs[1],
                                    &mut self.pns_rng,
                                );
                            } else {
                                // Apply channel 0 + optionally mirror to channel 1.
                                let (s0, s1) = spec.split_at_mut(1);
                                apply_pns_long(
                                    &mut s0[0],
                                    &infos[0],
                                    &secs[0],
                                    &sfs[0],
                                    &mut self.pns_rng,
                                    Some(&mut s1[0]),
                                    Some(&ms_used),
                                );
                                // Fill any NOISE_HCB bands channel 1 has that
                                // channel 0 did not (rare, but spec-legal).
                                apply_pns_long_ch1_leftover(
                                    &mut s1[0],
                                    &infos[1],
                                    &secs[0],
                                    &secs[1],
                                    &sfs[1],
                                    &mut self.pns_rng,
                                );
                            }
                            // Intensity stereo (§4.6.8.2.3): for each band whose
                            // channel-1 codebook is INTENSITY_HCB / INTENSITY_HCB2,
                            // synthesise spec[1] from spec[0] scaled by
                            // sign * 2^(-is_position/4). apply_ms_stereo below
                            // already skips IS bands, so IS must run first.
                            apply_intensity_stereo(
                                &infos[0],
                                &secs,
                                &sfs,
                                &ms_used,
                                ms_mask_present,
                                &mut spec,
                            );
                            // M/S stereo: replace L,R with (L+R)/sqrt(2), (L-R)/sqrt(2)?
                            // Per spec §4.6.13.3:
                            //   L = M + S; R = M - S  (no sqrt scaling — IS-only normalisation
                            //   uses sqrt(2), but MS as defined in 14496-3 is L=M+S, R=M-S).
                            apply_ms_stereo(&infos[0], &secs, &ms_used, &mut spec);
                            // TNS after PNS + M/S, before IMDCT.
                            for ch in 0..2 {
                                if let Some(tns) = tns_all[ch].as_ref() {
                                    if infos[ch].window_sequence == WindowSequence::EightShort {
                                        apply_tns_short(
                                            &mut spec[ch],
                                            tns,
                                            self.sf_index,
                                            infos[ch].max_sfb,
                                            &infos[ch],
                                        );
                                    } else {
                                        let swb = SWB_LONG[self.sf_index as usize];
                                        apply_tns_long(
                                            &mut spec[ch],
                                            tns,
                                            self.sf_index,
                                            infos[ch].max_sfb,
                                            swb,
                                        );
                                    }
                                }
                            }
                            if got_channels + 2 > pcm.len() {
                                return Err(Error::invalid(
                                    "AAC: CPE would overflow 8 channel slots",
                                ));
                            }
                            for ch in 0..2 {
                                let mut channel_pcm = [0.0f32; FRAME_LEN];
                                imdct_and_overlap(
                                    &spec[ch],
                                    infos[ch].window_sequence,
                                    infos[ch].window_shape,
                                    &mut self.chans[got_channels + ch],
                                    &mut channel_pcm,
                                );
                                pcm[got_channels + ch].copy_from_slice(&channel_pcm);
                            }
                            got_channels += 2;
                        } else {
                            // Independent ICS for each channel.
                            let mut spec = [[0.0f32; SPEC_LEN]; 2];
                            let mut infos: [IcsInfo; 2] = Default::default();
                            let mut tns_all: [Option<TnsData>; 2] = [None, None];
                            for ch in 0..2 {
                                let (info, sf, sec, tns, pulse) =
                                    decode_ics(&mut br, self.sf_index, true)?;
                                fill_spectrum(&mut br, &info, &sec, &sf, &mut spec[ch])?;
                                if let Some(pd) = pulse.as_ref() {
                                    if info.window_sequence == WindowSequence::EightShort {
                                        return Err(Error::invalid(
                                        "AAC: pulse_data_present=1 with EIGHT_SHORT window (non-conformant)",
                                    ));
                                    }
                                    apply_pulse_long(
                                        &mut spec[ch],
                                        pd,
                                        self.sf_index,
                                        info.max_sfb,
                                        &sf,
                                    )?;
                                }
                                // PNS per channel, independent (no CPE common_window
                                // => no shared ms flags).
                                if info.window_sequence == WindowSequence::EightShort {
                                    apply_pns_short(
                                        &mut spec[ch],
                                        &info,
                                        &sec,
                                        &sf,
                                        &mut self.pns_rng,
                                    );
                                } else {
                                    apply_pns_long(
                                        &mut spec[ch],
                                        &info,
                                        &sec,
                                        &sf,
                                        &mut self.pns_rng,
                                        None,
                                        None,
                                    );
                                }
                                infos[ch] = info;
                                tns_all[ch] = tns;
                            }
                            // Bounds check BEFORE the IMDCT loop — the loop
                            // body indexes `self.chans[got_channels + ch]` and
                            // `pcm[got_channels + ch]`, both of which would
                            // panic for `got_channels + ch >= 8`. Workspace
                            // task #759 follow-up: a fuzz input that chains
                            // four CPE elements (8 channels) followed by a
                            // fifth CPE used to OOB-panic at decoder.rs:704
                            // because the guard sat *after* the loop
                            // (mirrors the symmetric guard in the
                            // `common_window` branch above at line 614).
                            if got_channels + 2 > pcm.len() {
                                return Err(Error::invalid(
                                    "AAC: CPE would overflow 8 channel slots",
                                ));
                            }
                            for ch in 0..2 {
                                if let Some(tns) = tns_all[ch].as_ref() {
                                    if infos[ch].window_sequence == WindowSequence::EightShort {
                                        apply_tns_short(
                                            &mut spec[ch],
                                            tns,
                                            self.sf_index,
                                            infos[ch].max_sfb,
                                            &infos[ch],
                                        );
                                    } else {
                                        let swb = SWB_LONG[self.sf_index as usize];
                                        apply_tns_long(
                                            &mut spec[ch],
                                            tns,
                                            self.sf_index,
                                            infos[ch].max_sfb,
                                            swb,
                                        );
                                    }
                                }
                                let mut channel_pcm = [0.0f32; FRAME_LEN];
                                imdct_and_overlap(
                                    &spec[ch],
                                    infos[ch].window_sequence,
                                    infos[ch].window_shape,
                                    &mut self.chans[got_channels + ch],
                                    &mut channel_pcm,
                                );
                                pcm[got_channels + ch].copy_from_slice(&channel_pcm);
                            }
                            got_channels += 2;
                        }
                    }
                    ElementType::Lfe => {
                        last_elem_start = got_channels;
                        last_elem_is_cpe = false;
                        // LFE element (§4.6.10) has the same bitstream syntax as
                        // an SCE but is restricted to long-window output with a
                        // small max_sfb. Decode the same way and write to the
                        // next channel slot.
                        let _instance_tag = br.read_u32(4)?;
                        let mut spec = [0.0f32; SPEC_LEN];
                        let (info, sf, sec, tns, pulse) =
                            decode_ics(&mut br, self.sf_index, false)?;
                        if info.window_sequence == WindowSequence::EightShort {
                            return Err(Error::invalid(
                                "AAC: LFE element with EIGHT_SHORT window (non-conformant)",
                            ));
                        }
                        fill_spectrum(&mut br, &info, &sec, &sf, &mut spec)?;
                        if let Some(pd) = pulse.as_ref() {
                            apply_pulse_long(&mut spec, pd, self.sf_index, info.max_sfb, &sf)?;
                        }
                        apply_pns_long(&mut spec, &info, &sec, &sf, &mut self.pns_rng, None, None);
                        if let Some(tns) = tns.as_ref() {
                            let swb = SWB_LONG[self.sf_index as usize];
                            apply_tns_long(&mut spec, tns, self.sf_index, info.max_sfb, swb);
                        }
                        if got_channels >= pcm.len() {
                            return Err(Error::invalid("AAC: LFE would overflow 8 channel slots"));
                        }
                        let mut channel_pcm = [0.0f32; FRAME_LEN];
                        imdct_and_overlap(
                            &spec,
                            info.window_sequence,
                            info.window_shape,
                            &mut self.chans[got_channels],
                            &mut channel_pcm,
                        );
                        pcm[got_channels].copy_from_slice(&channel_pcm);
                        got_channels += 1;
                    }
                    ElementType::Cce => {
                        return Err(Error::unsupported("AAC: CCE element not implemented"));
                    }
                    ElementType::Dse => {
                        let _instance_tag = br.read_u32(4)?;
                        let data_byte_align = br.read_bit()?;
                        let mut count = br.read_u32(8)?;
                        if count == 255 {
                            count += br.read_u32(8)?;
                        }
                        if data_byte_align {
                            br.align_to_byte();
                        }
                        for _ in 0..count {
                            br.read_u32(8)?;
                        }
                    }
                    ElementType::Pce => {
                        // Parse the PCE so we stay aligned on the bitstream even
                        // if we don't yet route the described channel layout to
                        // the output. Multi-channel support is a separate wiring
                        // step tracked alongside LFE element handling.
                        let _pce = crate::pce::parse_program_config_element(&mut br)?;
                    }
                    ElementType::Fil => {
                        // ISO/IEC 14496-3:2009 Table 4.11 — fill_element():
                        //
                        //   cnt = count;                 // 4 bits
                        //   if (cnt == 15)
                        //       cnt += esc_count - 1;    // 8 bits
                        //
                        // i.e. when the 4-bit `count` saturates at 15 the real
                        // payload byte length is `15 + esc_count - 1`, which
                        // simplifies to `14 + esc_count`. The earlier
                        // `count += esc_count.saturating_sub(1)` form misread
                        // this — for the conformant `esc_count == 0` case it
                        // yielded `count = 15` instead of `14`, asking the
                        // bit-reader for one extra byte and tripping
                        // `bitreader: out of bits` on the trailing FIL of
                        // ~1.4 % of real-world ADTS-AAC frames (e.g. the AAC
                        // track of `congress_mtgox_coins.mp4`, where 51 of
                        // 3711 packets carry exactly this pattern). The
                        // correct expression is the literal spec subtraction;
                        // `esc_count` is unsigned so it can legitimately
                        // shrink the total by one when zero.
                        let mut count = br.read_u32(4)?;
                        if count == 15 {
                            let esc_count = br.read_u32(8)?;
                            count = 14 + esc_count;
                        }
                        if count > 0 {
                            let total_bits = 8 * count;
                            // Peek extension_type (4 bits) without committing.
                            let peeked = br.peek_u32(4)?;
                            let is_sbr = peeked == 0xD || peeked == 0xE;
                            // ISO/IEC 14496-3 §4.4.2.3 Note 1 (Table 4.57): an
                            // SBR extension payload attaches to the immediately
                            // preceding SCE / CPE / LFE element. If a fill_element
                            // carrying SBR appears BEFORE any channel element in
                            // the same raw_data_block, the bitstream is non-
                            // conformant — libavcodec rejects with "SBR was
                            // found before the first channel element" and emits
                            // the AAC-LC core (no SBR upsampling). Mirror that:
                            // skip the SBR payload bytes so trailing elements
                            // still decode but `sbr_data[..]` stays None and the
                            // frame emits 1024 samples per channel, matching
                            // libavcodec on the same bytes (fuzz oracle
                            // ffmpeg_oracle_decode crash-de1c9e6579a2 at
                            // sf_index=8 ch=2 with leading SBR FIL).
                            if is_sbr && got_channels == 0 {
                                // Drop the SBR payload bytes — same treatment
                                // as the non-SBR `else` branch below.
                                for _ in 0..count {
                                    br.read_u32(8)?;
                                }
                            } else if is_sbr {
                                // SBR payload belongs to the most recent SCE /
                                // CPE element (Note 1 of Table 4.57). `sf_index`
                                // carries the core sample rate.
                                let core_rate =
                                    crate::syntax::sample_rate(self.sf_index).unwrap_or(44_100);
                                let target_ch = last_elem_start;
                                let is_sce = !last_elem_is_cpe;
                                if target_ch < self.sbr_state.len() {
                                    // Split the mutable borrow so we can hand
                                    // out two disjoint `&mut SbrChannelState`
                                    // for the CPE path.
                                    let state_state_len = self.sbr_state.len();
                                    let (state_l_slice, tail) =
                                        self.sbr_state.split_at_mut(target_ch + 1);
                                    let state_l = &mut state_l_slice[target_ch];
                                    match try_parse_sbr_extension_ext(
                                        &mut br, total_bits, is_sce, state_l, core_rate,
                                    )? {
                                        Some(SbrPayload::Single { data: sbr, ps }) => {
                                            self.sbr_data[target_ch] = Some(sbr);
                                            self.sbr_pair[target_ch] = None;
                                            if ps.is_some() {
                                                self.ps_explicit = true;
                                            }
                                            self.sbr_ps[target_ch] = ps;
                                            self.sbr_explicit = true;
                                        }
                                        Some(SbrPayload::Pair { l, r, coupled }) => {
                                            // Replicate header + freq tables into
                                            // the right-channel state so it can
                                            // run the analysis/synthesis banks
                                            // with matching config.
                                            if target_ch + 1 < state_state_len {
                                                // Right state is the first entry
                                                // of `tail` (it was split off at
                                                // target_ch+1).
                                                let state_r = &mut tail[0];
                                                state_r.header = state_l.header.clone();
                                                state_r.freq = state_l.freq.clone();
                                                state_r.patches = state_l.patches.clone();
                                                state_r.header_seen = state_l.header_seen;
                                            }
                                            self.sbr_data[target_ch] = Some(l);
                                            self.sbr_pair[target_ch] = Some((r, coupled));
                                            self.sbr_explicit = true;
                                        }
                                        None => {
                                            // Not actually SBR — bits already
                                            // consumed.
                                        }
                                    }
                                } else {
                                    // No channel slot to attach this to — skip.
                                    for _ in 0..total_bits {
                                        let _ = br.read_u32(1)?;
                                    }
                                }
                            } else {
                                // Non-SBR extension — skip the payload bytes.
                                for _ in 0..count {
                                    br.read_u32(8)?;
                                }
                            }
                        }
                    }
                    ElementType::End => return Ok(true),
                }
                Ok(false)
            })();
            match iter_result {
                Ok(true) => break 'elements,
                Ok(false) => {}
                Err(e) => {
                    element_loop_err = Some(e);
                    break 'elements;
                }
            }
        }

        // Multi-RDB ADTS, no-CRC tail (ISO/IEC 13818-7 §6.2): the trailing
        // raw_data_block()s are not length-prefixed, so locate the next
        // block by the post-END byte position (each block is byte-aligned
        // per §4.4.2.1) and queue the remaining bytes for the next
        // `receive_frame`. `adts_rdb_remaining` counts blocks still to
        // come AFTER the one just decoded; each pass queues one more block
        // and decrements. Only fires while draining a `protection_absent
        // == 1` multi-RDB frame; the CRC path pre-splits via position
        // pointers and leaves `adts_rdb_remaining == 0`.
        if self.adts_rdb_remaining > 0 {
            br.align_to_byte();
            let next = br.byte_position();
            if next < payload.len() {
                self.adts_rdb_pending.push(payload[next..].to_vec());
                self.adts_rdb_remaining -= 1;
            } else {
                // Ran out of bytes before the declared block count —
                // stop draining rather than queue an empty tail.
                self.adts_rdb_remaining = 0;
            }
        }

        // If the element loop errored before producing ANY output
        // channel, propagate the error so callers see the
        // bitstream failure — UNLESS the stream is configured (we
        // know the channel layout from ADTS or ASC) AND the error
        // was a bit-reader truncation. Workspace task #773
        // (round-#759 follow-up): a 53-byte fuzz input with two
        // 15-byte ADTS frames at 88.2 kHz / 5.1 ch carries an
        // 8-byte payload too short to decode any element. The
        // fuzz oracle's reference decoder emits a silent frame
        // per ADTS header rather than dropping the run, and the
        // harness treats "reference emitted N frames, oxideav-aac
        // emitted 0" as a hard panic. Match that output: when
        // we've parsed an ADTS or ASC header and the payload is
        // just too short, emit a silent frame at the expected
        // channel count rather than erroring out. The pcm buffers
        // are already zero-initialised on line 360 so the
        // fall-through to S16 conversion below produces exactly
        // the silent frame we want. Other (non-truncation)
        // first-element errors still propagate so real decoder
        // bugs surface in tests.
        if got_channels == 0 {
            if let Some(e) = element_loop_err {
                let is_bit_truncation = matches!(
                    &e,
                    Error::InvalidData(msg) if msg.contains("bitreader: out of bits")
                );
                if !(self.configured && is_bit_truncation) {
                    return Err(e);
                }
            }
        }

        // Convert PCM to interleaved S16. Use whichever is smaller between
        // what we actually decoded (`got_channels`) and the count
        // advertised by the channel_configuration — a malformed stream
        // that produces too few elements is downgraded rather than
        // crashing. For config 0 (PCE-defined) we output whatever the
        // elements produced.
        let expected = expected_channels(self.channels).unwrap_or(got_channels);
        let channels_out = if got_channels == 0 {
            // No element decoded — emit silence at the configured
            // channel layout (workspace task #773 / #759 follow-up).
            // Pre-#773 this hard-coded 1, but the fuzz oracle compares
            // per-channel sample counts and a 5.1 ADTS frame whose
            // payload was too short to decode any element should still
            // emit 6-channel silence (matching libavcodec's fallback)
            // rather than mono. Cap at 8 (the `pcm` Vec capacity above).
            expected.clamp(1, 8)
        } else {
            got_channels.min(expected.max(1))
        };
        let bytes_per_sample = SampleFormat::S16.bytes_per_sample();

        // SBR doubling — if any channel has SBR data, run the HE-AACv1
        // decode path per-channel and produce 2× samples at 2× rate. Mono
        // SCE and stereo CPE (coupled + independent) are both supported.
        // HE-AACv2: a mono SBR with an attached PS payload produces a
        // stereo output via apply_ps_simple.
        let ps_active = self.ps_explicit || self.sbr_ps.iter().any(|p| p.is_some());
        // Implicit SBR validity gate — ISO/IEC 14496-3 §4.6.18.2.6.
        //
        // SBR output rate equals `2 * core_rate` (one HF synthesis QMF doubles
        // the per-frame sample count). For the doubled rate to be a usable
        // AAC output the post-SBR rate MUST itself be a valid sf_index per
        // Table 1.16. With AAC-LC sf_index ∈ {9..=12} (12 / 11.025 / 8 / 7.35
        // kHz) the doubled output (6 / 5.5125 / 4 / 3.675 kHz) is NOT in the
        // standard rate table — those sf_indices were never intended to
        // carry SBR. libavcodec's built-in AAC decoder rejects such streams
        // with "SBR was found before the first channel element" and emits
        // 1024 samples (no SBR upsampling). We mirror that behaviour by
        // gating `sbr_active` on the doubled rate being a recognised
        // sf_index — otherwise we'd emit 2048 samples while every other
        // conformant decoder emits 1024 (fuzz oracle crash:
        // ffmpeg_oracle_decode crash-321bb909a4 at sr=7350 ch=2).
        let sbr_doubled_rate_ok = crate::syntax::sample_rate(self.sf_index)
            .and_then(|core| {
                let doubled = core.checked_mul(2)?;
                if crate::syntax::SAMPLE_RATES.contains(&doubled) {
                    Some(())
                } else {
                    None
                }
            })
            .is_some();
        // HE-AAC channel-configuration gate. ISO/IEC 14496-3 Table 1.19
        // assigns channel_configuration 1 = mono, 2 = stereo; configs
        // 3..=7 (3.0 through 7.1) are multichannel and HE-AAC SBR is
        // explicitly NOT defined for them in §4.6.18. libavcodec
        // ignores SBR FIL data on streams advertising channel_cfg ≥ 3
        // and emits 1024 samples per channel. If we downgrade
        // `channels_out` from the configured 4 (etc.) to the 2 we
        // actually decoded, applying SBR to those 2 channels gives 2048
        // samples while libavcodec produces 1024 — fuzz oracle crash
        // ffmpeg_oracle_decode crash-3ecd0ba68e4c at sf_index=10
        // (11025 Hz) ch_cfg=4. Gate `sbr_active` on the ADTS-declared
        // channel_configuration being in {1, 2}, not on the downgraded
        // `channels_out`.
        let expected_cfg = expected_channels(self.channels).unwrap_or(channels_out);
        let sbr_active = (self.sbr_explicit || self.sbr_data.iter().any(|s| s.is_some()))
            && (1..=2).contains(&channels_out)
            && (1..=2).contains(&expected_cfg)
            && sbr_doubled_rate_ok;
        if sbr_active {
            let out_samples = 2 * FRAME_LEN;
            // Allocate for the HE-AACv1 output (mono or stereo CPE). PS
            // upmix adds a second buffer post-hoc when `ps_active && mono`.
            let mut sbr_pcm: Vec<Vec<f32>> = (0..channels_out)
                .map(|_| vec![0.0f32; out_samples])
                .collect();

            // Stereo CPE path: slot 0 holds the SCE/CPE-left data; slot 0
            // in `sbr_pair` holds the right-channel data + coupling flag.
            if channels_out == 2 && self.sbr_pair[0].is_some() {
                let sbr_l = self.sbr_data[0].take();
                let pair = self.sbr_pair[0].take();
                if let (Some(sbr_l), Some((sbr_r, coupled))) = (sbr_l, pair) {
                    // Split sbr_state into two disjoint borrows.
                    let (head, tail) = self.sbr_state.split_at_mut(1);
                    let state_l = &mut head[0];
                    let state_r = &mut tail[0];
                    let (lo, hi) = sbr_pcm.split_at_mut(1);
                    if decode_sbr_cpe_frame(
                        &pcm[0], &pcm[1], &sbr_l, &sbr_r, coupled, state_l, state_r, &mut lo[0],
                        &mut hi[0],
                    )
                    .is_err()
                    {
                        // Fallback: spec-compliant upsample-only path
                        // (§4.6.18.5) per channel — keeps the output at
                        // 2 * FRAME_LEN samples and feeds the synthesis
                        // QMF a zero high-band rather than the ZOH
                        // duplicate-and-double that we used pre-r91.
                        // ZOH defeats the QMF and yields nearest-neighbour
                        // aliasing on the trailing frame; the QMF-domain
                        // path produces the same fold-over rejection as
                        // a real SBR-active frame would.
                        let (head, tail) = self.sbr_state.split_at_mut(1);
                        let state_l2 = &mut head[0];
                        let state_r2 = &mut tail[0];
                        let _ = decode_sbr_upsample_only(&pcm[0], state_l2, &mut sbr_pcm[0]);
                        let _ = decode_sbr_upsample_only(&pcm[1], state_r2, &mut sbr_pcm[1]);
                    }
                }
            } else if channels_out == 1
                && ps_active
                && self.sbr_ps[0].is_some()
                && self.sbr_data[0].is_some()
            {
                // HE-AACv2: mono + PS. Route through the QMF-domain upmix —
                // one analysis QMF → HF generate/adjust → PS → two synthesis
                // QMFs producing stereo. This replaces the prior time-domain
                // duplicate-and-pan approach.
                let sbr = self.sbr_data[0].take().unwrap();
                let ps_frame = self.sbr_ps[0].take().unwrap();
                // Split the channel state so we can borrow `ps_right_qmf`
                // mutably alongside `state` (they live in the same struct).
                let state = &mut self.sbr_state[0];
                let mut l = vec![0.0f32; out_samples];
                let mut r = vec![0.0f32; out_samples];
                // SAFETY-LIKE: we need two disjoint mut borrows into the same
                // struct. Use a single mutable borrow and a raw pointer
                // trick via split: call the function with the struct and
                // then its right-qmf field. Easiest: temporarily move the
                // field out via std::mem::take, call, and restore.
                let mut right_qmf = std::mem::take(&mut state.ps_right_qmf);
                let decode_ok = decode_sbr_frame_ps(
                    &pcm[0],
                    &sbr,
                    &ps_frame,
                    state,
                    &mut right_qmf,
                    &mut l,
                    &mut r,
                )
                .is_ok();
                state.ps_right_qmf = right_qmf;
                if !decode_ok {
                    // Fallback: duplicate LBR.
                    for i in 0..FRAME_LEN {
                        l[2 * i] = pcm[0][i];
                        l[2 * i + 1] = pcm[0][i];
                        r[2 * i] = pcm[0][i];
                        r[2 * i + 1] = pcm[0][i];
                    }
                }
                sbr_pcm = vec![l, r];
            } else {
                for ch in 0..channels_out {
                    let mut this = std::mem::take(&mut sbr_pcm[ch]);
                    if let Some(sbr) = self.sbr_data[ch].take() {
                        let state = &mut self.sbr_state[ch];
                        if decode_sbr_frame(&pcm[ch], &sbr, state, &mut this).is_err() {
                            // Full-SBR decode failed — fall through to the
                            // spec-compliant upsample-only path (§4.6.18.5)
                            // so the output still carries 2 * FRAME_LEN
                            // samples at 2x the core rate. If that ALSO
                            // fails the buffer stays zeroed, which is a
                            // strictly better tail than half-rate silence
                            // (the buffer was pre-allocated at 2 * FRAME_LEN).
                            let state = &mut self.sbr_state[ch];
                            let _ = decode_sbr_upsample_only(&pcm[ch], state, &mut this);
                        }
                    } else {
                        // No SBR payload this frame but SBR is active —
                        // ISO/IEC 14496-3 §4.6.18.5 "If no SBR data is
                        // found once the decoding process has started,
                        // the SBR Tool can be used for upsampling only".
                        // The trailing frame of an SBR stream commonly
                        // lands here (some encoders drop the final FIL
                        // when bit-budget tightens; the implicit-SBR
                        // ADTS path at 88.2/96 kHz never carries SBR
                        // FIL at all — workspace task #771). Run the
                        // analysis + synthesis QMF without HF generation
                        // so the output count still matches 2 * FRAME_LEN
                        // at 2x rate, rather than zero-order-hold doubling
                        // (which collapses to nearest-neighbour aliasing
                        // and breaks the QMF synthesis history for the
                        // next full-SBR frame).
                        let state = &mut self.sbr_state[ch];
                        if decode_sbr_upsample_only(&pcm[ch], state, &mut this).is_err() {
                            // Buffer is zero-initialised; emit silence.
                            // Hitting this branch means the buffer sizes
                            // didn't satisfy the §4.6.18.5 contract and
                            // the caller should know — but we never
                            // return a partial-size output.
                            for i in 0..FRAME_LEN {
                                this[2 * i] = pcm[ch][i];
                                this[2 * i + 1] = pcm[ch][i];
                            }
                        }
                    }
                    sbr_pcm[ch] = this;
                }
            }

            // HE-AACv2 PS upmix fallback: if PS is active but the QMF-domain
            // path wasn't taken (no PS frame this packet, or mono SBR
            // produced), fall back to the simple time-domain duplication.
            let (out_channels, final_pcm) = if channels_out == 1 && ps_active && sbr_pcm.len() == 1
            {
                let mono = std::mem::take(&mut sbr_pcm[0]);
                let mut l = vec![0.0f32; out_samples];
                let mut r = vec![0.0f32; out_samples];
                let ps_state = &mut self.sbr_state[0].ps;
                apply_ps_simple(&mono, &mut l, &mut r, 0.0, 1.0, ps_state);
                (2usize, vec![l, r])
            } else if channels_out == 1 && ps_active {
                // Stereo already produced above.
                (2usize, sbr_pcm)
            } else {
                (channels_out, sbr_pcm)
            };

            // Clear any leftover SBR slots for next frame.
            for slot in self.sbr_data.iter_mut() {
                *slot = None;
            }
            for slot in self.sbr_pair.iter_mut() {
                *slot = None;
            }
            for slot in self.sbr_ps.iter_mut() {
                *slot = None;
            }

            // Interleave channels (S16 LE).
            //
            // IMDCT + windowing output is scaled to native int16 range — the
            // forward/inverse MDCT pair in this crate deliberately sits at a
            // gain of 2 (unscaled forward + `2/input_n` inverse) for
            // numerical reasons; the `* 0.5` here restores unity gain
            // relative to the encoder input. The synthesis QMF preserves
            // this scale (spec prescribes 1/64 inside the bank — see
            // §4.6.18.4.2 / Fig. 4.43). Clamp to i16 range and cast.
            let mut out_bytes = Vec::with_capacity(out_samples * out_channels * bytes_per_sample);
            for n in 0..out_samples {
                for ch in 0..out_channels {
                    // Round-half-away-from-zero, then clamp + cast. ffmpeg's
                    // av_clipf + lrintf path rounds; the previous `as i16`
                    // truncated toward zero and produced a +1 bias on
                    // every sample in (-1, 0).
                    let v = (final_pcm[ch][n] * 0.5).round().clamp(-32768.0, 32767.0);
                    let s = v as i16;
                    out_bytes.extend_from_slice(&s.to_le_bytes());
                }
            }
            return Ok(Frame::Audio(AudioFrame {
                samples: out_samples as u32,
                pts: pkt.pts,
                data: vec![out_bytes],
            }));
        }

        // IMDCT output is already in native int16 range (see SBR path above
        // for the rationale), so scale-by-½ to compensate the transform
        // pair's 2× gain and cast.
        let mut out_bytes = Vec::with_capacity(FRAME_LEN * channels_out * bytes_per_sample);
        for n in 0..FRAME_LEN {
            for ch in 0..channels_out {
                let v = (pcm[ch][n] * 0.5).round().clamp(-32768.0, 32767.0);
                let s = v as i16;
                out_bytes.extend_from_slice(&s.to_le_bytes());
            }
        }

        // Reset pending SBR data — frames without SBR payloads clear out any
        // stale bits.
        for slot in self.sbr_data.iter_mut() {
            *slot = None;
        }
        for slot in self.sbr_pair.iter_mut() {
            *slot = None;
        }
        for slot in self.sbr_ps.iter_mut() {
            *slot = None;
        }

        Ok(Frame::Audio(AudioFrame {
            samples: FRAME_LEN as u32,
            pts: pkt.pts,
            data: vec![out_bytes],
        }))
    }

    /// Decode one AAC-LD `er_raw_data_block()` payload.
    ///
    /// ISO/IEC 14496-3 §4.4.2.3 Table 4.19 — for `channelConfiguration`
    /// 1 the body is `single_channel_element()`; for `channelConfiguration`
    /// 2 it is `channel_pair_element()`. Both elements use the same ICS
    /// bitstream layout as AAC-LC but feed the LD SWB table + 512/480-
    /// sample LD-IMDCT + sine-windowed overlap-add per §4.6.17.
    ///
    /// The trailing `extension_payload()` loop (`while (cnt >= 1)`) and
    /// `byte_alignment()` are tolerated as a payload tail but not
    /// parsed in detail — this round restricts LD to plain channel
    /// elements; FIL-like extensions in the er_raw_data_block tail are
    /// reserved for SBR/AAC-ELD, neither of which is part of AOT 23.
    fn decode_packet_ld(&mut self, pkt: &Packet) -> Result<Frame> {
        let n = self
            .ld_frame_length
            .map(|f| f.samples() as usize)
            .unwrap_or(512);
        let frame_length = self.ld_frame_length.unwrap_or(LdFrameLength::Samples512);
        if (self.sf_index as usize) >= crate::syntax::SAMPLE_RATES.len() {
            return Err(Error::invalid(
                "AAC-LD: sampling_frequency_index out of table",
            ));
        }
        let swb = swb_ld_for(self.sf_index, frame_length);
        let payload = &pkt.data;
        let mut br = BitReader::new(payload);

        // PCM buffers per channel — sized to LD frame length (n).
        let channel_count = match self.channels {
            1 => 1,
            2 => 2,
            _ => {
                return Err(Error::unsupported(
                    "AAC-LD: only channelConfiguration 1/2 supported in this round",
                ));
            }
        };
        let mut pcm: Vec<Vec<f32>> = (0..channel_count).map(|_| vec![0.0f32; n]).collect();

        match channel_count {
            1 => {
                // single_channel_element() — §4.4.2.2 Table 4.4.
                let _instance_tag = br.read_u32(4)?;
                let mut prev_lag = self.ld_chans[0].ltp_prev_lag;
                let ((info, sf, sec, tns, pulse), ltp) =
                    decode_ics_ld(&mut br, self.sf_index, swb, &mut prev_lag)?;
                self.ld_chans[0].ltp_prev_lag = prev_lag;
                let mut spec = [0.0f32; SPEC_LEN];
                decode_spectrum_long_with_swb(&mut br, &info, &sec, &sf, swb, &mut spec, n)?;
                if let Some(pd) = pulse.as_ref() {
                    apply_pulse_long(&mut spec, pd, self.sf_index, info.max_sfb, &sf)?;
                }
                // PNS — LD reuses the same NOISE_HCB convention.
                apply_pns_long(&mut spec, &info, &sec, &sf, &mut self.pns_rng, None, None);
                // LTP (§4.6.7.3) sits between PNS and TNS in the GA
                // decoder tool chain (§4.1.1.1 Figure 4.2: PNS →
                // prediction → intensity → long-term prediction → TNS).
                // The predicted spectrum is added on enabled bands; PNS /
                // IS bands are skipped because those tools take precedence
                // over prediction (§4.6.7.4.2).
                if let Some(ltp) = ltp.as_ref() {
                    let skip = ltp_skip_bands(&sec, info.max_sfb as usize);
                    apply_ltp_ld(
                        &mut spec[..n],
                        &self.ld_chans[0],
                        ltp,
                        swb,
                        frame_length,
                        &skip,
                    );
                }
                // TNS uses the LD SWB table; same per-band signalling.
                if let Some(tns_data) = tns.as_ref() {
                    apply_tns_long(&mut spec, tns_data, self.sf_index, info.max_sfb, swb);
                }
                // LD IMDCT + overlap-add → n samples of PCM.
                let mut tmp = vec![0.0f32; n];
                let spec_slice = &spec[..n];
                imdct_and_overlap_ld(spec_slice, &mut self.ld_chans[0], &mut tmp, frame_length)?;
                // Update the LTP time-domain history with this frame's
                // reconstructed output (§4.6.7.3 — the "previous fully
                // reconstructed time domain samples").
                self.ld_chans[0].push_ltp_history(&tmp);
                pcm[0].copy_from_slice(&tmp);
            }
            2 => {
                // channel_pair_element() — §4.4.2.3 Table 4.5.
                let _instance_tag = br.read_u32(4)?;
                let common_window = br.read_bit()?;
                if common_window {
                    let mut prev_lags =
                        [self.ld_chans[0].ltp_prev_lag, self.ld_chans[1].ltp_prev_lag];
                    let (info, ltp) =
                        parse_ics_info_ld(&mut br, self.sf_index, swb, true, &mut prev_lags)?;
                    self.ld_chans[0].ltp_prev_lag = prev_lags[0];
                    self.ld_chans[1].ltp_prev_lag = prev_lags[1];
                    let ms_mask_present = br.read_u32(2)? as u8;
                    let max_sfb = info.max_sfb as usize;
                    let groups = info.num_window_groups as usize;
                    let mut ms_used = vec![false; groups * max_sfb];
                    match ms_mask_present {
                        0 => {}
                        1 => {
                            for slot in ms_used.iter_mut() {
                                *slot = br.read_bit()?;
                            }
                        }
                        2 => {
                            for slot in ms_used.iter_mut() {
                                *slot = true;
                            }
                        }
                        _ => {
                            return Err(Error::invalid("AAC-LD: reserved ms_mask_present=3"));
                        }
                    }
                    let infos = [info.clone(), info.clone()];
                    let mut spec = [[0.0f32; SPEC_LEN]; 2];
                    let mut secs: [SectionData; 2] = Default::default();
                    let mut sfs: [Vec<i32>; 2] = Default::default();
                    let mut tns_all: [Option<TnsData>; 2] = [None, None];
                    let mut pulse_all: [Option<PulseData>; 2] = [None, None];
                    for ch in 0..2 {
                        let gg = br.read_u32(8)? as u8;
                        let sec = parse_section_data(&mut br, &infos[ch])?;
                        let sf = parse_scalefactors(&mut br, &infos[ch], &sec, gg)?;
                        let pulse_present = br.read_bit()?;
                        if pulse_present {
                            pulse_all[ch] = Some(parse_pulse_data(&mut br)?);
                        }
                        let tns_present = br.read_bit()?;
                        if tns_present {
                            tns_all[ch] =
                                Some(parse_tns_data(&mut br, infos[ch].window_sequence, 1)?);
                        }
                        // gain_control_data_present — reserved 0 for LD,
                        // tolerated for resilience the same way LC does.
                        let _gain_control = br.read_bit()?;
                        decode_spectrum_long_with_swb(
                            &mut br,
                            &infos[ch],
                            &sec,
                            &sf,
                            swb,
                            &mut spec[ch],
                            n,
                        )?;
                        if let Some(pd) = pulse_all[ch].as_ref() {
                            apply_pulse_long(
                                &mut spec[ch],
                                pd,
                                self.sf_index,
                                infos[ch].max_sfb,
                                &sf,
                            )?;
                        }
                        secs[ch] = sec;
                        sfs[ch] = sf;
                    }
                    // PNS — channel 0 first (with mirror to ch1 for
                    // correlated-noise bands), then leftover ch1 bands.
                    let (s0, s1) = spec.split_at_mut(1);
                    apply_pns_long(
                        &mut s0[0],
                        &infos[0],
                        &secs[0],
                        &sfs[0],
                        &mut self.pns_rng,
                        Some(&mut s1[0]),
                        Some(&ms_used),
                    );
                    apply_pns_long_ch1_leftover(
                        &mut s1[0],
                        &infos[1],
                        &secs[0],
                        &secs[1],
                        &sfs[1],
                        &mut self.pns_rng,
                    );
                    // IS (§4.6.8.2.3) then M/S (§4.6.8.1.3) — long-only;
                    // the LD path runs the same helpers as the LC long
                    // path because the codebook semantics are identical.
                    apply_intensity_stereo(
                        &infos[0],
                        &secs,
                        &sfs,
                        &ms_used,
                        ms_mask_present,
                        &mut spec,
                    );
                    apply_ms_stereo(&infos[0], &secs, &ms_used, &mut spec);
                    // LTP per channel (§4.6.7.3) — after M/S + intensity,
                    // before TNS (§4.1.1.1 Figure 4.2). Each channel
                    // predicts from its own reconstructed-output history,
                    // so the X_est add operates on the de-matrixed L / R
                    // spectrum. PNS / IS bands are skipped (those tools
                    // take precedence per §4.6.7.4.2).
                    for ch in 0..2 {
                        if let Some(ltp) = ltp[ch].as_ref() {
                            let skip = ltp_skip_bands(&secs[ch], infos[ch].max_sfb as usize);
                            apply_ltp_ld(
                                &mut spec[ch][..n],
                                &self.ld_chans[ch],
                                ltp,
                                swb,
                                frame_length,
                                &skip,
                            );
                        }
                    }
                    // TNS per channel.
                    for ch in 0..2 {
                        if let Some(tns) = tns_all[ch].as_ref() {
                            apply_tns_long(
                                &mut spec[ch],
                                tns,
                                self.sf_index,
                                infos[ch].max_sfb,
                                swb,
                            );
                        }
                    }
                    // LD IMDCT + overlap per channel.
                    for ch in 0..2 {
                        let mut tmp = vec![0.0f32; n];
                        let spec_slice = &spec[ch][..n];
                        imdct_and_overlap_ld(
                            spec_slice,
                            &mut self.ld_chans[ch],
                            &mut tmp,
                            frame_length,
                        )?;
                        self.ld_chans[ch].push_ltp_history(&tmp);
                        pcm[ch].copy_from_slice(&tmp);
                    }
                } else {
                    // Independent ICS per channel — same shape as the LC
                    // !common_window CPE branch but with LD ICS info.
                    let mut spec = [[0.0f32; SPEC_LEN]; 2];
                    let mut infos: [IcsInfo; 2] = Default::default();
                    let mut secs: [SectionData; 2] = Default::default();
                    let mut tns_all: [Option<TnsData>; 2] = [None, None];
                    let mut ltp_all: [Option<LtpData>; 2] = [None, None];
                    for ch in 0..2 {
                        let mut prev_lag = self.ld_chans[ch].ltp_prev_lag;
                        let ((info, sf, sec, tns, pulse), ltp) =
                            decode_ics_ld(&mut br, self.sf_index, swb, &mut prev_lag)?;
                        self.ld_chans[ch].ltp_prev_lag = prev_lag;
                        decode_spectrum_long_with_swb(
                            &mut br,
                            &info,
                            &sec,
                            &sf,
                            swb,
                            &mut spec[ch],
                            n,
                        )?;
                        if let Some(pd) = pulse.as_ref() {
                            apply_pulse_long(&mut spec[ch], pd, self.sf_index, info.max_sfb, &sf)?;
                        }
                        apply_pns_long(
                            &mut spec[ch],
                            &info,
                            &sec,
                            &sf,
                            &mut self.pns_rng,
                            None,
                            None,
                        );
                        infos[ch] = info;
                        secs[ch] = sec;
                        tns_all[ch] = tns;
                        ltp_all[ch] = ltp;
                    }
                    for ch in 0..2 {
                        // LTP after PNS, before TNS (no M/S on independent
                        // ICS) — §4.1.1.1 Figure 4.2 tool order.
                        if let Some(ltp) = ltp_all[ch].as_ref() {
                            let skip = ltp_skip_bands(&secs[ch], infos[ch].max_sfb as usize);
                            apply_ltp_ld(
                                &mut spec[ch][..n],
                                &self.ld_chans[ch],
                                ltp,
                                swb,
                                frame_length,
                                &skip,
                            );
                        }
                        if let Some(tns) = tns_all[ch].as_ref() {
                            apply_tns_long(
                                &mut spec[ch],
                                tns,
                                self.sf_index,
                                infos[ch].max_sfb,
                                swb,
                            );
                        }
                        let mut tmp = vec![0.0f32; n];
                        let spec_slice = &spec[ch][..n];
                        imdct_and_overlap_ld(
                            spec_slice,
                            &mut self.ld_chans[ch],
                            &mut tmp,
                            frame_length,
                        )?;
                        self.ld_chans[ch].push_ltp_history(&tmp);
                        pcm[ch].copy_from_slice(&tmp);
                    }
                }
            }
            _ => unreachable!(),
        }

        // Convert to interleaved S16. Reuse the same /2 IMDCT scale-back
        // convention as the LC path (forward * 2, inverse * (2/N), net
        // gain of 2; the 0.5 here restores unity).
        let bytes_per_sample = SampleFormat::S16.bytes_per_sample();
        let mut out_bytes = Vec::with_capacity(n * channel_count * bytes_per_sample);
        for s in 0..n {
            for ch in 0..channel_count {
                let v = (pcm[ch][s] * 0.5).round().clamp(-32768.0, 32767.0);
                let bytes = (v as i16).to_le_bytes();
                out_bytes.extend_from_slice(&bytes);
            }
        }
        Ok(Frame::Audio(AudioFrame {
            samples: n as u32,
            pts: pkt.pts,
            data: vec![out_bytes],
        }))
    }
}

/// Parsed individual_channel_stream: everything except the spectrum.
/// Returned by [`decode_ics`]; the spectrum is filled in separately by
/// `fill_spectrum`.
pub type DecodedIcs = (
    IcsInfo,
    Vec<i32>,
    SectionData,
    Option<TnsData>,
    Option<PulseData>,
);

/// Decode a single-channel ICS into (info, scalefactors, section_data, tns_data, pulse_data).
/// Reads global_gain, ics_info, section_data, scalefactors, pulse / TNS / gain-control
/// fields. Spectrum decoding is left to caller.
///
/// Exposed at crate-public scope so test harnesses (e.g. the r24 SBR FIL
/// diff probe) can replay the production decoder's element walk without
/// duplicating the ICS state-machine.
pub fn decode_ics(br: &mut BitReader<'_>, sf_index: u8, is_in_cpe: bool) -> Result<DecodedIcs> {
    let global_gain = br.read_u32(8)? as u8;
    let info = parse_ics_info(br, sf_index)?;
    let sec = parse_section_data(br, &info)?;
    let sf = parse_scalefactors(br, &info, &sec, global_gain)?;
    let pulse_present = br.read_bit()?;
    let pulse = if pulse_present {
        Some(parse_pulse_data(br)?)
    } else {
        None
    };
    let tns_present = br.read_bit()?;
    let tns = if tns_present {
        // Per §4.6.9.1, tns_data has n_windows == num_windows (8 for short, 1 otherwise).
        let n_windows = if info.window_sequence.is_eight_short() {
            8
        } else {
            1
        };
        Some(parse_tns_data(br, info.window_sequence, n_windows)?)
    } else {
        None
    };
    // Per ISO/IEC 14496-3 §4.5.2.1, AAC-LC streams must have
    // `gain_control_data_present == 0` — gain control is reserved for
    // the AAC-SSR profile (object_type 3), which we don't decode. A
    // hard reject here turned ffmpeg-accepted-but-non-conformant
    // streams into "ours produced 0 frames" failures under the fuzz
    // oracle (`fuzz_targets/ffmpeg_oracle_decode.rs`); libavcodec
    // tolerates the bit and produces *some* PCM. We mirror that
    // behaviour: ignore the bit, leave the bitstream cursor where it
    // is (we don't attempt to parse the §4.5.2.6.4
    // `gain_control_data()` block, which would corrupt the rest of
    // the spectral_data alignment anyway). Downstream parsing
    // recovers what it can; on a real AAC-LC stream the bit is
    // always zero so this is a no-op.
    let _gain_control = br.read_bit()?;
    let _ = is_in_cpe;
    Ok((info, sf, sec, tns, pulse))
}

/// Build the per-sfb mask of bands that LTP must NOT predict (long
/// window). Per ISO/IEC 14496-3 §4.6.7.4.2, when both LTP and PNS are
/// enabled on the same band PNS takes precedence (no prediction), and
/// likewise intensity stereo takes precedence over LTP. Those bands are
/// already reconstructed by the PNS / IS passes that run *before* LTP in
/// the GA decoder tool chain, so adding the predicted spectrum on top
/// would corrupt them. Long-window LD has a single window group, so the
/// mask is simply per-sfb.
fn ltp_skip_bands(sec: &SectionData, max_sfb: usize) -> Vec<bool> {
    let mut skip = vec![false; max_sfb];
    for (sfb, slot) in skip.iter_mut().enumerate() {
        let cb = sec.sfb_cb.get(sfb).copied().unwrap_or(0);
        *slot = matches!(cb, NOISE_HCB | INTENSITY_HCB | INTENSITY_HCB2);
    }
    skip
}

/// LD variant of [`decode_ics`] — uses `parse_ics_info_ld` for the
/// header and the LD SWB offset table for max_sfb clamping. The rest of
/// the per-channel side-info layout (global_gain, section_data,
/// scalefactors, pulse, TNS, gain_control) matches AAC-LC bit-for-bit.
///
/// This is the single-channel (`common_window == false`) entry point, so
/// at most one `ltp_data()` block can appear in the ics_info; the parsed
/// long-term-prediction side info (if any) is returned alongside the rest
/// of the ICS. `prev_lag` carries the channel's previous-frame LTP lag
/// for the `ltp_lag_update == 0` reuse case and is updated in place.
pub fn decode_ics_ld(
    br: &mut BitReader<'_>,
    sf_index: u8,
    swb: &[u16],
    prev_lag: &mut u16,
) -> Result<(DecodedIcs, Option<LtpData>)> {
    let global_gain = br.read_u32(8)? as u8;
    let mut prev_lags = [*prev_lag, 0];
    let (info, ltp) = parse_ics_info_ld(br, sf_index, swb, false, &mut prev_lags)?;
    *prev_lag = prev_lags[0];
    let ltp = ltp[0].clone();
    let sec = parse_section_data(br, &info)?;
    let sf = parse_scalefactors(br, &info, &sec, global_gain)?;
    let pulse_present = br.read_bit()?;
    let pulse = if pulse_present {
        Some(parse_pulse_data(br)?)
    } else {
        None
    };
    let tns_present = br.read_bit()?;
    let tns = if tns_present {
        // AAC-LD is long-only; n_windows is always 1 per §4.6.17.2.2.
        Some(parse_tns_data(br, info.window_sequence, 1)?)
    } else {
        None
    };
    // gain_control_data_present must be 0 for LD (§4.6.17 — no SSR
    // gain-control on LD). Consume the bit defensively to keep the
    // cursor aligned even on a non-conformant encoder.
    let _gain_control = br.read_bit()?;
    Ok(((info, sf, sec, tns, pulse), ltp))
}

/// Fill the spectrum coefficients into `spec` from the bit-stream `br`.
/// Companion to [`decode_ics`]; exposed pub for test harnesses.
pub fn fill_spectrum(
    br: &mut BitReader<'_>,
    info: &IcsInfo,
    sec: &SectionData,
    sf: &[i32],
    spec: &mut [f32; SPEC_LEN],
) -> Result<()> {
    if info.window_sequence == WindowSequence::EightShort {
        decode_spectrum_short(br, info, sec, sf, spec)
    } else {
        decode_spectrum_long(br, info, sec, sf, spec)
    }
}

/// Fill any NOISE_HCB bands in channel 1 that were not handled by the shared
/// channel-0 PNS pass — only relevant when the two channels have diverging
/// NOISE_HCB band sets, which is rare but spec-legal for CPE common_window.
fn apply_pns_long_ch1_leftover(
    spec1: &mut [f32; SPEC_LEN],
    info: &IcsInfo,
    sec0: &SectionData,
    sec1: &SectionData,
    sf1: &[i32],
    rng: &mut PnsRng,
) {
    let swb = SWB_LONG[info.sf_index as usize];
    let max_sfb = info.max_sfb as usize;
    for sfb in 0..max_sfb {
        let cb1 = sec1.sfb_cb.get(sfb).copied().unwrap_or(0);
        let cb0 = sec0.sfb_cb.get(sfb).copied().unwrap_or(0);
        if cb1 == NOISE_HCB && cb0 != NOISE_HCB {
            let start = swb[sfb] as usize;
            let end = swb[sfb + 1] as usize;
            let gain = crate::pns::pns_gain(sf1[sfb]);
            for s in start..end {
                spec1[s] = rng.next_float() * gain;
            }
        }
    }
}

/// Apply Intensity Stereo (IS) decoding in-place. Spec §4.6.8.2.3.
///
/// For each scalefactor band where channel 1's (right-channel) codebook is
/// `INTENSITY_HCB` (15) or `INTENSITY_HCB2` (14), synthesise channel-1
/// coefficients from channel-0:
/// ```text
/// scale          = is_intensity * invert_intensity * 0.5^(0.25 * is_position)
/// spec[1][k]     = scale * spec[0][k]
/// is_intensity   = +1 (cb=15), -1 (cb=14)
/// invert_intensity = (1 - 2*ms_used)   only if  ms_mask_present == 1
///                  = +1                otherwise (mask=0 or mask=2)
/// ```
/// Note that per §4.6.8.2.3 the `ms_used` "phase-invert" only kicks in when
/// `ms_mask_present == 1` — when the mask is "all zeros" (0) or
/// "all ones" (2), `invert_intensity == +1` and the IS sign is taken
/// purely from the codebook id.
///
/// `is_position` is the accumulated scalefactor value stored in
/// `sfs[1][g·max_sfb + sfb]` (IS positions share the scalefactor codebook
/// but use a separate accumulator — see [`parse_scalefactors`]).
///
/// Must run BEFORE [`apply_ms_stereo`] — the M/S pass explicitly skips IS
/// bands and depends on them already carrying the IS-synthesised channel-1
/// values.
fn apply_intensity_stereo(
    info: &IcsInfo,
    secs: &[SectionData; 2],
    sfs: &[Vec<i32>; 2],
    ms_used: &[bool],
    ms_mask_present: u8,
    spec: &mut [[f32; SPEC_LEN]; 2],
) {
    let max_sfb = info.max_sfb as usize;
    let groups = info.num_window_groups as usize;
    let starts = crate::ics::group_starts(info);
    let is_short = info.window_sequence.is_eight_short();
    let swb = if is_short {
        SWB_SHORT[info.sf_index as usize]
    } else {
        SWB_LONG[info.sf_index as usize]
    };

    for g in 0..groups {
        let group_len = info.window_group_length[g] as usize;
        for sfb in 0..max_sfb {
            let cb = secs[1].sfb_cb[g * max_sfb + sfb];
            if cb != INTENSITY_HCB && cb != INTENSITY_HCB2 {
                continue;
            }
            // cb 15 → is_intensity = +1, cb 14 → is_intensity = -1.
            let is_intensity_sign: f32 = if cb == INTENSITY_HCB { 1.0 } else { -1.0 };
            // §4.6.8.2.3 — invert_intensity is (1 - 2*ms_used) ONLY when
            // ms_mask_present == 1. With mask = 0 (all zero) or mask = 2
            // (all ones — which forces ms_used = true everywhere) it's +1.
            let invert: f32 = if ms_mask_present == 1 && ms_used[g * max_sfb + sfb] {
                -1.0
            } else {
                1.0
            };
            let is_position = sfs[1][g * max_sfb + sfb];
            // scale = sign · 2^(-is_position/4)  (== 0.5^(0.25*is_position))
            let scale = (2.0f32).powf(-(is_position as f32) * 0.25);
            let coef = is_intensity_sign * invert * scale;
            let band_start = swb[sfb] as usize;
            let band_end = swb[sfb + 1] as usize;
            if is_short {
                let win_start_offset = starts[g] * 128;
                for w in 0..group_len {
                    for j in band_start..band_end {
                        let idx = win_start_offset + w * 128 + j;
                        spec[1][idx] = coef * spec[0][idx];
                    }
                }
            } else {
                for j in band_start..band_end {
                    spec[1][j] = coef * spec[0][j];
                }
            }
        }
    }
}

/// Apply M/S stereo decoding in-place. Spec §4.6.8.1.3 (decoding pseudo-code)
/// + §4.6.8.2.3 (`is_intensity`) + §4.6.13.3 (`is_noise`).
///
/// Spec pseudo-code (§4.6.8.1.3):
/// ```text
///   for each band:
///     if ((ms_used[g][sfb] || ms_mask_present == 2)
///         && !is_intensity(g,sfb) && !is_noise(g,sfb)) {
///        L = M + S; R = M - S;
///     }
/// ```
/// Where:
/// * `is_intensity` is determined by the **right channel's** codebook
///   (`secs[1].sfb_cb`) being `INTENSITY_HCB` (15) or `INTENSITY_HCB2`
///   (14) — **NOT** the left channel's codebook (§4.6.8.2.3).
/// * `is_noise` is true if either channel's codebook is `NOISE_HCB` (13).
///   Bug history: an earlier version checked only the left channel's
///   codebook for IS, so a band with `(secs[0].cb = 1, secs[1].cb = 15)`
///   wrongly went through M/S — the IS pass had already synthesised
///   `spec[1] = scale * spec[0]`, and the subsequent `L+S, L-S` mix
///   produced enormous spurious energy that overflowed the IMDCT to ±32k.
fn apply_ms_stereo(
    info: &IcsInfo,
    secs: &[SectionData; 2],
    ms_used: &[bool],
    spec: &mut [[f32; SPEC_LEN]; 2],
) {
    let max_sfb = info.max_sfb as usize;
    let groups = info.num_window_groups as usize;
    let starts = crate::ics::group_starts(info);
    let is_short = info.window_sequence.is_eight_short();
    let swb = if is_short {
        SWB_SHORT[info.sf_index as usize]
    } else {
        SWB_LONG[info.sf_index as usize]
    };

    for g in 0..groups {
        let group_len = info.window_group_length[g] as usize;
        for sfb in 0..max_sfb {
            if !ms_used[g * max_sfb + sfb] {
                continue;
            }
            let cb0 = secs[0].sfb_cb[g * max_sfb + sfb];
            let cb1 = secs[1].sfb_cb[g * max_sfb + sfb];
            // is_intensity → check RIGHT channel codebook only.
            if cb1 == INTENSITY_HCB || cb1 == INTENSITY_HCB2 {
                continue;
            }
            // is_noise → if EITHER channel is PNS, skip (PNS owns the band).
            if cb0 == NOISE_HCB || cb1 == NOISE_HCB {
                continue;
            }
            // ZERO_HCB on both channels: M/S of (0, 0) is (0, 0) — harmless
            // but skip for clarity / to avoid a tiny needless write.
            if cb0 == ZERO_HCB && cb1 == ZERO_HCB {
                continue;
            }
            let band_start = swb[sfb] as usize;
            let band_end = swb[sfb + 1] as usize;
            if is_short {
                let win_start_offset = starts[g] * 128;
                for w in 0..group_len {
                    for j in band_start..band_end {
                        let idx = win_start_offset + w * 128 + j;
                        let m = spec[0][idx];
                        let s = spec[1][idx];
                        spec[0][idx] = m + s;
                        spec[1][idx] = m - s;
                    }
                }
            } else {
                for j in band_start..band_end {
                    let m = spec[0][j];
                    let s = spec[1][j];
                    spec[0][j] = m + s;
                    spec[1][j] = m - s;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal long-window IcsInfo for unit-testing the stereo passes.
    fn long_info(sf_index: u8, max_sfb: u8) -> IcsInfo {
        let mut info = IcsInfo {
            sf_index,
            window_sequence: WindowSequence::OnlyLong,
            max_sfb,
            num_window_groups: 1,
            ..IcsInfo::default()
        };
        info.window_group_length[0] = 1;
        info
    }

    fn section_with_cb(max_sfb: usize, overrides: &[(usize, u8)]) -> SectionData {
        let mut sfb_cb = vec![1u8; max_sfb]; // default to codebook 1 (scaled)
        for (sfb, cb) in overrides.iter() {
            sfb_cb[*sfb] = *cb;
        }
        SectionData { sfb_cb }
    }

    #[test]
    fn intensity_stereo_cb15_positive_sign() {
        // IS band at sfb=2 with cb=15 (positive sign) and is_position=8 →
        // scale = 2^(-8/4) = 0.25, sign = +1.
        let max_sfb = 5u8;
        let info = long_info(4, max_sfb); // sf_index=4 → 44.1 kHz
        let swb = SWB_LONG[info.sf_index as usize];
        let mut spec: [[f32; SPEC_LEN]; 2] = [[0.0; SPEC_LEN]; 2];
        let band_start = swb[2] as usize;
        let band_end = swb[3] as usize;
        for k in band_start..band_end {
            spec[0][k] = (k as f32) * 0.1;
        }
        let secs = [
            section_with_cb(max_sfb as usize, &[]),
            section_with_cb(max_sfb as usize, &[(2, INTENSITY_HCB)]),
        ];
        let mut sfs: [Vec<i32>; 2] = [vec![0; max_sfb as usize], vec![0; max_sfb as usize]];
        sfs[1][2] = 8; // is_position
        let ms_used = vec![false; max_sfb as usize];
        apply_intensity_stereo(&info, &secs, &sfs, &ms_used, 0, &mut spec);
        for k in band_start..band_end {
            let expected = 0.25 * spec[0][k];
            assert!(
                (spec[1][k] - expected).abs() < 1e-6,
                "k={k}: got {} want {}",
                spec[1][k],
                expected
            );
        }
        // Neighbouring bands must remain zero in channel 1.
        let pre = swb[1] as usize;
        assert_eq!(spec[1][pre], 0.0);
    }

    #[test]
    fn intensity_stereo_cb14_negative_sign() {
        let max_sfb = 3u8;
        let info = long_info(4, max_sfb);
        let swb = SWB_LONG[info.sf_index as usize];
        let mut spec: [[f32; SPEC_LEN]; 2] = [[0.0; SPEC_LEN]; 2];
        for k in swb[0] as usize..swb[1] as usize {
            spec[0][k] = 2.0;
        }
        let secs = [
            section_with_cb(max_sfb as usize, &[]),
            section_with_cb(max_sfb as usize, &[(0, INTENSITY_HCB2)]),
        ];
        let mut sfs: [Vec<i32>; 2] = [vec![0; max_sfb as usize], vec![0; max_sfb as usize]];
        sfs[1][0] = 0; // is_position=0 → scale = 1.0
        let ms_used = vec![false; max_sfb as usize];
        apply_intensity_stereo(&info, &secs, &sfs, &ms_used, 0, &mut spec);
        for k in swb[0] as usize..swb[1] as usize {
            assert!((spec[1][k] - (-2.0)).abs() < 1e-6);
        }
    }

    #[test]
    fn intensity_stereo_ms_mask_flips_sign() {
        // cb 15 (sign=+1) with ms_mask_present=1 and ms_used=true on that band
        // should flip the sign to -1.
        let max_sfb = 2u8;
        let info = long_info(4, max_sfb);
        let swb = SWB_LONG[info.sf_index as usize];
        let mut spec: [[f32; SPEC_LEN]; 2] = [[0.0; SPEC_LEN]; 2];
        for k in swb[0] as usize..swb[1] as usize {
            spec[0][k] = 1.0;
        }
        let secs = [
            section_with_cb(max_sfb as usize, &[]),
            section_with_cb(max_sfb as usize, &[(0, INTENSITY_HCB)]),
        ];
        let mut sfs: [Vec<i32>; 2] = [vec![0; max_sfb as usize], vec![0; max_sfb as usize]];
        sfs[1][0] = 4; // scale = 2^(-1) = 0.5
        let mut ms_used = vec![false; max_sfb as usize];
        ms_used[0] = true;
        apply_intensity_stereo(&info, &secs, &sfs, &ms_used, 1, &mut spec);
        for k in swb[0] as usize..swb[1] as usize {
            assert!((spec[1][k] - (-0.5)).abs() < 1e-6, "got {}", spec[1][k]);
        }
    }

    #[test]
    fn intensity_stereo_non_is_bands_untouched() {
        // A band coded with a regular codebook (cb=1) must not be modified.
        let max_sfb = 2u8;
        let info = long_info(4, max_sfb);
        let swb = SWB_LONG[info.sf_index as usize];
        let mut spec: [[f32; SPEC_LEN]; 2] = [[0.0; SPEC_LEN]; 2];
        for k in swb[0] as usize..swb[1] as usize {
            spec[0][k] = 5.0;
            spec[1][k] = 3.0; // pre-existing ch-1 content from normal decode
        }
        let secs = [
            section_with_cb(max_sfb as usize, &[]),
            section_with_cb(max_sfb as usize, &[]),
        ];
        let sfs: [Vec<i32>; 2] = [vec![0; max_sfb as usize], vec![0; max_sfb as usize]];
        let ms_used = vec![false; max_sfb as usize];
        apply_intensity_stereo(&info, &secs, &sfs, &ms_used, 0, &mut spec);
        for k in swb[0] as usize..swb[1] as usize {
            assert_eq!(spec[1][k], 3.0);
        }
    }

    /// IS phase invert (`ms_used` re-purposed as sign toggle on IS bands)
    /// must trigger ONLY when `ms_mask_present == 1`. With mask=2 (all-ones)
    /// the spec sets `invert_intensity = +1` even though every `ms_used` bit
    /// is implicitly true.
    ///
    /// Bug history (solana-ad.mp4 round 21): the IS pass flipped the sign
    /// whenever `ms_mask_present != 0 && ms_used[sfb]`, which incorrectly
    /// inverted IS phase on ms_mask=2 streams — combined with the M/S
    /// codebook check looking at the wrong channel, this produced full
    /// ±32k saturation on real-content frames with IS bands.
    #[test]
    fn intensity_stereo_no_phase_invert_on_mask2() {
        let max_sfb = 2u8;
        let info = long_info(4, max_sfb);
        let swb = SWB_LONG[info.sf_index as usize];
        let mut spec: [[f32; SPEC_LEN]; 2] = [[0.0; SPEC_LEN]; 2];
        for k in swb[0] as usize..swb[1] as usize {
            spec[0][k] = 1.0;
        }
        let secs = [
            section_with_cb(max_sfb as usize, &[]),
            section_with_cb(max_sfb as usize, &[(0, INTENSITY_HCB)]),
        ];
        let mut sfs: [Vec<i32>; 2] = [vec![0; max_sfb as usize], vec![0; max_sfb as usize]];
        sfs[1][0] = 0; // is_position=0 → scale=1.0
        let ms_used = vec![true; max_sfb as usize]; // mask=2 fills all with true
        apply_intensity_stereo(&info, &secs, &sfs, &ms_used, 2, &mut spec);
        // Mask=2: invert_intensity = +1, so r = +1 * 1.0 * spec[0] = +1.0.
        // (If ms_used were honoured we'd erroneously get -1.0 here.)
        for k in swb[0] as usize..swb[1] as usize {
            assert!(
                (spec[1][k] - 1.0).abs() < 1e-6,
                "mask=2 should not flip IS phase, got {}",
                spec[1][k]
            );
        }
    }

    /// `apply_ms_stereo` must consult the **right** channel's codebook when
    /// deciding whether a band is intensity-stereo (per §4.6.8.2.3
    /// `is_intensity()`). A band where `secs[0]` is a regular codebook but
    /// `secs[1]` is `INTENSITY_HCB` / `INTENSITY_HCB2` MUST be skipped:
    /// the IS pass already synthesised `spec[1] = scale * spec[0]`, and
    /// running `(L+R, L-R)` on top of that produces enormous spurious
    /// energy.
    ///
    /// Bug history: an earlier version checked only `secs[0]`, so bands
    /// like `(secs[0]=1 regular, secs[1]=15 IS)` went through M/S and
    /// blew up the IMDCT input.
    #[test]
    fn ms_stereo_skips_is_bands_indicated_by_right_channel() {
        let max_sfb = 3u8;
        let info = long_info(4, max_sfb);
        let swb = SWB_LONG[info.sf_index as usize];
        let mut spec: [[f32; SPEC_LEN]; 2] = [[0.0; SPEC_LEN]; 2];
        // Band 1 (sfb=1) is the IS band. Pre-fill both channels' values to
        // a known constant so we can assert M/S doesn't mutate them.
        let band_start = swb[1] as usize;
        let band_end = swb[2] as usize;
        for k in band_start..band_end {
            spec[0][k] = 7.0;
            spec[1][k] = 11.0; // simulated IS-synthesised content
        }
        let secs = [
            // Channel 0 codebook for sfb 1 is regular (1)
            section_with_cb(max_sfb as usize, &[]),
            // Channel 1 codebook for sfb 1 is IS (15)
            section_with_cb(max_sfb as usize, &[(1, INTENSITY_HCB)]),
        ];
        let ms_used = vec![true; max_sfb as usize];
        apply_ms_stereo(&info, &secs, &ms_used, &mut spec);
        for k in band_start..band_end {
            assert_eq!(spec[0][k], 7.0, "M/S should skip IS band on ch0");
            assert_eq!(spec[1][k], 11.0, "M/S should skip IS band on ch1");
        }
    }

    /// `apply_ms_stereo` must skip bands where EITHER channel uses
    /// `NOISE_HCB`. The PNS pass already filled the band with shaped
    /// noise; mixing left/right at this point would corrupt that fill.
    /// Spec §4.6.8.1.3 (`!is_noise(g, sfb)`).
    #[test]
    fn ms_stereo_skips_noise_bands_on_either_channel() {
        let max_sfb = 3u8;
        let info = long_info(4, max_sfb);
        let swb = SWB_LONG[info.sf_index as usize];
        let mut spec: [[f32; SPEC_LEN]; 2] = [[0.0; SPEC_LEN]; 2];
        let band_start = swb[1] as usize;
        let band_end = swb[2] as usize;
        for k in band_start..band_end {
            spec[0][k] = 0.5;
            spec[1][k] = 0.7;
        }
        // sfb 1: ch0 is regular, ch1 is NOISE_HCB → must skip.
        let secs = [
            section_with_cb(max_sfb as usize, &[]),
            section_with_cb(max_sfb as usize, &[(1, NOISE_HCB)]),
        ];
        let ms_used = vec![true; max_sfb as usize];
        apply_ms_stereo(&info, &secs, &ms_used, &mut spec);
        for k in band_start..band_end {
            assert_eq!(spec[0][k], 0.5);
            assert_eq!(spec[1][k], 0.7);
        }
    }
}

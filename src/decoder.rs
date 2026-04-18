//! Top-level AAC-LC packet decoder. Wires the ADTS / ASC parser, the SCE /
//! CPE element decoders, and the IMDCT/overlap state into a single
//! `oxideav_codec::Decoder` impl.
//!
//! ISO/IEC 14496-3 §4.5.2.1 (raw_data_block) and §4.6.

use oxideav_codec::Decoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, Packet, Result, SampleFormat, TimeBase,
};

use crate::adts::parse_adts_header;
use crate::asc::parse_asc;
use crate::bitreader::BitReader;
use crate::ics::{
    decode_spectrum_long, decode_spectrum_short, parse_ics_info, parse_scalefactors,
    parse_section_data, IcsInfo, SectionData, INTENSITY_HCB, INTENSITY_HCB2, NOISE_HCB, SPEC_LEN,
    ZERO_HCB,
};
use crate::pns::{apply_pns_long, apply_pns_short, PnsRng};
use crate::pulse::{apply_pulse_long, parse_pulse_data, PulseData};
use crate::sfband::{SWB_LONG, SWB_SHORT};
use crate::syntax::{ElementType, WindowSequence, AOT_AAC_LC};
use crate::synth::{imdct_and_overlap, ChannelState, FRAME_LEN};
use crate::tns::{apply_tns_long, apply_tns_short, parse_tns_data, TnsData};

pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    // Figure out the stream config. Two paths:
    //   (a) extradata holds an AudioSpecificConfig (MP4 path).
    //   (b) ADTS — config will come from the first packet's ADTS header.
    let (sf_index, channels, object_type) = if !params.extradata.is_empty() {
        let asc = parse_asc(&params.extradata)?;
        if asc.sbr_present || asc.ps_present {
            return Err(Error::unsupported(
                "AAC: SBR/PS (HE-AAC v1/v2) not supported",
            ));
        }
        if asc.object_type != AOT_AAC_LC {
            return Err(Error::unsupported(
                "AAC: only AAC-LC profile (object_type=2) supported",
            ));
        }
        (
            sample_rate_to_index(asc.sampling_frequency).unwrap_or(asc.sampling_frequency_index),
            asc.channel_configuration,
            asc.object_type,
        )
    } else {
        // Will be filled in after seeing the first ADTS frame.
        (0xFF, 0, 0)
    };

    Ok(Box::new(AacDecoder {
        codec_id: params.codec_id.clone(),
        time_base: TimeBase::new(1, params.sample_rate.unwrap_or(44_100) as i64),
        pending: None,
        eof: false,
        sf_index,
        channels,
        object_type,
        chans: vec![ChannelState::new(); 8],
        configured: !params.extradata.is_empty(),
        pns_rng: PnsRng::new(),
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
    eof: bool,
    sf_index: u8,
    channels: u8,
    object_type: u8,
    chans: Vec<ChannelState>,
    configured: bool,
    pns_rng: PnsRng,
}

impl Decoder for AacDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        if self.pending.is_some() {
            return Err(Error::other(
                "AAC decoder: receive_frame must be called before sending another packet",
            ));
        }
        self.pending = Some(packet.clone());
        Ok(())
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        let Some(pkt) = self.pending.take() else {
            return if self.eof {
                Err(Error::Eof)
            } else {
                Err(Error::NeedMore)
            };
        };
        self.decode_packet(&pkt)
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
        self.pending = None;
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
                self.sf_index = hdr.sampling_freq_index;
                self.channels = hdr.channel_configuration;
                self.object_type = hdr.object_type;
                self.configured = true;
                self.time_base = TimeBase::new(1, hdr.sample_rate().unwrap_or(44_100) as i64);
                (hdr.header_length(), hdr.frame_length.min(data.len()))
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

        loop {
            let id = br.read_u32(3)?;
            let kind = ElementType::from_id(id);
            match kind {
                ElementType::Sce => {
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
                        apply_pns_long(&mut spec, &info, &sec, &sf, &mut self.pns_rng, None, None);
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
                            let _gain_control = br.read_bit()?;
                            if _gain_control {
                                return Err(Error::unsupported(
                                    "AAC: gain_control_data_present in LC stream",
                                ));
                            }
                            secs[ch] = sec;
                            sfs[ch] = sf;
                            fill_spectrum(&mut br, &infos[ch], &secs[ch], &sfs[ch], &mut spec[ch])?;
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
                                apply_pns_short(&mut spec[ch], &info, &sec, &sf, &mut self.pns_rng);
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
                        if got_channels + 2 > pcm.len() {
                            return Err(Error::invalid(
                                "AAC: CPE would overflow 8 channel slots",
                            ));
                        }
                        got_channels += 2;
                    }
                }
                ElementType::Lfe => {
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
                        return Err(Error::invalid(
                            "AAC: LFE would overflow 8 channel slots",
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
                    let mut count = br.read_u32(4)?;
                    if count == 15 {
                        count += br.read_u32(8)? - 1;
                    }
                    // SBR extension payloads start with extension_type=0xD/0xE/0xF —
                    // we treat any non-empty extension as SBR refusal if it claims so.
                    if count > 0 {
                        // Peek extension_type (4 bits) without committing.
                        let peeked = br.peek_u32(4)?;
                        let is_sbr = peeked == 0xD || peeked == 0xE;
                        if is_sbr {
                            return Err(Error::unsupported(
                                "AAC: SBR extension payload — HE-AAC not supported",
                            ));
                        }
                        // Otherwise skip.
                        for _ in 0..count {
                            br.read_u32(8)?;
                        }
                    }
                }
                ElementType::End => break,
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
            1
        } else {
            got_channels.min(expected.max(1))
        };
        let bytes_per_sample = SampleFormat::S16.bytes_per_sample();
        let mut out_bytes = Vec::with_capacity(FRAME_LEN * channels_out * bytes_per_sample);
        for n in 0..FRAME_LEN {
            for ch in 0..channels_out {
                let v = pcm[ch][n].clamp(-1.0, 1.0);
                let s = (v * 32767.0) as i16;
                out_bytes.extend_from_slice(&s.to_le_bytes());
            }
        }

        let sample_rate = crate::syntax::sample_rate(self.sf_index).unwrap_or(44_100);
        Ok(Frame::Audio(AudioFrame {
            format: SampleFormat::S16,
            channels: channels_out as u16,
            sample_rate,
            samples: FRAME_LEN as u32,
            pts: pkt.pts,
            time_base: self.time_base,
            data: vec![out_bytes],
        }))
    }
}

/// Decode a single-channel ICS into (info, scalefactors, section_data, tns_data, pulse_data).
/// Reads global_gain, ics_info, section_data, scalefactors, pulse / TNS / gain-control
/// fields. Spectrum decoding is left to caller.
fn decode_ics(
    br: &mut BitReader<'_>,
    sf_index: u8,
    is_in_cpe: bool,
) -> Result<(
    IcsInfo,
    Vec<i32>,
    SectionData,
    Option<TnsData>,
    Option<PulseData>,
)> {
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
    let _gain_control = br.read_bit()?;
    if _gain_control {
        return Err(Error::unsupported("AAC: gain_control in LC stream"));
    }
    let _ = is_in_cpe;
    Ok((info, sf, sec, tns, pulse))
}

fn fill_spectrum(
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
/// For each scalefactor band where channel 1's codebook is `INTENSITY_HCB`
/// (15) or `INTENSITY_HCB2` (14), synthesise channel-1 coefficients from
/// channel-0 coefficients:
/// ```text
/// spec[1][k] = sign · 2^(-is_position/4) · spec[0][k]
/// ```
/// where:
/// * `sign = +1` for codebook 15, `-1` for codebook 14.
/// * `is_position` is the accumulated scalefactor value stored in
///   `sfs[1][g·max_sfb + sfb]` (the IS-specific accumulator tracked by
///   [`parse_scalefactors`]).
/// * `ms_used[g·max_sfb + sfb]` on an IS band is repurposed as the sign
///   toggle (per §4.6.8.2.3); when it's set, the sign flips.
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
            // cb 15 → sign +1, cb 14 → sign -1.
            let mut sign: f32 = if cb == INTENSITY_HCB { 1.0 } else { -1.0 };
            // ms_mask_present != 0 makes ms_used[] meaningful. On IS bands
            // it's the sign-toggle bit rather than an MS flag (the MS pass
            // already skips IS bands).
            if ms_mask_present != 0 && ms_used[g * max_sfb + sfb] {
                sign = -sign;
            }
            let is_position = sfs[1][g * max_sfb + sfb];
            let scale = (2.0f32).powf(-(is_position as f32) * 0.25);
            let coef = sign * scale;
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

/// Apply M/S stereo decoding in-place. Spec §4.6.13.
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
            let cb = secs[0].sfb_cb[g * max_sfb + sfb];
            // M/S only applies to non-zero, non-IS, non-noise bands.
            if cb == ZERO_HCB || cb == NOISE_HCB || cb == INTENSITY_HCB || cb == INTENSITY_HCB2 {
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
        let mut info = IcsInfo::default();
        info.sf_index = sf_index;
        info.window_sequence = WindowSequence::OnlyLong;
        info.max_sfb = max_sfb;
        info.num_window_groups = 1;
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
}

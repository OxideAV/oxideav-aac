//! ADTS elementary-stream container (`.aac` / `.adts` files).
//!
//! An ADTS file is a plain concatenation of ISO/IEC 14496-3 §1.A.2
//! `adts_frame()`s (optionally preceded by an ID3v2 tag and followed by
//! an ID3v1 trailer). There is no container header: every frame
//! carries its own §1.A.2.2.1 fixed header (`profile_ObjectType`,
//! `sampling_frequency_index`, `channel_configuration`) and the
//! `aac_frame_length` that delimits it.
//!
//! ## Demuxer
//!
//! [`AdtsDemuxer`] walks the frame headers once at open time (resyncing
//! over garbage on the 12-bit `0xFFF` syncword) to build a frame index
//! for duration and seeking, then emits **one packet per ADTS frame,
//! header included** — the AAC decoder consumes ADTS natively and the
//! MP4 / Matroska muxers strip the header (§1.A.2: the header is pure
//! transport). The stream's `extradata` carries the equivalent
//! out-of-band `AudioSpecificConfig` so a stream-copy into MP4 /
//! Matroska can write its `esds` / `CodecPrivate` before the first
//! packet.
//!
//! ADTS can only signal HE-AAC implicitly (an `EXT_SBR_DATA` FIL in the
//! payload, §4.6.18 / §1.6.5). The demuxer therefore decodes the first
//! frame to learn the *output* geometry: an SBR-active stream is
//! reported at its doubled rate with an explicit backward-compatible
//! HE-AAC ASC, a PS stream as stereo with the HE-AAC v2 ASC.
//!
//! ## Muxer
//!
//! [`AdtsMuxer`] writes one AAC stream as ADTS. Packets that already
//! are complete ADTS frames (the in-crate encoder's output) are written
//! verbatim; bare access units (from MP4 / Matroska) get a 7-byte
//! `protection_absent = 1` header built from the stream's
//! `AudioSpecificConfig` (or, lacking one, AAC-LC at the stream's
//! sample rate / channel count). HE-AAC is written with implicit
//! signalling: the header carries the AAC-LC core configuration.

use std::io::{Read, Seek, SeekFrom, Write};

use oxideav_core::{
    CodecId, CodecParameters, CodecResolver, ContainerRegistry, Demuxer, Error, MediaType, Muxer,
    Packet, ProbeData, ReadSeek, Result, SampleFormat, StreamInfo, TimeBase, WriteSeek,
};

use crate::adts::{AdtsHeader, ADTS_HEADER_BYTES_NO_CRC, ADTS_SAMPLE_RATES_HZ};
use crate::asc::AudioSpecificConfig;
use crate::codec_decoder::CODEC_ID_STR;

/// Container name the ADTS demuxer / muxer register under.
pub const FORMAT_NAME: &str = "adts";

/// Install the ADTS demuxer, muxer, probe and the `.aac` / `.adts`
/// extensions into `reg`.
pub fn register_container(reg: &mut ContainerRegistry) {
    reg.register_demuxer(FORMAT_NAME, open_demuxer);
    reg.register_muxer(FORMAT_NAME, open_muxer);
    reg.register_extension("aac", FORMAT_NAME);
    reg.register_extension("adts", FORMAT_NAME);
    reg.register_probe(FORMAT_NAME, probe);
}

/// Length of a leading ID3v2 tag (header + body + optional footer), or
/// `0` when `buf` does not start with one.
fn id3v2_len(buf: &[u8]) -> usize {
    if buf.len() < 10 || &buf[..3] != b"ID3" {
        return 0;
    }
    let size = buf[6..10]
        .iter()
        .fold(0usize, |acc, &b| (acc << 7) | usize::from(b & 0x7f));
    let footer = if buf[5] & 0x10 != 0 { 10 } else { 0 };
    10 + size + footer
}

/// Parse an ADTS header at `buf[pos..]`, returning it when it is
/// structurally valid.
fn header_at(buf: &[u8], pos: usize) -> Option<AdtsHeader> {
    let b = buf.get(pos..)?;
    if b.len() < ADTS_HEADER_BYTES_NO_CRC || b[0] != 0xFF || b[1] & 0xF6 != 0xF0 {
        return None;
    }
    AdtsHeader::parse(b).ok().map(|(h, _)| h)
}

/// Content probe: an ADTS header (after an optional ID3v2 tag) whose
/// `aac_frame_length` lands on a second valid header is definitive; a
/// lone header is corroborated by the `.aac` / `.adts` extension.
pub fn probe(p: &ProbeData) -> u8 {
    let start = id3v2_len(p.buf);
    let Some(h) = header_at(p.buf, start) else {
        return 0;
    };
    let ext_ok = matches!(p.ext, Some("aac") | Some("adts"));
    let next = start + h.aac_frame_length as usize;
    match header_at(p.buf, next) {
        Some(h2)
            if h2.sampling_frequency_index == h.sampling_frequency_index
                && h2.profile == h.profile =>
        {
            // Two chained frames: still leave headroom for containers
            // with a real magic number.
            if ext_ok {
                100
            } else {
                90
            }
        }
        // The probe window ended inside the first frame.
        None if next >= p.buf.len() => {
            if ext_ok {
                75
            } else {
                40
            }
        }
        _ => {
            if ext_ok {
                50
            } else {
                0
            }
        }
    }
}

fn open_demuxer(input: Box<dyn ReadSeek>, _codecs: &dyn CodecResolver) -> Result<Box<dyn Demuxer>> {
    Ok(Box::new(AdtsDemuxer::open(input)?))
}

fn open_muxer(output: Box<dyn WriteSeek>, streams: &[StreamInfo]) -> Result<Box<dyn Muxer>> {
    Ok(Box::new(AdtsMuxer::new(output, streams)?))
}

/// One indexed ADTS frame.
#[derive(Debug, Clone, Copy)]
struct FrameEntry {
    offset: u64,
    len: u32,
    /// Output PCM samples per channel this frame decodes to.
    samples: u32,
    /// Presentation timestamp (in output samples) of the frame.
    pts: i64,
}

/// ADTS elementary-stream demuxer. See the module docs.
pub struct AdtsDemuxer {
    input: Box<dyn ReadSeek>,
    streams: Vec<StreamInfo>,
    frames: Vec<FrameEntry>,
    next: usize,
}

impl std::fmt::Debug for AdtsDemuxer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AdtsDemuxer")
            .field("streams", &self.streams)
            .field("frames", &self.frames.len())
            .field("next", &self.next)
            .finish()
    }
}

/// Read up to `buf.len()` bytes, stopping early only at end of input.
fn read_full<R: Read + ?Sized>(r: &mut R, buf: &mut [u8]) -> std::io::Result<usize> {
    let mut n = 0;
    while n < buf.len() {
        match r.read(&mut buf[n..]) {
            Ok(0) => break,
            Ok(k) => n += k,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }
    }
    Ok(n)
}

impl AdtsDemuxer {
    /// Open an ADTS stream: index every frame and derive the stream
    /// parameters from the first one.
    pub fn open(mut input: Box<dyn ReadSeek>) -> Result<Self> {
        let total = input.seek(SeekFrom::End(0))?;
        input.seek(SeekFrom::Start(0))?;
        let mut head = [0u8; 10];
        let n = read_full(&mut *input, &mut head)?;
        let mut pos = id3v2_len(&head[..n]) as u64;

        // Index pass: walk the headers, resyncing over garbage.
        let mut raw: Vec<(u64, AdtsHeader)> = Vec::new();
        let mut hdr = [0u8; ADTS_HEADER_BYTES_NO_CRC + 2];
        while pos + ADTS_HEADER_BYTES_NO_CRC as u64 <= total {
            input.seek(SeekFrom::Start(pos))?;
            let got = read_full(&mut *input, &mut hdr)?;
            match header_at(&hdr[..got], 0) {
                Some(h) if pos + u64::from(h.aac_frame_length) <= total => {
                    raw.push((pos, h));
                    pos += u64::from(h.aac_frame_length);
                }
                Some(_) => break, // truncated trailing frame
                None => match Self::resync(&mut *input, pos + 1, total)? {
                    Some(p) => pos = p,
                    None => break,
                },
            }
        }
        let Some(&(first_off, first)) = raw.first() else {
            return Err(Error::invalid("adts: no ADTS frame found"));
        };

        // Decode the first frame to learn the output geometry (implicit
        // SBR doubles the rate; PS turns a mono core into stereo).
        let mut buf = vec![0u8; first.aac_frame_length as usize];
        input.seek(SeekFrom::Start(first_off))?;
        input.read_exact(&mut buf)?;
        let core_rate = first.sample_rate();
        let cfg = first.channel_configuration;
        let cfg_channels: u16 = match cfg {
            7 => 8,
            c => u16::from(c),
        };
        let (out_rate, out_channels) =
            match crate::decode::StreamDecoder::new().decode_adts_frame(&buf) {
                Ok(d) if d.channels > 0 => (d.sample_rate, d.channels as u16),
                _ => (core_rate, cfg_channels),
            };
        let sbr = out_rate == core_rate.saturating_mul(2);
        let ps = sbr && cfg == 1 && out_channels == 2;
        let per_block: u32 = if sbr { 2048 } else { 1024 };

        let extradata = if cfg == 0 {
            // A PCE-defined layout cannot be expressed without the PCE;
            // it rides in-band in every frame instead.
            Vec::new()
        } else if ps {
            crate::asc_writer::he_aac_v2_asc(core_rate, out_rate, false)
        } else if sbr {
            crate::asc_writer::he_aac_v1_asc(core_rate, out_rate, cfg, false)
        } else {
            crate::asc_writer::ga_asc(first.audio_object_type(), core_rate, cfg)
        };

        let mut frames = Vec::with_capacity(raw.len());
        let mut pts = 0i64;
        for (offset, h) in &raw {
            let samples = per_block * u32::from(h.number_of_raw_data_blocks_in_frame);
            frames.push(FrameEntry {
                offset: *offset,
                len: u32::from(h.aac_frame_length),
                samples,
                pts,
            });
            pts += i64::from(samples);
        }

        let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        params.sample_rate = Some(out_rate);
        params.channels = Some(if out_channels > 0 { out_channels } else { 2 });
        params.sample_format = Some(SampleFormat::S16);
        params.extradata = extradata;
        let stream_bytes: u64 = frames.iter().map(|f| u64::from(f.len)).sum();
        if pts > 0 {
            params.bit_rate = Some(stream_bytes * 8 * u64::from(out_rate) / pts as u64);
        }
        let stream = StreamInfo {
            index: 0,
            time_base: TimeBase::new(1, i64::from(out_rate)),
            duration: Some(pts),
            start_time: Some(0),
            params,
        };
        Ok(AdtsDemuxer {
            input,
            streams: vec![stream],
            frames,
            next: 0,
        })
    }

    /// Find the next offset `>= from` holding a valid ADTS header whose
    /// frame is followed by another valid header (or ends exactly at
    /// end of input).
    fn resync(input: &mut dyn ReadSeek, from: u64, total: u64) -> Result<Option<u64>> {
        const WINDOW: usize = 64 * 1024;
        let mut base = from;
        let mut buf = vec![0u8; WINDOW];
        while base < total {
            input.seek(SeekFrom::Start(base))?;
            let got = read_full(input, &mut buf)?;
            if got < ADTS_HEADER_BYTES_NO_CRC {
                return Ok(None);
            }
            for i in 0..got.saturating_sub(ADTS_HEADER_BYTES_NO_CRC - 1) {
                let Some(h) = header_at(&buf[..got], i) else {
                    continue;
                };
                let cand = base + i as u64;
                let end = cand + u64::from(h.aac_frame_length);
                if end == total {
                    return Ok(Some(cand));
                }
                if end > total {
                    continue;
                }
                let mut nxt = [0u8; ADTS_HEADER_BYTES_NO_CRC + 2];
                input.seek(SeekFrom::Start(end))?;
                let n = read_full(input, &mut nxt)?;
                if header_at(&nxt[..n], 0).is_some() {
                    return Ok(Some(cand));
                }
            }
            base += (got - (ADTS_HEADER_BYTES_NO_CRC - 1)) as u64;
        }
        Ok(None)
    }
}

impl Demuxer for AdtsDemuxer {
    fn format_name(&self) -> &str {
        FORMAT_NAME
    }

    fn streams(&self) -> &[StreamInfo] {
        &self.streams
    }

    fn next_packet(&mut self) -> Result<Packet> {
        let Some(f) = self.frames.get(self.next).copied() else {
            return Err(Error::Eof);
        };
        self.next += 1;
        let mut data = vec![0u8; f.len as usize];
        self.input.seek(SeekFrom::Start(f.offset))?;
        self.input.read_exact(&mut data)?;
        Ok(Packet::new(0, self.streams[0].time_base, data)
            .with_pts(f.pts)
            .with_dts(f.pts)
            .with_duration(i64::from(f.samples))
            .with_keyframe(true))
    }

    fn seek_to(&mut self, _stream_index: u32, pts: i64) -> Result<i64> {
        // Last frame starting at or before `pts`.
        let idx = self
            .frames
            .partition_point(|f| f.pts <= pts)
            .saturating_sub(1);
        self.next = idx;
        Ok(self.frames.get(idx).map_or(0, |f| f.pts))
    }

    fn duration_micros(&self) -> Option<i64> {
        let s = &self.streams[0];
        let rate = s.params.sample_rate? as i64;
        Some(s.duration? * 1_000_000 / rate.max(1))
    }
}

/// ADTS muxer. See the module docs.
pub struct AdtsMuxer {
    output: Box<dyn WriteSeek>,
    /// ADTS `profile_ObjectType` (= core `audioObjectType - 1`).
    profile: u8,
    sampling_frequency_index: u8,
    channel_configuration: u8,
    header_written: bool,
}

impl std::fmt::Debug for AdtsMuxer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AdtsMuxer")
            .field("profile", &self.profile)
            .field("sampling_frequency_index", &self.sampling_frequency_index)
            .field("channel_configuration", &self.channel_configuration)
            .finish()
    }
}

impl AdtsMuxer {
    /// Build an ADTS muxer for exactly one AAC audio stream.
    pub fn new(output: Box<dyn WriteSeek>, streams: &[StreamInfo]) -> Result<Self> {
        let [s] = streams else {
            return Err(Error::invalid(
                "adts muxer: exactly one AAC audio stream is required",
            ));
        };
        if s.params.media_type != MediaType::Audio || s.params.codec_id.as_str() != CODEC_ID_STR {
            return Err(Error::invalid(format!(
                "adts muxer: stream codec {} is not aac",
                s.params.codec_id
            )));
        }
        let (aot, rate, cfg) = if s.params.extradata.is_empty() {
            let rate = s
                .params
                .sample_rate
                .ok_or_else(|| Error::invalid("adts muxer: aac stream has no sample rate"))?;
            let cfg = match s.params.channels.unwrap_or(2) {
                c @ 1..=6 => c as u8,
                8 => 7,
                c => {
                    return Err(Error::invalid(format!(
                        "adts muxer: {c} channels have no ADTS channel_configuration"
                    )))
                }
            };
            (2u8, rate, cfg)
        } else {
            let (asc, _) = AudioSpecificConfig::parse(&s.params.extradata)
                .map_err(|e| Error::invalid(format!("adts muxer: extradata ASC: {e}")))?;
            // HE-AAC rides with implicit signalling: the header carries
            // the AAC core configuration (§1.6.5 / §1.A.2.2.1).
            (asc.aot, asc.sample_rate, asc.channel_configuration)
        };
        if !(1..=4).contains(&aot) {
            return Err(Error::unsupported(format!(
                "adts muxer: audioObjectType {aot} cannot be carried in ADTS (Main/LC/SSR/LTP only)"
            )));
        }
        let sampling_frequency_index = ADTS_SAMPLE_RATES_HZ
            .iter()
            .position(|&r| r == rate)
            .ok_or_else(|| {
                Error::unsupported(format!("adts muxer: {rate} Hz has no ADTS sampling index"))
            })? as u8;
        if cfg > 7 {
            return Err(Error::unsupported(
                "adts muxer: channelConfiguration > 7 cannot be carried in ADTS",
            ));
        }
        Ok(AdtsMuxer {
            output,
            profile: aot - 1,
            sampling_frequency_index,
            channel_configuration: cfg,
            header_written: false,
        })
    }

    /// The 7-byte header for a bare access unit of `au_len` bytes.
    fn header_for(&self, au_len: usize) -> Result<[u8; ADTS_HEADER_BYTES_NO_CRC]> {
        let frame_len = au_len + ADTS_HEADER_BYTES_NO_CRC;
        if frame_len >= 1 << 13 {
            return Err(Error::invalid(format!(
                "adts muxer: {au_len}-byte access unit exceeds the 13-bit aac_frame_length"
            )));
        }
        AdtsHeader {
            mpeg_version_mpeg2: false,
            protection_absent: true,
            profile: self.profile,
            sampling_frequency_index: self.sampling_frequency_index,
            channel_configuration: self.channel_configuration,
            aac_frame_length: frame_len as u16,
            adts_buffer_fullness: 0x7FF,
            number_of_raw_data_blocks_in_frame: 1,
        }
        .write()
        .map_err(|e| Error::invalid(format!("adts muxer: header: {e}")))
    }
}

impl Muxer for AdtsMuxer {
    fn format_name(&self) -> &str {
        FORMAT_NAME
    }

    fn write_header(&mut self) -> Result<()> {
        self.header_written = true;
        Ok(())
    }

    fn write_packet(&mut self, packet: &Packet) -> Result<()> {
        if !self.header_written {
            return Err(Error::other("adts muxer: write_header not called"));
        }
        if packet.data.is_empty() {
            return Ok(());
        }
        if crate::codec_decoder::is_complete_adts_frame(&packet.data) {
            self.output.write_all(&packet.data)?;
        } else {
            let hdr = self.header_for(packet.data.len())?;
            self.output.write_all(&hdr)?;
            self.output.write_all(&packet.data)?;
        }
        Ok(())
    }

    fn write_trailer(&mut self) -> Result<()> {
        self.output.flush()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    /// A tiny two-frame ADTS stream from the in-crate encoder.
    fn encoded_adts(rate: u32, channels: u16, frames: usize) -> Vec<u8> {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        p.sample_rate = Some(rate);
        p.channels = Some(channels);
        let mut enc = crate::codec_encoder::make_encoder(&p).unwrap();
        let n = 1024 * frames;
        let mut pcm = Vec::with_capacity(n * channels as usize * 2);
        for i in 0..n {
            let v = ((i as f64 * 440.0 * std::f64::consts::TAU / f64::from(rate)).sin() * 8000.0)
                as i16;
            for _ in 0..channels {
                pcm.extend_from_slice(&v.to_le_bytes());
            }
        }
        enc.send_frame(&oxideav_core::Frame::Audio(oxideav_core::AudioFrame {
            samples: n as u32,
            pts: Some(0),
            data: vec![pcm],
        }))
        .unwrap();
        enc.flush().unwrap();
        let mut out = Vec::new();
        while let Ok(p) = enc.receive_packet() {
            out.extend_from_slice(&p.data);
        }
        out
    }

    #[test]
    fn probe_scores_chained_frames() {
        let bytes = encoded_adts(44_100, 2, 3);
        assert_eq!(
            probe(&ProbeData {
                buf: &bytes,
                ext: None
            }),
            90
        );
        assert_eq!(
            probe(&ProbeData {
                buf: &bytes,
                ext: Some("aac")
            }),
            100
        );
        assert_eq!(
            probe(&ProbeData {
                buf: b"not an adts stream at all",
                ext: None
            }),
            0
        );
    }

    #[test]
    fn demux_indexes_frames_and_emits_asc() {
        let mut bytes = b"garbage".to_vec();
        bytes.extend(encoded_adts(48_000, 1, 4));
        let mut d = AdtsDemuxer::open(Box::new(Cursor::new(bytes))).unwrap();
        let s = d.streams()[0].clone();
        assert_eq!(s.params.sample_rate, Some(48_000));
        assert_eq!(s.params.channels, Some(1));
        // AAC-LC, 48 kHz (index 3), mono.
        assert_eq!(s.params.extradata, crate::asc_writer::aac_lc_asc(48_000, 1));
        let mut n = 0;
        let mut last_pts = -1;
        while let Ok(p) = d.next_packet() {
            assert!(crate::codec_decoder::is_complete_adts_frame(&p.data));
            assert!(p.pts.unwrap() > last_pts);
            last_pts = p.pts.unwrap();
            n += 1;
        }
        assert!(n >= 5, "expected every encoded frame, got {n}");
        assert_eq!(d.seek_to(0, 2048).unwrap(), 2048);
        assert_eq!(d.next_packet().unwrap().pts, Some(2048));
    }

    #[test]
    fn mux_wraps_bare_access_units_and_passes_adts_through() {
        let adts = encoded_adts(44_100, 2, 2);
        let (h, off) = AdtsHeader::parse(&adts).unwrap();
        let first = &adts[..h.aac_frame_length as usize];
        let au = &first[off..];

        let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        params.sample_rate = Some(44_100);
        params.channels = Some(2);
        params.extradata = crate::asc_writer::aac_lc_asc(44_100, 2);
        let stream = StreamInfo {
            index: 0,
            time_base: TimeBase::new(1, 44_100),
            duration: None,
            start_time: Some(0),
            params,
        };
        let shared = std::sync::Arc::new(std::sync::Mutex::new(Cursor::new(Vec::new())));
        struct Sink(std::sync::Arc<std::sync::Mutex<Cursor<Vec<u8>>>>);
        impl Write for Sink {
            fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
                self.0.lock().unwrap().write(b)
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        impl Seek for Sink {
            fn seek(&mut self, p: SeekFrom) -> std::io::Result<u64> {
                self.0.lock().unwrap().seek(p)
            }
        }
        let mut m = AdtsMuxer::new(Box::new(Sink(shared.clone())), &[stream]).unwrap();
        m.write_header().unwrap();
        let tb = TimeBase::new(1, 44_100);
        m.write_packet(&Packet::new(0, tb, au.to_vec())).unwrap();
        m.write_packet(&Packet::new(0, tb, first.to_vec())).unwrap();
        m.write_trailer().unwrap();
        let out = shared.lock().unwrap().get_ref().clone();
        // The wrapped AU reproduces the encoder's own frame (the
        // encoder writes the same 7-byte protection-absent header).
        assert_eq!(&out[..first.len()], first);
        assert_eq!(&out[first.len()..], first);
    }
}

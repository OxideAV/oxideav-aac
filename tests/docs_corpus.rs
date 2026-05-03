//! Integration tests against the `docs/audio/aac/fixtures/` corpus.
//!
//! Each fixture under `../../docs/audio/aac/fixtures/<name>/` ships an
//! input AAC bitstream in one of three carriers — raw `.aac` (ADTS
//! frames, with or without an ID3v2 prefix), `.m4a` (ISOBMFF / MP4 with
//! the AudioSpecificConfig in `esds`), or `.latm` (LOAS/LATM transport,
//! parser TBD) — plus an `expected.wav` ground-truth produced by
//! FFmpeg's reference AAC decoder. This driver:
//!
//! 1. Demuxes the input through the appropriate path: in-tree
//!    [`oxideav_aac::adts::parse_adts_header`] iteration for the ADTS
//!    family (skipping any leading ID3v2 tag), or
//!    [`oxideav_mp4::demux::open`] (registered via the
//!    [`oxideav_core::ContainerRegistry`]) for the MP4 family.
//! 2. Feeds each access unit through [`oxideav_aac::decoder`].
//! 3. Collects the emitted interleaved S16 PCM and reports per-channel
//!    stats against the WAV reference: bit-exact match percentage, RMS
//!    error, and PSNR over the int16 full scale.
//!
//! AAC is a lossy codec — independent decoders almost never produce
//! sample-identical PCM (IMDCT carry, TNS coefficient quantisation,
//! and PNS PRNG seeding all differ across implementations). Every
//! fixture starts in [`Tier::ReportOnly`]: the test never fails on a
//! numeric divergence, it just records the delta so a human reviewer
//! can see at a glance how close we are. A BitExact tier is reserved
//! for future use.
//!
//! Carriers we don't yet support:
//! - `.latm` — the LOAS/LATM transport parser (ISO 14496-3 §1.7) is
//!   not implemented anywhere in the workspace today, so the
//!   `aac-latm-stream` fixture is skipped with `Tier::Ignored`.
//!   Adding a LATM parser would unblock it without changing this test.
//!
//! The trace.txt + probe.json files under each fixture dir are not
//! consumed here; they are an aid for the human implementer when
//! localising divergences.

use std::fs;
use std::path::PathBuf;

use oxideav_core::{
    CodecId, CodecParameters, ContainerRegistry, Error, Frame, NullCodecResolver, Packet, ReadSeek,
    SampleFormat, TimeBase,
};
// `Box<dyn Decoder>` / `Box<dyn Demuxer>` resolve trait methods through
// the dyn-vtable, so the traits don't need to be in scope at the call
// site here.

use oxideav_aac::adts::parse_adts_header;

/// Locate `docs/audio/aac/fixtures/<name>/`. When the test runs as
/// part of the umbrella workspace, CWD is the crate root and the docs
/// live two levels up at `../../docs/`. When the standalone
/// oxideav-aac repo is checked out alone (CI), `../../docs/` is
/// absent and every fixture access skips gracefully.
fn fixture_dir(name: &str) -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures").join(name)
}

#[derive(Clone, Copy, Debug)]
enum Tier {
    /// Decode is permitted to diverge from the FFmpeg reference; we
    /// log the deltas but do not gate CI on them. AAC is a lossy codec
    /// and two independent decoders typically differ at the sub-LSB
    /// level. See the module-level comment.
    ReportOnly,
    /// The fixture's carrier or codec variant is not yet supported.
    /// The driver records the skip and moves on; the test passes.
    Ignored,
}

#[derive(Clone, Copy, Debug)]
enum Carrier {
    /// Raw concatenated ADTS frames. Optional 10-byte ID3v2 prefix is
    /// stripped via the synchsafe-int header length.
    Adts,
    /// ISOBMFF / MP4 (`.m4a`). Demuxed through `oxideav-mp4`. The
    /// stream's `extradata` carries the AudioSpecificConfig.
    Mp4,
    /// LOAS/LATM transport. Not yet implemented; the driver returns
    /// `None` and the test passes.
    Latm,
}

struct CorpusCase {
    name: &'static str,
    /// Input file inside `fixture_dir(name)/`.
    input_file: &'static str,
    carrier: Carrier,
    /// Expected channels (used to sanity-check the carrier's parse).
    /// `None` skips the check.
    channels: Option<u16>,
    /// Expected sample rate. `None` skips the check.
    sample_rate: Option<u32>,
    tier: Tier,
}

/// Decoded output from one fixture: interleaved s16le samples plus the
/// channel count + sample rate the decoder advertised.
struct DecodedPcm {
    samples: Vec<i16>,
    channels: u16,
    sample_rate: u32,
}

/// Reference PCM extracted from the fixture's expected.wav.
struct RefPcm {
    samples: Vec<i16>,
    channels: u16,
    sample_rate: u32,
}

/// Per-channel diff numbers + aggregate match percentage and PSNR.
struct ChannelStat {
    rms_ref: f64,
    rms_ours: f64,
    sse: f64,
    exact: usize,
    near: usize,
    total: usize,
    max_abs_err: i32,
}

impl ChannelStat {
    fn new() -> Self {
        Self {
            rms_ref: 0.0,
            rms_ours: 0.0,
            sse: 0.0,
            exact: 0,
            near: 0,
            total: 0,
            max_abs_err: 0,
        }
    }

    fn match_pct(&self) -> f64 {
        if self.total == 0 {
            0.0
        } else {
            self.exact as f64 / self.total as f64 * 100.0
        }
    }

    fn near_pct(&self) -> f64 {
        if self.total == 0 {
            0.0
        } else {
            self.near as f64 / self.total as f64 * 100.0
        }
    }

    /// PSNR over a 16-bit signed full scale (peak = 32767). Returns
    /// `f64::INFINITY` on perfect match.
    fn psnr_db(&self) -> f64 {
        if self.total == 0 || self.sse == 0.0 {
            return f64::INFINITY;
        }
        let mse = self.sse / self.total as f64;
        let peak = 32767.0_f64;
        10.0 * (peak * peak / mse).log10()
    }
}

/// Skip an ID3v2.x tag (synchsafe-int 28-bit length at offset 6..10).
/// Returns the offset of the first byte after the tag, or 0 if absent.
fn skip_id3v2(data: &[u8]) -> usize {
    if data.len() < 10 || &data[0..3] != b"ID3" {
        return 0;
    }
    let size = ((data[6] as u32 & 0x7F) << 21)
        | ((data[7] as u32 & 0x7F) << 14)
        | ((data[8] as u32 & 0x7F) << 7)
        | (data[9] as u32 & 0x7F);
    let footer_extra = if (data[5] & 0x10) != 0 { 10 } else { 0 };
    10 + size as usize + footer_extra
}

/// Decode an ADTS-carriered fixture by walking frame-by-frame from the
/// first byte after any ID3v2 prefix.
fn decode_adts(case: &CorpusCase, bytes: &[u8]) -> Option<DecodedPcm> {
    let start = skip_id3v2(bytes);
    if start >= bytes.len() {
        eprintln!("{}: ID3v2 tag covers entire file", case.name);
        return None;
    }
    let body = &bytes[start..];

    // Probe the first ADTS header to seed CodecParameters.
    let mut i = 0usize;
    while i + 7 <= body.len() && (body[i] != 0xFF || (body[i + 1] & 0xF0) != 0xF0) {
        i += 1;
    }
    if i + 7 > body.len() {
        eprintln!("{}: no ADTS sync found", case.name);
        return None;
    }
    let first = match parse_adts_header(&body[i..]) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("{}: first ADTS header parse failed: {e}", case.name);
            return None;
        }
    };
    let sr = match first.sample_rate() {
        Some(s) => s,
        None => {
            eprintln!(
                "{}: first ADTS header has invalid sampling-frequency-index",
                case.name
            );
            return None;
        }
    };
    let ch = first.channel_configuration as u16;
    if ch == 0 {
        eprintln!(
            "{}: ADTS channel_configuration=0 (PCE-defined) not handled here",
            case.name
        );
        return None;
    }
    if let Some(want_ch) = case.channels {
        assert_eq!(
            ch, want_ch,
            "{}: ADTS says {ch} channels, expected {want_ch}",
            case.name
        );
    }
    if let Some(want_sr) = case.sample_rate {
        assert_eq!(
            sr, want_sr,
            "{}: ADTS says {sr} Hz, expected {want_sr}",
            case.name
        );
    }

    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(ch);
    params.sample_format = Some(SampleFormat::S16);
    let mut decoder = match oxideav_aac::decoder::make_decoder(&params) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("{}: decoder ctor failed: {e}", case.name);
            return None;
        }
    };

    let tb = TimeBase::new(1, sr as i64);
    let mut samples: Vec<i16> = Vec::new();
    let mut decoder_errors = 0usize;
    let mut frame_idx = 0i64;
    let mut pos = i;
    while pos + 7 <= body.len() {
        if body[pos] != 0xFF || (body[pos + 1] & 0xF0) != 0xF0 {
            pos += 1;
            continue;
        }
        let h = match parse_adts_header(&body[pos..]) {
            Ok(h) => h,
            Err(_) => {
                pos += 1;
                continue;
            }
        };
        if h.frame_length == 0 || pos + h.frame_length > body.len() {
            break;
        }
        let pkt =
            Packet::new(0, tb, body[pos..pos + h.frame_length].to_vec()).with_pts(frame_idx * 1024);
        if let Err(e) = decoder.send_packet(&pkt) {
            decoder_errors += 1;
            if decoder_errors <= 3 {
                eprintln!("{}: send_packet error at frame {frame_idx}: {e}", case.name);
            }
            pos += h.frame_length;
            frame_idx += 1;
            continue;
        }
        match decoder.receive_frame() {
            Ok(Frame::Audio(af)) => {
                let plane = &af.data[0];
                for chunk in plane.chunks_exact(2) {
                    samples.push(i16::from_le_bytes([chunk[0], chunk[1]]));
                }
            }
            Ok(other) => {
                eprintln!("{}: unexpected non-audio frame: {other:?}", case.name);
            }
            Err(Error::NeedMore) => {}
            Err(Error::Eof) => break,
            Err(e) => {
                decoder_errors += 1;
                if decoder_errors <= 3 {
                    eprintln!(
                        "{}: receive_frame error at frame {frame_idx}: {e}",
                        case.name
                    );
                }
            }
        }
        pos += h.frame_length;
        frame_idx += 1;
    }
    if decoder_errors > 0 {
        eprintln!(
            "{}: total decoder errors: {decoder_errors} (decoded {} samples)",
            case.name,
            samples.len(),
        );
    }
    // AudioFrame does NOT carry channel/rate; they're stream-level. We
    // report the demuxer/header values, which is what the caller expects
    // anyway. SBR may double the rate at decode time — that mismatch
    // (vs the WAV reference) shows up as the sample-rate WARN line.
    Some(DecodedPcm {
        samples,
        channels: ch,
        sample_rate: sr,
    })
}

/// Decode an MP4-carriered fixture: register `oxideav-mp4` in a one-shot
/// ContainerRegistry, find the AAC track, hand its extradata-bearing
/// CodecParameters to the AAC decoder, and pump every sample.
fn decode_mp4(case: &CorpusCase, file: fs::File) -> Option<DecodedPcm> {
    let mut creg = ContainerRegistry::new();
    creg.register_demuxer("mp4", oxideav_mp4::demux::open);

    let rs: Box<dyn ReadSeek> = Box::new(file);
    let mut demux = match creg.open_demuxer("mp4", rs, &NullCodecResolver) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("{}: mp4 demuxer open failed: {e}", case.name);
            return None;
        }
    };

    // Find the first AAC audio track.
    let aac_stream = demux.streams().iter().find(|s| {
        let cid = s.params.codec_id.as_str();
        cid == "aac"
    });
    let stream = match aac_stream {
        Some(s) => s.clone(),
        None => {
            let codecs: Vec<String> = demux
                .streams()
                .iter()
                .map(|s| s.params.codec_id.as_str().to_owned())
                .collect();
            eprintln!("{}: mp4 has no AAC track (got: {:?})", case.name, codecs);
            return None;
        }
    };
    let params = stream.params.clone();
    let channels = params.channels.unwrap_or(0);
    let sample_rate = params.sample_rate.unwrap_or(0);
    if let Some(want_ch) = case.channels {
        if channels != 0 {
            assert_eq!(
                channels, want_ch,
                "{}: mp4 says {channels} channels, expected {want_ch}",
                case.name
            );
        }
    }
    if let Some(want_sr) = case.sample_rate {
        if sample_rate != 0 {
            assert_eq!(
                sample_rate, want_sr,
                "{}: mp4 says {sample_rate} Hz, expected {want_sr}",
                case.name
            );
        }
    }

    let mut decoder = match oxideav_aac::decoder::make_decoder(&params) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("{}: decoder ctor failed: {e}", case.name);
            return None;
        }
    };

    let stream_index = stream.index;
    let mut samples: Vec<i16> = Vec::new();
    let mut decoder_errors = 0usize;
    loop {
        let pkt = match demux.next_packet() {
            Ok(p) => p,
            Err(Error::Eof) => break,
            Err(e) => {
                eprintln!(
                    "{}: demux error after {} samples: {e}",
                    case.name,
                    samples.len()
                );
                break;
            }
        };
        if pkt.stream_index != stream_index {
            continue;
        }
        if let Err(e) = decoder.send_packet(&pkt) {
            decoder_errors += 1;
            if decoder_errors <= 3 {
                eprintln!("{}: send_packet error: {e}", case.name);
            }
            continue;
        }
        loop {
            match decoder.receive_frame() {
                Ok(Frame::Audio(af)) => {
                    let plane = &af.data[0];
                    for chunk in plane.chunks_exact(2) {
                        samples.push(i16::from_le_bytes([chunk[0], chunk[1]]));
                    }
                }
                Ok(other) => {
                    eprintln!("{}: unexpected non-audio frame: {other:?}", case.name);
                    break;
                }
                Err(Error::NeedMore) => break,
                Err(Error::Eof) => break,
                Err(e) => {
                    decoder_errors += 1;
                    if decoder_errors <= 3 {
                        eprintln!("{}: receive_frame error: {e}", case.name);
                    }
                    break;
                }
            }
        }
    }
    if decoder_errors > 0 {
        eprintln!(
            "{}: total decoder errors: {decoder_errors} (decoded {} samples)",
            case.name,
            samples.len(),
        );
    }
    Some(DecodedPcm {
        samples,
        channels,
        sample_rate,
    })
}

fn decode_fixture_pcm(case: &CorpusCase) -> Option<DecodedPcm> {
    let dir = fixture_dir(case.name);
    let in_path = dir.join(case.input_file);
    match case.carrier {
        Carrier::Adts => {
            let bytes = match fs::read(&in_path) {
                Ok(b) => b,
                Err(e) => {
                    eprintln!("skip {}: missing {} ({e})", case.name, in_path.display());
                    return None;
                }
            };
            decode_adts(case, &bytes)
        }
        Carrier::Mp4 => {
            let file = match fs::File::open(&in_path) {
                Ok(f) => f,
                Err(e) => {
                    eprintln!("skip {}: missing {} ({e})", case.name, in_path.display());
                    return None;
                }
            };
            decode_mp4(case, file)
        }
        Carrier::Latm => {
            eprintln!(
                "skip {}: LATM/LOAS transport parser not implemented yet",
                case.name
            );
            None
        }
    }
}

/// Parse a minimal RIFF/WAVE file: locate the `fmt ` chunk to read
/// channels + sample-rate + bits-per-sample, then return the `data`
/// chunk as interleaved s16le samples. Skips any LIST/INFO,
/// JUNK, or other non-essential chunks between `fmt ` and `data`.
fn parse_wav(bytes: &[u8]) -> Option<RefPcm> {
    if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    let mut i = 12usize;
    let mut channels: u16 = 0;
    let mut sample_rate: u32 = 0;
    let mut bits_per_sample: u16 = 0;
    let mut data: Option<&[u8]> = None;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let sz =
            u32::from_le_bytes([bytes[i + 4], bytes[i + 5], bytes[i + 6], bytes[i + 7]]) as usize;
        let body_start = i + 8;
        let body_end = body_start + sz;
        if body_end > bytes.len() {
            break;
        }
        match id {
            b"fmt " => {
                if sz < 16 {
                    return None;
                }
                let format_tag = u16::from_le_bytes([bytes[body_start], bytes[body_start + 1]]);
                channels = u16::from_le_bytes([bytes[body_start + 2], bytes[body_start + 3]]);
                sample_rate = u32::from_le_bytes([
                    bytes[body_start + 4],
                    bytes[body_start + 5],
                    bytes[body_start + 6],
                    bytes[body_start + 7],
                ]);
                bits_per_sample =
                    u16::from_le_bytes([bytes[body_start + 14], bytes[body_start + 15]]);
                if format_tag != 1 && format_tag != 0xFFFE {
                    return None;
                }
            }
            b"data" => {
                data = Some(&bytes[body_start..body_end]);
                break;
            }
            _ => {}
        }
        i = body_end + (sz & 1);
    }
    let data = data?;
    if channels == 0 || sample_rate == 0 || bits_per_sample != 16 {
        return None;
    }
    let mut samples = Vec::with_capacity(data.len() / 2);
    for chunk in data.chunks_exact(2) {
        samples.push(i16::from_le_bytes([chunk[0], chunk[1]]));
    }
    Some(RefPcm {
        samples,
        channels,
        sample_rate,
    })
}

fn read_reference(case: &CorpusCase) -> Option<RefPcm> {
    let dir = fixture_dir(case.name);
    let wav_path = dir.join("expected.wav");
    let bytes = match fs::read(&wav_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("skip {}: missing {} ({e})", case.name, wav_path.display());
            return None;
        }
    };
    parse_wav(&bytes)
}

/// Compute per-channel match/PSNR statistics over the overlapping
/// prefix of decoded vs reference (lossy decoders produce slightly
/// fewer or more samples than the reference because of MDCT overlap-
/// add and encoder delay).
fn compare(ours: &DecodedPcm, refp: &RefPcm) -> Vec<ChannelStat> {
    let chs = ours.channels.min(refp.channels) as usize;
    if chs == 0 {
        return Vec::new();
    }
    let frames_ours = ours.samples.len() / ours.channels.max(1) as usize;
    let frames_ref = refp.samples.len() / refp.channels.max(1) as usize;
    let n = frames_ours.min(frames_ref);

    let mut stats: Vec<ChannelStat> = (0..chs).map(|_| ChannelStat::new()).collect();
    for f in 0..n {
        for (ch, s) in stats.iter_mut().enumerate() {
            let our = ours.samples[f * ours.channels as usize + ch] as i64;
            let r = refp.samples[f * refp.channels as usize + ch] as i64;
            let err = (our - r).abs();
            s.total += 1;
            if err == 0 {
                s.exact += 1;
            }
            if err <= 1 {
                s.near += 1;
            }
            if err as i32 > s.max_abs_err {
                s.max_abs_err = err as i32;
            }
            s.rms_ref += (r * r) as f64;
            s.rms_ours += (our * our) as f64;
            s.sse += (err * err) as f64;
        }
    }
    for s in &mut stats {
        if s.total > 0 {
            s.rms_ref = (s.rms_ref / s.total as f64).sqrt();
            s.rms_ours = (s.rms_ours / s.total as f64).sqrt();
        }
    }
    stats
}

/// Decode -> compare -> log -> tier-aware assert (no AAC fixture
/// currently bit-exact, so all tiers are ReportOnly today).
fn evaluate(case: &CorpusCase) {
    eprintln!(
        "--- {} (carrier={:?} tier={:?}) ---",
        case.name, case.carrier, case.tier
    );
    if matches!(case.tier, Tier::Ignored) {
        eprintln!("{}: Tier::Ignored — skipping", case.name);
        return;
    }
    let Some(ours) = decode_fixture_pcm(case) else {
        return;
    };
    let Some(refp) = read_reference(case) else {
        eprintln!("{}: could not parse expected.wav", case.name);
        return;
    };

    eprintln!(
        "{}: decoded ch={} sr={} samples={} ({} frames); reference ch={} sr={} samples={} ({} frames)",
        case.name,
        ours.channels,
        ours.sample_rate,
        ours.samples.len(),
        ours.samples.len() / ours.channels.max(1) as usize,
        refp.channels,
        refp.sample_rate,
        refp.samples.len(),
        refp.samples.len() / refp.channels.max(1) as usize,
    );

    if ours.channels != refp.channels {
        eprintln!(
            "{}: WARN channel count mismatch (decoded {} vs reference {})",
            case.name, ours.channels, refp.channels
        );
    }
    if ours.sample_rate != refp.sample_rate {
        eprintln!(
            "{}: WARN sample-rate mismatch (decoded {} vs reference {})",
            case.name, ours.sample_rate, refp.sample_rate
        );
    }

    let stats = compare(&ours, &refp);
    if stats.is_empty() {
        eprintln!("{}: no overlapping channels to compare", case.name);
        return;
    }

    let mut total_exact = 0usize;
    let mut total_near = 0usize;
    let mut total_samples = 0usize;
    let mut max_err_overall = 0i32;
    let mut psnr_min: f64 = f64::INFINITY;
    for (i, s) in stats.iter().enumerate() {
        let psnr = s.psnr_db();
        if psnr < psnr_min {
            psnr_min = psnr;
        }
        let rms_err_disp = if s.total > 0 {
            (s.sse / s.total as f64).sqrt()
        } else {
            0.0
        };
        eprintln!(
            "  ch{i}: rms_ref={:.1} rms_ours={:.1} rms_err={:.2} match={:.4}% near<=1LSB={:.4}% max_abs_err={} psnr={:.2} dB",
            s.rms_ref,
            s.rms_ours,
            rms_err_disp,
            s.match_pct(),
            s.near_pct(),
            s.max_abs_err,
            psnr,
        );
        total_exact += s.exact;
        total_near += s.near;
        total_samples += s.total;
        if s.max_abs_err > max_err_overall {
            max_err_overall = s.max_abs_err;
        }
    }
    let agg_pct = if total_samples > 0 {
        total_exact as f64 / total_samples as f64 * 100.0
    } else {
        0.0
    };
    let near_pct = if total_samples > 0 {
        total_near as f64 / total_samples as f64 * 100.0
    } else {
        0.0
    };
    eprintln!(
        "{}: aggregate match={:.4}% near<=1LSB={:.4}% max_abs_err={} min_psnr={:.2} dB",
        case.name, agg_pct, near_pct, max_err_overall, psnr_min,
    );

    match case.tier {
        Tier::ReportOnly => {
            // Logged; never gates CI.
        }
        Tier::Ignored => unreachable!(),
    }
}

// ---------------------------------------------------------------------------
// Per-fixture tests. Every fixture is Tier::ReportOnly today.
// ---------------------------------------------------------------------------

#[test]
fn corpus_aac_latm_stream() {
    // No LATM/LOAS parser in the workspace yet.
    evaluate(&CorpusCase {
        name: "aac-latm-stream",
        input_file: "input.latm",
        carrier: Carrier::Latm,
        channels: Some(2),
        sample_rate: Some(44_100),
        tier: Tier::Ignored,
    });
}

#[test]
fn corpus_aac_lc_5_1_48000_256kbps_mp4() {
    evaluate(&CorpusCase {
        name: "aac-lc-5.1-48000-256kbps-mp4",
        input_file: "input.m4a",
        carrier: Carrier::Mp4,
        channels: Some(6),
        sample_rate: Some(48_000),
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_7_1_48000_320kbps_mp4() {
    evaluate(&CorpusCase {
        name: "aac-lc-7.1-48000-320kbps-mp4",
        input_file: "input.m4a",
        carrier: Carrier::Mp4,
        channels: Some(8),
        sample_rate: Some(48_000),
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_chirp_windows() {
    evaluate(&CorpusCase {
        name: "aac-lc-chirp-windows",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: None,
        sample_rate: None,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_hexagonal_with_pce() {
    // PCE-defined topology — channel_configuration=0; the ADTS check
    // would fire, but the carrier is MP4 here, so we route through the
    // mp4 demuxer instead.
    evaluate(&CorpusCase {
        name: "aac-lc-hexagonal-with-pce",
        input_file: "input.m4a",
        carrier: Carrier::Mp4,
        channels: None,
        sample_rate: None,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_intensity_stereo() {
    evaluate(&CorpusCase {
        name: "aac-lc-intensity-stereo",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: Some(2),
        sample_rate: None,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_mono_11025_32kbps_adts() {
    evaluate(&CorpusCase {
        name: "aac-lc-mono-11025-32kbps-adts",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: Some(1),
        sample_rate: Some(11_025),
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_mono_44100_64kbps_adts() {
    evaluate(&CorpusCase {
        name: "aac-lc-mono-44100-64kbps-adts",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: Some(1),
        sample_rate: Some(44_100),
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_mono_8000_16kbps_adts() {
    evaluate(&CorpusCase {
        name: "aac-lc-mono-8000-16kbps-adts",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: Some(1),
        sample_rate: Some(8_000),
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_ms_stereo() {
    evaluate(&CorpusCase {
        name: "aac-lc-ms-stereo",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: Some(2),
        sample_rate: None,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_pns_noise() {
    evaluate(&CorpusCase {
        name: "aac-lc-pns-noise",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: None,
        sample_rate: None,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_stereo_22050_64kbps_adts() {
    evaluate(&CorpusCase {
        name: "aac-lc-stereo-22050-64kbps-adts",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: Some(2),
        sample_rate: Some(22_050),
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_stereo_44100_128kbps_adts() {
    evaluate(&CorpusCase {
        name: "aac-lc-stereo-44100-128kbps-adts",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: Some(2),
        sample_rate: Some(44_100),
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_stereo_48000_128kbps_mp4() {
    evaluate(&CorpusCase {
        name: "aac-lc-stereo-48000-128kbps-mp4",
        input_file: "input.m4a",
        carrier: Carrier::Mp4,
        channels: Some(2),
        sample_rate: Some(48_000),
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_lc_tns_active() {
    evaluate(&CorpusCase {
        name: "aac-lc-tns-active",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: None,
        sample_rate: None,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_aac_with_id3v2() {
    evaluate(&CorpusCase {
        name: "aac-with-id3v2",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: None,
        sample_rate: None,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_he_aac_v1_stereo_44100_32kbps_adts() {
    // HE-AAC v1 — implicit SBR, ADTS carries an LC core whose rate is
    // half the advertised reference WAV rate. Decoder may or may not
    // interpolate to the upper rate; either way, ReportOnly.
    evaluate(&CorpusCase {
        name: "he-aac-v1-stereo-44100-32kbps-adts",
        input_file: "input.aac",
        carrier: Carrier::Adts,
        channels: Some(2),
        sample_rate: None,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_he_aac_v2_stereo_32000_24kbps_m4a() {
    // HE-AAC v2 (PS) — explicit SBR + PS via ASC in the m4a esds.
    evaluate(&CorpusCase {
        name: "he-aac-v2-stereo-32000-24kbps-m4a",
        input_file: "input.m4a",
        carrier: Carrier::Mp4,
        channels: None,
        sample_rate: None,
        tier: Tier::ReportOnly,
    });
}

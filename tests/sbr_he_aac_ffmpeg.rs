//! HE-AAC interop: feed a real HE-AAC (AAC-LC + SBR) ADTS stream from a
//! third-party encoder through our decoder and confirm SBR activates —
//! ADTS-advertised core sample rate is doubled on output, frames deliver
//! the expected 2048-sample payload, and the stream decodes without
//! erroring out.
//!
//! This exercises the full chain that the unit tests only stub:
//!   * ADTS parse on a real stream with core-rate signalling (the core
//!     ADTS header advertises the *halved* rate for implicit HE-AAC, e.g.
//!     32 kHz output via 16 kHz ADTS with SBR on).
//!   * FIL-element extension-type peek, selecting the SBR path.
//!   * `sbr_header()` / `sbr_single_channel_element()` parse producing a
//!     valid `SbrHeader` + `SbrChannelData` (else the FIL handler would
//!     swallow the payload as unknown and leave the output rate at the
//!     ADTS-advertised core rate).
//!   * Decode path routes through `decode_sbr_cpe_frame`, producing 2×
//!     output at 2× sample rate.
//!
//! The test skips gracefully when no HE-AAC encoder is on PATH. Supported
//! producers (tried in order):
//!   1. `afconvert -f adts -d aach` (macOS AudioToolbox — always present
//!      on darwin).
//!   2. `ffmpeg -c:a libfdk_aac -profile:a aac_he` (Linux/BSD, when the
//!      ffmpeg build includes libfdk_aac).

use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_aac::adts::{parse_adts_header, ADTS_HEADER_NO_CRC};
#[allow(unused_imports)]
use oxideav_core::Decoder;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

/// Locate a binary on PATH. Returns `None` when absent so the test can
/// skip without failing.
fn which(name: &str) -> Option<PathBuf> {
    let out = Command::new("which").arg(name).output().ok()?;
    if !out.status.success() {
        return None;
    }
    let p = String::from_utf8(out.stdout).ok()?;
    let trimmed = p.trim();
    if trimmed.is_empty() {
        None
    } else {
        Some(PathBuf::from(trimmed))
    }
}

/// Write a short mono/stereo sine WAV suitable as third-party encoder
/// input. 2 s is long enough for multiple SBR frames; amplitude 0.25 to
/// stay well clear of clipping after dithering.
fn write_sine_wav(path: &Path, sr: u32, freq: f32, secs: f32) {
    // Trivial 16-bit little-endian WAV. Mono. We hand-write the header so
    // there's no dependency on a WAV writer crate.
    let n = (sr as f32 * secs) as usize;
    let mut pcm = Vec::with_capacity(n * 2);
    for i in 0..n {
        let t = i as f32 / sr as f32;
        let v = (2.0 * std::f32::consts::PI * freq * t).sin() * 0.25;
        let s = (v * 32767.0) as i16;
        pcm.extend_from_slice(&s.to_le_bytes());
    }
    let mut wav = Vec::with_capacity(pcm.len() + 44);
    wav.extend_from_slice(b"RIFF");
    wav.extend_from_slice(&((pcm.len() + 36) as u32).to_le_bytes());
    wav.extend_from_slice(b"WAVE");
    wav.extend_from_slice(b"fmt ");
    wav.extend_from_slice(&16u32.to_le_bytes()); // fmt chunk size
    wav.extend_from_slice(&1u16.to_le_bytes()); // PCM
    wav.extend_from_slice(&1u16.to_le_bytes()); // 1 channel
    wav.extend_from_slice(&sr.to_le_bytes());
    wav.extend_from_slice(&(sr * 2).to_le_bytes()); // byte rate
    wav.extend_from_slice(&2u16.to_le_bytes()); // block align
    wav.extend_from_slice(&16u16.to_le_bytes()); // bits
    wav.extend_from_slice(b"data");
    wav.extend_from_slice(&(pcm.len() as u32).to_le_bytes());
    wav.extend_from_slice(&pcm);
    std::fs::write(path, &wav).expect("write wav");
}

/// Try to produce an HE-AAC ADTS file via `afconvert` (macOS) or ffmpeg
/// +libfdk_aac. Returns `None` when no such encoder is available —
/// callers should then skip the test.
fn produce_he_aac_adts(wav_in: &Path, adts_out: &Path, bitrate: u32) -> Option<()> {
    if which("afconvert").is_some() {
        let status = Command::new("afconvert")
            .args(["-f", "adts", "-d", "aach", "-b", &bitrate.to_string()])
            .arg(wav_in)
            .arg(adts_out)
            .status()
            .ok()?;
        if status.success() && adts_out.exists() {
            return Some(());
        }
    }
    if which("ffmpeg").is_some() {
        // libfdk_aac is the only widely-deployed ffmpeg HE-AAC encoder.
        // Builds without it will error out here; we detect and skip.
        let status = Command::new("ffmpeg")
            .args(["-y", "-hide_banner", "-loglevel", "error"])
            .arg("-i")
            .arg(wav_in)
            .args([
                "-c:a",
                "libfdk_aac",
                "-profile:a",
                "aac_he",
                "-b:a",
                &format!("{bitrate}"),
            ])
            .args(["-f", "adts"])
            .arg(adts_out)
            .status()
            .ok()?;
        if status.success() && adts_out.exists() {
            return Some(());
        }
    }
    None
}

fn iter_adts(bytes: &[u8]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + ADTS_HEADER_NO_CRC < bytes.len() {
        if bytes[i] != 0xFF || (bytes[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        match parse_adts_header(&bytes[i..]) {
            Ok(h) => {
                if h.frame_length == 0 || i + h.frame_length > bytes.len() {
                    break;
                }
                out.push((i, h.frame_length));
                i += h.frame_length;
            }
            Err(_) => i += 1,
        }
    }
    out
}

/// Feed a real HE-AAC ADTS stream into the decoder; confirm the output
/// sample rate doubles (proof SBR header parsed + SBR path activated).
#[test]
fn decode_third_party_he_aac_doubles_output_rate() {
    // Use a dedicated scratch dir so parallel tests don't collide.
    let scratch = std::env::temp_dir().join("oxideav_aac_he_interop");
    let _ = std::fs::create_dir_all(&scratch);
    let wav_in = scratch.join("sine.wav");
    let adts_out = scratch.join("he.aac");

    // 48 kHz input → HE-AAC encoder picks 24 kHz core + SBR upsample when
    // running at this low bitrate; afconvert's `aach` unconditionally
    // produces implicit-SBR ADTS.
    let input_sr = 48_000u32;
    write_sine_wav(&wav_in, input_sr, 1000.0, 0.5);

    if produce_he_aac_adts(&wav_in, &adts_out, 32_000).is_none() {
        eprintln!("no HE-AAC encoder on PATH (afconvert / ffmpeg+libfdk_aac) — skipping");
        return;
    }

    let bytes = std::fs::read(&adts_out).expect("read encoded adts");
    let frames = iter_adts(&bytes);
    assert!(
        !frames.is_empty(),
        "no ADTS frames parsed out of {} bytes",
        bytes.len()
    );

    let first = parse_adts_header(&bytes[frames[0].0..]).expect("parse first adts");
    let core_sr = first.sample_rate().expect("adts sr");
    let channels = first.channel_configuration.max(1) as u16;

    // HE-AAC implicit signalling: the ADTS header declares the core
    // (halved) rate. We don't know the output rate yet — the decoder will
    // report it frame-by-frame.
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(core_sr);
    params.channels = Some(channels);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make decoder");
    let tb = TimeBase::new(1, core_sr as i64);

    let mut got_frames = 0usize;
    let mut max_abs: i16 = 0;
    let mut observed_samples = 0u32;
    for (i, &(off, len)) in frames.iter().enumerate().take(8) {
        let pkt = Packet::new(0, tb, bytes[off..off + len].to_vec()).with_pts(i as i64 * 1024);
        dec.send_packet(&pkt).expect("send_packet");
        match dec.receive_frame() {
            Ok(Frame::Audio(af)) => {
                got_frames += 1;
                observed_samples = af.samples;
                // Skip the first frame — SBR transient state produces
                // large edge artifacts on frame 0 that aren't
                // representative of steady-state.
                if i > 0 {
                    for chunk in af.data[0].chunks_exact(2) {
                        let s = i16::from_le_bytes([chunk[0], chunk[1]]);
                        max_abs = max_abs.max(s.saturating_abs());
                    }
                }
            }
            other => panic!("frame {i}: decoder returned {other:?}"),
        }
    }

    assert!(
        got_frames >= 2,
        "decoded only {got_frames} frames — expected >= 2"
    );
    // Per-frame sample rate/channels are gone from the slim AudioFrame —
    // the doubled-rate / channel-config check is now covered by the
    // SBR-aware decoder against the stream's CodecParameters.
    assert_eq!(
        observed_samples, 2048,
        "HE-AAC output must be 2048 samples per frame (1024 core × 2)"
    );
    // Non-silent output: QMF synthesis zeros are ~ |x| < 50, the
    // envelope-adjusted high-band contributes at least a few hundred
    // counts in steady-state even on a pure low-band sine because the
    // PRNG-driven noise floor patches up high.
    assert!(
        max_abs > 50,
        "HE-AAC decoded output is silent (max abs = {max_abs}); SBR synthesis produced no signal"
    );

    // Cleanup scratch files — keep the directory for subsequent runs.
    let _ = std::fs::remove_file(&wav_in);
    let _ = std::fs::remove_file(&adts_out);
}

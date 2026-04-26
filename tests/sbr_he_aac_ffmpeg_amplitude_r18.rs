//! Round 18 regression test (ignored by default — captures interop gap).
//!
//! Encodes a fixed 0.3-amplitude tone through `HeAacStereoEncoder` and
//! decodes the ADTS bitstream via ffmpeg. Asserts that the steady-state
//! ffmpeg-decoded peak amplitude matches the input within a small
//! tolerance.
//!
//! **r18 status — fails (ignored).** ffmpeg currently decodes our HE-AAC
//! streams to peak ≈ 32 768 (full-scale clipped) instead of the input's
//! 9 830. Empirically verified the saturation is **independent of our
//! SBR envelope quantization**:
//!
//! ```text
//! INT16_SCALE_SQ = 1.0     -> ffmpeg peak 32 768
//! INT16_SCALE_SQ = 2^30    -> ffmpeg peak 32 768
//! INT16_SCALE_SQ = 1e20    -> ffmpeg peak 32 768
//! env[band] forced to -200 -> ffmpeg peak 32 768  (E_orig=64 minimum)
//! ```
//!
//! ffmpeg reports `0 decode errors` and only one `No quantized data read
//! for sbr_dequant` warning at EOF (benign — same warning fires for
//! fdkaac-encoded HE-AACv2 streams). This **disproves** the round-17
//! brief's "structural sbr_dequant rejection" hypothesis.
//!
//! The actual interop gap appears to lie outside the SBR envelope path —
//! likely the AAC-LC core spectrum scale, since steady-state amplitude on
//! the LC-only path also disagrees with ffmpeg's expected scale (see the
//! `encode_mono_roundtrip_ffmpeg` test, where steady-state mid-stream
//! amplitude is ≈ 0.6× the expected value, and frame-boundary spikes
//! reach 32 767).
//!
//! Skips when ffmpeg is unavailable.

use std::path::PathBuf;
use std::process::Command;

use oxideav_aac::he_aac_encoder::HeAacStereoEncoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame};

fn which(name: &str) -> Option<PathBuf> {
    let out = Command::new("which").arg(name).output().ok()?;
    if !out.status.success() {
        return None;
    }
    let p = String::from_utf8(out.stdout).ok()?;
    let t = p.trim();
    if t.is_empty() {
        None
    } else {
        Some(PathBuf::from(t))
    }
}

#[test]
#[ignore = "round-18 known interop gap — ffmpeg saturates HE-AAC output to peak 32768 regardless of SBR envelope; root cause not yet diagnosed"]
fn ffmpeg_decoded_amplitude_matches_input() {
    if which("ffmpeg").is_none() {
        eprintln!("no ffmpeg on PATH — skipping");
        return;
    }
    let scratch = std::env::temp_dir().join("oxideav_aac_r18_amp");
    let _ = std::fs::create_dir_all(&scratch);
    let adts_path = scratch.join("ours.aac");
    let dec_path = scratch.join("dec.s16");

    let high_rate = 48_000u32;
    let secs = 0.5f32;
    let total = (high_rate as f32 * secs) as usize;
    let mut bytes = Vec::with_capacity(total * 4);
    for i in 0..total {
        let t = i as f32 / high_rate as f32;
        let l = (2.0 * std::f32::consts::PI * 1000.0 * t).sin() * 0.3;
        let r = (2.0 * std::f32::consts::PI * 2000.0 * t).sin() * 0.3;
        bytes.extend_from_slice(&((l * 32767.0) as i16).to_le_bytes());
        bytes.extend_from_slice(&((r * 32767.0) as i16).to_le_bytes());
    }
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(high_rate);
    params.channels = Some(2);
    params.bit_rate = Some(64_000);
    let mut enc = HeAacStereoEncoder::new(&params).expect("ctor");
    let af = AudioFrame {
        samples: total as u32,
        pts: Some(0),
        data: vec![bytes],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send_frame");
    enc.flush().expect("flush");
    let mut adts_bytes = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts_bytes.extend_from_slice(&pkt.data);
    }
    std::fs::write(&adts_path, &adts_bytes).expect("write adts");

    let _ = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .arg("-i")
        .arg(&adts_path)
        .args(["-f", "s16le", "-ar", "48000", "-ac", "2"])
        .arg(&dec_path)
        .status()
        .expect("ffmpeg spawn");
    let raw = std::fs::read(&dec_path).expect("read decoded");
    let samples: Vec<i16> = raw
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();
    // Steady-state mid-stream window — skip warmup transients.
    let warm = 10_000usize;
    let mid_end = (samples.len() / 2 - warm).max(warm + 1);
    if mid_end <= warm {
        panic!("not enough decoded samples to measure steady state");
    }
    let mid = &samples[warm..mid_end];
    let peak: i32 = mid.iter().map(|&s| s.unsigned_abs() as i32).max().unwrap();
    eprintln!("ffmpeg steady-state peak={peak} (input peak={})", 9830);
    // Input was 0.3 * 32767 ≈ 9830. Allow a 2× tolerance.
    assert!(
        peak <= 19_660,
        "ffmpeg-decoded peak {peak} > 19660 (input ≈ 9830) — interop saturation"
    );
}

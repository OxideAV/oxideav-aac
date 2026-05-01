//! Round 24 follow-up: check that our HE-AAC **mono** stream (vs the
//! stereo path used by the r18 amplitude test) ALSO saturates through
//! ffmpeg, so we can isolate the saturation to "the SBR FIL extension"
//! independently of CPE / coupling specifics.
//!
//! Skips when ffmpeg is missing.

use std::path::PathBuf;
use std::process::Command;

use oxideav_aac::he_aac_encoder::HeAacMonoEncoder;
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
fn ffmpeg_decodes_our_he_aac_mono_amplitude() {
    if which("ffmpeg").is_none() {
        eprintln!("ffmpeg not on PATH — skipping");
        return;
    }
    let scratch = std::env::temp_dir().join("oxideav_aac_r24_mono_amp");
    std::fs::create_dir_all(&scratch).unwrap();
    let aac_path = scratch.join("ours_mono.aac");
    let pcm_path = scratch.join("ours_mono.s16");

    let high_rate = 48_000u32;
    let secs = 0.5f32;
    let total = (high_rate as f32 * secs) as usize;
    let mut bytes = Vec::with_capacity(total * 2);
    for i in 0..total {
        let t = i as f32 / high_rate as f32;
        let v = (2.0 * std::f32::consts::PI * 1000.0 * t).sin() * 0.3;
        bytes.extend_from_slice(&((v * 32767.0) as i16).to_le_bytes());
    }
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(high_rate);
    params.channels = Some(1);
    params.bit_rate = Some(48_000);
    let mut enc = HeAacMonoEncoder::new(&params).expect("ctor");
    let af = AudioFrame {
        samples: total as u32,
        pts: Some(0),
        data: vec![bytes],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send");
    enc.flush().expect("flush");
    let mut adts = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts.extend_from_slice(&pkt.data);
    }
    std::fs::write(&aac_path, &adts).expect("write");

    let _ = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .arg("-i")
        .arg(&aac_path)
        .args(["-f", "s16le", "-ar", "48000", "-ac", "1"])
        .arg(&pcm_path)
        .status()
        .expect("ffmpeg");
    let raw = std::fs::read(&pcm_path).expect("read decoded");
    let samples: Vec<i16> = raw
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();
    let warm = 10_000usize;
    let mid_end = (samples.len() - 2048).max(warm + 1);
    let mid = &samples[warm..mid_end];
    let peak: i32 = mid.iter().map(|&s| s.unsigned_abs() as i32).max().unwrap();
    let rms: f64 = (mid.iter().map(|&s| (s as f64).powi(2)).sum::<f64>() / mid.len() as f64).sqrt();
    eprintln!(
        "MONO HE-AAC ffmpeg-decode: peak={peak} rms={:.0} (input peak ≈ 9830, RMS ≈ 6951)",
        rms
    );
    // No assertion — pure diagnostic. The r18 test is the gating one.
}

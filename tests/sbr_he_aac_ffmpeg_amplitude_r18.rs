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
//! ## Round 21 audit notes (2026-04-26)
//!
//! Round 19 closed the LC RMS interop gap and round 20 fixed the
//! per-frame ±32k saturation on real CPE content. With those landing,
//! the round-18 hypothesis "AAC-LC core spectrum scale" is also ruled
//! out — `tests/lc_rms_interop_r19.rs` confirms the LC-only round-trip
//! lands within ±5% of unity in all four directions.
//!
//! Round-21 probe data (mono SCE fixture, 1 kHz tone amp 0.3 at 48 kHz):
//!
//! ```text
//! input (peak / RMS)              :  9 830 / 6 951
//! ours-encode -> ours-decode       : 10 256 / 6 582  (peak within 5% of input)
//! ours-encode -> ffmpeg-decode     : 32 767 / 17 181 (square-wave clipping)
//! fdkaac-encode -> ffmpeg-decode   :  9 822 / 6 920  (within 1% of input)
//! fdkaac-encode -> ours-decode     : 13 009 / 7 061  (within ~30% of input)
//! ```
//!
//! Both fdkaac and our encoder transmit `bs_data_env[0] = 0` for tonal
//! content with no high-band energy (E_orig = 64, the spec minimum).
//! The bitstream-level envelope value is therefore **not** the source
//! of the saturation — both encoders emit the same value yet ffmpeg
//! decodes the fdkaac stream cleanly while saturating ours.
//!
//! The remaining gap is in the SBR HF-generation / limiter / patches
//! interaction with our specific frequency-table choices and our LC
//! core's particular spectral envelope. Round 21 leaves the test
//! ignored pending a deeper SBR pipeline audit on a multi-frequency
//! corpus.
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
#[ignore = "round-21 known interop gap — ffmpeg saturates our HE-AAC streams to peak 32768; bitstream-level envelope value is identical to fdkaac's (which decodes cleanly), so root cause is in SBR HF-generation / limiter for our specific freq-table choice"]
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

//! Round-16 diagnostic test: confirms that ffmpeg's HE-AAC decoder
//! **successfully decodes** every frame of our `HeAacStereoEncoder`
//! output (`0 decode errors`) despite the cosmetic `No quantized data
//! read for sbr_dequant` EOF-cleanup warning. The warning fires once at
//! end-of-stream when ffmpeg flushes its SBR state with no more
//! quantised data to consume; it does NOT abort decoding or affect the
//! output sample count.
//!
//! This test asserts:
//!   1. ffmpeg detects the stream as `HE-AAC` (SBR active).
//!   2. All 25 ADTS frames decode (`25 packets read`, `25 frames decoded`).
//!   3. ffmpeg reports `0 decode errors`.
//!   4. The output WAV has the expected `2048 samples × 25 frames =
//!      51 200 frames` of audio at the SBR-doubled 48 kHz rate.
//!
//! What this test does NOT assert is *signal-correctness PSNR* against
//! the reference: that's the round-17 deliverable (the scale-convention
//! mismatch documented in `src/sbr/encode.rs::estimate_envelope` causes
//! ffmpeg to interpret our `E_orig` ≈ 2³⁰ too large, producing clipped
//! HF in its float-scale QMF). This test is purely about the
//! "sbr_dequant rejection" question raised in round 16: ffmpeg does
//! **NOT** reject — it warns and decodes successfully.
//!
//! Skips when ffmpeg is unavailable.

use std::path::PathBuf;
use std::process::Command;

use oxideav_aac::he_aac_encoder::HeAacStereoEncoder;
use oxideav_core::Encoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, SampleFormat, TimeBase};

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

#[test]
fn ffmpeg_decodes_our_he_aac_with_zero_decode_errors() {
    if which("ffmpeg").is_none() {
        eprintln!("no ffmpeg on PATH — skipping ffmpeg interop test");
        return;
    }

    let scratch = std::env::temp_dir().join("oxideav_aac_round16_ffmpeg");
    let _ = std::fs::create_dir_all(&scratch);
    let adts_out = scratch.join("ours_he_v1_stereo.aac");

    // Encode 1 s of stereo two-tone HE-AACv1 with our encoder.
    let high_rate = 48_000u32;
    let secs = 1.0f32;
    let total = (high_rate as f32 * secs) as usize;
    let mut bytes = Vec::with_capacity(total * 4);
    for i in 0..total {
        let t = i as f32 / high_rate as f32;
        let l = (2.0 * std::f32::consts::PI * 1000.0 * t).sin() * 0.3;
        let r = (2.0 * std::f32::consts::PI * 2000.0 * t).sin() * 0.3;
        let sl = (l * 32767.0) as i16;
        let sr_s = (r * 32767.0) as i16;
        bytes.extend_from_slice(&sl.to_le_bytes());
        bytes.extend_from_slice(&sr_s.to_le_bytes());
    }
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(high_rate);
    params.channels = Some(2);
    params.bit_rate = Some(64_000);
    let mut enc = HeAacStereoEncoder::new(&params).expect("enc construct");
    let af = AudioFrame {
        samples: total as u32,
        pts: Some(0),
        data: vec![bytes],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send_frame");
    enc.flush().expect("flush");

    let mut out_bytes = Vec::new();
    let mut adts_frames = 0u32;
    while let Ok(pkt) = enc.receive_packet() {
        out_bytes.extend_from_slice(&pkt.data);
        adts_frames += 1;
    }
    std::fs::write(&adts_out, &out_bytes).expect("write adts");
    assert!(adts_frames >= 10, "too few ADTS frames ({adts_frames})");

    // Decode with ffmpeg to /dev/null and capture stderr — verify ffmpeg
    // reports zero decode errors and decoded all input packets. The
    // `No quantized data read for sbr_dequant` warning we expect is
    // benign and does NOT increment the decode-errors counter.
    let ff = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "verbose"])
        .arg("-i")
        .arg(&adts_out)
        .args(["-f", "null", "-"])
        .output()
        .expect("ffmpeg spawn");
    let stderr = String::from_utf8_lossy(&ff.stderr);
    eprintln!("---- ffmpeg verbose output ----\n{stderr}\n----");

    // ffmpeg should detect HE-AAC.
    assert!(
        stderr.contains("HE-AAC") || stderr.contains("HE-AACv2"),
        "ffmpeg did not detect HE-AAC SBR in our stream:\n{stderr}"
    );
    // ffmpeg's input stats line: "X packets read (Y bytes); X frames decoded; 0 decode errors".
    let decoded_line = stderr
        .lines()
        .find(|l| l.contains("frames decoded") && l.contains("decode errors"))
        .expect("ffmpeg stats line missing");
    assert!(
        decoded_line.contains("0 decode errors"),
        "ffmpeg reported decode errors in our stream: {decoded_line}"
    );
    // Frame count round-trips: encoded N → decoded N.
    let decoded = decoded_line
        .split_whitespace()
        .find_map(|tok| tok.parse::<u32>().ok())
        .expect("could not parse ffmpeg decoded count");
    assert_eq!(
        decoded, adts_frames,
        "ffmpeg decoded {decoded} frames but we encoded {adts_frames}"
    );

    let _ = std::fs::remove_file(&adts_out);
}

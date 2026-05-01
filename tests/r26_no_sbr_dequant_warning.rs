//! Regression pin for round-26 (task #111): the HE-AAC mono encoder no
//! longer trips ffmpeg's "No quantized data read for sbr_dequant" warning
//! on the round-18/24 1 kHz / 48 kHz / 0.3 amp tone fixture.
//!
//! Three independent bugs landed together in this round:
//!
//! 1. **`QmfAnalysis64` modulation matrix** (encoder side, `sbr/encode.rs`)
//!    used the **decoder-style** matrix `2 * exp(i*pi/128*(k+0.5)*(2n-0.5))`
//!    instead of the encoder-side spec matrix
//!    `exp(i*pi/128*(k+0.5)*(2n+1))` from ISO/IEC 14496-3 §4.B.18.2 /
//!    Figure 4.B.16. Each subband landed at the wrong centre frequency,
//!    leaking a 1 kHz tone into the SBR-band region (env_sf[0][0] = 29
//!    vs fdkaac's 0).
//!
//! 2. **Time-direction envelope writer** did not clamp the reconstructed
//!    accumulator to `[0, 127]`, so a frame whose raw env was lower than
//!    the previous frame's reconstructed env (typical for tonal input
//!    with some QMF cold-start transient) emitted a delta that decoded
//!    to a negative `env_facs_q`. ffmpeg stores it as `uint8_t`, sees
//!    `255 > 127`, logs `"env_facs_q 255 is invalid"` and tears the SBR
//!    apply path down. Encoder also stored the **raw** (clamping-naive)
//!    envelope as the next-frame baseline; the decoder's clamped baseline
//!    diverged → drift → repeated warning.
//!
//! 3. **HE-AAC mono encoder flush** failed to stage an SBR FIL element
//!    for the inner AAC encoder's silence-block tail (the held overlap
//!    that drains out on `inner.flush()`), so the very last AAC frame
//!    of every encoded stream had no `FIL/EXT_SBR_DATA` element. ffmpeg's
//!    `ff_aac_sbr_apply` then walked into the `start && !ready_for_dequant`
//!    branch on that final frame and logged
//!    `"No quantized data read for sbr_dequant"`.
//!
//! The fix landed in:
//!  - `src/sbr/encode.rs::QmfAnalysis64::process` — corrected matrix M
//!  - `src/sbr/encode.rs::write_envelope_1_5db_*_recon` — `[0, 127]` clamp
//!    + return reconstructed values
//!  - `src/sbr/encode.rs::SbrEncoder::emit_sbr_payload` — record
//!    reconstructed env as next-frame baseline
//!  - `src/he_aac_encoder.rs::HeAacMonoEncoder::flush` — stage one
//!    silence SBR FIL before `inner.flush()` so the tail frame carries
//!    EXT_SBR_DATA
//!
//! This test runs ffmpeg and asserts the warning string is **absent**
//! from stderr. Skips when ffmpeg is not on PATH.

use std::path::PathBuf;
use std::process::Command;

use oxideav_aac::he_aac_encoder::HeAacMonoEncoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame};

const HIGH_RATE: u32 = 48_000;
const SECS: f32 = 0.5;
const AMP: f32 = 0.3;
const TONE_HZ: f32 = 1_000.0;

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

fn make_mono_pcm_s16() -> Vec<u8> {
    let total = (HIGH_RATE as f32 * SECS) as usize;
    let mut bytes = Vec::with_capacity(total * 2);
    for i in 0..total {
        let t = i as f32 / HIGH_RATE as f32;
        let v = (2.0 * std::f32::consts::PI * TONE_HZ * t).sin() * AMP;
        bytes.extend_from_slice(&((v * 32767.0) as i16).to_le_bytes());
    }
    bytes
}

fn encode_ours() -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(HIGH_RATE);
    params.channels = Some(1);
    params.bit_rate = Some(48_000);
    let mut enc = HeAacMonoEncoder::new(&params).expect("ctor");
    let pcm = make_mono_pcm_s16();
    let total = (HIGH_RATE as f32 * SECS) as usize;
    let af = AudioFrame {
        samples: total as u32,
        pts: Some(0),
        data: vec![pcm],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send");
    enc.flush().expect("flush");
    let mut out = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        out.extend_from_slice(&pkt.data);
    }
    out
}

#[test]
fn ffmpeg_does_not_warn_about_sbr_dequant() {
    let Some(ffmpeg) = which("ffmpeg") else {
        eprintln!("ffmpeg not on PATH — skipping");
        return;
    };
    let scratch = std::env::temp_dir().join("oxideav_aac_r26_dequant");
    std::fs::create_dir_all(&scratch).expect("mkdir");
    let aac_path = scratch.join("ours.aac");
    let bytes = encode_ours();
    std::fs::write(&aac_path, &bytes).expect("write aac");

    let out = Command::new(&ffmpeg)
        .args(["-hide_banner", "-loglevel", "warning"])
        .arg("-i")
        .arg(&aac_path)
        .args(["-f", "null", "-"])
        .output()
        .expect("ffmpeg spawn");
    let stderr = String::from_utf8_lossy(&out.stderr);
    eprintln!("ffmpeg stderr:\n{stderr}");
    assert!(
        !stderr.contains("No quantized data read for sbr_dequant"),
        "ffmpeg logged the SBR dequant warning — round-26 fix regression"
    );
    assert!(
        !stderr.contains("env_facs_q") || !stderr.contains("invalid"),
        "ffmpeg logged env_facs_q invalid — round-26 envelope-clamp regression"
    );
}

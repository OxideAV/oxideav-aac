//! ffmpeg interop check for the HE-AACv1 stereo encoder.
//!
//! Encode a stereo 1 kHz / 2 kHz tone pair through our
//! `HeAacStereoEncoder` (24 kHz core, 48 kHz output) and decode the
//! resulting ADTS HE-AACv1 stream through ffmpeg's native AAC decoder.
//! Confirm:
//!   * ffmpeg accepts the stream (no decode error).
//!   * ffmpeg's output is at the SBR-doubled 48 kHz rate, stereo.
//!   * Each channel reconstructs its source tone with a Goertzel ratio
//!     well above the per-channel RMS floor (≥ 5 dB SNR — independent
//!     coupling means each channel is a separately-coded SBR mono
//!     payload, no cross-channel leakage).
//!
//! Skips gracefully if ffmpeg is not on PATH.

use std::path::PathBuf;
use std::process::Command;

use oxideav_aac::he_aac_encoder::HeAacStereoEncoder;
#[allow(unused_imports)]
use oxideav_core::Encoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame};

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

fn pcm_two_tones_stereo_bytes(freq_l: f32, freq_r: f32, sr: u32, secs: f32, amp: f32) -> Vec<u8> {
    let total = (sr as f32 * secs) as usize;
    let mut out = Vec::with_capacity(total * 4);
    for i in 0..total {
        let t = i as f32 / sr as f32;
        let l = (2.0 * std::f32::consts::PI * freq_l * t).sin() * amp;
        let r = (2.0 * std::f32::consts::PI * freq_r * t).sin() * amp;
        let sl = (l * 32767.0) as i16;
        let sr_s = (r * 32767.0) as i16;
        out.extend_from_slice(&sl.to_le_bytes());
        out.extend_from_slice(&sr_s.to_le_bytes());
    }
    out
}

fn goertzel_db(buf: &[f32], freq: f32, sr: f32) -> (f32, f32, f32) {
    let analysis_start = buf.len() / 4;
    let w = 2.0 * std::f32::consts::PI * freq / sr;
    let coeff = 2.0 * w.cos();
    let (mut s0, mut s1) = (0.0f32, 0.0f32);
    for &x in &buf[analysis_start..] {
        let s = x + coeff * s0 - s1;
        s1 = s0;
        s0 = s;
    }
    let mag = (s0 * s0 + s1 * s1 - coeff * s0 * s1).sqrt();
    let total_rms: f32 = (buf.iter().map(|v| v * v).sum::<f32>() / buf.len() as f32).sqrt();
    let n = (buf.len() - analysis_start) as f32;
    let tone_rms = mag / (n / 2.0).max(1.0).sqrt();
    let snr_db = 20.0 * (tone_rms / total_rms.max(1e-9)).log10();
    (mag, total_rms, snr_db)
}

#[test]
fn ffmpeg_decodes_our_he_aac_stereo() {
    let ffmpeg = match which("ffmpeg") {
        Some(p) => p,
        None => {
            eprintln!("no ffmpeg on PATH — skipping HE-AACv1 stereo ffmpeg interop test");
            return;
        }
    };

    let high_rate = 48_000u32;
    let freq_l = 1000.0f32;
    let freq_r = 2000.0f32;
    let secs = 1.0f32;
    let pcm_bytes = pcm_two_tones_stereo_bytes(freq_l, freq_r, high_rate, secs, 0.3);

    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(high_rate);
    params.channels = Some(2);
    params.bit_rate = Some(64_000);
    let mut enc = HeAacStereoEncoder::new(&params).expect("enc construct");
    let n = pcm_bytes.len() / 4;
    let af = AudioFrame {
        samples: n as u32,
        pts: Some(0),
        data: vec![pcm_bytes],
    };
    enc.send_frame(&Frame::Audio(af)).expect("enc send");
    enc.flush().expect("enc flush");
    let mut adts = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts.extend_from_slice(&pkt.data);
    }
    assert!(!adts.is_empty(), "encoder produced no ADTS data");

    let scratch = std::env::temp_dir().join("oxideav_he_aac_stereo_ffmpeg");
    let _ = std::fs::create_dir_all(&scratch);
    let aac_path = scratch.join("our_stereo.aac");
    let pcm_path = scratch.join("ffmpeg_decoded.pcm");
    let _ = std::fs::remove_file(&pcm_path);
    std::fs::write(&aac_path, &adts).expect("write adts");

    // ffmpeg decode at the SBR-doubled rate, forced 48 kHz / stereo.
    let status = Command::new(&ffmpeg)
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .arg("-i")
        .arg(&aac_path)
        .args(["-f", "s16le", "-ar", "48000", "-ac", "2"])
        .arg(&pcm_path)
        .status()
        .expect("spawn ffmpeg");
    assert!(
        status.success() && pcm_path.exists(),
        "ffmpeg failed to decode our HE-AACv1 stereo stream",
    );

    let pcm = std::fs::read(&pcm_path).expect("read decoded pcm");
    let n_samples = pcm.len() / 4; // 2 channels * 2 bytes
    assert!(
        n_samples > 4096,
        "ffmpeg decoded too few samples: {n_samples}"
    );

    let mut out_l = Vec::with_capacity(n_samples);
    let mut out_r = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let off = i * 4;
        let l = i16::from_le_bytes([pcm[off], pcm[off + 1]]);
        let r = i16::from_le_bytes([pcm[off + 2], pcm[off + 3]]);
        out_l.push(l as f32 / 32768.0);
        out_r.push(r as f32 / 32768.0);
    }
    let (mag_l, rms_l, snr_l) = goertzel_db(&out_l, freq_l, high_rate as f32);
    let (mag_r, rms_r, snr_r) = goertzel_db(&out_r, freq_r, high_rate as f32);
    eprintln!(
        "ffmpeg decode of our HE-AAC stereo: \
         L mag={mag_l:.3} rms={rms_l:.4} snr={snr_l:.2} dB | \
         R mag={mag_r:.3} rms={rms_r:.4} snr={snr_r:.2} dB"
    );

    assert!(rms_l > 1e-3, "L silent after ffmpeg decode: rms={rms_l}");
    assert!(rms_r > 1e-3, "R silent after ffmpeg decode: rms={rms_r}");
    assert!(
        mag_l > 1.0,
        "L tone not present after ffmpeg decode: mag={mag_l}",
    );
    assert!(
        mag_r > 1.0,
        "R tone not present after ffmpeg decode: mag={mag_r}",
    );
    // Independent coupling: per-channel SNR should match the per-channel
    // mono HE-AAC baseline. After fixing the Table 4.66 channel-pair
    // ordering bug (envelope L+R, then noise L+R — not interleaved per
    // channel), R now reconstructs through ffmpeg at ~23 dB SNR for a
    // 2 kHz tone, matching the mono baseline within rounding. L is
    // ~34 dB for the 1 kHz tone. Lock these in conservatively.
    assert!(
        snr_l > 30.0,
        "L SNR after ffmpeg decode too low: {snr_l:.2} dB",
    );
    assert!(
        snr_r > 20.0,
        "R SNR after ffmpeg decode too low: {snr_r:.2} dB",
    );
}

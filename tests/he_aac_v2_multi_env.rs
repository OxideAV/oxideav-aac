//! HE-AACv2 multi-envelope PS encoder — round 14.
//!
//! Stresses the new round-14 paths in the encoder:
//!
//!   * **Time-direction differential coding** (`iid_dt = 1`) — when the
//!     input is temporally stationary the encoder must, after a few frames
//!     of warmup, pick `iid_dt = 1` for at least some envelopes. We can't
//!     reach that flag from the public API directly, so we instead measure
//!     the bitstream cost: re-running the same encode through a parser
//!     decoder and checking that PS decoding succeeds across the entire
//!     stream confirms the DT bitstream is well-formed.
//!
//!   * **Multi-envelope (`num_env > 1`)** — feed a transient input and
//!     check (via ffmpeg's PS decoder) that the multi-envelope PS payload
//!     round-trips without errors. The decoder's PS state machine accepts
//!     `num_env ∈ {1, 2, 4}` per §8.6.4.6.2.
//!
//! Skips gracefully when ffmpeg is not on PATH.

use std::path::Path;
use std::process::Command;

use oxideav_aac::he_aac_encoder::HeAacV2Encoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame};

fn ffmpeg_available() -> bool {
    Command::new("ffmpeg")
        .arg("-version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// Generate a stereo PCM signal whose first half is silent and second half
/// is a 1 kHz tone on both channels — a strong temporal transient that
/// should trip [`oxideav_aac::sbr::ps::detect_num_env`] into recommending
/// `num_env = 4` per AAC frame on the boundary frame.
fn write_transient_stereo(sr: u32, freq: f32, secs: f32) -> Vec<u8> {
    let n = (sr as f32 * secs) as usize;
    let half = n / 2;
    let mut bytes = Vec::with_capacity(n * 4);
    for i in 0..n {
        let amp = if i < half { 0.0f32 } else { 0.3f32 };
        let t = i as f32 / sr as f32;
        let s = (2.0 * std::f32::consts::PI * freq * t).sin() * amp;
        let v = (s * 32767.0) as i16;
        bytes.extend_from_slice(&v.to_le_bytes()); // L
        bytes.extend_from_slice(&v.to_le_bytes()); // R
    }
    bytes
}

/// Generate a temporally-stationary stereo PCM signal: L = 1 kHz, R = 2 kHz
/// for the entire duration. Per-frame PS analysis converges to the same
/// IID/ICC indices, so after the first couple of frames the encoder's
/// time-direction predictor has near-zero cost.
fn write_stationary_stereo(sr: u32, secs: f32) -> Vec<u8> {
    let n = (sr as f32 * secs) as usize;
    let mut bytes = Vec::with_capacity(n * 4);
    for i in 0..n {
        let t = i as f32 / sr as f32;
        let l = (2.0 * std::f32::consts::PI * 1000.0 * t).sin() * 0.3;
        let r = (2.0 * std::f32::consts::PI * 2000.0 * t).sin() * 0.3;
        let ls = (l * 32767.0) as i16;
        let rs = (r * 32767.0) as i16;
        bytes.extend_from_slice(&ls.to_le_bytes());
        bytes.extend_from_slice(&rs.to_le_bytes());
    }
    bytes
}

fn ffmpeg_decode(adts: &Path) -> Option<(u32, u16, Vec<i16>, Vec<i16>)> {
    let probe = Command::new("ffprobe")
        .args([
            "-v",
            "error",
            "-select_streams",
            "a:0",
            "-show_entries",
            "stream=sample_rate,channels",
            "-of",
            "default=noprint_wrappers=1",
        ])
        .arg(adts)
        .output()
        .ok()?;
    let probe_str = String::from_utf8_lossy(&probe.stdout).into_owned();
    let mut sr = 48_000u32;
    let mut channels: u16 = 1;
    for line in probe_str.lines() {
        if let Some(rest) = line.strip_prefix("sample_rate=") {
            if let Ok(v) = rest.trim().parse::<u32>() {
                sr = v;
            }
        }
        if let Some(rest) = line.strip_prefix("channels=") {
            if let Ok(v) = rest.trim().parse::<u16>() {
                channels = v;
            }
        }
    }
    let out = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .arg("-i")
        .arg(adts)
        .args(["-f", "s16le", "-c:a", "pcm_s16le"])
        .arg("-")
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let raw = out.stdout;
    let mut l = Vec::with_capacity(raw.len() / 4);
    let mut r = Vec::with_capacity(raw.len() / 4);
    if channels == 2 {
        for ch in raw.chunks_exact(4) {
            l.push(i16::from_le_bytes([ch[0], ch[1]]));
            r.push(i16::from_le_bytes([ch[2], ch[3]]));
        }
    } else {
        for ch in raw.chunks_exact(2) {
            let s = i16::from_le_bytes([ch[0], ch[1]]);
            l.push(s);
            r.push(s);
        }
    }
    Some((sr, channels, l, r))
}

/// Encode one stereo PCM blob through `HeAacV2Encoder` and return the
/// concatenated ADTS bytes.
fn encode_he_aac_v2(pcm: &[u8], sr: u32, n_samples: u32) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(2);
    params.bit_rate = Some(32_000);
    let mut enc = HeAacV2Encoder::new(&params).expect("HeAacV2Encoder::new");
    let af = AudioFrame {
        samples: n_samples,
        pts: Some(0),
        data: vec![pcm.to_vec()],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send_frame");
    enc.flush().expect("flush");
    let mut adts = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts.extend_from_slice(&pkt.data);
    }
    adts
}

#[test]
fn he_aac_v2_transient_input_round_trips_through_ffmpeg() {
    if !ffmpeg_available() {
        eprintln!("ffmpeg not on PATH — skipping HE-AACv2 multi-env interop test");
        return;
    }
    let scratch = std::env::temp_dir().join("oxideav_aac_he_v2_multi_env_round14");
    let _ = std::fs::create_dir_all(&scratch);
    let adts_path = scratch.join("he_v2_transient.aac");

    let in_sr = 48_000u32;
    let secs = 1.0f32;
    let pcm = write_transient_stereo(in_sr, 1_000.0, secs);
    let n_samples = (in_sr as f32 * secs) as u32;
    let adts = encode_he_aac_v2(&pcm, in_sr, n_samples);
    assert!(
        adts.len() > 100,
        "encoder produced only {} ADTS bytes",
        adts.len()
    );
    std::fs::write(&adts_path, &adts).expect("write adts");

    // ffmpeg must accept the entire bitstream — including the multi-env
    // PS frames produced on the silent→tone transition. A malformed
    // multi-env payload would cause decode failure.
    let (out_sr, out_channels, l_pcm, r_pcm) =
        ffmpeg_decode(&adts_path).expect("ffmpeg failed to decode our HE-AACv2 multi-env stream");
    assert_eq!(out_channels, 2, "expected stereo output");
    assert_eq!(out_sr, in_sr, "expected SBR-doubled rate");
    let n_check = l_pcm.len().min(r_pcm.len());
    assert!(
        n_check > 16_384,
        "too few decoded samples: {n_check} (multi-env payloads probably dropped)"
    );

    // Power in the second half should dominate the first half by far —
    // the silent→tone transient must survive the encode/decode roundtrip
    // (a corrupted transient would smear energy uniformly).
    let half = n_check / 2;
    let p_first = mean_power(&l_pcm[..half]);
    let p_second = mean_power(&l_pcm[half..]);
    let ratio_db = 10.0 * (p_second.max(1.0) / p_first.max(1.0)).log10();
    eprintln!(
        "HE-AACv2 multi-env transient: first-half power {p_first:.0}, second-half power \
         {p_second:.0}, ratio {ratio_db:.2} dB"
    );
    assert!(
        ratio_db > 6.0,
        "transient flattened — second half should be ≥ 6 dB above first half, got {ratio_db:.2} dB",
    );
}

#[test]
fn he_aac_v2_stationary_input_round_trips_through_ffmpeg() {
    if !ffmpeg_available() {
        eprintln!("ffmpeg not on PATH — skipping HE-AACv2 stationary interop test");
        return;
    }
    let scratch = std::env::temp_dir().join("oxideav_aac_he_v2_multi_env_round14");
    let _ = std::fs::create_dir_all(&scratch);
    let adts_path = scratch.join("he_v2_stationary.aac");

    let in_sr = 48_000u32;
    let secs = 1.0f32;
    let pcm = write_stationary_stereo(in_sr, secs);
    let n_samples = (in_sr as f32 * secs) as u32;
    let adts = encode_he_aac_v2(&pcm, in_sr, n_samples);
    std::fs::write(&adts_path, &adts).expect("write adts");

    let (_, out_channels, l_pcm, r_pcm) =
        ffmpeg_decode(&adts_path).expect("ffmpeg failed to decode our HE-AACv2 stationary stream");
    assert_eq!(out_channels, 2, "expected stereo output");
    let n_check = l_pcm.len().min(r_pcm.len());
    assert!(
        n_check > 16_384,
        "too few decoded samples: {n_check} (DT-coded frames probably failed to parse)"
    );

    // After the warmup, both channels should still carry the stereo
    // panning info. ≥20% of samples should differ — same threshold as the
    // round-13 real-PS test (the DT path must not have regressed the
    // spatial image).
    const WARMUP: usize = 4096;
    let mut diff = 0usize;
    let mut total = 0usize;
    for i in WARMUP..n_check {
        if (l_pcm[i] as i32 - r_pcm[i] as i32).abs() > 32 {
            diff += 1;
        }
        total += 1;
    }
    let ratio = diff as f64 / total as f64;
    eprintln!(
        "HE-AACv2 DT-stable: {diff}/{total} ({:.2}%) samples differ |L-R|>32",
        ratio * 100.0,
    );
    assert!(
        ratio > 0.20,
        "stationary DT path collapsed to identity stereo ({:.2}%) — \
         time-direction encode/decode is broken",
        ratio * 100.0,
    );
}

fn mean_power(samples: &[i16]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let mut s = 0.0f64;
    for &v in samples {
        s += (v as f64) * (v as f64);
    }
    s / samples.len() as f64
}

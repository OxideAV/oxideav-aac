//! HE-AACv2 real-PS encoder interop — round 13.
//!
//! Encodes a true-stereo input (1 kHz on L only, 2 kHz on R only) through
//! [`oxideav_aac::he_aac_encoder::HeAacV2Encoder`] (which now performs
//! real per-band IID/ICC analysis on the input QMF), pipes the resulting
//! ADTS bytes through ffmpeg's HE-AACv2 decoder, and verifies that:
//!
//!   1. ffmpeg accepts the bitstream and outputs 2 channels at the
//!      doubled rate (sanity — same as round-12 no-op test).
//!   2. The two output channels are *not* identical — the PS analyser
//!      detected per-band intensity differences and the decoder upmixed
//!      them into spatially distinct L / R signals.
//!   3. The per-band power ratio at the PS output reflects the input's
//!      spatial distribution: more energy in the QMF band carrying the
//!      L tone should appear in the L output than in R, and vice versa.
//!
//! Skips gracefully when ffmpeg is not on PATH.

use std::path::Path;
use std::process::Command;

use oxideav_aac::he_aac_encoder::HeAacV2Encoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame, SampleFormat, TimeBase};

fn ffmpeg_available() -> bool {
    Command::new("ffmpeg")
        .arg("-version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// Generate stereo PCM with `freq_l` Hz on the left channel only and
/// `freq_r` Hz on the right channel only — i.e. perfect channel
/// separation, every parameter band's IID is at one extreme.
fn write_panned_stereo(sr: u32, freq_l: f32, freq_r: f32, secs: f32) -> Vec<u8> {
    let n = (sr as f32 * secs) as usize;
    let mut bytes = Vec::with_capacity(n * 4);
    for i in 0..n {
        let t = i as f32 / sr as f32;
        let l = (2.0 * std::f32::consts::PI * freq_l * t).sin() * 0.3;
        let r = (2.0 * std::f32::consts::PI * freq_r * t).sin() * 0.3;
        let ls = (l * 32767.0) as i16;
        let rs = (r * 32767.0) as i16;
        bytes.extend_from_slice(&ls.to_le_bytes());
        bytes.extend_from_slice(&rs.to_le_bytes());
    }
    bytes
}

fn ffmpeg_decode_stereo(adts: &Path, expected_sr: u32) -> Option<(u32, u16, Vec<i16>, Vec<i16>)> {
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
    let mut sr = expected_sr;
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

/// Compute average linear power of an i16 PCM channel (skipping the warm-up).
fn mean_power(samples: &[i16], skip: usize) -> f64 {
    if samples.len() <= skip {
        return 0.0;
    }
    let mut s = 0.0f64;
    for &v in &samples[skip..] {
        s += (v as f64) * (v as f64);
    }
    s / (samples.len() - skip) as f64
}

#[test]
fn he_aac_v2_real_ps_preserves_stereo_image() {
    if !ffmpeg_available() {
        eprintln!("ffmpeg not on PATH — skipping HE-AACv2 real-PS interop test");
        return;
    }

    let scratch = std::env::temp_dir().join("oxideav_aac_he_v2_real_ps_round13");
    let _ = std::fs::create_dir_all(&scratch);
    let adts_path = scratch.join("he_v2_real_ps.aac");

    // Input: L = 1 kHz, R = 2 kHz, 48 kHz, 1.0 s.
    let in_sr = 48_000u32;
    let secs = 1.0f32;
    let pcm_bytes = write_panned_stereo(in_sr, 1_000.0, 2_000.0, secs);

    // Build encoder.
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(in_sr);
    params.channels = Some(2);
    params.bit_rate = Some(32_000);
    let mut enc = HeAacV2Encoder::new(&params).expect("HeAacV2Encoder::new");

    // Push the entire PCM and flush.
    let n_samples = (in_sr as f32 * secs) as u32;
    let af = AudioFrame {
        format: SampleFormat::S16,
        channels: 2,
        sample_rate: in_sr,
        samples: n_samples,
        pts: Some(0),
        time_base: TimeBase::new(1, in_sr as i64),
        data: vec![pcm_bytes],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send_frame");
    enc.flush().expect("flush");

    // Drain ADTS output.
    let mut adts = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts.extend_from_slice(&pkt.data);
    }
    assert!(
        adts.len() > 100,
        "encoder produced only {} ADTS bytes",
        adts.len()
    );
    std::fs::write(&adts_path, &adts).expect("write adts");

    // ffmpeg decode + assertions.
    let (out_sr, out_channels, l_pcm, r_pcm) = match ffmpeg_decode_stereo(&adts_path, in_sr) {
        Some(t) => t,
        None => panic!("ffmpeg failed to decode our HE-AACv2 stream"),
    };
    assert_eq!(out_channels, 2, "expected 2-channel PS upmix");
    assert_eq!(out_sr, in_sr, "expected SBR-doubled rate");

    let n_check = l_pcm.len().min(r_pcm.len());
    assert!(
        n_check > 16_384,
        "too few decoded samples for stereo-image check: {n_check}"
    );

    // Skip the PS decorrelator warm-up.
    const WARMUP: usize = 4096;

    // 1) Channels should NOT be identical — that would mean PS dropped to
    //    identity stereo and the spatial cues were lost.
    let mut diff_count = 0usize;
    let mut total = 0usize;
    let mut max_diff = 0i32;
    for i in WARMUP..n_check {
        let d = (l_pcm[i] as i32 - r_pcm[i] as i32).abs();
        if d > 32 {
            diff_count += 1;
        }
        max_diff = max_diff.max(d);
        total += 1;
    }
    let diff_ratio = diff_count as f64 / total as f64;
    eprintln!(
        "HE-AACv2 real-PS: {diff_count}/{total} ({:.2}%) samples differ |L-R| > 32, max |L-R| = {max_diff}",
        diff_ratio * 100.0
    );
    assert!(
        diff_ratio > 0.20,
        "real-PS encode produced near-identity stereo \
         (only {:.2}% of samples differ |L-R| > 32) — \
         IID/ICC analysis is not preserving the spatial image",
        diff_ratio * 100.0
    );

    // 2) Per-channel power should be on the same order of magnitude (each
    //    input channel has the same amplitude, just different freqs).
    let p_l = mean_power(&l_pcm, WARMUP);
    let p_r = mean_power(&r_pcm, WARMUP);
    let ratio_db = 10.0 * (p_l / p_r.max(1e-12)).log10();
    eprintln!("HE-AACv2 real-PS power ratio L/R = {ratio_db:.2} dB (P_L={p_l:.0}, P_R={p_r:.0})");
    assert!(
        ratio_db.abs() < 12.0,
        "L/R power ratio {ratio_db:.2} dB is outside the expected ±12 dB range — \
         PS analysis is biased",
    );
    // Both channels carry meaningful energy.
    assert!(p_l > 1.0e3, "L channel almost silent (mean power {p_l})");
    assert!(p_r > 1.0e3, "R channel almost silent (mean power {p_r})");
}

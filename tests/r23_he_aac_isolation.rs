//! Round-23 isolation probe: HE-AAC stereo at 24 kHz core / 48 kHz output,
//! 1 kHz tone, amp 0.3. Compares the ffmpeg-decoded peak between:
//!
//! 1. Full HE-AAC (LC core + SBR FIL) — current production path.
//! 2. LC core only (the inner `AacEncoder` at the **core** rate, no FIL).
//!
//! If the saturation is in the LC core (r22 thesis), case 2 should also
//! produce ffmpeg peak ≈ 32768. If the saturation is in the SBR FIL path
//! (alternative thesis), case 2 should produce ffmpeg peak ≈ 9830.
//!
//! Skips cleanly without ffmpeg on PATH.

use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_aac::encoder::AacEncoder;
use oxideav_aac::he_aac_encoder::{HeAacMonoEncoder, HeAacStereoEncoder};
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame};

const HIGH_RATE: u32 = 48_000;
const CORE_RATE: u32 = 24_000;
const SECS: f32 = 0.5;
const AMP: f32 = 0.3;

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

fn make_stereo_pcm(rate: u32) -> Vec<u8> {
    let total = (rate as f32 * SECS) as usize;
    let mut bytes = Vec::with_capacity(total * 4);
    for i in 0..total {
        let t = i as f32 / rate as f32;
        let l = (2.0 * std::f32::consts::PI * 1000.0 * t).sin() * AMP;
        let r = (2.0 * std::f32::consts::PI * 2000.0 * t).sin() * AMP;
        bytes.extend_from_slice(&((l * 32767.0) as i16).to_le_bytes());
        bytes.extend_from_slice(&((r * 32767.0) as i16).to_le_bytes());
    }
    bytes
}

fn ffmpeg_decode_to(aac_path: &Path, pcm_path: &Path, rate: u32, channels: u16) {
    Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .arg("-i")
        .arg(aac_path)
        .args(["-f", "s16le", "-ar", &rate.to_string()])
        .args(["-ac", &channels.to_string()])
        .arg(pcm_path)
        .status()
        .expect("ffmpeg decode");
}

fn s16_from_bytes(bytes: &[u8]) -> Vec<i16> {
    bytes
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect()
}

fn rms(s: &[i16]) -> f64 {
    if s.is_empty() {
        return 0.0;
    }
    (s.iter().map(|&v| (v as f64).powi(2)).sum::<f64>() / s.len() as f64).sqrt()
}

fn peak(s: &[i16]) -> i32 {
    s.iter()
        .map(|&v| v.unsigned_abs() as i32)
        .max()
        .unwrap_or(0)
}

fn split_stereo(s: &[i16]) -> (Vec<i16>, Vec<i16>) {
    let mut l = Vec::new();
    let mut r = Vec::new();
    for ch in s.chunks_exact(2) {
        l.push(ch[0]);
        r.push(ch[1]);
    }
    (l, r)
}

#[test]
fn isolate_he_aac_saturation() {
    if which("ffmpeg").is_none() {
        eprintln!("ffmpeg not on PATH — skipping");
        return;
    }
    let scratch = std::env::temp_dir().join("oxideav_aac_r23_iso");
    std::fs::create_dir_all(&scratch).unwrap();

    // ----- Case 1: full HE-AAC stereo encode -----
    let he_path = scratch.join("he.aac");
    let he_pcm = scratch.join("he.s16");
    {
        let mut params = CodecParameters::audio(CodecId::new("aac"));
        params.sample_rate = Some(HIGH_RATE);
        params.channels = Some(2);
        params.bit_rate = Some(64_000);
        let mut enc = HeAacStereoEncoder::new(&params).expect("ctor");
        let pcm = make_stereo_pcm(HIGH_RATE);
        let total = (HIGH_RATE as f32 * SECS) as usize;
        let af = AudioFrame {
            samples: total as u32,
            pts: Some(0),
            data: vec![pcm],
        };
        enc.send_frame(&Frame::Audio(af)).expect("send_frame");
        enc.flush().expect("flush");
        let mut adts = Vec::new();
        while let Ok(pkt) = enc.receive_packet() {
            adts.extend_from_slice(&pkt.data);
        }
        std::fs::write(&he_path, &adts).expect("write");
    }
    ffmpeg_decode_to(&he_path, &he_pcm, HIGH_RATE, 2);

    // ----- Case 2: pure AAC-LC stereo at core rate, no SBR FIL -----
    let lc_path = scratch.join("lc.aac");
    let lc_pcm = scratch.join("lc.s16");
    {
        let mut params = CodecParameters::audio(CodecId::new("aac"));
        params.sample_rate = Some(CORE_RATE);
        params.channels = Some(2);
        params.bit_rate = Some(64_000);
        let mut enc = AacEncoder::new(&params).expect("ctor");
        // Use the same downsampled-rate input source as the inner LC sees.
        // For comparison, just use a 24 kHz tone directly at amp 0.3.
        let pcm = make_stereo_pcm(CORE_RATE);
        let total = (CORE_RATE as f32 * SECS) as usize;
        let af = AudioFrame {
            samples: total as u32,
            pts: Some(0),
            data: vec![pcm],
        };
        enc.send_frame(&Frame::Audio(af)).expect("send_frame");
        enc.flush().expect("flush");
        let mut adts = Vec::new();
        while let Ok(pkt) = enc.receive_packet() {
            adts.extend_from_slice(&pkt.data);
        }
        std::fs::write(&lc_path, &adts).expect("write");
    }
    ffmpeg_decode_to(&lc_path, &lc_pcm, CORE_RATE, 2);

    let warm = 2048usize;
    let trim = |s: Vec<i16>| {
        let mid_end = s.len().saturating_sub(2048).max(warm + 1);
        s[warm..mid_end.min(s.len())].to_vec()
    };
    let he = s16_from_bytes(&std::fs::read(&he_pcm).unwrap());
    let (he_l, he_r) = split_stereo(&he);
    let he_l = trim(he_l);
    let he_r = trim(he_r);
    let lc = s16_from_bytes(&std::fs::read(&lc_pcm).unwrap());
    let (lc_l, lc_r) = split_stereo(&lc);
    let lc_l = trim(lc_l);
    let lc_r = trim(lc_r);
    println!(
        "Case 1: full HE-AAC (24k core / 48k out, stereo, 1k+2k) — input peak ≈9830, RMS ≈6951"
    );
    println!(
        "  L: peak {:>5}, rms {:.0}     R: peak {:>5}, rms {:.0}",
        peak(&he_l),
        rms(&he_l),
        peak(&he_r),
        rms(&he_r)
    );
    println!("Case 2: pure AAC-LC stereo at 24 kHz, no FIL — input peak ≈9830, RMS ≈6951");
    println!(
        "  L: peak {:>5}, rms {:.0}     R: peak {:>5}, rms {:.0}",
        peak(&lc_l),
        rms(&lc_l),
        peak(&lc_r),
        rms(&lc_r)
    );

    // ----- Case 3: HE-AAC mono at 48k high / 24k core, 1 kHz -----
    let mono_path = scratch.join("he_mono.aac");
    let mono_pcm = scratch.join("he_mono.s16");
    {
        let mut params = CodecParameters::audio(CodecId::new("aac"));
        params.sample_rate = Some(HIGH_RATE);
        params.channels = Some(1);
        params.bit_rate = Some(48_000);
        let mut enc = HeAacMonoEncoder::new(&params).expect("ctor");
        let total = (HIGH_RATE as f32 * SECS) as usize;
        let mut bytes = Vec::with_capacity(total * 2);
        for i in 0..total {
            let t = i as f32 / HIGH_RATE as f32;
            let v = (2.0 * std::f32::consts::PI * 1000.0 * t).sin() * AMP;
            bytes.extend_from_slice(&((v * 32767.0) as i16).to_le_bytes());
        }
        let af = AudioFrame {
            samples: total as u32,
            pts: Some(0),
            data: vec![bytes],
        };
        enc.send_frame(&Frame::Audio(af)).expect("send_frame");
        enc.flush().expect("flush");
        let mut adts = Vec::new();
        while let Ok(pkt) = enc.receive_packet() {
            adts.extend_from_slice(&pkt.data);
        }
        std::fs::write(&mono_path, &adts).expect("write");
    }
    ffmpeg_decode_to(&mono_path, &mono_pcm, HIGH_RATE, 1);
    let mono = s16_from_bytes(&std::fs::read(&mono_pcm).unwrap());
    let mono = trim(mono);
    println!("Case 3: HE-AAC mono (24k core / 48k out, 1 kHz) — input peak ≈9830, RMS ≈6951");
    println!("  M: peak {:>5}, rms {:.0}", peak(&mono), rms(&mono));
}

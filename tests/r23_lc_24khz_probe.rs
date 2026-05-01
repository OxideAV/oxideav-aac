//! Round-23 scratch probe: pure AAC-LC at 24 kHz / 1 kHz, ffmpeg interop.
//!
//! Verifies the round-22 claim that even pure LC encoding at 24 kHz core
//! produces ffmpeg-decoded saturation. Runs the same code path as
//! `lc_rms_interop_r19.rs` but at 24 kHz with a 1 kHz tone.
//!
//! Skips cleanly without ffmpeg on PATH.

use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_aac::adts::parse_adts_header;
use oxideav_aac::encoder::AacEncoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame, Packet, TimeBase};

const SR: u32 = 24_000;
const SECS: f32 = 1.0;
const AMP: f32 = 0.3;
const FREQ: f32 = 1000.0;

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

fn make_pcm() -> Vec<u8> {
    let total = (SR as f32 * SECS) as usize;
    let mut bytes = Vec::with_capacity(total * 2);
    for i in 0..total {
        let t = i as f32 / SR as f32;
        let v = (2.0 * std::f32::consts::PI * FREQ * t).sin() * AMP;
        bytes.extend_from_slice(&((v * 32767.0) as i16).to_le_bytes());
    }
    bytes
}

fn ours_encode_to(path: &Path) {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(SR);
    params.channels = Some(1);
    params.bit_rate = Some(64_000);
    let mut enc = AacEncoder::new(&params).expect("ctor");
    let total = (SR as f32 * SECS) as usize;
    let af = AudioFrame {
        samples: total as u32,
        pts: Some(0),
        data: vec![make_pcm()],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send_frame");
    enc.flush().expect("flush");
    let mut adts = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts.extend_from_slice(&pkt.data);
    }
    std::fs::write(path, &adts).expect("write");
}

fn ffmpeg_decode_to(aac_path: &Path, pcm_path: &Path) {
    Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .arg("-i")
        .arg(aac_path)
        .args(["-f", "s16le", "-ar", &SR.to_string(), "-ac", "1"])
        .arg(pcm_path)
        .status()
        .expect("ffmpeg decode");
}

fn ours_decode(aac_bytes: &[u8]) -> Vec<i16> {
    let mut frames = Vec::new();
    let mut off = 0usize;
    while off + 7 < aac_bytes.len() {
        if aac_bytes[off] == 0xff && (aac_bytes[off + 1] & 0xf0) == 0xf0 {
            if let Ok(h) = parse_adts_header(&aac_bytes[off..]) {
                let len = h.header_length() + h.payload_length();
                if len > 0 && off + len <= aac_bytes.len() {
                    frames.push((off, len));
                    off += len;
                    continue;
                }
            }
        }
        off += 1;
    }
    if frames.is_empty() {
        return Vec::new();
    }
    let h = parse_adts_header(&aac_bytes[frames[0].0..]).unwrap();
    let mut dp = CodecParameters::audio(CodecId::new("aac"));
    dp.sample_rate = h.sample_rate();
    dp.channels = Some(h.channel_configuration as u16);
    let mut dec = oxideav_aac::decoder::make_decoder(&dp).expect("dec");
    let tb = TimeBase::new(1, SR as i64);
    let mut samples = Vec::<i16>::new();
    for (i, &(off, len)) in frames.iter().enumerate() {
        let pkt = Packet::new(0, tb, aac_bytes[off..off + len].to_vec()).with_pts(i as i64 * 1024);
        if dec.send_packet(&pkt).is_err() {
            continue;
        }
        if let Ok(Frame::Audio(af)) = dec.receive_frame() {
            for c in af.data[0].chunks_exact(2) {
                samples.push(i16::from_le_bytes([c[0], c[1]]));
            }
        }
    }
    samples
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

#[test]
fn lc_24khz_1khz_probe() {
    if which("ffmpeg").is_none() {
        eprintln!("ffmpeg not on PATH — skipping");
        return;
    }
    let scratch = std::env::temp_dir().join("oxideav_aac_r23_lc24");
    std::fs::create_dir_all(&scratch).unwrap();
    let ours_aac = scratch.join("ours.aac");
    let ours_via_ffmpeg_pcm = scratch.join("ours_via_ffmpeg.s16");
    ours_encode_to(&ours_aac);
    let ours_self = ours_decode(&std::fs::read(&ours_aac).unwrap());
    ffmpeg_decode_to(&ours_aac, &ours_via_ffmpeg_pcm);
    let ours_via_ffmpeg = s16_from_bytes(&std::fs::read(&ours_via_ffmpeg_pcm).unwrap());
    let expected_rms = (AMP as f64 * 32767.0) / 2.0f64.sqrt();
    let warm = 4096usize;
    let trim = |s: &[i16]| {
        let mid_end = (s.len() / 2 - warm).max(warm + 1);
        s[warm..mid_end.min(s.len())].to_vec()
    };
    let s_self = trim(&ours_self);
    let s_ff = trim(&ours_via_ffmpeg);
    println!(
        "Pure LC 24 kHz / 1 kHz tone (expected peak ~9830, RMS ~{:.0}):",
        expected_rms
    );
    println!(
        "  ours-encode → ours-decode  : peak {:>5}, rms {:.0}",
        peak(&s_self),
        rms(&s_self)
    );
    println!(
        "  ours-encode → ffmpeg-decode: peak {:>5}, rms {:.0}",
        peak(&s_ff),
        rms(&s_ff)
    );
}

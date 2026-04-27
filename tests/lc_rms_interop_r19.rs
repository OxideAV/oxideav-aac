//! Round-19 AAC-LC amplitude interop regression — RMS-based, not peak.
//!
//! Background: rounds 17-18 chased a phantom "mid-stream amplitude ~0.6×
//! expected" gap based on the *peak* of the decoded sine. Round 19 found
//! that the gap is entirely a PNS-noise artefact:
//!
//! 1. ffmpeg's AAC encoder routinely fills the high-frequency tail of a
//!    sine input with PNS-coded bands (codebook 13, §4.6.13). Those bands
//!    encode an envelope only, with the per-line magnitude resynthesised
//!    by the *decoder* from a uniform random source.
//! 2. Our decoder is spec-compliant on PNS (`apply_pns_long`). When fed
//!    ffmpeg's bitstream it correctly synthesises the noise floor, which
//!    rides on the sine peak and inflates the **peak** value.
//! 3. Because PNS is non-deterministic per-frame, peak ratio is not a
//!    meaningful interop metric for tonal+noise content. **RMS** is.
//!
//! This test pins the RMS interop within ±10% of unity for both
//! directions:
//!
//! - ours-encode → ffmpeg-decode → RMS ratio
//! - ffmpeg-encode → ours-decode → RMS ratio
//!
//! and the two self-roundtrips. Skips cleanly without ffmpeg on PATH.
//!
//! References used: ISO/IEC 14496-3 §4.6.1.3 (inverse quant
//! `sign(q)·|q|^(4/3)`), §4.6.2.3.3 (`gain = 2^(0.25·(sf-100))`),
//! §4.6.11.3.1 (IMDCT scale `2/N`), §4.6.13 (PNS).

use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_aac::adts::parse_adts_header;
use oxideav_aac::encoder::AacEncoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame, Packet, TimeBase};

const SR: u32 = 44_100;
const SECS: f32 = 1.0;
const AMP: f32 = 0.3;
const FREQ: f32 = 440.0;

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
    params.bit_rate = Some(128_000);
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

fn ffmpeg_encode_to(pcm_path: &Path, aac_path: &Path) {
    Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .args(["-f", "s16le", "-ar", "44100", "-ac", "1"])
        .arg("-i")
        .arg(pcm_path)
        .args(["-c:a", "aac", "-b:a", "128k"])
        .arg(aac_path)
        .status()
        .expect("ffmpeg encode");
}

fn ffmpeg_decode_to(aac_path: &Path, pcm_path: &Path) {
    Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .arg("-i")
        .arg(aac_path)
        .args(["-f", "s16le", "-ar", "44100", "-ac", "1"])
        .arg(pcm_path)
        .status()
        .expect("ffmpeg decode");
}

fn ours_decode(aac_bytes: &[u8]) -> Vec<i16> {
    // Iterate ADTS frames.
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

fn assert_rms_within(name: &str, samples: &[i16], expected_rms: f64, tol: f64) {
    let warm = 4096usize;
    let mid_end = (samples.len() / 2 - warm).max(warm + 1);
    let mid = &samples[warm..mid_end];
    let r = rms(mid);
    let ratio = r / expected_rms;
    println!("  {name}: rms={r:.0} (expected {expected_rms:.0}, ratio {ratio:.3})");
    assert!(
        (ratio - 1.0).abs() < tol,
        "{name}: rms ratio {ratio:.3} out of tolerance ±{tol:.3}"
    );
}

#[test]
fn lc_rms_interop_within_10_percent() {
    if which("ffmpeg").is_none() {
        eprintln!("ffmpeg not on PATH — skipping LC RMS interop test");
        return;
    }
    let scratch = std::env::temp_dir().join("oxideav_aac_lc_rms_r19");
    std::fs::create_dir_all(&scratch).unwrap();
    let pcm_in = scratch.join("sine_in.s16");
    let ours_aac = scratch.join("ours.aac");
    let ffmpeg_aac = scratch.join("ffmpeg.aac");
    let ffmpeg_self_pcm = scratch.join("ffmpeg_self.s16");
    let ours_via_ffmpeg_pcm = scratch.join("ours_via_ffmpeg.s16");
    std::fs::write(&pcm_in, make_pcm()).unwrap();

    // Encode both ways.
    ours_encode_to(&ours_aac);
    ffmpeg_encode_to(&pcm_in, &ffmpeg_aac);

    // Decode both ways.
    let ours_self = ours_decode(&std::fs::read(&ours_aac).unwrap());
    let ffmpeg_via_ours = ours_decode(&std::fs::read(&ffmpeg_aac).unwrap());
    ffmpeg_decode_to(&ours_aac, &ours_via_ffmpeg_pcm);
    ffmpeg_decode_to(&ffmpeg_aac, &ffmpeg_self_pcm);
    let ours_via_ffmpeg = s16_from_bytes(&std::fs::read(&ours_via_ffmpeg_pcm).unwrap());
    let ffmpeg_self = s16_from_bytes(&std::fs::read(&ffmpeg_self_pcm).unwrap());

    let expected_rms = (AMP as f64 * 32767.0) / 2.0f64.sqrt();
    println!("Round-19 AAC-LC RMS interop (sine 440 Hz, amp 0.3, expected RMS {expected_rms:.0}):");
    let tol = 0.10; // ±10%
    assert_rms_within("ours-encode → ours-decode", &ours_self, expected_rms, tol);
    assert_rms_within(
        "ours-encode → ffmpeg-decode",
        &ours_via_ffmpeg,
        expected_rms,
        tol,
    );
    assert_rms_within(
        "ffmpeg-encode → ours-decode",
        &ffmpeg_via_ours,
        expected_rms,
        tol,
    );
    assert_rms_within(
        "ffmpeg-encode → ffmpeg-decode (reference)",
        &ffmpeg_self,
        expected_rms,
        tol,
    );
}

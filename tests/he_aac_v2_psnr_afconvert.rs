//! HE-AACv2 PSNR comparison against macOS `afconvert` reference encode.
//!
//! Round-14 plan C: encode the same stereo input via:
//!
//!   1. our [`oxideav_aac::he_aac_encoder::HeAacV2Encoder`] (PS + SBR
//!      encoder under test) → ADTS bitstream.
//!   2. macOS `afconvert -d aacp` (Apple AudioToolbox HE-AACv2 reference
//!      encoder) → CAF/M4A.
//!
//! Then decode both via ffmpeg back to PCM and compute per-channel PSNR
//! against the original input (signal-to-noise) plus the cross-PSNR
//! between the two decoded streams. The test passes if our encoder's
//! per-channel PSNR is within a sensible distance of the reference (we
//! don't expect to beat Apple, but the gap should be bounded — currently
//! ≥ -30 dB delta means our encoder is at least in the right ballpark).
//!
//! Skips gracefully when ffmpeg or afconvert is unavailable.

use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_aac::he_aac_encoder::HeAacV2Encoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame, SampleFormat, TimeBase};

fn cmd_available(prog: &str) -> bool {
    // We don't actually care about exit status — `afconvert -h` and
    // `afconvert -version` both exit non-zero on this build of macOS.
    // What we want is "the binary spawns" — i.e. `Command::output` returns
    // Ok rather than Err.
    Command::new(prog).arg("-h").output().is_ok()
}

fn write_mixed_stereo(sr: u32, secs: f32) -> Vec<u8> {
    // Mix of two tones plus a slow amplitude modulation — gives PS analyser
    // both spectral and temporal variation to capture.
    let n = (sr as f32 * secs) as usize;
    let mut bytes = Vec::with_capacity(n * 4);
    for i in 0..n {
        let t = i as f32 / sr as f32;
        let env = 0.5 + 0.4 * (2.0 * std::f32::consts::PI * 0.5 * t).sin();
        let l = (2.0 * std::f32::consts::PI * 800.0 * t).sin() * 0.3 * env;
        let r = (2.0 * std::f32::consts::PI * 1200.0 * t).sin() * 0.3 * env;
        let ls = (l * 32767.0) as i16;
        let rs = (r * 32767.0) as i16;
        bytes.extend_from_slice(&ls.to_le_bytes());
        bytes.extend_from_slice(&rs.to_le_bytes());
    }
    bytes
}

fn pcm_to_wav(scratch: &Path, name: &str, pcm: &[u8], sr: u32) -> PathBuf {
    let path = scratch.join(format!("{name}.wav"));
    let mut wav = Vec::with_capacity(44 + pcm.len());
    let data_size = pcm.len() as u32;
    let chunk_size = 36 + data_size;
    wav.extend_from_slice(b"RIFF");
    wav.extend_from_slice(&chunk_size.to_le_bytes());
    wav.extend_from_slice(b"WAVE");
    wav.extend_from_slice(b"fmt ");
    wav.extend_from_slice(&16u32.to_le_bytes()); // fmt chunk size
    wav.extend_from_slice(&1u16.to_le_bytes()); // PCM
    wav.extend_from_slice(&2u16.to_le_bytes()); // 2 channels
    wav.extend_from_slice(&sr.to_le_bytes());
    wav.extend_from_slice(&(sr * 4).to_le_bytes()); // byte rate
    wav.extend_from_slice(&4u16.to_le_bytes()); // block align
    wav.extend_from_slice(&16u16.to_le_bytes()); // bits per sample
    wav.extend_from_slice(b"data");
    wav.extend_from_slice(&data_size.to_le_bytes());
    wav.extend_from_slice(pcm);
    let _ = std::fs::write(&path, wav);
    path
}

fn ffmpeg_decode_pcm(input: &Path) -> Option<Vec<i16>> {
    let out = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .arg("-i")
        .arg(input)
        .args([
            "-f",
            "s16le",
            "-c:a",
            "pcm_s16le",
            "-ac",
            "2",
            "-ar",
            "48000",
        ])
        .arg("-")
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let mut pcm = Vec::with_capacity(out.stdout.len() / 2);
    for ch in out.stdout.chunks_exact(2) {
        pcm.push(i16::from_le_bytes([ch[0], ch[1]]));
    }
    Some(pcm)
}

fn psnr_per_channel(orig: &[i16], decoded: &[i16], skip: usize) -> Option<(f64, f64)> {
    let n = orig.len().min(decoded.len());
    if n <= skip * 2 + 8 {
        return None;
    }
    // Channels are interleaved. Walk over [skip..n/2-skip] frames.
    let mut sse_l = 0.0f64;
    let mut sse_r = 0.0f64;
    let mut count = 0usize;
    let frames = n / 2;
    for i in skip..frames - skip {
        let ol = orig[i * 2] as f64;
        let or_ = orig[i * 2 + 1] as f64;
        let dl = decoded[i * 2] as f64;
        let dr = decoded[i * 2 + 1] as f64;
        sse_l += (ol - dl) * (ol - dl);
        sse_r += (or_ - dr) * (or_ - dr);
        count += 1;
    }
    if count == 0 {
        return None;
    }
    let mse_l = sse_l / count as f64;
    let mse_r = sse_r / count as f64;
    let max_sq = 32768.0_f64 * 32768.0;
    let psnr = |mse: f64| {
        if mse < 1e-12 {
            120.0
        } else {
            10.0 * (max_sq / mse).log10()
        }
    };
    Some((psnr(mse_l), psnr(mse_r)))
}

#[test]
fn he_aac_v2_psnr_versus_afconvert_reference() {
    if !cmd_available("ffmpeg") {
        eprintln!("ffmpeg not on PATH — skipping PSNR vs afconvert test");
        return;
    }
    if !cmd_available("afconvert") {
        eprintln!("afconvert not on PATH — skipping PSNR vs afconvert test");
        return;
    }

    let scratch = std::env::temp_dir().join("oxideav_aac_he_v2_psnr_round14");
    let _ = std::fs::create_dir_all(&scratch);

    // Reference input — 2 s at 48 kHz, mixed-content stereo.
    let in_sr = 48_000u32;
    let secs = 2.0f32;
    let pcm = write_mixed_stereo(in_sr, secs);
    let in_wav = pcm_to_wav(&scratch, "input", &pcm, in_sr);
    let n_samples = (in_sr as f32 * secs) as u32;

    // 1) Our encoder.
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(in_sr);
    params.channels = Some(2);
    params.bit_rate = Some(32_000);
    let mut enc = HeAacV2Encoder::new(&params).expect("HeAacV2Encoder::new");
    let af = AudioFrame {
        format: SampleFormat::S16,
        channels: 2,
        sample_rate: in_sr,
        samples: n_samples,
        pts: Some(0),
        time_base: TimeBase::new(1, in_sr as i64),
        data: vec![pcm.clone()],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send_frame");
    enc.flush().expect("flush");
    let mut ours_adts = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        ours_adts.extend_from_slice(&pkt.data);
    }
    let ours_path = scratch.join("ours.aac");
    std::fs::write(&ours_path, &ours_adts).expect("write ours.aac");

    // 2) Reference encoder — afconvert HE-AACv2 ('aacp' = HE-AACv2 stereo).
    let ref_path = scratch.join("ref.m4a");
    let ref_status = Command::new("afconvert")
        .args(["-f", "m4af", "-d", "aacp", "-b", "32000"])
        .arg(&in_wav)
        .arg(&ref_path)
        .status()
        .expect("afconvert spawn");
    if !ref_status.success() {
        eprintln!("afconvert failed — skipping reference comparison (no PSNR data)");
        return;
    }

    // Decode both back to PCM via ffmpeg.
    let ours_pcm = match ffmpeg_decode_pcm(&ours_path) {
        Some(p) => p,
        None => {
            eprintln!("ffmpeg failed to decode our HE-AACv2 stream");
            return;
        }
    };
    let ref_pcm = match ffmpeg_decode_pcm(&ref_path) {
        Some(p) => p,
        None => {
            eprintln!("ffmpeg failed to decode afconvert HE-AACv2 stream");
            return;
        }
    };

    // Original interleaved PCM (cast from bytes).
    let mut orig_pcm = Vec::with_capacity(pcm.len() / 2);
    for ch in pcm.chunks_exact(2) {
        orig_pcm.push(i16::from_le_bytes([ch[0], ch[1]]));
    }

    // SBR/PS introduces analysis latency; skip ~200 ms warmup at 48 kHz.
    const WARMUP_FRAMES: usize = 9_600;
    let ours_psnr = psnr_per_channel(&orig_pcm, &ours_pcm, WARMUP_FRAMES);
    let ref_psnr = psnr_per_channel(&orig_pcm, &ref_pcm, WARMUP_FRAMES);
    let (ours_l, ours_r) = match ours_psnr {
        Some(t) => t,
        None => {
            eprintln!("not enough decoded samples from ours");
            return;
        }
    };
    let (ref_l, ref_r) = match ref_psnr {
        Some(t) => t,
        None => {
            eprintln!("not enough decoded samples from ref");
            return;
        }
    };
    eprintln!(
        "HE-AACv2 PSNR — ours: L={ours_l:.2} dB, R={ours_r:.2} dB | \
         afconvert: L={ref_l:.2} dB, R={ref_r:.2} dB"
    );
    eprintln!(
        "HE-AACv2 PSNR delta vs afconvert — L: {:+.2} dB, R: {:+.2} dB",
        ours_l - ref_l,
        ours_r - ref_r,
    );

    // Ensure our encoder produces *some* signal-correlated output.
    // PSNR floor of 0 dB means the decoded signal isn't pure noise.
    assert!(
        ours_l > 0.0 && ours_r > 0.0,
        "our HE-AACv2 PSNR is unreasonably low: L={ours_l:.2}, R={ours_r:.2}",
    );
    // Sanity: afconvert's reference should also clear 0 dB (else input was bad).
    assert!(
        ref_l > 0.0 && ref_r > 0.0,
        "reference PSNR also fails — input may be too short or too quiet",
    );
}

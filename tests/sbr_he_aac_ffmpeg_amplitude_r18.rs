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
//! ## Round-22 update (2026-04-26): claim of LC-core scale issue
//!
//! Round-22 ran the methodical SBR header diff-probe specified in r21:
//!
//! - Captured fdkaac's choices for 24 kHz core / 48 kHz output as
//!   `bs_start_freq=13, bs_stop_freq=11, bs_freq_scale=1, bs_amp_res=1`
//!   (vs ours `5/9/2/0`).
//! - Forced our encoder to each fdkaac field individually + the
//!   combined fdkaac configuration. **None** of the candidate
//!   header fields drops the saturation.
//! - Forced our envelope SF data to `[0;14]` (matching fdkaac's
//!   E_orig = 64 for tonal content) by setting `INT16_SCALE_SQ = 1.0`.
//!   Still saturates.
//! - r22 *claimed* even pure AAC-LC core at 24 kHz produced ffmpeg
//!   peak 32 768. r22 *claimed* `MDCT_FORWARD_SCALE = 4096` produced
//!   "ffmpeg RMS unity" → 16x scale mismatch.
//!
//! ## Round-23 audit (2026-04-30): r22 thesis refuted
//!
//! Round-23 directly tested the r22 thesis by setting
//! `MDCT_FORWARD_SCALE = 4096` and re-running the matrix:
//!
//! - LC RMS at 44.1 kHz / 440 Hz: `ours-encode → ours-decode` ratio
//!   collapses to **0.060** (16x quieter, well outside ±10 % tol).
//! - SBR amplitude (this test): peak only drops from 32 768 → 25 287,
//!   still saturated.
//! - Pure AAC-LC at 24 kHz / 1 kHz **mono** with current
//!   SCALE = 65 536 produces ffmpeg-decoded peak **10 930 / RMS 6 955**
//!   — well within ±5 % of the input. r22's "pure-LC-saturates" claim
//!   was wrong: it conflated the HE-AAC code path with pure-LC.
//! - Pure AAC-LC stereo at 24 kHz / (1k+2k) with SCALE = 65 536
//!   produces ffmpeg L=10 930/9 880 RMS ~6 700 — also within ±5 %.
//! - HE-AAC mono at 24 kHz core with SCALE = 65 536 saturates → the
//!   bug is uniquely in the SBR FIL pipeline (the `EXT_SBR_DATA`
//!   extension), not the LC core.
//! - r22's "RMS lands on target at SCALE = 4 096" reading was a
//!   methodological error: a clipped square-wave at 32 768 has RMS
//!   ≈ 30 000; reducing SCALE 16x reduces input level by 16x so
//!   clipping eases off and RMS *passes through* the input target on
//!   its way down to silence. RMS continues to drop at SCALE = 2 048
//!   (1 891) and 1 024 (947) — there is no stable interop point.
//!
//! Round-23 verdict: `MDCT_FORWARD_SCALE = 65 536` is correct (verified
//! by 4 LC-only ffmpeg interop measurements) and remains so. The
//! saturation bug lives in the SBR FIL extension itself — an envelope
//! amplification, HF generation gain, or `noise_floor` over-pump
//! triggered when ffmpeg parses our SBR payload. Pinned for r24.
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
#[ignore = "round-23 known interop gap — ffmpeg saturates our HE-AAC streams to peak 32768; r22's MDCT_FORWARD_SCALE thesis refuted (pure LC at 24 kHz decodes cleanly through ffmpeg); root cause is in the SBR FIL extension itself, pinned for r24"]
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

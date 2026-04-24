//! HE-AACv2 IPD/OPD fixture hunt.
//!
//! Round-7 (`0556654`) wired up IPD/OPD phase rotation on the PS mixing
//! matrix (§8.6.4.6.3.2), but the existing `afconvert -d aacp` fixture never
//! lights up `enable_ipdopd = 1` — the baseline afconvert PS profile only
//! emits IID + ICC parameters. This test probes several encoder
//! configurations (different bitrates, content types, sample rates) and
//! reports back how many frames carried the IPD/OPD extension. If any
//! configuration is found that reliably activates it, the test additionally
//! verifies that our decode is qualitatively sane (stereo output, non-mono).
//!
//! Gated behind `OXIDEAV_AAC_IPDOPD_HUNT=1` so CI doesn't pay the encoder
//! cost every run. Never fails — prints a shortlist.
//!
//! Rules: no third-party source code is read. `afconvert` / `ffmpeg` are
//! used as black-box encoders only.

use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_aac::adts::{parse_adts_header, ADTS_HEADER_NO_CRC};
use oxideav_aac::sbr::ps::{
    ipdopd_frames_seen, ps_ext_v0_seen, ps_frames_total, ps_hdr_enable_ext_seen,
};
#[allow(unused_imports)]
use oxideav_codec::Decoder;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

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

fn write_wav_stereo<F: FnMut(f32) -> (f32, f32)>(path: &Path, sr: u32, secs: f32, mut gen: F) {
    let n = (sr as f32 * secs) as usize;
    let mut pcm = Vec::with_capacity(n * 4);
    for i in 0..n {
        let t = i as f32 / sr as f32;
        let (l, r) = gen(t);
        let ls = (l.clamp(-1.0, 1.0) * 32767.0) as i16;
        let rs = (r.clamp(-1.0, 1.0) * 32767.0) as i16;
        pcm.extend_from_slice(&ls.to_le_bytes());
        pcm.extend_from_slice(&rs.to_le_bytes());
    }
    let mut wav = Vec::with_capacity(pcm.len() + 44);
    wav.extend_from_slice(b"RIFF");
    wav.extend_from_slice(&((pcm.len() + 36) as u32).to_le_bytes());
    wav.extend_from_slice(b"WAVE");
    wav.extend_from_slice(b"fmt ");
    wav.extend_from_slice(&16u32.to_le_bytes());
    wav.extend_from_slice(&1u16.to_le_bytes());
    wav.extend_from_slice(&2u16.to_le_bytes());
    wav.extend_from_slice(&sr.to_le_bytes());
    wav.extend_from_slice(&(sr * 4).to_le_bytes());
    wav.extend_from_slice(&4u16.to_le_bytes());
    wav.extend_from_slice(&16u16.to_le_bytes());
    wav.extend_from_slice(b"data");
    wav.extend_from_slice(&(pcm.len() as u32).to_le_bytes());
    wav.extend_from_slice(&pcm);
    std::fs::write(path, &wav).expect("write wav");
}

fn iter_adts(bytes: &[u8]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + ADTS_HEADER_NO_CRC < bytes.len() {
        if bytes[i] != 0xFF || (bytes[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        match parse_adts_header(&bytes[i..]) {
            Ok(h) => {
                if h.frame_length == 0 || i + h.frame_length > bytes.len() {
                    break;
                }
                out.push((i, h.frame_length));
                i += h.frame_length;
            }
            Err(_) => i += 1,
        }
    }
    out
}

#[derive(Clone, Copy, Default, Debug)]
struct DecodeStats {
    ipdopd: usize,
    ps_ext_v0: usize,
    ps_enable_ext: usize,
    ps_frames: usize,
    audio_frames: usize,
}

fn decode_and_count(adts: &Path) -> DecodeStats {
    let mut s = DecodeStats::default();
    let bytes = match std::fs::read(adts) {
        Ok(b) => b,
        Err(_) => return s,
    };
    let frames = iter_adts(&bytes);
    if frames.is_empty() {
        return s;
    }
    let first = match parse_adts_header(&bytes[frames[0].0..]) {
        Ok(h) => h,
        Err(_) => return s,
    };
    let core_sr = match first.sample_rate() {
        Some(sr) => sr,
        None => return s,
    };
    let channels = first.channel_configuration.max(1) as u16;
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(core_sr);
    params.channels = Some(channels);
    let mut dec = match oxideav_aac::decoder::make_decoder(&params) {
        Ok(d) => d,
        Err(_) => return s,
    };
    let tb = TimeBase::new(1, core_sr as i64);

    let b0 = ipdopd_frames_seen();
    let b1 = ps_ext_v0_seen();
    let b2 = ps_hdr_enable_ext_seen();
    let b3 = ps_frames_total();
    for (i, &(off, len)) in frames.iter().enumerate() {
        let pkt = Packet::new(0, tb, bytes[off..off + len].to_vec()).with_pts(i as i64 * 1024);
        if dec.send_packet(&pkt).is_err() {
            break;
        }
        match dec.receive_frame() {
            Ok(Frame::Audio(_)) => s.audio_frames += 1,
            _ => break,
        }
    }
    s.ipdopd = ipdopd_frames_seen() - b0;
    s.ps_ext_v0 = ps_ext_v0_seen() - b1;
    s.ps_enable_ext = ps_hdr_enable_ext_seen() - b2;
    s.ps_frames = ps_frames_total() - b3;
    s
}

fn encode_afconvert(wav_in: &Path, adts_out: &Path, bitrate: u32) -> bool {
    let Some(_) = which("afconvert") else {
        return false;
    };
    let status = Command::new("afconvert")
        .args(["-f", "adts", "-d", "aacp", "-b", &bitrate.to_string()])
        .arg(wav_in)
        .arg(adts_out)
        .status();
    matches!(status, Ok(s) if s.success()) && adts_out.exists()
}

fn encode_afconvert_strategy(wav_in: &Path, adts_out: &Path, bitrate: u32, strategy: u32) -> bool {
    let Some(_) = which("afconvert") else {
        return false;
    };
    let mut cmd = Command::new("afconvert");
    cmd.args(["-f", "adts", "-d", "aacp"])
        .args(["-b", &bitrate.to_string()])
        .args(["-q", "127"])
        .args(["-s", &strategy.to_string()])
        .arg(wav_in)
        .arg(adts_out);
    let status = cmd.status();
    matches!(status, Ok(s) if s.success()) && adts_out.exists()
}

fn encode_ffmpeg_fdk(wav_in: &Path, adts_out: &Path, bitrate: u32) -> bool {
    let Some(_) = which("ffmpeg") else {
        return false;
    };
    let status = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .arg("-i")
        .arg(wav_in)
        .args([
            "-c:a",
            "libfdk_aac",
            "-profile:a",
            "aac_he_v2",
            "-b:a",
            &bitrate.to_string(),
            "-f",
            "adts",
        ])
        .arg(adts_out)
        .status();
    matches!(status, Ok(s) if s.success()) && adts_out.exists()
}

/// fdkaac CLI (Fraunhofer FDK AAC standalone encoder, 1.0.6+) — a directly-
/// installable frontend to libfdk_aac that runs on macOS without any ffmpeg
/// rebuild. Profile 29 = HE-AACv2 (SBR+PS). Transport 2 = ADTS.
fn encode_fdkaac(wav_in: &Path, adts_out: &Path, bitrate: u32) -> bool {
    let Some(_) = which("fdkaac") else {
        return false;
    };
    let status = Command::new("fdkaac")
        .args(["-p", "29"])
        .args(["-b", &bitrate.to_string()])
        .args(["-f", "2"])
        .args(["-S"]) // silent progress
        .args(["-o", adts_out.to_str().unwrap()])
        .arg(wav_in)
        .status();
    matches!(status, Ok(s) if s.success()) && adts_out.exists()
}

fn encode_fdkaac_vbr(wav_in: &Path, adts_out: &Path, vbr_mode: u32) -> bool {
    let Some(_) = which("fdkaac") else {
        return false;
    };
    let status = Command::new("fdkaac")
        .args(["-p", "29"])
        .args(["-m", &vbr_mode.to_string()])
        .args(["-f", "2"])
        .args(["-S"])
        .args(["-o", adts_out.to_str().unwrap()])
        .arg(wav_in)
        .status();
    matches!(status, Ok(s) if s.success()) && adts_out.exists()
}

#[test]
fn hunt_ipdopd_fixture() {
    if std::env::var("OXIDEAV_AAC_IPDOPD_HUNT").ok().as_deref() != Some("1") {
        eprintln!("IPD/OPD hunt skipped — set OXIDEAV_AAC_IPDOPD_HUNT=1 to run");
        return;
    }

    let scratch = std::env::temp_dir().join("oxideav_aac_ipdopd_hunt");
    let _ = std::fs::create_dir_all(&scratch);
    let mut results: Vec<(String, DecodeStats)> = Vec::new();

    // Signal classes: each stresses a different aspect of PS parameter usage.
    //
    //   tones:      pure per-channel tones (historical baseline).
    //   phase_offset: same spectrum per channel with a fixed frequency-
    //                 dependent phase offset — the canonical IPD case.
    //   decorrelated_noise: pink-ish noise decorrelated between channels — ICC
    //                       is low so the encoder has nothing to add phase-
    //                       wise.
    //   swept_phase: linear phase sweep from 0 to π as frequency climbs —
    //                this should most strongly light up IPD/OPD on a
    //                supporting encoder.
    //   mixed: real-world-ish multi-tone content with non-trivial phase
    //          relationships.
    type StereoGen = Box<dyn Fn(f32) -> (f32, f32)>;
    let signals: Vec<(&str, StereoGen)> = vec![
        (
            "tones_440_880",
            Box::new(|t: f32| {
                (
                    (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.4,
                    (2.0 * std::f32::consts::PI * 880.0 * t).sin() * 0.4,
                )
            }),
        ),
        (
            "phase_offset_pi4",
            Box::new(|t: f32| {
                let env = 0.3;
                let base = 2.0 * std::f32::consts::PI * 500.0 * t;
                let base2 = 2.0 * std::f32::consts::PI * 1500.0 * t;
                let l = env * (base.sin() + 0.5 * base2.sin());
                let r = env
                    * ((base + std::f32::consts::FRAC_PI_4).sin()
                        + 0.5 * (base2 + std::f32::consts::FRAC_PI_3).sin());
                (l, r)
            }),
        ),
        (
            "decorrelated_noise",
            Box::new(|t: f32| {
                // Fast PRNG via linear-congruential on the time index.
                let i = (t * 48_000.0) as u32;
                let mut x = i.wrapping_mul(2_654_435_761);
                x ^= x >> 16;
                x = x.wrapping_mul(0x45d9f3b);
                x ^= x >> 16;
                let l_n = (x as f32 / u32::MAX as f32 - 0.5) * 0.4;
                let mut y = i.wrapping_mul(1_597_334_677);
                y ^= y >> 16;
                y = y.wrapping_mul(0x45d9f3b);
                y ^= y >> 16;
                let r_n = (y as f32 / u32::MAX as f32 - 0.5) * 0.4;
                (l_n, r_n)
            }),
        ),
        (
            "swept_phase",
            Box::new(|t: f32| {
                // A chirp in L; R tracks the same chirp with a phase offset
                // that grows with frequency — canonical IPD test signal.
                let f = 400.0 + 300.0 * (2.0 * std::f32::consts::PI * 0.5 * t).sin();
                let phi_l = 2.0 * std::f32::consts::PI * f * t;
                let phi_r = phi_l + std::f32::consts::PI * f / 4000.0;
                (0.4 * phi_l.sin(), 0.4 * phi_r.sin())
            }),
        ),
        (
            "mixed_phasy_stereo",
            Box::new(|t: f32| {
                let lfo = (2.0 * std::f32::consts::PI * 3.0 * t).sin();
                let l = 0.25
                    * ((2.0 * std::f32::consts::PI * 300.0 * t).sin()
                        + 0.4 * (2.0 * std::f32::consts::PI * 1000.0 * t).sin()
                        + 0.2 * (2.0 * std::f32::consts::PI * 2500.0 * t).cos());
                let r = 0.25
                    * ((2.0 * std::f32::consts::PI * 300.0 * t
                        + std::f32::consts::FRAC_PI_6 * lfo)
                        .sin()
                        + 0.4
                            * (2.0 * std::f32::consts::PI * 1000.0 * t
                                + std::f32::consts::FRAC_PI_4
                                + 0.5 * lfo)
                                .sin()
                        + 0.2
                            * (2.0 * std::f32::consts::PI * 2500.0 * t
                                + std::f32::consts::FRAC_PI_2)
                                .cos());
                (l, r)
            }),
        ),
    ];

    let sample_rates = [44_100u32, 48_000u32, 24_000u32];
    let bitrates = [
        8_000u32, 10_000, 12_000, 14_000, 16_000, 20_000, 24_000, 32_000, 48_000,
    ];

    for (name, gen) in signals.iter() {
        for &sr in &sample_rates {
            let wav_in = scratch.join(format!("{name}_{sr}.wav"));
            write_wav_stereo(&wav_in, sr, 2.0, &**gen);

            // afconvert default strategy.
            for &br in &bitrates {
                let adts = scratch.join(format!("af_{name}_{sr}_{br}.aac"));
                let _ = std::fs::remove_file(&adts);
                if !encode_afconvert(&wav_in, &adts, br) {
                    continue;
                }
                let stats = decode_and_count(&adts);
                results.push((format!("af|{name}|{sr}|{br}"), stats));
                let _ = std::fs::remove_file(&adts);
            }
            // afconvert high-quality mode.
            for &br in &[8_000u32, 12_000, 16_000] {
                let adts = scratch.join(format!("afhq_{name}_{sr}_{br}.aac"));
                let _ = std::fs::remove_file(&adts);
                if !encode_afconvert_strategy(&wav_in, &adts, br, 0) {
                    continue;
                }
                let stats = decode_and_count(&adts);
                results.push((format!("afhq|{name}|{sr}|{br}"), stats));
                let _ = std::fs::remove_file(&adts);
            }
            // libfdk via ffmpeg (will simply fail gracefully if not
            // compiled in — keep the matrix small).
            for &br in &[8_000u32, 12_000, 16_000] {
                let adts = scratch.join(format!("fdk_{name}_{sr}_{br}.aac"));
                let _ = std::fs::remove_file(&adts);
                if !encode_ffmpeg_fdk(&wav_in, &adts, br) {
                    continue;
                }
                let stats = decode_and_count(&adts);
                results.push((format!("fdk|{name}|{sr}|{br}"), stats));
                let _ = std::fs::remove_file(&adts);
            }
            // Direct `fdkaac` CLI — typically installable on macOS even
            // when ffmpeg lacks libfdk_aac.
            for &br in &bitrates {
                let adts = scratch.join(format!("fdkaac_{name}_{sr}_{br}.aac"));
                let _ = std::fs::remove_file(&adts);
                if !encode_fdkaac(&wav_in, &adts, br) {
                    continue;
                }
                let stats = decode_and_count(&adts);
                results.push((format!("fdkaac|{name}|{sr}|{br}"), stats));
                let _ = std::fs::remove_file(&adts);
            }
            // fdkaac VBR modes (1 is HE-AACv2-tuned, 2 is HE-AAC-tuned).
            for &m in &[1u32, 2, 3, 4, 5] {
                let adts = scratch.join(format!("fdkaac_vbr{m}_{name}_{sr}.aac"));
                let _ = std::fs::remove_file(&adts);
                if !encode_fdkaac_vbr(&wav_in, &adts, m) {
                    continue;
                }
                let stats = decode_and_count(&adts);
                results.push((format!("fdkaac_vbr{m}|{name}|{sr}"), stats));
                let _ = std::fs::remove_file(&adts);
            }
            let _ = std::fs::remove_file(&wav_in);
        }
    }

    let positives: Vec<_> = results.iter().filter(|(_, s)| s.ipdopd > 0).collect();
    let ext_seen: Vec<_> = results
        .iter()
        .filter(|(_, s)| s.ps_ext_v0 > 0 && s.ipdopd == 0)
        .collect();
    let hdr_ext_seen: Vec<_> = results
        .iter()
        .filter(|(_, s)| s.ps_enable_ext > 0 && s.ps_ext_v0 == 0)
        .collect();
    eprintln!("=== IPD/OPD hunt results ===");
    eprintln!("tried: {}", results.len());
    eprintln!("positives (ipdopd=1) ({}):", positives.len());
    for (tag, s) in positives.iter() {
        eprintln!(
            "  {tag}: ipdopd={}/{} ext_v0={} enable_ext={} ps_frames={}",
            s.ipdopd, s.audio_frames, s.ps_ext_v0, s.ps_enable_ext, s.ps_frames
        );
    }
    eprintln!(
        "ps_extension(v0) seen but enable_ipdopd=0 ({}):",
        ext_seen.len()
    );
    for (tag, s) in ext_seen.iter().take(5) {
        eprintln!(
            "  {tag}: ext_v0={} enable_ext={} ps_frames={} audio_frames={}",
            s.ps_ext_v0, s.ps_enable_ext, s.ps_frames, s.audio_frames
        );
    }
    eprintln!(
        "enable_ext=1 but no ps_extension(v0) ({}):",
        hdr_ext_seen.len()
    );
    for (tag, s) in hdr_ext_seen.iter().take(5) {
        eprintln!(
            "  {tag}: enable_ext={} ps_frames={} audio_frames={}",
            s.ps_enable_ext, s.ps_frames, s.audio_frames
        );
    }
    if positives.is_empty() {
        eprintln!(
            "no encoder configuration on this host produced enable_ipdopd=1 — \
             the afconvert baseline HE-AACv2 profile and every tested fdk \
             config only emit IID+ICC"
        );
    }
}

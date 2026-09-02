//! Black-box validation of the HE-AAC v1 encoder against an external
//! reference decoder BINARY (`ffmpeg`, invoked as an opaque tool):
//! every emitted stream must decode without error, at the doubled
//! rate, and to PCM whose per-QMF-band energies agree with this
//! crate's own decoder — the two independent decoders hearing the
//! same thing is the cross-check.
//!
//! Skips (logged, success) when no `ffmpeg` binary is on PATH.

use std::fs;
use std::path::PathBuf;
use std::process::Command;

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::he_aac_encoder::{HeAacConfig, HeAacEncoder, HE_FRAME_LEN};
use oxideav_aac::sbr_qmf::EncoderAnalysisQmf;

fn ffmpeg() -> Option<&'static str> {
    ["ffmpeg"].into_iter().find(|cand| {
        Command::new(cand)
            .arg("-version")
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false)
    })
}

fn scratch_dir() -> PathBuf {
    let base = std::env::temp_dir().join("oxideav-aac-he-blackbox");
    fs::create_dir_all(&base).expect("scratch dir");
    base
}

/// Read a 16-bit WAV: (interleaved samples, channels, sample rate).
fn read_wav(path: &PathBuf) -> Option<(Vec<i16>, usize, u32)> {
    let d = fs::read(path).ok()?;
    let mut i = 12;
    let mut channels = 0usize;
    let mut rate = 0u32;
    while i + 8 <= d.len() {
        let cid = &d[i..i + 4];
        let sz = u32::from_le_bytes([d[i + 4], d[i + 5], d[i + 6], d[i + 7]]) as usize;
        let body = i + 8;
        if cid == b"fmt " && body + 8 <= d.len() {
            channels = usize::from(u16::from_le_bytes([d[body + 2], d[body + 3]]));
            rate = u32::from_le_bytes([d[body + 4], d[body + 5], d[body + 6], d[body + 7]]);
        }
        if cid == b"data" {
            let end = (body + sz).min(d.len());
            let pcm = d[body..end]
                .chunks_exact(2)
                .map(|c| i16::from_le_bytes([c[0], c[1]]))
                .collect();
            return Some((pcm, channels.max(1), rate));
        }
        i = body + sz + (sz & 1);
    }
    None
}

/// Long-term per-QMF-band energies of channel `c`.
fn band_energies(pcm: &[i16], channels: usize, c: usize) -> [f64; 64] {
    let mono: Vec<f64> = pcm
        .iter()
        .skip(c)
        .step_by(channels)
        .map(|&v| f64::from(v))
        .collect();
    let mut bank = EncoderAnalysisQmf::new();
    let mut e = [0.0f64; 64];
    for slot in mono.chunks_exact(64) {
        let cols = bank.push_slot(slot).unwrap();
        for (k, v) in cols.iter().enumerate() {
            e[k] += v.norm_sqr();
        }
    }
    e
}

fn synthetic(seconds: f64, fs: u32, channels: usize) -> Vec<i16> {
    let n = (seconds * f64::from(fs)) as usize;
    let mut seed = 0x2468_ace1u32;
    let mut out = Vec::with_capacity(n * channels);
    for i in 0..n {
        let t = i as f64 / f64::from(fs);
        let mut v = 0.0;
        for &(f, a) in &[
            (330.0, 5000.0),
            (1200.0, 3000.0),
            (3400.0, 2200.0),
            (6100.0, 1800.0),
            (8700.0, 1400.0),
            (11300.0, 1100.0),
            (14100.0, 900.0),
        ] {
            v += a * (2.0 * std::f64::consts::PI * f * t).sin();
        }
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        v += (f64::from(seed >> 8) / f64::from(1u32 << 24) - 0.5) * 2000.0;
        for c in 0..channels {
            let g = if c == 1 { 0.8 } else { 1.0 };
            out.push((v * g).clamp(-32768.0, 32767.0) as i16);
        }
    }
    out
}

#[test]
fn reference_binary_decodes_our_he_aac_like_our_decoder() {
    let Some(ff) = ffmpeg() else {
        eprintln!("skip: no ffmpeg binary on PATH");
        return;
    };
    let dir = scratch_dir();
    for (name, fs_out, channels, bitrate) in [
        ("mono44", 44_100u32, 1u8, 40_000u32),
        ("stereo48", 48_000, 2, 64_000),
    ] {
        let pcm = synthetic(1.5, fs_out, usize::from(channels));
        let mut enc = HeAacEncoder::new(HeAacConfig::new(fs_out, channels, bitrate)).unwrap();
        let stream = enc.encode_all(&pcm).unwrap();
        let aac = dir.join(format!("{name}.aac"));
        let wav = dir.join(format!("{name}.wav"));
        fs::write(&aac, &stream).unwrap();
        let _ = fs::remove_file(&wav);

        // Black-box decode: must succeed without error output.
        let out = Command::new(ff)
            .args(["-hide_banner", "-loglevel", "error", "-y", "-i"])
            .arg(&aac)
            .arg(&wav)
            .output()
            .expect("run reference decoder");
        assert!(
            out.status.success(),
            "reference decoder failed on {name}: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert!(
            stderr.trim().is_empty(),
            "reference decoder reported problems on {name}: {stderr}"
        );

        let (ref_pcm, ref_ch, ref_rate) = read_wav(&wav).expect("reference WAV");
        assert_eq!(ref_rate, fs_out, "{name}: SBR output rate");
        // A mono SBR stream may be decoded to two (identical)
        // channels by decoders that provision for late implicit PS.
        assert!(
            ref_ch == usize::from(channels) || (channels == 1 && ref_ch == 2),
            "{name}: {ref_ch} reference channels"
        );

        // Our decoder on the same stream.
        let mut dec = StreamDecoder::new();
        let frames = dec.decode_all(&stream).expect("own decode");
        let mut ours: Vec<i16> = Vec::new();
        for f in &frames {
            ours.extend_from_slice(&f.pcm);
        }
        // Duration: within one frame of ours.
        let n_ours = ours.len() / usize::from(channels);
        let n_ref = ref_pcm.len() / ref_ch;
        assert!(
            (n_ours as i64 - n_ref as i64).unsigned_abs() as usize <= 2 * HE_FRAME_LEN,
            "{name}: length {n_ref} vs ours {n_ours}"
        );

        // Per-band long-term energy agreement between the two
        // independent decoders (bands carrying real signal).
        let e_ref = band_energies(&ref_pcm, ref_ch, 0);
        let e_ours = band_energies(&ours, usize::from(channels), 0);
        let k_end = usize::min((enc.sbr().bands().k_x + enc.sbr().bands().m) as usize, 64);
        let mut worst = 0.0f64;
        let mut mean = 0.0f64;
        let mut count = 0usize;
        let floor = e_ours.iter().cloned().fold(0.0, f64::max) * 1e-5;
        for k in 1..k_end {
            if e_ours[k] < floor {
                continue;
            }
            let db = (10.0 * (e_ref[k] / e_ours[k]).log10()).abs();
            worst = worst.max(db);
            mean += db;
            count += 1;
        }
        mean /= count.max(1) as f64;
        eprintln!("{name}: {count} bands, mean |Δ| {mean:.2} dB, worst {worst:.2} dB");
        assert!(mean < 1.0, "{name}: mean band delta {mean} dB");
        assert!(worst < 3.0, "{name}: worst band delta {worst} dB");
    }
}

/// Two bursts per frame — a click train whose onsets fall inside one
/// SBR frame — drive the encoder's five-envelope VARVAR grids; the
/// reference decoder binary must accept every such frame without
/// diagnostics and agree with this crate's decoder on the per-band
/// energies (the grids are exercised in both directions of the
/// frame-class alphabet, so a border the decoders derived differently
/// would show up as a level disagreement).
#[test]
fn reference_binary_accepts_double_onset_varvar_frames() {
    let Some(ff) = ffmpeg() else {
        eprintln!("skip: no ffmpeg binary on PATH");
        return;
    };
    let dir = scratch_dir();
    let fs_out = 44_100u32;
    let n = (1.5 * f64::from(fs_out)) as usize;
    let mut seed = 0x1357_9bdfu32;
    let mut pcm = Vec::with_capacity(n);
    for i in 0..n {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let noise = f64::from(seed >> 8) / f64::from(1u32 << 24) - 0.5;
        let t = i as f64 / f64::from(fs_out);
        // A soft bed, plus 256-sample bursts of bright noise starting
        // at hop offsets 400 and 1300 (nominal slots ≈ 6 and 20 −
        // the second lands in the next frame's lead region on some
        // frames, so VARFIX / VARVAR leads are exercised too).
        let off = i % HE_FRAME_LEN;
        let burst = (400..656).contains(&off) || (1300..1556).contains(&off);
        let v = 300.0 * (2.0 * std::f64::consts::PI * 440.0 * t).sin()
            + if burst {
                noise * 24_000.0
            } else {
                noise * 60.0
            };
        pcm.push(v.clamp(-32768.0, 32767.0) as i16);
    }
    let mut enc = HeAacEncoder::new(HeAacConfig::new(fs_out, 1, 40_000)).unwrap();
    let mut stream = Vec::new();
    let mut classes = [0usize; 4];
    let mut five = 0usize;
    for chunk in pcm.chunks(HE_FRAME_LEN) {
        stream.extend_from_slice(&enc.encode_frame(chunk).unwrap());
        let g = &enc.last_sbr_frame().unwrap().element.channels[0].grid;
        classes[g.frame_class.to_bits() as usize] += 1;
        if g.num_env == 5 {
            five += 1;
        }
    }
    stream.extend_from_slice(&enc.finish().unwrap());
    eprintln!(
        "frame classes FIXFIX/FIXVAR/VARFIX/VARVAR = {classes:?}, five-envelope frames {five}"
    );
    assert!(classes[3] > 0, "no VARVAR frame elected");
    assert!(five > 0, "no five-envelope frame");

    let aac = dir.join("double_onset.aac");
    let wav = dir.join("double_onset.wav");
    fs::write(&aac, &stream).unwrap();
    let _ = fs::remove_file(&wav);
    let out = Command::new(ff)
        .args(["-hide_banner", "-loglevel", "error", "-y", "-i"])
        .arg(&aac)
        .arg(&wav)
        .output()
        .expect("run reference decoder");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.trim().is_empty(),
        "reference decoder diagnostics: {stderr}"
    );
    let (ref_pcm, ref_ch, ref_rate) = read_wav(&wav).expect("reference WAV");
    assert_eq!(ref_rate, fs_out);

    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).expect("own decode");
    let ours: Vec<i16> = frames.iter().flat_map(|f| f.pcm.iter().copied()).collect();
    let e_ref = band_energies(&ref_pcm, ref_ch, 0);
    let e_ours = band_energies(&ours, 1, 0);
    let k_end = usize::min((enc.sbr().bands().k_x + enc.sbr().bands().m) as usize, 64);
    let floor = e_ours.iter().cloned().fold(0.0, f64::max) * 1e-5;
    let (mut mean, mut worst, mut count) = (0.0f64, 0.0f64, 0usize);
    for k in 1..k_end {
        if e_ours[k] < floor {
            continue;
        }
        let db = (10.0 * (e_ref[k] / e_ours[k]).log10()).abs();
        worst = worst.max(db);
        mean += db;
        count += 1;
    }
    mean /= count.max(1) as f64;
    eprintln!("double onset: {count} bands, mean |Δ| {mean:.2} dB, worst {worst:.2} dB");
    assert!(mean < 1.0, "mean band delta {mean} dB");
    assert!(worst < 3.0, "worst band delta {worst} dB");
}

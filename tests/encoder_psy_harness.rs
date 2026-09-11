//! Equal-rate encoder harness against the black-box reference
//! encoders (`ffmpeg`'s native `aac` for AAC-LC and its AudioToolbox
//! `aac_at` HE-AAC profile, invoked as opaque tools; skip-if-absent):
//! the same input coded by this crate and by the reference at the
//! same target rate, every stream decoded by this crate's decoder
//! (the oracle — including the *reference encoder's* streams) and by
//! the reference decoder binary, the two decodes compared per QMF
//! band, and the coding noise of each stream measured per band
//! against the input after lag alignment:
//!
//! * `NSR_k = 10·log10(Σ|X_out − X_in|² / Σ|X_in|²)` per 64-band QMF
//!   band `k` (both channels), and
//! * `NMR_k = 10·log10(N_k / T_k)` under a simple spreading masker
//!   (`T_k = max_j S_j · 10^{−(1 + 1.2·|k − j|)}`: 10 dB below the
//!   band's own energy, 12 dB per band on either side) — a
//!   first-order stand-in for a masked-noise measure, comparable
//!   between encoders on the same input.
//!
//! The tables are the measurement; the assertions pin decoder parity
//! (both decoders agree on every stream), the rate discipline, and a
//! coarse regression guard on the noise-to-mask ratio.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};
use oxideav_aac::he_aac_encoder::{HeAacConfig, HeAacEncoder};
use oxideav_aac::sbr_qmf::EncoderAnalysisQmf;

fn ffmpeg() -> Option<&'static str> {
    Command::new("ffmpeg")
        .arg("-version")
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|_| "ffmpeg")
}

fn has_encoder(ff: &str, name: &str) -> bool {
    Command::new(ff)
        .args(["-hide_banner", "-encoders"])
        .output()
        .map(|o| {
            String::from_utf8_lossy(&o.stdout)
                .lines()
                .any(|l| l.split_whitespace().nth(1) == Some(name))
        })
        .unwrap_or(false)
}

fn scratch_dir() -> PathBuf {
    let d = std::env::temp_dir().join("oxideav-aac-psy-harness");
    fs::create_dir_all(&d).unwrap();
    d
}

fn read_wav(path: &Path) -> Option<(Vec<i16>, usize, u32)> {
    let d = fs::read(path).ok()?;
    let mut i = 12;
    let (mut channels, mut rate) = (0usize, 0u32);
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

/// Deterministic stereo test material on the ±32768 axis: a chord of
/// partials with a slow vibrato, a broadband noise bed, a repeating
/// click train (every 0.3 s), and a swept tone — panned so the pair
/// is neither identical nor independent.
fn mixed(seconds: f64, fs: u32) -> Vec<i16> {
    let n = (seconds * f64::from(fs)) as usize;
    let mut seed = 0x5eed_0a5cu32;
    let mut out = Vec::with_capacity(2 * n);
    let period = (0.3 * f64::from(fs)) as usize;
    for i in 0..n {
        let t = i as f64 / f64::from(fs);
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let noise = f64::from(seed >> 8) / f64::from(1u32 << 24) - 0.5;
        let vib = 1.0 + 0.004 * (2.0 * std::f64::consts::PI * 5.0 * t).sin();
        let mut tone = 0.0;
        for (h, a) in [
            (220.0, 4000.0),
            (330.0, 2800.0),
            (440.0, 2200.0),
            (660.0, 1500.0),
            (880.0, 1200.0),
            (1320.0, 900.0),
            (2640.0, 600.0),
            (5280.0, 300.0),
        ] {
            tone += a * (2.0 * std::f64::consts::PI * h * vib * t).sin();
        }
        let sweep = 1500.0 * (2.0 * std::f64::consts::PI * (300.0 * t + 2000.0 * t * t)).sin();
        let click = if i % period < 96 {
            12_000.0 * (1.0 - (i % period) as f64 / 96.0)
        } else {
            0.0
        };
        let bed = noise * 1200.0;
        let l = tone * 1.0 + sweep * 0.6 + click + bed;
        let r = tone * 0.7 + sweep * 1.0 + click * 0.5 + bed * 0.8;
        out.push(l.clamp(-32768.0, 32767.0) as i16);
        out.push(r.clamp(-32768.0, 32767.0) as i16);
    }
    out
}

/// Transient material: bursts of decaying noise every 1024 samples
/// with a soft tonal bed.
fn transients(seconds: f64, fs: u32) -> Vec<i16> {
    let n = (seconds * f64::from(fs)) as usize;
    let mut seed = 0x7a5e_1234u32;
    let mut out = Vec::with_capacity(2 * n);
    for i in 0..n {
        let t = i as f64 / f64::from(fs);
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let noise = (seed as i32) as f64 / 2_147_483_648.0;
        let phase = (i % 1024) as f64;
        let burst = 14_000.0 * (-phase / 150.0).exp() * noise;
        let bed = 1500.0 * (2.0 * std::f64::consts::PI * 523.0 * t).sin()
            + 800.0 * (2.0 * std::f64::consts::PI * 1568.0 * t).sin();
        out.push((burst + bed).clamp(-32768.0, 32767.0) as i16);
        out.push((burst * 0.6 + bed * 1.2).clamp(-32768.0, 32767.0) as i16);
    }
    out
}

fn mono_sum(pcm: &[i16]) -> Vec<f64> {
    pcm.chunks_exact(2)
        .map(|c| f64::from(c[0]) + f64::from(c[1]))
        .collect()
}

/// Lag of `out` behind `input` (samples per channel) by
/// cross-correlation of the mono sums over the first second.
fn best_lag(input: &[f64], out: &[f64], max_lag: usize) -> usize {
    let n = input.len().min(out.len()).min(48_000);
    let mut best = (0usize, f64::NEG_INFINITY);
    for lag in 0..max_lag.min(out.len().saturating_sub(n / 2)) {
        let mut acc = 0.0;
        for i in 0..n.saturating_sub(lag) {
            acc += input[i] * out[i + lag];
        }
        if acc > best.1 {
            best = (lag, acc);
        }
    }
    best.0
}

struct BandNoise {
    signal: [f64; 64],
    noise: [f64; 64],
}

/// Per-QMF-band signal and coding-noise energies of `out` against
/// `input` (interleaved stereo, `out` already lag-aligned).
fn band_noise(input: &[i16], out: &[i16]) -> BandNoise {
    let m = input.len().min(out.len()) / 2;
    let mut bn = BandNoise {
        signal: [0.0; 64],
        noise: [0.0; 64],
    };
    for c in 0..2 {
        let a: Vec<f64> = input
            .iter()
            .skip(c)
            .step_by(2)
            .take(m)
            .map(|&v| f64::from(v))
            .collect();
        let b: Vec<f64> = out
            .iter()
            .skip(c)
            .step_by(2)
            .take(m)
            .map(|&v| f64::from(v))
            .collect();
        let mut qa = EncoderAnalysisQmf::new();
        let mut qb = EncoderAnalysisQmf::new();
        for (sa, sb) in a.chunks_exact(64).zip(b.chunks_exact(64)) {
            let xa = qa.push_slot(sa).unwrap();
            let xb = qb.push_slot(sb).unwrap();
            for k in 0..64 {
                bn.signal[k] += xa[k].norm_sqr();
                bn.noise[k] += (xb[k] - xa[k]).norm_sqr();
            }
        }
    }
    bn
}

struct Summary {
    nsr_mean: f64,
    nmr_mean: f64,
    nmr_worst: f64,
    nmr_worst_band: usize,
    bands: usize,
    rows: Vec<(usize, f64, f64)>,
}

fn summarize(bn: &BandNoise) -> Summary {
    let peak = bn.signal.iter().cloned().fold(0.0, f64::max);
    let floor = peak * 1e-6;
    // Spreading masker.
    let thr: Vec<f64> = (0..64)
        .map(|k| {
            bn.signal
                .iter()
                .enumerate()
                .map(|(j, &sj)| sj * 10f64.powf(-(1.0 + 1.2 * (k as f64 - j as f64).abs())))
                .fold(0.0f64, f64::max)
        })
        .collect();
    let mut s = Summary {
        nsr_mean: 0.0,
        nmr_mean: 0.0,
        nmr_worst: f64::NEG_INFINITY,
        nmr_worst_band: 0,
        bands: 0,
        rows: Vec::new(),
    };
    for (k, (&sig, &noise)) in bn.signal.iter().zip(bn.noise.iter()).enumerate() {
        if sig < floor {
            continue;
        }
        let nsr = 10.0 * (noise / sig).log10();
        let nmr = 10.0 * (noise / thr[k].max(1e-30)).log10();
        s.nsr_mean += nsr;
        s.nmr_mean += nmr;
        if nmr > s.nmr_worst {
            s.nmr_worst = nmr;
            s.nmr_worst_band = k;
        }
        s.bands += 1;
        s.rows.push((k, nsr, nmr));
    }
    s.nsr_mean /= s.bands.max(1) as f64;
    s.nmr_mean /= s.bands.max(1) as f64;
    s
}

/// Decode an ADTS stream with the reference binary (no diagnostics
/// allowed) → interleaved PCM.
fn reference_decode(ff: &str, path: &Path) -> (Vec<i16>, usize, u32) {
    let wav = path.with_extension("wav");
    let _ = fs::remove_file(&wav);
    let out = Command::new(ff)
        .args(["-hide_banner", "-loglevel", "error", "-y", "-i"])
        .arg(path)
        .arg(&wav)
        .output()
        .expect("run reference decoder");
    assert!(
        out.status.success(),
        "reference decoder failed on {}: {}",
        path.display(),
        String::from_utf8_lossy(&out.stderr)
    );
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.trim().is_empty(),
        "reference decoder diagnostics on {}: {stderr}",
        path.display()
    );
    read_wav(&wav).expect("reference WAV")
}

/// Decode with this crate's decoder → interleaved PCM, channels, rate.
fn own_decode(stream: &[u8]) -> (Vec<i16>, usize, u32) {
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(stream).expect("own decode");
    let mut pcm = Vec::new();
    let (mut ch, mut rate) = (0usize, 0u32);
    for f in &frames {
        ch = f.channels;
        rate = f.sample_rate;
        pcm.extend_from_slice(&f.pcm);
    }
    (pcm, ch.max(1), rate)
}

/// Per-band long-term energy disagreement between two decodes of the
/// same stream (mean / worst dB over bands carrying signal).
fn decoder_parity(a: &[i16], b: &[i16]) -> (f64, f64) {
    let energies = |pcm: &[i16]| -> [f64; 64] {
        let mut e = [0.0f64; 64];
        for c in 0..2 {
            let mono: Vec<f64> = pcm
                .iter()
                .skip(c)
                .step_by(2)
                .map(|&v| f64::from(v))
                .collect();
            let mut q = EncoderAnalysisQmf::new();
            for slot in mono.chunks_exact(64) {
                for (k, v) in q.push_slot(slot).unwrap().iter().enumerate() {
                    e[k] += v.norm_sqr();
                }
            }
        }
        e
    };
    let ea = energies(a);
    let eb = energies(b);
    let floor = ea.iter().cloned().fold(0.0, f64::max) * 1e-5;
    let (mut mean, mut worst, mut n) = (0.0f64, 0.0f64, 0usize);
    for k in 1..64 {
        if ea[k] < floor {
            continue;
        }
        let d = (10.0 * (eb[k] / ea[k]).log10()).abs();
        mean += d;
        worst = worst.max(d);
        n += 1;
    }
    (mean / n.max(1) as f64, worst)
}

/// One encoder's result on one input.
struct Coded {
    label: String,
    stream: Vec<u8>,
}

/// Measure one coded stream: decoder parity (own vs reference
/// decode), lag, and the noise tables of the reference decode
/// against the input.
fn measure(ff: &str, dir: &Path, input: &[i16], fs: u32, coded: &Coded) -> Summary {
    let path = dir.join(format!("{}.aac", coded.label));
    fs::write(&path, &coded.stream).unwrap();
    let (ref_pcm, ref_ch, ref_rate) = reference_decode(ff, &path);
    let (own_pcm, own_ch, own_rate) = own_decode(&coded.stream);
    assert_eq!(
        (ref_ch, ref_rate),
        (2, fs),
        "{}: reference decode shape",
        coded.label
    );
    assert_eq!(
        (own_ch, own_rate),
        (2, fs),
        "{}: own decode shape",
        coded.label
    );
    let m = ref_pcm.len().min(own_pcm.len());
    let (p_mean, p_worst) = decoder_parity(&own_pcm[..m], &ref_pcm[..m]);
    let kbps =
        coded.stream.len() as f64 * 8.0 / (input.len() as f64 / 2.0 / f64::from(fs)) / 1000.0;
    let lag = best_lag(&mono_sum(input), &mono_sum(&ref_pcm), 8192);
    let aligned = &ref_pcm[2 * lag..];
    let s = summarize(&band_noise(input, aligned));
    eprintln!(
        "{}: {:.1} kbps, lag {lag}, decoder parity mean {p_mean:.2} dB worst {p_worst:.2} dB, {} bands: mean NSR {:.1} dB, mean NMR {:.1} dB, worst NMR {:.1} dB at band {}",
        coded.label, kbps, s.bands, s.nsr_mean, s.nmr_mean, s.nmr_worst, s.nmr_worst_band
    );
    assert!(
        p_mean < 1.0,
        "{}: decoders disagree by {p_mean} dB",
        coded.label
    );
    assert!(
        p_worst < 3.0,
        "{}: worst band disagreement {p_worst} dB",
        coded.label
    );
    s
}

fn reference_encode(
    ff: &str,
    dir: &Path,
    input: &[i16],
    fs: u32,
    args: &[&str],
    label: &str,
) -> Coded {
    let raw = dir.join(format!("{label}.raw"));
    let bytes: Vec<u8> = input.iter().flat_map(|v| v.to_le_bytes()).collect();
    fs::write(&raw, bytes).unwrap();
    let out_path = dir.join(format!("{label}.aac"));
    let _ = fs::remove_file(&out_path);
    let fs_s = fs.to_string();
    let mut cmd = Command::new(ff);
    cmd.args([
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "s16le",
        "-ar",
        &fs_s,
        "-ac",
        "2",
        "-i",
    ])
    .arg(&raw)
    .args(args)
    .args(["-f", "adts"])
    .arg(&out_path);
    let out = cmd.output().expect("run reference encoder");
    assert!(
        out.status.success(),
        "reference encoder failed ({label}): {}",
        String::from_utf8_lossy(&out.stderr)
    );
    Coded {
        label: label.to_string(),
        stream: fs::read(&out_path).unwrap(),
    }
}

fn print_rows(label: &str, ours: &Summary, theirs: &Summary) {
    eprintln!("{label}: band  NSR ours/ref   NMR ours/ref   (dB)");
    for ((k, nsr_o, nmr_o), (_, nsr_r, nmr_r)) in ours.rows.iter().zip(theirs.rows.iter()) {
        eprintln!("{label}:  {k:2}   {nsr_o:6.1} {nsr_r:6.1}   {nmr_o:6.1} {nmr_r:6.1}");
    }
}

/// AAC-LC stereo at 64 / 96 / 128 kbps on the mixed and transient
/// material against the reference encoder at the same target.
#[test]
fn lc_equal_rate_noise_tables() {
    let Some(ff) = ffmpeg() else {
        eprintln!("skip: no ffmpeg binary on PATH");
        return;
    };
    if !has_encoder(ff, "aac") {
        eprintln!("skip: reference binary has no aac encoder");
        return;
    }
    let dir = scratch_dir();
    let fs = 44_100u32;
    for (sig_name, input) in [
        ("mixed", mixed(2.0, fs)),
        ("transients", transients(2.0, fs)),
    ] {
        for kbps in [64u32, 96, 128] {
            let mut enc = StreamEncoder::new(EncoderConfig {
                sample_rate: fs,
                channels: 2,
                bitrate: kbps * 1000,
            })
            .unwrap();
            let ours = Coded {
                label: format!("lc_{sig_name}_{kbps}k_ours"),
                stream: enc.encode_all(&input).unwrap(),
            };
            let theirs = reference_encode(
                ff,
                &dir,
                &input,
                fs,
                &["-c:a", "aac", "-b:a", &format!("{kbps}k")],
                &format!("lc_{sig_name}_{kbps}k_ref"),
            );
            let so = measure(ff, &dir, &input, fs, &ours);
            let st = measure(ff, &dir, &input, fs, &theirs);
            print_rows(&format!("lc {sig_name} {kbps}k"), &so, &st);
            eprintln!(
                "lc {sig_name} {kbps}k: NMR delta ours − ref = {:+.1} dB mean, {:+.1} dB worst-band",
                so.nmr_mean - st.nmr_mean,
                so.nmr_worst - st.nmr_worst
            );
            // Rate discipline: at or under the target, not far under.
            let seconds = input.len() as f64 / 2.0 / f64::from(fs);
            let our_kbps = ours.stream.len() as f64 * 8.0 / seconds / 1000.0;
            assert!(
                our_kbps <= f64::from(kbps) * 1.15,
                "{our_kbps} kbps over target"
            );
            assert!(
                our_kbps >= f64::from(kbps) * 0.5,
                "{our_kbps} kbps far under target"
            );
            // Regression guard on the masked-noise measure.
            assert!(
                so.nmr_mean <= st.nmr_mean + 8.0,
                "{sig_name} {kbps}k: mean NMR {:.1} dB vs reference {:.1} dB",
                so.nmr_mean,
                st.nmr_mean
            );
        }
    }
}

/// HE-AAC v1 stereo at 32 / 48 kbps against the reference HE-AAC
/// encoder (AudioToolbox through the reference binary, profile 4).
#[test]
fn he_equal_rate_noise_tables() {
    let Some(ff) = ffmpeg() else {
        eprintln!("skip: no ffmpeg binary on PATH");
        return;
    };
    if !has_encoder(ff, "aac_at") {
        eprintln!("skip: reference binary has no aac_at encoder");
        return;
    }
    let dir = scratch_dir();
    let fs = 44_100u32;
    let input = mixed(2.0, fs);
    for kbps in [32u32, 48] {
        let mut enc = HeAacEncoder::new(HeAacConfig::new(fs, 2, kbps * 1000)).unwrap();
        let ours = Coded {
            label: format!("he_mixed_{kbps}k_ours"),
            stream: enc.encode_all(&input).unwrap(),
        };
        let theirs = reference_encode(
            ff,
            &dir,
            &input,
            fs,
            &[
                "-c:a",
                "aac_at",
                "-profile:a",
                "4",
                "-b:a",
                &format!("{kbps}k"),
            ],
            &format!("he_mixed_{kbps}k_ref"),
        );
        let so = measure(ff, &dir, &input, fs, &ours);
        let st = measure(ff, &dir, &input, fs, &theirs);
        print_rows(&format!("he mixed {kbps}k"), &so, &st);
        eprintln!(
            "he mixed {kbps}k: NMR delta ours − ref = {:+.1} dB mean, {:+.1} dB worst-band",
            so.nmr_mean - st.nmr_mean,
            so.nmr_worst - st.nmr_worst
        );
        let seconds = input.len() as f64 / 2.0 / f64::from(fs);
        let our_kbps = ours.stream.len() as f64 * 8.0 / seconds / 1000.0;
        assert!(
            our_kbps <= f64::from(kbps) * 1.3,
            "{our_kbps} kbps over target"
        );
        assert!(
            so.nmr_mean <= st.nmr_mean + 10.0,
            "{kbps}k: mean NMR {:.1} dB vs reference {:.1} dB",
            so.nmr_mean,
            st.nmr_mean
        );
    }
}

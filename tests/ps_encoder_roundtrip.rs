//! HE-AAC v2 (parametric stereo) encoder round trips through the
//! crate's own subpart-8 decoder — the oracle the encoder was built
//! against — and, when a reference decoder binary is on PATH, through
//! that black box too.
//!
//! Measurement: the source and the decoded stereo (lag-aligned by
//! cross-correlation of the channel sums) run through the encoder's
//! own analysis chain — 64-band QMF + the §8.6.4.3 hybrid bank +
//! the Annex 8.C.6.2 band excitations — and the per-frame, per-band
//! inter-channel level difference (IID, dB) and coherence (ICC) of
//! the two are compared over every band that carries energy.

use std::fs;
use std::path::PathBuf;
use std::process::Command;

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::he_aac_encoder::{HeAacConfig, HeAacEncoder, HE_FRAME_LEN};
use oxideav_aac::ps_analysis::{band_stats, hybrid_config_for, BandStats, PsAnalysis};
use oxideav_aac::ps_encoder::{PsBands, PsEncoderConfig};
use oxideav_aac::ps_hybrid::{LOOKAHEAD, NUM_QMF_SLOTS};
use oxideav_aac::sbr_qmf::{Complex, EncoderAnalysisQmf};

/// Band-wise stereo material: per QMF-band-aligned region a
/// multi-sine "noise" (dozens of random-phase sinusoids) with its own
/// pan and its own shared-versus-independent split, so every stereo
/// band carries a distinct level difference and coherence and the
/// decoder's de-correlator sees genuinely broadband content (a pure
/// tone through an all-pass is a phase-shifted copy, not a
/// de-correlated signal — the parametric model's own limit, not the
/// encoder's).
fn synthetic_stereo(seconds: f64, fs: u32) -> Vec<i16> {
    let n = (seconds * f64::from(fs)) as usize;
    let mut seed = 0x1357_9bdfu32;
    let mut rnd = move || {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        f64::from(seed >> 8) / f64::from(1u32 << 24)
    };
    // Regions in QMF-band units (fs/128 per band), amplitude, left
    // gain, right gain, shared fraction (ICC of the region).
    let regions: [(f64, f64, f64, f64, f64, f64); 14] = [
        (0.5, 1.0, 3000.0, 1.0, 0.4, 0.9),
        (1.0, 2.0, 2600.0, 0.5, 1.0, 0.8),
        (2.0, 3.0, 2200.0, 0.9, 0.9, 0.5),
        (3.0, 4.0, 2000.0, 0.2, 1.0, 0.95),
        (4.0, 5.0, 1800.0, 1.0, 0.15, 0.7),
        (5.0, 6.0, 1600.0, 0.7, 0.7, 0.2),
        (6.0, 8.0, 1400.0, 0.3, 1.0, 0.85),
        (8.0, 10.0, 1200.0, 1.0, 0.6, 0.6),
        (10.0, 13.0, 1000.0, 0.6, 1.0, 0.9),
        (13.0, 17.0, 900.0, 1.0, 1.0, 0.3),
        (17.0, 22.0, 800.0, 0.25, 1.0, 0.75),
        (22.0, 28.0, 700.0, 1.0, 0.3, 0.5),
        (28.0, 35.0, 600.0, 0.8, 1.0, 0.9),
        (35.0, 45.0, 500.0, 1.0, 0.8, 0.4),
    ];
    // Per region: 24 sinusoids each for the shared, left-only and
    // right-only components (frequency, phase).
    const PER: usize = 24;
    let band_hz = f64::from(fs) / 128.0;
    let mut comps: Vec<(f64, f64, f64, f64, f64)> = Vec::new(); // (f, phase, gl, gr, amp)
    for &(lo, hi, amp, gl, gr, shared) in &regions {
        let a_shared = amp * shared.sqrt() / (PER as f64).sqrt();
        let a_ind = amp * (1.0 - shared).sqrt() / (PER as f64).sqrt();
        for _ in 0..PER {
            let f = (lo + rnd() * (hi - lo)) * band_hz;
            comps.push((f, rnd() * std::f64::consts::TAU, gl, gr, a_shared));
            let f = (lo + rnd() * (hi - lo)) * band_hz;
            comps.push((f, rnd() * std::f64::consts::TAU, gl, 0.0, a_ind));
            let f = (lo + rnd() * (hi - lo)) * band_hz;
            comps.push((f, rnd() * std::f64::consts::TAU, 0.0, gr, a_ind));
        }
    }
    let mut out = Vec::with_capacity(2 * n);
    for i in 0..n {
        let t = i as f64 / f64::from(fs);
        let mut l = 0.0;
        let mut r = 0.0;
        for &(f, ph, gl, gr, a) in &comps {
            let v = a * (2.0 * std::f64::consts::PI * f * t + ph).sin();
            l += gl * v;
            r += gr * v;
        }
        out.push(l.clamp(-32768.0, 32767.0) as i16);
        out.push(r.clamp(-32768.0, 32767.0) as i16);
    }
    out
}

fn channel(pcm: &[i16], n: usize, c: usize) -> Vec<f64> {
    pcm.iter()
        .skip(c)
        .step_by(n)
        .map(|&v| f64::from(v))
        .collect()
}

/// Lag (output relative to input) maximising the cross-correlation of
/// the channel sums over `0..=max_lag`.
fn best_lag(input: &[f64], output: &[f64], max_lag: usize) -> usize {
    let n = input.len().min(output.len()).min(44_100 * 2);
    let mut best = (0usize, f64::MIN);
    for lag in 0..=max_lag {
        let mut acc = 0.0;
        for i in 0..n.saturating_sub(lag) {
            acc += input[i] * output[i + lag];
        }
        if acc > best.1 {
            best = (lag, acc);
        }
    }
    best.0
}

/// Per-frame stereo cues of a stereo pair on `n_pars` bands: the
/// encoder's own analysis chain over consecutive 32-column frames.
fn stereo_cues(l: &[f64], r: &[f64], n_pars: usize) -> Vec<Vec<BandStats>> {
    let mut bl = EncoderAnalysisQmf::new();
    let mut br = EncoderAnalysisQmf::new();
    let n = l.len().min(r.len());
    let cols = n / 64;
    let mut xl: Vec<[Complex; 64]> = Vec::with_capacity(cols);
    let mut xr: Vec<[Complex; 64]> = Vec::with_capacity(cols);
    for c in 0..cols {
        xl.push(bl.push_slot(&l[64 * c..64 * c + 64]).unwrap());
        xr.push(br.push_slot(&r[64 * c..64 * c + 64]).unwrap());
    }
    let mut fe = PsAnalysis::new(hybrid_config_for(n_pars));
    let mut out = Vec::new();
    let frames = cols.saturating_sub(LOOKAHEAD) / NUM_QMF_SLOTS;
    for f in 0..frames {
        let lo = f * NUM_QMF_SLOTS;
        let hi = lo + NUM_QMF_SLOTS + LOOKAHEAD;
        let hyb = fe.analyze(&xl[lo..hi], &xr[lo..hi]).unwrap();
        out.push(band_stats(&hyb, n_pars, 0, NUM_QMF_SLOTS - 1).unwrap());
    }
    out
}

struct CueError {
    iid_mean_db: f64,
    iid_worst_db: f64,
    icc_mean: f64,
    frames: usize,
}

/// Compare per-frame cues of `a` (reference) and `b` over bands
/// carrying energy in the reference (skipping the first / last frames
/// where the codec's warm-up and flush live).
fn cue_error(a: &[Vec<BandStats>], b: &[Vec<BandStats>], phase: bool) -> CueError {
    let n = a.len().min(b.len());
    let mut iid_sum = 0.0;
    let mut iid_worst: f64 = 0.0;
    let mut icc_sum = 0.0;
    let mut count = 0usize;
    for f in 2..n.saturating_sub(2) {
        let peak = a[f].iter().map(BandStats::energy).fold(0.0, f64::max);
        for (x, y) in a[f].iter().zip(b[f].iter()) {
            if x.energy() < peak * 1e-3 {
                continue;
            }
            let d = (x.iid_db() - y.iid_db()).abs();
            iid_sum += d;
            iid_worst = iid_worst.max(d);
            let (ix, iy) = if phase {
                (x.icc_magnitude(), y.icc_magnitude())
            } else {
                (x.icc_real(), y.icc_real())
            };
            icc_sum += (ix - iy).abs();
            count += 1;
        }
    }
    CueError {
        iid_mean_db: iid_sum / count.max(1) as f64,
        iid_worst_db: iid_worst,
        icc_mean: icc_sum / count.max(1) as f64,
        frames: n,
    }
}

struct RoundTrip {
    stream: Vec<u8>,
    /// Decoded interleaved stereo, lag-aligned to the input.
    aligned: Vec<i16>,
    lag: usize,
}

fn round_trip(pcm: &[i16], cfg: HeAacConfig) -> (HeAacEncoder, RoundTrip) {
    let mut enc = HeAacEncoder::new(cfg).unwrap();
    let stream = enc.encode_all(pcm).unwrap();
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).expect("own decode");
    let mut out: Vec<i16> = Vec::new();
    for f in &frames {
        assert_eq!(f.channels, 2, "PS renders stereo");
        assert_eq!(f.sample_rate, cfg.sample_rate);
        out.extend_from_slice(&f.pcm);
    }
    let sum_in: Vec<f64> = pcm
        .chunks_exact(2)
        .map(|c| f64::from(c[0]) + f64::from(c[1]))
        .collect();
    let sum_out: Vec<f64> = out
        .chunks_exact(2)
        .map(|c| f64::from(c[0]) + f64::from(c[1]))
        .collect();
    let lag = best_lag(&sum_in, &sum_out, 3 * HE_FRAME_LEN);
    let aligned = out[2 * lag..].to_vec();
    (
        enc,
        RoundTrip {
            stream,
            aligned,
            lag,
        },
    )
}

fn cues_of(pcm: &[i16], n_pars: usize) -> Vec<Vec<BandStats>> {
    stereo_cues(&channel(pcm, 2, 0), &channel(pcm, 2, 1), n_pars)
}

/// The default configuration (20 bands, coarse IID, ICC, Ra) on the
/// panned synthetic: the decoded stereo image tracks the source's
/// per-band level differences within the coarse-grid step and its
/// coherences within a grid step.
#[test]
fn stereo_image_survives_the_default_configuration() {
    let pcm = synthetic_stereo(2.0, 44_100);
    let (enc, rt) = round_trip(&pcm, HeAacConfig::new_v2(44_100, 32_000));
    let n = rt.aligned.len().min(pcm.len());
    let a = cues_of(&pcm[..n], 20);
    let b = cues_of(&rt.aligned[..n], 20);
    let e = cue_error(&a, &b, false);
    eprintln!(
        "v2 default: lag {} frames {} IID err mean {:.2} dB worst {:.2} dB, ICC err {:.3}, {} bytes ({:.1} kbps), last PS {} bits",
        rt.lag,
        e.frames,
        e.iid_mean_db,
        e.iid_worst_db,
        e.icc_mean,
        rt.stream.len(),
        rt.stream.len() as f64 * 8.0 / 2.0 / 1000.0,
        enc.last_ps_frame().map(|p| p.bits).unwrap_or(0)
    );
    assert!(e.frames > 30);
    assert!(e.iid_mean_db < 2.5, "IID error {} dB", e.iid_mean_db);
    assert!(e.icc_mean < 0.25, "ICC error {}", e.icc_mean);
    // Rate: within 1.5× of the target over 2 s (+ flush).
    assert!(
        rt.stream.len() < (32_000 / 8 * 2) * 3 / 2,
        "{} bytes",
        rt.stream.len()
    );
}

/// Every band count and grid, with and without the phase layer:
/// all decode as stereo and keep the image.
#[test]
fn configuration_matrix_keeps_the_image() {
    let pcm = synthetic_stereo(1.2, 48_000);
    for bands in [PsBands::Ten, PsBands::Twenty, PsBands::ThirtyFour] {
        for (fine, phase) in [(false, false), (true, true)] {
            let cfg = HeAacConfig {
                ps: PsEncoderConfig {
                    bands,
                    fine_iid: fine,
                    phase,
                    ..PsEncoderConfig::default()
                },
                ..HeAacConfig::new_v2(48_000, 40_000)
            };
            let (_enc, rt) = round_trip(&pcm, cfg);
            let n = rt.aligned.len().min(pcm.len());
            let n_pars = bands.count();
            let a = cues_of(&pcm[..n], n_pars);
            let b = cues_of(&rt.aligned[..n], n_pars);
            let e = cue_error(&a, &b, phase);
            eprintln!(
                "{bands:?} fine={fine} phase={phase}: IID err {:.2} dB (worst {:.2}), ICC err {:.3}, {} bytes",
                e.iid_mean_db, e.iid_worst_db, e.icc_mean, rt.stream.len()
            );
            assert!(
                e.iid_mean_db < 3.0,
                "{bands:?}: IID error {} dB",
                e.iid_mean_db
            );
            assert!(e.icc_mean < 0.3, "{bands:?}: ICC error {}", e.icc_mean);
        }
    }
}

/// Time alignment of the stereo cues against the audio: a hard
/// left→right switch in the source reappears at the same instant in
/// the decoded output (to within the hybrid bank's few-slot reach),
/// which pins the encoder's stereo-analysis / mono-path column
/// alignment and the transient-driven VAR_BORDERS path.
#[test]
fn cue_switch_is_time_aligned_with_the_audio() {
    let fs = 44_100u32;
    let n = 2 * fs as usize; // 2 s
    let switch = 44_100 + 700; // not on a frame boundary
    let mut seed = 0x2468_ace1u32;
    let mut pcm = Vec::with_capacity(2 * n);
    for i in 0..n {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let noise = (f64::from(seed >> 8) / f64::from(1u32 << 24) - 0.5) * 6000.0;
        let t = i as f64 / f64::from(fs);
        let v = noise
            + 4000.0 * (2.0 * std::f64::consts::PI * 300.0 * t).sin()
            + 2000.0 * (2.0 * std::f64::consts::PI * 3300.0 * t).sin();
        if i < switch {
            pcm.push(v as i16);
            pcm.push(0);
        } else {
            pcm.push(0);
            pcm.push(v as i16);
        }
    }
    let (_enc, rt) = round_trip(&pcm, HeAacConfig::new_v2(fs, 32_000));
    // Per-64-sample slot energies of the decoded channels.
    let l = channel(&rt.aligned, 2, 0);
    let r = channel(&rt.aligned, 2, 1);
    let slots = l.len().min(r.len()) / 64;
    let e = |x: &[f64], s: usize| x[64 * s..64 * s + 64].iter().map(|v| v * v).sum::<f64>();
    // First slot (after the warm-up) where the right channel holds
    // more energy than the left.
    let first_right = (8..slots)
        .find(|&s| e(&r, s) > e(&l, s))
        .expect("switch seen");
    let expect = switch / 64;
    let delta = first_right as i64 - expect as i64;
    eprintln!("switch at slot {expect}, decoded right-dominant from slot {first_right} (Δ {delta} slots, lag {})", rt.lag);
    assert!(delta.abs() <= 6, "cue switch misaligned by {delta} slots");
    // Before the switch the image is hard left, after it hard right
    // (long-term, away from the transition).
    let (mut el, mut er) = (0.0, 0.0);
    for s in 8..expect - 8 {
        el += e(&l, s);
        er += e(&r, s);
    }
    assert!(el > 30.0 * er, "pre-switch not left-dominant: {el} vs {er}");
    let (mut el, mut er) = (0.0, 0.0);
    for s in expect + 8..slots - 8 {
        el += e(&l, s);
        er += e(&r, s);
    }
    assert!(
        er > 30.0 * el,
        "post-switch not right-dominant: {el} vs {er}"
    );
}

/// Explicit signalling: the ADTS output wrapped into LOAS/LATM with
/// either HE-AAC v2 ASC form decodes through the crate's LOAS driver
/// identically to the ADTS path, as stereo at the full rate.
#[test]
fn loas_wrapped_he_aac_v2_decodes_as_stereo() {
    let pcm = synthetic_stereo(0.8, 44_100);
    let mut enc = HeAacEncoder::new(HeAacConfig::new_v2(44_100, 32_000)).unwrap();
    let adts = enc.encode_all(&pcm).unwrap();
    let mut adts_dec = StreamDecoder::new();
    let adts_pcm: Vec<i16> = adts_dec
        .decode_all(&adts)
        .unwrap()
        .iter()
        .flat_map(|f| f.pcm.iter().copied())
        .collect();
    for hierarchical in [false, true] {
        let asc = enc.audio_specific_config(hierarchical);
        let bits = enc.audio_specific_config_bits(hierarchical);
        let mut wtr = oxideav_aac::latm_writer::LoasWriter::new(asc, bits, 8).unwrap();
        let loas = wtr.wrap_adts_stream(&adts).unwrap();
        let mut dec = oxideav_aac::latm::LoasDecoder::new();
        let frames = dec.decode_all(&loas).expect("LOAS decode");
        assert!(!frames.is_empty());
        assert!(frames
            .iter()
            .all(|f| f.sample_rate == 44_100 && f.channels == 2));
        let latm_pcm: Vec<i16> = frames.iter().flat_map(|f| f.pcm.iter().copied()).collect();
        assert_eq!(latm_pcm, adts_pcm, "hierarchical={hierarchical}");
    }
}

fn ffmpeg() -> Option<&'static str> {
    ["ffmpeg"].into_iter().find(|cand| {
        Command::new(cand)
            .arg("-version")
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false)
    })
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

/// Long-term per-band IID (dB) of a stereo pair over all frames.
fn long_term_iid(pcm: &[i16]) -> Vec<f64> {
    let cues = cues_of(pcm, 20);
    let mut el = [0.0f64; 20];
    let mut er = [0.0f64; 20];
    for f in cues.iter().skip(2) {
        for (b, s) in f.iter().enumerate() {
            el[b] += s.el;
            er[b] += s.er;
        }
    }
    (0..20).map(|b| 10.0 * (el[b] / er[b]).log10()).collect()
}

/// Black box: an external reference decoder BINARY (invoked as an
/// opaque tool, skip-if-absent) must decode every HE-AAC v2 stream
/// without diagnostics, as stereo at the doubled rate, and its
/// long-term per-band level differences must agree with both this
/// crate's decoder and the source.
#[test]
fn reference_binary_decodes_our_he_aac_v2_as_stereo() {
    let Some(ff) = ffmpeg() else {
        eprintln!("skip: no ffmpeg binary on PATH");
        return;
    };
    let dir = std::env::temp_dir().join("oxideav-aac-ps-blackbox");
    fs::create_dir_all(&dir).unwrap();
    for (name, fs_out, bitrate, ps) in [
        (
            "v2_44k_default",
            44_100u32,
            32_000u32,
            PsEncoderConfig::default(),
        ),
        (
            "v2_48k_34fine_phase",
            48_000,
            48_000,
            PsEncoderConfig {
                bands: PsBands::ThirtyFour,
                fine_iid: true,
                phase: true,
                ..PsEncoderConfig::default()
            },
        ),
    ] {
        let pcm = synthetic_stereo(1.5, fs_out);
        let cfg = HeAacConfig {
            ps,
            ..HeAacConfig::new_v2(fs_out, bitrate)
        };
        let (_enc, rt) = round_trip(&pcm, cfg);
        let aac = dir.join(format!("{name}.aac"));
        let wav = dir.join(format!("{name}.wav"));
        fs::write(&aac, &rt.stream).unwrap();
        let _ = fs::remove_file(&wav);
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
        assert_eq!(ref_rate, fs_out, "{name}: output rate");
        assert_eq!(ref_ch, 2, "{name}: PS renders stereo");
        let n_ref = ref_pcm.len() / 2;
        let n_ours = rt.aligned.len() / 2 + rt.lag;
        assert!(
            (n_ours as i64 - n_ref as i64).unsigned_abs() as usize <= 2 * HE_FRAME_LEN,
            "{name}: length {n_ref} vs ours {n_ours}"
        );

        let iid_src = long_term_iid(&pcm);
        let iid_ours = long_term_iid(&rt.aligned);
        let iid_ref = long_term_iid(&ref_pcm);
        let mut d_ours = 0.0;
        let mut d_src = 0.0;
        let mut worst_ours: f64 = 0.0;
        for b in 0..20 {
            let d = (iid_ref[b] - iid_ours[b]).abs();
            d_ours += d;
            worst_ours = worst_ours.max(d);
            d_src += (iid_ref[b] - iid_src[b]).abs();
        }
        d_ours /= 20.0;
        d_src /= 20.0;
        eprintln!(
            "{name}: reference vs ours mean |ΔIID| {d_ours:.2} dB (worst {worst_ours:.2}), reference vs source {d_src:.2} dB"
        );
        assert!(d_ours < 1.5, "{name}: decoders disagree by {d_ours} dB");
        assert!(worst_ours < 4.0, "{name}: worst band {worst_ours} dB");
        assert!(d_src < 3.0, "{name}: image vs source {d_src} dB");
    }
}

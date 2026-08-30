//! HE-AAC v1 encoder round trips through the crate's own SBR decoder:
//! the reconstructed high band matches the input's per-QMF-band
//! envelope, and the core band survives at a real SNR.
//!
//! Measurement: the same 64-band analysis bank
//! ([`oxideav_aac::sbr_qmf::EncoderAnalysisQmf`]) runs over the input
//! and the decoded output (lag-aligned by cross-correlation of the
//! low band); per QMF band in the SBR range the long-term energies
//! are compared in dB, and per frame the envelope error is averaged.

use std::fs;
use std::path::PathBuf;

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::he_aac_encoder::{HeAacConfig, HeAacEncoder, HE_FRAME_LEN};
use oxideav_aac::sbr_qmf::EncoderAnalysisQmf;

/// Read the `data` chunk of a 16-bit WAV as interleaved `i16`.
fn read_wav_s16(path: &PathBuf) -> Option<Vec<i16>> {
    let d = fs::read(path).ok()?;
    let mut i = 12;
    while i + 8 <= d.len() {
        let cid = &d[i..i + 4];
        let sz = u32::from_le_bytes([d[i + 4], d[i + 5], d[i + 6], d[i + 7]]) as usize;
        let body = i + 8;
        if cid == b"data" {
            let end = (body + sz).min(d.len());
            return Some(
                d[body..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            );
        }
        i = body + sz + (sz & 1);
    }
    None
}

/// Deinterleave channel `c` of `n` channels.
fn channel(pcm: &[i16], n: usize, c: usize) -> Vec<f64> {
    pcm.iter()
        .skip(c)
        .step_by(n)
        .map(|&v| f64::from(v))
        .collect()
}

/// Lag (output relative to input) maximising the cross-correlation of
/// the two signals over `0..=max_lag`.
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

/// Per-QMF-band long-term energy of `x` (64 bands).
fn band_energies(x: &[f64]) -> [f64; 64] {
    let mut bank = EncoderAnalysisQmf::new();
    let mut e = [0.0f64; 64];
    for slot in x.chunks_exact(64) {
        let cols = bank.push_slot(slot).unwrap();
        for (k, c) in cols.iter().enumerate() {
            e[k] += c.norm_sqr();
        }
    }
    e
}

/// Per-frame, per-band envelope error in dB (mean absolute) over the
/// SBR range `k_x..k_end`, skipping bands quieter than `floor` in the
/// input.
fn envelope_error_db(input: &[f64], output: &[f64], table: &[i32]) -> (f64, f64) {
    let mut bi = EncoderAnalysisQmf::new();
    let mut bo = EncoderAnalysisQmf::new();
    let n = input.len().min(output.len());
    let frames = n / HE_FRAME_LEN;
    let mut sum = 0.0;
    let mut worst: f64 = 0.0;
    let mut count = 0usize;
    for f in 0..frames {
        let mut ei = vec![0.0f64; 64];
        let mut eo = vec![0.0f64; 64];
        for s in 0..HE_FRAME_LEN / 64 {
            let off = f * HE_FRAME_LEN + s * 64;
            let ci = bi.push_slot(&input[off..off + 64]).unwrap();
            let co = bo.push_slot(&output[off..off + 64]).unwrap();
            for k in 0..64 {
                ei[k] += ci[k].norm_sqr();
                eo[k] += co[k].norm_sqr();
            }
        }
        // Skip the priming frame and quiet bands (below −60 dBFS-ish).
        if f == 0 {
            continue;
        }
        for w in table.windows(2) {
            let (lo, hi) = (w[0] as usize, w[1] as usize);
            let bi_e: f64 = ei[lo..hi].iter().sum();
            let bo_e: f64 = eo[lo..hi].iter().sum();
            // Skip bands quieter than roughly −60 dBFS.
            if bi_e < 32.0 * 1e4 * (hi - lo) as f64 {
                continue;
            }
            let db = (10.0 * (bo_e.max(1.0) / bi_e).log10()).abs();
            sum += db;
            worst = worst.max(db);
            count += 1;
        }
    }
    (sum / count.max(1) as f64, worst)
}

struct Metrics {
    lag: usize,
    low_snr_db: f64,
    env_err_db: f64,
    env_worst_db: f64,
    hf_energy_ratio_db: f64,
    bytes: usize,
}

fn round_trip(pcm: &[i16], cfg: HeAacConfig) -> Metrics {
    let ch = usize::from(cfg.channels);
    let mut enc = HeAacEncoder::new(cfg).unwrap();
    let stream = enc.encode_all(pcm).unwrap();
    let k_x = enc.sbr().bands().k_x as usize;
    let k_end = (enc.sbr().bands().k_x + enc.sbr().bands().m) as usize;
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).expect("decode");
    assert!(frames.iter().all(|f| f.sample_rate == cfg.sample_rate));
    let mut out: Vec<i16> = Vec::new();
    for f in &frames {
        assert_eq!(f.channels, ch);
        out.extend_from_slice(&f.pcm);
    }
    let input = channel(pcm, ch, 0);
    let output = channel(&out, ch, 0);
    // Low-band alignment: filter both to the core band with the QMF
    // bank's lowest bands is overkill; the raw cross-correlation is
    // dominated by the (preserved) low band anyway.
    let lag = best_lag(&input, &output, 3 * HE_FRAME_LEN);
    let aligned: Vec<f64> = output[lag..].to_vec();
    let n = input.len().min(aligned.len());
    // Low-band SNR: compare energies below k_x via the QMF bank.
    let table = enc.sbr().bands().f_table_high.clone();
    let ei = band_energies(&input[..n]);
    let eo = band_energies(&aligned[..n]);
    let mut bd = EncoderAnalysisQmf::new();
    let mut err = 0.0;
    let mut sig = 0.0;
    for s in 0..n / 64 {
        let cols_d = bd
            .push_slot(
                &(0..64)
                    .map(|i| input[s * 64 + i] - aligned[s * 64 + i])
                    .collect::<Vec<_>>(),
            )
            .unwrap();
        for c in cols_d.iter().take(k_x.saturating_sub(1)) {
            err += c.norm_sqr();
        }
    }
    for e in ei.iter().take(k_x.saturating_sub(1)) {
        sig += e;
    }
    let low_snr_db = 10.0 * (sig / err.max(1.0)).log10();
    let hf_in: f64 = ei[k_x..k_end].iter().sum();
    let hf_out: f64 = eo[k_x..k_end].iter().sum();
    let (env_err_db, env_worst_db) = envelope_error_db(&input[..n], &aligned[..n], &table);
    Metrics {
        lag,
        low_snr_db,
        env_err_db,
        env_worst_db,
        hf_energy_ratio_db: 10.0 * (hf_out / hf_in.max(1.0)).log10(),
        bytes: stream.len(),
    }
}

/// Multitone spanning the core and SBR ranges plus a high-band noise
/// bed: every SBR band carries something to reproduce.
fn synthetic(seconds: f64, fs: u32, channels: usize) -> Vec<i16> {
    let n = (seconds * f64::from(fs)) as usize;
    let mut seed = 0x9e37_79b9u32;
    let mut out = Vec::with_capacity(n * channels);
    for i in 0..n {
        let t = i as f64 / f64::from(fs);
        let mut v = 0.0;
        for &(f, a) in &[
            (220.0, 4000.0),
            (880.0, 3000.0),
            (2500.0, 2500.0),
            (5200.0, 2000.0),
            (7300.0, 1500.0),
            (9800.0, 1200.0),
            (12500.0, 1000.0),
            (15000.0, 800.0),
        ] {
            v += a * (2.0 * std::f64::consts::PI * f * t).sin();
        }
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let noise = (f64::from(seed >> 8) / f64::from(1u32 << 24) - 0.5) * 1500.0;
        v += noise;
        for c in 0..channels {
            let g = if c == 1 { 0.7 } else { 1.0 };
            out.push((v * g).clamp(-32768.0, 32767.0) as i16);
        }
    }
    out
}

#[test]
fn synthetic_mono_44100_reconstructs_the_high_band_envelope() {
    let pcm = synthetic(2.0, 44_100, 1);
    for interp in [true, false] {
        let cfg = HeAacConfig {
            interpol_freq: interp,
            ..HeAacConfig::new(44_100, 1, 32_000)
        };
        let m = round_trip(&pcm, cfg);
        eprintln!(
            "mono 32k interp={interp}: lag {} low-SNR {:.1} dB env-err {:.2} dB (worst {:.1}) HF ratio {:.2} dB, {} bytes",
            m.lag, m.low_snr_db, m.env_err_db, m.env_worst_db, m.hf_energy_ratio_db, m.bytes
        );
    }
    let m = round_trip(&pcm, HeAacConfig::new(44_100, 1, 32_000));
    assert!(m.low_snr_db > 12.0, "low band SNR {}", m.low_snr_db);
    assert!(m.env_err_db < 3.5, "envelope error {} dB", m.env_err_db);
    assert!(
        m.hf_energy_ratio_db.abs() < 2.0,
        "HF energy {} dB",
        m.hf_energy_ratio_db
    );
    // Rate: within 1.5× of the target over 2 s (+ flush).
    assert!(m.bytes < (32_000 / 8 * 2) * 3 / 2, "{} bytes", m.bytes);
}

#[test]
fn synthetic_stereo_48000_with_crc_and_coupling() {
    let pcm = synthetic(1.5, 48_000, 2);
    for coupling in [false, true] {
        let cfg = HeAacConfig {
            sbr_crc: true,
            coupling,
            ..HeAacConfig::new(48_000, 2, 64_000)
        };
        let m = round_trip(&pcm, cfg);
        eprintln!(
            "stereo 64k coupling={coupling}: lag {} low-SNR {:.1} dB env-err {:.2} dB (worst {:.1}) HF ratio {:.2} dB, {} bytes",
            m.lag, m.low_snr_db, m.env_err_db, m.env_worst_db, m.hf_energy_ratio_db, m.bytes
        );
        assert!(m.low_snr_db > 12.0, "low band SNR {}", m.low_snr_db);
        assert!(m.env_err_db < 3.5, "envelope error {} dB", m.env_err_db);
        assert!(
            m.hf_energy_ratio_db.abs() < 2.0,
            "HF energy {} dB",
            m.hf_energy_ratio_db
        );
    }
}

/// Real material: the staged AAC-LC fixture's reference PCM, transcoded
/// to HE-AAC at 32 kbps stereo.
#[test]
fn fixture_pcm_transcodes_to_he_aac() {
    let dir = PathBuf::from("../../docs/audio/aac/fixtures/aac-lc-stereo-44100-128kbps-adts");
    let Some(pcm) = read_wav_s16(&dir.join("expected.wav")) else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let m = round_trip(&pcm, HeAacConfig::new(44_100, 2, 32_000));
    eprintln!(
        "fixture 32k stereo: lag {} low-SNR {:.1} dB env-err {:.2} dB (worst {:.1}) HF ratio {:.2} dB, {} bytes",
        m.lag, m.low_snr_db, m.env_err_db, m.env_worst_db, m.hf_energy_ratio_db, m.bytes
    );
    assert!(m.low_snr_db > 8.0, "low band SNR {}", m.low_snr_db);
    assert!(m.env_err_db < 4.0, "envelope error {} dB", m.env_err_db);
    assert!(
        m.hf_energy_ratio_db.abs() < 3.0,
        "HF energy {} dB",
        m.hf_energy_ratio_db
    );
}

/// Explicit-signalling carriage: the HE-AAC ADTS output wrapped into
/// LOAS/LATM with the backward-compatible ASC decodes through the
/// crate's LOAS driver at the full rate.
#[test]
fn loas_wrapped_he_aac_decodes_at_full_rate() {
    let pcm = synthetic(1.0, 44_100, 2);
    let mut enc = HeAacEncoder::new(HeAacConfig::new(44_100, 2, 48_000)).unwrap();
    let adts = enc.encode_all(&pcm).unwrap();
    let asc = enc.audio_specific_config(false);
    let mut wtr = oxideav_aac::latm_writer::LoasWriter::new(asc, 37, 8).unwrap();
    let loas = wtr.wrap_adts_stream(&adts).unwrap();

    let mut dec = oxideav_aac::latm::LoasDecoder::new();
    let frames = dec.decode_all(&loas).expect("LOAS decode");
    assert!(!frames.is_empty());
    assert!(frames.iter().all(|f| f.sample_rate == 44_100));
    assert!(frames.iter().all(|f| f.channels == 2));
    let latm_pcm: Vec<i16> = frames.iter().flat_map(|f| f.pcm.iter().copied()).collect();

    // Identical to the ADTS decode path.
    let mut adts_dec = StreamDecoder::new();
    let adts_frames = adts_dec.decode_all(&adts).unwrap();
    let adts_pcm: Vec<i16> = adts_frames
        .iter()
        .flat_map(|f| f.pcm.iter().copied())
        .collect();
    assert_eq!(latm_pcm, adts_pcm);
}

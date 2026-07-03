//! HE-AAC v1 (AAC-LC core + §4.6.18 SBR) end-to-end PCM validation
//! against the staged `expected.wav` reference.
//!
//! The SBR back-end (QMF analysis → HF generation → envelope
//! adjustment → 64-band synthesis) is driven through the ordinary
//! [`StreamDecoder`] ADTS path: the decoder auto-detects the SBR FIL
//! extension and emits 2048-sample frames at the doubled rate.
//!
//! The decoded PCM turns out **byte-exact to within 1 LSB** against
//! the reference (≥ 99.9% of samples identical, the residual being
//! `f64` vs `float32` filterbank rounding — the same bound as the
//! AAC-LC corpus), at zero lag and with an identical sample count, so
//! the test pins that directly alongside the §8 PCM-RMS ratio.
//!
//! Skips (logged, success) when `docs/` is absent (standalone CI).

use std::fs;
use std::path::PathBuf;

use oxideav_aac::decode::StreamDecoder;

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

#[test]
fn he_aac_v1_sbr_pcm_byte_exact() {
    let dir = PathBuf::from("../../docs/audio/aac/fixtures/he-aac-v1-stereo-44100-32kbps-adts");
    let Ok(data) = fs::read(dir.join("input.aac")) else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let expected = read_wav_s16(&dir.join("expected.wav")).expect("expected.wav");

    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&data).expect("decode_all");
    assert!(!frames.is_empty());
    let mut ours: Vec<i16> = Vec::new();
    for f in &frames {
        // Every frame of this stream is SBR-active: stereo, 2048
        // samples per channel, at the doubled (44.1 kHz) rate.
        assert_eq!(f.channels, 2);
        assert_eq!(f.sample_rate, 44_100);
        assert_eq!(f.pcm.len(), 2048 * 2);
        ours.extend_from_slice(&f.pcm);
    }

    // Zero lag, identical sample count.
    assert_eq!(ours.len(), expected.len(), "sample-count mismatch");

    let mut exact = 0usize;
    let mut max_err = 0i32;
    let mut err_sse = 0.0f64;
    let mut sig_sse = 0.0f64;
    for (a, b) in ours.iter().zip(expected.iter()) {
        let e = i32::from(*a) - i32::from(*b);
        if e == 0 {
            exact += 1;
        }
        max_err = max_err.max(e.abs());
        err_sse += f64::from(e) * f64::from(e);
        sig_sse += f64::from(*b) * f64::from(*b);
    }
    let exact_frac = exact as f64 / ours.len() as f64;
    let ratio = (err_sse / sig_sse.max(1.0)).sqrt();
    eprintln!("he-aac-v1 SBR: exact {exact_frac:.4}, max_err {max_err}, err/sig RMS {ratio:.6}");
    assert!(
        exact_frac >= 0.999,
        "exact-sample fraction {exact_frac:.4} below 99.9%"
    );
    assert!(max_err <= 1, "max per-sample error {max_err} exceeds 1 LSB");
    assert!(ratio < 1e-3, "error-to-signal RMS {ratio:.6}");
}

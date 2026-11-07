//! Multichannel encoder round-trips — the Table 1.19 default
//! `channelConfiguration` layouts (3 / 4 / 5 / 5.1 / 7.1) through the
//! crate's own decoder.
//!
//! Every speaker carries a distinct tone, so the per-channel
//! error-to-signal ratios pin the whole channel pipeline: the
//! encoder's canonical→element-order permutation must be the exact
//! inverse of the decoder's §1.6.3.5 element→canonical reorder, or a
//! swapped pair would blow its ratio to ~1.

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, FRAME_LEN, StreamEncoder};

/// One distinct tone per channel (canonical interleaved order); the
/// `lfe` channel (canonical index, if any) gets a low-frequency tone
/// that survives the §4.5.2.1.3 12-line LFE limit.
fn per_channel_tones(n: usize, channels: usize, lfe: Option<usize>) -> Vec<i16> {
    let mut out = Vec::with_capacity(n * channels);
    for i in 0..n {
        let t = i as f64;
        for c in 0..channels {
            let v = if Some(c) == lfe {
                // ~40 Hz at 44.1 kHz: well inside the first LFE band.
                8000.0 * (2.0 * std::f64::consts::PI * 40.0 / 44_100.0 * t).sin()
            } else {
                // Spread the tones across the spectrum, one per
                // speaker, plus a quieter second partial.
                let w = 0.02 + 0.045 * c as f64;
                9000.0 * (w * t).sin() + 3000.0 * (2.7 * w * t + 0.4).sin()
            };
            out.push(v.round() as i16);
        }
    }
    out
}

/// Encode → decode → per-channel err/sig ratios (aligned past the
/// one-hop encoder delay).
fn roundtrip_ratios(channels: usize, lfe: Option<usize>, expected_config: u8) -> Vec<f64> {
    let n = 6 * FRAME_LEN;
    let pcm = per_channel_tones(n, channels, lfe);
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: channels as u8,
        bitrate: 64_000 * channels as u32,
    })
    .unwrap();
    let stream = enc.encode_all(&pcm).unwrap();
    let (header, _) = AdtsHeader::parse(&stream).unwrap();
    assert_eq!(header.channel_configuration, expected_config);

    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).unwrap();
    let mut decoded = Vec::new();
    for f in &frames {
        assert_eq!(f.channels, channels);
        decoded.extend_from_slice(&f.pcm);
    }
    assert_eq!(decoded.len(), (n + FRAME_LEN) * channels);
    let aligned = &decoded[FRAME_LEN * channels..];
    (0..channels)
        .map(|c| {
            let mut err = 0.0f64;
            let mut sig = 0.0f64;
            for i in 0..n {
                let x = f64::from(pcm[i * channels + c]);
                let y = f64::from(aligned[i * channels + c]);
                err += (x - y) * (x - y);
                sig += x * x;
            }
            (err / sig.max(1.0)).sqrt()
        })
        .collect()
}

fn assert_ratios(ratios: &[f64], tol: f64, label: &str) {
    for (c, r) in ratios.iter().enumerate() {
        assert!(
            *r < tol,
            "{label}: channel {c} err/sig {r:.4} exceeds {tol}"
        );
    }
    eprintln!("{label}: per-channel err/sig = {ratios:?}");
}

/// Config 3 (C, L, R — one SCE + one CPE).
#[test]
fn three_channel_roundtrip() {
    assert_ratios(&roundtrip_ratios(3, None, 3), 0.05, "3.0");
}

/// Config 4 (C, L, R, rear C).
#[test]
fn four_channel_roundtrip() {
    assert_ratios(&roundtrip_ratios(4, None, 4), 0.05, "4.0");
}

/// Config 5 (C, L, R, Ls, Rs).
#[test]
fn five_channel_roundtrip() {
    assert_ratios(&roundtrip_ratios(5, None, 5), 0.05, "5.0");
}

/// Config 6 — 5.1. Canonical order `L R C LFE Ls Rs`: the LFE is
/// canonical channel 3 and must come back on channel 3 (the encoder's
/// inverse permutation moves it to the trailing LFE element and the
/// decoder's Table 1.19 reorder moves it back).
#[test]
fn five_one_roundtrip_with_lfe() {
    assert_ratios(&roundtrip_ratios(6, Some(3), 6), 0.05, "5.1");
}

/// Config 7 — 7.1 (canonical `L R C LFE Lc Rc Ls Rs`; LFE is
/// canonical channel 3).
#[test]
fn seven_one_roundtrip_with_lfe() {
    assert_ratios(&roundtrip_ratios(8, Some(3), 7), 0.05, "7.1");
}

/// The §4.5.2.1.3 LFE wire restrictions hold on a 5.1 stream: every
/// frame's LFE element is ONLY_LONG / sine, carries no TNS and no
/// spectrum above the lowest 12 lines. Pinned through the decoded
/// output: an LFE fed a *broadband* signal comes back band-limited —
/// only the sub-130 Hz part survives — while a full-range channel
/// keeps its highs.
#[test]
fn lfe_element_is_band_limited() {
    let n = 4 * FRAME_LEN;
    let channels = 6usize;
    let mut pcm = Vec::with_capacity(n * channels);
    for i in 0..n {
        let t = i as f64;
        // Broadband content (a mid-frequency tone) on every channel,
        // including the LFE (canonical index 3).
        for _c in 0..channels {
            pcm.push((9000.0 * (0.5 * t).sin()).round() as i16);
        }
    }
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: channels as u8,
        bitrate: 384_000,
    })
    .unwrap();
    let stream = enc.encode_all(&pcm).unwrap();
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).unwrap();
    let mut decoded = Vec::new();
    for f in &frames {
        decoded.extend_from_slice(&f.pcm);
    }
    let aligned = &decoded[FRAME_LEN * channels..];
    let energy = |c: usize| -> f64 {
        (0..n)
            .map(|i| {
                let v = f64::from(aligned[i * channels + c]);
                v * v
            })
            .sum()
    };
    // The 0.5 rad/sample tone (~3.5 kHz) is far above the 12-line LFE
    // ceiling (~130 Hz at 44.1 kHz / 1024 lines): the decoded LFE
    // must be near-silent while the full-range channels reproduce it.
    let full = energy(0);
    let lfe = energy(3);
    assert!(
        lfe < full * 1e-3,
        "LFE energy {lfe:.1} not band-limited vs full-range {full:.1}"
    );
    assert!(full > 0.0);
}

/// Deterministic noise in [-1, 1): a 32-bit LCG, so the fixture is the same on every run.
fn noise(state: &mut u32) -> f64 {
    *state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    f64::from(*state >> 8) / f64::from(1u32 << 24) * 2.0 - 1.0
}

/// A film-like 5.1 mix at 48 kHz (canonical `L R C LFE Ls Rs`): dialogue-like content on the
/// center at -25 dBFS RMS, room tone on the four other speakers 35 dB below it, a silent LFE.
fn film_like_five_one(seconds: usize) -> Vec<i16> {
    let n = seconds * 48_000;
    let mut out = Vec::with_capacity(n * 6);
    let mut state = 0x2545_F491u32;
    let center_rms = 32768.0 * 10f64.powf(-25.0 / 20.0);
    let quiet_rms = 32768.0 * 10f64.powf(-60.0 / 20.0);
    for i in 0..n {
        let t = i as f64 / 48_000.0;
        // Dialogue stand-in: a buzzing harmonic series around 180 Hz under a slow envelope, with
        // a little broadband noise so every band carries something.
        let env = 0.6 + 0.4 * (2.0 * std::f64::consts::PI * 3.0 * t).sin();
        let mut voice = 0.0;
        for h in 1..=12 {
            voice += (2.0 * std::f64::consts::PI * 180.0 * f64::from(h) * t).sin() / f64::from(h);
        }
        let center = center_rms * (0.9 * env * voice / 1.3 + 0.3 * noise(&mut state));
        for c in 0..6 {
            let v = match c {
                2 => center,
                3 => 0.0,
                _ => quiet_rms * noise(&mut state) * 1.7,
            };
            out.push(v.round().clamp(-32768.0, 32767.0) as i16);
        }
    }
    out
}

/// Encode → decode → per-channel SNR in dB for `pcm` at `rate` with `bitrate` over all channels.
fn roundtrip_snr_db(pcm: &[i16], channels: usize, rate: u32, bitrate: u32) -> Vec<f64> {
    let n = pcm.len() / channels;
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: rate,
        channels: channels as u8,
        bitrate,
    })
    .unwrap();
    let stream = enc.encode_all(pcm).unwrap();
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).unwrap();
    let mut decoded = Vec::new();
    for f in &frames {
        decoded.extend_from_slice(&f.pcm);
    }
    let aligned = &decoded[FRAME_LEN * channels..];
    (0..channels)
        .map(|c| {
            let (mut err, mut sig) = (0.0f64, 0.0f64);
            for i in 0..n.min(aligned.len() / channels) {
                let x = f64::from(pcm[i * channels + c]);
                let y = f64::from(aligned[i * channels + c]);
                err += (x - y) * (x - y);
                sig += x * x;
            }
            if err == 0.0 {
                f64::INFINITY
            } else {
                10.0 * (sig / err).log10()
            }
        })
        .collect()
}

/// A 5.1 film mix puts nearly all the sound on one speaker, with room tone 35 dB down on the
/// others and nothing on the LFE. The rate must follow the sound: the loud channel at
/// 6 x 96 kbit/s should come back at least as clean as a mono encode of it at 96 kbit/s does,
/// while the quiet channels, judged against the frame's loudest peak as the two channels of a
/// pair already are against each other, get the precision that leaves (their room tone, white
/// noise here, comes back a little noisier than itself: measured 6 dB, 7 dB before the rule).
/// Judged each against its own peak the center read 9.2 dB here against 13.1 dB alone, and a
/// library excerpt's center read 15 dB against 38 dB from a reference encoder at the same rate.
#[test]
fn five_one_rate_follows_the_loud_channel() {
    let pcm = film_like_five_one(6);
    let snr = roundtrip_snr_db(&pcm, 6, 48_000, 6 * 96_000);
    let center_alone: Vec<i16> = pcm.chunks_exact(6).map(|f| f[2]).collect();
    let mono = roundtrip_snr_db(&center_alone, 1, 48_000, 96_000)[0];
    eprintln!(
        "5.1 film-like per-channel SNR dB = {snr:.1?}; center alone at 96 kbit/s = {mono:.1}"
    );
    assert!(
        snr[2] >= mono - 3.0,
        "center {:.1} dB in the 5.1 mix against {mono:.1} dB alone",
        snr[2]
    );
    assert!(
        snr[3].is_infinite(),
        "a silent LFE comes back silent: {:.1}",
        snr[3]
    );
    for c in [0usize, 1, 4, 5] {
        assert!(snr[c] >= 3.0, "quiet channel {c} at {:.1} dB", snr[c]);
    }
}

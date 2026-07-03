//! End-to-end round-trip tests for [`oxideav_aac::encoder::StreamEncoder`]:
//! PCM → ADTS bitstream → the crate's own
//! [`oxideav_aac::decode::StreamDecoder`] → PCM, asserting the
//! reconstruction error stays a small fraction of the signal.
//!
//! The encoder delay is exactly one hop (1024 samples): decoded frame
//! 0 is the zero-primed warmup, and decoded frame `f ≥ 1` carries
//! input hop `f − 1`. The comparisons below therefore skip the first
//! 1024 decoded samples per channel.

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder, FRAME_LEN};

/// Encode `pcm` and decode it back, returning the interleaved
/// decoder output.
fn roundtrip(pcm: &[i16], config: EncoderConfig) -> Vec<i16> {
    let mut enc = StreamEncoder::new(config).expect("encoder builds");
    let stream = enc.encode_all(pcm).expect("encode succeeds");
    let mut dec = StreamDecoder::new();
    let frames = dec
        .decode_all(&stream)
        .expect("self-produced stream decodes");
    let mut out = Vec::new();
    for f in &frames {
        out.extend_from_slice(&f.pcm);
    }
    out
}

/// Error-to-signal RMS ratio between `a` and `b` (same length).
fn err_to_signal_rms(a: &[i16], b: &[i16]) -> f64 {
    assert_eq!(a.len(), b.len());
    assert!(!a.is_empty());
    let err: f64 = a
        .iter()
        .zip(b)
        .map(|(&x, &y)| {
            let d = f64::from(x) - f64::from(y);
            d * d
        })
        .sum::<f64>();
    let sig: f64 = a.iter().map(|&x| f64::from(x) * f64::from(x)).sum();
    (err / sig.max(1.0)).sqrt()
}

/// A deterministic multi-tone test signal on the ±32768 axis.
fn multitone(n: usize, channels: usize) -> Vec<i16> {
    let mut out = Vec::with_capacity(n * channels);
    for i in 0..n {
        let t = i as f64;
        for c in 0..channels {
            let phase = c as f64 * 0.7;
            let v = 9000.0 * (0.031 * t + phase).sin()
                + 5000.0 * (0.113 * t + 0.3 + phase).sin()
                + 2500.0 * (0.402 * t + 1.1 + phase).sin();
            out.push(v.round() as i16);
        }
    }
    out
}

#[test]
fn mono_multitone_roundtrips_within_tolerance() {
    let n = 8 * FRAME_LEN;
    let pcm = multitone(n, 1);
    let config = EncoderConfig {
        sample_rate: 44_100,
        channels: 1,
        bitrate: 128_000,
    };
    let decoded = roundtrip(&pcm, config);
    // ⌈n/1024⌉ + 1 frames, 1024 samples each.
    assert_eq!(decoded.len(), n + FRAME_LEN);
    // The warmup frame is the zero-primed analysis window's left
    // half: silence up to the quantization noise of hop 0 imaged
    // through the IMDCT (a lossy codec's noise is not confined to
    // the signal region). Bound it in RMS terms.
    let warmup_rms = (decoded[..FRAME_LEN]
        .iter()
        .map(|&s| f64::from(s) * f64::from(s))
        .sum::<f64>()
        / FRAME_LEN as f64)
        .sqrt();
    let sig_rms = (pcm
        .iter()
        .map(|&s| f64::from(s) * f64::from(s))
        .sum::<f64>()
        / pcm.len() as f64)
        .sqrt();
    assert!(
        warmup_rms < 0.05 * sig_rms,
        "warmup frame RMS {warmup_rms:.1} vs signal RMS {sig_rms:.1}"
    );
    // Steady-state reconstruction: compare hop f-1 of the input to
    // decoded frame f. Skip the final flush frame's tail.
    let ratio = err_to_signal_rms(&pcm, &decoded[FRAME_LEN..]);
    eprintln!("mono multitone err/sig RMS = {ratio:.5}");
    // At 128 kbps the budget cannot carry the full leakage skirts of
    // off-bin tones; ~-32 dB overall reconstruction is this
    // encoder's honest operating point for dense tonal content.
    assert!(
        ratio < 0.03,
        "error-to-signal RMS {ratio:.5} exceeds the 3% tolerance"
    );
}

#[test]
fn stereo_multitone_roundtrips_within_tolerance() {
    let n = 8 * FRAME_LEN;
    let pcm = multitone(n, 2);
    let config = EncoderConfig {
        sample_rate: 48_000,
        channels: 2,
        bitrate: 192_000,
    };
    let decoded = roundtrip(&pcm, config);
    assert_eq!(decoded.len(), (n + FRAME_LEN) * 2);
    let ratio = err_to_signal_rms(&pcm, &decoded[FRAME_LEN * 2..]);
    eprintln!("stereo multitone err/sig RMS = {ratio:.5}");
    assert!(
        ratio < 0.03,
        "error-to-signal RMS {ratio:.5} exceeds the 3% tolerance"
    );
}

#[test]
fn low_rate_mono_stays_within_budget_and_decodes() {
    // 8 kHz, 16 kbps: the rate loop must engage. Every frame respects
    // the per-frame budget, and the stream still decodes end to end.
    let n = 6 * FRAME_LEN;
    let pcm = multitone(n, 1);
    let config = EncoderConfig {
        sample_rate: 8_000,
        channels: 1,
        bitrate: 16_000,
    };
    let mut enc = StreamEncoder::new(config).unwrap();
    let stream = enc.encode_all(&pcm).unwrap();

    // Walk the ADTS frames and check sizes.
    let budget_bytes = (16_000u64 * 1024 / 8_000 / 8) as usize;
    let mut pos = 0;
    let mut frames = 0;
    while pos < stream.len() {
        let (hdr, _) = AdtsHeader::parse(&stream[pos..]).expect("frame parses");
        assert!(
            (hdr.aac_frame_length as usize) <= budget_bytes.max(16 + 7),
            "frame {frames} is {} bytes, budget {budget_bytes}",
            hdr.aac_frame_length
        );
        pos += hdr.aac_frame_length as usize;
        frames += 1;
    }
    assert_eq!(pos, stream.len());
    assert_eq!(frames, 7); // 6 hops + flush

    let mut dec = StreamDecoder::new();
    let decoded = dec.decode_all(&stream).expect("decodes");
    assert_eq!(decoded.len(), 7);
    // At 16 kbps quality is coarse; assert the reconstruction still
    // correlates strongly with the input rather than pinning a tight
    // RMS bound.
    let mut out = Vec::new();
    for f in &decoded {
        out.extend_from_slice(&f.pcm);
    }
    let ratio = err_to_signal_rms(&pcm, &out[FRAME_LEN..]);
    eprintln!("8 kHz / 16 kbps err/sig RMS = {ratio:.5}");
    assert!(ratio < 0.35, "reconstruction lost the signal: {ratio:.5}");
}

#[test]
fn every_supported_sample_rate_produces_a_decodable_stream() {
    // One hop of content per rate; every Table 1.18 ADTS rate with a
    // §4.5.4 scalefactor-band table (index 0..=11) must yield a
    // stream our decoder accepts with the right sample rate. Index
    // 12 (7350 Hz) has no long-window band table and is rejected at
    // config time (7350 Hz content conventionally ships under index
    // 11 — see the staged corpus notes on the 8 kHz fixture).
    assert!(StreamEncoder::new(EncoderConfig {
        sample_rate: 7_350,
        channels: 1,
        bitrate: 96_000,
    })
    .is_err());
    for &rate in &[
        96_000u32, 88_200, 64_000, 48_000, 44_100, 32_000, 24_000, 22_050, 16_000, 12_000, 11_025,
        8_000,
    ] {
        let pcm = multitone(FRAME_LEN, 1);
        let config = EncoderConfig {
            sample_rate: rate,
            channels: 1,
            bitrate: 96_000,
        };
        let mut enc = StreamEncoder::new(config).unwrap();
        let stream = enc.encode_all(&pcm).unwrap();
        let (hdr, _) = AdtsHeader::parse(&stream).unwrap();
        assert_eq!(hdr.sample_rate(), rate, "header rate mismatch at {rate}");
        let mut dec = StreamDecoder::new();
        let frames = dec.decode_all(&stream).expect("decodes");
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].sample_rate, rate);
    }
}

#[test]
fn full_scale_input_does_not_overflow() {
    // A full-scale square-ish signal exercises the quantizer ceiling
    // and the PCM clamp; the round-trip must not error and the output
    // must stay bounded.
    let n = 4 * FRAME_LEN;
    let pcm: Vec<i16> = (0..n)
        .map(|i| if (i / 64) % 2 == 0 { 32_767 } else { -32_768 })
        .collect();
    let config = EncoderConfig {
        sample_rate: 44_100,
        channels: 1,
        bitrate: 256_000,
    };
    let decoded = roundtrip(&pcm, config);
    assert_eq!(decoded.len(), n + FRAME_LEN);
    let ratio = err_to_signal_rms(&pcm, &decoded[FRAME_LEN..]);
    eprintln!("full-scale square err/sig RMS = {ratio:.5}");
    assert!(ratio < 0.25, "square-wave reconstruction ratio {ratio:.5}");
}

#[test]
fn hop_by_hop_api_matches_encode_all() {
    let n = 3 * FRAME_LEN + 500; // non-multiple tail
    let pcm = multitone(n, 2);
    let config = EncoderConfig {
        sample_rate: 44_100,
        channels: 2,
        bitrate: 128_000,
    };
    let mut a = StreamEncoder::new(config).unwrap();
    let one_shot = a.encode_all(&pcm).unwrap();

    let mut b = StreamEncoder::new(config).unwrap();
    let mut manual = Vec::new();
    for chunk in pcm.chunks(FRAME_LEN * 2) {
        manual.extend_from_slice(&b.encode_frame(chunk).unwrap());
    }
    manual.extend_from_slice(&b.finish().unwrap());
    assert_eq!(one_shot, manual);
}

// ---- §4.6.8.1 M/S joint stereo (encode side) ----

#[test]
fn identical_channels_engage_ms_and_shrink_the_stream() {
    // L == R: every band's side channel is exactly zero, so the M/S
    // decision must flag (ms_mask_present = 2) and the stereo stream
    // should cost little more than the mono encode of the same
    // signal (the side channel is all-ZERO_HCB sections).
    let n = 6 * FRAME_LEN;
    let mono = multitone(n, 1);
    let mut stereo = Vec::with_capacity(n * 2);
    for &s in &mono {
        stereo.push(s);
        stereo.push(s);
    }

    let mut enc_m = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: 1,
        bitrate: 128_000,
    })
    .unwrap();
    let mono_stream = enc_m.encode_all(&mono).unwrap();

    let mut enc_s = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: 2,
        bitrate: 128_000,
    })
    .unwrap();
    let stereo_stream = enc_s.encode_all(&stereo).unwrap();

    eprintln!(
        "mono {} bytes, identical-channel stereo {} bytes",
        mono_stream.len(),
        stereo_stream.len()
    );
    assert!(
        stereo_stream.len() < mono_stream.len() * 13 / 10,
        "M/S should make identical-channel stereo cost <1.3x mono: {} vs {}",
        stereo_stream.len(),
        mono_stream.len()
    );

    // And the reconstruction must keep the channels (nearly)
    // identical and close to the input.
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stereo_stream).unwrap();
    let mut out = Vec::new();
    for f in &frames {
        out.extend_from_slice(&f.pcm);
    }
    let ratio = err_to_signal_rms(&stereo, &out[FRAME_LEN * 2..]);
    eprintln!("identical-channel stereo err/sig RMS = {ratio:.5}");
    assert!(ratio < 0.03, "M/S round-trip ratio {ratio:.5}");
    // Channel coherence: decoded L and R stay near-identical.
    let steady = &out[FRAME_LEN * 2..];
    let max_lr_diff = steady
        .chunks_exact(2)
        .map(|p| (i32::from(p[0]) - i32::from(p[1])).abs())
        .max()
        .unwrap();
    assert!(
        max_lr_diff <= 2,
        "identical input channels decoded {max_lr_diff} LSB apart"
    );
}

#[test]
fn phase_inverted_channels_roundtrip_through_ms() {
    // R == −L: the mid channel is exactly zero; M/S concentrates all
    // energy in the side channel. Round-trip must preserve the
    // inversion.
    let n = 4 * FRAME_LEN;
    let mono = multitone(n, 1);
    let mut stereo = Vec::with_capacity(n * 2);
    for &s in &mono {
        stereo.push(s);
        stereo.push(s.saturating_neg());
    }
    let decoded = roundtrip(
        &stereo,
        EncoderConfig {
            sample_rate: 44_100,
            channels: 2,
            bitrate: 128_000,
        },
    );
    let ratio = err_to_signal_rms(&stereo, &decoded[FRAME_LEN * 2..]);
    eprintln!("phase-inverted stereo err/sig RMS = {ratio:.5}");
    assert!(ratio < 0.03, "anti-correlated M/S ratio {ratio:.5}");
}

#[test]
fn uncorrelated_channels_still_roundtrip() {
    // Fully independent channels: the M/S decision should mostly
    // stay off and quality must not regress against the joint path.
    let n = 4 * FRAME_LEN;
    let mut stereo = Vec::with_capacity(n * 2);
    for i in 0..n {
        let t = i as f64;
        stereo.push((9000.0 * (0.033 * t).sin()) as i16);
        stereo.push((9000.0 * (0.171 * t + 0.9).sin()) as i16);
    }
    let decoded = roundtrip(
        &stereo,
        EncoderConfig {
            sample_rate: 44_100,
            channels: 2,
            bitrate: 160_000,
        },
    );
    let ratio = err_to_signal_rms(&stereo, &decoded[FRAME_LEN * 2..]);
    eprintln!("uncorrelated stereo err/sig RMS = {ratio:.5}");
    assert!(ratio < 0.03, "independent-channel ratio {ratio:.5}");
}

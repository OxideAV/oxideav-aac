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

// ---- §4.6.11.3.2 block switching (encode side) ----

/// Walk an ADTS stream of mono SCE frames and return each frame's
/// `window_sequence` as parsed off the wire.
fn window_sequences(stream: &[u8]) -> Vec<oxideav_aac::ics_info::WindowSequence> {
    use oxideav_aac::ics_body::IcsBody;
    use oxideav_core::bits::BitReader;

    let mut seqs = Vec::new();
    let mut pos = 0usize;
    while pos < stream.len() {
        let (hdr, off) = AdtsHeader::parse(&stream[pos..]).expect("frame parses");
        let payload = &stream[pos + off..pos + hdr.aac_frame_length as usize];
        let mut br = BitReader::new(payload);
        // 3-bit id_syn_ele (SCE = 0) + 4-bit element_instance_tag.
        let id = br.read_u32(3).unwrap();
        assert_eq!(id, 0, "expected SCE");
        let _tag = br.read_u32(4).unwrap();
        let body = IcsBody::parse(&mut br, 2, hdr.sampling_frequency_index, false)
            .expect("ics body parses");
        seqs.push(body.ics_info.unwrap().window_sequence);
        pos += hdr.aac_frame_length as usize;
    }
    seqs
}

#[test]
fn transient_input_switches_to_short_windows() {
    use oxideav_aac::ics_info::WindowSequence as Ws;

    // Quiet tone for 3 hops, then a hard percussive attack in hop 3.
    let n = 6 * FRAME_LEN;
    let pcm: Vec<i16> = (0..n)
        .map(|i| {
            let quiet = (300.0 * (0.02 * i as f64).sin()) as i16;
            if (3 * FRAME_LEN..3 * FRAME_LEN + 400).contains(&i) {
                // Sharp burst.
                if i % 2 == 0 {
                    28_000
                } else {
                    -28_000
                }
            } else {
                quiet
            }
        })
        .collect();
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: 1,
        bitrate: 128_000,
    })
    .unwrap();
    let stream = enc.encode_all(&pcm).unwrap();
    let seqs = window_sequences(&stream);
    eprintln!("window sequences: {seqs:?}");

    // Hop 3 carries the attack, so frame 3 must be the LONG_START
    // lead-in and frame 4 EIGHT_SHORT, exiting through LONG_STOP.
    assert_eq!(seqs[3], Ws::LongStart, "lead-in frame");
    assert_eq!(seqs[4], Ws::EightShort, "transient frame");
    assert_eq!(seqs[5], Ws::LongStop, "exit frame");
    // Steady frames stay long.
    assert_eq!(seqs[0], Ws::OnlyLong);
    assert_eq!(seqs[1], Ws::OnlyLong);
    // The whole stream still round-trips.
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).expect("decodes");
    let mut out = Vec::new();
    for f in &frames {
        out.extend_from_slice(&f.pcm);
    }
    let ratio = err_to_signal_rms(&pcm, &out[FRAME_LEN..]);
    eprintln!("transient round-trip err/sig RMS = {ratio:.5}");
    assert!(ratio < 0.08, "transient reconstruction ratio {ratio:.5}");
}

#[test]
fn steady_tonal_input_never_switches() {
    use oxideav_aac::ics_info::WindowSequence as Ws;

    let n = 5 * FRAME_LEN;
    let pcm = multitone(n, 1);
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: 1,
        bitrate: 128_000,
    })
    .unwrap();
    let stream = enc.encode_all(&pcm).unwrap();
    let seqs = window_sequences(&stream);
    assert!(
        seqs.iter().all(|&s| s == Ws::OnlyLong),
        "steady tone must stay ONLY_LONG: {seqs:?}"
    );
}

#[test]
fn stereo_transient_stream_roundtrips() {
    // The same attack in a CPE stream (M/S disabled on short frames)
    // must still decode end to end.
    let n = 5 * FRAME_LEN;
    let mut pcm = Vec::with_capacity(n * 2);
    for i in 0..n {
        let base = (400.0 * (0.025 * i as f64).sin()) as i16;
        let burst = if (2 * FRAME_LEN..2 * FRAME_LEN + 300).contains(&i) {
            if i % 2 == 0 {
                24_000
            } else {
                -24_000
            }
        } else {
            0
        };
        let v = base.saturating_add(burst);
        pcm.push(v);
        pcm.push(v);
    }
    let decoded = roundtrip(
        &pcm,
        EncoderConfig {
            sample_rate: 48_000,
            channels: 2,
            bitrate: 192_000,
        },
    );
    let ratio = err_to_signal_rms(&pcm, &decoded[FRAME_LEN * 2..]);
    eprintln!("stereo transient err/sig RMS = {ratio:.5}");
    assert!(ratio < 0.08, "stereo transient ratio {ratio:.5}");
}

// ---- Transcode stability over the staged fixture corpus ----

#[test]
fn fixture_transcode_preserves_the_signal() {
    // Decode a staged real-encoder fixture to PCM, re-encode it with
    // our encoder, decode again, and compare the two PCM signals
    // (accounting for the 1024-sample encoder delay). This exercises
    // the encoder on dense real-world-like content — every §4.5.4
    // band populated, window switching, both channels — rather than
    // synthetic tones. Skipped when docs/ is absent.
    let cases: [(&str, u32, u8, u32); 3] = [
        ("aac-lc-stereo-44100-128kbps-adts", 44_100, 2, 128_000),
        ("aac-lc-mono-44100-64kbps-adts", 44_100, 1, 96_000),
        ("aac-lc-chirp-windows", 44_100, 1, 128_000),
    ];
    let root = std::path::PathBuf::from("../../docs/audio/aac/fixtures");
    for (name, rate, channels, bitrate) in cases {
        let input = root.join(name).join("input.aac");
        let Ok(bytes) = std::fs::read(&input) else {
            eprintln!("skip: fixture {name} not present");
            continue;
        };
        let mut dec = StreamDecoder::new();
        let Ok(frames) = dec.decode_all(&bytes) else {
            panic!("{name}: fixture must decode");
        };
        let mut pcm = Vec::new();
        for f in &frames {
            pcm.extend_from_slice(&f.pcm);
        }
        assert_eq!(frames[0].sample_rate, rate, "{name}");

        let mut enc = StreamEncoder::new(EncoderConfig {
            sample_rate: rate,
            channels,
            bitrate,
        })
        .unwrap();
        let stream = enc.encode_all(&pcm).unwrap();
        let mut dec2 = StreamDecoder::new();
        let frames2 = dec2.decode_all(&stream).expect("re-encoded stream decodes");
        let mut pcm2 = Vec::new();
        for f in &frames2 {
            pcm2.extend_from_slice(&f.pcm);
        }
        let ch = channels as usize;
        assert_eq!(pcm2.len(), pcm.len() + FRAME_LEN * ch);
        let ratio = err_to_signal_rms(&pcm, &pcm2[FRAME_LEN * ch..]);
        eprintln!("{name}: transcode err/sig RMS = {ratio:.5}");
        assert!(
            ratio < 0.02,
            "{name}: transcode ratio {ratio:.5} exceeds tolerance"
        );
    }
}

// ---- §4.6.9 TNS (encode side) ----

/// Count transmitted TNS filters across a mono SCE ADTS stream.
fn count_tns_filters(stream: &[u8]) -> usize {
    use oxideav_aac::ics_body::IcsBody;
    use oxideav_core::bits::BitReader;

    let mut count = 0usize;
    let mut pos = 0usize;
    while pos < stream.len() {
        let (hdr, off) = AdtsHeader::parse(&stream[pos..]).expect("frame parses");
        let payload = &stream[pos + off..pos + hdr.aac_frame_length as usize];
        let mut br = BitReader::new(payload);
        let id = br.read_u32(3).unwrap();
        assert_eq!(id, 0, "expected SCE");
        let _tag = br.read_u32(4).unwrap();
        let body = IcsBody::parse(&mut br, 2, hdr.sampling_frequency_index, false)
            .expect("ics body parses");
        if let Some(tns) = &body.tns_data {
            count += tns.windows.iter().map(|w| w.filters.len()).sum::<usize>();
        }
        pos += hdr.aac_frame_length as usize;
    }
    count
}

/// Deterministic burst-and-decay noise: a fresh noise burst every
/// 1024 samples decaying with time constant `tau` — a strongly
/// non-flat temporal envelope inside every long analysis window
/// (the §4.6.9.1 case TNS exists for), while the burst-to-burst
/// period keeps the block-switching detector quiet (each new burst
/// jumps only ~2.3x over the previous burst's retained maximum).
fn burst_decay_noise(n: usize, amp: f64, tau: f64, seed: u32) -> Vec<i16> {
    let mut state = seed;
    (0..n)
        .map(|i| {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let noise = (state as i32) as f64 / 2_147_483_648.0;
            let phase = (i % FRAME_LEN) as f64;
            (amp * (-phase / tau).exp() * noise) as i16
        })
        .collect()
}

#[test]
fn burst_decay_noise_engages_tns_and_roundtrips() {
    let n = 6 * FRAME_LEN;
    let pcm = burst_decay_noise(n, 8_000.0, 300.0, 0x5EED_5EED);
    let config = EncoderConfig {
        sample_rate: 44_100,
        channels: 1,
        bitrate: 128_000,
    };
    let mut enc = StreamEncoder::new(config).unwrap();
    let stream = enc.encode_all(&pcm).unwrap();

    // The burst-decay envelope must clear the temporal gate and the
    // prediction-gain threshold on a substantial number of frames.
    let filters = count_tns_filters(&stream);
    eprintln!("TNS filters across the burst-decay stream: {filters}");
    assert!(
        filters >= 3,
        "burst-decay noise should engage TNS, got {filters} filters"
    );

    // The stream must still round-trip through our decoder (whose
    // §4.6.9.3 synthesis pass inverts the applied analysis filter).
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).expect("TNS stream decodes");
    let mut out = Vec::new();
    for f in &frames {
        out.extend_from_slice(&f.pcm);
    }
    let ratio = err_to_signal_rms(&pcm, &out[FRAME_LEN..]);
    eprintln!("burst-decay TNS round-trip err/sig RMS = {ratio:.5}");

    // With TNS disabled the same content carries no filters.
    let mut enc_off = StreamEncoder::new(config).unwrap();
    enc_off.set_tns(false);
    let stream_off = enc_off.encode_all(&pcm).unwrap();
    assert_eq!(count_tns_filters(&stream_off), 0);
    // A/B: the TNS path must not degrade the reconstruction (noise
    // content codes coarsely at any setting; TNS re-shapes the error
    // in time rather than shrinking it, so parity is the bar).
    let mut dec_off = StreamDecoder::new();
    let frames_off = dec_off.decode_all(&stream_off).expect("TNS-off decodes");
    let mut out_off = Vec::new();
    for f in &frames_off {
        out_off.extend_from_slice(&f.pcm);
    }
    let ratio_off = err_to_signal_rms(&pcm, &out_off[FRAME_LEN..]);
    eprintln!("burst-decay TNS-off err/sig RMS = {ratio_off:.5}");
    assert!(
        ratio < ratio_off * 1.15,
        "TNS must not degrade the round-trip: {ratio:.5} vs {ratio_off:.5} without"
    );
}

#[test]
fn steady_tonal_content_stays_tns_free() {
    // The temporal gate must keep TNS off steady tonal windows even
    // though their leakage skirts show spectral prediction gain.
    let n = 5 * FRAME_LEN;
    let pcm = multitone(n, 1);
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: 1,
        bitrate: 128_000,
    })
    .unwrap();
    let stream = enc.encode_all(&pcm).unwrap();
    assert_eq!(
        count_tns_filters(&stream),
        0,
        "steady multitone must not engage TNS"
    );
}

// ---- §4.6.13 PNS (encode side) ----

/// Deterministic pseudo-noise on the ±`amp` axis (32-bit LCG).
fn noise_pcm(n: usize, amp: f64, seed: u32) -> Vec<i16> {
    let mut state = seed;
    (0..n)
        .map(|_| {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            ((state as i32) as f64 / 2_147_483_648.0 * amp) as i16
        })
        .collect()
}

/// Count NOISE_HCB (13) bands across a mono SCE ADTS stream.
fn count_noise_bands(stream: &[u8]) -> usize {
    use oxideav_aac::ics_body::IcsBody;
    use oxideav_core::bits::BitReader;

    let mut count = 0usize;
    let mut pos = 0usize;
    while pos < stream.len() {
        let (hdr, off) = AdtsHeader::parse(&stream[pos..]).expect("frame parses");
        let payload = &stream[pos + off..pos + hdr.aac_frame_length as usize];
        let mut br = BitReader::new(payload);
        let id = br.read_u32(3).unwrap();
        assert_eq!(id, 0, "expected SCE");
        let _tag = br.read_u32(4).unwrap();
        let body = IcsBody::parse(&mut br, 2, hdr.sampling_frequency_index, false)
            .expect("ics body parses");
        for group in &body.section_data.sfb_cb {
            count += group.iter().filter(|&&cb| cb == 13).count();
        }
        pos += hdr.aac_frame_length as usize;
    }
    count
}

#[test]
fn noise_input_engages_pns_and_preserves_energy() {
    let n = 6 * FRAME_LEN;
    let pcm = noise_pcm(n, 8_000.0, 0x1234_5678);
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: 1,
        bitrate: 64_000,
    })
    .unwrap();
    enc.set_pns(true);
    let stream = enc.encode_all(&pcm).unwrap();

    // Dense noise must trigger PNS on a substantial number of bands.
    let noise_bands = count_noise_bands(&stream);
    eprintln!("PNS bands across the noise stream: {noise_bands}");
    assert!(
        noise_bands > 50,
        "dense noise should engage PNS broadly, got {noise_bands} bands"
    );

    // The reconstruction is *different noise* (the §4.6.13 generator
    // phase is the decoder's own), so compare energy, not samples:
    // per-frame RMS must track the input RMS.
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).expect("decodes");
    let mut out = Vec::new();
    for f in &frames {
        out.extend_from_slice(&f.pcm);
    }
    let rms = |s: &[i16]| -> f64 {
        (s.iter().map(|&v| f64::from(v) * f64::from(v)).sum::<f64>() / s.len() as f64).sqrt()
    };
    for f in 1..(n / FRAME_LEN) {
        let in_rms = rms(&pcm[(f - 1) * FRAME_LEN..f * FRAME_LEN]);
        let out_rms = rms(&out[f * FRAME_LEN..(f + 1) * FRAME_LEN]);
        let rel = (out_rms - in_rms).abs() / in_rms;
        assert!(
            rel < 0.25,
            "frame {f}: decoded RMS {out_rms:.0} vs input {in_rms:.0} ({rel:.3})"
        );
    }
}

#[test]
fn default_config_never_emits_pns() {
    // PNS emission is opt-in; the default encoder must never emit
    // NOISE_HCB bands, even on noise content.
    let n = 4 * FRAME_LEN;
    for pcm in [multitone(n, 1), noise_pcm(n, 8_000.0, 0x0bad_cafe)] {
        let mut enc = StreamEncoder::new(EncoderConfig {
            sample_rate: 44_100,
            channels: 1,
            bitrate: 128_000,
        })
        .unwrap();
        let stream = enc.encode_all(&pcm).unwrap();
        assert_eq!(count_noise_bands(&stream), 0);
    }
}

#[test]
fn pns_shrinks_noise_frames() {
    // The same noise content coded at generous bitrate: with PNS the
    // stream should stay far below the budget ceiling (noise bands
    // cost a handful of bits each instead of dozens of Huffman
    // coefficients).
    let n = 4 * FRAME_LEN;
    let pcm = noise_pcm(n, 8_000.0, 0x9e37_79b9);
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44_100,
        channels: 1,
        bitrate: 256_000,
    })
    .unwrap();
    enc.set_pns(true);
    let stream = enc.encode_all(&pcm).unwrap();
    let budget = 256_000usize * 1024 / 44_100 / 8;
    let per_frame = stream.len() / 5;
    eprintln!(
        "noise stream: {} bytes over 5 frames (avg {per_frame}/frame, budget {budget})",
        stream.len()
    );
    assert!(
        per_frame < budget / 2,
        "PNS should leave noise frames well under budget: {per_frame} vs {budget}"
    );
}

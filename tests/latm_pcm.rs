//! End-to-end LATM/LOAS → PCM validation against the
//! `docs/audio/aac/fixtures/aac-latm-stream` fixture.
//!
//! [`oxideav_aac::latm::LoasDecoder`] walks the LOAS `AudioSyncStream()`
//! byte buffer, recovers each `AudioMuxElement` access unit, and drives
//! the payload's §4.4.2.1 `raw_data_block()` through the shared
//! [`oxideav_aac::decode::StreamDecoder`] core using the configuration
//! the LATM `AudioSpecificConfig` carries. This driver confirms that
//! transport path reconstructs the same PCM the bare ADTS path does.
//!
//! The `aac-latm-stream` fixture is a 440 Hz stereo sine encoded with
//! `-c:a aac -b:a 96k`; the encoder placed a §4.6.13 PNS (NOISE_HCB)
//! band in the upper spectrum, so — exactly as for the ADTS PNS
//! fixtures in `pcm_byte_exact.rs` — the per-coefficient noise *phase*
//! is spec-undefined (§4.6.13.3) and byte-exactness against any one
//! reference decoder is impossible. The comparison is therefore in the
//! §8 PCM-RMS domain: the reconstructed signal must track the reference
//! to a small fraction of its own RMS, and the decode must run to
//! completion over the whole multiplex.
//!
//! `oxideav-aac` is its own repository and `docs/` lives in the
//! workspace umbrella, so the test is skipped (logged, success) when the
//! fixture is absent — CI stays green in the standalone layout.

use std::fs;
use std::path::PathBuf;

use oxideav_aac::latm::LoasDecoder;

/// Read the `data` chunk of a 16-bit WAV as interleaved `i16` samples.
fn read_wav_s16(path: &PathBuf) -> Option<Vec<i16>> {
    let d = fs::read(path).ok()?;
    let mut i = 12; // skip "RIFF" + size + "WAVE"
    while i + 8 <= d.len() {
        let cid = &d[i..i + 4];
        let sz = u32::from_le_bytes([d[i + 4], d[i + 5], d[i + 6], d[i + 7]]) as usize;
        let body_start = i + 8;
        if cid == b"data" {
            let end = (body_start + sz).min(d.len());
            return Some(
                d[body_start..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            );
        }
        i = body_start + sz + (sz & 1);
    }
    None
}

fn fixture_dir() -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures/aac-latm-stream")
}

/// Decode the LATM fixture to one interleaved-PCM vector.
fn decode_latm() -> Option<(Vec<i16>, usize, u32)> {
    let bytes = fs::read(fixture_dir().join("input.latm")).ok()?;
    let mut dec = LoasDecoder::new();
    let frames = dec.decode_all(&bytes).expect("LATM/LOAS decode");
    let channels = frames
        .iter()
        .find(|f| f.channels > 0)
        .map_or(0, |f| f.channels);
    let sample_rate = frames.first().map_or(0, |f| f.sample_rate);
    let mut pcm = Vec::new();
    for f in &frames {
        pcm.extend_from_slice(&f.pcm);
    }
    Some((pcm, channels, sample_rate))
}

#[test]
fn latm_stream_decodes_to_reference_pcm_in_rms_domain() {
    let (Some((got, channels, sample_rate)), Some(exp)) = (
        decode_latm(),
        read_wav_s16(&fixture_dir().join("expected.wav")),
    ) else {
        eprintln!("skip aac-latm-stream: fixture unavailable");
        return;
    };

    // The fixture is stereo at 44.1 kHz; the multiplex carried a single
    // program / single layer, so the whole stream decodes to one
    // continuous PCM run.
    assert_eq!(channels, 2, "LATM fixture is stereo");
    assert_eq!(sample_rate, 44_100, "LATM fixture is 44.1 kHz");
    assert_eq!(
        got.len(),
        exp.len(),
        "decoded sample count must match the reference WAV"
    );

    let n = got.len();
    assert!(n > 0, "decode produced no samples");

    let mut sq_err = 0.0f64;
    let mut ref_energy = 0.0f64;
    for (&g, &e) in got.iter().zip(exp.iter()) {
        let d = f64::from(g) - f64::from(e);
        sq_err += d * d;
        ref_energy += f64::from(e) * f64::from(e);
    }
    let rms_err = (sq_err / n as f64).sqrt();
    let ref_rms = (ref_energy / n as f64).sqrt();
    assert!(ref_rms > 0.0, "reference is silent");
    let ratio = rms_err / ref_rms;

    // The only spec-undefined divergence is the §4.6.13.3 PNS phase
    // (one upper band) plus the ≤1-LSB IMDCT rounding; both keep the
    // error well under 2 % of the signal RMS.
    assert!(
        ratio < 0.02,
        "LATM PCM RMS error ratio {ratio:.5} exceeds 0.02"
    );
}

/// The LATM path must reconstruct the *same* PCM as the bare
/// `StreamDecoder` core would for the recovered access units — i.e. the
/// transport wrapper adds no divergence of its own. We verify this by
/// decoding the recovered raw-data-blocks both through `LoasDecoder` and
/// by hand-feeding the same payloads to a `StreamDecoder`.
#[test]
fn latm_path_matches_manual_raw_data_block_decode() {
    use oxideav_aac::decode::StreamDecoder;
    use oxideav_aac::latm::AudioSyncStream;

    let Ok(bytes) = fs::read(fixture_dir().join("input.latm")) else {
        eprintln!("skip aac-latm-stream: fixture unavailable");
        return;
    };

    // Path A: the high-level driver.
    let mut loas = LoasDecoder::new();
    let driver_frames = loas.decode_all(&bytes).expect("LoasDecoder");

    // Path B: walk the sync stream by hand, feed each payload's
    // raw_data_block() to a StreamDecoder configured from the layer ASC.
    let mut manual = StreamDecoder::new();
    let mut manual_frames = Vec::new();
    let mut walker = AudioSyncStream::new(&bytes);
    while let Some(frame) = walker.next_frame().expect("sync frame") {
        for payload in &frame.element.payloads {
            let layer = frame
                .element
                .config
                .layer(payload.stream_id)
                .expect("layer config");
            let asc = &layer.effective_asc;
            let decoded = manual
                .decode_raw_data_block(
                    asc.aot,
                    asc.sampling_frequency_index,
                    asc.sample_rate,
                    1,
                    &payload.data,
                )
                .expect("raw_data_block decode");
            manual_frames.push(decoded);
        }
    }

    assert_eq!(driver_frames.len(), manual_frames.len());
    for (a, b) in driver_frames.iter().zip(manual_frames.iter()) {
        // Both paths seed the PNS generator the same way (one
        // StreamDecoder per stream, fresh), so the PCM is bit-identical.
        assert_eq!(a.pcm, b.pcm, "LATM driver diverged from manual decode");
        assert_eq!(a.channels, b.channels);
        assert_eq!(a.sample_rate, b.sample_rate);
    }
}

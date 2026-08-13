//! Encoder-side §4.4.6.3 `pulse_data()` emission, end to end: PCM
//! whose quantized spectrum carries outlier-over-floor bands encodes
//! with pulse records (kept only where the measured stream is
//! smaller), the wire parses, and the stream round-trips through the
//! crate's own decoder — the §4.6.3.3 fix-up restores the identical
//! quantized spectrum, so the reconstruction cost of the tool is
//! exactly zero.

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder, FRAME_LEN};

/// Per-frame pulse records of a mono SCE ADTS stream: `Some(pd)`
/// when the frame's channel body carries `pulse_data()`.
fn pulse_records(stream: &[u8]) -> Vec<Option<oxideav_aac::pulse_data::PulseData>> {
    use oxideav_aac::ics_body::IcsBody;
    use oxideav_core::bits::BitReader;

    let mut out = Vec::new();
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
        assert_eq!(body.pulse_data_present, body.pulse_data.is_some());
        out.push(body.pulse_data);
        pos += hdr.aac_frame_length as usize;
    }
    out
}

/// Error-to-signal RMS ratio between `a` and `b` (same length).
fn err_to_signal_rms(a: &[i16], b: &[i16]) -> f64 {
    assert_eq!(a.len(), b.len());
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

/// A steady signal shaped for the pulse tool: a strong low tone
/// (the frame peak), a moderate tone centred in a wide high band
/// (its masking target lands in ESC-outlier territory), and a
/// deterministic dense floor around it (the band's cheap residual).
fn pulse_prone_signal(n: usize) -> Vec<i16> {
    let mut seed = 0x1234_5678u32;
    (0..n)
        .map(|i| {
            let t = i as f64;
            // 48 kHz frame geometry: MDCT bin k sits near
            // (k + 0.5)·fs/2048 Hz. Bin ~336 is inside the 32-line
            // band 29 (lines 320..352).
            let strong = 20_000.0 * (2.0 * std::f64::consts::PI * 1000.0 / 48_000.0 * t).sin();
            let outlier = 3_300.0 * (2.0 * std::f64::consts::PI * 7_890.0 / 48_000.0 * t).sin();
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let noise = ((seed >> 16) as f64 / 32_768.0 - 1.0) * 220.0;
            (strong + outlier + noise).round() as i16
        })
        .collect()
}

#[test]
fn pulse_bearing_stream_roundtrips_and_parses() {
    let pcm = pulse_prone_signal(16 * FRAME_LEN);
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 48_000,
        channels: 1,
        bitrate: 96_000,
    })
    .unwrap();
    let stream = enc.encode_all(&pcm).unwrap();

    // At least one frame must have chosen the measured pulse
    // variant, and every record must be Table 4.7 legal.
    let records = pulse_records(&stream);
    let carriers: Vec<_> = records.iter().flatten().collect();
    eprintln!(
        "pulse frames: {} of {} ({:?})",
        carriers.len(),
        records.len(),
        carriers.first()
    );
    assert!(
        !carriers.is_empty(),
        "outlier-over-floor content must engage the pulse tool"
    );
    for pd in &carriers {
        assert!((1..=4).contains(&pd.pulses.len()));
        assert!(pd.pulse_start_sfb <= 0x3f);
        for p in &pd.pulses {
            assert!(p.offset <= 0x1f && p.amp <= 0x0f);
        }
    }

    // The stream round-trips through the crate's own decoder.
    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).expect("pulse stream decodes");
    let mut out = Vec::new();
    for f in &frames {
        out.extend_from_slice(&f.pcm);
    }
    let ratio = err_to_signal_rms(&pcm, &out[FRAME_LEN..]);
    eprintln!("pulse round-trip err/sig RMS = {ratio:.5}");
    assert!(ratio < 0.05, "pulse stream reconstruction ratio {ratio:.5}");
}

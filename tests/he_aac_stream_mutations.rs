//! HE-AAC encoder output under stream damage and random access:
//!
//! * a deterministic bit-flip / truncation battery over the emitted
//!   ADTS stream must never panic the decoder (every outcome is a
//!   `Result`), and
//! * joining the stream at an `sbr_header()` frame (the encoder
//!   repeats the header every N frames and frequency-codes that
//!   frame's first envelope / noise floor) decodes to the same audio
//!   as the uninterrupted decode once the SBR state has settled.

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::he_aac_encoder::{HeAacConfig, HeAacEncoder, HE_FRAME_LEN};
use oxideav_aac::latm_writer::AdtsPayloads;
use oxideav_aac::sbr_qmf::EncoderAnalysisQmf;

fn material(frames: usize) -> Vec<i16> {
    let n = frames * HE_FRAME_LEN;
    let mut seed = 0x1357_9bdfu32;
    (0..n)
        .map(|i| {
            let t = i as f64 / 44_100.0;
            let mut v = 0.0;
            for &(f, a) in &[
                (440.0, 5000.0),
                (2900.0, 2500.0),
                (7100.0, 1600.0),
                (11800.0, 1000.0),
            ] {
                v += a * (2.0 * std::f64::consts::PI * f * t).sin();
            }
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            v += (f64::from(seed >> 8) / f64::from(1u32 << 24) - 0.5) * 1800.0;
            // A burst every 7 frames keeps the grid election busy.
            if (i / HE_FRAME_LEN) % 7 == 3 && (i % HE_FRAME_LEN) < 300 {
                v += 9000.0 * (0.3 * i as f64).sin();
            }
            v.clamp(-32768.0, 32767.0) as i16
        })
        .collect()
}

#[test]
fn damaged_streams_never_panic_the_decoder() {
    let pcm = material(12);
    let cfg = HeAacConfig {
        sbr_crc: true,
        ..HeAacConfig::new(44_100, 1, 40_000)
    };
    let stream = HeAacEncoder::new(cfg).unwrap().encode_all(&pcm).unwrap();
    let frame_offsets: Vec<usize> = {
        let mut offs = Vec::new();
        let mut pos = 0usize;
        for (h, _) in AdtsPayloads::new(&stream) {
            offs.push(pos);
            pos += usize::from(h.aac_frame_length);
        }
        offs
    };
    assert_eq!(frame_offsets.len(), 13);

    let mut outcomes = (0usize, 0usize);
    // Bit flips striding through the payloads (headers included).
    for bit in (0..stream.len() * 8).step_by(97) {
        let mut damaged = stream.clone();
        damaged[bit / 8] ^= 1 << (bit % 8);
        let mut dec = StreamDecoder::new();
        match dec.decode_all(&damaged) {
            Ok(_) => outcomes.0 += 1,
            Err(_) => outcomes.1 += 1,
        }
    }
    // Truncations at every 37th byte.
    for cut in (1..stream.len()).step_by(37) {
        let mut dec = StreamDecoder::new();
        let _ = dec.decode_all(&stream[..cut]);
    }
    // Per-frame decode of a stream with one frame's SBR bytes zeroed
    // (CRC mismatch surfaces as an error, never a panic).
    let mut zeroed = stream.clone();
    let f = frame_offsets[5];
    for b in &mut zeroed[f + 12..f + 24] {
        *b = 0;
    }
    let mut dec = StreamDecoder::new();
    let _ = dec.decode_all(&zeroed);
    eprintln!("bit flips: {} decoded, {} rejected", outcomes.0, outcomes.1);
    assert!(outcomes.0 + outcomes.1 > 100);
}

fn band_energies(pcm: &[i16]) -> [f64; 64] {
    let mut bank = EncoderAnalysisQmf::new();
    let mut e = [0.0f64; 64];
    let mono: Vec<f64> = pcm.iter().map(|&v| f64::from(v)).collect();
    for slot in mono.chunks_exact(64) {
        let cols = bank.push_slot(slot).unwrap();
        for (k, v) in cols.iter().enumerate() {
            e[k] += v.norm_sqr();
        }
    }
    e
}

#[test]
fn joining_at_a_header_frame_converges_to_the_full_decode() {
    let pcm = material(24);
    let cfg = HeAacConfig {
        header_interval: 8,
        ..HeAacConfig::new(44_100, 1, 40_000)
    };
    let mut enc = HeAacEncoder::new(cfg).unwrap();
    let stream = enc.encode_all(&pcm).unwrap();
    let k_x = enc.sbr().bands().k_x as usize;
    let k_end = (enc.sbr().bands().k_x + enc.sbr().bands().m) as usize;

    let mut offsets = Vec::new();
    let mut pos = 0usize;
    for (h, _) in AdtsPayloads::new(&stream) {
        offsets.push(pos);
        pos += usize::from(h.aac_frame_length);
    }
    let full = StreamDecoder::new().decode_all(&stream).unwrap();

    // Join at frame 16 (a header frame: 16 % 8 == 0).
    let join = 16usize;
    let tail = StreamDecoder::new()
        .decode_all(&stream[offsets[join]..])
        .expect("decode from a header frame");
    assert_eq!(tail.len(), full.len() - join);
    assert!(tail.iter().all(|f| f.sample_rate == 44_100));

    // Skip the first two joined frames (core overlap + SBR state
    // warm-up), then compare per-band energies over the rest.
    let skip = 2;
    let full_pcm: Vec<i16> = full[join + skip..]
        .iter()
        .flat_map(|f| f.pcm.iter().copied())
        .collect();
    let tail_pcm: Vec<i16> = tail[skip..]
        .iter()
        .flat_map(|f| f.pcm.iter().copied())
        .collect();
    assert_eq!(full_pcm.len(), tail_pcm.len());
    let ef = band_energies(&full_pcm);
    let et = band_energies(&tail_pcm);
    let mut worst = 0.0f64;
    for k in 1..k_end {
        if ef[k] < 1e6 {
            continue;
        }
        let db = (10.0 * (et[k] / ef[k]).log10()).abs();
        worst = worst.max(db);
    }
    eprintln!("join@{join}: worst band delta {worst:.2} dB over bands 1..{k_end} (k_x {k_x})");
    assert!(worst < 1.0, "worst band delta {worst} dB");

    // Joining at a non-header frame must NOT decode as SBR audio: the
    // decoder has no header and stays in the upsampling-only state
    // until the next one arrives — still never an error.
    let tail_nh = StreamDecoder::new()
        .decode_all(&stream[offsets[join + 3]..])
        .expect("header-less join decodes");
    assert_eq!(tail_nh.len(), full.len() - join - 3);
}

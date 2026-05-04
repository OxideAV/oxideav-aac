//! HE-AAC psychoacoustic-model validation gate.
//!
//! The plain AAC-LC encoder defaults the Bark-band PE / SMR allocator
//! ([`crate::psy::PsyModel`]) **on** as of round-24
//! (`tests/psy_corpus_validation.rs`). The HE-AAC v1 mono / stereo
//! and HE-AAC v2 wrappers in [`crate::he_aac_encoder`] previously kept
//! psy force-off pending an HE-AAC + SBR corpus validation; this gate
//! is that validation. As of round-25 the **mono** + **v2** (mono SCE
//! after L+R downmix) wrappers default psy-on; the **stereo** wrapper
//! still forces psy-off in the inner encoder pending the M/S CPE
//! quant-noise fix (see CHANGELOG round-25 entry).
//!
//! This test gate exercises the off-vs-on comparison against every
//! HE-AAC fixture under `docs/audio/aac/fixtures/` plus a couple of
//! synthetic stress shapes (clean tone + noise). A case is acceptable
//! if `psnr_on >= psnr_off - 2.0 dB` — same threshold as the AAC-LC
//! corpus gate (`psy_corpus_validation.rs`).

use std::fs;
use std::path::PathBuf;

use oxideav_aac::adts::parse_adts_header;
use oxideav_aac::decoder::make_decoder;
use oxideav_aac::he_aac_encoder::{HeAacMonoEncoder, HeAacStereoEncoder};
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Encoder, Frame, Packet, SampleFormat, TimeBase,
};

fn parse_wav(bytes: &[u8]) -> Option<(u16, u32, Vec<i16>)> {
    if bytes.len() < 44 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    let mut i = 12usize;
    let mut channels: u16 = 0;
    let mut sample_rate: u32 = 0;
    let mut bits_per_sample: u16 = 0;
    let mut data: Option<&[u8]> = None;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let sz =
            u32::from_le_bytes([bytes[i + 4], bytes[i + 5], bytes[i + 6], bytes[i + 7]]) as usize;
        let body_start = i + 8;
        let body_end = body_start + sz;
        if body_end > bytes.len() {
            break;
        }
        match id {
            b"fmt " => {
                if sz < 16 {
                    return None;
                }
                let format_tag = u16::from_le_bytes([bytes[body_start], bytes[body_start + 1]]);
                channels = u16::from_le_bytes([bytes[body_start + 2], bytes[body_start + 3]]);
                sample_rate = u32::from_le_bytes([
                    bytes[body_start + 4],
                    bytes[body_start + 5],
                    bytes[body_start + 6],
                    bytes[body_start + 7],
                ]);
                bits_per_sample =
                    u16::from_le_bytes([bytes[body_start + 14], bytes[body_start + 15]]);
                if format_tag != 1 && format_tag != 0xFFFE {
                    return None;
                }
            }
            b"data" => {
                data = Some(&bytes[body_start..body_end]);
                break;
            }
            _ => {}
        }
        i = body_end + (sz & 1);
    }
    let data = data?;
    if channels == 0 || sample_rate == 0 || bits_per_sample != 16 {
        return None;
    }
    let mut samples = Vec::with_capacity(data.len() / 2);
    for chunk in data.chunks_exact(2) {
        samples.push(i16::from_le_bytes([chunk[0], chunk[1]]));
    }
    Some((channels, sample_rate, samples))
}

fn downmix_to_mono(samples: &[i16], channels: u16) -> Vec<i16> {
    if channels <= 1 {
        return samples.to_vec();
    }
    let ch = channels as usize;
    let frames = samples.len() / ch;
    let mut out = Vec::with_capacity(frames);
    for f in 0..frames {
        let mut acc = 0i32;
        for c in 0..ch {
            acc += samples[f * ch + c] as i32;
        }
        out.push((acc / ch as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16);
    }
    out
}

fn pcm_to_bytes(pcm: &[i16]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(pcm.len() * 2);
    for &s in pcm {
        bytes.extend_from_slice(&s.to_le_bytes());
    }
    bytes
}

/// Encode mono S16 PCM through HE-AAC v1 mono. Returns concatenated
/// ADTS bytes (the inner LC encoder's output, with SBR FIL elements
/// staged before each `ID_END`).
fn encode_he_mono(pcm: &[i16], sample_rate: u32, bitrate: u64, psy_on: bool) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sample_rate);
    params.channels = Some(1);
    params.bit_rate = Some(bitrate);
    params.sample_format = Some(SampleFormat::S16);
    let mut enc = match HeAacMonoEncoder::new(&params) {
        Ok(e) => e,
        Err(_) => return Vec::new(),
    };
    enc.set_enable_psy_model(psy_on);
    let frame = Frame::Audio(AudioFrame {
        samples: pcm.len() as u32,
        pts: Some(0),
        data: vec![pcm_to_bytes(pcm)],
    });
    if enc.send_frame(&frame).is_err() {
        return Vec::new();
    }
    if enc.flush().is_err() {
        return Vec::new();
    }
    let mut adts = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts.extend_from_slice(&pkt.data);
    }
    adts
}

/// Encode stereo (interleaved s16) PCM through HE-AAC v1 stereo.
fn encode_he_stereo(pcm: &[i16], sample_rate: u32, bitrate: u64, psy_on: bool) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sample_rate);
    params.channels = Some(2);
    params.bit_rate = Some(bitrate);
    params.sample_format = Some(SampleFormat::S16);
    let mut enc = match HeAacStereoEncoder::new(&params) {
        Ok(e) => e,
        Err(_) => return Vec::new(),
    };
    enc.set_enable_psy_model(psy_on);
    let frame = Frame::Audio(AudioFrame {
        samples: (pcm.len() / 2) as u32,
        pts: Some(0),
        data: vec![pcm_to_bytes(pcm)],
    });
    if enc.send_frame(&frame).is_err() {
        return Vec::new();
    }
    if enc.flush().is_err() {
        return Vec::new();
    }
    let mut adts = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts.extend_from_slice(&pkt.data);
    }
    adts
}

/// Decode a HE-AAC ADTS stream back to PCM (interleaved s16) at the
/// **high** sample rate (= 2 × the AAC-LC core rate signalled in the
/// ADTS header). Returns empty on parse / decode failure.
fn decode_adts(adts: &[u8], sample_rate: u32, channels: u16) -> Vec<i16> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sample_rate);
    params.channels = Some(channels);
    params.sample_format = Some(SampleFormat::S16);
    let mut dec = match make_decoder(&params) {
        Ok(d) => d,
        Err(_) => return Vec::new(),
    };
    let tb = TimeBase::new(1, sample_rate as i64);
    let mut out = Vec::new();
    let mut off = 0usize;
    let mut pts = 0i64;
    while off + 7 <= adts.len() {
        if adts[off] != 0xFF || (adts[off + 1] & 0xF0) != 0xF0 {
            off += 1;
            continue;
        }
        let hdr = match parse_adts_header(&adts[off..]) {
            Ok(h) => h,
            Err(_) => {
                off += 1;
                continue;
            }
        };
        let frame_len = hdr.frame_length;
        if frame_len < 7 || off + frame_len > adts.len() {
            break;
        }
        let pkt = Packet::new(0, tb, adts[off..off + frame_len].to_vec())
            .with_pts(pts)
            .with_dts(pts);
        if oxideav_core::Decoder::send_packet(dec.as_mut(), &pkt).is_err() {
            break;
        }
        if let Ok(Frame::Audio(af)) = oxideav_core::Decoder::receive_frame(dec.as_mut()) {
            for plane in &af.data {
                for chunk in plane.chunks_exact(2) {
                    out.push(i16::from_le_bytes([chunk[0], chunk[1]]));
                }
            }
            pts += af.samples as i64;
        }
        off += frame_len;
    }
    out
}

/// PSNR (dB, full-scale s16). Trims the HE-AAC priming delay (2624
/// samples per ENCODER_DELAY_HE_AAC) so the pre-roll silence doesn't
/// dominate the metric.
fn psnr_db(source: &[i16], decoded: &[i16]) -> f64 {
    const PRIMING: usize = 2624;
    let dec = if decoded.len() > PRIMING {
        &decoded[PRIMING..]
    } else {
        return -200.0;
    };
    let n = source.len().min(dec.len());
    if n == 0 {
        return -200.0;
    }
    let mut sse = 0.0f64;
    for i in 0..n {
        let d = source[i] as f64 - dec[i] as f64;
        sse += d * d;
    }
    let mse = sse / n as f64;
    if mse <= 0.0 {
        return f64::INFINITY;
    }
    let peak = 32767.0_f64;
    10.0 * (peak * peak / mse).log10()
}

fn fixture_wav(name: &str) -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures")
        .join(name)
        .join("expected.wav")
}

#[derive(Debug)]
struct Outcome {
    name: String,
    sample_rate: u32,
    samples_in: usize,
    bytes_off: usize,
    bytes_on: usize,
    psnr_off: f64,
    psnr_on: f64,
}

/// Build a synthetic stereo PCM (interleaved s16) at the given high
/// rate. `freq_l` and `freq_r` are the per-channel sine frequencies in
/// Hz; `noise_amp` is added uncorrelated to each channel.
fn synth_stereo(sr: u32, freq_l: f32, freq_r: f32, noise_amp: f32, secs: f32) -> Vec<i16> {
    let n = (sr as f32 * secs) as usize;
    let mut state: u32 = 0x12345678;
    let mut out = Vec::with_capacity(n * 2);
    for i in 0..n {
        let t = i as f32 / sr as f32;
        let l = (2.0 * std::f32::consts::PI * freq_l * t).sin() * 0.4;
        let r = (2.0 * std::f32::consts::PI * freq_r * t).sin() * 0.4;
        state = state.wrapping_mul(1103515245).wrapping_add(12345);
        let nl = ((state >> 16) as f32 / 32768.0 - 1.0) * noise_amp;
        state = state.wrapping_mul(1103515245).wrapping_add(12345);
        let nr = ((state >> 16) as f32 / 32768.0 - 1.0) * noise_amp;
        out.push(((l + nl).clamp(-1.0, 1.0) * 32767.0) as i16);
        out.push(((r + nr).clamp(-1.0, 1.0) * 32767.0) as i16);
    }
    out
}

/// HE-AAC needs the input rate to be 2 × an AAC-LC sample rate. 44_100
/// → core 22_050 (valid). 48_000 → core 24_000 (valid). 32_000 → core
/// 16_000 (valid). All in the SAMPLE_RATES table.
const HE_RATES: &[u32] = &[44_100, 48_000, 32_000];

fn run_mono_fixture(name: &str) -> Option<Outcome> {
    let path = fixture_wav(name);
    let bytes = fs::read(&path).ok()?;
    let (channels, sample_rate, samples) = parse_wav(&bytes)?;
    if !HE_RATES.contains(&sample_rate) {
        return None;
    }
    let mono = downmix_to_mono(&samples, channels);
    if mono.len() < 8192 {
        return None;
    }
    let bitrate = 32_000u64; // typical HE-AAC v1 mono operating point
    let off = encode_he_mono(&mono, sample_rate, bitrate, false);
    let on = encode_he_mono(&mono, sample_rate, bitrate, true);
    if off.is_empty() || on.is_empty() {
        return None;
    }
    let dec_off = decode_adts(&off, sample_rate, 1);
    let dec_on = decode_adts(&on, sample_rate, 1);
    Some(Outcome {
        name: format!("{name}-mono"),
        sample_rate,
        samples_in: mono.len(),
        bytes_off: off.len(),
        bytes_on: on.len(),
        psnr_off: psnr_db(&mono, &dec_off),
        psnr_on: psnr_db(&mono, &dec_on),
    })
}

fn run_stereo_synth(name: &str, sr: u32, pcm: Vec<i16>, bitrate: u64) -> Option<Outcome> {
    let off = encode_he_stereo(&pcm, sr, bitrate, false);
    let on = encode_he_stereo(&pcm, sr, bitrate, true);
    if off.is_empty() || on.is_empty() {
        return None;
    }
    let dec_off = decode_adts(&off, sr, 2);
    let dec_on = decode_adts(&on, sr, 2);
    let psnr_off = psnr_db(&pcm, &dec_off);
    let psnr_on = psnr_db(&pcm, &dec_on);
    Some(Outcome {
        name: name.to_string(),
        sample_rate: sr,
        samples_in: pcm.len(),
        bytes_off: off.len(),
        bytes_on: on.len(),
        psnr_off,
        psnr_on,
    })
}

#[test]
fn he_aac_psy_does_not_regress_corpus_or_synthetic() {
    let mut outcomes: Vec<Outcome> = Vec::new();

    // Walk available HE-AAC fixtures, mono-downmix path through HE-AAC
    // v1 mono so the wrapper's LC + SBR mono path is exercised.
    for name in &[
        "he-aac-v1-stereo-44100-32kbps-adts",
        "he-aac-v2-stereo-32000-24kbps-m4a",
    ] {
        if let Some(o) = run_mono_fixture(name) {
            outcomes.push(o);
        }
    }

    // Synthetic stereo coverage — the corpus is tiny, so add a couple
    // of in-process shapes to give the regression gate teeth on the
    // stereo CPE + SBR path.
    if let Some(o) = run_stereo_synth(
        "synth-stereo-440-880-44100",
        44_100,
        synth_stereo(44_100, 440.0, 880.0, 0.02, 0.5),
        64_000,
    ) {
        outcomes.push(o);
    }
    if let Some(o) = run_stereo_synth(
        "synth-stereo-1000-noise-48000",
        48_000,
        synth_stereo(48_000, 1000.0, 1500.0, 0.05, 0.5),
        64_000,
    ) {
        outcomes.push(o);
    }

    if outcomes.is_empty() {
        eprintln!("HE-AAC psy gate: no eligible fixtures or synthetic encodes — skipping");
        return;
    }

    eprintln!(
        "\nHE-AAC psy corpus gate — {} cases evaluated:\n\
         {:<42} {:>5}  {:>9}  {:>7}/{:>7}  {:>7}  {:>7}  {:>7}",
        outcomes.len(),
        "case",
        "Hz",
        "samples",
        "B(off)",
        "B(on)",
        "PSNR_off",
        "PSNR_on",
        "Δ PSNR",
    );
    let mut sum_delta = 0.0f64;
    let mut worst = f64::INFINITY;
    let mut worst_name: &str = "";
    for o in &outcomes {
        let d = o.psnr_on - o.psnr_off;
        sum_delta += d;
        if d < worst {
            worst = d;
            worst_name = &o.name;
        }
        eprintln!(
            "{:<42} {:>5}  {:>9}  {:>7}/{:>7}  {:>7.2}  {:>7.2}  {:+7.2}",
            o.name, o.sample_rate, o.samples_in, o.bytes_off, o.bytes_on, o.psnr_off, o.psnr_on, d
        );
    }
    let mean = sum_delta / outcomes.len() as f64;
    eprintln!(
        "\nHE-AAC psy gate: mean Δ PSNR = {:+.2} dB, worst = {:+.2} dB on '{}'",
        mean, worst, worst_name
    );

    assert!(
        worst >= -2.0,
        "HE-AAC psy regressed '{worst_name}' by {worst:+.2} dB PSNR \
         (must be >= -2.0 dB to flip the wrapper default)"
    );
}

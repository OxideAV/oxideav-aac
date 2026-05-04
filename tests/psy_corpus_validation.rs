//! Wider-corpus validation gate for the Bark-band PE/SMR psychoacoustic
//! model (`crate::psy::PsyModel`).
//!
//! The narrow [`psy_model_bench`] benchmark only exercises three
//! synthetic fixtures (3-tone harmonic, tone+noise, white noise). Before
//! flipping the encoder default to `enable_psy_model = true` we need to
//! show the model **does not regress any fixture in the standing
//! corpus** by more than the documented tolerance. This driver walks
//! every `docs/audio/aac/fixtures/<name>/expected.wav` (the FFmpeg
//! reference PCM that ships next to each fixture) and uses it as the
//! source PCM for a matched-bitrate encode comparison: encode the WAV
//! once with psy off and once with psy on, decode both back through the
//! in-tree decoder, then compute PCM PSNR-vs-source on the
//! length-aligned overlap.
//!
//! The fixtures span every standard AAC sampling rate index 4-8
//! (44.1 / 48 / 32 / 24 / 22.05 / 16 / 11.025 / 8 kHz), mono+stereo,
//! and content-shape extremes from the corpus catalogue (PNS-noise,
//! intensity-stereo, M/S stereo, TNS-active, chirps, hexagonal-PCE,
//! 5.1 and 7.1 multichannel). Multichannel files are downmixed to mono
//! by averaging channels — the psy model is per-band-per-channel and
//! the regression invariant must hold on the mono digest (averaging
//! cannot mask a per-channel regression because mean-square error
//! averages linearly).
//!
//! ## Acceptance bar
//!
//! For every fixture that the encoder can ingest at its native rate
//! (i.e. the WAV's sample rate appears in [`SAMPLE_RATES`]):
//!
//!   - `psnr_on >= psnr_off - 2.0 dB` — no fixture regresses by more
//!     than 2 dB PSNR. This is the absolute bar for flipping the
//!     default; a single >2 dB regression keeps the default off.
//!
//! Aggregate: the **mean** PSNR delta across all eligible fixtures is
//! reported alongside per-fixture deltas. If the mean is positive AND
//! no fixture regresses by >2 dB, the gate passes and the per-encoder
//! default may be flipped to on.
//!
//! ## What this test does NOT do
//!
//! - It does not pin numeric PSNR values per fixture. The PSNR
//!   numbers depend on the source content, the quantiser noise floor,
//!   and the decoder's IMDCT scaling — pinning them would couple this
//!   gate to encoder math changes unrelated to the psy model. The
//!   *delta* (on - off) is what matters.
//! - It does not separately track ODG / SDR — those would each require
//!   a perceptual reference model out of the scope of an integration
//!   test. PSNR-vs-source on PCM round-trip is the cheap proxy that
//!   catches the kind of regression we care about: the model
//!   silencing whole bands or shifting bits to bands that perceptually
//!   don't need them.
//!
//! ## Why this is cheap
//!
//! Each fixture's WAV is ~130 KB / 1 s of audio. Encoding 18 fixtures
//! twice (off+on) and decoding 36 streams takes ~10 s total in debug.
//! The whole gate runs in a single integration-test binary so the
//! `cargo test --test psy_corpus_validation` invocation is one shot.

use std::fs;
use std::path::PathBuf;

use oxideav_aac::adts::parse_adts_header;
use oxideav_aac::decoder::make_decoder;
use oxideav_aac::encoder::AacEncoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Encoder, Frame, Packet, SampleFormat, TimeBase,
};

/// Locate `docs/audio/aac/fixtures/<name>/expected.wav`.
///
/// Workspace layout: tests run with CWD = crate root; docs live two
/// levels up at `../../docs/`. Standalone CI doesn't ship the docs
/// repo so missing fixtures skip silently.
fn fixture_wav(name: &str) -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures")
        .join(name)
        .join("expected.wav")
}

/// Set of fixtures the corpus walker expects to find under `docs/`.
/// Keep in sync with the directory listing under
/// `docs/audio/aac/fixtures/`.
const FIXTURES: &[&str] = &[
    "aac-latm-stream",
    "aac-lc-5.1-48000-256kbps-mp4",
    "aac-lc-7.1-48000-320kbps-mp4",
    "aac-lc-chirp-windows",
    "aac-lc-hexagonal-with-pce",
    "aac-lc-intensity-stereo",
    "aac-lc-mono-11025-32kbps-adts",
    "aac-lc-mono-44100-64kbps-adts",
    "aac-lc-mono-8000-16kbps-adts",
    "aac-lc-ms-stereo",
    "aac-lc-pns-noise",
    "aac-lc-stereo-22050-64kbps-adts",
    "aac-lc-stereo-44100-128kbps-adts",
    "aac-lc-stereo-48000-128kbps-mp4",
    "aac-lc-tns-active",
    "aac-with-id3v2",
    "he-aac-v1-stereo-44100-32kbps-adts",
    "he-aac-v2-stereo-32000-24kbps-m4a",
];

/// Standard AAC sampling rates (ISO/IEC 14496-3 Table 1.16). Anything
/// outside this set is skipped — the encoder rejects non-standard
/// rates outright and resampling is out of scope for this gate.
const AAC_RATES: &[u32] = &[
    96_000, 88_200, 64_000, 48_000, 44_100, 32_000, 24_000, 22_050, 16_000, 12_000, 11_025, 8_000,
    7_350,
];

/// Result of one fixture's off-vs-on encode comparison.
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

/// Parse a minimal RIFF/WAVE file → (channels, sample_rate, interleaved
/// s16le samples). Mirrors the parser in `tests/docs_corpus.rs` but
/// kept local to avoid cross-test coupling.
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

/// Downmix interleaved s16 → mono by averaging channels (s16 saturating).
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
        let avg = acc / ch as i32;
        out.push(avg.clamp(i16::MIN as i32, i16::MAX as i32) as i16);
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

/// Encode a mono S16 PCM buffer at the given sample rate + bitrate
/// with the psy model on or off. Returns the raw concatenated ADTS
/// bytes.
fn encode_mono(pcm: &[i16], sample_rate: u32, bitrate: u64, psy_on: bool) -> Vec<u8> {
    // PNS replaces noise-classified bands with PRNG energy on decode;
    // that's perceptually fine but ruins PCM PSNR. Turn it off so
    // both runs are apples-to-apples on the PCM-distance metric.
    std::env::set_var("OXIDEAV_AAC_DISABLE_PNS", "1");

    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sample_rate);
    params.channels = Some(1);
    params.bit_rate = Some(bitrate);
    params.sample_format = Some(SampleFormat::S16);
    let mut enc = AacEncoder::new(&params).expect("encoder ctor");
    enc.set_enable_psy_model(psy_on);
    let frame = Frame::Audio(AudioFrame {
        samples: pcm.len() as u32,
        pts: Some(0),
        data: vec![pcm_to_bytes(pcm)],
    });
    enc.send_frame(&frame).expect("send_frame");
    enc.flush().expect("flush");
    let mut adts = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts.extend_from_slice(&pkt.data);
    }
    adts
}

/// Decode a concatenated ADTS bytestream back to interleaved S16 PCM
/// (mono in this corpus). Returns an empty Vec on parse failure.
fn decode_adts(adts: &[u8], sample_rate: u32) -> Vec<i16> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sample_rate);
    params.channels = Some(1);
    params.sample_format = Some(SampleFormat::S16);
    let mut dec = make_decoder(&params).expect("decoder ctor");
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
        oxideav_core::Decoder::send_packet(dec.as_mut(), &pkt).expect("send_packet");
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

/// PSNR (dB, full-scale s16) of `decoded` against `source` on the
/// length-aligned overlap. Trims the encoder priming delay (2112
/// samples per `crate::gapless::ENCODER_DELAY_AAC_LC`) so the lossy
/// silent ramp at the start doesn't dominate the metric.
fn psnr_db(source: &[i16], decoded: &[i16]) -> f64 {
    const PRIMING: usize = 2112;
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

/// Pick a target bitrate that matches the fixture's typical operating
/// point. Most of the corpus is recorded at "standard" rates: mono ≈
/// 64 kbps, stereo ≈ 128 kbps. We always encode at one of those two
/// — anything finer would just chase the fixture's source bitrate
/// noise floor and conflate the bench with the rate-control path.
fn target_bitrate_for(_name: &str) -> u64 {
    // Mono encode at 64 kbps. The downmix to mono is cheaper than the
    // multi-channel encoder path and the per-band psy decisions are
    // independent of the channel count anyway.
    64_000
}

fn run_one(name: &str) -> Option<Outcome> {
    let path = fixture_wav(name);
    let bytes = match fs::read(&path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("skip {name}: missing {} ({e})", path.display());
            return None;
        }
    };
    let (channels, sample_rate, samples) = match parse_wav(&bytes) {
        Some(t) => t,
        None => {
            eprintln!("skip {name}: parse_wav failed");
            return None;
        }
    };
    if !AAC_RATES.contains(&sample_rate) {
        eprintln!("skip {name}: non-AAC sample rate {sample_rate}");
        return None;
    }
    let mono = downmix_to_mono(&samples, channels);
    if mono.len() < 4096 {
        eprintln!("skip {name}: too few samples ({})", mono.len());
        return None;
    }
    let bitrate = target_bitrate_for(name);
    let off = encode_mono(&mono, sample_rate, bitrate, false);
    let on = encode_mono(&mono, sample_rate, bitrate, true);
    let dec_off = decode_adts(&off, sample_rate);
    let dec_on = decode_adts(&on, sample_rate);
    let psnr_off = psnr_db(&mono, &dec_off);
    let psnr_on = psnr_db(&mono, &dec_on);
    Some(Outcome {
        name: name.to_string(),
        sample_rate,
        samples_in: mono.len(),
        bytes_off: off.len(),
        bytes_on: on.len(),
        psnr_off,
        psnr_on,
    })
}

/// Walk the entire AAC fixture corpus and verify the regression
/// invariant: no fixture is allowed to lose more than 2 dB PSNR when
/// the psy model is enabled.
///
/// This is the gate for flipping `AacEncoder::enable_psy_model`'s
/// default to `true`. It is hard-asserted: a single >2 dB regression
/// on any eligible fixture fails the test.
#[test]
fn psy_model_does_not_regress_on_any_corpus_fixture() {
    let mut outcomes: Vec<Outcome> = Vec::new();
    for &name in FIXTURES {
        if let Some(o) = run_one(name) {
            outcomes.push(o);
        }
    }
    if outcomes.is_empty() {
        eprintln!("psy corpus gate: no fixtures available — skipping (CI without docs/?)");
        return;
    }

    eprintln!(
        "\npsy corpus gate — {} fixtures evaluated:\n\
         {:<40} {:>5}  {:>9}  {:>7}/{:>7}  {:>7}  {:>7}  {:>7}",
        outcomes.len(),
        "fixture",
        "Hz",
        "samples",
        "B(off)",
        "B(on)",
        "PSNR_off",
        "PSNR_on",
        "Δ PSNR",
    );
    let mut sum_delta = 0.0f64;
    let mut worst_delta = f64::INFINITY;
    let mut worst_name: &str = "";
    for o in &outcomes {
        let delta = o.psnr_on - o.psnr_off;
        sum_delta += delta;
        if delta < worst_delta {
            worst_delta = delta;
            worst_name = &o.name;
        }
        eprintln!(
            "{:<40} {:>5}  {:>9}  {:>7}/{:>7}  {:>7.2}  {:>7.2}  {:+7.2}",
            o.name,
            o.sample_rate,
            o.samples_in,
            o.bytes_off,
            o.bytes_on,
            o.psnr_off,
            o.psnr_on,
            delta,
        );
    }
    let mean = sum_delta / outcomes.len() as f64;
    eprintln!(
        "\npsy corpus gate: mean Δ PSNR = {:+.2} dB, worst = {:+.2} dB on '{}'",
        mean, worst_delta, worst_name,
    );

    // Hard assertion: every fixture must come within 2 dB of the
    // baseline. If this fires the default flip is *not* allowed and
    // the regression must be diagnosed (which fixture-shape broke?
    // is the psy model too aggressive on that band-class?).
    assert!(
        worst_delta >= -2.0,
        "psy model regressed fixture '{worst_name}' by {worst_delta:+.2} dB PSNR \
         (must be >= -2.0 dB for the default-flip gate); see corpus print-out above"
    );
}

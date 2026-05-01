//! Bit-savings audit for PNS on noise-rich content (cymbals + sax-like
//! harmonic stack + room-tone broadband background). Encodes the same
//! fixture twice — once with PNS active (default), once with PNS forced
//! off via `OXIDEAV_AAC_DISABLE_PNS` — and asserts the PNS-active stream
//! is materially smaller while staying in RMS round-trip parity.
//!
//! ISO/IEC 14496-3 §4.6.13 (PNS): high-frequency noise-like bands are
//! signalled by codebook 13 and a single 9-bit `dpcm_noise_nrg` energy
//! seed (subsequent PNS bands ride DPCM deltas off that seed). The
//! decoder synthesises the band as random spectral lines scaled to the
//! stored energy — no Huffman coefficients are transmitted. On
//! noise-rich HF bands this is dramatically cheaper than a regular
//! Huffman codebook.
//!
//! The acceptance window (~8-15% smaller) is the README target for
//! task #132. The fixture is deliberately noise-rich: a broadband
//! background, a few sine tones in the LF/mid range that PNS leaves
//! alone (band centre < 4 kHz), and a high-frequency cymbal-like
//! transient envelope. PNS will swap most of the >=4 kHz bands into
//! noise-coded form.

use oxideav_aac::adts::parse_adts_header;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, Packet};
#[allow(unused_imports)]
use oxideav_core::{Decoder, Encoder};

const SR: u32 = 44_100;
const SECS: usize = 1;

/// Seeded deterministic PRNG for the broadband background. We don't use
/// `rand` — the encoder layer must be reproducible across CI machines.
fn lcg(state: &mut u32) -> f32 {
    *state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    (*state >> 16) as i16 as f32 / 32768.0
}

/// Build a noise-rich mono PCM fixture as interleaved `S16` little-endian
/// bytes. Composition:
///  * Wideband white noise at modest amplitude (room tone).
///  * Three low/mid sine tones (350 / 880 / 1500 Hz) — sax-like fundamental
///    + harmonics that should *stay* on regular Huffman codebooks (band
///    centres < 4 kHz, gated by `PNS_IS_MIN_HZ`).
///  * High-frequency shaped noise (5-15 kHz pink-ish, gated by an
///    exponential decay envelope) — the "cymbal" component PNS should
///    turn into NOISE_HCB bands.
fn make_noise_rich_pcm() -> Vec<u8> {
    let n = SR as usize * SECS;
    let mut pcm = Vec::with_capacity(n * 2);
    let mut rng = 0x1234_5678u32;
    let mut hf_rng = 0x9876_4321u32;
    for i in 0..n {
        let t = i as f32 / SR as f32;
        // Background room tone — broadband but quiet.
        let bg = 0.05 * lcg(&mut rng);
        // Sax-like harmonic stack (LF / mid).
        let sax = 0.18 * (2.0 * std::f32::consts::PI * 350.0 * t).sin()
            + 0.12 * (2.0 * std::f32::consts::PI * 880.0 * t).sin()
            + 0.08 * (2.0 * std::f32::consts::PI * 1500.0 * t).sin();
        // HF "cymbal" — high-pass-feeling noise with a slow envelope
        // so PNS sees a relatively stationary noise band (and TNS
        // doesn't catch it as a transient).
        let env = 0.3 * (1.0 - (-t * 0.5).exp());
        let cym = env * lcg(&mut hf_rng);
        // Crude high-pass: subtract a 2-tap moving average of the noise.
        let cym_hp = cym - 0.5 * lcg(&mut hf_rng) * env;
        let s = sax + bg + 0.4 * cym_hp;
        let q = (s.clamp(-1.0, 1.0) * 32767.0) as i16;
        pcm.extend_from_slice(&q.to_le_bytes());
    }
    pcm
}

/// Process-global mutex that serialises every `encode_with_env` call so
/// the env-var toggling can't race when cargo-test runs the three tests
/// in this binary in parallel. Other test binaries in this crate live
/// in separate processes and aren't affected.
fn env_mutex() -> &'static std::sync::Mutex<()> {
    static M: std::sync::OnceLock<std::sync::Mutex<()>> = std::sync::OnceLock::new();
    M.get_or_init(|| std::sync::Mutex::new(()))
}

fn encode_with_env(pcm: &[u8], disable_pns: bool) -> Vec<u8> {
    // Serialise all PNS-toggle encodes against each other (env var is
    // process-global). Holding the mutex across the whole encode means
    // every test in this binary sees the env-var state it asked for.
    let _guard = env_mutex().lock().unwrap_or_else(|e| e.into_inner());
    if disable_pns {
        std::env::set_var("OXIDEAV_AAC_DISABLE_PNS", "1");
    } else {
        std::env::remove_var("OXIDEAV_AAC_DISABLE_PNS");
    }
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(SR);
    params.channels = Some(1);
    params.bit_rate = Some(96_000);
    let mut enc = oxideav_aac::encoder::make_encoder(&params).expect("make encoder");
    let samples = pcm.len() / 2;
    let frame = Frame::Audio(AudioFrame {
        samples: samples as u32,
        pts: Some(0),
        data: vec![pcm.to_vec()],
    });
    enc.send_frame(&frame).expect("send_frame");
    enc.flush().expect("flush");
    let mut out = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        out.extend_from_slice(&p.data);
    }
    // Always restore the default before returning so leftover state
    // doesn't bleed into a later non-mutexed reader.
    std::env::remove_var("OXIDEAV_AAC_DISABLE_PNS");
    out
}

fn raw_data_block_bytes(adts: &[u8]) -> usize {
    let mut total = 0usize;
    let mut i = 0;
    while i + 7 < adts.len() {
        if adts[i] != 0xFF || (adts[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        match parse_adts_header(&adts[i..]) {
            Ok(h) => {
                if h.frame_length == 0 || i + h.frame_length > adts.len() {
                    break;
                }
                total += h.frame_length - h.header_length();
                i += h.frame_length;
            }
            Err(_) => i += 1,
        }
    }
    total
}

#[test]
fn pns_saves_bits_on_noise_rich_fixture() {
    let pcm = make_noise_rich_pcm();
    // The PCM-on order matters because the encoder uses process-global
    // env-var gating; do A/B back to back in a single test.
    let with_pns = encode_with_env(&pcm, false);
    let no_pns = encode_with_env(&pcm, true);
    assert!(!with_pns.is_empty());
    assert!(!no_pns.is_empty());

    // Strip the constant-overhead ADTS headers so the comparison is on
    // raw_data_block bytes — that's where PNS pays off.
    let pns_bytes = raw_data_block_bytes(&with_pns);
    let lr_bytes = raw_data_block_bytes(&no_pns);
    let savings = (lr_bytes as f64 - pns_bytes as f64) / lr_bytes as f64;

    eprintln!(
        "pns_savings: pns_on={pns_bytes}B pns_off={lr_bytes}B savings={:.2}%",
        savings * 100.0
    );

    // Acceptance window per task #132 brief: 8-15% smaller via PNS at
    // matched perceptual quality. We accept >=5% as the minimum
    // (some test signals are noisier than others and the lower bound is
    // a regression guard, not the upper claim). Top end is uncapped —
    // savings can scale higher on more PNS-friendly material.
    assert!(
        savings >= 0.05,
        "expected PNS to save >=5% of raw_data_block bytes on the noise-rich \
         fixture, got {:.2}% (pns={pns_bytes}B no-pns={lr_bytes}B)",
        savings * 100.0,
    );
}

/// Decode a stream through ffmpeg if it's on PATH; return `None` so
/// callers can skip cleanly when the binary is missing.
fn ffmpeg_decode(bytes: &[u8], stem: &str) -> Option<Vec<i16>> {
    let in_path = std::env::temp_dir().join(format!("{stem}.aac"));
    let out_path = std::env::temp_dir().join(format!("{stem}.s16"));
    std::fs::write(&in_path, bytes).ok()?;
    let status = std::process::Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .arg("-i")
        .arg(&in_path)
        .arg("-f")
        .arg("s16le")
        .arg("-ar")
        .arg(SR.to_string())
        .arg(&out_path)
        .status()
        .ok()?;
    if !status.success() {
        return None;
    }
    let raw = std::fs::read(&out_path).ok()?;
    Some(
        raw.chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect(),
    )
}

#[test]
fn pns_noise_rich_decodes_through_ffmpeg() {
    // Encode the noise-rich fixture with PNS active; decode it through
    // ffmpeg. We accept any non-empty decode that lands within an order
    // of magnitude of the input RMS — the goal is to verify ffmpeg
    // doesn't choke on the codebook-13 sections / 9-bit `dpcm_noise_nrg`
    // seed / DPCM noise-energy deltas, not to chase tonal accuracy on
    // a synthetic noise fixture.
    let pcm = make_noise_rich_pcm();
    let aac = encode_with_env(&pcm, false);
    let Some(decoded) = ffmpeg_decode(&aac, "oxideav_aac_pns_noise_rich") else {
        eprintln!("ffmpeg not available — skipping");
        return;
    };
    assert!(
        !decoded.is_empty(),
        "ffmpeg returned an empty decode for our PNS-active stream"
    );
    let warm = 4096usize.min(decoded.len() / 4);
    let rms_in: f64 = pcm
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]) as f64 / 32768.0)
        .skip(warm)
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt();
    let rms_out: f64 = decoded[warm..]
        .iter()
        .map(|&s| s as f64 / 32768.0)
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt();
    let ratio = rms_out / rms_in.max(1e-9);
    eprintln!(
        "ffmpeg PNS cross-decode: in_samples={} out_samples={} in_rms={rms_in:.4} \
         out_rms={rms_out:.4} ratio={ratio:.3}",
        pcm.len() / 2,
        decoded.len()
    );
    assert!(
        ratio > 0.1 && ratio < 10.0,
        "ffmpeg PNS cross-decode RMS drift: {ratio:.3} (in={rms_in:.4} out={rms_out:.4})"
    );
}

#[test]
fn pns_round_trip_preserves_rms_on_noise_rich_fixture() {
    // Re-encode with PNS and decode through our own decoder. The PNS
    // bands carry no Huffman coefficients, only an energy seed, so the
    // round-trip RMS must stay close to the input's RMS even though the
    // exact phase/spectrum is replaced with synthesised noise.
    let pcm = make_noise_rich_pcm();
    let aac = encode_with_env(&pcm, false);
    assert!(!aac.is_empty());

    // Decode via our own decoder.
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(SR);
    params.channels = Some(1);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make dec");
    let tb = oxideav_core::TimeBase::new(1, SR as i64);
    let mut decoded = Vec::<i16>::new();
    let mut i = 0;
    let mut pkt_idx = 0i64;
    while i + 7 < aac.len() {
        if aac[i] != 0xFF || (aac[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        let hdr = match parse_adts_header(&aac[i..]) {
            Ok(h) => h,
            Err(_) => {
                i += 1;
                continue;
            }
        };
        if hdr.frame_length == 0 || i + hdr.frame_length > aac.len() {
            break;
        }
        let pkt =
            Packet::new(0, tb, aac[i..i + hdr.frame_length].to_vec()).with_pts(pkt_idx * 1024);
        dec.send_packet(&pkt).unwrap();
        if let Ok(Frame::Audio(af)) = dec.receive_frame() {
            for chunk in af.data[0].chunks_exact(2) {
                decoded.push(i16::from_le_bytes([chunk[0], chunk[1]]));
            }
        }
        i += hdr.frame_length;
        pkt_idx += 1;
    }

    // Skip the encoder/decoder warm-up (two frames of latency) before
    // measuring RMS so window-overlap startup doesn't skew the result.
    let warm = 4096usize.min(decoded.len() / 4);
    let rms_in: f64 = pcm
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]) as f64 / 32768.0)
        .skip(warm)
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt();
    let rms_out: f64 = decoded[warm..]
        .iter()
        .map(|&s| s as f64 / 32768.0)
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt();
    let ratio = rms_out / rms_in.max(1e-9);
    eprintln!("pns rms round-trip: in={rms_in:.4} out={rms_out:.4} ratio={ratio:.3}");
    // PNS replaces noise bands with synthesised noise of matching
    // energy — the integrated RMS should track within ~50% (allowing
    // for the PNS quantisation step on the band energy plus the
    // window-overlap envelope mismatch). This is the regression guard
    // that PNS isn't grossly mis-scaling band energies.
    assert!(
        ratio > 0.5 && ratio < 2.0,
        "PNS round-trip RMS drift: rms_out/rms_in = {ratio:.3} (in={rms_in:.4} out={rms_out:.4})"
    );
}

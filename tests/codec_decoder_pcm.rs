//! End-to-end PCM validation through the registered `oxideav_core::Decoder`
//! trait path (not the bare `StreamDecoder`).
//!
//! `tests/pcm_byte_exact.rs` already pins `StreamDecoder`'s output against
//! the staged `expected.wav` corpus. This driver re-runs the same
//! comparison through the **framework trait surface** — the exact path a
//! container demuxer drives: build the decoder via
//! [`oxideav_aac::codec_decoder::make_decoder`], feed one ADTS frame per
//! [`Packet`], pull each [`AudioFrame`], and reassemble interleaved s16.
//! It confirms the packetisation / `AudioFrame` byte-layout / across-packet
//! state threading the trait wrapper adds preserves the underlying decode
//! to the last LSB.
//!
//! Two regimes, mirroring `pcm_byte_exact.rs`:
//!
//! * **Deterministic fixtures** (no PNS) — must match the reference s16 to
//!   ≤1 LSB (the only spec-allowed divergence being the §4.6.11 IMDCT
//!   `f64`-direct-sum vs `float32`-fast-transform rounding).
//! * **PNS fixtures** — the §4.6.13.3 noise *phase* is spec-undefined, so
//!   byte-exactness is impossible; compared in the §8 PCM-RMS domain.
//!
//! `oxideav-aac` is its own repository and `docs/` lives in the workspace
//! umbrella, so each fixture is skipped (logged, success) when its files
//! are absent — CI stays green in the standalone layout.

use std::fs;
use std::path::PathBuf;

use oxideav_aac::adts::{AdtsHeader, ADTS_HEADER_BYTES_NO_CRC};
use oxideav_aac::codec_decoder::make_decoder;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, SampleFormat, TimeBase};

fn fixture_dir(name: &str) -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures").join(name)
}

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

/// Skip a leading ID3v2 tag if present.
fn skip_id3v2(data: &[u8]) -> &[u8] {
    if data.len() < 10 || &data[..3] != b"ID3" {
        return data;
    }
    let size = data[6..10]
        .iter()
        .fold(0usize, |acc, &b| (acc << 7) | usize::from(b & 0x7f));
    let footer = if data[5] & 0x10 != 0 { 10 } else { 0 };
    let total = 10 + size + footer;
    if total >= data.len() {
        data
    } else {
        &data[total..]
    }
}

/// Slice a raw-ADTS byte buffer into one [`Packet`] per ADTS frame.
fn split_into_packets(bytes: &[u8]) -> Vec<Packet> {
    let bytes = skip_id3v2(bytes);
    let tb = TimeBase::new(1, 44_100);
    let mut packets = Vec::new();
    let mut pos = 0usize;
    while pos + ADTS_HEADER_BYTES_NO_CRC <= bytes.len() {
        let Ok((header, _)) = AdtsHeader::parse(&bytes[pos..]) else {
            break;
        };
        let fl = header.aac_frame_length as usize;
        if fl == 0 || pos + fl > bytes.len() {
            break;
        }
        packets.push(Packet::new(0, tb, bytes[pos..pos + fl].to_vec()));
        pos += fl;
    }
    packets
}

/// Drive the trait decoder over a fixture's `input.aac` and reassemble
/// the interleaved s16 PCM exactly as a pipeline consumer would.
fn decode_via_trait(name: &str) -> Option<Vec<i16>> {
    let data = fs::read(fixture_dir(name).join("input.aac")).ok()?;
    let packets = split_into_packets(&data);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_format = Some(SampleFormat::S16);
    let mut dec = make_decoder(&params).expect("make_decoder");

    let mut pcm: Vec<i16> = Vec::new();
    for pkt in &packets {
        dec.send_packet(pkt)
            .unwrap_or_else(|e| panic!("{name}: send_packet: {e}"));
        loop {
            match dec.receive_frame() {
                Ok(Frame::Audio(a)) => {
                    assert_eq!(a.data.len(), 1, "{name}: interleaved single plane");
                    for c in a.data[0].chunks_exact(2) {
                        pcm.push(i16::from_le_bytes([c[0], c[1]]));
                    }
                }
                Ok(other) => panic!("{name}: expected Audio, got {other:?}"),
                Err(oxideav_core::Error::NeedMore) => break,
                Err(e) => panic!("{name}: receive_frame: {e}"),
            }
        }
    }
    Some(pcm)
}

#[derive(Default)]
struct Compare {
    n: usize,
    exact: usize,
    max_err: i32,
    err_sse: f64,
    sig_sse: f64,
}

impl Compare {
    fn exact_fraction(&self) -> f64 {
        if self.n == 0 {
            0.0
        } else {
            self.exact as f64 / self.n as f64
        }
    }
    fn err_to_signal(&self) -> f64 {
        if self.n == 0 {
            return 0.0;
        }
        let err_rms = (self.err_sse / self.n as f64).sqrt();
        let sig_rms = (self.sig_sse / self.n as f64).sqrt();
        if sig_rms == 0.0 {
            0.0
        } else {
            err_rms / sig_rms
        }
    }
}

fn compare(ours: &[i16], expected: &[i16]) -> Compare {
    let n = ours.len().min(expected.len());
    let mut c = Compare {
        n,
        ..Default::default()
    };
    for i in 0..n {
        let d = (ours[i] as i32 - expected[i] as i32).abs();
        if d == 0 {
            c.exact += 1;
        }
        c.max_err = c.max_err.max(d);
        c.err_sse += (d as f64).powi(2);
        c.sig_sse += (expected[i] as f64).powi(2);
    }
    c
}

/// PNS-free fixtures: the trait path must match the reference s16 to ≤1 LSB.
const DETERMINISTIC_FIXTURES: &[&str] =
    &["aac-lc-mono-8000-16kbps-adts", "aac-lc-intensity-stereo"];

#[test]
fn deterministic_fixtures_byte_exact_through_trait() {
    let mut decoded_any = false;
    for &name in DETERMINISTIC_FIXTURES {
        let (Some(ours), Some(expected)) = (
            decode_via_trait(name),
            read_wav_s16(&fixture_dir(name).join("expected.wav")),
        ) else {
            eprintln!("skip {name}: fixtures unavailable");
            continue;
        };
        decoded_any = true;
        assert_eq!(
            ours.len(),
            expected.len(),
            "{name}: sample-count mismatch ({} vs {})",
            ours.len(),
            expected.len()
        );
        let c = compare(&ours, &expected);
        eprintln!(
            "{name}: n={} exact={:.3}% max_err={}",
            c.n,
            100.0 * c.exact_fraction(),
            c.max_err
        );
        assert!(
            c.max_err <= 1,
            "{name}: max error {} exceeds the 1-LSB IMDCT bound",
            c.max_err
        );
        assert!(
            c.exact_fraction() >= 0.995,
            "{name}: only {:.3}% byte-exact through the trait path",
            100.0 * c.exact_fraction()
        );
    }
    if !decoded_any {
        eprintln!("docs fixtures unavailable; trait-path byte-exact pass skipped");
    }
}

/// PNS fixtures: compared in the §8 PCM-RMS domain (phase is undefined).
const PNS_FIXTURES: &[(&str, f64)] = &[
    ("aac-lc-mono-44100-64kbps-adts", 0.02),
    ("aac-lc-stereo-44100-128kbps-adts", 0.02),
    ("aac-lc-ms-stereo", 0.02),
];

#[test]
fn pns_fixtures_match_in_rms_through_trait() {
    let mut decoded_any = false;
    for &(name, max_ratio) in PNS_FIXTURES {
        let (Some(ours), Some(expected)) = (
            decode_via_trait(name),
            read_wav_s16(&fixture_dir(name).join("expected.wav")),
        ) else {
            eprintln!("skip {name}: fixtures unavailable");
            continue;
        };
        decoded_any = true;
        assert_eq!(ours.len(), expected.len(), "{name}: sample-count mismatch");
        let c = compare(&ours, &expected);
        eprintln!("{name}: ratio={:.5}", c.err_to_signal());
        assert!(
            c.err_to_signal() <= max_ratio,
            "{name}: error-to-signal RMS {:.5} exceeds {:.3}",
            c.err_to_signal(),
            max_ratio
        );
    }
    if !decoded_any {
        eprintln!("docs fixtures unavailable; trait-path PNS RMS pass skipped");
    }
}

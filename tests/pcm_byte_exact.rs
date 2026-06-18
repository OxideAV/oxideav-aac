//! `expected.wav` PCM comparison against the `docs/audio/aac/fixtures/`
//! ADTS corpus.
//!
//! This driver decodes each staged ADTS fixture to interleaved 16-bit
//! PCM via [`oxideav_aac::decode::StreamDecoder`] and compares it,
//! sample for sample, against the fixture's `expected.wav` (the
//! reference decoder's `pcm_s16le` output).
//!
//! ## What "byte-exact" means here, and its hard limit
//!
//! The `docs/audio/aac/aac-fixtures-and-traces.md` §8 is explicit that
//! `expected.wav` is one specific decoder's *implementation-defined*
//! output — its IMDCT precision, rounding mode, and inter-frame
//! rendering are baked into the bytes. Two distinct decoders that both
//! follow the spec faithfully can still differ by a fraction of an LSB
//! per sample, which shows up as an occasional ±1 in the rounded 16-bit
//! word. So there are two regimes:
//!
//! * **Deterministic decode path (no PNS).** Every reconstruction step
//!   is spec-determined to the last bit *except* the IMDCT, where this
//!   crate uses an `f64` direct cosine sum and the reference uses a
//!   `float32` fast transform. The two agree to ≤1 LSB. These fixtures
//!   are pinned at ≥99.5 % exact, **max error ≤ 1**.
//!
//! * **PNS-bearing decode path (§4.6.13).** The perceptual-noise
//!   substitution band fill draws a random vector whose *phase* §4.6.13.3
//!   leaves entirely open ("a suitable random number generator can be
//!   realized…"); only the per-band *energy* is normative. A decoder
//!   that picks a different generator produces different — but
//!   energy-correct — noise samples. These fixtures therefore cannot be
//!   byte-exact against any one reference decoder; we assert only that
//!   the *non-noise* sample population still matches (a high exact-sample
//!   fraction) and that the decode runs end to end.
//!
//! `oxideav-aac` is its own repository and `docs/` lives in the
//! workspace umbrella, so each fixture is skipped (logged, success) when
//! its files are absent — CI stays green in the standalone layout.

use std::fs;
use std::path::PathBuf;

use oxideav_aac::decode::StreamDecoder;

/// Read the `data` chunk of a 16-bit mono/stereo WAV as interleaved
/// `i16` samples.
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

/// Decode a fixture's `input.aac` to interleaved 16-bit PCM.
fn decode_fixture_pcm(name: &str) -> Option<Vec<i16>> {
    let path = PathBuf::from("../../docs/audio/aac/fixtures")
        .join(name)
        .join("input.aac");
    let data = fs::read(&path).ok()?;
    let mut dec = StreamDecoder::new();
    let frames = dec
        .decode_all(&data)
        .unwrap_or_else(|e| panic!("{name}: decode_all: {e}"));
    let mut pcm = Vec::new();
    for f in &frames {
        pcm.extend_from_slice(&f.pcm);
    }
    Some(pcm)
}

#[derive(Debug, Default)]
struct Compare {
    n: usize,
    exact: usize,
    off_by_one: usize,
    max_err: i32,
}

impl Compare {
    fn exact_fraction(&self) -> f64 {
        if self.n == 0 {
            return 0.0;
        }
        self.exact as f64 / self.n as f64
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
        } else if d == 1 {
            c.off_by_one += 1;
        }
        c.max_err = c.max_err.max(d);
    }
    c
}

/// Fixtures whose entire decode path is deterministic (no PNS band in
/// any frame). These must match the reference s16 output to ≤1 LSB.
const DETERMINISTIC_FIXTURES: &[&str] =
    &["aac-lc-mono-8000-16kbps-adts", "aac-lc-intensity-stereo"];

#[test]
fn deterministic_fixtures_are_byte_exact_within_one_lsb() {
    let mut decoded_any = false;
    for &name in DETERMINISTIC_FIXTURES {
        let (Some(ours), Some(expected)) =
            (decode_fixture_pcm(name), read_wav_s16(&fixture_wav(name)))
        else {
            eprintln!("skip {name}: fixtures unavailable");
            continue;
        };
        decoded_any = true;
        // Same length: the decoder emits the reference frame count with
        // no encoder-delay trim (the reference keeps the priming
        // samples for raw ADTS), so the streams align one-to-one.
        assert_eq!(
            ours.len(),
            expected.len(),
            "{name}: sample-count mismatch (our {} vs reference {})",
            ours.len(),
            expected.len()
        );
        let c = compare(&ours, &expected);
        eprintln!(
            "{name}: n={} exact={} ({:.3}%) off1={} max_err={}",
            c.n,
            c.exact,
            100.0 * c.exact_fraction(),
            c.off_by_one,
            c.max_err
        );
        // The only divergence the spec allows on this path is the
        // sub-LSB IMDCT rounding difference (≤1 in the 16-bit word).
        assert!(
            c.max_err <= 1,
            "{name}: max error {} exceeds the 1-LSB IMDCT-rounding bound",
            c.max_err
        );
        // With only ±1 IMDCT-boundary rounding noise, the overwhelming
        // majority of samples land exactly on the reference word.
        assert!(
            c.exact_fraction() >= 0.995,
            "{name}: only {:.3}% byte-exact (expected ≥99.5%)",
            100.0 * c.exact_fraction()
        );
    }
    if !decoded_any {
        eprintln!("docs fixtures unavailable; byte-exact PCM pass skipped");
    }
}

/// Fixtures that carry PNS bands: the noise *phase* is §4.6.13.3
/// spec-undefined, so byte-exactness is impossible, but the
/// deterministic (non-noise) sample population still matches a large
/// fraction. The pair is `(fixture, min exact fraction)`.
const PNS_FIXTURES: &[(&str, f64)] = &[
    ("aac-lc-mono-11025-32kbps-adts", 0.60),
    ("aac-lc-mono-44100-64kbps-adts", 0.70),
    ("aac-lc-stereo-22050-64kbps-adts", 0.70),
    ("aac-lc-stereo-44100-128kbps-adts", 0.80),
    ("aac-lc-ms-stereo", 0.80),
    ("aac-lc-tns-active", 0.80),
];

#[test]
fn pns_fixtures_match_deterministic_population() {
    let mut decoded_any = false;
    for &(name, min_exact) in PNS_FIXTURES {
        let (Some(ours), Some(expected)) =
            (decode_fixture_pcm(name), read_wav_s16(&fixture_wav(name)))
        else {
            eprintln!("skip {name}: fixtures unavailable");
            continue;
        };
        decoded_any = true;
        assert_eq!(
            ours.len(),
            expected.len(),
            "{name}: sample-count mismatch (our {} vs reference {})",
            ours.len(),
            expected.len()
        );
        let c = compare(&ours, &expected);
        eprintln!(
            "{name}: n={} exact={} ({:.3}%) off1={} max_err={}",
            c.n,
            c.exact,
            100.0 * c.exact_fraction(),
            c.off_by_one,
            c.max_err
        );
        assert!(
            c.exact_fraction() >= min_exact,
            "{name}: only {:.3}% byte-exact (expected ≥{:.0}% — the \
             non-noise sample population)",
            100.0 * c.exact_fraction(),
            100.0 * min_exact
        );
    }
    if !decoded_any {
        eprintln!("docs fixtures unavailable; PNS-population pass skipped");
    }
}

fn fixture_wav(name: &str) -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures")
        .join(name)
        .join("expected.wav")
}

//! The rate-derived coded bandwidth (`EncoderConfig::default_bandwidth_hz`
//! / `StreamEncoder::set_bandwidth`): lines above the cutoff are
//! zeroed before quantisation so the rate loop spends the frame's
//! bits below it (measured on the equal-rate harness against the
//! black-box reference encoder: ~1–1.5 dB of mean noise-to-mask at
//! 64–96 kbps stereo).

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};
use oxideav_aac::sbr_qmf::EncoderAnalysisQmf;

fn noise_stereo(n: usize) -> Vec<i16> {
    let mut seed = 0x0bad_5eedu32;
    (0..2 * n)
        .map(|_| {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            ((seed >> 8) as f64 / f64::from(1u32 << 24) * 16_000.0 - 8_000.0) as i16
        })
        .collect()
}

/// Energy below / above `cutoff_hz` of interleaved stereo at `fs`.
fn split_energy(pcm: &[i16], fs: u32, cutoff_hz: f64) -> (f64, f64) {
    let k_cut = (cutoff_hz / (f64::from(fs) / 128.0)) as usize;
    let (mut lo, mut hi) = (0.0, 0.0);
    for c in 0..2 {
        let mono: Vec<f64> = pcm
            .iter()
            .skip(c)
            .step_by(2)
            .map(|&v| f64::from(v))
            .collect();
        let mut q = EncoderAnalysisQmf::new();
        for slot in mono.chunks_exact(64) {
            for (k, v) in q.push_slot(slot).unwrap().iter().enumerate() {
                // The two bands straddling the cutoff belong to
                // neither side (the analysis bank overlaps its
                // neighbours).
                if k + 1 < k_cut {
                    lo += v.norm_sqr();
                } else if k > k_cut + 1 {
                    hi += v.norm_sqr();
                }
            }
        }
    }
    (lo, hi)
}

fn decode(stream: &[u8]) -> Vec<i16> {
    let mut dec = StreamDecoder::new();
    dec.decode_all(stream)
        .unwrap()
        .iter()
        .flat_map(|f| f.pcm.iter().copied())
        .collect()
}

#[test]
fn default_bandwidth_follows_the_per_channel_rate() {
    let cfg = |bitrate: u32, channels: u8, sample_rate: u32| EncoderConfig {
        sample_rate,
        channels,
        bitrate,
    };
    assert!((cfg(64_000, 2, 44_100).default_bandwidth_hz() - 8_320.0).abs() < 1e-9);
    assert!((cfg(96_000, 2, 48_000).default_bandwidth_hz() - 12_480.0).abs() < 1e-9);
    assert!((cfg(128_000, 2, 44_100).default_bandwidth_hz() - 16_640.0).abs() < 1e-9);
    assert!((cfg(64_000, 1, 44_100).default_bandwidth_hz() - 16_640.0).abs() < 1e-9);
    // Ceiling, floor, and the Nyquist bound.
    assert_eq!(cfg(256_000, 2, 48_000).default_bandwidth_hz(), 20_000.0);
    assert_eq!(cfg(8_000, 2, 44_100).default_bandwidth_hz(), 4_000.0);
    assert_eq!(cfg(64_000, 1, 8_000).default_bandwidth_hz(), 4_000.0);
    let enc = StreamEncoder::new(cfg(64_000, 2, 44_100)).unwrap();
    assert_eq!(enc.bandwidth(), Some(8_320.0));
}

/// Full-band noise at 64 kbps stereo: the default cull leaves the
/// band above the 8.3 kHz cutoff essentially empty (the reconstructed
/// spectrum above it is ≥ 30 dB under the band below), a full-band
/// encoder keeps it within a few dB of the input's split, and the
/// culled stream spends its bits below the cutoff (lower noise
/// there than the full-band stream at the same rate).
#[test]
fn cull_moves_the_bits_below_the_cutoff() {
    let fs = 44_100u32;
    let n = 2 * 44_100;
    let pcm = noise_stereo(n);
    let cfg = EncoderConfig {
        sample_rate: fs,
        channels: 2,
        bitrate: 64_000,
    };
    let cutoff = cfg.default_bandwidth_hz();
    let (in_lo, in_hi) = split_energy(&pcm, fs, cutoff);

    let mut culled = StreamEncoder::new(cfg).unwrap();
    let s_culled = culled.encode_all(&pcm).unwrap();
    let mut full = StreamEncoder::new(cfg).unwrap();
    full.set_bandwidth(None);
    assert_eq!(full.bandwidth(), None);
    let s_full = full.encode_all(&pcm).unwrap();

    let out_c = decode(&s_culled);
    let out_f = decode(&s_full);
    let (c_lo, c_hi) = split_energy(&out_c, fs, cutoff);
    let (f_lo, f_hi) = split_energy(&out_f, fs, cutoff);
    let db = |x: f64, y: f64| 10.0 * (x / y.max(1e-30)).log10();
    eprintln!(
        "input hi/lo {:.1} dB; culled hi/lo {:.1} dB; full hi/lo {:.1} dB; {} vs {} bytes",
        db(in_hi, in_lo),
        db(c_hi, c_lo),
        db(f_hi, f_lo),
        s_culled.len(),
        s_full.len()
    );
    assert!(db(c_hi, c_lo) < -30.0, "culled stream keeps the top band");
    assert!(
        db(f_hi, f_lo) > db(in_hi, in_lo) - 6.0,
        "full-band stream lost the top band"
    );
    // Noise input at 64 kbps: the coding noise below the cutoff adds
    // to the band (NSR near 0 dB there), so the level sits within a
    // few dB of the input rather than on it.
    assert!(
        db(c_lo, in_lo).abs() < 4.0,
        "culled stream's low band level {:.1} dB",
        db(c_lo, in_lo)
    );

    // Noise below the cutoff, lag-aligned (the encoder delay is one
    // frame): the culled stream is the cleaner one there.
    let lag = 1024usize;
    let m = (n - lag)
        .min(out_c.len() / 2 - lag)
        .min(out_f.len() / 2 - lag);
    let noise_below = |out: &[i16]| -> f64 {
        let mut e = 0.0;
        let k_cut = (cutoff / (f64::from(fs) / 128.0)) as usize;
        for c in 0..2 {
            let a: Vec<f64> = pcm
                .iter()
                .skip(c)
                .step_by(2)
                .take(m)
                .map(|&v| f64::from(v))
                .collect();
            let b: Vec<f64> = out
                .iter()
                .skip(c + 2 * lag)
                .step_by(2)
                .take(m)
                .map(|&v| f64::from(v))
                .collect();
            let mut qa = EncoderAnalysisQmf::new();
            let mut qb = EncoderAnalysisQmf::new();
            for (sa, sb) in a.chunks_exact(64).zip(b.chunks_exact(64)) {
                let xa = qa.push_slot(sa).unwrap();
                let xb = qb.push_slot(sb).unwrap();
                for k in 1..k_cut {
                    e += (xb[k] - xa[k]).norm_sqr();
                }
            }
        }
        e
    };
    let (nc, nf) = (noise_below(&out_c), noise_below(&out_f));
    eprintln!(
        "noise below the cutoff: culled {:.1} dB vs full-band",
        db(nc, nf)
    );
    assert!(nc < nf, "culled {nc} vs full {nf}");
}

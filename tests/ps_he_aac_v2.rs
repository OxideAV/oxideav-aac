//! HE-AAC v2 (AAC-LC core + §4.6.18 SBR + §8.6.4 PS) end-to-end PCM
//! validation against the staged `expected.wav` reference.
//!
//! The fixture is an MP4 (`input.m4a`); a minimal ISO 14496-12
//! sample-table walk (`stsz`/`stco`/`stsc`) recovers the AAC access
//! units, which are fed as `raw_data_block()` payloads to the shared
//! [`StreamDecoder`] core at the fixture's AudioSpecificConfig
//! parameters (AOT 2, 16 kHz core, `channelConfiguration 1`). The SBR
//! auto-detect doubles the rate and the Annex 8.A PS tool synthesizes
//! the stereo image from the mono core, so every decoded frame is
//! 2 × 2048 samples at 32 kHz.
//!
//! PS reconstruction is parametric — the de-correlator output depends
//! on exactingly specified but numerically deep all-pass state — so
//! the comparison is in the §8 PCM-RMS domain per channel after lag
//! alignment (the MP4 edit list trims the encoder priming from the
//! reference; the raw decode keeps it).
//!
//! Skips (logged, success) when `docs/` is absent (standalone CI).

use std::fs;
use std::path::PathBuf;

use oxideav_aac::decode::StreamDecoder;

fn fixture_dir() -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures/he-aac-v2-stereo-32000-24kbps-m4a")
}

/// Read the `data` chunk of a 16-bit WAV as interleaved `i16`.
fn read_wav_s16(path: &PathBuf) -> Option<Vec<i16>> {
    let d = fs::read(path).ok()?;
    let mut i = 12;
    while i + 8 <= d.len() {
        let cid = &d[i..i + 4];
        let sz = u32::from_le_bytes([d[i + 4], d[i + 5], d[i + 6], d[i + 7]]) as usize;
        let body = i + 8;
        if cid == b"data" {
            let end = (body + sz).min(d.len());
            return Some(
                d[body..end]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect(),
            );
        }
        i = body + sz + (sz & 1);
    }
    None
}

/// Minimal MP4 box walk: return the byte ranges of the (sole) audio
/// track's samples via stsz + stco/co64 + stsc.
fn mp4_samples(data: &[u8]) -> Option<Vec<(usize, usize)>> {
    fn boxes(data: &[u8]) -> Vec<(&str, &[u8])> {
        let mut out = Vec::new();
        let mut i = 0usize;
        while i + 8 <= data.len() {
            let mut sz = u32::from_be_bytes(data[i..i + 4].try_into().unwrap()) as usize;
            let ty = std::str::from_utf8(&data[i + 4..i + 8]).unwrap_or("????");
            let mut body = i + 8;
            if sz == 1 {
                if i + 16 > data.len() {
                    break;
                }
                sz = u64::from_be_bytes(data[i + 8..i + 16].try_into().unwrap()) as usize;
                body = i + 16;
            } else if sz == 0 {
                sz = data.len() - i;
            }
            if sz < 8 || i + sz > data.len() {
                break;
            }
            out.push((ty, &data[body..i + sz]));
            i += sz;
        }
        out
    }
    fn find<'a>(data: &'a [u8], path: &[&str]) -> Option<&'a [u8]> {
        let mut cur = data;
        for want in path {
            cur = boxes(cur).into_iter().find(|(t, _)| t == want)?.1;
        }
        Some(cur)
    }
    let stbl = find(data, &["moov", "trak", "mdia", "minf", "stbl"])?;
    let stbl_boxes = boxes(stbl);
    let get = |name: &str| -> Option<&[u8]> {
        stbl_boxes.iter().find(|(t, _)| *t == name).map(|(_, b)| *b)
    };

    // stsz: version/flags, sample_size, sample_count, [sizes].
    let stsz = get("stsz")?;
    let fixed = u32::from_be_bytes(stsz[4..8].try_into().unwrap()) as usize;
    let count = u32::from_be_bytes(stsz[8..12].try_into().unwrap()) as usize;
    let sizes: Vec<usize> = if fixed != 0 {
        vec![fixed; count]
    } else {
        (0..count)
            .map(|i| u32::from_be_bytes(stsz[12 + 4 * i..16 + 4 * i].try_into().unwrap()) as usize)
            .collect()
    };

    // stco / co64.
    let offsets: Vec<usize> = if let Some(stco) = get("stco") {
        let n = u32::from_be_bytes(stco[4..8].try_into().unwrap()) as usize;
        (0..n)
            .map(|i| u32::from_be_bytes(stco[8 + 4 * i..12 + 4 * i].try_into().unwrap()) as usize)
            .collect()
    } else {
        let co64 = get("co64")?;
        let n = u32::from_be_bytes(co64[4..8].try_into().unwrap()) as usize;
        (0..n)
            .map(|i| u64::from_be_bytes(co64[8 + 8 * i..16 + 8 * i].try_into().unwrap()) as usize)
            .collect()
    };

    // stsc: (first_chunk, samples_per_chunk, _sdi) runs.
    let stsc = get("stsc")?;
    let n = u32::from_be_bytes(stsc[4..8].try_into().unwrap()) as usize;
    let entries: Vec<(usize, usize)> = (0..n)
        .map(|i| {
            let b = 8 + 12 * i;
            (
                u32::from_be_bytes(stsc[b..b + 4].try_into().unwrap()) as usize,
                u32::from_be_bytes(stsc[b + 4..b + 8].try_into().unwrap()) as usize,
            )
        })
        .collect();

    // Walk chunks, emitting per-sample (offset, size).
    let mut out = Vec::with_capacity(count);
    let mut sample = 0usize;
    for (ci, &chunk_off) in offsets.iter().enumerate() {
        let chunk_no = ci + 1;
        let spc = entries
            .iter()
            .rev()
            .find(|(first, _)| *first <= chunk_no)
            .map(|(_, spc)| *spc)?;
        let mut off = chunk_off;
        for _ in 0..spc {
            if sample >= count {
                break;
            }
            out.push((off, sizes[sample]));
            off += sizes[sample];
            sample += 1;
        }
    }
    (sample == count).then_some(out)
}

#[test]
fn he_aac_v2_ps_pcm_matches_reference() {
    let dir = fixture_dir();
    let Ok(m4a) = fs::read(dir.join("input.m4a")) else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let expected = read_wav_s16(&dir.join("expected.wav")).expect("expected.wav");
    let samples = mp4_samples(&m4a).expect("mp4 sample table");
    assert!(samples.len() > 10, "too few access units");

    // Fixture ASC: AOT 2 core at 16 kHz (index 8), mono config; the
    // SBR + PS extension upgrades each frame to 2048×2 @ 32 kHz.
    let mut dec = StreamDecoder::new();
    let mut ours: Vec<i16> = Vec::new();
    let mut ps_frames = 0usize;
    for &(off, len) in &samples {
        let au = &m4a[off..off + len];
        let f = dec
            .decode_raw_data_block(2, 8, 16_000, 1, 1, au)
            .expect("decode AU");
        assert_eq!(f.sample_rate, 32_000, "SBR output rate");
        if f.channels == 2 {
            ps_frames += 1;
            assert_eq!(f.pcm.len(), 2048 * 2);
        }
        ours.extend_from_slice(&f.pcm);
    }
    assert_eq!(
        ps_frames,
        samples.len(),
        "every frame must be PS stereo (the first ps_data carries a header)"
    );

    // Lag alignment: the reference has the encoder priming trimmed
    // (MP4 edit list); find the integer lag minimising the error of a
    // mid-stream window of the left channel, then compare both
    // channels at that lag.
    let n_exp = expected.len() / 2;
    let n_ours = ours.len() / 2;
    let win_lo = n_exp / 4;
    let win_hi = n_exp / 2;
    let max_lag = n_ours.saturating_sub(win_hi).min(16_000);
    let mut best = (f64::INFINITY, 0usize);
    for lag in 0..max_lag {
        let mut err = 0.0f64;
        let mut t = win_lo;
        while t < win_hi {
            let d = f64::from(ours[2 * (t + lag)]) - f64::from(expected[2 * t]);
            err += d * d;
            t += 3;
        }
        if err < best.0 {
            best = (err, lag);
        }
    }
    let lag = best.1;

    let mut err_sse = [0.0f64; 2];
    let mut sig_sse = [0.0f64; 2];
    let span = n_exp.min(n_ours - lag);
    for t in 0..span {
        for c in 0..2 {
            let a = f64::from(ours[2 * (t + lag) + c]);
            let b = f64::from(expected[2 * t + c]);
            err_sse[c] += (a - b) * (a - b);
            sig_sse[c] += b * b;
        }
    }
    let ratio_l = (err_sse[0] / sig_sse[0].max(1.0)).sqrt();
    let ratio_r = (err_sse[1] / sig_sse[1].max(1.0)).sqrt();
    eprintln!("he-aac-v2 PS: lag {lag}, err/sig RMS L {ratio_l:.5} R {ratio_r:.5}");

    // The whole chain is deterministic; the only divergence from the
    // reference is filterbank rounding (measured 5e-5), so pin an
    // order of magnitude above the observed level.
    assert!(ratio_l < 1e-3, "left error ratio {ratio_l:.6}");
    assert!(ratio_r < 1e-3, "right error ratio {ratio_r:.6}");
}

/// §4.6.18.8: the low-power SBR tool cannot host the complex-domain
/// PS processing — the v2 fixture's first PS-bearing access unit is
/// rejected with the dedicated error rather than decoded as mono.
#[test]
fn he_aac_v2_ps_rejected_in_low_power_mode() {
    let dir = fixture_dir();
    let Ok(m4a) = fs::read(dir.join("input.m4a")) else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let samples = mp4_samples(&m4a).expect("mp4 sample table");
    let mut dec = StreamDecoder::new();
    dec.set_sbr_low_power(true);
    let mut rejected = false;
    for &(off, len) in &samples {
        let au = &m4a[off..off + len];
        match dec.decode_raw_data_block(2, 8, 16_000, 1, 1, au) {
            Err(oxideav_aac::Error::SbrLowPowerPs) => {
                rejected = true;
                break;
            }
            Err(e) => panic!("unexpected error: {e}"),
            Ok(_) => {}
        }
    }
    assert!(rejected, "PS payload must be rejected in low-power mode");
}

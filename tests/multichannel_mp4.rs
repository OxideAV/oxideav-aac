//! 5.1 multichannel PCM validation against the staged
//! `aac-lc-5.1-48000-256kbps-mp4` fixture.
//!
//! The fixture is an MP4 (`input.m4a`, `channelConfiguration 6`); the
//! same minimal ISO 14496-12 sample-table walk the HE-AAC v2 test uses
//! (`stsz`/`stco`/`stsc`) recovers the access units, which are fed to
//! [`StreamDecoder::decode_raw_data_block`] at the fixture's ASC
//! parameters (AOT 2, 48 kHz, config 6). This pins the whole
//! multichannel path end to end: the SCE + CPE + CPE + LFE element
//! walk, the per-element state slots, and the §1.6.3.5 / Table 1.19
//! element→speaker reorder (`C,L,R,Ls,Rs,LFE` element order →
//! canonical `L,R,C,LFE,Ls,Rs` interleave) against a reference
//! `expected.wav` produced by an independent decoder.
//!
//! The stream carries §4.6.13 PNS bands (see the fixture trace), whose
//! per-bin noise phase is generator-defined — byte-exactness on those
//! bins is precluded by the standard itself (band *energy* is
//! normative; see `docs/audio/aac/pns-gen-rand-vector.md`), so the
//! comparison is the fixtures-doc §8 per-channel PCM-RMS check.
//!
//! Skips (logged, success) when `docs/` is absent (standalone CI).

use std::fs;
use std::path::PathBuf;

use oxideav_aac::decode::StreamDecoder;

fn fixture_dir() -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures/aac-lc-5.1-48000-256kbps-mp4")
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
fn aac_lc_5_1_pcm_matches_reference_per_channel() {
    const CH: usize = 6;
    let dir = fixture_dir();
    let Ok(m4a) = fs::read(dir.join("input.m4a")) else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let expected = read_wav_s16(&dir.join("expected.wav")).expect("expected.wav");
    let samples = mp4_samples(&m4a).expect("mp4 sample table");
    assert!(samples.len() > 10, "too few access units");

    // Fixture ASC: AOT 2, samplingFrequencyIndex 3 (48 kHz),
    // channelConfiguration 6 (see notes.md / trace.txt).
    let mut dec = StreamDecoder::new();
    let mut ours: Vec<i16> = Vec::new();
    for &(off, len) in &samples {
        let au = &m4a[off..off + len];
        let f = dec
            .decode_raw_data_block(2, 3, 48_000, 6, 1, au)
            .expect("decode AU");
        assert_eq!(f.sample_rate, 48_000);
        assert_eq!(f.channels, CH, "5.1 frame must carry 6 channels");
        assert_eq!(f.pcm.len(), 1024 * CH);
        ours.extend_from_slice(&f.pcm);
    }

    // Lag alignment: the MP4 edit list trims the encoder priming from
    // the reference; find the integer lag minimising the error of a
    // mid-stream window of output channel 0 (canonical L — the
    // 220 Hz source; a mismapped channel order could not null it).
    let n_exp = expected.len() / CH;
    let n_ours = ours.len() / CH;
    let win_lo = n_exp / 4;
    let win_hi = n_exp / 2;
    let max_lag = n_ours.saturating_sub(win_hi).min(8_192);
    let mut best = (f64::INFINITY, 0usize);
    for lag in 0..max_lag {
        let mut err = 0.0f64;
        let mut t = win_lo;
        while t < win_hi {
            let d = f64::from(ours[CH * (t + lag)]) - f64::from(expected[CH * t]);
            err += d * d;
            t += 3;
        }
        if err < best.0 {
            best = (err, lag);
        }
    }
    let lag = best.1;

    // Per-channel §8 PCM-RMS comparison. The stream carries PNS bands
    // (generator-defined noise phase, energy normative), so a byte
    // compare is precluded by the standard; the RMS bound pins the
    // deterministic remainder. The LFE channel is digital silence at
    // the source — bound it absolutely instead of relatively.
    let span = n_exp.min(n_ours - lag);
    let mut err_sse = [0.0f64; CH];
    let mut sig_sse = [0.0f64; CH];
    for t in 0..span {
        for c in 0..CH {
            let a = f64::from(ours[CH * (t + lag) + c]);
            let b = f64::from(expected[CH * t + c]);
            err_sse[c] += (a - b) * (a - b);
            sig_sse[c] += b * b;
        }
    }
    for c in 0..CH {
        let sig_rms = (sig_sse[c] / span as f64).sqrt();
        if sig_rms < 1.0 {
            // Silent reference channel (the LFE): ours must be
            // near-silent too.
            let our_rms = (err_sse[c] / span as f64).sqrt();
            eprintln!("5.1 ch{c}: silent reference, our residual RMS {our_rms:.3}");
            assert!(our_rms < 16.0, "channel {c} should be near-silent");
        } else {
            let ratio = (err_sse[c] / sig_sse[c]).sqrt();
            eprintln!("5.1 ch{c}: err/sig RMS = {ratio:.5} (lag {lag})");
            assert!(ratio < 5e-3, "channel {c} error ratio {ratio:.5}");
        }
    }

    // Channel identity: each canonical output channel carries its
    // source tone (L=220, R=330, C=440, Ls=550, Rs=660 Hz), so a
    // swapped reorder would explode the per-channel ratios above;
    // additionally pin that no non-LFE channel is silent.
    for (c, sse) in sig_sse.iter().enumerate() {
        if c != 3 {
            assert!(
                (sse / span as f64).sqrt() > 100.0,
                "reference channel {c} unexpectedly quiet"
            );
        }
    }
}

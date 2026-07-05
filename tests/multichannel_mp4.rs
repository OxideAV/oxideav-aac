//! Multichannel PCM validation against the staged MP4 fixtures:
//! 5.1 (`channelConfiguration 6`), 7.1 (`channelConfiguration 0` +
//! ASC-inline PCE), and the hexagonal custom layout
//! (`channelConfiguration 0` + PCE, no LFE).
//!
//! Each fixture is an MP4 (`input.m4a`); the same minimal ISO
//! 14496-12 sample-table walk the HE-AAC v2 test uses
//! (`stsz`/`stco`/`stsc`) recovers the access units, which are fed to
//! [`StreamDecoder::decode_raw_data_block`] at the fixture's ASC
//! parameters. This pins the whole multichannel path end to end: the
//! per-element state slots, the §1.6.3.5 / Table 1.19 element→speaker
//! reorder for config 6, and the ISO/IEC 13818-7 §8.5.2.2 PCE-driven
//! reorder for the config-0 layouts (the ASC's inline
//! `program_config_element()` extracted from the esds and installed
//! via [`StreamDecoder::set_program_config`]) — all against reference
//! `expected.wav` decodes produced by an independent decoder, with a
//! distinct source tone per speaker so any mis-mapped channel
//! explodes its per-channel error ratio.
//!
//! Streams may carry §4.6.13 PNS bands (see the fixture traces),
//! whose per-bin noise phase is generator-defined — byte-exactness on
//! those bins is precluded by the standard itself (band *energy* is
//! normative; see `docs/audio/aac/pns-gen-rand-vector.md`), so the
//! comparison is the fixtures-doc §8 per-channel PCM-RMS check.
//!
//! Skips (logged, success) when `docs/` is absent (standalone CI).

use std::fs;
use std::path::PathBuf;

use oxideav_aac::asc::AudioSpecificConfig;
use oxideav_aac::decode::StreamDecoder;

fn fixture_dir(name: &str) -> PathBuf {
    PathBuf::from("../../docs/audio/aac/fixtures").join(name)
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

/// Flat child-box list of an ISO 14496-12 container body.
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

/// Descend a box path.
fn find<'a>(data: &'a [u8], path: &[&str]) -> Option<&'a [u8]> {
    let mut cur = data;
    for want in path {
        cur = boxes(cur).into_iter().find(|(t, _)| t == want)?.1;
    }
    Some(cur)
}

/// Extract the AudioSpecificConfig bytes from the esds
/// DecoderSpecificInfo of the (sole) mp4a sample entry.
fn mp4_asc(data: &[u8]) -> Option<Vec<u8>> {
    let stsd = find(data, &["moov", "trak", "mdia", "minf", "stbl", "stsd"])?;
    // stsd: version/flags(4) + entry_count(4), then sample entries.
    let entries = &stsd[8..];
    let mp4a = boxes(entries).into_iter().find(|(t, _)| *t == "mp4a")?.1;
    // AudioSampleEntry: 6 reserved + 2 data_reference_index + 20 bytes
    // of audio fields = 28 bytes before the child boxes.
    let esds = boxes(&mp4a[28..])
        .into_iter()
        .find(|(t, _)| *t == "esds")?
        .1;
    // esds: version/flags(4), then the MPEG-4 descriptor tree with
    // 7-bit-continuation lengths: ES_Descr(0x03) → DecoderConfig
    // (0x04, 13 fixed bytes) → DecSpecificInfo(0x05) = the ASC.
    let d = &esds[4..];
    let mut i = 0usize;
    let rdlen = |d: &[u8], mut i: usize| -> Option<(usize, usize)> {
        let mut len = 0usize;
        loop {
            let b = *d.get(i)?;
            i += 1;
            len = (len << 7) | (b & 0x7f) as usize;
            if b & 0x80 == 0 {
                break;
            }
        }
        Some((len, i))
    };
    if *d.get(i)? != 0x03 {
        return None;
    }
    let (_, ni) = rdlen(d, i + 1)?;
    i = ni + 3; // ES_ID (2) + streamDependence/URL/OCR flags (1)
    if *d.get(i)? != 0x04 {
        return None;
    }
    let (_, ni) = rdlen(d, i + 1)?;
    i = ni + 13; // objectTypeIndication .. avgBitrate
    if *d.get(i)? != 0x05 {
        return None;
    }
    let (len, ni) = rdlen(d, i + 1)?;
    d.get(ni..ni + len).map(<[u8]>::to_vec)
}

/// Minimal MP4 box walk: return the byte ranges of the (sole) audio
/// track's samples via stsz + stco/co64 + stsc.
fn mp4_samples(data: &[u8]) -> Option<Vec<(usize, usize)>> {
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

/// Decode a multichannel MP4 fixture and compare per channel against
/// its `expected.wav` in the fixtures-doc §8 PCM-RMS domain.
///
/// * `ch` — channel count; every frame must carry exactly this many.
/// * `fs_index` / `sample_rate` / `channel_config` — the fixture's
///   ASC decode parameters.
/// * `use_pce` — install the ASC's inline `program_config_element()`
///   (extracted from the esds) before decoding, driving the §8.5.2.2
///   canonical reorder for `channel_config == 0`.
/// * `silent` — canonical channel indices whose *source* is digital
///   silence (the LFE of the tone fixtures): bounded absolutely.
fn validate_mc_fixture(
    name: &str,
    ch: usize,
    fs_index: u8,
    sample_rate: u32,
    channel_config: u8,
    use_pce: bool,
    silent: &[usize],
) {
    let dir = fixture_dir(name);
    let Ok(m4a) = fs::read(dir.join("input.m4a")) else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let expected = read_wav_s16(&dir.join("expected.wav")).expect("expected.wav");
    let samples = mp4_samples(&m4a).expect("mp4 sample table");
    assert!(samples.len() > 10, "too few access units");

    let mut dec = StreamDecoder::new();
    if use_pce {
        let asc_bytes = mp4_asc(&m4a).expect("esds AudioSpecificConfig");
        let (asc, _) = AudioSpecificConfig::parse(&asc_bytes).expect("ASC parses");
        assert_eq!(asc.channel_configuration, 0, "{name}: PCE-defined layout");
        let pce = asc.ga_body.pce.clone().expect("ASC-inline PCE");
        dec.set_program_config(pce);
    }
    let mut ours: Vec<i16> = Vec::new();
    for &(off, len) in &samples {
        let au = &m4a[off..off + len];
        let f = dec
            .decode_raw_data_block(2, fs_index, sample_rate, channel_config, 1, au)
            .expect("decode AU");
        assert_eq!(f.sample_rate, sample_rate);
        assert_eq!(f.channels, ch, "{name}: channel count");
        assert_eq!(f.pcm.len(), 1024 * ch);
        ours.extend_from_slice(&f.pcm);
    }

    // Lag alignment: the MP4 edit list trims the encoder priming from
    // the reference; find the integer lag minimising the error of a
    // mid-stream window of output channel 0 (a mis-mapped channel
    // order could not null it — every speaker carries its own tone).
    let n_exp = expected.len() / ch;
    let n_ours = ours.len() / ch;
    let win_lo = n_exp / 4;
    let win_hi = n_exp / 2;
    let max_lag = n_ours.saturating_sub(win_hi).min(8_192);
    let mut best = (f64::INFINITY, 0usize);
    for lag in 0..max_lag {
        let mut err = 0.0f64;
        let mut t = win_lo;
        while t < win_hi {
            let d = f64::from(ours[ch * (t + lag)]) - f64::from(expected[ch * t]);
            err += d * d;
            t += 3;
        }
        if err < best.0 {
            best = (err, lag);
        }
    }
    let lag = best.1;

    // Per-channel §8 PCM-RMS comparison. The streams carry PNS bands
    // (generator-defined noise phase, energy normative), so a byte
    // compare is precluded by the standard; the RMS bound pins the
    // deterministic remainder.
    let span = n_exp.min(n_ours - lag);
    let mut err_sse = vec![0.0f64; ch];
    let mut sig_sse = vec![0.0f64; ch];
    for t in 0..span {
        for c in 0..ch {
            let a = f64::from(ours[ch * (t + lag) + c]);
            let b = f64::from(expected[ch * t + c]);
            err_sse[c] += (a - b) * (a - b);
            sig_sse[c] += b * b;
        }
    }
    for c in 0..ch {
        let sig_rms = (sig_sse[c] / span as f64).sqrt();
        if silent.contains(&c) {
            assert!(sig_rms < 1.0, "{name} ch{c}: reference should be silent");
            let our_rms = (err_sse[c] / span as f64).sqrt();
            eprintln!("{name} ch{c}: silent reference, our residual RMS {our_rms:.3}");
            assert!(our_rms < 16.0, "{name} ch{c} should be near-silent");
        } else {
            assert!(
                sig_rms > 100.0,
                "{name}: reference channel {c} unexpectedly quiet"
            );
            let ratio = (err_sse[c] / sig_sse[c]).sqrt();
            eprintln!("{name} ch{c}: err/sig RMS = {ratio:.5} (lag {lag})");
            assert!(ratio < 5e-3, "{name} ch{c} error ratio {ratio:.5}");
        }
    }
}

#[test]
fn aac_lc_5_1_pcm_matches_reference_per_channel() {
    // channelConfiguration 6: the Table 1.19 default reorder.
    // Canonical L R C LFE Ls Rs = source tones 220/330/440/-/550/660.
    validate_mc_fixture("aac-lc-5.1-48000-256kbps-mp4", 6, 3, 48_000, 6, false, &[3]);
}

#[test]
fn aac_lc_7_1_pce_pcm_matches_reference_per_channel() {
    // channelConfiguration 0 with the PCE inline in the ASC:
    // front [SCE(C), CPE(L/R)], back [CPE, CPE] (§8.5.2.2 outside
    // in: side-surround pair then rear pair), LFE. Canonical
    // FL FR FC LFE BL BR SL SR = tones 220/330/440/-/550/660/770/880.
    validate_mc_fixture("aac-lc-7.1-48000-320kbps-mp4", 8, 3, 48_000, 0, true, &[3]);
}

#[test]
fn aac_lc_hexagonal_pce_pcm_matches_reference_per_channel() {
    // channelConfiguration 0, custom 6.0 layout: front [CPE, SCE]
    // (the lone front SCE is the center), back [CPE, SCE] (the final
    // unpaired back SCE is the rear center), no LFE. Canonical
    // FL FR FC BL BR BC = tones 220/330/440/550/660/770.
    validate_mc_fixture("aac-lc-hexagonal-with-pce", 6, 4, 44_100, 0, true, &[]);
}

//! ER AAC LD black-box round trip (skip-if-absent): the `fdkaac`
//! encoder binary produces ER AAC LD 512 streams that switch TNS on in
//! a large share of their access units (up to two filters, `length`
//! up to 29, `order` up to 7 — records only the §4.5.2.4.2 category-3
//! placement at the literal Table 4.155 widths can carry), and the
//! `afconvert` decoder binary decodes them; this crate's decode must
//! agree within 1 LSB on every sample, TNS-bearing access units
//! included. Mono and stereo, 48 and 22.05 kHz. Runs only where both
//! binaries are installed (the black-box tools are invoked as opaque
//! oracles; nothing from them is vendored).

use std::fs;
use std::path::Path;
use std::process::Command;

use oxideav_aac::asc::AudioSpecificConfig;
use oxideav_aac::decode::StreamDecoder;

fn tool(name: &str, probe: &[&str]) -> bool {
    Command::new(name)
        .args(probe)
        .output()
        .is_ok_and(|o| o.status.success() || !o.stdout.is_empty() || !o.stderr.is_empty())
}

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

/// AudioSpecificConfig bytes out of the esds DecoderSpecificInfo.
fn mp4_asc(data: &[u8]) -> Option<Vec<u8>> {
    let stsd = find(data, &["moov", "trak", "mdia", "minf", "stbl", "stsd"])?;
    let entries = &stsd[8..];
    let mp4a = boxes(entries).into_iter().find(|(t, _)| *t == "mp4a")?.1;
    let esds = boxes(&mp4a[28..])
        .into_iter()
        .find(|(t, _)| *t == "esds")?
        .1;
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
    i = ni + 3;
    if *d.get(i)? != 0x04 {
        return None;
    }
    let (_, ni) = rdlen(d, i + 1)?;
    i = ni + 13;
    if *d.get(i)? != 0x05 {
        return None;
    }
    let (len, ni) = rdlen(d, i + 1)?;
    d.get(ni..ni + len).map(<[u8]>::to_vec)
}

/// Byte ranges of the (sole) audio track's samples.
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

fn write_wav(path: &Path, pcm: &[i16], rate: u32, ch: u16) {
    let mut o = Vec::with_capacity(44 + 2 * pcm.len());
    let dl = (pcm.len() * 2) as u32;
    o.extend_from_slice(b"RIFF");
    o.extend_from_slice(&(36 + dl).to_le_bytes());
    o.extend_from_slice(b"WAVEfmt ");
    o.extend_from_slice(&16u32.to_le_bytes());
    o.extend_from_slice(&1u16.to_le_bytes());
    o.extend_from_slice(&ch.to_le_bytes());
    o.extend_from_slice(&rate.to_le_bytes());
    o.extend_from_slice(&(rate * 2 * u32::from(ch)).to_le_bytes());
    o.extend_from_slice(&(2 * ch).to_le_bytes());
    o.extend_from_slice(&16u16.to_le_bytes());
    o.extend_from_slice(b"data");
    o.extend_from_slice(&dl.to_le_bytes());
    for s in pcm {
        o.extend_from_slice(&s.to_le_bytes());
    }
    fs::write(path, o).unwrap();
}

fn read_wav_data(path: &Path) -> Vec<i16> {
    let d = fs::read(path).unwrap();
    let mut i = 12;
    while i + 8 <= d.len() {
        let sz = u32::from_le_bytes(d[i + 4..i + 8].try_into().unwrap()) as usize;
        if &d[i..i + 4] == b"data" {
            let end = (i + 8 + sz).min(d.len());
            return d[i + 8..end]
                .as_chunks::<2>()
                .0
                .iter()
                .map(|b| i16::from_le_bytes(*b))
                .collect();
        }
        i += 8 + sz + (sz & 1);
    }
    Vec::new()
}

/// Tone bursts, decaying noise bursts and a sweep — material the
/// black-box encoder answers with TNS filters.
fn material(rate: u32, ch: u16) -> Vec<i16> {
    let n = 3 * rate as usize;
    let period = rate as usize / 5;
    let mut seed = 0x1d7a_5eedu32;
    let mut out = Vec::with_capacity(n * usize::from(ch));
    for i in 0..n {
        let t = i as f64 / f64::from(rate);
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let noise = (seed as i32) as f64 / 2_147_483_648.0;
        let ph = i % period;
        let burst = if ph < period / 10 {
            12_000.0 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin()
        } else {
            12_000.0 * (-((ph - period / 10) as f64) / 200.0).exp() * noise
        };
        let sweep = 4000.0 * (2.0 * std::f64::consts::PI * (440.0 * t + 300.0 * t * t)).sin();
        for c in 0..ch {
            let v = if c == 0 {
                burst + sweep
            } else {
                burst * 0.5 + sweep
            };
            out.push(v.clamp(-32768.0, 32767.0) as i16);
        }
    }
    out
}

#[test]
fn black_box_ld_streams_with_tns_decode_within_one_lsb() {
    if !tool("fdkaac", &["--help"]) || !tool("afconvert", &["-h"]) {
        eprintln!("skip: fdkaac / afconvert not installed");
        return;
    }
    let dir = std::env::temp_dir().join("oxideav-aac-er-ld-black-box");
    fs::create_dir_all(&dir).unwrap();
    for (rate, ch) in [(48_000u32, 1u16), (48_000, 2), (22_050, 1)] {
        let stem = dir.join(format!("ld_{rate}_{ch}"));
        let wav = stem.with_extension("wav");
        let m4a = stem.with_extension("m4a");
        let reference = stem.with_extension("ref.wav");
        write_wav(&wav, &material(rate, ch), rate, ch);
        let bitrate = (64_000 * u32::from(ch)).to_string();
        let enc = Command::new("fdkaac")
            .args(["-S", "-p", "23", "-b", &bitrate, "-o"])
            .arg(&m4a)
            .arg(&wav)
            .output()
            .unwrap();
        assert!(
            enc.status.success(),
            "{}",
            String::from_utf8_lossy(&enc.stderr)
        );
        let dec = Command::new("afconvert")
            .args(["-f", "WAVE", "-d", "LEI16"])
            .arg(&m4a)
            .arg(&reference)
            .output()
            .unwrap();
        assert!(
            dec.status.success(),
            "{}",
            String::from_utf8_lossy(&dec.stderr)
        );

        let data = fs::read(&m4a).unwrap();
        let (asc, _) = AudioSpecificConfig::parse(&mp4_asc(&data).unwrap()).unwrap();
        assert_eq!(asc.aot, 23);
        let resilience = asc
            .ga_body
            .extension_body
            .as_ref()
            .and_then(|b| b.resilience)
            .unwrap_or_default();
        let family = asc.ga_body.frame_length.family(asc.aot);
        let mut sd = StreamDecoder::new();
        sd.set_frame_family(family);
        let mut ours = Vec::new();
        let mut tns_aus = 0usize;
        let samples = mp4_samples(&data).unwrap();
        for &(off, len) in &samples {
            let au = &data[off..off + len];
            // TNS census on the mono SCE (4-bit tag, then the body).
            let mut r = oxideav_core::bits::BitReader::new(au);
            if ch == 1
                && r.read_u32(4).is_ok()
                && oxideav_aac::ics_body::IcsBody::parse_er_family(
                    &mut r,
                    family,
                    23,
                    asc.sampling_frequency_index,
                    false,
                    resilience,
                )
                .is_ok_and(|b| b.tns_data_present)
            {
                tns_aus += 1;
            }
            let f = sd
                .decode_er_raw_data_block(
                    23,
                    asc.sampling_frequency_index,
                    asc.sample_rate,
                    asc.channel_configuration,
                    resilience,
                    au,
                )
                .unwrap();
            ours.extend_from_slice(&f.pcm);
        }
        let theirs = read_wav_data(&reference);
        // The reference trims one priming frame.
        let lag = family.frame_len() * usize::from(ch);
        let n = theirs.len().min(ours.len().saturating_sub(lag));
        assert!(n > theirs.len() / 2, "{rate}/{ch}: {n} comparable samples");
        let max = (0..n)
            .map(|i| (i32::from(ours[i + lag]) - i32::from(theirs[i])).abs())
            .max()
            .unwrap();
        eprintln!(
            "{rate} Hz {ch} ch: {} AUs ({tns_aus} TNS-bearing mono AUs), max |diff| {max}",
            samples.len()
        );
        if ch == 1 {
            assert!(
                tns_aus > samples.len() / 10,
                "{rate}: only {tns_aus} TNS AUs"
            );
        }
        assert!(max <= 1, "{rate} Hz {ch} ch: max diff {max}");
    }
}

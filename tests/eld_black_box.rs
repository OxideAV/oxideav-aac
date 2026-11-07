//! ER AAC ELD / LD black-box checks (skip-if-absent): the `afconvert`
//! binary is the opaque oracle (nothing from it is vendored).
//!
//! * AAC-ELD streams `afconvert` encodes decode through this crate's
//!   ELD path (§4.6.20) within 1 LSB of the `afconvert` decode;
//! * the crate's own ER AAC LD and ELD encoder streams, carried in a
//!   minimal MP4, decode through `afconvert` within 1 LSB of this
//!   crate's decode, at both frame lengths.

use std::fs;
use std::path::Path;
use std::process::Command;

use oxideav_aac::asc::{AacResilienceFlags, AudioSpecificConfig};
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};

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

fn bx(ty: &[u8; 4], body: &[u8]) -> Vec<u8> {
    let mut v = ((body.len() + 8) as u32).to_be_bytes().to_vec();
    v.extend_from_slice(ty);
    v.extend_from_slice(body);
    v
}

fn fullbx(ty: &[u8; 4], vf: u32, body: &[u8]) -> Vec<u8> {
    let mut b = vf.to_be_bytes().to_vec();
    b.extend_from_slice(body);
    bx(ty, &b)
}

fn desc(tag: u8, body: &[u8]) -> Vec<u8> {
    let mut v = vec![tag, 0x80, 0x80, 0x80, body.len() as u8];
    v.extend_from_slice(body);
    v
}

fn write_m4a(path: &Path, asc: &[u8], aus: &[Vec<u8>], rate: u32, ch: u16, frame_len: u32) {
    let n = aus.len() as u32;
    let ftyp = bx(b"ftyp", b"M4A \0\0\0\0M4A mp42isom");
    let mut mvhd = vec![0u8; 96];
    mvhd[8..12].copy_from_slice(&rate.to_be_bytes());
    mvhd[12..16].copy_from_slice(&(n * frame_len).to_be_bytes());
    mvhd[16..20].copy_from_slice(&0x0001_0000u32.to_be_bytes());
    mvhd[20..22].copy_from_slice(&0x0100u16.to_be_bytes());
    for (i, v) in [0x0001_0000u32, 0, 0, 0, 0x0001_0000, 0, 0, 0, 0x4000_0000]
        .iter()
        .enumerate()
    {
        mvhd[32 + 4 * i..36 + 4 * i].copy_from_slice(&v.to_be_bytes());
    }
    mvhd[92..96].copy_from_slice(&2u32.to_be_bytes());
    let mvhd = fullbx(b"mvhd", 0, &mvhd);
    let mut tkhd = vec![0u8; 80];
    tkhd[8..12].copy_from_slice(&1u32.to_be_bytes());
    tkhd[16..20].copy_from_slice(&(n * frame_len).to_be_bytes());
    tkhd[32..34].copy_from_slice(&0x0100u16.to_be_bytes());
    for (i, v) in [0x0001_0000u32, 0, 0, 0, 0x0001_0000, 0, 0, 0, 0x4000_0000]
        .iter()
        .enumerate()
    {
        tkhd[36 + 4 * i..40 + 4 * i].copy_from_slice(&v.to_be_bytes());
    }
    let tkhd = fullbx(b"tkhd", 7, &tkhd);
    let mut mdhd = vec![0u8; 20];
    mdhd[8..12].copy_from_slice(&rate.to_be_bytes());
    mdhd[12..16].copy_from_slice(&(n * frame_len).to_be_bytes());
    mdhd[16..18].copy_from_slice(&0x55c4u16.to_be_bytes());
    let mdhd = fullbx(b"mdhd", 0, &mdhd);
    let mut hdlr = vec![0u8; 4];
    hdlr.extend_from_slice(b"soun");
    hdlr.extend_from_slice(&[0u8; 12]);
    hdlr.extend_from_slice(b"snd\0");
    let hdlr = fullbx(b"hdlr", 0, &hdlr);
    let smhd = fullbx(b"smhd", 0, &[0u8; 4]);
    let dref = fullbx(
        b"dref",
        0,
        &[&1u32.to_be_bytes()[..], &fullbx(b"url ", 1, &[])].concat(),
    );
    let dinf = bx(b"dinf", &dref);
    let dsi = desc(0x05, asc);
    let mut dcd = vec![0x40, 0x15, 0, 0x18, 0];
    dcd.extend_from_slice(&0u32.to_be_bytes());
    dcd.extend_from_slice(&0u32.to_be_bytes());
    dcd.extend_from_slice(&dsi);
    let dcd = desc(0x04, &dcd);
    let mut es = vec![0, 1, 0];
    es.extend_from_slice(&dcd);
    es.extend_from_slice(&desc(0x06, &[2]));
    let esds = fullbx(b"esds", 0, &desc(0x03, &es));
    let mut mp4a = vec![0u8; 28];
    mp4a[7] = 1;
    mp4a[16..18].copy_from_slice(&ch.to_be_bytes());
    mp4a[18..20].copy_from_slice(&16u16.to_be_bytes());
    mp4a[24..28].copy_from_slice(&(rate << 16).to_be_bytes());
    mp4a.extend_from_slice(&esds);
    let stsd = fullbx(
        b"stsd",
        0,
        &[&1u32.to_be_bytes()[..], &bx(b"mp4a", &mp4a)].concat(),
    );
    let stts = fullbx(
        b"stts",
        0,
        &[1u32.to_be_bytes(), n.to_be_bytes(), frame_len.to_be_bytes()].concat(),
    );
    let stsc = fullbx(
        b"stsc",
        0,
        &[
            1u32.to_be_bytes(),
            1u32.to_be_bytes(),
            n.to_be_bytes(),
            1u32.to_be_bytes(),
        ]
        .concat(),
    );
    let mut stsz = vec![0u8; 4];
    stsz.extend_from_slice(&n.to_be_bytes());
    for a in aus {
        stsz.extend_from_slice(&(a.len() as u32).to_be_bytes());
    }
    let stsz = fullbx(b"stsz", 0, &stsz);
    let build = |off: u32| {
        let stco = fullbx(
            b"stco",
            0,
            &[1u32.to_be_bytes(), off.to_be_bytes()].concat(),
        );
        let stbl = bx(
            b"stbl",
            &[stsd.clone(), stts.clone(), stsc.clone(), stsz.clone(), stco].concat(),
        );
        let minf = bx(b"minf", &[smhd.clone(), dinf.clone(), stbl].concat());
        let mdia = bx(b"mdia", &[mdhd.clone(), hdlr.clone(), minf].concat());
        let trak = bx(b"trak", &[tkhd.clone(), mdia].concat());
        bx(b"moov", &[mvhd.clone(), trak].concat())
    };
    let moov_len = build(0).len();
    let off = (ftyp.len() + moov_len + 8) as u32;
    let moov = build(off);
    let mdat = bx(b"mdat", &aus.concat());
    fs::write(path, [ftyp, moov, mdat].concat()).unwrap();
}

/// Max |difference| between `ours` (delayed by the decoder) and
/// `theirs` at the best alignment within `max_lag` frames either way.
fn best_aligned_max_diff(ours: &[i16], theirs: &[i16], ch: usize, max_lag: usize) -> i32 {
    let mut best = (i64::MAX, i32::MAX);
    for lagf in -(max_lag as isize)..max_lag as isize {
        let lag = lagf * ch as isize;
        let (a, b): (&[i16], &[i16]) = if lag >= 0 {
            (ours.get(lag as usize..).unwrap_or(&[]), theirs)
        } else {
            (ours, theirs.get((-lag) as usize..).unwrap_or(&[]))
        };
        let n = a.len().min(b.len());
        if n < theirs.len() / 2 {
            continue;
        }
        let (mut se, mut mx) = (0i64, 0i32);
        for i in 0..n {
            let d = i32::from(a[i]) - i32::from(b[i]);
            se += i64::from(d * d);
            mx = mx.max(d.abs());
        }
        if se < best.0 {
            best = (se, mx);
        }
    }
    best.1
}

fn decode_aus(asc: &AudioSpecificConfig, aus: &[&[u8]]) -> Vec<i16> {
    let mut sd = StreamDecoder::new();
    sd.set_frame_family(asc.ga_body.frame_length.family(asc.aot));
    let mut out = Vec::new();
    for au in aus {
        let f = sd
            .decode_er_raw_data_block(
                asc.aot,
                asc.sampling_frequency_index,
                asc.sample_rate,
                asc.channel_configuration,
                AacResilienceFlags::default(),
                au,
            )
            .unwrap();
        out.extend_from_slice(&f.pcm);
    }
    out
}

fn afconvert_decode(m4a: &Path, wav: &Path) -> Vec<i16> {
    let dec = Command::new("afconvert")
        .args(["-f", "WAVE", "-d", "LEI16"])
        .arg(m4a)
        .arg(wav)
        .output()
        .unwrap();
    assert!(
        dec.status.success(),
        "{}",
        String::from_utf8_lossy(&dec.stderr)
    );
    read_wav_data(wav)
}

#[test]
fn black_box_eld_streams_decode_within_one_lsb() {
    if !tool("afconvert", &["-h"]) {
        eprintln!("skip: afconvert not installed");
        return;
    }
    let dir = std::env::temp_dir().join("oxideav-aac-eld-black-box");
    fs::create_dir_all(&dir).unwrap();
    for (rate, ch) in [(48_000u32, 1u16), (48_000, 2), (22_050, 1)] {
        let stem = dir.join(format!("eld_{rate}_{ch}"));
        let wav = stem.with_extension("wav");
        let m4a = stem.with_extension("m4a");
        write_wav(&wav, &material(rate, ch), rate, ch);
        let bitrate = (48_000 * u32::from(ch)).to_string();
        let enc = Command::new("afconvert")
            .args(["-f", "m4af", "-d", "aace", "-b", &bitrate])
            .arg(&wav)
            .arg(&m4a)
            .output()
            .unwrap();
        assert!(
            enc.status.success(),
            "{}",
            String::from_utf8_lossy(&enc.stderr)
        );
        let theirs = afconvert_decode(&m4a, &stem.with_extension("ref.wav"));
        let data = fs::read(&m4a).unwrap();
        let (asc, _) = AudioSpecificConfig::parse(&mp4_asc(&data).unwrap()).unwrap();
        assert_eq!(asc.aot, 39);
        let aus: Vec<&[u8]> = mp4_samples(&data)
            .unwrap()
            .into_iter()
            .map(|(off, len)| &data[off..off + len])
            .collect();
        let ours = decode_aus(&asc, &aus);
        let n = asc.ga_body.frame_length.family(asc.aot).frame_len();
        let max = best_aligned_max_diff(&ours, &theirs, usize::from(ch), 4 * n);
        eprintln!("ELD {rate} Hz {ch} ch: {} AUs, max |diff| {max}", aus.len());
        assert!(max <= 1, "ELD {rate} Hz {ch} ch: max diff {max}");
    }
}

#[test]
fn ld_and_eld_encoder_streams_decode_through_afconvert() {
    if !tool("afconvert", &["-h"]) {
        eprintln!("skip: afconvert not installed");
        return;
    }
    let dir = std::env::temp_dir().join("oxideav-aac-eld-black-box");
    fs::create_dir_all(&dir).unwrap();
    for eld in [false, true] {
        for frame_len in [512usize, 480] {
            for (rate, ch) in [(48_000u32, 2u16), (22_050, 1)] {
                let c = usize::from(ch);
                let pcm = material(rate, ch);
                let config = EncoderConfig::new(rate, ch as u8, 64_000 * u32::from(ch));
                let mut enc = if eld {
                    StreamEncoder::new_eld(config, frame_len)
                } else {
                    StreamEncoder::new_ld(config, frame_len)
                }
                .unwrap();
                let mut aus = Vec::new();
                for hop in pcm.chunks(frame_len * c) {
                    aus.push(enc.encode_access_unit(hop).unwrap());
                }
                aus.push(enc.finish_access_unit().unwrap());
                let asc_bytes = enc.audio_specific_config().unwrap();
                let (asc, _) = AudioSpecificConfig::parse(&asc_bytes).unwrap();
                assert_eq!(asc.aot, if eld { 39 } else { 23 });
                let stem = dir.join(format!("enc_{eld}_{frame_len}_{rate}_{ch}"));
                let m4a = stem.with_extension("m4a");
                write_m4a(&m4a, &asc_bytes, &aus, rate, ch, frame_len as u32);
                let theirs = afconvert_decode(&m4a, &stem.with_extension("ref.wav"));
                let refs: Vec<&[u8]> = aus.iter().map(Vec::as_slice).collect();
                let ours = decode_aus(&asc, &refs);
                let max = best_aligned_max_diff(&ours, &theirs, c, 3 * frame_len);
                let kind = if eld { "ELD" } else { "LD" };
                eprintln!("{kind} encoder N={frame_len} {rate} Hz {ch} ch: max |diff| {max}");
                assert!(max <= 1, "{kind} N={frame_len} {rate} Hz {ch} ch: {max}");
            }
        }
    }
}

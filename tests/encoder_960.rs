//! The 960-line AAC-LC encoder core (`frameLengthFlag == 1`,
//! ISO/IEC 14496-3 §4.5.1.1): `StreamEncoder::set_frame_family(Lc960)`
//! runs the whole analysis / quantization chain over the
//! `N = 1920 / 240` transform pair and the bracketed Table 4.129–4.141
//! band columns, emitting bare access units configured by
//! `audio_specific_config()`.
//!
//! The pins:
//!
//! * the access units decode through this crate's 960-line decoder
//!   (raw AUs + ASC, and the runtime decoder with the ASC as
//!   extradata) with 960 samples per channel per frame;
//! * coding quality per QMF band — the noise-to-mask ratio of the
//!   equal-rate harness (`tests/encoder_psy_harness.rs`: per-band
//!   noise against a 10 dB / 12 dB-per-band spreading masker) — stays
//!   within a small margin of the 1024-line path at the same rate on
//!   the same material (tonal mix with clicks and a sweep, and a
//!   transient burst train that drives block switching);
//! * when the black-box `ffmpeg` binary is installed, it decodes the
//!   LOAS-wrapped 960-line streams without diagnostics within 1 LSB
//!   of this crate's decode (skip-if-absent).

use std::fs;
use std::path::PathBuf;
use std::process::Command;

use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};
use oxideav_aac::latm_writer::LoasWriter;
use oxideav_aac::sbr_qmf::EncoderAnalysisQmf;
use oxideav_aac::swb_offset::FrameFamily;
use oxideav_core::{CodecId, CodecParameters, Error, Frame, Packet, TimeBase};

const FS: u32 = 48_000;
const FS_INDEX: u8 = 3;

/// Material length and the rate matrix: the full 2 s × three-rate
/// measurement under an optimised build, a 1 s single-rate pass under
/// a debug build (the analysis transform is a direct O(N²) MDCT).
const SECONDS: f64 = if cfg!(debug_assertions) { 1.0 } else { 2.0 };
const RATES: &[u32] = if cfg!(debug_assertions) {
    &[96_000]
} else {
    &[64_000, 96_000, 128_000]
};

/// Tonal mix + noise bed + click train + sweep (stereo).
fn mixed(seconds: f64) -> Vec<i16> {
    let n = (seconds * f64::from(FS)) as usize;
    let mut seed = 0x5eed_0a5cu32;
    let period = (0.3 * f64::from(FS)) as usize;
    let mut out = Vec::with_capacity(2 * n);
    for i in 0..n {
        let t = i as f64 / f64::from(FS);
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let noise = f64::from(seed >> 8) / f64::from(1u32 << 24) - 0.5;
        let mut tone = 0.0;
        for (h, a) in [
            (220.0, 4000.0),
            (330.0, 2800.0),
            (440.0, 2200.0),
            (880.0, 1200.0),
            (2640.0, 600.0),
            (5280.0, 300.0),
        ] {
            tone += a * (2.0 * std::f64::consts::PI * h * t).sin();
        }
        let sweep = 1500.0 * (2.0 * std::f64::consts::PI * (300.0 * t + 2000.0 * t * t)).sin();
        let click = if i % period < 96 {
            12_000.0 * (1.0 - (i % period) as f64 / 96.0)
        } else {
            0.0
        };
        let l = tone + sweep * 0.6 + click + noise * 1200.0;
        let r = tone * 0.7 + sweep + click * 0.5 + noise * 960.0;
        out.push(l.clamp(-32768.0, 32767.0) as i16);
        out.push(r.clamp(-32768.0, 32767.0) as i16);
    }
    out
}

/// Isolated sharp attacks (decaying noise every 0.25 s) over a quiet
/// tonal bed — the onsets that drive block switching.
fn attacks(seconds: f64) -> Vec<i16> {
    let n = (seconds * f64::from(FS)) as usize;
    let period = FS as usize / 4;
    let mut seed = 0x0a77_ac45u32;
    let mut out = Vec::with_capacity(2 * n);
    for i in 0..n {
        let t = i as f64 / f64::from(FS);
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let noise = (seed as i32) as f64 / 2_147_483_648.0;
        let burst = 20_000.0 * (-((i % period) as f64) / 400.0).exp() * noise;
        let bed = 300.0 * (2.0 * std::f64::consts::PI * 440.0 * t).sin();
        out.push((burst + bed).clamp(-32768.0, 32767.0) as i16);
        out.push((burst * 0.8 + bed).clamp(-32768.0, 32767.0) as i16);
    }
    out
}

/// Decaying noise bursts every 1024 samples over a tonal bed.
fn transients(seconds: f64) -> Vec<i16> {
    let n = (seconds * f64::from(FS)) as usize;
    let mut seed = 0x7a5e_1234u32;
    let mut out = Vec::with_capacity(2 * n);
    for i in 0..n {
        let t = i as f64 / f64::from(FS);
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let noise = (seed as i32) as f64 / 2_147_483_648.0;
        let burst = 14_000.0 * (-((i % 1024) as f64) / 150.0).exp() * noise;
        let bed = 1500.0 * (2.0 * std::f64::consts::PI * 523.0 * t).sin()
            + 800.0 * (2.0 * std::f64::consts::PI * 1568.0 * t).sin();
        out.push((burst + bed).clamp(-32768.0, 32767.0) as i16);
        out.push((burst * 0.6 + bed * 1.2).clamp(-32768.0, 32767.0) as i16);
    }
    out
}

struct Coded {
    aus: Vec<Vec<u8>>,
    asc: Vec<u8>,
    frame_len: usize,
}

fn encode(pcm: &[i16], bitrate: u32, family: FrameFamily) -> Coded {
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: FS,
        channels: 2,
        bitrate,
    })
    .unwrap();
    enc.set_frame_family(family).unwrap();
    let aus = enc.encode_all_access_units(pcm).unwrap();
    Coded {
        aus,
        asc: enc.audio_specific_config().unwrap(),
        frame_len: family.frame_len(),
    }
}

fn own_decode(c: &Coded) -> Vec<i16> {
    let mut dec = StreamDecoder::new();
    dec.set_frame_family(if c.frame_len == 960 {
        FrameFamily::Lc960
    } else {
        FrameFamily::Lc1024
    });
    let mut pcm = Vec::new();
    for au in &c.aus {
        let f = dec
            .decode_raw_data_block(2, FS_INDEX, FS, 2, 1, au)
            .unwrap();
        assert_eq!(f.pcm.len(), 2 * c.frame_len);
        pcm.extend_from_slice(&f.pcm);
    }
    pcm
}

fn kbps(c: &Coded) -> f64 {
    let bytes: usize = c.aus.iter().map(Vec::len).sum();
    bytes as f64 * 8.0 * f64::from(FS) / (c.aus.len() * c.frame_len) as f64 / 1000.0
}

/// Mean NMR (dB) over the signal-carrying 64-band QMF bands of the
/// decode against the input (the decode lags the input by exactly
/// one frame — the encoder's priming frame).
fn mean_nmr(input: &[i16], decoded: &[i16], frame_len: usize) -> f64 {
    let out = &decoded[2 * frame_len..];
    let m = input.len().min(out.len()) / 2;
    let (mut sig, mut noise) = ([0.0f64; 64], [0.0f64; 64]);
    for c in 0..2 {
        let a: Vec<f64> = input
            .iter()
            .skip(c)
            .step_by(2)
            .take(m)
            .map(|&v| f64::from(v))
            .collect();
        let b: Vec<f64> = out
            .iter()
            .skip(c)
            .step_by(2)
            .take(m)
            .map(|&v| f64::from(v))
            .collect();
        let (mut qa, mut qb) = (EncoderAnalysisQmf::new(), EncoderAnalysisQmf::new());
        for (sa, sb) in a.as_chunks::<64>().0.iter().zip(b.as_chunks::<64>().0) {
            let xa = qa.push_slot(sa).unwrap();
            let xb = qb.push_slot(sb).unwrap();
            for k in 0..64 {
                sig[k] += xa[k].norm_sqr();
                noise[k] += (xb[k] - xa[k]).norm_sqr();
            }
        }
    }
    let floor = sig.iter().copied().fold(0.0, f64::max) * 1e-6;
    let (mut sum, mut n) = (0.0f64, 0usize);
    for k in 0..64 {
        if sig[k] < floor {
            continue;
        }
        let thr = (0..64)
            .map(|j| sig[j] * 10f64.powf(-(1.0 + 1.2 * (k as f64 - j as f64).abs())))
            .fold(0.0f64, f64::max);
        sum += 10.0 * (noise[k] / thr.max(1e-30)).log10();
        n += 1;
    }
    sum / n.max(1) as f64
}

#[test]
fn lc960_matches_the_1024_path_nmr_at_equal_rate() {
    for (name, pcm) in [
        ("mixed", mixed(SECONDS)),
        ("transients", transients(SECONDS)),
        ("attacks", attacks(SECONDS)),
    ] {
        for &bitrate in RATES {
            let c960 = encode(&pcm, bitrate, FrameFamily::Lc960);
            let c1024 = encode(&pcm, bitrate, FrameFamily::Lc1024);
            assert_eq!(
                c960.asc,
                [0x11, 0x94],
                "AOT 2, 48 kHz, stereo, frameLengthFlag"
            );
            let n960 = mean_nmr(&pcm, &own_decode(&c960), 960);
            let n1024 = mean_nmr(&pcm, &own_decode(&c1024), 1024);
            eprintln!(
                "{name} {}k: 960 {:.1} kbps NMR {n960:.2} dB | 1024 {:.1} kbps NMR {n1024:.2} dB | Δ {:+.2} dB",
                bitrate / 1000,
                kbps(&c960),
                kbps(&c1024),
                n960 - n1024
            );
            assert!(kbps(&c960) <= f64::from(bitrate) / 1000.0 * 1.02);
            if name == "attacks" {
                // Block switching runs on the 960-line grid: the CPE's
                // shared ics_info window_sequence (the two bits after
                // id_syn_ele, tag, common_window, ics_reserved_bit).
                let short = |c: &Coded| c.aus.iter().filter(|au| (au[1] >> 5) & 3 == 2).count();
                eprintln!("EIGHT_SHORT: 960 {} / 1024 {}", short(&c960), short(&c1024));
                // One short run per attack after the stream start.
                let runs = (SECONDS * 4.0) as usize - 1;
                assert!(short(&c960) >= runs, "{} EIGHT_SHORT frames", short(&c960));
            }
            assert!(
                n960 <= n1024 + 0.75,
                "{name} {bitrate}: 960 NMR {n960:.2} vs 1024 {n1024:.2}"
            );
        }
    }
}

#[test]
fn adts_entry_points_reject_the_960_family() {
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: FS,
        channels: 1,
        bitrate: 64_000,
    })
    .unwrap();
    enc.set_frame_family(FrameFamily::Lc960).unwrap();
    assert_eq!(enc.frame_len(), 960);
    assert!(enc.encode_frame(&[0; 960]).is_err());
    assert!(enc.encode_access_unit(&[0; 960]).is_ok());
    assert!(enc.encode_access_unit(&[0; 961]).is_err());
    assert!(enc.set_frame_family(FrameFamily::Ld512).is_err());
}

#[test]
fn runtime_decoder_takes_the_960_asc_from_extradata() {
    let pcm = mixed(0.5);
    let c = encode(&pcm, 96_000, FrameFamily::Lc960);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.extradata = c.asc.clone();
    let mut dec = oxideav_aac::codec_decoder::make_decoder(&params).unwrap();
    let mut got = Vec::new();
    for au in &c.aus {
        dec.send_packet(&Packet::new(0, TimeBase::new(1, FS as i64), au.clone()))
            .unwrap();
        loop {
            match dec.receive_frame() {
                Ok(Frame::Audio(a)) => {
                    assert_eq!(a.samples, 960);
                    got.extend(
                        a.data[0]
                            .as_chunks::<2>()
                            .0
                            .iter()
                            .map(|b| i16::from_le_bytes(*b)),
                    );
                }
                Ok(f) => panic!("unexpected {f:?}"),
                Err(Error::NeedMore) => break,
                Err(e) => panic!("{e:?}"),
            }
        }
    }
    assert_eq!(got, own_decode(&c));
}

fn ffmpeg() -> Option<&'static str> {
    Command::new("ffmpeg")
        .arg("-version")
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|_| "ffmpeg")
}

fn read_wav_data(path: &PathBuf) -> Vec<i16> {
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

#[test]
fn black_box_decoder_reads_the_960_streams() {
    let Some(ff) = ffmpeg() else {
        eprintln!("skip: no ffmpeg");
        return;
    };
    let dir = std::env::temp_dir().join("oxideav-aac-encoder-960");
    fs::create_dir_all(&dir).unwrap();
    for (name, pcm) in [
        ("mixed", mixed(SECONDS / 2.0)),
        ("transients", transients(SECONDS / 2.0)),
        ("attacks", attacks(SECONDS / 2.0)),
    ] {
        let c = encode(&pcm, 96_000, FrameFamily::Lc960);
        let mut lw = LoasWriter::new(c.asc.clone(), 16, 1).unwrap();
        let mut loas = Vec::new();
        for au in &c.aus {
            loas.extend_from_slice(&lw.write_frame(au).unwrap());
        }
        let latm = dir.join(format!("{name}.latm"));
        let wav = dir.join(format!("{name}.wav"));
        fs::write(&latm, &loas).unwrap();
        let _ = fs::remove_file(&wav);
        let out = Command::new(ff)
            .args([
                "-hide_banner",
                "-loglevel",
                "error",
                "-y",
                "-f",
                "loas",
                "-i",
            ])
            .arg(&latm)
            .arg(&wav)
            .output()
            .unwrap();
        assert!(
            out.status.success(),
            "{}",
            String::from_utf8_lossy(&out.stderr)
        );
        assert!(
            String::from_utf8_lossy(&out.stderr).trim().is_empty(),
            "{name}: diagnostics {}",
            String::from_utf8_lossy(&out.stderr)
        );
        let theirs = read_wav_data(&wav);
        let ours = own_decode(&c);
        let n = theirs.len().min(ours.len());
        assert!(
            n >= ours.len() - 2 * 960,
            "{name}: {} of {} samples",
            theirs.len(),
            ours.len()
        );
        let max = ours[..n]
            .iter()
            .zip(&theirs[..n])
            .map(|(&a, &b)| (i32::from(a) - i32::from(b)).abs())
            .max()
            .unwrap();
        eprintln!("{name}: black-box decode max |diff| {max} over {n} samples");
        assert!(max <= 1, "{name}: max diff {max}");
    }
}

/// err/sig of channel 0 for one encode → decode under `family`
/// with PNS and intensity stereo on.
fn tool_round_trip(pcm: &[i16], channels: u8, family: FrameFamily) -> f64 {
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: FS,
        channels,
        bitrate: 48_000 * u32::from(channels),
    })
    .unwrap();
    enc.set_frame_family(family).unwrap();
    enc.set_pns(true);
    enc.set_intensity_stereo(true);
    let aus = enc.encode_all_access_units(pcm).unwrap();
    let n = family.frame_len();
    let mut dec = StreamDecoder::new();
    dec.set_frame_family(family);
    let mut out = Vec::new();
    for au in &aus {
        let f = dec
            .decode_raw_data_block(2, FS_INDEX, FS, channels, 1, au)
            .unwrap_or_else(|e| panic!("{family:?} {channels} ch: {e:?}"));
        assert_eq!(f.pcm.len(), n * usize::from(channels));
        out.extend_from_slice(&f.pcm);
    }
    let ch = usize::from(channels);
    let (mut e, mut s) = (0.0f64, 0.0f64);
    for t in 0..pcm.len() / ch {
        let a = f64::from(pcm[t * ch]);
        let b = f64::from(out[(t + n) * ch]);
        e += (a - b) * (a - b);
        s += a * a;
    }
    (e / s).sqrt()
}

#[test]
fn lc960_tool_paths_and_layouts_round_trip() {
    // PNS + intensity stereo on (long frames), and the 5.1 plan with
    // its LFE element — every tool's band walk on the 960-line tables,
    // measured against the same encode on the 1024-line family (PNS
    // noise is generator-phased, so the waveform error is the
    // comparison, not an absolute bound).
    let stereo = mixed(0.5);
    for channels in [1u8, 2, 6] {
        let pcm: Vec<i16> = stereo
            .as_chunks::<2>()
            .0
            .iter()
            .flat_map(|lr| {
                (0..channels).map(move |c| lr[usize::from(c) % 2] / (1 + i16::from(c / 2)))
            })
            .collect();
        let r1024 = tool_round_trip(&pcm, channels, FrameFamily::Lc1024);
        let r960 = tool_round_trip(&pcm, channels, FrameFamily::Lc960);
        eprintln!("{channels} ch: err/sig 960 {r960:.4} | 1024 {r1024:.4}");
        assert!(
            r960 < r1024 * 1.25 + 0.02,
            "{channels} ch: {r960} vs {r1024}"
        );
    }
}

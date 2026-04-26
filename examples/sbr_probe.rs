//! Instrumentation helper: decode an ADTS file through the crate and
//! print peak PCM levels frame-by-frame along with a peak from a
//! reference PCM file (ffmpeg-decoded). Used to diagnose SBR scaling.
//!
//! Usage:
//!   cargo run --example sbr_probe -- <adts.aac> <ref.s16le.pcm>

use std::env;
use std::path::PathBuf;

use oxideav_aac::adts::{parse_adts_header, ADTS_HEADER_NO_CRC};
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

fn iter_adts(bytes: &[u8]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + ADTS_HEADER_NO_CRC < bytes.len() {
        if bytes[i] != 0xFF || (bytes[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        match parse_adts_header(&bytes[i..]) {
            Ok(h) => {
                if h.frame_length == 0 || i + h.frame_length > bytes.len() {
                    break;
                }
                out.push((i, h.frame_length));
                i += h.frame_length;
            }
            Err(_) => i += 1,
        }
    }
    out
}

fn peak_i16(data: &[u8]) -> i16 {
    let mut peak: i16 = 0;
    for c in data.chunks_exact(2) {
        let s = i16::from_le_bytes([c[0], c[1]]);
        peak = peak.max(s.saturating_abs());
    }
    peak
}

fn minmax_i16(data: &[u8]) -> (i16, i16) {
    let mut lo = i16::MAX;
    let mut hi = i16::MIN;
    for c in data.chunks_exact(2) {
        let s = i16::from_le_bytes([c[0], c[1]]);
        if s < lo {
            lo = s;
        }
        if s > hi {
            hi = s;
        }
    }
    (lo, hi)
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: sbr_probe <adts.aac> [ref.s16le.pcm]");
        std::process::exit(2);
    }
    let adts = PathBuf::from(&args[1]);
    let ref_pcm_path = args.get(2).map(PathBuf::from);

    let bytes = std::fs::read(&adts).expect("read adts");
    let frames = iter_adts(&bytes);
    if frames.is_empty() {
        eprintln!("no ADTS frames in {}", adts.display());
        std::process::exit(1);
    }
    let first = parse_adts_header(&bytes[frames[0].0..]).unwrap();
    let core_sr = first.sample_rate().unwrap();
    let ch = first.channel_configuration.max(1) as u16;
    println!("ADTS: core_sr={} ch_cfg={}", core_sr, ch);

    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(core_sr);
    params.channels = Some(ch);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("dec");
    let tb = TimeBase::new(1, core_sr as i64);

    let mut all_pcm: Vec<u8> = Vec::new();
    let mut frame_peaks: Vec<i16> = Vec::new();
    let out_ch: u16 = ch;
    let out_sr: u32 = core_sr;
    for (i, &(off, len)) in frames.iter().enumerate() {
        let pkt = Packet::new(0, tb, bytes[off..off + len].to_vec()).with_pts(i as i64 * 1024);
        dec.send_packet(&pkt).unwrap();
        match dec.receive_frame() {
            Ok(Frame::Audio(af)) => {
                let peak = peak_i16(&af.data[0]);
                frame_peaks.push(peak);
                all_pcm.extend_from_slice(&af.data[0]);
            }
            Ok(_) => {}
            Err(_) => {}
        }
    }
    println!(
        "our decoder: {} frames, out_sr={}, out_ch={}",
        frame_peaks.len(),
        out_sr,
        out_ch
    );
    let our_peak = frame_peaks.iter().copied().max().unwrap_or(0);
    let (lo, hi) = minmax_i16(&all_pcm);
    println!(
        "our PCM peak: {} (min={} max={}, over {} frames)",
        our_peak,
        lo,
        hi,
        frame_peaks.len()
    );
    // Skip first 2 frames (SBR transient) for a more representative peak.
    if frame_peaks.len() > 3 {
        let steady_peak = frame_peaks[2..].iter().copied().max().unwrap_or(0);
        println!("our PCM peak (skip 2): {}", steady_peak);
    }

    // Optionally dump our PCM for external analysis — set
    // OXIDEAV_SBR_PROBE_DUMP=<path> to enable.
    if let Some(dump_path) = std::env::var_os("OXIDEAV_SBR_PROBE_DUMP") {
        std::fs::write(&dump_path, &all_pcm).expect("dump pcm");
        println!("dumped our PCM to {:?}", dump_path);
    }

    if let Some(refp) = ref_pcm_path {
        let ref_bytes = std::fs::read(&refp).expect("read ref");
        let ref_peak = peak_i16(&ref_bytes);
        println!("ref PCM peak: {} (len={})", ref_peak, ref_bytes.len());

        let min = ref_bytes.len().min(all_pcm.len());
        // Compute mean-square error and PSNR vs reference for identical length.
        let mut mse: f64 = 0.0;
        let mut count: usize = 0;
        for c in 0..(min / 2) {
            let a = i16::from_le_bytes([all_pcm[2 * c], all_pcm[2 * c + 1]]) as f64;
            let b = i16::from_le_bytes([ref_bytes[2 * c], ref_bytes[2 * c + 1]]) as f64;
            let d = a - b;
            mse += d * d;
            count += 1;
        }
        if count > 0 {
            mse /= count as f64;
            let peak_f = 32767.0f64;
            let psnr = if mse > 0.0 {
                10.0 * (peak_f * peak_f / mse).log10()
            } else {
                f64::INFINITY
            };
            println!(
                "vs ref: min_len={} count={} mse={:.2} PSNR={:.2} dB",
                min, count, mse, psnr
            );
        }
    }

    // Print first few peaks so we can see transient vs steady
    let n = frame_peaks.len().min(12);
    println!("first {} frame peaks: {:?}", n, &frame_peaks[..n]);
}

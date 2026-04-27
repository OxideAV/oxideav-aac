//! Round-19 amplitude diagnostic: compare the **raw spectrum magnitude** that
//! a third-party AAC encoder emits vs. what our encoder emits, then run the
//! same IMDCT/window/OLA chain on both and report the resulting peak PCM.
//!
//! This is a 1:1 spectrum-scale probe — it bypasses the public AacDecoder so
//! we can inspect intermediate values (peak spectrum coefficient, peak
//! IMDCT output before windowing, peak after windowed OLA).
//!
//! Reads two ADTS files:
//!   `/tmp/oxideav_probe/ffmpeg_enc.aac` — produced by ffmpeg from a 0.3-amp
//!                                         440 Hz sine at 44.1 kHz.
//!   `/tmp/oxideav_probe/ours.aac`        — produced by our encoder on the
//!                                         same input.
//!
//! Invoke with `cargo run --example spectrum_compare`. The probe regenerates
//! both files if they're missing.
use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_aac::adts::parse_adts_header;
use oxideav_aac::encoder::AacEncoder;
use oxideav_aac::ics::{
    decode_spectrum_long, parse_ics_info, parse_scalefactors, parse_section_data,
};
use oxideav_aac::imdct::imdct_long;
use oxideav_aac::syntax::{ElementType, WindowSequence};
use oxideav_core::bits::BitReader;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame};

const SR: u32 = 44_100;
const SECS: f32 = 1.0;
const AMP: f32 = 0.3;

fn write_pcm(path: &Path) -> std::io::Result<()> {
    let total = (SR as f32 * SECS) as usize;
    let mut bytes = Vec::with_capacity(total * 2);
    for i in 0..total {
        let t = i as f32 / SR as f32;
        let v = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * AMP;
        bytes.extend_from_slice(&((v * 32767.0) as i16).to_le_bytes());
    }
    std::fs::write(path, bytes)
}

fn ours_encode(path: &Path) {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(SR);
    params.channels = Some(1);
    params.bit_rate = Some(128_000);
    let mut enc = AacEncoder::new(&params).expect("ctor");
    let total = (SR as f32 * SECS) as usize;
    let mut bytes = Vec::with_capacity(total * 2);
    for i in 0..total {
        let t = i as f32 / SR as f32;
        let v = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * AMP;
        bytes.extend_from_slice(&((v * 32767.0) as i16).to_le_bytes());
    }
    let af = AudioFrame {
        samples: total as u32,
        pts: Some(0),
        data: vec![bytes],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send_frame");
    enc.flush().expect("flush");
    let mut adts = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        adts.extend_from_slice(&pkt.data);
    }
    std::fs::write(path, &adts).expect("write ours.aac");
}

fn ffmpeg_encode(pcm: &Path, aac: &Path) {
    let _ = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .args(["-f", "s16le", "-ar", "44100", "-ac", "1"])
        .arg("-i")
        .arg(pcm)
        .args(["-c:a", "aac", "-b:a", "128k"])
        .arg(aac)
        .status()
        .expect("ffmpeg encode");
}

fn iter_adts(bytes: &[u8]) -> Vec<(usize, usize)> {
    let mut frames = Vec::new();
    let mut off = 0usize;
    while off + 7 < bytes.len() {
        if bytes[off] == 0xff && (bytes[off + 1] & 0xf0) == 0xf0 {
            if let Ok(h) = parse_adts_header(&bytes[off..]) {
                let len = h.header_length() + h.payload_length();
                if len > 0 && off + len <= bytes.len() {
                    frames.push((off, len));
                    off += len;
                    continue;
                }
            }
        }
        off += 1;
    }
    frames
}

/// Parse a single SCE element's spectrum. Skips other element types. The
/// raw-data-block follows the ADTS header at offset `header_length()`.
fn first_sce_spectrum(packet: &[u8], sf_index: u8) -> Option<[f32; 1024]> {
    let h = parse_adts_header(packet).ok()?;
    let payload = &packet[h.header_length()..h.header_length() + h.payload_length()];
    let mut br = BitReader::new(payload);
    // Probe is intentionally one-shot — first SCE returns, anything else
    // exits the function. The `loop` shape used to mirror `decoder.rs`,
    // but clippy::never_loop flags it because every match arm returns.
    {
        let id = br.read_u32(3).ok()?;
        let kind = ElementType::from_id(id);
        match kind {
            ElementType::Sce => {
                let _instance_tag = br.read_u32(4).ok()?;
                let global_gain = br.read_u32(8).ok()? as u8;
                let info = parse_ics_info(&mut br, sf_index).ok()?;
                if info.window_sequence == WindowSequence::EightShort {
                    return None; // skip — only long-window probe.
                }
                let sec = parse_section_data(&mut br, &info).ok()?;
                let sf = parse_scalefactors(&mut br, &info, &sec, global_gain).ok()?;
                let sf_min = sf.iter().min().copied().unwrap_or(0);
                let sf_max = sf.iter().max().copied().unwrap_or(0);
                let pns_count = sec.sfb_cb.iter().filter(|&&c| c == 13).count();
                let is_count = sec.sfb_cb.iter().filter(|&&c| c == 14 || c == 15).count();
                eprintln!(
                    "    [debug] global_gain={global_gain} max_sfb={} sf_min={sf_min} sf_max={sf_max} pns_bands={pns_count} is_bands={is_count}",
                    info.max_sfb
                );
                // pulse_data_present, tns_data_present, gain_control_data_present
                let pulse_present = br.read_bit().ok()?;
                if pulse_present {
                    // pulse_data: number_pulse(2) + pulse_start_sfb(6) + 5 pulses * (offset(5)+amp(4))
                    let np = br.read_u32(2).ok()? as usize;
                    let _start = br.read_u32(6).ok()?;
                    for _ in 0..=np {
                        let _o = br.read_u32(5).ok()?;
                        let _a = br.read_u32(4).ok()?;
                    }
                }
                let tns_present = br.read_bit().ok()?;
                if tns_present {
                    // We don't apply TNS here, but we need to consume the bits.
                    // Use a simple reader that reads tns_data per ISO Table 4.48.
                    // For long: n_filt(2), then per filter: coef_res(1), length(6), order(5), if order: direction(1) + compress(1) + coef bits…
                    let n_filt = br.read_u32(2).ok()?;
                    for _ in 0..n_filt {
                        let coef_res = br.read_u32(1).ok()?;
                        let _length = br.read_u32(6).ok()?;
                        let order = br.read_u32(5).ok()?;
                        if order > 0 {
                            let _direction = br.read_u32(1).ok()?;
                            let coef_compress = br.read_u32(1).ok()?;
                            let nbits = if coef_res == 1 { 4 } else { 3 } - coef_compress;
                            for _ in 0..order {
                                let _c = br.read_u32(nbits).ok()?;
                            }
                        }
                    }
                }
                let gain_present = br.read_bit().ok()?;
                if gain_present {
                    return None;
                }
                let mut spec = [0.0f32; 1024];
                decode_spectrum_long(&mut br, &info, &sec, &sf, &mut spec).ok()?;
                Some(spec)
            }
            _ => None,
        }
    }
}

fn peak(arr: &[f32]) -> f32 {
    arr.iter().fold(0.0f32, |a, &b| a.max(b.abs()))
}

fn analyse(path: &Path, label: &str) {
    let bytes = std::fs::read(path).expect("read");
    let frames = iter_adts(&bytes);
    if frames.is_empty() {
        println!("{label}: no frames");
        return;
    }
    let h = parse_adts_header(&bytes[frames[0].0..]).unwrap();
    let sf_index = h.sampling_freq_index;
    // Use frame index 5 (mid-stream) to avoid encoder warm-up artefacts.
    let probe_idx = 5usize.min(frames.len() - 1);
    let (off, len) = frames[probe_idx];
    let spec = match first_sce_spectrum(&bytes[off..off + len], sf_index) {
        Some(s) => s,
        None => {
            println!("{label}: no SCE spectrum extracted");
            return;
        }
    };
    // Also pull frame at probe_idx + 1 to test OLA.
    let next_idx = probe_idx + 1;
    let spec_next = if next_idx < frames.len() {
        let (no, nl) = frames[next_idx];
        first_sce_spectrum(&bytes[no..no + nl], sf_index)
    } else {
        None
    };
    let pk_spec = peak(&spec);
    let pk_idx = spec
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
        .unwrap()
        .0;
    let energy: f64 = spec.iter().map(|&v| (v as f64).powi(2)).sum::<f64>();
    let nonzero = spec.iter().filter(|&&v| v.abs() > 1.0).count();
    let mut imdct_out = vec![0.0f32; 2048];
    imdct_long(&spec, &mut imdct_out);
    let pk_imdct = peak(&imdct_out);
    // Apply sine window then look at the first-half / second-half peaks.
    let sine = oxideav_aac::window::sine_long();
    let n = 1024;
    let mut win_out = imdct_out.clone();
    for i in 0..n {
        win_out[i] *= sine[i];
        win_out[n + i] *= sine[n - 1 - i];
    }
    let pk_first_half = win_out[..n].iter().fold(0.0f32, |a, &b| a.max(b.abs()));
    let pk_second_half = win_out[n..].iter().fold(0.0f32, |a, &b| a.max(b.abs()));
    println!(
        "{label}: spec peak={pk_spec:.2} at bin {pk_idx}, energy={energy:.2e}, nonzero_bins={nonzero}, IMDCT peak={pk_imdct:.2}, win-half peaks={pk_first_half:.2} / {pk_second_half:.2}",
    );
    // OLA two consecutive frames: prev's right half (= win_out[N..]) +
    // next's left half (after IMDCT + windowing).
    if let Some(spec_next) = spec_next {
        let mut imdct_n = vec![0.0f32; 2048];
        imdct_long(&spec_next, &mut imdct_n);
        let mut win_n = imdct_n.clone();
        for i in 0..n {
            win_n[i] *= sine[i];
            win_n[n + i] *= sine[n - 1 - i];
        }
        let mut ola = vec![0.0f32; n];
        for i in 0..n {
            ola[i] = win_out[n + i] + win_n[i];
        }
        let pk_ola = ola.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
        let pk_ola_half = (pk_ola * 0.5).round();
        println!("    OLA frames {probe_idx}+{next_idx}: peak={pk_ola:.2}, *0.5 = {pk_ola_half}",);
    }
}

fn main() {
    let scratch: PathBuf = std::env::temp_dir().join("oxideav_probe");
    let _ = std::fs::create_dir_all(&scratch);
    let pcm = scratch.join("sine.s16");
    let ffmpeg_aac = scratch.join("ffmpeg_enc.aac");
    let ours_aac = scratch.join("ours.aac");
    write_pcm(&pcm).expect("pcm");
    if !ffmpeg_aac.exists() {
        ffmpeg_encode(&pcm, &ffmpeg_aac);
    }
    ours_encode(&ours_aac);

    println!("--- frame index 5 in each stream ---");
    analyse(&ffmpeg_aac, "ffmpeg-encoded");
    analyse(&ours_aac, "ours-encoded   ");

    // Theoretical expected: input PCM peak = 0.3 (after /32768). For an
    // unscaled forward MDCT of a 440 Hz sine windowed by a 1024-sine over
    // a 2048-sample block, the MDCT peak hits the bin closest to
    //   440 / (44100/2048) ≈ 20.4
    // and has magnitude about (input_peak * window_energy / 2). The same
    // chain through our `mdct_long` (unscaled) on input range [-0.3, 0.3]
    // produces peak ≈ 166.5 — see `examples/probe_lc_amp.rs`.
    println!();
    println!("Theoretical reference for our encoder: a [-0.3, 0.3] PCM sine through");
    println!("  unscaled mdct_long gives spec peak ≈ 166.5 (see probe_lc_amp.rs's debug).");
    println!("  Our encoder multiplies that by MDCT_FORWARD_SCALE = 65_536 → spec ≈ 1.09e7.");
    println!("  Our IMDCT(2/N) of a 1.09e7-peak coefficient yields ≈ pcm peak 4.5e4 / window.");
}

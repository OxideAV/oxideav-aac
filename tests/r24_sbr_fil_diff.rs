//! Round-24 SBR FIL extension diff harness.
//!
//! Captures the SBR FIL bytes our HE-AAC mono encoder writes for the same
//! 1 kHz / amp-0.3 / 0.5 s tone the r18 amplitude regression uses, captures
//! fdkaac's bytes for the identical input, and parses both into
//! [`SbrChannelData`] structs that we diff field-by-field.
//!
//! r23 verdict: pure AAC-LC at 24 kHz core / 1 kHz tone decodes
//! byte-tight via ffmpeg, so the saturation lives **inside** the SBR FIL
//! payload itself. r24 narrows the search by direct comparison with a
//! known-good encoder.
//!
//! All field decoding goes through the existing
//! `oxideav_aac::sbr::bitstream::*` parser (the same one our decoder uses),
//! so what we observe here is exactly what ffmpeg is parsing on its side.
//!
//! Skips when fdkaac is not on PATH.

use std::path::PathBuf;
use std::process::Command;

use oxideav_aac::adts::parse_adts_header;
use oxideav_aac::decoder::{decode_ics, fill_spectrum};
use oxideav_aac::he_aac_encoder::HeAacMonoEncoder;
use oxideav_aac::ics::SPEC_LEN;
use oxideav_aac::sbr::bitstream::{
    parse_sbr_header, parse_single_channel_element, SbrChannelData, SbrHeader, EXT_SBR_DATA,
    EXT_SBR_DATA_CRC,
};
use oxideav_aac::sbr::freq::FreqTables;
use oxideav_aac::syntax::{sample_rate, ElementType};
use oxideav_core::bits::BitReader;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame};

const HIGH_RATE: u32 = 48_000;
const SECS: f32 = 0.5;
const AMP: f32 = 0.3;
const TONE_HZ: f32 = 1_000.0;

fn which(name: &str) -> Option<PathBuf> {
    let out = Command::new("which").arg(name).output().ok()?;
    if !out.status.success() {
        return None;
    }
    let p = String::from_utf8(out.stdout).ok()?;
    let t = p.trim();
    if t.is_empty() {
        None
    } else {
        Some(PathBuf::from(t))
    }
}

fn make_mono_pcm_s16() -> Vec<u8> {
    let total = (HIGH_RATE as f32 * SECS) as usize;
    let mut bytes = Vec::with_capacity(total * 2);
    for i in 0..total {
        let t = i as f32 / HIGH_RATE as f32;
        let v = (2.0 * std::f32::consts::PI * TONE_HZ * t).sin() * AMP;
        bytes.extend_from_slice(&((v * 32767.0) as i16).to_le_bytes());
    }
    bytes
}

/// Encode the test fixture through our HE-AAC mono encoder. Returns the
/// concatenated ADTS bytes.
fn encode_ours() -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(HIGH_RATE);
    params.channels = Some(1);
    params.bit_rate = Some(48_000);
    let mut enc = HeAacMonoEncoder::new(&params).expect("ctor");
    let pcm = make_mono_pcm_s16();
    let total = (HIGH_RATE as f32 * SECS) as usize;
    let af = AudioFrame {
        samples: total as u32,
        pts: Some(0),
        data: vec![pcm],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send");
    enc.flush().expect("flush");
    let mut out = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        out.extend_from_slice(&pkt.data);
    }
    out
}

/// Encode the same fixture through fdkaac (HE-AAC, mono, 48 kbps so SBR is
/// definitely on). Returns the concatenated ADTS bytes.
///
/// fdkaac mode `-p 5` selects HE-AAC (LC core + SBR).
fn encode_fdkaac() -> Option<Vec<u8>> {
    which("fdkaac")?;
    let scratch = std::env::temp_dir().join("oxideav_aac_r24_fdk");
    std::fs::create_dir_all(&scratch).ok()?;
    let wav_path = scratch.join("in.wav");
    let aac_path = scratch.join("fdk.aac");
    let pcm = make_mono_pcm_s16();
    let data_len = pcm.len() as u32;
    let mut wav = Vec::with_capacity(44 + pcm.len());
    wav.extend_from_slice(b"RIFF");
    wav.extend_from_slice(&(36u32 + data_len).to_le_bytes());
    wav.extend_from_slice(b"WAVEfmt ");
    wav.extend_from_slice(&16u32.to_le_bytes());
    wav.extend_from_slice(&1u16.to_le_bytes());
    wav.extend_from_slice(&1u16.to_le_bytes());
    wav.extend_from_slice(&HIGH_RATE.to_le_bytes());
    wav.extend_from_slice(&(HIGH_RATE * 2).to_le_bytes());
    wav.extend_from_slice(&2u16.to_le_bytes());
    wav.extend_from_slice(&16u16.to_le_bytes());
    wav.extend_from_slice(b"data");
    wav.extend_from_slice(&data_len.to_le_bytes());
    wav.extend_from_slice(&pcm);
    std::fs::write(&wav_path, &wav).ok()?;
    let status = Command::new("fdkaac")
        .args(["-p", "5"])
        .args(["-b", "48000"])
        .args(["-f", "2"]) // ADTS transport so we can scan frame-by-frame
        .arg("-o")
        .arg(&aac_path)
        .arg(&wav_path)
        .output()
        .ok()?;
    if !status.status.success() {
        eprintln!("fdkaac failed: {}", String::from_utf8_lossy(&status.stderr));
        return None;
    }
    std::fs::read(&aac_path).ok()
}

struct FrameDiag {
    frame_idx: usize,
    header: SbrHeader,
    data: SbrChannelData,
    n_high: usize,
    n_low: usize,
    nq: usize,
    /// Total bit-length of the FIL payload (incl. the 4-bit ext_id).
    fil_payload_bits: u32,
    /// Bits actually consumed by the SBR parser (excluding fill alignment).
    sbr_bits_consumed: u32,
}

/// Parse one ADTS-framed AAC stream, walking each raw_data_block element
/// using the production decoder's ICS routine to skip SCE bodies cleanly.
/// On each FIL element with EXT_SBR_DATA we run the SBR SCE parser and
/// record one [`FrameDiag`] per frame.
fn parse_stream(adts_bytes: &[u8]) -> Vec<FrameDiag> {
    let mut out = Vec::new();
    let mut cursor = 0usize;
    let mut header = SbrHeader::defaults();
    let mut header_seen = false;
    let mut freq: Option<FreqTables> = None;
    let mut frame_idx = 0usize;
    while cursor + 7 <= adts_bytes.len() {
        let hdr = match parse_adts_header(&adts_bytes[cursor..]) {
            Ok(h) => h,
            Err(_) => break,
        };
        let frame_end = cursor + hdr.frame_length;
        if frame_end > adts_bytes.len() {
            break;
        }
        let core_rate = sample_rate(hdr.sampling_freq_index).unwrap_or(24_000);
        let sf_index = hdr.sampling_freq_index;
        let payload = &adts_bytes[cursor + hdr.header_length()..frame_end];
        let mut br = BitReader::new(payload);

        while let Ok(id) = br.read_u32(3) {
            let kind = ElementType::from_id(id);
            match kind {
                ElementType::End => break,
                ElementType::Sce => {
                    let _instance_tag = match br.read_u32(4) {
                        Ok(v) => v,
                        Err(_) => break,
                    };
                    // Decode (and discard) the ICS body so the bit-cursor
                    // lands exactly on the next syntactic element.
                    let (info, sf, sec, _tns, _pulse) = match decode_ics(&mut br, sf_index, false) {
                        Ok(v) => v,
                        Err(_) => break,
                    };
                    let mut spec = [0.0f32; SPEC_LEN];
                    if fill_spectrum(&mut br, &info, &sec, &sf, &mut spec).is_err() {
                        break;
                    }
                }
                ElementType::Fil => {
                    let mut count = match br.read_u32(4) {
                        Ok(v) => v,
                        Err(_) => break,
                    };
                    if count == 15 {
                        let esc = match br.read_u32(8) {
                            Ok(v) => v,
                            Err(_) => break,
                        };
                        count += esc.saturating_sub(1);
                    }
                    if count == 0 {
                        continue;
                    }
                    let total_bits = 8 * count;
                    let start_pos = br.bit_position();
                    let ext_type = match br.read_u32(4) {
                        Ok(v) => v,
                        Err(_) => break,
                    };
                    if ext_type != EXT_SBR_DATA && ext_type != EXT_SBR_DATA_CRC {
                        let consumed = (br.bit_position() - start_pos) as u32;
                        for _ in 0..total_bits.saturating_sub(consumed) {
                            let _ = br.read_u32(1);
                        }
                        continue;
                    }
                    if ext_type == EXT_SBR_DATA_CRC {
                        let _crc = br.read_u32(10).ok();
                    }
                    let bs_header_flag = br.read_bit().unwrap_or(false);
                    if bs_header_flag {
                        if parse_sbr_header(&mut br, &mut header).is_err() {
                            break;
                        }
                        header_seen = true;
                        let fs_sbr = core_rate * 2;
                        match FreqTables::build(
                            fs_sbr,
                            header.bs_start_freq,
                            header.bs_stop_freq,
                            header.bs_xover_band,
                            header.bs_freq_scale,
                            header.bs_alter_scale,
                            header.bs_noise_bands,
                        ) {
                            Ok(ft) => freq = Some(ft),
                            Err(e) => {
                                eprintln!(
                                    "frame {frame_idx}: build freq tables failed: {e}; \
                                     header = {:?}",
                                    header
                                );
                                break;
                            }
                        }
                    }
                    if !header_seen || freq.is_none() {
                        let consumed = (br.bit_position() - start_pos) as u32;
                        for _ in 0..total_bits.saturating_sub(consumed) {
                            let _ = br.read_u32(1);
                        }
                        continue;
                    }
                    let ft = freq.as_ref().unwrap();
                    let mut data = SbrChannelData {
                        bs_amp_res: header.bs_amp_res,
                        ..SbrChannelData::default()
                    };
                    let before_sce = br.bit_position();
                    if parse_single_channel_element(
                        &mut br,
                        &mut data,
                        ft.nq,
                        [ft.n_low, ft.n_high],
                        ft.n_high,
                    )
                    .is_err()
                    {
                        break;
                    }
                    let sbr_consumed = (br.bit_position() - before_sce) as u32;
                    let consumed = (br.bit_position() - start_pos) as u32;
                    out.push(FrameDiag {
                        frame_idx,
                        header: header.clone(),
                        data,
                        n_high: ft.n_high,
                        n_low: ft.n_low,
                        nq: ft.nq,
                        fil_payload_bits: total_bits,
                        sbr_bits_consumed: sbr_consumed,
                    });
                    for _ in 0..total_bits.saturating_sub(consumed) {
                        let _ = br.read_u32(1);
                    }
                }
                _ => break,
            }
        }

        cursor = frame_end;
        frame_idx += 1;
    }
    out
}

fn dump_field_summary(label: &str, frames: &[FrameDiag]) {
    println!("--- {label}: {} SBR FIL frames captured ---", frames.len());
    if let Some(f0) = frames.first() {
        println!(
            "frame[{}] hdr: amp_res={} start_freq={} stop_freq={} xover={} freq_scale={} alter={} noise_bands={} limiter_bands={} limiter_gains={} interpol={} smoothing={} h1={} h2={}",
            f0.frame_idx,
            f0.header.bs_amp_res,
            f0.header.bs_start_freq,
            f0.header.bs_stop_freq,
            f0.header.bs_xover_band,
            f0.header.bs_freq_scale,
            f0.header.bs_alter_scale as u8,
            f0.header.bs_noise_bands,
            f0.header.bs_limiter_bands,
            f0.header.bs_limiter_gains,
            f0.header.bs_interpol_freq as u8,
            f0.header.bs_smoothing_mode as u8,
            f0.header.bs_header_extra_1 as u8,
            f0.header.bs_header_extra_2 as u8,
        );
        println!(
            "  freq tables: n_high={} n_low={} nq={}, FIL payload bits={}, SBR consumed bits={}",
            f0.n_high, f0.n_low, f0.nq, f0.fil_payload_bits, f0.sbr_bits_consumed,
        );
    }
    let mut sum_invf = [0i64; 5];
    let mut sum_addh = [0i64; 64];
    let mut max_addh_bits = 0usize;
    let mut sum_env_first = [0i64; 5];
    let mut env_count = 0i64;
    let mut sum_noise_first = [0i64; 5];
    let mut noise_count = 0i64;
    let mut sum_addh_flag = 0i64;
    let mut sum_dfenv = [0i64; 5];
    let mut sum_dfnoise = [0i64; 2];
    for f in frames {
        for (i, v) in f.data.bs_invf_mode.iter().enumerate() {
            sum_invf[i] += *v as i64;
        }
        for (i, v) in f.data.bs_add_harmonic[..f.n_high.min(64)]
            .iter()
            .enumerate()
        {
            sum_addh[i] += *v as i64;
            max_addh_bits = max_addh_bits.max(i + 1);
        }
        sum_addh_flag += f.data.bs_add_harmonic_flag as i64;
        for env in 0..(f.data.bs_num_env as usize).min(5) {
            sum_env_first[env] += f.data.env_sf[env][0] as i64;
            env_count += 1;
            sum_dfenv[env] += f.data.bs_df_env[env] as i64;
        }
        for n in 0..(f.data.bs_num_noise as usize).min(2) {
            sum_noise_first[n] += f.data.noise_sf[n][0] as i64;
            noise_count += 1;
            sum_dfnoise[n] += f.data.bs_df_noise[n] as i64;
        }
    }
    println!(
        "  bs_invf_mode totals (per noise-band index, summed over frames): {:?}",
        sum_invf
    );
    println!(
        "  bs_add_harmonic_flag set on {} / {} frames",
        sum_addh_flag,
        frames.len()
    );
    if max_addh_bits > 0 {
        println!(
            "  bs_add_harmonic per-band totals (first {} bands): {:?}",
            max_addh_bits,
            &sum_addh[..max_addh_bits]
        );
    }
    println!(
        "  bs_df_env totals = {:?}, bs_df_noise totals = {:?}",
        sum_dfenv, sum_dfnoise
    );
    if env_count > 0 {
        let mut means = [0.0f64; 5];
        for (i, m) in means.iter_mut().enumerate() {
            *m = sum_env_first[i] as f64 / env_count.max(1) as f64;
        }
        println!("  env_sf[*][0] mean across all envelope slots: {:?}", means);
    }
    if noise_count > 0 {
        let mut means = [0.0f64; 2];
        for (i, m) in means.iter_mut().enumerate() {
            *m = sum_noise_first[i] as f64 / noise_count.max(1) as f64;
        }
        println!("  noise_sf[*][0] mean across all noise floors: {:?}", means);
    }
    if let Some(f0) = frames.first() {
        let n = f0.n_high.min(32);
        println!(
            "  frame[0] env_sf[0][0..{n}] = {:?}",
            &f0.data.env_sf[0][..n]
        );
        let nq = f0.nq.min(5);
        println!(
            "  frame[0] noise_sf[0][0..{nq}] = {:?}",
            &f0.data.noise_sf[0][..nq]
        );
        println!(
            "  frame[0] grid: class={:?} num_env={} num_noise={} amp_res={} freq_res={:?}",
            f0.data.frame_class,
            f0.data.bs_num_env,
            f0.data.bs_num_noise,
            f0.data.bs_amp_res,
            &f0.data.freq_res[..f0.data.bs_num_env as usize],
        );
    }
}

#[test]
fn diff_sbr_fil_against_fdkaac() {
    if which("fdkaac").is_none() {
        eprintln!("fdkaac not on PATH — skipping");
        return;
    }

    let ours_bytes = encode_ours();
    println!("ours: {} ADTS bytes", ours_bytes.len());

    let fdk_bytes = match encode_fdkaac() {
        Some(b) => b,
        None => {
            eprintln!("fdkaac encode failed — skipping");
            return;
        }
    };
    println!("fdkaac: {} ADTS bytes", fdk_bytes.len());

    let ours_diag = parse_stream(&ours_bytes);
    let fdk_diag = parse_stream(&fdk_bytes);

    dump_field_summary("ours", &ours_diag);
    dump_field_summary("fdkaac", &fdk_diag);

    assert!(
        !ours_diag.is_empty(),
        "ours: failed to recover any SBR FIL payloads"
    );
}

/// Pinning regression: ours' FIRST-frame SBR FIL payload is structurally
/// well-formed and the parse is stable. The exact field values are
/// recorded so that any subsequent change to the encoder needs to declare
/// the diff intentionally.
#[test]
fn ours_first_frame_sbr_pinned() {
    let bytes = encode_ours();
    let diag = parse_stream(&bytes);
    assert!(!diag.is_empty(), "no SBR FIL parsed");
    let f0 = &diag[0];
    // Header — pinned snapshot of our current emitter (r23-era).
    assert_eq!(f0.header.bs_amp_res, 0, "bs_amp_res in header");
    assert_eq!(f0.header.bs_start_freq, 5, "bs_start_freq");
    assert_eq!(f0.header.bs_stop_freq, 9, "bs_stop_freq");
    assert_eq!(f0.header.bs_xover_band, 0, "bs_xover_band");
    assert_eq!(f0.header.bs_freq_scale, 2, "bs_freq_scale");
    assert!(f0.header.bs_alter_scale, "bs_alter_scale");
    assert_eq!(f0.header.bs_noise_bands, 2, "bs_noise_bands");
    // Grid — single-envelope FIXFIX with amp_res forced to 0 by the
    // parse_sbr_grid rule (§4.6.18.3.3).
    assert_eq!(f0.data.bs_num_env, 1);
    assert_eq!(f0.data.bs_num_noise, 1);
    assert_eq!(f0.data.bs_amp_res, 0);
    // We never set bs_add_harmonic_flag.
    assert!(!f0.data.bs_add_harmonic_flag);
    // We never explicitly send any bs_invf_mode > 0 (production encoder
    // always emits the per-noise-band pattern `bw mode 0 = OFF`).
    for (i, &v) in f0.data.bs_invf_mode[..f0.nq].iter().enumerate() {
        assert_eq!(v, 0, "bs_invf_mode[{i}] expected 0, got {v}");
    }
}

/// New-pin: across the full encode, EVERY frame must keep `bs_invf_mode = 0`
/// for every noise-band slot. r24 lead #2 was that the parser sees a
/// nonzero default — this regression nails down that ours always emits 0
/// so any future drift is loud.
#[test]
fn ours_invf_mode_always_off() {
    let bytes = encode_ours();
    let diag = parse_stream(&bytes);
    assert!(!diag.is_empty());
    for d in &diag {
        for (i, &v) in d.data.bs_invf_mode[..d.nq].iter().enumerate() {
            assert_eq!(
                v, 0,
                "frame {} bs_invf_mode[{i}] = {v} (expected 0)",
                d.frame_idx
            );
        }
        assert!(
            !d.data.bs_add_harmonic_flag,
            "frame {} unexpectedly set bs_add_harmonic_flag",
            d.frame_idx
        );
    }
}

/// Pin: the `OXIDEAV_AAC_SBR_ENV_FORCE_ZERO` env-var probe (introduced in
/// r24 to test the "envelope value drives saturation" hypothesis) actually
/// zeros the envelope on every frame. Without this pin, regressing the
/// override silently makes all subsequent r25+ probes meaningless.
#[test]
fn force_zero_env_var_actually_zeros_envelope() {
    // Save then set the env var for the duration of this test.
    // Tests in the same process share env state, so be defensive about
    // restoring the original value at end of scope.
    struct EnvGuard {
        prev: Option<String>,
    }
    impl Drop for EnvGuard {
        fn drop(&mut self) {
            // SAFETY: setting a process env var is unsafe in multi-threaded
            // contexts, but this is a single-threaded test scope and we
            // restore on drop. The std crate marks set_var unsafe as of
            // edition 2024, so we mark this entire block unsafe.
            unsafe {
                if let Some(p) = self.prev.take() {
                    std::env::set_var("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO", p);
                } else {
                    std::env::remove_var("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO");
                }
            }
        }
    }
    let prev = std::env::var("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO").ok();
    // SAFETY: see EnvGuard::drop comment; single-threaded test.
    unsafe {
        std::env::set_var("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO", "1");
    }
    let _g = EnvGuard { prev };

    let bytes = encode_ours();
    let diag = parse_stream(&bytes);
    assert!(!diag.is_empty(), "no SBR FIL parsed");
    for d in &diag {
        for env in 0..(d.data.bs_num_env as usize).min(5) {
            for band in 0..d.n_high {
                assert_eq!(
                    d.data.env_sf[env][band], 0,
                    "frame {} env_sf[{env}][{band}] = {} (expected 0 under FORCE_ZERO)",
                    d.frame_idx, d.data.env_sf[env][band]
                );
            }
        }
    }
}

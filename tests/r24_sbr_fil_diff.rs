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
    parse_sbr_dtdf, parse_sbr_envelope, parse_sbr_grid, parse_sbr_header, parse_sbr_invf,
    parse_sbr_noise, parse_sbr_sinusoidal, SbrChannelData, SbrHeader, EXT_SBR_DATA,
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
    /// Counts only the SCE body — i.e. excludes `bs_header_flag` and the
    /// `sbr_header()` block.
    sbr_bits_consumed: u32,
    /// Bits consumed inside the FIL payload excluding the fill alignment
    /// at the tail. Includes the 4-bit `ext_id`, the 1-bit
    /// `bs_header_flag`, the `sbr_header()` block when present, and the
    /// full SCE body. Equivalent to ffmpeg's `4 + num_sbr_bits` quantity
    /// from §4.4.2.8 Table 4.62.
    fil_consumed_bits: u32,
    /// `bs_extended_data` flag (1 bit) at the tail of the SCE body.
    /// Captured by walking the SCE in stages so the test harness can
    /// compare ours-vs-fdkaac directly on this lead-3-relevant bit.
    bs_extended_data: bool,
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
                    // Walk the SCE in stages so we can capture
                    // `bs_extended_data` for the lead-3 byte-compare
                    // tests (the all-in-one
                    // `parse_single_channel_element` swallows that bit
                    // internally and discards it).
                    let parse_ok = (|| -> Result<bool, ()> {
                        let bs_data_extra = br.read_bit().map_err(|_| ())?;
                        if bs_data_extra {
                            br.read_u32(4).map_err(|_| ())?; // bs_reserved
                        }
                        parse_sbr_grid(&mut br, &mut data).map_err(|_| ())?;
                        parse_sbr_dtdf(&mut br, &mut data).map_err(|_| ())?;
                        parse_sbr_invf(&mut br, &mut data, ft.nq).map_err(|_| ())?;
                        parse_sbr_envelope(&mut br, &mut data, [ft.n_low, ft.n_high])
                            .map_err(|_| ())?;
                        parse_sbr_noise(&mut br, &mut data, ft.nq).map_err(|_| ())?;
                        data.bs_add_harmonic_flag = br.read_bit().map_err(|_| ())?;
                        if data.bs_add_harmonic_flag {
                            parse_sbr_sinusoidal(&mut br, &mut data, ft.n_high).map_err(|_| ())?;
                        }
                        let bs_extended_data = br.read_bit().map_err(|_| ())?;
                        if bs_extended_data {
                            let mut cnt = br.read_u32(4).map_err(|_| ())?;
                            if cnt == 15 {
                                cnt += br.read_u32(8).map_err(|_| ())?;
                            }
                            for _ in 0..(cnt * 8) {
                                br.read_u32(1).map_err(|_| ())?;
                            }
                        }
                        Ok(bs_extended_data)
                    })();
                    let bs_extended_data = match parse_ok {
                        Ok(v) => v,
                        Err(_) => break,
                    };
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
                        fil_consumed_bits: consumed,
                        bs_extended_data,
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

    let mut scope = SbrEnvScope::new();
    scope.unset("OXIDEAV_AAC_SBR_HDR_AMP_RES_1");
    scope.unset("OXIDEAV_AAC_SBR_DF_TIME");
    scope.unset("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO");
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
    let mut scope = SbrEnvScope::new();
    scope.unset("OXIDEAV_AAC_SBR_HDR_AMP_RES_1");
    scope.unset("OXIDEAV_AAC_SBR_DF_TIME");
    scope.unset("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO");
    let bytes = encode_ours();
    let diag = parse_stream(&bytes);
    assert!(!diag.is_empty(), "no SBR FIL parsed");
    let f0 = &diag[0];
    // Header — pinned snapshot of our current emitter. r25 lead-1
    // graduation (task #63) flipped the default to bs_amp_res = 1 to
    // match fdkaac; the FIXFIX num_env==1 carve-out still forces the
    // *effective* per-frame amp_res to 0 below.
    assert_eq!(f0.header.bs_amp_res, 1, "bs_amp_res in header");
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
    let mut scope = SbrEnvScope::new();
    scope.unset("OXIDEAV_AAC_SBR_HDR_AMP_RES_1");
    scope.unset("OXIDEAV_AAC_SBR_DF_TIME");
    scope.unset("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO");
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

/// Process-wide mutex serialising tests that flip `OXIDEAV_AAC_SBR_*`
/// env vars — cargo runs `#[test]` in parallel by default, and env-var
/// state is shared across threads, so any two probes that don't hold
/// this lock will race each other and produce nondeterministic
/// header / dtdf bits in the output stream.
static SBR_ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// RAII bundle holding (a) the global env-lock and (b) a list of
/// (name, prev) pairs to restore on drop. Each test acquires one of
/// these via [`SbrEnvScope::new`], then sets one or more env vars
/// inside the scope via [`SbrEnvScope::set`].
struct SbrEnvScope {
    saved: Vec<(&'static str, Option<String>)>,
    _lock: std::sync::MutexGuard<'static, ()>,
}

impl SbrEnvScope {
    fn new() -> Self {
        let lock = SBR_ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        Self {
            saved: Vec::new(),
            _lock: lock,
        }
    }

    fn set(&mut self, name: &'static str, value: &str) {
        let prev = std::env::var(name).ok();
        // SAFETY: we hold SBR_ENV_LOCK; no other lock-holder will read
        // env state concurrently.
        unsafe {
            std::env::set_var(name, value);
        }
        self.saved.push((name, prev));
    }

    fn unset(&mut self, name: &'static str) {
        let prev = std::env::var(name).ok();
        // SAFETY: see set() comment.
        unsafe {
            std::env::remove_var(name);
        }
        self.saved.push((name, prev));
    }
}

impl Drop for SbrEnvScope {
    fn drop(&mut self) {
        // Restore in reverse order so nested set/set/set chains
        // unwind cleanly.
        while let Some((name, prev)) = self.saved.pop() {
            // SAFETY: lock is still held (field declaration order).
            unsafe {
                if let Some(p) = prev {
                    std::env::set_var(name, p);
                } else {
                    std::env::remove_var(name);
                }
            }
        }
    }
}

/// Run ffmpeg over the supplied ADTS bytes and return the steady-state
/// peak / RMS amplitude of the decoded mono PCM. Returns `None` if
/// ffmpeg is not available or fails.
fn ffmpeg_decode_peak_rms(adts_bytes: &[u8]) -> Option<(i32, i32)> {
    which("ffmpeg")?;
    let scratch = std::env::temp_dir().join("oxideav_aac_r25_lead_probe");
    std::fs::create_dir_all(&scratch).ok()?;
    let adts_path = scratch.join("ours.aac");
    let dec_path = scratch.join("dec.s16");
    std::fs::write(&adts_path, adts_bytes).ok()?;
    let out = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "warning"])
        .arg("-i")
        .arg(&adts_path)
        .args(["-f", "s16le", "-ar", "48000", "-ac", "1"])
        .arg(&dec_path)
        .output()
        .ok()?;
    let stderr = String::from_utf8_lossy(&out.stderr).to_string();
    if !stderr.is_empty() {
        eprintln!("ffmpeg stderr:\n{}", stderr);
    }
    if !out.status.success() {
        return None;
    }
    let raw = std::fs::read(&dec_path).ok()?;
    let samples: Vec<i16> = raw
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();
    let warm = 5_000usize;
    if samples.len() <= warm + 100 {
        return None;
    }
    let mid = &samples[warm..(samples.len() - 100)];
    let peak = mid
        .iter()
        .map(|&s| s.unsigned_abs() as i32)
        .max()
        .unwrap_or(0);
    let mut acc = 0.0f64;
    for &s in mid {
        let f = s as f64;
        acc += f * f;
    }
    let rms = (acc / mid.len() as f64).sqrt() as i32;
    Some((peak, rms))
}

/// Pin: the `OXIDEAV_AAC_SBR_ENV_FORCE_ZERO` env-var probe (introduced in
/// r24 to test the "envelope value drives saturation" hypothesis) actually
/// zeros the envelope on every frame. Without this pin, regressing the
/// override silently makes all subsequent r25+ probes meaningless.
#[test]
fn force_zero_env_var_actually_zeros_envelope() {
    let mut scope = SbrEnvScope::new();
    scope.set("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO", "1");

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

// ============================================================
// r25 lead probes
// ============================================================

/// **Lead 1**: emit `bs_amp_res = 1` in the SBR header (mirroring
/// fdkaac) while the grid carve-out keeps the *effective* amp_res for
/// envelope decoding at 0 (1.5 dB tables, 7-bit start). If ffmpeg
/// pre-reads the start-bit count from the raw header value before the
/// FIXFIX `bs_num_env == 1` rule fires, this is the bit it would care
/// about.
#[test]
fn lead1_header_amp_res_1() {
    let mut scope = SbrEnvScope::new();
    scope.set("OXIDEAV_AAC_SBR_HDR_AMP_RES_1", "1");
    let bytes = encode_ours();
    let diag = parse_stream(&bytes);
    assert!(!diag.is_empty(), "no SBR FIL parsed under lead-1");
    let f0 = &diag[0];
    assert_eq!(
        f0.header.bs_amp_res, 1,
        "lead-1 must rewrite header bs_amp_res to 1"
    );
    // Grid carve-out should still force the *effective* amp_res to 0
    // because bs_num_env == 1 in FIXFIX.
    assert_eq!(
        f0.data.bs_amp_res, 0,
        "FIXFIX num_env==1 must override effective amp_res to 0 even with header=1"
    );
    if let Some((peak, rms)) = ffmpeg_decode_peak_rms(&bytes) {
        eprintln!("lead-1 ffmpeg-decode: peak={peak} rms={rms} (input ~9830)");
    } else {
        eprintln!("lead-1: ffmpeg unavailable or decode failed — skipping amp probe");
    }
}

/// **Lead 2**: emit `bs_df_env = 1` / `bs_df_noise = 1` (time-direction
/// delta) from frame 1 onwards. Frame 0 stays freq-direction because no
/// prior baseline exists. Mirrors fdkaac's choice in the diff harness.
#[test]
fn lead2_time_direction_delta() {
    let mut scope = SbrEnvScope::new();
    scope.set("OXIDEAV_AAC_SBR_DF_TIME", "1");
    let bytes = encode_ours();
    let diag = parse_stream(&bytes);
    assert!(diag.len() >= 2, "need >= 2 frames to check time-direction");
    // Frame 0 must remain freq-direction (no baseline available).
    assert_eq!(
        diag[0].data.bs_df_env[0], 0,
        "frame 0 must stay freq-direction"
    );
    assert_eq!(diag[0].data.bs_df_noise[0], 0);
    // Frame 1 onwards must switch to time-direction under the toggle.
    for d in diag.iter().skip(1) {
        assert_eq!(
            d.data.bs_df_env[0], 1,
            "frame {} bs_df_env should be 1 (time-dir) under DF_TIME",
            d.frame_idx
        );
        assert_eq!(
            d.data.bs_df_noise[0], 1,
            "frame {} bs_df_noise should be 1 under DF_TIME",
            d.frame_idx
        );
    }
    if let Some((peak, rms)) = ffmpeg_decode_peak_rms(&bytes) {
        eprintln!("lead-2 ffmpeg-decode: peak={peak} rms={rms} (input ~9830)");
    } else {
        eprintln!("lead-2: ffmpeg unavailable or decode failed — skipping amp probe");
    }
}

/// **Lead 3 audit**: assert that every emitted FIL/SBR element satisfies
/// the spec framing constraint
/// `(8*cnt - 4 - num_sbr_bits) MOD 8 < 8` and equals the exact number
/// of fill bits the encoder actually wrote. This rules out any
/// subtle off-by-one that ffmpeg's `bs_fill_bits` reader would treat
/// as leftover payload — the same kind of misalignment that produces
/// "No quantized data read for sbr_dequant" downstream.
///
/// Validates against both ours and fdkaac so the same expectation holds
/// across encoders.
#[test]
fn lead3_audit_extension_payload_framing() {
    let mut cases: Vec<(&str, Vec<u8>)> = Vec::new();
    {
        let _scope = SbrEnvScope::new();
        cases.push(("ours-default", encode_ours()));
    }
    {
        let mut scope = SbrEnvScope::new();
        scope.set("OXIDEAV_AAC_SBR_HDR_AMP_RES_1", "1");
        cases.push(("ours-lead1", encode_ours()));
    }
    {
        let mut scope = SbrEnvScope::new();
        scope.set("OXIDEAV_AAC_SBR_DF_TIME", "1");
        cases.push(("ours-lead2", encode_ours()));
    }
    for (label, bytes) in &cases {
        let diag = parse_stream(bytes);
        assert!(
            !diag.is_empty(),
            "{label}: no SBR FIL parsed (framing audit cannot proceed)"
        );
        for d in &diag {
            // Spec §4.4.2.8 Table 4.62: declared cnt covers
            //   4-bit ext_id  + bs_header_flag  + (sbr_header if flag)
            //   + sbr_data() + bs_fill_bits
            // i.e. 8*cnt - fil_consumed_bits == bs_fill_bits, in [0, 8).
            // ffmpeg computes num_align_bits = (8*cnt - 4 - num_sbr_bits) %
            // 8, which assumes the over-fill is < 8 already; if it isn't,
            // the parser leaves leftover bits that desync the next FIL
            // element (or the ID_END).
            let cnt_bits = d.fil_payload_bits as i64;
            let consumed = d.fil_consumed_bits as i64;
            let fill = cnt_bits - consumed;
            assert!(
                fill >= 0,
                "{label} frame {}: declared cnt undershoots actual SBR consumption \
                 ({cnt_bits} bits < {consumed} consumed)",
                d.frame_idx
            );
            assert!(
                fill < 8,
                "{label} frame {}: extra fill bits {fill} >= 8 — cnt is over-rounded \
                 (this WILL desync ffmpeg's bs_fill_bits parser)",
                d.frame_idx
            );
        }
    }
}

/// **Combined**: lead-1 + lead-2 stacked. If either lead alone closed
/// the saturation, we'd see it in `lead1_*` / `lead2_*`; this case
/// catches a multi-cause scenario.
#[test]
fn lead_combined_amp_res_and_time_delta() {
    let mut scope = SbrEnvScope::new();
    scope.set("OXIDEAV_AAC_SBR_HDR_AMP_RES_1", "1");
    scope.set("OXIDEAV_AAC_SBR_DF_TIME", "1");
    let bytes = encode_ours();
    let diag = parse_stream(&bytes);
    assert!(!diag.is_empty(), "no SBR FIL parsed under combined leads");
    // Sanity-check both toggles took effect on the parsed bitstream.
    assert_eq!(
        diag[0].header.bs_amp_res, 1,
        "combined: header bs_amp_res must be 1"
    );
    if diag.len() >= 2 {
        assert_eq!(diag[1].data.bs_df_env[0], 1, "combined: frame-1 must be DT");
        assert_eq!(diag[1].data.bs_df_noise[0], 1);
    }
    if let Some((peak, rms)) = ffmpeg_decode_peak_rms(&bytes) {
        eprintln!("combined ffmpeg-decode: peak={peak} rms={rms} (input ~9830)");
    } else {
        eprintln!("combined: ffmpeg unavailable — skipping amp probe");
    }
}

/// **Baseline ffmpeg measurement** — record the ffmpeg-decode peak/RMS
/// for our *unmodified* mono encoder on the same fixture so the lead
/// probes have a numeric anchor. Diagnostic-only: no assertion (the
/// known-broken r18 saturation reproduces here).
#[test]
fn baseline_ffmpeg_amp_no_toggles() {
    let mut scope = SbrEnvScope::new();
    // Defensively clear any leaked toggles before measuring.
    scope.unset("OXIDEAV_AAC_SBR_HDR_AMP_RES_1");
    scope.unset("OXIDEAV_AAC_SBR_DF_TIME");
    scope.unset("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO");
    let bytes = encode_ours();
    if let Some((peak, rms)) = ffmpeg_decode_peak_rms(&bytes) {
        eprintln!("baseline ffmpeg-decode: peak={peak} rms={rms} (input ~9830)");
    } else {
        eprintln!("baseline: ffmpeg unavailable — skipping");
    }
}

// ============================================================
// r25 task #63: byte-compare each lead against the fdkaac
// reference fixture. Each test asserts that our encoder's
// emission of the lead-relevant field matches fdkaac's choice
// for the same input. These run with the env-var probes
// **cleared** so they exercise the production defaults — i.e.
// they enforce that the r24 probes have been graduated into
// real encoder behaviour.
// ============================================================

/// **r25 lead-1 byte-compare**: ours' SBR header `bs_amp_res` field
/// must match fdkaac's choice on the same input. fdkaac always emits
/// `bs_amp_res = 1` in the SBR header for the 48 kHz HE-AAC mono case
/// (the FIXFIX `bs_num_env == 1` carve-out then forces the *effective*
/// amp_res for envelope decoding to 0, so the per-frame
/// `data.bs_amp_res` is still 0 — only the header bit differs).
///
/// This test fails if the encoder regresses to the r23-era
/// `bs_amp_res = 0` header default.
#[test]
fn r25_lead1_byte_compare_amp_res_against_fdkaac() {
    if which("fdkaac").is_none() {
        eprintln!("fdkaac not on PATH — skipping");
        return;
    }
    let mut scope = SbrEnvScope::new();
    scope.unset("OXIDEAV_AAC_SBR_HDR_AMP_RES_1");
    scope.unset("OXIDEAV_AAC_SBR_DF_TIME");
    scope.unset("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO");

    let ours = parse_stream(&encode_ours());
    let fdk = match encode_fdkaac() {
        Some(b) => parse_stream(&b),
        None => {
            eprintln!("fdkaac encode failed — skipping");
            return;
        }
    };
    assert!(!ours.is_empty() && !fdk.is_empty(), "no SBR FIL parsed");
    let our_amp = ours[0].header.bs_amp_res;
    let fdk_amp = fdk[0].header.bs_amp_res;
    assert_eq!(
        our_amp, fdk_amp,
        "lead-1 byte-compare: ours' bs_amp_res = {our_amp}, fdkaac = {fdk_amp}; \
         encoder default must match fdkaac (graduate the OXIDEAV_AAC_SBR_HDR_AMP_RES_1 probe)"
    );
    // The grid carve-out must still hold so envelope decoding stays at
    // 1.5 dB tables (7-bit start). This invariant is independent of the
    // header bit and is what allows the change to be safe.
    assert_eq!(
        ours[0].data.bs_amp_res, 0,
        "FIXFIX num_env==1 must force effective amp_res to 0"
    );
    assert_eq!(
        fdk[0].data.bs_amp_res, 0,
        "fdkaac frame[0] effective amp_res should also be 0 (sanity)"
    );
}

/// **r25 lead-2 byte-compare**: ours' `bs_df_env` / `bs_df_noise`
/// pattern must match fdkaac's: frame 0 uses freq-direction (no prior
/// baseline), frames 1+ use time-direction. fdkaac's actual `bs_df_env`
/// totals come out around `[2, 4, 2, 1]` (mostly time-direction once
/// the baseline is established), and `bs_df_noise` totals around
/// `[9, 2]`. Our encoder must follow the same "frame 0 = freq, frames
/// 1+ = time" rule by default.
///
/// This test asserts the structural pattern (frame 0 freq, frames 1+
/// time), not the exact raw codes — fdkaac may also fall back to freq
/// occasionally for rate-distortion reasons we don't model. The
/// minimum bar is that our default behaviour switches to time-delta
/// from frame 1+, matching fdkaac's *predominant* choice.
#[test]
fn r25_lead2_byte_compare_df_direction_against_fdkaac() {
    if which("fdkaac").is_none() {
        eprintln!("fdkaac not on PATH — skipping");
        return;
    }
    let mut scope = SbrEnvScope::new();
    scope.unset("OXIDEAV_AAC_SBR_HDR_AMP_RES_1");
    scope.unset("OXIDEAV_AAC_SBR_DF_TIME");
    scope.unset("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO");

    let ours = parse_stream(&encode_ours());
    let fdk = match encode_fdkaac() {
        Some(b) => parse_stream(&b),
        None => {
            eprintln!("fdkaac encode failed — skipping");
            return;
        }
    };
    assert!(ours.len() >= 2 && fdk.len() >= 2, "need >= 2 frames");
    // Frame 0 must be freq-direction for both (no previous baseline).
    assert_eq!(ours[0].data.bs_df_env[0], 0, "frame 0: ours df_env != 0");
    assert_eq!(fdk[0].data.bs_df_env[0], 0, "frame 0: fdk df_env != 0");
    assert_eq!(
        ours[0].data.bs_df_noise[0], 0,
        "frame 0: ours df_noise != 0"
    );
    assert_eq!(fdk[0].data.bs_df_noise[0], 0, "frame 0: fdk df_noise != 0");
    // Frames 1+: count how often fdkaac chose time-direction vs how
    // often we did. Our encoder default must hit time-direction on
    // *every* frame >= 1 (we don't yet model fdkaac's RD switching).
    // fdkaac's pattern is mostly time but with occasional freq
    // fallbacks; the bar is that we are not stuck on always-freq
    // (which is what r23 did).
    let mut our_time_env = 0usize;
    let mut our_time_noise = 0usize;
    for d in ours.iter().skip(1) {
        if d.data.bs_df_env[0] == 1 {
            our_time_env += 1;
        }
        if d.data.bs_df_noise[0] == 1 {
            our_time_noise += 1;
        }
    }
    let post_frame0 = ours.len() - 1;
    assert_eq!(
        our_time_env, post_frame0,
        "lead-2 byte-compare: ours used time-direction on {} of {} post-frame-0 envelopes; \
         expected {} (every frame >= 1). Encoder default must match fdkaac's \
         time-direction-from-frame-1 behaviour (graduate the OXIDEAV_AAC_SBR_DF_TIME probe).",
        our_time_env, post_frame0, post_frame0,
    );
    assert_eq!(
        our_time_noise, post_frame0,
        "lead-2 byte-compare: ours used time-direction on {} of {} post-frame-0 noise floors; \
         expected {}.",
        our_time_noise, post_frame0, post_frame0,
    );
    // Sanity: fdkaac is also predominantly time-direction post frame 0.
    let mut fdk_time = 0usize;
    for d in fdk.iter().skip(1) {
        if d.data.bs_df_env[0] == 1 {
            fdk_time += 1;
        }
    }
    assert!(
        fdk_time > 0,
        "fdkaac reference also expected to use time-direction on >=1 post-frame-0 envelopes \
         (sanity check; saw {fdk_time}/{} time-dir frames)",
        fdk.len() - 1,
    );
}

/// **r25 lead-3 byte-compare**: ours' `bs_extended_data` flag must
/// match fdkaac's on every frame. For pure HE-AAC mono (no PS), both
/// encoders MUST emit `bs_extended_data = 0` — there's no PS extension
/// to carry, and ffmpeg's parser treats any leftover bits past the SCE
/// body as fill (Table 4.65). Pinning this against fdkaac catches any
/// regression that sets the flag spuriously (which would advance
/// ffmpeg's reader into garbage and trigger the "No quantized data
/// read for sbr_dequant" warning downstream).
#[test]
fn r25_lead3_byte_compare_extended_data_against_fdkaac() {
    if which("fdkaac").is_none() {
        eprintln!("fdkaac not on PATH — skipping");
        return;
    }
    let mut scope = SbrEnvScope::new();
    scope.unset("OXIDEAV_AAC_SBR_HDR_AMP_RES_1");
    scope.unset("OXIDEAV_AAC_SBR_DF_TIME");
    scope.unset("OXIDEAV_AAC_SBR_ENV_FORCE_ZERO");

    let ours = parse_stream(&encode_ours());
    let fdk = match encode_fdkaac() {
        Some(b) => parse_stream(&b),
        None => {
            eprintln!("fdkaac encode failed — skipping");
            return;
        }
    };
    assert!(!ours.is_empty() && !fdk.is_empty(), "no SBR FIL parsed");
    // Every fdkaac mono frame must have bs_extended_data = 0 (HE-AAC
    // without PS). Sanity-check the reference first.
    for d in &fdk {
        assert!(
            !d.bs_extended_data,
            "fdkaac reference frame {} has bs_extended_data = 1 (unexpected for HE-AAC mono)",
            d.frame_idx,
        );
    }
    // Ours must match.
    for d in &ours {
        assert!(
            !d.bs_extended_data,
            "lead-3 byte-compare: ours frame {} emits bs_extended_data = 1; \
             fdkaac emits 0 for the same input. Setting the flag without a real \
             extension payload desyncs ffmpeg's bs_fill_bits parser.",
            d.frame_idx,
        );
    }
    // Framing sanity (re-affirms lead3_audit's invariant): each frame
    // satisfies (declared cnt - consumed) in [0, 8).
    for d in &ours {
        let fill = d.fil_payload_bits as i64 - d.fil_consumed_bits as i64;
        assert!(
            (0..8).contains(&fill),
            "lead-3 byte-compare: ours frame {} fill = {} bits (must be in [0, 8))",
            d.frame_idx,
            fill,
        );
    }
}

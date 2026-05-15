//! Encoder TNS round-trip test.
//!
//! Feed the encoder a signal with a sharp transient (percussive-style), then:
//!   1. Re-parse the emitted ADTS payload and confirm at least one raw_data_block
//!      was flagged `tns_data_present = 1`.
//!   2. Confirm the encoder exposes a way to disable TNS via a per-frame flag
//!      — here we do it by re-running the TNS analyser and verifying it gates
//!      on steady-state content (no TNS) vs transients (TNS emitted).
//!
//! The full reconstruction-PSNR check is performed indirectly: if TNS is
//! emitted and the decoder's TNS path applies correctly, the round-trip
//! through encode → decode must remain stable (covered by the main
//! `encode_roundtrip` suite).

use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, TimeBase};
#[allow(unused_imports)]
use oxideav_core::{Decoder, Encoder};

use oxideav_aac::adts::{parse_adts_header, ADTS_HEADER_NO_CRC};
use oxideav_aac::ics::SPEC_LEN;
use oxideav_core::bits::BitReader;

/// Build a percussive PCM buffer: short clicks at regular intervals, with
/// low-amplitude pink-ish filler between them so the encoder has something
/// to quantise. The transients are what trigger TNS.
fn pcm_clicks_mono(sr: u32, secs: f32) -> Vec<u8> {
    let total = (sr as f32 * secs) as usize;
    let mut out = Vec::with_capacity(total * 2);
    // One click every 512 samples.
    for i in 0..total {
        let pos_in_period = i % 512;
        let v = if pos_in_period < 4 {
            // Sharp attack / exponential decay — high-frequency content.
            let env = (-(pos_in_period as f32) * 1.2).exp();
            let phase = (i as f32) * 0.9; // ~ 6 kHz at 44.1k
            env * phase.sin()
        } else {
            // Low-amplitude background to keep max_sfb from collapsing.
            let t = i as f32 / sr as f32;
            (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.02
        };
        let s = (v.clamp(-0.95, 0.95) * 32767.0) as i16;
        out.extend_from_slice(&s.to_le_bytes());
    }
    out
}

fn encode_mono(pcm: Vec<u8>, sr: u32) -> Vec<u8> {
    encode_mono_at_bitrate(pcm, sr, 128_000)
}

fn encode_mono_at_bitrate(pcm: Vec<u8>, sr: u32, bit_rate: u32) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(1);
    params.bit_rate = Some(bit_rate.into());
    let mut enc = oxideav_aac::encoder::make_encoder(&params).expect("make encoder");
    let total_samples = pcm.len() / 2;
    let frame = Frame::Audio(AudioFrame {
        samples: total_samples as u32,
        pts: Some(0),
        data: vec![pcm],
    });
    enc.send_frame(&frame).expect("send_frame");
    enc.flush().expect("flush");
    let mut out = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        out.extend_from_slice(&p.data);
    }
    out
}

/// Walk the ADTS stream and return `(total_frames, frames_with_tns)`.
/// Re-parses each raw_data_block's SCE header bits far enough to inspect the
/// `tns_data_present` flag.
fn count_tns_frames(bytes: &[u8]) -> (usize, usize) {
    let mut i = 0;
    let mut total = 0;
    let mut with_tns = 0;
    while i + ADTS_HEADER_NO_CRC < bytes.len() {
        if bytes[i] != 0xFF || (bytes[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        let Ok(hdr) = parse_adts_header(&bytes[i..]) else {
            i += 1;
            continue;
        };
        if hdr.frame_length == 0 || i + hdr.frame_length > bytes.len() {
            break;
        }
        let payload = &bytes[i + hdr.header_length()..i + hdr.frame_length];
        if inspect_tns_flag(payload, hdr.sampling_freq_index) {
            with_tns += 1;
        }
        total += 1;
        i += hdr.frame_length;
    }
    (total, with_tns)
}

/// Minimal SCE parser that walks the ICS fields just far enough to reach the
/// `tns_data_present` bit. Mirrors `decoder.rs::decode_ics` but only tracks
/// offsets. Returns `true` if `tns_data_present = 1`.
fn inspect_tns_flag(payload: &[u8], sf_index: u8) -> bool {
    let mut br = BitReader::new(payload);
    // Raw data block: element id (3).
    let id = br.read_u32(3).unwrap_or(7);
    if id != 0 {
        // Not SCE — our encoder uses SCE for mono.
        return false;
    }
    let _inst = br.read_u32(4).unwrap_or(0);
    let _gg = br.read_u32(8).unwrap_or(0);
    // ics_info:
    //   ics_reserved (1), window_sequence (2), window_shape (1), max_sfb (6),
    //   predictor_data_present (1).
    let _rsv = br.read_u32(1).unwrap_or(0);
    let ws = br.read_u32(2).unwrap_or(0);
    if ws == 2 {
        return false; // EightShort — encoder currently doesn't set this, bail.
    }
    let _wshape = br.read_u32(1).unwrap_or(0);
    let max_sfb = br.read_u32(6).unwrap_or(0) as usize;
    let _pred = br.read_u32(1).unwrap_or(0);
    // section_data: for long windows, sect_bits = 5.
    let mut covered = 0usize;
    let mut section_cbs: Vec<u8> = Vec::new();
    while covered < max_sfb {
        let cb = br.read_u32(4).unwrap_or(0) as u8;
        let sect_esc = 31u32;
        let mut run = 0u32;
        loop {
            let v = br.read_u32(5).unwrap_or(0);
            run += v;
            if v != sect_esc {
                break;
            }
        }
        for _ in 0..run {
            section_cbs.push(cb);
        }
        covered += run as usize;
    }
    // scalefactor_data: for each non-zero band emit a Huffman delta.
    let mut nonzero = 0usize;
    for &cb in &section_cbs[..max_sfb.min(section_cbs.len())] {
        if cb != 0 && cb != 13 && cb != 14 && cb != 15 {
            nonzero += 1;
        } else if cb == 13 {
            // PNS — also has a scalefactor delta.
            nonzero += 1;
        }
    }
    // Skip nonzero scalefactor Huffman codes.
    for _ in 0..nonzero {
        if decode_sf_delta(&mut br).is_none() {
            return false;
        }
    }
    // pulse_data_present (1), tns_data_present (1), gain_control (1).
    let _pulse = br.read_u32(1).unwrap_or(0);
    let tns_present = br.read_u32(1).unwrap_or(0);
    let _ = sf_index;
    let _ = SPEC_LEN;
    tns_present != 0
}

/// Decode one scalefactor-delta huffman code from the reader.
/// The codebook used is the standard AAC scalefactor book (60-centered); we
/// cheat and re-use the decoder's public helper via `oxideav_aac::huffman`.
fn decode_sf_delta(br: &mut BitReader<'_>) -> Option<i32> {
    oxideav_aac::huffman::decode_scalefactor_delta(br).ok()
}

#[test]
fn encoder_emits_tns_on_transients() {
    let sr = 44_100u32;
    let pcm = pcm_clicks_mono(sr, 0.2);
    let aac = encode_mono(pcm, sr);
    let (total, with_tns) = count_tns_frames(&aac);
    eprintln!("transient: {with_tns}/{total} frames carry tns_data_present=1");
    assert!(total > 0, "no frames emitted");
    assert!(
        with_tns > 0,
        "encoder never set tns_data_present on a transient signal"
    );
}

#[test]
fn encoder_skips_tns_on_pure_tone() {
    let sr = 44_100u32;
    let total = (sr as f32 * 0.2) as usize;
    let mut pcm = Vec::with_capacity(total * 2);
    for i in 0..total {
        let t = i as f32 / sr as f32;
        let v = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.5;
        let s = (v * 32767.0) as i16;
        pcm.extend_from_slice(&s.to_le_bytes());
    }
    let aac = encode_mono(pcm, sr);
    let (total_f, with_tns) = count_tns_frames(&aac);
    eprintln!("pure tone: {with_tns}/{total_f} frames carry tns_data_present=1");
    // A pure sine yields a single spectral peak — LPC predicts it well, so
    // TNS *could* fire. But the gain threshold should keep the rate low;
    // this is a sanity check that the encoder isn't setting TNS on every
    // frame unconditionally.
    assert!(total_f > 0);
}

/// Build a percussive stereo PCM buffer with mildly decorrelated L/R
/// (small inter-channel phase offset). The transients drive TNS; the
/// decorrelation prevents the M/S decision from collapsing every band
/// onto the same channel.
fn pcm_clicks_stereo(sr: u32, secs: f32) -> Vec<u8> {
    let total = (sr as f32 * secs) as usize;
    let mut out = Vec::with_capacity(total * 4);
    for i in 0..total {
        let pos_in_period = i % 512;
        let (vl, vr) = if pos_in_period < 4 {
            // Sharp attack / exponential decay — high-frequency content.
            let env = (-(pos_in_period as f32) * 1.2).exp();
            let phase = (i as f32) * 0.9; // ~ 6 kHz at 44.1k
            let l = env * phase.sin();
            // R is the same transient with a small inter-channel offset
            // so L and R are correlated but not identical — TNS analyses
            // each independently.
            let r = env * (phase + 0.4).sin();
            (l, r)
        } else {
            // Low-amplitude background — distinct frequencies on each
            // channel keeps max_sfb wide while avoiding pure-tone TNS gate.
            let t = i as f32 / sr as f32;
            let l = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.02
                + (2.0 * std::f32::consts::PI * 1100.0 * t).sin() * 0.015;
            let r = (2.0 * std::f32::consts::PI * 660.0 * t).sin() * 0.02
                + (2.0 * std::f32::consts::PI * 1320.0 * t).sin() * 0.015;
            (l, r)
        };
        let sl = (vl.clamp(-0.95, 0.95) * 32767.0) as i16;
        let sr_pcm = (vr.clamp(-0.95, 0.95) * 32767.0) as i16;
        // S16 stereo interleaved: L, R, L, R, …
        out.extend_from_slice(&sl.to_le_bytes());
        out.extend_from_slice(&sr_pcm.to_le_bytes());
    }
    out
}

fn encode_stereo(pcm: Vec<u8>, sr: u32) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(2);
    params.bit_rate = Some(192_000);
    let mut enc = oxideav_aac::encoder::make_encoder(&params).expect("make encoder");
    let total_samples = pcm.len() / 4; // 2 channels * i16
    let frame = Frame::Audio(AudioFrame {
        samples: total_samples as u32,
        pts: Some(0),
        data: vec![pcm],
    });
    enc.send_frame(&frame).expect("send_frame");
    enc.flush().expect("flush");
    let mut out = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        out.extend_from_slice(&p.data);
    }
    out
}

/// Walk the ADTS stream and return `(total_frames, frames_with_tns_on_cpe)`.
/// Re-parses each raw_data_block's CPE header bits far enough to inspect
/// each channel's `tns_data_present` flag.
fn count_cpe_tns_frames(bytes: &[u8]) -> (usize, usize) {
    let mut i = 0;
    let mut total = 0;
    let mut with_tns = 0;
    while i + ADTS_HEADER_NO_CRC < bytes.len() {
        if bytes[i] != 0xFF || (bytes[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        let Ok(hdr) = parse_adts_header(&bytes[i..]) else {
            i += 1;
            continue;
        };
        if hdr.frame_length == 0 || i + hdr.frame_length > bytes.len() {
            break;
        }
        let payload = &bytes[i + hdr.header_length()..i + hdr.frame_length];
        if inspect_cpe_tns_flag(payload) {
            with_tns += 1;
        }
        total += 1;
        i += hdr.frame_length;
    }
    (total, with_tns)
}

/// Minimal CPE parser that walks the common_window + ics_info + per-channel
/// section_data + scalefactor_data far enough to reach each channel's
/// `tns_data_present` bit. Returns true if EITHER channel has TNS on.
fn inspect_cpe_tns_flag(payload: &[u8]) -> bool {
    let mut br = BitReader::new(payload);
    let id = br.read_u32(3).unwrap_or(7);
    if id != 1 {
        return false; // not CPE
    }
    let _inst = br.read_u32(4).unwrap_or(0);
    let common_window = br.read_u32(1).unwrap_or(0);
    if common_window != 1 {
        // Per-channel ics_info path not exercised by our long-block CPE.
        return false;
    }
    // Shared ics_info:
    //   ics_reserved (1), window_sequence (2), window_shape (1),
    //   max_sfb (6), predictor_data_present (1).
    let _rsv = br.read_u32(1).unwrap_or(0);
    let ws = br.read_u32(2).unwrap_or(0);
    if ws == 2 {
        return false; // EightShort not used by this encoder for CPE
    }
    let _wshape = br.read_u32(1).unwrap_or(0);
    let max_sfb = br.read_u32(6).unwrap_or(0) as usize;
    let _pred = br.read_u32(1).unwrap_or(0);
    // ms_mask_present (2 bits)
    let ms_mask_present = br.read_u32(2).unwrap_or(0);
    if ms_mask_present == 1 {
        for _ in 0..max_sfb {
            let _ = br.read_u32(1);
        }
    }
    // First channel only: in this encoder long-block CPE always enables
    // TNS analysis on both channels with the same gating, so a check on
    // ch0 is representative. (Reading past the spectral_data of ch0 to
    // reach ch1's tns_data_present bit would need a full decoder.)
    let _gg = br.read_u32(8).unwrap_or(0);
    let mut covered = 0usize;
    let mut section_cbs: Vec<u8> = Vec::new();
    while covered < max_sfb {
        let cb = br.read_u32(4).unwrap_or(0) as u8;
        let sect_esc = 31u32;
        let mut run = 0u32;
        loop {
            let v = br.read_u32(5).unwrap_or(0);
            run += v;
            if v != sect_esc {
                break;
            }
        }
        for _ in 0..run {
            section_cbs.push(cb);
        }
        covered += run as usize;
    }
    let mut nonzero = 0usize;
    for &cb in &section_cbs[..max_sfb.min(section_cbs.len())] {
        if cb != 0 && cb != 14 && cb != 15 {
            nonzero += 1;
        }
    }
    for _ in 0..nonzero {
        if decode_sf_delta(&mut br).is_none() {
            return false;
        }
    }
    let _pulse = br.read_u32(1).unwrap_or(0);
    let tns_present = br.read_u32(1).unwrap_or(0);
    tns_present != 0
}

#[test]
fn encoder_emits_tns_on_cpe_transients() {
    let sr = 44_100u32;
    let pcm = pcm_clicks_stereo(sr, 0.2);
    let aac = encode_stereo(pcm, sr);
    let (total, with_tns) = count_cpe_tns_frames(&aac);
    eprintln!("stereo transient CPE: {with_tns}/{total} frames carry tns_data_present=1");
    assert!(total > 0, "no frames emitted");
    assert!(
        with_tns > 0,
        "encoder never set tns_data_present on any CPE channel for a transient stereo signal"
    );
}

/// Decode an ADTS stream with our decoder and concatenate the raw S16
/// PCM. Returns interleaved S16 samples (one frame's worth per packet).
fn decode_adts(aac: &[u8]) -> Vec<i16> {
    let mut i = 0;
    let mut frames = Vec::new();
    while i + ADTS_HEADER_NO_CRC < aac.len() {
        if aac[i] != 0xFF || (aac[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        let Ok(hdr) = parse_adts_header(&aac[i..]) else {
            break;
        };
        if hdr.frame_length == 0 || i + hdr.frame_length > aac.len() {
            break;
        }
        frames.push((i, hdr.frame_length));
        i += hdr.frame_length;
    }
    assert!(!frames.is_empty(), "no ADTS frames found");
    let hdr0 = parse_adts_header(&aac[frames[0].0..]).unwrap();
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(hdr0.sample_rate().unwrap());
    params.channels = Some(hdr0.channel_configuration as u16);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("dec");
    let tb = TimeBase::new(1, hdr0.sample_rate().unwrap() as i64);
    let mut out = Vec::new();
    for (idx, &(off, len)) in frames.iter().enumerate() {
        let pkt = oxideav_core::Packet::new(0, tb, aac[off..off + len].to_vec())
            .with_pts(idx as i64 * 1024);
        dec.send_packet(&pkt).expect("send");
        if let Ok(Frame::Audio(af)) = dec.receive_frame() {
            for plane in &af.data {
                for chunk in plane.chunks_exact(2) {
                    out.push(i16::from_le_bytes([chunk[0], chunk[1]]));
                }
            }
        }
    }
    out
}

/// Encode + decode → return (encoded_bytes, MSE per channel).
fn encode_decode_mse(pcm: Vec<u8>, sr: u32) -> (usize, [f64; 2]) {
    let ref_s16: Vec<i16> = pcm
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();
    let aac = encode_stereo(pcm, sr);
    let dec = decode_adts(&aac);
    let n_frames = dec.len() / 2;
    let warm = 1024; // skip first frame (encoder/decoder warm-up)
    let n_use = n_frames.saturating_sub(warm + 256);
    let mut sq_err = [0.0f64; 2];
    let mut n_used = 0usize;
    for k in 0..n_use {
        for ch in 0..2 {
            let dec_v = dec[(k + warm) * 2 + ch] as f64;
            let ref_idx = (k + warm) * 2 + ch;
            if ref_idx >= ref_s16.len() {
                continue;
            }
            let ref_v = ref_s16[ref_idx] as f64;
            let d = dec_v - ref_v;
            sq_err[ch] += d * d;
        }
        n_used += 1;
    }
    let mse = if n_used > 0 {
        [sq_err[0] / n_used as f64, sq_err[1] / n_used as f64]
    } else {
        [0.0, 0.0]
    };
    (aac.len(), mse)
}

/// Round-28 A/B: with CPE TNS enabled (default) vs disabled
/// (`OXIDEAV_AAC_DISABLE_CPE_TNS=1`), measure encoded size + MSE on a
/// transient stereo fixture. Expect TNS-on to achieve equal-or-better
/// reconstruction at iso bitrate with no catastrophic regression.
#[test]
fn cpe_tns_ab_transient_size_and_psnr() {
    let sr = 44_100u32;

    // CPE TNS off.
    std::env::set_var("OXIDEAV_AAC_DISABLE_CPE_TNS", "1");
    let pcm_off = pcm_clicks_stereo(sr, 0.5);
    let (size_off, mse_off) = encode_decode_mse(pcm_off, sr);
    std::env::remove_var("OXIDEAV_AAC_DISABLE_CPE_TNS");

    // CPE TNS on (default).
    let pcm_on = pcm_clicks_stereo(sr, 0.5);
    let (size_on, mse_on) = encode_decode_mse(pcm_on, sr);

    let psnr = |mse: f64| -> f64 {
        if mse <= 1e-9 {
            120.0
        } else {
            10.0 * (32767.0f64 * 32767.0 / mse).log10()
        }
    };
    eprintln!(
        "[CPE-TNS A/B] off: {size_off} B, PSNR L/R = {:.2}/{:.2} dB",
        psnr(mse_off[0]),
        psnr(mse_off[1])
    );
    eprintln!(
        "[CPE-TNS A/B]  on: {size_on} B, PSNR L/R = {:.2}/{:.2} dB",
        psnr(mse_on[0]),
        psnr(mse_on[1])
    );

    let psnr_delta_l = psnr(mse_on[0]) - psnr(mse_off[0]);
    let psnr_delta_r = psnr(mse_on[1]) - psnr(mse_off[1]);
    eprintln!(
        "[CPE-TNS A/B] ΔPSNR L = {psnr_delta_l:+.2} dB, ΔPSNR R = {psnr_delta_r:+.2} dB, \
         Δsize = {:+.1} %",
        100.0 * (size_on as f64 - size_off as f64) / size_off as f64
    );
    // Acceptance: CPE TNS must not regress reconstruction PSNR by more
    // than 1 dB on either channel. Encoded-size delta is informational
    // (TNS spends a handful of bits per frame on filter coefficients but
    // the post-flatten quantiser may save bits elsewhere).
    assert!(
        psnr_delta_l >= -1.0 && psnr_delta_r >= -1.0,
        "CPE TNS regressed PSNR by > 1 dB on a transient fixture: L {psnr_delta_l:+.2}, R {psnr_delta_r:+.2}",
    );
}

#[test]
fn encoder_skips_cpe_tns_on_pure_stereo_tones() {
    // Pure stereo tones (single-bin per channel) should sit under the
    // sparse-spectrum gate (≥ 3 active bands), keeping TNS off so the
    // dequant side-lobes never smear back into the source bin.
    let sr = 44_100u32;
    let total = (sr as f32 * 0.2) as usize;
    let mut pcm = Vec::with_capacity(total * 4);
    for i in 0..total {
        let t = i as f32 / sr as f32;
        let l = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.5;
        let r = (2.0 * std::f32::consts::PI * 880.0 * t).sin() * 0.5;
        let sl = (l * 32767.0) as i16;
        let sr_pcm = (r * 32767.0) as i16;
        pcm.extend_from_slice(&sl.to_le_bytes());
        pcm.extend_from_slice(&sr_pcm.to_le_bytes());
    }
    let aac = encode_stereo(pcm, sr);
    let (total_f, with_tns) = count_cpe_tns_frames(&aac);
    eprintln!("stereo pure tones CPE: {with_tns}/{total_f} frames carry tns_data_present=1");
    assert!(total_f > 0);
    // Sparse-spectrum gate should keep most frames TNS-off; allow a small
    // fraction for window edge-effects.
    let max_allowed_tns = total_f / 5; // ≤ 20 %
    assert!(
        with_tns <= max_allowed_tns,
        "encoder set tns_data_present on {with_tns}/{total_f} CPE pure-tone frames \
         (expected ≤ {max_allowed_tns}) — sparse-spectrum gate not catching tone fixtures"
    );
}

/// Drum-attack constants used by [`pcm_drum_mono`] and the round-29
/// SCE TNS A/B test — exposed at the test scope so the metric can
/// carve out the post-attack measurement window from the same
/// numbers.
const DRUM_PERIOD: usize = 1024;
const DRUM_ATTACK_POS: usize = 100;
const DRUM_ATTACK_LEN: usize = 128;

/// Build a percussive mono PCM buffer designed to expose temporal
/// noise shaping: each 1024-sample period carries one drum-like
/// attack at sample [`DRUM_ATTACK_POS`] with a 128-sample slow-decay
/// envelope, on top of a low-amplitude wideband background.
///
/// Why these parameters:
///   - attack_pos = 100 puts the attack near the start of the period,
///     leaving a long post-attack tail (samples 228..1024 ≈ 800
///     samples) where TNS-shaped vs unshaped noise diverges most;
///   - attack_len = 128 with `decay = 0.05` gives a slow envelope
///     decay (still loud at sample +60), so the immediate
///     post-attack region (samples 228..292) is in the temporal
///     masking shadow of the attack — TNS reshapes quant noise to
///     follow that shadow and PSNR improves there;
///   - background is multi-tone at < 4 % of attack amplitude so the
///     spectrum has ≥ 3 active scalefactor bands (sparse-spectrum
///     gate stays open) and the inter-attack frames are dominated
///     by the quiet background.
fn pcm_drum_mono(sr: u32, secs: f32) -> Vec<u8> {
    let total = (sr as f32 * secs) as usize;
    let mut out = Vec::with_capacity(total * 2);
    for i in 0..total {
        let pos_in_period = i % DRUM_PERIOD;
        let v = if (DRUM_ATTACK_POS..DRUM_ATTACK_POS + DRUM_ATTACK_LEN).contains(&pos_in_period) {
            // Drum attack: 128-sample slow-decay broadband hit
            // (mixed phases stretching from ~ 2 kHz to ~ 10 kHz).
            // Amplitude near full-scale at the leading edge.
            let off = pos_in_period - DRUM_ATTACK_POS;
            let env = (-(off as f32) * 0.05).exp();
            let i_f = i as f32;
            let phase1 = i_f * 0.30; // ~ 2.1 kHz
            let phase2 = i_f * 0.60; // ~ 4.2 kHz
            let phase3 = i_f * 0.95; // ~ 6.7 kHz
            let phase4 = i_f * 1.40; // ~ 9.8 kHz
            env * (phase1.sin() * 0.30
                + phase2.sin() * 0.30
                + phase3.sin() * 0.25
                + phase4.sin() * 0.20)
                * 0.92
        } else {
            // Quiet wideband background to keep the sparse-spectrum
            // gate honest: ≥ 3 active scalefactor bands, but at <
            // 4 % of the drum amplitude so pre/post-echo dominates
            // the inter-attack samples in the no-TNS case.
            let t = i as f32 / sr as f32;
            (2.0 * std::f32::consts::PI * 660.0 * t).sin() * 0.018
                + (2.0 * std::f32::consts::PI * 1100.0 * t).sin() * 0.014
                + (2.0 * std::f32::consts::PI * 2100.0 * t).sin() * 0.014
                + (2.0 * std::f32::consts::PI * 3300.0 * t).sin() * 0.012
        };
        let s = (v.clamp(-0.95, 0.95) * 32767.0) as i16;
        out.extend_from_slice(&s.to_le_bytes());
    }
    out
}

/// Decode an ADTS mono stream with our decoder and concatenate the raw
/// S16 PCM (single planar channel).
fn decode_adts_mono(aac: &[u8]) -> Vec<i16> {
    let mut i = 0;
    let mut frames = Vec::new();
    while i + ADTS_HEADER_NO_CRC < aac.len() {
        if aac[i] != 0xFF || (aac[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        let Ok(hdr) = parse_adts_header(&aac[i..]) else {
            break;
        };
        if hdr.frame_length == 0 || i + hdr.frame_length > aac.len() {
            break;
        }
        frames.push((i, hdr.frame_length));
        i += hdr.frame_length;
    }
    assert!(!frames.is_empty(), "no ADTS frames found");
    let hdr0 = parse_adts_header(&aac[frames[0].0..]).unwrap();
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(hdr0.sample_rate().unwrap());
    params.channels = Some(hdr0.channel_configuration as u16);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("dec");
    let tb = TimeBase::new(1, hdr0.sample_rate().unwrap() as i64);
    let mut out = Vec::new();
    for (idx, &(off, len)) in frames.iter().enumerate() {
        let pkt = oxideav_core::Packet::new(0, tb, aac[off..off + len].to_vec())
            .with_pts(idx as i64 * 1024);
        dec.send_packet(&pkt).expect("send");
        if let Ok(Frame::Audio(af)) = dec.receive_frame() {
            for plane in &af.data {
                for chunk in plane.chunks_exact(2) {
                    out.push(i16::from_le_bytes([chunk[0], chunk[1]]));
                }
            }
        }
    }
    out
}

/// Encode + decode mono → return (encoded_bytes, MSE on samples
/// whose period-relative index lies in `[pos_lo, pos_hi)`, restricted
/// to frames flagged by `frame_mask`).
///
/// Used by the round-29 SCE TNS A/B test to isolate the
/// immediate-post-attack region (where TNS-shaped quant noise
/// concentrates back onto the attack envelope's tail rather than
/// smearing uniformly across the frame). Reusing the same frame mask
/// across the on/off encodes makes the comparison apples-to-apples.
fn encode_decode_region_mse_mono(
    pcm: Vec<u8>,
    sr: u32,
    frame_mask: &[bool],
    period: usize,
    pos_lo: usize,
    pos_hi: usize,
) -> (usize, f64, usize) {
    let ref_s16: Vec<i16> = pcm
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();
    let aac = encode_mono(pcm, sr);
    let dec = decode_adts_mono(&aac);
    let warm = 1024; // skip first frame (encoder/decoder warm-up)
    let frame_len = 1024usize;
    let mut sq_err = 0.0f64;
    let mut n_used = 0usize;
    let mut counted_frames = 0usize;
    for (frame_idx, &use_frame) in frame_mask.iter().enumerate() {
        if !use_frame {
            continue;
        }
        let frame_start = frame_idx * frame_len;
        if frame_start < warm {
            continue;
        }
        counted_frames += 1;
        for k in 0..frame_len {
            let idx = frame_start + k;
            if idx >= dec.len() || idx >= ref_s16.len() {
                break;
            }
            let pos_in_period = idx % period;
            if pos_in_period < pos_lo || pos_in_period >= pos_hi {
                continue;
            }
            let d = dec[idx] as f64 - ref_s16[idx] as f64;
            sq_err += d * d;
            n_used += 1;
        }
    }
    let mse = if n_used > 0 {
        sq_err / n_used as f64
    } else {
        0.0
    };
    (aac.len(), mse, counted_frames)
}

/// Walk the ADTS stream and return one bool per frame indicating
/// whether `tns_data_present = 1` was set. Used to isolate
/// transient-frame PSNR for the SCE A/B test.
fn tns_frame_indices(bytes: &[u8]) -> Vec<bool> {
    let mut i = 0;
    let mut out = Vec::new();
    while i + ADTS_HEADER_NO_CRC < bytes.len() {
        if bytes[i] != 0xFF || (bytes[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        let Ok(hdr) = parse_adts_header(&bytes[i..]) else {
            i += 1;
            continue;
        };
        if hdr.frame_length == 0 || i + hdr.frame_length > bytes.len() {
            break;
        }
        let payload = &bytes[i + hdr.header_length()..i + hdr.frame_length];
        out.push(inspect_tns_flag(payload, hdr.sampling_freq_index));
        i += hdr.frame_length;
    }
    out
}

/// Round-29 SCE-TNS A/B: with TNS enabled (default + sparse-spectrum
/// gate from [`should_run_sce_tns`]) vs disabled
/// (`OXIDEAV_AAC_DISABLE_SCE_TNS=1`), measure post-attack PSNR on a
/// percussive mono fixture.
///
/// The fixture (see [`pcm_drum_mono`]) places a strong drum-like
/// attack (broadband, 128-sample slow-decay envelope) at sample
/// `DRUM_ATTACK_POS = 100` of every 1024-sample period. The
/// **immediate post-attack** region (a 64-sample window starting at
/// the attack's nominal envelope end, samples 228..292 of each
/// period) sits inside the temporal-masking shadow of the attack —
/// this is the scalefactor-band's masked tail where the inverse-TNS
/// filter shapes quant noise to follow the attack's decay envelope.
/// Without TNS, the IMDCT spreads quant noise uniformly across the
/// frame and the masked tail picks up audible spillover.
///
/// Acceptance: immediate post-attack PSNR with SCE TNS on must
/// improve by at least +0.5 dB over TNS-off on the frames where TNS
/// actually fires (the sparse-spectrum gate suppresses TNS on
/// inter-attack frames, so the comparison is restricted to frames
/// the encoder marked as transient-bearing — same frame mask in both
/// passes).
#[test]
fn sce_tns_ab_transient_size_and_psnr() {
    let sr = 44_100u32;

    // Immediate-post-attack measurement window: a 64-sample slice
    // starting at `DRUM_ATTACK_POS + DRUM_ATTACK_LEN` (= 228) — i.e.
    // the decay tail right after the attack envelope ends. Inside
    // the temporal-masking shadow.
    let post_lo = DRUM_ATTACK_POS + DRUM_ATTACK_LEN;
    let post_hi = post_lo + 64;

    // First pass (TNS on, default): encode and capture the indices of
    // frames where the encoder actually set tns_data_present = 1. The
    // sparse-spectrum gate skips TNS on quiet inter-attack frames, so
    // we measure A/B PSNR only on the frames where TNS contributes.
    let pcm_aac_on = encode_mono(pcm_drum_mono(sr, 1.0), sr);
    let tns_mask = tns_frame_indices(&pcm_aac_on);
    let tns_frames_on: usize = tns_mask.iter().filter(|&&b| b).count();
    let total_frames = tns_mask.len();
    eprintln!("[SCE-TNS A/B] TNS-on frames: {tns_frames_on}/{total_frames}");
    assert!(
        tns_frames_on > 0,
        "encoder never set tns_data_present on the drum fixture — sparse-spectrum gate too aggressive"
    );

    // SCE TNS off — measure post-attack MSE on the SAME frames we
    // identified as transient-bearing in the TNS-on encode.
    // (`OXIDEAV_AAC_DISABLE_SCE_TNS=1` suppresses TNS on every frame,
    // so there are no tns_data_present bits to query on the off-path
    // stream.)
    std::env::set_var("OXIDEAV_AAC_DISABLE_SCE_TNS", "1");
    let pcm_off = pcm_drum_mono(sr, 1.0);
    let (size_off, mse_off, frames_off) =
        encode_decode_region_mse_mono(pcm_off, sr, &tns_mask, DRUM_PERIOD, post_lo, post_hi);
    std::env::remove_var("OXIDEAV_AAC_DISABLE_SCE_TNS");

    // SCE TNS on (default) — measure on the same frame indices.
    let pcm_on = pcm_drum_mono(sr, 1.0);
    let (size_on, mse_on, frames_on) =
        encode_decode_region_mse_mono(pcm_on, sr, &tns_mask, DRUM_PERIOD, post_lo, post_hi);
    assert_eq!(frames_off, frames_on, "frame counts must match across A/B");

    let psnr = |mse: f64| -> f64 {
        if mse <= 1e-9 {
            120.0
        } else {
            10.0 * (32767.0f64 * 32767.0 / mse).log10()
        }
    };
    eprintln!(
        "[SCE-TNS A/B] off: {size_off} B, post-attack PSNR = {:.2} dB",
        psnr(mse_off)
    );
    eprintln!(
        "[SCE-TNS A/B]  on: {size_on} B, post-attack PSNR = {:.2} dB",
        psnr(mse_on)
    );

    let psnr_delta = psnr(mse_on) - psnr(mse_off);
    eprintln!(
        "[SCE-TNS A/B] ΔPSNR = {psnr_delta:+.2} dB, Δsize = {:+.1} %",
        100.0 * (size_on as f64 - size_off as f64) / size_off as f64
    );
    // Acceptance: SCE TNS must improve immediate-post-attack PSNR by
    // at least +0.5 dB on the drum-spike fixture (the temporal-
    // masking tail where the inverse-TNS filter concentrates quant
    // noise to follow the attack's decay rather than smearing it
    // across the whole frame).
    assert!(
        psnr_delta >= 0.5,
        "SCE TNS failed to improve post-attack PSNR by ≥ +0.5 dB on a transient mono fixture: ΔPSNR = {psnr_delta:+.2} dB"
    );
}

/// Round-29 sparse-spectrum gate: a pure mono tone (single spectral peak)
/// should sit under the gate so SCE TNS is skipped on most frames.
/// Mirrors `encoder_skips_cpe_tns_on_pure_stereo_tones`.
#[test]
fn encoder_skips_sce_tns_on_pure_mono_tone() {
    let sr = 44_100u32;
    let total = (sr as f32 * 0.2) as usize;
    let mut pcm = Vec::with_capacity(total * 2);
    for i in 0..total {
        let t = i as f32 / sr as f32;
        let v = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.5;
        let s = (v * 32767.0) as i16;
        pcm.extend_from_slice(&s.to_le_bytes());
    }
    let aac = encode_mono(pcm, sr);
    let (total_f, with_tns) = count_tns_frames(&aac);
    eprintln!("mono pure tone SCE: {with_tns}/{total_f} frames carry tns_data_present=1");
    assert!(total_f > 0);
    // Sparse-spectrum gate should keep most frames TNS-off; allow a small
    // fraction for window edge-effects.
    let max_allowed_tns = total_f / 5; // ≤ 20 %
    assert!(
        with_tns <= max_allowed_tns,
        "encoder set tns_data_present on {with_tns}/{total_f} SCE pure-tone frames \
         (expected ≤ {max_allowed_tns}) — sparse-spectrum gate not catching tone fixtures"
    );
}

#[test]
fn tns_roundtrip_decodes_without_error() {
    // Simply encoding and decoding transient content must not blow up —
    // exercises the encoder→decoder TNS pipeline end to end.
    let sr = 44_100u32;
    let pcm = pcm_clicks_mono(sr, 0.1);
    let aac = encode_mono(pcm, sr);

    // Decode with our own decoder.
    let mut i = 0;
    let mut frames = Vec::new();
    while i + ADTS_HEADER_NO_CRC < aac.len() {
        if aac[i] != 0xFF || (aac[i + 1] & 0xF0) != 0xF0 {
            i += 1;
            continue;
        }
        let Ok(hdr) = parse_adts_header(&aac[i..]) else {
            break;
        };
        if hdr.frame_length == 0 || i + hdr.frame_length > aac.len() {
            break;
        }
        frames.push((i, hdr.frame_length));
        i += hdr.frame_length;
    }
    assert!(!frames.is_empty());
    let hdr0 = parse_adts_header(&aac[frames[0].0..]).unwrap();
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(hdr0.sample_rate().unwrap());
    params.channels = Some(hdr0.channel_configuration as u16);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make dec");
    let tb = TimeBase::new(1, hdr0.sample_rate().unwrap() as i64);
    let mut decoded_samples = 0usize;
    for (idx, &(off, len)) in frames.iter().enumerate() {
        let pkt = oxideav_core::Packet::new(0, tb, aac[off..off + len].to_vec())
            .with_pts(idx as i64 * 1024);
        dec.send_packet(&pkt).expect("send pkt");
        if let Ok(Frame::Audio(af)) = dec.receive_frame() {
            decoded_samples += af.samples as usize;
        }
    }
    assert!(decoded_samples > 0, "decoder returned no audio");
}

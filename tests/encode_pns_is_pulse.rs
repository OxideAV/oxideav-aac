//! Targeted tests for the PNS, intensity stereo, and pulse_data
//! encoder paths that were enabled / added on top of the upstream
//! emission plumbing. Each test assembles a signal the feature should
//! obviously trigger on, inspects the resulting bitstream for the
//! right codebook/flag, and then round-trips through our own decoder
//! to confirm audible correctness.

use oxideav_aac::adts::parse_adts_header;
use oxideav_aac::ics::{parse_ics_info, parse_section_data, INTENSITY_HCB, NOISE_HCB};
use oxideav_aac::syntax::ElementType;
use oxideav_core::bits::BitReader;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, Packet, TimeBase};
#[allow(unused_imports)]
use oxideav_core::{Decoder, Encoder};

const SR: u32 = 44_100;

fn encode(pcm: Vec<u8>, channels: u16, bitrate: u64) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(SR);
    params.channels = Some(channels);
    params.bit_rate = Some(bitrate);
    let mut enc = oxideav_aac::encoder::make_encoder(&params).expect("make encoder");
    let samples = pcm.len() / (2 * channels as usize);
    let frame = Frame::Audio(AudioFrame {
        samples: samples as u32,
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

fn iter_adts_frames(bytes: &[u8]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + 7 < bytes.len() {
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

fn decode_self(bytes: &[u8], channels: u16) -> Vec<i16> {
    let frames = iter_adts_frames(bytes);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(SR);
    params.channels = Some(channels);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make dec");
    let tb = TimeBase::new(1, SR as i64);
    let mut out = Vec::<i16>::new();
    for (i, &(off, len)) in frames.iter().enumerate() {
        let pkt = Packet::new(0, tb, bytes[off..off + len].to_vec()).with_pts(i as i64 * 1024);
        dec.send_packet(&pkt).unwrap();
        if let Ok(Frame::Audio(af)) = dec.receive_frame() {
            for chunk in af.data[0].chunks_exact(2) {
                out.push(i16::from_le_bytes([chunk[0], chunk[1]]));
            }
        }
    }
    out
}

/// Parse the first SCE-frame's section_data. Returns (max_sfb, cbs,
/// sf_index). Assumes long blocks (our encoder's default).
fn parse_first_sce(bytes: &[u8]) -> (u8, Vec<u8>, u8) {
    let frames = iter_adts_frames(bytes);
    assert!(!frames.is_empty(), "no ADTS frames found");
    let (off, len) = frames[0];
    let hdr = parse_adts_header(&bytes[off..]).unwrap();
    let sf_index = hdr.sampling_freq_index;
    let payload = &bytes[off + hdr.header_length()..off + len];
    let mut br = BitReader::new(payload);
    let id = br.read_u32(3).unwrap();
    assert_eq!(id, ElementType::Sce as u32, "expected SCE");
    let _tag = br.read_u32(4).unwrap();
    let _gg = br.read_u32(8).unwrap();
    let info = parse_ics_info(&mut br, sf_index).unwrap();
    let sec = parse_section_data(&mut br, &info).unwrap();
    (info.max_sfb, sec.sfb_cb, sf_index)
}

/// Parse a CPE common-window frame into (max_sfb, ch0 cbs, ch1 cbs).
fn parse_first_cpe(bytes: &[u8]) -> (u8, Vec<u8>, Vec<u8>) {
    let frames = iter_adts_frames(bytes);
    assert!(!frames.is_empty(), "no ADTS frames found");
    let (off, len) = frames[0];
    let hdr = parse_adts_header(&bytes[off..]).unwrap();
    let sf_index = hdr.sampling_freq_index;
    let payload = &bytes[off + hdr.header_length()..off + len];
    let mut br = BitReader::new(payload);
    let id = br.read_u32(3).unwrap();
    assert_eq!(id, ElementType::Cpe as u32, "expected CPE");
    let _tag = br.read_u32(4).unwrap();
    let common = br.read_bit().unwrap();
    assert!(common, "expected common_window CPE");
    let info = parse_ics_info(&mut br, sf_index).unwrap();
    let ms_mask_present = br.read_u32(2).unwrap();
    if ms_mask_present == 1 {
        let total = info.num_window_groups as usize * info.max_sfb as usize;
        for _ in 0..total {
            let _ = br.read_bit().unwrap();
        }
    }
    let ch0_cbs = parse_ch_through_spectral(&mut br, &info).expect("parse ch0");
    let ch1_cbs = parse_ch_section_only(&mut br, &info).expect("parse ch1");
    (info.max_sfb, ch0_cbs, ch1_cbs)
}

fn parse_ch_through_spectral(
    br: &mut BitReader<'_>,
    info: &oxideav_aac::ics::IcsInfo,
) -> oxideav_core::Result<Vec<u8>> {
    let _gg = br.read_u32(8)?;
    let sec = parse_section_data(br, info)?;
    let sf = oxideav_aac::ics::parse_scalefactors(br, info, &sec, 0)?;
    let pulse_present = br.read_bit()?;
    if pulse_present {
        let _ = oxideav_aac::pulse::parse_pulse_data(br)?;
    }
    let tns_present = br.read_bit()?;
    if tns_present {
        let nw = if info.window_sequence.is_eight_short() {
            8
        } else {
            1
        };
        let _ = oxideav_aac::tns::parse_tns_data(br, info.window_sequence, nw)?;
    }
    let _gc = br.read_bit()?;
    let mut coef = [0.0f32; 1024];
    if info.window_sequence.is_eight_short() {
        oxideav_aac::ics::decode_spectrum_short(br, info, &sec, &sf, &mut coef)?;
    } else {
        oxideav_aac::ics::decode_spectrum_long(br, info, &sec, &sf, &mut coef)?;
    }
    Ok(sec.sfb_cb)
}

fn parse_ch_section_only(
    br: &mut BitReader<'_>,
    info: &oxideav_aac::ics::IcsInfo,
) -> oxideav_core::Result<Vec<u8>> {
    let _gg = br.read_u32(8)?;
    let sec = parse_section_data(br, info)?;
    Ok(sec.sfb_cb)
}

fn goertzel(samples: &[f32], sample_rate: f32, target: f32) -> f32 {
    let n = samples.len();
    if n == 0 {
        return 0.0;
    }
    let k = (0.5 + (n as f32 * target) / sample_rate).floor();
    let omega = (2.0 * std::f32::consts::PI * k) / n as f32;
    let coeff = 2.0 * omega.cos();
    let mut sp = 0.0;
    let mut sp2 = 0.0;
    for &x in samples {
        let s = x + coeff * sp - sp2;
        sp2 = sp;
        sp = s;
    }
    (sp2.powi(2) + sp.powi(2) - coeff * sp * sp2).sqrt()
}

// -------------------- PNS --------------------

#[test]
fn pns_triggers_on_white_noise_hf_bands() {
    let mut rng: u32 = 0x1234_5678;
    let mut pcm = Vec::with_capacity(SR as usize * 2);
    for _ in 0..SR {
        rng = rng.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        let v = ((rng >> 16) as i32 & 0xFFFF) as f32 / 65536.0 - 0.5;
        let s = (v * 32767.0) as i16;
        pcm.extend_from_slice(&s.to_le_bytes());
    }
    let aac = encode(pcm.clone(), 1, 96_000);
    assert!(!aac.is_empty());
    let (max_sfb, cbs, sf_index) = parse_first_sce(&aac);
    let swb = oxideav_aac::sfband::SWB_LONG[sf_index as usize];
    let mut hf_bands = 0usize;
    let mut pns_bands = 0usize;
    for sfb in 0..max_sfb as usize {
        let center_bin = (swb[sfb] as usize + swb[sfb + 1] as usize) as f32 * 0.5;
        let center_hz = center_bin * (SR as f32 * 0.5) / 1024.0;
        if center_hz < 4000.0 {
            continue;
        }
        hf_bands += 1;
        if cbs[sfb] == NOISE_HCB {
            pns_bands += 1;
        }
    }
    assert!(hf_bands > 4, "not enough HF bands to test");
    let ratio = pns_bands as f32 / hf_bands as f32;
    eprintln!(
        "pns: {} of {} HF bands are NOISE_HCB ({:.0}%)",
        pns_bands,
        hf_bands,
        ratio * 100.0
    );
    assert!(
        ratio >= 0.75,
        "expected >=75% of HF bands PNS-coded, got {:.0}% ({}/{})",
        ratio * 100.0,
        pns_bands,
        hf_bands
    );
    // Round-trip energy check.
    let decoded = decode_self(&aac, 1);
    let start = 4096.min(decoded.len() / 4);
    let rms_in: f64 = pcm
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]) as f64 / 32768.0)
        .skip(start)
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt();
    let rms_out: f64 = decoded[start..]
        .iter()
        .map(|&s| s as f64 / 32768.0)
        .map(|v| v * v)
        .sum::<f64>()
        .sqrt();
    let rms_ratio = (rms_out / rms_in.max(1e-9)) as f32;
    eprintln!("pns rms ratio out/in = {rms_ratio:.2}");
    assert!(
        rms_ratio > 0.25 && rms_ratio < 4.0,
        "pns energy drift: rms_out/rms_in = {rms_ratio}"
    );
}

// -------------------- Intensity stereo --------------------

#[test]
fn intensity_stereo_triggers_on_scaled_hf_copy() {
    // Cover a broad HF range so several bands are eligible.
    let mut l_pcm = Vec::<f32>::with_capacity(SR as usize);
    for i in 0..SR as usize {
        let t = i as f32 / SR as f32;
        let v = 0.08 * (2.0 * std::f32::consts::PI * 5000.0 * t).sin()
            + 0.08 * (2.0 * std::f32::consts::PI * 7000.0 * t).sin()
            + 0.08 * (2.0 * std::f32::consts::PI * 9000.0 * t).sin()
            + 0.08 * (2.0 * std::f32::consts::PI * 11000.0 * t).sin();
        l_pcm.push(v);
    }
    let mut pcm = Vec::with_capacity(l_pcm.len() * 4);
    for &l in &l_pcm {
        let sl = (l * 32767.0).clamp(-32768.0, 32767.0) as i16;
        let sr_v = l * 0.4;
        let sr_s = (sr_v * 32767.0).clamp(-32768.0, 32767.0) as i16;
        pcm.extend_from_slice(&sl.to_le_bytes());
        pcm.extend_from_slice(&sr_s.to_le_bytes());
    }
    let aac = encode(pcm, 2, 128_000);
    assert!(!aac.is_empty());
    let (_max_sfb, _cbs_l, cbs_r) = parse_first_cpe(&aac);
    let is_count = cbs_r.iter().filter(|&&c| c == INTENSITY_HCB).count();
    eprintln!("IS bands in R: {is_count} (of {})", cbs_r.len());
    assert!(is_count >= 2, "expected >=2 IS bands, got {is_count}");
    // Round-trip: R should have same sign as L most of the time.
    let decoded = decode_self(&aac, 2);
    let mut rms_r = 0.0f64;
    let mut same_sign = 0usize;
    let mut diff_sign = 0usize;
    let mut n = 0usize;
    let start = 4096;
    for pair in decoded.chunks_exact(2).skip(start) {
        let l = pair[0] as f64 / 32768.0;
        let r = pair[1] as f64 / 32768.0;
        rms_r += r * r;
        if l != 0.0 && r != 0.0 {
            if (l > 0.0) == (r > 0.0) {
                same_sign += 1;
            } else {
                diff_sign += 1;
            }
        }
        n += 1;
    }
    rms_r = (rms_r / n as f64).sqrt();
    eprintln!("IS decoded rms_R={rms_r:.3} same_sign={same_sign} diff_sign={diff_sign}");
    assert!(rms_r > 0.02, "IS R channel silent");
    let corr_ratio = same_sign as f32 / (same_sign + diff_sign).max(1) as f32;
    assert!(
        corr_ratio > 0.7,
        "IS R doesn't track L sign (same={same_sign} diff={diff_sign})"
    );
}

// -------------------- Pulse data --------------------

#[test]
fn pulse_data_emitted_on_tonal_peak() {
    // A loud low-frequency spike atop a quiet background. The single
    // coefficient peak is enough to force an expensive book 11 path,
    // so pulse_data should buy us bits.
    let mut pcm = Vec::with_capacity(SR as usize * 2);
    for i in 0..SR {
        let t = i as f32 / SR as f32;
        // 440 Hz at full scale to push the peak quantised coefficient
        // above the pulse outlier threshold (>= 8 units).
        let v = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.95;
        let s = (v * 32767.0) as i16;
        pcm.extend_from_slice(&s.to_le_bytes());
    }
    let aac = encode(pcm, 1, 128_000);
    assert!(!aac.is_empty());
    let frames = iter_adts_frames(&aac);
    let mut saw_pulse = false;
    for (off, len) in &frames {
        let hdr = parse_adts_header(&aac[*off..]).unwrap();
        let sf_index = hdr.sampling_freq_index;
        let payload = &aac[off + hdr.header_length()..off + len];
        let mut br = BitReader::new(payload);
        let id = br.read_u32(3).unwrap();
        if id != ElementType::Sce as u32 {
            continue;
        }
        let _tag = br.read_u32(4).unwrap();
        let _gg = br.read_u32(8).unwrap();
        let info = parse_ics_info(&mut br, sf_index).unwrap();
        let sec = parse_section_data(&mut br, &info).unwrap();
        let _sf = oxideav_aac::ics::parse_scalefactors(&mut br, &info, &sec, _gg as u8).unwrap();
        let pulse_present = br.read_bit().unwrap();
        if pulse_present {
            saw_pulse = true;
            break;
        }
    }
    assert!(saw_pulse, "expected pulse_data on a loud 440 Hz tone");
    // Goertzel round-trip bar >= 50x on the 440 Hz bin.
    let decoded = decode_self(&aac, 1);
    let warm = 4096usize;
    let analysis: Vec<f32> = decoded[warm..]
        .iter()
        .map(|&s| s as f32 / 32768.0)
        .collect();
    let g440 = goertzel(&analysis, SR as f32, 440.0);
    let goff: f32 = [220.0f32, 660.0, 1000.0, 100.0]
        .iter()
        .map(|&f| goertzel(&analysis, SR as f32, f))
        .fold(0.0, f32::max);
    let ratio = g440 / goff.max(1e-9);
    eprintln!("pulse roundtrip goertzel 440={g440} off={goff} ratio={ratio}");
    assert!(
        ratio >= 50.0,
        "pulse round-trip Goertzel ratio {ratio} < 50"
    );
}

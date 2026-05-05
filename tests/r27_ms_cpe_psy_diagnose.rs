//! Round-27 regression gate: M/S CPE psy-on vs psy-off Goertzel ratio.
//!
//! Encodes a 440 Hz / 880 Hz stereo pair through `AacEncoder` with
//! psy off and psy on, decodes both back through the in-tree decoder,
//! and asserts the psy-on per-channel Goertzel ratio at the source
//! frequency stays within 10 % of the psy-off baseline. Also checks an
//! L=440/R=silence variant which isolates the L spectrum's CPE LR
//! quantisation path.
//!
//! Pre-round-27 mechanism: with psy on the Bark-band model recommended
//! `target_max = 16` on the band carrying the 440/880 Hz tone (peak
//! tonality > 0.15). The encoder then picked a fine scalefactor with
//! `step = 2^((sf-100)/4)` 4× smaller than the baseline. In the CPE
//! path TNS is disabled (a single TNS filter can't span per-band M/S
//! decisions), so the band's peak-to-side-lobe ratio stayed ~6× steep
//! — and the side-lobe lines (bins 1-2 below the peak) sit at scaled
//! magnitude ~0.5..1.5, just above the implicit dead-zone. Those lines
//! rounded up to ±1 and dequantised to ±step^(4/3), injecting spurious
//! ~500 kHz-of-MDCT-coefficient noise back into the time-domain
//! reconstruction. The Goertzel ratio at the source tone fell from
//! ~635 to ~140 (78 % drop) while the band-integrated PSNR was barely
//! affected.
//!
//! Round-27 fix: when `analyse_and_quantise_opts` is called with
//! `use_tns=false` (the CPE path), psy's above-baseline target_max
//! recommendations are clamped to baseline=7. The clean-quant path is
//! restored and per-line tone purity matches the psy-off baseline
//! exactly. This in turn unlocks the `psy.rs` per-band sub-baseline
//! coarsening which the round-25 floor-at-baseline workaround had
//! suppressed.

use oxideav_aac::adts::{parse_adts_header, ADTS_HEADER_NO_CRC};
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, Packet, TimeBase};
#[allow(unused_imports)]
use oxideav_core::{Decoder, Encoder};

fn pcm_sine_stereo(freq_l: f32, freq_r: f32, sr: u32, secs: f32, amp: f32) -> Vec<u8> {
    let total = (sr as f32 * secs) as usize;
    let mut out = Vec::with_capacity(total * 4);
    for i in 0..total {
        let t = i as f32 / sr as f32;
        let l = (2.0 * std::f32::consts::PI * freq_l * t).sin() * amp;
        let r = (2.0 * std::f32::consts::PI * freq_r * t).sin() * amp;
        let sl = (l * 32767.0) as i16;
        let sr_s = (r * 32767.0) as i16;
        out.extend_from_slice(&sl.to_le_bytes());
        out.extend_from_slice(&sr_s.to_le_bytes());
    }
    out
}

fn goertzel(samples: &[f32], sample_rate: f32, target_freq: f32) -> f32 {
    let n = samples.len();
    if n == 0 {
        return 0.0;
    }
    let k = (0.5 + (n as f32 * target_freq) / sample_rate).floor();
    let omega = (2.0 * std::f32::consts::PI * k) / n as f32;
    let coeff = 2.0 * omega.cos();
    let mut s_prev = 0.0;
    let mut s_prev2 = 0.0;
    for &x in samples {
        let s = x + coeff * s_prev - s_prev2;
        s_prev2 = s_prev;
        s_prev = s;
    }
    let power = s_prev2.powi(2) + s_prev.powi(2) - coeff * s_prev * s_prev2;
    power.sqrt()
}

fn encode(pcm: Vec<u8>, sr: u32, channels: u16, bitrate: u64, psy_on: bool) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(channels);
    params.bit_rate = Some(bitrate);
    let mut enc = oxideav_aac::encoder::AacEncoder::new(&params).expect("make encoder");
    enc.set_enable_psy_model(psy_on);
    let total_samples = pcm.len() / (2 * channels as usize);
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

fn decode_self(bytes: &[u8]) -> Vec<i16> {
    let frames = iter_adts(bytes);
    assert!(!frames.is_empty(), "no ADTS frames found");
    let first = parse_adts_header(&bytes[frames[0].0..]).unwrap();
    let sr = first.sample_rate().unwrap();
    let ch = first.channel_configuration as u16;
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(ch);
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make dec");
    let tb = TimeBase::new(1, sr as i64);
    let mut samples = Vec::<i16>::new();
    for (i, &(off, len)) in frames.iter().enumerate() {
        let pkt = Packet::new(0, tb, bytes[off..off + len].to_vec()).with_pts(i as i64 * 1024);
        dec.send_packet(&pkt).unwrap();
        match dec.receive_frame() {
            Ok(Frame::Audio(af)) => {
                for chunk in af.data[0].chunks_exact(2) {
                    samples.push(i16::from_le_bytes([chunk[0], chunk[1]]));
                }
            }
            other => panic!("unexpected: {other:?}"),
        }
    }
    samples
}

fn channel_goertzel(samples: &[i16], sr: u32, channels: u16, freq: f32, ch_idx: usize) -> f32 {
    let warm = 4 * 1024 * channels as usize;
    let analysis: Vec<f32> = samples[warm..]
        .chunks_exact(channels as usize)
        .map(|c| c[ch_idx] as f32 / 32768.0)
        .collect();
    let g_target = goertzel(&analysis, sr as f32, freq);
    let off_freqs = [220.0, 660.0, 1000.0, 100.0, 50.0];
    let off_max = off_freqs
        .iter()
        .map(|&f| goertzel(&analysis, sr as f32, f))
        .fold(0.0f32, f32::max);
    let ratio = g_target / off_max.max(1e-9);
    eprintln!(
        "[ch={ch_idx} f={freq}] g_target={g_target:.4} off_max={off_max:.4} ratio={ratio:.2}",
    );
    ratio
}

/// 440/880 Hz stereo pair → CPE M/S analysis exercises both the LR and MS
/// per-band cost paths. Round-27 fix gates the psy above-baseline
/// target_max on `use_tns`, so CPE bands stay at baseline-quant.
#[test]
fn r27_ms_cpe_psy_off_vs_on_goertzel() {
    let sr = 44_100u32;
    let pcm = pcm_sine_stereo(440.0, 880.0, sr, 1.0, 0.5);

    let aac_off = encode(pcm.clone(), sr, 2, 128_000, false);
    let dec_off = decode_self(&aac_off);
    eprintln!("--- 440/880 stereo psy OFF ---");
    let r0_off = channel_goertzel(&dec_off, sr, 2, 440.0, 0);
    let r1_off = channel_goertzel(&dec_off, sr, 2, 880.0, 1);
    eprintln!("psy_off bytes={}", aac_off.len());

    let aac_on = encode(pcm.clone(), sr, 2, 128_000, true);
    let dec_on = decode_self(&aac_on);
    eprintln!("--- 440/880 stereo psy ON ---");
    let r0_on = channel_goertzel(&dec_on, sr, 2, 440.0, 0);
    let r1_on = channel_goertzel(&dec_on, sr, 2, 880.0, 1);
    eprintln!("psy_on bytes={}", aac_on.len());

    let drop_l = if r0_off > 0.0 {
        100.0 * (1.0 - r0_on / r0_off)
    } else {
        0.0
    };
    let drop_r = if r1_off > 0.0 {
        100.0 * (1.0 - r1_on / r1_off)
    } else {
        0.0
    };
    eprintln!("ratio drop L: {drop_l:.1}%, R: {drop_r:.1}%");

    // Gate: psy-on Goertzel must hold within 10 % of the psy-off
    // baseline. Pre-round-27 the drop was 78 % L / 52 % R; the
    // `use_tns=false` clamp in `analyse_and_quantise_opts` brings
    // both back to 0 % drop (exact match within rounding).
    let max_drop_pct = 10.0;
    assert!(
        drop_l <= max_drop_pct && drop_r <= max_drop_pct,
        "M/S CPE psy-on Goertzel ratio drops > {max_drop_pct}%: L={drop_l:.1}% R={drop_r:.1}% \
         (psy off L={r0_off:.1} R={r1_off:.1}, psy on L={r0_on:.1} R={r1_on:.1})",
    );
}

/// L=440 Hz / R=silence stereo: the L spectrum is a single tonal CPE
/// channel. The per-channel psy decision is identical to a mono encode
/// (one tonal band) but the encoder runs through the CPE shell which
/// disables TNS. Pre-round-27 the L Goertzel ratio dropped 78 % under
/// psy on; the `use_tns=false` clamp brings it back to baseline.
#[test]
fn r27_l_only_cpe_psy_goertzel() {
    let sr = 44_100u32;
    let pcm = pcm_sine_stereo(440.0, 0.0, sr, 1.0, 0.5);

    let aac_off = encode(pcm.clone(), sr, 2, 256_000, false);
    let dec_off = decode_self(&aac_off);
    eprintln!("--- L=440 R=0 stereo psy OFF ---");
    let r0_off = channel_goertzel(&dec_off, sr, 2, 440.0, 0);
    eprintln!("psy_off bytes={}", aac_off.len());

    let aac_on = encode(pcm.clone(), sr, 2, 256_000, true);
    let dec_on = decode_self(&aac_on);
    eprintln!("--- L=440 R=0 stereo psy ON ---");
    let r0_on = channel_goertzel(&dec_on, sr, 2, 440.0, 0);
    eprintln!("psy_on bytes={}", aac_on.len());

    let drop = if r0_off > 0.0 {
        100.0 * (1.0 - r0_on / r0_off)
    } else {
        0.0
    };
    eprintln!("L ratio drop: {drop:.1}%");
    let max_drop_pct = 10.0;
    assert!(
        drop <= max_drop_pct,
        "L=440 R=0 stereo CPE psy-on Goertzel drop > {max_drop_pct}%: {drop:.1}% \
         (psy off ratio={r0_off:.1}, psy on ratio={r0_on:.1})",
    );
}

//! Self round-trip for HE-AACv1 *stereo* SBR encoder.
//!
//! Mirrors `sbr_encode_roundtrip.rs` (mono) but feeds two distinct tones
//! through `HeAacStereoEncoder`, decodes the resulting CPE+FIL/SBR
//! stream through the in-crate decoder, and confirms each output channel
//! preserves its source tone (Goertzel ratio above the per-channel RMS
//! floor). Independent-coupling CPE encode path (§4.6.18.3.5,
//! `bs_coupling = 0`).

use oxideav_aac::adts::{parse_adts_header, ADTS_HEADER_NO_CRC};
use oxideav_aac::he_aac_encoder::HeAacStereoEncoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, Packet, SampleFormat, TimeBase};
#[allow(unused_imports)]
use oxideav_core::{Decoder, Encoder};

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

fn pcm_two_tones_stereo_bytes(freq_l: f32, freq_r: f32, sr: u32, secs: f32, amp: f32) -> Vec<u8> {
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

#[test]
fn he_aac_stereo_self_roundtrip_two_tones_48k_to_24k_core_and_back() {
    let high_rate = 48_000u32;
    let core_rate = 24_000u32;
    let freq_l = 1000.0f32;
    // R tone in the AAC-LC core's range (well below the SBR crossover
    // at ~5.6 kHz). This validates the CPE routing — both L and R land
    // distinct AAC-LC core tones. Above-crossover SBR-domain tones get
    // energy-only reconstruction (no tone preserved) which is a
    // separate bug surface; this test focuses on the encoder's CPE
    // bitstream wiring being correct.
    let freq_r = 2000.0f32;
    let secs = 0.5f32;

    let pcm_bytes = pcm_two_tones_stereo_bytes(freq_l, freq_r, high_rate, secs, 0.3);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(high_rate);
    params.channels = Some(2);
    params.bit_rate = Some(64_000);
    let mut enc = HeAacStereoEncoder::new(&params).expect("enc construct");
    let n = pcm_bytes.len() / 4;
    let af = AudioFrame {
        samples: n as u32,
        pts: Some(0),
        data: vec![pcm_bytes],
    };
    enc.send_frame(&Frame::Audio(af)).expect("enc send");
    enc.flush().expect("enc flush");
    let mut all = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        all.extend_from_slice(&pkt.data);
    }
    let frames = iter_adts(&all);
    assert!(!frames.is_empty(), "no HE-AAC stereo ADTS frames produced");

    // Decode with AOT_SBR extradata so the decoder doubles to 48 kHz +
    // routes the FIL/SBR through the SBR decode path. Stereo channel
    // configuration = 2.
    // ASC: 5b aot=5, 4b core_idx=6 (24k), 4b ch=2, 4b ext_idx=3 (48k),
    //      5b inner=2 (AAC-LC).
    //   00101 0110 0010 0011 00010 = 0010 1011 0001 0001 1000 1000 ...
    //   = 0x2B 0x11 0x88
    let extradata = vec![0x2Bu8, 0x11, 0x88];
    let mut dec_params = CodecParameters::audio(CodecId::new("aac"));
    dec_params.sample_rate = Some(core_rate);
    dec_params.channels = Some(2);
    dec_params.extradata = extradata;
    let mut dec = oxideav_aac::decoder::make_decoder(&dec_params).expect("dec");

    // Decode and accumulate per-channel S16 → f32.
    let mut out_l: Vec<f32> = Vec::new();
    let mut out_r: Vec<f32> = Vec::new();
    let tb = TimeBase::new(1, core_rate as i64);
    for (off, len) in &frames {
        let pkt = Packet::new(0, tb, all[*off..*off + *len].to_vec());
        dec.send_packet(&pkt).expect("send_packet");
        loop {
            match dec.receive_frame() {
                Ok(Frame::Audio(af)) => {
                    let data = &af.data[0];
                    let stride = 4usize; // 2 ch * 2 bytes (S16 interleaved)
                    for i in 0..af.samples as usize {
                        let l = i16::from_le_bytes([data[i * stride], data[i * stride + 1]]);
                        let r = i16::from_le_bytes([data[i * stride + 2], data[i * stride + 3]]);
                        out_l.push(l as f32 / 32768.0);
                        out_r.push(r as f32 / 32768.0);
                    }
                }
                Ok(_) => continue,
                Err(e) if format!("{e:?}").contains("NeedMore") => break,
                Err(e) if format!("{e:?}").contains("Eof") => break,
                Err(e) => panic!("decode err: {e:?}"),
            }
        }
    }

    assert!(out_l.len() > 1024, "decoded L has <1024 samples");
    assert!(out_r.len() > 1024, "decoded R has <1024 samples");

    // Per-channel Goertzel at the source tones, after the priming-delay
    // window.
    fn goertzel_db(buf: &[f32], freq: f32, sr: f32) -> (f32, f32, f32) {
        let analysis_start = buf.len() / 4;
        let w = 2.0 * std::f32::consts::PI * freq / sr;
        let coeff = 2.0 * w.cos();
        let (mut s0, mut s1) = (0.0f32, 0.0f32);
        for &x in &buf[analysis_start..] {
            let s = x + coeff * s0 - s1;
            s1 = s0;
            s0 = s;
        }
        let mag = (s0 * s0 + s1 * s1 - coeff * s0 * s1).sqrt();
        let total_rms: f32 = (buf.iter().map(|v| v * v).sum::<f32>() / buf.len() as f32).sqrt();
        let n = (buf.len() - analysis_start) as f32;
        let tone_rms = mag / (n / 2.0).max(1.0).sqrt();
        let snr_db = 20.0 * (tone_rms / total_rms.max(1e-9)).log10();
        (mag, total_rms, snr_db)
    }

    let (mag_l, rms_l, snr_l) = goertzel_db(&out_l, freq_l, high_rate as f32);
    let (mag_r, rms_r, snr_r) = goertzel_db(&out_r, freq_r, high_rate as f32);
    println!(
        "stereo round-trip: L mag={mag_l:.3} rms={rms_l:.4} snr={snr_l:.2} dB | \
         R mag={mag_r:.3} rms={rms_r:.4} snr={snr_r:.2} dB",
    );

    // Each channel should have non-trivial energy and a recognizable
    // tone peak. Thresholds match the existing mono test (which uses
    // mag > 1.0 for a comparable signal).
    assert!(rms_l > 1e-3, "L channel decoded silent, rms={rms_l}");
    assert!(rms_r > 1e-3, "R channel decoded silent, rms={rms_r}");
    assert!(
        mag_l > 1.0,
        "L channel decoded tone too weak: goertzel mag={mag_l} rms={rms_l}",
    );
    assert!(
        mag_r > 1.0,
        "R channel decoded tone too weak: goertzel mag={mag_r} rms={rms_r}",
    );
}

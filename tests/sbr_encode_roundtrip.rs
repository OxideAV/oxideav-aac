//! Self round-trip for HE-AACv1 mono SBR encoder.
//!
//! Produce a 1 kHz tone at 48 kHz input, encode via
//! `HeAacMonoEncoder` (24 kHz AAC-LC core + SBR), decode through the
//! in-crate decoder with an AOT_SBR extradata, and confirm the decoded
//! output contains energy at the source tone frequency.

use oxideav_aac::adts::{parse_adts_header, ADTS_HEADER_NO_CRC};
use oxideav_aac::he_aac_encoder::HeAacMonoEncoder;
#[allow(unused_imports)]
use oxideav_codec::{Decoder, Encoder};
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, Packet, SampleFormat, TimeBase};

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

fn pcm_sine_mono_bytes(freq: f32, sr: u32, secs: f32, amp: f32) -> Vec<u8> {
    let total = (sr as f32 * secs) as usize;
    let mut out = Vec::with_capacity(total * 2);
    for i in 0..total {
        let t = i as f32 / sr as f32;
        let v = (2.0 * std::f32::consts::PI * freq * t).sin() * amp;
        let s = (v * 32767.0) as i16;
        out.extend_from_slice(&s.to_le_bytes());
    }
    out
}

#[test]
fn he_aac_self_roundtrip_1khz_48k_to_24k_core_and_back() {
    let high_rate = 48_000u32;
    let core_rate = 24_000u32;
    let freq = 1000.0f32;
    let secs = 0.5f32;

    let pcm_bytes = pcm_sine_mono_bytes(freq, high_rate, secs, 0.3);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(high_rate);
    params.channels = Some(1);
    params.bit_rate = Some(32_000);
    let mut enc = HeAacMonoEncoder::new(&params).expect("enc construct");
    let n = pcm_bytes.len() / 2;
    let af = AudioFrame {
        format: SampleFormat::S16,
        channels: 1,
        sample_rate: high_rate,
        samples: n as u32,
        pts: Some(0),
        time_base: TimeBase::new(1, high_rate as i64),
        data: vec![pcm_bytes],
    };
    enc.send_frame(&Frame::Audio(af)).expect("enc send");
    enc.flush().expect("enc flush");
    let mut all = Vec::new();
    while let Ok(pkt) = enc.receive_packet() {
        all.extend_from_slice(&pkt.data);
    }
    let frames = iter_adts(&all);
    assert!(!frames.is_empty(), "no HE-AAC ADTS frames produced");

    // Decode with AOT_SBR extradata so the decoder routes output through
    // the SBR doubler.
    // ASC: 5b aot=5, 4b core_idx=6 (24k), 4b ch=1, 4b ext_idx=3 (48k),
    //      5b inner=2 (AAC-LC) — as in the existing SBR smoke test.
    let extradata = vec![0x2Bu8, 0x09, 0x88];
    let mut dec_params = CodecParameters::audio(CodecId::new("aac"));
    dec_params.sample_rate = Some(core_rate);
    dec_params.channels = Some(1);
    dec_params.extradata = extradata;
    let mut dec = oxideav_aac::decoder::make_decoder(&dec_params).expect("dec");

    // Decode frame-by-frame and concatenate S16 output.
    let mut out_pcm: Vec<f32> = Vec::new();
    let tb = TimeBase::new(1, core_rate as i64);
    for (off, len) in &frames {
        let pkt = Packet::new(0, tb, all[*off..*off + *len].to_vec());
        dec.send_packet(&pkt).expect("send_packet");
        loop {
            match dec.receive_frame() {
                Ok(Frame::Audio(af)) => {
                    // S16 interleaved, but this is mono — accumulate.
                    assert_eq!(af.channels, 1);
                    assert_eq!(af.sample_rate, high_rate);
                    let data = &af.data[0];
                    let per = 2usize;
                    for i in 0..af.samples as usize {
                        let s = i16::from_le_bytes([data[i * per], data[i * per + 1]]);
                        out_pcm.push(s as f32 / 32768.0);
                    }
                }
                Ok(_) => continue,
                Err(e) if format!("{e:?}").contains("NeedMore") => break,
                Err(e) if format!("{e:?}").contains("Eof") => break,
                Err(e) => panic!("decode err: {e:?}"),
            }
        }
    }

    assert!(out_pcm.len() > 512, "decode produced <512 samples");

    // Goertzel at the source tone frequency on the decoded 48 kHz output.
    let w = 2.0 * std::f32::consts::PI * freq / high_rate as f32;
    let (mut s0, mut s1) = (0.0f32, 0.0f32);
    let coeff = 2.0 * w.cos();
    for &x in &out_pcm[out_pcm.len() / 4..] {
        let s = x + coeff * s0 - s1;
        s1 = s0;
        s0 = s;
    }
    let mag = (s0 * s0 + s1 * s1 - coeff * s0 * s1).sqrt();

    // Total energy for ratio.
    let total_rms: f32 =
        (out_pcm.iter().map(|v| v * v).sum::<f32>() / out_pcm.len() as f32).sqrt();
    assert!(total_rms > 1e-3, "decoded output is silent, rms={total_rms}");
    // Tone energy concentrated at the source frequency — we expect the
    // Goertzel magnitude to be far above the per-sample RMS times a
    // constant (the exact threshold depends on the window size).
    assert!(
        mag > 1.0,
        "decoded tone is too weak: goertzel mag={mag} rms={total_rms}",
    );
}

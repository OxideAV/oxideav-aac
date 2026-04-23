//! End-to-end SBR smoke tests.
//!
//! These exercise the SBR decode path with a mix of:
//!   - direct `decode_sbr_frame` calls (no bitstream)
//!   - an ADTS-framed AAC-LC stream whose extradata declares SBR so the
//!     decoder routes output through the SBR doubler.

use oxideav_aac::sbr::bitstream::{FrameClass, SbrChannelData, SbrHeader};
use oxideav_aac::sbr::decode::{decode_sbr_frame, SbrChannelState};
use oxideav_aac::sbr::freq::FreqTables;
use oxideav_aac::sbr::hf_gen::build_patches;

use oxideav_aac::adts::{parse_adts_header, ADTS_HEADER_NO_CRC};
#[allow(unused_imports)]
use oxideav_codec::{Decoder, Encoder};
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, Packet, SampleFormat, TimeBase};

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

fn encode_mono(pcm_bytes: Vec<u8>, sr: u32, bitrate: u64) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr);
    params.channels = Some(1);
    params.bit_rate = Some(bitrate);
    let mut enc = oxideav_aac::encoder::make_encoder(&params).expect("make encoder");
    let total_samples = pcm_bytes.len() / 2;
    let frame = Frame::Audio(AudioFrame {
        format: SampleFormat::S16,
        channels: 1,
        sample_rate: sr,
        samples: total_samples as u32,
        pts: Some(0),
        time_base: TimeBase::new(1, sr as i64),
        data: vec![pcm_bytes],
    });
    enc.send_frame(&frame).expect("send_frame");
    enc.flush().expect("flush");
    let mut out = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        out.extend_from_slice(&p.data);
    }
    out
}

/// Confirm that `decode_sbr_frame` produces 2048 output samples given
/// 1024 input samples.
#[test]
fn sbr_decode_frame_doubles_length() {
    let sr_core = 24_000;
    let sr_sbr = sr_core * 2;
    let mut state = SbrChannelState::new();
    state.header = SbrHeader::defaults();
    state.header.bs_start_freq = 5;
    state.header.bs_stop_freq = 8;
    state.header.bs_xover_band = 0;
    state.header.bs_freq_scale = 2;
    state.header.bs_alter_scale = true;
    state.header.bs_noise_bands = 2;
    state.header_seen = true;
    state.freq = Some(
        FreqTables::build(
            sr_sbr,
            state.header.bs_start_freq,
            state.header.bs_stop_freq,
            state.header.bs_xover_band,
            state.header.bs_freq_scale,
            state.header.bs_alter_scale,
            state.header.bs_noise_bands,
        )
        .expect("freq tables"),
    );
    state.patches = Some(
        build_patches(state.freq.as_ref().unwrap(), sr_sbr).expect("patches"),
    );
    let mut data = SbrChannelData::default();
    data.bs_amp_res = 1;
    data.bs_num_env = 1;
    data.bs_num_noise = 1;
    data.frame_class = FrameClass::FixFix;
    for b in 0..10 {
        data.env_sf[0][b] = 10;
    }
    let pcm: Vec<f32> = (0..1024)
        .map(|n| (2.0 * std::f32::consts::PI * 1000.0 * n as f32 / sr_core as f32).sin() * 0.5)
        .collect();
    let mut out = vec![0.0f32; 2048];
    decode_sbr_frame(&pcm, &data, &mut state, &mut out).expect("sbr decode");
    assert_eq!(out.len(), 2048);
    let rms: f32 = (out.iter().map(|v| v * v).sum::<f32>() / out.len() as f32).sqrt();
    assert!(rms > 1e-4, "SBR output is silent: rms = {rms}");
}

/// Decoder accepts an AOT_PS extradata — previously this was rejected
/// outright with `Error::unsupported`. Confirm mono AAC-LC → PS
/// extradata path doesn't panic and emits stereo output.
#[test]
fn decoder_with_ps_extradata_upmixes_to_stereo() {
    let sr_core = 24_000u32;
    let pcm = pcm_sine_mono_bytes(440.0, sr_core, 0.2, 0.2);
    let aac = encode_mono(pcm, sr_core, 32_000);
    let frames = iter_adts(&aac);
    assert!(!frames.is_empty());
    // ASC: AOT_PS (29) needs 6-bit extended AOT. AOT = 31 then +6 bit value 29-32
    // but PS is value 29 which fits directly in 5 bits. Layout:
    //   5 bits aot=29 (PS = 11101)
    //   4 bits core_idx=6 (24 kHz)
    //   4 bits channels=1
    //   4 bits ext_idx=3 (48 kHz)
    //   5 bits inner_aot=2
    // = 22 bits.
    //   11101 0110 0001 0011 00010 -> 1110101100001001100010 (22 bits)
    //   padded to 24: 11101011 00001001 10001000
    //       = 0xEB, 0x09, 0x88
    let extradata = vec![0xEBu8, 0x09, 0x88];
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr_core);
    params.channels = Some(1);
    params.extradata = extradata;
    // Should not error out (previously returned Unsupported).
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make dec");
    let tb = TimeBase::new(1, sr_core as i64);
    let &(off, len) = &frames[0];
    let pkt = Packet::new(0, tb, aac[off..off + len].to_vec());
    dec.send_packet(&pkt).expect("send");
    let frame = dec.receive_frame().expect("receive");
    match frame {
        Frame::Audio(af) => {
            // PS upmixes mono to stereo; sample rate is still doubled.
            assert_eq!(af.channels, 2, "PS did not upmix to stereo");
            assert_eq!(af.sample_rate, sr_core * 2);
        }
        other => panic!("expected audio frame, got {other:?}"),
    }
}

/// Full pipeline: AAC-LC encode; decode through a decoder whose extradata
/// declares SBR; confirm the output sample rate is doubled.
#[test]
fn decoder_with_sbr_extradata_doubles_rate() {
    let sr_core = 24_000u32;
    let pcm = pcm_sine_mono_bytes(440.0, sr_core, 0.2, 0.2);
    let aac = encode_mono(pcm, sr_core, 32_000);
    let frames = iter_adts(&aac);
    assert!(!frames.is_empty(), "no ADTS frames encoded");

    // Build ASC that declares AOT_SBR wrapping AOT_AAC_LC. Layout:
    //   5 bits aot=5 (SBR)
    //   4 bits core_idx=6 (24 kHz)
    //   4 bits channels=1
    //   4 bits ext_idx=3 (48 kHz)
    //   5 bits inner_aot=2 (AAC-LC)
    // = 22 bits = 3 bytes padded.
    // Bit layout: aot|core_idx|chan|ext_idx|inner_aot
    //            00101| 0110 | 0001 | 0011  | 00010
    // Concatenated = 00101_0110_0001_0011_00010 = 22 bits
    // Padded to 24: 0010101100001001100010 00
    //   byte 0: 00101011 = 0x2B
    //   byte 1: 00001001 = 0x09
    //   byte 2: 10001000 = 0x88
    let extradata = vec![0x2Bu8, 0x09, 0x88];
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(sr_core);
    params.channels = Some(1);
    params.extradata = extradata;
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make dec");

    let tb = TimeBase::new(1, sr_core as i64);
    let &(off, len) = &frames[0];
    let pkt = Packet::new(0, tb, aac[off..off + len].to_vec());
    dec.send_packet(&pkt).expect("send");
    let frame = dec.receive_frame().expect("receive");
    match frame {
        Frame::Audio(af) => {
            assert_eq!(
                af.sample_rate,
                sr_core * 2,
                "expected doubled output rate, got {}",
                af.sample_rate
            );
            assert_eq!(af.samples as usize, 2048, "expected 2048 doubled samples");
            assert_eq!(af.channels, 1);
        }
        other => panic!("expected audio frame, got {other:?}"),
    }
}

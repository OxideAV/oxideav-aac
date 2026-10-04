//! Out-of-band-configured AAC: the MP4 / Matroska carriage, where every
//! packet is one bare access unit (ISO/IEC 14496-14 §3.1.2) and the
//! §1.6.2.1 `AudioSpecificConfig` rides in `CodecParameters::extradata`.
//!
//! Regression for "AAC from MP4 / MKV fails to decode": the registry
//! decoder used to require an ADTS or LOAS syncword on every packet.
//! These tests encode with the in-crate encoder (which advertises its
//! ASC in `output_params().extradata`), strip the ADTS headers the way a
//! container muxer does, and require the raw-AU decode to be
//! sample-identical to the ADTS decode of the same frames — and close to
//! the source signal.

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::adts_container::AdtsDemuxer;
use oxideav_aac::asc::AudioSpecificConfig;
use oxideav_aac::codec_decoder::make_decoder;
use oxideav_aac::codec_encoder::{make_encoder, make_he_aac_encoder, make_he_aac_v2_encoder};
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Decoder, Demuxer, Encoder, Frame, Packet, TimeBase,
};

/// A two-tone + chirp test signal, interleaved S16.
fn signal(rate: u32, channels: u16, samples: usize) -> Vec<i16> {
    let mut out = Vec::with_capacity(samples * channels as usize);
    for i in 0..samples {
        let t = i as f64 / f64::from(rate);
        for c in 0..channels {
            let f0 = 300.0 + 150.0 * f64::from(c);
            let v = 0.35 * (std::f64::consts::TAU * (f0 + 400.0 * t) * t).sin()
                + 0.15 * (std::f64::consts::TAU * 1234.5 * t).sin();
            out.push((v * 32767.0) as i16);
        }
    }
    out
}

fn to_bytes(pcm: &[i16]) -> Vec<u8> {
    pcm.iter().flat_map(|s| s.to_le_bytes()).collect()
}

/// Encode `pcm` and return (advertised extradata, ADTS packets).
fn encode(mut enc: Box<dyn Encoder>, pcm: &[i16], channels: u16) -> (Vec<u8>, Vec<Packet>) {
    let extradata = enc.output_params().extradata.clone();
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: (pcm.len() / channels as usize) as u32,
        pts: Some(0),
        data: vec![to_bytes(pcm)],
    }))
    .unwrap();
    enc.flush().unwrap();
    let mut pkts = Vec::new();
    while let Ok(p) = enc.receive_packet() {
        pkts.push(p);
    }
    (extradata, pkts)
}

/// Strip the ADTS header off every packet (what the MP4 / MKV muxers do).
fn strip(pkts: &[Packet]) -> Vec<Packet> {
    pkts.iter()
        .map(|p| {
            let (h, off) = AdtsHeader::parse(&p.data).unwrap();
            assert_eq!(h.aac_frame_length as usize, p.data.len());
            let mut q = p.clone();
            q.data = p.data[off..].to_vec();
            q
        })
        .collect()
}

fn decode_all(dec: &mut dyn Decoder, pkts: &[Packet]) -> Vec<i16> {
    let mut out = Vec::new();
    for p in pkts {
        dec.send_packet(p).unwrap();
        while let Ok(Frame::Audio(a)) = dec.receive_frame() {
            out.extend(
                a.data[0]
                    .chunks_exact(2)
                    .map(|b| i16::from_le_bytes([b[0], b[1]])),
            );
        }
    }
    out
}

fn params(rate: u32, channels: u16, extradata: Vec<u8>) -> CodecParameters {
    let mut p = CodecParameters::audio(CodecId::new("aac"));
    p.sample_rate = Some(rate);
    p.channels = Some(channels);
    p.extradata = extradata;
    p
}

/// Best SNR (dB) of `dec` against `src` over integer lags `0..max_lag`
/// (the decode carries the encoder delay), first channel only.
fn snr_db(src: &[i16], dec: &[i16], channels: usize, max_lag: usize) -> (f64, usize) {
    let n_src = src.len() / channels;
    let n_dec = dec.len() / channels;
    let lo = n_src / 4;
    let hi = n_src * 3 / 4;
    let mut best = (f64::INFINITY, 0usize, 1.0f64);
    for lag in 0..max_lag {
        if hi + lag >= n_dec {
            break;
        }
        let (mut err, mut sig) = (0.0f64, 0.0f64);
        for t in (lo..hi).step_by(2) {
            let a = f64::from(src[t * channels]);
            let b = f64::from(dec[(t + lag) * channels]);
            err += (a - b) * (a - b);
            sig += a * a;
        }
        if err < best.0 {
            best = (err, lag, sig);
        }
    }
    (10.0 * (best.2 / best.0.max(1.0)).log10(), best.1)
}

#[test]
fn lc_encoder_advertises_its_asc() {
    let enc = make_encoder(&params(48_000, 2, Vec::new())).unwrap();
    let (asc, _) = AudioSpecificConfig::parse(&enc.output_params().extradata).unwrap();
    assert_eq!(asc.aot, 2);
    assert_eq!(asc.sample_rate, 48_000);
    assert_eq!(asc.channel_configuration, 2);
}

#[test]
fn lc_raw_access_units_match_adts_decode() {
    for &(rate, ch) in &[(44_100u32, 2u16), (48_000, 1), (32_000, 6)] {
        let pcm = signal(rate, ch, rate as usize / 2);
        let (extradata, adts) = encode(
            make_encoder(&params(rate, ch, Vec::new())).unwrap(),
            &pcm,
            ch,
        );
        assert!(!extradata.is_empty());
        let raw = strip(&adts);

        let mut via_adts = make_decoder(&params(rate, ch, Vec::new())).unwrap();
        let ref_pcm = decode_all(&mut *via_adts, &adts);
        let mut via_raw = make_decoder(&params(rate, ch, extradata)).unwrap();
        let raw_pcm = decode_all(&mut *via_raw, &raw);
        assert_eq!(
            raw_pcm, ref_pcm,
            "{rate} Hz / {ch} ch raw-AU decode diverged"
        );

        let (snr, lag) = snr_db(&pcm, &raw_pcm, ch as usize, 3000);
        assert_eq!(lag, 1024, "AAC-LC encoder delay is one frame");
        assert!(snr > 30.0, "{rate} Hz / {ch} ch SNR {snr:.1} dB");
    }
}

#[test]
fn raw_access_units_without_extradata_use_the_stream_geometry() {
    // Legacy Matroska `A_AAC/MPEG4/LC` / WAVEFORMATEX carriage: no ASC,
    // just the track's sample rate and channel count.
    let pcm = signal(22_050, 2, 11_025);
    let (_, adts) = encode(
        make_encoder(&params(22_050, 2, Vec::new())).unwrap(),
        &pcm,
        2,
    );
    let raw = strip(&adts);
    let mut ref_dec = make_decoder(&params(22_050, 2, Vec::new())).unwrap();
    let ref_pcm = decode_all(&mut *ref_dec, &adts);
    let mut dec = make_decoder(&params(22_050, 2, Vec::new())).unwrap();
    assert_eq!(decode_all(&mut *dec, &raw), ref_pcm);

    // Without any geometry the decoder reports a clear error.
    let mut bare = CodecParameters::audio(CodecId::new("aac"));
    bare.sample_rate = None;
    bare.channels = None;
    let mut dec = make_decoder(&bare).unwrap();
    let err = dec.send_packet(&raw[1]).unwrap_err().to_string();
    assert!(err.contains("AudioSpecificConfig"), "{err}");
}

#[test]
fn he_aac_raw_access_units_decode_at_the_sbr_rate() {
    let pcm = signal(44_100, 2, 22_050);
    let (extradata, adts) = encode(
        make_he_aac_encoder(&params(44_100, 2, Vec::new())).unwrap(),
        &pcm,
        2,
    );
    let (asc, _) = AudioSpecificConfig::parse(&extradata).unwrap();
    assert!(asc.sbr_present);
    let raw = strip(&adts);
    let mut ref_dec = make_decoder(&params(44_100, 2, Vec::new())).unwrap();
    let ref_pcm = decode_all(&mut *ref_dec, &adts);
    // The container advertises the *core* rate in its sample entry; the
    // ASC must override it.
    let mut dec = make_decoder(&params(22_050, 2, extradata)).unwrap();
    let raw_pcm = decode_all(&mut *dec, &raw);
    assert_eq!(raw_pcm, ref_pcm);
    assert_eq!(raw_pcm.len() / 2, raw.len() * 2048);
}

#[test]
fn he_aac_v2_raw_access_units_decode_to_stereo() {
    let pcm = signal(48_000, 2, 24_000);
    let (extradata, adts) = encode(
        make_he_aac_v2_encoder(&params(48_000, 2, Vec::new())).unwrap(),
        &pcm,
        2,
    );
    let raw = strip(&adts);
    let mut ref_dec = make_decoder(&params(48_000, 2, Vec::new())).unwrap();
    let ref_pcm = decode_all(&mut *ref_dec, &adts);
    // An MP4 sample entry for HE-AAC v2 frequently says 1 channel.
    let mut dec = make_decoder(&params(24_000, 1, extradata)).unwrap();
    let raw_pcm = decode_all(&mut *dec, &raw);
    assert_eq!(raw_pcm, ref_pcm);
    assert!(!raw_pcm.is_empty());
}

#[test]
fn adts_packet_with_extradata_still_decodes() {
    // A Matroska track that carries ADTS frames *and* a CodecPrivate.
    let pcm = signal(44_100, 2, 8_192);
    let (extradata, adts) = encode(
        make_encoder(&params(44_100, 2, Vec::new())).unwrap(),
        &pcm,
        2,
    );
    let mut a = make_decoder(&params(44_100, 2, Vec::new())).unwrap();
    let mut b = make_decoder(&params(44_100, 2, extradata)).unwrap();
    assert_eq!(decode_all(&mut *b, &adts), decode_all(&mut *a, &adts));
}

#[test]
fn adts_container_round_trip() {
    let pcm = signal(44_100, 2, 44_100);
    let (_, adts) = encode(
        make_encoder(&params(44_100, 2, Vec::new())).unwrap(),
        &pcm,
        2,
    );
    let file: Vec<u8> = adts.iter().flat_map(|p| p.data.clone()).collect();
    let mut demux = AdtsDemuxer::open(Box::new(std::io::Cursor::new(file))).unwrap();
    let stream = demux.streams()[0].clone();
    assert_eq!(stream.time_base, TimeBase::new(1, 44_100));
    assert_eq!(stream.duration, Some(adts.len() as i64 * 1024));
    let mut pkts = Vec::new();
    while let Ok(p) = demux.next_packet() {
        pkts.push(p);
    }
    assert_eq!(pkts.len(), adts.len());
    let mut dec = make_decoder(&stream.params).unwrap();
    let out = decode_all(&mut *dec, &pkts);
    let (snr, _) = snr_db(&pcm, &out, 2, 3000);
    assert!(snr > 30.0, "ADTS container round trip SNR {snr:.1} dB");
}

#[test]
fn implicit_sbr_follows_the_container_declared_rate() {
    // Implicitly signalled HE-AAC: the ASC is plain AAC-LC at the core
    // rate; SBR is only discovered in the payload.
    let pcm = signal(44_100, 2, 22_050);
    let (_, adts) = encode(
        make_he_aac_encoder(&params(44_100, 2, Vec::new())).unwrap(),
        &pcm,
        2,
    );
    let raw = strip(&adts);
    let lc_asc = oxideav_aac::asc_writer::aac_lc_asc(22_050, 2);

    // Container declares the SBR output rate (the usual MP4 sample
    // entry): full dual-rate output, 2048 samples per access unit.
    let mut dec = make_decoder(&params(44_100, 2, lc_asc.clone())).unwrap();
    assert_eq!(decode_all(&mut *dec, &raw).len() / 2, raw.len() * 2048);

    // Container declares only the core rate (a Matroska track written
    // without OutputSamplingFrequency knowledge): a consumer configured
    // from that rate must receive that rate — downsampled SBR output,
    // 1024 samples per access unit.
    let mut dec = make_decoder(&params(22_050, 2, lc_asc)).unwrap();
    let out = decode_all(&mut *dec, &raw);
    assert_eq!(out.len() / 2, raw.len() * 1024);
    let (snr, _) = {
        // Compare against the source decimated 2:1: the test tones sit
        // far below the 11.025 kHz core Nyquist, so plain decimation is
        // a faithful reference for the band-limited output.
        let dec2: Vec<i16> = pcm
            .chunks_exact(4)
            .map(|c| c[0])
            .flat_map(|l| [l, l])
            .collect();
        snr_db(&dec2, &out, 2, 3000)
    };
    assert!(snr > 15.0, "downsampled SBR SNR {snr:.1} dB");
}

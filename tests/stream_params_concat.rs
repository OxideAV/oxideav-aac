//! `Frame::StreamParams` on concatenated ADTS streams.
//!
//! A spliced ADTS stream may switch sampling rate, channel
//! configuration, or (under implicit signalling) SBR on and off. The
//! runtime decoder must announce exactly one event per real output
//! shape change, immediately before the first frame of the new shape,
//! with `output_params()` / `output_params_generation()` already moved
//! (core 0.1.43 contract) — and a new configuration must restart the
//! per-stream state (an SBR latch from an HE-AAC segment must not keep
//! upsampling the plain AAC-LC segment after it).

use oxideav_aac::codec_decoder::make_decoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};
use oxideav_aac::he_aac_encoder::{HeAacConfig, HeAacEncoder};
use oxideav_core::{CodecId, CodecParameters, Decoder, Error, Frame, Packet, TimeBase};

fn tone(rate: u32, channels: u8, seconds: f64) -> Vec<i16> {
    let n = (f64::from(rate) * seconds) as usize;
    let mut out = Vec::with_capacity(n * usize::from(channels));
    for i in 0..n {
        let t = i as f64 / f64::from(rate);
        for c in 0..channels {
            let f = 440.0 * (1.0 + f64::from(c) * 0.5);
            out.push((8000.0 * (2.0 * std::f64::consts::PI * f * t).sin()) as i16);
        }
    }
    out
}

fn lc(rate: u32, channels: u8) -> Vec<u8> {
    let mut enc = StreamEncoder::new(EncoderConfig::new(
        rate,
        channels,
        64_000 * u32::from(channels),
    ))
    .unwrap();
    enc.encode_all(&tone(rate, channels, 0.25)).unwrap()
}

fn he(rate: u32, channels: u8) -> Vec<u8> {
    let mut enc = HeAacEncoder::new(HeAacConfig::new(rate, channels, 48_000)).unwrap();
    enc.encode_all(&tone(rate, channels, 0.25)).unwrap()
}

fn adts_frames(data: &[u8]) -> Vec<&[u8]> {
    let mut out = Vec::new();
    let mut pos = 0;
    while pos + 7 <= data.len() {
        let len = (usize::from(data[pos + 3] & 3) << 11)
            | (usize::from(data[pos + 4]) << 3)
            | (usize::from(data[pos + 5]) >> 5);
        out.push(&data[pos..pos + len]);
        pos += len;
    }
    out
}

#[derive(Debug, PartialEq)]
enum Ev {
    /// A run of `n` audio frames of one `(rate, channels)` shape.
    Run(u32, u16, usize),
    Event(u32, u16),
}

/// Drain every frame, checking the contract on the way, and return the
/// run/event sequence.
fn drain(dec: &mut dyn Decoder, log: &mut Vec<Ev>, generation: &mut u64, seen_audio: &mut bool) {
    loop {
        match dec.receive_frame() {
            Ok(Frame::StreamParams(p)) => {
                assert!(*seen_audio, "no event for the first shape");
                let now = dec.output_params().unwrap();
                assert_eq!((p.sample_rate, p.channels), (now.sample_rate, now.channels));
                assert_ne!(
                    dec.output_params_generation(),
                    *generation,
                    "event bumps the counter"
                );
                *generation = dec.output_params_generation();
                if let Some(Ev::Event(..)) = log.last() {
                    panic!("two events in a row");
                }
                log.push(Ev::Event(p.sample_rate.unwrap(), p.channels.unwrap()));
            }
            Ok(Frame::Audio(_)) => {
                *seen_audio = true;
                assert_eq!(
                    dec.output_params_generation(),
                    *generation,
                    "no silent bump"
                );
                let p = dec.output_params().unwrap();
                let (r, c) = (p.sample_rate.unwrap(), p.channels.unwrap());
                match log.last_mut() {
                    Some(Ev::Run(lr, lc, n)) if (*lr, *lc) == (r, c) => *n += 1,
                    Some(Ev::Run(lr, lc, _)) => {
                        panic!("shape {lr}/{lc} -> {r}/{c} without an event")
                    }
                    _ => log.push(Ev::Run(r, c, 1)),
                }
            }
            Ok(_) => {}
            Err(Error::NeedMore) | Err(Error::Eof) => return,
            Err(e) => panic!("{e:?}"),
        }
    }
}

fn run(segments: &[Vec<u8>]) -> Vec<Ev> {
    let mut dec = make_decoder(&CodecParameters::audio(CodecId::new("aac"))).unwrap();
    let (mut log, mut generation, mut seen) = (Vec::new(), dec.output_params_generation(), false);
    for seg in segments {
        for f in adts_frames(seg) {
            dec.send_packet(&Packet::new(0, TimeBase::new(1, 1), f.to_vec()))
                .unwrap();
            drain(dec.as_mut(), &mut log, &mut generation, &mut seen);
        }
    }
    log
}

fn shapes(log: &[Ev]) -> Vec<(u32, u16)> {
    log.iter()
        .filter_map(|e| match e {
            Ev::Run(r, c, _) => Some((*r, *c)),
            Ev::Event(..) => None,
        })
        .collect()
}

#[test]
fn rate_channel_and_sbr_switches_announce_one_event_each() {
    let log = run(&[
        lc(44_100, 2),
        lc(48_000, 1),
        he(32_000, 2), // implicit SBR: 16 kHz core, 32 kHz out
        lc(44_100, 2), // SBR off again: no upsampling carried over
        he(44_100, 2), // 22.05 kHz core → 44.1 kHz stereo: same shape
        lc(48_000, 1),
    ]);
    assert_eq!(
        shapes(&log),
        [
            (44_100, 2),
            (48_000, 1),
            (32_000, 2),
            (44_100, 2),
            (48_000, 1)
        ],
        "{log:?}"
    );
    let events = log.iter().filter(|e| matches!(e, Ev::Event(..))).count();
    assert_eq!(events, 4, "{log:?}");
}

#[test]
fn identical_configuration_splice_has_no_event() {
    let log = run(&[lc(48_000, 2), lc(48_000, 2)]);
    assert!(matches!(log.as_slice(), [Ev::Run(48_000, 2, _)]), "{log:?}");
}

#[test]
fn flush_drain_keeps_order_and_reset_is_relative_to_last_frame() {
    let mut dec = make_decoder(&CodecParameters::audio(CodecId::new("aac"))).unwrap();
    let a = lc(44_100, 2);
    let b = lc(22_050, 1);
    // Queue old-shape and new-shape frames in one packet run, flush,
    // then drain: old frames, the event, new frames.
    let fa = adts_frames(&a);
    let fb = adts_frames(&b);
    let mut both = fa[0].to_vec();
    both.extend_from_slice(fa[1]);
    both.extend_from_slice(fb[0]);
    dec.send_packet(&Packet::new(0, TimeBase::new(1, 1), both))
        .unwrap();
    dec.flush().unwrap();
    let (mut log, mut generation, mut seen) = (Vec::new(), dec.output_params_generation(), false);
    drain(dec.as_mut(), &mut log, &mut generation, &mut seen);
    assert_eq!(
        log,
        [
            Ev::Run(44_100, 2, 2),
            Ev::Event(22_050, 1),
            Ev::Run(22_050, 1, 1)
        ]
    );
    // After a reset (seek) the rule is relative to the last returned
    // frame (22.05 kHz mono): the same shape gets no event, a
    // different one gets one.
    dec.reset().unwrap();
    dec.send_packet(&Packet::new(0, TimeBase::new(1, 1), fb[1].to_vec()))
        .unwrap();
    drain(dec.as_mut(), &mut log, &mut generation, &mut seen);
    assert_eq!(log.len(), 3, "{log:?}");
    dec.reset().unwrap();
    dec.send_packet(&Packet::new(0, TimeBase::new(1, 1), fa[2].to_vec()))
        .unwrap();
    drain(dec.as_mut(), &mut log, &mut generation, &mut seen);
    assert_eq!(log[3..], [Ev::Event(44_100, 2), Ev::Run(44_100, 2, 1)]);
}

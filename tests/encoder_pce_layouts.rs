//! PCE-described channel layouts (`channelConfiguration = 0`) through
//! the crate's own decoder and, when a reference decoder binary is on
//! PATH, through that black box: every speaker carries a distinct
//! tone, so a swapped or dropped channel blows its error-to-signal
//! ratio to ~1.

use std::fs;
use std::process::Command;

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder, FRAME_LEN};
use oxideav_aac::raw_data_block::{Element, Walker};
use oxideav_core::bits::BitReader;
use oxideav_core::{ChannelLayout, ChannelPosition};

fn per_channel_tones(n: usize, positions: &[ChannelPosition]) -> Vec<i16> {
    let channels = positions.len();
    let mut out = Vec::with_capacity(n * channels);
    for i in 0..n {
        let t = i as f64;
        for (c, &p) in positions.iter().enumerate() {
            let v = if p == ChannelPosition::LowFrequency {
                8000.0 * (2.0 * std::f64::consts::PI * 40.0 / 44_100.0 * t).sin()
            } else {
                let w = 0.02 + 0.04 * c as f64;
                9000.0 * (w * t).sin() + 3000.0 * (2.7 * w * t + 0.4).sin()
            };
            out.push(v.round() as i16);
        }
    }
    out
}

/// Canonical (rank) order of a speaker set — the encoder's input and
/// the decoder's output interleave.
fn canonical(positions: &[ChannelPosition]) -> Vec<ChannelPosition> {
    use ChannelPosition::*;
    let rank = |p: ChannelPosition| match p {
        FrontLeft => 0,
        FrontRight => 1,
        FrontCenter => 2,
        LowFrequency => 3,
        BackLeft => 4,
        BackRight => 5,
        FrontLeftOfCenter => 6,
        FrontRightOfCenter => 7,
        BackCenter => 8,
        SideLeft => 9,
        SideRight => 10,
        _ => 99,
    };
    let mut v = positions.to_vec();
    v.sort_by_key(|&p| rank(p));
    v
}

struct RoundTrip {
    stream: Vec<u8>,
    ratios: Vec<f64>,
}

fn round_trip(positions: &[ChannelPosition]) -> RoundTrip {
    let canonical = canonical(positions);
    let channels = canonical.len();
    let n = 6 * FRAME_LEN;
    let pcm = per_channel_tones(n, &canonical);
    let mut enc = StreamEncoder::with_layout(
        EncoderConfig {
            sample_rate: 44_100,
            channels: channels as u8,
            bitrate: 64_000 * channels as u32,
        },
        positions,
    )
    .unwrap();
    assert!(enc.program_config().is_some());
    let stream = enc.encode_all(&pcm).unwrap();
    let (header, _) = AdtsHeader::parse(&stream).unwrap();
    assert_eq!(header.channel_configuration, 0);

    let mut dec = StreamDecoder::new();
    let frames = dec.decode_all(&stream).unwrap();
    let mut decoded = Vec::new();
    for f in &frames {
        assert_eq!(f.channels, channels);
        decoded.extend_from_slice(&f.pcm);
    }
    assert_eq!(decoded.len(), (n + FRAME_LEN) * channels);
    let aligned = &decoded[FRAME_LEN * channels..];
    let ratios = (0..channels)
        .map(|c| {
            let mut err = 0.0f64;
            let mut sig = 0.0f64;
            for i in 0..n {
                let x = f64::from(pcm[i * channels + c]);
                let y = f64::from(aligned[i * channels + c]);
                err += (x - y) * (x - y);
                sig += x * x;
            }
            (err / sig.max(1.0)).sqrt()
        })
        .collect();
    RoundTrip { stream, ratios }
}

fn assert_identity(rt: &RoundTrip, label: &str) {
    for (c, r) in rt.ratios.iter().enumerate() {
        assert!(
            *r < 0.2,
            "{label}: channel {c} err/sig {r:.3} — {:?}",
            rt.ratios
        );
    }
}

/// The named core layouts without a Table 1.19 default, plus a
/// cinema-style set with both front pairs and a rear centre: every
/// channel comes back on its own speaker.
#[test]
fn pce_layouts_round_trip_channel_identity() {
    use ChannelPosition::*;
    let cases: Vec<(&str, Vec<ChannelPosition>)> = vec![
        ("2.1", ChannelLayout::Stereo21.positions_owned()),
        ("quad", ChannelLayout::Quad.positions_owned()),
        ("4.1", ChannelLayout::Surround41.positions_owned()),
        ("6.0", ChannelLayout::Surround60.positions_owned()),
        ("6.1", ChannelLayout::Surround61.positions_owned()),
        ("7.0", ChannelLayout::Surround70.positions_owned()),
        ("7.1 side+back", ChannelLayout::Surround71.positions_owned()),
        (
            "cinema 9.1",
            vec![
                FrontLeft,
                FrontRight,
                FrontCenter,
                LowFrequency,
                FrontLeftOfCenter,
                FrontRightOfCenter,
                BackCenter,
                SideLeft,
                SideRight,
                BackLeft,
                BackRight,
            ],
        ),
        // A default set described explicitly still round-trips.
        ("5.1 via PCE", ChannelLayout::Surround51.positions_owned()),
    ];
    for (label, positions) in cases {
        let rt = round_trip(&positions);
        eprintln!("{label}: {:?}", rt.ratios);
        assert_identity(&rt, label);
    }
}

/// The first `raw_data_block()` opens with the PCE, whose lists follow
/// the §8.5.2.2 conventions (centre first, inner pair before outer,
/// `Ls/Rs` in the side list next to a back pair, `Cs` last).
#[test]
fn pce_lists_follow_the_speaker_conventions() {
    use ChannelPosition::*;
    let positions = [
        FrontLeft,
        FrontRight,
        FrontCenter,
        LowFrequency,
        FrontLeftOfCenter,
        FrontRightOfCenter,
        BackCenter,
        SideLeft,
        SideRight,
        BackLeft,
        BackRight,
    ];
    let enc = StreamEncoder::with_layout(
        EncoderConfig {
            sample_rate: 48_000,
            channels: 11,
            bitrate: 640_000,
        },
        &positions,
    )
    .unwrap();
    let pce = enc.program_config().unwrap();
    assert_eq!(pce.sampling_frequency_index, 3); // 48 kHz
    assert_eq!(pce.object_type, 1);
    let sel = |v: &[oxideav_aac::pce::ElementSelect]| -> Vec<(bool, u8)> {
        v.iter().map(|e| (e.is_cpe, e.tag_select)).collect()
    };
    assert_eq!(
        sel(&pce.front_elements),
        vec![(false, 0), (true, 0), (true, 1)]
    );
    assert_eq!(sel(&pce.side_elements), vec![(true, 2)]);
    assert_eq!(sel(&pce.back_elements), vec![(true, 3), (false, 1)]);
    assert_eq!(pce.lfe_element_tag_selects, vec![0]);

    // On the wire: the raw_data_block() opens with that PCE (the
    // element order and tags behind it are proven by the decoder's
    // PCE-driven reorder reproducing every channel on its speaker).
    let rt = round_trip(&positions);
    let (_, hdr_len) = AdtsHeader::parse(&rt.stream).unwrap();
    let mut reader = BitReader::new(&rt.stream[hdr_len..]);
    let mut walker = Walker::new(&mut reader);
    let first = walker.next_element().unwrap().unwrap();
    let Element::ProgramConfig(on_wire) = first else {
        panic!("first element {first:?}");
    };
    // Same lists (the round trip ran at 44.1 kHz: index 4).
    assert_eq!(on_wire.sampling_frequency_index, 4);
    assert_eq!(on_wire.front_elements, pce.front_elements);
    assert_eq!(on_wire.side_elements, pce.side_elements);
    assert_eq!(on_wire.back_elements, pce.back_elements);
    assert_eq!(on_wire.lfe_element_tag_selects, pce.lfe_element_tag_selects);
}

/// Sets the PCE lists cannot express are rejected up front.
#[test]
fn unrepresentable_sets_are_rejected() {
    use ChannelPosition::*;
    let cfg = |n: u8| EncoderConfig {
        sample_rate: 44_100,
        channels: n,
        bitrate: 64_000 * u32::from(n),
    };
    // Lone left.
    assert!(StreamEncoder::with_layout(cfg(2), &[FrontLeft, FrontCenter]).is_err());
    // Inner pair without the outer pair.
    assert!(StreamEncoder::with_layout(
        cfg(3),
        &[FrontLeftOfCenter, FrontRightOfCenter, FrontCenter]
    )
    .is_err());
    // A lone back pair reads as the side pair.
    assert!(
        StreamEncoder::with_layout(cfg(4), &[FrontLeft, FrontRight, BackLeft, BackRight]).is_err()
    );
    // Height channels have no canonical rank; duplicates; count mismatch.
    assert!(StreamEncoder::with_layout(cfg(3), &[FrontLeft, FrontRight, TopFrontLeft]).is_err());
    assert!(StreamEncoder::with_layout(cfg(3), &[FrontLeft, FrontRight, FrontLeft]).is_err());
    assert!(StreamEncoder::with_layout(cfg(3), &[FrontLeft, FrontRight]).is_err());
    // The plain constructor still has no 7-channel default.
    assert!(StreamEncoder::new(cfg(7)).is_err());
}

fn ffmpeg() -> Option<&'static str> {
    ["ffmpeg"].into_iter().find(|cand| {
        Command::new(cand)
            .arg("-version")
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false)
    })
}

/// Black box: the reference decoder binary accepts the PCE streams
/// (no diagnostics) and reports the layout's channel count.
#[test]
fn reference_binary_decodes_pce_layouts() {
    let Some(ff) = ffmpeg() else {
        eprintln!("skip: no ffmpeg binary on PATH");
        return;
    };
    let dir = std::env::temp_dir().join("oxideav-aac-pce-blackbox");
    fs::create_dir_all(&dir).unwrap();
    for (label, layout) in [
        ("s61", ChannelLayout::Surround61),
        ("s71", ChannelLayout::Surround71),
        ("quad", ChannelLayout::Quad),
    ] {
        let rt = round_trip(&layout.positions_owned());
        let aac = dir.join(format!("{label}.aac"));
        let wav = dir.join(format!("{label}.wav"));
        fs::write(&aac, &rt.stream).unwrap();
        let _ = fs::remove_file(&wav);
        let out = Command::new(ff)
            .args(["-hide_banner", "-loglevel", "error", "-y", "-i"])
            .arg(&aac)
            .arg(&wav)
            .output()
            .expect("run reference decoder");
        assert!(
            out.status.success(),
            "{label}: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert!(
            stderr.trim().is_empty(),
            "{label}: reference decoder diagnostics: {stderr}"
        );
        let wav_bytes = fs::read(&wav).unwrap();
        // fmt chunk: channel count at byte 22 of a canonical header.
        let channels = u16::from_le_bytes([wav_bytes[22], wav_bytes[23]]);
        assert_eq!(channels, layout.channel_count(), "{label}");
    }
}

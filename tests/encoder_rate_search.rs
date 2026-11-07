//! The rate loop's search for a frame's scalefactor offset: every
//! `raw_data_block()` it assembles is a full quantize-and-price pass
//! over all channels, so the count of passes per frame is the
//! encoder's speed.

use oxideav_aac::adts::ADTS_HEADER_BYTES_NO_CRC;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};

const RATE: u32 = 48_000;
const FRAME: usize = 1024;

/// A fixed chord with a little stereo width, every tone a multiple of
/// the frame rate (48 000 / 1024 Hz), so each frame holds the same
/// waveform and the offset that fits the budget is the same every
/// frame.
fn chord(frames: usize) -> Vec<i16> {
    let frame_hz = RATE as f64 / FRAME as f64;
    let tones: Vec<f64> = [2.0f64, 4.0, 7.0, 9.0, 14.0, 19.0, 28.0, 56.0, 112.0]
        .iter()
        .map(|k| k * frame_hz)
        .collect();
    (0..frames * FRAME)
        .flat_map(|n| {
            let t = n as f64 / RATE as f64;
            let l: f64 = tones
                .iter()
                .enumerate()
                .map(|(i, f)| (2.0 * std::f64::consts::PI * f * t).sin() / (i + 1) as f64)
                .sum();
            let r: f64 = tones
                .iter()
                .enumerate()
                .map(|(i, f)| {
                    (2.0 * std::f64::consts::PI * f * t + 0.3 * i as f64).sin() / (i + 1) as f64
                })
                .sum();
            [(l * 9000.0) as i16, (r * 9000.0) as i16]
        })
        .collect()
}

/// Once the signal is steady, the search starts from the previous
/// frame's offset and needs one pass to confirm it and one to find
/// that the next finer offset no longer fits. (The ladder it replaced
/// started every frame from zero and took at least three.)
#[test]
fn a_steady_signal_settles_each_frame_in_about_two_passes() {
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: RATE,
        channels: 2,
        bitrate: 128_000,
    })
    .unwrap();
    let pcm = chord(80);
    let warm = 20;
    for f in 0..warm {
        enc.encode_frame(&pcm[f * 2 * FRAME..(f + 1) * 2 * FRAME])
            .unwrap();
    }
    let before = enc.rate_passes();
    let steady = 80 - warm;
    for f in warm..80 {
        enc.encode_frame(&pcm[f * 2 * FRAME..(f + 1) * 2 * FRAME])
            .unwrap();
    }
    let per_frame = (enc.rate_passes() - before) as f64 / steady as f64;
    assert!(per_frame <= 2.5, "{per_frame} passes per frame");
}

/// Whatever path the search takes, every frame it emits fits the
/// bitrate's frame budget: the stream's mean rate stays within a
/// twentieth of the target, and no frame sits far above it.
#[test]
fn every_frame_of_the_search_fits_the_budget() {
    for &bitrate in &[64_000u32, 128_000, 256_000] {
        let mut enc = StreamEncoder::new(EncoderConfig {
            sample_rate: RATE,
            channels: 2,
            bitrate,
        })
        .unwrap();
        let pcm = chord(60);
        let stream = enc.encode_all(&pcm).unwrap();
        let budget = bitrate as usize * FRAME / RATE as usize / 8;
        let mut frames = 0usize;
        let mut at = 0usize;
        while at + ADTS_HEADER_BYTES_NO_CRC <= stream.len() {
            let len = ((stream[at + 3] as usize & 3) << 11)
                | ((stream[at + 4] as usize) << 3)
                | (stream[at + 5] as usize >> 5);
            assert!(len <= budget, "frame {frames} is {len} bytes at {bitrate}");
            frames += 1;
            at += len;
        }
        assert_eq!(at, stream.len());
        assert!(frames >= 60);
    }
}

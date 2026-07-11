//! Deterministic mutation battery over the SBR decode modes.
//!
//! Every byte-level corruption of the HE-AAC v1 fixture must surface
//! as a decode `Err` (or decode to different-but-finite PCM) — never a
//! panic, hang, or out-of-bounds — in all four §4.6.18 mode
//! combinations: high-quality / low-power × dual-rate / downsampled.
//! The mutation schedule is a fixed LCG so failures reproduce exactly.
//!
//! Skips (logged, success) when `docs/` is absent (standalone CI).

use std::fs;
use std::path::PathBuf;

use oxideav_aac::decode::StreamDecoder;

/// Fixed-seed LCG (numerical-recipes constants) for reproducibility.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0 >> 33
    }
}

/// The byte prefix carrying the stream's first `n` ADTS frames.
fn prefix_frames(data: &[u8], n: usize) -> Vec<u8> {
    use oxideav_aac::adts::AdtsHeader;
    let mut off = 0usize;
    for _ in 0..n {
        let Ok((hdr, _)) = AdtsHeader::parse(&data[off..]) else {
            break;
        };
        let fl = hdr.aac_frame_length as usize;
        if fl == 0 || off + fl > data.len() {
            break;
        }
        off += fl;
    }
    data[..off].to_vec()
}

fn decode_all_modes(data: &[u8]) {
    for (low_power, downsampled) in [(false, false), (false, true), (true, false), (true, true)] {
        let mut dec = StreamDecoder::new();
        dec.set_sbr_low_power(low_power);
        dec.set_sbr_downsampled(downsampled);
        // Err is fine; panic is not. A successful decode must keep its
        // frame geometry self-consistent.
        if let Ok(frames) = dec.decode_all(data) {
            for f in &frames {
                if f.channels > 0 {
                    assert_eq!(f.pcm.len() % f.channels, 0);
                }
            }
        }
    }
}

#[test]
fn corrupted_he_aac_streams_never_panic_in_any_mode() {
    let dir = PathBuf::from("../../docs/audio/aac/fixtures/he-aac-v1-stereo-44100-32kbps-adts");
    let Ok(full) = fs::read(dir.join("input.aac")) else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    // An 8-frame prefix keeps the battery fast while covering the
    // header/parse/reset paths every corrupt stream exercises.
    let data = prefix_frames(&full, 8);

    // Single-byte XOR mutations at LCG positions.
    let mut rng = Lcg(0x5bd1_e995_a412_0001);
    for _ in 0..40 {
        let pos = (rng.next() as usize) % data.len();
        let val = (rng.next() as u8) | 1; // never a no-op XOR
        let mut m = data.clone();
        m[pos] ^= val;
        decode_all_modes(&m);
    }

    // Truncations (mid-frame cuts).
    for _ in 0..8 {
        let cut = 1 + (rng.next() as usize) % (data.len() - 1);
        decode_all_modes(&data[..cut]);
    }

    // Burst corruption: 16-byte scrambles.
    for _ in 0..8 {
        let pos = (rng.next() as usize) % data.len().saturating_sub(16);
        let mut m = data.clone();
        for b in &mut m[pos..pos + 16] {
            *b = rng.next() as u8;
        }
        decode_all_modes(&m);
    }
}

#[test]
fn corrupted_sbrcrc_stream_never_panics_in_any_mode() {
    let dir = PathBuf::from("../../docs/audio/aac/fixtures/he-aac-v1-sbrcrc-adts");
    let Ok(full) = fs::read(dir.join("input.aac")) else {
        eprintln!("skip: fixtures unavailable");
        return;
    };
    let data = prefix_frames(&full, 8);
    let mut rng = Lcg(0x0bad_c0de_1234_5678);
    for _ in 0..16 {
        let pos = (rng.next() as usize) % data.len();
        let val = (rng.next() as u8) | 1;
        let mut m = data.clone();
        m[pos] ^= val;
        decode_all_modes(&m);
    }
}

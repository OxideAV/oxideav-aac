//! A fill element carrying any `extension_type` must leave the audio
//! untouched.
//!
//! ISO/IEC 14496-3:2009 Table 4.57 gives every 4-bit
//! `extension_type` a body: the defined types have their own
//! branches and every reserved value (Table 4.121: "can be used for
//! a further extension of the syntax in a compatible way") takes the
//! `default` branch — `8 * (cnt - 1) + 4` `other_bits`, "bits to be
//! discarded by the decoder". So a decoder steps over a fill element
//! of a type it does not interpret.
//!
//! These tests take streams from this crate's own encoders, splice a
//! fill element into every access unit by bit-level rewriting (ahead
//! of the first element, and again just before `ID_END`, where it
//! follows a channel element), and require the decoded samples to be
//! identical to those of the unmodified stream.

use oxideav_aac::adts::{ADTS_HEADER_BYTES_NO_CRC, AdtsHeader};
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, FRAME_LEN, StreamEncoder};
use oxideav_aac::he_aac_encoder::{HE_FRAME_LEN, HeAacConfig, HeAacEncoder};

/// Deterministic multichannel test signal with a transient, so the
/// streams carry both long and short windows.
fn signal(n: usize, channels: usize) -> Vec<i16> {
    let mut out = Vec::with_capacity(n * channels);
    for i in 0..n {
        let t = i as f64;
        let burst = (FRAME_LEN + 300..FRAME_LEN + 800).contains(&i);
        for c in 0..channels {
            let w = 0.021 + 0.017 * c as f64;
            let mut v = 5000.0 * (w * t).sin() + 1500.0 * (3.1 * w * t).cos();
            if burst {
                v += 12000.0 * (0.61 * t).sin();
            }
            out.push(v.round() as i16);
        }
    }
    out
}

fn to_bits(bytes: &[u8]) -> Vec<bool> {
    bytes
        .iter()
        .flat_map(|&b| (0..8).map(move |i| b & (0x80 >> i) != 0))
        .collect()
}

/// Pack MSB-first, zero-padding the last byte (the
/// `byte_alignment()` that closes a `raw_data_block()`).
fn to_bytes(bits: &[bool]) -> Vec<u8> {
    bits.chunks(8)
        .map(|chunk| {
            chunk
                .iter()
                .enumerate()
                .fold(0u8, |acc, (i, &bit)| acc | (u8::from(bit) << (7 - i)))
        })
        .collect()
}

fn push_value(bits: &mut Vec<bool>, value: u32, width: u32) {
    bits.extend((0..width).rev().map(|i| value >> i & 1 != 0));
}

/// The bits of a `fill_element()` (with its `ID_FIL` tag) whose
/// `extension_payload()` chain is `payload`: `count` (4 bits), an
/// `esc_count` byte when `count == 15` (`cnt = count + esc_count -
/// 1`), then the `cnt` payload bytes.
fn fill_element(payload: &[u8]) -> Vec<bool> {
    let cnt = payload.len() as u32;
    // The largest cnt a fill element can state is 15 + 255 - 1.
    assert!(cnt < 15 + 255);
    let mut bits = Vec::new();
    push_value(&mut bits, 0b110, 3); // ID_FIL
    if cnt < 15 {
        push_value(&mut bits, cnt, 4);
    } else {
        push_value(&mut bits, 15, 4);
        push_value(&mut bits, cnt - 14, 8);
    }
    bits.extend(to_bits(payload));
    bits
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Place {
    /// Ahead of the access unit's first syntactic element.
    Front,
    /// Just before `ID_END`: the fill follows the last channel
    /// element.
    BeforeEnd,
}

/// Splice `fill` into one `raw_data_block()`.
fn splice(block: &[u8], fill: &[bool], place: Place) -> Vec<u8> {
    let mut bits = to_bits(block);
    let at = match place {
        Place::Front => 0,
        Place::BeforeEnd => {
            // The block closes with ID_END ('111') and zero-valued
            // alignment bits, so ID_END is the last three set bits.
            let last = bits.iter().rposition(|&b| b).expect("empty block");
            assert!(last >= 2 && bits[last - 2] && bits[last - 1]);
            bits.truncate(last + 1);
            last - 2
        }
    };
    bits.splice(at..at, fill.iter().copied());
    to_bytes(&bits)
}

/// Rewrite every ADTS frame of `stream`, splicing the fill element
/// `fill_for(frame_index)` into its `raw_data_block()`.
fn rewrite(stream: &[u8], place: Place, fill_for: &dyn Fn(usize) -> Vec<u8>) -> Vec<u8> {
    let mut out = Vec::with_capacity(stream.len() * 2);
    let mut pos = 0;
    let mut index = 0;
    while pos < stream.len() {
        let (mut header, offset) = AdtsHeader::parse(&stream[pos..]).unwrap();
        assert!(header.protection_absent);
        assert_eq!(offset, ADTS_HEADER_BYTES_NO_CRC);
        assert_eq!(header.number_of_raw_data_blocks_in_frame, 1);
        let end = pos + header.aac_frame_length as usize;
        let block = splice(
            &stream[pos + offset..end],
            &fill_element(&fill_for(index)),
            place,
        );
        header.aac_frame_length = (offset + block.len()) as u16;
        out.extend_from_slice(&header.write().unwrap());
        out.extend_from_slice(&block);
        pos = end;
        index += 1;
    }
    assert!(index >= 3, "stream too short to be a meaningful test");
    out
}

/// Decode a whole ADTS stream to `(channels, sample_rate, pcm)` per
/// frame.
fn decode(stream: &[u8]) -> Vec<(u8, u32, Vec<i16>)> {
    StreamDecoder::new()
        .decode_all(stream)
        .unwrap()
        .into_iter()
        .map(|f| (f.channels as u8, f.sample_rate, f.pcm))
        .collect()
}

/// Deterministic filler bytes.
fn noise(seed: u32, n: usize) -> impl Iterator<Item = u8> {
    let mut state = seed.wrapping_mul(0x9E37_79B9) | 1;
    (0..n).map(move |_| {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        (state >> 11) as u8
    })
}

/// Fill-element lengths: the shortest, a plain `count`, the largest
/// plain `count`, the first escaped one, and a long escaped one.
const LENGTHS: [usize; 5] = [1, 6, 14, 15, 97];

/// A `cnt`-byte payload taking the syntax table's `default` branch
/// (or another whole-`cnt` branch): first byte `head`, then noise.
fn whole(head: u8, cnt: usize, seed: u32) -> Vec<u8> {
    let mut p = vec![head];
    p.extend(noise(seed, cnt - 1));
    p
}

/// An `EXT_DATA_ELEMENT` / `ANC_DATA` payload of `len` data bytes.
fn anc(len: usize, seed: u32) -> Vec<u8> {
    let mut p = vec![0x20];
    p.extend(std::iter::repeat_n(255u8, len / 255));
    p.push((len % 255) as u8);
    p.extend(noise(seed, len));
    p
}

/// Maps a frame index to that frame's fill-element payload.
type PayloadBuilder = Box<dyn Fn(usize) -> Vec<u8>>;

/// Every payload kind under test, as `(label, builder)`; the builder
/// maps a frame index to that frame's fill payload.
fn payload_kinds() -> Vec<(String, PayloadBuilder)> {
    let mut kinds: Vec<(String, PayloadBuilder)> = Vec::new();
    let len = |frame: usize, salt: usize| LENGTHS[(frame + salt) % LENGTHS.len()];
    // Reserved extension_type values (Table 4.121).
    for ty in [
        0b0011u8, 0b0100, 0b0101, 0b0110, 0b0111, 0b1000, 0b1001, 0b1010, 0b1111,
    ] {
        kinds.push((
            format!("reserved type {ty:#x}"),
            Box::new(move |frame| {
                let seed = frame as u32 * 16 + u32::from(ty);
                whole(
                    (ty << 4) | (seed as u8 & 0xf),
                    len(frame, ty as usize),
                    seed,
                )
            }),
        ));
    }
    kinds.push((
        "EXT_FILL".into(),
        Box::new(move |frame| whole(0x0b, len(frame, 1), frame as u32 + 900)),
    ));
    kinds.push((
        "EXT_FILL_DATA, pattern".into(),
        Box::new(move |frame| {
            let mut p = vec![0x10];
            p.resize(len(frame, 2), 0xa5);
            p
        }),
    ));
    kinds.push((
        "EXT_FILL_DATA, other values".into(),
        Box::new(move |frame| whole(0x17, len(frame, 3), frame as u32 + 700)),
    ));
    kinds.push((
        "EXT_DATA_ELEMENT, ANC_DATA".into(),
        Box::new(move |frame| anc([0usize, 3, 40, 255, 262][frame % 5], frame as u32 + 500)),
    ));
    kinds.push((
        "EXT_DATA_ELEMENT, reserved version".into(),
        Box::new(move |frame| {
            let version = 1 + (frame % 15) as u8;
            whole(0x20 | version, len(frame, 4), frame as u32 + 300)
        }),
    ));
    kinds.push((
        "EXT_SAC_DATA".into(),
        Box::new(move |frame| {
            whole(
                0xc0 | (frame as u8 & 0xf),
                len(frame, 0),
                frame as u32 + 100,
            )
        }),
    ));
    // One fill element chaining several extension_payload()s: an
    // ancillary data element (which returns its own length), then a
    // reserved type, then EXT_FILL_DATA.
    kinds.push((
        "chain: ANC_DATA + reserved + EXT_FILL_DATA".into(),
        Box::new(move |frame| {
            let mut p = anc(5 + frame % 4, frame as u32 + 40);
            p.extend(whole(0xa5, 1 + frame % 9, frame as u32 + 41));
            p.extend(whole(0x12, 3, frame as u32 + 42));
            p
        }),
    ));
    kinds
}

fn assert_fill_elements_are_inert(name: &str, stream: &[u8]) {
    let reference = decode(stream);
    assert!(reference.iter().any(|f| f.2.iter().any(|&s| s != 0)));
    for (label, build) in payload_kinds() {
        for place in [Place::Front, Place::BeforeEnd] {
            let rewritten = rewrite(stream, place, build.as_ref());
            assert_ne!(rewritten, stream);
            let frames = StreamDecoder::new()
                .decode_all(&rewritten)
                .unwrap_or_else(|e| panic!("{name}: {label} at {place:?}: {e}"));
            assert_eq!(
                frames.len(),
                reference.len(),
                "{name}: {label} at {place:?}"
            );
            for (i, (got, want)) in frames.iter().zip(&reference).enumerate() {
                assert!(
                    (got.channels as u8, got.sample_rate) == (want.0, want.1) && got.pcm == want.2,
                    "{name}: {label} at {place:?}: frame {i} differs"
                );
            }
        }
    }
}

#[test]
fn lc_mono_ignores_fill_elements_of_every_type() {
    let mut enc = StreamEncoder::new(EncoderConfig::new(44_100, 1, 64_000)).unwrap();
    let stream = enc.encode_all(&signal(5 * FRAME_LEN, 1)).unwrap();
    assert_fill_elements_are_inert("LC mono", &stream);
}

#[test]
fn lc_stereo_ignores_fill_elements_of_every_type() {
    let mut enc = StreamEncoder::new(EncoderConfig::new(48_000, 2, 128_000)).unwrap();
    let stream = enc.encode_all(&signal(5 * FRAME_LEN, 2)).unwrap();
    assert_fill_elements_are_inert("LC stereo", &stream);
}

/// The reported shape: a 5.1 AAC-LC track at 48 kHz, 224 kbit/s.
#[test]
fn lc_5_1_ignores_fill_elements_of_every_type() {
    let mut enc = StreamEncoder::new(EncoderConfig::new(48_000, 6, 224_000)).unwrap();
    let stream = enc.encode_all(&signal(4 * FRAME_LEN, 6)).unwrap();
    assert_eq!(decode(&stream)[0].0, 6);
    assert_fill_elements_are_inert("LC 5.1", &stream);
}

/// With SBR in the stream the access unit already carries an SBR
/// fill element after its channel element; the extra fill element
/// must not disturb it.
#[test]
fn he_aac_ignores_fill_elements_of_every_type() {
    let cfg = HeAacConfig::new(44_100, 1, 40_000);
    let stream = HeAacEncoder::new(cfg)
        .unwrap()
        .encode_all(&signal(5 * HE_FRAME_LEN, 1))
        .unwrap();
    assert_fill_elements_are_inert("HE-AAC mono", &stream);
}

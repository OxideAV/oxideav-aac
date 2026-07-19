//! `ep_frame()` end-to-end — §1.8.2.2 / §1.8.4: encode → decode
//! round-trips across the configuration space (interleave modes,
//! SRCPC/RS protection, CRCs, escapes, until-the-end classes,
//! reordering, stuffing), FEC error correction through the full
//! frame, and a corruption battery.

use oxideav_aac::ep_config::{EpClass, EpPredefinedSet, ErrorProtectionSpecificConfig};
use oxideav_aac::ep_frame::{EpFrameCodec, EpFrameData};

fn prand_bits(n: usize, mut seed: u32) -> Vec<bool> {
    let mut v = Vec::with_capacity(n);
    for _ in 0..n {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        v.push(seed & 0x8000_0000 != 0);
    }
    v
}

fn srcpc_class(len: Option<u16>, len_field: Option<u8>, rate: u8, crclen: u8) -> EpClass {
    EpClass {
        length_escape: len.is_none(),
        rate_escape: false,
        crclen_escape: false,
        concatenate_flag: false,
        fec_type: 0,
        termination_switch: Some(true),
        interleave_switch: None,
        class_optional: false,
        number_of_bits_for_length: len_field,
        class_length: len,
        class_rate: Some(rate),
        class_crclen: Some(crclen),
    }
}

fn rs_class(len_bits: u16, k: u8, crclen: u8) -> EpClass {
    EpClass {
        length_escape: false,
        rate_escape: false,
        crclen_escape: false,
        concatenate_flag: false,
        fec_type: 1,
        termination_switch: None,
        interleave_switch: None,
        class_optional: false,
        number_of_bits_for_length: None,
        class_length: Some(len_bits),
        class_rate: Some(k),
        class_crclen: Some(crclen),
    }
}

fn cfg_one_set(
    interleave_type: u8,
    bit_stuffing: u8,
    classes: Vec<EpClass>,
) -> ErrorProtectionSpecificConfig {
    ErrorProtectionSpecificConfig {
        interleave_type,
        bit_stuffing,
        number_of_concatenated_frame: 1,
        sets: vec![EpPredefinedSet {
            classes,
            class_reordered_output: false,
            class_output_order: Vec::new(),
        }],
        header_protection: false,
        header_rate: None,
        header_crclen: None,
    }
}

fn frame_for(codec: &EpFrameCodec, choice: usize, lens: &[usize], seed: u32) -> EpFrameData {
    let n = codec.sets()[choice].classes.len();
    EpFrameData {
        choice_of_pred: choice,
        classes: lens
            .iter()
            .enumerate()
            .map(|(i, &l)| prand_bits(l, seed ^ (i as u32) << 8))
            .collect(),
        rate_codes: vec![None; n],
        crc_codes: vec![None; n],
    }
}

#[test]
fn mode0_srcpc_roundtrip() {
    // Two fixed SRCPC classes + one until-the-end, mixed CRCs.
    let cfg = cfg_one_set(
        0,
        1,
        vec![
            srcpc_class(Some(40), None, 8, 6),
            srcpc_class(Some(64), None, 0, 8),
            srcpc_class(None, Some(0), 4, 0), // until the end
        ],
    );
    let codec = EpFrameCodec::new(cfg).unwrap();
    let frame = frame_for(&codec, 0, &[40, 64, 111], 0xAB);
    let bytes = codec.encode(&frame).unwrap();
    let decoded = codec.decode(&bytes).unwrap();
    assert_eq!(decoded, frame);
}

#[test]
fn mode0_escaped_length_roundtrip() {
    let cfg = cfg_one_set(
        0,
        1,
        vec![
            srcpc_class(None, Some(10), 8, 8), // in-band length
            srcpc_class(Some(56), None, 0, 0),
        ],
    );
    let codec = EpFrameCodec::new(cfg).unwrap();
    for len in [1usize, 17, 200, 900] {
        let frame = frame_for(&codec, 0, &[len, 56], 0xCD ^ len as u32);
        let bytes = codec.encode(&frame).unwrap();
        let decoded = codec.decode(&bytes).unwrap();
        assert_eq!(decoded, frame, "len {len}");
    }
}

#[test]
fn mode0_rs_roundtrip_and_chain() {
    // Independent RS class + a two-member chain (fec_type 2 then 1).
    let mut chain_head = rs_class(80, 3, 8);
    chain_head.fec_type = 2;
    let cfg = cfg_one_set(
        0,
        1,
        vec![rs_class(64, 2, 0), chain_head, rs_class(120, 3, 8)],
    );
    let codec = EpFrameCodec::new(cfg).unwrap();
    let frame = frame_for(&codec, 0, &[64, 80, 120], 0xEF);
    let bytes = codec.encode(&frame).unwrap();
    let decoded = codec.decode(&bytes).unwrap();
    assert_eq!(decoded, frame);
}

#[test]
fn mode0_reordered_output() {
    let cfg = ErrorProtectionSpecificConfig {
        interleave_type: 0,
        bit_stuffing: 1,
        number_of_concatenated_frame: 1,
        sets: vec![EpPredefinedSet {
            classes: vec![
                srcpc_class(Some(24), None, 0, 4),
                srcpc_class(Some(48), None, 8, 6),
                srcpc_class(Some(16), None, 4, 0),
            ],
            class_reordered_output: true,
            // Transmit class 2 first, then 0, then 1.
            class_output_order: vec![2, 0, 1],
        }],
        header_protection: false,
        header_rate: None,
        header_crclen: None,
    };
    let codec = EpFrameCodec::new(cfg).unwrap();
    let frame = frame_for(&codec, 0, &[24, 48, 16], 0x11);
    let bytes = codec.encode(&frame).unwrap();
    let decoded = codec.decode(&bytes).unwrap();
    assert_eq!(decoded, frame);
}

#[test]
fn choice_of_pred_and_optional_expansion() {
    // One wire set with an optional class → two expanded sets; the
    // in-band choice distinguishes them.
    let mut optional = srcpc_class(Some(32), None, 8, 6);
    optional.class_optional = true;
    let cfg = cfg_one_set(0, 1, vec![srcpc_class(Some(40), None, 0, 4), optional]);
    let codec = EpFrameCodec::new(cfg).unwrap();
    assert_eq!(codec.sets().len(), 2);
    assert_eq!(codec.npred(), 1);

    // Set 0: both classes.
    let f0 = frame_for(&codec, 0, &[40, 32], 0x21);
    let b0 = codec.encode(&f0).unwrap();
    assert_eq!(codec.decode(&b0).unwrap(), f0);

    // Set 1: the optional class absent.
    let f1 = frame_for(&codec, 1, &[40], 0x31);
    let b1 = codec.encode(&f1).unwrap();
    assert_eq!(codec.decode(&b1).unwrap(), f1);
    assert!(b1.len() < b0.len());
}

#[test]
fn inband_rate_and_crc_escapes() {
    let mut c = srcpc_class(Some(48), None, 0, 0);
    c.rate_escape = true;
    c.crclen_escape = true;
    c.class_rate = None;
    c.class_crclen = None;
    let cfg = cfg_one_set(0, 1, vec![c, srcpc_class(Some(24), None, 0, 4)]);
    let codec = EpFrameCodec::new(cfg).unwrap();
    for (rate_code, crc_code) in [(0u8, 0u8), (3, 2), (7, 7)] {
        let mut frame = frame_for(&codec, 0, &[48, 24], 0x41);
        frame.rate_codes[0] = Some(rate_code);
        frame.crc_codes[0] = Some(crc_code);
        let bytes = codec.encode(&frame).unwrap();
        let decoded = codec.decode(&bytes).unwrap();
        assert_eq!(decoded, frame, "rate {rate_code} crc {crc_code}");
    }
}

#[test]
fn mode1_roundtrip() {
    let cfg = cfg_one_set(
        1,
        1,
        vec![
            srcpc_class(Some(40), None, 8, 6),
            srcpc_class(Some(96), None, 4, 8),
            srcpc_class(Some(23), None, 0, 0),
        ],
    );
    let codec = EpFrameCodec::new(cfg).unwrap();
    let frame = frame_for(&codec, 0, &[40, 96, 23], 0x51);
    let bytes = codec.encode(&frame).unwrap();
    let decoded = codec.decode(&bytes).unwrap();
    assert_eq!(decoded, frame);
}

#[test]
fn mode1_rs_bytewise_roundtrip() {
    let cfg = cfg_one_set(1, 1, vec![rs_class(64, 2, 0), rs_class(160, 4, 8)]);
    let codec = EpFrameCodec::new(cfg).unwrap();
    let frame = frame_for(&codec, 0, &[64, 160], 0x61);
    let bytes = codec.encode(&frame).unwrap();
    let decoded = codec.decode(&bytes).unwrap();
    assert_eq!(decoded, frame);
}

#[test]
fn mode2_switch_matrix_roundtrip() {
    // Four classes exercising every interleave_switch value.
    let mk = |len: u16, sw: u8| {
        let mut c = srcpc_class(Some(len), None, 8, 4);
        c.interleave_switch = Some(sw);
        c
    };
    let cfg = cfg_one_set(2, 1, vec![mk(40, 0), mk(64, 1), mk(56, 2), mk(32, 3)]);
    let codec = EpFrameCodec::new(cfg).unwrap();
    let frame = frame_for(&codec, 0, &[40, 64, 56, 32], 0x71);
    let bytes = codec.encode(&frame).unwrap();
    let decoded = codec.decode(&bytes).unwrap();
    assert_eq!(decoded, frame);
}

#[test]
fn frame_corrects_channel_errors() {
    // A strongly protected frame (SRCPC 8/24, CRC16) survives sparse
    // bit errors in the class region: the Viterbi pass corrects them
    // and the CRC confirms.
    let cfg = cfg_one_set(0, 1, vec![srcpc_class(Some(120), None, 16, 16)]);
    let codec = EpFrameCodec::new(cfg).unwrap();
    let frame = frame_for(&codec, 0, &[120], 0x81);
    let mut bytes = codec.encode(&frame).unwrap();
    // The single-set frame has no choice_of_pred field and stuffing
    // is on; flip well-spread bits in the payload region.
    let n = bytes.len();
    for &pos in &[n / 4, n / 2, 3 * n / 4] {
        bytes[pos] ^= 0x10;
    }
    let decoded = codec.decode(&bytes).unwrap();
    assert_eq!(decoded, frame);
}

#[test]
fn rs_frame_corrects_byte_errors() {
    let cfg = cfg_one_set(0, 1, vec![rs_class(240, 5, 16)]);
    let codec = EpFrameCodec::new(cfg).unwrap();
    let frame = frame_for(&codec, 0, &[240], 0x91);
    let mut bytes = codec.encode(&frame).unwrap();
    // The in-band header shifts the class region off byte alignment,
    // so one flipped carrier byte can straddle two RS symbols; two
    // flips stay within the k = 5 correction budget.
    let n = bytes.len();
    bytes[n / 3] ^= 0xFF;
    bytes[2 * n / 3] ^= 0xA5;
    let decoded = codec.decode(&bytes).unwrap();
    assert_eq!(decoded, frame);
}

#[test]
fn corruption_battery_never_panics() {
    for (it, name) in [(0u8, "mode0"), (1, "mode1"), (2, "mode2")] {
        let mk = |len: u16, sw: u8| {
            let mut c = srcpc_class(Some(len), None, 8, 6);
            c.interleave_switch = if it == 2 { Some(sw) } else { None };
            c
        };
        let cfg = cfg_one_set(it, 1, vec![mk(40, 1), mk(64, 2), mk(24, 0)]);
        let codec = EpFrameCodec::new(cfg).unwrap();
        let frame = frame_for(&codec, 0, &[40, 64, 24], 0xA1);
        let bytes = codec.encode(&frame).unwrap();
        let mut oks = 0usize;
        let mut errs = 0usize;
        for bit in 0..bytes.len() * 8 {
            let mut mutated = bytes.clone();
            mutated[bit / 8] ^= 0x80 >> (bit % 8);
            match codec.decode(&mutated) {
                Ok(_) => oks += 1,
                Err(_) => errs += 1,
            }
        }
        // The FEC layer corrects most single-bit flips; some land in
        // uncovered positions either way — the point is no panic.
        assert!(oks + errs == bytes.len() * 8, "{name}");
        // Truncations.
        for keep in 0..bytes.len() {
            let _ = codec.decode(&bytes[..keep]);
        }
    }
}

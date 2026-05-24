//! AAC-LD (AOT 23) multichannel `er_raw_data_block()` decode tests.
//!
//! Round-111 wires channelConfiguration 3..=7 per ISO/IEC 14496-3
//! §4.4.2.3 Table 4.19 — the LD top-level payload has no `id_syn_ele`
//! dispatch loop; the element sequence is fixed by the channel config:
//!
//! | config | elements                       | channels |
//! |--------|--------------------------------|----------|
//! | 3      | SCE CPE                        | 3        |
//! | 4      | SCE CPE SCE                    | 4        |
//! | 5      | SCE CPE CPE                    | 5        |
//! | 6      | SCE CPE CPE LFE                | 6 (5.1)  |
//! | 7      | SCE CPE CPE CPE LFE            | 8 (7.1)  |
//!
//! No `ffmpeg`/`faad` LD multichannel encoder is available (the trace
//! doc's §11.2 documents the AAC-LD-profile fixture gap), so these are
//! synthetic-bitstream decodes: we assert the deterministic channel
//! count + silence on all-zero-spectrum frames, and a content-placement
//! frame that proves elements land in the correct output channel slots.

use oxideav_core::bits::BitWriter;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

/// LD AudioSpecificConfig for AOT 23 (sf_idx, channelConfiguration,
/// 512-sample frame). Mirrors `ld_decode_round_trip::build_ld_asc`.
fn build_ld_asc(sf_idx: u8, ch: u8) -> Vec<u8> {
    let mut bw = BitWriter::new();
    bw.write_u32(23, 5); // audioObjectType = ER AAC LD
    bw.write_u32(sf_idx as u32, 4);
    bw.write_u32(ch as u32, 4);
    bw.write_bit(false); // frameLengthFlag = 0 (512)
    bw.write_bit(false); // dependsOnCoreCoder
    bw.write_bit(false); // extensionFlag
    bw.finish()
}

/// Append a zero-spectrum SCE (or LFE — identical syntax) body: tag,
/// global_gain, ics_info(max_sfb=0), no predictor, pulse/tns/gain = 0.
fn push_zero_sce(bw: &mut BitWriter) {
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_u32(0, 8); // global_gain
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(0, 2); // window_sequence = OnlyLong
    bw.write_bit(false); // window_shape = sine
    bw.write_u32(0, 6); // max_sfb = 0
    bw.write_bit(false); // predictor_data_present = 0
    bw.write_bit(false); // pulse_data_present
    bw.write_bit(false); // tns_data_present
    bw.write_bit(false); // gain_control_data_present
}

/// Append a zero-spectrum CPE body (independent ICS, both channels
/// max_sfb=0).
fn push_zero_cpe(bw: &mut BitWriter) {
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_bit(false); // common_window = 0
    for _ in 0..2 {
        bw.write_u32(0, 8); // global_gain
        bw.write_bit(false); // ics_reserved_bit
        bw.write_u32(0, 2); // window_sequence = OnlyLong
        bw.write_bit(false); // window_shape
        bw.write_u32(0, 6); // max_sfb = 0
        bw.write_bit(false); // predictor_data_present
        bw.write_bit(false); // pulse
        bw.write_bit(false); // tns
        bw.write_bit(false); // gain_control
    }
}

/// BOOK1 (4-dim, lav=1, signed) Huffman code for one [-1,1]^4 tuple.
fn book1_code(tuple: [i32; 4]) -> (u32, u8) {
    use oxideav_aac::huffman_tables::{BOOK1_BITS, BOOK1_CODES};
    let mut idx = 0usize;
    for &v in &tuple {
        idx = idx * 3 + (v + 1) as usize;
    }
    (BOOK1_CODES[idx] as u32, BOOK1_BITS[idx])
}

/// Append an SCE body carrying one codebook-1 band (sfb 0, the four
/// lowest coefficients), with the given global_gain. Produces non-silent
/// PCM after the LD IMDCT.
fn push_content_sce(bw: &mut BitWriter, global_gain: u8, tuple: [i32; 4]) {
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_u32(global_gain as u32, 8); // global_gain
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(0, 2); // window_sequence = OnlyLong
    bw.write_bit(false); // window_shape = sine
    bw.write_u32(1, 6); // max_sfb = 1
    bw.write_bit(false); // predictor_data_present = 0
                         // section_data(): one section, codebook 1, length 1 sfb.
    bw.write_u32(1, 4); // sect_cb = 1
    bw.write_u32(1, 5); // sect_len_incr = 1
                        // scale_factor_data(): one band -> one delta of 0 (1-bit code).
    bw.write_bit(false);
    bw.write_bit(false); // pulse
    bw.write_bit(false); // tns
    bw.write_bit(false); // gain_control
    let (code, nbits) = book1_code(tuple);
    bw.write_u32(code, nbits as u32);
}

/// Decode one LD frame; return per-channel S16 sample vectors
/// (de-interleaved). `config` is the channelConfiguration value; the
/// produced PCM channel count is `produced_channels(config)` (8 for
/// config 7 = 7.1).
fn decode_frame(sf_idx: u8, config: u8, payload: Vec<u8>) -> Vec<Vec<i16>> {
    let asc = build_ld_asc(sf_idx, config);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(config as u16);
    params.extradata = asc;
    let mut dec = oxideav_aac::decoder::make_decoder(&params)
        .expect("LD multichannel make_decoder should succeed");
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), payload);
    dec.send_packet(&pkt).unwrap();
    match dec.receive_frame().expect("LD multichannel frame") {
        Frame::Audio(af) => {
            // Interleave stride = produced channel count, derived from
            // the PCM byte length so the de-interleave matches the
            // decoder's actual output (config 7 emits 8 channels).
            let total = af.data[0].len() / 2;
            let nch = produced_channels(config);
            let n = total / nch;
            let mut out: Vec<Vec<i16>> = vec![Vec::with_capacity(n); nch];
            for (i, b) in af.data[0].chunks_exact(2).enumerate() {
                let s = i16::from_le_bytes([b[0], b[1]]);
                out[i % nch].push(s);
            }
            out
        }
        _ => panic!("expected Frame::Audio"),
    }
}

/// Helper: PCM-channel count produced by a given channelConfiguration.
fn produced_channels(config: u8) -> usize {
    match config {
        3 => 3,
        4 => 4,
        5 => 5,
        6 => 6,
        7 => 8,
        _ => unreachable!(),
    }
}

#[test]
fn ld_config3_decodes_three_channels_silent() {
    // SCE CPE.
    let mut bw = BitWriter::new();
    push_zero_sce(&mut bw);
    push_zero_cpe(&mut bw);
    let chans = decode_frame(4, 3, bw.finish());
    assert_eq!(chans.len(), produced_channels(3));
    for ch in &chans {
        assert_eq!(ch.len(), 512, "LD-512 frame per channel");
        assert!(ch.iter().all(|&s| s == 0), "zero-spec config-3 is silent");
    }
}

#[test]
fn ld_config4_decodes_four_channels_silent() {
    // SCE CPE SCE.
    let mut bw = BitWriter::new();
    push_zero_sce(&mut bw);
    push_zero_cpe(&mut bw);
    push_zero_sce(&mut bw);
    let chans = decode_frame(4, 4, bw.finish());
    assert_eq!(chans.len(), produced_channels(4));
    for ch in &chans {
        assert_eq!(ch.len(), 512);
        assert!(ch.iter().all(|&s| s == 0));
    }
}

#[test]
fn ld_config5_decodes_five_channels_silent() {
    // SCE CPE CPE.
    let mut bw = BitWriter::new();
    push_zero_sce(&mut bw);
    push_zero_cpe(&mut bw);
    push_zero_cpe(&mut bw);
    let chans = decode_frame(4, 5, bw.finish());
    assert_eq!(chans.len(), produced_channels(5));
    for ch in &chans {
        assert_eq!(ch.len(), 512);
        assert!(ch.iter().all(|&s| s == 0));
    }
}

#[test]
fn ld_config6_decodes_six_channels_silent() {
    // SCE CPE CPE LFE (5.1).
    let mut bw = BitWriter::new();
    push_zero_sce(&mut bw);
    push_zero_cpe(&mut bw);
    push_zero_cpe(&mut bw);
    push_zero_sce(&mut bw); // LFE shares SCE syntax
    let chans = decode_frame(4, 6, bw.finish());
    assert_eq!(chans.len(), produced_channels(6));
    for ch in &chans {
        assert_eq!(ch.len(), 512);
        assert!(ch.iter().all(|&s| s == 0));
    }
}

#[test]
fn ld_config7_decodes_eight_channels_silent() {
    // SCE CPE CPE CPE LFE (7.1 = 8 channels).
    let mut bw = BitWriter::new();
    push_zero_sce(&mut bw);
    push_zero_cpe(&mut bw);
    push_zero_cpe(&mut bw);
    push_zero_cpe(&mut bw);
    push_zero_sce(&mut bw); // LFE
    let chans = decode_frame(4, 7, bw.finish());
    assert_eq!(chans.len(), produced_channels(7));
    for ch in &chans {
        assert_eq!(ch.len(), 512);
        assert!(ch.iter().all(|&s| s == 0));
    }
}

#[test]
fn ld_config3_places_content_in_front_channel_only() {
    // config 3 = SCE (front-center) + CPE (L/R). Put signal in the
    // leading SCE, leave the CPE silent. The decoded output must carry
    // energy in channel 0 and silence in channels 1 and 2 — proving the
    // Table 4.19 element layout maps the first element to channel slot 0
    // and the CPE to slots 1/2.
    let mut bw = BitWriter::new();
    push_content_sce(&mut bw, 160, [1, -1, 1, -1]);
    push_zero_cpe(&mut bw);
    let chans = decode_frame(4, 3, bw.finish());
    assert_eq!(chans.len(), 3);

    let energy0: i64 = chans[0].iter().map(|&s| (s as i64).abs()).sum();
    let energy1: i64 = chans[1].iter().map(|&s| (s as i64).abs()).sum();
    let energy2: i64 = chans[2].iter().map(|&s| (s as i64).abs()).sum();
    assert!(
        energy0 > 0,
        "front SCE must carry signal (energy {energy0})"
    );
    assert_eq!(energy1, 0, "CPE-left must be silent");
    assert_eq!(energy2, 0, "CPE-right must be silent");
}

#[test]
fn ld_config5_places_content_in_surround_pair_only() {
    // config 5 = SCE + CPE(front L/R) + CPE(surround Ls/Rs). Put signal
    // only in the *second* CPE (channels 3 and 4) and verify the front
    // SCE + front CPE stay silent — confirms the base-channel offset of
    // the third element is 3 (after SCE@0 + CPE@1..2).
    let mut bw = BitWriter::new();
    push_zero_sce(&mut bw); // ch 0
    push_zero_cpe(&mut bw); // ch 1,2 silent
                            // surround CPE: build a content CPE by hand (independent ICS, ch3
                            // carries a band, ch4 silent).
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_bit(false); // common_window = 0
                         // ch3 (Ls): one codebook-1 band.
    bw.write_u32(160, 8); // global_gain
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(0, 2); // window_sequence
    bw.write_bit(false); // window_shape
    bw.write_u32(1, 6); // max_sfb = 1
    bw.write_bit(false); // predictor_data_present
    bw.write_u32(1, 4); // sect_cb = 1
    bw.write_u32(1, 5); // sect_len_incr = 1
    bw.write_bit(false); // scalefactor delta = 0
    bw.write_bit(false); // pulse
    bw.write_bit(false); // tns
    bw.write_bit(false); // gain_control
    let (code, nbits) = book1_code([1, 1, -1, -1]);
    bw.write_u32(code, nbits as u32);
    // ch4 (Rs): silent (max_sfb=0).
    bw.write_u32(0, 8); // global_gain
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(0, 2); // window_sequence
    bw.write_bit(false); // window_shape
    bw.write_u32(0, 6); // max_sfb = 0
    bw.write_bit(false); // predictor_data_present
    bw.write_bit(false); // pulse
    bw.write_bit(false); // tns
    bw.write_bit(false); // gain_control

    let chans = decode_frame(4, 5, bw.finish());
    assert_eq!(chans.len(), 5);
    let e: Vec<i64> = chans
        .iter()
        .map(|c| c.iter().map(|&s| (s as i64).abs()).sum())
        .collect();
    assert_eq!(e[0], 0, "front-center SCE silent");
    assert_eq!(e[1], 0, "front-left silent");
    assert_eq!(e[2], 0, "front-right silent");
    assert!(
        e[3] > 0,
        "surround-left must carry signal (energy {})",
        e[3]
    );
    assert_eq!(e[4], 0, "surround-right silent");
}

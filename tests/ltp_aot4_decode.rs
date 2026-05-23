//! AAC-LTP (AOT 4) full frame-decode integration test — ISO/IEC
//! 14496-3 §4.6.7 long-term prediction wired through the LC
//! `raw_data_block()` path (round-108).
//!
//! AOT 4 shares AAC-LC's GASpecificConfig and `raw_data_block()` syntax;
//! the only bitstream difference is that ics_info's
//! `predictor_data_present` body carries `ltp_data()` (Table 4.6
//! `audioObjectType != 1` branch) instead of the AAC-Main backward
//! predictor, and the §4.6.7 long-term predictor runs between PNS/IS and
//! TNS in the GA tool chain. These tests build synthetic AOT 4 SCE
//! frames by hand (no external encoder — no faad/ffmpeg LTP encoder
//! exists in tree) and verify:
//!
//!   1. `make_decoder` accepts an AOT 4 AudioSpecificConfig (previously
//!      `Error::Unsupported`).
//!   2. An AOT 4 frame with a single codebook-1 band decodes to PCM.
//!   3. With identical seed frames, a second frame whose ics_info enables
//!      `ltp_data_present` + `ltp_long_used[0]` produces *different*
//!      output than the same second frame with prediction disabled —
//!      i.e. the predictor actually adds X_est to the residual.
//!   4. The LTP-disabled AOT 4 path is byte-identical to decoding the
//!      same bitstream as AAC-LC (object types 2 and 4 agree when no
//!      prediction is signalled).

use oxideav_aac::syntax::{AOT_AAC_LC, AOT_AAC_LTP};
use oxideav_core::bits::BitWriter;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

/// AudioSpecificConfig for a GA object type (LC=2 or LTP=4), mono,
/// sf_index 4 (44.1 kHz). GASpecificConfig tail is all-zero
/// (frameLengthFlag / dependsOnCoreCoder / extensionFlag).
fn build_ga_asc(aot: u8, sf_idx: u8, ch: u8) -> Vec<u8> {
    let mut bw = BitWriter::new();
    bw.write_u32(aot as u32, 5);
    bw.write_u32(sf_idx as u32, 4);
    bw.write_u32(ch as u32, 4);
    bw.write_bit(false); // frameLengthFlag
    bw.write_bit(false); // dependsOnCoreCoder
    bw.write_bit(false); // extensionFlag
    bw.finish()
}

/// BOOK1 (4-dim, lav=1, signed) Huffman code for one [-1,1]^4 tuple,
/// indexing the spectral codebook the same way the decoder reads it:
/// `idx = (((a+1)*3 + (b+1))*3 + (c+1))*3 + (d+1)`. Returns
/// `(code, n_bits)`.
fn book1_code(tuple: [i32; 4]) -> (u32, u8) {
    use oxideav_aac::huffman_tables::{BOOK1_BITS, BOOK1_CODES};
    let mut idx = 0usize;
    for &v in &tuple {
        idx = idx * 3 + (v + 1) as usize;
    }
    (BOOK1_CODES[idx] as u32, BOOK1_BITS[idx])
}

/// Build an AOT 4 (or LC, identical body when `ltp_present == false`)
/// single_channel_element carrying one codebook-1 band (sfb 0, the four
/// lowest spectral coefficients). When `ltp_present` is true the ics_info
/// predictor body enables long-term prediction on band 0.
fn build_sce(global_gain: u8, tuple: [i32; 4], ltp_present: bool) -> Vec<u8> {
    let mut bw = BitWriter::new();
    // raw_data_block element id: 0 = ID_SCE (Table 4.71). The decoder's
    // element loop reads this 3-bit id before the element body.
    bw.write_u32(0, 3); // id_syn_ele = SCE
                        // single_channel_element()
    bw.write_u32(0, 4); // element_instance_tag
                        // individual_channel_stream -> global_gain + ics_info
    bw.write_u32(global_gain as u32, 8);
    // ics_info()
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(0, 2); // window_sequence = OnlyLong
    bw.write_bit(false); // window_shape = sine
    bw.write_u32(1, 6); // max_sfb = 1
    if ltp_present {
        bw.write_bit(true); // predictor_data_present = 1
        bw.write_bit(true); // ltp_data_present = 1
                            // ltp_data() non-LD long branch:
        bw.write_u32(1024, 11); // ltp_lag (one frame back)
        bw.write_u32(7, 3); // ltp_coef index 7 (largest gain)
        bw.write_bit(true); // ltp_long_used[0] = 1
    } else {
        bw.write_bit(false); // predictor_data_present = 0
    }
    // section_data(): one section, codebook 1, length 1 sfb.
    bw.write_u32(1, 4); // sect_cb = 1
    bw.write_u32(1, 5); // sect_len_incr = 1 (< 31, so loop ends)
                        // scale_factor_data(): one regular band -> one delta of 0
                        // (SCALEFACTOR index 60 = delta 0 = code "0", 1 bit).
    bw.write_bit(false);
    // pulse_data_present / tns_data_present / gain_control_data_present
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    // spectral_data(): band 0 = coeffs [0,4) = one BOOK1 4-tuple.
    let (code, nbits) = book1_code(tuple);
    bw.write_u32(code, nbits as u32);
    bw.finish()
}

/// Decode one mono frame and return its S16 samples.
fn decode_one(aot: u8, payload: Vec<u8>) -> Vec<i16> {
    let asc = build_ga_asc(aot, 4, 1);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(1);
    params.extradata = asc;
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make_decoder");
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), payload);
    dec.send_packet(&pkt).unwrap();
    match dec.receive_frame().expect("frame") {
        Frame::Audio(af) => af.data[0]
            .chunks_exact(2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect(),
        _ => panic!("expected audio frame"),
    }
}

/// Run two mono frames through one decoder; return the second frame's
/// S16 samples (the first frame seeds overlap + LTP history).
fn decode_two(aot: u8, frame0: Vec<u8>, frame1: Vec<u8>) -> Vec<i16> {
    let asc = build_ga_asc(aot, 4, 1);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(1);
    params.extradata = asc;
    let mut dec = oxideav_aac::decoder::make_decoder(&params).expect("make_decoder");

    let p0 = Packet::new(0, TimeBase::new(1, 44_100), frame0);
    dec.send_packet(&p0).unwrap();
    let _ = dec.receive_frame().expect("frame 0");

    let p1 = Packet::new(1, TimeBase::new(1, 44_100), frame1);
    dec.send_packet(&p1).unwrap();
    match dec.receive_frame().expect("frame 1") {
        Frame::Audio(af) => af.data[0]
            .chunks_exact(2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect(),
        _ => panic!("expected audio frame"),
    }
}

#[test]
fn aot4_make_decoder_accepts_ltp_asc() {
    let asc = build_ga_asc(AOT_AAC_LTP, 4, 1);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(1);
    params.extradata = asc;
    // Previously returned Error::Unsupported ("only AAC-LC and AAC-LD").
    assert!(
        oxideav_aac::decoder::make_decoder(&params).is_ok(),
        "AOT 4 (AAC-LTP) AudioSpecificConfig must now build a decoder"
    );
}

#[test]
fn aot4_single_band_frame_decodes_to_1024_samples() {
    let payload = build_sce(120, [1, 0, 0, 0], false);
    let pcm = decode_one(AOT_AAC_LTP, payload);
    assert_eq!(pcm.len(), 1024, "mono AAC-LTP frame is 1024 samples");
}

#[test]
fn aot4_ltp_changes_second_frame_output() {
    // Identical seed frame for both decoders -> identical overlap + LTP
    // history after frame 0. A large global_gain makes the reconstructed
    // PCM well above the S16 quantisation floor so the predictor's
    // contribution survives rounding.
    let seed = || build_sce(180, [1, 1, 1, 1], false);

    // Frame 1 carrying a non-trivial spectrum; one decoder enables LTP,
    // the other does not. Everything else (gain, spectrum) is identical.
    let frame1_no_ltp = build_sce(180, [1, 1, 1, 1], false);
    let frame1_ltp = build_sce(180, [1, 1, 1, 1], true);

    let out_no_ltp = decode_two(AOT_AAC_LTP, seed(), frame1_no_ltp);
    let out_ltp = decode_two(AOT_AAC_LTP, seed(), frame1_ltp);

    assert_eq!(out_no_ltp.len(), out_ltp.len());
    // The predictor adds X_est (the analysis-MDCT of the windowed,
    // lag-shifted, coef-scaled history) to the residual on band 0, so the
    // reconstructed spectrum — and hence the IMDCT'd PCM — must differ.
    assert!(
        out_no_ltp != out_ltp,
        "enabling ltp_long_used[0] on frame 1 must change the decoded PCM"
    );
}

#[test]
fn aot4_without_prediction_matches_lc() {
    // An AOT 4 bitstream with predictor_data_present == 0 carries the
    // exact same bits as the equivalent AAC-LC frame, and the decode
    // paths only diverge on the LTP tools (which are inert here), so the
    // PCM must be byte-identical across object types 2 and 4.
    let payload_lc = build_sce(118, [1, 0, 1, 0], false);
    let payload_ltp = build_sce(118, [1, 0, 1, 0], false);
    let pcm_lc = decode_one(AOT_AAC_LC, payload_lc);
    let pcm_ltp = decode_one(AOT_AAC_LTP, payload_ltp);
    assert_eq!(
        pcm_lc, pcm_ltp,
        "AOT 4 with no prediction must decode identically to AAC-LC"
    );
}

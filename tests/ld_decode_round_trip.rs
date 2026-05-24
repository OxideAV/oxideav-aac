//! AAC-LD (AOT 23) bitstream decode integration test.
//!
//! Builds a synthetic `er_raw_data_block()` payload per ISO/IEC 14496-3
//! §4.4.2.3 Table 4.19, hands it to the decoder via the public
//! `make_decoder` registry path, and asserts the decoded frame is the
//! expected number of samples (512 or 480) of S16 PCM.
//!
//! No `ffmpeg` or `faad` LD encoder is available; the trace doc's §11.2
//! ("AAC LD profile is not represented") documents the gap. We
//! validate the bitstream decode against deterministic
//! all-zero-spectrum frames (an `ics_info` with `max_sfb == 0` codes
//! no spectrum at all per §4.4.2.7 Table 4.50) and against a one-band
//! ZERO_HCB frame (one section covering one band, all spectrum
//! coefficients zero).
//!
//! Once a real AAC-LD encoder lands or a docs collaborator commits
//! pre-encoded LD `.m4a` fixtures, a content cross-decode test will
//! join this synthetic check.

use oxideav_aac::syntax::AOT_ER_AAC_LD;
use oxideav_core::bits::BitWriter;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

/// Build a minimal LD AudioSpecificConfig for AOT 23 with the given
/// (sf_idx, channelConfiguration, frame_length).
///
/// Format (ISO/IEC 14496-3 §1.6.2.1 + §4.4.1.1):
///   audioObjectType        (5)  = 23
///   samplingFrequencyIndex (4)
///   channelConfiguration   (4)
///   ── LDSpecificConfig ──
///   frameLengthFlag        (1)  = 0 (512) or 1 (480)
///   dependsOnCoreCoder     (1)  = 0
///   extensionFlag          (1)  = 0
fn build_ld_asc(sf_idx: u8, ch: u8, frame_length_flag: bool) -> Vec<u8> {
    let mut bw = BitWriter::new();
    bw.write_u32(23, 5);
    bw.write_u32(sf_idx as u32, 4);
    bw.write_u32(ch as u32, 4);
    bw.write_bit(frame_length_flag);
    bw.write_bit(false);
    bw.write_bit(false);
    bw.finish()
}

/// Build an `er_raw_data_block()` carrying a single SCE with
/// `max_sfb = 0` — no scalefactor bands at all, so no section_data,
/// no scalefactors, no spectral_data. Pulse / TNS / gain bits are all
/// 0. The frame produces a zero spectrum, which through the LD
/// IMDCT/overlap path emits a (cold-start) silent frame.
fn build_ld_zero_sce() -> Vec<u8> {
    let mut bw = BitWriter::new();
    // single_channel_element()
    bw.write_u32(0, 4); // element_instance_tag
                        // individual_channel_stream(common_window=0, scale_flag=0)
    bw.write_u32(0, 8); // global_gain
                        // ics_info()
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(0, 2); // window_sequence = OnlyLong
    bw.write_bit(false); // window_shape = sine
    bw.write_u32(0, 6); // max_sfb = 0
    bw.write_bit(false); // predictor_data_present = 0
                         // section_data: no bands -> outer loop is empty, no bits
                         // scale_factor_data: no bands -> empty
                         // pulse_data_present, tns_data_present, gain_control_data_present
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    // spectral_data: max_sfb == 0 -> zero coefficients, no bits.
    bw.finish()
}

#[test]
fn ld_decoder_recognises_aot23_and_emits_512_silence() {
    // sf_idx=4 (44.1 kHz), mono, 512-sample frame.
    let asc = build_ld_asc(4, 1, false);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(1);
    params.extradata = asc;
    let mut dec =
        oxideav_aac::decoder::make_decoder(&params).expect("LD-mono make_decoder should succeed");

    let payload = build_ld_zero_sce();
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), payload);
    dec.send_packet(&pkt).unwrap();
    let frame = dec
        .receive_frame()
        .expect("LD decode should produce a frame");
    match frame {
        Frame::Audio(af) => {
            // 512-sample LD frame, mono.
            assert_eq!(af.samples, 512, "LD-512 mono frame should be 512 samples");
            // 1 channel * 512 samples * 2 bytes (S16) = 1024 bytes.
            assert_eq!(af.data[0].len(), 1024);
            // All bytes should be zero (cold-start zero-spec produces
            // silence after OLA against the zero-init prev buffer).
            for &b in &af.data[0] {
                assert_eq!(b, 0, "LD cold-start zero-spec frame must be silent");
            }
        }
        _ => panic!("expected Frame::Audio"),
    }
}

#[test]
fn ld_decoder_recognises_aot23_with_480_frame() {
    // sf_idx=3 (48 kHz), mono, 480-sample frame.
    let asc = build_ld_asc(3, 1, true);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(48_000);
    params.channels = Some(1);
    params.extradata = asc;
    let mut dec = oxideav_aac::decoder::make_decoder(&params)
        .expect("LD-480 mono make_decoder should succeed");

    let payload = build_ld_zero_sce();
    let pkt = Packet::new(0, TimeBase::new(1, 48_000), payload);
    dec.send_packet(&pkt).unwrap();
    let frame = dec
        .receive_frame()
        .expect("LD-480 decode should produce a frame");
    match frame {
        Frame::Audio(af) => {
            assert_eq!(af.samples, 480, "LD-480 mono frame should be 480 samples");
            // 1 channel * 480 samples * 2 bytes (S16) = 960 bytes.
            assert_eq!(af.data[0].len(), 960);
        }
        _ => panic!("expected Frame::Audio"),
    }
}

#[test]
fn ld_decoder_stereo_512_emits_two_channels() {
    // sf_idx=4 (44.1 kHz), stereo, 512-sample frame.
    let asc = build_ld_asc(4, 2, false);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(2);
    params.extradata = asc;
    let mut dec = oxideav_aac::decoder::make_decoder(&params)
        .expect("LD-512 stereo make_decoder should succeed");

    // Build a CPE frame: element_instance_tag(4) + common_window(1) +
    // two independent ICS streams, each with max_sfb=0.
    let mut bw = BitWriter::new();
    // channel_pair_element()
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_bit(false); // common_window = 0 (independent ICS per ch)
                         // individual_channel_stream(common_window=0, scale_flag=0) × 2
    for _ in 0..2 {
        bw.write_u32(0, 8); // global_gain
        bw.write_bit(false); // ics_reserved_bit
        bw.write_u32(0, 2); // window_sequence = OnlyLong
        bw.write_bit(false); // window_shape = sine
        bw.write_u32(0, 6); // max_sfb = 0
        bw.write_bit(false); // predictor_data_present = 0
        bw.write_bit(false); // pulse
        bw.write_bit(false); // tns
        bw.write_bit(false); // gain_control
    }
    let payload = bw.finish();
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), payload);
    dec.send_packet(&pkt).unwrap();
    let frame = dec
        .receive_frame()
        .expect("LD-512 CPE decode should succeed");
    match frame {
        Frame::Audio(af) => {
            assert_eq!(af.samples, 512);
            // 2 channels * 512 samples * 2 bytes = 2048 bytes interleaved.
            assert_eq!(af.data[0].len(), 2048);
            for &b in &af.data[0] {
                assert_eq!(b, 0, "LD-512 CPE zero-spec frame must be silent");
            }
        }
        _ => panic!("expected Frame::Audio"),
    }
}

#[test]
fn ld_decoder_stereo_common_window_512() {
    // CPE with common_window=1, ms_mask_present=0, max_sfb=0.
    let asc = build_ld_asc(4, 2, false);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(2);
    params.extradata = asc;
    let mut dec = oxideav_aac::decoder::make_decoder(&params).unwrap();
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_bit(true); // common_window = 1
                        // shared ics_info()
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(0, 2); // window_sequence = OnlyLong
    bw.write_bit(false); // window_shape
    bw.write_u32(0, 6); // max_sfb = 0
    bw.write_bit(false); // predictor_data_present
    bw.write_u32(0, 2); // ms_mask_present = 0
                        // No per-band ms_used bits (mask=0).
                        // Two individual_channel_stream(common_window=1, scale_flag=0)
                        // — they omit ics_info() but keep global_gain + sections +
                        // pulse/tns/gain bits + spectrum.
    for _ in 0..2 {
        bw.write_u32(0, 8); // global_gain
                            // Common_window suppresses ics_info; section/scalefactor/
                            // pulse/tns/gain follow directly.
        bw.write_bit(false); // pulse
        bw.write_bit(false); // tns
        bw.write_bit(false); // gain_control
    }
    let payload = bw.finish();
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), payload);
    dec.send_packet(&pkt).unwrap();
    let frame = dec.receive_frame().expect("LD CPE common_window decode");
    if let Frame::Audio(af) = frame {
        assert_eq!(af.samples, 512);
        for &b in &af.data[0] {
            assert_eq!(b, 0);
        }
    } else {
        panic!();
    }
}

#[test]
fn ld_decoder_two_frames_produce_continuous_silence() {
    // Two zero-spec frames in a row — second frame's OLA against the
    // previous frame's stash (already zero) must again be silent.
    // This validates the LdChannelState.prev mechanism persists across
    // send_packet / receive_frame calls.
    let asc = build_ld_asc(4, 1, false);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(1);
    params.extradata = asc;
    let mut dec = oxideav_aac::decoder::make_decoder(&params).unwrap();
    for _ in 0..2 {
        let payload = build_ld_zero_sce();
        let pkt = Packet::new(0, TimeBase::new(1, 44_100), payload);
        dec.send_packet(&pkt).unwrap();
        let frame = dec.receive_frame().expect("LD decode frame");
        match frame {
            Frame::Audio(af) => {
                assert_eq!(af.samples, 512);
                for &b in &af.data[0] {
                    assert_eq!(b, 0);
                }
            }
            _ => panic!(),
        }
    }
}

#[test]
fn ld_decoder_rejects_aot23_with_reserved_channel_config() {
    // channelConfiguration 1..=7 are now all decoded (§4.4.2.3
    // Table 4.19). Config 0 (PCE-defined) and 8.. (reserved) remain
    // unsupported — verify the up-front error still fires for a
    // reserved config rather than during decode. (Round-111 lifted the
    // earlier 1..=2-only gate that previously rejected config 6 here.)
    let asc = build_ld_asc(4, 8, false);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(8);
    params.extradata = asc;
    let res = oxideav_aac::decoder::make_decoder(&params);
    assert!(
        res.is_err(),
        "LD with reserved channelConfiguration=8 must error out"
    );
}

/// Build an LTP-bearing SCE frame: `predictor_data_present = 1`,
/// `ltp_data_present = 1`, then `ltp_data()` (ER AAC LD branch) with the
/// given lag / coef / per-sfb `long_used` flags, followed by a
/// single-band ZERO_HCB spectrum. Exercises the `ltp_data()` parse +
/// cursor alignment through the rest of the channel element.
///
/// Layout (§4.4.2.2 + §4.6.2 Table 4.6 + §4.6.7 Table 4.49):
///   element_instance_tag (4)
///   global_gain          (8)
///   ── ics_info() ──
///   ics_reserved_bit     (1)
///   window_sequence      (2)  = OnlyLong
///   window_shape         (1)
///   max_sfb              (6)  = 1
///   predictor_data_present (1) = 1
///     ltp_data_present   (1)  = 1
///       ltp_lag_update   (1)  = 1
///       ltp_lag          (10)
///       ltp_coef         (3)
///       ltp_long_used[0] (1)
///   ── section_data ── one section: ZERO_HCB covering the 1 band
///   ── scalefactors ── (none for ZERO_HCB)
///   pulse / tns / gain bits (1 + 1 + 1) = 0
///   ── spectral_data ── ZERO_HCB ⇒ no bits
fn build_ld_ltp_sce(lag: u16, coef_idx: u8, long_used0: bool) -> Vec<u8> {
    let mut bw = BitWriter::new();
    bw.write_u32(0, 4); // element_instance_tag
    bw.write_u32(0, 8); // global_gain
                        // ics_info()
    bw.write_bit(false); // ics_reserved_bit
    bw.write_u32(0, 2); // window_sequence = OnlyLong
    bw.write_bit(false); // window_shape = sine
    bw.write_u32(1, 6); // max_sfb = 1
    bw.write_bit(true); // predictor_data_present = 1
    bw.write_bit(true); // ltp_data_present = 1
                        // ltp_data() — ER AAC LD branch
    bw.write_bit(true); // ltp_lag_update = 1
    bw.write_u32(lag as u32, 10); // ltp_lag
    bw.write_u32(coef_idx as u32, 3); // ltp_coef
    bw.write_bit(long_used0); // ltp_long_used[0]
                              // section_data: one section, ZERO_HCB (cb=0), length 1.
    bw.write_u32(0, 4); // sect_cb = ZERO_HCB
    bw.write_u32(1, 5); // sect_len_incr = 1 (one band)
                        // scalefactors: ZERO_HCB band carries no scalefactor.
                        // pulse / tns / gain_control
    bw.write_bit(false);
    bw.write_bit(false);
    bw.write_bit(false);
    // spectral_data: ZERO_HCB ⇒ no coefficients coded.
    bw.finish()
}

#[test]
fn ld_decoder_consumes_ltp_data_block() {
    // The decoder must parse predictor_data_present=1 + ltp_data() and
    // keep the bit-cursor aligned through the rest of the SCE. With a
    // ZERO_HCB spectrum and cold-start (all-zero) history the LTP
    // prediction is zero, so the frame decodes to silence — the point of
    // the test is that the new ltp_data() parse path is reached and the
    // trailing section/pulse/tns/gain bits land at the correct offsets.
    let asc = build_ld_asc(4, 1, false);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(1);
    params.extradata = asc;
    let mut dec = oxideav_aac::decoder::make_decoder(&params)
        .expect("LD-mono LTP make_decoder should succeed");

    let payload = build_ld_ltp_sce(300, 5, true);
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), payload);
    dec.send_packet(&pkt).unwrap();
    let frame = dec
        .receive_frame()
        .expect("LD LTP frame should decode without cursor misalignment");
    match frame {
        Frame::Audio(af) => {
            assert_eq!(af.samples, 512);
            assert_eq!(af.data[0].len(), 1024);
            // Cold-start zero history ⇒ zero prediction ⇒ silence.
            for &b in &af.data[0] {
                assert_eq!(b, 0, "cold-start LTP frame must be silent");
            }
        }
        _ => panic!("expected Frame::Audio"),
    }
}

#[test]
fn ld_decoder_ltp_disabled_band_decodes() {
    // ltp_long_used[0] = 0: prediction parsed but not applied to the band.
    // Still must decode cleanly (cursor alignment with the flag cleared).
    let asc = build_ld_asc(4, 1, false);
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(1);
    params.extradata = asc;
    let mut dec = oxideav_aac::decoder::make_decoder(&params).unwrap();
    let payload = build_ld_ltp_sce(0, 0, false);
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), payload);
    dec.send_packet(&pkt).unwrap();
    let frame = dec.receive_frame().expect("LTP-disabled band must decode");
    if let Frame::Audio(af) = frame {
        assert_eq!(af.samples, 512);
    } else {
        panic!("expected Frame::Audio");
    }
}

#[test]
fn ld_decoder_aot_constant_is_23() {
    // Cross-check the syntax module constant; if this ever changes the
    // ASC builder above must change too.
    assert_eq!(AOT_ER_AAC_LD, 23);
}

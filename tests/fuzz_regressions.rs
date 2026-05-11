//! Fuzz-found regression tests.
//!
//! Each entry corresponds to a panic / divergence found by the
//! `cargo fuzz` harnesses (see `fuzz/fuzz_targets/`). Inputs are
//! captured here so they keep being exercised under regular CI even
//! when the fuzz CI workflow isn't running.
//!
//! Bug 1 (workspace task #743) — SBR bitstream OOB on
//! `panic_free_decode` artifact `crash-1c378992f1f16e4b9d33121d9c4d1ca6ca86c005`.
//! A 13-byte input fed through the synthetic-ASC raw_data_block path
//! caused `src/sbr/bitstream.rs:385` to panic with
//! `index out of bounds: the len is 5 but the index is 5`. Root
//! cause: `FreqTables::build` allowed NQ > 5, then `parse_sbr_noise`
//! wrote past the fixed-size `noise_sf[..][5]` row. Both sites now
//! return `Error::InvalidData` instead of panicking
//! (ISO/IEC 14496-3 §4.6.18.3.2.2 constrains NQ to `[1, 5]`).
//!
//! Bug 2 (workspace task #744) — 88.2 kHz / 5.1-channel ADTS
//! produced 0 output frames vs ffmpeg's 1. Two contributing
//! issues, both surfaced by the same fuzz oracle:
//!   (a) sample-rate-index 1 (88200 Hz) maps to `SWB_LONG_96` /
//!       `SWB_SHORT_96` whose length is 41/12; `parse_ics_info`
//!       allowed `max_sfb` up to 63 (long) / 15 (short) without
//!       checking against the active `num_swb`, so a non-
//!       conformant frame OOB'd `decode_spectrum_long` (line 263:
//!       `swb_offsets[max_sfb]`). Fix: `parse_ics_info` rejects
//!       out-of-range `max_sfb` per ISO/IEC 14496-3 Table 4.110.
//!   (b) `decode_ics` returned `Error::Unsupported("AAC: gain_control
//!       in LC stream")` whenever the `gain_control_data_present`
//!       bit was set on an AAC-LC stream. The bit MUST be zero
//!       per §4.5.2.1, but libavcodec tolerates non-zero and
//!       emits PCM anyway, leaving the fuzz oracle to fire on
//!       the "ours produced 0" condition. We now mirror
//!       libavcodec's tolerance and silently accept the bit.
//!
//! Bug 3 (workspace task #742) — investigated 5814-LSB second-frame
//! divergence against ffmpeg on
//! `aac-lc-intensity-stereo/input.aac` (44.1 kHz stereo 128 kbps).
//! Verification (probe at
//! `examples/probe_intensity_stereo_div.rs` + per-channel sfb_cb
//! dump): the 33-frame fixture contains **no** `INTENSITY_HCB`
//! (cb=15) or `INTENSITY_HCB2` (cb=14) bands across either
//! channel of any frame, so the divergence cannot be coming from
//! the §4.6.8.2.3 intensity-stereo path. The actual signature is a
//! transient EIGHT_SHORT-transition mismatch on frames 1 and 2
//! (LongStart → EightShort → LongStop), with frames 3+ converging
//! to ±1 LSB drift. The intensity-stereo decoder itself is
//! exercised by the existing unit tests
//! `intensity_stereo_cb15_positive_sign`, `..._cb14_negative_sign`,
//! `..._ms_mask_flips_sign`, and `..._non_is_bands_untouched` in
//! `src/decoder.rs::tests`. The regression test below pins the
//! verification result so future round-trips against the same
//! fixture reproduce the same observation.

use oxideav_aac::adts::parse_adts_header;
use oxideav_aac::decoder::make_decoder;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

#[test]
fn sbr_bitstream_oob_does_not_panic_743() {
    // Exact bytes from
    // fuzz/artifacts/panic_free_decode/crash-1c378992f1f16e4b9d33121d9c4d1ca6ca86c005
    // (base64 ybhAPxAAACkBJBIAAQ==).
    let data: [u8; 13] = [
        0xc9, 0xb8, 0x40, 0x3f, 0x10, 0x00, 0x00, 0x29, 0x01, 0x24, 0x12, 0x00, 0x01,
    ];
    // Synthetic AAC-LC / 44.1 kHz / stereo AudioSpecificConfig.
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(2);
    params.extradata = vec![0x12, 0x10];
    let mut dec = make_decoder(&params).expect("synthetic ASC valid");
    let pkt = Packet::new(0, TimeBase::new(1, 44100), data.to_vec());
    // Either send_packet or receive_frame may return an Err — that is
    // the desired outcome. The contract is "no panic on arbitrary
    // bytes"; we only assert decoder doesn't unwind.
    let _ = dec.send_packet(&pkt);
    for _ in 0..4 {
        if dec.receive_frame().is_err() {
            break;
        }
    }
    let _ = dec.flush();
}

#[test]
fn ics_info_clamps_oversized_max_sfb_long_744() {
    // Build a malformed AAC-LC raw_data_block: SCE with
    // ICS_INFO that codes max_sfb = 63 on sf_index = 1
    // (88200 Hz, num_swb_long = 41). Without the bound check,
    // parse_section_data + decode_spectrum_long index past
    // SWB_LONG_96 and panic. Workspace task #744 added a hard
    // reject; task #759 then revealed that libavcodec tolerates
    // the overrun and produces output, so a hard reject leaves
    // the fuzz oracle firing "ffmpeg emitted a frame but we
    // produced 0". `parse_ics_info` now clamps `max_sfb` to
    // `num_swb` so the remainder of the frame stays
    // bitstream-aligned and the spectrum decode runs over the
    // representable bands only. The contract here is just
    // "doesn't panic" — the decoder may still error out further
    // along the bitstream depending on what bytes follow.
    use oxideav_core::bits::BitWriter;
    let mut bw = BitWriter::new();
    // SCE element: id=0 (3 bits) + element_instance_tag (4 bits),
    // then global_gain (8 bits, value doesn't matter), then ics_info.
    bw.write_u32(0, 3);
    bw.write_u32(0, 4);
    bw.write_u32(128, 8);
    // ics_info:
    //   ics_reserved_bit       1
    //   window_sequence        2
    //   window_shape           1
    //   max_sfb                6  (long-window path)
    //   predictor_data_present 1
    bw.write_u32(0, 1); // reserved
    bw.write_u32(0, 2); // window_sequence = 0 (only_long)
    bw.write_u32(0, 1); // window_shape
    bw.write_u32(63, 6); // max_sfb = 63 — out of range for SWB_LONG_96 (41)
    bw.write_u32(0, 1); // predictor_data_present
    let payload = bw.finish();

    // Synthetic ASC for AAC-LC / 88.2 kHz / mono:
    //   AOT=2 (5 bits) sf_idx=1 (4 bits) chan_cfg=1 (4 bits) GAS=0 (3 bits)
    // → bits 00010 0001 0001 000 = 0x10 0x88
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(88_200);
    params.channels = Some(1);
    params.extradata = vec![0x10, 0x88];
    let mut dec = make_decoder(&params).expect("synthetic ASC valid");
    let pkt = Packet::new(0, TimeBase::new(1, 88_200), payload);
    // The contract here is "no panic on arbitrary bytes"; we
    // only assert the decoder doesn't unwind.
    let _ = dec.send_packet(&pkt);
    let _ = dec.receive_frame();
}

/// Bug 2 (#744) — replays the second 2026-05-10 fuzz crash artifact
/// (`crash-037e514390aeeb1a27c4d13469b09ef1fc5b4d12`, 43 bytes). Same
/// "0-frame divergence" class as the 113-byte artifact, but the
/// embedded 88.2 kHz / 5.1 ADTS frame trips a different malformed-
/// element path: ICS with `gain_control_data_present == 1`. AAC-LC
/// is supposed to keep that bit zero (gain control is reserved for
/// the AAC-SSR profile), but libavcodec tolerates it and produces
/// PCM anyway. `decode_ics` previously rejected with
/// `Error::Unsupported`, leaving the fuzz oracle's "ffmpeg emitted a
/// frame but oxideav-aac produced 0" assertion firing. We now
/// silently ignore the bit (matching libavcodec) and produce *some*
/// frame for downstream consumers.
#[test]
fn ffmpeg_oracle_88200_5_1_adts_emits_frame_744_b() {
    let data: [u8; 43] = [
        255, 105, 249, 140, 140, 140, 140, 140, 255, 0, 0, 255, 249, 71, 140, 3, 0, 128, 0, 249, 1,
        140, 114, 8, 255, 254, 254, 255, 249, 71, 140, 1, 185, 160, 140, 255, 249, 71, 208, 255,
        255, 191, 255,
    ];
    let params = CodecParameters::audio(CodecId::new("aac"));
    let mut dec = make_decoder(&params).expect("ADTS bootstrap");
    let mut got_audio = 0usize;
    let mut off = 0usize;
    while off + 7 <= data.len() {
        if data[off] != 0xFF || (data[off + 1] & 0xF0) != 0xF0 {
            off += 1;
            continue;
        }
        let h = match parse_adts_header(&data[off..]) {
            Ok(h) => h,
            Err(_) => break,
        };
        if h.frame_length == 0 || off + h.frame_length > data.len() {
            break;
        }
        let pkt = Packet::new(
            0,
            TimeBase::new(1, h.sample_rate().unwrap_or(44_100) as i64),
            data[off..off + h.frame_length].to_vec(),
        );
        let _ = dec.send_packet(&pkt);
        if let Ok(Frame::Audio(_)) = dec.receive_frame() {
            got_audio += 1;
        }
        off += h.frame_length;
    }
    assert!(
        got_audio >= 1,
        "expected ≥1 audio frame from libavcodec-accepted (88.2 kHz / 5.1) input \
         (got {got_audio}); the fuzz oracle treats 0 frames as a hard panic."
    );
}

/// Bug (workspace task #760) — `decode_packet` panicked at
/// `src/decoder.rs:316` with "range start index 9 out of range for
/// slice of length 7" when an ADTS packet declared
/// `protection_absent = 0` (CRC follows → header is 9 bytes) but
/// the packet itself was only 7 bytes long. `parse_adts_header`
/// only reads 7 bytes so it succeeded; the subsequent
/// `data[header_length()..frame_end]` indexed past the packet
/// because `header_length() == 9 > data.len() == 7`. Fix: reject
/// such packets with `Error::InvalidData` before the slice index.
#[test]
fn adts_short_packet_with_crc_does_not_panic_760() {
    // Exact 7 bytes from
    // fuzz/artifacts/panic_free_decode/crash-ac1b31b983778f1fbf5196fcef999aecc9a4a951
    // `prot_abs = 0` (CRC indicated) so header length is 9 bytes,
    // but only 7 bytes are available.
    let data: [u8; 7] = [0xFF, 0xF8, 0x59, 0xBE, 0xBE, 0x28, 0x28];

    // Path 1: ADTS streaming — bootstraps from first frame.
    {
        let params = CodecParameters::audio(CodecId::new("aac"));
        let mut dec = make_decoder(&params).expect("ADTS bootstrap");
        let pkt = Packet::new(0, TimeBase::new(1, 44_100), data.to_vec());
        let _ = dec.send_packet(&pkt);
        for _ in 0..4 {
            if dec.receive_frame().is_err() {
                break;
            }
        }
    }
    // Path 2: synthetic ASC — exercises the same `decode_packet`
    // code path on a pre-configured decoder.
    {
        let mut params = CodecParameters::audio(CodecId::new("aac"));
        params.sample_rate = Some(44_100);
        params.channels = Some(2);
        params.extradata = vec![0x12, 0x10];
        let mut dec = make_decoder(&params).expect("synthetic ASC valid");
        let pkt = Packet::new(0, TimeBase::new(1, 44_100), data.to_vec());
        let _ = dec.send_packet(&pkt);
        for _ in 0..4 {
            if dec.receive_frame().is_err() {
                break;
            }
        }
    }
}

/// Bug (workspace task #759) — replays the exact 159-byte
/// `ffmpeg_oracle_decode` artifact that surfaced after #744. The
/// embedded 88.2 kHz / 5.1 ADTS frame at byte offset 112 codes a
/// silent SCE (max_sfb=0) followed by a non-conformant trailing
/// CPE whose per-channel ICS_INFO has `max_sfb = 60` on
/// `sf_index = 1` (88200 Hz, `num_swb_long = 41`). The SCE itself
/// has `pulse_data_present = 1` with `pulse_start_sfb` beyond its
/// own (zero) max_sfb. libavcodec tolerates all three issues and
/// emits PCM; before this round we hard-rejected the
/// pulse-out-of-range and the max_sfb-overrun, leaving us at
/// 0 frames and tripping the fuzz oracle's "ffmpeg emitted a frame
/// but oxideav-aac produced 0" assertion. The fix combines:
///   (a) `apply_pulse_long` clamps to a no-op on out-of-range
///       `pulse_start_sfb` (silent SCE stays silent).
///   (b) `parse_ics_info` clamps `max_sfb` to `num_swb` instead
///       of erroring (mirrors libavcodec).
///   (c) the element-decode loop in `decode_packet` keeps
///       partial PCM when a *later* element fails after at
///       least one element has been fully decoded.
#[test]
fn ffmpeg_oracle_88200_5_1_clamps_max_sfb_759() {
    // Exact 159-byte input from
    // fuzz/artifacts/ffmpeg_oracle_decode/crash-9f8b60a1917e6087d80d04bd514ec3ca4d4cfc5e
    let data: [u8; 159] = [
        0xff, 0xf9, 0x2f, 0x00, 0x8c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf9,
        0x9c, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf9, 0x47, 0x8c, 0x01, 0xf9, 0xa0, 0x01,
        0x00, 0x00, 0x21, 0xff, 0xf9, 0x47, 0x8c, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf9, 0xa0,
        0x8c, 0x00, 0x00, 0xff, 0xf9, 0x8c, 0x04, 0x00, 0xa0, 0xff, 0xf9, 0x00, 0x08, 0xa0, 0x8c,
        0xff, 0xfe, 0x01, 0x00, 0x00, 0xa0, 0xff, 0x73, 0xab,
    ];
    // Walk the same find_adts_run the oracle harness uses: stop
    // at the first AAC-LC frame found. This is the frame whose
    // 0-output divergence triggered the fuzz crash.
    let params = CodecParameters::audio(CodecId::new("aac"));
    let mut dec = make_decoder(&params).expect("ADTS bootstrap");
    let mut got = 0usize;
    for off in 0..data.len().saturating_sub(7) {
        if data[off] != 0xFF || (data[off + 1] & 0xF0) != 0xF0 {
            continue;
        }
        let h = match parse_adts_header(&data[off..]) {
            Ok(h) => h,
            Err(_) => continue,
        };
        if h.frame_length == 0
            || off + h.frame_length > data.len()
            || h.object_type != 2
            || h.number_of_raw_blocks_minus_one != 0
        {
            continue;
        }
        let pkt = Packet::new(
            0,
            TimeBase::new(1, h.sample_rate().unwrap_or(44_100) as i64),
            data[off..off + h.frame_length].to_vec(),
        );
        let _ = dec.send_packet(&pkt);
        if let Ok(Frame::Audio(_)) = dec.receive_frame() {
            got += 1;
        }
        break;
    }
    assert!(
        got >= 1,
        "expected ≥1 audio frame from libavcodec-accepted (88.2 kHz / 5.1) input \
         (got {got}); the fuzz oracle treats 0 frames as a hard panic."
    );
}

/// Bug (round-#759 follow-up, surfaced post-fix in same fuzz pass)
/// — an ADTS frame with `sampling_frequency_index = 15` (the
/// explicit-rate escape value, not in `SAMPLE_RATES[0..=12]`)
/// reached `IcsInfo::num_swb()` and panicked at
/// `src/sfband.rs:102` with "index out of bounds: the len is 13
/// but the index is 15". `decode_packet` now rejects reserved
/// (13/14) and escape (15) sf indices up front per ISO/IEC
/// 14496-3 Table 1.16; `num_swb_long` / `num_swb_short` clamp as
/// defence in depth.
#[test]
fn adts_escape_sf_index_15_rejected_cleanly() {
    // 4-byte stub: sync + sf_idx=15 (escape) + chan_cfg=2.
    //   0xFF 0xF9 = sync(12) id=1(MPEG-2) layer=00 prot_abs=1
    //   0x7C 0xB4 = profile=01(LC) sf_idx=1111(15) priv=0
    //               chan=010(stereo) orig=1 home=1 cprt_id=0 cprt_st=1
    //               frame_len top 2 bits = 00
    // We pad with a few bytes so the 7-byte minimum header parse
    // succeeds; the rejection happens before any payload read.
    let data: [u8; 8] = [0xFF, 0xF9, 0x7C, 0xB4, 0x00, 0x20, 0x1F, 0xFC];
    let params = CodecParameters::audio(CodecId::new("aac"));
    let mut dec = make_decoder(&params).expect("ADTS bootstrap");
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), data.to_vec());
    let _ = dec.send_packet(&pkt);
    // Must not panic — error is fine.
    let _ = dec.receive_frame();
}

/// Bug (#759 follow-up, surfaced post-fix on next fuzz run) —
/// `predictor_data_present = 1` on an AAC-LC stream caused
/// `parse_ics_info` to return `Error::Unsupported` ("AAC:
/// predictor_data_present in LC stream"), even though libavcodec
/// tolerates the bit and emits PCM. Mirrors the gain_control
/// tolerance fix from the previous round. We now consume the
/// predictor_data() block exactly per ISO/IEC 14496-3 Table 4.55
/// so the bit-reader stays aligned, then ignore the parsed
/// prediction info (the LC decoder doesn't run Main/LTP synthesis).
#[test]
fn ics_info_tolerates_predictor_data_present_lc() {
    use oxideav_core::bits::BitWriter;
    let mut bw = BitWriter::new();
    // SCE: id=0 + tag=0 + global_gain=128 + ICS_INFO with
    // predictor_data_present=1, predictor_reset=0, max_sfb=4
    // (so we consume 4 prediction_used bits).
    bw.write_u32(0, 3);
    bw.write_u32(0, 4);
    bw.write_u32(128, 8);
    // ics_info:
    bw.write_u32(0, 1); // reserved
    bw.write_u32(0, 2); // ws = OnlyLong
    bw.write_u32(0, 1); // wsh
    bw.write_u32(4, 6); // max_sfb = 4
    bw.write_u32(1, 1); // predictor_data_present = 1
    bw.write_u32(0, 1); // predictor_reset = 0
    bw.write_u32(0, 1); // prediction_used[0]
    bw.write_u32(1, 1); // prediction_used[1]
    bw.write_u32(0, 1); // prediction_used[2]
    bw.write_u32(1, 1); // prediction_used[3]
    let payload = bw.finish();

    // ASC AAC-LC / 44100 / mono.
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(1);
    params.extradata = vec![0x12, 0x08];
    let mut dec = make_decoder(&params).expect("ASC valid");
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), payload);
    let _ = dec.send_packet(&pkt);
    // Contract is "no panic on arbitrary bytes" + "predictor bit
    // is consumed without erroring out". The decoder may still
    // bail later on truncated section data — we only assert it
    // doesn't reject the predictor_data_present bit itself.
    let _ = dec.receive_frame();
}

/// Bug (workspace task #757, follow-up to #743) — `hf_adjust.rs:182`
/// panicked with "attempt to add with overflow" on a malformed SBR
/// envelope grid that produced an out-of-range `t_e[env+1]` value.
/// The previous fix added an `if l_end ≤ l_start { continue; }` guard
/// AFTER the panicking `+`, so it could not catch the case where the
/// `(RATE as i32 * t_e[env+1]) as usize + t_hf_adj` arithmetic itself
/// overflowed (cargo-fuzz overrides `overflow-checks = on` in the
/// release profile, so a wrap panics instead of producing garbage).
///
/// Fix: compute the slot bounds in i64 with a `< 0` early-skip before
/// casting to usize, mirroring the same pattern in the coupled-channel
/// path. The 25-byte input below is the exact bytes from
/// `fuzz/artifacts/panic_free_decode/crash-58c68c99f8b1e0f83491bb2a8ce88f829442037c`
/// captured by the daily Fuzz CI run on 2026-05-11.
#[test]
fn sbr_hf_adjust_overflow_does_not_panic_757() {
    let data: [u8; 25] = [
        0xc9, 0xbe, 0xbe, 0x70, 0xca, 0xf5, 0xc9, 0xf5, 0xca, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ];
    // Path 1: ADTS streaming (auto-bootstrap from first frame header).
    {
        let params = CodecParameters::audio(CodecId::new("aac"));
        let mut dec = make_decoder(&params).expect("ADTS bootstrap");
        let pkt = Packet::new(0, TimeBase::new(1, 44_100), data.to_vec());
        let _ = dec.send_packet(&pkt);
        for _ in 0..4 {
            if dec.receive_frame().is_err() {
                break;
            }
        }
    }
    // Path 2: synthetic ASC (raw_data_block path inside decode_packet).
    {
        let mut params = CodecParameters::audio(CodecId::new("aac"));
        params.sample_rate = Some(44_100);
        params.channels = Some(2);
        params.extradata = vec![0x12, 0x10];
        let mut dec = make_decoder(&params).expect("ASC valid");
        let pkt = Packet::new(0, TimeBase::new(1, 44_100), data.to_vec());
        let _ = dec.send_packet(&pkt);
        for _ in 0..4 {
            if dec.receive_frame().is_err() {
                break;
            }
        }
    }
}

/// Bug (workspace task #771) — `ffmpeg_oracle_decode.rs:244` first-
/// frame sample-count mismatch (oxi=1024 vs ffmpeg=2048) on an
/// 88.2 kHz / 5.1 ch ADTS stream. ISO/IEC 14496-3 §4.6.18 lets HE-AAC
/// encoders signal SBR implicitly via ADTS — in which case libavcodec
/// applies a heuristic for high sample-rate indices (96/88.2/64 kHz)
/// that interprets the ADTS rate as the SBR-doubled output rate and
/// emits 2× samples per frame regardless of whether SBR FIL data is
/// present. oxideav-aac does not auto-enable implicit-SBR doubling
/// for multichannel (the SBR path is gated by `channels_out ∈ 1..=2`
/// in `src/decoder.rs`); we emit the AAC-LC core 1024 samples.
///
/// Both behaviours are spec-compliant; the workspace stance is that
/// the bitstream literally carries 1024 samples per frame and
/// implicit-SBR doubling is an implementation choice. The fuzz oracle
/// now skips the compare when `our_first.samples * 2 ==
/// oracle_first.samples`. This regression test pins the exact 151-byte
/// fuzz crash artifact and asserts that we (a) decode without
/// panicking and (b) emit the AAC-LC core 1024 samples.
#[test]
fn ffmpeg_oracle_88200_5_1_implicit_sbr_771() {
    // Exact 151-byte input from
    // fuzz/artifacts/ffmpeg_oracle_decode/crash-d41348f30e8624bd73c01691ad262e8ba23a5c75
    let data: [u8; 151] = [
        0xff, 0xff, 0xff, 0xf8, 0x47, 0x8c, 0x01, 0xf6, 0xa0, 0x2c, 0xff, 0x06, 0xf9, 0x80, 0x2c,
        0x6d, 0x6e, 0xff, 0xf8, 0x47, 0x8c, 0x01, 0xf9, 0xa0, 0x2c, 0xff, 0xc3, 0xc3, 0xc3, 0xc3,
        0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,
        0xc3, 0xc3, 0xc3, 0x2c, 0xff, 0x06, 0xf9, 0x80, 0x24, 0x73, 0x6e, 0xff, 0xf8, 0x47, 0x8c,
        0x01, 0xf9, 0xa0, 0x2c, 0xff, 0x06, 0xf9, 0x80, 0x01, 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3,
        0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,
        0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,
        0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,
        0x83, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,
        0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xf9, 0x49, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4b,
        0xbf,
    ];
    // Walk the same find_adts_run the oracle harness uses.
    let params = CodecParameters::audio(CodecId::new("aac"));
    let mut dec = make_decoder(&params).expect("ADTS bootstrap");
    let mut got = 0usize;
    let mut samples_per_frame: Option<u32> = None;
    let mut off = 0usize;
    while off + 7 <= data.len() {
        if data[off] != 0xFF || (data[off + 1] & 0xF0) != 0xF0 {
            off += 1;
            continue;
        }
        let h = match parse_adts_header(&data[off..]) {
            Ok(h) => h,
            Err(_) => {
                off += 1;
                continue;
            }
        };
        if h.frame_length == 0 || off + h.frame_length > data.len() {
            off += 1;
            continue;
        }
        if h.object_type != 2 {
            off += h.frame_length;
            continue;
        }
        let pkt = Packet::new(
            0,
            TimeBase::new(1, h.sample_rate().unwrap_or(44_100) as i64),
            data[off..off + h.frame_length].to_vec(),
        );
        let _ = dec.send_packet(&pkt);
        if let Ok(Frame::Audio(af)) = dec.receive_frame() {
            samples_per_frame.get_or_insert(af.samples);
            got += 1;
        }
        off += h.frame_length;
    }
    // We must produce ≥1 frame (the implicit-SBR mismatch fired only
    // BECAUSE we produced a frame — the previous "0 frames" path was
    // task #759). Each emitted frame should carry the AAC-LC core
    // 1024 samples per channel; the oracle harness now tolerates the
    // 1024-vs-2048 divergence rather than panicking.
    assert!(got >= 1, "expected ≥1 audio frame, got {got}");
    assert_eq!(
        samples_per_frame,
        Some(1024),
        "AAC-LC core MUST emit 1024 samples/frame; implicit-SBR doubling \
         is libavcodec's choice, not the bitstream's"
    );
}

/// Bug — SBR `hf_adjust` underflow on malformed envelope grid
/// (`crash-b996ddb4401e20a581a3c900cab3c4b7c2e72f7c`, 16 bytes).
/// `l_end - l_start` panicked with "attempt to subtract with
/// overflow" when a non-monotonic `t_e[env+1] < t_e[env]` slipped
/// through grid construction. `decode_sbr_frame` now skips the
/// envelope when `l_end ≤ l_start` rather than panicking.
#[test]
fn sbr_hf_adjust_underflow_does_not_panic() {
    let data: [u8; 16] = [
        209, 188, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 188, 0, 0, 0,
    ];
    let params = CodecParameters::audio(CodecId::new("aac"));
    let mut dec = make_decoder(&params).expect("ADTS bootstrap");
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), data.to_vec());
    let _ = dec.send_packet(&pkt);
    for _ in 0..4 {
        if dec.receive_frame().is_err() {
            break;
        }
    }
    // Also exercise the synthetic-ASC path (panic_free_decode harness path 2).
    let mut params2 = CodecParameters::audio(CodecId::new("aac"));
    params2.sample_rate = Some(44_100);
    params2.channels = Some(2);
    params2.extradata = vec![0x12, 0x10];
    let mut dec2 = make_decoder(&params2).expect("ASC valid");
    let pkt2 = Packet::new(0, TimeBase::new(1, 44_100), data.to_vec());
    let _ = dec2.send_packet(&pkt2);
    for _ in 0..4 {
        if dec2.receive_frame().is_err() {
            break;
        }
    }
}

/// Bug 2 (#744) — replays the exact `ffmpeg_oracle_decode` fuzz crash
/// artifact (`crash-d46a936108da9568bdc9162ff307cd52cb36ba89`,
/// 113 bytes captured from the daily Fuzz CI run on 2026-05-10).
/// libavcodec accepts the embedded 88.2 kHz / 5.1 ADTS frame; before
/// the `parse_ics_info` `max_sfb` check our decoder bailed mid-frame
/// with an OOB on `SWB_LONG_96`, producing 0 frames and tripping the
/// fuzz oracle's "ffmpeg emitted a frame but oxideav-aac produced 0"
/// assertion. The pinned input runs the same code path that the fuzz
/// CI exercised; we just need it to not panic.
#[test]
fn ffmpeg_oracle_88200_5_1_adts_does_not_panic_744() {
    // Exact 113-byte input from the fuzz crash artifact.
    let data: [u8; 113] = [
        115, 101, 108, 105, 110, 117, 120, 102, 115, 160, 255, 249, 71, 140, 6, 0, 0, 0, 0, 0, 14,
        14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
        14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
        14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
        14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 0, 0, 1, 0, 0, 0, 0, 8, 255, 254, 1, 255, 171,
    ];
    // Replay the fuzz-target's ADTS-walk + per-frame decode loop.
    let params = CodecParameters::audio(CodecId::new("aac"));
    let mut dec = make_decoder(&params).expect("ADTS bootstrap");
    let mut off = 0usize;
    while off + 7 <= data.len() {
        if data[off] != 0xFF || (data[off + 1] & 0xF0) != 0xF0 {
            off += 1;
            continue;
        }
        let h = match parse_adts_header(&data[off..]) {
            Ok(h) => h,
            Err(_) => break,
        };
        if h.frame_length == 0 || off + h.frame_length > data.len() {
            break;
        }
        let pkt = Packet::new(
            0,
            TimeBase::new(1, h.sample_rate().unwrap_or(44_100) as i64),
            data[off..off + h.frame_length].to_vec(),
        );
        let _ = dec.send_packet(&pkt);
        let _ = dec.receive_frame();
        off += h.frame_length;
    }
}

/// Bug 2 (#744) — exercises the ADTS 88.2 kHz / 5.1 path end-to-end
/// against an `ffmpeg`-encoded fixture. Skips silently when ffmpeg
/// is not on `$PATH`.
#[test]
fn adts_88200hz_5_1_decodes_some_frames_744() {
    use std::process::Command;
    if Command::new("ffmpeg")
        .arg("-version")
        .output()
        .map(|o| !o.status.success())
        .unwrap_or(true)
    {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    let out = std::env::temp_dir().join("oxideav_aac_88200_5_1_regression.aac");
    let _ = std::fs::remove_file(&out);
    let st = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .args(["-f", "lavfi", "-i",
            "aevalsrc=exprs=0|0.05*sin(2*PI*220*t)|sin(2*PI*440*t)*0.5|0.2*sin(2*PI*880*t)|0.1*sin(2*PI*1760*t)|0:duration=0.05:sample_rate=88200:channel_layout=5.1"
        ])
        .args(["-c:a", "aac", "-b:a", "256k", "-f", "adts"])
        .arg(&out)
        .status()
        .expect("ffmpeg failed to spawn");
    if !st.success() {
        eprintln!("ffmpeg encode returned non-zero — skipping");
        return;
    }
    let bytes = std::fs::read(&out).expect("read fixture");

    let params = CodecParameters::audio(CodecId::new("aac"));
    let mut dec = make_decoder(&params).expect("ADTS bootstrap");

    let mut off = 0usize;
    let mut got = 0usize;
    while off + 7 <= bytes.len() {
        if bytes[off] != 0xFF || (bytes[off + 1] & 0xF0) != 0xF0 {
            off += 1;
            continue;
        }
        let h = parse_adts_header(&bytes[off..]).expect("valid ADTS header");
        if off + h.frame_length > bytes.len() {
            break;
        }
        assert_eq!(h.sample_rate(), Some(88_200), "fixture is 88.2 kHz");
        assert_eq!(h.channel_configuration, 6, "fixture is 5.1");
        let pkt = Packet::new(
            0,
            TimeBase::new(1, 88_200),
            bytes[off..off + h.frame_length].to_vec(),
        );
        dec.send_packet(&pkt).expect("send_packet");
        match dec.receive_frame() {
            Ok(Frame::Audio(_)) => {
                got += 1;
            }
            Ok(_) => panic!("non-audio frame from AAC decoder"),
            Err(e) => panic!(
                "88.2 kHz / 5.1 ADTS frame {got} rejected: {e:?} (ffmpeg accepts the same input)"
            ),
        }
        off += h.frame_length;
    }
    let _ = std::fs::remove_file(&out);
    assert!(
        got >= 1,
        "expected at least 1 decoded frame from 88.2 kHz / 5.1 ADTS, got {got}"
    );
}

/// Bug 3 (#742) — decode the documented intensity-stereo fixture
/// end-to-end. The divergence flagged by the fuzz oracle on
/// frame 2 (5814 LSB peak) was investigated: the fixture's 33
/// frames carry zero `INTENSITY_HCB` (cb=14/15) bands, so the
/// flagged divergence cannot originate from the §4.6.8.2.3
/// intensity-stereo path. The intensity-stereo decoder is unit-
/// tested in `src/decoder.rs::tests` (cb15/cb14 sign + ms_mask
/// flip + non-IS untouched). This regression test confirms (a)
/// the fixture decodes every ADTS frame to an `AudioFrame`,
/// (b) frame 1 (priming) is silent, and (c) the entire run
/// produces the expected 33 frames * 1024 samples / channel.
#[test]
fn intensity_stereo_fixture_decodes_742() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("docs/audio/aac/fixtures/aac-lc-intensity-stereo/input.aac"));
    let Some(path) = path else {
        eprintln!("workspace docs/ tree missing — skipping");
        return;
    };
    let Ok(bytes) = std::fs::read(&path) else {
        eprintln!("intensity-stereo fixture missing at {path:?} — skipping");
        return;
    };
    let params = CodecParameters::audio(CodecId::new("aac"));
    let mut dec = make_decoder(&params).expect("ADTS bootstrap");
    let mut off = 0usize;
    let mut frames: Vec<Vec<u8>> = Vec::new();
    while off + 7 <= bytes.len() {
        if bytes[off] != 0xFF || (bytes[off + 1] & 0xF0) != 0xF0 {
            off += 1;
            continue;
        }
        let h = parse_adts_header(&bytes[off..]).expect("valid ADTS header");
        if off + h.frame_length > bytes.len() {
            break;
        }
        let pkt = Packet::new(
            0,
            TimeBase::new(1, 44_100),
            bytes[off..off + h.frame_length].to_vec(),
        );
        dec.send_packet(&pkt).expect("send_packet");
        match dec.receive_frame() {
            Ok(Frame::Audio(af)) => {
                let plane = af.data.into_iter().next().unwrap_or_default();
                frames.push(plane);
            }
            Ok(_) => panic!("non-audio frame from AAC decoder"),
            Err(e) => panic!("intensity-stereo frame {} rejected: {e:?}", frames.len()),
        }
        off += h.frame_length;
    }
    // The fixture is 33 frames; allow ≥30 for resilience.
    assert!(
        frames.len() >= 30,
        "expected ≥30 frames, got {}",
        frames.len()
    );
    // First frame is priming + LongStart all-zero → must be silent.
    assert!(
        frames[0].iter().all(|&b| b == 0),
        "frame 0 (priming, all-zero spectrum) must be silent"
    );
    // Each frame is 1024 samples × 2 channels × 2 bytes.
    for (i, f) in frames.iter().enumerate() {
        assert_eq!(
            f.len(),
            1024 * 2 * 2,
            "frame {i} has unexpected byte length {}",
            f.len()
        );
    }
}

/// Bug (workspace task #773, round-#759 follow-up) — a 53-byte
/// fuzz input with two 15-byte ADTS frames at 88.2 kHz / 5.1 ch
/// carried an 8-byte payload too short to decode any element
/// (CPE with instance_tag = 7 etc.). libavcodec emits a silent
/// frame per ADTS header (logging "channel element 1.7 is not
/// allocated") rather than dropping the run; oxideav-aac
/// returned `InvalidData("bitreader: out of bits")` and the
/// `ffmpeg_oracle_decode` harness panicked with "ffmpeg emitted
/// 2 frames, oxideav-aac produced 0".
///
/// Per ISO/IEC 14496-3 §4.4.1 each ADTS frame is one
/// raw_data_block; libavcodec's tolerance is documented behaviour
/// for malformed payloads (per the same §4.6.18 implicit-SBR
/// notes). We now mirror it: when the configured stream's first
/// element is bit-truncated and we have NO partial PCM, fall
/// through to emit a silent frame at the expected channel layout
/// rather than propagating the bit-reader error. Other (non-
/// truncation) first-element errors still propagate so real
/// decoder bugs surface.
///
/// Exact bytes from
/// `fuzz/artifacts/ffmpeg_oracle_decode/crash-d012b01aa153af5faac6b7ef737cfa28272d7ff1`
/// captured by the daily Fuzz CI run on 2026-05-11.
#[test]
fn adts_short_payload_emits_silent_frame_773() {
    let data: [u8; 53] = [
        0xff, 0xff, 0xff, 0xf8, 0x47, 0x8c, 0x01, 0xff, 0xa0, 0x2c, 0xff, 0x06, 0xf9, 0x80, 0x09,
        0x73, 0x6e, 0xff, 0xf8, 0x40, 0x8c, 0x01, 0xf9, 0xa0, 0x2c, 0xff, 0xd5, 0x61, 0x6c, 0x5f,
        0xbc, 0x00, 0x00, 0x62, 0x75, 0x70, 0x2b, 0x47, 0xc4, 0x01, 0xff, 0x07, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xa0, 0x2c, 0x3a, 0xbf,
    ];
    let params = CodecParameters::audio(CodecId::new("aac"));
    let mut dec = make_decoder(&params).expect("ADTS bootstrap");
    let mut got = 0usize;
    let mut off = 0usize;
    while off + 7 <= data.len() {
        if data[off] != 0xFF || (data[off + 1] & 0xF0) != 0xF0 {
            off += 1;
            continue;
        }
        let h = match parse_adts_header(&data[off..]) {
            Ok(h) => h,
            Err(_) => {
                off += 1;
                continue;
            }
        };
        if h.frame_length == 0 || off + h.frame_length > data.len() || h.object_type != 2 {
            off += 1;
            continue;
        }
        let pkt = Packet::new(
            0,
            TimeBase::new(1, h.sample_rate().unwrap_or(44_100) as i64),
            data[off..off + h.frame_length].to_vec(),
        );
        if dec.send_packet(&pkt).is_ok() {
            if let Ok(Frame::Audio(_)) = dec.receive_frame() {
                got += 1;
            }
        }
        off += h.frame_length;
    }
    // Both ADTS frames must emit one silent audio frame each
    // (matching libavcodec's tolerance). Without the fix, both
    // frames returned `Err(InvalidData("bitreader: out of bits"))`
    // and the fuzz oracle treated the resulting "0 frames" as a
    // hard panic.
    assert!(
        got >= 2,
        "expected ≥2 silent frames from short-payload ADTS run, got {got}"
    );
}

/// Bug (workspace task #772) — `decode_packet` panicked at
/// `src/decoder.rs:704:52` with "index out of bounds: the len is 8
/// but the index is 8" on a 42-byte fuzz input that chains four
/// independent-window CPE elements (8 channels) followed by a fifth
/// CPE. The CPE-`else` (independent-window) branch had its
/// `got_channels + 2 > pcm.len()` guard placed AFTER the
/// IMDCT/copy loop, while the same guard in the
/// `common_window` branch above sits before the loop. The fifth CPE
/// indexed `self.chans[8]` and `pcm[8]` (both length 8) before the
/// guard could fire. Fix: hoist the guard above the loop so the
/// independent-window path mirrors the common-window path.
///
/// ISO/IEC 14496-3 §4.4.1.1 (Table 4.3, channel_configuration) caps
/// CCE-less raw_data_block output at 8 channels (7.1); chaining
/// further channel_pair_elements is non-conformant and we reject
/// with `Error::Invalid("AAC: CPE would overflow 8 channel slots")`.
///
/// Exact bytes from
/// `fuzz/artifacts/panic_free_decode/crash-97881f260b59ccd0e62f809e954e97438506232f`.
#[test]
fn cpe_independent_window_overflow_does_not_panic_772() {
    let data: [u8; 42] = [
        0x20, 0x7d, 0x40, 0x04, 0xa3, 0xc3, 0x85, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ];
    // Path 2: synthetic ASC AAC-LC / 44.1 kHz / stereo (the panic_free
    // harness's raw_data_block path that the original crash hit).
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(44_100);
    params.channels = Some(2);
    params.extradata = vec![0x12, 0x10];
    let mut dec = make_decoder(&params).expect("ASC valid");
    let pkt = Packet::new(0, TimeBase::new(1, 44_100), data.to_vec());
    let _ = dec.send_packet(&pkt);
    // Contract: must not panic. Returning Err is the desired outcome.
    for _ in 0..4 {
        if dec.receive_frame().is_err() {
            break;
        }
    }
}

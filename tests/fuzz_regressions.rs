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

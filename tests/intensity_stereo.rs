//! §4.6.8.2 Intensity Stereo — integration coverage over the public API.
//!
//! [`oxideav_aac::intensity_stereo::apply_intensity_stereo`] is
//! unit-tested in `src/` for the `is_intensity` sign, the
//! `invert_intensity` phase-reversal branches, the `0.5^(0.25·is_pos)`
//! gain ladder, short-window grouping, and shape validation. This
//! driver pins the algebra the decoder relies on: that an encoder which
//! intensity-codes a band (right channel a scaled copy of the left)
//! is recovered exactly by the §4.6.8.2.3 left→right derivation, across
//! a whole `max_sfb`-band long frame with mixed positions and codebook
//! phases, and that the per-band M/S mask flips the phase as specified.

use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::intensity_stereo::{
    apply_intensity_stereo, intensity_gain, is_intensity, IntensityPairSpectra,
};
use oxideav_aac::section_data::{INTENSITY_HCB, INTENSITY_HCB2};
use oxideav_aac::swb_offset::long_window_offsets;

const FS_44100: u8 = 4;
const SPECTRUM_CB: u8 = 2;

fn long_ics_info(max_sfb: u8) -> IcsInfo {
    IcsInfo {
        family: oxideav_aac::swb_offset::FrameFamily::Lc1024,
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb,
        scale_factor_grouping: None,
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: NUM_SWB_LONG_WINDOW[FS_44100 as usize],
    }
}

/// An encoder that intensity-codes alternating bands with a chosen
/// position / phase produces a right channel equal to
/// `scale · left`; the §4.6.8.2.3 decode recovers exactly that.
#[test]
fn encode_then_decode_recovers_intensity_right() {
    let max_sfb = 20u8;
    let ics = long_ics_info(max_sfb);
    let offsets = long_window_offsets(FS_44100).unwrap();
    let band_top = offsets[max_sfb as usize] as usize;

    let mut left = vec![0.0f64; 1024];
    for (i, slot) in left.iter_mut().enumerate().take(band_top) {
        *slot = (i as f64 * 0.013).sin() * 7.0 + 0.5;
    }

    // Encoder side: intensity-code every 3rd band. Choose a per-band
    // position and an in/out-of-phase codebook, then synthesise the
    // right channel as the decoder would. Non-intensity bands keep a
    // distinct, untouched right-channel content.
    let mut right = vec![0.0f64; 1024];
    let mut cb = vec![vec![SPECTRUM_CB; max_sfb as usize]];
    let mut is_pos = vec![vec![0i32; max_sfb as usize]];
    let untouched_marker = 1234.5f64;
    for sfb in 0..max_sfb as usize {
        let s = offsets[sfb] as usize;
        let e = offsets[sfb + 1] as usize;
        if sfb % 3 == 0 {
            // Intensity band: alternate phase, vary the position.
            let book = if sfb % 2 == 0 {
                INTENSITY_HCB
            } else {
                INTENSITY_HCB2
            };
            cb[0][sfb] = book;
            let pos = (sfb as i32 % 9) - 4; // -4..=4
            is_pos[0][sfb] = pos;
            let scale = is_intensity(book) as f64 * intensity_gain(pos);
            for i in s..e {
                right[i] = scale * left[i];
            }
        } else {
            // Real spectrum band: untouched by intensity decode.
            for v in right.iter_mut().take(e).skip(s) {
                *v = untouched_marker;
            }
        }
    }

    // Decoder: start from a right buffer that already carries the
    // intensity-coded values garbled (the inverse-quantised junk) but
    // the untouched marker on real bands. The decode must regenerate
    // the intensity bands exactly and leave the others as-is.
    let mut decoded_right = right.clone();
    for sfb in (0..max_sfb as usize).filter(|s| s % 3 == 0) {
        let s = offsets[sfb] as usize;
        let e = offsets[sfb + 1] as usize;
        for v in decoded_right.iter_mut().take(e).skip(s) {
            *v = -9999.0; // junk to be overwritten
        }
    }

    let mut pair = IntensityPairSpectra {
        left: &left,
        right: &mut decoded_right,
        right_sfb_cb: &cb,
        is_pos: &is_pos,
    };
    apply_intensity_stereo(&mut pair, false, &[], &ics, FS_44100).unwrap();

    for i in 0..band_top {
        assert!(
            (decoded_right[i] - right[i]).abs() < 1e-9,
            "right[{i}] {} vs expected {}",
            decoded_right[i],
            right[i]
        );
    }
}

/// Under a per-band M/S mask, a set `ms_used` bit on an intensity band
/// reverses the phase (§4.6.8.2.3 `invert_intensity = 1 - 2·ms_used`).
#[test]
fn ms_mask_reverses_phase() {
    let ics = long_ics_info(1);
    let offsets = long_window_offsets(FS_44100).unwrap();
    let s = offsets[0] as usize;
    let e = offsets[1] as usize;

    let mut left = vec![0.0f64; 1024];
    for v in left.iter_mut().take(e).skip(s) {
        *v = 11.0;
    }
    // In-phase codebook, position 0 → base scale +1; ms_used flips it.
    let cb = vec![vec![INTENSITY_HCB]];
    let is_pos = vec![vec![0i32]];

    let mut right = vec![0.0f64; 1024];
    let mut pair = IntensityPairSpectra {
        left: &left,
        right: &mut right,
        right_sfb_cb: &cb,
        is_pos: &is_pos,
    };
    let ms_used = vec![vec![true]];
    apply_intensity_stereo(&mut pair, true, &ms_used, &ics, FS_44100).unwrap();

    for &v in right.iter().take(e).skip(s) {
        assert!((v + 11.0).abs() < 1e-9, "expected -11");
    }
}

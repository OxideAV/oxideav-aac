//! §4.6.8.1 M/S de-matrix — integration coverage over the public API.
//!
//! [`oxideav_aac::ms_stereo::apply_ms_stereo`] is unit-tested in
//! `src/` for the three `ms_mask_present` modes, the intensity / noise
//! exclusions, short-window grouping, and shape validation. This
//! driver pins the algebra the decoder relies on: that the de-matrix
//! reconstructs the same left/right pair an encoder started from for a
//! whole `max_sfb`-band long frame, that `mask_present == 0` is a true
//! no-op, and that the operation is its own structural inverse
//! (`l' = m+s`, `r' = m-s`).

use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::ms_stereo::{apply_ms_stereo, ChannelPairSpectra, MsMaskPresent};
use oxideav_aac::swb_offset::long_window_offsets;

const FS_44100: u8 = 4;

fn long_ics_info(max_sfb: u8) -> IcsInfo {
    IcsInfo {
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

/// Encode a left/right pair into mid/side for the bands flagged in
/// `ms_used` (the encoder side of the §4.6.8.1 matrix:
/// `m = (l+r)/2`, `s = (l-r)/2`), then decode and recover (l,r).
#[test]
fn encode_then_decode_recovers_left_right() {
    let max_sfb = 20u8;
    let ics = long_ics_info(max_sfb);
    let offsets = long_window_offsets(FS_44100).unwrap();

    // Original left/right spectra: distinct, deterministic content.
    let mut left = vec![0.0f64; 1024];
    let mut right = vec![0.0f64; 1024];
    let band_top = offsets[max_sfb as usize] as usize;
    for i in 0..band_top {
        left[i] = (i as f64 * 0.013).sin() * 7.0;
        right[i] = (i as f64 * 0.021).cos() * 4.0;
    }
    let orig_left = left.clone();
    let orig_right = right.clone();

    // Encoder: every other band uses M/S.
    let ms_used: Vec<bool> = (0..max_sfb as usize).map(|sfb| sfb % 2 == 0).collect();
    for (sfb, &on) in ms_used.iter().enumerate() {
        if !on {
            continue;
        }
        let s = offsets[sfb] as usize;
        let e = offsets[sfb + 1] as usize;
        for i in s..e {
            let m = (left[i] + right[i]) * 0.5;
            let side = (left[i] - right[i]) * 0.5;
            left[i] = m;
            right[i] = side;
        }
    }

    // Decoder: real spectrum codebook (2) on every band.
    let cb = vec![vec![2u8; max_sfb as usize]];
    let mut pair = ChannelPairSpectra {
        left: &mut left,
        right: &mut right,
        left_sfb_cb: &cb,
        right_sfb_cb: &cb,
    };
    apply_ms_stereo(&mut pair, MsMaskPresent::Mask, &[ms_used], &ics, FS_44100).unwrap();

    for i in 0..band_top {
        assert!(
            (left[i] - orig_left[i]).abs() < 1e-9,
            "left[{i}] {} vs {}",
            left[i],
            orig_left[i]
        );
        assert!(
            (right[i] - orig_right[i]).abs() < 1e-9,
            "right[{i}] {} vs {}",
            right[i],
            orig_right[i]
        );
    }
}

#[test]
fn mask_present_zero_is_noop() {
    let ics = long_ics_info(40);
    let cb = vec![vec![2u8; 40]];
    let left: Vec<f64> = (0..1024).map(|i| i as f64).collect();
    let right: Vec<f64> = (0..1024).map(|i| -(i as f64)).collect();
    let mut l = left.clone();
    let mut r = right.clone();
    let mut pair = ChannelPairSpectra {
        left: &mut l,
        right: &mut r,
        left_sfb_cb: &cb,
        right_sfb_cb: &cb,
    };
    apply_ms_stereo(&mut pair, MsMaskPresent::AllZeros, &[], &ics, FS_44100).unwrap();
    assert_eq!(l, left);
    assert_eq!(r, right);
}

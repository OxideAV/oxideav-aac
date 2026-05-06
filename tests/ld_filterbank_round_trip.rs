//! End-to-end LD/ELD filterbank round-trip integration test.
//!
//! Builds N-sample sine signals, runs the forward MDCT (encoder side),
//! then feeds the spectra through the LD overlap-add filterbank
//! (`crate::ld_eld::imdct_and_overlap_ld`) and measures PCM
//! reconstruction error in the second-frame's PCM output region. This
//! exercises the entire LD filterbank path that an eventual full AAC-LD
//! frame decoder will sit on top of.
//!
//! There are no LD/ELD bitstream fixtures yet (ffmpeg's encoder doesn't
//! produce LD or ELD streams); we instead self-validate the LD filterbank
//! via TDAC. Once an LD-capable encoder lands or a docs collaborator
//! provides an LD-encoded .m4a fixture, a "real-content cross-decode"
//! integration test will join this self-validation one.

use oxideav_aac::imdct::{LD_480_INPUT, LD_512_INPUT};
use oxideav_aac::ld_eld::{imdct_and_overlap_ld, LdChannelState, LdFrameLength};
use oxideav_aac::mdct::{mdct_ld_480, mdct_ld_512};
use oxideav_aac::window::{sine_ld_480, sine_ld_512};

use std::f64::consts::PI;

/// Reconstruct a windowed sine through the LD-512 filterbank and assert
/// peak-signal-to-noise ratio against the original.
#[test]
fn ld_512_filterbank_psnr_sine() {
    const N: usize = 512;
    // 4 frames of input (we drop the first frame's PCM as warm-up).
    let frames = 4;
    let total = frames * N;
    let mut x = vec![0.0f32; total];
    // Multi-tone signal: three sines at different frequencies summed.
    for (i, sample) in x.iter_mut().enumerate() {
        let t = i as f64 / N as f64;
        *sample = ((2.0 * PI * 7.0 * t).sin()
            + 0.5 * (2.0 * PI * 33.0 * t).sin()
            + 0.25 * (2.0 * PI * 101.0 * t).sin()) as f32
            / 1.75;
    }

    let win = sine_ld_512();
    let mut state = LdChannelState::new(N);
    let mut all_pcm = Vec::with_capacity(total);
    let mut all_x = Vec::with_capacity(total);

    // Encode + decode each frame in lockstep. Frame f covers input
    // window [f*N, (f+2)*N) but only emits PCM for samples [f*N, (f+1)*N)
    // after OLA against frame f-1.
    for f in 0..(frames - 1) {
        let start = f * N;
        let mut t = vec![0.0f32; 2 * N];
        for i in 0..N {
            t[i] = x[start + i] * win[i];
            t[N + i] = x[start + N + i] * win[N - 1 - i];
        }
        let mut spec = vec![0.0f32; LD_512_INPUT];
        mdct_ld_512(&t, &mut spec);
        let mut pcm = vec![0.0f32; N];
        imdct_and_overlap_ld(&spec, &mut state, &mut pcm, LdFrameLength::Samples512).unwrap();
        // Skip the first frame: its OLA against the zero-init state
        // doesn't reconstruct the input.
        if f >= 1 {
            all_pcm.extend_from_slice(&pcm);
            all_x.extend_from_slice(&x[start..start + N]);
        }
    }

    let psnr = compute_audio_psnr(&all_x, &all_pcm);
    assert!(
        psnr >= 60.0,
        "LD-512 multi-tone reconstruction PSNR {psnr} dB below 60 dB threshold"
    );
}

/// Same shape, but for the broadcast-profile 480-sample frame size.
#[test]
fn ld_480_filterbank_psnr_sine() {
    const N: usize = 480;
    let frames = 4;
    let total = frames * N;
    let mut x = vec![0.0f32; total];
    for (i, sample) in x.iter_mut().enumerate() {
        let t = i as f64 / N as f64;
        *sample = ((2.0 * PI * 5.0 * t).sin() + 0.4 * (2.0 * PI * 51.0 * t).sin()) as f32 / 1.4;
    }
    let win = sine_ld_480();
    let mut state = LdChannelState::new(N);
    let mut all_pcm = Vec::new();
    let mut all_x = Vec::new();
    for f in 0..(frames - 1) {
        let start = f * N;
        let mut t = vec![0.0f32; 2 * N];
        for i in 0..N {
            t[i] = x[start + i] * win[i];
            t[N + i] = x[start + N + i] * win[N - 1 - i];
        }
        let mut spec = vec![0.0f32; LD_480_INPUT];
        mdct_ld_480(&t, &mut spec);
        let mut pcm = vec![0.0f32; N];
        imdct_and_overlap_ld(&spec, &mut state, &mut pcm, LdFrameLength::Samples480).unwrap();
        if f >= 1 {
            all_pcm.extend_from_slice(&pcm);
            all_x.extend_from_slice(&x[start..start + N]);
        }
    }
    let psnr = compute_audio_psnr(&all_x, &all_pcm);
    assert!(
        psnr >= 60.0,
        "LD-480 multi-tone reconstruction PSNR {psnr} dB below 60 dB threshold"
    );
}

/// LD filterbank impulse response: a single δ[k] coefficient through the
/// IMDCT + windowed OLA must produce a finite, nonzero impulse-shaped PCM
/// response. Verifies the kernel + window + state plumbing is wired
/// correctly even when only one bin is non-zero.
#[test]
fn ld_512_single_bin_produces_impulse_response() {
    const N: usize = 512;
    let mut spec = vec![0.0f32; N];
    spec[10] = 1.0;
    let mut state = LdChannelState::new(N);
    let mut pcm = vec![0.0f32; N];
    imdct_and_overlap_ld(&spec, &mut state, &mut pcm, LdFrameLength::Samples512).unwrap();
    let energy: f64 = pcm.iter().map(|&v| (v as f64) * (v as f64)).sum();
    assert!(
        energy > 0.0,
        "LD-512 single-bin spectrum should produce nonzero PCM"
    );
    // No NaNs / Infs.
    for &v in &pcm {
        assert!(
            v.is_finite(),
            "LD filterbank produced non-finite PCM sample"
        );
    }
}

/// PSNR helper. Decibels = 10·log10(peak² / mse). Peak is fixed at 1.0
/// since our test signals are bounded to [-1, 1].
fn compute_audio_psnr(reference: &[f32], reconstructed: &[f32]) -> f64 {
    assert_eq!(reference.len(), reconstructed.len());
    let n = reference.len() as f64;
    let mse: f64 = reference
        .iter()
        .zip(reconstructed.iter())
        .map(|(&r, &v)| {
            let d = (r - v) as f64;
            d * d
        })
        .sum::<f64>()
        / n;
    if mse <= 0.0 {
        return 200.0; // perfect reconstruction
    }
    let peak = 1.0f64;
    10.0 * (peak * peak / mse).log10()
}

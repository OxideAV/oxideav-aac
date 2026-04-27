//! AAC windowing — sine and KBD (Kaiser-Bessel-Derived) windows.
//!
//! ISO/IEC 14496-3 §4.6.11 specifies two window shapes:
//! - sine window (W=0): `w(n) = sin( π/(2N)·(n + 0.5) )` for `0 ≤ n < 2N`
//! - KBD window (W=1): derived from a Kaiser window of length `N+1` with
//!   alpha=4 for long (N=1024) and alpha=6 for short (N=128). The full
//!   2N-long window is squared-OLA partition-of-unity.
//!
//! `sine_long`/`kbd_long`/`sine_short`/`kbd_short` each return the
//! **rising half** (length N) of the spec's full 2N-long window. The full
//! window is `[w, reverse(w)]` per the doubling scheme used by
//! `build_long_window_full` / `synth.rs`. The half being monotonic-rising
//! is what makes the doubling work: a symmetric "bell over N" half would
//! produce a TWO-bell 2N-long window — a real bug fixed in r20 for KBD,
//! which was using a symmetric bell where sine had always been correct.

use std::f64::consts::PI;
use std::sync::OnceLock;

/// Long-window length (= IMDCT 2N divided by 2).
pub const LONG_LEN: usize = 1024;
/// Short-window length.
pub const SHORT_LEN: usize = 128;

static SINE_LONG: OnceLock<Vec<f32>> = OnceLock::new();
static SINE_SHORT: OnceLock<Vec<f32>> = OnceLock::new();
static KBD_LONG: OnceLock<Vec<f32>> = OnceLock::new();
static KBD_SHORT: OnceLock<Vec<f32>> = OnceLock::new();

/// Build the AAC/MLT sine window — first half only (length `n`).
///
/// The full 2N-long window is `w[i] = sin((i+0.5)·π/(2N))` for i in [0, 2N).
/// The first half (i in [0, N)) goes from ~0 up to ~sin(π/2 - small) ≈ 1; the
/// second half (i in [N, 2N)) is the time-reversal of the first half (the
/// `synth.rs` doubling-scheme applies `w[N-1-i]` for the falling slope).
/// The squared overlap-add of two consecutive halves gives sin²+cos²=1
/// (partition-of-unity), which is what makes AAC's TDAC OLA reconstruct
/// the input exactly.
fn build_sine(n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| ((PI / (2.0 * n as f64)) * (i as f64 + 0.5)).sin() as f32)
        .collect()
}

/// Modified Bessel I0(x) via series expansion. Converges quickly for the
/// alpha values we use (4..=6).
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    let mut k = 1.0;
    while term > 1e-15 * sum {
        term *= (x / (2.0 * k)).powi(2);
        sum += term;
        k += 1.0;
        if k > 200.0 {
            break;
        }
    }
    sum
}

/// Build the **rising half** of a KBD window for AAC long/short blocks,
/// per ISO/IEC 14496-3 §4.6.11.4.
///
/// `n` is the half-window length (= 1024 for long, 128 for short). The
/// returned vector is monotonic-rising from `w[0]≈0` to `w[n-1]≈1` —
/// the spec full-length-`2n` window is constructed downstream as
/// `[w, reverse(w)]`, the same shape `sine_long`/`sine_short` use.
///
/// Uses a Kaiser window of length `n+1` indexed `0..=n` and the spec
/// formula `W[i] = sqrt( sum_{k=0..=i} kaiser[k] / sum_{k=0..=n} kaiser[k] )`
/// for `i in 0..n`.
fn build_kbd(n: usize, alpha: f64) -> Vec<f32> {
    let alpha_pi = alpha * PI;
    let denom = bessel_i0(alpha_pi);
    let mut kaiser = vec![0.0f64; n + 1];
    for i in 0..=n {
        let t = (2.0 * i as f64 / n as f64) - 1.0;
        let arg = alpha_pi * (1.0 - t * t).sqrt();
        kaiser[i] = bessel_i0(arg) / denom;
    }
    let mut cs = vec![0.0f64; n + 1];
    let mut acc = 0.0;
    for i in 0..=n {
        acc += kaiser[i];
        cs[i] = acc;
    }
    let total = cs[n];
    let mut out = vec![0.0f32; n];
    for i in 0..n {
        out[i] = (cs[i] / total).sqrt() as f32;
    }
    out
}

pub fn sine_long() -> &'static [f32] {
    SINE_LONG.get_or_init(|| build_sine(LONG_LEN))
}

pub fn sine_short() -> &'static [f32] {
    SINE_SHORT.get_or_init(|| build_sine(SHORT_LEN))
}

pub fn kbd_long() -> &'static [f32] {
    KBD_LONG.get_or_init(|| build_kbd(LONG_LEN, 4.0))
}

pub fn kbd_short() -> &'static [f32] {
    KBD_SHORT.get_or_init(|| build_kbd(SHORT_LEN, 6.0))
}

/// Resolve a `WindowShape` to its long half-window table.
pub fn long_window_for_shape(shape: crate::syntax::WindowShape) -> &'static [f32] {
    match shape {
        crate::syntax::WindowShape::Sine => sine_long(),
        crate::syntax::WindowShape::Kbd => kbd_long(),
    }
}

/// Resolve a `WindowShape` to its short half-window table.
pub fn short_window_for_shape(shape: crate::syntax::WindowShape) -> &'static [f32] {
    match shape {
        crate::syntax::WindowShape::Sine => sine_short(),
        crate::syntax::WindowShape::Kbd => kbd_short(),
    }
}

/// Build the full-length (2 * `LONG_LEN`) long-style window weight vector
/// for a given (`seq`, current shape, previous shape) triple, per ISO/IEC
/// 14496-3 §4.6.11 Figure 4.11.
///
/// * The first half (positions 0..N) is governed by `prev_shape` — it's
///   the rising slope used by the PREVIOUS block's right half, so the
///   TDAC overlap-add cancels the aliasing.
/// * The second half (positions N..2N) is governed by `shape`.
///
/// Result layout:
///
///   OnlyLong:  rising sine/KBD (0..N) | falling sine/KBD (N..2N)
///   LongStart: rising long   | 448 ones | 128 short-fall | 448 zeros
///   LongStop:  448 zeros | 128 short-rise | 448 ones | falling long
///
/// EightShort uses a different, non-1D layout handled by the caller.
pub fn build_long_window_full(
    seq: crate::syntax::WindowSequence,
    shape: crate::syntax::WindowShape,
    prev_shape: crate::syntax::WindowShape,
) -> Vec<f32> {
    use crate::syntax::WindowSequence;
    let n = LONG_LEN;
    let mut w = vec![0.0f32; 2 * n];
    let prev_w = long_window_for_shape(prev_shape);
    let cur_w = long_window_for_shape(shape);
    match seq {
        WindowSequence::OnlyLong | WindowSequence::EightShort => {
            for i in 0..n {
                w[i] = prev_w[i];
                w[n + i] = cur_w[n - 1 - i];
            }
        }
        WindowSequence::LongStart => {
            for i in 0..n {
                w[i] = prev_w[i];
            }
            let cur_short = short_window_for_shape(shape);
            for i in 0..448 {
                w[n + i] = 1.0;
            }
            for i in 0..128 {
                w[n + 448 + i] = cur_short[127 - i];
            }
        }
        WindowSequence::LongStop => {
            let prev_short = short_window_for_shape(prev_shape);
            for i in 0..128 {
                w[448 + i] = prev_short[i];
            }
            for i in 576..n {
                w[i] = 1.0;
            }
            for i in 0..n {
                w[n + i] = cur_w[n - 1 - i];
            }
        }
    }
    w
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::syntax::{WindowSequence, WindowShape};

    #[test]
    fn long_start_window_shape() {
        let w = build_long_window_full(
            WindowSequence::LongStart,
            WindowShape::Sine,
            WindowShape::Sine,
        );
        let n = LONG_LEN;
        // First half: ordinary sine_long rising.
        let sine = sine_long();
        for i in 0..n {
            assert!((w[i] - sine[i]).abs() < 1e-6, "first half[{i}]");
        }
        // [N..N+448): flat 1.0
        for i in 0..448 {
            assert!((w[n + i] - 1.0).abs() < 1e-6, "flat region[{i}]");
        }
        // [N+448..N+576): sine_short falling
        let short = sine_short();
        for i in 0..128 {
            assert!((w[n + 448 + i] - short[127 - i]).abs() < 1e-6);
        }
        // [N+576..2N): zero
        for i in 576..n {
            assert!(w[n + i] == 0.0);
        }
    }

    #[test]
    fn long_stop_window_shape() {
        let w = build_long_window_full(
            WindowSequence::LongStop,
            WindowShape::Sine,
            WindowShape::Sine,
        );
        let n = LONG_LEN;
        // [0..448): zero
        for i in 0..448 {
            assert!(w[i] == 0.0);
        }
        // [448..576): sine_short rising
        let short = sine_short();
        for i in 0..128 {
            assert!((w[448 + i] - short[i]).abs() < 1e-6);
        }
        // [576..1024): flat 1.0
        for i in 576..n {
            assert!((w[i] - 1.0).abs() < 1e-6);
        }
        // Second half: sine_long falling
        let sine = sine_long();
        for i in 0..n {
            assert!((w[n + i] - sine[n - 1 - i]).abs() < 1e-6);
        }
    }

    #[test]
    fn only_long_tdac_with_itself() {
        // Two consecutive OnlyLong blocks must reproduce the input via
        // overlap-add of their squared window halves: the falling half of
        // block A and the rising half of block B sum to 1 sample-wise.
        let wa = build_long_window_full(
            WindowSequence::OnlyLong,
            WindowShape::Sine,
            WindowShape::Sine,
        );
        let wb = build_long_window_full(
            WindowSequence::OnlyLong,
            WindowShape::Sine,
            WindowShape::Sine,
        );
        let n = LONG_LEN;
        for i in 0..n {
            let sum = wa[n + i] * wa[n + i] + wb[i] * wb[i];
            assert!((sum - 1.0).abs() < 1e-4, "TDAC at {i}: {sum}");
        }
    }

    #[test]
    fn sine_at_endpoints() {
        // First-half sine: w[0] = sin(π·0.5/(2N)) ≈ small, w[N-1] ≈ 1.
        let w = sine_long();
        assert!(w[0] > 0.0 && w[0] < 0.01);
        assert!(w[LONG_LEN - 1] > 0.999);
    }

    #[test]
    fn sine_partition_of_unity_full_window() {
        // POU on the doubling-scheme full window:
        //   full_w[i]² + full_w[i+L]² = w[i]² + w[L-1-i]² = sin² + cos² = 1.
        let w = sine_long();
        for i in 0..LONG_LEN {
            let a = w[i];
            let b = w[LONG_LEN - 1 - i];
            let s = a * a + b * b;
            assert!((s - 1.0).abs() < 1e-3, "sine POU off at {i}: {s}");
        }
    }

    #[test]
    fn kbd_partition_of_unity() {
        // KBD POU on the rising-half-window pattern that the doubling
        // scheme uses: w[i]² + w[N-1-i]² = 1 for i in 0..N (where N is
        // the half-window length). This is the same POU sine_long
        // satisfies — see `sine_partition_of_unity_full_window`.
        let w = kbd_long();
        for i in 0..LONG_LEN {
            let a = w[i];
            let b = w[LONG_LEN - 1 - i];
            let s = a * a + b * b;
            assert!((s - 1.0).abs() < 1e-3, "kbd POU off at {i}: {s}");
        }
    }

    #[test]
    fn kbd_long_is_monotonic_rising() {
        // The half-window must rise monotonically from ~0 to ~1 for
        // build_long_window_full to produce the spec single-bell
        // 2N-long window. A symmetric "bell over N" instead of a
        // monotonic rising "half over N" yields a TWO-bell 2N-long
        // window — the exact bug that broke ffmpeg-interop on
        // real-content AAC-LC streams before r20.
        let w = kbd_long();
        assert!(w[0] > 0.0 && w[0] < 0.01, "kbd_long[0] should be small");
        assert!(w[LONG_LEN - 1] > 0.999, "kbd_long[N-1] should be ~1");
        for i in 1..LONG_LEN {
            assert!(
                w[i] >= w[i - 1] - 1e-7,
                "kbd_long not monotonic rising at {i}: {} < {}",
                w[i],
                w[i - 1]
            );
        }
    }

    #[test]
    fn kbd_only_long_tdac_with_itself() {
        // Two consecutive OnlyLong KBD blocks must satisfy TDAC: the
        // falling half of A summed with the rising half of B equals 1.
        // (Counterpart to only_long_tdac_with_itself for SINE.) This
        // is the test that would have caught the pre-r20 symmetric-
        // bell KBD bug — it failed sample-by-sample then.
        let wa =
            build_long_window_full(WindowSequence::OnlyLong, WindowShape::Kbd, WindowShape::Kbd);
        let wb =
            build_long_window_full(WindowSequence::OnlyLong, WindowShape::Kbd, WindowShape::Kbd);
        let n = LONG_LEN;
        for i in 0..n {
            let sum = wa[n + i] * wa[n + i] + wb[i] * wb[i];
            assert!((sum - 1.0).abs() < 1e-3, "KBD TDAC at {i}: {sum}");
        }
    }
}

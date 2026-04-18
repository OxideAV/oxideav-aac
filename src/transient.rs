//! Transient detector for encoder window-sequence selection.
//!
//! Detects frames whose time-domain signal carries an abrupt onset — a
//! drum hit, attack transient, hard consonant — that a long MDCT would
//! smear across the 1024-sample block (pre-echo). When one of these is
//! detected the encoder should switch from `OnlyLong` to an `EightShort`
//! sequence so the finer temporal resolution localises the event.
//!
//! Algorithm: compare sub-frame short-term energy against the running
//! long-term energy. Split the incoming 1024-sample frame into eight
//! 128-sample sub-frames, compute each one's sum-of-squares, and flag a
//! transient when:
//!
//!  * Any sub-frame's energy exceeds `attack_ratio_db` dB above the
//!    long-term running-average energy, AND
//!  * The same sub-frame's energy is at least `min_sub_energy_floor`
//!    (absolute floor — keeps the detector from firing on near-silent
//!    input where quantisation noise alone drives the ratio up).
//!
//! The long-term average itself is maintained by the caller (typically
//! one [`TransientDetector`] per channel across the whole stream) and
//! updated from each frame's mean sub-frame energy with a one-pole IIR.

/// Stateful detector: one instance per channel, carried across frames.
#[derive(Clone, Debug)]
pub struct TransientDetector {
    /// Exponential moving average of per-frame mean sub-frame energy.
    avg_energy: f32,
    /// Threshold expressed as a linear amplitude ratio. A sub-frame
    /// whose energy exceeds `attack_ratio × avg_energy` triggers a
    /// transient classification.
    attack_ratio: f32,
    /// Absolute energy floor below which the detector returns `false`
    /// regardless of the ratio. Prevents false positives on silence.
    min_sub_energy_floor: f32,
    /// IIR smoothing factor for the running average. Larger = slower
    /// adaptation, so sustained loud sections don't mask subsequent
    /// transients.
    smoothing: f32,
}

impl Default for TransientDetector {
    fn default() -> Self {
        Self {
            avg_energy: 0.0,
            // 12 dB above average — picks up drum hits and vocal
            // plosives without firing on sustained tones.
            attack_ratio: 10.0_f32.powf(12.0 / 10.0),
            // ~-60 dBFS per sample, scaled for 128-sample sub-frames.
            min_sub_energy_floor: 1e-4,
            smoothing: 0.25,
        }
    }
}

impl TransientDetector {
    pub fn new() -> Self {
        Self::default()
    }

    /// Configure the attack threshold. `db` is the ratio of sub-frame
    /// energy over long-term average expressed in decibels; typical
    /// values are 8–15 dB.
    pub fn with_attack_ratio_db(mut self, db: f32) -> Self {
        self.attack_ratio = 10.0_f32.powf(db / 10.0);
        self
    }

    /// Feed one FRAME_LEN-sample frame of time-domain samples. Returns
    /// `true` if a transient is detected in this frame.
    pub fn analyse(&mut self, frame: &[f32]) -> bool {
        const SUB_COUNT: usize = 8;
        if frame.len() < SUB_COUNT {
            return false;
        }
        let sub_len = frame.len() / SUB_COUNT;
        let mut sub_energies = [0.0f32; SUB_COUNT];
        for (sub, slot) in sub_energies.iter_mut().enumerate() {
            let start = sub * sub_len;
            let end = start + sub_len;
            let mut e = 0.0f32;
            for &x in &frame[start..end] {
                e += x * x;
            }
            *slot = e;
        }
        let mean_sub = sub_energies.iter().sum::<f32>() / SUB_COUNT as f32;

        // Bootstrap on the first frame so the threshold is finite from
        // the very first call. Subsequent frames blend in via IIR.
        if self.avg_energy == 0.0 {
            self.avg_energy = mean_sub.max(self.min_sub_energy_floor);
        }

        let threshold = self.avg_energy * self.attack_ratio;
        let mut hit = false;
        for &e in sub_energies.iter() {
            if e >= self.min_sub_energy_floor && e >= threshold {
                hit = true;
                break;
            }
        }

        // Update the running average *after* the decision so a strong
        // transient doesn't pollute its own threshold and mask a
        // follow-up attack a few frames later.
        self.avg_energy = (1.0 - self.smoothing) * self.avg_energy + self.smoothing * mean_sub;

        hit
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Full-duration steady sine shouldn't trigger — all sub-frame
    /// energies are roughly equal.
    #[test]
    fn steady_tone_is_not_transient() {
        let mut det = TransientDetector::new();
        let mut frame = [0.0f32; 1024];
        for i in 0..1024 {
            let t = i as f32 / 1024.0;
            frame[i] = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.5;
        }
        // Feed a few frames to settle the running average.
        for _ in 0..4 {
            assert!(!det.analyse(&frame));
        }
    }

    /// A spike concentrated in one sub-frame should fire the detector.
    #[test]
    fn sudden_attack_is_transient() {
        let mut det = TransientDetector::new();
        // Warm-up: 3 frames of low-level noise.
        let noise = [0.01f32; 1024];
        for _ in 0..3 {
            det.analyse(&noise);
        }
        // Attack frame: sub-frames 0..3 silent, sub-frame 4 loud, rest quiet.
        let mut frame = [0.0f32; 1024];
        for i in 4 * 128..5 * 128 {
            frame[i] = 0.8;
        }
        assert!(det.analyse(&frame), "attack sub-frame should trigger");
    }

    /// Silent frames don't trigger (energy below the floor).
    #[test]
    fn silence_is_not_transient() {
        let mut det = TransientDetector::new();
        let frame = [0.0f32; 1024];
        for _ in 0..4 {
            assert!(!det.analyse(&frame));
        }
    }

    /// After a transient has passed, the running average catches up and
    /// the SAME signal no longer fires.
    #[test]
    fn running_average_adapts() {
        let mut det = TransientDetector::new();
        let noise = [0.01f32; 1024];
        for _ in 0..3 {
            det.analyse(&noise);
        }
        let mut attack = [0.0f32; 1024];
        for s in attack.iter_mut().take(128) {
            *s = 0.5;
        }
        // First attack: fires.
        assert!(det.analyse(&attack));
        // Same shape repeatedly for many frames → average rises, stops firing.
        for _ in 0..20 {
            det.analyse(&attack);
        }
        assert!(!det.analyse(&attack));
    }
}

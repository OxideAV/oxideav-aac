//! Pulse data — ISO/IEC 14496-3 §4.6.5.
//!
//! `pulse_data()` is an optional AAC tool that lets the encoder add up to
//! four signed pulses to the integer-quantised spectrum prior to inverse
//! quantisation. The receiver:
//!
//! 1. Parses `number_pulse` (2 bits), `pulse_start_sfb` (6 bits), then
//!    per-pulse `pulse_offset` (5 bits) and `pulse_amp` (4 bits).
//! 2. Locates the first pulse at `swb[pulse_start_sfb] + pulse_offset[0]`.
//!    Each subsequent pulse is offset further by `pulse_offset[i]`.
//! 3. For each pulse, increments `|x_quant[k]|` by `pulse_amp[i]` while
//!    preserving the existing sign (or applying `+amp` if the coefficient
//!    was zero).
//! 4. Inverse-quantises the resulting integer spectrum.
//!
//! Pulse data is only valid for long windows (window_sequence != EIGHT_SHORT).
//! It is rarely emitted by mainstream encoders but is part of AAC-LC and
//! shows up in test vectors.

use oxideav_core::{Error, Result};

use crate::bitreader::BitReader;
use crate::ics::{inv_quant, sf_to_gain, SPEC_LEN};
use crate::sfband::SWB_LONG;

/// Maximum number of pulses (`number_pulse + 1`).
pub const MAX_PULSES: usize = 4;

#[derive(Clone, Debug, Default)]
pub struct PulseData {
    /// Number of pulses present, 1..=4.
    pub number_pulse: u8,
    pub pulse_start_sfb: u8,
    pub pulse_offset: [u8; MAX_PULSES],
    pub pulse_amp: [u8; MAX_PULSES],
}

/// Parse `pulse_data()` per §4.6.5.
pub fn parse_pulse_data(br: &mut BitReader<'_>) -> Result<PulseData> {
    let number_pulse_minus_one = br.read_u32(2)? as u8;
    let pulse_start_sfb = br.read_u32(6)? as u8;
    let mut pulse_offset = [0u8; MAX_PULSES];
    let mut pulse_amp = [0u8; MAX_PULSES];
    let count = (number_pulse_minus_one as usize) + 1;
    for i in 0..count {
        pulse_offset[i] = br.read_u32(5)? as u8;
        pulse_amp[i] = br.read_u32(4)? as u8;
    }
    Ok(PulseData {
        number_pulse: count as u8,
        pulse_start_sfb,
        pulse_offset,
        pulse_amp,
    })
}

/// Apply pulse data to a long-window spectrum that has already been
/// inverse-quantised and scaled by `sf_to_gain(sf)`. We undo the inverse
/// quantisation locally, add the pulse to the integer coefficient, and
/// redo the inverse quantisation.
pub fn apply_pulse_long(
    spec: &mut [f32; SPEC_LEN],
    pd: &PulseData,
    sf_index: u8,
    max_sfb: u8,
    sf: &[i32],
) -> Result<()> {
    let swb = SWB_LONG[sf_index as usize];
    let start_sfb = pd.pulse_start_sfb as usize;
    if start_sfb >= max_sfb as usize {
        return Err(Error::invalid("AAC: pulse_start_sfb beyond max_sfb"));
    }
    let mut k = swb[start_sfb] as usize;
    // Track the current sfb so we know which scalefactor's gain applies.
    let mut sfb_now = start_sfb;
    for i in 0..pd.number_pulse as usize {
        k = k.wrapping_add(pd.pulse_offset[i] as usize);
        if k >= SPEC_LEN {
            return Err(Error::invalid("AAC: pulse_offset past SPEC_LEN"));
        }
        // Find which sfb owns this coefficient.
        while sfb_now + 1 < max_sfb as usize && (k as u16) >= swb[sfb_now + 1] {
            sfb_now += 1;
        }
        let scale = sf_to_gain(*sf.get(sfb_now).unwrap_or(&0));
        let inv_scale = if scale == 0.0 { 0.0 } else { 1.0 / scale };
        // Recover integer-quantised value: q = sign * (|spec/scale|)^(3/4).
        let v = spec[k] * inv_scale;
        let abs_v = v.abs();
        let q_abs = if abs_v == 0.0 { 0.0 } else { abs_v.powf(0.75) };
        let q_round = q_abs.round();
        // Sign of original coefficient (treat zero as positive for the pulse).
        let sign = if v < 0.0 { -1.0 } else { 1.0 };
        let new_q_abs = q_round + pd.pulse_amp[i] as f32;
        spec[k] = sign * inv_quant(new_q_abs) * scale;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bitwriter::BitWriter;

    #[test]
    fn parse_single_pulse_roundtrip() {
        let mut bw = BitWriter::new();
        bw.write_u32(0, 2); // number_pulse_minus_one = 0 → 1 pulse
        bw.write_u32(7, 6); // pulse_start_sfb
        bw.write_u32(3, 5); // pulse_offset[0]
        bw.write_u32(5, 4); // pulse_amp[0]
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let pd = parse_pulse_data(&mut br).unwrap();
        assert_eq!(pd.number_pulse, 1);
        assert_eq!(pd.pulse_start_sfb, 7);
        assert_eq!(pd.pulse_offset[0], 3);
        assert_eq!(pd.pulse_amp[0], 5);
    }

    #[test]
    fn apply_pulse_changes_target_coefficient() {
        // Set up a long spectrum with one non-zero coefficient.
        let mut spec = [0.0f32; SPEC_LEN];
        let sf_index = 4u8; // 44.1 kHz
        let max_sfb = 30u8;
        let mut sf = vec![100i32; max_sfb as usize];
        sf[5] = 100;
        // Coefficient at swb[5] + 0; pre-fill with 0 so the pulse adds amp.
        let pd = PulseData {
            number_pulse: 1,
            pulse_start_sfb: 5,
            pulse_offset: [0, 0, 0, 0],
            pulse_amp: [3, 0, 0, 0],
        };
        apply_pulse_long(&mut spec, &pd, sf_index, max_sfb, &sf).unwrap();
        let target = SWB_LONG[sf_index as usize][5] as usize;
        // amp=3, sf=100 ⇒ scale=1, inv_quant(3) = 3^(4/3).
        let expected = inv_quant(3.0);
        assert!(
            (spec[target] - expected).abs() < 1e-4,
            "got {} expected {}",
            spec[target],
            expected
        );
    }
}

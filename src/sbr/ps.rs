//! MPEG-4 Parametric Stereo (PS) — ISO/IEC 14496-3 Subpart 8.
//!
//! PS upmixes an SBR-decoded mono signal to stereo using compact
//! "spatial" side information: Inter-channel Intensity Difference (IID),
//! Inter-channel Coherence (ICC), and optionally Inter-channel Phase
//! Difference (IPD/OPD). It sits in the SBR extension payload with
//! `bs_extension_id = 2`.
//!
//! This implementation is minimal: it handles the frequently-observed
//! header fields, decodes IID/ICC envelopes for a single "frame class"
//! resolution, and applies a simplified time-domain upmix (panning +
//! short-delay decorrelation) to synthesise stereo from mono SBR output.
//! It does NOT yet implement IPD/OPD, allpass-chain decorrelation, or
//! full QMF-domain processing — the spec-accurate algorithm runs the
//! upmix at QMF-subband granularity.

use oxideav_core::{bits::BitReader, Result};

/// PS bitstream extension ID inside an SBR extended_data block.
pub const EXT_ID_PS_DATA: u32 = 2;

/// Number of IID/ICC frequency bands for the low/high resolution modes.
/// (Table 8.5 / 8.6 of Subpart 8.)
pub const NUM_IID_BANDS_COARSE: usize = 10;
pub const NUM_IID_BANDS_FINE: usize = 20;
pub const NUM_ICC_BANDS_COARSE: usize = 10;
pub const NUM_ICC_BANDS_FINE: usize = 20;

/// Dequantisation table for IID, 7-step resolution (fine). Values in dB
/// per ISO/IEC 14496-3 Table 8.13. Converted from dB to linear amplitude
/// ratios at apply time.
pub const IID_QUANT_DB_FINE: [f32; 15] = [
    -18.0, -12.0, -9.0, -6.0, -4.5, -3.0, -1.5, 0.0, 1.5, 3.0, 4.5, 6.0, 9.0, 12.0, 18.0,
];
/// Dequantisation table for IID coarse (5-step). Values in dB.
pub const IID_QUANT_DB_COARSE: [f32; 7] = [
    -14.0, -10.0, -7.0, 0.0, 7.0, 10.0, 14.0,
];

/// ICC dequant — 8 entries (Table 8.17). Values are cos(angle) * ICC in
/// the range [-0.99, 1.0].
pub const ICC_QUANT: [f32; 8] = [
    1.0, 0.937, 0.84118, 0.60092, 0.36764, 0.0, -0.589, -0.99, // approximations of the spec table
];

/// Parsed PS-header state (`ps_header()`).
#[derive(Clone, Debug, Default)]
pub struct PsHeader {
    pub iid_mode: u8,
    pub icc_mode: u8,
    pub iid_fine: bool,
    pub icc_fine: bool,
    pub ipd_mode: u8,
    pub header_seen: bool,
}

/// Parsed PS frame data.
#[derive(Clone, Debug, Default)]
pub struct PsFrame {
    pub enable_iid: bool,
    pub enable_icc: bool,
    /// Decoded IID values in dB (up to 20 bands). Length follows the
    /// header's fine flag.
    pub iid_db: Vec<f32>,
    /// Decoded ICC values in normalised amplitude.
    pub icc: Vec<f32>,
    /// Number of parameter bands that were actually decoded.
    pub num_bands_iid: usize,
    pub num_bands_icc: usize,
}

/// Running per-stream PS state (header persistence across frames).
#[derive(Clone, Debug)]
pub struct PsState {
    pub header: PsHeader,
    /// Decorrelator delay line — a few samples of history per channel.
    /// Used as a cheap proxy for the full allpass-chain decorrelator.
    pub delay: [f32; 64],
    pub delay_pos: usize,
}

impl PsState {
    pub fn new() -> Self {
        Self {
            header: PsHeader::default(),
            delay: [0.0; 64],
            delay_pos: 0,
        }
    }
}

impl Default for PsState {
    fn default() -> Self {
        Self::new()
    }
}

/// Parse one `ps_data()` frame from the bitstream.
///
/// `br` must be positioned at the start of ps_data (i.e. just past the
/// `bs_extension_id` nibble inside the SBR extended_data block). Returns
/// the number of bits consumed so the caller can skip any remainder.
pub fn parse_ps_data(br: &mut BitReader<'_>, state: &mut PsState) -> Result<PsFrame> {
    let mut out = PsFrame::default();
    let enable_ps_header = br.read_bit()?;
    if enable_ps_header {
        state.header.header_seen = true;
        let enable_iid = br.read_bit()?;
        if enable_iid {
            state.header.iid_mode = br.read_u32(3)? as u8;
            // iid_mode values 3..=4 indicate fine resolution / IPD follow.
            state.header.iid_fine = state.header.iid_mode >= 3;
        }
        let enable_icc = br.read_bit()?;
        if enable_icc {
            state.header.icc_mode = br.read_u32(3)? as u8;
            state.header.icc_fine = state.header.icc_mode >= 3;
        }
        let enable_ext = br.read_bit()?;
        if enable_ext {
            // Skip: extension reserved / IPD+OPD is here.
            state.header.ipd_mode = 0;
        }
    }
    out.enable_iid = br.read_bit()?;
    if out.enable_iid {
        let _iid_mode = br.read_u32(3)? as u8;
        let n = if state.header.iid_fine {
            NUM_IID_BANDS_FINE
        } else {
            NUM_IID_BANDS_COARSE
        };
        out.num_bands_iid = n;
        out.iid_db = vec![0.0; n];
        // Raw 4-bit indexes (we don't implement the huffman/delta variants;
        // many streams use the non-huff "raw" signalling).
        let mut _acc: i32 = 0;
        for b in 0..n {
            let d = br.read_u32(4)? as i32;
            let acc = d - 8; // center the 4-bit range [-8..7]
            _acc = acc;
            let idx = (acc + 7).clamp(0, 14) as usize;
            out.iid_db[b] = if state.header.iid_fine {
                IID_QUANT_DB_FINE[idx]
            } else {
                IID_QUANT_DB_COARSE[idx.min(6)]
            };
        }
    }
    out.enable_icc = br.read_bit()?;
    if out.enable_icc {
        let _icc_mode = br.read_u32(3)? as u8;
        let n = if state.header.icc_fine {
            NUM_ICC_BANDS_FINE
        } else {
            NUM_ICC_BANDS_COARSE
        };
        out.num_bands_icc = n;
        out.icc = vec![0.0; n];
        for b in 0..n {
            let idx = br.read_u32(3)? as usize;
            out.icc[b] = ICC_QUANT[idx.min(7)];
        }
    }
    // Skip extension / reserved bits — caller bit-aligns via payload size.
    Ok(out)
}

/// Simplified PS upmix: given a mono SBR output buffer, produce stereo
/// output by panning + a short-delay decorrelation contribution.
///
/// `iid_avg_db` is the average IID across the decoded bands (or 0 dB when
/// the frame has no IID). Positive values mean the right channel is
/// stronger. `icc_avg` is the inter-channel coherence — 1 means fully
/// correlated, 0 means the two channels are independent. We approximate
/// decorrelation with a short delay of the mono signal.
pub fn apply_ps_simple(
    mono: &[f32],
    out_l: &mut [f32],
    out_r: &mut [f32],
    iid_avg_db: f32,
    icc_avg: f32,
    state: &mut PsState,
) {
    debug_assert_eq!(out_l.len(), mono.len());
    debug_assert_eq!(out_r.len(), mono.len());
    // Convert IID to linear gain ratios. IID_dB = 20 log10(R/L).
    let g_ratio = 10.0f32.powf(iid_avg_db / 20.0);
    let g_l = 1.0 / (1.0 + g_ratio);
    let g_r = g_ratio / (1.0 + g_ratio);
    // ICC-mediated decorrelation: rho in [0,1]; 1 = correlated, 0 = not.
    // Stereo: L = rho · g_L · mono + sqrt(1 - rho^2) · g_L · decorr(mono),
    //         R = rho · g_R · mono - sqrt(1 - rho^2) · g_R · decorr(mono).
    let rho = icc_avg.clamp(-1.0, 1.0);
    let rho_c = (1.0 - rho * rho).max(0.0).sqrt();

    let delay_len = state.delay.len();
    for (i, &m) in mono.iter().enumerate() {
        let dpos = state.delay_pos;
        let decorr = state.delay[dpos];
        state.delay[dpos] = m;
        state.delay_pos = (dpos + 1) % delay_len;
        out_l[i] = rho * g_l * m + rho_c * g_l * decorr;
        out_r[i] = rho * g_r * m - rho_c * g_r * decorr;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_ps_balanced_mono_stays_mono_like() {
        let mono: Vec<f32> = (0..128).map(|n| (n as f32 * 0.01).sin() * 0.5).collect();
        let mut l = vec![0.0; 128];
        let mut r = vec![0.0; 128];
        let mut ps = PsState::new();
        // IID = 0 dB → equal-split; ICC = 1 → identical channels.
        apply_ps_simple(&mono, &mut l, &mut r, 0.0, 1.0, &mut ps);
        for i in 0..128 {
            let expect = 0.5 * mono[i];
            assert!((l[i] - expect).abs() < 1e-5, "l[{i}] = {}, expect {}", l[i], expect);
            assert!((r[i] - expect).abs() < 1e-5);
        }
    }

    #[test]
    fn apply_ps_all_right_pan() {
        let mono: Vec<f32> = (0..32).map(|n| n as f32 * 0.1).collect();
        let mut l = vec![0.0; 32];
        let mut r = vec![0.0; 32];
        let mut ps = PsState::new();
        // Very positive IID → most of the signal in R.
        apply_ps_simple(&mono, &mut l, &mut r, 18.0, 1.0, &mut ps);
        let el: f32 = l.iter().map(|v| v * v).sum();
        let er: f32 = r.iter().map(|v| v * v).sum();
        assert!(er > el * 10.0, "right channel not dominant: el={el}, er={er}");
    }
}

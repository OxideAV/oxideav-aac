//! Huffman codeword reordering (HCR) for AAC spectral data — ISO/IEC
//! 14496-3 §4.6.16.3.
//!
//! HCR is the error-resilience tool selected by
//! `aacSpectralDataResilienceFlag` (the Table 4.50 spectral branch
//! parsed by [`crate::ics_body::IcsBody::parse_er`]): the
//! `reordered_spectral_data()` block carries the same Huffman
//! codewords as a non-resilient `spectral_data()`, but the *priority*
//! codewords (PCWs) are placed at known segment boundaries so a bit
//! error inside one codeword cannot propagate into them.
//!
//! ## What this module covers (round 375)
//!
//! The deterministic, header-only **scaffolding** of HCR — the parts
//! that depend only on the two transmitted length fields, the active
//! `section_data()` codebooks, and the window geometry, *not* on the
//! reordered bit payload itself:
//!
//! * The §4.6.16.3.3.1 pre-sorting **priority metric**
//!   ([`codebook_priority`], [`assigned_unit_nr`]) — the
//!   `codebookPriority[32]` table and the `assignedUnitNr` formula
//!   that determines which codewords become PCWs.
//! * The §4.6.16.3.3.2 **segment width / instantiation** math
//!   ([`MAX_CW_LEN`], [`segment_width`], [`Segmentation::new`]) — the
//!   `segmentWidth = min(maxCwLen, length_of_longest_codeword)`
//!   derivation and the segment count / last-segment-remainder rule
//!   sized by `length_of_reordered_spectral_data`.
//!
//! The full §4.6.16.3.4 reordered-payload **decode** (the PCW /
//! non-PCW `WriteCodewordToSegment` trial loop inverted to recover the
//! codeword bit positions) keys off this scaffold and is a later
//! milestone; it needs an HCR-bearing conformance stream to validate
//! bit-exactly.
//!
//! ## Provenance
//!
//! Every constant and formula here is from ISO/IEC 14496-3
//! §4.6.16.3.3 / §4.6.16.3.5 (Table 4.170, the `codebookPriority[32]`
//! and `assignedUnitNr` listings) staged under `docs/audio/aac/`. The
//! `maxCwLen` column of Table 4.170 is a numeric data table; the
//! pre-sorting metric is the spec's own arithmetic. No external HCR
//! implementation was consulted.

/// `maxCwLen[cb]` — the maximum Huffman codeword length, in bits, for
/// each spectral codebook (ISO/IEC 14496-3 Table 4.170).
///
/// Indexed by the raw `sect_cb` value (`0..=31`): the base §4.A.1
/// books are `0..=11`, the §4.6.16.4 virtual codebooks (used only in
/// the error-resilient `section_data()` 5-bit branch) are `16..=31`.
/// Codebook `0` (`ZERO_HCB`) and the reserved gaps `12..=15` carry no
/// codeword, so their entry is `0`.
pub const MAX_CW_LEN: [u8; 32] = [
    0,  // 0  ZERO_HCB
    11, // 1
    9,  // 2
    20, // 3
    16, // 4
    13, // 5
    11, // 6
    14, // 7
    12, // 8
    17, // 9
    14, // 10
    49, // 11 ESC_HCB
    0,  // 12 reserved
    0,  // 13 NOISE_HCB (no spectral codeword)
    0,  // 14 INTENSITY_HCB2 (no spectral codeword)
    0,  // 15 INTENSITY_HCB (no spectral codeword)
    14, // 16 virtual
    17, // 17 virtual
    21, // 18 virtual
    21, // 19 virtual
    25, // 20 virtual
    25, // 21 virtual
    29, // 22 virtual
    29, // 23 virtual
    29, // 24 virtual
    29, // 25 virtual
    33, // 26 virtual
    33, // 27 virtual
    33, // 28 virtual
    37, // 29 virtual
    37, // 30 virtual
    41, // 31 virtual
];

/// `codebookPriority[32]` — the §4.6.16.3.3.1 pre-sorting priority
/// assigned to each codebook (ISO/IEC 14496-3 §4.6.16.3.3.1).
///
/// Higher values are pre-sorted earlier (become PCWs). The `x`
/// entries in the spec — codebooks `0` (`ZERO_HCB`) and the reserved
/// `12..=15` — carry no spectral codeword and never participate in
/// reordering, so they are mapped to `0`.
///
/// The spec listing is:
/// `{x,21,21,20,20,19,19,18,18,17,17,0,x,x,x,x,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1}`.
pub const CODEBOOK_PRIORITY: [u8; 32] = [
    0,  // 0  (x — no codeword)
    21, // 1
    21, // 2
    20, // 3
    20, // 4
    19, // 5
    19, // 6
    18, // 7
    18, // 8
    17, // 9
    17, // 10
    0,  // 11 ESC_HCB
    0,  // 12 (x)
    0,  // 13 (x)
    0,  // 14 (x)
    0,  // 15 (x)
    16, // 16
    15, // 17
    14, // 18
    13, // 19
    12, // 20
    11, // 21
    10, // 22
    9,  // 23
    8,  // 24
    7,  // 25
    6,  // 26
    5,  // 27
    4,  // 28
    3,  // 29
    2,  // 30
    1,  // 31
];

/// The §4.6.16.3.3.1 pre-sorting priority of a raw codebook value.
///
/// Returns `0` for a codebook that carries no spectral codeword
/// (`ZERO_HCB`, the reserved `12..=15`, and any value `>= 32`).
#[must_use]
pub fn codebook_priority(cb: u8) -> u8 {
    CODEBOOK_PRIORITY.get(cb as usize).copied().unwrap_or(0)
}

/// `maxCwLen` for a raw codebook value (Table 4.170). Returns `0` for
/// a codebook that carries no spectral codeword or an out-of-range
/// value.
#[must_use]
pub fn max_cw_len(cb: u8) -> u8 {
    MAX_CW_LEN.get(cb as usize).copied().unwrap_or(0)
}

/// The §4.6.16.3.3.1 `assignedUnitNr` metric for one unit (a group of
/// four spectral lines = two 2-D or one 4-D codeword).
///
/// ```text
/// assignedUnitNr = ( codebookPriority[cb] * maxNrOfLinesInWindow
///                    + nrOfFirstLineInUnit ) * maxNrOfWindows + window
/// ```
///
/// * `cb` — the codebook of the unit (its priority drives the
///   energy-based second pre-sorting step).
/// * `max_lines_in_window` — `1024` for one long window, `128` for
///   eight short windows.
/// * `nr_of_first_line_in_unit` — the first spectral line index of
///   the unit (a multiple of 4: `0..=1020` long, `0..=124` short).
/// * `max_windows` — `1` long, `8` short.
/// * `window` — `0` long, `0..=7` short.
///
/// Units sorted ascending by this number give the pre-sorted codeword
/// order (PCWs first).
#[must_use]
pub fn assigned_unit_nr(
    cb: u8,
    max_lines_in_window: u32,
    nr_of_first_line_in_unit: u32,
    max_windows: u32,
    window: u32,
) -> u32 {
    (u32::from(codebook_priority(cb)) * max_lines_in_window + nr_of_first_line_in_unit)
        * max_windows
        + window
}

/// The §4.6.16.3.3.2 per-codebook segment width:
/// `segmentWidth = min(maxCwLen, length_of_longest_codeword)`.
///
/// `length_of_longest_codeword` is the transmitted 6-bit field
/// (clamped to `49` for the reserved `50..=63` per §4.6.16.3.2). A
/// codebook with no codeword (`maxCwLen == 0`) yields a zero-width
/// segment.
#[must_use]
pub fn segment_width(cb: u8, length_of_longest_codeword: u8) -> u8 {
    max_cw_len(cb).min(clamp_longest_codeword(length_of_longest_codeword))
}

/// Clamp the transmitted `length_of_longest_codeword` to its valid
/// range per §4.6.16.3.2: values `50..=63` are reserved and a current
/// decoder replaces them with `49`.
#[must_use]
pub fn clamp_longest_codeword(length_of_longest_codeword: u8) -> u8 {
    length_of_longest_codeword.min(49)
}

/// Clamp the transmitted `length_of_reordered_spectral_data` to its
/// valid range per §4.6.16.3.2.
///
/// The maximum is `6144` bits for an SCE / CCE / LFE and `12288` bits
/// for a CPE; larger values are reserved and a current decoder
/// replaces them with the valid maximum.
#[must_use]
pub fn clamp_reordered_length(length_of_reordered_spectral_data: u16, is_cpe: bool) -> u16 {
    let max = if is_cpe { 12288 } else { 6144 };
    length_of_reordered_spectral_data.min(max)
}

/// The §4.6.16.3.3.2 segment layout for one `reordered_spectral_data()`
/// block: the per-segment bit widths derived from the active codebooks
/// and the two transmitted length fields.
///
/// "Segments are instantiated until the available buffer is
/// exhausted, whereas the size of this buffer is given by
/// `length_of_reordered_spectral_data`. The remaining bits at the end
/// of the buffer increase the size of the last segment."
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Segmentation {
    /// Per-segment bit width, in segment-instantiation order. The
    /// final entry absorbs any remaining buffer bits, so it may exceed
    /// its codebook's `segmentWidth`.
    pub segment_bits: Vec<u32>,
    /// `length_of_reordered_spectral_data` (clamped) — the total
    /// buffer size in bits. `segment_bits` sums to exactly this.
    pub total_bits: u32,
}

impl Segmentation {
    /// Build the segment layout from the pre-sorted PCW segment widths
    /// and the (clamped) reordered-buffer length.
    ///
    /// `pcw_segment_widths` is the ordered list of
    /// `segmentWidth = min(maxCwLen, length_of_longest_codeword)` for
    /// each priority codeword in pre-sorted order. Segments are taken
    /// in order while their cumulative width fits the buffer; once the
    /// next full segment would overflow (or the list is exhausted), the
    /// remaining buffer bits are folded into the last instantiated
    /// segment.
    ///
    /// Returns an empty layout (`segment_bits` empty, `total_bits` as
    /// given) when the buffer is zero-length.
    #[must_use]
    pub fn new(pcw_segment_widths: &[u8], total_bits: u32) -> Self {
        let mut segment_bits: Vec<u32> = Vec::new();
        if total_bits == 0 {
            return Segmentation {
                segment_bits,
                total_bits,
            };
        }

        let mut used: u32 = 0;
        for &w in pcw_segment_widths {
            let w = u32::from(w);
            if used + w > total_bits {
                // The next full segment would overrun the buffer; stop
                // instantiating new segments. The remainder folds into
                // the last one below.
                break;
            }
            segment_bits.push(w);
            used += w;
        }

        // The remaining bits at the end of the buffer increase the size
        // of the last segment (§4.6.16.3.3.2). If no segment fit at all
        // (every width exceeds the whole buffer, or the width list is
        // empty), the whole buffer is one segment.
        let remainder = total_bits - used;
        if remainder > 0 {
            if let Some(last) = segment_bits.last_mut() {
                *last += remainder;
            } else {
                segment_bits.push(remainder);
            }
        }

        Segmentation {
            segment_bits,
            total_bits,
        }
    }

    /// `numberOfSegments` — the count of instantiated segments.
    #[must_use]
    pub fn number_of_segments(&self) -> usize {
        self.segment_bits.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn max_cw_len_table_spot_checks() {
        assert_eq!(max_cw_len(0), 0);
        assert_eq!(max_cw_len(1), 11);
        assert_eq!(max_cw_len(3), 20);
        assert_eq!(max_cw_len(11), 49);
        // No-codeword books.
        assert_eq!(max_cw_len(13), 0);
        assert_eq!(max_cw_len(15), 0);
        // Virtual codebooks.
        assert_eq!(max_cw_len(16), 14);
        assert_eq!(max_cw_len(31), 41);
        assert_eq!(max_cw_len(200), 0);
    }

    #[test]
    fn codebook_priority_table_spot_checks() {
        assert_eq!(codebook_priority(0), 0);
        assert_eq!(codebook_priority(1), 21);
        assert_eq!(codebook_priority(2), 21);
        assert_eq!(codebook_priority(10), 17);
        assert_eq!(codebook_priority(11), 0);
        assert_eq!(codebook_priority(16), 16);
        assert_eq!(codebook_priority(31), 1);
        assert_eq!(codebook_priority(99), 0);
    }

    #[test]
    fn assigned_unit_nr_long_window() {
        // Long window: max_lines=1024, max_windows=1, window=0.
        // unit at line 0, cb 1 (priority 21): 21*1024 + 0 = 21504.
        assert_eq!(assigned_unit_nr(1, 1024, 0, 1, 0), 21504);
        // Same line, higher cb 11 (priority 0): just the line offset.
        assert_eq!(assigned_unit_nr(11, 1024, 0, 1, 0), 0);
        // A higher-priority codebook sorts ahead of a lower one at the
        // same line.
        assert!(assigned_unit_nr(1, 1024, 4, 1, 0) > assigned_unit_nr(31, 1024, 4, 1, 0));
    }

    #[test]
    fn assigned_unit_nr_short_window_interleaves_window() {
        // Short window: max_lines=128, max_windows=8. Two units at the
        // same line + codebook but different windows order by window.
        let a = assigned_unit_nr(5, 128, 8, 8, 0);
        let b = assigned_unit_nr(5, 128, 8, 8, 3);
        assert_eq!(b - a, 3);
    }

    #[test]
    fn segment_width_is_min_of_maxcwlen_and_longest() {
        // cb 3 maxCwLen 20, longest 16 → 16.
        assert_eq!(segment_width(3, 16), 16);
        // cb 2 maxCwLen 9, longest 16 → 9.
        assert_eq!(segment_width(2, 16), 9);
        // longest in reserved range clamps to 49.
        assert_eq!(segment_width(11, 60), 49);
    }

    #[test]
    fn clamp_reordered_length_per_element_kind() {
        assert_eq!(clamp_reordered_length(7000, false), 6144);
        assert_eq!(clamp_reordered_length(7000, true), 7000);
        assert_eq!(clamp_reordered_length(20000, true), 12288);
        assert_eq!(clamp_reordered_length(100, false), 100);
    }

    #[test]
    fn segmentation_folds_remainder_into_last_segment() {
        // Three PCW segments of width 10, 10, 10; buffer of 35 bits.
        // All three fit (30 bits); the trailing 5 bits fold into the
        // last segment → 10, 10, 15.
        let seg = Segmentation::new(&[10, 10, 10], 35);
        assert_eq!(seg.segment_bits, vec![10, 10, 15]);
        assert_eq!(seg.number_of_segments(), 3);
        assert_eq!(seg.segment_bits.iter().sum::<u32>(), 35);
    }

    #[test]
    fn segmentation_stops_before_overrun() {
        // Widths 10, 10, 10 but only 25 bits of buffer: two full
        // segments fit (20 bits); the third would overrun, so the
        // remaining 5 bits fold into the second segment → 10, 15.
        let seg = Segmentation::new(&[10, 10, 10], 25);
        assert_eq!(seg.segment_bits, vec![10, 15]);
        assert_eq!(seg.segment_bits.iter().sum::<u32>(), 25);
    }

    #[test]
    fn segmentation_single_segment_when_first_width_exceeds_buffer() {
        // First width 50 > buffer 30: no full segment fits; the whole
        // buffer becomes one segment.
        let seg = Segmentation::new(&[50, 50], 30);
        assert_eq!(seg.segment_bits, vec![30]);
        assert_eq!(seg.number_of_segments(), 1);
    }

    #[test]
    fn segmentation_zero_buffer_is_empty() {
        let seg = Segmentation::new(&[10, 10], 0);
        assert!(seg.segment_bits.is_empty());
        assert_eq!(seg.number_of_segments(), 0);
        assert_eq!(seg.total_bits, 0);
    }

    #[test]
    fn segmentation_exact_fit_no_remainder() {
        let seg = Segmentation::new(&[8, 8, 8], 24);
        assert_eq!(seg.segment_bits, vec![8, 8, 8]);
        assert_eq!(seg.segment_bits.iter().sum::<u32>(), 24);
    }
}

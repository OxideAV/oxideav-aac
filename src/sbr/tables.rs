//! SBR tables — Huffman codebooks, QMF window coefficients.
//!
//! All data transcribed from ISO/IEC 14496-3:2009 annex 4.A.6.

// The generated normative Annex D / Annex 4.A float tables included below keep
// the exact literal precision printed in the spec. The f32 values the compiler
// materializes are identical to a shorter form, but we preserve the spec's
// printed digits as a traceability aid, so clippy's excessive_precision lint
// is silenced for the included data.
#![allow(clippy::excessive_precision)]

include!("tables_data.rs");

/// Largest Absolute Value for each SBR Huffman table. A decoded Huffman
/// index `i` corresponds to delta = `i - LAV` (§4.A.6.1).
pub const LAV_ENV_1_5DB: i32 = 60;
pub const LAV_ENV_BAL_1_5DB: i32 = 24;
pub const LAV_ENV_3_0DB: i32 = 31;
pub const LAV_ENV_BAL_3_0DB: i32 = 12;
pub const LAV_NOISE_3_0DB: i32 = 31;
pub const LAV_NOISE_BAL_3_0DB: i32 = 12;

/// Decode a Huffman code from the bitstream using a (length, codeword) table.
///
/// Returns the table index (the delta value needs `-LAV` applied by the
/// caller).
pub fn sbr_huff_decode(
    br: &mut oxideav_core::bits::BitReader<'_>,
    table: &[(u8, u32)],
) -> oxideav_core::Result<u32> {
    // Straightforward: keep accumulating bits, search for a matching code.
    // Max code length is 20 bits in our tables; we cap at 24 to be safe.
    let mut code: u32 = 0;
    let mut len: u32 = 0;
    loop {
        code = (code << 1) | br.read_u32(1)?;
        len += 1;
        if len > 24 {
            return Err(oxideav_core::Error::invalid(
                "SBR huff decode: code length exceeded 24 bits",
            ));
        }
        // Linear scan — tables are small enough (<=121 entries).
        for (i, &(l, cw)) in table.iter().enumerate() {
            if l as u32 == len && cw == code {
                return Ok(i as u32);
            }
        }
    }
}

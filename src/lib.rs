//! # oxideav-aac
//!
//! Pure-Rust AAC (Advanced Audio Coding) parsing — currently **Phase 1**
//! of the post-r111 orphan-rebuild lineage. Decode and encode bodies are
//! *not* wired up yet; this crate's public surface is limited to:
//!
//! * The [`adts`] module — ISO/IEC 13818-7 §1.A.2 *Audio Data Transport
//!   Stream* fixed-header parser (sync, profile, sampling-frequency
//!   index, channel configuration, frame length, raw-data-block count,
//!   CRC presence flag).
//! * The [`asc`] module — ISO/IEC 14496-3 §1.6.2.1 *AudioSpecificConfig*
//!   parser, including the §4.4.1 *GASpecificConfig* body for all
//!   General Audio audio-object types (AOTs 1, 2, 3, 4, 6, 7, 17, 19,
//!   20, 21, 22, 23) and the hierarchical SBR (AOT 5) / PS (AOT 29)
//!   outer-wrapper unwrap. Embeds an inline
//!   [`pce::Pce`](pce::Pce) when `channelConfiguration == 0`.
//! * The [`pce`] module — ISO/IEC 14496-3 §4.4.1.1 *program_config_element*
//!   parser. Used both standalone (inside [`raw_data_block`]) and inline
//!   inside [`asc`].
//! * The [`raw_data_block`] module — ISO/IEC 14496-3 §4.4.2.1 syntactic
//!   *raw_data_block()* walker that visits each `id_syn_ele` in order
//!   and stops cleanly at `END (0b111)`. Per-element bodies for
//!   SCE / CPE / CCE / LFE are **not** parsed yet — the walker emits an
//!   element-header event and the consumer is responsible for
//!   advancing the bit-reader past the body (subsequent rounds will
//!   internalise this). PCE is fully parsed. **Round 160** added the
//!   matching encoder-side [`raw_data_block::FrameAssembler`] — the
//!   bit-exact inverse, with a typed push-API
//!   (`push_channel_header` / `push_channel_body_bits` / `push_fill` /
//!   `push_data` / `push_pce` / `push_end`) that composes the existing
//!   per-tool writers (`IcsInfo::write`, `SectionData::write`, …) into
//!   a complete byte stream. **Round 165** adds
//!   [`pce::Pce::write`] (the bit-exact inverse of the round-126
//!   `Pce::parse`) and the matching
//!   [`raw_data_block::FrameAssembler::push_pce`] entry point, closing
//!   the last per-element writer gap in the `raw_data_block()` frame
//!   assembler.
//! * The [`ics_info`] module — ISO/IEC 14496-3 §4.4.6 / Table 4.6
//!   *ics_info()* parser. The first piece of Phase 2
//!   (channel-element body parsing) — surfaces the window-sequence /
//!   shape, `max_sfb`, `scale_factor_grouping`, the Main predictor
//!   side-info (AOT 1), and the LTP `ltp_data()` body
//!   (Table 4.55) when the wire bit selects it, plus the
//!   §4.5.2.3.4 derivations (`num_windows`, `num_window_groups`,
//!   `window_group_length[]`, `num_swb`). **Round 140** added the
//!   matching `IcsInfo::write` encoder primitive (and a public
//!   `write_ltp_data` helper) — the second encode-side syntax-element
//!   writer in the crate. Self-roundtrip (`write` → `parse`) is
//!   bit-perfect across every branch the parser handles, including
//!   the Main predictor + Table 4.55 LTP body for both the non-LD
//!   and the ER-AAC-LD forms.
//! * The [`section_data`] module — ISO/IEC 14496-3 §4.4.6 / ISO/IEC
//!   13818-7 §6.3 Table 17 *section_data()* parser, **plus** (round
//!   137) the matching `SectionData::write` encoder primitive. The
//!   parser assigns a Huffman codebook (`sect_cb`) to each run of
//!   scalefactor bands per window group via run-length escape
//!   coding, building the per-group `sfb_cb[g][sfb]` map that
//!   `scale_factor_data()` (next round) consumes. The encoder is its
//!   inverse: given the same `(window_sequence, max_sfb)` context it
//!   emits a bit-exact Table 17 stream. Self-roundtrip
//!   (`write` → `parse`) is bit-perfect across the long, EIGHT_SHORT,
//!   single-escape, double-escape, and exact-multiple-of-`sect_esc_val`
//!   branches. No Huffman decode yet — every field is fixed-width.
//! * The [`pulse_data`] module — ISO/IEC 14496-3 §4.4.6.3 / Table 4.7
//!   *pulse_data()* parser **and** encoder primitive (**new in round
//!   142**). The parser reads the 2-bit `number_pulse`, 6-bit
//!   `pulse_start_sfb`, and `number_pulse + 1` `(5-bit pulse_offset,
//!   4-bit pulse_amp)` records into [`pulse_data::PulseData`]; the
//!   writer serialises the same structure back bit-for-bit. Every
//!   field is fixed-width — no Huffman tables, no `swb_offset`
//!   dependence, and no surrounding-element state. The §4.6.13
//!   reconstruction loop (`k += swb_offset[pulse_start_sfb] +
//!   pulse_offset[j]; x_quant[…] ±= pulse_amp[j]`) is **not**
//!   performed; it needs `swb_offset_long_window[]` and the
//!   post-Huffman `x_quant` array that arrive with `spectral_data()`.
//! * The [`scale_factor_data`] module — ISO/IEC 14496-3 §4.4.6 /
//!   Table 4.53 (non-resilient branch) plus §4.6.3 / Table 4.A.1
//!   *scale_factor_data()* parser **and** encoder primitive
//!   (round 149, the fifth encode-side syntax-element writer in
//!   the crate). Carries the AAC scalefactor Huffman codebook
//!   (codebook 12) — 121 entries indexed `0..=120` with
//!   `index_offset = -60`, producing DPCM deltas in `-60..=+60`. The
//!   parser walks the per-`(g, sfb)` non-`ZERO_HCB` subsequence
//!   driven by [`section_data::SectionData::sfb_cb`] and dispatches
//!   between `hcod_sf[]` (ordinary spectrum / PNS-after-first / both
//!   intensity codebooks) and the 9-bit `dpcm_noise_nrg` PCM seed
//!   (first PNS band of the frame). The writer serialises the same
//!   structure back bit-for-bit and validates the in-memory record
//!   variants against the codebook map.
//!
//!   **Round 152** adds the §4.6.2.3.2 / §4.6.8.1.4 / §4.6.13 DPCM
//!   accumulator pair [`scale_factor_data::accumulate`] (decoder
//!   side) / [`scale_factor_data::differentiate`] (encoder side)
//!   that converts between transmitted DPCM deltas and absolute
//!   per-band quantities. Three independent tracks: spectrum
//!   scalefactors (seed `last_sf = global_gain`, range `0..=255`),
//!   intensity stereo positions (seed `last_is = 0`), and PNS noise
//!   energies (seed `last_nrg = global_gain - NOISE_OFFSET - 256`,
//!   first PNS band carries a 9-bit `uimsbf` literal). The §4.4.6
//!   error-resilient branch (`aacScalefactorDataResilienceFlag ==
//!   1`, RVLC with `rev_global_gain`, `sf_concealment`,
//!   `length_of_rvlc_sf`) is still **not** implemented; ER AAC-LD /
//!   scalable profiles that flip the resilience flag will need a
//!   sibling `scale_factor_data_rvlc()` module.
//! * The [`tns_data`] module — ISO/IEC 14496-3 §4.4.6 / Table 4.54
//!   *tns_data()* parser **and** encoder primitive (**new in round
//!   146**). The parser walks every transform window of the
//!   surrounding `window_sequence` and reads `n_filt[w]`
//!   (1 or 2 bits per Table 4.155), an optional `coef_res[w]`
//!   (when `n_filt[w] > 0`), then per-filter `length` (4 or 6 bits),
//!   `order` (3 or 5 bits), and — when `order > 0` — `direction`,
//!   `coef_compress`, and `order` × `coef[i]` magnitudes whose width
//!   is `(3 + coef_res) − coef_compress` per §4.6.9.3. The writer
//!   serialises the same structure back bit-for-bit. The §4.6.9.3
//!   `tns_decode_coef` LPC reconstruction (signed conversion,
//!   `iqfac` arcsine inverse-quantisation, Levinson-style conversion
//!   to LPC) and the §4.6.9.3 `tns_ar_filter` all-pole pass over the
//!   spectrum are **not** performed; they need the spectral context
//!   that arrives with the per-AOT IMDCT back-end.
//!
//! Public API surface that requires a decoder or encoder body still
//! returns [`Error::NotImplemented`]; the syntactic skeleton lives
//! entirely in the modules above.
//!
//! ## Provenance
//!
//! No external implementation (FFmpeg's `libav*` family, FDK-AAC,
//! FAAD, Nero AAC, libfaac, …) was consulted at any stage. Every
//! numeric
//! constant, bit layout, and clause reference in this crate is sourced
//! from the staged ISO/IEC 13818-7 and ISO/IEC 14496-3 PDFs under
//! `docs/audio/aac/`. The fixture descriptions in
//! `docs/audio/aac/aac-fixtures-and-traces.md` were consulted as a
//! cross-reference against the spec wording.
//!
//! ## Status (Phase 1 + Phase 2 begin)
//!
//! * ADTS fixed header parsing: **complete** (sync + 7-byte body).
//! * ADTS CRC validation: deferred; the parser surfaces the
//!   `protection_absent` flag but does not validate the trailing
//!   16-bit CRC when present.
//! * `raw_data_block()` walker: iterates `id_syn_ele` and stops at
//!   `END`; FIL / DSE / PCE bodies are fully consumed. SCE / CPE /
//!   CCE / LFE bodies start with [`ics_info`] then [`section_data`]
//!   then [`scale_factor_data`] (Phase 2 in progress); the optional
//!   [`pulse_data`] and [`tns_data`] tools now parse and write; the
//!   remaining channel-stream tools (`gain_control_data`,
//!   `spectral_data`) are deferred.

#![warn(missing_debug_implementations)]
#![warn(missing_docs)]

use oxideav_core::RuntimeContext;

pub mod adts;
pub mod asc;
pub mod ics_info;
pub mod pce;
pub mod pulse_data;
pub mod raw_data_block;
pub mod scale_factor_data;
pub mod section_data;
pub mod tns_data;

mod error;

pub use error::Error;

/// Result alias used throughout the crate.
pub type Result<T> = core::result::Result<T, Error>;

/// Codec-registry entry point. The Phase 1 skeleton does **not** wire
/// a [`Decoder`](oxideav_core::Decoder) or
/// [`Encoder`](oxideav_core::Encoder) into the runtime context — those
/// arrive in subsequent rounds once decode / encode bodies land.
pub fn register(_ctx: &mut RuntimeContext) {}

oxideav_core::register!("aac", register);

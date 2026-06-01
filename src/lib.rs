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
//!   **Round 177** extends the GA body with the `extensionFlag == 1`
//!   subtree (Table 4.1: AOT 22's `numOfSubFrame` + `layer_length`;
//!   the AOT-17 / 19 / 20 / 23 resilience triplet; the
//!   always-present `extensionFlag3` tail bit) and the Table 1.15
//!   trailing `epConfig` 2-bit field for every ER AOT in
//!   {17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 39}. `epConfig == 2`
//!   or `3` (which mandate `ErrorProtectionSpecificConfig()` parsing)
//!   surface as [`Error::UnsupportedEpConfig`]; an `extensionFlag3
//!   == 1` body — whose layout is reserved by the spec — surfaces as
//!   [`Error::UnsupportedAscExtensionFlag3`].
//!   **Round 192** adds the Table 1.15 trailing
//!   `syncExtensionType == 0x2b7` implicit-SBR probe (§1.6.5):
//!   when the outer AOT is **not** the explicit SBR (5) or PS (29)
//!   wrapper and the carrier has at least 16 bits remaining,
//!   [`AudioSpecificConfig::parse`] now reads an 11-bit
//!   `syncExtensionType` field. On a `0x2b7` match it consumes the
//!   nested `GetAudioObjectType()` plus either the SBR branch
//!   (`sbrPresentFlag`, optional `extensionSamplingFrequencyIndex`,
//!   then a second 11-bit `syncExtensionType == 0x548` gating a
//!   1-bit `psPresentFlag`) or the BSAC branch (`sbrPresentFlag`,
//!   optional `extensionSamplingFrequencyIndex`, mandatory 4-bit
//!   `extensionChannelConfiguration`). The probe result is exposed
//!   as [`asc::AudioSpecificConfig::trailing_sbr_probe`] and the
//!   implicitly-signalled SBR / PS / extension-sample-rate values
//!   are also propagated to the top-level `sbr_present` /
//!   `ps_present` / `extension_sampling_frequency_index` /
//!   `extension_sample_rate` / `extension_channel_configuration`
//!   fields. A carrier-bounded entry point
//!   [`asc::AudioSpecificConfig::parse_bits_bounded`] is exposed so
//!   LATM `StreamMuxConfig` (and any future esds AudioObj
//!   descriptor) callers can pass the exact ASC bit length;
//!   [`asc::AudioSpecificConfig::parse_bits`] preserves its
//!   no-probe semantics for callers that hold a `BitReader`
//!   carrying trailing carrier bytes.
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
//! * The [`gain_control_data`] module — ISO/IEC 14496-3 §4.4.6.5 /
//!   Table 4.12 *gain_control_data()* parser **and** encoder
//!   primitive (**new in round 183**). Carries the SSR (AOT 3)
//!   PQF-band gain-control ladder: 2-bit `max_band`, then for each
//!   `bd ∈ 1..=max_band` a per-window `(3-bit adjust_num) +
//!   adjust_num × (4-bit alevcode + W(seq, wd)-bit aloccode)` ladder
//!   with the per-`window_sequence` window count `N ∈ {1, 2, 8, 2}`
//!   and the per-`(seq, wd)` `aloccode` width table from Table 4.12
//!   (5 / 4-2 / 2 / 4-5). The §4.6.12 ladder-application loop that
//!   reconstructs sample-domain attenuation factors is **not**
//!   performed; it needs the SSR PQF / IMDCT back-end.
//! * The [`swb_offset`] module — ISO/IEC 14496-3 §4.5.4.1 / Tables
//!   4.129–4.141 *swb_offset_long_window[]* and
//!   *swb_offset_short_window[]* lookup tables, **new in round 194**.
//!   The per-band lowest-coefficient index for each of the 12 valid
//!   `samplingFrequencyIndex` values is exposed as
//!   [`swb_offset::SWB_OFFSET_LONG_WINDOW`] (each slot
//!   `num_swb + 1` entries with trailing 1024 sentinel) and
//!   [`swb_offset::SWB_OFFSET_SHORT_WINDOW`] (each slot
//!   `num_swb + 1` entries with trailing 128 sentinel). Public
//!   accessors [`swb_offset::long_window_offsets`] and
//!   [`swb_offset::short_window_offsets`] bounds-check
//!   `fs_index`. [`swb_offset::apply_pulse_data`] applies the
//!   §4.6.13 pulse-escape reconstruction to a long-window
//!   `x_quant` slice — the first reconstruction-layer entry point in
//!   the crate, consuming a parsed [`pulse_data::PulseData`] block
//!   and folding the `±pulse_amp` fix-up into the quantised
//!   spectrum at the running coefficient index `k = swb_offset[fs][
//!   pulse_start_sfb] + Σ pulse_offset[i]`. The 960-line frame
//!   variant (Tables 4.142–4.147) is **not** covered.
//! * The [`tns_max`] module — ISO/IEC 14496-3 §4.6.9.4 Tables
//!   4.102 / 4.103 decoder-side `TNS_MAX_ORDER` / `TNS_MAX_BANDS`
//!   clamp tables and §4.6.17.2.5 Tables 4.119 / 4.120 LD-specific
//!   `TNS_MAX_BANDS` tables, **new in round 200**. The accessors
//!   [`tns_max::tns_max_order`] and [`tns_max::tns_max_bands`]
//!   surface the per-AOT / per-window-sequence / per-`fs_index`
//!   caps; [`tns_max::tns_max_bands_ld_480`] and
//!   [`tns_max::tns_max_bands_ld_512`] handle the LD frame-size
//!   split. The clamp helpers [`tns_max::clamp_tns_order`] and
//!   [`tns_max::clamp_tns_band`] fold the §4.6.9.3 three-way
//!   `min(band, TNS_MAX_BANDS, max_sfb)` and
//!   `min(order, TNS_MAX_ORDER)` pseudocode into one call so the
//!   eventual TNS reconstruction layer can consume them without
//!   re-deriving the AOT dispatch. The Table 4.103 dispatch splits
//!   AOT 3 (AAC SSR) into the PQF-filterbank columns; every other
//!   AOT uses the non-PQF columns.
//! * The [`ics_body`] module — ISO/IEC 14496-3 §4.4.6 / Table 4.50
//!   `individual_channel_stream()` body walker, **new in round 207**.
//!   Composes the existing per-tool parsers / writers (`global_gain`,
//!   [`ics_info`], [`section_data`], [`scale_factor_data`], optional
//!   [`pulse_data`] / [`tns_data`] / [`gain_control_data`]) into the
//!   complete Table 4.50 channel-element body, **up to but not
//!   including** `spectral_data()`. Surfaces the parsed structure plus
//!   the `spectral_data_bit_offset` so the caller (e.g. a future
//!   spectrum parser, or a frame-assembler that hands off the
//!   spectrum-bit-slice via `push_channel_body_bits`) can resume the
//!   walk at the right boundary. The shared-info `CPE` form
//!   ([`ics_body::IcsBody::parse_with_ics_info`] /
//!   [`ics_body::IcsBody::write_with_ics_info`]) accepts the
//!   externally-held [`ics_info::IcsInfo`] for the per-channel body.
//!   Table 4.50 Note 1's "pulse_data illegal on
//!   `EIGHT_SHORT_SEQUENCE`" and the §4.6.12 "gain_control_data is
//!   AOT-3 (SSR) only" normative constraints are enforced on the
//!   writer side; the parser surfaces literal bits to keep hostile
//!   streams from panicking. `scale_flag == true` (scalable AAC, AOT
//!   6) rejects with [`Error::NotImplemented`].
//! * The [`extension_payload`] module — ISO/IEC 14496-3 §4.4.2.7 /
//!   Table 4.51 *extension_payload()* parser **and** encoder
//!   primitive (**new in round 187**). Implements the three
//!   non-SBR `extension_type` branches whose body layouts are
//!   fully specified by fixed-width fields: `EXT_FILL` (`0b0000`)
//!   — the Table 4.51 default branch surfacing the
//!   `8 * (cnt - 1) + 4` `other_bits` as a packed byte buffer;
//!   `EXT_FILL_DATA` (`0b0001`) — the normative-pattern filler
//!   with `fill_nibble == 0b0000` and `fill_byte ==
//!   0b1010_0101`; and `EXT_DYNAMIC_RANGE` (`0b1011`) — the
//!   Table 4.52 `dynamic_range_info()` block (optional
//!   `pce_instance_tag`, optional Table 4.53 `excluded_channels()`
//!   exclude-mask list, optional per-band partitioning, optional
//!   `prog_ref_level`, and per-band `(dyn_rng_sgn, dyn_rng_ctl)`
//!   records). The two SBR-data values from ISO/IEC 13818-7
//!   Table 40 (`EXT_SBR_DATA` `0b1101` and `EXT_SBR_DATA_CRC`
//!   `0b1110`) surface as [`Error::UnsupportedExtensionSbr`] —
//!   their bodies are `sbr_extension_data()` which needs the QMF /
//!   patching back-end this crate does not yet provide. The
//!   §4.5.2.13 DRC companding-curve application is **not**
//!   performed; the raw `(dyn_rng_sgn, dyn_rng_ctl)` records are
//!   surfaced verbatim for a later round.
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
//!   CCE / LFE bodies now compose through the new [`ics_body`]
//!   walker (Table 4.50): `global_gain` → [`ics_info`] →
//!   [`section_data`] → [`scale_factor_data`] → optional
//!   [`pulse_data`] / [`tns_data`] / [`gain_control_data`]. The
//!   remaining channel-stream tool (`spectral_data`) is deferred;
//!   `ics_body` surfaces the start bit-offset of that tool so the
//!   caller can hand off the spectrum slice to a future parser.

#![warn(missing_debug_implementations)]
#![warn(missing_docs)]

use oxideav_core::RuntimeContext;

pub mod adts;
pub mod asc;
pub mod extension_payload;
pub mod gain_control_data;
pub mod ics_body;
pub mod ics_info;
pub mod pce;
pub mod pulse_data;
pub mod raw_data_block;
pub mod scale_factor_data;
pub mod section_data;
pub mod swb_offset;
pub mod tns_data;
pub mod tns_max;

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

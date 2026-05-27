# oxideav-aac

A pure-Rust **AAC** (Advanced Audio Coding) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status (round 165)

**Phase 1 complete + Phase 2 in progress + five tool-level encoder
primitives + the §4.6.2.3.2 / §4.6.8.1.4 / §4.6.13 DPCM accumulator
pair + the §4.4.2.1 `raw_data_block()` frame assembler + the §4.4.1.1
`program_config_element()` writer.** Round 165 lands
[`pce::Pce::write`] — the encoder-side counterpart of the round-126
`Pce::parse`, emitting the full Table 4.2 wire layout (4-bit
`element_instance_tag` + 2-bit `object_type` + 4-bit
`sampling_frequency_index`; the six element-count fields with widths
4 / 4 / 4 / 2 / 3 / 4; three mix-down presence bits and their
optional 4 / 4 / (2+1)-bit bodies; per-element select lists for
front / side / back as `(is_cpe, tag_select)` pairs plus bare
4-bit `tag_select` lists for LFE / assoc and `(is_ind_sw,
tag_select)` pairs for `valid_cc`; a §4.4.1.1 Note 1 origin-relative
`byte_alignment()`; then the 8-bit `comment_field_bytes` length
prefix and payload bytes) — and the matching
[`raw_data_block::FrameAssembler::push_pce`] that emits the 3-bit PCE
`id_syn_ele` (`0b101`) before the body and passes
`origin_bit_offset = 0` so the byte-alignment collapses to the
absolute byte boundary inside the assembler. The origin handling
mirrors the parser exactly: pass `0` for a standalone PCE inside a
`raw_data_block()`, or the ASC origin bit-position for a PCE inline
in [`AudioSpecificConfig`](crate::asc::AudioSpecificConfig). A new
`Error::PceEncodeInvalid` variant surfaces every caller-side
wire-field violation: `element_instance_tag > 0x0f`, `object_type >
0x03`, `sampling_frequency_index > 0x0f`, `front`/`side`/`back`
element-list lengths > 15, LFE list length > 3, assoc list length >
7, CC list length > 15, any per-element `tag_select > 0x0f`,
`matrix_mixdown.idx > 0x03`, mono / stereo mix-down element numbers >
0x0f, or `comment_field.len() > 0xff`. 26 new integration tests cover
self-roundtrip (`write` → `parse`) across the empty PCE, a 5.1 layout
with mixed SCE / CPE / LFE / CC and a comment field, the
all-mix-downs-set permutation, the maximum-fields ladder
(`tag = 0x0f`, `object_type = 3`, `sfi = 0x0f`, 255-byte comment),
and a non-zero-origin parser / writer agreement test; six
`FrameAssembler::push_pce` Walker round-trip tests (PCE+END, PCE
after SCE header, PCE+FIL+END, full-mix-downs PCE) plus two error-
propagation tests; and a hand-pinned wire-layout assertion that the
all-zero PCE produces 6 all-zero bytes (header + counts + flags +
6-bit pad + comment length). Suite size grows 294 → 320 tests.

Round 160 had landed [`raw_data_block::FrameAssembler`] — the encoder-side
composition primitive that splices the existing typed writers
(`IcsInfo::write`, `SectionData::write`, `ScaleFactorData::write`,
`PulseData::write`, `TnsData::write`) into a complete bit-exact
`raw_data_block()` byte stream, the inverse of the round-121
`Walker`. The push-API covers every element type with a Phase-1 / 2
parser counterpart: `push_channel_header` for SCE / CPE / CCE / LFE
(3-bit `id_syn_ele` + 4-bit `element_instance_tag`),
`push_channel_body_bits` for splicing a pre-serialised body bit-slice
behind a header, `push_fill` for §4.4.2.7 FIL (with the 8-bit
`esc_count` escape inverting the parser's `cnt = esc_count + 15 − 1`
to `esc_count = payload_bytes − 14`, single-element fill capped at
269 bytes), `push_data` for §4.4.2.5 DSE (with the 8-bit `esc_count`
escape inverting `cnt = count + esc_count` to `esc_count =
payload_bytes − 255`, single-element data capped at 510 bytes, plus
the optional `data_byte_align_flag` honoured **before** the payload),
and `push_end` which emits the 3-bit `END (0b111)`, byte-aligns per
§4.4.2.1, and consumes the assembler to return the finished
`Vec<u8>`. END is mandatory — `push_end` is a destructor-style
terminator, so post-END pushes are a compile-time error rather than a
runtime check. **Round 165** now closes the PCE-writer gap with
[`raw_data_block::FrameAssembler::push_pce`] (3-bit `id_syn_ele`
prefix + the round-165 `Pce::write` body). The new
`Error::RawDataBlockEncodeInvalid` surfaces every caller-side wire-
field violation: non-channel `IdSynEle` on `push_channel_header`,
over-4-bit `element_instance_tag` on channel or DSE elements, FIL
payload above 269 bytes, DSE payload above 510 bytes, and
`push_channel_body_bits` `bit_count` exceeding `bits.len() × 8`. 30
new integration tests in `tests/raw_data_block_encode.rs` exercise
every entry point against the round-121 `Walker`, including
hand-pinned wire-byte assertions (`[0xE0]` END-only, `[0x15, 0xC0]`
SCE(tag=0x0a)+END, `[0xC1, 0xC0]` empty FIL+END, `[0x01, 0x57,
0x9C]` SCE(tag=0)+0xABC(12-bit body)+END), escape-boundary roundtrips
at FIL(15) / FIL(16) / FIL(269) / DSE(255) / DSE(510), and a
composite CPE+13-bit-body+DSE+FIL+END frame that walks every push
entry-point's bit-position handoff. Suite size grows 264 → 294
tests.

Round 152 had landed `accumulate()` / `differentiate()` — the
symmetric decoder / encoder pair that converts between transmitted
DPCM deltas and absolute per-band quantities. Three independent
tracks run in
lockstep: spectrum scalefactors (seed `last_sf = global_gain`, range
`0..=255`), intensity stereo positions (seed `last_is = 0`), and PNS
noise energies (seed `last_nrg = global_gain - NOISE_OFFSET - 256`
with `NOISE_OFFSET = 90`; the first PNS band of the frame carries
the 9-bit `uimsbf` literal directly into `last_nrg`, every
subsequent PNS band a `-60..=+60` Huffman delta). The §4.6.2.3.2
illustrative pseudocode that would single-track everything is
deliberately not implemented — the surrounding §4.6.8.1.4 / §4.6.13
prose "differential decoding is done separately between
scalefactors, intensity stereo positions and noise energies" takes
precedence (the §4.6.2.3.2 listing predates MPEG-4 PNS and is
identical in 13818-7 §11.3.2 where PNS does not exist). The
encoder-side `differentiate()` checks every spectrum / intensity /
PNS-subsequent delta against Table 4.150's `-60..=+60`, the first
PNS band's literal against the 9-bit `uimsbf` field (0..=511), and
the §4.6.2.3.2 Note's `sf ∈ 0..=255` range — rejections surface as
`Error::ScaleFactorAccumulatorInvalid`. Round 149 had landed the
`scale_factor_data()` parser **and** `ScaleFactorData::write`
encoder primitive (ISO/IEC 14496-3 §4.4.6 / Table 4.53,
non-resilient branch, plus the §4.6.3 / Table 4.A.1 "scalefactor
Huffman codebook" — codebook 12) — the fifth encode-side
syntax-element writer in the crate. Table 4.A.1 is transcribed
verbatim from the spec (121 entries indexed `0..=120`, max length
19 bits, codeword for index 60 / delta 0 is the single bit `0`)
and is a *complete* prefix code (Kraft equality, exhaustively
verified by walking all `2^19` 19-bit prefixes). Public helpers
`hcod_sf_encode(dpcm: i8) -> (length, codeword)` and
`hcod_sf_decode(reader) -> i8` expose the Table 4.A.1 codebook
directly; `SF_INDEX_OFFSET = -60` and `NOISE_PCM_BITS = 9`
constants pin the §4.6.3 / Table 4.150 dispatch. The parser /
writer walk the per-`(g, sfb)` non-`ZERO_HCB` subsequence driven
by `SectionData::sfb_cb`, dispatching between `hcod_sf[]` (ordinary
spectrum / PNS-after-first / both intensity codebooks 14 + 15)
and the 9-bit `dpcm_noise_nrg` PCM seed (first PNS band of the
frame; `noise_pcm_flag` is *frame*-scoped, not group-scoped). The
writer's structural sieve rejects every illegal in-memory shape
(`Error::ScaleFactorDataEncodeInvalid`): outer length mismatch
vs `sfb_cb`, missing or surplus entries, variant ↔ codebook
mismatch, `NoisePcm` re-used after `noise_pcm_flag` clears,
`NoiseDpcm` on the first PNS band, DPCM delta outside `-60..=+60`,
`NoisePcm` magnitude > 0x1FF. The §4.6.2.3.2 `last_sf =
global_gain` accumulator (DPCM → absolute `sf[g][sfb] ∈ 0..=255`)
is **not** performed; that's a per-AOT decoder step. The §4.4.6
error-resilient branch (`aacScalefactorDataResilienceFlag == 1`,
RVLC + `rev_global_gain` + `sf_concealment`) is **not** implemented;
ER AAC-LD / scalable profiles will need a sibling
`scale_factor_data_rvlc()` module. Round 146 had landed the
`tns_data()` parser **and** `TnsData::write` encoder primitive
(ISO/IEC 14496-3 §4.4.6 / Table 4.54, with the §4.6.9.2 Table 4.155
size switch) — the fourth encode-side syntax-element writer in the
crate, covering every transform window of the surrounding
`window_sequence` (`num_windows` = 8 for `EIGHT_SHORT_SEQUENCE`, 1
otherwise), with `n_filt` (1 or 2 bits), the optional `coef_res`
(when `n_filt > 0`), per-filter `length` (4 or 6 bits), `order`
(3 or 5 bits), and — when `order > 0` — `direction`,
`coef_compress`, and `order` × `coef[i]` magnitudes whose width is
`coef_bits = (3 + coef_res) − coef_compress ∈ {2, 3, 4}` per
§4.6.9.3. Public helpers `field_widths`, `num_windows`, and
`coef_bits` expose the Table 4.155 / §4.5.2.3.4 / §4.6.9.3
dispatch. Round 142 had landed `PulseData::parse` /
`PulseData::write` (§4.4.6.3 / Table 4.7) — the third encode-side
primitive, covering the 2-bit `number_pulse` + 6-bit
`pulse_start_sfb` + 1..=4 `(5-bit offset, 4-bit amp)` records.
Round 140 had landed `IcsInfo::write` (and a public `write_ltp_data`
helper) — the inverse of the round-129 `ics_info()` parser and the
crate's second encode-side syntax-element writer. The new primitive
covers the full Table 4.6 surface (`ics_reserved_bit` 1 +
`window_sequence` 2 + `window_shape` 1; `max_sfb` 4 + 7-bit grouping
mask on `EIGHT_SHORT_SEQUENCE`; `max_sfb` 6 + `predictor_data_present`
1 plus the per-AOT predictor / LTP body on every other window
sequence) and the Table 4.55 `ltp_data()` body for both the non-LD
11-bit-lag form and the ER-AAC-LD delta-coded form, with the second
`ltp_data_present` (+ optional paired body) for CPE common-window
streams. 35 new self-roundtrip integration tests exercise the long
branch, EIGHT_SHORT grouping, Main predictor (with and without reset,
including the `PRED_SFB_MAX[fs_index]` cap), non-LD LTP, ER-AAC-LD LTP
with and without `lag_update`, CPE common-window paired LTP, every
encoder-rejection branch (`Error::IcsInfoEncodeInvalid` for field
overflow, wrong AOT vs body slot, missing flag matching, stale data
on the predictor-absent branch, …), plus a hand-pinned wire-layout
assertion (`buf[0] == 0x06`, `buf[1] == 0x40` for a 25-band LC long
sequence). Round 137 had landed the `section_data()` writer; round
133 the `section_data()` parser; round 129 the `ics_info()` parser;
earlier rounds (121 / 126) the ADTS framing, out-of-band
`AudioSpecificConfig`, the `raw_data_block()` walker, and the
`program_config_element()` parser. The current capability set:

- **ADTS fixed + variable header parser** (ISO/IEC 13818-7
  §1.A.2.2.1–.2): syncword, ID, layer, `protection_absent`, profile,
  `sampling_frequency_index`, `channel_configuration`,
  `aac_frame_length`, `adts_buffer_fullness`, and
  `number_of_raw_data_blocks_in_frame` (decoded as `N`, not the wire
  `N − 1`). Sample-rate index is resolved against ISO/IEC 14496-3
  Table 1.18; reserved indices (13, 14, 15) are rejected. CRC bytes
  are confirmed present when `protection_absent == 0` but the CRC
  value itself is **not** validated yet.
- **`AudioSpecificConfig` parser** (ISO/IEC 14496-3 §1.6.2.1 +
  §4.4.1, **new in round 126**): full `GetAudioObjectType()`
  escape (5-bit base + 6-bit extension), `samplingFrequencyIndex`
  (4-bit + 24-bit explicit-rate escape at `0xf`),
  `channelConfiguration` (Table 1.19), and `GASpecificConfig`
  (Table 4.1) body for every GA `audioObjectType` (1, 2, 3, 4, 6,
  7, 17, 19, 20, 21, 22, 23) including `frameLengthFlag`
  (1024 vs 960 lines), `dependsOnCoreCoder` (+ optional 14-bit
  `coreCoderDelay`), `extensionFlag`, the AOT-6 / 20 `layerNr`,
  and an inline `program_config_element()` when
  `channelConfiguration == 0`. The hierarchical SBR (AOT 5) and
  PS (AOT 29) outer wrappers are unwrapped — `sbr_present` /
  `ps_present` flags + extension sampling-frequency index +
  inner AOT are all surfaced.
- **`program_config_element()` parser** (ISO/IEC 14496-3 §4.4.1.1
  / ISO/IEC 13818-7 §8.5 Table 25, **new in round 126**): all
  element counts (`num_front`, `num_side`, `num_back`, `num_lfe`,
  `num_assoc`, `num_valid_cc`); every per-element select
  (`*_element_is_cpe`, `*_element_tag_select`); mono / stereo /
  matrix mix-down hints; `byte_alignment()` honouring the Table 4.2
  Note 1 ASC-origin relative-alignment requirement; full
  `comment_field` round-trip. Both standalone-in-`raw_data_block`
  and inline-in-ASC call sites are supported via a single
  `Pce::parse(reader, origin_bit_offset)`.
- **`raw_data_block()` syntactic walker** (ISO/IEC 14496-3
  §4.4.2.1): iterates `id_syn_ele` and recognises the eight element
  types (SCE, CPE, CCE, LFE, DSE, PCE, FIL, END). Per-element body
  handling:
  - SCE / CPE / CCE / LFE: emit `ChannelElement { kind, tag }`
    after consuming the 3-bit id and 4-bit `element_instance_tag`.
    The channel-element body itself is not parsed yet.
  - FIL (§4.4.2.7): full count + `esc_count` parse; payload bytes
    are skipped.
  - DSE (§4.4.2.5): full header parse including
    `data_byte_align_flag`; payload bytes are skipped.
  - PCE (§4.4.1.1, **round 126**): full parse via the `pce` module;
    walker emits `Element::ProgramConfig(Pce)`.
  - END: byte-aligns the bit-reader and stops cleanly; further calls
    return `None`.
- **`ics_info()` parser** (ISO/IEC 14496-3 §4.4.6 Table 4.6, **new
  in round 129**): full Table 4.6 surface — `ics_reserved_bit`,
  `window_sequence` (Table 4.128), `window_shape`, `max_sfb` (4-bit
  EIGHT_SHORT / 6-bit long), `scale_factor_grouping` mask plus the
  Main AOT's `predictor_data()` body (`predictor_reset` +
  `reset_group_number` + `prediction_used[]` capped at the
  sample-rate-dependent `PRED_SFB_MAX` from ISO/IEC 13818-7
  Table 62) and every non-Main AOT's `ltp_data_present` chain.
  `ltp_data()` (Table 4.55) is fully parsed for both the ER-AAC-LD
  delta-coded form (AOT 23) and the unconditional 11-bit-lag form
  (every other GA AOT), including the second `ltp_data_present` (+
  optional second `ltp_data()`) for CPE common-window streams. The
  parser exposes the §4.5.2.3.4 derivations (`num_windows`,
  `num_window_groups`, `window_group_length[]`, `num_swb`)
  alongside the literal wire fields. Sample-rate count tables for
  long (Tables 4.129 / 4.131 / 4.132 / 4.134 / 4.136 / 4.138 /
  4.140) and short windows (Tables 4.130 / 4.133 / 4.135 / 4.137 /
  4.139 / 4.141) are encoded as `NUM_SWB_LONG_WINDOW[12]` /
  `NUM_SWB_SHORT_WINDOW[12]` constants.
- **`section_data()` parser** (ISO/IEC 14496-3 §4.4.6 / ISO/IEC
  13818-7 §6.3 Table 17, **new in round 133**): run-length section
  assignment with the §6.3 escape mechanism — per window group it
  reads `sect_cb` (4 bits) and accumulates `sect_len` from
  `sect_len_incr` fields (3-bit / `sect_esc_val == 7` for
  EIGHT_SHORT, 5-bit / `sect_esc_val == 31` otherwise), repeating
  the increment read while it equals the escape value. Produces the
  ordered per-group `Section { codebook, start, end }` lists plus
  the flattened `sfb_cb[g][sfb]` map that `scale_factor_data()`
  consumes. The codebook constants (`ZERO_HCB`, `FIRST_PAIR_HCB`,
  `ESC_HCB`, `NOISE_HCB`, `INTENSITY_HCB2`, `INTENSITY_HCB`) and a
  `Codebook` classifier (QUAD / PAIR signedness per Table 59,
  intensity / PNS / zero predicates) are exposed for the downstream
  tools. Every field is fixed-width — no Huffman decode yet.
- **`ics_info()` encoder primitive** (ISO/IEC 14496-3 §4.4.6
  Table 4.6 / Table 4.55, **new in round 140**):
  `IcsInfo::write(writer, audio_object_type, sampling_frequency_index,
  common_window)` — the inverse of the round-129 parser. Emits the
  bit-exact Table 4.6 stream: 1-bit `ics_reserved_bit`, 2-bit
  `window_sequence`, 1-bit `window_shape`, then either the 4-bit
  `max_sfb` + 7-bit `scale_factor_grouping` short-window pair, or the
  6-bit `max_sfb` + 1-bit `predictor_data_present` long-window pair
  with the per-AOT predictor body (Main: 1-bit `reset` + optional
  5-bit `reset_group_number` + `min(max_sfb, PRED_SFB_MAX[fs_index])`
  `prediction_used` bits; LTP / other GA: `write_ltp_data` for the
  primary channel + an optional second `ltp_data_present` + paired
  body when `common_window == true`). The companion
  `write_ltp_data(writer, ltp, aot, window_sequence, max_sfb)` covers
  the Table 4.55 body for both the non-LD 11-bit-lag form and the
  ER-AAC-LD (AOT 23) delta-coded form, including the `EIGHT_SHORT`
  omission of `ltp_long_used[]` in the non-LD branch. Caller-side
  structural bugs surface as `Error::IcsInfoEncodeInvalid`
  (`max_sfb` field-width overflow, wrong AOT vs predictor / LTP slot,
  missing or extra pair flag, `prediction_used[]` / `long_used[]`
  length mismatch, `coef`/`lag`/`reset_group_number` field overflow,
  populated predictor or LTP slot on the `EIGHT_SHORT` or
  predictor-absent branch).
- **`section_data()` encoder primitive** (ISO/IEC 14496-3 §4.4.6 /
  ISO/IEC 13818-7 §6.3 Table 17, **new in round 137**):
  `SectionData::write(writer, window_sequence, max_sfb)` — the
  inverse of the parser. Walks each window group's section list and
  emits `sect_cb` (4 bits) + the inverse-§6.3 `sect_len_incr`
  sequence: while remaining `sect_len >= sect_esc_val`, emit the
  escape value and subtract; emit the residual (in
  `[0, sect_esc_val)`) once at the end. The "greater than or equal
  to" boundary forces a trailing non-escape `sect_len_incr == 0`
  when the run length is an exact multiple of `sect_esc_val`,
  preserving the parser's loop-termination invariant. Validates
  caller-provided structure: per-group sections must be contiguous
  `[0, max_sfb)`, `sect_cb` must fit the 4-bit field, and
  zero-length sections are rejected (`Error::SectionDataEncodeInvalid`).
  No Huffman tables of its own — purely the fixed-width side of
  Table 17.
- **`pulse_data()` parser + encoder primitive** (ISO/IEC 14496-3
  §4.4.6.3 / Table 4.7, **new in round 142**):
  `PulseData::parse(reader)` and `PulseData::write(writer)` cover the
  full pulse-escape tool — 2-bit `number_pulse` (wire value
  `pulses.len() - 1`, so 1..=4 pulses), 6-bit `pulse_start_sfb`, then
  `number_pulse + 1` `(5-bit pulse_offset, 4-bit pulse_amp)` records.
  Every field is fixed-width; no Huffman tables, no `swb_offset`
  dependence, no surrounding-element state. Public constants
  (`PULSE_OFFSET_BITS`, `PULSE_AMP_BITS`, `MAX_PULSES`) pin the
  Table 4.7 widths for downstream call sites. The §4.6.13
  reconstruction loop (`k += swb_offset[pulse_start_sfb] +
  pulse_offset[j]; x_quant[…] ±= pulse_amp[j]`) is **not** performed
  here — it needs `swb_offset_long_window[]` and the post-Huffman
  `x_quant` array that arrive with `spectral_data()`. Caller-side
  structural bugs surface as `Error::PulseDataEncodeInvalid` (empty
  pulse list, `> 4` pulses, `pulse_start_sfb > 63`, `pulse_offset > 31`,
  or `pulse_amp > 15`).
- **`tns_data()` parser + encoder primitive** (ISO/IEC 14496-3
  §4.4.6 / Table 4.54 with the §4.6.9.2 Table 4.155 size switch,
  **new in round 146**): `TnsData::parse(reader, window_sequence)`
  walks every transform window (`num_windows` = 8 for
  `EIGHT_SHORT_SEQUENCE`, 1 otherwise) and reads `n_filt[w]` with
  the field-width switch (1 bit short / 2 bits long), an optional
  `coef_res[w]` (when `n_filt[w] > 0`), then per-filter
  `length[w][filt]` (4 / 6 bits), `order[w][filt]` (3 / 5 bits),
  and — when `order > 0` — `direction`, `coef_compress`, and
  `order` × `coef[i]` unsigned magnitudes whose width is
  `coef_bits = (3 + coef_res) − coef_compress ∈ {2, 3, 4}` per
  §4.6.9.3 `tns_decode_coef`. `TnsData::write(writer,
  window_sequence)` is the bit-exact inverse. Public helpers
  `field_widths(window_sequence)`, `num_windows(window_sequence)`,
  and `coef_bits(coef_res, coef_compress)` plus the
  `N_FILT_BITS_*` / `LENGTH_BITS_*` / `ORDER_BITS_*` /
  `COEF_RES_BITS` / `DIRECTION_BITS` / `COEF_COMPRESS_BITS`
  constants pin the Table 4.155 / §4.5.2.3.4 / §4.6.9.3 dispatch.
  The §4.6.9.3 `tns_decode_coef` LPC reconstruction (signed
  conversion, `iqfac` arcsine inverse-quantisation, Levinson-style
  conversion to LPC coefficients) and the §4.6.9.3 `tns_ar_filter`
  all-pole pass over the spectrum are **not** performed here — they
  need the spectral context that arrives with the per-AOT IMDCT
  back-end. The §4.6.9.4 `TNS_MAX_ORDER` / `TNS_MAX_BANDS` clamp
  tables (Tables 4.156 / 4.157) are similarly the decoder
  reconstruction layer's responsibility. Caller-side structural
  bugs surface as `Error::TnsDataEncodeInvalid` (wrong
  `windows.len()` for the surrounding `window_sequence`,
  `n_filt` / `length` / `order` field overflow, `coef[]` length
  mismatch with `order`, coef-value overflow at any
  `coef_bits` tier, or zero-`order` filter carrying a non-default
  `direction` / `coef_compress` that would be silently dropped on
  the wire).

## What Phase 2 still omits

- The remaining channel-stream tools after `scale_factor_data()`:
  `gain_control_data()` and `spectral_data()`. The walker therefore
  still cannot iterate past a single channel-element body.
- The `scale_factor_data()` error-resilient branch
  (`aacScalefactorDataResilienceFlag == 1` → RVLC with
  `rev_global_gain`, `length_of_rvlc_sf`, `sf_concealment`,
  `length_of_rvlc_escapes`); ER AAC-LD / scalable profiles need
  a sibling `scale_factor_data_rvlc()` module.
- The §4.6.13 pulse-escape *reconstruction* loop. The pulse-data
  record is parseable / writable bit-for-bit, but applying the
  fix-up to the decoded `x_quant` coefficients requires the
  `swb_offset_long_window[]` table (still owed) and an already-
  Huffman-decoded spectrum (still owed).
- The §4.6.9.3 `tns_decode_coef` LPC reconstruction and the
  §4.6.9.3 `tns_ar_filter` all-pole pass. The `tns_data()` block
  is parseable / writable bit-for-bit, but applying the all-pole
  filter to the dequantised MDCT spectrum requires both the LPC
  reconstruction (signed conversion + `iqfac` arcsine inverse-
  quantisation + Levinson-style conversion to LPC coefficients)
  and a real spectral context.
- The full per-AOT IMDCT + windowing back-end (filterbank, TNS
  reconstruction, PNS / IS pair routing).
- The decoder-side channel-element body walker that drives
  `ics_info` → `section_data` → `scale_factor_data` → optional
  `pulse_data` / `tns_data` / `gain_control_data` → `spectral_data`
  inside the per-channel `Element::ChannelElement` branch — the
  Phase 1 walker still stops after the 4-bit `element_instance_tag`
  and the caller must advance the bit-reader past the body manually.
- All decode / encode runtime wiring. The `register()` entry point
  installs no `Decoder` or `Encoder` into the runtime context yet,
  and the factory functions return `Error::NotImplemented`.
- The `GASpecificConfig` `extensionFlag == 1` body for ER AOTs
  (numOfSubFrame / layer_length / `aacSectionDataResilienceFlag` /
  `aacScalefactorDataResilienceFlag` /
  `aacSpectralDataResilienceFlag` / `extensionFlag3`).
- `epConfig` for ER object types — the ASC parser stops at the end
  of the GA body.
- The `syncExtensionType == 0x2b7` trailing-bits probe at the end
  of Table 1.15 (implicit-SBR signalling detection).
- LATM/LOAS transport (ISO/IEC 14496-3 §1.7.3).
- ADTS CRC-16 validation.
- The `swb_offset_long_window` / `swb_offset_short_window` /
  `sect_sfb_offset` tables — only the **count** of scalefactor
  bands is needed to step through `ics_info()` and `section_data()`
  (which works in band units); the offset tables arrive with
  `spectral_data()`.

## Provenance

No external implementation (FFmpeg's `libav*` family, FDK-AAC, FAAD,
Nero AAC, libfaac, …) was consulted at any stage. Every numeric
constant, bit-layout decision, and clause reference in this crate
is sourced from the staged ISO/IEC 13818-7 and ISO/IEC 14496-3 PDFs
under `docs/audio/aac/`.

## License

MIT — see [LICENSE](./LICENSE).

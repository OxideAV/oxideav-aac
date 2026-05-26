# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added (round 142 — third encoder primitive: pulse_data parser + writer)

- `pulse_data` module: ISO/IEC 14496-3 §4.4.6.3 / Table 4.7
  `pulse_data()` parser **and** encoder primitive. `PulseData::parse`
  reads the 2-bit `number_pulse`, 6-bit `pulse_start_sfb`, and
  `number_pulse + 1` `(5-bit pulse_offset, 4-bit pulse_amp)` records
  into a [`PulseData`] struct; `PulseData::write` is the bit-exact
  inverse. Every field is fixed-width; no Huffman tables, no
  `swb_offset` dependence, no surrounding-element state. Public
  constants `PULSE_OFFSET_BITS` (5), `PULSE_AMP_BITS` (4), and
  `MAX_PULSES` (4) pin the Table 4.7 widths for downstream callers.
  A `PulseData::number_pulse()` accessor returns the wire value
  (`pulses.len() - 1`). The §4.6.13 reconstruction loop (`k +=
  swb_offset[pulse_start_sfb] + pulse_offset[j]; x_quant[…] ±=
  pulse_amp[j]`) is **not** performed — it needs the
  `swb_offset_long_window[]` table and the post-Huffman `x_quant`
  array that arrive with `spectral_data()`.
- `Error::PulseDataEncodeInvalid` for caller-side structural bugs the
  writer cannot represent on the wire: empty `pulses` vector (the
  loop bound is `number_pulse + 1 >= 1`), `pulses.len() > MAX_PULSES`
  (2-bit `number_pulse` field cap of 4), `pulse_start_sfb > 0x3f`
  (6-bit overflow), `Pulse::offset > 0x1f` (5-bit overflow), or
  `Pulse::amp > 0x0f` (4-bit overflow).
- 22 new integration tests in `tests/pulse_data.rs` covering single-
  pulse smallest legal block, every legal pulse count (1..=4),
  wire-field-at-max (per individual field and all fields
  simultaneously), two hand-pinned wire-layout assertions (one
  synthetic 17-bit layout for a `(start=42, offset=21, amp=5)`
  single-pulse block; one realistic 26-bit layout for a
  `(start=32, [(3,2), (17,4)])` two-pulse block), every
  `PulseDataEncodeInvalid` rejection branch, parser unexpected-end at
  both the header and the pulse-loop positions, accessor / constant
  sanity, and a back-to-back two-block sequence that asserts the
  parser / writer carry no inter-block state. Suite size grows 139
  → 161 tests.

### Added (round 140 — second encoder primitive: ics_info writer)

- `ics_info::IcsInfo::write(writer, audio_object_type,
  sampling_frequency_index, common_window)` — the inverse of the
  round-129 `IcsInfo::parse` and the crate's second encode-side
  syntax-element writer. Emits the bit-exact ISO/IEC 14496-3 §4.4.6
  Table 4.6 stream: `ics_reserved_bit` (1 bit), `window_sequence`
  (2 bits), `window_shape` (1 bit), then either `max_sfb` (4 bits) +
  `scale_factor_grouping` (7 bits) for `EIGHT_SHORT_SEQUENCE`, or
  `max_sfb` (6 bits) + `predictor_data_present` (1 bit) plus the
  per-AOT predictor / LTP body for every other window sequence.
  Main-AOT (1) predictor side info covers `predictor_reset` +
  optional `predictor_reset_group_number` (5 bits) +
  `prediction_used[]` capped at `min(max_sfb, PRED_SFB_MAX[fs_index])`.
  Non-Main AOTs use the LTP branch, including the second
  `ltp_data_present` (+ optional paired `ltp_data()`) when
  `common_window == true`.
- `ics_info::write_ltp_data(writer, ltp, audio_object_type,
  window_sequence, max_sfb)` — Table 4.55 `ltp_data()` body writer.
  Covers both the non-LD 11-bit-lag form (with the `EIGHT_SHORT`
  omission of `ltp_long_used[]`) and the ER-AAC-LD (AOT 23)
  delta-coded form (`lag_update` flag + optional 10-bit `lag` + 3-bit
  `coef` + `long_used[]` capped at `MAX_LTP_LONG_SFB`).
- `Error::IcsInfoEncodeInvalid` for caller-side structural bugs the
  writer cannot represent on the wire: `max_sfb` exceeds its 4-/6-bit
  field width, `scale_factor_grouping` missing on EIGHT_SHORT or
  present on long, `predictor_data_present == true` on EIGHT_SHORT,
  a predictor / LTP body slot populated on the wrong AOT branch or
  on the predictor-absent branch, paired-channel LTP populated
  without `common_window`, predictor `reset_group_number` parity
  not matching the `reset` bit, `prediction_used[]` length not
  equal to `min(max_sfb, PRED_SFB_MAX[fs_index])`, `long_used[]`
  length not equal to `min(max_sfb, MAX_LTP_LONG_SFB)`, or a
  numeric field (`coef`, `lag`, `reset_group_number`,
  `scale_factor_grouping`) exceeding its wire-slot width.
- 35 new self-roundtrip integration tests in
  `tests/ics_info_encode.rs` — long branch (LC, KBD, `LongStart`,
  `LongStop` with reserved-bit set, `max_sfb == 0`), EIGHT_SHORT
  branch (no grouping, all grouped, mixed grouping mask), Main
  predictor (no reset, with reset, `PRED_SFB_MAX` cap), non-LD LTP
  (long, `MAX_LTP_LONG_SFB` cap, short via direct `write_ltp_data`
  call), ER-AAC-LD LTP (with and without `lag_update`),
  common-window paired LTP (present and absent), every
  `IcsInfoEncodeInvalid` rejection branch, and a hand-pinned
  wire-layout assertion (`buf[0] == 0x06`, `buf[1] == 0x40` for a
  25-band LC long sequence). Suite size grows 104 → 139 tests.

### Added (round 137 — first encoder primitive: section_data writer)

- `section_data::SectionData::write(writer, window_sequence, max_sfb)`
  — the inverse of `SectionData::parse` and the AAC crate's first
  encode-side syntax-element writer. Emits the bit-exact ISO/IEC
  14496-3 §4.4.6 / ISO/IEC 13818-7 §6.3 Table 17 stream that the
  existing parser reads back: per window group, for each section,
  4-bit `sect_cb` followed by an inverse-§6.3 `sect_len_incr`
  sequence — while remaining `sect_len >= sect_esc_val`, emit
  `sect_esc_val` and subtract; emit the residual (which is in
  `[0, sect_esc_val)`) once at the end. The "greater than or equal
  to" boundary forces a trailing non-escape `sect_len_incr == 0`
  whenever the section length is an exact multiple of
  `sect_esc_val`, preserving the parser's `break-on-non-escape`
  invariant.
- `Error::SectionDataEncodeInvalid` for caller-side structural bugs
  the writer cannot represent on the wire: non-contiguous per-group
  sections, gaps or overlaps in band coverage, `sect_cb` exceeding
  the 4-bit field, or zero-length sections.
- 18 new self-roundtrip integration tests in
  `tests/section_data_encode.rs` — long branch (no escape, single
  escape, double escape, exact-multiple terminator, double exact
  multiple), EIGHT_SHORT branch (no escape, single escape,
  exact-multiple terminator), multi-window-group with intensity and
  PNS codebooks, `max_sfb == 0` empty list, every
  `SectionDataEncodeInvalid` rejection branch, and a hand-pinned
  wire-layout assertion (`buf[0] == 0xB1`, `buf[1] == 0x91`,
  `buf[2] == 0x00` for `[(11, 3), (2, 4)]` with `max_sfb=7`). Each
  test feeds the encoded buffer back through the existing parser
  and asserts both structural equality and matching bit position.
  Suite size grows 86 → 104 tests.

### Added (round 133 — Phase 2: section_data)

- `section_data` module: ISO/IEC 14496-3 §4.4.6 / ISO/IEC 13818-7
  §6.3 Table 17 `section_data()` parser. Walks each window group's
  run-length-coded section list — reading `sect_cb` (4 bits) and
  accumulating `sect_len` from `sect_len_incr` fields with the §6.3
  escape mechanism (3-bit field / `sect_esc_val == 7` for
  `EIGHT_SHORT_SEQUENCE`, 5-bit field / `sect_esc_val == 31`
  otherwise). Produces the ordered per-group `Section { codebook,
  start, end }` lists, the flattened `sfb_cb[g][sfb]` codebook map,
  and a `num_sec(group)` accessor. Every field is fixed-width — no
  Huffman decode.
- Codebook constants `ZERO_HCB` (0), `FIRST_PAIR_HCB` (5),
  `ESC_HCB` (11), `NOISE_HCB` (13), `INTENSITY_HCB2` (14),
  `INTENSITY_HCB` (15) per ISO/IEC 13818-7 §9.2.2, plus a
  `Codebook` classifier (QUAD / PAIR dimension + `unsigned_cb[]`
  signedness per Table 59, with `is_intensity()` / `is_noise()` /
  `is_zero()` predicates) for downstream `scale_factor_data()` /
  `spectral_data()` dispatch.
- `Error::SectionDataOverrun` for a non-conforming `sect_len` that
  would extend a section past `max_sfb`.
- 17 new integration tests covering single / multi-section long and
  EIGHT_SHORT branches, single and double escape coding, multi
  window-group parsing, `max_sfb == 0`, overrun rejection,
  unexpected-end, the codebook constants and `Codebook`
  classification, and the fixtures-doc reference section trace —
  bringing the suite to 86 tests.

### Added (round 129 — Phase 2 begin: ics_info)

- `ics_info` module: ISO/IEC 14496-3 §4.4.6 Table 4.6
  `ics_info()` parser. Surfaces `ics_reserved_bit`,
  `window_sequence` (Table 4.128), `window_shape`, `max_sfb`
  (4-bit / 6-bit branching on `EIGHT_SHORT_SEQUENCE`),
  `scale_factor_grouping` mask, the Main AOT's `predictor_data()`
  body (`predictor_reset` + `predictor_reset_group_number` +
  `prediction_used[]`), and every non-Main AOT's `ltp_data_present`
  chain. `ltp_data()` (Table 4.55) is fully parsed for both the
  ER-AAC-LD (AOT 23) and non-LD forms, including the second
  `ltp_data_present` for CPE common-window streams. The parser
  also computes the §4.5.2.3.4 derivations (`num_windows`,
  `num_window_groups`, `window_group_length[]`, `num_swb`) and
  exposes them as struct fields plus a public
  `derive_window_grouping()` helper.
- Sample-rate count tables: `NUM_SWB_LONG_WINDOW[12]`,
  `NUM_SWB_SHORT_WINDOW[12]`, and `PRED_SFB_MAX[12]` constants —
  transcribed from ISO/IEC 14496-3 Tables 4.129–4.141 and ISO/IEC
  13818-7 Table 62.
- `Error::IcsInfoUnsupportedSampleRateIndex(u8)` for the case
  where a caller invokes `IcsInfo::parse` with the 24-bit
  explicit-rate escape (or a reserved index) without first
  resolving it to a standard 0..=11 index.
- 26 new integration tests covering long / EIGHT_SHORT / KBD,
  grouping-mask edge cases, Main predictor, LTP, ER-AAC-LD LTP,
  CPE common-window LTP, and unexpected-end — bringing the suite
  to 69 tests.

### Added (round 126 — configuration layer)

- `asc` module: ISO/IEC 14496-3 §1.6.2.1 `AudioSpecificConfig` parser
  + §4.4.1 `GASpecificConfig` body for every General Audio
  `audioObjectType` (1, 2, 3, 4, 6, 7, 17, 19, 20, 21, 22, 23).
  Hierarchical SBR (AOT 5) and PS (AOT 29) outer-wrappers are
  unwrapped and surfaced as `sbr_present` / `ps_present` plus the
  extension sampling-frequency index. `samplingFrequencyIndex` 24-bit
  escape (`0xf`) is honoured; `GetAudioObjectType` 5/6-bit escape is
  honoured. Non-GA AOTs return `Error::UnsupportedAot`.
- `pce` module: ISO/IEC 14496-3 §4.4.1.1 / ISO/IEC 13818-7 §8.5
  `program_config_element()` parser — every wire field is preserved
  (`front` / `side` / `back` / `lfe` / `assoc` / `cc` element
  lists, mono / stereo / matrix mix-down hints, comment field).
  `byte_alignment()` is configurable via the `origin_bit_offset`
  parameter to honour Table 4.2 Note 1 (ASC-relative alignment when
  the PCE is embedded inside an `AudioSpecificConfig`).
- `raw_data_block::Element::ProgramConfig(Pce)`: the walker now
  fully parses a PCE element inside a raw_data_block (previously
  `Error::UnsupportedElementSkip(5)`).
- 20 new integration tests (13 ASC, 7 PCE), bringing the suite to
  43 tests.

### Added (round 121 — Phase 1 syntactic skeleton)

- `adts` module: ISO/IEC 13818-7 §1.A.2.2.1–.2 ADTS fixed + variable
  header parser. Surfaces sync, ID, `protection_absent`, profile,
  `sampling_frequency_index`, `channel_configuration`,
  `aac_frame_length`, `adts_buffer_fullness`, and the resolved
  `number_of_raw_data_blocks_in_frame` (decoded as `N`, not the wire
  `N − 1`). Reserved `sampling_frequency_index` values are rejected;
  CRC presence is confirmed but the CRC value is not validated yet.
- `raw_data_block` module: ISO/IEC 14496-3 §4.4.2.1 syntactic walker
  that iterates `id_syn_ele` and recognises all eight element types
  (SCE / CPE / CCE / LFE / DSE / PCE / FIL / END). FIL and DSE bodies
  are fully skipped per §4.4.2.7 / §4.4.2.5; SCE/CPE/CCE/LFE emit
  the element header (`kind`, `element_instance_tag`) without
  parsing the body; PCE returns `Error::UnsupportedElementSkip`;
  END byte-aligns the reader and terminates the block.
- 23 unit/integration tests covering the new surface, including a
  synthetic SCE-FIL-END round-trip that wraps the walker inside a
  real ADTS frame.

### Erased

- Prior master history was force-erased on **2026-05-24** under
  Hat-3 cold enforcement of the workspace clean-room policy
  (`docs/IMPLEMENTOR_ROUND.md`). The retired implementation included
  encoder-source comments describing matching an external reference
  encoder's behaviour by citing a specific source file of that
  implementation. The clean-room policy forbids consulting any
  external implementation's source for any reason, regardless of
  licensing or technical merit.

### Reset

- Crate reduced to a minimal `oxideav_core::register!` stub. Every
  public API returns `Error::NotImplemented`. The crates.io version
  (`0.1.2`) is preserved on the new master to avoid breaking any
  downstream version pins; the published versions on crates.io will
  be yanked by the maintainer.

### Next

- Clean-room re-implementation against the staged ISO/IEC 14496-3 /
  13818-7 AAC specifications (numeric tables and decode behaviour
  read only from the standards) in a future round.

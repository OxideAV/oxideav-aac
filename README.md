# oxideav-aac

A pure-Rust **AAC** (Advanced Audio Coding) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status (round 129)

**Phase 1 complete + Phase 2 begin.** Round 129 starts the
channel-element body work with `ics_info()` — the first table the
decoder reaches inside SCE / CPE / LFE. Earlier rounds (121 / 126)
landed the ADTS framing, out-of-band `AudioSpecificConfig`, the
`raw_data_block()` walker, and the `program_config_element()`
parser. The current capability set:

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

## What Phase 2 still omits

- The remaining channel-stream tools after `ics_info()`:
  `section_data()`, `scale_factor_data()`, `pulse_data()`,
  `tns_data()`, `gain_control_data()`, and `spectral_data()`. The
  walker therefore still cannot iterate past a single
  channel-element body.
- The full per-AOT IMDCT + windowing back-end (filterbank, TNS
  reconstruction, PNS / IS pair routing).
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
  bands is needed to step through `ics_info()`; the offset tables
  arrive with `section_data()` / `spectral_data()`.

## Provenance

No external implementation (FFmpeg's `libav*` family, FDK-AAC, FAAD,
Nero AAC, libfaac, …) was consulted at any stage. Every numeric
constant, bit-layout decision, and clause reference in this crate
is sourced from the staged ISO/IEC 13818-7 and ISO/IEC 14496-3 PDFs
under `docs/audio/aac/`.

## License

MIT — see [LICENSE](./LICENSE).

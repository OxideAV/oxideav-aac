# oxideav-aac

A pure-Rust **AAC** (Advanced Audio Coding) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status (round 126)

**Phase 1 of the post-r111 orphan-rebuild lineage — syntactic
skeleton, now covers the full configuration layer.** This release
extends round 121's bitstream framing with the out-of-band ASC
descriptor and the inline / standalone PCE parser:

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

## What Phase 1 deliberately omits

- All decode / encode bodies. The `register()` entry point installs
  no `Decoder` or `Encoder` into the runtime context yet, and the
  factory functions return `Error::NotImplemented`.
- The `GASpecificConfig` `extensionFlag == 1` body for ER AOTs
  (numOfSubFrame / layer_length / `aacSectionDataResilienceFlag` /
  `aacScalefactorDataResilienceFlag` / `aacSpectralDataResilienceFlag`
  / `extensionFlag3`).
- `epConfig` for ER object types — the ASC parser stops at the end
  of the GA body.
- The `syncExtensionType == 0x2b7` trailing-bits probe at the end
  of Table 1.15 (implicit-SBR signalling detection).
- LATM/LOAS transport (ISO/IEC 14496-3 §1.7.3).
- ADTS CRC-16 validation.
- The SCE / CPE / CCE / LFE channel-element bodies (ICS,
  section/scale-factor/spectral data, TNS, IMDCT, filterbank).

## Provenance

No external implementation (FFmpeg's `libav*` family, FDK-AAC, FAAD,
Nero AAC, libfaac, …) was consulted at any stage. Every numeric
constant, bit-layout decision, and clause reference in this crate
is sourced from the staged ISO/IEC 13818-7 and ISO/IEC 14496-3 PDFs
under `docs/audio/aac/`.

## License

MIT — see [LICENSE](./LICENSE).

# oxideav-aac

A pure-Rust **AAC** (Advanced Audio Coding) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status (round 121)

**Phase 1 of the post-r111 orphan-rebuild lineage — syntactic
skeleton.** This release lands the bitstream framing the decoder will
sit on top of in later rounds:

- **ADTS fixed + variable header parser** (ISO/IEC 13818-7
  §1.A.2.2.1–.2): syncword, ID, layer, `protection_absent`, profile,
  `sampling_frequency_index`, `channel_configuration`,
  `aac_frame_length`, `adts_buffer_fullness`, and
  `number_of_raw_data_blocks_in_frame` (decoded as `N`, not the wire
  `N − 1`). Sample-rate index is resolved against ISO/IEC 14496-3
  Table 1.18; reserved indices (13, 14, 15) are rejected. CRC bytes
  are confirmed present when `protection_absent == 0` but the CRC
  value itself is **not** validated yet.
- **`raw_data_block()` syntactic walker** (ISO/IEC 14496-3
  §4.4.2.1): iterates `id_syn_ele` and recognises the eight element
  types (SCE, CPE, CCE, LFE, DSE, PCE, FIL, END). Per-element body
  handling in Phase 1:
  - SCE / CPE / CCE / LFE: emit `ChannelElement { kind, tag }`
    after consuming the 3-bit id and 4-bit `element_instance_tag`.
    The channel-element body itself is not parsed yet.
  - FIL (§4.4.2.7): full count + `esc_count` parse; payload bytes
    are skipped.
  - DSE (§4.4.2.5): full header parse including
    `data_byte_align_flag`; payload bytes are skipped.
  - PCE (§4.4.1.1): deferred — the walker returns
    `Error::UnsupportedElementSkip(5)` to signal that Phase 1 cannot
    walk past a PCE.
  - END: byte-aligns the bit-reader and stops cleanly; further calls
    return `None`.

## What Phase 1 deliberately omits

- All decode / encode bodies. The `register()` entry point installs
  no `Decoder` or `Encoder` into the runtime context yet, and the
  factory functions return `Error::NotImplemented`.
- `AudioSpecificConfig` (ISO/IEC 14496-3 §1.6.2.1).
- LATM/LOAS transport (ISO/IEC 14496-3 §1.7.3).
- ADTS CRC-16 validation.
- The PCE body parser.

## Provenance

No external implementation (FFmpeg / `libavcodec`, FDK-AAC, FAAD,
Nero AAC, libfaac, …) was consulted at any stage. Every numeric
constant, bit-layout decision, and clause reference in this crate
is sourced from the staged ISO/IEC 13818-7 and ISO/IEC 14496-3 PDFs
under `docs/audio/aac/`.

## License

MIT — see [LICENSE](./LICENSE).

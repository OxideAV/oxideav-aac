# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.6](https://github.com/OxideAV/oxideav-aac/compare/v0.0.5...v0.0.6) - 2026-04-19

### Other

- enable PNS + intensity stereo, add pulse_data emission
- drop Cargo.lock — this crate is a library
- bump oxideav-core / oxideav-codec dep examples to "0.1"
- bump to oxideav-core 0.1.1 + codec 0.1.1
- migrate register() to CodecInfo builder
- bump oxideav-core + oxideav-codec deps to "0.1"

## [0.0.5](https://github.com/OxideAV/oxideav-aac/compare/v0.0.4...v0.0.5) - 2026-04-19

### Other

- claim WAVEFORMATEX tags via oxideav-codec CodecTag registry
- bump oxideav-core to 0.0.5
- migrate to oxideav_core::bits shared BitReader / BitWriter
- add short-window TNS analysis + bitstream emission
- fix CI clippy + rustfmt regressions

## [0.0.4](https://github.com/OxideAV/oxideav-aac/compare/v0.0.3...v0.0.4) - 2026-04-18

### Other

- percussive round-trip test + AacEncoder::new ctor + doc refresh
- wire short-block state machine into emit_block
- add per-channel short-block state + set_enable_short_blocks
- add encode_raw_data_block_seq dispatcher for per-channel seq
- thread WindowSequence through ics_info emission
- refresh decode/encode support tables
- update short-block status — emission complete, state machine pending
- EightShort emission helpers + decoder round-trip test
- add IcsShort + analyse_and_quantise_short
- note short-block encoder building blocks in top-level docs
- add mdct_short_eightshort encoder helper + round-trip test
- expose build_long_window_full + shape-resolvers
- add transient detector (helper — not yet wired)
- document multi-channel encoder + refined encoder gap list
- multi-channel encode (channel_configuration 1..=7)
- add IS-per-band decision path (detector gated off)
- refresh top-level docs for decoder + encoder state
- land PNS + IS emission plumbing (detection gated off)
- multi-channel + LFE (channel configs 1..=7)
- parse PCE (program_config_element) per §4.5.2.1
- reject pulse_data_present=1 on EIGHT_SHORT windows
- wire short-window TNS (§4.6.9)
- wire Intensity Stereo (§4.6.8.2.3)
- fix stale Cargo description + lib.rs doc to reflect encoder
- rewrite README to match real decoder + encoder coverage
- parse + apply pulse_data() (§4.6.5) on long windows
- add encoder TNS support (LPC analysis + filter application + bitstream emission)
- add TNS + PNS decode (§4.6.9, §4.6.13)

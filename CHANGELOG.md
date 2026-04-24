# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.7](https://github.com/OxideAV/oxideav-aac/compare/v0.0.6...v0.0.7) - 2026-04-24

### Other

- clippy round-9 — zero warnings crate-wide
- aac SBR: targeted lint cleanup (hf_adjust + decode + encode)
- aac PS: fixture hunt for enable_ipdopd + Fil-count underflow fix
- aac PS: apply IPD/OPD phase correction to mixing matrix (§8.6.4.6.3.2)
- aac PS: implement hybrid sub-QMF filterbank (§8.6.4.3)
- fix S16 output scale — HE-AAC decode PSNR 1dB → 48dB
- promote HE-AACv2 PS to spec-accurate QMF-domain upmix
- HE-AAC interop test against afconvert / libfdk-aac
- aac SBR encoder: clamp cumulative envelope scalefactor at 127
- document new HE-AACv1 mono encoder in lib.rs header
- aac SBR encoder: envelope estimation uses 1.5 dB quantisation directly
- aac SBR encoder: FIL element hook + HE-AACv1 mono wrapper
- aac SBR encoder: downsampler + 64-band analysis QMF + payload scaffold
- minimal HE-AACv2 Parametric Stereo upmix
- SBR noise + sinusoid synthesis + limiter-band pass
- SBR HF LPC — covariance method alpha0 / alpha1
- wire SBR CPE through decoder, coupled-mode envelope apply
- SBR channel_pair_element parser + balance Huffman tables
- clean up SBR warnings
- wire SBR into AAC-LC decoder + E2E smoke tests
- SBR bitstream parsing + QMF banks + freq tables

### Added
- IPD/OPD phase-correction in the HE-AACv2 PS mixing matrix
  (§8.6.4.6.3.2). Previously the baseline decoder decoded the IPD/OPD
  Huffman indices but threw them away (§8.A.4 "Baseline PS" profile).
  Now the mixing coefficients `h11, h12, h21, h22` are complex-valued
  vectors of shape `h_ij(b) = h_ij_real(b) · exp(j·phi_ij(b))`, with
  `phi1 = phi_opd`, `phi2 = phi_opd − phi_ipd`, and each phi smoothed
  over `(e-1, e, e+1)` via the spec's
  `angle(0.25·e^{jp-1} + 0.5·e^{jp} + 0.25·e^{jp+1})`. Sub-subband
  indices 0 and 1 (Table 8.48 "*") take the complex conjugate of
  `h_ij`, matching the spec's second equation block. Interpolation
  across envelope borders runs componentwise on the complex plane and
  is carried over between frames via `prev_h_end` (now complex).
  `PsFrame` gains `has_ipdopd`, `ipd`, `opd` fields; `PsState` gains
  `prev_ipd_idx / prev_opd_idx / prev_ipd_last / prev_opd_last` for
  modulo-8 differential decoding and cross-frame smoothing.
- `tests/sbr_he_aac_v2_ps.rs::decode_he_aac_v2_ps_preserves_inter_channel_phase`
  — regression that drives a phase-coherent coherent-stereo fixture
  through our decoder and ffmpeg's, then compares the inter-channel
  phase magnitude.
- Hybrid sub-QMF analysis/synthesis filterbank for the HE-AACv2 PS
  upmix (§8.6.4.3). QMF band 0 is split into 6 sub-subbands via a 13-tap
  Type A (8-way complex) filter; bands 1 and 2 are split into 2 each via
  Type B (2-way cosine) filters. Each of the resulting 10 low-band
  sub-subbands is mapped to its own parameter index per Table 8.48,
  giving finer stereo resolution below ~500 Hz where it matters most.
  `hybrid_analysis_slot` / `hybrid_synthesis_slot` run inside
  `apply_ps_qmf`; bands 3..63 continue to mix at QMF granularity.
  Net effect: HE-AACv2 PSNR vs ffmpeg on afconvert-encoded stereo
  fixtures improves from ~19 dB to ~24 dB; amplitude overshoot drops
  from ~3.2× to ~1.2× of reference.
- `tests/sbr_he_aac_psnr.rs` — end-to-end PSNR regression test that
  encodes a sine via afconvert (or ffmpeg+libfdk_aac), decodes through
  the crate and ffmpeg, and asserts steady-state PSNR ≥ 40 dB. Skips
  gracefully without an HE-AAC encoder.
- `examples/sbr_probe.rs` — CLI helper that prints per-frame PCM peaks
  and a PSNR figure against a reference PCM. Useful for bisecting SBR
  amplitude / scaling regressions.

### Fixed
- SBR PCM clipping / 6.8x reference amplitude overshoot on HE-AAC mono
  decode. Root cause was the S16 output path multiplying IMDCT results by
  `32767` even though the IMDCT pair operates at native int16 range; the
  MDCT+IMDCT transform pair in this crate sits at a gain of 2 by design
  (unscaled forward + `2/input_n` inverse), so the correct output scale
  is `* 0.5` clamped to i16 range, not `* 32767` clamped to [-1, 1].
  HE-AACv1 stereo PSNR vs ffmpeg jumps from ~1 dB to ~48 dB.
- Non-exhaustive-struct compile break in `tests/decode_fixture.rs`
  (`CodecParameters` is `#[non_exhaustive]`).


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

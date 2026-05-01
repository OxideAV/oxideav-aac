# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Notes (round 21)
- `tests/sbr_he_aac_ffmpeg_amplitude_r18.rs` updated with round-21 audit
  data:
  ```
  ours-encode -> ours-decode      : peak 10 256 / RMS 6 582  (within 5% of input)
  ours-encode -> ffmpeg-decode    : peak 32 767 / RMS 17 181 (saturated)
  fdkaac-encode -> ffmpeg-decode  : peak  9 822 / RMS 6 920  (within 1% of input)
  fdkaac-encode -> ours-decode    : peak 13 009 / RMS 7 061  (within 30% of input)
  ```
  Direct probe of the bitstream-level envelope value reveals **both
  fdkaac and our encoder transmit `bs_data_env[0] = 0`** for tonal
  content with no high-band energy (E_orig = 64, the spec minimum).
  The bitstream-level envelope value is therefore not the source of the
  saturation — both encoders emit the same value yet ffmpeg decodes the
  fdkaac stream cleanly while saturating ours.
- Round 21 audited the §4.6.18 SBR pipeline end to end: synthesis QMF
  gain (1/64 internal scale per §4.6.18.4.2 / Fig. 4.43) is correct;
  envelope adjuster `gain = sqrt(E_orig / (E_curr * (1 + Q_orig)))` is
  spec-correct; HF generation patches and limiter cap (`g_ref *
  limiter_gain_cap`) match Table 4.176. The remaining saturation must
  arise from a divergence between our encoder's SBR header / freq-table
  configuration and ffmpeg's decoder expectations for one of those —
  but the immediate audit on `bs_start_freq=5`, `bs_stop_freq=9`,
  `bs_freq_scale=2`, `bs_alter_scale=true`, `bs_xover_band=0`,
  `bs_noise_bands=2` did not isolate it. Round 22+ target.
- All 170 tests + 1 ignored remain unchanged from round 20.

### Added (round 19)
- `tests/lc_rms_interop_r19.rs` — pins AAC-LC ffmpeg-interop within ±10%
  of unity on the RMS metric across all four directions
  (ours/ffmpeg-encode × ours/ffmpeg-decode). Measured RMS ratios on a
  440 Hz / 0.3-amp / 44.1 kHz mono sine: **0.97 / 0.96 / 0.99 / 1.00**.
- `examples/spectrum_compare.rs` — diagnostic probe that parses one
  SCE element from each of (ffmpeg-encoded, ours-encoded) ADTS streams,
  reports peak / energy / nonzero-bin count / IMDCT peak / windowed-OLA
  peak. Surfaced that ffmpeg encodes 678 nonzero bins (vs our 71), 5 of
  them PNS-coded (`codebook 13`), at the same total energy — explaining
  why the *peak* metric diverges by 1.79x while the *RMS* tracks within
  1%.
- `examples/imdct_unit_test.rs` — unit-bin probe asserting our IMDCT
  output equals exactly `2x` the spec formula (`2/N`, N = window length
  per §4.6.11.3.1), confirming the documented "double-scale" convention
  pairs with the decoder's `* 0.5` S16-output stage to match the spec
  exactly.

### Changed (round 19)
- `encoder::MDCT_FORWARD_SCALE` from `32768` → `65536`. Spec derivation:
  with unscaled forward MDCT and the spec inverse `2/N`, sine-windowed
  TDAC OLA reconstructs 0.5x the input (verified empirically). To match
  §4.5.2.3.6 ("the integer part of the output of the IMDCT can be used
  directly as a 16-bit PCM audio output"), a spec-compliant encoder
  must scale its emitted spectrum by 2x, so the forward scale is
  `2 * 32768 = 65 536`. Lifts our LC self-roundtrip RMS from 0.55x to
  0.97x and brings ffmpeg → ours / ours → ffmpeg parity within ±5%.
- `examples/probe_lc_amp.rs` — augmented with reverse-direction
  measurement (ffmpeg-encode → ours-decode) and RMS reporting on every
  path; emits the round-19 verdict ("peak ratio is not a meaningful
  interop metric for tonal+noise content; RMS is") so future rounds
  don't re-chase the phantom 3.33x peak gap.
- `encoder.rs::MDCT_FORWARD_SCALE` doc comment rewritten with the
  clean-room spec derivation (§4.6.11.3.1 + §4.5.2.3.6).

### Notes (round 19)
- `tests/sbr_he_aac_ffmpeg_amplitude_r18.rs` (HE-AACv1 SBR amplitude
  saturation at peak 32_768 regardless of envelope) remains ignored.
  The r19 LC-core fix does **not** affect this — the SBR-path interop
  gap lives elsewhere (likely SBR envelope or HF-generation gain).
  Round 20+ target.

### Added (round 14)
- HE-AACv2 PS encoder picks **time-direction** vs frequency-direction
  differential coding per envelope (`iid_dt[e]`, `icc_dt[e]`) by comparing
  Huffman-coded bit cost (§8.6.4.6.2). On a stationary input, frame N's
  per-band IID/ICC indices match frame N-1, so every band's DT delta is 0
  → 1-bit-per-band codeword (Table 8.B.18 / 8.B.19), which beats DF for
  any non-trivial parameter set. `SbrEncoder` now tracks last-envelope
  `prev_iid_idx` / `prev_icc_idx` per stream to seed the DT baseline.
- **Multi-envelope PS** (`num_env ∈ {1, 2, 4}`, Table 8.29): new
  `PsParamsFrame` / `analyse_ps_params_10_multi_env` analyser splits the
  AAC frame into equal-size sub-blocks, quantises each independently, and
  emits `num_env_idx` per §8.6.4.6.2. `detect_num_env` picks the count
  from the per-channel QMF energy profile — 6 dB max-to-mean ratio →
  `num_env = 4`, 3 dB → `num_env = 2`, else 1. `HeAacV2Encoder` is wired
  to the multi-envelope path; transients now ride a per-quarter-frame
  envelope grid instead of being smeared across one whole-frame envelope.
- `huff_bits_iid_df0` / `huff_bits_iid_dt0` / `huff_bits_icc_df` /
  `huff_bits_icc_dt` — public Huffman cost functions on Tables 8.B.18 /
  8.B.19, used by the encoder's df-vs-dt selector.
- `tests/he_aac_v2_multi_env.rs` — interop tests: transient stereo (silent
  → tone) decodes through ffmpeg with the silent→loud energy step ≥ 6 dB
  preserved; stationary L=1 kHz / R=2 kHz panned input keeps ≥ 20 % of
  samples differing |L−R| > 32 after the DT-encoded warmup.
- `tests/he_aac_v2_psnr_afconvert.rs` — round-14 plan C: encode the same
  mixed-content stereo through `HeAacV2Encoder` and Apple `afconvert
  -d aacp` (HE-AACv2), decode both via ffmpeg, report per-channel PSNR.
  Current gap: ours ≈ 7-9 dB vs afconvert's 15-19 dB on the chosen test
  signal, so still room to tighten.

### Fixed
- HE-AACv1 stereo CPE bitstream ordering (Table 4.66, `bs_coupling = 0`):
  the **independent**-coupling branch transmits `envelope(L)`,
  `envelope(R)`, then `noise(L)`, `noise(R)` — both envelopes first,
  then both noise floors. Round 10 wrote and parsed the four blocks
  interleaved per-channel (`env(L), noise(L), env(R), noise(R)`),
  which is the **coupled** branch's layout. Round 11 fixes both the
  `write_channel_pair_element_independent` writer and the matching
  branch in `parse_channel_pair_element`. The bug caused ffmpeg to
  read our R-channel envelope bits as R-channel noise: ffmpeg-decoded
  R-channel SNR jumps from **5.9 dB → 22.8 dB** for a 2 kHz tone,
  matching the mono HE-AAC baseline at the same content.
  `sbr_he_aac_stereo_ffmpeg.rs` thresholds raised from 5 dB to 30 dB
  (L) / 20 dB (R) to lock in the fix.

### Added
- `sbr::encode::SbrStereoEncoder` and `he_aac_encoder::HeAacStereoEncoder`
  — HE-AACv1 stereo encoder. Emits a CPE with an SBR FIL payload in
  **independent coupling** mode (§4.6.18.3.5, `bs_coupling = 0`):
  shared SBR header, per-channel FIXFIX `bs_num_env=1` grid, freq-delta
  Huffman-coded envelope (1.5 dB) + noise (3.0 dB). One core AAC-LC
  CPE + one CPE-shaped FIL element per frame.
  `write_channel_pair_element_independent` is the new bitstream writer
  that mirrors `parse_channel_pair_element` in independent mode.
- `tests/sbr_encode_stereo_roundtrip.rs` — self round-trip: encode 1
  kHz / 2 kHz stereo at 48 kHz, decode through the in-crate decoder,
  confirm both channels recover their source tone with a Goertzel
  ratio above the per-channel RMS floor.
- `tests/sbr_he_aac_stereo_ffmpeg.rs` — ffmpeg interop: encode 1 kHz
  / 2 kHz stereo, hand the ADTS to ffmpeg's native AAC decoder, and
  assert per-channel SNR ≥ 5 dB at 48 kHz / stereo output. Skips
  cleanly when ffmpeg isn't available.

## [0.0.8](https://github.com/OxideAV/oxideav-aac/compare/v0.0.7...v0.0.8) - 2026-04-25

### Other

- drop oxideav-codec/oxideav-container shims, import from oxideav-core

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

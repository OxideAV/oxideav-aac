# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); the crate adheres
to [SemVer](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- LATM `StreamMuxConfig()` decode (`latm` module) — ISO/IEC 14496-3
  §1.7.3.1 Table 1.42 plus `LatmGetValue()` (Table 1.43).
  `StreamMuxConfig::parse` decodes the whole multiplex configuration:
  the `audioMuxVersion` / `audioMuxVersionA` version flags (with the
  `audioMuxVersion == 1` `taraBufferFullness` and per-ASC
  length-prefix + `fillBits` extensions), `allStreamsSameTimeFraming`,
  `numSubFrames` / `numProgram` / per-program `numLayer`, the
  per-`streamID[prog][lay]` `LayerConfig` table (each carrying its
  inline `AudioSpecificConfig()` or the resolved `useSameConfig`
  inheritance, the `frameLengthType`, and the type-0
  `latmBufferFullness` / CELP-core `coreFrameOffset` or type-1
  `frameLength`), the `otherDataPresent` / `otherDataLenBits` escape
  loop, and the `crcCheckPresent` / `crcCheckSum`. The `crcCheckSum` is
  recomputed against the configuration prefix via the §1.8.4.5 `CRC8`
  generator (`crc::stream_mux_config_crc`) and a mismatch surfaces
  `Error::LatmCrcMismatch`. The reserved `audioMuxVersionA == 1` branch
  and the CELP/HVXC `frameLengthType` values (`2`..`7`) are rejected
  with dedicated errors.
- SBR element framing (`sbr_element` module) — ISO/IEC 14496-3
  §4.4.2.8 Tables 4.65 / 4.66 / 4.74. `SbrElement::parse_single` and
  `SbrElement::parse_pair` decode a whole SBR data element in spec
  order: the optional `bs_data_extra` reserved field, the per-channel
  grid / dtdf / invf / envelope / noise blocks (the coupled-pair
  shared-grid layout vs. the independent-grid layout, with the second
  coupled channel decoded in balance mode), `sbr_sinusoidal_coding()`'s
  `NHigh` add-harmonic flags, and the `bs_extended_data` block (id +
  raw body bytes captured for a later PS pass; `EXTENSION_ID_PS`
  defined). The single-envelope FIXFIX `bs_amp_res` override is applied
  before envelope decode so start-value widths and codebook selection
  match the spec's in-order `bs_amp_res` mutation.
- SBR envelope / noise DPCM reconstruction (`sbr_reconstruct` module) —
  ISO/IEC 14496-3 §4.6.18.3.5. `EnvelopeScalefactors::reconstruct` and
  `NoiseScalefactors::reconstruct` invert the delta coding to recover
  the quantized scalefactors `E_Q(k,l)` / `Q(k,l)` from the raw
  `bs_data_*`: frequency-direction deltas accumulate across bands from
  the absolute start value; time-direction deltas add to the reference
  envelope (the previous envelope in-frame, or the last envelope of the
  prior frame for `l == 0`), including the `i(k)` high↔low band remap
  when the reference resolution differs (`r(l) ≠ g(l)`). The coupled
  second channel's `δ = 0.5` is applied as an integer ×2 on the
  transmitted (even) values. Cross-frame state is threaded via an
  optional previous-frame `prev` argument.
- `sbr_envelope()` / `sbr_noise()` raw decode (`sbr_envelope` module) —
  ISO/IEC 14496-3 §4.4.2.8 Tables 4.72–4.73. `SbrEnvelopeData::parse`
  and `SbrNoiseData::parse` read the per-envelope / per-noise-floor
  delta arrays driven by the grid, the `sbr_dtdf()` delta directions,
  and the derived band counts: a fixed-width absolute start value at
  band 0 of a frequency-coded envelope (5/6/7-bit per the coupling /
  channel / `bs_amp_res` context; noise always 5-bit) followed by
  frequency-direction Huffman deltas, or all-band time-direction
  Huffman deltas. Per-envelope band counts come from `NHigh` / `NLow`,
  noise floors from `NQ`. This produces the raw `bs_data_*` values;
  the §4.6.18.3.5 DPCM accumulation and dequantization are downstream.
- SBR time-frequency grid parsers (`sbr_grid` module) — ISO/IEC
  14496-3 §4.4.2.8 Tables 4.69–4.71. `SbrGrid::parse` decodes all four
  `bs_frame_class` cases (FIXFIX / FIXVAR / VARFIX / VARVAR): the
  envelope count (`2^raw` for FIXFIX, `bs_num_rel_*`-derived otherwise,
  bounded by the §4.6.18.3.6 `SBR_MAX_NUM_ENV`), the variable /
  relative borders, the `ptr_bits = ceil(log2(num_env + 1))`
  `bs_pointer`, the per-envelope frequency-resolution flags (reversed
  for FIXVAR), the single-envelope FIXFIX `bs_amp_res` override, and
  `bs_num_noise = (num_env > 1) ? 2 : 1`. `SbrDtdf::parse` reads the
  per-envelope / per-noise delta-direction flags (Table 4.70), and
  `SbrInvf::parse` the 2-bit inverse-filtering mode per noise band
  (Table 4.71). New `Error::SbrGridInvalid` for a truncated or
  out-of-range grid.
- `sbr_header()` parser (`sbr_header` module) — ISO/IEC 14496-3
  §4.4.2.8 Table 4.63. `SbrHeader::parse` reads the fixed-width header
  (`bs_amp_res` / `bs_start_freq` / `bs_stop_freq` / `bs_xover_band` /
  `bs_reserved` / the two `bs_header_extra_*` flags) and the optional
  extra blocks, filling in the Table 4.63 Note 3 default values
  (Tables 4.105–4.111: `bs_freq_scale=2`, `bs_alter_scale=1`,
  `bs_noise_bands=2`, `bs_limiter_bands=2`, `bs_limiter_gains=2`,
  `bs_interpol_freq=1`, `bs_smoothing_mode=1`) when an extra-header
  flag is clear. `band_geometry_changed()` reports whether the
  §4.6.18.3.3 band-reset parameters differ between two headers, and
  `derive_bands()` chains the header into the existing
  `sbr_freq_bands` k0/k2/master/HiLoTables pipeline.
- SBR Huffman codebooks + `sbr_huff_dec()` (`sbr_huffman` module) —
  ISO/IEC 14496-3 Annex 4.A.6.1. All ten normative SBR envelope /
  noise codebooks (Tables 4.A.79–4.A.88) are transcribed directly from
  the spec codeword grids: `t/f_huffman_env_1_5dB` (LAV 60),
  `t/f_huffman_env_bal_1_5dB` (LAV 24), `t/f_huffman_env_3_0dB` (LAV
  31), `t/f_huffman_env_bal_3_0dB` (LAV 12), `t_huffman_noise_3_0dB`
  (LAV 31), `t_huffman_noise_bal_3_0dB` (LAV 12). The
  frequency-direction noise codebooks alias the 3.0 dB envelope freq
  tables per Table 4.A.78 Note 2. `sbr_huff_dec()` reads MSB-first one
  bit at a time and returns `index - LAV` (the signed DPCM delta);
  `env_tables()` / `noise_tables()` pick the `(t_huff, f_huff)` pair
  from an `SbrHuffContext` (`bs_coupling` / channel / `bs_amp_res`) per
  the §4.6.18.3 `sbr_envelope()` / `sbr_noise()` selection. Every table
  is validated complete + prefix-free, and a codeword round-trip test
  exercises all entries. New `Error::SbrHuffInvalid` for an unmatched
  or truncated codeword.

- Runtime `oxideav_core::Decoder` wiring (`codec_decoder` module) — the
  no-op `register()` now installs a real AAC decoder under id `"aac"`.
  `AacDecoder` adapts the persistent `decode::StreamDecoder` into the
  framework's packet-in / frame-out trait: `send_packet` decodes every
  ADTS frame in a packet (ID3v2-skip + `aac_frame_length` framing, one
  or more frames per packet) against the carried-across-packets element
  state, `receive_frame` returns one interleaved-S16 `AudioFrame` (1024
  samples/channel) per decoded frame, `flush` drives to `Eof`, and
  `reset` drops the whole `StreamDecoder` (and with it the §4.6.11
  overlap / §4.6.7 LTP / §4.6.6 predictor memory) for a clean post-seek
  restart. `register_codecs` claims the MP4 object-type `0x40`,
  WAVEFORMATEX `0x00FF` / `0x1601`, the `mp4a` / `aac ` FourCCs, and the
  Matroska `A_AAC` CodecID, with an ADTS-syncword probe to win shared
  tags. A `trait_decode_matches_stream_decoder_pcm` test pins the trait
  output as byte-identical to the underlying `StreamDecoder`. No encoder
  is wired (the crate has the bit-exact wire writers but no rate-control
  back-end). New `tests/codec_decoder_pcm.rs` re-runs the
  `expected.wav` corpus comparison through the framework trait surface
  (packet-in / `AudioFrame`-out, the exact path a container drives):
  the two PNS-free fixtures stay **≥99.9% byte-exact, max error 1 LSB**,
  and the PNS fixtures stay below the §8 PCM-RMS tolerance — proving the
  packetisation / `AudioFrame` byte-layout / across-packet state
  threading preserve the underlying decode. A
  `he_aac_base_layer_decodes_through_trait` test pins that an HE-AAC v1
  ADTS stream (AAC-LC base layer + an `EXT_SBR_DATA` extension this
  crate has no back-end for) still decodes its base layer to PCM through
  the trait rather than erroring — the §4.4.2.1 walk consumes the FIL
  element and the SBR high band is simply absent.

- Stream-level ADTS decode driver (`decode` module) — the §4.4.2.1
  `raw_data_block()` walk above the per-element driver. `StreamDecoder`
  holds one `ElementDecoder` per `(syntactic-element-id,
  element_instance_tag)` slot so each element's §4.6.11 overlap, §4.6.7
  LTP history, and §4.6.6 predictor state persist across frames;
  `decode_frame` dispatches each `id_syn_ele` (SCE / LFE / CPE, including
  the §4.4.2.3 `common_window` / `ms_mask_present` header for a CPE) onto
  the matching slot and renders the frame's channels to element-order
  interleaved 16-bit PCM via the new `pcm` module, and `decode_all`
  walks a whole raw-ADTS buffer (ID3v2-skip + `aac_frame_length`
  framing) to a `Vec<DecodedFrame>`. With this driver the crate decodes
  a raw ADTS stream end to end to comparable s16 PCM. CCE / multi-RDB /
  SBR remain out of scope (the element driver's limits). New
  `tests/pcm_byte_exact.rs` compares the decoded PCM against each
  fixture's `expected.wav` in two regimes: the two PNS-free fixtures
  (`aac-lc-mono-8000-16kbps-adts`, `aac-lc-intensity-stereo`) match the
  reference s16 output at **99.9% byte-exact, max error 1 LSB** (the
  residual being the f64-direct-sum vs float32-fast-transform IMDCT
  difference); the six PNS-bearing fixtures are compared in the
  §8-prescribed PCM RMS domain, where the error-to-signal RMS ratio
  stays **below 0.1%** (the spec-undefined §4.6.13.3 noise phase carries
  negligible energy in these tonal fixtures, so full byte-exactness is
  precluded but the decode is energetically near-identical). A third
  test pins that a `StreamDecoder` decodes a PNS-heavy stream
  reproducibly across runs (the determinism the open §4.6.13.3 generator
  still must provide).
- §4.6.11 integer-PCM rendering (`pcm` module) — the final output stage
  that turns the §4.6.11 filterbank's `f64` time signal (already on the
  `±32768` full-scale amplitude axis from the `2/N` IMDCT normalisation +
  §4.6.2.3.3 scalefactor gain) into 16-bit signed PCM. `nint` is the
  §1.3 `NINT()` nearest-integer operator (half-integers rounded away from
  zero, per the spec arithmetic-operator definitions); `to_s16` applies
  it then saturates to `-32768..=32767`; `channel_to_s16` maps a channel
  and `interleave_s16` packs a frame's per-channel buffers into the
  element-order interleaved layout. The conversion is the *only*
  output-rendering step (no resampler, no dither, no channel remap), so
  it is fully spec-determined. New `Error::PcmInvalid` for a channel
  length disagreement on interleave. Pinned by 9 unit tests
  (away-from-zero half rounding, non-finite-to-zero, saturation extremes,
  interleave order / mismatch). This unblocks `expected.wav` byte-exact
  comparison: the deterministic (non-PNS) decode path now matches the
  reference s16 output to within ≤1 LSB (the residual is the difference
  between this crate's f64 direct-sum IMDCT and a float32 fast transform;
  PNS bands diverge by the §4.6.13.3 spec-undefined RNG phase, as
  expected).
- §1.8.4.5 error-protection CRC generator (`crc` module) — the full
  family of MPEG-4 Audio CRC generation polynomials (`CrcPoly::Crc4`
  through `Crc32`, including the `Crc8` used by the LATM
  `StreamMuxConfig()` `crcCheckSum` and the 16-bit `x¹⁶+x¹⁵+x²+1`),
  with `width()` / `generator()` accessors derived directly from the
  §1.8.4.5 polynomial listing. `crc_bits` / `crc_bytes` implement the
  zero-init, MSB-first, no-input-reflection shift register computing
  the §1.8.4.5 `M(x)·xᵏ = Q(x)·G(x) + R(x)` remainder and apply the
  normative output-bit inversion ("the CRC bits are written in a
  reversed manner, i.e. each bit is inverted"). `stream_mux_config_crc`
  is the LATM `crcCheckSum` (§1.7.3.1 / Table 1.42) helper. Validated
  against an independent GF(2) long-division reference across six
  widths and the `M(x)·xᵏ + R(x)` codeword-divisibility property. The
  §1.8.4.6 SRCPC FEC stage and the ADTS `adts_error_check()`
  region-selection CRC (ISO/IEC 13818-7 → ISO/IEC 11172-3 §2.4.3.1)
  are out of scope.
- §4.6.18.3.2 SBR frequency band tables (`sbr_freq_bands` module) — the
  static, header-only half of the Spectral Band Replication band setup,
  computed from the closed-form ISO/IEC 14496-3 algorithm with no QMF
  back-end. `k0` / `k2` derive the §4.6.18.3.2.1 low / high QMF subband
  boundaries (the per-`FsSBR` `offset` table, the
  `startMin`/`stopMin = NINT(c·128/FsSBR)` thresholds, the
  `stopDkSort` accumulation for `bs_stop_freq < 14`, and the
  `bs_stop_freq == 14/15` `min(64, 2·k0)` / `min(64, 3·k0)`
  shortcuts). `master_table` builds `fMaster` for both the Figure 4.39
  linear path (`bs_freq_scale == 0`) and the Figure 4.40 warped path
  (`bs_freq_scale > 0`, single-/two-region split at `k2/k0 > 2.2449`
  with the `min(vDk1) < max(vDk0)` smoothing). `HiLoTables::derive`
  produces the §4.6.18.3.2.2 `fTableHigh` / `fTableLow` / `fTableNoise`
  plus the `M` and `k_x` outputs. The §4.6.18.3.6 requirements
  (`k2 > k0`, `numBands > 0`, `vDk > 0`, `bs_xover_band < NMaster`) are
  enforced via the new `Error::SbrFreqBandInvalid`. The §4.6.18.3.2.3
  limiter band table is deferred (needs the §4.6.18.6 patch borders).
- §4.6.6 MPEG-2 frequency-domain prediction (`predictor` module) — the
  backward-adaptive intra-channel predictor of the AAC Main object type
  (AOT 1). A `PredictorBank` of second-order lattice `Predictor`s (one
  per MDCT line up to the §4.6.6.2 `PRED_SFB_MAX` limit) reconstructs
  `x_rec = x_est + y_rec` on the signalled bands. Implements the
  §4.6.6.3.2.1 lattice `predict()` + LMS adaptation (`α = 0.90625`,
  `a = b = 0.953125`), the §4.6.6.3.2.3 `flt_round_inf()` 16-bit-float
  rounding applied to every stored state variable and the predicted
  value, and the §4.6.6.3.3 reset (the 30 Table 4.97 cyclic groups plus
  the short-block reset-all). Wired into `element_decode`: each channel
  now carries a lazily-built per-rate predictor bank, run every long
  frame *before* §4.6.7 LTP / §4.6.9 TNS and reset on a short block, so
  the backward-adaptive state persists across frames. Prediction and LTP
  are mutually exclusive by object type. New `Error::PredictorInvalid`
  for an inconsistent offset table / spectrum length / reset-group
  number.
- §4.6.7.4.1 LTP-with-TNS integration — LTP is now wired into the
  `element_decode` driver in the Figure 4.30 block order.
  `finish_channel` runs long-term synthesis *before* the §4.6.9 TNS
  synthesis filter, with the LTP-predicted spectrum `X_est` first passed
  through the matching all-zero **TNS analysis filter** so the
  `X_rec = X_est + Y_rec` add is like-for-like; the subsequent TNS
  synthesis pass shapes the residual while undoing the analysis on the
  LTP contribution. `ElementDecoder` now holds a per-channel
  `ltp::LtpState`, advanced every frame (whether or not LTP fired) via
  the filterbank's new `aliased_tail()` / `prev_shape()` accessors so
  the §4.6.7.3 reconstruction history stays continuous. New
  `tns_coef::tns_ma_filter` (the all-zero inverse of the §4.6.9.3
  `tns_ar_filter`) and `tns_frame::tns_analysis_frame` (its per-window
  frame driver); `ltp::LtpState::apply_long_with_analysis` inserts the
  analysis filter between `MDCT(x_est)` and the per-sfb add.
  Short-window LTP and the ER AAC LD `M = N/2` lag offset remain out of
  scope. Pinned by analysis∘synthesis identity tests and three
  element-level LTP tests (zero-history no-op, second-frame divergence,
  LTP+TNS finiteness).

- §4.6.7 Long-Term Prediction (LTP) long-window synthesis (`ltp`): the
  Table 4.98 coefficient codebook, the `predict()` single-tap
  time-domain predictor over the §4.6.7.3 `x_rec` reconstruction
  history (`x_est(i) = ltp_coef·x_rec(i − ltp_lag)`), the windowed
  analysis `MDCT(x_est)`, and the per-sfb `X_rec = X_est + Y_rec`
  spectral combination on the bands flagged by `ltp_long_used`. The
  forward (analysis) MDCT (§4.6.15.3.3 / §4.6.11.3.1) is promoted to a
  reusable `filterbank` primitive. Short-window and ER AAC LD (`M =
  N/2`) LTP remain out of scope; the side info was already parsed.

## [0.1.5](https://github.com/OxideAV/oxideav-aac/compare/v0.1.4...v0.1.5) - 2026-06-15

### Other

- §4.6 element-level decode driver — SCE/CPE to PCM (r311)
- §4.6.13 Perceptual Noise Substitution synthesis (r307)
- §4.6.8.2 intensity stereo synthesis (r300)
- §4.6.8.1 M/S stereo de-matrix (r293)
- §4.6.11 filterbank — IMDCT + sine/KBD windows + overlap-add
- finish the positive-provenance sweep (README + remaining test preambles)
- state provenance positively in module/test docs
- dequant + decoded_spectrum: §4.6.1.3/§4.6.2.3.3 dequantisation and the per-channel decoded-spectrum pipeline
- add Table 4.56 spectral_data() wire walker + §4.5.2.3.4 sect_sfb_offset
- add §4.6.9.3 tns_decode_frame per-frame TNS orchestration
- add §4.6.9.3 tns_ar_filter all-pole IIR pass

### Added

- phase 2 (r311): `element_decode` — the element-level decode driver,
  the §4.6 block-order glue that chains every per-tool primitive into
  PCM-domain samples for a `single_channel_element()` (SCE / LFE) and a
  `channel_pair_element()` (CPE). `ElementDecoder` is stateful: it holds
  one `Filterbank` per channel slot (the §4.6.11.3.3 overlap-add tail
  and §4.6.11.3.2 previous-block window shape persist across frames) and
  the §4.6.13.3 PNS generator state. `decode_sce` runs pulse §4.6.3.3 →
  dequant §4.6.1.3 → rescale §4.6.2.3.3 → `quant_to_spec()` §4.6.3.3 →
  PNS §4.6.13 → TNS §4.6.9 → filterbank §4.6.11. `decode_cpe` runs the
  pair chain with the joint-stereo / noise tools in normative block
  order — per-channel pulse/dequant/de-interleave, then M/S §4.6.8.1 →
  intensity §4.6.8.2 → PNS §4.6.13 on the pre-TNS pair (the tools sit
  between `quant_to_spec()` and TNS per §4.6.13.5), then per-channel TNS
  + filterbank. `ChannelInput` bundles a parsed body + `ics_info` +
  spectrum; `CpeJointStereo` carries the Table 4.4 `ms_mask_present` +
  `ms_used`. New `Error::ElementDecodeInvalid` covers a CPE with
  mismatched window geometry, an under-covered `ms_used` mask, or a
  scalefactor-record count that does not match the `sfb_cb`
  non-`ZERO_HCB` band count. Pinned by 9 unit tests (band-indexed track
  expansion, SCE finite/non-silent PCM, inter-frame overlap coupling,
  CPE M/S reconstruction, mask-off independence, window-mismatch
  rejection, single-channel noise synthesis) and an end-to-end
  integration driver (`tests/element_decode.rs`) that decodes all 12
  staged ADTS fixtures to PCM with finite output, non-silent frames, and
  witnessed overlap coupling. The Main predictor (§4.6.7), LTP
  (§4.6.6), and SSR gain-control (§4.6.12) remain unapplied;
  `expected.wav` PCM comparison is a followup.
- phase 2 (r307): `pns` — the §4.6.13 Perceptual Noise Substitution
  synthesis, the third and last channel-pair / noise tool. `apply_pns`
  fills every `NOISE_HCB` (13) band of a channel in place over a
  `PnsChannel` (the de-interleaved window-major spectrum, the per-band
  `sfb_cb`, and the accumulated `noise_nrg[g][sfb]`) by drawing a
  random vector and applying the 2009 measured-energy normalisation
  `scale = 2^(0.25·noise_nrg) / sqrt(Σ spec²)`, so each band's L2 norm
  is exactly `2^(0.25·noise_nrg)` — a deterministic, spec-determined
  invariant (only the per-coefficient phase is generator-dependent,
  which §4.6.13.3 leaves open). Public `noise_target_norm`, `is_noise`,
  and the default `gen_rand_vector` generator; `apply_pns` takes the
  generator as a closure. `apply_pns_pair` implements the §4.6.13.3
  shared-random-vector correlation rule for a channel pair (same vector
  for both channels when both are noise on a band and the band signals
  correlation; independent otherwise; no M/S de-matrix on noise bands,
  §4.6.13.5). Runs prior to TNS in the §4.6 block order. New
  `Error::PnsInvalid`. With PNS landed, all three channel-pair / noise
  tools (M/S, intensity, PNS) are synthesised.
- phase 2 (r300): `intensity_stereo` — the §4.6.8.2 intensity stereo
  (IS) synthesis, the second deterministic channel-pair tool.
  `apply_intensity_stereo` derives the right channel from the left in
  place over an `IntensityPairSpectra` (the pair's de-interleaved
  window-major spectra, the right `sfb_cb`, and its accumulated
  `is_pos[g][sfb]`), for every `(group, window, sfb)` whose right
  codebook is an intensity book, via the §4.6.8.2.3 scale
  `is_intensity · invert_intensity · 0.5^(0.25·is_pos)` — left channel
  untouched. Public helpers `is_intensity` (`+1`/`−1`/`0` for
  `INTENSITY_HCB` / `INTENSITY_HCB2` / other), `invert_intensity`
  (`1 − 2·ms_used` under a per-band M/S mask, the §4.6.8.2.3 phase
  reversal; `+1` otherwise), and `intensity_gain` (`0.5^(0.25·is_pos)`).
  Runs after M/S and before TNS in the §4.6 block order. Pinned by an
  encode→decode round-trip over a 20-band long frame plus unit tests
  for the sign, both invert branches, the gain ladder, in/out-of-phase
  copy, position scaling, mask-on/off phase distinction, short-window
  grouping, and the shape-validation rejections. New
  `Error::IntensityStereoInvalid`. PNS (§4.6.13) is the remaining
  channel-pair / noise tool.
- phase 2 (r293): `ms_stereo` — the §4.6.8.1 M/S (mid/side) stereo
  de-matrix, the first of the channel-pair / noise synthesis tools
  between the round-289 single-channel chain and byte-exact PCM.
  `apply_ms_stereo` runs the §4.6.8.1.3 inverse matrix
  (`l' = m + s`, `r' = m − s`) in place over a `ChannelPairSpectra`
  (both channels' de-interleaved, pre-TNS window-major spectra plus
  per-channel `sfb_cb`), per `(group, window, sfb)` selected by the
  decoded `MsMaskPresent` (`00` no-op / `01` per-band `ms_used` mask /
  `10` all-ones; `11` reserved → rejected). Suppressed on
  intensity-coded bands (right-channel `INTENSITY_HCB` /
  `INTENSITY_HCB2`) and noise-substituted bands (`NOISE_HCB` in either
  channel), keeping M/S mutually exclusive with intensity and PNS.
  Pinned by an encode→decode round-trip over a 20-band long frame plus
  unit tests for all three mask modes, the intensity / noise
  exclusions, short-window grouping, exact integer invertibility, and
  the shape-validation paths. New `Error::MsStereoInvalid` covers
  channel-pair / `ms_used` / `sfb_cb` shapes that disagree with the
  shared `common_window` `ics_info` geometry.
- phase 2 (r289): `filterbank` — the §4.6.11 filterbank and block
  switching, the time-domain reconstruction stage that consumes the
  round-284 window-major decoded spectrum and emits PCM-domain
  samples. `Filterbank::synthesize` runs the §4.6.11.3.1 IMDCT
  (`x[n] = (2/N)·Σ spec[k]·cos((2π/N)(n + n0)(k + 1/2))`,
  `n0 = (N/2 + 1)/2`, `N ∈ {2048, 256}`), applies the §4.6.11.3.2
  sine and Kaiser-Bessel-derived windows for all four
  `window_sequence` shapes (`ONLY_LONG` / `LONG_START` /
  `EIGHT_SHORT` / `LONG_STOP`) — with the left-half shape inherited
  from the previous block — and performs the §4.6.11.3.3 inter-frame
  overlap-add, carried across frames as stateful overlap. The KBD
  window is the normalized running sum of the Kaiser-Bessel kernel
  `W'(n, α)` (`α = 4` long, `α = 6` short) over a power-series `I0`.
  Pinned by streaming TDAC perfect-reconstruction tests (sine + KBD,
  long + eight-short), `W(n)^2 + W(n + N/2)^2 = 1` window-power
  identities, tabulated `I0` values, and a mono-ADTS-fixture
  integration driver that confirms finite PCM with the overlap tail
  coupling consecutive frames. New `Error::FilterbankInvalid` covers
  spectrum-length / `window_sequence` mismatches. The frame-length-960
  (`N = 1920 / 240`) transform family is out of scope, matching the
  crate's 1024-coefficient `swb_offset` layout.
- phase 2 (r284): `dequant` + `decoded_spectrum` — the first numeric
  reconstruction stages after the wire walk, ending one step short of
  the §4.6.11 filterbank. `dequant::inverse_quantize` is the
  §4.6.1.3 non-uniform inverse quantizer
  (`Sign(x_quant) · |x_quant|^(4/3)`, computed as `|x| · cbrt(|x|)`
  so perfect cubes invert exactly); `dequant::scale_factor_gain` is
  the §4.6.2.3.3 `get_scale_factor_gain()`
  (`2^(0.25 · (sf − SF_OFFSET))`, `SF_OFFSET = 100`);
  `dequant::rescale_spectrum` applies both band-wise over the
  §4.5.2.3.4 `sect_sfb_offset` ranges directly in the §4.5.2.3.5
  interleaved transmission order, consuming the
  `scale_factor_data::accumulate` absolute records in wire-order
  lockstep with `sfb_cb` (PNS / intensity records are consumed but
  produce silence — their §4.6.13 / §4.6.8 synthesis is a later
  tool). `decoded_spectrum::quant_to_spec` is the §4.6.3.3
  de-interleaver from group-interleaved order to the window-major
  `spec[w][k]` layout TNS / the filterbank consume;
  `decoded_spectrum::decode_channel_spectrum` composes the full
  per-channel stage — §4.6.3.3 pulse fix-up on `x_quant` →
  scalefactor accumulation → inverse quantization + rescaling →
  `quant_to_spec()` → §4.6.9 `tns_decode_frame`. New
  `Error::DequantInvalid` / `Error::QuantToSpecInvalid` cover the
  structural rejections. 16 unit tests (hand-derived spec-formula
  vectors: exact cubes, exact power-of-two gains, grouped-short
  shared gains, layout rejections) + 5 pipeline integration tests
  (`tests/decoded_spectrum.rs`: stage-composition equivalence,
  pulse-before-dequant ordering, TNS-stage equivalence) + the new
  `tests/docs_adts_corpus.rs` structural driver that walks all 12
  staged ADTS fixtures frame-by-frame (ADTS header →
  `raw_data_block()` walk → SCE / LFE / CPE bodies including the
  Table 4.4 `common_window` / `ms_mask_present` header →
  `spectral_data()` → pipeline) and asserts every decoded spectrum
  is finite with plausible energy (skips cleanly when `docs/` is
  absent in standalone-repo CI).
- phase 2 (r281): `spectral_data` — the ISO/IEC 14496-3 Table 4.56
  `spectral_data()` wire walker plus its bit-exact writer, the §4.4.6
  driver the codebook rounds were building toward. `SpectralData::
  parse` loops over the window groups and sections established by
  `ics_info()` / `section_data()`, dispatches per section onto the
  Codebook 1..=11 Huffman decoders, applies the §4.6.3.3
  `quad_sign_bits` / `pair_sign_bits` suffix on unsigned books, and
  reconstructs ESC-book magnitudes ≥ 16 from the `hcod_esc_y` /
  `hcod_esc_z` escape sequences (`2^(N+4) + escape_word`, capped at
  `MAX_QUANT` per §4.6.1.3). Loop bounds come from the new public
  `sect_sfb_offset()` helper implementing the §4.5.2.3.4 derivation
  (long windows mirror `swb_offset_long_window`; eight-short groups
  scale the short-window band widths by `window_group_length[g]`).
  Coefficients are surfaced per group in the §4.5.2.3.5 transmission
  (interleaved) order; the `quant_to_spec()` deinterleave into the
  window-major layout TNS/the filterbank consume is the next tool.
  `SpectralData::write` is the symmetric inverse (ESC clamping to the
  in-band `ESC_FLAG`, sign-bit derivation, escape re-encoding). New
  `Error::SpectralDataInvalid` / `Error::SpectralDataEncodeInvalid`
  cover the structural and encode-side rejections (reserved codebook
  12, `max_sfb > num_swb`, group-count/buffer-shape mismatches,
  non-zero coefficients in spectrum-less bands). 20 unit tests
  (`src/spectral_data.rs`) + 9 integration tests
  (`tests/spectral_data.rs`), including an `IcsBody` → `SpectralData`
  composition that parses a complete Table 4.50 channel-element body
  from one reader.
- phase 2 (r278): `tns_frame::tns_decode_frame` — the ISO/IEC
  14496-3 §4.6.9.3 per-frame TNS orchestration. Chains the
  `tns_data` wire parser, `tns_coef::tns_decode_coef_to_lpc`, and
  `tns_coef::tns_ar_filter` per the spec pseudocode: top-down
  region slicing from `bottom = num_swb` in units of scalefactor
  bands, `TNS_MAX_ORDER` clamping of the wire `order` (only the
  first `tns_order` magnitudes participate), the three-way
  `min(band, TNS_MAX_BANDS, max_sfb)` band clamp through the
  `swb_offset` tables, and direction-dependent upward/downward
  strided walks over each window's slice of the dequantised
  spectrum. Order-0 filters and fully-clamped regions are the
  pseudocode's `continue` no-ops. New `Error::TnsFrameInvalid`
  covers frame-level precondition violations (spectrum length ≠
  `num_windows × window_len`, window-count mismatch, `coef`
  shorter than the clamped order); `swb_offset` / `TNS_MAX_BANDS`
  coverage and wire-magnitude errors propagate unchanged. ER AAC
  LD 480/512-line frames stay deferred with the standing
  `int_tns_decode_coef()` deferral. The complete §4.6.9 TNS decode
  tool — wire parse → coefficient decode → region slicing →
  in-place filtering — is now in place. 18 unit tests
  (`src/tns_frame.rs`) + 5 wire-round-trip integration tests
  (`tests/tns_frame.rs`).

## [0.1.4](https://github.com/OxideAV/oxideav-aac/compare/v0.1.3...v0.1.4) - 2026-06-08

### Other

- phase 2 (r263): tns_coef — §4.6.9.3 tns_decode_coef + §C.6 encode + LPC step-up
- phase 2 (r259): spectrum_huffman::HCOD11 — Table 4.A.12 Spectrum Huffman Codebook 11
- phase 2 (r255): spectrum_huffman::HCOD10 — Table 4.A.11 Spectrum Huffman Codebook 10
- phase 2 (r253): spectrum_huffman::HCOD9 — Table 4.A.10 Spectrum Huffman Codebook 9
- phase 2 (r250): spectrum_huffman::HCOD8 — Table 4.A.9 Spectrum Huffman Codebook 8
- drop release-plz.toml — use release-plz defaults across the workspace

### Added

- phase 2 (r272): `tns_coef::tns_ar_filter` — the ISO/IEC 14496-3
  §4.6.9.3 `tns_ar_filter()` all-pole (auto-regressive) IIR pass.
  Runs the spec recurrence `y(n) = x(n) − lpc[1]·y(n−1) − … −
  lpc[order]·y(n−order)` in place over a strided region of the
  dequantised MDCT spectrum, consuming the direct-form `lpc[]` array
  produced by [`tns_coef::tns_decode_coef_to_lpc`]. Filter state is
  zero-seeded at every invocation, the output overwrites the input,
  and the strided walk honours both the upward (`inc = +1`) and the
  downward (`inc = −1`, `start = end − 1`) directions that
  §4.6.9.3 `tns_decode_frame` sets from the per-filter `direction`
  flag. An empty `lpc`, an `inc ∉ {−1, +1}`, or a `start`/`size`/`inc`
  triple whose strided walk leaves the spectrum bounds is rejected
  with [`Error::TnsCoefOutOfRange`]; an order-0 filter (`lpc ==
  [1.0]`) and a zero-length region are well-defined no-ops. With the
  inverse-quantisation + LPC step-up from r263, both TNS building
  blocks the §4.6.9 `tns_decode_frame` orchestration chains are now
  in place; only the `swb_offset` / grouping region slicer (which
  needs the per-frame spectral context) remains. 10 new unit tests
  (`src/tns_coef.rs`): order-0 identity, hand-checked order-1 impulse
  response, order-2/3 cross-checks against an independent reference
  recurrence, the downward-walk equivalence, region-isolation
  (sentinel-padded buffer), and the four argument-validation
  rejections, plus an end-to-end wire-`coef` → LPC → filter path.

- phase 2 (r263): `tns_coef` module — ISO/IEC 14496-3 §4.6.9.3
  `tns_decode_coef` inverse-quantisation and conversion-to-LPC
  step-up procedure, plus the §C.6 encoder-side companion
  `tns_encode_coef`. This is the bridge between the
  [`tns_data`](src/tns_data.rs) wire parser (which surfaces the
  unsigned-magnitude `coef[i]` field as transmitted, packed into
  `coef_res2 ∈ {2, 3, 4}` bits per the `coef_res[w] +
  3 − coef_compress` arithmetic) and the eventual `tns_ar_filter()`
  all-pole IIR pass that operates on the reconstructed MDCT
  spectrum. The decode path follows the §4.6.9.3 pseudocode literally:
  the wire `coef[i]` is sign-extended via the §4.6.9.3 `sgn_mask` /
  `neg_mask` pair (equivalent to a two's-complement extension of the
  truncated `coef_res2`-bit field), then inverse-quantised via
  `sin(tmp[i] / iqfac_branch)` where the divisor branches on the sign
  of the sign-extended index: `iqfac = ((1 << (coef_res_bits-1)) -
  0.5) / (π/2)` for non-negative indices and `iqfac_m = ((1 <<
  (coef_res_bits-1)) + 0.5) / (π/2)` for negative indices. The
  half-bit offset matches the §C.6 encoder's rounded
  `NINT(arcsin(r) * iqfac_branch)` quantisation so every legal wire
  value round-trips through decode → encode bit-exactly (35 wire
  patterns across all four `(coef_res_bits, coef_compress)`
  combinations, enumerated at unit-test time). The §4.6.9.3
  conversion-to-LPC step-up loop is implemented as [`lpc_step_up`]:
  it takes the inverse-quantised PARCOR array and produces the
  `order + 1` direct-form LPC `a[]` vector with `a[0] = 1.0` and
  `a[order] = parcor[order-1]`, with intermediate slots derived by
  the spec's `b[i] = a[i] + k * a[m-i]` cross-term recurrence
  (validated against hand-arithmetic at orders 1, 2, and 3). The
  combined wrapper [`tns_decode_coef_to_lpc`] runs both passes for
  the eventual `tns_decode_frame()` orchestration. Encoder-side
  saturation: `r = ±1.0` rounds to `±NINT(π/2 * iqfac_branch)` and
  clamps to the `coef_res2`-bit signed field extrema (`±7` /
  `±8`-clamped-to-`+7,-8` at `coef_res2 = 4`). Public API:
  [`iqfac`] / [`iqfac_m`] (the §4.6.9.3 scaling constants),
  [`sign_extend_coef`] / [`pack_coef`] (signed-magnitude wire ↔
  signed integer), [`tns_decode_coef`] / [`tns_encode_coef`] (the
  full inverse-quantisation surface), [`lpc_step_up`] (the §4.6.9.3
  conversion-to-LPC second loop), and [`tns_decode_coef_to_lpc`]
  (the combined wrapper). Argument-validation errors all surface as
  the new [`Error::TnsCoefOutOfRange`] variant: invalid
  `coef_res_bits` (anything outside `{3, 4}`), `coef_compress > 1`,
  wire `coef[i]` that overflows the `coef_res2`-bit field, encode
  PARCOR values `|r| > 1.0` (or NaN / ±∞ — `arcsin` undefined), or
  `pack_coef` values outside the field-representable signed range.
  Adds 36 new unit tests (the `iqfac` / `iqfac_m` per-width formula
  / branch ordering invariant / full sign-extension enumeration
  across 4 / 3 / 2-bit widths / `pack_coef` ↔ `sign_extend_coef`
  round-trip across 4 / 3 / 2-bit widths / `pack_coef` rejection
  outside field / decode zero-wire ↔ zero-PARCOR / decode extrema
  match hand-arithmetic / negative branch uses `iqfac_m` / 3-bit
  `coef_res_bits` decode / decode rejects oversized wire value under
  `coef_compress = 1` / decode rejects invalid `coef_res_bits` /
  decode rejects invalid `coef_compress` / decode empty input /
  encode zero ↔ zero / encode `±1.0` saturates to field extrema /
  encode rejects `|r| > 1.0` / encode rejects invalid
  `coef_res_bits` / full decode→encode round-trip across every
  4-bit / 3-bit / compressed wire pattern / step-up unit at order 0
  / 1 / 2 / 3 / `a[0] = 1` invariant / `a[order] = parcor[order-1]`
  invariant / combined `decode_to_lpc` wrapper / wrapper propagates
  decode errors) plus 11 new integration tests in
  `tests/tns_coef.rs` (`iqfac_m − iqfac == 2/π` invariant /
  full sign-extend↔pack round-trip enumeration / decoded PARCOR
  magnitudes are `|·| ≤ 1` across every config / full decode→encode
  round-trip across every legal wire pattern / batch of mixed signs
  preserves per-element branch selection / `coef_compress` shrinks
  field but not iqfac contrast / `decode_to_lpc` matches manual
  composition / step-up stability for `|k| < 1` PARCOR / canonical
  `Error::TnsCoefOutOfRange` propagation / realistic order-8 filter
  decay invariant). Lays the groundwork for the eventual
  `tns_ar_filter()` IIR pass; the §4.6.9 `tns_decode_frame()`
  orchestration that chains `tns_decode_coef_to_lpc` and
  `tns_ar_filter` per (window, filter) lands once the IMDCT
  back-end is present. The §4.6.17.3.4 ER AAC LD
  `int_tns_decode_coef()` integer variant is deferred until the LD
  reconstruction path is wired.
- phase 2 (r259): `spectrum_huffman::HCOD11` — ISO/IEC 14496-3 §4.6.3
  / Annex 4.A Table 4.A.12 (Spectrum Huffman Codebook 11) wire layer.
  Codebook 11 is the only **ESC** spectrum book — Table 4.95 row 11
  declares `unsigned_cb = 1`, `dim = 2`, `LAV = 16` with an ESC
  threshold of `8191` (the §4.6.1.3 `x_quant` ceiling). The §4.6.3.3
  in-band universe widens to a `17 × 17 = 289`-entry lattice indexed
  `0..=288` with each `(y, z)` coefficient in `0..=16`; a coefficient
  value of `16` in either slot is the `escape_flag` whose actual
  magnitude is reconstructed from the `escape_sequence` bridged by
  `crate::spectral_codebook::decode_esc_value` /
  `crate::spectral_codebook::encode_esc_value` (round 213) — separate
  from the Huffman codeword carried by this module. The §4.6.3.3
  unsigned polynomial `idx = y * 17 + z` parks the zero-tuple `(0, 0)`
  at index 0 with the shortest 4-bit codeword `0b0000`, shares that
  4-bit floor with the interior `(1, 1)` pair at index 18 (codeword
  `0b0001`), pins the half-ESC tuples `(0, 16)` and `(16, 0)` to
  10-bit `0x38e` (index 16) and 9-bit `0x1c2` (index 272), and parks
  the full-ESC corner `(16, 16)` at index 288 with the 5-bit `0b00100`
  (`0x04`) — the wire layout extends with two sign bits and two
  escape sequences for that corner, so the in-band Huffman codeword
  stays short. The codeword ceiling matches Codebook 10's 12 bits —
  exactly six rows reach it (indices 12, 14, 15, 255, 269, 270 with
  codewords `0xffb`, `0xffa`, `0xffe`, `0xffd`, `0xffc`, `0xfff`) —
  because Codebook 11 pushes its tail distribution into the §4.6.3
  ESC sequence rather than spending longer Huffman codewords on it.
  Public constants `HCOD11_NUM_ENTRIES = 289`, `HCOD11_MAX_LEN = 12`.
  Public functions `hcod11_encode(idx) -> (length, codeword)`
  (right-aligned in `u16`), `hcod11_decode(reader) -> idx` (MSB-first
  prefix match, linear scan against the 289-row table), and the
  convenience writer `hcod11_write(writer, idx)`. Out-of-range indices
  (`idx > 288`) surface as `Error::SpectralCodebookIndexOutOfRange(11)`;
  the decoder surfaces `Error::UnexpectedEnd` on reader underflow. The
  table is a **complete** prefix code (Kraft equality
  `Σ 2^(12 − L) = 4096 = 2^12`), exhaustively verified by walking
  every 12-bit prefix at unit-test time and asserting each maps to
  exactly one entry — so the decoder's `unreachable!()` fall-through
  is dead by construction. Adds 20 new unit tests and 30 new
  integration tests covering 289-entry shape / 12-bit max / 4-bit
  min at indices 0 and 18 / per-row PDF spot checks at indices 0 / 1
  / 12 / 14 / 15 / 16 / 18 / 255 / 269 / 270 / 272 / 288 /
  six-12-bit-ceiling enumeration / half-ESC tuple ↔ index 16 and 272
  cross-check / Kraft equality / complete-prefix walk over all `2^12`
  prefixes / Table 4.95 row 11 ↔ Table 4.A.12 size cross-check /
  full writer→reader round-trip with bit-consumption invariant /
  the §4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check / 289-pair
  bijection sweep / per-index sign-bit-count invariant against
  `derive_sign_bits(11, …)` / zero-tuple zero-sign-bits invariant /
  far-corner two-sign-bits invariant / hand-pinned byte sequences for
  indices 0 and 270 / encode and write rejection at and above 289 /
  truncation rejection / Codebook-10-and-11-share-12-bit-ceiling
  cross-check / Codebook-11-is-the-only-ESC-spectrum-book cross-check
  / in-band ↔ ESC-border disjoint-set partition (256 in-band pairs +
  33 ESC-border pairs = 289). Completes the AAC spectrum Huffman
  Codebook 1..=11 table set; the next step is the §4.4.6
  `spectral_data()` wire walker.
- phase 2 (r255): `spectrum_huffman::HCOD10` — ISO/IEC 14496-3 §4.6.3
  / Annex 4.A Table 4.A.11 (Spectrum Huffman Codebook 10) wire layer.
  Codebook 10 is the second **expanded-LAV unsigned pair** spectrum
  book — Table 4.95 row 10 mirrors row 9 column-for-column
  (`unsigned_cb = 1`, `dim = 2`, `LAV = 12`) so the §4.6.3.3 universe
  is the same `13 × 13 = 169`-entry lattice indexed `0..=168` with
  each `(y, z)` coefficient in `0..=12`. Codebook 10 differs from
  Codebook 9 in its codeword-length distribution: where Codebook 9
  parks the §4.6.3.3 zero-tuple `(0, 0)` at index 0 with a single-bit
  `0` codeword and lets the rarest pair magnitudes reach a 15-bit
  ceiling, Codebook 10 lifts the zero-tuple at index 0 to a 6-bit
  `0b100010` (`0x22`), migrates the shortest 4-bit slot onto the
  interior `(1, 1)` tuple at index 14 with codeword `0b0000`, and
  pulls the codeword ceiling down to **12 bits** — the same head-
  displacement pattern Codebook 8 uses relative to Codebook 7 at the
  narrower `LAV = 7` universe. Exactly three rows reach the 4-bit
  floor (indices 14, 15, 27) and exactly eight rows reach the 12-bit
  ceiling (indices 12, 129, 142, 155, 165, 166, 167, 168), reflecting
  an encoder target whose magnitude statistics put more weight in the
  `(1..=4, 1..=4)` interior than Codebook 9's zero-heavy
  distribution. The §4.6.3.3 sign-bit suffix applies after every
  non-zero coefficient (one suffix bit per non-zero, low-frequency
  first via `apply_sign_bits` / `derive_sign_bits`) and lies outside
  the Huffman codeword carried by this module. Public constants
  `HCOD10_NUM_ENTRIES = 169`, `HCOD10_MAX_LEN = 12`. Public functions
  `hcod10_encode(idx) -> (length, codeword)` (right-aligned in
  `u16`), `hcod10_decode(reader) -> idx` (MSB-first prefix match,
  linear scan against the 169-row table), and the convenience writer
  `hcod10_write(writer, idx)`. Out-of-range indices (`idx > 168`)
  surface as `Error::SpectralCodebookIndexOutOfRange(10)`; the
  decoder surfaces `Error::UnexpectedEnd` on reader underflow. The
  table is a **complete** prefix code (Kraft equality `Σ 2^(12 − L)
  = 4096 = 2^12`), exhaustively verified by walking every 12-bit
  prefix at unit-test time and asserting each maps to exactly one
  entry — so the decoder's `unreachable!()` fall-through is dead by
  construction. Adds 17 new unit tests and 22 new integration tests
  covering table shape, Kraft completeness, per-row PDF spot checks,
  the §4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check, the
  169-pair bijection sweep, per-index sign-bit-count invariants
  against `derive_sign_bits(10, …)`, hand-pinned byte sequences for
  indices 14 and 168, rejection paths, and the Codebooks 9 / 10
  shared-universe / pulled-down-ceiling / displaced-zero-tuple
  cross-checks.
- phase 2 (r253): `spectrum_huffman::HCOD9` — ISO/IEC 14496-3 §4.6.3 /
  Annex 4.A Table 4.A.10 (Spectrum Huffman Codebook 9) wire layer.
  Codebook 9 is the first **expanded-LAV unsigned pair** spectrum
  book — Table 4.95 row 9 declares `unsigned_cb = 1`, `dim = 2`,
  `LAV = 12`, so the §4.6.3.3 universe shifts from Codebooks 7 / 8's
  shared `8 × 8 = 64`-entry `LAV = 7` lattice to a `(12 + 1)^2 =
  13^2 = 169`-entry lattice indexed `0..=168` with each `(y, z)`
  coefficient in `0..=12`. The §4.6.3.3 unsigned polynomial
  `idx = y * (LAV + 1) + z = y * 13 + z` parks the zero-tuple
  `(0, 0)` at index 0 with the 1-bit `0` codeword — matching the
  head-placement Codebook 7 uses — and pins the maximum tuple
  `(12, 12)` at index 168 with a 15-bit `0x7fff`. The maximum
  codeword length is **15 bits**, the widest non-ESC codeword in
  Annex 4.A — a 5-bit jump over Codebook 8's 10-bit ceiling
  reflecting the `169 / 64 ≈ 2.6×` universe expansion. Exactly four
  rows reach the 15-bit ceiling: indices 142 (`0x7ffc`), 154
  (`0x7ffd`), 155 (`0x7ffe`), and 168 (`0x7fff`) — the rarest pair
  magnitudes near the `LAV = 12` cap. The §4.6.3.3 sign-bit suffix
  applies after every non-zero coefficient (one suffix bit per
  non-zero, low-frequency first via `apply_sign_bits` /
  `derive_sign_bits`) and lies outside the Huffman codeword carried
  by this module. Public constants `HCOD9_NUM_ENTRIES = 169`,
  `HCOD9_MAX_LEN = 15`. Public functions `hcod9_encode(idx) ->
  (length, codeword)` (right-aligned in `u16`), `hcod9_decode(reader)
  -> idx` (MSB-first prefix match, linear scan against the 169-row
  table), and the convenience writer `hcod9_write(writer, idx)`.
  Out-of-range indices (`idx > 168`) surface as
  `Error::SpectralCodebookIndexOutOfRange(9)`; the decoder surfaces
  `Error::UnexpectedEnd` on reader underflow. The table is a
  **complete** prefix code (Kraft equality `Σ 2^(15 − L) = 32768 =
  2^15`), exhaustively verified by walking every 15-bit prefix at
  unit-test time and asserting each maps to exactly one entry — so
  the decoder's `unreachable!()` fall-through is dead by
  construction. Adds 16 new unit tests and 25 new integration tests
  covering table shape, Kraft completeness, per-row PDF spot checks,
  the §4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check, the
  169-pair bijection sweep, per-index sign-bit-count invariants
  against `derive_sign_bits(9, …)`, hand-pinned byte sequences for
  indices 0 and 168, rejection paths, and the Codebooks 7 / 8 / 9
  expanded-LAV / head-placement / ceiling-delta cross-checks.
  Lands one of the three remaining `Codebooks 9..=11` line items
  in the §4.6.3 spectrum-data trail and adds the largest non-ESC
  spectrum book in the entire Annex 4.A book set.

- phase 2 (r250): `spectrum_huffman::HCOD8` — ISO/IEC 14496-3 §4.6.3 /
  Annex 4.A Table 4.A.9 (Spectrum Huffman Codebook 8) wire layer.
  Codebook 8 is the second **unsigned pair** spectrum book and
  shares Codebook 7's Table 4.95 row shape (row 8: `unsigned_cb =
  1`, `dim = 2`, `LAV = 7` → `(7 + 1)^2 = 8^2 = 64` entries indexed
  `0..=63`, each tuple coefficient in `0..=7`) but uses a different
  per-row Huffman length tuning. Where Codebook 7 puts the zero-tuple
  `(0, 0)` at index 0 with a 1-bit `0` codeword and lets the
  upper-right quadrant of the lattice climb to a 12-bit ceiling,
  Codebook 8 lifts the zero-tuple at index 0 to a 5-bit `0xe` and
  migrates the shortest codeword (3 bits `0b000`) to the interior
  tuple `(y, z) = (1, 1)` at **index 9**. The maximum codeword
  length is **10 bits** (vs 12 for Codebook 7); exactly four rows
  reach the ceiling: indices 7 (`0x3fe`), 47 (`0x3fc`), 56
  (`0x3fd`), and 63 (`0x3ff`) — the rarest pair magnitudes. The
  §4.6.3.3 sign-bit suffix applies after every non-zero coefficient
  (one suffix bit per non-zero, low-frequency first via
  `apply_sign_bits` / `derive_sign_bits`) and lies outside the
  Huffman codeword carried by this module. Public constants
  `HCOD8_NUM_ENTRIES = 64`, `HCOD8_MAX_LEN = 10`. Public functions
  `hcod8_encode(idx) -> (length, codeword)` (right-aligned in `u16`),
  `hcod8_decode(reader) -> idx` (MSB-first prefix match, linear scan
  against the 64-row table), and the convenience writer
  `hcod8_write(writer, idx)`. Out-of-range indices (`idx > 63`)
  surface as `Error::SpectralCodebookIndexOutOfRange(8)`; the
  decoder surfaces `Error::UnexpectedEnd` on reader underflow. The
  table is a **complete** prefix code (Kraft equality
  `Σ 2^(10 − L) = 1024 = 2^10`), exhaustively verified by walking
  every 10-bit prefix at unit-test time and asserting each maps to
  exactly one entry — so the decoder's `unreachable!()` fall-through
  is verifiably dead. Encode-then-decode round-trips every index;
  17 new unit tests cover table-shape invariants (entry count,
  max/min length at index 9, codeword fit), Kraft equality,
  exhaustive completeness, four spot-checks against Table 4.A.9
  (rows 0, 8, 9, 63, and the four 10-bit ceiling rows), the
  round-trip walk, both out-of-range error paths, the
  `UnexpectedEnd` propagation, and two cross-checks against
  Codebook 7 (shortest-codeword-slot disagreement and shared
  far-corner `(7, 7)` placement at index 63). All numeric values
  sourced from `docs/audio/aac/ISO_IEC_14496-3-AAC-2001.pdf` §4.A.1
  page 198. Module docstring extended with a Codebook 8 invariants
  block (Table 4.95 row 8 mapping, entry-count derivation,
  polynomial origin, ceiling row inventory). Codebooks 9..=11
  (Tables 4.A.10 … 4.A.12) reuse the same module shape and will
  land one per future round; Codebook 9 (Table 4.A.10) is the
  first **expanded-LAV pair** book (Table 4.95 row 9: `unsigned =
  1`, `dim = 2`, `LAV = 12` → 169 entries on a `13 × 13` lattice).
- phase 2 (r244): `spectrum_huffman::HCOD7` — ISO/IEC 14496-3 §4.6.3 /
  Annex 4.A Table 4.A.8 (Spectrum Huffman Codebook 7) wire layer.
  Codebook 7 is the first **unsigned pair** spectrum book (Table 4.95
  row 7: `unsigned_cb = 1`, `dim = 2`, `LAV = 7` → `(7 + 1)^2 = 8^2 =
  64` entries indexed `0..=63`, each tuple coefficient in `0..=7`).
  The §4.6.3.3 polynomial `idx = y * (LAV + 1) + z = y * 8 + z` parks
  the zero-tuple `(0, 0)` at index 0 (the origin of the unsigned
  dim-2 lattice) and the far corner `(7, 7)` at index 63. The
  shortest codeword is a single bit `0` at index 0 — the same head
  placement Codebook 3 uses for its unsigned dim-4 universe. The
  maximum codeword length is **12 bits**; exactly four rows reach
  that ceiling: indices 54 (`0xffd`), 55 (`0xffe`), 62 (`0xffc`),
  and 63 (`0xfff`). The §4.6.3.3 sign-bit suffix applies after every
  non-zero coefficient (one suffix bit per non-zero, low-frequency
  first via `apply_sign_bits` / `derive_sign_bits`) and lies outside
  the Huffman codeword carried by this module. Public constants
  `HCOD7_NUM_ENTRIES = 64`, `HCOD7_MAX_LEN = 12`. Public functions
  `hcod7_encode(idx) -> (length, codeword)` (right-aligned in `u16`),
  `hcod7_decode(reader) -> idx` (MSB-first prefix match, linear scan
  against the 64-row table), and the convenience writer
  `hcod7_write(writer, idx)`. Out-of-range indices (`idx > 63`)
  surface as `Error::SpectralCodebookIndexOutOfRange(7)`; the
  decoder surfaces `Error::UnexpectedEnd` on reader underflow. The
  table is a **complete** prefix code (Kraft equality
  `Σ 2^(12 − L) = 4096 = 2^12`), exhaustively verified by walking
  every 12-bit prefix at unit-test time and asserting each maps to
  exactly one entry — so the decoder's `unreachable!()` fall-through
  is verifiably dead. Encode-then-decode round-trips every index;
  16 new unit tests cover table-shape invariants (entry count,
  max/min length, codeword fit), Kraft equality, exhaustive
  completeness, four spot-checks against Table 4.A.8 (rows 0, 8, 63,
  and the four 12-bit ceiling rows), the round-trip walk, both
  out-of-range error paths, the `UnexpectedEnd` propagation, and
  two cross-checks against Codebook 3 (zero-tuple placement) and
  Codebook 4 (dim-2 vs dim-4 universe size). All numeric values
  sourced from `docs/audio/aac/ISO_IEC_14496-3-AAC-2001.pdf` §4.A.1
  page 198. Module docstring extended with a Codebook 7 invariants
  block (Table 4.95 row 7 mapping, entry-count derivation,
  polynomial origin, ceiling row inventory). Codebooks 8..=11
  (Tables 4.A.9 … 4.A.12) reuse the same module shape and will land
  one per future round.
- phase 2 (r241): `spectrum_huffman::HCOD6` — ISO/IEC 14496-3 §4.6.3 /
  Annex 4.A Table 4.A.7 (Spectrum Huffman Codebook 6) wire layer.
  Codebook 6 is the second **signed pair** spectrum book and shares
  Codebook 5's Table 4.95 row shape (row 6: `unsigned_cb = 0`,
  `dim = 2`, `LAV = 4` → `(2 * 4 + 1)^2 = 9^2 = 81` entries indexed
  `0..=80`, each tuple coefficient in `-4..=+4`) but uses a different
  per-row Huffman length tuning. Where Codebook 5 puts the zero-tuple
  `(0, 0)` at index 40 with a 1-bit `0` codeword and lets the four
  lattice corners stretch out to a 13-bit ceiling, Codebook 6 lifts
  the zero-tuple to a 4-bit `0b0000` and tightens the maximum
  codeword length back to **11 bits**. Exactly four rows occupy that
  ceiling: indices 0 (`0x7fe`), 8 (`0x7fd`), 72 (`0x7ff`), and
  80 (`0x7fc`) — the same four `(±4, ±4)` lattice corners Codebook 5
  also pinned to its ceiling. No sign-bit suffix follows the
  codeword on the wire (Codebook 6 is signed: the index alone fully
  specifies each signed pair via the `offset = 4` shift). Public
  constants `HCOD6_NUM_ENTRIES = 81`, `HCOD6_MAX_LEN = 11`. Public
  functions `hcod6_encode(idx) -> (length, codeword)` (right-aligned
  in `u16`), `hcod6_decode(reader) -> idx` (MSB-first prefix match,
  linear scan against the 81-row table), and the convenience writer
  `hcod6_write(writer, idx)`. Out-of-range indices (`idx > 80`)
  surface as `Error::SpectralCodebookIndexOutOfRange(6)`; the
  decoder surfaces `Error::UnexpectedEnd` on reader underflow. The
  table is a **complete** prefix code (Kraft equality
  `Σ 2^(11 − L) = 2048 = 2^11`), exhaustively verified by walking
  every 11-bit prefix at unit-test time and asserting each maps to
  exactly one entry — so the decoder's `unreachable!()` fall-through
  is verifiably dead. Encode-then-decode round-trips every index for
  both the direct API and the cross-check against the round-213
  `spectral_codebook::decode_index_to_tuple` /
  `encode_tuple_to_index` §4.6.3.3 translation across the entire
  signed pair lattice. A `derive_sign_bits(6, &tuple)` cross-check
  confirms every Codebook 6 index emits zero sign bits on the wire,
  matching the signed-book contract. 16 new unit tests in
  `src/spectrum_huffman.rs` and 24 new integration tests in
  `tests/spectrum_huffman.rs` (eight per-row PDF spot checks at
  indices 0 / 8 / 13 / 31 / 40 / 41 / 72 / 80; Table 4.95 row 6 ↔
  Table 4.A.7 size cross-check; full writer→reader round-trip; the
  §4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check; zero-tuple
  ↔ index-40 invariant; per-index zero-sign-bit invariant; three
  hand-pinned byte sequences (`[0x00]` for index 40 padded;
  `[0xff, 0xe0]` for index 72 padded; `[0x03]` for indices 40 + 41
  packed); exact bit-consumption invariant across every index;
  out-of-range and truncation rejections; `HCOD6_MAX_LEN` constant
  consistency; a same-shape cross-check confirming Codebooks 5 and 6
  share the signed-pair Table 4.95 row but disagree on per-index
  codewords; and a corner-position cross-check confirming both
  books pin the four `(±4, ±4)` lattice corners to their respective
  maximum-length codewords at indices 0 / 8 / 72 / 80). Suite grows
  728 → 768 tests.

- phase 2 (r238): `spectrum_huffman::HCOD5` — ISO/IEC 14496-3 §4.6.3 /
  Annex 4.A Table 4.A.6 (Spectrum Huffman Codebook 5) wire layer.
  Codebook 5 is the first **pair** spectrum book (Table 4.95 row 5:
  `unsigned_cb = 0`, `dim = 2`, `LAV = 4` → `(2 * 4 + 1)^2 = 9^2 = 81`
  entries indexed `0..=80`, each tuple coefficient in `-4..=+4`). The
  zero-tuple `(0, 0)` lands at index 40 — the centre row — because
  the §4.6.3.3 polynomial `idx = (y + LAV) * 9 + (z + LAV)` puts the
  origin at the centre of the index range for signed pair books; the
  shortest codeword (1 bit `0`) parks there too. The maximum
  codeword length is **13 bits** and exactly four rows occupy the
  ceiling: indices 0 (`0x1fff`), 8 (`0x1ffd`), 72 (`0x1ffc`), and
  80 (`0x1ffe`) — the four `(±4, ±4)` corners of the signed `9 × 9`
  pair lattice. No sign-bit suffix follows the codeword on the wire
  (Codebook 5 is signed: the index alone fully specifies each
  signed pair via the `offset = 4` shift). Public constants
  `HCOD5_NUM_ENTRIES = 81`, `HCOD5_MAX_LEN = 13`. Public functions
  `hcod5_encode(idx) -> (length, codeword)` (right-aligned in `u16`),
  `hcod5_decode(reader) -> idx` (MSB-first prefix match, linear scan
  against the 81-row table), and the convenience writer
  `hcod5_write(writer, idx)`. Out-of-range indices (`idx > 80`)
  surface as `Error::SpectralCodebookIndexOutOfRange(5)`; the
  decoder surfaces `Error::UnexpectedEnd` on reader underflow. The
  table is a **complete** prefix code (Kraft equality
  `Σ 2^(13 − L) = 8192 = 2^13`), exhaustively verified by walking
  every 13-bit prefix at unit-test time and asserting each maps to
  exactly one entry — so the decoder's `unreachable!()` fall-through
  is verifiably dead. Encode-then-decode round-trips every index for
  both the direct API and the cross-check against the round-213
  `spectral_codebook::decode_index_to_tuple` /
  `encode_tuple_to_index` §4.6.3.3 translation across the entire
  signed pair lattice. A `derive_sign_bits(5, &tuple)` cross-check
  confirms every Codebook 5 index emits zero sign bits on the wire,
  matching the signed-book contract. 16 new unit tests in
  `src/spectrum_huffman.rs` and 21 new integration tests in
  `tests/spectrum_huffman.rs` (eight per-row PDF spot checks at
  indices 0 / 8 / 13 / 31 / 40 / 41 / 72 / 80; Table 4.95 row 5 ↔
  Table 4.A.6 size cross-check; full writer→reader round-trip; the
  §4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check; zero-tuple
  ↔ index-40 invariant plus the four-corner `(±4, ±4)` invariant;
  per-index zero-sign-bit invariant; three hand-pinned byte
  sequences; exact bit-consumption invariant across every index;
  out-of-range and truncation rejections; `HCOD5_MAX_LEN` constant
  consistency; and a Codebook 5-is-the-first-pair-book cross-check
  confirming Codebooks 1..=4 are all dim-4 while Codebook 5 is
  dim-2). Suite grows 691 → 728 tests.

- phase 2 (r234): `spectrum_huffman::HCOD4` — ISO/IEC 14496-3 §4.6.3 /
  Annex 4.A Table 4.A.5 (Spectrum Huffman Codebook 4) wire layer.
  Codebook 4 shares Codebook 3's unsigned dim-4 LAV-2 tuple universe
  (Table 4.95 row 4 = row 3 except for the source-table column:
  `unsigned_cb = 1`, `dim = 4`, `LAV = 2` → `3^4 = 81` entries indexed
  `0..=80`, each tuple coefficient in `0..=2`, with the §4.6.3.3
  sign-bit suffix carrying the sign of every non-zero coefficient
  outside the Huffman codeword) but uses a different per-row Huffman
  length tuning: maximum codeword length is **12 bits** (vs 16 for
  Codebook 3); the **shortest** codeword (4 bits `0b0000`) sits at
  **index 40** while index 0 (still the §4.6.3.3 zero-tuple
  `(0, 0, 0, 0)`) carries a 4-bit `0b0111`; the two 12-bit rows are
  index 62 (`0xfff`) and index 74 (`0xffe`). Public constants
  `HCOD4_NUM_ENTRIES = 81`, `HCOD4_MAX_LEN = 12`. Public functions
  `hcod4_encode(idx) -> (length, codeword)` (right-aligned in `u16`),
  `hcod4_decode(reader) -> idx` (MSB-first prefix match, linear scan
  against the 81-row table), and the convenience writer
  `hcod4_write(writer, idx)`. Out-of-range indices (`idx > 80`)
  surface as `Error::SpectralCodebookIndexOutOfRange(4)`; the
  decoder surfaces `Error::UnexpectedEnd` on reader underflow. The
  table is a **complete** prefix code (Kraft equality
  `Σ 2^(12 − L) = 4096 = 2^12`), exhaustively verified by walking
  every 12-bit prefix at unit-test time and asserting each maps to
  exactly one entry — so the decoder's `unreachable!()` fall-through
  is verifiably dead. Encode-then-decode round-trips every index for
  both the direct API and the cross-check against the round-213
  `spectral_codebook::decode_index_to_tuple` /
  `encode_tuple_to_index` §4.6.3.3 translation. A Codebook 3 /
  Codebook 4 cross-check confirms the two books map every index to
  the *same* magnitude tuple — only the codeword assignment differs,
  since the §4.6.3.3 translation depends on the Table 4.95 row shape
  (identical for rows 3 and 4) and not on the codeword bit pattern.
  16 new unit tests in `src/spectrum_huffman.rs` (table-shape: 81
  entries / max 12 bits / min 4 bits at index 40 / codewords fit
  declared length; Kraft equality sum = 4096; complete-prefix walk
  over all `2^12` prefixes; per-row PDF spot checks at indices 0 / 40
  / 62 / 74 / 80; encode + decode rejection; decode of four zero bits
  yields index 40; decode of full 12-bit codeword `0xfff` yields
  index 62; reader underflow; full round-trip across every index;
  cross-book Codebook 3 ↔ Codebook 4 zero-tuple-codeword
  disagreement) plus 24 new integration tests in
  `tests/spectrum_huffman.rs` (seven per-row PDF spot checks at
  indices 0 / 13 / 27 / 40 / 62 / 74 / 80; Table 4.95 row 4 ↔ Table
  4.A.5 size cross-check; full writer→reader round-trip; the §4.6.3.3
  wire-index ↔ tuple ↔ wire-index cross-check; Codebook 3 ↔ Codebook
  4 tuple-equivalence cross-check; zero-tuple ↔ index-0 invariant;
  full magnitude tuple ↔ index 80 invariant; zero-sign-bit /
  four-sign-bit / per-index-sign-count cross-checks against
  `derive_sign_bits`; three hand-pinned byte sequences (`[0x00]` for
  index 40 padded; `[0xff, 0xf0]` for index 62 padded; `[0x07]` for
  index 40 + index 0 packed); exact bit-consumption invariant across
  every index; out-of-range and truncation rejections;
  `HCOD4_MAX_LEN` constant consistency; and a Codebook 3 / Codebook 4
  max-codeword-length cross-check). Suite size grows 651 → 691 tests.
  Codebooks 5..=11 (Tables 4.A.6 … 4.A.12) reuse the same encode /
  decode shape and will land one per future round; Codebooks 5 and 6
  are the first **pair** (`dim = 2`) books with `LAV = 4`, opening a
  new codebook geometry (`9^2 = 81` entries per book).
- phase 2 (r231): `spectrum_huffman::HCOD3` — ISO/IEC 14496-3 §4.6.3 /
  Annex 4.A Table 4.A.4 (Spectrum Huffman Codebook 3) wire layer.
  Codebook 3 is the first *unsigned* spectrum book (Table 4.95 row 3:
  `unsigned_cb = 1`, `dim = 4`, `LAV = 2`); the index space is still
  `3^4 = 81` (the `(LAV + 1)^dim` polynomial for unsigned books
  coincides with the `(2*LAV + 1)^dim` of the `LAV = 1` signed books)
  but the tuple universe is now `0..=2` per coefficient instead of
  `-1..=+1`, and the §4.6.3.3 sign-bit suffix carries the sign of
  each non-zero coefficient outside the Huffman codeword. Maximum
  codeword length is **16 bits** (vs 11 for Codebook 1 and 9 for
  Codebook 2). The zero magnitude tuple `(0, 0, 0, 0)` lives at
  **index 0** (not 40 as in the signed books) because the unsigned
  polynomial puts all-zero at the origin; it carries the single-bit
  codeword `0`. Two distinct rows carry the full 16-bit codewords:
  index 62 (`0xffff`) and index 74 (`0xfffe`). Public constants
  `HCOD3_NUM_ENTRIES = 81`, `HCOD3_MAX_LEN = 16`. Public functions
  `hcod3_encode(idx) -> (length, codeword)` (right-aligned in `u16`),
  `hcod3_decode(reader) -> idx` (MSB-first prefix match, linear scan
  against the 81-row table), and the convenience writer
  `hcod3_write(writer, idx)`. Out-of-range indices (`idx > 80`)
  surface as `Error::SpectralCodebookIndexOutOfRange(3)`; the
  decoder surfaces `Error::UnexpectedEnd` on reader underflow. The
  table is a **complete** prefix code (Kraft equality
  `Σ 2^(16 − L) = 65536 = 2^16`), exhaustively verified by walking
  every 16-bit prefix at unit-test time and asserting each maps to
  exactly one entry — so the decoder's `unreachable!()`
  fall-through is verifiably dead. Encode-then-decode round-trips
  every index for both the direct API and the cross-check against
  the round-213 `spectral_codebook::decode_index_to_tuple` /
  `encode_tuple_to_index` §4.6.3.3 translation. The §4.6.3.3
  sign-bit suffix (`derive_sign_bits`) is exercised as a cross-check:
  every Codebook 3 index emits exactly as many sign bits as it has
  non-zero coefficients (the zero-tuple emits zero sign bits, the
  all-twos tuple emits four). 17 new unit tests in
  `src/spectrum_huffman.rs` (table-shape: 81 entries / max 16 bits /
  min 1 bit at index 0 / codewords fit declared length; Kraft
  equality sum = 65536; complete-prefix walk over all `2^16`
  prefixes; per-row PDF spot checks at indices 0 / 62 / 80;
  encode + decode rejection; decode of single zero bit yields
  index 0; decode of full 16-bit codeword `0xffff` yields index 62;
  reader underflow; full round-trip across every index;
  cross-book zero-tuple-position contrast between Codebook 1
  (index 40) and Codebook 3 (index 0)) plus 21 new integration
  tests in `tests/spectrum_huffman.rs` (seven per-row PDF spot
  checks at indices 0 / 1 / 27 / 40 / 62 / 74 / 80; Table 4.95 row
  3 ↔ Table 4.A.4 size cross-check; full writer→reader round-trip;
  the §4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check;
  zero-tuple ↔ index-0 invariant; full magnitude tuple ↔ index 80
  invariant; zero-sign-bit / four-sign-bit / per-index-sign-count
  cross-checks against `derive_sign_bits`; three hand-pinned byte
  sequences (`[0x00]` for index 0 padded; `[0xff, 0xff]` for index
  62; `[0x48]` for index 0 + index 1 packed); exact
  bit-consumption invariant across every index; out-of-range and
  truncation rejections; `HCOD3_MAX_LEN` constant consistency; and
  a cross-book tuple-universe disjointness check confirming a
  negative-entry tuple cannot encode under Codebook 3 and a
  magnitude-2 tuple cannot encode under Codebook 1). Suite size
  grows 613 → 651 tests. Codebooks 4..=11 (Tables 4.A.5 … 4.A.12)
  reuse the same encode / decode shape and will land one per
  future round.
- phase 2 (r226): `spectrum_huffman::HCOD2` — ISO/IEC 14496-3 §4.6.3 /
  Annex 4.A Table 4.A.3 (Spectrum Huffman Codebook 2) wire layer.
  Codebook 2 shares Codebook 1's tuple universe (signed 4-tuple,
  `unsigned_cb = 0`, `dim = 4`, `LAV = 1` → `3^4 = 81` entries indexed
  `0..=80`) but uses a different per-row Huffman length tuning:
  maximum codeword length is 9 bits (vs 11 for Codebook 1); index 40
  (the zero-tuple) carries the 3-bit codeword `0b000` (vs the
  single-bit `0` in Codebook 1); index 67 carries the shortest
  non-zero-tuple codeword at 4 bits (`0b0010`). Public constants
  `HCOD2_NUM_ENTRIES = 81`, `HCOD2_MAX_LEN = 9`. Public functions
  `hcod2_encode(idx) -> (length, codeword)` (right-aligned in `u16`),
  `hcod2_decode(reader) -> idx` (MSB-first prefix match, linear scan
  against the 81-row table), and the convenience writer
  `hcod2_write(writer, idx)`. Out-of-range indices (`idx > 80`)
  surface as `Error::SpectralCodebookIndexOutOfRange(2)`; the decoder
  surfaces `Error::UnexpectedEnd` on reader underflow. The table is a
  **complete** prefix code (Kraft equality
  `Σ 2^(9 − L) = 512 = 2^9`), exhaustively verified by walking every
  9-bit prefix at unit-test time and asserting each maps to exactly
  one entry — so the decoder's `unreachable!()` fall-through is
  verifiably dead. Encode-then-decode round-trips every index for
  both the direct API and the cross-check against the round-213
  `spectral_codebook::decode_index_to_tuple` /
  `encode_tuple_to_index` §4.6.3.3 translation. An explicit
  cross-check confirms Codebook 1 and Codebook 2 map every shared
  index to the same `(w, x, y, z)` spectral tuple — only the Huffman
  codewords differ, since the §4.6.3.3 translation depends on the
  Table 4.95 row shape (which is identical for both books) and not
  on the codeword assignment. 16 new unit tests in
  `src/spectrum_huffman.rs` (table-shape: 81 entries / max 9 bits /
  min 3 bits at index 40 / codewords fit declared length; Kraft
  equality sum = 512; complete-prefix walk over all 2^9 prefixes;
  per-row PDF spot checks at indices 0 / 40 / 80; encode rejection;
  decode of `0b000` and of a 9-bit `0x1f3`; reader underflow; full
  round-trip across every index; Codebook 1 ↔ Codebook 2 disagreement
  on the zero-tuple codeword length) plus 17 integration tests in
  `tests/spectrum_huffman.rs` (six per-row PDF spot checks at indices
  0 / 13 / 40 / 67 / 78 / 80; Table 4.95 row 2 ↔ Table 4.A.3 size
  cross-check; full writer→reader round-trip; the §4.6.3.3 wire-index
  ↔ tuple ↔ wire-index cross-check; tuple-equivalence cross-check
  between Codebooks 1 and 2; three hand-pinned byte sequences
  (`[0x00]` for index 40 padded; `[0xf9, 0x80]` for index 0 padded;
  `[0x1f, 0x30]` for index-40 + index-0 packed); exact
  bit-consumption invariant across every index; out-of-range and
  truncation rejections; `HCOD2_MAX_LEN` constant consistency).
  Suite size grows 580 → 613 tests. Codebooks 3..=11
  (Tables 4.A.4 … 4.A.12) reuse the same encode / decode shape and
  will land one per future round.
- phase 2 (r219): `spectrum_huffman` module — ISO/IEC 14496-3 §4.6.3 /
  Annex 4.A Table 4.A.2 (Spectrum Huffman Codebook 1) wire layer.
  Codebook 1 is the signed 4-tuple book (`unsigned_cb = 0`, `dim = 4`,
  `LAV = 1`) so the table enumerates every 4-tuple of coefficients in
  `(-1, 0, +1)^4` → `3^4 = 81` entries with codeword lengths
  `1..=11` bits. Index 40 — the zero-tuple `(0, 0, 0, 0)` — carries
  the single-bit codeword `0`. Public constants
  `HCOD1_NUM_ENTRIES = 81`, `HCOD1_MAX_LEN = 11`. Public functions
  `hcod1_encode(idx) -> (length, codeword)` (right-aligned in
  `u16`), `hcod1_decode(reader) -> idx` (MSB-first prefix match,
  linear scan against the 81-row table), and the convenience writer
  `hcod1_write(writer, idx)`. Out-of-range indices
  (`idx > 80`) surface as `Error::SpectralCodebookIndexOutOfRange(1)`;
  the decoder surfaces `Error::UnexpectedEnd` on reader underflow.
  The table is a **complete** prefix code (Kraft equality
  `Σ 2^(11 − L) = 2048 = 2^11`), exhaustively verified by walking
  every 11-bit prefix at unit-test time and asserting each maps to
  exactly one entry — so the decoder's `unreachable!()` fall-through
  is verifiably dead. Encode-then-decode round-trips every index
  for both the direct API and the cross-check against the round-213
  `spectral_codebook::decode_index_to_tuple` /
  `encode_tuple_to_index` §4.6.3.3 translation. 15 unit tests in
  `src/spectrum_huffman.rs` (table-shape: 81 entries / max 11 bits
  / min 1 bit at index 40 / codewords fit declared length;
  Kraft equality sum = 2048; complete-prefix walk over all 2^11
  prefixes; per-spot-row PDF spot checks at indices 0 / 40 / 80;
  encode rejection; decode of `0` and of an 11-bit `0x7f8`; reader
  underflow; full round-trip across every index) plus 16 integration
  tests in `tests/spectrum_huffman.rs` (six per-row PDF spot checks
  at indices 0 / 31 / 40 / 54 / 67 / 80; Table 4.95 row 1 ↔ Table
  4.A.2 size cross-check; full writer→reader round-trip; the
  §4.6.3.3 wire-index ↔ tuple ↔ wire-index cross-check; three
  hand-pinned byte sequences (`[0x00]` for index 40 padded;
  `[0xff, 0x00]` for index 0 padded; `[0x7f, 0x80]` for
  index-40 + index-0 packed); exact bit-consumption invariant
  across every index; out-of-range and truncation rejections;
  `HCOD1_MAX_LEN` constant consistency). Suite size grows
  549 → 580 tests. Codebooks 2..=11 (Tables 4.A.3 … 4.A.12)
  reuse the same encode / decode shape and will land one per future
  round; the `spectral_data()` driver that dispatches per-band onto
  the chosen codebook arrives once codebooks 2..=11 are in place.
- phase 2 (r213): `spectral_codebook` module — ISO/IEC 14496-3
  §4.6.3.1 / Table 4.95 Spectrum Huffman codebook parameters plus
  the §4.6.3.3 codeword-index → spectral-tuple translation, the
  §4.6.3.3 sign-bit fix-up, and the §4.6.3.3 ESC sequence handling
  for codebook 11. Public types: `Table495Row` (the four normative
  columns of Table 4.95: `unsigned_cb`, `dimension`, `lav`,
  `esc_threshold`, plus the source `huffman_table` index), the
  `TABLE_4_95: [Table495Row; 32]` static (one row per codebook
  `0..=31`, covering ZERO_HCB, the four QUAD spectrum books, the
  six PAIR spectrum books, the ESC book, the four non-spectral
  reserved / PNS / intensity slots, and the sixteen extension
  books 16..=31 with their increasing ESC thresholds 15, 31, 47,
  63, 95, 127, 159, 191, 223, 255, 319, 383, 511, 767, 1023, 2047).
  Accessor: `table_4_95(codebook)` (rejects `> 31` with
  `Error::SpectralCodebookOutOfRange`). §4.6.3.3 translation:
  `decode_index_to_tuple(codebook, idx)` returns the
  `dim`-tuple of quantised spectral coefficients (the spec's
  pseudocode for `dim == 4`: `w = INT(idx/mod^3) - off`, etc.; for
  `dim == 2`: `y, z`). Inverse: `encode_tuple_to_index(codebook,
  tuple)`. Sign-bit handling: `apply_sign_bits(codebook, tuple,
  signs)` folds the per-non-zero-coefficient sign bits from the
  wire onto an unsigned-codebook tuple; `derive_sign_bits(codebook,
  tuple)` extracts them in low-frequency-first order. ESC sequence:
  `decode_esc_value(prefix_len, escape_word)` returns
  `2^(prefix_len + 4) + escape_word`; `encode_esc_value(value)`
  inverts it. Public constant `MAX_QUANT = 8191` (§4.6.1.3
  maximum absolute amplitude for `x_quant`). All accessors reject
  non-spectral codebooks (`0`, `12..=15`) with
  `Error::SpectralCodebookHasNoTuple` and out-of-range indices /
  tuples / sign-bit counts with dedicated error variants
  (`SpectralCodebookIndexOutOfRange`,
  `SpectralCodebookTupleOutOfRange`,
  `SpectralCodebookSignBitsMismatch`,
  `SpectralCodebookEscOutOfRange`). New `classify(sect_cb)`
  convenience re-export of the existing `section_data::Codebook`
  classifier. 46 new integration tests in
  `tests/spectral_codebook.rs` cover: the 32-row table layout
  (ZERO_HCB row, QUAD books 1..=4, PAIR books 5..=10, ESC book 11,
  the four non-spectral books 12..=15, the sixteen extension books
  16..=31 row-by-row); table accessor rejection (`> 31`) and
  success paths; full §4.6.3.3 translation for codebooks 1 (signed
  QUAD), 3 (unsigned QUAD), 5 (signed PAIR), 7 (unsigned PAIR), 11
  (ESC unsigned PAIR); rejection of `ZERO_HCB` / `12..=15`; the
  `idx >= mod^dim` out-of-range branch; full encoder-side
  round-trip across every (w, x, y, z) tuple in 3^4 = 81 cells for
  codebook 1 (signed) and codebook 3 (unsigned), and every (y, z)
  tuple in 9² and 8² cells for codebooks 5 and 7; LAV-cap
  rejection (signed and unsigned); short-tuple rejection;
  non-spectral codebook rejection; sign-bit application with
  all-non-zero and partial-zero coefficient patterns; signed-book
  no-op + non-empty-signs rejection; sign-bit length mismatch
  rejection; `derive_sign_bits` round-trip including signed-book
  empty-bits and all-zero-tuple empty-bits; ESC sequence
  round-trip for every prefix length `N ∈ 0..=8`; LAV-boundary
  (16, 17, 31, 32) and MAX_QUANT-boundary (8191 = 2^12 + 4095)
  ESC encoding; rejection of in-band magnitudes (`< 16`),
  overflow (`> 8191`), prefix-length overflow (`> 9`),
  escape-word overflow, and decoded-magnitude overflow; row helper
  accessors (`is_unsigned`, `has_esc`) row-by-row; and the
  cross-check `MAX_QUANT == TABLE_4_95[11].esc_threshold` plus the
  `no in-band LAV exceeds MAX_QUANT` invariant across every row.
  Suite grows 503 → 549 tests.
- phase 2 (r207): `ics_body` module — ISO/IEC 14496-3 §4.4.6 /
  Table 4.50 `individual_channel_stream()` body walker. Composes
  the existing per-tool parsers / writers (`global_gain`, `ics_info`,
  `section_data`, `scale_factor_data`, optional `pulse_data` /
  `tns_data` / `gain_control_data`) into a single channel-element
  body parse / write cycle, **up to but not including**
  `spectral_data()`. Exposes `IcsBody::parse` /
  `IcsBody::parse_with_ics_info` (SCE/LFE/CPE-shared-info dispatch)
  and `IcsBody::write` / `IcsBody::write_with_ics_info`. Surfaces
  the parser-derived `spectral_data_bit_offset` so the caller can
  hand off the trailing spectrum block to a future
  `spectral_data()` parser (or to a frame-assembler's
  `push_channel_body_bits`). Public constants
  `GLOBAL_GAIN_BITS = 8`, `AOT_AAC_SSR = 3`. Normative constraint
  enforcement on the writer side: Table 4.50 Note 1
  (pulse_data illegal on `EIGHT_SHORT_SEQUENCE`) and §4.6.12
  (gain_control_data is AOT-3-only) reject with
  `Error::PulseDataEncodeInvalid` /
  `Error::GainControlDataEncodeInvalid`; the parser surfaces
  literal bits to keep hostile streams from panicking.
  `scale_flag == true` (scalable AAC, AOT 6) rejects with
  `Error::NotImplemented` on both sides. 15 new integration tests
  cover the minimal AAC-LC long body, the one-active-band variant,
  the pulse_data / tns_data / gain_control_data dispatch branches,
  the all-tools-dispatched SSR stress, the
  `EIGHT_SHORT_SEQUENCE` pulse-data rejection, the populated-slot-
  without-dispatch-bit rejection, the AOT-3-only gain_control
  rejection, the CPE-shared-ics_info round-trip, the missing-
  inline-ics_info writer rejection, the `scale_flag == true`
  rejection on both sides, and the
  `spectral_data_bit_offset == bits_written` invariant. Suite
  grows 488 → 503 tests.
- phase 2 (r200): `tns_max` module — ISO/IEC 14496-3 §4.6.9.4 /
  Tables 4.102 (`TNS_MAX_ORDER`) and 4.103 (`TNS_MAX_BANDS`)
  decoder-side clamp tables, plus the §4.6.17.2.5 Tables 4.119 /
  4.120 LD-specific `TNS_MAX_BANDS` tables for the 480- and
  512-sample AAC LD frame variants. Exposed as the accessors
  `tns_max_order(aot, window_sequence, fs_index)`,
  `tns_max_bands(aot, window_sequence, fs_index)`,
  `tns_max_bands_ld_480(fs_index)`,
  `tns_max_bands_ld_512(fs_index)`, plus the three-way
  `min(band, TNS_MAX_BANDS, max_sfb)` and
  `min(order, TNS_MAX_ORDER)` clamp helpers `clamp_tns_band` and
  `clamp_tns_order` that fold the §4.6.9.3 pseudocode into one
  call. Public AOT constants `AOT_AAC_MAIN`, `AOT_AAC_LC`,
  `AOT_AAC_SSR`, `AOT_AAC_LTP`, `AOT_ER_AAC_LD` and the per-rate
  LD tables `TNS_MAX_BANDS_LD_480: [Option<u8>; 12]` /
  `TNS_MAX_BANDS_LD_512: [Option<u8>; 12]`. Table 4.102 dispatch
  partitions the long-window column at the > 32 kHz / ≤ 32 kHz
  threshold; Table 4.103 dispatch routes AOT 3 (AAC SSR) through
  the PQF-filterbank columns and every other AOT through the
  non-PQF columns. Out-of-range `fs_index` (`>= 13`) and the
  uncovered LD rates surface as
  `Error::IcsInfoUnsupportedSampleRateIndex`. 34 new tests
  (24 unit + 10 integration). Suite grows 454 → 488 tests.
- phase 2 (r194): `swb_offset` module — ISO/IEC 14496-3 §4.5.4.1 /
  Tables 4.129–4.141 `swb_offset_long_window[]` and
  `swb_offset_short_window[]` lookup tables for all 12 standard
  `samplingFrequencyIndex` values. Exposed as
  `SWB_OFFSET_LONG_WINDOW[13]` and `SWB_OFFSET_SHORT_WINDOW[13]`
  const slices (slot `12` empty — 7350 Hz has no defined SWB
  table); safe accessors `long_window_offsets(fs_index)` and
  `short_window_offsets(fs_index)`. Adds `LONG_WINDOW_LEN` /
  `SHORT_WINDOW_LEN` spectrum-length constants (1024 / 128).
- phase 2 (r194): `swb_offset::apply_pulse_data` — §4.6.13
  pulse-escape reconstruction loop, the crate's first
  spectrum-reconstruction-layer entry point. Consumes a parsed
  `pulse_data::PulseData` block plus a `&mut [i32]` long-window
  `x_quant` slice, folds the per-pulse `±pulse_amp` fix-up at the
  running coefficient index
  `k = swb_offset_long[fs][pulse_start_sfb] + Σ pulse_offset[i]`.
  Rejects malformed bitstreams (`pulse_start_sfb` past the last
  real band, `k` overrun past `LONG_WINDOW_LEN`, empty / >4 pulse
  list) via `Error::PulseDataEncodeInvalid`, unsupported
  `fs_index` via `Error::IcsInfoUnsupportedSampleRateIndex`. Suite
  size grows 409 → 454 tests (+45: 33 unit + 12 integration).

## [0.1.2](https://github.com/OxideAV/oxideav-aac/releases/tag/v0.1.2) - 2026-05-30

### Other

- phase 2 (r192): asc Table 1.15 trailing syncExtensionType == 0x2b7 implicit-SBR / PS probe
- phase 2 (r187): extension_payload() parser + encoder primitive — seventh tool-level encode-side syntax writer
- phase 2 (r183): gain_control_data() parser + encoder primitive — sixth tool-level encode-side syntax writer
- Table 4.1 extensionFlag body + Table 1.15 epConfig for ER AOTs
- phase 2 (r165): pce::Pce::write + FrameAssembler::push_pce
- phase 2 (r160): raw_data_block() frame assembler — encoder primitive for §4.4.2.1
- phase 2 (r152): §4.6.2.3.2 / §4.6.8.1.4 / §4.6.13 DPCM accumulator pair
- phase 2 (r149): scale_factor_data() parser + encoder primitive — fifth encode-side syntax-element writer
- phase 2 (r146): tns_data() parser + encoder primitive — fourth encode-side syntax-element writer
- phase 2 (r142): pulse_data() parser + encoder primitive — third encode-side syntax-element writer
- phase 2 (r140): ics_info() encoder primitive — second encode-side syntax-element writer
- phase 2 (r137): section_data() encoder primitive — first encode-side syntax-element writer
- phase 2 (r133): section_data() parser (Table 17)
- phase 2 begin (r129): ics_info() parser (Table 4.6)
- phase 1 (r126): AudioSpecificConfig + program_config_element parsers
- tighten provenance attestation wording
- phase 1: ADTS header parser + raw_data_block syntactic walker
- drop fuzz workflow (no fuzz targets in scaffold)
- orphan rebuild — clean-room reset 2026-05-24

### Added (round 192 — Table 1.15 trailing syncExtensionType == 0x2b7 implicit-SBR probe)

- New `AudioSpecificConfig` field `trailing_sbr_probe:
  Option<SbrExtensionProbe>` carrying the result of the §1.6.5 /
  Table 1.15 trailing-bits backward-compatible implicit-SBR /
  PS / ER BSAC-extension probe.
- New entry point `AudioSpecificConfig::parse_bits_bounded(reader,
  origin_bit_offset, asc_bit_length)` for carriers that know the
  ASC bit length (LATM `StreamMuxConfig`, esds AudioObj
  descriptor). The existing `AudioSpecificConfig::parse(&[u8])`
  now computes the byte-slice bound and calls into the new entry
  point automatically; `AudioSpecificConfig::parse_bits` keeps
  its no-probe semantics so callers holding a `BitReader`
  carrying trailing carrier bytes are not surprised by a stray
  11-bit `0x2b7` match.
- Probe field-width / marker constants exposed as public
  module-level constants: `SYNC_EXTENSION_TYPE_BITS = 11`,
  `SYNC_EXTENSION_TYPE_SBR = 0x2b7`, `SYNC_EXTENSION_TYPE_PS =
  0x548`, `TRAILING_EXTENSION_AOT_SBR = 5`,
  `TRAILING_EXTENSION_AOT_BSAC = 22`.
- Probe semantics — driven by the spec's two guards:
  - Outer guard: `extensionAudioObjectType != 5` (skip when the
    outer `audioObjectType` is the explicit hierarchical SBR
    (5) or PS (29) wrapper, which already established
    `extensionAudioObjectType == 5`) **and** `bits_to_decode()
    >= 16` (the ASC carrier must have at least 11 + 5 bits
    remaining to attempt the 11-bit `syncExtensionType` plus a
    minimum-width nested `GetAudioObjectType()`).
  - On a `0x2b7` match the parser dispatches on the resolved
    extension AOT:
    - `5` → `sbrPresentFlag` + (when set) the 4-bit
      `extensionSamplingFrequencyIndex` (with the 24-bit
      `extensionSamplingFrequency` escape on the `0xf`
      sentinel) + (when at least 12 further bits remain) a
      second 11-bit `syncExtensionType == 0x548` gating a
      1-bit `psPresentFlag`.
    - `22` → `sbrPresentFlag` + (when set) the 4-bit
      `extensionSamplingFrequencyIndex` (with the same 24-bit
      escape) + mandatory 4-bit
      `extensionChannelConfiguration`.
    - any other value → `Error::UnsupportedTrailingExtensionAot`
      (Table 1.15 spells out no body layout for those).
- Propagation: when the probe records `sbr_present_flag == true`
  the resolved extension sample rate is also promoted to
  `AudioSpecificConfig::extension_sampling_frequency_index` /
  `AudioSpecificConfig::extension_sample_rate` and
  `AudioSpecificConfig::sbr_present` is set; when the probe
  records `ps_present_flag == Some(true)`
  `AudioSpecificConfig::ps_present` is set; when the probe
  resolves the BSAC branch the
  `AudioSpecificConfig::extension_channel_configuration` field
  is set. This mirrors the explicit-SBR / PS wrapper path so
  call sites can read SBR / PS state from a single top-level
  field set whether the signalling was hierarchical-explicit or
  trailing-implicit.
- New `Error::UnsupportedTrailingExtensionAot(u8)` variant for
  resolved extension AOTs whose Table 1.15 body is not defined
  (anything outside `{5, 22}`).
- 20 new integration tests in `tests/asc.rs` (`suite size 389 →
  409`) covering:
  - Field-width / marker constant pin.
  - Four "probe-skipped" branches: no trailing bits at all,
    fewer than 16 bits remaining (one byte of pad), 11-bit
    non-`0x2b7` value, outer SBR wrapper (AOT 5) and outer PS
    wrapper (AOT 29) guard skipping the probe even with
    trailing bits that *would* match.
  - SBR-branch round-trips: `sbrPresentFlag = 0`,
    `sbrPresentFlag = 1` with a Table 1.18 index, `sfi = 0xf`
    with the 24-bit escape, the PS sub-probe with
    `psPresentFlag = 1`, the PS marker mismatch (non-0x548
    inner sync — `ps_present_flag` stays `None`), and the
    inner 12-bit guard skipping the PS sub-probe with only 8
    bits of slack.
  - ER BSAC-branch round-trips: `sbrPresentFlag = 1` with a
    rate + `extensionChannelConfiguration = 2`,
    `sbrPresentFlag = 0` with `extensionChannelConfiguration = 3`.
  - `Error::UnsupportedTrailingExtensionAot(3)` reject for an
    `extensionAudioObjectType` outside `{5, 22}`.
  - `parse_bits` no-probe contract — the bit-level entry point
    stops at end-of-body and leaves trailing bits alone.
  - `parse_bits_bounded` explicit-`asc_bit_length` call shape —
    one round-trip plus a truncated-bound assertion that the
    probe does not over-read when the bound stops at the body.
  - Mid-escape truncation surfaces `Error::UnexpectedEnd` (24-bit
    explicit-rate escape cut to 16 bits).
  - Hand-pinned 7-byte HE-AAC v2 implicit chain (AOT 2 / 16 kHz
    / mono + 0x2b7 + ext_aot=5 + sbr=1 + sfi=5 + 0x548 + ps=1).

### Added (round 187 — extension_payload() parser + encoder primitive)

- New `extension_payload` module implementing the ISO/IEC 14496-3
  §4.4.2.7 / Table 4.51 `extension_payload()` body that rides inside
  every FIL element, plus the Table 4.52 `dynamic_range_info()` and
  Table 4.53 `excluded_channels()` sub-bodies:
  - `ExtensionType` enum with the five well-known values (`Fill`,
    `FillData`, `DynamicRange` per Table 4.59;`SbrData`,
    `SbrDataCrc` per ISO/IEC 13818-7 Table 40) and the
    `from_bits` / `as_u8` round-trip pair.
  - `ExtensionPayload::Fill { cnt, other_bits }` — Table 4.51
    default branch carrying `8 * (cnt - 1) + 4` `other_bits`
    packed MSB-first.
  - `ExtensionPayload::FillData { cnt }` — Table 4.51 normative
    pattern: 4-bit `fill_nibble == 0b0000` + (cnt - 1) × 8-bit
    `fill_byte == 0b1010_0101`.
  - `ExtensionPayload::DynamicRange(DynamicRangeInfo)` — Table 4.52
    DRC body carrier with optional `PceTagFields`, optional
    `ExcludedChannels` (Table 4.53), optional `DrcBands`
    partitioning, optional `ProgRefLevelFields`, and the per-band
    `(dyn_rng_sgn, dyn_rng_ctl)` `DrcBandRecord` list whose length
    matches the resolved `drc_num_bands`.
- `ExtensionPayload::parse(reader, cnt)` walks the Table 4.51
  dispatch (4-bit `extension_type`, then per-type body) and
  surfaces the SBR extension types as
  `Error::UnsupportedExtensionSbr` (since this crate does not
  carry the QMF / patching back-end), reserved values as
  `Error::UnsupportedExtensionType`, and structural / normative-
  value violations as `Error::ExtensionPayloadInvalid`.
- `ExtensionPayload::write(writer)` is the bit-exact inverse;
  returns Table 4.51's `n` byte count and surfaces caller-side
  field overflows / shape mismatches as
  `Error::ExtensionPayloadInvalid`.
- `ExtensionPayload::byte_length` / `DynamicRangeInfo::byte_length` /
  `DynamicRangeInfo::num_bands` accessors for callers that need the
  Table 4.51 returned `n` without round-tripping through `write`.
- Public helper `excluded_group_count(exclude_mask_len)` exposing
  Table 4.53's 7-bit-group → 8-bit-byte mapping.
- Public field-width constants: `EXTENSION_TYPE_BITS = 4`,
  `FILL_DATA_NIBBLE = 0b0000`, `FILL_DATA_BYTE = 0b1010_0101`.
- Three new `Error` variants: `UnsupportedExtensionSbr(u8)` for
  the SBR extension types, `UnsupportedExtensionType(u8)` for
  reserved 4-bit values, and `ExtensionPayloadInvalid` for
  caller-side wire-field violations and parser-side normative-
  value / structural mismatches.
- 36 integration tests in `tests/extension_payload.rs` covering
  Table 4.59 / Table 40 dispatch (known values, round-trip, SBR
  rejection, reserved rejection), the EXT_FILL `cnt == 1`
  collapsed body, an EXT_FILL `cnt == 3` round-trip with a
  partial trailing byte, the EXT_FILL_DATA cases with hand-
  pinned byte sequences and the two normative rejections (non-
  zero nibble, non-`0xA5` byte), the DRC minimal body (hand-
  pinned bytes `[0xB0, 0x42]`), single-optional round-trips for
  pce_tag / excluded_channels / drc_bands / prog_ref_level, an
  every-optional-present round-trip (`n == 9`), the
  continuation-bit positioning across a two-group exclude_mask
  (hand-pinned bytes `[0xB6, 0x04, 0x08, 0x00]`), six encoder-
  side overflow rejections, a parser-side `cnt`-vs-`n` mismatch
  detection, three truncation tests, an `excluded_group_count`
  lookup-table check, the `byte_length` / `num_bands` accessor
  checks, and a trailing-bit non-consumption assertion. Suite
  size grows 353 → 389.

### Added (round 183 — gain_control_data() parser + encoder primitive)

- New `gain_control_data` module implementing the ISO/IEC 14496-3
  §4.4.6.5 / Table 4.12 `gain_control_data()` syntax element used by
  the SSR (Scalable Sample Rate, AOT 3) gain-control tool:
  - `GainControlData { max_band: u8, bands: Vec<GainBand> }` —
    parsed Table 4.12 block, with `bands.len() == max_band` mapping
    the per-spec `bd ∈ 1..=max_band` band index onto `bands[bd - 1]`.
  - `GainBand { windows: Vec<GainWindow> }` — per-`bd` collection
    whose length matches the per-`window_sequence` window count
    (1 for `OnlyLong`, 2 for `LongStart` / `LongStop`, 8 for
    `EightShort`).
  - `GainWindow { adjustments: Vec<GainAdjust> }` — per-`(bd, wd)`
    ladder; `adjustments.len()` is the wire `adjust_num` value
    (`0..=7`).
  - `GainAdjust { alevcode: u8, aloccode: u8 }` — single ladder
    entry with the 4-bit `alevcode` and the per-slot-width
    `aloccode`.
- `GainControlData::parse(reader, window_sequence)` walks the
  Table 4.12 syntax: 2-bit `max_band`, then for each `bd` in
  `1..=max_band` and each `wd` in `0..N(window_sequence)` a 3-bit
  `adjust_num` followed by `adjust_num` `(4-bit alevcode,
  W(seq, wd)-bit aloccode)` records.
- `GainControlData::write(writer, window_sequence)` is the bit-exact
  inverse; caller-side field violations surface as
  `Error::GainControlDataEncodeInvalid`.
- Public helpers `num_windows(window_sequence)` and
  `aloccode_bits(window_sequence, wd)` pin Table 4.12's per-`seq`
  loop count and the per-`(seq, wd)` `aloccode` width (5 / 4-2 /
  2-2 / 4-5 across `OnlyLong` / `LongStart` / `EightShort` /
  `LongStop`).
- Public field-width constants `MAX_BAND_BITS = 2`,
  `ADJUST_NUM_BITS = 3`, `ALEVCODE_BITS = 4` and the cap constants
  `MAX_BAND_CAP = 0x03`, `MAX_ADJUST_NUM = 0x07`,
  `MAX_ALEVCODE = 0x0f`.
- New `Error::GainControlDataEncodeInvalid` variant for caller-side
  field-overflow / shape-mismatch on the encoder.
- 20 integration tests in `tests/gain_control_data.rs` covering the
  per-`window_sequence` `num_windows` and `aloccode_bits` lookup
  tables, the `max_band == 0` collapsed body byte layout, a hand-
  pinned `OnlyLong` wire-bit assertion, self-roundtrip across the
  four window sequences (including a max-everything `EightShort`
  with `max_band = 3` and every per-slot adjustment maxed out), six
  encoder-side overflow rejections (max-band, band-count mismatch,
  window-count mismatch, adjust-num overflow, alevcode overflow,
  aloccode overflow at the 5-bit / 2-bit slots), an
  `UnexpectedEnd` mid-record truncation test, and a trailing-bit
  non-consumption check confirming the parser does not over-read
  past the Table 4.12 body. Suite size grows 333 → 353.

### Added (round 177 — GASpecificConfig extensionFlag body + ER-AOT epConfig)

- `asc::GaSpecificConfig::extension_body: Option<GaExtensionBody>` —
  parsed body of the `if (extensionFlag)` branch of `GASpecificConfig`
  per ISO/IEC 14496-3 §4.4.1 / Table 4.1. Populated when the
  `extensionFlag` bit (which precedes the layerNr in Table 4.1) was
  `1` on the wire.
- `asc::GaExtensionBody { bsac_layer, resilience, extension_flag3 }`
  carrier struct:
  - `bsac_layer: Option<BsacLayerSpec { num_of_sub_frame: u8 (5 bit),
    layer_length: u16 (11 bit) }>` — emitted only when the
    surrounding `audioObjectType == 22` (ER BSAC).
  - `resilience: Option<AacResilienceFlags { section_data,
    scalefactor_data, spectral_data }>` — emitted only when the
    surrounding `audioObjectType ∈ {17, 19, 20, 23}` (ER AAC LC /
    ER AAC LTP / ER AAC scalable / ER AAC LD). Stores the three
    `aac*ResilienceFlag` bits used by the §4.4.6 RVLC scalefactor
    branch + the §4.4.6 HCR spectral branch in downstream rounds.
  - `extension_flag3: bool` — always present at the tail of the
    extension body. ISO/IEC 14496-3:2009 reserves the body behind
    this flag with the comment "tbd in version 3"; Phase 1
    surfaces the bit but rejects the body itself with the new
    `Error::UnsupportedAscExtensionFlag3` when the flag is `1`.
- `asc::AudioSpecificConfig::ep_config: Option<u8>` — Table 1.15
  trailing 2-bit `epConfig` field emitted for AOTs ∈
  {17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 39}. Phase 1 supports
  `epConfig ∈ {0, 1}` (no inline `ErrorProtectionSpecificConfig()`
  body); `epConfig == 2 || 3` (which mandate `EPSpecificConfig()`
  parsing) surface as the new `Error::UnsupportedEpConfig(u8)`.
- New `Error::UnsupportedEpConfig(u8)` variant carrying the literal
  2-bit `epConfig` value as read from the wire.
- New `Error::UnsupportedAscExtensionFlag3` variant.
- 13 new integration tests in `tests/asc.rs` covering: AAC-LC
  regression (extension body / ep_config both `None`); ER AAC LC
  with the full resilience triplet + epConfig 0; ER AAC LC with
  epConfig 1 accepted; ER AAC LC rejection at epConfig 2 and 3;
  ER BSAC (AOT 22) `numOfSubFrame` + `layer_length` pair with an
  inline PCE; ER AAC LD (AOT 23) resilience triplet only; ER AAC
  scalable (AOT 20) covering BOTH the layerNr branch AND the
  resilience triplet in the correct Table 4.1 order; ER TwinVQ
  (AOT 21) where the extension body collapses to `extensionFlag3`
  + `epConfig` only; `extensionFlag3 == 1` rejection; truncation
  inside the resilience body and at the `epConfig` field; and a
  hand-pinned bit-position assertion (a 22-bit ER AAC LC ASC).
  Suite size grows 320 → 333 tests.
- Module-level documentation in `src/asc.rs` rewritten to reflect
  the new coverage and to remove the round-126 "What is not parsed
  yet" entries that this round closes (`epConfig` and the
  `extensionFlag` body); the `syncExtensionType == 0x2b7` implicit-
  SBR probe and the FIL-extension-payload implicit path remain
  listed as deferred.

### Added (round 165 — `Pce::write` + `FrameAssembler::push_pce`)

- `pce::Pce::write(writer, origin_bit_offset)` — encoder primitive for
  the `program_config_element()` per ISO/IEC 14496-3 §4.4.1.1 /
  Table 4.2, the bit-exact inverse of the round-126 `Pce::parse`. Emits
  the full PCE wire layout: 4-bit `element_instance_tag` + 2-bit
  `object_type` + 4-bit `sampling_frequency_index`; element counts
  (4-bit `num_front`, 4-bit `num_side`, 4-bit `num_back`, 2-bit
  `num_lfe`, 3-bit `num_assoc`, 4-bit `num_valid_cc`); mix-down
  presence flags + bodies (`mono_mixdown_present` + 4-bit
  `mono_mixdown_element_number`; `stereo_mixdown_present` + 4-bit
  `stereo_mixdown_element_number`; `matrix_mixdown_idx_present` +
  2-bit `matrix_mixdown_idx` + 1-bit `pseudo_surround_enable`); the
  per-element select lists (`front` / `side` / `back` are
  `(1-bit is_cpe, 4-bit tag_select)` pairs; `lfe` and `assoc` are
  bare 4-bit `tag_select` values; `valid_cc` is
  `(1-bit is_ind_sw, 4-bit tag_select)` pairs); a §4.4.1.1 Note 1
  origin-relative `byte_alignment()`; then the 8-bit
  `comment_field_bytes` length prefix + `comment_field` bytes. The
  origin handling mirrors the parser exactly — pass `0` for a
  standalone PCE inside a `raw_data_block()` (collapses to absolute
  alignment), or the ASC origin bit-position for a PCE inline in
  `AudioSpecificConfig` (the ASC-relative form). Self-roundtrip
  (`write` → `parse`) is bit-perfect across the empty layout, a 5.1
  layout with mixed SCE / CPE / LFE / CC and a comment field, every
  mix-down body permutation, the maximum-fields ladder (`tag=0x0f`,
  `object_type=3`, `sfi=0x0f`, 255-byte comment), and a non-zero-
  origin ASC-relative roundtrip.
- `raw_data_block::FrameAssembler::push_pce(&Pce)` — emits the 3-bit
  PCE `id_syn_ele` (`0b101`) followed by the full PCE body via
  `Pce::write` with `origin_bit_offset = 0`. Pairs with the round-121
  walker's `Element::ProgramConfig` branch; round-trips through the
  walker bit-exactly when placed before END, after a channel header,
  next to a FIL element, or with every mix-down + CC slot populated.
  Propagates `Error::PceEncodeInvalid` from `Pce::write` when any
  wire field overflows its bit-width. The round-160 `push_channel_header`
  still rejects `IdSynEle::Pce` — PCE has its own bespoke wire shape
  (with the §4.4.1.1 Note 1 origin alignment) and routes through this
  dedicated entry point instead.
- New `Error::PceEncodeInvalid` variant — surfaces caller-side
  structural bugs that cannot be represented on the Table 4.2 wire
  (any of the field-width caps listed in the variant's doc-comment).
- 26 new integration tests (`tests/pce.rs` +17,
  `tests/raw_data_block_encode.rs` +9) — self-roundtrip + every
  encoder-rejection branch + a non-zero-origin parser / writer
  agreement test + a hand-pinned wire-layout assertion that the
  all-zero PCE produces 6 all-zero bytes (header + counts + flags +
  6-bit pad + comment length). Crate suite size grows 294 → 320.

### Added (round 160 — §4.4.2.1 raw_data_block frame assembler)

- `raw_data_block::FrameAssembler` — encoder-side composition primitive
  for `raw_data_block()` per ISO/IEC 14496-3 §4.4.2.1, the bit-exact
  inverse of the round-121 `raw_data_block::Walker`. Wraps an internal
  [`oxideav_core::bits::BitWriter`] and exposes a small typed push-API:
  - `push_channel_header(kind, element_instance_tag)` emits the 3-bit
    `id_syn_ele` (`SCE` / `CPE` / `CCE` / `LFE`) + 4-bit
    `element_instance_tag`. The channel-element *body* (`ics_info` →
    `section_data` → `scale_factor_data` → optional `pulse_data` /
    `tns_data` → `spectral_data`) is the caller's responsibility — it
    is appended via the next entry point.
  - `push_channel_body_bits(bits, bit_count)` appends a pre-serialised
    channel-element body as an MSB-first packed bit-slice. This lets
    callers run the existing typed writers (`IcsInfo::write`,
    `SectionData::write`, `ScaleFactorData::write`, `PulseData::write`,
    `TnsData::write`) into an auxiliary [`BitWriter`] and splice the
    resulting bits into the frame without forcing the assembler to own
    every per-tool body layout.
  - `push_fill(payload)` emits a FIL element per §4.4.2.7 — 3-bit id +
    4-bit `count` + optional 8-bit `esc_count` (when `payload.len() >=
    15`) + payload bytes. The escape arithmetic inverts the parser's
    `cnt = esc_count + 15 - 1` to `esc_count = payload_bytes - 14`,
    capping single-element fill at 269 bytes.
  - `push_data(tag, byte_align_flag, payload)` emits a DSE element per
    §4.4.2.5 — 3-bit id + 4-bit `element_instance_tag` + 1-bit
    `data_byte_align_flag` + 8-bit `count` + optional 8-bit `esc_count`
    (when `payload.len() >= 255`) + optional byte-align (before payload
    bytes) + payload bytes. The escape arithmetic inverts the parser's
    `cnt = count + esc_count` to `esc_count = payload_bytes - 255`,
    capping single-element data at 510 bytes.
  - `push_end()` consumes the assembler, emits the 3-bit `END` (`0b111`)
    terminator, byte-aligns the writer per §4.4.2.1, and returns the
    finished `Vec<u8>`. END is mandatory (the type-state forces every
    completed frame to call it) and post-END pushes are a compile-time
    error.
- `Error::RawDataBlockEncodeInvalid` for caller-side structural bugs
  the assembler cannot represent on the wire: non-channel `IdSynEle`
  passed to `push_channel_header` (FIL / DSE / END have their own
  bespoke entry points; PCE has no writer yet); `element_instance_tag
  > 0x0f` on a channel or DSE element; FIL `payload.len() > 269`; DSE
  `payload.len() > 510`; or `push_channel_body_bits` called with
  `bit_count > bits.len() * 8`. Long fill / data payloads above the
  per-element ceilings split naturally across multiple back-to-back
  FIL / DSE elements with the same `tag`; the assembler intentionally
  enforces the single-element ceiling rather than splitting silently.
- PCE encoding is **not** in this round — the [`crate::pce::Pce`]
  struct is fully parsed (round 126) but has no `write` primitive yet,
  and adding one is a separate round's worth of work (Tables 4.4 / 4.5
  front/side/back/lfe element selects, mono / stereo / matrix mix-down
  hints, comment field, plus the relative-origin `byte_alignment()`
  per Table 4.2 Note 1). The assembler rejects PCE in
  `push_channel_header` rather than admitting an `Element::ProgramConfig`
  variant the writer cannot honour.
- 30 new integration tests in `tests/raw_data_block_encode.rs`:
  END-only frame (one-byte `0xE0` wire-layout pin); channel-header
  push for every `(SCE, CPE, CCE, LFE)` × `(tag = 0, 7, 0x0a, 0x0f)`
  combination with a hand-pinned wire-byte assertion for the
  SCE(tag=0x0a)+END case (`[0x15, 0xC0]`); rejection of non-channel
  kinds (`Dse`, `Pce`, `Fil`, `End`) and over-4-bit tags; channel-body
  bit-append for partial-byte (12-bit), whole-byte (32-bit), and
  zero-count cases with a hand-pinned three-byte wire-layout assertion
  (`[0x01, 0x57, 0x9C]` for SCE(tag=0)+0xABC(12-bit)+END); FIL empty
  pin (`[0xC1, 0xC0]`), every `payload_bytes ∈ 0..15`, escape-boundary
  pair (15 / 16), max-payload-269 roundtrip, and rejection above 269;
  DSE short / byte-align-set / byte-align-with-mid-byte-offset /
  escape-boundary-255 / max-510 roundtrips and rejection above 510 /
  over-4-bit tag; composite frames (SCE+FIL+END mirroring the parser
  test, CPE+13-bit-body+DSE+FIL+END, two back-to-back FIL elements);
  `bit_position` tracking; and END byte-alignment after an unaligned
  body. Suite size grows 264 → 294 tests.

### Added (round 152 — §4.6.2.3.2 / §4.6.8.1.4 / §4.6.13 DPCM accumulator pair)

- `scale_factor_data::accumulate(sfd, sfb_cb, global_gain) ->
  Result<AbsoluteScaleFactors>` — the symmetric decoder-side
  accumulator that converts the transmitted DPCM record set
  produced by `ScaleFactorData::parse` into absolute per-band
  quantities. Runs three independent DPCM tracks: spectrum
  scalefactors (seed `last_sf = global_gain`, range `0..=255` per
  the §4.6.2.3.2 Note), intensity stereo positions (seed
  `last_is = 0` per §4.6.8.1.4), and PNS noise energies (seed
  `last_nrg = global_gain - NOISE_OFFSET - 256` per §4.6.13, with
  the first PNS band of the frame's 9-bit `uimsbf` literal added
  directly and every subsequent PNS band's Huffman delta in
  `-60..=+60`). Returns `Error::ScaleFactorAccumulatorInvalid` on
  shape mismatch with `sfb_cb`, variant ↔ codebook mismatch, second
  `NoisePcm` after the frame-scope flag cleared, or `Sf`-track
  running value escaping `0..=255`.
- `scale_factor_data::differentiate(abs, sfb_cb, global_gain) ->
  Result<ScaleFactorData>` — the symmetric encoder-side inverse.
  Takes absolute per-band records (the output of rate-allocation
  / scalefactor quantisation / PNS noise-energy estimation) and
  produces the DPCM record set the bit-exact `ScaleFactorData::write`
  consumes. Validates every spectrum / intensity / PNS-subsequent
  delta against Table 4.150's `-60..=+60`, the first PNS band's
  initial seed magnitude against the 9-bit `uimsbf` Table 4.53
  field (`0..=511`), variant ↔ codebook agreement, and outer / inner
  shape against `sfb_cb`. Roundtrip: `accumulate(differentiate(abs,
  sfb_cb, gg)?, sfb_cb, gg)? == abs` on every well-formed input.
- `scale_factor_data::AbsoluteScaleFactorEntry { Sf(u8) /
  IsPos(i16) / NoiseNrg(i32) }` — the per-band absolute record
  variant. `Sf` for spectrum-band scalefactors (`0..=255`); `IsPos`
  for intensity stereo positions (signed, the accumulator allows
  in-principle unbounded range, conforming streams stay within an
  8-bit window); `NoiseNrg` for PNS noise energies (signed `i32`
  because the seed `global_gain - 346` can be negative).
- `scale_factor_data::AbsoluteScaleFactors` — the per-window-group
  container, matching the outer / inner shape of `ScaleFactorData`.
- `scale_factor_data::NOISE_OFFSET = 90` — §4.6.13 constant that
  positions the PNS-track running register relative to
  `global_gain`. Made `pub` so downstream rate-allocation callers
  can derive the seed without hard-coding the constant.
- `Error::ScaleFactorAccumulatorInvalid` for caller-side bugs the
  accumulator / differentiator cannot represent: outer length
  mismatch with `sfb_cb`; per-group entry count mismatch with the
  non-`ZERO_HCB` band count of the matching `sfb_cb` group;
  variant ↔ codebook mismatch; spectrum-track running value
  escaping `0..=255`; spectrum / intensity / PNS-subsequent delta
  outside `-60..=+60`; first PNS band's initial seed magnitude
  outside the 9-bit `uimsbf` field (`0..=511`); second `NoisePcm`
  in the same frame (`noise_pcm_flag` already cleared);
  `NoiseDpcm` on the first PNS band of the frame.
- Module-level documentation calls out the spec ambiguity between
  the §4.6.2.3.2 illustrative pseudocode (single-track, lumps PNS
  into `last_sf`) and the §4.6.8.1.4 + §4.6.13 prose-level "done
  separately" wording. This crate honours the three-track prose
  interpretation; the rationale is that the §4.6.2.3.2 pseudocode
  is identical in 13818-7 §11.3.2 where PNS does not exist, so the
  MPEG-4 prose §4.6.13 supersedes for PNS bands.
- 37 new integration tests in `tests/scale_factor_accumulator.rs`
  covering: empty / all-`ZERO_HCB` no-op paths; spectrum-only
  ascending sequences; spectrum-track boundary deltas (±60) at
  both ends of the 0..=255 range; intensity-track zero-seed
  independence of `global_gain`; intensity-track boundary deltas;
  PNS first-band PCM seed including 9-bit boundaries (delta 0 and
  511); PNS subsequent-band Huffman delta; frame-scope
  `noise_pcm_flag` invariant across window groups; three-track
  independence under interleaved bands; `ZERO_HCB` skip behaviour;
  cross-window-group accumulator persistence; two hand-pinned
  §4.6.2.3.2 / §4.6.13 spec-pseudocode lockstep tests; every
  `ScaleFactorAccumulatorInvalid` rejection branch (spectrum /
  intensity / PNS-first-seed / PNS-subsequent overflows,
  underflows, variant ↔ codebook mismatch, outer / inner length
  mismatch, decoder-side sf-track 0..=255 overflow / underflow,
  decoder-side PCM-after-flag-clear and Huffman-before-flag-clear);
  a composite EIGHT_SHORT-shape frame with all three tracks
  interleaved across 8 window groups; and a `NOISE_OFFSET` constant
  pin. Suite size grows 227 → 264 tests.

### Added (round 149 — fifth encoder primitive: scale_factor_data parser + writer)

- `scale_factor_data` module: ISO/IEC 14496-3 §4.4.6 / Table 4.53
  (non-resilient branch) plus §4.6.3 / Table 4.A.1 *scalefactor
  Huffman codebook* (codebook 12) — the AAC crate's fifth encode-
  side syntax-element writer. The 121-entry Table 4.A.1 is
  transcribed verbatim from the spec (max length 19 bits, codeword
  for index 60 / delta 0 is the single bit `0`) and is a *complete*
  prefix code (Kraft equality, exhaustively verified by walking all
  `2^19` 19-bit prefixes).
- Public helpers `hcod_sf_encode(dpcm: i8) -> Result<(u8, u32)>`
  and `hcod_sf_decode(reader: &mut BitReader) -> Result<i8>` expose
  the Table 4.A.1 codebook directly for Auditor harnesses and
  fixture cross-checks.
- Public constants `SF_INDEX_OFFSET = -60` (Table 4.150 DPCM range
  centre), `NOISE_PCM_BITS = 9` (Table 4.53 first-PNS seed width),
  `HCOD_SF_NUM_ENTRIES = 121`, `HCOD_SF_MAX_LEN = 19` pin the
  §4.6.3 / Table 4.150 dispatch.
- `ScaleFactorData::parse(reader, sfb_cb)` walks the per-`(g, sfb)`
  non-`ZERO_HCB` subsequence driven by
  `SectionData::sfb_cb`, dispatching between `hcod_sf[]` (ordinary
  spectrum / PNS-after-first / both intensity codebooks 14 and 15)
  and the 9-bit `dpcm_noise_nrg` PCM seed (first PNS band of the
  frame). `noise_pcm_flag` is *frame*-scoped, not group-scoped:
  the first PNS band consumes the seed regardless of which window
  group it lives in.
- `ScaleFactorData::write(writer, sfb_cb)` is the bit-exact inverse.
  Validates per-band variant ↔ codebook classification, DPCM range
  `-60..=+60`, `NoisePcm` 9-bit cap, and the `noise_pcm_flag`
  consumption rule.
- `Error::ScaleFactorDataEncodeInvalid` for caller-side structural
  bugs the writer cannot represent on the wire: `entries.len()`
  differs from `sfb_cb.len()`; a group's entry count does not match
  the non-`ZERO_HCB` band count of its `sfb_cb` group; a variant
  paired with the wrong codebook (Intensity on a spectrum band,
  NoisePcm on a non-PNS band, NoiseDpcm on the first PNS band,
  Dpcm on an intensity / PNS band); a second NoisePcm in the same
  frame (`noise_pcm_flag` already cleared); a DPCM delta outside
  `-60..=+60`; a NoisePcm magnitude exceeding the 9-bit field cap
  (`> 0x1FF`).
- The §4.6.2.3.2 `last_sf = global_gain` accumulator that converts
  the transmitted DPCM deltas to absolute `sf[g][sfb] ∈ 0..=255`
  is **not** performed — it needs `global_gain` and is a per-AOT
  decoder step the back-end will own (with the symmetric `dpcm
  = sf[g][sfb] - last_sf` step on the encoder rate-allocation
  side).
- The §4.4.6 error-resilient branch (`aacScalefactorDataResilienceFlag
  == 1` → RVLC with `rev_global_gain`, `sf_concealment`,
  `length_of_rvlc_sf`, `length_of_rvlc_escapes`, etc.) is **not**
  implemented; ER AAC-LD / scalable profiles will need a sibling
  `scale_factor_data_rvlc()` module.
- 23 new integration tests in `tests/scale_factor_data.rs`
  covering: empty / all-`ZERO_HCB` no-op paths; single-band
  spectrum / intensity / PNS-seed / PNS-Huffman cases; frame-scope
  `noise_pcm_flag` invariant across window groups; exhaustive
  per-DPCM-value bit-length agreement against Table 4.A.1; a
  mixed-content frame with all four variants and ZERO_HCB skips;
  EIGHT_SHORT-style 8-group iteration; hand-pinned wire-byte
  assertions (single-bit `0x00`, `dpcm_noise_nrg = 0x1AB` →
  `[0xD5, 0x80]` MSB-first); every writer-rejection branch
  (outer length / extra / missing entries, variant ↔ band
  mismatch, second NoisePcm, NoiseDpcm-first-PNS, DPCM out-of-
  range, NoisePcm overflow); parse underflow on the PCM seed; and
  the exhaustive `hcod_sf_decode` completeness test (every 19-bit
  prefix decodes successfully — Kraft equality verification).
- 5 new module-level unit tests covering: known Table 4.A.1 rows
  (`HCOD_SF[60] = (1, 0)`, etc.); prefix-free transcription
  regression guard; `dpcm = 0` collapsing to a single bit;
  encode-boundary rejection at `-61` / `+61`; and the full
  encode→decode round-trip for every legal DPCM value `-60..=+60`.

### Added (round 146 — fourth encoder primitive: tns_data parser + writer)

- `tns_data` module: ISO/IEC 14496-3 §4.4.6 / Table 4.54 `tns_data()`
  parser **and** encoder primitive — the AAC crate's fourth encode-
  side syntax-element writer. `TnsData::parse(reader, window_sequence)`
  walks every transform window (`num_windows` = 8 for
  `EIGHT_SHORT_SEQUENCE`, 1 otherwise) and reads `n_filt[w]` with the
  §4.6.9.2 Table 4.155 size switch (1 bit short / 2 bits long), an
  optional `coef_res[w]` (when `n_filt[w] > 0`), then per-filter
  `length[w][filt]` (4 / 6 bits), `order[w][filt]` (3 / 5 bits), and —
  when `order > 0` — `direction`, `coef_compress`, and `order` ×
  `coef[i]` unsigned magnitudes whose width is
  `coef_bits = (3 + coef_res) − coef_compress` per §4.6.9.3.
  `TnsData::write(writer, window_sequence)` is the bit-exact inverse.
  Public helpers `field_widths(window_sequence)`,
  `num_windows(window_sequence)`, and `coef_bits(coef_res,
  coef_compress)` expose the Table 4.155 / §4.5.2.3.4 / §4.6.9.3
  selection rules; public constants `N_FILT_BITS_{SHORT,LONG}`,
  `LENGTH_BITS_{SHORT,LONG}`, `ORDER_BITS_{SHORT,LONG}`,
  `COEF_RES_BITS`, `DIRECTION_BITS`, `COEF_COMPRESS_BITS` pin the
  per-table widths. The §4.6.9.3 `tns_decode_coef` LPC reconstruction
  (signed conversion, `iqfac` arcsine inverse-quantisation, Levinson-
  style conversion to LPC) and the §4.6.9.3 `tns_ar_filter` all-pole
  pass over the spectrum are **not** performed here — they need the
  spectral context that arrives with the per-AOT IMDCT back-end. The
  §4.6.9.4 `TNS_MAX_ORDER` / `TNS_MAX_BANDS` clamp tables (Tables
  4.156 / 4.157) are similarly the decoder reconstruction layer's
  responsibility; the parser surfaces the literal wire `order` /
  `length` values regardless.
- `Error::TnsDataEncodeInvalid` for caller-side structural bugs the
  writer cannot represent on the wire: `windows.len()` differs from
  `num_windows(window_sequence)` (1 for long sequences, 8 for
  `EIGHT_SHORT_SEQUENCE`); per-window `filters.len()` exceeds the
  `n_filt` field cap (1 on `EIGHT_SHORT_SEQUENCE`, 3 otherwise); a
  filter's `length` exceeds the `length` field cap (15 / 63); a
  filter's `order` exceeds the `order` field cap (7 / 31); the
  `coef[]` length differs from `order`; a coefficient magnitude
  exceeds the `(1 << coef_bits) - 1` cap; or a zero-`order` filter
  carries a non-default `direction` / `coef_compress` that would
  silently be dropped on the wire (those fields are not transmitted
  when `order == 0`).
- 38 new integration tests in `tests/tns_data.rs` covering empty
  windows for every `WindowSequence` variant, every
  `(coef_res, coef_compress)` combination on the long-window branch
  (verifying `coef_bits ∈ {2, 3, 4}`), zero-order filters that skip
  the entire `direction`/`compress`/`coef[]` tail, mixed zero- and
  non-zero-order filters in the same window, max-`n_filt` long
  (3 filters) and short (1 filter) cases, max-field-width short
  windows (length=15 / order=7 / 4-bit coef on all 8 windows),
  heterogeneous `EIGHT_SHORT` packing, three hand-pinned wire-layout
  assertions (one for a 19-bit single-filter long-window block, two
  zero-filter pins for long and short), field-width / helper sanity
  matching Table 4.155 + §4.5.2.3.4 + §4.6.9.3 dispatch, every
  `TnsDataEncodeInvalid` rejection branch (wrong window count,
  `n_filt` / `length` / `order` overflow on both long and short,
  `coef[]` length mismatch, coef-value overflow at every
  `coef_bits` tier, zero-order with `direction`/`compress` set),
  parser unexpected-end at both the `n_filt` and inside-filter
  positions, and a back-to-back two-block sequence that asserts the
  parser / writer carry no inter-block state. Suite size grows
  161 → 199 tests.

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

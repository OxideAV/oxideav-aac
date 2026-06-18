# oxideav-aac

A pure-Rust **AAC** (Advanced Audio Coding) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

Every numeric constant, bit layout, and clause reference is sourced from
the staged ISO/IEC 13818-7 and ISO/IEC 14496-3 specifications under
`docs/audio/aac/`.

## Status

The crate implements the full per-tool AAC-LC decode chain as a set of
composable primitives, from bitstream parse all the way to PCM-domain
samples for a single channel element. The runtime `Decoder` / `Encoder`
registration is **not yet wired** — `register()` installs nothing and
the high-level factory entry points return `Error::NotImplemented`.
What is in place is the spec-faithful tool set plus an element-level
driver that chains it.

### Bitstream parsing

- **ADTS fixed header** (`adts`) — ISO/IEC 13818-7 §1.A.2: sync,
  profile, sampling-frequency index, channel configuration, frame
  length, raw-data-block count, CRC presence flag. The trailing 16-bit
  ADTS CRC is surfaced via `protection_absent` but not yet validated.
- **AudioSpecificConfig** (`asc`) — ISO/IEC 14496-3 §1.6.2.1 including
  the §4.4.1 GASpecificConfig body for all General Audio object types,
  the hierarchical SBR (AOT 5) / PS (AOT 29) wrappers, the
  `extensionFlag` subtree, the `epConfig` field, and the Table 1.15
  trailing `syncExtensionType == 0x2b7` implicit-SBR probe. A
  carrier-bounded `parse_bits_bounded` entry point is exposed for LATM
  `StreamMuxConfig` callers.
- **program_config_element** (`pce`) — §4.4.1.1, used standalone and
  inline inside `asc`.
- **raw_data_block()** walker (`raw_data_block`) — §4.4.2.1: visits each
  `id_syn_ele` and stops at `END`. FIL / DSE / PCE bodies are fully
  consumed; the channel-element body is composed by the modules below.
- **Channel-element body** (`ics_body`) — Table 4.50: `global_gain` →
  `ics_info` → `section_data` → `scale_factor_data` → optional
  `pulse_data` / `tns_data` / `gain_control_data`, surfacing the start
  bit-offset for the spectral data.
- **spectral_data()** (`spectral_data`) — Table 4.56 wire walker and
  bit-exact writer, dispatching onto the Huffman codebooks.
- **extension_payload()** (`extension_payload`) — §4.4.2.7 / Table 4.51
  parser + encoder for the `EXT_FILL`, `EXT_FILL_DATA`, and
  `EXT_DYNAMIC_RANGE` branches. The two SBR-data extension types are
  rejected (`Error::UnsupportedExtensionSbr`) pending an SBR back-end.

### Numeric reconstruction (AAC-LC tool chain)

- **Spectrum Huffman codebooks 1..=11** (`spectrum_huffman`,
  `spectral_codebook`) — the complete Annex 4.A spectrum book set,
  including the ESC book 11, with §4.6.3.3 index↔tuple translation and
  sign-bit / escape-sequence handling.
- **Inverse quantization + scalefactors** (`dequant`) — §4.6.1.3
  non-uniform inverse quantizer and §4.6.2.3.3 scalefactor gain.
- **Decoded spectrum** (`decoded_spectrum`) — §4.6.3.3 `quant_to_spec()`
  de-interleaver plus the per-channel pipeline composing pulse fix-up →
  scalefactor accumulation → inverse quantization + rescale →
  de-interleave → TNS.
- **TNS** (`tns_data`, `tns_coef`, `tns_frame`, `tns_max`,
  `swb_offset`) — §4.6.9 Temporal Noise Shaping: wire parse,
  coefficient inverse-quantisation + conversion to LPC, the all-pole IIR
  pass, and the per-frame region-slicing orchestration.
- **Filterbank** (`filterbank`) — §4.6.11 stateful per-channel IMDCT
  with sine / KBD windows, all four `window_sequence` shapes, eight-short
  internal overlap-add, and inter-frame overlap-add. Pinned by streaming
  TDAC perfect-reconstruction tests. Scope is the 1024/128-line transform
  family; the 960/120-line (`N = 1920/240`) family is out of scope.
- **Channel-pair / noise tools** — M/S stereo de-matrix (`ms_stereo`,
  §4.6.8.1), intensity stereo (`intensity_stereo`, §4.6.8.2), and
  Perceptual Noise Substitution (`pns`, §4.6.13). PNS produces
  energy-exact bands; only the per-coefficient phase is RNG-defined per
  §4.6.13.3, so its output is not byte-exact against any one decoder.
- **Frequency-domain prediction** (`predictor`) — §4.6.6 MPEG-2
  backward-adaptive intra-channel predictor for the AAC **Main** object
  type (AOT 1). A bank of second-order lattice predictors (one per MDCT
  line up to the §4.6.6.2 `PRED_SFB_MAX` limit) reconstructs
  `x_rec = x_est + y_rec` on the signalled bands. Implements the
  §4.6.6.3.2.1 lattice `predict()` + LMS adaptation
  (`α = 0.90625`, `a = b = 0.953125`), the §4.6.6.3.2.3
  `flt_round_inf()` 16-bit-float rounding applied to every stored state
  variable and the predicted value, and the §4.6.6.3.3 reset (the 30
  Table 4.97 cyclic groups + the short-block reset-all). Wired into
  `element_decode`: the bank runs every long frame *before* TNS (and is
  mutually exclusive with LTP by object type), persisting the
  backward-adaptive state across frames.
- **Long-Term Prediction** (`ltp`) — §4.6.7 long-window LTP: the
  Table 4.98 coefficient codebook, the §4.6.7.3 `predict()` single-tap
  time-domain predictor (`x_est(i) = ltp_coef·x_rec(i − ltp_lag)`) over
  a per-channel `x_rec` reconstruction history, the windowed analysis
  `MDCT(x_est)` (the §4.6.15.3.3 / §4.6.11.3.1 forward transform, now a
  reusable `filterbank` primitive), and the per-sfb
  `X_rec = X_est + Y_rec` combination on the bands flagged by
  `ltp_long_used`. LTP is restricted to long windows for the AAC LTP
  object type (§4.6.7.1); short-window LTP and the ER AAC LD `M = N/2`
  lag offset are out of scope.
- **TNS analysis filter** (`tns_coef::tns_ma_filter`,
  `tns_frame::tns_analysis_frame`) — §4.6.7.4.1 / Figure 4.30: the
  all-zero (moving-average, FIR) inverse of the §4.6.9.3 all-pole
  synthesis filter, `y(n) = x(n) + Σ lpc[k]·x(n−k)`. Run over the same
  per-window region walk as `tns_decode_frame`; analysis ∘ synthesis is
  the identity over a shared region, which is the §4.6.7.4.1
  noise-shaping invariant.
- **Element decode driver** (`element_decode`) — `ElementDecoder` chains
  the whole stack per element: `decode_sce` for SCE / LFE and
  `decode_cpe` for a CPE (pulse → dequant → `quant_to_spec()` → M/S →
  intensity → PNS → **LTP → TNS** → filterbank), carrying the
  per-channel overlap-add tail **and the §4.6.7.3 LTP reconstruction
  history** across frames. LTP runs in the §4.6.7.4.1 / Figure 4.30
  block order — long-term synthesis (with the all-zero TNS analysis
  filter applied to `X_est`) *before* the §4.6.9 TNS synthesis filter,
  so the single synthesis pass shapes the residual while undoing the
  analysis on the LTP contribution.

### SBR frequency band setup (HE-AAC)

- **SBR frequency band tables** (`sbr_freq_bands`) — §4.6.18.3.2 the
  static, header-only half of the Spectral Band Replication band setup,
  computed directly from the closed-form spec algorithm (no QMF back-end
  required):
  - `k0` / `k2` — §4.6.18.3.2.1 the low and high QMF subband
    boundaries. `k0 = startMin + offset(bs_start_freq)` with the
    per-`FsSBR` `offset` table and the `startMin = NINT(c·128/FsSBR)`
    thresholds; `k2` covers the `bs_stop_freq < 14` `stopDkSort`
    accumulation path and the `bs_stop_freq == 14 / 15`
    `min(64, 2·k0)` / `min(64, 3·k0)` shortcuts.
  - `master_table` — §4.6.18.3.2.1 `fMaster`, both the Figure 4.39
    linear path (`bs_freq_scale == 0`, the `dk`/`vDk`/`k2Diff`
    away-from-zero correction walk) and the Figure 4.40 warped path
    (`bs_freq_scale > 0`, the `bands`/`warp` log-spaced regions with
    the single-/two-region split at `k2/k0 > 2.2449` and the
    `min(vDk1) < max(vDk0)` smoothing step).
  - `HiLoTables::derive` — §4.6.18.3.2.2 the derived `fTableHigh`,
    `fTableLow` (the `i(k) = 2k − (1−(−1)^NHigh)/2` decimation), and
    `fTableNoise` (the `NQ = max(1, NINT(bs_noise_bands·log2(k2/kx)))`
    band count plus its `i(k)` recursion), along with the `M` and
    `k_x` outputs every later SBR stage keys off.
  - The §4.6.18.3.6 requirements are enforced (`k2 > k0`,
    `numBands > 0`, `vDk > 0`, `bs_xover_band < NMaster`), surfacing
    `Error::SbrFreqBandInvalid` on violation. The §4.6.18.3.2.3
    limiter band table is out of scope here — its `bs_limiter_bands >
    0` path consumes the §4.6.18.6 patch borders that need the QMF
    patching back-end.

## Not yet supported

- Runtime `Decoder` / `Encoder` registration — `register()` is a no-op
  and the factory functions return `Error::NotImplemented`.
- `expected.wav` PCM comparison (the crate carries no resampler /
  clipper, and PNS bands are energy-exact rather than sample-exact).
- The SSR gain-control ladder (§4.6.12) — its side-info is parsed but
  not applied. (The Main frequency-domain predictor, §4.6.6, is now
  fully wired into `element_decode` for the AAC Main object type on long
  windows — see `predictor` above. LTP, §4.6.7, is likewise wired in
  with the §4.6.7.4.1 / Figure 4.30 TNS-analysis-in-loop ordering.
  Short-window LTP and the ER AAC LD `M = N/2` lag offset remain out of
  scope per the §4.6.7.1 long-window restriction.)
- SBR / PS synthesis (needs a QMF analysis / synthesis filterbank and
  the §4.6.18.6 patching back-end) — the §4.6.18.3.2.1/.2 frequency
  band tables (`fMaster` / `fTableHigh` / `fTableLow` / `fTableNoise`)
  are in place (see `sbr_freq_bands` above), but the envelope / noise
  decode, the limiter table, and the QMF filterbanks are not. PS, the
  coupling-channel (CCE) contribution, and the ER AAC LD 480/512
  transform variants likewise remain out of scope.
- The `scale_factor_data()` error-resilient (RVLC) branch.
- LATM/LOAS transport (§1.7.3) and ADTS CRC-16 validation.

## License

MIT — see [LICENSE](./LICENSE).

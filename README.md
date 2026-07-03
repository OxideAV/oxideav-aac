# oxideav-aac

A pure-Rust **AAC** (Advanced Audio Coding) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

Every numeric constant, bit layout, and clause reference is sourced from
the staged ISO/IEC 13818-7 and ISO/IEC 14496-3 specifications under
`docs/audio/aac/`.

## Status

The crate implements the full AAC-LC decode chain end to end — from
ADTS bitstream parse through the per-tool reconstruction to interleaved
16-bit PCM — **plus the complete §4.6.18 SBR back-end (HE-AAC v1)**,
and **wires both into the framework's runtime `Decoder` trait**
(`register()` installs an AAC decoder under id `"aac"`; see
`codec_decoder` below). The PCM is validated byte-exactly (within the
1-LSB IMDCT-rounding bound) against the staged `expected.wav` corpus —
**including the HE-AAC v1 SBR fixture, which decodes 99.98%
sample-exact with a max error of 1 LSB** at the doubled output rate.
The `Encoder` side is **not yet wired** — the bit-exact wire writers are
in place but there is no rate-control / psychoacoustic encoder back-end.

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
- **Error-protection CRC generator** (`crc`) — §1.8.4.5: the full
  family of MPEG-4 Audio CRC generation polynomials (`CRC4`..`CRC32`,
  including the `CRC8` LATM `StreamMuxConfig()` `crcCheckSum` and the
  16-bit `x¹⁶+x¹⁵+x²+1`), a zero-init MSB-first shift-register
  (`crc_bits` / `crc_bytes`) implementing the §1.8.4.5
  `M(x)·xᵏ = Q(x)·G(x) + R(x)` remainder with the normative
  output-bit inversion ("written in a reversed manner, i.e. each bit
  is inverted"), and the [`crc::stream_mux_config_crc`] LATM helper.
  Cross-checked against an independent GF(2) long-division reference
  and the codeword-divisibility property. The ADTS
  `adts_error_check()` region-selection CRC is *not* covered here —
  ISO/IEC 13818-7 cites it to a different normative reference
  (ISO/IEC 11172-3 §2.4.3.1).
- **RVLC error-resilient scalefactor coding** (`rvlc`,
  `scale_factor_data::ErScaleFactorData`) — §4.6.16.2 the
  reversible-variable-length-coding replacement for the §4.6.3
  noiseless coding of scalefactors, used when
  `aacScalefactorDataResilienceFlag == 1`. The `rvlc` module
  transcribes the symmetric (bit-palindrome) RVLC codebook
  (Table 4.166, deltas `-7..=+7` with `±7` the `ESC_FLAG`), the eight
  asymmetric *forbidden* codewords (Table 4.167) whose appearance is
  surfaced as the §4.6.16.2.1 in-band error-detection event, and the
  54-entry RVLC-ESC Huffman codebook (Table 4.168) — every codebook
  proven prefix-free and round-tripping, and independently
  cross-validated against the staged packed binary-tree node tables.
  `ErScaleFactorData::parse` / `::write` decode and re-encode the whole
  Table 4.53 RVLC branch: the `sf_concealment` / `rev_global_gain` /
  `length_of_rvlc_sf` (11 bits for `EIGHT_SHORT_SEQUENCE`, else 9)
  header, the RVLC base-delta band loop (first PNS band keeping the
  9-bit PCM seed), the optional `sf_escapes_present` /
  `length_of_rvlc_escapes` second pass folding each escape into its
  `±ESC_FLAG` base (`+7 + esc` / `-7 - esc` per §4.6.16.2.1), and the
  `dpcm_is_last_position` / `dpcm_noise_last_position` backward seeds.
  Both `length_of_*` fields are validated against the bits actually
  consumed. The reconstructed records share the non-resilient
  `ScaleFactorData` shape, so the §4.6.2.3.2 forward DPCM
  `accumulate()` pass consumes them unchanged — pinned by a test that
  an RVLC stream and the Huffman stream carrying the same deltas
  accumulate to identical absolute scalefactors. The
  resilience-flag dispatch from `ics_body` is now wired (see the
  error-resilient ICS body below); the RVLC bitstream path itself is
  decoded end to end.
- **Error-resilient channel-element body**
  (`ics_body::IcsBody::parse_er` / `::parse_with_ics_info_er`,
  `section_data::SectionData::parse_er` / `::write_er`) — ISO/IEC
  14496-3 §4.4.6 Tables 4.50 / 4.52, the ER General-Audio object types
  (AOTs 17 / 19 / 20 / 23). Drives all three resilience branches off
  the `AacResilienceFlags` triplet: `section_data()` through the 5-bit
  `sect_cb` branch (carrying the §4.6.16.4 virtual codebooks 16..=31,
  whose `ESC_HCB` / `>= 16` runs take the fixed `sect_len_incr = 1`
  single-band coding) when `aacSectionDataResilienceFlag` is set;
  `scale_factor_data()` through the RVLC `ErScaleFactorData` branch
  (its reconstruction mirrored into the shared `scale_factor_data`
  field so the §4.6.2.3.2 accumulate pass is branch-agnostic, with the
  RVLC seeds retained in `er_scale_factor_data`) when
  `aacScalefactorDataResilienceFlag` is set; and the
  `length_of_reordered_spectral_data` (14-bit) +
  `length_of_longest_codeword` (6-bit) HCR length fields in
  `reordered_spectral_lengths` when `aacSpectralDataResilienceFlag` is
  set. The trailing `reordered_spectral_data()` (HCR) payload is the
  caller's responsibility, exactly as `spectral_data()` is on the
  non-resilient path.
- **HCR segmentation / pre-sorting scaffold** (`hcr`) — ISO/IEC
  14496-3 §4.6.16.3.3 / §4.6.16.3.5. The deterministic, header-only
  half of Huffman codeword reordering: the Table 4.170 `maxCwLen`
  table, the §4.6.16.3.3.1 `codebookPriority[32]` table + the
  `assignedUnitNr` pre-sorting metric, the
  `segmentWidth = min(maxCwLen, length_of_longest_codeword)`
  derivation, the §4.6.16.3.2 length-field clamps, and the
  `Segmentation` layout that instantiates PCW segments until the
  `length_of_reordered_spectral_data` buffer is exhausted (folding the
  trailing bits into the last segment). `ReorderPlan::build` then runs
  the §4.6.16.3.3.4 `ReorderSpectralData()` writing scheme (PCWs
  forward from each segment start, then the non-PCW set / trial loop
  with the per-set `ToggleWriteDirection()` and the modulo-shift
  `segment = (trial + codewordBase) % numberOfSegments`) to resolve,
  for each codeword, the ordered global buffer bit positions
  (MSB-first) that carry its bits — pinned by a bijection invariant
  (every buffer bit covered exactly once). The remaining step binds
  this geometry to the in-place §4.6.3.3 Huffman decode (PCWs first to
  learn the non-PCW lengths) and awaits an HCR-bearing conformance
  stream for bit-exact validation.

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
- **Coupling channel element** (`cce`) — §4.6.8.3 / Table 4.8. The CCE
  coupling header (`CouplingHeader`: `ind_sw_cce_flag`,
  `num_coupled_elements`, the per-target `cc_target_is_cpe` /
  `cc_target_tag_select` / `cc_l` / `cc_r` list with the Table 4.153
  shared-vs-split `num_gain_element_lists` derivation, `cc_domain` /
  `gain_element_sign` / `gain_element_scale`) and the trailing gain-list
  block (`CouplingGains`: per-target `common_gain_element` or per-`(g,
  sfb)` `dpcm_gain_element` running-sum lists — the §4.6.8.3.3
  `ind_sw_cce_flag ⇒ common-gain-only` constraint and the embedded-SCE
  `sfb_cb` `ZERO_HCB` skip — reusing the §4.A.1 scalefactor Huffman
  codebook `hcod_sf`). `CouplingChannelElement` ties the whole Table 4.8
  element (header → embedded `individual_channel_stream(0,0)` body +
  spectrum → gain lists) together, and `CouplingGains::cc_gain` computes
  the §4.6.8.3.3 `couple_channel()` factor `cc_gain = cc_sign ·
  cc_scale^gain` (Table 4.154 `cc_scale_table`, implicit list-0 natural
  scaling). `CouplingGains::couple_channel` applies the §4.6.8.3.3
  per-band scale-and-add — the spec's group / window-group / sfb /
  coefficient loop multiplying the embedded-SCE spectrum by the
  per-`(g, sfb)` `cc_gain` and adding it onto a target channel's
  window-major spectrum (implicit list 0 in natural scaling, `ZERO_HCB`
  bands skipped). The stream decode loop fully consumes a CCE so a
  CCE-bearing frame's SCE / CPE channels still decode; the per-band
  coupling math is implemented and unit-tested, but the decode-loop
  wiring that matches targets by `cc_target_tag_select` and intercepts
  their spectra at the `cc_domain` stage awaits a CCE fixture for
  end-to-end validation (docs-gap).
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

### SSR gain control back-end (§4.6.12)

The §4.6.12 SSR (Scalable Sample Rate, AOT 3) gain-control **back-end**
is now implemented — the reconstruction counterpart to the
`gain_control_data` Table 4.12 wire parser, validated independent of any
external SSR implementation:

- **Gain-control reconstruction** (`gain_control`) — §4.6.12.3.1–3. The
  §4.6.12.3.1 gain-control data decoding (the Table 4.108 `AdjLoc()` =
  `8·AC` and Table 4.109 `AdjLev()` = `AV − 4` tables, the `NADW` /
  `ALOC` / `ALEV` ladder with the step-(3) `ALOC(0)=0` / `ALEV(0)` rule
  and the step-(4) per-window-sequence endpoint), the §4.6.12.3.2
  gain-control function setting (the `M_{W,B,j}` index, the `FMD`
  fragment-modification function with the `Inter(a,b,j)` geometric-blend
  ramp, the per-sequence `GMF` composition threading the cross-frame
  `PFMD`, and the inversion `AD(j) = 1/GMF(j)`), and the §4.6.12.3.3
  windowing + overlapping (`GainBandState::window_overlap` applies
  `T = AD·U` then overlap-adds per `window_sequence` into the band sample
  data `V_B`, threading the cross-frame `PT_B` tail). All four
  `window_sequence` shapes are covered; the spec initial values
  `PFMD ≡ 1.0` / `PT ≡ 0.0` are honoured, and the input-read vs
  produced `PFMD` lengths (which differ per sequence) are tracked
  separately with a persistent 256-entry carry.
- **IPQF synthesis filter** (`ipqf`) — §4.6.12.3.4. The Table 4.110
  length-96 prototype `Q(j)` (the symmetric `Q(j) = Q(95 − j)` half
  mirrored to 96), the cosine modulation
  `Q_B(j) = Q(j)·cos((2B+1)(2j−3)π/16)`, the 4× upsampling
  `Ṽ_B(j) = V_B(j/4)`, and the streaming convolution
  `AS(n) = Σ_B Σ_j Q_B(j)·Ṽ_B(n−j)` as a polyphase bank (`Ipqf`) that
  retains a 24-deep per-band history across frames — pinned by an
  impulse-response test against the direct §4.6.12.3.4 convolution
  (`AS(n) = Q_0(n)`).
- **Per-channel driver** (`ssr`) — `SsrGainControl::decode_frame`
  composes the four-band `GainBandState` and the `Ipqf` into one
  persistent per-channel pipeline: the four per-band IMDCT outputs
  `U_{W,B}` plus the decoded `gain_control_data()` → the §4.6.12.3.3
  per-band windowing/overlap → the §4.6.12.3.4 IPQF synthesis → the PCM
  `AS(n)` (1024 samples/frame for the steady `ONLY_LONG` / `EIGHT_SHORT`
  case). PQF band 0 is never gain-controlled.

The remaining §4.6.12.1 **front half** — splitting the transmitted
spectrum into the four PQF-band coefficient columns, the even-band
spectral reversal, and the per-band 256-coefficient (long) /
32-coefficient (short) IMDCTs that feed `U_{W,B}` — is **not yet
wired**: the staged §4.6.12 text does not specify the spectrum-to-band
arrangement (contiguous-block vs interleaved split, the precise
even-band reversal indexing), so the spectrum→band mapping is a
documented docs gap. The back-end above is complete and validated; only
this front-half de-interleave is pending the spec detail.

### SBR bitstream decode (HE-AAC)

The full SBR side-info path is now decoded from the `extension_payload`
SBR element down to the reconstructed quantized envelope / noise-floor
scalefactors — every numeric table sourced from the ISO/IEC 14496-3
spec PDF (the §4.A normative Huffman grids and the §4.4.2.8 syntax
tables), independent of any external SBR table extraction.

- **SBR Huffman codebooks** (`sbr_huffman`) — §4.A.6.1, all ten
  normative envelope / noise codebooks (Tables 4.A.79–4.A.88)
  transcribed from the spec codeword grids and validated complete +
  prefix-free. `sbr_huff_dec()` reads MSB-first and returns the signed
  DPCM delta (`index − LAV`); `env_tables()` / `noise_tables()` pick the
  `(t_huff, f_huff)` pair from the §4.6.18.3 coupling / channel /
  `bs_amp_res` selection (the freq-direction noise tables alias the
  3.0 dB envelope freq tables per Table 4.A.78 Note 2).
- **`sbr_header()`** (`sbr_header`) — §4.4.2.8 Table 4.63: the
  fixed-width header plus the two optional extra blocks, with the
  Table 4.63 Note 3 defaults (Tables 4.105–4.111) applied when an extra
  flag is clear. `band_geometry_changed()` flags a §4.6.18.3.3 reset,
  and `derive_bands()` chains into the band-setup pipeline below.
- **`sbr_grid()` / `sbr_dtdf()` / `sbr_invf()`** (`sbr_grid`) —
  §4.4.2.8 Tables 4.69–4.71: all four `bs_frame_class` layouts (FIXFIX
  / FIXVAR / VARFIX / VARVAR) with the envelope count, variable /
  relative borders, `ptr_bits = ceil(log2(num_env + 1))` pointer,
  reversed FIXVAR freq-res order, single-envelope FIXFIX `bs_amp_res`
  override, and `bs_num_noise` derivation; the delta-direction flags;
  and the per-noise-band 2-bit inverse-filtering modes.
- **`sbr_envelope()` / `sbr_noise()`** (`sbr_envelope`) — §4.4.2.8
  Tables 4.72–4.73: the raw `bs_data_*` delta arrays, with the
  fixed-width absolute start value (5/6/7-bit per the coupling /
  channel / `bs_amp_res` context; noise always 5-bit) and the
  frequency- vs time-direction Huffman deltas, over `NHigh` / `NLow`
  envelope bands and `NQ` noise bands.
- **Envelope / noise DPCM reconstruction** (`sbr_reconstruct`) —
  §4.6.18.3.5: inverts the delta coding to the quantized scalefactors
  `E_Q(k,l)` / `Q(k,l)`. Frequency deltas accumulate from the start
  value; time deltas add to the reference envelope (previous in-frame,
  or the prior frame's last envelope for `l == 0`) with the `i(k)`
  high↔low band remap when the reference resolution differs; the
  coupled second channel's `δ = 0.5` is applied as an integer ×2 on the
  even transmitted values, threading cross-frame state.
- **Element framing** (`sbr_element`) — §4.4.2.8 Tables 4.65 / 4.66 /
  4.74: `SbrElement::parse_single` / `parse_pair` decode a whole SBR
  data element in spec order — the optional `bs_data_extra` field, the
  per-channel grid / dtdf / invf / envelope / noise blocks (coupled
  shared-grid vs. independent-grid layouts, second coupled channel in
  balance mode), the `sbr_sinusoidal_coding()` add-harmonic flags, and
  the `bs_extended_data` block (id + raw body captured for a later PS
  pass). The single-envelope FIXFIX `bs_amp_res` override is applied
  before envelope decode.
- **`sbr_extension_data()`** (`sbr_extension`) — §4.4.2.8 Table 4.62:
  the top-level walker that ties the header + element framing into a
  whole SBR extension payload, in spec order — the optional 10-bit
  `bs_sbr_crc_bits` (for the `EXT_SBR_DATA_CRC` type), the
  `bs_header_flag` + `sbr_header()`, then `sbr_data(id_aac, bs_amp_res)`
  dispatching onto `parse_single` (ID_SCE) / `parse_pair` (ID_CPE) with
  the band tables derived from the active header at the SBR internal
  rate (`FsSBR = 2·core`), and the trailing `bs_fill_bits` alignment
  (`num_align_bits = (8·cnt − 4 − num_sbr_bits) % 8`). A clear
  `bs_header_flag` reuses the threaded previous header (the
  non-scalable core fixes `sbr_layer == SBR_NOT_SCALABLE`, so the flag
  is always present). Reachable from the natural FIL entry point via
  `extension_payload::ExtensionPayload::parse_with_sbr`, which routes
  the SBR extension types here (the default `parse` still rejects them,
  keeping the byte-exact AAC-LC corpus path untouched).

The SBR *bitstream* side info is decoded end to end — CRC field,
header, element framing, band tables, and envelope / noise DPCM
reconstruction — and the **back-end DSP is now implemented too** (see
the next section).

### SBR back-end (HE-AAC v1) — §4.6.18

The complete SBR reconstruction chain, from the core decoder's time
signal to dual-rate PCM, validated **99.98% sample-exact (max error
1 LSB)** against the staged HE-AAC v1 `expected.wav`:

- **QMF filterbanks** (`sbr_qmf`) — §4.6.18.4 / Figures 4.42–4.44: the
  Table 4.A.89 640-tap prototype window (transcribed digit-for-digit
  from the spec PDF), the 32-band complex analysis bank, the 64-band
  real-output synthesis bank (dual-rate), and the downsampled
  32-channel synthesis variant. Pinned by near-perfect-reconstruction
  properties (< 1e-4 error ratios).
- **Dequantization + stereo decoding** (`sbr_dequant`) — §4.6.18.3.5:
  `EOrig = 64·2^(E/a)`, `QOrig = 2^(6 − Q)`, and the coupled-pair pan
  split with `panOffset = [24, 12]` (energy-sum-preserving).
- **Time / frequency grid** (`sbr_time_grid`) — §4.6.18.3.3: the
  `tE` / `tQ` border vectors for all four frame classes, the
  Table 4.174 `middleBorder` and the Table 4.176 `lA`.
- **HF generation** (`sbr_hf_gen`) — §4.6.18.6: the Figure 4.48 patch
  construction, the covariance-method second-order inverse filtering
  (`εInv = 1e-6`, `|α| ≥ 4` reset), the Table 4.175 chirp-factor
  blend, and the patched `XHigh` generator.
- **Limiter band table** (`sbr_limiter`) — §4.6.18.3.2.3 /
  Figure 4.41, fed by the patch borders (closing the previously
  deferred limiter-table item).
- **Envelope adjustment** (`sbr_env_adjust` + `sbr_noise_table`) —
  §4.6.18.7: mapping, `ECurr` estimation (both `bs_interpol_freq`
  regimes), amplitude-domain gains (the spec PDF's typeset equations
  carry square roots the plain text layer drops), the limiter /
  boost compensation, `hSmooth` smoothing with cross-frame tails, the
  Table 4.A.91 noise table with the running `fIndexNoise`, and the
  sinusoid injection with the `(−1)^(m+kx)` alternation.
- **Frame driver** (`sbr_decoder`) — §4.6.18.5 / Figure 4.47: the
  `tHFGen = 8`-slot `XLow` history, header-reset handling, the
  `lTemp` splice of the previous frame's `Y'`, the coupled-pair invf
  sharing, and the pure-upsampling path for SBR-less frames.
- **Stream wiring** (`decode`) — the ADTS `StreamDecoder` walks FIL
  extension payloads via `extension_payload::parse_with_sbr`, attaches
  each SBR payload to its preceding SCE / CPE, threads the
  `sbr_header()` reuse state per element slot, and (once SBR-active)
  emits every frame at the doubled rate — 2048 samples/channel — with
  SBR-less frames upsampled so the output rate never flaps. The
  runtime `Decoder` trait surfaces the dual-rate frames unchanged
  (pinned byte-identical to the raw `StreamDecoder`).

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

### LATM / LOAS transport framing

The §1.7 low-overhead transport layer is now decoded from the LOAS
sync frame down to the recovered MPEG-4 Audio access units — every
field sourced from the ISO/IEC 14496-3 §1.7 syntax tables.

- **`StreamMuxConfig()`** (`latm::StreamMuxConfig`) — §1.7.3.1
  Table 1.42 plus `LatmGetValue()` (Table 1.43). Decodes the whole
  multiplex configuration: the `audioMuxVersion` / `audioMuxVersionA`
  version flags (with the `audioMuxVersion == 1` `taraBufferFullness`
  and per-ASC length-prefix + `fillBits` extensions),
  `allStreamsSameTimeFraming`, `numSubFrames` / `numProgram` /
  per-program `numLayer`, and the per-`streamID[prog][lay]`
  `LayerConfig` table — each layer carrying its inline
  `AudioSpecificConfig()` (parsed via the `asc` module's
  `parse_bits` / `parse_bits_bounded` entry points) or the resolved
  `useSameConfig` inheritance, the `frameLengthType`, and the type-0
  `latmBufferFullness` / CELP-core `coreFrameOffset` or type-1
  `frameLength`. The `crcCheckSum` is recomputed against the
  configuration prefix via the §1.8.4.5 `CRC8` generator and
  validated. The reserved `audioMuxVersionA == 1` branch and the
  CELP (`3`/`4`/`5`) / HVXC (`6`/`7`) `frameLengthType` values index
  frame-length tables for object types this AAC-focused crate does not
  decode, so they surface dedicated errors.
- **`AudioMuxElement()`** (`latm::AudioMuxElement`) — §1.7.3.1
  Tables 1.41 / 1.44 / 1.45. Recovers a whole multiplexed element: the
  `muxConfigPresent` `useSameStreamMux` branch (inline
  `StreamMuxConfig()` vs. inherited previous config), the per-subframe
  `PayloadLengthInfo()` + `PayloadMux()` loop over `numSubFrames + 1`
  frames (both the `allStreamsSameTimeFraming` program/layer walk and
  the `numChunk` chunk layout with its `streamIndx` + `AuEndFlag`), the
  `frameLengthType`-0 `MuxSlotLengthBytes` 8-bit-escape byte count and
  the `frameLengthType`-1 fixed `(frameLength + 20) * 8` bits, the
  `otherData` skip, and the trailing `ByteAlign()`. Each access unit is
  returned as a `MuxPayload` carrying the raw §4.4.2.1
  `raw_data_block()` bytes.
- **`AudioSyncStream()` / `EPAudioSyncStream()`**
  (`latm::AudioSyncStream`, `latm::EpAudioSyncHeader`) — §1.7.2.1
  Tables 1.36 / 1.37. `AudioSyncStream` scans a LOAS byte buffer for
  the 11-bit `0x2B7` syncword, reads the 13-bit `audioMuxLengthBytes`,
  and decodes the byte-aligned `AudioMuxElement(1)` body, exposing an
  `Iterator` of `LoasFrame`s with the `StreamMuxConfig` threaded across
  frames for `useSameStreamMux` inheritance. `EpAudioSyncHeader`
  decodes the `EPAudioSyncStream` FEC header (`0x4DE1` syncword,
  `futureUse`, `audioMuxLengthBytes`, `frameCounter`, `headerParity`)
  and reports the byte-aligned `EPMuxElement` body offset.

- **`LoasDecoder`** (`latm::LoasDecoder`) — the end-to-end LATM/LOAS →
  PCM driver. `decode_all` walks the `AudioSyncStream`, and for every
  recovered `MuxPayload` drives the payload's §4.4.2.1 `raw_data_block()`
  through the shared `decode::StreamDecoder::decode_raw_data_block` core,
  configuring the decode from the layer's `AudioSpecificConfig` (AOT /
  `samplingFrequencyIndex` / resolved sample rate). One `StreamDecoder`
  is held per `streamID[prog][lay]` so each multiplexed stream's
  §4.6.11 overlap / §4.6.7 LTP / §4.6.6 predictor state threads
  independently. An SBR/PS-configured ASC is rejected with
  `Error::LatmSbrUnsupported` (no SBR back-end). Pinned against the
  `aac-latm-stream` fixture (stereo, 44.1 kHz) to a §8 PCM-RMS error
  ratio of 0.0004, and proven bit-identical to a hand-fed
  `decode_raw_data_block` pass.

The runtime `Decoder` (`codec_decoder::AacDecoder`) auto-detects its
carrier on the first packet and routes LOAS packets through `LoasDecoder`
(see "Runtime `Decoder` registration" below). The `EPMuxElement()`
EP-tool payload de-interleave is out of scope.

### Stream decode + PCM output

- **Integer-PCM rendering** (`pcm`) — §4.6.11 filterbank `f64` time
  signal → 16-bit signed PCM: `nint` (the §1.3 `NINT()` round-half-
  away-from-zero operator), `to_s16` (round + saturate), `channel_to_s16`,
  and `interleave_s16` (element-order interleave). The conversion is the
  only output-rendering step (no resampler / dither), so it is fully
  spec-determined. The **canonical channel reorder** for default
  `channelConfiguration` layouts (Table 1.19, see `channel_map` below) is
  applied to the per-channel buffers *before* this interleave.
- **Default-config channel reorder** (`channel_map`) — ISO/IEC 14496-3
  §1.6.3.5 / Table 1.19. A `raw_data_block()` lists its channel elements
  in bitstream order, so the decoder produces channels in element order
  (e.g. a 5.1 stream as `C, L, R, Ls, Rs, LFE` for `SCE, CPE, CPE, LFE`);
  `channel_map::reorder_channels` permutes them into the canonical
  interleaved order that `oxideav_core::ChannelLayout` adopts (the
  WAVE_FORMAT_EXTENSIBLE / BS.775 convention — 5.1 becomes
  `L, R, C, LFE, Ls, Rs`). The driver threads the signalled
  `channelConfiguration` through `decode_raw_data_block` and applies the
  reorder for default configs **1–6**; mono / stereo are identity
  permutations and configs **0** (PCE-defined) / **7** (the
  amendment-specific 7.1 element→speaker mapping) are left in element
  order pending a later pass. Multichannel byte-exact validation against
  the staged 5.1 / 7.1 `expected.wav` fixtures awaits an in-crate MP4
  demuxer (the fixtures are `.m4a`-wrapped); the reorder itself is pinned
  by unit + end-to-end assembled-frame tests.
- **Stream-level ADTS decode driver** (`decode`) — `StreamDecoder` walks
  the §4.4.2.1 `raw_data_block()` of each ADTS frame above the
  per-element driver, keying one `ElementDecoder` per
  `(syntactic-element-id, element_instance_tag)` slot so every element's
  §4.6.11 overlap / §4.6.7 LTP / §4.6.6 predictor state persists across
  frames, and renders to element-order interleaved s16 PCM. `decode_all`
  walks a whole raw-ADTS buffer (ID3v2-skip + `aac_frame_length`
  framing). **The decoded PCM is validated against the staged
  `expected.wav` corpus**: the two PNS-free ADTS fixtures
  (`aac-lc-mono-8000-16kbps-adts`, `aac-lc-intensity-stereo`) are
  **99.9% byte-exact** to the reference s16 output with a **max error of
  1 LSB** — the residual is purely the difference between this crate's
  `f64` direct-sum IMDCT and a `float32` fast transform. The PNS-bearing
  fixtures are compared in the PCM RMS domain (per the fixtures-doc §8),
  where the error-to-signal RMS ratio stays below 0.1%; full
  byte-exactness on those is precluded by the §4.6.13.3 spec-undefined
  noise-phase RNG (energy is normative, phase is not). A
  `coupling_channel_element()` (CCE) carried in the block is now fully
  consumed (parsed via `cce::CouplingChannelElement`) so the frame's
  SCE / CPE channels still decode; the §4.6.8.3.3 coupling *contribution*
  onto the targets is decoded but not yet applied. Multi-`raw_data_block`
  ADTS and SBR up-sampling remain out of scope.
- **Runtime `Decoder` registration** (`codec_decoder`) — `AacDecoder`
  adapts the persistent `StreamDecoder` / `LoasDecoder` into the
  framework's packet-in / frame-out `oxideav_core::Decoder` trait. It
  **auto-detects the carrier** on the first packet — the `0xFFF` ADTS
  syncword vs. the `0x2B7` LOAS `AudioSyncStream` syncword — and then
  routes every later packet the same way: ADTS frames through
  `StreamDecoder::decode_frame` (ID3v2-skip + `aac_frame_length`
  framing; one or many frames per packet), LOAS packets through
  `LoasDecoder::decode_all` (one or many sync frames per packet, with
  the `StreamMuxConfig` and per-stream state threaded across packets).
  `receive_frame` returns one interleaved-S16 `AudioFrame` (1024
  samples/channel) per decoded access unit, `flush` drains to `Eof`,
  and `reset` drops both backends and re-arms carrier detection for a
  clean post-seek restart. `register()` installs it under id `"aac"`,
  claiming the MP4 object-type `0x40`, WAVEFORMATEX `0x00FF` / `0x1601`,
  the `mp4a` / `aac ` FourCCs, and the Matroska `A_AAC` CodecID; the
  probe scores a structurally-confirmed ADTS header at 1.0 and a bare
  LOAS syncword at 0.9 to win shared tags. Both carrier outputs are
  pinned byte-identical to their underlying `StreamDecoder` /
  `LoasDecoder`.

## Not yet supported

- Runtime `Encoder` registration — `register()` installs the AAC
  **decoder** (id `"aac"`) but no encoder; the bit-exact wire writers
  exist (`raw_data_block::FrameAssembler` and the per-tool `write`
  primitives) but there is no rate-control / psychoacoustic encoder
  back-end to drive them.
- The SSR gain-control tool (§4.6.12) — the §4.6.12.3.1–4 **back end**
  is now implemented and validated (gain-control function
  reconstruction, the per-band windowing/overlap, and the IPQF synthesis
  filter — see the "SSR gain control back-end" section above). What
  remains is the §4.6.12.1 **front half**: the spectrum → PQF-band
  coefficient-column de-interleave, the even-band spectral reversal, and
  the per-band 256/32-coefficient IMDCTs that feed `U_{W,B}` — blocked on
  a docs gap (the staged §4.6.12 text does not specify the
  spectrum-to-band arrangement). (The Main frequency-domain predictor,
  §4.6.6, is now
  fully wired into `element_decode` for the AAC Main object type on long
  windows — see `predictor` above. LTP, §4.6.7, is likewise wired in
  with the §4.6.7.4.1 / Figure 4.30 TNS-analysis-in-loop ordering.
  Short-window LTP and the ER AAC LD `M = N/2` lag offset remain out of
  scope per the §4.6.7.1 long-window restriction.)
- SBR remainders — the §4.6.18 SBR tool is **implemented end to end**
  (bitstream + back-end DSP, see the "SBR back-end" section above) and
  wired into the ADTS `StreamDecoder` / runtime `Decoder`, validated
  99.98% sample-exact against the HE-AAC v1 fixture. Still open: PS
  (parametric stereo, HE-AAC v2) — whose `sbr_extension` payload bytes
  are captured but not decoded — so an HE-AAC v2 stream decodes as
  HE-AAC v1 (mono SBR without the stereo synthesis); the LATM/LOAS
  driver still rejects an SBR-configured ASC
  (`Error::LatmSbrUnsupported`, pending the same wiring the ADTS path
  got); the 10-bit `bs_sbr_crc_bits` field is captured, not verified;
  the §4.6.18.4.3 downsampled-output mode and the §4.6.18.8 low-power
  variant are not selectable; and the ER AAC LD 480/512 transform
  variants remain out of scope. The coupling-channel (CCE) bitstream is
  decoded end
  to end (`cce`, see the tool-chain section above) and consumed by the
  stream decode loop; the §4.6.8.3.3 per-band coupling math
  (`CouplingGains::couple_channel`) is implemented and unit-tested, but
  the cross-element *application* (matching targets by
  `cc_target_tag_select` and scaling the decoded CCE spectrum onto the
  addressed SCE / CPE channels at the `cc_domain` stage) awaits a CCE
  fixture for end-to-end validation.
- The full `reordered_spectral_data()` (HCR) payload decode — the
  error-resilient channel-element body (`ics_body::IcsBody::parse_er`)
  now selects all three §4.4.6 resilience branches off the
  `AacResilienceFlags` triplet (ER `section_data()`, RVLC
  `scale_factor_data()`, and the HCR `length_of_reordered_spectral_data`
  / `length_of_longest_codeword` fields), and the `hcr` module supplies
  the §4.6.16.3.3 segmentation / pre-sorting scaffold (`maxCwLen`,
  `codebookPriority`, `assignedUnitNr`, `segmentWidth`, the
  `Segmentation` PCW-segment layout). What remains is the
  §4.6.16.3.4 reordered-payload decode itself — the PCW / non-PCW
  `WriteCodewordToSegment` trial loop inverted to recover the codeword
  bit positions — and threading `aacScalefactorDataResilienceFlag` /
  the rest of the triplet from `GASpecificConfig` through the ADTS /
  LATM decode drivers (today the ER body parse is reachable directly
  but the drivers always take the non-resilient branch). Both await an
  HCR-bearing conformance stream for bit-exact validation.
- LATM/LOAS transport framing (§1.7) — the `StreamMuxConfig()`,
  `AudioMuxElement()`, `PayloadLengthInfo()`, `PayloadMux()`,
  `LatmGetValue()`, `AudioSyncStream()` and `EPAudioSyncStream()`
  bitstream walkers are now implemented and tested (see the
  LATM / LOAS transport section above), with the `crcCheckSum`
  recomputed against the §1.8.4.5 `CRC8` generator in the `crc`
  module. The runtime `Decoder` LOAS entry point that routes the
  recovered `MuxPayload` raw-data-blocks into a `StreamDecoder` is now
  wired (`latm::LoasDecoder` + the `codec_decoder` carrier
  auto-detection above). What remains is the `EPMuxElement()` EP-tool
  payload de-interleave. ADTS
  `adts_error_check()` CRC validation is likewise still open: its
  region selection (192/128-bit element protection with the
  double-protection edge cases) plus its ISO/IEC 11172-3 §2.4.3.1 CRC
  reference are out of scope for the §1.8.4.5 `crc` module.

## License

MIT — see [LICENSE](./LICENSE).

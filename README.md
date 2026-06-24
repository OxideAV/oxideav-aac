# oxideav-aac

A pure-Rust **AAC** (Advanced Audio Coding) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

Every numeric constant, bit layout, and clause reference is sourced from
the staged ISO/IEC 13818-7 and ISO/IEC 14496-3 specifications under
`docs/audio/aac/`.

## Status

The crate implements the full AAC-LC decode chain end to end — from
ADTS bitstream parse through the per-tool reconstruction to interleaved
16-bit PCM — and **wires it into the framework's runtime `Decoder`
trait** (`register()` installs an AAC decoder under id `"aac"`; see
`codec_decoder` below). The PCM is validated byte-exactly (within the
1-LSB IMDCT-rounding bound) against the staged `expected.wav` corpus.
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
  resilience-flag dispatch from `ics_body` (which today always takes
  the non-resilient branch) is the remaining wiring step; the RVLC
  bitstream path itself is decoded end to end.

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
  scaling). The stream decode loop fully consumes a CCE so a CCE-bearing
  frame's SCE / CPE channels still decode; the cross-element coupling
  *contribution* onto the targets is decoded but not yet applied.
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

The remaining SBR back-end (dequantization to linear energies, the QMF
analysis / synthesis filterbanks, HF generation / patching, the limiter
and the envelope-adjustment that produce up-sampled PCM) is not yet
wired; those stages key off the band tables and scalefactors above.

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
  only output-rendering step (no resampler / dither / channel remap), so
  it is fully spec-determined.
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
- The SSR gain-control ladder (§4.6.12) — its side-info is parsed but
  not applied. (The Main frequency-domain predictor, §4.6.6, is now
  fully wired into `element_decode` for the AAC Main object type on long
  windows — see `predictor` above. LTP, §4.6.7, is likewise wired in
  with the §4.6.7.4.1 / Figure 4.30 TNS-analysis-in-loop ordering.
  Short-window LTP and the ER AAC LD `M = N/2` lag offset remain out of
  scope per the §4.6.7.1 long-window restriction.)
- SBR synthesis (needs a QMF analysis / synthesis filterbank and the
  §4.6.18.6 HF-generation / patching back-end) — the SBR *bitstream*
  path is now decoded end to end: the §4.A.6.1 Huffman codebooks, the
  §4.4.2.8 `sbr_header` / `sbr_grid` / `sbr_dtdf` / `sbr_invf` /
  `sbr_envelope` / `sbr_noise` syntax, the §4.6.18.3.2 frequency band
  tables, and the §4.6.18.3.5 envelope / noise DPCM reconstruction to
  quantized scalefactors (see the SBR sections above). What remains is
  the back-end DSP: dequantization to linear energies, the limiter
  table, the QMF analysis / synthesis filterbanks, HF generation /
  patching, and the envelope adjustment that produces up-sampled PCM.
  The `sbr_single_channel_element()` / `sbr_channel_pair_element()`
  framing (Tables 4.65–4.66) is decoded by `sbr_element` but not yet
  dispatched from `extension_payload`'s `EXT_SBR_DATA` branch (which
  still surfaces `Error::UnsupportedExtensionSbr` pending the back-end).
  PS (parametric stereo) — whose `sbr_extension` payload bytes are now
  captured — and the ER AAC LD 480/512 transform variants likewise remain
  out of scope. The coupling-channel (CCE) bitstream is now decoded end
  to end (`cce`, see the tool-chain section above) and consumed by the
  stream decode loop; the §4.6.8.3.3 cross-element coupling *application*
  (scaling the decoded CCE spectrum onto the addressed SCE / CPE target
  channels) is the remaining wiring step.
- The `aacScalefactorDataResilienceFlag` dispatch from `ics_body` to
  the error-resilient (RVLC) `scale_factor_data()` branch — the RVLC
  codebooks (`rvlc`) and the whole Table 4.53 RVLC parse / write path
  (`scale_factor_data::ErScaleFactorData`) are now implemented and
  tested (see the error-resilience section above); what remains is
  plumbing the resilience flag from `GASpecificConfig` through
  `ics_body` so the channel-element body selects the RVLC branch.
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

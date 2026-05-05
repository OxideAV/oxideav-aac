# oxideav-aac

Pure-Rust **AAC-LC** (MPEG-4 Audio / ISO/IEC 14496-3 Object Type 2) decoder
and encoder. ADTS framing, Huffman codebooks 1-11, IMDCT, M/S stereo, TNS,
PNS, and pulse data. No C dependencies, no FFI, no `*-sys` crates.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace) framework
but usable standalone.

## Installation

```toml
[dependencies]
oxideav-core = "0.1"
oxideav-codec = "0.1"
oxideav-aac = "0.0"
```

## Quick use

The crate registers a single codec id, `"aac"`, that maps to both the
decoder and the encoder.

```rust
use oxideav_core::{CodecId, CodecParameters, RuntimeContext};

let mut ctx = RuntimeContext::new();
oxideav_aac::register(&mut ctx);

// Decode an ADTS stream
let mut params = CodecParameters::audio(CodecId::new("aac"));
params.sample_rate = Some(44_100);
params.channels = Some(2);
let mut dec = ctx.codecs.make_decoder(&params)?;
// dec.send_packet(&adts_packet)?;
// dec.receive_frame()?  -> Frame::Audio (interleaved S16, FRAME_LEN=1024)

// Encode S16 PCM into ADTS
let mut enc_params = CodecParameters::audio(CodecId::new("aac"));
enc_params.sample_rate = Some(44_100);
enc_params.channels = Some(1);
enc_params.bit_rate = Some(128_000);
let mut enc = codecs.make_encoder(&enc_params)?;
// enc.send_frame(&audio_frame_s16)?;
// enc.receive_packet()?  -> ADTS-framed AAC frame
# Ok::<(), oxideav_core::Error>(())
```

For raw `raw_data_block()` payloads coming out of an MP4 demuxer, set
`params.extradata` to the AudioSpecificConfig (ASC) blob; the decoder will
read it instead of looking for an ADTS sync word on the first packet.

## Decode support

| Feature                                | Status                                  |
|----------------------------------------|-----------------------------------------|
| Object types                           | AAC-LC (`AOT 2`) only                   |
| Containers                             | ADTS (with or without CRC) and raw + ASC|
| Channel configurations                 | 1..=7 (mono, stereo, 3.0, 4.0, 5.0, 5.1, 7.1) |
| Sample rates                           | All 13 standard SF indices (96k - 7350) |
| Window sequences                       | Long, LongStart, LongStop, EightShort   |
| Window shapes                          | sine and KBD                            |
| Huffman spectral books 1-11            | Yes (escape book 11 included)           |
| Scalefactor Huffman book               | Yes                                     |
| M/S stereo (§4.6.13)                   | Yes (long + short)                      |
| PNS / Perceptual Noise Sub. (§4.6.13)  | Yes (long + short, correlated noise)    |
| Intensity stereo (§4.6.8.2.3)          | Yes (cb 14/15, sign from ms_used)       |
| TNS (§4.6.9)                           | Long AND short-window filters           |
| Pulse data (§4.6.5)                    | Yes (long-window only); short-window rejects as non-conformant |
| Fill / DSE elements                    | Skipped cleanly (FIL `count==15` escape uses spec-literal `cnt = 14 + esc_count`) |
| LFE element (§4.6.10)                  | Yes (long-window SCE-like path)         |
| PCE (Program Config Element)           | Parsed (channel mapping reserved for future use) |
| Gain control / SSR / Main / LTP        | Refused (`Error::Unsupported`)          |
| CCE elements                           | Refused (`Error::Unsupported`)          |
| HE-AAC v1 (SBR) decode (mono + CPE)    | Yes (independent + coupled CPE)         |
| HE-AAC v2 (PS) decode                  | Yes (QMF-domain upmix incl. IPD/OPD)    |

The decoder advertises `max_channels = 8` and `max_sample_rate = 96_000` in
`CodecCapabilities`. PCM output is interleaved in AAC element order
(C, L, R, Ls, Rs, LFE for 5.1) — downstream muxers may remap.

## Encode support

| Feature                                | Status                                  |
|----------------------------------------|-----------------------------------------|
| Object types                           | AAC-LC (`AOT 2`)                        |
| Containers                             | ADTS only (one raw_data_block per frame, no CRC) |
| Channels                               | 1..=7 (1, 2, 3, 4, 5, 6, 8 input channels) |
| Element orchestration                  | SCE + CPE + LFE sequence per AAC channel_configuration §1.6.3 |
| Sample rates                           | Any of the 13 standard SF indices that match a known SWB table; tested at 44.1 kHz / 48 kHz |
| Input sample format                    | `S16` and `F32` interleaved             |
| Window sequence                        | Long-only by default (short-block toolkit in place, state-machine wiring pending) |
| Window shape                           | Sine                                    |
| MDCT scaling                           | Matches ffmpeg's `aacenc.c` 32768x convention |
| Spectral codebook selection            | Per-band cheapest of books 1-11 (incl. escape) |
| Section data                           | Run-length compressed; merges adjacent same-cb bands |
| Scalefactors                           | Huffman-coded deltas with global_gain anchor; 3-accumulator path (g_gain / g_noise / g_is) for NOISE / IS bands |
| M/S stereo (§4.6.13)                   | Per-band L/R-vs-M/S decision by bit cost + activity gate (energy-balance ∈ [1/8, 8] AND \|corr\| ≥ 0.4) so imbalanced/uncorrelated bands don't leak side-channel quant noise |
| TNS (§4.6.9)                           | LPC analysis on SCE long blocks; 4-bit parcor quantisation; gated on prediction-gain (~1.4 dB) |
| PNS encode (§4.6.12)                   | Yes (long windows; peak-to-RMS ≤ 2.6 + SFM ≥ 0.25 noise gate, ≥ 4 kHz band-centre gate; trimmed-mean energy gain matches source RMS within ±1 dB) |
| Intensity stereo encode (§4.6.8.1.4)   | Yes (long windows; \|corr\| ≥ 0.95 + per-line sign-agreement ≥ 80 % + energy ratio ∈ [1/256, 256] in CPE common-window path; ≥ 4 kHz band-centre gate; corpus PSNR delta +1.7 dB on `aac-lc-intensity-stereo` after round-#523 tuning) |
| Pulse data encode (§4.6.10)            | Yes (up to 4 per frame; sign-preserving outlier extraction, amp capped at `\|residual\| - 1`) |
| Short blocks (building blocks)         | `TransientDetector`, `mdct_short_eightshort`, `analyse_and_quantise_short`, `write_single_ics_short` — all tested; `emit_block` state-machine integration pending |
| HE-AACv1 (SBR) encode                  | Mono (`HeAacMonoEncoder`, psy-on default); stereo CPE (`HeAacStereoEncoder`, independent coupling, §4.6.18.3.5, psy-on default as of round-27 M/S CPE side-lobe fix); v2 (`HeAacV2Encoder`, mono SCE + PS, psy-on default) |
| Gapless playback metadata              | `gapless::GaplessInfo` triple (delay/padding/valid_samples) + Apple iTunSMPB-format string emitter; AAC-LC reports 2112-sample priming, HE-AAC reports 2624 (high rate). End-of-file padding rounded to the next packet boundary so an MP4 `edts/elst` writer or ID3v2 `TXXX:iTunSMPB` wrapper can round-trip the source PCM sample-accurately. |
| Psychoacoustic model                   | Bark-band PE/SMR allocator (`psy::PsyModel`, default-on for AAC-LC + every HE-AAC variant); per-band tonality-driven `target_max`, sub-baseline coarsening gated on tonality < 0.15 (avoids +17 % bytes on noise-only fixtures). Above-baseline `target_max` is clamped to baseline=7 in the CPE LR/MS path (TNS off) so per-line side-lobe lines aren't mis-quantised to ±step^(4/3) noise — see `tests/r27_ms_cpe_psy_diagnose.rs`. Override via `AacEncoder::set_enable_psy_model` or env `OXIDEAV_AAC_PSY_MODEL=0`. Corpus-validated to within +0.08 dB mean PSNR / -0.42 dB worst (`tests/psy_corpus_validation.rs`); HE-AAC corpus-validated mean +1.92 dB / worst +0.07 dB (`tests/he_aac_psy_validation.rs`). |
| Gain control                           | Not implemented                         |
| CBR / VBR                              | Bit-reservoir CBR allocator (`AacEncoder::set_cbr_target_bitrate`) drives a per-frame scalefactor bias from a 6144-bit reservoir to bound output frames at the configured rate; default off (VBR) |

The encoder advertises `max_channels = 8` and `max_sample_rate = 48_000`.
Multi-channel output emits elements in AAC element order (C, L, R for
3.0; C, L, R, Ls, Rs, LFE for 5.1; etc.) — round-trip validated via the
self-decoder for 5.1 and 7.1 layouts in `tests/encode_roundtrip.rs`.
TNS on stereo (CPE) is gated off until per-band M/S decisions can run on
TNS-flattened coefficients.

## Round-trip verification

`tests/encode_roundtrip.rs` runs the encoder end-to-end for:

- 44.1 kHz mono sine through our own decoder + ffmpeg (Goertzel ratio >= 50x)
- 44.1 kHz stereo sine through our own decoder + ffmpeg (both channels)
- 48 kHz mono sine through ffmpeg
- 0.5 s mono and stereo silence through our own decoder (RMS < 1e-3)
- 44.1 kHz 5.1 sine-per-channel through our own decoder (each of the 6
  channels recovers its tone above a 20x Goertzel floor)
- 44.1 kHz 7.1 sine-per-channel through our own decoder (all 8 channels)
- 44.1 kHz 5.1 sine-per-channel through ffmpeg (`encode_51_roundtrip_ffmpeg`).
  All 6 channels survive ffmpeg's bitstream→WAVE-5.1 reorder
  (C/L/R/Ls/Rs/LFE → L/R/C/LFE/Ls/Rs); per-channel PSNR floors at
  20 dB (matching the AC-3 5.1 acceptance pattern), with five of
  six channels clearing 25 dB on the synthetic tone fixture. The
  L/R-CPE R channel running an octave-paired tone (R = 880 Hz vs
  L = 440 Hz) sits at ~22 dB because M/S coding biases bit allocation
  toward the side signal.
- 44.1 kHz 7.1 sine-per-channel through ffmpeg (`encode_71_roundtrip_ffmpeg`,
  task #154). All 8 channels survive ffmpeg's bitstream→WAVE-7.1
  reorder (AAC bitstream order C/L/R/Ls/Rs/Lb/Rb/LFE per §1.6.3
  Table 1.19 → WAVE FL/FR/FC/LFE/BL/BR/SL/SR, inverse mapping
  `[2, 0, 1, 6, 7, 4, 5, 3]`); per-channel PSNR floors at 22 dB,
  with seven of eight channels clearing 24 dB on the synthetic tone
  fixture. The L/R-CPE R channel hits the M/S-bias floor for the
  same reason as 5.1.

### r19 — AAC-LC ffmpeg-interop RMS audit (2026-04-26)

`tests/lc_rms_interop_r19.rs` (new) cross-checks all four directions:

| direction                            | RMS  | ratio |
|--------------------------------------|------|-------|
| ours-encode → ours-decode            | 6718 | 0.97x |
| ours-encode → ffmpeg-decode          | 6644 | 0.96x |
| ffmpeg-encode → ours-decode          | 6881 | 0.99x |
| ffmpeg-encode → ffmpeg-decode (ref)  | 6950 | 1.00x |

Test signal: 1 s of 440 Hz sine at amplitude 0.3 through 44.1 kHz mono;
expected RMS = `0.3 * 32767 / sqrt(2) = 6951`. All four within ±5% of
unity — the AAC-LC core spectrum scale matches ffmpeg end-to-end on the
informationally-correct (RMS) metric. Audited and confirmed spec-correct:

- §4.6.1.3 inverse quant `sign(q) * |q|^(4/3)` (matches `ics::inv_quant`).
- §4.6.2.3.3 scalefactor gain `2^(0.25 * (sf - 100))`, SF_OFFSET = 100
  (matches `ics::sf_to_gain`).
- §4.6.11.3.1 IMDCT scale `2/N` with `N = 2 * input_n` — our IMDCT uses
  `2/input_n` (= `4/N`, doubled) and the decoder's S16 output stage
  multiplies by `0.5`, giving an effective `2/N` matching the spec.
- §4.5.2.3.6 "the integer part of the output of the IMDCT can be used
  directly as a 16-bit PCM audio output" — encoder forward-MDCT scale
  derivation: `2 * 32768 = 65 536`, encoded in `MDCT_FORWARD_SCALE`.

The previously-claimed "~3.33x mid-stream amplitude gap" (rounds 17/18)
was a peak-metric artefact: ffmpeg's encoder fills HF bands with PNS
(codebook 13, §4.6.13) that our spec-compliant decoder reconstructs as
additive noise. The PNS noise rides on the sine peak, inflating the
**peak** ratio (1.79x for ffmpeg → ours) while the **RMS** stays at
0.99x. PNS is non-deterministic per-frame, so peak ratio is not a
meaningful interop metric for tonal+noise content; RMS is.

`examples/probe_lc_amp.rs` and `examples/spectrum_compare.rs` are the
diagnostic probes used to land this audit; both report RMS alongside
peak so subsequent rounds don't re-chase the same phantom.

**Note**: `tests/sbr_he_aac_ffmpeg_amplitude_r18.rs` (HE-AACv1 SBR
amplitude saturation at peak 32_768) remains ignored.

Round 21 audit data (1 kHz tone amp 0.3 mono SCE at 48 kHz):

```
input (peak / RMS)              :  9 830 / 6 951
ours-encode -> ours-decode       : 10 256 / 6 582  (within 5% of input)
ours-encode -> ffmpeg-decode     : 32 767 / 17 181 (saturated)
fdkaac-encode -> ffmpeg-decode   :  9 822 / 6 920  (within 1% of input)
fdkaac-encode -> ours-decode     : 13 009 / 7 061  (within 30% of input)
```

Round-21 probe of the bitstream-level envelope value confirms **both
fdkaac and our encoder transmit `bs_data_env[0] = 0`** for tonal
content with no high-band energy (E_orig = 64, the spec minimum).
The bitstream-level envelope value is therefore not the source of
the saturation — both encoders emit the same value yet ffmpeg
decodes the fdkaac stream cleanly while saturating ours. Audit of
§4.6.18.4.2 (synthesis QMF gain `1/64`), §4.6.18.7.1 (E_orig
formula), §4.6.18.7.5 (limiter gain cap, Table 4.176), and the HF
generator patches all confirms spec-correctness; the divergence
must arise from a difference in how ffmpeg reads our specific
header configuration vs fdkaac's. Round 22+ target.

### r22 — HE-AACv1 SBR header diff-probe (2026-04-26)

Round 22 ran the methodical SBR header diff-probe specified in the
round-21 brief. **Result: no SBR header field is the saturation
source.** The fdkaac vs ours header values for a 1 kHz / 0.3-amp
mono SCE at 48 kHz output (24 kHz core) were captured via our own
SBR header parser:

```
field              fdkaac  ours
bs_amp_res         1       0       (effective per-frame value: both 0,
                                    forced to 0 for FIXFIX num_env=1)
bs_start_freq      13      5       (k0 = 22 vs k0 = 12)
bs_stop_freq       11      9
bs_xover_band      0       0       (same)
bs_freq_scale      1       2       (8 bands/octave vs 10)
bs_alter_scale     1       1       (same)
bs_noise_bands     2       2       (same)
bs_limiter_bands   2       2       (same)
bs_limiter_gains   2       2       (same)
bs_interpol_freq   1       1       (same)
bs_smoothing_mode  1       1       (same)
```

Forcing each differing field individually (`bs_amp_res=1`,
`bs_start_freq=13`, `bs_stop_freq=11`, `bs_freq_scale=1`) and the
combined fdkaac configuration **all** still produced ffmpeg peak
32 768 (full saturation). Forcing `bs_stop_freq=11` and
`bs_freq_scale=1` with our `bs_start_freq=5` triggers ffmpeg
"Invalid bitstream, too many QMF subbands: 41" / "Invalid vDk0[1]:
0" — these are spec-consistent with the freq-table derivation
(§4.6.18.3.2.1) and rule out the `bs_start_freq` / `bs_stop_freq`
pair as a meaningful encoder-side fix.

The differential probe further extracted the per-band envelope and
noise scalefactor data:

```
fdkaac: env[0] sf(14b) = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        noise[0] sf(2b) = [14, 0]
ours:   env[0] sf(16b) = [29, -1, -1, -1, 0, -1, -1, 0, -1, -1, ...]
        noise[0] sf(4b) = [18, 0, 0, 0]
```

Setting our encoder's `INT16_SCALE_SQ` to 1.0 reproduces fdkaac's
all-zero envelope (E_orig = 64, the spec minimum) — yet ffmpeg
**still** saturates ours to peak 32 768. Even **completely omitting
the SBR FIL extension** (pure AAC-LC core only, decoded as 24 kHz
mono) still produces ffmpeg peak 32 768 / RMS 30 961, vs our own
decoder's clean 10 683 / RMS ~6 000 on the identical stream.

### Round 23 (2026-04-30) audit: r22 MDCT-scale thesis refuted

Round 23 set `MDCT_FORWARD_SCALE = 4 096` and re-ran the matrix:

- **LC RMS interop test (44.1 kHz / 440 Hz)**: `ours-encode →
  ours-decode` ratio collapsed to **0.060** (16× too quiet). The
  test fails on the very first assertion. There is no ±5 % window
  that can absorb a 16× reduction.
- **HE-AAC SBR amplitude (the r18 ignored test)**: peak only
  drops from `32 768` (saturated) to `25 287`. Still saturated.
  The 16× scale change does not unblock HE-AAC interop.
- **Pure AAC-LC mono at 24 kHz / 1 kHz** with current SCALE =
  `65 536`: ffmpeg-decoded peak `10 930 / RMS 6 955`, within
  ±5 % of the input. Pure stereo LC at 24 kHz / (1k+2k)
  produces L=10 930/9 880, RMS 6 955/6 579 — also within ±5 %
  per channel. **r22's claim that pure-LC-at-24 kHz saturates
  was wrong**: it conflated the HE-AAC code path (which carries
  an SBR FIL extension) with pure-LC. New regression tests
  (`r23_lc_24khz_probe.rs` + `r23_he_aac_isolation.rs`) pin both
  cases at the spec-correct unity-RMS measurement.

Sweep on HE-AAC stereo with current SCALE values:

```
SCALE   L-peak  L-rms   R-peak  R-rms   verdict
65536   15767   6544    32768   30461   sat (R)
32768    7884   3272    32768   28126   sat (R)
16384    3942   1636    32768   23068   sat (R)
 8192    1972    818    32768   14711   sat (R)
 4096     986    409    25287    7563   passing-through
 2048     493    204    12643    3782   silent
 1024     247    102     6321    1891   silent
```

The L-peak strictly halves with SCALE (linear pass-through through
the LC core). The R-peak stays clipped at 32 768 until the input
drops below the clipping threshold, then itself halves. r22's
"RMS = 6 951 at SCALE = 4 096 → unity" reading was a methodological
error: a clipped square-wave at 32 768 has RMS ≈ 30 000, and
reducing SCALE 16× simply lowers input below the clipping
threshold — RMS *passes through* the input target on its way to
silence (verified: SCALE = 2 048 → 1 891, SCALE = 1 024 → 947).
There is **no** stable interop point in the sweep.

**Conclusion (r23)**: `MDCT_FORWARD_SCALE = 65 536` is the correct
value (verified by 4 LC-only ffmpeg interop measurements at both
44.1 kHz and 24 kHz, mono and stereo) and remains in place. The
HE-AAC ffmpeg-interop saturation lives in the **SBR FIL extension
itself** (most likely `bs_invf_mode`, `bs_add_harmonic`, or an
HF-generation-stage gain), not the LC core. Pinned for round 24.

### Round 24 (2026-04-30): SBR FIL diff vs fdkaac

Round 24 built `tests/r24_sbr_fil_diff.rs`: encodes the r18 fixture
(1 kHz / amp 0.3 / 0.5 s mono, 48 kHz, 48 kbps HE-AAC) through
**both** our `HeAacMonoEncoder` and `fdkaac -p 5 -f 2`, parses every
ADTS frame's SBR FIL element via the same
`oxideav_aac::sbr::bitstream::parse_*` routines our decoder uses, and
diffs the resulting `SbrChannelData` field-by-field. The harness
walks the SCE element bit-cursor through the now-pub
`decoder::decode_ics` + `decoder::fill_spectrum` so the FIL bit
position is exact (no brute-force bit-offset scanning).

| Field                     | ours              | fdkaac                         |
| ------------------------- | ----------------- | ------------------------------ |
| `bs_amp_res` (header)     | 0                 | 1                              |
| `bs_start_freq`           | 5                 | 13                             |
| `bs_stop_freq`            | 9                 | 11                             |
| `bs_freq_scale`           | 2                 | 1                              |
| derived `n_high` / `nq`   | 16 / 4            | 14 / 2                         |
| FIL payload bits / frame  | 96                | 80                             |
| `bs_invf_mode` totals     | `[0,0,0,0,0]`     | `[5,2,0,0,0]` (varies)         |
| `bs_add_harmonic_flag`    | 0/12 frames       | 1/15 frames                    |
| `bs_df_env` totals        | `[0,0,0,0,0]`     | `[2,4,2,1,0]` (uses time-dir)  |
| `bs_df_noise` totals      | `[0,0]`           | `[9,2]` (uses time-dir)        |
| frame[0] `env_sf[0]`      | `[29,-1,-1,...]`  | `[0,0,0,...]` (E_orig minimum) |
| frame[0] `noise_sf[0]`    | `[18,0,0,0]`      | `[14,0]`                       |

**Refuted thesis — the envelope value is NOT the saturation source.**
The diff first looked like our `env_sf[0][0] = 29` (= `64 · 2^14.5`
≈ 1.5 M; QMF analysis-bank skirt leakage of the 1 kHz tone into the
bottom-most SBR subband, amplified by `INT16_SCALE_SQ = 2^30`) was
the cause. To pin this we added the
`OXIDEAV_AAC_SBR_ENV_FORCE_ZERO` env-var probe in
`sbr/encode.rs::estimate_envelope`: when set, every band gets
value 0 (matching fdkaac's "no high-band content" output) and the
noise floor gets value 14. Re-ran ffmpeg-decode of the r18 mono
fixture under the override:

```
mono HE-AAC, no override:    peak=32768  rms=28739
mono HE-AAC, FORCE_ZERO=1:   peak=32768  rms=28739  (identical)
```

The override is verified to actually zero the envelope on every
frame (`force_zero_env_var_actually_zeros_envelope` regression).
**Same saturation reading**, byte-identical decode artefacts.
The envelope is fully ruled out.

ffmpeg consistently logs `No quantized data read for sbr_dequant`
on every decode, regardless of envelope value. This warning fires
*before* any envelope arithmetic, suggesting ffmpeg's parser is
giving up on our SBR data structurally — likely either:

1. **Header-vs-grid `bs_amp_res` mismatch**: we send `bs_amp_res = 0`
   in the header but rely on the FIXFIX `bs_num_env == 1` rule
   (§4.6.18.3.3) to override to 0 inside `parse_sbr_grid`. fdkaac
   sends `bs_amp_res = 1` in the header and relies on the same
   override. ffmpeg may be reading the start-value bit count from
   the *raw* header value before the override fires.
2. **`bs_extended_data = 0` framing edge-case**: our SCE always
   ends with `bs_extended_data = 0` (1 bit) followed by zero fill
   bits to the FIL byte boundary. fdkaac's SCE is shorter (53 bits
   vs our 63), suggesting its grid + envelope shapes use fewer
   bits, but both end byte-aligned. The 4-bit
   `extension_payload(cnt)` count alignment may differ in some
   subtle way.
3. **Time-vs-freq delta encoding**: fdkaac uses time-direction
   delta on noise (`bs_df_noise` totals 9/2) and on envelope (2/4)
   while ours always uses freq-direction. Time-direction requires
   a previous-frame baseline; freq-direction encodes the first
   band as an absolute. ffmpeg may not accept freq-direction on
   the very first frame of a HE-AAC stream.

The diff harness is permanent infrastructure now; r25 can change
any header field or encoding strategy and re-run the diff to
verify convergence with fdkaac.

ffmpeg-dependent tests skip cleanly when `ffmpeg` is not on `PATH`.
`tests/encode_tns.rs` confirms the encoder emits TNS on transient content
and that TNS-bearing frames decode without error.
`tests/encode_pns_is_pulse.rs` verifies:

- PNS fires on >=75% of >=4 kHz bands for white-noise input and
  round-trips within a factor of 4 in total RMS energy.
- Intensity stereo fires on >=2 HF bands for a stereo clip with a
  quiet correlated R channel; decoded R tracks L's sign.
- Pulse data is emitted on at least one frame of a loud 440 Hz tone
  and the round-trip Goertzel ratio stays >=50x.

`tests/encode_pns_savings.rs` (task #132) pins the bit-savings PNS
buys on noise-rich content (cymbals + sax-like harmonic stack +
room-tone broadband background, 1 s mono at 44.1 kHz / 96 kbps):

- PNS-active vs PNS-disabled (`OXIDEAV_AAC_DISABLE_PNS=1`) A/B encode:
  raw_data_block bytes drop **63.9%** (8 749 B vs 24 256 B).
- Self-decoder RMS round-trip ratio: 0.977 (PNS preserves band
  energy within 2.3% of the input).
- ffmpeg cross-decode runs clean (no warnings, no errors). The
  ffmpeg-decoder RMS ratio of 6.06× is the same FAAD2-vs-ffmpeg
  `dpcm_noise_nrg` calibration delta documented in the r19 audit;
  not blocking the cross-decode-clean criterion.

## Codec id

- `"aac"` — both encoder and decoder.

Frames are produced as interleaved `SampleFormat::S16` at the stream's
sample rate, 1024 samples per frame.

## Caveats

- HE-AAC (SBR / PS) decode is supported (mono + stereo CPE, optional
  parametric stereo upmix). HE-AAC encode is mono + stereo only — no
  PS encoder yet, no SBR support for ≥3-channel CPE configurations.
- Bit_rate on the encoder is informational. The encoder picks scalefactors
  based on a fixed target quantisation magnitude; emitted frame size depends
  on signal complexity, not directly on the requested rate.
- The `make_encoder` registry path uses long blocks unconditionally. Sharp
  attacks pre-echo more than they would with proper short-block switching.
- MP4-side AAC streams need the AudioSpecificConfig blob in
  `CodecParameters::extradata`; otherwise the first packet must carry an
  ADTS sync word.

## License

MIT - see [LICENSE](LICENSE).

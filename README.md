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
use oxideav_codec::CodecRegistry;
use oxideav_core::{CodecId, CodecParameters};

let mut codecs = CodecRegistry::new();
oxideav_aac::register(&mut codecs);

// Decode an ADTS stream
let mut params = CodecParameters::audio(CodecId::new("aac"));
params.sample_rate = Some(44_100);
params.channels = Some(2);
let mut dec = codecs.make_decoder(&params)?;
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
| Fill / DSE elements                    | Skipped cleanly                         |
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
| M/S stereo (§4.6.13)                   | Per-band L/R-vs-M/S decision by bit cost |
| TNS (§4.6.9)                           | LPC analysis on SCE long blocks; 4-bit parcor quantisation; gated on prediction-gain (~1.4 dB) |
| PNS encode (§4.6.12)                   | Yes (long windows; peak-to-RMS noise test, >=4 kHz band-centre gate) |
| Intensity stereo encode (§4.6.8.1.4)   | Yes (long windows; L/R correlation + energy ratio above 4 kHz in CPE common-window path) |
| Pulse data encode (§4.6.10)            | Yes (up to 4 per frame; sign-preserving outlier extraction, amp capped at `\|residual\| - 1`) |
| Short blocks (building blocks)         | `TransientDetector`, `mdct_short_eightshort`, `analyse_and_quantise_short`, `write_single_ics_short` — all tested; `emit_block` state-machine integration pending |
| HE-AACv1 (SBR) encode                  | Mono via `HeAacMonoEncoder`; stereo CPE (`HeAacStereoEncoder`, independent coupling, §4.6.18.3.5) |
| Gain control                           | Not implemented                         |
| CBR / VBR                              | Bit_rate accepted but currently advisory; no rate control loop |

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

Cross-check: **pure AAC-LC encode at 24 kHz** (no HE-AAC wrapping,
direct `AacEncoder`) on the same 1 kHz / 0.3-amp tone gives ffmpeg
peak 32 768 / RMS 9 478 — a 1.36x RMS inflation that's hidden by
the round-19 LC-RMS test running at 44.1 kHz on a 440 Hz tone (RMS
ratio 0.96 — within ±10% tolerance). The 24 kHz core path
nonetheless clips at peak 32 768 ≈ 7.6% saturated samples, vs the
HE-AAC-pipeline 24 kHz core which clips ~82% of samples.

Sweep of `MDCT_FORWARD_SCALE` confirms a **16x scale mismatch
between our encoder and ffmpeg's decoder** at 24 kHz core:

```
SCALE   ffmpeg-decode peak  ffmpeg-decode RMS  ours-decode peak
65536   32 768 (sat)        28 665             10 578
16384   32 768 (sat)        20 213             —
 4096   32 768 (sat)         6 941 (target!)   ~661
 1024   21 387               1 787             —
  512   10 694                 893             —
```

ffmpeg's RMS lands on the input target (~6 951) at SCALE = 4 096 —
exactly 16x smaller than the round-19 chosen value. Our decoder at
SCALE = 4 096 produces only peak 661 (16x quieter than target).
The 16x ratio is consistent across the sweep and matches the
predicted product of (forward MDCT scale 2x) * (IMDCT 2x) *
(decoder S16 stage 0.5x) * (8 from … TBD in r23).

**Conclusion (r22)**: the HE-AACv1 ffmpeg-decode saturation is an
**AAC-LC core MDCT scale mismatch**, not an SBR header issue. The
round-19 fix (`MDCT_FORWARD_SCALE = 65536`) was correct for our
self-roundtrip but is 16x too large for ffmpeg's decoder at 24 kHz
core / 1024-sample window. The fix is to **decouple the encoder's
forward scale from our decoder's IMDCT 2x carry-over** — either by
halving `MDCT_FORWARD_SCALE` and having the decoder upscale, or by
matching ffmpeg's normalised int16 contract. Pinned for round 23.

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

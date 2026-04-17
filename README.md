# oxideav-aac

Pure-Rust **AAC-LC** (MPEG-4 Audio / ISO/IEC 14496-3 Object Type 2) decoder
and encoder. ADTS framing, Huffman codebooks 1-11, IMDCT, M/S stereo, TNS,
PNS, and pulse data. No C dependencies, no FFI, no `*-sys` crates.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace) framework
but usable standalone.

## Installation

```toml
[dependencies]
oxideav-core = "0.0"
oxideav-codec = "0.0"
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
| Channel configurations                 | mono (1) and stereo (2)                 |
| Sample rates                           | All 13 standard SF indices (96k - 7350) |
| Window sequences                       | Long, LongStart, LongStop, EightShort   |
| Window shapes                          | sine and KBD                            |
| Huffman spectral books 1-11            | Yes (escape book 11 included)           |
| Scalefactor Huffman book               | Yes                                     |
| M/S stereo (§4.6.13)                   | Yes (long + short)                      |
| PNS / Perceptual Noise Sub. (§4.6.13)  | Yes (long + short, correlated noise)    |
| TNS (§4.6.9)                           | Long-window filters; short-window parsed but bypassed |
| Pulse data (§4.6.5)                    | Yes (long-window only, per spec)        |
| Fill / DSE elements                    | Skipped cleanly                         |
| Intensity stereo (§4.6.14)             | Bands flagged IS leave zeros (not decoded) |
| Gain control / SSR / Main / LTP        | Refused (`Error::Unsupported`)          |
| LFE / CCE / PCE elements               | Refused (`Error::Unsupported`)          |
| HE-AAC v1 (SBR) / v2 (PS)              | Refused at ASC parse and on FIL ext     |
| Multi-channel (5.1, 7.1)               | Not supported                           |

The decoder advertises `max_channels = 2` and `max_sample_rate = 96_000` in
`CodecCapabilities`.

## Encode support

| Feature                                | Status                                  |
|----------------------------------------|-----------------------------------------|
| Object types                           | AAC-LC (`AOT 2`)                        |
| Containers                             | ADTS only (one raw_data_block per frame, no CRC) |
| Channels                               | mono (SCE) and stereo (CPE common_window) |
| Sample rates                           | Any of the 13 standard SF indices that match a known SWB table; tested at 44.1 kHz / 48 kHz |
| Input sample format                    | `S16` and `F32` interleaved             |
| Window sequence                        | Long-only (no transient detector wired) |
| Window shape                           | Sine                                    |
| MDCT scaling                           | Matches ffmpeg's `aacenc.c` 32768x convention |
| Spectral codebook selection            | Per-band cheapest of books 1-11 (incl. escape) |
| Section data                           | Run-length compressed; merges adjacent same-cb bands |
| Scalefactors                           | Huffman-coded deltas with global_gain anchor |
| M/S stereo (§4.6.13)                   | Per-band L/R-vs-M/S decision by bit cost |
| TNS (§4.6.9)                           | LPC analysis on SCE long blocks; 4-bit parcor quantisation; gated on prediction-gain (~1.4 dB) |
| PNS encode                             | Not implemented                         |
| Intensity stereo encode                | Not implemented                         |
| Pulse data encode                      | Not implemented (decoder accepts it)    |
| Short blocks / transient detection     | Not implemented                         |
| Gain control                           | Not implemented                         |
| Multi-channel                          | Not supported                           |
| CBR / VBR                              | Bit_rate accepted but currently advisory; no rate control loop |

The encoder advertises `max_channels = 2` and `max_sample_rate = 48_000`.
TNS on stereo (CPE) is gated off until per-band M/S decisions can run on
TNS-flattened coefficients.

## Round-trip verification

`tests/encode_roundtrip.rs` runs the encoder end-to-end for:

- 44.1 kHz mono sine through our own decoder + ffmpeg (Goertzel ratio >= 50x)
- 44.1 kHz stereo sine through our own decoder + ffmpeg (both channels)
- 48 kHz mono sine through ffmpeg
- 0.5 s mono and stereo silence through our own decoder (RMS < 1e-3)

ffmpeg-dependent tests skip cleanly when `ffmpeg` is not on `PATH`.
`tests/encode_tns.rs` confirms the encoder emits TNS on transient content
and that TNS-bearing frames decode without error.

## Codec id

- `"aac"` — both encoder and decoder.

Frames are produced as interleaved `SampleFormat::S16` at the stream's
sample rate, 1024 samples per frame.

## Caveats

- HE-AAC (SBR / PS) is detected at parse time and rejected. There is no
  high-frequency reconstruction path.
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

# oxideav-aac

Pure-Rust AAC (AAC-LC) codec for oxideav — ADTS parser, AudioSpecificConfig,
decoder, and encoder.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace) framework — a
100% pure Rust media transcoding and streaming stack. No C libraries, no FFI
wrappers, no `*-sys` crates.

## Status

* Decoder: AAC-LC / long + short blocks / M/S stereo / PNS / TNS (long).
* Encoder: AAC-LC CBR / long blocks / M/S stereo / ADTS framing.
* Encoder TNS: LPC-based analysis on each long window, 4-bit parcor
  quantisation, forward filter applied to MDCT coefficients before
  quantisation. Gated on prediction-gain threshold (~1.4 dB) so pure tones
  and near-silent frames skip TNS. Applied to SCE (mono) only in the current
  cut; CPE (stereo) TNS is disabled until per-band M/S selection on
  TNS-flattened coefficients is wired up.

## Usage

```toml
[dependencies]
oxideav-aac = "0.0"
```

## License

MIT — see [LICENSE](LICENSE).

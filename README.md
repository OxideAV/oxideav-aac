# oxideav-aac

A pure-Rust **AAC** (Advanced Audio Coding) codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Orphan-rebuild scaffold (2026-05-24).** The prior implementation was
retired under the workspace
[clean-room policy](https://github.com/OxideAV/oxideav-workspace/blob/master/docs/IMPLEMENTOR_ROUND.md):
comments in the encoder source described matching an external
reference encoder's behaviour by citing a specific source file of
that implementation. The clean-room policy forbids consulting any
external implementation's source for any reason, regardless of
licensing or technical merit, so the provenance could not be defended.
Master history was fully erased per the Hat-3 cold-enforcement
procedure.

The implementation will be re-built from scratch against the staged
ISO/IEC 14496-3 / 13818-7 specifications (numeric tables and decode
behaviour read only from the standard) in a future clean-room round.

## License

MIT — see [LICENSE](./LICENSE).

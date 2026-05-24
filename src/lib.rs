//! # oxideav-aac
//!
//! Pure-Rust AAC (Advanced Audio Coding) parsing — currently **Phase 1**
//! of the post-r111 orphan-rebuild lineage. Decode and encode bodies are
//! *not* wired up yet; this crate's public surface is limited to:
//!
//! * The [`adts`] module — ISO/IEC 13818-7 §1.A.2 *Audio Data Transport
//!   Stream* fixed-header parser (sync, profile, sampling-frequency
//!   index, channel configuration, frame length, raw-data-block count,
//!   CRC presence flag).
//! * The [`raw_data_block`] module — ISO/IEC 14496-3 §4.4.2.1 syntactic
//!   *raw_data_block()* walker that visits each `id_syn_ele` in order
//!   and stops cleanly at `END (0b111)`. Per-element bodies for
//!   SCE / CPE / CCE / LFE are **not** parsed yet — the walker emits an
//!   element-header event and the consumer is responsible for
//!   advancing the bit-reader past the body (subsequent rounds will
//!   internalise this).
//!
//! Public API surface that requires a decoder or encoder body still
//! returns [`Error::NotImplemented`]; the syntactic skeleton lives
//! entirely in the modules above.
//!
//! ## Provenance
//!
//! No external implementation (FFmpeg's `libav*` family, FDK-AAC,
//! FAAD, Nero AAC, libfaac, …) was consulted at any stage. Every
//! numeric
//! constant, bit layout, and clause reference in this crate is sourced
//! from the staged ISO/IEC 13818-7 and ISO/IEC 14496-3 PDFs under
//! `docs/audio/aac/`. The fixture descriptions in
//! `docs/audio/aac/aac-fixtures-and-traces.md` were consulted as a
//! cross-reference against the spec wording.
//!
//! ## Status (Phase 1)
//!
//! * ADTS fixed header parsing: **complete** (sync + 7-byte body).
//! * ADTS CRC validation: deferred; the parser surfaces the
//!   `protection_absent` flag but does not validate the trailing
//!   16-bit CRC when present.
//! * `raw_data_block()` walker: **iterates `id_syn_ele` and stops at
//!   `END`**; element-body skipping is implemented for `FIL` /
//!   `DSE` only. SCE / CPE / CCE / LFE / PCE bodies are deferred to
//!   subsequent rounds.

#![warn(missing_debug_implementations)]
#![warn(missing_docs)]

use oxideav_core::RuntimeContext;

pub mod adts;
pub mod raw_data_block;

mod error;

pub use error::Error;

/// Result alias used throughout the crate.
pub type Result<T> = core::result::Result<T, Error>;

/// Codec-registry entry point. The Phase 1 skeleton does **not** wire
/// a [`Decoder`](oxideav_core::Decoder) or
/// [`Encoder`](oxideav_core::Encoder) into the runtime context — those
/// arrive in subsequent rounds once decode / encode bodies land.
pub fn register(_ctx: &mut RuntimeContext) {}

oxideav_core::register!("aac", register);

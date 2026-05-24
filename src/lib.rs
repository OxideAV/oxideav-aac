//! # oxideav-aac
//!
//! **Status:** orphan-rebuild scaffold (reset 2026-05-24).
//!
//! The prior implementation was retired under the workspace clean-room
//! policy: comments in the encoder source described matching an
//! external reference encoder's behaviour by citing a specific source
//! file of that implementation. The clean-room policy forbids
//! consulting any external implementation's source for any reason, so
//! the provenance could not be defended. The crate will be
//! re-implemented from scratch against the staged ISO/IEC 14496-3 /
//! 13818-7 specification in a future clean-room round.
//!
//! Every public API currently returns [`Error::NotImplemented`].

#![warn(missing_debug_implementations)]

use oxideav_core::RuntimeContext;

/// Crate-local error type. Until the clean-room rebuild lands every
/// public API path returns [`Error::NotImplemented`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The crate has been reset to a scaffold pending clean-room
    /// rebuild; no decoder or encoder functionality is wired up yet.
    NotImplemented,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "oxideav-aac: orphan-rebuild scaffold — no codec wired up"
        )
    }
}

impl std::error::Error for Error {}

/// No-op codec registration — the orphan-rebuild scaffold registers
/// nothing into the runtime context.
pub fn register(_ctx: &mut RuntimeContext) {}

oxideav_core::register!("aac", register);

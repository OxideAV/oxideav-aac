//! End-to-end test of the linkme distributed-slice auto-registration.
//!
//! Linking `oxideav-aac` into a binary should populate
//! `oxideav_core::REGISTRARS` with one entry — the `register` fn this
//! crate registers via the `oxideav_core::register!` macro. Calling
//! `RuntimeContext::with_all_features()` should then install the AAC
//! decoder factory without the test ever calling `oxideav_aac::register`
//! explicitly.

use oxideav_core::{CodecId, CodecParameters, RuntimeContext};

// Force-link the parent lib so the linkme static doesn't get DCE'd
// out of the integration-test binary. Without this reference Cargo
// links oxideav_aac in but rustc may strip every static the test
// binary doesn't directly observe — we observe through the slice
// walker, which is invisible to rustc's reachability analysis.
#[allow(unused_imports)]
use oxideav_aac as _force_link;

#[test]
fn aac_self_registers_into_runtime_context_via_slice() {
    // Build a context using ONLY the slice walker — no explicit
    // `oxideav_aac::register(&mut ctx)` call here. If the linkme entry
    // didn't materialise, this codec lookup would fail.
    let ctx = RuntimeContext::with_all_features();
    let id = CodecId::new("aac");
    let params = CodecParameters::audio(id);
    let _decoder = ctx
        .codecs
        .first_decoder(&params)
        .expect("aac decoder factory should be installed via the linkme slice walker");
}

#[test]
fn aac_appears_in_traced_walk() {
    let mut names = Vec::<String>::new();
    let _ctx = RuntimeContext::with_all_features_traced(|n| names.push(n.to_string()));
    assert!(
        names.iter().any(|n| n == "aac"),
        "expected `aac` to appear in registrar trace, got {names:?}"
    );
}

#[test]
fn aac_can_be_filtered_out() {
    let ctx = RuntimeContext::with_all_features_filtered(|n| n != "aac");
    let id = CodecId::new("aac");
    let params = CodecParameters::audio(id);
    assert!(
        ctx.codecs.first_decoder(&params).is_err(),
        "filter should suppress aac registration"
    );
}

//! `ps_data()` writer ↔ parser round trip under arbitrary input.
//!
//! Two views of the same bytes:
//!
//! 1. **Parse-first** — the bytes are a `ps_data()` payload (with and
//!    without a prior configuration). Whatever parses must
//!    re-serialise through `ps_writer` and reparse to the identical
//!    element, and resolve identically against a fresh index state.
//! 2. **Build-first** — the bytes are a parameter recipe: modes,
//!    frame class, envelope count, borders and per-band delta values
//!    (reduced into each codebook's range). The assembled element
//!    must always be writable and reparse to itself.
//!
//! Neither path may panic.

#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_aac::ps_data::{PsConfig, PsData, PsIndexState};
use oxideav_aac::ps_writer::{ps_data_bytes, write_ps_data};
use oxideav_core::bits::{BitReader, BitWriter};

/// Cyclic byte feed for the build-first recipe.
struct Feed<'a> {
    data: &'a [u8],
    pos: usize,
}

impl Feed<'_> {
    fn next(&mut self) -> u8 {
        let b = self.data[self.pos % self.data.len()];
        self.pos += 1;
        b
    }

    /// `num_env` rows of `n` deltas reduced into `[-span, span]`, or
    /// into `0..8` when `modulo`.
    fn rows(&mut self, num_env: usize, n: usize, span: i32, modulo: bool) -> Vec<Vec<i32>> {
        (0..num_env)
            .map(|_| {
                (0..n)
                    .map(|_| {
                        let v = i32::from(self.next());
                        if modulo {
                            v % 8
                        } else {
                            v % (2 * span + 1) - span
                        }
                    })
                    .collect()
            })
            .collect()
    }
}

fn config_from(b: u8) -> PsConfig {
    PsConfig {
        enable_iid: b & 1 != 0,
        iid_mode: (b >> 1) % 6,
        enable_icc: b & 0x10 != 0,
        icc_mode: (b >> 5) % 6,
        enable_ext: b & 0x80 != 0,
    }
}

fn check_roundtrip(ps: &PsData, prev: Option<&PsConfig>) {
    let bytes = match ps_data_bytes(ps) {
        Ok(b) => b,
        Err(e) => {
            // The parser tolerates one non-conformant shape the writer
            // refuses: `enable_ipdopd` without IID (§8.5.2 forbids it;
            // the parser reads zero-width phase rows). Nothing else
            // may be unwritable.
            assert!(
                ps.enable_ipdopd && !ps.config.enable_iid,
                "parsed element unwritable: {e:?} {ps:?}"
            );
            return;
        }
    };
    let mut r = BitReader::new(&bytes);
    let back = PsData::parse(&mut r, prev)
        .expect("written element parses")
        .expect("written element is decodable");
    assert_eq!(&back, ps, "writer/parser round trip");
    // Fewer than 8 padding bits follow the element.
    assert!(bytes.len() * 8 - (r.bit_position() as usize) < 8);
    // Deterministic bit count.
    let mut w = BitWriter::new();
    write_ps_data(&mut w, ps).unwrap();
    let mut w2 = BitWriter::new();
    write_ps_data(&mut w2, &back).unwrap();
    assert_eq!(w.bit_position(), w2.bit_position());
    // Resolution is a pure function of the element.
    let mut s1 = PsIndexState::default();
    let mut s2 = PsIndexState::default();
    assert_eq!(ps.resolve(&mut s1).ok(), back.resolve(&mut s2).ok());
    assert_eq!(s1, s2);
}

fuzz_target!(|data: &[u8]| {
    if data.is_empty() {
        return;
    }
    // 1. Parse-first.
    let prev = config_from(data[0]);
    for prev_cfg in [None, Some(&prev)] {
        let mut r = BitReader::new(&data[1..]);
        if let Ok(Some(ps)) = PsData::parse(&mut r, prev_cfg) {
            check_roundtrip(&ps, prev_cfg);
        }
    }

    // 2. Build-first.
    let mut feed = Feed {
        data,
        pos: 0,
    };
    let mut config = config_from(feed.next());
    let header = feed.next() & 1 != 0;
    let frame_class = feed.next() & 1 != 0;
    let num_env = if frame_class {
        1 + usize::from(feed.next() % 4)
    } else {
        [0usize, 1, 2, 4][usize::from(feed.next() % 4)]
    };
    let mut border_position = Vec::new();
    if frame_class {
        let mut last = 0u8;
        for _ in 0..num_env {
            last = (last + feed.next() % 8).min(31);
            border_position.push(last);
        }
    }
    let nr_iid = config.nr_iid_par();
    let nr_icc = config.nr_icc_par();
    let nr_ph = config.nr_ipdopd_par();
    let iid_span = if config.iid_quant_fine() { 30i32 } else { 14 };
    let iid_dt: Vec<bool> = (0..num_env).map(|_| feed.next() & 1 != 0).collect();
    let iid_deltas = if config.enable_iid {
        feed.rows(num_env, nr_iid, iid_span, false)
    } else {
        Vec::new()
    };
    let icc_dt: Vec<bool> = (0..num_env).map(|_| feed.next() & 1 != 0).collect();
    let icc_deltas = if config.enable_icc {
        feed.rows(num_env, nr_icc, 7, false)
    } else {
        Vec::new()
    };
    let enable_ipdopd = config.enable_ext && config.enable_iid && feed.next() & 1 != 0;
    let (ipd_dt, ipd_deltas, opd_dt, opd_deltas) = if enable_ipdopd {
        (
            (0..num_env).map(|_| feed.next() & 1 != 0).collect(),
            feed.rows(num_env, nr_ph, 0, true),
            (0..num_env).map(|_| feed.next() & 1 != 0).collect(),
            feed.rows(num_env, nr_ph, 0, true),
        )
    } else {
        (Vec::new(), Vec::new(), Vec::new(), Vec::new())
    };
    if !config.enable_iid {
        config.iid_mode = 0;
    }
    if !config.enable_icc {
        config.icc_mode = 0;
    }
    let ps = PsData {
        header_present: header,
        config,
        frame_class,
        num_env,
        border_position,
        iid_dt: if config.enable_iid { iid_dt } else { Vec::new() },
        iid_deltas,
        icc_dt: if config.enable_icc { icc_dt } else { Vec::new() },
        icc_deltas,
        enable_ipdopd,
        ipd_dt,
        ipd_deltas,
        opd_dt,
        opd_deltas,
    };
    check_roundtrip(&ps, Some(&config));
});

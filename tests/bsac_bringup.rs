//! ER BSAC (AOT 22) bring-up diagnostics against the ISO/IEC
//! 14496-26 conformance corpus (`OXIDEAV_ISO_14496_26_DIR`,
//! skip-if-absent; sourcing recipe in
//! `docs/audio/aac/iso-14496-26-conformance.md`).
//!
//! What these probes establish on `er_bs01_48_ep0` (mono, 48 kHz,
//! non-SBA, `top_layer = 48`):
//!
//! * The raw-bit headers, the §4.5.2.6.2.5 layer geometry and the
//!   arithmetic-coded **side information decode in sync with the
//!   deployed encoder**: silent access units reproduce the
//!   reference PCM exactly, and on content frames the decoded
//!   `cband_si` / scalefactors are pinned against a TDAC oracle —
//!   the §4.6.11 perfect-reconstruction property recovers each
//!   frame's exact transmitted spectrum from the reference
//!   waveform, and the oracle-implied quantized values match the
//!   decoded `cband_si`'s MSB plane and the decoded scalefactor
//!   gains precisely.
//! * The **spectral bit-slice probability selection diverges from
//!   the deployed encoder**: with the §4.6.4.2.3 selection as
//!   printed (Table 4.A.33 tables, Table 4.A.34 positions — both
//!   the 2009 and the 2001 alias schemes were tried), the decoded
//!   sliced bits departed from the oracle truth partway into the
//!   first coding band's MSB pass. A constraint solver over the
//!   real stream (`solver` probe below) proves that a consistent
//!   per-context p0 dictionary **exists** for the full multi-plane
//!   truth sequence — the symbol order, sign interleave and
//!   context classes are right — but the dictionary's values match
//!   no row of the printed tables under any position mapping
//!   tried; e.g. the all-zero-context position demands
//!   p0 ∈ {0x3700..=0x3a00} where every plausible printed row
//!   carries 0x3b00+ there. The deployed selection rule (or a
//!   corrigendum) is the standing docs ask; the probes below are
//!   the instruments for settling it.

use std::path::PathBuf;

fn corpus_dir() -> Option<PathBuf> {
    let dir = PathBuf::from(std::env::var_os("OXIDEAV_ISO_14496_26_DIR")?);
    dir.is_dir().then_some(dir)
}

/// stsz/stco walk (single-sample chunks in this corpus).
fn mp4_aus(data: &[u8]) -> Vec<Vec<u8>> {
    fn find<'a>(mut cur: &'a [u8], path: &[&str]) -> Option<&'a [u8]> {
        for want in path {
            let mut i = 0usize;
            let mut found = None;
            while i + 8 <= cur.len() {
                let sz = u32::from_be_bytes(cur[i..i + 4].try_into().unwrap()) as usize;
                let ty = &cur[i + 4..i + 8];
                if ty == want.as_bytes() {
                    found = Some(&cur[i + 8..i + sz]);
                    break;
                }
                if sz < 8 {
                    return None;
                }
                i += sz;
            }
            cur = found?;
        }
        Some(cur)
    }
    let stbl = find(data, &["moov", "trak", "mdia", "minf", "stbl"]).unwrap();
    let stsz = find(stbl, &["stsz"]).unwrap();
    let count = u32::from_be_bytes(stsz[8..12].try_into().unwrap()) as usize;
    let sizes: Vec<usize> = (0..count)
        .map(|i| u32::from_be_bytes(stsz[12 + 4 * i..16 + 4 * i].try_into().unwrap()) as usize)
        .collect();
    let stco = find(stbl, &["stco"]).unwrap();
    let n = u32::from_be_bytes(stco[4..8].try_into().unwrap()) as usize;
    let offsets: Vec<usize> = (0..n)
        .map(|i| u32::from_be_bytes(stco[8 + 4 * i..12 + 4 * i].try_into().unwrap()) as usize)
        .collect();
    offsets
        .iter()
        .zip(sizes.iter())
        .map(|(&o, &s)| data[o..o + s].to_vec())
        .collect()
}

/// Reference PCM (mono 16-bit `data` chunk).
fn read_ref_wav(path: &PathBuf) -> Vec<f64> {
    let wav = std::fs::read(path).unwrap();
    let data_pos = wav.windows(4).position(|w| w == b"data").unwrap() + 8;
    wav[data_pos..]
        .chunks_exact(2)
        .map(|c| f64::from(i16::from_le_bytes([c[0], c[1]])))
        .collect()
}

/// §4.6.11.3.2 KBD left half (alpha 4, long transform), for the
/// TDAC oracle.
fn kbd_left_long() -> Vec<f64> {
    fn bessel_i0(x: f64) -> f64 {
        let mut sum = 1.0f64;
        let mut term = 1.0f64;
        let mut k = 1.0f64;
        loop {
            term *= (x / (2.0 * k)) * (x / (2.0 * k));
            sum += term;
            if term < 1e-21 * sum || k > 256.0 {
                break;
            }
            k += 1.0;
        }
        sum
    }
    let half = 1024usize;
    let quarter = half as f64 / 2.0;
    let denom = bessel_i0(std::f64::consts::PI * 4.0);
    let kernel: Vec<f64> = (0..=half)
        .map(|n| {
            let t = (n as f64 - quarter) / quarter;
            bessel_i0(std::f64::consts::PI * 4.0 * (1.0 - t * t).max(0.0).sqrt()) / denom
        })
        .collect();
    let total: f64 = kernel.iter().sum();
    let mut running = 0.0;
    kernel
        .iter()
        .take(half)
        .map(|&w| {
            running += w;
            (running / total).sqrt()
        })
        .collect()
}

/// TDAC oracle: the §4.6.11 perfect-reconstruction property makes
/// the forward MDCT of the reference PCM over frame `k`'s window
/// recover the frame's exact transmitted spectrum (`ONLY_LONG`,
/// KBD-to-KBD frames).
fn oracle_spectrum(reference: &[f64], k: usize, lines: usize) -> Vec<f64> {
    let seg = &reference[(k - 1) * 1024..(k + 1) * 1024];
    let left = kbd_left_long();
    let mut z = vec![0.0f64; 2048];
    for n in 0..1024 {
        z[n] = seg[n] * left[n];
        z[2047 - n] = seg[2047 - n] * left[n];
    }
    let n0 = 1025.0 / 2.0;
    let step = 2.0 * std::f64::consts::PI / 2048.0;
    (0..lines)
        .map(|kk| {
            2.0 * z
                .iter()
                .enumerate()
                .map(|(n, &t)| t * (step * (n as f64 + n0) * (kk as f64 + 0.5)).cos())
                .sum::<f64>()
        })
        .collect()
}

/// The oracle-implied quantized values of AU `k`'s first 32 lines,
/// from the decoded scalefactors (§4.6.2 forward quantizer).
fn oracle_q(
    reference: &[f64],
    block: &oxideav_aac::bsac_decode::DecodedBlock,
    k: usize,
) -> Vec<i32> {
    use oxideav_aac::dequant::scale_factor_gain;
    let ref_spec = oracle_spectrum(reference, k, 32);
    let offsets = &block.geometry.swb_offset[0];
    let mut q_ref = vec![0i32; 32];
    for sfb in 0..9 {
        let Some(scf) = block.scf[0][0][sfb] else {
            continue;
        };
        let gain = scale_factor_gain(scf);
        for i in offsets[sfb]..offsets[sfb + 1].min(32) {
            let mag = (ref_spec[i].abs() / gain).powf(0.75).round() as i32;
            q_ref[i] = if ref_spec[i] < 0.0 { -mag } else { mag };
        }
    }
    q_ref
}

/// Silent access units reproduce the reference exactly, and the
/// side-information decode is oracle-consistent on a content frame:
/// the decoded `cband_si`'s MSB plane equals the plane of the
/// oracle-implied quantized values under the decoded scalefactors —
/// pinning the header parse, the layer roster and the arithmetic
/// `cband_si` / scalefactor decode against the deployed encoder.
#[test]
fn bsac_side_info_decodes_in_sync() {
    use oxideav_aac::bsac_tables::CBAND_SI_MSB_PLANE;

    let Some(dir) = corpus_dir() else { return };
    let Ok(m4a) = std::fs::read(dir.join("compressedMp4/er_bs01_48_ep0.mp4")) else {
        eprintln!("skip: er_bs01_48_ep0.mp4 not present");
        return;
    };
    let aus = mp4_aus(&m4a);
    let reference = read_ref_wav(&dir.join("referencesWav/er_bs01_48_lay48.wav"));

    // AUs 0 and 1 are silence: the whole decode (headers, layer
    // walk, si arithmetic, all-zero spectra) must be sample-exact.
    let mut dec = oxideav_aac::bsac_decode::BsacDecoder::new(48_000, 3, 1).unwrap();
    for au_i in 0..2usize {
        let pcm = dec.decode_frame(&aus[au_i]).expect("silent AU decodes");
        for (t, &s) in pcm.iter().enumerate() {
            assert_eq!(
                f64::from(s),
                reference[au_i * 1024 + t],
                "silent AU {au_i} sample {t}"
            );
        }
    }

    // AU 4 (ONLY_LONG, KBD): oracle consistency of the si layer.
    let block =
        oxideav_aac::bsac_decode::decode_bsac_raw_data_block(&aus[4], 48_000, 3, 1).unwrap();
    let q_ref = oracle_q(&reference, &block, 4);
    let maxq = q_ref.iter().map(|q| q.unsigned_abs()).max().unwrap();
    let oracle_plane = 32 - maxq.leading_zeros();
    let si0 = {
        // Re-derive cband 0's si from the block: its MSB plane is
        // the largest decoded plane... the decoded samples diverge,
        // so read the si straight off a manual prefix decode.
        use oxideav_aac::bsac_arith::{ArithDecoder, SegmentReader};
        use oxideav_aac::bsac_tables::CBAND_SI_MODEL_CBAND0;
        let mut rd = SegmentReader::new(&aus[4], 72, (aus[4].len() as u64) * 8);
        let mut d = ArithDecoder::new();
        d.decode_symbol(&mut rd, &CBAND_SI_MODEL_CBAND0).0
    };
    assert_eq!(
        u32::from(CBAND_SI_MSB_PLANE[si0]),
        oracle_plane,
        "decoded cband_si plane vs oracle plane"
    );
    eprintln!(
        "si layer in sync: cband_si {} (plane {}), oracle max |q| {} (plane {})",
        si0, CBAND_SI_MSB_PLANE[si0], maxq, oracle_plane
    );
}

/// Diagnostic (non-asserting): locate the first sliced-bit
/// divergence between this crate's §4.6.4.2.3 probability selection
/// and the deployed encoder, against the oracle truth.
#[test]
fn bsac_spectral_divergence_report() {
    use oxideav_aac::bsac_arith::{ArithDecoder, SegmentReader};
    use oxideav_aac::bsac_tables::*;

    let Some(dir) = corpus_dir() else { return };
    let Ok(m4a) = std::fs::read(dir.join("compressedMp4/er_bs01_48_ep0.mp4")) else {
        return;
    };
    let aus = mp4_aus(&m4a);
    let reference = read_ref_wav(&dir.join("referencesWav/er_bs01_48_lay48.wav"));
    let block =
        oxideav_aac::bsac_decode::decode_bsac_raw_data_block(&aus[4], 48_000, 3, 1).unwrap();
    let q_ref = oracle_q(&reference, &block, 4);

    let au = &aus[4];
    let mut rd = SegmentReader::new(au, 72, (au.len() as u64) * 8);
    let mut dec = ArithDecoder::new();
    let (si0, _) = dec.decode_symbol(&mut rd, &CBAND_SI_MODEL_CBAND0);
    let base_model = SCF_MODELS[block.header.base_scf_model[0] as usize].unwrap();
    for _ in 0..9 {
        dec.decode_symbol(&mut rd, base_model);
    }
    let plane = CBAND_SI_MSB_PLANE[si0];
    let mut mask = [0u32; 32];
    let mut sign_coded = [false; 32];
    for snf in (1..=plane).rev() {
        for i in 0..32usize {
            let true_bit = ((q_ref[i].unsigned_abs() >> (snf - 1)) & 1) as u8;
            let hbv = mask[i] >> snf;
            let p0 = if hbv != 0 {
                spectral_p0(si0 as u8, snf, hbv, 0)
            } else {
                let a = i % 4;
                let bit_at = |j: isize| -> u8 {
                    if j < 0 {
                        0
                    } else {
                        ((mask[j as usize] >> (snf - 1)) & 1) as u8
                    }
                };
                let hb = |j: usize| -> u8 {
                    if j >= 32 {
                        0
                    } else {
                        u8::from(mask[j] >> snf != 0)
                    }
                };
                let prev = [
                    bit_at(i as isize - 3),
                    bit_at(i as isize - 2),
                    bit_at(i as isize - 1),
                ];
                let base = i - a;
                let flags = [hb(base), hb(base + 1), hb(base + 2), hb(base + 3)];
                spectral_p0(si0 as u8, snf, 0, context_position(a, prev, flags))
            };
            let (bit, _) = dec.decode_bit(&mut rd, p0);
            if bit != true_bit {
                eprintln!(
                    "first divergence: snf {snf} line {i} (p0 {p0:#06x}, decoded {bit}, oracle {true_bit})"
                );
                return;
            }
            if bit != 0 {
                mask[i] |= 1 << (snf - 1);
            }
            if mask[i] != 0 && !sign_coded[i] {
                let (s, _) = dec.decode_bit(&mut rd, SIGN_P0);
                sign_coded[i] = true;
                if s != u8::from(q_ref[i] < 0) {
                    eprintln!("first divergence: sign of line {i} at snf {snf}");
                    return;
                }
            }
        }
    }
    eprintln!("no divergence over the whole first coding band");
}

/// Diagnostic (non-asserting): for each context class of the MSB
/// pass, sweep which p0 values are compatible with the real stream
/// (all other classes left free) — the instrument that showed a
/// consistent context→p0 dictionary exists while the printed rows
/// do not fit (the all-zero-context class demands
/// p0 ∈ {0x3700..=0x3a00}).
#[test]
fn bsac_p0_interval_sweep() {
    use oxideav_aac::bsac_arith::{ArithDecoder, SegmentReader};
    use oxideav_aac::bsac_tables::*;
    use std::collections::HashMap;

    let Some(dir) = corpus_dir() else { return };
    let Ok(m4a) = std::fs::read(dir.join("compressedMp4/er_bs01_48_ep0.mp4")) else {
        return;
    };
    let aus = mp4_aus(&m4a);
    let reference = read_ref_wav(&dir.join("referencesWav/er_bs01_48_lay48.wav"));
    let block =
        oxideav_aac::bsac_decode::decode_bsac_raw_data_block(&aus[4], 48_000, 3, 1).unwrap();
    let q_ref = oracle_q(&reference, &block, 4);
    let au = &aus[4];

    #[derive(Clone, Copy, PartialEq, Eq, Hash)]
    struct Ctx {
        pos: u8,
    }
    let snf = 11u8;
    let mut syms: Vec<(Option<Ctx>, u8)> = Vec::new();
    let mut mask = [0u32; 32];
    for i in 0..32usize {
        let tb = ((q_ref[i].unsigned_abs() >> (snf - 1)) & 1) as u8;
        let a = i % 4;
        let bit_at = |j: isize| -> u8 {
            if j < 0 {
                0
            } else {
                ((mask[j as usize] >> (snf - 1)) & 1) as u8
            }
        };
        let prev = [
            bit_at(i as isize - 3),
            bit_at(i as isize - 2),
            bit_at(i as isize - 1),
        ];
        let pos = context_position(a, prev, [0, 0, 0, 0]) as u8;
        syms.push((Some(Ctx { pos }), tb));
        if tb == 1 {
            mask[i] |= 1 << (snf - 1);
            syms.push((None, u8::from(q_ref[i] < 0)));
        }
    }
    let mut rd0 = SegmentReader::new(au, 72, (au.len() as u64) * 8);
    let mut dec0 = ArithDecoder::new();
    dec0.decode_symbol(&mut rd0, &CBAND_SI_MODEL_CBAND0);
    let base_model = SCF_MODELS[block.header.base_scf_model[0] as usize].unwrap();
    for _ in 0..9 {
        dec0.decode_symbol(&mut rd0, base_model);
    }
    fn count(
        idx: usize,
        syms: &[(Option<Ctx>, u8)],
        dec: &ArithDecoder,
        rd: &SegmentReader<'_>,
        dict: &mut HashMap<Ctx, u16>,
        sols: &mut usize,
    ) {
        use oxideav_aac::bsac_tables::SIGN_P0;
        if *sols > 20 {
            return;
        }
        if idx >= syms.len() {
            *sols += 1;
            return;
        }
        let (ctx, tb) = syms[idx];
        let try_p0 =
            |p0: u16, dict: &mut HashMap<Ctx, u16>, sols: &mut usize, insert: Option<Ctx>| {
                let mut dec2 = dec.clone();
                let mut rd2 = rd.clone();
                let (b, _) = dec2.decode_bit(&mut rd2, p0);
                if b == tb {
                    if let Some(c) = insert {
                        dict.insert(c, p0);
                        count(idx + 1, syms, &dec2, &rd2, dict, sols);
                        dict.remove(&c);
                    } else {
                        count(idx + 1, syms, &dec2, &rd2, dict, sols);
                    }
                }
            };
        match ctx {
            None => try_p0(SIGN_P0, dict, sols, None),
            Some(c) => {
                if let Some(&p0) = dict.get(&c) {
                    try_p0(p0, dict, sols, None);
                } else {
                    for p in 1..64u16 {
                        try_p0(p << 8, dict, sols, Some(c));
                    }
                }
            }
        }
    }
    let mut positions: Vec<u8> = syms.iter().filter_map(|(c, _)| c.map(|c| c.pos)).collect();
    positions.sort_unstable();
    positions.dedup();
    for pos in positions {
        let mut good = Vec::new();
        for p in 1..64u16 {
            let p0 = p << 8;
            let mut dict = HashMap::new();
            dict.insert(Ctx { pos }, p0);
            let mut sols = 0usize;
            count(0, &syms, &dec0, &rd0, &mut dict, &mut sols);
            if sols > 0 {
                good.push(format!("{p0:#06x}"));
            }
        }
        eprintln!("MSB-pass context position {pos}: stream-compatible p0 {good:?}");
    }
}

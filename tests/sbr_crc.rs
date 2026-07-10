//! SBR `bs_sbr_crc_bits` verification end to end — ISO/IEC 14496-3:2009
//! §4.4.2.8.1 (CRC-10 `G10`, zero init) over the Table 4.62 coverage
//! region (`num_sbr_bits − 10` payload bits after the CRC field),
//! wired through the `EXT_SBR_DATA_CRC` (type 14) fill-extension path
//! of the stream decoder.
//!
//! Builds a raw data block with the crate's own writers — an SCE core
//! channel followed by a FIL element whose `extension_payload()`
//! carries `sbr_extension_data(crc_flag = 1)` with a freshly computed
//! CRC — and pins:
//!
//! * a correct CRC decodes (dual-rate HE-AAC output), byte-identical
//!   to the same SBR payload sent through the plain `EXT_SBR_DATA`
//!   (type 13, CRC-less) path;
//! * a corrupted CRC field surfaces `Error::SbrCrcMismatch`;
//! * a corrupted *covered* payload bit (the structurally inert
//!   `bs_reserved` header field) surfaces `Error::SbrCrcMismatch`.

use oxideav_aac::adts_crc::sbr_crc;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::ics_info::{IcsInfo, WindowSequence, WindowShape, NUM_SWB_LONG_WINDOW};
use oxideav_aac::raw_data_block::{FrameAssembler, IdSynEle};
use oxideav_aac::sbr_freq_bands::HiLoTables;
use oxideav_aac::sbr_grid::FrameClass;
use oxideav_aac::sbr_header::SbrHeader;
use oxideav_aac::sbr_huffman::{env_tables, noise_tables, SbrHuffContext};
use oxideav_aac::scale_factor_data::{ScaleFactorData, ScaleFactorEntry};
use oxideav_aac::section_data::{Section, SectionData};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::Error;
use oxideav_core::bits::{BitReader, BitWriter};

const AOT_LC: u8 = 2;
const FS_INDEX: u8 = 4; // 44.1 kHz core
const SAMPLE_RATE: u32 = 44_100;
const FS_SBR: u32 = 88_200;

// ------------------------------------------------------------------
// Core SCE channel (long window, codebook 1, constant spectrum).
// ------------------------------------------------------------------

fn long_ics_info(max_sfb: u8) -> IcsInfo {
    IcsInfo {
        ics_reserved_bit: false,
        window_sequence: WindowSequence::OnlyLong,
        window_shape: WindowShape::Sine,
        max_sfb,
        scale_factor_grouping: None,
        predictor_data_present: false,
        predictor_data: None,
        ltp_data_present: false,
        ltp_data: None,
        ltp_data_present_pair: None,
        ltp_data_pair: None,
        num_windows: 1,
        num_window_groups: 1,
        window_group_length: vec![1],
        num_swb: NUM_SWB_LONG_WINDOW[FS_INDEX as usize],
    }
}

fn make_channel(max_sfb: u8, global_gain: u8) -> (IcsBody, SpectralData) {
    let sfb_cb = vec![vec![1u8; max_sfb as usize]];
    let sections = vec![vec![Section {
        codebook: 1,
        start: 0,
        end: max_sfb,
    }]];
    let entries: Vec<ScaleFactorEntry> = (0..max_sfb).map(|_| ScaleFactorEntry::Dpcm(0)).collect();
    let body = IcsBody {
        global_gain,
        ics_info: Some(long_ics_info(max_sfb)),
        section_data: SectionData { sections, sfb_cb },
        scale_factor_data: ScaleFactorData {
            entries: vec![entries],
        },
        pulse_data_present: false,
        pulse_data: None,
        tns_data_present: false,
        tns_data: None,
        gain_control_data_present: false,
        gain_control_data: None,
        spectral_data_bit_offset: 0,
        er_scale_factor_data: None,
        reordered_spectral_lengths: None,
    };
    let offsets = oxideav_aac::swb_offset::long_window_offsets(FS_INDEX).unwrap();
    let active = usize::from(offsets[max_sfb as usize]);
    let mut coeffs = vec![0i32; 1024];
    coeffs[..active].fill(1);
    let spectral = SpectralData {
        x_quant: vec![coeffs],
    };
    (body, spectral)
}

fn push_sce(fa: &mut FrameAssembler, body: &IcsBody, spectral: &SpectralData) {
    fa.push_channel_header(IdSynEle::Sce, 0).unwrap();
    let mut bw = BitWriter::new();
    body.write(&mut bw, AOT_LC, FS_INDEX, false).unwrap();
    let ics = body.ics_info.as_ref().unwrap();
    spectral
        .write(&mut bw, ics, &body.section_data, FS_INDEX)
        .unwrap();
    let bits = bw.bit_position();
    fa.push_channel_body_bits(&bw.finish(), bits).unwrap();
}

// ------------------------------------------------------------------
// Minimal single-channel SBR side info (Table 4.62 / 4.63 / 4.65).
// ------------------------------------------------------------------

/// `sbr_header()` with explicit extra-1 params (freq_scale 0,
/// alter_scale false, noise_bands 2) so the derived band geometry is
/// deterministic; extra-2 absent.
fn write_header(w: &mut BitWriter) {
    w.write_bit(true); // bs_amp_res
    w.write_u32(5, 4); // bs_start_freq
    w.write_u32(0, 4); // bs_stop_freq
    w.write_u32(1, 3); // bs_xover_band
    w.write_u32(0, 2); // bs_reserved
    w.write_bit(true); // bs_header_extra_1
    w.write_bit(false); // bs_header_extra_2
    w.write_u32(0, 2); // bs_freq_scale
    w.write_bit(false); // bs_alter_scale
    w.write_u32(2, 2); // bs_noise_bands
}

/// The band tables the `write_header`-built header derives.
fn header_bands() -> HiLoTables {
    let mut w = BitWriter::new();
    write_header(&mut w);
    let bytes = w.finish();
    let mut r = BitReader::new(&bytes);
    let h = SbrHeader::parse(&mut r).unwrap();
    h.derive_bands(FS_SBR).unwrap()
}

fn push_code(w: &mut BitWriter, table: &[(u8, u32)], idx: usize) {
    let (len, code) = table[idx];
    w.write_u32(code, u32::from(len));
}

/// Minimal single-channel SBR data element (FIXFIX single env, freq
/// deltas, no sinusoidal / extended data).
fn write_minimal_sce_sbr(w: &mut BitWriter, bands: &HiLoTables) {
    let n_high = bands.n_high();
    let n_q = bands.n_q();
    w.write_bit(false); // bs_data_extra
    w.write_u32(FrameClass::FixFix.to_bits(), 2);
    w.write_u32(0, 2); // 2^0 = 1 env
    w.write_bit(true); // freq_res[0] high
    w.write_bit(false); // df_env[0]
    w.write_bit(false); // df_noise[0]
    for _ in 0..n_q {
        w.write_u32(1, 2); // invf modes
    }
    let (_, (f_huff, f_lav)) = env_tables(SbrHuffContext {
        coupling: false,
        ch: false,
        amp_res: false,
    });
    w.write_u32(33, 7); // env start value (single-env FIXFIX override)
    for i in 1..n_high {
        push_code(w, f_huff, (i + f_lav as usize) % f_huff.len());
    }
    let (_, (nf, nfl)) = noise_tables(SbrHuffContext {
        coupling: false,
        ch: false,
        amp_res: false,
    });
    w.write_u32(10, 5); // noise start
    for i in 1..n_q {
        push_code(w, nf, (i + nfl as usize) % nf.len());
    }
    w.write_bit(false); // bs_add_harmonic_flag
    w.write_bit(false); // bs_extended_data
}

/// Build the FIL `extension_payload()` bytes: the 4-bit extension
/// type, the optional 10-bit CRC, the SBR side info, and the Table
/// 4.62 `bs_fill_bits` byte alignment. Returns the payload bytes.
fn build_sbr_fil_payload(with_crc: bool, corrupt_crc: bool) -> Vec<u8> {
    // The SBR payload bits (bs_header_flag .. end of sbr_data) — the
    // exact §4.4.2.8.1 coverage region when a CRC is present.
    let bands = header_bands();
    let mut sw = BitWriter::new();
    sw.write_bit(true); // bs_header_flag
    write_header(&mut sw);
    write_minimal_sce_sbr(&mut sw, &bands);
    let sbr_bits = sw.bit_position();
    let sbr_bytes = sw.finish();

    let mut crc = sbr_crc(&sbr_bytes, 0, sbr_bits);
    if corrupt_crc {
        crc ^= 0x001;
    }

    let mut w = BitWriter::new();
    if with_crc {
        w.write_u32(0b1110, 4); // EXT_SBR_DATA_CRC
        w.write_u32(u32::from(crc), 10); // bs_sbr_crc_bits
    } else {
        w.write_u32(0b1101, 4); // EXT_SBR_DATA
    }
    for i in 0..sbr_bits {
        w.write_bit(sbr_bytes[(i / 8) as usize] & (0x80 >> (i % 8)) != 0);
    }
    w.align_to_byte_zero(); // bs_fill_bits
    w.finish()
}

/// Assemble one raw data block: [SCE core, FIL(SBR), END]. Returns
/// the payload plus the absolute bit position of the FIL
/// `extension_payload()` body within it.
fn build_block(fil_payload: &[u8]) -> (Vec<u8>, u64) {
    let (body, spectral) = make_channel(8, 150);
    let mut fa = FrameAssembler::new();
    push_sce(&mut fa, &body, &spectral);
    let count_bits = if fil_payload.len() >= 15 { 12 } else { 4 };
    let fil_body_bit = fa.bit_position() + 3 + count_bits;
    fa.push_fill(fil_payload).unwrap();
    (fa.push_end(), fil_body_bit)
}

fn decode(payload: &[u8]) -> Result<oxideav_aac::decode::DecodedFrame, Error> {
    StreamDecoder::new().decode_raw_data_block(AOT_LC, FS_INDEX, SAMPLE_RATE, 1, 1, payload)
}

#[test]
fn sbr_crc_payload_decodes_and_matches_crcless_path() {
    let (with_crc, _) = build_block(&build_sbr_fil_payload(true, false));
    let (without, _) = build_block(&build_sbr_fil_payload(false, false));

    let a = decode(&with_crc).unwrap();
    assert_eq!(a.sample_rate, FS_SBR, "SBR output is dual-rate");
    assert_eq!(a.pcm.len(), 2048);
    assert!(a.pcm.iter().any(|&s| s != 0));

    // The CRC field only protects — the reconstruction must be
    // byte-identical to the type-13 (CRC-less) carriage of the same
    // side info.
    let b = decode(&without).unwrap();
    assert_eq!(a.pcm, b.pcm);
    assert_eq!(a.sample_rate, b.sample_rate);
}

#[test]
fn corrupt_sbr_crc_field_is_rejected() {
    let (payload, _) = build_block(&build_sbr_fil_payload(true, true));
    assert!(matches!(decode(&payload), Err(Error::SbrCrcMismatch)));
}

#[test]
fn corrupt_covered_sbr_bit_is_rejected() {
    // Flip one bs_reserved bit — structurally inert (the header still
    // parses; the field is read and ignored) but inside the coverage
    // region, so the transmitted CRC no longer matches.
    let (mut payload, fil_body_bit) = build_block(&build_sbr_fil_payload(true, false));
    // extension_type(4) + crc(10) + bs_header_flag(1) + bs_amp_res(1)
    // + start(4) + stop(4) + xover(3) → bs_reserved at +27.
    let bit = fil_body_bit + 27;
    payload[(bit / 8) as usize] ^= 0x80 >> (bit % 8);
    assert!(matches!(decode(&payload), Err(Error::SbrCrcMismatch)));
}

#[test]
fn crcless_type13_payload_ignores_reserved_corruption() {
    // The same bs_reserved flip through the type-13 path must decode
    // (nothing verifies it) — pinning that the mismatch above comes
    // from the CRC gate, not a parse failure.
    let (base, _) = build_block(&build_sbr_fil_payload(false, false));
    let (mut tweaked, fil_body_bit) = build_block(&build_sbr_fil_payload(false, false));
    // No CRC field on this path: bs_reserved sits at +17.
    let bit = fil_body_bit + 17;
    tweaked[(bit / 8) as usize] ^= 0x80 >> (bit % 8);
    let a = decode(&base).unwrap();
    let b = decode(&tweaked).unwrap();
    assert_eq!(a.pcm, b.pcm, "bs_reserved must not affect reconstruction");
}

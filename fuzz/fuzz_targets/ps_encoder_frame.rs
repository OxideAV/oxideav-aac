//! The PS encoder over arbitrary QMF input: any 38-slot stereo
//! `Xinput` pair (including non-finite values) must encode without
//! panicking, and every produced element must reparse and resolve to
//! the encoder's own indices through the decoder-side parser.

#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_aac::ps_data::{PsData, PsIndexState};
use oxideav_aac::ps_encoder::{PsBands, PsEncoder, PsEncoderConfig};
use oxideav_aac::ps_hybrid::{LOOKAHEAD, NUM_QMF_SLOTS};
use oxideav_aac::sbr_qmf::Complex;
use oxideav_core::bits::BitReader;

const SLOTS: usize = NUM_QMF_SLOTS + LOOKAHEAD;

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }
    let cfg = PsEncoderConfig {
        bands: [PsBands::Ten, PsBands::Twenty, PsBands::ThirtyFour][usize::from(data[0] % 3)],
        fine_iid: data[0] & 4 != 0,
        icc: data[0] & 8 != 0,
        phase: data[0] & 16 != 0,
        header_interval: u32::from(data[1] % 4),
        variable_borders: data[0] & 32 != 0,
    };
    let frames = usize::from(1 + data[2] % 3);
    let mut enc = PsEncoder::new(cfg).unwrap();
    let mut dec_cfg = None;
    let mut dec_state = PsIndexState::default();
    let mut it = data[3..].iter().copied().cycle();
    // Values: mostly moderate, occasionally huge / non-finite, so the
    // estimators see every corner.
    let val = |b: u8| -> f64 {
        match b {
            0 => f64::NAN,
            1 => f64::INFINITY,
            2 => f64::NEG_INFINITY,
            3 => 0.0,
            4 => 1e300,
            5 => -1e300,
            _ => (f64::from(b) - 128.0) * 64.0,
        }
    };
    for _ in 0..frames {
        let mut l = vec![[Complex::default(); 64]; SLOTS];
        let mut r = vec![[Complex::default(); 64]; SLOTS];
        for n in 0..SLOTS {
            for k in 0..64 {
                l[n][k] = Complex::new(val(it.next().unwrap()), val(it.next().unwrap()));
                r[n][k] = Complex::new(val(it.next().unwrap()), val(it.next().unwrap()));
            }
        }
        let fr = enc.encode_frame(&l, &r).expect("encoder never fails on shaped input");
        let mut rd = BitReader::new(&fr.payload);
        let back = PsData::parse(&mut rd, dec_cfg.as_ref())
            .expect("payload parses")
            .expect("payload decodable");
        dec_cfg = Some(back.config);
        assert_eq!(back, fr.data);
        let idx = back.resolve(&mut dec_state).expect("resolves");
        assert_eq!(idx, fr.indices);
    }
});

//! The SBR encoder over arbitrary QMF input: any run of analysis
//! frames (including non-finite values) under any configuration —
//! both core families (16 / 15 time slots), mono and coupled /
//! uncoupled pairs, every header option the encoder exposes — must
//! encode without panicking, and every payload must reparse through
//! the decoder-side parser to the encoder's own element with a grid
//! the decoder derives (the VARVAR solver, the per-envelope
//! resolution election and the 960-line layout are the writer paths
//! under test).

#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_aac::raw_data_block::IdSynEle;
use oxideav_aac::sbr_encoder::{SbrEncoder, SbrEncoderConfig, NUM_TIME_SLOTS, NUM_TIME_SLOTS_960};
use oxideav_aac::sbr_extension::SbrExtensionData;
use oxideav_aac::sbr_qmf::Complex;
use oxideav_aac::sbr_time_grid::derive_time_grid;
use oxideav_core::bits::BitReader;

fuzz_target!(|data: &[u8]| {
    // Six configuration bytes, then the (cycled) sample stream.
    if data.len() < 8 {
        return;
    }
    let fs_sbr = [32_000u32, 44_100, 48_000, 24_000, 96_000, 16_000][usize::from(data[0] % 6)];
    let channels = 1 + usize::from(data[0] >> 7);
    let crossover = f64::from(fs_sbr) * (0.12 + 0.02 * f64::from(data[1] % 6));
    let stop = f64::from(fs_sbr) * (0.30 + 0.03 * f64::from(data[1] >> 4));
    let Ok(mut cfg) = SbrEncoderConfig::new(fs_sbr, channels, crossover, stop) else {
        return;
    };
    cfg.num_time_slots = if data[2] & 1 != 0 {
        NUM_TIME_SLOTS_960
    } else {
        NUM_TIME_SLOTS
    };
    cfg.amp_res = data[2] & 2 != 0;
    cfg.crc = data[2] & 4 != 0;
    cfg.add_harmonic = data[2] & 8 != 0;
    cfg.variable_borders = data[2] & 16 != 0;
    cfg.coupling = data[2] & 32 != 0;
    cfg.interpol_freq = data[2] & 64 != 0;
    cfg.header_interval = u32::from(data[3] % 4);
    cfg.limiter_gains = data[3] >> 6;
    cfg.freq_scale = (data[4] & 3).min(3);
    cfg.alter_scale = data[4] & 4 != 0;
    cfg.noise_bands = (data[4] >> 3) & 3;
    let Ok(mut enc) = SbrEncoder::new(cfg) else {
        return;
    };
    let cols = cfg.enc_cols();
    let frames = usize::from(1 + data[5] % 3);
    let mut it = data[6..].iter().copied().cycle();
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
    let id_aac = if channels == 1 {
        IdSynEle::Sce
    } else {
        IdSynEle::Cpe
    };
    let mut prev_header = None;
    for _ in 0..frames {
        let mut x: Vec<Vec<[Complex; 64]>> = vec![vec![[Complex::default(); 64]; cols]; channels];
        for chan in x.iter_mut() {
            for col in chan.iter_mut() {
                for cell in col.iter_mut() {
                    *cell = Complex::new(val(it.next().unwrap()), val(it.next().unwrap()));
                }
            }
        }
        let refs: Vec<&[[Complex; 64]]> = x.iter().map(|c| c.as_slice()).collect();
        let frame = enc
            .encode_frame(&refs)
            .expect("encoder never fails on shaped input");
        let mut rd = BitReader::new(&frame.payload);
        rd.read_u32(4).expect("extension type");
        let parsed = SbrExtensionData::parse(
            &mut rd,
            id_aac,
            cfg.crc,
            fs_sbr,
            Some(frame.payload.len() as u32),
            prev_header,
        )
        .expect("payload parses");
        assert_eq!(parsed.element, frame.element);
        assert_eq!(parsed.header_present, frame.header_sent);
        prev_header = Some(parsed.header);
        for ch in &parsed.element.channels {
            let tg = derive_time_grid(&ch.grid, cfg.num_time_slots as i32).expect("grid derives");
            assert!(tg.t_e.windows(2).all(|w| w[1] > w[0]));
            assert!(*tg.t_e.last().unwrap() <= cfg.num_time_slots as i32 + 3);
        }
    }
});

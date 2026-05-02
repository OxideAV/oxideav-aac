//! Manual gapless-validation helper: encode two 0.5 s sine fixtures,
//! concatenate them, and dump the joined ADTS stream + metadata to
//! `/tmp/oxideav_aac_concat.aac` for ffmpeg cross-decode.
//!
//! Usage:
//! ```sh
//! cargo run -p oxideav-aac --example dump_concat_gapless
//! ffmpeg -i /tmp/oxideav_aac_concat.aac -f s16le -ar 44100 /tmp/concat.s16
//! ```
//!
//! The expected "no audible click" acceptance is checked programmatically
//! by `tests/encode_gapless.rs::encoded_concat_no_click_at_join` on the
//! self-decoder; this example is for hand-validating against ffmpeg.

use oxideav_aac::encoder::AacEncoder;
use oxideav_aac::gapless::GaplessInfo;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Frame};

fn main() {
    let sr = 44_100u32;
    let half_samples = (sr / 2) as usize;
    let mut pcm_a = Vec::with_capacity(half_samples * 2);
    let mut pcm_b = Vec::with_capacity(half_samples * 2);
    for i in 0..half_samples {
        let t = i as f32 / sr as f32;
        let v = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.3;
        let s = (v * 32767.0) as i16;
        pcm_a.extend_from_slice(&s.to_le_bytes());
        pcm_b.extend_from_slice(&s.to_le_bytes());
    }

    let make_aac = |pcm: Vec<u8>| -> (Vec<u8>, GaplessInfo) {
        let mut p = CodecParameters::audio(CodecId::new("aac"));
        p.sample_rate = Some(sr);
        p.channels = Some(1);
        p.bit_rate = Some(128_000);
        let mut e = AacEncoder::new(&p).unwrap();
        let n = (pcm.len() / 2) as u32;
        e.send_frame(&Frame::Audio(AudioFrame {
            samples: n,
            pts: Some(0),
            data: vec![pcm],
        }))
        .unwrap();
        e.flush().unwrap();
        let mut a = Vec::new();
        while let Ok(pk) = e.receive_packet() {
            a.extend_from_slice(&pk.data);
        }
        let g = e.gapless_info();
        (a, g)
    };

    let (adts_a, info_a) = make_aac(pcm_a);
    let (adts_b, info_b) = make_aac(pcm_b);
    println!("Stream A gapless: {info_a:?}");
    println!("  iTunSMPB: {}", info_a.format_itunsmpb());
    println!("Stream B gapless: {info_b:?}");
    println!("  iTunSMPB: {}", info_b.format_itunsmpb());

    let mut concat = adts_a.clone();
    concat.extend_from_slice(&adts_b);
    let path = std::env::temp_dir().join("oxideav_aac_concat.aac");
    std::fs::write(&path, &concat).unwrap();
    println!("\nWrote {} bytes to {}", concat.len(), path.display());

    // The composite gapless triple a player gets if it concatenates the
    // raw ADTS bitstreams: A's priming, both pads + B's priming as
    // total padding, sum of valid_samples.
    let composite = GaplessInfo::new(
        info_a.encoder_delay,
        info_a.padding_samples + info_b.encoder_delay + info_b.padding_samples,
        info_a.valid_samples + info_b.valid_samples,
    );
    println!("\nComposite (player view): {composite:?}");
    println!("  iTunSMPB: {}", composite.format_itunsmpb());

    println!(
        "\nValidate with:\n  ffmpeg -hide_banner -loglevel warning -i {} -f s16le -ar 44100 /tmp/concat.s16",
        path.display()
    );
}

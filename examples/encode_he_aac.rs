//! Tiny utility: encode a 1 kHz sine @ 48 kHz to HE-AACv1 and write to
//! `/tmp/oxideav_he_aac.aac` — useful for sanity-checking with ffmpeg:
//!
//!   cargo run --release --example encode_he_aac
//!   ffmpeg -y -i /tmp/oxideav_he_aac.aac -f wav /tmp/oxideav_he_aac.wav
//!   ffmpeg -hide_banner -i /tmp/oxideav_he_aac.wav -af astats=measure=rms:measure_overall=Overall_RMS_level -f null - 2>&1

use oxideav_aac::he_aac_encoder::HeAacMonoEncoder;
use oxideav_core::Encoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, SampleFormat, TimeBase};
use std::fs::File;
use std::io::Write;

fn main() {
    let high_rate = 48_000u32;
    let freq = 1000.0f32;
    let secs = 1.0f32;
    let total = (high_rate as f32 * secs) as usize;
    let mut bytes = Vec::with_capacity(total * 2);
    for i in 0..total {
        let t = i as f32 / high_rate as f32;
        let v = (2.0 * std::f32::consts::PI * freq * t).sin() * 0.3;
        let s = (v * 32767.0) as i16;
        bytes.extend_from_slice(&s.to_le_bytes());
    }
    let mut params = CodecParameters::audio(CodecId::new("aac"));
    params.sample_rate = Some(high_rate);
    params.channels = Some(1);
    params.bit_rate = Some(32_000);
    let mut enc = HeAacMonoEncoder::new(&params).expect("enc");
    let af = AudioFrame {
        format: SampleFormat::S16,
        channels: 1,
        sample_rate: high_rate,
        samples: total as u32,
        pts: Some(0),
        time_base: TimeBase::new(1, high_rate as i64),
        data: vec![bytes],
    };
    enc.send_frame(&Frame::Audio(af)).expect("send");
    enc.flush().expect("flush");
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/tmp/oxideav_he_aac.aac".into());
    let mut out = File::create(&path).expect("open out");
    let mut frames = 0usize;
    let mut byteslen = 0usize;
    while let Ok(pkt) = enc.receive_packet() {
        out.write_all(&pkt.data).expect("write");
        frames += 1;
        byteslen += pkt.data.len();
    }
    println!("wrote {byteslen} bytes / {frames} ADTS frames to {path}");
}

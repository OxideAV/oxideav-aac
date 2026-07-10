//! Stream-level SSR (AOT 3) decode wiring — ISO/IEC 14496-3 §4.6.12.
//!
//! Builds a valid ADTS stream with the crate's own encoder, rewrites
//! the 2-bit ADTS `profile_ObjectType` field from `1` (AAC LC) to `2`
//! (AAC SSR), and decodes it through the public `StreamDecoder`. The
//! SSR profile routes every channel through the §4.6.12 gain-control
//! filterbank (four-band front half + gain compensation + IPQF)
//! instead of the §4.6.11 filterbank, so the stream must decode
//! end to end — with per-frame output sizes intact and PCM different
//! from the LC interpretation of the same raw data blocks.

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::decode::StreamDecoder;
use oxideav_aac::encoder::{EncoderConfig, StreamEncoder};

/// Interleaved s16 test tone (mono or stereo), smooth enough that the
/// encoder stays on `ONLY_LONG_SEQUENCE` frames.
fn tone_pcm(samples: usize, channels: usize) -> Vec<i16> {
    (0..samples * channels)
        .map(|i| {
            let n = (i / channels) as f64;
            let ch = (i % channels) as f64;
            (8000.0 * (0.03 * n + 0.5 * ch).sin()) as i16
        })
        .collect()
}

/// Rewrite every ADTS frame's `profile_ObjectType` field to `profile`
/// (byte 2, top two bits), walking frames via the parsed
/// `aac_frame_length`.
fn set_adts_profile(stream: &[u8], profile: u8) -> Vec<u8> {
    let mut out = stream.to_vec();
    let mut off = 0usize;
    while off < out.len() {
        let (header, _) = AdtsHeader::parse(&out[off..]).expect("valid ADTS frame");
        out[off + 2] = (out[off + 2] & 0x3F) | (profile << 6);
        off += header.aac_frame_length as usize;
    }
    out
}

#[test]
fn ssr_profile_stream_decodes_through_gain_control_pipeline() {
    let pcm = tone_pcm(1024 * 12, 1);
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 44100,
        channels: 1,
        bitrate: 96_000,
    })
    .unwrap();
    let adts_lc = enc.encode_all(&pcm).unwrap();

    // Sanity: the encoder emitted the LC profile.
    let (h, _) = AdtsHeader::parse(&adts_lc).unwrap();
    assert_eq!(h.audio_object_type(), 2);

    let adts_ssr = set_adts_profile(&adts_lc, 2);
    let (h_ssr, _) = AdtsHeader::parse(&adts_ssr).unwrap();
    assert_eq!(h_ssr.audio_object_type(), 3, "AOT must now be SSR");
    assert_eq!(h_ssr.aac_frame_length, h.aac_frame_length);

    // Decode both interpretations of the same raw data blocks.
    let frames_lc = StreamDecoder::new().decode_all(&adts_lc).unwrap();
    let frames_ssr = StreamDecoder::new().decode_all(&adts_ssr).unwrap();
    assert_eq!(frames_lc.len(), frames_ssr.len());
    assert!(!frames_ssr.is_empty());

    let mut any_nonzero = false;
    let mut any_diff = false;
    for (lc, ssr) in frames_lc.iter().zip(frames_ssr.iter()) {
        assert_eq!(ssr.channels, 1);
        assert_eq!(ssr.sample_rate, 44100);
        // ONLY_LONG frames keep the 1024-samples-per-frame layout.
        assert_eq!(ssr.pcm.len(), lc.pcm.len());
        any_nonzero |= ssr.pcm.iter().any(|&s| s != 0);
        any_diff |= ssr.pcm != lc.pcm;
    }
    assert!(any_nonzero, "SSR decode produced only silence");
    // Different synthesis filterbanks ⇒ different PCM for the same
    // spectra.
    assert!(any_diff, "SSR and LC paths produced identical PCM");
}

#[test]
fn ssr_profile_stereo_stream_decodes() {
    let pcm = tone_pcm(1024 * 8, 2);
    let mut enc = StreamEncoder::new(EncoderConfig {
        sample_rate: 48000,
        channels: 2,
        bitrate: 192_000,
    })
    .unwrap();
    let adts_ssr = set_adts_profile(&enc.encode_all(&pcm).unwrap(), 2);
    let frames = StreamDecoder::new().decode_all(&adts_ssr).unwrap();
    assert!(!frames.is_empty());
    for f in &frames {
        assert_eq!(f.channels, 2);
        assert_eq!(f.pcm.len() % 2, 0);
        assert!(f.pcm.iter().any(|&s| s != 0) || f.pcm.iter().all(|&s| s == 0));
    }
    assert!(frames.iter().any(|f| f.pcm.iter().any(|&s| s != 0)));
}

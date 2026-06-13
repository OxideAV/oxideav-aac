//! §4.6.11 filterbank integration over a real ADTS fixture.
//!
//! [`oxideav_aac::filterbank::Filterbank`] is unit-tested in `src/`
//! for the IMDCT phase, the sine/KBD window construction, and the
//! perfect-reconstruction (TDAC) overlap-add against synthetic
//! analysis frames. This driver closes the loop end to end: it walks
//! the `aac-lc-mono-44100-64kbps-adts` fixture frame by frame, decodes
//! each SCE channel to a window-major spectrum via the round-284
//! pipeline, and feeds every frame to a single stateful `Filterbank`
//! so the §4.6.11.3.3 inter-frame overlap-add tail is carried across
//! the whole stream.
//!
//! Assertions stay structural (finite PCM, correct frame length, an
//! overlap tail that actually couples consecutive frames). The full
//! byte-exact `expected.wav` comparison needs M/S (§4.6.8.1),
//! intensity (§4.6.8.2), and PNS (§4.6.13) synthesis, which are later
//! tools — this driver pins the filterbank itself against genuine
//! decoded spectra rather than hand-built vectors.
//!
//! The test skips cleanly when `docs/` is absent (standalone-repo CI
//! layout), matching `docs_adts_corpus.rs`.

use std::fs;
use std::path::PathBuf;

use oxideav_aac::adts::AdtsHeader;
use oxideav_aac::decoded_spectrum::decode_channel_spectrum;
use oxideav_aac::filterbank::Filterbank;
use oxideav_aac::ics_body::IcsBody;
use oxideav_aac::raw_data_block::{Element, IdSynEle, Walker};
use oxideav_aac::spectral_data::SpectralData;
use oxideav_aac::swb_offset::LONG_WINDOW_LEN;
use oxideav_core::bits::BitReader;

fn mono_fixture() -> Option<Vec<u8>> {
    let path = PathBuf::from("../../docs/audio/aac/fixtures")
        .join("aac-lc-mono-44100-64kbps-adts")
        .join("input.aac");
    match fs::read(&path) {
        Ok(d) => Some(d),
        Err(_) => {
            eprintln!("skip filterbank integration: missing {}", path.display());
            None
        }
    }
}

#[test]
fn mono_fixture_filterbank_runs_finite_and_couples_frames() {
    let Some(data) = mono_fixture() else {
        return;
    };

    let mut fb = Filterbank::new();
    let mut frame_pcms: Vec<Vec<f64>> = Vec::new();
    let mut pos = 0usize;

    while pos + 7 <= data.len() {
        let (header, payload_offset) = AdtsHeader::parse(&data[pos..]).expect("adts header");
        let frame_len = header.aac_frame_length as usize;
        assert!(pos + frame_len <= data.len(), "truncated frame at {pos}");
        let payload = &data[pos + payload_offset..pos + frame_len];
        let aot = header.audio_object_type();
        let fs_index = header.sampling_frequency_index;

        let mut reader = BitReader::new(payload);
        for _ in 0..header.number_of_raw_data_blocks_in_frame {
            loop {
                let elem = Walker::new(&mut reader)
                    .next_element()
                    .expect("walker")
                    .expect("payload ended before END");
                match elem {
                    Element::ChannelElement {
                        kind: IdSynEle::Sce | IdSynEle::Lfe,
                        ..
                    } => {
                        let body =
                            IcsBody::parse(&mut reader, aot, fs_index, false).expect("ics_body");
                        let ics = body.ics_info.clone().expect("inline ics_info");
                        let spectral =
                            SpectralData::parse(&mut reader, &ics, &body.section_data, fs_index)
                                .expect("spectral_data");
                        let spec = decode_channel_spectrum(&body, &ics, &spectral, aot, fs_index)
                            .expect("pipeline");
                        // §4.6.11 — synthesize one frame of PCM.
                        let pcm = fb.synthesize(&spec, &ics).expect("filterbank");
                        assert_eq!(
                            pcm.len(),
                            LONG_WINDOW_LEN as usize,
                            "filterbank emits one long window of PCM per frame"
                        );
                        assert!(pcm.iter().all(|v| v.is_finite()), "all PCM samples finite");
                        frame_pcms.push(pcm);
                    }
                    Element::End => break,
                    // The mono fixture is pure SCE; anything else is
                    // either fill/data (ignored) or unexpected.
                    Element::Fill { .. } | Element::Data { .. } | Element::ProgramConfig(_) => {}
                    Element::ChannelElement { kind, .. } => {
                        panic!("unexpected channel element {} in mono fixture", kind.name())
                    }
                }
            }
        }
        pos += frame_len;
    }

    assert!(
        frame_pcms.len() >= 2,
        "fixture should decode multiple frames, got {}",
        frame_pcms.len()
    );

    // §4.6.11.3.3: the first PCM frame is overlap-added against a
    // zero tail (fresh Filterbank), so it equals just the windowed
    // IMDCT left half. A second, independent Filterbank fed only the
    // SECOND frame's spectrum (with a zero tail) must therefore differ
    // from the streamed second frame — proving the overlap tail from
    // frame 1 actually coupled into frame 2.
    //
    // Re-decode the second frame's spectrum in isolation.
    let second = decode_second_frame_isolated(&data);
    if let Some(iso) = second {
        let streamed = &frame_pcms[1];
        let mut differs = false;
        for (a, b) in streamed.iter().zip(iso.iter()) {
            if (a - b).abs() > 1e-12 {
                differs = true;
                break;
            }
        }
        assert!(
            differs,
            "streamed frame 2 must include the frame-1 overlap tail"
        );
    }
}

/// Decode only the second frame's SCE spectrum and run it through a
/// FRESH filterbank (zero overlap tail). Returns `None` if the stream
/// has fewer than two frames.
fn decode_second_frame_isolated(data: &[u8]) -> Option<Vec<f64>> {
    let mut pos = 0usize;
    let mut frame_idx = 0usize;
    while pos + 7 <= data.len() {
        let (header, payload_offset) = AdtsHeader::parse(&data[pos..]).ok()?;
        let frame_len = header.aac_frame_length as usize;
        if pos + frame_len > data.len() {
            return None;
        }
        if frame_idx == 1 {
            let payload = &data[pos + payload_offset..pos + frame_len];
            let aot = header.audio_object_type();
            let fs_index = header.sampling_frequency_index;
            let mut reader = BitReader::new(payload);
            for _ in 0..header.number_of_raw_data_blocks_in_frame {
                loop {
                    let elem = Walker::new(&mut reader).next_element().ok()??;
                    match elem {
                        Element::ChannelElement {
                            kind: IdSynEle::Sce | IdSynEle::Lfe,
                            ..
                        } => {
                            let body = IcsBody::parse(&mut reader, aot, fs_index, false).ok()?;
                            let ics = body.ics_info.clone()?;
                            let spectral = SpectralData::parse(
                                &mut reader,
                                &ics,
                                &body.section_data,
                                fs_index,
                            )
                            .ok()?;
                            let spec =
                                decode_channel_spectrum(&body, &ics, &spectral, aot, fs_index)
                                    .ok()?;
                            let mut fresh = Filterbank::new();
                            return fresh.synthesize(&spec, &ics).ok();
                        }
                        Element::End => break,
                        _ => {}
                    }
                }
            }
        }
        frame_idx += 1;
        pos += frame_len;
    }
    None
}

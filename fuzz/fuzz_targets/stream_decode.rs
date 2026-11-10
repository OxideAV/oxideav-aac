//! The stream decoder over arbitrary ADTS-framed input: any run of
//! frames, damaged or not, under any of the decoder's options (the
//! per-frame PNS seed, the shared pair seed, core-only, declaration
//! order) must decode or fail without panicking, and a frame that fails
//! must leave the decoder able to take the next.
//!
//! Byte 0 selects the options and the mode. In ADTS mode each frame is
//! decoded through its own header ([`StreamDecoder::decode_adts_frame`],
//! with its CRC layer). In raw mode each frame's payload is handed to
//! [`StreamDecoder::decode_raw_data_block`] under the configuration
//! bytes 1 and 2 give (object type, sampling frequency index,
//! `channelConfiguration`), whatever the header says, so real frames
//! meet configurations they were not coded for. Frames are found by
//! their ADTS sync words; bytes that do not start a frame are skipped
//! one at a time.

#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_aac::adts::{ADTS_SAMPLE_RATES_HZ, AdtsHeader};
use oxideav_aac::decode::StreamDecoder;

/// Frames decoded per input at most, to keep each run short.
const MAX_FRAMES: usize = 48;

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }
    let options = data[0];
    let mut dec = StreamDecoder::new();
    dec.set_pns_frame_seed(options & 1 != 0);
    dec.set_shared_pns_seed(options & 2 != 0);
    dec.set_core_only(options & 4 != 0);
    dec.set_declaration_order(options & 8 != 0);
    let adts_mode = options & 0x10 != 0;
    // AAC LC, Main, LTP, or LC again: the object types whose decode
    // paths share the element decoders.
    let aot = [2u8, 1, 4, 2][usize::from(options >> 6)];
    let fs_index = data[1] % ADTS_SAMPLE_RATES_HZ.len() as u8;
    let sample_rate = ADTS_SAMPLE_RATES_HZ[usize::from(fs_index)];
    let channel_configuration = data[2] % 8;

    let body = &data[3..];
    let mut at = 0;
    let mut frames = 0;
    while at < body.len() && frames < MAX_FRAMES {
        let Ok((header, payload_offset)) = AdtsHeader::parse(&body[at..]) else {
            at += 1;
            continue;
        };
        let len = header.aac_frame_length as usize;
        if len < payload_offset || at + len > body.len() {
            at += 1;
            continue;
        }
        let frame = &body[at..at + len];
        let _ = if adts_mode {
            dec.decode_adts_frame(frame)
        } else {
            dec.decode_raw_data_block(
                aot,
                fs_index,
                sample_rate,
                channel_configuration,
                1,
                &frame[payload_offset..],
            )
        };
        at += len;
        frames += 1;
    }
});

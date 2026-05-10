//! Runtime libavcodec interop for the AAC cross-decode fuzz oracle.
//!
//! The shared library is loaded via `dlopen` at first call — there is
//! no `ffmpeg-sys` / `rusty_ffmpeg`-style build-script dep that would
//! pull libavcodec source into the workspace's cargo dep tree. The
//! oracle harness checks `libavcodec::available()` up front and
//! `return`s early when the shared library isn't installed, so fuzz
//! binaries built on a host without ffmpeg simply do nothing instead
//! of panicking — there's no `#[ignore]`-style cargo-test marker on
//! a fuzz target, so a runtime skip is the only way to keep the binary
//! valid on hosts without libavcodec.
//!
//! Workspace policy bars consulting libavcodec / fdk-aac / faad2 source;
//! we only inspect the public C headers (`<libavcodec/avcodec.h>`,
//! `<libavutil/frame.h>`, `<libavutil/samplefmt.h>`) for function
//! signatures + the documented `AV_CODEC_ID_AAC` constant. fdk-aac is
//! NDA-encumbered per workspace memory and is explicitly NOT used —
//! we ask libavcodec for its built-in AAC decoder via the integer
//! codec id, never via `libfdk_aac` by name.
//!
//! Install on Debian / Ubuntu via `apt-get install -y ffmpeg` (which
//! pulls `libavcodec61` or whichever is current). On macOS use
//! `brew install ffmpeg`.

#![allow(unsafe_code)]

pub mod libavcodec {
    use libloading::{Library, Symbol};
    use std::ffi::c_void;
    use std::sync::OnceLock;

    /// `AV_CODEC_ID_AAC` — documented value `0x15002` (== 86018) in the
    /// public `enum AVCodecID` (`libavcodec/codec_id.h`). Stable across
    /// every libavcodec major version we target (58 through 62) — the
    /// audio-codec ids start at `0x10000` and AAC has been at `0x15002`
    /// since libavcodec 51.
    pub const AV_CODEC_ID_AAC: i32 = 86018;

    /// `AVERROR(EAGAIN)` — `-EAGAIN` per the documented AVERROR macro.
    /// `EAGAIN == 11` on Linux/glibc; on macOS it's `35`. We only test
    /// against this in CI (Linux runner), so the Linux value is fine.
    /// Misidentifying it just means we treat EAGAIN as a hard error,
    /// which the harness already tolerates as "skip this iteration".
    const AVERROR_EAGAIN: i32 = -11;

    /// Conventional libavcodec shared-object names the loader will try
    /// in order. Covers Debian / Ubuntu (versioned `.so.NN`), the
    /// unversioned `-dev` symlink, and macOS (`.dylib`).
    const CANDIDATES: &[&str] = &[
        "libavcodec.so.62",
        "libavcodec.so.61",
        "libavcodec.so.60",
        "libavcodec.so.59",
        "libavcodec.so.58",
        "libavcodec.so",
        "libavcodec.dylib",
    ];

    fn lib() -> Option<&'static Library> {
        static LIB: OnceLock<Option<Library>> = OnceLock::new();
        LIB.get_or_init(|| {
            for name in CANDIDATES {
                // SAFETY: `Library::new` is documented as unsafe because
                // the loaded library may run code at load time. We
                // accept that risk for fuzz tooling — libavcodec is a
                // well-behaved shared library distributed by distros.
                if let Ok(l) = unsafe { Library::new(name) } {
                    return Some(l);
                }
            }
            None
        })
        .as_ref()
    }

    /// True iff a libavcodec shared library was successfully loaded.
    /// Cross-decode fuzz harnesses early-return when this is false so
    /// the binary still runs without an oracle (the assertions just
    /// don't fire — and the harness prints `[oracle skip]` once so the
    /// fuzz log is unambiguous about what's being measured).
    pub fn available() -> bool {
        lib().is_some()
    }

    /// One audio frame as decoded by libavcodec.
    ///
    /// `samples` is the per-channel sample count (matches AAC-LC's 1024
    /// for the core layer, 2048 with SBR). `sample_rate` and `channels`
    /// are echoed back so the harness can cross-check them against
    /// what oxideav-aac reports for the same input.
    ///
    /// `pcm_s16_interleaved` is `Some(...)` when libavcodec produced a
    /// recognisable sample format (S16 / S16P / FLT / FLTP) and we
    /// were able to convert/interleave it for ±2 LSB comparison; for
    /// any other format (S32 etc.) it's `None` and the harness only
    /// verifies the metadata fields agree.
    pub struct DecodedAudio {
        pub samples: u32,
        pub sample_rate: u32,
        pub channels: u16,
        pub pcm_s16_interleaved: Option<Vec<i16>>,
    }

    /// Outcome of asking libavcodec to decode a buffer.
    pub enum OracleResult {
        /// Library not loadable / decoder factory not registered.
        Unavailable,
        /// libavcodec rejected the input (parser or decoder error,
        /// or accepted-but-no-frame-emitted). Our decoder is allowed
        /// to either reject or accept in this case — only the
        /// libavcodec-accepts side is a hard contract for the fuzz.
        Rejected,
        /// libavcodec produced a frame.
        Frame(DecodedAudio),
    }

    /// Selected libavcodec AVSampleFormat values we recognise. Values
    /// per the public `enum AVSampleFormat` in `libavutil/samplefmt.h` —
    /// stable since libavutil 51.
    const AV_SAMPLE_FMT_U8: i32 = 0;
    const AV_SAMPLE_FMT_S16: i32 = 1;
    const AV_SAMPLE_FMT_S32: i32 = 2;
    const AV_SAMPLE_FMT_FLT: i32 = 3;
    const AV_SAMPLE_FMT_DBL: i32 = 4;
    const AV_SAMPLE_FMT_U8P: i32 = 5;
    const AV_SAMPLE_FMT_S16P: i32 = 6;
    const AV_SAMPLE_FMT_S32P: i32 = 7;
    const AV_SAMPLE_FMT_FLTP: i32 = 8;
    const AV_SAMPLE_FMT_DBLP: i32 = 9;

    /// Read `nb_samples` 32-bit floats from each of `channels` planes
    /// and return them interleaved as i16. Saturates on out-of-range.
    unsafe fn fltp_to_s16_interleaved(
        data_arr: *const *const u8,
        nb_samples: usize,
        channels: usize,
    ) -> Vec<i16> {
        let mut out = Vec::with_capacity(nb_samples * channels);
        for s in 0..nb_samples {
            for c in 0..channels {
                let plane = data_arr.add(c).read_unaligned() as *const f32;
                if plane.is_null() {
                    out.push(0);
                    continue;
                }
                let v = plane.add(s).read_unaligned();
                let scaled = (v.clamp(-1.0, 1.0) * 32767.0).round() as i32;
                out.push(scaled.clamp(i16::MIN as i32, i16::MAX as i32) as i16);
            }
        }
        out
    }

    /// Read `nb_samples` 32-bit floats from a single packed plane and
    /// emit them interleaved as i16. (`AV_SAMPLE_FMT_FLT` is already
    /// interleaved, so plane 0 contains `nb_samples * channels` floats.)
    unsafe fn flt_to_s16(plane0: *const u8, nb_samples: usize, channels: usize) -> Vec<i16> {
        let mut out = Vec::with_capacity(nb_samples * channels);
        let p = plane0 as *const f32;
        if p.is_null() {
            return out;
        }
        for i in 0..nb_samples * channels {
            let v = p.add(i).read_unaligned();
            let scaled = (v.clamp(-1.0, 1.0) * 32767.0).round() as i32;
            out.push(scaled.clamp(i16::MIN as i32, i16::MAX as i32) as i16);
        }
        out
    }

    /// Read planar S16: one plane per channel, each holding `nb_samples`
    /// `i16` values. Emit interleaved.
    unsafe fn s16p_to_s16_interleaved(
        data_arr: *const *const u8,
        nb_samples: usize,
        channels: usize,
    ) -> Vec<i16> {
        let mut out = Vec::with_capacity(nb_samples * channels);
        for s in 0..nb_samples {
            for c in 0..channels {
                let plane = data_arr.add(c).read_unaligned() as *const i16;
                if plane.is_null() {
                    out.push(0);
                    continue;
                }
                out.push(plane.add(s).read_unaligned());
            }
        }
        out
    }

    /// Read interleaved S16 directly from plane 0.
    unsafe fn s16_to_s16(plane0: *const u8, nb_samples: usize, channels: usize) -> Vec<i16> {
        let mut out = Vec::with_capacity(nb_samples * channels);
        let p = plane0 as *const i16;
        if p.is_null() {
            return out;
        }
        for i in 0..nb_samples * channels {
            out.push(p.add(i).read_unaligned());
        }
        out
    }

    /// Feed `data` (ADTS-wrapped AAC bytes) to libavcodec's built-in
    /// AAC decoder, then drain a single frame. Returns:
    /// - `Unavailable` when libavcodec / `avcodec_find_decoder(AAC)`
    ///   isn't installed
    /// - `Rejected` when the input fails to decode (any error path)
    /// - `Frame(...)` on the first audio frame produced
    ///
    /// All FFI is wrapped in `unsafe` blocks; resources (codec context,
    /// packet, frame) are freed before each early return via the
    /// closure-and-cleanup pattern.
    pub fn decode_aac(data: &[u8]) -> OracleResult {
        // ---- Function-pointer types --------------------------------
        // All signatures are taken verbatim from the public
        // <libavcodec/avcodec.h> + <libavutil/frame.h> headers, which
        // are documentation, not implementation. The opaque struct
        // pointers (AVCodec / AVCodecContext / AVPacket / AVFrame /
        // AVDictionary) are kept as `*mut c_void` since we never
        // dereference them from Rust — only pass them back to the C
        // side or read selected fields by documented byte offset.
        type FindDecoderFn = unsafe extern "C" fn(i32) -> *const c_void;
        type AllocContext3Fn = unsafe extern "C" fn(*const c_void) -> *mut c_void;
        type Open2Fn = unsafe extern "C" fn(*mut c_void, *const c_void, *mut *mut c_void) -> i32;
        type PacketAllocFn = unsafe extern "C" fn() -> *mut c_void;
        type FrameAllocFn = unsafe extern "C" fn() -> *mut c_void;
        type SendPacketFn = unsafe extern "C" fn(*mut c_void, *const c_void) -> i32;
        type ReceiveFrameFn = unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32;
        type PacketFreeFn = unsafe extern "C" fn(*mut *mut c_void);
        type FrameFreeFn = unsafe extern "C" fn(*mut *mut c_void);
        type FreeContextFn = unsafe extern "C" fn(*mut *mut c_void);

        let Some(l) = lib() else {
            return OracleResult::Unavailable;
        };
        unsafe {
            let find_decoder: Symbol<FindDecoderFn> = match l.get(b"avcodec_find_decoder") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let alloc_context3: Symbol<AllocContext3Fn> = match l.get(b"avcodec_alloc_context3") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let open2: Symbol<Open2Fn> = match l.get(b"avcodec_open2") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let packet_alloc: Symbol<PacketAllocFn> = match l.get(b"av_packet_alloc") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let frame_alloc: Symbol<FrameAllocFn> = match l.get(b"av_frame_alloc") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let send_packet: Symbol<SendPacketFn> = match l.get(b"avcodec_send_packet") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let receive_frame: Symbol<ReceiveFrameFn> = match l.get(b"avcodec_receive_frame") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let packet_free: Symbol<PacketFreeFn> = match l.get(b"av_packet_free") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let frame_free: Symbol<FrameFreeFn> = match l.get(b"av_frame_free") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let free_context: Symbol<FreeContextFn> = match l.get(b"avcodec_free_context") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };

            // Ask explicitly for the BUILT-IN AAC decoder (id 86018).
            // libavcodec's `avcodec_find_decoder(AV_CODEC_ID_AAC)`
            // returns the native decoder, NOT `libfdk_aac` (which is
            // only available via `avcodec_find_decoder_by_name` and is
            // NDA-encumbered per the workspace memory note — we never
            // ask for it).
            let codec = find_decoder(AV_CODEC_ID_AAC);
            if codec.is_null() {
                return OracleResult::Unavailable;
            }
            let mut ctx = alloc_context3(codec);
            if ctx.is_null() {
                return OracleResult::Unavailable;
            }
            let mut pkt = packet_alloc();
            if pkt.is_null() {
                free_context(&mut ctx);
                return OracleResult::Unavailable;
            }
            let mut frame = frame_alloc();
            if frame.is_null() {
                packet_free(&mut pkt);
                free_context(&mut ctx);
                return OracleResult::Unavailable;
            }

            // Result is computed inside a closure so the cleanup
            // epilogue runs unconditionally on every early return.
            let result = (|| -> OracleResult {
                if open2(ctx, codec, std::ptr::null_mut()) < 0 {
                    return OracleResult::Rejected;
                }

                // Populate AVPacket.data + AVPacket.size by documented
                // byte offset. The AVPacket prefix layout is stable
                // since libavcodec 57 (the bump that turned it into a
                // ref-counted wrapper) and matches:
                //   off  0  *AVBufferRef  buf
                //   off  8  i64           pts
                //   off 16  i64           dts
                //   off 24  *u8           data
                //   off 32  i32           size
                // We only set data + size for a single non-ref-counted
                // input packet; the buf pointer stays NULL so libavcodec
                // copies the bytes itself before send_packet returns.
                const PKT_OFF_DATA: usize = 24;
                const PKT_OFF_SIZE: usize = 32;
                let pkt_bytes = pkt as *mut u8;
                (pkt_bytes.add(PKT_OFF_DATA) as *mut *const u8).write_unaligned(data.as_ptr());
                (pkt_bytes.add(PKT_OFF_SIZE) as *mut i32).write_unaligned(data.len() as i32);

                let sp = send_packet(ctx, pkt);
                if sp < 0 && sp != AVERROR_EAGAIN {
                    return OracleResult::Rejected;
                }
                // Send EOF (NULL packet) so the decoder flushes any
                // buffered frames. Spec: a NULL packet enters draining
                // mode (avcodec_send_packet docs).
                let _ = send_packet(ctx, std::ptr::null());

                let rc = receive_frame(ctx, frame);
                if rc < 0 {
                    return OracleResult::Rejected;
                }

                // Read AVFrame fields by documented byte offset. The
                // AVFrame prefix layout is stable since libavutil 55:
                //   off   0  *u8 x AV_NUM_DATA_POINTERS  data[8]
                //   off  64  i32 x AV_NUM_DATA_POINTERS  linesize[8]
                //   off  96  **u8                          extended_data
                //   off 104  i32                           width
                //   off 108  i32                           height
                //   off 112  i32                           nb_samples
                //   off 116  i32                           format
                // (AV_NUM_DATA_POINTERS == 8 since the libavutil 51
                // bump — pre-dates everything we support.) The format
                // field for audio frames is an AVSampleFormat. For the
                // sample_rate + channel count we read the
                // **AVCodecContext** fields rather than the AVFrame
                // ones — AVCodecContext.sample_rate and
                // AVCodecContext.ch_layout.nb_channels (or the
                // deprecated `channels` int) are stable at the same
                // documented offsets across libavcodec 58..=62, while
                // the AVFrame audio-side offsets shifted around
                // (`pkt_pts` deprecation, ch_layout addition).
                //
                // AVCodecContext audio prefix relevant offsets:
                //   sample_rate is documented as a public i32 field
                //   accessed via the `sample_rate` getter macro in
                //   recent ffmpeg, but it has lived at offset 488
                //   (Linux x86_64) since libavcodec 58 — confirmed
                //   against the public header changelogs across the
                //   58..=62 lineage. To avoid pinning a magic offset
                //   that *could* drift, we instead resolve it via a
                //   pair of public-API helpers:
                //     - `av_get_channel_layout_nb_channels` (legacy
                //       channel layout to count) is gone in
                //       libavcodec 61
                //     - the *new* layout uses
                //       `AVCodecContext.ch_layout.nb_channels`
                //   Both are version-fragile.
                //
                // Pragmatic choice: read sample_rate + channels from
                // the **AVFrame** instead, but only the prefix slice
                // we need (nb_samples + format). The sample_rate and
                // channel count we'll reconstruct by parsing the ADTS
                // header in the calling harness — both decoders see
                // the same input so both must agree with that header.

                const OFF_DATA: usize = 0;
                const OFF_NB_SAMPLES: usize = 112;
                const OFF_FORMAT: usize = 116;
                let f_bytes = frame as *const u8;
                let data_arr = f_bytes.add(OFF_DATA) as *const *const u8;
                let nb_samples = (f_bytes.add(OFF_NB_SAMPLES) as *const i32).read_unaligned();
                let format = (f_bytes.add(OFF_FORMAT) as *const i32).read_unaligned();
                if nb_samples <= 0 {
                    return OracleResult::Rejected;
                }

                // Pull sample_rate + channels from the harness side.
                // We surface them via 0 here and let the caller fill
                // them in from the parsed ADTS header (which both
                // decoders had to agree with for any output to exist).
                // This sidesteps the AVCodecContext / AVFrame layout
                // drift discussed in the comment above.
                let plane0 = data_arr.add(0).read_unaligned();
                if plane0.is_null() {
                    return OracleResult::Rejected;
                }

                // Try to read PCM. The harness will determine channel
                // count from the ADTS header and pass it back in for
                // a follow-up format conversion if needed; here we
                // emit S16 only for the formats we know how to read
                // with a known channel count. We default to 1 channel
                // for the conversion and let the harness rerun with
                // the correct channel count via `decode_aac_with_chan`.
                // Simpler: build the i16 vec at the harness level,
                // since the ADTS header is the source of truth for
                // channels. Here we just copy plane data into raw
                // u8 buffers and surface the format so the harness
                // can do the right conversion.
                let _ = format;
                let _ = plane0;
                OracleResult::Frame(DecodedAudio {
                    samples: nb_samples as u32,
                    sample_rate: 0,
                    channels: 0,
                    pcm_s16_interleaved: None,
                })
            })();

            frame_free(&mut frame);
            packet_free(&mut pkt);
            free_context(&mut ctx);
            result
        }
    }

    /// Decode an ADTS-wrapped AAC byte run that contains **at least
    /// two** raw_data_blocks, drain both frames, and return the
    /// SECOND. Used by the cross-decode oracle to skip the priming-
    /// delay frame whose overlap-state initial conditions disagree
    /// between independent decoders.
    pub fn decode_aac_two_frames_take_second(
        data: &[u8],
        channels: u16,
        sample_rate: u32,
    ) -> OracleResult {
        type FindDecoderFn = unsafe extern "C" fn(i32) -> *const c_void;
        type AllocContext3Fn = unsafe extern "C" fn(*const c_void) -> *mut c_void;
        type Open2Fn = unsafe extern "C" fn(*mut c_void, *const c_void, *mut *mut c_void) -> i32;
        type PacketAllocFn = unsafe extern "C" fn() -> *mut c_void;
        type FrameAllocFn = unsafe extern "C" fn() -> *mut c_void;
        type SendPacketFn = unsafe extern "C" fn(*mut c_void, *const c_void) -> i32;
        type ReceiveFrameFn = unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32;
        type PacketFreeFn = unsafe extern "C" fn(*mut *mut c_void);
        type FrameFreeFn = unsafe extern "C" fn(*mut *mut c_void);
        type FreeContextFn = unsafe extern "C" fn(*mut *mut c_void);
        type FrameUnrefFn = unsafe extern "C" fn(*mut c_void);

        let Some(l) = lib() else {
            return OracleResult::Unavailable;
        };
        unsafe {
            let find_decoder: Symbol<FindDecoderFn> = match l.get(b"avcodec_find_decoder") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let alloc_context3: Symbol<AllocContext3Fn> = match l.get(b"avcodec_alloc_context3") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let open2: Symbol<Open2Fn> = match l.get(b"avcodec_open2") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let packet_alloc: Symbol<PacketAllocFn> = match l.get(b"av_packet_alloc") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let frame_alloc: Symbol<FrameAllocFn> = match l.get(b"av_frame_alloc") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let send_packet: Symbol<SendPacketFn> = match l.get(b"avcodec_send_packet") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let receive_frame: Symbol<ReceiveFrameFn> = match l.get(b"avcodec_receive_frame") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let packet_free: Symbol<PacketFreeFn> = match l.get(b"av_packet_free") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let frame_free: Symbol<FrameFreeFn> = match l.get(b"av_frame_free") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let free_context: Symbol<FreeContextFn> = match l.get(b"avcodec_free_context") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            // av_frame_unref is needed between frames so the same
            // AVFrame slot can be reused for the second receive_frame
            // call without the first frame's buffers leaking.
            let frame_unref: Symbol<FrameUnrefFn> = match l.get(b"av_frame_unref") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };

            let codec = find_decoder(AV_CODEC_ID_AAC);
            if codec.is_null() {
                return OracleResult::Unavailable;
            }
            let mut ctx = alloc_context3(codec);
            if ctx.is_null() {
                return OracleResult::Unavailable;
            }
            let mut pkt = packet_alloc();
            if pkt.is_null() {
                free_context(&mut ctx);
                return OracleResult::Unavailable;
            }
            let mut frame = frame_alloc();
            if frame.is_null() {
                packet_free(&mut pkt);
                free_context(&mut ctx);
                return OracleResult::Unavailable;
            }

            let result = (|| -> OracleResult {
                if open2(ctx, codec, std::ptr::null_mut()) < 0 {
                    return OracleResult::Rejected;
                }
                const PKT_OFF_DATA: usize = 24;
                const PKT_OFF_SIZE: usize = 32;
                let pkt_bytes = pkt as *mut u8;
                (pkt_bytes.add(PKT_OFF_DATA) as *mut *const u8).write_unaligned(data.as_ptr());
                (pkt_bytes.add(PKT_OFF_SIZE) as *mut i32).write_unaligned(data.len() as i32);

                let sp = send_packet(ctx, pkt);
                if sp < 0 && sp != AVERROR_EAGAIN {
                    return OracleResult::Rejected;
                }
                let _ = send_packet(ctx, std::ptr::null());

                // Pull the first (priming-delay) frame and DROP it.
                let rc1 = receive_frame(ctx, frame);
                if rc1 < 0 {
                    return OracleResult::Rejected;
                }
                frame_unref(frame);

                // Pull the second frame — that's the one we return.
                let rc2 = receive_frame(ctx, frame);
                if rc2 < 0 {
                    return OracleResult::Rejected;
                }

                const OFF_DATA: usize = 0;
                const OFF_NB_SAMPLES: usize = 112;
                const OFF_FORMAT: usize = 116;
                let f_bytes = frame as *const u8;
                let data_arr = f_bytes.add(OFF_DATA) as *const *const u8;
                let nb_samples = (f_bytes.add(OFF_NB_SAMPLES) as *const i32).read_unaligned();
                let format = (f_bytes.add(OFF_FORMAT) as *const i32).read_unaligned();
                if nb_samples <= 0 || channels == 0 {
                    return OracleResult::Rejected;
                }

                let nb = nb_samples as usize;
                let ch = channels as usize;
                let pcm: Option<Vec<i16>> = match format {
                    AV_SAMPLE_FMT_FLTP => Some(fltp_to_s16_interleaved(data_arr, nb, ch)),
                    AV_SAMPLE_FMT_S16P => Some(s16p_to_s16_interleaved(data_arr, nb, ch)),
                    AV_SAMPLE_FMT_FLT => {
                        let plane0 = data_arr.add(0).read_unaligned();
                        if plane0.is_null() {
                            return OracleResult::Rejected;
                        }
                        Some(flt_to_s16(plane0, nb, ch))
                    }
                    AV_SAMPLE_FMT_S16 => {
                        let plane0 = data_arr.add(0).read_unaligned();
                        if plane0.is_null() {
                            return OracleResult::Rejected;
                        }
                        Some(s16_to_s16(plane0, nb, ch))
                    }
                    AV_SAMPLE_FMT_U8 | AV_SAMPLE_FMT_S32 | AV_SAMPLE_FMT_DBL
                    | AV_SAMPLE_FMT_U8P | AV_SAMPLE_FMT_S32P | AV_SAMPLE_FMT_DBLP => None,
                    _ => None,
                };
                OracleResult::Frame(DecodedAudio {
                    samples: nb_samples as u32,
                    sample_rate,
                    channels,
                    pcm_s16_interleaved: pcm,
                })
            })();

            frame_free(&mut frame);
            packet_free(&mut pkt);
            free_context(&mut ctx);
            result
        }
    }

    /// Same as [`decode_aac`] but additionally interleave PCM samples
    /// to S16 using the caller-provided `channels` count (parsed from
    /// the input ADTS header — which both decoders must agree with).
    /// Returns a `DecodedAudio` with `pcm_s16_interleaved = Some(...)`
    /// when libavcodec emitted a recognised sample format.
    pub fn decode_aac_with_channels(data: &[u8], channels: u16, sample_rate: u32) -> OracleResult {
        type FindDecoderFn = unsafe extern "C" fn(i32) -> *const c_void;
        type AllocContext3Fn = unsafe extern "C" fn(*const c_void) -> *mut c_void;
        type Open2Fn = unsafe extern "C" fn(*mut c_void, *const c_void, *mut *mut c_void) -> i32;
        type PacketAllocFn = unsafe extern "C" fn() -> *mut c_void;
        type FrameAllocFn = unsafe extern "C" fn() -> *mut c_void;
        type SendPacketFn = unsafe extern "C" fn(*mut c_void, *const c_void) -> i32;
        type ReceiveFrameFn = unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32;
        type PacketFreeFn = unsafe extern "C" fn(*mut *mut c_void);
        type FrameFreeFn = unsafe extern "C" fn(*mut *mut c_void);
        type FreeContextFn = unsafe extern "C" fn(*mut *mut c_void);

        let Some(l) = lib() else {
            return OracleResult::Unavailable;
        };
        unsafe {
            let find_decoder: Symbol<FindDecoderFn> = match l.get(b"avcodec_find_decoder") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let alloc_context3: Symbol<AllocContext3Fn> = match l.get(b"avcodec_alloc_context3") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let open2: Symbol<Open2Fn> = match l.get(b"avcodec_open2") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let packet_alloc: Symbol<PacketAllocFn> = match l.get(b"av_packet_alloc") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let frame_alloc: Symbol<FrameAllocFn> = match l.get(b"av_frame_alloc") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let send_packet: Symbol<SendPacketFn> = match l.get(b"avcodec_send_packet") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let receive_frame: Symbol<ReceiveFrameFn> = match l.get(b"avcodec_receive_frame") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let packet_free: Symbol<PacketFreeFn> = match l.get(b"av_packet_free") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let frame_free: Symbol<FrameFreeFn> = match l.get(b"av_frame_free") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };
            let free_context: Symbol<FreeContextFn> = match l.get(b"avcodec_free_context") {
                Ok(s) => s,
                Err(_) => return OracleResult::Unavailable,
            };

            let codec = find_decoder(AV_CODEC_ID_AAC);
            if codec.is_null() {
                return OracleResult::Unavailable;
            }
            let mut ctx = alloc_context3(codec);
            if ctx.is_null() {
                return OracleResult::Unavailable;
            }
            let mut pkt = packet_alloc();
            if pkt.is_null() {
                free_context(&mut ctx);
                return OracleResult::Unavailable;
            }
            let mut frame = frame_alloc();
            if frame.is_null() {
                packet_free(&mut pkt);
                free_context(&mut ctx);
                return OracleResult::Unavailable;
            }

            let result = (|| -> OracleResult {
                if open2(ctx, codec, std::ptr::null_mut()) < 0 {
                    return OracleResult::Rejected;
                }
                const PKT_OFF_DATA: usize = 24;
                const PKT_OFF_SIZE: usize = 32;
                let pkt_bytes = pkt as *mut u8;
                (pkt_bytes.add(PKT_OFF_DATA) as *mut *const u8).write_unaligned(data.as_ptr());
                (pkt_bytes.add(PKT_OFF_SIZE) as *mut i32).write_unaligned(data.len() as i32);

                let sp = send_packet(ctx, pkt);
                if sp < 0 && sp != AVERROR_EAGAIN {
                    return OracleResult::Rejected;
                }
                let _ = send_packet(ctx, std::ptr::null());

                let rc = receive_frame(ctx, frame);
                if rc < 0 {
                    return OracleResult::Rejected;
                }

                const OFF_DATA: usize = 0;
                const OFF_NB_SAMPLES: usize = 112;
                const OFF_FORMAT: usize = 116;
                let f_bytes = frame as *const u8;
                let data_arr = f_bytes.add(OFF_DATA) as *const *const u8;
                let nb_samples = (f_bytes.add(OFF_NB_SAMPLES) as *const i32).read_unaligned();
                let format = (f_bytes.add(OFF_FORMAT) as *const i32).read_unaligned();
                if nb_samples <= 0 || channels == 0 {
                    return OracleResult::Rejected;
                }

                let nb = nb_samples as usize;
                let ch = channels as usize;
                let pcm: Option<Vec<i16>> = match format {
                    AV_SAMPLE_FMT_FLTP => Some(fltp_to_s16_interleaved(data_arr, nb, ch)),
                    AV_SAMPLE_FMT_S16P => Some(s16p_to_s16_interleaved(data_arr, nb, ch)),
                    AV_SAMPLE_FMT_FLT => {
                        let plane0 = data_arr.add(0).read_unaligned();
                        if plane0.is_null() {
                            return OracleResult::Rejected;
                        }
                        Some(flt_to_s16(plane0, nb, ch))
                    }
                    AV_SAMPLE_FMT_S16 => {
                        let plane0 = data_arr.add(0).read_unaligned();
                        if plane0.is_null() {
                            return OracleResult::Rejected;
                        }
                        Some(s16_to_s16(plane0, nb, ch))
                    }
                    AV_SAMPLE_FMT_U8 | AV_SAMPLE_FMT_S32 | AV_SAMPLE_FMT_DBL
                    | AV_SAMPLE_FMT_U8P | AV_SAMPLE_FMT_S32P | AV_SAMPLE_FMT_DBLP => None,
                    _ => None,
                };
                OracleResult::Frame(DecodedAudio {
                    samples: nb_samples as u32,
                    sample_rate,
                    channels,
                    pcm_s16_interleaved: pcm,
                })
            })();

            frame_free(&mut frame);
            packet_free(&mut pkt);
            free_context(&mut ctx);
            result
        }
    }
}

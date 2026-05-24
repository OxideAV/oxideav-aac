//! `raw_data_block()` syntactic walker.
//!
//! ISO/IEC 14496-3 §4.4.2.1 defines `raw_data_block()` as a sequence
//! of *syntactic elements*, each prefixed by a 3-bit `id_syn_ele`
//! identifier (Table 4.71). The element types are:
//!
//! | id (binary) | id (decimal) | name | role                                       |
//! |-------------|--------------|------|--------------------------------------------|
//! | `0b000`     | 0            | SCE  | single-channel element (mono)              |
//! | `0b001`     | 1            | CPE  | channel-pair element                       |
//! | `0b010`     | 2            | CCE  | coupling channel element                   |
//! | `0b011`     | 3            | LFE  | low-frequency-effects element              |
//! | `0b100`     | 4            | DSE  | data stream element                        |
//! | `0b101`     | 5            | PCE  | program config element                     |
//! | `0b110`     | 6            | FIL  | fill element (padding / extension payload) |
//! | `0b111`     | 7            | END  | block terminator                           |
//!
//! After the terminating `END`, ISO/IEC 14496-3 §4.4.2.1 requires the
//! decoder to byte-align the bit-reader before the next
//! `raw_data_block()` begins. The walker performs that alignment so
//! the next call after `END` resumes on a fresh byte boundary.
//!
//! ## Phase 1 scope
//!
//! This module is the **syntactic skeleton** — the walker emits an
//! [`Element`] per `id_syn_ele` it encounters and stops at `END`.
//! Per-element bodies are handled as follows:
//!
//! * **SCE / CPE / CCE / LFE**: the walker reads the mandatory 4-bit
//!   `element_instance_tag` and then *stops body parsing*. The
//!   consumer must advance the [`BitReader`](oxideav_core::bits::BitReader)
//!   past the channel-element body itself; Phase 2 will absorb that
//!   logic. The emitted [`Element::ChannelElement`] carries the
//!   element kind and its tag.
//! * **FIL**: parsed as ISO/IEC 14496-3 §4.4.2.7 — 4-bit
//!   `count`, optional 8-bit `esc_count` escape (when `count == 15`,
//!   the real byte count is `count + esc_count − 1`), then *count*
//!   bytes of `extension_payload` which are skipped without
//!   interpretation. The emitted [`Element::Fill`] reports the byte
//!   length skipped.
//! * **DSE**: parsed as ISO/IEC 14496-3 §4.4.2.5 — 4-bit
//!   `element_instance_tag`, 1-bit `data_byte_align_flag`, 8-bit
//!   `count`, optional 8-bit `esc_count`, byte-align (if flag set),
//!   then *count* bytes of `data_stream_byte[]`.
//! * **PCE**: parsing deferred. Calling [`Walker::next_element`] on a
//!   `PCE` returns [`Error::UnsupportedElementSkip`] with id `5`.
//!   Phase 2 will provide the full §4.4.1.1 parse.
//! * **END**: emits [`Element::End`] and byte-aligns the reader.
//!   Subsequent calls return `None`.

use oxideav_core::bits::BitReader;

use crate::{Error, Result};

/// Syntactic element identifier — the 3-bit `id_syn_ele` field
/// defined in ISO/IEC 14496-3 Table 4.71.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum IdSynEle {
    /// `0b000` — single-channel element.
    Sce = 0,
    /// `0b001` — channel-pair element.
    Cpe = 1,
    /// `0b010` — coupling channel element.
    Cce = 2,
    /// `0b011` — low-frequency-effects element.
    Lfe = 3,
    /// `0b100` — data stream element.
    Dse = 4,
    /// `0b101` — program config element.
    Pce = 5,
    /// `0b110` — fill element.
    Fil = 6,
    /// `0b111` — raw-data-block terminator.
    End = 7,
}

impl IdSynEle {
    /// Map a 3-bit wire value (0..=7) to the corresponding variant.
    pub fn from_bits(bits: u8) -> Self {
        match bits & 0b111 {
            0 => IdSynEle::Sce,
            1 => IdSynEle::Cpe,
            2 => IdSynEle::Cce,
            3 => IdSynEle::Lfe,
            4 => IdSynEle::Dse,
            5 => IdSynEle::Pce,
            6 => IdSynEle::Fil,
            _ => IdSynEle::End,
        }
    }

    /// Short upper-case name as used in the spec table and the
    /// AAC_TRACE fixture corpus (`SCE`, `CPE`, `CCE`, `LFE`, `DSE`,
    /// `PCE`, `FIL`, `END`).
    pub fn name(self) -> &'static str {
        match self {
            IdSynEle::Sce => "SCE",
            IdSynEle::Cpe => "CPE",
            IdSynEle::Cce => "CCE",
            IdSynEle::Lfe => "LFE",
            IdSynEle::Dse => "DSE",
            IdSynEle::Pce => "PCE",
            IdSynEle::Fil => "FIL",
            IdSynEle::End => "END",
        }
    }
}

/// An event emitted by [`Walker::next_element`].
///
/// The walker emits exactly one event per `id_syn_ele` it consumes
/// and stops at `END`. For non-`End` events the bit-reader position
/// after the call reflects the bytes the walker itself consumed
/// (header + any per-element bookkeeping it parses); see the
/// per-variant docs for which bytes have been skipped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Element {
    /// SCE / CPE / CCE / LFE — channel element. The walker has
    /// consumed the 3-bit `id_syn_ele` and the 4-bit
    /// `element_instance_tag`. The channel-element body (`ics_info`,
    /// section data, scale factors, spectral data, …) starts at the
    /// current bit-reader position and is **not** parsed in Phase 1.
    ChannelElement {
        /// The channel element variant (`Sce`, `Cpe`, `Cce`, or
        /// `Lfe`).
        kind: IdSynEle,
        /// The 4-bit `element_instance_tag` read from the wire.
        element_instance_tag: u8,
    },
    /// FIL — fill element. The walker has consumed the 3-bit
    /// `id_syn_ele`, the 4-bit `count`, the optional 8-bit
    /// `esc_count`, and the resulting *count* `extension_payload`
    /// bytes.
    Fill {
        /// Total `extension_payload` bytes skipped (`count` after
        /// optional escape expansion).
        payload_bytes: u32,
    },
    /// DSE — data stream element. The walker has consumed the
    /// header (3-bit `id_syn_ele`, 4-bit `element_instance_tag`,
    /// 1-bit `data_byte_align_flag`, 8-bit `count`, optional 8-bit
    /// `esc_count`, optional byte-align) and the resulting *count*
    /// `data_stream_byte[]` values.
    Data {
        /// The 4-bit `element_instance_tag` read from the wire.
        element_instance_tag: u8,
        /// `true` ⇔ a `data_byte_align_flag == 1` was processed and
        /// the bit-reader was byte-aligned before the payload.
        byte_align_flag: bool,
        /// Total `data_stream_byte[]` bytes skipped (`count` after
        /// optional escape expansion).
        payload_bytes: u32,
    },
    /// END (`0b111`) — the raw-data-block terminator. The walker
    /// has consumed the 3-bit `id_syn_ele` and byte-aligned the
    /// bit-reader (ISO/IEC 14496-3 §4.4.2.1).
    End,
}

/// Walker over a `raw_data_block()` payload.
///
/// Drive the walker by calling [`Walker::next_element`] in a loop
/// until it returns either an [`Element::End`] event or `None`
/// (input exhausted before reaching `END`). See the [module
/// docs](self) for the per-element body-skipping rules and what
/// the walker currently does not parse.
pub struct Walker<'a, 'b> {
    reader: &'b mut BitReader<'a>,
    finished: bool,
}

impl<'a, 'b> core::fmt::Debug for Walker<'a, 'b> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Walker")
            .field("finished", &self.finished)
            .field("bit_position", &self.reader.bit_position())
            .finish()
    }
}

impl<'a, 'b> Walker<'a, 'b> {
    /// Bind a walker to an existing [`BitReader`] positioned at the
    /// first byte of a `raw_data_block()` payload.
    pub fn new(reader: &'b mut BitReader<'a>) -> Self {
        Self {
            reader,
            finished: false,
        }
    }

    /// Read the next syntactic element. Returns `Ok(Some(_))` for
    /// every non-terminating element, `Ok(Some(Element::End))` once
    /// (and the walker becomes `finished`), and `Ok(None)` for any
    /// further calls after `End`.
    ///
    /// Errors out with [`Error::UnsupportedElementSkip`] when the
    /// next `id_syn_ele` is one Phase 1 does not parse a body for
    /// (currently: PCE / SCE / CPE / CCE / LFE bodies, though the
    /// channel-element *headers* are recognised and emitted —
    /// `UnsupportedElementSkip` is returned only when a body skip
    /// would be required to continue, which Phase 1 only fires for
    /// PCE).
    pub fn next_element(&mut self) -> Result<Option<Element>> {
        if self.finished {
            return Ok(None);
        }

        let id_bits = self.reader.read_u32(3).map_err(|_| Error::UnexpectedEnd)? as u8;
        let id = IdSynEle::from_bits(id_bits);

        match id {
            IdSynEle::Sce | IdSynEle::Cpe | IdSynEle::Cce | IdSynEle::Lfe => {
                let element_instance_tag =
                    self.reader.read_u32(4).map_err(|_| Error::UnexpectedEnd)? as u8;
                Ok(Some(Element::ChannelElement {
                    kind: id,
                    element_instance_tag,
                }))
            }
            IdSynEle::Fil => {
                let payload_bytes = self.read_fill_count()?;
                self.skip_bytes(payload_bytes)?;
                Ok(Some(Element::Fill { payload_bytes }))
            }
            IdSynEle::Dse => {
                let element_instance_tag =
                    self.reader.read_u32(4).map_err(|_| Error::UnexpectedEnd)? as u8;
                let byte_align_flag = self.reader.read_bit().map_err(|_| Error::UnexpectedEnd)?;
                let payload_bytes = self.read_data_count()?;
                if byte_align_flag {
                    self.reader.align_to_byte();
                }
                self.skip_bytes(payload_bytes)?;
                Ok(Some(Element::Data {
                    element_instance_tag,
                    byte_align_flag,
                    payload_bytes,
                }))
            }
            IdSynEle::Pce => Err(Error::UnsupportedElementSkip(IdSynEle::Pce as u8)),
            IdSynEle::End => {
                self.reader.align_to_byte();
                self.finished = true;
                Ok(Some(Element::End))
            }
        }
    }

    /// `true` once an [`Element::End`] event has been returned.
    pub fn is_finished(&self) -> bool {
        self.finished
    }

    /// Fill-element byte-count read per ISO/IEC 14496-3 §4.4.2.7.
    /// 4-bit `count`; if `count == 15`, an 8-bit `esc_count` follows
    /// and the resulting count is `count + esc_count − 1`.
    fn read_fill_count(&mut self) -> Result<u32> {
        let count = self.reader.read_u32(4).map_err(|_| Error::UnexpectedEnd)?;
        if count == 15 {
            let esc = self.reader.read_u32(8).map_err(|_| Error::UnexpectedEnd)?;
            // §4.4.2.7: `cnt = esc_count + 15 - 1`.
            Ok(esc + 15 - 1)
        } else {
            Ok(count)
        }
    }

    /// Data-stream-element byte-count read per ISO/IEC 14496-3
    /// §4.4.2.5. 8-bit `count`; if `count == 255`, an 8-bit
    /// `esc_count` follows and the resulting count is
    /// `count + esc_count`.
    fn read_data_count(&mut self) -> Result<u32> {
        let count = self.reader.read_u32(8).map_err(|_| Error::UnexpectedEnd)?;
        if count == 255 {
            let esc = self.reader.read_u32(8).map_err(|_| Error::UnexpectedEnd)?;
            Ok(count + esc)
        } else {
            Ok(count)
        }
    }

    /// Skip `n` whole bytes via the bit-reader.
    fn skip_bytes(&mut self, n: u32) -> Result<()> {
        // Multiplication is safe within u32 because §4.4.2.5 caps
        // `count` at 2 × 255 = 510 and §4.4.2.7 caps `cnt` at
        // 15 + 255 − 1 = 269, well below `u32::MAX / 8`.
        let bits = n.saturating_mul(8);
        self.reader.skip(bits).map_err(|_| Error::UnexpectedEnd)
    }
}

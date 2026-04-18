//! Program Config Element parser — ISO/IEC 14496-3 §4.5.2.1 Table 4.2.
//!
//! A PCE describes the channel topology of an AAC stream when
//! `channel_configuration == 0` (or when the PCE appears inline in
//! `raw_data_block()` to override the stream config). It lists, for
//! each channel "slot" (front / side / back / LFE / data / coupling),
//! which SCE or CPE element carries that channel via a 4-bit
//! `*_element_tag_select` index.
//!
//! For the first-pass decoder we parse the PCE but do not yet act on
//! it: the per-element `instance_tag` matching needed to route SCE/CPE
//! output channels to the PCE's described speaker layout is wired
//! separately with multi-channel support.

use oxideav_core::{Error, Result};

use crate::bitreader::BitReader;

/// A single front / side / back channel element listing in a PCE.
#[derive(Clone, Copy, Debug, Default)]
pub struct PceChannelElement {
    /// True if this element is a CPE, false if it's a SCE.
    pub is_cpe: bool,
    /// Value of `element_instance_tag` the decoder should match on the
    /// corresponding SCE/CPE payload.
    pub tag_select: u8,
}

/// Coupling channel listing.
#[derive(Clone, Copy, Debug, Default)]
pub struct PceCcElement {
    pub is_ind_sw: bool,
    pub tag_select: u8,
}

/// Full decoded program_config_element. Fields mirror §4.5.2.1 Table 4.2.
#[derive(Clone, Debug, Default)]
pub struct ProgramConfigElement {
    pub element_instance_tag: u8,
    pub object_type: u8,
    pub sampling_frequency_index: u8,
    pub front_channels: Vec<PceChannelElement>,
    pub side_channels: Vec<PceChannelElement>,
    pub back_channels: Vec<PceChannelElement>,
    /// Each LFE element is identified only by its `tag_select`.
    pub lfe_tags: Vec<u8>,
    pub assoc_data_tags: Vec<u8>,
    pub cc_elements: Vec<PceCcElement>,
    pub mono_mixdown_element_number: Option<u8>,
    pub stereo_mixdown_element_number: Option<u8>,
    pub matrix_mixdown: Option<(u8, bool)>,
    pub comment: Vec<u8>,
}

impl ProgramConfigElement {
    /// Total channel count implied by this PCE's element list (counting a
    /// CPE as 2, an SCE/LFE as 1). Ignores CC/assoc-data elements.
    pub fn channel_count(&self) -> usize {
        let mut n = 0;
        for lists in [&self.front_channels, &self.side_channels, &self.back_channels] {
            for e in lists.iter() {
                n += if e.is_cpe { 2 } else { 1 };
            }
        }
        n + self.lfe_tags.len()
    }
}

/// Parse a PCE element. `br` is positioned immediately after the 3-bit
/// `id_syn_ele == 5` marker (i.e. on the `element_instance_tag` field).
pub fn parse_program_config_element(br: &mut BitReader<'_>) -> Result<ProgramConfigElement> {
    let element_instance_tag = br.read_u32(4)? as u8;
    let object_type = br.read_u32(2)? as u8;
    let sampling_frequency_index = br.read_u32(4)? as u8;
    let num_front = br.read_u32(4)? as usize;
    let num_side = br.read_u32(4)? as usize;
    let num_back = br.read_u32(4)? as usize;
    let num_lfe = br.read_u32(2)? as usize;
    let num_assoc_data = br.read_u32(3)? as usize;
    let num_valid_cc = br.read_u32(4)? as usize;

    let mono_mixdown_element_number = if br.read_bit()? {
        Some(br.read_u32(4)? as u8)
    } else {
        None
    };
    let stereo_mixdown_element_number = if br.read_bit()? {
        Some(br.read_u32(4)? as u8)
    } else {
        None
    };
    let matrix_mixdown = if br.read_bit()? {
        let idx = br.read_u32(2)? as u8;
        let pseudo_surround = br.read_bit()?;
        Some((idx, pseudo_surround))
    } else {
        None
    };

    let front_channels = parse_channel_list(br, num_front)?;
    let side_channels = parse_channel_list(br, num_side)?;
    let back_channels = parse_channel_list(br, num_back)?;

    let mut lfe_tags = Vec::with_capacity(num_lfe);
    for _ in 0..num_lfe {
        lfe_tags.push(br.read_u32(4)? as u8);
    }
    let mut assoc_data_tags = Vec::with_capacity(num_assoc_data);
    for _ in 0..num_assoc_data {
        assoc_data_tags.push(br.read_u32(4)? as u8);
    }
    let mut cc_elements = Vec::with_capacity(num_valid_cc);
    for _ in 0..num_valid_cc {
        let is_ind_sw = br.read_bit()?;
        let tag_select = br.read_u32(4)? as u8;
        cc_elements.push(PceCcElement {
            is_ind_sw,
            tag_select,
        });
    }

    br.align_to_byte();
    let comment_len = br.read_u32(8)? as usize;
    let mut comment = Vec::with_capacity(comment_len);
    for _ in 0..comment_len {
        comment.push(br.read_u32(8)? as u8);
    }

    Ok(ProgramConfigElement {
        element_instance_tag,
        object_type,
        sampling_frequency_index,
        front_channels,
        side_channels,
        back_channels,
        lfe_tags,
        assoc_data_tags,
        cc_elements,
        mono_mixdown_element_number,
        stereo_mixdown_element_number,
        matrix_mixdown,
        comment,
    })
}

fn parse_channel_list(br: &mut BitReader<'_>, n: usize) -> Result<Vec<PceChannelElement>> {
    let mut v = Vec::with_capacity(n);
    for _ in 0..n {
        let is_cpe = br.read_bit()?;
        let tag_select = br.read_u32(4)? as u8;
        if tag_select > 0xF {
            return Err(Error::invalid("AAC PCE: tag_select > 15"));
        }
        v.push(PceChannelElement {
            is_cpe,
            tag_select,
        });
    }
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bitwriter::BitWriter;

    #[test]
    fn round_trip_minimal_pce_stereo() {
        // Build a PCE describing one front CPE (= stereo), no other elements.
        let mut w = BitWriter::new();
        w.write_u32(0, 4); // element_instance_tag
        w.write_u32(1, 2); // object_type = 1 (LC)
        w.write_u32(4, 4); // sf_index = 4 (44.1 kHz)
        w.write_u32(1, 4); // num_front = 1
        w.write_u32(0, 4); // num_side
        w.write_u32(0, 4); // num_back
        w.write_u32(0, 2); // num_lfe
        w.write_u32(0, 3); // num_assoc_data
        w.write_u32(0, 4); // num_valid_cc
        w.write_u32(0, 1); // mono_mixdown_present
        w.write_u32(0, 1); // stereo_mixdown_present
        w.write_u32(0, 1); // matrix_mixdown_present
        // front element 0: is_cpe=1, tag_select=0
        w.write_u32(1, 1);
        w.write_u32(0, 4);
        // no side/back/lfe/assoc/cc
        // byte_alignment
        // comment_len=0
        w.write_u32(0, 8);
        let bytes = w.finish();
        let mut br = BitReader::new(&bytes);
        let pce = parse_program_config_element(&mut br).unwrap();
        assert_eq!(pce.front_channels.len(), 1);
        assert!(pce.front_channels[0].is_cpe);
        assert_eq!(pce.channel_count(), 2);
    }

    #[test]
    fn parse_51_pce_layout() {
        // 5.1: front = SCE (center) + CPE (L/R); side = 0; back = CPE (Ls/Rs);
        // lfe = 1 element.
        let mut w = BitWriter::new();
        w.write_u32(0, 4); // element_instance_tag
        w.write_u32(1, 2); // object_type
        w.write_u32(4, 4); // sf_index
        w.write_u32(2, 4); // num_front = 2 (center + front pair)
        w.write_u32(0, 4); // num_side
        w.write_u32(1, 4); // num_back = 1 (surround pair)
        w.write_u32(1, 2); // num_lfe = 1
        w.write_u32(0, 3);
        w.write_u32(0, 4);
        w.write_u32(0, 1);
        w.write_u32(0, 1);
        w.write_u32(0, 1);
        // front 0: SCE center
        w.write_u32(0, 1);
        w.write_u32(0, 4);
        // front 1: CPE L/R
        w.write_u32(1, 1);
        w.write_u32(0, 4);
        // back 0: CPE Ls/Rs
        w.write_u32(1, 1);
        w.write_u32(1, 4);
        // lfe 0
        w.write_u32(0, 4);
        // comment_len = 0
        w.write_u32(0, 8);
        let bytes = w.finish();
        let mut br = BitReader::new(&bytes);
        let pce = parse_program_config_element(&mut br).unwrap();
        assert_eq!(pce.front_channels.len(), 2);
        assert!(!pce.front_channels[0].is_cpe);
        assert!(pce.front_channels[1].is_cpe);
        assert_eq!(pce.back_channels.len(), 1);
        assert_eq!(pce.lfe_tags.len(), 1);
        assert_eq!(pce.channel_count(), 6); // SCE+CPE+CPE+LFE = 1+2+2+1
    }
}

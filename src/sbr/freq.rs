//! SBR frequency band tables (§4.6.18.3.2).

use oxideav_core::{Error, Result};

/// Result of the frequency-band derivation — owns all tables referenced in
/// the SBR decoding flow.
#[derive(Clone, Debug)]
pub struct FreqTables {
    /// Master frequency band table (§4.6.18.3.2.1).
    pub f_master: Vec<i32>,
    pub n_master: usize,
    /// High-resolution envelope band table.
    pub f_high: Vec<i32>,
    pub n_high: usize,
    /// Low-resolution envelope band table.
    pub f_low: Vec<i32>,
    pub n_low: usize,
    /// Noise-floor band table.
    pub f_noise: Vec<i32>,
    pub nq: usize,
    /// First subband in the SBR range (= `f_high[0]`).
    pub kx: i32,
    /// Number of subbands in the SBR range (= `f_high[n_high] - f_high[0]`).
    pub m: i32,
    /// Last QMF subband index in the fMaster table.
    pub k2: i32,
    /// First QMF subband of the fMaster table (= fMaster[0]).
    pub k0: i32,
}

impl FreqTables {
    /// Build all frequency-band tables from an SBR header and the internal
    /// SBR sample rate (Hz).
    pub fn build(
        fs_sbr: u32,
        bs_start_freq: u8,
        bs_stop_freq: u8,
        bs_xover_band: u8,
        bs_freq_scale: u8,
        bs_alter_scale: bool,
        bs_noise_bands: u8,
    ) -> Result<Self> {
        let k0 = compute_k0(fs_sbr, bs_start_freq)?;
        let k2 = compute_k2(fs_sbr, bs_stop_freq, k0)?;
        if k2 <= k0 {
            return Err(Error::invalid("SBR: k2 <= k0"));
        }
        let f_master = if bs_freq_scale == 0 {
            build_master_linear(k0, k2, bs_alter_scale)?
        } else {
            build_master_bark(k0, k2, bs_freq_scale, bs_alter_scale)?
        };
        let n_master = f_master.len() - 1;
        if (bs_xover_band as usize) >= n_master {
            return Err(Error::invalid("SBR: bs_xover_band >= NMaster"));
        }
        // fTableHigh: subset of fMaster starting at bs_xover_band.
        let n_high = n_master - bs_xover_band as usize;
        let mut f_high = Vec::with_capacity(n_high + 1);
        for k in 0..=n_high {
            f_high.push(f_master[k + bs_xover_band as usize]);
        }
        // NLow = int(NHigh/2) + (NHigh - 2 * int(NHigh/2))
        let n_low = n_high / 2 + (n_high - 2 * (n_high / 2));
        // fTableLow[k] = fTableHigh[ i(k) ] where i(k) = 0 or 2k - (1-(-1)^NHigh)/2
        let one_m_neg = if n_high % 2 == 0 { 0 } else { 1 };
        let mut f_low = Vec::with_capacity(n_low + 1);
        for k in 0..=n_low {
            // Re-evaluate per spec: i(k) = 2k - (1-(-1)^NHigh)/2.
            let _ = one_m_neg;
            let i_k = if k == 0 {
                0
            } else if n_high % 2 == 0 {
                2 * k
            } else {
                2 * k - 1
            };
            f_low.push(f_high[i_k]);
        }
        let kx = f_high[0];
        let m = f_high[n_high] - kx;

        // NQ: max(1, NINT(bs_noise_bands * log2(k2/kx) / log2(2)))
        let nq = {
            let ratio = (k2 as f32 / kx as f32).max(1.0);
            let v = bs_noise_bands as f32 * (ratio.log2()) / 1.0; // log(2)/log(2)=1
            (v.round() as i32).max(1) as usize
        };
        // fTableNoise built from fTableLow: i(k) = i(k-1) + INT((NLow - i(k-1)) / (NQ + 1 - k))
        let mut i_ns: Vec<usize> = vec![0; nq + 1];
        for k in 1..=nq {
            let denom = (nq + 1 - k) as i32;
            let num = n_low as i32 - i_ns[k - 1] as i32;
            let inc = if denom > 0 { num / denom } else { 0 };
            i_ns[k] = i_ns[k - 1] + inc.max(0) as usize;
        }
        let mut f_noise = Vec::with_capacity(nq + 1);
        for k in 0..=nq {
            let i_k = i_ns[k].min(n_low);
            f_noise.push(f_low[i_k]);
        }

        Ok(FreqTables {
            f_master,
            n_master,
            f_high,
            n_high,
            f_low,
            n_low,
            f_noise,
            nq,
            kx,
            m,
            k2,
            k0,
        })
    }
}

fn compute_k0(fs_sbr: u32, bs_start_freq: u8) -> Result<i32> {
    let start_min = if fs_sbr < 32000 {
        nint(3000.0 * 128.0 / fs_sbr as f32)
    } else if fs_sbr < 64000 {
        nint(4000.0 * 128.0 / fs_sbr as f32)
    } else {
        nint(5000.0 * 128.0 / fs_sbr as f32)
    };
    let offset = offset_for_rate(fs_sbr);
    let bs_i = bs_start_freq as usize;
    if bs_i >= offset.len() {
        return Err(Error::invalid("SBR: bs_start_freq out of range"));
    }
    Ok(start_min + offset[bs_i])
}

fn offset_for_rate(fs_sbr: u32) -> &'static [i32] {
    // §4.6.18.3.2.1 — offset table. 16 entries each.
    match fs_sbr {
        16000 => &[-8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7],
        22050 => &[-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13],
        24000 => &[-5, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16],
        32000 => &[-6, -4, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16],
        r if (44100..=64000).contains(&r) => {
            &[-4, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16, 20]
        }
        r if r > 64000 => &[-2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16, 20, 24],
        // Fallback — use 44.1k row.
        _ => &[-4, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 16, 20],
    }
}

fn compute_k2(fs_sbr: u32, bs_stop_freq: u8, k0: i32) -> Result<i32> {
    let stop_min = if fs_sbr < 32000 {
        nint(6000.0 * 128.0 / fs_sbr as f32)
    } else if fs_sbr < 64000 {
        nint(8000.0 * 128.0 / fs_sbr as f32)
    } else {
        nint(10000.0 * 128.0 / fs_sbr as f32)
    };
    let k2 = match bs_stop_freq {
        14 => (2 * k0).min(64),
        15 => (3 * k0).min(64),
        v if v < 14 => {
            // stopDk table sorted ascending, then summed.
            let mut stop_dk: Vec<i32> = (0..=12)
                .map(|p| {
                    let ratio = 64.0f32 / stop_min as f32;
                    let e1 = (p + 1) as f32 / 13.0;
                    let e0 = p as f32 / 13.0;
                    nint(stop_min as f32 * ratio.powf(e1)) - nint(stop_min as f32 * ratio.powf(e0))
                })
                .collect();
            stop_dk.sort_unstable();
            let mut sum = 0i32;
            for i in 0..v as usize {
                sum += stop_dk[i];
            }
            (stop_min + sum).min(64)
        }
        _ => return Err(Error::invalid("SBR: bs_stop_freq > 15")),
    };
    Ok(k2)
}

fn build_master_linear(k0: i32, k2: i32, bs_alter_scale: bool) -> Result<Vec<i32>> {
    let dk = if !bs_alter_scale { 1 } else { 2 };
    let num_bands = if !bs_alter_scale {
        2 * ((k2 - k0) / (dk * 2))
    } else {
        2 * nint_i((k2 - k0) as f32 / (dk as f32 * 2.0))
    };
    if num_bands <= 0 {
        return Err(Error::invalid("SBR: numBands <= 0 in linear fMaster"));
    }
    let k2_achieved = k0 + num_bands * dk;
    let mut k2_diff = k2 - k2_achieved;
    let mut v_dk: Vec<i32> = vec![dk; num_bands as usize];
    if k2_diff != 0 {
        let (incr, mut k) = if k2_diff < 0 {
            (1i32, 0i32)
        } else {
            (-1i32, num_bands - 1)
        };
        while k2_diff != 0 {
            let kk = k as usize;
            v_dk[kk] -= incr;
            k += incr;
            k2_diff += incr;
            if k < 0 || (k as usize) >= v_dk.len() {
                break;
            }
        }
    }
    let mut f_master = Vec::with_capacity(num_bands as usize + 1);
    f_master.push(k0);
    for k in 0..num_bands as usize {
        let prev = f_master[k];
        f_master.push(prev + v_dk[k]);
    }
    Ok(f_master)
}

fn build_master_bark(
    k0: i32,
    k2: i32,
    bs_freq_scale: u8,
    bs_alter_scale: bool,
) -> Result<Vec<i32>> {
    let bands = [12, 10, 8][(bs_freq_scale.clamp(1, 3) - 1) as usize];
    let warp = if bs_alter_scale { 1.3f32 } else { 1.0 };
    let k2_over_k0 = k2 as f32 / k0 as f32;
    let two_regions = k2_over_k0 > 2.2449;
    let k1 = if two_regions { 2 * k0 } else { k2 };
    let num_bands0 = 2 * nint_i(bands as f32 * (k1 as f32 / k0 as f32).ln() / (2.0 * 2.0f32.ln()));
    if num_bands0 <= 0 {
        return Err(Error::invalid("SBR: numBands0 <= 0 in bark fMaster"));
    }
    let mut v_dk0: Vec<i32> = (0..num_bands0 as usize)
        .map(|k| {
            let kp = (k + 1) as f32 / num_bands0 as f32;
            let km = k as f32 / num_bands0 as f32;
            let ratio = k1 as f32 / k0 as f32;
            nint_i(k0 as f32 * ratio.powf(kp)) - nint_i(k0 as f32 * ratio.powf(km))
        })
        .collect();
    v_dk0.sort_unstable();
    let mut vk0: Vec<i32> = vec![0; num_bands0 as usize + 1];
    vk0[0] = k0;
    for k in 1..=num_bands0 as usize {
        vk0[k] = vk0[k - 1] + v_dk0[k - 1];
    }
    if !two_regions {
        return Ok(vk0);
    }
    // Two-region case.
    let num_bands1 =
        2 * nint_i(bands as f32 * (k2 as f32 / k1 as f32).ln() / (2.0 * 2.0f32.ln() * warp));
    if num_bands1 <= 0 {
        return Err(Error::invalid("SBR: numBands1 <= 0"));
    }
    let mut v_dk1: Vec<i32> = (0..num_bands1 as usize)
        .map(|k| {
            let kp = (k + 1) as f32 / num_bands1 as f32;
            let km = k as f32 / num_bands1 as f32;
            let ratio = k2 as f32 / k1 as f32;
            nint_i(k1 as f32 * ratio.powf(kp)) - nint_i(k1 as f32 * ratio.powf(km))
        })
        .collect();
    v_dk1.sort_unstable();
    let max_vdk0 = *v_dk0.iter().max().unwrap_or(&0);
    if v_dk1[0] < max_vdk0 {
        let change = max_vdk0 - v_dk1[0];
        let half_range = (v_dk1[num_bands1 as usize - 1] - v_dk1[0]) / 2;
        let change = change.min(half_range);
        v_dk1[0] += change;
        v_dk1[num_bands1 as usize - 1] -= change;
        v_dk1.sort_unstable();
    }
    let mut vk1: Vec<i32> = vec![0; num_bands1 as usize + 1];
    vk1[0] = k1;
    for k in 1..=num_bands1 as usize {
        vk1[k] = vk1[k - 1] + v_dk1[k - 1];
    }
    // Concatenate vk0 (all) + vk1[1..].
    let mut f = Vec::with_capacity(vk0.len() + vk1.len() - 1);
    f.extend_from_slice(&vk0);
    f.extend_from_slice(&vk1[1..]);
    Ok(f)
}

fn nint(x: f32) -> i32 {
    // Rounding to the nearest integer (banker-like, but spec uses NINT).
    if x >= 0.0 {
        (x + 0.5).floor() as i32
    } else {
        -((-x + 0.5).floor() as i32)
    }
}

fn nint_i(x: f32) -> i32 {
    nint(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn freq_24k_default() {
        // Default-ish HE-AAC config: fs_sbr = 48 kHz, start_freq = 5,
        // stop_freq = 9, xover = 3, freq_scale = 2.
        let t = FreqTables::build(48_000, 5, 9, 3, 2, false, 2).expect("freq tables");
        assert!(t.kx > 0 && t.kx < 32);
        assert!(t.m > 0);
        assert_eq!(t.f_master[0], t.k0);
        // The noise table should have NQ >= 1 and at most 5 entries.
        assert!((1..=5).contains(&t.nq));
        assert_eq!(t.f_noise.len(), t.nq + 1);
    }

    #[test]
    fn freq_master_monotonic() {
        // Build a table known-good for 44.1 kHz HE-AAC (freq_scale=2, default).
        let t = FreqTables::build(44_100, 5, 9, 0, 2, false, 2).expect("freq tables");
        for i in 1..t.f_master.len() {
            assert!(
                t.f_master[i] > t.f_master[i - 1],
                "non-monotonic at {i}: {:?}",
                t.f_master
            );
        }
    }
}

//! The machine the supervisor runs on, and what its speaker card must show
//! before protection is granted (KERNEL.md 2.3 and 2.4).
//!
//! Nothing here is per board. The identity is the device tree's first root
//! compatible ("apple,j316s"); the parameter blob names the board it was
//! compiled for, so a blob from another Mac, even one with the same channel
//! geometry, is refused. The card, the amp count and the amp family come
//! from the self-check's record ([`crate::selfcheck::machine_for`]), which
//! derived them from this Mac's device tree and proved them on its card;
//! the operating point comes from the blob, compiled from this Mac's own
//! tuning, and is checked against the amps' gain readback.

/// The speaker amp family.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Amp {
    /// TI SN012776 (tas2764): V/I sense, "Amp Gain Volume", a DVC and an
    /// "Amp Fault Status" control per amp.
    Sn012776,
    /// Maxim MAX98360A (J700): fixed gain, no I2C, no fault line. The
    /// kernel's software gain is the per-amp "Speaker Playback Volume".
    Max98360a,
}

impl Amp {
    /// Whether every amp must have an "Amp Fault Status" control.
    pub fn reports_faults(self) -> bool {
        matches!(self, Amp::Sn012776)
    }

    /// The per-amp volume control the kernel sets on a grant.
    pub fn volume_suffix(self) -> &'static str {
        match self {
            Amp::Sn012776 => "Speaker Volume",
            Amp::Max98360a => "Speaker Playback Volume",
        }
    }

    /// The volume's raw value at the operating point: DVC 201 (0 dB), or
    /// software gain 127 (Q31 0.5, the MAX98360A's full scale).
    pub fn operating_volume(self) -> i64 {
        match self {
            Amp::Sn012776 => 201,
            Amp::Max98360a => 127,
        }
    }

    /// Whether the full-scale voltage comes from an "Amp Gain Volume"
    /// readback. Without one (the MAX98360A's gain is a strap no software
    /// can read) the operating point is the tuning's own, and only its
    /// channels' agreement is checked (see `supervise::Expect`).
    pub fn has_gain_control(self) -> bool {
        matches!(self, Amp::Sn012776)
    }
}

impl Amp {
    /// The family the self-check records (`family=` in the record: the
    /// device-tree compatible's family, `wiring.py` AMPS).
    pub fn from_family(family: &str) -> Option<Amp> {
        match family {
            "sn012776" => Some(Amp::Sn012776),
            "max98360a" => Some(Amp::Max98360a),
            _ => None,
        }
    }

    /// Whether every amp returns V/I sense (two sense channels each).
    pub fn has_sense(self) -> bool {
        matches!(self, Amp::Sn012776)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Machine {
    /// The board, as the blob names it: the first root compatible without
    /// "apple,".
    pub board: &'static str,
    /// `/proc/asound/<card_id>`: the macaudio card's id.
    pub card_id: &'static str,
    pub amps: usize,
    pub amp: Amp,
}

/// The board from `/proc/device-tree/compatible` (NUL-separated): the first
/// entry, without its "apple," vendor.
pub fn board_from_compatible(raw: &[u8]) -> Option<String> {
    let first = raw.split(|&b| b == 0).next()?;
    let s = core::str::from_utf8(first).ok()?;
    s.strip_prefix("apple,").filter(|b| !b.is_empty()).map(str::to_owned)
}

/// The output peak voltage at digital full scale for an amp gain of
/// `centi_db` hundredths of a dBV.
///
/// The tas2764/tas2770 "Amp Gain Volume" TLV is TI's AMP_LEVEL: the
/// full-scale output in dBV, relative to 1 V rms. The tuning's full scale
/// (spp3 `ag`, lsm2 `VoltsFullScale`) is a peak voltage (Daybreak t-4662,
/// from the binary and the datasheets), so peak = sqrt(2) * 10^(dBV/20):
/// gain 15, 18.5 dBV, is 8.414 V rms = 11.899 V peak, and AID9, AID13 and
/// AID35 all store ag = 11.89.
pub fn full_scale_mv(centi_db: i32) -> f64 {
    1000.0 * core::f64::consts::SQRT_2 * 10f64.powf(f64::from(centi_db) / 2000.0)
}

/// Every channel's full scale (peak mV) as the blob's payload states it:
/// spp3 `ag`, lsm2 `VoltsFullScale`; atsp has none per channel and uses
/// the header's. The header's `full_scale_mv` must be the smallest of them
/// (within the tolerance): a header is one number, the payload is per
/// channel, and the supervisor checks every channel against the amps.
pub fn blob_full_scales(blob: &[u8]) -> Result<Vec<u32>, String> {
    use sgv2_core::UnitKind;
    let (h, _) = sgv2_core::blob::Header::parse(blob).map_err(|e| format!("{e:?}"))?;
    let n = h.channels as usize;
    let mv = |v: f32| -> Result<u32, String> {
        if v.is_finite() && v > 0.0 && v < 1_000.0 {
            Ok((f64::from(v) * 1000.0).round() as u32)
        } else {
            Err(format!("full scale {v} V"))
        }
    };
    let per: Vec<u32> = if h.unit == UnitKind::Spp3 as u32 {
        let p = sgv2_core::spp3::Params::from_blob(blob).map_err(|e| format!("spp3: {e:?}"))?;
        p.ch[..n].iter().map(|c| mv(c.ag)).collect::<Result<_, _>>()?
    } else if h.unit == UnitKind::Lsm2 as u32 {
        let p = sgv2_core::lsm2::Params::from_blob(blob).map_err(|e| format!("lsm2: {e:?}"))?;
        p.ch[..n].iter().map(|c| mv(c.volts_full_scale)).collect::<Result<_, _>>()?
    } else {
        vec![h.full_scale_mv; n]
    };
    let min = per.iter().copied().min().unwrap_or(0);
    if !same_operating_point(f64::from(h.full_scale_mv), min) {
        return Err(format!("header full scale {} mV, the smallest channel's is {min} mV", h.full_scale_mv));
    }
    Ok(per)
}

/// Half a 0.5 dB gain step: a different gain setting never matches.
pub const OPERATING_POINT_TOLERANCE_DB: f64 = 0.25;

/// Whether `measured_mv` is the operating point `expected_mv` names.
pub fn same_operating_point(measured_mv: f64, expected_mv: u32) -> bool {
    expected_mv > 0
        && measured_mv > 0.0
        && (20.0 * (measured_mv / f64::from(expected_mv)).log10()).abs() <= OPERATING_POINT_TOLERANCE_DB
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boards_from_the_device_tree() {
        assert_eq!(board_from_compatible(b"apple,j316s\0apple,t6000\0apple,arm-platform\0").as_deref(), Some("j316s"));
        assert_eq!(board_from_compatible(b"apple,j616s\0apple,t6040\0").as_deref(), Some("j616s"));
        assert_eq!(board_from_compatible(b"vendor,board\0"), None);
        assert_eq!(board_from_compatible(b""), None);
        assert_eq!(Amp::from_family("sn012776"), Some(Amp::Sn012776));
        assert_eq!(Amp::from_family("max98360a"), Some(Amp::Max98360a));
        assert_eq!(Amp::from_family("tas2770"), None);
    }

    #[test]
    fn the_operating_point_tolerates_half_a_step_only() {
        // gain 15 on a tas2764: 18.5 dBV = 8.414 V rms = 11.899 V peak
        let mv = full_scale_mv(1850);
        assert!((mv - 11_899.0).abs() < 1.0, "{mv}");
        // the stored ag of AID9, AID13 and AID35
        assert!(same_operating_point(mv, 11_890));
        // one 0.5 dB step either way is another operating point
        assert!(!same_operating_point(full_scale_mv(1800), 11_890));
        assert!(!same_operating_point(full_scale_mv(1900), 11_890));
        // an rms reading would be 3 dB off
        assert!(!same_operating_point(mv / core::f64::consts::SQRT_2, 11_890));
        assert!(!same_operating_point(mv, 0));
        // the Neo's two channels against its one fixed full scale
        assert!(same_operating_point(7_300.0, 7_330));
    }

    #[test]
    fn only_sn012776_amps_must_report_faults() {
        assert!(Amp::Sn012776.reports_faults());
        assert!(!Amp::Max98360a.reports_faults());
        assert_eq!(Amp::Sn012776.operating_volume(), 201);
        assert_eq!(Amp::Max98360a.operating_volume(), 127);
    }

    #[test]
    fn every_channel_of_the_blob_is_read_and_the_header_is_the_smallest() {
        use sgv2_core::blob::{build_for, machine_field, Pod};
        use sgv2_core::{spp3, UnitKind};
        let m = machine_field("j616s").unwrap();
        let blob = |ags: [f32; 2], header_mv: u32| {
            let mut p = spp3::Params {
                channels: 2,
                sample_rate: 48_000,
                block_frames: 512,
                thermal_ambient_temp: 40.0,
                thermal_gain_attack_time_s: 1.0,
                thermal_gain_release_time_s: 5.0,
                thermal_lookahead_delay_ms: 2.0,
                thermal_protection_enabled: 1,
                ..spp3::Params::default()
            };
            for (c, ag) in ags.iter().enumerate() {
                p.ch[c] = spp3::ChannelParams {
                    source: c as u32,
                    t_sett_vc: 20.0,
                    t_sett_mg: 20.0,
                    tau_tvc: 1.0,
                    tau_tmg: 50.0,
                    temperature_limit: 100.0,
                    hard_temp_limit_headroom: 5.0,
                    reb_ref: 4.0,
                    rshunt: 0.1,
                    ag: *ag,
                    thermal_ff_speedup_factor: 0.5,
                    imon_scale: 1.0,
                    vmon_scale: 1.0,
                    sense_i: (2 * c) as u32,
                    sense_v: (2 * c + 1) as u32,
                    ..Default::default()
                };
            }
            assert!(spp3::Unit::init_error(&p).is_none());
            build_for(UnitKind::Spp3, 2, 4, 48_000, m, header_mv, p.as_bytes())
        };
        assert_eq!(blob_full_scales(&blob([11.89, 11.89], 11_890)), Ok(vec![11_890, 11_890]));
        // a header above the smallest channel is refused
        assert!(blob_full_scales(&blob([11.89, 8.41], 11_890)).is_err());
        // the smallest in the header, the other channel checked on its own
        assert_eq!(blob_full_scales(&blob([11.89, 8.41], 8_410)), Ok(vec![11_890, 8_410]));
        // atsp has no per-channel full scale: the header's, per channel
        let atsp = build_for(UnitKind::Atsp, 2, 0, 48_000, m, 7_300, &[0u8; 4]);
        assert_eq!(blob_full_scales(&atsp), Ok(vec![7_300, 7_300]));
    }
}

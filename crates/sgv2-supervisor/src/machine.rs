//! The machines the supervisor knows, and what each one's speaker card must
//! show before protection is granted (KERNEL.md 2.3 and 2.4).
//!
//! The identity is the device tree's first root compatible ("apple,j316s").
//! The parameter blob names the board it was compiled for, so a blob from
//! another Mac, even one with the same channel geometry, is refused.

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
    /// readback (otherwise it is the chip's fixed [`Machine::fixed_full_scale_mv`]).
    pub fn has_gain_control(self) -> bool {
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
    /// For amps without a gain control: the output peak voltage at digital
    /// full scale, mV.
    pub fixed_full_scale_mv: u32,
}

const fn sn(board: &'static str, card_id: &'static str, amps: usize) -> Machine {
    Machine { board, card_id, amps, amp: Amp::Sn012776, fixed_full_scale_mv: 0 }
}

/// The supported machines (packaging/session/machines.json).
pub const MACHINES: &[Machine] = &[
    sn("j316s", "AppleJ316", 6),
    sn("j316c", "AppleJ316", 6),
    sn("j616s", "AppleJ616", 6),
    sn("j616c", "AppleJ616", 6),
    sn("j413", "AppleJ413", 4),
    // KERNEL.md 1.1: 127 is the amp's full scale, about 7.3 V peak.
    // TODO(SPEC): replace with the measured MAX98360A full scale.
    Machine { board: "j700", card_id: "AppleJ700", amps: 2, amp: Amp::Max98360a, fixed_full_scale_mv: 7_300 },
];

pub fn lookup(board: &str) -> Option<&'static Machine> {
    MACHINES.iter().find(|m| m.board == board)
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
/// The tas2764/tas2770 "Amp Gain Volume" TLV is the datasheet's full-scale
/// output level in dBV (11 to 21 dB). It is read as the peak of a
/// full-scale signal: the top of the range, 22 dBV, is 12.6 V, inside the
/// 16 V PVDD, which an rms reading (17.8 V peak) would not be.
/// TODO(SPEC): confirm against a Mac's own VoltsFullScale.
pub fn full_scale_mv(centi_db: i32) -> f64 {
    1000.0 * 10f64.powf(f64::from(centi_db) / 2000.0)
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
        assert_eq!(lookup("j316s").map(|m| (m.card_id, m.amps)), Some(("AppleJ316", 6)));
        assert_eq!(lookup("j700").map(|m| m.amp), Some(Amp::Max98360a));
        assert!(lookup("j274").is_none());
        // every board fits the blob's machine field
        for m in MACHINES {
            assert!(sgv2_core::blob::machine_field(m.board).is_some(), "{}", m.board);
        }
    }

    #[test]
    fn the_operating_point_tolerates_half_a_step_only() {
        // gain 15 on a tas2764: 18.5 dBV, 8.41 V peak
        let mv = full_scale_mv(1850);
        assert!((mv - 8414.0).abs() < 1.0, "{mv}");
        assert!(same_operating_point(mv, 8414));
        // one 0.5 dB step either way is another operating point
        assert!(!same_operating_point(full_scale_mv(1800), 8414));
        assert!(!same_operating_point(full_scale_mv(1900), 8414));
        assert!(!same_operating_point(mv, 0));
    }

    #[test]
    fn only_sn012776_amps_must_report_faults() {
        assert!(Amp::Sn012776.reports_faults());
        assert!(!Amp::Max98360a.reports_faults());
        assert_eq!(Amp::Sn012776.operating_volume(), 201);
        assert_eq!(Amp::Max98360a.operating_volume(), 127);
    }
}

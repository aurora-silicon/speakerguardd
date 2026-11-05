//! One supervisor tick: observe, decide, act on the kernel. Pure apart from
//! the trait objects, so every fault path is tested against the kernel
//! double.
//!
//! Rules:
//! - renew only on a clean observation (and the lease state machine's two
//!   clean ticks before the first grant);
//! - the kernel-side checks are part of every observation:
//!   - the speaker mux selects the PCM the observer watches;
//!   - on amps with fault reporting, every amp has its fault control (a
//!     missing or short list is a fault, never "no faults");
//!   - the operating point (KERNEL.md 2.3): every amp's analog gain, read
//!     back with its dB scale, is the full-scale voltage the parameter blob
//!     was compiled for, before every grant and on every tick;
//! - any fault releases at once (no waiting for the lease to run out);
//! - a hard amp fault (over-temperature, over-current, group off), or a
//!   granted card whose amps are not at the operating volume, releases and
//!   latches released for the life of the process;
//! - a lapsed lease (`ETIMEDOUT`) grants nothing on this tick: the kernel
//!   has already stopped playback (a supervisor stall), or the system has
//!   just resumed (the first write after a resume). This tick's observation
//!   was clean, so it counts as the first of the two clean ticks, and the
//!   grant comes back on the next clean tick;
//! - losing ownership (`EPERM`) or any other kernel error ends the loop, so
//!   the process exits and the kernel's floor holds (the control file
//!   closes, which withdraws protection at once).

use crate::kernel::{Kernel, KernelError, GAIN_CONTROL_SUFFIX};
use crate::lease::{Action, Lease, Observation};
use crate::machine::{full_scale_mv, same_operating_point, Amp, Machine};

#[derive(Debug, PartialEq, Eq)]
pub enum Event {
    Granted,
    Renewed,
    Released { why: Observation },
    StayedReleased,
    HardFault(String),
    Lapsed,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Fatal {
    Kernel(KernelError),
}

/// What the card must show for this machine and parameter blob.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Expect {
    pub amps: usize,
    pub amp: Amp,
    /// The blob's operating point, per channel: output peak volts at full
    /// scale, mV (see `machine::blob_full_scales`).
    pub full_scale_mv: Vec<u32>,
    /// For amps without a gain control, the operating point the card is
    /// held to: the blob's smallest channel full scale. The amp's gain is a
    /// strap no software can read, so the tuning's own value is the only
    /// source, and the check is that every channel shares it.
    pub fixed_full_scale_mv: u32,
    /// The front-end the speaker mux must select: the watched PCM's id.
    pub route: String,
}

impl Expect {
    pub fn new(machine: &Machine, full_scale_mv: Vec<u32>, route: &str) -> Self {
        let fixed_full_scale_mv =
            if machine.amp.has_gain_control() { 0 } else { full_scale_mv.iter().copied().min().unwrap_or(0) };
        Expect { amps: machine.amps, amp: machine.amp, full_scale_mv, fixed_full_scale_mv, route: route.to_owned() }
    }
}

pub struct Supervisor {
    expect: Expect,
    lease: Lease,
    latched: Option<String>,
    problem: Option<String>,
}

impl Supervisor {
    pub fn new(expect: Expect) -> Self {
        Supervisor { expect, lease: Lease::default(), latched: None, problem: None }
    }

    /// Whether the last tick left protection granted.
    pub fn granted(&self) -> bool {
        self.latched.is_none() && self.lease.held()
    }

    pub fn latched_fault(&self) -> Option<&str> {
        self.latched.as_deref()
    }

    /// Why the kernel-side checks failed on the last tick, if they did.
    pub fn problem(&self) -> Option<&str> {
        self.problem.as_deref()
    }

    /// The kernel-side checks: `Ok(None)` when they all hold.
    fn card_problem(&self, kernel: &mut dyn Kernel, faults: usize) -> Result<Option<String>, KernelError> {
        let x = &self.expect;
        if x.amp.reports_faults() && faults != x.amps {
            return Ok(Some(format!("fault reporting on {faults} of {} amps", x.amps)));
        }
        match kernel.speaker_route()? {
            Some(r) if r == x.route => {}
            r => return Ok(Some(format!("the speaker mux selects {r:?}, not \"{}\"", x.route))),
        }
        let measured = if x.amp.has_gain_control() {
            let gains = kernel.levels(GAIN_CONTROL_SUFFIX)?;
            if gains.len() != x.amps {
                return Ok(Some(format!("amp gain on {} of {} amps", gains.len(), x.amps)));
            }
            let Some(db) = gains[0].centi_db else { return Ok(Some("amp gain has no dB scale".into())) };
            if let Some(g) = gains.iter().find(|g| g.centi_db != Some(db)) {
                return Ok(Some(format!("{} gain {:?} cdB differs from {db} cdB", g.amp, g.centi_db)));
            }
            full_scale_mv(db)
        } else {
            f64::from(x.fixed_full_scale_mv)
        };
        if x.full_scale_mv.len() != x.amps {
            return Ok(Some(format!("{} channel full scales for {} amps", x.full_scale_mv.len(), x.amps)));
        }
        if let Some((c, mv)) = x.full_scale_mv.iter().enumerate().find(|(_, mv)| !same_operating_point(measured, **mv))
        {
            return Ok(Some(format!(
                "operating point {measured:.0} mV full scale, the parameters assume {mv} mV (channel {c})"
            )));
        }
        Ok(None)
    }

    /// While the kernel grants, every amp must sit at the operating volume:
    /// 0 dB as the control's own dB TLV reads it (the tas2764 DVC, the
    /// J700 driver's software gain below the wire's full scale). The raw
    /// value that is 0 dB is the driver's scale, read from the card, not
    /// a number kept here. A volume without a dB scale is not proven to be
    /// at the operating point, and latches like a wrong one.
    fn volume_problem(&self, kernel: &mut dyn Kernel) -> Result<Option<String>, KernelError> {
        let x = &self.expect;
        let v = kernel.levels(x.amp.volume_suffix())?;
        let n = x.amp.volume_controls(x.amps);
        if v.len() != n {
            return Ok(Some(format!("granted, but {} of {n} amp volumes", v.len())));
        }
        Ok(v.iter()
            .find(|l| l.centi_db != Some(0))
            .map(|l| format!("granted, but {} is at {} ({:?} cdB), not 0 dB", l.amp, l.value, l.centi_db)))
    }

    pub fn tick(&mut self, obs: Observation, kernel: &mut dyn Kernel) -> Result<Event, Fatal> {
        if self.latched.is_some() {
            // stay at the kernel's floor; a release is idempotent
            if kernel.granted().map_err(Fatal::Kernel)? {
                kernel.release().map_err(Fatal::Kernel)?;
            }
            return Ok(Event::StayedReleased);
        }
        let faults = kernel.faults().map_err(Fatal::Kernel)?;
        if let Some(f) = faults.iter().find(|f| f.hard()) {
            return self.latch(kernel, format!("{} (bits {:#04x}, group off {})", f.amp, f.bits, f.group_off));
        }
        // a lease that lapsed shows the floor: only a live grant is checked
        if self.lease.held() && kernel.granted().map_err(Fatal::Kernel)? {
            if let Some(p) = self.volume_problem(kernel).map_err(Fatal::Kernel)? {
                return self.latch(kernel, p);
            }
        }
        self.problem = self.card_problem(kernel, faults.len()).map_err(Fatal::Kernel)?;
        let ok = self.problem.is_none();
        let obs = Observation { route_ok: ok, amps_ok: ok, ..obs };
        let was_held = self.lease.held();
        match self.lease.tick(obs) {
            Action::Renew => match kernel.renew() {
                Ok(()) => Ok(if was_held { Event::Renewed } else { Event::Granted }),
                Err(KernelError::Lapsed) => {
                    // released, with this clean tick as the first of two
                    self.lease.tick(Observation::default());
                    self.lease.tick(obs);
                    Ok(Event::Lapsed)
                }
                Err(KernelError::Sleeping) => {
                    self.lease.tick(Observation::default());
                    Ok(Event::StayedReleased)
                }
                Err(e) => Err(Fatal::Kernel(e)),
            },
            Action::Release => {
                kernel.release().map_err(Fatal::Kernel)?;
                Ok(Event::Released { why: obs })
            }
            Action::StayReleased => Ok(Event::StayedReleased),
        }
    }

    fn latch(&mut self, kernel: &mut dyn Kernel, why: String) -> Result<Event, Fatal> {
        self.latched = Some(why.clone());
        self.lease.tick(Observation::default());
        kernel.release().map_err(Fatal::Kernel)?;
        Ok(Event::HardFault(why))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::{AmpFault, AmpLevel, Recorder};

    /// Synthetic machines: six SN012776 amps, and a two-amp MAX98360A pair.
    fn sn6() -> &'static Machine {
        &Machine { board: "t6s", card_id: "AppleT6", amps: 6, amp: Amp::Sn012776 }
    }
    fn fixed2() -> &'static Machine {
        &Machine { board: "t2", card_id: "AppleT2", amps: 2, amp: Amp::Max98360a }
    }

    /// Clean as far as the observer can tell: it never vouches for the
    /// kernel-side checks.
    const CLEAN: Observation = Observation {
        heartbeat_fresh: true,
        plugin_ok: true,
        links_ok: true,
        carrier_ok: true,
        pcm_owner_ok: true,
        pcm_state_ok: true,
        route_ok: false,
        amps_ok: false,
    };

    /// The J316 with a blob for gain 15 (18.5 dBV, 11.89 V peak).
    fn j316() -> Supervisor {
        Supervisor::new(Expect::new(sn6(), vec![11_890; 6], "Secondary"))
    }

    #[test]
    fn grants_after_two_clean_ticks_and_releases_at_once_on_a_fault() {
        let mut k = Recorder::healthy(6);
        let mut s = j316();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert!(!k.held);
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        assert!(k.held);
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Renewed));
        let bad = Observation { heartbeat_fresh: false, ..CLEAN };
        assert!(matches!(s.tick(bad, &mut k), Ok(Event::Released { .. })));
        assert!(!k.held);
        assert_eq!(k.releases, 1);
    }

    #[test]
    fn the_observer_cannot_vouch_for_the_kernel_side_checks() {
        let mut k = Recorder::healthy(6);
        k.route = None;
        let mut s = j316();
        let all = Observation { route_ok: true, amps_ok: true, ..CLEAN };
        for _ in 0..5 {
            assert_eq!(s.tick(all, &mut k), Ok(Event::StayedReleased));
        }
        assert!(s.problem().unwrap().contains("mux"));
    }

    #[test]
    fn a_hard_amp_fault_latches_released() {
        let mut k = Recorder::healthy(6);
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        s.tick(CLEAN, &mut k).unwrap();
        assert!(k.held);
        k.amp_faults[2].bits = AmpFault::OVER_TEMPERATURE;
        assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::HardFault(_))));
        assert!(!k.held);
        k.amp_faults[2].bits = 0;
        for _ in 0..10 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
            assert!(!k.held);
        }
        assert!(s.latched_fault().is_some());
    }

    #[test]
    fn limiter_bits_are_not_hard_faults() {
        let mut k = Recorder::healthy(6);
        k.amp_faults[0].bits = AmpFault::LIMITER_ACTIVE | AmpFault::BOP_MUTE;
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
    }

    #[test]
    fn missing_fault_controls_are_a_fault_where_the_amps_report_faults() {
        for n in [0, 5] {
            let mut k = Recorder::healthy(6);
            k.amp_faults.truncate(n);
            let mut s = j316();
            for _ in 0..5 {
                assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
                assert!(!k.held);
            }
            assert_eq!(s.problem(), Some(format!("fault reporting on {n} of 6 amps").as_str()));
        }
        // losing them while granted releases
        let mut k = Recorder::healthy(6);
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        k.amp_faults.clear();
        assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::Released { .. })));
        assert!(!k.held);
    }

    #[test]
    fn the_neo_has_no_fault_controls_and_a_fixed_full_scale() {
        let neo = fixed2();
        let neo_card = || {
            let mut k = Recorder::healthy(2);
            k.amp_faults.clear();
            k.gain = None;
            // one card-level software gain for the pair, no amp prefix
            k.amps = vec![String::new()];
            // the driver's TLV: -63.5 dB + 0.5 dB a step, 0 dB at the top
            k.volume = Some(("Speaker Playback Volume".into(), (127, Some(0)), (87, Some(-2000))));
            k
        };
        let mut k = neo_card();
        let mut s = Supervisor::new(Expect::new(neo, vec![7_300, 7_330], "Secondary"));
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Renewed));
        // the shared gain away from the operating point while granted latches
        k.volume = Some(("Speaker Playback Volume".into(), (100, Some(-1350)), (87, Some(-2000))));
        assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::HardFault(_))));
        // a blob whose channels are not one operating point is refused
        let mut k = neo_card();
        let mut s = Supervisor::new(Expect::new(neo, vec![7_300, 6_000], "Secondary"));
        for _ in 0..4 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        // granted, the volume must read 0 dB on its own scale: a control
        // without a dB TLV, or a top that is not 0 dB, latches
        for held in [(127, None), (127, Some(-50)), (126, Some(-50))] {
            let mut k = neo_card();
            k.volume = Some(("Speaker Playback Volume".into(), held, (87, Some(-2000))));
            let mut s = Supervisor::new(Expect::new(neo, vec![7_300, 7_330], "Secondary"));
            s.tick(CLEAN, &mut k).unwrap();
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
            assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::HardFault(_))), "{held:?}");
            assert!(!k.held);
        }
    }

    #[test]
    fn the_speaker_mux_must_select_the_watched_pcm() {
        let mut k = Recorder::healthy(6);
        k.route = Some("Primary".into());
        let mut s = j316();
        for _ in 0..4 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        k.route = Some("Secondary".into());
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        // switched under a running grant: released at once
        k.route = Some("Primary".into());
        assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::Released { .. })));
        assert!(!k.held);
    }

    struct OneOff(Recorder);
    impl Kernel for OneOff {
        fn renew(&mut self) -> Result<(), KernelError> {
            self.0.renew()
        }
        fn release(&mut self) -> Result<(), KernelError> {
            self.0.release()
        }
        fn granted(&mut self) -> Result<bool, KernelError> {
            self.0.granted()
        }
        fn faults(&mut self) -> Result<Vec<AmpFault>, KernelError> {
            self.0.faults()
        }
        fn levels(&mut self, suffix: &str) -> Result<Vec<AmpLevel>, KernelError> {
            let mut v = self.0.levels(suffix)?;
            if suffix == GAIN_CONTROL_SUFFIX {
                v[3] = AmpLevel { amp: "Amp 3".into(), value: 14, centi_db: Some(1800) };
            }
            Ok(v)
        }
        fn speaker_route(&mut self) -> Result<Option<String>, KernelError> {
            self.0.speaker_route()
        }
    }

    #[test]
    fn a_wrong_amp_gain_never_grants() {
        // gain 16 (19 dBV) against a blob for 18.5 dBV
        let mut k = Recorder::healthy(6);
        k.gain = Some((16, 1900));
        let mut s = j316();
        for _ in 0..5 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        assert!(s.problem().unwrap().contains("operating point"));
        assert_eq!(k.renewals, 0);
        // a blob that states no operating point is refused too
        let mut k = Recorder::healthy(6);
        let mut s = Supervisor::new(Expect::new(sn6(), vec![0; 6], "Secondary"));
        for _ in 0..3 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        // and one amp that differs from the rest
        let mut k = OneOff(Recorder::healthy(6));
        let mut s = j316();
        for _ in 0..3 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        assert!(s.problem().unwrap().contains("Amp 3"));
    }

    #[test]
    fn a_grant_without_the_operating_volume_latches() {
        let mut k = Recorder::healthy(6);
        k.volume = Some(("Speaker Volume".into(), (180, Some(-1050)), (0, None)));
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::HardFault(_))));
        assert!(!k.held);
        for _ in 0..5 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
    }

    struct Lapsing(Recorder, bool);
    impl Kernel for Lapsing {
        fn renew(&mut self) -> Result<(), KernelError> {
            if self.1 {
                self.1 = false;
                self.0.held = false;
                return Err(KernelError::Lapsed);
            }
            self.0.renew()
        }
        fn release(&mut self) -> Result<(), KernelError> {
            self.0.release()
        }
        fn granted(&mut self) -> Result<bool, KernelError> {
            self.0.granted()
        }
        fn faults(&mut self) -> Result<Vec<AmpFault>, KernelError> {
            self.0.faults()
        }
        fn levels(&mut self, suffix: &str) -> Result<Vec<AmpLevel>, KernelError> {
            self.0.levels(suffix)
        }
        fn speaker_route(&mut self) -> Result<Option<String>, KernelError> {
            self.0.speaker_route()
        }
    }

    #[test]
    fn a_lapse_or_a_resume_grants_again_on_the_next_clean_tick() {
        let mut k = Lapsing(Recorder::healthy(6), false);
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        // sgv2k2: the first write after a resume (or a lapse) is ETIMEDOUT
        k.1 = true;
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Lapsed));
        assert!(!s.granted());
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        // a fault on the next tick still keeps it released
        k.1 = true;
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Lapsed));
        let bad = Observation { heartbeat_fresh: false, ..CLEAN };
        assert_eq!(s.tick(bad, &mut k), Ok(Event::StayedReleased));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
    }

    #[test]
    fn a_lapsed_floor_is_not_read_as_a_wrong_operating_volume() {
        // the supervisor stalled: the kernel let the lease run out and
        // floored the amps; that is a lapse, not a latched fault
        let mut k = Lapsing(Recorder::healthy(6), false);
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        k.0.held = false;
        k.1 = true;
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Lapsed));
        assert!(s.latched_fault().is_none());
    }

    struct Lost;
    impl Kernel for Lost {
        fn renew(&mut self) -> Result<(), KernelError> {
            Err(KernelError::NotOwner)
        }
        fn release(&mut self) -> Result<(), KernelError> {
            Ok(())
        }
        fn granted(&mut self) -> Result<bool, KernelError> {
            Ok(false)
        }
        fn faults(&mut self) -> Result<Vec<AmpFault>, KernelError> {
            Ok(Recorder::healthy(6).amp_faults)
        }
        fn levels(&mut self, suffix: &str) -> Result<Vec<AmpLevel>, KernelError> {
            Recorder::healthy(6).levels(suffix)
        }
        fn speaker_route(&mut self) -> Result<Option<String>, KernelError> {
            Ok(Some("Secondary".into()))
        }
    }

    #[test]
    fn losing_ownership_is_fatal() {
        let mut s = j316();
        s.tick(CLEAN, &mut Lost).unwrap();
        assert_eq!(s.tick(CLEAN, &mut Lost), Err(Fatal::Kernel(KernelError::NotOwner)));
    }
}

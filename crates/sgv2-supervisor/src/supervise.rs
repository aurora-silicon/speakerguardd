//! One supervisor tick: observe, decide, act on the kernel. Pure apart from
//! the trait objects, so every fault path is tested against the kernel
//! double.
//!
//! Rules:
//! - renew only on a clean observation (and the lease state machine's two
//!   clean ticks before the first grant);
//! - the kernel-side checks are part of every observation:
//!   - the speaker mux selects the PCM the observer watches; a first grant
//!     locks the mux, reads it again and grants only the route it read
//!     under that lock, so no other file can re-route between the check
//!     and the grant (under a live grant the kernel refuses re-routes);
//!   - on amps with fault reporting, every amp has its fault control (a
//!     missing or short list is a fault, never "no faults");
//!   - the operating point (KERNEL.md 2.3): every amp's analog gain, read
//!     back with its dB scale, is the full-scale voltage the parameter blob
//!     was compiled for, within half that control's own TLV step (at most
//!     0.25 dB), before every grant and on every tick;
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
use crate::machine::{
    full_scale_mv, gain_tolerance_db, same_operating_point_within, Amp, Machine, OPERATING_POINT_TOLERANCE_DB,
};

#[derive(Debug, PartialEq, Eq)]
pub enum Event {
    Granted,
    Renewed,
    Released {
        why: Observation,
    },
    /// Nothing plays: the grant was given back with every check holding.
    Idle,
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
    /// gate.json's cap on this board's operating volume, cdB on the volume
    /// control's own TLV (`None`: none below 0 dB), from the self-check's
    /// record (`volume_cap_cdb`). It only ever lowers what the machine
    /// reports: the operating volume is the ceiling the card reports under
    /// the grant (see `Supervisor::volume_problem`), and a ceiling above the
    /// cap latches.
    pub volume_cap_cdb: Option<i32>,
}

impl Expect {
    pub fn new(machine: &Machine, full_scale_mv: Vec<u32>, route: &str) -> Self {
        let fixed_full_scale_mv =
            if machine.amp.has_gain_control() { 0 } else { full_scale_mv.iter().copied().min().unwrap_or(0) };
        Expect {
            amps: machine.amps,
            amp: machine.amp,
            full_scale_mv,
            fixed_full_scale_mv,
            route: route.to_owned(),
            volume_cap_cdb: None,
        }
    }

    /// The same, with gate.json's cap on the operating volume (the
    /// self-check record's `volume_cap_cdb`).
    pub fn with_volume_cap_cdb(self, volume_cap_cdb: Option<i32>) -> Self {
        Expect { volume_cap_cdb, ..self }
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
        let (measured, tolerance) = if x.amp.has_gain_control() {
            let gains = kernel.levels(GAIN_CONTROL_SUFFIX)?;
            if gains.len() != x.amps {
                return Ok(Some(format!("amp gain on {} of {} amps", gains.len(), x.amps)));
            }
            let Some(db) = gains[0].centi_db else { return Ok(Some("amp gain has no dB scale".into())) };
            if let Some(g) = gains.iter().find(|g| g.centi_db != Some(db)) {
                return Ok(Some(format!("{} gain {:?} cdB differs from {db} cdB", g.amp, g.centi_db)));
            }
            // the match is held to half the gain control's own step, read
            // from its TLV: a card without a fixed step is not granted
            let Some(step) = gains[0].step_centi_db else {
                return Ok(Some("amp gain has no fixed dB step".into()));
            };
            if let Some(g) = gains.iter().find(|g| g.step_centi_db != Some(step)) {
                return Ok(Some(format!("{} gain step {:?} cdB differs from {step} cdB", g.amp, g.step_centi_db)));
            }
            let Some(tolerance) = gain_tolerance_db(step) else {
                return Ok(Some(format!("amp gain step {step} cdB")));
            };
            (full_scale_mv(db), tolerance)
        } else {
            (f64::from(x.fixed_full_scale_mv), OPERATING_POINT_TOLERANCE_DB)
        };
        if x.full_scale_mv.len() != x.amps {
            return Ok(Some(format!("{} channel full scales for {} amps", x.full_scale_mv.len(), x.amps)));
        }
        if let Some((c, mv)) =
            x.full_scale_mv.iter().enumerate().find(|(_, mv)| !same_operating_point_within(measured, **mv, tolerance))
        {
            return Ok(Some(format!(
                "operating point {measured:.0} mV full scale, the parameters assume {mv} mV (channel {c})"
            )));
        }
        Ok(None)
    }

    /// While the kernel grants, every amp must sit at the operating volume,
    /// as the control's own dB TLV reads it (the tas2764 DVC, the J700
    /// driver's software gain below the wire's full scale). The raw value
    /// is the driver's scale, read from the card, not a number kept here.
    /// A volume without a dB scale is not proven to be at the operating
    /// point, and latches like a wrong one.
    ///
    /// The operating volume is the machine's: the ceiling the card reports
    /// under the grant, the dB at every amp volume's maximum (its TLV at
    /// the limit macaudio sets): 0 dB on every board but one whose speaker
    /// only a lease may lift (macaudio's spk_fixed_ceiling, the J873g's
    /// -16 dB). The amps must report one ceiling, from -40 to 0 dB and no
    /// higher than gate.json's cap (`Expect::volume_cap_cdb`, which can
    /// only lower it), and sit exactly at it. A card that reports no
    /// ceiling latches, whatever gate.json says; so does one that put the
    /// amps louder, or anywhere else. The operating point is still the amp
    /// gain's full scale: the model is computed for that drive, which is
    /// the ceiling's dB more than the speaker gets.
    fn volume_problem(&self, kernel: &mut dyn Kernel) -> Result<Option<String>, KernelError> {
        let x = &self.expect;
        let v = kernel.levels(x.amp.volume_suffix())?;
        let n = x.amp.volume_controls(x.amps);
        if v.len() != n {
            return Ok(Some(format!("granted, but {} of {n} amp volumes", v.len())));
        }
        let db = |c: i32| f64::from(c) / 100.0;
        let Some(ceiling) = v.first().and_then(|l| l.max_centi_db) else {
            return Ok(Some("granted, but the amp volume reports no ceiling (no dB scale at its maximum)".into()));
        };
        if let Some(l) = v.iter().find(|l| l.max_centi_db != Some(ceiling)) {
            return Ok(Some(format!(
                "granted, but {} reports a ceiling of {:?} cdB, not the {} dB of {}",
                l.amp,
                l.max_centi_db,
                db(ceiling),
                v[0].amp
            )));
        }
        if !(-4000..=0).contains(&ceiling) {
            return Ok(Some(format!(
                "granted, but the amp volumes' ceiling is {} dB, not from -40 to 0 dB",
                db(ceiling)
            )));
        }
        if let Some(cap) = x.volume_cap_cdb.filter(|&cap| ceiling > cap) {
            return Ok(Some(format!(
                "granted, but the amp volumes' ceiling is {} dB, above gate.json's {} dB for this board",
                db(ceiling),
                db(cap)
            )));
        }
        Ok(v.iter().find(|l| l.centi_db != Some(ceiling)).map(|l| {
            format!(
                "granted, but {} is at {} ({:?} cdB), not at its ceiling of {} dB",
                l.amp,
                l.value,
                l.centi_db,
                db(ceiling)
            )
        }))
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
                // The lease can end between the two reads: a supervisor
                // late for its tick under host load (J413, load 55) read
                // the grant live, the kernel closed the gate and floored the
                // amps, and the floor read as a wrong operating volume,
                // latched until restart. The kernel closes the gate before
                // it floors, so a grant still live after the volume read
                // means the volume was wrong under it. Otherwise this is a
                // lapse: the renew below reports it.
                if kernel.granted().map_err(Fatal::Kernel)? {
                    return self.latch(kernel, p);
                }
            }
        }
        self.problem = self.card_problem(kernel, faults.len()).map_err(Fatal::Kernel)?;
        let ok = self.problem.is_none();
        let obs = Observation { route_ok: ok, amps_ok: ok, ..obs };
        let was_held = self.lease.held();
        match self.lease.tick(obs) {
            Action::Renew if was_held => {
                let renewed = kernel.renew();
                self.renewed(renewed, obs, Event::Renewed)
            }
            Action::Renew => self.first_grant(kernel, obs),
            Action::Release => {
                kernel.release().map_err(Fatal::Kernel)?;
                Ok(Event::Released { why: obs })
            }
            Action::Idle => {
                kernel.release().map_err(Fatal::Kernel)?;
                Ok(Event::Idle)
            }
            Action::StayReleased => Ok(Event::StayedReleased),
        }
    }

    /// A grant on a released lease. The route was checked earlier in this
    /// tick, but until a grant the kernel lets any file write the mux: lock
    /// it, read it again, and grant only if it still selects the watched
    /// PCM. No other file can re-route between that read and the grant, and
    /// from the grant on the kernel refuses every re-route itself. A mux
    /// that another file has locked cannot be held, so nothing is granted.
    fn first_grant(&mut self, kernel: &mut dyn Kernel, obs: Observation) -> Result<Event, Fatal> {
        if !kernel.hold_route().map_err(Fatal::Kernel)? {
            return Ok(self.refuse("the speaker mux is locked by another file".into()));
        }
        let event = match kernel.speaker_route() {
            Ok(Some(r)) if r == self.expect.route => {
                let renewed = kernel.renew();
                self.renewed(renewed, obs, Event::Granted)
            }
            Ok(r) => Ok(self.refuse(format!("the speaker mux selects {r:?}, not \"{}\"", self.expect.route))),
            Err(e) => Err(Fatal::Kernel(e)),
        };
        let freed = kernel.free_route();
        let event = event?;
        freed.map_err(Fatal::Kernel)?;
        Ok(event)
    }

    fn renewed(&mut self, renewed: Result<(), KernelError>, obs: Observation, ok: Event) -> Result<Event, Fatal> {
        match renewed {
            Ok(()) => Ok(ok),
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
        }
    }

    /// Grant nothing on this tick: the lease takes two clean ticks again.
    fn refuse(&mut self, why: String) -> Event {
        self.problem = Some(why);
        self.lease.tick(Observation::default());
        Event::StayedReleased
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
        demand: true,
    };

    /// The J316 with a blob for gain 15 (18.5 dBV, 11.89 V peak).
    fn j316() -> Supervisor {
        Supervisor::new(Expect::new(sn6(), vec![11_890; 6], "Secondary"))
    }

    /// A four-amp TAS5770L Mac (the J293 shape, atsp): gain 15 read back
    /// from "Amp Gain Volume", the DVC "Speaker Playback Volume" at 201
    /// (0 dB) under a grant, and no fault-status controls to require.
    #[test]
    fn a_tas2770_mac_grants_without_fault_controls_and_holds_its_volume() {
        let m = &Machine { board: "t4", card_id: "AppleT4", amps: 4, amp: Amp::Tas2770 };
        let mut k = Recorder::healthy(4);
        k.amp_faults.clear();
        k.volume = Some(("Speaker Playback Volume".into(), (201, Some(0)), (161, Some(-2000))));
        let mut s = Supervisor::new(Expect::new(m, vec![11_890; 4], "Secondary"));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Renewed));
        // a granted amp off the operating volume releases to the floor
        k.volume = Some(("Speaker Playback Volume".into(), (200, Some(-50)), (161, Some(-2000))));
        k.ceiling = Some(Some(0));
        assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::Released { .. }) | Ok(Event::HardFault(_))));
        assert!(!k.held);
        // a blob for another operating point never grants
        let mut k = Recorder::healthy(4);
        k.amp_faults.clear();
        k.volume = Some(("Speaker Playback Volume".into(), (201, Some(0)), (161, Some(-2000))));
        let mut s = Supervisor::new(Expect::new(m, vec![8_913; 4], "Secondary"));
        for _ in 0..4 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        assert!(s.problem().unwrap().contains("operating point"), "{:?}", s.problem());
    }

    /// The J316s row with no stream: an engine that beats while nothing is
    /// granted reads fresh and stale on alternate idle ticks. With a steady
    /// 0.5 dB gain TLV the kernel-side checks hold on every one of them (no
    /// problem, so no "not granting" and no new ready spell), nothing is
    /// granted or released, and "protection ready" is said exactly once.
    #[test]
    fn a_steady_card_with_a_flickering_idle_engine_is_ready_once() {
        use crate::pace::{ReadyNote, Spell};
        let mut k = Recorder::healthy(6);
        let mut s = j316();
        let mut note = ReadyNote::default();
        let mut last_problem: Option<String> = None;
        let mut said = 0;
        for i in 0..420 {
            let obs = Observation { demand: false, heartbeat_fresh: i % 2 == 0, ..CLEAN };
            let event = s.tick(obs, &mut k);
            assert!(matches!(event, Ok(Event::StayedReleased)), "tick {i}: {event:?}");
            assert_eq!(s.problem(), None, "tick {i}");
            let ready = s.problem().is_none() && Observation { route_ok: true, amps_ok: true, ..obs }.clean();
            let spell = if s.problem() != last_problem.as_deref() { Spell::New } else { Spell::Same };
            last_problem = s.problem().map(str::to_owned);
            said += usize::from(note.tick(ready, spell));
        }
        assert_eq!(said, 1);
        assert_eq!((k.renewals, k.releases), (0, 0));
    }

    #[test]
    fn nothing_playing_gives_the_grant_back_and_a_stream_takes_it_at_once() {
        let mut k = Recorder::healthy(6);
        let mut s = j316();
        let idle = Observation { demand: false, ..CLEAN };
        // idle from the start: the kernel floor holds, nothing is granted
        assert_eq!(s.tick(idle, &mut k), Ok(Event::StayedReleased));
        assert_eq!(s.tick(idle, &mut k), Ok(Event::StayedReleased));
        assert!(!k.held);
        assert_eq!(k.renewals, 0);
        // a stream: granted on its first tick, the checks having held
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        assert!(k.held);
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Renewed));
        // it stops: released to the floor, and not as a fault
        let releases = k.releases;
        assert_eq!(s.tick(idle, &mut k), Ok(Event::Idle));
        assert!(!k.held);
        assert_eq!(k.releases, releases + 1);
        assert_eq!(s.tick(idle, &mut k), Ok(Event::StayedReleased));
        // the kernel-side checks hold while idle too: a card off the
        // operating point blocks the next grant
        k.gain = Some((12, 1250));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert!(s.problem().unwrap().contains("operating point"), "{:?}", s.problem());
        // and a hard amp fault while idle still latches
        k.gain = Some((15, 1850));
        k.amp_faults[0].bits = AmpFault::OVER_TEMPERATURE;
        assert!(matches!(s.tick(idle, &mut k), Ok(Event::HardFault(_))));
        k.amp_faults[0].bits = 0;
        for _ in 0..3 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        assert!(!k.held);
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
        k.ceiling = Some(Some(0));
        assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::HardFault(_))));
        // a blob whose channels are not one operating point is refused
        let mut k = neo_card();
        let mut s = Supervisor::new(Expect::new(neo, vec![7_300, 6_000], "Secondary"));
        for _ in 0..4 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        // granted, the volume must read its ceiling on its own scale: a
        // control without a dB TLV, or a level below a 0 dB top, latches
        for (held, top) in [((127, None), None), ((126, Some(-50)), Some(0)), ((127, Some(0)), None)] {
            let mut k = neo_card();
            k.volume = Some(("Speaker Playback Volume".into(), held, (87, Some(-2000))));
            k.ceiling = Some(top);
            let mut s = Supervisor::new(Expect::new(neo, vec![7_300, 7_330], "Secondary"));
            s.tick(CLEAN, &mut k).unwrap();
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
            assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::HardFault(_))), "{held:?}");
            assert!(!k.held);
        }
    }

    /// The J873g's shape: one SN012776 whose kernel lifts it under the
    /// lease only to its fixed ceiling, -16 dB on the DVC (raw 169, the
    /// control's limit under the grant), and a blob for amp gain 15
    /// (11.89 V peak). The ceiling comes from the card; nothing names the
    /// board.
    #[test]
    fn a_fixed_ceiling_board_grants_only_at_the_ceiling_the_card_reports() {
        let m = &Machine { board: "j873g", card_id: "AppleJ873", amps: 1, amp: Amp::Sn012776 };
        let card = |held: crate::kernel::Level, ceiling: Option<Option<i32>>| {
            let mut k = Recorder::healthy(1);
            k.volume = Some(("Speaker Volume".into(), held, (1, Some(-10000))));
            k.ceiling = ceiling;
            k
        };
        let at = |cap: Option<i32>| Expect::new(m, vec![11_890], "Secondary").with_volume_cap_cdb(cap);
        // the reported ceiling, no gate.json entry, or one at or above it: grants
        for cap in [None, Some(-1600), Some(0)] {
            let mut k = card((169, Some(-1600)), None);
            let mut s = Supervisor::new(at(cap));
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased), "{cap:?}");
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted), "{cap:?}");
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Renewed), "{cap:?}");
        }
        let latches = |mut k: Recorder, cap: Option<i32>, want: &str| {
            let mut s = Supervisor::new(at(cap));
            s.tick(CLEAN, &mut k).unwrap();
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted), "{want}");
            match s.tick(CLEAN, &mut k) {
                Ok(Event::HardFault(why)) => assert!(why.contains(want), "{want}: {why}"),
                e => panic!("{want}: {e:?}"),
            }
            assert!(!k.held, "{want}");
        };
        // held off the ceiling: louder, just above or below it, or without a dB scale
        for held in [(201, Some(0)), (170, Some(-1550)), (168, Some(-1650)), (169, None)] {
            latches(card(held, Some(Some(-1600))), None, "not at its ceiling of -16 dB");
        }
        // no ceiling from the card: latched, with or without a gate.json entry
        for cap in [None, Some(-1600)] {
            latches(card((169, Some(-1600)), Some(None)), cap, "reports no ceiling");
        }
        // a ceiling above gate.json's cap: latched, the cap only lowers it
        latches(card((169, Some(-1600)), None), Some(-2000), "above gate.json's -20 dB");
        // a ceiling above 0 dB, or below -40 dB
        latches(card((213, Some(600)), None), None, "not from -40 to 0 dB");
        latches(card((100, Some(-5050)), None), None, "not from -40 to 0 dB");
        // amps that report different ceilings
        let mut k = card((169, Some(-1600)), None);
        k.amps.push("Amp 1".into());
        k.amp_faults.push(AmpFault { amp: "Amp 1".into(), bits: 0, events: 0, group_off: false });
        struct Split(Recorder);
        impl Kernel for Split {
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
                if suffix != GAIN_CONTROL_SUFFIX && self.0.held {
                    v[1].max_centi_db = Some(0);
                }
                Ok(v)
            }
            fn speaker_route(&mut self) -> Result<Option<String>, KernelError> {
                self.0.speaker_route()
            }
        }
        let two = &Machine { board: "j873g", card_id: "AppleJ873", amps: 2, amp: Amp::Sn012776 };
        let mut k = Split(k);
        let mut s = Supervisor::new(Expect::new(two, vec![11_890, 11_890], "Secondary"));
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        match s.tick(CLEAN, &mut k) {
            Ok(Event::HardFault(why)) => assert!(why.contains("reports a ceiling of Some(0) cdB"), "{why}"),
            e => panic!("{e:?}"),
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

    #[test]
    fn a_reroute_between_the_route_check_and_the_grant_grants_nothing() {
        let mut k = Recorder::healthy(6);
        let mut s = j316();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        // another file switches the mux right after this tick's route check
        k.reroute = Some((1, "Primary".into()));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert_eq!(k.route.as_deref(), Some("Primary"));
        assert_eq!(k.renewals, 0);
        assert!(s.problem().unwrap().contains("Primary"), "{:?}", s.problem());
        assert!(!k.route_held);
        for _ in 0..3 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        // switched back: granted on the second clean tick, on that route
        k.route = Some("Secondary".into());
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        assert_eq!(k.renewed_routes, vec![Some("Secondary".to_owned())]);
        assert!(!k.route_held);
    }

    #[test]
    fn the_mux_is_held_from_the_last_route_read_to_the_grant() {
        let mut k = Recorder::healthy(6);
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        // the re-route lands while the grant holds the mux: refused
        k.reroute = Some((2, "Primary".into()));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        assert_eq!(k.reroutes_refused, 1);
        assert_eq!(k.route.as_deref(), Some("Secondary"));
        assert_eq!((k.route_holds, k.route_held), (1, false));
        // renewals under the live grant take no lock: the kernel holds the route
        for _ in 0..3 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Renewed));
        }
        assert_eq!(k.route_holds, 1);
        assert!(k.renewed_routes.iter().all(|r| r.as_deref() == Some("Secondary")));
    }

    #[test]
    fn a_mux_locked_by_another_file_grants_nothing() {
        let mut k = Recorder::healthy(6);
        k.route_locked_elsewhere = true;
        let mut s = j316();
        for _ in 0..4 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        assert_eq!(k.renewals, 0);
        assert!(s.problem().unwrap().contains("locked by another file"), "{:?}", s.problem());
        k.route_locked_elsewhere = false;
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
    }

    #[test]
    fn a_grant_the_kernel_refuses_after_a_reroute_waits_for_two_clean_ticks() {
        // EBUSY on a first grant: a sleep, or the kernel saw another file
        // write the mux since its last grant decision
        let mut k = Recorder::healthy(6);
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        k.refuse_next = Some(KernelError::Sleeping);
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert!(!s.granted());
        assert!(!k.route_held);
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
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
                v[3] = AmpLevel {
                    amp: "Amp 3".into(),
                    value: 14,
                    centi_db: Some(1800),
                    step_centi_db: Some(50),
                    max_centi_db: Some(1800),
                };
            }
            Ok(v)
        }
        fn speaker_route(&mut self) -> Result<Option<String>, KernelError> {
            self.0.speaker_route()
        }
        fn hold_route(&mut self) -> Result<bool, KernelError> {
            self.0.hold_route()
        }
        fn free_route(&mut self) -> Result<(), KernelError> {
            self.0.free_route()
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

    /// The operating-point match is held to half the gain control's own
    /// TLV step. At 0.5 dB (every tas2764/tas2770 card today) it grants and
    /// refuses exactly as the fixed 0.25 dB did; a gain without a readable
    /// fixed step, or amps that disagree on it, never grant.
    #[test]
    fn the_gain_match_uses_the_controls_own_step() {
        // 0.5 dB a step: the J316s/J413/J616s shape grants as before
        let mut k = Recorder::healthy(6);
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        // a 0.25 dB-step card one setting below the blob's gain (18.25
        // dBV, 0.243 dB under 11.89 V): inside the old fixed 0.25 dB, but
        // it is another gain setting, so it is refused now
        let mut k = Recorder::healthy(6);
        k.step_centi_db = Some(25);
        k.gain = Some((29, 1825));
        let mut s = j316();
        for _ in 0..4 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        assert!(s.problem().unwrap().contains("operating point"), "{:?}", s.problem());
        assert_eq!(k.renewals, 0);
        // the same card at the blob's own gain grants
        k.gain = Some((30, 1850));
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        // no fixed step (no DB_SCALE TLV): refused before any grant
        let mut k = Recorder::healthy(6);
        k.step_centi_db = None;
        let mut s = j316();
        for _ in 0..4 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        assert!(s.problem().unwrap().contains("no fixed dB step"), "{:?}", s.problem());
        assert_eq!(k.renewals, 0);
        // a grant held while the step stops reading releases at once
        let mut k = Recorder::healthy(6);
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        k.step_centi_db = None;
        assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::Released { .. })));
        assert!(!k.held);
        // one amp on another step
        struct OtherStep(Recorder);
        impl Kernel for OtherStep {
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
                    v[2].step_centi_db = Some(100);
                }
                Ok(v)
            }
            fn speaker_route(&mut self) -> Result<Option<String>, KernelError> {
                self.0.speaker_route()
            }
            fn hold_route(&mut self) -> Result<bool, KernelError> {
                self.0.hold_route()
            }
            fn free_route(&mut self) -> Result<(), KernelError> {
                self.0.free_route()
            }
        }
        let mut k = OtherStep(Recorder::healthy(6));
        let mut s = j316();
        for _ in 0..4 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        }
        assert!(s.problem().unwrap().contains("Amp 2 gain step"), "{:?}", s.problem());
        // the J700 (no gain control) is unaffected: its channels are
        // checked against the blob's own full scale as before
        let mut k = Recorder::healthy(2);
        k.amp_faults.clear();
        k.gain = None;
        k.step_centi_db = None;
        k.amps = vec![String::new()];
        k.volume = Some(("Speaker Playback Volume".into(), (127, Some(0)), (87, Some(-2000))));
        let mut s = Supervisor::new(Expect::new(fixed2(), vec![7_300, 7_330], "Secondary"));
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
    }

    #[test]
    fn a_grant_without_the_operating_volume_latches() {
        // the kernel lifted the limit to 0 dB, the amps stayed at -10.5 dB
        let mut k = Recorder::healthy(6);
        k.volume = Some(("Speaker Volume".into(), (180, Some(-1050)), (0, None)));
        k.ceiling = Some(Some(0));
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
        fn hold_route(&mut self) -> Result<bool, KernelError> {
            self.0.hold_route()
        }
        fn free_route(&mut self) -> Result<(), KernelError> {
            self.0.free_route()
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

    /// A lease that ends between the supervisor's grant read and its volume
    /// read: the first `granted()` is live, the floor lands before the
    /// volume read, and the renew finds the lease lapsed.
    struct EndsMidTick(Recorder, u32);
    impl Kernel for EndsMidTick {
        fn renew(&mut self) -> Result<(), KernelError> {
            if self.1 == 1 {
                self.1 = 2;
                return Err(KernelError::Lapsed);
            }
            self.0.renew()
        }
        fn release(&mut self) -> Result<(), KernelError> {
            self.0.release()
        }
        fn granted(&mut self) -> Result<bool, KernelError> {
            let g = self.0.granted();
            if self.1 == 0 && self.0.held {
                // this read sees the grant; the lease ends right after it
                self.1 = 1;
                self.0.held = false;
            }
            g
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
        fn hold_route(&mut self) -> Result<bool, KernelError> {
            self.0.hold_route()
        }
        fn free_route(&mut self) -> Result<(), KernelError> {
            self.0.free_route()
        }
    }

    #[test]
    fn a_lease_ending_between_the_grant_and_volume_reads_is_a_lapse_not_a_fault() {
        let mut k = EndsMidTick(Recorder::healthy(6), 9);
        let mut s = j316();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        k.1 = 0;
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Lapsed));
        assert!(s.latched_fault().is_none());
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        // a volume that is wrong while the grant stays live still latches
        k.0.volume = Some(("Speaker Volume".into(), (180, Some(-1050)), (0, None)));
        k.0.ceiling = Some(Some(0));
        assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::HardFault(_))));
        assert!(!k.0.held);
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
        fn hold_route(&mut self) -> Result<bool, KernelError> {
            Ok(true)
        }
        fn free_route(&mut self) -> Result<(), KernelError> {
            Ok(())
        }
    }

    #[test]
    fn losing_ownership_is_fatal() {
        let mut s = j316();
        s.tick(CLEAN, &mut Lost).unwrap();
        assert_eq!(s.tick(CLEAN, &mut Lost), Err(Fatal::Kernel(KernelError::NotOwner)));
    }
}

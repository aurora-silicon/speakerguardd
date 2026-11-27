// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! A lapse after a late tick against a lapse after a sleep, through the
//! observer, the supervisor and the lapse witness together, as the loop
//! drives them.
//!
//! The J713 shape (11-28, 4136af40): at a stream's end the session helper
//! closes the amps' and the sense PCMs under the grant; one tick ran late,
//! the 30 ms lease ran out with no sleep anywhere, and the loop took the
//! lapse for a sleep. The next stream's grant then waited for the
//! post-sleep settle (~530 ms after its demand) and its first frames
//! overflowed the plugin's 500 ms wake. Timings here are synthetic.

use crate::kernel::{AmpFault, AmpLevel, Kernel, KernelError, Recorder};
use crate::lease::{LapseCause, LapseWitness};
use crate::machine::{Amp, Machine};
use crate::observe::{
    EngineState, EngineView, Observer, PcmState, PcmView, SessionReport, SessionView, RESUME_SETTLE_NS,
};
use crate::supervise::{Event, Expect, Supervisor};

const MS: u64 = 1_000_000;
const TICK: u64 = 10 * MS;
const PID: u32 = 4242;
const CRC: u32 = 7;
/// 512 frames at 48 kHz.
const BEAT: u64 = 512 * 1_000_000_000 / 48_000;

const SN6: Machine = Machine { board: "t6s", card_id: "AppleT6", amps: 6, amp: Amp::Sn012776 };

/// A card whose next renewal lapses when `lapse_next` is set, as the
/// kernel's does once the lease has run out.
struct Card {
    k: Recorder,
    lapse_next: bool,
}
impl Kernel for Card {
    fn renew(&mut self) -> Result<(), KernelError> {
        if std::mem::take(&mut self.lapse_next) {
            self.k.held = false;
            return Err(KernelError::Lapsed);
        }
        self.k.renew()
    }
    fn release(&mut self) -> Result<(), KernelError> {
        self.k.release()
    }
    fn granted(&mut self) -> Result<bool, KernelError> {
        self.k.granted()
    }
    fn faults(&mut self) -> Result<Vec<AmpFault>, KernelError> {
        self.k.faults()
    }
    fn levels(&mut self, suffix: &str) -> Result<Vec<AmpLevel>, KernelError> {
        self.k.levels(suffix)
    }
    fn speaker_route(&mut self) -> Result<Option<String>, KernelError> {
        self.k.speaker_route()
    }
}

struct E(EngineState);
impl EngineView for E {
    fn read(&mut self) -> EngineState {
        self.0
    }
}
struct P(PcmState);
impl PcmView for P {
    fn read(&mut self) -> PcmState {
        self.0
    }
}
struct S(SessionReport);
impl SessionView for S {
    fn read(&mut self) -> SessionReport {
        self.0
    }
}

/// The loop's state, and one tick of it as `main` runs it.
struct Loop {
    o: Observer,
    s: Supervisor,
    k: Card,
    w: LapseWitness,
    now: u64,
}

impl Loop {
    fn new() -> Self {
        Loop {
            o: Observer::new(CRC, 48_000, TICK),
            s: Supervisor::new(Expect::new(&SN6, vec![11_890; 6], "Secondary")),
            k: Card { k: Recorder::healthy(6), lapse_next: false },
            w: LapseWitness::default(),
            now: 0,
        }
    }

    /// One tick at `self.now`, the engine beating steadily and a stream
    /// playing; `slept`: the suspend clock or counters moved since the last.
    fn tick(&mut self, slept: bool) -> (Event, Option<LapseCause>) {
        if slept {
            self.o.note_sleep();
            self.w.sleep();
        }
        let now = self.now;
        let seq = now / BEAT;
        let e = EngineState {
            valid: true,
            pid: PID,
            params_crc: CRC,
            seq,
            frames: seq * 512,
            mono_ns: seq * BEAT,
            health: 0,
            min_gain_bits: 1.0f32.to_bits(),
        };
        let r = SessionReport {
            present: true,
            links_ok: true,
            carrier_ok: true,
            mono_ns: now.saturating_sub(5 * MS),
            helper: Some((77, 1234)),
            helper_alive: true,
            wiring: None,
            demand: true,
        };
        let obs = self.o.observe(&mut || now, &mut E(e), &mut P(PcmState::Running { owner: PID }), &mut S(r));
        let event = self.s.tick(obs, &mut self.k).expect("no fatal kernel error");
        let mut cause = None;
        match event {
            Event::Lapsed => cause = Some(self.w.answer(now, &mut self.o)),
            Event::Granted | Event::Renewed => self.w.renewed(now),
            _ if !self.s.granted() => self.w.released(),
            _ => {}
        }
        (event, cause)
    }

    /// Ticks every 10 ms until a grant; how long it took.
    fn until_granted(&mut self) -> u64 {
        let t0 = self.now;
        loop {
            self.now += TICK;
            if self.tick(false).0 == Event::Granted {
                return self.now - t0;
            }
            assert!(self.now - t0 < 5_000 * MS, "never granted again");
        }
    }

    /// Granted and steady for a second.
    fn steady() -> Self {
        let mut l = Loop::new();
        l.now = 1_000 * MS;
        l.until_granted();
        for _ in 0..100 {
            l.now += TICK;
            assert_eq!(l.tick(false).0, Event::Renewed);
        }
        l
    }
}

#[test]
fn a_late_tick_lapse_regrants_on_the_next_clean_tick_without_the_sleep_settle() {
    let mut l = Loop::steady();
    // the tick runs 29 ms late (the renewal 39 ms after the last): the
    // kernel's lease ended, no sleep happened
    l.now += 39 * MS;
    l.k.lapse_next = true;
    let (event, cause) = l.tick(false);
    assert_eq!(event, Event::Lapsed);
    assert_eq!(cause, Some(LapseCause::Stall { since_renew_ns: 39 * MS }));
    assert!(!l.s.granted());
    assert!(!l.o.waiting_after_sleep(), "a late tick was taken for a sleep");
    // the lapsed tick was clean, so the next clean tick grants
    assert_eq!(l.until_granted(), TICK);
}

#[test]
fn a_lapse_after_a_sleep_still_waits_for_the_engine_to_settle() {
    // a sleep the loop saw: the pre-sleep beat is held stale, so that tick
    // releases; the kernel's first write after the resume lapses, on the
    // grant, and the grant after it still waits for the engine to settle
    let mut l = Loop::steady();
    l.now += 39 * MS;
    l.k.lapse_next = true;
    assert!(matches!(l.tick(true).0, Event::Released { .. }));
    let t0 = l.now;
    let mut lapsed = None;
    while lapsed.is_none() {
        l.now += TICK;
        if let (Event::Lapsed, c) = l.tick(false) {
            lapsed = c;
        }
        assert!(l.now - t0 < 5_000 * MS, "never tried to grant");
    }
    assert_eq!(lapsed, Some(LapseCause::Sleep));
    assert!(l.o.waiting_after_sleep());
    assert!(l.until_granted() >= RESUME_SETTLE_NS, "granted before the post-sleep settle");

    // the first write after a resume, on a grant: the kernel refused the
    // renewals during the sleep (here: the lease was given back), so the
    // lapse comes with nothing held and counts as a sleep
    let mut l = Loop::steady();
    l.k.lapse_next = true;
    l.s.tick(crate::lease::Observation::default(), &mut l.k).unwrap();
    l.w.released();
    assert!(!l.s.granted());
    let mut lapsed = None;
    for _ in 0..3 {
        l.now += TICK;
        if let (Event::Lapsed, c) = l.tick(false) {
            lapsed = c;
            break;
        }
    }
    assert_eq!(lapsed, Some(LapseCause::Sleep));
    assert!(l.until_granted() >= RESUME_SETTLE_NS);

    // a supervisor stopped for longer than a late tick (frozen, a sleep
    // the counters missed): taken as a sleep
    let mut l = Loop::steady();
    l.now += crate::lease::LAPSE_STALL_MAX_NS;
    l.k.lapse_next = true;
    assert_eq!(l.tick(false), (Event::Lapsed, Some(LapseCause::Sleep)));
    assert!(l.o.waiting_after_sleep());
}

#[test]
fn a_late_tick_lapse_with_a_fault_stays_released() {
    // fail-closed is unchanged: the re-grant still needs two clean ticks
    let mut l = Loop::steady();
    l.now += 39 * MS;
    l.k.lapse_next = true;
    assert_eq!(l.tick(false).0, Event::Lapsed);
    // the engine stops beating: its last beat goes stale, nothing grants
    let stopped_at = l.now;
    for _ in 0..30 {
        l.now += TICK;
        let seq = stopped_at / BEAT;
        let e = EngineState {
            valid: true,
            pid: PID,
            params_crc: CRC,
            seq,
            frames: seq * 512,
            mono_ns: seq * BEAT,
            health: 0,
            min_gain_bits: 1.0f32.to_bits(),
        };
        let r = SessionReport {
            present: true,
            links_ok: true,
            carrier_ok: true,
            mono_ns: l.now - 5 * MS,
            helper: Some((77, 1234)),
            helper_alive: true,
            wiring: None,
            demand: true,
        };
        let now = l.now;
        let obs = l.o.observe(&mut || now, &mut E(e), &mut P(PcmState::Running { owner: PID }), &mut S(r));
        let event = l.s.tick(obs, &mut l.k).unwrap();
        if l.now - stopped_at > 40 * MS {
            assert_eq!(event, Event::StayedReleased, "granted on a stale heartbeat");
        }
    }
    assert!(!l.k.k.held);
}

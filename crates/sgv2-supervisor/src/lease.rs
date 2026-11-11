//! The path-integrity state machine, as pure logic.
//!
//! Every tick (10 ms) the supervisor gathers an [`Observation`]. The lease is
//! renewed only while every check holds and a client stream plays; any
//! failure releases it at once, and it is taken back only after two
//! consecutive clean ticks. With nothing playing (no demand) a clean tick
//! releases too, as idle rather than a fault: the kernel holds the amps at
//! its floor and refuses every start, as macOS powers the speaker path down
//! when nothing plays. The checks keep running while idle, so the grant
//! comes back on the first tick with demand once two clean ticks have
//! passed.

/// What one tick observed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Observation {
    /// The plugin's heartbeat advanced within its allowed staleness (or the
    /// stream is idle and closed).
    pub heartbeat_fresh: bool,
    /// The plugin reports healthy and runs the published parameters.
    pub plugin_ok: bool,
    /// The graph's links are exactly the configured set.
    pub links_ok: bool,
    /// The sense carrier stream is at unity and unmuted.
    pub carrier_ok: bool,
    /// The speaker PCM is owned by the process hosting the plugin.
    pub pcm_owner_ok: bool,
    /// The speaker PCM is Closed, Open (setup, prepared) or Running, or in
    /// an XRUN the sink is recovering from (no longer than the heartbeat
    /// bound). Stopped (paused, suspended), a longer XRUN or unknown is not.
    pub pcm_state_ok: bool,
    /// The speaker mux selects the watched PCM (from the kernel).
    pub route_ok: bool,
    /// Every expected amp reports faults, and the amps sit at the operating
    /// point the parameters assume (from the kernel).
    pub amps_ok: bool,
    /// A client stream plays into the protect stage (the session helper's
    /// fresh report). Not a check: without it a clean tick is idle.
    pub demand: bool,
}

impl Observation {
    pub fn clean(&self) -> bool {
        self.heartbeat_fresh
            && self.plugin_ok
            && self.links_ok
            && self.carrier_ok
            && self.pcm_owner_ok
            && self.pcm_state_ok
            && self.route_ok
            && self.amps_ok
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    Renew,
    /// A check failed while the lease was held.
    Release,
    /// Every check holds, but nothing plays: the lease was held and is
    /// given back.
    Idle,
    StayReleased,
}

/// Clean ticks needed before the lease is taken back.
pub const CLEAN_TICKS_TO_LOCK: u32 = 2;

#[derive(Debug, Default)]
pub struct Lease {
    held: bool,
    clean_run: u32,
}

impl Lease {
    pub fn held(&self) -> bool {
        self.held
    }

    pub fn tick(&mut self, obs: Observation) -> Action {
        if !obs.clean() {
            self.clean_run = 0;
            let was = self.held;
            self.held = false;
            return if was { Action::Release } else { Action::StayReleased };
        }
        self.clean_run = self.clean_run.saturating_add(1);
        if !obs.demand {
            let was = self.held;
            self.held = false;
            return if was { Action::Idle } else { Action::StayReleased };
        }
        if self.held || self.clean_run >= CLEAN_TICKS_TO_LOCK {
            self.held = true;
            Action::Renew
        } else {
            Action::StayReleased
        }
    }
}

/// A grant that a fault ends sooner than this after it was given is short:
/// the stream it was for never got going (its start failed, the engine
/// stalled behind it). One short grant is re-granted at once, as before; a
/// run of them is spaced out.
pub const SHORT_GRANT_NS: u64 = 2_000_000_000;
/// The hold before the re-grant after the second short grant in a row,
/// doubled for each one after it up to [`MAX_REGRANT_HOLD_NS`].
pub const FIRST_REGRANT_HOLD_NS: u64 = 250_000_000;
pub const MAX_REGRANT_HOLD_NS: u64 = 4_000_000_000;

/// Spaces re-grants after grants that keep ending in a fault at once.
///
/// When a stream cannot start under a grant (a sink left unable to start,
/// an engine that stalls on every start), each grant is followed by a
/// release a few hundred ms later and the next grant two clean ticks
/// after that. Nothing plays in that loop (the kernel refuses every start
/// outside a grant), but the audio server retries its start every cycle
/// and logs every failure (J413, M1 Pro J316: ~15 lines a second until
/// PipeWire restarted). A hold only ever delays a grant: every check keeps
/// running, and a held-off tick is treated as a tick without demand.
#[derive(Debug, Default)]
pub struct Regrant {
    granted_at: Option<u64>,
    short_run: u32,
    hold_until: u64,
}

impl Regrant {
    /// The supervisor granted at `now_ns`.
    pub fn granted(&mut self, now_ns: u64) {
        self.granted_at = Some(now_ns);
    }

    /// The grant ended at `now_ns`: on a fault (`fault`), or otherwise
    /// (nothing plays, a lapse, a sleep). Returns the hold before the next
    /// grant, if one starts.
    pub fn released(&mut self, now_ns: u64, fault: bool) -> Option<u64> {
        let t = self.granted_at.take()?;
        if !fault || now_ns.saturating_sub(t) >= SHORT_GRANT_NS {
            self.short_run = 0;
            self.hold_until = 0;
            return None;
        }
        self.short_run = self.short_run.saturating_add(1);
        if self.short_run < 2 {
            return None;
        }
        let hold = FIRST_REGRANT_HOLD_NS.saturating_mul(1 << (self.short_run - 2).min(16)).min(MAX_REGRANT_HOLD_NS);
        self.hold_until = now_ns.saturating_add(hold);
        Some(hold)
    }

    /// Nothing plays: the next stream starts afresh.
    pub fn idle(&mut self) {
        self.short_run = 0;
        self.hold_until = 0;
    }

    /// Whether a grant is held off at `now_ns`.
    pub fn holding(&self, now_ns: u64) -> bool {
        now_ns < self.hold_until
    }

    /// Short grants in a row so far.
    pub fn short_run(&self) -> u32 {
        self.short_run
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CLEAN: Observation = Observation {
        heartbeat_fresh: true,
        plugin_ok: true,
        links_ok: true,
        carrier_ok: true,
        pcm_owner_ok: true,
        pcm_state_ok: true,
        route_ok: true,
        amps_ok: true,
        demand: true,
    };

    #[test]
    fn locks_after_two_clean_ticks_and_releases_on_any_fault() {
        let mut l = Lease::default();
        assert_eq!(l.tick(CLEAN), Action::StayReleased);
        assert_eq!(l.tick(CLEAN), Action::Renew);
        assert_eq!(l.tick(CLEAN), Action::Renew);
        for fault in [
            Observation { heartbeat_fresh: false, ..CLEAN },
            Observation { plugin_ok: false, ..CLEAN },
            Observation { links_ok: false, ..CLEAN },
            Observation { carrier_ok: false, ..CLEAN },
            Observation { pcm_owner_ok: false, ..CLEAN },
            Observation { pcm_state_ok: false, ..CLEAN },
            Observation { route_ok: false, ..CLEAN },
            Observation { amps_ok: false, ..CLEAN },
        ] {
            assert_eq!(l.tick(fault), Action::Release);
            assert!(!l.held());
            assert_eq!(l.tick(fault), Action::StayReleased);
            assert_eq!(l.tick(CLEAN), Action::StayReleased);
            assert_eq!(l.tick(CLEAN), Action::Renew);
        }
    }

    #[test]
    fn idle_releases_without_a_fault_and_demand_grants_at_once() {
        let idle = Observation { demand: false, ..CLEAN };
        let mut l = Lease::default();
        // idle from the start: never granted, but the clean ticks count
        assert_eq!(l.tick(idle), Action::StayReleased);
        assert_eq!(l.tick(idle), Action::StayReleased);
        assert!(!l.held());
        // a stream starts after two clean idle ticks: granted on this tick
        assert_eq!(l.tick(CLEAN), Action::Renew);
        assert!(l.held());
        // it stops: given back, once
        assert_eq!(l.tick(idle), Action::Idle);
        assert!(!l.held());
        assert_eq!(l.tick(idle), Action::StayReleased);
        assert_eq!(l.tick(CLEAN), Action::Renew);
    }

    #[test]
    fn idle_ticks_never_hide_a_fault() {
        let idle = Observation { demand: false, ..CLEAN };
        let mut l = Lease::default();
        assert_eq!(l.tick(idle), Action::StayReleased);
        assert_eq!(l.tick(idle), Action::StayReleased);
        // a fault while idle resets the clean run: demand right after it
        // waits for two clean ticks again
        assert_eq!(l.tick(Observation { heartbeat_fresh: false, demand: false, ..CLEAN }), Action::StayReleased);
        assert_eq!(l.tick(CLEAN), Action::StayReleased);
        assert_eq!(l.tick(CLEAN), Action::Renew);
        // a fault with demand while held is a fault release, not idle
        assert_eq!(l.tick(Observation { links_ok: false, demand: false, ..CLEAN }), Action::Release);
        // demand never makes an unclean tick grant
        let mut l = Lease::default();
        for _ in 0..4 {
            assert_eq!(l.tick(Observation { carrier_ok: false, ..CLEAN }), Action::StayReleased);
        }
        assert!(!l.held());
    }

    #[test]
    fn one_clean_tick_between_faults_is_not_enough() {
        let mut l = Lease::default();
        let bad = Observation::default();
        assert_eq!(l.tick(CLEAN), Action::StayReleased);
        assert_eq!(l.tick(bad), Action::StayReleased);
        assert_eq!(l.tick(CLEAN), Action::StayReleased);
        assert!(!l.held());
    }

    const MS: u64 = 1_000_000;

    #[test]
    fn one_short_grant_regrants_at_once_and_a_run_of_them_is_spaced_out() {
        let mut r = Regrant::default();
        let mut now = 1_000 * MS;
        // the first short grant: no hold, as before
        r.granted(now);
        now += 400 * MS;
        assert_eq!(r.released(now, true), None);
        assert!(!r.holding(now));
        // then 250 ms, 500 ms, 1 s, 2 s, 4 s, and 4 s from then on
        for want in [250, 500, 1_000, 2_000, 4_000, 4_000, 4_000] {
            now += 20 * MS;
            r.granted(now);
            now += 400 * MS;
            assert_eq!(r.released(now, true), Some(want * MS));
            assert!(r.holding(now + want * MS - 1));
            assert!(!r.holding(now + want * MS));
            now += want * MS;
        }
    }

    #[test]
    fn a_grant_that_held_or_ended_without_a_fault_resets_the_run() {
        for end in
            [|r: &mut Regrant, t| r.released(t + SHORT_GRANT_NS, true), |r: &mut Regrant, t| r.released(t + MS, false)]
        {
            let mut r = Regrant::default();
            let mut now = 0;
            for _ in 0..4 {
                r.granted(now);
                now += 100 * MS;
                r.released(now, true);
            }
            assert!(r.holding(now));
            now += MAX_REGRANT_HOLD_NS;
            r.granted(now);
            assert_eq!(end(&mut r, now), None);
            assert_eq!(r.short_run(), 0);
            r.granted(now + 3 * SHORT_GRANT_NS);
            assert_eq!(r.released(now + 3 * SHORT_GRANT_NS + MS, true), None, "the first short grant again");
        }
        // idle clears a hold at once; a release with no grant changes nothing
        let mut r = Regrant::default();
        for t in 0..3 {
            r.granted(t * MS);
            r.released(t * MS + MS, true);
        }
        assert!(r.holding(3 * MS));
        r.idle();
        assert!(!r.holding(3 * MS));
        assert_eq!(r.released(10 * MS, true), None);
    }
}

//! How fast the supervisor looks while nothing plays.
//!
//! With a stream playing, the loop ticks every 10 ms and its two pollers
//! (the speaker PCM's `/proc` status and the session helper's liveness)
//! every millisecond, so a fault releases the grant within a poll. With
//! nothing playing there is no grant to release: the kernel holds the amps
//! at its floor and refuses every start. Polling at full rate then only
//! costs power (about 2 x 1000 wakeups a second from the pollers alone),
//! while macOS stops its speaker engine shortly after the last client.
//!
//! So once the path has been quiet (no grant, no demand in the helper's
//! report) for [`IDLE_AFTER_NS`], the supervisor goes idle: the pollers run
//! every [`IDLE_POLL_NS`] and the loop every [`IDLE_TICK_NS`], or at once
//! when the helper renames a new report into place or the PCM reader sees
//! a change, and the loop drops its frequency floor. Every check keeps running at that pace (hard amp faults still
//! latch, the kernel-side checks still gate the next grant).
//!
//! Leaving idle is the first thing a demand does. The tick that sees a
//! demand while idle does not act on it: it wakes the pollers to full rate
//! and schedules the next tick [`WAKE_TICK_NS`] later, after their fresh
//! readings. A grant therefore always comes from a tick whose PCM snapshot
//! and helper verdict were taken at full rate, and the lease's rule is
//! unchanged: two clean ticks in a row, each clean only if every reading
//! in it was fresh.

/// Quiet this long (no grant, no demand) before going idle.
pub const IDLE_AFTER_NS: u64 = 500_000_000;
/// The loop's tick while idle. A new report wakes it at once; where the
/// report's directory cannot be watched the loop keeps its 10 ms tick.
pub const IDLE_TICK_NS: u64 = 250_000_000;
/// The pollers' period while idle. Their snapshots are then older than the
/// RT loop accepts as proof, which is harmless with nothing to grant.
pub const IDLE_POLL_NS: u64 = 100_000_000;
/// After leaving idle, the next tick comes this soon: long enough for the
/// woken pollers to take fresh readings.
pub const WAKE_TICK_NS: u64 = 2_000_000;

/// What changed after a tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Change {
    None,
    /// The path has been quiet long enough: slow down.
    Idle,
    /// A demand or a grant: full rate from now on.
    Active,
}

#[derive(Debug, Default)]
pub struct Pace {
    idle: bool,
    quiet_since: Option<u64>,
}

impl Pace {
    /// Whether the supervisor is idle (slow pollers, slow loop).
    pub fn idle(&self) -> bool {
        self.idle
    }

    /// The demand a tick may act on: none while idle, so a grant never
    /// rests on a reading taken at the idle pace.
    pub fn demand(&self, demand: bool) -> bool {
        demand && !self.idle
    }

    /// After each tick: `busy` is a grant held or a demand seen on this
    /// tick (before [`Pace::demand`] gated it).
    pub fn after_tick(&mut self, now_ns: u64, busy: bool) -> Change {
        if busy {
            self.quiet_since = None;
            if self.idle {
                self.idle = false;
                return Change::Active;
            }
            return Change::None;
        }
        let since = *self.quiet_since.get_or_insert(now_ns);
        if !self.idle && now_ns.saturating_sub(since) >= IDLE_AFTER_NS {
            self.idle = true;
            return Change::Idle;
        }
        Change::None
    }
}

/// When to say "protection ready" (every check holds, nothing plays yet).
///
/// Said once per spell. A spell starts at boot and at every grant, release,
/// demand, fault or new kernel-side problem; it does not start again when a
/// tick's readings merely went stale and fresh again. While idle the pollers
/// run every [`IDLE_POLL_NS`] and the loop also ticks when they wake it, so
/// an engine that keeps beating with nothing granted reads clean on one
/// tick and stale on the next. Keyed to the previous tick, the line came
/// back on every poll (every 100 ms on the J316s).
#[derive(Debug, Default)]
pub struct ReadyNote {
    said: bool,
}

/// What a tick did, for [`ReadyNote::tick`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Spell {
    /// Nothing that starts a new spell.
    Same,
    /// A grant, release, fault, lapse, demand or a changed problem.
    New,
    /// The grant was given back with every check holding: the release is
    /// reported on its own, and readiness is not said again for it.
    GivenBack,
}

impl ReadyNote {
    /// Whether to say the line on this tick: `ready` is this tick's verdict.
    pub fn tick(&mut self, ready: bool, spell: Spell) -> bool {
        match spell {
            Spell::New => self.said = false,
            Spell::GivenBack => {
                self.said = true;
                return false;
            }
            Spell::Same => {}
        }
        if ready && !self.said {
            self.said = true;
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ready_is_said_once_however_the_readings_flicker() {
        let mut n = ReadyNote::default();
        // boot: said on the first ready tick
        assert!(!n.tick(false, Spell::Same));
        assert!(n.tick(true, Spell::Same));
        // an idle engine that beats: clean, stale, clean ... for 21 s of
        // 100 ms polls (the J316s row that said it 211 times)
        let said = (0..420).filter(|i| n.tick(i % 2 == 0, Spell::Same)).count();
        assert_eq!(said, 0);
        // a grant and its release start a new spell
        assert!(!n.tick(false, Spell::New));
        assert!(n.tick(true, Spell::Same));
        assert!(!n.tick(true, Spell::Same));
        // a problem that comes and goes: said again once it holds
        assert!(!n.tick(false, Spell::New));
        assert!(!n.tick(false, Spell::Same));
        // the tick it clears on is a new spell, and ready: said once
        assert!(n.tick(true, Spell::New));
        assert!(!n.tick(true, Spell::Same));
        // the grant given back after a stream: not said for it
        let mut n = ReadyNote::default();
        n.tick(true, Spell::Same);
        n.tick(false, Spell::New);
        assert!(!n.tick(true, Spell::GivenBack));
        assert!(!n.tick(true, Spell::Same));
    }

    const MS: u64 = 1_000_000;

    #[test]
    fn goes_idle_only_after_a_quiet_spell() {
        let mut p = Pace::default();
        assert!(!p.idle());
        assert_eq!(p.after_tick(0, false), Change::None);
        assert_eq!(p.after_tick(IDLE_AFTER_NS - MS, false), Change::None);
        assert!(!p.idle());
        assert_eq!(p.after_tick(IDLE_AFTER_NS, false), Change::Idle);
        assert!(p.idle());
        // stays idle, no repeated change
        assert_eq!(p.after_tick(IDLE_AFTER_NS + 100 * MS, false), Change::None);
        assert!(p.idle());
    }

    #[test]
    fn any_busy_tick_restarts_the_quiet_spell() {
        let mut p = Pace::default();
        p.after_tick(0, false);
        p.after_tick(400 * MS, true);
        assert_eq!(p.after_tick(410 * MS, false), Change::None);
        assert_eq!(p.after_tick(400 * MS + IDLE_AFTER_NS, false), Change::None);
        assert_eq!(p.after_tick(410 * MS + IDLE_AFTER_NS, false), Change::Idle);
    }

    #[test]
    fn a_demand_while_idle_wakes_first_and_counts_only_from_the_next_tick() {
        let mut p = Pace::default();
        p.after_tick(0, false);
        p.after_tick(IDLE_AFTER_NS, false);
        assert!(p.idle());
        // the tick that sees the demand while idle may not act on it
        assert!(!p.demand(true));
        assert_eq!(p.after_tick(IDLE_AFTER_NS + 50 * MS, true), Change::Active);
        assert!(!p.idle());
        // the next tick may
        assert!(p.demand(true));
        assert!(!p.demand(false));
        // and it stays active while busy
        assert_eq!(p.after_tick(IDLE_AFTER_NS + 60 * MS, true), Change::None);
    }

    #[test]
    fn a_grant_alone_keeps_it_active() {
        let mut p = Pace::default();
        for t in 0..100 {
            assert_eq!(p.after_tick(t * 10 * MS, true), Change::None);
        }
        assert!(!p.idle());
    }
}

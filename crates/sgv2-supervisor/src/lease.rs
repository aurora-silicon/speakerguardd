//! The path-integrity state machine, as pure logic.
//!
//! Every tick (10 ms) the supervisor gathers an [`Observation`]. The lease is
//! renewed only while every check holds; any failure releases it at once,
//! and it is taken back only after two consecutive clean ticks.

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
    Release,
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
        if self.held || self.clean_run >= CLEAN_TICKS_TO_LOCK {
            self.held = true;
            Action::Renew
        } else {
            Action::StayReleased
        }
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
    fn one_clean_tick_between_faults_is_not_enough() {
        let mut l = Lease::default();
        let bad = Observation::default();
        assert_eq!(l.tick(CLEAN), Action::StayReleased);
        assert_eq!(l.tick(bad), Action::StayReleased);
        assert_eq!(l.tick(CLEAN), Action::StayReleased);
        assert!(!l.held());
    }
}

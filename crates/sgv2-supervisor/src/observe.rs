//! Gathering one tick's [`Observation`] from the plugin's engine page, the
//! speaker PCM's state and the session helper's report, each behind a trait
//! so the loop is tested without hardware.
//!
//! Every check fails closed: a page that is absent, from another layout, or
//! not advancing, a PCM whose owner is unknown, or a missing session report
//! is a fault, and the lease is not renewed.

use crate::lease::Observation;

/// What the supervisor reads from the plugin's engine page.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EngineState {
    pub valid: bool,
    pub pid: u32,
    pub params_crc: u32,
    pub seq: u64,
    pub mono_ns: u64,
    pub health: u32,
}

pub trait EngineView {
    fn read(&mut self) -> EngineState;
}

/// The speaker playback PCM as `/proc/asound/card<N>/pcm<D>p/sub0/status`
/// shows it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PcmState {
    Closed,
    /// Open but not running (setup, prepared, paused, xrun).
    Open {
        owner: u32,
    },
    Running {
        owner: u32,
    },
    Unknown,
}

pub trait PcmView {
    fn read(&mut self) -> PcmState;
}

/// What the per-user session helper reports about the PipeWire graph: the
/// link set to the speaker sink and sense ports, and the carrier stream's
/// volume. TODO(lane B): the WirePlumber component that writes it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SessionReport {
    pub present: bool,
    pub links_ok: bool,
    pub carrier_ok: bool,
}

pub trait SessionView {
    fn read(&mut self) -> SessionReport;
}

/// Freshness bound while the stream runs: two unit quanta (the
/// architecture's "at most 2 quanta stale") plus one supervisor tick.
pub fn stale_ns(quantum_frames: u32, rate: u32, tick_ns: u64) -> u64 {
    2 * quantum_frames as u64 * 1_000_000_000 / rate.max(1) as u64 + tick_ns
}

pub struct Observer {
    pub expected_params_crc: u32,
    pub stale_ns: u64,
    last_seq: Option<u64>,
    /// When the sequence last advanced. The plugin beats once per graph
    /// quantum (10.7 ms at 512 frames), longer than the 10 ms tick, so some
    /// ticks see no new beat: freshness is an advance within `stale_ns`.
    last_advance_ns: Option<u64>,
}

impl Observer {
    pub fn new(expected_params_crc: u32, stale_ns: u64) -> Self {
        Observer { expected_params_crc, stale_ns, last_seq: None, last_advance_ns: None }
    }

    pub fn observe(
        &mut self,
        now_ns: u64,
        engine: &mut dyn EngineView,
        pcm: &mut dyn PcmView,
        session: &mut dyn SessionView,
    ) -> Observation {
        let e = engine.read();
        let p = pcm.read();
        let s = session.read();
        let plugin_ok = e.valid
            && e.params_crc == self.expected_params_crc
            && e.health & sgv2_core::Health::FAILSAFE == 0
            && e.health & sgv2_core::Health::NO_PARAMS == 0;
        if self.last_seq.is_some_and(|l| e.seq > l) {
            self.last_advance_ns = Some(now_ns);
        }
        let advanced = self.last_advance_ns.is_some_and(|t| now_ns.saturating_sub(t) <= self.stale_ns);
        let recent = now_ns.saturating_sub(e.mono_ns) <= self.stale_ns && e.mono_ns <= now_ns;
        self.last_seq = Some(e.seq);
        let (heartbeat_fresh, pcm_owner_ok) = match p {
            // Idle: nothing can reach the amps; the plugin need not run.
            PcmState::Closed => (e.valid, true),
            // Open: only the plugin's host may hold the speaker PCM.
            PcmState::Open { owner } => (e.valid, owner == e.pid && e.pid != 0),
            PcmState::Running { owner } => (e.valid && advanced && recent, owner == e.pid && e.pid != 0),
            PcmState::Unknown => (false, false),
        };
        Observation {
            heartbeat_fresh,
            plugin_ok,
            links_ok: s.present && s.links_ok,
            carrier_ok: s.present && s.carrier_ok,
            pcm_owner_ok,
        }
    }
}

/// Parse the `/proc` status text of a playback substream.
pub fn parse_pcm_status(text: &str) -> PcmState {
    let t = text.trim();
    if t == "closed" {
        return PcmState::Closed;
    }
    let mut state = None;
    let mut owner = None;
    for line in t.lines() {
        let (k, v) = match line.split_once(':') {
            Some(kv) => kv,
            None => continue,
        };
        match k.trim() {
            "state" => state = Some(v.trim().to_string()),
            "owner_pid" => owner = v.trim().parse::<u32>().ok(),
            _ => {}
        }
    }
    match (state.as_deref(), owner) {
        (Some("RUNNING") | Some("DRAINING"), Some(o)) => PcmState::Running { owner: o },
        (Some(_), Some(o)) => PcmState::Open { owner: o },
        _ => PcmState::Unknown,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    const OK_SESSION: SessionReport = SessionReport { present: true, links_ok: true, carrier_ok: true };

    fn engine(seq: u64, mono: u64) -> EngineState {
        EngineState { valid: true, pid: 4242, params_crc: 7, seq, mono_ns: mono, health: 0 }
    }

    #[test]
    fn running_needs_an_advancing_recent_heartbeat_and_the_plugin_as_owner() {
        let mut o = Observer::new(7, stale_ns(512, 48_000, 10_000_000));
        let mut s = S(OK_SESSION);
        let run = PcmState::Running { owner: 4242 };
        // first tick: no previous sequence, not yet fresh
        assert!(!o.observe(1_000, &mut E(engine(1, 1_000)), &mut P(run), &mut s).clean());
        assert!(o.observe(11_000_000, &mut E(engine(5, 10_000_000)), &mut P(run), &mut s).clean());
        // one tick without a new beat (a quantum is longer than a tick) is still fresh
        assert!(o.observe(21_000_000, &mut E(engine(5, 10_000_000)), &mut P(run), &mut s).heartbeat_fresh);
        // stalled sequence: no advance for longer than the staleness bound
        assert!(!o.observe(45_000_000, &mut E(engine(5, 10_000_000)), &mut P(run), &mut s).heartbeat_fresh);
        // advancing but old timestamp
        let obs = o.observe(200_000_000, &mut E(engine(9, 100_000_000)), &mut P(run), &mut s);
        assert!(!obs.heartbeat_fresh);
        // a foreign owner (raw PCM bypass)
        let obs =
            o.observe(210_000_000, &mut E(engine(10, 205_000_000)), &mut P(PcmState::Running { owner: 1 }), &mut s);
        assert!(!obs.pcm_owner_ok);
    }

    #[test]
    fn idle_is_clean_with_a_valid_page_and_failsafe_or_wrong_params_are_not() {
        let mut o = Observer::new(7, 1);
        let mut s = S(OK_SESSION);
        assert!(o.observe(5, &mut E(engine(0, 0)), &mut P(PcmState::Closed), &mut s).clean());
        let mut bad = engine(0, 0);
        bad.params_crc = 8;
        assert!(!o.observe(5, &mut E(bad), &mut P(PcmState::Closed), &mut s).plugin_ok);
        let mut fs = engine(0, 0);
        fs.health = sgv2_core::Health::FAILSAFE;
        assert!(!o.observe(5, &mut E(fs), &mut P(PcmState::Closed), &mut s).plugin_ok);
        assert!(!o.observe(5, &mut E(EngineState::default()), &mut P(PcmState::Closed), &mut s).clean());
    }

    #[test]
    fn a_missing_session_report_fails_closed() {
        let mut o = Observer::new(7, 1);
        let obs = o.observe(5, &mut E(engine(0, 0)), &mut P(PcmState::Closed), &mut S(SessionReport::default()));
        assert!(!obs.links_ok && !obs.carrier_ok && !obs.clean());
    }

    #[test]
    fn proc_status_parsing() {
        assert_eq!(parse_pcm_status("closed\n"), PcmState::Closed);
        let run = "state: RUNNING\nowner_pid   : 342135\ntrigger_time: 1.0\n";
        assert_eq!(parse_pcm_status(run), PcmState::Running { owner: 342135 });
        let prep = "state: PREPARED\nowner_pid   : 12\n";
        assert_eq!(parse_pcm_status(prep), PcmState::Open { owner: 12 });
        assert_eq!(parse_pcm_status("garbage"), PcmState::Unknown);
    }
}

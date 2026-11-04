//! Gathering one tick's [`Observation`] from the plugin's engine page, the
//! speaker PCM's state and the session helper's report, each behind a trait
//! so the loop is tested without hardware.
//!
//! Every check fails closed: a page that is absent, from another layout, or
//! not advancing, a PCM whose owner is unknown, or a missing, stale or
//! future-dated session report is a fault, and the lease is not renewed.
//!
//! # Proof that the engine runs
//!
//! The heartbeat must be fresh in every PCM state, Closed included: the
//! lease is never renewed on a page that merely exists. That works because
//! the protect stage runs before the amps are linked (`node.always-process`,
//! driven by PipeWire's dummy driver until the amps' sink joins its graph).
//! The grant-before-start sequence is:
//!
//! 1. The supervisor owns "Speaker Protection"; the kernel holds the floor.
//! 2. PipeWire instantiates the plugin, which maps the page, and the protect
//!    stage runs on the dummy driver: the heartbeat advances while the
//!    speaker PCM is Closed.
//! 3. Two clean ticks (fresh heartbeat, plugin healthy with the published
//!    parameters, the helper's report fresh with only allowed links, the
//!    speaker mux on the watched PCM, the amps at the operating point) and
//!    the supervisor grants.
//! 4. The session helper sees the grant and links the amps; the amps' sink
//!    opens and prepares the PCM and starts it (the kernel admits the start
//!    because the grant is live), and the driver moves to the sink.
//!    - While the PCM is OPEN, SETUP or PREPARED within [`START_GRACE_NS`]
//!      of the first tick that saw it open, the supervisor holds: it
//!      neither renews nor releases ([`Observation::pcm_settling`]). The
//!      30 ms lease covers that; a start slower than the lease lapses, and
//!      the grant comes back through two clean ticks.
//!    - The heartbeat may pause across the driver switch: the same window
//!      adds [`START_GRACE_NS`] to its bound, once per start.
//! 5. From then on only RUNNING renews. XRUN, PAUSED, SUSPENDED or a
//!    PREPARED state outside the start window releases at once, as does any
//!    other fault. The kernel stops the stream, the helper unlinks the amps,
//!    the protect stage falls back to the dummy driver and beats again, the
//!    PCM closes, and the sequence resumes at 3.
//!
//! A dead or stopped engine therefore releases within one staleness bound,
//! or sooner when the PCM XRUNs, and nothing renews without it.

use crate::lease::Observation;

/// What the supervisor reads from the plugin's engine page.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EngineState {
    pub valid: bool,
    pub pid: u32,
    pub params_crc: u32,
    pub seq: u64,
    /// Frames processed in all, as of `seq`.
    pub frames: u64,
    pub mono_ns: u64,
    pub health: u32,
}

pub trait EngineView {
    fn read(&mut self) -> EngineState;
}

/// The speaker playback PCM as `/proc/asound/card<N>/pcm<D>p/sub*/status`
/// shows it, every substream folded into one. Owners are process ids
/// (tgid), never thread ids.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PcmState {
    Closed,
    /// Open and being set up or started (OPEN, SETUP, PREPARED).
    Open {
        owner: u32,
    },
    /// Open and stopped by anything but a close: XRUN (the kernel's own
    /// withdrawal, or an engine that stopped feeding it), PAUSED,
    /// SUSPENDED, DISCONNECTED. Never renewed.
    Stopped {
        owner: u32,
    },
    Running {
        owner: u32,
    },
    /// Not readable, or not read recently enough: no proof of anything.
    Unknown,
}

impl PcmState {
    /// Packed for a lock-free snapshot: kind in the high bits, owner low.
    pub fn pack(self) -> u64 {
        let (k, o) = match self {
            PcmState::Closed => (0u64, 0),
            PcmState::Open { owner } => (1, owner),
            PcmState::Stopped { owner } => (2, owner),
            PcmState::Running { owner } => (3, owner),
            PcmState::Unknown => (4, 0),
        };
        k << 32 | u64::from(o)
    }

    pub fn unpack(v: u64) -> PcmState {
        let owner = v as u32;
        match v >> 32 {
            0 => PcmState::Closed,
            1 => PcmState::Open { owner },
            2 => PcmState::Stopped { owner },
            3 => PcmState::Running { owner },
            _ => PcmState::Unknown,
        }
    }

    fn owner(self) -> Option<u32> {
        match self {
            PcmState::Open { owner } | PcmState::Stopped { owner } | PcmState::Running { owner } => Some(owner),
            _ => None,
        }
    }
}

pub trait PcmView {
    fn read(&mut self) -> PcmState;
}

/// What the per-user session helper reports about the PipeWire graph: the
/// link set to the speaker sink and sense ports, the carrier stream's
/// volume, and when (CLOCK_MONOTONIC) it looked.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SessionReport {
    pub present: bool,
    pub links_ok: bool,
    pub carrier_ok: bool,
    pub mono_ns: u64,
}

pub trait SessionView {
    fn read(&mut self) -> SessionReport;
}

/// The largest graph quantum the heartbeat bound accepts (PipeWire's
/// default `clock.max-quantum`). A page claiming more is not fresh.
pub const MAX_QUANTUM: u64 = 2048;
/// The helper writes every 50 ms: two periods.
pub const SESSION_STALE_NS: u64 = 100_000_000;
/// How far in the future a session report may be dated. The helper and
/// the supervisor read the same CLOCK_MONOTONIC and the supervisor reads its
/// clock after the file, so a report is never ahead: 1 ms is slack only.
pub const SESSION_SKEW_NS: u64 = 1_000_000;
/// Extra heartbeat age tolerated once, from the tick a speaker PCM start is
/// first seen (Closed to Open or Running), while PipeWire moves the protect
/// stage from the dummy driver to the amps' sink. Steady state has none.
pub const START_GRACE_NS: u64 = 50_000_000;

/// Freshness bound while the engine runs: two unit quanta (the
/// architecture's "at most 2 quanta stale") plus one supervisor tick.
pub fn stale_ns(quantum_frames: u64, rate: u32, tick_ns: u64) -> u64 {
    2 * quantum_frames * 1_000_000_000 / rate.max(1) as u64 + tick_ns
}

pub struct Observer {
    pub expected_params_crc: u32,
    pub rate: u32,
    pub tick_ns: u64,
    last_seq: Option<u64>,
    last_frames: u64,
    /// When the sequence was last seen to advance.
    last_advance_ns: Option<u64>,
    /// The graph quantum, from frames over beats between two advances.
    quantum: Option<u64>,
    pcm_was_closed: bool,
    grace_until: Option<u64>,
}

impl Observer {
    pub fn new(expected_params_crc: u32, rate: u32, tick_ns: u64) -> Self {
        Observer {
            expected_params_crc,
            rate,
            tick_ns,
            last_seq: None,
            last_frames: 0,
            last_advance_ns: None,
            quantum: None,
            pcm_was_closed: true,
            grace_until: None,
        }
    }

    /// The current heartbeat staleness bound, once the quantum is known.
    pub fn bound_ns(&self) -> Option<u64> {
        self.quantum.map(|q| stale_ns(q, self.rate, self.tick_ns))
    }

    /// One tick. `clock` is read after the page, the PCM and the report, so
    /// a beat that lands between the reads is never dated in the future.
    pub fn observe(
        &mut self,
        clock: &mut dyn FnMut() -> u64,
        engine: &mut dyn EngineView,
        pcm: &mut dyn PcmView,
        session: &mut dyn SessionView,
    ) -> Observation {
        let e = engine.read();
        let p = pcm.read();
        let s = session.read();
        let now_ns = clock();
        let plugin_ok = e.valid
            && e.params_crc == self.expected_params_crc
            && e.health & sgv2_core::Health::FAILSAFE == 0
            && e.health & sgv2_core::Health::NO_PARAMS == 0;

        match self.last_seq {
            Some(l) if e.seq > l => {
                let beats = e.seq - l;
                self.quantum = e.frames.checked_sub(self.last_frames).filter(|f| *f > 0).map(|f| f / beats);
                self.last_advance_ns = Some(now_ns);
            }
            Some(l) if e.seq < l => {
                // a new page or a restarted plugin: start over
                self.quantum = None;
                self.last_advance_ns = None;
            }
            _ => {}
        }
        self.last_seq = Some(e.seq);
        self.last_frames = e.frames;

        let closed = matches!(p, PcmState::Closed);
        if self.pcm_was_closed && matches!(p, PcmState::Open { .. } | PcmState::Running { .. }) {
            self.grace_until = Some(now_ns + START_GRACE_NS);
        }
        self.pcm_was_closed = closed;
        let in_start = match self.grace_until {
            Some(t) if now_ns < t && matches!(p, PcmState::Open { .. } | PcmState::Running { .. }) => true,
            _ => {
                self.grace_until = None;
                false
            }
        };
        let grace = if in_start { START_GRACE_NS } else { 0 };

        let heartbeat_fresh = match self.quantum.filter(|q| (1..=MAX_QUANTUM).contains(q)) {
            Some(q) => {
                let bound = stale_ns(q, self.rate, self.tick_ns) + grace;
                let advanced = self.last_advance_ns.is_some_and(|t| now_ns.saturating_sub(t) <= bound);
                let recent = e.mono_ns <= now_ns && now_ns - e.mono_ns <= bound;
                e.valid && advanced && recent
            }
            None => false,
        };
        // only the plugin's host process may hold the speaker PCM
        let pcm_owner_ok = match p.owner() {
            Some(owner) => owner == e.pid && e.pid != 0,
            None => closed,
        };
        let (pcm_state_ok, pcm_settling) = match p {
            PcmState::Closed | PcmState::Running { .. } => (true, false),
            PcmState::Open { .. } => (in_start, in_start),
            PcmState::Stopped { .. } | PcmState::Unknown => (false, false),
        };
        let session_fresh = s.present
            && s.mono_ns <= now_ns.saturating_add(SESSION_SKEW_NS)
            && now_ns.saturating_sub(s.mono_ns) <= SESSION_STALE_NS;
        Observation {
            heartbeat_fresh,
            plugin_ok,
            links_ok: session_fresh && s.links_ok,
            carrier_ok: session_fresh && s.carrier_ok,
            pcm_owner_ok,
            pcm_state_ok,
            pcm_settling,
            // the supervisor fills these from the kernel
            route_ok: false,
            amps_ok: false,
        }
    }
}

/// Parse the `/proc` status text of a playback substream. The owner is the
/// opening thread's id, as the kernel prints it (`task_pid(current)`).
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
        (Some("RUNNING" | "DRAINING"), Some(o)) => PcmState::Running { owner: o },
        (Some("OPEN" | "SETUP" | "PREPARED"), Some(o)) => PcmState::Open { owner: o },
        (Some(_), Some(o)) => PcmState::Stopped { owner: o },
        _ => PcmState::Unknown,
    }
}

/// Map a state's owner from a thread id to its process id with `tgid`; an
/// owner that cannot be mapped is Unknown.
pub fn owner_to_tgid(state: PcmState, tgid: &mut dyn FnMut(u32) -> Option<u32>) -> PcmState {
    match state {
        PcmState::Open { owner } => tgid(owner).map_or(PcmState::Unknown, |o| PcmState::Open { owner: o }),
        PcmState::Stopped { owner } => tgid(owner).map_or(PcmState::Unknown, |o| PcmState::Stopped { owner: o }),
        PcmState::Running { owner } => tgid(owner).map_or(PcmState::Unknown, |o| PcmState::Running { owner: o }),
        s => s,
    }
}

/// The `Tgid:` line of `/proc/<tid>/status`.
pub fn parse_tgid(status: &str) -> Option<u32> {
    status.lines().find_map(|l| l.strip_prefix("Tgid:")).and_then(|v| v.trim().parse().ok())
}

/// Fold every substream of the speaker PCM into one state. The worst state
/// wins (Unknown, then Stopped, Running, Open, Closed), and two different
/// owners are a foreign opener (owner 0, which never matches the plugin).
pub fn combine(states: &[PcmState]) -> PcmState {
    fn rank(x: PcmState) -> u8 {
        match x {
            PcmState::Closed => 0,
            PcmState::Open { .. } => 1,
            PcmState::Running { .. } => 2,
            PcmState::Stopped { .. } => 3,
            PcmState::Unknown => 4,
        }
    }
    let mut out = PcmState::Closed;
    let mut owner: Option<u32> = None;
    for &s in states {
        if let Some(o) = s.owner() {
            owner = Some(match owner {
                Some(prev) if prev != o => 0,
                _ => o,
            });
        }
        if rank(s) > rank(out) {
            out = s;
        }
    }
    let o = owner.unwrap_or(0);
    match out {
        PcmState::Open { .. } => PcmState::Open { owner: o },
        PcmState::Stopped { .. } => PcmState::Stopped { owner: o },
        PcmState::Running { .. } => PcmState::Running { owner: o },
        s => s,
    }
}

/// A playback PCM's `/proc/asound/card<N>/pcm<D>p/info`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PcmInfo {
    pub card: u32,
    pub device: u32,
    pub playback: bool,
    /// The front-end's name, without the " (*)" ASoC adds.
    pub id: String,
    pub subdevices: u32,
}

pub fn parse_pcm_info(text: &str) -> Option<PcmInfo> {
    let get = |k: &str| {
        text.lines().find_map(|l| l.split_once(':').filter(|(a, _)| a.trim() == k).map(|(_, v)| v.trim().to_owned()))
    };
    let id = get("id")?;
    let id = id.strip_suffix("(*)").map(str::trim_end).unwrap_or(&id).to_owned();
    Some(PcmInfo {
        card: get("card")?.parse().ok()?,
        device: get("device")?.parse().ok()?,
        playback: get("stream")? == "PLAYBACK",
        id,
        subdevices: get("subdevices_count")?.parse().ok()?,
    })
}

/// Why `info` is not a macaudio speaker front-end on card `card` at
/// `device`, or `None` if it is one. Which front-end the speaker mux
/// selects is checked on every tick.
pub fn speaker_pcm_problem(info: &PcmInfo, card: u32, device: u32) -> Option<String> {
    if info.card != card || info.device != device {
        return Some(format!("pcm info names card {} device {}", info.card, info.device));
    }
    if !info.playback {
        return Some("not a playback PCM".into());
    }
    if info.id != "Primary" && info.id != "Secondary" {
        return Some(format!("\"{}\" is not a macaudio speaker front-end", info.id));
    }
    if info.subdevices == 0 || info.subdevices > 8 {
        return Some(format!("{} substreams", info.subdevices));
    }
    None
}

/// Parse the helper's report, `links=<0|1> carrier=<0|1> mono_ns=<t>\n`.
/// Anything else, including a report without its final newline, is absent.
pub fn parse_session(text: &str) -> SessionReport {
    let Some(body) = text.strip_suffix('\n') else { return SessionReport::default() };
    let mut links = None;
    let mut carrier = None;
    let mut mono = None;
    for w in body.split_whitespace() {
        let Some((k, v)) = w.split_once('=') else { return SessionReport::default() };
        let slot = match k {
            "links" => &mut links,
            "carrier" => &mut carrier,
            "mono_ns" => &mut mono,
            _ => continue,
        };
        if slot.is_some() {
            return SessionReport::default();
        }
        *slot = Some(v.parse::<u64>().ok());
    }
    match (links.flatten(), carrier.flatten(), mono.flatten()) {
        (Some(l @ 0..=1), Some(c @ 0..=1), Some(m)) if m > 0 => {
            SessionReport { present: true, links_ok: l == 1, carrier_ok: c == 1, mono_ns: m }
        }
        _ => SessionReport::default(),
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

    const TICK: u64 = 10_000_000;
    const PID: u32 = 4242;

    fn session(at: u64) -> SessionReport {
        SessionReport { present: true, links_ok: true, carrier_ok: true, mono_ns: at }
    }

    fn beat(seq: u64, quantum: u64, mono_ns: u64) -> EngineState {
        EngineState { valid: true, pid: PID, params_crc: 7, seq, frames: seq * quantum, mono_ns, health: 0 }
    }

    /// A healthy engine beating once per `quantum` frames at 48 kHz from
    /// t = 0, and a helper reporting every 50 ms, observed by 10 ms ticks
    /// that start at `phase_ns` and run late by up to `jitter_ns`. Returns
    /// the faulted ticks after the first 100 ms (the quantum is learnt then).
    fn simulate(quantum: u64, pcm: PcmState, seconds: u64, phase_ns: u64, jitter_ns: u64) -> usize {
        let period = quantum * 1_000_000_000 / 48_000;
        let mut o = Observer::new(7, 48_000, TICK);
        let mut faults = 0;
        for k in 0..seconds * 100 {
            let now = phase_ns + k * TICK + (k * 7919 % 13) * jitter_ns / 12;
            let beats = now / period;
            let r = session(now / 50_000_000 * 50_000_000 + 1);
            let obs = o.observe(&mut || now, &mut E(beat(beats, quantum, beats * period)), &mut P(pcm), &mut S(r));
            if k >= 10
                && !(obs.heartbeat_fresh && obs.plugin_ok && obs.links_ok && obs.pcm_owner_ok && obs.pcm_state_ok)
            {
                faults += 1;
            }
        }
        faults
    }

    #[test]
    fn a_healthy_engine_is_fresh_at_every_quantum() {
        let run = PcmState::Running { owner: PID };
        for q in [256, 512, 1024, 2048] {
            for phase in [0, 1_000_000, 3_333_333, 9_999_999] {
                assert_eq!(simulate(q, run, 10, phase, 0), 0, "quantum {q}, phase {phase}");
                // the supervisor's own wakeups late by up to 2 ms
                assert_eq!(simulate(q, run, 10, phase, 2_000_000), 0, "quantum {q}, phase {phase}, jitter");
            }
            // idle (the protect stage on the dummy driver) needs the same proof
            assert_eq!(simulate(q, PcmState::Closed, 5, 0, 0), 0, "quantum {q} closed");
        }
    }

    #[test]
    fn the_bound_follows_the_quantum() {
        let mut o = Observer::new(7, 48_000, TICK);
        let mut s = S(session(1));
        o.observe(&mut || 1, &mut E(beat(1, 1024, 1)), &mut P(PcmState::Closed), &mut s);
        assert_eq!(o.bound_ns(), None);
        o.observe(&mut || 21_400_000, &mut E(beat(2, 1024, 21_333_333)), &mut P(PcmState::Closed), &mut s);
        assert_eq!(o.bound_ns(), Some(stale_ns(1024, 48_000, TICK)));
        assert_eq!(stale_ns(1024, 48_000, TICK), 52_666_666);
        assert_eq!(stale_ns(512, 48_000, TICK), 31_333_333);
    }

    #[test]
    fn a_dead_engine_is_stale_within_the_bound_in_every_pcm_state() {
        for pcm in [PcmState::Closed, PcmState::Running { owner: PID }] {
            let mut o = Observer::new(7, 48_000, TICK);
            let period = 10_666_666;
            let mut last_fresh = 0;
            for k in 0..30u64 {
                let now = k * TICK;
                // the engine dies at 100 ms
                let beats = now.min(100_000_000) / period;
                let obs =
                    o.observe(&mut || now, &mut E(beat(beats, 512, beats * period)), &mut P(pcm), &mut S(session(now)));
                if obs.heartbeat_fresh {
                    last_fresh = now;
                }
            }
            let last_beat = 100_000_000 / period * period;
            assert!(last_fresh > 0, "{pcm:?}");
            assert!(last_fresh - last_beat <= stale_ns(512, 48_000, TICK), "{pcm:?}: fresh until {last_fresh}");
        }
    }

    #[test]
    fn the_page_existing_is_not_proof_of_an_engine() {
        // magic and the right parameters, but nothing beats: never fresh,
        // not even while the PCM is closed
        let mut o = Observer::new(7, 48_000, TICK);
        let e = EngineState { valid: true, pid: PID, params_crc: 7, ..Default::default() };
        for k in 0..20 {
            let obs = o.observe(&mut || k * TICK, &mut E(e), &mut P(PcmState::Closed), &mut S(session(k * TICK)));
            assert!(!obs.heartbeat_fresh);
        }
    }

    #[test]
    fn an_absurd_quantum_is_not_fresh() {
        let mut o = Observer::new(7, 48_000, TICK);
        o.observe(&mut || 1, &mut E(beat(1, 100_000, 1)), &mut P(PcmState::Closed), &mut S(session(1)));
        let obs = o.observe(&mut || 10, &mut E(beat(2, 100_000, 10)), &mut P(PcmState::Closed), &mut S(session(10)));
        assert!(!obs.heartbeat_fresh);
    }

    #[test]
    fn a_beat_dated_after_the_clock_is_not_fresh_and_the_clock_is_read_last() {
        let mut o = Observer::new(7, 48_000, TICK);
        let mut s = S(session(1));
        o.observe(&mut || 1, &mut E(beat(1, 512, 1)), &mut P(PcmState::Closed), &mut s);
        let obs = o.observe(&mut || 20_000_000, &mut E(beat(2, 512, 30_000_000)), &mut P(PcmState::Closed), &mut s);
        assert!(!obs.heartbeat_fresh);

        use std::cell::RefCell;
        let order = RefCell::new(Vec::new());
        struct Logged<'a>(&'a RefCell<Vec<&'static str>>);
        impl EngineView for Logged<'_> {
            fn read(&mut self) -> EngineState {
                self.0.borrow_mut().push("page");
                beat(1, 512, 1)
            }
        }
        let mut o = Observer::new(7, 48_000, TICK);
        o.observe(
            &mut || {
                order.borrow_mut().push("clock");
                5
            },
            &mut Logged(&order),
            &mut P(PcmState::Closed),
            &mut s,
        );
        assert_eq!(*order.borrow(), vec!["page", "clock"]);
    }

    #[test]
    fn the_first_start_has_a_grace_and_steady_state_does_not() {
        let period = 10_666_666u64;
        let run = PcmState::Running { owner: PID };
        let mut o = Observer::new(7, 48_000, TICK);
        let mut tick = |now: u64, seq: u64, mono: u64, pcm: PcmState| {
            o.observe(&mut || now, &mut E(beat(seq, 512, mono)), &mut P(pcm), &mut S(session(now))).heartbeat_fresh
        };
        // idle, beating on the dummy driver
        let (mut seq, mut mono) = (0, 0);
        for k in 0..10u64 {
            let now = k * TICK;
            seq = now / period;
            mono = seq * period;
            let fresh = tick(now, seq, mono, PcmState::Closed);
            assert!(fresh || k < 2, "idle tick {k}");
        }
        // at 100 ms the sink starts, and the engine stalls for 50 ms while
        // the driver moves (60 ms after its last beat): inside the grace
        for k in 0..5u64 {
            assert!(tick(100_000_000 + k * TICK, seq, mono, run), "start tick {k}");
        }
        // it resumes at 150 ms and runs
        for k in 0..20u64 {
            seq += 1;
            mono = 150_000_000 + k * period;
            assert!(tick(mono + 1_000, seq, mono, run), "steady beat {k}");
        }
        // a stall in steady state gets no grace
        let bound = stale_ns(512, 48_000, TICK);
        assert!(tick(mono + bound - 1_000_000, seq, mono, run));
        assert!(!tick(mono + bound + 1_000_000, seq, mono, run));
    }

    #[test]
    fn the_grace_is_never_a_way_around_the_owner_or_the_report() {
        let mut o = Observer::new(7, 48_000, TICK);
        let e = beat(1, 512, 1);
        o.observe(&mut || 1, &mut E(e), &mut P(PcmState::Closed), &mut S(session(1)));
        let obs = o.observe(&mut || 2, &mut E(e), &mut P(PcmState::Running { owner: 1 }), &mut S(session(2)));
        assert!(!obs.pcm_owner_ok);
        let obs = o.observe(&mut || 3, &mut E(e), &mut P(PcmState::Running { owner: PID }), &mut S(Default::default()));
        assert!(!obs.links_ok && !obs.carrier_ok);
    }

    #[test]
    fn the_plugin_must_own_the_pcm_and_failsafe_or_wrong_params_are_not_ok() {
        let mut o = Observer::new(7, 48_000, TICK);
        let mut s = S(session(5));
        let e = beat(0, 512, 0);
        assert!(!o.observe(&mut || 5, &mut E(e), &mut P(PcmState::Running { owner: 1 }), &mut s).pcm_owner_ok);
        assert!(!o.observe(&mut || 5, &mut E(e), &mut P(PcmState::Open { owner: 1 }), &mut s).pcm_owner_ok);
        assert!(o.observe(&mut || 5, &mut E(e), &mut P(PcmState::Open { owner: PID }), &mut s).pcm_owner_ok);
        assert!(!o.observe(&mut || 5, &mut E(e), &mut P(PcmState::Stopped { owner: 1 }), &mut s).pcm_owner_ok);
        assert!(!o.observe(&mut || 5, &mut E(e), &mut P(PcmState::Unknown), &mut s).pcm_owner_ok);
        let mut bad = e;
        bad.params_crc = 8;
        assert!(!o.observe(&mut || 5, &mut E(bad), &mut P(PcmState::Closed), &mut s).plugin_ok);
        let mut fs = e;
        fs.health = sgv2_core::Health::FAILSAFE;
        assert!(!o.observe(&mut || 5, &mut E(fs), &mut P(PcmState::Closed), &mut s).plugin_ok);
        assert!(!o.observe(&mut || 5, &mut E(EngineState::default()), &mut P(PcmState::Closed), &mut s).plugin_ok);
    }

    #[test]
    fn session_reports_must_be_fresh_and_never_from_the_future() {
        let mut o = Observer::new(7, 48_000, TICK);
        let now = 10_000_000_000u64;
        let mut at = |r: SessionReport| {
            o.observe(&mut || now, &mut E(EngineState::default()), &mut P(PcmState::Closed), &mut S(r)).links_ok
        };
        assert!(at(session(now)));
        assert!(at(session(now - SESSION_STALE_NS)));
        assert!(!at(session(now - SESSION_STALE_NS - 1)));
        assert!(at(session(now + SESSION_SKEW_NS)));
        // a stamp from the future (another clock, or CLOCK_BOOTTIME after a
        // suspend) is never fresh
        assert!(!at(session(now + SESSION_SKEW_NS + 1)));
        assert!(!at(session(u64::MAX)));
        assert!(!at(SessionReport::default()));
    }

    #[test]
    fn session_parsing_rejects_partial_and_mixed_reports() {
        let r = parse_session("links=1 carrier=1 mono_ns=123\n");
        assert_eq!(r, SessionReport { present: true, links_ok: true, carrier_ok: true, mono_ns: 123 });
        let r = parse_session("links=0 carrier=1 mono_ns=123\n");
        assert!(r.present && !r.links_ok);
        for bad in [
            "",
            "links=1 carrier=1 mono_ns=123",
            "links=1 carrier=1 mono_ns=\n",
            "links=1 carrier=1\n",
            "links=1 carrier=1 mono_ns=12 mono_ns=13\n",
            "links=2 carrier=1 mono_ns=12\n",
            "links=1 carrier=1 mono_ns=0\n",
            "links=1 carrier=1 mono_ns=12 junk\n",
            "links=1 carrier=1 mono_ns=12\nlinks=1 carrier=1 mono_ns=13\n",
        ] {
            assert!(!parse_session(bad).present, "{bad:?}");
        }
    }

    #[test]
    fn proc_status_parsing_and_owner_mapping() {
        assert_eq!(parse_pcm_status("closed\n"), PcmState::Closed);
        let run = "state: RUNNING\nowner_pid   : 342135\ntrigger_time: 1.0\n";
        assert_eq!(parse_pcm_status(run), PcmState::Running { owner: 342135 });
        let prep = "state: PREPARED\nowner_pid   : 12\n";
        assert_eq!(parse_pcm_status(prep), PcmState::Open { owner: 12 });
        assert_eq!(parse_pcm_status("garbage"), PcmState::Unknown);
        for stopped in ["XRUN", "PAUSED", "SUSPENDED", "DISCONNECTED"] {
            let t = format!("state: {stopped}\nowner_pid   : 12\n");
            assert_eq!(parse_pcm_status(&t), PcmState::Stopped { owner: 12 }, "{stopped}");
        }
        for st in [
            PcmState::Closed,
            PcmState::Open { owner: 7 },
            PcmState::Stopped { owner: 8 },
            PcmState::Running { owner: u32::MAX },
            PcmState::Unknown,
        ] {
            assert_eq!(PcmState::unpack(st.pack()), st);
        }
        // owner_pid is the opening thread (a PipeWire data-loop thread);
        // the page stores the process id
        let status = "Name:\tdata-loop.0\nUmask:\t0077\nState:\tS (sleeping)\nTgid:\t4242\nNgid:\t0\nPid:\t4250\n";
        assert_eq!(parse_tgid(status), Some(4242));
        let mut map = |tid: u32| if tid == 4250 { Some(4242) } else { None };
        assert_eq!(owner_to_tgid(PcmState::Running { owner: 4250 }, &mut map), PcmState::Running { owner: 4242 });
        assert_eq!(owner_to_tgid(PcmState::Open { owner: 1 }, &mut map), PcmState::Unknown);
        assert_eq!(owner_to_tgid(PcmState::Closed, &mut map), PcmState::Closed);
        // the thread's own id never passes for the process
        let mut o = Observer::new(7, 48_000, TICK);
        let obs = o.observe(
            &mut || 5,
            &mut E(beat(0, 512, 0)),
            &mut P(PcmState::Running { owner: 4250 }),
            &mut S(session(5)),
        );
        assert!(!obs.pcm_owner_ok);
    }

    #[test]
    fn substreams_fold_into_one_state() {
        use PcmState::*;
        assert_eq!(combine(&[Closed, Closed]), Closed);
        assert_eq!(combine(&[Closed, Open { owner: 5 }]), Open { owner: 5 });
        assert_eq!(combine(&[Open { owner: 5 }, Running { owner: 5 }]), Running { owner: 5 });
        assert_eq!(combine(&[Running { owner: 5 }, Open { owner: 6 }]), Running { owner: 0 });
        assert_eq!(combine(&[Running { owner: 5 }, Unknown]), Unknown);
        assert_eq!(combine(&[Running { owner: 5 }, Stopped { owner: 5 }]), Stopped { owner: 5 });
        assert_eq!(combine(&[Open { owner: 6 }, Running { owner: 5 }]), Running { owner: 0 });
        assert_eq!(combine(&[]), Closed);
    }

    #[test]
    fn only_a_macaudio_playback_front_end_is_the_speaker_pcm() {
        let info = "card: 1\ndevice: 1\nsubdevice: 0\nstream: PLAYBACK\nid: Secondary (*)\nname: \n\
                    subname: subdevice #0\nclass: 0\nsubclass: 0\nsubdevices_count: 1\nsubdevices_avail: 1\n";
        let i = parse_pcm_info(info).unwrap();
        assert_eq!((i.card, i.device, i.playback, i.id.as_str(), i.subdevices), (1, 1, true, "Secondary", 1));
        assert_eq!(speaker_pcm_problem(&i, 1, 1), None);
        assert!(speaker_pcm_problem(&i, 1, 0).is_some());
        assert!(speaker_pcm_problem(&i, 0, 1).is_some());
        let sense = info.replace("PLAYBACK", "CAPTURE").replace("Secondary", "Speaker Sense");
        assert!(speaker_pcm_problem(&parse_pcm_info(&sense).unwrap(), 1, 1).is_some());
        let hdmi = info.replace("Secondary (*)", "HDMI 0");
        assert!(speaker_pcm_problem(&parse_pcm_info(&hdmi).unwrap(), 1, 1).is_some());
        assert_eq!(parse_pcm_info("card: 1\n"), None);
    }

    #[test]
    fn any_stopped_state_is_never_renewed_and_prepared_only_settles_at_a_start() {
        let mut o = Observer::new(7, 48_000, TICK);
        let period = 10_666_666u64;
        let mut seq = 0;
        let mut at = |o: &mut Observer, now: u64, pcm: PcmState| {
            seq = now / period;
            o.observe(&mut || now, &mut E(beat(seq, 512, seq * period)), &mut P(pcm), &mut S(session(now)))
        };
        for k in 0..5 {
            at(&mut o, k * TICK, PcmState::Closed);
        }
        // the sink opens and prepares: hold, neither renew nor release
        let obs = at(&mut o, 50_000_000, PcmState::Open { owner: PID });
        assert!(obs.pcm_state_ok && obs.pcm_settling);
        let obs = at(&mut o, 60_000_000, PcmState::Running { owner: PID });
        assert!(obs.pcm_state_ok && !obs.pcm_settling && obs.heartbeat_fresh);
        // an XRUN (the engine stopped feeding the amps) is a fault at once,
        // with the heartbeat still fresh
        let obs = at(&mut o, 70_000_000, PcmState::Stopped { owner: PID });
        assert!(!obs.pcm_state_ok && obs.heartbeat_fresh);
        // re-prepared after an XRUN, outside a start from Closed: a fault
        let obs = at(&mut o, 80_000_000, PcmState::Open { owner: PID });
        assert!(!obs.pcm_state_ok && !obs.pcm_settling);
        // a start that never runs stops settling after the window
        let mut o = Observer::new(7, 48_000, TICK);
        for k in 0..5 {
            at(&mut o, k * TICK, PcmState::Closed);
        }
        let open_at = 50_000_000;
        assert!(at(&mut o, open_at, PcmState::Open { owner: PID }).pcm_settling);
        assert!(at(&mut o, open_at + START_GRACE_NS - TICK, PcmState::Open { owner: PID }).pcm_settling);
        let obs = at(&mut o, open_at + START_GRACE_NS, PcmState::Open { owner: PID });
        assert!(!obs.pcm_state_ok && !obs.pcm_settling);
        // unknown (unreadable, or a read that did not complete in time)
        assert!(!at(&mut o, open_at + START_GRACE_NS + TICK, PcmState::Unknown).pcm_state_ok);
    }

    #[test]
    fn a_killed_engine_with_the_pcm_closed_never_regrants() {
        // B2, J616s: after a SIGKILL the page keeps its magic, pid and
        // parameters; the grant must not come back on that alone
        let mut o = Observer::new(7, 48_000, TICK);
        let period = 10_666_666u64;
        let mut last = 0;
        for k in 0..10u64 {
            last = k * TICK / period;
            o.observe(
                &mut || k * TICK,
                &mut E(beat(last, 512, last * period)),
                &mut P(PcmState::Closed),
                &mut S(session(k * TICK)),
            );
        }
        let mut fresh_after = 0;
        for k in 10..60u64 {
            let now = k * TICK;
            let obs = o.observe(
                &mut || now,
                &mut E(beat(last, 512, last * period)),
                &mut P(PcmState::Closed),
                &mut S(session(now)),
            );
            if obs.heartbeat_fresh {
                fresh_after = now;
            }
        }
        assert!(fresh_after <= last * period + stale_ns(512, 48_000, TICK));
    }
}

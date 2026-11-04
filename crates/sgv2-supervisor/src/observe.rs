//! Gathering one tick's [`Observation`] from the plugin's engine page, the
//! speaker PCM's state and the session helper's report, each behind a trait
//! so the loop is tested without hardware.
//!
//! Every check fails closed: a page that is absent, from another layout, or
//! not advancing, a PCM whose owner is unknown, or a missing, stale or
//! future-dated session report is a fault, and the lease is not renewed.
//!
//! A session report counts only while the helper that wrote it is alive
//! (its pid and start time, checked against /proc), and only if it was
//! written after the current engine generation began (the engine page's
//! process changed, the page became valid, or its sequence restarted). A
//! stopped helper's last report, or one from before a PipeWire restart,
//! never grants (lane B6 on the M1 Pro: one 10 ms grant on a stopped
//! helper's report across a PipeWire restart).
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
//!    because the grant is live), and the driver moves from the dummy driver
//!    to the sink. The graph reactivates across that switch (the page's
//!    frame count restarts) and the heartbeat pauses: lane B2 measured up to
//!    63 ms on the J616s.
//! 5. From then on the grant is renewed while the engine beats and the PCM
//!    is Closed, Open (OPEN, SETUP, PREPARED) or RUNNING, owned by the
//!    engine's process. XRUN, PAUSED, SUSPENDED or an unknown state releases
//!    at once, as does any other fault. The kernel stops the stream, the
//!    helper unlinks the amps, the protect stage falls back to the dummy
//!    driver and beats again; the sink keeps the PCM PREPARED and retries its
//!    start, and the sequence resumes at 3 from there.
//!
//! The heartbeat bound is two quanta plus a tick, plus [`START_GRACE_NS`]
//! where the driver switch happens and nothing can play:
//! - Closed: strict. Anyone could open and start the PCM while it is granted.
//! - Open, owned by the engine's process: + grace. The PCM is not running,
//!   only that process holds it and can start it, and it can do so only by
//!   running the graph, which beats.
//! - RUNNING: + grace for [`START_GRACE_NS`] after it is first seen running,
//!   strict from then on.
//! - For [`GRANT_GRACE_WINDOW_NS`] after each grant, in any of those states,
//!   the grace is [`GRANT_GRACE_NS`]. Each grant is followed by the helper's
//!   re-link and the driver switch, and lane B2 saw the heartbeat pause there
//!   more than once in the first 200 ms after a re-grant. The PCM state
//!   checks (XRUN, owner) are never relaxed, and they are what floors a
//!   stopped or killed engine (B2: 19.6-31.0 ms).
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
    /// The most restrictive gain of the active channels in the last record,
    /// as f32 bits. NaN when an active channel's gain is non-finite (a model
    /// the unit no longer trusts, v2-B/engine afcd163): a fault, never "no
    /// attenuation".
    pub min_gain_bits: u32,
}

impl EngineState {
    /// Whether the published gain is a gain: finite and not negative.
    pub fn gain_ok(&self) -> bool {
        let g = f32::from_bits(self.min_gain_bits);
        g.is_finite() && g >= 0.0
    }
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
    /// The writer, as the report names it: pid and start time (clock ticks
    /// since boot, `/proc/<pid>/stat` field 22).
    pub helper: Option<(u32, u64)>,
    /// Set by the view: that process is alive, is the one that started at
    /// that time, and is the session user's.
    pub helper_alive: bool,
}

pub trait SessionView {
    fn read(&mut self) -> SessionReport;
}

/// The largest graph quantum the heartbeat bound accepts (PipeWire's
/// default `clock.max-quantum`). A page claiming more is not fresh.
pub const MAX_QUANTUM: u64 = 2048;
/// How old the helper's report may be. It bounds only the links and carrier
/// checks, which guard against a foreign link in the graph. The writer's
/// liveness is checked separately and strictly (a dead or replaced helper
/// counts at once), and so are the engine heartbeat, the PCM state and its
/// owner, which protect the speakers. The helper writes on every link
/// change and every 50 ms; 500 ms rides out a helper starved by desktop
/// load (J616s, full-volume YouTube with a busy browser: a missed 50 ms
/// window went past the old 100 ms bound and dropped the audio).
pub const SESSION_STALE_NS: u64 = 500_000_000;
/// How far in the future a session report may be dated. The helper and
/// the supervisor read the same CLOCK_MONOTONIC and the supervisor reads its
/// clock after the file, so a report is never ahead: 1 ms is slack only.
pub const SESSION_SKEW_NS: u64 = 1_000_000;
/// Extra heartbeat age tolerated while PipeWire moves the protect stage from
/// the dummy driver to the amps' sink: while the PCM is open but not
/// running, and for this long after it starts running. Steady state (RUNNING
/// after that, or Closed) has none.
pub const START_GRACE_NS: u64 = 50_000_000;
/// How long after a grant [`GRANT_GRACE_NS`] applies.
pub const GRANT_GRACE_WINDOW_NS: u64 = 250_000_000;
/// Extra heartbeat age tolerated just after a grant (not added to
/// [`START_GRACE_NS`]: the larger of the two applies).
pub const GRANT_GRACE_NS: u64 = 100_000_000;

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
    last_seq_before: Option<u64>,
    /// The engine's process and validity, to see a new generation.
    engine_pid: Option<(u32, bool)>,
    /// When the current engine generation was first seen.
    generation_ns: Option<u64>,
    last_frames: u64,
    /// When the sequence was last seen to advance.
    last_advance_ns: Option<u64>,
    /// The graph quantum, from frames over beats between two advances.
    quantum: Option<u64>,
    was_running: bool,
    grace_until: Option<u64>,
    granted_at: Option<u64>,
}

impl Observer {
    pub fn new(expected_params_crc: u32, rate: u32, tick_ns: u64) -> Self {
        Observer {
            expected_params_crc,
            rate,
            tick_ns,
            last_seq: None,
            last_seq_before: None,
            engine_pid: None,
            generation_ns: None,
            last_frames: 0,
            last_advance_ns: None,
            quantum: None,
            was_running: false,
            grace_until: None,
            granted_at: None,
        }
    }

    /// The supervisor granted at `now_ns`: the re-link and driver switch
    /// that follow get [`GRANT_GRACE_NS`].
    pub fn note_grant(&mut self, now_ns: u64) {
        self.granted_at = Some(now_ns);
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
            && e.health & sgv2_core::Health::NO_PARAMS == 0
            && e.gain_ok();

        self.last_seq_before = self.last_seq;
        match self.last_seq {
            Some(l) if e.seq > l => {
                let beats = e.seq - l;
                // a reactivated graph restarts the frame count: keep the
                // quantum learnt so far and learn again from the next beat
                if let Some(q) = e.frames.checked_sub(self.last_frames).filter(|f| *f > 0).map(|f| f / beats) {
                    self.quantum = Some(q);
                }
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
        let running = matches!(p, PcmState::Running { .. });
        if running && !self.was_running {
            self.grace_until = Some(now_ns + START_GRACE_NS);
        }
        if !running {
            self.grace_until = None;
        }
        self.was_running = running;
        let start_grace = match p {
            PcmState::Open { .. } => START_GRACE_NS,
            PcmState::Running { .. } if self.grace_until.is_some_and(|t| now_ns < t) => START_GRACE_NS,
            _ => 0,
        };
        let grant_grace = match self.granted_at {
            Some(t) if now_ns.saturating_sub(t) < GRANT_GRACE_WINDOW_NS => GRANT_GRACE_NS,
            _ => {
                self.granted_at = None;
                0
            }
        };
        let grace = start_grace.max(grant_grace);

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
        let pcm_state_ok = matches!(p, PcmState::Closed | PcmState::Open { .. } | PcmState::Running { .. });
        // A new engine generation: another process, a page that became
        // valid, or a sequence that restarted.
        let restarted = self.last_seq_before.is_some_and(|l| e.seq < l);
        if self.engine_pid != Some((e.pid, e.valid)) || restarted || self.generation_ns.is_none() {
            if self.engine_pid != Some((e.pid, e.valid)) || restarted {
                self.generation_ns = Some(now_ns);
            }
            self.generation_ns.get_or_insert(now_ns);
            self.engine_pid = Some((e.pid, e.valid));
        }
        let generation = self.generation_ns.unwrap_or(now_ns);
        let session_fresh = s.present
            && s.helper_alive
            && s.mono_ns >= generation
            && s.mono_ns <= now_ns.saturating_add(SESSION_SKEW_NS)
            && now_ns.saturating_sub(s.mono_ns) <= SESSION_STALE_NS;
        Observation {
            heartbeat_fresh,
            plugin_ok,
            links_ok: session_fresh && s.links_ok,
            carrier_ok: session_fresh && s.carrier_ok,
            pcm_owner_ok,
            pcm_state_ok,
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

/// Parse the helper's report,
/// `links=<0|1> carrier=<0|1> mono_ns=<t> pid=<pid> start=<ticks>\n`.
/// Anything else, including a report without its final newline or without
/// its writer, is absent.
///
/// The view then sets `helper_alive` with [`helper_is`].
pub fn parse_session(text: &str) -> SessionReport {
    let Some(body) = text.strip_suffix('\n') else { return SessionReport::default() };
    let mut links = None;
    let mut carrier = None;
    let mut mono = None;
    let mut pid = None;
    let mut start = None;
    for w in body.split_whitespace() {
        let Some((k, v)) = w.split_once('=') else { return SessionReport::default() };
        let slot = match k {
            "links" => &mut links,
            "carrier" => &mut carrier,
            "mono_ns" => &mut mono,
            "pid" => &mut pid,
            "start" => &mut start,
            _ => continue,
        };
        if slot.is_some() {
            return SessionReport::default();
        }
        *slot = Some(v.parse::<u64>().ok());
    }
    match (links.flatten(), carrier.flatten(), mono.flatten()) {
        (Some(l @ 0..=1), Some(c @ 0..=1), Some(m)) if m > 0 => match (pid.flatten(), start.flatten()) {
            (Some(p), Some(st)) if p > 0 && p <= u64::from(u32::MAX) => SessionReport {
                present: true,
                links_ok: l == 1,
                carrier_ok: c == 1,
                mono_ns: m,
                helper: Some((p as u32, st)),
                helper_alive: false,
            },
            _ => SessionReport::default(),
        },
        _ => SessionReport::default(),
    }
}

/// The start time (clock ticks since boot) in `/proc/<pid>/stat`: field 22,
/// counted after the command name's closing parenthesis (the name may hold
/// spaces and parentheses).
pub fn parse_stat_start(stat: &str) -> Option<u64> {
    let rest = &stat[stat.rfind(')')? + 1..];
    // rest starts at field 3 (state)
    let mut f = rest.split_whitespace();
    let state = f.next()?;
    if state == "Z" || state == "X" {
        return None;
    }
    f.nth(18)?.parse().ok()
}

/// Whether the report's writer is alive: `stat` is `/proc/<pid>/stat` (if
/// readable), `owner` that directory's uid, and `uid` the session user.
pub fn helper_is(report: &SessionReport, stat: Option<&str>, owner: Option<u32>, uid: u32) -> bool {
    match (report.helper, stat, owner) {
        (Some((_, start)), Some(st), Some(o)) => o == uid && parse_stat_start(st) == Some(start),
        _ => false,
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
        SessionReport {
            present: true,
            links_ok: true,
            carrier_ok: true,
            mono_ns: at,
            helper: Some((77, 1234)),
            helper_alive: true,
        }
    }

    fn beat(seq: u64, quantum: u64, mono_ns: u64) -> EngineState {
        EngineState {
            valid: true,
            pid: PID,
            params_crc: 7,
            seq,
            frames: seq * quantum,
            mono_ns,
            health: 0,
            min_gain_bits: 1.0f32.to_bits(),
        }
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
        // the engine generation began well before these reports
        o.observe(&mut || 1, &mut E(EngineState::default()), &mut P(PcmState::Closed), &mut S(session(1)));
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
        let w = " pid=77 start=1234";
        let r = parse_session(&format!("links=1 carrier=1 mono_ns=123{w}\n"));
        assert_eq!(
            r,
            SessionReport {
                present: true,
                links_ok: true,
                carrier_ok: true,
                mono_ns: 123,
                helper: Some((77, 1234)),
                helper_alive: false
            }
        );
        let r = parse_session(&format!("links=0 carrier=1 mono_ns=123{w}\n"));
        assert!(r.present && !r.links_ok);
        for bad in [
            String::new(),
            format!("links=1 carrier=1 mono_ns=123{w}"),
            format!("links=1 carrier=1 mono_ns={w}\n"),
            format!("links=1 carrier=1{w}\n"),
            format!("links=1 carrier=1 mono_ns=12 mono_ns=13{w}\n"),
            format!("links=2 carrier=1 mono_ns=12{w}\n"),
            format!("links=1 carrier=1 mono_ns=0{w}\n"),
            format!("links=1 carrier=1 mono_ns=12 junk{w}\n"),
            format!("links=1 carrier=1 mono_ns=12{w}\nlinks=1 carrier=1 mono_ns=13{w}\n"),
            // no writer: an older helper's report
            "links=1 carrier=1 mono_ns=12\n".to_string(),
            "links=1 carrier=1 mono_ns=12 pid=77\n".to_string(),
            "links=1 carrier=1 mono_ns=12 pid=0 start=5\n".to_string(),
        ] {
            assert!(!parse_session(&bad).present, "{bad:?}");
        }
    }

    #[test]
    fn the_helper_must_be_the_live_process_that_wrote_the_report() {
        let r = parse_session("links=1 carrier=1 mono_ns=12 pid=4321 start=98765\n");
        // /proc/<pid>/stat: the name may hold spaces and parentheses
        let stat = |state: &str, start: u64| {
            let mut f = vec![state.to_string()];
            f.extend((4..=21).map(|n| n.to_string()));
            f.push(start.to_string());
            f.extend(["1000", "2000"].map(String::from));
            format!("4321 (python3 -I (sgs)) {}\n", f.join(" "))
        };
        assert_eq!(parse_stat_start(&stat("S", 98765)), Some(98765));
        assert!(helper_is(&r, Some(&stat("S", 98765)), Some(1000), 1000));
        // another process now holds the pid (it started later)
        assert!(!helper_is(&r, Some(&stat("S", 99999)), Some(1000), 1000));
        // a zombie or a dead pid
        assert!(!helper_is(&r, Some(&stat("Z", 98765)), Some(1000), 1000));
        assert!(!helper_is(&r, None, None, 1000));
        // another user's process
        assert!(!helper_is(&r, Some(&stat("S", 98765)), Some(1001), 1000));
        assert!(!helper_is(&SessionReport::default(), Some(&stat("S", 98765)), Some(1000), 1000));
    }

    /// Lane B6 on the M1 Pro: the helper is stopped, PipeWire restarts, and
    /// the supervisor granted once (10 ms) on the stopped helper's last
    /// report. Neither the dead helper's report nor one written before the
    /// new engine generation may count.
    #[test]
    fn a_stopped_helpers_report_never_grants_across_a_pipewire_restart() {
        use crate::kernel::Recorder;
        use crate::supervise::{Event, Expect, Supervisor};
        let q = 10_666_666u64;
        let mut o = Observer::new(7, 48_000, TICK);
        let mut k = Recorder::healthy(6);
        let mut sup =
            Supervisor::new(Expect::new(crate::machine::lookup("j316s").unwrap(), vec![11_890; 6], "Secondary"));
        let tick = |o: &mut Observer,
                    sup: &mut Supervisor,
                    k: &mut Recorder,
                    now: u64,
                    pid: u32,
                    seq: u64,
                    r: SessionReport| {
            let e = EngineState { pid, ..beat(seq, 512, now - 1_000) };
            let obs = o.observe(&mut || now, &mut E(e), &mut P(PcmState::Closed), &mut S(r));
            sup.tick(obs, k).unwrap()
        };
        // granted on engine A with a live helper
        let mut seq = 0;
        let mut now = 0;
        let mut granted = false;
        for _ in 0..10 {
            seq += 1;
            now += q;
            granted |= tick(&mut o, &mut sup, &mut k, now, PID, seq, session(now - 1)) == Event::Granted;
        }
        assert!(granted && k.held);
        // the helper is stopped: its last report stays on disk, fresh by
        // its stamp, but its process is gone
        let last = SessionReport { helper_alive: false, ..session(now) };
        seq += 1;
        now += q;
        assert!(matches!(tick(&mut o, &mut sup, &mut k, now, PID, seq, last), Event::Released { .. }));
        // PipeWire restarts: engine B (another pid) beats within the old
        // report's 100 ms; whether or not the dead helper's pid passes the
        // liveness check (pid reuse), nothing grants
        let restart = now + 5_000_000;
        let stale_but_alive = SessionReport { helper_alive: true, ..session(now) };
        for (n, r) in [last, stale_but_alive].into_iter().enumerate() {
            let mut o = Observer::new(7, 48_000, TICK);
            let mut t = restart;
            let mut s2 = 0;
            for _ in 0..8 {
                s2 += 1;
                t += q;
                if t - now > SESSION_STALE_NS {
                    break;
                }
                let ev = tick(&mut o, &mut sup, &mut k, t, PID + 1, s2, r);
                assert_ne!(ev, Event::Granted, "case {n} at +{} ms", (t - now) / 1_000_000);
                assert!(!k.held);
            }
        }
        // a new helper writes after engine B started: grants again
        let mut t = restart + 10 * q;
        let mut s2 = 100;
        let mut granted = false;
        for _ in 0..4 {
            s2 += 1;
            t += q;
            granted |= tick(
                &mut o,
                &mut sup,
                &mut k,
                t,
                PID + 1,
                s2,
                SessionReport { helper: Some((88, 5)), ..session(t - 1) },
            ) == Event::Granted;
        }
        assert!(granted);
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

    /// Lane B2's J616s cold start (f9d2b08, r3): the protect stage beats at
    /// 1024 on the dummy driver; after the grant the sink opens and prepares
    /// the PCM, the heartbeat pauses 63 ms across the driver switch, the
    /// graph reactivates (frames restart at 768), and it runs at 512.
    #[test]
    fn the_j616s_cold_start_stays_fresh_and_a_prepared_pcm_can_be_granted() {
        let q1024 = 21_333_333u64;
        let q512 = 10_666_666u64;
        let mut o = Observer::new(7, 48_000, TICK);
        let obs_at = |o: &mut Observer, now: u64, seq: u64, frames: u64, mono: u64, pcm: PcmState| {
            let e = EngineState { frames, ..beat(seq, 0, mono) };
            o.observe(&mut || now, &mut E(e), &mut P(pcm), &mut S(session(now)))
        };
        let ok = |obs: Observation| obs.heartbeat_fresh && obs.pcm_state_ok && obs.pcm_owner_ok;
        // idle on the dummy driver, 1024 frames a beat
        let (mut seq, mut frames, mut mono) = (0u64, 0u64, 0u64);
        for k in 0..20u64 {
            let now = k * TICK;
            let beats = now / q1024;
            if beats > seq {
                frames += (beats - seq) * 1024;
                seq = beats;
                mono = beats * q1024;
            }
            let obs = obs_at(&mut o, now, seq, frames, mono, PcmState::Closed);
            assert!(ok(obs) || k < 3, "idle tick {k}");
        }
        // granted at 200 ms; the sink prepares, and the engine pauses 63 ms
        // after its last beat
        let open = PcmState::Open { owner: PID };
        let pause_end = mono + 63_000_000;
        let mut now = 200_000_000;
        while now < pause_end {
            assert!(ok(obs_at(&mut o, now, seq, frames, mono, open)), "prepared, paused, at {now}");
            now += TICK;
        }
        // the graph reactivates: frames restart at 768, then 512 a beat, and
        // the sink starts
        seq += 1;
        frames = 768;
        mono = pause_end;
        assert!(ok(obs_at(&mut o, now, seq, frames, mono, open)), "reactivated");
        let run = PcmState::Running { owner: PID };
        for k in 0..30u64 {
            let t = now + 1_000 + k * q512;
            seq += 1;
            frames += 512;
            mono = t - 1_000;
            assert!(ok(obs_at(&mut o, t, seq, frames, mono, run)), "running beat {k}");
            now = t;
        }
        assert_eq!(o.bound_ns(), Some(stale_ns(512, 48_000, TICK)));
        // steady state is strict: a stall past 2 quanta + a tick is stale
        assert!(
            !obs_at(&mut o, mono + stale_ns(512, 48_000, TICK) + 1_000_000, seq, frames, mono, run).heartbeat_fresh
        );
        // an XRUN is a fault at once, even with a fresh heartbeat
        seq += 1;
        frames += 512;
        mono += q512;
        let obs = obs_at(&mut o, mono + 1_000, seq, frames, mono, PcmState::Stopped { owner: PID });
        assert!(obs.heartbeat_fresh && !obs.pcm_state_ok);
        // after the release the sink re-prepares and retries; the engine
        // beats again on the dummy driver: a prepared PCM owned by the engine
        // can be granted again (B2 r3 waited 5.2 s for a close)
        let t0 = mono;
        for k in 1..6u64 {
            seq += 1;
            frames += 1024;
            mono = t0 + k * q1024;
            assert!(ok(obs_at(&mut o, mono + 1_000, seq, frames, mono, open)), "re-prepared {k}");
        }
        // but not owned by anyone else, and not with a dead engine
        assert!(!obs_at(&mut o, mono + 2_000, seq, frames, mono, PcmState::Open { owner: 1 }).pcm_owner_ok);
        let bound = stale_ns(1024, 48_000, TICK) + START_GRACE_NS;
        assert!(!obs_at(&mut o, mono + bound + 1_000_000, seq, frames, mono, open).heartbeat_fresh);
    }

    #[test]
    fn closed_is_strict_and_unknown_or_stopped_is_never_ok() {
        let mut o = Observer::new(7, 48_000, TICK);
        let q = 10_666_666u64;
        for k in 0..5u64 {
            let seq = k * TICK / q;
            o.observe(
                &mut || k * TICK,
                &mut E(beat(seq, 512, seq * q)),
                &mut P(PcmState::Closed),
                &mut S(session(k * TICK)),
            );
        }
        let seq = 40_000_000 / q;
        let last = seq * q;
        let strict = stale_ns(512, 48_000, TICK);
        let at = |o: &mut Observer, now: u64, pcm: PcmState| {
            o.observe(&mut || now, &mut E(beat(seq, 512, last)), &mut P(pcm), &mut S(session(now)))
        };
        assert!(!at(&mut o, last + strict + 1_000_000, PcmState::Closed).heartbeat_fresh);
        assert!(!at(&mut o, last + 1_000_000, PcmState::Unknown).pcm_state_ok);
        assert!(!at(&mut o, last + 1_000_000, PcmState::Stopped { owner: PID }).pcm_state_ok);
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

    #[test]
    fn a_regrant_tolerates_the_relink_pauses_and_then_is_strict() {
        // B2 0d7ee6c: after a re-grant the heartbeat paused twice in the
        // first 200 ms (the re-link and the driver switch)
        let q = 10_666_666u64;
        let mut o = Observer::new(7, 48_000, TICK);
        let open = PcmState::Open { owner: PID };
        let run = PcmState::Running { owner: PID };
        let mut seq = 0u64;
        let mut mono = 0u64;
        let mut at = |o: &mut Observer, now: u64, beat_now: bool, pcm: PcmState| {
            if beat_now {
                seq += 1;
                mono = now - 1_000;
            }
            o.observe(&mut || now, &mut E(beat(seq, 512, mono)), &mut P(pcm), &mut S(session(now)))
        };
        for k in 1..10u64 {
            at(&mut o, k * q, true, open);
        }
        let grant = 10 * q;
        o.note_grant(grant);
        // a 90 ms pause while prepared, then a 90 ms pause just after it runs
        let mut now = grant;
        while now < grant + 90_000_000 {
            assert!(at(&mut o, now, false, open).heartbeat_fresh, "first pause at {}", now - grant);
            now += TICK;
        }
        at(&mut o, now, true, run);
        let resume = now;
        while now < resume + 90_000_000 {
            assert!(at(&mut o, now, false, run).heartbeat_fresh, "second pause at {}", now - grant);
            now += TICK;
        }
        // after the window: strict again
        let t = grant + GRANT_GRACE_WINDOW_NS + q;
        assert!(at(&mut o, t, true, run).heartbeat_fresh);
        let strict = stale_ns(512, 48_000, TICK);
        assert!(at(&mut o, t + strict - 1_000_000, false, run).heartbeat_fresh);
        assert!(!at(&mut o, t + strict + 1_000_000, false, run).heartbeat_fresh);
        // and the grace never covers an XRUN
        o.note_grant(t);
        assert!(!at(&mut o, t + 1_000_000, true, PcmState::Stopped { owner: PID }).pcm_state_ok);
    }

    #[test]
    fn a_nan_or_non_finite_gain_is_a_fault_and_releases() {
        // v2-B/engine afcd163: an active channel whose gain is non-finite is
        // published as NaN; it must never read as "no attenuation"
        let mut o = Observer::new(7, 48_000, TICK);
        let at = |o: &mut Observer, k: u64, g: f32| {
            let now = k * 10_666_666;
            let e = EngineState { min_gain_bits: g.to_bits(), ..beat(k, 512, now) };
            o.observe(&mut || now + 1_000, &mut E(e), &mut P(PcmState::Running { owner: PID }), &mut S(session(now)))
        };
        at(&mut o, 1, 1.0);
        assert!(at(&mut o, 2, 1.0).plugin_ok);
        assert!(at(&mut o, 3, 0.25).plugin_ok);
        assert!(at(&mut o, 4, 0.0).plugin_ok);
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -0.5, f32::from_bits(0x7fc0_0001)] {
            let obs = at(&mut o, 5, bad);
            assert!(!obs.plugin_ok && !obs.clean(), "{bad}");
        }
        // through the lease: a granted supervisor releases at once
        use crate::kernel::Recorder;
        use crate::lease::Observation;
        use crate::supervise::{Event, Expect, Supervisor};
        let mut k = Recorder::healthy(6);
        let mut s =
            Supervisor::new(Expect::new(crate::machine::lookup("j316s").unwrap(), vec![11_890; 6], "Secondary"));
        let clean = Observation {
            heartbeat_fresh: true,
            plugin_ok: true,
            links_ok: true,
            carrier_ok: true,
            pcm_owner_ok: true,
            pcm_state_ok: true,
            route_ok: false,
            amps_ok: false,
        };
        s.tick(clean, &mut k).unwrap();
        assert_eq!(s.tick(clean, &mut k), Ok(Event::Granted));
        let nan = at(&mut o, 6, f32::NAN);
        assert!(matches!(s.tick(nan, &mut k), Ok(Event::Released { .. })));
        assert!(!k.held);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn the_start_time_parses_from_a_real_proc_stat() {
        let stat = std::fs::read_to_string("/proc/self/stat").unwrap();
        let start = parse_stat_start(&stat).unwrap();
        let r = parse_session(&format!("links=1 carrier=1 mono_ns=1 pid={} start={start}\n", std::process::id()));
        // SAFETY: getuid has no preconditions.
        let uid = unsafe { libc::getuid() };
        assert!(helper_is(&r, Some(&stat), Some(uid), uid));
        assert!(!helper_is(&r, Some(&stat), Some(uid), uid + 1));
    }

    /// J616s under desktop load: the helper (alive) misses its window and
    /// its report is 150 ms old while the engine is healthy and the PCM runs.
    /// No release; a dead helper still releases on the next tick.
    #[test]
    fn a_150_ms_helper_stall_with_a_healthy_engine_does_not_release() {
        use crate::kernel::Recorder;
        use crate::supervise::{Event, Expect, Supervisor};
        let q = 10_666_666u64;
        let mut o = Observer::new(7, 48_000, TICK);
        let mut k = Recorder::healthy(6);
        let mut sup =
            Supervisor::new(Expect::new(crate::machine::lookup("j316s").unwrap(), vec![11_890; 6], "Secondary"));
        let run = PcmState::Running { owner: PID };
        let mut seq = 0;
        let mut now = 1_000_000_000;
        let mut report = session(now);
        let step = |o: &mut Observer, sup: &mut Supervisor, k: &mut Recorder, now: u64, seq: u64, r| {
            let e = beat(seq, 512, now - 1_000);
            sup.tick(o.observe(&mut || now, &mut E(e), &mut P(run), &mut S(r)), k).unwrap()
        };
        // steady: the helper reports every 50 ms
        for i in 0..30u64 {
            seq += 1;
            now += q;
            if i % 5 == 0 {
                report = session(now);
            }
            step(&mut o, &mut sup, &mut k, now, seq, report);
        }
        assert!(k.held);
        let releases = k.releases;
        // the helper stalls for 150 ms (its last report ages to 150+ ms)
        let stall_end = now + 150_000_000;
        while now < stall_end {
            seq += 1;
            now += q;
            let ev = step(&mut o, &mut sup, &mut k, now, seq, report);
            assert!(matches!(ev, Event::Renewed), "{ev:?} at report age {} ms", (now - report.mono_ns) / 1_000_000);
        }
        assert_eq!(k.releases, releases);
        // it is still bounded: a report past the bound releases
        let old = SessionReport { mono_ns: now - SESSION_STALE_NS - 1, ..report };
        seq += 1;
        now += q;
        assert!(matches!(step(&mut o, &mut sup, &mut k, now, seq, old), Event::Released { .. }));
        // and a dead helper releases at once, whatever its report's age
        let mut o = Observer::new(7, 48_000, TICK);
        let mut sup =
            Supervisor::new(Expect::new(crate::machine::lookup("j316s").unwrap(), vec![11_890; 6], "Secondary"));
        let mut k = Recorder::healthy(6);
        let mut t = 0;
        for s2 in 1..6u64 {
            t += q;
            step(&mut o, &mut sup, &mut k, t, s2, session(t));
        }
        assert!(k.held);
        t += q;
        let dead = SessionReport { helper_alive: false, ..session(t) };
        assert!(matches!(step(&mut o, &mut sup, &mut k, t, 6, dead), Event::Released { .. }));
    }
}

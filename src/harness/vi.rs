// SPDX-License-Identifier: MIT
//! The harness's V/I world: the J316 and the J413, whose amplifiers sense
//! voltage and current (S16 words) and have a volume control each, with
//! the asahi governor writing the card, the unified models in shadow or
//! (with the command line's opt-in) governing, against an oracle of its
//! own, or the unified governor writing it under the asahi guard.  The
//! card, its lease and its events, the capture, the clock, the faults and
//! the restarts behave as in the J700 world (harness.rs), with these
//! differences.
//!
//! * The program: each stream has a level (as there) and a share of it
//!   for each speaker (a crossover's), played as a sine of the stream's
//!   frequency or as a square at the Nyquist rate.
//! * The load: each speaker's current is its voltage over an impedance of
//!   random magnitude and phase (up to 60 degrees either way), so V*I is
//!   negative for part of every cycle of a sine.  The magnitude is never
//!   under the speaker's smallest (`z_nominal` or `r_ff`).  Where it draws
//!   more than the sense's full scale, the current word clips at its rail
//!   while the speaker gets the whole current.
//! * The sense words: V over `vs_scale` and I over `is_scale`, rounded to
//!   S16, with up to `noise` LSB either way on every word and a fixed
//!   offset of up to `dc` LSB per channel (the M1 Pro's sense lines carry
//!   7 to 9 LSB of noise and offsets of up to 10).  Bad sense data: every
//!   period read inside a `BadData` fault says half scale of V and of
//!   reversed I, a mean V*I far below speakersafetyd's -0.01 W bound.
//!   Late blocks: a period can reach the daemon well after it ended, held
//!   in the capture.  A dead current line: inside an `IDead` fault every
//!   current word carries only its noise and offset, while the speaker
//!   gets the whole current.
//! * The gain: the amplifier's own volume, applied as each sample plays,
//!   which is how these cards apply it; or the J700's copy queue, applied
//!   as each chunk is converted and stopped when the lock goes (harsher).
//! * A quiet tail: the speakers stay open with silence for as long as the
//!   daemon's own models need to cool, with no power, past the point where
//!   the governor may take the lock back.
//!
//! The oracle integrates every sample's true power (before rounding and
//! noise) into three two-node models per speaker, each its own RK4: the
//! measured-power constants (`vi_*`) on V*I, and the feed-forward ones on
//! V^2 over `z_nominal` (the legacy model's) and over `r_ff` (unified's).
//! After every pass of the loop it asserts:
//!
//! * (a) each model the daemon runs is never cooler than the oracle's
//!   model with its constants, coil and magnet, at the time accounted (to
//!   a sample: frames are whole samples);
//! * (b) while the daemon holds the lease and the asahi governor writes
//!   the card, or the unified one under the asahi guard, each speaker's
//!   card reduction is at least what speakersafetyd's ramp asks for on the
//!   oracle's true coil and magnet, linked by group, and that need is
//!   under --max-reduction: never less protective than speakersafetyd
//!   alone; whatever governs, the lease is not held with a true coil or
//!   magnet at its hard limit, the measured-power truth's and, when the
//!   unified governor writes the card, the feed-forward truth's (the one
//!   unified's model is checked against).  After a dead current line the
//!   asahi models and unified's measured-power model, which then bound
//!   the line's power by V^2 over the speaker's smallest impedance, are
//!   checked against the measured-power truth less what that bound can
//!   miss (`dead_slack`: under the floors where the line's noise passes
//!   for a reading, at a dead stretch's two ends, and inside a cycle);
//! * (c) as in harness.rs (no lock without events, none after a failed
//!   rate read in that pass, no renewal blind past the limit, a start from
//!   anything but a clean stop latched), no lock until REARM_PERIODS good
//!   periods have followed bad sense data, and none while the governing
//!   model is latched;
//! * (d) by the end of the quiet tail the asahi latch has cleared and the
//!   lock has been taken back.
//!
//! The random cases run 1 to 4 s.  A long class (review r4 BB2 of
//! speakerguardd#8) runs steady programs for minutes, long enough to heat
//! the feed-forward truth, under the asahi governor alone and the guard,
//! with the current lines live and dead, and asserts besides that the
//! measured-power truth never reaches its limit (and under the guard, the
//! feed-forward truth).  It runs six programs of 30 s by default and all
//! 120, of 120 s each, with `SPEAKERGUARDD_VI_LONG` set.
//!
//! `SPEAKERGUARDD_VI_CASES` sets the number of random cases (default 60)
//! and `SPEAKERGUARDD_VI_SEED` the first seed.  A failure is shrunk and
//! printed as a scenario to paste into a witness test.

use std::cell::Cell;
use std::f64::consts::PI;
use std::io;
use std::time::{Duration, Instant};

use super::{damage, state_dir, Damage, Fault, Restart, Rng, Violation};
use crate::config::{Config, Governor};
use crate::engine::{dead_block, Engine, DEAD_LONG_S, DEAD_RATIO, DEAD_RATIO_SHORT, DEAD_SHORT_S, DEAD_V_MIN};
use crate::guard::{Guard, Hw, BLIND_LIMIT, DRAIN, MUTE_DB, REARM_MARGIN_DB, REARM_PERIODS};
use crate::model::SpeakerState;
use crate::state;

mod recurrent;

const J316: &str = include_str!("../../conf/apple/j316.conf");
const J413: &str = include_str!("../../conf/apple/j413.conf");
/// The longest quiet tail run (s); a tail its bound puts beyond this is
/// run but not asserted.
const TAIL_MAX: f64 = 400.0;
/// What the tail allows past the cooling bound: the drain, the periods to
/// take the lock back, the idle poll, the save interval.
const TAIL_SLACK: f64 = 3.0;

#[derive(Clone, Debug, PartialEq)]
pub struct ViStream {
    pub open: f64,
    pub close: f64,
    /// How far behind conversion the stream plays (s, copy queue only).
    pub depth: f64,
    /// Copy chunk (s, copy queue only).
    pub chunk: f64,
    /// (from, level as a fraction of full scale), in play time.
    pub levels: Vec<(f64, f64)>,
    /// Each speaker's share of the level.
    pub share: Vec<f64>,
    /// Sine frequency (Hz); 0 is a square at the Nyquist rate.
    pub freq: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ViFault {
    /// A fault of the J700 world.
    Base(Fault),
    /// Every period read in [at, at + len) is bad sense data.
    BadData { at: f64, len: f64 },
    /// One period reaches the daemon `delay` after it ended.
    LateBlock { at: f64, delay: f64 },
    /// Every current word of the samples played in [at, at + len) reads
    /// only its noise and offset.
    IDead { at: f64, len: f64 },
    /// ... reads `k` of the current plus `offset` amps (a line that reads
    /// low but plausibly, or a dead one carrying an offset: review r4 DD1).
    ILow { at: f64, len: f64, k: f64, offset: f64 },
}

#[derive(Clone, Debug, PartialEq)]
pub struct ViScenario {
    pub j413: bool,
    pub rate: u32,
    /// The gain is the amplifier's own, applied as the samples play (else
    /// a copy queue's, applied as they are converted).
    pub amp_gain: bool,
    /// The unified models run in shadow.
    pub shadow: bool,
    /// The unified governor writes the card (--experimental-unified).
    pub unified: bool,
    /// The unified governor writes the card under the asahi guard.
    pub guard: bool,
    pub noise: i32,
    pub dc: i32,
    pub seed: u64,
    /// Per speaker: the impedance as a multiple of its least allowed, and
    /// its phase (rad).
    pub loads: Vec<(f64, f64)>,
    pub horizon: f64,
    pub tail: bool,
    pub events: bool,
    pub steps: bool,
    pub initial_db: f64,
    pub clean_start: bool,
    pub max_reduction: f64,
    pub latency: f64,
    pub streams: Vec<ViStream>,
    pub faults: Vec<ViFault>,
    pub restarts: Vec<Restart>,
}

impl ViScenario {
    pub fn random(seed: u64) -> ViScenario {
        let mut r = Rng::new(seed ^ 0x7669_2d77_6f72_6c64);
        let j413 = r.chance(0.5);
        let n = if j413 { 4 } else { 6 };
        let rate = if r.chance(0.5) { 8000 } else { 16000 };
        let horizon = r.range(1.0, 4.0);
        let tail = r.chance(0.35);
        let mut streams = Vec::new();
        let mut t = r.range(0.0, 0.3);
        while t < horizon {
            let len = if r.chance(0.3) { r.range(0.002, 0.04) } else { r.range(0.05, 1.5) };
            let mut levels = vec![(t, if r.chance(0.3) { 1.0 } else { r.f() })];
            let mut u = t;
            while r.chance(0.5) {
                u += r.range(0.005, len.max(0.01));
                if u >= t + len {
                    break;
                }
                levels.push((u, if r.chance(0.3) { 1.0 } else { r.f() }));
            }
            let share = (0..n).map(|_| if r.chance(0.15) { 0.0 } else if r.chance(0.3) { 1.0 } else { r.f() }).collect();
            let freq = if r.chance(0.3) { 0.0 } else { 40.0 * (rate as f64 * 0.45 / 40.0).powf(r.f()) };
            let chunk = r.range(0.004, 0.128);
            streams.push(ViStream { open: t, close: t + len, depth: r.range(0.0, 0.341 - chunk), chunk, levels, share, freq });
            t += len + if r.chance(0.3) { r.range(0.0005, 0.03) } else { r.range(0.03, 0.8) };
        }
        let mut faults = Vec::new();
        for _ in 0..(r.f() * 6.0) as usize {
            let at = r.range(0.0, horizon);
            faults.push(match (r.f() * 14.0) as u32 {
                0 => ViFault::Base(Fault::RateFail { at, len: r.range(0.01, 0.6) }),
                1 => ViFault::Base(Fault::RateSlow { at, delay: r.range(0.01, 0.3) }),
                2 => ViFault::Base(Fault::WakeLate { at, len: r.range(0.05, 1.0), delay: r.range(0.005, 0.1) }),
                3 => ViFault::Base(Fault::Pause { at, len: r.range(0.05, 0.8) }),
                4 => ViFault::Base(Fault::WriteFail { at, len: r.range(0.01, 0.4), partial: r.chance(0.5) }),
                5 => ViFault::Base(Fault::MinFail { at, len: r.range(0.01, 0.4) }),
                6 => ViFault::Base(Fault::SenseLoss { at, len: r.range(0.02, 0.9) }),
                7 => ViFault::Base(Fault::ShortPeriod { at }),
                8 => ViFault::Base(Fault::PositionFail { at }),
                9 => ViFault::Base(Fault::ClockSplit { at, len: r.range(0.1, 1.0), pause: r.range(0.01, 1.0) }),
                10 => ViFault::Base(Fault::SlowOpen { at, delay: r.range(0.05, 0.2) }),
                11 => ViFault::Base(Fault::SlowPosition { at, delay: r.range(0.005, 0.3) }),
                12 => ViFault::BadData { at, len: r.range(0.01, 0.4) },
                _ => ViFault::LateBlock { at, delay: r.range(0.01, 0.4) },
            });
        }
        let mut restarts = Vec::new();
        if r.chance(0.4) {
            let damage = match (r.f() * 5.0) as u32 {
                0 => Damage::Truncate,
                1 => Damage::OldFormat,
                2 => Damage::Garbage,
                3 => Damage::Remove,
                _ => Damage::None,
            };
            restarts.push(Restart { at: r.range(0.2, horizon), clean: r.chance(0.5), gap: if r.chance(0.5) { r.range(0.0, 0.05) } else { r.range(0.05, 2.0) }, damage });
        }
        let loads = (0..n).map(|_| (r.range(1.0, 4.0), r.range(-PI / 3.0, PI / 3.0))).collect();
        let noise = [0, 1, 3, 9, 20][(r.f() * 5.0) as usize];
        let dc = [0, 3, 10][(r.f() * 3.0) as usize];
        let grid = |x: f64| (x * rate as f64).round() / rate as f64;
        for s in streams.iter_mut() {
            s.open = grid(s.open);
            s.close = grid(s.close).max(s.open + 1.0 / rate as f64);
            for l in s.levels.iter_mut() {
                l.0 = grid(l.0);
            }
        }
        let mut sc = ViScenario {
            j413,
            rate,
            amp_gain: r.chance(0.7),
            shadow: r.chance(0.5),
            unified: r.chance(0.15),
            noise,
            dc,
            seed: r.next(),
            loads,
            horizon,
            tail,
            events: !r.chance(0.08),
            steps: r.chance(0.7),
            initial_db: r.range(0.0, 30.0),
            clean_start: r.chance(0.8),
            max_reduction: if r.chance(0.5) { 7.0 } else { r.range(6.0, 30.0) },
            latency: r.range(0.0, 0.002),
            streams,
            faults,
            restarts,
            guard: false,
        };
        // drawn after everything else, so every other field of a seed is
        // what it was before the guard
        sc.guard = r.chance(0.35);
        if sc.guard && r.chance(0.4) {
            let at = r.range(0.0, horizon);
            sc.faults.push(ViFault::IDead { at, len: r.range(0.05, 2.0) });
        }
        // and after that: dead lines under the other governors (review r4
        // BB1 of speakerguardd#8), and the review's two onsets
        if !sc.guard && r.chance(0.15) {
            let at = r.range(0.0, horizon);
            sc.faults.push(ViFault::IDead { at, len: r.range(0.05, 2.0) });
        }
        match (r.f() * 12.0) as u32 {
            0 => sc.onset(&mut r, true),
            1 => sc.onset(&mut r, false),
            _ => {}
        }
        sc.add_tail();
        sc
    }

    /// A full-scale onset on every speaker for 1.5 s (review r4 of
    /// speakerguardd#8), at the top of a staircase from 0.2 to 0.5 of full
    /// scale in steps of 12 to 50 % every 0.1 to 0.5 s (its attacks land
    /// inside the half second before the next step), or inside a partial
    /// write failure that starts up to 0.1 s before it.
    fn onset(&mut self, r: &mut Rng, staircase: bool) {
        let n = self.loads.len();
        let rate = self.rate as f64;
        let grid = |x: f64| (x * rate).round() / rate;
        let t_on = grid(r.range(0.4, self.horizon.max(1.0) - 0.2));
        let t0 = if staircase { grid((t_on - r.range(0.6, 2.0)).max(0.05)) } else { t_on };
        self.streams.retain(|s| s.close <= t0 - 0.05 || s.open >= t_on + 2.0);
        let mut levels = Vec::new();
        if staircase {
            let (mut t, mut l, f) = (t0, r.range(0.2, 0.5), r.range(1.12, 1.5));
            while t < t_on - 0.01 && l < 1.0 {
                levels.push((grid(t), l));
                l *= f;
                t += r.range(0.1, 0.5);
            }
        } else {
            self.faults.push(ViFault::Base(Fault::WriteFail { at: t_on - r.range(0.0, 0.1), len: r.range(0.05, 0.4), partial: true }));
        }
        levels.push((t_on, 1.0));
        self.streams.push(ViStream { open: t0, close: grid(t_on + 1.5), depth: 0.0, chunk: 0.01, levels, share: vec![1.0; n], freq: 0.0 });
        self.streams.sort_by(|a, b| a.open.total_cmp(&b.open));
        self.horizon = self.horizon.max(t_on + 1.6);
    }

    /// A long steady program from 0.05 s, the card's own gain, no faults
    /// (review r4 BB2 of speakerguardd#8): `freq` on every speaker, or on
    /// the tweeters alone, at `level`, with the current lines dead
    /// throughout or live.
    pub fn long(j413: bool, freq: f64, tweeters: bool, level: f64, guard: bool, dead: bool, secs: f64) -> ViScenario {
        let n = if j413 { 4 } else { 6 };
        let mut sc = ViScenario {
            j413,
            rate: 16000,
            amp_gain: true,
            shadow: false,
            unified: false,
            guard,
            noise: 3,
            dc: 3,
            seed: 700000001,
            loads: vec![(1.2, 0.3); n],
            horizon: secs,
            tail: false,
            events: true,
            steps: true,
            initial_db: 0.0,
            clean_start: true,
            max_reduction: 7.0,
            latency: 0.0,
            streams: Vec::new(),
            faults: Vec::new(),
            restarts: Vec::new(),
        };
        let cfg = vi_cfg(&sc);
        let share = cfg.speakers.iter().map(|s| if tweeters && !s.name.contains("Tweeter") { 0.0 } else { 1.0 }).collect();
        sc.streams.push(ViStream { open: 0.05, close: secs + 1.0, depth: 0.0, chunk: 0.01, levels: vec![(0.05, level)], share, freq });
        if dead {
            sc.faults.push(ViFault::IDead { at: 0.0, len: secs + 10.0 });
        }
        sc
    }

    /// With a tail, the program stops at the horizon and the speakers stay
    /// open with silence.
    fn add_tail(&mut self) {
        if !self.tail {
            return;
        }
        let h = (self.horizon * self.rate as f64).round() / self.rate as f64;
        self.horizon = h;
        self.streams.retain(|s| s.open < h);
        for s in self.streams.iter_mut() {
            s.close = s.close.min(h);
            s.levels.retain(|l| l.0 < h);
        }
        let n = self.loads.len();
        self.streams.push(ViStream { open: h, close: h + TAIL_MAX + 10.0, depth: 0.0, chunk: 0.01, levels: vec![(h, 0.0)], share: vec![0.0; n], freq: 0.0 });
    }

    /// The scenario as Rust, to paste into a witness test.
    pub fn to_rust(&self) -> String {
        let f = |x: f64| format!("{x:?}");
        let fs = |v: &[f64]| v.iter().map(|&x| f(x)).collect::<Vec<_>>().join(", ");
        let streams: Vec<String> = self
            .streams
            .iter()
            .map(|s| {
                let levels: Vec<String> = s.levels.iter().map(|l| format!("({}, {})", f(l.0), f(l.1))).collect();
                format!("ViStream {{ open: {}, close: {}, depth: {}, chunk: {}, levels: vec![{}], share: vec![{}], freq: {} }}", f(s.open), f(s.close), f(s.depth), f(s.chunk), levels.join(", "), fs(&s.share), f(s.freq))
            })
            .collect();
        let faults: Vec<String> = self.faults.iter().map(|x| match x {
            ViFault::Base(b) => format!("ViFault::Base(Fault::{b:?})"),
            other => format!("ViFault::{other:?}"),
        }).collect();
        let restarts: Vec<String> = self.restarts.iter().map(|r| format!("Restart {{ at: {}, clean: {}, gap: {}, damage: Damage::{:?} }}", f(r.at), r.clean, f(r.gap), r.damage)).collect();
        let loads: Vec<String> = self.loads.iter().map(|l| format!("({}, {})", f(l.0), f(l.1))).collect();
        format!(
            "ViScenario {{ j413: {}, rate: {}, amp_gain: {}, shadow: {}, unified: {}, guard: {}, noise: {}, dc: {}, seed: {}, loads: vec![{}], horizon: {}, tail: {}, events: {}, steps: {}, initial_db: {}, clean_start: {}, max_reduction: {}, latency: {}, streams: vec![{}], faults: vec![{}], restarts: vec![{}] }}",
            self.j413, self.rate, self.amp_gain, self.shadow, self.unified, self.guard, self.noise, self.dc, self.seed, loads.join(", "), f(self.horizon), self.tail, self.events, self.steps,
            f(self.initial_db), self.clean_start, f(self.max_reduction), f(self.latency), streams.join(", "), faults.join(", "), restarts.join(", ")
        )
    }

    fn open_at(&self, t: f64) -> Option<&ViStream> {
        self.streams.iter().find(|s| s.open <= t && t < s.close)
    }

    fn edges(&self) -> Vec<f64> {
        let mut e: Vec<f64> = self.streams.iter().flat_map(|s| [s.open, s.close]).collect();
        e.sort_by(f64::total_cmp);
        e
    }

    fn base(&self) -> impl Iterator<Item = (usize, &Fault)> + '_ {
        self.faults.iter().enumerate().filter_map(|(i, f)| if let ViFault::Base(b) = f { Some((i, b)) } else { None })
    }
}

/// The configuration a scenario runs: the machine's own, at a shorter
/// period (the rules are per period or per second, not per frame count).
pub fn vi_cfg(sc: &ViScenario) -> Config {
    let text = if sc.j413 { J413 } else { J316 };
    let mut c = Config::parse(&text.replace("period = 4096", "period = 512")).unwrap();
    c.allow_unqualified = true;
    c.globals.shadow = sc.shadow;
    if sc.guard {
        c.globals.governor = Governor::Unified;
        c.globals.guard = Some(Governor::Asahi);
    } else if sc.unified {
        c.globals.governor = Governor::Unified;
        c.globals.guard = None;
        c.allow_unified = true;
    } else {
        // asahi alone (speakersafetyd's behaviour), whatever the file says
        c.globals.governor = Governor::Asahi;
        c.globals.guard = None;
    }
    c
}

/// One two-node model: coil and magnet, as deviations from ambient.
#[derive(Clone, Copy)]
struct Node {
    trc: f64,
    tau_c: f64,
    trm: f64,
    tau_m: f64,
}

impl Node {
    /// `h` seconds at `p` watts, by one classical Runge-Kutta step.
    fn step(&self, (c, m): (f64, f64), p: f64, h: f64) -> (f64, f64) {
        let f = |c: f64, m: f64| ((m + p * self.trc - c) / self.tau_c, (p * self.trm - m) / self.tau_m);
        let (a1, b1) = f(c, m);
        let (a2, b2) = f(c + a1 * h / 2.0, m + b1 * h / 2.0);
        let (a3, b3) = f(c + a2 * h / 2.0, m + b2 * h / 2.0);
        let (a4, b4) = f(c + a3 * h, m + b3 * h);
        (c + h / 6.0 * (a1 + 2.0 * a2 + 2.0 * a3 + a4), m + h / 6.0 * (b1 + 2.0 * b2 + 2.0 * b3 + b4))
    }
}

/// The oracle's three models of one speaker.
const VI: usize = 0;
const FFZ: usize = 1;
const FFR: usize = 2;

struct Oracle {
    ta: f64,
    nodes: Vec<[Node; 3]>,
    /// Per speaker and model: (coil, magnet) deviations now.
    now: Vec<[(f64, f64); 3]>,
    /// The samples integrated so far; `hist` keeps the state after each of
    /// the newest of them, from sample `base` on.
    n: u64,
    base: u64,
    hist: Vec<Vec<[f32; 6]>>,
    audit: Option<Vec<[super::continuous::Thermal;3]>>,
    /// Exact sample peaks, including unaccounted time and lost current lines.
    vi_peaks: [f64; 6],
}

impl Oracle {
    fn new(cfg: &Config) -> Oracle {
        let nodes: Vec<[Node;3]> = cfg
            .speakers
            .iter()
            .map(|s| {
                let ff = Node { trc: s.tr_coil, tau_c: s.tau_coil, trm: s.tr_magnet, tau_m: s.tau_magnet };
                [Node { trc: s.vi_tr_coil, tau_c: s.vi_tau_coil, trm: s.vi_tr_magnet, tau_m: s.vi_tau_magnet }, ff, ff]
            })
            .collect();
        let k = cfg.speakers.len();
        let audit=super::continuous::enabled().then(||nodes.iter().map(|ns|std::array::from_fn(|j|{let n=ns[j];super::continuous::Thermal::new(cfg.globals.t_ambient,n.trc,n.trm,n.tau_c,n.tau_m)})).collect());
        Oracle { ta: cfg.globals.t_ambient, nodes, now: vec![[(0.0, 0.0); 3]; k], n: 0, base: 0, hist: vec![Vec::new(); k], audit, vi_peaks: [0.0; 6] }
    }

    /// One sample of `h` seconds; per speaker the true V*I, V^2/z_nominal
    /// and V^2/r_ff.
    fn step(&mut self, h: f64, p: &[[f64; 3]]) {
        for k in 0..self.now.len() {
            let mut out = [0f32; 6];
            for j in 0..3 {
                if let Some(a)=self.audit.as_mut() {let(c,m)=self.now[k][j];a[k][j].observe(c+self.ta,m+self.ta,p[k][j],h);}
                self.now[k][j] = self.nodes[k][j].step(self.now[k][j], p[k][j], h);
                out[2 * j] = (self.now[k][j].0 + self.ta) as f32;
                out[2 * j + 1] = (self.now[k][j].1 + self.ta) as f32;
            }
            let vi = self.now[k][VI];
            self.vi_peaks[k] = self.vi_peaks[k].max(self.ta + vi.0.max(vi.1));
            self.hist[k].push(out);
        }
        self.n += 1;
        // keep the last 30 s or so (the account never lags that far)
        let keep = (30.0 / h) as usize;
        if self.hist[0].len() > 2 * keep {
            let drop = self.hist[0].len() - keep;
            for v in self.hist.iter_mut() {
                v.drain(..drop);
            }
            self.base += drop as u64;
        }
    }

    /// Model `j`'s (coil, magnet) of speaker `k` at time `t`: the cooler
    /// of the sample boundaries on either side of it.
    fn at(&self, k: usize, j: usize, t: f64, rate: f64) -> (f64, f64) {
        let pos = (t * rate).floor().max(0.0) as u64;
        let get = |i: u64| -> (f64, f64) {
            // the state after sample i is the state at (i + 1) / rate
            if i < self.base || self.hist[k].is_empty() {
                return (self.ta, self.ta);
            }
            let v = self.hist[k][((i - self.base) as usize).min(self.hist[k].len() - 1)];
            (v[2 * j] as f64, v[2 * j + 1] as f64)
        };
        let after = get(pos);
        let before = if pos == 0 { (self.ta, self.ta) } else { get(pos - 1) };
        (after.0.min(before.0), after.1.min(before.1))
    }
}

/// The card and the world around the daemon.
pub struct ViWorld<'a> {
    sc: &'a ViScenario,
    cfg: &'a Config,
    rate: f64,
    t0: Instant,
    boot0: f64,
    t: Cell<f64>,
    split_pending: Cell<f64>,
    pauses_used: Vec<bool>,
    once_used: Vec<bool>,
    controls: Vec<f64>,
    owner: bool,
    unlocked: bool,
    lease_left: f64,
    notice: bool,
    lease_t: f64,
    /// Per speaker: (from, effective reduction).
    gains: Vec<Vec<(f64, f64)>>,
    /// Streams converted before `.0` stop playing from `.1` on (copy queue).
    stops: Vec<(f64, f64)>,
    sense_open: bool,
    next_end: f64,
    period_frames_end: f64,
    events_seen: f64,
    edges: Vec<f64>,
    /// Per speaker: impedance (ohm) and phase.
    z: Vec<f64>,
    phi: Vec<f64>,
    /// The amplifier's peak output at full scale (V).
    vpk: f64,
    offsets: Vec<i32>,
    oracle: Oracle,
    /// Faults fire only before this (the quiet tail has none).
    calm_from: f64,
    pub last_period: f64,
    pub rate_failed: bool,
    pub last_ping_at: f64,
    pub sighted: f64,
    last_rate: i64,
    /// Bad sense data delivered to a run that knows of it, and the good
    /// complete periods delivered since.  A period counts once the daemon
    /// has its position (else it discards it unread).
    bad_seen: bool,
    good_since_bad: u32,
    pending: Option<bool>,
    /// Per speaker, models (asahi, legacy, unified ff, unified vi) that do
    /// not govern and whose truth was hotter than their latched start: (a)
    /// no longer holds for them (see `latched_start`).
    unverified: Vec<[bool; 4]>,
    /// A period with a dead current line was delivered: the current-fed
    /// models cannot know their truth from then on.
    i_dead_seen: Cell<bool>,
    /// A period with a line reading low (ILow) was delivered: the
    /// current-fed models are not checked against their truth from then on
    /// (its detection takes seconds of program); the long class checks the
    /// outcome.
    i_low_seen: Cell<bool>,
    gate: Option<crate::gate_sim::DescriptorQueue>,
    energy_deadline: f64,
}

impl<'a> ViWorld<'a> {
    pub fn new(sc: &'a ViScenario, cfg: &'a Config, use_gate:bool) -> ViWorld<'a> {
        let safe = cfg.globals.safe_reduction;
        let n = cfg.speakers.len();
        let vpk = 2f64.sqrt() * 10f64.powf(cfg.globals.amp_gain_dbv / 20.0);
        // down to the speaker's smallest impedance: where that draws more
        // than the sense's full scale, the current word clips at its rail
        // and the speaker still gets the whole current (review Z2)
        let z: Vec<f64> = cfg.speakers.iter().zip(&sc.loads).map(|(s, l)| s.z_min() * l.0).collect();
        let mut x = sc.seed | 1;
        let offsets = (0..cfg.globals.channels)
            .map(|_| {
                x = mix(x);
                if sc.dc > 0 { (x % (2 * sc.dc as u64 + 1)) as i32 - sc.dc } else { 0 }
            })
            .collect();
        ViWorld {
            sc,
            cfg,
            rate: sc.rate as f64,
            t0: Instant::now(),
            boot0: 1000.0,
            t: Cell::new(0.0),
            split_pending: Cell::new(0.0),
            pauses_used: vec![false; sc.faults.len()],
            once_used: vec![false; sc.faults.len()],
            controls: vec![sc.initial_db; n],
            owner: false,
            unlocked: false,
            lease_left: 0.0,
            notice: false,
            lease_t: 0.0,
            gains: vec![vec![(0.0, sc.initial_db.max(safe))]; n],
            stops: Vec::new(),
            sense_open: false,
            next_end: 0.0,
            period_frames_end: 0.0,
            events_seen: 0.0,
            edges: sc.edges(),
            phi: sc.loads.iter().map(|l| l.1).collect(),
            z,
            vpk,
            offsets,
            oracle: Oracle::new(cfg),
            calm_from: if sc.tail { sc.horizon } else { f64::INFINITY },
            last_period: 0.0,
            rate_failed: false,
            last_ping_at: 0.0,
            sighted: 0.0,
            last_rate: 0,
            bad_seen: false,
            good_since_bad: 0,
            pending: None,
            unverified: vec![[false; 4]; n],
            i_dead_seen: Cell::new(false),
            i_low_seen: Cell::new(false),
            gate: (use_gate && sc.amp_gain && !cfg.playback_cards.is_empty()).then(||
                {let mut q=crate::gate_sim::DescriptorQueue::new(cfg,sc.rate,cfg.globals.period,1_000_000_000_000);
                 q.set_playing(sc.open_at(0.0).is_some(),0.0);q}),
            energy_deadline: 0.0,
        }
    }

    pub fn now_s(&self) -> f64 {
        self.t.get()
    }

    fn safe(&self) -> f64 {
        self.cfg.globals.safe_reduction
    }

    /// The reduction speaker `k` gets now.
    pub fn effective(&self, k: usize) -> f64 {
        if self.unlocked { self.controls[k] } else { self.controls[k].max(self.safe()) }
    }

    fn note_gain(&mut self, at: f64) {
        for k in 0..self.controls.len() {
            let e = self.effective(k);
            if self.gains[k].last().is_none_or(|g| g.1 != e) {
                self.gains[k].push((at, e));
            }
        }
    }

    /// The lock went (given up, lapsed or its owner gone): locked at once;
    /// with a copy queue, streams converted at the old gain stop within
    /// `by`.
    fn relock(&mut self, at: f64, by: f64) {
        self.unlocked = false;
        self.lease_left = 0.0;
        if !self.sc.amp_gain {
            self.stops.push((at, at + by.clamp(0.0, DRAIN.as_secs_f64())));
        }
        self.note_gain(at);
    }

    fn gain_at(&self, k: usize, x: f64) -> f64 {
        let g = &self.gains[k];
        let i = g.partition_point(|e| e.0 <= x);
        if i == 0 { g[0].1 } else { g[i - 1].1 }
    }

    /// Speaker `k`'s true voltage and current at sample `n` (play time
    /// n / rate).
    fn truth(&self, k: usize, n: u64) -> (f64, f64) {
        let tau = n as f64 / self.rate;
        let Some(s) = self.sc.open_at(tau) else { return (0.0, 0.0) };
        let i = s.levels.partition_point(|l| l.0 <= tau);
        let level = if i == 0 { s.levels[0].1 } else { s.levels[i - 1].1 } * s.share[k];
        if level == 0.0 {
            return (0.0, 0.0);
        }
        let gain = if self.sc.amp_gain {
            self.gain_at(k, tau)
        } else {
            let src = (tau - s.depth).max(s.open);
            let kappa = s.open + ((src - s.open) / s.chunk).floor() * s.chunk;
            if self.stops.iter().any(|&(at, from)| kappa < at && tau >= from) {
                return (0.0, 0.0);
            }
            self.gain_at(k, kappa)
        };
        let level = recurrent::level(self.cfg.speakers[k].name.contains("Tweeter"), tau, level);
        let digital = self.gate.as_ref().map_or(1.0,|g|g.ratio(n,k));
        let a = level * digital * 10f64.powf(-gain / 20.0) * self.vpk;
        let freq = recurrent::frequency(tau, s.freq);
        let analogue=10f64.powf(-gain/20.0);
        if freq == 0.0 {
            let v = self.gate.as_ref().map_or_else(||if n%2==0 {a} else {-a},
                |q|f64::from(q.sample(n,k))/2147483648.0*self.vpk*analogue);
            (v, v / self.z[k])
        } else {
            let x = 2.0 * PI * freq * tau;
            let v=self.gate.as_ref().map_or(a*x.sin(),|q|
                f64::from(q.sample(n,k))/2147483648.0*self.vpk*analogue);
            (v, a / self.z[k] * (x - self.phi[k]).sin())
        }
    }

    /// The sense words of sample `n` into one frame.
    fn frame(&self, n: u64, f: &mut [i32], bad: bool) {
        let tau = n as f64 / self.rate;
        let dead = tau < self.calm_from && self.sc.faults.iter().any(|x| matches!(x, ViFault::IDead { at, len } if tau >= *at && tau < at + len));
        if dead {
            self.i_dead_seen.set(true);
        }
        let low = if tau < self.calm_from {
            self.sc.faults.iter().find_map(|x| match x {
                ViFault::ILow { at, len, k, offset } if tau >= *at && tau < at + len => Some((*k, *offset)),
                _ => None,
            })
        } else {
            None
        };
        if low.is_some() {
            self.i_low_seen.set(true);
        }
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            let ic = s.is_chan.unwrap();
            if bad {
                f[s.vs_chan] = 16384 << 16;
                f[ic] = -16384 << 16;
                continue;
            }
            let (v, i) = self.truth(k, n);
            let word = |x: f64, fs: f64, c: usize| -> i32 {
                let noise = if self.sc.noise > 0 { (mix(self.sc.seed ^ n.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ (c as u64) << 56) % (2 * self.sc.noise as u64 + 1)) as i32 - self.sc.noise } else { 0 };
                (((x / fs * 32768.0).round() as i64 + (noise + self.offsets[c]) as i64).clamp(-32768, 32767) as i32) << 16
            };
            f[s.vs_chan] = word(v, s.vs_scale, s.vs_chan);
            f[ic] = word(if dead { 0.0 } else if let Some((k, o)) = low { k * i + o } else { i }, s.is_scale, ic);
        }
    }

    /// Bring the world to `to`: the lease in playing time, the oracle.
    fn advance(&mut self, to: f64) {
        let to = to.max(self.now_s());
        let gate_deadline=self.gate.as_ref().map(|_|self.energy_deadline);
        let mut gate_epoch=None;
        if let Some(gate)=&mut self.gate {
            let sc=self.sc;let cfg=self.cfg;let rate=self.rate;
            let source_at=Cell::new(sc.open_at(self.lease_t));
            let mut programme=|n:u64,k:usize| {
                let tau=n as f64/rate;
                let Some(s)=source_at.get() else {return 0.0;};
                if tau<s.open || tau>=s.close {return 0.0;}
                let i=s.levels.partition_point(|l|l.0<=tau);
                let level=if i==0 {s.levels[0].1} else {s.levels[i-1].1} * s.share[k];
                let level=recurrent::level(cfg.speakers[k].name.contains("Tweeter"),tau,level);
                let freq=recurrent::frequency(tau,s.freq);
                if freq==0.0 {if n%2==0 {level} else {-level}} else {level*(2.0*PI*freq*tau).sin()}
            };
            for edge in self.edges.iter().copied().filter(|e|*e>self.lease_t && *e<=to) {
                gate.advance(edge,&mut programme);
                let next=sc.open_at(edge);
                if let (Some(a),Some(b))=(source_at.get(),next) {
                    if !std::ptr::eq(a,b) && gate.set_playing(false,edge) {gate_epoch.get_or_insert(edge);}
                }
                if gate.set_playing(next.is_some(),edge) {gate_epoch.get_or_insert(edge);}
                source_at.set(next);
            }
            gate.advance(to,&mut programme);
        }
        let mut t = self.lease_t;
        let step = 0.0005;
        if let Some(deadline)=gate_deadline {
            let end=gate_epoch.map_or(deadline,|e|e.min(deadline));
            if self.unlocked && end<=to {self.notice=true;self.relock(end.max(self.lease_t),0.0);}
        }
        while t < to {
            let b = (t + step).min(to);
            if self.gate.is_none() && self.unlocked && self.sc.open_at(t).is_some() {
                self.lease_left -= b - t;
                if self.lease_left <= 0.0 {
                    self.notice = true;
                    self.relock(b, DRAIN.as_secs_f64() * 0.9);
                }
            }
            t = b;
        }
        self.lease_t = to;
        self.t.set(to);
        let h = 1.0 / self.rate;
        let k = self.cfg.speakers.len();
        let mut p = vec![[0.0; 3]; k];
        while (self.oracle.n + 1) as f64 * h <= self.now_s() + 1e-12 {
            let n = self.oracle.n;
            for (j, s) in self.cfg.speakers.iter().enumerate() {
                let (v, i) = self.truth(j, n);
                p[j] = [v * i, v * v / s.z_nominal, v * v / s.r_ff];
            }
            self.oracle.step(h, &p);
            recurrent::observe(self);
        }
        if let Some(g)=&mut self.gate {g.prune_before(self.oracle.n.saturating_sub((self.rate*5.0) as u64));}
    }

    fn fault_on(&self, f: impl Fn(&Fault) -> Option<(f64, f64)>) -> bool {
        let t = self.now_s();
        t < self.calm_from && self.sc.base().any(|(_, x)| f(x).is_some_and(|(a, l)| t >= a && t < a + l))
    }

    fn once(&mut self, f: impl Fn(&ViFault) -> Option<f64>) -> Option<usize> {
        let t = self.now_s();
        if t >= self.calm_from {
            return None;
        }
        for (i, x) in self.sc.faults.iter().enumerate() {
            if let Some(at) = f(x) {
                if !self.once_used[i] && t >= at {
                    self.once_used[i] = true;
                    return Some(i);
                }
            }
        }
        None
    }

    fn maybe_pause(&mut self) {
        let t = self.now_s();
        if t >= self.calm_from {
            return;
        }
        for i in 0..self.sc.faults.len() {
            if let ViFault::Base(Fault::Pause { at, len }) = self.sc.faults[i] {
                if !self.pauses_used[i] && t >= at {
                    self.pauses_used[i] = true;
                    self.advance(t + len);
                    return;
                }
            }
        }
    }

    fn change_in(&self, from: f64, to: f64) -> Option<f64> {
        self.edges.iter().map(|e| e + self.sc.latency).find(|&e| e > from && e <= to)
    }

    pub fn owner_gone(&mut self) {
        if let Some(g)=&mut self.gate {g.gate.owner(false,g.ns(self.t.get())).unwrap();}
        self.owner = false;
        self.notice = false;
        let t = self.now_s();
        self.advance(t);
        self.relock(t, DRAIN.as_secs_f64());
        self.sense_open = false;
    }

    /// A new run: whether it resumed a clean stop (which carries what it
    /// knew of bad sense data).
    fn restarted(&mut self, resumed: bool) {
        if !resumed {
            self.bad_seen = false;
            self.good_since_bad = 0;
        }
    }

    fn period_s(&self) -> f64 {
        self.cfg.globals.period as f64 / self.rate
    }

    fn on_grid(&self, t: f64) -> f64 {
        (t * self.rate).ceil() / self.rate
    }

    fn fill(&self, words: &mut [i32], start: f64, frames: usize, bad: bool) {
        let ch = self.cfg.globals.channels;
        for i in 0..frames {
            let n = ((start * self.rate).round() as i64 + i as i64).max(0) as u64;
            self.frame(n, &mut words[i * ch..(i + 1) * ch], bad);
        }
    }
}

/// SplitMix64's finalizer: the world's noise from a sample and a channel.
fn mix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

impl Hw for ViWorld<'_> {
    fn now(&self) -> Instant {
        let p = self.split_pending.replace(0.0);
        if p > 0.0 {
            self.t.set(self.t.get() + p);
        }
        self.t0 + Duration::from_secs_f64(self.t.get())
    }
    fn boottime(&self) -> Duration {
        let t = self.t.get();
        if t < self.calm_from {
            if let Some((_, &Fault::ClockSplit { pause, .. })) = self.sc.base().find(|(_, f)| matches!(f, Fault::ClockSplit { at, len, .. } if t >= *at && t < at + len)) {
                self.split_pending.set(pause);
            }
        }
        Duration::from_secs_f64(self.boot0 + t)
    }
    fn sleep(&mut self, d: Duration) {
        let t = self.now_s();
        self.advance(t + d.as_secs_f64());
    }
    fn take_lock(&mut self) -> io::Result<()> {
        self.maybe_pause();
        self.owner = true;
        if let Some(g)=&mut self.gate {
            g.gate.owner(true,g.ns(self.t.get()))?;
            self.relock(self.t.get(),0.0);
        }
        Ok(())
    }
    fn playback_status(&mut self)->io::Result<Option<crate::playback::Snapshot>> {
        let t=self.now_s();self.advance(t);
        self.gate.as_mut().map(|g|g.gate.status(g.ns(t))).transpose()
    }
    fn playback_write(&mut self,wire:&[u8;crate::playback::BYTES])->io::Result<()> {
        self.maybe_pause();let t=self.now_s();self.advance(t);
        let Some(g)=&mut self.gate else {return Err(io::Error::from_raw_os_error(libc::ENOTTY));};
        let result=g.gate.write(wire,g.ns(t));
        self.unlocked=g.gate.unlocked(g.ns(t));
        if result.is_ok() {
            self.energy_deadline=u64::from_le_bytes(wire[32..40].try_into().unwrap()).saturating_sub(1_000_000_000_000) as f64*1e-9;
        }
        if !self.unlocked {self.lease_left=0.0;}
        self.note_gain(t);result
    }
    fn ping(&mut self) -> io::Result<()> {
        self.maybe_pause();
        let t = self.now_s();
        self.advance(t);
        if self.notice && self.gate.is_none() {
            self.notice = false;
            return Err(io::Error::from_raw_os_error(libc::ETIMEDOUT));
        }
        if !self.owner {
            return Err(io::Error::from_raw_os_error(libc::EPERM));
        }
        if let Some(g)=&mut self.gate {
            let result=g.gate.ping(g.ns(t));self.notice=false;result?;
        }
        self.unlocked = true;
        self.lease_left = 0.25;
        self.last_ping_at = self.now_s();
        let t = self.now_s();
        self.note_gain(t);
        Ok(())
    }
    fn step_db(&self) -> f64 {
        if self.sc.steps { 0.5 } else { 0.0 }
    }
    fn set_reduction(&mut self, db: f64) -> io::Result<f64> {
        let n = self.controls.len();
        self.set_reductions(&vec![db; n]).map(|a| a[0])
    }
    fn set_reductions(&mut self, db: &[f64]) -> io::Result<Vec<f64>> {
        self.maybe_pause();
        let t = self.now_s();
        self.advance(t);
        let applied: Vec<f64> = db.iter().map(|&d| if self.sc.steps { (d * 2.0 - 1e-9).ceil() / 2.0 } else { d }).collect();
        let fail = if t < self.calm_from {
            self.sc.base().find_map(|(_, f)| match f {
                Fault::WriteFail { at, len, partial } if t >= *at && t < at + len => Some(*partial),
                _ => None,
            })
        } else {
            None
        };
        if let Some(partial) = fail {
            if partial {
                // the first half of the controls take the new value
                for k in 0..applied.len().div_ceil(2) {
                    self.controls[k] = applied[k];
                }
                self.note_gain(t);
            }
            return Err(io::Error::from_raw_os_error(libc::EIO));
        }
        if !self.unlocked && db.iter().any(|&d| d < self.safe()) {
            return Err(io::Error::from_raw_os_error(libc::EINVAL));
        }
        self.controls.copy_from_slice(&applied);
        self.note_gain(t);
        Ok(applied)
    }
    fn set_min(&mut self) -> io::Result<()> {
        self.maybe_pause();
        let t = self.now_s();
        self.advance(t);
        if self.fault_on(|f| if let Fault::MinFail { at, len } = f { Some((*at, *len)) } else { None }) {
            return Err(io::Error::from_raw_os_error(libc::EIO));
        }
        for c in self.controls.iter_mut() {
            *c = MUTE_DB;
        }
        self.note_gain(t);
        Ok(())
    }
    fn relinquish(&mut self) -> io::Result<()> {
        self.maybe_pause();
        let t = self.now_s();
        self.advance(t);
        self.notice = false;
        if let Some(g)=&mut self.gate {
            let now=g.ns(t);g.gate.owner(true,now)?;
        }
        let t = self.now_s();
        self.relock(t, DRAIN.as_secs_f64() * 0.9);
        Ok(())
    }
    fn sample_rate(&mut self) -> io::Result<i64> {
        self.maybe_pause();
        let t = self.now_s();
        let mut delay = 0.0;
        if let Some(i) = self.once(|f| if let ViFault::Base(Fault::RateSlow { at, .. }) = f { Some(*at) } else { None }) {
            if let ViFault::Base(Fault::RateSlow { delay: d, .. }) = self.sc.faults[i] {
                delay = d;
            }
        }
        let seen = t + delay * 0.5;
        self.advance(t + delay);
        self.rate_failed = self.fault_on(|f| if let Fault::RateFail { at, len } = f { Some((*at, *len)) } else { None });
        if self.rate_failed {
            return Err(io::Error::from_raw_os_error(libc::EIO));
        }
        let v = if self.sc.open_at(seen).is_some() { self.rate as i64 } else { 0 };
        if v > 0 && self.last_rate == 0 {
            self.sighted = self.now_s();
        }
        self.last_rate = v;
        Ok(v)
    }
    fn open_sense(&mut self, _rate: u32) -> io::Result<()> {
        self.maybe_pause();
        let mut delay = 0.0;
        if let Some(i) = self.once(|f| if let ViFault::Base(Fault::SlowOpen { at, .. }) = f { Some(*at) } else { None }) {
            if let ViFault::Base(Fault::SlowOpen { delay: d, .. }) = self.sc.faults[i] {
                delay = d;
            }
        }
        let t = self.now_s();
        self.advance(t + delay);
        self.sense_open = true;
        self.next_end = self.on_grid(self.now_s() + self.period_s());
        Ok(())
    }
    fn close_sense(&mut self) {
        self.sense_open = false;
    }
    fn read_period(&mut self, words: &mut [i32]) -> io::Result<Option<bool>> {
        self.maybe_pause();
        let t = self.now_s();
        let deadline = t + 0.05;
        if self.sc.events {
            if let Some(c) = self.change_in(t, deadline) {
                if c < self.next_end {
                    self.advance(c);
                    return Ok(None);
                }
            }
        }
        if self.fault_on(|f| if let Fault::SenseLoss { at, len } = f { Some((*at, *len)) } else { None }) || self.sc.open_at(t).is_none() {
            self.advance(deadline);
            self.next_end = self.on_grid(self.now_s() + self.period_s());
            return Ok(None);
        }
        if self.next_end > deadline {
            self.advance(deadline);
            return Ok(None);
        }
        if t > self.next_end + self.period_s() {
            let behind = ((t - self.next_end) / self.period_s()).floor();
            self.next_end += behind * self.period_s();
        }
        let end = self.next_end;
        // a late block: the period reaches the daemon `delay` after it ended
        let mut late = 0.0;
        if let Some(i) = self.once(|f| if let ViFault::LateBlock { at, .. } = f { Some(*at) } else { None }) {
            if let ViFault::LateBlock { delay, .. } = self.sc.faults[i] {
                late = delay;
            }
        }
        self.advance(t.max(end + late));
        self.next_end += self.period_s();
        if self.once(|f| if let ViFault::Base(Fault::ShortPeriod { at }) = f { Some(*at) } else { None }).is_some() {
            return Ok(Some(false));
        }
        let now = self.now_s();
        let bad = now < self.calm_from && self.sc.faults.iter().any(|f| matches!(f, ViFault::BadData { at, len } if now >= *at && now < at + len));
        let p = self.cfg.globals.period;
        self.fill(words, end - p as f64 / self.rate, p, bad);
        self.pending = Some(bad);
        self.last_period = self.now_s();
        self.period_frames_end = end;
        Ok(Some(true))
    }
    fn queued(&mut self) -> io::Result<u64> {
        let pending = self.pending.take();
        if self.once(|f| if let ViFault::Base(Fault::PositionFail { at }) = f { Some(*at) } else { None }).is_some() {
            return Err(io::Error::from_raw_os_error(libc::EIO));
        }
        match pending {
            Some(true) => {
                self.bad_seen = true;
                self.good_since_bad = 0;
            }
            Some(false) => self.good_since_bad = self.good_since_bad.saturating_add(1),
            None => {}
        }
        let q = ((self.now_s() - self.period_frames_end).max(0.0) * self.rate) as u64;
        if let Some(i) = self.once(|f| if let ViFault::Base(Fault::SlowPosition { at, .. }) = f { Some(*at) } else { None }) {
            if let ViFault::Base(Fault::SlowPosition { delay, .. }) = self.sc.faults[i] {
                let t = self.now_s();
                self.advance(t + delay);
            }
        }
        Ok(q)
    }
    fn read_tail(&mut self, words: &mut [i32]) -> io::Result<usize> {
        if !self.sense_open {
            return Ok(0);
        }
        let start = self.next_end - self.period_s();
        let t = self.now_s();
        let stop = self.sc.streams.iter().find(|s| s.open <= start && start < s.close).map_or(start, |s| s.close.min(t));
        let p = self.cfg.globals.period;
        let k = (((stop - start) * self.rate).round().max(0.0) as usize).min(p - 1);
        let bad = t < self.calm_from && self.sc.faults.iter().any(|f| matches!(f, ViFault::BadData { at, len } if t >= *at && t < at + len));
        self.fill(words, start, k, bad);
        self.period_frames_end = start + k as f64 / self.rate;
        Ok(k)
    }
    fn wait_rate_change(&mut self, timeout: Duration) -> bool {
        self.maybe_pause();
        let t = self.now_s();
        let to = t + timeout.as_secs_f64();
        let late = if t < self.calm_from {
            self.sc.base().find_map(|(_, f)| match f {
                Fault::WakeLate { at, len, delay } if t >= *at && t < at + len => Some(*delay),
                _ => None,
            })
        } else {
            None
        };
        if self.sc.events {
            if let Some(c) = self.change_in(t, to) {
                self.advance(c + late.unwrap_or(0.0));
                return true;
            }
        }
        self.advance(to);
        false
    }
    fn rate_events(&mut self) -> bool {
        let t = self.now_s();
        let changed = self.sc.events && self.change_in(self.events_seen, t).is_some();
        self.events_seen = t;
        changed
    }
    fn events_available(&self) -> bool {
        self.sc.events
    }
}

/// What a case exercised (for the run's summary).
#[derive(Debug, Default, Clone, Copy)]
pub struct Outcome {
    pub latched: bool,
    pub handed_off: bool,
    pub rearmed_after_latch: bool,
    pub bad_data: bool,
    pub tail_checked: bool,
    pub tail_skipped: bool,
    pub hottest_true: f64,
    /// Per speaker, maximum true V/I temperature over every oracle sample.
    pub true_vi_peaks: [f64; 6],
    pub guarded: bool,
    pub i_dead: bool,
    /// The least, over speakers and time, of the hard limit less the
    /// measured-power truth and less the feed-forward truth (K).
    pub vi_to_limit: f64,
    pub ff_to_limit: f64,
    /// Times the lease was given up, and the playing time spent at the
    /// card's safe level after the first lock (s).
    pub handoffs: u32,
    pub kernel_s: f64,
    was_armed: bool,
    last_t: f64,
    /// The hottest asahi model coil (C) and the largest reduction the
    /// card applied while the lease was held (dB).
    pub asahi_peak: f64,
    pub card_max: f64,
}

/// speakersafetyd's ramp on the oracle's true coil and magnet (dB), linked
/// by group, for speaker `k` at time `t`.
fn need(cfg: &Config, o: &Oracle, k: usize, t: f64, rate: f64, slack: &[f64]) -> f64 {
    let g = &cfg.globals;
    let own = |j: usize| {
        let s = &cfg.speakers[j];
        let work = s.t_limit - s.t_headroom;
        let max_pwr = (work - g.t_ambient) / (s.vi_tr_magnet + s.vi_tr_coil);
        let peak_pwr = 2.0 * 10f64.powf(g.amp_gain_dbv / 10.0) / s.z_nominal;
        let min_gain = (10.0 * (max_pwr / peak_pwr).log10()).min(0.0);
        let (c, m) = o.at(j, VI, t, rate);
        -min_gain * ((c.max(m) - slack[j] - (work - g.t_window)) / g.t_window).max(0.0)
    };
    let group = cfg.speakers[k].group;
    (0..cfg.speakers.len()).filter(|&j| cfg.speakers[j].group == group).map(own).fold(0.0, f64::max)
}

/// How much cooler than its truth a current-fed model may be at `t` once a
/// dead current line was seen (K, coil and magnet alike), review r4 BB1
/// and DD3 of speakerguardd#8.  The engine bounds a lost line's power by
/// V^2 over the speaker's smallest impedance.  It can miss: steadily,
/// whole long blocks under the floor where the line's noise passes for a
/// reading; at each end of a dead stretch, until the engine has charged
/// what it may have missed there (by the end of the next stretch, or at a
/// gap), up to three short blocks at full power (two lost in a row are
/// needed, DD2) and two long blocks at the short blocks' floor; and inside
/// a cycle of a sine, the lead a reactive load's power can take over V^2
/// over its impedance, at most 0.171 Vp^2 / (|Z| omega) joules.
fn dead_slack(cfg: &Config, sc: &ViScenario, vpk: f64, k: usize, t: f64) -> f64 {
    let s = &cfg.speakers[k];
    let step = 1.0 / sc.rate as f64;
    let z_min = s.z_min();
    let i_noise = (sc.noise + sc.dc.abs()) as f64 / 32768.0 * s.is_scale;
    let floor = |ratio: f64| DEAD_V_MIN.max(ratio * s.z_nominal * i_noise).powi(2) / z_min;
    let (p_long, p_short, p_full) = (floor(DEAD_RATIO), floor(DEAD_RATIO_SHORT), vpk * vpk / z_min);
    let (short, long) = (dead_block(DEAD_SHORT_S, step) as f64, dead_block(DEAD_LONG_S, step) as f64);
    let edge = step * (3.0 * short * p_full + 2.0 * long * p_short);
    let (rc, rm) = (s.vi_tr_coil / s.vi_tau_coil, s.vi_tr_magnet / s.vi_tau_magnet);
    let period = cfg.globals.period as f64 * step;
    let mut edges = 0.0;
    for f in &sc.faults {
        if let ViFault::IDead { at, len } = *f {
            for e in [at, at + len] {
                if e <= t && t - e <= 3.0 * period {
                    edges += edge * rc.max(rm);
                }
            }
        }
    }
    let f_min = sc.streams.iter().filter(|x| x.freq > 0.0).map(|x| x.freq).fold(f64::INFINITY, f64::min);
    let lead = if f_min.is_finite() { 0.171 * p_full / (2.0 * PI * f_min) } else { 0.0 };
    p_long * (s.vi_tr_coil + s.vi_tr_magnet) + edges + lead * rc.max(rm)
}

fn check(cfg: &Config, w: &ViWorld, g: &Guard, sc: &ViScenario, out: &mut Outcome) -> Result<(), Violation> {
    let rate = w.rate;
    let at = (g.account_to() - w.t0).as_secs_f64();
    let now = w.now_s();
    let ta = cfg.globals.t_ambient;
    let e = g.engine.as_ref().expect("a V/I machine runs the engine");
    let tol = 0.02;
    let dead = w.i_dead_seen.get();
    let low = w.i_low_seen.get();
    let asahi_bound = g.governing == Governor::Asahi || g.guard() == Some(Governor::Asahi);
    // after a dead current line, what the current-fed models may lack
    let slack: Vec<f64> = (0..cfg.speakers.len()).map(|k| if dead { dead_slack(cfg, sc, w.vpk, k, at) } else { 0.0 }).collect();
    for (k, s) in cfg.speakers.iter().enumerate() {
        let name = &s.name;
        let vi = w.oracle.at(k, VI, at, rate);
        let ffz = w.oracle.at(k, FFZ, at, rate);
        let ffr = w.oracle.at(k, FFR, at, rate);
        out.hottest_true = out.hottest_true.max(vi.0).max(vi.1);
        out.vi_to_limit = out.vi_to_limit.min(s.t_limit - vi.0.max(vi.1));
        out.ff_to_limit = out.ff_to_limit.min(s.t_limit - ffr.0.max(ffr.1));
        let tv = tol + slack[k];
        // (a) every model never cooler than its truth
        let skip = w.unverified[k];
        let a = &e.asahi.as_ref().unwrap()[k];
        if !skip[0] && !low && (a.th.t_coil < vi.0 - tv || a.th.t_magnet < vi.1 - tv) {
            return Err(Violation(format!("(a) at {now:.4} s, accounted {at:.4} s: {name} asahi {:.3}/{:.3} C, true {:.3}/{:.3} C", a.th.t_coil, a.th.t_magnet, vi.0, vi.1)));
        }
        let l = &g.speakers[k];
        if !skip[1] && (l.t_coil < ffz.0 - tol || l.t_magnet < ffz.1 - tol) {
            return Err(Violation(format!("(a) at {now:.4} s, accounted {at:.4} s: {name} legacy {:.3}/{:.3} C, true {:.3}/{:.3} C", l.t_coil, l.t_magnet, ffz.0, ffz.1)));
        }
        if let Some(u) = e.unified.as_ref().map(|u| &u[k]) {
            if !skip[2] && (u.ff.t_coil < ffr.0 - tol || u.ff.t_magnet < ffr.1 - tol) {
                return Err(Violation(format!("(a) at {now:.4} s, accounted {at:.4} s: {name} unified ff {:.3}/{:.3} C, true {:.3}/{:.3} C", u.ff.t_coil, u.ff.t_magnet, ffr.0, ffr.1)));
            }
            if let Some(m) = &u.vi {
                if !skip[3] && !low && (m.t_coil < vi.0 - tv || m.t_magnet < vi.1 - tv) {
                    return Err(Violation(format!("(a) at {now:.4} s, accounted {at:.4} s: {name} unified vi {:.3}/{:.3} C, true {:.3}/{:.3} C", m.t_coil, m.t_magnet, vi.0, vi.1)));
                }
            }
        }
        let _ = ta;
        // (b) while the lease is held
        if g.armed() && w.unlocked {
            if !low && vi.0.max(vi.1) >= s.t_limit + slack[k] {
                return Err(Violation(format!("(b) at {now:.4} s: armed with {name} truly at {:.3}/{:.3} C", vi.0, vi.1)));
            }
            if g.governing == Governor::Unified && ffr.0.max(ffr.1) >= s.t_limit {
                return Err(Violation(format!("(b) at {now:.4} s: armed with {name}'s feed-forward truth at {:.3}/{:.3} C", ffr.0, ffr.1)));
            }
            if asahi_bound && !low {
                let need = need(cfg, &w.oracle, k, at, rate, &slack);
                if need > w.reduction_limit_db().unwrap() + 0.05 {
                    return Err(Violation(format!("(b) at {now:.4} s: armed while {name}'s true state needs {need:.3} dB, beyond the hardware range")));
                }
                if w.effective(k) < need - 0.05 {
                    return Err(Violation(format!("(b) at {now:.4} s: {name} card applies {:.3} dB, the true state needs {need:.3} dB", w.effective(k))));
                }
            }
        }
    }
    // (c)
    if g.armed() && w.unlocked {
        if !sc.events {
            return Err(Violation(format!("(c) at {now:.4} s: armed without change events")));
        }
        if w.rate_failed {
            return Err(Violation(format!("(c) at {now:.4} s: armed after a failed rate read")));
        }
        if w.bad_seen && w.good_since_bad < REARM_PERIODS {
            return Err(Violation(format!("(c) at {now:.4} s: armed {} good periods after bad sense data", w.good_since_bad)));
        }
        if e.governed(g.governing).1 {
            return Err(Violation(format!("(c) at {now:.4} s: armed with the governing model latched")));
        }
        let period = w.period_s();
        if sc.open_at(w.last_ping_at).is_some() && w.last_rate > 0 {
            let since = w.last_ping_at - w.last_period.max(w.sighted);
            if since > BLIND_LIMIT.as_secs_f64() + period + 0.06 {
                return Err(Violation(format!("(c) at {now:.4} s: the lease renewed at {:.4} s, {since:.3} s without a complete period while the speakers play", w.last_ping_at)));
            }
        }
    }
    let armed = g.armed() && w.unlocked;
    out.handoffs += (out.was_armed && !armed) as u32;
    if !armed && out.was_armed | (out.handoffs > 0) && sc.open_at(now).is_some() {
        out.kernel_s += (now - out.last_t).max(0.0);
    }
    out.was_armed = armed;
    out.last_t = now;
    out.asahi_peak = e.asahi.as_ref().unwrap().iter().map(|a| a.coil_peak).fold(out.asahi_peak, f64::max);
    if armed {
        out.card_max = (0..cfg.speakers.len()).map(|k| w.effective(k)).fold(out.card_max, f64::max);
    }
    let hard = e.governed(g.governing).1;
    out.latched |= hard;
    out.guarded |= g.guard().is_some();
    out.i_dead |= dead;
    out.handed_off |= !g.armed() && out.latched;
    out.rearmed_after_latch |= out.latched && g.armed() && !hard;
    out.bad_data |= w.bad_seen;
    Ok(())
}

/// How long the asahi models need from their state now, with no power, to
/// be under their working limits and the ramp's rearm point (s); None when
/// the lock could not come back at all.
fn cooling_bound(cfg: &Config, g: &Guard, max_reduction: f64) -> Option<f64> {
    let gl = &cfg.globals;
    let e = g.engine.as_ref()?;
    let a = e.asahi.as_ref()?;
    let allowed = max_reduction - REARM_MARGIN_DB;
    if allowed <= 0.0 {
        return None;
    }
    let mut worst: f64 = 0.0;
    for (s, x) in cfg.speakers.iter().zip(a) {
        let work = s.t_limit - s.t_headroom;
        let max_pwr = (work - gl.t_ambient) / (s.vi_tr_magnet + s.vi_tr_coil);
        let peak_pwr = 2.0 * 10f64.powf(gl.amp_gain_dbv / 10.0) / s.z_nominal;
        let min_gain = -(10.0 * (max_pwr / peak_pwr).log10()).min(0.0);
        // the ramp's held temperature is at most the hotter of coil and
        // magnet plus the hysteresis; the latch needs both under the limit
        let rearm = if min_gain > 0.0 { work - gl.t_window + gl.t_window * allowed / min_gain - gl.t_hysteresis } else { work };
        let node = Node { trc: s.vi_tr_coil, tau_c: s.vi_tau_coil, trm: s.vi_tr_magnet, tau_m: s.vi_tau_magnet };
        let ta = gl.t_ambient;
        let mut y = (x.th.t_coil.max(x.coil_peak).max(x.coil_hyst) - ta, x.th.t_magnet.max(x.magnet_peak).max(x.magnet_hyst) - ta);
        let mut t = 0.0;
        let h = 0.01;
        while y.0.max(y.1) + ta >= rearm.min(work) - 1e-9 {
            y = node.step(y, 0.0, h);
            t += h;
            if t > TAIL_MAX {
                return Some(f64::INFINITY);
            }
        }
        worst = worst.max(t);
    }
    Some(worst)
}

/// Run one scenario; the first violation, if any.
pub fn run(sc: &ViScenario, seed: u64) -> Result<Outcome, Violation> {
    run_internal(sc,seed,std::env::var_os("SPEAKERGUARDD_GATE_MODEL").is_some())
}

fn run_internal(sc:&ViScenario,seed:u64,use_gate:bool)->Result<Outcome,Violation> {
    let cfg = vi_cfg(sc);
    let path = state_dir(seed ^ 0x76);
    if sc.clean_start {
        let ta = cfg.globals.t_ambient;
        let cold: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), ta)).collect();
        let engine = Engine::new(&cfg, sc.unified || sc.guard || sc.shadow);
        let boot = Duration::from_secs(1000);
        std::fs::write(&path, state::encode_with(boot, boot, &cold, engine.as_ref(), false, true)).unwrap();
    }
    let mut rng = Rng::new(seed ^ 0x5eed);
    let mut w = ViWorld::new(sc, &cfg,use_gate);
    if let Some(q)=&mut w.gate {q.debug_case=seed;}
    let mut restarts = sc.restarts.clone();
    restarts.sort_by(|a, b| a.at.total_cmp(&b.at));
    let mut next_restart = 0;
    let mut expect_latched = !sc.clean_start;
    let mut out = Outcome { vi_to_limit: f64::INFINITY, ff_to_limit: f64::INFINITY, ..Outcome::default() };
    let result = (|| -> Result<(), Violation> {
        loop {
            let mut g = match Guard::start(&mut w, &cfg, sc.max_reduction, Some(path.clone())) {
                Ok(g) => g,
                Err(e) => return Err(Violation(format!("start failed: {e}"))),
            };
            let e = g.engine.as_ref().unwrap();
            if expect_latched && !e.asahi.as_ref().unwrap().iter().zip(&cfg.speakers).all(|(a, s)| a.hard && a.th.t_coil >= s.t_limit) {
                return Err(Violation(format!("at {:.4} s: a start from no clean stop was not latched", w.now_s())));
            }
            w.restarted(!expect_latched);
            if expect_latched {
                latched_start(&cfg, &mut w, &g);
            }
            let stop_at = restarts.get(next_restart).map_or(f64::INFINITY, |r| r.at);
            let mut exited = false;
            loop {
                if w.now_s() >= sc.horizon {
                    return if sc.tail { tail(&cfg, &mut w, &mut g, sc, &mut out) } else { Ok(()) };
                }
                if w.now_s() >= stop_at {
                    break;
                }
                if g.tick(&mut w).is_err() {
                    exited = true;
                    break;
                }
                let t = w.now_s();
                w.advance(t);
                check(&cfg, &w, &g, sc, &mut out)?;
            }
            let r = if exited {
                Restart { at: w.now_s(), clean: false, gap: 1.0, damage: Damage::None }
            } else {
                next_restart += 1;
                restarts[next_restart - 1]
            };
            let mut clean = false;
            if r.clean && g.stop(&mut w).is_ok() {
                let mut n = 0;
                while !g.released(&w) && n < 1000 {
                    if g.tick(&mut w).is_err() {
                        break;
                    }
                    let t = w.now_s();
                    w.advance(t);
                    check(&cfg, &w, &g, sc, &mut out)?;
                    n += 1;
                }
                clean = g.released(&w) && g.final_save(&w).is_ok();
            }
            drop(g);
            w.owner_gone();
            let t = w.now_s();
            w.advance(t + r.gap);
            damage(&path, r.damage, &mut rng);
            expect_latched = !clean || r.damage != Damage::None;
            if w.now_s() >= sc.horizon && !sc.tail {
                return Ok(());
            }
        }
    })();
    if let Some(a)=&w.oracle.audit {
        let mut rows=Vec::new();
        for (k,s) in cfg.speakers.iter().enumerate(){for(j,model)in[(VI,"vi"),(FFR,"ff")] {
            let(c,m)=a[k][j].peaks();rows.push(super::continuous::Row{speaker:s.name.clone(),model,active:j==VI||cfg.globals.governor==Governor::Unified,limit:s.t_limit,coil:c,magnet:m});
        }}
        super::continuous::record(seed,sc.to_rust(),if sc.j413{"j413"}else{"j316"},rows,result.as_ref().err().map(|e|e.0.clone()),out.handoffs as u64,w.gate.is_some());
    }
    let _ = std::fs::remove_dir_all(path.parent().unwrap());
    out.true_vi_peaks = w.oracle.vi_peaks;
    result.map(|()| out)
}

/// A start from no clean stop puts every model at its hard limit plus
/// what the drain can add.  That holds the truth only for the models
/// whose governor kept it under the limit: one that runs in shadow may
/// have been let past it.  Such a model whose truth is hotter than its
/// start is not checked against it from here on.  (A governing model's
/// is a violation, which (a) reports.)
fn latched_start(cfg: &Config, w: &mut ViWorld, g: &Guard) {
    let at = (g.account_to() - w.t0).as_secs_f64();
    let e = g.engine.as_ref().unwrap();
    let guarded = g.guard() == Some(Governor::Asahi);
    let governs = |m: usize| match g.governing {
        Governor::Unified => m == 2 || m == 3 || (guarded && m == 0),
        _ => m == 0,
    };
    for k in 0..cfg.speakers.len() {
        let vi = w.oracle.at(k, VI, at, w.rate);
        let ffz = w.oracle.at(k, FFZ, at, w.rate);
        let ffr = w.oracle.at(k, FFR, at, w.rate);
        let a = &e.asahi.as_ref().unwrap()[k];
        let l = &g.speakers[k];
        let mut models = vec![(0, (a.th.t_coil, a.th.t_magnet), vi), (1, (l.t_coil, l.t_magnet), ffz)];
        if let Some(u) = e.unified.as_ref().map(|u| &u[k]) {
            models.push((2, (u.ff.t_coil, u.ff.t_magnet), ffr));
            if let Some(v) = &u.vi {
                models.push((3, (v.t_coil, v.t_magnet), vi));
            }
        }
        for (m, model, truth) in models {
            if !governs(m) && (truth.0 > model.0 || truth.1 > model.1) {
                w.unverified[k][m] = true;
            }
        }
    }
}

/// The quiet tail and (d).
fn tail(cfg: &Config, w: &mut ViWorld, g: &mut Guard, sc: &ViScenario, out: &mut Outcome) -> Result<(), Violation> {
    let debug = std::env::var("SPEAKERGUARDD_VI_DEBUG").is_ok();
    let mut last_print = 0.0;
    let mut quiet = |w: &mut ViWorld, g: &mut Guard, until: f64| -> Result<(), Violation> {
        while w.now_s() < until {
            if debug && w.now_s() >= last_print + 1.0 {
                last_print = w.now_s();
                let e = g.engine.as_ref().unwrap();
                let a = e.asahi.as_ref().unwrap();
                let at = (g.account_to() - w.t0).as_secs_f64();
                eprintln!("TAIL now {:.3} at {:.3} armed {} unlocked {} hard {} red {:?} asahi {:?}", w.now_s(), at, g.armed(), w.unlocked, e.hard(Governor::Asahi), e.reductions(Governor::Asahi).iter().map(|r| (r * 100.0).round() / 100.0).collect::<Vec<_>>(), a.iter().map(|x| ((x.th.t_coil * 100.0).round() / 100.0, (x.th.t_magnet * 100.0).round() / 100.0, x.hard)).collect::<Vec<_>>());
            }
            if let Err(e) = g.tick(w) {
                return Err(Violation(format!("(d) at {:.4} s: the daemon exited in the quiet tail: {e}", w.now_s())));
            }
            let t = w.now_s();
            w.advance(t);
            check(cfg, w, g, sc, out)?;
        }
        Ok(())
    };
    // past the drain of a hand-off at the tail's start first: until then
    // uncovered time may still be charged at the old gain
    let settle = w.now_s() + DRAIN.as_secs_f64() + 0.5;
    quiet(w, g, settle)?;
    let bound = if sc.events && g.governing == Governor::Asahi { cooling_bound(cfg, g, w.reduction_limit_db().unwrap()) } else { None };
    let assert_d = bound.is_some_and(|b| b + TAIL_SLACK <= TAIL_MAX);
    let len = bound.map_or(5.0, |b| (b + TAIL_SLACK).min(TAIL_MAX));
    let from = w.now_s();
    quiet(w, g, from + len)?;
    if assert_d {
        out.tail_checked = true;
        let e = g.engine.as_ref().unwrap();
        if e.hard(Governor::Asahi) || !(g.armed() && w.unlocked) {
            let a = e.asahi.as_ref().unwrap();
            let temps: Vec<String> = a.iter().map(|x| format!("{:.2}/{:.2}{}", x.th.t_coil, x.th.t_magnet, if x.hard { " latched" } else { "" })).collect();
            return Err(Violation(format!("(d) at {:.4} s: {:.1} s of quiet ({:.1} s to cool) and the lock is not back: asahi {}", w.now_s(), len, bound.unwrap(), temps.join(", "))));
        }
    } else {
        out.tail_skipped = true;
    }
    Ok(())
}

/// Shrink a failing scenario while it still fails.
pub fn shrink(mut sc: ViScenario, seed: u64) -> ViScenario {
    let fails = |s: &ViScenario| run(s, seed).is_err();
    let mut progress = true;
    while progress {
        progress = false;
        for i in (0..sc.faults.len()).rev() {
            let mut t = sc.clone();
            t.faults.remove(i);
            if fails(&t) {
                sc = t;
                progress = true;
            }
        }
        for i in (0..sc.restarts.len()).rev() {
            let mut t = sc.clone();
            t.restarts.remove(i);
            if fails(&t) {
                sc = t;
                progress = true;
            }
        }
        let keep = if sc.tail { 1 } else { 0 };
        for i in (0..sc.streams.len().saturating_sub(keep)).rev() {
            let mut t = sc.clone();
            t.streams.remove(i);
            if fails(&t) {
                sc = t;
                progress = true;
            }
        }
        for i in 0..sc.streams.len() {
            while sc.streams[i].levels.len() > 1 {
                let mut t = sc.clone();
                t.streams[i].levels.pop();
                if fails(&t) {
                    sc = t;
                    progress = true;
                } else {
                    break;
                }
            }
        }
        for (field, zero) in [(0, 0), (1, 0)] {
            let mut t = sc.clone();
            match field {
                0 if t.noise != zero => t.noise = zero,
                1 if t.dc != zero => t.dc = zero,
                _ => continue,
            }
            if fails(&t) {
                sc = t;
                progress = true;
            }
        }
        if !sc.tail {
            let mut t = sc.clone();
            t.horizon *= 0.8;
            if t.horizon > 0.05 && fails(&t) {
                sc = t;
                progress = true;
            }
        }
    }
    sc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cases() -> u64 {
        std::env::var("SPEAKERGUARDD_VI_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(60)
    }

    fn first_seed() -> u64 {
        std::env::var("SPEAKERGUARDD_VI_SEED").ok().and_then(|v| v.parse().ok()).unwrap_or(1)
    }

    /// The oracle's integrator against the closed form of a constant power
    /// (a check that the truth is right, written apart from engine.rs).
    #[test]
    fn the_oracles_integrator_matches_the_closed_form() {
        let n = Node { trc: 104.9, tau_c: 1.85, trm: 200.0, tau_m: 70.0 };
        let (p, h) = (3.0, 1.0 / 16000.0);
        let mut y = (0.0, 0.0);
        let steps = 16000 * 30;
        for _ in 0..steps {
            y = n.step(y, p, h);
        }
        let t = steps as f64 * h;
        let em = (-t / n.tau_m).exp();
        let ec = (-t / n.tau_c).exp();
        let m = p * n.trm * (1.0 - em);
        let k = -p * n.trm * n.tau_m / (n.tau_m - n.tau_c);
        let c = p * (n.trc + n.trm) + k * em - (p * (n.trc + n.trm) + k) * ec;
        assert!((y.1 - m).abs() < 1e-6 && (y.0 - c).abs() < 1e-6, "{y:?} vs {c} / {m}");
    }

    /// Silence followed by full-scale squares on every output, including
    /// a gain write that fails partly just before another onset. Exercise
    /// both instantaneous amplifier gain and old-gain audio in a queue.
    #[test]
    fn vi_full_scale_onsets_with_queued_audio_stay_bounded() {
        for j413 in [false, true] {
            for guard in [false, true] {
                for amp_gain in [false, true] {
                    let mut sc = ViScenario::long(j413, 0.0, false, 1.0, guard, false, 20.0);
                    sc.amp_gain = amp_gain;
                    sc.loads.fill((1.0, 0.0));
                    sc.noise = 0;
                    sc.dc = 0;
                    sc.rate = 48000;
                    sc.streams[0].depth = 0.213;
                    sc.streams[0].chunk = 0.128;
                    sc.streams[0].levels = vec![(0.05, 0.0), (2.0, 1.0), (7.0, 0.0), (10.0, 1.0)];
                    sc.faults.push(ViFault::Base(Fault::WriteFail { at: 9.95, len: 0.2, partial: true }));
                    let out = run(&sc, 3085).unwrap_or_else(|v| panic!("{}\n{}", sc.to_rust(), v.0));
                    assert!(out.vi_to_limit > 0.0, "VI truth reached its limit: {out:?}");
                    if guard { assert!(out.ff_to_limit > 0.0, "FF truth reached its limit: {out:?}"); }
                }
            }
        }
    }

    #[test]
    fn vi_trace_one() {
        let Ok(seed) = std::env::var("SPEAKERGUARDD_VI_TRACE").map(|v| v.parse::<u64>().unwrap()) else { return };
        let sc = ViScenario::random(seed);
        let r = run(&sc, seed);
        eprintln!("{r:?}");
        if r.is_err() {
            let small = shrink(sc, seed);
            eprintln!("{}\n{:?}", small.to_rust(), run(&small, seed));
        }
    }

    /// Review r4 BB2 of speakerguardd#8: the long class (see the module's
    /// doc).  Before BB1 a dead line left the feed-forward model alone and
    /// took the measured-power truth to 176-186 C on the J316, and the
    /// asahi governor alone did nothing at all.
    #[test]
    fn vi_long_programs_keep_the_properties() {
        let all = std::env::var("SPEAKERGUARDD_VI_LONG").is_ok();
        let secs = if all { 120.0 } else { 30.0 };
        let mut jobs = Vec::new();
        for j413 in [false, true] {
            for (freq, tweeters) in [(100.0, false), (3000.0, true), (0.0, false)] {
                for level in [0.2, 0.35, 0.5, 0.7, 1.0] {
                    for (guard, dead) in [(false, false), (true, false), (false, true), (true, true)] {
                        jobs.push((j413, freq, tweeters, level, guard, dead));
                    }
                }
            }
        }
        if !all {
            // the review's hottest dead-line programs
            jobs.retain(|&(j413, freq, _, level, _, dead)| dead && matches!((j413, freq as u32, (level * 100.0) as u32), (false, 100, 35) | (false, 3000, 50) | (true, 0, 20)));
        }
        // lines that read low on a program with low-band current, and dead
        // lines carrying an offset (review r4 DD1): (program, k, offset)
        let mut low: Vec<((bool, f64, bool, f64), f64, f64, bool)> = Vec::new();
        for guard in [true, false] {
            for k in [0.11, 0.2, 0.5] {
                low.push(((false, 100.0, false, 0.35), k, 0.0, guard));
                low.push(((true, 100.0, false, 0.35), k, 0.0, guard));
            }
            for o in [0.1, 0.3] {
                low.push(((false, 0.0, false, 0.35), 0.0, o, guard));
                low.push(((false, 3000.0, true, 0.5), 0.0, o, guard));
                low.push(((true, 0.0, false, 0.2), 0.0, o, guard));
            }
        }
        if !all {
            low.retain(|&((j413, freq, _, _), k, o, guard)| !j413 && guard && ((freq == 100.0 && k == 0.2) || (freq == 0.0 && o == 0.3)));
        }
        let threads = std::thread::available_parallelism().map_or(1, |p| p.get()).min(32);
        let lines = std::sync::Mutex::new(Vec::new());
        let failed = std::sync::Mutex::new(Vec::new());
        std::thread::scope(|s| {
            for chunk in jobs.chunks(jobs.len().div_ceil(threads)) {
                let (lines, failed) = (&lines, &failed);
                s.spawn(move || {
                    for &(j413, freq, tweeters, level, guard, dead) in chunk {
                        let sc = ViScenario::long(j413, freq, tweeters, level, guard, dead, secs);
                        let what = format!("{} {} Hz{} at {level} {} {}", if j413 { "J413" } else { "J316" }, freq, if tweeters { " (tweeters)" } else { "" }, if guard { "guarded" } else { "asahi" }, if dead { "dead" } else { "live" });
                        match run(&sc, 700000001) {
                            Ok(o) => {
                                recurrent::assert_true_vi_limit(&sc, &o);
                                lines.lock().unwrap().push(format!("{what}: VI truth {:.1} K under its limit, FF truth {:.1}", o.vi_to_limit, o.ff_to_limit));
                                if !(o.vi_to_limit > 0.0) || (guard && !(o.ff_to_limit > 0.0)) {
                                    failed.lock().unwrap().push(format!("{what}: a truth reached its limit: VI {:.2} K, FF {:.2} K under it", o.vi_to_limit, o.ff_to_limit));
                                }
                            }
                            Err(v) => failed.lock().unwrap().push(format!("{what}: {}", v.0)),
                        }
                    }
                });
            }
        });
        std::thread::scope(|s| {
            for chunk in low.chunks(low.len().div_ceil(threads).max(1)) {
                let (lines, failed) = (&lines, &failed);
                s.spawn(move || {
                    for &((j413, freq, tweeters, level), k, offset, guard) in chunk {
                        let mut sc = ViScenario::long(j413, freq, tweeters, level, guard, false, secs);
                        sc.faults.push(ViFault::ILow { at: 0.0, len: secs + 10.0, k, offset });
                        let what = format!("{} {} Hz{} at {level} {} reading {k} plus {offset} A", if j413 { "J413" } else { "J316" }, freq, if tweeters { " (tweeters)" } else { "" }, if guard { "guarded" } else { "asahi" });
                        match run(&sc, 700000001) {
                            Ok(o) => {
                                recurrent::assert_true_vi_limit(&sc, &o);
                                lines.lock().unwrap().push(format!("{what}: VI truth {:.1} K under its limit, FF truth {:.1}", o.vi_to_limit, o.ff_to_limit));
                                if !(o.vi_to_limit > 0.0) {
                                    failed.lock().unwrap().push(format!("{what}: the measured-power truth reached its limit: {:.2} K under it", o.vi_to_limit));
                                }
                            }
                            Err(v) => failed.lock().unwrap().push(format!("{what}: {}", v.0)),
                        }
                    }
                });
            }
        });
        let mut v = lines.into_inner().unwrap();
        v.sort();
        eprintln!("VI LONG\n{}", v.join("\n"));
        let f = failed.into_inner().unwrap();
        assert!(f.is_empty(), "{}", f.join("\n"));
    }

    /// Review r4 DD1 of speakerguardd#8: lines that read low but plausibly,
    /// or a dead line carrying a DC offset, over the review's 120 s programs
    /// (SPEAKERGUARDD_VI_LOW_S sets the length).  Prints the hottest
    /// measured-power truth per program, line and mode; the properties are
    /// checked as for any case, with the current-fed models' (a) and (b)
    /// left out once such a line is seen.  SPEAKERGUARDD_VI_PROBE runs it.
    #[test]
    fn vi_low_lines_probe() {
        if std::env::var("SPEAKERGUARDD_VI_PROBE").is_err() {
            return;
        }
        let secs: f64 = std::env::var("SPEAKERGUARDD_VI_LOW_S").ok().and_then(|v| v.parse().ok()).unwrap_or(120.0);
        let programs = [(false, 0.0, false, 0.35), (false, 100.0, false, 0.35), (false, 3000.0, true, 0.5), (true, 0.0, false, 0.2), (true, 100.0, false, 0.35)];
        let lines = [(0.0, 0.0), (0.11, 0.0), (0.2, 0.0), (0.5, 0.0), (0.7, 0.0), (1.0, 0.0), (0.0, 0.1), (0.0, 0.3)];
        let mut jobs = Vec::new();
        for &p in &programs {
            for &l in &lines {
                for guard in [true, false] {
                    jobs.push((p, l, guard));
                }
            }
        }
        let threads = std::thread::available_parallelism().map_or(1, |p| p.get()).min(32);
        let out = std::sync::Mutex::new(Vec::new());
        std::thread::scope(|sc| {
            for chunk in jobs.chunks(jobs.len().div_ceil(threads)) {
                let out = &out;
                sc.spawn(move || {
                    for &((j413, freq, tw, level), (k, offset), guard) in chunk {
                        let mut v = ViScenario::long(j413, freq, tw, level, guard, false, secs);
                        if (k, offset) != (1.0, 0.0) {
                            v.faults.push(ViFault::ILow { at: 0.0, len: secs + 10.0, k, offset });
                        }
                        let what = format!("{} {} Hz{} {level} {} k {k} offset {offset}", if j413 { "J413" } else { "J316" }, freq, if tw { " tweeters" } else { "" }, if guard { "guarded" } else { "asahi" });
                        let line = match run(&v, 700000003) {
                            Ok(o) => format!("{what}: VI truth {:.1} C, {:.1} K under the limit", o.hottest_true, o.vi_to_limit),
                            Err(e) => format!("{what}: {}", e.0),
                        };
                        out.lock().unwrap().push(line);
                    }
                });
            }
        });
        let mut v = out.into_inner().unwrap();
        v.sort();
        eprintln!("LOW\n{}", v.join("\n"));
    }

    /// Short streams a few seconds apart on the J413 (a J713 user's report
    /// on speakerguardd#8): each stream's open and close are charged at
    /// full scale for the time no frames measured, about 2 C a millisecond
    /// on the tweeters' models.  Prints, per scenario, the hand-offs, the
    /// playing time at the card's safe level, the hottest asahi coil and the
    /// largest reduction while the lease was held.  SPEAKERGUARDD_VI_PROBE
    /// runs it.
    #[test]
    fn vi_short_streams_probe() {
        if std::env::var("SPEAKERGUARDD_VI_PROBE").is_err() {
            return;
        }
        let mut lines = Vec::new();
        for (j413, guard) in [(true, false), (true, true), (false, false), (false, true)] {
            for (n, gap, len, level) in [(4, 8.0, 0.5, 0.3), (4, 8.0, 0.5, 0.05), (10, 3.0, 0.3, 0.3), (6, 1.0, 0.2, 0.1)] {
                let mut sc = ViScenario::long(j413, 1000.0, false, level, guard, false, 1.0);
                let spk = sc.loads.len();
                sc.streams.clear();
                let start = 1.0;
                for i in 0..n {
                    let open = start + i as f64 * gap;
                    sc.streams.push(ViStream { open, close: open + len, depth: 0.0, chunk: 0.01, levels: vec![(open, level)], share: vec![1.0; spk], freq: 1000.0 });
                }
                sc.horizon = start + n as f64 * gap + 1.0;
                let what = format!("{} {} {n} x {len} s every {gap} s at {level}", if j413 { "J413" } else { "J316" }, if guard { "guarded" } else { "asahi" });
                match run(&sc, 700000002) {
                    Ok(o) => lines.push(format!("{what}: hand-offs {}, {:.2} s at the safe level, asahi coil peak {:.1} C, card up to {:.1} dB", o.handoffs, o.kernel_s, o.asahi_peak, o.card_max)),
                    Err(v) => lines.push(format!("{what}: {}", v.0)),
                }
            }
        }
        eprintln!("PROBE\n{}", lines.join("\n"));
    }

    /// The randomized interleavings on the V/I machines (see the module's
    /// doc).
    #[test]
    fn vi_random_interleavings_keep_the_properties() {
        let n = cases();
        let from = first_seed();
        let threads = std::thread::available_parallelism().map_or(1, |p| p.get()).min(32) as u64;
        let found: std::sync::Mutex<Option<(u64, String)>> = std::sync::Mutex::new(None);
        let tally = std::sync::Mutex::new([0u64; 10]);
        std::thread::scope(|s| {
            for th in 0..threads {
                let (found, tally) = (&found, &tally);
                s.spawn(move || {
                    let mut seed = from + th;
                    while seed < from + n {
                        if found.lock().unwrap().as_ref().is_some_and(|f| f.0 < seed) {
                            return;
                        }
                        let sc = ViScenario::random(seed);
                        match run(&sc, seed) {
                            Ok(o) => {
                                let mut t = tally.lock().unwrap();
                                t[0] += 1;
                                t[1] += o.latched as u64;
                                t[2] += o.handed_off as u64;
                                t[3] += o.rearmed_after_latch as u64;
                                t[4] += o.bad_data as u64;
                                t[5] += o.tail_checked as u64;
                                t[6] += o.tail_skipped as u64;
                                t[7] += (o.hottest_true >= 150.0) as u64;
                                t[8] += o.guarded as u64;
                                t[9] += o.i_dead as u64;
                            }
                            Err(v) => {
                                let mut f = found.lock().unwrap();
                                if f.as_ref().is_none_or(|x| seed < x.0) {
                                    *f = Some((seed, v.0));
                                }
                                return;
                            }
                        }
                        seed += threads;
                    }
                });
            }
        });
        let t = tally.into_inner().unwrap();
        let summary = format!("VI SUMMARY seeds {from}..{} passed {} latched {} handed-off {} rearmed-after-latch {} bad-data {} tail-checked {} tail-skipped {} true-over-150 {} guarded {} dead-current {}", from + n - 1, t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8], t[9]);
        eprintln!("{summary}");
        if let Ok(path) = std::env::var("SPEAKERGUARDD_VI_SUMMARY") {
            use std::io::Write as _;
            let found = found.lock().unwrap().clone();
            let line = format!("{summary}{}\n", found.map_or(String::new(), |(seed, why)| format!(" FAILED seed {seed}: {why}")));
            let _ = std::fs::OpenOptions::new().create(true).append(true).open(path).and_then(|mut f| f.write_all(line.as_bytes()));
        }
        if let Some((seed, why)) = found.into_inner().unwrap() {
            let small = shrink(ViScenario::random(seed), seed);
            let after = run(&small, seed).err().map(|v| v.0).unwrap_or_default();
            panic!("seed {seed}: {why}\nshrunk: {after}\nWITNESS {}", small.to_rust());
        }
    }
}

#[test]
fn kernel_gate_reference_plants_survive_retained_fault_schedules() {
    for seed in [7312918695,7312919903,7312900005] {
        let mut sc=ViScenario::random(seed);
        // Explicit scalar reference path, with the original fault schedule.
        // The unchanged continuous campaign separately retains both gain types.
        sc.amp_gain=true;
        run_internal(&sc,seed,true).unwrap();
    }
}

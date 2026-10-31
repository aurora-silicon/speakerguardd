// SPDX-License-Identifier: MIT
//! Randomized interleavings of streams, the card, its controls, its lease,
//! the clock and restarts, run against the real guard, and an oracle
//! written apart from the daemon's code that knows what was really played.
//!
//! The world here is ground truth: the program before the card's gain, the
//! gain the card really converted each copy chunk at (its controls, rounded
//! to their steps, or its own safe level whenever no live lease is held),
//! the playback queue each stream runs behind, the streams that macaudio
//! stops when its lock is given up or lapses, the lease counted in playing
//! time, the change events and their latency, failing and slow controls,
//! a sense capture that loses, shortens and stalls, a daemon descheduled
//! between any two calls, and restarts (clean, crashed, with the state file
//! damaged, removed or rewritten as the old format).
//!
//! The oracle integrates the true delivered power of every sample into its
//! own two-node thermal model (its own integrator, sub-stepped Euler) and
//! keeps the true power before the gain for the budgets.  After every pass
//! of the loop it asserts:
//!
//! * (a) the daemon's model is never cooler than the oracle's at the time
//!   the model has accounted up to, and its budgets never hold less energy
//!   before the gain than the oracle's over the same windows;
//! * (b) while the daemon holds the lease, the reduction the card applies to
//!   new conversions is never less than what the oracle's true state needs
//!   at that time (the governor's ramp and both budgets, without release);
//! * (c) what cannot be proven ends with the card at its safe level: no
//!   lock without change events, none after a failed rate read in that
//!   pass, no renewal while the speakers play without a complete period for
//!   the blind limit, and a start from anything but a clean stop is
//!   latched.
//!
//! `SPEAKERGUARDD_CASES` sets the number of random cases (default 200) and
//! `SPEAKERGUARDD_SEED` the first seed; failures are shrunk and printed as a
//! scenario to paste into a witness test.

use std::cell::Cell;
use std::io;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use crate::config::Config;
use crate::guard::{Guard, Hw, BLIND_LIMIT, DRAIN, MUTE_DB};
use crate::model::SpeakerState;
use crate::state;

/// The V/I machines' world (the J316 and the J413).
pub mod vi;

const J700: &str = include_str!("../conf/apple/j700.conf");
/// The world's sample rate.
const HARNESS_RATE: f64 = 16000.0;

/// SplitMix64.
#[derive(Clone)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed ^ 0x9e37_79b9_7f4a_7c15)
    }
    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    pub fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.f()
    }
    pub fn chance(&mut self, p: f64) -> bool {
        self.f() < p
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Stream {
    pub open: f64,
    pub close: f64,
    /// How far behind conversion the stream plays (s, at most 341 ms).
    pub depth: f64,
    /// Copy chunk (s).
    pub chunk: f64,
    /// (from, level as a fraction of full scale), in play time.
    pub levels: Vec<(f64, f64)>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Fault {
    /// Sample-rate reads fail.
    RateFail { at: f64, len: f64 },
    /// One sample-rate read takes this long.
    RateSlow { at: f64, delay: f64 },
    /// Idle waits wake this late.
    WakeLate { at: f64, len: f64, delay: f64 },
    /// The daemon is descheduled once, inside its next call.
    Pause { at: f64, len: f64 },
    /// Volume writes fail; partly when `partial` (the new value lands).
    WriteFail { at: f64, len: f64, partial: bool },
    /// Minimum writes fail.
    MinFail { at: f64, len: f64 },
    /// The capture delivers nothing.
    SenseLoss { at: f64, len: f64 },
    /// One period is short (overrun).
    ShortPeriod { at: f64 },
    /// One position read fails.
    PositionFail { at: f64 },
    /// A pause between a boot-time read and the next monotonic read.
    ClockSplit { at: f64, len: f64, pause: f64 },
    /// Opening the capture blocks this long.
    SlowOpen { at: f64, delay: f64 },
    /// One position read is taken, then the call blocks this long.
    SlowPosition { at: f64, delay: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Damage {
    None,
    Truncate,
    OldFormat,
    Garbage,
    Remove,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Restart {
    pub at: f64,
    pub clean: bool,
    pub gap: f64,
    pub damage: Damage,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Scenario {
    pub horizon: f64,
    pub events: bool,
    /// The card rounds reductions up to 0.5 dB steps.
    pub steps: bool,
    /// The controls' value before the first run (dB).
    pub initial_db: f64,
    /// Start from a cold clean stop (else no state at all).
    pub clean_start: bool,
    pub max_reduction: f64,
    /// Change-event latency (s).
    pub latency: f64,
    pub streams: Vec<Stream>,
    pub faults: Vec<Fault>,
    pub restarts: Vec<Restart>,
}

impl Scenario {
    pub fn random(seed: u64) -> Scenario {
        let mut r = Rng::new(seed);
        let horizon = r.range(1.0, 4.0);
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
            // conversion to playback within the driver's 16384-frame queue
            let chunk = r.range(0.004, 0.128);
            streams.push(Stream { open: t, close: t + len, depth: r.range(0.0, 0.341 - chunk), chunk, levels });
            t += len + if r.chance(0.3) { r.range(0.0005, 0.03) } else { r.range(0.03, 0.8) };
        }
        let mut faults = Vec::new();
        let n = (r.f() * 6.0) as usize;
        for _ in 0..n {
            let at = r.range(0.0, horizon);
            faults.push(match (r.f() * 12.0) as u32 {
                0 => Fault::RateFail { at, len: r.range(0.01, 0.6) },
                1 => Fault::RateSlow { at, delay: r.range(0.01, 0.3) },
                2 => Fault::WakeLate { at, len: r.range(0.05, 1.0), delay: r.range(0.005, 0.1) },
                3 => Fault::Pause { at, len: r.range(0.05, 0.8) },
                4 => Fault::WriteFail { at, len: r.range(0.01, 0.4), partial: r.chance(0.5) },
                5 => Fault::MinFail { at, len: r.range(0.01, 0.4) },
                6 => Fault::SenseLoss { at, len: r.range(0.02, 0.9) },
                7 => Fault::ShortPeriod { at },
                8 => Fault::PositionFail { at },
                9 => Fault::ClockSplit { at, len: r.range(0.1, 1.0), pause: r.range(0.01, 1.0) },
                10 => Fault::SlowOpen { at, delay: r.range(0.05, 0.2) },
                _ => Fault::SlowPosition { at, delay: r.range(0.005, 0.3) },
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
        // the program changes on sample boundaries (the oracle integrates
        // per sample, the capture reads per frame)
        let grid = |x: f64| (x * HARNESS_RATE).round() / HARNESS_RATE;
        for s in streams.iter_mut() {
            s.open = grid(s.open);
            s.close = grid(s.close).max(s.open + 1.0 / HARNESS_RATE);
            for l in s.levels.iter_mut() {
                l.0 = grid(l.0);
            }
        }
        Scenario {
            horizon,
            events: !r.chance(0.08),
            steps: r.chance(0.7),
            initial_db: r.range(0.0, 30.0),
            clean_start: r.chance(0.8),
            max_reduction: if r.chance(0.5) { 20.0 } else { r.range(6.0, 30.0) },
            latency: r.range(0.0, 0.002),
            streams,
            faults,
            restarts,
        }
    }

    /// The scenario as Rust, to paste into a witness test.
    pub fn to_rust(&self) -> String {
        let f = |x: f64| format!("{x:?}");
        let streams: Vec<String> = self
            .streams
            .iter()
            .map(|s| {
                let levels: Vec<String> = s.levels.iter().map(|l| format!("({}, {})", f(l.0), f(l.1))).collect();
                format!("Stream {{ open: {}, close: {}, depth: {}, chunk: {}, levels: vec![{}] }}", f(s.open), f(s.close), f(s.depth), f(s.chunk), levels.join(", "))
            })
            .collect();
        let faults: Vec<String> = self.faults.iter().map(|x| format!("Fault::{x:?}")).collect();
        let restarts: Vec<String> = self.restarts.iter().map(|r| format!("Restart {{ at: {}, clean: {}, gap: {}, damage: Damage::{:?} }}", f(r.at), r.clean, f(r.gap), r.damage)).collect();
        format!(
            "Scenario {{ horizon: {}, events: {}, steps: {}, initial_db: {}, clean_start: {}, max_reduction: {}, latency: {}, streams: vec![{}], faults: vec![{}], restarts: vec![{}] }}",
            f(self.horizon),
            self.events,
            self.steps,
            f(self.initial_db),
            self.clean_start,
            f(self.max_reduction),
            f(self.latency),
            streams.join(", "),
            faults.join(", "),
            restarts.join(", ")
        )
    }

    fn open_at(&self, t: f64) -> Option<&Stream> {
        self.streams.iter().find(|s| s.open <= t && t < s.close)
    }

    /// Times the speakers open or close.
    fn edges(&self) -> Vec<f64> {
        let mut e: Vec<f64> = self.streams.iter().flat_map(|s| [s.open, s.close]).collect();
        e.sort_by(f64::total_cmp);
        e
    }
}

/// Stream-level facts the oracle works from: what played, before and after
/// the card's gain.
struct Oracle {
    full: Vec<f64>,
    trc: Vec<f64>,
    tau_c: Vec<f64>,
    trm: Vec<f64>,
    tau_m: Vec<f64>,
    ta: f64,
    tc: Vec<f64>,
    tm: Vec<f64>,
    /// Up to where (s, world time) it has integrated.
    to: f64,
    /// Per sample: coil temperature per speaker, and cumulative energy
    /// before the gain per speaker (J).
    coil: Vec<Vec<f32>>,
    energy: Vec<Vec<f64>>,
}

impl Oracle {
    fn new(cfg: &Config) -> Oracle {
        let n = cfg.speakers.len();
        Oracle {
            full: cfg.speakers.iter().map(|s| s.vs_scale * s.vs_scale / s.z_nominal).collect(),
            trc: cfg.speakers.iter().map(|s| s.tr_coil).collect(),
            tau_c: cfg.speakers.iter().map(|s| s.tau_coil).collect(),
            trm: cfg.speakers.iter().map(|s| s.tr_magnet).collect(),
            tau_m: cfg.speakers.iter().map(|s| s.tau_magnet).collect(),
            ta: cfg.globals.t_ambient,
            tc: vec![cfg.globals.t_ambient; n],
            tm: vec![cfg.globals.t_ambient; n],
            to: 0.0,
            coil: vec![Vec::new(); n],
            energy: vec![vec![0.0]; n],
        }
    }

    /// One sample of `h` seconds: delivered power `p` (W) per speaker and
    /// power before the gain `pre` (W).
    fn step(&mut self, h: f64, p: &[f64], pre: &[f64]) {
        for k in 0..self.tc.len() {
            // sub-stepped forward Euler of the two-node model
            let sub = 4;
            let d = h / sub as f64;
            for _ in 0..sub {
                let dm = (self.ta + p[k] * self.trm[k] - self.tm[k]) / self.tau_m[k];
                let dc = (self.tm[k] + p[k] * self.trc[k] - self.tc[k]) / self.tau_c[k];
                self.tm[k] += dm * d;
                self.tc[k] += dc * d;
            }
            self.coil[k].push(self.tc[k] as f32);
            let e = *self.energy[k].last().unwrap() + pre[k] * h;
            self.energy[k].push(e);
        }
        self.to += h;
    }

    fn coil_at(&self, k: usize, t: f64, rate: f64) -> f64 {
        let i = ((t * rate).floor() as usize).min(self.coil[k].len().saturating_sub(1));
        if self.coil[k].is_empty() { self.ta } else { self.coil[k][i] as f64 }
    }

    /// Mean power before the gain over the `w` seconds up to `t`.
    fn mean_pre(&self, k: usize, t: f64, w: f64, rate: f64) -> f64 {
        // the cumulative energy at `x`, linear inside a sample (whose power
        // is constant)
        let at = |x: f64| {
            let e = &self.energy[k];
            let pos = (x * rate).max(0.0);
            let i = (pos.floor() as usize).min(e.len() - 1);
            let next = e.get(i + 1).copied().unwrap_or(e[i]);
            e[i] + (pos - i as f64).clamp(0.0, 1.0) * (next - e[i])
        };
        (at(t) - at(t - w)) / w
    }
}

/// The card and the world around the daemon.
pub struct World<'a> {
    sc: &'a Scenario,
    cfg: &'a Config,
    rate: u32,
    t0: Instant,
    boot0: f64,
    t: Cell<f64>,
    /// A boot-time read inside a ClockSplit asks for a pause before the
    /// next monotonic read.
    split_pending: Cell<f64>,
    pauses_used: Vec<bool>,
    // the card
    control: f64,
    owner: bool,
    unlocked: bool,
    lease_left: f64,
    notice: bool,
    lease_t: f64,
    /// (from, effective reduction for new conversions).
    gains: Vec<(f64, f64)>,
    /// Streams converted before `.0` stop playing from `.1` on.
    stops: Vec<(f64, f64)>,
    // the capture
    sense_open: bool,
    next_end: f64,
    /// Where the frames last returned ended.
    period_frames_end: f64,
    events_seen: f64,
    edges: Vec<f64>,
    // the faults that fire once
    once_used: Vec<bool>,
    oracle: Oracle,
    /// When the last complete period reached the daemon.
    pub last_period: f64,
    pub rate_failed: bool,
    pub writes: u32,
    /// When the lease was last renewed.
    pub last_ping_at: f64,
    /// When a sample-rate read last first showed the speakers open.
    pub sighted: f64,
    last_rate: i64,
}

impl<'a> World<'a> {
    pub fn new(sc: &'a Scenario, cfg: &'a Config) -> World<'a> {
        let safe = cfg.globals.safe_reduction;
        World {
            sc,
            cfg,
            rate: HARNESS_RATE as u32,
            t0: Instant::now(),
            boot0: 1000.0,
            t: Cell::new(0.0),
            split_pending: Cell::new(0.0),
            pauses_used: vec![false; sc.faults.len()],
            control: sc.initial_db,
            owner: false,
            unlocked: false,
            lease_left: 0.0,
            notice: false,
            lease_t: 0.0,
            gains: vec![(0.0, sc.initial_db.max(safe))],
            stops: Vec::new(),
            sense_open: false,
            next_end: 0.0,
            period_frames_end: 0.0,
            events_seen: 0.0,
            edges: sc.edges(),
            once_used: vec![false; sc.faults.len()],
            oracle: Oracle::new(cfg),
            last_period: 0.0,
            rate_failed: false,
            writes: 0,
            last_ping_at: 0.0,
            sighted: 0.0,
            last_rate: 0,
        }
    }

    fn now_s(&self) -> f64 {
        self.t.get()
    }

    fn safe(&self) -> f64 {
        self.cfg.globals.safe_reduction
    }

    /// The reduction new conversions get now.
    fn effective(&self) -> f64 {
        if self.unlocked { self.control } else { self.control.max(self.safe()) }
    }

    fn note_gain(&mut self) {
        let e = self.effective();
        let t = self.now_s();
        if self.gains.last().is_none_or(|g| g.1 != e) {
            self.gains.push((t, e));
        }
    }

    /// The card's lock went away (given up, lapsed, or its owner died):
    /// locked at once, and the streams converted at the old gain stop
    /// within the drain.
    fn relock(&mut self, at: f64, by: f64) {
        self.unlocked = false;
        self.lease_left = 0.0;
        self.stops.push((at, at + by.clamp(0.0, DRAIN.as_secs_f64())));
        let e = self.effective();
        if self.gains.last().is_none_or(|g| g.1 != e) {
            self.gains.push((at, e));
        }
    }

    fn gain_at(&self, kappa: f64) -> f64 {
        let i = self.gains.partition_point(|g| g.0 <= kappa);
        if i == 0 { self.gains[0].1 } else { self.gains[i - 1].1 }
    }

    /// The amplitude speaker `k`'s sample at play time `tau` was delivered
    /// at (fraction of full scale), and the program's level before the gain.
    fn sample(&self, tau: f64) -> (f64, f64) {
        let Some(s) = self.sc.open_at(tau) else { return (0.0, 0.0) };
        let i = s.levels.partition_point(|l| l.0 <= tau);
        let level = if i == 0 { s.levels[0].1 } else { s.levels[i - 1].1 };
        let src = (tau - s.depth).max(s.open);
        let kappa = s.open + ((src - s.open) / s.chunk).floor() * s.chunk;
        if self.stops.iter().any(|&(at, from)| kappa < at && tau >= from) {
            // macaudio stopped it: nothing of it reached the amplifier
            return (0.0, 0.0);
        }
        (level * 10f64.powf(-self.gain_at(kappa) / 20.0), level)
    }

    fn word(&self, tau: f64, frame: usize) -> i32 {
        let a = self.sample(tau).0.clamp(0.0, 1.0);
        let w = (a * 2147483647.0) as i32;
        if frame % 2 == 0 { w } else { -w }
    }

    /// Bring the world to `to`: the lease in playing time, the oracle.
    fn advance(&mut self, to: f64) {
        let to = to.max(self.now_s());
        let mut t = self.lease_t;
        let step = 0.0005;
        while t < to {
            let b = (t + step).min(to);
            if self.unlocked && self.sc.open_at(t).is_some() {
                self.lease_left -= b - t;
                if self.lease_left <= 0.0 {
                    // macaudio's timeout: locked, the one-shot notice, and
                    // the streams at the old gain stopped
                    self.notice = true;
                    self.relock(b, DRAIN.as_secs_f64() * 0.9);
                }
            }
            t = b;
        }
        self.lease_t = to;
        self.t.set(to);
        self.integrate();
    }

    fn integrate(&mut self) {
        let rate = self.rate as f64;
        let h = 1.0 / rate;
        let n = self.cfg.speakers.len();
        while self.oracle.to + h <= self.now_s() {
            let tau = self.oracle.to;
            let (a, level) = self.sample(tau);
            let p: Vec<f64> = (0..n).map(|k| a * a * self.oracle.full[k]).collect();
            let pre: Vec<f64> = (0..n).map(|k| level * level * self.oracle.full[k]).collect();
            self.oracle.step(h, &p, &pre);
        }
    }

    fn fault_on(&self, f: impl Fn(&Fault) -> Option<(f64, f64)>) -> bool {
        let t = self.now_s();
        self.sc.faults.iter().any(|x| f(x).is_some_and(|(a, l)| t >= a && t < a + l))
    }

    /// A fault that fires once, at or after its time.
    fn once(&mut self, f: impl Fn(&Fault) -> Option<f64>) -> Option<usize> {
        let t = self.now_s();
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

    /// A descheduled daemon: a pause that is due lands in this call.
    fn maybe_pause(&mut self) {
        let t = self.now_s();
        for i in 0..self.sc.faults.len() {
            if let Fault::Pause { at, len } = self.sc.faults[i] {
                if !self.pauses_used[i] && t >= at {
                    self.pauses_used[i] = true;
                    self.advance(t + len);
                    return;
                }
            }
        }
    }

    /// The first change event delivered in (from, to].
    fn change_in(&self, from: f64, to: f64) -> Option<f64> {
        self.edges.iter().map(|e| e + self.sc.latency).find(|&e| e > from && e <= to)
    }

    /// The daemon process died or exited: the kernel takes the card.
    pub fn owner_gone(&mut self) {
        self.owner = false;
        self.notice = false;
        let t = self.now_s();
        self.advance(t);
        self.relock(t, DRAIN.as_secs_f64());
        self.sense_open = false;
    }
}

impl Hw for World<'_> {
    fn now(&self) -> Instant {
        let p = self.split_pending.replace(0.0);
        if p > 0.0 {
            // no lease or oracle work here (&self): the next call settles it
            self.t.set(self.t.get() + p);
        }
        self.t0 + Duration::from_secs_f64(self.t.get())
    }
    fn boottime(&self) -> Duration {
        let t = self.t.get();
        if let Some(&Fault::ClockSplit { pause, .. }) = self.sc.faults.iter().find(|f| matches!(f, Fault::ClockSplit { at, len, .. } if t >= *at && t < at + len)) {
            self.split_pending.set(pause);
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
        Ok(())
    }
    fn ping(&mut self) -> io::Result<()> {
        self.maybe_pause();
        let t = self.now_s();
        self.advance(t);
        if self.notice {
            self.notice = false;
            return Err(io::Error::from_raw_os_error(libc::ETIMEDOUT));
        }
        if !self.owner {
            return Err(io::Error::from_raw_os_error(libc::EPERM));
        }
        self.unlocked = true;
        self.lease_left = 0.25;
        self.last_ping_at = self.now_s();
        self.note_gain();
        Ok(())
    }
    fn step_db(&self) -> f64 {
        if self.sc.steps { 0.5 } else { 0.0 }
    }
    fn set_reduction(&mut self, db: f64) -> io::Result<f64> {
        self.maybe_pause();
        let t = self.now_s();
        self.advance(t);
        let applied = if self.sc.steps { (db * 2.0 - 1e-9).ceil() / 2.0 } else { db };
        if let Some(Fault::WriteFail { partial, .. }) = self.sc.faults.iter().find(|f| matches!(f, Fault::WriteFail { at, len, .. } if t >= *at && t < at + len)).copied() {
            if partial {
                self.control = applied;
                self.note_gain();
            }
            return Err(io::Error::from_raw_os_error(libc::EIO));
        }
        if !self.unlocked && db < self.safe() {
            return Err(io::Error::from_raw_os_error(libc::EINVAL));
        }
        self.control = applied;
        self.writes += 1;
        self.note_gain();
        Ok(applied)
    }
    fn set_min(&mut self) -> io::Result<()> {
        self.maybe_pause();
        let t = self.now_s();
        self.advance(t);
        if self.fault_on(|f| if let Fault::MinFail { at, len } = f { Some((*at, *len)) } else { None }) {
            return Err(io::Error::from_raw_os_error(libc::EIO));
        }
        self.control = MUTE_DB;
        self.note_gain();
        Ok(())
    }
    fn relinquish(&mut self) -> io::Result<()> {
        self.maybe_pause();
        let t = self.now_s();
        self.advance(t);
        self.notice = false;
        let t = self.now_s();
        self.relock(t, DRAIN.as_secs_f64() * 0.9);
        Ok(())
    }
    fn sample_rate(&mut self) -> io::Result<i64> {
        self.maybe_pause();
        let t = self.now_s();
        let mut delay = 0.0;
        if let Some(i) = self.once(|f| if let Fault::RateSlow { at, .. } = f { Some(*at) } else { None }) {
            if let Fault::RateSlow { delay: d, .. } = self.sc.faults[i] {
                delay = d;
            }
        }
        // the value the read returns is the state at some point inside it
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
        if let Some(i) = self.once(|f| if let Fault::SlowOpen { at, .. } = f { Some(*at) } else { None }) {
            if let Fault::SlowOpen { delay: d, .. } = self.sc.faults[i] {
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
        // late: the frames before the newest whole period are gone
        if t > self.next_end + self.period_s() {
            let behind = ((t - self.next_end) / self.period_s()).floor();
            self.next_end += behind * self.period_s();
        }
        let end = self.next_end;
        self.advance(t.max(end));
        self.next_end += self.period_s();
        if self.once(|f| if let Fault::ShortPeriod { at } = f { Some(*at) } else { None }).is_some() {
            return Ok(Some(false));
        }
        let p = self.cfg.globals.period;
        let ch = self.cfg.globals.channels;
        let rate = self.rate as f64;
        for i in 0..p {
            let tau = end - (p - i) as f64 / rate;
            let w = self.word(tau, i);
            for c in 0..ch {
                words[i * ch + c] = w;
            }
        }
        self.last_period = self.now_s();
        self.period_frames_end = end;
        if std::env::var("SPEAKERGUARDD_TRACE").is_ok() {
            eprintln!("WORLD period end {end:.7} now {:.7}", self.now_s());
        }
        Ok(Some(true))
    }
    fn queued(&mut self) -> io::Result<u64> {
        if self.once(|f| if let Fault::PositionFail { at } = f { Some(*at) } else { None }).is_some() {
            return Err(io::Error::from_raw_os_error(libc::EIO));
        }
        let q = ((self.now_s() - self.period_frames_end).max(0.0) * self.rate as f64) as u64;
        if let Some(i) = self.once(|f| if let Fault::SlowPosition { at, .. } = f { Some(*at) } else { None }) {
            if let Fault::SlowPosition { delay, .. } = self.sc.faults[i] {
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
        let rate = self.rate as f64;
        let p = self.cfg.globals.period;
        // whole frames: the edges are on the sample grid
        let k = (((stop - start) * rate).round().max(0.0) as usize).min(p - 1);
        let ch = self.cfg.globals.channels;
        for i in 0..k {
            let tau = start + i as f64 / rate;
            let w = self.word(tau, i);
            for c in 0..ch {
                words[i * ch + c] = w;
            }
        }
        self.period_frames_end = start + k as f64 / rate;
        if std::env::var("SPEAKERGUARDD_TRACE").is_ok() {
            eprintln!("WORLD tail start {start:.7} stop {stop:.7} k {k} now {t:.7}");
        }
        Ok(k)
    }
    fn wait_rate_change(&mut self, timeout: Duration) -> bool {
        self.maybe_pause();
        let t = self.now_s();
        let to = t + timeout.as_secs_f64();
        let late = self.sc.faults.iter().find_map(|f| match f {
            Fault::WakeLate { at, len, delay } if t >= *at && t < at + len => Some(*delay),
            _ => None,
        });
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

impl World<'_> {
    fn period_s(&self) -> f64 {
        self.cfg.globals.period as f64 / self.rate as f64
    }

    /// The next sample boundary at or after `t`.
    fn on_grid(&self, t: f64) -> f64 {
        (t * self.rate as f64).ceil() / self.rate as f64
    }
}

/// A violated property: what and when.
#[derive(Debug)]
pub struct Violation(pub String);

fn harness_cfg() -> Config {
    // shorter periods at a lower rate keep a case cheap; every rule the
    // guard keeps is per period or per second, not per frame count
    Config::parse(&J700.replace("period = 4096", "period = 512")).unwrap()
}

fn state_dir(tag: u64) -> PathBuf {
    use std::sync::atomic::{AtomicU64, Ordering};
    static N: AtomicU64 = AtomicU64::new(0);
    let dir = std::env::temp_dir().join(format!("speakerguardd-harness-{}-{tag}-{}", std::process::id(), N.fetch_add(1, Ordering::Relaxed)));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir.join("state")
}

fn damage(path: &PathBuf, how: Damage, rng: &mut Rng) {
    match how {
        Damage::None => {}
        Damage::Remove => {
            let _ = std::fs::remove_file(path);
        }
        Damage::Truncate => {
            // cut into the last line at least ("end\n" is 4 bytes)
            if let Ok(t) = std::fs::read(path) {
                let n = (rng.f() * t.len().saturating_sub(5) as f64) as usize;
                let _ = std::fs::write(path, &t[..n]);
            }
        }
        Damage::OldFormat => {
            if let Ok(t) = std::fs::read_to_string(path) {
                let v1: Vec<String> = t
                    .replace("speakerguardd-state 2", "speakerguardd-state 1")
                    .lines()
                    .filter(|l| !l.starts_with("covered_ns") && !l.starts_with("released") && *l != "end")
                    .map(|l| if l.starts_with("speaker ") { l.split_whitespace().take(8).collect::<Vec<_>>().join(" ") } else { l.to_string() })
                    .collect();
                let _ = std::fs::write(path, v1.join("\n"));
            }
        }
        Damage::Garbage => {
            let g: Vec<u8> = (0..64).map(|_| (rng.next() & 0xff) as u8).collect();
            let _ = std::fs::write(path, g);
        }
    }
}

/// The oracle's need (dB) at world time `t`: the governor's thermal ramp
/// on the true coil and both budgets on the true power before the gain,
/// without any release.
fn need(cfg: &Config, o: &Oracle, k: usize, t: f64, rate: f64) -> f64 {
    let s = &cfg.speakers[k];
    let g = &cfg.globals;
    let start = s.t_limit - s.t_headroom - g.t_window;
    let thermal = ((o.coil_at(k, t, rate) - start) / g.t_window * g.t_reduction_max).max(0.0);
    let mut budget: f64 = 0.0;
    for (w, limit) in [(1.0, s.p_limit_1s), (60.0, s.p_limit_60s)] {
        if limit > 0.0 {
            let m = o.mean_pre(k, t, w, rate);
            if m > limit {
                budget = budget.max(10.0 * (m / limit).log10());
            }
        }
    }
    thermal.max(budget)
}

/// Run one scenario; the first violation, if any.
pub fn run(sc: &Scenario, seed: u64) -> Result<(), Violation> {
    let cfg = harness_cfg();
    let path = state_dir(seed);
    if sc.clean_start {
        let cold: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), cfg.globals.t_ambient)).collect();
        std::fs::write(&path, state::encode(Duration::from_secs(1000), Duration::from_secs(1000), &cold, false, true)).unwrap();
    }
    let mut rng = Rng::new(seed ^ 0x5eed);
    let mut w = World::new(sc, &cfg);
    let rate = w.rate as f64;
    let mut restarts = sc.restarts.clone();
    restarts.sort_by(|a, b| a.at.total_cmp(&b.at));
    let mut next_restart = 0;
    let mut expect_latched = !sc.clean_start;
    let result = (|| -> Result<(), Violation> {
        loop {
            let mut g = match Guard::start(&mut w, &cfg, sc.max_reduction, Some(path.clone())) {
                Ok(g) => g,
                Err(e) => return Err(Violation(format!("start failed: {e}"))),
            };
            if expect_latched && !g.speakers.iter().all(|s| s.hard && s.t_coil >= s.spec.t_limit) {
                return Err(Violation(format!("at {:.4} s: a start from no clean stop was not latched", w.now_s())));
            }
            let stop_at = restarts.get(next_restart).map_or(f64::INFINITY, |r| r.at);
            let mut exited = false;
            loop {
                if w.now_s() >= sc.horizon {
                    return Ok(());
                }
                if w.now_s() >= stop_at {
                    break;
                }
                if g.tick(&mut w).is_err() {
                    // the daemon exited: as a crash, restarted by systemd
                    exited = true;
                    break;
                }
                // the world up to date (a pause at the end of the pass)
                let t = w.now_s();
                w.advance(t);
                if std::env::var("SPEAKERGUARDD_TRACE").is_ok() {
                    let at = (g.account_to() - w.t0).as_secs_f64();
                    eprintln!("TRACE now {:.6} at {:.6} armed {} unlocked {} eff {:.2} coil {:.3}/{:.3} m1 {:.5}/{:.5} hist {:?}", w.now_s(), at, g.armed(), w.unlocked, w.effective(), g.speakers[0].t_coil, w.oracle.coil_at(0, at, rate), g.speakers[0].mean_power(1.0), w.oracle.mean_pre(0, at, 1.0, rate), g.speakers[0].history_tail(8));
                }
                check(&cfg, &w, &g, sc, rate)?;
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
                    if std::env::var("SPEAKERGUARDD_TRACE").is_ok() {
                        let at = (g.account_to() - w.t0).as_secs_f64();
                        eprintln!("TRACE stop now {:.4} at {:.4} eff {:.2} m1 {:.4}/{:.4} hist {:?}", w.now_s(), at, w.effective(), g.speakers[0].mean_power(1.0), w.oracle.mean_pre(0, at, 1.0, rate), g.speakers[0].history_tail(4));
                    }
                    check(&cfg, &w, &g, sc, rate)?;
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
            if w.now_s() >= sc.horizon {
                return Ok(());
            }
        }
    })();
    let _ = std::fs::remove_dir_all(path.parent().unwrap());
    result
}

fn check(cfg: &Config, w: &World, g: &Guard, sc: &Scenario, rate: f64) -> Result<(), Violation> {
    let at = (g.account_to() - w.t0).as_secs_f64();
    let now = w.now_s();
    for k in 0..cfg.speakers.len() {
        // (a) the model is never cooler than the speaker, its budgets never
        // hold less energy before the gain
        let truth = w.oracle.coil_at(k, at, rate);
        if g.speakers[k].t_coil < truth - 0.02 {
            return Err(Violation(format!("(a) at {now:.4} s, accounted {at:.4} s: {} model {:.3} C, true coil {:.3} C", cfg.speakers[k].name, g.speakers[k].t_coil, truth)));
        }
        for win in [1.0, 60.0] {
            let m = g.speakers[k].mean_power(win);
            // The capture's position is whole frames, and the daemon places
            // the frames up to one before it: its energy can sit up to two
            // frames earlier than the truth's, which at a window's old edge
            // is that much leaving early.  The oracle's window starts two
            // frames later (the resolution of the measurement, not time).
            let edge = 2.0 / rate;
            let o = w.oracle.mean_pre(k, at, win - edge, rate) * (win - edge) / win;
            if m < o * (1.0 - 1e-3) - 1e-6 {
                return Err(Violation(format!("(a) at {now:.4} s, accounted {at:.4} s: {} {win} s budget {:.4} W before the gain, true {:.4} W", cfg.speakers[k].name, m, o)));
            }
        }
        // (b) while the daemon holds the lease, new conversions get at
        // least what the true state needs
        if g.armed() && w.unlocked {
            let need = need(cfg, &w.oracle, k, at, rate);
            if w.effective() < need - 0.01 && need < sc.max_reduction {
                return Err(Violation(format!("(b) at {now:.4} s: {} card applies {:.3} dB, the true state needs {:.3} dB", cfg.speakers[k].name, w.effective(), need)));
            }
            if need >= sc.max_reduction + 0.01 {
                return Err(Violation(format!("(b) at {now:.4} s: armed while the true state needs {need:.3} dB, over --max-reduction")));
            }
        }
    }
    // (c) what cannot be proven ends at the card's safe level (the lease
    // the daemon holds is what keeps its gain on the card)
    if g.armed() && w.unlocked {
        if !sc.events {
            return Err(Violation(format!("(c) at {now:.4} s: armed without change events")));
        }
        if w.rate_failed {
            return Err(Violation(format!("(c) at {now:.4} s: armed after a failed rate read")));
        }
        let period = cfg.globals.period as f64 / rate;
        // the lease was renewed while blind past the limit (what happens
        // after the last renewal is the lease's to bound)
        if sc.open_at(w.last_ping_at).is_some() && w.last_rate > 0 {
            // from when the daemon could know the stream: its sighting
            let since = w.last_ping_at - w.last_period.max(w.sighted);
            if since > BLIND_LIMIT.as_secs_f64() + period + 0.06 {
                return Err(Violation(format!("(c) at {now:.4} s: the lease renewed at {:.4} s, {since:.3} s without a complete period while the speakers play", w.last_ping_at)));
            }
        }
    }
    Ok(())
}

/// Shrink a failing scenario: drop streams, faults, restarts and level
/// changes one at a time, shorten the horizon, while it still fails.
pub fn shrink(mut sc: Scenario, seed: u64) -> Scenario {
    let fails = |s: &Scenario| run(s, seed).is_err();
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
        for i in (0..sc.streams.len()).rev() {
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
        let mut t = sc.clone();
        t.horizon *= 0.8;
        if t.horizon > 0.05 && fails(&t) {
            sc = t;
            progress = true;
        }
    }
    sc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cases() -> u64 {
        std::env::var("SPEAKERGUARDD_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(200)
    }

    fn first_seed() -> u64 {
        std::env::var("SPEAKERGUARDD_SEED").ok().and_then(|v| v.parse().ok()).unwrap_or(1)
    }

    #[test]
    fn trace_one() {
        let Ok(seed) = std::env::var("SPEAKERGUARDD_TRACE").map(|v| v.parse::<u64>().unwrap()) else { return };
        let sc = shrink(Scenario::random(seed), seed);
        eprintln!("{sc:?}");
        let r = run(&sc, seed);
        eprintln!("{r:?}");
    }

    /// The randomized interleavings (see the module's doc).
    #[test]
    fn random_interleavings_keep_the_properties() {
        let n = cases();
        let from = first_seed();
        let threads = std::thread::available_parallelism().map_or(1, |p| p.get()).min(32) as u64;
        let found: std::sync::Mutex<Option<(u64, String)>> = std::sync::Mutex::new(None);
        std::thread::scope(|s| {
            for th in 0..threads {
                let found = &found;
                s.spawn(move || {
                    let mut seed = from + th;
                    while seed < from + n {
                        if found.lock().unwrap().as_ref().is_some_and(|f| f.0 < seed) {
                            return;
                        }
                        let sc = Scenario::random(seed);
                        if let Err(v) = run(&sc, seed) {
                            let mut f = found.lock().unwrap();
                            if f.as_ref().is_none_or(|x| seed < x.0) {
                                *f = Some((seed, v.0));
                            }
                            return;
                        }
                        seed += threads;
                    }
                });
            }
        });
        if let Some((seed, why)) = found.into_inner().unwrap() {
            let small = shrink(Scenario::random(seed), seed);
            let after = run(&small, seed).err().map(|v| v.0).unwrap_or_default();
            panic!("seed {seed}: {why}\nshrunk: {after}\nWITNESS {}", small.to_rust());
        }
    }

    // ---- what the harness found (each a defect in 0.1.10's draft, put back
    // to check that its case still finds it) ----

    /// a muted card (the controls at their minimum) let nothing of the
    /// program through; 0.1.9 undid at most 30 dB of it, so the budgets lost
    /// the program
    #[test]
    fn h01_a_muted_cards_program_counts_up_to_full_scale() {
        let sc = Scenario { horizon: 0.8512992841882598, events: true, steps: true, initial_db: 5.069778520926467, clean_start: false, max_reduction: 20.0, latency: 0.0018184671294329293, streams: vec![Stream { open: 0.1786875, close: 0.6805, depth: 0.22262230157550328, chunk: 0.03503873415172144, levels: vec![(0.1786875, 0.7263536145167477)] }], faults: vec![], restarts: vec![] };
        if let Err(v) = run(&sc, 2) {
            panic!("{}", v.0);
        }
    }

    /// a budget window that cut a step took its energy pro rata, though the
    /// step's energy may all lie inside the window
    #[test]
    fn h02_a_window_edge_inside_a_step_keeps_its_energy() {
        let sc = Scenario { horizon: 2.2825362494252692, events: true, steps: true, initial_db: 23.94279022270308, clean_start: true, max_reduction: 28.66189827231709, latency: 0.000390993496961817, streams: vec![Stream { open: 0.02425, close: 1.246875, depth: 0.011040719404831987, chunk: 0.05484923872317915, levels: vec![(0.02425, 0.1295805609583719), (0.9309375, 0.820600623277012)] }], faults: vec![], restarts: vec![] };
        if let Err(v) = run(&sc, 90) {
            panic!("{}", v.0);
        }
    }

    /// a position read that blocked after taking the position placed the
    /// frames late, over time they did not see
    #[test]
    fn h03_a_pause_inside_the_position_read_is_charged() {
        let sc = Scenario { horizon: 2.3885480926654754, events: true, steps: true, initial_db: 21.498398967648416, clean_start: false, max_reduction: 20.0, latency: 0.0004522025101898588, streams: vec![Stream { open: 0.1785, close: 1.0409375, depth: 0.006633982013763149, chunk: 0.07409119427696939, levels: vec![(0.1785, 0.4511432842990031)] }, Stream { open: 1.11425, close: 1.998125, depth: 0.16514753344546107, chunk: 0.08442897332825991, levels: vec![(1.11425, 1.0)] }], faults: vec![Fault::SlowPosition { at: 1.0334075760348387, delay: 0.2791832557563448 }], restarts: vec![] };
        if let Err(v) = run(&sc, 9730) {
            panic!("{}", v.0);
        }
    }

    /// a lease that ran out while the daemon was descheduled was marked only
    /// on some hand-off paths, and frames after it were undone at the old
    /// gain
    #[test]
    fn h04_a_lease_that_lapsed_in_a_pause_counts_at_the_safe_level() {
        let sc = Scenario { horizon: 3.155570610102389, events: true, steps: false, initial_db: 26.10114978447397, clean_start: true, max_reduction: 20.0, latency: 0.0012320645760977352, streams: vec![Stream { open: 1.9656875, close: 3.1703125, depth: 0.042835273227909865, chunk: 0.12382195164106484, levels: vec![(1.9656875, 0.32633048955365695), (2.049, 1.0)] }], faults: vec![Fault::Pause { at: 2.51102217381004, len: 0.4713081063261734 }], restarts: vec![] };
        if let Err(v) = run(&sc, 491) {
            panic!("{}", v.0);
        }
    }

    /// a word is up to a step below what was played; undone by a deep gain,
    /// that step is most of it
    #[test]
    fn h05_a_quantized_word_is_undone_from_the_step_above() {
        let sc = Scenario { horizon: 0.0599409674611242, events: true, steps: true, initial_db: 21.957687476343008, clean_start: true, max_reduction: 20.0, latency: 0.001616603188832694, streams: vec![Stream { open: 0.0, close: 0.2429375, depth: 0.16837715593902544, chunk: 0.05718612666174021, levels: vec![(0.0, 0.6010230250311033)] }], faults: vec![], restarts: vec![] };
        if let Err(v) = run(&sc, 31207) {
            panic!("{}", v.0);
        }
    }

    /// a run that died near the coil's limit left its gain on the queued
    /// samples; a latched start at the limit was cooler than the coil after
    /// they played
    #[test]
    fn h06_a_latched_start_allows_for_the_drain() {
        let sc = Scenario { horizon: 1.251812419924582, events: true, steps: true, initial_db: 27.44081520087309, clean_start: true, max_reduction: 27.13686626220497, latency: 0.0016988581872146402, streams: vec![Stream { open: 0.1671875, close: 1.6003125, depth: 0.3230662247814654, chunk: 0.014692416538317876, levels: vec![(0.1671875, 1.0)] }], faults: vec![], restarts: vec![Restart { at: 0.9856943206086262, clean: false, gap: 0.010534984879947952, damage: Damage::Truncate }] };
        if let Err(v) = run(&sc, 7536) {
            panic!("{}", v.0);
        }
    }

    /// a latched start knew nothing of the minute before it, yet began with
    /// empty budgets
    #[test]
    fn h07_a_latched_start_fills_the_budgets() {
        let sc = Scenario { horizon: 2.0783673930377433, events: true, steps: false, initial_db: 9.731376993566748, clean_start: true, max_reduction: 20.335554254317596, latency: 0.000354247808518634, streams: vec![Stream { open: 0.1786875, close: 0.6805, depth: 0.22262230157550328, chunk: 0.03503873415172144, levels: vec![(0.1786875, 0.7263536145167477)] }], faults: vec![], restarts: vec![Restart { at: 1.2851157652684382, clean: true, gap: 0.04486077686697355, damage: Damage::Garbage }] };
        if let Err(v) = run(&sc, 2) {
            panic!("{}", v.0);
        }
    }

    /// after a failed (partial) write the controls may hold the new value,
    /// rounded up by the card's step, or the old one
    #[test]
    fn h08_a_failed_write_may_hold_either_value_rounded() {
        let sc = Scenario { horizon: 1.627405116610055, events: true, steps: true, initial_db: 28.237640200955045, clean_start: true, max_reduction: 20.380038427832506, latency: 0.0015341868532922792, streams: vec![Stream { open: 0.095375, close: 1.38825, depth: 0.21188516420537634, chunk: 0.10680010421938611, levels: vec![(0.095375, 1.0)] }], faults: vec![Fault::WriteFail { at: 0.777646723519306, len: 0.29781161818308416, partial: true }], restarts: vec![] };
        if let Err(v) = run(&sc, 1825) {
            panic!("{}", v.0);
        }
    }

    /// a pause between the restore's boot time and the account's start was
    /// never accounted
    #[test]
    fn h09_a_pause_between_the_restore_and_the_account_is_charged() {
        let sc = Scenario { horizon: 2.928420911226999, events: true, steps: false, initial_db: 4.888315429532138, clean_start: true, max_reduction: 23.234492511002504, latency: 0.0011127913864885102, streams: vec![Stream { open: 2.141625, close: 2.8581875, depth: 0.02145407892811289, chunk: 0.12186952711139563, levels: vec![(2.141625, 0.7958562745533154), (2.304625, 1.0)] }], faults: vec![Fault::ClockSplit { at: 1.9204718059173433, len: 0.8763733500041099, pause: 0.5508989092032914 }], restarts: vec![Restart { at: 1.341825145655577, clean: true, gap: 0.400185452348696, damage: Damage::None }] };
        if let Err(v) = run(&sc, 2390) {
            panic!("{}", v.0);
        }
    }

    /// a ping that blocked (or found the lease lapsed and took it again)
    /// renewed the lease for a model that fell behind in it
    #[test]
    fn h10_a_renewal_that_blocked_is_given_up() {
        let sc = Scenario { horizon: 0.3314836495643965, events: true, steps: false, initial_db: 11.943046727961843, clean_start: true, max_reduction: 20.0, latency: 0.0011068940999893085, streams: vec![Stream { open: 0.189375, close: 1.251, depth: 0.05990837127267815, chunk: 0.07367432026365783, levels: vec![(0.189375, 0.13390352805020111)] }], faults: vec![Fault::Pause { at: 0.30805992731269566, len: 0.6532891305737285 }], restarts: vec![] };
        if let Err(v) = run(&sc, 836) {
            panic!("{}", v.0);
        }
    }

    /// a look at closed speakers vouched for the time right up to it, though
    /// the event of a stream that had just opened was still on its way
    #[test]
    fn h11_a_quiet_look_leaves_room_for_an_event_in_flight() {
        let sc = Scenario { horizon: 1.2107584259764679, events: true, steps: false, initial_db: 22.304290374327, clean_start: true, max_reduction: 20.0, latency: 0.0011260319537750706, streams: vec![Stream { open: 0.081125, close: 0.1085, depth: 0.18839094734845602, chunk: 0.07371997855972491, levels: vec![(0.081125, 0.45867228920373126)] }, Stream { open: 0.819, close: 1.7278125, depth: 0.1305016407120471, chunk: 0.10246173918957761, levels: vec![(0.819, 0.9583760879040607)] }], faults: vec![Fault::RateSlow { at: 0.6470822909786041, delay: 0.16004630036156928 }], restarts: vec![] };
        if let Err(v) = run(&sc, 8186) {
            panic!("{}", v.0);
        }
    }

    /// a whole-frame position leaves the frames' place a frame open; the
    /// newest frame (silence) stood for audio
    #[test]
    fn h12_a_whole_frame_position_credits_only_the_frames_in_place() {
        let sc = Scenario { horizon: 0.05578727863485379, events: true, steps: true, initial_db: 21.88303195441253, clean_start: true, max_reduction: 12.551147664370191, latency: 0.00010863814179938114, streams: vec![Stream { open: 0.0, close: 0.0319375, depth: 0.07950821479397462, chunk: 0.014305498443731114, levels: vec![(0.0, 0.9181227951171709)] }], faults: vec![], restarts: vec![] };
        if let Err(v) = run(&sc, 182140) {
            panic!("{}", v.0);
        }
    }

    /// Found by a 2M-case run on #8 (seed 8100719359), and on 0.1.10 too:
    /// frames that did not split evenly into blocks lost energy in the
    /// longer ones, here a 1 s budget 0.5% under the truth (fleet's h18 on
    /// #7, with the cause)
    #[test]
    fn h14_uneven_blocks_keep_their_energy() {
        let sc = Scenario { horizon: 1.3191381306084726, events: true, steps: true, initial_db: 7.493479721540933, clean_start: true, max_reduction: 23.88707311652583, latency: 2.823637145963387e-5, streams: vec![Stream { open: 0.5955625, close: 0.6128125, depth: 0.010634848836597837, chunk: 0.015157138903645648, levels: vec![(0.5955625, 0.08864516066450345), (0.6115, 1.0)] }], faults: vec![Fault::Pause { at: 0.7913133366864289, len: 0.28896169769964836 }], restarts: vec![Restart { at: 1.1394017029306982, clean: true, gap: 0.033278459087805544, damage: Damage::OldFormat }] };
        if let Err(v) = run(&sc, 8100719359) {
            panic!("{}", v.0);
        }
    }

    /// The same at a stream's start (seed 8100823990): three levels in
    /// 37 ms, a 1 s budget 0.16% under the truth
    #[test]
    fn h15_uneven_blocks_at_a_streams_start() {
        let sc = Scenario { horizon: 0.05482742479889784, events: true, steps: true, initial_db: 3.404129695199801, clean_start: true, max_reduction: 25.617181783492246, latency: 0.0007715919483243825, streams: vec![Stream { open: 0.0, close: 0.037, depth: 0.1401714025960735, chunk: 0.06664888229065302, levels: vec![(0.0, 0.28306136900502155), (0.010125, 0.8684249612520664), (0.0281875, 0.061139625341538495)] }], faults: vec![], restarts: vec![] };
        if let Err(v) = run(&sc, 8100823990) {
            panic!("{}", v.0);
        }
    }

    /// Found on the V/I rework (#8): a clock read made for a recording that
    /// was not running held the daemon up 0.16 s after the stream's rate
    /// read, and the time without sense data was counted from after the
    /// hold-up, so the lease was renewed 0.596 s blind.
    #[test]
    fn h13_blindness_counts_from_the_rate_read() {
        let sc = Scenario { horizon: 0.8710052049715515, events: true, steps: false, initial_db: 21.736526505997787, clean_start: true, max_reduction: 23.671284306102798, latency: 0.0009549198328375215, streams: vec![Stream { open: 0.284625, close: 1.2036875, depth: 0.3192567226583039, chunk: 0.00866678128177472, levels: vec![(0.284625, 0.39076532479498227)] }], faults: vec![Fault::SenseLoss { at: 0.5332447001779974, len: 0.3978517580520215 }, Fault::ClockSplit { at: 0.09689160937261324, len: 0.35863305430364045, pause: 0.16035834711889616 }, Fault::SlowOpen { at: 0.35542582069194645, delay: 0.18545552738036591 }], restarts: vec![] };
        if let Err(v) = run(&sc, 387177) {
            panic!("{}", v.0);
        }
    }
}

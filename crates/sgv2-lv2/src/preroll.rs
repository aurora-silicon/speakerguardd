//! The first frames of a stream that wakes the speakers, held until the amps
//! play.
//!
//! With nothing playing the speaker path is powered down: the amps' PCM is
//! closed and the kernel holds them at its floor. A stream that starts then
//! runs into this plugin at once (the protect stage keeps running on the
//! audio server's own clock), but the amps only play once the supervisor has
//! granted, the session helper has linked them and their PCM has started,
//! tens of milliseconds later. macOS loses nothing there: a client's first
//! buffer is asked for only once the speaker engine streams, and it is the
//! first thing played (traces/audio/20261111-m1pro-macos-speaker-wake). This
//! does the same from inside the graph: while the supervisor says the amps
//! are not playing, the program is kept here, and the unit gets silence;
//! once they play, the kept frames go out first, in order.
//!
//! - Only a wake is kept: the hold starts from silence (a run of
//!   [`SILENT_RUN`] digital-zero frames, or nothing yet), as after the idle
//!   power-down. Leading silence is not kept. When the amps stop under a
//!   running program (a withdrawal, a fault, a sleep), nothing is kept: that
//!   program is lost as before and the kernel grant's ramp brings the next
//!   one in.
//! - At most [`CAPACITY_MS`] is kept; beyond it the oldest frames go (and
//!   are counted).
//! - The frames kept are latency for the rest of the stream until it falls
//!   silent: in a run of digital-zero frames past [`SILENT_RUN`], zero frames
//!   are dropped instead of queued, which shortens that silence and nothing
//!   else, until the queue is empty and the program passes straight again.
//!
//! Real time: the queue is allocated at instantiate; `process` only copies.

use crate::MAX_CHANNELS;

/// The most a wake keeps, ms: well above the wake itself (the supervisor's
/// grant, the helper's links and the PCM start).
pub const CAPACITY_MS: f64 = 500.0;
/// Consecutive digital-zero frames that count as silence: 1 ms at 48 kHz.
/// Shorter zero runs (a waveform crossing zero on every channel) are
/// program and always kept.
pub const SILENT_RUN: u32 = 48;

/// What the supervisor says about the amps (the engine page's `amps`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Amps {
    /// No word from a supervisor (none, an older one, or a stale word): pass
    /// the program straight, as before.
    Unknown,
    /// The amps are not playing yet: hold a wake.
    Held,
    /// The amps play.
    Playing,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// The program passes straight.
    Pass,
    /// The amps do not play and the program was running when they stopped:
    /// nothing is kept until it falls silent.
    Drop,
    /// Holding a wake: silence out, the program kept.
    Hold,
    /// The amps play: the kept frames go out first.
    Drain,
}

pub struct PreRoll {
    channels: usize,
    cap: usize,
    /// Interleaved frames, `cap * channels`.
    buf: Box<[f32]>,
    head: usize,
    len: usize,
    state: State,
    /// Digital-zero input frames in a row (saturating).
    zero_run: u32,
    /// Frames dropped because the queue was full, since instantiate.
    pub overflow: u64,
    /// Frames kept when the amps last started playing.
    pub last_wake: u32,
    /// Bumped on every wake that kept frames.
    pub wakes: u32,
}

impl PreRoll {
    /// Not real time.
    pub fn new(channels: usize, rate: f64) -> Self {
        let channels = channels.clamp(1, MAX_CHANNELS);
        let cap = ((rate * CAPACITY_MS / 1000.0).round() as usize).max(1);
        PreRoll {
            channels,
            cap,
            buf: vec![0.0; cap * channels].into_boxed_slice(),
            head: 0,
            len: 0,
            state: State::Pass,
            zero_run: SILENT_RUN,
            overflow: 0,
            last_wake: 0,
            wakes: 0,
        }
    }

    /// Frames kept now.
    pub fn queued(&self) -> usize {
        self.len
    }

    /// The histories are cleared (a stream starts): nothing kept.
    pub fn reset(&mut self) {
        self.head = 0;
        self.len = 0;
        self.state = State::Pass;
        self.zero_run = SILENT_RUN;
    }

    fn silent(&self) -> bool {
        self.zero_run >= SILENT_RUN
    }

    fn push(&mut self, frame: &[f32; MAX_CHANNELS]) {
        if self.len == self.cap {
            self.head = (self.head + 1) % self.cap;
            self.len -= 1;
            self.overflow += 1;
        }
        let at = (self.head + self.len) % self.cap * self.channels;
        self.buf[at..at + self.channels].copy_from_slice(&frame[..self.channels]);
        self.len += 1;
    }

    fn pop(&mut self, frame: &mut [f32; MAX_CHANNELS]) {
        let at = self.head * self.channels;
        frame[..self.channels].copy_from_slice(&self.buf[at..at + self.channels]);
        self.head = (self.head + 1) % self.cap;
        self.len -= 1;
    }

    /// Follow the supervisor's word at the start of a call.
    fn follow(&mut self, amps: Amps) {
        self.state = match (self.state, amps) {
            (_, Amps::Unknown) => {
                // no supervisor to say when the amps play: what is kept is
                // played out (never lost), then the program passes
                if self.len > 0 {
                    State::Drain
                } else {
                    State::Pass
                }
            }
            (State::Pass, Amps::Held) => {
                if self.silent() {
                    State::Hold
                } else {
                    State::Drop
                }
            }
            (State::Drop, Amps::Held) => {
                if self.silent() {
                    State::Hold
                } else {
                    State::Drop
                }
            }
            (State::Hold, Amps::Held) => State::Hold,
            (State::Drain, Amps::Held) => {
                // stopped while the kept frames were still going out: the
                // program was running, so this is no wake
                self.head = 0;
                self.len = 0;
                State::Drop
            }
            (State::Hold, Amps::Playing) => {
                if self.len > 0 {
                    self.last_wake = self.len as u32;
                    self.wakes = self.wakes.wrapping_add(1);
                    State::Drain
                } else {
                    State::Pass
                }
            }
            (State::Drain, Amps::Playing) => State::Drain,
            (State::Pass | State::Drop, Amps::Playing) => State::Pass,
        };
    }

    /// Run `frames` frames of `bufs` (one slice per channel) in place.
    pub fn process(&mut self, bufs: &mut [&mut [f32]], frames: usize, amps: Amps) {
        self.follow(amps);
        if self.state == State::Pass && self.len == 0 {
            // the common case: only follow the silence
            for k in 0..frames {
                self.note(bufs, k);
            }
            return;
        }
        let n = self.channels.min(bufs.len());
        if self.state == State::Hold
            && self.len == 0
            && bufs.iter().take(n).all(|b| b[..frames].iter().all(|&x| x == 0.0))
        {
            // idle: silence in, nothing kept, silence out (as it came)
            self.zero_run = self.zero_run.saturating_add(frames as u32);
            return;
        }
        let mut frame = [0.0f32; MAX_CHANNELS];
        for k in 0..frames {
            for (c, b) in bufs.iter().enumerate().take(n) {
                frame[c] = b[k];
            }
            let zero = frame[..n].iter().all(|&x| x == 0.0);
            self.zero_run = if zero { self.zero_run.saturating_add(1) } else { 0 };
            let mut out = [0.0f32; MAX_CHANNELS];
            match self.state {
                State::Pass => out = frame,
                State::Drop => {}
                State::Hold => {
                    // leading silence is not kept
                    if !(zero && self.len == 0) {
                        self.push(&frame);
                    }
                }
                State::Drain => {
                    // the oldest out first (a full queue drops nothing here)
                    if self.len > 0 {
                        self.pop(&mut out);
                    }
                    // in a silence, zero frames are dropped, not queued
                    if !(zero && self.silent()) {
                        self.push(&frame);
                    }
                    if self.len == 0 {
                        self.state = State::Pass;
                    }
                }
            }
            for (c, b) in bufs.iter_mut().enumerate().take(n) {
                b[k] = out[c];
            }
        }
    }

    #[inline]
    fn note(&mut self, bufs: &[&mut [f32]], k: usize) {
        let zero = bufs.iter().take(self.channels).all(|b| b[k] == 0.0);
        self.zero_run = if zero { self.zero_run.saturating_add(1) } else { 0 };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 48000.0;

    /// One call of `frames` frames of channel-0 values `f(i)` (other
    /// channels the negation), starting at absolute frame `at`.
    fn call(p: &mut PreRoll, amps: Amps, at: usize, frames: usize, f: &dyn Fn(usize) -> f32) -> Vec<f32> {
        let mut a: Vec<f32> = (0..frames).map(|k| f(at + k)).collect();
        let mut b: Vec<f32> = a.iter().map(|x| -x).collect();
        {
            let mut bufs: [&mut [f32]; 2] = [&mut a, &mut b];
            p.process(&mut bufs, frames, amps);
        }
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(*x, -*y, "channels stay together");
        }
        a
    }

    /// A program: silence until `start`, then frames numbered 1, 2, ...
    fn numbered(start: usize) -> impl Fn(usize) -> f32 {
        move |i| if i < start { 0.0 } else { (i - start + 1) as f32 }
    }

    #[test]
    fn passes_straight_without_a_supervisor() {
        let mut p = PreRoll::new(2, RATE);
        let f = numbered(100);
        let mut i = 0;
        for _ in 0..10 {
            let out = call(&mut p, Amps::Unknown, i, 512, &f);
            assert_eq!(out, (i..i + 512).map(&f).collect::<Vec<_>>());
            i += 512;
        }
        assert_eq!(p.queued(), 0);
    }

    #[test]
    fn a_wake_loses_nothing_and_plays_in_order() {
        let mut p = PreRoll::new(2, RATE);
        let f = numbered(700);
        let mut out = Vec::new();
        let mut i = 0;
        // idle: held, silence
        for _ in 0..4 {
            out.extend(call(&mut p, Amps::Held, i, 512, &f));
            i += 512;
        }
        // the program started at 700; the amps play from frame 2048
        assert!(out.iter().all(|&x| x == 0.0), "silence while held");
        assert_eq!(p.queued(), 2048 - 700);
        for _ in 0..8 {
            out.extend(call(&mut p, Amps::Playing, i, 512, &f));
            i += 512;
        }
        // from the first frame the amps play: 1, 2, 3, ... with no gap
        let played: Vec<f32> = out[2048..].to_vec();
        let want: Vec<f32> = (1..=played.len()).map(|n| n as f32).collect();
        assert_eq!(played, want);
        assert_eq!(p.overflow, 0);
        assert_eq!(p.last_wake, (2048 - 700) as u32);
        assert_eq!(p.wakes, 1);
    }

    #[test]
    fn the_kept_frames_drain_in_the_next_silence_only() {
        let mut p = PreRoll::new(2, RATE);
        // 3000 frames of program from 1000, then silence
        let f = |i: usize| if (1000..4000).contains(&i) { (i - 999) as f32 } else { 0.0 };
        let mut out = Vec::new();
        let mut i = 0;
        for _ in 0..3 {
            out.extend(call(&mut p, Amps::Held, i, 512, &f));
            i += 512;
        }
        let kept = p.queued();
        assert_eq!(kept, 1536 - 1000);
        for _ in 0..20 {
            out.extend(call(&mut p, Amps::Playing, i, 512, &f));
            i += 512;
        }
        // the whole program, once, in order
        let nz: Vec<f32> = out.iter().copied().filter(|&x| x != 0.0).collect();
        assert_eq!(nz, (1..=3000).map(|n| n as f32).collect::<Vec<_>>());
        // it ended `kept` frames late, then the silence took the queue back
        let first = out.iter().position(|&x| x != 0.0).unwrap();
        assert_eq!(first, 1536);
        assert_eq!(p.queued(), 0);
        // and the program passes straight again
        let g = |i: usize| if i >= 20000 { 1.0 } else { 0.0 };
        let o = call(&mut p, Amps::Playing, 20000, 64, &g);
        assert!(o.iter().all(|&x| x == 1.0));
    }

    #[test]
    fn a_short_zero_run_inside_the_program_is_kept() {
        let mut p = PreRoll::new(2, RATE);
        // program with 10-frame zero runs every 100 frames
        let f = |i: usize| if i < 600 || i % 100 < 10 { 0.0 } else { 1.0 + (i % 7) as f32 };
        let mut out = Vec::new();
        let mut i = 0;
        for _ in 0..2 {
            out.extend(call(&mut p, Amps::Held, i, 512, &f));
            i += 512;
        }
        for _ in 0..6 {
            out.extend(call(&mut p, Amps::Playing, i, 512, &f));
            i += 512;
        }
        // output from the first program frame equals the program from 600 on
        let first = out.iter().position(|&x| x != 0.0).unwrap();
        let shift = first - 610;
        for (k, &y) in out.iter().enumerate().skip(first) {
            assert_eq!(y, f(k - shift), "frame {k}");
        }
    }

    #[test]
    fn a_stop_under_a_running_program_keeps_nothing() {
        let mut p = PreRoll::new(2, RATE);
        let f = |_: usize| 0.5f32;
        let mut i = 0;
        for _ in 0..4 {
            call(&mut p, Amps::Playing, i, 512, &f);
            i += 512;
        }
        // withdrawn mid-program: dropped, not kept
        let out = call(&mut p, Amps::Held, i, 512, &f);
        i += 512;
        assert!(out.iter().all(|&x| x == 0.0));
        assert_eq!(p.queued(), 0);
        // back: straight on, no stale frames
        let out = call(&mut p, Amps::Playing, i, 512, &f);
        assert!(out.iter().all(|&x| x == 0.5));
        assert_eq!(p.wakes, 0);
    }

    #[test]
    fn a_drop_turns_into_a_hold_once_the_program_falls_silent() {
        let mut p = PreRoll::new(2, RATE);
        let g = |i: usize| {
            if i < 1024 {
                0.25
            } else if i < 2048 {
                0.0
            } else {
                0.75
            }
        };
        let mut i = 0;
        for _ in 0..2 {
            call(&mut p, Amps::Playing, i, 512, &g);
            i += 512;
        }
        // withdrawn while playing: Drop; silence follows; then a new sound
        let mut out = Vec::new();
        for _ in 0..4 {
            out.extend(call(&mut p, Amps::Held, i, 512, &g));
            i += 512;
        }
        assert!(out.iter().all(|&x| x == 0.0));
        // the new sound (from 2048) is kept as a wake
        assert_eq!(p.queued(), 3072 - 2048);
        let out = call(&mut p, Amps::Playing, i, 512, &g);
        assert!(out.iter().all(|&x| x == 0.75));
        assert_eq!(p.wakes, 1);
    }

    #[test]
    fn a_long_hold_keeps_the_latest_capacity_and_counts_the_rest() {
        let mut p = PreRoll::new(2, RATE);
        let cap = (RATE * CAPACITY_MS / 1000.0) as usize;
        let f = |i: usize| (i + 1) as f32;
        let mut i = 0;
        // the program starts with the hold (leading frame non-zero) and runs
        while i < cap + 1000 {
            call(&mut p, Amps::Held, i, 500, &f);
            i += 500;
        }
        assert_eq!(p.queued(), cap);
        assert_eq!(p.overflow, (i - cap) as u64);
        let out = call(&mut p, Amps::Playing, i, 500, &f);
        assert_eq!(out[0], (i - cap + 1) as f32);
    }

    #[test]
    fn losing_the_supervisor_plays_out_what_was_kept() {
        let mut p = PreRoll::new(2, RATE);
        let f = numbered(0);
        call(&mut p, Amps::Held, 0, 512, &f);
        assert_eq!(p.queued(), 512);
        let out = call(&mut p, Amps::Unknown, 512, 512, &f);
        assert_eq!(out, (1..=512).map(|n| n as f32).collect::<Vec<_>>());
    }
}

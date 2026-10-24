// SPDX-License-Identifier: MIT
//! The protection model: the power delivered to each voice coil is estimated
//! from the words on the speaker wire (feed-forward: V^2 against the coil's
//! DC resistance `r_ff`, which is `z_nominal` unless the configuration
//! names it), fed into a two-stage lumped thermal model (coil into magnet,
//! magnet into ambient, each a first-order lag), and the coil temperature
//! drives a gain governor with a window below the limit and hysteresis.
//! A windowed power budget (mean power over the last second and the last
//! minute) reduces the gain at once when a burst would exceed what the
//! amplifier/speaker pair is rated for, before the coil has warmed up.
//!
//! The model is integrated in steps of at most MAX_SUB_DT, and a sense
//! period in SUB_BLOCKS blocks of its own power, so that a peak inside a
//! period is seen: each step keeps an upper bound of the coil temperature
//! over its span, the governor works from it, and a bound at the coil's
//! limit latches a hard fault that only the daemon clears.

use crate::config::{Globals, Speaker};

/// Thermal state of one driver.
#[derive(Debug, Clone)]
pub struct SpeakerState {
    pub spec: Speaker,
    /// Voice coil and magnet temperatures (°C).
    pub t_coil: f64,
    pub t_magnet: f64,
    /// Power of the last step (W) and its RMS voltage (V), for logging.
    pub power: f64,
    pub v_rms: f64,
    /// Upper bound of the coil temperature over the last step (°C).
    pub t_peak: f64,
    /// Latched once the coil may have reached its limit (`t_limit`); the
    /// daemon hands the card to the kernel and clears it once the coil is
    /// back under its working ceiling (`clear_hard_if_cool`).
    pub hard: bool,
    /// Gain reduction applied by the governor (dB, >= 0).
    pub reduction_db: f64,
    reducing: bool,
    /// Power history for the budgets: (seconds, watts before the card's
    /// gain) per step, newest last.
    history: std::collections::VecDeque<(f64, f64)>,
    history_s: f64,
}

/// How long samples converted at an earlier gain can keep playing (s): the
/// guard's QUEUE_BOUND.
pub const DRAIN_S: f64 = 0.5;
/// Longest single integration step (s).
pub const MAX_SUB_DT: f64 = 0.01;
/// A sense period is integrated in this many blocks of its own power.
pub const SUB_BLOCKS: usize = 16;

impl SpeakerState {
    pub fn new(spec: Speaker, t_ambient: f64) -> SpeakerState {
        SpeakerState {
            spec,
            t_coil: t_ambient,
            t_magnet: t_ambient,
            power: 0.0,
            v_rms: 0.0,
            t_peak: t_ambient,
            hard: false,
            reduction_db: 0.0,
            reducing: false,
            history: std::collections::VecDeque::new(),
            history_s: 0.0,
        }
    }

    /// The RMS voltage a period of sense words puts across the coil.
    #[cfg(test)]
    pub fn v_rms_of(&self, words: &[i32], channels: usize) -> f64 {
        let ch = self.spec.vs_chan;
        let frames = words.len() / channels;
        if frames == 0 {
            return 0.0;
        }
        let scale = self.spec.vs_scale / 2147483648.0;
        let sum: f64 = words
            .chunks_exact(channels)
            .map(|f| {
                let v = f[ch] as f64 * scale;
                v * v
            })
            .sum();
        (sum / frames as f64).sqrt()
    }

    /// The power of each of `blocks` equal blocks of a sense period (W),
    /// and the period's RMS voltage (the tests' reference; the guard
    /// converts frame by frame).
    #[cfg(test)]
    pub fn block_powers(&self, words: &[i32], channels: usize, blocks: usize) -> (Vec<f64>, f64) {
        let ch = self.spec.vs_chan;
        let frames = words.len() / channels;
        let blocks = blocks.clamp(1, frames.max(1));
        let scale = self.spec.vs_scale / 2147483648.0;
        let mut out = Vec::with_capacity(blocks);
        let mut total = 0.0;
        for b in 0..blocks {
            let (lo, hi) = (frames * b / blocks, frames * (b + 1) / blocks);
            let sum: f64 = (lo..hi)
                .map(|f| {
                    let v = words[f * channels + ch] as f64 * scale;
                    v * v
                })
                .sum();
            total += sum;
            out.push(if hi > lo { sum / (hi - lo) as f64 / self.spec.r_ff } else { 0.0 });
        }
        let v_rms = if frames > 0 { (total / frames as f64).sqrt() } else { 0.0 };
        (out, v_rms)
    }

    /// Each frame's power (W) and its power before the card's gain: each
    /// frame inverted on its own at the most of the `possible` reductions
    /// (ranges, dB) that can have delivered it, never more than a
    /// full-scale signal.  A block can hold frames converted at different
    /// gains (review r5 V5), a frame only one.  A frame converted at
    /// reduction r was at most vs_scale 10^(-r/20) and its word is at most
    /// what was played: r fits iff r <= 20 log10(vs_scale / word).  What
    /// was played is within a step above the word: the step up is its
    /// power's bound (it matters only where the gain undone is deep).
    pub fn frame_powers(&self, words: &[i32], channels: usize, possible: &[(f64, f64)]) -> (Vec<f64>, Vec<f64>) {
        let ch = self.spec.vs_chan;
        let frames = words.len() / channels;
        let scale = self.spec.vs_scale / 2147483648.0;
        let full = self.worst_case_power(0.0);
        let mut p = Vec::with_capacity(frames);
        let mut q = Vec::with_capacity(frames);
        for f in 0..frames {
            let w = words[f * channels + ch] as f64;
            let v = w * scale;
            p.push(v * v / self.spec.z_nominal);
            let w = w.abs();
            let room = if w > 0.0 { 20.0 * (2147483648.0 / w).log10() } else { f64::INFINITY };
            let up = (w + 1.0) * scale;
            let pre = up * up / self.spec.z_nominal * 10f64.powf(converted_at(room, possible) / 10.0);
            q.push(if pre > full { full } else { pre });
        }
        (p, q)
    }

    /// Advance the thermal model by `dt` seconds with `power` watts in the
    /// coil, the card at full gain.
    pub fn step(&mut self, power: f64, dt: f64, t_ambient: f64) {
        self.step_at(power, dt, t_ambient, 0.0)
    }

    /// Advance the thermal model by `dt` seconds with `power` watts in the
    /// coil while the card applied `applied_db` of reduction.  The coil
    /// heats with the power it got; the budgets keep the power before the
    /// card's gain, so that the reduction they ask for is the whole one
    /// that brings the mean to the budget, not only what is left over the
    /// reduction already applied.
    pub fn step_at(&mut self, power: f64, dt: f64, t_ambient: f64, applied_db: f64) {
        self.power = power;
        self.record(power, dt, applied_db);
        self.t_peak = f64::NEG_INFINITY;
        self.heat(power, dt, t_ambient);
    }

    /// `dt` seconds with `power` watts in the coil and `pre` watts before
    /// the card's gain (for the budgets).
    pub fn step_pre(&mut self, power: f64, pre: f64, dt: f64, t_ambient: f64) {
        self.power = power;
        self.record_pre(pre, dt);
        self.t_peak = f64::NEG_INFINITY;
        self.heat(power, dt, t_ambient);
    }

    /// A sense period of `dt` seconds whose equal blocks carried `powers`
    /// watts each: the budgets get the period's mean, the coil each block.
    #[cfg(test)]
    pub fn step_period(&mut self, powers: &[f64], dt: f64, t_ambient: f64, applied_db: f64) {
        let g = 10f64.powf(applied_db.max(0.0) / 10.0);
        let pre: Vec<f64> = powers.iter().map(|p| p * g).collect();
        self.step_period_pre(powers, &pre, dt, t_ambient)
    }

    /// The same with each block's power before the card's gain.
    pub fn step_period_pre(&mut self, powers: &[f64], pre: &[f64], dt: f64, t_ambient: f64) {
        let n = powers.len().max(1);
        let mean = powers.iter().sum::<f64>() / n as f64;
        self.power = mean;
        self.record_pre(pre.iter().sum::<f64>() / n as f64, dt);
        self.t_peak = f64::NEG_INFINITY;
        if powers.is_empty() {
            self.heat(0.0, dt, t_ambient);
        }
        for &p in powers {
            self.heat(p, dt / n as f64, t_ambient);
        }
    }

    /// Keep one minute of the power before the card's gain for the budgets.
    fn record(&mut self, power: f64, dt: f64, applied_db: f64) {
        // a gain deeper than the words can say anything about (a muted
        // card) leaves full scale, which the cap is
        self.record_pre(power * 10f64.powf(applied_db.max(0.0) / 10.0), dt)
    }

    /// The same with the power before the gain given: never more than a
    /// full-scale signal (nothing before the gain is louder).
    fn record_pre(&mut self, pre: f64, dt: f64) {
        if !(dt > 0.0) {
            return;
        }
        let full = self.worst_case_power(0.0);
        let pre = if pre > full { full } else { pre };
        self.history.push_back((dt, pre));
        self.history_s += dt;
        // drop what is older than a minute: whole entries, then the expired
        // part of the oldest one, whose energy may all lie in the rest (as
        // much of it as full scale fits there)
        while self.history_s > 60.0 {
            let excess = self.history_s - 60.0;
            match self.history.front_mut() {
                Some(front) if front.0 <= excess => {
                    self.history_s -= front.0;
                    self.history.pop_front();
                }
                Some(front) => {
                    let energy = front.0 * front.1;
                    front.0 -= excess;
                    front.1 = (energy.min(full * front.0) / front.0).max(front.1);
                    self.history_s = 60.0;
                }
                None => {
                    self.history_s = 0.0;
                    break;
                }
            }
        }
    }

    /// `dt` seconds at `power` watts, in steps of at most MAX_SUB_DT.  After
    /// each step the coil's bound over it goes into `t_peak`: the step's end
    /// value, plus what a power that varied inside the step (its mean is
    /// what the step used) could have hidden.  From the hottest moment to
    /// the end of a step of length h the coil cools at most toward the
    /// magnet with its time constant, so it was at most
    /// (end - magnet) (e^(h/tau) - 1) above the end; and a step that put
    /// all of its energy at the very end ends at most
    /// P tr (h/tau - (1 - e^(-h/tau))) above the constant-power step.
    fn heat(&mut self, power: f64, dt: f64, t_ambient: f64) {
        if !(dt > 0.0) {
            self.t_peak = self.t_peak.max(self.t_coil);
            return;
        }
        let s = &self.spec;
        let n = (dt / MAX_SUB_DT).ceil().max(1.0);
        let n = if n.is_finite() { n as u64 } else { 1 };
        let h = dt / n as f64;
        let k_magnet = 1.0 - (-h / s.tau_magnet).exp();
        let k_coil = 1.0 - (-h / s.tau_coil).exp();
        let grow = (h / s.tau_coil).exp() - 1.0;
        let late = power * s.tr_coil * (h / s.tau_coil - k_coil).max(0.0);
        for _ in 0..n {
            // magnet: first-order lag toward ambient + P * tr_magnet
            self.t_magnet += (t_ambient + power * s.tr_magnet - self.t_magnet) * k_magnet;
            // coil: first-order lag toward magnet + P * tr_coil
            self.t_coil += (self.t_magnet + power * s.tr_coil - self.t_coil) * k_coil;
            let end = self.t_coil + late;
            let bound = end + (end - self.t_magnet).max(0.0) * grow;
            if !(bound <= self.t_peak) {
                // also takes a NaN, which finite() then rejects
                self.t_peak = bound;
            }
        }
        if !(self.t_peak < s.t_limit) {
            self.hard = true;
        }
    }

    /// Clear a latched hard fault once the coil is back under its working
    /// ceiling.  True if one was cleared.
    pub fn clear_hard_if_cool(&mut self) -> bool {
        let ceiling = self.spec.t_limit - self.spec.t_headroom;
        if self.hard && self.t_peak.max(self.t_coil) < ceiling {
            self.hard = false;
            return true;
        }
        false
    }

    /// The most power the coil can take while the card's gain is
    /// `-reduction_db`: a full-scale square wave (V rms = V peak) into the
    /// coil resistance.  Time the daemon could not measure is charged at it.
    pub fn worst_case_power(&self, reduction_db: f64) -> f64 {
        let v = self.spec.vs_scale * 10f64.powf(-reduction_db.max(0.0) / 20.0);
        v * v / self.spec.r_ff
    }

    /// Start at the coil's limit with the hard fault latched, the magnet
    /// where steady state would put it: nothing about the speaker's past is
    /// known (no state of this daemon, an unreadable one).  The card stays
    /// with the kernel until the model has cooled under the working ceiling.
    pub fn assume_latched(&mut self, t_ambient: f64) {
        let s = &self.spec;
        // A run that kept the coil under its limit may have died with its
        // gain on the samples still queued: they play out for up to the
        // drain at full scale (the hottest it can then be).
        let drain = self.worst_case_power(0.0) * s.tr_coil * (1.0 - (-DRAIN_S / s.tau_coil).exp());
        self.t_coil = s.t_limit + drain;
        self.t_magnet = t_ambient + (self.t_coil - t_ambient) * s.tr_magnet / (s.tr_magnet + s.tr_coil);
        self.t_peak = self.t_coil;
        self.hard = true;
        // nor is the power before the gain: a minute of full scale for the
        // budgets (they ask for their most until the minute has passed)
        let full = self.worst_case_power(0.0);
        self.history.clear();
        self.history.push_back((60.0, full));
        self.history_s = 60.0;
    }

    /// The steady state at `power` watts, as if it had run for ever; the
    /// budgets see it as a minute at `pre` watts before the gain.
    pub fn settle_at(&mut self, power: f64, pre: f64, t_ambient: f64) {
        let s = &self.spec;
        self.t_magnet = t_ambient + power * s.tr_magnet;
        self.t_coil = self.t_magnet + power * s.tr_coil;
        self.t_peak = self.t_coil;
        let full = self.worst_case_power(0.0);
        self.history.clear();
        self.history.push_back((60.0, if pre > full { full } else { pre }));
        self.history_s = 60.0;
    }

    /// Continue from a saved state: temperatures and the power history's
    /// last second and minute.
    pub fn resume(&mut self, t_coil: f64, t_magnet: f64, mean_1s: f64, mean_60s: f64) {
        self.t_coil = t_coil;
        self.t_magnet = t_magnet;
        self.t_peak = t_coil;
        self.hard = false;
        // two means do not say when the energy came (budget_envelope)
        (self.history, self.history_s) = budget_envelope(mean_1s, mean_60s, self.worst_case_power(0.0));
    }

    /// The governor's state a restart keeps: the latched fault, the peak
    /// that set it and the reduction being released.  A state without them
    /// (an older file) is latched whenever its coil is not under the
    /// working ceiling, where a latch could still have been set.
    pub fn resume_governor(&mut self, hard: Option<bool>, t_peak: f64, reduction_db: f64) {
        let ceiling = self.spec.t_limit - self.spec.t_headroom;
        self.t_peak = t_peak.max(self.t_coil);
        self.hard = hard.unwrap_or(!(self.t_coil < ceiling));
        self.reduction_db = reduction_db.max(0.0);
        self.reducing = self.reduction_db > 0.0;
    }

    /// Mean power before the card's gain over the last `seconds` (W); time
    /// before the first step counts as silence.  A step the window only
    /// partly covers may have put all of its energy inside it (as much of
    /// it as full scale fits there): the mean is an upper bound, whatever
    /// the energy's place inside each step.
    pub fn mean_power(&self, seconds: f64) -> f64 {
        let full = self.worst_case_power(0.0);
        let mut t = 0.0;
        let mut e = 0.0;
        for (dt, p) in self.history.iter().rev() {
            if t >= seconds {
                break;
            }
            let take = dt.min(seconds - t);
            e += if take < *dt { (p * dt).min(full * take).max(p * take) } else { p * dt };
            t += take;
        }
        e / seconds
    }

    /// The reduction (dB) the power budgets ask for: what brings the mean
    /// power the speaker gets over each window to its limit.
    fn budget_reduction(&self) -> f64 {
        let mut db: f64 = 0.0;
        for (seconds, limit) in [(1.0, self.spec.p_limit_1s), (60.0, self.spec.p_limit_60s)] {
            if limit > 0.0 {
                let mean = self.mean_power(seconds);
                if mean > limit {
                    db = db.max(10.0 * (mean / limit).log10());
                }
            }
        }
        db
    }

    #[cfg(test)]
    pub fn history_tail(&self, n: usize) -> Vec<(f64, f64)> {
        self.history.iter().rev().take(n).cloned().collect()
    }

    /// Overflow must be rejected before a period renews the sensing permit.
    pub fn finite(&self) -> bool {
        [
            self.t_coil,
            self.t_magnet,
            self.t_peak,
            self.power,
            self.v_rms,
            self.reduction_db,
            self.history_s,
            self.mean_power(1.0),
            self.mean_power(60.0),
        ]
        .iter()
        .all(|v| v.is_finite())
    }

    /// Governor: the reduction (dB) that keeps the coil inside the window
    /// and the mean power inside the budgets.  Thermal reduction grows
    /// linearly across the window from 0 dB at `limit - headroom - window`
    /// to `t_reduction_max` at `limit - headroom` (and keeps growing at the
    /// same rate above it), and is released only once the coil is
    /// `hysteresis` below where it started.
    pub fn govern(&mut self, globals: &Globals) -> f64 {
        let ceiling = self.spec.t_limit - self.spec.t_headroom;
        let start = ceiling - globals.t_window;
        // the hottest the coil may have been over the last step
        let t = self.t_coil.max(self.t_peak);
        if t > start {
            self.reducing = true;
        } else if t < start - globals.t_hysteresis {
            self.reducing = false;
        }
        let thermal = if self.reducing {
            ((t - start) / globals.t_window * globals.t_reduction_max).max(0.0)
        } else {
            0.0
        };
        let wanted = thermal.max(self.budget_reduction());
        // attack at once, release at most 0.5 dB per step so the level does
        // not pump
        self.reduction_db = if wanted >= self.reduction_db { wanted } else { (self.reduction_db - 0.5).max(wanted) };
        self.reduction_db
    }
}

/// The reduction (dB) a measured frame was converted at, for the budgets:
/// the most of the `possible` ones (ranges) that can have delivered it,
/// `room` being the most reduction that still delivers it at full scale;
/// the most of all when none can.  No limit: a frame the card may have
/// muted says nothing about what it was, and counts as full scale (the
/// callers' cap).
pub fn converted_at(room: f64, possible: &[(f64, f64)]) -> f64 {
    let mut fit: Option<f64> = None;
    let mut most = 0.0f64;
    for &(lo, hi) in possible {
        most = most.max(hi);
        if lo <= room {
            let v = hi.min(room);
            fit = Some(fit.map_or(v, |f| f.max(v)));
        }
    }
    fit.unwrap_or(most).max(0.0)
}

/// A budget history from its two saved means: all of the energy as late as
/// it can have come, at `full` (the power of a full-scale signal; nothing
/// before the gain is louder), the last second's at the very end of it and
/// the rest of the minute's just before that second.  Each window then
/// keeps every joule until the latest moment it could have left.  (A mean
/// over full scale, from an older file, keeps its whole window instead.)
pub fn budget_envelope(mean_1s: f64, mean_60s: f64, full: f64) -> (std::collections::VecDeque<(f64, f64)>, f64) {
    let mut h = std::collections::VecDeque::new();
    let last = mean_1s.max(0.0);
    let older = (60.0 * mean_60s - last).max(0.0);
    let d_older = (older / full).min(59.0);
    let d_last = (last / full).min(1.0);
    if older > 0.0 && d_older > 0.0 {
        h.push_back((d_older, older / d_older));
    }
    if d_last < 1.0 {
        h.push_back((1.0 - d_last, 0.0));
    }
    if last > 0.0 && d_last > 0.0 {
        h.push_back((d_last, last / d_last));
    }
    let total = h.iter().map(|e| e.0).sum();
    (h, total)
}

/// The card-wide result of one step: the reduction to apply to the shared
/// volume control is the largest any speaker needs.
pub fn reduction_db(states: &[SpeakerState]) -> f64 {
    states.iter().map(|s| s.reduction_db).fold(0.0, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> Speaker {
        Speaker {
            name: "Left".into(),
            group: 0,
            tr_coil: 60.0,
            tau_coil: 3.0,
            tr_magnet: 40.0,
            tau_magnet: 200.0,
            t_limit: 110.0,
            t_headroom: 10.0,
            z_nominal: 4.43,
            vs_scale: 3.3,
            vs_chan: 0,
            p_limit_1s: 0.0,
            p_limit_60s: 0.0,
            is_chan: None,
            is_scale: 0.0,
            r_ff: 4.43,
            vi_tr_coil: 60.0,
            vi_tau_coil: 3.0,
            vi_tr_magnet: 40.0,
            vi_tau_magnet: 200.0,
            a_t_20c: 0.0,
        }
    }

    fn globals() -> Globals {
        Globals {
            sense_pcm: 2,
            t_ambient: 35.0,
            t_hysteresis: 5.0,
            t_window: 20.0,
            channels: 2,
            period: 4096,
            t_reduction_max: 20.0,
            safe_reduction: 20.0,
            governor: crate::config::Governor::Legacy,
            shadow: false,
            sense_format: crate::config::SenseFormat::S32,
            amp_gain_dbv: 0.0,
            horizon_s: 4.6,
            release_s: 20.0,
            t_ctrl_headroom: 0.0,
            unified_caps: false,
            p_reduction_max: 0.0,
            max_reduction: 20.0,
            qualified: true,
        }
    }

    #[test]
    fn full_scale_sine_is_the_amplifier_rms() {
        let s = SpeakerState::new(spec(), 35.0);
        let n = 4096;
        let words: Vec<i32> = (0..n)
            .flat_map(|i| {
                let v = ((i as f64) * 2.0 * std::f64::consts::PI * 1000.0 / 48000.0).sin();
                [(v * 2147483647.0) as i32, 0]
            })
            .collect();
        let v = s.v_rms_of(&words, 2);
        assert!((v - 3.3 / 2f64.sqrt()).abs() < 0.01, "{v}");
        assert_eq!(s.v_rms_of(&vec![0; 8192], 2), 0.0);
    }

    #[test]
    fn thermal_steady_state_and_time_constants() {
        let mut s = SpeakerState::new(spec(), 35.0);
        // 1 W for a long time: coil = ambient + P * (tr_magnet + tr_coil)
        for _ in 0..(3000 * 10) {
            s.step(1.0, 0.1, 35.0);
        }
        assert!((s.t_magnet - 75.0).abs() < 0.1, "{}", s.t_magnet);
        assert!((s.t_coil - 135.0).abs() < 0.1, "{}", s.t_coil);
        // cooling: after one coil time constant the coil closed ~63% of the gap to the magnet
        let gap0 = s.t_coil - s.t_magnet;
        for _ in 0..30 {
            s.step(0.0, 0.1, 35.0);
        }
        let gap = s.t_coil - s.t_magnet;
        assert!((gap / gap0 - (-1f64).exp()).abs() < 0.05, "{}", gap / gap0);
    }

    #[test]
    fn ceiling_power_never_governs() {
        // -20 dBFS on the wire at 3.3 V full scale into 4.43 ohm: ~2.5 mW sine
        let mut s = SpeakerState::new(spec(), 35.0);
        let v = 3.3 * 0.1 / 2f64.sqrt();
        let p = v * v / 4.43;
        for _ in 0..(3600 * 10) {
            s.step(p, 0.1, 35.0);
            assert_eq!(s.govern(&globals()), 0.0);
        }
        // 12 mW * (60 + 40) K/W = 1.2 K above ambient
        assert!(s.t_coil < 37.0, "{}", s.t_coil);
    }

    #[test]
    fn governor_window_and_hysteresis() {
        let mut s = SpeakerState::new(spec(), 35.0);
        let g = globals(); // t_reduction_max 20 dB at the working limit
        // heat the coil into the window: limit 110 - headroom 10 = 100, window starts at 80
        s.t_coil = 90.0;
        s.t_magnet = 60.0;
        let r = s.govern(&g);
        assert!((r - 10.0).abs() < 1e-9, "{r}");
        s.t_coil = 100.0;
        assert_eq!(s.govern(&g), 20.0);
        // beyond the working limit the reduction keeps growing at the same rate
        s.t_coil = 110.0;
        assert_eq!(s.govern(&g), 30.0);
        // back inside the window but above start - hysteresis: still reducing
        s.t_coil = 78.0;
        let r = s.govern(&g);
        assert!(r > 0.0 && r < 30.0, "{r}");
        // released only below start - hysteresis, 0.5 dB per step
        s.t_coil = 70.0;
        let mut last = r;
        for _ in 0..80 {
            let now = s.govern(&g);
            assert!(now <= last && last - now <= 0.5 + 1e-9);
            last = now;
        }
        assert_eq!(last, 0.0);
    }

    #[test]
    fn power_budget_reduces_before_the_coil_warms() {
        // the J700's own numbers
        let mut spec = spec();
        spec.tr_coil = 38.0;
        spec.tau_coil = 3.3;
        spec.tr_magnet = 26.0;
        spec.tau_magnet = 100.0;
        spec.t_limit = 135.0;
        spec.t_headroom = 15.0;
        spec.p_limit_1s = 2.8154;
        spec.p_limit_60s = 2.5;
        spec.vs_scale = 7.3;
        spec.z_nominal = 4.2;
        let mut s = SpeakerState::new(spec, 35.0);
        let g = globals();
        // a 6.3 W burst (a full-scale sine into 4.2 ohm at 7.3 V): after 0.5 s
        // the 1 s mean is 3.15 W, over the budget, while the coil (69 C) is
        // still well below the thermal window (100 C)
        for _ in 0..5 {
            s.step(6.3, 0.1, 35.0);
        }
        assert!(s.t_coil < 75.0, "{}", s.t_coil);
        let r = s.govern(&g);
        assert!((r - 10.0 * (3.15f64 / 2.8154).log10()).abs() < 0.05, "{r}");
        // silence: the mean falls back under the budget and the reduction releases
        for _ in 0..20 {
            s.step(0.0, 0.1, 35.0);
            s.govern(&g);
        }
        assert_eq!(s.govern(&g), 0.0);
        // the minute budget: 2.6 W for a minute ends up over 2.5 W
        for _ in 0..600 {
            s.step(2.6, 0.1, 35.0);
        }
        assert!((s.mean_power(60.0) - 2.6).abs() < 0.01);
        assert!(s.budget_reduction() > 0.1);
    }
    #[test]
    fn finite_guard_rejects_overflow_before_governor_or_renewal() {
        let mut s = SpeakerState::new(spec(), 35.0);
        s.step(f64::MAX, 0.1, 35.0);
        assert!(!s.finite());
        let mut s = SpeakerState::new(spec(), 35.0);
        s.v_rms = f64::NAN;
        assert!(!s.finite());
        let mut s = SpeakerState::new(spec(), 35.0);
        s.history.push_back((1.0, f64::MAX));
        s.history.push_back((1.0, f64::MAX));
        assert!(!s.finite());
    }

    #[test]
    fn silence_and_negative_waveform_samples_are_valid() {
        let s = SpeakerState::new(spec(), 35.0);
        assert_eq!(s.v_rms_of(&[0; 8], 2), 0.0);
        let rms = s.v_rms_of(&[i32::MIN, 0, i32::MAX, 0], 2);
        assert!(rms.is_finite() && rms > 0.0);
        let mut s = s;
        s.step(0.0, 0.1, 35.0);
        s.govern(&globals());
        assert!(s.finite());
    }

    #[test]
    fn worst_case_power_is_a_full_scale_square_wave_at_the_card_gain() {
        let mut sp = spec();
        sp.vs_scale = 7.3;
        // the parser defaults r_ff to z_nominal (the J700)
        sp.z_nominal = 4.2;
        sp.r_ff = 4.2;
        let s = SpeakerState::new(sp.clone(), 35.0);
        assert!((s.worst_case_power(0.0) - 7.3 * 7.3 / 4.2).abs() < 1e-9);
        assert!((s.worst_case_power(20.0) - 0.073 * 7.3 / 4.2).abs() < 1e-9);
        // a negative reduction cannot raise the card above its full scale
        assert_eq!(s.worst_case_power(-6.0), s.worst_case_power(0.0));
        // a machine whose DC resistance is known and below the nominal
        // impedance (the J316's woofers: 2.7 against 3.6 ohm) is charged
        // against the DC resistance, as its feed-forward constants assume
        sp.z_nominal = 3.6;
        sp.r_ff = 2.7;
        let s = SpeakerState::new(sp, 35.0);
        assert!((s.worst_case_power(0.0) - 7.3 * 7.3 / 2.7).abs() < 1e-9);
        let (p, _) = s.block_powers(&[i32::MAX, 0], 2, 1);
        assert!((p[0] - 7.3 * 7.3 / 2.7).abs() < 1e-6, "{}", p[0]);
    }
}

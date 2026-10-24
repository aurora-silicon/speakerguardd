// SPDX-License-Identifier: MIT
//! The governors besides the legacy one (model.rs), and their models.
//!
//! The guard feeds every model here exactly the account it gives the legacy
//! model, under the same rule: each stretch of measured sense frames (only
//! the part over time not yet accounted), every other interval as a
//! full-scale signal at each speaker's least reduction since the last
//! verified boundary, and verified closed time as cooling.  No model is ever
//! fed a gain the card did not hold: measured power is what the sense lines
//! saw after the card's gain, whatever any governor asked for.  Every model
//! latches a hard fault the moment it may have reached its hard limit,
//! inside a period as much as at its end, and is saved with the legacy
//! model (state.rs); a restart resumes it only from a clean stop, as it
//! does the legacy model.
//!
//! * `asahi`: the behaviour Asahi Linux's speakersafetyd documents and
//!   ships (measured V*I power integrated sample by sample into the
//!   two-node model; a linear ramp from 0 at `t_limit - t_window` to its
//!   `min_gain` at `t_limit`, with `min_gain = 10 log10(max_pwr /
//!   peak_pwr)`; peak-hold hysteresis on the coil and the magnet; a period
//!   whose mean V*I is below -0.01 W is bad sense data), reimplemented here
//!   from its description.  Credit: the Asahi Linux contributors,
//!   speakersafetyd (MIT up to 2.0.2).  Its `t_limit` is our working limit
//!   (`t_limit - t_headroom` here) and its panic margin our hard limit.
//!   The differences all go toward protection: unmeasured time is charged
//!   at the worst case instead of skipped, the hard limit latches at any
//!   sample instead of being checked at the end, bad sense data and faults
//!   hand the speakers to the kernel's safe level instead of a panic, and
//!   the hold also keeps the hottest point of the last step.
//! * `unified`: the open-loop (feed-forward) model is always on, as on
//!   macOS: power from the amplifier voltage against the speaker's DC
//!   resistance `r_ff`.  Where V/I sense exists, the measured-power model
//!   runs too and the more restrictive of the two governs (it can only
//!   tighten).  The regulator lets through the most power that keeps each
//!   model's coil at or under `t_limit - t_headroom - t_ctrl_headroom` over
//!   the next `horizon_s`, attacks at once and releases with `release_s`;
//!   the 1 s and 60 s power budgets hold on the power before the gain.
//!   Shadow-only unless the command line says --experimental-unified.
//! * Resistance thermometry from low-band V/I (`R = sum(v i) / sum(i^2)`)
//!   is only measured and logged.

use std::collections::VecDeque;
use std::fmt::Write as _;

use crate::config::{Config, Globals, Governor, Speaker};
use crate::model::{budget_envelope, converted_at, MAX_SUB_DT, PRE_GAIN_MAX_DB, SUB_BLOCKS};

/// The asahi models see this much more power than measured (+0.02 dB): on
/// recorded sense data their gains then never ask for less than
/// speakersafetyd's (the f32-versus-f64 rounding was worth up to 0.04 C).
pub const ASAHI_MARGIN: f64 = 1.005;
/// A mean V*I below this (W) is bad sense data, not a measurement
/// (speakersafetyd's sanity bound).
pub const VI_MIN: f64 = -0.01;
/// What the unified regulator asks for when nothing may pass (dB): far past
/// any --max-reduction, so the kernel takes over.
const NOTHING_DB: f64 = 120.0;
/// Below this the low band carries too little current to read a resistance.
const RTH_I_MIN: f64 = 0.02;
/// Low band for the thermometer: well below laptop drivers' resonance.
const RTH_FC: f64 = 120.0;

fn db(ratio: f64) -> f64 {
    10.0 * ratio.log10()
}

/// The larger of two temperatures, NaN if either is: a NaN must reach the
/// limit checks, never be dropped by f64::max.
fn hi(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.max(b)
    }
}

/// Two cascaded first-order lags: the coil follows the magnet plus P*Rc,
/// the magnet follows ambient plus P*Rm (the legacy model's equations).
#[derive(Debug, Clone, PartialEq)]
pub struct Thermal {
    pub tr_coil: f64,
    pub tau_coil: f64,
    pub tr_magnet: f64,
    pub tau_magnet: f64,
    pub t_coil: f64,
    pub t_magnet: f64,
}

impl Thermal {
    pub fn new(tr_coil: f64, tau_coil: f64, tr_magnet: f64, tau_magnet: f64, t: f64) -> Thermal {
        Thermal { tr_coil, tau_coil, tr_magnet, tau_magnet, t_coil: t, t_magnet: t }
    }

    /// The coil at `t`, the magnet where steady state would put it.
    pub fn set_at(&mut self, t: f64, t_ambient: f64) {
        self.t_coil = t;
        self.t_magnet = t_ambient + (t - t_ambient) * self.tr_magnet / (self.tr_magnet + self.tr_coil);
    }

    pub fn finite(&self) -> bool {
        self.t_coil.is_finite() && self.t_magnet.is_finite()
    }

    /// The coil after `h` seconds at a constant `power`, in closed form:
    /// `alpha + beta * power`.
    pub fn coil_after(&self, h: f64, t_ambient: f64) -> (f64, f64) {
        if (self.tau_magnet - self.tau_coil).abs() <= 1e-6 * self.tau_coil {
            return self.coil_after_equal(h, t_ambient);
        }
        let e_c = (-h / self.tau_coil).exp();
        let e_m = (-h / self.tau_magnet).exp();
        // coil drive A + B e^(-t/tau_m); particular K e^(-t/tau_m), K = B r
        let r = self.tau_magnet / (self.tau_magnet - self.tau_coil);
        let alpha = self.t_coil * e_c + t_ambient * (1.0 - e_c) + (self.t_magnet - t_ambient) * r * (e_m - e_c);
        let beta = (self.tr_magnet + self.tr_coil) * (1.0 - e_c) - self.tr_magnet * r * (e_m - e_c);
        (alpha, beta)
    }

    /// Equal time constants: the limit of r (e_m - e_c) is h/tau e^(-h/tau).
    fn coil_after_equal(&self, h: f64, t_ambient: f64) -> (f64, f64) {
        let tau = self.tau_coil;
        let e = (-h / tau).exp();
        let g = h / tau * e;
        let alpha = self.t_coil * e + t_ambient * (1.0 - e) + (self.t_magnet - t_ambient) * g;
        let beta = (self.tr_magnet + self.tr_coil) * (1.0 - e) - self.tr_magnet * g;
        (alpha, beta)
    }

    /// The most constant power that keeps the coil at or under `target`
    /// for the next `h` seconds (0 when it is already there).  The coil's
    /// maximum over the horizon is its value now or at the end: it lags a
    /// drive that only rises (the magnet warms) or, hotter than the drive,
    /// first falls toward it.  Over the target already, it is the power
    /// that brings the coil back to the target by the end of the horizon
    /// (0 when even silence would not).
    pub fn max_power(&self, target: f64, h: f64, t_ambient: f64) -> f64 {
        let (alpha, beta) = self.coil_after(h, t_ambient);
        if beta <= 0.0 {
            return 0.0;
        }
        ((target - alpha) / beta).max(0.0)
    }

    /// `h` seconds at a constant `power`, exactly, for any `h`.  Returns
    /// bounds of the coil and of the magnet over the interval.  The magnet
    /// moves monotonically toward its target, so its bound is an end.  The
    /// coil's drive (magnet plus P Rc) does too; a coil below a drive that
    /// falls rises until it meets the drive and follows it down, so it never
    /// passes where the drive started, and in every other case the coil's
    /// extremes are the interval's ends.
    pub fn hold(&mut self, power: f64, h: f64, t_ambient: f64) -> (f64, f64) {
        let (c0, m0) = (self.t_coil, self.t_magnet);
        if !(h > 0.0) {
            return (c0, m0);
        }
        let (alpha, beta) = self.coil_after(h, t_ambient);
        let m_inf = t_ambient + power * self.tr_magnet;
        self.t_magnet = m_inf + (m0 - m_inf) * (-h / self.tau_magnet).exp();
        self.t_coil = alpha + beta * power;
        let mut coil = hi(c0, self.t_coil);
        let drive0 = m0 + power * self.tr_coil;
        if !(c0 >= drive0) && !(m0 <= m_inf) {
            coil = hi(coil, drive0);
        }
        (coil, hi(m0, self.t_magnet))
    }

    /// `dt` seconds at a power that varies inside them around a mean of
    /// `power`, in steps of at most MAX_SUB_DT; bounds of the coil and the
    /// magnet as model.rs keeps them: each step's end, plus what energy
    /// placed anywhere inside it could have hidden.
    pub fn varying(&mut self, power: f64, dt: f64, t_ambient: f64) -> (f64, f64) {
        let (mut coil, mut magnet) = (self.t_coil, self.t_magnet);
        if !(dt > 0.0) {
            return (coil, magnet);
        }
        let n = (dt / MAX_SUB_DT).ceil().max(1.0);
        let n = if n.is_finite() { n as u64 } else { 1 };
        let h = dt / n as f64;
        let k_magnet = 1.0 - (-h / self.tau_magnet).exp();
        let k_coil = 1.0 - (-h / self.tau_coil).exp();
        let grow = (h / self.tau_coil).exp() - 1.0;
        let late = power * self.tr_coil * (h / self.tau_coil - k_coil).max(0.0);
        for _ in 0..n {
            self.t_magnet += (t_ambient + power * self.tr_magnet - self.t_magnet) * k_magnet;
            self.t_coil += (self.t_magnet + power * self.tr_coil - self.t_coil) * k_coil;
            let end = self.t_coil + late;
            coil = hi(coil, end + (end - self.t_magnet).max(0.0) * grow);
            magnet = hi(magnet, self.t_magnet);
        }
        (coil, magnet)
    }
}

/// What one stretch of sense frames says about one speaker (power after
/// the card's gain).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Feature {
    /// Mean square voltage (V^2).
    pub v_ms: f64,
    /// Mean v*i (W) and mean square current (A^2), with current sense.
    pub vi: Option<f64>,
    pub i_ms: Option<f64>,
    /// Low-band resistance estimate (Ω), when the band had enough current.
    pub r_lf: Option<f64>,
}

/// A second-order low-pass (RBJ biquad) for the resistance thermometer.
#[derive(Debug, Clone, Default)]
struct Biquad {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    z1: f64,
    z2: f64,
}

impl Biquad {
    fn lowpass(fc: f64, rate: f64) -> Biquad {
        let w = 2.0 * std::f64::consts::PI * fc / rate;
        let (sn, cs) = w.sin_cos();
        let alpha = sn / (2.0 * std::f64::consts::FRAC_1_SQRT_2);
        let a0 = 1.0 + alpha;
        Biquad {
            b0: (1.0 - cs) / 2.0 / a0,
            b1: (1.0 - cs) / a0,
            b2: (1.0 - cs) / 2.0 / a0,
            a1: -2.0 * cs / a0,
            a2: (1.0 - alpha) / a0,
            z1: 0.0,
            z2: 0.0,
        }
    }

    fn run(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }
}

/// Per-speaker feature extraction, with the thermometer's filter state.
#[derive(Debug, Clone)]
pub struct Extractor {
    lp_v: Vec<Biquad>,
    lp_i: Vec<Biquad>,
    rate: f64,
}

impl Extractor {
    pub fn new(n: usize) -> Extractor {
        Extractor { lp_v: vec![Biquad::default(); n], lp_i: vec![Biquad::default(); n], rate: 0.0 }
    }

    /// The features of interleaved frames (S32 full scale; S16 sense words
    /// arrive widened by << 16).
    pub fn extract(&mut self, cfg: &Config, words: &[i32], rate: f64) -> Vec<Feature> {
        let ch = cfg.globals.channels;
        let frames = words.len() / ch;
        if rate != self.rate {
            for f in self.lp_v.iter_mut().chain(self.lp_i.iter_mut()) {
                *f = Biquad::lowpass(RTH_FC, rate);
            }
            self.rate = rate;
        }
        let mut out = Vec::with_capacity(cfg.speakers.len());
        for (k, s) in cfg.speakers.iter().enumerate() {
            let vs = s.vs_scale / 2147483648.0;
            let mut v2 = 0.0;
            let (mut vi, mut i2, mut lvi, mut li2) = (0.0, 0.0, 0.0, 0.0);
            match s.is_chan {
                None => {
                    for f in words.chunks_exact(ch) {
                        let v = f[s.vs_chan] as f64 * vs;
                        v2 += v * v;
                    }
                }
                Some(ic) => {
                    let is = s.is_scale / 2147483648.0;
                    for f in words.chunks_exact(ch) {
                        let v = f[s.vs_chan] as f64 * vs;
                        let i = f[ic] as f64 * is;
                        v2 += v * v;
                        vi += v * i;
                        i2 += i * i;
                        let lv = self.lp_v[k].run(v);
                        let li = self.lp_i[k].run(i);
                        lvi += lv * li;
                        li2 += li * li;
                    }
                }
            }
            let n = frames.max(1) as f64;
            let mut f = Feature { v_ms: v2 / n, ..Default::default() };
            if s.is_chan.is_some() {
                f.vi = Some(vi / n);
                f.i_ms = Some(i2 / n);
                if (li2 / n).sqrt() >= RTH_I_MIN {
                    let r = lvi / li2;
                    if r.is_finite() && r > 0.5 * s.r_ff && r < 3.0 * s.r_ff {
                        f.r_lf = Some(r);
                    }
                }
            }
            out.push(f);
        }
        out
    }
}

/// Mean power before the card's gain over the last `seconds` of a history
/// of (seconds, watts), newest last; time before it counts as silence.
fn mean_of(history: &VecDeque<(f64, f64)>, seconds: f64) -> f64 {
    let (mut t, mut e) = (0.0, 0.0);
    for (dt, p) in history.iter().rev() {
        if t >= seconds {
            break;
        }
        let take = dt.min(seconds - t);
        e += p * take;
        t += take;
    }
    e / seconds
}

/// Add `dt` seconds at `power` and keep one minute: whole entries drop
/// out, then only the expired part of the oldest one (its energy for the
/// rest still counts; review R5 of speakerguardd#7).
fn record(history: &mut VecDeque<(f64, f64)>, history_s: &mut f64, dt: f64, power: f64) {
    if !(dt > 0.0) {
        return;
    }
    history.push_back((dt, power));
    *history_s += dt;
    while *history_s > 60.0 {
        let excess = *history_s - 60.0;
        match history.front_mut() {
            Some(front) if front.0 <= excess => {
                *history_s -= front.0;
                history.pop_front();
            }
            Some(front) => {
                front.0 -= excess;
                *history_s = 60.0;
            }
            None => {
                *history_s = 0.0;
                break;
            }
        }
    }
}

/// The power before the card's gain, from the power after it: no gain past
/// PRE_GAIN_MAX_DB is undone (under a muted card the sense data is noise).
fn pre_gain(power: f64, applied_db: f64) -> f64 {
    power * 10f64.powf(applied_db.clamp(0.0, PRE_GAIN_MAX_DB) / 10.0)
}

/// speakersafetyd's model and governor for one speaker.
#[derive(Debug, Clone)]
pub struct AsahiSpeaker {
    pub th: Thermal,
    /// The peak-hold values the ramp works from (°C).
    pub coil_hyst: f64,
    pub magnet_hyst: f64,
    /// dB, <= 0: the ramp's end at the working limit.
    pub min_gain: f64,
    /// dB, <= 0: what the governor asks for.
    pub gain: f64,
    /// Latched once the coil or the magnet may have reached the hard limit.
    pub hard: bool,
    /// The hottest the coil and the magnet may have been over the last step.
    pub coil_peak: f64,
    pub magnet_peak: f64,
    t_limit: f64,
    ceiling: f64,
    z_min: f64,
    v_worst: f64,
}

impl AsahiSpeaker {
    fn new(s: &Speaker, g: &Globals) -> AsahiSpeaker {
        let work = s.t_limit - s.t_headroom;
        let max_pwr = (work - g.t_ambient) / (s.vi_tr_magnet + s.vi_tr_coil);
        // worst-case peak power is twice a full-scale sine's
        let peak_pwr = 10f64.powf(g.amp_gain_dbv / 10.0) / s.z_nominal * 2.0;
        let min_gain = db(max_pwr / peak_pwr).min(0.0);
        let th = Thermal::new(s.vi_tr_coil, s.vi_tau_coil, s.vi_tr_magnet, s.vi_tau_magnet, g.t_ambient);
        AsahiSpeaker {
            coil_hyst: th.t_coil,
            magnet_hyst: th.t_magnet,
            coil_peak: th.t_coil,
            magnet_peak: th.t_magnet,
            th,
            min_gain,
            gain: 0.0,
            hard: false,
            t_limit: s.t_limit,
            ceiling: work,
            z_min: s.z_nominal.min(s.r_ff),
            v_worst: v_worst(s, g),
        }
    }

    fn assume_latched(&mut self, t_ambient: f64) {
        self.th.set_at(self.t_limit, t_ambient);
        self.coil_peak = self.th.t_coil;
        self.magnet_peak = self.th.t_magnet;
        self.coil_hyst = self.th.t_coil;
        self.magnet_hyst = self.th.t_magnet;
        self.hard = true;
    }

    /// Latch on bounds at or past the hard limit (NaN included).
    fn note(&mut self, coil: f64, magnet: f64) {
        self.coil_peak = coil;
        self.magnet_peak = magnet;
        if !(coil < self.t_limit && magnet < self.t_limit) {
            self.hard = true;
        }
    }

    /// Measured frames, sample by sample, each `step` seconds long: the
    /// power is the sensed V*I, after the card's gain, whatever this
    /// governor asked for.  The hard limit is checked at every sample, so
    /// that power swinging positive then negative inside a period cannot
    /// carry the model over the limit and back unseen (review F4).  A
    /// crossing also leaves the model at the limit at least: the negative
    /// V*I after it cools the model, never the coil, so the model cannot
    /// say the coil came back down.
    /// The first frame is `first` of a step long (the part of it over
    /// time not yet accounted).
    #[allow(clippy::too_many_arguments)]
    fn samples(&mut self, words: &[i32], channels: usize, (vc, ic): (usize, usize), (vs, is): (f64, f64), step: f64, first: f64, t_ambient: f64) {
        let coeffs = |h: f64| (h / (self.th.tau_coil + h), h / (self.th.tau_magnet + h));
        let (ac1, am1) = coeffs(step * first);
        let (acn, amn) = coeffs(step);
        let (trc, trm) = (self.th.tr_coil, self.th.tr_magnet);
        let (mut tc, mut tm) = (self.th.t_coil, self.th.t_magnet);
        let (mut pc, mut pm) = (tc, tm);
        // the samples' own highs: a model that starts at the limit (latched)
        // may cool from it
        let (mut xc, mut xm) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        let mut crossed = false;
        for (i, f) in words.chunks_exact(channels).enumerate() {
            let (ac, am) = if i == 0 { (ac1, am1) } else { (acn, amn) };
            let p = f[vc] as f64 * vs * (f[ic] as f64 * is) * ASAHI_MARGIN;
            let coil_target = tm + p * trc;
            let magnet_target = t_ambient + p * trm;
            tc = coil_target * ac + tc * (1.0 - ac);
            tm = magnet_target * am + tm * (1.0 - am);
            if !(tc < self.t_limit && tm < self.t_limit) {
                crossed = true;
            }
            pc = hi(pc, tc);
            pm = hi(pm, tm);
            xc = hi(xc, tc);
            xm = hi(xm, tm);
        }
        if crossed {
            if !(xc < self.t_limit) {
                tc = hi(tc, self.t_limit);
            }
            if !(xm < self.t_limit) {
                tm = hi(tm, self.t_limit);
            }
        }
        self.th.t_coil = tc;
        self.th.t_magnet = tm;
        self.note(pc, pm);
        if crossed {
            self.hard = true;
        }
    }

    /// `d` seconds at the most power the card's `reduction` lets through:
    /// a full-scale square wave into the smallest impedance the speaker
    /// can present.
    fn worst(&mut self, reduction: f64, d: f64, t_ambient: f64) {
        let v = self.v_worst * 10f64.powf(-reduction.max(0.0) / 20.0);
        let (c, m) = self.th.hold(v * v / self.z_min, d, t_ambient);
        self.note(c, m);
    }

    fn idle(&mut self, d: f64, t_ambient: f64) {
        let (c, m) = self.th.hold(0.0, d, t_ambient);
        self.note(c, m);
    }

    /// speakersafetyd's ramp: hold each temperature up to `t_hysteresis`
    /// above where it is, and ramp from 0 at the window's start to min_gain
    /// at the working limit (and on beyond it).
    fn govern(&mut self, g: &Globals) {
        self.coil_hyst = hi(self.coil_hyst.max(self.th.t_coil).min(self.th.t_coil + g.t_hysteresis), self.coil_peak);
        self.magnet_hyst = hi(self.magnet_hyst.max(self.th.t_magnet).min(self.th.t_magnet + g.t_hysteresis), self.magnet_peak);
        let temp = hi(self.coil_hyst, self.magnet_hyst);
        let frac = (temp - (self.ceiling - g.t_window)) / g.t_window;
        let mut gain = self.min_gain * frac.max(0.0);
        if gain > -0.01 {
            gain = 0.0;
        }
        // a NaN stays NaN and fails the guard's finite check
        self.gain = if temp.is_nan() { f64::NAN } else { gain };
    }

    fn clear_hard_if_cool(&mut self) -> bool {
        let hottest = hi(hi(self.coil_peak, self.magnet_peak), self.th.hottest());
        if self.hard && hottest < self.ceiling {
            self.hard = false;
            return true;
        }
        false
    }

    fn finite(&self) -> bool {
        self.th.finite() && [self.coil_hyst, self.magnet_hyst, self.gain, self.coil_peak, self.magnet_peak].iter().all(|v| v.is_finite())
    }

    /// Continue from a saved state.
    pub fn resume(&mut self, x: &SavedAsahi) {
        self.th.t_coil = x.t_coil;
        self.th.t_magnet = x.t_magnet;
        self.coil_peak = x.t_peak.max(x.t_coil);
        self.magnet_peak = x.t_magnet;
        self.coil_hyst = x.coil_hyst.max(x.t_coil);
        self.magnet_hyst = x.magnet_hyst.max(x.t_magnet);
        self.gain = x.gain.min(0.0);
        self.hard = x.hard;
    }
}

impl Thermal {
    fn hottest(&self) -> f64 {
        hi(self.t_coil, self.t_magnet)
    }
}

/// RMS volts of the most the amplifier can put out at the card's full gain:
/// a full-scale square wave.
pub fn v_worst(s: &Speaker, g: &Globals) -> f64 {
    if g.amp_gain_dbv > 0.0 {
        // a 0 dBFS sine is amp_gain_dbv; the square has sqrt(2) more
        2f64.sqrt() * 10f64.powf(g.amp_gain_dbv / 20.0)
    } else {
        s.vs_scale
    }
}

/// The unified regulator's models and state for one speaker.
#[derive(Debug, Clone)]
pub struct UnifiedSpeaker {
    pub ff: Thermal,
    pub vi: Option<Thermal>,
    /// (dt, feed-forward power before the gain) per step, one minute.
    history: VecDeque<(f64, f64)>,
    history_s: f64,
    /// dB, >= 0
    pub reduction: f64,
    /// The latest power before the gain: feed-forward and measured (W).
    last_ff_pre: f64,
    last_vi_pre: Option<f64>,
    pub hard: bool,
    /// The hottest either model's coil or magnet may have been over the last step.
    pub peak: f64,
    /// A bound the next measured blocks' peak starts from (the first frame
    /// of a period integrated on its own).
    peak_floor: Option<f64>,
    t_limit: f64,
    ceiling: f64,
    r_ff: f64,
    z_min: f64,
    v_worst: f64,
}

impl UnifiedSpeaker {
    fn new(s: &Speaker, g: &Globals) -> UnifiedSpeaker {
        let ta = g.t_ambient;
        UnifiedSpeaker {
            ff: Thermal::new(s.tr_coil, s.tau_coil, s.tr_magnet, s.tau_magnet, ta),
            vi: s.is_chan.map(|_| Thermal::new(s.vi_tr_coil, s.vi_tau_coil, s.vi_tr_magnet, s.vi_tau_magnet, ta)),
            history: VecDeque::new(),
            history_s: 0.0,
            reduction: 0.0,
            last_ff_pre: 0.0,
            last_vi_pre: None,
            hard: false,
            peak: ta,
            peak_floor: None,
            t_limit: s.t_limit,
            ceiling: s.t_limit - s.t_headroom,
            r_ff: s.r_ff,
            z_min: s.z_nominal.min(s.r_ff),
            v_worst: v_worst(s, g),
        }
    }

    fn assume_latched(&mut self, t_ambient: f64) {
        self.ff.set_at(self.t_limit, t_ambient);
        if let Some(v) = self.vi.as_mut() {
            v.set_at(self.t_limit, t_ambient);
        }
        self.peak = self.t_limit;
        self.hard = true;
    }

    fn note(&mut self, peak: f64) {
        self.peak = peak;
        if !(peak < self.t_limit) {
            self.hard = true;
        }
    }

    pub fn mean_pre(&self, seconds: f64) -> f64 {
        mean_of(&self.history, seconds)
    }

    /// The power before the card's gain of a block whose mean V^2 is
    /// `v_ms` and whose power after it is `power`, converted at one of the
    /// `possible` reductions: the most that can have delivered a block this
    /// loud, never more than full scale gives (`full`).
    fn pre_of(&self, v_ms: f64, power: f64, full: f64, possible: &[(f64, f64)]) -> f64 {
        if !(power > 0.0) {
            return power.max(0.0);
        }
        let room = if v_ms > 0.0 { 10.0 * (self.v_worst * self.v_worst / v_ms).log10() } else { PRE_GAIN_MAX_DB };
        let pre = power * 10f64.powf(converted_at(room, possible) / 10.0);
        if pre > full { full } else { pre }
    }

    /// Measured blocks of equal length: (V^2 mean, V*I mean) each, after
    /// the card's gain, which applied one of the `possible` reductions.
    fn blocks(&mut self, blocks: &[(f64, Option<f64>)], span: f64, possible: &[(f64, f64)], t_ambient: f64) {
        let n = blocks.len().max(1) as f64;
        let h = span / n;
        let mut peak = self.peak_floor.take().unwrap_or(f64::NEG_INFINITY).max(self.ff.hottest());
        for &(v_ms, _) in blocks {
            let (c, m) = self.ff.varying(v_ms / self.r_ff, h, t_ambient);
            peak = hi(peak, hi(c, m));
        }
        let mut mean_vi = None;
        if let Some(vi) = self.vi.as_mut() {
            peak = hi(peak, vi.hottest());
            // negative blocks only ever dropped: never cooler than measured
            for &(_, p) in blocks {
                let (c, m) = vi.varying(p.unwrap_or(0.0).max(0.0), h, t_ambient);
                peak = hi(peak, hi(c, m));
            }
            mean_vi = Some(blocks.iter().map(|b| b.1.unwrap_or(0.0).max(0.0)).sum::<f64>() / n);
        }
        self.note(peak);
        let full_ff = self.v_worst * self.v_worst / self.r_ff;
        let full_vi = self.v_worst * self.v_worst / self.z_min;
        let pre = blocks.iter().map(|b| self.pre_of(b.0, b.0 / self.r_ff, full_ff, possible)).sum::<f64>() / n;
        record(&mut self.history, &mut self.history_s, span, pre);
        self.last_ff_pre = pre;
        self.last_vi_pre = mean_vi.map(|_| blocks.iter().map(|b| self.pre_of(b.0, b.1.unwrap_or(0.0).max(0.0), full_vi, possible)).sum::<f64>() / n);
    }

    fn worst(&mut self, reduction: f64, d: f64, t_ambient: f64) {
        let v = self.v_worst * 10f64.powf(-reduction.max(0.0) / 20.0);
        let (c, m) = self.ff.hold(v * v / self.r_ff, d, t_ambient);
        let mut peak = hi(c, m);
        if let Some(vi) = self.vi.as_mut() {
            let (c, m) = vi.hold(v * v / self.z_min, d, t_ambient);
            peak = hi(peak, hi(c, m));
        }
        self.note(peak);
        let pre = pre_gain(v * v / self.r_ff, reduction);
        record(&mut self.history, &mut self.history_s, d, pre);
        self.last_ff_pre = pre;
        self.last_vi_pre = self.vi.as_ref().map(|_| pre_gain(v * v / self.z_min, reduction));
    }

    fn idle(&mut self, d: f64, t_ambient: f64) {
        let (c, m) = self.ff.hold(0.0, d, t_ambient);
        let mut peak = hi(c, m);
        if let Some(vi) = self.vi.as_mut() {
            let (c, m) = vi.hold(0.0, d, t_ambient);
            peak = hi(peak, hi(c, m));
        }
        self.note(peak);
        record(&mut self.history, &mut self.history_s, d, 0.0);
        self.last_ff_pre = 0.0;
        self.last_vi_pre = self.vi.as_ref().map(|_| 0.0);
    }

    fn govern(&mut self, s: &Speaker, g: &Globals, dt: f64, playing: bool) {
        let ta = g.t_ambient;
        let target = s.t_limit - s.t_headroom - g.t_ctrl_headroom;
        let mut wanted: f64 = 0.0;
        if playing {
            let need = |p_pre: f64, th: &Thermal| -> f64 {
                if p_pre <= 0.0 {
                    return 0.0;
                }
                // Under the target, look ahead over the horizon; at or
                // over it, get back to it by the end of the next period.
                let h = if th.t_coil < target { g.horizon_s } else { dt.max(0.02) };
                let pmax = th.max_power(target, h, ta);
                if pmax <= 0.0 {
                    NOTHING_DB
                } else {
                    db(p_pre / pmax).clamp(0.0, NOTHING_DB)
                }
            };
            let mut thermal = need(self.last_ff_pre, &self.ff);
            // where V/I sense exists the measured-power model can only
            // tighten the feed-forward one
            if let (Some(vi), Some(p)) = (self.vi.as_ref(), self.last_vi_pre) {
                thermal = thermal.max(need(p, vi));
            }
            let mut power: f64 = 0.0;
            for (seconds, limit) in [(1.0, s.p_limit_1s), (60.0, s.p_limit_60s)] {
                if limit > 0.0 {
                    let mean = self.mean_pre(seconds);
                    if mean > limit {
                        power = power.max(db(mean / limit));
                    }
                }
            }
            if g.unified_caps {
                thermal = thermal.min(g.t_reduction_max);
                if g.p_reduction_max > 0.0 {
                    power = power.min(g.p_reduction_max);
                }
            }
            wanted = wanted.max(thermal).max(power);
        }
        let wanted = wanted.max(0.0);
        self.reduction = if wanted >= self.reduction {
            wanted
        } else {
            wanted + (self.reduction - wanted) * (-dt.max(0.0) / g.release_s).exp()
        };
        if self.reduction < 0.01 {
            self.reduction = 0.0;
        }
    }

    #[cfg(test)]
    pub fn idle_for_test(&mut self, d: f64, t_ambient: f64) {
        self.idle(d, t_ambient)
    }

    #[cfg(test)]
    pub fn blocks_for_test(&mut self, blocks: &[(f64, Option<f64>)], span: f64, applied: f64, t_ambient: f64) {
        self.blocks(blocks, span, &[(applied, applied)], t_ambient)
    }

    fn clear_hard_if_cool(&mut self) -> bool {
        let mut hottest = hi(self.peak, self.ff.hottest());
        if let Some(v) = &self.vi {
            hottest = hi(hottest, v.hottest());
        }
        if self.hard && hottest < self.ceiling {
            self.hard = false;
            return true;
        }
        false
    }

    fn finite(&self) -> bool {
        self.ff.finite()
            && self.vi.as_ref().is_none_or(Thermal::finite)
            && [self.reduction, self.peak, self.last_ff_pre, self.history_s, self.mean_pre(1.0), self.mean_pre(60.0)].iter().all(|v| v.is_finite())
            && self.last_vi_pre.is_none_or(f64::is_finite)
    }

    /// Continue from a saved state: temperatures, the reduction being
    /// released and the power history's last second and minute, the
    /// minute's energy as late as it could have been (model.rs).  A
    /// measured-power model the file does not carry starts latched.
    pub fn resume(&mut self, x: &SavedUnified, t_ambient: f64) {
        self.ff.t_coil = x.ff.0;
        self.ff.t_magnet = x.ff.1;
        let mut peak = x.t_peak.max(x.ff.0);
        let mut missing = false;
        if let Some(vi) = self.vi.as_mut() {
            match x.vi {
                Some((c, m)) => {
                    vi.t_coil = c;
                    vi.t_magnet = m;
                    peak = peak.max(c);
                }
                None => {
                    vi.set_at(self.t_limit, t_ambient);
                    peak = peak.max(self.t_limit);
                    missing = true;
                }
            }
        }
        self.peak = peak;
        self.hard = x.hard || missing;
        self.reduction = x.reduction.max(0.0);
        // two means do not say when the energy came (model::budget_envelope)
        (self.history, self.history_s) = budget_envelope(x.mean_1s, x.mean_60s, self.v_worst * self.v_worst / self.r_ff);
    }
}

/// A speaker's name as one word ("Left Woofer 1" -> "Left_Woofer_1").
pub fn state_name(name: &str) -> String {
    name.replace(' ', "_")
}

/// The asahi state a restart resumes from.
#[derive(Debug, Clone, PartialEq)]
pub struct SavedAsahi {
    pub name: String,
    pub t_coil: f64,
    pub t_magnet: f64,
    pub coil_hyst: f64,
    pub magnet_hyst: f64,
    pub gain: f64,
    pub hard: bool,
    pub t_peak: f64,
}

/// The unified state a restart resumes from.
#[derive(Debug, Clone, PartialEq)]
pub struct SavedUnified {
    pub name: String,
    pub ff: (f64, f64),
    pub vi: Option<(f64, f64)>,
    pub reduction: f64,
    pub mean_1s: f64,
    pub mean_60s: f64,
    pub hard: bool,
    pub t_peak: f64,
}

/// The engine: the asahi models (machines with V/I sense) and the unified
/// ones (when they govern or the shadow log runs).
pub struct Engine {
    pub cfg: Config,
    pub asahi: Option<Vec<AsahiSpeaker>>,
    pub unified: Option<Vec<UnifiedSpeaker>>,
    pub extractor: Extractor,
    /// The features of the last stretch of measured frames.
    pub last: Vec<Feature>,
}

/// Every speaker of a group gets the group's largest reduction.
pub fn link(cfg: &Config, own: &[f64]) -> Vec<f64> {
    cfg.speakers
        .iter()
        .map(|s| cfg.speakers.iter().zip(own).filter(|(o, _)| o.group == s.group).map(|(_, &r)| r).fold(0.0, hi))
        .collect()
}

impl Engine {
    /// The models this configuration runs, at ambient and unlatched (the
    /// guard then restores them or starts them latched); None when there
    /// is nothing to run besides the legacy model.
    pub fn new(cfg: &Config, unified: bool) -> Option<Engine> {
        let g = &cfg.globals;
        let asahi = cfg.has_vi().then(|| cfg.speakers.iter().map(|s| AsahiSpeaker::new(s, g)).collect::<Vec<_>>());
        let unified = unified.then(|| cfg.speakers.iter().map(|s| UnifiedSpeaker::new(s, g)).collect::<Vec<_>>());
        if asahi.is_none() && unified.is_none() {
            return None;
        }
        let n = cfg.speakers.len();
        Some(Engine { cfg: cfg.clone(), asahi, unified, extractor: Extractor::new(n), last: vec![Feature::default(); n] })
    }

    pub fn has(&self, gov: Governor) -> bool {
        match gov {
            Governor::Legacy => false,
            Governor::Asahi => self.asahi.is_some(),
            Governor::Unified => self.unified.is_some(),
        }
    }

    /// Nothing is known about the speakers: every model at its hard limit,
    /// latched.
    pub fn assume_latched(&mut self) {
        let ta = self.cfg.globals.t_ambient;
        for s in self.asahi.iter_mut().flatten() {
            s.assume_latched(ta);
        }
        for s in self.unified.iter_mut().flatten() {
            s.assume_latched(ta);
        }
    }

    /// Continue from a clean stop's saved models, then charge `safe`
    /// seconds at the worst case the card's safe level (`safe_db`) lets
    /// through (no daemon held the card since).  A speaker a model has no
    /// saved line for starts latched at its limit, uncharged.
    pub fn resume(&mut self, asahi: &[SavedAsahi], unified: &[SavedUnified], safe: f64, safe_db: f64) {
        let ta = self.cfg.globals.t_ambient;
        let names: Vec<String> = self.cfg.speakers.iter().map(|s| state_name(&s.name)).collect();
        for (k, s) in self.asahi.iter_mut().flatten().enumerate() {
            match asahi.iter().find(|x| x.name == names[k]) {
                Some(x) => {
                    s.resume(x);
                    let (mut pc, mut pm) = (s.coil_peak, s.magnet_peak);
                    if safe > 0.0 {
                        s.worst(safe_db, safe, ta);
                        pc = hi(pc, s.coil_peak);
                        pm = hi(pm, s.magnet_peak);
                    }
                    // the hold sees the hottest point of both charges
                    s.coil_peak = pc;
                    s.magnet_peak = pm;
                }
                None => s.assume_latched(ta),
            }
        }
        for (k, s) in self.unified.iter_mut().flatten().enumerate() {
            match unified.iter().find(|x| x.name == names[k]) {
                Some(x) => {
                    s.resume(x, ta);
                    let mut p = s.peak;
                    if safe > 0.0 {
                        s.worst(safe_db, safe, ta);
                        p = hi(p, s.peak);
                    }
                    s.peak = p;
                }
                None => s.assume_latched(ta),
            }
        }
    }

    /// The features of measured frames (kept for the log), and whether they
    /// are a measurement at all: a mean V*I below VI_MIN, or a nonfinite
    /// one, is bad sense data (review F3).
    pub fn check(&mut self, words: &[i32], rate: f64) -> Result<(), String> {
        let f = self.extractor.extract(&self.cfg, words, rate);
        self.last = f;
        for (s, f) in self.cfg.speakers.iter().zip(&self.last) {
            if !f.v_ms.is_finite() {
                return Err(format!("{}: nonfinite sensed voltage", s.name));
            }
            if let Some(p) = f.vi {
                if !(p >= VI_MIN) {
                    return Err(format!("{}: negative or invalid sensed power ({p:.3} W)", s.name));
                }
            }
        }
        Ok(())
    }

    /// `frames` measured frames in `words`, `step` seconds each, all of
    /// them over time not yet accounted except the first, of which only
    /// `first` (0 < first <= 1) of its step is.  The card applied one of
    /// `possible[k]` to speaker k (the models get the sensed power; the
    /// reductions only rebuild the power before the gain for the unified
    /// budgets).  The asahi models integrate every sample; the unified ones
    /// blocks, the first frame on its own when it is only partly new
    /// (review P8-3: no frame is integrated over time already accounted,
    /// and no stretch is reduced to its blocks' means).
    pub fn measured(&mut self, words: &[i32], frames: usize, step: f64, first: f64, possible: &[Vec<(f64, f64)>]) {
        if frames == 0 || !(step > 0.0) || !(first > 0.0) {
            return;
        }
        let g = self.cfg.globals.clone();
        let ch = g.channels;
        let words = &words[..frames * ch];
        let span = step * ((frames - 1) as f64 + first.min(1.0));
        if let Some(asahi) = self.asahi.as_mut() {
            for (k, s) in self.cfg.speakers.iter().enumerate() {
                let Some(ic) = s.is_chan else { continue };
                asahi[k].samples(words, ch, (s.vs_chan, ic), (s.vs_scale / 2147483648.0, s.is_scale / 2147483648.0), step, first.min(1.0), g.t_ambient);
            }
        }
        if self.unified.is_some() {
            let (head, rest, rest_frames) = if first < 1.0 && frames > 1 { (Some(&words[..ch]), &words[ch..], frames - 1) } else { (None, words, frames) };
            let n = (SUB_BLOCKS * rest_frames).div_ceil(g.period).max(1).min(rest_frames);
            let head_f = head.map(|w| block_features(&self.cfg, w, 1));
            let per = block_features(&self.cfg, rest, n);
            let rest_s = if head.is_some() { step * rest_frames as f64 } else { span };
            let unified = self.unified.as_mut().unwrap();
            for (k, b) in per.iter().enumerate() {
                if let Some(h) = &head_f {
                    unified[k].blocks(&h[k], step * first, &possible[k], g.t_ambient);
                    unified[k].peak_floor = Some(unified[k].peak);
                }
                unified[k].blocks(b, rest_s, &possible[k], g.t_ambient);
            }
        }
        self.govern(span, true);
    }

    /// `d` seconds of possible playback no frames measured, at the most the
    /// card's per-speaker `reduction` lets through.
    pub fn worst(&mut self, d: f64, reduction: &[f64]) {
        if !(d > 0.0) {
            return;
        }
        let ta = self.cfg.globals.t_ambient;
        for (k, s) in self.asahi.iter_mut().flatten().enumerate() {
            s.worst(reduction[k], d, ta);
        }
        for (k, s) in self.unified.iter_mut().flatten().enumerate() {
            s.worst(reduction[k], d, ta);
        }
        self.govern(d, true);
    }

    /// `d` seconds with the speakers known to be closed.
    pub fn idle(&mut self, d: f64) {
        if !(d > 0.0) {
            self.govern(0.0, false);
            return;
        }
        let ta = self.cfg.globals.t_ambient;
        for s in self.asahi.iter_mut().flatten() {
            s.idle(d, ta);
        }
        for s in self.unified.iter_mut().flatten() {
            s.idle(d, ta);
        }
        self.govern(d.max(0.0), false);
    }

    pub fn govern(&mut self, dt: f64, playing: bool) {
        let g = &self.cfg.globals;
        for s in self.asahi.iter_mut().flatten() {
            s.govern(g);
        }
        if let Some(u) = self.unified.as_mut() {
            for (k, s) in self.cfg.speakers.iter().enumerate() {
                u[k].govern(s, g, dt, playing);
            }
        }
    }

    /// Per-speaker reduction (dB) a governor asks for, group-linked.
    pub fn reductions(&self, gov: Governor) -> Vec<f64> {
        let own: Vec<f64> = match gov {
            Governor::Asahi => self.asahi.iter().flatten().map(|s| -s.gain).collect(),
            Governor::Unified => self.unified.iter().flatten().map(|s| s.reduction).collect(),
            Governor::Legacy => Vec::new(),
        };
        if own.len() != self.cfg.speakers.len() {
            // a governor this engine does not run: nothing may pass
            return vec![f64::INFINITY; self.cfg.speakers.len()];
        }
        link(&self.cfg, &own)
    }

    /// A model of `gov` latched at its hard limit (a governor this engine
    /// does not run counts as one).
    pub fn hard(&self, gov: Governor) -> bool {
        match gov {
            Governor::Asahi => self.asahi.as_ref().is_none_or(|a| a.iter().any(|s| s.hard)),
            Governor::Unified => self.unified.as_ref().is_none_or(|u| u.iter().any(|s| s.hard)),
            Governor::Legacy => false,
        }
    }

    /// Clear latches whose models are back under the working ceiling; the
    /// names of the ones cleared.
    pub fn clear_hard_if_cool(&mut self) -> Vec<String> {
        let mut out = Vec::new();
        for (k, s) in self.asahi.iter_mut().flatten().enumerate() {
            if s.clear_hard_if_cool() {
                out.push(format!("{} (asahi)", self.cfg.speakers[k].name));
            }
        }
        for (k, s) in self.unified.iter_mut().flatten().enumerate() {
            if s.clear_hard_if_cool() {
                out.push(format!("{} (unified)", self.cfg.speakers[k].name));
            }
        }
        out
    }

    pub fn finite(&self) -> bool {
        self.asahi.iter().flatten().all(AsahiSpeaker::finite) && self.unified.iter().flatten().all(UnifiedSpeaker::finite)
    }

    /// Coil temperatures of speaker `k`: asahi, unified feed-forward,
    /// unified measured-power (NaN where a model does not run).
    pub fn temps(&self, k: usize) -> (f64, f64, f64) {
        let a = self.asahi.as_ref().map_or(f64::NAN, |a| a[k].th.t_coil);
        let (ff, vi) = self.unified.as_ref().map_or((f64::NAN, f64::NAN), |u| (u[k].ff.t_coil, u[k].vi.as_ref().map_or(f64::NAN, |v| v.t_coil)));
        (a, ff, vi)
    }

    /// The engine's lines of the state file.
    pub fn encode(&self) -> String {
        let mut o = String::new();
        if let Some(a) = &self.asahi {
            o += "# asahi name t_coil_C t_magnet_C coil_hold_C magnet_hold_C gain_dB hard t_peak_C\n";
            for (s, x) in self.cfg.speakers.iter().zip(a) {
                let _ = writeln!(o, "asahi {} {:.6} {:.6} {:.6} {:.6} {:.6} {} {:.6}", state_name(&s.name), x.th.t_coil, x.th.t_magnet, x.coil_hyst, x.magnet_hyst, x.gain, x.hard as u8, hi(x.coil_peak, x.th.t_coil));
            }
        }
        if let Some(u) = &self.unified {
            o += "# unified name ff_coil_C ff_magnet_C vi_coil_C vi_magnet_C reduction_dB mean_1s_W mean_60s_W hard t_peak_C\n";
            for (s, x) in self.cfg.speakers.iter().zip(u) {
                let vi = x.vi.as_ref().map_or("- -".to_string(), |v| format!("{:.6} {:.6}", v.t_coil, v.t_magnet));
                let _ = writeln!(o, "unified {} {:.6} {:.6} {vi} {:.6} {:.6} {:.6} {} {:.6}", state_name(&s.name), x.ff.t_coil, x.ff.t_magnet, x.reduction, x.mean_pre(1.0), x.mean_pre(60.0), x.hard as u8, x.peak);
            }
        }
        o
    }

    /// One JSON line for the shadow log: what every model holds and every
    /// governor asks for, on the account the card's real gains gave them.
    pub fn shadow_line(&self, boottime_s: f64, governing: Governor, armed: bool, written: &[f64], legacy: &[(f64, f64)]) -> String {
        let mut o = String::new();
        let _ = write!(o, "{{\"t\":{boottime_s:.3},\"gov\":\"{}\",\"armed\":{armed}", governing.name());
        let ra = self.asahi.as_ref().map(|_| self.reductions(Governor::Asahi));
        let ru = self.unified.as_ref().map(|_| self.reductions(Governor::Unified));
        o.push_str(",\"spk\":[");
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            if k > 0 {
                o.push(',');
            }
            let f = self.last.get(k).copied().unwrap_or_default();
            let w = written.get(k).copied().unwrap_or(f64::INFINITY);
            let _ = write!(o, "{{\"n\":\"{}\",\"card\":{},\"p_ff\":{:.5}", s.name, if w.is_finite() { format!("{w:.2}") } else { "null".into() }, f.v_ms / s.r_ff);
            if let Some(l) = legacy.get(k) {
                let _ = write!(o, ",\"leg\":[{:.2},{:.2}]", l.0, l.1);
            }
            if let Some(p) = f.vi {
                let _ = write!(o, ",\"p_vi\":{p:.5}");
            }
            if let (Some(a), Some(r)) = (&self.asahi, &ra) {
                let x = &a[k];
                let _ = write!(o, ",\"asa\":{:.2},\"asa_t\":[{:.2},{:.2}],\"asa_hard\":{}", r[k], x.th.t_coil, x.th.t_magnet, x.hard);
            }
            if let (Some(u), Some(r)) = (&self.unified, &ru) {
                let x = &u[k];
                let _ = write!(o, ",\"uni\":{:.2},\"ff\":[{:.2},{:.2}]", r[k], x.ff.t_coil, x.ff.t_magnet);
                if let Some(v) = &x.vi {
                    let _ = write!(o, ",\"vi\":[{:.2},{:.2}]", v.t_coil, v.t_magnet);
                }
            }
            if let Some(r) = f.r_lf {
                let t = if s.a_t_20c > 0.0 { 20.0 + (r / s.r_ff - 1.0) / s.a_t_20c } else { f64::NAN };
                let _ = write!(o, ",\"r_lf\":{r:.4},\"t_r\":{}", if t.is_finite() { format!("{t:.1}") } else { "null".into() });
            }
            o.push('}');
        }
        o.push_str("]}\n");
        o
    }
}

/// Per speaker, `blocks` equal blocks of the frames: (V^2 mean, V*I mean).
fn block_features(cfg: &Config, words: &[i32], blocks: usize) -> Vec<Vec<(f64, Option<f64>)>> {
    let ch = cfg.globals.channels;
    let frames = words.len() / ch;
    let blocks = blocks.clamp(1, frames.max(1));
    cfg.speakers
        .iter()
        .map(|s| {
            let vs = s.vs_scale / 2147483648.0;
            let is = s.is_scale / 2147483648.0;
            (0..blocks)
                .map(|b| {
                    let (lo, hi) = (frames * b / blocks, frames * (b + 1) / blocks);
                    let n = (hi - lo).max(1) as f64;
                    let (mut v2, mut vi) = (0.0, 0.0);
                    for f in lo..hi {
                        let v = words[f * ch + s.vs_chan] as f64 * vs;
                        v2 += v * v;
                        if let Some(ic) = s.is_chan {
                            vi += v * (words[f * ch + ic] as f64 * is);
                        }
                    }
                    (v2 / n, s.is_chan.map(|_| vi / n))
                })
                .collect()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const J700: &str = include_str!("../conf/apple/j700.conf");
    const J316: &str = include_str!("../conf/apple/j316.conf");
    const J413: &str = include_str!("../conf/apple/j413.conf");

    fn cfg(text: &str) -> Config {
        Config::parse(text).unwrap()
    }

    fn idx(c: &Config, name: &str) -> usize {
        c.speakers.iter().position(|s| s.name == name).unwrap()
    }

    #[test]
    fn closed_form_matches_stepping() {
        let mut th = Thermal::new(38.0, 3.3, 26.0, 100.0, 60.0);
        th.t_magnet = 55.0;
        for (p, h) in [(0.0, 5.0), (1.0, 3.3), (2.5, 30.0), (6.3, 0.7)] {
            let (a, b) = th.coil_after(h, 50.0);
            let mut sim = th.clone();
            let n = (h / 0.001).round() as usize;
            for _ in 0..n {
                sim.varying(p, h / n as f64, 50.0);
            }
            assert!((a + b * p - sim.t_coil).abs() < 0.05, "p {p} h {h}: {} vs {}", a + b * p, sim.t_coil);
            let mut exact = th.clone();
            exact.hold(p, h, 50.0);
            assert!((exact.t_coil - sim.t_coil).abs() < 0.05 && (exact.t_magnet - sim.t_magnet).abs() < 0.05);
        }
        let eq = Thermal::new(10.0, 5.0, 10.0, 5.0, 50.0);
        let (a, b) = eq.coil_after(5.0, 50.0);
        let mut sim = eq.clone();
        for _ in 0..5000 {
            sim.varying(1.0, 0.001, 50.0);
        }
        assert!((a + b - sim.t_coil).abs() < 0.05);
    }

    /// The bound `hold` returns is never below the coil or the magnet at
    /// any point of the interval, whatever the start (a hot magnet over a
    /// cool coil included, where the coil peaks inside the interval).
    #[test]
    fn a_constant_power_step_bounds_its_inside() {
        for (c0, m0, p) in [(60.0, 120.0, 0.5), (140.0, 110.0, 0.0), (80.0, 70.0, 3.0), (100.0, 140.0, 0.0), (150.0, 113.0, 39.0)] {
            let mut th = Thermal::new(45.0, 1.3, 50.0, 73.26, c0);
            th.t_magnet = m0;
            let mut fine = th.clone();
            let (bc, bm) = th.hold(p, 20.0, 50.0);
            let (mut mc, mut mm) = (c0, m0);
            for _ in 0..20000 {
                fine.hold(p, 0.001, 50.0);
                mc = mc.max(fine.t_coil);
                mm = mm.max(fine.t_magnet);
            }
            assert!(bc >= mc - 1e-6 && bm >= mm - 1e-6, "start {c0}/{m0} at {p} W: bound {bc}/{bm}, inside {mc}/{mm}");
            assert!((th.t_coil - fine.t_coil).abs() < 1e-6);
        }
    }

    #[test]
    fn exact_cooling_matches_fine_steps() {
        let mut a = Thermal::new(45.0, 1.3, 50.0, 73.26, 140.0);
        a.t_magnet = 110.0;
        let mut b = a.clone();
        a.hold(0.0, 30.0, 50.0);
        for _ in 0..30000 {
            b.varying(0.0, 0.001, 50.0);
        }
        assert!((a.t_coil - b.t_coil).abs() < 0.01 && (a.t_magnet - b.t_magnet).abs() < 0.01, "{a:?} {b:?}");
    }

    #[test]
    fn the_most_power_reaches_the_target_at_the_horizon() {
        let th = Thermal::new(38.0, 3.3, 26.0, 100.0, 70.0);
        let p = th.max_power(115.0, 3.3, 50.0);
        assert!(p > 0.0);
        let mut sim = th.clone();
        sim.hold(p, 3.3, 50.0);
        assert!((sim.t_coil - 115.0).abs() < 0.05, "{}", sim.t_coil);
        let mut hot = Thermal::new(38.0, 3.3, 26.0, 100.0, 116.0);
        hot.t_magnet = 70.0;
        let p = hot.max_power(115.0, 3.3, 50.0);
        assert!(p > 0.0);
        let mut sim = hot.clone();
        sim.hold(p, 3.3, 50.0);
        assert!((sim.t_coil - 115.0).abs() < 0.05, "{}", sim.t_coil);
        // a magnet at the target: nothing brings the coil back in time
        hot.t_magnet = 118.0;
        assert_eq!(hot.max_power(115.0, 3.3, 50.0), 0.0);
    }

    /// speakersafetyd logs these at start on the M1 Pro and the M2 (amp
    /// gain 18.5 dBV): "Min gain: -14.40 dB" (J316 woofers), "-16.18 dB"
    /// (J316 tweeters), "-13.12 dB" (J413 woofers), "-21.10 dB" (J413 tweeters).
    #[test]
    fn asahi_min_gain_matches_speakersafetyd() {
        for (text, want) in [
            (J316, vec![("Left Woofer 1", -14.40), ("Right Woofer 2", -14.40), ("Left Tweeter", -16.18), ("Right Tweeter", -16.18)]),
            (J413, vec![("Left Woofer", -13.12), ("Right Woofer", -13.12), ("Left Tweeter", -21.10), ("Right Tweeter", -21.10)]),
        ] {
            let c = cfg(text);
            let e = Engine::new(&c, false).unwrap();
            let a = e.asahi.as_ref().unwrap();
            for (name, w) in want {
                let g = a[idx(&c, name)].min_gain;
                assert!((g - w).abs() < 0.005, "{name}: {g} vs {w}");
            }
        }
    }

    fn set_asahi(e: &mut Engine, tc: f64, tm: f64) {
        for s in e.asahi.as_mut().unwrap() {
            s.th.t_coil = tc;
            s.th.t_magnet = tm;
            s.coil_peak = tc;
            s.magnet_peak = tm;
        }
    }

    #[test]
    fn asahi_ramp_and_hysteresis_follow_speakersafetyd() {
        let c = cfg(J316);
        let mut e = Engine::new(&c, false).unwrap();
        set_asahi(&mut e, 119.0, 90.0);
        e.govern(0.1, true);
        assert!(e.reductions(Governor::Asahi).iter().all(|&r| r == 0.0));
        // halfway through the window: half of min_gain
        set_asahi(&mut e, 130.0, 80.0);
        e.govern(0.1, true);
        let a = e.asahi.as_ref().unwrap();
        assert!((a[0].gain - a[0].min_gain * 0.5).abs() < 1e-9, "{}", a[0].gain);
        // peak hold: back to 127 C, the hysteresis keeps 130 (127 + 5 > 130)
        set_asahi(&mut e, 127.0, 80.0);
        e.govern(0.1, true);
        let a = e.asahi.as_ref().unwrap();
        assert!((a[0].gain - a[0].min_gain * 0.5).abs() < 1e-9);
        // linked groups: the woofers get the woofer group's largest
        let r = e.reductions(Governor::Asahi);
        assert_eq!(r[idx(&c, "Left Woofer 1")], r[idx(&c, "Right Woofer 2")]);
        assert_eq!(r[idx(&c, "Left Tweeter")], r[idx(&c, "Right Tweeter")]);
    }

    fn word(x: f64, fs: f64) -> i32 {
        (((x / fs * 32768.0).round().clamp(-32768.0, 32767.0)) as i16 as i32) << 16
    }

    /// Review F1 (speakerguardd#6): the asahi model multiplied the sensed
    /// power by its own requested gain, so a gain the card rejected made it
    /// model less heat than the speaker got.  The measured power is what
    /// the sense lines saw, whatever the governor asked for.
    #[test]
    fn the_asahi_model_heats_with_the_sensed_power_whatever_it_asked_for() {
        let c = cfg(J413);
        let k = idx(&c, "Left Tweeter");
        let s = &c.speakers[k];
        let mut words = vec![0i32; c.globals.channels * c.globals.period];
        for f in words.chunks_exact_mut(c.globals.channels) {
            f[s.vs_chan] = word(2.0, 14.0);
            f[s.is_chan.unwrap()] = word(0.5, 3.75);
        }
        let run = |gain: f64| {
            let mut e = Engine::new(&c, false).unwrap();
            set_asahi(&mut e, 100.0, 80.0);
            e.asahi.as_mut().unwrap()[k].gain = gain;
            e.measured(&words, c.globals.period, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
            e.asahi.as_ref().unwrap()[k].th.clone()
        };
        let (asked_nothing, asked_six) = (run(0.0), run(-6.0));
        assert_eq!(asked_nothing, asked_six, "the requested gain changed the modelled heat");
        assert!(asked_six.t_coil > 100.0 + 0.5, "{}", asked_six.t_coil);
    }

    /// The +25 W / -20 W period of review F4 on the J413 left tweeter:
    /// the model peaks at 175 C inside the period and ends at 122 C.  The
    /// hard limit latches at the sample that crosses it.
    #[test]
    fn an_intra_period_crossing_latches_the_asahi_model() {
        let c = cfg(J413);
        let k = idx(&c, "Left Tweeter");
        let s = &c.speakers[k];
        let mut w = vec![0i32; c.globals.channels * c.globals.period];
        for (i, f) in w.chunks_exact_mut(c.globals.channels).enumerate() {
            let p = if i < 1920 { 25.0 } else { -20.0 };
            f[s.vs_chan] = word(7.0, 14.0);
            f[s.is_chan.unwrap()] = word(p / 7.0, 3.75);
        }
        let mut e = Engine::new(&c, false).unwrap();
        let tm = 50.0 + (119.0 - 50.0) * 200.0 / 304.9;
        set_asahi(&mut e, 119.0, tm);
        e.check(&w, 48000.0).expect("the period's mean power is positive");
        e.measured(&w, c.globals.period, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
        let a = &e.asahi.as_ref().unwrap()[k];
        assert!(a.coil_peak > 170.0, "peak {}", a.coil_peak);
        assert!(a.hard && e.hard(Governor::Asahi), "the crossing did not latch");
        // the samples after the crossing (122 C at the end) do not undo it
        assert!(a.th.t_coil >= 150.0, "ends at {}", a.th.t_coil);
        assert!(!e.asahi.as_mut().unwrap()[k].clear_hard_if_cool(), "cleared above the ceiling");
        // the same stimulus a little quieter peaks under the limit: no latch,
        // and the end is what the samples leave
        let mut quiet = w.clone();
        for f in quiet.chunks_exact_mut(c.globals.channels) {
            f[s.is_chan.unwrap()] /= 4;
        }
        let mut e = Engine::new(&c, false).unwrap();
        set_asahi(&mut e, 119.0, tm);
        e.measured(&quiet, c.globals.period, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
        let a = &e.asahi.as_ref().unwrap()[k];
        assert!(!a.hard && a.coil_peak < 150.0 && a.th.t_coil < a.coil_peak, "{} / {}", a.th.t_coil, a.coil_peak);
    }

    /// The same for the magnet, and for arithmetic that goes nonfinite.
    #[test]
    fn a_magnet_crossing_or_a_nonfinite_sample_latches_too() {
        let mut c = cfg(J413);
        let k = idx(&c, "Left Tweeter");
        // a coil that barely follows: only the magnet can cross
        c.speakers[k].vi_tr_coil = 0.001;
        let s = c.speakers[k].clone();
        let mut w = vec![0i32; c.globals.channels * c.globals.period];
        for (i, f) in w.chunks_exact_mut(c.globals.channels).enumerate() {
            let p = if i < 1500 { 25.0 } else { -25.0 };
            f[s.vs_chan] = word(7.0, 14.0);
            f[s.is_chan.unwrap()] = word(p / 7.0, 3.75);
        }
        let mut e = Engine::new(&c, false).unwrap();
        set_asahi(&mut e, 60.0, 149.99);
        e.measured(&w, c.globals.period, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
        let a = &e.asahi.as_ref().unwrap()[k];
        assert!(a.magnet_peak >= 150.0 && a.hard, "magnet peak {} not latched", a.magnet_peak);
        assert!(a.th.t_magnet >= 150.0 && a.th.t_coil < 150.0, "ends at {} / {}", a.th.t_coil, a.th.t_magnet);
        // nonfinite arithmetic latches, and the guard's finite check refuses it
        let mut e = Engine::new(&c, false).unwrap();
        set_asahi(&mut e, 60.0, 60.0);
        e.asahi.as_mut().unwrap()[k].th.tr_magnet = f64::INFINITY;
        e.measured(&w, c.globals.period, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
        assert!(e.asahi.as_ref().unwrap()[k].hard && !e.finite());
    }

    /// Review F3: a period whose mean V*I is below -0.01 W is bad data.
    #[test]
    fn negative_or_nonfinite_sensed_power_is_not_a_measurement() {
        let c = cfg(J413);
        let mut e = Engine::new(&c, false).unwrap();
        let mut w = vec![0i32; c.globals.channels * c.globals.period];
        for f in w.chunks_exact_mut(c.globals.channels) {
            f[1] = word(2.0, 14.0);
            f[0] = word(-0.5, 3.75);
        }
        assert!(e.check(&w, 48000.0).is_err());
        for f in w.chunks_exact_mut(c.globals.channels) {
            f[0] = word(0.5, 3.75);
        }
        assert!(e.check(&w, 48000.0).is_ok());
        let mut nan = c.clone();
        nan.speakers[0].is_scale = f64::NAN;
        let mut e = Engine::new(&nan, false).unwrap();
        assert!(e.check(&w, 48000.0).is_err());
    }

    /// Run `gov` as the governor that writes the card, from ambient: each
    /// period the card applies its reduction and the sensed power is the
    /// program's `pre_v_ms` (and `vi_ratio` of it as measured V*I) after
    /// that gain.  Returns the hottest coil each model reached after the
    /// first second.  (PR #6 started the asahi models at 119 C here and
    /// checked only the end: the first full-scale period at 0 dB took the
    /// J316 tweeter's model to about 174 C, which the latch now sees.)
    fn govern(c: &Config, gov: Governor, pre_v_ms: f64, vi_ratio: Option<f64>, secs: f64) -> (Engine, f64, f64) {
        let mut e = Engine::new(c, true).unwrap();
        let dt = 4096.0 / 48000.0;
        let (mut hot_ff, mut hot_vi): (f64, f64) = (0.0, 0.0);
        let mut t = 0.0;
        while t < secs {
            let r = e.reductions(gov);
            let blocks: Vec<Vec<(f64, Option<f64>)>> = c
                .speakers
                .iter()
                .zip(&r)
                .map(|(s, &r)| {
                    let k = 10f64.powf(-r / 10.0);
                    vec![(pre_v_ms * k, vi_ratio.map(|x| x * pre_v_ms / s.r_ff * k)); SUB_BLOCKS]
                })
                .collect();
            let ta = c.globals.t_ambient;
            for (k, b) in blocks.iter().enumerate() {
                e.unified.as_mut().unwrap()[k].blocks(b, dt, &[(r[k], r[k])], ta);
                if let (Some(a), Some(p)) = (e.asahi.as_mut(), b[0].1) {
                    // the asahi model on the period's mean power
                    let (cc, mm) = a[k].th.varying(p * ASAHI_MARGIN, dt, ta);
                    a[k].note(cc, mm);
                }
            }
            e.govern(dt, true);
            for k in 0..c.speakers.len() {
                let (_, ff, vi) = e.temps(k);
                if t > 1.0 {
                    hot_ff = hot_ff.max(ff);
                    if vi.is_finite() {
                        hot_vi = hot_vi.max(vi);
                    }
                }
            }
            t += dt;
        }
        (e, hot_ff, hot_vi)
    }

    #[test]
    fn unified_holds_the_j700_coil_at_its_target_under_full_scale() {
        let c = cfg(J700);
        // a full-scale sine at 7.3 V: 26.6 V^2 mean square, 6.3 W into 4.2 ohm
        let v_ms = 7.3f64.powi(2) / 2.0;
        let (e, hot, _) = govern(&c, Governor::Unified, v_ms, None, 900.0);
        let target = 135.0 - 15.0 - 5.0;
        assert!(hot <= target + 0.3, "coil reached {hot}");
        let (_, ff, _) = e.temps(0);
        assert!((ff - target).abs() < 1.0, "settled at {ff}");
        assert!(!e.hard(Governor::Unified));
    }

    #[test]
    fn unified_enforces_the_j700_power_budget() {
        let c = cfg(J700);
        let v_ms = 7.3f64.powi(2) / 2.0;
        let (e, _, _) = govern(&c, Governor::Unified, v_ms, None, 1.5);
        // 6.3 W against the 2.8154 W one-second budget: at least 3.5 dB
        let r = e.reductions(Governor::Unified)[0];
        assert!(r >= 10.0 * (6.3443 / 2.8154f64).log10() - 0.05, "{r}");
    }

    #[test]
    fn unified_on_a_vi_machine_never_lets_either_model_past_the_target() {
        for text in [J316, J413] {
            let c = cfg(text);
            // a full-scale sine at the amp's 18.5 dBV: 70.8 V^2
            let v_ms = 10f64.powf(18.5 / 10.0);
            for ratio in [0.5, 1.0, 1.5] {
                let (_, hot_ff, hot_vi) = govern(&c, Governor::Unified, v_ms, Some(ratio), 600.0);
                assert!(hot_ff <= 140.3, "ff {hot_ff} at ratio {ratio}");
                assert!(hot_vi <= 140.3, "vi {hot_vi} at ratio {ratio}");
            }
        }
    }

    #[test]
    fn measured_power_can_only_tighten() {
        let c = cfg(J316);
        let v_ms = 10f64.powf(18.5 / 10.0) / 10.0;
        let (lo, _, _) = govern(&c, Governor::Unified, v_ms, Some(0.1), 120.0);
        let (hi, _, _) = govern(&c, Governor::Unified, v_ms, Some(3.0), 120.0);
        let a = lo.reductions(Governor::Unified);
        let b = hi.reductions(Governor::Unified);
        assert!(b.iter().zip(&a).all(|(b, a)| b >= a), "{b:?} vs {a:?}");
    }

    /// A program the ramp can follow (20 dB under full scale) stays under
    /// the hard limit without ever latching.  At full scale, 22 W into the
    /// J316 tweeter adds 50 K a period, faster than speakersafetyd's ramp
    /// reacts: the model crosses and latches, and the guard hands the card
    /// to the kernel (PR #6 checked only the end and missed the crossing).
    #[test]
    fn asahi_governing_keeps_its_model_under_the_hard_limit() {
        for text in [J316, J413] {
            let c = cfg(text);
            let v_ms = 10f64.powf(18.5 / 10.0) / 100.0;
            let (e, _, _) = govern(&c, Governor::Asahi, v_ms, Some(1.0), 600.0);
            // the latch never set: no model crossed at any sample
            assert!(!e.hard(Governor::Asahi));
            for k in 0..c.speakers.len() {
                let (ac, _, _) = e.temps(k);
                assert!(ac < 150.0, "{ac}");
            }
            let (e, _, _) = govern(&c, Governor::Asahi, v_ms * 100.0, Some(1.0), 5.0);
            // the J413's ramp catches its tweeter at 140 C after the first
            // full-scale period; the J316's does not
            assert_eq!(e.hard(Governor::Asahi), text == J316, "full scale on {}", c.speakers[0].name);
        }
    }

    #[test]
    fn a_muted_card_does_not_amplify_noise() {
        let c = cfg(J316);
        let mut e = Engine::new(&c, true).unwrap();
        let ta = c.globals.t_ambient;
        for _ in 0..200 {
            for u in e.unified.as_mut().unwrap() {
                u.blocks(&[(5e-5, Some(1e-5))], 0.085, &[(99.5, 99.5)], ta);
            }
            e.govern(0.085, true);
        }
        assert!(e.temps(0).1 < 51.0, "{}", e.temps(0).1);
        // the power before the gain is rebuilt up to PRE_GAIN_MAX_DB only
        assert!(e.unified.as_ref().unwrap()[0].mean_pre(1.0) < 5e-5 / 2.7 * 1001.0);
    }

    #[test]
    fn the_extractor_reads_v_and_i_and_the_low_band_resistance() {
        let c = cfg(J413);
        let mut x = Extractor::new(4);
        let n = 4096;
        let mut words = vec![0i32; n * 8];
        // a 46.875 Hz tone (4 cycles a period): 2 V peak across 4 ohm on the
        // left woofer (V on 1, I on 0)
        let lw = idx(&c, "Left Woofer");
        let lt = idx(&c, "Left Tweeter");
        for k in 0..n {
            let ph = 2.0 * std::f64::consts::PI * 46.875 * k as f64 / 48000.0;
            let v = 2.0 * ph.sin();
            let i = v / 4.0;
            words[k * 8 + 1] = (v / 14.0 * 2147483648.0) as i32;
            words[k * 8] = (i / 3.75 * 2147483648.0) as i32;
        }
        let mut f = x.extract(&c, &words, 48000.0);
        for _ in 0..3 {
            f = x.extract(&c, &words, 48000.0);
        }
        assert!((f[lw].v_ms - 2.0).abs() < 0.01, "{}", f[lw].v_ms);
        assert!((f[lw].vi.unwrap() - 0.5).abs() < 0.01);
        let r = f[lw].r_lf.unwrap();
        assert!((r - 4.0).abs() < 0.05, "{r}");
        assert!(f[lt].r_lf.is_none());
    }

    /// The worst case of the measured-power models is a full-scale square
    /// wave into the smallest impedance the speaker can present: the J316
    /// woofer's 2.7 ohm DC resistance, not its 3.6 ohm nominal impedance.
    #[test]
    fn the_measured_power_worst_case_uses_the_dc_resistance() {
        let c = cfg(J316);
        let k = idx(&c, "Left Woofer 1");
        let mut e = Engine::new(&c, true).unwrap();
        let before = e.asahi.as_ref().unwrap()[k].th.clone();
        e.worst(1.0, &vec![0.0; c.speakers.len()]);
        let v = 2f64.sqrt() * 10f64.powf(18.5 / 20.0);
        let mut want = before.clone();
        want.hold(v * v / 2.7, 1.0, c.globals.t_ambient);
        assert!((e.asahi.as_ref().unwrap()[k].th.t_coil - want.t_coil).abs() < 1e-9);
        // and a group-linked reduction is the group's largest
        let r = link(&c, &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        assert_eq!(r[idx(&c, "Left Woofer 1")], 6.0);
        assert_eq!(r[idx(&c, "Left Tweeter")], 4.0);
    }
}

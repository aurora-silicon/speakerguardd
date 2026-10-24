// SPDX-License-Identifier: MIT
//! The governors besides the legacy one, and the shadow bookkeeping.
//!
//! Every governor keeps its own thermal models and is fed the power its own
//! gains would have let through: the power before the card's gain,
//! `P_pre = P_sensed * 10^(applied/10)`, times `10^(-r/10)` for the
//! governor's own reduction `r`.  For the governor that writes the card that
//! is exactly the sensed power; for the others it is a counterfactual, which
//! is what lets a shadow run say how warm the coils would have got, and how
//! much louder the music would have been, under a governor that did not
//! write the card.
//!
//! * `asahi`: the behaviour Asahi Linux's speakersafetyd documents and
//!   ships (measured V*I power into the two-node model; a linear ramp from 0
//!   at `t_limit - t_window` to its `min_gain` at `t_limit`, with
//!   `min_gain = 10 log10(max_pwr / peak_pwr)`; peak-hold hysteresis on the
//!   coil and magnet), reimplemented from its description.  Credit: the
//!   Asahi Linux contributors, speakersafetyd (MIT up to 2.0.2).  Its
//!   `t_limit` is our working limit (`t_limit - t_headroom` here) and its
//!   panic margin our hard limit.  Differences, all toward protection:
//!   unmeasured time is charged at the worst case instead of skipped, and
//!   faults hand the speakers to the kernel's safe level instead of a panic.
//! * `unified`: the open-loop (feed-forward) model is always on, as on
//!   macOS: power from the amplifier voltage against the speaker's DC
//!   resistance `r_ff`.  Where V/I sense exists, the measured-power model
//!   runs too and the more restrictive of the two governs (it can only
//!   tighten).  The regulator lets through the most power that keeps each
//!   model's coil at or under `t_limit - t_headroom - t_ctrl_headroom` over
//!   the next `horizon_s`, attacks at once and releases with `release_s`;
//!   the 1 s and 60 s power budgets hold on the power before the gain.
//! * Resistance thermometry from low-band V/I (`R = sum(v i) / sum(i^2)`) is
//!   only measured and logged.

use std::collections::VecDeque;
use std::fmt::Write as _;

use crate::config::{Config, Globals, Governor, Speaker};

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

    pub fn step(&mut self, power: f64, dt: f64, t_ambient: f64) {
        let k_magnet = 1.0 - (-dt / self.tau_magnet).exp();
        self.t_magnet += (t_ambient + power * self.tr_magnet - self.t_magnet) * k_magnet;
        let k_coil = 1.0 - (-dt / self.tau_coil).exp();
        self.t_coil += (self.t_magnet + power * self.tr_coil - self.t_coil) * k_coil;
    }

    /// The coil after `h` seconds at a constant `power`, in closed form:
    /// `alpha + beta * power`.
    pub fn coil_after(&self, h: f64, t_ambient: f64) -> (f64, f64) {
        let e_c = (-h / self.tau_coil).exp();
        let e_m = (-h / self.tau_magnet).exp();
        // coil drive A + B e^(-t/tau_m); particular K e^(-t/tau_m), K = B r
        let r = if (self.tau_magnet - self.tau_coil).abs() > 1e-9 {
            self.tau_magnet / (self.tau_magnet - self.tau_coil)
        } else {
            // equal time constants: the limit of r (e_m - e_c) is h/tau e^(-h/tau)
            return self.coil_after_equal(h, t_ambient);
        };
        let alpha = self.t_coil * e_c + t_ambient * (1.0 - e_c) + (self.t_magnet - t_ambient) * r * (e_m - e_c);
        let beta = (self.tr_magnet + self.tr_coil) * (1.0 - e_c) - self.tr_magnet * r * (e_m - e_c);
        (alpha, beta)
    }

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
    /// first falls toward it.
    /// Over the target already, it is the power that brings the coil back
    /// to the target by the end of the horizon (0 when even silence would
    /// not).
    pub fn max_power(&self, target: f64, h: f64, t_ambient: f64) -> f64 {
        let (alpha, beta) = self.coil_after(h, t_ambient);
        if beta <= 0.0 {
            return 0.0;
        }
        ((target - alpha) / beta).max(0.0)
    }

    /// Cool for `dt` seconds without power, exactly (one stepped lag over a
    /// long gap would let the coil chase the magnet's end temperature).
    pub fn cool_exact(&mut self, dt: f64, t_ambient: f64) {
        let (alpha, _) = self.coil_after(dt, t_ambient);
        self.t_magnet = t_ambient + (self.t_magnet - t_ambient) * (-dt / self.tau_magnet).exp();
        self.t_coil = alpha;
    }

    pub fn finite(&self) -> bool {
        self.t_coil.is_finite() && self.t_magnet.is_finite()
    }

    fn hottest(&self) -> f64 {
        self.t_coil.max(self.t_magnet)
    }
}

/// What one sense period says about one speaker (power after the card's gain).
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

/// The asahi models see this much more power than measured (+0.02 dB): on
/// recorded sense data their gains then never ask for less than
/// speakersafetyd's (the f32-versus-f64 rounding was worth up to 0.04 C).
pub const ASAHI_MARGIN: f64 = 1.005;

/// A reduction that lets nothing through (dB).
pub const MUTE_DB: f64 = 120.0;

/// The most card gain undone when reconstructing the power before it (dB).
pub const PRE_MAX_DB: f64 = 30.0;

/// Below this the low band carries too little current to read a resistance.
const RTH_I_MIN: f64 = 0.02;
/// Low band for the thermometer: well below laptop drivers' resonance.
const RTH_FC: f64 = 120.0;

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

    /// The features of one complete period of interleaved words (S32 full
    /// scale; S16 sense words are widened by << 16).
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

#[derive(Debug, Clone)]
struct AsahiSpeaker {
    th: Thermal,
    coil_hyst: f64,
    magnet_hyst: f64,
    min_gain: f64,
    /// dB, <= 0
    gain: f64,
}

#[derive(Debug, Clone)]
struct UnifiedSpeaker {
    ff: Thermal,
    vi: Option<Thermal>,
    /// (dt, power before the gain) per step, newest last, one minute.
    history: VecDeque<(f64, f64)>,
    history_s: f64,
    /// dB, >= 0
    reduction: f64,
    /// The latest power before the gain: feed-forward and measured (W).
    last_ff_pre: f64,
    last_vi_pre: Option<f64>,
    /// Sensed (after-gain) feed-forward power, one minute: for the
    /// log-only "legacy-0.1.1" budget estimate.
    post_history: VecDeque<(f64, f64)>,
    post_history_s: f64,
}

impl UnifiedSpeaker {
    fn mean_pre(&self, seconds: f64) -> f64 {
        let (mut t, mut e) = (0.0, 0.0);
        for (dt, p) in self.history.iter().rev() {
            let take = dt.min(seconds - t);
            if take <= 0.0 {
                break;
            }
            e += p * take;
            t += take;
        }
        e / seconds
    }
}

/// Peak temperatures and reduction statistics since the last report.
#[derive(Debug, Clone, Default)]
pub struct Stats {
    pub playing_s: f64,
    /// time-weighted sums of each governor's reduction while playing
    pub sum_legacy: f64,
    pub sum_asahi: f64,
    pub sum_unified: f64,
    pub max_coil_ff: f64,
    pub max_coil_vi: f64,
    pub max_coil_asahi: f64,
}

pub struct Engine {
    pub cfg: Config,
    asahi: Option<Vec<AsahiSpeaker>>,
    unified: Vec<UnifiedSpeaker>,
    pub extractor: Extractor,
    pub last: Vec<Feature>,
    /// Since the last shadow line.
    pub stats: Stats,
    /// Since start.
    pub total: Stats,
    /// Replay only: play the recording this much louder (dB).
    pub extra_gain_db: f64,
    /// The asahi models were already stepped sample by sample this period.
    asahi_stepped: bool,
}

/// How the model starts: no state (cold boot), saved state, or unreadable.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Start {
    Cold,
    Hot,
}

fn db(ratio: f64) -> f64 {
    10.0 * ratio.log10()
}

impl Engine {
    pub fn new(cfg: &Config, start: Start) -> Engine {
        let g = &cfg.globals;
        let asahi = if cfg.speakers.iter().all(|s| s.is_chan.is_some()) && g.amp_gain_dbv > 0.0 {
            Some(cfg.speakers.iter().map(|s| Self::asahi_speaker(s, g, start)).collect())
        } else {
            None
        };
        let unified = cfg
            .speakers
            .iter()
            .map(|s| {
                let t = match start {
                    Start::Cold => g.t_ambient,
                    Start::Hot => s.t_limit - s.t_headroom,
                };
                let mut ff = Thermal::new(s.tr_coil, s.tau_coil, s.tr_magnet, s.tau_magnet, t);
                ff.t_magnet = g.t_ambient + (t - g.t_ambient) * s.tr_magnet / (s.tr_magnet + s.tr_coil);
                let vi = s.is_chan.map(|_| {
                    let mut v = Thermal::new(s.vi_tr_coil, s.vi_tau_coil, s.vi_tr_magnet, s.vi_tau_magnet, t);
                    v.t_magnet = g.t_ambient + (t - g.t_ambient) * s.vi_tr_magnet / (s.vi_tr_magnet + s.vi_tr_coil);
                    v
                });
                UnifiedSpeaker { ff, vi, history: VecDeque::new(), history_s: 0.0, reduction: 0.0, last_ff_pre: 0.0, last_vi_pre: None, post_history: VecDeque::new(), post_history_s: 0.0 }
            })
            .collect();
        let n = cfg.speakers.len();
        Engine {
            cfg: cfg.clone(),
            asahi,
            unified,
            extractor: Extractor::new(n),
            last: vec![Feature::default(); n],
            stats: Stats::default(),
            total: Stats::default(),
            extra_gain_db: 0.0,
            asahi_stepped: false,
        }
    }

    fn asahi_speaker(s: &Speaker, g: &Globals, start: Start) -> AsahiSpeaker {
        let work = s.t_limit - s.t_headroom;
        let t_coil = match start {
            // speakersafetyd's cold boot: warm, just short of limiting
            Start::Cold => work - g.t_window - 1.0,
            // its warm restart: at the limit
            Start::Hot => work,
        };
        let t_magnet = g.t_ambient + (t_coil - g.t_ambient) * s.vi_tr_magnet / (s.vi_tr_magnet + s.vi_tr_coil);
        let max_pwr = (work - g.t_ambient) / (s.vi_tr_magnet + s.vi_tr_coil);
        // worst-case peak power is twice a full-scale sine's
        let peak_pwr = 10f64.powf(g.amp_gain_dbv / 10.0) / s.z_nominal * 2.0;
        let min_gain = db(max_pwr / peak_pwr).min(0.0);
        let mut th = Thermal::new(s.vi_tr_coil, s.vi_tau_coil, s.vi_tr_magnet, s.vi_tau_magnet, t_coil);
        th.t_magnet = t_magnet;
        AsahiSpeaker { th, coil_hyst: 0.0, magnet_hyst: 0.0, min_gain, gain: 0.0 }
    }

    pub fn has_asahi(&self) -> bool {
        self.asahi.is_some()
    }

    /// Coil temperatures of speaker `k`: asahi, unified feed-forward,
    /// unified measured-power (NaN where a model does not exist).
    pub fn temps(&self, k: usize) -> (f64, f64, f64) {
        let u = &self.unified[k];
        (
            self.asahi.as_ref().map_or(f64::NAN, |a| a[k].th.t_coil),
            u.ff.t_coil,
            u.vi.as_ref().map_or(f64::NAN, |v| v.t_coil),
        )
    }

    /// Continue from a saved engine state (see `encode`), cooled already.
    pub fn restore(&mut self, saved: &SavedEngine) {
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            let Some(x) = saved.speakers.iter().find(|x| x.name == state_name(&s.name)) else { continue };
            let u = &mut self.unified[k];
            u.ff.t_coil = x.ff.0;
            u.ff.t_magnet = x.ff.1;
            if let (Some(vi), Some(v)) = (u.vi.as_mut(), x.vi) {
                vi.t_coil = v.0;
                vi.t_magnet = v.1;
            }
            if let (Some(a), Some(v)) = (self.asahi.as_mut(), x.asahi) {
                a[k].th.t_coil = v.0;
                a[k].th.t_magnet = v.1;
            }
            u.history.clear();
            u.history.push_back((60.0, x.mean_60s_pre));
            u.history_s = 60.0;
        }
    }

    /// Cool every model by `dt` seconds without power.
    pub fn cool(&mut self, dt: f64) {
        let ta = self.cfg.globals.t_ambient;
        for u in self.unified.iter_mut() {
            u.ff.cool_exact(dt, ta);
            if let Some(v) = u.vi.as_mut() {
                v.cool_exact(dt, ta);
            }
        }
        if let Some(a) = self.asahi.as_mut() {
            for s in a.iter_mut() {
                s.th.cool_exact(dt, ta);
            }
        }
    }

    /// The speakers are closed: cool, and let the reductions settle.
    pub fn idle(&mut self, dt: f64) {
        self.cool(dt);
        for u in self.unified.iter_mut() {
            u.history.push_back((dt, 0.0));
            u.history_s += dt;
            u.last_ff_pre = 0.0;
            u.last_vi_pre = u.vi.as_ref().map(|_| 0.0);
        }
        self.trim_history();
        self.govern_all(dt, false);
    }

    /// Time nobody measured, charged at the most each governor's own gain
    /// lets through.
    pub fn blind(&mut self, dt: f64) {
        let g = self.cfg.globals.clone();
        let ta = g.t_ambient;
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            let v = s.v_worst(&g);
            let u = &mut self.unified[k];
            let a = 10f64.powf(-u.reduction / 10.0);
            u.ff.step(v * v / s.r_ff * a, dt, ta);
            if let Some(vi) = u.vi.as_mut() {
                vi.step(v * v / s.z_nominal * a, dt, ta);
            }
            u.history.push_back((dt, v * v / s.r_ff));
            u.history_s += dt;
            u.last_ff_pre = v * v / s.r_ff;
            u.last_vi_pre = u.vi.as_ref().map(|_| v * v / s.z_nominal);
            if let Some(asahi) = self.asahi.as_mut() {
                let sa = &mut asahi[k];
                sa.th.step(v * v / s.z_nominal * 10f64.powf(sa.gain / 10.0), dt, ta);
            }
        }
        self.trim_history();
        self.govern_all(dt, true);
    }

    fn trim_history(&mut self) {
        for u in self.unified.iter_mut() {
            while u.history_s > 60.0 {
                match u.history.pop_front() {
                    Some((d, _)) => u.history_s -= d,
                    None => break,
                }
            }
        }
    }

    /// One complete sense period: `post` are the sensed features, `applied`
    /// the reduction (dB) the card applied to each speaker during it, and
    /// `words` the period itself (V/I machines: the asahi models integrate
    /// sample by sample, as speakersafetyd does).
    pub fn period(&mut self, post: &[Feature], applied: &[f64], dt: f64, legacy_reduction: f64, words: Option<&[i32]>) {
        if let (Some(words), Some(asahi)) = (words, self.asahi.as_mut()) {
            let ch = self.cfg.globals.channels;
            let frames = words.len() / ch;
            let step = if frames > 0 { dt / frames as f64 } else { dt };
            let ta = self.cfg.globals.t_ambient;
            for (k, s) in self.cfg.speakers.iter().enumerate() {
                let Some(ic) = s.is_chan else { continue };
                let sa = &mut asahi[k];
                // the power before the card's gain, under this governor's own
                // gain, with ASAHI_MARGIN on top so that rounding never leaves
                // this model cooler than speakersafetyd's own arithmetic
                let factor = ASAHI_MARGIN * 10f64.powf((applied[k].min(PRE_MAX_DB) + self.extra_gain_db + sa.gain) / 10.0);
                let (vs, is) = (s.vs_scale / 2147483648.0, s.is_scale / 2147483648.0);
                let ac = step / (sa.th.tau_coil + step);
                let am = step / (sa.th.tau_magnet + step);
                let (mut tc, mut tm) = (sa.th.t_coil, sa.th.t_magnet);
                for f in words.chunks_exact(ch) {
                    let p = f[s.vs_chan] as f64 * vs * (f[ic] as f64 * is) * factor;
                    let coil_target = tm + p * sa.th.tr_coil;
                    let magnet_target = ta + p * sa.th.tr_magnet;
                    tc = coil_target * ac + tc * (1.0 - ac);
                    tm = magnet_target * am + tm * (1.0 - am);
                }
                sa.th.t_coil = tc;
                sa.th.t_magnet = tm;
            }
            self.asahi_stepped = true;
        }
        let extra = self.extra_gain_db;
        // the power before the card's gain
        // the power before the card's gain
        let pre: Vec<Feature> = post
            .iter()
            .zip(applied)
            .map(|(f, &a)| {
                // Under a muted or near-muted card (the SN012776 machines
                // lock to -100 dB) the sense data is noise: no gain past
                // PRE_MAX_DB is reconstructed.
                let k = 10f64.powf((a.min(PRE_MAX_DB) + extra) / 10.0);
                Feature { v_ms: f.v_ms * k, vi: f.vi.map(|p| p * k), i_ms: f.i_ms.map(|p| p * k), r_lf: f.r_lf }
            })
            .collect();
        self.last = post.to_vec();
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            let u = &mut self.unified[k];
            u.post_history.push_back((dt, post[k].v_ms / s.z_nominal));
            u.post_history_s += dt;
            while u.post_history_s > 60.0 {
                match u.post_history.pop_front() {
                    Some((d, _)) => u.post_history_s -= d,
                    None => break,
                }
            }
        }
        self.advance(&pre, dt, Some(legacy_reduction));
    }

    /// What speakerguardd 0.1.1's power budget would ask for on the sensed
    /// data: the budgets computed on the power after the card's gain (it
    /// settles at about half the dB the budget needs).  Logged only; it
    /// never governs.
    pub fn legacy_011_budget(&self) -> f64 {
        let mut db_max: f64 = 0.0;
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            let u = &self.unified[k];
            for (seconds, limit) in [(1.0, s.p_limit_1s), (60.0, s.p_limit_60s)] {
                if limit > 0.0 {
                    let (mut t, mut e) = (0.0, 0.0);
                    for (dt, p) in u.post_history.iter().rev() {
                        let take = dt.min(seconds - t);
                        if take <= 0.0 {
                            break;
                        }
                        e += p * take;
                        t += take;
                    }
                    let mean = e / seconds;
                    if mean > limit {
                        db_max = db_max.max(db(mean / limit));
                    }
                }
            }
        }
        db_max
    }

    fn advance(&mut self, pre: &[Feature], dt: f64, legacy_reduction: Option<f64>) {
        let ta = self.cfg.globals.t_ambient;
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            let f = pre[k];
            let u = &mut self.unified[k];
            let a = 10f64.powf(-u.reduction / 10.0);
            let p_ff = f.v_ms / s.r_ff;
            u.ff.step(p_ff * a, dt, ta);
            if let (Some(vi), Some(p)) = (u.vi.as_mut(), f.vi) {
                vi.step(p.max(0.0) * a, dt, ta);
            }
            u.history.push_back((dt, p_ff));
            u.history_s += dt;
            u.last_ff_pre = p_ff;
            u.last_vi_pre = f.vi.map(|p| p.max(0.0));
            if let Some(asahi) = self.asahi.as_mut() {
                if !self.asahi_stepped {
                    let sa = &mut asahi[k];
                    let p = f.vi.unwrap_or(0.0).max(0.0) * 10f64.powf(sa.gain / 10.0);
                    sa.th.step(p, dt, ta);
                }
            }
        }
        self.asahi_stepped = false;
        self.trim_history();
        let playing = legacy_reduction.is_some();
        self.govern_all(dt, playing);
        if let Some(lr) = legacy_reduction {
            let r = self.reductions(Governor::Unified);
            let ra = self.asahi.as_ref().map(|_| self.reductions(Governor::Asahi));
            for st in [&mut self.stats, &mut self.total] {
                st.playing_s += dt;
                st.sum_legacy += lr * dt;
                st.sum_unified += r.iter().cloned().fold(0.0, f64::max) * dt;
                if let Some(ra) = &ra {
                    st.sum_asahi += ra.iter().cloned().fold(0.0, f64::max) * dt;
                }
            }
        }
        for k in 0..self.cfg.speakers.len() {
            let u = &self.unified[k];
            let vi = u.vi.as_ref().map_or(f64::MIN, |v| v.t_coil);
            let asa = self.asahi.as_ref().map_or(f64::MIN, |a| a[k].th.t_coil);
            for st in [&mut self.stats, &mut self.total] {
                st.max_coil_ff = st.max_coil_ff.max(u.ff.t_coil);
                st.max_coil_vi = st.max_coil_vi.max(vi);
                st.max_coil_asahi = st.max_coil_asahi.max(asa);
            }
        }
    }

    fn govern_all(&mut self, dt: f64, playing: bool) {
        let g = self.cfg.globals.clone();
        let ta = g.t_ambient;
        // asahi: speakersafetyd's ramp, per speaker
        if let Some(asahi) = self.asahi.as_mut() {
            for (k, s) in self.cfg.speakers.iter().enumerate() {
                let sa = &mut asahi[k];
                let work = s.t_limit - s.t_headroom;
                sa.coil_hyst = sa.coil_hyst.max(sa.th.t_coil).min(sa.th.t_coil + g.t_hysteresis);
                sa.magnet_hyst = sa.magnet_hyst.max(sa.th.t_magnet).min(sa.th.t_magnet + g.t_hysteresis);
                let temp = sa.coil_hyst.max(sa.magnet_hyst);
                let frac = (temp - (work - g.t_window)) / g.t_window;
                let mut gain = sa.min_gain * frac.max(0.0);
                if gain > -0.01 {
                    gain = 0.0;
                }
                sa.gain = gain;
            }
        }
        // unified: predictive regulator per speaker
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            let u = &mut self.unified[k];
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
                    // nothing may pass: as good as muted (and far past
                    // --max-reduction, so the kernel takes over)
                    if pmax <= 0.0 {
                        MUTE_DB
                    } else {
                        db(p_pre / pmax).clamp(0.0, MUTE_DB)
                    }
                };
                let mut thermal = need(u.last_ff_pre, &u.ff);
                // where V/I sense exists the measured-power model can only
                // tighten the feed-forward one
                if let (Some(vi), Some(p)) = (u.vi.as_ref(), u.last_vi_pre) {
                    thermal = thermal.max(need(p, vi));
                }
                let mut power: f64 = 0.0;
                for (seconds, limit) in [(1.0, s.p_limit_1s), (60.0, s.p_limit_60s)] {
                    if limit > 0.0 {
                        let mean = u.mean_pre(seconds);
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
            // No silent cap: past --max-reduction the guard hands the
            // speakers to the kernel's safe level instead.
            let capped = wanted.max(0.0);
            u.reduction = if capped >= u.reduction {
                capped
            } else {
                capped + (u.reduction - capped) * (-dt / g.release_s).exp()
            };
            if u.reduction < 0.01 {
                u.reduction = 0.0;
            }
        }
    }

    /// Per-speaker reduction (dB) a governor asks for, group-linked: every
    /// speaker of a group gets the group's largest.
    pub fn reductions(&self, gov: Governor) -> Vec<f64> {
        let own: Vec<f64> = match gov {
            Governor::Unified => self.unified.iter().map(|u| u.reduction).collect(),
            Governor::Asahi => match &self.asahi {
                Some(a) => a.iter().map(|s| -s.gain).collect(),
                None => vec![0.0; self.cfg.speakers.len()],
            },
            Governor::Legacy => vec![0.0; self.cfg.speakers.len()],
        };
        self.cfg
            .speakers
            .iter()
            .map(|s| {
                self.cfg
                    .speakers
                    .iter()
                    .zip(&own)
                    .filter(|(o, _)| o.group == s.group)
                    .map(|(_, &r)| r)
                    .fold(0.0, f64::max)
            })
            .collect()
    }

    /// A model of `gov` past its hard limit (`t_limit`): the daemon must
    /// hand the speakers to the kernel.
    pub fn hard_fault(&self, gov: Governor) -> Option<String> {
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            let hot = match gov {
                Governor::Unified => {
                    let u = &self.unified[k];
                    u.ff.hottest().max(u.vi.as_ref().map_or(f64::MIN, Thermal::hottest))
                }
                Governor::Asahi => self.asahi.as_ref().map_or(f64::MIN, |a| a[k].th.hottest()),
                Governor::Legacy => f64::MIN,
            };
            if hot > s.t_limit {
                return Some(format!("{}: {} model at {hot:.1} C, over its {:.1} C hard limit", s.name, gov.name(), s.t_limit));
            }
        }
        None
    }

    pub fn finite(&self) -> bool {
        self.unified.iter().all(|u| u.ff.finite() && u.vi.as_ref().map_or(true, Thermal::finite) && u.reduction.is_finite())
            && self.asahi.as_ref().map_or(true, |a| a.iter().all(|s| s.th.finite() && s.gain.is_finite()))
    }

    /// One JSON line for the shadow log.
    pub fn shadow_line(&mut self, boottime_s: f64, governing: Governor, armed: bool, applied: &[f64], legacy: &[(f64, f64)], legacy_reduction: f64) -> String {
        let mut o = String::new();
        let st = std::mem::take(&mut self.stats);
        let _ = write!(o, "{{\"t\":{boottime_s:.3},\"gov\":\"{}\",\"armed\":{armed},\"play_s\":{:.3}", governing.name(), st.playing_s);
        let mean = |sum: f64| if st.playing_s > 0.0 { sum / st.playing_s } else { 0.0 };
        let _ = write!(o, ",\"mean_red\":{{\"legacy\":{:.3},\"asahi\":{:.3},\"unified\":{:.3}}}", mean(st.sum_legacy), mean(st.sum_asahi), mean(st.sum_unified));
        let _ = write!(o, ",\"max_coil\":{{\"ff\":{:.2},\"vi\":{:.2},\"asahi\":{:.2}}}", st.max_coil_ff, st.max_coil_vi.max(0.0), st.max_coil_asahi.max(0.0));
        let _ = write!(o, ",\"legacy_red\":{legacy_reduction:.3},\"legacy_011_budget\":{:.3}", self.legacy_011_budget());
        let ru = self.reductions(Governor::Unified);
        let ra = self.reductions(Governor::Asahi);
        o.push_str(",\"spk\":[");
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            let f = self.last.get(k).copied().unwrap_or_default();
            let u = &self.unified[k];
            if k > 0 {
                o.push(',');
            }
            let _ = write!(o, "{{\"n\":\"{}\",\"applied\":{:.2},\"p_ff\":{:.5},\"leg\":[{:.2},{:.2}],\"uni\":{:.2},\"ff\":[{:.2},{:.2}]", s.name, applied.get(k).copied().unwrap_or(0.0), f.v_ms / s.r_ff, legacy.get(k).map_or(0.0, |l| l.0), legacy.get(k).map_or(0.0, |l| l.1), ru[k], u.ff.t_coil, u.ff.t_magnet);
            if let Some(vi) = &u.vi {
                let _ = write!(o, ",\"p_vi\":{:.5},\"vi\":[{:.2},{:.2}]", f.vi.unwrap_or(0.0), vi.t_coil, vi.t_magnet);
            }
            if let Some(a) = &self.asahi {
                let _ = write!(o, ",\"asa\":{:.2},\"asa_t\":[{:.2},{:.2}]", ra[k], a[k].th.t_coil, a[k].th.t_magnet);
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

    pub fn encode(&self, boottime: std::time::Duration) -> String {
        let mut o = format!("speakerguardd-engine 1\nboottime_ns {}\n", boottime.as_nanos());
        for (k, s) in self.cfg.speakers.iter().enumerate() {
            let u = &self.unified[k];
            let vi = u.vi.as_ref().map_or("- -".to_string(), |v| format!("{:.6} {:.6}", v.t_coil, v.t_magnet));
            let asa = self.asahi.as_ref().map_or("- -".to_string(), |a| format!("{:.6} {:.6}", a[k].th.t_coil, a[k].th.t_magnet));
            let _ = writeln!(o, "speaker {} {:.6} {:.6} {vi} {asa} {:.6}", state_name(&s.name), u.ff.t_coil, u.ff.t_magnet, u.mean_pre(60.0));
        }
        o
    }
}

/// A speaker's name as one word ("Left Woofer 1" -> "Left_Woofer_1").
pub fn state_name(name: &str) -> String {
    name.replace(' ', "_")
}

#[derive(Debug, Clone, PartialEq)]
pub struct SavedEngineSpeaker {
    pub name: String,
    pub ff: (f64, f64),
    pub vi: Option<(f64, f64)>,
    pub asahi: Option<(f64, f64)>,
    pub mean_60s_pre: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SavedEngine {
    pub boottime: std::time::Duration,
    pub speakers: Vec<SavedEngineSpeaker>,
}

pub fn decode(text: &str) -> Option<SavedEngine> {
    let mut lines = text.lines();
    if lines.next()? != "speakerguardd-engine 1" {
        return None;
    }
    let ns: u64 = lines.next()?.strip_prefix("boottime_ns ")?.parse().ok()?;
    let pair = |a: &str, b: &str| -> Option<Option<(f64, f64)>> {
        if a == "-" && b == "-" {
            return Some(None);
        }
        let (x, y): (f64, f64) = (a.parse().ok()?, b.parse().ok()?);
        (x.is_finite() && y.is_finite() && x > -50.0 && y > -50.0 && x < 400.0 && y < 400.0).then_some(Some((x, y)))
    };
    let mut speakers = Vec::new();
    for line in lines {
        let w: Vec<&str> = line.split_whitespace().collect();
        if w.len() != 9 || w[0] != "speaker" {
            return None;
        }
        let ff = pair(w[2], w[3])??;
        let mean: f64 = w[8].parse().ok()?;
        if !mean.is_finite() || mean < 0.0 {
            return None;
        }
        speakers.push(SavedEngineSpeaker { name: w[1].to_string(), ff, vi: pair(w[4], w[5])?, asahi: pair(w[6], w[7])?, mean_60s_pre: mean });
    }
    if speakers.is_empty() {
        return None;
    }
    Some(SavedEngine { boottime: std::time::Duration::from_nanos(ns), speakers })
}

#[cfg(test)]
mod tests {
    use super::*;

    const J700: &str = include_str!("../conf/apple/j700.conf");
    const J316: &str = include_str!("../conf/apple/j316.conf");
    const J413: &str = include_str!("../conf/apple/j413.conf");
    const J616: &str = include_str!("../conf/apple/j616.conf");

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
                sim.step(p, h / n as f64, 50.0);
            }
            assert!((a + b * p - sim.t_coil).abs() < 0.05, "p {p} h {h}: {} vs {}", a + b * p, sim.t_coil);
        }
        let eq = Thermal::new(10.0, 5.0, 10.0, 5.0, 50.0);
        let (a, b) = eq.coil_after(5.0, 50.0);
        let mut sim = eq.clone();
        for _ in 0..5000 {
            sim.step(1.0, 0.001, 50.0);
        }
        assert!((a + b - sim.t_coil).abs() < 0.05);
    }

    #[test]
    fn exact_cooling_matches_fine_steps() {
        let mut a = Thermal::new(45.0, 1.3, 50.0, 73.26, 140.0);
        a.t_magnet = 110.0;
        let mut b = a.clone();
        a.cool_exact(30.0, 50.0);
        for _ in 0..300000 {
            b.step(0.0, 0.0001, 50.0);
        }
        assert!((a.t_coil - b.t_coil).abs() < 0.01 && (a.t_magnet - b.t_magnet).abs() < 0.01, "{a:?} {b:?}");
    }

    #[test]
    fn the_most_power_reaches_the_target_at_the_horizon() {
        let th = Thermal::new(38.0, 3.3, 26.0, 100.0, 70.0);
        let p = th.max_power(115.0, 3.3, 50.0);
        assert!(p > 0.0);
        let mut sim = th.clone();
        for _ in 0..3300 {
            sim.step(p, 0.001, 50.0);
        }
        assert!((sim.t_coil - 115.0).abs() < 0.05, "{}", sim.t_coil);
        // over the target: the power that brings it back to the target in h
        let mut hot = Thermal::new(38.0, 3.3, 26.0, 100.0, 116.0);
        hot.t_magnet = 70.0;
        let p = hot.max_power(115.0, 3.3, 50.0);
        assert!(p > 0.0);
        let mut sim = hot.clone();
        for _ in 0..3300 {
            sim.step(p, 0.001, 50.0);
        }
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
            let e = Engine::new(&c, Start::Cold);
            let a = e.asahi.as_ref().unwrap();
            for (name, w) in want {
                let g = a[idx(&c, name)].min_gain;
                assert!((g - w).abs() < 0.005, "{name}: {g} vs {w}");
            }
        }
    }

    #[test]
    fn asahi_ramp_and_hysteresis_follow_speakersafetyd() {
        let c = cfg(J316);
        let mut e = Engine::new(&c, Start::Cold);
        // cold boot: 1 C short of the window (140 - 20 - 1)
        assert!((e.asahi.as_ref().unwrap()[0].th.t_coil - 119.0).abs() < 1e-9);
        e.govern_all(0.1, true);
        assert!(e.reductions(Governor::Asahi).iter().all(|&r| r == 0.0));
        // halfway through the window: half of min_gain
        {
            let a = e.asahi.as_mut().unwrap();
            for s in a.iter_mut() {
                s.th.t_coil = 130.0;
                s.th.t_magnet = 80.0;
            }
        }
        e.govern_all(0.1, true);
        let a = e.asahi.as_ref().unwrap();
        assert!((a[0].gain - a[0].min_gain * 0.5).abs() < 1e-9, "{}", a[0].gain);
        // peak hold: back to 127 C, the hysteresis keeps 130 (127 + 5 > 130)
        {
            let a = e.asahi.as_mut().unwrap();
            for s in a.iter_mut() {
                s.th.t_coil = 127.0;
            }
        }
        e.govern_all(0.1, true);
        let a = e.asahi.as_ref().unwrap();
        assert!((a[0].gain - a[0].min_gain * 0.5).abs() < 1e-9);
        // linked groups: the woofers get the woofer group's largest
        let r = e.reductions(Governor::Asahi);
        let w1 = idx(&c, "Left Woofer 1");
        assert_eq!(r[w1], r[idx(&c, "Right Woofer 2")]);
        assert_eq!(r[idx(&c, "Left Tweeter")], r[idx(&c, "Right Tweeter")]);
    }

    /// Run `gov` as the governor that writes the card, `pre` watts of
    /// feed-forward power (and `vi_ratio` of it as measured V*I) before the
    /// gain, for `secs`; returns the hottest coil each model reached after
    /// the first second (a daemon can only act one sense period late, so the
    /// first period of a burst always plays at the gain set before it).
    fn govern(c: &Config, gov: Governor, pre_v_ms: f64, vi_ratio: Option<f64>, secs: f64) -> (Engine, f64, f64) {
        let mut e = Engine::new(c, Start::Cold);
        let dt = 4096.0 / 48000.0;
        let (mut hot_ff, mut hot_vi): (f64, f64) = (0.0, 0.0);
        let mut t = 0.0;
        while t < secs {
            let r = e.reductions(gov);
            let post: Vec<Feature> = c
                .speakers
                .iter()
                .zip(&r)
                .map(|(s, &r)| {
                    let k = 10f64.powf(-r / 10.0);
                    Feature { v_ms: pre_v_ms * k, vi: vi_ratio.map(|x| x * pre_v_ms / s.r_ff * k), i_ms: None, r_lf: None }
                })
                .collect();
            e.period(&post, &r, dt, 0.0, None);
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
        assert!(e.hard_fault(Governor::Unified).is_none());
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

    #[test]
    fn asahi_governing_keeps_its_model_under_the_hard_limit() {
        for text in [J316, J413] {
            let c = cfg(text);
            let v_ms = 10f64.powf(18.5 / 10.0);
            let (e, _, _) = govern(&c, Governor::Asahi, v_ms, Some(1.0), 600.0);
            assert!(e.hard_fault(Governor::Asahi).is_none());
            let (ac, _, _) = e.temps(0);
            assert!(ac < 150.0, "{ac}");
        }
    }

    #[test]
    fn engine_state_round_trips_with_spaces_in_names() {
        let c = cfg(J316);
        let (e, _, _) = govern(&c, Governor::Unified, 5.0, Some(1.0), 30.0);
        let text = e.encode(std::time::Duration::from_secs(7));
        let saved = decode(&text).unwrap();
        assert_eq!(saved.speakers.len(), 6);
        assert!(saved.speakers.iter().any(|s| s.name == "Left_Woofer_1"));
        let mut f = Engine::new(&c, Start::Cold);
        f.restore(&saved);
        assert!((f.temps(0).1 - e.temps(0).1).abs() < 1e-4);
        assert!((f.temps(0).2 - e.temps(0).2).abs() < 1e-4);
        assert!(decode(&text.replace("speakerguardd-engine 1", "x")).is_none());
        assert!(decode(&text.replacen(&format!("{:.6}", e.temps(0).1), "NaN", 1)).is_none());
    }

    #[test]
    fn a_muted_card_does_not_amplify_noise() {
        let c = cfg(J316);
        let mut e = Engine::new(&c, Start::Cold);
        let post = vec![Feature { v_ms: 5e-5, vi: Some(1e-5), i_ms: Some(1e-5), r_lf: None }; 6];
        for _ in 0..200 {
            e.period(&post, &[99.5; 6], 0.085, 0.0, None);
        }
        assert!(e.temps(0).1 < 51.0, "{}", e.temps(0).1);
    }

    #[test]
    fn s16_sense_words_widen_to_s32_full_scale() {
        let mut out = [0i32; 3];
        crate::alsa::widen(&[i16::MIN, -1, i16::MAX], &mut out);
        assert_eq!(out, [i32::MIN, -65536, 0x7fff0000]);
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

    #[test]
    fn configs_parse_and_governors_check() {
        for text in [J316, J413, J616, J700] {
            cfg(text);
        }
        // the J616's sense order follows its slots: LW1 RW1 LW2 RW2 LT RT
        let j616 = cfg(J616);
        let ch = |n: &str| { let s = &j616.speakers[idx(&j616, n)]; (s.is_chan.unwrap(), s.vs_chan) };
        assert_eq!(ch("Left Woofer 1"), (0, 1));
        assert_eq!(ch("Right Woofer 2"), (6, 7));
        assert_eq!(ch("Left Tweeter"), (8, 9));
        assert_eq!(ch("Right Tweeter"), (10, 11));
        // the S2 family values are the J316's
        let j316 = cfg(J316);
        for n in ["Left Woofer 1", "Left Tweeter"] {
            let (a, b) = (&j616.speakers[idx(&j616, n)], &j316.speakers[idx(&j316, n)]);
            assert_eq!((a.tr_coil, a.tau_coil, a.r_ff, a.vi_tr_coil, a.z_nominal), (b.tr_coil, b.tau_coil, b.r_ff, b.vi_tr_coil, b.z_nominal));
        }
        assert!(Config::parse(&J316.replace("[Globals]", "[Globals]\ngovernor = asahi")).is_ok());
        assert!(Config::parse(&J700.replace("[Globals]", "[Globals]\ngovernor = asahi")).is_err());
        assert!(Config::parse(&J700.replace("[Globals]", "[Globals]\ngovernor = unified")).is_ok());
        assert!(Config::parse(&J700.replace("[Globals]", "[Globals]\ngovernor = loud")).is_err());
        let c = cfg(J316);
        assert_eq!(c.speakers.iter().find(|s| s.name == "Left Tweeter").unwrap().is_chan, Some(4));
        assert!((c.speakers[idx(&c, "Left Woofer 1")].r_ff - 2.7).abs() < 1e-9);
        assert!((c.speakers[idx(&c, "Left Tweeter")].r_ff - 3.2).abs() < 1e-9);
        let j700 = cfg(J700);
        assert_eq!(j700.speakers[0].r_ff, j700.speakers[0].z_nominal);
    }
}

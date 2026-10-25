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
//!   Its prediction of the program is the power before the gain of the
//!   frames last measured at a gain the card is known to have applied
//!   (review r4 of speakerguardd#8): a charge stands for time nobody
//!   measured and heats every model at the worst case, but is not the
//!   program, and neither are frames the card held at its safe level.
//!   It writes the card under the asahi guard, or alone with
//!   --experimental-unified.
//! * The asahi guard (`guard = asahi`, the unified governor only): the
//!   card gets each speaker's larger reduction of the unified and the
//!   asahi governor, group-linked, and either one's hard latch hands it to
//!   the kernel.  The unified regulator then holds the feed-forward model
//!   only, and the measured-power side is the guard's.  (Unified's own
//!   measured-power model has the same constants under a look-ahead: on
//!   loud tweeters it asked for more than speakersafetyd's ramp and handed
//!   the card over more often, for no protection the guard lacks.  It
//!   still runs, latches and is logged.)  A dead current line is bounded,
//!   below.  Sense data that is not a measurement at all stays a fault, as
//!   without the guard: the feed-forward model's voltage comes from the
//!   same words.
//! * A current line that reads far too little for its voltage (a dead or
//!   faulty line, see DEAD_RATIO) is lost for the stretch (review r4 BB1
//!   of speakerguardd#8).  The measured-power models then take V^2 over
//!   the speaker's smallest impedance, the most current there can be, as
//!   at the sense's rail: the asahi governor, and the guard, keep Asahi's
//!   constants with a bound at least as hot as a live line, instead of
//!   seeing nothing.  What a line lost too briefly for a block to show
//!   may have under-read at a stretch's ends is charged late (Engine::
//!   measured).
//! * Resistance thermometry from low-band V/I (`R = sum(v i) / sum(i^2)`)
//!   is only measured and logged.

use std::collections::VecDeque;
use std::fmt::Write as _;

use crate::config::{Config, Globals, Governor, Speaker};
use crate::model::{budget_envelope, converted_at, DRAIN_S, MAX_SUB_DT, SUB_BLOCKS};

/// The asahi models see this much more power than measured (+0.02 dB): on
/// recorded sense data their gains then never ask for less than
/// speakersafetyd's (the f32-versus-f64 rounding was worth up to 0.04 C).
pub const ASAHI_MARGIN: f64 = 1.005;
/// A mean V*I below this (W) is bad sense data, not a measurement
/// (speakersafetyd's sanity bound).
pub const VI_MIN: f64 = -0.01;
/// A sample's V*I between this (W) and zero is sense noise, not power the
/// speaker gave back (review X1 of speakerguardd#8).  An M1 Pro recording
/// (14 V and 3.75 A S16 sense) has 7 to 9 LSB of noise on V and I: of 14.4
/// million samples in its quiet periods 45% have negative V*I, the most
/// negative -44.6 uW and none below -78 uW.  Its playback periods go down
/// to -50 mW on the tweeters and -0.93 W on the woofers.  1 mW is 22 times
/// the noise and 50 times under the tweeters' playback.
pub const VI_NOISE_FLOOR: f64 = 1e-3;
/// A current word at or past this (S16 words widened to S32) is at the
/// sense's rail.
const I_RAIL: f64 = (32767i64 << 16) as f64;

/// The power one sample of V and I words stands for (W).  A current word
/// at its rail says only that the current was at least that much (review
/// Z2 of speakerguardd#8: at full scale the J316 and J616 woofers' 2.7 ohm
/// draw 4.4 A, past the 3.75 A the sense reads, and V*I read up to 15%
/// low).  No current can pass `i_max`, the amplifier's peak voltage over
/// the smallest impedance the speaker can present, so such a sample stands
/// for |V| times that.  V^2 over the impedance would not do: a reactive
/// load's current peaks while the voltage is low.
pub fn sense_power(wv: f64, wi: f64, (vs, is): (f64, f64), i_max: f64) -> f64 {
    let (v, p) = (wv * vs, wv * vs * (wi * is));
    if wi.abs() >= I_RAIL { p.max(v.abs() * i_max) } else { p }
}
/// What the unified regulator asks for when nothing may pass (dB): far past
/// any --max-reduction, so the kernel takes over.
const NOTHING_DB: f64 = 120.0;
/// A current line reads far too little for its voltage (review r4 BB1 of
/// speakerguardd#8) when, over a 10 ms block, |V|/|I| (rms) is over
/// DEAD_RATIO times the speaker's nominal impedance, or over a 1 ms block
/// DEAD_RATIO_SHORT times it in two short blocks in a row.  A live line
/// stays under both: through a load
/// of up to 60 degrees, a sine of 20 Hz or more reads at most 2.4 times
/// its impedance over 10 ms and 23.7 times over 1 ms, and a laptop
/// driver's impedance peaks at a few times its nominal (the J616s's left
/// woofer 1, whose current sense reads half, at twice).  The short block
/// catches a loud program's lost line within a millisecond; the long one
/// a quiet program's, whose sense noise the short one cannot see past.
pub const DEAD_RATIO: f64 = 10.0;
pub const DEAD_RATIO_SHORT: f64 = 100.0;
pub const DEAD_LONG_S: f64 = 0.010;
pub const DEAD_SHORT_S: f64 = 0.001;
/// Neither block is shorter than this (frames).
pub const DEAD_BLOCK_MIN: usize = 8;
/// A block is judged only at this much voltage or more (V rms): under it
/// the most current there can be heats next to nothing (3.7 mW into 2.7
/// ohm), and the current is too close to the sense noise to judge.
pub const DEAD_V_MIN: f64 = 0.1;
/// A line that reads low but plausibly (review r4 DD1 of speakerguardd#8):
/// its low-band resistance, sum(v i) / sum(i^2) over the last LOW_WINDOW_S
/// of program with at least RTH_I_MIN rms of low-band current, puts the
/// coil above anything a live line can.  A live coil's is its DC
/// resistance, which rises with its temperature (a_t_20c, copper's 0.39 %/K
/// where the file has none): LOW_R_RATIO times z_min at 20 C, scaled to the
/// measured-power model's coil (no cooler than the truth while the line
/// reads, and cooler still when it reads low).  On the J616s's woofers at
/// full volume it is 1.28 to 1.49 times z_min against a bound of 1.6 to
/// 1.8, and one misfired at 1.002 of it at +10 dB of replay gain (review
/// r4 EE2); a line reading half doubles it, and one reading 0.55 to 0.65
/// still clears it.  At LOW_FAST times the bound or more the line reads
/// low at once; in the marginal band under that, once R has stayed over
/// the bound for LOW_SET_S of program, its power bounded meanwhile (review
/// r4 addendum 6a: a 1 s hold on every verdict left a bad line unbounded
/// for its first second).  The wait and the verdict are judged against
/// the least bound since R went over it.  It reads again under
/// LOW_R_CLEAR of that bound, and a marginal verdict (R
/// never past LOW_MARGINAL times the bound) also after LOW_FORGET_S of
/// program with no low-band current to judge it by, counted only while
/// the card is not at its safe level; a line far over the bound stays
/// low through any passage without bass.
pub const LOW_R_RATIO: f64 = 1.35;
pub const LOW_R_CLEAR: f64 = 0.9;
pub const LOW_WINDOW_S: f64 = 3.0;
pub const LOW_FAST: f64 = 1.25;
pub const LOW_SET_S: f64 = 1.0;
pub const LOW_MARGINAL: f64 = 1.15;
pub const LOW_FORGET_S: f64 = 30.0;
const COPPER_ALPHA: f64 = 0.0039;
/// ... and a line whose current does not follow its voltage at all (a
/// dead line carrying a DC offset: the rms ratio stays plausible while
/// V*I averages to nothing): over a stretch with at least PF_V_MIN and a
/// current the dead-line test passes (at least the voltage over DEAD_RATIO
/// times the nominal impedance, so that a dead line with a small offset is
/// judged here if it is not there: review r4 FF1), sum(v i) / sqrt(sum(v^2)
/// sum(i^2)) under PF_MIN.  A live
/// load of up to 60 degrees keeps it at 0.4 or more on any program whose
/// impedance spans up to 4 to 1.  At PF_FAST_V or more the line is lost
/// at once; under it, once the condition has held for PF_HOLD_S of judged
/// program (review r4 EE1).  Every misfire seen on real data (the M2's
/// right woofer at 0.1 V and 25 mA in a quiet passage, the M1 Pro's
/// tweeters at 0.12 to 0.38 V) lasted at most four periods, while an offset
/// line stays offset; what it can hide meanwhile is at most PF_FAST_V^2
/// over the smallest impedance for PF_HOLD_S: 92.6 mW on the J316's and
/// J616's woofers, 78.1 mW on their tweeters, 62.5 mW on the J413's
/// woofers and 67.2 mW on its tweeters (5.0 K after 2 s there), once.
pub const PF_MIN: f64 = 0.2;
pub const PF_V_MIN: f64 = 0.1;
pub const PF_FAST_V: f64 = 0.5;
pub const PF_HOLD_S: f64 = 2.0;
/// A lost line is logged once it has held for LOG_LOST stretches, and as
/// reading again once that has held for LOG_BACK (review r4 DD2); the
/// bound itself follows the line at once.
const LOG_LOST: u32 = 2;
/// The twin comparison's window (s).  It counts only periods where both
/// twins carry at least TWIN_V2_MIN mean square volts and the line reads,
/// at least TWIN_MIN_S of them, and only when the twins' voltages are
/// alike, a correlation of TWIN_CORR_MIN or more, so a source panned to
/// one side cannot trip it.  A speaker reading low by it reads again over
/// TWIN_CLEAR times twin_ratio.
const TWIN_WINDOW_S: f64 = 10.0;
const TWIN_V2_MIN: f64 = 0.01;
const TWIN_MIN_S: f64 = 5.0;
const TWIN_CORR_MIN: f64 = 0.6;
const TWIN_CLEAR: f64 = 1.1;
const LOG_BACK: u32 = 5;
/// Measured stretches further apart than this (s) do not follow each
/// other: what a line lost at the end of the first may have under-read is
/// charged, since the next cannot show it.
pub const DEAD_GAP_S: f64 = 0.010;

/// Each speaker's left/right twin: the speaker whose name has Left and
/// Right swapped.
fn twins(cfg: &Config) -> Vec<Option<usize>> {
    let names: Vec<&str> = cfg.speakers.iter().map(|s| s.name.as_str()).collect();
    names
        .iter()
        .map(|n| {
            let other = if n.contains("Left") { n.replace("Left", "Right") } else if n.contains("Right") { n.replace("Right", "Left") } else { return None };
            names.iter().position(|m| *m == other)
        })
        .collect()
}

/// Block `len` frames long at `step` seconds a frame.
pub fn dead_block(len_s: f64, step: f64) -> usize {
    ((len_s / step).round() as usize).max(DEAD_BLOCK_MIN)
}

/// Whether the current line `i` (A) lost its reading against the voltage
/// `v` (V) anywhere in a stretch, and whether a 10 ms block of it carried
/// a voltage to judge (and so, not lost, showed the line reading).
pub fn line_lost(v: &[f64], i: &[f64], z_nominal: f64, step: f64) -> (bool, bool) {
    let n = v.len().min(i.len());
    let mut lost = false;
    let mut judged = false;
    for (len, ratio, long) in [(DEAD_LONG_S, DEAD_RATIO, true), (DEAD_SHORT_S, DEAD_RATIO_SHORT, false)] {
        let block = dead_block(len, step);
        let limit = (ratio * z_nominal).powi(2);
        let mut f0 = 0;
        // a short block counts only after another: a dead line stays dead,
        // a reactive load's current zero does not (review r4 DD2)
        let mut run = 0;
        while f0 + block <= n {
            let (mut v_ms, mut i_ms) = (0.0, 0.0);
            for f in f0..f0 + block {
                v_ms += v[f] * v[f];
                i_ms += i[f] * i[f];
            }
            let low = v_ms >= DEAD_V_MIN * DEAD_V_MIN * block as f64 && !(i_ms * limit >= v_ms);
            if v_ms >= DEAD_V_MIN * DEAD_V_MIN * block as f64 {
                judged |= long;
            }
            run = if low { run + 1 } else { 0 };
            if run >= if long { 1 } else { 2 } {
                lost = true;
            }
            f0 += block;
        }
    }
    (lost, judged)
}

/// Frames the card may have converted at more reduction than this (dB)
/// say too little about the program for the unified regulator to predict
/// it from them (the card's safe level, an unknown control): the
/// prediction stays where the last known frames left it.  The replay's
/// PRE_GAIN_MAX_DB draws the same line.
pub const PREDICT_MAX_DB: f64 = 30.0;
/// Nor do frames that fit one candidate range wider than this (dB): an
/// unknown control value (0 dB to mute, at a start that did not follow a
/// clean stop), under which the most that fits makes any word full scale.
/// (A failed write is its old and new values, not the span between them.)  A frame that fits several exact values keeps the most
/// that fits (review r4 CC1 of speakerguardd#8): the V/I amplifiers apply
/// a new gain as they play, so that is the newer value after an attack and
/// the right one after a retake, and leaving such frames out blinded the
/// regulator for half a second after every gain change over this.
pub const PREDICT_SPREAD_DB: f64 = 1.0;
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

    pub fn finite(&self) -> bool {
        self.t_coil.is_finite() && self.t_magnet.is_finite()
    }

    /// A start that knows nothing of the speaker's past (review X3 of
    /// speakerguardd#8).  The run before kept this model's coil and magnet
    /// under `limit` (its latch hands the card over at either), so the
    /// truth was under it too; after that it may have left its gain on the
    /// samples still to play, up to the drain at `full` watts; after that
    /// the card held its safe level, at most `safe` watts.  The two-node
    /// model is monotone in its state and its power, so the bounds of the
    /// drain held from (limit, limit), no lower than the steady state of
    /// the safe level, hold every such past from then on, however long ago
    /// the drain ended.  (The magnet used to start where steady state would
    /// put it for the coil's bound, 736 C on the J413 tweeter where the
    /// drain can take it to 203 C, and cooled for two minutes.)
    pub fn latched_start(&mut self, limit: f64, full: f64, safe: f64, t_ambient: f64) -> (f64, f64) {
        self.t_coil = limit;
        self.t_magnet = limit;
        let (coil, magnet) = self.hold(full, DRAIN_S, t_ambient);
        let magnet = hi(magnet, t_ambient + safe * self.tr_magnet);
        let coil = hi(coil, magnet + safe * self.tr_coil);
        self.t_coil = coil;
        self.t_magnet = magnet;
        (coil, magnet)
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
    /// coil is a constant plus two exponentials, so it has at most one
    /// extremum inside the interval: its bound is the larger end or that
    /// extremum.  (A coil below a falling drive rises until it meets the
    /// drive; the bound was where the drive started, however short the
    /// interval, and a sliver of uncovered time at 20 dB then latched a
    /// model 10 K under its limit: review r2 of speakerguardd#8, found by
    /// the V/I harness.)
    pub fn hold(&mut self, power: f64, h: f64, t_ambient: f64) -> (f64, f64) {
        let (c0, m0) = (self.t_coil, self.t_magnet);
        if !(h > 0.0) {
            return (c0, m0);
        }
        let (alpha, beta) = self.coil_after(h, t_ambient);
        let m_inf = t_ambient + power * self.tr_magnet;
        let inside = self.coil_extremum(power, h, t_ambient);
        self.t_magnet = m_inf + (m0 - m_inf) * (-h / self.tau_magnet).exp();
        self.t_coil = alpha + beta * power;
        (hi(hi(c0, self.t_coil), inside), hi(m0, self.t_magnet))
    }

    /// `e` joules at once: heat a stretch under-read, charged late (a lost
    /// current line, review r4 BB1 of speakerguardd#8).  The coil rises by
    /// e * Rc / tau_c and the magnet by e * Rm / tau_m, as at an instant of
    /// the power that carries it, the coil by at least the magnet's rise:
    /// the response then only falls, so heat charged late is never cooler
    /// than the same heat earlier.
    pub fn impulse(&mut self, e: f64) -> (f64, f64) {
        if e != 0.0 {
            let m = e * self.tr_magnet / self.tau_magnet;
            self.t_coil += hi(e * self.tr_coil / self.tau_coil, m);
            self.t_magnet += m;
        }
        (self.t_coil, self.t_magnet)
    }

    /// The coil at its extremum strictly inside (0, h) at a constant
    /// `power` from the current state, or -inf when it has none there.
    /// The coil is c_inf + K e^(-t/tau_m) + C e^(-t/tau_c), whose
    /// derivative has one zero at most.  Equal time constants keep the
    /// old bound (where the drive starts, for a coil below a falling
    /// drive).
    fn coil_extremum(&self, power: f64, h: f64, t_ambient: f64) -> f64 {
        let (tc, tm) = (self.tau_coil, self.tau_magnet);
        let m_inf = t_ambient + power * self.tr_magnet;
        let c_inf = m_inf + power * self.tr_coil;
        if (tm - tc).abs() <= 1e-6 * tc {
            let drive0 = self.t_magnet + power * self.tr_coil;
            return if !(self.t_coil >= drive0) && !(self.t_magnet <= m_inf) { drive0 } else { f64::NEG_INFINITY };
        }
        let k = tm / (tm - tc) * (self.t_magnet - m_inf);
        let c = self.t_coil - c_inf - k;
        // c'(t) = -(K / tau_m) e^(-t/tau_m) - (C / tau_c) e^(-t/tau_c) = 0
        let ratio = -(k * tc) / (c * tm);
        if !(ratio > 0.0) {
            return f64::NEG_INFINITY;
        }
        let t = ratio.ln() / (1.0 / tm - 1.0 / tc);
        if !(t > 0.0 && t < h) {
            return f64::NEG_INFINITY;
        }
        let v = c_inf + k * (-t / tm).exp() + c * (-t / tc).exp();
        // rounding never brings the bound under the coil
        v + 1e-9 * v.abs().max(1.0)
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
    /// The low band's sums of v*i and i^2 and the frames they cover.
    pub lf: (f64, f64, usize),
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
                f.lf = (lvi, li2, frames);
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

/// The power before the card's gain, from the power after it (the callers
/// cap it at full scale).
fn pre_gain(power: f64, applied_db: f64) -> f64 {
    power * 10f64.powf(applied_db.max(0.0) / 10.0)
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
            z_min: s.z_min(),
            v_worst: v_worst(s, g),
        }
    }

    fn assume_latched(&mut self, g: &Globals) {
        // a run that died under the limit may have left its gain on the
        // queued samples: they play out for up to the drain at full scale
        let p = self.v_worst * self.v_worst / self.z_min;
        self.th.latched_start(self.t_limit, p, p * 10f64.powf(-g.safe_reduction.max(0.0) / 10.0), g.t_ambient);
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
    /// crossing also leaves the model at the limit at least when negative
    /// V*I beyond the noise floor follows it: that cools the model, never
    /// the coil, so the model cannot say the coil came back down.  Negative
    /// noise does not (review X1 of speakerguardd#8): it would hold a model
    /// at the limit for as long as the sense lines carry noise.
    /// The first frame is `first` of a step long (the part of it over
    /// time not yet accounted).
    #[allow(clippy::too_many_arguments)]
    fn samples(&mut self, words: &[i32], channels: usize, (vc, ic): (usize, usize), (vs, is): (f64, f64), step: f64, first: f64, t_ambient: f64, lost: bool) {
        let coeffs = |h: f64| (h / (self.th.tau_coil + h), h / (self.th.tau_magnet + h));
        let (ac1, am1) = coeffs(step * first);
        let (acn, amn) = coeffs(step);
        let (trc, trm) = (self.th.tr_coil, self.th.tr_magnet);
        let (mut tc, mut tm) = (self.th.t_coil, self.th.t_magnet);
        let (mut pc, mut pm) = (tc, tm);
        // At or over the limit the model latches; negative V*I after that
        // point cools the model, never the coil, so the model may not say
        // the coil came back under the limit in this stretch.  Cooling with
        // no negative power after the crossing is real (a model that starts
        // over the limit, latched, may cool from it), and so is cooling
        // through sense noise (VI_NOISE_FLOOR).
        let mut crossed = false;
        let mut negative_after = false;
        for (i, f) in words.chunks_exact(channels).enumerate() {
            let (ac, am) = if i == 0 { (ac1, am1) } else { (acn, amn) };
            let p = if lost {
                // the current line is lost: the most current there can be
                (f[vc] as f64 * vs).powi(2) / self.z_min
            } else {
                sense_power(f[vc] as f64, f[ic] as f64, (vs, is), self.v_worst / self.z_min)
            } * ASAHI_MARGIN;
            if crossed && p < -VI_NOISE_FLOOR {
                negative_after = true;
            }
            let coil_target = tm + p * trc;
            let magnet_target = t_ambient + p * trm;
            tc = coil_target * ac + tc * (1.0 - ac);
            tm = magnet_target * am + tm * (1.0 - am);
            if !(tc < self.t_limit && tm < self.t_limit) {
                crossed = true;
            }
            pc = hi(pc, tc);
            pm = hi(pm, tm);
        }
        if negative_after {
            if !(pc < self.t_limit) {
                tc = hi(tc, self.t_limit);
            }
            if !(pm < self.t_limit) {
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

    /// `e` joules a lost current line may have under-read, charged late
    /// (see Engine::measured).
    fn charge(&mut self, e: f64) {
        let (c, m) = self.th.impulse(e * ASAHI_MARGIN);
        self.note(hi(self.coil_peak, c), hi(self.magnet_peak, m));
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
    /// The regulator's prediction of the program: the power before the
    /// gain of the last frames measured at a known gain, feed-forward and
    /// measured (W).  Not a charge's worst case.
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
            z_min: s.z_min(),
            v_worst: v_worst(s, g),
        }
    }

    fn assume_latched(&mut self, g: &Globals) {
        // the drain's heat on top of the limit, and a minute of full scale
        // in the budgets (nothing about the past is known)
        let safe = 10f64.powf(-g.safe_reduction.max(0.0) / 10.0);
        let full_ff = self.v_worst * self.v_worst / self.r_ff;
        let (c, m) = self.ff.latched_start(self.t_limit, full_ff, full_ff * safe, g.t_ambient);
        let mut peak = hi(c, m);
        let full_vi = self.v_worst * self.v_worst / self.z_min;
        if let Some(v) = self.vi.as_mut() {
            let (c, m) = v.latched_start(self.t_limit, full_vi, full_vi * safe, g.t_ambient);
            peak = hi(peak, hi(c, m));
        }
        self.peak = peak;
        self.hard = true;
        self.history.clear();
        self.history.push_back((60.0, full_ff));
        self.history_s = 60.0;
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

    /// Measured blocks of equal length: the V^2 means, the V*I means (V/I
    /// machines), and each block's feed-forward power before the card's
    /// gain for the budgets (each frame inverted on its own, at most full
    /// scale: Engine::measured).  The regulator's prediction is `predict`'s.
    fn blocks(&mut self, v_ms: &[f64], vi: Option<&[f64]>, pre_ff: &[f64], span: f64, t_ambient: f64) {
        let n = v_ms.len().max(1) as f64;
        let h = span / n;
        let mut peak = self.peak_floor.take().unwrap_or(f64::NEG_INFINITY).max(self.ff.hottest());
        for &v2 in v_ms {
            let (c, m) = self.ff.varying(v2 / self.r_ff, h, t_ambient);
            peak = hi(peak, hi(c, m));
        }
        if let (Some(model), Some(vi)) = (self.vi.as_mut(), vi) {
            peak = hi(peak, model.hottest());
            // negative blocks only ever dropped: never cooler than measured
            for &p in vi {
                let (c, m) = model.varying(p.max(0.0), h, t_ambient);
                peak = hi(peak, hi(c, m));
            }
        }
        self.note(peak);
        let pre = pre_ff.iter().sum::<f64>() / n;
        record(&mut self.history, &mut self.history_s, span, pre);
    }

    /// The program as the last frames measured at a known gain had it
    /// before the gain (None: no such frame, the prediction stays).
    fn predict(&mut self, ff: Option<f64>, vi: Option<f64>) {
        if let Some(p) = ff {
            self.last_ff_pre = p;
        }
        if let (Some(p), true) = (vi, self.vi.is_some()) {
            self.last_vi_pre = Some(p);
        }
    }

    /// `d` seconds nobody measured, at the most the card's `reduction`
    /// lets through: every model heats at that worst case and the budgets
    /// count it, but the regulator's prediction stays the measured
    /// program's.  The account charges at least a frame before every
    /// period (where a period's frames end is known only to a frame), and
    /// that frame's full scale, taken as the program for the next horizon,
    /// held the regulator 9 to 14 dB down on quiet content on the J616
    /// (review r4 of speakerguardd#8).
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
            // tighten the feed-forward one; under the asahi guard the
            // measured-power side is the guard's (its own model still runs,
            // latches and is logged): the regulator holds the feed-forward
            // model, as macOS's does
            if let (Some(vi), Some(p), None) = (self.vi.as_ref(), self.last_vi_pre, g.guard) {
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

    /// `e` joules a lost current line may have under-read, charged late
    /// on the measured-power model (see Engine::measured).
    fn charge_vi(&mut self, e: f64) {
        let Some(vi) = self.vi.as_mut() else { return };
        let (c, m) = vi.impulse(e);
        self.note(hi(self.peak, hi(c, m)));
    }

    #[cfg(test)]
    pub fn idle_for_test(&mut self, d: f64, t_ambient: f64) {
        self.idle(d, t_ambient)
    }

    #[cfg(test)]
    pub fn blocks_for_test(&mut self, blocks: &[(f64, Option<f64>)], span: f64, applied: f64, t_ambient: f64) {
        let full_ff = self.v_worst * self.v_worst / self.r_ff;
        let full_vi = self.v_worst * self.v_worst / self.z_min;
        let v: Vec<f64> = blocks.iter().map(|b| b.0).collect();
        let vi: Vec<f64> = blocks.iter().map(|b| b.1.unwrap_or(0.0)).collect();
        let pf: Vec<f64> = v.iter().map(|&x| pre_gain(x / self.r_ff, applied).min(full_ff)).collect();
        let pv: Vec<f64> = vi.iter().map(|&x| pre_gain(x.max(0.0), applied).min(full_vi)).collect();
        let has_vi = blocks.iter().any(|b| b.1.is_some());
        self.blocks(&v, has_vi.then_some(&vi[..]), &pf, span, t_ambient);
        if applied <= PREDICT_MAX_DB {
            let mean = |x: &[f64]| x.iter().sum::<f64>() / x.len().max(1) as f64;
            self.predict(Some(mean(&pf)), has_vi.then(|| mean(&pv)));
        }
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
    pub fn resume(&mut self, x: &SavedUnified) {
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
                    // its coil and magnet were under the limit, no more
                    vi.t_coil = self.t_limit;
                    vi.t_magnet = self.t_limit;
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
    /// Per speaker: whether its current line was lost over the last
    /// stretch, and the transitions since the guard last logged them.
    pub lost: Vec<bool>,
    pub lost_changed: Vec<(usize, bool)>,
    /// Per speaker: the heat (J) a line lost at the end of the last stretch
    /// may have under-read, not yet charged, and whether that stretch
    /// showed the line reading (review r4 BB1).
    pending: Vec<f64>,
    reading: Vec<bool>,
    /// Per speaker: the low band's decaying sums of v*i, i^2 and frames,
    /// whether the line reads low by them (DD1), and the stretches the
    /// lost state has held since the log last changed.
    lf: Vec<(f64, f64, f64)>,
    pub low: Vec<bool>,
    /// Per speaker: its left/right twin, the decaying sums of v*i, v^2,
    /// seconds and v times the twin's v over its periods with a reading
    /// line, and whether it reads low against the twin.
    twin: Vec<Option<usize>>,
    tw: Vec<(f64, f64, f64, f64)>,
    pub low_twin: Vec<bool>,
    /// Per speaker: seconds of judged program the power-factor condition
    /// has held, the low-band bound has been exceeded, and the low-band
    /// test has had nothing to judge.
    pf_held: Vec<f64>,
    low_held: Vec<f64>,
    low_idle: Vec<f64>,
    /// ... and how far over the bound (a share of it) R has been since the
    /// line read low, and the least bound since R went over it (the wait
    /// and the verdict are judged against it).
    low_peak: Vec<f64>,
    low_set_bound: Vec<f64>,
    logged: Vec<(bool, u32)>,
    /// Seconds accounted since the last measured stretch, or none since an
    /// idle span (the next stretch does not follow it).
    gap: f64,
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
        Some(Engine { cfg: cfg.clone(), asahi, unified, extractor: Extractor::new(n), last: vec![Feature::default(); n], lost: vec![false; n], lost_changed: Vec::new(), pending: vec![0.0; n], reading: vec![false; n], gap: f64::INFINITY, lf: vec![(0.0, 0.0, 0.0); n], low: vec![false; n], logged: vec![(false, 0); n], pf_held: vec![0.0; n], low_held: vec![0.0; n], low_idle: vec![0.0; n], low_peak: vec![0.0; n], low_set_bound: vec![f64::INFINITY; n], twin: twins(cfg), tw: vec![(0.0, 0.0, 0.0, 0.0); n], low_twin: vec![false; n] })
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
        let g = self.cfg.globals.clone();
        for s in self.asahi.iter_mut().flatten() {
            s.assume_latched(&g);
        }
        for s in self.unified.iter_mut().flatten() {
            s.assume_latched(&g);
        }
    }

    /// Continue from a clean stop's saved models, then charge `safe`
    /// seconds at the worst case the card's safe level (`safe_db`) lets
    /// through (no daemon held the card since).  A speaker a model has no
    /// saved line for starts latched at its limit, uncharged.
    pub fn resume(&mut self, asahi: &[SavedAsahi], unified: &[SavedUnified], safe: f64, safe_db: f64) {
        let g = self.cfg.globals.clone();
        let ta = g.t_ambient;
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
                None => s.assume_latched(&g),
            }
        }
        for (k, s) in self.unified.iter_mut().flatten().enumerate() {
            match unified.iter().find(|x| x.name == names[k]) {
                Some(x) => {
                    s.resume(x);
                    let mut p = s.peak;
                    if safe > 0.0 {
                        s.worst(safe_db, safe, ta);
                        p = hi(p, s.peak);
                    }
                    s.peak = p;
                }
                None => s.assume_latched(&g),
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

    /// `frames` measured frames in `words`, `step` seconds each, of which
    /// only `certain` certainly played up to the account's new end (their
    /// place is `shift` frames open) and the newest `k` of those are over
    /// time not yet accounted, the oldest of them for `first` (0 < first <=
    /// 1) of its step.  Each model takes the most energetic run of them for
    /// itself (the asahi models the most V*I, the unified ones the most V^2,
    /// V*I and power before the gain).  The card applied one of
    /// `possible[k]` to speaker k: the models get the sensed power, the
    /// reductions only rebuild each frame's power before the gain for the
    /// unified budgets, frame by frame (a block can hold frames converted
    /// at different gains).  The asahi models integrate every sample, the
    /// unified ones blocks, the first frame on its own when it is only
    /// partly new (review P8-3).
    #[allow(clippy::too_many_arguments)]
    pub fn measured(&mut self, words: &[i32], frames: usize, certain: usize, k: usize, shift: usize, step: f64, first: f64, possible: &[Vec<(f64, f64)>]) {
        if frames == 0 || k == 0 || !(step > 0.0) || !(first > 0.0) || certain + shift > frames || k > certain {
            return;
        }
        let g = self.cfg.globals.clone();
        let ch = g.channels;
        let first = first.min(1.0);
        let span = step * ((k - 1) as f64 + first);
        let run = |o: usize| (o + certain - k, o + certain);
        let best = |x: &[f64]| (0..=shift).max_by(|&a, &b| {
            let (sa, sb) = (run(a), run(b));
            x[sa.0..sa.1].iter().sum::<f64>().total_cmp(&x[sb.0..sb.1].iter().sum::<f64>())
        }).unwrap_or(0);
        for (sk, s) in self.cfg.speakers.iter().enumerate() {
            let vs = s.vs_scale / 2147483648.0;
            let is = s.is_scale / 2147483648.0;
            // per frame: V^2, V*I, and the powers before the gain
            let mut v2 = Vec::with_capacity(frames);
            let mut vi = Vec::with_capacity(frames);
            let mut pre_ff = Vec::with_capacity(frames);
            let mut pre_vi = Vec::with_capacity(frames);
            // per frame: whether the gain it was converted at is known
            // well enough to say what the program was
            let mut known = Vec::with_capacity(frames);
            let unified = self.unified.as_ref().map(|u| &u[sk]);
            let i_max = v_worst(s, &g) / s.z_min();
            let z_min = s.z_min();
            // A current line that reads far too little for its voltage in
            // any block of the stretch is lost for all of it (review r4 BB1)
            let (mut lost, judged) = match s.is_chan {
                Some(ic) => {
                    let v: Vec<f64> = (0..frames).map(|f| words[f * ch + s.vs_chan] as f64 * vs).collect();
                    let i: Vec<f64> = (0..frames).map(|f| words[f * ch + ic] as f64 * is).collect();
                    line_lost(&v, &i, s.z_nominal, step)
                }
                None => (false, false),
            };
            // A low-band verdict that waits (in the marginal band) bounds
            // the stretch's power while it waits, but is neither logged nor
            // kept until it has held (review r4 addendum 6a: the wait alone
            // left a bad line unbounded for its first second).  The
            // power-factor wait under PF_FAST_V does not: what it can hide is
            // at most PF_FAST_V^2 / z_min for PF_HOLD_S, once
            let mut waiting = false;
            if s.is_chan.is_some() {
                // a line that reads low but plausibly, or not with its
                // voltage at all (DD1), from the stretch's features
                let f = &self.last[sk];
                let (vi_m, i_ms) = (f.vi.unwrap_or(0.0), f.i_ms.unwrap_or(0.0));
                if f.v_ms >= PF_V_MIN * PF_V_MIN && i_ms * (DEAD_RATIO * s.z_nominal).powi(2) >= f.v_ms {
                    if !(vi_m >= PF_MIN * (f.v_ms * i_ms).sqrt()) {
                        self.pf_held[sk] += span;
                        if f.v_ms >= PF_FAST_V * PF_FAST_V || self.pf_held[sk] >= PF_HOLD_S {
                            lost = true;
                        }
                    } else {
                        self.pf_held[sk] = 0.0;
                    }
                }
                // the low band is judged on program the card played at a
                // gain it may have applied: at its safe level the words are
                // the sense's noise, whose resistance is anything (it
                // cleared a line reading 0.65 at a hand-off: review r4
                // addendum 6a)
                let audible = f.v_ms >= DEAD_V_MIN * DEAD_V_MIN && possible[sk].iter().any(|r| r.0 <= PREDICT_MAX_DB);
                if audible {
                    let decay = (-span / LOW_WINDOW_S).exp();
                    let w = &mut self.lf[sk];
                    *w = (w.0 * decay + f.lf.0, w.1 * decay + f.lf.1, w.2 * decay + f.lf.2 as f64);
                }
                let w = self.lf[sk];
                if !audible {
                    // nothing to judge by, and no time toward forgetting
                } else if w.2 > 0.0 && w.1 / w.2 >= RTH_I_MIN * RTH_I_MIN {
                    self.low_idle[sk] = 0.0;
                    let r = w.0 / w.1;
                    let alpha = if s.a_t_20c > 0.0 { s.a_t_20c } else { COPPER_ALPHA };
                    let t_coil = match (self.asahi.as_ref(), self.unified.as_ref().and_then(|u| u[sk].vi.as_ref())) {
                        (Some(a), _) => a[sk].th.t_coil,
                        (None, Some(v)) => v.t_coil,
                        (None, None) => g.t_ambient,
                    };
                    let bound = LOW_R_RATIO * z_min * (1.0 + alpha * (t_coil - 20.0).max(0.0));
                    // R is judged against the least bound since the
                    // evidence began (since R first went over it, or since
                    // the line read low): the bound rises with the model,
                    // which the bounded line heats with the most power there
                    // can be, and that must not end the wait or clear the
                    // verdict.  On a J316 tweeter on bass, with its line
                    // reading 0.65, the model ran from 95 to 113 C in four
                    // periods of the wait, past the line's R, and the line
                    // was never bounded again (review r4 addendum 6a)
                    let held_bound = bound.min(self.low_set_bound[sk]);
                    let over = if r.is_finite() { r / held_bound } else { f64::INFINITY };
                    if over > 1.0 {
                        self.low_held[sk] += span;
                        self.low_set_bound[sk] = held_bound;
                        if !self.low[sk] && (over >= LOW_FAST || self.low_held[sk] >= LOW_SET_S) {
                            self.low[sk] = true;
                        }
                        if self.low[sk] {
                            self.low_peak[sk] = self.low_peak[sk].max(over);
                        } else {
                            waiting = true;
                        }
                    } else {
                        self.low_held[sk] = 0.0;
                        if !self.low[sk] {
                            self.low_set_bound[sk] = f64::INFINITY;
                        }
                    }
                    // it reads again under LOW_R_CLEAR of that bound
                    if self.low[sk] && r < LOW_R_CLEAR * held_bound {
                        self.low[sk] = false;
                        self.low_peak[sk] = 0.0;
                        self.low_set_bound[sk] = f64::INFINITY;
                    }
                } else {
                    // program at a gain the card may have played it at, with
                    // nothing in the low band to judge the line by
                    self.low_idle[sk] += span;
                    if self.low_idle[sk] >= LOW_FORGET_S && self.low_peak[sk] < LOW_MARGINAL {
                        self.low[sk] = false;
                        self.low_held[sk] = 0.0;
                        self.low_peak[sk] = 0.0;
                        self.low_set_bound[sk] = f64::INFINITY;
                    }
                }
                lost |= self.low[sk];
                // against its left/right twin, from the periods its line
                // reads (the comparison's own verdict aside)
                if let (true, Some(o)) = (g.twin_ratio > 0.0, self.twin[sk]) {
                    let decay = (-span / TWIN_WINDOW_S).exp();
                    let ov = &self.last[o];
                    let take = !(lost || waiting) && f.v_ms >= TWIN_V2_MIN && ov.v_ms >= TWIN_V2_MIN;
                    // the twins' voltages over the period, against each other
                    let os = &self.cfg.speakers[o];
                    let ovs = os.vs_scale / 2147483648.0;
                    let x = if take { (0..frames).map(|q| words[q * ch + s.vs_chan] as f64 * vs * words[q * ch + os.vs_chan] as f64 * ovs).sum::<f64>() } else { 0.0 };
                    let n = frames as f64;
                    let t = &mut self.tw[sk];
                    *t = (t.0 * decay + if take { vi_m * n } else { 0.0 }, t.1 * decay + if take { f.v_ms * n } else { 0.0 }, t.2 * decay + if take { span } else { 0.0 }, t.3 * decay + x);
                    let (a, b) = (self.tw[sk], self.tw[o]);
                    if a.2 >= TWIN_MIN_S && b.2 >= TWIN_MIN_S && b.0 > 0.0 && a.3 >= TWIN_CORR_MIN * (a.1 * b.1).sqrt() {
                        let ratio = (a.0 / a.1) / (b.0 / b.1);
                        if !(ratio >= g.twin_ratio) {
                            self.low_twin[sk] = true;
                        } else if ratio > TWIN_CLEAR * g.twin_ratio {
                            self.low_twin[sk] = false;
                        }
                    }
                    lost |= self.low_twin[sk];
                }
            }
            let bounded = lost || waiting;
            for f in 0..frames {
                let wv = words[f * ch + s.vs_chan] as f64;
                let wi = s.is_chan.map_or(0.0, |ic| words[f * ch + ic] as f64);
                v2.push((wv * vs).powi(2));
                vi.push(if bounded { (wv * vs).powi(2) / z_min } else { sense_power(wv, wi, (vs, is), i_max) });
                if let Some(u) = unified {
                    let room = if wv != 0.0 { 20.0 * (2147483648.0 / wv.abs()).log10() } else { f64::INFINITY };
                    let at = converted_at(room, &possible[sk]);
                    let wide = possible[sk].iter().any(|r| r.0 <= room && r.1 - r.0 > PREDICT_SPREAD_DB);
                    known.push(at <= PREDICT_MAX_DB && !wide);
                    let gain = 10f64.powf(at / 10.0);
                    let vup = (wv.abs() + 1.0) * vs;
                    let iup = (wi.abs() + 1.0) * is;
                    let full_ff = u.v_worst * u.v_worst / u.r_ff;
                    let full_vi = u.v_worst * u.v_worst / u.z_min;
                    pre_ff.push((vup * vup / u.r_ff * gain).min(full_ff));
                    let up = if wi.abs() >= I_RAIL { (vup * iup).max(vup * i_max) } else { vup * iup };
                    pre_vi.push(if bounded {
                        (vup * vup / z_min * gain).min(full_vi)
                    } else if wv * wi > 0.0 || wi.abs() >= I_RAIL {
                        (up * gain).min(full_vi)
                    } else {
                        0.0
                    });
                }
            }
            let mut charge = 0.0;
            if s.is_chan.is_some() {
                // What a line lost where no block could show it may have
                // under-read: the most current there can be, less what the
                // sense read, at the end of the stretch before (its last
                // long block and the rest) and at the start of this one.
                // The end's is charged unless this stretch follows it and
                // shows the line reading; the start's unless the stretch
                // before did.  A lost stretch is bounded throughout.  The
                // charge comes after the stretch's own frames: heat charged
                // later is never cooler (Thermal::impulse).
                let deficit: Vec<f64> = (0..frames).map(|f| {
                    let wv = words[f * ch + s.vs_chan] as f64;
                    let wi = words[f * ch + s.is_chan.unwrap()] as f64;
                    ((wv * vs).powi(2) / z_min - sense_power(wv, wi, (vs, is), i_max)).max(0.0) * step
                }).collect();
                let long = dead_block(DEAD_LONG_S, step);
                let follows = self.gap <= DEAD_GAP_S;
                let reading = !bounded && judged;
                let mut e = 0.0;
                if !(follows && reading) {
                    e += self.pending[sk];
                }
                if !bounded && !(follows && self.reading[sk]) {
                    e += deficit[..long.min(frames)].iter().sum::<f64>();
                }
                charge = e;
                self.pending[sk] = if bounded { 0.0 } else { deficit[frames.saturating_sub(long + frames % long)..].iter().sum() };
                self.reading[sk] = reading;
                let (shown, held) = self.logged[sk];
                let held = if lost != shown { held + 1 } else { 0 };
                if held >= if lost { LOG_LOST } else { LOG_BACK } {
                    self.lost_changed.push((sk, lost));
                    self.logged[sk] = (lost, 0);
                } else {
                    self.logged[sk] = (shown, held);
                }
                self.lost[sk] = lost;
            }
            if let (Some(asahi), Some(ic)) = (self.asahi.as_mut(), s.is_chan) {
                let (a, b) = run(best(&vi));
                asahi[sk].samples(&words[a * ch..b * ch], ch, (s.vs_chan, ic), (vs, is), step, first, g.t_ambient, bounded);
            }
            if let Some(u) = self.unified.as_mut() {
                let vi_pos: Vec<f64> = vi.iter().map(|x| x.max(0.0)).collect();
                let rf = run(best(&v2));
                let rv = run(best(&vi_pos));
                let rp = run(best(&pre_ff));
                let rq = run(best(&pre_vi));
                let has_vi = s.is_chan.is_some();
                // the prediction: the newest k frames' program, over the
                // frames converted at a known gain
                let known_mean = |x: &[f64], r: (usize, usize)| -> Option<f64> {
                    let (sum, n) = (r.0..r.1).filter(|&f| known[f]).fold((0.0, 0usize), |(s, n), f| (s + x[f], n + 1));
                    (n > 0).then(|| sum / n as f64)
                };
                let (pf, pv) = (known_mean(&pre_ff, rp), known_mean(&pre_vi, rq));
                // the first frame on its own when only part of it is new
                let parts: &[(usize, usize, f64)] = if first < 1.0 && k > 1 { &[(0, 1, first), (1, k, 1.0)] } else { &[(0, k, 1.0)] };
                for (i, &(lo, hi_, w)) in parts.iter().enumerate() {
                    let n = hi_ - lo;
                    let blocks = (SUB_BLOCKS * n).div_ceil(g.period).clamp(1, n);
                    // equal shares of the time, as the models take them:
                    // the run's energy is exact however its frames split
                    let means = |x: &[f64], r: (usize, usize)| -> Vec<f64> {
                        let x = &x[r.0 + lo..r.0 + hi_];
                        (0..blocks).map(|b| {
                            let (a, c) = (n * b / blocks, n * (b + 1) / blocks);
                            x[a..c].iter().sum::<f64>() / (n as f64 / blocks as f64)
                        }).collect()
                    };
                    let (bv, bvi, bp) = (means(&v2, rf), means(&vi, rv), means(&pre_ff, rp));
                    let secs = if n == 1 { step * w } else { step * n as f64 };
                    if i > 0 {
                        u[sk].peak_floor = Some(u[sk].peak);
                    }
                    u[sk].blocks(&bv, has_vi.then_some(&bvi[..]), &bp, secs, g.t_ambient);
                }
                u[sk].predict(pf, if has_vi { pv } else { None });
            }
            if charge > 0.0 {
                if let Some(a) = self.asahi.as_mut() {
                    a[sk].charge(charge);
                }
                if let Some(u) = self.unified.as_mut() {
                    u[sk].charge_vi(charge);
                }
            }
        }
        self.gap = 0.0;
        self.govern(span, true);
    }

    /// Time accounted without a measurement: past DEAD_GAP_S (or `idle`)
    /// the next stretch does not follow the last, and what a line lost at
    /// that one's end may have under-read is charged now.
    fn unmeasured(&mut self, d: f64, idle: bool) {
        self.gap = if idle { f64::INFINITY } else { self.gap + d };
        if self.gap > DEAD_GAP_S {
            for (k, e) in std::mem::take(&mut self.pending).into_iter().enumerate() {
                if e > 0.0 {
                    if let Some(a) = self.asahi.as_mut() {
                        a[k].charge(e);
                    }
                    if let Some(u) = self.unified.as_mut() {
                        u[k].charge_vi(e);
                    }
                }
            }
            self.pending = vec![0.0; self.cfg.speakers.len()];
        }
    }

    /// `d` seconds of possible playback no frames measured, at the most the
    /// card's per-speaker `reduction` lets through.
    pub fn worst(&mut self, d: f64, reduction: &[f64]) {
        if !(d > 0.0) {
            return;
        }
        self.unmeasured(d, false);
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
        self.unmeasured(d.max(0.0), true);
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

    /// What the card gets when `gov` governs: each speaker's reduction
    /// (dB, group-linked) and whether a model it answers to latched its
    /// hard limit.  Under the asahi guard the unified governor's card gets
    /// each speaker's larger reduction of the two and either one's latch;
    /// a NaN in either reaches the guard's finite check.
    pub fn governed(&self, gov: Governor) -> (Vec<f64>, bool) {
        let mut per = self.reductions(gov);
        let mut hard = self.hard(gov);
        if let (Governor::Unified, Some(guard)) = (gov, self.cfg.globals.guard) {
            for (r, g) in per.iter_mut().zip(self.reductions(guard)) {
                *r = hi(*r, g);
            }
            hard |= self.hard(guard);
        }
        (per, hard)
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
        if let (Governor::Unified, Some(g)) = (governing, self.cfg.globals.guard) {
            let _ = write!(o, ",\"guard\":\"{}\"", g.name());
        }
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

    /// The bound over a short interval is the coil's own maximum, not
    /// where the drive started: a coil at 139.9 C over a magnet at 138.3 C,
    /// 0.42 W for 100 us (a sliver of uncovered time at 20 dB on the J316
    /// tweeter) peaks at 139.9 C.  The old bound said 157.5 C and latched
    /// the model at every renewal.  Over random states, powers and
    /// intervals the bound is never under the coil at any point inside,
    /// and never more than a fine step above its maximum.
    #[test]
    fn a_constant_power_step_bounds_its_inside_tightly() {
        let mut th = Thermal::new(45.0, 1.3, 50.0, 73.26, 139.9);
        th.t_magnet = 138.3;
        let (bc, _) = th.hold(0.42, 1e-4, 50.0);
        assert!(bc < 139.91, "a 100 us sliver bounded at {bc} C");
        let mut x = 0x9e37_79b9_7f4a_7c15u64;
        let mut f = move || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x >> 11) as f64 / (1u64 << 53) as f64
        };
        for _ in 0..600 {
            let (trc, tauc, trm, taum) = (5.0 + 100.0 * f(), 0.5 + 5.0 * f(), 5.0 + 200.0 * f(), 20.0 + 200.0 * f());
            let c0 = 50.0 + 150.0 * f();
            let m0 = 50.0 + 150.0 * f();
            let p = [0.0, 0.01, 0.4, 3.0, 40.0][(f() * 5.0) as usize];
            let h = [1e-4, 0.01, 0.3, 3.0, 30.0][(f() * 5.0) as usize];
            let mut th = Thermal::new(trc, tauc, trm, taum, c0);
            th.t_magnet = m0;
            let mut fine = th.clone();
            let (bc, bm) = th.hold(p, h, 50.0);
            let (mut mc, mut mm) = (c0, m0);
            let n = 4000;
            for _ in 0..n {
                fine.hold(p, h / n as f64, 50.0);
                mc = mc.max(fine.t_coil);
                mm = mm.max(fine.t_magnet);
            }
            assert!(bc >= mc - 1e-6 && bm >= mm - 1e-6, "{trc}/{tauc}/{trm}/{taum} from {c0}/{m0} at {p} W for {h} s: bound {bc}/{bm}, inside {mc}/{mm}");
            // a fine step's own bound at most above the maximum
            let slack = (p * trc + (m0 - c0).abs()) * (h / n as f64) / tauc + 1e-6;
            assert!(bc <= mc + slack, "{trc}/{tauc}/{trm}/{taum} from {c0}/{m0} at {p} W for {h} s: bound {bc}, maximum {mc}");
        }
    }

    /// Review X3 of speakerguardd#8: a start that knows nothing put the
    /// magnet where steady state would put the drain's coil (736 C on the
    /// J413 tweeter), and the V/I machines sat at their safe level for two
    /// to three minutes.  The start holds the drain from coil and magnet
    /// at the limit instead.  It is never early: from any state under the
    /// limit, any part of the drain at full scale and any time at the safe
    /// level after it, the truth is never hotter than the start's model at
    /// any time after, nor under the working limit later.
    #[test]
    fn a_latched_start_bounds_every_past_it_stands_for() {
        for text in [J316, J413, J700] {
            let c = cfg(text);
            let g = &c.globals;
            let ta = g.t_ambient;
            let mut e = Engine::new(&c, true).unwrap();
            e.assume_latched();
            let safe = 10f64.powf(-g.safe_reduction / 10.0);
            for (k, s) in c.speakers.iter().enumerate() {
                let u = &e.unified.as_ref().unwrap()[k];
                let mut models = vec![(u.ff.clone(), u.v_worst * u.v_worst / u.r_ff)];
                if let Some(v) = &u.vi {
                    models.push((v.clone(), u.v_worst * u.v_worst / u.z_min));
                }
                if let Some(a) = e.asahi.as_ref().map(|a| &a[k]) {
                    assert!(a.hard && a.th.t_coil >= s.t_limit);
                    models.push((a.th.clone(), a.v_worst * a.v_worst / a.z_min));
                }
                let ceiling = s.t_limit - s.t_headroom;
                for (start, full) in models {
                    for c0 in [ta, 100.0, s.t_limit] {
                        for m0 in [ta, 100.0, s.t_limit] {
                            for drain in [0.0, 0.1, DRAIN_S / 2.0, DRAIN_S] {
                                for gap in [0.0, 0.05, 2.0, 30.0] {
                                    let mut truth = start.clone();
                                    truth.t_coil = c0;
                                    truth.t_magnet = m0;
                                    truth.hold(full, drain, ta);
                                    truth.hold(full * safe, gap, ta);
                                    let mut model = start.clone();
                                    let (mut t_cool, mut m_cool) = (None, None);
                                    for i in 0..1200 {
                                        assert!(model.t_coil >= truth.t_coil - 1e-9 && model.t_magnet >= truth.t_magnet - 1e-9,
                                            "{}: from {c0}/{m0}, {drain} s of drain and {gap} s after it, the truth {:?} is hotter than the start {:?} {} s later", s.name, truth, model, i as f64 * 0.5);
                                        if t_cool.is_none() && truth.hottest() < ceiling {
                                            t_cool = Some(i);
                                        }
                                        if m_cool.is_none() && model.hottest() < ceiling {
                                            m_cool = Some(i);
                                        }
                                        truth.hold(full * safe, 0.5, ta);
                                        model.hold(full * safe, 0.5, ta);
                                    }
                                    assert!(m_cool.is_none() || m_cool >= t_cool, "{}: the start cools before the truth", s.name);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// Frames that do not split evenly into blocks: each block was its own
    /// mean, but the models take the blocks as equal shares of the time, so
    /// the longer blocks lost energy (1.8% of the loud one in fleet's J700
    /// case, 274 frames in nine blocks).  The unified models' minute of
    /// power before the gain now holds the run's energy exactly.
    #[test]
    fn uneven_blocks_keep_the_runs_energy() {
        let c = cfg(&J413.replace("period = 4096", "period = 512"));
        let (ch, rate) = (c.globals.channels, 48000.0);
        let n = 274;
        let k = idx(&c, "Left Tweeter");
        let s = &c.speakers[k];
        let mut w = vec![0i32; ch * n];
        for (i, f) in w.chunks_exact_mut(ch).enumerate() {
            // quiet, then loud in the last few frames (the last block)
            f[s.vs_chan] = word(if i >= n - 20 { 9.0 } else { 0.5 }, 14.0) * if i % 2 == 0 { 1 } else { -1 };
            f[s.is_chan.unwrap()] = word(0.1, 3.75);
        }
        let mut e = Engine::new(&c, true).unwrap();
        e.measured(&w, n, n, n, 0, 1.0 / rate, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
        let u = &e.unified.as_ref().unwrap()[k];
        let full = u.v_worst * u.v_worst / u.r_ff;
        let vs = s.vs_scale / 2147483648.0;
        let want: f64 = w.chunks_exact(ch).map(|f| (((f[s.vs_chan] as f64).abs() + 1.0) * vs).powi(2) / u.r_ff).map(|p| p.min(full)).sum::<f64>() / rate;
        let got = u.mean_pre(1.0);
        assert!((got - want).abs() <= 1e-9 * want, "{got} J in the last second, the frames carry {want} J");
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
            e.measured(&words, c.globals.period, c.globals.period, c.globals.period, 0, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
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
        e.measured(&w, c.globals.period, c.globals.period, c.globals.period, 0, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
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
        e.measured(&quiet, c.globals.period, c.globals.period, c.globals.period, 0, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
        let a = &e.asahi.as_ref().unwrap()[k];
        assert!(!a.hard && a.coil_peak < 150.0 && a.th.t_coil < a.coil_peak, "{} / {}", a.th.t_coil, a.coil_peak);
    }

    /// A latched model a little over the limit (a period's uncertain frames
    /// charged at full scale) cools through samples with no negative power:
    /// holding it at the limit there kept the latch for good.  Negative
    /// power after the crossing still leaves it at the limit.
    #[test]
    fn a_latched_model_over_the_limit_cools_unless_negative_power_follows() {
        let c = cfg(J413);
        let k = idx(&c, "Left Tweeter");
        let s = &c.speakers[k];
        let tm = 50.0 + (150.0 - 50.0) * 200.0 / 304.9;
        for negative in [false, true] {
            let mut w = vec![0i32; c.globals.channels * c.globals.period];
            if negative {
                for f in w.chunks_exact_mut(c.globals.channels).skip(2000).take(4) {
                    f[s.vs_chan] = word(1.0, 14.0);
                    f[s.is_chan.unwrap()] = word(-0.5, 3.75);
                }
            }
            let mut e = Engine::new(&c, false).unwrap();
            set_asahi(&mut e, 150.04, tm);
            e.asahi.as_mut().unwrap()[k].hard = true;
            e.measured(&w, c.globals.period, c.globals.period, c.globals.period, 0, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
            let a = &e.asahi.as_ref().unwrap()[k];
            assert!(a.hard, "the latch holds");
            if negative {
                assert!(a.th.t_coil >= 150.0, "negative power after the crossing cooled it to {}", a.th.t_coil);
            } else {
                assert!(a.th.t_coil < 150.0, "silence left it at {}", a.th.t_coil);
            }
        }
    }

    /// Review X1 of speakerguardd#8: sense noise of a few LSB gives nearly
    /// half the samples of a silent period negative V*I, and each held a
    /// latched model over the limit at the limit.  A model a little over
    /// it cools through such noise; negative power beyond the noise floor
    /// after the crossing still leaves it at the limit.
    #[test]
    fn a_latched_model_over_the_limit_cools_through_sense_noise() {
        let c = cfg(J413);
        let k = idx(&c, "Left Tweeter");
        let s = &c.speakers[k];
        let (vc, ic, ch, period) = (s.vs_chan, s.is_chan.unwrap(), c.globals.channels, c.globals.period);
        let tm = 50.0 + (150.0 - 50.0) * 200.0 / 304.9;
        let mut x = 0x2545_f491_4f6c_dd1du64;
        let mut lsb = move |n: i32| {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x % (2 * n as u64 + 1)) as i32 - n
        };
        for (n, dc) in [(1, 0), (3, 2), (9, 8), (30, 10)] {
            for beyond in [false, true] {
                let mut w = vec![0i32; ch * period];
                let mut negative = 0;
                for (i, f) in w.chunks_exact_mut(ch).enumerate() {
                    // offsets of opposite signs: a negative mean as well
                    let (v, cur) = (lsb(n) + dc, lsb(n) - dc);
                    negative += (v * cur < 0) as usize;
                    f[vc] = v << 16;
                    f[ic] = cur << 16;
                    if beyond && (2000..2004).contains(&i) {
                        // 1 V and -250 mA: -250 mW, a current that fits
                        // the voltage (one that reads far too little for
                        // it is a lost line, bounded instead: review r4
                        // BB1)
                        f[vc] = word(1.0, 14.0);
                        f[ic] = word(-0.25, 3.75);
                    }
                }
                assert!(negative > period / 6, "+-{n} LSB: {negative} negative samples");
                let mut e = Engine::new(&c, false).unwrap();
                set_asahi(&mut e, 150.04, tm);
                e.asahi.as_mut().unwrap()[k].hard = true;
                e.check(&w, 48000.0).expect("noise is a measurement");
                e.measured(&w, period, period, period, 0, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
                let a = &e.asahi.as_ref().unwrap()[k];
                assert!(a.hard, "the latch holds");
                if beyond {
                    assert!(a.th.t_coil >= 150.0, "+-{n} LSB: negative power after the crossing cooled it to {}", a.th.t_coil);
                } else {
                    assert!(a.th.t_coil < 150.0, "+-{n} LSB: the noise held it at {}", a.th.t_coil);
                }
            }
        }
    }

    /// Review Z2 of speakerguardd#8: the current sense reads at most
    /// 3.75 A, and a full-scale program into the J316 woofer's 2.7 ohm
    /// draws 4.4 A.  A current word at its rail stands for |V| times the
    /// most current the amplifier can drive into the speaker, not for V
    /// times the rail, so the model heats at least as much as the speaker
    /// can have.
    #[test]
    fn a_current_word_at_its_rail_is_bounded_by_the_voltage() {
        let c = cfg(J316);
        let k = idx(&c, "Left Woofer 1");
        let s = c.speakers[k].clone();
        let (ch, period) = (c.globals.channels, c.globals.period);
        let v = 11.9;
        let mut w = vec![0i32; ch * period];
        for f in w.chunks_exact_mut(ch) {
            f[s.vs_chan] = word(v, 14.0);
            // 4.4 A, which the sense clips to its rail
            f[s.is_chan.unwrap()] = word(4.4, 3.75);
        }
        let mut e = Engine::new(&c, false).unwrap();
        set_asahi(&mut e, 60.0, 55.0);
        // right after a stretch that showed the line reading: nothing a
        // lost line's start may have under-read is charged (the samples'
        // 11.90 V is a hair over the amplifier's 11.899)
        e.reading = vec![true; c.speakers.len()];
        e.gap = 0.0;
        e.measured(&w, period, period, period, 0, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
        let got = e.asahi.as_ref().unwrap()[k].th.t_coil;
        // speakersafetyd's integration at the bound, sample by sample
        let vq = (v / 14.0 * 32768.0).round() / 32768.0 * 14.0;
        let p = vq * v_worst(&s, &c.globals) / s.z_nominal.min(s.r_ff) * ASAHI_MARGIN;
        let step = 1.0 / 48000.0;
        let (ac, am) = (step / (s.vi_tau_coil + step), step / (s.vi_tau_magnet + step));
        let (mut tc, mut tm) = (60.0f64, 55.0f64);
        for _ in 0..period {
            tc = (tm + p * s.vi_tr_coil) * ac + tc * (1.0 - ac);
            tm = (c.globals.t_ambient + p * s.vi_tr_magnet) * am + tm * (1.0 - am);
        }
        assert!((got - tc).abs() < 1e-9, "a clipped period heated the model to {got} C, the bound to {tc} C");
        // under the rail, V*I as it is
        assert_eq!(sense_power(word(v, 14.0) as f64, word(3.0, 3.75) as f64, (14.0 / 2147483648.0, 3.75 / 2147483648.0), 2.7), (word(v, 14.0) as f64 * 14.0 / 2147483648.0) * (word(3.0, 3.75) as f64 * 3.75 / 2147483648.0));
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
        e.measured(&w, c.globals.period, c.globals.period, c.globals.period, 0, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
        let a = &e.asahi.as_ref().unwrap()[k];
        assert!(a.magnet_peak >= 150.0 && a.hard, "magnet peak {} not latched", a.magnet_peak);
        assert!(a.th.t_magnet >= 150.0 && a.th.t_coil < 150.0, "ends at {} / {}", a.th.t_coil, a.th.t_magnet);
        // nonfinite arithmetic latches, and the guard's finite check refuses it
        let mut e = Engine::new(&c, false).unwrap();
        set_asahi(&mut e, 60.0, 60.0);
        e.asahi.as_mut().unwrap()[k].th.tr_magnet = f64::INFINITY;
        e.measured(&w, c.globals.period, c.globals.period, c.globals.period, 0, 1.0 / 48000.0, 1.0, &vec![vec![(0.0, 0.0)]; c.speakers.len()]);
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
                e.unified.as_mut().unwrap()[k].blocks_for_test(b, dt, r[k], ta);
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
                u.blocks_for_test(&[(5e-5, Some(1e-5))], 0.085, 99.5, ta);
            }
            e.govern(0.085, true);
        }
        assert!(e.temps(0).1 < 51.0, "{}", e.temps(0).1);
        // the power before the gain under a muted card is not known: it
        // counts as up to full scale, never more
        let u = &e.unified.as_ref().unwrap()[0];
        assert!(u.mean_pre(1.0) <= u.v_worst * u.v_worst / u.r_ff + 1e-9);
        // and it is not the regulator's prediction (review r4 of
        // speakerguardd#8): nothing is asked for on its account
        assert!(e.reductions(Governor::Unified).iter().all(|&r| r == 0.0), "{:?}", e.reductions(Governor::Unified));
    }

    /// A period of sense words for `c`: every speaker a sine of `amp` volts
    /// peak at 1 kHz into `z` ohm, V and I in phase, S16 widened.
    fn program(c: &Config, amp: f64, z: f64) -> Vec<i32> {
        let (ch, n) = (c.globals.channels, c.globals.period);
        let mut w = vec![0i32; n * ch];
        for f in 0..n {
            let v = amp * (2.0 * std::f64::consts::PI * 1000.0 * f as f64 / 48000.0).sin();
            for s in &c.speakers {
                w[f * ch + s.vs_chan] = ((v / s.vs_scale * 32767.0).round() as i32) << 16;
                if let Some(ic) = s.is_chan {
                    w[f * ch + ic] = ((v / z / s.is_scale * 32767.0).round() as i32) << 16;
                }
            }
        }
        w
    }

    /// `periods` periods of `words` measured at `possible`, each after a
    /// one-frame charge at 0 dB when `sliver` (where the account cannot
    /// place a period's frames closer than a frame, as live: review r4 of
    /// speakerguardd#8).
    fn periods(e: &mut Engine, words: &[i32], periods: usize, sliver: bool, possible: &[Vec<(f64, f64)>]) {
        let n = e.cfg.globals.period;
        let k = e.cfg.speakers.len();
        for _ in 0..periods {
            if sliver {
                e.worst(1.0 / 48000.0, &vec![0.0; k]);
            }
            e.measured(words, n, n, n, 0, 1.0 / 48000.0, 1.0, possible);
        }
    }

    /// Review r4 of speakerguardd#8: the account charges at least a frame
    /// at full scale before every period, and the unified regulator took
    /// that frame for the program of its next horizon.  A quiet program
    /// was held 9 to 14 dB down (13.7 dB on the J616's tweeters through a
    /// whole live session, at 52 C).  Its prediction is the measured
    /// program's now; the charges still heat every model as they did.
    #[test]
    fn a_charge_before_each_quiet_period_holds_no_reduction() {
        for text in [J316, J616] {
            let c = cfg(text);
            let n = c.speakers.len();
            let possible = vec![vec![(0.0, 0.0)]; n];
            let quiet = program(&c, 0.15, 4.0);
            let run = |sliver: bool| {
                let mut e = Engine::new(&c, true).unwrap();
                e.govern(0.0, false);
                periods(&mut e, &quiet, 60, sliver, &possible);
                e
            };
            let (plain, charged) = (run(false), run(true));
            assert!(charged.reductions(Governor::Unified).iter().all(|&r| r == 0.0), "{:?}", charged.reductions(Governor::Unified));
            assert!(plain.reductions(Governor::Unified).iter().all(|&r| r == 0.0));
            // the charges heat every model: at least half of what one
            // frame at full scale adds from ambient, on every model
            for k in 0..n {
                let s = &c.speakers[k];
                let full = v_worst(s, &c.globals).powi(2);
                let frame = 1.0 / 48000.0;
                let least = |tr: f64, tau: f64, r: f64| 0.5 * full / r * tr * (1.0 - (-frame / tau).exp());
                let (pa, pf, pv) = plain.temps(k);
                let (ca, cf, cv) = charged.temps(k);
                assert!(cf - pf >= least(s.tr_coil, s.tau_coil, s.r_ff), "{}: ff {cf} vs {pf}", s.name);
                assert!(cv - pv >= least(s.vi_tr_coil, s.vi_tau_coil, s.z_min()), "{}: vi {cv} vs {pv}", s.name);
                assert!(ca - pa >= least(s.vi_tr_coil, s.vi_tau_coil, s.z_min()), "{}: asahi {ca} vs {pa}", s.name);
            }
        }
    }

    /// The guard on that fix: a full-scale onset measured at a known gain
    /// is attacked within its own period, at the whole of what the
    /// regulator needs for it.
    #[test]
    fn a_full_scale_onset_is_attacked_at_once() {
        for text in [J316, J616] {
            let c = cfg(text);
            let n = c.speakers.len();
            let g = &c.globals;
            let possible = vec![vec![(0.0, 0.0)]; n];
            let mut e = Engine::new(&c, true).unwrap();
            e.govern(0.0, false);
            periods(&mut e, &program(&c, 0.15, 4.0), 30, true, &possible);
            assert!(e.reductions(Governor::Unified).iter().all(|&r| r == 0.0));
            // the amplifier's loudest sine, 18.5 dBV
            let amp = 2f64.sqrt() * 10f64.powf(g.amp_gain_dbv / 20.0);
            let loud = program(&c, amp, 4.0);
            periods(&mut e, &loud, 1, true, &possible);
            for (k, s) in c.speakers.iter().enumerate() {
                let u = &e.unified.as_ref().unwrap()[k];
                let ch = g.channels;
                let vs = s.vs_scale / 2147483648.0;
                let p = (0..g.period).map(|f| ((loud[f * ch + s.vs_chan] as f64).abs() + 1.0).powi(2) * vs * vs / s.r_ff).sum::<f64>() / g.period as f64;
                let target = s.t_limit - s.t_headroom - g.t_ctrl_headroom;
                assert!(u.ff.t_coil < target);
                let need = db(p / u.ff.max_power(target, g.horizon_s, g.t_ambient));
                assert!(need > 3.0, "{}: {need} dB", s.name);
                assert!(u.reduction >= need - 1e-9, "{}: {} dB after the onset, the regulator needs {need} dB", s.name, u.reduction);
            }
        }
    }

    /// Nor do frames that fit one wide candidate range.  With the
    /// controls' value unknown (0 dB to mute: a start that did not follow
    /// a clean stop) the most that fits made audible words full scale: the
    /// J316 asked 13.6 dB on its tweeters and 9.5 dB on its woofers, and
    /// the frames at the card's safe level that followed never replaced
    /// it, 85 s on.  The same for a write in flight from the safe level.
    /// A step of the release is not that wide.
    #[test]
    fn frames_at_an_ambiguous_gain_are_not_the_prediction() {
        let c = cfg(J316);
        let n = c.speakers.len();
        let safe = c.globals.safe_reduction;
        let mut e = Engine::new(&c, true).unwrap();
        e.govern(0.0, false);
        let program = program(&c, 3.5, 4.0);
        periods(&mut e, &program, 6, true, &vec![vec![(0.0, crate::guard::MUTE_DB)]; n]);
        assert!(e.reductions(Governor::Unified).iter().all(|&r| r == 0.0), "{:?}", e.reductions(Governor::Unified));
        periods(&mut e, &program, 3, true, &vec![vec![(0.0, safe)]; n]);
        assert!(e.reductions(Governor::Unified).iter().all(|&r| r == 0.0), "{:?}", e.reductions(Governor::Unified));
        periods(&mut e, &program, 3, true, &vec![vec![(1.5, 2.0)]; n]);
        assert!(e.unified.as_ref().unwrap().iter().all(|u| u.last_ff_pre > 0.1));
    }

    /// Review r4 CC1 of speakerguardd#8: a frame that fits several exact
    /// values (a retake at 4 dB with the hold's 0 dB still in the
    /// look-back, an attack over 1 dB) is the program at the most that
    /// fits.  Left out, a full-scale program after every retake was
    /// invisible to the regulator for half a second: on a dead current
    /// line the measured-power truth went from 90 to 208 C.
    #[test]
    fn frames_that_fit_several_exact_gains_are_the_prediction() {
        let c = cfg(J316);
        let n = c.speakers.len();
        let mut e = Engine::new(&c, true).unwrap();
        e.govern(0.0, false);
        // the amplifier's loudest sine, at 4 dB, with 0 dB still a candidate
        let amp = 2f64.sqrt() * 10f64.powf((c.globals.amp_gain_dbv - 4.0) / 20.0);
        periods(&mut e, &program(&c, amp, 4.0), 1, true, &vec![vec![(0.0, 0.0), (4.0, 4.0)]; n]);
        for (k, u) in e.unified.as_ref().unwrap().iter().enumerate() {
            let s = &c.speakers[k];
            // the program before the 4 dB: the amplifier's full sine
            let full = 10f64.powf(c.globals.amp_gain_dbv / 10.0) / s.r_ff;
            assert!(u.last_ff_pre > 0.9 * full, "{}: predicted {} W, the program is {full} W", s.name, u.last_ff_pre);
            assert!(u.reduction > 3.0, "{}: {} dB", s.name, u.reduction);
        }
    }

    /// Review r4 BB1 of speakerguardd#8: a current line that reads nothing
    /// while the voltage carries a program is lost, and the measured-power
    /// models take V^2 over the smallest impedance: exactly what a live
    /// line at that impedance gives them, the most current there can be.
    #[test]
    fn a_dead_current_line_is_bounded_by_the_voltage() {
        for text in [J316, J413] {
            let c = cfg(text);
            let n = c.speakers.len();
            let possible = vec![vec![(0.0, 0.0)]; n];
            let amp = 4.0;
            let run = |live: bool| {
                let mut e = Engine::new(&c, true).unwrap();
                e.govern(0.0, false);
                let (ch, p) = (c.globals.channels, c.globals.period);
                let mut w = vec![0i32; p * ch];
                for f in 0..p {
                    let v = amp * (2.0 * std::f64::consts::PI * 1000.0 * f as f64 / 48000.0).sin();
                    for s in &c.speakers {
                        w[f * ch + s.vs_chan] = ((v / s.vs_scale * 2147483648.0) as i64).clamp(i32::MIN as i64, i32::MAX as i64) as i32;
                        let i = if live { v / s.z_min() } else { 0.0 };
                        w[f * ch + s.is_chan.unwrap()] = ((i / s.is_scale * 2147483648.0) as i64).clamp(i32::MIN as i64, i32::MAX as i64) as i32;
                    }
                }
                periods(&mut e, &w, 40, true, &possible);
                e
            };
            let (dead, live) = (run(false), run(true));
            assert!(dead.lost.iter().all(|&l| l) && live.lost.iter().all(|&l| !l), "{:?} / {:?}", dead.lost, live.lost);
            for k in 0..n {
                let (a, b) = (&dead.asahi.as_ref().unwrap()[k], &live.asahi.as_ref().unwrap()[k]);
                assert!(a.th.t_coil > 60.0, "{}: the lost line's model stayed at {} C", c.speakers[k].name, a.th.t_coil);
                assert!((a.th.t_coil - b.th.t_coil).abs() < 0.05 && (a.th.t_magnet - b.th.t_magnet).abs() < 0.05, "{}: {} / {} C against the live line's {} / {} C", c.speakers[k].name, a.th.t_coil, a.th.t_magnet, b.th.t_coil, b.th.t_magnet);
                let (uv, lv) = (dead.unified.as_ref().unwrap()[k].vi.as_ref().unwrap(), live.unified.as_ref().unwrap()[k].vi.as_ref().unwrap());
                assert!((uv.t_coil - lv.t_coil).abs() < 0.05, "{}: unified's measured-power model {} against {} C", c.speakers[k].name, uv.t_coil, lv.t_coil);
            }
        }
    }

    /// Stretches of a 10 V sine at 1 kHz into 4 ohm on every speaker of the
    /// J413, the current line dead at the frames where `dead(stretch,
    /// frame)` holds; half a second idle before the stretches in `idle`
    /// (and after the last with `idle_after`).  With `bound` the dead frames
    /// read the most current there can be instead: the truth's bound.
    fn lost_run(dead: &dyn Fn(usize, usize) -> bool, stretches: usize, idle: &[usize], idle_after: bool, bound: bool) -> Engine {
        let c = cfg(J413);
        let (ch, p) = (c.globals.channels, c.globals.period);
        let possible = vec![vec![(0.0, 0.0)]; c.speakers.len()];
        let mut e = Engine::new(&c, true).unwrap();
        e.govern(0.0, false);
        let w32 = |x: f64, fs: f64| ((x / fs * 2147483648.0) as i64).clamp(i32::MIN as i64, i32::MAX as i64) as i32;
        for st in 0..stretches {
            if idle.contains(&st) {
                e.idle(0.5);
            }
            let mut w = vec![0i32; p * ch];
            for f in 0..p {
                let v = 10.0 * (2.0 * std::f64::consts::PI * 1000.0 * (st * p + f) as f64 / 48000.0).sin();
                for s in &c.speakers {
                    w[f * ch + s.vs_chan] = w32(v, s.vs_scale);
                    let i = if !dead(st, f) { v / 4.0 } else if bound { v / s.z_min() } else { 0.0 };
                    w[f * ch + s.is_chan.unwrap()] = w32(i, s.is_scale);
                }
            }
            e.measured(&w, p, p, p, 0, 1.0 / 48000.0, 1.0, &possible);
        }
        if idle_after {
            e.idle(0.5);
        }
        e
    }

    /// Review r4 BB1 of speakerguardd#8: what a lost line hides where no
    /// block can show it is charged, so the models are never cooler than
    /// ones that took the bound from the moment the line died.  A line
    /// lost in the last 30 frames of a stretch (a short block is 48): its
    /// tail, once the next stretch shows the line lost, and at once when
    /// the speakers close after it.  A line that reads again 40 frames
    /// into a stretch: that stretch's start.  And a loud program's line
    /// lost for 5 ms inside a stretch is lost there, though no long block
    /// is mostly dead.
    #[test]
    fn what_a_lost_line_hides_is_charged() {
        let p = cfg(J413).globals.period;
        let tail = move |st: usize, f: usize| st > 0 || f >= p - 30;
        let cases: [(&str, &dyn Fn(usize, usize) -> bool, usize, &[usize], bool, &[bool]); 4] = [
            ("lost at a stretch's end", &tail, 2, &[], false, &[false, true]),
            ("lost at a stretch's end, then closed", &|st, f| st == 0 && f >= p - 30, 1, &[], true, &[false]),
            ("reading again 40 frames in", &|st, f| st == 0 || f < 40, 2, &[], false, &[true, false]),
            ("lost for 5 ms", &|_, f| (2000..2240).contains(&f), 1, &[], false, &[true]),
        ];
        let c = cfg(J413);
        for (what, dead, n, idle, idle_after, lost) in cases {
            let truth = lost_run(dead, n, idle, idle_after, true);
            assert!(truth.lost.iter().all(|&l| !l), "{what}: the bound read as lost");
            // the flags after each stretch
            for st in 0..n {
                let e = lost_run(dead, st + 1, idle, false, false);
                assert!(e.lost.iter().all(|&l| l == lost[st]), "{what}: after stretch {st}: {:?}", e.lost);
            }
            let e = lost_run(dead, n, idle, idle_after, false);
            for k in 0..c.speakers.len() {
                let (a, r) = (&e.asahi.as_ref().unwrap()[k], &truth.asahi.as_ref().unwrap()[k]);
                assert!(a.th.t_coil >= r.th.t_coil - 1e-9 && a.th.t_magnet >= r.th.t_magnet - 1e-9, "{what}: {} asahi {} / {} C under the bound's {} / {} C", c.speakers[k].name, a.th.t_coil, a.th.t_magnet, r.th.t_coil, r.th.t_magnet);
                let (u, v) = (e.unified.as_ref().unwrap()[k].vi.as_ref().unwrap(), truth.unified.as_ref().unwrap()[k].vi.as_ref().unwrap());
                assert!(u.t_coil >= v.t_coil - 1e-9 && u.t_magnet >= v.t_magnet - 1e-9, "{what}: {} unified vi {} C under the bound's {} C", c.speakers[k].name, u.t_coil, v.t_coil);
            }
        }
    }

    /// Review r4 DD3 of speakerguardd#8, after its randomized edge-charge
    /// test: a line that dies at any frame, for three short blocks or more,
    /// with a gap or a close after any stretch, leaves the models no cooler
    /// than ones fed the bound (a live line at z_min) once its stretches are
    /// charged: the stretch after it, a gap, or the speakers closing.
    #[test]
    fn a_line_lost_anywhere_is_charged_by_the_next_stretch() {
        let c = cfg(J413);
        let (ch, p) = (c.globals.channels, c.globals.period);
        let possible = vec![vec![(0.0, 0.0)]; c.speakers.len()];
        let vw = v_worst(&c.speakers[0], &c.globals);
        let mut x = 0x2545_f491_4f6c_dd1du64;
        let mut rnd = |n: u64| {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x % n
        };
        for case in 0..600 {
            let rate = [8000.0, 16000.0, 48000.0][rnd(3) as usize];
            let freq = [0.0, 100.0, 1000.0, 3000.0][rnd(4) as usize];
            let amp = vw * (0.3 + 0.7 * rnd(1000) as f64 / 1000.0);
            let short = dead_block(DEAD_SHORT_S, 1.0 / rate) as u64;
            // a quarter of them die in the last three short blocks of a
            // stretch, where no block of it can show the line lost
            let d0 = if rnd(4) == 0 { (1 + rnd(4)) * p as u64 - 1 - rnd(3 * short) } else { rnd(4 * p as u64) };
            let len = 3 * short + 1 + rnd(2 * p as u64);
            let gap = rnd(4);
            let run = |bound: bool| {
                let mut e = Engine::new(&c, true).unwrap();
                e.govern(0.0, false);
                for st in 0..4u64 {
                    let mut w = vec![0i32; p * ch];
                    for f in 0..p as u64 {
                        let n = st * p as u64 + f;
                        let t = n as f64 / rate;
                        let v = if freq == 0.0 { if n % 2 == 0 { amp } else { -amp } } else { amp * (2.0 * std::f64::consts::PI * freq * t).sin() };
                        let dead = n >= d0 && n < d0 + len;
                        for s in &c.speakers {
                            w[f as usize * ch + s.vs_chan] = ((v / s.vs_scale * 32767.0).round() as i32) << 16;
                            // the bound's current stays under the sense's full scale
                            let i = if !dead { v / 4.0 } else if bound { v / s.z_min() } else { 0.0 };
                            w[f as usize * ch + s.is_chan.unwrap()] = ((i / s.is_scale * 32767.0).round().clamp(-32767.0, 32767.0) as i32) << 16;
                        }
                    }
                    e.measured(&w, p, p, p, 0, 1.0 / rate, 1.0, &possible);
                    if st == gap {
                        e.worst(0.02, &vec![60.0; c.speakers.len()]);
                    }
                }
                e.idle(0.5);
                e
            };
            let (a, b) = (run(false), run(true));
            for k in 0..c.speakers.len() {
                let (m, r) = (&a.asahi.as_ref().unwrap()[k].th, &b.asahi.as_ref().unwrap()[k].th);
                // the bound's current words round to S16, a part in 10^4
                let tol = |r: f64| 1e-4 * (r - c.globals.t_ambient).abs() + 1e-6;
                assert!(m.t_coil >= r.t_coil - tol(r.t_coil) && m.t_magnet >= r.t_magnet - tol(r.t_magnet), "case {case}: {} Hz at {rate} Hz, {amp:.1} V, dead from {d0} for {len}, gap after {gap}: {} asahi {} / {} C under the bound's {} / {} C", freq, c.speakers[k].name, m.t_coil, m.t_magnet, r.t_coil, r.t_magnet);
                let (m, r) = (a.unified.as_ref().unwrap()[k].vi.as_ref().unwrap(), b.unified.as_ref().unwrap()[k].vi.as_ref().unwrap());
                assert!(m.t_coil >= r.t_coil - tol(r.t_coil), "case {case}: {} unified vi {} under {}", c.speakers[k].name, m.t_coil, r.t_coil);
            }
        }
    }

    /// A live line through a reactive load, a sine as low as 20 Hz, is
    /// never read as lost, however its blocks fall on the cycle; nor a
    /// quiet program on a line that carries only its noise.
    #[test]
    fn a_live_line_through_a_reactive_load_is_not_lost() {
        // and at the 8 kHz sense rate, where single short blocks of a 20 Hz
        // load at 60 degrees and 4 times nominal read as lost (review r4 DD2)
        for step in [1.0 / 48000.0, 1.0 / 8000.0] {
        for freq in [20.0, 40.0, 100.0, 1000.0] {
            for phase in [-1.047, -0.5, 0.5, 1.047] {
                for z in [1.0, 2.0, 4.0] {
                    for start in 0..16 {
                        let t0 = start as f64 / 16.0 / freq;
                        let v: Vec<f64> = (0..4096).map(|f| 10.0 * (2.0 * std::f64::consts::PI * freq * (t0 + f as f64 * step)).sin()).collect();
                        let i: Vec<f64> = (0..4096).map(|f| 10.0 / (4.0 * z) * (2.0 * std::f64::consts::PI * freq * (t0 + f as f64 * step) - phase).sin()).collect();
                        assert!(!line_lost(&v, &i, 4.0, step).0, "{freq} Hz, {phase} rad, {z} x nominal, from {t0} s, {} Hz", 1.0 / step);
                    }
                }
            }
        }
        }
        // the review's corner in sense words (review r4 DD2): 20 Hz at 60
        // degrees and 4 times nominal, 8 kHz, S16 with up to 3 LSB of noise
        // and a 3 LSB offset, from 0.15 to 12 V peak
        let step = 1.0 / 8000.0;
        let mut x = 0x9e37_79b9_7f4a_7c15u64;
        let mut noise = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x % 7) as f64 - 3.0
        };
        for amp in [0.15, 0.3, 1.0, 3.0, 12.0] {
            for phase in [-1.047, 1.047] {
                for start in 0..64 {
                    let t0 = start as f64 / 64.0 / 20.0;
                    let q = |x: f64, fs: f64, n: f64, o: f64| ((x / fs * 32768.0).round() + n + o).clamp(-32768.0, 32767.0) / 32768.0 * fs;
                    let v: Vec<f64> = (0..512).map(|f| q(amp * (2.0 * std::f64::consts::PI * 20.0 * (t0 + f as f64 * step)).sin(), 14.0, 0.0, 0.0)).collect();
                    let i: Vec<f64> = (0..512).map(|f| q(amp / 16.0 * (2.0 * std::f64::consts::PI * 20.0 * (t0 + f as f64 * step) - phase).sin(), 3.75, noise(), 3.0)).collect();
                    assert!(!line_lost(&v, &i, 4.0, step).0, "{amp} V, {phase} rad, from {t0} s at 8 kHz");
                }
            }
        }
        let step = 1.0 / 48000.0;
        // 0.05 V rms on a line of 3 LSB of noise: too quiet to judge
        let v: Vec<f64> = (0..4096).map(|f| 0.07 * (f as f64 * 0.13).sin()).collect();
        let i: Vec<f64> = (0..4096).map(|f| if f % 2 == 0 { 3.0 } else { -3.0 } * 3.75 / 32768.0).collect();
        assert_eq!(line_lost(&v, &i, 4.0, step), (false, false));
        // a dead one under a program of a volt: lost
        let v: Vec<f64> = (0..4096).map(|f| 1.4 * (f as f64 * 0.13).sin()).collect();
        assert_eq!(line_lost(&v, &i, 4.0, step), (true, true));
    }

    /// Periods of a sine of `amp` volts at `freq` on every speaker of the
    /// J316, the current reading `k` of a load of `z` times the speaker's
    /// smallest impedance plus `offset` amps; through check() and measured()
    /// as the guard runs them, the models at `t` C.
    fn low_run(freq: f64, amp: f64, z: f64, k: f64, offset: f64, periods: usize, t: f64) -> Engine {
        low_run_with(freq, amp, z, k, offset, periods, t, &|_| Some(t))
    }

    /// ... with the asahi models set to `t` at the start and to `at(n)`
    /// after period n (None: left as the period took them)
    #[allow(clippy::too_many_arguments)]
    fn low_run_with(freq: f64, amp: f64, z: f64, k: f64, offset: f64, periods: usize, t: f64, at: &dyn Fn(usize) -> Option<f64>) -> Engine {
        let c = cfg(J316);
        let (ch, p) = (c.globals.channels, c.globals.period);
        let possible = vec![vec![(0.0, 0.0)]; c.speakers.len()];
        let mut e = Engine::new(&c, true).unwrap();
        set_asahi(&mut e, t, t);
        e.govern(0.0, false);
        let w32 = |x: f64, fs: f64| ((x / fs * 32767.0).round().clamp(-32768.0, 32767.0) as i32) << 16;
        for n in 0..periods {
            let mut w = vec![0i32; p * ch];
            for f in 0..p {
                let x = 2.0 * std::f64::consts::PI * freq * (n * p + f) as f64 / 48000.0;
                for s in &c.speakers {
                    w[f * ch + s.vs_chan] = w32(amp * x.sin(), s.vs_scale);
                    w[f * ch + s.is_chan.unwrap()] = w32(k * amp / (z * s.z_min()) * x.sin() + offset, s.is_scale);
                }
            }
            e.check(&w, 48000.0).unwrap();
            e.measured(&w, p, p, p, 0, 1.0 / 48000.0, 1.0, &possible);
            if let Some(t) = at(n) {
                set_asahi(&mut e, t, t);
            }
        }
        e
    }

    /// Review r4 DD1 of speakerguardd#8: a line that reads low but
    /// plausibly, or a dead one carrying a DC offset, is lost too.  A line
    /// reading 0.4 of a load at 1.2 times z_min puts the low-band
    /// resistance at 3 times z_min, over the bound a live coil can reach
    /// even at its limit; within the window it is lost.  A dead line with
    /// 0.2 A of offset reads a plausible rms against 3 V, but its current
    /// does not follow the voltage.  A hot coil is no low line: a live line
    /// at 1.75 times z_min is lost with the model at 20 C, not at 120 C
    /// (the J616s's woofers at full volume read 1.28 to 1.49).
    #[test]
    fn a_line_that_reads_low_or_off_its_voltage_is_lost() {
        // 3 s of 60 Hz at 3 V, 85 ms a period
        let low = low_run(60.0, 3.0, 1.2, 0.4, 0.0, 40, 50.0);
        assert!(low.lost.iter().all(|&l| l), "a line reading 0.4: {:?}", low.lost);
        let live = low_run(60.0, 3.0, 1.2, 1.0, 0.0, 40, 50.0);
        assert!(live.lost.iter().all(|&l| !l), "a live line: {:?}", live.lost);
        let offset = low_run(1000.0, 3.0, 1.2, 0.0, 0.2, 2, 50.0);
        assert!(offset.lost.iter().all(|&l| l), "a dead line with an offset: {:?}", offset.lost);
        // and one with 15 mA of offset on a quiet program (0.4 V rms), too
        // much current for the dead-line test and too little for the old
        // 20 mA gate (review r4 FF1): lost within its 2 s
        let small = low_run(1000.0, 0.57, 1.2, 0.0, 0.015, 30, 50.0);
        assert!(small.lost.iter().all(|&l| l), "a dead line with 15 mA of offset at 0.4 V: {:?}", small.lost);
        let cold = low_run(60.0, 3.0, 1.75, 1.0, 0.0, 40, 20.0);
        assert!(cold.lost.iter().all(|&l| l), "1.75 times z_min at 20 C: {:?}", cold.lost);
        let hot = low_run(60.0, 3.0, 1.75, 1.0, 0.0, 40, 120.0);
        assert!(hot.lost.iter().all(|&l| !l), "1.75 times z_min at 120 C: {:?}", hot.lost);
        // review r4 addendum 6a: a line reading 0.11 on bass is bounded in
        // its first period, and one reading 0.6 is flagged
        let first = low_run(60.0, 3.0, 1.2, 0.11, 0.0, 1, 50.0);
        assert!(first.lost.iter().all(|&l| l), "a line reading 0.11, first period: {:?}", first.lost);
        let six = low_run(60.0, 3.0, 1.2, 0.6, 0.0, 40, 50.0);
        assert!(six.lost.iter().all(|&l| l), "a line reading 0.6: {:?}", six.lost);
        // the review's misfire at +10 dB of replay gain was 1.002 of the
        // bound (1.55 times z_min with the model at 59 C): in the marginal
        // band a verdict waits a second of program
        let real = low_run(60.0, 3.0, 1.55, 1.0, 0.0, 5, 59.0);
        assert!(real.lost.iter().all(|&l| !l), "1.55 times z_min at 59 C, five periods: {:?}", real.lost);
        // ... but its power is bounded while it waits: three periods of a
        // line reading 0.65 (1.19 of the bound at 59 C) heat the asahi
        // models as a dead line's do (review r4 addendum 6a)
        let free = |_: usize| None;
        let waits = low_run_with(60.0, 3.0, 1.2, 0.65, 0.0, 3, 59.0, &free);
        let dead = low_run_with(60.0, 3.0, 1.2, 0.0, 0.0, 3, 59.0, &free);
        assert!(waits.lost.iter().all(|&l| !l), "a verdict in the marginal band did not wait: {:?}", waits.lost);
        for (w, d) in waits.asahi.as_ref().unwrap().iter().zip(dead.asahi.as_ref().unwrap()) {
            assert!((w.th.t_coil - d.th.t_coil).abs() < 1e-9, "a waiting line was not bounded: {} against {} C", w.th.t_coil, d.th.t_coil);
        }
        // and the wait is judged against the bound it began at: the
        // bounded model heats (a J316 tweeter ran from 95 to 113 C in four
        // periods), and its bound passing the line's R must not end the
        // wait.  The same line, the model climbing from 59 to 125 C over
        // the first six periods, reads low after its second
        let climb = |n: usize| Some(59.0 + 11.0 * (n + 1).min(6) as f64);
        let heats = low_run_with(60.0, 3.0, 1.2, 0.65, 0.0, 20, 59.0, &climb);
        assert!(heats.lost.iter().all(|&l| l), "a wait ended by the bounded model's heat: {:?}", heats.lost);
        // and once it reads low, the model's heat must not clear it: set
        // after its second at 59 C, it stays low with the model at 165 C,
        // where 0.9 of the bound now would be over its R
        let hot = |n: usize| Some(if n < 14 { 59.0 } else { 165.0 });
        let kept = low_run_with(60.0, 3.0, 1.2, 0.65, 0.0, 40, 59.0, &hot);
        assert!(kept.lost.iter().all(|&l| l), "a verdict cleared by the bounded model's heat: {:?}", kept.lost);
        let c = cfg(J316);
        let (ch, p) = (c.globals.channels, c.globals.period);
        let treble = |level: f64| -> Vec<i32> {
            (0..p * ch).map(|q| {
                let (f, k) = (q / ch, q % ch);
                let x = (level * (2.0 * std::f64::consts::PI * 3000.0 * f as f64 / 48000.0).sin() / 14.0 * 32767.0) as i32;
                if c.speakers.iter().any(|s| s.vs_chan == k) { x << 16 } else { (x / 3) << 16 }
            }).collect()
        };
        // the models held where they were, so that only the low-band test
        // decides
        let pass = |e: &mut Engine, w: &[i32], n: usize, possible: &[Vec<(f64, f64)>], t: f64| {
            for _ in 0..n {
                e.check(w, 48000.0).unwrap();
                e.measured(w, p, p, p, 0, 1.0 / 48000.0, 1.0, possible);
                set_asahi(e, t, t);
            }
        };
        let open = vec![vec![(0.0, 0.0)]; c.speakers.len()];
        // a line reading 0.2 stays low through 80 s of treble (bass, treble,
        // bass: review r4 addendum 6a), where nothing judges it
        let mut e = low_run(60.0, 3.0, 1.2, 0.2, 0.0, 240, 50.0);
        assert!(e.low.iter().all(|&l| l));
        pass(&mut e, &treble(3.0), 950, &open, 50.0);
        assert!(e.low.iter().all(|&l| l), "a line reading 0.2 was forgotten in treble: {:?}", e.low);
        // a marginal verdict (a hot coil just over the bound, the misfire's
        // kind) clears after 30 s of treble, not of the card's safe level
        let mut m = low_run(60.0, 3.0, 1.6, 1.0, 0.0, 40, 59.0);
        assert!(m.low.iter().all(|&l| l), "1.6 times z_min at 59 C for 3 s: {:?}", m.low);
        let safe = vec![vec![(99.5, 99.5)]; c.speakers.len()];
        pass(&mut m, &treble(3.0), 600, &safe, 59.0);
        assert!(m.low.iter().all(|&l| l), "a marginal flag cleared at the card's safe level");
        pass(&mut m, &treble(3.0), 600, &open, 59.0);
        assert!(m.low.iter().all(|&l| !l), "a marginal flag stuck: {:?}", m.low);
    }

    /// The power-factor test's voltage gate: the M2's right woofer read
    /// 0.10 V rms against 25 mA with the current near a right angle to the
    /// voltage (a power factor of -0.015 to 0.014) for four periods of a
    /// quiet passage, and was bounded and logged for it.  Under 0.5 V the
    /// condition has to hold for 2 s of program (review r4 EE1): those four
    /// periods are not enough, three seconds at 0.2 V are; the same current
    /// at 1 V is no reading at once.
    #[test]
    fn a_quiet_line_at_a_right_angle_is_not_judged() {
        let run = |v_rms: f64, periods: usize| {
            let c = cfg(J413);
            let (ch, p) = (c.globals.channels, c.globals.period);
            let possible = vec![vec![(0.0, 0.0)]; c.speakers.len()];
            let mut e = Engine::new(&c, true).unwrap();
            e.govern(0.0, false);
            let w16 = |x: f64, fs: f64| ((x / fs * 32767.0).round().clamp(-32768.0, 32767.0) as i32) << 16;
            for n in 0..periods {
                let mut w = vec![0i32; p * ch];
                for f in 0..p {
                    let x = 2.0 * std::f64::consts::PI * 1000.0 * (n * p + f) as f64 / 48000.0;
                    for s in &c.speakers {
                        w[f * ch + s.vs_chan] = w16(v_rms * 2f64.sqrt() * x.sin(), s.vs_scale);
                        w[f * ch + s.is_chan.unwrap()] = w16(0.0255 * 2f64.sqrt() * x.cos(), s.is_scale);
                    }
                }
                e.check(&w, 48000.0).unwrap();
                e.measured(&w, p, p, p, 0, 1.0 / 48000.0, 1.0, &possible);
            }
            e.lost.clone()
        };
        assert!(run(0.10, 4).iter().all(|&l| !l), "0.1 V at a right angle for four periods was lost");
        assert!(run(0.2, 36).iter().all(|&l| l), "0.2 V at a right angle for 3 s reads");
        assert!(run(1.0, 1).iter().all(|&l| l), "1 V at a right angle reads");
    }

    /// Review r4 DD1 of speakerguardd#8, what V and I alone cannot see: a
    /// tweeter line reading half on a treble program (no low-band current
    /// to judge it by).  Its effective admittance against its right twin's
    /// gives it away within the window, the twin's own line stays read,
    /// and nothing is compared with the comparison off, with the right side
    /// silent (a source panned left), or with the twins playing different
    /// programs.
    #[test]
    fn a_line_reading_low_against_its_twin_is_lost() {
        let run = |ratio: f64, right_amp: f64, right_freq: f64| {
            let mut c = cfg(J413);
            c.globals.twin_ratio = ratio;
            let (ch, p) = (c.globals.channels, c.globals.period);
            let possible = vec![vec![(0.0, 0.0)]; c.speakers.len()];
            let mut e = Engine::new(&c, true).unwrap();
            e.govern(0.0, false);
            let w16 = |x: f64, fs: f64| ((x / fs * 32767.0).round().clamp(-32768.0, 32767.0) as i32) << 16;
            // 8 s of periods
            for n in 0..95 {
                let mut w = vec![0i32; p * ch];
                for f in 0..p {
                    let t = (n * p + f) as f64 / 48000.0;
                    for s in &c.speakers {
                        let left = s.name.contains("Left");
                        let (amp, freq) = if left { (3.0, 3000.0) } else { (right_amp, right_freq) };
                        let v = amp * (2.0 * std::f64::consts::PI * freq * t).sin();
                        w[f * ch + s.vs_chan] = w16(v, s.vs_scale);
                        w[f * ch + s.is_chan.unwrap()] = w16(if left { 0.5 } else { 1.0 } * v / (1.2 * s.z_min()), s.is_scale);
                    }
                }
                e.check(&w, 48000.0).unwrap();
                e.measured(&w, p, p, p, 0, 1.0 / 48000.0, 1.0, &possible);
            }
            let names: Vec<String> = c.speakers.iter().map(|s| s.name.clone()).collect();
            names.into_iter().zip(e.lost.clone()).collect::<Vec<_>>()
        };
        let lost = run(0.8, 3.0, 3000.0);
        assert!(lost.iter().all(|(n, l)| *l == n.contains("Left")), "the left side reading half: {lost:?}");
        for (what, r) in [("off", run(0.0, 3.0, 3000.0)), ("panned left", run(0.8, 0.0, 3000.0)), ("different programs", run(0.8, 3.0, 4700.0))] {
            assert!(r.iter().all(|(_, l)| !l), "{what}: {r:?}");
        }
    }

    /// Review r4 DD2: a line flapping between lost and reading every
    /// stretch is logged once, not every period; the bound follows it.
    #[test]
    fn a_line_at_the_threshold_is_logged_once() {
        let c = cfg(J316);
        let (ch, p) = (c.globals.channels, c.globals.period);
        let possible = vec![vec![(0.0, 0.0)]; c.speakers.len()];
        let mut e = Engine::new(&c, true).unwrap();
        e.govern(0.0, false);
        let mut changes = 0;
        for n in 0..40 {
            let dead = n % 2 == 0;
            let mut w = vec![0i32; p * ch];
            for f in 0..p {
                let v = 3.0 * (2.0 * std::f64::consts::PI * 1000.0 * f as f64 / 48000.0).sin();
                for s in &c.speakers {
                    w[f * ch + s.vs_chan] = ((v / s.vs_scale * 32767.0).round() as i32) << 16;
                    w[f * ch + s.is_chan.unwrap()] = if dead { 0 } else { ((v / 4.0 / s.is_scale * 32767.0).round() as i32) << 16 };
                }
            }
            e.measured(&w, p, p, p, 0, 1.0 / 48000.0, 1.0, &possible);
            assert!(e.lost.iter().all(|&l| l == dead), "the bound follows the line");
            changes += std::mem::take(&mut e.lost_changed).len();
        }
        assert_eq!(changes, 0, "a line flapping every period was logged {changes} times");
    }

    /// Frames the card held at its safe level say nothing of the program:
    /// they do not become the regulator's prediction (at full scale, the
    /// most a word inverted at -99.5 dB can stand for, they kept a stream
    /// that plays through a restart muted), while the budgets still count
    /// them at up to full scale.
    #[test]
    fn frames_at_the_cards_safe_level_are_not_the_prediction() {
        let c = cfg(J316);
        let n = c.speakers.len();
        let safe = c.globals.safe_reduction;
        let mut e = Engine::new(&c, true).unwrap();
        e.govern(0.0, false);
        // the noise a muted card's sense lines carry: a few LSB
        let noise = program(&c, 14.0 * 4.0 / 32768.0, 4.0);
        periods(&mut e, &noise, 60, false, &vec![vec![(safe, safe)]; n]);
        assert!(e.reductions(Governor::Unified).iter().all(|&r| r == 0.0), "{:?}", e.reductions(Governor::Unified));
        let u = &e.unified.as_ref().unwrap()[0];
        assert!(u.mean_pre(1.0) > 1.0, "the budgets no longer count a muted card's frames as loud: {} W", u.mean_pre(1.0));
        // the same frames at a known gain are a prediction like any other
        periods(&mut e, &program(&c, 4.0, 4.0), 5, false, &vec![vec![(0.0, 0.0)]; n]);
        assert!(e.unified.as_ref().unwrap().iter().all(|u| u.last_ff_pre > 1.0));
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

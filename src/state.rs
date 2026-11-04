// SPDX-License-Identifier: MIT
//! The model's state across restarts.  A daemon that exits leaves the
//! speakers at the card's safe level; the one systemd starts next must not
//! begin with a cold coil, or a crash would reset the protection.  The file
//! under /run, stamped with CLOCK_BOOTTIME, is rewritten every half second
//! as a status readout (`cat /run/speakerguardd/state`: temperatures, recent
//! power, sensed RMS voltage, reduction, whether the daemon holds the card),
//! and once more by a clean stop, which alone is resumed: written after the
//! lock was given up and drained, its model covering every moment up to the
//! write, marked `released 1`.  A run that resumes it overwrites it before
//! it can unlock.  The time since is charged at the card's safe level;
//! anything else starts every speaker from independent hot coil and magnet
//! bounds, including the queued drain and the safe-level floor, latched.
//!
//! The engine's models (engine.rs) are saved in the same file, after the
//! legacy speaker lines (`asahi ...`, `unified ...`): one atomic write and
//! the same clean-stop rule for every model.  A model the file does not
//! carry starts latched at its limit. Generation 2 snapshots predate the
//! independent hot-node bounds and are refused even after a clean stop.
//! Readers from before generation 3 refuse it rather than resuming its temperatures.

use std::time::Duration;

/// The state text rounds temperatures to six decimals. A one-microkelvin
/// allowance dominates the half-microkelvin encoding error at either node
/// and floating-point evaluation noise. The thermal system is monotone
/// and does not amplify a uniform initial temperature error.
pub(crate) const RECORD_TEMP_EPS: f64 = 1e-6;

use crate::config::Globals;
use crate::engine::{state_name, Engine, SavedAsahi, SavedUnified};
use crate::model::SpeakerState;

// A clean stop from an older initializer is not a temperature bound.
// New readers reject v2, and old readers reject v3 on rollback.
const MAGIC: &str = "speakerguardd-state 3";
/// The format 0.1.2 to 0.1.5 wrote: never resumed.
const MAGIC_V1: &str = "speakerguardd-state 1";

#[derive(Debug, Clone, PartialEq)]
pub struct SavedSpeaker {
    pub name: String,
    pub t_coil: f64,
    pub t_magnet: f64,
    pub mean_1s: f64,
    pub mean_60s: f64,
    /// The governor's reduction being released (dB).
    pub reduction_db: f64,
    /// The latched hard fault and the peak bound.
    pub hard: bool,
    pub t_peak: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Saved {
    /// When the file was written.
    pub boottime: Duration,
    /// Up to when the model it holds has accounted the speakers' power.
    pub covered: Duration,
    /// Whether the run held the card's volume lock at the write (status).
    pub armed: bool,
    /// Written by a clean stop: the lock given up, its converted samples
    /// drained, every moment up to `covered` accounted.  Only such a state
    /// is resumed.
    pub released: bool,
    /// The run was recovering from bad or stalled sense data.
    pub recovering: bool,
    pub speakers: Vec<SavedSpeaker>,
    /// The engine's models (none in a file written without them).
    pub asahi: Vec<SavedAsahi>,
    pub unified: Vec<SavedUnified>,
}

impl Saved {
    /// A clean stop's state: released, and its model covering up to the
    /// write.  The only kind a restart resumes.
    pub fn proven(&self) -> bool {
        self.released && self.covered == self.boottime
    }
}

#[cfg(test)]
pub fn encode(boottime: Duration, covered: Duration, speakers: &[SpeakerState], armed: bool, released: bool) -> String {
    encode_with(boottime, covered, speakers, None, armed, released)
}

/// The state with the engine's models.
pub fn encode_with(boottime: Duration, covered: Duration, speakers: &[SpeakerState], engine: Option<&Engine>, armed: bool, released: bool) -> String {
    encode_full(boottime, covered, speakers, engine, armed, released, false)
}

/// `encode_with`, and whether the run is still recovering from bad or stalled sense
/// data (a `recovering 1` line: the next run does not take the lock before
/// REARM_PERIODS valid periods either; a reader without it refuses the
/// file, and so starts latched).
pub fn encode_full(boottime: Duration, covered: Duration, speakers: &[SpeakerState], engine: Option<&Engine>, armed: bool, released: bool, recovering: bool) -> String {
    let mut out = format!(
        "{MAGIC}\nboottime_ns {}\ncovered_ns {}\narmed {}\nreleased {}\n",
        boottime.as_nanos(),
        covered.min(boottime).as_nanos(),
        armed as u8,
        (released && !armed) as u8
    );
    if recovering {
        out += "recovering 1\n";
    }
    out += "# speaker name t_coil_C t_magnet_C mean_1s_W mean_60s_W v_rms_V reduction_dB hard t_peak_C\n";
    for s in speakers {
        out += &format!(
            "speaker {} {:.6} {:.6} {:.6} {:.6} {:.6} {:.3} {} {:.6}\n",
            state_name(&s.spec.name),
            s.t_coil,
            s.t_magnet,
            s.mean_power(1.0),
            s.mean_power(60.0),
            s.v_rms,
            s.reduction_db,
            s.hard as u8,
            s.t_peak.max(s.t_coil)
        );
    }
    if let Some(e) = engine {
        out += &e.encode();
    }
    out += "end\n";
    out
}

/// A temperature a file may carry (°C): finite and physically possible
/// from below, as the speaker lines.  A bound model charged at full scale
/// can pass any physical temperature; it is still the model.
fn temperature(x: &str) -> Option<f64> {
    x.parse::<f64>().ok().filter(|v| v.is_finite() && *v > -50.0)
}

fn flag(x: &str) -> Option<bool> {
    match x {
        "0" => Some(false),
        "1" => Some(true),
        _ => None,
    }
}

fn asahi_line(f: &[&str]) -> Option<SavedAsahi> {
    if f.len() != 9 {
        return None;
    }
    let gain: f64 = f[6].parse().ok().filter(|g: &f64| g.is_finite() && *g <= 0.0)?;
    Some(SavedAsahi {
        name: f[1].to_string(),
        t_coil: temperature(f[2])?,
        t_magnet: temperature(f[3])?,
        coil_hyst: temperature(f[4])?,
        magnet_hyst: temperature(f[5])?,
        gain,
        hard: flag(f[7])?,
        t_peak: temperature(f[8])?,
    })
}

fn unified_line(f: &[&str]) -> Option<SavedUnified> {
    if f.len() != 11 {
        return None;
    }
    let vi = match (f[4], f[5]) {
        ("-", "-") => None,
        (c, m) => Some((temperature(c)?, temperature(m)?)),
    };
    let watts = |x: &str| x.parse::<f64>().ok().filter(|v| v.is_finite() && *v >= 0.0);
    Some(SavedUnified {
        name: f[1].to_string(),
        ff: (temperature(f[2])?, temperature(f[3])?),
        vi,
        reduction: watts(f[6])?,
        mean_1s: watts(f[7])?,
        mean_60s: watts(f[8])?,
        hard: flag(f[9])?,
        t_peak: temperature(f[10])?,
    })
}

/// None for anything but a complete, finite, physically possible current-generation state.
/// A v1 file decodes as never released (it is not resumed).
pub fn decode(text: &str) -> Option<Saved> {
    let mut lines = text.lines();
    let current = match lines.next()? {
        MAGIC => true,
        MAGIC_V1 => false,
        _ => return None,
    };
    let ns: u64 = lines.next()?.strip_prefix("boottime_ns ")?.parse().ok()?;
    let mut covered = None;
    let mut speakers = Vec::new();
    let (mut asahi, mut unified) = (Vec::new(), Vec::new());
    let mut armed = true;
    let mut released = false;
    let mut recovering = false;
    let mut ended = false;
    for line in lines {
        if ended {
            return None;
        }
        if line.starts_with('#') {
            continue;
        }
        if line == "end" {
            ended = true;
            continue;
        }
        let flag = |v: &str| match v {
            "0" => Some(false),
            "1" => Some(true),
            _ => None,
        };
        if let Some(v) = line.strip_prefix("armed ") {
            armed = flag(v)?;
            continue;
        }
        if let Some(v) = line.strip_prefix("recovering ") {
            recovering = flag(v)?;
            continue;
        }
        if let Some(v) = line.strip_prefix("released ") {
            released = flag(v)?;
            continue;
        }
        if let Some(v) = line.strip_prefix("covered_ns ") {
            covered = Some(v.parse::<u64>().ok()?);
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        if current && f.first() == Some(&"asahi") {
            asahi.push(asahi_line(&f)?);
            continue;
        }
        if current && f.first() == Some(&"unified") {
            unified.push(unified_line(&f)?);
            continue;
        }
        let fields = if current { 10 } else { 8 };
        if f.len() != fields || f[0] != "speaker" {
            return None;
        }
        let v: Vec<f64> = f[2..8].iter().map(|x| x.parse().ok()).collect::<Option<_>>()?;
        if v.iter().any(|x| !x.is_finite()) || v[2] < 0.0 || v[3] < 0.0 || v[0] < -50.0 || v[1] < -50.0 || v[5] < 0.0 {
            return None;
        }
        let (hard, t_peak) = if current {
            let peak: f64 = f[9].parse().ok().filter(|p: &f64| p.is_finite())?;
            (flag(f[8])?, peak)
        } else {
            (true, v[0])
        };
        speakers.push(SavedSpeaker { name: f[1].to_string(), t_coil: v[0], t_magnet: v[1], mean_1s: v[2], mean_60s: v[3], reduction_db: v[5], hard, t_peak });
    }
    if speakers.is_empty() || (current && !ended) {
        return None;
    }
    let covered = Duration::from_nanos(covered.unwrap_or(0).min(ns));
    Some(Saved { boottime: Duration::from_nanos(ns), covered, armed, released: current && released && !armed, recovering, speakers, asahi, unified })
}

/// Continue every speaker from a clean stop's state and charge the time
/// since at the worst case the card's own safe level lets through (no
/// daemon held the card), a full-scale signal before the gain.  Anything else - a v1 file, a state not written
/// by a clean stop, one whose model did not cover up to its write, a
/// speaker it does not name - starts from independent hot coil and magnet
/// bounds, latched. True
/// if the state was resumed.
pub fn restore(speakers: &mut [SpeakerState], saved: &Saved, now: Duration, globals: &Globals) -> bool {
    // The voltage cap is derived from the active configuration, not saved.
    for speaker in speakers.iter_mut() { speaker.configure_voltage(globals); }
    let proven = saved.proven();
    let elapsed = now.saturating_sub(saved.boottime).as_secs_f64();
    for s in speakers.iter_mut() {
        let x = match saved.speakers.iter().find(|x| x.name == state_name(&s.spec.name)) {
            Some(x) if proven => x,
            _ => {
                s.assume_latched(globals);
                continue;
            }
        };
        s.resume(x.t_coil, x.t_magnet, x.mean_1s, x.mean_60s);
        s.resume_governor(Some(x.hard), x.t_peak, x.reduction_db);
        // the gap: what the card's safe level lets through for the heat,
        // and a full-scale signal before the gain for the budgets (review
        // r5 V3: the input may have been full scale all along)
        let p = s.worst_case_power(globals.safe_reduction);
        let pre = s.worst_case_power(0.0);
        let mut left = elapsed;
        if left > 3600.0 {
            // an hour at it is its steady state for any real speaker
            let hard = s.hard;
            s.settle_at(p, pre, globals.t_ambient);
            s.hard = hard;
        } else {
            while left > 0.0 {
                let dt = left.min(1.0);
                s.step_pre(p, pre, dt, globals.t_ambient);
                left -= dt;
            }
        }
    }
    proven
}

/// The engine's models by the same rule as `restore`: from a clean stop's
/// state, charged since at the safe level's worst case; from anything else,
/// or for a model the file does not carry, latched at the limit.  True if
/// the state was resumed.
pub fn restore_engine(engine: &mut Engine, saved: &Saved, now: Duration, globals: &Globals) -> bool {
    if !saved.proven() {
        engine.assume_latched();
        return false;
    }
    let elapsed = now.saturating_sub(saved.boottime).as_secs_f64();
    engine.resume(&saved.asahi, &saved.unified, elapsed, globals.safe_reduction);
    true
}

/// Restore an uncertain cross-boot age. Histories use the longest age at
/// full-scale pre-gain power. Temperatures bound the entire interval,
/// including a coil warmed by a cooling magnet between the endpoints.
pub fn restore_interval(speakers: &mut [SpeakerState], engine: &mut Option<Engine>, saved: &Saved, lo: Duration, hi: Duration, globals: &Globals) -> bool {
    if hi < lo || !restore(speakers, saved, hi, globals) {
        return false;
    }
    let mut low = speakers.to_vec();
    restore(&mut low, saved, lo, globals);
    let start = lo.saturating_sub(saved.boottime).as_secs_f64();
    let span = (hi - lo).as_secs_f64();
    for (s, low) in speakers.iter_mut().zip(&low) {
        let Some(x) = saved.speakers.iter().find(|x| x.name == state_name(&s.spec.name)) else { continue };
        let p = s.worst_case_power(globals.safe_reduction);
        let spec = s.spec.clone();
        let mut th = crate::engine::Thermal::new(spec.tr_coil, spec.tau_coil, spec.tr_magnet, spec.tau_magnet, x.t_coil);
        th.t_magnet = x.t_magnet;
        th.hold(p, start, globals.t_ambient);
        // Legacy heat steps advance the magnet before the coil. Bound
        // both that discretization and the continuous plant. The magnet
        // is monotone; neither update can leave this invariant rectangle:
        // M <= max(M(lo), M(infinity)), C <= max(C(lo), M_bound + P Rc).
        // This also covers the arbitrary phase of the legacy substeps.
        let magnet = th.t_magnet.max(low.t_magnet).max(globals.t_ambient + p * spec.tr_magnet);
        let coil = th.t_coil.max(low.t_coil).max(magnet + p * spec.tr_coil);
        let (pc, pm) = th.hold(p, span, globals.t_ambient);
        s.raise_to(low);
        s.t_coil = s.t_coil.max(coil).max(pc) + RECORD_TEMP_EPS;
        s.t_magnet = s.t_magnet.max(magnet).max(pm) + RECORD_TEMP_EPS;
        s.t_peak = (s.t_peak.max(x.t_peak) + RECORD_TEMP_EPS).max(s.t_coil);
        s.hard |= s.t_peak >= spec.t_limit;
    }
    if let Some(e) = engine {
        restore_engine(e, saved, hi, globals);
        e.bound_resume_interval(saved, lo, hi);
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Config, Governor};

    const J700: &str = include_str!("../conf/apple/j700.conf");

    fn hot() -> (Config, Vec<SpeakerState>) {
        let cfg = Config::parse(J700).unwrap();
        let mut sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), 50.0)).collect();
        for s in sp.iter_mut() {
            for _ in 0..300 {
                s.step(1.5, 0.1, 50.0); // 30 s at 1.5 W
            }
        }
        (cfg, sp)
    }

    fn fresh(cfg: &Config) -> Vec<SpeakerState> {
        cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), cfg.globals.t_ambient)).collect()
    }

    const T: Duration = Duration::from_secs(100);

    /// PR #20 uses this generation for the same independently hot-node
    /// initializer. Its legacy-only writer has no engine records. Keep
    /// that clean-stop format readable without allocating a new generation.
    #[test]
    fn neo_generation_three_clean_stop_remains_readable() {
        let text = include_str!("../tests/neo-generation-three.state");
        let saved = decode(text).unwrap();
        assert!(saved.proven());
        assert_eq!(saved.boottime, T);
        assert!(saved.asahi.is_empty() && saved.unified.is_empty());
        let cfg = Config::parse(J700).unwrap();
        let mut speakers = fresh(&cfg);
        assert!(restore(&mut speakers, &saved, T, &cfg.globals));
        assert_eq!(speakers.len(), 2);
        for (i, s) in speakers.iter().enumerate() {
            assert!(!s.hard);
            assert!((s.t_coil - (125.12345649 + i as f64)).abs() < RECORD_TEMP_EPS);
            assert!((s.t_magnet - (95.12345649 + i as f64)).abs() < RECORD_TEMP_EPS);
            assert!((s.t_peak - (131.12345649 + i as f64)).abs() < RECORD_TEMP_EPS);
        }
        let rewritten = encode(T, T, &speakers, false, true);
        assert!(rewritten.starts_with("speakerguardd-state 3\n"));
        assert!(decode(&rewritten).unwrap().proven());
        assert!(decode(&text.replacen("state 3", "state 2", 1)).is_none());
    }

    /// A genuine in-memory state can round down in the record. Compare
    /// with that unrounded state, rather than treating decoded values as
    /// truth. No tolerance may hide a colder restored node or peak.
    #[test]
    fn persistent_temperatures_cover_record_rounding() {
        let cfg = Config::parse(include_str!("../conf/apple/j316.conf")).unwrap();
        let (tc, tm) = (125.12345649, 95.12345649);
        let mut truth = fresh(&cfg);
        for s in &mut truth { s.resume(tc, tm, 0.0, 0.0); }
        let mut engine_truth = Engine::new(&cfg, true).unwrap();
        for a in engine_truth.asahi.iter_mut().flatten() {
            (a.th.t_coil, a.th.t_magnet, a.coil_hyst, a.magnet_hyst, a.coil_peak, a.magnet_peak, a.hard) = (tc, tm, tc, tm, tc, tm, false);
        }
        for u in engine_truth.unified.iter_mut().flatten() {
            (u.ff.t_coil, u.ff.t_magnet, u.peak, u.hard) = (tc, tm, tc, false);
            if let Some(v) = u.vi.as_mut() { (v.t_coil, v.t_magnet) = (tc, tm); }
        }
        let saved = decode(&encode_with(T, T, &truth, Some(&engine_truth), false, true)).unwrap();
        let saved = Saved { boottime: Duration::ZERO, covered: Duration::ZERO, ..saved };
        let mut bound = fresh(&cfg);
        let mut engine_bound = Engine::new(&cfg, true);
        assert!(restore_interval(&mut bound, &mut engine_bound, &saved, Duration::from_secs(1), Duration::from_secs(1), &cfg.globals));
        for s in &mut truth {
            s.step_pre(s.worst_case_power(cfg.globals.safe_reduction), s.worst_case_power(0.0), 1.0, cfg.globals.t_ambient);
        }
        engine_truth.worst(1.0, &vec![cfg.globals.safe_reduction; cfg.speakers.len()]);
        for (b, a) in bound.iter().zip(&truth) {
            assert!(b.t_coil >= a.t_coil && b.t_magnet >= a.t_magnet, "{}: rounded restore {b:?}, unrounded truth {a:?}", b.spec.name);
        }
        let engine_bound = engine_bound.unwrap();
        for (b, a) in engine_bound.asahi.iter().flatten().zip(engine_truth.asahi.iter().flatten()) {
            assert!(b.th.t_coil >= a.th.t_coil && b.th.t_magnet >= a.th.t_magnet);
            assert!(b.coil_peak >= a.coil_peak && b.magnet_peak >= a.magnet_peak);
            assert!(b.coil_hyst >= a.coil_hyst && b.magnet_hyst >= a.magnet_hyst);
        }
        for (b, a) in engine_bound.unified.iter().flatten().zip(engine_truth.unified.iter().flatten()) {
            assert!(b.ff.t_coil >= a.ff.t_coil && b.ff.t_magnet >= a.ff.t_magnet);
            assert!(b.peak >= a.peak);
            if let (Some(b), Some(a)) = (&b.vi, &a.vi) {
                assert!(b.t_coil >= a.t_coil && b.t_magnet >= a.t_magnet);
            }
        }
    }

    /// Every interior age, not just the endpoints, must fit the restored
    /// state. Include cooling magnets, rising magnets, long ages crossing
    /// the legacy steady-state shortcut, and every shipped thermal model.
    #[test]
    fn interval_restore_bounds_all_nodes_and_governing_peaks() {
        for text in [J700, include_str!("../conf/apple/j316.conf"), include_str!("../conf/apple/j413.conf"), include_str!("../conf/apple/j616.conf")] {
            let cfg = Config::parse(text).unwrap();
            for (tc, tm) in [(66.062964, 64.843034), (125.0, 95.0), (55.0, 125.0), (50.0, 50.0)] {
                let mut sp = fresh(&cfg);
                for s in &mut sp { s.resume(tc, tm, 0.0, 0.0); }
                let mut en = Engine::new(&cfg, true);
                for e in en.iter_mut() {
                    for a in e.asahi.iter_mut().flatten() {
                        (a.th.t_coil, a.th.t_magnet, a.coil_hyst, a.magnet_hyst, a.coil_peak, a.magnet_peak, a.hard) = (tc, tm, tc, tm, tc, tm, false);
                    }
                    for u in e.unified.iter_mut().flatten() {
                        (u.ff.t_coil, u.ff.t_magnet, u.peak, u.hard) = (tc, tm, tc, false);
                        if let Some(v) = u.vi.as_mut() { (v.t_coil, v.t_magnet) = (tc, tm); }
                    }
                }
                let mut saved = decode(&encode_with(T, T, &sp, en.as_ref(), false, true)).unwrap();
                saved.boottime = Duration::ZERO;
                saved.covered = Duration::ZERO;
                for (lo, hi) in [(0.0, 0.1), (3.95, 11.05), (20.0, 60.0), (3599.0, 3601.0), (7200.0, 9000.0)] {
                    let mut bound = fresh(&cfg);
                    let mut engine = Engine::new(&cfg, true);
                    assert!(restore_interval(&mut bound, &mut engine, &saved, Duration::from_secs_f64(lo), Duration::from_secs_f64(hi), &cfg.globals));
                    for i in 0..=40 {
                        let age = Duration::from_secs_f64(lo + (hi - lo) * i as f64 / 40.0);
                        let mut actual = fresh(&cfg);
                        restore(&mut actual, &saved, age, &cfg.globals);
                        for (b, a) in bound.iter().zip(&actual) {
                            assert!(b.t_coil + 1e-6 >= a.t_coil && b.t_magnet + 1e-6 >= a.t_magnet, "{} at {age:?}: {b:?} / {a:?}", b.spec.name);
                            assert!(b.t_peak + 1e-6 >= a.t_peak);
                            // Continuous two-node plant, independently evaluated
                            // from the serialized state, including the residual
                            // that the old hour shortcut discarded.
                            let x = saved.speakers.iter().find(|x| x.name == state_name(&b.spec.name)).unwrap();
                            let p = b.worst_case_power(cfg.globals.safe_reduction);
                            let s = &b.spec;
                            let t = age.as_secs_f64();
                            let m_inf = cfg.globals.t_ambient + p * s.tr_magnet;
                            let c_inf = m_inf + p * s.tr_coil;
                            let k = (x.t_magnet - m_inf) * s.tau_magnet / (s.tau_magnet - s.tau_coil);
                            let coil = c_inf + k * (-t / s.tau_magnet).exp() + (x.t_coil - c_inf - k) * (-t / s.tau_coil).exp();
                            let magnet = m_inf + (x.t_magnet - m_inf) * (-t / s.tau_magnet).exp();
                            assert!(b.t_coil + 1e-6 >= coil && b.t_magnet + 1e-6 >= magnet);
                        }
                        let mut at = Engine::new(&cfg, true);
                        if let (Some(b), Some(a)) = (engine.as_ref(), at.as_mut()) {
                            restore_engine(a, &saved, age, &cfg.globals);
                            for (b, a) in b.asahi.iter().flatten().zip(a.asahi.iter().flatten()) {
                                assert!(b.th.t_coil + 1e-6 >= a.th.t_coil && b.th.t_magnet + 1e-6 >= a.th.t_magnet);
                                assert!(b.coil_peak + 1e-6 >= a.coil_peak && b.magnet_peak + 1e-6 >= a.magnet_peak);
                                assert!(b.coil_hyst + 1e-6 >= a.coil_hyst && b.magnet_hyst + 1e-6 >= a.magnet_hyst);
                                assert!(!a.hard || b.hard);
                            }
                            for (b, a) in b.unified.iter().flatten().zip(a.unified.iter().flatten()) {
                                assert!(b.ff.t_coil + 1e-6 >= a.ff.t_coil && b.ff.t_magnet + 1e-6 >= a.ff.t_magnet);
                                assert!(b.peak + 1e-6 >= a.peak);
                                if let (Some(b), Some(a)) = (&b.vi, &a.vi) {
                                    assert!(b.t_coil + 1e-6 >= a.t_coil && b.t_magnet + 1e-6 >= a.t_magnet);
                                }
                                assert!(!a.hard || b.hard);
                            }
                        }
                    }
                }
            }
        }
    }

    /// A clean marker proves the previous process stopped, not that its
    /// older initializer bounded an independently hot magnet. Do not
    /// promote those temperatures across the model-generation boundary.
    #[test]
    fn an_old_clean_snapshot_cannot_carry_the_old_magnet_bound() {
        let (cfg, mut sp) = hot();
        for s in &mut sp {
            s.t_coil = s.spec.t_limit;
            s.t_magnet = cfg.globals.t_ambient
                + (s.spec.t_limit - cfg.globals.t_ambient)
                    * s.spec.tr_magnet / (s.spec.tr_coil + s.spec.tr_magnet);
            s.hard = true;
            s.t_peak = s.spec.t_limit;
            assert!(s.t_magnet < s.spec.t_limit);
        }
        let current = encode(T, T, &sp, false, true);
        let old = format!("speakerguardd-state 2\n{}", current.split_once('\n').unwrap().1);
        assert!(decode(&old).is_none(), "pre-bound model state was trusted after upgrade");
        assert!(decode(&current).unwrap().released);
    }

    #[test]
    fn round_trip() {
        let (_, mut sp) = hot();
        sp[0].hard = true;
        sp[0].t_peak = 131.0;
        let saved = decode(&encode(T, T, &sp, false, true)).unwrap();
        assert_eq!(saved.boottime, T);
        assert_eq!(saved.covered, T);
        assert!(saved.released && !saved.armed);
        assert_eq!(saved.speakers.len(), 2);
        assert!((saved.speakers[0].t_coil - sp[0].t_coil).abs() < 1e-5);
        assert!(saved.speakers[0].hard && (saved.speakers[0].t_peak - 131.0).abs() < 1e-5);
        assert!((saved.speakers[1].mean_60s - sp[1].mean_power(60.0)).abs() < 1e-5);
        // a held card is never a release
        assert!(!decode(&encode(T, T, &sp, true, true)).unwrap().released);
    }

    #[test]
    fn rejects_damaged_files() {
        let (_, sp) = hot();
        let good = encode(T, T, &sp, false, true);
        assert!(decode("").is_none());
        assert!(decode(&good.replace("speakerguardd-state 3", "speakerguardd-state 4")).is_none());
        let end = good.find("\nend").unwrap() + 4;
        for n in 0..end {
            assert!(decode(&good[..n]).is_none(), "accepted a prefix of {n} bytes");
        }
        let nan = good.replacen(&format!("{:.6}", sp[0].t_coil), "NaN", 1);
        assert!(decode(&nan).is_none());
    }

    /// Only a clean stop's state is resumed: released, and its model
    /// covering up to its write.
    #[test]
    fn only_a_clean_stop_is_resumed() {
        let (cfg, sp) = hot();
        let g = &cfg.globals;
        for (text, want) in [
            (encode(T, T, &sp, false, true), true),
            (encode(T, T, &sp, false, false), false),
            (encode(T, T, &sp, true, false), false),
            (encode(T, T - Duration::from_millis(50), &sp, false, true), false),
        ] {
            let saved = decode(&text).unwrap();
            let mut f = fresh(&cfg);
            assert_eq!(restore(&mut f, &saved, T, g), want, "{text}");
            if want {
                assert!((f[0].t_coil - sp[0].t_coil).abs() < 1e-5 && !f[0].hard);
            } else {
                assert!(f.iter().all(|s| s.hard && s.t_coil >= s.spec.t_limit), "not resumed, so latched at the limit");
            }
        }
    }

    /// A clean stop's state is cooled over the gap at the safe level's
    /// worst case (no daemon held the card), never below its steady state.
    #[test]
    fn a_resumed_state_cools_at_the_safe_level() {
        let (cfg, sp) = hot();
        let g = &cfg.globals;
        let saved = decode(&encode(T, T, &sp, false, true)).unwrap();
        let mut later = fresh(&cfg);
        assert!(restore(&mut later, &saved, T + Duration::from_secs(10), g));
        assert!(later[0].t_coil < sp[0].t_coil - 10.0, "{} vs {}", later[0].t_coil, sp[0].t_coil);
        let floor = {
            let s = &later[0];
            g.t_ambient + s.worst_case_power(g.safe_reduction) * (s.spec.tr_coil + s.spec.tr_magnet)
        };
        let mut cold = fresh(&cfg);
        restore(&mut cold, &saved, T + Duration::from_secs(7200), g);
        assert!((cold[0].t_coil - floor).abs() < 0.5, "{} vs {floor}", cold[0].t_coil);
    }

    #[test]
    fn the_hard_latch_survives_a_restart() {
        let (cfg, mut sp) = hot();
        for s in sp.iter_mut() {
            s.resume(131.9, 100.0, 0.0, 0.0);
            s.hard = true;
            s.t_peak = 135.256;
        }
        let saved = decode(&encode(T, T, &sp, false, true)).unwrap();
        let mut f = fresh(&cfg);
        assert!(restore(&mut f, &saved, T + Duration::from_millis(50), &cfg.globals));
        assert!(f[0].hard && f[0].t_peak >= 131.9);
        assert!(!f[0].clear_hard_if_cool(), "cleared above the ceiling");
    }

    #[test]
    fn an_unknown_speaker_starts_latched() {
        let (cfg, sp) = hot();
        let mut saved = decode(&encode(T, T, &sp, false, true)).unwrap();
        saved.speakers.retain(|s| s.name == "Left");
        let mut f = fresh(&cfg);
        restore(&mut f, &saved, T, &cfg.globals);
        assert!(f[1].t_coil >= 135.0);
        assert!(f[1].hard);
    }

    /// Review r3 T7: a v1 file (0.1.2 to 0.1.5) has no coverage and no clean
    /// stop: it is never resumed.  0.1.6 took its write time as coverage and
    /// restored 100 C where 50 ms of full gain before the write needed 107 C.
    #[test]
    fn a_v1_file_is_never_resumed() {
        let (cfg, mut sp) = hot();
        for s in sp.iter_mut() {
            s.resume(100.0, 100.0, 0.0, 0.0);
        }
        let v1 = encode(T, T, &sp, false, true)
            .replace("speakerguardd-state 3", "speakerguardd-state 1")
            .lines()
            .filter(|l| !l.starts_with("covered_ns") && !l.starts_with("released") && *l != "end")
            .map(|l| if l.starts_with("speaker ") { l.split_whitespace().take(8).collect::<Vec<_>>().join(" ") } else { l.to_string() })
            .collect::<Vec<_>>()
            .join("\n");
        let saved = decode(&v1).expect("v1 decodes");
        assert!(!saved.released);
        let mut f = fresh(&cfg);
        assert!(!restore(&mut f, &saved, T, &cfg.globals));
        assert!(f[0].hard && f[0].t_coil >= 107.251);
    }

    /// Review r3 T5: two saved means do not say when in the minute the
    /// energy came.  A minute of 29 s quiet, 30 s at 10 W before the gain
    /// and 1 s quiet still means 5 W twenty seconds later; 0.1.6's uniform
    /// reconstruction had aged it to 3.3 W.  The restore keeps the energy
    /// as late as it could be.
    #[test]
    fn the_minute_budget_keeps_energy_as_late_as_it_could_be() {
        let cfg = Config::parse(J700).unwrap();
        let mut sp = fresh(&cfg);
        for s in sp.iter_mut() {
            for _ in 0..290 { s.step_at(0.0, 0.1, 50.0, 10.0); }
            for _ in 0..300 { s.step_at(1.0, 0.1, 50.0, 10.0); }
            for _ in 0..10 { s.step_at(0.0, 0.1, 50.0, 10.0); }
        }
        let saved = decode(&encode(T, T, &sp, false, true)).unwrap();
        let mut restored = fresh(&cfg);
        assert!(restore(&mut restored, &saved, T, &cfg.globals));
        for step in 0..600 {
            for s in sp.iter_mut().chain(restored.iter_mut()) {
                s.step(0.0, 0.1, 50.0);
                s.govern(&cfg.globals);
            }
            assert!(restored[0].mean_power(60.0) >= sp[0].mean_power(60.0) - 1e-6, "at {:.1} s: {} vs {} W", (step + 1) as f64 / 10.0, restored[0].mean_power(60.0), sp[0].mean_power(60.0));
            assert!(restored[0].reduction_db >= sp[0].reduction_db - 1e-6);
        }
    }

    /// The uniform case too (the PR #7 review's R5): an 8 W minute keeps
    /// at least its true share at every later moment.
    #[test]
    fn the_minute_budget_survives_the_first_steps_after_a_restore() {
        let (cfg, mut sp) = hot();
        for s in sp.iter_mut() {
            s.resume(60.0, 55.0, 8.0, 8.0);
        }
        let saved = decode(&encode(T, T, &sp, false, true)).unwrap();
        let mut f = fresh(&cfg);
        restore(&mut f, &saved, T, &cfg.globals);
        for step in 1..=600 {
            f[0].step(0.0, 0.1, cfg.globals.t_ambient);
            let elapsed = step as f64 / 10.0;
            let truth = 8.0 * ((60.0 - elapsed).max(0.0)) / 60.0;
            assert!(f[0].mean_power(60.0) >= truth - 1e-6, "at {elapsed} s: {} W vs {truth} W", f[0].mean_power(60.0));
        }
    }

    // ---- the engine's models ----

    const J413: &str = include_str!("../conf/apple/j413.conf");
    const J316: &str = include_str!("../conf/apple/j316.conf");

    /// An engine for `text` (asahi and unified) with every model at `tc`/`tm`.
    fn engine_at(text: &str, tc: f64, tm: f64) -> (Config, Vec<SpeakerState>, Engine) {
        let cfg = Config::parse(&Config::asahi_alone(text)).unwrap();
        let mut sp = fresh(&cfg);
        for s in sp.iter_mut() {
            s.resume(tc, tm, 0.0, 0.0);
        }
        let mut e = Engine::new(&cfg, true).unwrap();
        for a in e.asahi.iter_mut().flatten() {
            a.th.t_coil = tc;
            a.th.t_magnet = tm;
            a.coil_peak = tc;
            a.magnet_peak = tm;
            a.coil_hyst = tc;
            a.magnet_hyst = tm;
        }
        for u in e.unified.iter_mut().flatten() {
            u.ff.t_coil = tc;
            u.ff.t_magnet = tm;
            if let Some(v) = u.vi.as_mut() {
                v.t_coil = tc;
                v.t_magnet = tm;
            }
            u.peak = tc;
        }
        e.govern(0.0, false);
        (cfg, sp, e)
    }

    fn engine(cfg: &Config) -> Engine {
        Engine::new(cfg, true).unwrap()
    }

    /// Review F2 (speakerguardd#6): the asahi governor's state (its peak
    /// hold, gain and latch) did not survive a restart; PR #6 saved only
    /// temperatures.  From a clean stop it all comes back, with names that
    /// have spaces in them (the J316's "Left Woofer 1").
    #[test]
    fn the_engines_models_survive_a_clean_stop() {
        let (cfg, sp, mut e) = engine_at(J316, 128.0, 95.0);
        {
            let a = &mut e.asahi.as_mut().unwrap()[2];
            a.coil_hyst = 133.0;
            a.hard = true;
            a.coil_peak = 151.0;
        }
        e.govern(0.0, false);
        let text = encode_with(T, T, &sp, Some(&e), false, true);
        assert!(text.contains("speaker Left_Woofer_1 ") && text.contains("asahi Left_Woofer_1 ") && text.contains("unified Left_Woofer_1 "));
        let saved = decode(&text).unwrap();
        assert_eq!((saved.speakers.len(), saved.asahi.len(), saved.unified.len()), (6, 6, 6));
        let mut sp2 = fresh(&cfg);
        let mut e2 = engine(&cfg);
        assert!(restore(&mut sp2, &saved, T, &cfg.globals));
        assert!(restore_engine(&mut e2, &saved, T, &cfg.globals));
        e2.govern(0.0, false);
        let (a, b) = (&e.asahi.as_ref().unwrap()[2], &e2.asahi.as_ref().unwrap()[2]);
        assert!((a.th.t_coil - b.th.t_coil).abs() < 1e-5);
        assert!((a.coil_hyst - b.coil_hyst).abs() < 1e-5, "{} vs {}", a.coil_hyst, b.coil_hyst);
        assert!((a.gain - b.gain).abs() < 1e-5 && b.gain < -2.0, "{} vs {}", a.gain, b.gain);
        assert!(b.hard, "the latch was lost");
        assert_eq!(e.reductions(Governor::Asahi), e2.reductions(Governor::Asahi));
        let (u, v) = (&e.unified.as_ref().unwrap()[0], &e2.unified.as_ref().unwrap()[0]);
        assert!((u.ff.t_coil - v.ff.t_coil).abs() < 1e-5 && (u.vi.as_ref().unwrap().t_coil - v.vi.as_ref().unwrap().t_coil).abs() < 1e-5);
        assert!((sp2[0].t_coil - 128.0).abs() < 1e-5);
    }

    /// Review r3 T2, T4 and T7 on the engine: only a clean stop's state is
    /// resumed.  A held snapshot, a running daemon's readout, a state whose
    /// model trails its write and a v1 file all start every engine model at
    /// its limit, latched.  A clean stop's is cooled at the safe level.
    #[test]
    fn the_engine_resumes_only_from_a_clean_stop() {
        let (cfg, sp, e) = engine_at(J413, 119.0, 95.0);
        let g = &cfg.globals;
        for (text, want) in [
            (encode_with(T, T, &sp, Some(&e), false, true), true),
            (encode_with(T, T, &sp, Some(&e), true, false), false),
            (encode_with(T, T, &sp, Some(&e), false, false), false),
            (encode_with(T, T - Duration::from_millis(50), &sp, Some(&e), false, true), false),
        ] {
            let saved = decode(&text).unwrap();
            let mut e2 = engine(&cfg);
            assert_eq!(restore_engine(&mut e2, &saved, T + Duration::from_secs(1), g), want, "{text}");
            if want {
                let a = &e2.asahi.as_ref().unwrap()[0];
                assert!(!a.hard && a.th.t_coil < 119.0 && a.th.t_coil > 100.0, "{}", a.th.t_coil);
            } else {
                assert!(e2.asahi.as_ref().unwrap().iter().all(|a| a.hard && a.th.t_coil >= 150.0));
                assert!(e2.unified.as_ref().unwrap().iter().all(|u| u.hard));
            }
        }
        // a v1 file carries no engine at all and is never resumed
        let v1 = encode(T, T, &sp, false, true)
            .replace("speakerguardd-state 3", "speakerguardd-state 1")
            .lines()
            .filter(|l| !l.starts_with("covered_ns") && !l.starts_with("released") && *l != "end")
            .map(|l| if l.starts_with("speaker ") { l.split_whitespace().take(8).collect::<Vec<_>>().join(" ") } else { l.to_string() })
            .collect::<Vec<_>>()
            .join("\n");
        let mut e3 = engine(&cfg);
        assert!(!restore_engine(&mut e3, &decode(&v1).unwrap(), T, g));
        assert!(e3.asahi.as_ref().unwrap().iter().all(|a| a.hard));
    }

    /// A current-generation clean stop without an engine says nothing
    /// about its models: they start latched while the legacy model resumes.
    /// Older generations are refused separately, even without an engine.
    #[test]
    fn a_model_the_file_does_not_carry_starts_latched() {
        let (cfg, sp, _) = engine_at(J413, 60.0, 55.0);
        let saved = decode(&encode(T, T, &sp, false, true)).unwrap();
        assert!(saved.asahi.is_empty());
        let mut sp2 = fresh(&cfg);
        let mut e2 = engine(&cfg);
        assert!(restore(&mut sp2, &saved, T, &cfg.globals));
        restore_engine(&mut e2, &saved, T, &cfg.globals);
        assert!(e2.asahi.as_ref().unwrap().iter().all(|a| a.hard && a.th.t_coil >= 150.0));
        assert!(e2.unified.as_ref().unwrap().iter().all(|u| u.hard));
        assert!(sp2.iter().all(|s| !s.hard));
        let (_, hot) = hot();
        let a = encode(T, T, &hot, false, true);
        assert_eq!(a, encode_with(T, T, &hot, None, false, true));
        assert!(a.starts_with("speakerguardd-state 3\n") && !a.contains("asahi") && !a.contains("unified"));
    }

    /// Review r3 T5 on the unified budgets: two means do not say when in
    /// the minute the energy came.  29 s quiet, 30 s at 10 W before the
    /// gain and 1 s quiet still means 5 W twenty seconds later; the restore
    /// keeps the energy as late as it could be, never below the history.
    #[test]
    fn the_unified_minute_budget_keeps_energy_as_late_as_it_could_be() {
        let (cfg, sp, mut e) = engine_at(J413, 60.0, 55.0);
        let ta = cfg.globals.t_ambient;
        for u in e.unified.as_mut().unwrap() {
            // 1 W delivered under 10 dB is 10 W before the gain
            for _ in 0..290 {
                u.idle_for_test(0.1, ta);
            }
            for _ in 0..300 {
                u.blocks_for_test(&[(4.0 * 1.0, Some(1.0))], 0.1, 10.0, ta);
            }
            for _ in 0..10 {
                u.idle_for_test(0.1, ta);
            }
        }
        let live = e.unified.as_ref().unwrap()[0].clone();
        let saved = decode(&encode_with(T, T, &sp, Some(&e), false, true)).unwrap();
        let mut e2 = engine(&cfg);
        assert!(restore_engine(&mut e2, &saved, T, &cfg.globals));
        let mut restored = e2.unified.as_ref().unwrap()[0].clone();
        let mut live = live;
        assert!((restored.mean_pre(60.0) - live.mean_pre(60.0)).abs() < 1e-3);
        for step in 0..600 {
            live.idle_for_test(0.1, ta);
            restored.idle_for_test(0.1, ta);
            assert!(restored.mean_pre(60.0) >= live.mean_pre(60.0) - 1e-6, "at {:.1} s: {} vs {} W", (step + 1) as f64 / 10.0, restored.mean_pre(60.0), live.mean_pre(60.0));
        }
    }

    #[test]
    fn rejects_damaged_engine_lines() {
        let (_, sp, e) = engine_at(J413, 60.0, 55.0);
        let good = encode_with(T, T, &sp, Some(&e), false, true);
        assert!(decode(&good).is_some());
        let line = good.lines().find(|l| l.starts_with("asahi ")).unwrap().to_string();
        let mut f: Vec<String> = line.split_whitespace().map(String::from).collect();
        for (i, bad) in [(2, "NaN"), (6, "3.0"), (7, "2"), (8, "inf")] {
            let mut g = f.clone();
            g[i] = bad.to_string();
            assert!(decode(&good.replace(&line, &g.join(" "))).is_none(), "accepted {bad} in field {i}");
        }
        f.pop();
        assert!(decode(&good.replace(&line, &f.join(" "))).is_none());
        let uline = good.lines().find(|l| l.starts_with("unified ")).unwrap().to_string();
        let u: Vec<String> = uline.split_whitespace().map(String::from).collect();
        for (i, bad) in [(2, "NaN"), (4, "-"), (6, "-1"), (8, "inf"), (9, "2")] {
            let mut g = u.clone();
            g[i] = bad.to_string();
            assert!(decode(&good.replace(&uline, &g.join(" "))).is_none(), "accepted {bad} in unified field {i}");
        }
    }

    /// Review r4 U3: the two saved means were spread over whole seconds,
    /// so energy at the newest edge of either window left it too early
    /// (2.6 W and 0 dB where 3.9 W needed 1.4 dB half a second later; 2.48
    /// W where 2.53 W needed 0.06 dB a minute later).  Both witnesses and a
    /// sweep of concentrated histories: the restored budgets are never
    /// below the uninterrupted ones, at any later moment, with new input
    /// near either budget.
    #[test]
    fn restored_budgets_never_expire_energy_early() {
        let cfg = Config::parse(J700).unwrap();
        let g = &cfg.globals;
        let spec = cfg.speakers[0].clone();
        let full = SpeakerState::new(spec.clone(), 50.0).worst_case_power(0.0);
        // (quiet before, burst at full scale before the gain, quiet after)
        let mut cases = vec![(59.8, 0.2, 0.0), (58.5, 0.5, 1.0)];
        for &after in &[0.0, 0.05, 0.3, 0.7, 0.95, 1.0, 1.1, 2.0, 20.0, 57.0, 58.4] {
            for &len in &[0.05, 0.3, 1.0] {
                if after + len <= 60.0 {
                    cases.push((60.0 - after - len, len, after));
                }
            }
        }
        for &(before, len, after) in &cases {
            let mut live = SpeakerState::new(spec.clone(), 50.0);
            live.step_at(0.0, before, 50.0, 20.0);
            live.step_at(live.worst_case_power(20.0), len, 50.0, 20.0);
            if after > 0.0 {
                live.step_at(0.0, after, 50.0, 20.0);
            }
            let saved = decode(&encode(T, T, std::slice::from_ref(&live), false, true)).unwrap();
            let mut restored = vec![SpeakerState::new(spec.clone(), 50.0)];
            assert!(restore(&mut restored, &saved, T, g));
            let mut r = restored.pop().unwrap();
            let mut l = live.clone();
            for step in 0..1240 {
                // 50 ms steps: near the 1 s budget first, then the minute's
                let pre = if step < 30 { 2.7 } else { 2.49 };
                for s in [&mut l, &mut r] {
                    s.step_at(pre / 10.0, 0.05, 50.0, 10.0);
                    s.govern(g);
                }
                let at = (step + 1) as f64 * 0.05;
                for w in [1.0, 60.0] {
                    assert!(r.mean_power(w) >= l.mean_power(w) - 1e-5, "{len} s burst {after} s before the save, {at:.2} s later: {w} s mean {} W restored, {} W live", r.mean_power(w), l.mean_power(w));
                }
                assert!(r.reduction_db >= l.reduction_db - 1e-4, "{len} s burst {after} s before the save, {at:.2} s later: {} dB restored, {} dB live", r.reduction_db, l.reduction_db);
            }
        }
        // nothing before the gain is louder than full scale
        let mut s = SpeakerState::new(spec, 50.0);
        s.step_at(1000.0, 1.0, 50.0, 0.0);
        assert!((s.mean_power(1.0) - full).abs() < 1e-9);
    }
}

// SPDX-License-Identifier: MIT
//! `--shadow-follow`: speakerguardd's models next to a live speakersafetyd,
//! driven by speakersafetyd's own figures.
//!
//! On the machines with V/I sense the sense PCM has one substream, and
//! speakersafetyd holds it.  This mode never opens it, nor the card's
//! controls, nor any other device: it reads text.  speakersafetyd logs, at
//! debug level (`-v`), one line per speaker per period with the measured
//! power it integrated (mean V*I, to 0.01 W) and its coil, magnet and gain,
//! and at info level each group gain it applies.  Read from its journal
//! (`journalctl -u speakersafetyd -o short-monotonic`, a file or `-` for
//! standard input), those periods drive speakerguardd's own models:
//!
//! * the asahi models, sample by sample on the period's power, as the guard
//!   integrates them (with ASAHI_MARGIN), from speakersafetyd's state at the
//!   first period;
//! * the unified models on the same measured power.  speakersafetyd's log
//!   has no voltage, so their feed-forward half sees nothing, and their
//!   reduction is a bound on how much louder unified could play: the
//!   feed-forward model can only tighten it;
//! * the same unified models on the power that would have flowed under
//!   their own gains instead of speakersafetyd's (the counterfactual).
//!
//! Every power is taken at the top of its rounding (+0.005 W).  A gap
//! between periods longer than four of them is time speakersafetyd did not
//! see (it cools its model then, and so do these).
//!
//! Each period gives one JSON line (`--shadow-out`); the end gives a
//! summary: whether speakerguardd's asahi governor ever asked for less than
//! speakersafetyd applied while speakersafetyd's model was hot, how much
//! less unified would have asked for, and how hot the counterfactual coil
//! got.  The exit status is 4 when the asahi governor was ever less
//! protective.

use std::fmt::Write as _;
use std::io::{BufRead, Write};

use crate::config::{Config, Governor};
use crate::engine::Engine;

/// One speaker's line of a period.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line {
    pub coil: f64,
    pub magnet: f64,
    pub power: f64,
    pub gain: f64,
}

/// What a line of speakersafetyd's log says.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    Speaker(String, Line),
    Group(i64, f64),
    Rate(u32),
    Start,
    Skip(f64),
    Panic(String),
}

/// The monotonic time a journal line carries (`-o short-monotonic`), and
/// its message.
fn split(line: &str) -> (Option<f64>, &str) {
    let t = line.trim_start();
    if let Some(rest) = t.strip_prefix('[') {
        if let Some(end) = rest.find(']') {
            if let Ok(v) = rest[..end].trim().parse::<f64>() {
                let after = &rest[end + 1..];
                // "host unit[pid]: message"
                let msg = after.find("]: ").map_or(after, |i| &after[i + 3..]);
                return (Some(v), msg);
            }
        }
    }
    (None, line)
}

/// The number after `key` in `words`.
fn after(words: &[&str], key: &str) -> Option<f64> {
    let i = words.iter().position(|w| *w == key)?;
    words.get(i + 1)?.parse().ok()
}

/// Parse one line of the journal (or of speakersafetyd's own output).
pub fn parse(line: &str) -> (Option<f64>, Option<Event>) {
    let (t, msg) = split(line);
    if let Some(i) = msg.find(": Coil ") {
        let head = &msg[..i];
        let name = head.rsplit(']').next().unwrap_or(head).trim();
        let words: Vec<&str> = msg[i + 1..].split_whitespace().collect();
        if words.contains(&"SKIP:") {
            return (t, None);
        }
        if let (Some(coil), Some(magnet), Some(power), Some(gain)) = (after(&words, "Coil"), after(&words, "Magnet"), after(&words, "Power"), after(&words, "Gain")) {
            return (t, Some(Event::Speaker(name.to_string(), Line { coil, magnet, power, gain })));
        }
        return (t, None);
    }
    if let Some(i) = msg.find("Speaker group ") {
        let words: Vec<&str> = msg[i..].split_whitespace().collect();
        let idx = words.get(2).and_then(|w| w.parse().ok());
        if let Some(idx) = idx {
            if msg.contains("gain nominal") {
                return (t, Some(Event::Group(idx, 0.0)));
            }
            if let Some(g) = after(&words, "to") {
                return (t, Some(Event::Group(idx, g)));
            }
        }
        return (t, None);
    }
    if let Some(i) = msg.find("Sample rate: ") {
        if let Ok(r) = msg[i + 13..].trim().parse() {
            return (t, Some(Event::Rate(r)));
        }
    }
    if msg.contains("Starting up") {
        return (t, Some(Event::Start));
    }
    if let Some(i) = msg.find("Skipping ") {
        let words: Vec<&str> = msg[i..].split_whitespace().collect();
        if let Some(v) = words.get(1).and_then(|w| w.parse().ok()) {
            return (t, Some(Event::Skip(v)));
        }
    }
    if msg.contains("panicked") {
        return (t, Some(Event::Panic(msg.trim().to_string())));
    }
    (t, None)
}

fn norm(name: &str) -> String {
    name.split_whitespace().collect::<Vec<_>>().join(" ").to_lowercase()
}

/// Running statistics of one quantity.
#[derive(Default)]
struct Dist(Vec<f64>);

impl Dist {
    fn q(&self, p: f64) -> f64 {
        if self.0.is_empty() {
            return f64::NAN;
        }
        let mut v = self.0.clone();
        v.sort_by(f64::total_cmp);
        v[((v.len() - 1) as f64 * p).round() as usize]
    }
    fn mean(&self) -> f64 {
        if self.0.is_empty() { f64::NAN } else { self.0.iter().sum::<f64>() / self.0.len() as f64 }
    }
}

/// The follow's outcome.
pub struct Outcome {
    pub text: String,
    pub periods: usize,
    pub less_protective: usize,
}

/// Follow speakersafetyd's log in `input`; one JSON line a period into
/// `out`, if given.
pub fn follow(input: impl BufRead, cfg: &Config, mut out: Option<&mut dyn Write>) -> Result<Outcome, String> {
    let n = cfg.speakers.len();
    let g = &cfg.globals;
    let names: Vec<String> = cfg.speakers.iter().map(|s| norm(&s.name)).collect();
    let mut main = Engine::new(cfg, true).ok_or("no models for this configuration")?;
    let mut cf = Engine::new(cfg, true).ok_or("no models for this configuration")?;
    if main.asahi.is_none() {
        return Err("the follow mode needs a machine with V/I sense".into());
    }
    let full: Vec<f64> = cfg.speakers.iter().map(|s| {
        let v = crate::engine::v_worst(s, g);
        v * v / s.z_nominal.min(s.r_ff)
    }).collect();
    let mut rate: u32 = 48000;
    let mut cur: Vec<Option<Line>> = vec![None; n];
    let mut last_t: Option<f64> = None;
    let mut started = false;
    let (mut periods, mut incomplete, mut gaps, mut restarts) = (0usize, 0usize, 0usize, 0usize);
    let mut gap_s = 0.0;
    let mut panics: Vec<String> = Vec::new();
    let mut unknown: Vec<String> = Vec::new();
    let mut group_lines = 0usize;
    let mut skipped = false;
    let mut limited = 0usize;
    let mut less = 0usize;
    let mut worst_less = 0.0f64;
    let mut first_less: Option<f64> = None;
    let mut louder = Dist::default();
    let mut tighter = 0usize;
    let mut max_ssd = vec![(f64::MIN, f64::MIN); n];
    let mut max_asa = vec![(f64::MIN, f64::MIN); n];
    let mut max_cf = vec![(f64::MIN, f64::MIN); n];
    let mut max_diff = 0.0f64;
    let mut max_rs = 0.0f64;
    let mut cf_latched = false;
    let mut cf_red = vec![0.0; n];
    let mut line_no = 0usize;
    let mut process = |lines: &[Line], t: Option<f64>, rate: u32, skipped: &mut bool, out: &mut Option<&mut dyn Write>| -> Result<(), String> {
        let step = 1.0 / rate as f64;
        let pt = g.period as f64 * step;
        if !started {
            for (k, l) in lines.iter().enumerate() {
                main.follow_start(k, l.coil + 0.005, l.magnet + 0.005);
                cf.follow_start(k, l.coil + 0.005, l.magnet + 0.005);
            }
            started = true;
        } else if let (Some(t), Some(prev)) = (t, last_t) {
            let dt = t - prev;
            if dt > 4.0 * pt || *skipped {
                let idle = (dt - pt).max(0.0);
                main.idle(idle);
                cf.idle(idle);
                gaps += 1;
                gap_s += idle;
            }
        }
        *skipped = false;
        last_t = t.or(last_t.map(|p| p + pt));
        // what speakersafetyd applied: its group's lowest gain
        let applied: Vec<f64> = cfg.speakers.iter().map(|s| {
            cfg.speakers.iter().zip(lines).filter(|(o, _)| o.group == s.group).map(|(_, l)| -l.gain).fold(0.0, f64::max)
        }).collect();
        let power: Vec<f64> = lines.iter().map(|l| l.power.max(0.0) + 0.005).collect();
        let pre: Vec<f64> = (0..n).map(|k| (power[k] * 10f64.powf(applied[k] / 10.0)).min(full[k])).collect();
        main.follow_period(&power, &pre, g.period, step);
        let cf_power: Vec<f64> = (0..n).map(|k| (power[k] * 10f64.powf((applied[k] - cf_red[k]) / 10.0)).min(full[k])).collect();
        cf.follow_period(&cf_power, &pre, g.period, step);
        let asa = main.reductions(Governor::Asahi);
        let uni = main.reductions(Governor::Unified);
        cf_red = cf.reductions(Governor::Unified);
        cf_latched |= cf.hard(Governor::Unified);
        periods += 1;
        let mut any_limited = false;
        let mut row = String::new();
        let _ = write!(row, "{{\"t\":{},\"rate\":{rate},\"spk\":[", t.map_or("null".to_string(), |t| format!("{t:.6}")));
        for k in 0..n {
            let s = &cfg.speakers[k];
            let l = lines[k];
            let a = &main.asahi.as_ref().unwrap()[k];
            let u = &cf.unified.as_ref().unwrap()[k];
            let cfv = u.vi.as_ref().map_or((f64::NAN, f64::NAN), |v| (v.t_coil, v.t_magnet));
            max_ssd[k] = (max_ssd[k].0.max(l.coil), max_ssd[k].1.max(l.magnet));
            max_asa[k] = (max_asa[k].0.max(a.th.t_coil), max_asa[k].1.max(a.th.t_magnet));
            max_cf[k] = (max_cf[k].0.max(cfv.0), max_cf[k].1.max(cfv.1));
            max_diff = max_diff.max((a.th.t_coil - l.coil).abs());
            max_rs = max_rs.max(applied[k]);
            let work = s.t_limit - s.t_headroom;
            let hot = applied[k] > 0.0 || l.coil.max(l.magnet) >= work - g.t_window;
            if applied[k] > 0.0 {
                any_limited = true;
                louder.0.push(applied[k] - uni[k]);
                if uni[k] > applied[k] {
                    tighter += 1;
                }
            }
            if hot && asa[k] < applied[k] - 0.05 {
                less += 1;
                worst_less = worst_less.max(applied[k] - asa[k]);
                first_less.get_or_insert(t.unwrap_or(f64::NAN));
            }
            if k > 0 {
                row.push(',');
            }
            let _ = write!(row, "{{\"n\":\"{}\",\"p\":{:.3},\"ssd\":[{:.2},{:.2},{:.2}],\"asa\":[{:.2},{:.2},{:.2}],\"uni\":{:.2},\"cf\":[{:.2},{:.2},{:.2}]}}", s.name, l.power, l.coil, l.magnet, applied[k], a.th.t_coil, a.th.t_magnet, asa[k], uni[k], cfv.0, cfv.1, cf_red[k]);
        }
        row.push_str("]}\n");
        if any_limited {
            limited += 1;
        }
        if let Some(o) = out.as_mut() {
            o.write_all(row.as_bytes()).map_err(|e| format!("shadow record: {e}"))?;
        }
        Ok(())
    };
    for line in input.lines() {
        let line = line.map_err(|e| format!("input: {e}"))?;
        line_no += 1;
        let (t, ev) = parse(&line);
        match ev {
            Some(Event::Speaker(name, l)) => {
                let Some(k) = names.iter().position(|x| *x == norm(&name)) else {
                    if !unknown.contains(&name) {
                        unknown.push(name);
                    }
                    continue;
                };
                if cur[k].is_some() {
                    // a period with lines missing
                    incomplete += 1;
                    cur = vec![None; n];
                }
                cur[k] = Some(l);
                if cur.iter().all(Option::is_some) {
                    let lines: Vec<Line> = cur.iter().map(|x| x.unwrap()).collect();
                    process(&lines, t, rate, &mut skipped, &mut out)?;
                    cur = vec![None; n];
                }
            }
            Some(Event::Group(..)) => group_lines += 1,
            Some(Event::Rate(r)) if r > 0 => rate = r,
            Some(Event::Start) => {
                restarts += 1;
                skipped = true;
                cur = vec![None; n];
            }
            Some(Event::Skip(_)) => skipped = true,
            Some(Event::Panic(m)) => panics.push(m),
            _ => {}
        }
    }
    drop(process);
    if !unknown.is_empty() {
        return Err(format!("speakers the configuration does not name: {}", unknown.join(", ")));
    }
    let mut o = String::new();
    let _ = writeln!(o, "shadow follow: {line_no} lines, {periods} periods, {incomplete} incomplete, {gaps} gaps ({gap_s:.1} s not seen), {group_lines} group gain lines, {restarts} speakersafetyd starts, {} panics", panics.len());
    if periods == 0 {
        let _ = writeln!(o, "no per-period lines: speakersafetyd logs them only with -v");
    }
    let _ = writeln!(o, "speakersafetyd limited in {limited} periods, at most {max_rs:.2} dB");
    let _ = writeln!(o, "asahi (speakerguardd) less protective than speakersafetyd in {less} speaker-periods{}", if less > 0 { format!(" (worst {worst_less:.2} dB under, first at {:.3} s)", first_less.unwrap_or(f64::NAN)) } else { String::new() });
    let _ = writeln!(o, "asahi model against speakersafetyd's: coil within {max_diff:.2} C");
    let _ = writeln!(o, "unified on measured power (a bound: no feed-forward without voltage): {} limited speaker-periods, louder by mean {:.2} dB, p50 {:.2}, p95 {:.2}, max {:.2}; tighter in {tighter}", louder.0.len(), louder.mean(), louder.q(0.5), louder.q(0.95), louder.q(1.0));
    for (k, s) in cfg.speakers.iter().enumerate() {
        let f = |v: f64| if v > f64::MIN { format!("{v:.1}") } else { "-".into() };
        let _ = writeln!(o, "  {}: hottest coil/magnet speakersafetyd {}/{}, asahi {}/{}, unified counterfactual {}/{} C (limit {:.0}, working {:.0})", s.name, f(max_ssd[k].0), f(max_ssd[k].1), f(max_asa[k].0), f(max_asa[k].1), f(max_cf[k].0), f(max_cf[k].1), s.t_limit, s.t_limit - s.t_headroom);
    }
    let _ = writeln!(o, "unified counterfactual {}", if cf_latched { "latched its hard limit" } else { "never latched" });
    for p in panics.iter().take(5) {
        let _ = writeln!(o, "  speakersafetyd: {p}");
    }
    Ok(Outcome { text: o, periods, less_protective: less })
}

#[cfg(test)]
mod tests {
    use super::*;

    const J616: &str = include_str!("../conf/apple/j616.conf");

    fn cfg() -> Config {
        Config::parse(J616).unwrap()
    }

    #[test]
    fn speakersafetyds_lines_parse() {
        let l = "[ 4321.123456] mbp speakersafetyd[875]: DEBUG [speakersafetyd::types]    Left Tweeter: Coil  52.31 °C Magnet  50.12 °C Power  0.03 W Gain  -1.25 dB";
        assert_eq!(parse(l), (Some(4321.123456), Some(Event::Speaker("Left Tweeter".into(), Line { coil: 52.31, magnet: 50.12, power: 0.03, gain: -1.25 }))));
        let l = "Oct 25 09:19:10 mbp speakersafetyd[875]: INFO  [speakersafetyd] Speaker group 1 gain limited to -1.83 dBFS";
        assert_eq!(parse(l).1, Some(Event::Group(1, -1.83)));
        assert_eq!(parse("[ 1.0] mbp speakersafetyd[875]: INFO  [speakersafetyd] Speaker group 0 gain nominal").1, Some(Event::Group(0, 0.0)));
        assert_eq!(parse("[ 2.0] mbp speakersafetyd[875]: INFO  [speakersafetyd] Sample rate: 48000").1, Some(Event::Rate(48000)));
        assert_eq!(parse("[ 3.0] mbp speakersafetyd[875]: DEBUG [speakersafetyd] Skipping 1.25 seconds").1, Some(Event::Skip(1.25)));
        assert_eq!(parse("[ 3.0] mbp speakersafetyd[875]: DEBUG [speakersafetyd::types] Left Tweeter: SKIP: Coil 52.00 °C Magnet 50.00 °C (1.25 seconds)").1, None);
        assert_eq!(parse("[ 4.0] mbp speakersafetyd[9]: INFO  [speakersafetyd] Starting up").1, Some(Event::Start));
        assert_eq!(parse("random text"), (None, None));
    }

    /// A log of `secs` seconds at a constant power per speaker, written as
    /// speakersafetyd would, with its model run here too (its equations),
    /// and its gains as given by `gains`.
    fn log(c: &Config, secs: f64, power: &[f64], gain: impl Fn(usize, f64) -> f64) -> String {
        let pt = c.globals.period as f64 / 48000.0;
        let mut o = String::from("[ 0.000000] mbp speakersafetyd[1]: INFO  [speakersafetyd] Starting up\n");
        let mut temps: Vec<(f64, f64)> = vec![(60.0, 55.0); c.speakers.len()];
        let mut t = 1.0;
        while t < 1.0 + secs {
            for (k, s) in c.speakers.iter().enumerate() {
                let step = 1.0 / 48000.0;
                let (ac, am) = (step / (s.vi_tau_coil + step), step / (s.vi_tau_magnet + step));
                let (mut tc, mut tm) = temps[k];
                for _ in 0..c.globals.period {
                    tc = (tm + power[k] * s.vi_tr_coil) * ac + tc * (1.0 - ac);
                    tm = (c.globals.t_ambient + power[k] * s.vi_tr_magnet) * am + tm * (1.0 - am);
                }
                temps[k] = (tc, tm);
                let _ = writeln!(o, "[{t:12.6}] mbp speakersafetyd[1]: DEBUG [speakersafetyd::types] {:>15}: Coil {tc:>6.2} °C Magnet {tm:>6.2} °C Power {:>5.2} W Gain {:>6.2} dB", s.name, power[k], gain(k, t));
            }
            t += pt;
        }
        o
    }

    /// The follow drives speakerguardd's models on speakersafetyd's figures,
    /// and its asahi governor asks for at least what speakersafetyd applied
    /// (here a steady 2 W into every speaker, hot enough to limit).
    #[test]
    fn the_follow_tracks_speakersafetyd_and_is_at_least_as_protective() {
        let c = cfg();
        let n = c.speakers.len();
        // speakersafetyd's own ramp on its own model, from the log's temps
        let text = log(&c, 60.0, &vec![2.0; n], |_, _| 0.0);
        let out = follow(text.as_bytes(), &c, None).unwrap();
        assert!(out.periods > 600, "{}", out.text);
        assert!(out.text.contains("coil within"), "{}", out.text);
        // speakersafetyd applied nothing: the asahi models are hot and limit,
        // never less than nothing
        assert_eq!(out.less_protective, 0, "{}", out.text);
        // the same log with speakersafetyd applying 12 dB throughout: the
        // asahi governor, which asks for less early on, is reported
        let text = log(&c, 5.0, &vec![2.0; n], |_, _| -12.0);
        let out = follow(text.as_bytes(), &c, None).unwrap();
        assert!(out.less_protective > 0 && out.text.contains("less protective than speakersafetyd in"), "{}", out.text);
    }

    /// Its record has one line a period, and a log without -v says so.
    #[test]
    fn the_follow_records_each_period_and_needs_the_debug_lines() {
        let c = cfg();
        let n = c.speakers.len();
        let text = log(&c, 2.0, &vec![0.5; n], |_, _| 0.0);
        let mut rec = Vec::new();
        let out = follow(text.as_bytes(), &c, Some(&mut rec)).unwrap();
        let rec = String::from_utf8(rec).unwrap();
        assert_eq!(rec.lines().count(), out.periods);
        assert!(rec.lines().all(|l| l.starts_with("{\"t\":") && l.ends_with("]}")));
        let info = "Oct 25 09:19:10 mbp speakersafetyd[875]: INFO  [speakersafetyd] Speaker group 1 gain limited to -1.83 dBFS\n";
        let out = follow(info.as_bytes(), &c, None).unwrap();
        assert!(out.periods == 0 && out.text.contains("only with -v") && out.text.contains("1 group gain lines"), "{}", out.text);
        // a speaker the configuration does not name is an error
        let bad = "[ 1.0] h speakersafetyd[1]: DEBUG [speakersafetyd::types] Center: Coil 50.00 °C Magnet 50.00 °C Power 0.00 W Gain 0.00 dB\n";
        assert!(follow(bad.as_bytes(), &c, None).is_err());
    }
}

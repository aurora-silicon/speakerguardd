// SPDX-License-Identifier: MIT
//! Continuous acceptance is separate from the accounted/armed properties.
//! These peaks include every integrated interval and both thermal nodes.
use super::vi::{ViFault, ViScenario};
use super::{Scenario, Violation};
use std::cell::{Cell, RefCell};
use std::collections::BTreeMap;
use std::io::Write;

thread_local! {
    static ENABLED: Cell<bool> = const { Cell::new(false) };
    static LAST: RefCell<Option<Case>> = const { RefCell::new(None) };
}
pub(super) fn enabled() -> bool {
    ENABLED.with(Cell::get)
}

#[derive(Clone)]
pub(super) struct Thermal {
    ta: f64,
    rc: f64,
    rm: f64,
    tc: f64,
    tm: f64,
    c: f64,
    m: f64,
    pc: f64,
    pm: f64,
    h: f64,
    ec: f64,
    em: f64,
}
impl Thermal {
    pub(super) fn new(ta: f64, rc: f64, rm: f64, tc: f64, tm: f64) -> Self {
        Self {
            ta,
            rc,
            rm,
            tc,
            tm,
            c: ta,
            m: ta,
            pc: ta,
            pm: ta,
            h: f64::NAN,
            ec: 0.0,
            em: 0.0,
        }
    }
    /// Exact constant-power interval. Coefficients are cached per sample
    /// duration; the original RK4/Euler assertion oracles stay unchanged.
    pub(super) fn observe(&mut self, c: f64, m: f64, p: f64, h: f64) -> (f64, f64) {
        if h != self.h {
            self.h = h;
            self.ec = (-h / self.tc).exp();
            self.em = (-h / self.tm).exp();
        }
        let mi = self.ta + p * self.rm;
        let ci = mi + p * self.rc;
        let (end, mut peak) = if self.tc == self.tm {
            let a = c - ci;
            let b = (m - mi) / self.tc;
            let end = ci + (a + b * h) * self.ec;
            let mut peak = c.max(end);
            if b != 0.0 {
                let t = self.tc - a / b;
                if t > 0.0 && t < h {
                    peak = peak.max(ci + (a + b * t) * (-t / self.tc).exp());
                }
            }
            (end, peak)
        } else {
            let slow = (m - mi) * self.tm / (self.tm - self.tc);
            let fast = c - ci - slow;
            let end = ci + fast * self.ec + slow * self.em;
            let mut peak = c.max(end);
            let d0 = -fast / self.tc - slow / self.tm;
            let dh = -fast / self.tc * self.ec - slow / self.tm * self.em;
            if d0 > 0.0 && dh < 0.0 {
                let t = (-fast * self.tm / (slow * self.tc)).ln() / (1.0 / self.tc - 1.0 / self.tm);
                if t > 0.0 && t < h {
                    peak = peak.max(ci + fast * (-t / self.tc).exp() + slow * (-t / self.tm).exp());
                }
            }
            (end, peak)
        };
        let mend = mi + (m - mi) * self.em;
        peak = peak.max(c).max(end);
        self.pc = self.pc.max(peak);
        self.pm = self.pm.max(m).max(mend);
        (end, mend)
    }
    pub(super) fn advance(&mut self, p: f64, h: f64) {
        let (c, m) = self.observe(self.c, self.m, p, h);
        self.c = c;
        self.m = m;
    }
    pub(super) fn peaks(&self) -> (f64, f64) {
        (self.pc, self.pm)
    }
}
#[derive(Clone)]
pub(super) struct Row {
    pub speaker: String,
    pub model: &'static str,
    pub active: bool,
    pub limit: f64,
    pub coil: f64,
    pub magnet: f64,
}
struct Case {
    kind: &'static str,
    seed: u64,
    input: u64,
    profile: &'static str,
    rows: Vec<Row>,
    error: Option<String>,
    handoffs: u64,
}
fn id(s: &str) -> u64 {
    s.bytes().fold(14695981039346656037, |h, b| {
        (h ^ b as u64).wrapping_mul(1099511628211)
    })
}
pub(super) fn record(
    seed: u64,
    input: String,
    profile: &'static str,
    rows: Vec<Row>,
    error: Option<String>,
    handoffs: u64,
) {
    LAST.with(|x| {
        *x.borrow_mut() = Some(Case {
            kind: "",
            seed,
            input: id(&input),
            profile,
            rows,
            error,
            handoffs,
        })
    });
}
fn capture<T>(kind: &'static str, f: impl FnOnce() -> Result<T, Violation>) -> Case {
    ENABLED.with(|x| x.set(true));
    LAST.with(|x| *x.borrow_mut() = None);
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f));
    ENABLED.with(|x| x.set(false));
    let mut c = LAST
        .with(|x| x.borrow_mut().take())
        .expect("continuous case did not produce its rows");
    c.kind = kind;
    if result.is_err() {
        c.error = Some("case panicked".into());
    }
    c
}
fn long_cases() -> Vec<ViScenario> {
    let mut all = Vec::new();
    let secs = 120.0;
    for j413 in [false, true] {
        for (freq, tw) in [(100.0, false), (3000.0, true), (0.0, false)] {
            for level in [0.2, 0.35, 0.5, 0.7, 1.0] {
                for (guard, dead) in [(false, false), (true, false), (false, true), (true, true)] {
                    all.push(ViScenario::long(j413, freq, tw, level, guard, dead, secs));
                }
            }
        }
    }
    for guard in [true, false] {
        for k in [0.11, 0.2, 0.5] {
            for j413 in [false, true] {
                let mut sc = ViScenario::long(j413, 100.0, false, 0.35, guard, false, secs);
                sc.faults.push(ViFault::ILow {
                    at: 0.0,
                    len: secs + 10.0,
                    k,
                    offset: 0.0,
                });
                all.push(sc);
            }
        }
        for offset in [0.1, 0.3] {
            for (j413, freq, tw, level) in [
                (false, 0.0, false, 0.35),
                (false, 3000.0, true, 0.5),
                (true, 0.0, false, 0.2),
            ] {
                let mut sc = ViScenario::long(j413, freq, tw, level, guard, false, secs);
                sc.faults.push(ViFault::ILow {
                    at: 0.0,
                    len: secs + 10.0,
                    k: 0.0,
                    offset,
                });
                all.push(sc);
            }
        }
    }
    assert_eq!(all.len(), 144);
    all
}
#[derive(Clone)]
enum Job {
    Vi(u64),
    Legacy(u64),
    Long(ViScenario),
}
impl Job {
    fn run(&self) -> Case {
        match self {
            Self::Vi(seed) => capture("vi-random", || {
                super::vi::run(&ViScenario::random(*seed), *seed)
            }),
            Self::Legacy(seed) => capture("legacy-random", || {
                super::run(&Scenario::random(*seed), *seed)
            }),
            Self::Long(sc) => capture("long", || super::vi::run(sc, 700000001)),
        }
    }
}
fn references() -> BTreeMap<(String, u64, u64), (String, bool)> {
    include_str!("continuous-inputs.csv")
        .lines()
        .skip(1)
        .filter(|s| !s.is_empty())
        .map(|line| {
            let p: Vec<_> = line.split(',').collect();
            assert_eq!(p.len(), 7);
            (
                (p[0].into(), p[1].parse().unwrap(), p[2].parse().unwrap()),
                (
                    p[3].into(),
                    p[4].parse::<f64>().unwrap() < -1e-6 || p[5].parse::<f64>().unwrap() < -1e-6,
                ),
            )
        })
        .collect()
}
fn campaign(full: bool) {
    crate::log::set_verbosity(crate::log::Level::Error);
    let refs = references();
    assert_eq!(refs.len(), 30144);
    let mut jobs: Vec<_> = (7312900001..7312920001)
        .map(Job::Vi)
        .chain((7313000001..7313010001).map(Job::Legacy))
        .chain(long_cases().into_iter().map(Job::Long))
        .collect();
    if !full {
        jobs.retain(|j| {
            let (kind, seed, input) = match j {
                Job::Vi(s) => ("vi-random", *s, id(&ViScenario::random(*s).to_rust())),
                Job::Legacy(s) => ("legacy-random", *s, id(&Scenario::random(*s).to_rust())),
                Job::Long(sc) => ("long", 700000001, id(&sc.to_rust())),
            };
            refs.get(&(kind.into(), seed, input))
                .expect("retained input changed")
                .1
        });
        assert_eq!(jobs.len(), 865);
    }
    let lanes = std::thread::available_parallelism()
        .map_or(1, |x| x.get())
        .min(8);
    let cases = std::sync::Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for lane in 0..lanes {
            let jobs = &jobs;
            let cases = &cases;
            scope.spawn(move || {
                let mut v = Vec::new();
                for j in jobs.iter().skip(lane).step_by(lanes) {
                    v.push(j.run());
                }
                cases.lock().unwrap().extend(v);
            });
        }
    });
    let mut cases = cases.into_inner().unwrap();
    cases.sort_by_key(|c| (c.kind, c.seed, c.input));
    assert_eq!(cases.len(), jobs.len());
    let mut failures = Vec::new();
    let mut rows = 0;
    let mut csv = std::env::var_os("PAIR_METRICS")
        .map(|p| std::io::BufWriter::new(std::fs::File::create(p).unwrap()));
    if let Some(f) = csv.as_mut() {
        writeln!(f,"kind,seed,scenario,profile,thermal,active,speaker,properties_pass,handoffs,limit_c,coil_c,magnet_c,error").unwrap();
    }
    for c in &cases {
        let r = refs
            .get(&(c.kind.into(), c.seed, c.input))
            .expect("audit input changed");
        assert_eq!(r.0, c.profile);
        if let Some(e) = &c.error {
            failures.push(format!("{} {}: {e}", c.kind, c.seed));
        }
        for p in &c.rows {
            rows += 1;
            let peak = p.coil.max(p.magnet);
            if p.active && (!peak.is_finite() || peak > p.limit + 1e-6) {
                failures.push(format!(
                    "{} {} {} {} {}: {peak:.12} > {}",
                    c.kind, c.seed, c.profile, p.model, p.speaker, p.limit
                ));
            }
            if let Some(f) = csv.as_mut() {
                writeln!(
                    f,
                    "{},{},{},{},{},{},{},{},{},{:.12},{:.12},{:.12},{}",
                    c.kind,
                    c.seed,
                    c.input,
                    c.profile,
                    p.model,
                    p.active,
                    p.speaker,
                    c.error.is_none(),
                    c.handoffs,
                    p.limit,
                    p.coil,
                    p.magnet,
                    c.error
                        .as_deref()
                        .unwrap_or("")
                        .replace([',', '\n', '\r'], ";")
                )
                .unwrap();
            }
        }
    }
    if let Some(f) = csv.as_mut() {
        f.flush().unwrap();
    }
    if full {
        assert_eq!(rows, 221844);
    }
    eprintln!(
        "CONTINUOUS cases={} rows={} failures={}",
        cases.len(),
        rows,
        failures.len()
    );
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
#[test]
fn retained_continuous_counterexamples_stay_within_limits() {
    campaign(false);
}
#[test]
#[ignore = "complete 30,144-case paired acceptance campaign"]
fn all_30144_continuous_cases_stay_within_limits() {
    campaign(true);
}
#[test]
fn published_copy_gain_witness_stays_within_limits() {
    let c = capture("vi-random", || {
        super::vi::run(&ViScenario::random(7312900005), 7312900005)
    });
    assert!(c.error.is_none(), "{:?}", c.error);
    for p in c.rows {
        assert!(
            !p.active || p.coil.max(p.magnet) <= p.limit + 1e-6,
            "{}: {} / {}",
            p.speaker,
            p.coil.max(p.magnet),
            p.limit
        );
    }
}

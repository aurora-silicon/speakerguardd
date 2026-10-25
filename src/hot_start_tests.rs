// SPDX-License-Identifier: MIT
//! Hot starts use independent coil and magnet bounds. No device is opened.

use super::fake::*;
use super::tests::missing_state;
use super::*;
use crate::engine::Thermal;

fn profiles() -> Vec<(String, Config)> {
    let mut paths: Vec<_> = std::fs::read_dir(concat!(env!("CARGO_MANIFEST_DIR"), "/conf/apple"))
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|x| x == "conf"))
        .collect();
    paths.sort();
    paths
        .into_iter()
        .map(|p| {
            let name = p.file_stem().unwrap().to_str().unwrap().to_owned();
            let c = Config::parse(&std::fs::read_to_string(p).unwrap()).unwrap();
            (name, c)
        })
        .collect()
}

/// Any independent node pair below the hard limit, any part of a queued
/// full-scale drain, and any elapsed time at the kernel safe level must
/// remain below the initialized model. This also checks the legacy budget
/// history and latch, which must survive the change in thermal bounds.
#[test]
fn every_profile_bounds_independent_hot_nodes_and_unknown_history() {
    for (name, cfg) in profiles() {
        let g = &cfg.globals;
        for spec in &cfg.speakers {
            let mut s = SpeakerState::new(spec.clone(), g.t_ambient);
            s.assume_latched(g);
            let full = s.worst_case_power(0.0);
            let safe = s.worst_case_power(g.safe_reduction);
            assert!(s.hard && s.t_peak >= spec.t_limit);
            assert_eq!(s.mean_power(1.0), full);
            assert!((s.mean_power(60.0) - full).abs() < 1e-12);
            let mut bound = Thermal::new(
                spec.tr_coil,
                spec.tau_coil,
                spec.tr_magnet,
                spec.tau_magnet,
                s.t_coil,
            );
            bound.t_magnet = s.t_magnet;
            for c0 in [g.t_ambient, spec.t_limit] {
                for m0 in [g.t_ambient, spec.t_limit] {
                    for drain in [0.0, 0.1, model::DRAIN_S] {
                        for gap in [0.0, 0.05, 2.0, 30.0, 600.0] {
                            let mut truth = bound.clone();
                            truth.t_coil = c0;
                            truth.t_magnet = m0;
                            truth.hold(full, drain, g.t_ambient);
                            truth.hold(safe, gap, g.t_ambient);
                            for after in [0.0, 0.1, 1.0, 10.0, 60.0, 600.0] {
                                let mut actual = truth.clone();
                                let mut upper = bound.clone();
                                actual.hold(safe, after, g.t_ambient);
                                upper.hold(safe, after, g.t_ambient);
                                assert!(upper.t_coil >= actual.t_coil - 1e-9 && upper.t_magnet >= actual.t_magnet - 1e-9,
                                    "{name} {} from {c0}/{m0}, drain {drain}, gap {gap}, after {after}: {upper:?} under {actual:?}", spec.name);
                            }
                        }
                    }
                }
            }
        }
    }
}

/// A status snapshot or a snapshot which did not account up to its write
/// cannot supply a cold magnet. Exercise state::restore as well as start.
#[test]
fn unproven_state_uses_the_same_independent_envelope() {
    for (_, cfg) in profiles() {
        let mut cold: Vec<_> = cfg
            .speakers
            .iter()
            .map(|s| SpeakerState::new(s.clone(), cfg.globals.t_ambient))
            .collect();
        let at = Duration::from_secs(1000);
        let mut saved = state::decode(&state::encode(at, at, &cold, true, false)).unwrap();
        for uncovered in [false, true] {
            saved.armed = !uncovered;
            saved.released = uncovered;
            saved.covered = if uncovered {
                at - Duration::from_secs(1)
            } else {
                at
            };
            assert!(!state::restore(&mut cold, &saved, at, &cfg.globals));
            for s in &cold {
                assert!(s.hard && s.t_coil >= s.spec.t_limit && s.t_magnet >= s.spec.t_limit);
            }
        }
    }
}

struct Tracked {
    hw: FakeHw,
    gains: Vec<(Duration, Vec<f64>)>,
}
impl Tracked {
    fn snap(&mut self) {
        let db: Vec<_> = (0..self.hw.per.len())
            .map(|k| self.hw.effective_of(k))
            .collect();
        if self.gains.last().is_none_or(|(_, last)| *last != db) {
            self.gains.push((self.hw.t, db));
        }
    }
}
impl Hw for Tracked {
    fn now(&self) -> Instant {
        self.hw.now()
    }
    fn boottime(&self) -> Duration {
        self.hw.boottime()
    }
    fn events_available(&self) -> bool {
        self.hw.events_available()
    }
    fn sleep(&mut self, d: Duration) {
        self.snap();
        self.hw.sleep(d);
        self.snap();
    }
    fn take_lock(&mut self) -> io::Result<()> {
        self.snap();
        let out = self.hw.take_lock();
        self.snap();
        out
    }
    fn ping(&mut self) -> io::Result<()> {
        self.snap();
        let out = self.hw.ping();
        self.snap();
        out
    }
    fn set_reduction(&mut self, d: f64) -> io::Result<f64> {
        self.snap();
        let out = self.hw.set_reduction(d);
        self.snap();
        out
    }
    fn set_reductions(&mut self, d: &[f64]) -> io::Result<Vec<f64>> {
        self.snap();
        let out = self.hw.set_reductions(d);
        self.snap();
        out
    }
    fn set_min(&mut self) -> io::Result<()> {
        self.snap();
        let out = self.hw.set_min();
        self.snap();
        out
    }
    fn relinquish(&mut self) -> io::Result<()> {
        self.snap();
        let out = self.hw.relinquish();
        self.snap();
        out
    }
    fn sample_rate(&mut self) -> io::Result<i64> {
        self.snap();
        let out = self.hw.sample_rate();
        self.snap();
        out
    }
    fn open_sense(&mut self, r: u32) -> io::Result<()> {
        self.snap();
        let out = self.hw.open_sense(r);
        self.snap();
        out
    }
    fn close_sense(&mut self) {
        self.snap();
        self.hw.close_sense();
        self.snap();
    }
    fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> {
        self.snap();
        let out = self.hw.read_period(w);
        self.snap();
        out
    }
    fn queued(&mut self) -> io::Result<u64> {
        self.snap();
        let out = self.hw.queued();
        self.snap();
        out
    }
    fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> {
        self.snap();
        let out = self.hw.read_tail(w);
        self.snap();
        out
    }
    fn wait_rate_change(&mut self, d: Duration) -> bool {
        self.snap();
        let out = self.hw.wait_rate_change(d);
        self.snap();
        out
    }
    fn rate_events(&mut self) -> bool {
        self.snap();
        let out = self.hw.rate_events();
        self.snap();
        out
    }
}

/// The review's J700 witness drove full scale with both actual nodes at
/// 135 C and no initial queue. Integrate every applied-control interval,
/// including its interior peak. The old start reached 142.61 / 142.75 C.
/// Run the same legacy FF check on every installed config as well, with
/// voltage bounded by its amplifier and sense rail and load set to r_ff.
#[test]
fn every_legacy_profile_holds_full_scale_from_a_hot_start() {
    for (name, cfg) in profiles().into_iter().filter(|(name, _)| name != "j700" && name != "j413") {
        check_legacy_hot_start(name, cfg);
    }
}

#[test]
fn j413_tweeters_hold_full_scale_for_twenty_minutes() {
    check_legacy_hot_start(
        "j413".to_owned(),
        Config::parse(include_str!("../conf/apple/j413.conf")).unwrap(),
    );
}

#[test]
fn j700_full_scale_after_hot_other_os_session() {
    check_legacy_hot_start(
        "j700".to_owned(),
        Config::parse(include_str!("../conf/apple/j700.conf")).unwrap(),
    );
}

fn check_legacy_hot_start(name: String, mut cfg: Config) {
    crate::log::set_verbosity(crate::log::Level::Error);
    cfg.allow_unqualified = true;
    cfg.globals.governor = Governor::Legacy;
    cfg.globals.guard = None;
    let mut hw = FakeHw::per_speaker(&cfg);
    hw.level = Box::new(|_| 1.0);
    let channels = cfg.globals.channels;
    let specs = cfg.speakers.clone();
    let gg = cfg.globals.clone();
    hw.tweak = Some(Box::new(move |_, words| {
        for frame in words.chunks_exact_mut(channels) {
            for s in &specs {
                let v = crate::engine::v_worst(s, &gg).min(s.vs_scale);
                frame[s.vs_chan] = (frame[s.vs_chan] as f64 * v / s.vs_scale) as i32;
            }
        }
    }));
    let mut hw = Tracked {
        hw,
        gains: Vec::new(),
    };
    let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, None).unwrap();
    while hw.hw.t
        < Duration::from_secs(
            std::env::var("SPEAKERGUARDD_HOT_SECONDS")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(1200),
        )
    {
        g.tick(&mut hw).unwrap();
    }
    hw.snap();
    hw.gains.push((
        hw.hw.t,
        (0..cfg.speakers.len())
            .map(|k| hw.hw.effective_of(k))
            .collect(),
    ));
    for lapse in &hw.hw.lapses {
        let exact = hw.gains.iter().any(|(at, _)| at == lapse);
        let already_safe = hw.gains.iter().rev().find(|(at, _)| at <= lapse)
            .is_some_and(|(_, db)| db.iter().all(|r| *r >= hw.hw.safe));
        assert!(exact || already_safe, "{name}: missed effective lease change at {lapse:?}");
    }
    if let Ok(dir) = std::env::var("SPEAKERGUARDD_HOT_CSV_DIR") {
        use std::io::Write as _;
        let mut file =
            std::fs::File::create(PathBuf::from(dir).join(format!("legacy-{name}.csv"))).unwrap();
        writeln!(
            file,
            "time_ns,{}",
            cfg.speakers
                .iter()
                .map(|s| s.name.clone())
                .collect::<Vec<_>>()
                .join(",")
        )
        .unwrap();
        for (at, db) in &hw.gains {
            writeln!(
                file,
                "{},{}",
                at.as_nanos(),
                db.iter()
                    .map(|d| format!("{d:.17}"))
                    .collect::<Vec<_>>()
                    .join(",")
            )
            .unwrap();
        }
    }
    for (k, s) in cfg.speakers.iter().enumerate() {
        let mut truth = Thermal::new(s.tr_coil, s.tau_coil, s.tr_magnet, s.tau_magnet, s.t_limit);
        let (mut at, mut db, mut peak) = (Duration::ZERO, MUTE_DB, s.t_limit);
        for (to, dbs) in &hw.gains {
            if *to > at {
                let power = crate::engine::v_worst(s, &cfg.globals)
                    .min(s.vs_scale)
                    .powi(2)
                    / s.r_ff
                    * 10f64.powf(-db / 10.0);
                peak = peak.max(
                    truth
                        .hold(power, (*to - at).as_secs_f64(), cfg.globals.t_ambient)
                        .0,
                );
            }
            at = *to;
            db = dbs[k];
        }
        eprintln!(
            "LEGACY_HOT {name} {} limit={} peak={peak:.9} first_ping={:?}",
            s.name,
            s.t_limit,
            hw.hw.pings.first()
        );
        assert!(
            peak <= s.t_limit + 1e-6,
            "{name} {} peaked at {peak} C, limit {} C",
            s.name,
            s.t_limit
        );
    }
    if name == "j700" {
        let first = *hw
            .hw
            .pings
            .first()
            .expect("J700 must recover after cooling");
        assert!(
            first > Duration::from_secs(20) && first < Duration::from_secs(60),
            "{first:?}"
        );
    }
}

#[test]
fn missing_and_corrupt_state_start_with_a_hot_magnet() {
    let cfg = Config::parse(include_str!("../conf/apple/j700.conf")).unwrap();
    let path = missing_state().unwrap();
    for corrupt in [false, true] {
        if corrupt {
            std::fs::write(&path, "incomplete state").unwrap();
        }
        let mut hw = FakeHw::per_speaker(&cfg);
        let g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, Some(path.clone())).unwrap();
        assert!(hw.pings.is_empty());
        for s in &g.speakers {
            assert!(s.hard && s.t_magnet >= s.spec.t_limit && s.t_coil >= s.spec.t_limit);
        }
    }
    std::fs::remove_dir_all(path.parent().unwrap()).unwrap();
}

#[test]
fn random_unknown_histories_stay_below_every_profiles_start() {
    let seed = std::env::var("SPEAKERGUARDD_HOT_SEED")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(7308500001);
    let cases = std::env::var("SPEAKERGUARDD_HOT_CASES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(64);
    let mut rng = crate::harness::Rng::new(seed);
    let mut count = 0;
    for (name, cfg) in profiles() {
        for spec in cfg.speakers {
            let mut s = SpeakerState::new(spec.clone(), cfg.globals.t_ambient);
            s.assume_latched(&cfg.globals);
            let full = s.worst_case_power(0.0);
            let safe = s.worst_case_power(cfg.globals.safe_reduction);
            let mut start = Thermal::new(
                spec.tr_coil,
                spec.tau_coil,
                spec.tr_magnet,
                spec.tau_magnet,
                s.t_coil,
            );
            start.t_magnet = s.t_magnet;
            for case in 0..cases {
                let mut truth = start.clone();
                truth.t_coil = rng.range(cfg.globals.t_ambient, spec.t_limit);
                truth.t_magnet = rng.range(cfg.globals.t_ambient, spec.t_limit);
                truth.hold(
                    full * rng.f(),
                    rng.range(0.0, model::DRAIN_S),
                    cfg.globals.t_ambient,
                );
                truth.hold(
                    safe * rng.f(),
                    rng.range(0.0, 3600.0),
                    cfg.globals.t_ambient,
                );
                let mut upper = start.clone();
                for _ in 0..20 {
                    assert!(
                        upper.t_coil >= truth.t_coil - 1e-9
                            && upper.t_magnet >= truth.t_magnet - 1e-9,
                        "{name} {} seed {seed} case {case}: {upper:?} under {truth:?}",
                        spec.name
                    );
                    let power = safe * rng.f();
                    let dt = rng.range(0.0, 60.0);
                    upper.hold(power, dt, cfg.globals.t_ambient);
                    truth.hold(power, dt, cfg.globals.t_ambient);
                }
                count += 1;
            }
        }
    }
    eprintln!("HOT_RANDOM seed={seed} histories={count} cases_per_speaker={cases}");
}

#[test]
fn legacy_onset_ceiling_uses_its_ff_nodes_instead_of_the_vi_shadow() {
    let mut cfg = Config::parse(include_str!("../conf/apple/j413.conf")).unwrap();
    cfg.allow_unqualified = true;
    cfg.globals.governor = Governor::Legacy;
    cfg.globals.guard = None;
    let mut hw = FakeHw::per_speaker(&cfg);
    let mut g = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, None).unwrap();
    for s in &mut g.speakers {
        s.settle_at(0.0, 0.0, cfg.globals.t_ambient);
        s.t_coil = 130.0;
        s.t_magnet = 130.0;
    }
    for a in g.engine.as_mut().unwrap().asahi.as_mut().unwrap() {
        a.th.t_coil = cfg.globals.t_ambient;
        a.th.t_magnet = cfg.globals.t_ambient;
    }
    let mut warm = vec![0.0; cfg.speakers.len()];
    assert!(g.bound_onset(hw.now(), &mut warm));
    for s in &mut g.speakers {
        s.settle_at(0.0, 0.0, cfg.globals.t_ambient);
    }
    let mut cold = vec![0.0; cfg.speakers.len()];
    assert!(g.bound_onset(hw.now(), &mut cold));
    assert!(warm.iter().zip(cold).all(|(w, c)| *w > c));
    assert!(warm.iter().all(|r| *r > 0.0 && *r < MUTE_DB));
}

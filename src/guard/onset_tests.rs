// SPDX-License-Identifier: MIT
//! Full-scale onset witnesses. No sound card is opened.
use super::fake::FakeHw;
use super::*;
use crate::engine::{v_worst, Thermal};

// Keep every applied gain boundary, including lease expiry while a read
// blocks. The truth is integrated separately from the guard's account.
struct Tracked {
    hw: FakeHw,
    gains: Vec<(Duration, Vec<f64>)>,
    clean_start: bool,
}
impl Tracked {
    fn snap(&mut self) {
        let gains: Vec<_> = (0..self.hw.per.len())
            .map(|k| self.hw.effective_of(k))
            .collect();
        if self.gains.last().is_none_or(|(_, old)| *old != gains) {
            self.gains.push((self.hw.t, gains));
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
        let result = self.hw.take_lock();
        self.snap();
        result
    }
    fn ping(&mut self) -> io::Result<()> {
        self.snap();
        let result = self.hw.ping();
        self.snap();
        result
    }
    fn set_reduction(&mut self, d: f64) -> io::Result<f64> {
        self.snap();
        let result = self.hw.set_reduction(d);
        self.snap();
        result
    }
    fn set_reductions(&mut self, d: &[f64]) -> io::Result<Vec<f64>> {
        self.snap();
        let result = self.hw.set_reductions(d);
        self.snap();
        result
    }
    fn set_min(&mut self) -> io::Result<()> {
        self.snap();
        let result = self.hw.set_min();
        self.snap();
        result
    }
    fn relinquish(&mut self) -> io::Result<()> {
        self.snap();
        let result = self.hw.relinquish();
        self.snap();
        result
    }
    fn sample_rate(&mut self) -> io::Result<i64> {
        self.snap();
        let result = self.hw.sample_rate();
        self.snap();
        result
    }
    fn open_sense(&mut self, r: u32) -> io::Result<()> {
        self.snap();
        let result = self.hw.open_sense(r);
        self.snap();
        result
    }
    fn close_sense(&mut self) {
        self.snap();
        self.hw.close_sense();
        self.snap();
    }
    fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> {
        self.snap();
        let result = self.hw.read_period(w);
        self.snap();
        result
    }
    fn queued(&mut self) -> io::Result<u64> {
        self.snap();
        let result = self.hw.queued();
        self.snap();
        result
    }
    fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> {
        self.snap();
        let result = self.hw.read_tail(w);
        self.snap();
        result
    }
    fn wait_rate_change(&mut self, d: Duration) -> bool {
        self.snap();
        let result = self.hw.wait_rate_change(d);
        self.snap();
        result
    }
    fn rate_events(&mut self) -> bool {
        self.snap();
        let result = self.hw.rate_events();
        self.snap();
        result
    }
}

fn full_scale_run(mut cfg: Config, seconds: u64, clean_start: bool) -> (Config, Tracked) {
    cfg.allow_unqualified = true;
    cfg.allow_unified = true;
    let mut hw = FakeHw::per_speaker(&cfg);
    hw.level = Box::new(|_| 1.0);
    // Preserve the review's static loads. Avoid a current rail in this
    // witness so the onset cannot be confused with current-rail handling.
    let specs = cfg.speakers.clone();
    let globals = cfg.globals.clone();
    hw.tweak = Some(Box::new(move |_, words| {
        for frame in words.chunks_exact_mut(globals.channels) {
            for s in &specs {
                let v = v_worst(s, &globals).min(s.vs_scale);
                let z = s.z_nominal.max(v / s.is_scale * 1.01);
                frame[s.vs_chan] = (frame[s.vs_chan] as f64 * v / s.vs_scale) as i32;
                frame[s.is_chan.unwrap()] =
                    (frame[s.vs_chan] as f64 * s.vs_scale / (z * s.is_scale)) as i32;
            }
        }
    }));
    let mut hw = Tracked {
        hw,
        gains: Vec::new(),
        clean_start,
    };
    let state = if clean_start {
        Some(tests::vi_state(
            &cfg,
            cfg.globals.t_ambient,
            cfg.globals.t_ambient,
        ))
    } else {
        tests::missing_state()
    };
    let mut guard = Guard::start(&mut hw, &cfg, cfg.globals.max_reduction, state).unwrap();
    while hw.hw.t < Duration::from_secs(seconds) {
        guard.tick(&mut hw).unwrap();
    }
    hw.snap();
    hw.gains.push((
        hw.hw.t,
        (0..hw.hw.per.len())
            .map(|k| hw.hw.effective_of(k))
            .collect(),
    ));
    (cfg, hw)
}

fn check_truth(cfg: &Config, hw: &Tracked, name: &str, must_rearm: bool) {
    eprintln!(
        "ONSET_RUN {name} {:?} pings={} first={:?}",
        cfg.globals.governor,
        hw.hw.pings.len(),
        hw.hw.pings.first()
    );
    for (k, s) in cfg.speakers.iter().enumerate() {
        let v = v_worst(s, &cfg.globals).min(s.vs_scale);
        let z = s.z_nominal.max(v / s.is_scale * 1.01);
        // A clean cold state only vouches for a cold speaker. The reviewed
        // unknown-history starts must also bound both hot thermal nodes.
        let starts = if hw.clean_start {
            vec![cfg.globals.t_ambient]
        } else {
            vec![cfg.globals.t_ambient, 135.0_f64.min(s.t_limit), s.t_limit]
        };
        for initial in starts {
            for ff in [false, true] {
                if ff && cfg.globals.governor != Governor::Unified {
                    continue;
                }
                let mut truth = if ff {
                    Thermal::new(s.tr_coil, s.tau_coil, s.tr_magnet, s.tau_magnet, initial)
                } else {
                    Thermal::new(
                        s.vi_tr_coil,
                        s.vi_tau_coil,
                        s.vi_tr_magnet,
                        s.vi_tau_magnet,
                        initial,
                    )
                };
                let mut peak = initial;
                let (mut at, mut gain) = (Duration::ZERO, MUTE_DB);
                for (to, next) in &hw.gains {
                    peak = peak.max(
                        truth
                            .hold(
                                v * v / if ff { s.r_ff } else { z } * 10f64.powf(-gain / 10.0),
                                to.saturating_sub(at).as_secs_f64(),
                                cfg.globals.t_ambient,
                            )
                            .0,
                    );
                    at = *to;
                    gain = next[k];
                }
                eprintln!(
                    "ONSET {name} {:?} {} ff={ff} initial={initial} peak={peak:.6}",
                    cfg.globals.governor, s.name
                );
                assert!(
                    peak <= s.t_limit + 1e-6,
                    "{name} {} from {initial}: {peak} > {}",
                    s.name,
                    s.t_limit
                );
            }
        }
    }
    assert!(
        !must_rearm || !hw.hw.pings.is_empty(),
        "{name}: the bound prevented every re-arm"
    );
}

// The review's J316/J616 Asahi peaks were 194.649 C from ambient and
// 211.830 C from 135 C. Unified + Asahi reached 162.518 / 189.896 C.
// J413 reached 164.894 / 181.487 C with Asahi and 122.809 / 160.826 C
// with the guard. Keep the full 120 s programs and all three start states.
#[test]
fn reviewed_full_scale_onsets_stay_under_the_limit() {
    crate::log::set_verbosity(crate::log::Level::Error);
    for name in ["j316", "j413", "j616"] {
        let text = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join(format!("conf/apple/{name}.conf")),
        )
        .unwrap();
        for gov in [Governor::Asahi, Governor::Unified] {
            let mut cfg = Config::parse(&text).unwrap();
            cfg.globals.governor = gov;
            cfg.globals.guard = (gov == Governor::Unified).then_some(Governor::Asahi);
            let (cfg, hw) = full_scale_run(cfg, 120, false);
            check_truth(&cfg, &hw, name, true);
        }
    }
}

#[test]
fn every_vi_profile_bounds_full_scale_after_mute() {
    crate::log::set_verbosity(crate::log::Level::Error);
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("conf/apple");
    let mut paths: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .map(|p| p.unwrap().path())
        .filter(|p| p.extension().is_some_and(|e| e == "conf"))
        .collect();
    paths.sort();
    for path in paths {
        let cfg = Config::parse(&std::fs::read_to_string(&path).unwrap()).unwrap();
        if !cfg.has_vi() {
            continue;
        }
        let (cfg, hw) = full_scale_run(cfg, 60, true);
        check_truth(
            &cfg,
            &hw,
            path.file_stem().unwrap().to_str().unwrap(),
            false,
        );
    }
}

#[test]
fn a_mute_write_does_not_erase_energy_at_an_earlier_gain() {
    let cfg = tests::j413();
    let mut hw = FakeHw::per_speaker(&cfg);
    let state = tests::vi_state(&cfg, 55.0, 52.0);
    let mut g = Guard::start(
        &mut hw,
        &cfg,
        cfg.globals.max_reduction,
        Some(state.clone()),
    )
    .unwrap();
    let n = cfg.speakers.len();
    let start = hw.now();
    g.gains = vec![Gain {
        at: start,
        lo: vec![0.0; n],
        hi: vec![0.0; n],
        kernel: false,
        cands: vec![vec![(0.0, 0.0)]; n],
    }];
    hw.sleep(Duration::from_millis(100));
    g.written = vec![MUTE_DB; n];
    g.written_hi = g.written.clone();
    g.written_set = vec![vec![(MUTE_DB, MUTE_DB)]; n];
    g.account_to = hw.now();
    g.locked_at = None;
    g.note_written(hw.now());
    let mut per = vec![0.0; n];
    assert!(
        !g.bound_onset(hw.now(), &mut per),
        "old full-scale samples still fit the queue"
    );
    assert!(per.iter().any(|r| *r >= MUTE_DB));
    // A boundary after the old samples have drained may release them.
    hw.sleep(QUEUE_BOUND + Duration::from_millis(1));
    g.account_to = hw.now();
    let mut per = vec![0.0; n];
    assert!(g.bound_onset(hw.now(), &mut per));
    assert!(
        per.iter().any(|r| *r > cfg.globals.max_reduction),
        "a quiet model still needs the full-scale ceiling"
    );
    let _ = std::fs::remove_file(state);
}

#[test]
fn the_onset_ceiling_cannot_bypass_a_governor_handoff() {
    let cfg = tests::j413();
    let mut hw = FakeHw::per_speaker(&cfg);
    hw.level = Box::new(|_| 0.0);
    let state = tests::vi_state(&cfg, 55.0, 52.0);
    let mut g = Guard::start(
        &mut hw,
        &cfg,
        cfg.globals.max_reduction,
        Some(state.clone()),
    )
    .unwrap();
    while hw.t < Duration::from_secs(2) {
        g.tick(&mut hw).unwrap();
    }
    assert!(g.armed());
    assert!(hw.per.iter().any(|r| *r > cfg.globals.max_reduction));
    for a in g.engine.as_mut().unwrap().asahi.as_mut().unwrap() {
        a.gain = -cfg.globals.max_reduction;
    }
    g.decide(&mut hw, false).unwrap();
    assert!(
        !g.armed(),
        "the original governor threshold still hands the card off"
    );
    assert!(hw.per.iter().all(|r| *r >= 0.0));
    let _ = std::fs::remove_file(state);
}

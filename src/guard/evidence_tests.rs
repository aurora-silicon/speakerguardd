// SPDX-License-Identifier: MIT
//! The first start of a boot from evidence (boot.rs): every start path, at
//! the guard, on the Neo's configuration.

use super::fake::*;
use super::tests::{j700, missing_state, run_until, tmp_state};
use super::*;
use crate::boot::{self, Stamp};

const UPTIME: Duration = Duration::from_millis(8500);
const RTC0: u64 = 1_800_000_000;

fn stamp(boot: &str, rtc: u64, synced: bool) -> Stamp {
    Stamp { boot_id: Some(boot.into()), machine_id: Some("m".into()), rtc: Some(rtc), realtime: Some(Duration::from_secs(rtc) + Duration::from_millis(300)), synced }
}

fn secs(s: f64) -> Duration {
    Duration::from_secs_f64(s)
}

/// A run of `play_s` seconds at `level` of full scale, stopped cleanly;
/// the record it left for the next boot, and its coil at the stop.
fn shutdown(level: f64, play_s: f64) -> (Config, PathBuf, f64, f64) {
    let cfg = j700();
    // from a cold clean stop of this same boot (/run), at boot time 1000 s
    let (run, keep) = (tmp_state().unwrap(), missing_state().unwrap());
    let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
    hw.boot = Duration::from_secs(1000);
    hw.stamp = stamp("first", RTC0 - 3600, true);
    hw.level = Box::new(move |_| level);
    let mut g = Guard::start_persist(&mut hw, &cfg, 20.0, Some(run), Some(keep.clone())).unwrap();
    run_until(&mut g, &mut hw, secs(play_s)).unwrap();
    hw.stamp = stamp("first", RTC0, true);
    g.stop(&mut hw).unwrap();
    while !g.released(&hw) {
        g.tick(&mut hw).unwrap();
    }
    g.final_save(&hw).unwrap();
    (cfg, keep, g.speakers[0].t_coil, g.speakers[0].t_magnet)
}

/// The next boot `gap` RTC seconds after the record, 8.5 s up.
fn boot_with(cfg: &Config, keep: Option<&PathBuf>, now: Stamp, chassis: Option<f64>) -> (Guard, FakeHw) {
    let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
    hw.boot = UPTIME;
    hw.stamp = now;
    hw.chassis = chassis;
    let keep = keep.cloned().unwrap_or_else(|| missing_state().unwrap());
    let g = Guard::start_persist(&mut hw, cfg, 20.0, missing_state(), Some(keep.clone())).unwrap();
    assert!(!keep.exists(), "the start did not consume the record");
    (g, hw)
}

fn magnet(g: &Guard) -> f64 {
    g.speakers.iter().map(|s| s.t_magnet).fold(f64::NEG_INFINITY, f64::max)
}

fn coil(g: &Guard) -> f64 {
    g.speakers.iter().map(|s| s.t_coil).fold(f64::NEG_INFINITY, f64::max)
}

/// The no-record start: the protected-past bound (M* = 84.5 C on the Neo)
/// after 8.5 s at the safe level, not latched, and the lock taken within a
/// fraction of a second at a few dB at most.
#[test]
fn no_record_starts_from_the_protected_bound_and_unlocks_at_once() {
    let cfg = j700();
    let (mut g, mut hw) = boot_with(&cfg, None, stamp("second", RTC0, false), None);
    assert!(g.speakers.iter().all(|s| !s.hard), "latched without evidence of a hot coil");
    let m_star = (50.0 + 26.0 / 38.0 * 135.0) / (1.0 + 26.0 / 38.0);
    // M* (plus the margin), cooled for the uptime at the safe level
    assert!(magnet(&g) <= m_star + boot::MAGNET_MARGIN_K && magnet(&g) > m_star - 5.0, "magnet {}", magnet(&g));
    assert!(coil(&g) < 100.0, "coil {}", coil(&g));
    run_until(&mut g, &mut hw, secs(1.0)).unwrap();
    assert!(g.armed(), "the lock was not taken within a second");
    assert!(hw.reduction <= 4.0, "{} dB at the first unlock", hw.reduction);
    // and it releases as the magnet cools
    run_until(&mut g, &mut hw, secs(200.0)).unwrap();
    assert!(hw.reduction <= 1.5, "{} dB after 200 s idle", hw.reduction);
}

/// A clean reboot of a cold machine: the record narrows the magnet to what
/// the half minute since can have added, under the no-record start.
#[test]
fn a_quick_clean_reboot_starts_near_cold() {
    let (cfg, keep, c0, _) = shutdown(0.0, 20.0);
    assert!(c0 < 60.0, "{c0}");
    let (cold, _) = boot_with(&j700(), None, stamp("second", RTC0 + 35, false), None);
    let (mut g, mut hw) = boot_with(&cfg, Some(&keep), stamp("second", RTC0 + 35, false), None);
    assert!(g.speakers.iter().all(|s| !s.hard));
    assert!(magnet(&g) < magnet(&cold) - 15.0, "record magnet {} against none {}", magnet(&g), magnet(&cold));
    // the first sound (-14 dBFS square) a second after the start: what
    // the floor asks for at this start, against the no-record one
    let (mut none, mut hw0) = boot_with(&j700(), None, stamp("second", RTC0 + 35, false), None);
    for (g, hw) in [(&mut g, &mut hw), (&mut none, &mut hw0)] {
        run_until(g, hw, secs(1.0)).unwrap();
        assert!(g.armed());
        hw.level = Box::new(|_| 0.2);
        run_until(g, hw, secs(1.5)).unwrap();
    }
    assert!(hw.reduction <= 2.0 && hw.reduction <= hw0.reduction - 1.0, "{} dB at the first sound after a quick reboot, {} without a record", hw.reduction, hw0.reduction);
}

/// A hot clean stop: the start is never cooler than the stop decayed at
/// the safe level for the uptime alone, and a long gap gives no less than
/// the no-record start (another owner can have had all of it).
#[test]
fn a_hot_record_decays_only_by_the_uptime() {
    let (cfg, keep, c0, m0) = shutdown(0.6, 120.0);
    assert!(m0 > 60.0, "the run was not hot: coil {c0} magnet {m0}");
    let text = std::fs::read_to_string(&keep).unwrap();
    for gap in [30u64, 600, 86_400] {
        std::fs::write(&keep, &text).unwrap();
        let (g, _) = boot_with(&cfg, Some(&keep), stamp("second", RTC0 + gap, false), None);
        // the record's own magnet, after 8.5 s at the safe level at most
        assert!(magnet(&g) >= m0 - 3.0, "gap {gap}: magnet {} against {m0} at the stop", magnet(&g));
        let (none, _) = boot_with(&cfg, None, stamp("second", RTC0 + gap, false), None);
        if gap >= 600 {
            assert!(magnet(&g) >= magnet(&none) - 0.01, "gap {gap}: under the no-record start");
        }
    }
}

/// A checkpoint of a running daemon (the dirty marker) is used only for its
/// magnet: the same as a clean stop's, never a promise that the program
/// stopped with it.
#[test]
fn a_running_checkpoint_is_written_and_used_without_trust() {
    let cfg = j700();
    let (run, keep) = (missing_state().unwrap(), missing_state().unwrap());
    let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
    hw.boot = UPTIME;
    hw.stamp = stamp("first", RTC0, true);
    hw.level = Box::new(|_| 0.6);
    let mut g = Guard::start_persist(&mut hw, &cfg, 20.0, Some(run), Some(keep.clone())).unwrap();
    run_until(&mut g, &mut hw, secs(25.0)).unwrap();
    g.checkpoint.as_mut().unwrap().finish();
    let text = std::fs::read_to_string(&keep).expect("no checkpoint after 25 s");
    let rec = boot::decode(&text).expect("a damaged checkpoint");
    assert!(!rec.clean);
    // a crash now: the next boot, a minute on
    let (g2, _) = boot_with(&cfg, Some(&keep), stamp("second", RTC0 + 60, false), None);
    assert!(g2.speakers.iter().all(|s| !s.hard));
    assert!(magnet(&g2) >= g.speakers.iter().map(|s| s.t_magnet).fold(0.0, f64::max) - 3.0);
    // the same snapshot as a clean stop starts no cooler
    std::fs::write(&keep, boot::encode(&boot::Record { clean: true, ..rec })).unwrap();
    let (g3, _) = boot_with(&cfg, Some(&keep), stamp("second", RTC0 + 60, false), None);
    assert!((magnet(&g3) - magnet(&g2)).abs() < 1e-9);
}

/// Clocks that went back, disagree, or were not synchronized when the
/// record was written give no interval: the magnet bound of an unknown
/// time (no colder than the no-record start).
#[test]
fn untrustworthy_clocks_give_no_interval() {
    let (cfg, keep, _, _) = shutdown(0.0, 20.0);
    let text = std::fs::read_to_string(&keep).unwrap();
    let (none, _) = boot_with(&cfg, None, stamp("second", RTC0 + 35, false), None);
    let mut disagree = stamp("second", RTC0 + 35, false);
    disagree.realtime = Some(Duration::from_secs(RTC0 + 3600));
    let mut no_rtc = stamp("second", RTC0 + 35, false);
    no_rtc.rtc = None;
    for (why, now) in [("went back", stamp("second", RTC0 - 100, false)), ("disagree now", disagree), ("no RTC", no_rtc)] {
        std::fs::write(&keep, &text).unwrap();
        let (g, _) = boot_with(&cfg, Some(&keep), now, None);
        assert!(magnet(&g) >= magnet(&none) - 1.0, "{why}: magnet {} against {}", magnet(&g), magnet(&none));
        assert!(g.speakers.iter().all(|s| !s.hard), "{why}");
    }
    // unsynchronized when written
    let rec = boot::decode(&text).unwrap();
    std::fs::write(&keep, boot::encode(&boot::Record { stamp: Stamp { synced: false, ..rec.stamp.clone() }, ..rec })).unwrap();
    let (g, _) = boot_with(&cfg, Some(&keep), stamp("second", RTC0 + 35, false), None);
    assert!(magnet(&g) >= magnet(&none) - 1.0);
}

/// Damage, another machine or configuration: as no record.  A record of
/// this very boot (the daemon ran this boot, its /run state is gone):
/// latched, as before.
#[test]
fn corrupt_foreign_and_same_boot_records() {
    let (cfg, keep, _, _) = shutdown(0.0, 20.0);
    let text = std::fs::read_to_string(&keep).unwrap();
    let (none, _) = boot_with(&cfg, None, stamp("second", RTC0 + 35, false), None);
    let at = text.find("speaker Left ").unwrap() + "speaker Left ".len();
    let digit = text.as_bytes()[at];
    let mut flipped = text.clone();
    flipped.replace_range(at..at + 1, if digit == b'5' { "4" } else { "5" });
    assert_ne!(flipped, text);
    let mut other = cfg.clone();
    other.globals.t_hysteresis += 1.0;
    let mut foreign = stamp("second", RTC0 + 35, false);
    foreign.machine_id = Some("other".into());
    for (why, body, c, now) in [
        ("flipped", flipped, cfg.clone(), stamp("second", RTC0 + 35, false)),
        ("truncated", text[..text.len() / 2].to_string(), cfg.clone(), stamp("second", RTC0 + 35, false)),
        ("garbage", "\u{0}\u{1}nonsense".to_string(), cfg.clone(), stamp("second", RTC0 + 35, false)),
        ("another configuration", text.clone(), other, stamp("second", RTC0 + 35, false)),
        ("another machine", text.clone(), cfg.clone(), foreign),
    ] {
        std::fs::write(&keep, body).unwrap();
        let (g, _) = boot_with(&c, Some(&keep), now, None);
        assert!((magnet(&g) - magnet(&none)).abs() < 1e-6, "{why}: {} against {}", magnet(&g), magnet(&none));
    }
    std::fs::write(&keep, &text).unwrap();
    let (g, _) = boot_with(&cfg, Some(&keep), stamp("first", RTC0 + 35, false), None);
    assert!(g.speakers.iter().all(|s| s.hard), "a record of this boot was taken as the last boot's");
}

/// A hot chassis raises the start; a cool one changes nothing.
#[test]
fn chassis_sensors_only_raise_the_start() {
    let cfg = j700();
    let (base, _) = boot_with(&cfg, None, stamp("second", RTC0, false), None);
    let (cool, _) = boot_with(&cfg, None, stamp("second", RTC0, false), Some(35.0));
    assert_eq!(magnet(&cool), magnet(&base));
    let (hot, _) = boot_with(&cfg, None, stamp("second", RTC0, false), Some(95.0));
    assert!(magnet(&hot) >= 97.0 && coil(&hot) >= 97.0);
}

/// Without a record path (`--persist ''`) the old latched start remains.
#[test]
fn latched_remains_selectable() {
    let cfg = j700();
    let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
    hw.boot = UPTIME;
    let g = Guard::start(&mut hw, &cfg, 20.0, missing_state()).unwrap();
    assert!(g.speakers.iter().all(|s| s.hard));
}

/// From the no-record start, ten minutes of full-scale square wave: the
/// model never reaches the limit and the guard keeps the card.  Truth is
/// stepped beside it from the worst protected past (coil at the limit, the
/// magnet at M*, 8.5 s at the safe level) with the gain the card applied:
/// it stays under the hard limit as well.
#[test]
fn full_scale_from_the_evidence_start_stays_under_the_limit() {
    let cfg = j700();
    let (mut g, mut hw) = boot_with(&cfg, None, stamp("second", RTC0, false), None);
    hw.level = Box::new(|_| 1.0);
    let ta = cfg.globals.t_ambient;
    let mut truth: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), cfg.globals.t_ambient)).collect();
    let m_star = (ta + 26.0 / 38.0 * 135.0) / (1.0 + 26.0 / 38.0);
    for t in truth.iter_mut() {
        t.t_coil = t.spec.t_limit - 1e-6;
        t.t_magnet = m_star;
        let p = t.worst_case_power(cfg.globals.safe_reduction);
        t.step(p, UPTIME.as_secs_f64(), ta);
        t.hard = false;
    }
    let mut last = hw.t;
    let mut hottest: f64 = 0.0;
    while hw.t < secs(600.0) {
        let applied = if g.armed() { hw.reduction } else { cfg.globals.safe_reduction.max(hw.reduction) };
        g.tick(&mut hw).unwrap();
        let dt = (hw.t - last).as_secs_f64();
        last = hw.t;
        for (t, s) in truth.iter_mut().zip(&g.speakers) {
            let p = t.worst_case_power(applied);
            t.step(p, dt, ta);
            assert!(s.t_peak < s.spec.t_limit, "the model reached its limit at {:?}", hw.t);
            hottest = hottest.max(t.t_peak);
        }
    }
    assert!(hottest < cfg.speakers[0].t_limit, "the worst past's coil reached {hottest}");
}

/// The attenuation against time after boot, for the report (run with
/// --ignored --nocapture): no record, a quick clean reboot, and the latched
/// start it replaces; idle, then pink-like program at -10 dBFS RMS.
#[test]
#[ignore]
fn print_attenuation_after_boot() {
    let (cfg, keep, _, _) = shutdown(0.0, 20.0);
    for (name, c, k, gap) in [("no record", j700(), None, 35u64), ("quick reboot", cfg.clone(), Some(&keep), 35), ("latched (old)", j700(), None, 0)] {
        let (mut g, mut hw) = if gap == 0 {
            let mut hw = FakeHw::new(c.globals.channels, c.globals.period);
            hw.boot = UPTIME;
            (Guard::start(&mut hw, &c, 20.0, missing_state()).unwrap(), hw)
        } else {
            boot_with(&c, k, stamp("second", RTC0 + gap, false), None)
        };
        hw.level = Box::new(|_| 0.316);
        let mut line = format!("{name:>14}:");
        for t in [0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 60.0, 120.0, 240.0, 480.0] {
            run_until(&mut g, &mut hw, secs(t)).unwrap();
            let db = if g.armed() { hw.reduction } else { 20.0 };
            line += &format!(" {t}s={db:.1}");
        }
        eprintln!("{line}");
    }
}

/// The attenuation the guard holds at idle from a resumed (c, m), for the
/// report (--ignored --nocapture).
#[test]
#[ignore]
fn print_idle_attenuation_by_temperature() {
    let cfg = j700();
    for &(c, m) in &[(50.0, 50.0), (60.0, 55.0), (70.0, 60.0), (80.0, 70.0), (85.0, 75.0), (92.0, 83.0), (100.0, 85.0)] {
        let path = missing_state().unwrap();
        let mut sp: Vec<SpeakerState> = cfg.speakers.iter().map(|s| SpeakerState::new(s.clone(), 50.0)).collect();
        for s in sp.iter_mut() { s.t_coil = c; s.t_magnet = m; s.t_peak = c; }
        let boot = Duration::from_secs(1000);
        std::fs::write(&path, state::encode(boot, boot, &sp, false, true)).unwrap();
        let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
        hw.boot = boot;
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(path)).unwrap();
        run_until(&mut g, &mut hw, secs(1.0)).unwrap();
        let idle = hw.reduction;
        hw.level = Box::new(|_| 0.2);
        run_until(&mut g, &mut hw, secs(1.5)).unwrap();
        eprintln!("coil {c} magnet {m}: idle {idle:.1} dB, -14 dBFS square {:.1} dB, floor {:.2} dB", hw.reduction, g.floor_db(0.0));
    }
}

// ---- C1 of the 2026-10-31 safety review: a missing /run state does not
// prove that no daemon unlocked the card this boot.

/// A start of this boot with its /run state at `run` (and the marker next
/// to it), the record at `keep`, `setup` applied to the card first.
fn start_at(cfg: &Config, run: &std::path::Path, keep: &std::path::Path, now: Stamp, setup: impl FnOnce(&mut FakeHw)) -> (Guard, FakeHw) {
    let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
    hw.boot = UPTIME;
    hw.stamp = now;
    setup(&mut hw);
    let g = Guard::start_persist(&mut hw, cfg, 20.0, Some(run.to_path_buf()), Some(keep.to_path_buf())).unwrap();
    (g, hw)
}

fn marker_of(run: &std::path::Path) -> PathBuf {
    boot::marker_path(run)
}

/// Truth from `crashed` (coil, magnet per speaker), 1 s at the safe level,
/// then `g` driving full scale for 120 s with the gains it lets through;
/// the hottest the true coil gets.
fn truth_after_restart(cfg: &Config, g: &mut Guard, hw: &mut FakeHw, crashed: &[(f64, f64)]) -> f64 {
    let ta = cfg.globals.t_ambient;
    let mut truth: Vec<SpeakerState> = g.speakers.iter().zip(crashed).map(|(s, &(c, m))| {
        let mut t = SpeakerState::new(s.spec.clone(), ta);
        t.t_coil = c;
        t.t_magnet = m;
        let p = t.worst_case_power(cfg.globals.safe_reduction);
        t.step(p, 1.0, ta);
        t.hard = false;
        t
    }).collect();
    hw.level = Box::new(|_| 1.0);
    let mut last = hw.t;
    let mut hottest: f64 = 0.0;
    while hw.t < secs(120.0) {
        let applied = if g.armed() { hw.reduction } else { cfg.globals.safe_reduction.max(hw.reduction) };
        g.tick(hw).unwrap();
        let dt = (hw.t - last).as_secs_f64();
        last = hw.t;
        for t in truth.iter_mut() {
            let p = t.worst_case_power(applied);
            t.step(p, dt, ta);
            hottest = hottest.max(t.t_peak.max(t.t_coil));
        }
    }
    hottest
}

/// SAFETY REVIEW counterexample (C1), kept as a regression test.  The
/// reviewer's premise (both writes fail) can no longer unlock at all, see
/// the next test; here the first run unlocks normally, plays full scale
/// for a minute and dies without a clean stop, and its /run state is then
/// lost and the record under /var/lib never written (its temporary name
/// is a directory, standing in for ENOSPC).  The restart a second later in
/// the same boot finds neither file, but finds the marker: latched, never
/// cooler than the crashed model, and truth stays under the hard limit.
#[test]
fn review_crash_restart_without_run_state_is_not_a_first_start() {
    let cfg = j700();
    let (run, keep) = (missing_state().unwrap(), missing_state().unwrap());
    std::fs::create_dir_all(keep.with_extension("tmp")).unwrap();
    let mut hw = FakeHw::new(cfg.globals.channels, cfg.globals.period);
    hw.boot = UPTIME;
    hw.stamp = stamp("first", RTC0, true);
    hw.level = Box::new(|_| 1.0);
    let mut g = Guard::start_persist(&mut hw, &cfg, 20.0, Some(run.clone()), Some(keep.clone())).unwrap();
    run_until(&mut g, &mut hw, secs(60.0)).unwrap();
    assert!(g.armed(), "the first run never unlocked");
    let crashed: Vec<(f64, f64)> = g.speakers.iter().map(|s| (s.t_coil, s.t_magnet)).collect();
    let boot_at_crash = hw.boot + hw.t;
    // a crash: no stop, no final save; the checkpointer thread just ends
    drop(g);
    std::fs::remove_file(&run).unwrap();
    assert!(!run.exists() && !keep.exists(), "the test premise: neither file is there");
    assert!(marker_of(&run).exists(), "no marker after an unlock");
    let mut hw2 = FakeHw::new(cfg.globals.channels, cfg.globals.period);
    hw2.boot = boot_at_crash + secs(1.0);
    hw2.stamp = stamp("first", RTC0 + 70, true);
    let mut g2 = Guard::start_persist(&mut hw2, &cfg, 20.0, Some(run), Some(keep)).unwrap();
    let start: Vec<(f64, f64)> = g2.speakers.iter().map(|s| (s.t_coil, s.t_magnet)).collect();
    eprintln!("crashed model {crashed:?}; restart starts at {start:?}");
    assert!(g2.speakers.iter().all(|s| s.hard), "the restart did not start latched");
    assert!(start.iter().zip(&crashed).all(|(s, c)| s.0 >= c.0 - 1.0 && s.1 >= c.1 - 1.0), "the restart started cooler than the crashed model: {start:?} against {crashed:?}");
    let hottest = truth_after_restart(&cfg, &mut g2, &mut hw2, &crashed);
    let limit = cfg.speakers[0].t_limit;
    eprintln!("truth hottest {hottest} C over 120 s of full scale from the restart");
    assert!(hottest < limit, "truth reached {hottest} C against the {limit} C hard limit");
}

/// The reviewer's literal premise: the /run state and the record both fail
/// to write.  Fail closed: the run never unlocks (no removal of the /run
/// state counts as invalidating it), and its restart starts latched.
#[test]
fn a_run_state_that_cannot_be_written_keeps_the_card_locked() {
    let cfg = j700();
    let (run, keep) = (missing_state().unwrap(), missing_state().unwrap());
    std::fs::create_dir_all(run.with_extension("tmp")).unwrap();
    std::fs::create_dir_all(keep.with_extension("tmp")).unwrap();
    let (mut g, mut hw) = start_at(&cfg, &run, &keep, stamp("first", RTC0, true), |hw| hw.level = Box::new(|_| 1.0));
    run_until(&mut g, &mut hw, secs(60.0)).unwrap();
    assert!(!g.armed(), "unlocked without a /run state");
    assert!(hw.reduction >= cfg.globals.safe_reduction - 1e-9, "{} dB", hw.reduction);
    drop(g);
    assert!(!run.exists() && !keep.exists());
    let (g2, _) = start_at(&cfg, &run, &keep, stamp("first", RTC0 + 70, true), |_| {});
    assert!(g2.speakers.iter().all(|s| s.hard), "the restart did not start latched");
}

/// The marker is written (with the boot id) before the first unlock, and a
/// clean stop leaves it in place.
#[test]
fn the_marker_is_written_before_the_unlock_and_kept() {
    let cfg = j700();
    let (run, keep) = (missing_state().unwrap(), missing_state().unwrap());
    let (mut g, mut hw) = start_at(&cfg, &run, &keep, stamp("second", RTC0, false), |_| {});
    assert!(!g.speakers.iter().any(|s| s.hard), "not an evidence start");
    assert_eq!(std::fs::read_to_string(marker_of(&run)).unwrap().trim(), "second", "no marker before the unlock");
    run_until(&mut g, &mut hw, secs(1.0)).unwrap();
    assert!(g.armed());
    g.stop(&mut hw).unwrap();
    while !g.released(&hw) {
        g.tick(&mut hw).unwrap();
    }
    g.final_save(&hw).unwrap();
    assert!(marker_of(&run).exists(), "the marker was removed");
}

/// A marker that cannot be written: the lock is never taken.
#[test]
fn a_marker_that_cannot_be_written_keeps_the_card_locked() {
    let cfg = j700();
    let (run, keep) = (missing_state().unwrap(), missing_state().unwrap());
    std::fs::create_dir_all(marker_of(&run).with_extension("tmp")).unwrap();
    let (mut g, mut hw) = start_at(&cfg, &run, &keep, stamp("second", RTC0, false), |_| {});
    run_until(&mut g, &mut hw, secs(5.0)).unwrap();
    assert!(!g.armed(), "unlocked without the marker");
}

/// This boot's marker, or one that cannot be read, refuses the evidence
/// start; another boot's (a /run that is not a tmpfs) does not.
#[test]
fn the_marker_refuses_the_evidence_start() {
    let cfg = j700();
    for (why, body, latched) in [("this boot's", Some("second\n"), true), ("this boot's, dashes and case", Some("SE-COND\n"), true), ("unknown boot", Some("unknown\n"), true), ("empty", Some(""), true), ("unreadable", None, true), ("another boot's", Some("first\n"), false)] {
        let (run, keep) = (missing_state().unwrap(), missing_state().unwrap());
        match body {
            Some(b) => std::fs::write(marker_of(&run), b).unwrap(),
            None => std::fs::create_dir_all(marker_of(&run)).unwrap(),
        }
        let (g, _) = start_at(&cfg, &run, &keep, stamp("second", RTC0, false), |_| {});
        assert_eq!(g.speakers.iter().all(|s| s.hard), latched, "{why}");
    }
    // no boot id now: any marker counts
    let (run, keep) = (missing_state().unwrap(), missing_state().unwrap());
    std::fs::write(marker_of(&run), "first\n").unwrap();
    let mut now = stamp("x", RTC0, false);
    now.boot_id = None;
    let (g, _) = start_at(&cfg, &run, &keep, now, |_| {});
    assert!(g.speakers.iter().all(|s| s.hard), "a marker was set aside without a boot id");
}

/// speakersafetyd's flag or a kernel override of the safe level refuses
/// the evidence start.
#[test]
fn a_protection_override_refuses_the_evidence_start() {
    let cfg = j700();
    let (run, keep) = (missing_state().unwrap(), missing_state().unwrap());
    let (g, _) = start_at(&cfg, &run, &keep, stamp("second", RTC0, false), |hw| hw.overridden = Some("please_blow_up_my_speakers is 1".into()));
    assert!(g.speakers.iter().all(|s| s.hard), "an override did not latch the start");
}

/// The override check itself, on files: a missing parameter or flag is no
/// override; 0/N is none; anything else, or an unreadable one, is one.
#[test]
fn protection_override_reads_flags_and_parameters() {
    let dir = missing_state().unwrap().parent().unwrap().to_path_buf();
    let p = dir.join("param");
    let flag = dir.join("flag");
    let (ps, fs) = (p.to_str().unwrap(), flag.to_str().unwrap());
    assert_eq!(boot::protection_override(&[fs], &[ps]), None);
    for (v, set) in [("0\n", false), ("N\n", false), ("1\n", true), ("Y\n", true), ("-1\n", true)] {
        std::fs::write(&p, v).unwrap();
        assert_eq!(boot::protection_override(&[fs], &[ps]).is_some(), set, "{v:?}");
    }
    std::fs::remove_file(&p).unwrap();
    std::fs::create_dir_all(&p).unwrap();
    assert!(boot::protection_override(&[fs], &[ps]).is_some(), "an unreadable parameter");
    std::fs::remove_dir(&p).unwrap();
    std::fs::write(&flag, "").unwrap();
    assert!(boot::protection_override(&[fs], &[ps]).is_some(), "speakersafetyd's flag");
}

/// A chassis sensor over the model's ambient is the bound's ambient, not
/// only a floor: the magnet starts at least at M* for that ambient
/// (cooled for the uptime), above the floor alone.
#[test]
fn a_hot_chassis_raises_the_ambient_of_the_bound() {
    let cfg = j700();
    let (base, _) = boot_with(&cfg, None, stamp("second", RTC0, false), None);
    let (warm, _) = boot_with(&cfg, None, stamp("second", RTC0, false), Some(65.0));
    let r = 26.0 / 38.0;
    let m_star = |ta: f64| (ta + r * 135.0) / (1.0 + r);
    // the floor alone (67 C) is under the 50 C ambient's bound
    assert!(magnet(&base) > 67.0 + 10.0);
    assert!(magnet(&warm) >= magnet(&base) + (m_star(65.0) - m_star(50.0)) - 0.5, "{} against {}", magnet(&warm), magnet(&base));
}

/// The marker sits outside the /run state's directory (the unit's
/// RuntimeDirectory): `systemctl clean --what=runtime` or removing the
/// directory after an unlock leaves it, and the next start of the boot is
/// latched (non-blocking note 1 of the round 2 safety review).
#[test]
fn the_marker_survives_the_runtime_directory() {
    assert_eq!(boot::marker_path(std::path::Path::new("/run/speakerguardd/state")), std::path::Path::new("/run/speakerguardd.unlocked"));
    assert_eq!(boot::marker_path(std::path::Path::new("state")), std::path::Path::new("unlocked"));
    let cfg = j700();
    let (run, keep) = (missing_state().unwrap(), missing_state().unwrap());
    let (mut g, mut hw) = start_at(&cfg, &run, &keep, stamp("second", RTC0, false), |_| {});
    run_until(&mut g, &mut hw, secs(2.0)).unwrap();
    assert!(g.armed());
    drop(g);
    std::fs::remove_dir_all(run.parent().unwrap()).unwrap();
    assert!(marker_of(&run).exists(), "removing the runtime directory took the marker");
    let (g2, _) = start_at(&cfg, &run, &keep, stamp("second", RTC0 + 10, false), |_| {});
    assert!(g2.speakers.iter().all(|s| s.hard), "the start after the runtime directory was removed was not latched");
    let _ = std::fs::remove_file(marker_of(&run));
}

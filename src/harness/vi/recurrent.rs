// SPDX-License-Identifier: MIT
//! Recovered low-line regressions. Inputs are synthetic, with no recordings.
use super::*;
use std::cell::RefCell;

#[derive(Default)]
struct Program {
    frequencies: Vec<(f64, f64)>,
    mute_tweeters: Option<f64>,
    prefix: Vec<Vec<f64>>,
    record_prefix: bool,
}
thread_local! { static PROGRAM: RefCell<Program> = RefCell::new(Program::default()); }

pub(super) fn frequency(at: f64, default: f64) -> f64 {
    PROGRAM.with(|p| {
        p.borrow()
            .frequencies
            .iter()
            .rev()
            .find(|x| at >= x.0)
            .map_or(default, |x| x.1)
    })
}
pub(super) fn level(tweeter: bool, at: f64, default: f64) -> f64 {
    PROGRAM.with(|p| {
        if tweeter && p.borrow().mute_tweeters.is_some_and(|t| at >= t) {
            0.0
        } else {
            default
        }
    })
}
struct Reset;
impl Drop for Reset {
    fn drop(&mut self) {
        PROGRAM.with(|p| *p.borrow_mut() = Program::default());
    }
}
fn program(frequencies: Vec<(f64, f64)>, mute_tweeters: Option<f64>) -> Reset {
    PROGRAM.with(|p| {
        *p.borrow_mut() = Program {
            frequencies,
            mute_tweeters,
            ..Program::default()
        }
    });
    Reset
}

/// A low current reading must not exempt this assertion, nor may a
/// hand-off, a hot sibling or an accounting boundary hide a crossing.
pub(super) fn assert_true_vi_limit(sc: &ViScenario, out: &Outcome) {
    for (k, s) in vi_cfg(sc).speakers.iter().enumerate() {
        assert!(
            out.true_vi_peaks[k].is_finite() && out.true_vi_peaks[k] <= s.t_limit,
            "{} true V/I peak {:.9} C exceeds {} C",
            s.name,
            out.true_vi_peaks[k],
            s.t_limit
        );
    }
}

pub(super) fn observe(world: &ViWorld<'_>) {
    if world.oracle.n % (world.rate as u64 / 100) != 0 {
        return;
    }
    let at = world.oracle.n as f64 / world.rate;
    if at >= 40.0 {
        return;
    }
    PROGRAM.with(|p| {
        let mut p = p.borrow_mut();
        if !p.record_prefix {
            return;
        }
        let mut row = vec![at];
        for k in 0..world.cfg.speakers.len() {
            let vi = world.oracle.now[k][VI];
            row.extend([vi.0, vi.1, world.gain_at(k, at)]);
        }
        p.prefix.push(row);
    });
}

fn witness() -> ViScenario {
    let mut sc = ViScenario::long(true, 180.0, false, 0.3930526909033998, false, false, 360.0);
    sc.seed = 680000668;
    sc.loads.fill((1.4336625005819963, -0.44312087250893706));
    sc.streams[0].levels = vec![
        (0.05, 0.3930526909033998),
        (18.766523648600923, 0.3367213347629122),
        (24.511524363702225, 0.3930526909033998),
    ];
    let from = sc.streams[0].levels[2].1;
    for n in 1..=20 {
        sc.streams[0].levels.push((
            60.0 + n as f64 * 0.25,
            from + (0.7 - from) * n as f64 / 20.0,
        ));
    }
    sc.faults = vec![
        ViFault::ILow {
            at: 0.0,
            len: 18.766523648600923,
            k: 0.7510968424254598,
            offset: 0.0,
        },
        ViFault::ILow {
            at: 24.511524363702225,
            len: 370.0,
            k: 0.6455407021983011,
            offset: 0.0,
        },
        ViFault::Base(Fault::SenseLoss {
            at: 20.132470954032225,
            len: 0.25,
        }),
    ];
    sc
}

#[test]
fn recurrent_line_late_mute_stays_below_the_true_vi_limit() {
    crate::log::set_verbosity(crate::log::Level::Error);
    for guard in [false, true] {
        let _reset = program(vec![(0.0, 180.0), (26.511524363702225, 1000.0)], Some(40.0));
        PROGRAM.with(|p| p.borrow_mut().record_prefix = true);
        let mut sc = witness();
        sc.guard = guard;
        let out = run(&sc, sc.seed).unwrap();
        eprintln!(
            "RECURRENT guard={guard} peaks={:?} handoffs={} kernel_s={}",
            out.true_vi_peaks, out.handoffs, out.kernel_s
        );
        assert_true_vi_limit(&sc, &out);
        let late_prefix = PROGRAM.with(|p| p.borrow().prefix.clone());
        assert_eq!(late_prefix.len(), 3999);
        // Run the unmodified case 667 prelude separately. All oracle and
        // gain rows before the late mute must match, not just its maximum.
        let (mut original, frequencies) = adversarial(667);
        original.guard = guard;
        original.horizon = 40.0;
        let _original = program(frequencies, None);
        PROGRAM.with(|p| p.borrow_mut().record_prefix = true);
        run(&original, original.seed).unwrap();
        PROGRAM.with(|p| {
            assert_eq!(
                p.borrow().prefix,
                late_prefix,
                "late mute changed the causal prelude"
            )
        });
    }
}

#[test]
fn recurrent_line_early_mute_control_stays_below_the_true_vi_limit() {
    crate::log::set_verbosity(crate::log::Level::Error);
    for guard in [false, true] {
        let _reset = program(vec![(0.0, 180.0), (26.511524363702225, 1000.0)], Some(0.0));
        let mut sc = witness();
        sc.guard = guard;
        let out = run(&sc, sc.seed).unwrap();
        eprintln!("EARLY guard={guard} peaks={:?}", out.true_vi_peaks);
        assert_true_vi_limit(&sc, &out);
    }
}

/// The independent 768-case family, including all of its RNG draws. The
/// first half changes program and fault timing, the second half recovers
/// before another low line. Do not filter out cases that already exposed
/// the baseline's separate no-bass/partial-line gap.
fn adversarial(id: usize) -> (ViScenario, Vec<(f64, f64)>) {
    let mut r = Rng::new(680000001 + id as u64);
    let j413 = r.chance(0.5);
    let guard = r.chance(0.5);
    let bass = [40.0, 60.0, 100.0, 180.0, 250.0][(r.f() * 5.0) as usize];
    let treble = [1000.0, 3000.0, 6000.0][(r.f() * 3.0) as usize];
    let edge = r.range(3.0, 25.0);
    let end = edge + r.range(8.0, 85.0);
    let secs = end + r.range(5.0, 30.0);
    let first_ratio = r.range(0.5, 0.9);
    let next_ratio = if id % 3 == 0 { 1.0 } else { r.range(0.5, 0.9) };
    let level = r.range(0.15, 0.85);
    let next_level = r.range(0.1, 1.0);
    let mut sc = ViScenario::long(j413, bass, false, level, guard, false, secs);
    sc.seed = 680000001 + id as u64;
    sc.rate = if id % 4 == 0 { 8000 } else { 16000 };
    let impedance = r.range(1.0, 1.6);
    let phase = r.range(-1.0, 1.0);
    sc.loads.fill((impedance, phase));
    sc.streams[0].levels = vec![(0.05, level), (edge, next_level), (end, level)];
    sc.faults.push(ViFault::ILow {
        at: 0.0,
        len: edge,
        k: first_ratio,
        offset: 0.0,
    });
    sc.faults.push(ViFault::ILow {
        at: edge,
        len: secs + 10.0,
        k: next_ratio,
        offset: 0.0,
    });
    let mut frequencies = vec![(0.0, bass), (edge, treble), (end, bass)];
    if id >= 384 {
        let recovery = r.range(8.0, 40.0);
        let onset = recovery + r.range(0.2, 12.0);
        let first_ratio = r.range(0.65, 0.85);
        let next_ratio = r.range(0.55, 0.8);
        sc = ViScenario::long(j413, bass, false, level, guard, false, onset + 100.0);
        sc.seed = 680000001 + id as u64;
        sc.loads.fill((impedance, phase));
        sc.streams[0].levels = vec![(0.05, level), (recovery, next_level), (onset, level)];
        sc.faults.push(ViFault::ILow {
            at: 0.0,
            len: recovery,
            k: first_ratio,
            offset: 0.0,
        });
        sc.faults.push(ViFault::ILow {
            at: onset,
            len: 110.0,
            k: next_ratio,
            offset: 0.0,
        });
        frequencies = vec![(0.0, bass), (onset + 2.0, treble)];
    }
    match id % 8 {
        1 => sc.faults.push(ViFault::Base(Fault::WriteFail {
            at: edge,
            len: 0.25,
            partial: true,
        })),
        2 => sc.faults.push(ViFault::Base(Fault::ClockSplit {
            at: edge,
            len: 0.2,
            pause: 0.3,
        })),
        3 => sc.faults.push(ViFault::Base(Fault::SenseLoss {
            at: edge,
            len: 0.25,
        })),
        4 => {
            let mut next = sc.streams[0].clone();
            sc.streams[0].close = edge;
            next.open = edge + 0.2;
            sc.streams.push(next);
        }
        5 => sc.faults.push(ViFault::Base(Fault::WriteFail {
            at: 0.05,
            len: 0.25,
            partial: true,
        })),
        6 => sc.faults.push(ViFault::LateBlock {
            at: edge,
            delay: 0.15,
        }),
        _ => {}
    }
    (sc, frequencies)
}

#[test]
fn all_768_adversarial_cases() {
    crate::log::set_verbosity(crate::log::Level::Error);
    let rows = std::sync::Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for worker in 0..8 {
            let rows = &rows;
            scope.spawn(move || {
                for id in (worker..768).step_by(8) {
                    let (sc, frequencies) = adversarial(id);
                    let input = format!("{} {frequencies:?}", sc.to_rust())
                        .bytes()
                        .fold(0xcbf29ce484222325u64, |h, b| {
                            (h ^ b as u64).wrapping_mul(0x100000001b3)
                        });
                    let _reset = program(frequencies, None);
                    let out = run(&sc, sc.seed).unwrap_or_else(|v| panic!("case {id}: {}", v.0));
                    rows.lock().unwrap().push((id, input, out.true_vi_peaks));
                }
            });
        }
    });
    let mut rows = rows.into_inner().unwrap();
    rows.sort_by_key(|x| x.0);
    assert_eq!(rows.len(), 768);
    let baseline: Vec<Vec<&str>> = include_str!("recurrent-baseline.csv")
        .lines()
        .filter(|s| !s.starts_with('#') && !s.is_empty())
        .map(|s| s.split(',').collect())
        .collect();
    assert_eq!(
        baseline.len(),
        768,
        "every adversarial case needs its baseline pair"
    );
    let reviewed: Vec<Vec<&str>> = include_str!("recurrent-reviewed.csv")
        .lines()
        .filter(|s| !s.starts_with('#') && !s.is_empty())
        .map(|s| s.split(',').collect())
        .collect();
    assert_eq!(
        reviewed.len(),
        768,
        "every case needs its reviewed-parent pair"
    );
    let mut failures = Vec::new();
    for (id, input, peaks) in rows {
        eprintln!("PAIRED {id} {input:016x} {peaks:?}");
        let expected = &baseline[id];
        assert_eq!(expected.len(), 8);
        assert_eq!(expected[0].parse::<usize>().unwrap(), id);
        assert_eq!(
            u64::from_str_radix(expected[1], 16).unwrap(),
            input,
            "case {id} changed its inputs"
        );
        let parent = &reviewed[id];
        assert_eq!(parent.len(), 8);
        assert_eq!(parent[0].parse::<usize>().unwrap(), id);
        assert_eq!(
            u64::from_str_radix(parent[1], 16).unwrap(),
            input,
            "case {id} changed its reviewed inputs"
        );
        let (sc, _) = adversarial(id);
        for (k, speaker) in vi_cfg(&sc).speakers.iter().enumerate() {
            let before = expected[k + 2].parse::<f64>().unwrap();
            let after = peaks[k];
            // Existing no-bass/partial-line gaps are kept in the pair.
            // Never grant their allowance to a different speaker: every
            // previously safe speaker must still pass its hard limit.
            if before <= speaker.t_limit && !(after <= speaker.t_limit) {
                failures.push(format!(
                    "case {id}, {} new true V/I limit crossing: {before:.9} -> {after:.9} C",
                    speaker.name
                ));
            }
            // The reviewed parent already changes some complete-loop
            // peaks relative to baseline. Keep both references: forbid
            // new baseline limit crossings above, and any added peak
            // relative to this fix's exact parent here. The ordinary
            // recovered-line witness also keeps the woofer's baseline
            // ceiling, not its hotter parent result.
            let inherited = parent[k + 2].parse::<f64>().unwrap();
            assert!(before.is_finite() && inherited.is_finite());
            if !(after.is_finite() && after <= inherited + 0.02) {
                failures.push(format!(
                    "case {id}, {} true V/I peak exceeds reviewed parent: baseline {before:.9}, reviewed {inherited:.9}, candidate {after:.9} C",
                    speaker.name
                ));
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The earlier bass/treble/bass witness must stay protected while the
/// line cannot be judged in the long treble passage.
#[test]
fn low_line_is_kept_through_eighty_seconds_without_bass() {
    crate::log::set_verbosity(crate::log::Level::Error);
    for guard in [false, true] {
        let _reset = program(vec![(0.0, 100.0), (20.0, 3000.0), (100.0, 100.0)], None);
        let mut sc = ViScenario::long(false, 100.0, false, 0.5, guard, false, 140.0);
        sc.faults.push(ViFault::ILow {
            at: 0.0,
            len: 150.0,
            k: 0.65,
            offset: 0.0,
        });
        let out = run(&sc, 700000001).unwrap();
        eprintln!("NO_BASS guard={guard} peaks={:?}", out.true_vi_peaks);
        assert_true_vi_limit(&sc, &out);
    }
}

/// The ordinary case must also lose the original woofer regression,
/// before the late-mute/level-ramp variant turns it into a limit crossing.
#[test]
fn ordinary_recurrent_line_keeps_the_baseline_woofer_peak() {
    crate::log::set_verbosity(crate::log::Level::Error);
    let (sc, frequencies) = adversarial(667);
    assert!(sc.j413 && !sc.guard);
    let _reset = program(frequencies, None);
    let out = run(&sc, sc.seed).unwrap();
    let baseline: Vec<&str> = include_str!("recurrent-baseline.csv")
        .lines()
        .find(|s| s.starts_with("667,"))
        .unwrap()
        .split(',')
        .collect();
    for (k, speaker) in vi_cfg(&sc).speakers.iter().enumerate() {
        if speaker.name.contains("Woofer") {
            let before = baseline[k + 2].parse::<f64>().unwrap();
            assert!(
                out.true_vi_peaks[k] <= before + 0.02,
                "{} ordinary recovered-line peak {} exceeds baseline {} C",
                speaker.name,
                out.true_vi_peaks[k],
                before
            );
        }
    }
    eprintln!("ORDINARY case=667 peaks={:?}", out.true_vi_peaks);
}

// SPDX-License-Identifier: MIT
//! Mainline queue and floor regressions, with per-speaker histories.
use super::*;
use super::fake::*;
use super::tests::{j700, tmp_state};
fn secs(s: f64) -> Duration { Duration::from_secs_f64(s) }
    /// A guard holding the J700's lock at 48 kHz, both coils at `coil` and
    /// their magnets at `magnet`, the controls' history (seconds ago, dB)
    /// as given and the lock taken `unlocked` s ago: the W1 rules' state.
    fn w1_state(coil: f64, magnet: f64, history: &[(f64, f64)], unlocked: f64) -> (Guard, Instant) {
        let cfg = j700();
        let mut hw = FakeHw::new(2, 4096);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        hw.t += secs(5.0);
        let now = hw.now();
        g.rate = 48000;
        g.armed = true;
        g.armed_at = Some(now - secs(unlocked));
        g.gains = history.iter().map(|&(ago, db)| Gain { at: now - secs(ago), lo: vec![db; 2], hi: vec![db; 2], kernel: false, cands: vec![vec![(db, db)]; 2] }).collect();
        for s in g.speakers.iter_mut() {
            s.resume(coil, magnet, 0.0, 0.0);
            s.govern(&cfg.globals);
        }
        (g, now)
    }

    /// Review r7 Y4, the queue rule's boost: the program was measured
    /// through 6 dB, and audio converted at 0 dB 0.1 s ago is still in the
    /// queue for 0.24 s.  It plays the program 6 dB louder than measured:
    /// 141 C from a 110 C coil, where the measured level gives 116 C.
    #[test]
    fn the_queue_rule_plays_the_program_louder_through_the_least_gain() {
        let (mut g, now) = w1_state(110.0, 70.0, &[(0.3, 0.0), (0.1, 6.0)], 1.0);
        for s in g.speakers.iter_mut() {
            s.p_recent = s.worst_case_power(6.0);
        }
        assert!(matches!(g.queue_threat(now), Some((_, peak, Threat::Queue)) if peak > 135.0), "{:?}", g.queue_threat(now));
    }

    /// Review r7 Y4, the clamp: the same history, but the lock was taken
    /// 50 ms ago.  What was converted at 0 dB before it was stopped then
    /// (or played at the card's safe level): nothing is queued at a lower
    /// gain, and nothing is stopped.
    #[test]
    fn the_queue_rule_looks_back_only_to_the_last_unlock() {
        let (mut g, now) = w1_state(110.0, 70.0, &[(0.3, 0.0), (0.1, 6.0)], 0.05);
        for s in g.speakers.iter_mut() {
            s.p_recent = s.worst_case_power(6.0);
        }
        assert_eq!(g.queue_threat(now), None);
    }

    /// Review r7 Y1: audio converted at the least gain plays only until a
    /// copy queue (341 ms) after the controls stopped holding it.  0 dB was
    /// replaced 0.4 s ago: none of it can still be playing, though it lies
    /// inside QUEUE_BOUND, and the burst is not stopped (0.1.11 stopped it,
    /// at 172 C over a whole QUEUE_BOUND from now).
    #[test]
    fn audio_queued_at_a_replaced_gain_plays_only_a_queue_after_it() {
        let (mut g, now) = w1_state(110.0, 70.0, &[(0.45, 0.0), (0.4, 6.0)], 1.0);
        for s in g.speakers.iter_mut() {
            s.p_recent = s.worst_case_power(6.0);
        }
        assert_eq!(g.queue_threat(now), None);
    }

    /// Review r7 Y8: each rule hands off in its own words.  A 125 C coil at
    /// 0 dB could reach 138 C in the next look at full scale: the reach
    /// rule, not the queue's.
    #[test]
    fn each_w1_rule_logs_its_own_reason() {
        let (g, now) = w1_state(125.0, 80.0, &[(1.0, 0.0)], 2.0);
        let Some((k, peak, rule)) = g.queue_threat(now) else { panic!("no threat") };
        assert_eq!(rule, Threat::Reach);
        let reach = g.threat_reason(k, peak, Threat::Reach);
        let queue = g.threat_reason(k, peak, Threat::Queue);
        assert!(reach.contains("at full scale through the least gain") && !reach.contains("queued"), "{reach}");
        assert!(queue.contains("queued at a lower gain"), "{queue}");
    }

    /// Review r7 Y2: the floor keeps full scale for a lease, a period and
    /// a stop behind a copy queue (0.68 s at 48 kHz) under every coil's
    /// limit less FLUSH_MARGIN_K, and is no higher than that needs (within
    /// 0.05 dB).  Its cost, for a coil at `t` and its magnet where a steady
    /// program puts it: 0.5 dB at 50 C, 1.5 at 70 C, 2.5 at 85 C, 3.9 at
    /// 100 C, 5.0 at 110 C, where the governor's 6 dB takes over.
    #[test]
    fn the_floor_keeps_a_held_up_full_scale_under_the_limit() {
        let cfg = j700();
        let secs_held = f64::from(COPY_QUEUE_FRAMES) / 48000.0 + 0.25 + 4096.0 / 48000.0 + 0.005;
        for (t, cost) in [(50.0, 0.46), (60.0, 0.96), (70.0, 1.53), (85.0, 2.54), (100.0, 3.86), (110.0, 5.04), (120.0, 6.66)] {
            let ta = cfg.globals.t_ambient;
            let magnet = ta + (t - ta) * 26.0 / 64.0;
            let (g, _) = w1_state(t, magnet, &[(1.0, 0.0)], 2.0);
            let floor = g.floor_db(0.0);
            eprintln!("floor at a {t} C coil (magnet {magnet:.1} C): {floor:.2} dB, the governor {:.2} dB", model::reduction_db(&g.speakers));
            assert!((floor - cost).abs() < 0.05, "{t} C: {floor} dB");
            let near = |s: &SpeakerState, db: f64| s.peak_if(s.worst_case_power(db), secs_held, ta) >= s.spec.t_limit - FLUSH_MARGIN_K;
            assert!(!g.speakers.iter().any(|s| near(s, floor)), "{t} C: the floor lets a coil near its limit");
            assert!(g.speakers.iter().any(|s| near(s, floor - 0.05)), "{t} C: the floor is higher than it needs");
        }
    }

    struct SafeSays {
        hw: FakeHw,
        card: Option<f64>,
    }
    impl Hw for SafeSays {
        fn now(&self) -> Instant { self.hw.now() }
        fn boottime(&self) -> Duration { self.hw.boottime() }
        fn sleep(&mut self, d: Duration) { self.hw.sleep(d) }
        fn take_lock(&mut self) -> io::Result<()> { self.hw.take_lock() }
        fn ping(&mut self) -> io::Result<()> { self.hw.ping() }
        fn set_min(&mut self) -> io::Result<()> { self.hw.set_min() }
        fn card_safe_db(&self) -> Option<f64> { self.card }
        fn set_reduction(&mut self, d: f64) -> io::Result<f64> { self.hw.set_reduction(d) }
        fn relinquish(&mut self) -> io::Result<()> { self.hw.relinquish() }
        fn sample_rate(&mut self) -> io::Result<i64> { self.hw.sample_rate() }
        fn open_sense(&mut self, r: u32) -> io::Result<()> { self.hw.open_sense(r) }
        fn close_sense(&mut self) { self.hw.close_sense() }
        fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> { self.hw.read_period(w) }
        fn queued(&mut self) -> io::Result<u64> { self.hw.queued() }
        fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> { self.hw.read_tail(w) }
        fn wait_rate_change(&mut self, d: Duration) -> bool { self.hw.wait_rate_change(d) }
        fn rate_events(&mut self) -> bool { self.hw.rate_events() }
        fn events_available(&self) -> bool { true }
    }

    #[test]
    fn a_card_louder_at_its_safe_level_than_configured_is_never_unlocked() {
        let cfg = j700();
        for (card, unlocks) in [(15.0, false), (20.0, true), (25.0, true)] {
            let mut hw = SafeSays { hw: FakeHw::new(2, 4096), card: Some(card) };
            hw.hw.level = Box::new(|_| 0.0);
            let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
            while hw.hw.t < secs(5.0) {
                if let Err(e) = g.tick(&mut hw) {
                    assert!(!unlocks && e.to_string().contains("kernel safe level"), "{e}");
                    break;
                }
            }
            assert_eq!(g.armed(), unlocks, "a card safe level of {card} dB");
            if !unlocks {
                assert!(hw.hw.pings.len() <= 1 && !hw.hw.unlocked_at(hw.hw.t), "{:?}", hw.hw.pings);
            }
        }
    }

    /// A Hw that stalls after a ping or a write took effect (descheduled
    /// on the way back), as the sixth review's pauses after the effect.
    struct StallAfter {
        hw: FakeHw,
        ping_stall: Duration,
        write_stall: Duration,
    }
    impl Hw for StallAfter {
        fn now(&self) -> Instant { self.hw.now() }
        fn boottime(&self) -> Duration { self.hw.boottime() }
        fn sleep(&mut self, d: Duration) { self.hw.sleep(d) }
        fn take_lock(&mut self) -> io::Result<()> { self.hw.take_lock() }
        fn ping(&mut self) -> io::Result<()> {
            let r = self.hw.ping();
            self.hw.t += self.ping_stall;
            r
        }
        fn set_min(&mut self) -> io::Result<()> { self.hw.set_min() }
        fn set_reduction(&mut self, d: f64) -> io::Result<f64> {
            let r = self.hw.set_reduction(d);
            self.hw.t += self.write_stall;
            r
        }
        fn relinquish(&mut self) -> io::Result<()> { self.hw.relinquish() }
        fn sample_rate(&mut self) -> io::Result<i64> { self.hw.sample_rate() }
        fn open_sense(&mut self, r: u32) -> io::Result<()> { self.hw.open_sense(r) }
        fn close_sense(&mut self) { self.hw.close_sense() }
        fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> { self.hw.read_period(w) }
        fn queued(&mut self) -> io::Result<u64> { self.hw.queued() }
        fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> { self.hw.read_tail(w) }
        fn wait_rate_change(&mut self, d: Duration) -> bool { self.hw.wait_rate_change(d) }
        fn rate_events(&mut self) -> bool { self.hw.rate_events() }
        fn events_available(&self) -> bool { true }
    }

    /// Review r6 W4: a ping that took effect before the daemon stalled
    /// 0.2 s on its way back started the lease before the clock read; the
    /// lapse it bounds is counted from before the call.
    #[test]
    fn the_lease_counts_from_before_the_ping() {
        let cfg = j700();
        let mut hw = StallAfter { hw: FakeHw::new(2, 4096), ping_stall: Duration::ZERO, write_stall: Duration::ZERO };
        hw.hw.level = Box::new(|_| 0.0);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
        while !g.armed() {
            g.tick(&mut hw).unwrap();
        }
        hw.ping_stall = secs(0.2);
        let before = hw.now();
        g.ping_lock(&mut hw).unwrap();
        assert!(g.last_ping <= before, "the lease counted from {:?} after the call began", g.last_ping.saturating_duration_since(before));
    }

    /// Review r6 W5: frames placed at `end` may have played up to `shift`
    /// frames later; a lease that ran out in between counts.
    #[test]
    fn a_shifted_run_checks_the_lapse_at_its_late_bound() {
        let cfg = j700();
        let p = tmp_state().unwrap();
        let mut hw = FakeHw::new(2, 4096);
        let mut g = Guard::start(&mut hw, &cfg, 20.0, Some(p)).unwrap();
        let t0 = hw.now();
        g.rate = 48000;
        g.armed = true;
        g.last_ping = t0;
        g.account_to = t0 + secs(0.2);
        g.gains = vec![Gain { at: t0 - secs(1.0), lo: vec![0.0; 2], hi: vec![0.0; 2], kernel: false, cands: vec![vec![(0.0, 0.0)]; 2] }];
        // ends 10 ms before the lapse, may have played 20 ms later
        g.measure_shifted(t0 + secs(0.24), 4096, 960).unwrap();
        let lapse = t0 + KERNEL_LOCK_TIMEOUT;
        assert!(g.gains.iter().any(|e| e.kernel && e.at <= lapse + secs(1e-6) && e.at >= lapse - secs(1e-6)), "no safe level from the lapse: {:?}", g.gains);
    }


#[test]
fn recent_programme_power_is_observed_and_cleared_on_close() {
    let cfg = j700();
    let mut hw = FakeHw::new(2, 4096);
    hw.level = Box::new(|_| 0.5);
    hw.playing = Box::new(|t| t < secs(2.0));
    let mut g = Guard::start(&mut hw, &cfg, 20.0, tmp_state()).unwrap();
    super::tests::run_until(&mut g, &mut hw, secs(1.0)).unwrap();
    assert!(g.speakers.iter().all(|s| s.p_recent > 0.0));
    super::tests::run_until(&mut g, &mut hw, secs(3.0)).unwrap();
    assert!(g.speakers.iter().all(|s| s.p_recent == 0.0));
}

/// The full V/I smoothing stack must not silently remove main's J700
/// configured hand-off policy. The queue and onset bounds alone permit
/// this state, so only the ordinary-demand limit hands it back.
#[test]
fn j700_keeps_the_configured_governor_handoff_limit() {
    let cfg = j700();
    let mut hw = FakeHw::new(2, 4096);
    let mut g = Guard::start(&mut hw, &cfg, 4.0, tmp_state()).unwrap();
    let now = hw.now();
    g.armed = true;
    g.armed_at = Some(now);
    g.last_ping = now;
    g.rate = 48000;
    g.written = vec![10.0; 2];
    g.written_hi = g.written.clone();
    g.written_set = vec![vec![(10.0, 10.0)]; 2];
    g.gains = vec![Gain { at: now, lo: vec![10.0; 2], hi: vec![10.0; 2],
        kernel: false, cands: vec![vec![(10.0, 10.0)]; 2] }];
    for s in &mut g.speakers {
        s.resume(110.0, 70.0, 0.0, 0.0);
        s.govern(&cfg.globals);
    }
    let (demand, mut per, hard) = g.governing_state();
    assert!(demand > 4.0 && !hard);
    assert!(g.bound_onset(now, &mut per));
    assert!(g.queue_threat(now).is_none());
    g.decide(&mut hw, false).unwrap();
    assert!(!g.armed(), "ordinary J700 demand bypassed its configured hand-off limit");
    assert_eq!(hw.relinquished, 1);
}

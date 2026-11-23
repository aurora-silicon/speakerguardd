//! The path-integrity checks on a fuzzed sequence of ticks: the engine page
//! (written by the PipeWire process, so anything), the speaker PCM, the
//! session helper's report, the clock and system sleeps, through
//! `Observer::observe`, `Lease::tick` and `Regrant` as the supervisor's
//! loop drives them. Nothing may panic or overflow (build with `-a`);
//! every check that passes is backed by what was read, and the lease is
//! renewed only on a clean tick with demand, after two clean ticks.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sgv2_core::Health;
use sgv2_supervisor::lease::{Action, Lease, Observation, Regrant};
use sgv2_supervisor::observe::*;
use sgv2_supervisor_fuzz::arbitrary::{Result, Unstructured};

struct E(EngineState);
impl EngineView for E {
    fn read(&mut self) -> EngineState {
        self.0
    }
}
struct P(PcmState);
impl PcmView for P {
    fn read(&mut self) -> PcmState {
        self.0
    }
}
struct S(SessionReport);
impl SessionView for S {
    fn read(&mut self) -> SessionReport {
        self.0
    }
}

const CRC: u32 = 0x5eed_c0de;
const WIRING: u64 = 0x0979_c55e_d5dd_f377;
const PID: u32 = 4242;

fn run(u: &mut Unstructured) -> Result<()> {
    let rate = [48_000, 44_100, 96_000, 8_000, 192_000][u.int_in_range(0..=4usize)?];
    let tick = 10_000_000u64;
    let mut obs = Observer::new(CRC, rate, tick);
    if u.arbitrary()? {
        obs.expect_wiring(WIRING);
    }
    let mut lease = Lease::default();
    let mut regrant = Regrant::default();
    let mut now = u.int_in_range(1..=u64::from(u32::MAX))?;
    const CLOCK_MAX: u64 = 1 << 62;
    let mut e =
        EngineState { valid: true, pid: PID, params_crc: CRC, min_gain_bits: 1f32.to_bits(), ..Default::default() };
    let mut clean_run = 0u32;
    for _ in 0..u.int_in_range(1..=64u8)? {
        now = now
            .saturating_add(match u.int_in_range(0..=7u8)? {
                0 => 0,
                1 => u.int_in_range(0..=1_000_000_000u64)?,
                // a long gap (a monotonic clock in ns runs for centuries, so
                // the model keeps it below 2^62: the observer may add its
                // bounds to it)
                2 => 1 << 40,
                _ => tick,
            })
            .min(CLOCK_MAX);
        // the page: mostly a steady plugin, now and then anything
        if u.ratio(1, 8)? {
            e = EngineState {
                valid: u.arbitrary()?,
                pid: [PID, 0, 7, u.arbitrary()?][u.int_in_range(0..=3usize)?],
                params_crc: if u.arbitrary()? { CRC } else { u.arbitrary()? },
                seq: u.arbitrary()?,
                frames: u.arbitrary()?,
                mono_ns: u.arbitrary()?,
                health: u.arbitrary()?,
                min_gain_bits: u.arbitrary()?,
            };
        } else {
            let beats = u.int_in_range(0..=2u64)?;
            e.seq = e.seq.wrapping_add(beats);
            e.frames = e.frames.wrapping_add(beats * [256, 1024, 0, 4096][u.int_in_range(0..=3usize)?]);
            if beats > 0 {
                e.mono_ns = now.saturating_sub(u.int_in_range(0..=tick)?);
            }
        }
        let owner = [PID, 0, 99, e.pid][u.int_in_range(0..=3usize)?];
        let p = match u.int_in_range(0..=7u8)? {
            0 => PcmState::Closed,
            1 => PcmState::Open { owner },
            2 => PcmState::Xrun { owner },
            3 => PcmState::Stopped { owner },
            4 => PcmState::Opening,
            5 => PcmState::Unknown,
            _ => PcmState::Running { owner },
        };
        let s = SessionReport {
            present: u.ratio(7, 8)?,
            links_ok: u.ratio(7, 8)?,
            carrier_ok: u.ratio(7, 8)?,
            mono_ns: match u.int_in_range(0..=3u8)? {
                0 => u.arbitrary()?,
                _ => now.saturating_sub(u.int_in_range(0..=600_000_000u64)?),
            },
            helper: Some((PID + 1, 777)),
            helper_alive: u.ratio(7, 8)?,
            wiring: if u.ratio(7, 8)? { Some(WIRING) } else { u.arbitrary()? },
            demand: u.ratio(3, 4)?,
        };
        if u.ratio(1, 32)? {
            obs.note_sleep();
        }
        let o = obs.observe(&mut || now, &mut E(e), &mut P(p), &mut S(s));
        let fresh = s.present && s.helper_alive && s.mono_ns <= now.saturating_add(SESSION_SKEW_NS);
        if o.plugin_ok {
            assert!(e.valid && e.params_crc == CRC && e.gain_ok());
            assert!(e.health & (Health::FAILSAFE | Health::NO_PARAMS) == 0);
        }
        if o.heartbeat_fresh {
            assert!(e.valid && e.mono_ns <= now, "fresh heartbeat from {e:?} at {now}");
        }
        if o.pcm_owner_ok {
            if let Some(o) = p.owner() {
                assert!(o == e.pid && o != 0, "PCM held by {o}, engine {}", e.pid);
            } else {
                assert!(matches!(p, PcmState::Closed | PcmState::Opening));
            }
        }
        if o.pcm_state_ok {
            assert!(!matches!(p, PcmState::Stopped { .. } | PcmState::Unknown));
        }
        if o.links_ok || o.carrier_ok || o.demand {
            assert!(fresh, "a session check passed on a report that is not fresh: {s:?}");
        }
        if o.links_ok {
            assert!(s.links_ok);
        }
        if obs.sense_proven() {
            assert!(fresh && s.wiring == Some(WIRING) && e.valid && e.params_crc == CRC);
        }
        // the kernel's side holds in this model
        let o = Observation { route_ok: true, amps_ok: true, ..o };
        let o = if regrant.holding(now) { Observation { demand: false, ..o } } else { o };
        let was = lease.held();
        let a = lease.tick(o);
        clean_run = if o.clean() { clean_run + 1 } else { 0 };
        match a {
            Action::Renew => {
                assert!(o.clean() && o.demand, "renewed on {o:?}");
                assert!(was || clean_run >= 2, "granted after {clean_run} clean ticks");
                if !was {
                    obs.note_grant(now);
                    regrant.granted(now);
                }
            }
            Action::Release => {
                assert!(was && !o.clean());
                obs.note_fault_release(&o);
                regrant.released(now, true);
            }
            Action::Idle => {
                assert!(was && o.clean() && !o.demand);
                regrant.released(now, false);
                regrant.idle();
            }
            Action::StayReleased => assert!(!was),
        }
        assert_eq!(lease.held(), a == Action::Renew);
    }
    Ok(())
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let _ = run(&mut u);
});

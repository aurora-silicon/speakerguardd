//! Adversarial property tests for the protection core (lane T).
//!
//! The core's one job is never to drive a speaker past what its model
//! allows, so these tests attack that from every side the host, the sense
//! bus and the parameter blob can reach:
//!
//! 1. **Output bound.** `|out[n]| <= |in[n - latency]|` for every unit (no
//!    unit has gain above 0 dB; no unit generates a pilot yet, so there is no
//!    pilot headroom to allow), and a non-finite input sample leaves as 0.
//! 2. **Finite output.** Any input, including NaN, +-Inf, subnormals and very
//!    large finite values, gives finite output and finite model state.
//! 3. **Slice invariance.** Output and state are bit-identical across random
//!    host slice sequences (1..=4096 frames), and `reset` returns a unit to
//!    exactly its initial state.
//! 4. **Hard path.** Once the modelled coil temperature is past the hard
//!    limit at a block boundary, the whole group's next block leaves at least
//!    20 dB down (spp3, atsp). lsm2's soft temperature path reaches its floor.
//! 5. **Parameter blobs.** Truncated, corrupted, wrong-CRC, out-of-range and
//!    hostile blobs never panic; whatever they build obeys 1 and 2, and a
//!    rejected blob gives the 20 dB fail-safe.
//! 6. **Sense dropouts.** Missing, short, NaN, infinite or implausible V/I
//!    sense never raises the gain above the open-loop or valid-sense gain.
//! 7. **No allocation.** Every `process` and `telemetry` call in this file is
//!    run under a per-thread counting allocator.
//!
//! Every parameter set is synthetic, generated from ranges around the
//! crate's own test builders; no Mac's tuning appears here. Case counts are
//! modest so CI stays fast; set `PROPTEST_CASES` for long runs. The fuzz
//! targets in `crates/sgv2-core/fuzz` cover the same properties with
//! coverage guidance.
//!
//! Regression tests for the defects these tests found are at the end. They
//! are `#[ignore]`d so the suite stays green until lane B fixes them; run
//! them with `cargo test --test adversarial -- --ignored`.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use proptest::prelude::*;
use proptest::test_runner::Config;
use sgv2_core::blob::{self, Pod};
use sgv2_core::{atsp, lsm2, placeholder, spp3, FailSafe, Health, Io, ProtectionUnit, Telemetry, UnitKind};
use sgv2_core::{FAILSAFE_GAIN, MAX_CHANNELS};

// ---------------------------------------------------------------------------
// Property 7: a per-thread counting allocator (tests run on many threads).

struct Counting;

thread_local! {
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

// SAFETY: forwards to the system allocator; the counter is a const-initialised
// thread-local `Cell`, which never allocates.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.realloc(ptr, layout, new_size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn allocs() -> usize {
    ALLOCS.with(|c| c.get())
}

// ---------------------------------------------------------------------------
// Deterministic generators. Proptest drives the seeds and the shape knobs
// (which shrink); the samples themselves come from SplitMix64 so a case with
// 8 x 20k samples stays cheap to generate and to shrink.

#[derive(Clone)]
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn uni(&mut self, lo: f64, hi: f64) -> f32 {
        (lo + (hi - lo) * self.unit()) as f32
    }
    fn log(&mut self, lo: f64, hi: f64) -> f32 {
        (lo.ln() + (hi.ln() - lo.ln()) * self.unit()).exp() as f32
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }
    fn chance(&mut self, p: f64) -> bool {
        self.unit() < p
    }
    fn perm(&mut self, n: usize) -> Vec<u32> {
        let mut v: Vec<u32> = (0..n as u32).collect();
        for i in (1..n).rev() {
            v.swap(i, self.below(i + 1));
        }
        v
    }
}

/// How hostile the samples are.
#[derive(Clone, Copy, Debug)]
enum Hostility {
    /// Finite audio only, up to `amp`.
    Clean,
    /// Clean plus NaN, +-Inf, subnormals, signed zeros and `MIN_POSITIVE`.
    Specials,
    /// Specials plus large finite samples (up to 1e6).
    Wild,
    /// Large finite samples (up to 1e6), no specials.
    Large,
}

fn hostility() -> impl Strategy<Value = Hostility> {
    prop_oneof![Just(Hostility::Clean), Just(Hostility::Specials), Just(Hostility::Wild), Just(Hostility::Large)]
}

/// Finite input only: spp3 and atsp apply their non-finite fail-safe to the
/// whole host segment, which is not slice invariant (FINDING 5).
fn finite_hostility() -> impl Strategy<Value = Hostility> {
    prop_oneof![Just(Hostility::Clean), Just(Hostility::Large)]
}

/// A special sample (never "large": see the huge-input regression test).
fn special(r: &mut Rng) -> f32 {
    match r.below(8) {
        0 => f32::NAN,
        1 => f32::INFINITY,
        2 => f32::NEG_INFINITY,
        3 => f32::from_bits(1 + r.below(0x7F_FFFE) as u32),
        4 => -f32::from_bits(1 + r.below(0x7F_FFFE) as u32),
        5 => -0.0,
        6 => f32::MIN_POSITIVE,
        _ => f32::from_bits(0x7FC0_0000 | r.below(0x3F_FFFF) as u32 | 1),
    }
}

/// `channels` signals of `len` frames: bursts of noise and tones at random
/// levels up to `amp`, with silences, and specials at `hostility`.
fn signal(seed: u64, channels: usize, len: usize, amp: f32, h: Hostility) -> Vec<Vec<f32>> {
    let mut r = Rng(seed);
    (0..channels)
        .map(|_| {
            let mut v = Vec::with_capacity(len);
            while v.len() < len {
                let run = 1 + r.below(6000);
                let level = match r.below(5) {
                    0 => 0.0,
                    1 => amp * r.uni(0.0, 0.05),
                    _ => amp * r.uni(0.2, 1.0),
                };
                let tone = r.chance(0.4);
                let f = r.uni(20.0, 12000.0) as f64 / 48000.0;
                let ph0 = v.len();
                for k in 0..run {
                    if v.len() == len {
                        break;
                    }
                    let mut x = if tone {
                        level * (2.0 * std::f64::consts::PI * f * (ph0 + k) as f64).sin() as f32
                    } else {
                        level * r.uni(-1.0, 1.0)
                    };
                    match h {
                        Hostility::Clean => {}
                        Hostility::Specials => {
                            if r.chance(0.002) {
                                x = special(&mut r)
                            }
                        }
                        Hostility::Wild => {
                            if r.chance(0.004) {
                                x = special(&mut r)
                            } else if r.chance(0.002) {
                                x = r.log(10.0, 1e6) * if r.chance(0.5) { 1.0 } else { -1.0 };
                            }
                        }
                        Hostility::Large => {
                            if r.chance(0.003) {
                                x = r.log(10.0, 1e6) * if r.chance(0.5) { 1.0 } else { -1.0 };
                            }
                        }
                    }
                    v.push(x);
                }
            }
            v
        })
        .collect()
}

/// A random host slice sequence, each slice 1..=4096 frames, biased toward
/// the sizes hosts use and toward awkward ones.
fn slices(seed: u64) -> Vec<usize> {
    let mut r = Rng(seed ^ 0x51ce);
    (0..64)
        .map(|_| match r.below(6) {
            0 => 1 + r.below(16),
            1 => [64, 128, 256, 512, 1024, 2048, 4096][r.below(7)],
            2 => [255, 257, 511, 513, 1023, 4095][r.below(6)],
            _ => 1 + r.below(4096),
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Synthetic parameter sets, around the crate's own test builders.

#[derive(Clone, Copy, Debug)]
enum Ff {
    /// Every channel's `ThermalFFSpeedupFactor` is 0: only the hard path acts.
    Off,
    On,
    Mixed,
}

fn ff() -> impl Strategy<Value = Ff> {
    prop_oneof![Just(Ff::Off), Just(Ff::On), Just(Ff::Mixed)]
}

/// A valid spp3 set. `hot` picks constants that reach the limits within a
/// few seconds of full-scale drive.
fn spp3_params(seed: u64, n: usize, block: u32, rate: u32, ff: Ff, hot: bool) -> spp3::Params {
    let mut r = Rng(seed);
    let ambient = r.uni(0.0, 45.0);
    let mut p = spp3::Params {
        channels: n as u32,
        sample_rate: rate,
        block_frames: block,
        thermal_ambient_temp: ambient,
        thermal_gain_attack_time_s: r.log(1e-3, 5.0),
        thermal_gain_release_time_s: r.log(1e-3, 20.0),
        thermal_lookahead_delay_ms: r.uni(0.0, 5.0),
        thermal_safe_tlim: r.uni(0.0, 150.0),
        thermal_safe_tlim_offset: r.uni(-10.0, 10.0),
        thermal_safe_tlim_time_min: r.uni(0.0, 10.0),
        thermal_protection_enabled: 1,
        ..spp3::Params::default()
    };
    let src = r.perm(n);
    for (c, &s) in src.iter().enumerate() {
        let limit = ambient + r.uni(5.0, 120.0);
        p.ch[c] = spp3::ChannelParams {
            source: s,
            group: r.below(4) as u32 * 7,
            ignore_telemetry: r.chance(0.2) as u32,
            t_sett_vc: if hot { r.log(5.0, 200.0) } else { r.log(0.5, 200.0) },
            t_sett_mg: if r.chance(0.1) { 0.0 } else { r.uni(0.0, 100.0) },
            tau_tvc: if hot { r.log(1e-3, 0.5) } else { r.log(1e-3, 3.0) },
            tau_tmg: r.log(1e-2, 60.0),
            temperature_limit: limit,
            hard_temp_limit_headroom: r.uni(-5.0, 25.0),
            reb_ref: r.uni(2.0, 10.0),
            rshunt: r.uni(0.0, 0.5),
            thermal_ff_speedup_factor: match ff {
                Ff::Off => 0.0,
                Ff::On => [0.25, 0.5, 1.0][r.below(3)],
                Ff::Mixed => [0.0, 0.5, 1.0][r.below(3)],
            },
            cl_thermal_feedback_enabled: r.chance(0.5) as u32,
            rampout: 0.0,
            ag: if hot { r.log(5.0, 40.0) } else { r.log(1.0, 40.0) },
            imon_scale: r.uni(0.1, 20.0),
            vmon_scale: r.uni(0.1, 20.0),
            i_pwr_thres: r.uni(0.0, 1.0),
            v_pwr_thres: r.uni(0.0, 1.0),
            sense_i: r.below(2 * n + 2) as u32,
            sense_v: r.below(2 * n + 2) as u32,
            initial_theta_vc: if r.chance(0.3) { r.uni(0.0, limit as f64 - ambient as f64 + 10.0) } else { 0.0 },
            initial_theta_mg: if r.chance(0.3) { r.uni(0.0, 30.0) } else { 0.0 },
        };
    }
    assert_eq!(p.validate(), Ok(()), "generator made an invalid spp3 set: {p:?}");
    p
}

fn atsp_params(seed: u64, n: usize, block: u32, rate: u32, hot: bool) -> atsp::Params {
    let mut r = Rng(seed);
    let mut slot = [atsp::Slot::default(); 3];
    for c in 0..n {
        slot[r.below(3)].channel_mask |= 1 << c;
    }
    for s in slot.iter_mut() {
        let ambient = r.uni(0.0, 45.0);
        *s = atsp::Slot {
            channel_mask: s.channel_mask,
            gain_link: r.chance(0.5) as u32,
            amplifier_sensitivity: if hot { r.log(5.0, 40.0) } else { r.log(1.0, 40.0) },
            rdc: r.uni(2.0, 10.0),
            rth_vc: if hot { r.log(5.0, 100.0) } else { r.log(0.5, 100.0) },
            tau_vc: if hot { r.log(1e-3, 0.5) } else { r.log(1e-3, 3.0) },
            rth_mg: if r.chance(0.1) { 0.0 } else { r.uni(0.0, 80.0) },
            tau_mg: r.log(1e-2, 60.0),
            ambient,
            temperature_limit: ambient + r.uni(5.0, 120.0),
            hard_limit_headroom: r.uni(-5.0, 25.0),
            attack_ms: r.log(0.1, 5000.0),
            release_ms: r.log(0.1, 20000.0),
        };
    }
    let p = atsp::Params {
        channels: n as u32,
        sample_rate: rate,
        block_frames: block,
        bypass: 0,
        gain_link_all: r.chance(0.3) as u32,
        slot,
    };
    assert_eq!(p.validate(), Ok(()), "generator made an invalid atsp set: {p:?}");
    p
}

fn lsm2_params(seed: u64, n: usize, rate: u32, hot: bool) -> lsm2::Params {
    let mut r = Rng(seed);
    let mut ts = [r.log(1e-3, 60.0), r.log(1e-3, 60.0), r.log(1e-3, 60.0), r.log(1e-3, 60.0)];
    ts.sort_by(f32::total_cmp);
    let mut p = lsm2::Params {
        channels: n as u32,
        sample_rate: rate,
        enable_mitigation: 1,
        control_sample_peak: r.chance(0.7) as u32,
        control_displacement: r.chance(0.7) as u32,
        control_velocity: r.chance(0.7) as u32,
        control_power: r.chance(0.7) as u32,
        control_temperature: r.chance(0.7) as u32,
        link_slow_gains: r.chance(0.5) as u32,
        unlink_temperature_gains: r.chance(0.5) as u32,
        model_only: 1,
        attack_time_ms: r.log(0.05, 50.0),
        fast_release_time_ms: r.log(1.0, 2000.0),
        slow_release_time_ms: r.log(10.0, 30000.0),
        filterbank_latency_ms: r.uni(0.0, 5.0),
        speaker_response_lookahead_ms: r.uni(0.0, 3.0),
        power_time_scale_s: ts,
        power_hard_limit_w: [r.log(0.1, 50.0), r.log(0.1, 50.0), r.log(0.1, 50.0), r.log(0.1, 50.0)],
        power_soft_limit_pct: [100.0; 4],
        max_power_attenuation_db: r.uni(0.0, 20.0),
        low_rail_idle_power_w: r.uni(0.0, 1.0),
        low_rail_power_ratio: r.uni(0.5, 2.0),
        other_power_features: 0,
        ..lsm2::Params::default()
    };
    let src = r.perm(n);
    for (c, &s) in src.iter().enumerate() {
        let ambient = r.uni(10.0, 45.0);
        let soft = ambient + if hot { r.uni(5.0, 30.0) } else { r.uni(10.0, 100.0) };
        p.ch[c] = lsm2::ChannelParams {
            source: s,
            volts_full_scale: r.log(1.0, 20.0),
            parasitic_resistance: r.uni(0.0, 1.0),
            rdc: r.uni(2.0, 8.0),
            bl: r.uni(0.3, 3.0),
            mms_kg: r.log(5e-5, 5e-3),
            rms: r.uni(0.05, 2.0),
            kms: r.log(200.0, 5000.0),
            disp_bl: r.uni(0.3, 3.0),
            disp_mms_kg: r.log(5e-5, 5e-3),
            disp_rms: r.uni(0.05, 2.0),
            disp_kms: r.log(200.0, 5000.0),
            rth_vc: r.uni(5.0, 60.0),
            tau_vc: if hot { r.log(0.01, 1.0) } else { r.log(0.1, 10.0) },
            rth_mag: r.uni(5.0, 60.0),
            tau_mag: r.log(5.0, 200.0),
            ambient,
            amplitude_limit: r.uni(0.1, 1.5),
            displacement_limit: r.log(1e-4, 2e-3),
            velocity_limit0: r.uni(0.05, 2.0),
            max_velocity_attenuation_db: r.uni(0.0, 30.0),
            temperature_soft_limit: soft,
            temperature_hard_limit: soft + r.uni(0.0, 30.0),
            temperature_headroom: r.uni(0.0, 10.0),
            max_temperature_attenuation_db: r.uni(0.0, 30.0),
        };
    }
    assert_eq!(p.validate(), Ok(()), "generator made an invalid lsm2 set: {p:?}");
    p
}

fn rate() -> impl Strategy<Value = u32> {
    prop_oneof![Just(48_000u32), Just(44_100), Just(96_000), 8_000u32..=192_000]
}

fn block() -> impl Strategy<Value = u32> {
    prop_oneof![Just(512u32), Just(256), Just(1), 1u32..=2048]
}

// ---------------------------------------------------------------------------
// The host harness.

/// What the sense bus delivers on one call.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Sense {
    /// The generated V/I for these frames.
    Valid,
    /// No sense slices at all.
    Empty,
    /// Fewer ports than the parameters name.
    Missing,
    /// Slices shorter than `frames`.
    Short,
    /// Filled with this value (NaN, Inf, huge, zero, negative, subnormal).
    Fill(f32),
    /// Valid with NaN sprinkled in.
    Holes,
}

fn sense_mode(r: &mut Rng) -> Sense {
    match r.below(10) {
        0 => Sense::Empty,
        1 => Sense::Missing,
        2 => Sense::Short,
        3 => Sense::Fill(f32::NAN),
        4 => Sense::Fill(f32::INFINITY),
        5 => Sense::Fill(1e30),
        6 => Sense::Fill(0.0),
        7 => Sense::Fill(-3.0),
        8 => Sense::Holes,
        _ => Sense::Valid,
    }
}

struct Render {
    out: Vec<Vec<f32>>,
    health: Vec<Health>,
    /// Telemetry after each call, with the frame count processed so far.
    tele: Vec<(usize, Telemetry)>,
}

/// Feed `input` (and, when given, `sense` with a per-call mode from
/// `modes`) through `unit` in `slices` (cycled). Asserts property 7 on every
/// call.
fn render(
    unit: &mut dyn ProtectionUnit,
    input: &[Vec<f32>],
    sense: Option<&[Vec<f32>]>,
    slices: &[usize],
    modes: &mut dyn FnMut(usize) -> Sense,
    keep_tele: bool,
) -> Render {
    let len = input[0].len();
    let mut out = vec![vec![0.0f32; len]; input.len()];
    let mut health = Vec::new();
    let mut tele = Vec::new();
    let mut t = Telemetry::new();
    let mut done = 0;
    let mut call = 0;
    let mut scratch: Vec<Vec<f32>> = sense.map_or(Vec::new(), |s| s.iter().map(|v| vec![0.0; v.len()]).collect());
    while done < len {
        let m = slices[call % slices.len()].min(len - done);
        let ins: Vec<&[f32]> = input.iter().map(|b| &b[done..done + m]).collect();
        let mode = modes(call);
        let sns: Vec<&[f32]> = match sense {
            None => Vec::new(),
            Some(s) => {
                let mut rr = Rng(call as u64 * 31 + 7);
                for (k, v) in scratch.iter_mut().enumerate() {
                    for (j, x) in v[done..done + m].iter_mut().enumerate() {
                        *x = match mode {
                            Sense::Fill(f) => f,
                            Sense::Holes if rr.chance(0.05) => f32::NAN,
                            _ => s[k][done + j],
                        };
                    }
                }
                match mode {
                    Sense::Empty => Vec::new(),
                    Sense::Missing => scratch.iter().take(scratch.len() / 2).map(|b| &b[done..done + m]).collect(),
                    Sense::Short => scratch.iter().map(|b| &b[done..done + m / 2]).collect(),
                    _ => scratch.iter().map(|b| &b[done..done + m]).collect(),
                }
            }
        };
        let mut outs: Vec<&mut [f32]> = out.iter_mut().map(|b| &mut b[done..done + m]).collect();
        let before = allocs();
        let h = unit.process(Io { frames: m, input: &ins, sense: &sns, output: &mut outs });
        unit.telemetry(&mut t);
        assert_eq!(allocs(), before, "{:?} allocated in process/telemetry ({m} frames)", unit.kind());
        drop(outs);
        health.push(h);
        done += m;
        if keep_tele {
            tele.push((done, t));
        }
        call += 1;
    }
    if !keep_tele {
        tele.push((done, t));
    }
    Render { out, health, tele }
}

fn no_sense(_: usize) -> Sense {
    Sense::Valid
}

/// Plausible V/I for the drive: `v = ag * x`, `i = v / 4.2`, delayed.
fn synth_sense(input: &[Vec<f32>], ports: usize, delay: usize) -> Vec<Vec<f32>> {
    let len = input[0].len();
    (0..ports)
        .map(|p| {
            let x = &input[(p / 2) % input.len()];
            (0..len)
                .map(|k| {
                    let s = if k >= delay { x[k - delay] } else { 0.0 };
                    let s = if s.is_finite() { s.clamp(-1.0, 1.0) } else { 0.0 };
                    if p % 2 == 1 {
                        10.0 * s
                    } else {
                        10.0 * s / 4.2
                    }
                })
                .collect()
        })
        .collect()
}

/// Properties 1 and 2 on one render: every output sample is finite, and no
/// louder than the input `latency` frames earlier (0 for a non-finite one).
fn check_bound(input: &[Vec<f32>], out: &[Vec<f32>], latency: usize, gain: f32, what: &str) {
    for (c, (x, y)) in input.iter().zip(out).enumerate() {
        for (k, &o) in y.iter().enumerate() {
            assert!(o.is_finite(), "{what}: non-finite output {o} at ch{c}[{k}]");
            let i = if k >= latency { x[k - latency] } else { 0.0 };
            let lim = if i.is_finite() { (i * gain).abs() } else { 0.0 };
            assert!(o.abs() <= lim, "{what}: |out| {o:e} > {lim:e} (in {i:e}, gain {gain}) at ch{c}[{k}]");
        }
    }
}

/// Model state that must stay finite while the input is not huge.
fn check_state_finite(t: &Telemetry, n: usize, what: &str) {
    for (c, s) in t.channels.iter().take(n).enumerate() {
        assert!(s.gain.is_finite() && (0.0..=1.0).contains(&s.gain), "{what}: ch{c} gain {}", s.gain);
        for (name, v) in [("coil", s.coil_temp_c), ("magnet", s.magnet_temp_c), ("power", s.modeled_power_w)] {
            assert!(v.is_finite(), "{what}: ch{c} {name} state is {v}");
        }
    }
}

fn tele_bits(t: &Telemetry) -> Vec<u64> {
    let mut v = vec![t.frame, t.health.0 as u64];
    for s in &t.channels {
        for f in [
            s.modeled_power_w,
            s.measured_power_w,
            s.coil_temp_c,
            s.magnet_temp_c,
            s.ol_temp_c,
            s.cl_temp_c,
            s.tlim_adj_c,
            s.displacement_m,
            s.velocity_m_s,
            s.mask_db,
            s.gain,
        ] {
            v.push(f.to_bits() as u64);
        }
        v.extend(s.axis_gain.iter().chain(&s.band_gain).map(|f| f.to_bits() as u64));
        v.push(s.closed_loop as u64);
    }
    v
}

fn bits(out: &[Vec<f32>]) -> Vec<Vec<u32>> {
    out.iter().map(|c| c.iter().map(|v| v.to_bits()).collect()).collect()
}

fn cfg(cases: u32) -> Config {
    Config {
        cases: std::env::var("PROPTEST_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(cases),
        ..Config::default()
    }
}

// ---------------------------------------------------------------------------
// Properties 1, 2 and 7 on every unit.

proptest! {
    #![proptest_config(cfg(48))]

    #[test]
    fn spp3_bounded_finite_no_alloc(
        seed in any::<u64>(), n in 1usize..=8, blk in block(), rate in rate(), ff in ff(),
        hot in any::<bool>(), len in 1usize..40_000, amp in 0.01f32..4.0, h in hostility(), sl in any::<u64>(),
    ) {
        let p = spp3_params(seed, n, blk, rate, ff, hot);
        let mut u = spp3::Unit::new(p);
        prop_assert!(!u.is_failsafe());
        let x = signal(seed ^ 1, n, len, amp, h);
        let s = synth_sense(&x, 2 * n, 37);
        let mut r = Rng(sl ^ 3);
        let rr = render(&mut u, &x, Some(&s), &slices(sl), &mut |_| sense_mode(&mut r), false);
        check_bound(&x, &rr.out, 0, 1.0, "spp3");
        check_state_finite(&rr.tele[0].1, n, "spp3");
    }

    #[test]
    fn atsp_bounded_finite_no_alloc(
        seed in any::<u64>(), n in 1usize..=8, blk in block(), rate in rate(), hot in any::<bool>(),
        len in 1usize..40_000, amp in 0.01f32..4.0, h in hostility(), sl in any::<u64>(),
    ) {
        let p = atsp_params(seed, n, blk, rate, hot);
        let mut u = atsp::Unit::new(p);
        prop_assert!(!u.is_failsafe());
        let x = signal(seed ^ 1, n, len, amp, h);
        let rr = render(&mut u, &x, None, &slices(sl), &mut no_sense, false);
        check_bound(&x, &rr.out, 0, 1.0, "atsp");
        check_state_finite(&rr.tele[0].1, n, "atsp");
    }

    #[test]
    fn lsm2_bounded_finite_no_alloc(
        seed in any::<u64>(), n in 1usize..=8, rate in rate(), hot in any::<bool>(),
        len in 1usize..30_000, amp in 0.01f32..4.0, h in hostility(), sl in any::<u64>(),
    ) {
        let p = lsm2_params(seed, n, rate, hot);
        let mut u = lsm2::Unit::new(p);
        prop_assert!(!u.is_failsafe());
        let lat = u.latency_frames() as usize;
        let x = signal(seed ^ 1, n, len, amp, h);
        let rr = render(&mut u, &x, None, &slices(sl), &mut no_sense, false);
        check_bound(&x, &rr.out, lat, 1.0, "lsm2");
        check_state_finite(&rr.tele[0].1, n, "lsm2");
    }

    /// Geometry the parameters do not describe (wrong channel counts, short
    /// buffers, zero frames) never panics, never indexes past a buffer and
    /// never leaves louder than the fail-safe.
    #[test]
    fn mismatched_geometry_is_the_failsafe(
        seed in any::<u64>(), n in 1usize..=8, extra in 0usize..=3, fewer in 0usize..=3,
        frames in 0usize..2000, short in 0usize..2000, which in 0usize..3,
    ) {
        let mut unit: Box<dyn ProtectionUnit> = match which {
            0 => Box::new(spp3::Unit::new(spp3_params(seed, n, 512, 48_000, Ff::On, false))),
            1 => Box::new(atsp::Unit::new(atsp_params(seed, n, 512, 48_000, false))),
            _ => Box::new(lsm2::Unit::new(lsm2_params(seed, n, 48_000, false))),
        };
        let nin = (n + extra).saturating_sub(fewer).max(1);
        let nout = (n + extra / 2).saturating_sub(fewer / 2).max(1);
        let mut r = Rng(seed);
        let inputs: Vec<Vec<f32>> = (0..nin).map(|_| {
            let l = if r.chance(0.5) { frames } else { short };
            (0..l).map(|_| r.uni(-1.0, 1.0)).collect()
        }).collect();
        let mut outputs: Vec<Vec<f32>> = (0..nout).map(|_| vec![7.0; if r.chance(0.5) { frames } else { short }]).collect();
        let geometry_ok = nin == n && nout == n
            && inputs.iter().all(|b| b.len() >= frames) && outputs.iter().all(|b| b.len() >= frames);
        let ins: Vec<&[f32]> = inputs.iter().map(|b| &b[..]).collect();
        let mut outs: Vec<&mut [f32]> = outputs.iter_mut().map(|b| &mut b[..]).collect();
        let before = allocs();
        let h = unit.process(Io { frames, input: &ins, sense: &[], output: &mut outs });
        prop_assert_eq!(allocs(), before);
        drop(outs);
        if !geometry_ok {
            prop_assert!(h.is_failsafe() && h.0 & Health::CHANNEL_MISMATCH != 0, "{:?}", h);
            for (c, o) in outputs.iter().enumerate() {
                for (k, &v) in o.iter().take(frames).enumerate() {
                    let i = inputs.get(c).and_then(|b| b.get(k)).copied().unwrap_or(0.0);
                    prop_assert!(v.abs() <= (i * FAILSAFE_GAIN).abs(), "ch{} [{}] {} from {}", c, k, v, i);
                }
                // frames past `frames` are never touched
                prop_assert!(o.iter().skip(frames).all(|&v| v == 7.0));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Property 3: slice invariance and reset.

proptest! {
    #![proptest_config(cfg(32))]

    #[test]
    fn spp3_slice_invariant(
        seed in any::<u64>(), n in 1usize..=8, blk in block(), rate in rate(), ff in ff(),
        len in 1usize..30_000, amp in 0.01f32..4.0, h in finite_hostility(), a in any::<u64>(), b in any::<u64>(),
    ) {
        let p = spp3_params(seed, n, blk, rate, ff, true);
        let x = signal(seed ^ 1, n, len, amp, h);
        // sense dropouts defined per frame (NaN holes, garbage), so both
        // slicings see the same bus
        let mut s = synth_sense(&x, 2 * n, 300);
        let mut r = Rng(seed ^ 9);
        for v in s.iter_mut() {
            for k in v.iter_mut() {
                if r.chance(0.001) { *k = special(&mut r); }
            }
        }
        let mut u1 = spp3::Unit::new(p);
        let mut u2 = spp3::Unit::new(p);
        let r1 = render(&mut u1, &x, Some(&s), &slices(a), &mut no_sense, false);
        let r2 = render(&mut u2, &x, Some(&s), &slices(b), &mut no_sense, false);
        prop_assert!(bits(&r1.out) == bits(&r2.out), "audio differs between slicings");
        prop_assert_eq!(tele_bits(&r1.tele[0].1), tele_bits(&r2.tele[0].1));
        for c in 0..n {
            prop_assert_eq!(u1.sense_power(c).to_bits(), u2.sense_power(c).to_bits());
        }
        prop_assert_eq!(u1.sense_delay(), u2.sense_delay());
        // reset returns the unit to exactly the state `new` builds
        u1.reset();
        let r3 = render(&mut u1, &x, Some(&s), &slices(b), &mut no_sense, false);
        prop_assert!(bits(&r3.out) == bits(&r2.out), "audio differs after reset");
        prop_assert_eq!(tele_bits(&r3.tele[0].1), tele_bits(&r2.tele[0].1));
    }

    #[test]
    fn atsp_slice_invariant(
        seed in any::<u64>(), n in 1usize..=8, blk in block(), rate in rate(),
        len in 1usize..30_000, amp in 0.01f32..4.0, h in finite_hostility(), a in any::<u64>(), b in any::<u64>(),
    ) {
        let p = atsp_params(seed, n, blk, rate, true);
        let x = signal(seed ^ 1, n, len, amp, h);
        let mut u1 = atsp::Unit::new(p);
        let r1 = render(&mut u1, &x, None, &slices(a), &mut no_sense, false);
        let r2 = render(&mut atsp::Unit::new(p), &x, None, &slices(b), &mut no_sense, false);
        prop_assert!(bits(&r1.out) == bits(&r2.out), "audio differs between slicings");
        prop_assert_eq!(tele_bits(&r1.tele[0].1), tele_bits(&r2.tele[0].1));
        u1.reset();
        let r3 = render(&mut u1, &x, None, &slices(b), &mut no_sense, false);
        prop_assert!(bits(&r3.out) == bits(&r2.out), "audio differs after reset");
        prop_assert_eq!(tele_bits(&r3.tele[0].1), tele_bits(&r2.tele[0].1));
    }

    #[test]
    fn lsm2_slice_invariant(
        seed in any::<u64>(), n in 1usize..=8, rate in rate(),
        len in 1usize..20_000, amp in 0.01f32..4.0, h in hostility(), a in any::<u64>(), b in any::<u64>(),
    ) {
        let p = lsm2_params(seed, n, rate, true);
        let x = signal(seed ^ 1, n, len, amp, h);
        let mut u1 = lsm2::Unit::new(p);
        let r1 = render(&mut u1, &x, None, &slices(a), &mut no_sense, false);
        let r2 = render(&mut lsm2::Unit::new(p), &x, None, &slices(b), &mut no_sense, false);
        prop_assert!(bits(&r1.out) == bits(&r2.out), "audio differs between slicings");
        prop_assert_eq!(tele_bits(&r1.tele[0].1), tele_bits(&r2.tele[0].1));
        u1.reset();
        let r3 = render(&mut u1, &x, None, &slices(b), &mut no_sense, false);
        prop_assert!(bits(&r3.out) == bits(&r2.out), "audio differs after reset");
        prop_assert_eq!(tele_bits(&r3.tele[0].1), tele_bits(&r2.tele[0].1));
    }
}

// ---------------------------------------------------------------------------
// Property 4: the hard path.

/// Render one block per call and check that every block that follows a block
/// boundary at which a group member's coil temperature exceeded its hard
/// limit leaves at least 20 dB down on every channel of the group. Returns
/// how many blocks were under the hard path.
fn check_hard_path(
    unit: &mut dyn ProtectionUnit,
    x: &[Vec<f32>],
    block: usize,
    // per host channel: (group id, hard limit)
    chans: &[(usize, f64)],
) -> usize {
    let n = x.len();
    let len = x[0].len();
    let mut out = vec![vec![0.0f32; block]; n];
    let mut t = Telemetry::new();
    let mut hard_groups: Vec<usize> = Vec::new();
    let mut hard_blocks = 0;
    let mut done = 0;
    while done + block <= len {
        let ins: Vec<&[f32]> = x.iter().map(|b| &b[done..done + block]).collect();
        let mut outs: Vec<&mut [f32]> = out.iter_mut().map(|b| &mut b[..]).collect();
        let before = allocs();
        unit.process(Io { frames: block, input: &ins, sense: &[], output: &mut outs });
        unit.telemetry(&mut t);
        assert_eq!(allocs(), before);
        drop(outs);
        // the block just rendered was decided at the previous boundary
        for (c, &(g, _)) in chans.iter().enumerate() {
            if !hard_groups.contains(&g) {
                continue;
            }
            for k in 0..block {
                let i = x[c][done + k];
                let lim = if i.is_finite() { (i * FAILSAFE_GAIN).abs() } else { 0.0 };
                assert!(
                    out[c][k].abs() <= lim,
                    "hard path missed: ch{c} frame {} out {} in {} (group {g} over its hard limit)",
                    done + k,
                    out[c][k],
                    i
                );
            }
        }
        if !hard_groups.is_empty() {
            hard_blocks += 1;
        }
        hard_groups.clear();
        for (c, &(g, limit)) in chans.iter().enumerate() {
            let temp = t.channels[c].coil_temp_c as f64;
            // telemetry is f32; skip a boundary the rounding could decide
            // either way
            if temp > limit + 1e-4 * limit.abs().max(1.0) && !hard_groups.contains(&g) {
                hard_groups.push(g);
            }
        }
        done += block;
    }
    hard_blocks
}

proptest! {
    #![proptest_config(cfg(32))]

    #[test]
    fn spp3_hard_path_cuts_20db_within_one_block(
        seed in any::<u64>(), n in 1usize..=8, blk in prop_oneof![Just(512u32), Just(64), 16u32..=2048],
        ff in ff(), secs in 1.0f64..8.0, amp in 0.5f32..4.0, h in hostility(),
    ) {
        let rate = 48_000;
        let mut p = spp3_params(seed, n, blk, rate, ff, true);
        // short coil constants and low limits so the coil reaches the hard
        // limit within the run
        for c in 0..n {
            p.ch[c].tau_tvc = p.ch[c].tau_tvc.min(0.2);
            p.ch[c].temperature_limit = p.thermal_ambient_temp + 5.0 + (p.ch[c].temperature_limit - p.thermal_ambient_temp) / 8.0;
            p.ch[c].hard_temp_limit_headroom = p.ch[c].hard_temp_limit_headroom.abs() / 4.0;
        }
        let len = (secs * rate as f64) as usize;
        let x = signal(seed ^ 1, n, len, amp, h);
        let mut chans = vec![(0, 0.0); n];
        for c in &p.ch[..n] {
            chans[c.source as usize] = (c.group as usize, c.temperature_limit as f64 + c.hard_temp_limit_headroom as f64);
        }
        let mut u = spp3::Unit::new(p);
        prop_assert!(!u.is_failsafe());
        check_hard_path(&mut u, &x, blk as usize, &chans);
    }

    #[test]
    fn atsp_hard_path_cuts_20db_within_one_block(
        seed in any::<u64>(), n in 1usize..=8, blk in prop_oneof![Just(512u32), Just(64), 16u32..=2048],
        secs in 1.0f64..8.0, amp in 0.5f32..4.0, h in hostility(),
    ) {
        let rate = 48_000;
        let mut p = atsp_params(seed, n, blk, rate, true);
        for s in p.slot.iter_mut() {
            s.tau_vc = s.tau_vc.min(0.2);
            s.temperature_limit = s.ambient + 5.0 + (s.temperature_limit - s.ambient) / 8.0;
            s.hard_limit_headroom = s.hard_limit_headroom.abs() / 4.0;
        }
        let len = (secs * rate as f64) as usize;
        let x = signal(seed ^ 1, n, len, amp, h);
        // atsp's grouping: one group when linked globally, one per linked
        // slot, else one per channel
        let mut chans = vec![(0, 0.0); n];
        for (si, s) in p.slot.iter().enumerate() {
            for (c, ch) in chans.iter_mut().enumerate() {
                if s.channel_mask & (1 << c) != 0 {
                    let g = if p.gain_link_all != 0 { 0 } else if s.gain_link != 0 { 100 + si } else { 200 + c };
                    *ch = (g, s.temperature_limit as f64 + s.hard_limit_headroom as f64);
                }
            }
        }
        let mut u = atsp::Unit::new(p);
        prop_assert!(!u.is_failsafe());
        check_hard_path(&mut u, &x, blk as usize, &chans);
    }

    /// lsm2 has no separate hard path; its temperature axis takes the full
    /// `MaxTemperatureAttenuation` once the predicted coil temperature reaches
    /// the soft limit, through `AttackTime`. Check the gain is at most that
    /// floor (plus what the attack smoother has left) one controller interval
    /// plus ten attack time constants after every update that engaged it.
    #[test]
    fn lsm2_temperature_path_reaches_its_floor(
        seed in any::<u64>(), n in 1usize..=4, secs in 1.0f64..6.0, amp in 0.5f32..2.0,
    ) {
        let rate = 48_000u32;
        let mut p = lsm2_params(seed, n, rate, true);
        p.control_temperature = 1;
        p.enable_mitigation = 1;
        let len = (secs * rate as f64) as usize;
        let x = signal(seed ^ 1, n, len, amp, Hostility::Clean);
        let mut u = lsm2::Unit::new(p);
        let settle = 256 + (10.0 * p.attack_time_ms as f64 * 1e-3 * rate as f64) as usize;
        let rr = render(&mut u, &x, None, &[256], &mut no_sense, true);
        // per host channel: (frame engaged, floor)
        let mut engaged: Vec<Option<(usize, f32)>> = vec![None; n];
        for (done, t) in &rr.tele {
            for c in &p.ch[..n] {
                let s = &t.channels[c.source as usize];
                let floor = 10f32.powf(-c.max_temperature_attenuation_db / 20.0);
                let margin = s.coil_temp_c + c.temperature_headroom - c.temperature_soft_limit;
                let e = &mut engaged[c.source as usize];
                // f32 telemetry: a boundary within rounding is undecided
                if margin.abs() < 1e-3 * c.temperature_soft_limit.abs().max(1.0) {
                    *e = None;
                    continue;
                }
                match (*e, margin > 0.0) {
                    (None, true) => *e = Some((*done, floor)),
                    (Some(_), false) => *e = None,
                    (Some((at, f)), true) if *done >= at + settle => {
                        let tol = f * 1e-4 + (1.0 - f) * 1e-4 + 1e-6;
                        prop_assert!(s.gain <= f + tol, "lsm2 temperature floor {} not reached: gain {}", f, s.gain);
                    }
                    _ => {}
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Property 5: parameter blobs.

fn probe(unit: &mut dyn ProtectionUnit, n: usize, latency: usize, gain: f32, what: &str, h: Hostility) {
    let x = signal(0x5eed, n, 6000, 1.5, h);
    let s = synth_sense(&x, 2 * n, 0);
    let rr = render(unit, &x, Some(&s), &[1, 511, 4096, 96], &mut no_sense, false);
    check_bound(&x, &rr.out, latency, gain, what);
}

/// Decode `bytes` as every unit kind and as the placeholder; whatever comes
/// out must be usable (bounded, finite) and a rejection is the fail-safe.
fn load_and_probe(bytes: &[u8]) {
    let _ = blob::Header::parse(bytes);
    let _ = placeholder::Params::decode(bytes);
    let n_hint = blob::Header::parse(bytes).map_or(2, |(h, _)| (h.channels as usize).clamp(1, MAX_CHANNELS));
    match spp3::Params::from_blob(bytes) {
        Ok(p) => {
            let mut u = spp3::Unit::new(p);
            let g = if u.is_failsafe() { FAILSAFE_GAIN } else { 1.0 };
            probe(&mut u, p.channels as usize, 0, g, "spp3 blob", Hostility::Wild);
        }
        // Clean input only: FailSafe passes NaN/Inf through (FINDING 2).
        Err(_) => {
            let mut u = FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS);
            probe(&mut u, n_hint, 0, FAILSAFE_GAIN, "fail-safe", Hostility::Clean)
        }
    }
    if let Ok(p) = lsm2::Params::from_blob(bytes) {
        let mut u = lsm2::Unit::new(p);
        let (g, lat) = if u.is_failsafe() { (FAILSAFE_GAIN, 0) } else { (1.0, u.latency_frames() as usize) };
        probe(&mut u, p.channels as usize, lat, g, "lsm2 blob", Hostility::Wild);
    }
    if let Ok(p) = atsp::Params::from_blob(bytes) {
        let mut u = atsp::Unit::new(p);
        let g = if u.is_failsafe() { FAILSAFE_GAIN } else { 1.0 };
        probe(&mut u, p.channels as usize, 0, g, "atsp blob", Hostility::Wild);
    }
}

/// Hostile replacements for one 4-byte field.
fn hostile_word(r: &mut Rng) -> u32 {
    match r.below(16) {
        0 => f32::NAN.to_bits(),
        1 => f32::INFINITY.to_bits(),
        2 => f32::NEG_INFINITY.to_bits(),
        3 => 0,
        4 => (-0.0f32).to_bits(),
        5 => f32::MAX.to_bits(),
        6 => f32::MIN.to_bits(),
        7 => f32::MIN_POSITIVE.to_bits(),
        8 => 1, // smallest subnormal as f32, 1 as u32
        9 => u32::MAX,
        10 => 0x8000_0000,
        11 => 16_385,
        12 => (-1.0f32).to_bits(),
        13 => (1e30f32).to_bits(),
        14 => r.below(9) as u32,
        _ => r.next() as u32,
    }
}

fn valid_blob(seed: u64, which: usize, n: usize) -> Vec<u8> {
    match which {
        0 => {
            let p = spp3_params(seed, n, 512, 48_000, Ff::Mixed, false);
            blob::build(UnitKind::Spp3, n as u32, 2 * n as u32, 48_000, p.as_bytes())
        }
        1 => {
            let p = lsm2_params(seed, n, 48_000, false);
            blob::build(UnitKind::Lsm2, n as u32, 0, 48_000, p.as_bytes())
        }
        _ => {
            let p = atsp_params(seed, n, 512, 48_000, false);
            blob::build(UnitKind::Atsp, n as u32, 0, 48_000, p.as_bytes())
        }
    }
}

fn fix_crc(b: &mut [u8]) {
    if b.len() >= blob::HEADER_LEN {
        let crc = blob::crc32(&b[blob::HEADER_LEN..]);
        b[32..36].copy_from_slice(&crc.to_le_bytes());
    }
}

proptest! {
    #![proptest_config(cfg(256))]

    #[test]
    fn arbitrary_bytes_never_panic(bytes in proptest::collection::vec(any::<u8>(), 0..1200)) {
        load_and_probe(&bytes);
    }

    #[test]
    fn mutated_blobs_are_rejected_or_safe(
        seed in any::<u64>(), which in 0usize..3, n in 1usize..=8, mutations in 1usize..6, fix in any::<bool>(),
    ) {
        let mut b = valid_blob(seed, which, n);
        let mut r = Rng(seed ^ 0xb10b);
        for _ in 0..mutations {
            match r.below(6) {
                0 => { let l = r.below(b.len() + 1); b.truncate(l); }
                1 if !b.is_empty() => { let i = r.below(b.len()); b[i] ^= 1 << r.below(8); }
                2 | 3 => {
                    // a hostile value in a payload or header field
                    if b.len() >= 4 {
                        let i = r.below(b.len() / 4) * 4;
                        b[i..i + 4].copy_from_slice(&hostile_word(&mut r).to_le_bytes());
                    }
                }
                4 => { let extra = r.below(64); b.extend((0..extra).map(|_| r.next() as u8)); }
                _ => {
                    // a header field out of range
                    if b.len() >= blob::HEADER_LEN {
                        let f = 8 + 4 * r.below(7);
                        b[f..f + 4].copy_from_slice(&hostile_word(&mut r).to_le_bytes());
                    }
                }
            }
        }
        if fix { fix_crc(&mut b); }
        load_and_probe(&b);
    }

    /// A wrong CRC is always rejected, whatever else is right.
    #[test]
    fn wrong_crc_is_always_rejected(seed in any::<u64>(), which in 0usize..3, n in 1usize..=8, flip in any::<u32>()) {
        let mut b = valid_blob(seed, which, n);
        let flip = flip.max(1);
        let crc = u32::from_le_bytes([b[32], b[33], b[34], b[35]]) ^ flip;
        b[32..36].copy_from_slice(&crc.to_le_bytes());
        prop_assert_eq!(blob::Header::parse(&b).err(), Some(blob::BlobError::Crc));
        prop_assert!(spp3::Params::from_blob(&b).is_err());
        prop_assert!(lsm2::Params::from_blob(&b).is_err());
        prop_assert!(atsp::Params::from_blob(&b).is_err());
    }
}

// ---------------------------------------------------------------------------
// Property 6: sense dropouts never raise the gain.

proptest! {
    #![proptest_config(cfg(48))]

    #[test]
    fn spp3_sense_dropouts_never_raise_the_gain(
        seed in any::<u64>(), n in 1usize..=8, blk in block(), ff in ff(),
        len in 1usize..30_000, amp in 0.1f32..4.0, sl in any::<u64>(), delay in 0usize..3000,
    ) {
        let mut p = spp3_params(seed, n, blk, 48_000, ff, true);
        // every channel uses telemetry, with valid sense ports
        for c in 0..n {
            p.ch[c].ignore_telemetry = 0;
            p.ch[c].sense_i = 2 * c as u32;
            p.ch[c].sense_v = 2 * c as u32 + 1;
        }
        let x = signal(seed ^ 1, n, len, amp, Hostility::Clean);
        let s = synth_sense(&x, 2 * n, delay);
        let cuts = slices(sl);
        let valid = render(&mut spp3::Unit::new(p), &x, Some(&s), &cuts, &mut no_sense, false);
        let open = render(&mut spp3::Unit::new(p), &x, None, &cuts, &mut no_sense, false);
        let mut r = Rng(sl ^ 77);
        let mut modes = Vec::new();
        let mut u = spp3::Unit::new(p);
        let drop = render(&mut u, &x, Some(&s), &cuts, &mut |_| { let m = sense_mode(&mut r); modes.push(m); m }, false);
        for c in 0..n {
            for k in 0..len {
                let d = drop.out[c][k].abs();
                prop_assert!(d <= valid.out[c][k].abs() && d <= open.out[c][k].abs(),
                    "sense dropout raised ch{} [{}]: {} vs valid {} open {}", c, k, d, valid.out[c][k], open.out[c][k]);
            }
        }
        // a block whose every frame came without usable sense is reported
        // open loop (health reflects the last completed block)
        let blk = blk as usize;
        let mut d0 = 0;
        for (call, (h, m)) in drop.health.iter().zip(&modes).enumerate() {
            let d1 = (d0 + cuts[call % cuts.len()]).min(len);
            let last_end = d1 / blk * blk;
            let unusable = matches!(m, Sense::Empty | Sense::Missing | Sense::Short)
                || matches!(m, Sense::Fill(f) if !f.is_finite() || f.abs() > 1e20);
            if unusable && last_end >= blk && last_end - blk >= d0 {
                prop_assert!(h.0 & Health::OPEN_LOOP != 0, "{:?} with {:?}", h, m);
            }
            d0 = d1;
        }
        // implausible sense never makes the supervisor's meter non-finite
        // other than as NaN ("no measurement")
        for c in 0..n {
            let v = u.sense_power(c);
            prop_assert!(v.is_nan() || v.is_finite(), "sense power {}", v);
        }
    }

    /// Units without sense ignore whatever the bus carries.
    #[test]
    fn sense_free_units_ignore_the_bus(
        seed in any::<u64>(), n in 1usize..=8, which in any::<bool>(), len in 1usize..20_000, sl in any::<u64>(),
    ) {
        let x = signal(seed ^ 1, n, len, 1.5, Hostility::Specials);
        let s = synth_sense(&x, 2 * n + 1, 5);
        let cuts = slices(sl);
        let mut r = Rng(sl);
        let (a, b) = if which {
            let p = lsm2_params(seed, n, 48_000, true);
            (render(&mut lsm2::Unit::new(p), &x, None, &cuts, &mut no_sense, false),
             render(&mut lsm2::Unit::new(p), &x, Some(&s), &cuts, &mut |_| sense_mode(&mut r), false))
        } else {
            let p = atsp_params(seed, n, 512, 48_000, true);
            (render(&mut atsp::Unit::new(p), &x, None, &cuts, &mut no_sense, false),
             render(&mut atsp::Unit::new(p), &x, Some(&s), &cuts, &mut |_| sense_mode(&mut r), false))
        };
        prop_assert!(bits(&a.out) == bits(&b.out));
    }
}

// ---------------------------------------------------------------------------
// Deterministic anchors: the properties above mean something only if the
// generators actually reach the paths they test.

#[test]
fn hard_path_anchor_is_reached() {
    let mut hits = 0;
    for seed in 0..12u64 {
        let n = 2;
        let mut p = spp3_params(seed, n, 512, 48_000, Ff::Off, true);
        for c in 0..n {
            p.ch[c].tau_tvc = 0.05;
            p.ch[c].temperature_limit = p.thermal_ambient_temp + 10.0;
            p.ch[c].hard_temp_limit_headroom = 2.0;
        }
        let x = signal(seed, n, 48_000 * 4, 2.0, Hostility::Clean);
        let mut chans = vec![(0, 0.0); n];
        for c in &p.ch[..n] {
            chans[c.source as usize] =
                (c.group as usize, c.temperature_limit as f64 + c.hard_temp_limit_headroom as f64);
        }
        hits += check_hard_path(&mut spp3::Unit::new(p), &x, 512, &chans);
    }
    assert!(hits > 100, "the hard-path property never saw the hard path ({hits} blocks)");
}

// ---------------------------------------------------------------------------
// Regression tests for defects found by the tests above (lane B to fix).

/// Minimal spp3 set for the regressions: one channel, feed-forward off (the
/// specified `ThermalFFSpeedupFactor = 0` behaviour: only the hard path acts).
fn spp3_minimal(ff: f32) -> spp3::Params {
    let mut p = spp3::Params {
        channels: 1,
        sample_rate: 48_000,
        block_frames: 512,
        thermal_ambient_temp: 40.0,
        thermal_gain_attack_time_s: 0.5,
        thermal_gain_release_time_s: 2.0,
        thermal_protection_enabled: 1,
        ..spp3::Params::default()
    };
    p.ch[0] = spp3::ChannelParams {
        t_sett_vc: 20.0,
        t_sett_mg: 10.0,
        tau_tvc: 0.4,
        tau_tmg: 3.0,
        temperature_limit: 120.0,
        hard_temp_limit_headroom: 8.0,
        reb_ref: 4.0,
        rshunt: 0.2,
        thermal_ff_speedup_factor: ff,
        ag: 10.0,
        ..Default::default()
    };
    p
}

fn one_block(u: &mut dyn ProtectionUnit, x: f32, first: Option<f32>) -> f32 {
    let mut i = vec![x; 512];
    if let Some(v) = first {
        i[0] = v;
    }
    let mut o = vec![0.0f32; 512];
    u.process(Io { frames: 512, input: &[&i], sense: &[], output: &mut [&mut o] });
    o[511] / x
}

/// FINDING 1: a single large finite sample (|x| > ~1.8e19, so x^2 overflows
/// the f32 block accumulator `LaneAcc`) drives the spp3 thermal state to
/// +Inf, then to NaN on the next block (`inf - inf`). `NaN > hard_limit` is
/// false, so the hard path is disarmed for good: with the feed-forward off,
/// full-scale drive then passes at unity forever while the model reports NaN
/// (until `reset`). With the feed-forward on, `NaN.max(0.0)` makes the target
/// 0 and the unit mutes forever instead. atsp has the same accumulator.
#[test]
#[ignore = "FINDING 1 (lane B): huge finite input poisons the thermal state with NaN"]
fn spp3_huge_finite_sample_must_not_disarm_the_hard_path() {
    // control: full-scale drive reaches the hard path within 4 s
    let mut fresh = spp3::Unit::new(spp3_minimal(0.0));
    let mut t = Telemetry::new();
    // blocks under the hard path, after the first two (the poisoned unit's
    // +Inf block still trips it once before the state turns NaN)
    let engaged = |u: &mut spp3::Unit| {
        (0..400).map(|_| one_block(u, 1.0, None)).skip(2).filter(|&g| g <= FAILSAFE_GAIN * 1.0001).count()
    };
    assert!(engaged(&mut fresh) > 0, "control: the hard path engages on a fresh unit");
    // one block containing one 2e19 sample, then the same full-scale drive
    let mut u = spp3::Unit::new(spp3_minimal(0.0));
    one_block(&mut u, 0.0, Some(2e19));
    let hard_blocks = engaged(&mut u);
    u.telemetry(&mut t);
    assert!(
        hard_blocks > 0,
        "hard path disarmed: 4 s of full-scale drive at unity gain, coil temperature {}",
        t.channels[0].coil_temp_c
    );
    assert!(t.channels[0].coil_temp_c.is_finite(), "coil temperature is {}", t.channels[0].coil_temp_c);
}

#[test]
#[ignore = "FINDING 1 (lane B): huge finite input poisons the thermal state with NaN"]
fn atsp_huge_finite_sample_must_not_poison_the_model() {
    let s = atsp::Slot {
        channel_mask: 1,
        gain_link: 1,
        amplifier_sensitivity: 10.0,
        rdc: 8.0,
        rth_vc: 30.0,
        tau_vc: 1.0,
        rth_mg: 40.0,
        tau_mg: 20.0,
        ambient: 40.0,
        temperature_limit: 110.0,
        hard_limit_headroom: 8.0,
        attack_ms: 500.0,
        release_ms: 5000.0,
    };
    let p = atsp::Params {
        channels: 1,
        sample_rate: 48_000,
        block_frames: 512,
        bypass: 0,
        gain_link_all: 0,
        slot: [s, Default::default(), Default::default()],
    };
    let mut u = atsp::Unit::new(p);
    one_block(&mut u, 0.0, Some(2e19));
    for _ in 0..4 {
        one_block(&mut u, 0.0, None);
    }
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    assert!(t.channels[0].coil_temp_c.is_finite(), "coil temperature is {}", t.channels[0].coil_temp_c);
    // and the unit recovers from silence instead of muting forever
    let mut g = 0.0;
    for _ in 0..2000 {
        g = one_block(&mut u, 1e-3, None);
    }
    assert!(g > 0.5, "stuck muted after one huge sample: gain {g}");
}

/// FINDING 5: spp3 and atsp apply the non-finite ("meter failure") 20 dB
/// to every frame of the *host segment* that holds the bad sample, so where
/// the host happened to split its buffers decides which good frames are cut,
/// and therefore the protected drive the thermal model integrates. Output and
/// state then differ between slicings (property 3). The cut is on the safe
/// side, but the trajectory is no longer reproducible against the rig.
#[test]
#[ignore = "FINDING 5 (lane B): the non-finite fail-safe region depends on host slicing"]
fn non_finite_failsafe_region_is_slice_invariant() {
    let mut x = vec![0.5f32; 512];
    x[100] = f32::NAN;
    let x = vec![x];
    for (name, mk) in [
        (
            "spp3",
            Box::new(|| Box::new(spp3::Unit::new(spp3_minimal(1.0))) as Box<dyn ProtectionUnit>)
                as Box<dyn Fn() -> Box<dyn ProtectionUnit>>,
        ),
        (
            "atsp",
            Box::new(|| {
                let mut p = atsp_params(5, 1, 512, 48_000, false);
                p.slot[0].channel_mask = 1;
                (p.slot[1].channel_mask, p.slot[2].channel_mask) = (0, 0);
                Box::new(atsp::Unit::new(p)) as Box<dyn ProtectionUnit>
            }),
        ),
    ] {
        let whole = render(mk().as_mut(), &x, None, &[512], &mut no_sense, false);
        let split = render(mk().as_mut(), &x, None, &[64], &mut no_sense, false);
        assert_eq!(whole.out[0][0], split.out[0][0], "{name}: frame 0 (far from the NaN) depends on the slicing");
        assert!(bits(&whole.out) == bits(&split.out), "{name}: output depends on the slicing");
    }
}

/// FINDING 2: `FailSafe::process` (the unit `sgv2-lv2::load_unit` installs
/// for any unusable blob) is not as hardened as spp3/lsm2/atsp's own fault
/// path, `failsafe_io`: it slices `out[..frames]` and `input[..frames]`
/// unchecked, so a host buffer shorter than `Io::frames` panics (an abort
/// inside PipeWire with `panic = "abort"`), and it does not sanitize, so a
/// NaN or +-Inf input sample leaves as NaN or +-Inf (Inf * 0.1 = Inf: full
/// scale at the amplifier).
#[test]
#[ignore = "FINDING 2 (lane B): FailSafe::process panics on a short host buffer"]
fn failsafe_unit_never_panics_on_short_buffers() {
    let r = std::panic::catch_unwind(|| {
        let x = [0.5f32; 8];
        let mut o = [0.0f32; 8];
        let mut u = FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS);
        u.process(Io { frames: 16, input: &[&x], sense: &[], output: &mut [&mut o] });
    });
    assert!(r.is_ok(), "FailSafe::process panicked on a buffer shorter than frames");
}

#[test]
#[ignore = "FINDING 2 (lane B): FailSafe::process passes NaN/Inf through"]
fn failsafe_unit_output_is_finite() {
    let x = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 0.5];
    let mut o = [0.0f32; 4];
    let mut u = FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS);
    u.process(Io { frames: 4, input: &[&x], sense: &[], output: &mut [&mut o] });
    assert!(o.iter().all(|v| v.is_finite()), "fail-safe output {o:?}");
}

/// FINDING 3: blobs that validate and load as a real unit, yet protect
/// nothing (property 5's "never pass-through"). spp3 refuses
/// `ThermalProtectionEnabled = 0` and atsp refuses `Bypass`, but lsm2 accepts
/// `EnableMitigation = 0`, and all five `Control*` flags off, and the units
/// accept limits no speaker has (a temperature limit of `f32::MAX`). Whether
/// these are refused (fail-safe) or allowed as Apple's own semantics is a
/// lane B / spec decision; until then these tests document the gap.
#[test]
#[ignore = "FINDING 3 (decision for lane B): accepted blobs that disable protection"]
fn hostile_but_valid_blobs_do_not_pass_through() {
    let x = vec![vec![1.0f32; 48_000 * 10]; 1];
    let passes = |u: &mut dyn ProtectionUnit, lat: usize| {
        let rr = render(u, &x, None, &[512], &mut no_sense, false);
        rr.out[0][lat + 48_000 * 9..].iter().all(|&v| v == 1.0)
    };
    let mut failures = Vec::new();
    // lsm2, mitigation off
    let mut p = lsm2_params(1, 1, 48_000, true);
    p.control_temperature = 1;
    p.enable_mitigation = 0;
    let b = blob::build(UnitKind::Lsm2, 1, 0, 48_000, p.as_bytes());
    let mut u = lsm2::Unit::new(lsm2::Params::from_blob(&b).unwrap());
    let lat = u.latency_frames() as usize;
    if !u.is_failsafe() && passes(&mut u, lat) {
        failures.push("lsm2 EnableMitigation=0 loads and passes full scale at unity");
    }
    // lsm2, every axis off
    let mut p = lsm2_params(1, 1, 48_000, true);
    (p.control_sample_peak, p.control_displacement, p.control_velocity) = (0, 0, 0);
    (p.control_power, p.control_temperature) = (0, 0);
    let mut u = lsm2::Unit::new(p);
    let lat = u.latency_frames() as usize;
    if !u.is_failsafe() && passes(&mut u, lat) {
        failures.push("lsm2 with every Control* flag off loads and passes full scale at unity");
    }
    // spp3, a limit no speaker has
    let mut p = spp3_minimal(1.0);
    p.ch[0].temperature_limit = f32::MAX;
    let mut u = spp3::Unit::new(p);
    if !u.is_failsafe() && passes(&mut u, 0) {
        failures.push("spp3 TemperatureLimit=f32::MAX loads and passes full scale at unity");
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// FINDING 4: lsm2 parses and validates `TemperatureHardLimit` but never
/// uses it: there is no hard path. With `MaxTemperatureAttenuation = 0 dB`
/// (accepted) the coil runs past the hard limit at unity gain.
#[test]
#[ignore = "FINDING 4 (decision for lane B): lsm2 never enforces TemperatureHardLimit"]
fn lsm2_enforces_the_hard_temperature_limit() {
    let mut p = lsm2_params(3, 1, 48_000, true);
    (p.control_sample_peak, p.control_displacement, p.control_velocity, p.control_power) = (0, 0, 0, 0);
    p.control_temperature = 1;
    p.ch[0].max_temperature_attenuation_db = 0.0;
    p.ch[0].temperature_soft_limit = p.ch[0].ambient + 5.0;
    p.ch[0].temperature_hard_limit = p.ch[0].ambient + 10.0;
    let mut u = lsm2::Unit::new(p);
    assert!(!u.is_failsafe());
    let x = vec![signal(9, 1, 48_000 * 20, 1.0, Hostility::Clean)[0].iter().map(|v| v.signum()).collect::<Vec<_>>()];
    let rr = render(&mut u, &x, None, &[512], &mut no_sense, true);
    let over = rr.tele.iter().filter(|(_, t)| t.channels[0].coil_temp_c > p.ch[0].temperature_hard_limit).count();
    let (_, last) = rr.tele.last().unwrap();
    assert!(over > 0, "precondition: the drive must reach the hard limit");
    assert!(
        last.channels[0].gain <= FAILSAFE_GAIN,
        "coil {} C past the {} C hard limit for {over} calls at gain {}",
        last.channels[0].coil_temp_c,
        p.ch[0].temperature_hard_limit,
        last.channels[0].gain
    );
}

//! Adversarial property tests for the protection core (lane T).
//!
//! The core's one job is never to drive a speaker past what its model
//! allows, so these tests attack that from every side the host, the sense
//! bus and the parameter blob can reach:
//!
//! 1. **Output bound.** `|out[n]| <= |in[n - latency]|` for every unit (no
//!    unit has gain above 0 dB; the spp3 pilot is not generated yet, only its
//!    program headroom, so there is no pilot to allow for), and a non-finite
//!    input sample leaves as 0.
//! 2. **Finite output.** Any input, including NaN, +-Inf, subnormals and very
//!    large finite values, gives finite output and finite model state.
//! 3. **Slice invariance.** Output and state are bit-identical across random
//!    host slice sequences (1..=4096 frames), and `reset` returns a unit to
//!    exactly its initial state.
//! 4. **Hard path.** spp3: while the modelled coil is past the hard limit
//!    (latched until it is back below the limit), the group's target is at
//!    most the specified hard target and the audio follows it through the
//!    5 ms dezippers, checked sample by sample against an independent oracle
//!    ([`HardOracle`]); with the feed-forward on, sustained drive settles at
//!    or below the hard limit. atsp: once past the hard limit at a block
//!    boundary, the group's next block leaves at least 20 dB down. lsm2's
//!    soft temperature path reaches its floor.
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
//! Regression tests for the defects these tests found are at the end
//! (FINDING 1-12). Fixed ones run as ordinary tests; open ones are
//! `#[ignore]`d so the suite stays green until lane B fixes them: run them
//! with `cargo test --test adversarial -- --ignored`. The LV2-side checks
//! (null program port, engine-page gain) are in
//! `crates/sgv2-lv2/tests/adversarial_lv2.rs`.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

mod review_common;

use proptest::prelude::*;
use proptest::test_runner::Config;
use sgv2_core::blob::{self, Pod};
use sgv2_core::{atsp, lsm2, placeholder, spp3, FailSafe, HandOff, Health, Io, ProtectionUnit, Telemetry, UnitKind};
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
/// whole host segment, which is not slice invariant (FINDING 6).
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

/// Host slicings of one atsp family (SPEC-ATSP 11.3), whose output is
/// the same bit for bit: whole multiples of the 512-frame fragment
/// (`staged`), or fragments of 2 frames up to one tick, which never start
/// before the window their tick closes. One-frame fragments take the
/// one-frame dezipper path, a family of their own. A one-channel stream's
/// dezipper groups each call's frames by eight (SPEC-ATSP 11.4), so its
/// small family is whole groups of eight up to one tick.
fn atsp_family_slices(seed: u64, win: usize, staged: bool, mono: bool) -> Vec<usize> {
    let mut r = Rng(seed ^ 0xa75);
    let small = win.min(atsp::FRAGMENT);
    let staged = staged || (mono && small < 8);
    (0..64)
        .map(|_| {
            if staged {
                atsp::FRAGMENT * (1 + r.below(8))
            } else if mono {
                8 * (1 + r.below(small / 8))
            } else {
                2 + r.below(small - 1)
            }
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
        let ignore = r.chance(0.2);
        // closed loop with a synthetic calibration that `synth_sense`'s
        // 4.2-ohm V/I can satisfy (cap >= 0: FINDING 4)
        let cl = !ignore && r.chance(0.5);
        let pilot = r.chance(0.3);
        p.ch[c] = spp3::ChannelParams {
            source: s,
            group: r.below(4) as u32 * 7,
            ignore_telemetry: ignore as u32,
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
            cl_thermal_feedback_enabled: (cl || r.chance(0.3)) as u32,
            rampout: if r.chance(0.3) { r.uni(0.0, 0.5) } else { 0.0 },
            ag: if hot { r.log(5.0, 40.0) } else { r.log(1.0, 40.0) },
            imon_scale: if cl { 1.0 } else { r.uni(0.1, 20.0) },
            vmon_scale: if cl { 1.0 } else { r.uni(0.1, 20.0) },
            i_pwr_thres: if cl { r.log(1e-6, 1e-2) } else { r.uni(0.0, 1.0) },
            v_pwr_thres: if cl { r.log(1e-6, 1e-2) } else { r.uni(0.0, 1.0) },
            sense_i: if cl { 2 * c as u32 } else { r.below(2 * n + 2) as u32 },
            sense_v: if cl { 2 * c as u32 + 1 } else { r.below(2 * n + 2) as u32 },
            pilot_tone_enabled: pilot as u32,
            pilot_ampl_hi_db: if pilot { r.uni(-40.0, -1.0) } else { r.uni(-60.0, 10.0) },
            temp_sense_window_time_s: r.log(1e-3, 0.1),
            temp_sense_smoothing_tau_s: r.log(1e-3, 2.0),
            rdc_scale: r.uni(0.9, 1.1),
            reb_range_min: r.uni(1.0, 4.0),
            reb_range_max: r.uni(4.5, 10.0),
            cal_temp_range_min: 10.0,
            cal_temp_range_max: 40.0,
            a_t_20c: r.uni(0.003, 0.0045),
            a_t_35c: r.uni(0.003, 0.0045),
            tlim_err_decay_time_s: r.log(0.05, 10.0),
            tlim_adj_cap: r.uni(0.0, 40.0),
            cal_present: cl as u32,
            cal_r: r.uni(4.0, 4.5),
            cal_t: r.uni(15.0, 35.0),
            pilot_ampl_lo_db: -40.0,
            pilot_upper_thres: 90.0,
            pilot_lower_thres: 80.0,
            pilot_decay_time_s: 0.03,
            pilot_decay_time_stage2_s: 0.03,
            pilot_freq: 43.0664,
            pilot_enable_thres: 0.03,
            thermal_measurement_method: spp3::METHOD_PILOT_TONE,
        };
    }
    stable_spp3(&mut p);
    assert_eq!(p.validate(), Ok(()), "generator made an invalid spp3 set: {p:?}");
    p
}

/// Keep every spp3 thermal filter (coil, magnet, virtual coil) at a time
/// constant of at least one update: the linear-decay recurrence diverges
/// below half an update (FINDING 9).
fn stable_spp3(p: &mut spp3::Params) {
    // the accepted floor: one update and the documented 0.1 s minimum
    let dt = (spp3::THERMAL_FRAMES as f32 / p.sample_rate as f32).max(spp3::TAU_MIN_S);
    for c in p.ch[..p.channels as usize].iter_mut() {
        let f = c.thermal_ff_speedup_factor;
        let shortest = if f > 0.0 { f.min(1.0) } else { 1.0 };
        c.tau_tvc = c.tau_tvc.max(dt / shortest);
        c.tau_tmg = c.tau_tmg.max(dt);
    }
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
        low_band_release_time_ms: r.log(1.0, 2000.0),
        filterbank_latency_ms: r.uni(0.0, 5.0),
        speaker_response_lookahead_ms: r.uni(0.0, 3.0),
        power_time_scale_s: ts,
        power_hard_limit_w: [r.log(0.1, 50.0), r.log(0.1, 50.0), r.log(0.1, 50.0), r.log(0.1, 50.0)],
        power_soft_limit_pct: [100.0; 4],
        max_power_attenuation_db: r.uni(0.0, 20.0),
        low_rail_idle_power_w: r.uni(0.0, 1.0),
        low_rail_power_ratio: r.uni(0.5, 2.0),
        velocity_parameter_max: 64.0,
        other_power_features: 0,
        runtime_budget_present: r.chance(0.3) as u32,
        runtime_budget_w: [r.log(0.1, 50.0), r.log(0.1, 50.0), r.log(0.1, 50.0), r.log(0.1, 50.0)],
        ..lsm2::Params::default()
    };
    // refused otherwise: no thermal axis (a blob must protect), or a
    // look-ahead region (filterbank stage plus look-ahead) that is not whole
    // substeps inside one controller frame
    if p.control_power == 0 && p.control_temperature == 0 {
        p.control_temperature = 1;
    }
    let cadence = lsm2::controller_frames(rate);
    let sub = cadence / lsm2::SUBSTEPS;
    let od = sub * (1 + (r.uni(0.0, 6.99) as usize));
    let fb = (2 + (r.uni(0.0, 1.0) as f64 * (od - 2) as f64) as usize).min(od);
    p.filterbank_latency_ms = (fb as f64 * 1e3 / rate as f64) as f32;
    p.speaker_response_lookahead_ms = ((od - fb) as f64 * 1e3 / rate as f64) as f32;
    let nyq = rate as f64 / 2.0;
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
            disp_rdc: r.uni(2.0, 8.0),
            disp_bl: r.uni(0.3, 3.0),
            disp_mms_kg: r.log(5e-5, 5e-3),
            disp_rms: r.uni(0.05, 2.0),
            disp_kms: r.log(200.0, 5000.0),
            vel_rdc: r.uni(2.0, 8.0),
            vel_bl: r.uni(0.3, 3.0),
            vel_mms_kg: r.log(5e-5, 5e-3),
            vel_rms: r.uni(0.05, 2.0),
            vel_kms: r.log(200.0, 5000.0),
            rth_vc: r.uni(5.0, 60.0),
            tau_vc: if hot { r.log(0.01, 1.0) } else { r.log(0.1, 10.0) },
            rth_mag: r.uni(5.0, 60.0),
            tau_mag: r.log(5.0, 200.0),
            ambient,
            amplitude_limit: r.uni(0.1, 1.5),
            displacement_limit: r.log(1e-4, 2e-3),
            velocity_limit: {
                let v0 = r.uni(0.05, 2.0);
                [v0, v0 * r.uni(1.0, 2.0), v0 * r.uni(2.0, 3.0)]
            },
            mask_energy_db: {
                let e0 = r.uni(-60.0, 40.0);
                [e0, e0 + r.uni(0.0, 10.0), e0 + r.uni(10.0, 20.0), e0 + r.uni(20.0, 30.0)]
            },
            mask_band_hz: [(0.3 * nyq) as f32, (0.45 * nyq) as f32],
            mask_attack_s: r.log(1e-3, 0.1),
            mask_release_s: r.log(1e-2, 1.0),
            mask_attack_fast_s: r.log(1e-3, 0.05),
            mask_attack_range_db: r.uni(0.0, 80.0),
            velocity_ignore_low_band: r.chance(0.5) as u32,
            velocity_ignore_high_band: r.chance(0.5) as u32,
            band_center_hz: {
                let f0 = r.log(20.0, 0.01 * nyq);
                [f0, f0 * 4.0, f0 * 10.0, f0 * 25.0, (f0 * 60.0).min((0.9 * nyq) as f32)]
            },
            bandwidth: (r.uni(0.0, 2.99)) as u32,
            pilot_reserve: r.uni(0.0, 0.05),
            max_velocity_attenuation_db: r.uni(0.0, 30.0),
            temperature_soft_limit: soft,
            temperature_hard_limit: soft + r.uni(0.0, 30.0),
            temperature_headroom: r.uni(0.0, 10.0),
            max_temperature_attenuation_db: r.uni(0.0, 30.0),
            ..lsm2::ChannelParams::default()
        };
    }
    assert_eq!(p.validate(), Ok(()), "generator made an invalid lsm2 set");
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
    let mut handoff = HandOff::EMPTY;
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
        // the plugin calls save_state after every process
        let _ = unit.save_state(&mut handoff);
        assert_eq!(allocs(), before, "{:?} allocated in process/telemetry/save_state ({m} frames)", unit.kind());
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
///
/// lsm2 is a five-band limiter: an output sample mixes 319 input samples
/// with per-band gains, so it is bounded by digital full scale, not by the
/// one input sample at its delay (its per-frequency bound is in
/// tests/lsm2.rs; review_bound does the same).
fn check_bound(input: &[Vec<f32>], out: &[Vec<f32>], latency: usize, gain: f32, what: &str) {
    check_bound_pilot(input, out, latency, gain, &[], what)
}

/// [`check_bound`] with a per-host-channel additive allowance for a
/// generated pilot tone (`pilot[c]`, missing = 0), and never above full
/// scale: `|out| <= min(1, |in| * gain + pilot)`.
fn check_bound_pilot(input: &[Vec<f32>], out: &[Vec<f32>], latency: usize, gain: f32, pilot: &[f32], what: &str) {
    if what.starts_with("lsm2") {
        for (c, y) in out.iter().enumerate() {
            for (k, &o) in y.iter().enumerate() {
                assert!(o.is_finite() && o.abs() <= 1.0, "{what}: output {o:e} at ch{c}[{k}]");
            }
        }
        return;
    }
    for (c, (x, y)) in input.iter().zip(out).enumerate() {
        let extra = pilot.get(c).copied().unwrap_or(0.0);
        for (k, &o) in y.iter().enumerate() {
            assert!(o.is_finite(), "{what}: non-finite output {o} at ch{c}[{k}]");
            let i = if k >= latency { x[k - latency] } else { 0.0 };
            let prog = if i.is_finite() { (i * gain).abs() } else { 0.0 };
            // the pilot sum is one more f32 rounding
            let lim = if extra > 0.0 { (prog + extra) * (1.0 + 1e-6) } else { prog };
            let lim = if extra > 0.0 { lim.min(1.0) } else { lim };
            assert!(
                o.abs() <= lim,
                "{what}: |out| {o:e} > {lim:e} (in {i:e}, gain {gain}, pilot {extra}) at ch{c}[{k}]"
            );
        }
    }
}

/// Per host channel, the largest pilot an spp3 set may generate: the law's
/// 0.3 cap on any channel with `PilotToneEnabled` (AID35 uses 0.0316).
fn spp3_pilot_allowance(p: &spp3::Params) -> Vec<f32> {
    let n = (p.channels as usize).min(MAX_CHANNELS);
    let mut v = vec![0.0; n];
    for c in &p.ch[..n] {
        if c.pilot_tone_enabled != 0 && (c.source as usize) < n {
            v[c.source as usize] = 0.3;
        }
    }
    v
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
        check_bound_pilot(&x, &rr.out, 0, 1.0, &spp3_pilot_allowance(&p), "spp3");
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
        // the dezipper passes up to its own peak, as the reference's does
        check_bound(&x, &rr.out, 0, u.gain_peak(), "atsp");
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
        let mut p = spp3_params(seed, n, blk, rate, ff, true);
        // the closed loop's window sums depend on the host slicing on
        // aarch64 (FINDING 16, pinned by its own test)
        for c in p.ch.iter_mut() {
            c.cal_present = 0;
        }
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
        // Each render call snaps the output dezipper to the selected target
        // (rig dezipper law, as the reference does), so the output depends
        // on where calls start. Fixed call sizes that divide the thermal
        // update never cross an update inside a call, hold the gain
        // constant per update and must agree bit for bit.
        // (the update is `spp3::thermal_frames(rate)`: 256 at 44.1/48 kHz,
        // 64 at 8 kHz, 1024 at 192 kHz)
        let upd = spp3::thermal_frames(rate);
        let pow = |v: u64| vec![1usize << (v % (upd.trailing_zeros() as u64 + 1))];
        let mut u1 = spp3::Unit::new(p);
        let mut u2 = spp3::Unit::new(p);
        let r1 = render(&mut u1, &x, Some(&s), &pow(a), &mut no_sense, false);
        let r2 = render(&mut u2, &x, Some(&s), &pow(b), &mut no_sense, false);
        prop_assert!(bits(&r1.out) == bits(&r2.out), "audio differs between slicings");
        prop_assert_eq!(tele_bits(&r1.tele[0].1), tele_bits(&r2.tele[0].1));
        for c in 0..n {
            prop_assert_eq!(u1.sense_power(c).to_bits(), u2.sense_power(c).to_bits());
        }
        prop_assert_eq!(u1.sense_delay(), u2.sense_delay());
        // reset returns the unit to exactly the state `new` builds
        u1.reset();
        let r3 = render(&mut u1, &x, Some(&s), &pow(b), &mut no_sense, false);
        prop_assert!(bits(&r3.out) == bits(&r2.out), "audio differs after reset");
        prop_assert_eq!(tele_bits(&r3.tele[0].1), tele_bits(&r2.tele[0].1));
        // any fixed call size (production: 512) is deterministic: the same
        // slicing twice, and again after reset, is bit for bit the same
        let fixed = vec![1 + (a ^ b) as usize % 4096];
        let mut u4 = spp3::Unit::new(p);
        let r4 = render(&mut u4, &x, Some(&s), &fixed, &mut no_sense, false);
        let r5 = render(&mut spp3::Unit::new(p), &x, Some(&s), &fixed, &mut no_sense, false);
        prop_assert!(bits(&r4.out) == bits(&r5.out), "audio not deterministic at a fixed {:?}-frame call", fixed);
        prop_assert_eq!(tele_bits(&r4.tele[0].1), tele_bits(&r5.tele[0].1));
        u4.reset();
        let r6 = render(&mut u4, &x, Some(&s), &fixed, &mut no_sense, false);
        prop_assert!(bits(&r6.out) == bits(&r5.out), "audio differs after reset at a fixed call size");
    }

    #[test]
    fn atsp_slice_invariant(
        seed in any::<u64>(), n in 1usize..=8, blk in block(), rate in rate(),
        len in 1usize..30_000, amp in 0.01f32..4.0, h in finite_hostility(), a in any::<u64>(), b in any::<u64>(),
        staged in any::<bool>(),
    ) {
        // SPEC-ATSP 11.3: the output depends on the host slice only through
        // its family; within one family any slicing is the same
        let p = atsp_params(seed, n, blk, rate, true);
        let win = atsp::window_frames(rate as f64);
        let x = signal(seed ^ 1, n, len, amp, h);
        // a one-frame last call (the input running out) is a family of its
        // own: leave the input one frame longer than every slicing's end
        let mono = n == 1;
        let (sa, sb) = (atsp_family_slices(a, win, staged, mono), atsp_family_slices(b, win, staged, mono));
        // one channel: every call, the last too, whole groups of eight
        let x: Vec<Vec<f32>> = if mono { x.iter().map(|c| c[..c.len() / 8 * 8].to_vec()).collect() } else { x };
        prop_assume!(!x[0].is_empty());
        let len = x[0].len();
        let ends_on_one = |sl: &[usize]| {
            let (mut done, mut k) = (0, 0);
            while done < len {
                let m = sl[k % sl.len()].min(len - done);
                if done + m == len {
                    return m == 1;
                }
                done += m;
                k += 1;
            }
            false
        };
        let x: Vec<Vec<f32>> = if ends_on_one(&sa) || ends_on_one(&sb) {
            x.iter().map(|c| c[..len - 1].to_vec()).collect()
        } else {
            x
        };
        prop_assume!(!x[0].is_empty() && !ends_on_one(&sa) && !ends_on_one(&sb));
        let mut u1 = atsp::Unit::new(p);
        let r1 = render(&mut u1, &x, None, &sa, &mut no_sense, false);
        let mut u2 = atsp::Unit::new(p);
        let r2 = render(&mut u2, &x, None, &sb, &mut no_sense, false);
        prop_assert!(bits(&r1.out) == bits(&r2.out), "audio differs between slicings");
        prop_assert_eq!(tele_bits(&r1.tele[0].1), tele_bits(&r2.tele[0].1));
        // reset returns the thermal and gain state to `new`'s; the output
        // dezipper is not restarted (25G83 rig), so compare two units that
        // took the same path
        u1.reset();
        u2.reset();
        let r3 = render(&mut u1, &x, None, &sb, &mut no_sense, false);
        let r4 = render(&mut u2, &x, None, &sa, &mut no_sense, false);
        prop_assert!(bits(&r3.out) == bits(&r4.out), "audio differs after reset");
        prop_assert_eq!(tele_bits(&r3.tele[0].1), tele_bits(&r4.tele[0].1));
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
///
/// `glide`: a unit whose panic is a target step through its output
/// dezipper (atsp, rig law 4) reaches the 20 dB target, within the
/// dezipper's fixed point `bound`, once the group has been over its hard
/// limit for `frames`; the check applies from then on, while it stays over.
fn check_hard_path(
    unit: &mut dyn ProtectionUnit,
    x: &[Vec<f32>],
    block: usize,
    // per host channel: (group id, hard limit)
    chans: &[(usize, f64)],
    glide: Option<(usize, f32)>,
) -> usize {
    let n = x.len();
    let len = x[0].len();
    let (settle, bound) = glide.unwrap_or((0, FAILSAFE_GAIN));
    let mut out = vec![vec![0.0f32; block]; n];
    let mut t = Telemetry::new();
    // per group: the frame from which it has been over its hard limit
    let mut hot_since: Vec<(usize, usize)> = Vec::new();
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
        let mut any = false;
        for (c, &(g, _)) in chans.iter().enumerate() {
            let Some(&(_, since)) = hot_since.iter().find(|h| h.0 == g) else { continue };
            any = true;
            for k in since.saturating_add(settle).saturating_sub(done).min(block)..block {
                let i = x[c][done + k];
                let lim = if i.is_finite() { (i * bound).abs() } else { 0.0 };
                assert!(
                    out[c][k].abs() <= lim,
                    "hard path missed: ch{c} frame {} out {} in {} (group {g} over its hard limit)",
                    done + k,
                    out[c][k],
                    i
                );
            }
        }
        if any {
            hard_blocks += 1;
        }
        let mut hot: Vec<usize> = Vec::new();
        for (c, &(g, limit)) in chans.iter().enumerate() {
            let temp = t.channels[c].coil_temp_c as f64;
            // telemetry is f32; skip a boundary the rounding could decide
            // either way
            if temp > limit + 1e-4 * limit.abs().max(1.0) && !hot.contains(&g) {
                hot.push(g);
            }
        }
        done += block;
        hot_since.retain(|h| hot.contains(&h.0));
        for g in hot {
            if !hot_since.iter().any(|h| h.0 == g) {
                hot_since.push((g, done));
            }
        }
    }
    hard_blocks
}

/// The atsp panic's glide (rig law 4, SPEC-ATSP 11): the panic target is
/// scheduled at the first tick after the coil is seen hot (at most 5 ms),
/// and the dezipper glides from at most its peak to within 3 % of the
/// 20 dB target (its Float32 fixed point is 0.6 % above it) in about 31 ms
/// at 48 kHz; 60 ms bounds both.
fn atsp_glide(rate: u32) -> Option<(usize, f32)> {
    Some(((0.06 * rate as f64) as usize, FAILSAFE_GAIN * 1.03))
}

/// atsp's channel groups as SPEC-ATSP 4 routes them: the first slot (A, B,
/// C) whose mask claims the channel, an unclaimed channel to A; one group
/// when linked globally, one per linked slot, else one per channel. Per
/// host channel: (group id, hard limit).
fn atsp_hard_groups(p: &atsp::Params, n: usize) -> Vec<(usize, f64)> {
    // the unit runs every constant held to its published range
    let p = &p.clamped();
    (0..n)
        .map(|c| {
            let si = (0..3).find(|&si| p.slot[si].channel_mask & (1 << c) != 0).unwrap_or(0);
            let s = &p.slot[si];
            let g = if p.gain_link_all != 0 {
                0
            } else if s.gain_link != 0 {
                100 + si
            } else {
                200 + c
            };
            (g, s.temperature_limit as f64 + s.hard_limit_headroom as f64)
        })
        .collect()
}

/// An independent model of the spp3 controller (open loop), from the laws
/// the module documents, checked against the unit sample by sample.
///
/// What it models, from the parameters and the input alone:
/// - `Pdrv = elec * mean(clamp(x, +-1.5 FS)^2)` on each update's own 256
///   frames (no history shift; a non-finite sample counts as 0);
/// - the feed-forward node `theta_ff` (trapezoidal-input, linear-decay,
///   `tau_ff = factor * tau_Tvc`), seeded by 64007: 2 is cold (0), any
///   other value the hot split at the limit (`p0 * T_sett_vc`);
/// - the candidates `g_ff = sqrt(clamp((Tlim - amb - theta_mg) /
///   (theta_ff + 1), 1e-4, 1))` and, while armed, the same 5 C lower (the
///   fixed 20 dB without feed-forward);
/// - the 1 s release hold of the candidate against the previous selected
///   gain, the 99 % attack/release smoother from the selected minimum, the
///   group minimum with the hard target;
/// - the two cascaded 5 ms dezippers, per sample, both stages starting each
///   render call at the selected target (rig dezipper law).
///
/// From the telemetry it takes `T_OL` and the magnet (they depend on the
/// protected drive, which the unit meters itself), and decides arming with
/// the documented hysteresis (`T_OL > Tlim + headroom` arms, `< Tlim`
/// disarms).
///
/// Two checks: while no decision so far fell within rounding of a
/// threshold, the unit's selected target equals the model's (to 1e-3); at
/// every sample, the output is no louder than the model's dezippered target
/// times the clamped input and the pilot headroom, where after an undecided
/// update the model allows the largest target any of its branches could
/// give (the hard bound while certainly armed, else unity). It needs one
/// call per thermal update, the closed loop off and SafeTlim not latching.
struct HardOracle {
    /// Frames per thermal update at this rate (`spp3::thermal_frames`).
    frames: usize,
    updates: usize,
    /// Per unit channel, the update at which its SafeTlim latched.
    latched_at: Vec<Option<usize>>,
    amb: f64,
    k_dez: f64,
    k_attack: f64,
    k_release: f64,
    hold_n: u32,
    ch: Vec<OracleChan>,
    /// Per group: smoothed gain, selected target, the dezipper stages.
    g: Vec<f64>,
    sel: Vec<f64>,
    allowed: Vec<f64>,
    z1: Vec<f64>,
    z2: Vec<f64>,
    /// The exact model is still in step with the unit.
    exact: bool,
    /// Per group, the unit's selected target published at the last update.
    unit_sel_prev: Vec<f64>,
    /// Updates at which some channel was certainly armed.
    armed_updates: usize,
    /// Updates checked exactly.
    exact_updates: usize,
}

#[derive(Clone, Copy, PartialEq)]
enum HoldState {
    Attack,
    Holding,
    Released,
}

struct OracleChan {
    src: usize,
    group: usize,
    elec: f64,
    ff_on: bool,
    /// Float32 node coefficients, as the unit computes them.
    a: f32,
    b: f32,
    theta_ff: f32,
    p_prev: f64,
    headroom_c: f64,
    safe: OracleSafe,
    headroom: f32,
    armed: Option<bool>,
    /// The hold states consistent with the unit so far (a comparison within
    /// rounding forks them; the unit's published candidate prunes them).
    holds: Vec<(HoldState, u32, f64, f64)>,
}

const ORACLE_CLAMP: f32 = 1.5;

/// SafeTlim as specified: the base limit leaves `TemperatureLimit` for
/// `min(SafeTlim, TemperatureLimit)` after `init = ceil(TimeMin * 60 * fs /
/// update)` consecutive updates with `f32(T_OL) > f32(Tlim_eff) + min(offset,
/// 0)` (strict; equality or NaN reloads the count), one way, from the next
/// update. A rebuild (`new`, `reset`) restarts it; AudioUnitReset keeps it.
#[derive(Clone, Copy)]
struct OracleSafe {
    normal: f64,
    safe: f64,
    offset: f32,
    init: u32,
    count: u32,
    latched: bool,
}

impl OracleSafe {
    fn base(&self) -> f64 {
        if self.latched {
            self.safe
        } else {
            self.normal
        }
    }
    fn update(&mut self, t_ol: f32, tlim_eff: f64) {
        if t_ol > tlim_eff as f32 + self.offset {
            self.count = self.count.wrapping_sub(1);
        } else {
            self.count = self.init;
        }
        if self.count == 0 {
            self.latched = true;
        }
    }
}

impl HardOracle {
    fn new(p: &spp3::Params) -> Self {
        let n = p.channels as usize;
        let rate = p.sample_rate as f64;
        let upd = spp3::thermal_frames(p.sample_rate);
        let dt32 = upd as f32 / p.sample_rate as f32;
        let r = rate / upd as f64;
        let amb = p.thermal_ambient_temp as f64;
        let init = spp3::safe_count_init(p.thermal_safe_tlim_time_min, p.sample_rate).expect("valid TimeMin");
        let cold = p.thermal_state_override == spp3::THERMAL_STATE_COLD;
        let mut ids: Vec<u32> = Vec::new();
        let ch = p.ch[..n]
            .iter()
            .map(|c| {
                let group = ids.iter().position(|&g| g == c.group).unwrap_or_else(|| {
                    ids.push(c.group);
                    ids.len() - 1
                });
                let (reb, series, ag) = (c.reb_ref as f64, (c.rshunt + c.rampout) as f64, c.ag as f64);
                let ff_on = c.thermal_ff_speedup_factor > 0.0;
                let tau = if ff_on { c.thermal_ff_speedup_factor * c.tau_tvc } else { c.tau_tvc };
                let (tvc, tmg) = (c.t_sett_vc as f64, c.t_sett_mg as f64);
                let seed = if cold { 0.0 } else { (c.temperature_limit as f64 - amb) / (tvc + tmg) * tvc };
                OracleChan {
                    src: c.source as usize,
                    group,
                    elec: ag * ag * reb / ((reb + series) * (reb + series)),
                    ff_on,
                    a: 1.0 - dt32 / tau,
                    b: 0.5 * c.t_sett_vc * dt32 / tau,
                    theta_ff: seed as f32,
                    p_prev: 0.0,
                    headroom_c: c.hard_temp_limit_headroom as f64,
                    safe: OracleSafe {
                        normal: c.temperature_limit as f64,
                        safe: p.thermal_safe_tlim.min(c.temperature_limit) as f64,
                        offset: p.thermal_safe_tlim_offset.min(0.0),
                        init,
                        count: init,
                        latched: false,
                    },
                    // pilot amplitude capped at 0.3 (rig law 4)
                    headroom: if c.pilot_tone_enabled != 0 {
                        1.0 - (10f64.powf(c.pilot_ampl_hi_db as f64 / 20.0) as f32).min(0.3)
                    } else {
                        1.0
                    },
                    armed: Some(false),
                    holds: vec![(HoldState::Attack, 0, 1.0, 1.0)],
                }
            })
            .collect();
        let g = ids.len();
        let settle = |s: f64, rate: f64| 1.0 - (0.01f64.ln() / (s * rate)).exp();
        HardOracle {
            frames: upd,
            updates: 0,
            latched_at: vec![None; n],
            amb,
            k_dez: settle(0.005, rate),
            k_attack: settle(p.thermal_gain_attack_time_s as f64, r),
            k_release: settle(p.thermal_gain_release_time_s as f64, r),
            hold_n: (1.0 * r).ceil() as u32,
            ch,
            g: vec![1.0; g],
            sel: vec![1.0; g],
            allowed: vec![1.0; g],
            z1: vec![1.0; g],
            z2: vec![1.0; g],
            exact: true,
            unit_sel_prev: vec![1.0; g],
            armed_updates: 0,
            exact_updates: 0,
        }
    }

    /// The release hold (rig law 5). Returns the candidate and whether the
    /// comparison was within rounding.
    /// The release hold (rig law 5), one step with the comparison
    /// `g >= sel_prev` given as `ge`.
    fn hold_step(h: (HoldState, u32, f64, f64), g: f64, ge: bool, n: u32) -> ((HoldState, u32, f64, f64), f64) {
        let (mut state, mut cnt, mut held, _) = h;
        let u = if cnt > 0 && ge {
            cnt -= 1;
            if cnt == 0 {
                state = HoldState::Released;
            }
            held
        } else {
            cnt = 0;
            if ge {
                if state == HoldState::Attack {
                    held = h.3;
                    cnt = n.saturating_sub(1);
                    state = if cnt == 0 { HoldState::Released } else { HoldState::Holding };
                    held
                } else {
                    g
                }
            } else {
                state = HoldState::Attack;
                g
            }
        };
        ((state, cnt, held, u), u)
    }

    /// The hold for this update over every state still consistent with the
    /// unit. A comparison within what the published temperatures resolve
    /// (the candidate rests on the f32 magnet temperature; exact only when
    /// clamped at unity or the floor) forks the state into both branches;
    /// branches whose candidate disagrees with the unit's published one
    /// (`controller_state()[0]`) are dropped. Returns the candidate, or
    /// `None` when no lawful branch gives the unit's candidate although a
    /// fork was involved (undecided). Without a fork the model's own
    /// prediction is returned, and the exact check judges it.
    fn hold(holds: &mut Vec<(HoldState, u32, f64, f64)>, g: f64, sel_prev: f64, n: u32, unit_u: f64) -> Option<f64> {
        let exact_sides = (g == 1.0 || g == 0.01) && g == sel_prev;
        let tie = !exact_sides && (g - sel_prev).abs() <= 1e-3 * sel_prev.abs().max(1e-3);
        let near = |a: f64, b: f64| (a - b).abs() <= 1e-4 * b.abs().max(1e-3);
        let forked = tie || holds.len() > 1;
        let mut next: Vec<((HoldState, u32, f64, f64), f64)> = Vec::new();
        for &h in holds.iter() {
            let branches: &[bool] = if tie {
                &[true, false]
            } else if g >= sel_prev {
                &[true]
            } else {
                &[false]
            };
            for &ge in branches {
                let (nh, u) = Self::hold_step(h, g, ge, n);
                if !next.iter().any(|(o, _)| o.0 == nh.0 && o.1 == nh.1 && near(o.2, nh.2) && near(o.3, nh.3)) {
                    next.push((nh, u));
                }
            }
        }
        let first = next[0].1;
        let mut keep: Vec<_> = next.iter().filter(|(_, u)| near(*u, unit_u)).cloned().collect();
        // the closest first, so a full set drops the least likely states
        keep.sort_by(|a, b| (a.1 - unit_u).abs().total_cmp(&(b.1 - unit_u).abs()));
        if keep.is_empty() {
            if forked {
                *holds = vec![next[0].0];
                return None;
            }
            *holds = vec![next[0].0];
            return Some(first);
        }
        *holds = keep.iter().take(32).map(|(h, _)| *h).collect();
        Some(keep[0].1)
    }

    /// One thermal update: `x` and `out` are the update's 256 frames per host
    /// channel, `t` the telemetry after it.
    /// `unit_cs[c]` is unit channel `c`'s `spp3::Unit::controller_state(c)`:
    /// `[candidate, selected target, Tlim_eff, theta_ff, theta_vc, theta_mg,
    /// ..]` (the telemetry's axis 4 is the applied, dezippered gain). The
    /// oracle checks the unit's `theta_ff` against its own independent
    /// virtual coil, then takes one step of the controller laws from the
    /// unit's published state, so rounding in the published temperatures
    /// never compounds.
    fn step(&mut self, x: &[&[f32]], out: &[&[f32]], t: &Telemetry, unit_cs: &[[f64; 11]]) {
        let frames = x[0].len();
        assert_eq!(frames, self.frames, "the oracle steps one thermal update per call");
        // each render call (one per update here) starts both dezipper stages
        // at the selected target (rig dezipper law); the audio of this update
        // then glides toward the target set at the last one
        for g in 0..self.allowed.len() {
            self.z1[g] = self.allowed[g];
            self.z2[g] = self.allowed[g];
        }
        for k in 0..frames {
            for g in 0..self.allowed.len() {
                self.z1[g] += self.k_dez * (self.allowed[g] - self.z1[g]);
                self.z2[g] += self.k_dez * (self.z1[g] - self.z2[g]);
            }
            for c in &self.ch {
                let i = x[c.src][k];
                if !i.is_finite() {
                    continue; // check_bound: it leaves as 0
                }
                let drive = i.clamp(-ORACLE_CLAMP, ORACLE_CLAMP).abs() as f64;
                // two f32 roundings, absolute on a subnormal input
                let bound = (drive * (self.z2[c.group] * c.headroom as f64 * (1.0 + 1e-4) + 1e-6)).min(1.0)
                    + 2.0 * f32::from_bits(1) as f64;
                let o = out[c.src][k].abs() as f64;
                assert!(
                    o <= bound,
                    "spp3 controller: ch{} sample {k} of the update leaves at {o:e} for input {i:e}, allowed {:.6} x",
                    c.src,
                    self.z2[c.group] * c.headroom as f64
                );
            }
        }
        // the feed-forward node, on this update's own unprotected frames
        for c in self.ch.iter_mut() {
            let acc: f64 = x[c.src]
                .iter()
                .map(|&v| if v.is_finite() { v.clamp(-ORACLE_CLAMP, ORACLE_CLAMP) as f64 } else { 0.0 })
                .map(|v| v * v)
                .sum();
            let p = c.elec * acc / frames as f64;
            // the Float32 fused recurrence
            c.theta_ff = c.a.mul_add(c.theta_ff, c.b * (p as f32 + c.p_prev as f32));
            c.p_prev = p;
        }
        // candidates, hold, arming, from the published temperatures
        let ng = self.g.len();
        let mut ff_t = vec![1.0f64; ng];
        let mut hard_t = vec![1.0f64; ng];
        let mut hard_bound = vec![1.0f64; ng];
        let mut certain = true;
        let mut any = false;
        let base_used: Vec<f64> = self.ch.iter().map(|c| c.safe.base()).collect();
        let ratio = |num: f64, vcff: f64| (num / (vcff + 1.0)).clamp(1e-4, 1.0).sqrt();
        for (ci, c) in self.ch.iter_mut().enumerate() {
            let s = &t.channels[c.src];
            let (t_ol, mag) = (s.coil_temp_c as f64, s.magnet_temp_c as f64);
            if !(t_ol.is_finite() && mag.is_finite()) {
                c.armed = None;
                certain = false;
                continue;
            }
            // the published T_OL is the unit's f64 value rounded to f32: within
            // a few f32 ulps a comparison could go either way
            let tol = 4.0 * (f32::from_bits((t_ol as f32).abs().to_bits() + 1) - (t_ol as f32).abs()) as f64;
            // the unit: arm above `hard_on`, else disarm below `limit`, else
            // keep; a comparison within rounding takes both branches
            let cmp = |a: f64, b: f64| {
                if a > b + tol {
                    Some(true)
                } else if a < b - tol {
                    Some(false)
                } else {
                    None
                }
            };
            let keep_or_disarm = |prev: Option<bool>| match cmp(t_ol, c.safe.base()) {
                // above the limit: keep; below: disarm
                Some(true) => prev,
                Some(false) => Some(false),
                None => (prev == Some(false)).then_some(false),
            };
            let prev = c.armed;
            let (limit, hard_on) = (c.safe.base(), c.safe.base() + c.headroom_c);
            c.armed = match cmp(t_ol, hard_on) {
                Some(true) => Some(true),
                Some(false) => keep_or_disarm(prev),
                None => (keep_or_disarm(prev) == Some(true)).then_some(true),
            };
            // rig law 6a by policy: the feed-forward node above 1000 K holds
            // the hard path (the unit flags MODEL_RANGE), undecided within
            // rounding of the threshold
            if c.ff_on {
                c.armed = match cmp(c.theta_ff as f64, 1000.0) {
                    Some(true) => Some(true),
                    Some(false) => c.armed,
                    None => (c.armed == Some(true)).then_some(true),
                };
            }
            // the virtual coil, independently: the unit's must agree
            let cs = &unit_cs[ci];
            if c.ff_on {
                let (mine, unit) = (c.theta_ff as f64, cs[3]);
                assert!(
                    (mine - unit).abs() <= 1e-4 * unit.abs().max(1.0),
                    "spp3 feed-forward node: ch{} theta_ff {unit}, the specified recurrence gives {mine} (update {})",
                    c.src,
                    self.updates
                );
            }
            let theta_mg = if cs[5].is_finite() { cs[5] } else { mag - self.amb };
            let num = limit - self.amb - theta_mg;
            let theta_ff = if c.ff_on && cs[3].is_finite() { cs[3] } else { c.theta_ff as f64 };
            let fresh = if c.ff_on { ratio(num, theta_ff) } else { 1.0 };
            let g_ff = match Self::hold(&mut c.holds, fresh, self.g[c.group], self.hold_n, unit_cs[ci][0]) {
                Some(u) => u,
                None => {
                    certain = false;
                    unit_cs[ci][0]
                }
            };
            ff_t[c.group] = ff_t[c.group].min(g_ff);
            match c.armed {
                Some(true) => {
                    any = true;
                    let g_hard = if c.ff_on { ratio(num - 5.0, theta_ff) } else { FAILSAFE_GAIN as f64 };
                    hard_t[c.group] = hard_t[c.group].min(g_hard);
                    // the bound: generous in theta_mg and theta_ff rounding
                    let slack = 1e-4 * (limit.abs() + mag.abs() + 1.0);
                    let gb = if c.ff_on {
                        ratio(num - 5.0 + slack, theta_ff * (1.0 - 1e-4) - 1e-6)
                    } else {
                        FAILSAFE_GAIN as f64
                    };
                    hard_bound[c.group] = hard_bound[c.group].min(gb);
                }
                Some(false) => {}
                None => certain = false,
            }
            // SafeTlim, for the next update: the unit's own Float32 T_OL
            // (the published value) against the base it used
            let was = c.safe.latched;
            c.safe.update(s.coil_temp_c, limit);
            if c.safe.latched && !was {
                self.latched_at[ci] = Some(self.updates);
            }
        }
        self.updates += 1;
        if any {
            self.armed_updates += 1;
        }
        if !certain {
            self.exact = false;
        }
        for gi in 0..ng {
            let tg = ff_t[gi];
            let k = if tg < self.g[gi] { self.k_attack } else { self.k_release };
            let g = self.g[gi] + k * (tg - self.g[gi]);
            self.sel[gi] = g.min(hard_t[gi]);
            self.g[gi] = self.sel[gi];
        }
        // the checks, then the targets the next update's audio may follow
        for (ci, c) in self.ch.iter().enumerate() {
            let sel = unit_cs[ci][1];
            let hb = hard_bound[c.group];
            // the base limit the unit used (TlimAdj meter: effective limit
            // against TemperatureLimit; open loop, so the effective limit is
            // the SafeTlim base), exactly
            let adj = t.channels[c.src].tlim_adj_c as f64;
            let want_adj = (base_used[ci] - c.safe.normal) as f32 as f64;
            assert!(
                (adj - want_adj).abs() <= 1e-4 * want_adj.abs().max(1.0),
                "spp3 SafeTlim: ch{} effective limit {} against {} (update {})",
                c.src,
                c.safe.normal + adj,
                base_used[ci],
                self.updates - 1
            );
            assert!(sel <= hb * (1.0 + 1e-4) + 1e-6, "spp3 hard path: group target {sel} above the hard target {hb}");
            if self.exact {
                let want = self.sel[c.group];
                assert!(
                    (sel - want).abs() <= 1e-3 * want + 1e-6,
                    "spp3 controller: group target {sel}, the specified laws give {want} (update {})",
                    self.exact_updates
                );
            }
        }
        if self.exact {
            self.exact_updates += 1;
        }
        for ((a, &sel), &hb) in self.allowed.iter_mut().zip(&self.sel).zip(&hard_bound) {
            *a = if self.exact { sel * (1.0 + 1e-3) + 1e-6 } else { hb };
        }
        // each update is checked as one step of the laws from the unit's own
        // previous selected gain (checked above), so rounding in the
        // published temperatures never accumulates in the model's smoother
        for (ci, c) in self.ch.iter().enumerate() {
            self.g[c.group] = unit_cs[ci][1];
        }
        for (ci, c) in self.ch.iter().enumerate() {
            self.unit_sel_prev[c.group] = unit_cs[ci][1];
        }
    }
}

/// A host event between two thermal updates.
#[derive(Clone, Copy, Debug, PartialEq)]
enum HostEvent {
    /// AudioUnitReset: the unit continues (only the closed loop restarts).
    AudioUnitReset,
    /// Kernel rebuild (`reset`, as Uninitialize+Initialize): the start
    /// state, SafeTlim NORMAL.
    Rebuild,
}

struct HardRun {
    armed: usize,
    exact: usize,
    tele: Telemetry,
    /// Per host channel, the peak `T_OL` over the last third of the run.
    peak: Vec<f32>,
    /// Per unit channel, the update (counted since the last rebuild) at
    /// which the oracle's SafeTlim latched, and the oracle at the end.
    latched_at: Vec<Option<usize>>,
    base: Vec<f64>,
}

/// Render `x` through spp3 one thermal update per call under the oracle,
/// with host `events` before the given updates.
fn spp3_hard_render_ev(p: &spp3::Params, x: &[Vec<f32>], events: &[(usize, HostEvent)]) -> HardRun {
    let mut u = spp3::Unit::new(*p);
    assert!(!u.is_failsafe());
    let mut o = HardOracle::new(p);
    let n = x.len();
    let f = spp3::thermal_frames(p.sample_rate);
    let mut out = vec![vec![0.0f32; f]; n];
    let mut t = Telemetry::new();
    let mut peak = vec![f32::MIN; n];
    let (mut armed, mut exact) = (0, 0);
    let (mut done, mut update) = (0, 0);
    while done + f <= x[0].len() {
        for &(_, e) in events.iter().filter(|(k, _)| *k == update) {
            match e {
                HostEvent::AudioUnitReset => u.host_reset(),
                HostEvent::Rebuild => {
                    u.reset();
                    armed += o.armed_updates;
                    exact += o.exact_updates;
                    o = HardOracle::new(p);
                }
            }
        }
        let ins: Vec<&[f32]> = x.iter().map(|b| &b[done..done + f]).collect();
        let mut outs: Vec<&mut [f32]> = out.iter_mut().map(|b| &mut b[..]).collect();
        let before = allocs();
        u.process(Io { frames: f, input: &ins, sense: &[], output: &mut outs });
        u.telemetry(&mut t);
        assert_eq!(allocs(), before);
        drop(outs);
        let outs: Vec<&[f32]> = out.iter().map(|b| &b[..]).collect();
        let cs: Vec<[f64; 11]> = (0..p.channels as usize).map(|c| u.controller_state(c)).collect();
        o.step(&ins, &outs, &t, &cs);
        done += f;
        update += 1;
        if 3 * done >= 2 * x[0].len() {
            for (pk, s) in peak.iter_mut().zip(&t.channels) {
                *pk = pk.max(s.coil_temp_c);
            }
        }
    }
    HardRun {
        armed: armed + o.armed_updates,
        exact: exact + o.exact_updates,
        tele: t,
        peak,
        latched_at: o.latched_at.clone(),
        base: o.ch.iter().map(|c| c.safe.base()).collect(),
    }
}

/// [`spp3_hard_render_ev`] without host events: (updates certainly armed,
/// updates checked exactly, telemetry, peak `T_OL` per host channel).
fn spp3_hard_render(p: &spp3::Params, x: &[Vec<f32>]) -> (usize, usize, Telemetry, Vec<f32>) {
    let r = spp3_hard_render_ev(p, x, &[]);
    (r.armed, r.exact, r.tele, r.peak)
}

/// `n` channels of a constant sine for `secs`.
fn sine_drive(n: usize, secs: f64, rate: u32, amp: f32, f: f64) -> Vec<Vec<f32>> {
    let len = (secs * rate as f64) as usize;
    let w = 2.0 * std::f64::consts::PI * f / rate as f64;
    let x: Vec<f32> = (0..len).map(|k| amp * (w * k as f64).sin() as f32).collect();
    vec![x; n]
}

proptest! {
    #![proptest_config(cfg(32))]

    /// spp3 (property 4, as specified now): while armed, the group target is
    /// at most the hard target and the audio follows it through the
    /// dezippers; see [`HardOracle`].
    #[test]
    fn spp3_hard_path_holds_the_hard_target(
        seed in any::<u64>(), n in 1usize..=8, rate in prop_oneof![Just(48_000u32), 8_000u32..=192_000],
        ff in ff(), secs in 1.0f64..8.0, amp in 0.5f32..4.0, h in hostility(),
    ) {
        let mut p = spp3_params(seed, n, 512, rate, ff, true);
        // short coil constants and low limits so the coil reaches the hard
        // limit within the run; closed loop off (the corrected limit is
        // internal)
        for c in 0..n {
            p.ch[c].tau_tvc = p.ch[c].tau_tvc.clamp(0.1, 0.2);
            p.ch[c].tau_tmg = p.ch[c].tau_tmg.max(0.1);
            p.ch[c].temperature_limit = p.thermal_ambient_temp + 5.0 + (p.ch[c].temperature_limit - p.thermal_ambient_temp) / 8.0;
            p.ch[c].hard_temp_limit_headroom = p.ch[c].hard_temp_limit_headroom.abs() / 4.0;
            p.ch[c].cal_present = 0;
        }
        stable_spp3(&mut p);
        // both start states; SafeTlim at shipped TimeMin (5, 10 min) and at
        // short ones that latch within the run (the oracle models it)
        p.thermal_state_override = if seed % 2 == 0 { spp3::THERMAL_STATE_COLD } else { 0 };
        p.thermal_safe_tlim_time_min = [10.0, 5.0, 0.01, 0.02, 0.05, 0.002][(seed >> 8) as usize % 6];
        let len = (secs * rate as f64) as usize;
        let x = signal(seed ^ 1, n, len, amp, h);
        let _ = spp3_hard_render(&p, &x);
    }

    #[test]
    fn atsp_hard_path_cuts_20db_within_one_block(
        seed in any::<u64>(), n in 1usize..=8, blk in prop_oneof![Just(512u32), Just(64), Just(0), 16u32..=2048],
        call in 2usize..=240, secs in 1.0f64..8.0, amp in 0.5f32..4.0, h in hostility(),
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
        let chans = atsp_hard_groups(&p, n);
        let mut u = atsp::Unit::new(p);
        prop_assert!(!u.is_failsafe());
        // the panic is decided at every 5 ms tick from the post-gain coil
        // as it stood at the start of the tick's fragment (SPEC-ATSP 11.3),
        // and a tick that finds it back under the limit releases at once.
        // Calls of at most one tick make the telemetry this test reads at
        // each call boundary exactly what the next tick reads (the reserved
        // `block_frames` does not change that)
        check_hard_path(&mut u, &x, call, &chans, atsp_glide(rate));
    }

    /// lsm2's temperature axis (spec 15.5) reaches the audio: the applied
    /// gain is at most the most permissive temperature candidate over the
    /// last controller interval plus ten attack time constants (what the
    /// attack ramp can still be catching up on).
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
        // spec 15.5: the applied slow gain follows the temperature axis's
        // candidate (0.99 below it on a cut, at most it on a release)
        let window = settle.div_ceil(256) + 1;
        for (k, (done, t)) in rr.tele.iter().enumerate() {
            if *done < settle || k < window {
                continue;
            }
            for c in &p.ch[..n] {
                let s = &t.channels[c.source as usize];
                let mut most = 0.0f64;
                for (_, w) in &rr.tele[k - window..=k] {
                    most = most.max(w.channels[c.source as usize].axis_gain[4] as f64);
                }
                let tol = 1e-4;
                prop_assert!(
                    s.gain as f64 <= most + tol,
                    "lsm2 temperature candidate {} not reached: gain {} at {} C", most, s.gain, s.coil_temp_c
                );
            }
        }
    }
}

proptest! {
    // 60 s of audio per case: few cases by default
    #![proptest_config(cfg(4))]

    /// Finding 15, in the physical regime: with the feed-forward on and
    /// every thermal time constant at or above the documented 0.1 s minimum
    /// (now the load-time floor), a magnet at least ten times slower than
    /// the coil, release no faster than attack and a nonnegative headroom,
    /// sustained drive never takes the modelled coil above the hard limit
    /// (+0.5 C) over the last third of a 60 s run, cold or hot start,
    /// unless the gain sits at the 0.01 candidate floor. The controller laws
    /// are checked along the way by [`HardOracle`]. Outside this regime (16-
    /// 21 ms coils) the reference itself limit-cycles above the limit, and
    /// the engine now refuses those tunings.
    #[test]
    fn spp3_feed_forward_settles_below_the_hard_limit(
        seed in any::<u64>(), n in 1usize..=4, amp in 0.3f32..3.0, f in 50.0f64..3000.0,
        tvc in 0.1f32..2.0, mg_ratio in 10.0f32..30.0, cold in any::<bool>(),
    ) {
        let rate = 48_000u32;
        let mut p = spp3_params(seed, n, 512, rate, Ff::On, true);
        for (k, c) in p.ch[..n].iter_mut().enumerate() {
            // spread the channels around the drawn coil constant
            c.tau_tvc = tvc * (1.0 + 0.3 * k as f32);
            c.tau_tmg = (c.tau_tvc * mg_ratio).min(15.0).max(10.0 * c.tau_tvc);
            c.cal_present = 0;
            c.hard_temp_limit_headroom = c.hard_temp_limit_headroom.abs();
        }
        stable_spp3(&mut p);
        p.thermal_gain_release_time_s = p.thermal_gain_release_time_s.max(p.thermal_gain_attack_time_s);
        p.thermal_state_override = if cold { spp3::THERMAL_STATE_COLD } else { 0 };
        p.thermal_safe_tlim_time_min = 1e3;
        let x = sine_drive(n, 60.0, rate, amp, f);
        let (_, _, t, peak) = spp3_hard_render(&p, &x);
        for c in &p.ch[..n] {
            let s = &t.channels[c.source as usize];
            let hard_on = c.temperature_limit + c.hard_temp_limit_headroom;
            let pk = peak[c.source as usize];
            prop_assert!(
                pk <= hard_on + 0.5 || s.axis_gain[4] <= 0.01 + 1e-4,
                "coil peaks at {} C over the last 20 s, hard limit {} C, limit {} C, gain {}; params {:?}",
                pk, hard_on, c.temperature_limit, s.gain, p
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Property 5: parameter blobs.

fn probe(unit: &mut dyn ProtectionUnit, n: usize, latency: usize, gain: f32, what: &str, h: Hostility) {
    probe_pilot(unit, n, latency, gain, &[], what, h)
}

fn probe_pilot(
    unit: &mut dyn ProtectionUnit,
    n: usize,
    latency: usize,
    gain: f32,
    pilot: &[f32],
    what: &str,
    h: Hostility,
) {
    let x = signal(0x5eed, n, 6000, 1.5, h);
    let s = synth_sense(&x, 2 * n, 0);
    let rr = render(unit, &x, Some(&s), &[1, 511, 4096, 96], &mut no_sense, false);
    check_bound_pilot(&x, &rr.out, latency, gain, pilot, what);
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
            probe_pilot(&mut u, p.channels as usize, 0, g, &spp3_pilot_allowance(&p), "spp3 blob", Hostility::Wild);
        }
        // Clean input only: FailSafe passes NaN/Inf through (FINDING 5).
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
        let g = if u.is_failsafe() { FAILSAFE_GAIN } else { u.gain_peak() };
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
        let cl = p.ch[..n].iter().any(|c| c.cal_present != 0);
        let x = signal(seed ^ 1, n, len, amp, Hostility::Clean);
        let s = synth_sense(&x, 2 * n, delay);
        let cuts = slices(sl);
        let valid = render(&mut spp3::Unit::new(p), &x, Some(&s), &cuts, &mut no_sense, false);
        let open = render(&mut spp3::Unit::new(p), &x, None, &cuts, &mut no_sense, false);
        let mut r = Rng(sl ^ 77);
        let mut modes = Vec::new();
        let mut u = spp3::Unit::new(p);
        let drop = render(&mut u, &x, Some(&s), &cuts, &mut |_| { let m = sense_mode(&mut r); modes.push(m); m }, false);
        // With the closed loop configured, a dropout currently keeps the last
        // correction and can end up above both references (FINDING 3,
        // pinned by its regression test); the comparison is made for the
        // open-loop sets, where sense must not matter at all.
        for c in 0..n {
            for k in 0..len {
                if cl {
                    break;
                }
                let d = drop.out[c][k].abs();
                prop_assert!(d <= valid.out[c][k].abs() && d <= open.out[c][k].abs(),
                    "sense dropout raised ch{} [{}]: {} vs valid {} open {}", c, k, d, valid.out[c][k], open.out[c][k]);
            }
        }
        // a block whose every frame came without usable sense is reported
        // open loop (health reflects the last completed block)
        let _ = blk;
        let blk = spp3::THERMAL_FRAMES;
        let mut d0 = 0;
        for (call, (h, m)) in drop.health.iter().zip(&modes).enumerate() {
            let d1 = (d0 + cuts[call % cuts.len()]).min(len);
            let last_end = d1 / blk * blk;
            // (non-finite sense is not reported: FINDING 10)
            let unusable = matches!(m, Sense::Empty | Sense::Missing | Sense::Short);
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
fn hard_path_anchors_are_reached() {
    let (mut armed, mut exact) = (0, 0);
    for seed in 0..12u64 {
        let n = 2;
        let mut p = spp3_params(seed, n, 512, 48_000, if seed % 2 == 0 { Ff::Off } else { Ff::On }, true);
        for c in 0..n {
            p.ch[c].tau_tvc = 0.1;
            p.ch[c].tau_tmg = p.ch[c].tau_tmg.max(0.1);
            p.ch[c].temperature_limit = p.thermal_ambient_temp + 10.0;
            p.ch[c].hard_temp_limit_headroom = 2.0;
            p.ch[c].cal_present = 0;
        }
        stable_spp3(&mut p);
        p.thermal_state_override = if seed % 4 < 2 { spp3::THERMAL_STATE_COLD } else { 0 };
        p.thermal_safe_tlim_time_min = 1e3;
        let x = signal(seed, n, 48_000 * 4, 2.0, Hostility::Clean);
        let (a, e, _, _) = spp3_hard_render(&p, &x);
        armed += a;
        exact += e;
    }
    assert!(armed > 100, "the spp3 hard-path property never saw the hard path ({armed} updates)");
    // the exact controller model must actually run, not just the bound
    assert!(exact > 2000, "the spp3 controller model stayed in step for only {exact} updates");
    let mut hits = 0;
    for seed in 0..12u64 {
        let mut p = atsp_params(seed, 2, 512, 48_000, true);
        for s in p.slot.iter_mut() {
            s.tau_vc = 0.05;
            s.temperature_limit = s.ambient + 10.0;
            s.hard_limit_headroom = 2.0;
        }
        let x = signal(seed, 2, 48_000 * 4, 2.0, Hostility::Clean);
        let chans = atsp_hard_groups(&p, 2);
        // one tick per call (see atsp_hard_path_cuts_20db_within_one_block)
        hits += check_hard_path(&mut atsp::Unit::new(p), &x, atsp::window_frames(48_000.0), &chans, atsp_glide(48_000));
    }
    assert!(hits > 100, "the atsp hard-path property never saw the hard path ({hits} blocks)");
}

/// AID35-like synthetic SafeTlim on one channel (base 100 C, offset -15 C,
/// limit 120 C) at `time_min`, driven at full scale so the predicate
/// `T_OL > Tlim - 15` holds once the coil is warm.
fn safetlim_case(time_min: f32, rate: u32) -> spp3::Params {
    let mut p = spp3_minimal(0.5);
    p.sample_rate = rate;
    p.thermal_state_override = spp3::THERMAL_STATE_COLD;
    p.thermal_safe_tlim_time_min = time_min;
    p
}

proptest! {
    #![proptest_config(cfg(16))]

    /// SafeTlim (property 4, its latch): the base limit moves to
    /// `min(SafeTlim, Tlim)` exactly `ceil(TimeMin * 60 * fs / update)`
    /// consecutive updates after `f32(T_OL) > f32(Tlim_eff) + min(offset, 0)`
    /// first holds (strict, equality reloads), from the next update, never
    /// back; the oracle checks the unit's effective limit on every update
    /// and the controller against the latched base.
    #[test]
    fn spp3_safetlim_latches_as_specified(
        time_min in prop_oneof![Just(0.01f32), Just(0.02), 0.002f32..0.05],
        rate in prop_oneof![Just(48_000u32), Just(44_100), Just(96_000)],
        offset in -20.0f32..5.0, safe in 60.0f32..140.0, amp in 0.5f32..1.5, secs in 3.0f64..8.0,
    ) {
        let mut p = safetlim_case(time_min, rate);
        (p.thermal_safe_tlim, p.thermal_safe_tlim_offset) = (safe, offset);
        let x = sine_drive(1, secs, rate, amp, 997.0);
        let r = spp3_hard_render_ev(&p, &x, &[]);
        prop_assert!(r.exact > 0);
        if let Some(k) = r.latched_at[0] {
            let init = spp3::safe_count_init(time_min, rate).unwrap() as usize;
            prop_assert!(k + 1 >= init, "latched after {} updates, the count is {}", k + 1, init);
        }
    }
}

/// SafeTlim across host events: AudioUnitReset keeps the latch (and the
/// thermal state), a rebuild restarts it at NORMAL with the full count; the
/// oracle follows both and checks the unit's effective limit every update.
#[test]
fn spp3_safetlim_survives_audio_unit_reset_and_restarts_on_rebuild() {
    let rate = 48_000;
    let p = safetlim_case(0.02, rate); // 1.2 s: 225 updates
    let init = spp3::safe_count_init(0.02, rate).unwrap() as usize;
    let x = sine_drive(1, 12.0, rate, 1.0, 997.0);
    let upd_s = rate as f64 / spp3::thermal_frames(rate) as f64;
    let reset_at = (4.0 * upd_s) as usize;
    let rebuild_at = (6.0 * upd_s) as usize;
    let events = [(reset_at, HostEvent::AudioUnitReset), (rebuild_at, HostEvent::Rebuild)];
    let r = spp3_hard_render_ev(&p, &x, &events);
    // after the rebuild the oracle (and, checked every update, the unit)
    // started NORMAL and latched again one full count later
    let k = r.latched_at[0].expect("SafeTlim relatches after the rebuild");
    assert!(k + 1 >= init, "relatched {} updates after the rebuild, the count is {init}", k + 1);
    assert_eq!(r.base[0], 100.0, "latched at SafeTlim");
    // and the AudioUnitReset in between kept the first latch: a run with the
    // reset alone ends latched without relatching
    let r2 = spp3_hard_render_ev(&p, &x, &events[..1]);
    assert!(r2.latched_at[0].is_some_and(|k| k < reset_at), "latched before the reset and kept");
    assert_eq!(r2.base[0], 100.0);
    // exact through the first latch at least (it may turn undecided later, when
    // T_OL hovers within rounding of a threshold under limiting)
    assert!(r.exact > init && r2.exact > init, "the oracle stayed exact for only {} / {} updates", r.exact, r2.exact);
}

/// The SafeTlim predicate is strict: a `T_OL` exactly at `Tlim_eff +
/// offset` reloads the count. A constant drive settles the Float32 thermal
/// nodes on a fixed point; the offset is then chosen so that `Tlim +
/// offset` is that `T_OL` exactly, and a short count must never latch
/// (with `>=` it would after `init` updates).
#[test]
fn spp3_safetlim_predicate_is_strict_at_equality() {
    let rate = 48_000;
    let mut p = safetlim_case(0.002, rate);
    p.ch[0].temperature_limit = 200.0; // far from limiting: unity gain
    p.thermal_safe_tlim = 100.0;
    let x = vec![vec![0.1f32; 48_000 * 120]];
    let mut probe_unit = spp3::Unit::new(p);
    let rr = render(&mut probe_unit, &x, None, &[512], &mut no_sense, true);
    let n = rr.tele.len();
    let t_fix = rr.tele[n - 1].1.channels[0].coil_temp_c;
    assert_eq!(t_fix, rr.tele[n - 100].1.channels[0].coil_temp_c, "control: T_OL settled on a Float32 fixed point");
    // a limit at 1.5 T_OL keeps the candidate at unity (the same trajectory)
    // and makes `t_fix - Tlim` exact (Sterbenz), so `Tlim + offset == t_fix`
    let tlim = 1.5 * t_fix;
    let offset = t_fix - tlim;
    assert!(offset < 0.0 && tlim + offset == t_fix, "control: the threshold is exactly T_OL");
    p.ch[0].temperature_limit = tlim;
    p.thermal_safe_tlim = tlim.min(100.0);
    p.thermal_safe_tlim_offset = offset;
    let r = spp3_hard_render_ev(&p, &x, &[]);
    assert_eq!(r.tele.channels[0].coil_temp_c, t_fix, "control: the same fixed point");
    assert!(r.latched_at[0].is_none(), "the oracle latched at equality");
    assert_eq!(r.base[0], tlim as f64);
    // and the unit agrees (checked every update), still at its limit
    assert_eq!(r.tele.channels[0].tlim_adj_c, 0.0, "the unit latched at equality");
    // control: one f32 step below the fixed point latches
    p.thermal_safe_tlim_offset = f32::from_bits(offset.to_bits() + 1); // more negative
    let r = spp3_hard_render_ev(&p, &x, &[]);
    assert!(r.latched_at[0].is_some(), "control: just below the fixed point latches");
}

/// SafeTlim at the shipped TimeMin values (5 and 10 min) under sustained
/// full-scale drive: latches exactly at the specified count, and the hard
/// path and controller follow the 100 C base afterwards. Long (11 min of
/// audio): `cargo test --release --test adversarial -- --ignored shipped`.
#[test]
#[ignore = "long: 6 and 11 minutes of audio; run with --ignored in release"]
fn spp3_safetlim_at_shipped_time_min() {
    for (time_min, secs) in [(5.0f32, 6.0 * 60.0), (10.0, 11.0 * 60.0)] {
        for rate in [48_000u32, 44_100] {
            let p = safetlim_case(time_min, rate);
            let init = spp3::safe_count_init(time_min, rate).unwrap() as usize;
            let x = sine_drive(1, secs, rate, 1.0, 997.0);
            let r = spp3_hard_render_ev(&p, &x, &[]);
            let k = r.latched_at[0].unwrap_or_else(|| panic!("TimeMin {time_min} at {rate}: never latched"));
            assert!(k + 1 >= init && k < init + 2 * rate as usize / 256, "latched at update {k}, count {init}");
            assert_eq!(r.base[0], 100.0);
            eprintln!(
                "TimeMin {time_min} min at {rate} Hz: latched at update {k} (count {init}), {} exact updates",
                r.exact
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Regression tests for defects found by the tests above (lane B to fix).
// Open ones are `#[ignore]`d; remove the attribute with the fix.

/// Minimal spp3 set for the regressions: one channel, closed loop off.
fn spp3_minimal(ff: f32) -> spp3::Params {
    let mut p = spp3::Params {
        channels: 1,
        sample_rate: 48_000,
        block_frames: 512,
        thermal_ambient_temp: 40.0,
        thermal_gain_attack_time_s: 0.5,
        thermal_gain_release_time_s: 2.0,
        thermal_lookahead_delay_ms: 1.5,
        thermal_protection_enabled: 1,
        // AID35-like SafeTlim (synthetic): base 100 C after 10 min above
        // the limit - 15 C. (TimeMin 0 latches on the first update.)
        thermal_safe_tlim: 100.0,
        thermal_safe_tlim_offset: -15.0,
        thermal_safe_tlim_time_min: 10.0,
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

/// The same with the closed loop configured and a synthetic calibration
/// (4.0 ohm at 25 C, 0.4 %/K, cap 30 K), sense ports (I, V) = (0, 1).
fn spp3_cl() -> spp3::Params {
    let mut p = spp3_minimal(0.5);
    let c = &mut p.ch[0];
    c.cl_thermal_feedback_enabled = 1;
    (c.imon_scale, c.vmon_scale, c.i_pwr_thres, c.v_pwr_thres) = (1.0, 1.0, 1e-6, 1e-6);
    (c.sense_i, c.sense_v) = (0, 1);
    (c.temp_sense_window_time_s, c.temp_sense_smoothing_tau_s, c.rdc_scale) = (0.01, 0.1, 1.0);
    (c.reb_range_min, c.reb_range_max, c.cal_temp_range_min, c.cal_temp_range_max) = (2.0, 8.0, 10.0, 50.0);
    (c.a_t_20c, c.a_t_35c, c.tlim_err_decay_time_s, c.tlim_adj_cap) = (0.004, 0.004, 1.0, 30.0);
    (c.cal_present, c.cal_r, c.cal_t) = (1, 4.0, 25.0);
    // the pilot-tone analyser (method 1): a 0.1 s Goertzel period on the
    // bin `review_common::pilot_wave` drives
    c.thermal_measurement_method = spp3::METHOD_PILOT_TONE;
    c.temp_sense_window_time_s = 0.1;
    (c.pilot_freq, c.pilot_enable_thres) = (43.0664, 0.03);
    (c.pilot_ampl_hi_db, c.pilot_ampl_lo_db) = (-30.0, -40.0);
    (c.pilot_upper_thres, c.pilot_lower_thres) = (90.0, 80.0);
    (c.pilot_decay_time_s, c.pilot_decay_time_stage2_s) = (0.03, 0.03);
    p
}

thread_local! {
    /// Sense phase, continuous across `drive1` calls on one test thread.
    static SENSE_POS: Cell<usize> = const { Cell::new(0) };
}

/// Drive one channel with `x` for `frames` in 512-frame calls (`first`
/// replaces the very first sample). `sense = Some(r)` delivers a V/I pair
/// with resistance `r`; `None` delivers no sense slices. Returns the last
/// call's gain and the telemetry.
fn drive1(
    u: &mut dyn ProtectionUnit,
    x: f32,
    first: Option<f32>,
    frames: usize,
    sense: Option<f32>,
) -> (f32, Telemetry) {
    let (mut done, mut g, mut t) = (0, 0.0, Telemetry::new());
    while done < frames {
        let m = 512.min(frames - done);
        let mut i = vec![x; m];
        if done == 0 {
            if let Some(v) = first {
                i[0] = v;
            }
        }
        // sense at the pilot analyser's bin (a V/I pair of resistance `r`)
        let pos = SENSE_POS.with(|p| p.replace(p.get() + m));
        let cur: Vec<f32> = (0..m).map(|j| 0.5 * review_common::pilot_wave(pos + j)).collect();
        let vol: Vec<f32> = cur.iter().map(|c| c * sense.unwrap_or(0.0)).collect();
        let s: Vec<&[f32]> = if sense.is_some() { vec![&cur, &vol] } else { Vec::new() };
        let mut o = vec![0.0f32; m];
        u.process(Io { frames: m, input: &[&i], sense: &s, output: &mut [&mut o] });
        g = o[m - 1] / x;
        done += m;
    }
    u.telemetry(&mut t);
    (g, t)
}

/// FINDING 1 (critical): one large finite sample (`abs(x) > ~1.8e19`, so `x^2`
/// overflows the f32 `LaneAcc` lanes) makes the modelled power +Inf, so
/// `theta_vc`, `theta_mg` and `theta_ff` go to +Inf for good. The
/// candidates are then `sqrt(clamp(-inf / inf))` = NaN, and `f64::min`
/// drops a NaN, so the group target becomes 1. From then on (until `reset`)
/// spp3 passes full-scale drive at **unity gain** with `T_OL = inf`, whether
/// the feed-forward is on or off. A fresh unit holds the same drive at about
/// 0.34 (ff 0.5) or 0.48 (ff 0).
#[test]
fn spp3_huge_finite_sample_must_not_unprotect() {
    for ff in [0.5f32, 0.0] {
        let (fresh, _) = drive1(&mut spp3::Unit::new(spp3_minimal(ff)), 1.0, None, 48_000 * 10, None);
        assert!(fresh < 0.9, "control: a fresh unit limits full-scale drive (gain {fresh})");
        let mut u = spp3::Unit::new(spp3_minimal(ff));
        drive1(&mut u, 0.0, Some(2e19), 512, None);
        let (g, t) = drive1(&mut u, 1.0, None, 48_000 * 10, None);
        assert!(
            g <= fresh * 1.01 && t.channels[0].coil_temp_c.is_finite(),
            "ff {ff}: after one 2e19 sample, 10 s of full-scale drive leave at gain {g} (fresh unit {fresh}), T_OL {}",
            t.channels[0].coil_temp_c
        );
    }
}

/// FINDING 1, atsp: the same accumulator overflow turns the state NaN and
/// the unit then mutes forever (fail-closed, but never recovers).
#[test]
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
    drive1(&mut u, 0.0, Some(2e19), 512 * 5, None);
    let (g, t) = drive1(&mut u, 1e-3, None, 48_000 * 20, None);
    assert!(t.channels[0].coil_temp_c.is_finite(), "coil temperature is {}", t.channels[0].coil_temp_c);
    assert!(g > 0.5, "stuck muted after one huge sample: gain {g}");
}

/// FINDING 2 (critical): with `ThermalFFSpeedupFactor = 0` (the feed-forward
/// off, a specified configuration) only the hard path acts, and its target
/// takes the real coil as the virtual coil:
/// `g^2 = (Tlim - 5 - ambient - theta_mg) / (theta_vc + 1)`. The real coil's
/// rise scales with `g^2` itself, so this does not hold the coil at the
/// limit: under sustained full-scale drive the unit settles at gain 0.48
/// with the modelled coil at **194 C against a 128 C hard limit**, for as
/// long as the drive lasts (the same drive with the feed-forward on settles
/// at 119.9 C). Module docs: "TODO(SPEC): the hard candidate when the
/// feed-forward filter is disabled".
#[test]
fn spp3_feed_forward_off_still_holds_the_hard_limit() {
    let p = spp3_minimal(0.0);
    let hard_on = p.ch[0].temperature_limit + p.ch[0].hard_temp_limit_headroom;
    let (g, t) = drive1(&mut spp3::Unit::new(p), 1.0, None, 48_000 * 30, None);
    assert!(
        t.channels[0].coil_temp_c <= hard_on + 0.5,
        "after 30 s of full-scale drive the modelled coil is at {} C (hard limit {hard_on} C) with gain {g}",
        t.channels[0].coil_temp_c
    );
}

/// FINDING 3 (high): closed-loop sense loss keeps the last correction. When
/// the sense slices are absent, short or missing ports, `Cl::feed` is not
/// called, so `valid` stays true and `T_CL` stays at the last measurement:
/// the corrected limit keeps (and, as `T_OL` rises, grows) its
/// `TlimAdjCap` allowance with nothing measured. After 30 s with no sense
/// bus the correction is still +30 K, the gain 0.669 and `T_OL` 149.6 C,
/// while the open loop holds the same drive at 0.570 and 119.7 C, and
/// `health` meanwhile reports `OPEN_LOOP`. NaN-filled sense, by contrast,
/// is rejected and the limit returns to plain (the specified behaviour).
#[test]
fn spp3_closed_loop_falls_back_when_the_sense_bus_disappears() {
    let mut u = spp3::Unit::new(spp3_cl());
    let (g0, t0) = drive1(&mut u, 0.6, None, 48_000 * 10, Some(4.0));
    // control: the closed loop runs and accepts windows; the correction is
    // capped at the base (a unit for no proven board, sgv2_core::machine)
    assert!(t0.channels[0].closed_loop, "control: the closed loop runs");
    assert!(t0.channels[0].tlim_adj_c <= 0.0, "control: the closed loop never relaxes the limit");
    assert!(g0 > 0.0);
    let (g, t) = drive1(&mut u, 0.6, None, 48_000 * 30, None);
    let (g_open, _) = drive1(&mut spp3::Unit::new(spp3_minimal(0.5)), 0.6, None, 48_000 * 40, None);
    assert!(t.health.0 & Health::OPEN_LOOP != 0, "health reports the loss");
    assert!(
        t.channels[0].tlim_adj_c < 1.0 && g <= g_open * 1.01,
        "30 s after the sense bus vanished: limit correction {} K, gain {g} vs open loop {g_open}, T_OL {} C",
        t.channels[0].tlim_adj_c,
        t.channels[0].coil_temp_c
    );
}

/// FINDING 4 (high): `TlimAdjCap` is not validated, and `Cl::update` calls
/// `f64::clamp(limit, limit + cap)`, which panics when `cap < 0` or is NaN.
/// A blob with the closed loop configured and such a cap passes `validate`
/// and panics in `process` on the first accepted sense window (an abort
/// inside PipeWire with `panic = "abort"`).
#[test]
fn spp3_hostile_tlim_adj_cap_never_panics() {
    for cap in [-1.0f32, f32::NAN] {
        let mut p = spp3_cl();
        p.ch[0].tlim_adj_cap = cap;
        let b = blob::build(UnitKind::Spp3, 1, 2, 48_000, p.as_bytes());
        // refused at load (the fail-safe) is the fix; accepted must not panic
        let Ok(p) = spp3::Params::from_blob(&b) else { continue };
        let r = std::panic::catch_unwind(|| {
            let mut u = spp3::Unit::new(p);
            drive1(&mut u, 0.6, None, 48_000, Some(4.0));
        });
        assert!(r.is_ok(), "TlimAdjCap {cap}: process() panicked");
    }
}

/// FINDING 5 (medium): `FailSafe::process` (the unit `sgv2-lv2::load_unit`
/// installs for any unusable blob) is not as hardened as the units' own
/// fault path, `failsafe_io`: it slices `out[..frames]` and `input[..frames]`
/// unchecked, so a host buffer shorter than `Io::frames` panics, and it does
/// not sanitize, so a NaN or +-Inf input sample leaves as NaN or +-Inf
/// (`Inf * 0.1 = Inf`: full scale at the amplifier).
#[test]
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
fn failsafe_unit_output_is_finite() {
    let x = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 0.5];
    let mut o = [0.0f32; 4];
    let mut u = FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS);
    u.process(Io { frames: 4, input: &[&x], sense: &[], output: &mut [&mut o] });
    assert!(o.iter().all(|v| v.is_finite()), "fail-safe output {o:?}");
}

/// FINDING 6 (low): spp3 and atsp apply the non-finite ("meter failure")
/// 20 dB to every frame of the *host segment* that holds the bad sample, so
/// where the host split its buffers decides which good frames are cut, and
/// with them the protected drive the model integrates. Output and state
/// then differ between slicings (property 3). The cut is on the safe side,
/// but the trajectory is no longer reproducible against the rig.
#[test]
fn non_finite_failsafe_region_is_slice_invariant() {
    let mut x = vec![0.5f32; 512];
    x[100] = f32::NAN;
    let x = vec![x];
    type Make = Box<dyn Fn() -> Box<dyn ProtectionUnit>>;
    let units: [(&str, Make); 2] = [
        ("spp3", Box::new(|| Box::new(spp3::Unit::new(spp3_minimal(1.0))))),
        (
            "atsp",
            Box::new(|| {
                let mut p = atsp_params(5, 1, 512, 48_000, false);
                p.slot[0].channel_mask = 1;
                (p.slot[1].channel_mask, p.slot[2].channel_mask) = (0, 0);
                Box::new(atsp::Unit::new(p))
            }),
        ),
    ];
    for (name, mk) in units {
        // spp3 snaps its dezipper at each call start (rig dezipper law), so
        // compare call sizes that divide its 256-frame update
        let whole = render(mk().as_mut(), &x, None, &[256], &mut no_sense, false);
        let split = render(mk().as_mut(), &x, None, &[64], &mut no_sense, false);
        assert_eq!(whole.out[0][0], split.out[0][0], "{name}: frame 0 (far from the NaN) depends on the slicing");
        assert!(bits(&whole.out) == bits(&split.out), "{name}: output depends on the slicing");
    }
}

/// FINDING 7 (lane B): blobs that validate and load as a real unit, yet
/// protect nothing (property 5's "never pass-through"). lsm2's
/// `EnableMitigation = 0` and all-axes-off are refused since 660f33d. spp3
/// still accepts limits no speaker has (`TemperatureLimit = f32::MAX`),
/// smoothing times that never move (`f32::MAX` s) with a huge headroom, and
/// unbounded series resistance (`Rshunt`, `Rampout`): the modelled power
/// `ag^2 Reb / (Reb + Rshunt + Rampout)^2` goes to zero, so the model never
/// heats and full scale passes at unity.
#[test]
fn hostile_but_valid_blobs_do_not_pass_through() {
    let x = vec![vec![1.0f32; 48_000 * 10]; 1];
    // the last second of 10 s of full-scale drive leaves at unity
    let passes = |u: &mut dyn ProtectionUnit, lat: usize| {
        let rr = render(u, &x, None, &[512], &mut no_sense, false);
        rr.out[0][lat + 48_000 * 9..].iter().all(|&v| v == 1.0)
    };
    let mut failures: Vec<&str> = Vec::new();
    // lsm2: refused at load (`from_blob` Err or a fail-safe unit) is safe
    let lsm2_case = |p: lsm2::Params, what: &'static str, failures: &mut Vec<&str>| {
        let b = blob::build(UnitKind::Lsm2, 1, 0, 48_000, p.as_bytes());
        if let Ok(p) = lsm2::Params::from_blob(&b) {
            let mut u = lsm2::Unit::new(p);
            let lat = u.latency_frames() as usize;
            if !u.is_failsafe() && passes(&mut u, lat) {
                failures.push(what);
            }
        }
    };
    let mut p = lsm2_params(1, 1, 48_000, true);
    p.control_temperature = 1;
    p.enable_mitigation = 0;
    lsm2_case(p, "lsm2 EnableMitigation=0 loads and passes full scale at unity", &mut failures);
    let mut p = lsm2_params(1, 1, 48_000, true);
    (p.control_sample_peak, p.control_displacement, p.control_velocity) = (0, 0, 0);
    (p.control_power, p.control_temperature) = (0, 0);
    lsm2_case(p, "lsm2 with every Control* flag off loads and passes full scale at unity", &mut failures);
    let spp3_case = |p: spp3::Params, what: &'static str, failures: &mut Vec<&str>| {
        let b = blob::build(UnitKind::Spp3, 1, 0, 48_000, p.as_bytes());
        if let Ok(p) = spp3::Params::from_blob(&b) {
            let mut u = spp3::Unit::new(p);
            if !u.is_failsafe() && passes(&mut u, 0) {
                failures.push(what);
            }
        }
    };
    let mut p = spp3_minimal(1.0);
    p.ch[0].temperature_limit = f32::MAX;
    spp3_case(p, "spp3 TemperatureLimit=f32::MAX loads and passes full scale at unity", &mut failures);
    let mut p = spp3_minimal(1.0);
    p.thermal_gain_attack_time_s = f32::MAX;
    p.ch[0].hard_temp_limit_headroom = 1e30;
    spp3_case(
        p,
        "spp3 Thermal_GainAttackTime_s=f32::MAX (and a huge headroom) passes full scale at unity",
        &mut failures,
    );
    // no upper bound on the series resistances: the modelled power
    // `ag^2 Reb / (Reb + Rshunt + Rampout)^2` goes to 0 and the model never
    // heats
    let mut p = spp3_minimal(1.0);
    p.ch[0].rshunt = 1e30;
    spp3_case(p, "spp3 Rshunt=1e30 loads and passes full scale at unity", &mut failures);
    let mut p = spp3_minimal(1.0);
    p.ch[0].rampout = 1e30;
    spp3_case(p, "spp3 Rampout=1e30 loads and passes full scale at unity", &mut failures);
    let mut p = spp3_minimal(1.0);
    p.ch[0].rshunt = 1e3;
    spp3_case(p, "spp3 Rshunt=1 kohm (finite, implausible) loads and passes full scale at unity", &mut failures);
    assert!(failures.is_empty(), "{failures:#?}");
}

/// FINDING 8 (decision for lane B): lsm2 parses and validates
/// `TemperatureHardLimit` but never uses it: there is no hard path. With
/// `MaxTemperatureAttenuation = 0 dB` (accepted) the coil runs past the hard
/// limit at unity gain.
#[test]
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

/// FINDING 9 (high): the thermal filters are the linear-decay recurrence
/// `theta = (1 - dt/tau) theta + ...` with `dt = 256/fs`, which diverges
/// (alternating, growing) for `tau < dt/2`: 2.7 ms at 48 kHz, 16 ms at
/// 8 kHz, for the coil, the magnet and the virtual coil
/// (`tau_ff = ThermalFFSpeedupFactor * tau_Tvc`). `validate` accepts any
/// positive `tau`. The model reaches +-Inf within a second; depending on
/// which filters diverge the unit then sticks at the 20 dB floor or, with
/// the magnet and virtual coil both gone, releases to unity (0.998 below,
/// where a stable unit holds 0.68) while `T_OL = -inf`.
#[test]
fn spp3_short_time_constants_stay_stable() {
    let stable = {
        let p = spp3_minimal(1.0);
        drive1(&mut spp3::Unit::new(p), 0.5, None, 48_000 * 10, None).0
    };
    let mut p = spp3_minimal(1.0);
    (p.ch[0].tau_tvc, p.ch[0].tau_tmg) = (0.002, 0.002);
    // refused at load is the fix; accepted must stay stable
    if p.validate().is_err() {
        assert!(spp3::Unit::new(p).is_failsafe());
        return;
    }
    let (g, t) = drive1(&mut spp3::Unit::new(p), 0.5, None, 48_000 * 10, None);
    assert!(
        t.channels[0].coil_temp_c.is_finite() && g <= stable.max(0.9),
        "tau 2 ms: gain {g} with T_OL {} (stable constants: gain {stable})",
        t.channels[0].coil_temp_c
    );
}

/// FINDING 10 (low): non-finite or overflowing sense (NaN, Inf, or 1e30,
/// whose square is Inf) is not reported as `OPEN_LOOP`: the update's
/// `complete` no longer checks the sums' finiteness, so `health` reads 0
/// while `sense_power` is NaN and the closed loop rejects every window.
#[test]
fn spp3_non_finite_sense_is_open_loop() {
    for v in [f32::NAN, f32::INFINITY, 1e30] {
        let mut u = spp3::Unit::new(spp3_cl());
        let i = vec![0.5f32; 512];
        let s = vec![v; 512];
        let mut o = vec![0.0f32; 512];
        let h = u.process(Io { frames: 512, input: &[&i], sense: &[&s, &s], output: &mut [&mut o] });
        assert!(h.0 & Health::OPEN_LOOP != 0, "sense {v}: health {h:?}");
    }
}

/// FINDING 12 (low): one NaN (or Inf) sense frame poisons the sense delay
/// estimator for good. Its correlations are exponentially forgotten
/// (`c = c * LAMBDA + v * w`), so a NaN never leaves them, `c > best` is
/// then always false and no delay ever locks again until reset. spp3 feeds
/// the estimator non-finite sense because `complete` no longer checks
/// finiteness (FINDING 10).
#[test]
fn delay_estimator_recovers_from_a_nan_sense_frame() {
    use sgv2_core::sense::DelayEstimator;
    let d = 1536usize;
    let n = 48_000 * 6;
    let mut s = 1u32;
    let prog: Vec<f32> = (0..n)
        .map(|k| {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            let env = if (k / 2400) % 3 == 0 { 1.0 } else { 0.1 };
            ((s >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * env
        })
        .collect();
    let run = |poison: Option<f32>| {
        let mut e = DelayEstimator::new();
        for k in 0..n {
            let o = prog[k] * prog[k];
            let mut v = if k >= d { 0.7 * prog[k - d] * prog[k - d] } else { 0.0 };
            if k == 1000 {
                v = poison.unwrap_or(v);
            }
            e.push(o, v);
            if k % 512 == 511 {
                e.end_block();
            }
        }
        e.delay()
    };
    assert_eq!(run(None), Some(d as u32), "control: the estimator locks");
    for p in [f32::NAN, f32::INFINITY] {
        assert_eq!(run(Some(p)), Some(d as u32), "one {p} sense frame 5 s earlier");
    }
}

/// The spp3 view of FINDING 12: one call of NaN sense, then valid sense
/// delayed by a fixed amount; the unit should lock the delay as a fresh
/// unit does.
#[test]
fn spp3_sense_delay_survives_a_nan_sense_call() {
    let p = spp3_minimal(1.0);
    let x = signal(21, 1, 48_000 * 8, 0.8, Hostility::Clean);
    let s = synth_sense(&x, 2, 1536);
    let fresh = {
        let mut u = spp3::Unit::new(p);
        render(&mut u, &x, Some(&s), &[512], &mut no_sense, false);
        u.sense_delay()
    };
    assert!(fresh.is_some(), "control: a fresh unit locks the sense delay");
    let mut u = spp3::Unit::new(p);
    render(
        &mut u,
        &x,
        Some(&s),
        &[512],
        &mut |call| if call == 2 { Sense::Fill(f32::NAN) } else { Sense::Valid },
        false,
    );
    assert_eq!(u.sense_delay(), fresh, "after one NaN sense call");
}

/// Property 7 on the paths added since the first rt_alloc test: atsp
/// host-block mode, the spp3 closed loop through every sense dropout, lsm2
/// with runtime budgets, and `save_state` (called by the plugin after every
/// `process`), at every awkward call size.
#[test]
fn rt_alloc_covers_the_new_paths() {
    let mut a = atsp_params(4, 4, 512, 48_000, true);
    a.block_frames = 0;
    let mut cl = spp3_cl();
    cl.ch[0].tlim_adj_cap = 10.0;
    let mut l = lsm2_params(4, 2, 48_000, true);
    l.runtime_budget_present = 1;
    let units: Vec<(Box<dyn ProtectionUnit>, usize, usize)> = vec![
        (Box::new(atsp::Unit::new(a)), 4, 0),
        (Box::new(spp3::Unit::new(cl)), 1, 2),
        (Box::new(spp3::Unit::new(spp3_params(9, 6, 512, 48_000, Ff::Mixed, true))), 6, 12),
        (Box::new(lsm2::Unit::new(l)), 2, 0),
    ];
    for (mut u, n, ports) in units {
        assert!(!u.is_failsafe_unit(), "{:?} built as the fail-safe", u.kind());
        let x = signal(5, n, 48_000 * 2, 1.5, Hostility::Wild);
        let s = synth_sense(&x, ports, 0);
        let mut r = Rng(3);
        // render asserts zero allocations in process, telemetry and save_state
        render(u.as_mut(), &x, Some(&s), &[1, 7, 64, 255, 256, 257, 4096], &mut |_| sense_mode(&mut r), false);
    }
}

trait FailSafeUnit {
    fn is_failsafe_unit(&self) -> bool;
}

impl FailSafeUnit for Box<dyn ProtectionUnit> {
    fn is_failsafe_unit(&self) -> bool {
        let mut t = Telemetry::new();
        self.telemetry(&mut t);
        t.health.0 & Health::NO_PARAMS != 0
    }
}

/// FINDING 16 (aarch64, superseded by B5): the closed loop's window sums
/// depended on the host slicing (NEON `sum_sq` per segment). B5 measures
/// with a per-sample Float32 Goertzel; as for the open loop, the output
/// now depends on the slicing by law (the dezipper snaps to the target at
/// each call, and a call holding a whole 512-frame block takes its sense
/// ahead). What must hold: call sizes that divide 256 agree bit for bit,
/// and any fixed call size is deterministic.
#[test]
fn spp3_closed_loop_is_slice_invariant() {
    let p = spp3_cl();
    let x = signal(31, 1, 48_000 * 4, 0.7, Hostility::Clean);
    let i: Vec<f32> = (0..x[0].len()).map(|k| 0.5 * review_common::pilot_wave(k)).collect();
    let v: Vec<f32> = i.iter().map(|c| c * 4.3).collect();
    let s = vec![i, v];
    let reference = render(&mut spp3::Unit::new(p), &x, Some(&s), &[256], &mut no_sense, false);
    assert!(reference.tele[0].1.channels[0].closed_loop, "control: the closed loop runs");
    // the correction is capped at the base (no proven board)
    assert!(reference.tele[0].1.channels[0].tlim_adj_c <= 0.0, "control: the closed loop never relaxes the limit");
    for cuts in [vec![1usize], vec![16], vec![64], vec![128]] {
        let r = render(&mut spp3::Unit::new(p), &x, Some(&s), &cuts, &mut no_sense, false);
        assert_eq!(tele_bits(&r.tele[0].1), tele_bits(&reference.tele[0].1), "state differs with slices {cuts:?}");
        assert!(bits(&r.out) == bits(&reference.out), "audio differs with slices {cuts:?}");
    }
    for cuts in [vec![512usize], vec![333], vec![4096]] {
        let a = render(&mut spp3::Unit::new(p), &x, Some(&s), &cuts, &mut no_sense, false);
        let b = render(&mut spp3::Unit::new(p), &x, Some(&s), &cuts, &mut no_sense, false);
        assert_eq!(tele_bits(&a.tele[0].1), tele_bits(&b.tele[0].1), "not deterministic at {cuts:?}");
        assert!(bits(&a.out) == bits(&b.out), "audio not deterministic at {cuts:?}");
    }
}

//! Shared harness for the sgv2-core fuzz targets: input decoding, the host
//! loop, and the property checks (see `tests/adversarial.rs` for the
//! properties). Every check is an `assert!`, so a violation is a crash
//! libFuzzer records and minimises.
//!
//! Targets: `blob` (property 5, through every `from_blob` and
//! `sgv2_lv2::load_unit`), `spp3`, `atsp` and `lsm2` (properties 1-4, 6, 7
//! under fuzzed parameter words, audio, host slicing and sense). Run from
//! `crates/sgv2-core` with cargo-fuzz; a stable toolchain works with
//! `RUSTC_BOOTSTRAP=1` (cargo-fuzz passes `-Zsanitizer`):
//!
//! ```text
//! RUSTC_BOOTSTRAP=1 cargo fuzz run -O blob -- -max_total_time=900
//! RUSTC_BOOTSTRAP=1 cargo fuzz run -O -s none --target-dir fuzz/target-nosan spp3 -- -max_len=65536
//! ```
//!
//! spp3 and atsp have no `unsafe` on x86_64, so they run fastest without a
//! sanitizer; lsm2 (the `Delay` pointer code) and blob run under ASan.
//!
//! Known defects are kept out of the generated inputs so the fuzzer keeps
//! looking for new ones; each exclusion names its finding:
//! - FINDING 1: finite samples are clamped to +-1e15 (larger ones overflow
//!   the f32 block accumulators and poison the thermal state);
//! - FINDING 3: spp3 sense dropouts are only compared with the references
//!   when no channel runs the closed loop;
//! - FINDING 4: a negative or NaN `TlimAdjCap` is replaced by its magnitude
//!   (or 0) before spp3 is built;
//! - FINDING 5: the `FailSafe` unit is only probed with finite input;
//! - FINDING 6: spp3/atsp slice invariance is only checked on finite input;
//! - FINDING 9: spp3 thermal time constants are raised to at least one
//!   update after mutation;
//! - FINDING 11: atsp host-block mode is fed calls of 64 frames or more.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use arbitrary::{Result, Unstructured};
use sgv2_core::{HandOff, Health, Io, ProtectionUnit, Telemetry, FAILSAFE_GAIN};

pub use arbitrary;

// ---------------------------------------------------------------------------
// Property 7: count allocations on this thread.

pub struct Counting;

thread_local! {
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

// SAFETY: forwards to the system allocator; the counter never allocates.
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

pub fn allocs() -> usize {
    ALLOCS.with(|c| c.get())
}

// ---------------------------------------------------------------------------
// Decoding.

/// Largest finite magnitude fed to the units (FINDING 1).
pub const MAX_FINITE: f32 = 1e15;

pub fn sample(bits: u32) -> f32 {
    let x = f32::from_bits(bits);
    if x.is_finite() {
        x.clamp(-MAX_FINITE, MAX_FINITE)
    } else {
        x
    }
}

/// `n` channels of up to `max_len` frames: short fuzzer-chosen patterns
/// (any bit pattern, so NaN, Inf and subnormals included) tiled over
/// segments with fuzzer-chosen gains, so long, loud signals stay cheap.
pub fn signal(u: &mut Unstructured, n: usize, max_len: usize) -> Result<Vec<Vec<f32>>> {
    // mostly short (cheap executions), sometimes up to `max_len` (long
    // enough to heat the coil past its limits)
    let len = if u.ratio(1, 4)? { u.int_in_range(1..=max_len)? } else { u.int_in_range(1..=4096.min(max_len))? };
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let plen = u.int_in_range(1..=64usize)?;
        let mut pat = Vec::with_capacity(plen);
        for _ in 0..plen {
            // mostly audio-range values, sometimes raw bit patterns
            let v = if u.ratio(1, 4)? {
                sample(u.arbitrary()?)
            } else {
                (u.int_in_range(-32768..=32767i32)? as f32) / 32768.0
            };
            pat.push(v);
        }
        let mut v = Vec::with_capacity(len);
        let mut k = 0usize;
        while v.len() < len {
            let seg = u.int_in_range(1..=16384usize)?.min(len - v.len());
            let gain = u.int_in_range(0..=255u8)? as f32 / 64.0;
            for _ in 0..seg {
                let s = pat[k % plen];
                v.push(if s.is_finite() { s * gain } else { s });
                k += 1;
            }
            if u.is_empty() {
                // out of entropy: repeat the pattern at the last gain
                while v.len() < len {
                    let s = pat[k % plen];
                    v.push(if s.is_finite() { s * gain } else { s });
                    k += 1;
                }
            }
        }
        out.push(v);
    }
    Ok(out)
}

pub fn finite_only(x: &mut [Vec<f32>]) {
    for c in x.iter_mut() {
        for v in c.iter_mut() {
            if !v.is_finite() {
                *v = 0.0;
            }
        }
    }
}

pub fn all_finite(x: &[Vec<f32>]) -> bool {
    x.iter().all(|c| c.iter().all(|v| v.is_finite()))
}

/// Host slices, 1..=4096 frames each.
pub fn slices(u: &mut Unstructured) -> Result<Vec<usize>> {
    let k = u.int_in_range(1..=32usize)?;
    let mut v = Vec::with_capacity(k);
    for _ in 0..k {
        v.push(u.int_in_range(1..=4096usize)?);
    }
    Ok(v)
}

pub fn hostile_word(u: &mut Unstructured) -> Result<u32> {
    Ok(match u.int_in_range(0..=15u8)? {
        0 => f32::NAN.to_bits(),
        1 => f32::INFINITY.to_bits(),
        2 => f32::NEG_INFINITY.to_bits(),
        3 => 0,
        4 => (-0.0f32).to_bits(),
        5 => f32::MAX.to_bits(),
        6 => f32::MIN.to_bits(),
        7 => f32::MIN_POSITIVE.to_bits(),
        8 => 1,
        9 => u32::MAX,
        10 => 0x8000_0000,
        11 => 16_385,
        12 => (-1.0f32).to_bits(),
        13 => 1e30f32.to_bits(),
        14 => u.int_in_range(0..=9u32)?,
        _ => u.arbitrary()?,
    })
}

/// Overwrite or rescale up to 12 four-byte words of a `Pod` payload.
pub fn mutate_words(bytes: &mut [u8], u: &mut Unstructured) -> Result<()> {
    let words = bytes.len() / 4;
    if words == 0 {
        return Ok(());
    }
    let k = u.int_in_range(0..=12usize)?;
    for _ in 0..k {
        let i = u.choose_index(words)? * 4;
        let old = u32::from_le_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]);
        let new = match u.int_in_range(0..=2u8)? {
            0 => hostile_word(u)?,
            1 => {
                // rescale an f32 field by 2^k: explores ranges, keeps sign
                let e = u.int_in_range(-60..=60i32)?;
                (f32::from_bits(old) * 2f32.powi(e)).to_bits()
            }
            _ => old ^ (1 << u.int_in_range(0..=31u32)?),
        };
        bytes[i..i + 4].copy_from_slice(&new.to_le_bytes());
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// The host loop.

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sense {
    Valid,
    Empty,
    Missing,
    Short,
    Fill(f32),
}

pub fn sense_mode(b: u8) -> Sense {
    match b % 9 {
        0 => Sense::Empty,
        1 => Sense::Missing,
        2 => Sense::Short,
        3 => Sense::Fill(f32::NAN),
        4 => Sense::Fill(f32::INFINITY),
        5 => Sense::Fill(1e30),
        6 => Sense::Fill(0.0),
        7 => Sense::Fill(-3.0),
        _ => Sense::Valid,
    }
}

pub struct Render {
    pub out: Vec<Vec<f32>>,
    pub tele: Telemetry,
}

/// Feed `x` through `unit` in `cuts` (cycled). `modes[call % len]` picks the
/// sense delivered on each call (all `Valid` when empty). Asserts property 7.
/// `each` sees the telemetry after every call with the frames done so far.
pub fn render(
    unit: &mut dyn ProtectionUnit,
    x: &[Vec<f32>],
    sense: &[Vec<f32>],
    cuts: &[usize],
    modes: &[u8],
    each: &mut Each<'_>,
) -> Render {
    let len = x[0].len();
    let mut out = vec![vec![0.0f32; len]; x.len()];
    let mut t = Telemetry::new();
    let mut scratch: Vec<Vec<f32>> = sense.to_vec();
    let mut handoff = HandOff::EMPTY;
    let mut done = 0;
    let mut call = 0;
    while done < len {
        let m = cuts[call % cuts.len()].min(len - done);
        let mode = if modes.is_empty() { Sense::Valid } else { sense_mode(modes[call % modes.len()]) };
        if let Sense::Fill(f) = mode {
            for v in scratch.iter_mut() {
                v[done..done + m].fill(f);
            }
        }
        let ins: Vec<&[f32]> = x.iter().map(|b| &b[done..done + m]).collect();
        let sns: Vec<&[f32]> = match mode {
            Sense::Empty => Vec::new(),
            Sense::Missing => scratch.iter().take(scratch.len() / 2).map(|b| &b[done..done + m]).collect(),
            Sense::Short => scratch.iter().map(|b| &b[done..done + m / 2]).collect(),
            _ => scratch.iter().map(|b| &b[done..done + m]).collect(),
        };
        let mut outs: Vec<&mut [f32]> = out.iter_mut().map(|b| &mut b[done..done + m]).collect();
        let before = allocs();
        let _h: Health = unit.process(Io { frames: m, input: &ins, sense: &sns, output: &mut outs });
        unit.telemetry(&mut t);
        let _ = unit.save_state(&mut handoff);
        assert_eq!(allocs(), before, "{:?} allocated in process/telemetry/save_state ({m} frames)", unit.kind());
        drop(outs);
        done += m;
        call += 1;
        each(done, &t, &out);
    }
    Render { out, tele: t }
}

/// Called after every host call with the frames done, the telemetry and the
/// output so far.
pub type Each<'a> = dyn FnMut(usize, &Telemetry, &[Vec<f32>]) + 'a;

pub fn nothing(_: usize, _: &Telemetry, _: &[Vec<f32>]) {}

// ---------------------------------------------------------------------------
// Checks.

/// Properties 1 and 2: finite, and never louder than `gain` times the input
/// `latency` frames earlier (a non-finite input sample leaves as 0).
pub fn check_bound(x: &[Vec<f32>], out: &[Vec<f32>], latency: usize, gain: f32, what: &str) {
    for (c, (xi, yo)) in x.iter().zip(out).enumerate() {
        for (k, &o) in yo.iter().enumerate() {
            assert!(o.is_finite(), "{what}: non-finite output {o} at ch{c}[{k}]");
            let i = if k >= latency { xi[k - latency] } else { 0.0 };
            let lim = if i.is_finite() { (i * gain).abs() } else { 0.0 };
            assert!(o.abs() <= lim, "{what}: |out| {o:e} > {lim:e} (in {i:e}) at ch{c}[{k}]");
        }
    }
}

/// Property 6: `y` is nowhere louder than `reference`.
pub fn check_not_louder(y: &[Vec<f32>], reference: &[Vec<f32>], what: &str) {
    for (c, (a, b)) in y.iter().zip(reference).enumerate() {
        for (k, (&p, &q)) in a.iter().zip(b).enumerate() {
            assert!(p.abs() <= q.abs(), "{what}: ch{c}[{k}] {p:e} louder than {q:e}");
        }
    }
}

/// Property 4, used from `render`'s `each` with one block per call: after a
/// block boundary at which a channel's coil was past its hard limit, every
/// channel of its group leaves the next block at least 20 dB down.
pub struct HardPath<'a> {
    pub x: &'a [Vec<f32>],
    /// Per host channel: (group, hard limit).
    pub chans: Vec<(usize, f64)>,
    pub armed: Vec<usize>,
    pub last: usize,
    pub hits: usize,
}

impl HardPath<'_> {
    pub fn step(&mut self, done: usize, t: &Telemetry, out: &[Vec<f32>]) {
        for (c, &(g, _)) in self.chans.iter().enumerate() {
            if !self.armed.contains(&g) {
                continue;
            }
            let block = self.x[c][self.last..done].iter().zip(&out[c][self.last..done]);
            for (k, (&i, &o)) in block.enumerate() {
                let lim = if i.is_finite() { (i * FAILSAFE_GAIN).abs() } else { 0.0 };
                let k = self.last + k;
                assert!(o.abs() <= lim, "hard path missed: ch{c}[{k}] {o} from {i} (group {g})");
            }
        }
        if !self.armed.is_empty() {
            self.hits += 1;
        }
        self.armed.clear();
        for (c, &(g, limit)) in self.chans.iter().enumerate() {
            let temp = t.channels[c].coil_temp_c as f64;
            // f32 telemetry: skip boundaries the rounding could decide either
            // way, and temperatures beyond f32 range
            if temp.is_finite() && temp > limit + 1e-4 * limit.abs().max(1.0) && !self.armed.contains(&g) {
                self.armed.push(g);
            }
        }
        self.last = done;
    }
}

pub fn tele_bits(t: &Telemetry) -> Vec<u64> {
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

pub fn bits(out: &[Vec<f32>]) -> Vec<Vec<u32>> {
    out.iter().map(|c| c.iter().map(|v| v.to_bits()).collect()).collect()
}

/// Plausible V/I for the drive: `v = 10 x`, `i = v / 4.2`, delayed.
pub fn synth_sense(x: &[Vec<f32>], ports: usize, delay: usize) -> Vec<Vec<f32>> {
    let len = x[0].len();
    (0..ports)
        .map(|p| {
            let s = &x[(p / 2) % x.len()];
            (0..len)
                .map(|k| {
                    let v = if k >= delay { s[k - delay] } else { 0.0 };
                    let v = if v.is_finite() { v.clamp(-1.0, 1.0) } else { 0.0 };
                    if p % 2 == 1 {
                        10.0 * v
                    } else {
                        10.0 * v / 4.2
                    }
                })
                .collect()
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Synthetic base parameter sets (the crate's own test builders, generalised
// to `n` channels). No Mac's tuning.

pub fn spp3_base(n: usize, block: u32, rate: u32, cl: bool) -> sgv2_core::spp3::Params {
    use sgv2_core::spp3::{ChannelParams, Params};
    let mut p = Params {
        channels: n as u32,
        sample_rate: rate,
        block_frames: block,
        thermal_ambient_temp: 40.0,
        thermal_gain_attack_time_s: 0.5,
        thermal_gain_release_time_s: 2.0,
        thermal_lookahead_delay_ms: 1.5,
        thermal_protection_enabled: 1,
        ..Params::default()
    };
    for c in 0..n {
        p.ch[c] = ChannelParams {
            source: c as u32,
            group: if c < n.div_ceil(2) { 7 } else { 3 },
            t_sett_vc: 20.0 + c as f32,
            t_sett_mg: 10.0,
            tau_tvc: 0.4,
            tau_tmg: 3.0,
            temperature_limit: 120.0,
            hard_temp_limit_headroom: 8.0,
            reb_ref: 4.0,
            rshunt: 0.2,
            thermal_ff_speedup_factor: 0.5,
            cl_thermal_feedback_enabled: 1,
            ag: 10.0,
            // with these scales `synth_sense` reads 4.2 ohm
            imon_scale: 1.0,
            vmon_scale: 1.0,
            i_pwr_thres: 1e-6,
            v_pwr_thres: 1e-6,
            sense_i: 2 * c as u32,
            sense_v: 2 * c as u32 + 1,
            pilot_ampl_hi_db: -30.0,
            temp_sense_window_time_s: 0.01,
            temp_sense_smoothing_tau_s: 0.1,
            rdc_scale: 1.0,
            reb_range_min: 2.0,
            reb_range_max: 8.0,
            cal_temp_range_min: 10.0,
            cal_temp_range_max: 50.0,
            a_t_20c: 0.004,
            a_t_35c: 0.0038,
            tlim_err_decay_time_s: 1.0,
            tlim_adj_cap: 30.0,
            cal_present: cl as u32,
            cal_r: 4.0,
            cal_t: 25.0,
            ..Default::default()
        };
    }
    p
}

/// FINDING 4 and 9 exclusions (see the module docs).
pub fn spp3_exclusions(p: &mut sgv2_core::spp3::Params) {
    let dt = sgv2_core::spp3::THERMAL_FRAMES as f32 / p.sample_rate.max(1) as f32;
    for c in p.ch.iter_mut() {
        c.tlim_adj_cap = if c.tlim_adj_cap.is_nan() { 0.0 } else { c.tlim_adj_cap.abs() };
        let f = c.thermal_ff_speedup_factor;
        let shortest = if f > 0.0 && f.is_finite() { f.min(1.0) } else { 1.0 };
        if c.tau_tvc.is_finite() {
            c.tau_tvc = c.tau_tvc.max(dt / shortest);
        }
        if c.tau_tmg.is_finite() {
            c.tau_tmg = c.tau_tmg.max(dt);
        }
    }
}

/// Whether any spp3 channel would run the closed loop (a superset: the unit
/// also needs the calibration inside its ranges).
pub fn spp3_closed_loop(p: &sgv2_core::spp3::Params) -> bool {
    p.ch[..(p.channels as usize).min(8)]
        .iter()
        .any(|c| c.cl_thermal_feedback_enabled != 0 && c.cal_present != 0 && c.ignore_telemetry == 0)
}

pub fn atsp_base(n: usize, block: u32, rate: u32) -> sgv2_core::atsp::Params {
    use sgv2_core::atsp::{Params, Slot};
    let s = Slot {
        channel_mask: (1u32 << n) - 1,
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
    let mut b = s;
    b.channel_mask = 0;
    Params { channels: n as u32, sample_rate: rate, block_frames: block, bypass: 0, gain_link_all: 0, slot: [s, b, b] }
}

pub fn lsm2_base(n: usize, rate: u32) -> sgv2_core::lsm2::Params {
    use sgv2_core::lsm2::{ChannelParams, Params};
    let mut p = Params {
        channels: n as u32,
        sample_rate: rate,
        enable_mitigation: 1,
        control_sample_peak: 1,
        control_displacement: 1,
        control_velocity: 1,
        control_power: 1,
        control_temperature: 1,
        link_slow_gains: 1,
        model_only: 1,
        attack_time_ms: 2.0,
        fast_release_time_ms: 150.0,
        slow_release_time_ms: 10_000.0,
        filterbank_latency_ms: 2.5,
        speaker_response_lookahead_ms: 1.0,
        power_time_scale_s: [0.002, 0.2, 2.0, 30.0],
        power_hard_limit_w: [50.0, 20.0, 2.0, 1.5],
        power_soft_limit_pct: [100.0; 4],
        max_power_attenuation_db: 4.0,
        low_rail_idle_power_w: 0.0,
        low_rail_power_ratio: 1.0,
        ..Params::default()
    };
    for c in 0..n {
        p.ch[c] = ChannelParams {
            source: c as u32,
            volts_full_scale: 6.0,
            parasitic_resistance: 0.2,
            rdc: 4.0,
            bl: 0.8,
            mms_kg: 0.0002,
            rms: 0.2,
            kms: 1500.0,
            disp_bl: 0.8,
            disp_mms_kg: 0.00022,
            disp_rms: 0.25,
            disp_kms: 1600.0,
            rth_vc: 30.0,
            tau_vc: 2.0,
            rth_mag: 20.0,
            tau_mag: 60.0,
            ambient: 40.0,
            amplitude_limit: 1.0,
            displacement_limit: 0.0005,
            velocity_limit0: 0.5,
            max_velocity_attenuation_db: 18.0,
            temperature_soft_limit: 110.0,
            temperature_hard_limit: 125.0,
            temperature_headroom: 4.0,
            max_temperature_attenuation_db: 10.0,
        };
    }
    p
}

pub fn rate(u: &mut Unstructured) -> Result<u32> {
    Ok(match u.int_in_range(0..=4u8)? {
        0 => 44_100,
        1 => 96_000,
        2 => u.int_in_range(8_000..=192_000u32)?,
        _ => 48_000,
    })
}

pub fn block(u: &mut Unstructured) -> Result<u32> {
    Ok(match u.int_in_range(0..=3u8)? {
        0 => u.int_in_range(1..=2048u32)?,
        1 => 256,
        _ => 512,
    })
}

// ---------------------------------------------------------------------------
// The spp3 hard-path oracle (the same as in tests/adversarial.rs).

/// The spp3 hard-path oracle; see `HardOracle` in `tests/adversarial.rs`.
pub struct Spp3HardOracle {
    amb: f64,
    k_dez: f64,
    ch: Vec<OracleChan>,
    /// Per group: the largest allowed target and the dezipper stages it drives.
    allowed: Vec<f64>,
    z1: Vec<f64>,
    z2: Vec<f64>,
    /// Updates at which some channel was certainly armed.
    pub armed_updates: usize,
}

struct OracleChan {
    src: usize,
    group: usize,
    elec: f64,
    /// Virtual-coil filter `(a, b)`, `None` with the feed-forward off.
    ff: Option<(f64, f64)>,
    theta_ff: f64,
    p_prev: f64,
    hist: std::collections::VecDeque<f32>,
    look: usize,
    limit: f64,
    hard_on: f64,
    headroom: f32,
    armed: Option<bool>,
}

impl Spp3HardOracle {
    pub fn new(p: &sgv2_core::spp3::Params) -> Self {
        let n = p.channels as usize;
        let rate = p.sample_rate as f64;
        let dt = sgv2_core::spp3::THERMAL_FRAMES as f64 / rate;
        let look = (p.thermal_lookahead_delay_ms as f64 * 1e-3 * rate).round() as usize;
        let mut ids: Vec<u32> = Vec::new();
        let ch = p.ch[..n]
            .iter()
            .map(|c| {
                let group = ids.iter().position(|&g| g == c.group).unwrap_or_else(|| {
                    ids.push(c.group);
                    ids.len() - 1
                });
                let (reb, series, ag) = (c.reb_ref as f64, (c.rshunt + c.rampout) as f64, c.ag as f64);
                let f = c.thermal_ff_speedup_factor as f64;
                let tau = f * c.tau_tvc as f64;
                OracleChan {
                    src: c.source as usize,
                    group,
                    elec: ag * ag * reb / ((reb + series) * (reb + series)),
                    ff: (f > 0.0).then(|| (1.0 - dt / tau, 0.5 * c.t_sett_vc as f64 * dt / tau)),
                    theta_ff: 0.0,
                    p_prev: 0.0,
                    hist: std::collections::VecDeque::from(vec![0.0; look]),
                    look,
                    limit: c.temperature_limit as f64,
                    hard_on: c.temperature_limit as f64 + c.hard_temp_limit_headroom as f64,
                    headroom: if c.pilot_tone_enabled != 0 {
                        (1.0 - 10f64.powf(c.pilot_ampl_hi_db as f64 / 20.0)) as f32
                    } else {
                        1.0
                    },
                    armed: Some(false),
                }
            })
            .collect();
        let g = ids.len();
        Spp3HardOracle {
            amb: p.thermal_ambient_temp as f64,
            k_dez: 1.0 - (0.01f64.ln() / (0.005 * rate)).exp(),
            ch,
            allowed: vec![1.0; g],
            z1: vec![1.0; g],
            z2: vec![1.0; g],
            armed_updates: 0,
        }
    }

    /// One thermal update: `x` and `out` are the update's 256 frames per host
    /// channel, `t` the telemetry after it.
    pub fn step(&mut self, x: &[&[f32]], out: &[&[f32]], t: &Telemetry) {
        let frames = x[0].len();
        // the audio of this update follows the targets allowed at the last one
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
                // two f32 roundings (gain, headroom), which on a subnormal input
                // are absolute: allow two subnormal steps
                let bound = (i.abs() as f64) * (self.z2[c.group] * c.headroom as f64 * (1.0 + 1e-4) + 1e-6)
                    + 2.0 * f32::from_bits(1) as f64;
                let o = out[c.src][k].abs() as f64;
                assert!(
                    o <= bound,
                    "hard path: ch{} sample {k} of the update leaves at {:.6} of its input, allowed {:.6}",
                    c.src,
                    o / i.abs() as f64,
                    self.z2[c.group] * c.headroom as f64
                );
            }
        }
        // the virtual coil, on the unprotected drive through the alignment
        // history
        for c in self.ch.iter_mut() {
            let mut acc = 0.0f64;
            for &v in x[c.src] {
                c.hist.push_back(v);
                let d = if c.look == 0 { c.hist.pop_back().unwrap() } else { c.hist.pop_front().unwrap() };
                // the program is held to full scale before the meters
                let d = if d.is_finite() { d.clamp(-1.0, 1.0) as f64 } else { 0.0 };
                acc += d * d;
            }
            let p = c.elec * acc / frames as f64;
            if let Some((a, b)) = c.ff {
                c.theta_ff = a * c.theta_ff + b * (p + c.p_prev);
            }
            c.p_prev = p;
        }
        // arming, from the published temperatures
        self.allowed.iter_mut().for_each(|a| *a = 1.0);
        let mut any = false;
        for c in self.ch.iter_mut() {
            let s = &t.channels[c.src];
            let (t_ol, mag) = (s.coil_temp_c as f64, s.magnet_temp_c as f64);
            if !(t_ol.is_finite() && mag.is_finite()) {
                c.armed = None;
                continue;
            }
            let tol = 1e-4 * t_ol.abs().max(1.0);
            let prev = c.armed;
            c.armed = if t_ol > c.hard_on + tol {
                Some(true)
            } else if t_ol < c.hard_on - tol {
                if t_ol < c.limit - tol {
                    Some(false)
                } else if t_ol > c.limit + tol {
                    prev
                } else {
                    None
                }
            } else {
                None
            };
            if c.armed != Some(true) {
                continue;
            }
            any = true;
            // without the feed-forward there is no virtual coil: the hard
            // path holds the fixed 20 dB
            if c.ff.is_none() {
                self.allowed[c.group] = self.allowed[c.group].min(FAILSAFE_GAIN as f64);
                continue;
            }
            let theta_mg = mag - self.amb;
            let vcff = c.theta_ff * (1.0 - 1e-4) - 1e-6;
            if vcff + 1.0 <= 0.0 {
                continue;
            }
            let num = c.limit - 5.0 - self.amb - theta_mg + 1e-4 * (c.limit.abs() + mag.abs() + 1.0);
            // ratio floor 1e-4: a 0.01 (-40 dB) gain floor
            let g_hard = (num / (vcff + 1.0)).clamp(1e-4, 1.0).sqrt();
            self.allowed[c.group] = self.allowed[c.group].min(g_hard);
        }
        if any {
            self.armed_updates += 1;
        }
        // the unit's own selected target never exceeds what is allowed
        for c in &self.ch {
            let sel = t.channels[c.src].axis_gain[4] as f64;
            let a = self.allowed[c.group];
            assert!(sel <= a * (1.0 + 1e-4) + 1e-6, "hard path: group target {sel} above the hard target {a}");
        }
    }
}

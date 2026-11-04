//! Shared synthetic parameter sets and drivers for the safety-review
//! regression tests (synthetic values only; no Mac's tuning appears here).
#![allow(dead_code)]

use sgv2_core::{atsp, lsm2, spp3, Health, Io, ProtectionUnit, Telemetry};

pub const RATE: u32 = 48_000;

pub fn spp3_params() -> spp3::Params {
    let mut p = spp3::Params {
        channels: 4,
        sample_rate: RATE,
        block_frames: 512,
        thermal_ambient_temp: 40.0,
        thermal_gain_attack_time_s: 0.5,
        thermal_gain_release_time_s: 2.0,
        thermal_lookahead_delay_ms: 1.5,
        thermal_protection_enabled: 1,
        ..spp3::Params::default()
    };
    for c in 0..4 {
        p.ch[c] = spp3::ChannelParams {
            source: c as u32,
            group: if c < 2 { 7 } else { 3 },
            ignore_telemetry: 0,
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
            rampout: 0.0,
            ag: 10.0,
            imon_scale: 2.0,
            vmon_scale: 12.0,
            i_pwr_thres: 0.0,
            v_pwr_thres: 0.0,
            sense_i: 2 * c as u32,
            sense_v: 2 * c as u32 + 1,
            initial_theta_vc: 0.0,
            initial_theta_mg: 0.0,
            pilot_tone_enabled: 0,
            pilot_ampl_hi_db: -30.0,
            temp_sense_window_time_s: 0.1,
            temp_sense_smoothing_tau_s: 0.5,
            rdc_scale: 1.0,
            reb_range_min: 2.0,
            reb_range_max: 8.0,
            cal_temp_range_min: 10.0,
            cal_temp_range_max: 50.0,
            a_t_20c: 0.004,
            a_t_35c: 0.0038,
            tlim_err_decay_time_s: 5.0,
            tlim_adj_cap: 30.0,
            cal_present: 0,
            cal_r: 0.0,
            cal_t: 0.0,
        };
    }
    p
}

/// Closed loop with a calibration of 4 ohm at 25 C on every channel.
pub fn spp3_cl_params(cap: f32) -> spp3::Params {
    let mut p = spp3_params();
    for c in 0..4 {
        p.ch[c].cal_present = 1;
        p.ch[c].cal_r = 4.0;
        p.ch[c].cal_t = 25.0;
        p.ch[c].tlim_adj_cap = cap;
    }
    p
}

/// One host call of `n` frames at level `x` on every channel; `sense` is
/// `(i, v)` on all eight sense ports (alternating sign, so not DC), with
/// `sense_len` samples per port. Returns channel 0's output.
pub fn blk_s(u: &mut dyn ProtectionUnit, x: f32, n: usize, sense: Option<(f32, f32)>, sense_len: usize) -> Vec<f32> {
    let input: Vec<Vec<f32>> = (0..4).map(|_| vec![x; n]).collect();
    let mut output = vec![vec![0.0f32; n]; 4];
    let ins: Vec<&[f32]> = input.iter().map(|b| &b[..]).collect();
    let mut outs: Vec<&mut [f32]> = output.iter_mut().map(|b| &mut b[..]).collect();
    let sb: Vec<Vec<f32>> = match sense {
        Some((i, v)) => (0..8)
            .map(|k| {
                (0..sense_len).map(|j| if k % 2 == 0 { i } else { v } * if j % 2 == 0 { 1.0 } else { -1.0 }).collect()
            })
            .collect(),
        None => vec![],
    };
    let sns: Vec<&[f32]> = sb.iter().map(|b| &b[..]).collect();
    u.process(Io { frames: n, input: &ins, sense: &sns, output: &mut outs });
    output.swap_remove(0)
}

pub fn blk(u: &mut dyn ProtectionUnit, x: f32) -> f32 {
    blk_s(u, x, 512, None, 0)[511]
}

pub fn tel(u: &dyn ProtectionUnit) -> Telemetry {
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    t
}

pub fn secs(s: f64) -> usize {
    (s * RATE as f64 / 512.0) as usize
}

pub fn atsp_params() -> atsp::Params {
    let s = atsp::Slot {
        channel_mask: 0b1111,
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
    atsp::Params {
        channels: 4,
        sample_rate: RATE,
        block_frames: 512,
        bypass: 0,
        gain_link_all: 0,
        slot: [s, atsp::Slot::default(), atsp::Slot::default()],
    }
}

pub fn lsm2_params() -> lsm2::Params {
    let mut p = lsm2::Params {
        channels: 2,
        sample_rate: RATE,
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
        ..lsm2::Params::default()
    };
    for c in 0..2 {
        p.ch[c] = lsm2::ChannelParams {
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

pub fn lsm2_go(u: &mut lsm2::Unit, x: &[f32]) -> (Vec<f32>, Health) {
    let n = x.len();
    let ins: Vec<&[f32]> = vec![x, x];
    let mut o0 = vec![0.0f32; n];
    let mut o1 = vec![0.0f32; n];
    let mut outs: Vec<&mut [f32]> = vec![&mut o0, &mut o1];
    let h = u.process(Io { frames: n, input: &ins, sense: &[], output: &mut outs });
    (o0, h)
}

/// xorshift64*, deterministic.
pub struct Rng(pub u64);
impl Rng {
    pub fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    pub fn unit(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32 * 2.0 - 1.0
    }
    /// A sample from the adversarial mix.
    pub fn sample(&mut self, mode: u64) -> f32 {
        match (mode + self.next() % 7) % 12 {
            0 => f32::NAN,
            1 => f32::INFINITY,
            2 => f32::NEG_INFINITY,
            3 => 1e30 * self.unit(),
            4 => 1e-40 * self.unit(),
            5 => f32::MAX * self.unit(),
            6 => 4.0 * self.unit(),
            7 => f32::from_bits(self.next() as u32),
            _ => self.unit(),
        }
    }
}

/// Run `unit` over `blocks` host calls of random sizes and check, sample by
/// sample against the input `latency` frames earlier, that the output is
/// finite, never above full scale, never above the input (the most any unit
/// may pass is unity gain on in-range input), and silent for non-finite
/// input.
pub fn never_above_input(unit: &mut dyn ProtectionUnit, channels: usize, latency: usize, seed: u64, blocks: usize) {
    never_above_input_sensed(unit, channels, 0, latency, seed, blocks)
}

/// As [`never_above_input`], with `sense` sense ports fed the same
/// adversarial mix (a closed loop must never relax the gain above unity).
pub fn never_above_input_sensed(
    unit: &mut dyn ProtectionUnit,
    channels: usize,
    sense: usize,
    latency: usize,
    seed: u64,
    blocks: usize,
) {
    let mut rng = Rng(seed);
    let mut hist: Vec<Vec<f32>> = vec![vec![0.0; latency]; channels];
    for b in 0..blocks {
        let n = 1 + (rng.next() % 1100) as usize;
        let mode = rng.next() % 4;
        let calm = b % 5 != 0; // mostly in-range audio, with storms between
        let input: Vec<Vec<f32>> =
            (0..channels).map(|_| (0..n).map(|_| if calm { rng.unit() } else { rng.sample(mode) }).collect()).collect();
        let sense_buf: Vec<Vec<f32>> = (0..sense)
            .map(|_| (0..n).map(|_| if calm { 0.3 * rng.unit() } else { rng.sample(mode) }).collect())
            .collect();
        let mut output = vec![vec![0.0f32; n]; channels];
        let ins: Vec<&[f32]> = input.iter().map(|v| &v[..]).collect();
        let sns: Vec<&[f32]> = sense_buf.iter().map(|v| &v[..]).collect();
        let mut outs: Vec<&mut [f32]> = output.iter_mut().map(|v| &mut v[..]).collect();
        unit.process(Io { frames: n, input: &ins, sense: &sns, output: &mut outs });
        for c in 0..channels {
            hist[c].extend_from_slice(&input[c]);
            for k in 0..n {
                let x = hist[c][k];
                let y = output[c][k];
                assert!(y.is_finite(), "{:?} block {b} ch {c} frame {k}: {y} from {x}", unit.kind());
                if !x.is_finite() {
                    assert_eq!(y, 0.0, "{:?} block {b}: non-finite input passed", unit.kind());
                } else {
                    let bound = x.abs().min(1.0) * (1.0 + 1e-6);
                    assert!(y.abs() <= bound, "{:?} block {b} ch {c} frame {k}: {y} from {x}", unit.kind());
                }
            }
            hist[c].drain(..n);
        }
        let t = tel(unit);
        for c in 0..channels {
            let g = t.channels[c].gain;
            assert!(g.is_finite() && (0.0..=1.0).contains(&g), "{:?} gain {g}", unit.kind());
        }
    }
}

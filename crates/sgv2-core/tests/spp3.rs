//! spp3 against the behaviour specification's equations, on synthetic
//! parameters only (no Mac's tuning appears here).

use sgv2_core::spp3::{self, ChannelParams, Params};
use sgv2_core::{Health, Io, ProtectionUnit, Telemetry, FAILSAFE_GAIN};

const RATE: u32 = 48_000;
const BLOCK: u32 = 512;

/// Synthetic two-group, four-channel parameter set with short time
/// constants so the tests settle quickly.
fn params() -> Params {
    let mut p = Params {
        channels: 4,
        sample_rate: RATE,
        block_frames: BLOCK,
        thermal_ambient_temp: 40.0,
        thermal_gain_attack_time_s: 0.5,
        thermal_gain_release_time_s: 2.0,
        thermal_lookahead_delay_ms: 1.5,
        thermal_protection_enabled: 1,
        thermal_safe_tlim: 0.0,
        thermal_safe_tlim_offset: 0.0,
        thermal_safe_tlim_time_min: 0.0,
        ..Params::default()
    };
    for c in 0..4 {
        p.ch[c] = ChannelParams {
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
        };
    }
    p
}

fn elec(p: &Params, c: usize) -> f64 {
    let ch = &p.ch[c];
    let (ag, reb, rs) = (ch.ag as f64, ch.reb_ref as f64, ch.rshunt as f64);
    ag * ag * reb / ((reb + rs) * (reb + rs))
}

/// Run `seconds` of constant-level drive `x[c]` through the unit in host
/// slices of `slice` frames; return the final telemetry and the last output
/// sample per channel.
fn drive(unit: &mut spp3::Unit, x: [f32; 4], seconds: f64, slice: usize) -> (Telemetry, [f32; 4]) {
    let total = (seconds * RATE as f64) as usize;
    let input: Vec<Vec<f32>> = (0..4).map(|c| vec![x[c]; slice]).collect();
    let mut output = vec![vec![0.0f32; slice]; 4];
    let mut done = 0;
    while done < total {
        let n = slice.min(total - done);
        let ins: Vec<&[f32]> = input.iter().map(|b| &b[..n]).collect();
        let mut outs: Vec<&mut [f32]> = output.iter_mut().map(|b| &mut b[..n]).collect();
        unit.process(Io { frames: n, input: &ins, sense: &[], output: &mut outs });
        done += n;
    }
    let mut t = Telemetry::new();
    unit.telemetry(&mut t);
    let last = [0, 1, 2, 3].map(|c| output[c][(total - 1) % slice]);
    (t, last)
}

#[test]
fn power_topology_and_first_block() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    assert!(!u.is_failsafe());
    assert_eq!(u.latency_frames(), 72); // 1.5 ms at 48 kHz
    let (t, _) = drive(&mut u, [0.1; 4], BLOCK as f64 / RATE as f64, 512);
    // first block: look-ahead fill, so only BLOCK-72 frames of drive
    let want = elec(&p, 0) * 0.01 * (BLOCK - 72) as f64 / BLOCK as f64;
    assert!(
        (t.channels[0].modeled_power_w as f64 - want).abs() < 1e-6 * want,
        "{} {want}",
        t.channels[0].modeled_power_w
    );
    let (t, _) = drive(&mut u, [0.1; 4], BLOCK as f64 / RATE as f64, 512);
    let want = elec(&p, 0) * 0.01;
    assert!((t.channels[0].modeled_power_w as f64 - want).abs() < 1e-6 * want);
}

#[test]
fn backward_euler_matches_the_recurrence() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let x = 0.05f64;
    let dt = BLOCK as f64 / RATE as f64;
    let c = &p.ch[1];
    let (a_vc, a_mg) = (dt / (c.tau_tvc as f64 + dt), dt / (c.tau_tmg as f64 + dt));
    let (mut vc, mut mg) = (0.0f64, 0.0f64);
    let look = 72.0;
    for b in 0..400 {
        let fill = if b == 0 { (BLOCK as f64 - look) / BLOCK as f64 } else { 1.0 };
        let pw = elec(&p, 1) * x * x * fill;
        let old = mg;
        vc += a_vc * (old + pw * c.t_sett_vc as f64 - vc);
        mg += a_mg * (pw * c.t_sett_mg as f64 - mg);
    }
    let (t, _) = drive(&mut u, [x as f32; 4], 400.0 * dt, 333);
    let got = t.channels[1].ol_temp_c as f64;
    assert!((got - (40.0 + vc)).abs() < 1e-3, "{got} vs {}", 40.0 + vc);
    assert!((t.channels[1].magnet_temp_c as f64 - (40.0 + mg)).abs() < 1e-3);
    assert_eq!(t.channels[1].gain, 1.0, "well under the limit: no attenuation");
}

#[test]
fn steady_state_temperature() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let (t, _) = drive(&mut u, [0.08; 4], 60.0, 1024);
    for c in 0..4 {
        let pw = elec(&p, c) * 0.0064;
        let want = 40.0 + pw * (p.ch[c].t_sett_vc as f64 + p.ch[c].t_sett_mg as f64);
        let got = t.channels[c].ol_temp_c as f64;
        assert!((got - want).abs() < 0.01, "ch{c} {got} {want}");
    }
}

#[test]
fn limiting_converges_to_the_power_ceiling() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    // Drive ch0 hard; P_hold = (limit - ambient) / (Rvc + Rmg).
    let x = 0.5f32;
    let (t, last) = drive(&mut u, [x, 0.0, 0.0, 0.0], 120.0, 1024);
    let c = &p.ch[0];
    let p_hold = (c.temperature_limit as f64 - 40.0) / (c.t_sett_vc as f64 + c.t_sett_mg as f64);
    let p_drive = elec(&p, 0) * (x as f64) * (x as f64);
    let want = (p_hold / p_drive).sqrt();
    let g = t.channels[0].gain as f64;
    assert!((g - want).abs() < 2e-3, "gain {g} want {want}");
    assert!((last[0] as f64 - x as f64 * g).abs() < 1e-6);
    let temp = t.channels[0].ol_temp_c as f64;
    assert!((temp - 120.0).abs() < 0.2, "{temp}");
    // ch1 is in ch0's group: same gain although undriven
    assert_eq!(t.channels[1].gain, t.channels[0].gain);
    // the other group is untouched
    assert_eq!(t.channels[2].gain, 1.0);
}

#[test]
fn never_passes_the_hard_limit_on_a_step() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let total_blocks = (30.0 * RATE as f64 / BLOCK as f64) as usize;
    let ins_buf = vec![vec![1.0f32; 512]; 4];
    let mut outs_buf = vec![vec![0.0f32; 512]; 4];
    let mut t = Telemetry::new();
    let mut peak: f32 = 0.0;
    let mut hard_seen = false;
    for _ in 0..total_blocks {
        let ins: Vec<&[f32]> = ins_buf.iter().map(|b| &b[..]).collect();
        let mut outs: Vec<&mut [f32]> = outs_buf.iter_mut().map(|b| &mut b[..]).collect();
        u.process(Io { frames: 512, input: &ins, sense: &[], output: &mut outs });
        u.telemetry(&mut t);
        peak = peak.max(t.channels[0].ol_temp_c);
        if t.channels[0].ol_temp_c > 128.0 {
            hard_seen = true;
            assert!(t.channels[0].gain <= FAILSAFE_GAIN + 1e-6);
        }
    }
    // with these synthetic constants a full-scale step overshoots into the
    // hard path, which then pulls the coil back
    assert!(hard_seen);
    assert!(t.channels[0].ol_temp_c < 128.5, "{}", t.channels[0].ol_temp_c);
    assert!(peak < 135.0, "{peak}");
}

#[test]
fn host_slicing_does_not_change_anything() {
    let p = params();
    let n = 48_000 * 3;
    let sig: Vec<Vec<f32>> = (0..4)
        .map(|c| {
            let mut s = 17u32 + c;
            (0..n)
                .map(|k| {
                    s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                    let env = if (k / 9000) % 2 == 0 { 0.9 } else { 0.05 };
                    ((s >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 2.0 * env
                })
                .collect()
        })
        .collect();
    let render = |slice: usize| {
        let mut u = spp3::Unit::new(p);
        let mut out = vec![vec![0.0f32; n]; 4];
        let mut t = Telemetry::new();
        let mut done = 0;
        while done < n {
            let m = slice.min(n - done);
            let ins: Vec<&[f32]> = sig.iter().map(|b| &b[done..done + m]).collect();
            let mut outs: Vec<&mut [f32]> = out.iter_mut().map(|b| &mut b[done..done + m]).collect();
            u.process(Io { frames: m, input: &ins, sense: &[], output: &mut outs });
            done += m;
        }
        u.telemetry(&mut t);
        let state: Vec<(f32, f32, f32)> =
            (0..4).map(|c| (t.channels[c].ol_temp_c, t.channels[c].magnet_temp_c, t.channels[c].gain)).collect();
        (out, (t.frame, state))
    };
    let (ref_out, ref_t) = render(512);
    // the limiter must actually have engaged for this to mean anything
    assert!(ref_t.1[0].2 < 0.999, "{:?}", ref_t);
    for slice in [1usize, 7, 31, 96, 127, 160, 1024, 4096] {
        let (o, t) = render(slice);
        assert!(o == ref_out, "audio differs at slice {slice}");
        assert_eq!(t, ref_t, "state differs at slice {slice}");
    }
}

#[test]
fn lookahead_delays_an_impulse() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let mut x = vec![0.0f32; 512];
    x[10] = 0.25;
    let ins: Vec<&[f32]> = (0..4).map(|_| &x[..]).collect();
    let mut ob = vec![vec![0.0f32; 512]; 4];
    let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
    u.process(Io { frames: 512, input: &ins, sense: &[], output: &mut outs });
    for o in &ob {
        let k = o.iter().position(|&v| v != 0.0).unwrap();
        assert_eq!(k, 10 + 72);
        assert_eq!(o[k], 0.25);
    }
}

#[test]
fn failsafe_cases_are_exactly_twenty_db() {
    let x = [0.5f32; 64];
    let mut o = [[0.0f32; 64]; 4];
    // channel-count mismatch
    let mut u = spp3::Unit::new(params());
    let ins: [&[f32]; 3] = [&x, &x, &x];
    {
        let [a, b, c, _] = &mut o;
        let mut outs: [&mut [f32]; 3] = [a, b, c];
        let h = u.process(Io { frames: 64, input: &ins, sense: &[], output: &mut outs });
        assert!(h.is_failsafe() && h.0 & Health::CHANNEL_MISMATCH != 0);
    }
    assert!(o[0].iter().all(|&v| (v - 0.05).abs() < 1e-7));
    // invalid essential parameters
    let mut bad = params();
    bad.ch[2].tau_tvc = f32::NAN;
    assert!(spp3::Unit::init_error(&bad).is_some());
    // a controller the implementation does not have
    let mut cl = params();
    cl.enable_cpms_control = 1;
    assert_eq!(spp3::Unit::init_error(&cl), Some(spp3::InitError::Unsupported("EnableCPMSControl")));
    for p in [bad, cl, Params::default()] {
        let mut u = spp3::Unit::new(p);
        assert!(u.is_failsafe());
        let ins: [&[f32]; 4] = [&x; 4];
        let [a, b, c, d] = &mut o;
        let mut outs: [&mut [f32]; 4] = [a, b, c, d];
        let h = u.process(Io { frames: 64, input: &ins, sense: &[], output: &mut outs });
        assert!(h.is_failsafe() && h.0 & Health::NO_PARAMS != 0);
        assert!(o[3].iter().all(|&v| (v - 0.05).abs() < 1e-7));
    }
}

#[test]
fn non_finite_input_never_reaches_the_output_or_the_model() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let mut x = vec![0.1f32; 512];
    x[5] = f32::NAN;
    x[6] = f32::INFINITY;
    let ins: Vec<&[f32]> = (0..4).map(|_| &x[..]).collect();
    let mut ob = vec![vec![0.0f32; 512]; 4];
    let mut any_fs = false;
    for _ in 0..3 {
        let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
        let h = u.process(Io { frames: 512, input: &ins, sense: &[], output: &mut outs });
        any_fs |= h.is_failsafe();
        assert!(ob.iter().flatten().all(|v| v.is_finite()));
    }
    assert!(any_fs);
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    assert!(t.channels[0].ol_temp_c.is_finite() && t.channels[0].gain.is_finite());
}

#[test]
fn blob_roundtrip() {
    use sgv2_core::blob::{build, Pod};
    let p = params();
    let b = build(sgv2_core::UnitKind::Spp3, 4, 8, RATE, p.as_bytes());
    assert_eq!(Params::from_blob(&b).unwrap(), p);
    let mut bad = b.clone();
    let n = bad.len();
    bad[n - 3] ^= 0x40;
    assert!(Params::from_blob(&bad).is_err());
}

#[test]
fn measured_power_and_sense_geometry() {
    let p = params();
    let mut u = spp3::Unit::new(p);
    let x = vec![0.1f32; 512];
    let i = vec![0.05f32; 512];
    let v = vec![0.2f32; 512];
    let ins: Vec<&[f32]> = (0..4).map(|_| &x[..]).collect();
    let sense: Vec<&[f32]> = (0..8).map(|k| if k % 2 == 0 { &i[..] } else { &v[..] }).collect();
    let mut ob = vec![vec![0.0f32; 512]; 4];
    let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
    let h = u.process(Io { frames: 512, input: &ins, sense: &sense, output: &mut outs });
    assert_eq!(h, Health::OK);
    let mut t = Telemetry::new();
    u.telemetry(&mut t);
    let want = 0.05 * 2.0 * 0.2 * 12.0;
    assert!((t.channels[0].measured_power_w - want).abs() < 1e-6);
    // too few sense ports: open loop, never fail-safe
    let mut outs: Vec<&mut [f32]> = ob.iter_mut().map(|b| &mut b[..]).collect();
    let h = u.process(Io { frames: 512, input: &ins, sense: &sense[..3], output: &mut outs });
    assert_eq!(h.0, Health::OPEN_LOOP);
}

#[test]
fn zero_speedup_factor_holds_the_feed_forward_reduction_at_unity() {
    let mut p = params();
    for c in 0..4 {
        p.ch[c].thermal_ff_speedup_factor = 0.0;
    }
    let mut u = spp3::Unit::new(p);
    // a drive whose steady state sits between the limit and the hard limit
    let (t, _) = drive(&mut u, [0.355, 0.0, 0.0, 0.0], 40.0, 1024);
    assert!(t.channels[0].ol_temp_c > 120.0, "{}", t.channels[0].ol_temp_c);
    assert_eq!(t.channels[0].axis_gain[4], 1.0);
    assert_eq!(t.channels[0].gain, 1.0);
}

#[test]
fn disabled_thermal_protection_is_refused() {
    let mut p = params();
    p.thermal_protection_enabled = 0;
    assert!(spp3::Unit::new(p).is_failsafe());
}

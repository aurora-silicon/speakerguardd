// SPDX-License-Identifier: MIT
//! Proof prototype for the playback contract. Test-only until the kernel
//! descriptor adapter, device transfer bounds and ledger ABI are qualified.
//! Production retains the existing full-scale onset reserve on every kernel.

use crate::engine::Thermal;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    InvalidModel,
    InvalidLedger,
    Exhausted,
}

/// One active model, referred to the last authenticated completed boundary.
/// No unobserved or still queued energy has been integrated into `state`.
pub struct Constraint<'a> {
    pub state: &'a Thermal,
    pub ambient: f64,
    pub ceiling: f64,
    /// Worst power of an indefinite hardware-safe fallback, including gains.
    pub safe_power: f64,
}

/// Future energy which cannot be treated as observed or cooled. All terms
/// must be joules at the worst amplifier gain and smallest valid load.
#[derive(Debug, Clone, Copy)]
pub struct Debt {
    pub queued: f64,
    pub unspent: f64,
    pub downstream_tail: f64,
}

fn nonnegative(x: f64) -> bool {
    x.is_finite() && x >= 0.0
}

impl Debt {
    fn total(self) -> Result<f64, Refusal> {
        if ![self.queued, self.unspent, self.downstream_tail]
            .into_iter()
            .all(nonnegative)
        {
            return Err(Refusal::InvalidLedger);
        }
        let total = self.queued + self.unspent + self.downstream_tail;
        if total.is_finite() {
            Ok(total)
        } else {
            Err(Refusal::InvalidLedger)
        }
    }
}

impl Constraint<'_> {
    /// A positive-system envelope bounds *any* distribution of E joules,
    /// including an impulse, arbitrary queue delay and the infinite safe tail.
    /// This is sufficient, not an optimizer or a loudness guarantee.
    fn available(&self) -> Result<f64, Refusal> {
        let th = self.state;
        if ![th.tr_coil, th.tr_magnet, th.tau_coil, th.tau_magnet]
            .into_iter()
            .all(|x| x.is_finite() && x > 0.0)
            || ![th.t_coil, th.t_magnet, self.ambient, self.ceiling]
                .into_iter()
                .all(f64::is_finite)
            || !nonnegative(self.safe_power)
        {
            return Err(Refusal::InvalidModel);
        }
        let b = th.tr_magnet / th.tau_magnet;
        let k = (th.tr_coil / th.tau_coil).max(b);
        let magnet = th
            .t_magnet
            .max(self.ambient + self.safe_power * th.tr_magnet);
        let coil = th.t_coil.max(magnet + self.safe_power * th.tr_coil);
        // A deliberate numerical margin; not a raised thermal limit.
        let margin = 1e-8 * self.ceiling.abs().max(1.0);
        let available =
            ((self.ceiling - margin - coil) / k).min((self.ceiling - margin - magnet) / b);
        if ![b, k, coil, magnet, available]
            .into_iter()
            .all(f64::is_finite)
        {
            return Err(Refusal::InvalidModel);
        }
        if available <= 0.0 {
            Err(Refusal::Exhausted)
        } else {
            Ok(available)
        }
    }
}

/// Additional credit only. Outstanding old credit is not reissued, and
/// outstanding queue energy is never silently dropped on a restart/epoch.
pub fn additional_joules(models: &[Constraint<'_>], debt: Debt) -> Result<f64, Refusal> {
    if models.is_empty() {
        return Err(Refusal::InvalidModel);
    }
    let debt = debt.total()?;
    let mut room = f64::INFINITY;
    for model in models {
        room = room.min(model.available()?);
    }
    let additional = room - debt;
    if !additional.is_finite() || additional <= 0.0 {
        Err(Refusal::Exhausted)
    } else {
        Ok(additional)
    }
}

/// Convert joules to the kernel prototype's Q30-squared-frame units,
/// rounding DOWN. Rate, voltage/load bound and counters belong to one epoch.
/// Lifetime counters travel as integers; never convert the total to f64.
pub fn cumulative_ceiling(
    submitted: u64,
    joules: f64,
    watts_full_scale: f64,
    rate: u32,
) -> Result<u64, Refusal> {
    if !nonnegative(joules)
        || !watts_full_scale.is_finite()
        || watts_full_scale <= 0.0
        || rate == 0
        || rate > 192000
    {
        return Err(Refusal::InvalidLedger);
    }
    // Keep conversions far from f64's ambiguous u64 endpoint and below its
    // exact integer range. One grant may be at most 2^52 normalized units.
    let units = joules / watts_full_scale * f64::from(rate) * (1u64 << 30) as f64;
    if !units.is_finite() || !(0.0..=(1u64 << 52) as f64).contains(&units) {
        return Err(Refusal::InvalidLedger);
    }
    // Remove a relative error allowance before floor, so floating-point
    // evaluation can never round a tiny positive credit upward to one unit.
    let units = (units - units.abs() * 1e-12 - 1e-9).max(0.0).floor() as u64;
    submitted.checked_add(units).ok_or(Refusal::InvalidLedger)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn zero_debt() -> Debt {
        Debt {
            queued: 0.0,
            unspent: 0.0,
            downstream_tail: 0.0,
        }
    }
    fn constraint(th: &Thermal) -> Constraint<'_> {
        Constraint {
            state: th,
            ambient: 50.0,
            ceiling: 130.0,
            safe_power: 0.2,
        }
    }

    // Independent exact oracle, including the repeated-root extremum. The
    // production hold() deliberately returns the initial drive as a loose
    // equal-tau peak bound; that is not the actual temperature of a burst.
    fn exact_interval(th: &mut Thermal, ambient: f64, power: f64, h: f64) -> (f64, f64) {
        let (tc, tm) = (th.tau_coil, th.tau_magnet);
        let mi = ambient + power * th.tr_magnet;
        let ci = mi + power * th.tr_coil;
        let ec = (-h / tc).exp();
        let em = (-h / tm).exp();
        let (end, inside) = if tc == tm {
            let a = th.t_coil - ci;
            let b = (th.t_magnet - mi) / tc;
            let t = tc - a / b;
            let inside = if t > 0.0 && t < h {
                ci + (a + b * t) * (-t / tc).exp()
            } else {
                f64::NEG_INFINITY
            };
            (ci + (a + b * h) * ec, inside)
        } else {
            let slow = (th.t_magnet - mi) * tm / (tm - tc);
            let fast = th.t_coil - ci - slow;
            let ratio = -fast * tm / (slow * tc);
            let t = ratio.ln() / (1.0 / tc - 1.0 / tm);
            let inside = if t > 0.0 && t < h {
                ci + fast * (-t / tc).exp() + slow * (-t / tm).exp()
            } else {
                f64::NEG_INFINITY
            };
            (ci + fast * ec + slow * em, inside)
        };
        let mend = mi + (th.t_magnet - mi) * em;
        let peak = th.t_coil.max(end).max(inside);
        let mpeak = th.t_magnet.max(mend);
        th.t_coil = end;
        th.t_magnet = mend;
        (peak, mpeak)
    }

    #[test]
    fn no_model_or_invalid_transfer_refuses_credit() {
        assert_eq!(
            additional_joules(&[], zero_debt()),
            Err(Refusal::InvalidModel)
        );
        for value in [f64::NAN, f64::INFINITY, 0.0, -1.0] {
            let th = Thermal::new(value, 3.9, 129.5, 70.0, 50.0);
            assert_eq!(
                additional_joules(&[constraint(&th)], zero_debt()),
                Err(Refusal::InvalidModel)
            );
        }
        assert_eq!(
            cumulative_ceiling(0, 1.0, 0.0, 48000),
            Err(Refusal::InvalidLedger)
        );
        assert_eq!(
            cumulative_ceiling(0, 1.0, 20.0, 0),
            Err(Refusal::InvalidLedger)
        );
    }

    #[test]
    fn queued_unspent_and_filter_tail_all_reduce_credit() {
        let th = Thermal::new(104.9, 3.9, 129.5, 70.0, 50.0);
        let free = additional_joules(&[constraint(&th)], zero_debt()).unwrap();
        let debt = Debt {
            queued: 0.1,
            unspent: 0.2,
            downstream_tail: 0.3,
        };
        let room = additional_joules(&[constraint(&th)], debt).unwrap();
        assert!((room - (free - 0.6)).abs() < 1e-12);
        assert_eq!(
            additional_joules(
                &[constraint(&th)],
                Debt {
                    queued: free,
                    ..zero_debt()
                }
            ),
            Err(Refusal::Exhausted)
        );
        assert_eq!(
            additional_joules(
                &[constraint(&th)],
                Debt {
                    queued: f64::NAN,
                    ..zero_debt()
                }
            ),
            Err(Refusal::InvalidLedger)
        );
    }

    #[test]
    fn hot_magnet_blocks_even_when_coil_is_cold() {
        let mut th = Thermal::new(104.9, 3.9, 129.5, 70.0, 50.0);
        th.t_magnet = 129.0;
        assert_eq!(
            additional_joules(&[constraint(&th)], zero_debt()),
            Err(Refusal::Exhausted)
        );
    }

    #[test]
    fn feedback_can_only_tighten() {
        let ff = Thermal::new(26.91, 1.55, 23.93, 73.26, 50.0);
        let mut vi = ff.clone();
        vi.t_coil = 125.0;
        let alone = additional_joules(&[constraint(&ff)], zero_debt()).unwrap();
        let combined = additional_joules(&[constraint(&ff), constraint(&vi)], zero_debt()).unwrap();
        assert!(combined < alone);
        vi.t_coil = 130.0;
        assert_eq!(
            additional_joules(&[constraint(&ff), constraint(&vi)], zero_debt()),
            Err(Refusal::Exhausted)
        );
    }

    #[test]
    fn conversion_rounds_down_and_never_wraps_lifetime_counter() {
        let rate = 48000;
        let watts = 130.0;
        for n in 0..10000 {
            let joules = f64::from(n) / 12345.0;
            let total = cumulative_ceiling(u64::MAX - (1u64 << 50), joules, watts, rate).unwrap();
            let credit = total - (u64::MAX - (1u64 << 50));
            assert!((credit as f64 / (1u64 << 30) as f64 / f64::from(rate)) * watts <= joules);
        }
        assert_eq!(
            cumulative_ceiling(u64::MAX, 1.0, watts, rate),
            Err(Refusal::InvalidLedger)
        );
        assert_eq!(
            cumulative_ceiling(0, f64::INFINITY, watts, rate),
            Err(Refusal::InvalidLedger)
        );
    }

    #[test]
    fn arbitrary_bursts_and_infinite_safe_tail_stay_below_both_limits() {
        // Includes the retained J413 onset constants, J316 tweeter constants,
        // and equal/reversed time constants (interior and delayed extrema).
        for (rc, tc, rm, tm) in [
            (104.9, 3.9, 129.5, 70.0),
            (26.91, 1.55, 23.93, 73.26),
            (38.0, 3.3, 26.0, 100.0),
            (20.0, 3.0, 30.0, 3.0),
            (20.0, 30.0, 30.0, 3.0),
        ] {
            for seed in 1u64..41 {
                let mut rng = seed;
                let mut rand = || {
                    rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                    (rng >> 32) as f64 / u32::MAX as f64
                };
                let mut truth = Thermal::new(rc, tc, rm, tm, 50.0 + 10.0 * rand());
                truth.t_magnet = 50.0 + 10.0 * rand();
                let grant = additional_joules(&[constraint(&truth)], zero_debt()).unwrap();
                let mut remaining = grant;
                // Include a long queue stall before the first energy arrives.
                let (c, m) = exact_interval(&mut truth, 50.0, 0.2, rand() * 300.0);
                assert!(c < 130.0 && m < 130.0);
                for _ in 0..40 {
                    let energy = remaining * rand();
                    remaining -= energy;
                    let dt = 1e-4 + rand();
                    let (c, m) = exact_interval(&mut truth, 50.0, 0.2 + energy / dt, dt);
                    assert!(c < 130.0 && m < 130.0, "seed={seed}, model=({rc},{tc},{rm},{tm}), grant={grant}, remain={remaining}, coil={c}, magnet={m}, end=({}, {})", truth.t_coil, truth.t_magnet);
                }
                // Concentrate the rest into a short onset, then check the
                // entire long tail using exact extrema, not endpoint samples.
                let (c, m) = exact_interval(&mut truth, 50.0, 0.2 + remaining / 1e-6, 1e-6);
                assert!(c < 130.0 && m < 130.0);
                let (c, m) = exact_interval(&mut truth, 50.0, 0.2, 100000.0);
                assert!(c < 130.0 && m < 130.0);
            }
        }
    }

    #[test]
    fn every_packaged_profile_reserves_each_active_model_and_tail() {
        let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("conf/apple");
        let mut profiles = 0;
        let mut models = 0;
        let mut refused = 0;
        let mut unsafe_tails = 0;
        for entry in std::fs::read_dir(directory).unwrap() {
            let path = entry.unwrap().path();
            if path.extension().is_none_or(|e| e != "conf") {
                continue;
            }
            let cfg = crate::config::Config::load(&path).unwrap();
            profiles += 1;
            let ambient = cfg.globals.t_ambient;
            for speaker in &cfg.speakers {
                let full = crate::engine::v_worst(speaker, &cfg.globals).powi(2) / speaker.z_min();
                let safe = full * 10f64.powf(-cfg.globals.safe_reduction / 10.0);
                let ceiling = speaker.t_limit - speaker.t_headroom;
                let mut active = vec![Thermal::new(
                    speaker.tr_coil,
                    speaker.tau_coil,
                    speaker.tr_magnet,
                    speaker.tau_magnet,
                    ambient,
                )];
                if speaker.is_chan.is_some() {
                    active.push(Thermal::new(
                        speaker.vi_tr_coil,
                        speaker.vi_tau_coil,
                        speaker.vi_tr_magnet,
                        speaker.vi_tau_magnet,
                        ambient,
                    ));
                }
                let constraints: Vec<_> = active
                    .iter()
                    .map(|state| Constraint {
                        state,
                        ambient,
                        ceiling,
                        safe_power: safe,
                    })
                    .collect();
                let unsafe_tail = active
                    .iter()
                    .any(|th| ambient + safe * (th.tr_coil + th.tr_magnet) >= speaker.t_limit);
                let credit = match additional_joules(&constraints, zero_debt()) {
                    Ok(credit) => credit,
                    Err(Refusal::Exhausted) => {
                        // Some draft profiles' fallback equilibrium is above
                        // their working or even hard ceiling. Refuse credit;
                        // do not raise that ceiling to make them admissible.
                        assert!(!["j316", "j413", "j616", "j700"]
                            .contains(&path.file_stem().unwrap().to_str().unwrap()));
                        refused += 1;
                        0.0
                    }
                    Err(error) => panic!("{} {}: {error:?}", path.display(), speaker.name),
                };
                let cap = cumulative_ceiling(0, credit, full, 48000).unwrap();
                let actual = cap as f64 / (1u64 << 30) as f64 / 48000.0 * full;
                assert!(actual <= credit);
                if unsafe_tail {
                    // This contract must never be activated. Preserve the
                    // counterexample instead of claiming its fallback safe.
                    assert_eq!(credit, 0.0);
                    unsafe_tails += 1;
                    let mut crossing = false;
                    for mut truth in active {
                        models += 1;
                        let (c, m) = exact_interval(&mut truth, ambient, safe, 100000.0);
                        crossing |= c >= speaker.t_limit || m >= speaker.t_limit;
                        eprintln!(
                            "GRANT_UNSAFE_TAIL {} {} power={safe} coil={c} magnet={m} hard={}",
                            path.file_stem().unwrap().to_str().unwrap(),
                            speaker.name,
                            speaker.t_limit
                        );
                    }
                    assert!(crossing);
                    continue;
                }
                for mut truth in active {
                    models += 1;
                    let bound = if credit > 0.0 {
                        ceiling
                    } else {
                        speaker.t_limit
                    };
                    // A full grant can arrive after an arbitrarily long
                    // scheduling gap, not just at the calculation boundary.
                    let (c, m) = exact_interval(&mut truth, ambient, safe, 600.0);
                    assert!(c < bound && m < bound, "{}", path.display());
                    let (c, m) =
                        exact_interval(&mut truth, ambient, safe + actual / 0.00001, 0.00001);
                    assert!(
                        c < bound && m < bound,
                        "{} {} onset {c} {m}",
                        path.display(),
                        speaker.name
                    );
                    let (c, m) = exact_interval(&mut truth, ambient, safe, 100000.0);
                    assert!(
                        c < bound && m < bound,
                        "{} {} tail {c} {m}",
                        path.display(),
                        speaker.name
                    );
                }
            }
        }
        assert_eq!(profiles, 35);
        eprintln!("GRANT_ADMISSION profiles={profiles} models={models} refused-speakers={refused} unsafe-tails={unsafe_tails}");
    }
}

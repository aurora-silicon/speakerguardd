// SPDX-License-Identifier: MIT
//! Production grant model. Heat is charged once, from retired immutable
//! payloads. Queued energy stays a reservation even if it may already have
//! played: a delayed report can never forgive that debt or count it as cooled.

use crate::config::Config;
use crate::engine::{Engine, Thermal, ASAHI_MARGIN};
use crate::model::SpeakerState;
use crate::playback::{self, Snapshot, UNITY};
use crate::playback_grant::{additional_joules, cumulative_ceiling, Constraint, Debt};
use std::collections::VecDeque;
use std::io;
use std::time::{Duration, Instant};

fn bad() -> io::Error {
    io::Error::from_raw_os_error(libc::EPROTO)
}

#[derive(Clone)]
pub struct Projection {
    pub ff: Thermal,
    pub vi: Option<Thermal>,
    pub ff_peak: (f64, f64),
    pub vi_peak: (f64, f64),
    pub means: (f64, f64),
}

struct Driver {
    ff: Thermal,
    vi: Option<Thermal>,
    full: f64,
    safe: f64,
}

pub struct PlaybackModel {
    cfg: Config,
    pub last: Snapshot,
    drivers: Vec<Driver>,
    // Latest-possible placement of each bounded energy increment.
    source: VecDeque<(u64, Vec<f64>)>,
    delivered: VecDeque<(u64, Vec<f64>)>,
    initial: Vec<(f64, f64)>,
    origin_ns: u64,
    pub monitor: Option<Engine>,
    monitor_to: Instant,
    pub ready: bool,
}

impl PlaybackModel {
    pub fn new(
        cfg: &Config,
        states: &[SpeakerState],
        engine: Option<&Engine>,
        at: Instant,
        status: Snapshot,
    ) -> io::Result<Self> {
        if !status.qualified(cfg) || states.len() != cfg.speakers.len() {
            return Err(bad());
        }
        let status = status.for_profile(cfg)?;
        let drivers = cfg
            .speakers
            .iter()
            .enumerate()
            .map(|(k, s)| {
                let mut ff = Thermal::new(
                    s.tr_coil,
                    s.tau_coil,
                    s.tr_magnet,
                    s.tau_magnet,
                    states[k].t_coil,
                );
                ff.t_magnet = states[k].t_magnet;
                if let Some(u) = engine.and_then(|e| e.unified.as_ref()) {
                    ff.t_coil = ff.t_coil.max(u[k].ff.t_coil);
                    ff.t_magnet = ff.t_magnet.max(u[k].ff.t_magnet);
                }
                let vi = s.is_chan.map(|_| {
                    let mut vi = Thermal::new(
                        s.vi_tr_coil,
                        s.vi_tau_coil,
                        s.vi_tr_magnet,
                        s.vi_tau_magnet,
                        s.t_limit,
                    );
                    if let Some(a) = engine.and_then(|e| e.asahi.as_ref()) {
                        vi = a[k].th.clone();
                    }
                    if let Some(u) = engine
                        .and_then(|e| e.unified.as_ref())
                        .and_then(|u| u[k].vi.as_ref())
                    {
                        vi.t_coil = vi.t_coil.max(u.t_coil);
                        vi.t_magnet = vi.t_magnet.max(u.t_magnet);
                    }
                    vi
                });
                let voltage = crate::engine::v_worst(s, &cfg.globals).max(s.vs_scale);
                let margin = if s.is_chan.is_some() {
                    ASAHI_MARGIN
                } else {
                    1.0
                };
                let full = voltage * voltage / s.z_min() * margin;
                let denominator = if status.transfer == 2 {
                    20000.0
                } else {
                    10000.0
                };
                let safe =
                    full * 10f64.powf(-f64::from(status.safe_reduction_millidb) / denominator);
                Driver { ff, vi, full, safe }
            })
            .collect();
        let initial = states
            .iter()
            .map(|s| (s.mean_power(1.0), s.mean_power(60.0)))
            .collect();
        let origin_ns = status.clock_ns;
        Ok(Self {
            cfg: cfg.clone(),
            last: status,
            drivers,
            source: VecDeque::new(),
            delivered: VecDeque::new(),
            initial,
            origin_ns,
            monitor: engine.cloned(),
            monitor_to: at,
            ready: false,
        })
    }

    fn window(
        history: &VecDeque<(u64, Vec<f64>)>,
        now: u64,
        k: usize,
        seconds: f64,
        full: f64,
    ) -> f64 {
        history
            .iter()
            .map(|(end, e)| {
                let remaining = seconds - now.saturating_sub(*end) as f64 * 1e-9;
                if remaining > 0.0 {
                    e[k].min(full * remaining)
                } else {
                    0.0
                }
            })
            .sum()
    }

    fn initial_energy(&self, now: u64, k: usize, seconds: f64) -> f64 {
        // The same late-placement envelope used by generation-3 restore.
        let (history, _) = crate::model::budget_envelope(
            self.initial[k].0,
            self.initial[k].1,
            self.drivers[k].full,
        );
        let elapsed = now.saturating_sub(self.origin_ns) as f64 * 1e-9;
        let mut age = elapsed;
        let mut energy = 0.0;
        for (dt, p) in history.iter().rev() {
            let overlap = (seconds - age).clamp(0.0, *dt);
            energy += (p * dt).min(self.drivers[k].full * overlap);
            age += dt;
        }
        energy
    }

    pub fn update(&mut self, status: Snapshot) -> io::Result<Vec<Projection>> {
        let status = status.for_profile(&self.cfg)?;
        if status.flags & playback::FAILED != 0
            || status.card != self.last.card
            || status.amp_gain_millidbv != self.last.amp_gain_millidbv
            || status.safe_reduction_millidb != self.last.safe_reduction_millidb
            || status.epoch < self.last.epoch
            || (status.epoch == self.last.epoch && status.sequence < self.last.sequence)
            || status.speakers != self.drivers.len()
            || status.clock_ns < self.last.clock_ns
        {
            return Err(bad());
        }
        let dt = (status.clock_ns - self.last.clock_ns) as f64 * 1e-9;
        let unobserved_source = (status.flags & playback::PLAYING != 0
            && !status.qualified(&self.cfg))
            || (self.last.flags & playback::PLAYING != 0 && !self.last.qualified(&self.cfg));
        let mut source = Vec::new();
        let mut delivered = Vec::new();
        let mut rows = Vec::new();
        for k in 0..self.drivers.len() {
            let retired = status.completed[k]
                .checked_sub(self.last.completed[k])
                .ok_or_else(bad)?;
            let refunded = status.refunded[k]
                .checked_sub(self.last.refunded[k])
                .ok_or_else(bad)?;
            let energy = retired.checked_sub(refunded).ok_or_else(bad)?;
            let raw = status.source_completed[k]
                .checked_sub(self.last.source_completed[k])
                .ok_or_else(bad)?;
            if status.submitted[k] < self.last.submitted[k]
                || status.source_submitted[k] < self.last.source_submitted[k]
            {
                return Err(bad());
            }
            let driver = &mut self.drivers[k];
            let e = playback::joules(energy, driver.full);
            let pending = playback::joules(status.submitted[k] - status.completed[k], driver.full);
            let fp = driver.ff.hold(driver.safe, dt, self.cfg.globals.t_ambient);
            let impulse = driver.ff.impulse(e);
            let mut ff = driver.ff.clone();
            let future = ff.impulse(pending);
            let mut ff_peak = (
                fp.0.max(impulse.0).max(future.0),
                fp.1.max(impulse.1).max(future.1),
            );
            let mut vi_peak = ff_peak;
            let mut vi = driver.vi.as_mut().map(|vi| {
                let vp = vi.hold(driver.safe, dt, self.cfg.globals.t_ambient);
                let impulse = vi.impulse(e);
                let mut projected = vi.clone();
                let future = projected.impulse(pending);
                vi_peak = (
                    vp.0.max(impulse.0).max(future.0),
                    vp.1.max(impulse.1).max(future.1),
                );
                projected
            });
            // A separate measured-power observer can only tighten the model.
            if let Some(m) = &self.monitor {
                if let Some(u) = &m.unified {
                    ff.t_coil = ff.t_coil.max(u[k].ff.t_coil);
                    ff.t_magnet = ff.t_magnet.max(u[k].ff.t_magnet);
                    ff_peak.0 = ff_peak.0.max(u[k].peak);
                }
                if let (Some(v), Some(a)) = (&mut vi, &m.asahi) {
                    v.t_coil = v.t_coil.max(a[k].th.t_coil);
                    v.t_magnet = v.t_magnet.max(a[k].th.t_magnet);
                    vi_peak.0 = vi_peak.0.max(a[k].coil_peak);
                    vi_peak.1 = vi_peak.1.max(a[k].magnet_peak);
                }
            }
            // Unsupported live routes retain the old full-scale source
            // budget assumption, even while hardware caps bound their heat.
            source.push(
                playback::joules(raw, driver.full)
                    + if unobserved_source {
                        driver.full * dt
                    } else {
                        0.0
                    },
            );
            delivered.push(e);
            rows.push(Projection {
                ff,
                vi,
                ff_peak,
                vi_peak,
                means: (0.0, 0.0),
            });
        }
        self.source.push_back((status.clock_ns, source));
        self.delivered.push_back((status.clock_ns, delivered));
        for history in [&mut self.source, &mut self.delivered] {
            while history
                .front()
                .is_some_and(|(t, _)| status.clock_ns.saturating_sub(*t) >= 60_000_000_000)
            {
                history.pop_front();
            }
        }
        for (k, row) in rows.iter_mut().enumerate() {
            let d = &self.drivers[k];
            let pending = playback::joules(
                status.source_submitted[k] - status.source_completed[k],
                d.full,
            );
            let mean = |seconds| {
                ((Self::window(&self.source, status.clock_ns, k, seconds, d.full)
                    + self.initial_energy(status.clock_ns, k, seconds)
                    + pending)
                    / seconds)
                    .min(d.full)
            };
            row.means = (mean(1.0), mean(60.0));
        }
        self.last = status;
        self.ready = self.packet()?.is_some();
        Ok(rows)
    }

    pub fn packet(&self) -> io::Result<Option<[u8; playback::BYTES]>> {
        if !self.last.qualified(&self.cfg) {
            return Ok(None);
        }
        let mut ceiling = Vec::new();
        for (k, d) in self.drivers.iter().enumerate() {
            let s = &self.cfg.speakers[k];
            let mut ff = d.ff.clone();
            let mut vi = d.vi.clone();
            if let Some(m) = &self.monitor {
                if let Some(u) = &m.unified {
                    ff.t_coil = ff.t_coil.max(u[k].ff.t_coil);
                    ff.t_magnet = ff.t_magnet.max(u[k].ff.t_magnet);
                }
                if let (Some(v), Some(a)) = (&mut vi, &m.asahi) {
                    v.t_coil = v.t_coil.max(a[k].th.t_coil);
                    v.t_magnet = v.t_magnet.max(a[k].th.t_magnet);
                }
            }
            let mut constraints = vec![Constraint {
                state: &ff,
                ambient: self.cfg.globals.t_ambient,
                ceiling: s.t_limit - s.t_headroom,
                safe_power: d.safe,
            }];
            if let Some(v) = &vi {
                constraints.push(Constraint {
                    state: v,
                    ambient: self.cfg.globals.t_ambient,
                    ceiling: s.t_limit - s.t_headroom,
                    safe_power: d.safe,
                });
            }
            let pending = playback::joules(self.last.submitted[k] - self.last.completed[k], d.full);
            let Ok(mut room) = additional_joules(
                &constraints,
                Debt {
                    queued: pending,
                    unspent: 0.0,
                    downstream_tail: 0.0,
                },
            ) else {
                return Ok(None);
            };
            for (seconds, limit) in [(1.0, s.p_limit_1s), (60.0, s.p_limit_60s)] {
                if limit > 0.0 {
                    let past =
                        Self::window(&self.delivered, self.last.clock_ns, k, seconds, d.full);
                    room = room.min((limit - d.safe) * seconds - past - pending);
                }
            }
            if !(room.is_finite() && room > 0.0) {
                return Ok(None);
            }
            let cap = cumulative_ceiling(self.last.submitted[k], room, d.full, 48000)
                .map_err(|_| bad())?;
            ceiling.push(cap);
        }
        self.last
            .grant(&ceiling, &vec![UNITY; ceiling.len()])
            .map(Some)
    }

    pub fn observe(
        &mut self,
        words: &[i32],
        frames: usize,
        end: Instant,
        rate: f64,
    ) -> io::Result<()> {
        let Some(monitor) = &mut self.monitor else {
            return Ok(());
        };
        if end <= self.monitor_to || frames == 0 || rate <= 0.0 {
            return Ok(());
        }
        let start = end
            .checked_sub(Duration::from_secs_f64(frames as f64 / rate))
            .unwrap_or(end);
        if start > self.monitor_to {
            monitor.idle((start - self.monitor_to).as_secs_f64());
        }
        monitor.check(words, rate).map_err(|_| bad())?;
        let count = ((end - self.monitor_to).as_secs_f64() * rate)
            .ceil()
            .min(frames as f64) as usize;
        monitor.measured(
            words,
            frames,
            frames,
            count.max(1),
            0,
            1.0 / rate,
            1.0,
            &vec![vec![(0.0, 0.0)]; self.drivers.len()],
        );
        monitor.clear_hard_if_cool();
        self.monitor_to = end;
        Ok(())
    }

    pub fn closed_monitor(&mut self, to: Instant) {
        if self.last.flags & playback::PLAYING == 0
            && self.last.submitted == self.last.completed
            && to > self.monitor_to
        {
            if let Some(monitor) = &mut self.monitor {
                monitor.idle((to - self.monitor_to).as_secs_f64());
                monitor.clear_hard_if_cool();
            }
            self.monitor_to = to;
        }
    }

    pub fn close_to_for_save(&mut self, ns: u64) -> io::Result<Vec<Projection>> {
        if self.last.flags & playback::LIVE != 0 || self.last.submitted != self.last.completed {
            return Err(bad());
        }
        if ns < self.last.clock_ns {
            return Err(bad());
        }
        let mut snapshot = self.last.clone();
        snapshot.clock_ns = ns;
        let dt = (snapshot.clock_ns - self.last.clock_ns) as f64 * 1e-9;
        // The stop forbids further grants. Unknown source before the safe
        // hardware level is still charged to the saved programme budgets.
        self.source.push_back((
            snapshot.clock_ns,
            self.drivers.iter().map(|d| d.full * dt).collect(),
        ));
        self.update(snapshot)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gate_sim::KernelGate;
    #[test]
    fn unsupported_live_routes_keep_conservative_source_budgets() {
        let cfg = Config::parse(include_str!("../conf/apple/j413.conf")).unwrap();
        let mut gate = KernelGate::new(&cfg, 48000, 64);
        gate.start(1).unwrap();
        gate.owner(true, 1).unwrap();
        let states = cfg
            .speakers
            .iter()
            .cloned()
            .map(|s| SpeakerState::with_globals(s, &cfg.globals))
            .collect::<Vec<_>>();
        let engine = Engine::new(&cfg, true);
        let mut model = PlaybackModel::new(
            &cfg,
            &states,
            engine.as_ref(),
            Instant::now(),
            gate.status(1).unwrap(),
        )
        .unwrap();
        let mut unsupported = gate.status(1_000_000_001).unwrap();
        unsupported.flags &= !playback::CONFIGURED;
        let rows = model.update(unsupported).unwrap();
        assert!(model.packet().unwrap().is_none());
        assert!(rows.iter().all(|r| r.means.0 > 1.0));
        let mut bad = gate.status(1_000_000_002).unwrap();
        bad.safe_reduction_millidb -= 1;
        assert!(model.update(bad).is_err());
    }
}

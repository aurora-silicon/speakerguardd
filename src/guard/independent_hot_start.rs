// SPDX-License-Identifier: MIT
//! Main #17 independent thermal oracle, retained across the RC reconciliation.
use super::*;
use super::fake::*;
struct Tracked {
    hw: FakeHw,
    gains: Vec<(Duration, Vec<f64>)>,
}
impl Tracked {
    fn snap(&mut self) {
        let db: Vec<_> = (0..self.hw.channels)
            .map(|_| self.hw.effective_reduction())
            .collect();
        if self.gains.last().is_none_or(|(_, last)| *last != db) {
            self.gains.push((self.hw.t, db));
        }
    }
}
impl Hw for Tracked {
    fn now(&self) -> Instant {
        self.hw.now()
    }
    fn boottime(&self) -> Duration {
        self.hw.boottime()
    }
    fn events_available(&self) -> bool {
        self.hw.events_available()
    }
    fn sleep(&mut self, d: Duration) {
        self.snap();
        self.hw.sleep(d);
        self.snap();
    }
    fn take_lock(&mut self) -> io::Result<()> {
        self.snap();
        let out = self.hw.take_lock();
        self.snap();
        out
    }
    fn ping(&mut self) -> io::Result<()> {
        self.snap();
        let out = self.hw.ping();
        self.snap();
        out
    }
    fn set_reduction(&mut self, d: f64) -> io::Result<f64> {
        self.snap();
        let out = self.hw.set_reduction(d);
        self.snap();
        out
    }
    fn set_min(&mut self) -> io::Result<()> {
        self.snap();
        let out = self.hw.set_min();
        self.snap();
        out
    }
    fn relinquish(&mut self) -> io::Result<()> {
        self.snap();
        let out = self.hw.relinquish();
        self.snap();
        out
    }
    fn sample_rate(&mut self) -> io::Result<i64> {
        self.snap();
        let out = self.hw.sample_rate();
        self.snap();
        out
    }
    fn open_sense(&mut self, r: u32) -> io::Result<()> {
        self.snap();
        let out = self.hw.open_sense(r);
        self.snap();
        out
    }
    fn close_sense(&mut self) {
        self.snap();
        self.hw.close_sense();
        self.snap();
    }
    fn read_period(&mut self, w: &mut [i32]) -> io::Result<Option<bool>> {
        self.snap();
        let out = self.hw.read_period(w);
        self.snap();
        out
    }
    fn queued(&mut self) -> io::Result<u64> {
        self.snap();
        let out = self.hw.queued();
        self.snap();
        out
    }
    fn read_tail(&mut self, w: &mut [i32]) -> io::Result<usize> {
        self.snap();
        let out = self.hw.read_tail(w);
        self.snap();
        out
    }
    fn wait_rate_change(&mut self, d: Duration) -> bool {
        self.snap();
        let out = self.hw.wait_rate_change(d);
        self.snap();
        out
    }
    fn rate_events(&mut self) -> bool {
        self.snap();
        let out = self.hw.rate_events();
        self.snap();
        out
    }
}


#[test]
fn full_scale_after_independent_hot_nodes() {
    check_full_scale_after_independent_hot_nodes(true);
}

#[test]
fn full_scale_after_hot_start_without_control_events() {
    check_full_scale_after_independent_hot_nodes(false);
}

fn check_full_scale_after_independent_hot_nodes(events: bool) {
crate::log::set_verbosity(crate::log::Level::Error);
let cfg=Config::parse(include_str!("../../conf/apple/j700.conf")).unwrap();
let mut hw=FakeHw::new(cfg.globals.channels,cfg.globals.period);hw.events=events;hw.level=Box::new(|_|1.0);
let mut hw=Tracked{hw,gains:Vec::new()};
let mut g=Guard::start(&mut hw,&cfg,20.0,None).unwrap();
while hw.hw.t<Duration::from_secs(1200){g.tick(&mut hw).unwrap();}hw.snap();
hw.gains.push((hw.hw.t,vec![hw.hw.effective_reduction();cfg.speakers.len()]));
for lapse in &hw.hw.lapses {assert!(hw.gains.iter().any(|(at,_)|at==lapse)||hw.gains.iter().rev().find(|(at,_)|at<=lapse).is_some_and(|(_,ds)|ds.iter().all(|d|*d>=cfg.globals.safe_reduction)));}
for (k,s) in cfg.speakers.iter().enumerate() {
    let (mut c,mut m,mut peak)=(s.t_limit,s.t_limit,s.t_limit);
    let (mut at,mut db)=(Duration::ZERO,MUTE_DB);
    for (to,ds) in &hw.gains {
        let h=(*to-at).as_secs_f64();
        if h>0.0 {
            let p=s.vs_scale.powi(2)/s.z_nominal*10f64.powf(-db/10.0);
            let mi=cfg.globals.t_ambient+p*s.tr_magnet;let ci=mi+p*s.tr_coil;
            let b=(m-mi)*s.tau_magnet/(s.tau_magnet-s.tau_coil);let a=c-ci-b;
            let cc=|t:f64|ci+a*(-t/s.tau_coil).exp()+b*(-t/s.tau_magnet).exp();
            peak=peak.max(c).max(cc(h));
            let ratio=-a*s.tau_magnet/(b*s.tau_coil);
            if ratio>0.0 {let t=ratio.ln()/(1.0/s.tau_coil-1.0/s.tau_magnet);if t>0.0&&t<h {peak=peak.max(cc(t));}}
            c=cc(h);m=mi+(m-mi)*(-h/s.tau_magnet).exp();
        }
        at=*to;db=ds[k];
    }
    eprintln!("NEO_HOT {} peak={peak:.9} limit={}",s.name,s.t_limit);
    assert!(peak<=s.t_limit+1e-6);
}
eprintln!("NEO_TRACE events={events} pings={} first={:?} lapses={}",hw.hw.pings.len(),hw.hw.pings.first(),hw.hw.lapses.len());
assert!(events || hw.hw.pings.is_empty(), "unlocked without control events");
}


//! Rig-only: derive a test variant of a compiled spp3 blob, the way the
//! macOS rig derives test sidecars / property values.
//! `blob-patch IN OUT key=value ...`; keys: cal=R,T (all channels),
//! safe_time_min=X, start=N (64007), rampout=X (all channels), method=N.
use sgv2_core::blob::{build, Header, Pod};
use sgv2_core::spp3::Params;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let bytes = std::fs::read(&a[1]).unwrap();
    let (h, _) = Header::parse(&bytes).unwrap();
    let mut p = Params::from_blob(&bytes).unwrap_or_else(|e| panic!("{e:?}"));
    let n = p.channels as usize;
    for kv in &a[3..] {
        let (k, v) = kv.split_once('=').unwrap_or((kv.as_str(), ""));
        match k {
            "cal" => {
                let (r, t) = v.split_once(',').unwrap();
                for c in &mut p.ch[..n] {
                    c.cal_present = 1;
                    c.cal_r = r.parse().unwrap();
                    c.cal_t = t.parse().unwrap();
                }
            }
            // calch=C:R,T sets one channel; nocal clears every channel
            "calch" => {
                let (c, rt) = v.split_once(':').unwrap();
                let (r, t) = rt.split_once(',').unwrap();
                let c = &mut p.ch[c.parse::<usize>().unwrap()];
                c.cal_present = 1;
                c.cal_r = r.parse().unwrap();
                c.cal_t = t.parse().unwrap();
            }
            "nocal" => {
                for c in &mut p.ch[..n] {
                    c.cal_present = 0;
                }
            }
            "safe_time_min" => p.thermal_safe_tlim_time_min = v.parse().unwrap(),
            "rampout" => {
                for c in &mut p.ch[..n] {
                    c.rampout = v.parse().unwrap();
                }
            }
            "start" => p.thermal_state_override = v.parse().unwrap(),
            _ => panic!("unknown key {k}"),
        }
    }
    let out = build(sgv2_core::UnitKind::Spp3, p.channels, h.sense_channels, p.sample_rate, p.as_bytes());
    Params::from_blob(&out).unwrap();
    std::fs::write(&a[2], out).unwrap();
}

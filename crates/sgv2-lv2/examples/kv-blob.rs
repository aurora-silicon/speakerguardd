//! Compile a parameter blob from a plain `key = value` file through the same
//! `spp3::TuningLookup` path the tuning reader uses. A development aid until
//! the reader's compiler lands; the input file is the user's own data and
//! never part of this repository.
//!
//! File format (one per line, `#` comments):
//! ```text
//! channels = 6
//! sample_rate = 48000
//! block_frames = 512
//! Thermal_AmbientTemp = 40          # global keys
//! ch*.T_sett_vc = 20                # every channel
//! ch4.T_sett_vc = 22                # one channel (overrides ch*)
//! ch0.source = 0                    # host port, default = channel index
//! ch0.sense = 1 0                   # (current, voltage) ports, default V,I pairs
//! ```
//! Usage: `kv-blob <in.kv> <out.bin>`.

use std::collections::HashMap;

use sgv2_core::blob::{build, Pod};
use sgv2_core::spp3::{Params, TuningLookup, Unit};
use sgv2_core::UnitKind;

struct Kv(HashMap<String, String>);

impl Kv {
    fn num(&self, k: &str) -> Option<f64> {
        self.0.get(k).and_then(|v| v.split_whitespace().next()?.parse().ok())
    }
}

impl TuningLookup for Kv {
    fn channels(&self) -> usize {
        self.num("channels").unwrap_or(0.0) as usize
    }
    fn sample_rate(&self) -> Option<u32> {
        self.num("sample_rate").map(|v| v as u32)
    }
    fn block_frames(&self) -> Option<u32> {
        self.num("block_frames").map(|v| v as u32)
    }
    fn global(&self, key: &str) -> Option<f64> {
        self.num(key)
    }
    fn channel(&self, ch: usize, key: &str) -> Option<f64> {
        self.num(&format!("ch{ch}.{key}")).or_else(|| self.num(&format!("ch*.{key}")))
    }
    fn source(&self, ch: usize) -> Option<u32> {
        Some(self.num(&format!("ch{ch}.source")).map_or(ch as u32, |v| v as u32))
    }
    fn sense(&self, ch: usize) -> Option<(u32, u32)> {
        match self.0.get(&format!("ch{ch}.sense")) {
            Some(v) => {
                let mut it = v.split_whitespace().map(|x| x.parse::<u32>().ok());
                Some((it.next()??, it.next()??))
            }
            // the telemetry bus is [V0, I0, V1, I1, ...]
            None => Some((2 * ch as u32 + 1, 2 * ch as u32)),
        }
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (inp, out) = (a.get(1).expect("usage: kv-blob <in.kv> <out.bin>"), a.get(2).expect("out"));
    let mut m = HashMap::new();
    for line in std::fs::read_to_string(inp).unwrap().lines() {
        let line = line.split('#').next().unwrap().trim();
        if let Some((k, v)) = line.split_once('=') {
            m.insert(k.trim().to_string(), v.trim().to_string());
        }
    }
    let kv = Kv(m);
    let p = Params::from_lookup(&kv).unwrap_or_else(|e| panic!("{e:?}"));
    if let Some(e) = Unit::init_error(&p) {
        eprintln!("warning: the unit refuses these parameters ({e:?}); it would run the fail-safe");
    }
    let sense = kv.num("sense_channels").unwrap_or(2.0 * p.channels as f64) as u32;
    std::fs::write(out, build(UnitKind::Spp3, p.channels, sense, p.sample_rate, p.as_bytes())).unwrap();
    println!("wrote {out}");
}

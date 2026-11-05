//! Watch an lsm2 engine page the supervisor created (never creates or
//! resets it): every 50 ms drain the telemetry ring, check it is gap-free,
//! and write every record as one CSV line with lsm2's state: the applied
//! gain, the five axis candidates, the band gains, the monitor's coil and
//! magnet temperatures, the velocity limit and mask level, and the power
//! meters. A one-line summary a second goes to stdout.
//! Usage: `lsm2-watch <page> <out.csv> [seconds]`.

use std::io::Write;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use sgv2_core::ring::Reader;
use sgv2_lv2::page::{EnginePage, LAYOUT, MAGIC};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (Some(path), Some(out)) = (a.get(1), a.get(2)) else {
        eprintln!("usage: lsm2-watch <page> <out.csv> [seconds]");
        std::process::exit(2);
    };
    let secs: u64 = a.get(3).and_then(|v| v.parse().ok()).unwrap_or(30);
    let page = EnginePage::open(path).expect("open the engine page");
    let mut csv = std::io::BufWriter::new(std::fs::File::create(out).expect("csv"));
    let _ = write!(csv, "mono_ns,frame,health,amp_in_w,amp_model_w,power_gain");
    for c in 0..2 {
        let _ = write!(
            csv,
            ",g{c},peak{c},disp{c},vel{c},pow{c},temp{c},b0_{c},b1_{c},b2_{c},b3_{c},b4_{c},tvc{c},tmg{c},vlim{c},mask{c},pmod{c}"
        );
    }
    let _ = writeln!(csv);
    let mut rd = Reader::new();
    while rd.poll(page.ring()).is_some() {}
    let lost0 = rd.lost;
    let start = Instant::now();
    let mut last_print = Instant::now();
    let (mut prev, mut gaps, mut records) = (None::<u64>, 0u64, 0u64);
    let mut step = None::<u64>;
    let mut health_or = 0u32;
    let (mut tmax, mut gmin) = ([f32::MIN; 2], [1.0f32; 2]);
    while start.elapsed() < Duration::from_secs(secs) {
        std::thread::sleep(Duration::from_millis(50));
        let h = page.header();
        if h.magic.load(Ordering::Acquire) != MAGIC || h.layout.load(Ordering::Relaxed) != LAYOUT {
            println!("page not started or another layout");
            continue;
        }
        let mono = h.mono_ns.load(Ordering::Relaxed);
        while let Some(t) = rd.poll(page.ring()) {
            records += 1;
            if let Some(p) = prev {
                let d = t.frame.wrapping_sub(p);
                match step {
                    None => step = Some(d),
                    Some(s) if d != s => {
                        gaps += 1;
                        step = Some(d);
                    }
                    _ => {}
                }
            }
            prev = Some(t.frame);
            health_or |= t.health.0;
            let g = &t.global;
            let _ = write!(
                csv,
                "{mono},{},{:#x},{},{},{}",
                t.frame, t.health.0, g.amp_input_power_w, g.modeled_amp_power_w, g.power_gain
            );
            for (k, c) in t.channels[..2].iter().enumerate() {
                tmax[k] = tmax[k].max(c.coil_temp_c);
                gmin[k] = gmin[k].min(c.gain);
                let x = c.axis_gain;
                let b = c.band_gain;
                let _ = write!(
                    csv,
                    ",{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                    c.gain,
                    x[0],
                    x[1],
                    x[2],
                    x[3],
                    x[4],
                    b[0],
                    b[1],
                    b[2],
                    b[3],
                    b[4],
                    c.coil_temp_c,
                    c.magnet_temp_c,
                    c.velocity_limit_m_s,
                    c.mask_db,
                    c.modeled_power_w
                );
            }
            let _ = writeln!(csv);
        }
        if last_print.elapsed() >= Duration::from_secs(1) {
            last_print = Instant::now();
            let mut t = sgv2_core::Telemetry::new();
            if let Ok(v) = page.ring().read(page.ring().head().saturating_sub(1)) {
                t = v;
            }
            let ch = &t.channels[..2];
            println!(
                "t={:>3}s step={:?} health={:#x} health_or={:#x} records={} lost={} gaps={} gain_db={:?} \
                 temp_c={:?} max_temp_c={:?} min_gain_db={:?} vlim={:?}",
                start.elapsed().as_secs(),
                step,
                h.health.load(Ordering::Relaxed),
                health_or,
                records,
                rd.lost - lost0,
                gaps,
                ch.iter().map(|c| (20.0 * c.gain.log10() * 100.0).round() / 100.0).collect::<Vec<_>>(),
                ch.iter().map(|c| (c.coil_temp_c * 100.0).round() / 100.0).collect::<Vec<_>>(),
                tmax.map(|v| (v * 100.0).round() / 100.0),
                gmin.map(|v| (20.0 * v.log10() * 100.0).round() / 100.0),
                ch.iter().map(|c| (c.velocity_limit_m_s * 1000.0).round() / 1000.0).collect::<Vec<_>>(),
            );
            let _ = csv.flush();
        }
    }
    let _ = csv.flush();
}

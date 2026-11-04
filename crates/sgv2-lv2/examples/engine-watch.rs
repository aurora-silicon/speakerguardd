//! Watch the engine page the supervisor created (never creates or resets
//! it): every 50 ms drain the telemetry ring and check that it is gap-free,
//! write every record as one CSV line, and print a one-line summary a
//! second. Usage: `engine-watch <page> <out.csv> [seconds]`.

use std::io::Write;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use sgv2_core::ring::Reader;
use sgv2_lv2::page::{EnginePage, LAYOUT, MAGIC};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (Some(path), Some(out)) = (a.get(1), a.get(2)) else {
        eprintln!("usage: engine-watch <page> <out.csv> [seconds]");
        std::process::exit(2);
    };
    let secs: u64 = a.get(3).and_then(|v| v.parse().ok()).unwrap_or(30);
    let page = EnginePage::open(path).expect("open the engine page");
    let mut csv = std::io::BufWriter::new(std::fs::File::create(out).expect("csv"));
    let _ = write!(csv, "mono_ns,frame,health,sense_delay");
    for c in 0..6 {
        let _ = write!(csv, ",g{c},sel{c},pmod{c},tol{c},tmg{c},tcl{c},tadj{c},cl{c},vms{c},ims{c},r{c}");
    }
    let _ = writeln!(csv);
    let mut rd = Reader::new();
    // start at the ring's head: older records are history
    while rd.poll(page.ring()).is_some() {}
    let lost0 = rd.lost;
    let start = Instant::now();
    let (mut last_print, mut last_seq, mut last_frames) = (Instant::now(), 0u64, 0u64);
    let (mut prev, mut gaps, mut records, mut stale_polls) = (None::<u64>, 0u64, 0u64, 0u64);
    let mut health_or = 0u32;
    let mut seq_prev = page.header().seq.load(Ordering::Acquire);
    while start.elapsed() < Duration::from_secs(secs) {
        std::thread::sleep(Duration::from_millis(50));
        let h = page.header();
        if h.magic.load(Ordering::Acquire) != MAGIC || h.layout.load(Ordering::Relaxed) != LAYOUT {
            println!("page not started or another layout");
            continue;
        }
        let seq = h.seq.load(Ordering::Acquire);
        if seq == seq_prev {
            stale_polls += 1;
        }
        seq_prev = seq;
        let mono = h.mono_ns.load(Ordering::Relaxed);
        let delay = h.sense_delay.load(Ordering::Relaxed);
        while let Some(t) = rd.poll(page.ring()) {
            records += 1;
            if let Some(p) = prev {
                if t.frame != p + 512 {
                    gaps += 1;
                }
            }
            prev = Some(t.frame);
            health_or |= t.health.0;
            let _ = write!(csv, "{mono},{},{:#x},{delay}", t.frame, t.health.0);
            for c in &t.channels[..6] {
                let _ = write!(
                    csv,
                    ",{},{},{},{},{},{},{},{},{:e},{:e},{}",
                    c.gain,
                    c.axis_gain[4],
                    c.modeled_power_w,
                    c.ol_temp_c,
                    c.magnet_temp_c,
                    c.cl_temp_c,
                    c.tlim_adj_c,
                    c.closed_loop as u8,
                    c.sense_v_ms,
                    c.sense_i_ms,
                    c.sense_r_ohm
                );
            }
            let _ = writeln!(csv);
        }
        if last_print.elapsed() >= Duration::from_secs(1) {
            last_print = Instant::now();
            let frames = h.frames.load(Ordering::Relaxed);
            let mut t = sgv2_core::Telemetry::new();
            if let Ok(v) = page.ring().read(page.ring().head().saturating_sub(1)) {
                t = v;
            }
            let ch = &t.channels[..6];
            println!(
                "t={:>3}s pid={} crc={:08x} calls/s={} frames/s={} health={:#x} delay={} records={} lost={} gaps={} \
                 stale_polls={} health_or={:#x} gain_db={:?} tol={:?} tcl={:?} r={:?}",
                start.elapsed().as_secs(),
                h.pid.load(Ordering::Relaxed),
                h.params_crc.load(Ordering::Relaxed),
                seq - last_seq,
                frames - last_frames,
                h.health.load(Ordering::Relaxed),
                delay,
                records,
                rd.lost - lost0,
                gaps,
                stale_polls,
                health_or,
                ch.iter().map(|c| (20.0 * c.gain.log10() * 100.0).round() / 100.0).collect::<Vec<_>>(),
                ch.iter().map(|c| (c.ol_temp_c * 1000.0).round() / 1000.0).collect::<Vec<_>>(),
                ch.iter().map(|c| (c.cl_temp_c * 100.0).round() / 100.0).collect::<Vec<_>>(),
                ch.iter().map(|c| (c.sense_r_ohm * 1000.0).round() / 1000.0).collect::<Vec<_>>(),
            );
            last_seq = seq;
            last_frames = frames;
            let _ = csv.flush();
        }
    }
    let _ = csv.flush();
}

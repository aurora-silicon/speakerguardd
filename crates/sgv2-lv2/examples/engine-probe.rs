//! Create an engine page and print what the plugin writes into it, once a
//! second: heartbeat, health, sense delay, and per-channel telemetry.
//! Usage: `engine-probe <path> [seconds]`.

use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use sgv2_core::ring::Reader;
use sgv2_lv2::page::EnginePage;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = a.get(1).expect("usage: engine-probe <path> [seconds]");
    let secs: u64 = a.get(2).and_then(|v| v.parse().ok()).unwrap_or(30);
    let page = EnginePage::create(path).unwrap();
    let mut rd = Reader::new();
    let start = Instant::now();
    let mut last_seq = 0;
    let mut last_frames = 0;
    let mut gaps = 0u64;
    let mut prev_frame: Option<u64> = None;
    let mut records = 0u64;
    let mut open_loop = 0u64;
    while start.elapsed() < Duration::from_secs(secs) {
        std::thread::sleep(Duration::from_millis(1000));
        while let Some(t) = rd.poll(page.ring()) {
            records += 1;
            if let Some(p) = prev_frame {
                if t.frame != p + 512 {
                    gaps += 1;
                }
            }
            prev_frame = Some(t.frame);
            if t.health.0 & sgv2_core::Health::OPEN_LOOP != 0 {
                open_loop += 1;
            }
        }
        let h = page.header();
        let seq = h.seq.load(Ordering::Acquire);
        let frames = h.frames.load(Ordering::Relaxed);
        let mut t = sgv2_core::Telemetry::new();
        if let Ok(v) = page.ring().read(page.ring().head().saturating_sub(1)) {
            t = v;
        }
        println!(
            "t={:>3}s pid={} crc={:08x} latency={} calls/s={} frames/s={} health={:#x} sense_delay={} records={} lost={} gaps={} open_loop_blocks={} p_meas={:?} gain={:?}",
            start.elapsed().as_secs(),
            h.pid.load(Ordering::Relaxed),
            h.params_crc.load(Ordering::Relaxed),
            h.latency.load(Ordering::Relaxed),
            seq - last_seq,
            frames - last_frames,
            h.health.load(Ordering::Relaxed),
            h.sense_delay.load(Ordering::Relaxed),
            records,
            rd.lost,
            gaps,
            open_loop,
            t.channels.iter().take(6).map(|c| c.measured_power_w).collect::<Vec<_>>(),
            t.channels.iter().take(6).map(|c| c.gain).collect::<Vec<_>>(),
        );
        last_seq = seq;
        last_frames = frames;
    }
}

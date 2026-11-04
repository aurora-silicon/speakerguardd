//! Adversarial checks on the LV2 shim (lane T): the plugin's own handling of
//! host ports and its engine page. See `sgv2-core/tests/adversarial.rs` for
//! the core properties.

use std::sync::atomic::Ordering;
use std::sync::Mutex;

use sgv2_core::Telemetry;
use sgv2_lv2::page::EnginePage;
use sgv2_lv2::{lv2_descriptor, VARIANTS};

/// The C ABI reads its paths from the environment: tests take turns.
static ENV: Mutex<()> = Mutex::new(());

/// FINDING 13 (lane B): when any program input or output port is null,
/// `run` returns before touching anything, so every connected output keeps
/// whatever the buffer held: in a real graph that is the previous quantum,
/// repeated at full level, and no protection, telemetry or heartbeat runs
/// for the call. A missing port should mute every connected output.
#[test]
fn a_null_program_port_mutes_the_connected_outputs() {
    let _env = ENV.lock().unwrap_or_else(|e| e.into_inner());
    std::env::set_var("SGV2_PARAMS", "/nonexistent/sgv2-adversarial");
    std::env::set_var("SGV2_ENGINE_PAGE", "/nonexistent/sgv2-adversarial");
    let (_, channels, sense, _) = VARIANTS[0];
    assert_eq!((channels, sense), (2, 0), "variant 0 is the 2-channel, no-sense one");
    let d = lv2_descriptor(0);
    for missing in 0..2 * channels {
        let mut bufs = [[0.9f32; 64]; 4];
        let mut ctl = [0.0f32; 3];
        unsafe {
            let h = ((*d).instantiate)(d, 48_000.0, core::ptr::null(), core::ptr::null());
            assert!(!h.is_null());
            for (p, b) in bufs.iter_mut().enumerate() {
                let ptr = if p == missing { core::ptr::null_mut() } else { b.as_mut_ptr() };
                ((*d).connect_port)(h, p as u32, ptr.cast());
            }
            for k in 0..3 {
                ((*d).connect_port)(h, 4 + k as u32, ctl.as_mut_ptr().add(k).cast());
            }
            ((*d).activate.unwrap())(h);
            ((*d).run)(h, 64);
            ((*d).cleanup)(h);
        }
        for (out, buf) in bufs.iter().enumerate().skip(channels) {
            if out == missing {
                continue;
            }
            let worst = buf.iter().fold(0.0f32, |m, v| m.max(v.abs()));
            assert!(worst <= 0.9 * 0.1, "port {missing} null: output port {out} left at {worst} (stale buffer)");
        }
    }
}

/// FINDING 14 (lane B): the engine page's `min_gain_bits` drops non-finite
/// gains and starts from 1.0, so a channel whose published gain is NaN (a
/// model the unit no longer trusts) reads as unity, "no attenuation", to
/// the supervisor. Absent channels are NaN by design, so the page needs the
/// channel count it was started with to tell the two apart.
#[test]
fn the_engine_page_never_reports_a_nan_gain_as_unity() {
    let path = std::env::temp_dir().join(format!("sgv2-adversarial-page-{}", std::process::id()));
    let path = path.to_str().unwrap().to_string();
    let page = EnginePage::create(&path).unwrap();
    page.start(0, 0, 2, 0);
    let mut t = Telemetry::new();
    t.channels[0].gain = f32::NAN;
    t.channels[1].gain = f32::NAN;
    page.push(&t);
    let g = f32::from_bits(page.header().min_gain_bits.load(Ordering::Relaxed));
    let _ = std::fs::remove_file(&path);
    assert!(g != 1.0, "both active channels NaN, the page reports min gain {g}");
}

/// The page's minimum gain is the most restrictive active channel.
#[test]
fn the_engine_page_reports_the_most_restrictive_gain() {
    let path = std::env::temp_dir().join(format!("sgv2-adversarial-page2-{}", std::process::id()));
    let path = path.to_str().unwrap().to_string();
    let page = EnginePage::create(&path).unwrap();
    page.start(0, 0, 2, 0);
    let mut t = Telemetry::new();
    t.channels[0].gain = 0.7;
    t.channels[1].gain = 0.2;
    page.push(&t);
    let g = f32::from_bits(page.header().min_gain_bits.load(Ordering::Relaxed));
    let _ = std::fs::remove_file(&path);
    assert_eq!(g, 0.2);
}

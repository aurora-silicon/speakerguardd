// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT
//! The kernel gain cap on the host side: the plugin models the drive level
//! the supervisor asks for on the engine page, and acknowledges what it
//! models. All values are synthetic.

use sgv2_core::blob::{self, Pod};
use sgv2_core::{spp3, FailSafe, Health, UnitKind};
use sgv2_lv2::page::{drive_scale, EnginePage};
use sgv2_lv2::{load_unit, AnyUnit, Role};

fn spp3_blob() -> Vec<u8> {
    let mut p = spp3::Params {
        channels: 2,
        sample_rate: 48_000,
        block_frames: 512,
        thermal_ambient_temp: 40.0,
        thermal_gain_attack_time_s: 1.0,
        thermal_gain_release_time_s: 5.0,
        thermal_lookahead_delay_ms: 2.0,
        thermal_protection_enabled: 1,
        ..spp3::Params::default()
    };
    for (c, ch) in p.ch.iter_mut().enumerate().take(2) {
        *ch = spp3::ChannelParams {
            source: c as u32,
            t_sett_vc: 20.0,
            t_sett_mg: 20.0,
            tau_tvc: 1.0,
            tau_tmg: 50.0,
            temperature_limit: 100.0,
            hard_temp_limit_headroom: 5.0,
            reb_ref: 4.0,
            rshunt: 0.1,
            ag: 10.0,
            thermal_ff_speedup_factor: 0.5,
            imon_scale: 1.0,
            vmon_scale: 1.0,
            sense_i: (2 * c + 1) as u32,
            sense_v: (2 * c) as u32,
            ..Default::default()
        };
    }
    assert!(spp3::Unit::init_error(&p).is_none());
    blob::build(UnitKind::Spp3, 2, 4, 48_000, p.as_bytes())
}

#[test]
fn the_unit_models_the_request_and_acknowledges_it() {
    let (mut unit, _, _) = load_unit(Some(&spp3_blob()), 48_000.0, 2, 4, Role::Protect);
    assert!(matches!(unit, AnyUnit::Spp3(_)));
    let scale = |u: &AnyUnit| match u {
        AnyUnit::Spp3(u) => u.drive_scale(),
        _ => f32::NAN,
    };
    // the kernel's floor, -20 dB
    assert_eq!(unit.follow_drive_cap(Some((5, -40))), (5, -40));
    assert!((scale(&unit) - 0.1).abs() < 1e-6);
    assert!((scale(&unit) - drive_scale(-40)).abs() < 1e-7);
    // -6 dB
    assert_eq!(unit.follow_drive_cap(Some((6, -12))), (6, -12));
    assert!((scale(&unit) - 10f32.powf(-6.0 / 20.0)).abs() < 1e-6);
    // no fresh request: the blob's own operating point, the most drive
    assert_eq!(unit.follow_drive_cap(None), (0, 0));
    assert_eq!(scale(&unit), 1.0);
}

#[test]
fn a_unit_that_cannot_scale_acknowledges_the_most_drive() {
    let mut fs = AnyUnit::FailSafe(FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS));
    assert_eq!(fs.follow_drive_cap(Some((9, -20))), (9, 0));
    assert_eq!(fs.follow_drive_cap(None), (0, 0));
}

#[test]
fn the_ack_reaches_the_supervisor_through_the_page() {
    let path = std::env::temp_dir().join(format!("sgv2-drive-cap-{}", std::process::id()));
    let path = path.to_string_lossy().into_owned();
    let sup = EnginePage::create(&path).unwrap();
    let plugin = EnginePage::open(&path).unwrap();
    let (mut unit, _, _) = load_unit(Some(&spp3_blob()), 48_000.0, 2, 4, Role::Protect);
    sup.set_drive_cap(41, -30);
    let ack = unit.follow_drive_cap(plugin.drive_cap());
    plugin.report_drive_cap(ack.0, ack.1);
    assert_eq!(sup.drive_cap_ack(), (41, -30));
    let _ = std::fs::remove_file(&path);
}

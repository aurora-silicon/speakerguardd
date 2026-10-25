// SPDX-License-Identifier: MIT
//! Offline config and admission tests. No ALSA device is opened.

use super::*;
use std::collections::BTreeSet;

// Counts and routing are transcribed from the cited speakersafetyd 2.0.2
// profiles (including its aliases). Later boards deliberately retain a
// sibling map until captures can replace that assumption.
const ADDED: &[(&str, &str, usize, f64)] = &[
    ("j180", include_str!("../conf/apple/j180.conf"), 2, 16.0),
    ("j274", include_str!("../conf/apple/j274.conf"), 1, 21.0),
    ("j293", include_str!("../conf/apple/j293.conf"), 4, 18.5),
    ("j313", include_str!("../conf/apple/j313.conf"), 2, 16.0),
    ("j314", include_str!("../conf/apple/j314.conf"), 6, 18.5),
    ("j375", include_str!("../conf/apple/j375.conf"), 1, 21.0),
    ("j414", include_str!("../conf/apple/j414.conf"), 6, 18.5),
    ("j415", include_str!("../conf/apple/j415.conf"), 6, 18.5),
    ("j416", include_str!("../conf/apple/j416.conf"), 6, 18.5),
    ("j473", include_str!("../conf/apple/j473.conf"), 1, 21.0),
    ("j474", include_str!("../conf/apple/j474.conf"), 1, 21.0),
    ("j475", include_str!("../conf/apple/j475.conf"), 1, 21.0),
    ("j493", include_str!("../conf/apple/j493.conf"), 4, 18.5),
    ("j504", include_str!("../conf/apple/j504.conf"), 6, 18.5),
    ("j514", include_str!("../conf/apple/j514.conf"), 6, 18.5),
    ("j516", include_str!("../conf/apple/j516.conf"), 6, 18.5),
    ("j575", include_str!("../conf/apple/j575.conf"), 1, 21.0),
    ("j604", include_str!("../conf/apple/j604.conf"), 6, 21.0),
    ("j613", include_str!("../conf/apple/j613.conf"), 4, 18.5),
    ("j614", include_str!("../conf/apple/j614.conf"), 6, 21.0),
    ("j615", include_str!("../conf/apple/j615.conf"), 6, 18.5),
    ("j704", include_str!("../conf/apple/j704.conf"), 6, 21.0),
    ("j713", include_str!("../conf/apple/j713.conf"), 4, 21.0),
    ("j714", include_str!("../conf/apple/j714.conf"), 6, 21.0),
    ("j715", include_str!("../conf/apple/j715.conf"), 6, 21.0),
    ("j716", include_str!("../conf/apple/j716.conf"), 6, 21.0),
    ("j773", include_str!("../conf/apple/j773.conf"), 1, 21.0),
    ("j775", include_str!("../conf/apple/j775.conf"), 1, 21.0),
    ("j813", include_str!("../conf/apple/j813.conf"), 4, 21.0),
    ("j815", include_str!("../conf/apple/j815.conf"), 6, 21.0),
    ("j873", include_str!("../conf/apple/j873.conf"), 1, 21.0),
];

#[test]
fn every_public_draft_parses_and_refuses_to_govern_without_opt_in() {
    for &(board, text, speakers, gain) in ADDED {
        let mut c = Config::parse(text).unwrap_or_else(|e| panic!("{board}: {e}"));
        assert_eq!(c.speakers.len(), speakers, "{board}");
        assert_eq!(c.globals.channels, speakers * 2, "{board}");
        assert_eq!(c.globals.amp_gain_dbv, gain, "{board}");
        assert!(c.has_vi(), "{board}");
        assert_eq!(c.globals.sense_format, SenseFormat::S16, "{board}");
        assert!(
            !c.globals.qualified,
            "{board} was promoted without qualification"
        );
        assert!(!c.allow_unqualified, "{board}");
        assert_eq!(c.globals.governor, Governor::Unified, "{board}");
        assert_eq!(c.globals.guard, Some(Governor::Asahi), "{board}");
        assert!(!c.globals.unified_caps, "{board}");
        assert_eq!(c.globals.max_reduction, 7.0, "{board}");
        let fallback = match board {
            "j575" | "j604" | "j614" | "j704" | "j713" | "j714" | "j715" | "j716" | "j773"
            | "j775" | "j813" | "j815" | "j873" => 20.0,
            _ => 99.5,
        };
        assert_eq!(c.globals.safe_reduction, fallback, "{board}");
        assert!(
            c.check_governor().unwrap_err().0.contains("not qualified"),
            "{board}"
        );
        c.allow_unified = true;
        assert!(
            c.check_governor().is_err(),
            "{board}: experimental bypassed qualification"
        );
        // Opt-in is required even if the caller changes the governor.
        for governor in [Governor::Asahi, Governor::Legacy, Governor::Unified] {
            c.globals.governor = governor;
            c.globals.guard = if governor == Governor::Unified {
                Some(Governor::Asahi)
            } else {
                None
            };
            assert!(
                c.check_governor().unwrap_err().0.contains("not qualified"),
                "{board}"
            );
            c.allow_unqualified = true;
            assert!(
                c.check_governor().is_ok(),
                "{board}: explicit opt-in was not sufficient"
            );
            c.allow_unqualified = false;
        }
    }
}

#[test]
fn every_new_file_is_in_the_admission_matrix() {
    let expected: BTreeSet<_> = ADDED
        .iter()
        .map(|(b, ..)| b.to_string())
        .chain(["j316", "j413", "j616", "j700"].map(str::to_string))
        .collect();
    let actual: BTreeSet<_> =
        fs::read_dir(Path::new(env!("CARGO_MANIFEST_DIR")).join("conf/apple"))
            .unwrap()
            .map(|entry| entry.unwrap().path())
            .filter(|p| p.extension().is_some_and(|e| e == "conf"))
            .map(|p| p.file_stem().unwrap().to_str().unwrap().to_string())
            .collect();
    assert_eq!(
        actual, expected,
        "a shipped config lacks an explicit admission test"
    );
}

#[test]
fn draft_maps_cover_each_sense_slot_once_and_keep_roles_linked() {
    for &(board, text, ..) in ADDED {
        let c = Config::parse(text).unwrap();
        let mut seen = BTreeSet::new();
        for sp in &c.speakers {
            let i = sp.is_chan.unwrap();
            assert_eq!(sp.vs_chan, i + 1, "{board}: {}", sp.name);
            assert_eq!(i % 2, 0, "{board}: {}", sp.name);
            assert!(seen.insert(i), "{board}: duplicated current slot");
            assert!(seen.insert(sp.vs_chan), "{board}: duplicated voltage slot");
            // This is a pessimistic borrowed resistance, never a qualified measurement.
            assert_eq!(sp.z_min(), 2.7, "{board}");
            assert_eq!(sp.r_ff, 2.7, "{board}");
            assert_eq!(sp.p_limit_1s, 2.8154, "{board}");
            assert_eq!(sp.p_limit_60s, 2.5, "{board}");
            assert!(sp.z_min() <= sp.z_nominal, "{board}");
            assert_eq!(sp.tr_coil, sp.vi_tr_coil, "{board}");
            assert_eq!(sp.tr_magnet, sp.vi_tr_magnet, "{board}");
            assert_eq!(sp.tau_coil, sp.vi_tau_coil, "{board}");
            assert_eq!(sp.tau_magnet, sp.vi_tau_magnet, "{board}");
            assert_eq!(
                sp.group,
                u32::from(sp.name.contains("Woofer")),
                "{board}: {}",
                sp.name
            );
            assert!(sp.t_limit <= 140.0, "{board}");
        }
        assert_eq!(seen, (0..c.globals.channels).collect(), "{board}");
    }
}

#[test]
fn named_routes_match_the_public_or_declared_sibling_layout() {
    // The old six-output map interleaves tweeters before the second woofer.
    // J504 and its later 14-inch drafts put the tweeters at the end.
    const OLD_SIX: &[(&str, usize)] = &[
        ("Left Woofer 1", 0),
        ("Right Woofer 1", 2),
        ("Left Tweeter", 4),
        ("Right Tweeter", 6),
        ("Left Woofer 2", 8),
        ("Right Woofer 2", 10),
    ];
    const NEW_SIX: &[(&str, usize)] = &[
        ("Left Woofer 1", 0),
        ("Right Woofer 1", 2),
        ("Left Woofer 2", 4),
        ("Right Woofer 2", 6),
        ("Left Tweeter", 8),
        ("Right Tweeter", 10),
    ];
    for &(board, text, count, ..) in ADDED {
        let expected: &[(&str, usize)] = match board {
            "j180" => &[("Woofer", 0), ("Tweeter", 2)],
            "j274" | "j375" | "j473" | "j474" | "j475" | "j575" | "j773" | "j775" | "j873" => {
                &[("Mono", 0)]
            }
            "j313" => &[("Left", 0), ("Right", 2)],
            "j293" | "j493" => &[
                ("Left Front", 0),
                ("Right Front", 2),
                ("Left Rear", 4),
                ("Right Rear", 6),
            ],
            "j613" | "j713" | "j813" => &[
                ("Left Woofer", 0),
                ("Right Woofer", 2),
                ("Left Tweeter", 4),
                ("Right Tweeter", 6),
            ],
            "j504" | "j604" | "j614" | "j704" | "j714" | "j716" => NEW_SIX,
            _ => {
                assert_eq!(count, 6, "{board}");
                OLD_SIX
            }
        };
        let c = Config::parse(text).unwrap();
        assert_eq!(c.speakers.len(), expected.len(), "{board}");
        for &(name, i) in expected {
            let sp = c
                .speakers
                .iter()
                .find(|s| s.name == name)
                .unwrap_or_else(|| panic!("{board}: missing {name}"));
            assert_eq!(sp.is_chan, Some(i), "{board}: {name}");
            assert_eq!(sp.vs_chan, i + 1, "{board}: {name}");
        }
    }
}

//! The first-boot self-check's record, and what the supervisor demands of it.
//!
//! `speakerguardd-selfcheck` (packaging/session) derives this Mac's speaker
//! wiring from its device tree and its tuning, proves it silently on the
//! hardware, and writes `/var/lib/speakerguardd/selfcheck.bind`:
//!
//! ```text
//! schema=1
//! version=1
//! result=pass
//! board=j316s
//! kernel=7.3.0-aurora-sgv2k5
//! params_sha256=<64 hex>
//! wiring=<16 hex>
//! card=AppleJ316
//! amps=6
//! sense=12
//! family=sn012776
//! key=j316
//! method=derived
//! ```
//!
//! A board with a gate.json cap on its operating volume
//! (`operating_volume_cap_db`) adds it, in hundredths of a dB on the volume
//! control's own TLV: `volume_cap_cdb=-1600`. The cap only lowers what the
//! machine reports: the operating volume is the ceiling the card reports
//! under the grant (the volume controls' TLV maximum), and a ceiling above
//! the cap is not granted. Without the line there is no cap below 0 dB.
//!
//! The supervisor grants only with a passing record for exactly this board,
//! this kernel release and this parameter blob, and then only while the
//! session helper reports that the protect stage was built with the same
//! wiring. Anything else is refused with the reason, and the kernel's floor
//! holds.

use crate::machine::{Amp, Machine};

/// The record, parsed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Bind {
    pub result: String,
    pub board: String,
    pub kernel: String,
    pub params_sha256: String,
    /// The wiring hash (16 hex digits) as a number.
    pub wiring: u64,
    pub card: String,
    pub amps: usize,
    pub sense: usize,
    pub family: String,
    pub key: String,
    pub method: String,
    /// gate.json's cap on the operating volume, cdB on the control's own
    /// TLV (`volume_cap_cdb`; `None`: no cap below 0 dB).
    pub volume_cap_cdb: Option<i32>,
    /// The first failure, for a failing record.
    pub reason: Option<String>,
}

/// Parses a 16-digit lower-case hex wiring hash.
pub fn parse_wiring(s: &str) -> Option<u64> {
    (s.len() == 16 && s.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)))
        .then(|| u64::from_str_radix(s, 16).ok())
        .flatten()
}

pub fn parse(text: &str) -> Result<Bind, String> {
    let mut get = std::collections::BTreeMap::new();
    for line in text.lines() {
        let Some((k, v)) = line.split_once('=') else { return Err(format!("malformed line {line:?}")) };
        if get.insert(k, v).is_some() {
            return Err(format!("{k} given twice"));
        }
    }
    let field = |k: &str| get.get(k).map(|v| v.to_string()).ok_or_else(|| format!("no {k}"));
    if field("schema")? != "1" {
        return Err(format!("schema {}", field("schema")?));
    }
    let num = |k: &str| field(k)?.parse::<usize>().map_err(|_| format!("{k} is not a number"));
    // The operating volume itself is the machine's (the card's ceiling): a
    // record that names one is from a self-check that took it from
    // gate.json, and is checked again.
    if get.contains_key("volume_cdb") {
        return Err("volume_cdb is from a self-check that took the volume from gate.json: run it again".into());
    }
    // A cap only ever lowers the volume: at most 0 dB, and no lower than
    // the -40 dB a speaker would still be worth granting at.
    let volume_cap_cdb = match get.get("volume_cap_cdb") {
        None => None,
        Some(v) => match v.parse::<i32>() {
            Ok(c) if (-4000..=0).contains(&c) => Some(c),
            _ => return Err(format!("volume_cap_cdb {v:?} is not a volume from -4000 to 0 cdB")),
        },
    };
    Ok(Bind {
        result: field("result")?,
        board: field("board")?,
        kernel: field("kernel")?,
        params_sha256: field("params_sha256")?,
        wiring: parse_wiring(&field("wiring")?).ok_or("the wiring is not 16 hex digits")?,
        card: field("card")?,
        amps: num("amps")?,
        sense: num("sense")?,
        family: field("family")?,
        key: field("key")?,
        method: field("method")?,
        volume_cap_cdb,
        reason: get.get("reason").map(|s| s.to_string()),
    })
}

/// Whether `bind` covers exactly this Mac, kernel and blob. `Err` says why
/// not, in words for the journal.
pub fn covers(bind: &Bind, board: &str, kernel: &str, params_sha256: &str) -> Result<(), String> {
    if bind.result != "pass" {
        return Err(format!(
            "the self-check failed on {} ({}): {}",
            bind.board,
            bind.kernel,
            bind.reason.as_deref().unwrap_or("no reason recorded")
        ));
    }
    if bind.board != board {
        return Err(format!("the self-check ran on {}, this Mac is {board}", bind.board));
    }
    if bind.kernel != kernel {
        return Err(format!("the self-check ran on kernel {}, this is {kernel}: run it again", bind.kernel));
    }
    if bind.params_sha256 != params_sha256 {
        // the first 12 characters: the record is text from a file, not
        // necessarily the hex it should be
        let short = |s: &str| s.chars().take(12).collect::<String>();
        return Err(format!(
            "the self-check ran with parameters {}, these are {}: run it again",
            short(&bind.params_sha256),
            short(params_sha256)
        ));
    }
    Ok(())
}

/// The machine for a board, from its self-check record: the card, the amp
/// count and the amp family the self-check derived from this Mac's device
/// tree and proved on its card. There is no table of boards: a new Mac
/// runs on its own record. A family with V/I sense must have been proven
/// with it (`method=derived`, two sense channels an amp), unless the
/// self-check found that this Mac's speaker transport carries no sense
/// (`method=derived-nosense`, no sense: the J773g's AOP, whose device tree
/// names no sense DMA). That is admitted for one amp only: without sense
/// nothing proves which of several amps plays which channel. One without
/// has nothing to prove silently (`method=derived-static`, no sense).
pub fn machine_for(board: &str, bind: &Bind) -> Result<&'static Machine, String> {
    if bind.board != board {
        return Err(format!("the self-check ran on {}, this Mac is {board}", bind.board));
    }
    let Some(amp) = Amp::from_family(&bind.family) else {
        return Err(format!("{board}: amp family {} has no operating-point check yet", bind.family));
    };
    if bind.amps == 0 {
        return Err(format!("{board}: the self-check found no amps"));
    }
    let (method, sense) = match (amp.has_sense(), bind.method.as_str()) {
        (true, "derived-nosense") if bind.amps == 1 => ("derived-nosense", 0),
        (true, "derived-nosense") => {
            return Err(format!(
                "{board}: {} amps were proven without V/I sense, which is admitted for one amp only",
                bind.amps
            ))
        }
        (true, _) => ("derived", 2 * bind.amps),
        (false, _) => ("derived-static", 0),
    };
    if bind.method != method {
        return Err(format!("{board}: {} amps need a {method} wiring, the record says {}", bind.family, bind.method));
    }
    if bind.sense != sense {
        return Err(format!("{board}: {} sense channels for {} {} amps", bind.sense, bind.amps, bind.family));
    }
    // Leaked once at startup: the machine is &'static.
    let leak = |s: &str| -> &'static str { Box::leak(s.to_owned().into_boxed_str()) };
    Ok(Box::leak(Box::new(Machine { board: leak(board), card_id: leak(&bind.card), amps: bind.amps, amp })))
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOOD: &str = "schema=1\nversion=1\nresult=pass\nboard=j316s\nkernel=7.3.0-aurora-sgv2k5\n\
        params_sha256=aa11\nwiring=0979c55ed5ddf377\ncard=AppleJ316\namps=6\nsense=12\nfamily=sn012776\n\
        key=j316\nmethod=derived\n";

    #[test]
    fn a_passing_record_covers_exactly_its_board_kernel_and_blob() {
        let b = parse(GOOD).unwrap();
        assert_eq!(b.wiring, 0x0979_c55e_d5dd_f377);
        assert_eq!(covers(&b, "j316s", "7.3.0-aurora-sgv2k5", "aa11"), Ok(()));
        assert!(covers(&b, "j316c", "7.3.0-aurora-sgv2k5", "aa11").unwrap_err().contains("this Mac is j316c"));
        assert!(covers(&b, "j316s", "7.3.0-aurora-sgv2k6", "aa11").unwrap_err().contains("run it again"));
        assert!(covers(&b, "j316s", "7.3.0-aurora-sgv2k5", "bb22").unwrap_err().contains("parameters"));
    }

    #[test]
    fn a_parameter_hash_that_is_not_hex_is_refused_without_a_panic() {
        // a multi-byte character across the 12 bytes the message quotes
        let t = GOOD.replace("params_sha256=aa11", "params_sha256=0123456789a\u{e9}f");
        let b = parse(&t).unwrap();
        let e = covers(&b, "j316s", "7.3.0-aurora-sgv2k5", "aa11").unwrap_err();
        assert!(e.contains("0123456789a\u{e9}") && e.contains("aa11"), "{e}");
        let e = covers(&parse(GOOD).unwrap(), "j316s", "7.3.0-aurora-sgv2k5", "0123456789a\u{e9}f").unwrap_err();
        assert!(e.contains("parameters aa11"), "{e}");
    }

    #[test]
    fn a_failing_record_covers_nothing_and_says_why() {
        let t = GOOD.replace("result=pass", "result=fail") + "reason=Left Woofer 1 VSENSE off quietened channel 1\n";
        let b = parse(&t).unwrap();
        let e = covers(&b, "j316s", "7.3.0-aurora-sgv2k5", "aa11").unwrap_err();
        assert!(e.contains("failed") && e.contains("VSENSE"), "{e}");
    }

    #[test]
    fn a_record_may_cap_the_volume_below_0_db() {
        assert_eq!(parse(GOOD).unwrap().volume_cap_cdb, None);
        assert_eq!(parse(&format!("{GOOD}volume_cap_cdb=-1600\n")).unwrap().volume_cap_cdb, Some(-1600));
        assert_eq!(parse(&format!("{GOOD}volume_cap_cdb=0\n")).unwrap().volume_cap_cdb, Some(0));
        // a cap only lowers the volume, within -40 dB, and is a number
        for v in ["100", "1", "-4001", "-16.0", "", "x"] {
            assert!(parse(&format!("{GOOD}volume_cap_cdb={v}\n")).is_err(), "{v:?}");
        }
        assert!(parse(&format!("{GOOD}volume_cap_cdb=-1600\nvolume_cap_cdb=-1600\n")).is_err());
        // the volume itself is the card's: a record that names it is checked again
        for v in ["-1600", "0"] {
            assert!(parse(&format!("{GOOD}volume_cdb={v}\n")).is_err(), "{v:?}");
        }
    }

    #[test]
    fn malformed_records_are_refused() {
        assert!(parse(&GOOD.replace("wiring=0979c55ed5ddf377", "wiring=0979C55ED5DDF377")).is_err());
        assert!(parse(&GOOD.replace("wiring=0979c55ed5ddf377", "wiring=12")).is_err());
        assert!(parse(&GOOD.replace("schema=1", "schema=2")).is_err());
        assert!(parse(&(GOOD.to_owned() + "board=j413\n")).is_err());
        assert!(parse(&GOOD.replace("amps=6", "amps=six")).is_err());
        assert!(parse("garbage").is_err());
        assert!(parse(&GOOD.replace("method=derived\n", "")).is_err());
    }

    #[test]
    fn every_board_comes_from_its_record() {
        let b = parse(GOOD).unwrap();
        let m = machine_for("j316s", &b).unwrap();
        assert_eq!((m.board, m.card_id, m.amps, m.amp), ("j316s", "AppleJ316", 6, Amp::Sn012776));
        // a board nothing was written for
        let new = Bind { board: "j999".into(), card: "AppleJ999".into(), ..b.clone() };
        let m = machine_for("j999", &new).unwrap();
        assert_eq!((m.board, m.card_id, m.amps, m.amp), ("j999", "AppleJ999", 6, Amp::Sn012776));
        assert!(machine_for("j316c", &b).is_err());
        assert!(machine_for("j999", &Bind { method: "hand".into(), ..new.clone() }).is_err());
        assert!(machine_for("j999", &Bind { method: "derived-static".into(), ..new.clone() }).is_err());
        assert!(machine_for("j999", &Bind { family: "tas3000".into(), ..new.clone() }).is_err());
        // the TAS5770L Macs (J274, J293, J313): V/I sense, proven with it
        let tas = Bind {
            board: "j293".into(),
            card: "AppleJ293".into(),
            amps: 4,
            sense: 8,
            family: "tas2770".into(),
            ..b.clone()
        };
        let m = machine_for("j293", &tas).unwrap();
        assert_eq!((m.amps, m.amp), (4, Amp::Tas2770));
        assert!(m.amp.has_gain_control() && m.amp.has_sense() && !m.amp.reports_faults());
        assert_eq!((m.amp.volume_suffix(), m.amp.volume_controls(m.amps)), ("Speaker Playback Volume", m.amps));
        assert!(machine_for("j293", &Bind { sense: 0, method: "derived-static".into(), ..tas }).is_err());
        assert!(machine_for("j999", &Bind { sense: 0, ..new.clone() }).is_err());
        assert!(machine_for("j999", &Bind { amps: 0, sense: 0, ..new.clone() }).is_err());
        // a fixed-gain pair without sense: derived, nothing to prove silently
        let pair =
            Bind { amps: 2, sense: 0, family: "max98360a".into(), method: "derived-static".into(), ..new.clone() };
        assert_eq!(machine_for("j999", &pair).unwrap().amp, Amp::Max98360a);
        assert!(machine_for("j999", &Bind { sense: 4, ..pair.clone() }).is_err());
        assert!(machine_for("j999", &Bind { method: "derived".into(), ..pair.clone() }).is_err());
        assert!(machine_for("j999", &Bind { method: "derived-nosense".into(), ..pair }).is_err());
        // a sense family on a transport without sense (the J773g): no sense
        let mono = Bind { amps: 1, sense: 0, method: "derived-nosense".into(), ..new.clone() };
        assert_eq!(machine_for("j999", &mono).unwrap().amp, Amp::Sn012776);
        assert!(machine_for("j999", &Bind { sense: 2, ..mono.clone() }).is_err());
        assert!(machine_for("j999", &Bind { method: "derived-static".into(), ..mono.clone() }).is_err());
        // several sense amps without sense: their order would be unproven
        let several = machine_for("j999", &Bind { amps: 4, ..mono }).unwrap_err();
        assert!(several.contains("one amp only"), "{several}");
    }
}

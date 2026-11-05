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
        return Err(format!(
            "the self-check ran with parameters {}, these are {}: run it again",
            &bind.params_sha256[..bind.params_sha256.len().min(12)],
            &params_sha256[..params_sha256.len().min(12)]
        ));
    }
    Ok(())
}

/// The machine for a board, from its self-check record: the card, the amp
/// count and the amp family the self-check derived from this Mac's device
/// tree and proved on its card. There is no table of boards: a new Mac
/// runs on its own record. A family with V/I sense must have been proven
/// with it (`method=derived`, two sense channels an amp); one without has
/// nothing to prove silently (`method=derived-static`, no sense).
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
    let (method, sense) = if amp.has_sense() { ("derived", 2 * bind.amps) } else { ("derived-static", 0) };
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
    fn a_failing_record_covers_nothing_and_says_why() {
        let t = GOOD.replace("result=pass", "result=fail") + "reason=Left Woofer 1 VSENSE off quietened channel 1\n";
        let b = parse(&t).unwrap();
        let e = covers(&b, "j316s", "7.3.0-aurora-sgv2k5", "aa11").unwrap_err();
        assert!(e.contains("failed") && e.contains("VSENSE"), "{e}");
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
        assert!(machine_for("j999", &Bind { family: "tas2770".into(), ..new.clone() }).is_err());
        assert!(machine_for("j999", &Bind { sense: 0, ..new.clone() }).is_err());
        assert!(machine_for("j999", &Bind { amps: 0, sense: 0, ..new.clone() }).is_err());
        // a fixed-gain pair without sense: derived, nothing to prove silently
        let pair =
            Bind { amps: 2, sense: 0, family: "max98360a".into(), method: "derived-static".into(), ..new.clone() };
        assert_eq!(machine_for("j999", &pair).unwrap().amp, Amp::Max98360a);
        assert!(machine_for("j999", &Bind { sense: 4, ..pair.clone() }).is_err());
        assert!(machine_for("j999", &Bind { method: "derived".into(), ..pair }).is_err());
    }
}

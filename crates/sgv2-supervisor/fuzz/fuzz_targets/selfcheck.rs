//! The first-boot self-check record (`selfcheck.bind`): `parse`, then
//! `covers` and `machine_for` against fuzzed board, kernel and blob names.
//! Nothing may panic (the supervisor is `panic = "abort"`); a parsed record
//! renders back to text that parses to the same record; a machine is only
//! ever built for the record's own board, with amps and the sense count its
//! method demands.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sgv2_supervisor::selfcheck::{covers, machine_for, parse, parse_wiring, Bind};

fn render(b: &Bind) -> String {
    let mut t = format!(
        "schema=1\nresult={}\nboard={}\nkernel={}\nparams_sha256={}\nwiring={:016x}\ncard={}\namps={}\nsense={}\n\
         family={}\nkey={}\nmethod={}\n",
        b.result, b.board, b.kernel, b.params_sha256, b.wiring, b.card, b.amps, b.sense, b.family, b.key, b.method
    );
    if let Some(r) = &b.reason {
        t += &format!("reason={r}\n");
    }
    t
}

fuzz_target!(|data: &[u8]| {
    // the record, then three NUL-separated names to check it against
    let mut parts = data.splitn(4, |&b| b == 0);
    let text = String::from_utf8_lossy(parts.next().unwrap_or_default());
    let names: Vec<String> = parts.map(|p| String::from_utf8_lossy(p).into_owned()).collect();
    let name = |i: usize| names.get(i).map(String::as_str).unwrap_or("");

    if let Some(w) = parse_wiring(&text) {
        assert_eq!(format!("{w:016x}"), *text, "the wiring hash is its 16 hex digits");
    }
    let Ok(bind) = parse(&text) else { return };
    // a value that ends in a carriage return loses it to `lines()`; any
    // other record survives the round trip
    let fields = [&bind.result, &bind.board, &bind.kernel, &bind.params_sha256, &bind.card, &bind.family];
    if !fields.into_iter().chain([&bind.key, &bind.method]).any(|v| v.ends_with('\r'))
        && !bind.reason.as_deref().is_some_and(|r| r.ends_with('\r'))
    {
        assert_eq!(parse(&render(&bind)).as_ref(), Ok(&bind), "the record does not round-trip");
    }
    let _ = covers(&bind, name(0), name(1), name(2));
    let _ = covers(&bind, &bind.board.clone(), &bind.kernel.clone(), name(2));
    for board in [name(0).to_owned(), bind.board.clone()] {
        if let Ok(m) = machine_for(&board, &bind) {
            assert_eq!(m.board, bind.board, "a machine for another board");
            assert!(m.amps > 0 && m.amps == bind.amps, "{} amps", m.amps);
            let sense = if m.amp.has_sense() && bind.method == "derived" { 2 * m.amps } else { 0 };
            assert_eq!(bind.sense, sense, "sense count against the method");
        }
    }
});

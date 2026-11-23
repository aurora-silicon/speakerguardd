//! The supervisor's `/proc` and session-helper readers: PCM status and
//! info, `Tgid:`, `/proc/<pid>/stat` start time, and the helper's report
//! (written by the session user). Nothing may panic; a report that parses
//! as present is well formed, and a PCM state survives its lock-free
//! packing.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sgv2_supervisor::observe::{
    combine, helper_is, owner_to_tgid, parse_pcm_info, parse_pcm_status, parse_session, parse_stat_start, parse_tgid,
    speaker_pcm_problem, PcmState,
};

fuzz_target!(|data: &[u8]| {
    let Some((&sel, rest)) = data.split_first() else { return };
    let text = String::from_utf8_lossy(rest);
    let t = text.as_ref();

    let st = parse_pcm_status(t);
    assert_eq!(PcmState::unpack(st.pack()), st, "pack/unpack");
    assert!(!matches!(st, PcmState::Opening), "a /proc parse never says Opening");
    let mapped = owner_to_tgid(st, &mut |o| (o % 3 != 0).then_some(o / 2));
    assert_eq!(mapped.owner().is_some(), st.owner().is_some_and(|o| o % 3 != 0));
    let half = t.len() / 2;
    let (a, b) = (t.get(..half).unwrap_or(""), t.get(half..).unwrap_or(""));
    let all = [st, parse_pcm_status(a), parse_pcm_status(b)];
    let c = combine(&all);
    if let Some(o) = c.owner() {
        let owners: Vec<u32> = all.iter().filter_map(|s| s.owner()).collect();
        assert!(o == 0 || owners.iter().all(|x| *x == o), "combined owner {o} of {owners:?}");
    }
    // any snapshot word unpacks to a state that packs back to itself
    let any = PcmState::unpack(u64::from(sel) << 32 | u64::from(st.owner().unwrap_or(7)));
    assert_eq!(PcmState::unpack(any.pack()), any);

    let _ = parse_tgid(t);
    if let Some(info) = parse_pcm_info(t) {
        if speaker_pcm_problem(&info, info.card, info.device).is_none() {
            assert!(info.playback && (1..=8).contains(&info.subdevices));
            assert!(info.id == "Primary" || info.id == "Secondary");
        }
    }

    let start = parse_stat_start(t);
    let r = parse_session(t);
    if r.present {
        assert!(t.ends_with('\n') && r.mono_ns > 0, "present report without its newline or its stamp");
        let (pid, st0) = r.helper.expect("a present report names its writer");
        assert!(pid > 0);
        assert!(!r.helper_alive, "the parse never proves the writer alive");
        let alive = helper_is(&r, Some(t), Some(u32::from(sel)), u32::from(sel));
        assert_eq!(alive, start == Some(st0));
    } else {
        assert_eq!(r, Default::default(), "an absent report carries nothing");
    }
});

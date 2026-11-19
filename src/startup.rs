// SPDX-License-Identifier: MIT
//! Bounded card discovery, before any speaker interlock operation.

use std::io;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const POLL: Duration = Duration::from_millis(100);

trait Clock {
    fn elapsed(&self) -> Duration;
    fn sleep(&mut self, delay: Duration);
}

struct MonotonicClock(Instant);

impl Clock for MonotonicClock {
    fn elapsed(&self) -> Duration {
        self.0.elapsed()
    }

    fn sleep(&mut self, delay: Duration) {
        std::thread::sleep(delay);
    }
}

/// Only absence is retried. Permission, ioctl and other errors still fail
/// immediately. The probe must only discover the card, never unlock it.
pub fn wait_for_card<T>(timeout: Duration, probe: impl FnMut() -> io::Result<T>) -> io::Result<T> {
    wait(timeout, probe, &mut MonotonicClock(Instant::now()))
}

fn wait<T>(
    timeout: Duration,
    mut probe: impl FnMut() -> io::Result<T>,
    clock: &mut impl Clock,
) -> io::Result<T> {
    loop {
        match probe() {
            Ok(card) => return Ok(card),
            Err(e) if e.kind() == io::ErrorKind::NotFound => {
                let remaining = timeout.saturating_sub(clock.elapsed());
                if remaining.is_zero() {
                    return Err(e);
                }
                clock.sleep(POLL.min(remaining));
            }
            Err(e) => return Err(e),
        }
    }
}

/// The id of the speaker card in `/proc/asound/cards`: the one the
/// macaudio driver registered.  Apple machines carry other cards whose ids
/// also start with "AppleJ" (the AOP's "AppleJ700AOP", "AppleJ316HPAI"),
/// and the order of /proc/asound entries is not a promise.
pub fn macaudio_card(cards: &str) -> Option<String> {
    cards.lines().find_map(|line| {
        // " 1 [AppleJ700      ]: macaudio - MacBook Neo J700"
        let (_, rest) = line.split_once('[')?;
        let (id, rest) = rest.split_once(']')?;
        let driver = rest.strip_prefix(':')?.split(" - ").next()?.trim();
        (driver == "macaudio").then(|| id.trim().to_string())
    })
}

/// The profile file a card id names: Apple's ADT-derived "AppleJ<model>"
/// to `apple/j<model>.conf` (AppleJ700 -> apple/j700.conf).
pub fn profile_path(dir: &Path, card: &str) -> PathBuf {
    let model = card.strip_prefix("Apple").unwrap_or(card).to_lowercase();
    dir.join("apple").join(format!("{model}.conf"))
}

/// Whether this daemon is the one to protect `card`'s speakers (the unit's
/// start condition): it ships a profile for the card, and speakersafetyd
/// (profiles under `defer_to`) does not.  A card speakersafetyd describes
/// is left to it, so the udev rule can start the unit for every macaudio
/// card without taking one over; a card neither describes keeps its
/// speakers at the kernel's safe level.  Err carries why not.
pub fn owns_card(dir: &Path, defer_to: Option<&Path>, card: &str) -> Result<PathBuf, String> {
    let ours = profile_path(dir, card);
    if !ours.is_file() {
        return Err(format!("{card}: no speakerguardd profile ({} absent)", ours.display()));
    }
    if let Some(other) = defer_to.map(|d| profile_path(d, card)) {
        // anything there but a clear absence (a file, or one we cannot
        // check) leaves the card to speakersafetyd
        match std::fs::symlink_metadata(&other) {
            Err(e) if e.kind() == io::ErrorKind::NotFound => {}
            Ok(_) => return Err(format!("{card}: speakersafetyd has a profile ({}); leaving the card to it", other.display())),
            Err(e) => return Err(format!("{card}: {}: {e}; leaving the card to speakersafetyd", other.display())),
        }
    }
    Ok(ours)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    use std::rc::Rc;

    #[derive(Default)]
    struct FakeClock {
        now: Rc<Cell<Duration>>,
        sleeps: Vec<Duration>,
    }

    impl Clock for FakeClock {
        fn elapsed(&self) -> Duration {
            self.now.get()
        }

        fn sleep(&mut self, delay: Duration) {
            assert!(delay > Duration::ZERO && delay <= POLL);
            self.sleeps.push(delay);
            self.now.set(self.now.get() + delay);
        }
    }

    fn missing<T>() -> io::Result<T> {
        Err(io::Error::new(
            io::ErrorKind::NotFound,
            "no sound card with id AppleJ700",
        ))
    }

    #[test]
    fn an_existing_card_starts_without_sleeping() {
        let mut clock = FakeClock::default();
        assert_eq!(
            wait(Duration::from_secs(30), || Ok("AppleJ700"), &mut clock).unwrap(),
            "AppleJ700"
        );
        assert!(clock.sleeps.is_empty());
    }

    #[test]
    fn a_late_card_succeeds_in_the_first_invocation() {
        // Covers the observed 1.3 s boot race and cards arriving much later.
        for delay_ms in [100, 1300, 8900, 29_900, 30_000] {
            let mut clock = FakeClock::default();
            let now = clock.now.clone();
            let result = wait(
                Duration::from_secs(30),
                || {
                    if now.get() >= Duration::from_millis(delay_ms) {
                        Ok("AppleJ700")
                    } else {
                        missing()
                    }
                },
                &mut clock,
            );
            assert_eq!(result.unwrap(), "AppleJ700");
            assert_eq!(clock.elapsed(), Duration::from_millis(delay_ms));
        }
    }

    #[test]
    fn an_absent_card_fails_at_the_deadline_with_the_discovery_error() {
        let mut clock = FakeClock::default();
        let mut probes = 0;
        let err = wait(
            Duration::from_millis(1250),
            || {
                probes += 1;
                missing::<()>()
            },
            &mut clock,
        )
        .unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::NotFound);
        assert_eq!(err.to_string(), "no sound card with id AppleJ700");
        assert_eq!(clock.elapsed(), Duration::from_millis(1250));
        assert_eq!(clock.sleeps.last(), Some(&Duration::from_millis(50)));
        assert_eq!(probes, 14);
    }

    #[test]
    fn a_card_after_the_deadline_does_not_extend_the_wait() {
        let mut clock = FakeClock::default();
        let now = clock.now.clone();
        let result = wait(
            Duration::from_secs(30),
            || {
                if now.get() >= Duration::from_millis(30_100) {
                    Ok(())
                } else {
                    missing()
                }
            },
            &mut clock,
        );
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::NotFound);
        assert_eq!(clock.elapsed(), Duration::from_secs(30));
    }

    #[test]
    fn a_zero_timeout_probes_once_and_does_not_sleep() {
        let mut clock = FakeClock::default();
        let mut probes = 0;
        assert!(wait(
            Duration::ZERO,
            || {
                probes += 1;
                missing::<()>()
            },
            &mut clock
        )
        .is_err());
        assert_eq!(probes, 1);
        assert!(clock.sleeps.is_empty());
        assert_eq!(wait(Duration::ZERO, || Ok(7), &mut clock).unwrap(), 7);
    }

    #[test]
    fn errors_other_than_absence_fail_immediately_even_after_a_retry() {
        for errno in [1, 5, 13, 19] {
            // EPERM, EIO, EACCES, ENODEV on Linux
            let mut clock = FakeClock::default();
            let mut probes = 0;
            let err = wait(
                Duration::from_secs(30),
                || {
                    probes += 1;
                    if probes == 1 {
                        missing::<()>()
                    } else {
                        Err(io::Error::from_raw_os_error(errno))
                    }
                },
                &mut clock,
            )
            .unwrap_err();
            assert_eq!(err.raw_os_error(), Some(errno));
            assert_eq!(probes, 2);
            assert_eq!(clock.elapsed(), POLL);
        }
    }

    #[test]
    fn time_spent_probing_counts_towards_the_deadline() {
        let mut clock = FakeClock::default();
        let now = clock.now.clone();
        let mut probes = 0;
        assert!(wait(
            Duration::from_secs(1),
            || {
                probes += 1;
                now.set(now.get() + Duration::from_millis(600));
                missing::<()>()
            },
            &mut clock
        )
        .is_err());
        assert_eq!(probes, 2);
        assert_eq!(clock.sleeps, vec![POLL]);
    }

    fn tree(files: &[&str]) -> PathBuf {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static N: AtomicUsize = AtomicUsize::new(0);
        let root = std::env::temp_dir().join(format!("sg-owns-{}-{}", std::process::id(), N.fetch_add(1, Ordering::Relaxed)));
        let _ = std::fs::remove_dir_all(&root);
        for f in files {
            let p = root.join(f);
            std::fs::create_dir_all(p.parent().unwrap()).unwrap();
            std::fs::write(p, "").unwrap();
        }
        std::fs::create_dir_all(&root).unwrap();
        root
    }

    #[test]
    fn profile_follows_the_card_id() {
        let d = Path::new("/usr/share/speakerguardd");
        assert_eq!(profile_path(d, "AppleJ700"), d.join("apple/j700.conf"));
        assert_eq!(profile_path(d, "AppleJ616"), d.join("apple/j616.conf"));
        assert_eq!(profile_path(d, "AppleJack"), d.join("apple/jack.conf"));
    }

    /// The J700 starts as before: our profile, none of speakersafetyd's.
    #[test]
    fn the_j700_is_ours() {
        let root = tree(&["sg/apple/j700.conf", "ss/apple/j316.conf", "ss/apple/j616.conf"]);
        let (sg, ss) = (root.join("sg"), root.join("ss"));
        assert_eq!(owns_card(&sg, Some(&ss), "AppleJ700").unwrap(), sg.join("apple/j700.conf"));
        assert!(owns_card(&sg, None, "AppleJ700").is_ok());
        // no speakersafetyd installed at all
        assert!(owns_card(&sg, Some(&root.join("absent")), "AppleJ700").is_ok());
        std::fs::remove_dir_all(root).unwrap();
    }

    /// Cards without a profile here (the other Macs, the J616s jack card)
    /// and cards speakersafetyd describes are not started.
    #[test]
    fn other_cards_are_left_alone() {
        let root = tree(&["sg/apple/j700.conf", "sg/apple/j413.conf", "ss/apple/j413.conf", "ss/apple/j316.conf"]);
        let (sg, ss) = (root.join("sg"), root.join("ss"));
        assert!(owns_card(&sg, Some(&ss), "AppleJ316").unwrap_err().contains("no speakerguardd profile"));
        assert!(owns_card(&sg, Some(&ss), "AppleJack").unwrap_err().contains("no speakerguardd profile"));
        // both describe it: speakersafetyd keeps it
        assert!(owns_card(&sg, Some(&ss), "AppleJ413").unwrap_err().contains("speakersafetyd has a profile"));
        // a profile that is a directory is not a profile
        std::fs::create_dir_all(sg.join("apple/j999.conf")).unwrap();
        assert!(owns_card(&sg, Some(&ss), "AppleJ999").is_err());
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn the_speaker_card_is_the_macaudio_one() {
        let neo = " 0 [AppleJ700AOP   ]: t8140-aop-audio - MacBook Neo J700 AOP\n                      MacBook Neo J700 AOP Audio\n 1 [AppleJ700      ]: macaudio - MacBook Neo J700\n                      MacBook Neo J700\n 2 [UDULTC4K       ]: USB-Audio - Plugable UD-ULTC4K\n                      DisplayLink Plugable UD-ULTC4K at usb-xhci-hcd.2.auto-1.2, super speed\n";
        assert_eq!(macaudio_card(neo).as_deref(), Some("AppleJ700"));
        let m1pro = " 0 [AppleJ316HPAI  ]: aop_audio - MacBook Pro J316 HPAI\n                      MacBook Pro J316 High-Power Audio Interface\n 1 [AppleJ316      ]: macaudio - MacBook Pro J316\n                      MacBook Pro J316\n";
        assert_eq!(macaudio_card(m1pro).as_deref(), Some("AppleJ316"));
        assert_eq!(macaudio_card(" 0 [Generic        ]: HDA-Intel - HD-Audio Generic\n"), None);
        assert_eq!(macaudio_card(""), None);
        assert_eq!(macaudio_card("--- no cards ---\n"), None);
    }
}

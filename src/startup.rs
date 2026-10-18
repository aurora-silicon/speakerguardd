// SPDX-License-Identifier: MIT
//! Bounded card discovery, before any speaker interlock operation.

use std::io;
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
}

// SPDX-License-Identifier: MIT
//! Pair synchronization status with a realtime sample. Two status reads
//! alone miss an unsynchronized step between them. A CANCEL_ON_SET timer
//! watches the whole read, including both status queries.
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::Duration;

trait Clock {
    fn watch(&mut self) -> bool;
    fn synced(&mut self) -> bool;
    fn realtime(&mut self) -> Option<Duration>;
    fn unchanged(&mut self) -> bool;
}

fn sample(clock: &mut impl Clock) -> Option<Duration> {
    if !clock.watch() || !clock.synced() {
        return None;
    }
    let time = clock.realtime()?;
    (clock.synced() && clock.unchanged()).then_some(time)
}

struct LinuxClock(Option<OwnedFd>);
impl Clock for LinuxClock {
    fn watch(&mut self) -> bool {
        // SAFETY: creates a private descriptor; OwnedFd closes it on every path.
        let fd = unsafe {
            libc::timerfd_create(libc::CLOCK_REALTIME, libc::TFD_CLOEXEC | libc::TFD_NONBLOCK)
        };
        if fd < 0 {
            return false;
        }
        self.0 = Some(unsafe { OwnedFd::from_raw_fd(fd) });
        // A nonzero absolute timer is required for CANCEL_ON_SET. Expiry is
        // also a rejection, so the particular future date is not trusted.
        let timer = libc::itimerspec {
            it_interval: libc::timespec {
                tv_sec: 0,
                tv_nsec: 0,
            },
            it_value: libc::timespec {
                tv_sec: 4_102_444_800,
                tv_nsec: 0,
            },
        };
        (unsafe {
            libc::timerfd_settime(
                fd,
                libc::TFD_TIMER_ABSTIME | libc::TFD_TIMER_CANCEL_ON_SET,
                &timer,
                std::ptr::null_mut(),
            )
        }) == 0
    }
    fn synced(&mut self) -> bool {
        // SAFETY: modes 0 reads the kernel's state without changing it.
        let mut tx: libc::timex = unsafe { std::mem::zeroed() };
        let state = unsafe { libc::adjtimex(&mut tx) };
        // Leap transitions are not an elapsed-time proof either.
        state == libc::TIME_OK && tx.status & (libc::STA_UNSYNC | libc::STA_CLOCKERR) == 0
    }
    fn realtime(&mut self) -> Option<Duration> {
        let mut ts = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        // SAFETY: a valid writable timespec.
        let ok = unsafe { libc::clock_gettime(libc::CLOCK_REALTIME, &mut ts) } == 0;
        (ok && ts.tv_sec > 0 && (0..1_000_000_000).contains(&ts.tv_nsec))
            .then(|| Duration::new(ts.tv_sec as u64, ts.tv_nsec as u32))
    }
    fn unchanged(&mut self) -> bool {
        let Some(fd) = &self.0 else { return false };
        let mut ticks: u64 = 0;
        // SAFETY: writable u64 of the size required by timerfd read.
        let n = unsafe {
            libc::read(
                fd.as_raw_fd(),
                (&mut ticks as *mut u64).cast(),
                std::mem::size_of::<u64>(),
            )
        };
        // ECANCELED, expiry, EINTR and every other failure reject. Only an
        // armed, unexpired, uncancelled timer returns EAGAIN.
        n == -1 && io::Error::last_os_error().raw_os_error() == Some(libc::EAGAIN)
    }
}

pub fn synchronized_realtime() -> Option<Duration> {
    sample(&mut LinuxClock(None))
}

#[cfg(test)]
mod tests {
    use super::*;
    struct Transition {
        step: u64,
        status: [bool; 2],
        query: usize,
        changed: bool,
        watch_ok: bool,
    }
    impl Clock for Transition {
        fn watch(&mut self) -> bool {
            self.watch_ok
        }
        fn synced(&mut self) -> bool {
            let s = self.status[self.query];
            self.query += 1;
            s
        }
        fn realtime(&mut self) -> Option<Duration> {
            // Review witnesses: stale time, then a +1 s or +60 s NTP step.
            // The latter has no RTC. The change watch is independent of RTC.
            self.changed = self.step != 0;
            Some(Duration::from_secs(1_800_000_000))
        }
        fn unchanged(&mut self) -> bool {
            !self.changed
        }
    }
    #[test]
    fn synchronization_transitions_cannot_certify_a_stale_timestamp() {
        for step in [1, 60] {
            for status in [[false, true], [true, false], [true, true]] {
                let mut c = Transition {
                    step,
                    status,
                    query: 0,
                    changed: false,
                    watch_ok: true,
                };
                assert_eq!(sample(&mut c), None, "step {step}, status {status:?}");
            }
        }
        let mut c = Transition {
            step: 0,
            status: [true, true],
            query: 0,
            changed: false,
            watch_ok: true,
        };
        assert_eq!(sample(&mut c), Some(Duration::from_secs(1_800_000_000)));
        c.watch_ok = false;
        assert_eq!(sample(&mut c), None);
    }
    #[test]
    fn read_only_kernel_sample_does_not_change_the_clock() {
        // This needs neither clock-setting privileges nor a synchronized host.
        // It checks the real timerfd error path, including descriptor cleanup.
        let mut c = LinuxClock(None);
        assert!(c.watch());
        assert!(c.unchanged());
        assert!(c.realtime().is_some());
    }
}

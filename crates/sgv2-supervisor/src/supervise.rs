//! One supervisor tick: observe, decide, act on the kernel. Pure apart from
//! the trait objects, so every fault path is tested against the kernel
//! double.
//!
//! Rules:
//! - renew only on a clean observation (and the lease state machine's two
//!   clean ticks before the first grant);
//! - any fault releases at once (no waiting for the lease to run out);
//! - a hard amp fault (over-temperature, over-current, group off) releases
//!   and latches released for the life of the process: the engine is never
//!   asked to protect a faulted amp;
//! - a lapsed lease (`ETIMEDOUT`) is a fault for this tick: the kernel has
//!   already stopped playback, and the grant comes back only through two
//!   clean ticks;
//! - losing ownership (`EPERM`) or any other kernel error ends the loop, so
//!   the process exits and the kernel's floor holds (the control file
//!   closes, which withdraws protection at once).

use crate::kernel::{Kernel, KernelError};
use crate::lease::{Action, Lease, Observation};

#[derive(Debug, PartialEq, Eq)]
pub enum Event {
    Granted,
    Renewed,
    Released { why: Observation },
    StayedReleased,
    HardFault(String),
    Lapsed,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Fatal {
    Kernel(KernelError),
}

#[derive(Default)]
pub struct Supervisor {
    lease: Lease,
    latched: Option<String>,
}

impl Supervisor {
    /// Whether the last tick left protection granted.
    pub fn granted(&self) -> bool {
        self.latched.is_none() && self.lease.held()
    }

    pub fn latched_fault(&self) -> Option<&str> {
        self.latched.as_deref()
    }

    pub fn tick(&mut self, obs: Observation, kernel: &mut dyn Kernel) -> Result<Event, Fatal> {
        if self.latched.is_none() {
            let faults = kernel.faults().map_err(Fatal::Kernel)?;
            if let Some(f) = faults.iter().find(|f| f.hard()) {
                self.latched = Some(format!("{} (bits {:#04x}, group off {})", f.amp, f.bits, f.group_off));
                kernel.release().map_err(Fatal::Kernel)?;
                return Ok(Event::HardFault(self.latched.clone().unwrap_or_default()));
            }
        } else {
            // stay at the kernel's floor; a release is idempotent
            if kernel.granted().map_err(Fatal::Kernel)? {
                kernel.release().map_err(Fatal::Kernel)?;
            }
            return Ok(Event::StayedReleased);
        }
        let was_held = self.lease.held();
        match self.lease.tick(obs) {
            Action::Renew => match kernel.renew() {
                Ok(()) => Ok(if was_held { Event::Renewed } else { Event::Granted }),
                Err(KernelError::Lapsed) => {
                    // count it as a faulted tick: back through two clean ticks
                    self.lease.tick(Observation::default());
                    Ok(Event::Lapsed)
                }
                Err(KernelError::Sleeping) => {
                    self.lease.tick(Observation::default());
                    Ok(Event::StayedReleased)
                }
                Err(e) => Err(Fatal::Kernel(e)),
            },
            Action::Release => {
                kernel.release().map_err(Fatal::Kernel)?;
                Ok(Event::Released { why: obs })
            }
            Action::StayReleased => Ok(Event::StayedReleased),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::{AmpFault, Recorder};

    const CLEAN: Observation =
        Observation { heartbeat_fresh: true, plugin_ok: true, links_ok: true, carrier_ok: true, pcm_owner_ok: true };

    #[test]
    fn grants_after_two_clean_ticks_and_releases_at_once_on_a_fault() {
        let mut k = Recorder::default();
        let mut s = Supervisor::default();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert!(!k.held);
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        assert!(k.held);
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Renewed));
        let bad = Observation { heartbeat_fresh: false, ..CLEAN };
        assert_eq!(s.tick(bad, &mut k), Ok(Event::Released { why: bad }));
        assert!(!k.held);
        assert_eq!(k.releases, 1);
    }

    #[test]
    fn a_hard_amp_fault_latches_released() {
        let mut k = Recorder::default();
        let mut s = Supervisor::default();
        s.tick(CLEAN, &mut k).unwrap();
        s.tick(CLEAN, &mut k).unwrap();
        assert!(k.held);
        k.amp_faults.push(AmpFault {
            amp: "Left Tweeter".into(),
            bits: AmpFault::OVER_TEMPERATURE,
            events: 1,
            group_off: false,
        });
        assert!(matches!(s.tick(CLEAN, &mut k), Ok(Event::HardFault(_))));
        assert!(!k.held);
        k.amp_faults.clear();
        for _ in 0..10 {
            assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
            assert!(!k.held);
        }
        assert!(s.latched_fault().is_some());
    }

    #[test]
    fn limiter_bits_are_not_hard_faults() {
        let mut k = Recorder::default();
        k.amp_faults.push(AmpFault {
            amp: "Left Woofer 1".into(),
            bits: AmpFault::LIMITER_ACTIVE | AmpFault::BOP_MUTE,
            events: 3,
            group_off: false,
        });
        let mut s = Supervisor::default();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
    }

    struct Lapsing(Recorder, bool);
    impl Kernel for Lapsing {
        fn renew(&mut self) -> Result<(), KernelError> {
            if self.1 {
                self.1 = false;
                self.0.held = false;
                return Err(KernelError::Lapsed);
            }
            self.0.renew()
        }
        fn release(&mut self) -> Result<(), KernelError> {
            self.0.release()
        }
        fn granted(&mut self) -> Result<bool, KernelError> {
            self.0.granted()
        }
        fn faults(&mut self) -> Result<Vec<AmpFault>, KernelError> {
            self.0.faults()
        }
    }

    #[test]
    fn a_lapse_goes_back_through_two_clean_ticks() {
        let mut k = Lapsing(Recorder::default(), false);
        let mut s = Supervisor::default();
        s.tick(CLEAN, &mut k).unwrap();
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
        k.1 = true;
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Lapsed));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::StayedReleased));
        assert_eq!(s.tick(CLEAN, &mut k), Ok(Event::Granted));
    }

    struct Lost;
    impl Kernel for Lost {
        fn renew(&mut self) -> Result<(), KernelError> {
            Err(KernelError::NotOwner)
        }
        fn release(&mut self) -> Result<(), KernelError> {
            Ok(())
        }
        fn granted(&mut self) -> Result<bool, KernelError> {
            Ok(false)
        }
        fn faults(&mut self) -> Result<Vec<AmpFault>, KernelError> {
            Ok(vec![])
        }
    }

    #[test]
    fn losing_ownership_is_fatal() {
        let mut s = Supervisor::default();
        s.tick(CLEAN, &mut Lost).unwrap();
        assert_eq!(s.tick(CLEAN, &mut Lost), Err(Fatal::Kernel(KernelError::NotOwner)));
    }
}

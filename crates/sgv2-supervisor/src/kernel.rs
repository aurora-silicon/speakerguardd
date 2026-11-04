//! The kernel and amp interface, behind a trait so tests never touch
//! hardware.
//!
//! The kernel contract (macaudio, "Speaker Protection"):
//!
//! - The owner is the control file that holds `SNDRV_CTL_IOCTL_ELEM_LOCK` on
//!   the card's "Speaker Protection" control. Only one file can hold it.
//! - Writing a lease length in milliseconds ([`LEASE_MIN_MS`] to
//!   [`LEASE_MAX_MS`]) grants protection for that long: the kernel puts every
//!   speaker amp at its fixed operating gain and lets speaker playback start.
//!   Writing again renews it.
//! - Writing 0, unlocking the control or closing the file withdraws it at
//!   once: speaker playback is refused and stopped, and the amps go to the
//!   kernel's safe level (the hardware floor).
//! - A lease that runs out does the same, and the owner's next write fails
//!   with `ETIMEDOUT` and grants nothing ([`KernelError::Lapsed`]).
//! - The speaker volume controls are the kernel's: userspace never writes
//!   them, so there is no amp control traffic during playback.
//! - Each V/I amp has a read-only "<amp> Amp Fault Status" control: the
//!   interrupt's latched fault bits since probe, an event count and the
//!   SDZ-group shutdown flag. A change is notified on the control file.

/// The lease the supervisor asks for: three 10 ms ticks.
pub const LEASE_MS: u32 = 30;
/// The kernel's accepted range.
pub const LEASE_MIN_MS: u32 = 10;
pub const LEASE_MAX_MS: u32 = 100;

/// The protection control on the speaker card.
pub const PROTECTION_CONTROL: &str = "Speaker Protection";
/// The per-amp fault control's name suffix.
pub const FAULT_CONTROL_SUFFIX: &str = "Amp Fault Status";

#[derive(Debug, PartialEq, Eq)]
pub enum KernelError {
    /// No card with the protection control.
    NoCard,
    /// Another file owns the speakers (`EBUSY` on the lock).
    Busy,
    /// The lock was lost (`EPERM`): take ownership again.
    NotOwner,
    /// The lease ran out before this renewal, which granted nothing.
    Lapsed,
    /// A system sleep is in progress (`EBUSY` on the write).
    Sleeping,
    /// The kernel's safe level failed on an amp: no grant until reload.
    FloorFailed,
    Control(String),
}

/// One amp's fault report.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AmpFault {
    /// The amp's control prefix, for example "Left Tweeter".
    pub amp: String,
    /// The latched INT_LTCH0 bits since probe.
    pub bits: u8,
    /// Fault interrupts since probe.
    pub events: u32,
    /// The amp's SDZ group was forced off.
    pub group_off: bool,
}

impl AmpFault {
    pub const OVER_TEMPERATURE: u8 = 1 << 0;
    pub const OVER_CURRENT: u8 = 1 << 1;
    pub const TDM_CLOCK: u8 = 1 << 2;
    pub const LIMITER_ACTIVE: u8 = 1 << 3;
    pub const PVDD_BELOW_INFLECTION: u8 = 1 << 4;
    pub const LIMITER_MAX_ATTENUATION: u8 = 1 << 5;
    pub const BOP_INFINITE_HOLD: u8 = 1 << 6;
    pub const BOP_MUTE: u8 = 1 << 7;

    /// A hardware fault: protection is withdrawn and not granted again.
    pub fn hard(&self) -> bool {
        self.group_off || self.bits & (Self::OVER_TEMPERATURE | Self::OVER_CURRENT) != 0
    }
}

pub trait Kernel {
    /// Grant or renew protection for [`LEASE_MS`]. While granted the amps run
    /// at the fixed operating gain; otherwise the kernel's safe level applies.
    fn renew(&mut self) -> Result<(), KernelError>;
    /// Withdraw protection now (the kernel's safe level applies at once).
    fn release(&mut self) -> Result<(), KernelError>;
    /// Whether the kernel currently grants protection.
    fn granted(&mut self) -> Result<bool, KernelError>;
    /// Every amp's fault report (empty on amps without fault reporting).
    fn faults(&mut self) -> Result<Vec<AmpFault>, KernelError>;
}

/// A kernel double for tests: records calls.
#[derive(Default, Debug)]
pub struct Recorder {
    pub held: bool,
    pub renewals: u32,
    pub releases: u32,
    pub amp_faults: Vec<AmpFault>,
}

impl Kernel for Recorder {
    fn renew(&mut self) -> Result<(), KernelError> {
        self.held = true;
        self.renewals += 1;
        Ok(())
    }
    fn release(&mut self) -> Result<(), KernelError> {
        self.held = false;
        self.releases += 1;
        Ok(())
    }
    fn granted(&mut self) -> Result<bool, KernelError> {
        Ok(self.held)
    }
    fn faults(&mut self) -> Result<Vec<AmpFault>, KernelError> {
        Ok(self.amp_faults.clone())
    }
}

#[cfg(target_os = "linux")]
pub use alsa::Macaudio;

/// The ALSA control-device implementation: raw ioctls, no alsa-lib.
#[cfg(target_os = "linux")]
mod alsa {
    use super::*;
    use std::fs::{File, OpenOptions};
    use std::os::unix::fs::OpenOptionsExt;
    use std::os::unix::io::AsRawFd;

    pub(super) const IFACE_MIXER: i32 = 2;
    const ID_NAME_LEN: usize = 44;
    const O_CLOEXEC: i32 = 0o2000000;
    const EPERM: i32 = 1;
    const ENOENT: i32 = 2;
    const EIO: i32 = 5;
    const EBUSY: i32 = 16;
    const ETIMEDOUT: i32 = 110;

    #[repr(C)]
    #[derive(Clone, Copy)]
    pub(super) struct ElemId {
        pub numid: u32,
        pub iface: i32,
        pub device: u32,
        pub subdevice: u32,
        pub name: [u8; ID_NAME_LEN],
        pub index: u32,
    }

    #[repr(C)]
    pub(super) struct ElemValue {
        pub id: ElemId,
        pub indirect: u32,
        pub value: [i64; 128],
        pub reserved: [u8; 128],
    }

    #[repr(C)]
    pub(super) struct ElemList {
        pub offset: u32,
        pub space: u32,
        pub used: u32,
        pub count: u32,
        pub pids: *mut ElemId,
        pub reserved: [u8; 50],
    }

    const fn ioc(dir: u64, nr: u64, size: usize) -> u64 {
        (dir << 30) | ((size as u64) << 16) | (0x55 << 8) | nr
    }
    const W: u64 = 1;
    const RW: u64 = 3;
    pub(super) const ELEM_LIST: u64 = ioc(RW, 0x10, core::mem::size_of::<ElemList>());
    pub(super) const ELEM_READ: u64 = ioc(RW, 0x12, core::mem::size_of::<ElemValue>());
    pub(super) const ELEM_WRITE: u64 = ioc(RW, 0x13, core::mem::size_of::<ElemValue>());
    pub(super) const ELEM_LOCK: u64 = ioc(W, 0x14, core::mem::size_of::<ElemId>());
    pub(super) const SUBSCRIBE_EVENTS: u64 = ioc(RW, 0x16, core::mem::size_of::<i32>());

    extern "C" {
        fn ioctl(fd: i32, request: u64, ...) -> i32;
    }

    pub(super) fn elem_id(name: &str) -> ElemId {
        let mut id = ElemId { numid: 0, iface: IFACE_MIXER, device: 0, subdevice: 0, name: [0; ID_NAME_LEN], index: 0 };
        let n = name.len().min(ID_NAME_LEN - 1);
        id.name[..n].copy_from_slice(&name.as_bytes()[..n]);
        id
    }

    fn id_name(id: &ElemId) -> String {
        let end = id.name.iter().position(|&b| b == 0).unwrap_or(ID_NAME_LEN);
        String::from_utf8_lossy(&id.name[..end]).into_owned()
    }

    fn errno() -> i32 {
        std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
    }

    /// The speaker card's control file, owning "Speaker Protection".
    /// Dropping it closes the file, which withdraws protection at once.
    pub struct Macaudio {
        file: File,
    }

    impl Macaudio {
        /// Open `/dev/snd/controlC<card>`, take ownership and subscribe to
        /// its events.
        pub fn open(card: u32) -> Result<Self, KernelError> {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .custom_flags(O_CLOEXEC)
                .open(format!("/dev/snd/controlC{card}"))
                .map_err(|_| KernelError::NoCard)?;
            let k = Macaudio { file };
            let id = elem_id(PROTECTION_CONTROL);
            // SAFETY: a valid fd and a pointer to a live, correctly sized ElemId.
            if unsafe { ioctl(k.fd(), ELEM_LOCK, &id as *const ElemId) } < 0 {
                return Err(match errno() {
                    EBUSY => KernelError::Busy,
                    // Not a protected speaker card
                    ENOENT => KernelError::NoCard,
                    e => KernelError::Control(format!("lock: errno {e}")),
                });
            }
            let on: i32 = 1;
            // SAFETY: as above, with a live i32.
            if unsafe { ioctl(k.fd(), SUBSCRIBE_EVENTS, &on as *const i32) } < 0 {
                return Err(KernelError::Control(format!("subscribe: errno {}", errno())));
            }
            Ok(k)
        }

        /// The fd to poll for control events (grant changes, amp faults).
        pub fn fd(&self) -> i32 {
            self.file.as_raw_fd()
        }

        fn value(id: ElemId) -> Box<ElemValue> {
            Box::new(ElemValue { id, indirect: 0, value: [0; 128], reserved: [0; 128] })
        }

        fn write(&mut self, ms: u32) -> Result<(), KernelError> {
            let mut v = Self::value(elem_id(PROTECTION_CONTROL));
            v.value[0] = i64::from(ms);
            // SAFETY: a valid fd and a live, correctly sized ElemValue.
            if unsafe { ioctl(self.fd(), ELEM_WRITE, &mut *v as *mut ElemValue) } < 0 {
                return Err(match errno() {
                    ETIMEDOUT => KernelError::Lapsed,
                    EPERM => KernelError::NotOwner,
                    EBUSY => KernelError::Sleeping,
                    EIO => KernelError::FloorFailed,
                    e => KernelError::Control(format!("write: errno {e}")),
                });
            }
            Ok(())
        }

        fn read(&mut self, id: ElemId) -> Result<Box<ElemValue>, KernelError> {
            let mut v = Self::value(id);
            // SAFETY: a valid fd and a live, correctly sized ElemValue.
            if unsafe { ioctl(self.fd(), ELEM_READ, &mut *v as *mut ElemValue) } < 0 {
                return Err(KernelError::Control(format!("read {}: errno {}", id_name(&id), errno())));
            }
            Ok(v)
        }

        fn list(&mut self) -> Result<Vec<ElemId>, KernelError> {
            let mut l =
                ElemList { offset: 0, space: 0, used: 0, count: 0, pids: core::ptr::null_mut(), reserved: [0; 50] };
            // SAFETY: a valid fd and a live ElemList with no space: the kernel only counts.
            if unsafe { ioctl(self.fd(), ELEM_LIST, &mut l as *mut ElemList) } < 0 {
                return Err(KernelError::Control(format!("list: errno {}", errno())));
            }
            let mut ids = vec![elem_id(""); l.count as usize];
            l.space = l.count;
            l.pids = ids.as_mut_ptr();
            // SAFETY: pids points at `space` live ElemIds.
            if unsafe { ioctl(self.fd(), ELEM_LIST, &mut l as *mut ElemList) } < 0 {
                return Err(KernelError::Control(format!("list: errno {}", errno())));
            }
            ids.truncate(l.used as usize);
            Ok(ids)
        }
    }

    impl Kernel for Macaudio {
        fn renew(&mut self) -> Result<(), KernelError> {
            self.write(LEASE_MS)
        }
        fn release(&mut self) -> Result<(), KernelError> {
            self.write(0)
        }
        fn granted(&mut self) -> Result<bool, KernelError> {
            Ok(self.read(elem_id(PROTECTION_CONTROL))?.value[0] == 1)
        }
        fn faults(&mut self) -> Result<Vec<AmpFault>, KernelError> {
            let mut out = Vec::new();
            for id in self.list()? {
                let name = id_name(&id);
                let Some(amp) = name.strip_suffix(FAULT_CONTROL_SUFFIX) else { continue };
                let v = self.read(id)?;
                out.push(AmpFault {
                    amp: amp.trim_end().to_owned(),
                    bits: v.value[0] as u8,
                    events: v.value[1].clamp(0, i64::from(u32::MAX)) as u32,
                    group_off: v.value[2] != 0,
                });
            }
            Ok(out)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lease_is_inside_the_kernel_range_and_spans_three_ticks() {
        assert!((LEASE_MIN_MS..=LEASE_MAX_MS).contains(&LEASE_MS));
        assert_eq!(LEASE_MS, 3 * 10);
    }

    #[test]
    fn hard_faults_are_over_temperature_over_current_and_group_off() {
        let f = |bits, group_off| AmpFault { amp: "Left Tweeter".into(), bits, events: 1, group_off };
        assert!(f(AmpFault::OVER_TEMPERATURE, false).hard());
        assert!(f(AmpFault::OVER_CURRENT, false).hard());
        assert!(f(0, true).hard());
        for soft in [
            AmpFault::TDM_CLOCK,
            AmpFault::LIMITER_ACTIVE,
            AmpFault::PVDD_BELOW_INFLECTION,
            AmpFault::LIMITER_MAX_ATTENUATION,
            AmpFault::BOP_INFINITE_HOLD,
            AmpFault::BOP_MUTE,
        ] {
            assert!(!f(soft, false).hard());
        }
    }

    #[test]
    fn recorder_tracks_the_grant() {
        let mut k = Recorder::default();
        assert_eq!(k.granted(), Ok(false));
        k.renew().unwrap();
        assert_eq!(k.granted(), Ok(true));
        k.release().unwrap();
        assert_eq!(k.granted(), Ok(false));
        assert_eq!((k.renewals, k.releases), (1, 1));
    }

    #[cfg(all(target_os = "linux", target_pointer_width = "64"))]
    #[test]
    fn ioctl_numbers_match_the_kernel_uapi() {
        use super::alsa::*;
        // include/uapi/sound/asound.h on a 64-bit kernel
        assert_eq!(core::mem::size_of::<ElemId>(), 64);
        assert_eq!(core::mem::size_of::<ElemValue>(), 1224);
        assert_eq!(core::mem::size_of::<ElemList>(), 80);
        assert_eq!(ELEM_LIST, 0xc050_5510);
        assert_eq!(ELEM_READ, 0xc4c8_5512);
        assert_eq!(ELEM_WRITE, 0xc4c8_5513);
        assert_eq!(ELEM_LOCK, 0x4040_5514);
        assert_eq!(SUBSCRIBE_EVENTS, 0xc004_5516);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn control_names_fit_and_are_nul_terminated() {
        let id = super::alsa::elem_id(PROTECTION_CONTROL);
        assert_eq!(&id.name[..PROTECTION_CONTROL.len()], PROTECTION_CONTROL.as_bytes());
        assert_eq!(id.name[PROTECTION_CONTROL.len()], 0);
        assert_eq!(id.iface, super::alsa::IFACE_MIXER);
    }
}

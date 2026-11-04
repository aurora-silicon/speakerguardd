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
//! - Every amp's "<amp> Amp Gain Volume" and DVC ("<amp> Speaker Volume",
//!   "<amp> Speaker Playback Volume" on software-gain amps) are readable, with
//!   a dB TLV: the supervisor reads the operating point back (KERNEL.md 2.3).
//! - "Speaker Playback Mux" names the front-end ("Primary" or "Secondary")
//!   that drives the speakers: the supervisor watches that PCM only, so it
//!   checks the mux every tick.

/// The lease the supervisor asks for: three 10 ms ticks.
pub const LEASE_MS: u32 = 30;
/// The kernel's accepted range.
pub const LEASE_MIN_MS: u32 = 10;
pub const LEASE_MAX_MS: u32 = 100;

/// The protection control on the speaker card.
pub const PROTECTION_CONTROL: &str = "Speaker Protection";
/// The per-amp fault control's name suffix.
pub const FAULT_CONTROL_SUFFIX: &str = "Amp Fault Status";
/// The per-amp analog gain control's name suffix.
pub const GAIN_CONTROL_SUFFIX: &str = "Amp Gain Volume";
/// The mux that picks the front-end driving the speakers.
pub const SPEAKER_MUX: &str = "Speaker Playback Mux";

#[derive(Debug, PartialEq, Eq)]
pub enum KernelError {
    /// The control device is not there (yet): the card has not registered.
    /// Retryable.
    NoDevice,
    /// Opening the control device was refused (`EACCES`/`EPERM`: device
    /// policy or SELinux). Retryable, and logged.
    Denied,
    /// The card has no "Speaker Protection" control (`ENOENT` on the lock):
    /// not a v2 kernel.
    NotV2,
    /// Another file owns the speakers (`EBUSY` on the lock). Retryable: a
    /// previous owner may still be closing.
    Busy,
    /// The lock was lost (`EPERM`): take ownership again.
    NotOwner,
    /// The lease ran out before this renewal, which granted nothing.
    Lapsed,
    /// A system sleep is in progress (`EBUSY` on the write).
    Sleeping,
    /// `EIO` on a grant or release: an amp write failed (the operating gain
    /// on a grant, the floor on a release; an I2C error either way). Fatal.
    AmpIo,
    Control(String),
}

impl KernelError {
    /// Whether waiting and opening again can help.
    pub fn retryable(&self) -> bool {
        matches!(self, KernelError::NoDevice | KernelError::Denied | KernelError::Busy)
    }
}

/// One amp's level control, read back: its raw value and, from the
/// control's dB TLV, the level in hundredths of a dB (`None`: no dB scale,
/// or muted).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AmpLevel {
    /// The amp's control prefix, for example "Left Woofer 1".
    pub amp: String,
    pub value: i64,
    pub centi_db: Option<i32>,
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
    /// Every control named "<amp> <suffix>", read back with its dB level.
    fn levels(&mut self, suffix: &str) -> Result<Vec<AmpLevel>, KernelError>;
    /// The front-end "Speaker Playback Mux" selects ("Primary", "Secondary"),
    /// or `None` if the card has no such mux.
    fn speaker_route(&mut self) -> Result<Option<String>, KernelError>;
}

/// A level's raw value and centi-dB, for the double.
pub type Level = (i64, Option<i32>);

/// A kernel double for tests: records calls. [`Recorder::healthy`] builds
/// a card whose amps follow the kernel contract: floor while released, the
/// operating volume while held.
#[derive(Default, Debug)]
pub struct Recorder {
    pub held: bool,
    pub renewals: u32,
    pub releases: u32,
    pub amp_faults: Vec<AmpFault>,
    pub amps: Vec<String>,
    /// (suffix, value and dB while held, value and dB while released)
    pub volume: Option<(String, Level, Level)>,
    /// Analog gain per amp, as (value, centi-dB).
    pub gain: Option<(i64, i32)>,
    pub route: Option<String>,
}

impl Recorder {
    /// A SN012776 card: `n` amps with fault controls, gain 15 (18.5 dB),
    /// DVC 201 (0 dB) held and 0 (mute) released, the mux on "Secondary".
    pub fn healthy(n: usize) -> Self {
        let amps: Vec<String> = (0..n).map(|i| format!("Amp {i}")).collect();
        Recorder {
            amp_faults: amps
                .iter()
                .map(|a| AmpFault { amp: a.clone(), bits: 0, events: 0, group_off: false })
                .collect(),
            amps,
            volume: Some(("Speaker Volume".into(), (201, Some(0)), (0, None))),
            gain: Some((15, 1850)),
            route: Some("Secondary".into()),
            ..Default::default()
        }
    }
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
    fn levels(&mut self, suffix: &str) -> Result<Vec<AmpLevel>, KernelError> {
        let level = if suffix == GAIN_CONTROL_SUFFIX {
            self.gain.map(|(v, d)| (v, Some(d)))
        } else {
            match &self.volume {
                Some((s, on, off)) if s == suffix => Some(if self.held { *on } else { *off }),
                _ => None,
            }
        };
        Ok(level
            .map(|(value, centi_db)| self.amps.iter().map(|a| AmpLevel { amp: a.clone(), value, centi_db }).collect())
            .unwrap_or_default())
    }
    fn speaker_route(&mut self) -> Result<Option<String>, KernelError> {
        Ok(self.route.clone())
    }
}

/// The level in hundredths of a dB of `value` on a `SNDRV_CTL_TLVT_DB_SCALE`
/// TLV (`[type 1, 8, min, step | mute << 16]`), for a control whose minimum
/// is `min_value`. `None` for any other TLV, or for the muted minimum.
pub fn db_scale_centi(tlv: &[u32], value: i64, min_value: i64) -> Option<i32> {
    const DB_SCALE: u32 = 1;
    if tlv.len() < 4 || tlv[0] != DB_SCALE || tlv[1] < 8 {
        return None;
    }
    let min = tlv[2] as i32;
    let step = (tlv[3] & 0xffff) as i32;
    let mute = tlv[3] & 0x1_0000 != 0;
    let steps = value.checked_sub(min_value)?;
    if steps < 0 || (mute && steps == 0) {
        return None;
    }
    i32::try_from(steps).ok()?.checked_mul(step)?.checked_add(min)
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
    const ENXIO: i32 = 6;
    const EIO: i32 = 5;
    const EACCES: i32 = 13;
    const EBUSY: i32 = 16;
    const ENODEV: i32 = 19;
    const ETIMEDOUT: i32 = 110;
    const ELEM_TYPE_INTEGER: i32 = 2;
    const ELEM_TYPE_ENUMERATED: i32 = 3;

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

    /// `struct snd_ctl_elem_info`, with the value union as raw words.
    #[repr(C)]
    pub(super) struct ElemInfo {
        pub id: ElemId,
        pub kind: i32,
        pub access: u32,
        pub count: u32,
        pub owner: i32,
        /// integer: min, max, step (i64); enumerated: items, item (u32),
        /// name[64]
        pub value: [u64; 16],
        pub reserved: [u8; 64],
    }

    /// `struct snd_ctl_tlv` with room for a short TLV.
    #[repr(C)]
    pub(super) struct Tlv {
        pub numid: u32,
        pub length: u32,
        pub tlv: [u32; 30],
    }

    const fn ioc(dir: u64, nr: u64, size: usize) -> u64 {
        (dir << 30) | ((size as u64) << 16) | (0x55 << 8) | nr
    }
    const W: u64 = 1;
    const RW: u64 = 3;
    pub(super) const ELEM_LIST: u64 = ioc(RW, 0x10, core::mem::size_of::<ElemList>());
    pub(super) const ELEM_INFO: u64 = ioc(RW, 0x11, core::mem::size_of::<ElemInfo>());
    /// `_IOWR('U', 0x1a, struct snd_ctl_tlv)`: the header only.
    pub(super) const TLV_READ: u64 = ioc(RW, 0x1a, 8);
    pub(super) const ELEM_READ: u64 = ioc(RW, 0x12, core::mem::size_of::<ElemValue>());
    pub(super) const ELEM_WRITE: u64 = ioc(RW, 0x13, core::mem::size_of::<ElemValue>());
    pub(super) const ELEM_LOCK: u64 = ioc(W, 0x14, core::mem::size_of::<ElemId>());
    #[cfg(test)]
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

    /// How a failure to open the control device maps: a missing node is a
    /// card that has not registered yet, a refusal is policy; both retry.
    pub(super) fn open_error(errno: Option<i32>) -> KernelError {
        match errno {
            Some(ENOENT | ENODEV | ENXIO) => KernelError::NoDevice,
            Some(EACCES | EPERM) => KernelError::Denied,
            e => KernelError::Control(format!("open: errno {}", e.unwrap_or(0))),
        }
    }

    /// The speaker card's control file, owning "Speaker Protection".
    /// Dropping it closes the file, which withdraws protection at once.
    pub struct Macaudio {
        file: File,
        /// The card's control ids, listed once (the set is fixed while the
        /// file is open: a card that goes away takes the file with it).
        ids: Option<Vec<ElemId>>,
        /// The mux's id and item names, read once.
        mux: Option<Option<(ElemId, Vec<String>)>>,
    }

    impl Macaudio {
        /// Open `/dev/snd/controlC<card>` and take ownership. The event
        /// subscription is not taken: the loop reads the fault controls on
        /// every 10 ms tick, so events would add nothing.
        pub fn open(card: u32) -> Result<Self, KernelError> {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .custom_flags(O_CLOEXEC)
                .open(format!("/dev/snd/controlC{card}"))
                .map_err(|e| open_error(e.raw_os_error()))?;
            let k = Macaudio { file, ids: None, mux: None };
            let id = elem_id(PROTECTION_CONTROL);
            // SAFETY: a valid fd and a pointer to a live, correctly sized ElemId.
            if unsafe { ioctl(k.fd(), ELEM_LOCK, &id as *const ElemId) } < 0 {
                return Err(match errno() {
                    EBUSY => KernelError::Busy,
                    // The card has no "Speaker Protection": not a v2 kernel
                    ENOENT => KernelError::NotV2,
                    e => KernelError::Control(format!("lock: errno {e}")),
                });
            }
            Ok(k)
        }

        /// Open the control device without taking ownership: reads only
        /// (levels, route, faults), for the probe tool. A write fails with
        /// `EPERM`.
        pub fn open_unowned(card: u32) -> Result<Self, KernelError> {
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .custom_flags(O_CLOEXEC)
                .open(format!("/dev/snd/controlC{card}"))
                .map_err(|e| open_error(e.raw_os_error()))?;
            Ok(Macaudio { file, ids: None, mux: None })
        }

        fn ids(&mut self) -> Result<Vec<ElemId>, KernelError> {
            if self.ids.is_none() {
                self.ids = Some(self.list()?);
            }
            Ok(self.ids.clone().unwrap_or_default())
        }

        fn info(&mut self, id: ElemId, item: u32) -> Result<Box<ElemInfo>, KernelError> {
            let mut i =
                Box::new(ElemInfo { id, kind: 0, access: 0, count: 0, owner: 0, value: [0; 16], reserved: [0; 64] });
            // enumerated.item: the item whose name to return
            i.value[0] = u64::from(item) << 32;
            // SAFETY: a valid fd and a live, correctly sized ElemInfo.
            if unsafe { ioctl(self.fd(), ELEM_INFO, &mut *i as *mut ElemInfo) } < 0 {
                return Err(KernelError::Control(format!("info {}: errno {}", id_name(&id), errno())));
            }
            Ok(i)
        }

        fn tlv(&mut self, numid: u32) -> Option<Vec<u32>> {
            let mut t = Tlv { numid, length: (30 * 4) as u32, tlv: [0; 30] };
            // SAFETY: a valid fd and a live Tlv whose `length` is its payload size.
            if unsafe { ioctl(self.fd(), TLV_READ, &mut t as *mut Tlv) } < 0 {
                return None;
            }
            let words = ((t.tlv[1] as usize + 8) / 4).min(30);
            Some(t.tlv[..words].to_vec())
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
                    EIO => KernelError::AmpIo,
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
            for id in self.ids()? {
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
        fn levels(&mut self, suffix: &str) -> Result<Vec<AmpLevel>, KernelError> {
            let mut out = Vec::new();
            for id in self.ids()? {
                let name = id_name(&id);
                let Some(amp) = name.strip_suffix(suffix) else { continue };
                if !amp.is_empty() && !amp.ends_with(' ') {
                    continue;
                }
                let info = self.info(id, 0)?;
                if info.kind != ELEM_TYPE_INTEGER {
                    continue;
                }
                let min = info.value[0] as i64;
                let value = self.read(id)?.value[0];
                let centi_db = self.tlv(id.numid).and_then(|t| db_scale_centi(&t, value, min));
                out.push(AmpLevel { amp: amp.trim_end().to_owned(), value, centi_db });
            }
            Ok(out)
        }
        fn speaker_route(&mut self) -> Result<Option<String>, KernelError> {
            if self.mux.is_none() {
                let mut found = None;
                for id in self.ids()? {
                    if id_name(&id) != SPEAKER_MUX || id.iface != IFACE_MIXER {
                        continue;
                    }
                    let info = self.info(id, 0)?;
                    if info.kind != ELEM_TYPE_ENUMERATED {
                        break;
                    }
                    let items = info.value[0] as u32;
                    let mut names = Vec::new();
                    for k in 0..items.min(16) {
                        let i = self.info(id, k)?;
                        // enumerated.name: 64 bytes after items and item
                        let raw: Vec<u8> = i.value[1..9].iter().flat_map(|w| w.to_le_bytes()).collect();
                        let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
                        names.push(String::from_utf8_lossy(&raw[..end]).into_owned());
                    }
                    found = Some((id, names));
                    break;
                }
                self.mux = Some(found);
            }
            let Some(Some((id, names))) = self.mux.clone() else { return Ok(None) };
            // enumerated.item[0]: the low word of the first value
            let item = self.read(id)?.value[0] as u32;
            Ok(names.get(item as usize).cloned())
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
        assert_eq!(core::mem::size_of::<ElemInfo>(), 272);
        assert_eq!(ELEM_INFO, 0xc110_5511);
        assert_eq!(TLV_READ, 0xc008_551a);
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

    #[test]
    fn db_scale_tlv_levels() {
        // tas2764 "Amp Gain Volume": 11 dB + 0.5 dB a step
        let gain = [1, 8, 1100, 50];
        assert_eq!(db_scale_centi(&gain, 15, 0), Some(1850));
        assert_eq!(db_scale_centi(&gain, 0, 0), Some(1100));
        // tas2764 "Speaker Volume": -100.5 dB + 0.5 dB a step, mute at 0
        let dvc = [1, 8, (-10050i32) as u32, 50 | 0x1_0000];
        assert_eq!(db_scale_centi(&dvc, 201, 0), Some(0));
        assert_eq!(db_scale_centi(&dvc, 0, 0), None);
        assert_eq!(db_scale_centi(&[3, 8, 0, 0], 1, 0), None);
        assert_eq!(db_scale_centi(&gain, -1, 0), None);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn a_missing_device_retries_and_a_missing_control_does_not() {
        use super::alsa::open_error;
        assert_eq!(open_error(Some(2)), KernelError::NoDevice);
        assert_eq!(open_error(Some(19)), KernelError::NoDevice);
        assert_eq!(open_error(Some(13)), KernelError::Denied);
        assert!(open_error(Some(13)).retryable());
        assert!(KernelError::Busy.retryable());
        assert!(!KernelError::NotV2.retryable());
        assert!(!KernelError::AmpIo.retryable());
    }

    #[test]
    fn the_healthy_double_follows_the_contract() {
        let mut k = Recorder::healthy(6);
        assert_eq!(k.levels("Speaker Volume").unwrap()[0].value, 0);
        k.renew().unwrap();
        let v = k.levels("Speaker Volume").unwrap();
        assert_eq!((v.len(), v[0].value, v[0].centi_db), (6, 201, Some(0)));
        assert_eq!(k.levels(GAIN_CONTROL_SUFFIX).unwrap()[5].centi_db, Some(1850));
        assert_eq!(k.speaker_route().unwrap().as_deref(), Some("Secondary"));
        assert_eq!(k.faults().unwrap().len(), 6);
    }
}

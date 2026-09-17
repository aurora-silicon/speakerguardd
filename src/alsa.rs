// SPDX-License-Identifier: MIT
//! The ALSA kernel interface the daemon needs, spoken directly over ioctls so
//! the binary can be static: card lookup, control elements (list, read,
//! write, lock) and an interleaved capture PCM.  Layouts follow
//! `include/uapi/sound/asound.h` for 64-bit Linux.

use std::ffi::CStr;
use std::fs::{File, OpenOptions};
use std::io;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

const IOC_NRBITS: u32 = 8;
const IOC_TYPEBITS: u32 = 8;
const IOC_SIZEBITS: u32 = 14;
const IOC_NRSHIFT: u32 = 0;
const IOC_TYPESHIFT: u32 = IOC_NRSHIFT + IOC_NRBITS;
const IOC_SIZESHIFT: u32 = IOC_TYPESHIFT + IOC_TYPEBITS;
const IOC_DIRSHIFT: u32 = IOC_SIZESHIFT + IOC_SIZEBITS;
const IOC_WRITE: u32 = 1;
const IOC_READ: u32 = 2;

const fn ioc(dir: u32, ty: u8, nr: u8, size: usize) -> libc::c_ulong {
    ((dir << IOC_DIRSHIFT) | ((size as u32) << IOC_SIZESHIFT) | ((ty as u32) << IOC_TYPESHIFT) | ((nr as u32) << IOC_NRSHIFT)) as libc::c_ulong
}
const fn io(ty: u8, nr: u8) -> libc::c_ulong {
    ioc(0, ty, nr, 0)
}
const fn ior<T>(ty: u8, nr: u8) -> libc::c_ulong {
    ioc(IOC_READ, ty, nr, std::mem::size_of::<T>())
}
const fn iow<T>(ty: u8, nr: u8) -> libc::c_ulong {
    ioc(IOC_WRITE, ty, nr, std::mem::size_of::<T>())
}
const fn iowr<T>(ty: u8, nr: u8) -> libc::c_ulong {
    ioc(IOC_READ | IOC_WRITE, ty, nr, std::mem::size_of::<T>())
}

fn ioctl<T>(file: &File, req: libc::c_ulong, arg: *mut T) -> io::Result<i64> {
    // SAFETY: the request codes carry the argument size and every caller
    // passes a pointer to the matching, fully initialised structure.
    let ret = unsafe { libc::ioctl(file.as_raw_fd(), req as _, arg) };
    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(ret as i64)
    }
}

// ---- control interface ----

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ElemId {
    pub numid: u32,
    pub iface: i32,
    pub device: u32,
    pub subdevice: u32,
    pub name: [u8; 44],
    pub index: u32,
}

impl ElemId {
    pub fn name(&self) -> String {
        CStr::from_bytes_until_nul(&self.name).map(|s| s.to_string_lossy().into_owned()).unwrap_or_default()
    }
}

#[repr(C)]
struct ElemList {
    offset: u32,
    space: u32,
    used: u32,
    count: u32,
    pids: *mut ElemId,
    reserved: [u8; 50],
}

#[repr(C)]
struct CardInfo {
    card: i32,
    pad: i32,
    id: [u8; 16],
    driver: [u8; 16],
    name: [u8; 32],
    longname: [u8; 80],
    reserved: [u8; 16],
    mixername: [u8; 80],
    components: [u8; 128],
}

/// `struct snd_ctl_elem_value`: the id, then the largest member of the value
/// union (128 longs) and the reserved tail.
#[repr(C)]
struct ElemValue {
    id: ElemId,
    indirect: u32,
    value: [i64; 128],
    reserved: [u8; 128],
}

#[repr(C)]
struct ElemInfo {
    id: ElemId,
    ty: i32,
    access: u32,
    count: u32,
    owner: i32,
    value: [u8; 128],
    reserved: [u8; 64],
}

const CTL_IOCTL_CARD_INFO: libc::c_ulong = ior::<CardInfo>(b'U', 0x01);
const CTL_IOCTL_ELEM_LIST: libc::c_ulong = iowr::<ElemList>(b'U', 0x10);
const CTL_IOCTL_ELEM_INFO: libc::c_ulong = iowr::<ElemInfo>(b'U', 0x11);
const CTL_IOCTL_ELEM_READ: libc::c_ulong = iowr::<ElemValue>(b'U', 0x12);
const CTL_IOCTL_ELEM_WRITE: libc::c_ulong = iowr::<ElemValue>(b'U', 0x13);
const CTL_IOCTL_ELEM_LOCK: libc::c_ulong = iow::<ElemId>(b'U', 0x14);

/// A control device; the file descriptor is the lock owner the kernel sees.
pub struct Ctl {
    file: File,
    pub card: u32,
}

impl Ctl {
    /// Open the card whose ALSA id (e.g. "AppleJ700") matches.
    pub fn open_by_id(id: &str) -> io::Result<Ctl> {
        for card in 0..32u32 {
            let path = format!("/dev/snd/controlC{card}");
            let file = match OpenOptions::new().read(true).write(true).open(&path) {
                Ok(f) => f,
                Err(e) if e.kind() == io::ErrorKind::NotFound => continue,
                Err(e) => return Err(e),
            };
            // SAFETY: all-zero is a valid CardInfo; the kernel fills it.
            let mut info: CardInfo = unsafe { std::mem::zeroed() };
            ioctl(&file, CTL_IOCTL_CARD_INFO, &mut info)?;
            let card_id = CStr::from_bytes_until_nul(&info.id).map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
            if card_id == id {
                return Ok(Ctl { file, card });
            }
        }
        Err(io::Error::new(io::ErrorKind::NotFound, format!("no sound card with id {id}")))
    }

    /// Every element id of the card.
    pub fn elements(&self) -> io::Result<Vec<ElemId>> {
        // SAFETY: zeroed ElemList/ElemId are valid; the kernel fills them.
        let mut list: ElemList = unsafe { std::mem::zeroed() };
        ioctl(&self.file, CTL_IOCTL_ELEM_LIST, &mut list)?;
        let count = list.count as usize;
        let mut ids: Vec<ElemId> = vec![unsafe { std::mem::zeroed() }; count];
        list.offset = 0;
        list.space = count as u32;
        list.pids = ids.as_mut_ptr();
        ioctl(&self.file, CTL_IOCTL_ELEM_LIST, &mut list)?;
        ids.truncate(list.used as usize);
        Ok(ids)
    }

    /// The ids whose name matches: verbatim, or a `* suffix` pattern like
    /// the kernel's snd_soc_control_matches().
    pub fn find(&self, pattern: &str) -> io::Result<Vec<ElemId>> {
        let suffix = pattern.strip_prefix('*').map(|p| p.trim_start());
        Ok(self
            .elements()?
            .into_iter()
            .filter(|id| {
                let name = id.name();
                match suffix {
                    Some(sfx) => name.ends_with(sfx),
                    None => name == pattern,
                }
            })
            .collect())
    }

    /// One id, or NotFound.
    pub fn find_one(&self, name: &str) -> io::Result<ElemId> {
        self.find(name)?
            .into_iter()
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, format!("no control {name:?}")))
    }

    fn value_of(&self, id: &ElemId) -> io::Result<ElemValue> {
        // SAFETY: zeroed is valid; the kernel fills the value.
        let mut v: ElemValue = unsafe { std::mem::zeroed() };
        v.id = *id;
        ioctl(&self.file, CTL_IOCTL_ELEM_READ, &mut v)?;
        Ok(v)
    }

    /// Integer control: the first channel.
    pub fn read_int(&self, id: &ElemId) -> io::Result<i64> {
        Ok(self.value_of(id)?.value[0])
    }

    /// Integer control: every channel to the same value.
    pub fn write_int(&self, id: &ElemId, value: i64) -> io::Result<()> {
        let mut v = self.value_of(id)?;
        // SAFETY: zeroed is valid.
        let mut info: ElemInfo = unsafe { std::mem::zeroed() };
        info.id = *id;
        ioctl(&self.file, CTL_IOCTL_ELEM_INFO, &mut info)?;
        for ch in 0..(info.count as usize).min(128) {
            v.value[ch] = value;
        }
        ioctl(&self.file, CTL_IOCTL_ELEM_WRITE, &mut v)?;
        Ok(())
    }

    /// The integer range of a control.
    pub fn int_range(&self, id: &ElemId) -> io::Result<(i64, i64)> {
        // SAFETY: zeroed is valid.
        let mut info: ElemInfo = unsafe { std::mem::zeroed() };
        info.id = *id;
        ioctl(&self.file, CTL_IOCTL_ELEM_INFO, &mut info)?;
        // union { long min, max, step; } for integer controls
        let min = i64::from_ne_bytes(info.value[0..8].try_into().unwrap());
        let max = i64::from_ne_bytes(info.value[8..16].try_into().unwrap());
        Ok((min, max))
    }

    /// Take the element lock: only this file descriptor may write it now.
    pub fn lock(&self, id: &ElemId) -> io::Result<()> {
        let mut id = *id;
        ioctl(&self.file, CTL_IOCTL_ELEM_LOCK, &mut id)?;
        Ok(())
    }
}

// ---- PCM capture ----

#[repr(C)]
#[derive(Clone, Copy)]
struct Mask {
    bits: [u32; 8],
}

#[repr(C)]
#[derive(Clone, Copy)]
struct Interval {
    min: u32,
    max: u32,
    flags: u32, // openmin:1 openmax:1 integer:1 empty:1
}

#[repr(C)]
struct HwParams {
    flags: u32,
    masks: [Mask; 3],
    mres: [Mask; 5],
    intervals: [Interval; 12],
    ires: [Interval; 9],
    rmask: u32,
    cmask: u32,
    info: u32,
    msbits: u32,
    rate_num: u32,
    rate_den: u32,
    fifo_size: u64,
    reserved: [u8; 64],
}

#[repr(C)]
struct SwParams {
    tstamp_mode: i32,
    period_step: u32,
    sleep_min: u32,
    avail_min: u64,
    xfer_align: u64,
    start_threshold: u64,
    stop_threshold: u64,
    silence_threshold: u64,
    silence_size: u64,
    boundary: u64,
    proto: u32,
    tstamp_type: u32,
    reserved: [u8; 56],
}

#[repr(C)]
struct Xferi {
    result: i64,
    buf: *mut u8,
    frames: u64,
}

const PARAM_ACCESS: usize = 0;
const PARAM_FORMAT: usize = 1;
const PARAM_SUBFORMAT: usize = 2;
const PARAM_CHANNELS: usize = 10 - 8;
const PARAM_RATE: usize = 11 - 8;
const PARAM_PERIOD_SIZE: usize = 13 - 8;
const PARAM_PERIODS: usize = 15 - 8;
const ACCESS_RW_INTERLEAVED: u32 = 3;
const FORMAT_S32_LE: u32 = 10;
const SUBFORMAT_STD: u32 = 0;
const INTERVAL_INTEGER: u32 = 1 << 2;

const PCM_IOCTL_HW_PARAMS: libc::c_ulong = iowr::<HwParams>(b'A', 0x11);
const PCM_IOCTL_SW_PARAMS: libc::c_ulong = iowr::<SwParams>(b'A', 0x13);
const PCM_IOCTL_PREPARE: libc::c_ulong = io(b'A', 0x40);
const PCM_IOCTL_START: libc::c_ulong = io(b'A', 0x42);
const PCM_IOCTL_DROP: libc::c_ulong = io(b'A', 0x43);
const PCM_IOCTL_READI_FRAMES: libc::c_ulong = ior::<Xferi>(b'A', 0x51);

/// An interleaved S32_LE capture stream with fixed period geometry.
pub struct Capture {
    file: File,
    pub channels: usize,
    pub period: usize,
    started: bool,
}

impl Capture {
    pub fn open(card: u32, device: u32, channels: usize, rate: u32, period: usize) -> io::Result<Capture> {
        let path = format!("/dev/snd/pcmC{card}D{device}c");
        // A blocking open would wait, silently, for whoever holds the sense
        // PCM to let go, and the volume lock would time out meanwhile: ask
        // for EBUSY instead and wait for frames with poll().
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(&path)?;
        // SAFETY: an all-ones/zeroed parameter block is the "unconstrained"
        // request the kernel refines.
        let mut hw: HwParams = unsafe { std::mem::zeroed() };
        for m in hw.masks.iter_mut() {
            m.bits = [0; 8];
        }
        for i in hw.intervals.iter_mut() {
            *i = Interval { min: 0, max: u32::MAX, flags: 0 };
        }
        hw.masks[PARAM_ACCESS].bits[0] = 1 << ACCESS_RW_INTERLEAVED;
        hw.masks[PARAM_FORMAT].bits[0] = 1 << FORMAT_S32_LE;
        hw.masks[PARAM_SUBFORMAT].bits[0] = 1 << SUBFORMAT_STD;
        let fixed = |i: &mut Interval, v: u32| {
            *i = Interval { min: v, max: v, flags: INTERVAL_INTEGER };
        };
        fixed(&mut hw.intervals[PARAM_CHANNELS], channels as u32);
        fixed(&mut hw.intervals[PARAM_RATE], rate);
        fixed(&mut hw.intervals[PARAM_PERIOD_SIZE], period as u32);
        fixed(&mut hw.intervals[PARAM_PERIODS], 4);
        hw.rmask = u32::MAX;
        ioctl(&file, PCM_IOCTL_HW_PARAMS, &mut hw)?;

        // SAFETY: zeroed is a valid base; every field is set below.
        let mut sw: SwParams = unsafe { std::mem::zeroed() };
        sw.tstamp_mode = 0;
        sw.period_step = 1;
        sw.avail_min = period as u64;
        sw.xfer_align = 1;
        sw.start_threshold = 1;
        sw.stop_threshold = (period * 4) as u64;
        sw.silence_threshold = 0;
        sw.silence_size = 0;
        sw.boundary = (period * 4) as u64 * (1 << 20);
        ioctl(&file, PCM_IOCTL_SW_PARAMS, &mut sw)?;
        ioctl(&file, PCM_IOCTL_PREPARE, std::ptr::null_mut::<u8>())?;
        Ok(Capture { file, channels, period, started: false })
    }

    /// One period of frames into `buf` (period * channels words).  An overrun
    /// is recovered transparently (the stream is re-prepared) and reported as
    /// Ok(false) so the caller can discard the model step.
    pub fn read_period(&mut self, buf: &mut [i32]) -> io::Result<bool> {
        assert!(buf.len() >= self.period * self.channels);
        if !self.started {
            ioctl(&self.file, PCM_IOCTL_START, std::ptr::null_mut::<u8>())?;
            self.started = true;
        }
        let mut xfer = Xferi { result: 0, buf: buf.as_mut_ptr().cast(), frames: self.period as u64 };
        loop {
            match ioctl(&self.file, PCM_IOCTL_READI_FRAMES, &mut xfer) {
                Ok(_) => return Ok(true),
                Err(e) if e.raw_os_error() == Some(libc::EAGAIN) => self.wait_frames()?,
                Err(e) if e.raw_os_error() == Some(libc::EPIPE) => {
                    ioctl(&self.file, PCM_IOCTL_PREPARE, std::ptr::null_mut::<u8>())?;
                    self.started = false;
                    return Ok(false);
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Waits for the next period, bounded so that a stalled stream surfaces
    /// as ETIMEDOUT rather than a hang.
    fn wait_frames(&self) -> io::Result<()> {
        let mut pfd = libc::pollfd { fd: self.file.as_raw_fd(), events: libc::POLLIN, revents: 0 };
        // SAFETY: one initialised pollfd, count 1.
        let ret = unsafe { libc::poll(&mut pfd, 1, PERIOD_WAIT_MS) };
        if ret < 0 {
            return Err(io::Error::last_os_error());
        }
        if ret == 0 {
            return Err(io::Error::from_raw_os_error(libc::ETIMEDOUT));
        }
        Ok(())
    }
}

/// Longest wait for one sense period: several periods at the lowest rate.
const PERIOD_WAIT_MS: libc::c_int = 500;

impl Drop for Capture {
    fn drop(&mut self) {
        let _ = ioctl(&self.file, PCM_IOCTL_DROP, std::ptr::null_mut::<u8>());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layouts_match_asound_h() {
        assert_eq!(std::mem::size_of::<ElemId>(), 64);
        assert_eq!(std::mem::size_of::<ElemList>(), 80);
        assert_eq!(std::mem::size_of::<ElemInfo>(), 272);
        assert_eq!(std::mem::size_of::<ElemValue>(), 1224);
        assert_eq!(std::mem::size_of::<CardInfo>(), 376);
        assert_eq!(std::mem::size_of::<HwParams>(), 608);
        assert_eq!(std::mem::size_of::<SwParams>(), 136);
        assert_eq!(std::mem::size_of::<Xferi>(), 24);
    }

    #[test]
    fn ioctl_numbers() {
        // known values from the kernel headers on 64-bit Linux
        assert_eq!(CTL_IOCTL_ELEM_LOCK, 0x4040_5514);
        assert_eq!(CTL_IOCTL_ELEM_READ, 0xc4c8_5512);
        assert_eq!(PCM_IOCTL_HW_PARAMS, 0xc260_4111);
        assert_eq!(PCM_IOCTL_READI_FRAMES, 0x8018_4151);
        assert_eq!(PCM_IOCTL_START, 0x4142);
    }
}

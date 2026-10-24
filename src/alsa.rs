// SPDX-License-Identifier: MIT
//! The ALSA kernel interface the daemon needs, spoken directly over ioctls so
//! the binary can be static: card lookup, control elements (list, read,
//! write, lock) and an interleaved capture PCM.  Layouts follow
//! `include/uapi/sound/asound.h` for 64-bit Linux.

use std::ffi::CStr;
use std::fs::{File, OpenOptions};
use std::io;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::{AsRawFd, RawFd};

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
const CTL_IOCTL_ELEM_UNLOCK: libc::c_ulong = iow::<ElemId>(b'U', 0x15);
const CTL_IOCTL_SUBSCRIBE_EVENTS: libc::c_ulong = iowr::<libc::c_int>(b'U', 0x16);

/// `struct snd_ctl_event` for SNDRV_CTL_EVENT_ELEM: the mask and the id.
#[repr(C)]
#[derive(Clone, Copy)]
struct CtlEvent {
    ty: i32,
    mask: u32,
    id: ElemId,
}
const CTL_EVENT_ELEM: i32 = 0;

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

    /// Give the element lock up.
    pub fn unlock(&self, id: &ElemId) -> io::Result<()> {
        let mut id = *id;
        ioctl(&self.file, CTL_IOCTL_ELEM_UNLOCK, &mut id)?;
        Ok(())
    }

    /// Ask for change events on this file, and make it non-blocking so
    /// that `events` can drain them.
    pub fn subscribe(&self) -> io::Result<()> {
        let mut on: libc::c_int = 1;
        ioctl(&self.file, CTL_IOCTL_SUBSCRIBE_EVENTS, &mut on)?;
        // SAFETY: plain fcntl calls on an open descriptor.
        unsafe {
            let fl = libc::fcntl(self.file.as_raw_fd(), libc::F_GETFL);
            if fl < 0 || libc::fcntl(self.file.as_raw_fd(), libc::F_SETFL, fl | libc::O_NONBLOCK) < 0 {
                return Err(io::Error::last_os_error());
            }
        }
        Ok(())
    }

    /// Drain the queued events; true if one was about `numid`.
    pub fn events(&self, numid: u32) -> io::Result<bool> {
        let mut hit = false;
        // SAFETY: all-zero events are valid; read() fills whole ones.
        let mut buf: [CtlEvent; 32] = unsafe { std::mem::zeroed() };
        loop {
            // SAFETY: the buffer is writable for its full size.
            let n = unsafe { libc::read(self.file.as_raw_fd(), buf.as_mut_ptr().cast(), std::mem::size_of_val(&buf)) };
            if n < 0 {
                let e = io::Error::last_os_error();
                return match e.raw_os_error() {
                    Some(libc::EAGAIN) => Ok(hit),
                    Some(libc::EINTR) => continue,
                    _ => Err(e),
                };
            }
            let count = n as usize / std::mem::size_of::<CtlEvent>();
            hit |= buf[..count].iter().any(|ev| ev.ty == CTL_EVENT_ELEM && ev.id.numid == numid);
            if count < buf.len() {
                return Ok(hit);
            }
        }
    }

    pub fn fd(&self) -> RawFd {
        self.file.as_raw_fd()
    }
}

/// poll() two descriptors for input for at most `timeout_ms`: whether each
/// is readable.  An interrupted wait reports neither.
pub fn poll2(a: RawFd, b: Option<RawFd>, timeout_ms: libc::c_int) -> io::Result<(bool, bool)> {
    let mut fds = [
        libc::pollfd { fd: a, events: libc::POLLIN, revents: 0 },
        libc::pollfd { fd: b.unwrap_or(-1), events: libc::POLLIN, revents: 0 },
    ];
    let n = if b.is_some() { 2 } else { 1 };
    // SAFETY: n initialised pollfds.
    let ret = unsafe { libc::poll(fds.as_mut_ptr(), n, timeout_ms) };
    if ret < 0 {
        let e = io::Error::last_os_error();
        if e.raw_os_error() == Some(libc::EINTR) {
            return Ok((false, false));
        }
        return Err(e);
    }
    Ok((fds[0].revents != 0, n == 2 && fds[1].revents != 0))
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
const FORMAT_S16_LE: u32 = 2;
const FORMAT_S32_LE: u32 = 10;
const SUBFORMAT_STD: u32 = 0;
const INTERVAL_INTEGER: u32 = 1 << 2;

const PCM_IOCTL_HW_PARAMS: libc::c_ulong = iowr::<HwParams>(b'A', 0x11);
const PCM_IOCTL_SW_PARAMS: libc::c_ulong = iowr::<SwParams>(b'A', 0x13);
const PCM_IOCTL_PREPARE: libc::c_ulong = io(b'A', 0x40);
const PCM_IOCTL_START: libc::c_ulong = io(b'A', 0x42);
const PCM_IOCTL_DROP: libc::c_ulong = io(b'A', 0x43);
const PCM_IOCTL_READI_FRAMES: libc::c_ulong = ior::<Xferi>(b'A', 0x51);
const PCM_IOCTL_DELAY: libc::c_ulong = ior::<libc::c_long>(b'A', 0x21);

/// The four calls the period reader makes on a capture PCM, so that its
/// read discipline can be checked against a simulated kernel.
pub trait PcmIo {
    fn start(&mut self) -> io::Result<()>;
    fn prepare(&mut self) -> io::Result<()>;
    /// READI_FRAMES on the non-blocking file: the frames transferred, which
    /// may be fewer than asked for; EAGAIN when none are available.
    fn readi(&mut self, buf: &mut [i32], frames: usize) -> io::Result<i64>;
    /// poll(POLLIN) for at most `timeout_ms`.  True when the kernel reports
    /// the stream readable (a whole period available, or an error state).
    fn wait(&mut self, timeout_ms: libc::c_int) -> io::Result<bool>;
    /// DELAY: for a capture, the frames captured and not yet read (the
    /// kernel syncs the hardware pointer first).
    fn delay(&mut self) -> io::Result<i64>;
}

/// The capture PCM file itself.  An S16 stream is read into `scratch` and
/// widened (<< 16), so every reader sees S32 full scale.
pub struct PcmFile {
    file: File,
    s16: bool,
    scratch: Vec<i16>,
}

impl PcmIo for PcmFile {
    fn start(&mut self) -> io::Result<()> {
        ioctl(&self.file, PCM_IOCTL_START, std::ptr::null_mut::<u8>()).map(|_| ())
    }

    fn prepare(&mut self) -> io::Result<()> {
        ioctl(&self.file, PCM_IOCTL_PREPARE, std::ptr::null_mut::<u8>()).map(|_| ())
    }

    fn readi(&mut self, buf: &mut [i32], frames: usize) -> io::Result<i64> {
        if !self.s16 {
            let mut xfer = Xferi { result: 0, buf: buf.as_mut_ptr().cast(), frames: frames as u64 };
            ioctl(&self.file, PCM_IOCTL_READI_FRAMES, &mut xfer)?;
            return Ok(xfer.result);
        }
        let channels = if frames == 0 { 0 } else { buf.len() / frames };
        self.scratch.resize(frames * channels, 0);
        let mut xfer = Xferi { result: 0, buf: self.scratch.as_mut_ptr().cast(), frames: frames as u64 };
        ioctl(&self.file, PCM_IOCTL_READI_FRAMES, &mut xfer)?;
        let got = (xfer.result.max(0) as usize).min(frames) * channels;
        widen(&self.scratch[..got], &mut buf[..got]);
        Ok(xfer.result)
    }

    fn wait(&mut self, timeout_ms: libc::c_int) -> io::Result<bool> {
        let mut pfd = libc::pollfd { fd: self.file.as_raw_fd(), events: libc::POLLIN, revents: 0 };
        // SAFETY: one initialised pollfd, count 1.
        let ret = unsafe { libc::poll(&mut pfd, 1, timeout_ms) };
        if ret < 0 {
            let e = io::Error::last_os_error();
            if e.raw_os_error() == Some(libc::EINTR) {
                return Ok(false);
            }
            return Err(e);
        }
        Ok(ret != 0)
    }

    fn delay(&mut self) -> io::Result<i64> {
        let mut frames: libc::c_long = 0;
        ioctl(&self.file, PCM_IOCTL_DELAY, &mut frames)?;
        Ok(frames as i64)
    }
}

/// S16 sense words as S32 full scale.
pub fn widen(src: &[i16], dst: &mut [i32]) {
    for (d, &x) in dst.iter_mut().zip(src) {
        *d = (x as i32) << 16;
    }
}

impl Drop for PcmFile {
    fn drop(&mut self) {
        let _ = ioctl(&self.file, PCM_IOCTL_DROP, std::ptr::null_mut::<u8>());
    }
}

/// Reads whole sense periods from a non-blocking capture.
///
/// A READI on a non-blocking PCM transfers whatever the DMA has written so
/// far, and the kernel refreshes the hardware pointer for it, so with a
/// fine-grained pointer (the J700's ADMAC reports its residue) a read made
/// before a whole period is there comes back short.  The reader therefore
/// never reads on spec: it waits until poll() reports the stream readable,
/// which for a running capture means at least `avail_min`, one period, is
/// available, and only then reads a period.  Reading first and polling
/// second takes whatever arrived during the last 50 ms wait, keeps the
/// buffer below a period for good, and no read is ever complete again.
pub struct PeriodReader<P: PcmIo> {
    io: P,
    pub channels: usize,
    pub period: usize,
    started: bool,
}

/// An interleaved S32_LE (or S16_LE, widened) capture stream with fixed
/// period geometry.
pub type Capture = PeriodReader<PcmFile>;

impl Capture {
    pub fn open(card: u32, device: u32, channels: usize, rate: u32, period: usize, s16: bool) -> io::Result<Capture> {
        let path = format!("/dev/snd/pcmC{card}D{device}c");
        // A blocking open would wait, silently, for whoever holds the sense
        // PCM to let go, and the volume lock would time out meanwhile: ask
        // for EBUSY instead and wait for frames with poll().
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(&path)?;
        // The hardware period is the sense period, or the largest even
        // fraction of it the device takes (the SN012776 machines' 12-channel
        // sense allows 64 KiB per period, less than one 4096-frame S16
        // period).  The buffer always holds four sense periods, and the
        // reader waits for a whole sense period either way (avail_min).
        let mut split = 1;
        loop {
            // SAFETY: an all-ones/zeroed parameter block is the
            // "unconstrained" request the kernel refines.
            let mut hw: HwParams = unsafe { std::mem::zeroed() };
            for m in hw.masks.iter_mut() {
                m.bits = [0; 8];
            }
            for i in hw.intervals.iter_mut() {
                *i = Interval { min: 0, max: u32::MAX, flags: 0 };
            }
            hw.masks[PARAM_ACCESS].bits[0] = 1 << ACCESS_RW_INTERLEAVED;
            hw.masks[PARAM_FORMAT].bits[0] = 1 << if s16 { FORMAT_S16_LE } else { FORMAT_S32_LE };
            hw.masks[PARAM_SUBFORMAT].bits[0] = 1 << SUBFORMAT_STD;
            let fixed = |i: &mut Interval, v: u32| {
                *i = Interval { min: v, max: v, flags: INTERVAL_INTEGER };
            };
            fixed(&mut hw.intervals[PARAM_CHANNELS], channels as u32);
            fixed(&mut hw.intervals[PARAM_RATE], rate);
            fixed(&mut hw.intervals[PARAM_PERIOD_SIZE], (period / split) as u32);
            fixed(&mut hw.intervals[PARAM_PERIODS], (4 * split) as u32);
            hw.rmask = u32::MAX;
            match ioctl(&file, PCM_IOCTL_HW_PARAMS, &mut hw) {
                Ok(_) => break,
                Err(e) if e.raw_os_error() == Some(libc::EINVAL) && split < 16 && period % (split * 2) == 0 => split *= 2,
                Err(e) => return Err(e),
            }
        }

        // SAFETY: zeroed is a valid base; every field is set below.
        let mut sw: SwParams = unsafe { std::mem::zeroed() };
        sw.tstamp_mode = 0;
        sw.period_step = 1;
        // poll() reports the stream readable once a whole period is there;
        // the reader relies on it (see PeriodReader).
        sw.avail_min = period as u64;
        sw.xfer_align = 1;
        sw.start_threshold = 1;
        sw.stop_threshold = (period * 4) as u64;
        sw.silence_threshold = 0;
        sw.silence_size = 0;
        sw.boundary = (period * 4) as u64 * (1 << 20);
        ioctl(&file, PCM_IOCTL_SW_PARAMS, &mut sw)?;
        let mut io = PcmFile { file, s16, scratch: Vec::new() };
        io.prepare()?;
        Ok(PeriodReader::new(io, channels, period))
    }
}

impl<P: PcmIo> PeriodReader<P> {
    pub fn new(io: P, channels: usize, period: usize) -> PeriodReader<P> {
        PeriodReader { io, channels, period, started: false }
    }

    #[cfg(test)]
    pub fn io(&mut self) -> &mut P {
        &mut self.io
    }

    /// The frames the capture holds beyond what has been read: how long
    /// ago the period just read ended.
    pub fn queued(&mut self) -> io::Result<u64> {
        Ok(self.io.delay()?.max(0) as u64)
    }

    /// Start the capture if it is not running yet.
    pub fn ensure_started(&mut self) -> io::Result<()> {
        if !self.started {
            self.io.start()?;
            self.started = true;
        }
        Ok(())
    }

    /// Reads one sense period into `buf` (period * channels words).
    /// `Ok(Some(true))` is a valid period, `Ok(Some(false))` a short
    /// transfer or a recovered overrun (discarded; the stream is
    /// re-prepared after an overrun), `Ok(None)` no whole period within one
    /// ping interval: the caller must ping the lock and call again, because
    /// the card locks the volume 250 ms after the last ping and nothing may
    /// block longer than that.
    pub fn read_period(&mut self, buf: &mut [i32]) -> io::Result<Option<bool>> {
        self.read_period_within(buf, PERIOD_WAIT_MS)
    }

    /// `read_period` waiting at most `timeout_ms`.
    pub fn read_period_within(&mut self, buf: &mut [i32], timeout_ms: libc::c_int) -> io::Result<Option<bool>> {
        assert!(buf.len() >= self.period * self.channels);
        self.ensure_started()?;
        // Wait first: a read is only issued once a whole period is there.
        if !self.io.wait(timeout_ms)? {
            return Ok(None);
        }
        self.read_ready(buf)
    }

    /// The read half of `read_period`, once the stream is readable.
    pub fn read_ready(&mut self, buf: &mut [i32]) -> io::Result<Option<bool>> {
        match self.io.readi(buf, self.period) {
            Ok(frames) => Ok(Some(full_period(frames, self.period)?)),
            // readable but empty: a spurious wakeup
            Err(e) if e.raw_os_error() == Some(libc::EAGAIN) => Ok(None),
            Err(e) if e.raw_os_error() == Some(libc::EPIPE) => {
                self.io.prepare()?;
                self.started = false;
                Ok(Some(false))
            }
            Err(e) => Err(e),
        }
    }

    /// Whatever the capture holds now, up to a period, without waiting:
    /// the frames a stream that is closing left after its last whole
    /// period.  0 when there are none or the stream cannot give them.
    pub fn read_tail(&mut self, buf: &mut [i32]) -> io::Result<usize> {
        assert!(buf.len() >= self.period * self.channels);
        if !self.started {
            return Ok(0);
        }
        match self.io.readi(buf, self.period) {
            Ok(frames) if frames >= 0 => Ok((frames as usize).min(self.period)),
            Ok(frames) => Err(io::Error::from_raw_os_error(frames.saturating_neg().min(i32::MAX as i64) as i32)),
            Err(e) if matches!(e.raw_os_error(), Some(libc::EAGAIN | libc::EPIPE | libc::EBADFD | libc::ESTRPIPE | libc::ENODEV)) => Ok(0),
            Err(e) => Err(e),
        }
    }
}

impl Capture {
    pub fn fd(&self) -> RawFd {
        self.io.file.as_raw_fd()
    }
}

/// Longest single wait for sense data: well inside the card's 250 ms lock
/// timeout, so the lock ping between waits is never late.
pub const PERIOD_WAIT_MS: libc::c_int = 50;

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
    fn s16_sense_words_widen_to_s32_full_scale() {
        let mut out = [0i32; 3];
        widen(&[i16::MIN, -1, i16::MAX], &mut out);
        assert_eq!(out, [i32::MIN, -65536, 0x7fff0000]);
    }

    #[test]
    fn ioctl_numbers() {
        // known values from the kernel headers on 64-bit Linux
        assert_eq!(CTL_IOCTL_ELEM_LOCK, 0x4040_5514);
        assert_eq!(CTL_IOCTL_ELEM_READ, 0xc4c8_5512);
        assert_eq!(PCM_IOCTL_HW_PARAMS, 0xc260_4111);
        assert_eq!(PCM_IOCTL_READI_FRAMES, 0x8018_4151);
        assert_eq!(PCM_IOCTL_START, 0x4142);
        assert_eq!(PCM_IOCTL_DELAY, 0x8008_4121);
        assert_eq!(CTL_IOCTL_SUBSCRIBE_EVENTS, 0xc004_5516);
        assert_eq!(CTL_IOCTL_ELEM_UNLOCK, 0x4040_5515);
        assert_eq!(std::mem::size_of::<CtlEvent>(), 72);
    }
}

/// A successful ioctl reports the actual count separately from its return code.
fn full_period(frames: i64, requested: usize) -> io::Result<bool> {
    if frames < 0 {
        return Err(io::Error::from_raw_os_error(
            frames.saturating_neg().min(i32::MAX as i64) as i32,
        ));
    }
    if frames as u64 > requested as u64 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "sense transfer exceeds requested period",
        ));
    }
    Ok(frames as u64 == requested as u64 && requested != 0)
}

#[cfg(test)]
mod transfer_tests {
    use super::*;
    #[test]
    fn only_a_complete_positive_transfer_is_valid() {
        for frames in [0, 1, 4095] {
            assert!(!full_period(frames, 4096).unwrap());
        }
        assert!(full_period(4096, 4096).unwrap());
        assert!(!full_period(0, 0).unwrap());
        assert!(full_period(4097, 4096).is_err());
        assert_eq!(
            full_period(-libc::EPIPE as i64, 4096)
                .unwrap_err()
                .raw_os_error(),
            Some(libc::EPIPE)
        );
    }
}

#[cfg(test)]
pub mod sim {
    //! A capture PCM as the ALSA core runs one, on a virtual clock: the DMA
    //! writes frames continuously (its pointer is `granule` frames fine), a
    //! period interrupt every `period` frames refreshes the pointer that
    //! poll() looks at, READI refreshes it itself and transfers what is
    //! there, and `stop` unread frames are an overrun.
    use super::*;
    use std::rc::Rc;
    use std::time::Duration;

    const STEP: Duration = Duration::from_micros(500);

    #[derive(Clone, Copy, PartialEq, Debug)]
    enum State {
        Prepared,
        Running,
        Xrun,
    }

    pub struct SimPcm {
        pub t: Duration,
        rate: f64,
        period: u64,
        stop: u64,
        granule: u64,
        /// Fraction of the nominal rate the DMA delivers at time t (0 = stalled).
        flow: Rc<dyn Fn(Duration) -> f64>,
        state: State,
        written: f64,
        hw: u64,
        irq_hw: u64,
        next_irq: u64,
        appl: u64,
        channels: usize,
        /// The sample word the next transfer carries.
        pub word: i32,
        pub short_reads: u32,
        pub overruns: u32,
    }

    impl SimPcm {
        pub fn new(rate: u32, channels: usize, period: usize, granule: usize, flow: Rc<dyn Fn(Duration) -> f64>, t: Duration) -> SimPcm {
            let mut s = SimPcm {
                t,
                rate: rate as f64,
                period: period as u64,
                stop: 4 * period as u64,
                granule: granule.max(1) as u64,
                flow,
                state: State::Prepared,
                written: 0.0,
                hw: 0,
                irq_hw: 0,
                next_irq: 0,
                appl: 0,
                channels,
                word: 0,
                short_reads: 0,
                overruns: 0,
            };
            s.reset();
            s
        }

        fn reset(&mut self) {
            self.state = State::Prepared;
            self.written = 0.0;
            self.hw = 0;
            self.irq_hw = 0;
            self.appl = 0;
            self.next_irq = self.period;
        }

        /// Runs the DMA up to `t`.
        pub fn advance_to(&mut self, t: Duration) {
            while self.t < t {
                let dt = STEP.min(t - self.t);
                if self.state == State::Running {
                    self.written += self.rate * (self.flow)(self.t).clamp(0.0, 1.0) * dt.as_secs_f64();
                    let hw = (self.written as u64 / self.granule) * self.granule;
                    self.hw = hw;
                    while self.hw >= self.next_irq {
                        // the period interrupt: snd_pcm_period_elapsed()
                        self.irq_hw = self.hw;
                        self.next_irq += self.period;
                    }
                    if self.hw - self.appl >= self.stop {
                        self.state = State::Xrun;
                        self.overruns += 1;
                    }
                }
                self.t += dt;
            }
        }
    }

    impl PcmIo for SimPcm {
        fn start(&mut self) -> io::Result<()> {
            if self.state != State::Prepared {
                return Err(io::Error::from_raw_os_error(libc::EBADFD));
            }
            self.state = State::Running;
            Ok(())
        }
        fn prepare(&mut self) -> io::Result<()> {
            self.reset();
            Ok(())
        }
        fn readi(&mut self, buf: &mut [i32], frames: usize) -> io::Result<i64> {
            match self.state {
                State::Xrun => return Err(io::Error::from_raw_os_error(libc::EPIPE)),
                State::Prepared => return Err(io::Error::from_raw_os_error(libc::EAGAIN)),
                State::Running => {}
            }
            // __snd_pcm_lib_xfer: hwsync, then whatever is available
            self.irq_hw = self.hw;
            let avail = self.hw - self.appl;
            if avail == 0 {
                return Err(io::Error::from_raw_os_error(libc::EAGAIN));
            }
            let n = avail.min(frames as u64);
            for (i, w) in buf[..n as usize * self.channels].iter_mut().enumerate() {
                let frame = (self.appl + (i / self.channels) as u64) % 2;
                *w = if frame == 0 { self.word } else { -self.word };
            }
            self.appl += n;
            if n < frames as u64 {
                self.short_reads += 1;
            }
            Ok(n as i64)
        }
        fn delay(&mut self) -> io::Result<i64> {
            match self.state {
                State::Xrun => Err(io::Error::from_raw_os_error(libc::EPIPE)),
                // DELAY syncs the pointer, as a read does
                _ => {
                    self.irq_hw = self.hw;
                    Ok((self.hw - self.appl) as i64)
                }
            }
        }
        fn wait(&mut self, timeout_ms: libc::c_int) -> io::Result<bool> {
            let deadline = self.t + Duration::from_millis(timeout_ms as u64);
            loop {
                // snd_pcm_poll: the pointer as of the last interrupt or read
                let ready = match self.state {
                    State::Running | State::Prepared => self.irq_hw - self.appl >= self.period,
                    State::Xrun => true,
                };
                if ready {
                    return Ok(true);
                }
                if self.t >= deadline {
                    return Ok(false);
                }
                let next = (self.t + STEP).min(deadline);
                self.advance_to(next);
            }
        }
    }
}

#[cfg(test)]
mod reader_tests {
    use super::sim::SimPcm;
    use super::*;
    use std::rc::Rc;
    use std::time::Duration;

    const P: usize = 4096;

    fn reader(granule: usize, flow: Rc<dyn Fn(Duration) -> f64>) -> PeriodReader<SimPcm> {
        PeriodReader::new(SimPcm::new(48000, 2, P, granule, flow, Duration::ZERO), 2, P)
    }

    /// (complete, short or overrun, empty waits) over `secs` of reading,
    /// with `work` of processing between reads.
    fn run(r: &mut PeriodReader<SimPcm>, secs: f64, work: Duration) -> (u32, u32, u32) {
        let mut buf = vec![0i32; P * 2];
        let (mut ok, mut bad, mut none) = (0, 0, 0);
        while r.io().t < Duration::from_secs_f64(secs) {
            match r.read_period(&mut buf).unwrap() {
                Some(true) => ok += 1,
                Some(false) => bad += 1,
                None => none += 1,
            }
            let t = r.io().t + work;
            r.io().advance_to(t);
        }
        (ok, bad, none)
    }

    /// The J700 sense PCM reports a DMA-precise pointer.  Every period must
    /// come back whole, at the stream's own rate.
    #[test]
    fn a_fine_grained_pointer_gives_only_complete_periods() {
        for granule in [1, 32, 480, 1024, P] {
            let mut r = reader(granule, Rc::new(|_| 1.0));
            let (ok, bad, _) = run(&mut r, 10.0, Duration::from_millis(2));
            assert_eq!(bad, 0, "granule {granule}");
            assert_eq!(r.io().short_reads, 0, "granule {granule}");
            // 10 s at 48 kHz is 117 periods
            assert!((116..=118).contains(&ok), "granule {granule}: {ok} periods");
        }
    }

    /// The PR #2 regression, reproduced: reading before waiting takes what
    /// the last 50 ms wait let in, short of a period, and from then on the
    /// buffer never holds a whole one again, although sense data keeps
    /// flowing at the full rate.  This is the read discipline the reader
    /// must not have.
    #[test]
    fn reading_before_waiting_never_completes_a_period() {
        let mut pcm = SimPcm::new(48000, 2, P, 32, Rc::new(|_| 1.0), Duration::ZERO);
        pcm.start().unwrap();
        let mut buf = vec![0i32; P * 2];
        let (mut ok, mut short) = (0, 0);
        while pcm.t < Duration::from_secs(5) {
            match pcm.readi(&mut buf, P) {
                Ok(n) if n as usize == P => ok += 1,
                Ok(_) => short += 1,
                Err(e) if e.raw_os_error() == Some(libc::EAGAIN) => {
                    pcm.wait(PERIOD_WAIT_MS).unwrap();
                }
                Err(e) => panic!("{e}"),
            }
        }
        assert_eq!(ok, 0);
        assert!(short > 50, "{short}");
    }

    /// A daemon that falls a whole buffer behind gets one overrun, reported
    /// as an incomplete period, and complete periods right after it.
    #[test]
    fn an_overrun_is_reported_once_and_the_stream_resumes() {
        let mut r = reader(32, Rc::new(|_| 1.0));
        let (ok, bad, _) = run(&mut r, 1.0, Duration::ZERO);
        assert!(ok >= 10 && bad == 0);
        let t = r.io().t + Duration::from_millis(400); // > 4 periods (341 ms)
        r.io().advance_to(t);
        let mut buf = vec![0i32; P * 2];
        assert_eq!(r.read_period(&mut buf).unwrap(), Some(false));
        assert_eq!(r.io().overruns, 1);
        let from = r.io().t;
        let (ok, bad, _) = run(&mut r, 3.0, Duration::ZERO);
        assert_eq!(bad, 0);
        let due = ((Duration::from_secs(3) - from).as_secs_f64() * 48000.0 / P as f64) as u32;
        assert!(ok + 1 >= due, "{ok} of {due} periods");
    }

    /// No data, no period: a stalled DMA only ever gives empty waits, and a
    /// stream at a tenth of its rate gives a period a tenth as often.
    #[test]
    fn a_stalled_or_slow_stream_gives_no_extra_periods() {
        let mut r = reader(32, Rc::new(|_| 0.0));
        assert_eq!(run(&mut r, 2.0, Duration::ZERO), (0, 0, 40));
        let mut r = reader(32, Rc::new(|_| 0.1));
        let (ok, bad, _) = run(&mut r, 10.0, Duration::ZERO);
        assert_eq!(bad, 0);
        assert!((11..=12).contains(&ok), "{ok}");
    }
}

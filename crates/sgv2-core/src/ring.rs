//! Lock-free single-producer single-consumer ring of fixed-size `Copy`
//! records, laid out `#[repr(C)]` so it can live in a shared mapping (the
//! heartbeat file the supervisor creates and the plugin maps) as well as in
//! process memory.
//!
//! The producer is the RT thread: `push` is two relaxed/acquire loads, one
//! `memcpy` of the record and one release store, and never blocks. When the
//! consumer falls behind, the oldest records are *overwritten* rather than
//! the producer waiting: telemetry is lossy by design, and the consumer can
//! see exactly how many it missed from the sequence numbers.
//!
//! Protocol (seqlock per slot, so overwrite-on-full is still race-free):
//! the producer bumps the slot's `seq` to odd, writes the record, then
//! stores `seq = 2 * (index + 1)` with release. A consumer that reads the
//! slot, then re-reads `seq` and sees the same even value, has a consistent
//! copy.

use core::cell::UnsafeCell;
use core::sync::atomic::{fence, AtomicU64, Ordering};

#[repr(C)]
pub struct Slot<T: Copy> {
    seq: AtomicU64,
    val: UnsafeCell<T>,
}

/// `N` must be a power of two.
#[repr(C)]
pub struct Ring<T: Copy, const N: usize> {
    /// Total records ever pushed (written only by the producer).
    head: AtomicU64,
    _pad: [u64; 7],
    slots: [Slot<T>; N],
}

unsafe impl<T: Copy + Send, const N: usize> Sync for Ring<T, N> {}
unsafe impl<T: Copy + Send, const N: usize> Send for Ring<T, N> {}

impl<T: Copy, const N: usize> Ring<T, N> {
    const MASK: u64 = {
        assert!(N.is_power_of_two(), "ring size must be a power of two");
        N as u64 - 1
    };

    /// Initialise a ring in place (for shared memory that is zero-filled:
    /// an all-zero ring is already a valid empty ring).
    ///
    /// # Safety
    /// `p` must point to writable memory of `size_of::<Self>()` bytes,
    /// aligned for `Self`, that no one else is using yet.
    pub unsafe fn init_in_place(p: *mut Self) {
        // SAFETY: the caller guarantees `p` is valid for the whole ring.
        unsafe { core::ptr::write_bytes(p as *mut u8, 0, core::mem::size_of::<Self>()) };
    }

    /// Producer side. Overwrites the oldest record when full.
    #[inline]
    pub fn push(&self, v: &T) {
        let idx = self.head.load(Ordering::Relaxed);
        let slot = &self.slots[(idx & Self::MASK) as usize];
        slot.seq.store(2 * idx + 1, Ordering::Relaxed);
        fence(Ordering::Release);
        // SAFETY: single producer; readers detect a torn copy via `seq`.
        unsafe { core::ptr::write_volatile(slot.val.get(), *v) };
        slot.seq.store(2 * (idx + 1), Ordering::Release);
        self.head.store(idx + 1, Ordering::Release);
    }

    /// Records pushed so far.
    pub fn head(&self) -> u64 {
        self.head.load(Ordering::Acquire)
    }

    /// Consumer side: read record number `idx` (0-based, monotonic).
    /// `Err(Lost)` when it has been overwritten; `Err(NotYet)` when it has
    /// not been written; `Err(Busy)` when the producer is writing it now.
    pub fn read(&self, idx: u64) -> Result<T, ReadError> {
        let head = self.head.load(Ordering::Acquire);
        if idx >= head {
            return Err(ReadError::NotYet);
        }
        if head - idx > N as u64 {
            return Err(ReadError::Lost);
        }
        let slot = &self.slots[(idx & Self::MASK) as usize];
        let s1 = slot.seq.load(Ordering::Acquire);
        // SAFETY: plain-old-data copy; validated by the seqlock below.
        let v = unsafe { core::ptr::read_volatile(slot.val.get()) };
        fence(Ordering::Acquire);
        let s2 = slot.seq.load(Ordering::Relaxed);
        if s1 != s2 || s1 & 1 == 1 {
            return Err(ReadError::Busy);
        }
        if s1 != 2 * (idx + 1) {
            return Err(ReadError::Lost);
        }
        Ok(v)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadError {
    NotYet,
    Lost,
    Busy,
}

/// A consumer cursor that skips lost records and reports how many.
pub struct Reader {
    pub next: u64,
    pub lost: u64,
}

impl Reader {
    pub fn new() -> Self {
        Reader { next: 0, lost: 0 }
    }
    pub fn poll<T: Copy, const N: usize>(&mut self, r: &Ring<T, N>) -> Option<T> {
        loop {
            match r.read(self.next) {
                Ok(v) => {
                    self.next += 1;
                    return Some(v);
                }
                Err(ReadError::NotYet) => return None,
                Err(ReadError::Busy) => return None,
                Err(ReadError::Lost) => {
                    let head = r.head();
                    let oldest = head.saturating_sub(N as u64 - 1);
                    self.lost += oldest.saturating_sub(self.next).max(1);
                    self.next = oldest.max(self.next + 1);
                }
            }
        }
    }
}

impl Default for Reader {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[derive(Clone, Copy, Debug, PartialEq)]
    #[repr(C)]
    struct Rec {
        n: u64,
        x: [f32; 30],
    }

    fn boxed<const N: usize>() -> Box<Ring<Rec, N>> {
        unsafe {
            let layout = std::alloc::Layout::new::<Ring<Rec, N>>();
            let p = std::alloc::alloc_zeroed(layout) as *mut Ring<Rec, N>;
            Box::from_raw(p)
        }
    }

    #[test]
    fn order_and_loss() {
        let r = boxed::<8>();
        let mut rd = Reader::new();
        for n in 0..5 {
            r.push(&Rec { n, x: [n as f32; 30] });
        }
        for n in 0..5 {
            assert_eq!(rd.poll(&r).unwrap().n, n);
        }
        assert!(rd.poll(&r).is_none());
        for n in 5..40 {
            r.push(&Rec { n, x: [n as f32; 30] });
        }
        let first = rd.poll(&r).unwrap().n;
        assert!(first >= 32, "{first}");
        assert!(rd.lost > 0);
    }

    #[test]
    fn concurrent_never_torn() {
        let r: Arc<Ring<Rec, 16>> = Arc::from(boxed::<16>());
        let w = r.clone();
        let t = std::thread::spawn(move || {
            for n in 0..200_000u64 {
                w.push(&Rec { n, x: [n as f32; 30] });
            }
        });
        let mut rd = Reader::new();
        let mut last = None;
        let mut got = 0;
        while !t.is_finished() {
            if let Some(v) = rd.poll(&r) {
                assert!(v.x.iter().all(|&e| e == v.n as f32), "torn record");
                if let Some(l) = last {
                    assert!(v.n > l);
                }
                last = Some(v.n);
                got += 1;
            }
        }
        t.join().unwrap();
        while rd.poll(&r).is_some() {
            got += 1;
        }
        assert!(got > 0);
    }
}

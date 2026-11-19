//! Fixed look-ahead delay line, alias-safe (input and output may be the
//! same buffer) and copy-based, so its cost is two `memmove`s per segment
//! instead of a per-sample ring walk.

use alloc::boxed::Box;
use alloc::vec;

pub struct Delay {
    len: usize,
    /// `len` most recent input samples, oldest first.
    hist: Box<[f32]>,
    /// Scratch for the next history (preallocated, same length).
    next: Box<[f32]>,
}

impl Delay {
    /// Allocates; call off the RT thread.
    pub fn new(len: usize) -> Self {
        Delay { len, hist: vec![0.0; len].into_boxed_slice(), next: vec![0.0; len].into_boxed_slice() }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn clear(&mut self) {
        self.hist.iter_mut().for_each(|v| *v = 0.0);
    }

    /// The delayed signal for `n` frames: `out[k] = in[k - len]`.
    ///
    /// # Safety
    /// `inp` and `out` must each be valid for `n` elements; they may be the
    /// same pointer but must not otherwise partially overlap.
    pub unsafe fn run(&mut self, inp: *const f32, out: *mut f32, n: usize) {
        // SAFETY: the caller guarantees `inp`/`out` hold `n` samples; the
        // history buffers hold `len`. `copy` (memmove) covers the aliased case.
        unsafe { self.run_inner(inp, out, n) }
    }

    unsafe fn run_inner(&mut self, inp: *const f32, out: *mut f32, n: usize) {
        // SAFETY: see `run`.
        unsafe {
            let l = self.len;
            if l == 0 {
                if !core::ptr::eq(inp, out) {
                    core::ptr::copy(inp, out, n);
                }
                return;
            }
            if n >= l {
                // new history = last l inputs (save before out overwrites them)
                core::ptr::copy_nonoverlapping(inp.add(n - l), self.next.as_mut_ptr(), l);
                // out[l..n] = in[0..n-l]  (memmove: may overlap when aliased)
                core::ptr::copy(inp, out.add(l), n - l);
                // out[0..l] = old history
                core::ptr::copy_nonoverlapping(self.hist.as_ptr(), out, l);
            } else {
                // new history = hist[n..l] ++ in[0..n]
                core::ptr::copy_nonoverlapping(self.hist.as_ptr().add(n), self.next.as_mut_ptr(), l - n);
                core::ptr::copy_nonoverlapping(inp, self.next.as_mut_ptr().add(l - n), n);
                // out[0..n] = hist[0..n]
                core::ptr::copy_nonoverlapping(self.hist.as_ptr(), out, n);
            }
            core::mem::swap(&mut self.hist, &mut self.next);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn delays_any_split_and_aliased() {
        for l in [0usize, 1, 96, 192] {
            let x: std::vec::Vec<f32> = (0..3000).map(|i| i as f32 + 1.0).collect();
            for split in [1usize, 7, 95, 96, 97, 256, 512, 1024] {
                let mut d = Delay::new(l);
                let mut buf = x.clone();
                let mut pos = 0;
                while pos < buf.len() {
                    let n = split.min(buf.len() - pos);
                    unsafe {
                        let p = buf.as_mut_ptr().add(pos);
                        d.run(p, p, n);
                    }
                    pos += n;
                }
                for (k, &v) in buf.iter().enumerate() {
                    let want = if k >= l { x[k - l] } else { 0.0 };
                    assert_eq!(v, want, "l={l} split={split} k={k}");
                }
            }
        }
    }
}

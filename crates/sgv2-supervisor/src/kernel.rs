//! The kernel and amp interface, behind a trait so tests never touch
//! hardware.
//!
//! TODO(lane D): the macaudio implementation: the "Speaker Volume Unlock"
//! interlock (lock, renew within the 250 ms lease, explicit unlock), the
//! amp's fixed operating gain from the parameters, and the sense PCM
//! period.

#[derive(Debug, PartialEq, Eq)]
pub enum KernelError {
    NoCard,
    Control(String),
}

pub trait Kernel {
    /// Take or renew the interlock. While held, the amp runs at the fixed
    /// operating gain; when released, the kernel's safe level applies.
    fn renew(&mut self) -> Result<(), KernelError>;
    /// Release the interlock now (the kernel's safe level applies at once).
    fn release(&mut self) -> Result<(), KernelError>;
}

/// A kernel double for tests: records calls.
#[derive(Default, Debug)]
pub struct Recorder {
    pub held: bool,
    pub renewals: u32,
    pub releases: u32,
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
}

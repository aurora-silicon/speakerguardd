//! The per-quantum state the differential rig compares with macOS.
//!
//! Field names follow what the macOS units publish. Fields a unit does not
//! have stay NaN, so "absent" is never confused with zero.

use crate::{Health, MAX_CHANNELS};

#[derive(Clone, Copy, Debug)]
pub struct ChannelState {
    // spp3 / atsp / lsm2 thermal
    pub modeled_power_w: f32,
    pub measured_power_w: f32,
    pub coil_temp_c: f32,
    pub magnet_temp_c: f32,
    pub ol_temp_c: f32,
    pub cl_temp_c: f32,
    pub tlim_adj_c: f32,
    pub closed_loop: bool,
    // lsm2 mechanical and controller
    pub displacement_m: f32,
    pub velocity_m_s: f32,
    pub mask_db: f32,
    /// Candidate gains per axis: sample peak, displacement, velocity,
    /// power (most restrictive of the four budgets), temperature.
    pub axis_gain: [f32; 5],
    pub band_gain: [f32; 5],
    /// Final linear gain applied this quantum.
    pub gain: f32,
}

impl ChannelState {
    pub const ABSENT: ChannelState = ChannelState {
        modeled_power_w: f32::NAN,
        measured_power_w: f32::NAN,
        coil_temp_c: f32::NAN,
        magnet_temp_c: f32::NAN,
        ol_temp_c: f32::NAN,
        cl_temp_c: f32::NAN,
        tlim_adj_c: f32::NAN,
        closed_loop: false,
        displacement_m: f32::NAN,
        velocity_m_s: f32::NAN,
        mask_db: f32::NAN,
        axis_gain: [f32::NAN; 5],
        band_gain: [f32::NAN; 5],
        gain: f32::NAN,
    };
}

#[derive(Clone, Copy, Debug)]
pub struct Telemetry {
    /// Sample index of the first frame of the quantum this describes.
    pub frame: u64,
    pub health: Health,
    pub channels: [ChannelState; MAX_CHANNELS],
}

impl Telemetry {
    pub const fn new() -> Self {
        Self { frame: 0, health: Health::OK, channels: [ChannelState::ABSENT; MAX_CHANNELS] }
    }
    pub fn clear(&mut self) {
        *self = Self { frame: self.frame, ..Self::new() };
    }
}

impl Default for Telemetry {
    fn default() -> Self {
        Self::new()
    }
}

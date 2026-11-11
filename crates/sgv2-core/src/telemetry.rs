//! The per-quantum state the differential rig compares with macOS.
//!
//! Field names follow what the macOS units publish. Fields a unit does not
//! have stay NaN, so "absent" is never confused with zero.

use crate::{Health, MAX_CHANNELS};

/// `repr(C)`: records cross a shared mapping between the plugin (inside
/// PipeWire) and the supervisor.
#[repr(C)]
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
    /// lsm2: the temperature the controller's knee acts on.
    pub control_temp_c: f32,
    /// spp3: the channel runs a closed-loop estimator (the kernel decision
    /// the rig compares with macOS). It does not say the estimate is
    /// current: see `cl_fresh`.
    pub closed_loop: bool,
    /// spp3: 1 while `cl_temp_c` is a current measurement (a pilot-bin
    /// window accepted within the last window plus one thermal update), 0
    /// otherwise. `cl_temp_c` keeps the reference's value either way: the
    /// calibration temperature before the first accepted window, then the
    /// last accepted one, so a channel whose pilot is off (drive below
    /// `PilotEnableThres`) shows a constant `cl_temp_c` with `cl_fresh` 0.
    /// Only a fresh estimate is used by the controller.
    ///
    /// A byte, in what was padding after `closed_loop`: the record's size
    /// and every other offset are unchanged (asserted below), so the engine
    /// page keeps its layout; an engine that predates the field leaves 0.
    pub cl_fresh: u8,
    // lsm2 mechanical and controller
    pub displacement_m: f32,
    pub velocity_m_s: f32,
    pub mask_db: f32,
    /// lsm2: the masking-dependent velocity limit in force.
    pub velocity_limit_m_s: f32,
    /// Candidate gains per axis: sample peak, displacement, velocity,
    /// power (most restrictive of the four budgets), temperature.
    pub axis_gain: [f32; 5],
    pub band_gain: [f32; 5],
    /// Final linear gain applied this quantum.
    pub gain: f32,
    // V/I sense diagnostics (spp3): the last completed closed-loop window's
    // scaled mean squares (V^2, A^2) and its unsmoothed resistance
    // `sqrt(V_ms / I_ms)` (ohm), whether or not the window passed the power
    // floors and `RebRange`. Bring-up and rig observables only; nothing in
    // the protection path reads them.
    pub sense_v_ms: f32,
    pub sense_i_ms: f32,
    pub sense_r_ohm: f32,
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
        control_temp_c: f32::NAN,
        closed_loop: false,
        cl_fresh: 0,
        displacement_m: f32::NAN,
        velocity_m_s: f32::NAN,
        mask_db: f32::NAN,
        velocity_limit_m_s: f32::NAN,
        axis_gain: [f32::NAN; 5],
        band_gain: [f32::NAN; 5],
        gain: f32::NAN,
        sense_v_ms: f32::NAN,
        sense_i_ms: f32::NAN,
        sense_r_ohm: f32::NAN,
    };
}

// The engine page shares these records across versions: `cl_fresh` sits in
// former padding and must not move anything.
const _: () = {
    assert!(core::mem::offset_of!(ChannelState, cl_fresh) == 33);
    assert!(core::mem::offset_of!(ChannelState, displacement_m) == 36);
    assert!(core::mem::size_of::<ChannelState>() == 108);
};

/// Unit-wide state (spp3 and lsm2 meters, the lsm2 controller). NaN when
/// absent.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GlobalState {
    /// spp3: the unit-wide modeled and measured speaker power, W.
    pub modeled_power_w: f32,
    pub measured_power_w: f32,
    /// spp3: the unit-wide thermal gain, dB.
    pub thermal_gain_db: f32,
    /// The four power budgets in force (preset or runtime), W.
    pub power_hard_limit_w: [f32; 4],
    /// The linked (slow) power gain, linear.
    pub power_gain: f32,
    /// `ModeledAmplifierPower`: slow-smoothed total amplifier input power, W.
    pub modeled_amp_power_w: f32,
    /// `AmplifierInputPowerPerSpeaker`: the current controller frame's
    /// amplifier input power over all systems, W.
    pub amp_input_power_w: f32,
}

impl GlobalState {
    pub const ABSENT: GlobalState = GlobalState {
        modeled_power_w: f32::NAN,
        measured_power_w: f32::NAN,
        thermal_gain_db: f32::NAN,
        power_hard_limit_w: [f32::NAN; 4],
        power_gain: f32::NAN,
        modeled_amp_power_w: f32::NAN,
        amp_input_power_w: f32::NAN,
    };
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Telemetry {
    /// Sample index of the first frame of the quantum this describes.
    pub frame: u64,
    pub health: Health,
    pub channels: [ChannelState; MAX_CHANNELS],
    pub global: GlobalState,
}

impl Telemetry {
    pub const fn new() -> Self {
        Self {
            frame: 0,
            health: Health::OK,
            channels: [ChannelState::ABSENT; MAX_CHANNELS],
            global: GlobalState::ABSENT,
        }
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

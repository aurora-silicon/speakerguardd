//! `sgv2-render`: the Linux side of the differential rig.
//!
//! Ownership: lane C. TODO(lane C): read program and telemetry WAVs and a
//! parameter blob, render at the case's slice sizes (1, 7, 31, 96, 127,
//! 160, 512, 1024, 4096), and write one JSON line per slice in the schema of
//! tools/diffrig/schema.json.

use sgv2_core::{FailSafe, Health, Io, ProtectionUnit, Telemetry, UnitKind};

fn main() {
    // Smoke path until the rig lands: one slice of silence through the
    // fail-safe, printed in the trace schema's field names.
    let input = [0.0f32; 16];
    let mut output = [0.0f32; 16];
    let ins: [&[f32]; 1] = [&input];
    let mut outs: [&mut [f32]; 1] = [&mut output];
    let mut unit = FailSafe::new(UnitKind::Spp3, Health::NO_PARAMS);
    let health = unit.process(Io { frames: 16, input: &ins, sense: &[], output: &mut outs });
    let mut t = Telemetry::new();
    unit.telemetry(&mut t);
    println!("{{\"frame\":0,\"frames\":16,\"health\":{},\"gain\":[{}]}}", health.0, t.channels[0].gain);
}

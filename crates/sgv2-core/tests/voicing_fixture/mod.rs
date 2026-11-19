//! Synthetic parameters only, using the public boxes' test/default shapes.
//! Board fixtures supply geometry; these are no Mac's tuning values.
#![allow(dead_code)]
use sgv2_core::prestage::{Channel, DynamicAt, DynamicFilter, DynamicKind};
use sgv2_core::voicing::{self, chain::*};
use sgv2_core::{blob, UnitKind, MAX_CHANNELS};

pub fn spec(lanes: usize) -> Spec {
    let volt = voicing::volt::Params {
        saved_scalar: 1.0,
        pre_gain_db: 0.0,
        curve_db: core::array::from_fn(|i| i as f64 - 16.0),
        offset_db: [0.0; 5],
    };
    let mut points = [voicing::mstk::AutomationPoint::default(); voicing::mstk::MAX_AUTOMATION_POINTS];
    points[0] = voicing::mstk::AutomationPoint { input: -16.0, value: -12.0 };
    points[1] = voicing::mstk::AutomationPoint { input: 0.0, value: -24.0 };
    let comp = voicing::mstk::Params {
        threshold_automation: core::array::from_fn(|i| {
            if i == 0 {
                voicing::mstk::ThresholdAutomation { points, count: 2 }
            } else {
                voicing::mstk::ThresholdAutomation::default()
            }
        }),
        automation_min: -16.0,
        automation_max: 0.0,
        ..Default::default()
    };
    let mut ch = Channel::IDENTITY;
    ch.count = 1;
    ch.sections[0] = voicing::tone::low_shelf(3.0, 150.0, 0.7, 48_000.0);
    let tone = Tone {
        channels: vec![ch; 2],
        automation: vec![ToneAutomation {
            shape: ToneShape::LowShelf,
            quality: 0.7,
            fixed: [0.0; 3],
            max_order: 10,
            a: TonePoint { scalar: 0.0, gain_db: 3.0, frequency_hz: 150.0 },
            b: TonePoint { scalar: 1.0, gain_db: 0.0, frequency_hz: 400.0 },
            midpoint: Some(TonePoint { scalar: 0.5, gain_db: 1.0, frequency_hz: 200.0 }),
            targets: vec![ToneTarget { channel: 0, section: 0 }, ToneTarget { channel: 1, section: 0 }],
        }],
        dynamic: vec![DynamicAt {
            active_mask: 3,
            sidechain_mask: 3,
            params: DynamicFilter { kind: DynamicKind::HighShelf, range_db: -3.0, ..Default::default() },
            ..Default::default()
        }],
        dynamic_automation: vec![DynamicToneAutomation {
            index: 0,
            a: TonePoint { scalar: 0.0, gain_db: -3.0, frequency_hz: 1000.0 },
            b: TonePoint { scalar: 1.0, gain_db: -1.0, frequency_hz: 2000.0 },
            midpoint: None,
        }],
    };
    let vareq = voicing::vareq::Params { breakpoints: [0.1, 0.3, 0.6, 0.8], ..Default::default() };
    let kinds = vec![
        Kind::NonFinite,
        Kind::Gain,
        Kind::Volt(Box::new(volt)),
        Kind::Tone(tone),
        Kind::Mstk(Box::new(comp)),
        Kind::Norm(Box::default()),
        Kind::Xtc(Box::new(voicing::xtc::Params::identity())),
        Kind::Vareq(Box::new(vareq)),
        Kind::Vbass(Box::new(voicing::vbass::Params {
            fc_hz: 100.0,
            hp_order: 4,
            fu_hz: 250.0,
            upper_order: 4,
            lower_order: 4,
            harmonic_gain: 0.1,
            blend_gain: 1.0,
            input_gain: 1.0,
            output_gain: 1.0,
            device: voicing::vbass::Device::Saturator3,
            level_control: true,
            harmonic_compressor: true,
            compressor_threshold_db: -12.0,
            compressor_ratio: 2.0,
        })),
        Kind::Cfreak(Box::new(voicing::cfreak::Params {
            lookahead_s: 0.001,
            hold_s: 0.002,
            threshold_db: -6.0,
            min_fast_gain_db: 20.0,
            true_peak: false,
            saturation_mode: 0,
            saturation_flavor: 0,
            saturation_ceiling_db: 0.0,
            deesser_hz: 0.0,
            deesser_db: 0.0,
            deesser_q: 0.0,
            input_gain_db: 0.0,
            output_gain_db: 0.0,
            highpass_order: 0,
            highpass_hz: 0.0,
            highpass_location: 0,
        })),
        Kind::Buzz(Box::default()),
        Kind::Cdelay(Box::new(voicing::cdelay::Params { channels: 2, delay_frames: [0; MAX_CHANNELS] })),
        Kind::Pchr(Box::new(voicing::pchr::Params {
            channels: 2.0,
            matrix: core::array::from_fn(|r| core::array::from_fn(|c| if r == c { 1.0 } else { 0.0 })),
        })),
        Kind::Lowflow(Box::new(lowflow_params())),
        Kind::Bypass,
    ];
    let nodes: Vec<_> = kinds
        .into_iter()
        .enumerate()
        .map(|(i, kind)| NodeSpec {
            kind,
            inputs: (0..2).map(|c| if i == 0 { Source::Program(c) } else { Source::Node(i - 1, c) }).collect(),
        })
        .collect();
    let last = nodes.len() - 1;
    Spec {
        program_channels: 2,
        nodes,
        outputs: (0..lanes).map(|c| Source::Node(last, c % 2)).collect(),
        links: vec![
            Link { param: GraphParam::VolumeScalar, node: 2, id: 4 },
            Link { param: GraphParam::VolumeScalar, node: 3, id: 3 },
            Link { param: GraphParam::VolumeScalar, node: 7, id: 5 },
            Link { param: GraphParam::VolumeDb, node: 1, id: 0 },
            Link { param: GraphParam::VolumeDb, node: last - 1, id: 4 },
        ],
        relays: vec![Relay { from_node: 2, from_id: 100, to_node: 4, to_id: 79 }],
        volume: Default::default(),
    }
}

pub fn unit(board: &str, lanes: usize) -> Vec<u8> {
    let blob = blob::build_for(
        UnitKind::Spp3,
        lanes as u32,
        (lanes * 2) as u32,
        48_000,
        blob::machine_field(board).unwrap(),
        10_000,
        &[],
    );
    let mut pre = sgv2_core::prestage::Params::identity(lanes);
    pre.flags = sgv2_core::prestage::FLAG_EXTERNAL_VOICING | sgv2_core::prestage::FLAG_FROM_TUNING;
    blob::with_prestage(&blob, &pre).unwrap()
}

pub fn lowflow_params() -> voicing::lowflow::Params {
    use voicing::lowflow::{Model, Params, MAX_MODELS};
    let model = Model {
        bl: 1.0,
        kms: 1.0,
        kms_front_port: 1.0,
        mms: 1.0,
        mms_front_port: 1.0,
        reb: 1.0,
        rms: 1.0,
        rms_front_port: 1.0,
        analog_gain: 1.0,
        tweeter_gain: 1.0,
        mask_band_min_hz: 100.0,
        mask_band_max_hz: 3000.0,
        min_velocity: 1.0,
        mid_velocity: 2.0,
        high_velocity: 3.0,
        mid_level_db: -12.0,
        min_frequency_hz: 20.0,
        max_frequency_hz: 3000.0,
        enabled: true,
        max_attenuation_db: 6.0,
        mid_attenuation_db: 3.0,
        ..Default::default()
    };
    let mut models = [Model::default(); MAX_MODELS];
    models[0] = model;
    models[0].pressure_response[0] = 1.0;
    models[0].tweeter_response[0] = 1.0;
    Params { model_count: 1, link_channel_energies: true, latency_frames: 256, conservative_fallback: false, models }
}

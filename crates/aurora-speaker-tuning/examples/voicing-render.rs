//! Rig renderer for the voicing chain: compiles a Mac's chain from its
//! tuning and renders a stereo program into the unit's lanes.
//!
//! ```text
//! voicing-render (--bundle PATH | --tunings-root DIR --aid N [--speaker-id N])
//!     --in PROGRAM.f32 --out LANES.f32 [--mvsc S] [--mvol DB|auto]
//! (`--mvol auto`: the dB the plugin derives from S with the artifact's volume law)
//!     [--tap NODE=FILE.f32 ...] [--info FILE.json] [--block N] [--only NODE]
//!     [--profile PROFILE.json]
//! ```
//!
//! `--only NODE` runs that one box alone: the input has the box's width and
//! the output is the box's output (the per-box gate of the rig).
//!
//! Interleaved float32 at 48 kHz. `--tap` writes a box's output (all its
//! channels, interleaved). `--info` writes the plan, the stubs and the
//! graph parameters the chain does not drive. Output from a tuning is
//! Mac-derived data: keep it out of git.
use aurora_speaker_tuning::bundle::Bundle;
use aurora_speaker_tuning::tuning::{Options, Tuning};
use aurora_speaker_tuning::voicing::chain::compile;
use aurora_speaker_tuning::Diagnostics;
use sgv2_core::voicing::chain::{Chain, GraphParam, Link, Source, Spec};
use std::io::Write;

fn rd(p: &str) -> Vec<f32> {
    std::fs::read(p)
        .unwrap_or_else(|e| panic!("{p}: {e}"))
        .chunks_exact(4)
        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect()
}

fn main() {
    let mut args = std::env::args().skip(1);
    let (mut root, mut aid, mut bundle, mut input, mut output, mut info) = (None, None, None, None, None, None);
    let (mut mvsc, mut mvol, mut block) = (1.0f64, 0.0f64, 512usize);
    let mut mvol_auto = false;
    let mut ramp: Option<(f64, f64, usize)> = None;
    let mut taps: Vec<(String, String)> = Vec::new();
    let mut only: Option<String> = None;
    let mut profile: Option<String> = None;
    let mut speaker_id: Option<i128> = None;
    while let Some(a) = args.next() {
        let mut v = || args.next().expect("value");
        match a.as_str() {
            "--tunings-root" => root = Some(v()),
            "--aid" => aid = Some(v().trim_start_matches("AID").parse::<u32>().expect("aid")),
            "--bundle" => bundle = Some(v()),
            "--in" => input = Some(v()),
            "--out" => output = Some(v()),
            "--info" => info = Some(v()),
            "--mvsc" => mvsc = v().parse().expect("mvsc"),
            "--mvol" => match v().as_str() {
                "auto" => mvol_auto = true,
                x => mvol = x.parse().expect("mvol"),
            },
            "--block" => block = v().parse().expect("block"),
            "--speaker-id" => speaker_id = Some(v().parse().expect("speaker id")),
            "--only" => only = Some(v()),
            "--profile" => profile = Some(v()),
            // --mvsc-ramp TO:STEP:EVERY: from --mvsc, move the scalar by STEP toward TO every EVERY frames
            // (a host volume slider moved slowly; mvol follows the artifact's law)
            "--mvsc-ramp" => {
                let s = v();
                let p: Vec<f64> = s.split(':').map(|x| x.parse().expect("--mvsc-ramp TO:STEP:EVERY")).collect();
                assert_eq!(p.len(), 3, "--mvsc-ramp TO:STEP:EVERY");
                ramp = Some((p[0], p[1], p[2] as usize));
            }
            "--tap" => {
                let s = v();
                let (n, f) = s.split_once('=').expect("--tap NODE=FILE");
                taps.push((n.to_string(), f.to_string()));
            }
            x => panic!("bad arg {x}"),
        }
    }
    let b = match (root, aid, bundle) {
        (Some(r), Some(a), None) => Bundle::from_tunings_root(r.as_ref(), a),
        (None, None, Some(p)) => Bundle::open(p.as_ref()),
        _ => panic!("--tunings-root DIR --aid N, or --bundle PATH"),
    }
    .expect("bundle");
    let mut diags = Diagnostics::default();
    let mut t = Tuning::load(&b, &Options { speaker_id, ..Options::default() }, &mut diags)
        .unwrap_or_else(|e| panic!("load: {e}"));
    // Raw tuning roots have no installer manifest. Offline cross-OS rigs
    // supply the source build explicitly; production bundles carry it in
    // their manifest and therefore never consult this harness override.
    if t.macos_build.is_none() {
        t.macos_build = std::env::var("SGV2_MACOS_BUILD").ok();
    }
    let c = compile(&t, 48_000.0, &mut diags).unwrap_or_else(|e| {
        eprintln!("compile refused: {e}");
        std::process::exit(3)
    });
    if let Some(p) = &info {
        let j = serde_json::json!({
            "plan": c.plan,
            "stubs": c.stubs,
            "gaps": c.gaps,
            "unwired": c.unwired,
        });
        std::fs::write(p, serde_json::to_vec_pretty(&j).unwrap()).unwrap();
    }
    let (Some(input), Some(output)) = (input, output) else { return };
    let mut c = c;
    if let Some(name) = &only {
        // one box alone: its channels are the input, its output the lanes
        let n = c.plan.node(name).unwrap_or_else(|| panic!("no node {name}"));
        let mut node = c.spec.nodes[n].clone();
        let w = node.inputs.len();
        node.inputs = (0..w).map(Source::Program).collect();
        c.spec = Spec {
            program_channels: w,
            nodes: vec![node],
            outputs: (0..w).map(|k| Source::Node(0, k)).collect(),
            links: c.spec.links.iter().filter(|l| l.node == n).map(|l| Link { node: 0, ..*l }).collect(),
            relays: vec![],
            volume: c.spec.volume,
        };
        assert!(taps.is_empty(), "--only takes no --tap");
    }
    let pc = c.spec.program_channels;
    let lanes = c.spec.outputs.len();
    let tap_nodes: Vec<(usize, String)> =
        taps.iter().map(|(n, f)| (c.plan.node(n).unwrap_or_else(|| panic!("no node {n}")), f.clone())).collect();
    if mvol_auto {
        mvol = c.spec.volume.db(mvsc);
        eprintln!("mvol auto: {:?} at mvsc {mvsc} -> {mvol} dB", c.spec.volume);
    }
    let curve = c.spec.volume.clone();
    let mut chain = Chain::new(c.spec, 48_000.0, block).expect("spec");
    chain.set_param(GraphParam::VolumeScalar, mvsc);
    chain.set_param(GraphParam::VolumeDb, mvol);

    let x = rd(&input);
    let frames = x.len() / pc;
    let mut prog = vec![vec![0f32; block]; pc];
    let mut outs = vec![vec![0f32; block]; lanes];
    let mut w = std::io::BufWriter::new(std::fs::File::create(&output).unwrap());
    let mut tw: Vec<_> =
        tap_nodes.iter().map(|(_, f)| std::io::BufWriter::new(std::fs::File::create(f).unwrap())).collect();
    let mut node_ns = vec![0u128; chain.spec().nodes.len()];
    let mut block_ns = Vec::new();
    let mut node_start = std::time::Instant::now();
    let mut f = 0;
    while f < frames {
        let n = block.min(frames - f);
        if let Some((to, step, every)) = ramp {
            if every > 0 && f > 0 && f % every < n && (mvsc - to).abs() > 1e-12 {
                mvsc = if to > mvsc { (mvsc + step).min(to) } else { (mvsc - step).max(to) };
                chain.set_param(GraphParam::VolumeScalar, mvsc);
                chain.set_param(GraphParam::VolumeDb, curve.db(mvsc));
            }
        }
        for (c, p) in prog.iter_mut().enumerate() {
            for i in 0..n {
                p[i] = x[(f + i) * pc + c];
            }
        }
        let pr: Vec<&[f32]> = prog.iter().map(|v| &v[..n]).collect();
        let mut or: Vec<&mut [f32]> = outs.iter_mut().map(|v| &mut v[..n]).collect();
        if profile.is_some() {
            let start = std::time::Instant::now();
            chain.process_observed(&pr, &mut or, n, |node, entering| {
                if entering {
                    node_start = std::time::Instant::now();
                } else {
                    node_ns[node] += node_start.elapsed().as_nanos();
                }
            });
            block_ns.push(start.elapsed().as_nanos());
        } else {
            chain.process(&pr, &mut or, n);
        }
        for i in 0..n {
            for o in &outs {
                w.write_all(&o[i].to_le_bytes()).unwrap();
            }
        }
        for ((node, _), tw) in tap_nodes.iter().zip(tw.iter_mut()) {
            let width = chain.spec().nodes[*node].inputs.len();
            for i in 0..n {
                for ch in 0..width {
                    tw.write_all(&chain.node_output(*node, ch).unwrap()[i].to_le_bytes()).unwrap();
                }
            }
        }
        f += n;
    }
    if let Some(path) = profile {
        let total_ns: u128 = block_ns.iter().sum();
        let nodes: Vec<_> = chain
            .spec()
            .nodes
            .iter()
            .enumerate()
            .map(|(i, node)| {
                // The kind's variant name is public; omit its tuning parameters.
                let kind = format!("{:?}", node.kind).split(['(', ' ', '{']).next().unwrap().to_owned();
                serde_json::json!({"index": i, "kind": kind, "total_ns": node_ns[i]})
            })
            .collect();
        block_ns.sort_unstable();
        let j = serde_json::json!({
            "frames": frames, "sample_rate": 48_000, "block": block,
            "blocks": block_ns.len(), "total_ns": total_ns,
            "max_block_ns": block_ns.last(),
            "median_block_ns": block_ns.get(block_ns.len() / 2),
            "nodes": nodes,
        });
        std::fs::write(path, serde_json::to_vec_pretty(&j).unwrap()).unwrap();
    }
}

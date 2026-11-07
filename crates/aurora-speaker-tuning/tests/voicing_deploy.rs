//! Deployment uses the same compiler as voicing-render. These bundles use
//! only the existing public synthetic fixture values, with stereo fan-out
//! shaped to the three public board geometries. No Apple tuning is stored.
mod common;
use aurora_speaker_tuning::aupreset::{ParamSection, ParamState};
use aurora_speaker_tuning::plist::{binary, Value};
use aurora_speaker_tuning::{
    bundle::Bundle,
    compile::{self, Provenance},
    machine::Machine,
    tuning::{Options, Tuning},
    Diagnostics,
};
use common::*;
use sgv2_core::voicing::chain::{Chain, GraphParam};
use sgv2_lv2::voicing::{Voicing, BLOCK};

fn tuning(board: &str, lanes: usize) -> Tuning {
    let mut files = spp3_files(lanes);
    let mut graph=format!("in in\nin sense\nout out\nbox Guard nfnp 1 1\nbox Vol (aufx volt) 1 1\nbox Split split 1 2\nbox Join join {lanes} 1\nbox Prot (aufx spp3) 2 1\nwire in Guard ([fs] 2)\nwire Guard Vol ([fs] 2)\nwire Vol Split ([fs] 2)\n");
    for c in 0..lanes {
        graph.push_str(&format!("wire (Split {}) (Join {c}) ([fs] 1)\n", c % 2));
    }
    graph.push_str(&format!("wire Join (Prot 0) ([fs] {lanes} 256)\nwire sense (Prot 1) ([fs] {}i 256)\nwire Prot out ([fs] {lanes})\nparam mvsc 1\nwireGraphParam mvsc (Vol 4 global 0)\n",lanes*2));
    files.insert(format!("{AID}/DSP/Graphs/builtin_speaker_out.dspg"), graph.into_bytes());
    let mut params = vec![
        (0, 0.0),
        (1, 0.0),
        (2, 1.0),
        (3, 1.0),
        (4, 1.0),
        (5, 0.0),
        (6, -200.0),
        (7, 0.0),
        (2048, 0.0),
        (2049, 0.0),
        (2050, 0.0),
    ];
    params.extend((0..17).map(|n| (1024 + n, n as f32 - 16.0)));
    params.extend((0..5).map(|n| (3072 + n, 0.0)));
    let vol = ParamState { sections: vec![ParamSection { scope: 0, element: 0, params }] };
    files.insert(
        format!("{AID}/DSP/Strips/builtin_speaker_out_general.austrip"),
        binary::write(&dict(vec![(
            "strips",
            Value::Array(vec![dict(vec![(
                "effects",
                Value::Array(vec![
                    effect("Vol", "volt", preset("volt", "synthetic volume", &vol)),
                    effect("Prot", "spp3", preset("spp3", "synthetic protection", &spp3_state(lanes as u32))),
                ]),
            )])]),
        )])),
    );
    let dir = match std::env::var_os("SGV2_EQ_DIR") {
        Some(root) => std::path::PathBuf::from(root).join(board).join("bundle"),
        None => scratch(&format!("voicing-deploy-{board}")),
    };
    write_dir(&dir, &files, &manifest_json_ident(&files, "25G83", None, Some(board), Some(&test_unit())));
    let bundle = Bundle::open(&dir).unwrap();
    Tuning::load(&bundle, &Options::default(), &mut Diagnostics::default()).unwrap()
}

#[test]
fn compile_publish_load_equals_the_voicing_render_host_on_three_board_fixtures() {
    for (board, n) in [("j616s", 6), ("j316s", 6), ("j413", 4)] {
        let t = tuning(board, n);
        let m = Machine::with_unit(board, &test_unit()).unwrap();
        let c = compile::compile_voicing_with(&t, &m, Provenance::Verified, &mut Diagnostics::default()).unwrap();
        if let Some(root) = std::env::var_os("SGV2_EQ_DIR") {
            let dir = std::path::PathBuf::from(root).join(board);
            std::fs::write(dir.join("params.bin"), &c.blob).unwrap();
            let samples: Vec<u8> = (0..8193)
                .flat_map(|i| [((i as f64 * 0.071).sin() * 0.3) as f32, ((i as f64 * 0.133).cos() * 0.2) as f32])
                .flat_map(f32::to_le_bytes)
                .collect();
            std::fs::write(dir.join("program.f32"), samples).unwrap();
        }
        let reference =
            aurora_speaker_tuning::voicing::chain::compile(&t, 48000.0, &mut Diagnostics::default()).unwrap();
        assert_eq!(sgv2_core::blob::voicing(&c.blob).unwrap().unwrap(), reference.spec);
        let pre = sgv2_core::blob::prestage(&c.blob, 48000.0).unwrap();
        assert!(pre.channel[..n].iter().all(|c| c.count == 0 && c.gain == 1.0));
        let (unit, err, _) = sgv2_lv2::load_unit(Some(&c.blob), 48000.0, n, n * 2, sgv2_lv2::Role::Protect);
        assert!(err.is_none());
        assert!(sgv2_lv2::load_prestage(Some(&c.blob), 48000.0, n, &unit).is_some());
        let mut render = Chain::new(reference.spec, 48000.0, BLOCK).unwrap();
        let mut plugin = Voicing::load(Some(&c.blob), board, 48000.0, n);
        assert!(plugin.ready());
        let input = core::array::from_fn::<_, BLOCK, _>(|i| ((i as f64 * 0.37).sin() * 0.25) as f32);
        let mut expected = vec![vec![0.0; BLOCK]; n];
        let mut actual = expected.clone();
        let mut max_error = 0.0f32;
        for scalar in [1.0, 0.75, 0.25, 0.5, 1.0] {
            let db = 60.0 * f64::log10(scalar);
            // This is the exact sequence and block size in voicing-render.
            render.set_param(GraphParam::VolumeScalar, scalar);
            render.set_param(GraphParam::VolumeDb, db);
            let mut e: Vec<_> = expected.iter_mut().map(|b| &mut b[..]).collect();
            let mut a: Vec<_> = actual.iter_mut().map(|b| &mut b[..]).collect();
            render.process(&[&input, &input], &mut e, BLOCK);
            plugin.process(&[&input, &input], &mut a, BLOCK, scalar, db);
            for (a, b) in actual.iter().flatten().zip(expected.iter().flatten()) {
                max_error = max_error.max((a - b).abs());
            }
        }
        assert_eq!(max_error, 0.0);
        println!("COMPILER_EQUIVALENCE {board}: lanes={n} max_abs_error={max_error:.9}");
        let mut wrong = t.clone();
        wrong.board = Some("j999".into());
        assert!(compile::compile_voicing_with(&wrong, &m, Provenance::Verified, &mut Diagnostics::default()).is_err());
        let mut unverified = t.clone();
        unverified.verified = false;
        assert!(
            compile::compile_voicing_with(&unverified, &m, Provenance::Verified, &mut Diagnostics::default()).is_err()
        );
        let other = Machine::with_unit(board, &other_unit()).unwrap();
        assert!(compile::compile_voicing_with(&t, &other, Provenance::Verified, &mut Diagnostics::default()).is_err());
    }
}

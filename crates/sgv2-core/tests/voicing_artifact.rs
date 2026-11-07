mod voicing_fixture;
use sgv2_core::{blob, voicing::artifact};

#[test]
fn full_spec_roundtrips_and_is_bound_to_the_unit_blob() {
    let spec = voicing_fixture::spec(6);
    let base = voicing_fixture::unit("j616s", 6);
    let bytes = artifact::encode(&spec, &base).unwrap();
    assert_eq!(artifact::decode(&bytes, &base).unwrap(), spec);
    let joined = blob::with_voicing(&base, &spec).unwrap();
    assert_eq!(blob::voicing(&joined).unwrap().unwrap(), spec);
    let mut changed = spec.clone();
    changed.relays[0].to_id = 78;
    let updated = blob::with_voicing(&base, &changed).unwrap();
    assert_eq!(joined.len(), updated.len());
    assert_ne!(
        blob::engine_crc(&joined).unwrap(),
        blob::engine_crc(&updated).unwrap(),
        "voicing-only update must invalidate a live heartbeat"
    );
    assert!(blob::voicing(&base).is_err(), "raw feed without artifact must refuse");
    assert!(artifact::decode(&bytes, &voicing_fixture::unit("j316s", 6)).is_err());
    let mut other = base.clone();
    other[44] ^= 1; // operating point is bound too
    assert!(artifact::decode(&bytes, &other).is_err());
    for at in (0..bytes.len()).step_by(251) {
        let mut bad = bytes.clone();
        bad[at] ^= 0x80;
        assert!(artifact::decode(&bad, &base).is_err());
    }
    for end in (0..bytes.len()).step_by(251) {
        assert!(artifact::decode(&bytes[..end], &base).is_err());
    }
    let mut bad = bytes;
    bad.extend_from_slice(&[0; 8]);
    assert!(artifact::decode(&bad, &base).is_err());
}

#[test]
fn rejects_stubs_bad_shapes_and_oversized_counts_without_panicking() {
    use sgv2_core::voicing::chain::Kind;
    let base = voicing_fixture::unit("j413", 4);
    let mut spec = voicing_fixture::spec(4);
    spec.nodes[0].kind = Kind::Stub(*b"test");
    assert!(artifact::encode(&spec, &base).is_err());
    let mut spec = voicing_fixture::spec(4);
    let Kind::Tone(t) = &mut spec.nodes[3].kind else { panic!() };
    t.channels[0].count = usize::MAX;
    assert!(artifact::encode(&spec, &base).is_err());
}

#[test]
fn lowflow_parameter_words_are_carried_including_responses() {
    use sgv2_core::voicing::{chain::Kind, lowflow::*};
    let mut spec = voicing_fixture::spec(4);
    let m = Model {
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
        max_frequency_hz: 3000.0,
        ..Default::default()
    };
    let mut models = [Model::default(); MAX_MODELS];
    models[0] = m;
    models[0].pressure_response[7] = 0.125;
    models[0].tweeter_response[13] = 0.0625;
    spec.nodes[0].kind =
        Kind::Lowflow(Box::new(Params {
            model_count: 1,
            link_channel_energies: true,
            latency_frames: 256,
            conservative_fallback: false,
            models,
        }));
    let base = voicing_fixture::unit("j413", 4);
    let bytes = artifact::encode(&spec, &base).unwrap();
    assert_eq!(u32::from_le_bytes(bytes[12..16].try_into().unwrap()), CONSERVATIVE as u32);
    assert_eq!(artifact::decode(&bytes, &base).unwrap(), spec);
    let mut wrong_flavor = bytes;
    wrong_flavor[12..16].copy_from_slice(&((!CONSERVATIVE) as u32).to_le_bytes());
    let end = wrong_flavor.len() - 4;
    let crc = blob::crc32(&wrong_flavor[..end]);
    wrong_flavor[end..].copy_from_slice(&crc.to_le_bytes());
    assert_eq!(artifact::decode(&wrong_flavor, &base), Err(blob::BlobError::Version));
}

#[test]
fn artifacts_without_lowflow_keep_build_independent_bytes_and_read_legacy_flavors() {
    use sgv2_core::voicing::chain::Kind;
    let mut spec = voicing_fixture::spec(4);
    let lowflow = spec.nodes.iter().position(|node| matches!(node.kind, Kind::Lowflow(_))).unwrap();
    spec.nodes[lowflow].kind = Kind::Bypass;
    spec.links.retain(|link| link.node != lowflow);
    let base = voicing_fixture::unit("j413", 4);
    let bytes = artifact::encode(&spec, &base).unwrap();
    assert_eq!(u32::from_le_bytes(bytes[12..16].try_into().unwrap()), 1);

    let fix_crc = |b: &mut [u8]| {
        let end = b.len() - 4;
        let crc = blob::crc32(&b[..end]);
        b[end..].copy_from_slice(&crc.to_le_bytes());
    };
    let mut legacy_exact = bytes.clone();
    legacy_exact[12..16].copy_from_slice(&0u32.to_le_bytes());
    fix_crc(&mut legacy_exact);
    assert_eq!(artifact::decode(&legacy_exact, &base).unwrap(), spec);

    let mut unknown = bytes;
    unknown[12..16].copy_from_slice(&2u32.to_le_bytes());
    fix_crc(&mut unknown);
    assert_eq!(artifact::decode(&unknown, &base), Err(blob::BlobError::Version));
}

#[test]
fn interim_dynamics_kind_is_roundtripped_too() {
    use sgv2_core::prestage::{Band, Dynamics};
    use sgv2_core::voicing::chain::Kind;
    let mut spec = voicing_fixture::spec(4);
    spec.nodes[0].kind = Kind::Comp(Dynamics {
        crossover_hz: [100.0, 1000.0, 10000.0],
        release_s: 0.05,
        band: [Band { ratio: 2.0, ..Default::default() }; 4],
        ..Default::default()
    });
    let base = voicing_fixture::unit("j413", 4);
    let bytes = artifact::encode(&spec, &base).unwrap();
    assert_eq!(artifact::decode(&bytes, &base).unwrap(), spec);
}

#[test]
fn valid_crc_does_not_admit_unknown_versions_or_unbounded_spec_lengths() {
    let base = voicing_fixture::unit("j413", 4);
    let good = artifact::encode(&voicing_fixture::spec(4), &base).unwrap();
    let fix_crc = |b: &mut [u8]| {
        let end = b.len() - 4;
        let crc = blob::crc32(&b[..end]);
        b[end..].copy_from_slice(&crc.to_le_bytes());
    };
    let mut bad = good.clone();
    bad[8..12].copy_from_slice(&u32::MAX.to_le_bytes());
    fix_crc(&mut bad);
    assert_eq!(artifact::decode(&bad, &base), Err(blob::BlobError::Version));
    let mut bad = good;
    // Identity is 20 bytes; program_channels follows, then nodes' count.
    bad[24..28].copy_from_slice(&u32::MAX.to_le_bytes());
    fix_crc(&mut bad);
    assert_eq!(artifact::decode(&bad, &base), Err(blob::BlobError::Length));
}

#[test]
fn cfreak_gains_roundtrip_without_changing_legacy_unity_records() {
    use sgv2_core::voicing::chain::Kind;
    let base = voicing_fixture::unit("j413", 4);
    let mut spec = voicing_fixture::spec(4);
    let legacy = artifact::encode(&spec, &base).unwrap();
    assert_eq!(artifact::decode(&legacy, &base).unwrap(), spec);
    let node = spec.nodes.iter_mut().find(|n| matches!(n.kind, Kind::Cfreak(_))).unwrap();
    let Kind::Cfreak(p) = &mut node.kind else { unreachable!() };
    p.input_gain_db = 3.0;
    p.output_gain_db = -2.0;
    let extended = artifact::encode(&spec, &base).unwrap();
    assert_eq!(extended.len(), legacy.len() + 16);
    assert_eq!(artifact::decode(&extended, &base).unwrap(), spec);
    let node = spec.nodes.iter_mut().find(|n| matches!(n.kind, Kind::Cfreak(_))).unwrap();
    let Kind::Cfreak(p) = &mut node.kind else { unreachable!() };
    p.input_gain_db = 0.0;
    p.output_gain_db = 0.0;
    assert_eq!(artifact::encode(&spec, &base).unwrap(), legacy);
}

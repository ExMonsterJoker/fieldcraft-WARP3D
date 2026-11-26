use super::*;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};

static NEXT: AtomicU64 = AtomicU64::new(0);
fn folder() -> PathBuf {
    let path = std::env::temp_dir().join(format!("fieldcraft-deck-read-{}-{}", std::process::id(), NEXT.fetch_add(1, Ordering::Relaxed)));
    std::fs::create_dir_all(&path).unwrap();
    path
}

fn bar() -> Session {
    let mut s = Session::default();
    super::super::define::tests::tagged_bar(&mut s);
    s.run_script(&json!([
        {"command":"model.units","params":{"system":"mm-t-s-N-MPa"}},
        {"command":"model.set.create","params":{"name":"all","geometry":[{"body":"Bar"}]}},
        {"command":"model.set.create","params":{"name":"root","geometry":[{"body":"Bar","face":"left"}]}},
        {"command":"model.set.create","params":{"name":"end","geometry":[{"body":"Bar","face":"right"}]}},
        {"command":"model.material.create","params":{"name":"Steel","elastic":{"type":"isotropic","young":"210 GPa","poisson":0.3},"density":"7850 kg/m^3"}},
        {"command":"model.section.create","params":{"name":"BarSection","set":"all","material":"Steel","kind":{"type":"solid"},"formulation":{"incompatible_modes":true}}}
    ])).unwrap();
    constrain(&mut s);
    s
}

fn constrain(s: &mut Session) {
    s.run_script(&json!([
        {"command":"model.boundary_condition.create","params":{"name":"clamp","type":"fixed","target":{"set":"root"}}},
        {"command":"model.load.create","params":{"name":"push","type":"force","target":{"set":"end"},"vector":["0 N","0 N","-100 N"]}},
        {"command":"model.step.create","params":{"name":"Bend","procedure":{"type":"static"},"boundary_conditions":["clamp"],"loads":["push"]}}
    ]))
    .unwrap();
}

fn assert_unchanged(s: &Session, before: &Session) {
    assert_eq!(s.model, before.model);
    assert_eq!(s.document(), before.document());
    assert_eq!(s.journal, before.journal);
    assert_eq!(s.results_report(), before.results_report());
    assert_eq!(s.results.len(), before.results.len());
    for (name, (revision, result)) in &before.results {
        let (kept_revision, kept) = s.results.get(name).unwrap();
        assert_eq!(revision, kept_revision);
        assert!(std::sync::Arc::ptr_eq(result, kept));
    }
    assert_eq!(s.undo_depth(), before.undo_depth());
    assert_eq!(s.revision(), before.revision());
    assert_eq!(s.opened(), before.opened());
    assert_eq!(s.is_dirty(), before.is_dirty());
    assert_eq!(s.file_access, before.file_access);
    assert_eq!(s.view, before.view);
    assert_eq!(s.design.is_some(), before.design.is_some());
    if let (Some(kept), Some(original)) = (&s.design, &before.design) {
        assert!(std::sync::Arc::ptr_eq(kept, original));
    }
}

#[test]
fn read_replaces_only_after_success_and_writer_history_solves_the_same() {
    let dir = folder();
    let path = dir.join("original.inp");
    let mut s = bar();
    let original = s.execute("solve.static", &json!({"step":"Bend"})).unwrap();
    s.execute("deck.write", &json!({"format":"abaqus","path":path})).unwrap();
    s.path = Some(dir.join("previous.fieldcraft").to_string_lossy().into_owned());
    let result = s.execute("deck.read", &json!({"format":"abaqus","path":path,"units":"mm-t-s-N-MPa"})).unwrap();
    assert_eq!(result["nodes"], 12);
    assert_eq!(result["elements"]["hex8"], 2);
    assert_eq!(result["steps"], 1);
    assert_eq!(result["boundary_conditions"], 1);
    assert_eq!(result["loads"], 1);
    assert_eq!(result["units"], "mm-t-s-N-MPa");
    assert_eq!(result["history_imported"], true);
    assert!(!s.model.steps().is_empty() && !s.model.loads().is_empty() && !s.model.boundary_conditions().is_empty());
    assert_eq!(s.undo_depth(), (0, 0));
    assert!(s.journal.is_empty() && s.path.is_none() && s.result("Bend").is_none() && s.is_dirty());
    assert!(s.design.is_none() && s.surfaces.is_empty());
    let resumed = s.execute("solve.static", &json!({"step":"Bend"})).unwrap();
    let before = original["max_displacement"]["value"].as_f64().unwrap();
    let after = resumed["max_displacement"]["value"].as_f64().unwrap();
    assert!((before - after).abs() <= before.abs() * 1e-12, "{before} vs {after}");
    assert_eq!(original["max_displacement"]["node"], resumed["max_displacement"]["node"]);
}

#[test]
fn writer_roundtrips_fixed_and_symmetry_conditions_with_shared_nodes() {
    let dir = folder();
    let path = dir.join("shared-boundary.inp");
    let mut s = bar();
    s.execute("model.boundary_condition.create", &json!({"name":"sym","type":"symmetry","target":{"set":"all"},"plane":"x"})).unwrap();
    s.execute("model.step.activate", &json!({"step":"Bend","boundary_conditions":["sym"]})).unwrap();
    let original = s.execute("solve.static", &json!({"step":"Bend"})).unwrap();
    s.execute("deck.write", &json!({"format":"abaqus","path":path,"units":"mm-t-s-N-MPa"})).unwrap();
    s.execute("deck.read", &json!({"format":"abaqus","path":path,"units":"mm-t-s-N-MPa"})).unwrap();
    assert_eq!(s.model.steps().len(), 1);
    assert_eq!(s.model.steps()[0].boundary_conditions.len(), 2);
    assert!(
        s.model
            .boundary_conditions()
            .values()
            .any(|bc| matches!(bc, fieldcraft_model::BoundaryCondition::Fixed { target: fieldcraft_model::Target::Set(set) } if set == "root"))
    );
    assert!(s.model.boundary_conditions().values().any(|bc| matches!(bc, fieldcraft_model::BoundaryCondition::Symmetry { target: fieldcraft_model::Target::Set(set), plane: fieldcraft_model::Symmetry::X } if set == "all")));
    let resumed = s.execute("solve.static", &json!({"step":"Bend"})).unwrap();
    let before = original["max_displacement"]["value"].as_f64().unwrap();
    let after = resumed["max_displacement"]["value"].as_f64().unwrap();
    assert!((before - after).abs() <= before.abs() * 1e-12, "{before} vs {after}");
    assert_eq!(original["max_displacement"]["node"], resumed["max_displacement"]["node"]);
}

#[test]
fn command_built_four_node_total_force_roundtrips_through_multiple_steps() {
    let dir = folder();
    let path = dir.join("multi-step.inp");
    let mut s = bar();
    s.execute("model.step.create", &json!({"name":"Hold","procedure":{"type":"static"},"boundary_conditions":["clamp"],"loads":["push"]})).unwrap();
    s.execute("model.step.create", &json!({"name":"Modes","procedure":{"type":"frequency","modes":2},"boundary_conditions":["clamp"]})).unwrap();
    s.execute("deck.write", &json!({"format":"abaqus","path":path,"units":"mm-t-s-N-MPa"})).unwrap();
    s.execute("deck.read", &json!({"format":"abaqus","path":path,"units":"mm-t-s-N-MPa"})).unwrap();
    assert_eq!(s.model.steps().iter().map(|step| step.name.as_str()).collect::<Vec<_>>(), ["Bend", "Hold", "Modes"]);
    assert_eq!(s.model.boundary_conditions().len(), 1);
    assert_eq!(s.model.loads().len(), 1);
    let Some(fieldcraft_model::Load::Force {
        target: fieldcraft_model::Target::Set(set),
        vector,
        distribution: fieldcraft_model::Distribution::PerNode,
    }) = s.model.loads().values().next()
    else {
        panic!("{:?}", s.model.loads());
    };
    assert_eq!(set, "end");
    assert_eq!(*vector, [0.0, 0.0, -25.0]);
    assert_eq!(s.model.set(set).map(|set| set.nodes.len()), Some(4));
    assert_eq!(s.model.steps()[0].loads, s.model.steps()[1].loads);
    assert!(s.execute("solve.static", &json!({"step":"Hold"})).is_ok());
}

#[test]
fn every_read_error_keeps_model_results_path_undo_and_unsaved_work() {
    let dir = folder();
    let path = dir.join("bad.inp");
    let mut s = bar();
    s.execute("solve.static", &json!({"step":"Bend"})).unwrap();
    s.execute("file.save_as", &json!({"path":dir.join("saved.fieldcraft"),"results":true})).unwrap();
    s.execute("mesh.node.add", &json!({"label":99,"xyz":["5 mm","0 mm","0 mm"]})).unwrap();
    assert!(s.is_dirty());
    let before = s.clone();
    for text in [
        "",
        "*HEADING\nTitle\n",
        "*UNKNOWN\ndata\n",
        "*NSET,NSET=Empty\n",
        "*NODE\n0,0,0,0",
        "*NODE\n1,0,0,0\n*ELEMENT,TYPE=C3D4\n1,1,2,3,4",
        "*INCLUDE,INPUT=secret.inp",
        "*STEP\n*NODE\n9,1,2,3",
    ] {
        std::fs::write(&path, text).unwrap();
        assert!(s.execute("deck.read", &json!({"format":"abaqus","path":path})).is_err(), "{text}");
        assert_unchanged(&s, &before);
    }
    std::fs::write(&path, [0xff, 0xfe]).unwrap();
    for params in [
        json!({"format":"abaqus","path":path}),
        json!({"format":"nastran","path":path}),
        json!({"format":"abaqus"}),
        json!({"format":"abaqus","path":dir}),
        json!({"format":"abaqus","path":"/dev/zero"}),
        json!({"format":"abaqus","path":"x".repeat(100_000)}),
        json!({"format":"abaqus","path":path,"units":"guess"}),
    ] {
        assert!(s.execute("deck.read", &params).is_err());
        assert_unchanged(&s, &before);
    }
    // The existing undo stack is still usable, not merely the same depth.
    s.execute("edit.undo", &json!({})).unwrap();
    assert!(s.model.node(99).is_none());
}

#[test]
fn cooperative_read_cancellation_preserves_the_whole_previous_session() {
    let dir = folder();
    let path = dir.join("original.inp");
    let mut s = bar();
    s.execute("solve.static", &json!({"step":"Bend"})).unwrap();
    s.execute("deck.write", &json!({"format":"abaqus","path":path})).unwrap();
    s.path = Some(dir.join("prior.fieldcraft").to_string_lossy().into_owned());
    let before = s.clone();
    let progress = fieldcraft_model::progress::Progress::new();
    progress.cancel();
    let error =
        fieldcraft_model::progress::scoped(Some(progress), || s.execute("deck.read", &json!({"format":"abaqus","path":path,"units":"mm-t-s-N-MPa"})))
            .unwrap_err();
    assert!(error.is_cancelled(), "{error}");
    assert_unchanged(&s, &before);
    // Neither cancellation nor its reporter poisons the next ordinary import.
    assert!(s.execute("deck.read", &json!({"format":"abaqus","path":path})).is_ok());
}

#[test]
fn read_roots_default_units_and_unknown_output_are_explicit() {
    let dir = folder();
    let allowed = dir.join("allowed");
    std::fs::create_dir(&allowed).unwrap();
    let path = allowed.join("mesh.inp");
    let outside = dir.join("outside.inp");
    let text = format!("*NODE\n1,1,0,0\n{}", "*IGNORED\n".repeat(120));
    std::fs::write(&path, &text).unwrap();
    std::fs::write(&outside, &text).unwrap();
    let mut s = bar();
    s.execute("model.units", &json!({"system":"in-lbf-s-psi","overrides":{"length":"mm"}})).unwrap();
    s.file_access = fieldcraft_io::roots::Roots::new(Some(&allowed), None).unwrap();
    let before = s.clone();
    assert!(s.execute("deck.read", &json!({"format":"abaqus","path":outside})).is_err());
    assert_unchanged(&s, &before);
    let reply = s.execute("deck.read", &json!({"format":"abaqus","path":path})).unwrap();
    assert_eq!(reply["units"], "in-lbf-s-psi");
    assert_eq!(reply["unknown_count"], 120);
    assert_eq!(reply["unknown"].as_array().unwrap().len(), 100);
    let inch = fieldcraft_model::units::Quantity::Length.parse("1 in", crate::units::si()).unwrap();
    assert_eq!(s.model.node(1).unwrap().xyz, [inch, 0., 0.]);
    assert_eq!(s.file_access, before.file_access);
}

#[test]
fn the_session_import_caps_bound_the_read() {
    let dir = folder();
    let path = dir.join("original.inp");
    let mut s = bar();
    s.execute("deck.write", &json!({"format":"abaqus","path":path})).unwrap();
    for limits in [
        crate::Limits { max_import_nodes: 11, ..crate::Limits::default() },
        crate::Limits { max_import_elements: 1, ..crate::Limits::default() },
        crate::Limits { max_import_members: 4, ..crate::Limits::default() },
    ] {
        s.limits = limits;
        let before = s.clone();
        let error = s.execute("deck.read", &json!({"format":"abaqus","path":path,"units":"mm-t-s-N-MPa"})).unwrap_err().to_string();
        assert!(error.contains("limit"), "{error}");
        assert_unchanged(&s, &before);
    }
    s.limits = crate::Limits { max_import_nodes: 12, max_import_elements: 2, ..crate::Limits::default() };
    assert!(s.execute("deck.read", &json!({"format":"abaqus","path":path,"units":"mm-t-s-N-MPa"})).is_ok());
    assert_eq!(s.limits.max_import_nodes, 12, "the new document keeps the session's caps");
}

fn bar_full_hex8() -> Session {
    let mut s = Session::default();
    super::super::define::tests::tagged_bar(&mut s);
    s.run_script(&json!([
        {"command":"model.units","params":{"system":"mm-t-s-N-MPa"}},
        {"command":"model.set.create","params":{"name":"all","geometry":[{"body":"Bar"}]}},
        {"command":"model.set.create","params":{"name":"root","geometry":[{"body":"Bar","face":"left"}]}},
        {"command":"model.set.create","params":{"name":"end","geometry":[{"body":"Bar","face":"right"}]}},
        {"command":"model.material.create","params":{"name":"Steel","elastic":{"type":"isotropic","young":"210 GPa","poisson":0.3},"density":"7850 kg/m^3"}},
        {"command":"model.section.create","params":{"name":"BarSection","set":"all","material":"Steel","kind":{"type":"solid"}}}
    ])).unwrap();
    constrain(&mut s);
    s
}

#[test]
fn warp3d_write_then_read_round_trips_and_solves_identically_after_reconstraining() {
    let dir = folder();
    let path = dir.join("bar_warp3d.inp");
    let mut s = bar_full_hex8();
    let original = s.execute("solve.static", &json!({"step":"Bend"})).unwrap();
    s.execute("deck.write", &json!({"format":"warp3d","path":path})).unwrap();
    s.path = Some(dir.join("previous.fieldcraft").to_string_lossy().into_owned());

    let result = s.execute("deck.read", &json!({"format":"warp3d","path":path,"units":null})).unwrap();
    assert_eq!(result["format"], "warp3d");
    assert_eq!(result["units"], "mm-t-s-N-MPa");
    assert_eq!(result["units_source"], "header");
    assert_eq!(result["nodes"], 12);
    assert_eq!(result["elements"]["hex8"], 2);
    assert_eq!(result["boundary_conditions"], 1);
    assert_eq!(result["loads"], 1);
    assert_eq!(result["includes_read"], json!([]));
    assert_eq!(result["history_imported"], false);
    assert_eq!(result["stopped_at"]["reason"], "compute");
    assert_eq!(result["stopped_at"]["file"], "bar_warp3d.inp");
    assert!(result["stopped_at"]["line"].as_u64().unwrap() > 0);
    assert!(result["unknown"].as_array().unwrap().iter().any(|u| u["keyword"] == "compute" && u.get("file").is_none()));
    assert!(s.model.steps().is_empty());
    assert_eq!(s.model.boundary_conditions().len(), 1);
    assert_eq!(s.model.loads().len(), 1);
    assert_eq!(s.undo_depth(), (0, 0));
    assert!(s.journal.is_empty() && s.path.is_none() && s.result("Bend").is_none() && s.is_dirty());

    let bcs: Vec<String> = s.model.boundary_conditions().keys().cloned().collect();
    let lds: Vec<String> = s.model.loads().keys().cloned().collect();
    s.execute("model.step.create", &json!({"name":"Bend","procedure":{"type":"static"},"boundary_conditions":bcs,"loads":lds})).unwrap();
    let resumed = s.execute("solve.static", &json!({"step":"Bend"})).unwrap();
    let before = original["max_displacement"]["value"].as_f64().unwrap();
    let after = resumed["max_displacement"]["value"].as_f64().unwrap();
    assert!((before - after).abs() <= before.abs() * 1e-7, "{before} vs {after}");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn warp3d_units_precedence_null_empty_refusal_and_header_validation() {
    let dir = folder();
    let mut s = bar_full_hex8();
    s.execute("model.units", &json!({"system":"in-lbf-s-psi","overrides":{"length":"mm"}})).unwrap();
    let before = s.clone();

    // 1. Explicit units > Header > Session, and `"units": null` acts as omitted for both formats.
    let header_path = dir.join("with_header.inp");
    std::fs::write(
        &header_path,
        "! fieldcraft-units: m-kg-s-N-Pa\nmaterial S\n  properties linear e 2e11 nu 0.3\ncoordinates\n  1 2.0 0.0 0.0\n  2 3.0 0.0 0.0\n  3 2.0 1.0 0.0\n  4 2.0 0.0 1.0\nincidences\n  1 1 2 3 4\nelements\n  1 type tet4 linear material S\n",
    )
    .unwrap();
    let abaqus_null_path = dir.join("abaqus_null.inp");
    std::fs::write(&abaqus_null_path, "*NODE\n1,1,0,0\n").unwrap();

    // Explicit overrides header.
    let mut s_exp = bar_full_hex8();
    let r_exp = s_exp.execute("deck.read", &json!({"format":"warp3d","path":header_path,"units":"mm-t-s-N-MPa"})).unwrap();
    assert_eq!(r_exp["units"], "mm-t-s-N-MPa");
    assert_eq!(r_exp["units_source"], "explicit");
    assert!((s_exp.model.node(1).unwrap().xyz[0] - 0.002).abs() < 1e-12);

    // `units: null` auto-detects header.
    let mut s_hdr = bar_full_hex8();
    let r_hdr = s_hdr.execute("deck.read", &json!({"format":"warp3d","path":header_path,"units":null})).unwrap();
    assert_eq!(r_hdr["units"], "m-kg-s-N-Pa");
    assert_eq!(r_hdr["units_source"], "header");
    assert!((s_hdr.model.node(1).unwrap().xyz[0] - 2.0).abs() < 1e-12);
    assert!(r_hdr.get("unknown").is_some());

    // `units: null` on Abaqus falls back to session preset without leaking WARP3D-only fields.
    let mut s_abq = bar_full_hex8();
    s_abq.execute("model.units", &json!({"system":"in-lbf-s-psi"})).unwrap();
    let r_abq = s_abq.execute("deck.read", &json!({"format":"abaqus","path":abaqus_null_path,"units":null})).unwrap();
    assert_eq!(r_abq["units"], "in-lbf-s-psi");
    assert!(r_abq.get("units_source").is_none());
    assert!(r_abq.get("stopped_at").is_none());
    assert!(r_abq.get("includes_read").is_none());

    // 2. Headerless WARP3D falls back to session display preset (ignoring overrides) and warns.
    let headerless = dir.join("headerless.inp");
    std::fs::write(&headerless, "coordinates\n  1 1.0 0.0 0.0\n").unwrap();
    let r_sess = s.execute("deck.read", &json!({"format":"warp3d","path":headerless})).unwrap();
    assert_eq!(r_sess["units"], "in-lbf-s-psi");
    assert_eq!(r_sess["units_source"], "session");
    assert!(r_sess["warnings"].as_array().unwrap().iter().any(|w| w.as_str().unwrap_or("").contains("in-lbf-s-psi")));
    let inch = fieldcraft_model::units::Quantity::Length.parse("1 in", crate::units::si()).unwrap();
    assert_eq!(s.model.node(1).unwrap().xyz, [inch, 0.0, 0.0]);

    // 3. Malformed header & empty-model deck leave prior session untouched.
    let mut s_guard = before.clone();
    let bad_hdr = dir.join("bad_hdr.inp");
    std::fs::write(&bad_hdr, "! fieldcraft-units: nope\ncoordinates\n  1 0 0 0\n").unwrap();
    let err = s_guard.execute("deck.read", &json!({"format":"warp3d","path":bad_hdr})).unwrap_err();
    assert!(matches!(err, EngineError::Params(_)), "{err:?}");
    assert_unchanged(&s_guard, &before);

    let empty_deck = dir.join("empty.inp");
    std::fs::write(&empty_deck, "structure empty\nblocking automatic\ncompute displacements\n").unwrap();
    let err = s_guard.execute("deck.read", &json!({"format":"warp3d","path":empty_deck})).unwrap_err();
    assert!(matches!(&err, EngineError::Param(p) if p.path == "path"), "{err:?}");
    assert_unchanged(&s_guard, &before);

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn warp3d_comments_encoding_errors_and_cancellation() {
    let dir = folder();
    let mut s = bar_full_hex8();
    s.execute("solve.static", &json!({"step":"Bend"})).unwrap();
    let before = s.clone();

    // Latin-1 inside col-1 & `$` comments succeeds; active Latin-1 fails with Params (not Param).
    let latin1_ok = dir.join("latin1_ok.inp");
    let mut bytes = Vec::new();
    bytes.extend_from_slice(b"! Latin-1 comment \xff\xfe\n");
    bytes.extend_from_slice(b"! fieldcraft-units: m-kg-s-N-Pa\n");
    bytes.extend_from_slice(b"coordinates $ post-dollar \xff\n  1 1.0 2.0 3.0\n");
    std::fs::write(&latin1_ok, &bytes).unwrap();
    assert!(s.execute("deck.read", &json!({"format":"warp3d","path":latin1_ok})).is_ok());

    let mut s = before.clone();
    let latin1_bad = dir.join("latin1_bad.inp");
    std::fs::write(&latin1_bad, b"coordinates\n  1 0.0 0.0 \xff\n").unwrap();
    let err = s.execute("deck.read", &json!({"format":"warp3d","path":latin1_bad})).unwrap_err();
    assert!(matches!(err, EngineError::Params(_)), "{err:?}");
    assert_unchanged(&s, &before);

    // Unsupported feature maps to EngineError::Deck(Unsupported { format: "warp3d", .. }), not Param.
    let unsup = dir.join("unsup.inp");
    std::fs::write(&unsup, "restart\n").unwrap();
    let err = s.execute("deck.read", &json!({"format":"warp3d","path":unsup})).unwrap_err();
    assert!(matches!(err, EngineError::Deck(fieldcraft_decks::Error::Unsupported { format: "warp3d", .. })), "{err:?}");
    assert_unchanged(&s, &before);

    // Limit maps to EngineError::Deck(Limit(_)).
    s.limits.max_import_members = 2;
    let bomb = dir.join("bomb.inp");
    std::fs::write(&bomb, "list n 1-100\n").unwrap();
    let err = s.execute("deck.read", &json!({"format":"warp3d","path":bomb})).unwrap_err();
    assert!(matches!(err, EngineError::Deck(fieldcraft_decks::Error::Limit(_))), "{err:?}");
    s.limits = before.limits;
    assert_unchanged(&s, &before);

    // `EngineError::Param` only for `format`, `path`, `units`.
    for (expected_key, bad_params) in [
        ("format", json!({"format":"nope","path":latin1_ok})),
        ("path", json!({"format":"warp3d","path":""})),
        ("units", json!({"format":"warp3d","path":latin1_ok,"units":"bad"})),
        ("units", json!({"format":"warp3d","path":latin1_ok,"units":42})),
    ] {
        let err = s.execute("deck.read", &bad_params).unwrap_err();
        assert!(matches!(&err, EngineError::Param(p) if p.path == expected_key), "{expected_key}: {err:?}");
        assert_unchanged(&s, &before);
    }

    // Cooperative cancellation preserves prior session.
    let mut big = String::from("! fieldcraft-units: m-kg-s-N-Pa\ncoordinates\n");
    for i in 1..=600 {
        big.push_str(&format!("  {i} 0.0 0.0 0.0\n"));
    }
    let cancel_path = dir.join("cancel.inp");
    std::fs::write(&cancel_path, &big).unwrap();
    let progress = fieldcraft_model::progress::Progress::new();
    progress.cancel();
    let err =
        fieldcraft_model::progress::scoped(Some(progress), || s.execute("deck.read", &json!({"format":"warp3d","path":cancel_path}))).unwrap_err();
    assert!(err.is_cancelled(), "{err}");
    assert_unchanged(&s, &before);

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn warp3d_includes_macros_bcs_loads_and_resolve() {
    let dir = folder();
    std::fs::create_dir_all(dir.join("sub")).unwrap();
    std::fs::write(dir.join("params.inp"), "*parameter E0 = '2.0e11' Y0 = '2.0e8' NEXP = '5'\n").unwrap();
    std::fs::write(
        dir.join("sub/mesh.inp"),
        "coordinates\n  1 0 0 0\n  2 1 0 0\n  3 1 1 0\n  4 0 1 0\n  5 0 0 1\n  6 1 0 1\n  7 1 1 1\n  8 0 1 1\nincidences\n  1 1 2 3 4 5 6 7 8\n",
    )
    .unwrap();
    std::fs::write(dir.join("bcs_loads.inp"), "constraints\n  plane x=0 fixed\nloading p\n  element loads\n    1 face 4 pressure 100.0\n").unwrap();

    let root = dir.join("root");
    std::fs::write(
        &root,
        "! fieldcraft-units: m-kg-s-N-Pa\n*input from 'params.inp'\nmaterial M\n  properties mises e @E0 nu 0.3 yld_pt @Y0 n_power @NEXP\nelements\n  1 type l3disop linear material M\n*input from 'sub/mesh.inp'\n*input 'bcs_loads.inp\ncompute displacements\n*input 'post_compute_missing.inp'\n",
    )
    .unwrap();

    let mut s = Session::default();
    let reply = s.execute("deck.read", &json!({"format":"warp3d","path":root})).unwrap();
    assert_eq!(reply["boundary_conditions"], 1);
    assert_eq!(reply["loads"], 1);
    assert_eq!(reply["includes_read"], json!(["params.inp", "sub/mesh.inp", "bcs_loads.inp"]));

    let bcs: Vec<String> = s.model.boundary_conditions().keys().cloned().collect();
    let lds: Vec<String> = s.model.loads().keys().cloned().collect();
    s.execute("model.step.create", &json!({"name":"S1","procedure":{"type":"static"},"boundary_conditions":bcs,"loads":lds})).unwrap();
    assert!(s.model.resolve().is_ok(), "{:?}", s.model.check());

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn warp3d_include_confinement_redacts_host_paths_and_keeps_session() {
    let dir = folder();
    let deck_dir = dir.join("deck");
    let outside_dir = dir.join("outside");
    std::fs::create_dir_all(&deck_dir).unwrap();
    std::fs::create_dir_all(&outside_dir).unwrap();
    std::fs::write(outside_dir.join("secret.inp"), "coordinates\n  1 0 0 0\n").unwrap();

    let mut s = bar_full_hex8();
    s.execute("solve.static", &json!({"step":"Bend"})).unwrap();
    let before = s.clone();
    let dir_str = dir.to_string_lossy().into_owned();

    // Unsafe lexical targets, Windows device names, ADS, and `~/`
    for bad_inc in ["../outside/secret.inp", "~/secret.inp", "/etc/passwd", "CON", "NUL.inp", "sub.inp:stream"] {
        let root = deck_dir.join("root.inp");
        std::fs::write(&root, format!("*input '{bad_inc}'\n")).unwrap();
        let err = s.execute("deck.read", &json!({"format":"warp3d","path":root})).unwrap_err();
        assert!(matches!(err, EngineError::File(fieldcraft_io::Error::Refused(_))), "{bad_inc}: {err:?}");
        assert!(!err.to_string().contains(&dir_str), "host path leaked for `{bad_inc}`: {err}");
        assert_unchanged(&s, &before);
    }

    // Cycle detection -> Refused
    std::fs::write(deck_dir.join("a.inp"), "*input 'b.inp'\n").unwrap();
    std::fs::write(deck_dir.join("b.inp"), "*input 'a.inp'\n").unwrap();
    let cyc = deck_dir.join("cyc.inp");
    std::fs::write(&cyc, "*input 'a.inp'\n").unwrap();
    let err = s.execute("deck.read", &json!({"format":"warp3d","path":cyc})).unwrap_err();
    assert!(matches!(err, EngineError::File(fieldcraft_io::Error::Refused(_))), "{err:?}");
    assert!(!err.to_string().contains(&dir_str), "host path leaked in cycle error: {err}");
    assert_unchanged(&s, &before);

    // Symlink escape inside `deck_dir` pointing to `outside_dir/secret.inp`
    #[cfg(unix)]
    {
        std::os::unix::fs::symlink(outside_dir.join("secret.inp"), deck_dir.join("sym.inp")).unwrap();
        let sym_root = deck_dir.join("sym_root.inp");
        std::fs::write(&sym_root, "*input 'sym.inp'\n").unwrap();
        let err = s.execute("deck.read", &json!({"format":"warp3d","path":sym_root})).unwrap_err();
        assert!(matches!(err, EngineError::File(fieldcraft_io::Error::Refused(_))), "{err:?}");
        assert!(!err.to_string().contains(&dir_str), "host path leaked in symlink error: {err}");
        assert_unchanged(&s, &before);
    }

    // Confined `s.file_access = Roots::new(Some(&deck_dir), None)` allows relative include inside `deck_dir`.
    let sub_dir = deck_dir.join("sub");
    std::fs::create_dir_all(&sub_dir).unwrap();
    std::fs::write(sub_dir.join("ok.inp"), "coordinates\n  1 0 0 0\n").unwrap();
    let ok_root = deck_dir.join("ok_confined.inp");
    std::fs::write(&ok_root, "! fieldcraft-units: m-kg-s-N-Pa\n*input 'sub/ok.inp'\n").unwrap();
    let mut s_conf = before.clone();
    s_conf.file_access = fieldcraft_io::roots::Roots::new(Some(&deck_dir), None).unwrap();
    assert!(s_conf.execute("deck.read", &json!({"format":"warp3d","path":ok_root})).is_ok());

    // Cumulative 64 MiB cap across root + include: 33 MiB root + 33 MiB include -> Limit without leaking host path.
    let big_inc = deck_dir.join("big_inc.inp");
    let big_root = deck_dir.join("big_root.inp");
    let pad = vec![b'!'; 33 * 1024 * 1024];
    let mut inc_bytes = pad.clone();
    inc_bytes.extend_from_slice(b"\ncoordinates\n  1 0 0 0\n");
    std::fs::write(&big_inc, &inc_bytes).unwrap();
    let mut root_bytes = pad;
    root_bytes.extend_from_slice(b"\n! fieldcraft-units: m-kg-s-N-Pa\n*input 'big_inc.inp'\n");
    std::fs::write(&big_root, &root_bytes).unwrap();
    let err = s.execute("deck.read", &json!({"format":"warp3d","path":big_root})).unwrap_err();
    assert!(matches!(err, EngineError::Deck(fieldcraft_decks::Error::Limit(_))), "{err:?}");
    assert!(!err.to_string().contains(&dir_str), "host path leaked in limit error: {err}");
    assert_unchanged(&s, &before);

    // Bare-filename `deck_dir_for` fallback to `"."` and `wasm_include_refused`:
    assert_eq!(super::deck_dir_for(Path::new("root.inp")), Path::new("."));
    let wasm_err = super::wasm_include_refused("sub.inp").unwrap_err();
    assert!(matches!(wasm_err, fieldcraft_warp3d::reader::ReadError::IncludeRefused(_)), "{wasm_err:?}");

    let _ = std::fs::remove_dir_all(&dir);
}

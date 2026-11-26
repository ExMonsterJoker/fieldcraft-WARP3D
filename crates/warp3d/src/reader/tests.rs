use std::sync::Arc;

use fieldcraft_model::progress::{self, Progress};
use fieldcraft_model::units::Preset;
use fieldcraft_model::{
    BoundaryCondition, Elastic, ElementKind, Hardening, HardeningPoint, Integration, Load, Material, Model, Plastic, Procedure, Section, SectionKind,
    Set, Step, Symmetry, Target,
};

use super::*;
use crate::deck::write_deck;

fn hex8_cantilever_model() -> Model {
    let mut model = Model::default();
    let coords = [
        (1, [0.0, 0.0, 0.0]),
        (2, [0.1, 0.0, 0.0]),
        (3, [0.1, 0.02, 0.0]),
        (4, [0.0, 0.02, 0.0]),
        (5, [0.0, 0.0, 0.02]),
        (6, [0.1, 0.0, 0.02]),
        (7, [0.1, 0.02, 0.02]),
        (8, [0.0, 0.02, 0.02]),
    ];
    for (id, xyz) in coords {
        model.set_node(id, xyz).unwrap();
    }
    model.set_element(1, ElementKind::Hex8, vec![1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
    model
        .add_material(
            "Steel",
            Material { elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }), density: Some(7850.0), ..Default::default() },
        )
        .unwrap();
    model.add_set("all", Set { elements: vec![1], ..Set::default() }).unwrap();
    model.add_set("fix", Set { nodes: vec![1, 4, 5, 8], ..Set::default() }).unwrap();
    model.add_set("tip", Set { nodes: vec![2, 3, 6, 7], ..Set::default() }).unwrap();
    model
        .add_section("solid", Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() })
        .unwrap();
    model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("fix".into()) }).unwrap();
    model.add_load("pull", Load::Force { target: Target::Set("tip".into()), vector: [1000.0, 0.0, 0.0], distribution: Default::default() }).unwrap();
    let mut step = Step::new("Pull", Procedure::Static {});
    step.boundary_conditions.insert("clamp".into());
    step.loads.insert("pull".into());
    model.add_step(step, None).unwrap();
    model
}

fn rel_close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * a.abs().max(b.abs()).max(1.0)
}

#[test]
fn roundtrip_write_deck_hex8_full_and_reduced_all_presets() {
    for preset in Preset::ALL {
        for integration in [Integration::Full, Integration::Reduced] {
            let mut model = hex8_cantilever_model();
            let mut sec = model.section("solid").unwrap().clone();
            sec.formulation.integration = integration;
            model.replace_section("solid", sec).unwrap();

            let written = write_deck(&model, None, None, preset).unwrap();
            let imported = read_limited("roundtrip.inp", written.text.as_bytes(), None, Preset::MKgS, ReadLimits::default()).unwrap();

            assert_eq!(imported.units, preset);
            assert_eq!(imported.units_source, UnitsSource::Header);
            assert!(imported.stopped_at.is_some());
            assert_eq!(imported.stopped_at.as_ref().unwrap().reason, "compute");
            assert_eq!(imported.model.nodes().len(), 8);
            assert_eq!(imported.model.elements().len(), 1);
            assert_eq!(imported.model.element(1).unwrap().kind, ElementKind::Hex8);
            for (nid, orig) in model.nodes() {
                let got = imported.model.node(*nid).unwrap();
                for i in 0..3 {
                    assert!(rel_close(got.xyz[i], orig.xyz[i], 1e-8), "{preset:?} node {nid}[{i}]: {} vs {}", got.xyz[i], orig.xyz[i]);
                }
            }
            let mat = imported.model.material("Steel").unwrap();
            match mat.elastic {
                Some(Elastic::Isotropic { young, poisson }) => {
                    assert!(rel_close(young, 2.1e11, 1e-7), "{preset:?}: {young}");
                    assert!(rel_close(poisson, 0.3, 1e-7), "{preset:?}: {poisson}");
                }
                other => panic!("unexpected elastic: {other:?}"),
            }
            assert!(mat.plastic.is_none(), "n_power=1.0 should map to elastic");
            let imp_sec = imported.model.sections().values().next().unwrap();
            assert_eq!(imp_sec.formulation.integration, integration);
        }
    }
}

#[test]
fn roundtrip_hex20_tet4_tet10_bilinear_and_table_mises() {
    let mut model = Model::default();
    let hex_corners =
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0]];
    let mut hex_xyz = hex_corners.to_vec();
    hex_xyz.resize(20, [0.0; 3]);
    for e in ElementKind::Hex20.edges() {
        if let [a, b, m] = **e {
            hex_xyz[m] = [0, 1, 2].map(|i| 0.5 * (hex_corners[a][i] + hex_corners[b][i]));
        }
    }
    for (i, xyz) in hex_xyz.iter().enumerate() {
        model.set_node((i + 1) as u32, *xyz).unwrap();
    }
    model.set_element(10, ElementKind::Hex20, (1..=20).collect()).unwrap();

    // Tet4 nodes 21..=24
    for (i, xyz) in [[2.0, 0.0, 0.0], [3.0, 0.0, 0.0], [2.0, 1.0, 0.0], [2.0, 0.0, 1.0]].iter().enumerate() {
        model.set_node((21 + i) as u32, *xyz).unwrap();
    }
    model.set_element(20, ElementKind::Tet4, vec![21, 22, 23, 24]).unwrap();

    // Tet10 nodes 25..=34
    let tet_corners = [[4.0, 0.0, 0.0], [5.0, 0.0, 0.0], [4.0, 1.0, 0.0], [4.0, 0.0, 1.0]];
    let mut tet_xyz = tet_corners.to_vec();
    tet_xyz.resize(10, [0.0; 3]);
    for e in ElementKind::Tet10.edges() {
        if let [a, b, m] = **e {
            tet_xyz[m] = [0, 1, 2].map(|i| 0.5 * (tet_corners[a][i] + tet_corners[b][i]));
        }
    }
    for (i, xyz) in tet_xyz.iter().enumerate() {
        model.set_node((25 + i) as u32, *xyz).unwrap();
    }
    model.set_element(30, ElementKind::Tet10, (25..=34).collect()).unwrap();

    model
        .add_material(
            "BiMat",
            Material {
                elastic: Some(Elastic::Isotropic { young: 2.0e11, poisson: 0.3 }),
                plastic: Some(Plastic::Bilinear { yield_stress: 2.5e8, hardening_modulus: 2.0e9, hardening: Hardening::Isotropic }),
                ..Default::default()
            },
        )
        .unwrap();
    model
        .add_material(
            "TabMat",
            Material {
                elastic: Some(Elastic::Isotropic { young: 2.0e11, poisson: 0.29 }),
                plastic: Some(Plastic::Table {
                    points: vec![HardeningPoint { strain: 0.0, stress: 3.0e8 }, HardeningPoint { strain: 0.05, stress: 4.5e8 }],
                }),
                ..Default::default()
            },
        )
        .unwrap();

    model.add_set("s_hex20", Set { elements: vec![10], ..Set::default() }).unwrap();
    model.add_set("s_tets", Set { elements: vec![20, 30], ..Set::default() }).unwrap();
    model.add_set("fix", Set { nodes: vec![1, 4, 5, 8, 21, 23, 24, 25, 27, 28], ..Set::default() }).unwrap();
    model
        .add_section(
            "sec1",
            Section { set: "s_hex20".into(), material: "BiMat".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
        )
        .unwrap();
    model
        .add_section(
            "sec2",
            Section { set: "s_tets".into(), material: "TabMat".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
        )
        .unwrap();
    model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("fix".into()) }).unwrap();
    let mut step = Step::new("Load", Procedure::Static {});
    step.boundary_conditions.insert("clamp".into());
    model.add_step(step, None).unwrap();

    let written = write_deck(&model, None, None, Preset::InLbfS).unwrap();
    let imported = read_limited("multi.inp", written.text.as_bytes(), None, Preset::MmTonneS, ReadLimits::default()).unwrap();

    assert_eq!(imported.units, Preset::InLbfS);
    assert_eq!(imported.model.nodes().len(), 34);
    assert_eq!(imported.model.elements().len(), 3);
    assert_eq!(imported.model.element(1).unwrap().kind, ElementKind::Hex20);
    assert_eq!(imported.model.element(2).unwrap().kind, ElementKind::Tet4);
    assert_eq!(imported.model.element(3).unwrap().kind, ElementKind::Tet10);

    match imported.model.material("BiMat").unwrap().plastic.as_ref().unwrap() {
        Plastic::Bilinear { yield_stress, hardening_modulus, hardening } => {
            assert!(rel_close(*yield_stress, 2.5e8, 1e-6));
            assert!(rel_close(*hardening_modulus, 2.0e9, 1e-6));
            assert_eq!(*hardening, Hardening::Isotropic);
        }
        other => panic!("unexpected BiMat plastic: {other:?}"),
    }
    match imported.model.material("TabMat").unwrap().plastic.as_ref().unwrap() {
        Plastic::Table { points } => {
            assert_eq!(points.len(), 2);
            assert_eq!(points[0].strain, 0.0);
            assert!(rel_close(points[0].stress, 3.0e8, 1e-6));
            assert!((points[1].strain - 0.05).abs() < 1e-7);
            assert!(rel_close(points[1].stress, 4.5e8, 1e-6));
        }
        other => panic!("unexpected TabMat plastic: {other:?}"),
    }
}

#[test]
fn explicit_list_scoping_and_material_order_independence() {
    let deck = "\
! fieldcraft-units: m-kg-s-N-Pa
material Steel
  properties linear e 2.0e11 nu 0.3 rho 7850.0 alpha 1.2e-5
structure plate
stress-strain curve 1 smooth
  0.0 0.0
  0.0005 1.0e8
  0.0010 2.0e8
  0.0210 3.0e8
material CurveMat
  properties mises e = 2.0e11 nu = 0.3 curve = 1 sig_o = 2.0e8 beta = 1.0
list 'front_nodes' 1-4
list \"elem_group\" 1
elements
  elem_group type tet4 linear material CurveMat order 1pt_rule
  all type tet4 linear material Steel order 4pt
coordinates
  1 0.0 0.0 0.0
  2 1.0 0.0 0.0
  3 0.0 1.0 0.0
  4 0.0 0.0 1.0
incidences
  1 1 2 3 4
stop
";
    let imported = read_limited("order.inp", deck.as_bytes(), None, Preset::MmTonneS, ReadLimits::default()).unwrap();
    assert!(imported.stopped_at.is_none());
    assert_eq!(imported.unknown_count, 0);
    // `elem_group` was referenced by `elements` so must NOT create a bogus node Set.
    assert!(imported.model.set("ls_elem_group").is_none());
    // `front_nodes` matches existing nodes and creates `ls_front_nodes`.
    assert_eq!(imported.model.set("ls_front_nodes").unwrap().nodes, vec![1, 2, 3, 4]);
    // Multi-point elastic ramp snapped cleanly to highest elastic stress 2.0e8 at eps_p = 0.0.
    match imported.model.material("CurveMat").unwrap().plastic.as_ref().unwrap() {
        Plastic::Table { points } => {
            assert_eq!(points.len(), 2);
            assert_eq!(points[0], HardeningPoint { strain: 0.0, stress: 2.0e8 });
            assert!((points[1].strain - (0.0210 - 3.0e8 / 2.0e11)).abs() < 1e-12);
        }
        other => panic!("unexpected CurveMat plastic: {other:?}"),
    }
    assert!(rel_close(imported.model.material("Steel").unwrap().density.unwrap(), 7850.0, 1e-12));
    assert!(imported.warnings.iter().any(|w| w.contains("alpha")));
    assert!(imported.warnings.iter().any(|w| w.contains("piecewise linear")));
}

#[test]
fn lex_comments_encoding_continuations_and_tokenization() {
    let mut raw = Vec::new();
    // Col-1 Latin-1 comments & inline `$` Latin-1 comments are tolerated, while active Latin-1 fails.
    raw.extend_from_slice(b"! Latin-1 col-1 comment: \xff\xfe\xb0\n");
    raw.extend_from_slice(b"# Hash comment: \xe9\n");
    raw.extend_from_slice(b"c\tTab-after-c comment \xff\n");
    raw.extend_from_slice(b"C Fortran col-1 comment \xff\n");
    raw.extend_from_slice(b"! fieldcraft-units: m-kg-s-N-Pa $ comment \xff\n");
    raw.extend_from_slice(b"structure demo $ inline latin-1 \xff\n");
    raw.extend_from_slice(b"material M1-Steel\n");
    raw.extend_from_slice(b"  properties mises e 2.0d+11 nu 3.0d-1 ,\n");
    raw.extend_from_slice(b"! comment inside continuation\n\n");
    raw.extend_from_slice(b"  yld_pt 2.5e+08 n_power 1.0\n");
    raw.extend_from_slice(b"*echo on\n");
    raw.extend_from_slice(b"coordinates\n");
    raw.extend_from_slice(b"  1\t-1.5e-02\t0.0\t0.0\n");
    raw.extend_from_slice(b"  2  1.0e0   0.0  0.0\n");
    raw.extend_from_slice(b"  3  0.0     1.0  0.0\n");
    raw.extend_from_slice(b"  4  0.0     0.0  1.0\n");
    raw.extend_from_slice(b"incidences\n  1 1 2 3 4\n");
    raw.extend_from_slice(b"elements\n  1-1 type tet4 linear material M1-Steel\n");
    raw.extend_from_slice(b"list 'mismatched\" 1-4\n");

    let imported = read_limited("lex.inp", &raw, None, Preset::MmTonneS, ReadLimits::default()).unwrap();
    assert!((imported.model.node(1).unwrap().xyz[0] - (-1.5e-02)).abs() < 1e-15);
    assert!(imported.model.material("M1-Steel").is_some());
    assert_eq!(imported.model.set("ls_mismatched").unwrap().nodes, vec![1, 2, 3, 4]);

    // Invalid active UTF-8 fails with Parse.
    let bad_utf8 = b"coordinates\n  1 0 0 \xff\n";
    let err = read_limited("bad_utf8.inp", bad_utf8, None, Preset::MKgS, ReadLimits::default()).unwrap_err();
    assert!(matches!(err, ReadError::Parse { line: 2, .. }), "{err:?}");

    // `*` inside continuation fails with Parse.
    let star_in_cont = b"material M\n  properties linear e 1e11 ,\n*echo on\n  nu 0.3\n";
    let err = read_limited("star_cont.inp", star_in_cont, None, Preset::MKgS, ReadLimits::default()).unwrap_err();
    assert!(matches!(err, ReadError::Parse { line: 3, .. }), "{err:?}");

    // EOF inside continuation fails with Parse.
    let eof_in_cont = b"material M\n  properties linear e 1e11 ,\n";
    let err = read_limited("eof_cont.inp", eof_in_cont, None, Preset::MKgS, ReadLimits::default()).unwrap_err();
    assert!(matches!(err, ReadError::Parse { line: 2, .. }), "{err:?}");
}

#[test]
fn scan_units_header_edge_cases() {
    assert_eq!(scan_units_header(b"  !\tFieldCraft-Units : in-lbf-s-psi $ imperial\n").unwrap(), Some((Preset::InLbfS, 1)));
    // Duplicate header ignored after first valid header.
    assert_eq!(scan_units_header(b"! fieldcraft-units: m-kg-s-N-Pa\n! fieldcraft-units: bad-preset\n").unwrap(), Some((Preset::MKgS, 1)));
    // Past line 128 ignored.
    let mut past_128 = vec![b'\n'; 128];
    past_128.extend_from_slice(b"! fieldcraft-units: in-lbf-s-psi\n");
    assert_eq!(scan_units_header(&past_128).unwrap(), None);

    // Malformed / empty headers return Parse error.
    for bad in [b"! fieldcraft-units:\n".as_slice(), b"! fieldcraft-units: $ only comment\n", b"! fieldcraft-units: unknown-unit\n"] {
        assert!(matches!(scan_units_header(bad).unwrap_err(), ReadError::Parse { line: 1, .. }));
    }
}

#[test]
fn immediate_compute_halt_and_skipped_blocks() {
    let mut raw = Vec::new();
    raw.extend_from_slice(b"! fieldcraft-units: m-kg-s-N-Pa\n");
    raw.extend_from_slice(b"material S\n  properties linear e 2e11 nu 0.3\n");
    raw.extend_from_slice(b"coordinates\n  1 0 0 0\n  2 1 0 0\n  3 0 1 0\n  4 0 0 1\n");
    raw.extend_from_slice(b"incidences\n  1 1 2 3 4\n");
    raw.extend_from_slice(b"elements\n  1 type tet4 linear material S\n");
    raw.extend_from_slice(b"blocking automatic size = 128 display\n");
    raw.extend_from_slice(b"constraints\n  1 u 0.0 v 0.0 w 0.0\n");
    raw.extend_from_slice(b"loading pull\n  nodal loads\n    2 force_x 10.0\n");
    raw.extend_from_slice(b"loading driver\n  nonlinear\n    step 1 pull 1.0\n");
    raw.extend_from_slice(
        b"nonlinear analysis parameters\n  solution technique direct sparse\n  material messages off\n  consistent q-matrix on\n  maximum iterations 5\n",
    );
    raw.extend_from_slice(b"compute displacements for loading driver step 1\n");
    // Post-compute invalid UTF-8 and `*input` must never be lexed or touched!
    raw.extend_from_slice(b"*input 'domain_defs.inp\n");
    raw.extend_from_slice(b"\xff\xfe invalid utf8 after compute\n");

    let imported = read_limited("halt.inp", &raw, None, Preset::MmTonneS, ReadLimits::default()).unwrap();
    assert_eq!(imported.stopped_at, Some(StoppedAt { file: "halt.inp".into(), line: 27, reason: "compute".into() }));
    assert_eq!(imported.unknown_count, 4);
    assert_eq!(imported.unknown.iter().map(|u| u.keyword.as_str()).collect::<Vec<_>>(), vec!["blocking", "loading", "nonlinear analysis", "compute"]);
    assert_eq!(imported.model.boundary_conditions().len(), 1);
    assert_eq!(imported.model.loads().len(), 1);
    assert!(imported.warnings.iter().any(|w| w.contains("compute")));
}

#[test]
fn matchs_prefixes_and_unsupported_refusals() {
    // `symmetric` in `blocking` / `list` and `linear` matching `linear_elastic` prefix.
    let deck = "\
! fieldcraft-units: m-kg-s-N-Pa
material M
  properties linear e 2e11 nu 0.3
coordinates
  1 0 0 0
  2 1 0 0
  3 0 1 0
  4 0 0 1
incidences
  1 1 2 3 4
elements
  1 type tet4 linear material M
";
    assert!(read_limited("ok.inp", deck.as_bytes(), None, Preset::MKgS, ReadLimits::default()).is_ok());

    // Unsupported directives & features (Bucket B)
    let unsupported_cases = [
        "*input window\n",
        "coordinates from file 'coords.bin'\n",
        "incidences from file 'inc.bin'\n",
        "material G\n  properties gurson e 2e11 nu 0.3\n",
        "material U\n  properties umat e 2e11 nu 0.3\n",
        "material C\n  properties curves e 2e11 nu 0.3\n",
        "material B\n  properties mises e 2e11 nu 0.3 yld_pt 2e8 beta 0.5\n",
        "elements\n  1 type bar2 material S\n",
        "elements\n  1 type q3disop material S order 1x1x1\n",
        "stress-strain curve 1 temperature\n  0.0 0.0\n",
        "constraints\n  transformation matrix 1\n",
        "constraints\n  multipoint\n  1 1.0 u - 2 1.0 u = 0.0\n",
        "constraints\n  multipoint\n  1 1.0 u = 0.5\n",
        "constraints\n  multipoint\n  'a' rigid 'b'\n",
        "loading p\n  element loads\n    1 face 1 piston tab1\n",
        "loading p\n  element loads\n    1 face 1 local pressure 1.0\n",
        "loading p\n  element loads\n    1 face 1 projected pressure 1.0\n",
        "loading p\n  element loads\n    1 face 1 pressure 1.0 2.0\n",
        "material S\n  properties linear e 2e11 nu 0.3\ncoordinates\n  1 0 0 0\n  2 1 0 0\n  3 0 1 0\n  4 0 0 1\nincidences\n  1 1 2 3 4\nelements\n  1 type tet4 linear material S\nloading p\n  element loads\n    1 face abaqus 1 pressure 1.0\n",
        "restart\n",
        "tie mesh\n",
        "contact\n",
        "table piston_one rows 1 type piston\n",
        "surface left\n",
    ];
    for case in unsupported_cases {
        let err = read_limited("unsup.inp", case.as_bytes(), None, Preset::MKgS, ReadLimits::default()).unwrap_err();
        assert!(matches!(err, ReadError::Unsupported { .. }), "expected Unsupported for:\n{case}\ngot: {err:?}");
    }

    // Standalone `mesh`, reversed range `10-1`, malformed constraints/multipoint/loads must return Parse.
    for parse_err_case in [
        "mesh\n",
        "list bad 10-1\n",
        "list bad 1 to 5 by 0\n",
        "material Bad\n  properties bilinear e 2e11 nu 0.3 yld_pt 2e8 tan_e 3e11\n",
        "material BadN\n  properties mises e 2e11 nu 0.3 yld_pt 2e8 n_power 0.5\n",
        "material NoYld\n  properties mises e 2e11 nu 0.3 n_power 5.0\n",
        "constraints\n  plane fixed\n",
        "constraints\n  plane x=0 x=1 symmetry\n",
        "constraints\n  plane x=0 proximity -1e-3 symmetry\n",
        "constraints\n  multipoint\n  1 1.0 u\n",
        "constraints\n  multipoint\n  1 1.0 u - 1 1.0 u = 0.0\n",
    ] {
        let err = read_limited("bad.inp", parse_err_case.as_bytes(), None, Preset::MKgS, ReadLimits::default()).unwrap_err();
        assert!(matches!(err, ReadError::Parse { .. }), "expected Parse for:\n{parse_err_case}\ngot: {err:?}");
    }
}

#[test]
fn hostile_limits_range_bomb_and_cancellation() {
    // Range bomb (within MAX_LABEL) must refuse instantaneously with Limit without OOM or looping.
    let bomb = "list huge 1-99999999\n";
    let err = read_limited("bomb.inp", bomb.as_bytes(), None, Preset::MKgS, ReadLimits { nodes: 100, elements: 100, members: 50 }).unwrap_err();
    assert!(matches!(err, ReadError::Limit(_)), "{err:?}");

    let elem_bomb = "elements 1-99999999 type tet4 material S\n";
    let err = read_limited("ebomb.inp", elem_bomb.as_bytes(), None, Preset::MKgS, ReadLimits { nodes: 100, elements: 50, members: 100 }).unwrap_err();
    assert!(matches!(err, ReadError::Limit(_)), "{err:?}");

    // Internal lexer limits.
    let err = read_with_internal_limits(
        "tiny.inp",
        b"structure a\n",
        None,
        Preset::MKgS,
        ReadLimits::default(),
        LexLimits { max_input_bytes: 5, ..LexLimits::default() },
    )
    .unwrap_err();
    assert!(matches!(err, ReadError::Limit(_)), "{err:?}");

    // Cooperative cancellation via Progress::cancel() across 256-record boundary.
    let mut big = String::from("! fieldcraft-units: m-kg-s-N-Pa\ncoordinates\n");
    for i in 1..=600 {
        big.push_str(&format!("  {i} 0.0 0.0 0.0\n"));
    }
    let progress = Arc::new(Progress::default());
    progress.cancel();
    let err = progress::scoped(Some(progress), || read_limited("cancel.inp", big.as_bytes(), None, Preset::MKgS, ReadLimits::default())).unwrap_err();
    assert!(matches!(err, ReadError::Model(fieldcraft_model::Error::Cancelled)), "{err:?}");
}

#[test]
fn includes_macros_and_error_cases() {
    use std::collections::BTreeMap;

    let mut files: BTreeMap<&str, &[u8]> = BTreeMap::new();
    files.insert(
        "params.inp",
        b"*parameter E_val = '2.0e11' nu_val = '0.3' matName = 'Alloy'\n*parameter Y0 = '2.0e8', NEXP = '10'\n*parameter display\n",
    );
    files.insert("sub/coords.inp", b"coordinates\n  1 0 0 0\n  2 1 0 0\n  3 0 1 0\n  4 0 0 1\n");
    files.insert("incids.inp", b"incidences\n  1 1 2 3 4\n");
    files.insert("bcs.inp", b"  plane x=0 symmetry\n");

    let root = b"! fieldcraft-units: m-kg-s-N-Pa
*input from 'params.inp'
material '@matName'
  properties mises e @E_val nu @nu_val yld_pt @Y0 n_power @NEXP
elements
  1 type tet4 linear material '@matName'
*inout from file \"sub/coords.inp\"
*input 'incids.inp
constraints
*input'bcs.inp'
  2 u 0.0
";

    let imported = read_with_includes("root.inp", root, None, Preset::MmTonneS, ReadLimits::default(), |req| {
        let Some(&data) = files.get(req.target) else {
            return Err(ReadError::Parse { file: req.target.into(), line: 0, message: "not found".into() });
        };
        Ok(ResolvedInclude { display_name: req.target.to_string(), canonical_key: req.target.to_string(), bytes: data.to_vec() })
    })
    .unwrap();

    assert_eq!(imported.includes_read, vec!["params.inp", "sub/coords.inp", "incids.inp", "bcs.inp"]);
    assert!(imported.model.material("Alloy").is_some());
    assert_eq!(imported.model.nodes().len(), 4);
    assert_eq!(imported.model.elements().len(), 1);
    assert_eq!(imported.model.boundary_conditions().len(), 1);

    // Sequential non-cyclic reuse of the same include is allowed once popped.
    let seq_root = b"! fieldcraft-units: m-kg-s-N-Pa\n*input 'p.inp'\n*input 'p.inp'\ncoordinates\n  1 0 0 0\n";
    let seq_res = read_with_includes("seq.inp", seq_root, None, Preset::MKgS, ReadLimits::default(), |req| {
        Ok(ResolvedInclude { display_name: req.target.into(), canonical_key: "same_key".into(), bytes: b"*parameter X = '1'\n".to_vec() })
    })
    .unwrap();
    assert_eq!(seq_res.includes_read, vec!["p.inp", "p.inp"]);

    // Cycle detection -> IncludeRefused.
    let cyc_err = read_with_includes("cyc.inp", b"*input 'a.inp'\n", None, Preset::MKgS, ReadLimits::default(), |req| {
        Ok(ResolvedInclude { display_name: req.target.into(), canonical_key: "a_key".into(), bytes: b"*input 'a.inp'\n".to_vec() })
    })
    .unwrap_err();
    assert!(matches!(cyc_err, ReadError::IncludeRefused(_)), "{cyc_err:?}");

    // Depth limit & total include limit.
    let depth_err = read_with_includes_internal_limits(
        "depth.inp",
        b"*input 'next.inp'\n",
        None,
        Preset::MKgS,
        ReadLimits::default(),
        LexLimits { max_include_depth: 2, ..LexLimits::default() },
        |req| {
            static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            let k = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            Ok(ResolvedInclude { display_name: req.target.into(), canonical_key: format!("k_{k}"), bytes: b"*input 'next.inp'\n".to_vec() })
        },
    )
    .unwrap_err();
    assert!(matches!(depth_err, ReadError::Limit(_)), "{depth_err:?}");

    // Case sensitivity, non-recursive expansion, missing param, `~/`, and `> subs` cap.
    let non_rec = b"! fieldcraft-units: m-kg-s-N-Pa\n*parameter a = '@b' b = '99' A = '2.0'\ncoordinates\n  1 @A 0 0\n";
    let imp_case = read_limited("case.inp", non_rec, None, Preset::MKgS, ReadLimits::default()).unwrap();
    assert!((imp_case.model.node(1).unwrap().xyz[0] - 2.0).abs() < 1e-12);

    assert!(matches!(
        read_limited("missing.inp", b"coordinates\n  1 @NOPE 0 0\n", None, Preset::MKgS, ReadLimits::default()).unwrap_err(),
        ReadError::Parse { .. }
    ));
    assert!(matches!(
        read_limited("tilde.inp", b"*input '~/secret.inp'\n", None, Preset::MKgS, ReadLimits::default()).unwrap_err(),
        ReadError::IncludeRefused(_)
    ));

    let too_many_subs = read_with_internal_limits(
        "subs.inp",
        b"*parameter x = '1'\ncoordinates\n  1 @x @x @x\n",
        None,
        Preset::MKgS,
        ReadLimits::default(),
        LexLimits { max_subs_per_line: 2, ..LexLimits::default() },
    )
    .unwrap_err();
    assert!(matches!(too_many_subs, ReadError::Parse { .. }), "{too_many_subs:?}");
}

#[test]
fn power_law_n_power_discretization_accuracy_and_clamping() {
    let young = 2.0e11_f64;
    for eps0 in [1.0e-4_f64, 1.0e-3, 1.0e-2] {
        let sigma0 = eps0 * young;
        for n in [1.5_f64, 10.0, 100.0, 1000.0] {
            let deck = format!(
                "! fieldcraft-units: m-kg-s-N-Pa\nmaterial P\n  properties mises e {young:e} nu 0.3 yld_pt {sigma0:e} n_power {n}\ncoordinates\n  1 0 0 0\n"
            );
            let imported = read_limited("pow.inp", deck.as_bytes(), None, Preset::MKgS, ReadLimits::default()).unwrap();
            assert!(imported.warnings.iter().any(|w| w.contains("clamped flat past 1.0")));
            let Some(Plastic::Table { points }) = imported.model.material("P").unwrap().plastic.as_ref() else {
                panic!("expected Plastic::Table for n={n}");
            };
            assert_eq!(points.len(), 64);
            assert_eq!(points[0].strain, 0.0);
            assert!((points.last().unwrap().strain - 1.0).abs() < 1e-10);

            // Sample log-spaced plastic strains in (0, 1.0] and verify < 0.25% max relative stress error.
            let inv_n = 1.0 / n;
            let exact_stress_at_eps_p = |target_ep: f64| -> f64 {
                let mut lo = 1.0_f64;
                let mut hi = 1.0 + (target_ep / eps0) + 10.0;
                while eps0 * (hi - hi.powf(inv_n)) < target_ep {
                    hi *= 2.0;
                }
                for _ in 0..80 {
                    let mid = 0.5 * (lo + hi);
                    if eps0 * (mid - mid.powf(inv_n)) < target_ep {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                sigma0 * (0.5 * (lo + hi)).powf(inv_n)
            };

            let mut max_rel_err = 0.0_f64;
            let max_strain = points.last().unwrap().strain;
            for j in 0..200 {
                let ep = (10.0_f64.powf(-8.0 + 8.0 * (j as f64 / 199.0))).min(max_strain);
                let seg = points.windows(2).find(|w| ep >= w[0].strain && ep <= w[1].strain).unwrap();
                let t = (ep - seg[0].strain) / (seg[1].strain - seg[0].strain);
                let interp = seg[0].stress + t * (seg[1].stress - seg[0].stress);
                let exact = exact_stress_at_eps_p(ep);
                let rel = (interp - exact).abs() / exact;
                max_rel_err = max_rel_err.max(rel);
            }
            let bound = if eps0 < 5.0e-4 && n < 2.0 { 3.0e-3 } else { 2.5e-3 };
            assert!(max_rel_err < bound, "eps0={eps0}, n={n}: max_rel_err={max_rel_err}");
        }
    }
}

#[test]
fn constraints_loading_and_roundtrip_bcs_loads() {
    // 1. Test `constraints` (`all`, `plane` permutations + 0-match warning, single-node `multipoint`),
    //    per-pattern `loading`, `face abaqus`, `tractions`, `body [forces]`, `temperature` ignore.
    let deck = "\
! fieldcraft-units: m-kg-s-N-Pa
material M
  properties linear e 2e11 nu 0.3
coordinates
  1 0 0 0
  2 1 0 0
  3 1 1 0
  4 0 1 0
  5 0 0 1
  6 1 0 1
  7 1 1 1
  8 0 1 1
incidences
  1 1 2 3 4 5 6 7 8
elements
  1 type l3disop linear material M
list 'tip' 2 3 6 7
constraints
  plane x=0.0 verify fixed
  plane symmetry y proximity 1e-4
  plane z=99.0 fixed
  7 w=0.0
  multipoint
  4 1.0 u + 4 1.0 v = 0.0
loading pat_a
  nodal loads
    tip force_x 100.0
    tip force_x -100.0
    2 force_y 50.0
  element loads
    1 face 4 pressure 10.0 face abaqus 4 pressure 5.0
    1 face 6 tractions ty 3.0 tx 4.0
    1 body forces bx 1.0 bz 2.0
    1 temperature 25.0
loading pat_b
  element loads
    1 face 5 pressure 7.5
stop
";
    let imported = read_limited("bcs_loads.inp", deck.as_bytes(), None, Preset::MmTonneS, ReadLimits::default()).unwrap();
    let m = &imported.model;
    assert!(imported.warnings.iter().any(|w| w.contains("matched 0 nodes")));
    assert!(imported.warnings.iter().any(|w| w.contains("temperature")));
    assert_eq!(m.boundary_conditions().len(), 4);
    assert!(m.boundary_conditions().values().any(|bc| matches!(bc, BoundaryCondition::Symmetry { plane: Symmetry::Normal(_), .. })));
    // Zero net nodal force on nodes 3, 6, 7 was dropped; node 2 retains force_y = 50.0.
    // Face 4 (WARP3D -> FC face 5: 10.0 Pa) and face abaqus 4 (FC face 4: 5.0 Pa), plus face 5 (WARP3D -> FC face 4: 7.5 Pa).
    assert_eq!(m.loads().len(), 6, "loads: {:?}", m.loads());
    assert!(m.loads().values().any(|l| matches!(l, Load::Pressure { magnitude, .. } if (*magnitude - 10.0).abs() < 1e-12)));
    assert!(m.loads().values().any(|l| matches!(l, Load::Pressure { magnitude, .. } if (*magnitude - 7.5).abs() < 1e-12)));
    assert!(m.loads().values().any(|l| matches!(l, Load::Traction { vector, .. } if *vector == [4.0, 3.0, 0.0])));
    assert!(m.loads().values().any(|l| matches!(l, Load::BodyForce { vector, .. } if *vector == [1.0, 0.0, 2.0])));

    // 2. Round-trip BCs & Hex face 4/5 pressures through `write_deck -> read_with_includes`!
    let mut rt_model = hex8_cantilever_model();
    rt_model.add_set("face4", Set { faces: vec![fieldcraft_model::ElementFace { element: 1, face: 4 }], ..Set::default() }).unwrap();
    rt_model.add_set("face5", Set { faces: vec![fieldcraft_model::ElementFace { element: 1, face: 5 }], ..Set::default() }).unwrap();
    rt_model.add_load("p4", Load::Pressure { set: "face4".into(), magnitude: 2500.0 }).unwrap();
    rt_model.add_load("p5", Load::Pressure { set: "face5".into(), magnitude: 4500.0 }).unwrap();
    rt_model.add_boundary_condition("sym_y", BoundaryCondition::Symmetry { target: Target::Set("fix".into()), plane: Symmetry::Y }).unwrap();
    let mut step = rt_model.step("Pull").unwrap().clone();
    step.loads.insert("p4".into());
    step.loads.insert("p5".into());
    step.boundary_conditions.insert("sym_y".into());
    rt_model.replace_step(step).unwrap();

    let written = write_deck(&rt_model, None, None, Preset::InLbfS).unwrap();
    let rt = read_limited("rt_full.inp", written.text.as_bytes(), None, Preset::MKgS, ReadLimits::default()).unwrap();
    assert!(!rt.model.boundary_conditions().is_empty());
    let has_p4 = rt.model.loads().iter().any(|(_, l)| {
        matches!(l, Load::Pressure { set, magnitude } if (*magnitude - 2500.0).abs() < 1e-2 && rt.model.set(set).unwrap().faces == vec![fieldcraft_model::ElementFace { element: 1, face: 4 }])
    });
    let has_p5 = rt.model.loads().iter().any(|(_, l)| {
        matches!(l, Load::Pressure { set, magnitude } if (*magnitude - 4500.0).abs() < 1e-2 && rt.model.set(set).unwrap().faces == vec![fieldcraft_model::ElementFace { element: 1, face: 5 }])
    });
    assert!(has_p4 && has_p5, "Hex face 4 <-> 5 swap failed round-trip: {:?}", rt.model.loads());
}

#[test]
fn corpus_edge_cases_and_collapsed_wedges() {
    let base_mesh = "coordinates\n 1 0 0 0\n 2 1 0 0\n 3 0 1 0\n 4 0 0 1\n 5 1 0 1\n 6 0 1 1\nincidences\n 1 1 2 3 1 4 5 6 4\n";

    // 1. Rate-dependent curves, FGM properties, and rate-dependent plasticity keywords refuse cleanly (Bucket B).
    let rate_curve = format!("stress-strain curve 1 plastic strain-rate 500\n 0.0 80\n 0.1 100\n{base_mesh}");
    assert!(matches!(
        read_limited("rate_curve.inp", rate_curve.as_bytes(), None, Preset::MKgS, ReadLimits::default()),
        Err(ReadError::Unsupported { feature, .. }) if feature == "rate-dependent curve"
    ));

    let fgm_mat = format!("material fgm\n properties linear_elastic e 'fgm' nu 'fgm'\n{base_mesh}");
    assert!(matches!(
        read_limited("fgm.inp", fgm_mat.as_bytes(), None, Preset::MKgS, ReadLimits::default()),
        Err(ReadError::Unsupported { feature, .. }) if feature == "functionally graded material"
    ));

    let vp_mat = format!("material steel\n properties mises e 2e11 nu 0.3 yld_pt 3e8 n_power 10 ref_eps 1.0 m_power 35.0\n{base_mesh}");
    assert!(matches!(
        read_limited("vp.inp", vp_mat.as_bytes(), None, Preset::MKgS, ReadLimits::default()),
        Err(ReadError::Unsupported { feature, .. }) if feature == "rate-dependent plasticity"
    ));

    // 2. Implicit constitutive keyword + `list copy/display` + collapsed l3disop -> Wedge6 + face loads.
    let collapsed_deck = format!(
        "\
! fieldcraft-units: m-kg-s-N-Pa
material M
  properties e 2e11 nu 0.3 yld_pt 3e8 tan_e 2e9
elements
  1 type l3disop linear material M
{base_mesh}\
list 'a' x=0.0
list 'a' copy 'b'
list 'b' display
loading pull
  element loads
    1 face 3 pressure 10.0
    1 face 5 pressure 20.0
stop
"
    );
    let imported = read_limited("wedge.inp", collapsed_deck.as_bytes(), None, Preset::MKgS, ReadLimits::default()).unwrap();
    let elem = imported.model.element(1).unwrap();
    assert_eq!(elem.kind, ElementKind::Wedge6);
    assert_eq!(elem.nodes, vec![1, 2, 3, 4, 5, 6]);
    assert!(imported.warnings.iter().any(|w| w.contains("wedge6")));
    let p_faces: std::collections::BTreeMap<u8, f64> = imported
        .model
        .loads()
        .values()
        .filter_map(|l| match l {
            Load::Pressure { set, magnitude } => Some((imported.model.set(set)?.faces.first()?.face, *magnitude)),
            _ => None,
        })
        .collect();
    assert_eq!(p_faces.get(&3), Some(&10.0));
    assert_eq!(p_faces.get(&4), Some(&20.0));

    // 3. Loading the collapsed crease face (hex face 6 = nodes {1, 4}) rejects with Parse.
    let bad_face = format!(
        "\
! fieldcraft-units: m-kg-s-N-Pa
material M
  properties linear e 2e11 nu 0.3
elements
  1 type l3disop linear material M
{base_mesh}\
loading bad
  element loads
    1 face 6 pressure 10.0
stop
"
    );
    assert!(matches!(
        read_limited("bad_wedge_face.inp", bad_face.as_bytes(), None, Preset::MKgS, ReadLimits::default()),
        Err(ReadError::Parse { message, .. }) if message.contains("collapsed")
    ));
}

//! Solver decks: write model data, or read a flat Abaqus deck into a new session.

use std::path::Path;

use fieldcraft_io::roots::Access;
use serde_json::{Value, json};

use super::{opt, req};
use crate::requirement::Requirement;
use crate::{CommandKind, CommandSpec, EngineError, Result, Session};

/// Format metadata exposed to the UI for deck-export pickers across both
/// `fieldcraft_decks::FORMATS` and WARP3D `.inp` decks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeckFormatInfo {
    pub name: &'static str,
    pub extension: &'static str,
    pub alternatives: &'static [&'static str],
}

const WARP3D_ALT: &[&str] = &["inp"];

/// Look up deck export extension metadata for a supported deck format (`abaqus`, `nastran`,
/// `lsdyna`, `warp3d`).
pub fn deck_format_info(name: &str) -> Option<DeckFormatInfo> {
    if name == "warp3d" {
        return Some(DeckFormatInfo { name: "warp3d", extension: "inp", alternatives: WARP3D_ALT });
    }
    let fmt = fieldcraft_decks::format(name)?;
    let alternatives: &'static [&'static str] = match fmt.extension {
        "inp" => &["inp"],
        "bdf" => &["bdf", "dat", "nas"],
        "k" => &["k", "key", "dyn"],
        _ => &[],
    };
    Some(DeckFormatInfo { name: fmt.name, extension: fmt.extension, alternatives })
}

pub const WRITE: CommandSpec = CommandSpec {
    id: "deck.write",
    label: "Write Solver Deck",
    params: r#"{"format": "abaqus|nastran|lsdyna|warp3d", "path": "part.inp", "steps": ["Load"], "units": "mm-t-s-N-MPa|m-kg-s-N-Pa|mm-kg-ms-kN-GPa|mm-g-ms-N-MPa|in-lbf-s-psi", "crack_front": {"set": "front"}} — writes the model as a solver deck in one consistent unit system, stated in its header (default: the display system's, ignoring its overrides) (formats: fieldcraft_decks::FORMATS, plus warp3d); `steps` (optional) writes only those steps, as a Nastran or WARP3D run needs (WARP3D runs a single structural step); LS-DYNA writes several steps as *CASE analyses (run it with CASE); fails naming every error of the model check and everything the format cannot express"#,
    form: Some(super::forms::DECK),
    kind: CommandKind::Session,
    requires: &[Requirement::Analysable, Requirement::ElementMaterial, Requirement::ElementSection],
    run: write,
};

pub const READ: CommandSpec = CommandSpec {
    id: "deck.read",
    label: "Read Abaqus Deck",
    params: r#"{"format":"abaqus","path":"part.inp","units"?:"mm-t-s-N-MPa|m-kg-s-N-Pa|mm-kg-ms-kN-GPa|mm-g-ms-N-MPa|in-lbf-s-psi"} — replaces the model with a flat Abaqus deck's mesh, sets, element surfaces, elastic/density materials, sections and the linear history written by the current writer (static, frequency and buckling steps, boundary conditions and concentrated loads); input numbers use this consistent preset (default: the current display preset, ignoring overrides), then store SI; the reply states counts for nodes, elements by kind, sets, materials, sections, steps, boundary conditions and loads, plus units, unread keywords with original lines (first 100 and total count), and warnings. Unsupported history remains reported and is not stored; INCLUDE/parts/assemblies and unsupported physical variants are refused. Empty supported data is refused. Success clears old results/design/path/undo and is unsaved; errors leave the whole session intact."#,
    form: Some(super::forms::DECK_READ),
    kind: CommandKind::Session,
    requires: &[],
    run: read,
};

fn read(s: &mut Session, p: &Value) -> Result<Value> {
    let format: &str = req(p, "format")?;
    if format != "abaqus" {
        return Err(EngineError::invalid("format", "only abaqus is supported by deck.read"));
    }
    let units = match opt::<String>(p, "units")? {
        Some(label) => fieldcraft_model::units::Preset::from_label(&label)
            .ok_or_else(|| EngineError::invalid("units", "choose a known consistent unit preset"))?,
        None => s.units().preset(),
    };
    let path = super::file_param(s, p, "path", Access::Read)?;
    let bytes = super::file::read_file(&path, fieldcraft_decks::abaqus::read::MAX_INPUT_BYTES as u64)?;
    let text = std::str::from_utf8(&bytes).map_err(|_| EngineError::invalid("path", "the input deck is not UTF-8 text"))?;
    let limits = fieldcraft_decks::abaqus::Limits {
        nodes: s.limits.max_import_nodes,
        elements: s.limits.max_import_elements,
        members: s.limits.max_import_members,
    };
    let imported = fieldcraft_decks::abaqus::read_analysis_limited(text, units, limits)?;
    let model = &imported.model;
    if model.nodes().is_empty()
        && model.elements().is_empty()
        && model.sets().values().all(fieldcraft_model::Set::is_empty)
        && model.materials().is_empty()
        && model.sections().is_empty()
    {
        return Err(EngineError::invalid(
            "path",
            format!("the deck contains no supported model data ({} unread keywords); the current model is unchanged", imported.unknown_count),
        ));
    }
    let mut kinds = std::collections::BTreeMap::<&str, usize>::new();
    for element in model.elements().values() {
        *kinds.entry(element.kind.name()).or_default() += 1;
    }
    let unknown: Vec<Value> = imported.unknown.iter().map(|u| json!({"keyword":u.keyword,"line":u.line})).collect();
    let reply = json!({"format":"abaqus","path":path,"units":units.label(),"nodes":model.nodes().len(),"elements":kinds,
        "sets":model.sets().len(),"materials":model.materials().len(),"sections":model.sections().len(),
        "steps":model.steps().len(),"boundary_conditions":model.boundary_conditions().len(),"loads":model.loads().len(),"unknown":unknown,
        "unknown_count":imported.unknown_count,"warnings":imported.warnings,"history_imported":!model.steps().is_empty()});
    // The new, unsaved document has no journal, but still needs its own dirty revision.
    let opened = Session { model: imported.model, revision: crate::next_revision(), ..Session::default() };
    s.replace_with(opened);
    Ok(reply)
}

pub(super) fn parse_crack_front(p: &Value) -> Result<Option<fieldcraft_warp3d::CrackFrontSpec>> {
    let Some(raw) = p.get("crack_front") else {
        return Ok(None);
    };
    if raw.is_null() {
        return Ok(None);
    }
    let Some(obj) = raw.as_object() else {
        return Err(EngineError::invalid("crack_front", "must be a JSON object"));
    };
    let set = obj
        .get("set")
        .and_then(Value::as_str)
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .ok_or_else(|| EngineError::invalid("crack_front/set", "must name an existing crack-front set"))?
        .to_string();

    let normal = match obj.get("normal") {
        None | Some(Value::Null) => None,
        Some(Value::Array(arr)) if arr.len() == 3 => {
            let mut v = [0.0; 3];
            for (i, item) in arr.iter().enumerate() {
                let n = item
                    .as_f64()
                    .filter(|n| n.is_finite())
                    .ok_or_else(|| EngineError::invalid("crack_front/normal", "each component must be a finite number"))?;
                if let Some(slot) = v.get_mut(i) {
                    *slot = n;
                }
            }
            Some(v)
        }
        Some(_) => return Err(EngineError::invalid("crack_front/normal", "must be [nx, ny, nz] finite numbers")),
    };

    let symmetric = match obj.get("symmetric") {
        None | Some(Value::Null) => None,
        Some(Value::Bool(b)) => Some(*b),
        Some(_) => return Err(EngineError::invalid("crack_front/symmetric", "must be true or false")),
    };

    let rings = match obj.get("rings") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let r = v
                .as_u64()
                .and_then(|n| usize::try_from(n).ok())
                .filter(|r| (1..=20).contains(r))
                .ok_or_else(|| EngineError::invalid("crack_front/rings", "must be an integer from 1 to 20"))?;
            Some(r)
        }
    };

    let interaction_integral = match obj.get("interaction_integral") {
        None | Some(Value::Null) => None,
        Some(Value::Bool(b)) => Some(*b),
        Some(_) => return Err(EngineError::invalid("crack_front/interaction_integral", "must be true or false")),
    };

    let function_type = match obj.get("function_type") {
        None | Some(Value::Null) => fieldcraft_warp3d::CrackFunctionType::Pointwise,
        Some(Value::String(s)) if s == "pointwise" => fieldcraft_warp3d::CrackFunctionType::Pointwise,
        Some(Value::String(s)) if s == "front_average" => fieldcraft_warp3d::CrackFunctionType::FrontAverage,
        Some(_) => {
            return Err(EngineError::invalid("crack_front/function_type", "must be `pointwise` or `front_average`"));
        }
    };

    Ok(Some(fieldcraft_warp3d::CrackFrontSpec { set, normal, symmetric, rings, interaction_integral, function_type }))
}

fn write(s: &mut Session, p: &Value) -> Result<Value> {
    let name: &str = req(p, "format")?;
    // WARP3D lives at L2 (`fieldcraft-warp3d`) alongside solvers rather than in L1
    // `fieldcraft-decks` so deck generation shares types with crack-front execution and results.
    let deck_format = if name == "warp3d" {
        None
    } else {
        Some(fieldcraft_decks::format(name).ok_or_else(|| {
            let shown: String = name.chars().take(32).collect();
            let mut known: Vec<&str> = fieldcraft_decks::FORMATS.iter().map(|f| f.name).collect();
            known.push("warp3d");
            EngineError::Params(format!("unknown deck format `{shown}`; known: {}", known.join(", ")))
        })?)
    };
    let path = super::file_param(s, p, "path", Access::Write)?;
    let steps: Option<Vec<String>> = super::opt(p, "steps")?;
    let units = match p.get("units").and_then(Value::as_str) {
        None => s.units().preset(),
        Some(label) => fieldcraft_model::units::Preset::from_label(label).ok_or_else(|| {
            let known: Vec<&str> = fieldcraft_model::units::Preset::ALL.iter().map(|u| u.label()).collect();
            EngineError::invalid("units", format!("must be one of {}", known.join(", ")))
        })?,
    };
    let crack_front = parse_crack_front(p)?;

    let Some(format) = deck_format else {
        let step_name = match steps.as_deref() {
            None => None,
            Some([one]) => Some(one.as_str()),
            Some([]) => return Err(EngineError::invalid("steps", "pass a single step name or leave empty")),
            Some(_) => return Err(EngineError::invalid("steps", "WARP3D writes one structural step per deck")),
        };
        let written = fieldcraft_warp3d::write_deck(&s.model, step_name, crack_front.as_ref(), units).map_err(|e| match e {
            fieldcraft_warp3d::Warp3dError::Model(m) => EngineError::Model(m),
            other => EngineError::Params(format!("not supported by WARP3D: {other}")),
        })?;
        fieldcraft_io::fs::write_output(Path::new(&path), written.text.as_bytes())?;
        return Ok(json!({
            "format": "warp3d",
            "path": path,
            "bytes": written.text.len(),
            "units": units.label(),
            "warnings": written.warnings
        }));
    };

    if crack_front.is_some() {
        return Err(EngineError::invalid("crack_front", "`crack_front` is only supported with format `warp3d`"));
    }

    let deck = format.write_steps(&s.model, steps.as_deref(), units)?;
    fieldcraft_io::fs::write_output(Path::new(&path), deck.text.as_bytes())?;
    Ok(json!({"format": format.name, "path": path, "bytes": deck.text.len(), "units": units.label(), "warnings": deck.warnings}))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_the_defined_model_and_refuses_what_it_cannot() {
        let dir = std::env::temp_dir().join(format!("fieldcraft-deck-write-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("bar.inp");
        let path_str = path.to_string_lossy().into_owned();

        let mut s = Session::default();
        crate::cmd::define::tests::defined_bar(&mut s);
        let journal = s.journal.len();
        let out = s.execute("deck.write", &json!({"format": "abaqus", "path": path_str})).unwrap();
        let written = std::fs::read_to_string(&path).unwrap();
        assert_eq!(written, fieldcraft_decks::abaqus::write(&s.model, fieldcraft_model::units::Preset::MmTonneS).unwrap());
        assert_eq!(out["bytes"], written.len());
        assert_eq!(out["warnings"].as_array().map(Vec::len), Some(1), "`press` is in no step: {out}");
        assert!(written.contains("*ELEMENT, TYPE=C3D8I"), "{written}");
        assert_eq!(s.journal.len(), journal, "writing a deck does not change the model");
        assert!(std::fs::read_dir(&dir).unwrap().count() == 1, "no partial file is left behind");

        // Unknown formats, missing parameters and models that are not ready fail without writing.
        let other = dir.join("other.inp").to_string_lossy().into_owned();
        assert!(s.execute("deck.write", &json!({"format": "nope", "path": other})).unwrap_err().to_string().contains("known: abaqus"));
        assert!(s.execute("deck.write", &json!({"path": other})).is_err());
        assert!(s.execute("deck.write", &json!({"format": "abaqus"})).is_err());
        let mut empty = Session::default();
        assert!(matches!(empty.execute("deck.write", &json!({"format": "abaqus", "path": other})), Err(EngineError::Unavailable(_))));
        assert!(!dir.join("other.inp").exists());
        assert!(matches!(s.execute("deck.write", &json!({"format": "abaqus", "path": dir.to_string_lossy()})), Err(EngineError::File(_))));

        // Nastran: one run is static or frequency steps, so the bar's two steps need two decks.
        let bdf = dir.join("bar.bdf").to_string_lossy().into_owned();
        let e = s.execute("deck.write", &json!({"format": "nastran", "path": bdf})).unwrap_err().to_string();
        assert!(e.contains("not supported by Nastran") && e.contains("separate decks"), "{e}");
        for (step, sol) in [("Bend", "SOL 101"), ("Modes", "SOL 103")] {
            s.execute("deck.write", &json!({"format": "nastran", "path": bdf, "steps": [step]})).unwrap();
            let deck = std::fs::read_to_string(&bdf).unwrap();
            assert!(deck.contains(sol) && deck.contains("RBE2") && deck.ends_with("ENDDATA\n"), "{deck}");
        }
        assert_eq!(s.journal.len(), journal);
        for steps in [json!([]), json!(["Nope"]), json!("Bend"), json!([1, 2]), json!([["Bend"]])] {
            assert!(s.execute("deck.write", &json!({"format": "nastran", "path": bdf, "steps": steps})).is_err(), "{steps}");
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn writes_lsdyna_keyword_decks() {
        let dir = std::env::temp_dir().join(format!("fieldcraft-deck-lsdyna-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("bar.k");
        let mut s = Session::default();
        crate::cmd::define::tests::defined_bar(&mut s);
        let out = s.execute("deck.write", &json!({"format": "lsdyna", "path": path.to_string_lossy()})).unwrap();
        let written = std::fs::read_to_string(&path).unwrap();
        assert_eq!(written, fieldcraft_decks::lsdyna::write(&s.model, fieldcraft_model::units::Preset::MmTonneS).unwrap().text);
        assert_eq!(out["format"], "lsdyna");
        assert!(written.starts_with("*KEYWORD long=y\n") && written.contains("*CASE_BEGIN_2"), "two steps, two cases");
        // One chosen step: a single analysis, no cases.
        s.execute("deck.write", &json!({"format": "lsdyna", "path": path.to_string_lossy(), "steps": ["Modes"]})).unwrap();
        let one = std::fs::read_to_string(&path).unwrap();
        assert!(one.contains("*CONTROL_IMPLICIT_EIGENVALUE") && !one.contains("*CASE_BEGIN") && !one.contains("*CONTROL_IMPLICIT_SOLUTION"), "{one}");
        // What LS-DYNA cannot take is named and nothing is written.
        s.execute("model.load.create", &json!({"name": "heat", "type": "body_force", "set": "all", "vector": [0, 0, 1]})).unwrap();
        s.execute("model.step.activate", &json!({"step": "Bend", "loads": ["heat"]})).unwrap();
        let other = dir.join("heat.k");
        let e = s.execute("deck.write", &json!({"format": "lsdyna", "path": other.to_string_lossy()})).unwrap_err().to_string();
        assert!(e.contains("not supported by LS-DYNA") && e.contains("load `heat`"), "{e}");
        assert!(!other.exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn writes_warp3d_decks() {
        let dir = std::env::temp_dir().join(format!("fieldcraft-deck-warp3d-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();

        // ReferencePoint / Coupling / multiple steps in `defined_bar` are refused cleanly without writing.
        let mut unsupported = Session::default();
        crate::cmd::define::tests::defined_bar(&mut unsupported);
        let bad_path = dir.join("unsupported.inp");
        let err = unsupported.execute("deck.write", &json!({"format": "warp3d", "path": bad_path.to_string_lossy()})).unwrap_err().to_string();
        assert!(err.contains("not supported by WARP3D"), "{err}");
        assert!(!bad_path.exists());

        // Pure 3D Hex8 static bar with fixed root, end pressure, and optional crack_front.
        let mut s = Session::default();
        crate::cmd::define::tests::tagged_bar(&mut s);
        let script = json!([
            {"command": "model.units", "params": {"system": "mm-t-s-N-MPa"}},
            {"command": "model.set.create", "params": {"name": "all", "geometry": [{"body": "Bar"}]}},
            {"command": "model.set.create", "params": {"name": "root", "geometry": [{"body": "Bar", "face": "left"}]}},
            {"command": "model.set.create", "params": {"name": "end", "geometry": [{"body": "Bar", "face": "right"}]}},
            {"command": "model.set.create", "params": {"name": "front", "nodes": [1, 2]}},
            {"command": "model.material.create", "params": {"name": "Steel", "elastic": {"type": "isotropic", "young": 210000, "poisson": 0.3}, "density": "7850 kg/m^3"}},
            {"command": "model.section.create", "params": {"name": "bar", "set": "all", "material": "Steel", "kind": {"type": "solid"}}},
            {"command": "model.boundary_condition.create", "params": {"name": "clamp", "type": "fixed", "target": {"set": "root"}}},
            {"command": "model.load.create", "params": {"name": "press", "type": "pressure", "set": "end", "magnitude": 2}},
            {"command": "model.step.create", "params": {"name": "Bend", "procedure": {"type": "static"}, "boundary_conditions": ["clamp"], "loads": ["press"]}},
        ]);
        s.run_script(&script).unwrap();

        let path = dir.join("bar.inp");
        let out = s.execute("deck.write", &json!({"format": "warp3d", "path": path.to_string_lossy()})).unwrap();
        let written = std::fs::read_to_string(&path).unwrap();
        let direct = fieldcraft_warp3d::write_deck(&s.model, None, None, fieldcraft_model::units::Preset::MmTonneS).unwrap();
        assert_eq!(written, direct.text);
        assert_eq!(out["format"], "warp3d");
        assert!(written.contains("type l3disop linear material Steel"));

        let cf_path = dir.join("bar_cf.inp");
        s.execute(
            "deck.write",
            &json!({
                "format": "warp3d",
                "path": cf_path.to_string_lossy(),
                "steps": ["Bend"],
                "crack_front": {
                    "set": "front",
                    "normal": [0, 1, 0],
                    "rings": 4,
                    "interaction_integral": true,
                    "function_type": "pointwise"
                }
            }),
        )
        .unwrap();
        let cf_written = std::fs::read_to_string(&cf_path).unwrap();
        assert!(cf_written.contains("domain cf_0001"));
        assert!(cf_written.contains("domain cf_0002"));
        assert!(cf_written.contains("compute domain interaction integral"));

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn crack_front_negative_and_hostile_params_are_refused() {
        let dir = std::env::temp_dir().join(format!("fieldcraft-deck-cf-neg-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();

        let mut s = Session::default();
        crate::cmd::define::tests::tagged_bar(&mut s);
        s.run_script(&json!([
            {"command": "model.set.create", "params": {"name": "all", "geometry": [{"body": "Bar"}]}},
            {"command": "model.set.create", "params": {"name": "root", "geometry": [{"body": "Bar", "face": "left"}]}},
            {"command": "model.set.create", "params": {"name": "end", "geometry": [{"body": "Bar", "face": "right"}]}},
            {"command": "model.set.create", "params": {"name": "front", "nodes": [1, 2]}},
            {"command": "model.material.create", "params": {"name": "Steel", "elastic": {"type": "isotropic", "young": 210000, "poisson": 0.3}}},
            {"command": "model.section.create", "params": {"name": "bar", "set": "all", "material": "Steel", "kind": {"type": "solid"}}},
            {"command": "model.boundary_condition.create", "params": {"name": "clamp", "type": "fixed", "target": {"set": "root"}}},
            {"command": "model.load.create", "params": {"name": "press", "type": "pressure", "set": "end", "magnitude": 2}},
            {"command": "model.step.create", "params": {"name": "Bend", "procedure": {"type": "static"}, "boundary_conditions": ["clamp"], "loads": ["press"]}},
        ]))
        .unwrap();

        let abaqus_path = dir.join("abaqus_cf.inp");
        let err = s
            .execute("deck.write", &json!({"format": "abaqus", "path": abaqus_path.to_string_lossy(), "crack_front": {"set": "front"}}))
            .unwrap_err();
        assert!(matches!(&err, EngineError::Param(p) if p.path == "crack_front"), "{err}");
        assert!(!abaqus_path.exists());

        let cases: &[(&str, Value)] = &[
            ("crack_front", json!("bad")),
            ("crack_front/set", json!({})),
            ("crack_front/set", json!({"set": "   "})),
            ("crack_front/rings", json!({"set": "front", "rings": 0})),
            ("crack_front/rings", json!({"set": "front", "rings": 21})),
            ("crack_front/rings", json!({"set": "front", "rings": "two"})),
            ("crack_front/normal", json!({"set": "front", "normal": [1.0, 0.0]})),
            ("crack_front/normal", json!({"set": "front", "normal": [null, 0.0, 1.0]})),
            ("crack_front/normal", json!({"set": "front", "normal": ["x", 0.0, 1.0]})),
            ("crack_front/symmetric", json!({"set": "front", "symmetric": "yes"})),
            ("crack_front/interaction_integral", json!({"set": "front", "interaction_integral": 1})),
            ("crack_front/function_type", json!({"set": "front", "function_type": "invalid"})),
        ];
        for (i, (expected_path, cf)) in cases.iter().enumerate() {
            let out = dir.join(format!("bad_{i}.inp"));
            let err = s.execute("deck.write", &json!({"format": "warp3d", "path": out.to_string_lossy(), "crack_front": cf})).unwrap_err();
            assert!(matches!(&err, EngineError::Param(p) if p.path == *expected_path), "case {i} ({expected_path}): {err}");
            assert!(!out.exists(), "case {i} must not write file on refusal");
        }

        let missing_set_path = dir.join("missing_set.inp");
        let err = s
            .execute("deck.write", &json!({"format": "warp3d", "path": missing_set_path.to_string_lossy(), "crack_front": {"set": "no_such_set"}}))
            .unwrap_err()
            .to_string();
        assert!(err.contains("no_such_set"), "{err}");
        assert!(!missing_set_path.exists());

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn deck_format_info_covers_all_formats() {
        for (name, ext, alt) in [("abaqus", "inp", "inp"), ("nastran", "bdf", "dat"), ("lsdyna", "k", "key"), ("warp3d", "inp", "inp")] {
            let info = deck_format_info(name).expect(name);
            assert_eq!(info.name, name);
            assert_eq!(info.extension, ext);
            assert!(info.alternatives.contains(&alt));
        }
        assert!(deck_format_info("unknown").is_none());
    }
}

#[cfg(test)]
#[path = "deck/read_tests.rs"]
mod read_tests;

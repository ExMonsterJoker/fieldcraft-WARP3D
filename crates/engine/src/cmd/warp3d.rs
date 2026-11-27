//! WARP3D pre-flight check command (`warp3d.check`), host binary status facade, and $O(1)$ form notes.

use fieldcraft_warp3d::{CheckScope, CrackFrontSpec, Warp3dBinaryStatus, Warp3dCheckReport, Warp3dQuickNote};
use serde_json::Value;

use super::{forms, opt};
use crate::{CommandKind, CommandSpec, EngineError, Result, Session};

pub const CHECK: CommandSpec = CommandSpec {
    id: "warp3d.check",
    label: "Check WARP3D Model",
    params: r#"{"step"?: "Step-1", "crack_front"?: {"set": "crack_front", "normal"?: [0, 1, 0], "symmetric"?: true, "ring_start"?: 1, "rings"?: 4, "interaction_integral"?: true, "interaction_stress_state"?: "plane_strain", "function_type"?: "pointwise", "increment_interval"?: 2}, "crack_face"?: "crack_face"}"#,
    form: Some(forms::WARP3D_CHECK),
    kind: CommandKind::Session,
    requires: &[],
    run: check,
};

/// Resolve the active WARP3D executable status for the host session (`L7 -> L6 -> L2` facade).
pub fn warp3d_binary_status(s: &Session) -> Warp3dBinaryStatus {
    fieldcraft_warp3d::resolve_warp3d_binary(s.warp3d_bin.as_deref())
}

/// Return `true` when `p` selects the external WARP3D backend explicitly or via `FIELDCRAFT_STRUCTURAL_BACKEND`.
pub fn effective_structural_backend(p: &Value) -> bool {
    super::solve::effective_structural_backend_is_warp3d(p)
}

fn extract_check_inputs(p: &Value, allow_empty_crack_set: bool) -> Result<(Option<String>, Option<CrackFrontSpec>, Option<String>)> {
    let step = opt::<Option<String>>(p, "step")?.flatten().map(|s| s.trim().to_string()).filter(|s| !s.is_empty());
    let crack_face = opt::<Option<String>>(p, "crack_face")?.flatten().map(|s| s.trim().to_string());
    let crack_front = super::deck::parse_crack_front_with(p, allow_empty_crack_set)?;
    Ok((step, crack_front, crack_face))
}

fn check(s: &mut Session, p: &Value) -> Result<Value> {
    let (step, crack_front, crack_face) = extract_check_inputs(p, true)?;
    let report = fieldcraft_warp3d::validate_model(&s.model, step.as_deref(), crack_front.as_ref(), crack_face.as_deref(), CheckScope::Export);
    serde_json::to_value(report).map_err(|e| EngineError::Params(format!("unable to serialize WARP3D check report: {e}")))
}

/// Preview WARP3D readiness on a command payload without rejecting unrelated command keys.
pub fn warp3d_check_preview(s: &Session, p: &Value, scope: CheckScope) -> Result<Warp3dCheckReport> {
    let (step, crack_front, crack_face) = extract_check_inputs(p, true)?;
    Ok(fieldcraft_warp3d::validate_model(&s.model, step.as_deref(), crack_front.as_ref(), crack_face.as_deref(), scope))
}

/// Constant-time ($O(1)$) definition preview for active WARP3D command forms (`deck.write`, `solve.static`, `solve.nonlinear`).
pub fn warp3d_quick_note(s: &Session, command: &str, p: &Value) -> Option<Warp3dQuickNote> {
    let (step, scope) = match command {
        "deck.write" => {
            if p.get("format").and_then(Value::as_str) != Some("warp3d") {
                return None;
            }
            let step = match p.get("steps") {
                Some(Value::Array(items)) if items.len() > 1 => {
                    return Some(Warp3dQuickNote {
                        summary: "WARP3D (multiple steps selected): run Check WARP3D Model for full check".into(),
                        problem: Some("WARP3D writes one structural step per deck".into()),
                    });
                }
                Some(Value::Array(items)) => items.first().and_then(Value::as_str).map(str::trim).filter(|s| !s.is_empty()),
                _ => None,
            };
            (step, CheckScope::Export)
        }
        "solve.static" | "solve.nonlinear" => {
            if !effective_structural_backend(p) {
                return None;
            }
            let step = p.get("step").and_then(Value::as_str).map(str::trim).filter(|s| !s.is_empty());
            (step, CheckScope::Solve { want_nonlinear: command == "solve.nonlinear" })
        }
        _ => return None,
    };

    let (crack_set, crack_set_omitted) = match p.get("crack_front") {
        Some(Value::Object(obj)) => match obj.get("set").and_then(Value::as_str).map(str::trim).filter(|s| !s.is_empty()) {
            Some(name) => (Some(name), false),
            None => (None, true),
        },
        _ => (None, false),
    };

    fieldcraft_warp3d::quick_note(&s.model, step, crack_set, crack_set_omitted, scope)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use fieldcraft_model::{
        BoundaryCondition, Elastic, ElementKind, Hardening, Load, Material, NonlinearStatic, Plastic, Procedure, Section, SectionKind, Set, Step,
        Target,
    };
    use serde_json::json;

    use super::*;

    fn hex8_model_with_static_and_nl(plastic: bool) -> Session {
        let mut s = Session::default();
        let coords = [
            (1, [0.0, 0.0, 0.0]),
            (2, [1.0, 0.0, 0.0]),
            (3, [1.0, 1.0, 0.0]),
            (4, [0.0, 1.0, 0.0]),
            (5, [0.0, 0.0, 1.0]),
            (6, [1.0, 0.0, 1.0]),
            (7, [1.0, 1.0, 1.0]),
            (8, [0.0, 1.0, 1.0]),
        ];
        for (id, xyz) in coords {
            s.model.set_node(id, xyz).unwrap();
        }
        s.model.set_element(1, ElementKind::Hex8, vec![1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        s.model.add_set("all", Set { elements: vec![1], ..Set::default() }).unwrap();
        s.model.add_set("fix", Set { nodes: vec![1, 2, 3, 4], ..Set::default() }).unwrap();
        s.model.add_set("top", Set { nodes: vec![5, 6, 7, 8], ..Set::default() }).unwrap();
        s.model
            .add_material(
                "Steel",
                Material {
                    elastic: Some(Elastic::Isotropic { young: 2.0e11, poisson: 0.3 }),
                    density: Some(7850.0),
                    plastic: plastic.then_some(Plastic::Bilinear { yield_stress: 2.0e8, hardening_modulus: 0.0, hardening: Hardening::Isotropic }),
                    ..Default::default()
                },
            )
            .unwrap();
        s.model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        s.model.add_boundary_condition("fix_bc", BoundaryCondition::Fixed { target: Target::Set("fix".into()) }).unwrap();
        s.model
            .add_load("pull", Load::Force { target: Target::Set("top".into()), vector: [0.0, 0.0, 100.0], distribution: Default::default() })
            .unwrap();
        s
    }

    #[test]
    fn warp3d_check_command_accepts_nulls_reports_missing_refs_and_empty_crack_front() {
        let mut s = hex8_model_with_static_and_nl(false);
        let mut step = Step::new("Step-1", Procedure::Static {});
        step.boundary_conditions.insert("fix_bc".into());
        step.loads.insert("pull".into());
        s.model.add_step(step, None).unwrap();

        let ok_nulls = s.execute("warp3d.check", &json!({"step": null, "crack_face": null, "crack_front": null})).unwrap();
        assert_eq!(ok_nulls["ready"], true);
        assert_eq!(ok_nulls["step"], "Step-1");

        let empty_cf = s.execute("warp3d.check", &json!({"crack_front": {}})).unwrap();
        assert_eq!(empty_cf["ready"], false);
        assert!(empty_cf["diagnostics"].as_array().unwrap().iter().any(|d| d["code"] == "warp3d.missing_crack_front_set"));

        let missing_face = s.execute("warp3d.check", &json!({"step": "NoStep", "crack_face": "NoFace"})).unwrap();
        assert_eq!(missing_face["ready"], false);
        let codes: Vec<&str> = missing_face["diagnostics"].as_array().unwrap().iter().filter_map(|d| d["code"].as_str()).collect();
        assert!(codes.contains(&"warp3d.missing_step"));
        assert!(codes.contains(&"warp3d.missing_crack_face"));
    }

    #[test]
    fn warp3d_check_rejects_malformed_json_types_naming_form_fields() {
        let mut s = Session::default();
        for (payload, expected) in [
            (json!({"step": 123}), "step"),
            (json!({"crack_face": []}), "crack_face"),
            (json!({"crack_front": "bad"}), "crack_front"),
            (json!({"crack_front": {"set": "cf", "normal": [1.0, 2.0]}}), "crack_front/normal"),
            (json!({"crack_front": {"set": "cf", "rings": 99}}), "crack_front/rings"),
        ] {
            let err = s.execute("warp3d.check", &payload).unwrap_err().to_string();
            assert!(err.contains(expected), "payload {payload} error `{err}` should name `{expected}`");
        }
    }

    #[test]
    fn warp3d_quick_note_handles_export_and_solve_forms_in_o1() {
        let mut s = hex8_model_with_static_and_nl(true);
        let mut s1 = Step::new("S1", Procedure::Static {});
        s1.boundary_conditions.insert("fix_bc".into());
        s1.loads.insert("pull".into());
        s.model.add_step(s1, None).unwrap();

        let mut nl1 = Step::new("NL1", Procedure::NonlinearStatic(NonlinearStatic::default()));
        nl1.boundary_conditions.insert("fix_bc".into());
        nl1.loads.insert("pull".into());
        s.model.add_step(nl1, None).unwrap();

        assert!(warp3d_quick_note(&s, "deck.write", &json!({"format": "abaqus"})).is_none());
        let multi = warp3d_quick_note(&s, "deck.write", &json!({"format": "warp3d", "steps": ["S1", "NL1"]})).unwrap();
        assert!(multi.problem.as_deref().unwrap_or_default().contains("one structural step"));

        let st_note = warp3d_quick_note(&s, "solve.static", &json!({"backend": "warp3d"})).unwrap();
        assert!(st_note.summary.contains("plastic material"), "unexpected summary: {}", st_note.summary);

        let nl_missing_cf = warp3d_quick_note(&s, "solve.nonlinear", &json!({"backend": "warp3d", "crack_front": {"set": "missing"}})).unwrap();
        assert!(nl_missing_cf.problem.as_deref().unwrap_or_default().contains("does not exist"));

        let preview = warp3d_check_preview(&s, &json!({"step": "NL1", "path": "/ignored/for/preview.inp"}), CheckScope::Export).unwrap();
        assert!(preview.ready);
        assert_eq!(preview.step.as_deref(), Some("NL1"));
    }
}

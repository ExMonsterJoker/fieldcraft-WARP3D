//! Single-source-of-truth SI pre-flight validation and $O(1)$ definition preview for WARP3D.
//!
//! Both [`crate::deck::write_deck`] and `warp3d.check` share [`validate_resolved`] so export and
//! solve readiness agree on every hard blocker and non-blocking warning in SI units. UI inline
//! forms call [`quick_note`] for a constant-time definition check that never calls
//! `Model::resolve` or iterates mesh nodes or elements.

use std::collections::{BTreeMap, BTreeSet};

use fieldcraft_model::{
    BoundaryCondition, Elastic, ElementKind, Hardening, Load, Model, Plastic, Procedure, Resolved, ResolvedSet, SectionKind, Severity, Step, Target,
};
use serde::{Deserialize, Serialize};

use crate::CrackFrontSpec;
use crate::deck::{select_step as select_export_step, warp3d_face_for_elem};
use crate::fracture::{order_crack_front, resolve_crack_normal_and_symmetry};

/// Maximum diagnostics retained in a [`Warp3dCheckReport`].
pub const MAX_DIAGNOSTICS: usize = 100;

/// Maximum entity or node labels sampled inside a single aggregated diagnostic message.
pub const MAX_SAMPLE_IDS: usize = 10;

/// Tolerance for conflicting prescribed translational displacements on the same `(node, dof)` in SI (m).
const DISPLACEMENT_CONFLICT_TOL: f64 = 1.0e-15;

/// Linear-independence threshold for unit constraint normals on crack-front nodes (Gram-Schmidt).
const CONSTRAINT_RANK_TOL: f64 = 1.0e-10;

/// Diagnostic severity for [`Warp3dDiagnostic`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DiagnosticSeverity {
    /// Blocks WARP3D export and execution.
    Error,
    /// Non-blocking condition users should review.
    Warning,
}

/// One structured diagnostic produced by WARP3D pre-flight validation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Warp3dDiagnostic {
    pub severity: DiagnosticSeverity,
    pub code: &'static str,
    pub subject: String,
    pub message: String,
}

/// Bounded report returned by [`validate_resolved`] and [`validate_model`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Warp3dCheckReport {
    pub ready: bool,
    pub step: Option<String>,
    pub errors: usize,
    pub warnings: usize,
    pub diagnostics: Vec<Warp3dDiagnostic>,
    pub truncated: usize,
}

/// Validation context distinguishing single-step deck export from typed solve commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CheckScope {
    /// Deck export (`deck.write`) or general pre-flight check (`warp3d.check`).
    Export,
    /// Solve (`solve.static` when `want_nonlinear == false`, `solve.nonlinear` when `true`).
    Solve { want_nonlinear: bool },
}

/// Constant-time definition preview shown inline on active WARP3D forms.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Warp3dQuickNote {
    pub summary: String,
    pub problem: Option<String>,
}

struct DiagnosticList(Vec<Warp3dDiagnostic>);

impl DiagnosticList {
    fn new() -> Self {
        Self(Vec::new())
    }

    fn error(&mut self, code: &'static str, subject: impl Into<String>, message: impl Into<String>) {
        self.0.push(Warp3dDiagnostic { severity: DiagnosticSeverity::Error, code, subject: subject.into(), message: message.into() });
    }

    fn warning(&mut self, code: &'static str, subject: impl Into<String>, message: impl Into<String>) {
        self.0.push(Warp3dDiagnostic { severity: DiagnosticSeverity::Warning, code, subject: subject.into(), message: message.into() });
    }

    fn finish(mut self, step: Option<String>) -> Warp3dCheckReport {
        // Stable sort puts all Error diagnostics strictly before all Warning diagnostics.
        self.0.sort_by_key(|d| d.severity);
        let errors = self.0.iter().filter(|d| d.severity == DiagnosticSeverity::Error).count();
        let warnings = self.0.len().saturating_sub(errors);
        let total = self.0.len();
        self.0.truncate(MAX_DIAGNOSTICS);
        let truncated = total.saturating_sub(self.0.len());
        Warp3dCheckReport { ready: errors == 0, step, errors, warnings, diagnostics: self.0, truncated }
    }
}

fn format_sample_u32(samples: &[u32], total: usize) -> String {
    let joined: Vec<String> = samples.iter().take(MAX_SAMPLE_IDS).map(u32::to_string).collect();
    match total.saturating_sub(joined.len()) {
        0 => joined.join(", "),
        more => format!("{}, and {more} more", joined.join(", ")),
    }
}

fn format_sample_strings<'a>(samples: impl IntoIterator<Item = &'a str>, total: usize) -> String {
    let joined: Vec<&'a str> = samples.into_iter().take(MAX_SAMPLE_IDS).collect();
    match total.saturating_sub(joined.len()) {
        0 => joined.join(", "),
        more => format!("{}, and {more} more", joined.join(", ")),
    }
}

fn is_supported_solid_element(kind: ElementKind) -> bool {
    matches!(kind, ElementKind::Hex8 | ElementKind::Wedge6 | ElementKind::Hex20 | ElementKind::Tet4 | ElementKind::Tet10)
}

fn step_subject(name: &str) -> String {
    format!("step `{name}`")
}

fn section_subject(name: &str) -> String {
    format!("section `{name}`")
}

fn material_subject(name: &str) -> String {
    format!("material `{name}`")
}

fn set_subject(name: &str) -> String {
    format!("set `{name}`")
}

fn bc_subject(name: &str) -> String {
    format!("boundary condition `{name}`")
}

fn load_subject(name: &str) -> String {
    format!("load `{name}`")
}

/// Resolve step choice for `scope` while emitting structured missing/ambiguous/procedure errors.
fn select_step_with_diagnostics<'a>(
    model: &'a Model,
    requested: Option<&str>,
    scope: CheckScope,
    diags: &mut DiagnosticList,
) -> (Option<&'a Step>, Option<String>) {
    match scope {
        CheckScope::Export => match select_export_step(model, requested) {
            Ok(step) => {
                check_step_procedure(step, scope, diags);
                (Some(step), Some(step.name.clone()))
            }
            Err(msg) => {
                let code = if requested.is_none() && model.steps().len() >= 2 { "warp3d.ambiguous_step" } else { "warp3d.missing_step" };
                diags.error(code, "model", msg);
                (None, requested.map(str::to_string))
            }
        },
        CheckScope::Solve { want_nonlinear } => {
            if let Some(name) = requested {
                if let Some(st) = model.step(name) {
                    check_step_procedure(st, scope, diags);
                    (Some(st), Some(st.name.clone()))
                } else {
                    diags.error("warp3d.missing_step", "model", format!("step `{name}` does not exist"));
                    (None, Some(name.to_string()))
                }
            } else {
                let matching: Vec<&Step> = model
                    .steps()
                    .iter()
                    .filter(|st| {
                        if want_nonlinear {
                            matches!(st.procedure, Procedure::NonlinearStatic(_))
                        } else {
                            matches!(st.procedure, Procedure::Static {})
                        }
                    })
                    .collect();
                match matching.as_slice() {
                    [only] => {
                        check_step_procedure(only, scope, diags);
                        (Some(*only), Some(only.name.clone()))
                    }
                    [] => {
                        let msg = if model.steps().is_empty() {
                            "the model has no steps to solve".to_string()
                        } else {
                            let expected = if want_nonlinear { "nonlinear static" } else { "static" };
                            format!("the model has no {expected} step to solve with WARP3D")
                        };
                        diags.error("warp3d.missing_step", "model", msg);
                        (None, None)
                    }
                    _ => {
                        diags.error("warp3d.ambiguous_step", "model", "the model has several matching steps; specify `step`");
                        (None, None)
                    }
                }
            }
        }
    }
}

fn check_step_procedure(step: &Step, scope: CheckScope, diags: &mut DiagnosticList) {
    let subj = step_subject(&step.name);
    match (scope, &step.procedure) {
        (CheckScope::Export, Procedure::Static {}) | (CheckScope::Solve { want_nonlinear: false }, Procedure::Static {}) => {}
        (CheckScope::Export, Procedure::NonlinearStatic(nl)) | (CheckScope::Solve { want_nonlinear: true }, Procedure::NonlinearStatic(nl)) => {
            if nl.arc_length.is_some() {
                diags.error("warp3d.unsupported_procedure", subj.clone(), "arc-length (Riks) control is not supported in WARP3D");
            }
            if nl.imperfection.is_some() {
                diags.error("warp3d.unsupported_procedure", subj.clone(), "buckling-mode imperfections are not supported in WARP3D");
            }
            if !nl.load_paths.is_empty() {
                diags.error("warp3d.unsupported_procedure", subj, "per-load `load_paths` are not supported in WARP3D Stage 1");
            }
        }
        (CheckScope::Solve { want_nonlinear: false }, other) => {
            diags.error(
                "warp3d.unsupported_procedure",
                subj,
                format!("step `{}` is {} (expected static for WARP3D solve.static)", step.name, other.name()),
            );
        }
        (CheckScope::Solve { want_nonlinear: true }, other) => {
            diags.error(
                "warp3d.unsupported_procedure",
                subj,
                format!("step `{}` is {} (expected nonlinear_static for WARP3D solve.nonlinear)", step.name, other.name()),
            );
        }
        (CheckScope::Export, other) => {
            diags.error(
                "warp3d.unsupported_procedure",
                subj,
                format!("procedure for step `{}` ({}) is not a structural static or nonlinear static step", step.name, other.name()),
            );
        }
    }
}

fn check_unsupported_entities(model: &Model, diags: &mut DiagnosticList) {
    if !model.couplings().is_empty() {
        diags.error("warp3d.unsupported_entity", "model", "couplings (`coupling`) are not supported in WARP3D");
    }
    if !model.reference_points().is_empty() {
        diags.error("warp3d.unsupported_entity", "model", "reference points are not supported in WARP3D");
    }
    if !model.contacts().is_empty() {
        diags.error("warp3d.unsupported_entity", "model", "contact pairs are not supported in WARP3D");
    }
    if !model.ties().is_empty() {
        diags.error("warp3d.unsupported_entity", "model", "tied surface pairs are not supported in WARP3D");
    }
    if !model.masses().is_empty() {
        diags.error("warp3d.unsupported_entity", "model", "point masses are not supported in WARP3D");
    }
    if !model.springs().is_empty() {
        diags.error("warp3d.unsupported_entity", "model", "discrete springs and dashpots are not supported in WARP3D");
    }
    if !model.initial_fields().is_empty() {
        diags.error("warp3d.unsupported_entity", "model", "initial fields are not supported in WARP3D");
    }
}

fn check_mesh_elements(model: &Model, check_node_refs: bool, diags: &mut DiagnosticList) {
    if model.nodes().is_empty() || model.elements().is_empty() {
        diags.error("warp3d.empty_mesh", "model", "model must contain nodes and 3D solid elements");
        return;
    }

    // Group unsupported elements by ElementKind and keep up to MAX_SAMPLE_IDS IDs per kind so
    // large meshes cannot flood diagnostics or slow sorting.
    let mut bad_kinds: BTreeMap<ElementKind, (usize, Vec<u32>)> = BTreeMap::new();
    let mut missing_ref_count: usize = 0;
    let mut missing_ref_samples: Vec<(u32, u32)> = Vec::new();

    for (&elem_id, elem) in model.elements() {
        if !is_supported_solid_element(elem.kind) {
            let entry = bad_kinds.entry(elem.kind).or_insert_with(|| (0, Vec::new()));
            entry.0 += 1;
            if entry.1.len() < MAX_SAMPLE_IDS {
                entry.1.push(elem_id);
            }
        }
        if check_node_refs {
            for &nid in &elem.nodes {
                if model.node(nid).is_none() {
                    missing_ref_count += 1;
                    if missing_ref_samples.len() < MAX_SAMPLE_IDS {
                        missing_ref_samples.push((elem_id, nid));
                    }
                    break;
                }
            }
        }
    }

    for (kind, (count, samples)) in bad_kinds {
        if count == 1
            && let Some(&first_id) = samples.first()
        {
            diags.error(
                "warp3d.unsupported_element",
                "model",
                format!("only 3D solid elements (Hex8, Wedge6, Hex20, Tet4, Tet10) are supported; element {first_id} is `{}`", kind.name()),
            );
        } else {
            diags.error(
                "warp3d.unsupported_element",
                "model",
                format!(
                    "only 3D solid elements (Hex8, Wedge6, Hex20, Tet4, Tet10) are supported; {count} `{}` elements found (elements {})",
                    kind.name(),
                    format_sample_u32(&samples, count)
                ),
            );
        }
    }

    if missing_ref_count > 0 {
        let pairs: Vec<String> = missing_ref_samples.iter().map(|(e, n)| format!("{e}->{n}")).collect();
        let more = missing_ref_count.saturating_sub(pairs.len());
        let suffix = if more > 0 { format!(", and {more} more") } else { String::new() };
        diags.error(
            "warp3d.missing_node_ref",
            "model",
            format!("{missing_ref_count} element(s) reference missing node(s): {}{suffix}", pairs.join(", ")),
        );
    }
}

fn resolve_set_view<'a>(model: &Model, resolved: Option<&'a Resolved>, set_name: &str) -> Option<std::borrow::Cow<'a, ResolvedSet>> {
    match resolved {
        Some(r) => r.sets.get(set_name).map(std::borrow::Cow::Borrowed),
        None => model.resolve_set_partial(set_name).map(std::borrow::Cow::Owned),
    }
}

fn check_sections_and_materials(
    model: &Model,
    resolved: Option<&Resolved>,
    step_ref: Option<&Step>,
    diags: &mut DiagnosticList,
) -> BTreeMap<u32, String> {
    let mut elem_section: BTreeMap<u32, String> = BTreeMap::new();
    let mut checked_materials: BTreeSet<&str> = BTreeSet::new();
    let mut assigned_materials: BTreeSet<&str> = BTreeSet::new();
    let is_linear_static = step_ref.is_some_and(|s| matches!(s.procedure, Procedure::Static {}));

    for (sec_name, sec) in model.sections() {
        if !matches!(sec.kind, SectionKind::Solid {}) {
            diags.error(
                "warp3d.unsupported_section",
                section_subject(sec_name),
                format!("section `{sec_name}` has kind `{}` (WARP3D only supports 3D solid sections)", sec.kind.name()),
            );
        }

        let mut sec_has_elements = false;
        if let Some(rset) = resolve_set_view(model, resolved, &sec.set) {
            sec_has_elements = !rset.elements.is_empty();
            for &eid in &rset.elements {
                elem_section.entry(eid).or_insert_with(|| sec_name.clone());
            }
        }
        if sec_has_elements {
            assigned_materials.insert(sec.material.as_str());
        }

        if sec.formulation.incompatible_modes {
            diags.warning(
                "warp3d.incompatible_modes_bbar",
                section_subject(sec_name),
                format!("section `{sec_name}` requests incompatible modes; WARP3D uses B-bar (order 2x2x2 bbar) instead"),
            );
        }

        let Some(mat) = model.material(&sec.material) else {
            diags.error(
                "warp3d.unsupported_section",
                section_subject(sec_name),
                format!("section `{sec_name}` references missing material `{}`", sec.material),
            );
            continue;
        };

        if checked_materials.insert(sec.material.as_str()) {
            let mat_subj = material_subject(&sec.material);
            if mat.hyperelastic.is_some() {
                diags.error(
                    "warp3d.unsupported_material",
                    mat_subj.clone(),
                    format!("material `{}` uses hyperelasticity, which is not supported in WARP3D Stage 1", sec.material),
                );
            }
            if mat.thermal.as_ref().and_then(|t| t.expansion).is_some() {
                diags.error(
                    "warp3d.unsupported_material",
                    mat_subj.clone(),
                    format!("material `{}` defines thermal expansion, which is not supported in WARP3D Stage 1", sec.material),
                );
            }
            match mat.elastic {
                Some(Elastic::Isotropic { .. }) => {}
                Some(Elastic::EngineeringConstants { .. }) => {
                    diags.error(
                        "warp3d.unsupported_material",
                        mat_subj.clone(),
                        format!("material `{}` uses orthotropic engineering constants; WARP3D adapter requires isotropic elasticity", sec.material),
                    );
                }
                None => {
                    diags.error("warp3d.unsupported_material", mat_subj.clone(), format!("material `{}` has no elastic properties", sec.material));
                }
            }
            match &mat.plastic {
                None => {}
                Some(Plastic::Bilinear { hardening, .. }) => {
                    if *hardening == Hardening::Kinematic {
                        diags.error(
                            "warp3d.unsupported_material",
                            mat_subj,
                            format!("material `{}` uses kinematic hardening; WARP3D bilinear plasticity supports isotropic hardening", sec.material),
                        );
                    }
                }
                Some(Plastic::Table { points }) => {
                    if points.is_empty() {
                        diags.error("warp3d.unsupported_material", mat_subj, format!("material `{}` has an empty hardening table", sec.material));
                    }
                }
                Some(Plastic::JohnsonCook { .. }) => {
                    diags.error(
                        "warp3d.unsupported_material",
                        mat_subj,
                        format!("material `{}` uses Johnson-Cook plasticity, which is not supported in WARP3D static decks", sec.material),
                    );
                }
            }
        }
    }

    if is_linear_static {
        for mat_name in assigned_materials {
            if model.material(mat_name).is_some_and(|m| m.plastic.is_some()) {
                diags.warning(
                    "warp3d.plasticity_in_static",
                    material_subject(mat_name),
                    format!(
                        "material `{mat_name}` defines plasticity on mesh elements in linear static step; WARP3D solves only 1 increment without load stepping (consider nonlinear_static)"
                    ),
                );
            }
        }
    }

    elem_section
}

fn check_face_load_targets(
    model: &Model,
    resolved: Option<&Resolved>,
    load_name: &str,
    set: &str,
    is_traction: bool,
    crack: Option<&CrackFrontSpec>,
    crack_front_nodes: &BTreeSet<u32>,
    front_face_loaded_elems: &mut BTreeSet<u32>,
    diags: &mut DiagnosticList,
) {
    let Some(rset) = resolve_set_view(model, resolved, set) else {
        return;
    };
    let error_code = if is_traction { "warp3d.invalid_traction_face" } else { "warp3d.invalid_pressure_face" };
    let kind_label = if is_traction { "traction" } else { "pressure" };

    if is_traction && rset.faces.is_empty() {
        diags.error(
            error_code,
            load_subject(load_name),
            format!("surface traction load `{load_name}` requires non-empty element faces on set `{set}`"),
        );
        return;
    }

    let mut invalid_faces: Vec<String> = Vec::new();
    let mut invalid_total: usize = 0;
    let mut wedge_cap_elems: BTreeSet<u32> = BTreeSet::new();
    let mut front_traction_elems: BTreeSet<u32> = BTreeSet::new();

    for face in &rset.faces {
        let Some(elem) = model.element(face.element) else {
            continue;
        };
        if warp3d_face_for_elem(elem, face.face, crack_front_nodes).is_none() {
            invalid_total += 1;
            if invalid_faces.len() < MAX_SAMPLE_IDS {
                invalid_faces.push(format!("invalid face {} on element {} ({})", face.face, face.element, elem.kind.name()));
            }
            continue;
        }
        let touches_front = !crack_front_nodes.is_empty() && elem.nodes.iter().any(|n| crack_front_nodes.contains(n));
        if crack.is_some() && touches_front {
            front_face_loaded_elems.insert(face.element);
            if elem.kind == ElementKind::Wedge6 && (face.face == 1 || face.face == 2) {
                wedge_cap_elems.insert(face.element);
            }
            if is_traction {
                front_traction_elems.insert(face.element);
            }
        }
    }

    if invalid_total == 1
        && let Some(first) = invalid_faces.first()
    {
        diags.error(error_code, load_subject(load_name), first.clone());
    } else if invalid_total > 1 {
        diags.error(
            error_code,
            load_subject(load_name),
            format!(
                "{invalid_total} invalid {kind_label} face(s) in `{load_name}` ({})",
                format_sample_strings(invalid_faces.iter().map(String::as_str), invalid_total)
            ),
        );
    }

    if !wedge_cap_elems.is_empty() {
        let samples: Vec<u32> = wedge_cap_elems.iter().copied().collect();
        diags.warning(
            "warp3d.wedge_cap_face_load",
            load_subject(load_name),
            format!(
                "load `{load_name}` acts on triangular end-cap face(s) of fracture-adjacent wedge6 element(s) ({}); exported for structural equilibrium, but skipped by WARP3D domain surface integration (`dielwf`)",
                format_sample_u32(&samples, samples.len())
            ),
        );
    }

    if !front_traction_elems.is_empty() && crack.is_some_and(|c| c.interaction_integral == Some(true)) {
        let samples: Vec<u32> = front_traction_elems.iter().copied().collect();
        diags.warning(
            "warp3d.interaction_crack_face_load",
            load_subject(load_name),
            format!(
                "traction load `{load_name}` acts on element(s) adjacent to the crack front ({}) with `interaction_integral` enabled; WARP3D Method-1 `II8` (`dielem_c.f`) evaluates only the in-plane advance component `t_1` and omits `t_2`/`t_3` traction contributions (`dm5` `J`/`K_J` integrates all components; use `Load::Pressure` for normal crack pressure)",
                format_sample_u32(&samples, samples.len())
            ),
        );
    }
}

fn check_step_bc_and_load_kinds(
    model: &Model,
    resolved: Option<&Resolved>,
    step_ref: &Step,
    crack: Option<&CrackFrontSpec>,
    elem_section: &BTreeMap<u32, String>,
    diags: &mut DiagnosticList,
) {
    for bc_name in &step_ref.boundary_conditions {
        if let Some(bc) = model.boundary_condition(bc_name) {
            match bc {
                BoundaryCondition::Fixed { target: Target::Set(_) }
                | BoundaryCondition::Displacement { target: Target::Set(_), .. }
                | BoundaryCondition::Symmetry { target: Target::Set(_), .. } => {}
                BoundaryCondition::Temperature { .. } => {
                    diags.error("warp3d.unsupported_bc", bc_subject(bc_name), format!("temperature BC `{bc_name}` is not supported in WARP3D"));
                }
                _ => {
                    diags.error(
                        "warp3d.unsupported_bc",
                        bc_subject(bc_name),
                        format!("boundary condition `{bc_name}` target is not supported in WARP3D"),
                    );
                }
            }
        }
    }

    let crack_front_nodes: BTreeSet<u32> =
        crack.and_then(|spec| resolve_set_view(model, resolved, &spec.set)).map(|rset| rset.nodes.iter().copied().collect()).unwrap_or_default();
    let mut front_face_loaded_elems: BTreeSet<u32> = BTreeSet::new();

    for load_name in &step_ref.loads {
        let Some(load) = model.load(load_name) else {
            continue;
        };
        match load {
            Load::Force { target: Target::Set(_), .. } | Load::BodyForce { .. } => {}
            Load::Traction { set, .. } => {
                check_face_load_targets(model, resolved, load_name, set, true, crack, &crack_front_nodes, &mut front_face_loaded_elems, diags);
            }
            Load::Pressure { set, .. } => {
                check_face_load_targets(model, resolved, load_name, set, false, crack, &crack_front_nodes, &mut front_face_loaded_elems, diags);
            }
            Load::Gravity { set, .. } => {
                let target_elems: Vec<u32> = match set.as_deref() {
                    Some(s) => resolve_set_view(model, resolved, s).map(|rs| rs.elements.clone()).unwrap_or_default(),
                    None => model.elements().keys().copied().collect(),
                };
                let mut missing_density_mats: BTreeSet<&str> = BTreeSet::new();
                for eid in target_elems {
                    if let Some(sec_name) = elem_section.get(&eid)
                        && let Some(sec) = model.section(sec_name)
                        && let Some(mat) = model.material(&sec.material)
                        && mat.density.is_none()
                    {
                        missing_density_mats.insert(sec.material.as_str());
                    }
                }
                for mat_name in missing_density_mats {
                    diags.error(
                        "warp3d.unsupported_load",
                        load_subject(load_name),
                        format!("gravity load `{load_name}` requires `density` on material `{mat_name}`"),
                    );
                }
            }
            _ => {
                diags.error(
                    "warp3d.unsupported_load",
                    load_subject(load_name),
                    format!("load `{load_name}` kind is not supported in WARP3D structural static decks"),
                );
            }
        }
    }

    // Check body/gravity loads after collecting all face-loaded front elements so load-name ordering never matters.
    if !front_face_loaded_elems.is_empty() {
        for load_name in &step_ref.loads {
            let Some(load) = model.load(load_name) else {
                continue;
            };
            let overlapping: Vec<u32> = match load {
                Load::BodyForce { set, vector } if vector.iter().any(|c| *c != 0.0) => resolve_set_view(model, resolved, set)
                    .map(|rs| {
                        let uniq: BTreeSet<u32> = rs.elements.iter().copied().filter(|eid| front_face_loaded_elems.contains(eid)).collect();
                        uniq.into_iter().collect()
                    })
                    .unwrap_or_default(),
                Load::Gravity { acceleration, set } if acceleration.iter().any(|c| *c != 0.0) => {
                    let candidates: Vec<u32> = match set.as_deref() {
                        Some(s) => resolve_set_view(model, resolved, s)
                            .map(|rs| rs.elements.iter().copied().filter(|eid| front_face_loaded_elems.contains(eid)).collect())
                            .unwrap_or_default(),
                        None => front_face_loaded_elems.iter().copied().collect(),
                    };
                    let uniq: BTreeSet<u32> = candidates
                        .into_iter()
                        .filter(|eid| {
                            elem_section
                                .get(eid)
                                .and_then(|sn| model.section(sn))
                                .and_then(|s| model.material(&s.material))
                                .is_some_and(|m| m.density.is_some_and(|rho| rho != 0.0))
                        })
                        .collect();
                    uniq.into_iter().collect()
                }
                _ => Vec::new(),
            };
            if !overlapping.is_empty() {
                diags.warning(
                    "warp3d.body_load_masks_crack_face",
                    load_subject(load_name),
                    format!(
                        "non-zero body/gravity load `{load_name}` overlaps face-loaded crack-front element(s) ({}); WARP3D `dielwf` skips element surface-traction domain integration when all element nodes carry non-zero equivalent loads",
                        format_sample_u32(&overlapping, overlapping.len())
                    ),
                );
            }
        }
    }
}

fn check_normal_spec_basic(normal: Option<[f64; 3]>) -> std::result::Result<(), &'static str> {
    let Some(raw) = normal else {
        return Ok(());
    };
    if !raw.iter().all(|c| c.is_finite()) {
        return Err("crack plane `normal` must contain finite numbers");
    }
    let scale = raw.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
    if !scale.is_finite() || scale <= DISPLACEMENT_CONFLICT_TOL {
        return Err("crack plane `normal` must be non-zero");
    }
    let scaled = raw.map(|c| c / scale);
    let norm = (scaled[0] * scaled[0] + scaled[1] * scaled[1] + scaled[2] * scaled[2]).sqrt();
    if !norm.is_finite() || norm <= DISPLACEMENT_CONFLICT_TOL {
        return Err("crack plane `normal` must be non-zero");
    }
    Ok(())
}

/// Gram-Schmidt linear-independence rank (0..=3) of translational constraint directions on a node.
fn add_constraint_direction(basis: &mut [[f64; 3]; 3], rank: &mut usize, raw: [f64; 3]) {
    if *rank >= 3 {
        return;
    }
    let mut v = raw;
    let mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if !mag.is_finite() || mag <= CONSTRAINT_RANK_TOL {
        return;
    }
    for c in &mut v {
        *c /= mag;
    }
    for b in basis.iter().take(*rank) {
        let dot = v[0] * b[0] + v[1] * b[1] + v[2] * b[2];
        v[0] -= dot * b[0];
        v[1] -= dot * b[1];
        v[2] -= dot * b[2];
    }
    let rem = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if rem.is_finite()
        && rem > CONSTRAINT_RANK_TOL
        && let Some(slot) = basis.get_mut(*rank)
    {
        *slot = [v[0] / rem, v[1] / rem, v[2] / rem];
        *rank += 1;
    }
}

/// Check resolved constraints, loads, excitation, crack front, and crack face rules.
fn check_resolved_step_and_crack(
    model: &Model,
    resolved: &Resolved,
    step_ref: Option<&Step>,
    crack: Option<&CrackFrontSpec>,
    crack_face: Option<&str>,
    elem_section: &BTreeMap<u32, String>,
    diags: &mut DiagnosticList,
) -> bool {
    let mut suppressed_no_load_warning = false;
    let crack_front_nodes: BTreeSet<u32> =
        crack.and_then(|spec| resolved.sets.get(&spec.set)).map(|rset| rset.nodes.iter().copied().collect()).unwrap_or_default();

    if let Some(st) = step_ref {
        // Symmetric displacement conflict check across Fixed, Displacement, and axis-aligned Symmetry.
        let mut assigned: BTreeMap<(u32, usize), (f64, &str)> = BTreeMap::new();
        let mut conflicts_by_bc: BTreeMap<&str, (BTreeSet<(u32, usize)>, BTreeSet<&str>)> = BTreeMap::new();
        let mut has_constraints = false;
        let mut has_nonzero_trans_disp = false;

        for bc_name in &st.boundary_conditions {
            let Some(bc) = model.boundary_condition(bc_name) else {
                continue;
            };
            match bc {
                BoundaryCondition::Fixed { target: Target::Set(set_name) } => {
                    if let Some(rset) = resolved.sets.get(set_name) {
                        for &fc_node in &rset.nodes {
                            has_constraints = true;
                            for dof in 0..3 {
                                if let Some(&(prev, first_bc)) = assigned.get(&(fc_node, dof)) {
                                    if (prev - 0.0).abs() > DISPLACEMENT_CONFLICT_TOL {
                                        let entry = conflicts_by_bc.entry(bc_name.as_str()).or_default();
                                        entry.0.insert((fc_node, dof));
                                        entry.1.insert(first_bc);
                                    }
                                } else {
                                    assigned.insert((fc_node, dof), (0.0, bc_name.as_str()));
                                }
                            }
                        }
                    }
                }
                BoundaryCondition::Displacement { target: Target::Set(set_name), values } => {
                    if values.get(3..6).is_some_and(|rots| rots.iter().any(Option::is_some)) {
                        diags.warning(
                            "warp3d.ignored_solid_rotations",
                            bc_subject(bc_name),
                            format!("boundary condition `{bc_name}` prescribes rotations on solid nodes without rotational DOFs (ignored)"),
                        );
                    }
                    if values.iter().take(3).flatten().any(|v| *v != 0.0) {
                        has_nonzero_trans_disp = true;
                    }
                    if let Some(rset) = resolved.sets.get(set_name) {
                        for &fc_node in &rset.nodes {
                            for dof in 0..3 {
                                if let Some(val) = values.get(dof).and_then(|v| *v) {
                                    has_constraints = true;
                                    if let Some(&(prev, first_bc)) = assigned.get(&(fc_node, dof)) {
                                        if (prev - val).abs() > DISPLACEMENT_CONFLICT_TOL {
                                            let entry = conflicts_by_bc.entry(bc_name.as_str()).or_default();
                                            entry.0.insert((fc_node, dof));
                                            entry.1.insert(first_bc);
                                        }
                                    } else {
                                        assigned.insert((fc_node, dof), (val, bc_name.as_str()));
                                    }
                                }
                            }
                        }
                    }
                }
                BoundaryCondition::Symmetry { target: Target::Set(set_name), plane } => {
                    if let Some(rset) = resolved.sets.get(set_name) {
                        if let Some(axis) = plane.axis() {
                            for &fc_node in &rset.nodes {
                                has_constraints = true;
                                if let Some(&(prev, first_bc)) = assigned.get(&(fc_node, axis)) {
                                    if (prev - 0.0).abs() > DISPLACEMENT_CONFLICT_TOL {
                                        let entry = conflicts_by_bc.entry(bc_name.as_str()).or_default();
                                        entry.0.insert((fc_node, axis));
                                        entry.1.insert(first_bc);
                                    }
                                } else {
                                    assigned.insert((fc_node, axis), (0.0, bc_name.as_str()));
                                }
                            }
                        } else if plane.unit_normal().is_some() && !rset.nodes.is_empty() {
                            // Axisymmetric/inclined symmetry applies multi-point constraint lines.
                            has_constraints = true;
                        }
                    }
                }
                _ => {}
            }
        }

        for (bc_name, (pairs, other_bcs)) in conflicts_by_bc {
            let sample_nodes: Vec<u32> = pairs.iter().map(|(n, _)| *n).collect();
            let total = pairs.len();
            let first_other = other_bcs.iter().next().copied().unwrap_or("");
            if total == 1
                && let Some(&(node, dof)) = pairs.iter().next()
            {
                diags.error(
                    "warp3d.conflicting_displacement",
                    bc_subject(bc_name),
                    format!("conflicting prescribed displacements on node {node} DOF {} (conflicts with `{first_other}`)", dof + 1),
                );
            } else {
                diags.error(
                    "warp3d.conflicting_displacement",
                    bc_subject(bc_name),
                    format!(
                        "conflicting prescribed displacements on {total} node DOF(s) with `{first_other}` (nodes {})",
                        format_sample_u32(&sample_nodes, total)
                    ),
                );
            }
        }

        let mut has_loads = false;
        for load_name in &st.loads {
            let Some(load) = model.load(load_name) else {
                continue;
            };
            match load {
                Load::Force { .. } => {
                    if resolved.nodal_loads.get(load_name).is_some_and(|v| !v.is_empty()) {
                        has_loads = true;
                    }
                }
                Load::Pressure { set, .. } | Load::Traction { set, .. } => {
                    if let Some(rset) = resolved.sets.get(set)
                        && rset
                            .faces
                            .iter()
                            .any(|f| model.element(f.element).is_some_and(|e| warp3d_face_for_elem(e, f.face, &crack_front_nodes).is_some()))
                    {
                        has_loads = true;
                    }
                }
                Load::BodyForce { set, .. } => {
                    if resolved.sets.get(set).is_some_and(|rs| !rs.elements.is_empty()) {
                        has_loads = true;
                    }
                }
                Load::Gravity { set, .. } => {
                    let target_elems: Vec<u32> = match set.as_deref() {
                        Some(s) => resolved.sets.get(s).map(|rs| rs.elements.clone()).unwrap_or_default(),
                        None => model.elements().keys().copied().collect(),
                    };
                    if target_elems.into_iter().any(|eid| {
                        elem_section
                            .get(&eid)
                            .and_then(|s| model.section(s))
                            .and_then(|s| model.material(&s.material))
                            .is_some_and(|m| m.density.is_some())
                    }) {
                        has_loads = true;
                    }
                }
                _ => {}
            }
        }

        if !has_loads && !has_constraints {
            diags.error(
                "warp3d.no_excitation",
                step_subject(&st.name),
                format!("step `{}` has no active structural loads or boundary conditions", st.name),
            );
        } else if has_constraints && st.loads.is_empty() && !has_nonzero_trans_disp {
            suppressed_no_load_warning = true;
            diags.warning(
                "warp3d.no_loading",
                step_subject(&st.name),
                format!("step `{}` has no active structural loads or non-zero prescribed displacements", st.name),
            );
        }
    }

    // Validate crack_front and crack_front_overconstrained.
    if let Some(spec) = crack {
        if let Some(r) = spec.rings
            && !(1..=20).contains(&r)
        {
            let subj = if spec.set.trim().is_empty() { "model".to_string() } else { set_subject(&spec.set) };
            diags.error("warp3d.invalid_crack_front", subj, format!("crack front domain `rings` must be in 1..=20, got {r}"));
        }

        if spec.set.trim().is_empty() {
            diags.error("warp3d.missing_crack_front_set", "model", "crack front `set` must name an existing node set");
        } else if let Some(rset) = resolved.sets.get(&spec.set) {
            match order_crack_front(model, &spec.set, &rset.nodes) {
                Ok(ordered) => {
                    if let Some(st) = step_ref {
                        if let Err(err) = resolve_crack_normal_and_symmetry(model, resolved, st, spec, &ordered.corners) {
                            diags.error("warp3d.invalid_crack_front", set_subject(&spec.set), err.to_string());
                        }
                    } else if let Err(msg) = check_normal_spec_basic(spec.normal) {
                        diags.error("warp3d.invalid_crack_front", set_subject(&spec.set), msg);
                    }

                    if let Some(st) = step_ref {
                        check_crack_front_overconstrained(model, resolved, st, &spec.set, &rset.nodes, &ordered, diags);
                    }
                }
                Err(err) => {
                    diags.error("warp3d.invalid_crack_front", set_subject(&spec.set), err.to_string());
                }
            }
        } else {
            diags.error("warp3d.invalid_crack_front", set_subject(&spec.set), format!("crack front node set `{}` not found", spec.set));
        }
    }

    // Validate crack_face and check non-front constrained crack-face nodes.
    if let Some(cf_name) = crack_face {
        if cf_name.trim().is_empty() {
            diags.error("warp3d.missing_crack_face", "model", "crack face set name must not be empty");
        } else if let Some(cf_rset) = resolved.sets.get(cf_name) {
            if cf_rset.nodes.is_empty() {
                diags.error("warp3d.missing_crack_face", set_subject(cf_name), format!("crack face set `{cf_name}` resolves to zero nodes"));
            } else if let Some(st) = step_ref {
                let face_interior: BTreeSet<u32> = cf_rset.nodes.iter().copied().filter(|n| !crack_front_nodes.contains(n)).collect();
                if !face_interior.is_empty() {
                    let mut constrained_nodes: BTreeSet<u32> = BTreeSet::new();
                    let mut constrained_bcs: BTreeSet<&str> = BTreeSet::new();
                    for bc_name in &st.boundary_conditions {
                        let Some(bc) = model.boundary_condition(bc_name) else {
                            continue;
                        };
                        let (set_name, constrains_trans) = match bc {
                            BoundaryCondition::Fixed { target: Target::Set(s) } => (s.as_str(), true),
                            BoundaryCondition::Displacement { target: Target::Set(s), values } => {
                                (s.as_str(), values.iter().take(3).any(Option::is_some))
                            }
                            BoundaryCondition::Symmetry { target: Target::Set(s), .. } => (s.as_str(), true),
                            _ => ("", false),
                        };
                        if !constrains_trans {
                            continue;
                        }
                        if let Some(bc_rset) = resolved.sets.get(set_name) {
                            let mut matched = false;
                            for &n in &bc_rset.nodes {
                                if face_interior.contains(&n) {
                                    constrained_nodes.insert(n);
                                    matched = true;
                                }
                            }
                            if matched {
                                constrained_bcs.insert(bc_name.as_str());
                            }
                        }
                    }
                    if !constrained_nodes.is_empty() {
                        let sample: Vec<u32> = constrained_nodes.iter().copied().take(MAX_SAMPLE_IDS).collect();
                        let bcs_str = constrained_bcs.into_iter().collect::<Vec<_>>().join(", ");
                        diags.warning(
                            "warp3d.crack_face_constrained",
                            set_subject(cf_name),
                            format!(
                                "crack face set `{cf_name}` has {} non-front node(s) constrained by active boundary condition(s) ({bcs_str}; nodes {}); crack opening requires free crack faces",
                                constrained_nodes.len(),
                                format_sample_u32(&sample, constrained_nodes.len())
                            ),
                        );
                    }
                }
            }
        } else {
            diags.error("warp3d.missing_crack_face", set_subject(cf_name), format!("crack face set `{cf_name}` does not exist"));
        }
    }

    suppressed_no_load_warning
}

fn check_crack_front_overconstrained(
    model: &Model,
    resolved: &Resolved,
    step_ref: &Step,
    set_name: &str,
    front_nodes: &[u32],
    ordered: &crate::fracture::OrderedFront,
    diags: &mut DiagnosticList,
) {
    if front_nodes.is_empty() {
        return;
    }
    let front_set: BTreeSet<u32> = front_nodes.iter().copied().collect();
    let mut node_constraints: BTreeMap<u32, ([[f64; 3]; 3], usize)> = BTreeMap::new();

    for bc_name in &step_ref.boundary_conditions {
        let Some(bc) = model.boundary_condition(bc_name) else {
            continue;
        };
        match bc {
            BoundaryCondition::Fixed { target: Target::Set(bc_set) } => {
                if let Some(rset) = resolved.sets.get(bc_set) {
                    for &n in &rset.nodes {
                        if front_set.contains(&n) {
                            let (basis, rank) = node_constraints.entry(n).or_insert(([[0.0; 3]; 3], 0));
                            add_constraint_direction(basis, rank, [1.0, 0.0, 0.0]);
                            add_constraint_direction(basis, rank, [0.0, 1.0, 0.0]);
                            add_constraint_direction(basis, rank, [0.0, 0.0, 1.0]);
                        }
                    }
                }
            }
            BoundaryCondition::Displacement { target: Target::Set(bc_set), values } => {
                let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
                if let Some(rset) = resolved.sets.get(bc_set) {
                    for &n in &rset.nodes {
                        if front_set.contains(&n) {
                            let (basis, rank) = node_constraints.entry(n).or_insert(([[0.0; 3]; 3], 0));
                            for (dof, dir) in axes.iter().enumerate() {
                                if values.get(dof).and_then(|v| *v).is_some() {
                                    add_constraint_direction(basis, rank, *dir);
                                }
                            }
                        }
                    }
                }
            }
            BoundaryCondition::Symmetry { target: Target::Set(bc_set), plane } => {
                if let Some(normal) = plane.unit_normal()
                    && let Some(rset) = resolved.sets.get(bc_set)
                {
                    for &n in &rset.nodes {
                        if front_set.contains(&n) {
                            let (basis, rank) = node_constraints.entry(n).or_insert(([[0.0; 3]; 3], 0));
                            add_constraint_direction(basis, rank, normal);
                        }
                    }
                }
            }
            _ => {}
        }
    }

    let is_open = !ordered.is_closed && ordered.corners.len() >= 2;
    let endpoints: BTreeSet<u32> =
        if is_open { [ordered.corners.first().copied(), ordered.corners.last().copied()].into_iter().flatten().collect() } else { BTreeSet::new() };

    let mut interior_or_closed_locked: Vec<u32> = Vec::new();
    let mut endpoint_locked: Vec<u32> = Vec::new();
    for (node, (_, rank)) in node_constraints {
        if rank >= 3 {
            if endpoints.contains(&node) {
                endpoint_locked.push(node);
            } else {
                interior_or_closed_locked.push(node);
            }
        }
    }

    if !interior_or_closed_locked.is_empty() {
        let total = interior_or_closed_locked.len();
        diags.warning(
            "warp3d.crack_front_overconstrained",
            set_subject(set_name),
            format!(
                "crack front set `{set_name}` has {total} interior or closed-loop node(s) fully constrained in all 3 translational DOFs (nodes {}); crack-tip opening requires at least one free DOF",
                format_sample_u32(&interior_or_closed_locked, total)
            ),
        );
    }
    if !endpoint_locked.is_empty() {
        let total = endpoint_locked.len();
        diags.warning(
            "warp3d.crack_front_overconstrained",
            set_subject(set_name),
            format!(
                "crack front set `{set_name}` has {total} single open-chain endpoint node(s) constrained in all 3 translational DOFs (endpoint nodes {}); benign if intentionally pinned for quarter-symmetry RBM restraint, otherwise overconstrained",
                format_sample_u32(&endpoint_locked, total)
            ),
        );
    }
}

/// Validate a resolved SI `Model` against WARP3D deck generation and execution constraints.
///
/// Both [`crate::deck::write_deck`] and [`validate_model`] delegate here so export and on-demand
/// checking stay consistent.
pub fn validate_resolved(
    model: &Model,
    resolved: &Resolved,
    step: Option<&str>,
    crack: Option<&CrackFrontSpec>,
    crack_face: Option<&str>,
    scope: CheckScope,
) -> Warp3dCheckReport {
    let mut diags = DiagnosticList::new();

    check_unsupported_entities(model, &mut diags);
    check_mesh_elements(model, true, &mut diags);

    let (step_ref, report_step) = select_step_with_diagnostics(model, step, scope, &mut diags);
    let elem_section = check_sections_and_materials(model, Some(resolved), step_ref, &mut diags);

    if let Some(st) = step_ref {
        check_step_bc_and_load_kinds(model, Some(resolved), st, crack, &elem_section, &mut diags);
    }

    let suppressed_no_load = check_resolved_step_and_crack(model, resolved, step_ref, crack, crack_face, &elem_section, &mut diags);

    // Forward remaining SI Model::check warnings cleanly (excluding duplicate per-step no-load warnings).
    for w in &resolved.warnings {
        if w.severity != Severity::Warning {
            continue;
        }
        let is_dup_no_load = suppressed_no_load
            && step_ref.is_some_and(|st| w.subject == step_subject(&st.name) && w.message.contains("nothing loads the model in this step"));
        if !is_dup_no_load {
            diags.warning("model.warning", w.subject.clone(), w.message.clone());
        }
    }

    diags.finish(report_step)
}

/// Resolve `model` in SI and produce a complete [`Warp3dCheckReport`].
///
/// When `Model::resolve()` fails due to general model errors, combines `Model::check()` findings
/// (`model.error` / `model.warning`) with non-resolved structural WARP3D checks so users see both
/// model and WARP3D issues in one report.
pub fn validate_model(
    model: &Model,
    step: Option<&str>,
    crack: Option<&CrackFrontSpec>,
    crack_face: Option<&str>,
    scope: CheckScope,
) -> Warp3dCheckReport {
    match model.resolve() {
        Ok(resolved) => validate_resolved(model, &resolved, step, crack, crack_face, scope),
        Err(_) => {
            let mut diags = DiagnosticList::new();
            for d in model.check() {
                match d.severity {
                    Severity::Error => diags.error("model.error", d.subject, d.message),
                    Severity::Warning => diags.warning("model.warning", d.subject, d.message),
                }
            }

            check_unsupported_entities(model, &mut diags);
            check_mesh_elements(model, false, &mut diags);

            let (step_ref, report_step) = select_step_with_diagnostics(model, step, scope, &mut diags);
            let elem_section = check_sections_and_materials(model, None, step_ref, &mut diags);

            if let Some(st) = step_ref {
                check_step_bc_and_load_kinds(model, None, st, crack, &elem_section, &mut diags);
            }

            if let Some(spec) = crack {
                if let Some(r) = spec.rings
                    && !(1..=20).contains(&r)
                {
                    let subj = if spec.set.trim().is_empty() { "model".to_string() } else { set_subject(&spec.set) };
                    diags.error("warp3d.invalid_crack_front", subj, format!("crack front domain `rings` must be in 1..=20, got {r}"));
                }
                if spec.set.trim().is_empty() {
                    diags.error("warp3d.missing_crack_front_set", "model", "crack front `set` must name an existing node set");
                } else if model.set(&spec.set).is_none() {
                    diags.error("warp3d.invalid_crack_front", set_subject(&spec.set), format!("crack front node set `{}` not found", spec.set));
                } else if let Err(msg) = check_normal_spec_basic(spec.normal) {
                    diags.error("warp3d.invalid_crack_front", set_subject(&spec.set), msg);
                }
            }

            if let Some(cf_name) = crack_face {
                if cf_name.trim().is_empty() {
                    diags.error("warp3d.missing_crack_face", "model", "crack face set name must not be empty");
                } else if model.set(cf_name).is_none() {
                    diags.error("warp3d.missing_crack_face", set_subject(cf_name), format!("crack face set `{cf_name}` does not exist"));
                }
            }

            diags.finish(report_step)
        }
    }
}

/// Constant-time ($O(1)$ w.r.t. mesh nodes/elements) definition check for WARP3D form banners.
///
/// Does **not** call `Model::resolve()` or iterate mesh nodes/elements.
pub fn quick_note(model: &Model, step: Option<&str>, crack_set: Option<&str>, crack_set_omitted: bool, scope: CheckScope) -> Option<Warp3dQuickNote> {
    let mut problems: Vec<String> = Vec::new();

    if model.nodes().is_empty() || model.elements().is_empty() {
        problems.push("model has no nodes or elements".into());
    }

    let mut unsupported_kinds = Vec::new();
    if !model.couplings().is_empty() {
        unsupported_kinds.push("couplings");
    }
    if !model.reference_points().is_empty() {
        unsupported_kinds.push("reference points");
    }
    if !model.contacts().is_empty() {
        unsupported_kinds.push("contacts");
    }
    if !model.ties().is_empty() {
        unsupported_kinds.push("ties");
    }
    if !model.masses().is_empty() {
        unsupported_kinds.push("point masses");
    }
    if !model.springs().is_empty() {
        unsupported_kinds.push("springs");
    }
    if !model.initial_fields().is_empty() {
        unsupported_kinds.push("initial fields");
    }
    if !unsupported_kinds.is_empty() {
        problems.push(format!("WARP3D does not support {}", unsupported_kinds.join(", ")));
    }

    let mut step_diags = DiagnosticList::new();
    let (step_ref, step_name) = select_step_with_diagnostics(model, step, scope, &mut step_diags);
    for d in &step_diags.0 {
        if d.severity == DiagnosticSeverity::Error {
            problems.push(d.message.clone());
        }
    }

    let is_linear_static = step_ref.is_some_and(|s| matches!(s.procedure, Procedure::Static {}));
    let mut non_solid_sections = 0usize;
    let mut has_plastic_in_static = false;
    for sec in model.sections().values() {
        if !matches!(sec.kind, SectionKind::Solid {}) {
            non_solid_sections += 1;
        }
        if let Some(mat) = model.material(&sec.material)
            && is_linear_static
            && mat.plastic.is_some()
        {
            has_plastic_in_static = true;
        }
    }
    if non_solid_sections > 0 {
        problems.push(format!("{non_solid_sections} non-solid section(s) not supported in WARP3D"));
    }

    if crack_set_omitted {
        problems.push("crack front `set` must name an existing node set".into());
    } else if let Some(cs) = crack_set
        && model.set(cs).is_none()
    {
        problems.push(format!("crack front set `{cs}` does not exist"));
    }

    let step_tag = step_name.map(|n| format!("step `{n}`")).unwrap_or_else(|| "no resolved step".into());
    let summary = if has_plastic_in_static && problems.is_empty() {
        format!("WARP3D ({step_tag}): plastic material runs 1 increment in `Static`; run Check WARP3D Model for full mesh check")
    } else {
        format!("WARP3D ({step_tag}): run Check WARP3D Model for full mesh and crack-front checks")
    };

    let problem = if problems.is_empty() { None } else { Some(problems.join("; ")) };

    Some(Warp3dQuickNote { summary, problem })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use fieldcraft_model::{
        BoundaryCondition, Elastic, ElementFace, ElementKind, Hardening, Load, Material, Model, NonlinearStatic, Plastic, Procedure, Section,
        SectionKind, Set, Step, Symmetry, Target,
    };

    use super::*;
    use crate::{CrackFunctionType, write_deck};
    use fieldcraft_model::units::Preset;

    fn unit_hex8_base() -> Model {
        let mut model = Model::default();
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
            model.set_node(id, xyz).unwrap();
        }
        model.set_element(1, ElementKind::Hex8, vec![1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("all", Set { elements: vec![1], ..Set::default() }).unwrap();
        model.add_set("fix", Set { nodes: vec![1, 4, 5, 8], ..Set::default() }).unwrap();
        model.add_set("tip", Set { nodes: vec![2, 3, 6, 7], ..Set::default() }).unwrap();
        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("fix".into()) }).unwrap();
        model
            .add_load("pull", Load::Force { target: Target::Set("tip".into()), vector: [1000.0, 0.0, 0.0], distribution: Default::default() })
            .unwrap();
        let mut step = Step::new("Pull", Procedure::Static {});
        step.boundary_conditions.insert("clamp".into());
        step.loads.insert("pull".into());
        model.add_step(step, None).unwrap();
        model
    }

    #[test]
    fn diagnostic_severity_serializes_lowercase() {
        let diag = Warp3dDiagnostic {
            severity: DiagnosticSeverity::Warning,
            code: "warp3d.no_loading",
            subject: "step `Pull`".into(),
            message: "msg".into(),
        };
        let v = serde_json::to_value(&diag).unwrap();
        assert_eq!(v["severity"], "warning");
        let err = Warp3dDiagnostic { severity: DiagnosticSeverity::Error, ..diag };
        assert_eq!(serde_json::to_value(&err).unwrap()["severity"], "error");
    }

    #[test]
    fn hard_rejects_pyramid5_elements_and_samples_up_to_10() {
        let mut model = unit_hex8_base();
        for w in 0..15u32 {
            model.set_element(100 + w, ElementKind::Pyramid5, vec![1, 2, 3, 4, 5]).unwrap();
        }
        model.replace_set("all", Set { elements: [1].into_iter().chain(100..115).collect(), ..Set::default() }).unwrap();
        let report = validate_model(&model, None, None, None, CheckScope::Export);
        assert!(!report.ready);
        let err = report.diagnostics.iter().find(|d| d.code == "warp3d.unsupported_element").unwrap();
        assert!(err.message.contains("pyramid5"));
        assert!(err.message.contains("15 `pyramid5`"));
        assert!(err.message.contains("and 5 more"));
        assert!(matches!(write_deck(&model, None, None, Preset::MKgS), Err(crate::Warp3dError::Unsupported(ref msg)) if msg.contains("pyramid5")));
    }

    #[test]
    fn displacement_controlled_step_accepted_without_loading_warning() {
        let mut model = Model::default();
        for (id, xyz) in [
            (1, [0.0, 0.0, 0.0]),
            (2, [1.0, 0.0, 0.0]),
            (3, [1.0, 1.0, 0.0]),
            (4, [0.0, 1.0, 0.0]),
            (5, [0.0, 0.0, 1.0]),
            (6, [1.0, 0.0, 1.0]),
            (7, [1.0, 1.0, 1.0]),
            (8, [0.0, 1.0, 1.0]),
        ] {
            model.set_node(id, xyz).unwrap();
        }
        model.set_element(1, ElementKind::Hex8, vec![1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("all", Set { elements: vec![1], ..Set::default() }).unwrap();
        model.add_set("fix", Set { nodes: vec![1, 4, 5, 8], ..Set::default() }).unwrap();
        model.add_set("pull_nodes", Set { nodes: vec![2, 3, 6, 7], ..Set::default() }).unwrap();
        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("fix".into()) }).unwrap();
        model
            .add_boundary_condition(
                "move_x",
                BoundaryCondition::Displacement { target: Target::Set("pull_nodes".into()), values: [Some(1.0e-4), None, None, None, None, None] },
            )
            .unwrap();
        let mut step = Step::new("Disp", Procedure::Static {});
        step.boundary_conditions.insert("clamp".into());
        step.boundary_conditions.insert("move_x".into());
        model.add_step(step, None).unwrap();

        let report = validate_model(&model, None, None, None, CheckScope::Export);
        assert!(report.ready, "{report:?}");
        assert_eq!(report.warnings, 0, "{report:?}");
        assert!(!report.diagnostics.iter().any(|d| d.code == "warp3d.no_loading"));
    }

    #[test]
    fn zero_displacement_constrained_step_warns_without_blocking() {
        let mut model = unit_hex8_base();
        let mut step = model.step("Pull").unwrap().clone();
        step.loads.clear();
        model.replace_step(step).unwrap();

        let report = validate_model(&model, None, None, None, CheckScope::Export);
        assert!(report.ready, "{report:?}");
        assert_eq!(report.diagnostics.iter().filter(|d| d.code == "warp3d.no_loading").count(), 1);
        assert!(!report.diagnostics.iter().any(|d| d.code == "model.warning" && d.message.contains("nothing loads")));
    }

    #[test]
    fn symmetric_displacement_conflict_detects_fixed_and_displacement_in_either_order() {
        let mut model = unit_hex8_base();
        model
            .add_boundary_condition(
                "a_disp",
                BoundaryCondition::Displacement { target: Target::Set("fix".into()), values: [Some(1.0e-3), None, None, None, None, None] },
            )
            .unwrap();
        let mut step = model.step("Pull").unwrap().clone();
        step.boundary_conditions.insert("a_disp".into());
        model.replace_step(step).unwrap();

        let report = validate_model(&model, None, None, None, CheckScope::Export);
        assert!(!report.ready);
        assert!(report.diagnostics.iter().any(|d| d.code == "warp3d.conflicting_displacement"));
        assert!(write_deck(&model, None, None, Preset::MKgS).is_err());
    }

    #[test]
    fn crack_face_subtracts_crack_front_and_distinguishes_quarter_symmetry_endpoints() {
        // 2 Hex8 elements along Z (0, 1, 2): crack front corners [1, 11, 21]
        // plus crack_face containing [1, 11, 21, 4, 14, 24] (same y=0 ligament/face plane or x=0 plane).
        let mut model = Model::default();
        for (k, base) in [(0, 0u32), (1, 10), (2, 20)] {
            let z = k as f64;
            model.set_node(base + 1, [1.0, 0.0, z]).unwrap();
            model.set_node(base + 2, [2.0, 0.0, z]).unwrap();
            model.set_node(base + 3, [2.0, 1.0, z]).unwrap();
            model.set_node(base + 4, [1.0, 1.0, z]).unwrap();
        }
        for i in 0..2u32 {
            let b0 = i * 10;
            let b1 = (i + 1) * 10;
            model.set_element(i + 1, ElementKind::Hex8, vec![b0 + 1, b0 + 2, b0 + 3, b0 + 4, b1 + 1, b1 + 2, b1 + 3, b1 + 4]).unwrap();
        }
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("all", Set { elements: vec![1, 2], ..Set::default() }).unwrap();
        model.add_set("front", Set { nodes: vec![1, 11, 21], ..Set::default() }).unwrap();
        // Ligament symmetry BC on y=0 touches nodes [1, 2, 11, 12, 21, 22] (which share crack front).
        model.add_set("ligament", Set { nodes: vec![1, 2, 11, 12, 21, 22], ..Set::default() }).unwrap();
        // Crack face shares front nodes [1, 11, 21] and has free non-front nodes [4, 14, 24].
        model.add_set("cface", Set { nodes: vec![1, 11, 21, 4, 14, 24], ..Set::default() }).unwrap();
        model.add_set("end_pin", Set { nodes: vec![1], ..Set::default() }).unwrap();
        model.add_set("tip", Set { nodes: vec![3, 13, 23], ..Set::default() }).unwrap();
        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_boundary_condition("sym_y", BoundaryCondition::Symmetry { target: Target::Set("ligament".into()), plane: Symmetry::Y }).unwrap();
        model.add_boundary_condition("pin_end", BoundaryCondition::Fixed { target: Target::Set("end_pin".into()) }).unwrap();
        model
            .add_load("pull", Load::Force { target: Target::Set("tip".into()), vector: [0.0, 100.0, 0.0], distribution: Default::default() })
            .unwrap();
        let mut step = Step::new("Pull", Procedure::Static {});
        step.boundary_conditions.insert("sym_y".into());
        step.boundary_conditions.insert("pin_end".into());
        step.loads.insert("pull".into());
        model.add_step(step, None).unwrap();

        let crack = CrackFrontSpec {
            set: "front".into(),
            normal: None,
            symmetric: None,
            rings: Some(3),
            interaction_integral: Some(false),
            function_type: CrackFunctionType::Pointwise,
            every_step: None,
        };
        let report = validate_model(&model, Some("Pull"), Some(&crack), Some("cface"), CheckScope::Export);
        assert!(report.ready, "{report:?}");
        // Subtracting front nodes leaves [4, 14, 24] unconstrained -> zero crack_face_constrained warnings!
        assert!(!report.diagnostics.iter().any(|d| d.code == "warp3d.crack_face_constrained"), "{report:?}");
        // Single open-chain endpoint at rank 3 triggers endpoint warning (NOT interior/closed-loop wording).
        let over: Vec<_> = report.diagnostics.iter().filter(|d| d.code == "warp3d.crack_front_overconstrained").collect();
        assert_eq!(over.len(), 1);
        assert!(over[0].message.contains("open-chain endpoint"), "{}", over[0].message);

        // Now constrain an interior front node + non-front crack face node and verify both trigger properly.
        model.add_boundary_condition("fix_interior", BoundaryCondition::Fixed { target: Target::Set("cface".into()) }).unwrap();
        let mut step2 = model.step("Pull").unwrap().clone();
        step2.boundary_conditions.insert("fix_interior".into());
        model.replace_step(step2).unwrap();
        let report2 = validate_model(&model, Some("Pull"), Some(&crack), Some("cface"), CheckScope::Export);
        assert!(report2.diagnostics.iter().any(|d| d.code == "warp3d.crack_face_constrained"));
        assert!(
            report2.diagnostics.iter().any(|d| { d.code == "warp3d.crack_front_overconstrained" && d.message.contains("interior or closed-loop") })
        );
    }

    #[test]
    fn plasticity_in_static_warns_only_for_element_assigned_sections() {
        let mut model = unit_hex8_base();
        model
            .add_material(
                "UnusedPlastic",
                Material {
                    elastic: Some(Elastic::Isotropic { young: 2.0e11, poisson: 0.3 }),
                    plastic: Some(Plastic::Bilinear { yield_stress: 2.0e8, hardening_modulus: 0.0, hardening: Hardening::Isotropic }),
                    ..Default::default()
                },
            )
            .unwrap();
        model.add_set("empty_elems", Set { nodes: vec![1], ..Set::default() }).unwrap();
        model
            .add_section(
                "unused_sec",
                Section { set: "empty_elems".into(), material: "UnusedPlastic".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();

        let report_unused = validate_model(&model, None, None, None, CheckScope::Export);
        assert!(!report_unused.diagnostics.iter().any(|d| d.code == "warp3d.plasticity_in_static"), "{report_unused:?}");

        model
            .replace_material(
                "Steel",
                Material {
                    elastic: Some(Elastic::Isotropic { young: 2.0e11, poisson: 0.3 }),
                    plastic: Some(Plastic::Bilinear { yield_stress: 2.0e8, hardening_modulus: 0.0, hardening: Hardening::Isotropic }),
                    ..Default::default()
                },
            )
            .unwrap();
        let report_used = validate_model(&model, None, None, None, CheckScope::Export);
        assert!(report_used.diagnostics.iter().any(|d| d.code == "warp3d.plasticity_in_static" && d.subject == "material `Steel`"));
    }

    #[test]
    fn resolve_error_fallback_surfaces_model_and_warp3d_blockers_together() {
        let mut s = unit_hex8_base();
        s.add_material(
            "BadMat",
            Material {
                elastic: Some(Elastic::EngineeringConstants { young: [1.0e11; 3], poisson: [0.3; 3], shear: [4.0e10; 3] }),
                ..Default::default()
            },
        )
        .unwrap();
        s.add_set("empty_el", Set { nodes: vec![1], ..Set::default() }).unwrap();
        s.replace_section(
            "solid",
            Section {
                set: "empty_el".into(),
                material: "BadMat".into(),
                kind: SectionKind::Shell { thickness: 0.01, offset: 0.0, points: 5 },
                formulation: Default::default(),
            },
        )
        .unwrap();

        assert!(s.resolve().is_err());
        let report = validate_model(&s, None, None, None, CheckScope::Export);
        assert!(!report.ready);
        assert!(report.diagnostics.iter().any(|d| d.code == "model.error"));
        assert!(report.diagnostics.iter().any(|d| d.code == "warp3d.unsupported_section"));
        assert!(report.diagnostics.iter().any(|d| d.code == "warp3d.unsupported_material"));
    }

    #[test]
    fn errors_sort_before_warnings_and_100k_element_report_caps_fast() {
        let mut model = unit_hex8_base();
        // Add 100k elements (Hex8 + 100k total to verify scan speed) plus >120 incompatible-mode sections + errors.
        for eid in 2..=100_000u32 {
            model.set_element(eid, ElementKind::Hex8, vec![1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        }
        // Add an error (`Pyramid5` element) alongside the 120+ warnings, all covered by disjoint sections.
        model.set_element(100_001, ElementKind::Pyramid5, vec![1, 2, 3, 4, 5]).unwrap();
        model.replace_set("all", Set { elements: (121..=100_001).collect(), ..Set::default() }).unwrap();
        for i in 0..120usize {
            let sname = format!("w_sec_{i:03}");
            model.add_set(&sname, Set { elements: vec![(i + 1) as u32], ..Set::default() }).unwrap();
            model
                .add_section(
                    &sname,
                    Section {
                        set: sname.clone(),
                        material: "Steel".into(),
                        kind: SectionKind::Solid {},
                        formulation: fieldcraft_model::Formulation { incompatible_modes: true, ..Default::default() },
                    },
                )
                .unwrap();
        }

        let si_resolved = model.resolve().unwrap();
        let crack = CrackFrontSpec {
            set: "missing_cf".into(),
            normal: None,
            symmetric: None,
            rings: Some(99),
            interaction_integral: None,
            function_type: CrackFunctionType::Pointwise,
            every_step: None,
        };
        let t0 = std::time::Instant::now();
        let report = validate_resolved(&model, &si_resolved, None, Some(&crack), Some("missing_face"), CheckScope::Export);
        assert!(t0.elapsed() < std::time::Duration::from_secs(1), "took {:?}", t0.elapsed());
        assert_eq!(report, validate_model(&model, None, Some(&crack), Some("missing_face"), CheckScope::Export));
        assert!(!report.ready);
        assert!(report.errors >= 3);
        assert!(report.warnings >= 120);
        assert_eq!(report.diagnostics.len(), MAX_DIAGNOSTICS);
        assert_eq!(report.truncated, report.errors + report.warnings - MAX_DIAGNOSTICS);
        // Every error appears strictly before every warning.
        let seen_warning = report.diagnostics.iter().position(|d| d.severity == DiagnosticSeverity::Warning).unwrap();
        assert!(report.diagnostics[..seen_warning].iter().all(|d| d.severity == DiagnosticSeverity::Error));
        assert!(report.diagnostics[seen_warning..].iter().all(|d| d.severity == DiagnosticSeverity::Warning));

        let q = quick_note(&model, None, Some("fix"), false, CheckScope::Export).unwrap();
        assert!(q.problem.is_none());
        let q_bad = quick_note(&model, None, None, true, CheckScope::Solve { want_nonlinear: true }).unwrap();
        assert!(q_bad.problem.is_some());
        let _ = (ElementFace { element: 1, face: 1 }, NonlinearStatic::default());
    }

    #[test]
    fn accepts_wedge6_and_checks_traction_cap_interaction_and_body_load_warnings() {
        let mut model = Model::default();
        for (id, xyz) in [
            (1, [0.0, 0.0, 0.0]),
            (2, [1.0, 0.0, 0.0]),
            (3, [1.0, 1.0, 0.0]),
            (4, [0.0, 0.0, 1.0]),
            (5, [1.0, 0.0, 1.0]),
            (6, [1.0, 1.0, 1.0]),
            (7, [0.0, 1.0, 0.0]),
            (8, [0.0, 1.0, 1.0]),
        ] {
            model.set_node(id, xyz).unwrap();
        }
        model.set_element(1, ElementKind::Wedge6, vec![1, 2, 3, 4, 5, 6]).unwrap();
        model.set_element(2, ElementKind::Hex8, vec![1, 3, 7, 2, 4, 6, 8, 5]).unwrap();
        model
            .add_material(
                "Steel",
                Material { elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }), density: Some(7850.0), ..Default::default() },
            )
            .unwrap();
        model.add_set("all", Set { elements: vec![1, 2], ..Set::default() }).unwrap();
        model.add_set("other_elems", Set { elements: vec![2], ..Set::default() }).unwrap();
        model.add_set("fix", Set { nodes: vec![2, 3, 5, 6, 7, 8], ..Set::default() }).unwrap();
        model.add_set("front", Set { nodes: vec![1, 4], ..Set::default() }).unwrap();
        model.add_set("w_quad", Set { faces: vec![ElementFace { element: 1, face: 3 }], ..Set::default() }).unwrap();
        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("fix".into()) }).unwrap();
        model.add_load("z_trac", Load::Traction { set: "w_quad".into(), vector: [0.0, 1.0e6, 0.0] }).unwrap();
        model.add_load("a_body_other", Load::BodyForce { set: "other_elems".into(), vector: [0.0, 0.0, -10.0] }).unwrap();

        // 1. Traction-only on Wedge6 quad face without crack_front -> ready, zero warnings, no cap warning.
        let mut st_ok = Step::new("TracOnly", Procedure::Static {});
        st_ok.boundary_conditions.insert("clamp".into());
        st_ok.loads.insert("z_trac".into());
        st_ok.loads.insert("a_body_other".into());
        model.add_step(st_ok, None).unwrap();

        let r_ok = validate_model(&model, Some("TracOnly"), None, None, CheckScope::Export);
        assert!(r_ok.ready, "{r_ok:?}");
        assert_eq!(r_ok.warnings, 0, "{r_ok:?}");

        // 2. Invalid Wedge6 face 6 triggers warp3d.invalid_traction_face via fallback check.
        let mut bad_model = model.clone();
        bad_model.add_set("w_bad", Set { faces: vec![ElementFace { element: 1, face: 6 }], ..Set::default() }).unwrap();
        bad_model.add_load("t_bad", Load::Traction { set: "w_bad".into(), vector: [0.0, 1.0, 0.0] }).unwrap();
        let mut st_bad = Step::new("BadFace", Procedure::Static {});
        st_bad.boundary_conditions.insert("clamp".into());
        st_bad.loads.insert("t_bad".into());
        bad_model.add_step(st_bad, None).unwrap();
        let r_bad = validate_model(&bad_model, Some("BadFace"), None, None, CheckScope::Export);
        assert!(!r_bad.ready);
        assert!(r_bad.diagnostics.iter().any(|d| d.code == "warp3d.invalid_traction_face"), "{r_bad:?}");

        // 3. With crack_front + interaction_integral + cap load + non-zero gravity (named alphabetically before z_trac).
        model.add_set("w_cap", Set { faces: vec![ElementFace { element: 1, face: 1 }], ..Set::default() }).unwrap();
        model.add_load("p_cap", Load::Pressure { set: "w_cap".into(), magnitude: 1.0e5 }).unwrap();
        model.add_load("a_grav", Load::Gravity { acceleration: [0.0, 0.0, -9.81], set: None }).unwrap();
        let mut st_crack = Step::new("CrackStep", Procedure::Static {});
        st_crack.boundary_conditions.insert("clamp".into());
        st_crack.loads.insert("z_trac".into());
        st_crack.loads.insert("p_cap".into());
        st_crack.loads.insert("a_grav".into());
        st_crack.loads.insert("a_body_other".into());
        model.add_step(st_crack, None).unwrap();

        let crack = CrackFrontSpec {
            set: "front".into(),
            normal: Some([0.0, 1.0, 0.0]),
            symmetric: Some(false),
            rings: Some(2),
            interaction_integral: Some(true),
            function_type: crate::CrackFunctionType::FrontAverage,
            every_step: None,
        };
        let r_crack = validate_model(&model, Some("CrackStep"), Some(&crack), None, CheckScope::Export);
        assert!(r_crack.ready, "{r_crack:?}");
        assert!(r_crack.diagnostics.iter().any(|d| d.code == "warp3d.wedge_cap_face_load"), "{r_crack:?}");
        assert!(r_crack.diagnostics.iter().any(|d| d.code == "warp3d.interaction_crack_face_load"), "{r_crack:?}");
        assert_eq!(
            r_crack.diagnostics.iter().filter(|d| d.code == "warp3d.body_load_masks_crack_face").count(),
            1,
            "only overlapping a_grav should warn, not non-overlapping a_body_other: {r_crack:?}"
        );
    }
}

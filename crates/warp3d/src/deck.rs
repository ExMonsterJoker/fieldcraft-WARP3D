//! WARP3D v18.5.0 ASCII `.inp` deck generator.
//!
//! Resolves the SI `Model` first for validation, converts quantities into the target
//! `Preset` through `DisplaySystem` (no hardcoded conversion factors), groups elements
//! contiguously by `(kind, section)` via `LabelMap`, and wraps all records at 72 chars.

use std::collections::{BTreeMap, BTreeSet};

use fieldcraft_model::units::{DisplaySystem, Preset};
use fieldcraft_model::{
    BoundaryCondition, Elastic, ElementKind, HardeningPoint, Integration, Load, Model, Plastic, Procedure, Resolved, Step, Target,
};

use crate::fracture::{emit_domain_commands, push_wrapped};
use crate::{CrackFrontSpec, LabelMap, Result, Warp3dError, WrittenDeck};

/// Sanitize a FieldCraft name into a unique WARP3D ASCII identifier (`[A-Za-z][A-Za-z0-9_]*`).
fn warp3d_ident(raw: &str, prefix: &str, used: &mut BTreeSet<String>) -> String {
    let mut base = String::new();
    for ch in raw.chars() {
        if ch.is_ascii_alphanumeric() || ch == '_' {
            base.push(ch);
        } else {
            base.push('_');
        }
    }
    if base.is_empty() || !base.chars().next().is_some_and(|c| c.is_ascii_alphabetic()) {
        base.insert_str(0, prefix);
    }
    // Keep identifiers compact for <= 72-char WARP3D command lines.
    base.truncate(24);
    let mut candidate = base.clone();
    let mut suffix = 2usize;
    while !used.insert(candidate.clone()) {
        let tag = format!("_{suffix}");
        let stem_len = 24usize.saturating_sub(tag.len());
        let stem: String = base.chars().take(stem_len).collect();
        candidate = format!("{stem}{tag}");
        suffix += 1;
    }
    candidate
}

/// Map FieldCraft / Abaqus solid face numbers (1-based) to WARP3D face numbers.
///
/// FieldCraft Hex faces 4 (`+xi`) and 5 (`+eta`) are swapped relative to WARP3D `l3disop`/`q3disop`.
pub fn warp3d_face(kind: ElementKind, face: u8) -> Option<u8> {
    match kind {
        ElementKind::Hex8 | ElementKind::Hex20 => match face {
            1 => Some(1),
            2 => Some(2),
            3 => Some(3),
            4 => Some(5),
            5 => Some(4),
            6 => Some(6),
            _ => None,
        },
        ElementKind::Tet4 | ElementKind::Tet10 if (1..=4).contains(&face) => Some(face),
        _ => None,
    }
}

pub(crate) fn piecewise_load_factor(points: &[[f64; 2]], t: f64) -> f64 {
    let Some(first) = points.first() else {
        return t;
    };
    if t <= first[0] {
        return first[1];
    }
    for win in points.windows(2) {
        if let &[[t0, f0], [t1, f1]] = win
            && t <= t1
        {
            let dt = t1 - t0;
            if dt.abs() <= 1.0e-15 {
                return f1;
            }
            return f0 + (f1 - f0) * ((t - t0) / dt);
        }
    }
    points.last().map_or(t, |p| p[1])
}

pub(crate) fn select_step<'a>(model: &'a Model, requested: Option<&str>) -> std::result::Result<&'a Step, String> {
    if model.steps().is_empty() {
        return Err("model has no analysis steps".into());
    }
    match requested {
        Some(name) => model.steps().iter().find(|s| s.name == name).ok_or_else(|| format!("step `{name}` not found")),
        None if model.steps().len() == 1 => model.steps().first().ok_or_else(|| "model has no analysis steps".into()),
        None => Err("WARP3D runs one structural step per deck; pass a single step name via `steps: [\"...\"]`".into()),
    }
}

fn format_element_spec(kind: ElementKind, nlgeom: bool, integration: Integration, mat_ident: &str) -> (&'static str, String) {
    let nl_kw = if nlgeom { "nonlinear" } else { "linear" };
    match kind {
        ElementKind::Hex8 => {
            let order = match integration {
                Integration::Full => "order 2x2x2 bbar",
                Integration::Reduced => "order 1x1x1",
            };
            ("l3disop", format!("type l3disop {nl_kw} material {mat_ident} {order} center_output short"))
        }
        ElementKind::Hex20 => ("q3disop", format!("type q3disop {nl_kw} material {mat_ident} order 2x2x2 bbar center_output short")),
        ElementKind::Tet4 => ("tet4", format!("type tet4 {nl_kw} material {mat_ident} order 1pt center_output short")),
        ElementKind::Tet10 => ("tet10", format!("type tet10 {nl_kw} material {mat_ident} order 4pt center_output short")),
        _ => ("unknown", String::new()),
    }
}

/// Write a complete WARP3D v18.5.0 `.inp` deck for one structural step of `model` in the
/// consistent `units` system.
///
/// Resolving in SI first preserves FieldCraft's calibrated validation checks before converting
/// all geometry, materials, constraints, and loads via [`DisplaySystem`] for WARP3D's unitless
/// deck.
pub fn write_deck(model: &Model, step: Option<&str>, crack: Option<&CrackFrontSpec>, units: Preset) -> Result<WrittenDeck> {
    // 1. Validate & resolve in SI first via the shared pre-flight validator.
    let si_resolved = model.resolve()?;
    let report = crate::check::validate_resolved(model, &si_resolved, step, crack, None, crate::check::CheckScope::Export);
    if !report.ready {
        let reasons: BTreeSet<String> =
            report.diagnostics.iter().filter(|d| d.severity == crate::check::DiagnosticSeverity::Error).map(|d| d.message.clone()).collect();
        let joined = reasons.into_iter().collect::<Vec<_>>().join("; ");
        return Err(Warp3dError::Unsupported(joined));
    }
    let warnings: Vec<String> = report
        .diagnostics
        .iter()
        .filter(|d| d.severity == crate::check::DiagnosticSeverity::Warning)
        .map(|d| if d.message.starts_with(&d.subject) { d.message.clone() } else { format!("{}: {}", d.subject, d.message) })
        .collect();

    // 2. Convert coordinates, material properties, BCs, and loads into `units`.
    let display = DisplaySystem::new(units);
    let expressed = model.converted(&|q, v| display.express(q, v).ok())?;
    let resolved: Resolved = expressed.resolve()?;

    let mut elem_section: BTreeMap<u32, String> = BTreeMap::new();
    for (sec_name, sec) in expressed.sections() {
        if let Some(rset) = resolved.sets.get(&sec.set) {
            for &eid in &rset.elements {
                elem_section.entry(eid).or_insert_with(|| sec_name.clone());
            }
        }
    }

    let step_ref = select_step(&expressed, step).map_err(Warp3dError::Unsupported)?;
    let label_map = LabelMap::from_model(&expressed);

    let mut used_idents: BTreeSet<String> = BTreeSet::new();
    let step_ident = warp3d_ident(&step_ref.name, "s_", &mut used_idents);
    let load_pat_ident = format!("{step_ident}_l");

    // Assign WARP3D identifiers and curve IDs to used materials.
    let mut used_materials: BTreeSet<String> = BTreeSet::new();
    for sec in expressed.sections().values() {
        used_materials.insert(sec.material.clone());
    }
    let mut mat_idents: BTreeMap<String, String> = BTreeMap::new();
    let mut stress_strain_curves: Vec<(usize, &Elastic, &Vec<HardeningPoint>)> = Vec::new();
    let mut mat_curve_ids: BTreeMap<String, usize> = BTreeMap::new();

    for mat_name in &used_materials {
        let ident = warp3d_ident(mat_name, "m_", &mut used_idents);
        mat_idents.insert(mat_name.clone(), ident);
        if let Some(mat) = expressed.material(mat_name)
            && let (Some(elastic), Some(Plastic::Table { points })) = (&mat.elastic, &mat.plastic)
        {
            let cid = stress_strain_curves.len() + 1;
            stress_strain_curves.push((cid, elastic, points));
            mat_curve_ids.insert(mat_name.clone(), cid);
        }
    }

    let (nlgeom, n_steps, max_iters, conv_tol, load_factors) = match &step_ref.procedure {
        Procedure::Static {} => (false, 1usize, 16u32, 1.0e-5, vec![1.0]),
        Procedure::NonlinearStatic(nl) => {
            let n = ((1.0 / nl.initial_increment).round() as usize).clamp(1, nl.max_increments.max(1) as usize);
            let mut factors = Vec::with_capacity(n);
            let mut prev_lam = 0.0;
            for k in 1..=n {
                let t = k as f64 / n as f64;
                let lam = match &nl.load_path {
                    Some(pts) => piecewise_load_factor(pts, t),
                    None => t,
                };
                factors.push(lam - prev_lam);
                prev_lam = lam;
            }
            (nl.nonlinear_geometry, n, nl.max_iterations.max(1), nl.tolerances.force, factors)
        }
        _ => (false, 1, 16, 1.0e-5, vec![1.0]),
    };

    // Assemble constraints per WARP3D node and inclined MPC lines.
    let mut nodal_bcs: BTreeMap<u32, [Option<f64>; 3]> = BTreeMap::new();
    let mut mpc_lines: Vec<String> = Vec::new();
    for bc_name in &step_ref.boundary_conditions {
        let Some(bc) = expressed.boundary_condition(bc_name) else {
            continue;
        };
        match bc {
            BoundaryCondition::Fixed { target: Target::Set(set_name) } => {
                if let Some(rset) = resolved.sets.get(set_name) {
                    for fc_node in &rset.nodes {
                        if let Some(&wn) = label_map.fc_node_to_warp.get(fc_node) {
                            let entry = nodal_bcs.entry(wn).or_insert([None; 3]);
                            *entry = [Some(0.0), Some(0.0), Some(0.0)];
                        }
                    }
                }
            }
            BoundaryCondition::Displacement { target: Target::Set(set_name), values } => {
                if let Some(rset) = resolved.sets.get(set_name) {
                    for fc_node in &rset.nodes {
                        if let Some(&wn) = label_map.fc_node_to_warp.get(fc_node) {
                            let entry = nodal_bcs.entry(wn).or_insert([None; 3]);
                            for dof in 0..3 {
                                if let Some(val) = values.get(dof).and_then(|v| *v)
                                    && let Some(slot) = entry.get_mut(dof)
                                {
                                    *slot = Some(val);
                                }
                            }
                        }
                    }
                }
            }
            BoundaryCondition::Symmetry { target: Target::Set(set_name), plane } => {
                if let Some(rset) = resolved.sets.get(set_name) {
                    if let Some(axis) = plane.axis() {
                        for fc_node in &rset.nodes {
                            if let Some(&wn) = label_map.fc_node_to_warp.get(fc_node)
                                && let Some(slot) = nodal_bcs.entry(wn).or_insert([None; 3]).get_mut(axis)
                            {
                                *slot = Some(0.0);
                            }
                        }
                    } else if let Some(n) = plane.unit_normal() {
                        for fc_node in &rset.nodes {
                            if let Some(&wn) = label_map.fc_node_to_warp.get(fc_node) {
                                mpc_lines.push(format!("  {wn} u {:.8e} + {wn} v {:.8e} + {wn} w {:.8e} = 0.0", n[0], n[1], n[2]));
                            }
                        }
                    }
                }
            }
            _ => {}
        }
    }

    // Collect nodal loads, pressures, and body forces.
    let mut nodal_forces: BTreeMap<u32, [f64; 3]> = BTreeMap::new();
    let mut pressure_loads: Vec<(u32, u8, f64)> = Vec::new();
    let mut body_forces: BTreeMap<u32, [f64; 3]> = BTreeMap::new();

    for load_name in &step_ref.loads {
        let Some(load) = expressed.load(load_name) else {
            continue;
        };
        match load {
            Load::Force { .. } | Load::Traction { .. } => {
                if let Some(items) = resolved.nodal_loads.get(load_name) {
                    for (fc_node, vec) in items {
                        if let Some(&wn) = label_map.fc_node_to_warp.get(fc_node) {
                            let acc = nodal_forces.entry(wn).or_insert([0.0; 3]);
                            acc[0] += vec[0];
                            acc[1] += vec[1];
                            acc[2] += vec[2];
                        }
                    }
                }
            }
            Load::Pressure { set, magnitude } => {
                if let Some(rset) = resolved.sets.get(set) {
                    for face in &rset.faces {
                        let Some(elem) = expressed.element(face.element) else {
                            continue;
                        };
                        let Some(&we) = label_map.fc_elem_to_warp.get(&face.element) else {
                            continue;
                        };
                        let wface = warp3d_face(elem.kind, face.face).ok_or_else(|| {
                            Warp3dError::Unsupported(format!("invalid face {} on element {} ({})", face.face, face.element, elem.kind.name()))
                        })?;
                        pressure_loads.push((we, wface, *magnitude));
                    }
                }
            }
            Load::BodyForce { set, vector } => {
                if let Some(rset) = resolved.sets.get(set) {
                    for &fc_elem in &rset.elements {
                        if let Some(&we) = label_map.fc_elem_to_warp.get(&fc_elem) {
                            let acc = body_forces.entry(we).or_insert([0.0; 3]);
                            acc[0] += vector[0];
                            acc[1] += vector[1];
                            acc[2] += vector[2];
                        }
                    }
                }
            }
            Load::Gravity { acceleration, set } => {
                let target_elems: Vec<u32> = match set.as_deref() {
                    Some(s) => resolved.sets.get(s).map(|rs| rs.elements.clone()).unwrap_or_default(),
                    None => expressed.elements().keys().copied().collect(),
                };
                for fc_elem in target_elems {
                    if let Some(&we) = label_map.fc_elem_to_warp.get(&fc_elem)
                        && let Some(sec_name) = elem_section.get(&fc_elem)
                        && let Some(sec) = expressed.section(sec_name)
                        && let Some(mat) = expressed.material(&sec.material)
                        && let Some(rho) = mat.density
                    {
                        let acc = body_forces.entry(we).or_insert([0.0; 3]);
                        acc[0] += rho * acceleration[0];
                        acc[1] += rho * acceleration[1];
                        acc[2] += rho * acceleration[2];
                    }
                }
            }
            _ => {}
        }
    }

    let has_loads = !nodal_forces.is_empty() || !pressure_loads.is_empty() || !body_forces.is_empty();
    let has_constraints = !nodal_bcs.is_empty() || !mpc_lines.is_empty();

    // 3. Emit WARP3D deck sections.
    let mut out = String::with_capacity(4096);
    push_wrapped(&mut out, "! FieldCraft WARP3D v18.5.0 Input Deck");
    push_wrapped(&mut out, &format!("! fieldcraft-step: {}", step_ref.name));
    push_wrapped(&mut out, &format!("! fieldcraft-units: {}", units.label()));
    push_wrapped(&mut out, "!");
    push_wrapped(&mut out, "structure fc_model");
    push_wrapped(&mut out, "!");

    // WARP3D expects (total_strain, yield_stress) pairs, whereas FieldCraft stores plastic strain.
    // Emit curves before materials so `curve <id>` forward-references remain valid.
    for (cid, elastic, points) in &stress_strain_curves {
        let young = match elastic {
            Elastic::Isotropic { young, .. } => *young,
            _ => continue,
        };
        push_wrapped(&mut out, &format!("stress-strain curve {cid}"));
        let mut pairs = Vec::with_capacity(points.len());
        for pt in *points {
            let eps_total = pt.stress / young + pt.strain;
            pairs.push(format!("{eps_total:.8e} {:.8e}", pt.stress));
        }
        push_wrapped(&mut out, &format!("  {}", pairs.join(" ")));
    }

    for mat_name in &used_materials {
        let Some(mat) = expressed.material(mat_name) else {
            continue;
        };
        let Some(Elastic::Isotropic { young, poisson }) = mat.elastic else {
            continue;
        };
        let Some(m_ident) = mat_idents.get(mat_name) else {
            continue;
        };
        push_wrapped(&mut out, &format!("material {m_ident}"));
        // Set `rho 0.0` so quasi-static equilibrium omits inertial forces (gravity is emitted as body forces).
        match &mat.plastic {
            None => {
                push_wrapped(&mut out, &format!("  properties mises e {young:.8e} nu {poisson:.8} yld_pt 1.0e20 n_power 1.0 alpha 0.0 rho 0.0"));
            }
            Some(Plastic::Bilinear { yield_stress, hardening_modulus, .. }) => {
                // Convert FieldCraft plastic hardening modulus H to WARP3D total-strain tangent modulus E_t.
                let tan_e = if *hardening_modulus <= 0.0 { 0.0 } else { (young * hardening_modulus) / (young + hardening_modulus) };
                push_wrapped(
                    &mut out,
                    &format!("  properties bilinear e {young:.8e} nu {poisson:.8} yld_pt {yield_stress:.8e} tan_e {tan_e:.8e} alpha 0.0 rho 0.0"),
                );
            }
            Some(Plastic::Table { .. }) => {
                if let Some(&cid) = mat_curve_ids.get(mat_name) {
                    push_wrapped(&mut out, &format!("  properties mises e {young:.8e} nu {poisson:.8} curve {cid} alpha 0.0 rho 0.0"));
                }
            }
            Some(Plastic::JohnsonCook { .. }) => {}
        }
    }

    push_wrapped(&mut out, &format!("number of nodes {} elements {}", expressed.nodes().len(), expressed.elements().len()));

    // Elements grouped contiguously by (kind, section_name) matching LabelMap order.
    push_wrapped(&mut out, "elements");
    let mut group_start: Option<(u32, ElementKind, String)> = None;
    let mut prev_w_id = 0u32;

    let flush_group = |out: &mut String, start_w: u32, end_w: u32, kind: ElementKind, sec_name: &str| {
        let Some(sec) = expressed.section(sec_name) else {
            return;
        };
        let Some(m_ident) = mat_idents.get(&sec.material) else {
            return;
        };
        let (_, spec_str) = format_element_spec(kind, nlgeom, sec.formulation.integration, m_ident);
        let range = if start_w == end_w { format!("{start_w}") } else { format!("{start_w}-{end_w}") };
        push_wrapped(out, &format!("  {range} {spec_str}"));
    };

    for (idx, &fc_elem_id) in label_map.warp_elem_to_fc.iter().enumerate() {
        let w_id = (idx + 1) as u32;
        let Some(elem) = expressed.element(fc_elem_id) else {
            continue;
        };
        let sec_name = elem_section.get(&fc_elem_id).cloned().unwrap_or_default();
        match &group_start {
            Some((s_id, s_kind, s_sec)) if *s_kind == elem.kind && *s_sec == sec_name => {
                prev_w_id = w_id;
                let _ = s_id;
            }
            Some((s_id, s_kind, s_sec)) => {
                flush_group(&mut out, *s_id, prev_w_id, *s_kind, s_sec);
                group_start = Some((w_id, elem.kind, sec_name));
                prev_w_id = w_id;
            }
            None => {
                group_start = Some((w_id, elem.kind, sec_name));
                prev_w_id = w_id;
            }
        }
    }
    if let Some((s_id, s_kind, s_sec)) = group_start {
        flush_group(&mut out, s_id, prev_w_id, s_kind, &s_sec);
    }

    // Coordinates.
    push_wrapped(&mut out, "coordinates");
    for (idx, &fc_node_id) in label_map.warp_node_to_fc.iter().enumerate() {
        let w_id = idx + 1;
        if let Some(node) = expressed.node(fc_node_id) {
            push_wrapped(&mut out, &format!("  {w_id} {:.10e} {:.10e} {:.10e}", node.xyz[0], node.xyz[1], node.xyz[2]));
        }
    }

    // Element incidences (FieldCraft solid node order matches WARP3D directly).
    push_wrapped(&mut out, "incidences");
    for (idx, &fc_elem_id) in label_map.warp_elem_to_fc.iter().enumerate() {
        let w_id = idx + 1;
        if let Some(elem) = expressed.element(fc_elem_id) {
            let mut node_strs = Vec::with_capacity(elem.nodes.len());
            for fc_node in &elem.nodes {
                let wn = label_map
                    .fc_node_to_warp
                    .get(fc_node)
                    .copied()
                    .ok_or_else(|| Warp3dError::Unsupported(format!("element {fc_elem_id} references missing node {fc_node}")))?;
                node_strs.push(wn.to_string());
            }
            push_wrapped(&mut out, &format!("  {w_id} {}", node_strs.join(" ")));
        }
    }

    // Let WARP3D automatically partition elements into SIMD/OpenMP-friendly blocks of <= 128 elements.
    push_wrapped(&mut out, "blocking automatic size = 128 display");

    // Constraints.
    if has_constraints {
        push_wrapped(&mut out, "constraints");
        for (w_node, dofs) in &nodal_bcs {
            let mut parts = Vec::new();
            for (axis, kw) in ["u", "v", "w"].iter().enumerate() {
                if let Some(val) = dofs.get(axis).and_then(|v| *v) {
                    parts.push(format!("{kw} {val:.8e}"));
                }
            }
            if !parts.is_empty() {
                push_wrapped(&mut out, &format!("  {w_node} {}", parts.join(" ")));
            }
        }
        for mpc in &mpc_lines {
            push_wrapped(&mut out, mpc);
        }
    }

    // Load pattern.
    if has_loads {
        push_wrapped(&mut out, &format!("loading {load_pat_ident}"));
        if !nodal_forces.is_empty() {
            push_wrapped(&mut out, "  nodal loads");
            for (w_node, f) in &nodal_forces {
                let mut comps = Vec::new();
                if f[0].abs() > 0.0 {
                    comps.push(format!("force_x {:.8e}", f[0]));
                }
                if f[1].abs() > 0.0 {
                    comps.push(format!("force_y {:.8e}", f[1]));
                }
                if f[2].abs() > 0.0 {
                    comps.push(format!("force_z {:.8e}", f[2]));
                }
                if comps.is_empty() {
                    comps.push("force_x 0.0".into());
                }
                push_wrapped(&mut out, &format!("    {w_node} {}", comps.join(" ")));
            }
        }
        if !pressure_loads.is_empty() || !body_forces.is_empty() {
            push_wrapped(&mut out, "  element loads");
            for &(w_elem, w_face, mag) in &pressure_loads {
                push_wrapped(&mut out, &format!("    {w_elem} face {w_face} pressure {mag:.8e}"));
            }
            for (w_elem, b) in &body_forces {
                push_wrapped(&mut out, &format!("    {w_elem} body forces bx {:.8e} by {:.8e} bz {:.8e}", b[0], b[1], b[2]));
            }
        }
    }

    // Nonlinear driver loading + step schedule.
    push_wrapped(&mut out, &format!("loading {step_ident}"));
    push_wrapped(&mut out, "  nonlinear");
    let first_factor = load_factors.first().copied().unwrap_or(1.0);
    let uniform = load_factors.iter().all(|&f| (f - first_factor).abs() <= 1.0e-15);
    if uniform {
        let range = if n_steps == 1 { "step 1".into() } else { format!("steps 1-{n_steps}") };
        if has_loads && has_constraints {
            push_wrapped(&mut out, &format!("    {range} {load_pat_ident} {first_factor:.8e} constraints {first_factor:.8e}"));
        } else if has_loads {
            push_wrapped(&mut out, &format!("    {range} {load_pat_ident} {first_factor:.8e}"));
        } else {
            push_wrapped(&mut out, &format!("    {range} constraints {first_factor:.8e}"));
        }
    } else {
        for (idx, &factor) in load_factors.iter().enumerate() {
            let step_no = idx + 1;
            if has_loads && has_constraints {
                push_wrapped(&mut out, &format!("    step {step_no} {load_pat_ident} {factor:.8e} constraints {factor:.8e}"));
            } else if has_loads {
                push_wrapped(&mut out, &format!("    step {step_no} {load_pat_ident} {factor:.8e}"));
            } else {
                push_wrapped(&mut out, &format!("    step {step_no} constraints {factor:.8e}"));
            }
        }
    }

    push_wrapped(&mut out, "nonlinear analysis parameters");
    push_wrapped(&mut out, "  solution technique sparse direct");
    push_wrapped(&mut out, &format!("  maximum iterations {max_iters}"));
    push_wrapped(&mut out, &format!("  convergence test norm res tol {conv_tol:.4e}"));
    // Large pseudo-time step suppresses spurious rate/inertia effects alongside `rho 0.0`.
    push_wrapped(&mut out, "  time step 1.0e06");
    push_wrapped(&mut out, "  trace solution on");
    push_wrapped(&mut out, "  extrapolate on");

    for k in 1..=n_steps {
        push_wrapped(&mut out, &format!("compute displacements for loading {step_ident} step {k}"));
        push_wrapped(&mut out, "output flat text nodal displacements");
        push_wrapped(&mut out, "output flat text nodal stresses");
        push_wrapped(&mut out, "output flat text nodal reactions");
        push_wrapped(&mut out, "output flat text element stresses");
    }

    if let Some(spec) = crack {
        emit_domain_commands(&mut out, &expressed, &resolved, step_ref, &label_map, spec)?;
    }

    push_wrapped(&mut out, "stop");

    Ok(WrittenDeck { text: out, label_map, step_name: step_ref.name.clone(), step_count: n_steps, units, warnings })
}

#[cfg(test)]
mod tests {
    use fieldcraft_model::{
        BoundaryCondition, ElementKind, Hardening, Integration, Load, Material, Model, NonlinearStatic, Plastic, Procedure, Section, SectionKind,
        Set, Step, Target,
    };

    use super::*;
    use crate::CrackFunctionType;

    fn unit_hex8_model() -> Model {
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
    fn write_deck_single_hex8_cantilever() {
        let model = unit_hex8_model();
        let written = write_deck(&model, None, None, Preset::MmTonneS).unwrap();
        assert_eq!(written.step_name, "Pull");
        assert_eq!(written.step_count, 1);
        assert!(written.text.contains("! fieldcraft-units: mm-t-s-N-MPa"));
        assert!(written.text.contains("type l3disop linear material Steel order 2x2x2 bbar"));
        assert!(written.text.contains("rho 0.0"));
        assert!(written.text.contains("time step 1.0e06"));
        assert!(written.text.contains("blocking automatic size = 128 display"));
        assert!(written.text.contains("output flat text nodal displacements"));
        assert!(written.text.contains("output flat text nodal stresses"));
        assert!(written.text.contains("output flat text nodal reactions"));
        assert!(written.text.contains("output flat text element stresses"));
        assert!(written.text.contains("stop\n"));
        for line in written.text.lines() {
            if !line.starts_with("! ") && line != "!" {
                assert!(line.len() <= 72, "line exceeded 72 chars ({}): {line}", line.len());
            }
        }
    }

    #[test]
    fn write_deck_hex20_tet4_tet10_gravity_and_face_swap() {
        let mut model = Model::default();
        // Hex8 elem 10 with face 4 pressure, Tet4 elem 20 with face 4 pressure.
        let coords = [
            (1, [0.0, 0.0, 0.0]),
            (2, [1.0, 0.0, 0.0]),
            (3, [1.0, 1.0, 0.0]),
            (4, [0.0, 1.0, 0.0]),
            (5, [0.0, 0.0, 1.0]),
            (6, [1.0, 0.0, 1.0]),
            (7, [1.0, 1.0, 1.0]),
            (8, [0.0, 1.0, 1.0]),
            (9, [2.0, 0.0, 0.0]),
        ];
        for (id, xyz) in coords {
            model.set_node(id, xyz).unwrap();
        }
        model.set_element(10, ElementKind::Hex8, vec![1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        model.set_element(20, ElementKind::Tet4, vec![2, 9, 3, 6]).unwrap();
        model
            .add_material(
                "Steel",
                Material {
                    elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }),
                    density: Some(8000.0),
                    plastic: Some(Plastic::Bilinear { yield_stress: 2.5e8, hardening_modulus: 2.1e9, hardening: Hardening::Isotropic }),
                    ..Default::default()
                },
            )
            .unwrap();
        model.add_set("all_elems", Set { elements: vec![10, 20], ..Set::default() }).unwrap();
        model.add_set("fix", Set { nodes: vec![1, 4, 5, 8], ..Set::default() }).unwrap();
        model
            .add_set(
                "press_faces",
                Set {
                    faces: vec![fieldcraft_model::ElementFace { element: 10, face: 4 }, fieldcraft_model::ElementFace { element: 20, face: 4 }],
                    ..Set::default()
                },
            )
            .unwrap();
        model
            .add_section(
                "s",
                Section {
                    set: "all_elems".into(),
                    material: "Steel".into(),
                    kind: SectionKind::Solid {},
                    formulation: fieldcraft_model::Formulation {
                        integration: Integration::Full,
                        incompatible_modes: false,
                        hybrid: false,
                        ..fieldcraft_model::Formulation::default()
                    },
                },
            )
            .unwrap();
        model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("fix".into()) }).unwrap();
        model.add_load("p", Load::Pressure { set: "press_faces".into(), magnitude: 1.0e5 }).unwrap();
        model.add_load("g", Load::Gravity { acceleration: [0.0, 0.0, -9.81], set: None }).unwrap();
        let mut step = Step::new(
            "NL",
            Procedure::NonlinearStatic(NonlinearStatic { initial_increment: 0.25, max_increments: 20, ..NonlinearStatic::default() }),
        );
        step.boundary_conditions.insert("clamp".into());
        step.loads.insert("p".into());
        step.loads.insert("g".into());
        model.add_step(step, None).unwrap();

        let written = write_deck(&model, Some("NL"), None, Preset::MKgS).unwrap();
        assert_eq!(written.step_count, 4);
        assert!(written.text.contains("properties bilinear"));
        assert!(written.text.contains("rho 0.0"));
        // Hex face 4 maps to WARP3D face 5; Tet face 4 stays 4.
        assert!(written.text.contains("1 face 5 pressure"));
        assert!(written.text.contains("2 face 4 pressure"));
        assert!(written.text.contains("body forces bx 0.00000000e0 by 0.00000000e0 bz"));
        assert!(written.text.contains("steps 1-4 NL_l 2.50000000e-1 constraints 2.50000000e-1"));
        assert!(written.text.contains("compute displacements for loading NL step 4"));
    }

    #[test]
    fn write_deck_tabular_plasticity_and_crack_refusals() {
        let mut model = unit_hex8_model();
        model
            .replace_material(
                "Steel",
                Material {
                    elastic: Some(Elastic::Isotropic { young: 2.0e11, poisson: 0.3 }),
                    plastic: Some(Plastic::Table {
                        points: vec![HardeningPoint { strain: 0.0, stress: 2.0e8 }, HardeningPoint { strain: 0.05, stress: 4.0e8 }],
                    }),
                    ..Default::default()
                },
            )
            .unwrap();
        let crack = CrackFrontSpec {
            set: "fix".into(),
            normal: Some([0.0, 1.0, 0.0]),
            symmetric: Some(false),
            rings: Some(4),
            interaction_integral: Some(true),
            function_type: CrackFunctionType::FrontAverage,
        };
        // `fix` forms a quad loop [1,4,8,5], check open edge subset instead:
        model.add_set("front_edge", Set { nodes: vec![1, 5], ..Set::default() }).unwrap();
        let valid_crack = CrackFrontSpec { set: "front_edge".into(), ..crack };
        let written = write_deck(&model, None, Some(&valid_crack), Preset::MKgS).unwrap();
        let logical = written.text.replace(" ,\n  ", " ");
        assert!(logical.contains("stress-strain curve 1"));
        assert!(logical.contains("curve 1 alpha 0.0 rho 0.0"));
        assert!(logical.contains("domain cf_0001"));
        assert!(logical.contains("function type d"));
        assert!(logical.contains("compute domain interaction integral"));
        for line in written.text.lines() {
            if !line.starts_with("! ") && line != "!" {
                assert!(line.len() <= 72, "line exceeded 72 chars ({}): {line}", line.len());
            }
        }
    }

    #[test]
    fn write_deck_hex20_q3disop_and_tet10_faces_1_to_4() {
        let mut model = Model::default();
        // Build a 20-node Hex20 at x in [0, 1] and a 10-node Tet10 at x in [2, 3].
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
        model.set_element(100, ElementKind::Hex20, (1..=20).collect()).unwrap();

        let tet_corners = [[2.0, 0.0, 0.0], [3.0, 0.0, 0.0], [2.0, 1.0, 0.0], [2.0, 0.0, 1.0]];
        let mut tet_xyz = tet_corners.to_vec();
        tet_xyz.resize(10, [0.0; 3]);
        for e in ElementKind::Tet10.edges() {
            if let [a, b, m] = **e {
                tet_xyz[m] = [0, 1, 2].map(|i| 0.5 * (tet_corners[a][i] + tet_corners[b][i]));
            }
        }
        for (i, xyz) in tet_xyz.iter().enumerate() {
            model.set_node((101 + i) as u32, *xyz).unwrap();
        }
        model.set_element(200, ElementKind::Tet10, (101..=110).collect()).unwrap();

        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("all_elems", Set { elements: vec![100, 200], ..Set::default() }).unwrap();
        model.add_set("fix", Set { nodes: vec![1, 4, 5, 8, 101, 103, 104], ..Set::default() }).unwrap();
        model.add_set("hex_f4", Set { faces: vec![fieldcraft_model::ElementFace { element: 100, face: 4 }], ..Set::default() }).unwrap();
        model.add_set("hex_f5", Set { faces: vec![fieldcraft_model::ElementFace { element: 100, face: 5 }], ..Set::default() }).unwrap();
        for f in 1..=4 {
            model
                .add_set(&format!("tet_f{f}"), Set { faces: vec![fieldcraft_model::ElementFace { element: 200, face: f }], ..Set::default() })
                .unwrap();
        }
        model
            .add_section(
                "solid",
                Section { set: "all_elems".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();

        model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("fix".into()) }).unwrap();
        model.add_load("ph4", Load::Pressure { set: "hex_f4".into(), magnitude: 1.0e5 }).unwrap();
        model.add_load("ph5", Load::Pressure { set: "hex_f5".into(), magnitude: 2.0e5 }).unwrap();
        for f in 1..=4 {
            model.add_load(&format!("pt{f}"), Load::Pressure { set: format!("tet_f{f}"), magnitude: f64::from(f) * 1.0e4 }).unwrap();
        }
        let mut step = Step::new("Load", Procedure::Static {});
        step.boundary_conditions.insert("clamp".into());
        step.loads.insert("ph4".into());
        step.loads.insert("ph5".into());
        for f in 1..=4 {
            step.loads.insert(format!("pt{f}"));
        }
        model.add_step(step, None).unwrap();

        let written = write_deck(&model, Some("Load"), None, Preset::MKgS).unwrap();
        assert_eq!(written.label_map.warp_elem_to_fc, vec![100, 200]);
        let logical = written.text.replace(" ,\n  ", " ");
        assert!(logical.contains("1 type q3disop linear material Steel order 2x2x2 bbar center_output short"), "{logical}");
        assert!(logical.contains("2 type tet10 linear material Steel order 4pt center_output short"), "{logical}");
        assert!(logical.contains("1 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20"), "{logical}");
        assert!(logical.contains("2 21 22 23 24 25 26 27 28 29 30"), "{logical}");
        // Hex20 faces 4 <-> 5 swap; Tet10 faces 1..=4 remain 1..=4.
        assert!(logical.contains("1 face 5 pressure 1.00000000e5"), "{logical}");
        assert!(logical.contains("1 face 4 pressure 2.00000000e5"), "{logical}");
        for f in 1..=4 {
            assert!(logical.contains(&format!("2 face {f} pressure")), "missing Tet10 face {f} in {logical}");
        }
        for line in written.text.lines() {
            if !line.starts_with("! ") && line != "!" {
                assert!(line.len() <= 72, "line exceeded 72 chars ({}): {line}", line.len());
            }
        }
    }

    #[test]
    fn write_deck_matches_validate_resolved_and_preserves_model_resolve_error() {
        // 1. Model::resolve() error still returns Warp3dError::Model.
        let mut unresolved = unit_hex8_model();
        unresolved.remove_section("solid").unwrap();
        assert!(matches!(write_deck(&unresolved, None, None, Preset::MKgS), Err(Warp3dError::Model(_))));

        // 2. Hard rejection parity (`Wedge6`) with validate_resolved.
        let mut wedge_model = unit_hex8_model();
        wedge_model.set_element(2, ElementKind::Wedge6, vec![1, 2, 3, 5, 6, 7]).unwrap();
        wedge_model.replace_set("all", Set { elements: vec![1, 2], ..Set::default() }).unwrap();
        let resolved = wedge_model.resolve().unwrap();
        let report = crate::check::validate_resolved(&wedge_model, &resolved, None, None, None, crate::check::CheckScope::Export);
        assert!(!report.ready);
        assert!(matches!(write_deck(&wedge_model, None, None, Preset::MKgS), Err(Warp3dError::Unsupported(_))));

        // 3. Warning parity (incompatible modes + rotational DOF + plasticity in Static step).
        let mut warn_model = unit_hex8_model();
        warn_model
            .replace_material(
                "Steel",
                Material {
                    elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }),
                    density: Some(7850.0),
                    plastic: Some(Plastic::Bilinear { yield_stress: 2.5e8, hardening_modulus: 0.0, hardening: Hardening::Isotropic }),
                    ..Default::default()
                },
            )
            .unwrap();
        warn_model
            .replace_section(
                "solid",
                Section {
                    set: "all".into(),
                    material: "Steel".into(),
                    kind: SectionKind::Solid {},
                    formulation: fieldcraft_model::Formulation { incompatible_modes: true, ..Default::default() },
                },
            )
            .unwrap();
        warn_model
            .add_boundary_condition(
                "rot",
                BoundaryCondition::Displacement { target: Target::Set("tip".into()), values: [None, None, None, Some(0.01), None, None] },
            )
            .unwrap();
        let mut st = warn_model.step("Pull").unwrap().clone();
        st.boundary_conditions.insert("rot".into());
        warn_model.replace_step(st).unwrap();

        let si_res = warn_model.resolve().unwrap();
        let report_w = crate::check::validate_resolved(&warn_model, &si_res, None, None, None, crate::check::CheckScope::Export);
        let written = write_deck(&warn_model, None, None, Preset::MKgS).unwrap();
        assert_eq!(written.warnings.len(), report_w.warnings);
        assert!(written.warnings.iter().any(|w| w.contains("B-bar")));
        assert!(written.warnings.iter().any(|w| w.contains("rotations")));
        assert!(written.warnings.iter().any(|w| w.contains("plasticity")));
    }
}

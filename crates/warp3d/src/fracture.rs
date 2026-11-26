//! 3D fracture mechanics: WARP3D deck line wrapping, topological crack-front chaining,
//! symmetry/normal resolution, and domain / interaction-integral command generation.

use std::collections::{BTreeMap, BTreeSet};

use fieldcraft_model::{BoundaryCondition, ElementKind, Model, Resolved, Step, Target};

use crate::{CrackFrontSpec, CrackFunctionType, LabelMap, Result, Warp3dError};

// WARP3D's fixed-format Fortran input reader truncates/rejects records past column ~80.
const MAX_WARP3D_LINE_LEN: usize = 72;

/// Ordered crack front extracted from hex-element edge connectivity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OrderedFront {
    /// Ordered FieldCraft corner node labels along the crack front.
    pub corners: Vec<u32>,
    /// Ordered FieldCraft midside node labels for `Hex20` segments (empty for `Hex8`).
    /// For an open front of `M` edges (`corners.len() == M + 1`), `midsides.len() == M`.
    /// For a closed loop of `M` edges (`corners.len() == M`), `midsides.len() == M`.
    pub midsides: Vec<u32>,
    /// Whether the crack front forms a closed loop.
    pub is_closed: bool,
    /// Whether the crack-front elements are quadratic (`Hex20`) vs linear (`Hex8`).
    pub is_quadratic: bool,
}

/// One WARP3D `domain` block derived from an [`OrderedFront`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DomainStation {
    pub domain_id: String,
    /// FieldCraft corner node label(s) represented by this domain.
    pub front_nodes: Vec<u32>,
    /// Sequential WARP3D `front nodes` list in FieldCraft node labels.
    pub station_fc_nodes: Vec<u32>,
    /// WARP3D domain q-function type (`'a'`, `'b'`, `'c'`, or `'d'`).
    pub function_type: char,
}

fn is_comment_line(line: &str) -> bool {
    let trimmed = line.trim_start();
    trimmed == "c" || trimmed == "C" || trimmed.starts_with("c ") || trimmed.starts_with("C ") || trimmed.starts_with('!') || trimmed.starts_with('*')
}

fn split_tokens_preserving_quotes(line: &str) -> Vec<&str> {
    let mut tokens = Vec::new();
    let bytes = line.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        while i < bytes.len() && bytes.get(i).is_some_and(|b| b.is_ascii_whitespace()) {
            i += 1;
        }
        if i >= bytes.len() {
            break;
        }
        let start = i;
        let mut in_single_quote = false;
        while i < bytes.len() {
            let b = bytes.get(i).copied().unwrap_or(b' ');
            if b == b'\'' {
                in_single_quote = !in_single_quote;
                i += 1;
            } else if b.is_ascii_whitespace() && !in_single_quote {
                break;
            } else {
                i += 1;
            }
        }
        if let Some(tok) = line.get(start..i) {
            tokens.push(tok);
        }
    }
    tokens
}

/// Append `line` to `out`, wrapping non-comment lines with trailing ` ,` continuations so no
/// non-comment output line exceeds 72 characters (when tokens themselves fit).
pub fn push_wrapped(out: &mut String, line: &str) {
    if is_comment_line(line) || line.len() <= MAX_WARP3D_LINE_LEN {
        out.push_str(line);
        out.push('\n');
        return;
    }

    let leading_spaces = line.len().saturating_sub(line.trim_start().len());
    let indent: String = line.chars().take(leading_spaces).collect();
    let tokens = split_tokens_preserving_quotes(line);
    if tokens.is_empty() {
        out.push('\n');
        return;
    }

    let mut current = String::with_capacity(MAX_WARP3D_LINE_LEN);
    current.push_str(&indent);

    for (idx, tok) in tokens.iter().enumerate() {
        let is_first = current.len() == indent.len();
        let is_last = idx + 1 == tokens.len();
        // Reserve 2 chars for ` ,` whenever more tokens follow.
        let max_len = if is_last { MAX_WARP3D_LINE_LEN } else { MAX_WARP3D_LINE_LEN - 2 };
        let sep_len = usize::from(!is_first);
        if !is_first && current.len() + sep_len + tok.len() > max_len {
            out.push_str(&current);
            out.push_str(" ,\n");
            current.clear();
            current.push_str("  ");
            current.push_str(tok);
        } else {
            if !is_first {
                current.push(' ');
            }
            current.push_str(tok);
        }
    }

    if !current.is_empty() {
        out.push_str(&current);
        out.push('\n');
    }
}

/// Order the nodes of `front_set` along shared `Hex8` or `Hex20` element edges into a continuous
/// open chain or closed loop. Refuses tetrahedral (`Tet4`/`Tet10`) meshes and branched or
/// disconnected crack fronts.
pub fn order_crack_front(model: &Model, set_name: &str, front_set: &[u32]) -> Result<OrderedFront> {
    if model.elements().values().any(|e| matches!(e.kind, ElementKind::Tet4 | ElementKind::Tet10)) {
        return Err(Warp3dError::Unsupported(
            "WARP3D does not support J-integral or interaction-integral domains on tetrahedral (tet4/tet10) meshes; mesh with hex8 or hex20".into(),
        ));
    }

    let in_front: BTreeSet<u32> = front_set.iter().copied().collect();
    if in_front.len() < 2 {
        return Err(Warp3dError::Unsupported(format!("crack front set `{set_name}` must contain at least 2 nodes along hex edges")));
    }

    let mut has_hex8 = false;
    let mut has_hex20 = false;
    // Key: (min(corner_a, corner_b), max(corner_a, corner_b)), value: optional Hex20 midside node.
    let mut segments: BTreeMap<(u32, u32), Option<u32>> = BTreeMap::new();

    for (elem_id, elem) in model.elements() {
        let touches_front = elem.nodes.iter().any(|n| in_front.contains(n));
        if !touches_front {
            continue;
        }
        match elem.kind {
            ElementKind::Hex8 => has_hex8 = true,
            ElementKind::Hex20 => has_hex20 = true,
            other => {
                return Err(Warp3dError::Unsupported(format!(
                    "crack front element {elem_id} has unsupported kind `{}` (only Hex8 and Hex20 are supported for WARP3D domain integrals)",
                    other.name()
                )));
            }
        }

        for edge in elem.kind.edges() {
            let Some(&n0) = edge.first().and_then(|&idx| elem.nodes.get(idx)) else {
                continue;
            };
            let Some(&n1) = edge.get(1).and_then(|&idx| elem.nodes.get(idx)) else {
                continue;
            };
            if n0 == n1 || !in_front.contains(&n0) || !in_front.contains(&n1) {
                continue;
            }
            let mid = edge.get(2).and_then(|&idx| elem.nodes.get(idx)).copied();
            if elem.kind == ElementKind::Hex20 {
                // Only accept the segment when its midside node (if present in the mesh) belongs
                // to the front set, or allow corner-only sets if all midsides are omitted consistently.
                if let Some(m) = mid {
                    let key = (n0.min(n1), n0.max(n1));
                    if in_front.contains(&m) {
                        segments.insert(key, Some(m));
                    } else {
                        segments.entry(key).or_insert(Some(m));
                    }
                }
            } else {
                segments.entry((n0.min(n1), n0.max(n1))).or_insert(None);
            }
        }
    }

    if has_hex8 && has_hex20 {
        return Err(Warp3dError::Unsupported(format!("crack front set `{set_name}` mixes Hex8 and Hex20 elements along the front")));
    }
    if segments.is_empty() {
        return Err(Warp3dError::Unsupported(format!("crack front set `{set_name}` does not follow connected Hex8/Hex20 element edges")));
    }

    let mut adj: BTreeMap<u32, Vec<(u32, Option<u32>)>> = BTreeMap::new();
    for (&(a, b), &mid) in &segments {
        adj.entry(a).or_default().push((b, mid));
        adj.entry(b).or_default().push((a, mid));
    }
    for neighbors in adj.values_mut() {
        neighbors.sort_by_key(|&(n, _)| n);
    }

    let mut endpoints = Vec::new();
    for (&node, neighbors) in &adj {
        if neighbors.len() > 2 {
            return Err(Warp3dError::Unsupported(format!("crack front set `{set_name}` is branched at node {node}")));
        }
        if neighbors.len() == 1 {
            endpoints.push(node);
        }
    }

    let is_closed = if endpoints.is_empty() {
        true
    } else if endpoints.len() == 2 {
        false
    } else {
        return Err(Warp3dError::Unsupported(format!("crack front set `{set_name}` is disconnected or has multiple open segments")));
    };

    let start = if is_closed { adj.keys().next().copied() } else { endpoints.iter().min().copied() }
        .ok_or_else(|| Warp3dError::Unsupported(format!("crack front set `{set_name}` is empty")))?;

    let mut corners = Vec::with_capacity(adj.len());
    let mut midsides = Vec::new();
    let mut visited_edges: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut visited_nodes: BTreeSet<u32> = BTreeSet::new();

    let mut curr = start;
    loop {
        corners.push(curr);
        visited_nodes.insert(curr);
        let next_step = adj.get(&curr).and_then(|neighbors| {
            neighbors.iter().copied().find(|&(nbr, _)| {
                let key = (curr.min(nbr), curr.max(nbr));
                !visited_edges.contains(&key)
            })
        });
        let Some((next, mid)) = next_step else {
            break;
        };
        let edge_key = (curr.min(next), curr.max(next));
        visited_edges.insert(edge_key);
        if let Some(m) = mid {
            midsides.push(m);
            visited_nodes.insert(m);
        }
        if is_closed && next == start {
            break;
        }
        curr = next;
    }

    // Verify all edges were traversed and every node specified in `in_front` belongs to the
    // single connected crack-front curve.
    if visited_edges.len() != segments.len() || !in_front.iter().all(|n| visited_nodes.contains(n)) {
        return Err(Warp3dError::Unsupported(format!("crack front set `{set_name}` is disconnected or contains isolated nodes")));
    }

    Ok(OrderedFront { corners, midsides, is_closed, is_quadratic: has_hex20 })
}

/// Resolve the unit crack-plane normal vector (`x2`) and half-symmetry flag (`symmetric`).
pub fn resolve_crack_normal_and_symmetry(
    model: &Model,
    resolved: &Resolved,
    step: &Step,
    spec: &CrackFrontSpec,
    front_nodes: &[u32],
) -> Result<([f64; 3], bool)> {
    if let Some(raw) = spec.normal {
        if !raw.iter().all(|c| c.is_finite()) {
            return Err(Warp3dError::Unsupported("crack plane `normal` must contain finite numbers".into()));
        }
        let scale = raw.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
        if !scale.is_finite() || scale <= 1.0e-15 {
            return Err(Warp3dError::Unsupported("crack plane `normal` must be non-zero".into()));
        }
        let scaled = raw.map(|c| c / scale);
        let norm = (scaled[0] * scaled[0] + scaled[1] * scaled[1] + scaled[2] * scaled[2]).sqrt();
        if !norm.is_finite() || norm <= 1.0e-15 {
            return Err(Warp3dError::Unsupported("crack plane `normal` must be non-zero".into()));
        }
        let unit = scaled.map(|c| c / norm);
        return Ok((unit, spec.symmetric.unwrap_or(false)));
    }

    for bc_name in &step.boundary_conditions {
        let Some(BoundaryCondition::Symmetry { target: Target::Set(bc_set), plane }) = model.boundary_condition(bc_name) else {
            continue;
        };
        let Some(rset) = resolved.sets.get(bc_set) else {
            continue;
        };
        let set_nodes: BTreeSet<u32> = rset.nodes.iter().copied().collect();
        if front_nodes.iter().all(|n| set_nodes.contains(n))
            && let Some(unit) = plane.unit_normal()
        {
            return Ok((unit, spec.symmetric.unwrap_or(true)));
        }
    }

    Err(Warp3dError::Unsupported("specify crack plane `normal` [nx, ny, nz] or apply a symmetry boundary condition on the crack plane".into()))
}

/// Build the per-station WARP3D domain definitions from an [`OrderedFront`].
pub fn build_domain_stations(ordered: &OrderedFront, func_type: CrackFunctionType) -> Vec<DomainStation> {
    let corners = &ordered.corners;
    let mids = &ordered.midsides;
    let m = if ordered.is_closed { corners.len() } else { corners.len().saturating_sub(1) };

    match func_type {
        CrackFunctionType::FrontAverage => {
            let mut station_fc_nodes = Vec::new();
            for (i, &c) in corners.iter().enumerate() {
                station_fc_nodes.push(c);
                if ordered.is_quadratic
                    && let Some(&mid) = mids.get(i)
                    && (ordered.is_closed || i < m)
                {
                    station_fc_nodes.push(mid);
                }
            }
            if ordered.is_closed
                && let Some(&c0) = corners.first()
            {
                station_fc_nodes.push(c0);
            }
            vec![DomainStation { domain_id: "cf_0001".into(), front_nodes: corners.clone(), station_fc_nodes, function_type: 'd' }]
        }
        CrackFunctionType::Pointwise => {
            let mut stations = Vec::new();
            if ordered.is_closed {
                for i in 0..m {
                    let prev = (i + m - 1) % m;
                    let next = (i + 1) % m;
                    let mut nodes = Vec::new();
                    if let Some(&cp) = corners.get(prev) {
                        nodes.push(cp);
                    }
                    if ordered.is_quadratic
                        && let Some(&mp) = mids.get(prev)
                    {
                        nodes.push(mp);
                    }
                    if let Some(&ci) = corners.get(i) {
                        nodes.push(ci);
                    }
                    if ordered.is_quadratic
                        && let Some(&mi) = mids.get(i)
                    {
                        nodes.push(mi);
                    }
                    if let Some(&cn) = corners.get(next) {
                        nodes.push(cn);
                    }
                    let front_nodes = corners.get(i).copied().into_iter().collect();
                    stations.push(DomainStation { domain_id: format!("cf_{:04}", i + 1), front_nodes, station_fc_nodes: nodes, function_type: 'b' });
                }
            } else {
                for i in 0..corners.len() {
                    let mut nodes = Vec::new();
                    let func = if i == 0 {
                        if let Some(&c0) = corners.first() {
                            nodes.push(c0);
                        }
                        if ordered.is_quadratic
                            && let Some(&m0) = mids.first()
                        {
                            nodes.push(m0);
                        }
                        if let Some(&c1) = corners.get(1) {
                            nodes.push(c1);
                        }
                        'a'
                    } else if i == m {
                        if let Some(&cp) = corners.get(m - 1) {
                            nodes.push(cp);
                        }
                        if ordered.is_quadratic
                            && let Some(&mp) = mids.get(m - 1)
                        {
                            nodes.push(mp);
                        }
                        if let Some(&cm) = corners.get(m) {
                            nodes.push(cm);
                        }
                        'c'
                    } else {
                        if let Some(&cp) = corners.get(i - 1) {
                            nodes.push(cp);
                        }
                        if ordered.is_quadratic
                            && let Some(&mp) = mids.get(i - 1)
                        {
                            nodes.push(mp);
                        }
                        if let Some(&ci) = corners.get(i) {
                            nodes.push(ci);
                        }
                        if ordered.is_quadratic
                            && let Some(&mi) = mids.get(i)
                        {
                            nodes.push(mi);
                        }
                        if let Some(&cn) = corners.get(i + 1) {
                            nodes.push(cn);
                        }
                        'b'
                    };
                    let front_nodes = corners.get(i).copied().into_iter().collect();
                    stations.push(DomainStation { domain_id: format!("cf_{:04}", i + 1), front_nodes, station_fc_nodes: nodes, function_type: func });
                }
            }
            stations
        }
    }
}

/// Emit WARP3D `domain`, `compute domain integral`, and optional `compute domain interaction integrals`
/// commands for `spec` onto `out`.
pub fn emit_domain_commands(
    out: &mut String,
    model: &Model,
    resolved: &Resolved,
    step: &Step,
    label_map: &LabelMap,
    spec: &CrackFrontSpec,
) -> Result<()> {
    let rings = spec.rings.unwrap_or(4);
    if !(1..=20).contains(&rings) {
        return Err(Warp3dError::Unsupported(format!("crack front domain `rings` must be in 1..=20, got {rings}")));
    }

    let rset = resolved.sets.get(&spec.set).ok_or_else(|| Warp3dError::Unsupported(format!("crack front node set `{}` not found", spec.set)))?;
    let ordered = order_crack_front(model, &spec.set, &rset.nodes)?;
    let (normal, symmetric) = resolve_crack_normal_and_symmetry(model, resolved, step, spec, &ordered.corners)?;

    let interaction = spec.interaction_integral.unwrap_or(false);
    let order_kw = if ordered.is_quadratic { "quadratic" } else { "linear" };
    let stations = build_domain_stations(&ordered, spec.function_type);

    for station in &stations {
        let mut warp_ids = Vec::with_capacity(station.station_fc_nodes.len());
        for fc_id in &station.station_fc_nodes {
            let wid = label_map
                .fc_node_to_warp
                .get(fc_id)
                .copied()
                .ok_or_else(|| Warp3dError::Unsupported(format!("crack front node {fc_id} missing from WARP3D label map")))?;
            warp_ids.push(wid.to_string());
        }

        push_wrapped(out, &format!("domain {}", station.domain_id));
        if symmetric {
            push_wrapped(out, "  symmetric");
        }
        push_wrapped(out, &format!("  normal plane nx {:.8e} ny {:.8e} nz {:.8e}", normal[0], normal[1], normal[2]));
        push_wrapped(out, &format!("  front nodes {} {} verify", warp_ids.join(" "), order_kw));
        push_wrapped(out, &format!("  q-values automatic rings 1-{rings}"));
        push_wrapped(out, &format!("  function type {}", station.function_type));
        if interaction {
            push_wrapped(out, "  plane strain output for interaction integral");
        }
        push_wrapped(out, "  J to K");
        push_wrapped(out, "  dump");
        if interaction {
            push_wrapped(out, "compute domain interaction integral");
        } else {
            push_wrapped(out, "compute domain integral");
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use fieldcraft_model::{BoundaryCondition, Elastic, ElementKind, Material, Model, Procedure, Section, SectionKind, Set, Step, Symmetry, Target};

    use super::*;

    fn hex8_chain_model() -> (Model, LabelMap, Resolved, Step) {
        let mut model = Model::default();
        // 4 Hex8 elements along Z (0..=4), with crack front nodes on (X=1, Y=0, Z=0..=4):
        // scrambled labels along front: 3 (z=0), 27 (z=1), 12 (z=2), 99 (z=3), 50 (z=4).
        let front_labels = [3u32, 27, 12, 99, 50];
        for (i, &fz) in front_labels.iter().enumerate() {
            let z = i as f64;
            let base = 200 + (i as u32) * 10;
            model.set_node(fz, [1.0, 0.0, z]).unwrap();
            model.set_node(base + 1, [2.0, 0.0, z]).unwrap();
            model.set_node(base + 2, [2.0, 1.0, z]).unwrap();
            model.set_node(base + 3, [1.0, 1.0, z]).unwrap();
        }
        for i in 0..4u32 {
            let b0 = 200 + i * 10;
            let b1 = 200 + (i + 1) * 10;
            let f0 = front_labels[i as usize];
            let f1 = front_labels[(i + 1) as usize];
            model.set_element(i + 1, ElementKind::Hex8, vec![f0, b0 + 1, b0 + 2, b0 + 3, f1, b1 + 1, b1 + 2, b1 + 3]).unwrap();
        }
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("all", Set { elements: vec![1, 2, 3, 4], ..Set::default() }).unwrap();
        model.add_set("crack_front", Set { nodes: vec![50, 12, 99, 3, 27], ..Set::default() }).unwrap();
        model.add_set("ligament", Set { nodes: vec![3, 27, 12, 99, 50, 201, 211, 221, 231, 241], ..Set::default() }).unwrap();
        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_boundary_condition("sym_y", BoundaryCondition::Symmetry { target: Target::Set("ligament".into()), plane: Symmetry::Y }).unwrap();
        let mut step = Step::new("Load", Procedure::Static {});
        step.boundary_conditions.insert("sym_y".into());
        model.add_step(step.clone(), None).unwrap();

        let resolved = model.resolve().unwrap();
        let label_map = LabelMap::from_model(&model);
        (model, label_map, resolved, step)
    }

    #[test]
    fn chains_scrambled_linear_hex8_crack_front_nodes() {
        let (model, label_map, resolved, step) = hex8_chain_model();
        let spec = CrackFrontSpec {
            set: "crack_front".into(),
            normal: None,
            symmetric: None,
            rings: Some(4),
            interaction_integral: Some(false),
            function_type: CrackFunctionType::Pointwise,
            every_step: None,
        };
        let ordered = order_crack_front(&model, "crack_front", &[50, 12, 99, 3, 27]).unwrap();
        assert_eq!(ordered.corners, vec![3, 27, 12, 99, 50]);
        assert!(!ordered.is_quadratic);
        assert!(!ordered.is_closed);

        let mut out = String::new();
        emit_domain_commands(&mut out, &model, &resolved, &step, &label_map, &spec).unwrap();
        assert!(out.contains("domain cf_0001"));
        assert!(out.contains("domain cf_0005"));
        assert!(out.contains("  symmetric\n"));
        assert!(out.contains("function type a\n"));
        assert!(out.contains("function type b\n"));
        assert!(out.contains("function type c\n"));
        assert!(out.contains("linear verify\n"));
        assert!(out.contains("  dump\n"));
        assert_eq!(out.matches("compute domain integral\n").count(), 5);
        assert!(!out.contains("compute domain interaction integral"));
    }

    #[test]
    fn chains_scrambled_quadratic_hex20_crack_front_nodes_with_midsides() {
        let mut model = Model::default();
        // Two Hex20 elements along Z (0..2), front at (X=1, Y=0, Z):
        // corners: 3 (z=0), 27 (z=1), 50 (z=2); midsides along front: 77 (z=0.5), 88 (z=1.5).
        // For each slice of corners at z=0,1,2 and midsides on transverse edges at z=0.5,1.5:
        let corner_z = [(3u32, 200u32, 0.0), (27u32, 300u32, 1.0), (50u32, 400u32, 2.0)];
        for &(f, b, z) in &corner_z {
            model.set_node(f, [1.0, 0.0, z]).unwrap();
            model.set_node(b + 1, [2.0, 0.0, z]).unwrap();
            model.set_node(b + 2, [2.0, 1.0, z]).unwrap();
            model.set_node(b + 3, [1.0, 1.0, z]).unwrap();
            // in-plane midsides: (12, 23, 34, 41)
            model.set_node(b + 4, [1.5, 0.0, z]).unwrap();
            model.set_node(b + 5, [2.0, 0.5, z]).unwrap();
            model.set_node(b + 6, [1.5, 1.0, z]).unwrap();
            model.set_node(b + 7, [1.0, 0.5, z]).unwrap();
        }
        // vertical midsides at z = 0.5 and 1.5
        let vert_z = [(77u32, 500u32, 0.5), (88u32, 600u32, 1.5)];
        for &(fm, bm, z) in &vert_z {
            model.set_node(fm, [1.0, 0.0, z]).unwrap();
            model.set_node(bm + 1, [2.0, 0.0, z]).unwrap();
            model.set_node(bm + 2, [2.0, 1.0, z]).unwrap();
            model.set_node(bm + 3, [1.0, 1.0, z]).unwrap();
        }
        for i in 0..2usize {
            let (f0, b0, _) = corner_z[i];
            let (f1, b1, _) = corner_z[i + 1];
            let (fm, bm, _) = vert_z[i];
            let nodes = vec![
                f0,
                b0 + 1,
                b0 + 2,
                b0 + 3,
                f1,
                b1 + 1,
                b1 + 2,
                b1 + 3,
                b0 + 4,
                b0 + 5,
                b0 + 6,
                b0 + 7,
                b1 + 4,
                b1 + 5,
                b1 + 6,
                b1 + 7,
                fm,
                bm + 1,
                bm + 2,
                bm + 3,
            ];
            model.set_element((i + 1) as u32, ElementKind::Hex20, nodes).unwrap();
        }
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("all", Set { elements: vec![1, 2], ..Set::default() }).unwrap();
        model.add_set("crack_front", Set { nodes: vec![88, 50, 3, 77, 27], ..Set::default() }).unwrap();
        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        let step = Step::new("Load", Procedure::Static {});
        model.add_step(step.clone(), None).unwrap();

        let resolved = model.resolve().unwrap();
        let label_map = LabelMap::from_model(&model);
        let spec = CrackFrontSpec {
            set: "crack_front".into(),
            normal: Some([0.0, 2.0, 0.0]),
            symmetric: Some(false),
            rings: Some(5),
            interaction_integral: Some(true),
            function_type: CrackFunctionType::Pointwise,
            every_step: None,
        };
        let ordered = order_crack_front(&model, "crack_front", &[88, 50, 3, 77, 27]).unwrap();
        assert_eq!(ordered.corners, vec![3, 27, 50]);
        assert_eq!(ordered.midsides, vec![77, 88]);
        assert!(ordered.is_quadratic);

        let mut out = String::new();
        emit_domain_commands(&mut out, &model, &resolved, &step, &label_map, &spec).unwrap();
        assert!(out.contains("quadratic verify\n"));
        assert!(out.contains("q-values automatic rings 1-5\n"));
        assert!(out.contains("plane strain output for interaction integral\n"));
        assert!(out.contains("  dump\n"));
        assert_eq!(out.matches("compute domain interaction integral\n").count(), 3);
        assert!(!out.contains("compute domain integral\n"));
        assert!(!out.contains("  symmetric\n"));
    }

    #[test]
    fn refuses_tet_and_branched_crack_fronts() {
        let mut tet_model = Model::default();
        for (id, xyz) in [(1, [0.0, 0.0, 0.0]), (2, [1.0, 0.0, 0.0]), (3, [0.0, 1.0, 0.0]), (4, [0.0, 0.0, 1.0])] {
            tet_model.set_node(id, xyz).unwrap();
        }
        tet_model.set_element(1, ElementKind::Tet4, vec![1, 2, 3, 4]).unwrap();
        let err = order_crack_front(&tet_model, "cf", &[1, 2]).unwrap_err();
        assert!(err.to_string().contains("tet4/tet10"));

        // Branched front: T-junction in Hex8 edges (degree 3 at node 12).
        let (m, _, _, _) = hex8_chain_model();
        let err = order_crack_front(&m, "crack_front", &[3, 27, 12, 99, 50, 221]).unwrap_err();
        assert!(err.to_string().contains("branched"));
    }

    #[test]
    fn wraps_long_lines_under_72_chars() {
        let mut out = String::new();
        let long_line = "  front nodes 101 102 103 104 105 106 107 108 109 110 111 112 113 114 115 116 117 118 linear verify";
        push_wrapped(&mut out, long_line);
        for line in out.lines() {
            assert!(line.len() <= MAX_WARP3D_LINE_LEN, "line too long ({}): {line}", line.len());
        }
        assert!(out.contains(" ,\n"));
        // Comments stay untouched even if longer than 72 chars.
        let mut c_out = String::new();
        let long_comment = "c This comment exceeds seventy-two characters intentionally so we verify it is not wrapped";
        push_wrapped(&mut c_out, long_comment);
        assert_eq!(c_out.lines().count(), 1);
    }

    #[test]
    fn chains_closed_crack_loop_and_emits_type_b_stations() {
        let mut model = Model::default();
        let inner_z0 = [(10u32, [1.0, 1.0, 0.0]), (20, [2.0, 1.0, 0.0]), (30, [2.0, 2.0, 0.0]), (40, [1.0, 2.0, 0.0])];
        let outer_z0 = [(11u32, [0.0, 0.0, 0.0]), (21, [3.0, 0.0, 0.0]), (31, [3.0, 3.0, 0.0]), (41, [0.0, 3.0, 0.0])];
        let inner_z1 = [(12u32, [1.0, 1.0, 1.0]), (22, [2.0, 1.0, 1.0]), (32, [2.0, 2.0, 1.0]), (42, [1.0, 2.0, 1.0])];
        let outer_z1 = [(13u32, [0.0, 0.0, 1.0]), (23, [3.0, 0.0, 1.0]), (33, [3.0, 3.0, 1.0]), (43, [0.0, 3.0, 1.0])];
        for (id, xyz) in inner_z0.into_iter().chain(outer_z0).chain(inner_z1).chain(outer_z1) {
            model.set_node(id, xyz).unwrap();
        }
        let ring_elems = [
            (1u32, vec![11, 21, 20, 10, 13, 23, 22, 12]),
            (2, vec![21, 31, 30, 20, 23, 33, 32, 22]),
            (3, vec![31, 41, 40, 30, 33, 43, 42, 32]),
            (4, vec![41, 11, 10, 40, 43, 13, 12, 42]),
        ];
        for (eid, nodes) in ring_elems {
            model.set_element(eid, ElementKind::Hex8, nodes).unwrap();
        }
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("all", Set { elements: vec![1, 2, 3, 4], ..Set::default() }).unwrap();
        model.add_set("loop_front", Set { nodes: vec![30, 10, 40, 20], ..Set::default() }).unwrap();
        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        let step = Step::new("Load", Procedure::Static {});
        model.add_step(step.clone(), None).unwrap();

        let resolved = model.resolve().unwrap();
        let label_map = LabelMap::from_model(&model);

        let ordered = order_crack_front(&model, "loop_front", &[30, 10, 40, 20]).unwrap();
        assert!(ordered.is_closed);
        assert!(!ordered.is_quadratic);
        assert_eq!(ordered.corners, vec![10, 20, 30, 40]);

        let pointwise = build_domain_stations(&ordered, CrackFunctionType::Pointwise);
        assert_eq!(pointwise.len(), 4);
        assert!(pointwise.iter().all(|s| s.function_type == 'b'));
        assert_eq!(pointwise[0].front_nodes, vec![10]);
        assert_eq!(pointwise[0].station_fc_nodes, vec![40, 10, 20]);
        assert_eq!(pointwise[3].front_nodes, vec![40]);
        assert_eq!(pointwise[3].station_fc_nodes, vec![30, 40, 10]);

        let avg = build_domain_stations(&ordered, CrackFunctionType::FrontAverage);
        assert_eq!(avg.len(), 1);
        assert_eq!(avg[0].function_type, 'd');
        assert_eq!(avg[0].front_nodes, vec![10, 20, 30, 40]);
        assert_eq!(avg[0].station_fc_nodes, vec![10, 20, 30, 40, 10]);
        assert_eq!(avg[0].station_fc_nodes.first(), avg[0].station_fc_nodes.last());

        let spec = CrackFrontSpec {
            set: "loop_front".into(),
            normal: Some([0.0, 0.0, 1.0]),
            symmetric: Some(false),
            rings: Some(3),
            interaction_integral: Some(false),
            function_type: CrackFunctionType::Pointwise,
            every_step: None,
        };
        let mut out = String::new();
        emit_domain_commands(&mut out, &model, &resolved, &step, &label_map, &spec).unwrap();
        assert_eq!(out.matches("function type b\n").count(), 4);
        assert!(!out.contains("function type a\n"));
        assert!(!out.contains("function type c\n"));
        assert_eq!(out.matches("compute domain integral\n").count(), 4);
        for line in out.lines() {
            assert!(line.len() <= MAX_WARP3D_LINE_LEN, "line too long ({}): {line}", line.len());
        }
    }
}

//! SI unit conversion, material/power-law conversion, section assembly, list resolution,
//! constraints, and static load pattern lowering.

use std::collections::{BTreeMap, BTreeSet};

use fieldcraft_model::materials::MAX_TABLE_POINTS;
use fieldcraft_model::units::{DisplaySystem, Preset, Quantity};
use fieldcraft_model::{
    BoundaryCondition, Distribution, Elastic, ElementFace, ElementKind, Formulation, Hardening, HardeningPoint, Integration, Load, MAX_DEFINITIONS,
    MAX_NAME_LEN, Model, Plastic, Section, SectionKind, Set, Symmetry, Target, check_name, progress, suggest_name,
};

use super::parse::{ElementSpec, IdSelector, ParsedDeck, PlaneAction, RawCurve, RawList, RawMaterial, RawMaterialKind};
use super::{ImportedDeck, ReadError, ReadLimits, UnitsSource};

const ELASTIC_TOL: f64 = 1e-12; // unit-factors: dimensionless tolerance for n_power == 1, beta == 1, zero-span planes, and MPC sign
const YIELD_SNAP_TOL: f64 = 1e-6; // unit-factors: dimensionless yield-onset snap tolerance on elastic strain
const MPC_ROUND_STEP: f64 = 1e-9; // unit-factors: dimensionless unit-normal rounding step for deterministic MPC grouping
const PLANE_BBOX_FRAC: f64 = 1e-5;
const POWER_LAW_DELTA1: f64 = 1e-4;
const DOF_CONFLICT_TOL: f64 = 1e-15;
const POWER_LAW_POINTS: usize = 64;

fn canon_f64(v: f64) -> f64 {
    if v == 0.0 { 0.0 } else { v }
}

fn canon_vec3_bits(v: [f64; 3]) -> [u64; 3] {
    [canon_f64(v[0]).to_bits(), canon_f64(v[1]).to_bits(), canon_f64(v[2]).to_bits()]
}

fn unique_name(raw: &str, fallback_prefix: &str, used: &mut BTreeSet<String>) -> Result<String, ReadError> {
    if used.len() >= MAX_DEFINITIONS {
        return Err(ReadError::Limit(format!("definition limit of {MAX_DEFINITIONS} reached")));
    }
    let base = if check_name("imported", raw).is_ok() {
        raw.to_string()
    } else {
        let candidate = suggest_name(raw).unwrap_or_else(|| fallback_prefix.to_string());
        if check_name("imported", &candidate).is_ok() { candidate } else { fallback_prefix.to_string() }
    };

    if used.insert(base.clone()) {
        return Ok(base);
    }
    let mut suffix = 2usize;
    loop {
        let tag = format!("-{suffix}");
        let stem_len = MAX_NAME_LEN.saturating_sub(tag.len());
        let stem = base.get(..base.len().min(stem_len)).unwrap_or(&base);
        let candidate = format!("{stem}{tag}");
        if used.insert(candidate.clone()) {
            return Ok(candidate);
        }
        suffix += 1;
    }
}

fn to_si(display: &DisplaySystem, q: Quantity, val: f64, file: &str, line: usize) -> Result<f64, ReadError> {
    display.to_si(q, val).map_err(|e| ReadError::Parse { file: file.to_string(), line, message: e.to_string() })
}

fn build_power_law_table(young_si: f64, sigma0_si: f64, n: f64, file: &str, line: usize) -> Result<Plastic, ReadError> {
    let eps0 = sigma0_si / young_si;
    if !(eps0 > 0.0 && eps0.is_finite()) {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "invalid yield-to-modulus ratio in `n_power` material".into() });
    }
    let inv_n = 1.0 / n;
    let eval = |u: f64| -> f64 { eps0 * (u.exp() - (u * inv_n).exp()) - 1.0 };

    let mut u_lo = 0.0_f64;
    let mut u_hi = (1.0 + 1.0 / eps0 + 2.0).ln();
    for _ in 0..64 {
        if eval(u_hi) >= 0.0 {
            break;
        }
        u_hi *= 2.0;
    }
    if eval(u_hi) < 0.0 || !u_hi.is_finite() {
        return Err(ReadError::Parse {
            file: file.to_string(),
            line,
            message: format!("failed to bracket power-law `n_power = {n}` hardening endpoint"),
        });
    }

    let mut u = 0.5 * (u_lo + u_hi);
    for _ in 0..100 {
        let f = eval(u);
        if f.abs() <= 1e-14 || (u_hi - u_lo) <= 1e-15 {
            break;
        }
        if f < 0.0 {
            u_lo = u;
        } else {
            u_hi = u;
        }
        let deriv = eps0 * (u.exp() - inv_n * (u * inv_n).exp());
        let next = if deriv > 0.0 && deriv.is_finite() { u - f / deriv } else { 0.5 * (u_lo + u_hi) };
        u = if next > u_lo && next < u_hi { next } else { 0.5 * (u_lo + u_hi) };
    }

    let r_max = u.exp();
    let delta_max = r_max - 1.0;
    if !(delta_max > 0.0 && delta_max.is_finite()) {
        return Err(ReadError::Parse { file: file.to_string(), line, message: format!("degenerate power-law `n_power = {n}` range") });
    }

    let delta_1 = POWER_LAW_DELTA1.min(0.01 * delta_max);
    let log_ratio = (delta_max / delta_1).ln();
    let mut points = Vec::with_capacity(POWER_LAW_POINTS);
    points.push(HardeningPoint { strain: 0.0, stress: sigma0_si });

    let sub_points = POWER_LAW_POINTS - 1;
    for k in 0..sub_points {
        let frac = k as f64 / (sub_points - 1) as f64;
        let r = if k + 1 == sub_points { r_max } else { 1.0 + delta_1 * (frac * log_ratio).exp() };
        let r_pow = r.powf(inv_n);
        let stress = sigma0_si * r_pow;
        let strain = eps0 * (r - r_pow);
        let prev_strain = points.last().map_or(0.0, |p| p.strain);
        if !(strain.is_finite() && stress.is_finite() && strain > prev_strain) {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("non-monotonic `n_power = {n}` discretization at point {}", k + 1),
            });
        }
        points.push(HardeningPoint { strain, stress });
    }
    Ok(Plastic::Table { points })
}

fn build_curve_table(curve: &RawCurve, young_si: f64, display: &DisplaySystem, file: &str, mat_line: usize) -> Result<Plastic, ReadError> {
    let mut iter = curve.points.iter().copied();
    let mut raw_pairs: Vec<[f64; 2]> = Vec::with_capacity(curve.points.len());
    if let Some(first) = iter.next()
        && !(first[0] == 0.0 && first[1] == 0.0)
    {
        raw_pairs.push(first);
    }
    raw_pairs.extend(iter);

    if raw_pairs.is_empty() {
        return Err(ReadError::Parse {
            file: file.to_string(),
            line: curve.line,
            message: format!("stress-strain curve {} has no points beyond (0, 0)", curve.id),
        });
    }

    let first_stress_raw = raw_pairs.first().map(|p| p[1]).unwrap_or(0.0);
    let sigma0_si = to_si(display, Quantity::Pressure, first_stress_raw, file, curve.line)?;
    if !(sigma0_si > 0.0) {
        return Err(ReadError::Parse {
            file: file.to_string(),
            line: curve.line,
            message: format!("stress-strain curve {} initial stress must be > 0", curve.id),
        });
    }
    let snap_tol = YIELD_SNAP_TOL * (sigma0_si / young_si);

    let mut converted: Vec<HardeningPoint> = Vec::with_capacity(raw_pairs.len());
    for [eps_total, stress_raw] in raw_pairs {
        let stress_si = to_si(display, Quantity::Pressure, stress_raw, file, curve.line)?;
        if !(stress_si > 0.0) {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line: curve.line,
                message: format!("stress-strain curve {} requires strictly positive stresses", curve.id),
            });
        }
        let eps_p = eps_total - stress_si / young_si;
        converted.push(HardeningPoint { strain: eps_p, stress: stress_si });
    }

    // Drop leading elastic segment points (eps_p <= snap_tol) while keeping the highest-stress
    // elastic point snapped to exact plastic strain 0.0.
    let mut elastic_end = 0usize;
    let mut highest_elastic_stress: Option<f64> = None;
    for pt in &converted {
        if pt.strain < -snap_tol {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line: mat_line,
                message: format!("stress-strain curve {} exceeds elastic slope E before yield", curve.id),
            });
        }
        if pt.strain <= snap_tol {
            highest_elastic_stress = Some(highest_elastic_stress.map_or(pt.stress, |prev| prev.max(pt.stress)));
            elastic_end += 1;
        } else {
            break;
        }
    }

    let mut points: Vec<HardeningPoint> = Vec::new();
    if let Some(y_stress) = highest_elastic_stress {
        points.push(HardeningPoint { strain: 0.0, stress: y_stress });
    } else if let Some(first) = converted.first() {
        return Err(ReadError::Parse {
            file: file.to_string(),
            line: curve.line,
            message: format!("stress-strain curve {} first point has positive plastic strain {} without yield onset at 0", curve.id, first.strain),
        });
    }

    for pt in converted.into_iter().skip(elastic_end) {
        if let Some(last) = points.last()
            && pt.strain <= last.strain
        {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line: curve.line,
                message: format!("stress-strain curve {} plastic strains must strictly increase", curve.id),
            });
        }
        points.push(pt);
    }

    if points.is_empty() || points.len() > MAX_TABLE_POINTS {
        return Err(ReadError::Limit(format!("stress-strain curve {} table has {} points (allowed 1..={MAX_TABLE_POINTS})", curve.id, points.len())));
    }
    Ok(Plastic::Table { points })
}

fn build_material(
    raw: &RawMaterial,
    curves: &BTreeMap<u32, RawCurve>,
    display: &DisplaySystem,
    file: &str,
    warnings: &mut Vec<String>,
) -> Result<fieldcraft_model::Material, ReadError> {
    let e_raw = raw.young.ok_or_else(|| ReadError::Parse {
        file: file.to_string(),
        line: raw.line,
        message: format!("material `{}` is missing Young's modulus `e`", raw.name),
    })?;
    let nu = raw.poisson.ok_or_else(|| ReadError::Parse {
        file: file.to_string(),
        line: raw.line,
        message: format!("material `{}` is missing Poisson's ratio `nu`", raw.name),
    })?;
    let young_si = to_si(display, Quantity::Pressure, e_raw, file, raw.line)?;
    let elastic = Elastic::Isotropic { young: young_si, poisson: nu };
    elastic.validate().map_err(|e| ReadError::Parse { file: file.to_string(), line: raw.line, message: format!("material `{}`: {e}", raw.name) })?;

    let density = match raw.rho {
        None | Some(0.0) => None,
        Some(r) if r > 0.0 => Some(to_si(display, Quantity::Density, r, file, raw.line)?),
        Some(r) => {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line: raw.line,
                message: format!("material `{}` density `rho` must be >= 0, got {r}", raw.name),
            });
        }
    };

    if let Some(a) = raw.alpha
        && a != 0.0
    {
        warnings.push(format!("material `{}` thermal expansion `alpha = {a}` was ignored", raw.name));
    }

    let beta = raw.beta.unwrap_or(1.0);
    if (beta - 1.0).abs() > ELASTIC_TOL {
        return Err(ReadError::Unsupported {
            file: file.to_string(),
            line: raw.line,
            feature: "kinematic/mixed hardening (beta != 1.0)".into(),
            message: format!("material `{}` sets `beta = {beta}`; only isotropic hardening (beta = 1.0) is supported", raw.name),
        });
    }

    let plastic = match raw.kind {
        RawMaterialKind::LinearElastic => None,
        RawMaterialKind::Bilinear => {
            let y_raw = raw.yld_pt.ok_or_else(|| ReadError::Parse {
                file: file.to_string(),
                line: raw.line,
                message: format!("bilinear material `{}` requires `yld_pt` or `sig_o`", raw.name),
            })?;
            let tan_raw = raw.tan_e.unwrap_or(0.0);
            if tan_raw < 0.0 || tan_raw >= e_raw {
                return Err(ReadError::Parse {
                    file: file.to_string(),
                    line: raw.line,
                    message: format!("bilinear material `{}` requires 0 <= tan_e < e", raw.name),
                });
            }
            let y_si = to_si(display, Quantity::Pressure, y_raw, file, raw.line)?;
            let et_si = to_si(display, Quantity::Pressure, tan_raw, file, raw.line)?;
            let hardening_modulus = if et_si == 0.0 { 0.0 } else { (young_si * et_si) / (young_si - et_si) };
            Some(Plastic::Bilinear { yield_stress: y_si, hardening_modulus, hardening: Hardening::Isotropic })
        }
        RawMaterialKind::Mises => {
            if let Some(cid) = raw.curve_id {
                let curve = curves.get(&cid).ok_or_else(|| ReadError::Parse {
                    file: file.to_string(),
                    line: raw.line,
                    message: format!("material `{}` references missing stress-strain curve {cid}", raw.name),
                })?;
                if curve.non_linear_interp {
                    warnings.push(format!("stress-strain curve {cid} (`pchip`/`smooth`) was imported as a piecewise linear hardening table"));
                }
                Some(build_curve_table(curve, young_si, display, file, raw.line)?)
            } else if let Some(n) = raw.n_power {
                if n < 1.0 {
                    return Err(ReadError::Parse {
                        file: file.to_string(),
                        line: raw.line,
                        message: format!("material `{}` `n_power` must be >= 1.0, got {n}", raw.name),
                    });
                }
                if (n - 1.0).abs() <= ELASTIC_TOL {
                    None
                } else {
                    let y_raw = raw.yld_pt.filter(|&y| y > 0.0).ok_or_else(|| ReadError::Parse {
                        file: file.to_string(),
                        line: raw.line,
                        message: format!("mises material `{}` with `n_power > 1.0` requires positive `yld_pt` or `sig_o`", raw.name),
                    })?;
                    let sigma0_si = to_si(display, Quantity::Pressure, y_raw, file, raw.line)?;
                    warnings.push(format!(
                        "material `{}` power-law hardening (`n_power = {n}`) was discretized as a {POWER_LAW_POINTS}-point piecewise linear table up to plastic strain 1.0 (clamped flat past 1.0)",
                        raw.name
                    ));
                    Some(build_power_law_table(young_si, sigma0_si, n, file, raw.line)?)
                }
            } else if let Some(y_raw) = raw.yld_pt {
                let y_si = to_si(display, Quantity::Pressure, y_raw, file, raw.line)?;
                Some(Plastic::Bilinear { yield_stress: y_si, hardening_modulus: 0.0, hardening: Hardening::Isotropic })
            } else {
                return Err(ReadError::Parse {
                    file: file.to_string(),
                    line: raw.line,
                    message: format!("mises material `{}` requires `curve`, `n_power`, or `yld_pt`/`sig_o`", raw.name),
                });
            }
        }
    };

    Ok(fieldcraft_model::Material { elastic: Some(elastic), density, plastic, ..Default::default() })
}

pub(crate) fn build_model(
    root_name: &str,
    mut parsed: ParsedDeck,
    units: Preset,
    units_source: UnitsSource,
    limits: ReadLimits,
) -> Result<ImportedDeck, ReadError> {
    let display = DisplaySystem::new(units);
    let mut model = Model::default();
    model.set_display_units(Some(display.clone()));

    let mut warnings = std::mem::take(&mut parsed.warnings);

    // 1. Insert nodes converted into SI.
    let nodes = std::mem::take(&mut parsed.nodes);
    for (idx, (id, (xyz, line))) in nodes.into_iter().enumerate() {
        if idx.is_multiple_of(256) {
            progress::check()?;
        }
        let x = to_si(&display, Quantity::Length, xyz[0], root_name, line)?;
        let y = to_si(&display, Quantity::Length, xyz[1], root_name, line)?;
        let z = to_si(&display, Quantity::Length, xyz[2], root_name, line)?;
        model.set_node(id, [x, y, z])?;
    }

    // 2. Materials (sanitized deterministic names, mapping WARP3D material name -> FieldCraft name).
    let mut used_mat_names = BTreeSet::new();
    let mut mat_name_map: BTreeMap<String, String> = BTreeMap::new();
    for (raw_name, raw_mat) in &parsed.materials {
        progress::check()?;
        let fc_name = unique_name(raw_name, "w_", &mut used_mat_names)?;
        let mat = build_material(raw_mat, &parsed.curves, &display, root_name, &mut warnings)?;
        model.add_material(&fc_name, mat)?;
        mat_name_map.insert(raw_name.clone(), fc_name);
    }

    // 3. Apply deferred `elements <list_name>` assignments in sequence order.
    let mut elem_overrides = std::mem::take(&mut parsed.elem_overrides);
    for deferred in &parsed.deferred_elem_lists {
        let Some(raw_list) = parsed.lists.get_mut(&deferred.list_name) else {
            return Err(ReadError::Parse {
                file: root_name.to_string(),
                line: deferred.line,
                message: format!("`elements` references undefined list `{}`", deferred.list_name),
            });
        };
        let ids = match raw_list {
            RawList::Explicit { ids, referenced_by_elements, .. } => {
                *referenced_by_elements = true;
                ids
            }
            RawList::Geometric { referenced_by_elements, .. } => {
                *referenced_by_elements = true;
                return Err(ReadError::Unsupported {
                    file: root_name.to_string(),
                    line: deferred.line,
                    feature: "geometric list in elements".into(),
                    message: format!("geometric list `{}` cannot be referenced by `elements`", deferred.list_name),
                });
            }
        };
        for &eid in ids.iter() {
            if !elem_overrides.contains_key(&eid) && elem_overrides.len() >= limits.elements {
                return Err(ReadError::Limit(format!("element override count exceeds limit {}", limits.elements)));
            }
            let replace = elem_overrides.get(&eid).is_none_or(|(prev_seq, _)| deferred.seq > *prev_seq);
            if replace {
                elem_overrides.insert(eid, (deferred.seq, deferred.spec.clone()));
            }
        }
    }

    // 4. Elements & deterministic section grouping strictly over present incidences.
    let mut any_nlgeom = false;
    let mut collapsed_wedge_count = 0usize;
    let mut collapsed_hex_nodes: BTreeMap<u32, [u32; 8]> = BTreeMap::new();
    let mut groups: BTreeMap<(String, Integration), Vec<u32>> = BTreeMap::new();
    let incidences = std::mem::take(&mut parsed.incidences);
    for (idx, (eid, elem_nodes, inc_line)) in incidences.into_iter().enumerate() {
        if idx.is_multiple_of(256) {
            progress::check()?;
        }
        let chosen: Option<&ElementSpec> = match (&parsed.default_elem_spec, elem_overrides.get(&eid)) {
            (Some((d_seq, d_spec)), Some((o_seq, o_spec))) => Some(if o_seq > d_seq { o_spec } else { d_spec }),
            (Some((_, d_spec)), None) => Some(d_spec),
            (None, Some((_, o_spec))) => Some(o_spec),
            (None, None) => None,
        };
        let spec = chosen.ok_or_else(|| ReadError::Parse {
            file: root_name.to_string(),
            line: inc_line,
            message: format!("element {eid} in `incidences` has no `elements` specification"),
        })?;
        if spec.nlgeom {
            any_nlgeom = true;
        }
        let fc_mat = mat_name_map.get(&spec.material).ok_or_else(|| ReadError::Parse {
            file: root_name.to_string(),
            line: spec.line,
            message: format!("element {eid} references missing material `{}`", spec.material),
        })?;
        if spec.kind == ElementKind::Hex8
            && let Some((hex_orig, wedge_nodes)) = collapse_hex8_to_wedge6(&elem_nodes)
        {
            collapsed_hex_nodes.insert(eid, hex_orig);
            collapsed_wedge_count += 1;
            model.set_element(eid, ElementKind::Wedge6, wedge_nodes.to_vec())?;
        } else {
            model.set_element(eid, spec.kind, elem_nodes)?;
        }
        groups.entry((fc_mat.clone(), spec.integration)).or_default().push(eid);
    }

    if collapsed_wedge_count > 0 {
        warnings.push(format!("imported {collapsed_wedge_count} collapsed 8-node `l3disop` wedge element(s) as `wedge6`"));
    }
    if any_nlgeom {
        warnings.push(
            "WARP3D elements specify geometric nonlinearity (`nonlinear`/`nlgeom`); section formulation does not carry step nonlinearity".into(),
        );
    }

    if let Some(decl_n) = parsed.declared_nodes
        && decl_n != model.nodes().len()
    {
        warnings.push(format!("`number of nodes` declared {decl_n}, but {} node(s) were read", model.nodes().len()));
    }
    if let Some(decl_e) = parsed.declared_elements
        && decl_e != model.elements().len()
    {
        warnings.push(format!("`number of elements` declared {decl_e}, but {} element(s) were read", model.elements().len()));
    }

    // 5. Create element sets & sections.
    let mut used_set_names = BTreeSet::new();
    let mut used_section_names = BTreeSet::new();
    let mut mat_group_idx: BTreeMap<String, usize> = BTreeMap::new();
    for ((fc_mat, integration), mut elem_ids) in groups {
        progress::check()?;
        elem_ids.sort_unstable();
        elem_ids.dedup();
        let idx = mat_group_idx.entry(fc_mat.clone()).or_insert(0);
        *idx += 1;
        let raw_sec_name = format!("sec_{fc_mat}_{idx}");
        let set_name = unique_name(&raw_sec_name, "w_", &mut used_set_names)?;
        let sec_name = unique_name(&raw_sec_name, "w_", &mut used_section_names)?;
        model.add_set(&set_name, Set { elements: elem_ids, ..Set::default() })?;
        model.add_section(
            &sec_name,
            Section {
                set: set_name,
                material: fc_mat,
                kind: SectionKind::Solid {},
                formulation: Formulation { integration, ..Formulation::default() },
            },
        )?;
    }

    // 6. Mark list roles from constraints & loading, then resolve explicit vs geometric `list` definitions.
    let mark_selectors =
        |selectors: &[IdSelector], lists: &mut BTreeMap<String, RawList>, is_elem: bool, file: &str, line: usize| -> Result<(), ReadError> {
            for sel in selectors {
                if let IdSelector::List(name) = sel {
                    let Some(raw) = lists.get_mut(name) else {
                        return Err(ReadError::Parse { file: file.to_string(), line, message: format!("reference to undefined list `{name}`") });
                    };
                    match raw {
                        RawList::Explicit { referenced_by_elements, referenced_by_nodes, .. }
                        | RawList::Geometric { referenced_by_elements, referenced_by_nodes, .. } => {
                            if is_elem {
                                *referenced_by_elements = true;
                            } else {
                                *referenced_by_nodes = true;
                            }
                        }
                    }
                }
            }
            Ok(())
        };
    for c in &parsed.nodal_constraints {
        mark_selectors(&c.selectors, &mut parsed.lists, false, &c.file, c.line)?;
    }
    for l in &parsed.nodal_loads {
        mark_selectors(&l.selectors, &mut parsed.lists, false, &l.file, l.line)?;
    }
    for l in &parsed.elem_pressures {
        mark_selectors(&l.selectors, &mut parsed.lists, true, &l.file, l.line)?;
    }
    for l in &parsed.elem_tractions {
        mark_selectors(&l.selectors, &mut parsed.lists, true, &l.file, l.line)?;
    }
    for l in &parsed.elem_bodies {
        mark_selectors(&l.selectors, &mut parsed.lists, true, &l.file, l.line)?;
    }

    for (name, raw) in &parsed.lists {
        progress::check()?;
        match raw {
            RawList::Geometric { line, referenced_by_elements, referenced_by_nodes } => {
                if *referenced_by_elements || *referenced_by_nodes {
                    return Err(ReadError::Unsupported {
                        file: root_name.to_string(),
                        line: *line,
                        feature: "geometric list".into(),
                        message: format!("geometric list `{name}` is not supported"),
                    });
                }
                warnings.push(format!("geometric list `{name}` (line {line}) was ignored"));
            }
            RawList::Explicit { ids, line, referenced_by_elements, referenced_by_nodes } => {
                let all_elems = ids.iter().all(|id| model.element(*id).is_some());
                let all_nodes = ids.iter().all(|id| model.node(*id).is_some());
                if *referenced_by_elements && !all_elems {
                    return Err(ReadError::Parse {
                        file: root_name.to_string(),
                        line: *line,
                        message: format!("list `{name}` referenced by elements names missing element IDs"),
                    });
                }
                if *referenced_by_nodes && !all_nodes {
                    return Err(ReadError::Parse {
                        file: root_name.to_string(),
                        line: *line,
                        message: format!("list `{name}` referenced by nodes names missing node IDs"),
                    });
                }
                if *referenced_by_elements && !*referenced_by_nodes {
                    // Referenced solely as an element list: keep parser-internal without emitting a node Set.
                    continue;
                }
                if all_nodes && !ids.is_empty() {
                    let set_name = unique_name(&format!("ls_{name}"), "w_", &mut used_set_names)?;
                    model.add_set(&set_name, Set { nodes: ids.clone(), ..Set::default() })?;
                } else if !*referenced_by_elements && !*referenced_by_nodes && !all_elems {
                    warnings.push(format!("list `{name}` (line {line}) references missing node IDs and was skipped"));
                }
            }
        }
    }

    // 7. Lower `constraints` (nodal, deferred `plane`, and single-node zero-RHS `multipoint`).
    build_constraints(&mut model, &parsed, &display, &mut used_set_names, &mut warnings)?;

    // 8. Lower static `loading <name>` patterns (per-pattern scoping & Hex 4 <-> 5 swap).
    build_loads(&mut model, &parsed, &display, &collapsed_hex_nodes, &mut used_set_names)?;

    Ok(ImportedDeck {
        model,
        units,
        units_source,
        unknown: parsed.unknown,
        unknown_count: parsed.unknown_count,
        warnings,
        stopped_at: parsed.stopped_at,
        includes_read: parsed.includes_read,
    })
}

fn expand_node_selectors<F>(
    model: &Model,
    selectors: &[IdSelector],
    lists: &BTreeMap<String, RawList>,
    file: &str,
    line: usize,
    mut on_node: F,
) -> Result<(), ReadError>
where
    F: FnMut(u32) -> Result<(), ReadError>,
{
    for sel in selectors {
        match sel {
            IdSelector::All => {
                for (idx, &nid) in model.nodes().keys().enumerate() {
                    if idx.is_multiple_of(256) {
                        progress::check()?;
                    }
                    on_node(nid)?;
                }
            }
            IdSelector::Ids(ids) => {
                for (idx, &nid) in ids.iter().enumerate() {
                    if idx.is_multiple_of(256) {
                        progress::check()?;
                    }
                    if model.node(nid).is_none() {
                        return Err(ReadError::Parse { file: file.to_string(), line, message: format!("reference to missing node {nid}") });
                    }
                    on_node(nid)?;
                }
            }
            IdSelector::List(name) => {
                let Some(RawList::Explicit { ids, .. }) = lists.get(name) else {
                    return Err(ReadError::Parse { file: file.to_string(), line, message: format!("invalid node list `{name}`") });
                };
                for (idx, &nid) in ids.iter().enumerate() {
                    if idx.is_multiple_of(256) {
                        progress::check()?;
                    }
                    on_node(nid)?;
                }
            }
        }
    }
    Ok(())
}

fn expand_elem_selectors<F>(
    model: &Model,
    selectors: &[IdSelector],
    lists: &BTreeMap<String, RawList>,
    file: &str,
    line: usize,
    mut on_elem: F,
) -> Result<(), ReadError>
where
    F: FnMut(u32) -> Result<(), ReadError>,
{
    for sel in selectors {
        match sel {
            IdSelector::All => {
                for (idx, &eid) in model.elements().keys().enumerate() {
                    if idx.is_multiple_of(256) {
                        progress::check()?;
                    }
                    on_elem(eid)?;
                }
            }
            IdSelector::Ids(ids) => {
                for (idx, &eid) in ids.iter().enumerate() {
                    if idx.is_multiple_of(256) {
                        progress::check()?;
                    }
                    if model.element(eid).is_none() {
                        return Err(ReadError::Parse { file: file.to_string(), line, message: format!("reference to missing element {eid}") });
                    }
                    on_elem(eid)?;
                }
            }
            IdSelector::List(name) => {
                let Some(RawList::Explicit { ids, .. }) = lists.get(name) else {
                    return Err(ReadError::Parse { file: file.to_string(), line, message: format!("invalid element list `{name}`") });
                };
                for (idx, &eid) in ids.iter().enumerate() {
                    if idx.is_multiple_of(256) {
                        progress::check()?;
                    }
                    on_elem(eid)?;
                }
            }
        }
    }
    Ok(())
}

fn merge_node_dof(
    node_dofs: &mut BTreeMap<u32, [Option<f64>; 3]>,
    nid: u32,
    axis: usize,
    val_si: f64,
    file: &str,
    line: usize,
) -> Result<(), ReadError> {
    let val_si = canon_f64(val_si);
    let entry = node_dofs.entry(nid).or_insert([None; 3]);
    let Some(slot) = entry.get_mut(axis) else {
        return Ok(());
    };
    if let Some(prev) = *slot
        && (prev - val_si).abs() > DOF_CONFLICT_TOL
    {
        return Err(ReadError::Parse {
            file: file.to_string(),
            line,
            message: format!("conflicting displacement constraint on node {nid} DOF {}", axis + 1),
        });
    }
    *slot = Some(val_si);
    Ok(())
}

fn build_constraints(
    model: &mut Model,
    parsed: &ParsedDeck,
    display: &DisplaySystem,
    used_set_names: &mut BTreeSet<String>,
    warnings: &mut Vec<String>,
) -> Result<(), ReadError> {
    let mut node_dofs: BTreeMap<u32, [Option<f64>; 3]> = BTreeMap::new();

    for raw in &parsed.nodal_constraints {
        let mut si_vals = [None; 3];
        for (ax, slot) in si_vals.iter_mut().enumerate() {
            if let Some(v) = raw.values.get(ax).copied().flatten() {
                *slot = Some(canon_f64(to_si(display, Quantity::Length, v, &raw.file, raw.line)?));
            }
        }
        expand_node_selectors(model, &raw.selectors, &parsed.lists, &raw.file, raw.line, |nid| {
            for (ax, slot) in si_vals.iter().enumerate() {
                if let Some(v) = *slot {
                    merge_node_dof(&mut node_dofs, nid, ax, v, &raw.file, raw.line)?;
                }
            }
            Ok(())
        })?;
    }

    if !parsed.deferred_planes.is_empty() {
        let mut min_xyz = [f64::INFINITY; 3];
        let mut max_xyz = [f64::NEG_INFINITY; 3];
        for node in model.nodes().values() {
            for ax in 0..3 {
                min_xyz[ax] = min_xyz[ax].min(node.xyz[ax]);
                max_xyz[ax] = max_xyz[ax].max(node.xyz[ax]);
            }
        }
        let spans = if model.nodes().is_empty() { [0.0; 3] } else { [0, 1, 2].map(|ax| max_xyz[ax] - min_xyz[ax]) };

        let mut seen_planes: BTreeSet<(usize, u64, Option<u64>, [Option<u64>; 3])> = BTreeSet::new();
        for plane in &parsed.deferred_planes {
            progress::check()?;
            let coord_si = canon_f64(to_si(display, Quantity::Length, plane.value, &plane.file, plane.line)?);
            let prox_si = match plane.proximity {
                Some(p) => Some(canon_f64(to_si(display, Quantity::Length, p, &plane.file, plane.line)?)),
                None => None,
            };
            let target_dofs: [Option<f64>; 3] = match plane.action {
                PlaneAction::Fixed => [Some(0.0); 3],
                PlaneAction::Symmetry => {
                    let mut d = [None; 3];
                    if let Some(slot) = d.get_mut(plane.axis) {
                        *slot = Some(0.0);
                    }
                    d
                }
                PlaneAction::Dofs(arr) => {
                    let mut d = [None; 3];
                    for (ax, slot) in d.iter_mut().enumerate() {
                        if let Some(v) = arr.get(ax).copied().flatten() {
                            *slot = Some(canon_f64(to_si(display, Quantity::Length, v, &plane.file, plane.line)?));
                        }
                    }
                    d
                }
            };
            let action_key = target_dofs.map(|v| v.map(f64::to_bits));
            if !seen_planes.insert((plane.axis, coord_si.to_bits(), prox_si.map(f64::to_bits), action_key)) {
                continue;
            }
            let tol = match prox_si {
                Some(p) => p,
                None => {
                    let span = spans.get(plane.axis).copied().unwrap_or(0.0);
                    if span == 0.0 { ELASTIC_TOL } else { PLANE_BBOX_FRAC * span }
                }
            };

            let mut matched = 0usize;
            for (&nid, node) in model.nodes() {
                if (node.xyz[plane.axis] - coord_si).abs() <= tol {
                    matched += 1;
                    for (ax, slot) in target_dofs.iter().enumerate() {
                        if let Some(v) = *slot {
                            merge_node_dof(&mut node_dofs, nid, ax, v, &plane.file, plane.line)?;
                        }
                    }
                }
            }
            if matched == 0 {
                warnings.push(format!("`plane` constraint ({}:{}) matched 0 nodes and was skipped", plane.file, plane.line));
            }
        }
    }

    let mut bc_groups: BTreeMap<[Option<u64>; 3], (Vec<u32>, [Option<f64>; 3])> = BTreeMap::new();
    for (nid, vals) in node_dofs {
        if vals.iter().all(Option::is_none) {
            continue;
        }
        let key = vals.map(|v| v.map(|x| canon_f64(x).to_bits()));
        let entry = bc_groups.entry(key).or_insert_with(|| (Vec::new(), vals));
        entry.0.push(nid);
    }

    let mut used_bc_names = BTreeSet::new();
    for (idx, (_, (mut nids, vals))) in bc_groups.into_iter().enumerate() {
        progress::check()?;
        nids.sort_unstable();
        nids.dedup();
        let bc_num = idx + 1;
        let raw_name = format!("bc_{bc_num}");
        let set_name = unique_name(&raw_name, "w_", used_set_names)?;
        let bc_name = unique_name(&raw_name, "w_", &mut used_bc_names)?;
        model.add_set(&set_name, Set { nodes: nids, ..Set::default() })?;
        let bc = if vals == [Some(0.0), Some(0.0), Some(0.0)] {
            BoundaryCondition::Fixed { target: Target::Set(set_name) }
        } else {
            BoundaryCondition::Displacement { target: Target::Set(set_name), values: [vals[0], vals[1], vals[2], None, None, None] }
        };
        model.add_boundary_condition(&bc_name, bc)?;
    }

    if !parsed.multipoints.is_empty() {
        #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
        enum SymKey {
            X,
            Y,
            Z,
            Normal([u64; 3]),
        }
        let mut mpc_groups: BTreeMap<SymKey, (Symmetry, BTreeSet<u32>)> = BTreeMap::new();
        for mpc in &parsed.multipoints {
            if model.node(mpc.node).is_none() {
                return Err(ReadError::Parse {
                    file: mpc.file.clone(),
                    line: mpc.line,
                    message: format!("`multipoint` constraint references missing node {}", mpc.node),
                });
            }
            let mut raw = [0.0_f64; 3];
            for &(ax, c) in &mpc.terms {
                if let Some(slot) = raw.get_mut(ax) {
                    *slot += c;
                }
            }
            let norm = (raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]).sqrt();
            if !norm.is_finite() || norm == 0.0 {
                return Err(ReadError::Parse { file: mpc.file.clone(), line: mpc.line, message: "`multipoint` equation has zero normal".into() });
            }
            let mut unit = [raw[0] / norm, raw[1] / norm, raw[2] / norm];
            if !unit.iter().all(|c| c.is_finite()) {
                return Err(ReadError::Parse {
                    file: mpc.file.clone(),
                    line: mpc.line,
                    message: "`multipoint` equation normalized to non-finite normal".into(),
                });
            }
            if let Some(&first_sig) = unit.iter().find(|c| c.abs() > ELASTIC_TOL)
                && first_sig < 0.0
            {
                unit = unit.map(|c| -c);
            }
            unit = unit.map(|c| canon_f64((c / MPC_ROUND_STEP).round() * MPC_ROUND_STEP));
            let Some(sym) = Symmetry::from_normal(unit) else {
                return Err(ReadError::Parse {
                    file: mpc.file.clone(),
                    line: mpc.line,
                    message: "`multipoint` equation produced an invalid symmetry plane normal".into(),
                });
            };
            let key = match sym {
                Symmetry::X => SymKey::X,
                Symmetry::Y => SymKey::Y,
                Symmetry::Z => SymKey::Z,
                Symmetry::Normal(n) => SymKey::Normal(canon_vec3_bits(n)),
            };
            mpc_groups.entry(key).or_insert_with(|| (sym, BTreeSet::new())).1.insert(mpc.node);
        }

        for (idx, (_, (sym, nids))) in mpc_groups.into_iter().enumerate() {
            progress::check()?;
            let mpc_num = idx + 1;
            let raw_name = format!("mpc_{mpc_num}");
            let set_name = unique_name(&raw_name, "w_", used_set_names)?;
            let bc_name = unique_name(&raw_name, "w_", &mut used_bc_names)?;
            model.add_set(&set_name, Set { nodes: nids.into_iter().collect(), ..Set::default() })?;
            model.add_boundary_condition(&bc_name, BoundaryCondition::Symmetry { target: Target::Set(set_name), plane: sym })?;
        }
    }

    Ok(())
}

/// Collapse a degenerate 8-node `l3disop` hex into a right-handed 6-node `Wedge6` when one quad
/// side has collapsed to two opposite edges (`a == b && c == d`) leaving 6 distinct vertices.
///
/// Every collapse pattern is cyclically canonicalized so the collapsed edge always lands on
/// `(w2, w5)` (`candidate[2]` and `candidate[5]`), keeping round-trip `l3disop` exports aligned
/// even when no crack-front node set is specified.
fn collapse_hex8_to_wedge6(nodes: &[u32]) -> Option<([u32; 8], [u32; 6])> {
    let [h0, h1, h2, h3, h4, h5, h6, h7] =
        [*nodes.first()?, *nodes.get(1)?, *nodes.get(2)?, *nodes.get(3)?, *nodes.get(4)?, *nodes.get(5)?, *nodes.get(6)?, *nodes.get(7)?];
    let hex_orig = [h0, h1, h2, h3, h4, h5, h6, h7];
    let candidate = if h0 == h3 && h4 == h7 {
        [h1, h2, h0, h5, h6, h4]
    } else if h1 == h2 && h5 == h6 {
        [h3, h0, h1, h7, h4, h5]
    } else if h0 == h1 && h4 == h5 {
        [h2, h3, h0, h6, h7, h4]
    } else if h2 == h3 && h6 == h7 {
        [h0, h1, h2, h4, h5, h6]
    } else if h0 == h4 && h1 == h5 {
        [h3, h7, h0, h2, h6, h1]
    } else if h2 == h6 && h3 == h7 {
        [h4, h0, h3, h5, h1, h2]
    } else if h0 == h4 && h3 == h7 {
        [h5, h1, h0, h6, h2, h3]
    } else if h1 == h5 && h2 == h6 {
        [h0, h4, h1, h3, h7, h2]
    } else if h0 == h3 && h1 == h2 {
        [h7, h4, h0, h6, h5, h1]
    } else if h4 == h7 && h5 == h6 {
        [h0, h3, h4, h1, h2, h5]
    } else if h0 == h1 && h3 == h2 {
        [h4, h5, h0, h7, h6, h3]
    } else if h4 == h5 && h7 == h6 {
        [h1, h0, h4, h2, h3, h7]
    } else {
        return None;
    };
    for (i, a) in candidate.iter().enumerate() {
        if candidate.get(..i).is_some_and(|before| before.contains(a)) {
            return None;
        }
    }
    Some((hex_orig, candidate))
}

fn map_warp_face(
    elem_id: u32,
    elem: &fieldcraft_model::Element,
    raw_face: u8,
    is_abaqus: bool,
    collapsed_hex_nodes: &BTreeMap<u32, [u32; 8]>,
    file: &str,
    line: usize,
) -> Result<u8, ReadError> {
    match elem.kind {
        ElementKind::Hex8 | ElementKind::Hex20 => {
            if is_abaqus {
                Ok(raw_face)
            } else {
                crate::deck::warp3d_face(elem.kind, raw_face).ok_or_else(|| ReadError::Parse {
                    file: file.to_string(),
                    line,
                    message: format!("invalid face {raw_face} for `{}` element", elem.kind.name()),
                })
            }
        }
        ElementKind::Wedge6 => {
            let Some(hex_orig) = collapsed_hex_nodes.get(&elem_id) else {
                return Err(ReadError::Unsupported {
                    file: file.to_string(),
                    line,
                    feature: "element face load".into(),
                    message: "face loads on native `wedge6` elements without hex provenance are not supported".into(),
                });
            };
            let fc_hex_face = if is_abaqus {
                if (1..=6).contains(&raw_face) {
                    raw_face
                } else {
                    return Err(ReadError::Parse {
                        file: file.to_string(),
                        line,
                        message: format!("invalid Abaqus face {raw_face} for collapsed `l3disop` element {elem_id}"),
                    });
                }
            } else {
                crate::deck::warp3d_face(ElementKind::Hex8, raw_face).ok_or_else(|| ReadError::Parse {
                    file: file.to_string(),
                    line,
                    message: format!("invalid face {raw_face} for collapsed `l3disop` element {elem_id}"),
                })?
            };
            let hex_face_indices = ElementKind::Hex8.faces().get((fc_hex_face - 1) as usize).ok_or_else(|| ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("invalid face {raw_face} for collapsed `l3disop` element {elem_id}"),
            })?;
            let target_nodes: BTreeSet<u32> = hex_face_indices.iter().filter_map(|&idx| hex_orig.get(idx).copied()).collect();
            for (f_idx, w_indices) in ElementKind::Wedge6.faces().iter().enumerate() {
                let w_nodes: BTreeSet<u32> = w_indices.iter().filter_map(|&idx| elem.nodes.get(idx).copied()).collect();
                if w_nodes == target_nodes {
                    return Ok((f_idx + 1) as u8);
                }
            }
            Err(ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("load references collapsed degenerate face {raw_face} on element {elem_id}"),
            })
        }
        ElementKind::Tet4 | ElementKind::Tet10 => {
            if is_abaqus {
                return Err(ReadError::Unsupported {
                    file: file.to_string(),
                    line,
                    feature: "face abaqus on tet".into(),
                    message: "`face abaqus` is only supported on hexahedral elements".into(),
                });
            }
            if !(1..=4).contains(&raw_face) {
                return Err(ReadError::Parse {
                    file: file.to_string(),
                    line,
                    message: format!("face {raw_face} is outside 1..=4 for `{}` element", elem.kind.name()),
                });
            }
            Ok(raw_face)
        }
        _ => Err(ReadError::Unsupported {
            file: file.to_string(),
            line,
            feature: "element face load".into(),
            message: format!("face loads on `{}` are not supported", elem.kind.name()),
        }),
    }
}

#[derive(Default)]
struct PatternAccum {
    nodal_forces: BTreeMap<u32, [f64; 3]>,
    pressures: BTreeMap<(u32, u8), f64>,
    tractions: BTreeMap<(u32, u8), [f64; 3]>,
    bodies: BTreeMap<u32, [f64; 3]>,
}

fn build_loads(
    model: &mut Model,
    parsed: &ParsedDeck,
    display: &DisplaySystem,
    collapsed_hex_nodes: &BTreeMap<u32, [u32; 8]>,
    used_set_names: &mut BTreeSet<String>,
) -> Result<(), ReadError> {
    let mut patterns: BTreeMap<String, PatternAccum> = BTreeMap::new();

    for raw in &parsed.nodal_loads {
        let fx = to_si(display, Quantity::Force, raw.force[0], &raw.file, raw.line)?;
        let fy = to_si(display, Quantity::Force, raw.force[1], &raw.file, raw.line)?;
        let fz = to_si(display, Quantity::Force, raw.force[2], &raw.file, raw.line)?;
        let pat = patterns.entry(raw.pattern.clone()).or_default();
        expand_node_selectors(model, &raw.selectors, &parsed.lists, &raw.file, raw.line, |nid| {
            let acc = pat.nodal_forces.entry(nid).or_insert([0.0; 3]);
            acc[0] += fx;
            acc[1] += fy;
            acc[2] += fz;
            Ok(())
        })?;
    }

    for raw in &parsed.elem_pressures {
        let p_si = to_si(display, Quantity::Pressure, raw.pressure, &raw.file, raw.line)?;
        let pat = patterns.entry(raw.pattern.clone()).or_default();
        expand_elem_selectors(model, &raw.selectors, &parsed.lists, &raw.file, raw.line, |eid| {
            let Some(elem) = model.element(eid) else {
                return Err(ReadError::Parse { file: raw.file.clone(), line: raw.line, message: format!("missing element {eid}") });
            };
            let fc_face = map_warp_face(eid, elem, raw.face, raw.is_abaqus, collapsed_hex_nodes, &raw.file, raw.line)?;
            let acc = pat.pressures.entry((eid, fc_face)).or_insert(0.0);
            *acc += p_si;
            Ok(())
        })?;
    }

    for raw in &parsed.elem_tractions {
        let tx = to_si(display, Quantity::Pressure, raw.traction[0], &raw.file, raw.line)?;
        let ty = to_si(display, Quantity::Pressure, raw.traction[1], &raw.file, raw.line)?;
        let tz = to_si(display, Quantity::Pressure, raw.traction[2], &raw.file, raw.line)?;
        let pat = patterns.entry(raw.pattern.clone()).or_default();
        expand_elem_selectors(model, &raw.selectors, &parsed.lists, &raw.file, raw.line, |eid| {
            let Some(elem) = model.element(eid) else {
                return Err(ReadError::Parse { file: raw.file.clone(), line: raw.line, message: format!("missing element {eid}") });
            };
            let fc_face = map_warp_face(eid, elem, raw.face, raw.is_abaqus, collapsed_hex_nodes, &raw.file, raw.line)?;
            let acc = pat.tractions.entry((eid, fc_face)).or_insert([0.0; 3]);
            acc[0] += tx;
            acc[1] += ty;
            acc[2] += tz;
            Ok(())
        })?;
    }

    for raw in &parsed.elem_bodies {
        let bx = to_si(display, Quantity::ForcePerVolume, raw.body[0], &raw.file, raw.line)?;
        let by = to_si(display, Quantity::ForcePerVolume, raw.body[1], &raw.file, raw.line)?;
        let bz = to_si(display, Quantity::ForcePerVolume, raw.body[2], &raw.file, raw.line)?;
        let pat = patterns.entry(raw.pattern.clone()).or_default();
        expand_elem_selectors(model, &raw.selectors, &parsed.lists, &raw.file, raw.line, |eid| {
            let acc = pat.bodies.entry(eid).or_insert([0.0; 3]);
            acc[0] += bx;
            acc[1] += by;
            acc[2] += bz;
            Ok(())
        })?;
    }

    let mut used_load_names = BTreeSet::new();
    for (pattern, accum) in patterns {
        let mut pat_idx = 0usize;

        let mut force_groups: BTreeMap<[u64; 3], (Vec<u32>, [f64; 3])> = BTreeMap::new();
        for (nid, vec) in accum.nodal_forces {
            let vec = vec.map(canon_f64);
            if vec == [0.0, 0.0, 0.0] {
                continue;
            }
            force_groups.entry(canon_vec3_bits(vec)).or_insert_with(|| (Vec::new(), vec)).0.push(nid);
        }
        for (_, (nids, vec)) in force_groups {
            progress::check()?;
            pat_idx += 1;
            let set_name = unique_name(&format!("sf_{pattern}_{pat_idx}"), "w_", used_set_names)?;
            let load_name = unique_name(&format!("ld_{pattern}_{pat_idx}"), "w_", &mut used_load_names)?;
            model.add_set(&set_name, Set { nodes: nids, ..Set::default() })?;
            model.add_load(&load_name, Load::Force { target: Target::Set(set_name), vector: vec, distribution: Distribution::PerNode })?;
        }

        let mut press_groups: BTreeMap<u64, (Vec<ElementFace>, f64)> = BTreeMap::new();
        for ((eid, face), mag) in accum.pressures {
            let mag = canon_f64(mag);
            if mag == 0.0 {
                continue;
            }
            press_groups.entry(mag.to_bits()).or_insert_with(|| (Vec::new(), mag)).0.push(ElementFace { element: eid, face });
        }
        for (_, (faces, magnitude)) in press_groups {
            progress::check()?;
            pat_idx += 1;
            let set_name = unique_name(&format!("sf_{pattern}_{pat_idx}"), "w_", used_set_names)?;
            let load_name = unique_name(&format!("ld_{pattern}_{pat_idx}"), "w_", &mut used_load_names)?;
            model.add_set(&set_name, Set { faces, ..Set::default() })?;
            model.add_load(&load_name, Load::Pressure { set: set_name, magnitude })?;
        }

        let mut trac_groups: BTreeMap<[u64; 3], (Vec<ElementFace>, [f64; 3])> = BTreeMap::new();
        for ((eid, face), vec) in accum.tractions {
            let vec = vec.map(canon_f64);
            if vec == [0.0, 0.0, 0.0] {
                continue;
            }
            trac_groups.entry(canon_vec3_bits(vec)).or_insert_with(|| (Vec::new(), vec)).0.push(ElementFace { element: eid, face });
        }
        for (_, (faces, vector)) in trac_groups {
            progress::check()?;
            pat_idx += 1;
            let set_name = unique_name(&format!("sf_{pattern}_{pat_idx}"), "w_", used_set_names)?;
            let load_name = unique_name(&format!("ld_{pattern}_{pat_idx}"), "w_", &mut used_load_names)?;
            model.add_set(&set_name, Set { faces, ..Set::default() })?;
            model.add_load(&load_name, Load::Traction { set: set_name, vector })?;
        }

        let mut body_groups: BTreeMap<[u64; 3], (Vec<u32>, [f64; 3])> = BTreeMap::new();
        for (eid, vec) in accum.bodies {
            let vec = vec.map(canon_f64);
            if vec == [0.0, 0.0, 0.0] {
                continue;
            }
            body_groups.entry(canon_vec3_bits(vec)).or_insert_with(|| (Vec::new(), vec)).0.push(eid);
        }
        for (_, (elements, vector)) in body_groups {
            progress::check()?;
            pat_idx += 1;
            let set_name = unique_name(&format!("sf_{pattern}_{pat_idx}"), "w_", used_set_names)?;
            let load_name = unique_name(&format!("ld_{pattern}_{pat_idx}"), "w_", &mut used_load_names)?;
            model.add_set(&set_name, Set { elements, ..Set::default() })?;
            model.add_load(&load_name, Load::BodyForce { set: set_name, vector })?;
        }
    }

    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    fn signed_tet_vol(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3]) -> f64 {
        let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let ad = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
        let cross = [ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]];
        cross[0] * ad[0] + cross[1] * ad[1] + cross[2] * ad[2]
    }

    #[test]
    fn collapse_hex8_to_wedge6_canonicalizes_all_12_patterns_onto_w2_w5_with_positive_jacobian() {
        let hex_coords =
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0]];

        // All 12 opposite-edge collapse pairs: ((a0, b0), (a1, b1)) where a0==b0 and a1==b1.
        let patterns: [((usize, usize), (usize, usize)); 12] = [
            ((0, 3), (4, 7)),
            ((1, 2), (5, 6)),
            ((0, 1), (4, 5)),
            ((2, 3), (6, 7)),
            ((0, 4), (1, 5)),
            ((3, 7), (2, 6)),
            ((0, 4), (3, 7)),
            ((1, 5), (2, 6)),
            ((0, 3), (1, 2)),
            ((4, 7), (5, 6)),
            ((0, 1), (3, 2)),
            ((4, 5), (7, 6)),
        ];

        for ((a0, b0), (a1, b1)) in patterns {
            let mut coords: BTreeMap<u32, [f64; 3]> = BTreeMap::new();
            let mut inc = [0u32; 8];
            let mut next_id = 1u32;
            let mid = |u: usize, v: usize| [0, 1, 2].map(|k| 0.5 * (hex_coords[u][k] + hex_coords[v][k]));
            let id_c0 = next_id;
            next_id += 1;
            coords.insert(id_c0, mid(a0, b0));
            let id_c1 = next_id;
            next_id += 1;
            coords.insert(id_c1, mid(a1, b1));

            for i in 0..8 {
                if i == a0 || i == b0 {
                    inc[i] = id_c0;
                } else if i == a1 || i == b1 {
                    inc[i] = id_c1;
                } else {
                    let id = next_id;
                    next_id += 1;
                    coords.insert(id, hex_coords[i]);
                    inc[i] = id;
                }
            }

            let (_, wedge) = collapse_hex8_to_wedge6(&inc).expect("pattern should collapse to wedge6");
            // Collapsed edge must land on canonical (w2, w5).
            assert_eq!((wedge[2], wedge[5]), (id_c0, id_c1), "pattern (({a0},{b0}),({a1},{b1})) -> {wedge:?}");

            // Right-handed positive Jacobian at both triangular end caps.
            let p: Vec<[f64; 3]> = wedge.iter().map(|id| coords[id]).collect();
            assert!(signed_tet_vol(p[0], p[1], p[2], p[3]) > 0.0, "negative Jacobian at base for {wedge:?}");
            assert!(signed_tet_vol(p[3], p[5], p[4], p[0]) > 0.0, "negative Jacobian at top for {wedge:?}");

            // Default rot=0 expansion puts the canonical collapsed edge back onto (h2==h3, h6==h7).
            let expanded = crate::wedge6::expand_wedge6_to_l3disop(&wedge, 0).unwrap();
            assert_eq!((expanded[2], expanded[3], expanded[6], expanded[7]), (id_c0, id_c0, id_c1, id_c1));
        }
    }
}

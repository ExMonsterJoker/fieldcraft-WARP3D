//! `results.ffs`: crack-front fracture profiles, ring-exclusion diagnostics, API 579 Option 1/2
//! Failure Assessment Diagram (FAD) evaluation, and RFC 4180 CSV export.

use std::collections::BTreeMap;

use fieldcraft_io::roots::Access;
use fieldcraft_model::units::Quantity;
use fieldcraft_warp3d::ffs::{
    FadConfig, FadOption, FadPoint, FfsReport, FfsRequest, KrSource, LimitingMode, LrInput, Option2Curve, ReactionComponent, RingExclusion,
    SpreadStatus, StationProfile, StationSelection, ToughnessInput, build_station_profiles, evaluate_step_ffs_detailed,
};
use serde_json::{Map, Value, json};

use super::results::{Cell, Table, titled};
use crate::schema::ProblemKind;
use crate::{CommandKind, CommandSpec, EngineError, Result, Session};

pub const FFS: CommandSpec = CommandSpec {
    id: "results.ffs",
    label: "Fitness-for-Service (FAD)",
    params: r#"{"step": "Load", "frame": null, "exclude_inner_rings": "auto", "fad": {"option": "option1", "option2_source": "model_material", "ramberg_alpha": 1.0, "ramberg_n": 5.0, "lr_mode": "reference_stress", "sigma_ref_full": 150.0, "l_r_full": 0.5, "bc_set": "clamp", "reaction_component": "magnitude", "limit_load": 100000.0, "toughness_mode": "k_mat", "k_mat": 100.0, "j_ic": 45.0, "j_ic_plane_strain": true, "kr_source": "auto", "station_mode": "governing", "domain_id": "cf_0001", "sigma_ys": 400.0, "sigma_uts": 550.0, "l_r_max": 1.18, "young": 210000.0, "poisson": 0.3}, "path": "ffs_report.csv"} — evaluate ring-filtered WARP3D crack-front fracture profiles (J, K_J, K_I, K_II, K_III, T-stress, ring spread) and optional API 579-1 / ASME FFS-1 Option 1 or Option 2 Failure Assessment Diagram trajectory and active-frame station verdicts in display units. Optional `path` writes an RFC 4180 CSV report (`trajectory` rows across increments followed by `station` rows at the active frame) in display units. Replies with the FfsReport (plus `path`, `rows`, `columns` when CSV is written)."#,
    form: Some(super::forms::RESULTS_FFS),
    kind: CommandKind::Session,
    requires: &[crate::requirement::Requirement::Result],
    run: run_ffs,
};

const MAX_FRAME_INDEX: usize = 1_000_000_000;

const ALLOWED_FAD_KEYS: &[&str] = &[
    "option",
    "option2_source",
    "ramberg_alpha",
    "ramberg_n",
    "lr_mode",
    "sigma_ref_full",
    "l_r_full",
    "bc_set",
    "reaction_component",
    "limit_load",
    "toughness_mode",
    "k_mat",
    "j_ic",
    "j_ic_plane_strain",
    "kr_source",
    "station_mode",
    "domain_id",
    "sigma_ys",
    "sigma_uts",
    "l_r_max",
    "young",
    "poisson",
];

fn run_ffs(s: &mut Session, p: &Value) -> Result<Value> {
    let Some(obj) = p.as_object() else {
        return Err(EngineError::param("", ProblemKind::NotAnObject));
    };
    if let Some(problem) = crate::schema::problems(super::forms::RESULTS_FFS, p).into_iter().next() {
        return Err(EngineError::Param(problem));
    }

    let requested_step = match obj.get("step") {
        None | Some(Value::Null) => None,
        Some(Value::String(st)) => {
            let trimmed = st.trim();
            if trimmed.is_empty() {
                return Err(EngineError::invalid("step", "step name must not be empty"));
            }
            Some(trimmed.to_string())
        }
        Some(_) => return Err(EngineError::param("step", ProblemKind::NotText)),
    };

    let active_frame = match obj.get("frame") {
        None | Some(Value::Null) => None,
        Some(v) => Some(
            v.as_u64()
                .and_then(|n| usize::try_from(n).ok())
                .filter(|&n| n <= MAX_FRAME_INDEX)
                .ok_or_else(|| EngineError::param("frame", ProblemKind::NotAWholeNumber { min: 0, max: MAX_FRAME_INDEX as i64 }))?,
        ),
    };

    let ring_exclusion = parse_ring_exclusion(obj.get("exclude_inner_rings"))?;
    let fad = parse_fad_config(obj.get("fad"))?;

    let csv_path = match obj.get("path") {
        None | Some(Value::Null) => None,
        Some(Value::String(raw)) => {
            let trimmed = raw.trim();
            if trimmed.is_empty() {
                return Err(EngineError::invalid("path", "enter a CSV file path"));
            }
            Some(trimmed.to_string())
        }
        Some(_) => return Err(EngineError::param("path", ProblemKind::NotText)),
    };

    let step_name = resolve_fracture_step(s, requested_step.as_deref())?;
    let sr = match s.result(&step_name).map(|r| r.kind()) {
        Some(crate::results::Kind::Static(sr)) => sr,
        Some(_) => {
            return Err(EngineError::Params(format!("step `{step_name}` is not a static step with WARP3D fracture results")));
        }
        None => {
            if s.fracture_results.contains_key(&step_name) {
                return Err(EngineError::Params(format!(
                    "the fracture result of step `{step_name}` is stale (the model changed since it was solved); solve it again"
                )));
            }
            return Err(EngineError::Params(format!(
                "no WARP3D fracture results are stored for step `{step_name}`; solve or import a WARP3D model with a crack_front first"
            )));
        }
    };
    let Some(fdata) = s.fracture_data(&step_name) else {
        if s.fracture_results.contains_key(&step_name) {
            return Err(EngineError::Params(format!(
                "the fracture result of step `{step_name}` is stale (the model changed since it was solved); solve it again"
            )));
        }
        return Err(EngineError::Params(format!(
            "no WARP3D fracture results are stored for step `{step_name}`; solve or import a WARP3D model with a crack_front first"
        )));
    };
    if fdata.domains.is_empty() {
        return Err(EngineError::Params(format!(
            "no WARP3D fracture results are stored for step `{step_name}`; solve or import a WARP3D model with a crack_front first"
        )));
    }

    let req = FfsRequest { step: step_name.clone(), ring_exclusion, active_frame, fad, step1_is_elastic: fdata.step1_is_elastic };
    let report =
        evaluate_step_ffs_detailed(&s.model, sr, &fdata.domains, &fdata.step_numbers, fdata.domains_truncated, &req).map_err(|e| match e.path {
            Some(path) => EngineError::invalid(path, e.message),
            None => EngineError::Params(e.message),
        })?;

    let mut out = serde_json::to_value(&report).map_err(|e| EngineError::Params(format!("failed to serialize FFS report: {e}")))?;

    if let Some(raw_path) = csv_path {
        let checked = s.checked_path(&raw_path, Access::Write)?;
        let display = s.units().clone();
        let elastic_override = req.fad.as_ref().and_then(|f| {
            let e = f.young.or(report.defaults.young)?;
            let nu = f.poisson.or(report.defaults.poisson)?;
            (e.is_finite() && e > 0.0 && nu.is_finite() && nu > -1.0 && nu < 0.5).then_some((e, nu))
        });
        let (all_profiles, _, _, _) = build_station_profiles(&s.model, &fdata.domains, req.ring_exclusion, elastic_override);
        let table = ffs_csv_table(&report, &all_profiles, &display)?;
        fieldcraft_io::fs::write_output(std::path::Path::new(&checked), table.csv().as_bytes())?;
        out["path"] = json!(checked);
        out["rows"] = json!(table.rows.len());
        out["columns"] = json!(table.header);
    }

    Ok(out)
}

fn resolve_fracture_step(s: &Session, requested: Option<&str>) -> Result<String> {
    if let Some(name) = requested {
        if s.fracture_data(name).is_some_and(|d| !d.domains.is_empty()) {
            return Ok(name.to_string());
        }
        if s.fracture_results.contains_key(name) {
            return Err(EngineError::Params(format!(
                "the fracture result of step `{name}` is stale (the model changed since it was solved); solve it again"
            )));
        }
        if s.latest_result(name).is_some() {
            return Err(EngineError::Params(format!(
                "no WARP3D fracture results are stored for step `{name}`; solve or import a WARP3D model with a crack_front first"
            )));
        }
        return Err(EngineError::Params(format!(
            "no WARP3D fracture results are stored for unknown step `{name}`; solve or import a WARP3D model with a crack_front first"
        )));
    }
    if let Some(v) = &s.results_view
        && s.fracture_data(&v.step).is_some_and(|d| !d.domains.is_empty())
        && s.result(&v.step).is_some()
    {
        return Ok(v.step.clone());
    }
    let steps = s.unpersisted_fracture_steps();
    match steps.as_slice() {
        [only] => Ok((*only).to_string()),
        [] => {
            if !s.fracture_results.is_empty() {
                Err(EngineError::Params("WARP3D fracture results are stale (the model changed since they were solved); solve again".into()))
            } else {
                Err(EngineError::Params("no WARP3D fracture results are stored; solve or import a WARP3D model with a crack_front first".into()))
            }
        }
        _ => Err(EngineError::Params("multiple steps have WARP3D fracture results; specify `step`".into())),
    }
}

fn parse_ring_exclusion(v: Option<&Value>) -> Result<RingExclusion> {
    match v {
        None | Some(Value::Null) => Ok(RingExclusion::Auto),
        Some(Value::String(s)) => match s.as_str() {
            "auto" => Ok(RingExclusion::Auto),
            "0" => Ok(RingExclusion::Count { count: 0 }),
            "1" => Ok(RingExclusion::Count { count: 1 }),
            "2" => Ok(RingExclusion::Count { count: 2 }),
            "3" => Ok(RingExclusion::Count { count: 3 }),
            "4" => Ok(RingExclusion::Count { count: 4 }),
            _ => Err(EngineError::param("exclude_inner_rings", ProblemKind::NotAChoice)),
        },
        Some(_) => Err(EngineError::param("exclude_inner_rings", ProblemKind::NotAChoice)),
    }
}

fn opt_str<'a>(obj: &'a Map<String, Value>, key: &str, path: &str) -> Result<Option<&'a str>> {
    match obj.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) => Ok(Some(s.as_str())),
        Some(_) => Err(EngineError::param(path, ProblemKind::NotText)),
    }
}

fn req_f64(obj: &Map<String, Value>, key: &str, path: &str) -> Result<f64> {
    match obj.get(key) {
        None | Some(Value::Null) => Err(EngineError::param(path, ProblemKind::Required)),
        Some(v) => v.as_f64().filter(|x| x.is_finite()).ok_or_else(|| EngineError::param(path, ProblemKind::NotANumber)),
    }
}

fn opt_f64(obj: &Map<String, Value>, key: &str, path: &str) -> Result<Option<f64>> {
    match obj.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => v.as_f64().filter(|x| x.is_finite()).map(Some).ok_or_else(|| EngineError::param(path, ProblemKind::NotANumber)),
    }
}

fn check_gt(val: f64, lo: f64, path: &str) -> Result<f64> {
    if val <= lo {
        return Err(EngineError::param(path, ProblemKind::NotAbove(lo)));
    }
    Ok(val)
}

fn check_ge(val: f64, lo: f64, path: &str) -> Result<f64> {
    if val < lo {
        return Err(EngineError::param(path, ProblemKind::Below(lo)));
    }
    Ok(val)
}

fn parse_fad_config(v: Option<&Value>) -> Result<Option<FadConfig>> {
    let Some(val) = v else {
        return Ok(None);
    };
    if val.is_null() {
        return Ok(None);
    }
    let Some(fobj) = val.as_object() else {
        return Err(EngineError::param("fad", ProblemKind::NotAnObject));
    };

    for key in fobj.keys() {
        if !ALLOWED_FAD_KEYS.contains(&key.as_str()) {
            return Err(EngineError::invalid("fad", format!("unknown parameter `{key}` inside `fad`")));
        }
    }

    let option = match opt_str(fobj, "option", "fad/option")? {
        None | Some("option1") => FadOption::Option1,
        Some("option2") => FadOption::Option2,
        Some(_) => return Err(EngineError::param("fad/option", ProblemKind::NotAChoice)),
    };

    let option2_source_str = opt_str(fobj, "option2_source", "fad/option2_source")?.unwrap_or("model_material");
    let option2_curve = match option2_source_str {
        "model_material" => {
            if let Some(alpha) = opt_f64(fobj, "ramberg_alpha", "fad/ramberg_alpha")? {
                check_gt(alpha, 0.0, "fad/ramberg_alpha")?;
            }
            if let Some(n) = opt_f64(fobj, "ramberg_n", "fad/ramberg_n")? {
                check_gt(n, 1.0, "fad/ramberg_n")?;
            }
            Option2Curve::ModelMaterial
        }
        "ramberg_osgood" => {
            let alpha = check_gt(req_f64(fobj, "ramberg_alpha", "fad/ramberg_alpha")?, 0.0, "fad/ramberg_alpha")?;
            let n = check_gt(req_f64(fobj, "ramberg_n", "fad/ramberg_n")?, 1.0, "fad/ramberg_n")?;
            Option2Curve::RambergOsgood { alpha, n }
        }
        _ => return Err(EngineError::param("fad/option2_source", ProblemKind::NotAChoice)),
    };

    let lr_mode_str = opt_str(fobj, "lr_mode", "fad/lr_mode")?.unwrap_or("reference_stress");
    let lr_input = match lr_mode_str {
        "reference_stress" => {
            let sigma_ref_full = check_ge(req_f64(fobj, "sigma_ref_full", "fad/sigma_ref_full")?, 0.0, "fad/sigma_ref_full")?;
            LrInput::ReferenceStress { sigma_ref_full }
        }
        "direct_load_ratio" => {
            let l_r_full = check_ge(req_f64(fobj, "l_r_full", "fad/l_r_full")?, 0.0, "fad/l_r_full")?;
            LrInput::DirectLoadRatio { l_r_full }
        }
        "reaction_ratio" => {
            let bc_set = match fobj.get("bc_set") {
                None | Some(Value::Null) => return Err(EngineError::param("fad/bc_set", ProblemKind::Required)),
                Some(Value::String(s)) if s.trim().is_empty() => return Err(EngineError::param("fad/bc_set", ProblemKind::Required)),
                Some(Value::String(s)) => s.trim().to_string(),
                Some(_) => return Err(EngineError::param("fad/bc_set", ProblemKind::NotText)),
            };
            let component = match opt_str(fobj, "reaction_component", "fad/reaction_component")? {
                None | Some("magnitude") => ReactionComponent::Magnitude,
                Some("x") => ReactionComponent::X,
                Some("y") => ReactionComponent::Y,
                Some("z") => ReactionComponent::Z,
                Some(_) => return Err(EngineError::param("fad/reaction_component", ProblemKind::NotAChoice)),
            };
            let limit_load = check_gt(req_f64(fobj, "limit_load", "fad/limit_load")?, 0.0, "fad/limit_load")?;
            LrInput::ReactionRatio { bc_set, component, limit_load }
        }
        _ => return Err(EngineError::param("fad/lr_mode", ProblemKind::NotAChoice)),
    };

    let toughness_mode_str = opt_str(fobj, "toughness_mode", "fad/toughness_mode")?.unwrap_or("k_mat");
    let toughness = match toughness_mode_str {
        "k_mat" => {
            let value = check_gt(req_f64(fobj, "k_mat", "fad/k_mat")?, 0.0, "fad/k_mat")?;
            ToughnessInput::KMat { value }
        }
        "j_ic" => {
            let value = check_gt(req_f64(fobj, "j_ic", "fad/j_ic")?, 0.0, "fad/j_ic")?;
            let plane_strain = match fobj.get("j_ic_plane_strain") {
                None | Some(Value::Null) => true,
                Some(Value::Bool(b)) => *b,
                Some(_) => return Err(EngineError::param("fad/j_ic_plane_strain", ProblemKind::NotABool)),
            };
            ToughnessInput::JIc { value, plane_strain }
        }
        _ => return Err(EngineError::param("fad/toughness_mode", ProblemKind::NotAChoice)),
    };

    let kr_source = match opt_str(fobj, "kr_source", "fad/kr_source")? {
        None | Some("auto") => KrSource::Auto,
        Some("elastic_ki") => KrSource::ElasticKi,
        Some("linear_scaled_step1_ki") => KrSource::LinearScaledStep1Ki,
        Some("kj_plane_strain") => KrSource::KjPlaneStrain,
        Some("kj_plane_stress") => KrSource::KjPlaneStress,
        Some(_) => return Err(EngineError::param("fad/kr_source", ProblemKind::NotAChoice)),
    };

    let station_selection = match opt_str(fobj, "station_mode", "fad/station_mode")? {
        None | Some("governing") => StationSelection::Governing,
        Some("domain_id") => {
            let domain_id = match fobj.get("domain_id") {
                None | Some(Value::Null) => return Err(EngineError::param("fad/domain_id", ProblemKind::Required)),
                Some(Value::String(s)) if s.trim().is_empty() => return Err(EngineError::param("fad/domain_id", ProblemKind::Required)),
                Some(Value::String(s)) => s.trim().to_string(),
                Some(_) => return Err(EngineError::param("fad/domain_id", ProblemKind::NotText)),
            };
            StationSelection::DomainId { domain_id }
        }
        Some(_) => return Err(EngineError::param("fad/station_mode", ProblemKind::NotAChoice)),
    };

    let sigma_ys = opt_f64(fobj, "sigma_ys", "fad/sigma_ys")?.map(|v| check_gt(v, 0.0, "fad/sigma_ys")).transpose()?;
    let sigma_uts = opt_f64(fobj, "sigma_uts", "fad/sigma_uts")?.map(|v| check_gt(v, 0.0, "fad/sigma_uts")).transpose()?;
    let l_r_max = opt_f64(fobj, "l_r_max", "fad/l_r_max")?.map(|v| check_ge(v, 1.0, "fad/l_r_max")).transpose()?;
    let young = opt_f64(fobj, "young", "fad/young")?.map(|v| check_gt(v, 0.0, "fad/young")).transpose()?;
    let poisson = match opt_f64(fobj, "poisson", "fad/poisson")? {
        None => None,
        Some(nu) if nu <= -1.0 => return Err(EngineError::param("fad/poisson", ProblemKind::NotAbove(-1.0))),
        Some(nu) if nu >= 0.5 => return Err(EngineError::invalid("fad/poisson", "Poisson's ratio must be strictly between -1.0 and 0.5")),
        Some(nu) => Some(nu),
    };

    Ok(Some(FadConfig { option, option2_curve, lr_input, toughness, kr_source, station_selection, sigma_ys, sigma_uts, l_r_max, young, poisson }))
}

fn spread_status_str(s: SpreadStatus) -> &'static str {
    match s {
        SpreadStatus::Good => "good",
        SpreadStatus::Caution => "caution",
        SpreadStatus::Poor => "poor",
        SpreadStatus::Unverified => "unverified",
    }
}

fn limiting_mode_str(m: LimitingMode) -> &'static str {
    match m {
        LimitingMode::Unloaded => "unloaded",
        LimitingMode::Fracture => "fracture",
        LimitingMode::ElasticPlastic => "elastic_plastic",
        LimitingMode::PlasticCollapse => "plastic_collapse",
    }
}

fn ffs_csv_table(report: &FfsReport, all_profiles: &[StationProfile], display: &fieldcraft_model::DisplaySystem) -> Result<Table> {
    let len_unit = display.symbol(Quantity::Length);
    let j_unit = display.symbol(Quantity::ForcePerLength);
    let p_unit = display.symbol(Quantity::Pressure);
    let (k_scale, k_unit) = crate::units::stress_sqrt_length_scale(crate::units::si(), display);

    let header = vec![
        "record_type".to_string(),
        "step".to_string(),
        "increment".to_string(),
        "warp_step".to_string(),
        "load_factor".to_string(),
        "domain_id".to_string(),
        "front_nodes".to_string(),
        "focus_node".to_string(),
        titled("s_length", Some(len_unit)),
        "s_norm".to_string(),
        "kept_rings".to_string(),
        "total_rings".to_string(),
        titled("j_mean", Some(j_unit)),
        titled("j_min", Some(j_unit)),
        titled("j_max", Some(j_unit)),
        "ring_spread".to_string(),
        "spread_status".to_string(),
        titled("k_i_mean", Some(&k_unit)),
        titled("k_j_plane_strain", Some(&k_unit)),
        titled("k_j_plane_stress", Some(&k_unit)),
        titled("t_11_mean", Some(p_unit)),
        "l_r".to_string(),
        "k_r".to_string(),
        "k_r_allow".to_string(),
        titled("k_eff", Some(&k_unit)),
        "reserve_factor".to_string(),
        "safety_margin".to_string(),
        "acceptable".to_string(),
        "limiting_mode".to_string(),
        "epfm_k_j_ratio".to_string(),
        "epfm_j_ratio".to_string(),
    ];

    let express = |q: Quantity, v: f64| -> Result<Cell> {
        if !v.is_finite() {
            return Ok(Cell::Empty);
        }
        let shown = display.express(q, v).map_err(|e| EngineError::Params(format!("unit conversion error: {e}")))?;
        Ok(if shown.is_finite() { Cell::Number(shown) } else { Cell::Empty })
    };
    let opt_express = |q: Quantity, v: Option<f64>| -> Result<Cell> {
        match v {
            Some(x) => express(q, x),
            None => Ok(Cell::Empty),
        }
    };
    let opt_num = |v: Option<f64>| -> Cell {
        match v {
            Some(x) if x.is_finite() => Cell::Number(x),
            _ => Cell::Empty,
        }
    };
    let req_num = |v: f64| -> Cell { if v.is_finite() { Cell::Number(v) } else { Cell::Empty } };
    let opt_k = |v: Option<f64>| -> Cell {
        match v.map(|x| x * k_scale) {
            Some(x) if x.is_finite() => Cell::Number(x),
            _ => Cell::Empty,
        }
    };

    let push_row = |rows: &mut Vec<Vec<Cell>>,
                    record_type: &str,
                    increment: usize,
                    warp_step: usize,
                    load_factor: f64,
                    domain_id: &str,
                    prof: Option<&StationProfile>,
                    pt: Option<&FadPoint>|
     -> Result<()> {
        let front_nodes = prof
            .map(|pr| pr.front_nodes.iter().map(u32::to_string).collect::<Vec<_>>().join(" "))
            .filter(|s| !s.is_empty())
            .map_or(Cell::Empty, Cell::Text);
        let focus_node = prof.and_then(|pr| pr.focus_node).map_or(Cell::Empty, |n| Cell::Number(f64::from(n)));
        let s_length = prof.map_or(Ok(Cell::Empty), |pr| opt_express(Quantity::Length, pr.s_length))?;
        let s_norm = prof.map_or(Cell::Empty, |pr| req_num(pr.s_norm));
        let kept_rings = prof.map_or(Cell::Empty, |pr| Cell::Number(pr.kept_rings as f64));
        let total_rings = prof.map_or(Cell::Empty, |pr| Cell::Number(pr.total_rings as f64));
        let j_mean = prof.map_or(Ok(Cell::Empty), |pr| express(Quantity::ForcePerLength, pr.j_mean))?;
        let j_min = prof.map_or(Ok(Cell::Empty), |pr| express(Quantity::ForcePerLength, pr.j_min))?;
        let j_max = prof.map_or(Ok(Cell::Empty), |pr| express(Quantity::ForcePerLength, pr.j_max))?;
        let ring_spread = prof.map_or(Cell::Empty, |pr| req_num(pr.ring_spread));
        let spread_status = prof.map_or(Cell::Empty, |pr| Cell::Text(spread_status_str(pr.spread_status).to_string()));
        let k_i_mean = prof.map_or(Cell::Empty, |pr| opt_k(pr.k_i_mean));
        let k_j_strain = prof.map_or(Cell::Empty, |pr| opt_k(pr.k_j_plane_strain));
        let k_j_stress = prof.map_or(Cell::Empty, |pr| opt_k(pr.k_j_plane_stress));
        let t_11_mean = prof.map_or(Ok(Cell::Empty), |pr| opt_express(Quantity::Pressure, pr.t_11_mean))?;

        let (l_r, k_r, k_r_allow, k_eff, rf, margin, acceptable, limiting_mode, epfm_kj, epfm_j) = if let Some(fp) = pt {
            (
                req_num(fp.l_r),
                req_num(fp.k_r),
                req_num(fp.k_r_allow),
                opt_k(fp.k_eff),
                opt_num(fp.reserve_factor),
                opt_num(fp.safety_margin),
                Cell::Text(fp.acceptable.to_string()),
                Cell::Text(limiting_mode_str(fp.limiting_mode).to_string()),
                opt_num(fp.epfm_k_j_ratio),
                opt_num(fp.epfm_j_ratio),
            )
        } else {
            (Cell::Empty, Cell::Empty, Cell::Empty, Cell::Empty, Cell::Empty, Cell::Empty, Cell::Empty, Cell::Empty, Cell::Empty, Cell::Empty)
        };

        rows.push(vec![
            Cell::Text(record_type.to_string()),
            Cell::Text(report.step.clone()),
            Cell::Number(increment as f64),
            Cell::Number(warp_step as f64),
            req_num(load_factor),
            Cell::Text(domain_id.to_string()),
            front_nodes,
            focus_node,
            s_length,
            s_norm,
            kept_rings,
            total_rings,
            j_mean,
            j_min,
            j_max,
            ring_spread,
            spread_status,
            k_i_mean,
            k_j_strain,
            k_j_stress,
            t_11_mean,
            l_r,
            k_r,
            k_r_allow,
            k_eff,
            rf,
            margin,
            acceptable,
            limiting_mode,
            epfm_kj,
            epfm_j,
        ]);
        Ok(())
    };

    let mut profile_by_key: BTreeMap<(usize, &str), &StationProfile> = BTreeMap::new();
    for p in all_profiles {
        profile_by_key.insert((p.warp_step, p.domain_id.as_str()), p);
    }

    let trajectory_len = report.fad.as_ref().map_or(0, |f| f.trajectory.len());
    let mut rows = Vec::with_capacity(trajectory_len + report.active_profiles.len());

    if let Some(fad) = &report.fad {
        for pt in &fad.trajectory {
            let prof = profile_by_key.get(&(pt.warp_step, pt.domain_id.as_str())).copied();
            push_row(&mut rows, "trajectory", pt.increment, pt.warp_step, pt.load_factor, &pt.domain_id, prof, Some(pt))?;
        }
        let station_pts: BTreeMap<&str, &FadPoint> = fad.active_stations.iter().map(|pt| (pt.domain_id.as_str(), pt)).collect();
        for prof in &report.active_profiles {
            let pt = station_pts.get(prof.domain_id.as_str()).copied();
            let inc =
                pt.map_or_else(|| if report.is_nonlinear { report.active_frame.unwrap_or(report.active_warp_step) } else { 1 }, |p| p.increment);
            let lf = pt.map_or(report.active_load_factor, |p| p.load_factor);
            push_row(&mut rows, "station", inc, prof.warp_step, lf, &prof.domain_id, Some(prof), pt)?;
        }
    } else {
        let default_inc = if report.is_nonlinear { report.active_frame.unwrap_or(report.active_warp_step) } else { 1 };
        for prof in &report.active_profiles {
            push_row(&mut rows, "station", default_inc, prof.warp_step, report.active_load_factor, &prof.domain_id, Some(prof), None)?;
        }
    }

    Ok(Table { header, rows })
}

#[cfg(test)]
mod tests {
    use fieldcraft_model::units::{DisplaySystem, Preset, Quantity};
    use fieldcraft_solver::{StaticResult, nonlinear};
    use fieldcraft_warp3d::{DomainIntegralResult, RingIntegral};
    use serde_json::json;

    use crate::ffs::{RingExclusion, material_defaults, reaction_bc_sets, summarize_fracture_spread};
    use crate::schema::{field_at, prefill};
    use crate::units::stress_sqrt_length_scale;
    use crate::{EngineError, Session, StepResult};

    fn seeded_fracture_session(nonlinear_steps: bool) -> Session {
        let mut s = Session::default();
        s.run_script(&json!([
            {"command": "model.units", "params": {"system": "m-kg-s-N-Pa"}},
            {"command": "mesh.node.add", "params": {"label": 1, "xyz": [0.0, 0.0, 0.0]}},
            {"command": "mesh.node.add", "params": {"label": 2, "xyz": [0.1, 0.0, 0.0]}},
            {"command": "mesh.node.add", "params": {"label": 3, "xyz": [0.1, 0.1, 0.0]}},
            {"command": "mesh.node.add", "params": {"label": 4, "xyz": [0.0, 0.1, 0.0]}},
            {"command": "mesh.node.add", "params": {"label": 5, "xyz": [0.0, 0.0, 0.1]}},
            {"command": "mesh.node.add", "params": {"label": 6, "xyz": [0.1, 0.0, 0.1]}},
            {"command": "mesh.node.add", "params": {"label": 7, "xyz": [0.1, 0.1, 0.1]}},
            {"command": "mesh.node.add", "params": {"label": 8, "xyz": [0.0, 0.1, 0.1]}},
            {"command": "mesh.element.add", "params": {"label": 1, "kind": "hex8", "nodes": [1, 2, 3, 4, 5, 6, 7, 8]}},
            {"command": "model.set.create", "params": {"name": "all", "elements": [1]}},
            {"command": "model.set.create", "params": {"name": "clamp", "nodes": [1, 2, 3, 4]}},
            {"command": "model.set.create", "params": {"name": "load_face", "nodes": [5, 6, 7, 8]}},
            {"command": "model.set.create", "params": {"name": "front", "nodes": [1, 2]}},
            {"command": "model.material.create", "params": {"name": "Steel", "elastic": {"type": "isotropic", "young": 200.0e9, "poisson": 0.3}}},
            {"command": "model.material.plasticity", "params": {"name": "Steel", "plastic": {"type": "bilinear", "yield_stress": 400.0e6, "hardening_modulus": 2.0e9, "hardening": "isotropic"}}},
            {"command": "model.section.create", "params": {"name": "solid", "set": "all", "material": "Steel", "kind": {"type": "solid"}}},
            {"command": "model.boundary_condition.create", "params": {"name": "fix", "type": "fixed", "target": {"set": "clamp"}}},
            {"command": "model.load.create", "params": {"name": "pull", "type": "force", "target": {"set": "load_face"}, "vector": [0.0, 0.0, 100.0e3]}},
            {"command": "model.step.create", "params": {"name": "Load", "procedure": if nonlinear_steps { json!({"type": "nonlinear_static", "nonlinear_geometry": false, "max_increments": 2}) } else { json!({"type": "static"}) }, "boundary_conditions": ["fix"], "loads": ["pull"]}}
        ]))
        .unwrap();

        let nodes = vec![1, 2, 3, 4, 5, 6, 7, 8];
        let mut sr = StaticResult {
            step: "Load".into(),
            nodes,
            displacement: vec![[0.0; 3]; 8],
            rotation: vec![[0.0; 3]; 8],
            reaction: vec![[0.0, 0.0, -25.0e3]; 4].into_iter().chain(vec![[0.0; 3]; 4]).collect(),
            reaction_moment: vec![],
            stress: vec![],
            von_mises: vec![],
            elements: vec![],
            shells: vec![],
            beams: vec![],
            reference_points: vec![],
            applied_total: [0.0, 0.0, 100.0e3],
            reaction_total: [0.0, 0.0, -100.0e3],
            stats: Default::default(),
            warnings: vec![],
            temperature: None,
            nonlinear: None,
            contacts: vec![],
            table_ranges: vec![],
        };

        let mut domains = Vec::new();
        let make_dom = |domain_id: &str, step: usize, node: u32, j_base: f64, k_base: f64| DomainIntegralResult {
            domain_id: domain_id.into(),
            step,
            front_nodes: vec![node],
            s_position: None,
            normal: [0.0, 1.0, 0.0],
            tangent: [1.0, 0.0, 0.0],
            advance: [0.0, 0.0, 1.0],
            area_aq: 1.0e-4,
            rings: vec![
                RingIntegral {
                    ring: 1,
                    j: j_base * 0.85,
                    k_i: Some(k_base * 0.85),
                    k_ii: Some(0.0),
                    k_iii: Some(0.0),
                    t_11: Some(-10.0e6),
                    t_33: Some(-3.0e6),
                },
                RingIntegral { ring: 2, j: j_base, k_i: Some(k_base), k_ii: Some(0.0), k_iii: Some(0.0), t_11: Some(-12.0e6), t_33: Some(-4.0e6) },
                RingIntegral {
                    ring: 3,
                    j: j_base * 1.02,
                    k_i: Some(k_base * 1.02),
                    k_ii: Some(0.0),
                    k_iii: Some(0.0),
                    t_11: Some(-12.2e6),
                    t_33: Some(-4.1e6),
                },
            ],
            j_mean: j_base,
            j_min: j_base * 0.85,
            j_max: j_base * 1.02,
            ring_spread: 0.17,
            j_total: None,
            k_j_plane_strain: Some(k_base * 1.02),
            k_j_plane_stress: Some(k_base),
            k_i_mean: Some(k_base),
            k_ii_mean: Some(0.0),
            k_iii_mean: Some(0.0),
            t_11_mean: Some(-12.0e6),
            t_33_mean: Some(-4.0e6),
            interaction_stress_state: None,
        };

        let step_numbers = if nonlinear_steps {
            let f0 = nonlinear::Frame { load_factor: 0.0, displacement: vec![[0.0; 3]; 8], reaction: vec![], stress: vec![], contacts: vec![] };
            let f1 = nonlinear::Frame {
                load_factor: 0.5,
                displacement: vec![[0.0; 3]; 8],
                reaction: vec![(0, [0.0, 0.0, -12.5e3]), (1, [0.0, 0.0, -12.5e3]), (2, [0.0, 0.0, -12.5e3]), (3, [0.0, 0.0, -12.5e3])],
                stress: vec![],
                contacts: vec![],
            };
            let f2 = nonlinear::Frame {
                load_factor: 1.0,
                displacement: vec![[0.0; 3]; 8],
                reaction: vec![(0, [0.0, 0.0, -25.0e3]), (1, [0.0, 0.0, -25.0e3]), (2, [0.0, 0.0, -25.0e3]), (3, [0.0, 0.0, -25.0e3])],
                stress: vec![],
                contacts: vec![],
            };
            sr.nonlinear = Some(nonlinear::History {
                frames: vec![f0, f1, f2],
                increments: vec![
                    nonlinear::Increment {
                        number: 1,
                        load_factor: 0.5,
                        size: 0.5,
                        iterations: vec![],
                        converged: true,
                        cutback: None,
                        negative_pivots: 0,
                        augmentations: 0,
                    },
                    nonlinear::Increment {
                        number: 2,
                        load_factor: 1.0,
                        size: 0.5,
                        iterations: vec![],
                        converged: true,
                        cutback: None,
                        negative_pivots: 0,
                        augmentations: 0,
                    },
                ],
                iterations: 5,
                critical_points: vec![],
            });
            domains.push(make_dom("cf_0001", 1, 1, 5_000.0, 30.0e6));
            domains.push(make_dom("cf_0002", 1, 2, 6_000.0, 35.0e6));
            domains.push(make_dom("cf_0001", 2, 1, 15_000.0, 50.0e6));
            domains.push(make_dom("cf_0002", 2, 2, 22_000.0, 62.0e6));
            vec![1, 2]
        } else {
            domains.push(make_dom("cf_0001", 1, 1, 15_000.0, 50.0e6));
            domains.push(make_dom("cf_0002", 1, 2, 22_000.0, 62.0e6));
            vec![1]
        };

        s.store_result(StepResult::Implicit(fieldcraft_solver::StepResult::Static(sr)));
        s.store_fracture_results("Load".into(), domains, step_numbers, false, None, Some("front"));
        s
    }

    #[test]
    fn results_ffs_hostile_params_resolve_via_field_at() {
        let mut s = seeded_fracture_session(false);
        let form = crate::cmd::forms::RESULTS_FFS;
        let base = prefill(form, &json!({}));
        let hostile = [json!("x"), json!(-1), json!(0), json!(0.5), json!(1e300), json!([]), json!({}), json!(null), json!(true)];

        let mut checked_param_errors = 0usize;
        let mut check_call = |payload: serde_json::Value| {
            if let Err(EngineError::Param(e)) = s.execute("results.ffs", &payload) {
                checked_param_errors += 1;
                assert!(e.field.is_some(), "missing field handle for `{}` on {payload}", e.path);
                assert!(field_at(form, &e.path).is_some(), "path `{}` not found by field_at for {payload}", e.path);
            }
        };

        // Top-level non-path fields.
        for key in ["step", "frame", "exclude_inner_rings", "fad"] {
            for v in &hostile {
                let mut p = base.clone();
                p[key] = v.clone();
                check_call(p);
            }
        }
        check_call(json!({"step": ""}));
        check_call(json!({"path": ""}));

        // All nested `fad/*` subfields.
        let valid_fad = json!({
            "option": "option2",
            "option2_source": "ramberg_osgood",
            "ramberg_alpha": 1.0,
            "ramberg_n": 5.0,
            "lr_mode": "reference_stress",
            "sigma_ref_full": 150.0e6,
            "toughness_mode": "k_mat",
            "k_mat": 100.0e6,
            "kr_source": "auto",
            "station_mode": "governing",
            "sigma_ys": 400.0e6,
            "sigma_uts": 550.0e6,
            "l_r_max": 1.18,
            "young": 200.0e9,
            "poisson": 0.3
        });
        let nested_keys = [
            "option",
            "option2_source",
            "ramberg_alpha",
            "ramberg_n",
            "lr_mode",
            "sigma_ref_full",
            "l_r_full",
            "bc_set",
            "reaction_component",
            "limit_load",
            "toughness_mode",
            "k_mat",
            "j_ic",
            "j_ic_plane_strain",
            "kr_source",
            "station_mode",
            "domain_id",
            "sigma_ys",
            "sigma_uts",
            "l_r_max",
            "young",
            "poisson",
            "unknown_key",
        ];
        for key in nested_keys {
            for v in &hostile {
                let mut fad = valid_fad.clone();
                fad[key] = v.clone();
                check_call(json!({"fad": fad}));
            }
        }
        // Cross-field missing/invalid subfields must attribute to exact `fad/<subkey>`.
        for (fad, expected_path) in [
            (json!({"lr_mode": "direct_load_ratio", "toughness_mode": "k_mat", "k_mat": 100.0e6}), "fad/l_r_full"),
            (json!({"lr_mode": "reaction_ratio", "limit_load": 100.0e3, "toughness_mode": "k_mat", "k_mat": 100.0e6}), "fad/bc_set"),
            (json!({"lr_mode": "reaction_ratio", "bc_set": "clamp", "toughness_mode": "k_mat", "k_mat": 100.0e6}), "fad/limit_load"),
            (json!({"lr_mode": "direct_load_ratio", "l_r_full": 0.5, "toughness_mode": "j_ic"}), "fad/j_ic"),
            (
                json!({"option": "option2", "option2_source": "ramberg_osgood", "lr_mode": "direct_load_ratio", "l_r_full": 0.5, "toughness_mode": "k_mat", "k_mat": 100.0e6}),
                "fad/ramberg_alpha",
            ),
            (
                json!({"lr_mode": "direct_load_ratio", "l_r_full": 0.5, "toughness_mode": "k_mat", "k_mat": 100.0e6, "station_mode": "domain_id"}),
                "fad/domain_id",
            ),
            (json!({"lr_mode": "direct_load_ratio", "l_r_full": 0.5, "toughness_mode": "k_mat", "k_mat": 100.0e6, "poisson": 0.5}), "fad/poisson"),
            (
                json!({"lr_mode": "direct_load_ratio", "l_r_full": 0.5, "toughness_mode": "k_mat", "k_mat": 100.0e6, "sigma_ys": 500.0e6, "sigma_uts": 400.0e6}),
                "fad/sigma_uts",
            ),
        ] {
            let err = s.execute("results.ffs", &json!({"fad": fad})).unwrap_err();
            assert!(matches!(err, EngineError::Param(ref p) if p.path == expected_path), "expected `{expected_path}`, got {err:?}");
            checked_param_errors += 1;
        }
        assert!(checked_param_errors > 100);
    }

    #[test]
    fn results_ffs_json_unit_conversion_and_csv_export_across_presets() {
        let mut s = seeded_fracture_session(true);
        let dir = std::env::temp_dir().join(format!("fieldcraft-ffs-cmd-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();

        for preset in Preset::ALL {
            let sys_name = preset.label();
            s.execute("model.units", &json!({"system": sys_name})).unwrap();
            let display = DisplaySystem::new(preset);
            let si = DisplaySystem::new(Preset::MKgS);
            let (k_scale, k_unit) = stress_sqrt_length_scale(&si, &display);

            let csv_path = dir.join(format!("ffs_{sys_name}.csv"));
            let k_mat_display = 100.0e6 * k_scale;
            let limit_load_display = display.express(Quantity::Force, 200.0e3).unwrap();

            let reply = s
                .execute(
                    "results.ffs",
                    &json!({
                        "step": "Load",
                        "exclude_inner_rings": "auto",
                        "fad": {
                            "option": "option2",
                            "option2_source": "model_material",
                            "lr_mode": "reaction_ratio",
                            "bc_set": "clamp",
                            "reaction_component": "z",
                            "limit_load": limit_load_display,
                            "toughness_mode": "k_mat",
                            "k_mat": k_mat_display,
                            "kr_source": "auto",
                            "station_mode": "governing"
                        },
                        "path": csv_path
                    }),
                )
                .unwrap();

            // Reply values converted cleanly to active display units.
            assert_eq!(reply["units"]["stress_intensity"], k_unit);
            assert_eq!(reply["units"]["force"], display.symbol(Quantity::Force));
            assert_eq!(reply["units"]["pressure"], display.symbol(Quantity::Pressure));
            assert_eq!(reply["units"]["length"], display.symbol(Quantity::Length));
            assert_eq!(reply["units"]["force_per_length"], display.symbol(Quantity::ForcePerLength));

            let got_k_mat = reply["fad"]["k_mat"].as_f64().unwrap();
            assert!((got_k_mat - k_mat_display).abs() <= 1e-6 * k_mat_display.abs().max(1.0));
            assert_eq!(reply["fad"]["trajectory"].as_array().unwrap().len(), 2); // increments 1, 2
            assert_eq!(reply["fad"]["active_stations"].as_array().unwrap().len(), 2);
            assert_eq!(reply["rows"], 4); // 2 trajectory + 2 active stations

            // CSV headers and unit-converted values match display system.
            let csv = std::fs::read_to_string(&csv_path).unwrap();
            let mut lines = csv.lines();
            let header = lines.next().unwrap();
            assert!(header.starts_with("record_type,step,increment,warp_step,load_factor,domain_id,front_nodes,focus_node,"), "{header}");
            assert!(header.contains(&format!("s_length [{}]", display.symbol(Quantity::Length))), "{header}");
            assert!(header.contains(&format!("j_mean [{}]", display.symbol(Quantity::ForcePerLength))), "{header}");
            assert!(header.contains(&format!("k_i_mean [{k_unit}]")), "{header}");
            assert!(header.contains(&format!("k_eff [{k_unit}]")), "{header}");
            assert!(header.contains(&format!("t_11_mean [{}]", display.symbol(Quantity::Pressure))), "{header}");
            let data_rows: Vec<&str> = lines.collect();
            assert_eq!(data_rows.len(), 4);
            assert!(data_rows[0].starts_with("trajectory,Load,1,"));
            assert!(data_rows[1].starts_with("trajectory,Load,2,"));
            assert!(data_rows[2].starts_with("station,Load,2,"));
            assert!(data_rows[3].starts_with("station,Load,2,"));
        }

        // Underlying stored fracture data remains SI.
        let si_stored = s.fracture_results("Load").unwrap();
        assert!((si_stored[0].j_mean - 5_000.0).abs() < 1e-9);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn ffs_facade_and_crack_front_every_step_parsing() {
        let s = seeded_fracture_session(false);
        let defs = material_defaults(&s, "Load").unwrap();
        assert_eq!(defs.young, Some(200.0e9));
        assert_eq!(defs.poisson, Some(0.3));
        assert_eq!(defs.sigma_ys, Some(400.0e6));
        assert!(defs.has_plasticity);
        assert!(defs.option2_supported);

        let domains = s.fracture_results("Load").unwrap();
        let (profiles, spread, _, _) = summarize_fracture_spread(&s.model, domains, RingExclusion::Auto);
        assert_eq!(profiles.len(), 2);
        // Auto exclusion on 3 rings drops ring 1 (0.85), leaving rings 2..=3 (1.00..1.02) -> ~2% spread.
        assert!(spread < 0.05, "spread={spread}");

        let bc_sets = reaction_bc_sets(&s, "Load");
        assert_eq!(bc_sets, vec!["clamp".to_string()]);

        // crack_front.every_step parsing.
        let p_true = json!({"crack_front": {"set": "front", "every_step": true}});
        let parsed_true = crate::cmd::deck::parse_crack_front(&p_true).unwrap().unwrap();
        assert_eq!(parsed_true.every_step, Some(true));

        let p_omitted = json!({"crack_front": {"set": "front"}});
        let parsed_omitted = crate::cmd::deck::parse_crack_front(&p_omitted).unwrap().unwrap();
        assert_eq!(parsed_omitted.every_step, None);

        let p_bad = json!({"crack_front": {"set": "front", "every_step": "yes"}});
        let err = crate::cmd::deck::parse_crack_front(&p_bad).unwrap_err();
        assert!(matches!(err, EngineError::Param(ref pr) if pr.path == "crack_front/every_step"), "{err:?}");
    }
}

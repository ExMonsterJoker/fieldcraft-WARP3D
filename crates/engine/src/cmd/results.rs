//! `results.*`: what the results display shows (deformed shape, contours, mode shapes,
//! animation; a fluid step's slice, coloured obstacles and streamlines) and reaction sums. The display is session state ([`ResultsView`]): these
//! commands are not edits, so they are not journalled or undone.

use crate::schema::ProblemKind;
use serde_json::{Value, json};

use super::{opt, xyz};
use crate::flow::{Axis, FlowView, MAX_STREAMLINE_STEPS, MAX_STREAMLINES, SliceView, symmetry_position};
use crate::requirement::Requirement;
use crate::results::{Field, Kind, MAX_BANDS, MIN_BANDS, ResultsView};
use crate::{CommandKind, CommandSpec, EngineError, Result, Session};

pub const DEFORMED: CommandSpec = CommandSpec {
    id: "results.deformed",
    label: "Deformed Shape",
    params: r#"{"step": "Load", "show": true, "scale": "auto"} — shows a solved step's deformed shape (`show: false` the undeformed mesh); `scale` is a factor (0 draws the undeformed shape, so a sequence from 0 swings it out) or "auto" (the largest displacement drawn a tenth of the model's size); `step` defaults to the step shown or the first solved one"#,
    form: Some(super::forms::DEFORMED),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: deformed,
};

pub const CONTOUR: CommandSpec = CommandSpec {
    id: "results.contour",
    label: "Contours",
    params: r#"{"step": "Load", "field": "von_mises", "bands": 12, "range"?: {"min"?: 0, "max"?: 80} | "auto"} — contours of a nodal field: displacement, displacement_x/y/z, velocity (explicit and fluid steps), pressure (fluid steps), temperature (thermal frames), wall_heat_flux (signed face average on thermal fluid obstacles), von_mises, max/mid/min_principal, stress_xx/yy/zz/xy/xz/yz (stresses for static steps), contact_pressure, contact_gap, contact_status (0 open, 1 closed or stick, 2 slip), contact_shear, contact_slip (friction; nonlinear static steps with contact, at the contacts' secondary surfaces); "none" turns them off; 2 to 24 bands; `range` clamps the legend (a bound left out, null or "auto" follows the field; values above `max` are drawn in the over colour), kept until the field changes"#,
    form: Some(super::forms::CONTOUR),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: contour,
};

pub const MODES: CommandSpec = CommandSpec {
    id: "results.modes",
    label: "Mode Shapes",
    params: r#"{"step": "Modes", "mode": 1} — shows a mode shape of a solved frequency step (from 1) and lists every mode's frequency"#,
    form: Some(super::forms::MODES),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: modes,
};

pub const ANIMATE: CommandSpec = CommandSpec {
    id: "results.animate",
    label: "Animate",
    params: r#"{"on": true} — animates shape and contours: a mode swings through positive and negative amplitudes, a linear static result interpolates a proportional load from zero to full and back with a fixed legend; absolute temperatures are unchanged. Nonlinear steps play their converged increments, each with its own stress and contact fields; explicit or fluid steps play their recorded fields in order"#,
    form: Some(super::forms::ANIMATE),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: animate,
};

pub const FRAME: CommandSpec = CommandSpec {
    id: "results.frame",
    label: "Field Frame",
    params: r#"{"step": "Impact", "frame": 3, "time": 0.002} — shows a field frame of a solved explicit or fluid step, by index from 0 or the frame nearest a time (neither: the last), or of a nonlinear static step (one per converged increment, `time` its load factor). The shape and recorded contours follow the frame: a nonlinear frame's stresses and contact pressure, gap, status, shear and slip are those of its own converged increment (a frame keeps its stress only when the step's `output` requests stress, and frames past the step's field cap, 30 million values, keep displacements only, showing no stress or contact contours; the complete final frame always has the final fields)"#,
    form: Some(super::forms::FRAME),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: frame,
};

pub const REACTIONS: CommandSpec = CommandSpec {
    id: "results.reactions",
    label: "Reaction Sums",
    params: r#"{"step": "Load", "sets": ["root"], "about": [0, 0, 0], "frame": 3} — reaction force and moment (about the point) summed over each node set, and over the whole model; `sets` (none or empty) defaults to the sets the step's boundary conditions hold; `frame` (nonlinear static steps, from 0 as in results.frame; default the final state) reads the reactions of that converged increment and adds its `load_factor` (solve.history gives every frame's sums at once)"#,
    form: Some(super::forms::REACTIONS),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: reactions,
};

pub const EXPORT_VTU: CommandSpec = CommandSpec {
    id: "results.export.vtu",
    label: "Export VTU",
    params: r#"{"step": "Load", "path": "out.vtu" | "out.pvd", "mode": 1, "frames": "all|last", "encoding": "ascii|binary"} — writes a solved step as VTK XML unstructured grids for ParaView: the mesh (points in node label order, cells in element label order, arrays `node_label` and `element_label`), the step's nodal fields as point data and per-element fields as cell data. Static: displacement, rotation (with shells or beams), reaction, stress (6 components in VTK order XX, YY, ZZ, XY, YZ, XZ), von_mises, temperature (coupled steps); nonlinear static: displacement per frame, with stress and von_mises when the step's `output` requests stress, the final fields on the last; frequency: mode_shape, with mode, frequency and eigenvalue as field data; explicit: displacement and velocity per frame, cell stress, plastic_strain, damage, eroded; heat: temperature per frame, heat_flux on the last. Point array `has_result` is 0 at nodes the step has no values for (written as 0, as are values that are not finite). A `.vtu` path writes one state (the last frame, or `mode`, default 1); a `.pvd` path writes every frame or mode (`frames: "last"` only the last) as `out_0000.vtu`, … beside it and the collection listing each with its time (load factor, mode number). `encoding` ascii (default) or binary (inline base64). Fluid steps are not supported yet. Replies {files, bytes, points, cells}"#,
    form: Some(super::forms::EXPORT_VTU),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: export_vtu,
};

pub const EXPORT_CSV: CommandSpec = CommandSpec {
    id: "results.export.csv",
    label: "Export Results (CSV)",
    params: r#"{"what": "probe" | "reactions" | "fracture", "path": "tip.csv", "step": "Load", …} — writes probe, reaction, or WARP3D fracture domain-integral results as a CSV file (RFC 4180, a header line, numbers in shortest round-trip form); the other parameters are solve.probe's (`nodes`, `at` or `set`, `mode`, `frame`) or results.reactions' (`sets`, `about`). Probe: a row per node with node, x, y, z and the quantities the step has; reactions: a row per set, a total row and an applied row; fracture: a row per crack-front domain with domain_id, step, s_position, front_nodes, area_aq, J, K, T-stress, and normal/tangent/advance directions in display units. Replies with the path, rows, columns, step and units"#,
    form: Some(super::forms::EXPORT_CSV),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: export_csv,
};

pub const SLICE: CommandSpec = CommandSpec {
    id: "results.slice",
    label: "Flow Slice",
    params: r#"{"step": "Flow", "field": "velocity" | "pressure" | "temperature", "axis": "x" | "y" | "z" | "none", "position": 20 | "auto", "frame": 10} — shows a fluid step's field (velocity magnitude, gauge pressure or temperature when its frames carry heat, banded with the legend as results.contour sets it) on a slice through the domain normal to `axis`, at `position` along it (inside the domain; "auto" or left out with a new axis: the symmetry plane along `axis` (its first cell layer, the low one when both faces are symmetry), else the domain's middle); "none" hides the slice; `frame` from 0 (left out: the one shown, at first the last); what is left out stays as shown. A fluid step is first shown with velocity on a slice normal to the horizontal axis across the flow, on its symmetry plane when the domain has one along that axis, else through the middle. The reply's `contour.range` (and the legend's Max/Min) is the max over interior fluid cells; the open-boundary layers are excluded, convective outlets too, because a wake reaching a uniform pressure outlet distorts the last layer"#,
    form: Some(super::forms::SLICE),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: slice,
};

pub const SURFACE: CommandSpec = CommandSpec {
    id: "results.surface",
    label: "Flow on Surfaces",
    params: r#"{"step": "Flow", "show": true, "field"?: "velocity" | "pressure" | "temperature" | "wall_heat_flux", "frame"?: 10} — colours a fluid step's obstacles (the part drawn) by the field shown, read in the fluid cells beside them; `wall_heat_flux` is signed power received by each obstacle face divided by its complete tessellated area, in model power/area units (positive into the wall); histories interpolate at the shown frame without extrapolation; recorded adiabatic faces are zero, missing histories remain uncoloured; no slice is drawn for this surface-only field. `show: false` draws the part in its own colours"#,
    form: Some(super::forms::FLOW_SURFACE),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: surface,
};

pub const STREAMLINES: CommandSpec = CommandSpec {
    id: "results.streamlines",
    label: "Streamlines",
    params: r#"{"step": "Flow", "show": true, "field"?: "velocity" | "pressure" | "temperature", "count": 64, "max_steps": 4000, "seeds": [[x, y, z], …], "line": {"from": [x, y, z], "to": [x, y, z]}, "frame": 10} — streamlines of a fluid step, traced downstream through its velocity field (trilinear between the cells, midpoint rule, half-cell steps) and coloured by speed or temperature when that field is selected (plain for pressure): from `seeds` (at most 2000 points), from `count` points evenly along `line`, or else from `count` seeds (1 to 2000, default 64) on the inlet face upstream of the obstacles; each at most `max_steps` steps (1 to 100000, default 4000), 2 000 000 points in all; `show: false` hides them; `frame` as for results.slice. Replies with the view and `traced`: the streamlines, their points, the seeds and how many started none (outside the fluid)"#,
    form: Some(super::forms::STREAMLINES),
    kind: CommandKind::Session,
    requires: &[Requirement::Result],
    run: streamlines,
};

pub const CLEAR: CommandSpec = CommandSpec {
    id: "results.clear",
    label: "Hide Results",
    params: "{} — back to the model: hides the deformed shape, contours and modes",
    form: Some(&[]),
    kind: CommandKind::Session,
    requires: &[Requirement::ResultsShown],
    run: clear,
};

pub const READ: CommandSpec = CommandSpec {
    id: "results.read",
    label: "Read External Results",
    params: r#"{"format": "warp3d", "path": "run_dir", "step": "Load", "units": "mm-t-s-N-MPa|m-kg-s-N-Pa|mm-kg-ms-kN-GPa|mm-g-ms-N-MPa|in-lbf-s-psi"} — reads external solver results (WARP3D flat text files `wnd*_text`, `wns*_text`, `wes*_text`, `wnr*_text`, and optional `.out`/`.log` 3D fracture domain/interaction integral tables) for a step of the current model, converts them to SI, and keeps them in the session for contours, deformed view, reactions, XY plots, fracture CSV export (`results.export.csv`) and saving (session-only fracture tables are not persisted in `.fieldcraft` files)"#,
    form: Some(super::forms::RESULTS_READ),
    kind: CommandKind::Session,
    requires: &[Requirement::Analysable, Requirement::ElementMaterial, Requirement::ElementSection, Requirement::Step],
    run: read,
};

/// Most sets one reaction sum lists.
const MAX_SETS: usize = 1000;
/// Largest deformation scale factor.
const MAX_SCALE: f64 = crate::results::MAX_DEFORMATION_SCALE;

fn solved_list(s: &Session) -> String {
    let names: Vec<&str> = s.result_steps().map(|(n, _)| n).collect();
    if names.is_empty() { "no step is solved yet (solve.static, solve.frequency)".into() } else { format!("solved steps: {}", names.join(", ")) }
}

/// The steps a command works on.
#[derive(Clone, Copy, PartialEq)]
pub(super) enum Want {
    Any,
    Frequency,
    /// Steps with frames: explicit dynamics, fluid and nonlinear static (one per converged
    /// increment).
    Frames,
    Fluid,
}

impl Want {
    fn fits(self, s: &Session, step: &str) -> bool {
        s.latest_result(step).is_some_and(|(r, _)| match (self, r.kind()) {
            (Want::Any, _) | (Want::Frequency, Kind::Frequency(_)) | (Want::Frames | Want::Fluid, Kind::Fluid(_)) => true,
            (Want::Frames, Kind::Explicit(_)) => true,
            (Want::Frames, Kind::Static(r)) => r.nonlinear.is_some(),
            (Want::Frequency | Want::Frames | Want::Fluid, _) => false,
        })
    }
    fn name(self) -> &'static str {
        match self {
            Want::Any => "solved",
            Want::Frequency => "frequency",
            Want::Frames => "explicit, fluid or nonlinear static",
            Want::Fluid => "fluid",
        }
    }
}

/// The step a command works on: `step`, else the step shown, else the first solved one
/// (current results first) of the kind it wants.
pub(super) fn step_for(s: &Session, p: &Value, want: Want) -> Result<String> {
    if let Some(name) = opt::<String>(p, "step")? {
        if s.latest_result(&name).is_none() {
            return Err(EngineError::Params(format!("step `{name}` has no result; {}", solved_list(s))));
        }
        if !want.fits(s, &name) {
            return Err(EngineError::Params(format!("step `{name}` is not a {} step", want.name())));
        }
        return Ok(name);
    }
    if let Some(v) = &s.results_view
        && want.fits(s, &v.step)
    {
        return Ok(v.step.clone());
    }
    let mut steps: Vec<(&str, bool)> = s.result_steps().filter(|(n, _)| want.fits(s, n)).collect();
    // Current results first, then by name (stable).
    steps.sort_by_key(|(_, current)| !*current);
    steps.first().map(|(n, _)| n.to_string()).ok_or_else(|| {
        EngineError::Params(if want == Want::Any { solved_list(s) } else { format!("no {} step has a result; {}", want.name(), solved_list(s)) })
    })
}

/// The view of `step`: the one shown, or the step's first look ([`ResultsView::for_result`])
/// when another step (or none) is shown.
fn view_of<'a>(s: &'a mut Session, step: &str) -> Result<&'a mut ResultsView> {
    if s.results_view.as_ref().is_none_or(|v| v.step != step) {
        let fresh = s.latest_result(step).map(|(r, _)| ResultsView::for_result(r, &s.model)).ok_or_else(|| EngineError::Params(solved_list(s)))?;
        s.results_view = Some(fresh);
    }
    s.results_view.as_mut().ok_or_else(|| EngineError::Params("no results are shown".into()))
}

fn deformed(s: &mut Session, p: &Value) -> Result<Value> {
    let step = step_for(s, p, Want::Any)?;
    let show = opt::<bool>(p, "show")?.unwrap_or(true);
    let scale = match p.get("scale") {
        None => None,
        Some(Value::String(a)) if a == "auto" => Some(None),
        Some(v) => Some(Some(
            v.as_f64()
                .filter(|x| x.is_finite() && (0.0..=MAX_SCALE).contains(x))
                .map(f64::abs)
                .ok_or_else(|| EngineError::Params(format!("`scale` must be \"auto\" or a factor from 0 (undeformed) to {MAX_SCALE}")))?,
        )),
    };
    let v = view_of(s, &step)?;
    v.deformed = show;
    if let Some(scale) = scale {
        v.scale = scale;
    }
    Ok(s.results_report())
}

fn contour(s: &mut Session, p: &Value) -> Result<Value> {
    let step = step_for(s, p, Want::Any)?;
    let name = opt::<String>(p, "field")?.ok_or_else(|| EngineError::param("field", ProblemKind::Required))?;
    let field = match name.as_str() {
        "none" => None,
        id => Some(Field::parse(id).ok_or_else(|| {
            let ids: Vec<&str> = Field::ALL.iter().map(|f| f.id()).collect();
            EngineError::Params(format!("unknown field `{id}` (one of {}, or none)", ids.join(", ")))
        })?),
    };
    let bands = match p.get("bands") {
        None => None,
        Some(b) => Some(
            b.as_u64()
                .and_then(|n| u32::try_from(n).ok())
                .filter(|n| (MIN_BANDS..=MAX_BANDS).contains(n))
                .ok_or_else(|| EngineError::Params(format!("`bands` must be a whole number from {MIN_BANDS} to {MAX_BANDS}")))?,
        ),
    };
    if let (Some(f), Some((r, _))) = (field, s.latest_result(&step))
        && !Field::available(r).any(|a| a == f)
    {
        if f == Field::Temperature && matches!(r.kind(), Kind::Fluid(_)) {
            return Err(EngineError::Params(format!(
                "temperature is not available for fluid step `{step}`: its frames have no complete temperature field (isothermal flow)"
            )));
        }
        if f == Field::WallHeatFlux {
            return Err(EngineError::Params(format!("wall heat flux is not available for step `{step}`: it requires thermal obstacle histories")));
        }
        return Err(EngineError::Params(format!("{} is not available for step `{step}` (mode shapes have no stress scale)", f.label())));
    }
    // The legend's range is in the field's display unit; it is kept in SI.
    let range = match (p.get("range"), field, s.latest_result(&step)) {
        (None, ..) => None,
        (Some(r), Some(f), Some((result, _))) => {
            let input = s.input_units().clone();
            let stored = |v: f64| f.stored(result, v, &input);
            Some(legend_range(r, &|k, v| match f.quantity().filter(|_| !f.is_mode_shape(result)) {
                Some(q) if !v.is_number() => {
                    crate::units::read(v, q, &input).map(|x| x.map_err(|e| EngineError::invalid(format!("range/{k}"), e.to_string())))
                }
                _ => v.as_f64().filter(|x| x.is_finite()).map(|x| Ok(stored(x))),
            })?)
        }
        (Some(r), ..) => Some(legend_range(r, &|_, v| v.as_f64().filter(|x| x.is_finite()).map(Ok))?),
    };
    if field == Some(Field::WallHeatFlux)
        && let Some((r, _)) = s.latest_result(&step)
        && let Kind::Fluid(f) = r.kind()
    {
        let pick = s.results_view.as_ref().filter(|v| v.step == step).map(|v| v.pick()).unwrap_or_default();
        crate::flow::wall::faces(s, f, pick)?;
    }
    let v = view_of(s, &step)?;
    if v.contour != field {
        (v.range_min, v.range_max) = (None, None);
    }
    v.contour = field;
    if let Some(b) = bands {
        v.bands = b;
    }
    if let Some((lo, hi)) = range {
        (v.range_min, v.range_max) = (lo, hi);
    }
    Ok(s.results_report())
}

/// `results.contour`'s `range`: `"auto"` or null (follow the field), or `{min?, max?}`, each a
/// value `value` reads (to SI; `None`: not a value), null or `"auto"`; `min` below `max`.
fn legend_range(r: &Value, value: &dyn Fn(&str, &Value) -> Option<Result<f64>>) -> Result<(Option<f64>, Option<f64>)> {
    let bound = |k: &str| -> Result<Option<f64>> {
        match r.get(k) {
            None | Some(Value::Null) => Ok(None),
            Some(Value::String(a)) if a == "auto" => Ok(None),
            Some(v) => value(k, v).ok_or_else(|| EngineError::Params(format!("`range.{k}` must be a number or \"auto\"")))?.map(Some),
        }
    };
    match r {
        Value::Null => Ok((None, None)),
        Value::String(a) if a == "auto" => Ok((None, None)),
        Value::Object(_) => {
            let (lo, hi) = (bound("min")?, bound("max")?);
            if let (Some(a), Some(b)) = (lo, hi)
                && a >= b
            {
                return Err(EngineError::Params(format!("`range.min` ({a}) must be below `range.max` ({b})")));
            }
            Ok((lo, hi))
        }
        _ => Err(EngineError::Params("`range` must be {\"min\"?, \"max\"?} or \"auto\"".into())),
    }
}

fn modes(s: &mut Session, p: &Value) -> Result<Value> {
    let step = step_for(s, p, Want::Frequency)?;
    let count = match s.latest_result(&step).map(|(r, _)| r.kind()) {
        Some(Kind::Frequency(f)) => f.modes.len(),
        _ => 0,
    };
    let mode = match p.get("mode") {
        None => s.results_view.as_ref().filter(|v| v.step == step).map_or(1, |v| v.mode),
        Some(m) => m
            .as_u64()
            .and_then(|n| usize::try_from(n).ok())
            .filter(|n| (1..=count).contains(n))
            .ok_or_else(|| EngineError::Params(format!("`mode` must be from 1 to {count}")))?,
    };
    let v = view_of(s, &step)?;
    v.mode = mode;
    v.deformed = true;
    Ok(s.results_report())
}

fn animate(s: &mut Session, p: &Value) -> Result<Value> {
    let on = opt::<bool>(p, "on")?.unwrap_or(true);
    if !on {
        if let Some(v) = &mut s.results_view {
            v.animate = false;
        }
        return Ok(s.results_report());
    }
    let step = step_for(s, &json!({}), Want::Any)?;
    let fluid = s.latest_result(&step).is_some_and(|(r, _)| matches!(r.kind(), Kind::Fluid(_)));
    let v = view_of(s, &step)?;
    v.animate = true;
    // A fluid step has nothing to deform; animating plays its frames (as results.frame does) in
    // order. A structural step animates its deformation (or an explicit step its frames).
    v.deformed = !fluid;
    Ok(s.results_report())
}

fn frame(s: &mut Session, p: &Value) -> Result<Value> {
    let step = step_for(s, p, Want::Frames)?;
    let times: Vec<f64> = match s.latest_result(&step).map(|(r, _)| r.kind()) {
        Some(Kind::Explicit(e)) => e.frames.iter().map(|f| f.time).collect(),
        Some(Kind::Static(r)) => r.nonlinear.as_ref().map(|h| h.load_factors()).unwrap_or_default(),
        Some(Kind::Fluid(f)) => f.frames.iter().map(|f| f.time).collect(),
        _ => Vec::new(),
    };
    let last = times.len().checked_sub(1).ok_or_else(|| EngineError::Params(format!("step `{step}` has no field frames")))?;
    // A time (explicit, fluid) is in the display system; a nonlinear static step's is a load factor.
    let time_value = match (p.get("time"), s.latest_result(&step).map(|(r, _)| r.kind())) {
        (None, _) | (Some(Value::Null), _) => None,
        (Some(v), Some(Kind::Static(_))) => Some(v.as_f64().ok_or_else(|| EngineError::param("time", ProblemKind::NotANumber))?),
        (Some(v), _) => Some(
            crate::units::read(v, crate::results::Quantity::Time, s.input_units())
                .ok_or_else(|| EngineError::param("time", ProblemKind::NotANumber))?
                .map_err(|e| EngineError::invalid("time", e.to_string()))?,
        ),
    };
    let k = match (p.get("frame"), time_value) {
        (Some(f), _) => f
            .as_u64()
            .and_then(|n| usize::try_from(n).ok())
            .filter(|n| *n <= last)
            .ok_or_else(|| EngineError::Params(format!("`frame` must be from 0 to {last}")))?,
        (None, Some(t)) if t.is_finite() => {
            times.iter().enumerate().min_by(|a, b| (a.1 - t).abs().total_cmp(&(b.1 - t).abs())).map_or(last, |(i, _)| i)
        }
        (None, Some(_)) => return Err(EngineError::param("time", ProblemKind::NotANumber)),
        (None, None) => last,
    };
    let v = view_of(s, &step)?;
    v.frame = Some(k);
    Ok(s.results_report())
}

/// The fluid step a fluid display command works on (`step`, or the one shown) and its result.
fn fluid_step(s: &Session, p: &Value) -> Result<(String, std::sync::Arc<crate::StepResult>)> {
    let step = step_for(s, p, Want::Fluid)?;
    let r = s.latest_result(&step).map(|(r, _)| r.clone()).ok_or_else(|| EngineError::Params(solved_list(s)))?;
    Ok((step, r))
}

/// A whole number in `range`, when given.
fn whole(p: &Value, key: &str, range: std::ops::RangeInclusive<u64>) -> Result<Option<u64>> {
    match p.get(key).filter(|v| !v.is_null()) {
        None => Ok(None),
        Some(v) => v.as_u64().filter(|n| range.contains(n)).map(Some).ok_or_else(|| EngineError::param(key, super::mesh::whole_in(&range))),
    }
}

/// A fluid command's `frame`: from 0 to the last; `None` when left out.
pub(super) fn fluid_frame(f: &fieldcraft_lbm::FluidResult, p: &Value) -> Result<Option<usize>> {
    let last = f.frames.len().saturating_sub(1) as u64;
    Ok(whole(p, "frame", 0..=last)?.and_then(|k| usize::try_from(k).ok()))
}

/// A whole number from 1 to `max`, when given.
fn count_param(p: &Value, key: &str, max: u32) -> Result<Option<u32>> {
    Ok(whole(p, key, 1..=u64::from(max))?.and_then(|n| u32::try_from(n).ok()))
}

/// Largest coordinate a seed may have.
const MAX_COORDINATE: f64 = 1e12; // unit-factors: a cap on hostile input

/// A point `[x, y, z]` of finite coordinates no larger than [`MAX_COORDINATE`].
fn seed_point(v: &Value) -> Option<[f64; 3]> {
    let a = v.as_array().filter(|a| a.len() == 3)?;
    let c: Vec<f64> = a.iter().filter_map(Value::as_f64).filter(|x| x.is_finite() && x.abs() <= MAX_COORDINATE).collect();
    match c[..] {
        [x, y, z] => Some([x, y, z]),
        _ => None,
    }
}

fn slice(s: &mut Session, p: &Value) -> Result<Value> {
    let (step, r) = fluid_step(s, p)?;
    let Kind::Fluid(f) = r.kind() else { return Err(EngineError::Params(format!("step `{step}` is not a fluid step"))) };
    let field = fluid_field(p, &r)?;
    // Some(None): hide the slice.
    let axis = match opt::<String>(p, "axis")?.as_deref() {
        None => None,
        Some("none") => Some(None),
        Some(a) => Some(Some(Axis::parse(a).ok_or_else(|| EngineError::invalid("axis", "must be x, y, z or none"))?)),
    };
    // Some(None): the symmetry plane along the axis, else the domain's middle.
    let position = match p.get("position").filter(|v| !v.is_null()) {
        None => None,
        Some(Value::String(a)) if a == "auto" => Some(None),
        Some(v) => Some(Some(v.as_f64().filter(|x| x.is_finite()).ok_or_else(|| EngineError::param("position", ProblemKind::NotANumber))?)),
    };
    let frame = fluid_frame(f, p)?;
    let symmetry = crate::flow::symmetry_faces(&s.model, f);
    let first = FlowView::for_result(f, symmetry);
    let shown = s.results_view.as_ref().filter(|v| v.step == step).and_then(|v| v.flow.clone()).unwrap_or_else(|| first.clone());
    // Not given: the shown position; "auto": the symmetry plane along the axis, else the middle.
    let at = |axis: Axis, kept: Option<f64>| match position {
        None => kept,
        Some(None) => symmetry_position(f, axis, symmetry),
        Some(Some(x)) => Some(x),
    };
    let slice = match axis {
        Some(None) => None,
        Some(Some(a)) => {
            // A new axis starts on its symmetry plane; the same one keeps its position.
            let kept = shown.slice.as_ref().filter(|s| s.axis == a).map(|s| s.position);
            Some(SliceView { axis: a, position: at(a, kept.unwrap_or_else(|| symmetry_position(f, a, symmetry))) })
        }
        None => {
            let current = shown.slice.clone().or(first.slice);
            current.map(|s| SliceView { position: at(s.axis, s.position), ..s })
        }
    };
    if let Some(sv) = &slice
        && let Some(at) = sv.position
    {
        let g = crate::flow::grid(f);
        let (lo, hi) = (g.start(sv.axis.index()), g.end(sv.axis.index()));
        if !(at >= lo && at <= hi) {
            return Err(EngineError::invalid("position", format!("must be inside the domain: {lo} to {hi} along {}", sv.axis.id())));
        }
    }
    let v = view_of(s, &step)?;
    if let Some(field) = field {
        if v.contour != Some(field) {
            (v.range_min, v.range_max) = (None, None);
        }
        v.contour = Some(field);
    }
    if frame.is_some() {
        v.frame = frame;
    }
    v.flow = Some(FlowView { slice, ..shown });
    Ok(s.results_report())
}

fn surface(s: &mut Session, p: &Value) -> Result<Value> {
    let (step, r) = fluid_step(s, p)?;
    let Kind::Fluid(f) = r.kind() else { return Err(EngineError::Params(format!("step `{step}` is not a fluid step"))) };
    let show = opt::<bool>(p, "show")?.unwrap_or(true);
    let field = if opt::<String>(p, "field")?.as_deref() == Some("wall_heat_flux") {
        if !Field::available(&r).any(|f| f == Field::WallHeatFlux) {
            return Err(EngineError::invalid("field", "wall heat flux is unavailable: this step has no thermal obstacle histories"));
        }
        Some(Field::WallHeatFlux)
    } else {
        fluid_field(p, &r)?
    };
    let frame = fluid_frame(f, p)?;
    if field == Some(Field::WallHeatFlux) {
        let mut pick = s.results_view.as_ref().filter(|v| v.step == step).map(|v| v.pick()).unwrap_or_default();
        if frame.is_some() {
            pick.frame = frame;
        }
        crate::flow::wall::faces(s, f, pick)?;
    }
    let v = view_of(s, &step)?;
    set_fluid_field(v, field);
    if frame.is_some() {
        v.frame = frame;
    }
    let mut flow = FlowView::of(v, f).into_owned();
    flow.surface = show;
    v.flow = Some(flow);
    Ok(s.results_report())
}

fn streamlines(s: &mut Session, p: &Value) -> Result<Value> {
    let (step, r) = fluid_step(s, p)?;
    let Kind::Fluid(f) = r.kind() else { return Err(EngineError::Params(format!("step `{step}` is not a fluid step"))) };
    let show = opt::<bool>(p, "show")?.unwrap_or(true);
    let field = fluid_field(p, &r)?;
    let count = count_param(p, "count", MAX_STREAMLINES)?;
    let max_steps = count_param(p, "max_steps", MAX_STREAMLINE_STEPS)?;
    let max = MAX_STREAMLINES as usize;
    let seeds = match p.get("seeds").filter(|v| !v.is_null()) {
        None => None,
        Some(_) => {
            let list = super::points(p, "seeds", max)?;
            if list.is_empty() || list.iter().flatten().any(|c| !(c.is_finite() && c.abs() <= MAX_COORDINATE)) {
                return Err(EngineError::invalid("seeds", format!("must be 1 to {max} points [x, y, z] of finite coordinates")));
            }
            Some(list)
        }
    };
    let line = match p.get("line").filter(|v| !v.is_null()) {
        None => None,
        Some(v) => {
            let end = |k: &str| v.get(k).and_then(seed_point);
            match (end("from"), end("to")) {
                (Some(a), Some(b)) => Some((a, b)),
                _ => return Err(EngineError::invalid("line", "must be {\"from\": [x, y, z], \"to\": [x, y, z]} of finite coordinates")),
            }
        }
    };
    if seeds.is_some() && line.is_some() {
        return Err(EngineError::invalid("line", "give `seeds` or `line`, not both"));
    }
    let frame = fluid_frame(f, p)?;
    let v = view_of(s, &step)?;
    set_fluid_field(v, field);
    let mut flow = FlowView::of(v, f).into_owned();
    flow.streamlines = if show {
        let mut sl = flow.streamlines.take().unwrap_or_default();
        sl.count = count.unwrap_or(sl.count);
        sl.max_steps = max_steps.unwrap_or(sl.max_steps);
        sl.seeds = match (seeds, line) {
            (Some(points), _) => Some(points),
            (None, Some((a, b))) => {
                let n = sl.count.max(1);
                let at = |i: u32| if n == 1 { 0.5 } else { f64::from(i) / f64::from(n - 1) };
                Some((0..n).map(|i| std::array::from_fn(|k| a[k] + (b[k] - a[k]) * at(i))).collect())
            }
            (None, None) => None,
        };
        Some(sl)
    } else {
        None
    };
    if frame.is_some() {
        v.frame = frame;
    }
    let pick = v.pick();
    let traced = flow.streamlines.clone();
    v.flow = Some(flow);
    let mut out = s.results_report();
    if let Some(sl) = traced {
        // Traced once: the plot drawn after this reply takes the same lines from the session.
        let k = crate::flow::frame_index(f, pick)?;
        let (seeds, lines) = s.traced.trace(&r, k, &sl)?;
        let points: usize = lines.iter().map(|l| l.points.len()).sum();
        out["traced"] = json!({"streamlines": lines.len(), "points": points, "seeds": seeds, "no_start": seeds.saturating_sub(lines.len())});
    }
    Ok(out)
}

fn fluid_field(p: &Value, r: &crate::StepResult) -> Result<Option<Field>> {
    let field = match opt::<String>(p, "field")?.as_deref() {
        None => return Ok(None),
        Some("velocity") => Field::Velocity,
        Some("pressure") => Field::Pressure,
        Some("temperature") => Field::Temperature,
        Some(_) => return Err(EngineError::invalid("field", "must be velocity, pressure or temperature")),
    };
    if !Field::available(r).any(|f| f == field) {
        return Err(EngineError::invalid(
            "field",
            "temperature is unavailable: this isothermal step has no complete temperature field in its frames",
        ));
    }
    Ok(Some(field))
}

fn set_fluid_field(view: &mut ResultsView, field: Option<Field>) {
    if let Some(field) = field {
        if view.contour != Some(field) {
            (view.range_min, view.range_max) = (None, None);
        }
        view.contour = Some(field);
    }
}

fn clear(s: &mut Session, _: &Value) -> Result<Value> {
    s.results_view = None;
    Ok(json!({"view": null}))
}

fn reactions(s: &mut Session, p: &Value) -> Result<Value> {
    let step = step_for(s, p, Want::Any)?;
    let about = if p.get("about").is_some() { xyz(p, "about")? } else { [0.0; 3] };
    if !about.iter().all(|c| c.is_finite()) {
        return Err(EngineError::param("about", ProblemKind::NotAVector { len: 3 }));
    }
    let frame_index: Option<usize> = opt(p, "frame")?;
    let Some((result, current)) = s.latest_result(&step) else { return Err(EngineError::Params(solved_list(s))) };
    let Kind::Static(r) = result.kind() else {
        return Err(EngineError::Params(format!(
            "step `{step}` is not a static step: reactions are kept for static steps (explicit steps keep the constraint force history, solve.history)"
        )));
    };
    let frame = match frame_index {
        None => None,
        Some(k) => Some(nonlinear_frame(&step, r, k)?),
    };
    let sets = reaction_sets(s, &step, p)?;
    let reaction = |i: usize| -> Option<[f64; 3]> {
        match frame {
            Some(f) => Some(f.reaction_at(i)),
            None => r.reaction.get(i).copied(),
        }
    };
    let mut per_set = Vec::with_capacity(sets.len());
    for name in &sets {
        let resolved = s.model.resolve_set(name)?;
        let mut row = reaction_sum(s, r, &reaction, &mut resolved.nodes.iter().copied(), about);
        row["set"] = json!(name);
        per_set.push(row);
    }
    let total = reaction_sum(s, r, &reaction, &mut r.nodes.iter().copied(), about);
    let mut out = json!({"step": step, "stale": !current, "about": about, "sets": per_set, "total": total});
    match (frame_index, frame) {
        (Some(k), Some(f)) => {
            out["frame"] = json!(k);
            out["load_factor"] = json!(f.load_factor);
        }
        _ => out["applied"] = json!(r.applied_total),
    }
    Ok(out)
}

/// Frame `k` of a nonlinear static step.
pub(crate) fn nonlinear_frame<'a>(step: &str, r: &'a fieldcraft_solver::StaticResult, k: usize) -> Result<&'a fieldcraft_solver::nonlinear::Frame> {
    let h = r
        .nonlinear
        .as_ref()
        .ok_or_else(|| EngineError::invalid("frame", format!("step `{step}` is a linear static step: it has one state and no frames")))?;
    h.frames.get(k).ok_or_else(|| {
        EngineError::invalid("frame", format!("step `{step}` has frames 0 to {}; there is no frame {k}", h.frames.len().saturating_sub(1)))
    })
}

/// The sets `p.sets` names (at most [`MAX_SETS`]); none or an empty list (a form with none
/// ticked): the sets the step's boundary conditions hold.
pub(crate) fn reaction_sets(s: &Session, step: &str, p: &Value) -> Result<Vec<String>> {
    Ok(match p.get("sets").filter(|v| v.as_array().is_none_or(|a| !a.is_empty())) {
        Some(v) => {
            let list = v
                .as_array()
                .filter(|a| a.len() <= MAX_SETS)
                .ok_or_else(|| EngineError::Params(format!("`sets` must be a list of at most {MAX_SETS} set names")))?;
            list.iter()
                .map(|n| n.as_str().map(str::to_string).ok_or_else(|| EngineError::param("sets", ProblemKind::NotNames)))
                .collect::<Result<_>>()?
        }
        None => {
            let mut names: Vec<String> = s
                .model
                .step(step)
                .map(|st| st.boundary_conditions.iter().filter_map(|bc| s.model.boundary_condition(bc)?.target().set().map(str::to_string)).collect())
                .unwrap_or_default();
            names.sort();
            names.dedup();
            names
        }
    })
}

/// Σ R and Σ (x − about) × R over `nodes` (undeformed positions), the reaction of the node of
/// index `i` in the result's order being `reaction(i)`: {nodes, force, moment}.
pub(crate) fn reaction_sum(
    s: &Session,
    r: &fieldcraft_solver::StaticResult,
    reaction: &dyn Fn(usize) -> Option<[f64; 3]>,
    nodes: &mut dyn Iterator<Item = u32>,
    about: [f64; 3],
) -> Value {
    let (mut force, mut moment, mut count) = ([0.0; 3], [0.0; 3], 0usize);
    for label in nodes {
        let (Ok(i), Some(node)) = (r.nodes.binary_search(&label), s.model.node(label)) else { continue };
        let Some(f) = reaction(i) else { continue };
        let d = [node.xyz[0] - about[0], node.xyz[1] - about[1], node.xyz[2] - about[2]];
        for k in 0..3 {
            force[k] += f[k];
        }
        moment[0] += d[1] * f[2] - d[2] * f[1];
        moment[1] += d[2] * f[0] - d[0] * f[2];
        moment[2] += d[0] * f[1] - d[1] * f[0];
        count += 1;
    }
    json!({"nodes": count, "force": force, "moment": moment})
}

/// Most files one `results.export.vtu` writes.
pub const MAX_VTU_FILES: usize = 10_000;
/// Most bytes one `results.export.vtu` writes, over all its files.
pub const MAX_VTU_BYTES: u64 = 8 << 30;

/// One state of a result written as a `.vtu`: the end of a linear step, a mode (from 1) or a
/// frame (from 0).
#[derive(Clone, Copy, Debug, PartialEq)]
enum State {
    Final,
    Mode(usize),
    Frame(usize),
}

fn export_vtu(s: &mut Session, p: &Value) -> Result<Value> {
    use fieldcraft_io::vtk::{self, Encoding};
    let step = step_for(s, p, Want::Any)?;
    let mode: Option<usize> = opt(p, "mode")?;
    let last_only = match opt::<String>(p, "frames")?.as_deref() {
        None | Some("all") => false,
        Some("last") => true,
        Some(_) => return Err(EngineError::invalid("frames", "must be \"all\" or \"last\"")),
    };
    let encoding = match opt::<String>(p, "encoding")?.as_deref() {
        None | Some("ascii") => Encoding::Ascii,
        Some("binary") => Encoding::Base64,
        Some(_) => return Err(EngineError::invalid("encoding", "must be \"ascii\" or \"binary\"")),
    };
    let path = super::file_param(s, p, "path", fieldcraft_io::roots::Access::Write)?;
    let target = std::path::Path::new(&path);
    let series = match target.extension().map(|e| e.to_string_lossy().to_ascii_lowercase()).as_deref() {
        Some("vtu") => false,
        Some("pvd") => true,
        _ => return Err(EngineError::Params("`path` must end in .vtu (one file) or .pvd (a series of .vtu files)".into())),
    };
    let r = match s.latest_result(&step) {
        Some((_, false)) => return Err(EngineError::Params(format!("the model changed since step `{step}` was solved; solve it again"))),
        Some((r, true)) => std::sync::Arc::clone(r),
        None => return Err(EngineError::Params(format!("step `{step}` has no result; solve it first"))),
    };
    let frames = |n: usize, what: &str| -> Result<Vec<State>> {
        let last = n.checked_sub(1).ok_or_else(|| EngineError::Params(format!("step `{step}` has no {what}")))?;
        Ok(if series && !last_only { (0..n).map(State::Frame).collect() } else { vec![State::Frame(last)] })
    };
    if mode.is_some() && !matches!(r.kind(), Kind::Frequency(_)) {
        return Err(EngineError::invalid("mode", "is for frequency and buckling steps"));
    }
    let states: Vec<State> = match r.kind() {
        Kind::Fluid(_) => return Err(EngineError::Params("fluid results are per cell; not supported yet".into())),
        Kind::Static(st) => match &st.nonlinear {
            Some(h) if !h.frames.is_empty() => frames(h.frames.len(), "frames")?,
            _ => vec![State::Final],
        },
        Kind::Frequency(f) => match mode {
            Some(m) if m == 0 || m > f.modes.len() => {
                return Err(EngineError::invalid("mode", format!("step `{step}` has modes 1 to {}", f.modes.len())));
            }
            Some(m) => vec![State::Mode(m)],
            None if series && !last_only => (1..=f.modes.len()).map(State::Mode).collect(),
            None if f.modes.is_empty() => return Err(EngineError::Params(format!("step `{step}` has no modes"))),
            None => vec![State::Mode(1)],
        },
        Kind::Explicit(e) => frames(e.frames.len(), "field frames")?,
        Kind::Heat(h) => frames(h.frames.len(), "temperatures")?,
    };
    if states.len() > MAX_VTU_FILES {
        return Err(EngineError::Params(format!("step `{step}` has {} frames; at most {MAX_VTU_FILES} files are written", states.len())));
    }

    // Written in the display system, its units named in each file.
    let display = s.units().clone();
    let model = &s.model.converted(&|q, v| display.express(q, v).ok())?;
    let (nodes, elements) = vtk::points_and_cells(model);
    let build = |state: State| -> Result<(f64, String)> {
        let (time, fields, data, units) = vtu_fields(&r, state, &nodes, &elements, &display);
        let data: Vec<(&str, f64)> = data.iter().map(|(n, v)| (*n, *v)).collect();
        Ok((time, vtk::write_vtu_in(model, &fields, &data, encoding, &units)?))
    };
    // The last state has the most fields: its size times the count bounds the output before
    // anything is written.
    let Some(&last) = states.last() else { return Err(EngineError::Params(format!("step `{step}` has nothing to write"))) };
    let sample = build(last)?;
    let estimate = (sample.1.len() as u64).saturating_mul(states.len() as u64);
    if estimate > MAX_VTU_BYTES {
        return Err(EngineError::Params(format!(
            "the export would write about {estimate} bytes; at most {MAX_VTU_BYTES} (8 GiB) are written: choose `frames: \"last\"` or a single .vtu"
        )));
    }

    let mut written: Vec<std::path::PathBuf> = Vec::new();
    let mut bytes = 0usize;
    let result = (|| -> Result<Vec<String>> {
        if !series {
            fieldcraft_io::fs::write_output(target, sample.1.as_bytes())?;
            written.push(target.to_path_buf());
            bytes += sample.1.len();
            return Ok(vec![path.clone()]);
        }
        let stem = target.file_stem().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
        let dir = target.parent().unwrap_or(std::path::Path::new(""));
        let mut entries: Vec<(f64, String)> = Vec::with_capacity(states.len());
        let mut files = Vec::with_capacity(states.len() + 1);
        for (k, &state) in states.iter().enumerate() {
            let name = format!("{stem}_{k:04}.vtu");
            let file = dir.join(&name);
            let built = if state == last { None } else { Some(build(state)?) };
            let (time, text) = built.as_ref().map_or((sample.0, sample.1.as_str()), |(t, x)| (*t, x.as_str()));
            fieldcraft_io::fs::write_output(&file, text.as_bytes())?;
            written.push(file.clone());
            bytes += text.len();
            entries.push((time, name));
            files.push(file.to_string_lossy().into_owned());
        }
        let list: Vec<(f64, &str)> = entries.iter().map(|(t, n)| (*t, n.as_str())).collect();
        let collection = vtk::pvd(&list);
        fieldcraft_io::fs::write_output(target, collection.as_bytes())?;
        bytes += collection.len();
        files.push(path.clone());
        Ok(files)
    })();
    match result {
        Ok(files) => Ok(
            json!({"files": files, "bytes": bytes, "points": nodes.len(), "cells": elements.len(), "units": display.label(), "display_units": display.label()}),
        ),
        Err(e) => {
            // Leave nothing half written behind.
            for f in &written {
                let _ = std::fs::remove_file(f);
            }
            Err(e)
        }
    }
}

/// A stress (xx, yy, zz, xy, xz, yz) in VTK's symmetric tensor order (XX, YY, ZZ, XY, YZ, XZ).
fn vtk_tensor(t: &[f64; 6]) -> [f64; 6] {
    [t[0], t[1], t[2], t[3], t[5], t[4]]
}

/// Values given per label (`labels` sorted, as a result's nodes and elements are) spread over
/// `order` (the points' or cells' labels): `N` components each, 0 where a label has none.
fn spread<const N: usize>(order: &[u32], labels: &[u32], values: &[[f64; N]]) -> Vec<f64> {
    let mut out = vec![0.0; order.len() * N];
    for (label, v) in labels.iter().zip(values) {
        if let Ok(i) = order.binary_search(label)
            && let Some(slot) = out.get_mut(i * N..(i + 1) * N)
        {
            slot.copy_from_slice(v);
        }
    }
    out
}

fn scalars(values: &[f64]) -> Vec<[f64; 1]> {
    values.iter().map(|v| [*v]).collect()
}

/// The fields of one state of a result, its time (load factor, mode number), its field data and
/// the units of each, every value in `display`.
fn vtu_fields(
    r: &crate::StepResult,
    state: State,
    nodes: &[u32],
    elements: &[u32],
    display: &fieldcraft_model::DisplaySystem,
) -> (f64, Vec<fieldcraft_io::vtk::Field>, Vec<(&'static str, f64)>, Vec<(String, String)>) {
    use crate::results::Quantity as Q;
    use fieldcraft_io::vtk::{Field, Location};
    let mut fields = Vec::new();
    let mut data = Vec::new();
    let mut units = vec![("Points".to_string(), display.symbol(Q::Length).to_string())];
    // A field's values in `display`: a quantity, a factor (a mode shape), or as they are.
    enum Unit {
        Of(Q),
        Factor(f64),
        Plain,
    }
    let heat = matches!(r.kind(), Kind::Heat(_));
    let mut add = |name: &str, location: Location, components: usize, mut values: Vec<f64>, integer: bool| {
        let unit = match name {
            "displacement" => Unit::Of(Q::Length),
            "rotation" => Unit::Of(Q::Angle),
            "reaction" if heat => Unit::Of(Q::Power),
            "reaction" => Unit::Of(Q::Force),
            "stress" | "von_mises" => Unit::Of(Q::Pressure),
            "temperature" => Unit::Of(Q::Temperature),
            "velocity" => Unit::Of(Q::Velocity),
            "heat_flux" => Unit::Of(Q::HeatFlux),
            "mode_shape" => Unit::Factor(crate::results::mode_scale(r, display).0),
            _ => Unit::Plain,
        };
        match unit {
            Unit::Of(q) => {
                // A node without a result keeps its 0 (`has_result` says which).
                for v in values.iter_mut().filter(|v| **v != 0.0) {
                    *v = crate::results::shown(q, *v, display);
                }
                units.push((name.to_string(), display.symbol(q).to_string()));
            }
            Unit::Factor(k) => values.iter_mut().for_each(|v| *v *= k),
            Unit::Plain => {}
        }
        fields.push(Field { name: name.into(), location, components, values, integer });
    };
    let on = |labels: &[u32]| -> Vec<f64> { nodes.iter().map(|n| if labels.binary_search(n).is_ok() { 1.0 } else { 0.0 }).collect() };
    add("has_result", Location::Point, 1, on(r.nodes()), true);
    let point = Location::Point;
    let cell = Location::Cell;
    let time = match (r.kind(), state) {
        (Kind::Static(st), _) => {
            let frame = match (state, &st.nonlinear) {
                (State::Frame(k), Some(h)) => {
                    h.frames.get(k).map(|f| (f.load_factor, f.displacement.as_slice(), k + 1 == h.frames.len(), f.stress.as_slice()))
                }
                _ => None,
            };
            let (time, displacement, last, frame_stress) = frame.unwrap_or((0.0, st.displacement.as_slice(), true, &[]));
            add("displacement", point, 3, spread(nodes, &st.nodes, displacement), false);
            if last {
                if !st.shells.is_empty() || !st.beams.is_empty() {
                    add("rotation", point, 3, spread(nodes, &st.nodes, &st.rotation), false);
                }
                add("reaction", point, 3, spread(nodes, &st.nodes, &st.reaction), false);
            }
            // The final stress on the last state, a nonlinear frame's own before it (#205).
            let (stress, von_mises) = if last {
                (st.stress.as_slice(), std::borrow::Cow::Borrowed(st.von_mises.as_slice()))
            } else {
                (frame_stress, std::borrow::Cow::Owned(frame_stress.iter().map(fieldcraft_fem::von_mises).collect()))
            };
            if !stress.is_empty() {
                let stress: Vec<[f64; 6]> = stress.iter().map(vtk_tensor).collect();
                add("stress", point, 6, spread(nodes, &st.nodes, &stress), false);
                add("von_mises", point, 1, spread(nodes, &st.nodes, &scalars(&von_mises)), false);
            }
            if let Some(t) = st.temperature.as_ref().filter(|_| last) {
                add("temperature", point, 1, spread(nodes, &st.nodes, &scalars(t)), false);
            }
            if st.nonlinear.is_some() {
                data.push(("load_factor", time));
            }
            time
        }
        (Kind::Frequency(f), State::Mode(m)) => {
            if let Some(mode) = m.checked_sub(1).and_then(|k| f.modes.get(k)) {
                add("mode_shape", point, 3, spread(nodes, &f.nodes, &mode.shape), false);
                data.extend([
                    ("mode", m as f64),
                    ("frequency", crate::results::shown(Q::Frequency, mode.frequency, display)),
                    ("eigenvalue", crate::results::shown(Q::Eigenvalue, mode.eigenvalue, display)),
                ]);
            }
            m as f64
        }
        (Kind::Explicit(e), State::Frame(k)) => {
            let Some(f) = e.frames.get(k) else { return (0.0, fields, data, units) };
            if !f.displacement.is_empty() {
                add("displacement", point, 3, spread(nodes, &e.nodes, &f.displacement), false);
            }
            if !f.velocity.is_empty() {
                add("velocity", point, 3, spread(nodes, &e.nodes, &f.velocity), false);
            }
            if !f.stress.is_empty() {
                let stress: Vec<[f64; 6]> = f.stress.iter().map(vtk_tensor).collect();
                add("stress", cell, 6, spread(elements, &e.elements, &stress), false);
            }
            if !f.plastic_strain.is_empty() {
                add("plastic_strain", cell, 1, spread(elements, &e.elements, &scalars(&f.plastic_strain)), false);
            }
            if !f.damage.is_empty() {
                add("damage", cell, 1, spread(elements, &e.elements, &scalars(&f.damage)), false);
            }
            if !f.failed.is_empty() {
                let eroded: Vec<[f64; 1]> = f.failed.iter().map(|&b| [if b { 1.0 } else { 0.0 }]).collect();
                add("eroded", cell, 1, spread(elements, &e.elements, &eroded), true);
            }
            let time = crate::results::shown(Q::Time, f.time, display);
            data.push(("time", time));
            time
        }
        (Kind::Heat(h), State::Frame(k)) => {
            let time = crate::results::shown(Q::Time, h.times.get(k).copied().unwrap_or(0.0), display);
            if let Some(t) = h.frames.get(k) {
                add("temperature", point, 1, spread(nodes, &h.nodes, &scalars(t)), false);
            }
            if k + 1 == h.frames.len() && !h.flux.is_empty() {
                add("heat_flux", point, 3, spread(nodes, &h.nodes, &h.flux), false);
            }
            data.push(("time", time));
            time
        }
        _ => 0.0,
    };
    for (name, q) in [("frequency", Q::Frequency), ("eigenvalue", Q::Eigenvalue), ("time", Q::Time)] {
        if data.iter().any(|(n, _)| *n == name) {
            units.push((name.to_string(), display.symbol(q).to_string()));
        }
    }
    (time, fields, data, units)
}

fn export_csv(s: &mut Session, p: &Value) -> Result<Value> {
    let what: String = super::req(p, "what")?;
    let path = super::file_param(s, p, "path", fieldcraft_io::roots::Access::Write)?;
    let display = s.units().clone();
    let (step, table) = match what.as_str() {
        "probe" => {
            let r = super::solve::probe(s, p)?;
            let position = |label: u32| s.model.node(label).map(|n| crate::results::shown_all(crate::results::Quantity::Length, n.xyz, &display));
            (r.get("step").cloned().unwrap_or(Value::Null), probe_table(&r, &position, &display)?)
        }
        "reactions" => {
            let r = reactions(s, p)?;
            if r.get("stale").and_then(Value::as_bool).unwrap_or(false) {
                let step = r.get("step").and_then(Value::as_str).unwrap_or_default();
                return Err(EngineError::Params(format!(
                    "the result of step `{step}` is stale (the model changed since it was solved); solve it again"
                )));
            }
            let mut r = r;
            crate::units::convert_reply("results.reactions", &mut r, crate::units::si(), &display)?;
            (r.get("step").cloned().unwrap_or(Value::Null), reactions_table(&r, &display))
        }
        "fracture" => {
            let step = if let Some(step) = opt::<String>(p, "step")? {
                step
            } else if let Some(v) = &s.results_view
                && s.fracture_results.contains_key(&v.step)
            {
                v.step.clone()
            } else {
                let mut steps: Vec<&str> = s.unpersisted_fracture_steps();
                if steps.is_empty() {
                    steps = s.fracture_results.keys().map(String::as_str).collect();
                }
                match steps.as_slice() {
                    [only] => (*only).to_string(),
                    [] => return Err(EngineError::Params("no step has WARP3D fracture results to export".into())),
                    _ => return Err(EngineError::Params("multiple steps have fracture results; specify `step`".into())),
                }
            };
            let domains = match s.fracture_results(&step) {
                Some(d) if !d.is_empty() => d,
                _ if s.fracture_results.contains_key(&step) => {
                    return Err(EngineError::Params(format!(
                        "the fracture result of step `{step}` is stale (the model changed since it was solved); solve it again"
                    )));
                }
                _ => return Err(EngineError::Params(format!("step `{step}` has no WARP3D fracture results to export"))),
            };
            (json!(step), fracture_table(&step, domains, &display)?)
        }
        _ => return Err(EngineError::Params("`what` must be \"probe\", \"reactions\", or \"fracture\"".into())),
    };
    let text = table.csv();
    fieldcraft_io::fs::write_output(std::path::Path::new(&path), text.as_bytes())?;
    Ok(
        json!({"path": path, "rows": table.rows.len(), "columns": table.header, "step": step, "units": display.label(), "display_units": display.label()}),
    )
}

/// A table of cells (a number, text, or empty) under a header, written as CSV.
pub(super) struct Table {
    pub(super) header: Vec<String>,
    pub(super) rows: Vec<Vec<Cell>>,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) enum Cell {
    Number(f64),
    Text(String),
    Empty,
}

impl Table {
    /// RFC 4180: comma-separated, CRLF line ends, a field with a comma, quote or line break in
    /// quotes (its quotes doubled); numbers in Rust's shortest round-trip form.
    pub(super) fn csv(&self) -> String {
        let field = |s: &str| if s.contains([',', '"', '\r', '\n']) { format!("\"{}\"", s.replace('"', "\"\"")) } else { s.to_string() };
        let mut out = String::new();
        let line = |cells: Vec<String>, out: &mut String| {
            out.push_str(&cells.join(","));
            out.push_str("\r\n");
        };
        line(self.header.iter().map(|h| field(h)).collect(), &mut out);
        for row in &self.rows {
            let cells = row
                .iter()
                .map(|c| match c {
                    Cell::Number(v) => format!("{v}"),
                    Cell::Text(t) => field(t),
                    Cell::Empty => String::new(),
                })
                .collect();
            line(cells, &mut out);
        }
        out
    }

    /// Rows of named cells into a table whose columns are every name that has a value, in the
    /// order first met; a column no row has a value in is left out.
    fn from_named(rows: Vec<Vec<(String, Cell)>>) -> Table {
        let mut header: Vec<String> = Vec::new();
        for row in &rows {
            for (name, c) in row {
                if *c != Cell::Empty && !header.contains(name) {
                    header.push(name.clone());
                }
            }
        }
        let rows = rows
            .into_iter()
            .map(|row| header.iter().map(|h| row.iter().find(|(n, _)| n == h).map_or(Cell::Empty, |(_, c)| c.clone())).collect())
            .collect();
        Table { header, rows }
    }
}

/// A column name with its unit: `ux [mm]`.
pub(super) fn titled(name: &str, unit: Option<&str>) -> String {
    match unit {
        Some(u) => format!("{name} [{u}]"),
        None => name.to_string(),
    }
}

/// The CSV columns of a quantity of a probe's node entry: a vector's components (and a
/// displacement's magnitude), a stress tensor's six components, a scalar under its own name;
/// each with `unit`.
fn quantity_cells(key: &str, v: &Value, unit: Option<&str>, out: &mut Vec<(String, Cell)>) {
    const VECTORS: [(&str, &[&str]); 7] = [
        ("displacement", &["ux", "uy", "uz"]),
        ("rotation", &["rx", "ry", "rz"]),
        ("reaction", &["rfx", "rfy", "rfz"]),
        ("reaction_moment", &["rmx", "rmy", "rmz"]),
        ("velocity", &["vx", "vy", "vz"]),
        ("shape", &["ux", "uy", "uz", "rx", "ry", "rz"]),
        ("stress", &["s_xx", "s_yy", "s_zz", "s_xy", "s_xz", "s_yz"]),
    ];
    match v {
        Value::Number(n) => out.push((titled(key, unit), n.as_f64().map_or(Cell::Empty, Cell::Number))),
        Value::Array(items) if items.iter().all(Value::is_number) => {
            let nums: Vec<f64> = items.iter().filter_map(Value::as_f64).collect();
            let names: Vec<String> = match VECTORS.iter().find(|(k, _)| *k == key) {
                Some((_, n)) if n.len() == nums.len() => n.iter().map(|s| (*s).to_string()).collect(),
                _ => (1..=nums.len()).map(|i| format!("{key}_{i}")).collect(),
            };
            out.extend(names.into_iter().map(|n| titled(&n, unit)).zip(nums.iter().map(|&x| Cell::Number(x))));
            if key == "displacement" && nums.len() == 3 {
                out.push((titled("u_mag", unit), Cell::Number(nums.iter().map(|x| x * x).sum::<f64>().sqrt())));
            }
        }
        _ => {}
    }
}

/// `solve.probe`'s reply (in the display system, with its `units`) as a table: a row per node
/// with its label, undeformed position and the quantities the step has, the units in the header.
fn probe_table(r: &Value, position: &dyn Fn(u32) -> Option<[f64; 3]>, display: &fieldcraft_model::DisplaySystem) -> Result<Table> {
    let length = display.symbol(fieldcraft_model::units::Quantity::Length).to_string();
    let unit_of = |key: &str| r.get("units").and_then(|u| u.get(key)).and_then(Value::as_str).map(str::to_string);
    let nodes = r.get("nodes").and_then(Value::as_array).ok_or_else(|| EngineError::Params("the probe has no node results to export".into()))?;
    let rows = nodes
        .iter()
        .map(|e| {
            let label = e.get("node").and_then(Value::as_u64).and_then(|n| u32::try_from(n).ok());
            let mut row = vec![("node".to_string(), label.map_or(Cell::Empty, |l| Cell::Number(f64::from(l))))];
            if let Some(name) = e.get("reference_point").and_then(Value::as_str) {
                row.push(("reference_point".into(), Cell::Text(name.to_string())));
            }
            let xyz = label.and_then(position);
            for (k, axis) in ["x", "y", "z"].iter().enumerate() {
                row.push((titled(axis, Some(&length)), xyz.and_then(|p| p.get(k).copied()).map_or(Cell::Empty, Cell::Number)));
            }
            if let Some(o) = e.as_object() {
                for (k, v) in o.iter().filter(|(k, _)| !matches!(k.as_str(), "node" | "reference_point" | "xyz" | "distance")) {
                    quantity_cells(k, v, unit_of(k).as_deref(), &mut row);
                }
            }
            row
        })
        .collect();
    Ok(Table::from_named(rows))
}

/// `results.reactions`' reply (in `display`) as a table: a row per set, the total and the
/// applied load, the units in the header.
fn reactions_table(r: &Value, display: &fieldcraft_model::DisplaySystem) -> Table {
    use fieldcraft_model::units::Quantity;
    let (force, moment) = (display.symbol(Quantity::Force).to_string(), display.symbol(Quantity::Moment).to_string());
    let row = |name: &str, e: &Value| {
        let mut row = vec![("set".to_string(), Cell::Text(name.to_string()))];
        row.push(("nodes".into(), e.get("nodes").and_then(Value::as_f64).map_or(Cell::Empty, Cell::Number)));
        for (key, names, unit) in [("force", ["fx", "fy", "fz"], &force), ("moment", ["mx", "my", "mz"], &moment)] {
            let v = e.get(key).and_then(Value::as_array);
            for (k, n) in names.iter().enumerate() {
                row.push((titled(n, Some(unit)), v.and_then(|a| a.get(k)).and_then(Value::as_f64).map_or(Cell::Empty, Cell::Number)));
            }
        }
        row
    };
    let mut rows: Vec<Vec<(String, Cell)>> = r
        .get("sets")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .map(|e| row(e.get("set").and_then(Value::as_str).unwrap_or_default(), e))
        .collect();
    rows.push(row("total", r.get("total").unwrap_or(&Value::Null)));
    if let Some(applied) = r.get("applied").filter(|a| a.is_array()) {
        rows.push(row("applied", &json!({"force": applied})));
    }
    Table::from_named(rows)
}

fn fracture_table(step: &str, domains: &[crate::results::DomainIntegralResult], display: &fieldcraft_model::DisplaySystem) -> Result<Table> {
    use fieldcraft_model::units::Quantity;
    let area_unit = display.symbol(Quantity::Area);
    let j_unit = display.symbol(Quantity::ForcePerLength);
    let p_unit = display.symbol(Quantity::Pressure);
    let (k_scale, k_unit) = crate::units::stress_sqrt_length_scale(crate::units::si(), display);

    let express = |q: Quantity, v: f64| -> Result<Cell> {
        let shown = display.express(q, v).map_err(|e| EngineError::Params(format!("unit conversion error: {e}")))?;
        Ok(if shown.is_finite() { Cell::Number(shown) } else { Cell::Empty })
    };
    let opt_express = |q: Quantity, v: Option<f64>| -> Result<Cell> {
        match v {
            Some(x) => express(q, x),
            None => Ok(Cell::Empty),
        }
    };
    let opt_k = |v: Option<f64>| -> Cell {
        match v.map(|x| x * k_scale) {
            Some(x) if x.is_finite() => Cell::Number(x),
            _ => Cell::Empty,
        }
    };

    let mut rows = Vec::with_capacity(domains.len());
    for d in domains {
        let nodes_str = d.front_nodes.iter().map(u32::to_string).collect::<Vec<_>>().join(" ");
        rows.push(vec![
            ("domain_id".into(), Cell::Text(d.domain_id.clone())),
            ("step".into(), Cell::Text(step.to_string())),
            ("s_position".into(), d.s_position.filter(|v| v.is_finite()).map_or(Cell::Empty, Cell::Number)),
            ("front_nodes".into(), if nodes_str.is_empty() { Cell::Empty } else { Cell::Text(nodes_str) }),
            (titled("area_aq", Some(area_unit)), express(Quantity::Area, d.area_aq)?),
            (titled("j_mean", Some(j_unit)), express(Quantity::ForcePerLength, d.j_mean)?),
            (titled("j_min", Some(j_unit)), express(Quantity::ForcePerLength, d.j_min)?),
            (titled("j_max", Some(j_unit)), express(Quantity::ForcePerLength, d.j_max)?),
            ("ring_spread".into(), if d.ring_spread.is_finite() { Cell::Number(d.ring_spread) } else { Cell::Empty }),
            (titled("j_total", Some(j_unit)), opt_express(Quantity::ForcePerLength, d.j_total)?),
            (titled("k_j_plane_strain", Some(&k_unit)), opt_k(d.k_j_plane_strain)),
            (titled("k_j_plane_stress", Some(&k_unit)), opt_k(d.k_j_plane_stress)),
            (titled("k_i_mean", Some(&k_unit)), opt_k(d.k_i_mean)),
            (titled("k_ii_mean", Some(&k_unit)), opt_k(d.k_ii_mean)),
            (titled("k_iii_mean", Some(&k_unit)), opt_k(d.k_iii_mean)),
            (titled("t_11_mean", Some(p_unit)), opt_express(Quantity::Pressure, d.t_11_mean)?),
            (titled("t_33_mean", Some(p_unit)), opt_express(Quantity::Pressure, d.t_33_mean)?),
            ("normal_x".into(), Cell::Number(d.normal[0])),
            ("normal_y".into(), Cell::Number(d.normal[1])),
            ("normal_z".into(), Cell::Number(d.normal[2])),
            ("tangent_x".into(), Cell::Number(d.tangent[0])),
            ("tangent_y".into(), Cell::Number(d.tangent[1])),
            ("tangent_z".into(), Cell::Number(d.tangent[2])),
            ("advance_x".into(), Cell::Number(d.advance[0])),
            ("advance_y".into(), Cell::Number(d.advance[1])),
            ("advance_z".into(), Cell::Number(d.advance[2])),
            ("warp_step".into(), Cell::Number(d.step as f64)),
            (
                "interaction_stress_state".into(),
                d.interaction_stress_state.map_or(Cell::Empty, |st| {
                    Cell::Text(
                        match st {
                            fieldcraft_warp3d::InteractionStressState::PlaneStrain => "plane_strain",
                            fieldcraft_warp3d::InteractionStressState::PlaneStress => "plane_stress",
                        }
                        .to_string(),
                    )
                }),
            ),
        ]);
    }
    Ok(Table::from_named(rows))
}

fn read(s: &mut Session, p: &Value) -> Result<Value> {
    let format: &str = super::req(p, "format")?;
    if format != "warp3d" {
        return Err(EngineError::invalid("format", "only `warp3d` is supported by results.read"));
    }
    let path = super::file_param(s, p, "path", fieldcraft_io::roots::Access::Read)?;
    let step = opt::<String>(p, "step")?;
    let units = match opt::<String>(p, "units")? {
        Some(label) => Some(
            fieldcraft_model::units::Preset::from_label(&label)
                .ok_or_else(|| EngineError::invalid("units", "choose a known consistent unit preset"))?,
        ),
        None => None,
    };
    let sol =
        fieldcraft_warp3d::read_results_dir(&s.model, step.as_deref().unwrap_or(""), std::path::Path::new(&path), units).map_err(|e| match e {
            fieldcraft_warp3d::Warp3dError::Model(m) => EngineError::Model(m),
            other => EngineError::Params(format!("cannot read WARP3D results: {other}")),
        })?;
    let step_name = sol.static_result.step.clone();
    let mut summary = if sol.static_result.nonlinear.is_some() {
        super::solve::nonlinear_summary(&sol.static_result, s.units())
    } else {
        super::solve::static_summary(&sol.static_result, s.units())
    };
    summary["format"] = json!("warp3d");
    summary["path"] = json!(path);
    if !sol.fracture.is_empty() {
        summary["fracture"] = json!(&sol.fracture);
    }
    s.store_result(crate::StepResult::Implicit(fieldcraft_solver::StepResult::Static(sol.static_result)));
    s.store_fracture_results(step_name, sol.fracture, sol.step_numbers, sol.domains_truncated, sol.step1_is_elastic, None);
    Ok(summary)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cmd::solve::tests::cantilever;

    /// The numbers of the first `DataArray` called `name` in a written `.vtu`.
    fn vtu_array(xml: &str, name: &str) -> Vec<f64> {
        let at = xml.find(&format!("Name=\"{name}\"")).unwrap_or_else(|| panic!("no array {name}"));
        let start = at + xml[at..].find('>').unwrap() + 1;
        let end = start + xml[start..].find("</DataArray>").unwrap();
        xml[start..end].split_whitespace().map(|t| t.parse().unwrap()).collect()
    }

    fn scratch(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("fieldcraft-vtu-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn export_vtu_writes_a_static_step_and_its_modes() {
        let dir = scratch("static");
        let out = dir.join("out.vtu").to_string_lossy().into_owned();
        let mut s = Session::default();
        s.execute("file.sample", &json!({"name": "cantilever"})).unwrap();
        let e = s.execute("results.export.vtu", &json!({"step": "Load", "path": out})).unwrap_err().to_string();
        assert!(e.contains("Needs a solved step"), "{e}");
        s.execute("solve.static", &json!({"step": "Load"})).unwrap();
        let reply = s.execute("results.export.vtu", &json!({"step": "Load", "path": out})).unwrap();
        let xml = std::fs::read_to_string(&out).unwrap();
        assert_eq!(reply["points"], s.model.nodes().len());
        assert_eq!(reply["cells"], s.model.elements().len());
        assert_eq!(reply["bytes"], xml.len());
        assert!(xml.contains(&format!("NumberOfPoints=\"{}\"", s.model.nodes().len())));

        // The tip node's displacement is the probe's, at the node's point.
        let labels = vtu_array(&xml, "node_label");
        let u = vtu_array(&xml, "displacement");
        let r = s.result("Load").unwrap().clone();
        let Kind::Static(st) = r.kind() else { panic!() };
        let (k, tip) = st.displacement.iter().enumerate().max_by(|a, b| a.1[2].abs().total_cmp(&b.1[2].abs())).unwrap();
        let tip_label = st.nodes[k];
        let probe = s.execute("solve.probe", &json!({"step": "Load", "nodes": [tip_label]})).unwrap();
        let i = labels.iter().position(|&l| l == f64::from(tip_label)).unwrap();
        // The file is in the display system (mm, MPa), as the probe is, and says so.
        let display = s.units().clone();
        assert!(xml.contains("<!-- units: Points mm; displacement mm;") && xml.contains("stress MPa"), "{}", &xml[..200]);
        let tip = crate::results::shown_all(crate::results::Quantity::Length, *tip, &display);
        assert_eq!(&u[3 * i..3 * i + 3], tip.as_slice());
        assert_eq!(
            probe["nodes"][0]["displacement"].as_array().map(|a| a.iter().filter_map(Value::as_f64).collect::<Vec<_>>()),
            Some(tip.to_vec()),
            "{probe}"
        );
        // Stress components in VTK order: XY at 3, YZ at 4, XZ at 5.
        let stress = vtu_array(&xml, "stress");
        let t = crate::results::shown_all(crate::results::Quantity::Pressure, st.stress[k], &display);
        assert_eq!(&stress[6 * i..6 * i + 6], [t[0], t[1], t[2], t[3], t[5], t[4]].as_slice());
        assert_eq!(vtu_array(&xml, "has_result").iter().filter(|&&h| h == 1.0).count(), st.nodes.len());
        // Deterministic.
        s.execute("results.export.vtu", &json!({"step": "Load", "path": out})).unwrap();
        assert_eq!(std::fs::read_to_string(&out).unwrap(), xml);

        // Modes: one file per mode with its frequency, and a collection.
        s.execute("solve.frequency", &json!({"step": "Modes"})).unwrap();
        let pvd = dir.join("modes.pvd").to_string_lossy().into_owned();
        let reply = s.execute("results.export.vtu", &json!({"step": "Modes", "path": pvd})).unwrap();
        let files = reply["files"].as_array().unwrap();
        assert_eq!(files.len(), 7, "six modes and the collection: {reply}");
        let mode2 = std::fs::read_to_string(dir.join("modes_0001.vtu")).unwrap();
        assert_eq!(vtu_array(&mode2, "mode"), [2.0]);
        let collection = std::fs::read_to_string(&pvd).unwrap();
        assert!(collection.contains("timestep=\"2.0\" part=\"0\" file=\"modes_0001.vtu\""), "{collection}");
        let one = dir.join("mode3.vtu").to_string_lossy().into_owned();
        s.execute("results.export.vtu", &json!({"step": "Modes", "path": one, "mode": 3, "encoding": "binary"})).unwrap();
        assert!(std::fs::read_to_string(&one).unwrap().contains("format=\"binary\""));

        // Refusals write nothing.
        let none = dir.join("none.vtu").to_string_lossy().into_owned();
        for (p, why) in [
            (json!({"step": "Modes", "path": none, "mode": 0}), "modes 1 to 6"),
            (json!({"step": "Modes", "path": none, "mode": 1_000_000_000}), "modes 1 to 6"),
            (json!({"step": "Load", "path": none, "mode": 1}), "frequency"),
            (json!({"step": "Load", "path": dir.join("x.vtk").to_string_lossy()}), ".vtu"),
            (json!({"step": "Load", "path": none, "frames": 3}), "frames"),
            (json!({"step": "Load", "path": none, "encoding": "hex"}), "encoding"),
            (json!({"step": "Nope", "path": none}), "has no result"),
        ] {
            let e = s.execute("results.export.vtu", &p).unwrap_err().to_string();
            assert!(e.contains(why), "{p}: {e}");
        }
        s.execute("mesh.node.add", &json!({"label": 9999, "xyz": [50, 50, 50]})).unwrap();
        let e = s.execute("results.export.vtu", &json!({"step": "Load", "path": none})).unwrap_err().to_string();
        assert!(e.contains("the model changed since step `Load` was solved"), "{e}");
        assert!(!dir.join("none.vtu").exists() && !dir.join("x.vtk").exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn export_vtu_writes_nonlinear_frames_and_heat() {
        let dir = scratch("nl-heat");
        let mut s = Session::default();
        cantilever(&mut s);
        s.run_script(&json!([
            {"command": "model.section.edit", "params": {"name": "bar", "set": "all", "material": "Mat", "kind": {"type": "solid"}, "formulation": {}}},
            {"command": "model.step.create", "params": {"name": "Large", "procedure": {"type": "nonlinear_static", "initial_increment": 0.25},
                "boundary_conditions": ["clamp"], "loads": ["push"]}},
            {"command": "model.material.edit", "params": {"name": "Mat", "elastic": {"type": "isotropic", "young": 1000, "poisson": 0}, "density": 1, "thermal": {"conductivity": 2}}},
            {"command": "model.boundary_condition.create", "params": {"name": "Heater", "type": "temperature", "target": {"set": "root"}, "value": 100}},
            {"command": "model.set.create", "params": {"name": "skin", "faces": (1..=6).map(|f| json!({"element": 10, "face": f})).collect::<Vec<_>>()}},
            {"command": "model.load.create", "params": {"name": "Air", "type": "convection", "set": "skin", "film": 0.5, "sink": 20}},
            {"command": "model.step.create", "params": {"name": "Therm", "procedure": {"type": "heat_transfer"}, "boundary_conditions": ["Heater"], "loads": ["Air"]}},
        ]))
        .unwrap();
        s.execute("solve.nonlinear", &json!({"step": "Large"})).unwrap();
        s.execute("solve.heat", &json!({"step": "Therm"})).unwrap();

        let pvd = dir.join("large.pvd").to_string_lossy().into_owned();
        let reply = s.execute("results.export.vtu", &json!({"step": "Large", "path": pvd})).unwrap();
        let Kind::Static(st) = s.result("Large").unwrap().kind() else { panic!() };
        let h = st.nonlinear.as_ref().unwrap();
        assert_eq!(reply["files"].as_array().unwrap().len(), h.frames.len() + 1);
        let first = std::fs::read_to_string(dir.join("large_0000.vtu")).unwrap();
        let last = std::fs::read_to_string(dir.join(format!("large_{:04}.vtu", h.frames.len() - 1))).unwrap();
        assert!(first.contains("Name=\"displacement\"") && !first.contains("Name=\"reaction\""), "the final fields on the last frame only");
        // Each frame's own stress (#205): none unloaded, the frame's at an increment.
        assert!(vtu_array(&first, "von_mises").iter().all(|v| *v == 0.0));
        let second = std::fs::read_to_string(dir.join("large_0001.vtu")).unwrap();
        let largest = |xml: &str| vtu_array(xml, "von_mises").into_iter().fold(0.0, f64::max);
        assert!(largest(&second) > 0.0 && largest(&second) < largest(&last), "{} of {}", largest(&second), largest(&last));
        assert!(last.contains("Name=\"stress\"") && last.contains("Name=\"von_mises\""));
        assert_eq!(vtu_array(&last, "load_factor"), [h.frames.last().unwrap().load_factor]);

        let heat = dir.join("heat.vtu").to_string_lossy().into_owned();
        s.execute("results.export.vtu", &json!({"step": "Therm", "path": heat})).unwrap();
        let xml = std::fs::read_to_string(&heat).unwrap();
        let t = vtu_array(&xml, "temperature");
        assert!(t.contains(&100.0), "the heater's temperature");
        assert_eq!(vtu_array(&xml, "heat_flux").len(), 3 * s.model.nodes().len());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn export_vtu_writes_explicit_frames_as_a_series() {
        let dir = scratch("impact");
        let mut s = Session::default();
        s.execute("file.sample", &json!({"name": "impact"})).unwrap();
        s.execute("solve.explicit", &json!({})).unwrap();
        let pvd = dir.join("impact.pvd").to_string_lossy().into_owned();
        let reply = s.execute("results.export.vtu", &json!({"path": pvd})).unwrap();
        let Kind::Explicit(e) = s.result("Impact").unwrap().kind() else { panic!() };
        let frames = e.frames.len();
        assert!(frames > 2);
        assert_eq!(reply["files"].as_array().unwrap().len(), frames + 1);
        let collection = std::fs::read_to_string(&pvd).unwrap();
        let times: Vec<f64> =
            collection.match_indices("timestep=\"").map(|(i, m)| collection[i + m.len()..].split('"').next().unwrap().parse().unwrap()).collect();
        assert_eq!(times.len(), frames);
        assert!(times.windows(2).all(|w| w[0] < w[1]), "{times:?}");
        let last = std::fs::read_to_string(dir.join(format!("impact_{:04}.vtu", frames - 1))).unwrap();
        let cell = &last[last.find("<CellData>").unwrap()..last.find("</CellData>").unwrap()];
        assert!(cell.contains("Name=\"stress\" NumberOfComponents=\"6\""), "{cell}");
        // A .vtu takes the last frame; `frames: "last"` makes a one-entry series.
        let one = dir.join("last.vtu").to_string_lossy().into_owned();
        s.execute("results.export.vtu", &json!({"path": one})).unwrap();
        assert_eq!(std::fs::read_to_string(&one).unwrap(), last);
        let lastp = dir.join("last.pvd").to_string_lossy().into_owned();
        assert_eq!(s.execute("results.export.vtu", &json!({"path": lastp, "frames": "last"})).unwrap()["files"].as_array().unwrap().len(), 2);
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn solved() -> Session {
        let mut s = Session::default();
        cantilever(&mut s);
        s.execute("solve.static", &json!({})).unwrap();
        s.execute("solve.frequency", &json!({})).unwrap();
        s
    }

    /// `range` clamps the legend (#191): kept with the field, cleared by "auto" or another
    /// field, reported in the reply; the true range is still there.
    /// A CSV reader (RFC 4180) for the tests: rows of fields, quotes undone.
    fn read_csv(text: &str) -> Vec<Vec<String>> {
        let (mut rows, mut row, mut field, mut quoted, mut chars) = (Vec::new(), Vec::new(), String::new(), false, text.chars().peekable());
        while let Some(c) = chars.next() {
            match (quoted, c) {
                (true, '"') if chars.peek() == Some(&'"') => {
                    chars.next();
                    field.push('"');
                }
                (true, '"') => quoted = false,
                (true, c) => field.push(c),
                (false, '"') => quoted = true,
                (false, ',') => row.push(std::mem::take(&mut field)),
                (false, '\r') => {}
                (false, '\n') => {
                    row.push(std::mem::take(&mut field));
                    rows.push(std::mem::take(&mut row));
                }
                (false, c) => field.push(c),
            }
        }
        rows
    }

    fn temp_csv(tag: &str) -> String {
        std::env::temp_dir().join(format!("fieldcraft-csv-{tag}-{}.csv", std::process::id())).to_string_lossy().into_owned()
    }

    /// `results.export.csv` writes what results.reactions and solve.probe reply, bit for bit
    /// after parsing; a set name with a comma and a quote round-trips (#92).
    #[test]
    fn results_export_as_csv() {
        let mut s = solved();
        let path = temp_csv("reactions");
        let sets = json!(["root", "tip"]);
        let reply = s.execute("results.export.csv", &json!({"what": "reactions", "path": path, "step": "Bend", "sets": sets})).unwrap();
        let sums = s.execute("results.reactions", &json!({"step": "Bend", "sets": sets})).unwrap();
        let rows = read_csv(&std::fs::read_to_string(&path).unwrap());
        assert_eq!(rows[0], ["set", "nodes", "fx [N]", "fy [N]", "fz [N]", "mx [N·mm]", "my [N·mm]", "mz [N·mm]"]);
        assert_eq!((reply["rows"].as_u64(), reply["step"].as_str()), (Some(4), Some("Bend")), "{reply}");
        // A name with a comma and a quote round-trips through a CSV reader.
        let named =
            Table { header: vec!["set".into(), "fx".into()], rows: vec![vec![Cell::Text("the \"root\", clamped".into()), Cell::Number(0.1)]] };
        assert_eq!(read_csv(&named.csv()), [vec!["set", "fx"], vec!["the \"root\", clamped", "0.1"]]);
        let total = rows.iter().find(|r| r[0] == "total").unwrap();
        assert_eq!(total[4].parse::<f64>().unwrap(), sums["total"]["force"][2].as_f64().unwrap(), "bit for bit");
        let applied = rows.iter().find(|r| r[0] == "applied").unwrap();
        assert_eq!((applied[1].as_str(), applied[5].as_str()), ("", ""), "the applied load has no nodes or moments");
        let path = temp_csv("probe");
        s.execute("results.export.csv", &json!({"what": "probe", "path": path, "step": "Bend", "nodes": [1, 2, 3]})).unwrap();
        let probe = s.execute("solve.probe", &json!({"step": "Bend", "nodes": [1, 2, 3]})).unwrap();
        let rows = read_csv(&std::fs::read_to_string(&path).unwrap());
        assert_eq!(rows.len(), 4);
        let col = |name: &str| rows[0].iter().position(|h| h == name).unwrap_or_else(|| panic!("no {name}: {:?}", rows[0]));
        for (i, node) in probe["nodes"].as_array().unwrap().iter().enumerate() {
            assert_eq!(rows[i + 1][col("node")], node["node"].to_string());
            assert_eq!(rows[i + 1][col("uz [mm]")].parse::<f64>().unwrap(), node["displacement"][2].as_f64().unwrap());
            assert_eq!(rows[i + 1][col("s_yz [MPa]")].parse::<f64>().unwrap(), node["stress"][5].as_f64().unwrap());
            assert_eq!(rows[i + 1][col("von_mises [MPa]")].parse::<f64>().unwrap(), node["von_mises"].as_f64().unwrap());
        }
        assert!(!rows[0].iter().any(|h| h == "temperature"), "columns the step does not have are left out: {:?}", rows[0]);
        // A stale result is refused.
        // Every parameter the description names is taken: a set's nodes, points, a moment point.
        let set = s.execute("results.export.csv", &json!({"what": "probe", "path": path, "step": "Bend", "set": "tip"})).unwrap();
        assert!(set["rows"].as_u64().unwrap_or(0) > 0, "{set}");
        s.execute("results.export.csv", &json!({"what": "probe", "path": path, "step": "Bend", "at": [[0, 0, 0]]})).unwrap();
        s.execute("results.export.csv", &json!({"what": "reactions", "path": path, "step": "Bend", "about": [1, 0, 0]})).unwrap();
        s.execute("mesh.node.add", &json!({"label": 99_999, "xyz": [0, 0, 0]})).unwrap();
        assert!(s.execute("results.export.csv", &json!({"what": "reactions", "path": temp_csv("stale"), "step": "Bend"})).is_err());
        for p in ["reactions", "probe"] {
            let _ = std::fs::remove_file(temp_csv(p));
        }
    }

    #[test]
    fn the_legend_range_can_be_clamped() {
        let mut s = solved();
        let r = s.execute("results.contour", &json!({"field": "von_mises", "range": {"max": 10}})).unwrap();
        let top = r["contour"]["range"]["max"]["value"].as_f64().unwrap();
        assert!(top > 10.0, "{r}");
        assert_eq!(r["contour"]["legend"], json!({"min": null, "max": 10.0}), "{r}");
        // The view keeps its range in SI: 10 MPa.
        let pa = |mpa: f64| fieldcraft_model::DisplaySystem::default().to_si(crate::results::Quantity::Pressure, mpa).unwrap();
        let v = s.results_view.clone().unwrap();
        let b = v.bands(0.0, pa(top));
        assert!(b.over && b.max == pa(10.0) && b.band(pa(top)) == Some(b.count));
        s.execute("results.contour", &json!({"field": "von_mises", "bands": 6})).unwrap();
        assert_eq!(s.results_view.as_ref().unwrap().range_max, Some(pa(10.0)), "kept with the field");
        s.execute("results.contour", &json!({"field": "von_mises", "range": {"max": "auto", "min": "2 MPa"}})).unwrap();
        assert_eq!(s.results_view.as_ref().map(|v| (v.range_min, v.range_max)), Some((Some(pa(2.0)), None)));
        s.execute("results.contour", &json!({"field": "displacement"})).unwrap();
        assert_eq!(s.results_view.as_ref().map(|v| (v.range_min, v.range_max)), Some((None, None)), "another field follows its own range");
        s.execute("results.contour", &json!({"field": "displacement", "range": {"max": 1e-3}})).unwrap();
        let r = s.execute("results.contour", &json!({"field": "displacement", "range": "auto"})).unwrap();
        assert!(r["contour"].get("legend").is_none(), "{r}");
    }

    #[test]
    fn the_display_follows_the_commands_and_is_not_an_edit() {
        let mut s = solved();
        let (journal, revision) = (s.journal.len(), s.revision());
        // Solving showed the first solved step, a static one: contours on the undeformed shape.
        assert_eq!(s.results_view.as_ref().map(|v| (v.step.as_str(), v.contour, v.deformed)), Some(("Bend", Some(Field::VonMises), false)));
        s.execute("results.clear", &json!({})).unwrap();
        let r = s.execute("results.contour", &json!({"field": "von_mises"})).unwrap();
        assert_eq!(r["view"]["step"], "Bend", "{r}");
        assert_eq!(r["contour"]["field"], "von_mises");
        assert!(r["contour"]["range"]["max"]["value"].as_f64().unwrap() > 0.0, "{r}");
        let r = s.execute("results.deformed", &json!({"scale": 2.5})).unwrap();
        assert_eq!((r["view"]["deformed"].as_bool(), r["view"]["scale"].as_f64()), (Some(true), Some(2.5)));
        // Scale 0: the undeformed shape, still the deformed view (an animation's first picture).
        let r = s.execute("results.deformed", &json!({"scale": 0})).unwrap();
        assert_eq!((r["view"]["deformed"].as_bool(), r["view"]["scale"].as_f64()), (Some(true), Some(0.0)));
        s.execute("results.deformed", &json!({"scale": "auto", "show": false})).unwrap();
        assert_eq!(s.results_view.as_ref().map(|v| (v.deformed, v.scale)), Some((false, None)));
        // Modes switch to the frequency step (its own first look) and list the frequencies.
        let r = s.execute("results.modes", &json!({"mode": 3})).unwrap();
        assert_eq!((r["view"]["step"].as_str(), r["view"]["mode"].as_u64()), (Some("Modes"), Some(3)));
        assert_eq!(r["modes"].as_array().map(Vec::len), Some(4));
        assert_eq!(s.results_view.as_ref().map(|v| (v.contour, v.deformed)), Some((Some(Field::Displacement), true)), "a mode shape is deformed");
        assert!(s.execute("results.contour", &json!({"field": "von_mises"})).is_err());
        s.execute("results.contour", &json!({"field": "displacement", "bands": 6})).unwrap();
        s.execute("results.animate", &json!({})).unwrap();
        // Changing the field keeps the deformation, its scale and the animation (#332).
        s.execute("results.deformed", &json!({"scale": 3})).unwrap();
        let r = s.execute("results.contour", &json!({"step": "Modes", "field": "displacement_z"})).unwrap();
        assert_eq!(
            (r["view"]["deformed"].as_bool(), r["view"]["scale"].as_f64(), r["view"]["animate"].as_bool()),
            (Some(true), Some(3.0), Some(true))
        );
        assert!(s.results_view.as_ref().is_some_and(|v| v.animate && v.bands == 6));
        s.execute("results.animate", &json!({"on": false})).unwrap();
        assert!(s.results_view.as_ref().is_some_and(|v| !v.animate));
        assert_eq!((s.journal.len(), s.revision()), (journal, revision), "display commands are not edits");
        // An edit makes the results stale; the display keeps showing them, marked.
        s.execute("model.material.edit", &json!({"name": "Mat", "elastic": {"type": "isotropic", "young": 2000, "poisson": 0}, "density": 1}))
            .unwrap();
        assert_eq!(s.results_report()["stale"], true);
        s.execute("results.clear", &json!({})).unwrap();
        assert!(s.results_view.is_none());
        assert!(s.execute("edit.undo", &json!({})).is_ok());
    }

    #[test]
    fn reactions_balance_the_load() {
        let mut s = solved();
        let r = s.execute("results.reactions", &json!({})).unwrap();
        // The clamp holds the whole downward tip force of 1 and its moment: about the origin the
        // tip force −1 z at x = 10 turns +10 about y, so the reactions turn −10.
        let root = &r["sets"][0];
        assert_eq!((root["set"].as_str(), root["nodes"].as_u64()), (Some("root"), Some(4)), "{r}");
        assert!((root["force"][2].as_f64().unwrap() - 1.0).abs() < 1e-9, "{r}");
        assert!((root["moment"][1].as_f64().unwrap() + 10.0).abs() < 1e-6, "{r}");
        assert!((r["total"]["force"][2].as_f64().unwrap() - 1.0).abs() < 1e-9);
        // About a point on the tip's line of action the moment about y vanishes; the tip set
        // has no reactions.
        let r = s.execute("results.reactions", &json!({"sets": ["root", "tip"], "about": [10, 0, 0]})).unwrap();
        assert!(r["sets"][0]["moment"][1].as_f64().unwrap().abs() < 1e-6, "{r}");
        assert_eq!(r["sets"][1]["force"][2].as_f64(), Some(0.0));
        let empty = s.execute("results.reactions", &json!({"sets": []})).unwrap();
        assert_eq!(empty["sets"][0]["set"], "root", "an empty list sums the step's boundary-condition sets: {empty}");
        assert!(s.execute("results.reactions", &json!({"step": "Modes"})).is_err());
        assert!(s.execute("results.reactions", &json!({"sets": ["nope"]})).is_err());
    }

    #[test]
    fn explicit_frames_drive_the_display() {
        let mut s = Session::default();
        s.run_script(&crate::sample::impact()).unwrap();
        s.execute("solve.explicit", &json!({})).unwrap();
        // The first look: the last frame, true scale, von Mises averaged from the elements.
        let r = s.results_report();
        assert_eq!((r["view"]["step"].as_str(), r["view"]["scale"].as_f64(), r["view"]["frame"].is_null()), (Some("Impact"), Some(1.0), true), "{r}");
        assert_eq!(r["frames"].as_array().map(Vec::len), Some(11));
        assert!(r["contour"]["range"]["max"]["value"].as_f64().unwrap() > 0.0, "{r}");
        // A frame by time: the nearest one.
        let r = s.execute("results.frame", &json!({"time": 1.3e-5})).unwrap();
        assert_eq!(r["view"]["frame"], 3, "{r}");
        assert!((r["time"].as_f64().unwrap() - 1.3e-5).abs() < 1e-6, "frames land on increments: {r}");
        // At frame 0 the bar has not hit yet: no stress.
        let r = s.execute("results.frame", &json!({"frame": 0})).unwrap();
        assert!(r["contour"]["range"]["max"]["value"].as_f64().unwrap().abs() < 1e-6, "{r}");
        s.execute("results.contour", &json!({"field": "displacement_x"})).unwrap();
        // Frames keep velocities: every node starts at 10 m/s.
        let r = s.execute("results.contour", &json!({"field": "velocity"})).unwrap();
        assert!((r["contour"]["range"]["min"]["value"].as_f64().unwrap() - 10000.0).abs() < 1e-6, "{r}");
        assert!(s.execute("results.frame", &json!({"frame": 11})).is_err());
        assert!(s.execute("results.modes", &json!({})).is_err());
        assert!(s.execute("results.reactions", &json!({})).is_err());
    }

    /// A session showing a synthetic fluid result ([`crate::flow::tests::channel`]).
    fn fluid() -> Session {
        // The synthetic result is in SI, and shown so.
        let mut s = Session { default_units: crate::units::si().clone(), ..Session::default() };
        s.store_result(crate::StepResult::Fluid(Box::new(crate::flow::tests::channel())));
        s
    }

    /// A fluid step is shown with velocity on a slice across the flow, its obstacles coloured
    /// and streamlines from the inlet (#193); the commands change what is shown, not the model.
    #[test]
    fn fluid_steps_show_a_slice_the_obstacles_and_streamlines() {
        let mut s = fluid();
        let (journal, revision) = (s.journal.len(), s.revision());
        let r = s.results_report();
        assert_eq!(r["view"]["contour"], "velocity", "{r}");
        assert_eq!(r["contour"]["range"]["max"]["value"], 10.0, "{r}");
        assert_eq!(r["contour"]["range"]["max"]["at"], json!([9.5, 0.5, 0.5]), "{r}");
        assert_eq!(r["slice"], json!({"axis": "y", "position": 2.0}));
        assert_eq!((r["surface"].as_bool(), r["streamlines"]["inlet"].as_str()), (Some(true), Some("x_min")), "{r}");
        // A slice normal to z at 1.5 coloured by pressure; "auto" goes back to the middle, a new
        // axis starts there; positions outside the domain are refused; "none" hides it.
        let r = s.execute("results.slice", &json!({"axis": "z", "position": 1.5, "field": "pressure"})).unwrap();
        assert_eq!((r["slice"].clone(), r["contour"]["field"].as_str()), (json!({"axis": "z", "position": 1.5}), Some("pressure")), "{r}");
        assert_eq!(s.execute("results.slice", &json!({"position": "auto"})).unwrap()["slice"]["position"], 2.0);
        assert_eq!(s.execute("results.slice", &json!({"axis": "x"})).unwrap()["slice"], json!({"axis": "x", "position": 5.0}));
        let e = s.execute("results.slice", &json!({"position": 11})).unwrap_err().to_string();
        assert!(e.contains("0 to 10"), "{e}");
        assert!(s.execute("results.slice", &json!({"axis": "none"})).unwrap()["slice"].is_null());
        // results.contour reaches the same fields; nodal ones are refused.
        s.execute("results.contour", &json!({"field": "velocity", "bands": 6})).unwrap();
        assert!(s.execute("results.contour", &json!({"field": "von_mises"})).is_err());
        // Frames, by results.frame or with the slice.
        assert_eq!(s.execute("results.frame", &json!({"frame": 0})).unwrap()["time"], 0.0);
        assert_eq!(s.execute("results.slice", &json!({"frame": 1, "axis": "y"})).unwrap()["time"], 1.0);
        // Streamlines from the inlet, from seeds, along a line; hidden.
        let r = s.execute("results.streamlines", &json!({"count": 4})).unwrap();
        assert_eq!((r["traced"]["seeds"].as_u64(), r["traced"]["streamlines"].as_u64()), (Some(4), Some(4)), "{r}");
        // The reply's trace is the one the plot draws.
        let (view, result) = (s.results_view.as_ref().unwrap(), s.latest_result("Flow").unwrap().0);
        let p = crate::flow::plot(view, result, &s.traced).unwrap();
        let sl = view.flow.as_ref().and_then(|f| f.streamlines.as_ref()).unwrap();
        assert!(std::sync::Arc::ptr_eq(&p.streamlines, &s.traced.trace(result, 1, sl).unwrap().1));
        let r = s.execute("results.streamlines", &json!({"seeds": [[0.5, 0.5, 0.5], [5.5, 1.5, 1.5], [50, 0, 0]]})).unwrap();
        assert_eq!((r["traced"]["streamlines"].as_u64(), r["traced"]["no_start"].as_u64()), (Some(1), Some(2)), "{r}");
        assert_eq!(r["streamlines"]["seeds"], 3);
        // #590: the plot (and the results panel) counts the seeds used, not the default `count`.
        let (view, result) = (s.results_view.as_ref().unwrap(), s.latest_result("Flow").unwrap().0);
        assert_eq!(crate::flow::plot(view, result, &s.traced).unwrap().seeds, 3);
        let r =
            s.execute("results.streamlines", &json!({"line": {"from": [0.5, 0.5, 0.5], "to": [0.5, 3.5, 3.5]}, "count": 5, "max_steps": 3})).unwrap();
        assert_eq!((r["traced"]["seeds"].as_u64(), r["traced"]["points"].as_u64()), (Some(5), Some(20)), "{r}");
        assert!(s.execute("results.streamlines", &json!({"show": false})).unwrap()["streamlines"].is_null());
        assert_eq!(s.execute("results.surface", &json!({"show": false})).unwrap()["surface"], false);
        // A fluid step animates its frames (the start-up transient), not a deformation.
        let r = s.execute("results.animate", &json!({})).unwrap();
        assert_eq!((r["view"]["animate"].as_bool(), r["view"]["deformed"].as_bool()), (Some(true), Some(false)), "{r}");
        assert!(s.execute("results.animate", &json!({"on": false})).unwrap()["view"]["animate"] == json!(false));
        assert_eq!((s.journal.len(), s.revision()), (journal, revision), "display commands are not edits");
        // Structural steps have none of it.
        let mut t = solved();
        for id in ["results.slice", "results.surface", "results.streamlines"] {
            assert!(t.execute(id, &json!({})).is_err(), "{id}");
            assert!(t.execute(id, &json!({"step": "Bend"})).is_err(), "{id}");
        }
    }

    #[test]
    fn temperature_views_require_thermal_frames_and_follow_the_selected_frame() {
        let mut r = crate::flow::tests::channel();
        r.frames[0].temperature = vec![20.0; r.grid.len()];
        r.frames[1].temperature = vec![70.0; r.grid.len()];
        // The synthetic result is in SI (70 K), and shown so.
        let mut s = Session { default_units: crate::units::si().clone(), ..Session::default() };
        s.store_result(crate::StepResult::Fluid(Box::new(r)));
        for command in ["results.slice", "results.surface", "results.streamlines", "results.contour"] {
            let report = s.execute(command, &json!({"field": "temperature"})).unwrap();
            assert_eq!(report["contour"]["field"], "temperature", "{command}: {report}");
            assert_eq!(report["contour"]["range"]["max"]["value"], 70.0);
        }
        let report = s.execute("results.frame", &json!({"frame": 0})).unwrap();
        assert_eq!(report["contour"]["range"]["max"]["value"], 20.0);
        assert!(Field::available(s.latest_result("Flow").unwrap().0).any(|f| f == Field::Temperature));
        let before = s.results_view.clone();
        for command in ["results.slice", "results.surface", "results.streamlines"] {
            for params in [json!({"field": "temperature", "frame": u64::MAX}), json!({"field": ["temperature"]})] {
                assert!(s.execute(command, &params).is_err());
                assert_eq!(s.results_view, before);
            }
        }
        let mut incomplete = crate::flow::tests::channel();
        incomplete.frames[0].temperature = vec![20.0; incomplete.grid.len()];
        incomplete.frames[1].temperature = vec![70.0; incomplete.grid.len().saturating_sub(1)];
        let incomplete = crate::StepResult::Fluid(Box::new(incomplete));
        assert!(!Field::available(&incomplete).any(|f| f == Field::Temperature));
        let mut cold = fluid();
        let before = cold.results_view.clone();
        for command in ["results.slice", "results.surface", "results.streamlines", "results.contour"] {
            let error = cold.execute(command, &json!({"field": "temperature"})).unwrap_err().to_string();
            assert!(error.contains("isothermal"), "{command}: {error}");
            assert_eq!(cold.results_view, before);
        }
        assert!(!Field::available(cold.latest_result("Flow").unwrap().0).any(|f| f == Field::Temperature));
    }

    #[test]
    fn fluid_display_hostile_params_never_panic() {
        let s = fluid();
        let point = json!([0.5, 0.5, 0.5]);
        let hostile = [
            json!(null),
            json!([]),
            json!({"step": 7}),
            json!({"step": "Nope"}),
            json!({"field": "nope"}),
            json!({"field": 3}),
            json!({"axis": "w"}),
            json!({"axis": 1}),
            json!({"axis": ""}),
            json!({"position": 1e308}),
            json!({"position": -1e308}),
            json!({"position": "middle"}),
            json!({"position": [1]}),
            json!({"axis": "z", "position": -0.001}),
            json!({"frame": -1}),
            json!({"frame": 2}),
            json!({"frame": 1e300}),
            json!({"frame": "last"}),
            json!({"show": "yes"}),
            json!({"count": 0}),
            json!({"count": 2001}),
            json!({"count": -5}),
            json!({"count": 1e300}),
            json!({"count": "many"}),
            json!({"max_steps": 0}),
            json!({"max_steps": 100_001}),
            json!({"max_steps": u64::MAX}),
            json!({"seeds": []}),
            json!({"seeds": [[1, 2]]}),
            json!({"seeds": [[1e300, 0, 0]]}),
            json!({"seeds": [[1, "x", 0]]}),
            json!({"seeds": "here"}),
            json!({"seeds": vec![point.clone(); 2001]}),
            json!({"seeds": [point], "line": {"from": [0, 0, 0], "to": [1, 1, 1]}}),
            json!({"line": {"from": [0, 0, 0]}}),
            json!({"line": 5}),
            json!({"line": {"from": [0, 0, 0], "to": [1e13, 0, 0]}}),
            json!({"count": 2000, "max_steps": 100_000}),
            json!({"seeds": vec![point; 2000], "max_steps": 100_000}),
        ];
        for id in
            ["results.slice", "results.surface", "results.streamlines", "results.frame", "results.contour", "results.deformed", "results.animate"]
        {
            for p in &hostile {
                let mut t = s.clone();
                let _ = t.execute(id, p);
                assert_eq!(t.model, s.model);
                // Whatever was accepted still draws.
                let _ = crate::view::scene(&t, &crate::view::Palette::LIGHT);
            }
        }
    }

    #[test]
    fn results_hostile_params_never_panic() {
        let s = solved();
        let hostile = [
            json!(null),
            json!([]),
            json!({"step": 7}),
            json!({"step": "Nope"}),
            json!({"step": "Bend", "field": "nope"}),
            json!({"field": 3, "bands": 1e300}),
            json!({"field": "von_mises", "bands": 0}),
            json!({"field": "von_mises", "bands": 25}),
            json!({"field": "von_mises", "range": {"max": "high"}}),
            json!({"field": "von_mises", "range": {"min": 5, "max": 5}}),
            json!({"field": "von_mises", "range": {"min": 1e308, "max": -1e308}}),
            json!({"field": "von_mises", "range": 7}),
            json!({"what": 7, "path": "x.csv"}),
            json!({"what": "probe"}),
            json!({"what": "probe", "path": "", "nodes": [1]}),
            json!({"what": "probe", "path": "/no/such/dir/x.csv", "nodes": [1]}),
            json!({"what": "probe", "path": "x.csv", "nodes": vec![1; 1_000_000]}),
            json!({"what": "reactions", "path": "x.csv", "step": "Modes"}),
            json!({"what": "reactions", "path": "x.csv", "step": "Nope"}),
            json!({"scale": -1}),
            json!({"scale": -0.0001}),
            json!({"scale": 1e300}),
            json!({"scale": "big"}),
            json!({"scale": f64::NAN}),
            json!({"show": "yes"}),
            json!({"mode": 0}),
            json!({"mode": 99}),
            json!({"mode": -1}),
            json!({"step": "Bend", "mode": 1}),
            json!({"on": 5}),
            json!({"sets": "root"}),
            json!({"sets": [1, 2]}),
            json!({"sets": vec!["root"; 2000]}),
            json!({"about": [1e308, 1e308, 1e308]}),
            json!({"about": "here"}),
            json!({"frame": -1}),
            json!({"frame": 1e300}),
            json!({"time": "late"}),
            json!({"time": 1e308}),
            json!({"step": "Bend", "frame": 0}),
        ];
        for id in ["results.deformed", "results.contour", "results.modes", "results.animate", "results.frame", "results.reactions", "results.clear"] {
            for p in &hostile {
                let mut t = s.clone();
                let _ = t.execute(id, p);
                assert_eq!(t.model, s.model);
            }
        }
        // Nothing solved: every command says so.
        let mut empty = Session::default();
        for id in ["results.deformed", "results.modes", "results.reactions", "results.animate"] {
            let e = empty.execute(id, &json!({"field": "displacement"})).unwrap_err().to_string();
            assert!(e.contains("solve"), "{id}: {e}");
        }
    }

    #[test]
    fn results_read_imports_warp3d_flat_files_and_round_trips_mcrslt05() {
        let dir = scratch("warp3d-read");
        let mut s = Session::default();
        let script = json!([
            {"command": "model.units", "params": {"system": "mm-t-s-N-MPa"}},
            {"command": "mesh.node.add", "params": {"label": 1, "xyz": [0, 0, 0]}},
            {"command": "mesh.node.add", "params": {"label": 2, "xyz": [1000, 0, 0]}},
            {"command": "mesh.node.add", "params": {"label": 3, "xyz": [1000, 1000, 0]}},
            {"command": "mesh.node.add", "params": {"label": 4, "xyz": [0, 1000, 0]}},
            {"command": "mesh.node.add", "params": {"label": 5, "xyz": [0, 0, 1000]}},
            {"command": "mesh.node.add", "params": {"label": 6, "xyz": [1000, 0, 1000]}},
            {"command": "mesh.node.add", "params": {"label": 7, "xyz": [1000, 1000, 1000]}},
            {"command": "mesh.node.add", "params": {"label": 8, "xyz": [0, 1000, 1000]}},
            {"command": "mesh.element.add", "params": {"label": 1, "kind": "hex8", "nodes": [1, 2, 3, 4, 5, 6, 7, 8]}},
            {"command": "model.set.create", "params": {"name": "all", "elements": [1]}},
            {"command": "model.set.create", "params": {"name": "fix", "nodes": [1, 4, 5, 8]}},
            {"command": "model.set.create", "params": {"name": "tip", "nodes": [2, 3, 6, 7]}},
            {
                "command": "model.material.create",
                "params": {
                    "name": "Steel",
                    "elastic": {"type": "isotropic", "young": 210000, "poisson": 0.3},
                    "plastic": {"type": "bilinear", "yield_stress": 250, "hardening_modulus": 1000, "hardening": "isotropic"}
                }
            },
            {"command": "model.section.create", "params": {"name": "solid", "set": "all", "material": "Steel", "kind": {"type": "solid"}}},
            {"command": "model.boundary_condition.create", "params": {"name": "clamp", "type": "fixed", "target": {"set": "fix"}}},
            {"command": "model.load.create", "params": {"name": "pull", "type": "force", "target": {"set": "tip"}, "vector": [20000, 0, 0]}},
            {
                "command": "model.step.create",
                "params": {
                    "name": "Pull",
                    "procedure": {"type": "nonlinear_static", "initial_increment": 0.5, "max_increments": 10},
                    "boundary_conditions": ["clamp"],
                    "loads": ["pull"],
                    "output": ["displacement", "stress", "reaction_force"]
                }
            }
        ]);
        s.run_script(&script).unwrap();

        let err_fmt = s.execute("results.read", &json!({"format": "abaqus", "path": dir.to_string_lossy()})).unwrap_err();
        assert!(matches!(&err_fmt, EngineError::Param(p) if p.path == "format"), "{err_fmt}");
        let err_missing = s.execute("results.read", &json!({"format": "warp3d"})).unwrap_err();
        assert!(matches!(&err_missing, EngineError::Param(p) if p.path == "path"), "{err_missing}");
        let err_no_dir =
            s.execute("results.read", &json!({"format": "warp3d", "path": dir.join("missing").to_string_lossy()})).unwrap_err().to_string();
        assert!(err_no_dir.contains("does not exist"), "{err_no_dir}");

        // Write header declaring mm-t-s-N-MPa + two increment flat-file sets (0000001 and 0000002).
        std::fs::write(dir.join("model.inp"), "! fieldcraft-units: mm-t-s-N-MPa\n").unwrap();
        let row26 = |sxx: f64, sxy: f64, syz: f64, sxz: f64, vm: f64, eps: f64| -> String {
            let mut cols = [0.0f64; 26];
            cols[0] = sxx;
            cols[3] = sxy;
            cols[4] = syz;
            cols[5] = sxz;
            cols[7] = vm;
            cols[8] = eps;
            cols.iter().map(|v| format!("{v:.8e}")).collect::<Vec<_>>().join(" ")
        };

        for (step_tag, u_tip, rx_fix, sxx, vm, eps) in
            [("0000001", 0.25, -2500.0, 100.0, 150.0, 0.0025), ("0000002", 1.5, -5000.0, 200.0, 250.0, 0.012)]
        {
            let wnd = (1..=8)
                .map(|n| if [2, 3, 6, 7].contains(&n) { format!("{u_tip} 0.0 0.0") } else { "0.0 0.0 0.0".into() })
                .collect::<Vec<_>>()
                .join("\n");
            let wnr = (1..=8)
                .map(|n| if [1, 4, 5, 8].contains(&n) { format!("{rx_fix} 0.0 0.0") } else { "999.0 0.0 0.0".into() })
                .collect::<Vec<_>>()
                .join("\n");
            let srow = row26(sxx, 10.0, 20.0, 30.0, vm, 0.0);
            let wns = vec![srow; 8].join("\n");
            let wes = row26(sxx, 0.0, 0.0, 0.0, vm, eps);
            std::fs::write(dir.join(format!("wnd{step_tag}_text")), format!("#\n{wnd}\n")).unwrap();
            std::fs::write(dir.join(format!("wnr{step_tag}_text")), format!("#\n{wnr}\n")).unwrap();
            std::fs::write(dir.join(format!("wns{step_tag}_text")), format!("#\n{wns}\n")).unwrap();
            std::fs::write(dir.join(format!("wes{step_tag}_text")), format!("#\n{wes}\n")).unwrap();
        }

        let reply = s.execute("results.read", &json!({"format": "warp3d", "path": dir.to_string_lossy()})).unwrap();
        assert_eq!(reply["format"], "warp3d");
        assert_eq!(reply["step"], "Pull");
        // Display units are mm-t-s-N-MPa, so reply matches input mm / MPa / N numbers!
        assert!((reply["max_displacement"]["value"].as_f64().unwrap() - 1.5).abs() < 1e-12, "{reply}");
        assert!((reply["max_von_mises"]["value"].as_f64().unwrap() - 250.0).abs() < 1e-9, "{reply}");
        assert!((reply["max_plastic_strain"]["value"].as_f64().unwrap() - 0.012).abs() < 1e-12, "{reply}");
        assert!((reply["reaction_total"][0].as_f64().unwrap() - (-20000.0)).abs() < 1e-9, "{reply}");

        // Stored SI values in session match SI conversion (mm -> m, MPa -> Pa) and Voigt 4<->5 swap.
        let stored = s.result("Pull").unwrap().clone();
        let Kind::Static(st) = stored.kind() else { panic!() };
        assert!((st.displacement[1][0] - 0.0015).abs() < 1e-12);
        assert!((st.stress[1][0] - 200.0e6).abs() < 1e-3);
        assert!((st.stress[1][4] - 30.0e6).abs() < 1e-3, "sxz mapped to index 4");
        assert!((st.stress[1][5] - 20.0e6).abs() < 1e-3, "syz mapped to index 5");
        assert_eq!(st.elements[0].plastic_strain, vec![0.012]);
        assert_eq!(st.reaction[1], [0.0, 0.0, 0.0], "free node residual masked to zero");

        // Downstream visualization, query, and .fieldcraft persistence all work.
        s.execute("results.contour", &json!({"step": "Pull", "field": "von_mises"})).unwrap();
        s.execute("results.deformed", &json!({"step": "Pull", "show": true, "scale": 10.0})).unwrap();
        let rx = s.execute("results.reactions", &json!({"step": "Pull", "sets": ["fix"]})).unwrap();
        assert!((rx["total"]["force"][0].as_f64().unwrap() - (-20000.0)).abs() < 1e-9, "{rx}");
        let xy = s.execute("results.xy", &json!({"step": "Pull", "y": {"field": "displacement_x", "node": 2}})).unwrap();
        assert_eq!(xy["points"], 3, "Frame 0 + 2 increments");
        let probe = s.execute("solve.probe", &json!({"step": "Pull", "nodes": [2], "elements": [1]})).unwrap();
        assert!((probe["nodes"][0]["displacement"][0].as_f64().unwrap() - 1.5).abs() < 1e-12, "{probe}");
        assert!((probe["elements"][0]["plastic_strain"][0].as_f64().unwrap() - 0.012).abs() < 1e-12, "{probe}");

        let fc_path = dir.join("roundtrip.fieldcraft");
        s.execute("file.save_as", &json!({"path": fc_path.to_string_lossy()})).unwrap();
        let mut reopened = Session::default();
        reopened.execute("file.open", &json!({"path": fc_path.to_string_lossy()})).unwrap();
        assert_eq!(reopened.result("Pull").unwrap(), s.result("Pull").unwrap(), "MCRSLT05 round-trip preserves stored WARP3D result");

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn results_read_imports_warp3d_fracture_stdout_and_surfaces_in_probe() {
        let dir = scratch("warp3d-fracture-read");
        let mut s = Session::default();
        let script = json!([
            {"command": "model.units", "params": {"system": "mm-t-s-N-MPa"}},
            {"command": "mesh.node.add", "params": {"label": 1, "xyz": [0, 0, 0]}},
            {"command": "mesh.node.add", "params": {"label": 2, "xyz": [1000, 0, 0]}},
            {"command": "mesh.node.add", "params": {"label": 3, "xyz": [1000, 1000, 0]}},
            {"command": "mesh.node.add", "params": {"label": 4, "xyz": [0, 1000, 0]}},
            {"command": "mesh.node.add", "params": {"label": 5, "xyz": [0, 0, 1000]}},
            {"command": "mesh.node.add", "params": {"label": 6, "xyz": [1000, 0, 1000]}},
            {"command": "mesh.node.add", "params": {"label": 7, "xyz": [1000, 1000, 1000]}},
            {"command": "mesh.node.add", "params": {"label": 8, "xyz": [0, 1000, 1000]}},
            {"command": "mesh.element.add", "params": {"label": 1, "kind": "hex8", "nodes": [1, 2, 3, 4, 5, 6, 7, 8]}},
            {"command": "model.set.create", "params": {"name": "all", "elements": [1]}},
            {"command": "model.set.create", "params": {"name": "fix", "nodes": [1, 4, 5, 8]}},
            {"command": "model.set.create", "params": {"name": "tip", "nodes": [2, 3, 6, 7]}},
            {"command": "model.material.create", "params": {"name": "Steel", "elastic": {"type": "isotropic", "young": 210000, "poisson": 0.3}}},
            {"command": "model.section.create", "params": {"name": "solid", "set": "all", "material": "Steel", "kind": {"type": "solid"}}},
            {"command": "model.boundary_condition.create", "params": {"name": "clamp", "type": "fixed", "target": {"set": "fix"}}},
            {"command": "model.load.create", "params": {"name": "pull", "type": "force", "target": {"set": "tip"}, "vector": [20000, 0, 0]}},
            {"command": "model.step.create", "params": {"name": "Pull", "procedure": {"type": "static"}, "boundary_conditions": ["clamp"], "loads": ["pull"]}}
        ]);
        s.run_script(&script).unwrap();

        std::fs::write(dir.join("model.inp"), "! fieldcraft-units: mm-t-s-N-MPa\n").unwrap();
        let row26 = vec!["0.0"; 26].join(" ");
        std::fs::write(dir.join("wnd0000001_text"), format!("#\n{}", "0.1 0.0 0.0\n".repeat(8))).unwrap();
        std::fs::write(dir.join("wnr0000001_text"), format!("#\n{}", "0.0 0.0 0.0\n".repeat(8))).unwrap();
        std::fs::write(dir.join("wns0000001_text"), format!("#\n{}", [row26.as_str(); 8].join("\n"))).unwrap();
        std::fs::write(dir.join("wes0000001_text"), format!("#\n{row26}\n")).unwrap();
        // Values below are in mm-t-s-N-MPa (N/mm for J, MPa·√mm for K, MPa for T, mm^2 for A_q).
        let stdout_table = "\
     Domain definition:  cf_0001

        Number of front nodes:    2 Nodes on front:
              2      3
        Direction cosines:   0.000000  1.000000  0.000000
        Tangent vector direction cosines:   0.000000  0.000000  1.000000
        Front order:  1     Verify front: T Domain type:  4
     ** end of domain definition **

                         domain-integral post-processing
                         -------------------------------
 step:     1
 domain id: cf_0001
 area under q along front:        2.500000E+01

                                             J-integral components
                                          ===========================
 domain        dm1          dm2          dm3          dm4          dm5          dm6          dm7          dm8       total J   killed ele
     1     1.1500E+01   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   1.1500E+01   (  0)
     2     1.2500E+01   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   1.2500E+01   (  0)

    average:  1.2000E+01   minimum:  1.1500E+01   maximum: 1.2500E+01     (cf_0001)
  1.2000E+01  1.1500E+01  1.2500E+01

          Stress intensity factors obtained from J-values assuming pure mode I, mode II or mode III loading
 domain   KI plane stress   KI plane strain   KII plane stress   KII plane strain   KIII anti-plane shear
     1     5.0000E+01       5.2000E+01        0.0000E+00        0.0000E+00          0.0000E+00
     2     5.4000E+01       5.6000E+01        0.0000E+00        0.0000E+00          0.0000E+00

               Static I-values
                          -- KI = 1, KII = 0, KIII = 0, plane strain --
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I     KI (pstrn) killed ele
     1     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   5.1000E+01   (  0)
     2     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   5.5000E+01   (  0)
    average:  5.3000E+01   minimum:  5.1000E+01   maximum: 5.5000E+01     (cf_0001)
  5.3000E+01  5.1000E+01  5.5000E+01

                          --T-stress (\"plane strain\" auxiliary fields)--
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I     T11 (pstrn)      T33     killed ele
     1     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00  -1.4000E+01  -4.0000E+00   (  0)
     2     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00  -1.6000E+01  -5.0000E+00   (  0)
    average:     minimum:     maximum:     (cf_0001)
 -1.5000E+01 -1.4000E+01 -1.6000E+01
 -4.5000E+00 -4.0000E+00 -5.0000E+00
";
        std::fs::write(dir.join("warp3d.out"), stdout_table).unwrap();

        let reply = s.execute("results.read", &json!({"format": "warp3d", "path": dir.to_string_lossy()})).unwrap();
        assert!(reply.get("k_unit").is_none());
        assert_eq!(reply["units"]["stress_intensity"], "MPa·√mm");
        assert_eq!(reply["units"]["force_per_length"], "N/mm");
        assert_eq!(reply["units"]["area"], "mm²");
        let f = &reply["fracture"][0];
        assert_eq!(f["domain_id"], "cf_0001");
        assert_eq!(f["front_nodes"], json!([2, 3]));
        assert!((f["area_aq"].as_f64().unwrap() - 25.0).abs() < 1e-12);
        assert!((f["j_mean"].as_f64().unwrap() - 12.0).abs() < 1e-12);
        assert!((f["k_i_mean"].as_f64().unwrap() - 53.0).abs() < 1e-9);
        assert!((f["t_11_mean"].as_f64().unwrap() - (-15.0)).abs() < 1e-9);
        assert!((f["t_33_mean"].as_f64().unwrap() - (-4.5)).abs() < 1e-9);

        // Session stores exact SI values.
        let stored = s.fracture_results("Pull").unwrap();
        assert_eq!(stored.len(), 1);
        assert!((stored[0].j_mean - 12_000.0).abs() < 1e-6);
        assert!((stored[0].area_aq - 2.5e-5).abs() < 1e-12);

        // solve.probe no longer attaches fracture data or k_unit.
        let probe = s.execute("solve.probe", &json!({"step": "Pull", "nodes": [2]})).unwrap();
        assert!(probe.get("k_unit").is_none());
        assert!(probe.get("fracture").is_none());
        assert!(probe["units"].get("stress_intensity").is_none());

        // results.export.csv with what: "fracture" writes display-unit domain rows.
        let frac_csv_mm = dir.join("fracture_mm.csv");
        let csv_reply_mm =
            s.execute("results.export.csv", &json!({"what": "fracture", "step": "Pull", "path": frac_csv_mm.to_string_lossy()})).unwrap();
        assert_eq!(csv_reply_mm["rows"], 1);
        let csv_text_mm = std::fs::read_to_string(&frac_csv_mm).unwrap();
        assert!(csv_text_mm.lines().next().unwrap().ends_with(",warp_step,interaction_stress_state"), "{csv_text_mm}");
        assert!(csv_text_mm.contains("j_mean [N/mm]"), "{csv_text_mm}");
        assert!(csv_text_mm.contains("k_i_mean [MPa·√mm]"), "{csv_text_mm}");
        assert!(csv_text_mm.contains("cf_0001") && csv_text_mm.contains("2 3"), "{csv_text_mm}");
        assert!(csv_text_mm.contains(",1,plane_strain"), "{csv_text_mm}");

        // Switching display units preserves fracture results via refresh_results() and re-exports in SI.
        s.execute("model.units", &json!({"system": "m-kg-s-N-Pa"})).unwrap();
        assert!(s.fracture_results("Pull").is_some());
        let frac_csv_si = dir.join("fracture_si.csv");
        s.execute("results.export.csv", &json!({"what": "fracture", "step": "Pull", "path": frac_csv_si.to_string_lossy()})).unwrap();
        let csv_text_si = std::fs::read_to_string(&frac_csv_si).unwrap();
        assert!(csv_text_si.contains("j_mean [N/m]"), "{csv_text_si}");
        assert!(csv_text_si.contains("k_i_mean [Pa·√m]"), "{csv_text_si}");
        assert!(csv_text_si.contains("12000"), "{csv_text_si}");

        // A model edit invalidates stored fracture results and CSV export refuses stale data.
        s.execute("mesh.node.add", &json!({"label": 99, "xyz": [500, 500, 500]})).unwrap();
        assert!(s.fracture_results("Pull").is_none());
        let stale_err = s
            .execute("results.export.csv", &json!({"what": "fracture", "step": "Pull", "path": frac_csv_si.to_string_lossy()}))
            .unwrap_err()
            .to_string();
        assert!(stale_err.contains("stale"), "{stale_err}");

        let _ = std::fs::remove_dir_all(&dir);
    }
}

//! WARP3D v18.5.0 solver adapter for FieldCraft.
//!
//! Placed at Layer 2 alongside `fieldcraft-solver` rather than inside `fieldcraft-decks`
//! (L1) so a single crate can own deck generation, native subprocess execution, and
//! flat-file / domain-integral result parsing (see `docs/warp3d.md`).
//!
//! Current scope:
//! - Converts a resolved FieldCraft 3D solid `Model` into a unit-consistent WARP3D ASCII
//!   `.inp` deck, including section blocking, material curves, BCs/MPCs, loads, and
//!   optional crack-front `domain` J/interaction-integral definitions.
//! - Native subprocess execution (`runner`) with streaming drain, cancellation, timeout,
//!   and flat-file / domain-integral result parsing (`results`), plus wasm-safe stubs.
//! - Imports flat WARP3D 18.5.0 ASCII `.inp` model-data decks (`reader`) into SI `Model`s.
#![forbid(unsafe_code)]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

pub mod check;
pub mod deck;
pub mod ffs;
pub mod fracture;
pub mod reader;
pub mod results;
pub mod runner;
pub(crate) mod wedge6;

use std::collections::BTreeMap;

use fieldcraft_model::units::Preset;
use fieldcraft_model::{ElementKind, Model};
use fieldcraft_solver::StaticResult;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub use check::{
    CheckScope, DiagnosticSeverity, Warp3dCheckReport, Warp3dDiagnostic, Warp3dQuickNote, quick_note, validate_model, validate_resolved,
};
pub use deck::write_deck;
pub use ffs::{
    EvaluatedStep, FAD_CURVE_POINTS, FadConfig, FadOption, FadPoint, FadSummary, FfsDiagnostic, FfsError, FfsMaterialDefaults, FfsReport, FfsRequest,
    KrSource, LimitingMode, LrInput, MAX_RING_EXCLUSION, Option2Curve, Option2Evaluator, ReactionComponent, RingExclusion, SpreadStatus,
    StationProfile, StationSelection, StepStationPoint, ToughnessInput, build_station_profiles, evaluate_fad_curve, evaluate_step_ffs,
    evaluate_step_ffs_detailed, extract_material_defaults, reserve_factor,
};
pub use results::{parse_domain_integrals, read_results_dir};
pub use runner::{Warp3dBinarySource, Warp3dBinaryStatus, find_warp3d_binary, resolve_warp3d_binary, run_warp3d, run_warp3d_with_override};

/// Errors returned by the FieldCraft ↔ WARP3D adapter.
#[derive(Debug, Error)]
pub enum Warp3dError {
    #[error(transparent)]
    Model(#[from] fieldcraft_model::Error),

    #[error("WARP3D cannot express this model: {0}")]
    Unsupported(String),

    #[error(
        "WARP3D executable not found ({0}); set FIELDCRAFT_WARP3D_BIN or WARP3D_HOME, or use `deck.write` (format: \"warp3d\") and `results.read`"
    )]
    BinaryNotFound(String),

    #[error("WARP3D I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("WARP3D exited with status {code:?}:\n{log_tail}")]
    SolverFailed { code: Option<i32>, log_tail: String },

    #[error("WARP3D timed out after {seconds}s:\n{log_tail}")]
    Timeout { seconds: u64, log_tail: String },

    #[error("WARP3D analysis cancelled")]
    Cancelled,

    #[error("WARP3D result parse error: {0}")]
    Parse(String),
}

pub type Result<T> = std::result::Result<T, Warp3dError>;

/// Deterministic, bidirectional 1-based label mapping between sparse FieldCraft labels
/// and contiguous WARP3D 1..=N / 1..=E IDs.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LabelMap {
    /// FieldCraft node label -> 1-based WARP3D node ID.
    pub fc_node_to_warp: BTreeMap<u32, u32>,
    /// 0-indexed lookup: `warp_node_to_fc[(warp_id - 1) as usize] == fc_label`.
    pub warp_node_to_fc: Vec<u32>,
    /// FieldCraft element label -> 1-based WARP3D element ID (sorted by `(ElementKind, section, label)`).
    pub fc_elem_to_warp: BTreeMap<u32, u32>,
    /// 0-indexed lookup: `warp_elem_to_fc[(warp_id - 1) as usize] == fc_label`.
    pub warp_elem_to_fc: Vec<u32>,
}

fn kind_rank(kind: ElementKind) -> u8 {
    match kind {
        ElementKind::Hex8 => 0,
        ElementKind::Wedge6 => 1,
        ElementKind::Hex20 => 2,
        ElementKind::Tet4 => 3,
        ElementKind::Tet10 => 4,
        _ => 5,
    }
}

impl LabelMap {
    /// Build a deterministic label mapping for `model`.
    ///
    /// Nodes are numbered 1..=N in ascending FieldCraft label order.
    /// Elements are grouped by `(kind_rank, section_name, fc_label)` so elements sharing a type
    /// and section form contiguous 1..=E ranges for WARP3D's `blocking automatic`.
    pub fn from_model(model: &Model) -> Self {
        let mut fc_node_to_warp = BTreeMap::new();
        let mut warp_node_to_fc = Vec::with_capacity(model.nodes().len());
        for (idx, &fc_label) in model.nodes().keys().enumerate() {
            let warp_id = u32::try_from(idx + 1).unwrap_or(u32::MAX);
            fc_node_to_warp.insert(fc_label, warp_id);
            warp_node_to_fc.push(fc_label);
        }

        let mut elem_to_section: BTreeMap<u32, &str> = BTreeMap::new();
        for (sec_name, sec) in model.sections() {
            if let Some(resolved_set) = model.resolve_set_partial(&sec.set) {
                for elem_id in resolved_set.elements {
                    elem_to_section.entry(elem_id).or_insert(sec_name.as_str());
                }
            }
        }

        let mut sorted: Vec<(u8, &str, u32)> = model
            .elements()
            .iter()
            .map(|(&fc_label, elem)| {
                let sec = elem_to_section.get(&fc_label).copied().unwrap_or("");
                (kind_rank(elem.kind), sec, fc_label)
            })
            .collect();
        sorted.sort();

        let mut fc_elem_to_warp = BTreeMap::new();
        let mut warp_elem_to_fc = Vec::with_capacity(sorted.len());
        for (idx, (_, _, fc_label)) in sorted.into_iter().enumerate() {
            let warp_id = u32::try_from(idx + 1).unwrap_or(u32::MAX);
            fc_elem_to_warp.insert(fc_label, warp_id);
            warp_elem_to_fc.push(fc_label);
        }

        Self { fc_node_to_warp, warp_node_to_fc, fc_elem_to_warp, warp_elem_to_fc }
    }
}

/// Maximum rings allowed per WARP3D crack-front domain (`1..=64`).
pub(crate) const MAX_RINGS_PER_DOMAIN: usize = 64;

/// Auxiliary stress state for WARP3D interaction-integral (`II8`) evaluation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InteractionStressState {
    #[default]
    PlaneStrain,
    PlaneStress,
}

/// q-function distribution type for WARP3D `domain` evaluation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CrackFunctionType {
    /// Pointwise evaluation along the crack front (WARP3D function types a/b/c per node).
    #[default]
    Pointwise,
    /// Single average evaluation over the entire front (WARP3D function type d).
    FrontAverage,
}

/// Optional 3D fracture-mechanics domain integral specification.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CrackFrontSpec {
    /// Name of the FieldCraft node `Set` lying along the 3D crack front.
    pub set: String,
    /// Unit normal to the crack plane (WARP3D x2 direction); inferred from a symmetry BC if None.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub normal: Option<[f64; 3]>,
    /// Whether only half the crack geometry is modeled across a symmetry plane.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub symmetric: Option<bool>,
    /// Outer element ring to integrate (`1..=64`, default `4`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rings: Option<usize>,
    /// Starting element ring to integrate (`1..=rings`, default `1`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ring_start: Option<usize>,
    /// Also run `compute domain interaction integrals` for K_I, K_II, K_III, T11, T33.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub interaction_integral: Option<bool>,
    /// Auxiliary stress state for interaction integrals (`plane_strain` default | `plane_stress`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub interaction_stress_state: Option<InteractionStressState>,
    /// Pointwise (a/b/c per station) vs whole-front average (type d).
    #[serde(default)]
    pub function_type: CrackFunctionType,
    /// Backward-compatible flag to evaluate domain integrals after every nonlinear increment (`Some(true)`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub every_step: Option<bool>,
    /// Evaluate domain integrals every `k` nonlinear increments, always anchoring Increment 1 and the final increment.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub increment_interval: Option<usize>,
}

/// Values extracted for one ring of a WARP3D domain integral evaluation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RingIntegral {
    pub ring: usize,
    pub j: f64,
    pub k_i: Option<f64>,
    pub k_ii: Option<f64>,
    pub k_iii: Option<f64>,
    pub t_11: Option<f64>,
    pub t_33: Option<f64>,
}

/// Parsed J-integral and interaction-integral result for one WARP3D domain (one crack-front
/// station or whole front) in SI units after conversion.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DomainIntegralResult {
    pub domain_id: String,
    pub step: usize,
    /// FieldCraft node label(s) on the crack front associated with this domain.
    pub front_nodes: Vec<u32>,
    /// Normalized position s in [0, 1] along the front (if applicable).
    pub s_position: Option<f64>,
    /// Crack plane normal unit vector (x2).
    pub normal: [f64; 3],
    /// Crack front tangent unit vector (x3).
    pub tangent: [f64; 3],
    /// Crack advance unit vector (x1 = normal x tangent).
    pub advance: [f64; 3],
    pub area_aq: f64,
    pub rings: Vec<RingIntegral>,
    pub j_mean: f64,
    pub j_min: f64,
    pub j_max: f64,
    /// Dimensionless path-independence spread: `(j_max - j_min).abs() / j_mean.abs().max(MIN_J_DENOM)`.
    pub ring_spread: f64,
    pub j_total: Option<f64>,
    pub k_j_plane_strain: Option<f64>,
    pub k_j_plane_stress: Option<f64>,
    pub k_i_mean: Option<f64>,
    pub k_ii_mean: Option<f64>,
    pub k_iii_mean: Option<f64>,
    pub t_11_mean: Option<f64>,
    pub t_33_mean: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub interaction_stress_state: Option<InteractionStressState>,
}

/// Result of translating a FieldCraft model step into a WARP3D `.inp` deck.
#[derive(Debug, Clone, PartialEq)]
pub struct WrittenDeck {
    pub text: String,
    pub label_map: LabelMap,
    pub step_name: String,
    pub step_count: usize,
    pub units: Preset,
    pub warnings: Vec<String>,
}

/// Complete WARP3D solution mapped back into FieldCraft types and SI units.
#[derive(Debug, Clone, PartialEq)]
pub struct Warp3dSolution {
    pub static_result: StaticResult,
    pub fracture: Vec<DomainIntegralResult>,
    /// Exact ascending WARP3D flat-file step numbers (`wnd*` keys) corresponding to solved
    /// increments (`history.increments[pos]` / `history.frames[pos + 1]`).
    pub step_numbers: Vec<usize>,
    /// True if stdout or domain-integral parser bounds truncated any fracture output.
    pub domains_truncated: bool,
    /// Tri-state Increment-1 elastic status from `wes00001*` (`Some(true)` = max peeq <= 1e-12,
    /// `Some(false)` = yielded at Increment 1, `None` = missing/incomplete Increment-1 `wes*`).
    pub step1_is_elastic: Option<bool>,
    pub warnings: Vec<String>,
    pub stdout_log: String,
}

#[cfg(test)]
mod tests {
    use fieldcraft_model::{ElementKind, Model, Section, SectionKind, Set};

    use super::*;

    #[test]
    fn label_map_groups_elements_by_kind_and_section() {
        let mut model = Model::default();
        // Sparse node labels inserted out of order.
        for (idx, label) in [10u32, 5, 42, 100, 12, 7, 88, 60, 101, 102, 103, 104].into_iter().enumerate() {
            model.set_node(label, [idx as f64, 0.0, 0.0]).unwrap();
        }
        // Elements with different sections and kinds inserted out of order.
        model.add_set("s_hex_b", Set { elements: vec![30], ..Set::default() }).unwrap();
        model.add_set("s_hex_a", Set { elements: vec![10], ..Set::default() }).unwrap();
        model.add_set("s_tet", Set { elements: vec![20], ..Set::default() }).unwrap();
        model.add_material("M", fieldcraft_model::Material::default()).ok();
        model
            .add_material(
                "Steel",
                fieldcraft_model::Material {
                    elastic: Some(fieldcraft_model::Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }),
                    ..Default::default()
                },
            )
            .unwrap();
        model
            .add_section(
                "sec_b",
                Section { set: "s_hex_b".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model
            .add_section(
                "sec_a",
                Section { set: "s_hex_a".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model
            .add_section(
                "sec_t",
                Section { set: "s_tet".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();

        let h_nodes = vec![5, 7, 10, 12, 42, 60, 88, 100];
        model.set_element(30, ElementKind::Hex8, h_nodes.clone()).unwrap();
        model.set_element(20, ElementKind::Tet4, vec![5, 7, 10, 42]).unwrap();
        model.set_element(10, ElementKind::Hex8, h_nodes).unwrap();
        model.set_element(25, ElementKind::Wedge6, vec![5, 7, 10, 42, 60, 88]).unwrap();
        model.add_set("s_wedge", Set { elements: vec![25], ..Set::default() }).unwrap();
        model
            .add_section(
                "sec_w",
                Section { set: "s_wedge".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();

        let map = LabelMap::from_model(&model);
        assert_eq!(map.warp_node_to_fc, vec![5, 7, 10, 12, 42, 60, 88, 100, 101, 102, 103, 104]);
        assert_eq!(map.fc_node_to_warp.get(&5), Some(&1));
        assert_eq!(map.fc_node_to_warp.get(&104), Some(&12));
        // Hex8 sec_a (10), Hex8 sec_b (30), Wedge6 sec_w (25), Tet4 sec_t (20).
        assert_eq!(map.warp_elem_to_fc, vec![10, 30, 25, 20]);
        assert_eq!(map.fc_elem_to_warp.get(&10), Some(&1));
        assert_eq!(map.fc_elem_to_warp.get(&30), Some(&2));
        assert_eq!(map.fc_elem_to_warp.get(&25), Some(&3));
        assert_eq!(map.fc_elem_to_warp.get(&20), Some(&4));
    }
}

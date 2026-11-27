//! Read-only Fitness-for-Service (FFS) facade over SI WARP3D fracture data and model definitions.

use fieldcraft_model::Model;
pub use fieldcraft_warp3d::DiagnosticSeverity;
use fieldcraft_warp3d::DomainIntegralResult;
pub use fieldcraft_warp3d::InteractionStressState;
pub use fieldcraft_warp3d::ffs::{
    EvaluatedStep, FAD_CURVE_POINTS, FadConfig, FadOption, FadPoint, FadSummary, FfsDiagnostic, FfsMaterialDefaults, FfsReport, FfsRequest, KrSource,
    LimitingMode, LrInput, MAX_RING_EXCLUSION, Option2Curve, ReactionComponent, RingExclusion, SpreadStatus, StationProfile, StationSelection,
    StepStationPoint, ToughnessInput, build_station_profiles, evaluate_fad_curve, extract_material_defaults, reserve_factor,
};

use crate::Session;

/// Extract crack-zone elastic/plastic defaults in SI for `step` in O(sections + |front_nodes|),
/// without calling `model.resolve()`.
pub fn material_defaults(session: &Session, step: &str) -> Option<FfsMaterialDefaults> {
    let domains = session.fracture_results(step)?;
    if domains.is_empty() {
        return None;
    }
    Some(extract_material_defaults(&session.model, domains))
}

/// Summarize ring-filtered crack-front profiles and spread status in SI, shared across `results.ffs`
/// and UI summary panels so displayed spread numbers stay identical.
pub fn summarize_fracture_spread(
    model: &Model,
    domains: &[DomainIntegralResult],
    ring_exclusion: RingExclusion,
) -> (Vec<StationProfile>, f64, SpreadStatus, Vec<FfsDiagnostic>) {
    build_station_profiles(model, domains, ring_exclusion, None)
}

/// Sorted, deduplicated boundary-condition set names on `step` for `LrInput::ReactionRatio`.
pub fn reaction_bc_sets(session: &Session, step: &str) -> Vec<String> {
    let mut names: Vec<String> = session
        .model
        .step(step)
        .map(|st| st.boundary_conditions.iter().filter_map(|bc| session.model.boundary_condition(bc)?.target().set().map(str::to_string)).collect())
        .unwrap_or_default();
    names.sort();
    names.dedup();
    names
}

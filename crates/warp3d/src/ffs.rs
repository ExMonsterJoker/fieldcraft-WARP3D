//! Pure SI crack-front fracture profiles, ring-exclusion diagnostics, and API 579-1 / ASME FFS-1
//! Part 9 Option 1 / Option 2 Failure Assessment Diagram (FAD) evaluation.
//!
//! All values inside this module are strictly SI (`m`, `N`, `Pa`, `J/m^2`, `Pa·√m`). Boundary
//! conversions into user display systems happen exclusively in `crates/engine`.

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};

use fieldcraft_model::{BoundaryCondition, Elastic, HardeningPoint, Material, Model, Plastic, Procedure, SectionKind, Target};
use fieldcraft_solver::StaticResult;
use serde::{Deserialize, Serialize};

use crate::results::{MIN_J_DENOM, MIN_VEC_NORM};
use crate::{DiagnosticSeverity, DomainIntegralResult, Result, RingIntegral, Warp3dError};

pub const MAX_RING_EXCLUSION: usize = 64;
pub const FAD_CURVE_POINTS: usize = 101;
pub const MAX_DIAGNOSTICS: usize = 100;
pub const MAX_SAMPLE_IDS: usize = 10;

const ACCEPT_TOL: f64 = 1e-12; // unit-factors: dimensionless FAD acceptance tolerance

fn default_true() -> bool {
    true
}

/// Structured error returned by [`evaluate_step_ffs_detailed`] so callers (`crates/engine`) can
/// attribute parameter failures directly to exact `RESULTS_FFS` form field paths.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FfsError {
    pub path: Option<&'static str>,
    pub message: String,
}

impl FfsError {
    pub fn param(path: &'static str, message: impl Into<String>) -> Self {
        Self { path: Some(path), message: message.into() }
    }

    pub fn other(message: impl Into<String>) -> Self {
        Self { path: None, message: message.into() }
    }
}

impl std::fmt::Display for FfsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for FfsError {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpreadStatus {
    Good,    // kept_rings >= 2 && spread <= 0.05
    Caution, // kept_rings >= 2 && 0.05 < spread <= 0.10
    Poor,    // kept_rings >= 2 && spread > 0.10
    #[default]
    Unverified, // kept_rings < 2 || |j_mean| <= MIN_J_DENOM || non-finite
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum RingExclusion {
    /// Exclude innermost sorted ring when total rings >= 3; otherwise keep all rings.
    #[default]
    Auto,
    /// Exclude the innermost `count` sorted rings, clamped to keep at least 1 ring.
    Count { count: usize },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FadOption {
    #[default]
    Option1,
    Option2,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(tag = "source", rename_all = "snake_case")]
pub enum Option2Curve {
    /// Derive true stress-strain curve from crack-zone solid Material (Bilinear or Table).
    #[default]
    ModelMaterial,
    /// Ramberg-Osgood: eps = sigma/E + alpha * (sigma_ys/E) * (sigma/sigma_ys)^n.
    RambergOsgood { alpha: f64, n: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReactionComponent {
    X,
    Y,
    Z,
    #[default]
    Magnitude,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum LrInput {
    ReferenceStress { sigma_ref_full: f64 },
    DirectLoadRatio { l_r_full: f64 },
    ReactionRatio { bc_set: String, component: ReactionComponent, limit_load: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ToughnessInput {
    KMat {
        value: f64,
    },
    JIc {
        value: f64,
        #[serde(default = "default_true")]
        plane_strain: bool,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum KrSource {
    #[default]
    Auto,
    ElasticKi,
    LinearScaledStep1Ki,
    KjPlaneStrain,
    KjPlaneStress,
}

#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum StationSelection {
    /// Worst station at each increment: lowest Reserve Factor (tie-break: highest radial
    /// distance sqrt(Lr^2 + Kr^2), then highest Lr, then earliest traversal index).
    #[default]
    Governing,
    DomainId {
        domain_id: String,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LimitingMode {
    Unloaded,
    Fracture,
    ElasticPlastic,
    PlasticCollapse,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FadConfig {
    #[serde(default)]
    pub option: FadOption,
    #[serde(default)]
    pub option2_curve: Option2Curve,
    pub lr_input: LrInput,
    pub toughness: ToughnessInput,
    #[serde(default)]
    pub kr_source: KrSource,
    #[serde(default)]
    pub station_selection: StationSelection,
    #[serde(default)]
    pub sigma_ys: Option<f64>,
    #[serde(default)]
    pub sigma_uts: Option<f64>,
    #[serde(default)]
    pub l_r_max: Option<f64>,
    #[serde(default)]
    pub young: Option<f64>,
    #[serde(default)]
    pub poisson: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct FfsRequest {
    pub step: String,
    #[serde(default)]
    pub ring_exclusion: RingExclusion,
    /// Nonlinear 0-based frame index (`None`: final frame). Ignored on linear steps.
    #[serde(default)]
    pub active_frame: Option<usize>,
    /// Optional FAD configuration (`None`: evaluate crack-front profiles & ring spread only).
    #[serde(default)]
    pub fad: Option<FadConfig>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StationProfile {
    pub domain_id: String,
    pub warp_step: usize,
    pub frame_index: Option<usize>,
    pub front_nodes: Vec<u32>,
    pub focus_node: Option<u32>,
    pub s_length: Option<f64>,
    pub s_norm: f64,
    pub total_rings: usize,
    pub kept_rings: usize,
    pub excluded_rings: Vec<usize>,
    pub j_mean: f64,
    pub j_min: f64,
    pub j_max: f64,
    pub ring_spread: f64,
    pub spread_status: SpreadStatus,
    pub k_i_mean: Option<f64>,
    pub k_ii_mean: Option<f64>,
    pub k_iii_mean: Option<f64>,
    pub k_j_plane_strain: Option<f64>,
    pub k_j_plane_stress: Option<f64>,
    pub t_11_mean: Option<f64>,
    pub t_33_mean: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FadPoint {
    pub domain_id: String,
    pub warp_step: usize,
    pub frame_index: Option<usize>,
    pub increment: usize,
    pub load_factor: f64,
    pub l_r: f64,
    pub k_r: f64,
    pub k_r_allow: f64,
    pub k_eff: Option<f64>,
    pub kr_source_used: KrSource,
    pub reserve_factor: Option<f64>,
    pub safety_margin: Option<f64>,
    pub acceptable: bool,
    pub limiting_mode: LimitingMode,
    pub intersection: Option<[f64; 2]>,
    pub epfm_k_j_ratio: Option<f64>,
    pub epfm_j_ratio: Option<f64>,
    pub reaction_value: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FadSummary {
    pub option: FadOption,
    pub effective_lr_max: f64,
    pub k_mat: f64,
    pub j_mat: Option<f64>,
    pub sigma_ys: f64,
    pub sigma_uts: Option<f64>,
    pub envelope: Vec<[f64; 2]>,
    pub trajectory: Vec<FadPoint>,
    pub active_stations: Vec<FadPoint>,
    pub active_verdict: Option<FadPoint>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct FfsDiagnostic {
    pub severity: DiagnosticSeverity,
    pub code: Cow<'static, str>,
    pub message: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FfsMaterialDefaults {
    pub young: Option<f64>,
    pub poisson: Option<f64>,
    pub sigma_ys: Option<f64>,
    pub has_plasticity: bool,
    pub option2_supported: bool,
    pub material_names: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FfsReport {
    pub step: String,
    pub active_warp_step: usize,
    pub active_frame: Option<usize>,
    pub active_load_factor: f64,
    pub is_nonlinear: bool,
    pub defaults: FfsMaterialDefaults,
    pub active_profiles: Vec<StationProfile>,
    pub max_ring_spread: f64,
    pub worst_spread_status: SpreadStatus,
    pub fad: Option<FadSummary>,
    pub diagnostics: Vec<FfsDiagnostic>,
}

/// Runtime evaluator for API 579 Option 2 uniaxial true stress–true strain curves.
#[derive(Debug, Clone)]
pub enum Option2Evaluator {
    Bilinear { young: f64, yield_stress: f64, hardening_modulus: f64 },
    Table { young: f64, points: Vec<(f64, f64)> }, // (total_strain, stress), prepended with (0.0, 0.0)
    RambergOsgood { young: f64, sigma_ys: f64, alpha: f64, n: f64 },
}

impl Option2Evaluator {
    fn eps_ref(&self, sigma: f64) -> f64 {
        if !sigma.is_finite() || sigma <= 0.0 {
            return 0.0;
        }
        match self {
            Option2Evaluator::Bilinear { young, yield_stress, hardening_modulus } => {
                if sigma <= *yield_stress || *hardening_modulus <= 0.0 {
                    sigma / *young
                } else {
                    sigma / *young + (sigma - *yield_stress) / *hardening_modulus
                }
            }
            Option2Evaluator::Table { points, .. } => {
                let Some(&last) = points.last() else {
                    return f64::NAN;
                };
                if sigma >= last.1 {
                    return last.0;
                }
                for w in points.windows(2) {
                    let (Some(&(e0, s0)), Some(&(e1, s1))) = (w.first(), w.get(1)) else {
                        continue;
                    };
                    if sigma <= s1 && s1 > s0 {
                        let t = (sigma - s0) / (s1 - s0);
                        return e0 + t * (e1 - e0);
                    }
                }
                last.0
            }
            Option2Evaluator::RambergOsgood { young, sigma_ys, alpha, n } => {
                let ratio = sigma / *sigma_ys;
                let pow = ratio.powf(*n);
                if !pow.is_finite() {
                    return f64::INFINITY;
                }
                sigma / *young + *alpha * (*sigma_ys / *young) * pow
            }
        }
    }

    fn young(&self) -> f64 {
        match *self {
            Option2Evaluator::Bilinear { young, .. } | Option2Evaluator::Table { young, .. } | Option2Evaluator::RambergOsgood { young, .. } => young,
        }
    }
}

fn push_diag(diags: &mut Vec<FfsDiagnostic>, severity: DiagnosticSeverity, code: &'static str, message: impl Into<String>) {
    if diags.len() < MAX_DIAGNOSTICS {
        diags.push(FfsDiagnostic { severity, code: Cow::Borrowed(code), message: message.into() });
    }
}

fn sample_ids(ids: &[String]) -> String {
    let shown: Vec<&str> = ids.iter().take(MAX_SAMPLE_IDS).map(String::as_str).collect();
    match ids.len().saturating_sub(shown.len()) {
        0 => shown.join(", "),
        more => format!("{}, and {more} more", shown.join(", ")),
    }
}

fn validate_table_points(points: &[HardeningPoint]) -> std::result::Result<(), &'static str> {
    let Some(first) = points.first() else {
        return Err("plastic hardening table is empty");
    };
    if !first.strain.is_finite() || first.strain != 0.0 {
        return Err("Option 2 requires a hardening table starting at plastic strain 0.0; use Option 1 or Ramberg-Osgood");
    }
    for p in points {
        if !p.strain.is_finite() || p.strain < 0.0 || !p.stress.is_finite() || p.stress <= 0.0 {
            return Err("plastic hardening table contains non-finite, negative strain, or non-positive stress values");
        }
    }
    for w in points.windows(2) {
        let (Some(a), Some(b)) = (w.first(), w.get(1)) else {
            continue;
        };
        if b.strain <= a.strain {
            return Err("Option 2 requires strictly increasing plastic strains in hardening table");
        }
        if b.stress <= a.stress {
            return Err("Option 2 does not support softening or flat-plateau hardening tables; use Option 1 or Ramberg-Osgood");
        }
    }
    Ok(())
}

fn is_option2_plastic_supported(plastic: Option<&Plastic>) -> bool {
    match plastic {
        Some(Plastic::Bilinear { yield_stress, hardening_modulus, .. }) => {
            yield_stress.is_finite() && *yield_stress > 0.0 && hardening_modulus.is_finite() && *hardening_modulus >= 0.0
        }
        Some(Plastic::Table { points }) => validate_table_points(points).is_ok(),
        Some(Plastic::JohnsonCook { .. }) | None => false,
    }
}

fn select_crack_zone_material<'a>(model: &'a Model, domains: &[DomainIntegralResult]) -> (Option<(&'a str, &'a Material)>, Vec<String>, bool) {
    let front_nodes: BTreeSet<u32> = domains.iter().flat_map(|d| d.front_nodes.iter().copied()).collect();

    let mut touching_mats: BTreeSet<String> = BTreeSet::new();
    let mut all_solid_mats: BTreeSet<String> = BTreeSet::new();
    let mut any_solid_plastic = false;

    for sec in model.sections().values() {
        if !matches!(sec.kind, SectionKind::Solid {}) {
            continue;
        }
        if let Some(mat) = model.material(&sec.material) {
            if mat.plastic.is_some() {
                any_solid_plastic = true;
            }
            all_solid_mats.insert(sec.material.clone());
            if !front_nodes.is_empty()
                && let Some(rset) = model.resolve_set_partial(&sec.set)
                && rset.elements.iter().any(|eid| model.element(*eid).is_some_and(|el| el.nodes.iter().any(|n| front_nodes.contains(n))))
            {
                touching_mats.insert(sec.material.clone());
            }
        }
    }

    let candidate_names: BTreeSet<String> = if !touching_mats.is_empty() { touching_mats } else { all_solid_mats };

    let mut candidates: Vec<(&'a str, &'a Material)> =
        candidate_names.iter().filter_map(|name| model.materials().get_key_value(name.as_str()).map(|(k, v)| (k.as_str(), v))).collect();

    candidates.sort_by(|(name_a, mat_a), (name_b, mat_b)| {
        let key = |mat: &Material| -> (bool, f64) {
            match mat.plastic.as_ref().map(Plastic::initial_yield) {
                Some(sy) if sy.is_finite() && sy > 0.0 => (false, sy),
                _ => (true, f64::INFINITY),
            }
        };
        let ka = key(mat_a);
        let kb = key(mat_b);
        ka.0.cmp(&kb.0).then_with(|| ka.1.total_cmp(&kb.1)).then_with(|| name_a.cmp(name_b))
    });

    let names: Vec<String> = candidate_names.into_iter().collect();
    let chosen = candidates.first().copied();
    (chosen, names, any_solid_plastic)
}

/// Extract crack-zone elastic and plastic defaults in $O(\text{sections} + |\text{front\_nodes}|)$
/// without calling `model.resolve()`.
pub fn extract_material_defaults(model: &Model, domains: &[DomainIntegralResult]) -> FfsMaterialDefaults {
    let (chosen, material_names, any_solid_plastic) = select_crack_zone_material(model, domains);
    let Some((_, mat)) = chosen else {
        return FfsMaterialDefaults {
            young: None,
            poisson: None,
            sigma_ys: None,
            has_plasticity: any_solid_plastic,
            option2_supported: false,
            material_names,
        };
    };

    let (young, poisson) = match mat.elastic {
        Some(Elastic::Isotropic { young, poisson }) => (Some(young), Some(poisson)),
        _ => (None, None),
    };
    let sigma_ys = mat.plastic.as_ref().map(Plastic::initial_yield).filter(|v| v.is_finite() && *v > 0.0);
    let has_plasticity = mat.plastic.is_some() || any_solid_plastic;
    let option2_supported = young.is_some() && is_option2_plastic_supported(mat.plastic.as_ref());

    FfsMaterialDefaults { young, poisson, sigma_ys, has_plasticity, option2_supported, material_names }
}

fn focus_node_for_domain(d: &DomainIntegralResult) -> Option<u32> {
    if d.front_nodes.len() == 1 {
        d.front_nodes.first().copied()
    } else if d.front_nodes.len() >= 2 {
        d.front_nodes.get(d.front_nodes.len() / 2).copied()
    } else {
        None
    }
}

fn kept_ring_mean(
    kept: &[&RingIntegral],
    all_rings: &[RingIntegral],
    extract: fn(&RingIntegral) -> Option<f64>,
    fallback: Option<f64>,
) -> Option<f64> {
    let has_any_ring_col = all_rings.iter().any(|r| extract(r).is_some());
    if !has_any_ring_col {
        return fallback;
    }
    let mut sum = 0.0;
    let mut cnt = 0usize;
    for r in kept {
        if let Some(v) = extract(r)
            && v.is_finite()
        {
            sum += v;
            cnt += 1;
        }
    }
    (cnt > 0).then(|| sum / (cnt as f64))
}

fn spread_rank(s: SpreadStatus) -> u8 {
    match s {
        SpreadStatus::Good => 0,
        SpreadStatus::Caution => 1,
        SpreadStatus::Unverified => 2,
        SpreadStatus::Poor => 3,
    }
}

/// Build ring-filtered [`StationProfile`]s and path-independence spread diagnostics in SI.
///
/// Groups domains by `d.step` so cumulative chord length `s_length` and `s_norm` stay normalized
/// within each WARP3D step while preserving station traversal order unless `s_position` is present
/// on all stations of a step and out of order.
pub fn build_station_profiles(
    model: &Model,
    domains: &[DomainIntegralResult],
    ring_exclusion: RingExclusion,
    elastic_override: Option<(f64, f64)>,
) -> (Vec<StationProfile>, f64, SpreadStatus, Vec<FfsDiagnostic>) {
    let mut diags = Vec::new();
    let defaults = extract_material_defaults(model, domains);
    if defaults.material_names.len() > 1 {
        push_diag(
            &mut diags,
            DiagnosticSeverity::Warning,
            "ffs.multiple_crack_materials",
            format!(
                "crack front touches multiple solid materials ({}); using `{}`",
                sample_ids(&defaults.material_names),
                defaults.material_names.first().map(String::as_str).unwrap_or_default()
            ),
        );
    }

    let elastic = elastic_override.filter(|(e, nu)| e.is_finite() && *e > 0.0 && nu.is_finite() && *nu > -1.0 && *nu < 0.5).or_else(|| {
        match (defaults.young, defaults.poisson) {
            (Some(e), Some(nu)) if e.is_finite() && e > 0.0 && nu.is_finite() && nu > -1.0 && nu < 0.5 => Some((e, nu)),
            _ => None,
        }
    });

    let mut by_step: BTreeMap<usize, Vec<&DomainIntegralResult>> = BTreeMap::new();
    for d in domains {
        by_step.entry(d.step).or_default().push(d);
    }

    let mut clamped_ids = Vec::new();
    let mut insufficient_ids = Vec::new();
    let mut nonpositive_j_ids = Vec::new();
    let mut profiles = Vec::with_capacity(domains.len());

    for (warp_step, mut step_domains) in by_step {
        let all_s_present = step_domains.iter().all(|d| d.s_position.is_some_and(f64::is_finite));
        if all_s_present {
            let sorted = step_domains.windows(2).all(|w| {
                let (Some(a), Some(b)) = (w.first().and_then(|d| d.s_position), w.get(1).and_then(|d| d.s_position)) else {
                    return true;
                };
                a <= b
            });
            if !sorted {
                step_domains.sort_by(|a, b| {
                    let sa = a.s_position.unwrap_or(0.0);
                    let sb = b.s_position.unwrap_or(0.0);
                    sa.total_cmp(&sb).then_with(|| a.domain_id.cmp(&b.domain_id))
                });
            }
        }

        let m = step_domains.len();
        let focus_nodes: Vec<Option<u32>> = step_domains.iter().map(|d| focus_node_for_domain(d)).collect();
        let coords: Vec<Option<[f64; 3]>> = focus_nodes.iter().map(|fn_id| fn_id.and_then(|id| model.node(id)).map(|n| n.xyz)).collect();
        let all_coords = m > 0 && coords.iter().all(Option::is_some);
        let mut cum_lengths = Vec::with_capacity(m);
        if all_coords {
            cum_lengths.push(0.0_f64);
            for i in 1..m {
                let prev = cum_lengths.last().copied().unwrap_or(0.0);
                let dist = match (coords.get(i - 1).copied().flatten(), coords.get(i).copied().flatten()) {
                    (Some(a), Some(b)) => ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt(),
                    _ => 0.0,
                };
                cum_lengths.push(prev + dist);
            }
        }
        let total_len = cum_lengths.last().copied().unwrap_or(0.0);

        for (i, d) in step_domains.iter().enumerate() {
            let focus_node = focus_nodes.get(i).copied().flatten();
            let s_length = if all_coords { cum_lengths.get(i).copied() } else { None };
            let fallback_s = || d.s_position.filter(|v| v.is_finite()).unwrap_or_else(|| if m <= 1 { 0.0 } else { (i as f64) / ((m - 1) as f64) });
            let s_norm =
                if all_coords && total_len > MIN_VEC_NORM { cum_lengths.get(i).map_or_else(fallback_s, |s| s / total_len) } else { fallback_s() };

            let mut sorted_rings = d.rings.clone();
            sorted_rings.sort_by_key(|r| r.ring);

            let (
                total_rings,
                kept_rings,
                excluded_rings,
                j_mean,
                j_min,
                j_max,
                ring_spread,
                spread_status,
                k_i_mean,
                k_ii_mean,
                k_iii_mean,
                k_j_plane_strain,
                k_j_plane_stress,
                t_11_mean,
                t_33_mean,
            ) = if sorted_rings.is_empty() {
                insufficient_ids.push(d.domain_id.clone());
                if d.j_mean.is_finite() && d.j_mean < -MIN_J_DENOM {
                    nonpositive_j_ids.push(d.domain_id.clone());
                }
                (
                    0usize,
                    0usize,
                    Vec::new(),
                    d.j_mean,
                    d.j_min,
                    d.j_max,
                    d.ring_spread,
                    SpreadStatus::Unverified,
                    d.k_i_mean,
                    d.k_ii_mean,
                    d.k_iii_mean,
                    d.k_j_plane_strain,
                    d.k_j_plane_stress,
                    d.t_11_mean,
                    d.t_33_mean,
                )
            } else {
                let total = sorted_rings.len();
                let skip = match ring_exclusion {
                    RingExclusion::Auto => usize::from(total >= 3),
                    RingExclusion::Count { count } => {
                        let capped_count = count.min(MAX_RING_EXCLUSION);
                        let clamped = capped_count.min(total.saturating_sub(1));
                        if count > clamped {
                            clamped_ids.push(d.domain_id.clone());
                        }
                        clamped
                    }
                };
                let excluded: Vec<usize> = sorted_rings.get(..skip).unwrap_or(&[]).iter().map(|r| r.ring).collect();
                let kept_slice = sorted_rings.get(skip..).unwrap_or(&[]);
                let kept_refs: Vec<&RingIntegral> = kept_slice.iter().collect();
                let kept_count = kept_refs.len();
                if kept_count < 2 {
                    insufficient_ids.push(d.domain_id.clone());
                }

                let mut j_sum = 0.0;
                let mut j_lo = f64::INFINITY;
                let mut j_hi = f64::NEG_INFINITY;
                let mut finite_j_cnt = 0usize;
                for r in &kept_refs {
                    if r.j.is_finite() {
                        j_sum += r.j;
                        j_lo = j_lo.min(r.j);
                        j_hi = j_hi.max(r.j);
                        finite_j_cnt += 1;
                    }
                }
                let (j_mean, j_min, j_max, ring_spread) = if finite_j_cnt > 0 && finite_j_cnt == kept_count {
                    let jm = j_sum / (finite_j_cnt as f64);
                    let spread = (j_hi - j_lo).abs() / jm.abs().max(MIN_J_DENOM);
                    (jm, j_lo, j_hi, spread)
                } else {
                    (f64::NAN, f64::NAN, f64::NAN, f64::NAN)
                };

                if j_mean.is_finite() && j_mean < -MIN_J_DENOM {
                    nonpositive_j_ids.push(d.domain_id.clone());
                }

                let status = if kept_count < 2 || !j_mean.is_finite() || j_mean.abs() <= MIN_J_DENOM || !ring_spread.is_finite() {
                    SpreadStatus::Unverified
                } else if ring_spread <= 0.05 {
                    SpreadStatus::Good
                } else if ring_spread <= 0.10 {
                    SpreadStatus::Caution
                } else {
                    SpreadStatus::Poor
                };

                let k_i_mean = kept_ring_mean(&kept_refs, &sorted_rings, |r| r.k_i, d.k_i_mean);
                let k_ii_mean = kept_ring_mean(&kept_refs, &sorted_rings, |r| r.k_ii, d.k_ii_mean);
                let k_iii_mean = kept_ring_mean(&kept_refs, &sorted_rings, |r| r.k_iii, d.k_iii_mean);
                let t_11_mean = kept_ring_mean(&kept_refs, &sorted_rings, |r| r.t_11, d.t_11_mean);
                let t_33_mean = kept_ring_mean(&kept_refs, &sorted_rings, |r| r.t_33, d.t_33_mean);

                let (k_j_pstrn, k_j_pstrs) = if let Some((e, nu)) = elastic
                    && j_mean.is_finite()
                {
                    let sgn = if j_mean < 0.0 {
                        -1.0
                    } else if j_mean > 0.0 {
                        1.0
                    } else {
                        0.0
                    };
                    let pstrn = sgn * (j_mean.abs() * e / (1.0 - nu * nu)).sqrt();
                    let pstrs = sgn * (j_mean.abs() * e).sqrt();
                    (Some(pstrn), Some(pstrs))
                } else if j_mean.is_finite() && d.j_mean.is_finite() && d.j_mean.abs() > MIN_J_DENOM && d.j_mean * j_mean > 0.0 {
                    let scale = (j_mean / d.j_mean).sqrt();
                    (d.k_j_plane_strain.map(|v| v * scale), d.k_j_plane_stress.map(|v| v * scale))
                } else {
                    (None, None)
                };

                (
                    total,
                    kept_count,
                    excluded,
                    j_mean,
                    j_min,
                    j_max,
                    ring_spread,
                    status,
                    k_i_mean,
                    k_ii_mean,
                    k_iii_mean,
                    k_j_pstrn,
                    k_j_pstrs,
                    t_11_mean,
                    t_33_mean,
                )
            };

            profiles.push(StationProfile {
                domain_id: d.domain_id.clone(),
                warp_step,
                frame_index: None,
                front_nodes: d.front_nodes.clone(),
                focus_node,
                s_length,
                s_norm,
                total_rings,
                kept_rings,
                excluded_rings,
                j_mean,
                j_min,
                j_max,
                ring_spread,
                spread_status,
                k_i_mean,
                k_ii_mean,
                k_iii_mean,
                k_j_plane_strain,
                k_j_plane_stress,
                t_11_mean,
                t_33_mean,
            });
        }
    }

    if !clamped_ids.is_empty() {
        push_diag(
            &mut diags,
            DiagnosticSeverity::Warning,
            "ffs.rings_clamped",
            format!("inner ring exclusion clamped to retain at least 1 ring on: {}", sample_ids(&clamped_ids)),
        );
    }
    if !insufficient_ids.is_empty() {
        push_diag(
            &mut diags,
            DiagnosticSeverity::Warning,
            "ffs.insufficient_rings",
            format!("fewer than 2 kept rings remain for path-independence verification on: {}", sample_ids(&insufficient_ids)),
        );
    }
    if !nonpositive_j_ids.is_empty() {
        push_diag(
            &mut diags,
            DiagnosticSeverity::Warning,
            "ffs.nonpositive_j",
            format!("negative J-integral (possible crack closure/compression) on: {}", sample_ids(&nonpositive_j_ids)),
        );
    }

    let max_ring_spread = profiles.iter().map(|p| p.ring_spread).filter(|v| v.is_finite()).fold(0.0_f64, f64::max);
    let worst_spread_status = profiles.iter().map(|p| p.spread_status).max_by_key(|&s| spread_rank(s)).unwrap_or(SpreadStatus::Unverified);

    (profiles, max_ring_spread, worst_spread_status, diags)
}

/// Evaluate API 579-1 / ASME FFS-1 Part 9 Option 1 or Option 2 allowable toughness ratio $K_{r,\text{allow}}(L_r)$.
pub fn evaluate_fad_curve(option: FadOption, lr: f64, lr_max_eff: f64, opt2: Option<&Option2Evaluator>) -> f64 {
    if !lr.is_finite() || !lr_max_eff.is_finite() || lr < 0.0 || lr_max_eff <= 0.0 || lr > lr_max_eff {
        return 0.0;
    }
    if lr == 0.0 {
        return 1.0;
    }
    match option {
        FadOption::Option1 => {
            let term1 = 1.0 - 0.14 * lr * lr;
            let term2 = 0.3 + 0.7 * (-0.65 * lr.powi(6)).exp();
            let val = term1 * term2;
            if val.is_finite() { val.max(0.0) } else { 0.0 }
        }
        FadOption::Option2 => {
            let Some(eval) = opt2 else {
                return 0.0;
            };
            let sigma_ys = match eval {
                Option2Evaluator::Bilinear { yield_stress, .. } => *yield_stress,
                Option2Evaluator::Table { points, .. } => points.get(1).map_or(0.0, |p| p.1),
                Option2Evaluator::RambergOsgood { sigma_ys, .. } => *sigma_ys,
            };
            if !sigma_ys.is_finite() || sigma_ys <= 0.0 {
                return 0.0;
            }
            let e = eval.young();
            let sigma_ref = lr * sigma_ys;
            let eps_ref = eval.eps_ref(sigma_ref);
            if !eps_ref.is_finite() || eps_ref <= 0.0 {
                return 0.0;
            }
            let ratio = (e * eps_ref) / sigma_ref;
            let denom = ratio + (lr * lr * lr * sigma_ys) / (2.0 * e * eps_ref);
            if !denom.is_finite() || denom <= 0.0 {
                return 0.0;
            }
            let val = denom.powf(-0.5);
            if val.is_finite() { val.max(0.0) } else { 0.0 }
        }
    }
}

/// Evaluate Option 2 curve using explicit reference `sigma_ys` (allowing user yield overrides while
/// keeping `ModelMaterial` constitutive stress-strain transition intact).
fn evaluate_fad_curve_with_ys(option: FadOption, lr: f64, lr_max_eff: f64, sigma_ys: f64, opt2: Option<&Option2Evaluator>) -> f64 {
    if !lr.is_finite() || !lr_max_eff.is_finite() || lr < 0.0 || lr_max_eff <= 0.0 || lr > lr_max_eff {
        return 0.0;
    }
    if lr == 0.0 {
        return 1.0;
    }
    match option {
        FadOption::Option1 => evaluate_fad_curve(FadOption::Option1, lr, lr_max_eff, None),
        FadOption::Option2 => {
            let Some(eval) = opt2 else {
                return 0.0;
            };
            if !sigma_ys.is_finite() || sigma_ys <= 0.0 {
                return 0.0;
            }
            let e = eval.young();
            let sigma_ref = lr * sigma_ys;
            let eps_ref = eval.eps_ref(sigma_ref);
            if !eps_ref.is_finite() || eps_ref <= 0.0 {
                return 0.0;
            }
            let ratio = (e * eps_ref) / sigma_ref;
            let denom = ratio + (lr * lr * lr * sigma_ys) / (2.0 * e * eps_ref);
            if !denom.is_finite() || denom <= 0.0 {
                return 0.0;
            }
            let val = denom.powf(-0.5);
            if val.is_finite() { val.max(0.0) } else { 0.0 }
        }
    }
}

/// Compute radial Reserve Factor (`RF`), safety margin (`RF - 1`), acceptability, limiting mode,
/// and FAD envelope intersection `(RF * L_r, RF * K_r)`.
pub fn reserve_factor(
    l_r: f64,
    k_r: f64,
    lr_max_eff: f64,
    kr_allow_fn: impl Fn(f64) -> f64,
) -> (Option<f64>, Option<f64>, bool, LimitingMode, Option<[f64; 2]>) {
    if !l_r.is_finite() || !k_r.is_finite() || !lr_max_eff.is_finite() || l_r < 0.0 || k_r < 0.0 || lr_max_eff <= 0.0 {
        return (None, None, false, LimitingMode::Fracture, None);
    }

    let kr_at_lr = kr_allow_fn(l_r);
    let acceptable = (l_r <= lr_max_eff) && (k_r <= kr_at_lr + ACCEPT_TOL);

    if l_r == 0.0 && k_r == 0.0 {
        return (None, None, true, LimitingMode::Unloaded, None);
    }
    if k_r == 0.0 {
        let rf = lr_max_eff / l_r;
        return (Some(rf), Some(rf - 1.0), acceptable, LimitingMode::PlasticCollapse, Some([lr_max_eff, 0.0]));
    }
    if l_r == 0.0 {
        let rf = 1.0 / k_r;
        return (Some(rf), Some(rf - 1.0), acceptable, LimitingMode::Fracture, Some([0.0, 1.0]));
    }

    let s_max = lr_max_eff / l_r;
    let k_cut = kr_allow_fn(lr_max_eff);
    if s_max * k_r <= k_cut {
        let rf = s_max;
        return (Some(rf), Some(rf - 1.0), acceptable, LimitingMode::PlasticCollapse, Some([lr_max_eff, rf * k_r]));
    }

    // Exact boundary check so a point placed exactly on K_{r,allow}(L_r) evaluates RF == 1.0.
    let rf = if l_r <= lr_max_eff && (k_r - kr_at_lr).abs() == 0.0 {
        1.0
    } else {
        let mut lo = 0.0_f64;
        let mut hi = s_max;
        for _ in 0..64 {
            let mid = 0.5 * (lo + hi);
            let g = mid * k_r - kr_allow_fn(mid * l_r);
            if !g.is_finite() || g >= 0.0 {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        0.5 * (lo + hi)
    };
    let inter = [rf * l_r, rf * k_r];
    let mode = if inter[0] < 0.2 * lr_max_eff { LimitingMode::Fracture } else { LimitingMode::ElasticPlastic };
    (Some(rf), Some(rf - 1.0), acceptable, mode, Some(inter))
}

#[derive(Debug, Clone, Copy)]
struct MatchedStep {
    warp_step: usize,
    frame_index: Option<usize>,
    increment: usize,
    load_factor: f64,
}

fn check_bc_set_mpc(model: &Model, step_name: &str, bc_nodes: &BTreeSet<u32>) -> bool {
    let bc_iter: Box<dyn Iterator<Item = &BoundaryCondition>> = match model.step(step_name) {
        Some(step) => Box::new(step.boundary_conditions.iter().filter_map(|name| model.boundary_condition(name))),
        None => Box::new(model.boundary_conditions().values()),
    };
    for bc in bc_iter {
        if let BoundaryCondition::Symmetry { target: Target::Set(set_name), plane } = bc
            && plane.axis().is_none()
            && let Some(rset) = model.resolve_set_partial(set_name)
            && rset.nodes.iter().any(|n| bc_nodes.contains(n))
        {
            return true;
        }
    }
    false
}

fn sum_reaction_scalar(vec: [f64; 3], component: ReactionComponent) -> f64 {
    match component {
        ReactionComponent::X => vec[0].abs(),
        ReactionComponent::Y => vec[1].abs(),
        ReactionComponent::Z => vec[2].abs(),
        ReactionComponent::Magnitude => (vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]).sqrt(),
    }
}

fn pick_governing_index(points: &[FadPoint]) -> Option<usize> {
    let mut best: Option<(usize, &FadPoint)> = None;
    for (idx, pt) in points.iter().enumerate() {
        let replace = match best {
            None => true,
            Some((best_idx, best_pt)) => {
                let better_rf = match (pt.reserve_factor, best_pt.reserve_factor) {
                    (Some(a), Some(b)) => a.total_cmp(&b),
                    (Some(_), None) => std::cmp::Ordering::Less,
                    (None, Some(_)) => std::cmp::Ordering::Greater,
                    (None, None) => std::cmp::Ordering::Equal,
                };
                match better_rf {
                    std::cmp::Ordering::Less => true,
                    std::cmp::Ordering::Greater => false,
                    std::cmp::Ordering::Equal => {
                        let r_a = (pt.l_r * pt.l_r + pt.k_r * pt.k_r).sqrt();
                        let r_b = (best_pt.l_r * best_pt.l_r + best_pt.k_r * best_pt.k_r).sqrt();
                        match r_b.total_cmp(&r_a) {
                            std::cmp::Ordering::Less => true,
                            std::cmp::Ordering::Greater => false,
                            std::cmp::Ordering::Equal => match best_pt.l_r.total_cmp(&pt.l_r) {
                                std::cmp::Ordering::Less => true,
                                std::cmp::Ordering::Greater => false,
                                std::cmp::Ordering::Equal => idx < best_idx,
                            },
                        }
                    }
                }
            }
        };
        if replace {
            best = Some((idx, pt));
        }
    }
    best.map(|(i, _)| i)
}

/// Evaluate complete crack-front fracture profiles and optional API 579 Option 1 / Option 2 FAD
/// assessment on SI inputs, returning detailed [`FfsError`] field attributions on parameter error.
pub fn evaluate_step_ffs_detailed(
    model: &Model,
    static_result: &StaticResult,
    domains: &[DomainIntegralResult],
    step_numbers: &[usize],
    domains_truncated: bool,
    req: &FfsRequest,
) -> std::result::Result<FfsReport, FfsError> {
    if let RingExclusion::Count { count } = req.ring_exclusion
        && count > MAX_RING_EXCLUSION
    {
        return Err(FfsError::param("exclude_inner_rings", format!("inner ring exclusion count must be at most {MAX_RING_EXCLUSION}, got {count}")));
    }

    let mut diags = Vec::new();
    if domains_truncated {
        push_diag(
            &mut diags,
            DiagnosticSeverity::Warning,
            "ffs.domains_truncated",
            "WARP3D fracture stdout/parser limits were reached; some crack-front stations or increments may be missing",
        );
    }

    let defaults = extract_material_defaults(model, domains);
    let (chosen_mat_entry, _, _) = select_crack_zone_material(model, domains);

    // Pre-validate FAD elastic overrides so `build_station_profiles` uses consistent (E, nu).
    let elastic_override = if let Some(fad) = &req.fad {
        let e = fad.young.or(defaults.young).ok_or_else(|| {
            FfsError::param("fad/young", "Young's modulus could not be derived from an isotropic crack-zone material; specify `fad/young`")
        })?;
        if !e.is_finite() || e <= 0.0 {
            return Err(FfsError::param("fad/young", "Young's modulus must be a positive finite number"));
        }
        let nu = fad.poisson.or(defaults.poisson).ok_or_else(|| {
            FfsError::param("fad/poisson", "Poisson's ratio could not be derived from an isotropic crack-zone material; specify `fad/poisson`")
        })?;
        if !nu.is_finite() || !(nu > -1.0 && nu < 0.5) {
            return Err(FfsError::param("fad/poisson", "Poisson's ratio must satisfy -1.0 < poisson < 0.5"));
        }
        Some((e, nu))
    } else {
        None
    };

    let history = static_result.nonlinear.as_ref();
    let is_nonlinear = history.is_some();

    let distinct_domain_steps: BTreeSet<usize> = domains.iter().map(|d| d.step).collect();
    let mut matched_steps: Vec<MatchedStep> = Vec::new();
    let mut unmatched_steps: Vec<usize> = Vec::new();

    match history {
        None => {
            if req.active_frame.is_some() {
                return Err(FfsError::param("frame", format!("step `{}` is a linear static step: it has one state and no frames", req.step)));
            }
            if distinct_domain_steps.len() > 1 {
                push_diag(
                    &mut diags,
                    DiagnosticSeverity::Warning,
                    "ffs.step_frame_mismatch",
                    "linear static step contains multiple WARP3D domain steps; selecting step 1",
                );
            }
            if let Some(&warp_step) = if distinct_domain_steps.contains(&1) { distinct_domain_steps.get(&1) } else { distinct_domain_steps.first() } {
                matched_steps.push(MatchedStep { warp_step, frame_index: None, increment: 1, load_factor: 1.0 });
            }
        }
        Some(hist) => {
            if let Some(frame_idx) = req.active_frame
                && hist.frames.get(frame_idx).is_none()
            {
                return Err(FfsError::param(
                    "frame",
                    format!("step `{}` has frames 0 to {}; there is no frame {frame_idx}", req.step, hist.frames.len().saturating_sub(1)),
                ));
            }
            for &dom_step in &distinct_domain_steps {
                let pos = if !step_numbers.is_empty() {
                    step_numbers.iter().position(|&s| s == dom_step)
                } else if hist.frames.len() >= 2 && hist.increments.len() + 1 == hist.frames.len() {
                    dom_step.checked_sub(1).filter(|&p| p < hist.increments.len())
                } else {
                    None
                };
                if let Some(p) = pos {
                    let frame_idx = p + 1;
                    let load_factor = hist
                        .increments
                        .get(p)
                        .map(|inc| inc.load_factor)
                        .or_else(|| hist.frames.get(frame_idx).map(|f| f.load_factor))
                        .unwrap_or(1.0);
                    let increment = hist.increments.get(p).map_or(p + 1, |inc| inc.number);
                    let frame_index = (frame_idx < hist.frames.len()).then_some(frame_idx);
                    matched_steps.push(MatchedStep { warp_step: dom_step, frame_index, increment, load_factor });
                } else {
                    unmatched_steps.push(dom_step);
                }
            }
        }
    }

    if !unmatched_steps.is_empty() {
        let sample: Vec<String> = unmatched_steps.iter().map(usize::to_string).collect();
        push_diag(
            &mut diags,
            DiagnosticSeverity::Warning,
            "ffs.unmatched_domain_step",
            format!("unmatched WARP3D domain step(s) skipped: {}", sample_ids(&sample)),
        );
    }

    // Determine active warp_step, active_frame, and active_load_factor.
    let (active_warp_step, active_frame, active_load_factor) = match history {
        None => (matched_steps.first().map_or(1, |m| m.warp_step), None, 1.0),
        Some(hist) => {
            if let Some(frame_idx) = req.active_frame {
                let lf = hist.frames.get(frame_idx).map_or(0.0, |f| f.load_factor);
                let ws = if frame_idx == 0 {
                    0
                } else {
                    let pos = frame_idx - 1;
                    step_numbers
                        .get(pos)
                        .copied()
                        .or_else(|| matched_steps.iter().find(|m| m.frame_index == Some(frame_idx)).map(|m| m.warp_step))
                        .unwrap_or(frame_idx)
                };
                (ws, Some(frame_idx), lf)
            } else {
                let last_matched = matched_steps.last().copied();
                let last_frame_idx = last_matched.map_or_else(|| hist.frames.len().checked_sub(1), |m| m.frame_index);
                let last_lf = last_matched
                    .map(|m| m.load_factor)
                    .or_else(|| hist.increments.last().map(|inc| inc.load_factor))
                    .or_else(|| hist.frames.last().map(|f| f.load_factor))
                    .unwrap_or(1.0);
                let ws = last_matched.map(|m| m.warp_step).or_else(|| step_numbers.last().copied()).unwrap_or(hist.increments.len());
                (ws, last_frame_idx, last_lf)
            }
        }
    };

    // Build profiles across all matched domains so station `frame_index` and multi-material diagnostics are unified.
    let matched_step_set: BTreeSet<usize> = matched_steps.iter().map(|m| m.warp_step).collect();
    let matched_domains: Vec<DomainIntegralResult> = domains.iter().filter(|d| matched_step_set.contains(&d.step)).cloned().collect();

    let (mut all_profiles, _, _, profile_diags) = build_station_profiles(model, &matched_domains, req.ring_exclusion, elastic_override);
    for d in profile_diags {
        if diags.len() < MAX_DIAGNOSTICS {
            diags.push(d);
        }
    }

    let step_to_frame: BTreeMap<usize, Option<usize>> = matched_steps.iter().map(|m| (m.warp_step, m.frame_index)).collect();
    for p in &mut all_profiles {
        p.frame_index = step_to_frame.get(&p.warp_step).copied().flatten();
    }

    let active_profiles: Vec<StationProfile> =
        if active_warp_step == 0 { Vec::new() } else { all_profiles.iter().filter(|p| p.warp_step == active_warp_step).cloned().collect() };

    let max_ring_spread = active_profiles.iter().map(|p| p.ring_spread).filter(|v| v.is_finite()).fold(0.0_f64, f64::max);
    let worst_spread_status = active_profiles.iter().map(|p| p.spread_status).max_by_key(|&s| spread_rank(s)).unwrap_or(SpreadStatus::Unverified);

    // Optional FAD evaluation.
    let fad_summary = if let Some(fad) = &req.fad {
        let Some((young, poisson)) = elastic_override else {
            return Err(FfsError::param("fad/young", "elastic properties could not be resolved"));
        };
        let sigma_ys = fad
            .sigma_ys
            .or(defaults.sigma_ys)
            .ok_or_else(|| FfsError::param("fad/sigma_ys", "yield strength could not be derived from crack-zone material; specify `fad/sigma_ys`"))?;
        if !sigma_ys.is_finite() || sigma_ys <= 0.0 {
            return Err(FfsError::param("fad/sigma_ys", "yield strength must be a positive finite number"));
        }

        if let Some(uts) = fad.sigma_uts {
            if !uts.is_finite() || uts <= 0.0 {
                return Err(FfsError::param("fad/sigma_uts", "ultimate tensile strength must be a positive finite number"));
            }
            if uts < sigma_ys {
                return Err(FfsError::param("fad/sigma_uts", "ultimate tensile strength must be greater than or equal to yield strength"));
            }
        }

        if let Some(lrm) = fad.l_r_max
            && (!lrm.is_finite() || lrm < 1.0)
        {
            return Err(FfsError::param("fad/l_r_max", "`l_r_max` must be a finite number >= 1.0"));
        }

        let base_lr_max = if let Some(lrm) = fad.l_r_max {
            lrm
        } else if let Some(uts) = fad.sigma_uts {
            (sigma_ys + uts) / (2.0 * sigma_ys)
        } else {
            push_diag(
                &mut diags,
                DiagnosticSeverity::Warning,
                "ffs.default_lr_max",
                "neither `sigma_uts` nor `l_r_max` specified; using conservative plastic collapse cutoff L_r_max = 1.0",
            );
            1.0
        };

        let (effective_lr_max, opt2_eval) = match fad.option {
            FadOption::Option1 => (base_lr_max, None),
            FadOption::Option2 => match &fad.option2_curve {
                Option2Curve::RambergOsgood { alpha, n } => {
                    if !alpha.is_finite() || *alpha <= 0.0 {
                        return Err(FfsError::param("fad/ramberg_alpha", "Ramberg-Osgood alpha must be > 0"));
                    }
                    if !n.is_finite() || *n <= 1.0 {
                        return Err(FfsError::param("fad/ramberg_n", "Ramberg-Osgood exponent n must be > 1"));
                    }
                    (base_lr_max, Some(Option2Evaluator::RambergOsgood { young, sigma_ys, alpha: *alpha, n: *n }))
                }
                Option2Curve::ModelMaterial => {
                    let Some((mat_name, mat)) = chosen_mat_entry else {
                        return Err(FfsError::param(
                            "fad/option2_source",
                            "Option 2 (Model Material) requires a solid material in the model; use Option 1 or Ramberg-Osgood",
                        ));
                    };
                    let Some(plastic) = mat.plastic.as_ref() else {
                        return Err(FfsError::param(
                            "fad/option2_source",
                            format!("material `{mat_name}` has no plasticity curve for Option 2; use Option 1 or Ramberg-Osgood"),
                        ));
                    };
                    match plastic {
                        Plastic::Bilinear { yield_stress, hardening_modulus, .. } => {
                            if !yield_stress.is_finite() || *yield_stress <= 0.0 || !hardening_modulus.is_finite() || *hardening_modulus < 0.0 {
                                return Err(FfsError::param(
                                    "fad/option2_source",
                                    format!("material `{mat_name}` has invalid bilinear plasticity parameters"),
                                ));
                            }
                            let eff = if *hardening_modulus == 0.0 {
                                let plateau = *yield_stress / sigma_ys;
                                let clamped = base_lr_max.min(plateau);
                                if clamped < base_lr_max {
                                    push_diag(
                                        &mut diags,
                                        DiagnosticSeverity::Warning,
                                        "ffs.option2_perfectly_plastic_cutoff",
                                        format!(
                                            "perfectly plastic bilinear material `{mat_name}` (H = 0) caps Option 2 cutoff at L_r_max,eff = {clamped:.4}"
                                        ),
                                    );
                                }
                                clamped
                            } else {
                                base_lr_max
                            };
                            (eff, Some(Option2Evaluator::Bilinear { young, yield_stress: *yield_stress, hardening_modulus: *hardening_modulus }))
                        }
                        Plastic::Table { points } => {
                            if let Err(msg) = validate_table_points(points) {
                                return Err(FfsError::param("fad/option2_source", msg));
                            }
                            let mut curve_pts = Vec::with_capacity(points.len() + 1);
                            curve_pts.push((0.0_f64, 0.0_f64));
                            for pt in points {
                                curve_pts.push((pt.stress / young + pt.strain, pt.stress));
                            }
                            let last_stress = points.last().map_or(sigma_ys, |p| p.stress);
                            let plateau = last_stress / sigma_ys;
                            let eff = base_lr_max.min(plateau);
                            if eff < base_lr_max {
                                push_diag(
                                    &mut diags,
                                    DiagnosticSeverity::Warning,
                                    "ffs.option2_table_plateau",
                                    format!("hardening table for `{mat_name}` caps Option 2 collapse cutoff at table plateau L_r_max,eff = {eff:.4}"),
                                );
                            }
                            (eff, Some(Option2Evaluator::Table { young, points: curve_pts }))
                        }
                        Plastic::JohnsonCook { .. } => {
                            return Err(FfsError::param(
                                "fad/option2_source",
                                format!(
                                    "material `{mat_name}` uses Johnson-Cook plasticity (unsupported by WARP3D static decks and Option 2); use Option 1 or Ramberg-Osgood"
                                ),
                            ));
                        }
                    }
                }
            },
        };

        // Toughness conversion in SI.
        let e_prime = young / (1.0 - poisson * poisson);
        let (k_mat, j_mat) = match fad.toughness {
            ToughnessInput::KMat { value } => {
                if !value.is_finite() || value <= 0.0 {
                    return Err(FfsError::param("fad/k_mat", "`k_mat` must be a positive finite number"));
                }
                (value, Some(value * value / e_prime))
            }
            ToughnessInput::JIc { value, plane_strain } => {
                if !value.is_finite() || value <= 0.0 {
                    return Err(FfsError::param("fad/j_ic", "`j_ic` must be a positive finite number"));
                }
                let e_eff = if plane_strain { e_prime } else { young };
                ((value * e_eff).sqrt(), Some(value))
            }
        };

        // Validate L_r inputs and pre-resolve ReactionRatio node indices.
        enum ResolvedLr<'a> {
            RefStress(f64),
            Direct(f64),
            Reaction { component: ReactionComponent, limit_load: f64, node_indices: Vec<usize>, bc_set: &'a str },
        }
        let resolved_lr = match &fad.lr_input {
            LrInput::ReferenceStress { sigma_ref_full } => {
                if !sigma_ref_full.is_finite() || *sigma_ref_full < 0.0 {
                    return Err(FfsError::param("fad/sigma_ref_full", "`sigma_ref_full` must be a finite number >= 0"));
                }
                ResolvedLr::RefStress(*sigma_ref_full)
            }
            LrInput::DirectLoadRatio { l_r_full } => {
                if !l_r_full.is_finite() || *l_r_full < 0.0 {
                    return Err(FfsError::param("fad/l_r_full", "`l_r_full` must be a finite number >= 0"));
                }
                ResolvedLr::Direct(*l_r_full)
            }
            LrInput::ReactionRatio { bc_set, component, limit_load } => {
                if !limit_load.is_finite() || *limit_load <= 0.0 {
                    return Err(FfsError::param("fad/limit_load", "`limit_load` must be a positive finite number"));
                }
                let Some(rset) = model.resolve_set_partial(bc_set) else {
                    return Err(FfsError::param("fad/bc_set", format!("set `{bc_set}` does not exist")));
                };
                let node_indices: Vec<usize> = rset.nodes.iter().filter_map(|l| static_result.nodes.binary_search(l).ok()).collect();
                if node_indices.is_empty() {
                    return Err(FfsError::param("fad/bc_set", format!("set `{bc_set}` has no nodes in solved step `{}`", req.step)));
                }
                let bc_nodes: BTreeSet<u32> = rset.nodes.iter().copied().collect();
                if check_bc_set_mpc(model, &static_result.step, &bc_nodes) {
                    push_diag(
                        &mut diags,
                        DiagnosticSeverity::Warning,
                        "ffs.reaction_mpc_warning",
                        format!("BC set `{bc_set}` includes inclined/MPC symmetry nodes; verify reaction component alignment"),
                    );
                }
                ResolvedLr::Reaction { component: *component, limit_load: *limit_load, node_indices, bc_set: bc_set.as_str() }
            }
        };

        // Validate domain_id selection up front against matched stations.
        if let StationSelection::DomainId { domain_id } = &fad.station_selection
            && !all_profiles.iter().any(|p| p.domain_id == *domain_id)
        {
            return Err(FfsError::param("fad/domain_id", format!("domain `{domain_id}` was not found in fracture results for step `{}`", req.step)));
        }

        // Check plasticity across model & step results.
        let has_plasticity = defaults.has_plasticity || static_result.elements.iter().any(|e| !e.plastic_strain.is_empty());

        // Check proportional Step-1 scaling readiness when KrSource::Auto or LinearScaledStep1Ki.
        let first_step = matched_steps.first().copied();
        let step1_k_by_domain: BTreeMap<String, f64> = if let Some(fs) = first_step {
            all_profiles
                .iter()
                .filter(|p| p.warp_step == fs.warp_step)
                .filter_map(|p| p.k_i_mean.or(p.k_j_plane_strain).map(|k| (p.domain_id.clone(), k)))
                .collect()
        } else {
            BTreeMap::new()
        };
        let monotonic_positive_lambdas = matched_steps.first().is_some_and(|fs| fs.load_factor > 0.0)
            && matched_steps.windows(2).all(|w| match (w.first(), w.get(1)) {
                (Some(a), Some(b)) => b.load_factor >= a.load_factor && a.load_factor > 0.0,
                _ => true,
            });
        let can_linear_scale_step1 = matched_steps.len() >= 2 && monotonic_positive_lambdas && !step1_k_by_domain.is_empty();

        if fad.kr_source == KrSource::LinearScaledStep1Ki && !can_linear_scale_step1 {
            return Err(FfsError::param(
                "fad/kr_source",
                "`linear_scaled_step1_ki` requires at least 2 matched increments with positive monotonic load factors and Step-1 fracture results",
            ));
        }

        let kr_source_effective = match fad.kr_source {
            KrSource::Auto => {
                if !has_plasticity {
                    KrSource::ElasticKi
                } else if can_linear_scale_step1 {
                    KrSource::LinearScaledStep1Ki
                } else {
                    KrSource::KjPlaneStrain
                }
            }
            explicit => explicit,
        };

        if fad.kr_source == KrSource::ElasticKi && !all_profiles.is_empty() && all_profiles.iter().all(|p| p.k_i_mean.is_none()) {
            return Err(FfsError::param(
                "fad/kr_source",
                "`elastic_ki` requires interaction integral output (`interaction_integral: true`), which is absent on this step",
            ));
        }

        if has_plasticity && matches!(kr_source_effective, KrSource::KjPlaneStrain | KrSource::KjPlaneStress) && !all_profiles.is_empty() {
            push_diag(
                &mut diags,
                DiagnosticSeverity::Warning,
                "ffs.epfm_double_count",
                "plotting elastic-plastic K_J on an Option 1 / Option 2 FAD double-counts crack-tip plasticity already embedded in f(L_r); prefer multi-increment Step-1 elastic K_I scaling",
            );
        }

        let nlgeom_active =
            model.step(&static_result.step).is_some_and(|s| matches!(s.procedure, Procedure::NonlinearStatic(ref nl) if nl.nonlinear_geometry));
        let contact_active = !static_result.contacts.is_empty() || !model.contacts().is_empty();

        let mut negative_lambda_warned = false;
        let mut nonpositive_k_ids = Vec::new();
        let mut reaction_truncated_warned = false;
        let mut reaction_zero_warned = false;
        let mut step1_plastic_warned = false;

        let kr_allow_fn = |lr: f64| evaluate_fad_curve_with_ys(fad.option, lr, effective_lr_max, sigma_ys, opt2_eval.as_ref());

        // Helper closure to compute L_r and optional reaction scalar for one matched step.
        let mut eval_step_lr = |ms: MatchedStep, diags: &mut Vec<FfsDiagnostic>| -> Option<(f64, Option<f64>)> {
            if ms.load_factor < 0.0 && !negative_lambda_warned {
                push_diag(
                    diags,
                    DiagnosticSeverity::Warning,
                    "ffs.negative_load_factor",
                    "negative load factor encountered; L_r uses |lambda| magnitude",
                );
                negative_lambda_warned = true;
            }
            match &resolved_lr {
                ResolvedLr::RefStress(sref) => Some((ms.load_factor.abs() * (*sref) / sigma_ys, None)),
                ResolvedLr::Direct(lr_full) => Some((ms.load_factor.abs() * (*lr_full), None)),
                ResolvedLr::Reaction { component, limit_load, node_indices, bc_set } => {
                    if let Some(hist) = history {
                        let Some(frame_idx) = ms.frame_index else {
                            if !reaction_truncated_warned {
                                push_diag(
                                    diags,
                                    DiagnosticSeverity::Warning,
                                    "ffs.reaction_truncated",
                                    format!(
                                        "frame reactions unavailable past MAX_FRAME_VALUES at WARP3D step {}; omitting ReactionRatio FAD point(s)",
                                        ms.warp_step
                                    ),
                                );
                                reaction_truncated_warned = true;
                            }
                            return None;
                        };
                        let Some(frame) = hist.frames.get(frame_idx) else {
                            if !reaction_truncated_warned {
                                push_diag(
                                    diags,
                                    DiagnosticSeverity::Warning,
                                    "ffs.reaction_truncated",
                                    format!("frame {frame_idx} missing from history; omitting ReactionRatio FAD point(s)"),
                                );
                                reaction_truncated_warned = true;
                            }
                            return None;
                        };
                        if frame_idx == 0 {
                            return Some((0.0, Some(0.0)));
                        }
                        if frame.reaction.is_empty() {
                            if ms.load_factor.abs() > 0.0 && !reaction_zero_warned {
                                push_diag(
                                    diags,
                                    DiagnosticSeverity::Warning,
                                    "ffs.reaction_zero_or_missing",
                                    format!("frame {frame_idx} has no nodal reactions on `{bc_set}` (wnr* missing or zero reaction)"),
                                );
                                reaction_zero_warned = true;
                            }
                            return Some((0.0, Some(0.0)));
                        }
                        let mut sum = [0.0; 3];
                        for &i in node_indices {
                            let f = frame.reaction_at(i);
                            sum[0] += f[0];
                            sum[1] += f[1];
                            sum[2] += f[2];
                        }
                        let r_val = sum_reaction_scalar(sum, *component);
                        if r_val == 0.0 && ms.load_factor.abs() > 0.0 && !reaction_zero_warned {
                            push_diag(
                                diags,
                                DiagnosticSeverity::Warning,
                                "ffs.reaction_zero_or_missing",
                                format!("nodal reaction sum on `{bc_set}` is zero at non-zero load factor {}", ms.load_factor),
                            );
                            reaction_zero_warned = true;
                        }
                        Some((r_val / (*limit_load), Some(r_val)))
                    } else {
                        if static_result.reaction.is_empty() {
                            if !reaction_zero_warned {
                                push_diag(
                                    diags,
                                    DiagnosticSeverity::Warning,
                                    "ffs.reaction_zero_or_missing",
                                    format!("no nodal reactions stored for step `{}` on `{bc_set}`", req.step),
                                );
                                reaction_zero_warned = true;
                            }
                            return Some((0.0, Some(0.0)));
                        }
                        let mut sum = [0.0; 3];
                        for &i in node_indices {
                            let f = static_result.reaction.get(i).copied().unwrap_or([0.0; 3]);
                            sum[0] += f[0];
                            sum[1] += f[1];
                            sum[2] += f[2];
                        }
                        let r_val = sum_reaction_scalar(sum, *component);
                        if r_val == 0.0 && !reaction_zero_warned {
                            push_diag(
                                diags,
                                DiagnosticSeverity::Warning,
                                "ffs.reaction_zero_or_missing",
                                format!("nodal reaction sum on `{bc_set}` is zero"),
                            );
                            reaction_zero_warned = true;
                        }
                        Some((r_val / (*limit_load), Some(r_val)))
                    }
                }
            }
        };

        // Group profiles by warp_step for fast increment lookup.
        let mut profiles_by_step: BTreeMap<usize, Vec<&StationProfile>> = BTreeMap::new();
        for p in &all_profiles {
            profiles_by_step.entry(p.warp_step).or_default().push(p);
        }

        let mut trajectory = Vec::with_capacity(matched_steps.len());
        let mut active_stations = Vec::new();

        for (step_idx, &ms) in matched_steps.iter().enumerate() {
            let Some((l_r, reaction_value)) = eval_step_lr(ms, &mut diags) else {
                continue;
            };
            if step_idx == 0
                && kr_source_effective == KrSource::LinearScaledStep1Ki
                && !step1_plastic_warned
                && (l_r >= 0.5 || nlgeom_active || contact_active)
            {
                push_diag(
                    &mut diags,
                    DiagnosticSeverity::Warning,
                    "ffs.epfm_step1_may_be_plastic",
                    "proportional scaling from Step-1 K_I assumes Increment 1 is linear-elastic; Step-1 L_r >= 0.5, nlgeom, or contact may introduce approximation",
                );
                step1_plastic_warned = true;
            }

            let step_profiles = profiles_by_step.get(&ms.warp_step).map(Vec::as_slice).unwrap_or(&[]);
            let mut step_points = Vec::with_capacity(step_profiles.len());
            for prof in step_profiles {
                let (used_src, raw_k) = match kr_source_effective {
                    KrSource::Auto | KrSource::ElasticKi => {
                        if let Some(ki) = prof.k_i_mean {
                            (KrSource::ElasticKi, Some(ki))
                        } else {
                            (KrSource::KjPlaneStrain, prof.k_j_plane_strain)
                        }
                    }
                    KrSource::LinearScaledStep1Ki => {
                        let lambda_1 = first_step.map_or(1.0, |fs| fs.load_factor);
                        let k_1 = step1_k_by_domain.get(&prof.domain_id).copied();
                        let scaled = k_1.filter(|_| lambda_1 > 0.0).map(|k1| k1 * (ms.load_factor / lambda_1));
                        (KrSource::LinearScaledStep1Ki, scaled)
                    }
                    KrSource::KjPlaneStrain => (KrSource::KjPlaneStrain, prof.k_j_plane_strain),
                    KrSource::KjPlaneStress => (KrSource::KjPlaneStress, prof.k_j_plane_stress),
                };

                let k_eff = raw_k.filter(|k| k.is_finite()).map(|k| {
                    if k < 0.0 {
                        nonpositive_k_ids.push(prof.domain_id.clone());
                        0.0
                    } else {
                        k
                    }
                });

                let k_r = k_eff.map_or(0.0, |k| k / k_mat);
                let k_r_allow = kr_allow_fn(l_r);
                let (reserve_factor, safety_margin, acceptable, limiting_mode, intersection) =
                    reserve_factor(l_r, k_r, effective_lr_max, kr_allow_fn);

                let epfm_k_j_ratio = prof.k_j_plane_strain.filter(|k| k.is_finite()).map(|kj| kj.max(0.0) / k_mat);
                let epfm_j_ratio = match (prof.j_mean, j_mat) {
                    (jm, Some(jmat)) if jm.is_finite() && jm >= 0.0 && jmat > 0.0 => Some(jm / jmat),
                    _ => None,
                };

                step_points.push(FadPoint {
                    domain_id: prof.domain_id.clone(),
                    warp_step: ms.warp_step,
                    frame_index: ms.frame_index,
                    increment: ms.increment,
                    load_factor: ms.load_factor,
                    l_r,
                    k_r,
                    k_r_allow,
                    k_eff,
                    kr_source_used: used_src,
                    reserve_factor,
                    safety_margin,
                    acceptable,
                    limiting_mode,
                    intersection,
                    epfm_k_j_ratio,
                    epfm_j_ratio,
                    reaction_value,
                });
            }

            let selected_idx = match &fad.station_selection {
                StationSelection::Governing => pick_governing_index(&step_points),
                StationSelection::DomainId { domain_id } => step_points.iter().position(|pt| pt.domain_id == *domain_id),
            };
            if let Some(idx) = selected_idx
                && let Some(pt) = step_points.get(idx).cloned()
            {
                trajectory.push(pt);
            }

            if ms.warp_step == active_warp_step {
                active_stations = step_points;
            }
        }

        if !nonpositive_k_ids.is_empty() {
            push_diag(
                &mut diags,
                DiagnosticSeverity::Warning,
                "ffs.nonpositive_k",
                format!("negative effective stress intensity clamped to K_r = 0 on: {}", sample_ids(&nonpositive_k_ids)),
            );
        }

        let active_verdict = if active_frame == Some(0) && is_nonlinear {
            Some(FadPoint {
                domain_id: String::new(),
                warp_step: 0,
                frame_index: Some(0),
                increment: 0,
                load_factor: active_load_factor,
                l_r: 0.0,
                k_r: 0.0,
                k_r_allow: 1.0,
                k_eff: Some(0.0),
                kr_source_used: kr_source_effective,
                reserve_factor: None,
                safety_margin: None,
                acceptable: true,
                limiting_mode: LimitingMode::Unloaded,
                intersection: None,
                epfm_k_j_ratio: None,
                epfm_j_ratio: None,
                reaction_value: matches!(resolved_lr, ResolvedLr::Reaction { .. }).then_some(0.0),
            })
        } else {
            let idx = match &fad.station_selection {
                StationSelection::Governing => pick_governing_index(&active_stations),
                StationSelection::DomainId { domain_id } => active_stations.iter().position(|pt| pt.domain_id == *domain_id),
            };
            idx.and_then(|i| active_stations.get(i).cloned())
        };

        let mut envelope = Vec::with_capacity(FAD_CURVE_POINTS);
        for i in 0..FAD_CURVE_POINTS {
            let lr = effective_lr_max * (i as f64) / ((FAD_CURVE_POINTS - 1) as f64);
            let kr = kr_allow_fn(lr);
            envelope.push([lr, kr]);
        }

        Some(FadSummary {
            option: fad.option,
            effective_lr_max,
            k_mat,
            j_mat,
            sigma_ys,
            sigma_uts: fad.sigma_uts,
            envelope,
            trajectory,
            active_stations,
            active_verdict,
        })
    } else {
        None
    };

    Ok(FfsReport {
        step: req.step.clone(),
        active_warp_step,
        active_frame,
        active_load_factor,
        is_nonlinear,
        defaults,
        active_profiles,
        max_ring_spread,
        worst_spread_status,
        fad: fad_summary,
        diagnostics: diags,
    })
}

/// Evaluate complete crack-front fracture profiles and optional API 579 Option 1 / Option 2 FAD
/// assessment on SI inputs.
pub fn evaluate_step_ffs(
    model: &Model,
    static_result: &StaticResult,
    domains: &[DomainIntegralResult],
    step_numbers: &[usize],
    domains_truncated: bool,
    req: &FfsRequest,
) -> Result<FfsReport> {
    evaluate_step_ffs_detailed(model, static_result, domains, step_numbers, domains_truncated, req).map_err(|e| Warp3dError::Unsupported(e.message))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use fieldcraft_model::{
        BoundaryCondition, Elastic, ElementKind, Hardening, HardeningPoint, Material, Model, NonlinearStatic, Plastic, Procedure, Section,
        SectionKind, Set, Step, Symmetry, Target,
    };
    use fieldcraft_solver::Stats;
    use fieldcraft_solver::nonlinear::{Frame, History, Increment};

    use super::*;

    fn hex8_test_model(plastic: Option<Plastic>) -> Model {
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
        model.add_set("all", Set { elements: vec![1], ..Set::default() }).unwrap();
        model.add_set("fix", Set { nodes: vec![1, 2, 3, 4], ..Set::default() }).unwrap();
        model
            .add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 210.0e9, poisson: 0.3 }), plastic, ..Default::default() })
            .unwrap();
        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("fix".into()) }).unwrap();
        let mut step = Step::new("Load", Procedure::Static {});
        step.boundary_conditions.insert("clamp".into());
        model.add_step(step, None).unwrap();
        model
    }

    fn base_static_result(step: &str, nodes: Vec<u32>, nonlinear: Option<History>) -> StaticResult {
        let n = nodes.len();
        StaticResult {
            step: step.to_string(),
            nodes,
            displacement: vec![[0.0; 3]; n],
            rotation: Vec::new(),
            reaction: vec![[0.0; 3]; n],
            reaction_moment: Vec::new(),
            stress: Vec::new(),
            von_mises: Vec::new(),
            elements: Vec::new(),
            shells: Vec::new(),
            beams: Vec::new(),
            reference_points: Vec::new(),
            applied_total: [0.0; 3],
            reaction_total: [0.0; 3],
            stats: Stats::default(),
            warnings: Vec::new(),
            temperature: None,
            nonlinear,
            contacts: Vec::new(),
            table_ranges: Vec::new(),
        }
    }

    fn sample_domain(id: &str, step: usize, front_nodes: Vec<u32>, ring_js: &[(usize, f64)], ring_kis: Option<&[f64]>) -> DomainIntegralResult {
        let rings: Vec<RingIntegral> = ring_js
            .iter()
            .enumerate()
            .map(|(idx, &(ring, j))| RingIntegral {
                ring,
                j,
                k_i: ring_kis.and_then(|ks| ks.get(idx).copied()),
                k_ii: Some(0.0),
                k_iii: Some(0.0),
                t_11: Some(-10.0e6),
                t_33: Some(-3.0e6),
            })
            .collect();
        let (sum, lo, hi) = rings.iter().fold((0.0, f64::INFINITY, f64::NEG_INFINITY), |(s, l, h), r| (s + r.j, l.min(r.j), h.max(r.j)));
        let j_mean = if rings.is_empty() { 10.0e3 } else { sum / rings.len() as f64 };
        let j_min = if rings.is_empty() { 10.0e3 } else { lo };
        let j_max = if rings.is_empty() { 10.0e3 } else { hi };
        let ring_spread = (j_max - j_min).abs() / j_mean.abs().max(MIN_J_DENOM);
        let k_i_mean = ring_kis.map(|ks| ks.iter().sum::<f64>() / ks.len() as f64);
        DomainIntegralResult {
            domain_id: id.into(),
            step,
            front_nodes,
            s_position: None,
            normal: [0.0, 1.0, 0.0],
            tangent: [0.0, 0.0, 1.0],
            advance: [1.0, 0.0, 0.0],
            area_aq: 1.0e-4,
            rings,
            j_mean,
            j_min,
            j_max,
            ring_spread,
            j_total: Some(j_mean),
            k_j_plane_strain: Some(50.0e6),
            k_j_plane_stress: Some(47.0e6),
            k_i_mean,
            k_ii_mean: Some(0.0),
            k_iii_mean: Some(0.0),
            t_11_mean: Some(-10.0e6),
            t_33_mean: Some(-3.0e6),
        }
    }

    #[test]
    fn api579_option1_closed_form_oracle_and_cutoff() {
        let lr_max = 1.15;
        assert!((evaluate_fad_curve(FadOption::Option1, 0.0, lr_max, None) - 1.0).abs() < 1e-12);
        for lr in [0.0_f64, 0.25, 0.5, 0.8, 1.0, 1.15] {
            let expected = (1.0 - 0.14 * lr * lr) * (0.3 + 0.7 * (-0.65 * lr.powi(6)).exp());
            let got = evaluate_fad_curve(FadOption::Option1, lr, lr_max, None);
            assert!((got - expected).abs() < 1e-12, "lr={lr}: got {got}, expected {expected}");
        }
        assert_eq!(evaluate_fad_curve(FadOption::Option1, 1.1500001, lr_max, None), 0.0);
        assert_eq!(evaluate_fad_curve(FadOption::Option1, -0.1, lr_max, None), 0.0);
    }

    #[test]
    fn api579_option2_elastic_equivalence_bilinear_table_ro_and_refusals() {
        let young = 200.0e9;
        let sigma_ys = 400.0e6;

        // 1. Bilinear with H > 0: strictly elastic segment (Lr <= 1.0) matches (1 + 0.5 Lr^2)^(-1/2).
        let bil = Option2Evaluator::Bilinear { young, yield_stress: sigma_ys, hardening_modulus: 2.0e9 };
        for lr in [0.2_f64, 0.5, 0.9, 1.0] {
            let expected = (1.0_f64 + 0.5 * lr * lr).powf(-0.5);
            let got = evaluate_fad_curve(FadOption::Option2, lr, 1.25, Some(&bil));
            assert!((got - expected).abs() < 1e-12, "bilinear elastic lr={lr}: got {got}, expected {expected}");
        }
        let got_plastic = evaluate_fad_curve(FadOption::Option2, 1.1, 1.25, Some(&bil));
        assert!(got_plastic < (1.0_f64 + 0.5 * 1.1 * 1.1).powf(-0.5) && got_plastic > 0.0);

        // 2. Table below sigma_0 / sigma_ys matches (1 + 0.5 Lr^2)^(-1/2) to 1e-12.
        let tbl = Option2Evaluator::Table { young, points: vec![(0.0, 0.0), (sigma_ys / young, sigma_ys), (500.0e6 / young + 0.02, 500.0e6)] };
        for lr in [0.1_f64, 0.6, 1.0] {
            let expected = (1.0_f64 + 0.5 * lr * lr).powf(-0.5);
            let got = evaluate_fad_curve(FadOption::Option2, lr, 1.25, Some(&tbl));
            assert!((got - expected).abs() < 1e-12, "table elastic lr={lr}: got {got}, expected {expected}");
        }

        // 3. Ramberg-Osgood includes plastic strain alpha * Lr^n for all Lr > 0 (verified against closed form).
        let (alpha, n) = (0.002 * young / sigma_ys, 5.0);
        let ro = Option2Evaluator::RambergOsgood { young, sigma_ys, alpha, n };
        for lr in [0.5_f64, 1.0, 1.2] {
            let eps_ratio = 1.0 + alpha * lr.powf(n - 1.0); // (E * eps_ref) / sigma_ref
            let expected = (eps_ratio + 0.5 * lr * lr / eps_ratio).powf(-0.5);
            let got = evaluate_fad_curve(FadOption::Option2, lr, 1.3, Some(&ro));
            assert!((got - expected).abs() < 1e-12, "RO lr={lr}: got {got}, expected {expected}");
        }

        // 4. Perfectly plastic Bilinear (H == 0) clamps lr_max_eff = min(lr_max, 1.0) with continuous K_r_allow(1.0) = (1.5)^(-0.5).
        let model_pp = hex8_test_model(Some(Plastic::Bilinear { yield_stress: sigma_ys, hardening_modulus: 0.0, hardening: Hardening::Isotropic }));
        let sr = base_static_result("Load", vec![1, 2, 3, 4], None);
        let dom = sample_domain("cf_0001", 1, vec![1], &[(1, 10.0e3), (2, 10.0e3)], Some(&[40.0e6, 40.0e6]));
        let req_pp = FfsRequest {
            step: "Load".into(),
            ring_exclusion: RingExclusion::Auto,
            active_frame: None,
            fad: Some(FadConfig {
                option: FadOption::Option2,
                option2_curve: Option2Curve::ModelMaterial,
                lr_input: LrInput::DirectLoadRatio { l_r_full: 1.0 },
                toughness: ToughnessInput::KMat { value: 100.0e6 },
                kr_source: KrSource::Auto,
                station_selection: StationSelection::Governing,
                sigma_ys: None,
                sigma_uts: Some(550.0e6),
                l_r_max: None,
                young: None,
                poisson: None,
            }),
        };
        let rep_pp = evaluate_step_ffs(&model_pp, &sr, std::slice::from_ref(&dom), &[], false, &req_pp).unwrap();
        let fad_pp = rep_pp.fad.unwrap();
        assert!((fad_pp.effective_lr_max - 1.0).abs() < 1e-12);
        assert!(rep_pp.diagnostics.iter().any(|d| d.code == "ffs.option2_perfectly_plastic_cutoff"));
        let v_at_1 = fad_pp.active_verdict.unwrap();
        assert!((v_at_1.k_r_allow - (1.5_f64).powf(-0.5)).abs() < 1e-12);

        // 5. Plastic::Table plateau clamping when l_r_max > sigma_last / sigma_ys.
        let model_tbl = hex8_test_model(Some(Plastic::Table {
            points: vec![HardeningPoint { strain: 0.0, stress: 400.0e6 }, HardeningPoint { strain: 0.05, stress: 440.0e6 }],
        }));
        let mut req_tbl = req_pp.clone();
        req_tbl.fad.as_mut().unwrap().l_r_max = Some(1.3);
        let rep_tbl = evaluate_step_ffs(&model_tbl, &sr, std::slice::from_ref(&dom), &[], false, &req_tbl).unwrap();
        let fad_tbl = rep_tbl.fad.unwrap();
        assert!((fad_tbl.effective_lr_max - 1.1).abs() < 1e-12);
        assert!(rep_tbl.diagnostics.iter().any(|d| d.code == "ffs.option2_table_plateau"));

        // 6. Rejection of softening Table (unvalidated JSON-style or manual model) and JohnsonCook.
        let mut model_soft = hex8_test_model(None);
        model_soft
            .replace_material(
                "Steel",
                Material {
                    elastic: Some(Elastic::Isotropic { young, poisson: 0.3 }),
                    plastic: Some(Plastic::Table {
                        points: vec![HardeningPoint { strain: 0.0, stress: 400.0e6 }, HardeningPoint { strain: 0.05, stress: 380.0e6 }],
                    }),
                    ..Default::default()
                },
            )
            .unwrap();
        let err_soft = evaluate_step_ffs_detailed(&model_soft, &sr, std::slice::from_ref(&dom), &[], false, &req_pp).unwrap_err();
        assert_eq!(err_soft.path, Some("fad/option2_source"));

        let model_jc = hex8_test_model(Some(Plastic::JohnsonCook {
            a: 400.0e6,
            b: 100.0e6,
            n: 0.2,
            c: 0.0,
            m: 1.0,
            reference_strain_rate: 1.0,
            room_temperature: 293.15,
            melt_temperature: 1700.0,
            specific_heat: None,
            taylor_quinney: 0.9,
        }));
        let err_jc = evaluate_step_ffs_detailed(&model_jc, &sr, &[dom], &[], false, &req_pp).unwrap_err();
        assert_eq!(err_jc.path, Some("fad/option2_source"));
    }

    #[test]
    fn radial_reserve_factor_invariants_across_all_limiting_modes() {
        let lr_max = 1.15;
        let curve = |lr: f64| evaluate_fad_curve(FadOption::Option1, lr, lr_max, None);

        // Unloaded
        let (rf, sm, ok, mode, inter) = reserve_factor(0.0, 0.0, lr_max, curve);
        assert_eq!((rf, sm, ok, mode, inter), (None, None, true, LimitingMode::Unloaded, None));

        // Pure Fracture (L_r == 0)
        let (rf, _, ok, mode, inter) = reserve_factor(0.0, 0.5, lr_max, curve);
        assert!((rf.unwrap() - 2.0).abs() < 1e-12);
        assert!(ok);
        assert_eq!(mode, LimitingMode::Fracture);
        assert_eq!(inter, Some([0.0, 1.0]));

        // Pure PlasticCollapse (K_r == 0)
        let (rf, _, ok, mode, inter) = reserve_factor(0.5, 0.0, lr_max, curve);
        assert!((rf.unwrap() - 2.3).abs() < 1e-12);
        assert!(ok);
        assert_eq!(mode, LimitingMode::PlasticCollapse);
        assert_eq!(inter, Some([lr_max, 0.0]));

        // General points across Fracture, ElasticPlastic, and PlasticCollapse vertical drop (both Opt1 & Opt2 H=0)
        let cases = [
            (0.05, 0.80, true, LimitingMode::Fracture),
            (0.05, 1.20, false, LimitingMode::Fracture),
            (0.50, 0.50, true, LimitingMode::ElasticPlastic),
            (0.75, 0.95, false, LimitingMode::ElasticPlastic),
            (1.00, 0.20, true, LimitingMode::PlasticCollapse),
            (1.20, 0.15, false, LimitingMode::PlasticCollapse),
        ];
        for (lr, kr, expect_ok, expect_mode) in cases {
            let (rf_opt, sm_opt, ok, mode, inter_opt) = reserve_factor(lr, kr, lr_max, curve);
            let rf = rf_opt.unwrap();
            assert_eq!(ok, expect_ok, "lr={lr}, kr={kr}, rf={rf}");
            assert_eq!(rf >= 1.0, ok, "RF >= 1 <=> acceptable failed at ({lr}, {kr})");
            assert!((sm_opt.unwrap() - (rf - 1.0)).abs() < 1e-12);
            assert_eq!(mode, expect_mode, "lr={lr}, kr={kr}");
            let inter = inter_opt.unwrap();
            if mode == LimitingMode::PlasticCollapse {
                assert!((inter[0] - lr_max).abs() < 1e-12);
                assert!(inter[1] <= curve(lr_max) + 1e-12);
            } else {
                assert!((inter[1] - curve(inter[0])).abs() < 1e-12);
            }
        }

        // Exact boundary points on curve & vertical cutoff
        for lr in [0.1, 0.6, 1.15] {
            let kr = curve(lr);
            let (rf, _, ok, _, _) = reserve_factor(lr, kr, lr_max, curve);
            assert!(ok);
            assert!((rf.unwrap() - 1.0).abs() < 1e-12);
        }
        let (rf_cut, _, ok_cut, mode_cut, _) = reserve_factor(lr_max, curve(lr_max) * 0.5, lr_max, curve);
        assert!(ok_cut);
        assert!((rf_cut.unwrap() - 1.0).abs() < 1e-12);
        assert_eq!(mode_cut, LimitingMode::PlasticCollapse);

        // Option 2 H=0 vertical drop at Lr = 1.0
        let bil_pp = Option2Evaluator::Bilinear { young: 200e9, yield_stress: 400e6, hardening_modulus: 0.0 };
        let curve_pp = |lr: f64| evaluate_fad_curve(FadOption::Option2, lr, 1.0, Some(&bil_pp));
        let (rf_pp, _, ok_pp, mode_pp, inter_pp) = reserve_factor(0.8, 0.4, 1.0, curve_pp);
        assert!(ok_pp);
        assert_eq!(mode_pp, LimitingMode::PlasticCollapse);
        assert!((rf_pp.unwrap() - 1.25).abs() < 1e-12);
        assert!((inter_pp.unwrap()[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn ring_exclusion_spread_boundaries_empty_rings_and_multi_material_defaults() {
        let mut model = hex8_test_model(Some(Plastic::Bilinear { yield_stress: 350.0e6, hardening_modulus: 1.0e9, hardening: Hardening::Isotropic }));
        model.set_node(10, [0.0, 0.0, 0.0]).unwrap();
        model.set_node(11, [0.0, 0.0, 1.0]).unwrap();
        model.set_node(12, [0.0, 0.0, 2.0]).unwrap();
        // Multi-material element touching node 2 with lower yield (250 MPa) vs Steel (350 MPa)
        model.set_element(2, ElementKind::Hex8, vec![2, 3, 4, 1, 6, 7, 8, 5]).unwrap();
        model
            .add_material(
                "Alu",
                Material {
                    elastic: Some(Elastic::Isotropic { young: 70.0e9, poisson: 0.33 }),
                    plastic: Some(Plastic::Bilinear { yield_stress: 250.0e6, hardening_modulus: 500.0e6, hardening: Hardening::Isotropic }),
                    ..Default::default()
                },
            )
            .unwrap();
        model.add_set("sec2_set", Set { elements: vec![2], ..Set::default() }).unwrap();
        model
            .add_section(
                "sec2",
                Section { set: "sec2_set".into(), material: "Alu".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();

        let d0 = sample_domain("d0_empty", 1, vec![], &[], None);
        let d1 = sample_domain("d1_one", 1, vec![10], &[(1, 100.0)], None);
        let d2 = sample_domain("d2_two", 1, vec![10, 11, 12], &[(2, 100.0), (4, 105.0)], None); // sparse IDs, exact 0.05 spread on Good boundary
        let d3 = sample_domain("d3_three", 1, vec![11], &[(1, 50.0), (2, 100.0), (3, 105.0)], None); // Auto drops Ring 1 -> Good (<= 0.05)
        let d5 = sample_domain("d5_five", 1, vec![12], &[(5, 100.0), (1, 80.0), (2, 100.0), (3, 104.0), (4, 105.0)], None);

        // Exact boundary checks: 0.05 Good, 0.0500001 Caution, 0.10 Caution, 0.1000001 Poor
        // mean = 100, span = 5 -> 0.05; span = 5.00001 -> 0.0500001; span = 10 -> 0.10; span = 10.00001 -> 0.1000001
        let d_b1 = sample_domain("b_good", 1, vec![10], &[(2, 97.5), (3, 102.5)], None);
        let d_b2 = sample_domain("b_caut_lo", 1, vec![10], &[(2, 97.499995), (3, 102.500005)], None);
        let d_b3 = sample_domain("b_caut_hi", 1, vec![10], &[(2, 95.0), (3, 105.0)], None);
        let d_b4 = sample_domain("b_poor", 1, vec![10], &[(2, 94.999995), (3, 105.000005)], None);
        let d_neg = sample_domain("b_neg", 1, vec![10], &[(2, -5.0), (3, -5.2)], None);
        let d_zero = sample_domain("b_zero", 1, vec![10], &[(2, 1e-15), (3, -1e-15)], None);

        let all = vec![d0, d1, d2, d3, d5, d_b1, d_b2, d_b3, d_b4, d_neg, d_zero];

        // Deterministic multi-material pick: Alu (250 MPa < 350 MPa)
        let defs = extract_material_defaults(&model, &all);
        assert_eq!(defs.sigma_ys, Some(250.0e6));
        assert_eq!(defs.material_names, vec!["Alu".to_string(), "Steel".to_string()]);

        let (profs, _, worst, diags) = build_station_profiles(&model, &all, RingExclusion::Auto, None);
        assert_eq!(profs[0].total_rings, 0);
        assert_eq!(profs[0].kept_rings, 0);
        assert_eq!(profs[0].focus_node, None);
        assert_eq!(profs[0].spread_status, SpreadStatus::Unverified);

        assert_eq!(profs[1].kept_rings, 1);
        assert_eq!(profs[1].spread_status, SpreadStatus::Unverified);

        assert_eq!(profs[2].focus_node, Some(11)); // FrontAverage middle node
        assert_eq!(profs[2].excluded_rings, Vec::<usize>::new()); // total=2 -> Auto keeps all
        assert_eq!(profs[2].kept_rings, 2);

        assert_eq!(profs[3].excluded_rings, vec![1]); // total=3 -> Auto drops Ring 1
        assert_eq!(profs[3].kept_rings, 2);
        // (105 - 100) / 102.5 = 5 / 102.5 = 0.04878 <= 0.05 => Good!
        assert_eq!(profs[3].spread_status, SpreadStatus::Good);
        assert_eq!(profs[4].excluded_rings, vec![1]); // sorted even when input unsorted
        assert_eq!(profs[4].kept_rings, 4);

        assert_eq!(profs[5].spread_status, SpreadStatus::Good);
        assert_eq!(profs[6].spread_status, SpreadStatus::Caution);
        assert_eq!(profs[7].spread_status, SpreadStatus::Caution);
        assert_eq!(profs[8].spread_status, SpreadStatus::Poor);
        assert_eq!(profs[9].spread_status, SpreadStatus::Good); // |-5.1| > MIN_J_DENOM, spread <= 0.05, and emits ffs.nonpositive_j
        assert_eq!(profs[10].spread_status, SpreadStatus::Unverified); // |j_mean| <= MIN_J_DENOM
        assert_eq!(worst, SpreadStatus::Poor);
        assert!(diags.iter().any(|d| d.code == "ffs.multiple_crack_materials"));
        assert!(diags.iter().any(|d| d.code == "ffs.insufficient_rings"));
        assert!(diags.iter().any(|d| d.code == "ffs.nonpositive_j"));

        // Explicit clamping Count(99) clamped to keep at least 1 ring + Count(1) on 1-ring domain
        let d_single = vec![sample_domain("one_r", 1, vec![10], &[(1, 10.0)], None)];
        let (p_clamp, _, _, d_clamp) = build_station_profiles(&model, &d_single, RingExclusion::Count { count: 99 }, None);
        assert_eq!(p_clamp[0].kept_rings, 1);
        assert!(d_clamp.iter().any(|d| d.code == "ffs.rings_clamped"));

        // Physical arc length check on nodes 10 -> 11 -> 12 (z = 0, 1, 2)
        let d_arc = vec![
            sample_domain("s1", 1, vec![10], &[(2, 10.0), (3, 10.0)], None),
            sample_domain("s2", 1, vec![11], &[(2, 10.0), (3, 10.0)], None),
            sample_domain("s3", 1, vec![12], &[(2, 10.0), (3, 10.0)], None),
        ];
        let (p_arc, _, _, _) = build_station_profiles(&model, &d_arc, RingExclusion::Count { count: 0 }, None);
        assert!((p_arc[0].s_length.unwrap() - 0.0).abs() < 1e-12);
        assert!((p_arc[1].s_length.unwrap() - 1.0).abs() < 1e-12);
        assert!((p_arc[2].s_length.unwrap() - 2.0).abs() < 1e-12);
        assert!((p_arc[1].s_norm - 0.5).abs() < 1e-12);
    }

    #[test]
    fn sparse_steps_frame_truncation_reaction_ratio_and_epfm_guard() {
        let mut model = hex8_test_model(Some(Plastic::Bilinear { yield_stress: 400.0e6, hardening_modulus: 1.0e9, hardening: Hardening::Isotropic }));
        // Replace clamp with inclined symmetry MPC on fix set to test ffs.reaction_mpc_warning + ReactionRatio
        model.add_set("inclined", Set { nodes: vec![1, 2], ..Set::default() }).unwrap();
        model
            .add_boundary_condition(
                "sym_inc",
                BoundaryCondition::Symmetry { target: Target::Set("inclined".into()), plane: Symmetry::Normal([1.0, 1.0, 0.0]) },
            )
            .unwrap();
        let mut nl_step = Step::new("Pull", Procedure::NonlinearStatic(NonlinearStatic::default()));
        nl_step.boundary_conditions.insert("clamp".into());
        nl_step.boundary_conditions.insert("sym_inc".into());
        model.add_step(nl_step, None).unwrap();

        // Sparse step_numbers [5, 10, 15], with Frame 3 truncated (MAX_FRAME_VALUES simulation: 3 increments, only Frame 0, 1, 2 stored!)
        let hist = History {
            increments: vec![
                Increment {
                    number: 1,
                    load_factor: 0.25,
                    size: 0.25,
                    iterations: vec![],
                    converged: true,
                    cutback: None,
                    negative_pivots: 0,
                    augmentations: 0,
                },
                Increment {
                    number: 2,
                    load_factor: 0.60,
                    size: 0.35,
                    iterations: vec![],
                    converged: true,
                    cutback: None,
                    negative_pivots: 0,
                    augmentations: 0,
                },
                Increment {
                    number: 3,
                    load_factor: 1.00,
                    size: 0.40,
                    iterations: vec![],
                    converged: true,
                    cutback: None,
                    negative_pivots: 0,
                    augmentations: 0,
                },
            ],
            frames: vec![
                Frame { load_factor: 0.0, displacement: vec![[0.0; 3]; 4], reaction: vec![], stress: vec![], contacts: vec![] },
                Frame {
                    load_factor: 0.25,
                    displacement: vec![[0.0; 3]; 4],
                    reaction: vec![(0, [0.0, 0.0, -50.0e3])],
                    stress: vec![],
                    contacts: vec![],
                },
                Frame {
                    load_factor: 0.60,
                    displacement: vec![[0.0; 3]; 4],
                    reaction: vec![(0, [0.0, 0.0, -120.0e3])],
                    stress: vec![],
                    contacts: vec![],
                },
            ],
            iterations: 6,
            critical_points: vec![],
        };
        let sr = base_static_result("Pull", vec![1, 2, 3, 4], Some(hist));

        let domains = vec![
            sample_domain("cf_0001", 5, vec![1], &[(2, 2.0e3), (3, 2.0e3)], Some(&[20.0e6, 20.0e6])),
            sample_domain("cf_0001", 10, vec![1], &[(2, 14.0e3), (3, 14.0e3)], Some(&[48.0e6, 48.0e6])),
            sample_domain("cf_0001", 15, vec![1], &[(2, 45.0e3), (3, 45.0e3)], Some(&[80.0e6, 80.0e6])),
            sample_domain("cf_0001", 99, vec![1], &[(2, 1.0e3), (3, 1.0e3)], Some(&[10.0e6, 10.0e6])), // orphan unmatched domain step
        ];

        // 1. Auto KrSource uses LinearScaledStep1Ki (Step 1 K_I = 20 MPa*sqrt(m) scaled by lambda_k / 0.25)
        //    and ReactionRatio skips step 15 with ffs.reaction_truncated while warning on MPC nodes!
        let req_auto = FfsRequest {
            step: "Pull".into(),
            ring_exclusion: RingExclusion::Auto,
            active_frame: Some(2),
            fad: Some(FadConfig {
                option: FadOption::Option1,
                option2_curve: Option2Curve::ModelMaterial,
                lr_input: LrInput::ReactionRatio { bc_set: "fix".into(), component: ReactionComponent::Z, limit_load: 200.0e3 },
                toughness: ToughnessInput::JIc { value: 100.0e3, plane_strain: true },
                kr_source: KrSource::Auto,
                station_selection: StationSelection::Governing,
                sigma_ys: None,
                sigma_uts: Some(500.0e6),
                l_r_max: None,
                young: None,
                poisson: None,
            }),
        };
        let rep = evaluate_step_ffs(&model, &sr, &domains, &[5, 10, 15], true, &req_auto).unwrap();
        assert!(rep.diagnostics.iter().any(|d| d.code == "ffs.domains_truncated"));
        assert!(rep.diagnostics.iter().any(|d| d.code == "ffs.unmatched_domain_step"));
        assert!(rep.diagnostics.iter().any(|d| d.code == "ffs.reaction_mpc_warning"));
        assert!(rep.diagnostics.iter().any(|d| d.code == "ffs.reaction_truncated"));
        let fad = rep.fad.unwrap();
        assert_eq!(fad.trajectory.len(), 2); // step 5 and step 10; step 15 truncated
        assert_eq!(fad.trajectory[0].warp_step, 5);
        assert!((fad.trajectory[0].l_r - 0.25).abs() < 1e-12); // |-50k| / 200k
        assert!((fad.trajectory[1].l_r - 0.60).abs() < 1e-12); // |-120k| / 200k
        assert_eq!(fad.trajectory[1].kr_source_used, KrSource::LinearScaledStep1Ki);
        assert!((fad.trajectory[1].k_eff.unwrap() - 20.0e6 * (0.60 / 0.25)).abs() < 1e-6);

        // 2. Frame 0 gives unloaded verdict with zero spurious ffs.reaction_zero_or_missing warning.
        let mut req_f0 = req_auto.clone();
        req_f0.active_frame = Some(0);
        let rep_f0 = evaluate_step_ffs(&model, &sr, &domains, &[5, 10, 15], false, &req_f0).unwrap();
        assert!(!rep_f0.diagnostics.iter().any(|d| d.code == "ffs.reaction_zero_or_missing"));
        let v0 = rep_f0.fad.unwrap().active_verdict.unwrap();
        assert_eq!(v0.limiting_mode, LimitingMode::Unloaded);
        assert_eq!(v0.l_r, 0.0);
        assert_eq!(v0.reaction_value, Some(0.0));

        // 3. Explicit KjPlaneStrain on plastic step emits ffs.epfm_double_count.
        let mut req_kj = req_auto;
        req_kj.fad.as_mut().unwrap().kr_source = KrSource::KjPlaneStrain;
        let rep_kj = evaluate_step_ffs(&model, &sr, &domains, &[5, 10, 15], false, &req_kj).unwrap();
        assert!(rep_kj.diagnostics.iter().any(|d| d.code == "ffs.epfm_double_count"));
    }
}

//! Container-agnostic Fitness-for-Service (FFS) crack-front profile & API 579 FAD widget,
//! hosted in a `kit::Surface::Panel` strip and accompanied by a
//! 3D viewport crack-front overlay (`overlay`) with station click-to-focus.
//!
//! All fracture computations and unit conversions run through `results.ffs` and read-only
//! `fieldcraft_engine::ffs` helpers; nothing here calls `model.resolve()` or touches `fieldcraft-warp3d`.

use std::collections::BTreeMap;

use egui::{Align2, Color32, FontId, Pos2, Rect, RichText, Stroke, pos2, vec2};
use fieldcraft_engine::ffs::{
    EvaluatedStep, FadOption, FadPoint, FadSummary, FfsMaterialDefaults, FfsReport, InteractionStressState, KrSource, LimitingMode,
    ReactionComponent, SpreadStatus, StationProfile, StepStationPoint,
};
use fieldcraft_engine::results::{DomainIntegralResult, Quantity};
use fieldcraft_model::DisplaySystem;
use serde_json::{Map, Value, json};
use solidcraft_ui_kit as kit;

use crate::ModelApp;
use crate::results::{Plot, count, number, with_unit};
use crate::theme::{Tokens, label_font};

const PLOT_HEIGHT: f32 = 148.0;
const FAD_HEIGHT: f32 = 168.0;
const AXIS_LEFT: f32 = 54.0;
const AXIS_BOTTOM: f32 = 20.0;
const AXIS_TOP: f32 = 8.0;
const AXIS_RIGHT: f32 = 10.0;
const WIDE_SPLIT_MIN_WIDTH: f32 = 640.0;
const HIT_RADIUS_PX: f32 = 12.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ProfileQuantity {
    #[default]
    J,
    Ki,
    Kii,
    Kiii,
    KjPlaneStrain,
    KjPlaneStress,
    T11,
    T33,
}

impl ProfileQuantity {
    const ALL: [Self; 8] = [Self::J, Self::Ki, Self::Kii, Self::Kiii, Self::KjPlaneStrain, Self::KjPlaneStress, Self::T11, Self::T33];

    fn label(self) -> &'static str {
        match self {
            Self::J => "J-integral",
            Self::Ki => "K_I (interaction)",
            Self::Kii => "K_II (interaction)",
            Self::Kiii => "K_III (interaction)",
            Self::KjPlaneStrain => "K_J (plane strain)",
            Self::KjPlaneStress => "K_J (plane stress)",
            Self::T11 => "T_11 stress",
            Self::T33 => "T_33 stress",
        }
    }

    fn value(self, p: &StationProfile) -> Option<f64> {
        match self {
            Self::J => Some(p.j_mean),
            Self::Ki => p.k_i_mean,
            Self::Kii => p.k_ii_mean,
            Self::Kiii => p.k_iii_mean,
            Self::KjPlaneStrain => p.k_j_plane_strain,
            Self::KjPlaneStress => p.k_j_plane_stress,
            Self::T11 => p.t_11_mean,
            Self::T33 => p.t_33_mean,
        }
    }

    fn history_value(self, pt: &StepStationPoint) -> Option<f64> {
        match self {
            Self::J => Some(pt.j_mean),
            Self::Ki => pt.k_i_mean,
            Self::Kii => pt.k_ii_mean,
            Self::Kiii => pt.k_iii_mean,
            Self::KjPlaneStrain => pt.k_j_plane_strain,
            Self::KjPlaneStress => pt.k_j_plane_stress,
            Self::T11 => pt.t_11_mean,
            Self::T33 => pt.t_33_mean,
        }
    }

    fn unit(self, units: &DisplaySystem, k_unit: &str) -> String {
        match self {
            Self::J => units.symbol(Quantity::ForcePerLength).to_string(),
            Self::Ki | Self::Kii | Self::Kiii | Self::KjPlaneStrain | Self::KjPlaneStress => k_unit.to_string(),
            Self::T11 | Self::T33 => units.symbol(Quantity::Pressure).to_string(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FfsChartTab {
    #[default]
    FrontProfile,
    RingConvergence,
    LoadHistory,
}

impl FfsChartTab {
    const ALL: [Self; 3] = [Self::FrontProfile, Self::RingConvergence, Self::LoadHistory];

    fn as_str(self) -> &'static str {
        match self {
            Self::FrontProfile => "front_profile",
            Self::RingConvergence => "ring_convergence",
            Self::LoadHistory => "load_history",
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::FrontProfile => "Front Profile",
            Self::RingConvergence => "Ring Convergence",
            Self::LoadHistory => "Load History",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FrontAxis {
    #[default]
    PhysicalS,
    NormalizedSNorm,
}

impl FrontAxis {
    fn label(self) -> &'static str {
        match self {
            Self::PhysicalS => "s (arc length)",
            Self::NormalizedSNorm => "s / L (normalised)",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RingExclusionChoice {
    #[default]
    Auto,
    Count(u8),
}

impl RingExclusionChoice {
    const ALL: [Self; 6] = [Self::Auto, Self::Count(0), Self::Count(1), Self::Count(2), Self::Count(3), Self::Count(4)];

    fn as_str(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Count(0) => "0",
            Self::Count(1) => "1",
            Self::Count(2) => "2",
            Self::Count(3) => "3",
            _ => "4",
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Auto => "Auto (drop inner if ≥ 3)",
            Self::Count(0) => "0 (keep all rings)",
            Self::Count(1) => "1 inner ring",
            Self::Count(2) => "2 inner rings",
            Self::Count(3) => "3 inner rings",
            _ => "4 inner rings",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Option2SourceChoice {
    #[default]
    ModelMaterial,
    RambergOsgood,
}

impl Option2SourceChoice {
    fn as_str(self) -> &'static str {
        match self {
            Self::ModelMaterial => "model_material",
            Self::RambergOsgood => "ramberg_osgood",
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::ModelMaterial => "Model material",
            Self::RambergOsgood => "Ramberg–Osgood",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LrModeChoice {
    #[default]
    ReferenceStress,
    DirectLoadRatio,
    ReactionRatio,
}

impl LrModeChoice {
    fn as_str(self) -> &'static str {
        match self {
            Self::ReferenceStress => "reference_stress",
            Self::DirectLoadRatio => "direct_load_ratio",
            Self::ReactionRatio => "reaction_ratio",
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::ReferenceStress => "Reference stress",
            Self::DirectLoadRatio => "Direct L_r",
            Self::ReactionRatio => "BC reaction ratio",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ToughnessModeChoice {
    #[default]
    KMat,
    JIc,
}

impl ToughnessModeChoice {
    fn as_str(self) -> &'static str {
        match self {
            Self::KMat => "k_mat",
            Self::JIc => "j_ic",
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::KMat => "K_mat",
            Self::JIc => "J_Ic",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum StationModeChoice {
    #[default]
    Governing,
    SelectedStation,
}

impl StationModeChoice {
    fn label(self) -> &'static str {
        match self {
            Self::Governing => "Governing (worst station)",
            Self::SelectedStation => "Selected station",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct FfsCacheKey {
    step: String,
    revision: u64,
    results_stored: u64,
    active_frame: Option<usize>,
    units_label: String,
    params_fingerprint: String,
}

#[derive(Clone, Debug, Default)]
struct FfsTextBuffers {
    units_label: String,
    sigma_ref_full: String,
    l_r_full: String,
    limit_load: String,
    k_mat: String,
    j_ic: String,
    ramberg_alpha: String,
    ramberg_n: String,
    sigma_ys: String,
    sigma_uts: String,
    l_r_max: String,
    young: String,
    poisson: String,
}

/// Session-only state for the FFS crack-front profile & API 579 FAD panel.
/// Physical inputs are stored in SI so switching display units preserves exact values.
#[derive(Clone, Debug)]
pub struct FfsState {
    pub chart_tab: FfsChartTab,
    pub quantity: ProfileQuantity,
    pub x_axis: FrontAxis,
    pub ring_exclusion: RingExclusionChoice,
    pub show_ring_band: bool,
    pub selected_domain: Option<String>,
    pub hovered_domain: Option<String>,

    pub fad_enabled: bool,
    pub fad_option: FadOption,
    pub option2_source: Option2SourceChoice,
    pub ramberg_alpha: Option<f64>,
    pub ramberg_n: Option<f64>,
    pub lr_mode: LrModeChoice,
    pub sigma_ref_full_si: Option<f64>,
    pub l_r_full: Option<f64>,
    pub bc_set: String,
    pub reaction_component: ReactionComponent,
    pub limit_load_si: Option<f64>,
    pub toughness_mode: ToughnessModeChoice,
    pub k_mat_si: Option<f64>,
    pub j_ic_si: Option<f64>,
    pub j_ic_plane_strain: bool,
    pub kr_source: KrSource,
    pub station_mode: StationModeChoice,
    pub sigma_ys_si: Option<f64>,
    pub sigma_uts_si: Option<f64>,
    pub l_r_max: Option<f64>,
    pub young_si: Option<f64>,
    pub poisson: Option<f64>,

    buffers: FfsTextBuffers,
    cache_key: Option<FfsCacheKey>,
    pub cached_report: Option<FfsReport>,
    pub last_error: Option<String>,
}

impl Default for FfsState {
    fn default() -> Self {
        Self {
            chart_tab: FfsChartTab::default(),
            quantity: ProfileQuantity::default(),
            x_axis: FrontAxis::default(),
            ring_exclusion: RingExclusionChoice::default(),
            show_ring_band: true,
            selected_domain: None,
            hovered_domain: None,
            fad_enabled: false,
            fad_option: FadOption::Option1,
            option2_source: Option2SourceChoice::ModelMaterial,
            ramberg_alpha: Some(1.0),
            ramberg_n: Some(5.0),
            lr_mode: LrModeChoice::ReferenceStress,
            sigma_ref_full_si: None,
            l_r_full: None,
            bc_set: String::new(),
            reaction_component: ReactionComponent::Magnitude,
            limit_load_si: None,
            toughness_mode: ToughnessModeChoice::KMat,
            k_mat_si: None,
            j_ic_si: None,
            j_ic_plane_strain: true,
            kr_source: KrSource::Auto,
            station_mode: StationModeChoice::Governing,
            sigma_ys_si: None,
            sigma_uts_si: None,
            l_r_max: None,
            young_si: None,
            poisson: None,
            buffers: FfsTextBuffers::default(),
            cache_key: None,
            cached_report: None,
            last_error: None,
        }
    }
}

impl FfsState {
    /// Return the cached FFS report if it matches `step`.
    pub fn report_for(&self, step: &str) -> Option<&FfsReport> {
        self.cached_report.as_ref().filter(|r| r.step == step)
    }

    /// Invalidate the cached `results.ffs` evaluation so the next frame re-evaluates.
    pub fn invalidate(&mut self) {
        self.cache_key = None;
    }

    /// Summary JSON exposed through `ui.inspect["ffs"]`.
    pub fn inspect_json(&self, open: bool) -> Value {
        let verdict = self.cached_report.as_ref().and_then(|r| r.fad.as_ref()).and_then(|f| f.active_verdict.as_ref());
        json!({
            "open": open,
            "chart_tab": self.chart_tab.as_str(),
            "step": self.cached_report.as_ref().map(|r| r.step.as_str()),
            "selected_domain": self.selected_domain,
            "active_frame": self.cached_report.as_ref().and_then(|r| r.active_frame),
            "acceptable": verdict.map(|v| v.acceptable),
            "reserve_factor": verdict.and_then(|v| v.reserve_factor),
            "stations": self.cached_report.as_ref().map(|r| r.active_profiles.len()),
            "worst_spread_status": self.cached_report.as_ref().map(|r| r.worst_spread_status),
        })
    }

    fn sync_buffers(&mut self, units: &DisplaySystem) {
        let label = units.label();
        if self.buffers.units_label == label {
            return;
        }
        self.buffers.units_label = label;
        let fmt_opt = |v: Option<f64>| v.filter(|x| x.is_finite()).map_or_else(String::new, format_input_number);
        let fmt_qty = |q: Quantity, v: Option<f64>| fmt_opt(v.and_then(|si| units.express(q, si).ok()));
        let (k_scale, _) = k_scale_and_unit(units);
        let fmt_k = |v: Option<f64>| fmt_opt(v.map(|si| si * k_scale));

        self.buffers.sigma_ref_full = fmt_qty(Quantity::Pressure, self.sigma_ref_full_si);
        self.buffers.l_r_full = fmt_opt(self.l_r_full);
        self.buffers.limit_load = fmt_qty(Quantity::Force, self.limit_load_si);
        self.buffers.k_mat = fmt_k(self.k_mat_si);
        self.buffers.j_ic = fmt_qty(Quantity::ForcePerLength, self.j_ic_si);
        self.buffers.ramberg_alpha = fmt_opt(self.ramberg_alpha);
        self.buffers.ramberg_n = fmt_opt(self.ramberg_n);
        self.buffers.sigma_ys = fmt_qty(Quantity::Pressure, self.sigma_ys_si);
        self.buffers.sigma_uts = fmt_qty(Quantity::Pressure, self.sigma_uts_si);
        self.buffers.l_r_max = fmt_opt(self.l_r_max);
        self.buffers.young = fmt_qty(Quantity::Pressure, self.young_si);
        self.buffers.poisson = fmt_opt(self.poisson);
    }

    /// Build `results.ffs` JSON parameters in `units`. Returns `(params, fad_incomplete)`.
    pub fn build_params(&self, step: &str, active_frame: Option<usize>, units: &DisplaySystem, include_fad_if_ready: bool) -> (Value, bool) {
        let mut root = Map::new();
        root.insert("step".into(), json!(step));
        if let Some(frame) = active_frame {
            root.insert("frame".into(), json!(frame));
        }
        root.insert("exclude_inner_rings".into(), json!(self.ring_exclusion.as_str()));

        if !self.fad_enabled || !include_fad_if_ready {
            if let Some(dom) = self.selected_domain.as_deref().filter(|s| !s.is_empty()) {
                root.insert("domain_id".into(), json!(dom));
            }
            return (Value::Object(root), false);
        }

        let (k_scale, _) = k_scale_and_unit(units);
        let express_opt = |q: Quantity, si: Option<f64>| -> Option<f64> { si.and_then(|v| units.express(q, v).ok()).filter(|v| v.is_finite()) };

        let mut incomplete = false;
        let mut fad = Map::new();
        fad.insert(
            "option".into(),
            json!(match self.fad_option {
                FadOption::Option1 => "option1",
                FadOption::Option2 => "option2",
            }),
        );
        fad.insert("option2_source".into(), json!(self.option2_source.as_str()));
        if self.fad_option == FadOption::Option2 && self.option2_source == Option2SourceChoice::RambergOsgood {
            match (self.ramberg_alpha, self.ramberg_n) {
                (Some(a), Some(n)) => {
                    fad.insert("ramberg_alpha".into(), json!(a));
                    fad.insert("ramberg_n".into(), json!(n));
                }
                _ => incomplete = true,
            }
        }

        fad.insert("lr_mode".into(), json!(self.lr_mode.as_str()));
        match self.lr_mode {
            LrModeChoice::ReferenceStress => match express_opt(Quantity::Pressure, self.sigma_ref_full_si) {
                Some(v) => {
                    fad.insert("sigma_ref_full".into(), json!(v));
                }
                None => incomplete = true,
            },
            LrModeChoice::DirectLoadRatio => match self.l_r_full.filter(|x| x.is_finite()) {
                Some(v) => {
                    fad.insert("l_r_full".into(), json!(v));
                }
                None => incomplete = true,
            },
            LrModeChoice::ReactionRatio => {
                let bc = self.bc_set.trim();
                let lim = express_opt(Quantity::Force, self.limit_load_si);
                if bc.is_empty() || lim.is_none() {
                    incomplete = true;
                } else if let Some(lim) = lim {
                    fad.insert("bc_set".into(), json!(bc));
                    fad.insert("reaction_component".into(), json!(reaction_component_str(self.reaction_component)));
                    fad.insert("limit_load".into(), json!(lim));
                }
            }
        }

        fad.insert("toughness_mode".into(), json!(self.toughness_mode.as_str()));
        match self.toughness_mode {
            ToughnessModeChoice::KMat => match self.k_mat_si.map(|v| v * k_scale).filter(|v| v.is_finite()) {
                Some(v) => {
                    fad.insert("k_mat".into(), json!(v));
                }
                None => incomplete = true,
            },
            ToughnessModeChoice::JIc => match express_opt(Quantity::ForcePerLength, self.j_ic_si) {
                Some(v) => {
                    fad.insert("j_ic".into(), json!(v));
                    fad.insert("j_ic_plane_strain".into(), json!(self.j_ic_plane_strain));
                }
                None => incomplete = true,
            },
        }

        fad.insert("kr_source".into(), json!(kr_source_str(self.kr_source)));
        match self.station_mode {
            StationModeChoice::Governing => {
                fad.insert("station_mode".into(), json!("governing"));
            }
            StationModeChoice::SelectedStation => {
                if let Some(dom) = self.selected_domain.as_deref().filter(|s| !s.is_empty()) {
                    fad.insert("station_mode".into(), json!("domain_id"));
                    fad.insert("domain_id".into(), json!(dom));
                } else {
                    fad.insert("station_mode".into(), json!("governing"));
                }
            }
        }

        if let Some(v) = express_opt(Quantity::Pressure, self.sigma_ys_si) {
            fad.insert("sigma_ys".into(), json!(v));
        }
        if let Some(v) = express_opt(Quantity::Pressure, self.sigma_uts_si) {
            fad.insert("sigma_uts".into(), json!(v));
        }
        if let Some(v) = self.l_r_max.filter(|x| x.is_finite()) {
            fad.insert("l_r_max".into(), json!(v));
        }
        if let Some(v) = express_opt(Quantity::Pressure, self.young_si) {
            fad.insert("young".into(), json!(v));
        }
        if let Some(v) = self.poisson.filter(|x| x.is_finite()) {
            fad.insert("poisson".into(), json!(v));
        }

        if !incomplete {
            root.insert("fad".into(), Value::Object(fad));
        }
        let fad_handles_domain =
            !incomplete && self.station_mode == StationModeChoice::SelectedStation && self.selected_domain.as_deref().is_some_and(|s| !s.is_empty());
        if !fad_handles_domain && let Some(dom) = self.selected_domain.as_deref().filter(|s| !s.is_empty()) {
            root.insert("domain_id".into(), json!(dom));
        }
        (Value::Object(root), incomplete)
    }
}

impl ModelApp {
    /// Toggle the container-agnostic FFS assessment panel from the ribbon or results controls.
    pub(crate) fn open_ffs(&mut self) {
        if self.ui.show_ffs {
            self.ui.show_ffs = false;
            return;
        }
        if self.session.results_view.is_none() {
            if let Some(step) = self.session.unpersisted_fracture_steps().first().map(|s| (*s).to_string()) {
                let _ = self.run("results.deformed", json!({"step": step}));
            } else {
                let _ = self.run("results.deformed", json!({}));
            }
        }
        self.ui.show_ffs = true;
        self.ffs.get_or_insert_with(FfsState::default);
        self.follow_stage("results.ffs");
    }
}

fn k_scale_and_unit(units: &DisplaySystem) -> (f64, String) {
    fieldcraft_engine::units::stress_sqrt_length_scale(fieldcraft_engine::units::si(), units)
}

/// Thresholds for formatting plain decimals vs scientific notation in numeric inputs.
const INPUT_PLAIN_RANGE: std::ops::Range<f64> = 1e-3..1e6; // unit-factors: formatting range, not a unit conversion

fn format_input_number(v: f64) -> String {
    if !v.is_finite() {
        return String::new();
    }
    let abs = v.abs();
    if abs == 0.0 {
        "0".to_string()
    } else if INPUT_PLAIN_RANGE.contains(&abs) {
        let s = format!("{v:.6}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        format!("{v:.4e}")
    }
}

fn parse_input_f64(raw: &str) -> Option<f64> {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return None;
    }
    trimmed.replace(',', "").parse::<f64>().ok().filter(|x| x.is_finite())
}

fn reaction_component_str(c: ReactionComponent) -> &'static str {
    match c {
        ReactionComponent::X => "x",
        ReactionComponent::Y => "y",
        ReactionComponent::Z => "z",
        ReactionComponent::Magnitude => "magnitude",
    }
}

fn reaction_component_label(c: ReactionComponent) -> &'static str {
    match c {
        ReactionComponent::Magnitude => "Magnitude |R|",
        ReactionComponent::X => "|R_x|",
        ReactionComponent::Y => "|R_y|",
        ReactionComponent::Z => "|R_z|",
    }
}

fn kr_source_str(k: KrSource) -> &'static str {
    match k {
        KrSource::Auto => "auto",
        KrSource::ElasticKi => "elastic_ki",
        KrSource::LinearScaledStep1Ki => "linear_scaled_step1_ki",
        KrSource::KjPlaneStrain => "kj_plane_strain",
        KrSource::KjPlaneStress => "kj_plane_stress",
    }
}

fn kr_source_label(k: KrSource) -> &'static str {
    match k {
        KrSource::Auto => "Auto (elastic K_I preferred)",
        KrSource::ElasticKi => "Elastic K_I",
        KrSource::LinearScaledStep1Ki => "Scaled Step-1 K_I",
        KrSource::KjPlaneStrain => "K_J (plane strain)",
        KrSource::KjPlaneStress => "K_J (plane stress)",
    }
}

fn spread_color(status: SpreadStatus, t: &Tokens) -> Color32 {
    match status {
        SpreadStatus::Good => t.ok,
        SpreadStatus::Caution => t.warm,
        SpreadStatus::Poor => t.error,
        SpreadStatus::Unverified => t.text_dim,
    }
}

fn spread_label(status: SpreadStatus) -> &'static str {
    match status {
        SpreadStatus::Good => "Good",
        SpreadStatus::Caution => "Caution",
        SpreadStatus::Poor => "Poor",
        SpreadStatus::Unverified => "Unverified",
    }
}

fn limiting_mode_label(mode: LimitingMode) -> &'static str {
    match mode {
        LimitingMode::Unloaded => "Unloaded",
        LimitingMode::Fracture => "Fracture",
        LimitingMode::ElasticPlastic => "Elastic–plastic",
        LimitingMode::PlasticCollapse => "Plastic collapse",
    }
}

/// Draw the dockable bottom FFS panel host when `app.ui.show_ffs` is active and a plot is open.
pub fn panel(app: &mut ModelApp, ui: &mut egui::Ui) {
    if !app.ui.show_ffs {
        return;
    }
    let Some(plot) = app.plot() else { return };
    let available = app.shell.panel_space(ui.available_rect_before_wrap());
    let height = (available.height() * 0.42).clamp(160.0, 360.0).min((available.height() - 60.0).max(120.0)).min(available.height());
    if !available.is_finite() || available.width() < 1.0 || height < 60.0 {
        return;
    }
    let t = Tokens::get(app.ui.dark);
    let host = kit::Container::new(ui.ctx(), kit::Surface::Panel);
    let mut close = false;
    let draw_inner = |app: &mut ModelApp, ui: &mut egui::Ui, close: &mut bool| {
        egui::Frame::NONE.fill(t.heading_bg).inner_margin(egui::Margin::symmetric(8, 4)).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new("FITNESS-FOR-SERVICE (FFS / FAD)").font(label_font(ui.ctx(), 12.0)));
                ui.label(RichText::new(format!("Step {}", plot.view.step)).small().color(t.text_dim));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    *close = ui.small_button("×").on_hover_text("Close FFS assessment panel").clicked();
                });
            });
        });
        ui.add_space(4.0);
        egui::ScrollArea::vertical().id_salt("fc_ffs_scroll").auto_shrink([false, false]).show(ui, |ui| {
            show_contents(app, &plot, ui, &t);
        });
    };
    if host.floating() {
        let rect = kit::Container::floating_card(kit::Surface::Panel).bottom_strip_rect(available, height);
        if !rect.is_finite() || rect.width() < 1.0 || rect.height() < 60.0 {
            return;
        }
        app.place("ffs", rect);
        let frame = host.frame(ui.ctx(), t.dialog_bg, kit::frame::Side::Left);
        host.area(egui::Id::unique("fc_ffs_panel")).fixed_pos(rect.min).show(ui.ctx(), |ui| {
            let (card, _) = ui.allocate_exact_size(rect.size(), egui::Sense::hover());
            kit::Container::paint_panel_frame(ui.painter(), frame, card);
            let inner = card.shrink2(vec2(8.0, 6.0));
            if inner.is_positive() {
                let mut child = ui.new_child(egui::UiBuilder::new().id_salt("fc_ffs_inner").max_rect(inner));
                child.set_clip_rect(inner);
                draw_inner(app, &mut child, &mut close);
            }
        });
    } else {
        let panel = host.edge("fc_ffs_panel", kit::Edge::Bottom).exact_size(height).show_separator_line(false);
        host.show_edge(ui, panel, host.edge_frame(t.dialog_bg), |ui| {
            let r = ui.max_rect();
            app.place("ffs", r);
            ui.painter().line_segment([r.left_top(), r.right_top()], Stroke::new(1.0, t.border));
            egui::Frame::NONE.inner_margin(egui::Margin::symmetric(8, 6)).show(ui, |ui| {
                draw_inner(app, ui, &mut close);
            });
        });
    }
    if close {
        app.ui.show_ffs = false;
    }
}

fn stress_state_label(s: InteractionStressState) -> &'static str {
    match s {
        InteractionStressState::PlaneStrain => "Plane strain",
        InteractionStressState::PlaneStress => "Plane stress",
    }
}

const LOCAL_AXIS_LEGEND: &str = "(x1=advance, x2=normal, x3=tangent)";

/// Container-agnostic FFS crack-front and API 579 FAD contents.
/// Can be embedded in the bottom panel today or a pop-out window (`#1088`) without rewriting.
pub fn show_contents(app: &mut ModelApp, plot: &Plot, ui: &mut egui::Ui, t: &Tokens) {
    let has_fracture = app.session.fracture_results(&plot.view.step).is_some_and(|d: &[DomainIntegralResult]| !d.is_empty());
    let has_current_result = app.session.result(&plot.view.step).is_some();
    if !has_current_result || !has_fracture {
        let msg = if !has_current_result {
            format!("Fracture results for step `{}` are stale or cleared; solve or import results again.", plot.view.step)
        } else {
            format!("Step `{}` has no WARP3D crack-front fracture results. Select a fracture step or solve with crack_front.", plot.view.step)
        };
        ui.label(RichText::new(msg).small().color(t.text_dim));
        return;
    }

    let mut state = app.ffs.take().unwrap_or_default();
    state.sync_buffers(&plot.units);

    if let Some(sel) = state.selected_domain.as_deref()
        && !app.session.fracture_results(&plot.view.step).is_some_and(|ds| ds.iter().any(|d| d.domain_id == sel))
    {
        state.selected_domain = None;
    }

    let defaults = fieldcraft_engine::ffs::material_defaults(&app.session, &plot.view.step);
    let bc_sets = fieldcraft_engine::ffs::reaction_bc_sets(&app.session, &plot.view.step);
    if state.bc_set.is_empty()
        && let Some(first) = bc_sets.first()
    {
        state.bc_set = first.clone();
    }

    let active_frame = plot.frame().map(|(k, ..)| k);
    let (params, fad_incomplete) = state.build_params(&plot.view.step, active_frame, &plot.units, true);
    let desired_key = FfsCacheKey {
        step: plot.view.step.clone(),
        revision: app.session.revision(),
        results_stored: app.session.results_stored(),
        active_frame,
        units_label: plot.units.label(),
        params_fingerprint: params.to_string(),
    };

    if state.cache_key.as_ref() != Some(&desired_key) {
        // Store the attempted key immediately so invalid user inputs never loop at 60 FPS.
        state.cache_key = Some(desired_key);
        match app.run("results.ffs", params.clone()) {
            Ok(val) => match serde_json::from_value::<FfsReport>(val) {
                Ok(report) => {
                    if !report.active_profiles.is_empty()
                        && state.selected_domain.as_ref().is_none_or(|sel| !report.active_profiles.iter().any(|p| p.domain_id == *sel))
                    {
                        state.selected_domain = report.active_profiles.first().map(|p| p.domain_id.clone());
                    }
                    state.cached_report = Some(report);
                    state.last_error = None;
                }
                Err(e) => {
                    state.last_error = Some(format!("Failed to decode FFS report: {e}"));
                }
            },
            Err(err) => {
                state.last_error = Some(err);
                if state.cached_report.as_ref().is_some_and(|r| r.step != plot.view.step) {
                    state.cached_report = None;
                }
            }
        }
    }

    let mut export_params: Option<Value> = None;
    let mut clicked_station: Option<StationProfile> = None;
    let mut sync_frame: Option<usize> = None;
    let report_snapshot = state.report_for(&plot.view.step).cloned();
    let session_export_params = app.session.params_with_units("results.ffs", &params);

    ui.push_id("fc_ffs_contents", |ui| {
        let ring_bounds: Option<(usize, usize)> = report_snapshot.as_ref().and_then(|rep| {
            let mut lo = usize::MAX;
            let mut hi = 0usize;
            let mut any = false;
            for p in &rep.active_profiles {
                for r in &p.rings {
                    lo = lo.min(r.ring);
                    hi = hi.max(r.ring);
                    any = true;
                }
            }
            any.then_some((lo, hi))
        });
        let has_ring_1 = ring_bounds.is_none_or(|(lo, _)| lo == 1);

        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("Quantity").small().color(t.text_dim));
            egui::ComboBox::from_id_salt("fc_ffs_qty").selected_text(state.quantity.label()).width(142.0).show_ui(ui, |ui| {
                for q in ProfileQuantity::ALL {
                    ui.selectable_value(&mut state.quantity, q, q.label());
                }
            });

            ui.label(RichText::new("X").small().color(t.text_dim));
            egui::ComboBox::from_id_salt("fc_ffs_axis").selected_text(state.x_axis.label()).width(126.0).show_ui(ui, |ui| {
                ui.selectable_value(&mut state.x_axis, FrontAxis::PhysicalS, FrontAxis::PhysicalS.label());
                ui.selectable_value(&mut state.x_axis, FrontAxis::NormalizedSNorm, FrontAxis::NormalizedSNorm.label());
            });

            let mut exclude_ring1 = state.ring_exclusion != RingExclusionChoice::Count(0);
            if ui
                .add_enabled(has_ring_1, egui::Checkbox::new(&mut exclude_ring1, "Exclude Ring 1"))
                .on_hover_text("Exclude innermost crack-tip rosette ring distortion when 3+ rings are present")
                .on_disabled_hover_text("Ring 1 is not present in this solve (rings start past Ring 1); Exclude Ring 1 has no effect")
                .changed()
            {
                state.ring_exclusion = if exclude_ring1 { RingExclusionChoice::Auto } else { RingExclusionChoice::Count(0) };
            }
            if let Some((min_r, max_r)) = ring_bounds {
                ui.label(RichText::new(format!("Rings {min_r}–{max_r}")).small().color(t.text_dim));
            }

            ui.label(RichText::new("Rings").small().color(t.text_dim));
            egui::ComboBox::from_id_salt("fc_ffs_rings").selected_text(state.ring_exclusion.label()).width(156.0).show_ui(ui, |ui| {
                for choice in RingExclusionChoice::ALL {
                    ui.selectable_value(&mut state.ring_exclusion, choice, choice.label());
                }
            });

            if state.quantity == ProfileQuantity::J {
                ui.checkbox(&mut state.show_ring_band, "Ring band").on_hover_text("Shade J_min..J_max across kept outer rings");
            }

            ui.checkbox(&mut state.fad_enabled, "API 579 FAD").on_hover_text("Evaluate API 579-1 / ASME FFS-1 Option 1 or Option 2 FAD");

            if ui.button("Export CSV…").on_hover_text("Write crack-front profiles and FAD trajectory to RFC 4180 CSV (results.ffs)").clicked() {
                export_params = Some(session_export_params.clone());
            }
        });

        ui.add_space(4.0);

        if let Some(report) = report_snapshot.as_ref() {
            let eval_frames: Vec<&EvaluatedStep> = report.evaluated_steps.iter().filter(|e| e.frame_index.is_some()).collect();
            if report.active_profiles.is_empty() && !eval_frames.is_empty() {
                let cur_frame = report.active_frame.unwrap_or(0);
                let nearest = eval_frames.iter().copied().min_by(|a, b| {
                    let da = a.frame_index.unwrap_or(0).abs_diff(cur_frame);
                    let db = b.frame_index.unwrap_or(0).abs_diff(cur_frame);
                    da.cmp(&db).then_with(|| b.frame_index.unwrap_or(0).cmp(&a.frame_index.unwrap_or(0)))
                });
                let prev_eval = eval_frames.iter().rev().copied().find(|e| e.frame_index.is_some_and(|f| f < cur_frame));
                let next_eval = eval_frames.iter().copied().find(|e| e.frame_index.is_some_and(|f| f > cur_frame));
                egui::Frame::NONE
                    .fill(t.field)
                    .stroke(Stroke::new(1.0, t.border))
                    .corner_radius(3.0)
                    .inner_margin(egui::Margin::symmetric(8, 4))
                    .show(ui, |ui| {
                        ui.horizontal_wrapped(|ui| {
                            let frame_desc = report.active_frame.map_or_else(|| "this state".into(), |f| format!("frame {f}"));
                            ui.label(
                                RichText::new(format!("No fracture domains evaluated on {frame_desc} (λ = {}).", number(report.active_load_factor)))
                                    .small()
                                    .color(t.warm),
                            );
                            if let Some(n) = nearest
                                && let Some(fi) = n.frame_index
                                && ui.small_button(format!("Jump to Inc {} (λ = {})", n.warp_step, number(n.load_factor))).clicked()
                            {
                                sync_frame = Some(fi);
                            }
                            if let Some(p) = prev_eval
                                && nearest.is_none_or(|n| n.frame_index != p.frame_index)
                                && let Some(fi) = p.frame_index
                                && ui.small_button(format!("◀ Inc {}", p.warp_step)).clicked()
                            {
                                sync_frame = Some(fi);
                            }
                            if let Some(nx) = next_eval
                                && nearest.is_none_or(|n| n.frame_index != nx.frame_index)
                                && let Some(fi) = nx.frame_index
                                && ui.small_button(format!("Inc {} ▶", nx.warp_step)).clicked()
                            {
                                sync_frame = Some(fi);
                            }
                        });
                    });
                ui.add_space(4.0);
            }

            let wide = ui.available_width() >= WIDE_SPLIT_MIN_WIDTH;
            if wide && state.fad_enabled {
                ui.columns(2, |cols| {
                    if let Some((left, right)) = cols.split_first_mut() {
                        draw_left_chart_pane(left, &mut state, report, plot, t, &mut clicked_station, &mut sync_frame);
                        if let Some(right) = right.first_mut() {
                            draw_fad_section(
                                right,
                                &mut state,
                                report,
                                defaults.as_ref(),
                                &bc_sets,
                                &plot.units,
                                fad_incomplete,
                                t,
                                &mut clicked_station,
                                &mut sync_frame,
                            );
                        }
                    }
                });
            } else {
                draw_left_chart_pane(ui, &mut state, report, plot, t, &mut clicked_station, &mut sync_frame);
                if state.fad_enabled {
                    ui.separator();
                    draw_fad_section(
                        ui,
                        &mut state,
                        report,
                        defaults.as_ref(),
                        &bc_sets,
                        &plot.units,
                        fad_incomplete,
                        t,
                        &mut clicked_station,
                        &mut sync_frame,
                    );
                }
            }

            if !report.diagnostics.is_empty() {
                ui.add_space(4.0);
                for d in &report.diagnostics {
                    let (prefix, color) = match d.severity {
                        fieldcraft_engine::ffs::DiagnosticSeverity::Error => ("✖ ", t.error),
                        fieldcraft_engine::ffs::DiagnosticSeverity::Warning => ("⚠ ", t.warm),
                    };
                    ui.add(egui::Label::new(RichText::new(format!("{prefix}{}", d.message)).small().color(color)).wrap());
                }
            }
        }

        if let Some(err) = &state.last_error {
            ui.add_space(2.0);
            ui.add(egui::Label::new(RichText::new(err).small().color(t.error)).wrap());
        }
    });

    app.ffs = Some(state);

    if let Some(prof) = clicked_station {
        focus_station(app, plot, &prof);
    }
    if let Some(frame) = sync_frame {
        let _ = app.run("results.frame", json!({"step": plot.view.step, "frame": frame}));
    }
    if let Some(export) = export_params {
        app.export_csv_with("results.ffs", export);
    }
}

fn draw_left_chart_pane(
    ui: &mut egui::Ui,
    state: &mut FfsState,
    report: &FfsReport,
    plot: &Plot,
    t: &Tokens,
    clicked_station: &mut Option<StationProfile>,
    sync_frame: &mut Option<usize>,
) {
    let stress_state = report.active_profiles.iter().find_map(|p| p.interaction_stress_state);
    ui.horizontal_wrapped(|ui| {
        for tab in FfsChartTab::ALL {
            ui.selectable_value(&mut state.chart_tab, tab, tab.label());
        }
        if let Some(st) = stress_state {
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(RichText::new(stress_state_label(st)).small().color(t.text_dim));
            });
        }
    });
    ui.add_space(2.0);

    match state.chart_tab {
        FfsChartTab::FrontProfile => draw_profile_section(ui, state, report, plot, t, clicked_station),
        FfsChartTab::RingConvergence => draw_ring_convergence_section(ui, state, report, plot, t),
        FfsChartTab::LoadHistory => draw_load_history_section(ui, state, report, plot, t, sync_frame),
    }
}

/// Center the camera on `prof.focus_node` (strictly via `plot.node_position` so hidden/deformed
/// meshes are honored) and probe it when viewing the final solved state.
pub(crate) fn focus_station(app: &mut ModelApp, plot: &Plot, prof: &StationProfile) {
    let Some(node) = prof.focus_node else { return };
    let Some(at) = plot.node_position(node).filter(|p| p.is_finite()) else { return };
    app.cam.target = at;
    let is_final = plot.frame().is_none_or(|(k, _, n)| k + 1 >= n);
    if is_final && !plot.stale {
        let _ = app.probe_node(node);
    }
}

fn draw_profile_section(
    ui: &mut egui::Ui,
    state: &mut FfsState,
    report: &FfsReport,
    plot: &Plot,
    t: &Tokens,
    clicked_station: &mut Option<StationProfile>,
) {
    let (_, k_unit) = k_scale_and_unit(&plot.units);
    let len_unit = plot.units.symbol(Quantity::Length);
    let y_unit = state.quantity.unit(&plot.units, &k_unit);

    let (x_label, use_s_length) = match state.x_axis {
        FrontAxis::PhysicalS if report.active_profiles.iter().all(|p| p.s_length.is_some_and(f64::is_finite)) => (format!("s [{len_unit}]"), true),
        _ => ("s / L".to_string(), false),
    };

    ui.horizontal(|ui| {
        ui.label(RichText::new(format!("{} along crack front", state.quantity.label())).strong());
        ui.label(RichText::new(format!("[{y_unit}]")).small().color(t.text_dim));
        let spread_pct = if report.max_ring_spread.is_finite() { format!("{:.1}%", report.max_ring_spread * 100.0) } else { "—".into() };
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(format!("Spread: {} ({spread_pct})", spread_label(report.worst_spread_status)))
                    .small()
                    .color(spread_color(report.worst_spread_status, t)),
            );
        });
    });

    let profiles = &report.active_profiles;
    let points: Vec<(usize, f64, f64)> = profiles
        .iter()
        .enumerate()
        .filter_map(|(i, p)| {
            let x = if use_s_length { p.s_length? } else { p.s_norm };
            let y = state.quantity.value(p)?;
            (x.is_finite() && y.is_finite()).then_some((i, x, y))
        })
        .collect();

    if points.is_empty() {
        let msg = if profiles.is_empty() {
            "No fracture domains were evaluated on this frame; use the jump banner above or switch to Load History.".to_string()
        } else {
            format!("{} is unavailable for this solve (enable interaction_integral or switch quantity)", state.quantity.label())
        };
        ui.label(RichText::new(msg).small().color(t.text_dim));
        return;
    }

    let (outer, response) = ui.allocate_exact_size(vec2(ui.available_width().max(120.0), PLOT_HEIGHT + AXIS_BOTTOM), egui::Sense::click());
    let rect = Rect::from_min_max(outer.min + vec2(AXIS_LEFT, AXIS_TOP), outer.max - vec2(AXIS_RIGHT, AXIS_BOTTOM));
    let painter = ui.painter_at(outer);
    painter.rect_filled(rect, 3.0, t.field);
    painter.rect_stroke(rect, 3.0, Stroke::new(1.0, t.border), egui::StrokeKind::Inside);

    let (mut x0, mut x1) = points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &(_, x, _)| (a.min(x), b.max(x)));
    if !(x0 < x1) {
        x0 -= 0.5;
        x1 += 0.5;
    }
    let (mut y0, mut y1) = (f64::INFINITY, f64::NEG_INFINITY);
    for &(i, _, y) in &points {
        y0 = y0.min(y);
        y1 = y1.max(y);
        if state.quantity == ProfileQuantity::J
            && state.show_ring_band
            && let Some(p) = profiles.get(i)
        {
            if p.j_min.is_finite() {
                y0 = y0.min(p.j_min);
            }
            if p.j_max.is_finite() {
                y1 = y1.max(p.j_max);
            }
        }
    }
    if !(y0 < y1) {
        let pad = y0.abs().max(1.0) * 0.1;
        y0 -= pad;
        y1 += pad;
    } else {
        let pad = (y1 - y0) * 0.08;
        y0 -= pad;
        y1 += pad;
    }

    let map = |x: f64, y: f64| -> Pos2 {
        let tx = ((x - x0) / (x1 - x0)).clamp(0.0, 1.0) as f32;
        let ty = ((y - y0) / (y1 - y0)).clamp(0.0, 1.0) as f32;
        pos2(rect.left() + tx * rect.width(), rect.bottom() - ty * rect.height())
    };

    let small = FontId::proportional(10.0);
    painter.text(pos2(rect.left() - 4.0, rect.top()), Align2::RIGHT_TOP, number(y1), small.clone(), t.text_dim);
    painter.text(pos2(rect.left() - 4.0, rect.bottom()), Align2::RIGHT_BOTTOM, number(y0), small.clone(), t.text_dim);
    painter.text(pos2(rect.left(), rect.bottom() + 2.0), Align2::LEFT_TOP, number(x0), small.clone(), t.text_dim);
    painter.text(pos2(rect.right(), rect.bottom() + 2.0), Align2::RIGHT_TOP, format!("{}  {x_label}", number(x1)), small, t.text_dim);

    if y0 < 0.0 && y1 > 0.0 {
        let y_zero = map(x0, 0.0).y;
        painter.line_segment([pos2(rect.left(), y_zero), pos2(rect.right(), y_zero)], Stroke::new(1.0, t.border));
    }

    // Ring min..max shaded band for J-integral.
    if state.quantity == ProfileQuantity::J && state.show_ring_band && points.len() >= 2 {
        let mut mesh = egui::epaint::Mesh::default();
        let band_color = t.accent.gamma_multiply(0.16);
        for pair in points.windows(2) {
            let (Some(&(i0, x_a, _)), Some(&(i1, x_b, _))) = (pair.first(), pair.get(1)) else { continue };
            let (Some(p0), Some(p1)) = (profiles.get(i0), profiles.get(i1)) else { continue };
            if !(p0.j_min.is_finite() && p0.j_max.is_finite() && p1.j_min.is_finite() && p1.j_max.is_finite()) {
                continue;
            }
            let a_lo = map(x_a, p0.j_min);
            let a_hi = map(x_a, p0.j_max);
            let b_lo = map(x_b, p1.j_min);
            let b_hi = map(x_b, p1.j_max);
            let base = mesh.vertices.len() as u32;
            for pt in [a_lo, a_hi, b_hi, b_lo] {
                mesh.vertices.push(egui::epaint::Vertex { pos: pt, uv: egui::epaint::WHITE_UV, color: band_color });
            }
            mesh.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        }
        if !mesh.is_empty() {
            painter.add(egui::Shape::mesh(mesh));
        }
    }

    let screen_pts: Vec<(usize, Pos2)> = points.iter().map(|&(i, x, y)| (i, map(x, y))).collect();
    if screen_pts.len() >= 2 {
        painter.add(egui::Shape::line(screen_pts.iter().map(|(_, p)| *p).collect(), Stroke::new(1.6, t.accent)));
    }

    let pointer = response.hover_pos().filter(|p| rect.contains(*p));
    let hovered_idx = pointer.and_then(|ptr| {
        screen_pts
            .iter()
            .filter_map(|&(i, pt)| {
                let d = ptr.distance(pt);
                (d <= HIT_RADIUS_PX * 1.8).then_some((i, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    });

    if response.hovered() {
        state.hovered_domain = hovered_idx.and_then(|i| profiles.get(i)).map(|p| p.domain_id.clone());
    } else if state.hovered_domain.is_some() {
        state.hovered_domain = None;
    }

    for &(i, pt) in &screen_pts {
        let Some(p) = profiles.get(i) else { continue };
        let col = spread_color(p.spread_status, t);
        let is_selected = state.selected_domain.as_deref() == Some(p.domain_id.as_str());
        let is_hovered = hovered_idx == Some(i);
        painter.circle(pt, if is_selected { 4.5 } else { 3.5 }, col, Stroke::new(1.0, t.overlay_text));
        if is_hovered {
            painter.circle_stroke(pt, 6.5, Stroke::new(1.5, t.pick_hover));
        }
        if is_selected {
            painter.circle_stroke(pt, 7.5, Stroke::new(2.0, t.pick_selected));
        }
    }

    if let Some(i) = hovered_idx
        && let Some(p) = profiles.get(i)
        && let Some((_, pt)) = screen_pts.iter().find(|&&(idx, _)| idx == i)
    {
        painter.line_segment([pos2(pt.x, rect.top()), pos2(pt.x, rect.bottom())], Stroke::new(1.0, t.border));
        let j_unit = plot.units.symbol(Quantity::ForcePerLength);
        let s_text = p.s_length.map_or_else(|| format!("s/L {}", number(p.s_norm)), |s| with_unit(number(s), Some(len_unit)));
        let mut tip = format!(
            "{} ({s_text})\nJ = {} ({}..{})\nRings: {}/{} · Spread {:.1}% ({})",
            p.domain_id,
            with_unit(number(p.j_mean), Some(j_unit)),
            number(p.j_min),
            number(p.j_max),
            p.kept_rings,
            p.total_rings,
            p.ring_spread * 100.0,
            spread_label(p.spread_status)
        );
        if let Some(ki) = p.k_i_mean {
            tip.push_str(&format!("\nK_I = {}", with_unit(number(ki), Some(k_unit.as_str()))));
        }
        if let Some(kii) = p.k_ii_mean {
            tip.push_str(&format!("\nK_II = {}", with_unit(number(kii), Some(k_unit.as_str()))));
        }
        if let Some(kiii) = p.k_iii_mean {
            tip.push_str(&format!("\nK_III = {}", with_unit(number(kiii), Some(k_unit.as_str()))));
        }
        if let Some(kj) = p.k_j_plane_strain {
            tip.push_str(&format!("\nK_J = {}", with_unit(number(kj), Some(k_unit.as_str()))));
        }
        if let Some(t11) = p.t_11_mean {
            tip.push_str(&format!("\nT_11 = {}", with_unit(number(t11), Some(plot.units.symbol(Quantity::Pressure)))));
        }
        if let Some(t33) = p.t_33_mean {
            tip.push_str(&format!("\nT_33 = {}", with_unit(number(t33), Some(plot.units.symbol(Quantity::Pressure)))));
        }
        if let Some(st) = p.interaction_stress_state {
            tip.push_str(&format!("\nStress state: {}", stress_state_label(st)));
        }
        tip.push_str("\nClick to focus in 3D");
        response.clone().on_hover_text(tip);
    }

    if response.clicked()
        && let Some(ptr) = response.interact_pointer_pos()
    {
        let hit = screen_pts
            .iter()
            .filter_map(|&(i, pt)| {
                let d = ptr.distance(pt);
                (d <= HIT_RADIUS_PX * 1.8).then_some((i, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i);
        if let Some(i) = hit
            && let Some(prof) = profiles.get(i)
        {
            state.selected_domain = Some(prof.domain_id.clone());
            *clicked_station = Some(prof.clone());
        }
    }

    // Selected station summary line beneath the chart.
    if let Some(sel) = state.selected_domain.as_deref().and_then(|id| profiles.iter().find(|p| p.domain_id == id)).or_else(|| profiles.first()) {
        let node_txt = sel.focus_node.map_or_else(|| "—".into(), |n| format!("node {n}"));
        let j_unit = plot.units.symbol(Quantity::ForcePerLength);
        let val = state.quantity.value(sel).map_or_else(|| "—".into(), |v| with_unit(number(v), Some(y_unit.as_str())));
        ui.horizontal_wrapped(|ui| {
            ui.label(
                RichText::new(format!(
                    "Selected {}: {node_txt} · {} = {val} · J = {}",
                    sel.domain_id,
                    state.quantity.label(),
                    with_unit(number(sel.j_mean), Some(j_unit))
                ))
                .small()
                .color(t.text),
            );
            if plot.frame().is_some_and(|(k, _, n)| k + 1 < n) {
                ui.label(RichText::new("(Node probe applies to final state)").small().color(t.text_dim));
            }
            ui.label(RichText::new(LOCAL_AXIS_LEGEND).small().color(t.text_dim));
        });
    }
}

fn draw_ring_convergence_section(ui: &mut egui::Ui, state: &mut FfsState, report: &FfsReport, plot: &Plot, t: &Tokens) {
    let profiles = &report.active_profiles;
    if profiles.is_empty() {
        ui.label(
            RichText::new("No fracture domains were evaluated on this frame; use the jump banner above or switch to Load History.")
                .small()
                .color(t.text_dim),
        );
        return;
    }

    let j_unit = plot.units.symbol(Quantity::ForcePerLength);
    let p_unit = plot.units.symbol(Quantity::Pressure);
    let (_, k_unit) = k_scale_and_unit(&plot.units);

    let sel_idx = state.selected_domain.as_deref().and_then(|id| profiles.iter().position(|p| p.domain_id == id)).unwrap_or(0);
    let Some(prof) = profiles.get(sel_idx) else { return };

    ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new("Ring convergence").strong());
        ui.label(RichText::new(format!("[{j_unit}]")).small().color(t.text_dim));
        if profiles.len() > 1 {
            let mut active_id = prof.domain_id.clone();
            egui::ComboBox::from_id_salt("fc_ffs_ring_dom").selected_text(&active_id).width(104.0).show_ui(ui, |ui| {
                for p in profiles {
                    ui.selectable_value(&mut active_id, p.domain_id.clone(), &p.domain_id);
                }
            });
            if active_id != prof.domain_id {
                state.selected_domain = Some(active_id);
            }
        } else {
            ui.label(RichText::new(&prof.domain_id).small().color(t.text_dim));
        }
        let spread_pct = if prof.ring_spread.is_finite() { format!("{:.1}%", prof.ring_spread * 100.0) } else { "—".into() };
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(format!("Spread: {} ({spread_pct})", spread_label(prof.spread_status)))
                    .small()
                    .color(spread_color(prof.spread_status, t)),
            );
        });
    });

    let ring_pts: Vec<(usize, &fieldcraft_engine::results::RingIntegral, f64, bool)> =
        prof.rings.iter().enumerate().filter(|(_, r)| r.j.is_finite()).map(|(i, r)| (i, r, r.j, !prof.excluded_rings.contains(&r.ring))).collect();

    if ring_pts.is_empty() {
        ui.label(RichText::new("No per-ring domain integral values available for this station.").small().color(t.text_dim));
        return;
    }

    let (outer, response) = ui.allocate_exact_size(vec2(ui.available_width().max(120.0), PLOT_HEIGHT + AXIS_BOTTOM), egui::Sense::hover());
    let rect = Rect::from_min_max(outer.min + vec2(AXIS_LEFT, AXIS_TOP), outer.max - vec2(AXIS_RIGHT, AXIS_BOTTOM));
    let painter = ui.painter_at(outer);
    painter.rect_filled(rect, 3.0, t.field);
    painter.rect_stroke(rect, 3.0, Stroke::new(1.0, t.border), egui::StrokeKind::Inside);

    let (mut x0, mut x1) = ring_pts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &(_, r, _, _)| {
        let rx = r.ring as f64;
        (a.min(rx), b.max(rx))
    });
    if !(x0 < x1) {
        x0 -= 0.5;
        x1 += 0.5;
    } else {
        x0 -= 0.25;
        x1 += 0.25;
    }

    let (mut y0, mut y1) = ring_pts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &(_, _, y, _)| (a.min(y), b.max(y)));
    if prof.j_mean.is_finite() {
        y0 = y0.min(prof.j_mean);
        y1 = y1.max(prof.j_mean);
    }
    if !(y0 < y1) {
        let pad = y0.abs().max(1.0) * 0.1;
        y0 -= pad;
        y1 += pad;
    } else {
        let pad = (y1 - y0) * 0.1;
        y0 -= pad;
        y1 += pad;
    }

    let map = |x: f64, y: f64| -> Pos2 {
        let tx = ((x - x0) / (x1 - x0)).clamp(0.0, 1.0) as f32;
        let ty = ((y - y0) / (y1 - y0)).clamp(0.0, 1.0) as f32;
        pos2(rect.left() + tx * rect.width(), rect.bottom() - ty * rect.height())
    };

    let small = FontId::proportional(10.0);
    let min_ring = ring_pts.first().map_or(1, |(_, r, _, _)| r.ring);
    let max_ring = ring_pts.last().map_or(min_ring, |(_, r, _, _)| r.ring);
    painter.text(pos2(rect.left() - 4.0, rect.top()), Align2::RIGHT_TOP, number(y1), small.clone(), t.text_dim);
    painter.text(pos2(rect.left() - 4.0, rect.bottom()), Align2::RIGHT_BOTTOM, number(y0), small.clone(), t.text_dim);
    painter.text(pos2(rect.left(), rect.bottom() + 2.0), Align2::LEFT_TOP, format!("Ring {min_ring}"), small.clone(), t.text_dim);
    painter.text(pos2(rect.right(), rect.bottom() + 2.0), Align2::RIGHT_TOP, format!("Ring {max_ring}"), small, t.text_dim);

    if state.show_ring_band && prof.j_min.is_finite() && prof.j_max.is_finite() {
        let band_top = map(x0, prof.j_max).y;
        let band_bot = map(x0, prof.j_min).y;
        let band_rect = Rect::from_min_max(pos2(rect.left(), band_top.min(band_bot)), pos2(rect.right(), band_top.max(band_bot)));
        painter.rect_filled(band_rect, 0.0, t.accent.gamma_multiply(0.14));
    }

    if prof.j_mean.is_finite() {
        let ym = map(x0, prof.j_mean).y;
        painter.line_segment([pos2(rect.left(), ym), pos2(rect.right(), ym)], Stroke::new(1.2, t.ok));
    }

    let screen_pts: Vec<(&fieldcraft_engine::results::RingIntegral, bool, Pos2)> =
        ring_pts.iter().map(|&(_, r, j, kept)| (r, kept, map(r.ring as f64, j))).collect();
    if screen_pts.len() >= 2 {
        painter.add(egui::Shape::line(screen_pts.iter().map(|(_, _, p)| *p).collect(), Stroke::new(1.5, t.accent)));
    }

    let hovered_ring = response.hover_pos().filter(|p| rect.contains(*p)).and_then(|ptr| {
        screen_pts
            .iter()
            .filter_map(|&(r, kept, pt)| {
                let d = ptr.distance(pt);
                (d <= HIT_RADIUS_PX * 1.8).then_some((r, kept, pt, d))
            })
            .min_by(|a, b| a.3.total_cmp(&b.3))
    });

    for &(r, kept, pt) in &screen_pts {
        let is_hovered = hovered_ring.is_some_and(|(hr, _, _, _)| hr.ring == r.ring);
        if kept {
            painter.circle(pt, 4.0, t.accent, Stroke::new(1.0, t.overlay_text));
        } else {
            painter.circle(pt, 4.0, t.field, Stroke::new(1.6, t.text_dim));
        }
        if is_hovered {
            painter.circle_stroke(pt, 6.5, Stroke::new(1.5, t.pick_hover));
        }
    }

    if let Some((r, kept, pt, _)) = hovered_ring {
        painter.line_segment([pos2(pt.x, rect.top()), pos2(pt.x, rect.bottom())], Stroke::new(1.0, t.border));
        let tag = if kept { "kept" } else { "excluded" };
        let mut tip = format!("Ring {} ({tag})\nJ = {}", r.ring, with_unit(number(r.j), Some(j_unit)));
        if let Some(ki) = r.k_i {
            tip.push_str(&format!("\nK_I = {}", with_unit(number(ki), Some(k_unit.as_str()))));
        }
        if let Some(kii) = r.k_ii {
            tip.push_str(&format!("\nK_II = {}", with_unit(number(kii), Some(k_unit.as_str()))));
        }
        if let Some(kiii) = r.k_iii {
            tip.push_str(&format!("\nK_III = {}", with_unit(number(kiii), Some(k_unit.as_str()))));
        }
        if let Some(t11) = r.t_11 {
            tip.push_str(&format!("\nT_11 = {}", with_unit(number(t11), Some(p_unit))));
        }
        if let Some(t33) = r.t_33 {
            tip.push_str(&format!("\nT_33 = {}", with_unit(number(t33), Some(p_unit))));
        }
        response.on_hover_text(tip);
    }

    ui.horizontal_wrapped(|ui| {
        ui.label(
            RichText::new(format!(
                "Station {}: ● kept ({}) · ○ excluded ({}) · J_mean = {}",
                prof.domain_id,
                prof.kept_rings,
                prof.excluded_rings.len(),
                with_unit(number(prof.j_mean), Some(j_unit))
            ))
            .small()
            .color(t.text),
        );
        ui.label(RichText::new(LOCAL_AXIS_LEGEND).small().color(t.text_dim));
    });
}

fn draw_load_history_section(ui: &mut egui::Ui, state: &FfsState, report: &FfsReport, plot: &Plot, t: &Tokens, sync_frame: &mut Option<usize>) {
    let (_, k_unit) = k_scale_and_unit(&plot.units);
    let y_unit = state.quantity.unit(&plot.units, &k_unit);
    let j_unit = plot.units.symbol(Quantity::ForcePerLength);
    let p_unit = plot.units.symbol(Quantity::Pressure);
    let station_label = report.step_history_domain_id.as_deref().unwrap_or("—");

    ui.horizontal(|ui| {
        ui.label(RichText::new(format!("{} vs. load factor ({station_label})", state.quantity.label())).strong());
        ui.label(RichText::new(format!("[{y_unit}]")).small().color(t.text_dim));
    });

    let points: Vec<(&StepStationPoint, f64, f64)> = report
        .step_history
        .iter()
        .filter_map(|pt| {
            let y = state.quantity.history_value(pt)?;
            (pt.load_factor.is_finite() && y.is_finite()).then_some((pt, pt.load_factor, y))
        })
        .collect();

    if points.is_empty() {
        ui.label(
            RichText::new(format!(
                "{} load history is unavailable for this solve (switch quantity or enable interaction_integral).",
                state.quantity.label()
            ))
            .small()
            .color(t.text_dim),
        );
        return;
    }

    let (outer, response) = ui.allocate_exact_size(vec2(ui.available_width().max(120.0), PLOT_HEIGHT + AXIS_BOTTOM), egui::Sense::click());
    let rect = Rect::from_min_max(outer.min + vec2(AXIS_LEFT, AXIS_TOP), outer.max - vec2(AXIS_RIGHT, AXIS_BOTTOM));
    let painter = ui.painter_at(outer);
    painter.rect_filled(rect, 3.0, t.field);
    painter.rect_stroke(rect, 3.0, Stroke::new(1.0, t.border), egui::StrokeKind::Inside);

    let (mut x0, mut x1) = points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &(_, x, _)| (a.min(x), b.max(x)));
    if !(x0 < x1) {
        x0 -= 0.5;
        x1 += 0.5;
    } else {
        let pad = (x1 - x0) * 0.05;
        x0 -= pad;
        x1 += pad;
    }

    let (mut y0, mut y1) = points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &(_, _, y)| (a.min(y), b.max(y)));
    if !(y0 < y1) {
        let pad = y0.abs().max(1.0) * 0.1;
        y0 -= pad;
        y1 += pad;
    } else {
        let pad = (y1 - y0) * 0.08;
        y0 -= pad;
        y1 += pad;
    }

    let map = |x: f64, y: f64| -> Pos2 {
        let tx = ((x - x0) / (x1 - x0)).clamp(0.0, 1.0) as f32;
        let ty = ((y - y0) / (y1 - y0)).clamp(0.0, 1.0) as f32;
        pos2(rect.left() + tx * rect.width(), rect.bottom() - ty * rect.height())
    };

    let small = FontId::proportional(10.0);
    painter.text(pos2(rect.left() - 4.0, rect.top()), Align2::RIGHT_TOP, number(y1), small.clone(), t.text_dim);
    painter.text(pos2(rect.left() - 4.0, rect.bottom()), Align2::RIGHT_BOTTOM, number(y0), small.clone(), t.text_dim);
    painter.text(pos2(rect.left(), rect.bottom() + 2.0), Align2::LEFT_TOP, number(x0), small.clone(), t.text_dim);
    painter.text(pos2(rect.right(), rect.bottom() + 2.0), Align2::RIGHT_TOP, format!("{}  λ", number(x1)), small, t.text_dim);

    if y0 < 0.0 && y1 > 0.0 {
        let y_zero = map(x0, 0.0).y;
        painter.line_segment([pos2(rect.left(), y_zero), pos2(rect.right(), y_zero)], Stroke::new(1.0, t.border));
    }

    let screen_pts: Vec<(&StepStationPoint, Pos2)> = points.iter().map(|&(pt, x, y)| (pt, map(x, y))).collect();
    if screen_pts.len() >= 2 {
        painter.add(egui::Shape::line(screen_pts.iter().map(|(_, p)| *p).collect(), Stroke::new(1.6, t.accent)));
    }

    let is_active_pt = |pt: &StepStationPoint| -> bool {
        match report.active_frame {
            Some(f) => pt.frame_index == Some(f),
            None => pt.warp_step == report.active_warp_step,
        }
    };

    let hovered_pt = response.hover_pos().filter(|p| rect.contains(*p)).and_then(|ptr| {
        screen_pts
            .iter()
            .filter_map(|&(pt, pos)| {
                let d = ptr.distance(pos);
                (d <= HIT_RADIUS_PX * 1.8).then_some((pt, pos, d))
            })
            .min_by(|a, b| a.2.total_cmp(&b.2))
    });

    for &(pt, pos) in &screen_pts {
        let active = is_active_pt(pt);
        let is_hovered = hovered_pt.is_some_and(|(hp, _, _)| hp.warp_step == pt.warp_step);
        painter.circle(pos, if active { 5.0 } else { 3.5 }, t.accent, Stroke::new(1.0, t.overlay_text));
        if is_hovered {
            painter.circle_stroke(pos, 6.5, Stroke::new(1.5, t.pick_hover));
        }
        if active {
            painter.circle_stroke(pos, 7.0, Stroke::new(2.0, t.pick_selected));
        }
    }

    if let Some((pt, pos, _)) = hovered_pt {
        painter.line_segment([pos2(pos.x, rect.top()), pos2(pos.x, rect.bottom())], Stroke::new(1.0, t.border));
        let mut tip = format!("Inc {} (λ = {})\nJ = {}", pt.warp_step, number(pt.load_factor), with_unit(number(pt.j_mean), Some(j_unit)));
        if let Some(ki) = pt.k_i_mean {
            tip.push_str(&format!("\nK_I = {}", with_unit(number(ki), Some(k_unit.as_str()))));
        }
        if let Some(kii) = pt.k_ii_mean {
            tip.push_str(&format!("\nK_II = {}", with_unit(number(kii), Some(k_unit.as_str()))));
        }
        if let Some(kiii) = pt.k_iii_mean {
            tip.push_str(&format!("\nK_III = {}", with_unit(number(kiii), Some(k_unit.as_str()))));
        }
        if let Some(kj) = pt.k_j_plane_strain {
            tip.push_str(&format!("\nK_J (pstrn) = {}", with_unit(number(kj), Some(k_unit.as_str()))));
        }
        if let Some(t11) = pt.t_11_mean {
            tip.push_str(&format!("\nT_11 = {}", with_unit(number(t11), Some(p_unit))));
        }
        if let Some(t33) = pt.t_33_mean {
            tip.push_str(&format!("\nT_33 = {}", with_unit(number(t33), Some(p_unit))));
        }
        if report.is_nonlinear && pt.frame_index.is_some() {
            tip.push_str("\nClick to view increment");
        }
        response.clone().on_hover_text(tip);
    }

    if response.clicked()
        && let Some(ptr) = response.interact_pointer_pos()
    {
        let hit = screen_pts
            .iter()
            .filter_map(|&(pt, pos)| {
                let d = ptr.distance(pos);
                (d <= HIT_RADIUS_PX * 1.8).then_some((pt, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(pt, _)| pt);
        if let Some(pt) = hit
            && report.is_nonlinear
            && let Some(frame) = pt.frame_index
        {
            *sync_frame = Some(frame);
        }
    }

    ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new(format!("Station {station_label} · {} evaluated increment(s)", report.step_history.len())).small().color(t.text));
        ui.label(RichText::new(LOCAL_AXIS_LEGEND).small().color(t.text_dim));
    });
}

#[allow(clippy::too_many_arguments)]
fn draw_fad_section(
    ui: &mut egui::Ui,
    state: &mut FfsState,
    report: &FfsReport,
    defaults: Option<&FfsMaterialDefaults>,
    bc_sets: &[String],
    units: &DisplaySystem,
    fad_incomplete: bool,
    t: &Tokens,
    clicked_station: &mut Option<StationProfile>,
    sync_frame: &mut Option<usize>,
) {
    let (k_scale, k_unit) = k_scale_and_unit(units);
    let p_unit = units.symbol(Quantity::Pressure);
    let f_unit = units.symbol(Quantity::Force);
    let j_unit = units.symbol(Quantity::ForcePerLength);

    ui.label(RichText::new("API 579-1 Option 1 / Option 2 FAD").strong());

    egui::Grid::new("fc_ffs_fad_grid").num_columns(2).spacing(vec2(8.0, 4.0)).show(ui, |ui| {
        ui.label(RichText::new("Option").small().color(t.text_dim));
        ui.horizontal(|ui| {
            ui.selectable_value(&mut state.fad_option, FadOption::Option1, "Option 1");
            ui.selectable_value(&mut state.fad_option, FadOption::Option2, "Option 2");
        });
        ui.end_row();

        if state.fad_option == FadOption::Option2 {
            ui.label(RichText::new("Curve").small().color(t.text_dim));
            egui::ComboBox::from_id_salt("fc_ffs_opt2_src").selected_text(state.option2_source.label()).width(156.0).show_ui(ui, |ui| {
                ui.selectable_value(&mut state.option2_source, Option2SourceChoice::ModelMaterial, Option2SourceChoice::ModelMaterial.label());
                ui.selectable_value(&mut state.option2_source, Option2SourceChoice::RambergOsgood, Option2SourceChoice::RambergOsgood.label());
            });
            ui.end_row();

            if state.option2_source == Option2SourceChoice::RambergOsgood {
                ui.label(RichText::new("α / n").small().color(t.text_dim));
                ui.horizontal(|ui| {
                    plain_num_input(ui, "α", &mut state.buffers.ramberg_alpha, &mut state.ramberg_alpha, 56.0);
                    plain_num_input(ui, "n", &mut state.buffers.ramberg_n, &mut state.ramberg_n, 56.0);
                });
                ui.end_row();
            }
        }

        ui.label(RichText::new("Load ratio L_r").small().color(t.text_dim));
        egui::ComboBox::from_id_salt("fc_ffs_lr_mode").selected_text(state.lr_mode.label()).width(156.0).show_ui(ui, |ui| {
            ui.selectable_value(&mut state.lr_mode, LrModeChoice::ReferenceStress, LrModeChoice::ReferenceStress.label());
            ui.selectable_value(&mut state.lr_mode, LrModeChoice::DirectLoadRatio, LrModeChoice::DirectLoadRatio.label());
            ui.selectable_value(&mut state.lr_mode, LrModeChoice::ReactionRatio, LrModeChoice::ReactionRatio.label());
        });
        ui.end_row();

        match state.lr_mode {
            LrModeChoice::ReferenceStress => {
                ui.label(RichText::new(format!("σ_ref,full [{p_unit}]")).small().color(t.text_dim));
                qty_input(ui, Quantity::Pressure, units, "full-load σ_ref", &mut state.buffers.sigma_ref_full, &mut state.sigma_ref_full_si, 120.0);
                ui.end_row();
            }
            LrModeChoice::DirectLoadRatio => {
                ui.label(RichText::new("L_r,full").small().color(t.text_dim));
                plain_num_input(ui, "full-load L_r", &mut state.buffers.l_r_full, &mut state.l_r_full, 120.0);
                ui.end_row();
            }
            LrModeChoice::ReactionRatio => {
                ui.label(RichText::new("BC set / comp").small().color(t.text_dim));
                ui.horizontal(|ui| {
                    let sel = if state.bc_set.is_empty() { "Choose…" } else { state.bc_set.as_str() };
                    egui::ComboBox::from_id_salt("fc_ffs_bc_set").selected_text(sel).width(100.0).show_ui(ui, |ui| {
                        for name in bc_sets {
                            ui.selectable_value(&mut state.bc_set, name.clone(), name.as_str());
                        }
                    });
                    egui::ComboBox::from_id_salt("fc_ffs_r_comp")
                        .selected_text(reaction_component_label(state.reaction_component))
                        .width(96.0)
                        .show_ui(ui, |ui| {
                            for comp in [ReactionComponent::Magnitude, ReactionComponent::X, ReactionComponent::Y, ReactionComponent::Z] {
                                ui.selectable_value(&mut state.reaction_component, comp, reaction_component_label(comp));
                            }
                        });
                });
                ui.end_row();

                ui.label(RichText::new(format!("Limit load R_L [{f_unit}]")).small().color(t.text_dim));
                qty_input(ui, Quantity::Force, units, "limit load", &mut state.buffers.limit_load, &mut state.limit_load_si, 120.0);
                ui.end_row();
            }
        }

        ui.label(RichText::new("Toughness").small().color(t.text_dim));
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("fc_ffs_tough_mode").selected_text(state.toughness_mode.label()).width(72.0).show_ui(ui, |ui| {
                ui.selectable_value(&mut state.toughness_mode, ToughnessModeChoice::KMat, ToughnessModeChoice::KMat.label());
                ui.selectable_value(&mut state.toughness_mode, ToughnessModeChoice::JIc, ToughnessModeChoice::JIc.label());
            });
            match state.toughness_mode {
                ToughnessModeChoice::KMat => {
                    k_input(ui, k_scale, &k_unit, &mut state.buffers.k_mat, &mut state.k_mat_si, 100.0);
                }
                ToughnessModeChoice::JIc => {
                    qty_input(ui, Quantity::ForcePerLength, units, j_unit, &mut state.buffers.j_ic, &mut state.j_ic_si, 90.0);
                    ui.checkbox(&mut state.j_ic_plane_strain, "Plane strain");
                }
            }
        });
        ui.end_row();
    });

    egui::CollapsingHeader::new(RichText::new("Advanced FAD options").small().color(t.text_dim)).id_salt("fc_ffs_adv_fad").default_open(false).show(
        ui,
        |ui| {
            egui::Grid::new("fc_ffs_adv_grid").num_columns(2).spacing(vec2(8.0, 4.0)).show(ui, |ui| {
                ui.label(RichText::new("K_r source").small().color(t.text_dim));
                egui::ComboBox::from_id_salt("fc_ffs_kr_src").selected_text(kr_source_label(state.kr_source)).width(168.0).show_ui(ui, |ui| {
                    for ks in [KrSource::Auto, KrSource::ElasticKi, KrSource::LinearScaledStep1Ki, KrSource::KjPlaneStrain, KrSource::KjPlaneStress] {
                        ui.selectable_value(&mut state.kr_source, ks, kr_source_label(ks));
                    }
                });
                ui.end_row();

                ui.label(RichText::new("Station mode").small().color(t.text_dim));
                egui::ComboBox::from_id_salt("fc_ffs_st_mode").selected_text(state.station_mode.label()).width(168.0).show_ui(ui, |ui| {
                    ui.selectable_value(&mut state.station_mode, StationModeChoice::Governing, StationModeChoice::Governing.label());
                    ui.selectable_value(&mut state.station_mode, StationModeChoice::SelectedStation, StationModeChoice::SelectedStation.label());
                });
                ui.end_row();

                let ys_hint = defaults
                    .and_then(|d| d.sigma_ys)
                    .and_then(|v| units.express(Quantity::Pressure, v).ok())
                    .map_or_else(|| "auto".to_string(), number);
                ui.label(RichText::new(format!("σ_ys [{p_unit}]")).small().color(t.text_dim));
                qty_input(ui, Quantity::Pressure, units, &ys_hint, &mut state.buffers.sigma_ys, &mut state.sigma_ys_si, 110.0);
                ui.end_row();

                ui.label(RichText::new(format!("σ_uts [{p_unit}]")).small().color(t.text_dim));
                qty_input(ui, Quantity::Pressure, units, "optional", &mut state.buffers.sigma_uts, &mut state.sigma_uts_si, 110.0);
                ui.end_row();

                ui.label(RichText::new("L_r,max").small().color(t.text_dim));
                plain_num_input(ui, "auto", &mut state.buffers.l_r_max, &mut state.l_r_max, 110.0);
                ui.end_row();

                let e_hint =
                    defaults.and_then(|d| d.young).and_then(|v| units.express(Quantity::Pressure, v).ok()).map_or_else(|| "auto".to_string(), number);
                ui.label(RichText::new(format!("E [{p_unit}]")).small().color(t.text_dim));
                qty_input(ui, Quantity::Pressure, units, &e_hint, &mut state.buffers.young, &mut state.young_si, 110.0);
                ui.end_row();

                let nu_hint = defaults.and_then(|d| d.poisson).map_or_else(|| "auto".to_string(), number);
                ui.label(RichText::new("ν").small().color(t.text_dim));
                plain_num_input(ui, &nu_hint, &mut state.buffers.poisson, &mut state.poisson, 110.0);
                ui.end_row();
            });
        },
    );

    if fad_incomplete {
        ui.label(RichText::new("Enter load ratio and toughness to evaluate API 579 FAD").small().color(t.text_dim));
        return;
    }

    let Some(fad) = report.fad.as_ref() else { return };
    draw_fad_chart(ui, state, report, fad, &k_unit, t, clicked_station, sync_frame);
}

#[allow(clippy::too_many_arguments)]
fn draw_fad_chart(
    ui: &mut egui::Ui,
    state: &mut FfsState,
    report: &FfsReport,
    fad: &FadSummary,
    k_unit: &str,
    t: &Tokens,
    clicked_station: &mut Option<StationProfile>,
    sync_frame: &mut Option<usize>,
) {
    let max_lr_pts = fad.trajectory.iter().chain(&fad.active_stations).map(|p| p.l_r).filter(|x| x.is_finite()).fold(fad.effective_lr_max, f64::max);
    let max_kr_pts = fad.trajectory.iter().chain(&fad.active_stations).map(|p| p.k_r).filter(|x| x.is_finite()).fold(1.0_f64, f64::max);

    let x_max = (max_lr_pts * 1.1).max(1.2);
    let y_max = (max_kr_pts * 1.1).max(1.15);

    let (outer, response) = ui.allocate_exact_size(vec2(ui.available_width().max(140.0), FAD_HEIGHT + AXIS_BOTTOM), egui::Sense::click());
    let rect = Rect::from_min_max(outer.min + vec2(AXIS_LEFT, AXIS_TOP), outer.max - vec2(AXIS_RIGHT, AXIS_BOTTOM));
    let painter = ui.painter_at(outer);
    painter.rect_filled(rect, 3.0, t.field);
    painter.rect_stroke(rect, 3.0, Stroke::new(1.0, t.border), egui::StrokeKind::Inside);

    let map = |lr: f64, kr: f64| -> Pos2 {
        let tx = (lr / x_max).clamp(0.0, 1.0) as f32;
        let ty = (kr / y_max).clamp(0.0, 1.0) as f32;
        pos2(rect.left() + tx * rect.width(), rect.bottom() - ty * rect.height())
    };

    let small = FontId::proportional(10.0);
    painter.text(pos2(rect.left() - 4.0, rect.top()), Align2::RIGHT_TOP, number(y_max), small.clone(), t.text_dim);
    painter.text(pos2(rect.left() - 4.0, map(0.0, 1.0).y), Align2::RIGHT_CENTER, "1.0", small.clone(), t.text_dim);
    painter.text(pos2(rect.left() - 4.0, rect.bottom()), Align2::RIGHT_BOTTOM, "0", small.clone(), t.text_dim);
    painter.text(pos2(rect.left(), rect.bottom() + 2.0), Align2::LEFT_TOP, "0", small.clone(), t.text_dim);
    painter.text(pos2(rect.right(), rect.bottom() + 2.0), Align2::RIGHT_TOP, format!("{}  L_r", number(x_max)), small, t.text_dim);

    // Safe area shading via triangle-strip Mesh (accurate for non-convex Option 1/2 curves).
    if fad.envelope.len() >= 2 {
        let mut mesh = egui::epaint::Mesh::default();
        let safe_fill = t.ok.gamma_multiply(0.12);
        for pair in fad.envelope.windows(2) {
            let (Some(&[x0, y0]), Some(&[x1, y1])) = (pair.first(), pair.get(1)) else { continue };
            if !(x0.is_finite() && y0.is_finite() && x1.is_finite() && y1.is_finite()) {
                continue;
            }
            let b0 = map(x0, 0.0);
            let t0 = map(x0, y0);
            let t1 = map(x1, y1);
            let b1 = map(x1, 0.0);
            let base = mesh.vertices.len() as u32;
            for pt in [b0, t0, t1, b1] {
                mesh.vertices.push(egui::epaint::Vertex { pos: pt, uv: egui::epaint::WHITE_UV, color: safe_fill });
            }
            mesh.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        }
        if !mesh.is_empty() {
            painter.add(egui::Shape::mesh(mesh));
        }
        let env_pts: Vec<Pos2> = fad.envelope.iter().filter(|[x, y]| x.is_finite() && y.is_finite()).map(|&[x, y]| map(x, y)).collect();
        if env_pts.len() >= 2 {
            painter.add(egui::Shape::line(env_pts, Stroke::new(2.0, t.ok)));
        }
    }

    // L_r,max,eff vertical cutoff line.
    let lr_cut_x = map(fad.effective_lr_max, 0.0).x;
    painter.line_segment([pos2(lr_cut_x, rect.top()), pos2(lr_cut_x, rect.bottom())], Stroke::new(1.0, t.warm));

    // Radial assessment ray for the active verdict.
    if let Some(verdict) = &fad.active_verdict
        && let Some([ix, iy]) = verdict.intersection
    {
        painter.line_segment([map(0.0, 0.0), map(ix, iy)], Stroke::new(1.0, t.text_dim));
        painter.circle_filled(map(ix, iy), 2.5, t.text_dim);
    }

    // Trajectory polyline + increment dots.
    let traj_pts: Vec<(&FadPoint, Pos2)> =
        fad.trajectory.iter().filter(|pt| pt.l_r.is_finite() && pt.k_r.is_finite()).map(|pt| (pt, map(pt.l_r, pt.k_r))).collect();
    if traj_pts.len() >= 2 {
        painter.add(egui::Shape::line(traj_pts.iter().map(|(_, p)| *p).collect(), Stroke::new(1.5, t.accent)));
    }

    let is_active_pt = |pt: &FadPoint| -> bool {
        match report.active_frame {
            Some(f) => pt.frame_index == Some(f),
            None => pt.warp_step == report.active_warp_step,
        }
    };

    for &(pt, pos) in &traj_pts {
        let active = is_active_pt(pt);
        let fill = if pt.acceptable { t.accent } else { t.error };
        painter.circle(pos, if active { 5.0 } else { 3.5 }, fill, Stroke::new(1.0, t.overlay_text));
        if active {
            painter.circle_stroke(pos, 7.0, Stroke::new(2.0, t.pick_selected));
        }
    }

    // Active-increment station cloud.
    let station_pts: Vec<(&FadPoint, Pos2)> =
        fad.active_stations.iter().filter(|pt| pt.l_r.is_finite() && pt.k_r.is_finite()).map(|pt| (pt, map(pt.l_r, pt.k_r))).collect();
    for &(pt, pos) in &station_pts {
        let fill = if pt.acceptable { t.ok } else { t.error };
        let selected = state.selected_domain.as_deref() == Some(pt.domain_id.as_str());
        painter.circle(pos, if selected { 4.2 } else { 3.0 }, fill, Stroke::new(1.0, t.overlay_text));
        if selected {
            painter.circle_stroke(pos, 6.2, Stroke::new(1.8, t.pick_selected));
        }
    }

    // Hover tooltip on trajectory or active-station dots.
    if let Some(ptr) = response.hover_pos().filter(|p| rect.contains(*p)) {
        let best_traj = traj_pts
            .iter()
            .filter_map(|&(pt, pos)| {
                let d = ptr.distance(pos);
                (d <= HIT_RADIUS_PX).then_some((pt, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1));
        let best_st = station_pts
            .iter()
            .filter_map(|&(pt, pos)| {
                let d = ptr.distance(pos);
                (d <= HIT_RADIUS_PX).then_some((pt, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1));
        let nearest = match (best_traj, best_st) {
            (Some((a, da)), Some((b, db))) => Some(if da <= db { a } else { b }),
            (Some((a, _)), None) => Some(a),
            (None, Some((b, _))) => Some(b),
            (None, None) => None,
        };
        if let Some(pt) = nearest {
            let rf = pt.reserve_factor.map_or_else(|| "—".to_string(), number);
            let margin = pt.safety_margin.map_or_else(|| "—".to_string(), number);
            let tip = format!(
                "Inc {} (λ = {}) · {}\nL_r = {} · K_r = {} (allow {})\nRF = {rf} · Margin = {margin} · {}",
                pt.increment,
                number(pt.load_factor),
                pt.domain_id,
                number(pt.l_r),
                number(pt.k_r),
                number(pt.k_r_allow),
                limiting_mode_label(pt.limiting_mode)
            );
            response.clone().on_hover_text(tip);
        }
    }

    // Click: sync nonlinear increment if clicking a trajectory dot with frame_index, else select station.
    if response.clicked()
        && let Some(ptr) = response.interact_pointer_pos()
    {
        let hit_traj = traj_pts
            .iter()
            .filter_map(|&(pt, pos)| {
                let d = ptr.distance(pos);
                (d <= HIT_RADIUS_PX).then_some((pt, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(pt, _)| pt);
        let hit_st = station_pts
            .iter()
            .filter_map(|&(pt, pos)| {
                let d = ptr.distance(pos);
                (d <= HIT_RADIUS_PX).then_some((pt, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(pt, _)| pt);

        if let Some(pt) = hit_traj
            && let Some(frame) = pt.frame_index
            && hit_st.is_none_or(|st| ptr.distance(map(pt.l_r, pt.k_r)) <= ptr.distance(map(st.l_r, st.k_r)))
        {
            *sync_frame = Some(frame);
        } else if let Some(st) = hit_st {
            state.selected_domain = Some(st.domain_id.clone());
            if let Some(prof) = report.active_profiles.iter().find(|p| p.domain_id == st.domain_id) {
                *clicked_station = Some(prof.clone());
            }
        }
    }

    // Verdict readout strip.
    if let Some(v) = &fad.active_verdict {
        let (badge, color) = if v.acceptable { ("✔ ACCEPTABLE", t.ok) } else { ("⚠ UNACCEPTABLE", t.error) };
        let rf = v.reserve_factor.map_or_else(|| "—".to_string(), number);
        let margin = v.safety_margin.map_or_else(|| "—".to_string(), number);
        let k_eff = v.k_eff.map_or_else(|| "—".to_string(), |k| with_unit(number(k), Some(k_unit)));
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new(badge).strong().color(color));
            ui.label(
                RichText::new(format!(
                    "RF = {rf} (margin {margin}) · L_r = {} · K_r = {} / {} · K_eff = {k_eff} · {}",
                    number(v.l_r),
                    number(v.k_r),
                    number(v.k_r_allow),
                    limiting_mode_label(v.limiting_mode)
                ))
                .small()
                .color(t.text),
            );
        });
        if v.epfm_k_j_ratio.is_some() || v.epfm_j_ratio.is_some() {
            let kj_r = v.epfm_k_j_ratio.map_or_else(|| "—".to_string(), number);
            let j_r = v.epfm_j_ratio.map_or_else(|| "—".to_string(), number);
            ui.label(RichText::new(format!("Separate EPFM ratios: K_J / K_mat = {kj_r} · J / J_mat = {j_r}")).small().color(t.text_dim));
        }
    }
}

fn plain_num_input(ui: &mut egui::Ui, hint: &str, buf: &mut String, out: &mut Option<f64>, width: f32) {
    let resp = ui.add(egui::TextEdit::singleline(buf).hint_text(hint).desired_width(width));
    if resp.changed() {
        *out = parse_input_f64(buf);
    }
}

fn qty_input(ui: &mut egui::Ui, q: Quantity, units: &DisplaySystem, hint: &str, buf: &mut String, out_si: &mut Option<f64>, width: f32) {
    let resp = ui.add(egui::TextEdit::singleline(buf).hint_text(hint).desired_width(width));
    if resp.changed() {
        *out_si = parse_input_f64(buf).and_then(|v| units.to_si(q, v).ok()).filter(|v| v.is_finite());
    }
}

fn k_input(ui: &mut egui::Ui, k_scale: f64, k_unit: &str, buf: &mut String, out_si: &mut Option<f64>, width: f32) {
    let resp = ui.add(egui::TextEdit::singleline(buf).hint_text(k_unit).desired_width(width));
    if resp.changed() {
        *out_si = parse_input_f64(buf).filter(|_| k_scale.is_finite() && k_scale > 0.0).map(|v| v / k_scale);
    }
}

/// 3D viewport crack-front overlay drawn strictly through `plot.node_position(node)` so
/// deformed frames and `view.hide` hidden bodies are always respected.
pub fn overlay(app: &mut ModelApp, painter: &egui::Painter, rect: Rect, t: &Tokens) {
    if !app.ui.show_ffs {
        return;
    }
    let Some(plot) = app.plot() else { return };
    let Some(state) = app.ffs.as_ref() else { return };
    let Some(report) = state.report_for(&plot.view.step) else { return };

    let mut by_step: BTreeMap<usize, Vec<&StationProfile>> = BTreeMap::new();
    for p in &report.active_profiles {
        by_step.entry(p.warp_step).or_default().push(p);
    }

    for (_, stations) in by_step {
        let mut prev: Option<Pos2> = None;
        for prof in stations {
            let screen =
                prof.focus_node.and_then(|n| plot.node_position(n)).and_then(|w| crate::viewport::to_screen(app, w)).filter(|p| rect.contains(*p));
            let Some(at) = screen else {
                // Break polyline across hidden or offscreen nodes so no line bridges gaps.
                prev = None;
                continue;
            };
            if let Some(before) = prev {
                painter.line_segment([before, at], Stroke::new(2.0, t.accent));
            }
            prev = Some(at);

            let col = spread_color(prof.spread_status, t);
            let is_selected = state.selected_domain.as_deref() == Some(prof.domain_id.as_str());
            let is_hovered = state.hovered_domain.as_deref() == Some(prof.domain_id.as_str());
            painter.circle(at, 3.5, col, Stroke::new(1.0, t.overlay_text));
            if is_hovered {
                painter.circle_stroke(at, 5.5, Stroke::new(1.5, t.pick_hover));
            }
            if is_selected {
                painter.circle_stroke(at, 6.5, Stroke::new(2.0, t.pick_selected));
                let galley = painter.layout_no_wrap(prof.domain_id.clone(), FontId::proportional(11.0), t.overlay_text);
                let label_rect = Rect::from_min_size(at + vec2(10.0, -galley.size().y * 0.5 - 3.0), galley.size() + vec2(10.0, 6.0));
                painter.rect_filled(label_rect, 3.0, t.overlay);
                painter.galley(label_rect.left_top() + vec2(5.0, 3.0), galley, t.overlay_text);
            }
        }
    }
}

/// Format active station count for summary strips.
#[allow(dead_code)]
fn _station_count(n: usize) -> String {
    count(n as u64)
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicUsize, Ordering};

    use fieldcraft_engine::Session;
    use fieldcraft_model::units::Preset;
    use serde_json::json;

    use super::*;

    fn scratch_dir(tag: &str) -> std::path::PathBuf {
        static SEQ: AtomicUsize = AtomicUsize::new(0);
        let n = SEQ.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("fieldcraft-ui-ffs-{tag}-{}-{n}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn collect_texts(shapes: &[egui::epaint::ClippedShape], out: &mut Vec<String>) {
        fn walk(s: &egui::Shape, out: &mut Vec<String>) {
            match s {
                egui::Shape::Text(t) => out.push(t.galley.text().to_string()),
                egui::Shape::Vec(v) => {
                    for child in v {
                        walk(child, out);
                    }
                }
                _ => {}
            }
        }
        for cs in shapes {
            walk(&cs.shape, out);
        }
    }

    fn collect_circles(shapes: &[egui::epaint::ClippedShape], out: &mut Vec<(Pos2, f32, Color32)>) {
        fn walk(s: &egui::Shape, out: &mut Vec<(Pos2, f32, Color32)>) {
            match s {
                egui::Shape::Circle(c) => out.push((c.center, c.radius, c.fill)),
                egui::Shape::Vec(v) => {
                    for child in v {
                        walk(child, out);
                    }
                }
                _ => {}
            }
        }
        for cs in shapes {
            walk(&cs.shape, out);
        }
    }

    fn find_button_center(shapes: &[egui::epaint::ClippedShape], label: &str) -> Option<Pos2> {
        fn walk(s: &egui::Shape, label: &str) -> Option<Pos2> {
            match s {
                egui::Shape::Text(t) if t.galley.text() == label => Some(t.pos + t.galley.size() * 0.5),
                egui::Shape::Vec(v) => v.iter().find_map(|c| walk(c, label)),
                _ => None,
            }
        }
        shapes.iter().find_map(|cs| walk(&cs.shape, label))
    }

    fn build_fracture_app() -> (ModelApp, u32, u32) {
        use fieldcraft_geometry::sample::{Plate, plate_design};
        let dir = scratch_dir("session");
        let plate_path = dir.join("plate.solidcraft");
        std::fs::write(&plate_path, plate_design(&Plate::default()).unwrap()).unwrap();
        let mut s = Session::default();
        s.execute("geometry.open", &json!({"path": plate_path.to_str().unwrap()})).unwrap();
        s.run_script(&json!([
            {"command": "model.units", "params": {"system": "mm-t-s-N-MPa"}},
            {"command": "mesh.block", "params": {"body": "Body1", "kind": "hex8", "corners": [[0, 0, 0], [20, 0, 0], [20, 10, 0], [0, 10, 0], [0, 0, 10], [20, 0, 10], [20, 10, 10], [0, 10, 10]], "divisions": [2, 1, 1]}}
        ]))
        .unwrap();

        let mut clamp = Vec::new();
        let mut tip = Vec::new();
        let mut front = Vec::new();
        for (&label, n) in s.model.nodes() {
            if n.xyz[0].abs() < 1e-6 {
                clamp.push(label);
            } else if (n.xyz[0] - 0.02).abs() < 1e-6 {
                tip.push(label);
            } else if (n.xyz[0] - 0.01).abs() < 1e-6 && n.xyz[2].abs() < 1e-6 {
                front.push((n.xyz[1], label));
            }
        }
        front.sort_by(|a, b| a.0.total_cmp(&b.0));
        let n_a = front[0].1;
        let n_b = front[1].1;

        s.run_script(&json!([
            {"command": "model.set.create", "params": {"name": "all", "elements": [1, 2]}},
            {"command": "model.set.create", "params": {"name": "fix", "nodes": clamp}},
            {"command": "model.set.create", "params": {"name": "pull_face", "nodes": tip}},
            {"command": "model.set.create", "params": {"name": "front", "nodes": [n_a, n_b]}},
            {"command": "model.material.create", "params": {"name": "Steel", "elastic": {"type": "isotropic", "young": 210000, "poisson": 0.3}}},
            {"command": "model.material.plasticity", "params": {"name": "Steel", "plastic": {"type": "bilinear", "yield_stress": 400.0, "hardening_modulus": 2000.0, "hardening": "isotropic"}}},
            {"command": "model.section.create", "params": {"name": "solid", "set": "all", "material": "Steel", "kind": {"type": "solid"}}},
            {"command": "model.boundary_condition.create", "params": {"name": "clamp", "type": "fixed", "target": {"set": "fix"}}},
            {"command": "model.load.create", "params": {"name": "pull", "type": "force", "target": {"set": "pull_face"}, "vector": [20000, 0, 0]}},
            {"command": "model.step.create", "params": {"name": "Pull", "procedure": {"type": "nonlinear_static", "nonlinear_geometry": false, "max_increments": 2}, "boundary_conditions": ["clamp"], "loads": ["pull"]}},
            {"command": "model.step.create", "params": {"name": "Other", "procedure": {"type": "static"}, "boundary_conditions": ["clamp"], "loads": ["pull"]}}
        ]))
        .unwrap();

        let n_count = s.model.nodes().len();
        let e_count = s.model.elements().len();
        std::fs::write(dir.join("model.inp"), "! fieldcraft-units: mm-t-s-N-MPa\n").unwrap();
        let row26 = vec!["0.0"; 26].join(" ");
        for (tag, u_tip, rx) in [("0000001", 0.2, -2500.0), ("0000002", 0.8, -5000.0)] {
            let wnd = (1..=n_count)
                .map(|lbl| if tip.contains(&(lbl as u32)) { format!("{u_tip} 0.0 0.0") } else { "0.0 0.0 0.0".into() })
                .collect::<Vec<_>>()
                .join("\n");
            let wnr = (1..=n_count)
                .map(|lbl| if clamp.contains(&(lbl as u32)) { format!("{rx} 0.0 0.0") } else { "0.0 0.0 0.0".into() })
                .collect::<Vec<_>>()
                .join("\n");
            std::fs::write(dir.join(format!("wnd{tag}_text")), format!("#\n{wnd}\n")).unwrap();
            std::fs::write(dir.join(format!("wnr{tag}_text")), format!("#\n{wnr}\n")).unwrap();
            std::fs::write(dir.join(format!("wns{tag}_text")), format!("#\n{}", vec![row26.as_str(); n_count].join("\n"))).unwrap();
            std::fs::write(dir.join(format!("wes{tag}_text")), format!("#\n{}", vec![row26.as_str(); e_count].join("\n"))).unwrap();
        }

        let domain_block = |id: &str, step: usize, node: u32, j_base: f64, k_base: f64| -> String {
            let (j1, j2, j3) = (j_base * 0.85, j_base, j_base * 1.02);
            let (k1, k2, k3) = (k_base * 0.85, k_base, k_base * 1.02);
            format!(
                "\
 loading: pull  step: {step}
     Domain definition:  {id}

        Number of front nodes:    1 Nodes on front:
              {node}
        Direction cosines:   0.000000  1.000000  0.000000
        Tangent vector direction cosines:   0.000000  0.000000  1.000000
        Front order:  1     Verify front: T Domain type:  4
     ** end of domain definition **

                         domain-integral post-processing
                         -------------------------------
 step:     {step}
 domain id: {id}
 area under q along front:        2.500000E+01

                                             J-integral components
                                          ===========================
 domain        dm1          dm2          dm3          dm4          dm5          dm6          dm7          dm8       total J   killed ele
     1     {j1:.4E}   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {j1:.4E}   (  0)
     2     {j2:.4E}   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {j2:.4E}   (  0)
     3     {j3:.4E}   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {j3:.4E}   (  0)

    average:  {j2:.4E}   minimum:  {j1:.4E}   maximum: {j3:.4E}     ({id})

               Static I-values
                          -- KI = 1, KII = 0, KIII = 0, plane strain --
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I     KI (pstrn) killed ele
     1     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {k1:.4E}   (  0)
     2     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {k2:.4E}   (  0)
     3     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {k3:.4E}   (  0)
    average:  {k2:.4E}   minimum:  {k1:.4E}   maximum: {k3:.4E}     ({id})
"
            )
        };
        let stdout = [
            domain_block("cf_0001", 1, n_a, 5.0, 250.0),
            domain_block("cf_0002", 1, n_b, 8.0, 320.0),
            domain_block("cf_0001", 2, n_a, 15.0, 450.0),
            domain_block("cf_0002", 2, n_b, 24.0, 580.0),
        ]
        .join("\n");
        std::fs::write(dir.join("warp3d.out"), stdout).unwrap();

        s.execute("solve.static", &json!({"step": "Other"})).unwrap();
        s.execute("results.read", &json!({"format": "warp3d", "step": "Pull", "path": dir.to_string_lossy()})).unwrap();
        let _ = std::fs::remove_dir_all(&dir);

        let mut app = ModelApp::new(s);
        app.run("results.deformed", json!({"step": "Pull", "show": true, "scale": 10.0})).unwrap();
        (app, n_a, n_b)
    }

    fn run_app_ui(app: &mut ModelApp, ctx: &egui::Context, w: f32, h: f32, time: f64, events: Vec<egui::Event>) -> egui::FullOutput {
        let mut out = ctx.run_ui(
            egui::RawInput { time: Some(time), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(w, h))), events, ..Default::default() },
            |ui| {
                app.logic(ui.ctx());
                app.ui(ui);
            },
        );
        out.textures_delta.clear();
        out
    }

    #[test]
    fn responsive_layout_and_si_preservation_across_all_unit_presets() {
        let (mut app, _, _) = build_fracture_app();
        app.open_ffs();
        {
            let ffs = app.ffs.as_mut().unwrap();
            ffs.fad_enabled = true;
            ffs.fad_option = FadOption::Option2;
            ffs.sigma_ref_full_si = Some(220.0e6);
            ffs.k_mat_si = Some(80.0e6);
        }

        let ctx = egui::Context::default();
        for (i, (w, h)) in [(1280.0, 800.0), (640.0, 480.0)].into_iter().enumerate() {
            let _warm = run_app_ui(&mut app, &ctx, w, h, (i * 2 + 1) as f64, Vec::new());
            let out = run_app_ui(&mut app, &ctx, w, h, (i * 2 + 2) as f64, Vec::new());
            let vp = app.viewport.rect.expect("CentralPanel viewport rect");
            assert!(vp.height() > 50.0, "CentralPanel must not be starved at {w}x{h}: {vp:?}");
            assert!(app.layout.contains_key("ffs"), "fc_ffs_panel recorded in app.layout");
            let mut texts = Vec::new();
            collect_texts(&out.shapes, &mut texts);
            assert!(texts.iter().any(|t| t.contains("FITNESS-FOR-SERVICE")), "ffs={:?} {texts:?}", app.layout.get("ffs"));
            assert!(texts.iter().any(|t| t.contains("J-integral along crack front")), "{texts:?}");
            if w >= 1280.0 {
                assert!(texts.iter().any(|t| t.contains("API 579-1 Option 1 / Option 2 FAD")), "{texts:?}");
            }
            assert!(
                app.ffs.as_ref().and_then(|f| f.report_for("Pull")).and_then(|r| r.fad.as_ref()).and_then(|f| f.active_verdict.as_ref()).is_some(),
                "FAD verdict evaluated cleanly at {w}x{h}"
            );
        }

        // Switching across every Preset::ALL preserves SI inputs and updates display units cleanly.
        for (i, preset) in Preset::ALL.into_iter().enumerate() {
            app.run("model.units", json!({"system": preset.label()})).unwrap();
            let plot = app.plot().unwrap();
            let mut out = ctx.run_ui(
                egui::RawInput {
                    time: Some((10 + i) as f64),
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(1000.0, 600.0))),
                    ..Default::default()
                },
                |ui| show_contents(&mut app, &plot, ui, &Tokens::dark()),
            );
            out.textures_delta.clear();
            let mut texts = Vec::new();
            collect_texts(&out.shapes, &mut texts);
            assert!(texts.iter().any(|t| t.contains("ACCEPTABLE") || t.contains("UNACCEPTABLE")), "{texts:?}");
            let ffs = app.ffs.as_ref().unwrap();
            assert!((ffs.sigma_ref_full_si.unwrap() - 220.0e6).abs() < 1e-6);
            assert!((ffs.k_mat_si.unwrap() - 80.0e6).abs() < 1e-6);
            let report = ffs.report_for("Pull").unwrap();
            assert!(report.fad.as_ref().and_then(|f| f.active_verdict.as_ref()).is_some());
            assert!(ffs.last_error.is_none());
        }
    }

    #[test]
    fn station_click_to_focus_probing_and_view_hide_overlay() {
        let (mut app, n_a, n_b) = build_fracture_app();
        app.open_ffs();
        let ctx = egui::Context::default();
        let t = Tokens::dark();

        // Render show_contents once to populate cached_report and screen dots for cf_0001 / cf_0002.
        let plot = app.plot().unwrap();
        let mut out = ctx.run_ui(
            egui::RawInput { time: Some(1.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(800.0, 320.0))), ..Default::default() },
            |ui| show_contents(&mut app, &plot, ui, &t),
        );
        out.textures_delta.clear();

        let mut circles = Vec::new();
        collect_circles(&out.shapes, &mut circles);
        // Two station dots drawn on the crack-front profile plot (rightmost = cf_0002 at node n_b).
        let mut station_dots: Vec<Pos2> =
            circles.iter().filter(|(_, r, _)| (*r - 3.5).abs() < 0.2 || (*r - 4.5).abs() < 0.2).map(|(c, _, _)| *c).collect();
        station_dots.sort_by(|a, b| a.x.total_cmp(&b.x));
        let second_dot = *station_dots.last().expect("second station dot");

        let click = vec![
            egui::Event::PointerMoved(second_dot),
            egui::Event::PointerButton { pos: second_dot, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
            egui::Event::PointerButton { pos: second_dot, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() },
        ];
        let mut click_out = ctx.run_ui(
            egui::RawInput {
                time: Some(2.0),
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(800.0, 320.0))),
                events: click,
                ..Default::default()
            },
            |ui| show_contents(&mut app, &plot, ui, &t),
        );
        click_out.textures_delta.clear();

        assert_eq!(app.ffs.as_ref().and_then(|f| f.selected_domain.as_deref()), Some("cf_0002"));
        let expected_b = plot.node_position(n_b).unwrap();
        assert!(app.cam.target.dist(expected_b) < 1e-12, "camera focused on cf_0002 node {n_b}");
        assert_eq!(app.probe.as_ref().map(|p| p.node), Some(n_b), "final state probes focus_node");

        // Intermediate increment: click moves camera to deformed position WITHOUT probing final state.
        app.run("results.frame", json!({"step": "Pull", "frame": 1})).unwrap();
        app.probe = None;
        let plot_mid = app.plot().unwrap();
        let mut mid_out = ctx.run_ui(
            egui::RawInput { time: Some(3.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(800.0, 320.0))), ..Default::default() },
            |ui| show_contents(&mut app, &plot_mid, ui, &t),
        );
        mid_out.textures_delta.clear();
        let mut mid_circles = Vec::new();
        collect_circles(&mid_out.shapes, &mut mid_circles);
        let mut mid_dots: Vec<Pos2> =
            mid_circles.iter().filter(|(_, r, _)| (*r - 3.5).abs() < 0.2 || (*r - 4.5).abs() < 0.2).map(|(c, _, _)| *c).collect();
        mid_dots.sort_by(|a, b| a.x.total_cmp(&b.x));
        let first_dot = *mid_dots.first().expect("first station dot");
        let mid_click = vec![
            egui::Event::PointerMoved(first_dot),
            egui::Event::PointerButton { pos: first_dot, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
            egui::Event::PointerButton { pos: first_dot, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() },
        ];
        let mut mid_click_out = ctx.run_ui(
            egui::RawInput {
                time: Some(4.0),
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(800.0, 320.0))),
                events: mid_click,
                ..Default::default()
            },
            |ui| show_contents(&mut app, &plot_mid, ui, &t),
        );
        mid_click_out.textures_delta.clear();
        assert_eq!(app.ffs.as_ref().and_then(|f| f.selected_domain.as_deref()), Some("cf_0001"));
        assert!(app.cam.target.dist(plot_mid.node_position(n_a).unwrap()) < 1e-12);
        assert!(app.probe.is_none(), "intermediate increment must not probe final-state nodal values");

        // 3D overlay draws stations when visible, and skips hidden body completely on view.hide.
        app.viewport.rect = Some(Rect::from_min_size(Pos2::ZERO, vec2(800.0, 600.0)));
        app.fit_view();
        let mut vis_out = ctx.run_ui(
            egui::RawInput { time: Some(5.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(800.0, 600.0))), ..Default::default() },
            |ui| {
                overlay(
                    &mut app,
                    &ui.painter_at(Rect::from_min_size(Pos2::ZERO, vec2(800.0, 600.0))),
                    Rect::from_min_size(Pos2::ZERO, vec2(800.0, 600.0)),
                    &t,
                )
            },
        );
        vis_out.textures_delta.clear();
        let mut vis_circles = Vec::new();
        collect_circles(&vis_out.shapes, &mut vis_circles);
        assert!(vis_circles.len() >= 2, "overlay draws crack-front dots when body is visible");

        app.run("view.hide", json!({"items": [{"kind": "body", "name": "Body1"}]})).unwrap();
        let hidden_plot = app.plot().unwrap();
        assert!(hidden_plot.node_position(n_a).is_none());
        let saved_target = app.cam.target;
        let prof_a = app.ffs.as_ref().unwrap().report_for("Pull").unwrap().active_profiles[0].clone();
        focus_station(&mut app, &hidden_plot, &prof_a);
        assert_eq!(app.cam.target, saved_target, "click-to-focus must skip hidden body instead of falling back to model.node");

        let mut hid_out = ctx.run_ui(
            egui::RawInput { time: Some(6.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(800.0, 600.0))), ..Default::default() },
            |ui| {
                overlay(
                    &mut app,
                    &ui.painter_at(Rect::from_min_size(Pos2::ZERO, vec2(800.0, 600.0))),
                    Rect::from_min_size(Pos2::ZERO, vec2(800.0, 600.0)),
                    &t,
                )
            },
        );
        hid_out.textures_delta.clear();
        let mut hid_circles = Vec::new();
        collect_circles(&hid_out.shapes, &mut hid_circles);
        assert!(hid_circles.is_empty(), "overlay must emit zero circles for hidden body");
    }

    #[test]
    fn bidirectional_fad_increment_sync_shared_spread_and_file_flows() {
        let (mut app, _, _) = build_fracture_app();
        let ctx = egui::Context::default();
        let t = Tokens::dark();

        // Results > Fracture Integrals uses Ring-1-excluded Auto spread (2.0% Good) and FFS Assessment… toggles panel.
        let plot = app.plot().unwrap();
        let mut rp_out = ctx.run_ui(
            egui::RawInput { time: Some(1.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(320.0, 800.0))), ..Default::default() },
            |ui| crate::results_panel::controls(&mut app, &plot, ui, &t),
        );
        rp_out.textures_delta.clear();
        let mut rp_texts = Vec::new();
        collect_texts(&rp_out.shapes, &mut rp_texts);
        assert!(rp_texts.iter().any(|s| s.contains("2.0% (Good)")), "expected Ring-1-excluded spread in {rp_texts:?}");
        let ffs_btn = find_button_center(&rp_out.shapes, "FFS Assessment…").expect("FFS Assessment… button");
        let mut rp_click = ctx.run_ui(
            egui::RawInput {
                time: Some(2.0),
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(320.0, 800.0))),
                events: vec![
                    egui::Event::PointerMoved(ffs_btn),
                    egui::Event::PointerButton { pos: ffs_btn, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
                    egui::Event::PointerButton { pos: ffs_btn, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() },
                ],
                ..Default::default()
            },
            |ui| crate::results_panel::controls(&mut app, &plot, ui, &t),
        );
        rp_click.textures_delta.clear();
        assert!(app.ui.show_ffs);

        // Enable FAD and render wide view so both plots appear.
        {
            let ffs = app.ffs.as_mut().unwrap();
            ffs.fad_enabled = true;
            ffs.fad_option = FadOption::Option2;
            ffs.sigma_ref_full_si = Some(220.0e6);
            ffs.k_mat_si = Some(80.0e6);
        }
        let mut out1 = ctx.run_ui(
            egui::RawInput { time: Some(3.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(1000.0, 400.0))), ..Default::default() },
            |ui| show_contents(&mut app, &plot, ui, &t),
        );
        out1.textures_delta.clear();
        let mut circles = Vec::new();
        collect_circles(&out1.shapes, &mut circles);
        // Trajectory dots have radius 3.5 (non-active) and color t.accent (if acceptable) or t.error.
        // Increment 1 is non-active when viewing final frame (frame 2), so find the trajectory dot with radius ~3.5 and fill t.accent / t.error inside the right half.
        let inc1_dot = circles
            .iter()
            .filter(|(c, r, f)| c.x > 520.0 && (*r - 3.5).abs() < 0.2 && (*f == t.accent || *f == t.error))
            .map(|(c, _, _)| *c)
            .min_by(|a, b| a.x.total_cmp(&b.x))
            .expect("increment 1 trajectory dot");
        let mut out2 = ctx.run_ui(
            egui::RawInput {
                time: Some(4.0),
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(1000.0, 400.0))),
                events: vec![
                    egui::Event::PointerMoved(inc1_dot),
                    egui::Event::PointerButton { pos: inc1_dot, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
                    egui::Event::PointerButton { pos: inc1_dot, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() },
                ],
                ..Default::default()
            },
            |ui| show_contents(&mut app, &plot, ui, &t),
        );
        out2.textures_delta.clear();
        assert_eq!(app.plot().unwrap().frame().map(|(k, ..)| k), Some(1), "clicking FAD inc-1 dot syncs results.frame");

        // Ribbon file_action("results.ffs") toggles show_ffs cleanly.
        assert!(app.file_action("results.ffs", &json!({}), false));
        assert!(!app.ui.show_ffs);
        assert!(app.file_action("results.ffs", &json!({}), false));
        assert!(app.ui.show_ffs);

        // export_csv_with("results.ffs") without save picker opens results.ffs form; with save picker writes CSV.
        app.export_csv_with("results.ffs", json!({"step": "Pull"}));
        assert_eq!(app.dialog.as_ref().map(|d| d.command), Some("results.ffs"));
        app.close_dialog();

        let dir = scratch_dir("export");
        let out_csv = dir.join("pull-ffs.csv");
        let out_str = out_csv.to_string_lossy().into_owned();
        app.services.pick_save = Some(Box::new(move |_, _| Some(out_str.clone())));
        app.export_csv_with("results.ffs", json!({"step": "Pull"}));
        assert!(!app.poll_picker());
        let csv_content = std::fs::read_to_string(&out_csv).unwrap();
        assert!(csv_content.contains("record_type,step,increment"), "{csv_content}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn control_channel_step_switching_lifecycle_and_one_line_replies() {
        let (mut app, _, _) = build_fracture_app();
        let ctx = egui::Context::default();
        let call = |app: &mut ModelApp, method: &str, params: Value| -> Value {
            let (tx, _rx) = std::sync::mpsc::channel();
            match crate::control::handle(app, &ctx, &crate::control::ControlRequest::new(method, params, tx)) {
                crate::control::Outcome::Done(v) => v,
                _ => json!({"ok": false}),
            }
        };

        // ui.set {show_ffs: true} and ui.inspect report show_ffs, layout.ffs, and ffs summary.
        assert_eq!(call(&mut app, "ui.set", json!({"show_ffs": true}))["ok"], true);
        assert!(app.ui.show_ffs);
        app.ffs.get_or_insert_with(Default::default).sigma_ref_full_si = Some(200.0e6);
        app.ffs.as_mut().unwrap().k_mat_si = Some(90.0e6);
        app.ffs.as_mut().unwrap().fad_enabled = true;

        run_app_ui(&mut app, &ctx, 1280.0, 800.0, 1.0, Vec::new());
        let inspected = call(&mut app, "ui.inspect", json!({}));
        assert_eq!(inspected["result"]["ui"]["show_ffs"], true);
        assert!(inspected["result"]["layout"]["ffs"].is_array());
        assert_eq!(inspected["result"]["ffs"]["open"], true);
        assert_eq!(inspected["result"]["ffs"]["step"], "Pull");
        assert!(inspected["result"]["ffs"]["acceptable"].is_boolean());
        assert!(inspected["result"]["ffs"]["reserve_factor"].is_number());

        // results.ffs never pops open app.reply when plot().is_some(), and replies read in <=200 chars on 1 line.
        assert!(app.reply.is_none());
        let ffs_val = app.run("results.ffs", json!({"step": "Pull"})).unwrap();
        assert!(app.reply.is_none(), "results.ffs must not pop open right-hand reply card when plot is active");
        let status = app.status.as_ref().unwrap().text.clone();
        assert!(!status.contains('\n') && status.chars().count() <= 200 && !status.contains("{\""));

        let s_line = crate::reply::summary_in("results.ffs", &ffs_val, Some(app.session.units()));
        assert!(!s_line.contains('\n') && s_line.chars().count() <= 200);
        let d_line = crate::reply::done("results.ffs", "Fitness-for-Service (FAD)", &json!({}), &ffs_val);
        assert!(!d_line.contains('\n') && d_line.chars().count() <= 200);

        // Switching to a non-fracture step preserves FfsState inputs while showing the empty state.
        app.run("results.deformed", json!({"step": "Other"})).unwrap();
        let out_other = run_app_ui(&mut app, &ctx, 1280.0, 800.0, 2.0, Vec::new());
        let mut texts = Vec::new();
        collect_texts(&out_other.shapes, &mut texts);
        assert!(texts.iter().any(|s| s.contains("has no WARP3D crack-front fracture results")), "{texts:?}");
        assert_eq!(app.ffs.as_ref().and_then(|f| f.k_mat_si), Some(90.0e6));

        // results.clear resets ffs and show_ffs.
        app.run("results.clear", json!({})).unwrap();
        assert!(!app.ui.show_ffs && app.ffs.is_none());

        // file.new resets ffs and show_ffs via frame_new_document.
        app.ui.show_ffs = true;
        app.ffs = Some(FfsState::default());
        app.run("file.new", json!({})).unwrap();
        app.model_changed();
        assert!(!app.ui.show_ffs && app.ffs.is_none());
    }

    fn build_sparse_mixed_mode_fracture_app(ring_start: usize) -> (ModelApp, u32, u32) {
        use fieldcraft_geometry::sample::{Plate, plate_design};
        let dir = scratch_dir("sparse-mixed");
        let plate_path = dir.join("plate.solidcraft");
        std::fs::write(&plate_path, plate_design(&Plate::default()).unwrap()).unwrap();
        let mut s = Session::default();
        s.execute("geometry.open", &json!({"path": plate_path.to_str().unwrap()})).unwrap();
        s.run_script(&json!([
            {"command": "model.units", "params": {"system": "mm-t-s-N-MPa"}},
            {"command": "mesh.block", "params": {"body": "Body1", "kind": "hex8", "corners": [[0, 0, 0], [20, 0, 0], [20, 10, 0], [0, 10, 0], [0, 0, 10], [20, 0, 10], [20, 10, 10], [0, 10, 10]], "divisions": [2, 1, 1]}}
        ]))
        .unwrap();

        let mut clamp = Vec::new();
        let mut tip = Vec::new();
        let mut front = Vec::new();
        for (&label, n) in s.model.nodes() {
            if n.xyz[0].abs() < 1e-6 {
                clamp.push(label);
            } else if (n.xyz[0] - 0.02).abs() < 1e-6 {
                tip.push(label);
            } else if (n.xyz[0] - 0.01).abs() < 1e-6 && n.xyz[2].abs() < 1e-6 {
                front.push((n.xyz[1], label));
            }
        }
        front.sort_by(|a, b| a.0.total_cmp(&b.0));
        let n_a = front[0].1;
        let n_b = front[1].1;

        s.run_script(&json!([
            {"command": "model.set.create", "params": {"name": "all", "elements": [1, 2]}},
            {"command": "model.set.create", "params": {"name": "fix", "nodes": clamp}},
            {"command": "model.set.create", "params": {"name": "pull_face", "nodes": tip}},
            {"command": "model.set.create", "params": {"name": "front", "nodes": [n_a, n_b]}},
            {"command": "model.material.create", "params": {"name": "Steel", "elastic": {"type": "isotropic", "young": 210000, "poisson": 0.3}}},
            {"command": "model.section.create", "params": {"name": "solid", "set": "all", "material": "Steel", "kind": {"type": "solid"}}},
            {"command": "model.boundary_condition.create", "params": {"name": "clamp", "type": "fixed", "target": {"set": "fix"}}},
            {"command": "model.load.create", "params": {"name": "pull", "type": "force", "target": {"set": "pull_face"}, "vector": [20000, 0, 0]}},
            {"command": "model.step.create", "params": {"name": "Pull", "procedure": {"type": "nonlinear_static", "nonlinear_geometry": false, "max_increments": 3}, "boundary_conditions": ["clamp"], "loads": ["pull"]}}
        ]))
        .unwrap();

        let n_count = s.model.nodes().len();
        let e_count = s.model.elements().len();
        std::fs::write(dir.join("model.inp"), "! fieldcraft-units: mm-t-s-N-MPa\n").unwrap();
        let row26 = vec!["0.0"; 26].join(" ");
        for (tag, u_tip, rx) in [("0000001", 0.2, -2000.0), ("0000002", 0.5, -3500.0), ("0000003", 0.8, -5000.0)] {
            let wnd = (1..=n_count)
                .map(|lbl| if tip.contains(&(lbl as u32)) { format!("{u_tip} 0.0 0.0") } else { "0.0 0.0 0.0".into() })
                .collect::<Vec<_>>()
                .join("\n");
            let wnr = (1..=n_count)
                .map(|lbl| if clamp.contains(&(lbl as u32)) { format!("{rx} 0.0 0.0") } else { "0.0 0.0 0.0".into() })
                .collect::<Vec<_>>()
                .join("\n");
            std::fs::write(dir.join(format!("wnd{tag}_text")), format!("#\n{wnd}\n")).unwrap();
            std::fs::write(dir.join(format!("wnr{tag}_text")), format!("#\n{wnr}\n")).unwrap();
            std::fs::write(dir.join(format!("wns{tag}_text")), format!("#\n{}", vec![row26.as_str(); n_count].join("\n"))).unwrap();
            std::fs::write(dir.join(format!("wes{tag}_text")), format!("#\n{}", vec![row26.as_str(); e_count].join("\n"))).unwrap();
        }

        let r0 = ring_start;
        let r1 = ring_start + 1;
        let r2 = ring_start + 2;
        let r3 = ring_start + 3;
        let domain_block = |id: &str, step: usize, node: u32, j_base: f64, k_base: f64| -> String {
            let (j0, j1, j2, j3) = (j_base * 0.80, j_base, j_base * 1.01, j_base * 1.02);
            let (k0, k1, k2, k3) = (k_base * 0.80, k_base, k_base * 1.01, k_base * 1.02);
            format!(
                "\
 loading: pull  step: {step}
     Domain definition:  {id}

        Number of front nodes:    1 Nodes on front:
              {node}
        Direction cosines:   0.000000  1.000000  0.000000
        Tangent vector direction cosines:   0.000000  0.000000  1.000000
        Front order:  1     Verify front: T Domain type:  4
     ** end of domain definition **

                         domain-integral post-processing
                         -------------------------------
 step:     {step}
 domain id: {id}
 area under q along front:        2.500000E+01

                                             J-integral components
                                          ===========================
 domain        dm1          dm2          dm3          dm4          dm5          dm6          dm7          dm8       total J   killed ele
    {r0}     {j0:.4E}   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {j0:.4E}   (  0)
    {r1}     {j1:.4E}   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {j1:.4E}   (  0)
    {r2}     {j2:.4E}   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {j2:.4E}   (  0)
    {r3}     {j3:.4E}   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {j3:.4E}   (  0)

    average:  {j1:.4E}   minimum:  {j0:.4E}   maximum: {j3:.4E}     ({id})

               Static I-values
                          -- KI = 1, KII = 0, KIII = 0, plane strain --
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I     KI (pstrn) killed ele
    {r0}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {k0:.4E}   (  0)
    {r1}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {k1:.4E}   (  0)
    {r2}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {k2:.4E}   (  0)
    {r3}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {k3:.4E}   (  0)
    average:  {k1:.4E}   minimum:  {k0:.4E}   maximum: {k3:.4E}     ({id})
  {k1:.4E}  {k0:.4E}  {k3:.4E}

                          -- KI = 0, KII = 1, KIII = 0, plane strain --
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I     KII (pstrn) killed ele
    {r0}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {:.4E}   (  0)
    {r1}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {:.4E}   (  0)
    {r2}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {:.4E}   (  0)
    {r3}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {:.4E}   (  0)
    average:  {:.4E}   minimum:  {:.4E}   maximum: {:.4E}     ({id})
  {:.4E}  {:.4E}  {:.4E}

                          -- KI = 0, KII = 0, KIII = 1 --
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I     KIII        killed ele
    {r0}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {:.4E}   (  0)
    {r1}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {:.4E}   (  0)
    {r2}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {:.4E}   (  0)
    {r3}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   {:.4E}   (  0)
    average:  {:.4E}   minimum:  {:.4E}   maximum: {:.4E}     ({id})
  {:.4E}  {:.4E}  {:.4E}

                          --T-stress (\"plane strain\" auxiliary fields)--
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I     T11 (pstrn)      T33     killed ele
    {r0}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00  -1.2000E+01  -3.5000E+00   (  0)
    {r1}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00  -1.4000E+01  -4.0000E+00   (  0)
    {r2}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00  -1.4200E+01  -4.1000E+00   (  0)
    {r3}     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00  -1.4400E+01  -4.2000E+00   (  0)
    average:     minimum:     maximum:     ({id})
 -1.4000E+01 -1.2000E+01 -1.4400E+01
 -4.0000E+00 -3.5000E+00 -4.2000E+00
",
                k0 * 0.2,
                k1 * 0.2,
                k2 * 0.2,
                k3 * 0.2,
                k1 * 0.2,
                k0 * 0.2,
                k3 * 0.2,
                k1 * 0.2,
                k0 * 0.2,
                k3 * 0.2,
                k0 * 0.1,
                k1 * 0.1,
                k2 * 0.1,
                k3 * 0.1,
                k1 * 0.1,
                k0 * 0.1,
                k3 * 0.1,
                k1 * 0.1,
                k0 * 0.1,
                k3 * 0.1,
            )
        };
        // Step 1 and Step 3 evaluated; Step 2 intentionally omitted (sparse increment schedule).
        let stdout = [
            domain_block("cf_0001", 1, n_a, 5.0, 250.0),
            domain_block("cf_0002", 1, n_b, 8.0, 320.0),
            domain_block("cf_0001", 3, n_a, 15.0, 450.0),
            domain_block("cf_0002", 3, n_b, 24.0, 580.0),
        ]
        .join("\n");
        std::fs::write(dir.join("warp3d.out"), stdout).unwrap();
        s.execute("results.read", &json!({"format": "warp3d", "step": "Pull", "path": dir.to_string_lossy()})).unwrap();
        let _ = std::fs::remove_dir_all(&dir);

        let mut app = ModelApp::new(s);
        app.run("results.deformed", json!({"step": "Pull", "show": true, "scale": 10.0})).unwrap();
        (app, n_a, n_b)
    }

    #[test]
    fn mixed_mode_tabs_ring_bounds_jump_banner_and_build_params() {
        let (mut app, _, _) = build_sparse_mixed_mode_fracture_app(1);
        app.open_ffs();
        let ctx = egui::Context::default();
        let t = Tokens::dark();

        // 1. Mixed-mode ProfileQuantity (K_II, K_III, T_33), Plane strain badge, and local-axis legend render cleanly.
        for q in [ProfileQuantity::Kii, ProfileQuantity::Kiii, ProfileQuantity::T33] {
            app.ffs.as_mut().unwrap().quantity = q;
            let plot = app.plot().unwrap();
            let mut out = ctx.run_ui(
                egui::RawInput { time: Some(1.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(900.0, 420.0))), ..Default::default() },
                |ui| show_contents(&mut app, &plot, ui, &t),
            );
            out.textures_delta.clear();
            let mut texts = Vec::new();
            collect_texts(&out.shapes, &mut texts);
            assert!(texts.iter().any(|s| s.contains(q.label())), "missing {} in {texts:?}", q.label());
            assert!(texts.iter().any(|s| s == "Plane strain"), "missing Plane strain badge in {texts:?}");
            assert!(texts.iter().any(|s| s.contains(LOCAL_AXIS_LEGEND)), "missing local axis legend in {texts:?}");
            assert!(texts.iter().any(|s| s == "Rings 1–4"), "missing Rings 1–4 label in {texts:?}");
        }

        // 2. Ring Convergence tab renders kept & excluded rings and inspect_json reports chart_tab.
        app.ffs.as_mut().unwrap().chart_tab = FfsChartTab::RingConvergence;
        let plot = app.plot().unwrap();
        let mut ring_out = ctx.run_ui(
            egui::RawInput { time: Some(2.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(900.0, 420.0))), ..Default::default() },
            |ui| show_contents(&mut app, &plot, ui, &t),
        );
        ring_out.textures_delta.clear();
        let mut ring_texts = Vec::new();
        collect_texts(&ring_out.shapes, &mut ring_texts);
        assert!(ring_texts.iter().any(|s| s.contains("Ring convergence")), "{ring_texts:?}");
        assert!(ring_texts.iter().any(|s| s.contains("● kept (3)") && s.contains("○ excluded (1)")), "{ring_texts:?}");
        assert_eq!(app.ffs.as_ref().unwrap().inspect_json(true)["chart_tab"], "ring_convergence");

        // 3. Load History tab renders across evaluated increments and clicking an earlier point syncs results.frame.
        {
            let ffs = app.ffs.as_mut().unwrap();
            ffs.chart_tab = FfsChartTab::LoadHistory;
            ffs.quantity = ProfileQuantity::Kii;
            ffs.selected_domain = Some("cf_0002".into());
        }
        let mut hist_out = ctx.run_ui(
            egui::RawInput { time: Some(3.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(900.0, 420.0))), ..Default::default() },
            |ui| show_contents(&mut app, &plot, ui, &t),
        );
        hist_out.textures_delta.clear();
        let mut hist_texts = Vec::new();
        collect_texts(&hist_out.shapes, &mut hist_texts);
        assert!(hist_texts.iter().any(|s| s.contains("2 evaluated increment(s)")), "{hist_texts:?}");
        let mut hist_circles = Vec::new();
        collect_circles(&hist_out.shapes, &mut hist_circles);
        // Step 1 point has radius 3.5 (inactive while frame 3 is active); leftmost is Step 1.
        let step1_dot = hist_circles
            .iter()
            .filter(|(_, r, _)| (*r - 3.5).abs() < 0.2)
            .map(|(c, _, _)| *c)
            .min_by(|a, b| a.x.total_cmp(&b.x))
            .expect("step 1 load history dot");
        let mut hist_click_out = ctx.run_ui(
            egui::RawInput {
                time: Some(4.0),
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(900.0, 420.0))),
                events: vec![
                    egui::Event::PointerMoved(step1_dot),
                    egui::Event::PointerButton { pos: step1_dot, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
                    egui::Event::PointerButton {
                        pos: step1_dot,
                        button: egui::PointerButton::Primary,
                        pressed: false,
                        modifiers: Default::default(),
                    },
                ],
                ..Default::default()
            },
            |ui| show_contents(&mut app, &plot, ui, &t),
        );
        hist_click_out.textures_delta.clear();
        assert_eq!(app.plot().unwrap().frame().map(|(k, ..)| k), Some(1), "clicking Load History step 1 syncs results.frame");

        // 4. Scrubbing to unevaluated frame 2 preserves selected_domain, renders jump banner, and jumping returns to an evaluated frame.
        app.run("results.frame", json!({"step": "Pull", "frame": 2})).unwrap();
        let plot_f2 = app.plot().unwrap();
        let mut empty_out = ctx.run_ui(
            egui::RawInput { time: Some(5.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(900.0, 420.0))), ..Default::default() },
            |ui| show_contents(&mut app, &plot_f2, ui, &t),
        );
        empty_out.textures_delta.clear();
        assert_eq!(app.ffs.as_ref().and_then(|f| f.selected_domain.as_deref()), Some("cf_0002"), "unevaluated frame preserves selected_domain");
        let mut empty_texts = Vec::new();
        collect_texts(&empty_out.shapes, &mut empty_texts);
        let jump_label = empty_texts.iter().find(|s| s.starts_with("Jump to Inc ")).cloned().expect("jump banner button");
        let jump_pos = find_button_center(&empty_out.shapes, &jump_label).expect("jump button center");
        let mut jump_out = ctx.run_ui(
            egui::RawInput {
                time: Some(6.0),
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(900.0, 420.0))),
                events: vec![
                    egui::Event::PointerMoved(jump_pos),
                    egui::Event::PointerButton { pos: jump_pos, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
                    egui::Event::PointerButton { pos: jump_pos, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() },
                ],
                ..Default::default()
            },
            |ui| show_contents(&mut app, &plot_f2, ui, &t),
        );
        jump_out.textures_delta.clear();
        assert_ne!(app.plot().unwrap().frame().map(|(k, ..)| k), Some(2), "jump banner switches away from unevaluated frame 2");

        // 5. build_params omits top-level domain_id only when complete FAD already sets station_mode: domain_id.
        let mut st = FfsState { selected_domain: Some("cf_0002".into()), ..Default::default() };
        let (p_no_fad, _) = st.build_params("Pull", None, &plot.units, true);
        assert_eq!(p_no_fad["domain_id"], "cf_0002");

        st.fad_enabled = true;
        st.station_mode = StationModeChoice::SelectedStation;
        st.sigma_ref_full_si = Some(200.0e6);
        st.k_mat_si = Some(80.0e6);
        let (p_fad_sel, inc) = st.build_params("Pull", None, &plot.units, true);
        assert!(!inc);
        assert!(p_fad_sel.get("domain_id").is_none(), "must omit top-level domain_id when fad sets station_mode=domain_id");
        assert_eq!(p_fad_sel["fad"]["station_mode"], "domain_id");
        assert_eq!(p_fad_sel["fad"]["domain_id"], "cf_0002");

        // 6. When ring_start > 1 (rings 5..=8), Exclude Ring 1 is disabled and Rings 5–8 is displayed.
        let (mut app_outer, _, _) = build_sparse_mixed_mode_fracture_app(5);
        app_outer.open_ffs();
        let plot_outer = app_outer.plot().unwrap();
        let mut outer_out = ctx.run_ui(
            egui::RawInput { time: Some(7.0), screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(900.0, 420.0))), ..Default::default() },
            |ui| show_contents(&mut app_outer, &plot_outer, ui, &t),
        );
        outer_out.textures_delta.clear();
        let mut outer_texts = Vec::new();
        collect_texts(&outer_out.shapes, &mut outer_texts);
        assert!(outer_texts.iter().any(|s| s == "Rings 5–8"), "{outer_texts:?}");
        // Try clicking "Exclude Ring 1" — since it's disabled, ring_exclusion stays Auto!
        let ex_pos = find_button_center(&outer_out.shapes, "Exclude Ring 1").expect("Exclude Ring 1 label");
        let mut ex_click = ctx.run_ui(
            egui::RawInput {
                time: Some(8.0),
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, vec2(900.0, 420.0))),
                events: vec![
                    egui::Event::PointerMoved(ex_pos),
                    egui::Event::PointerButton { pos: ex_pos, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
                    egui::Event::PointerButton { pos: ex_pos, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() },
                ],
                ..Default::default()
            },
            |ui| show_contents(&mut app_outer, &plot_outer, ui, &t),
        );
        ex_click.textures_delta.clear();
        assert_eq!(app_outer.ffs.as_ref().unwrap().ring_exclusion, RingExclusionChoice::Auto);
    }
}

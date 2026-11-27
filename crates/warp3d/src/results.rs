//! WARP3D result readers: flat-file nodal/element results and domain-integral stdout tables.
//!
//! Reads WARP3D ASCII flat text output (`wnd*`, `wns*`, `wes*`, `wnr*`) across 5-digit and
//! 7-digit step numbers, masks unconstrained DOFs on reaction records, reorders Voigt shear
//! components (`4 <-> 5`) and element indices (`fc_elem_to_warp`), synthesizes nonlinear Frame 0
//! histories, and converts all fields back into FieldCraft SI units via [`DisplaySystem`].

use std::collections::{BTreeMap, BTreeSet};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use fieldcraft_model::units::{DisplaySystem, Preset, Quantity};
use fieldcraft_model::{BoundaryCondition, OutputVariable, Procedure, Resolved, Step, Target};
use fieldcraft_solver::nonlinear::{Frame, History, Increment, MAX_FIELD_VALUES, MAX_FRAME_VALUES};
use fieldcraft_solver::{ElementStress, StaticResult, Stats};

use crate::deck::{piecewise_load_factor, select_step};
use crate::{DomainIntegralResult, InteractionStressState, LabelMap, MAX_RINGS_PER_DOMAIN, Result, RingIntegral, Warp3dError, Warp3dSolution};

const MAX_STDOUT_BYTES: u64 = 64 * 1024 * 1024;
pub(crate) const MAX_LINE_BYTES: usize = 64 * 1024;
const MAX_FLAT_FILE_BYTES: u64 = 256 * 1024 * 1024;
const MAX_FLAT_LINE_BYTES: usize = 16 * 1024;
const MAX_SKIPPED_LINES: usize = 100_000;
const MAX_STEPS: usize = 10_000;
const MAX_INP_HEADER_LINES: usize = 128;
const MAX_DOMAIN_LINES: usize = 2_000_000;
pub(crate) const MAX_DOMAINS: usize = 4_096;
const MAX_DOMAIN_WARNINGS: usize = 32;
const STEP1_ELASTIC_PEEQ_TOL: f64 = 1e-12; // unit-factors: dimensionless plastic strain zero threshold
pub(crate) const MIN_J_DENOM: f64 = 1e-12; // unit-factors: denominator floor for path-independence ring spread
pub(crate) const MIN_VEC_NORM: f64 = 1e-15;

pub(crate) fn clamp_str_utf8(s: &str, max_bytes: usize) -> &str {
    if s.len() <= max_bytes {
        return s;
    }
    let mut end = max_bytes;
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    s.get(..end).unwrap_or("")
}

/// Reads a single `\n`-delimited line bounded to `max_bytes`, draining any remainder to `\n`/EOF.
pub(crate) fn read_bounded_line<R: BufRead>(reader: &mut R, scratch: &mut Vec<u8>, max_bytes: usize) -> std::io::Result<Option<(String, bool)>> {
    scratch.clear();
    let mut truncated = false;
    let mut saw_any = false;

    loop {
        let chunk = match reader.fill_buf() {
            Ok(c) => c,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e),
        };
        if chunk.is_empty() {
            break;
        }
        saw_any = true;
        if let Some(nl_pos) = chunk.iter().position(|&b| b == b'\n') {
            let line_slice = &chunk[..nl_pos];
            let rem = max_bytes.saturating_sub(scratch.len());
            if line_slice.len() > rem {
                scratch.extend_from_slice(&line_slice[..rem]);
                truncated = true;
            } else {
                scratch.extend_from_slice(line_slice);
            }
            reader.consume(nl_pos + 1);
            break;
        }
        let rem = max_bytes.saturating_sub(scratch.len());
        if chunk.len() > rem {
            scratch.extend_from_slice(&chunk[..rem]);
            truncated = true;
        } else {
            scratch.extend_from_slice(chunk);
        }
        let n = chunk.len();
        reader.consume(n);
    }

    if !saw_any {
        return Ok(None);
    }
    if scratch.last() == Some(&b'\r') {
        scratch.pop();
    }
    Ok(Some((String::from_utf8_lossy(scratch).into_owned(), truncated)))
}

#[derive(Debug, Default)]
struct StepFiles {
    wnd: Option<PathBuf>,
    wns: Option<PathBuf>,
    wes: Option<PathBuf>,
    wnr: Option<PathBuf>,
}

fn compute_ring_spread(j_min: f64, j_max: f64, j_mean: f64) -> f64 {
    if !j_min.is_finite() || !j_max.is_finite() || !j_mean.is_finite() {
        return 0.0;
    }
    (j_max - j_min).abs() / j_mean.abs().max(MIN_J_DENOM)
}

fn trim_warp_token(tok: &str) -> &str {
    tok.trim().trim_matches([',', ';', '(', ')'])
}

fn parse_warp_f64(tok: &str) -> Option<f64> {
    let cleaned = trim_warp_token(tok);
    if cleaned.is_empty() || cleaned.contains('*') {
        return None;
    }
    let normalized =
        if cleaned.contains(['d', 'D']) { std::borrow::Cow::Owned(cleaned.replace(['d', 'D'], "e")) } else { std::borrow::Cow::Borrowed(cleaned) };
    let v: f64 = normalized.parse().ok()?;
    v.is_finite().then_some(v)
}

fn parse_warp_u32(tok: &str) -> Option<u32> {
    let id: u32 = trim_warp_token(tok).parse().ok()?;
    (id > 0).then_some(id)
}

fn parse_warp_usize(tok: &str) -> Option<usize> {
    trim_warp_token(tok).parse().ok()
}

fn map_warp_node(label_map: &LabelMap, warp_id: u32) -> Option<u32> {
    usize::try_from(warp_id).ok().and_then(|id| id.checked_sub(1)).and_then(|idx| label_map.warp_node_to_fc.get(idx).copied())
}

fn normalize_vec3(v: [f64; 3]) -> [f64; 3] {
    if !v.iter().all(|c| c.is_finite()) {
        return [0.0; 3];
    }
    let scale = v.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
    if !scale.is_finite() || scale <= MIN_VEC_NORM {
        return [0.0; 3];
    }
    let scaled = v.map(|c| c / scale);
    let norm = (scaled[0] * scaled[0] + scaled[1] * scaled[1] + scaled[2] * scaled[2]).sqrt();
    if !norm.is_finite() || norm <= MIN_VEC_NORM { [0.0; 3] } else { scaled.map(|c| c / norm) }
}

fn vec3_is_nonzero(v: [f64; 3]) -> bool {
    v.iter().all(|c| c.is_finite()) && v.iter().any(|c| c.abs() > MIN_VEC_NORM)
}

fn cross_unit(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    normalize_vec3([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])
}

fn parse_vec3_tail(s: &str) -> Option<[f64; 3]> {
    let mut vals = [0.0f64; 3];
    let mut count = 0usize;
    for tok in s.split_whitespace() {
        if let Some(v) = parse_warp_f64(tok) {
            if let Some(slot) = vals.get_mut(count) {
                *slot = v;
            }
            count += 1;
            if count == 3 {
                return Some(vals);
            }
        }
    }
    None
}

fn collect_all_floats(line: &str) -> Vec<f64> {
    let spaced = line.replace(':', ": ");
    spaced.split_whitespace().filter_map(parse_warp_f64).collect()
}

fn parse_avg_min_max_line(line: &str) -> Option<(f64, f64, f64)> {
    let spaced = line.replace(':', ": ");
    let mut avg = None;
    let mut min = None;
    let mut max = None;
    let tokens: Vec<&str> = spaced.split_whitespace().collect();
    for (i, tok) in tokens.iter().enumerate() {
        let lower = tok.to_ascii_lowercase();
        if lower.starts_with("average") {
            avg = tokens.get(i + 1).and_then(|t| parse_warp_f64(t));
        } else if lower.starts_with("minimum") {
            min = tokens.get(i + 1).and_then(|t| parse_warp_f64(t));
        } else if lower.starts_with("maximum") {
            max = tokens.get(i + 1).and_then(|t| parse_warp_f64(t));
        }
    }
    match (avg, min, max) {
        (Some(a), Some(lo), Some(hi)) => Some((a, lo, hi)),
        _ => None,
    }
}

#[derive(Debug, Clone, Default)]
struct DumpMeta {
    domain_id: String,
    expected_nodes: usize,
    reading_nodes: bool,
    station_fc_nodes: Vec<u32>,
    normal: Option<[f64; 3]>,
    tangent: Option<[f64; 3]>,
    domain_type: Option<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WarpSection {
    None,
    JComponents,
    StaticJ,
    KFromJ,
    StaticIHeader,
    Ki { plane_strain: bool },
    Kii { plane_strain: bool },
    Kiii,
    JFromKs,
    TStress { plane_strain: bool },
    T13,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SchematicSection {
    JTable,
    InteractionTable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParserMode {
    None,
    Warp(WarpSection),
    Schematic(SchematicSection),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PendingSummary {
    None,
    Ki,
    Kii,
    Kiii,
    TStressRow1,
    TStressRow2,
}

#[derive(Debug)]
struct DomainAccum {
    domain_id: String,
    step: usize,
    origin_fc: Option<u32>,
    dump_fc_nodes: Vec<u32>,
    explicit_front_nodes: Vec<u32>,
    domain_type: Option<u8>,
    s_position: Option<f64>,
    normal: [f64; 3],
    tangent: [f64; 3],
    advance: [f64; 3],
    area_aq: Option<f64>,
    rings: BTreeMap<usize, RingIntegral>,
    j_mean: Option<f64>,
    j_min: Option<f64>,
    j_max: Option<f64>,
    j_total: Option<f64>,
    k_j_pstrain_sum: f64,
    k_j_pstrain_cnt: usize,
    k_j_pstress_sum: f64,
    k_j_pstress_cnt: usize,
    k_j_plane_strain_explicit: Option<f64>,
    k_j_plane_stress_explicit: Option<f64>,
    k_i_mean: Option<f64>,
    k_ii_mean: Option<f64>,
    k_iii_mean: Option<f64>,
    t_11_mean: Option<f64>,
    t_33_mean: Option<f64>,
    interaction_stress_state: Option<InteractionStressState>,
    symmetric_zero_modes: bool,
    has_invalid_summary: bool,
}

impl DomainAccum {
    fn new(domain_id: String, step: usize, dump: Option<DumpMeta>) -> Self {
        let mut normal = [0.0; 3];
        let mut tangent = [0.0; 3];
        let mut dump_fc_nodes = Vec::new();
        let mut domain_type = None;
        if let Some(d) = dump {
            if let Some(n) = d.normal {
                normal = normalize_vec3(n);
            }
            if let Some(t) = d.tangent {
                tangent = normalize_vec3(t);
            }
            dump_fc_nodes = d.station_fc_nodes;
            domain_type = d.domain_type;
        }
        Self {
            domain_id,
            step: step.max(1),
            origin_fc: None,
            dump_fc_nodes,
            explicit_front_nodes: Vec::new(),
            domain_type,
            s_position: None,
            normal,
            tangent,
            advance: [0.0; 3],
            area_aq: None,
            rings: BTreeMap::new(),
            j_mean: None,
            j_min: None,
            j_max: None,
            j_total: None,
            k_j_pstrain_sum: 0.0,
            k_j_pstrain_cnt: 0,
            k_j_pstress_sum: 0.0,
            k_j_pstress_cnt: 0,
            k_j_plane_strain_explicit: None,
            k_j_plane_stress_explicit: None,
            k_i_mean: None,
            k_ii_mean: None,
            k_iii_mean: None,
            t_11_mean: None,
            t_33_mean: None,
            interaction_stress_state: None,
            symmetric_zero_modes: false,
            has_invalid_summary: false,
        }
    }

    fn ring_entry(&mut self, ring: usize) -> Option<&mut RingIntegral> {
        if ring == 0 || (!self.rings.contains_key(&ring) && self.rings.len() >= MAX_RINGS_PER_DOMAIN) {
            return None;
        }
        Some(self.rings.entry(ring).or_insert(RingIntegral { ring, j: 0.0, k_i: None, k_ii: None, k_iii: None, t_11: None, t_33: None }))
    }

    fn build(mut self, allow_interaction_only: bool) -> Option<(DomainIntegralResult, Vec<u32>)> {
        let has_interaction_summary =
            self.k_i_mean.is_some() || self.k_ii_mean.is_some() || self.k_iii_mean.is_some() || self.t_11_mean.is_some() || self.t_33_mean.is_some();
        if self.rings.is_empty() && self.j_mean.is_none() && !(allow_interaction_only && has_interaction_summary) {
            return None;
        }
        if self.has_invalid_summary && self.rings.is_empty() {
            return None;
        }
        let rings: Vec<RingIntegral> = self.rings.into_values().collect();

        let (j_mean, j_min, j_max) = match (self.j_mean, self.j_min, self.j_max) {
            (Some(mean), Some(lo), Some(hi)) => (mean, lo, hi),
            _ if !rings.is_empty() => {
                let mut sum = 0.0;
                let mut lo = f64::INFINITY;
                let mut hi = f64::NEG_INFINITY;
                for r in &rings {
                    sum += r.j;
                    lo = lo.min(r.j);
                    hi = hi.max(r.j);
                }
                let mean = self.j_mean.unwrap_or(sum / rings.len() as f64);
                (mean, self.j_min.unwrap_or(lo), self.j_max.unwrap_or(hi))
            }
            (Some(mean), lo, hi) => (mean, lo.unwrap_or(mean), hi.unwrap_or(mean)),
            _ if allow_interaction_only && has_interaction_summary => (0.0, 0.0, 0.0),
            _ => return None,
        };

        let mean_of = |f: fn(&RingIntegral) -> Option<f64>| -> Option<f64> {
            let mut sum = 0.0;
            let mut cnt = 0usize;
            for r in &rings {
                if let Some(v) = f(r) {
                    sum += v;
                    cnt += 1;
                }
            }
            (cnt > 0).then(|| sum / cnt as f64)
        };

        let k_j_plane_strain =
            self.k_j_plane_strain_explicit.or_else(|| (self.k_j_pstrain_cnt > 0).then(|| self.k_j_pstrain_sum / self.k_j_pstrain_cnt as f64));
        let k_j_plane_stress =
            self.k_j_plane_stress_explicit.or_else(|| (self.k_j_pstress_cnt > 0).then(|| self.k_j_pstress_sum / self.k_j_pstress_cnt as f64));

        let k_i_mean = self.k_i_mean.or_else(|| mean_of(|r| r.k_i));
        let mut k_ii_mean = self.k_ii_mean.or_else(|| mean_of(|r| r.k_ii));
        let mut k_iii_mean = self.k_iii_mean.or_else(|| mean_of(|r| r.k_iii));
        let t_11_mean = self.t_11_mean.or_else(|| mean_of(|r| r.t_11));
        let t_33_mean = self.t_33_mean.or_else(|| mean_of(|r| r.t_33));

        let mut rings = rings;
        if self.symmetric_zero_modes && k_i_mean.is_some() {
            for r in &mut rings {
                if r.k_i.is_some() {
                    r.k_ii.get_or_insert(0.0);
                    r.k_iii.get_or_insert(0.0);
                }
            }
            k_ii_mean.get_or_insert(0.0);
            k_iii_mean.get_or_insert(0.0);
        }

        let front_nodes = if !self.explicit_front_nodes.is_empty() {
            self.explicit_front_nodes.clone()
        } else if self.domain_type == Some(4) && !self.dump_fc_nodes.is_empty() {
            let mut dedup = Vec::with_capacity(self.dump_fc_nodes.len());
            for &n in &self.dump_fc_nodes {
                if !dedup.contains(&n) {
                    dedup.push(n);
                }
            }
            dedup
        } else if let Some(orig) = self.origin_fc {
            vec![orig]
        } else if !self.dump_fc_nodes.is_empty() {
            let pick = match self.domain_type {
                Some(1) => self.dump_fc_nodes.first().copied(),
                Some(3) => self.dump_fc_nodes.last().copied(),
                _ => self.dump_fc_nodes.get(self.dump_fc_nodes.len() / 2).copied(),
            };
            pick.into_iter().collect()
        } else {
            Vec::new()
        };

        let station_nodes = if !self.dump_fc_nodes.is_empty() { self.dump_fc_nodes } else { front_nodes.clone() };

        self.normal = normalize_vec3(self.normal);
        self.tangent = normalize_vec3(self.tangent);
        self.advance = if vec3_is_nonzero(self.advance) {
            normalize_vec3(self.advance)
        } else if vec3_is_nonzero(self.normal) && vec3_is_nonzero(self.tangent) {
            cross_unit(self.normal, self.tangent)
        } else {
            [0.0; 3]
        };

        let ring_spread = compute_ring_spread(j_min, j_max, j_mean);
        let res = DomainIntegralResult {
            domain_id: self.domain_id,
            step: self.step,
            front_nodes,
            s_position: self.s_position,
            normal: self.normal,
            tangent: self.tangent,
            advance: self.advance,
            area_aq: self.area_aq.unwrap_or(0.0),
            rings,
            j_mean,
            j_min,
            j_max,
            ring_spread,
            j_total: Some(self.j_total.unwrap_or(j_mean)),
            k_j_plane_strain,
            k_j_plane_stress,
            k_i_mean,
            k_ii_mean,
            k_iii_mean,
            t_11_mean,
            t_33_mean,
            interaction_stress_state: self.interaction_stress_state,
        };
        Some((res, station_nodes))
    }
}

fn push_finalized(out: &mut Vec<(DomainIntegralResult, Vec<u32>)>, acc: DomainAccum, truncated: &mut bool) {
    if let Some(built) = acc.build(false) {
        if out.len() >= MAX_DOMAINS {
            *truncated = true;
            return;
        }
        out.push(built);
    }
}

fn flush_active_domain(out: &mut Vec<(DomainIntegralResult, Vec<u32>)>, acc: DomainAccum, is_schematic_interaction: bool, truncated: &mut bool) {
    if is_schematic_interaction {
        if let Some((built, _)) = acc.build(true)
            && let Some((existing, _)) = out.iter_mut().rev().find(|(r, _)| r.domain_id == built.domain_id && r.step == built.step)
        {
            merge_interaction_into_existing(existing, built);
        }
    } else {
        push_finalized(out, acc, truncated);
    }
}

fn parse_keyed_f64(lower_line: &str, raw_line: &str, keys: &[&str]) -> Option<(bool, Option<f64>)> {
    for key in keys {
        if let Some(pos) = lower_line.find(key) {
            let after = raw_line.get(pos + key.len()..).unwrap_or("");
            let cleaned = after.trim_start_matches([':', '=', ' ', '\t']);
            if let Some(tok) = cleaned.split_whitespace().next() {
                return Some((true, parse_warp_f64(tok)));
            }
            return Some((true, None));
        }
    }
    None
}

fn parse_schematic_header(line: &str) -> Option<(String, bool, usize)> {
    let trimmed = line.trim();
    let lower = trimmed.to_ascii_lowercase();
    let rest = if lower.starts_with("domain:") { trimmed.get(7..)? } else { return None };
    let rest_trim = rest.trim();
    if rest_trim.is_empty() {
        return None;
    }
    let is_interaction = rest_trim.to_ascii_lowercase().contains("(interaction integral");
    let before_paren = rest_trim.split('(').next().unwrap_or("").trim();
    let id = before_paren.split_whitespace().next()?.to_string();
    let mut step = 1usize;
    let rest_lower = rest_trim.to_ascii_lowercase();
    if let Some(pos) = rest_lower.find("step")
        && let Some(after) = rest_trim.get(pos + 4..)
    {
        for tok in after.split_whitespace() {
            if let Some(n) = parse_warp_usize(tok.trim_start_matches([':', '='])) {
                if n > 0 {
                    step = n;
                }
                break;
            }
        }
    }
    Some((id, is_interaction, step))
}

fn merge_interaction_into_existing(existing: &mut DomainIntegralResult, incoming: DomainIntegralResult) {
    if existing.area_aq == 0.0 && incoming.area_aq > 0.0 {
        existing.area_aq = incoming.area_aq;
    }
    if existing.front_nodes.is_empty() && !incoming.front_nodes.is_empty() {
        existing.front_nodes = incoming.front_nodes;
    }
    if vec3_is_nonzero(incoming.normal) && !vec3_is_nonzero(existing.normal) {
        existing.normal = incoming.normal;
    }
    if vec3_is_nonzero(incoming.tangent) && !vec3_is_nonzero(existing.tangent) {
        existing.tangent = incoming.tangent;
    }
    if vec3_is_nonzero(incoming.advance) && !vec3_is_nonzero(existing.advance) {
        existing.advance = incoming.advance;
    } else if !vec3_is_nonzero(existing.advance) && vec3_is_nonzero(existing.normal) && vec3_is_nonzero(existing.tangent) {
        existing.advance = cross_unit(existing.normal, existing.tangent);
    }
    for ir in incoming.rings {
        if let Some(target) = existing.rings.iter_mut().find(|r| r.ring == ir.ring) {
            if ir.k_i.is_some() {
                target.k_i = ir.k_i;
            }
            if ir.k_ii.is_some() {
                target.k_ii = ir.k_ii;
            }
            if ir.k_iii.is_some() {
                target.k_iii = ir.k_iii;
            }
            if ir.t_11.is_some() {
                target.t_11 = ir.t_11;
            }
            if ir.t_33.is_some() {
                target.t_33 = ir.t_33;
            }
        } else if existing.rings.len() < MAX_RINGS_PER_DOMAIN {
            existing.rings.push(ir);
        }
    }
    existing.rings.sort_by_key(|r| r.ring);
    if incoming.k_i_mean.is_some() {
        existing.k_i_mean = incoming.k_i_mean;
    }
    if incoming.k_ii_mean.is_some() {
        existing.k_ii_mean = incoming.k_ii_mean;
    }
    if incoming.k_iii_mean.is_some() {
        existing.k_iii_mean = incoming.k_iii_mean;
    }
    if incoming.t_11_mean.is_some() {
        existing.t_11_mean = incoming.t_11_mean;
    }
    if incoming.t_33_mean.is_some() {
        existing.t_33_mean = incoming.t_33_mean;
    }
    if incoming.interaction_stress_state.is_some() {
        existing.interaction_stress_state = incoming.interaction_stress_state;
    }
}

/// Output of streaming WARP3D domain-integral stdout parsing.
#[derive(Debug, Default)]
pub(crate) struct ParsedDomainOutput {
    pub domains: Vec<DomainIntegralResult>,
    pub station_nodes: Vec<Vec<u32>>,
    pub truncated: bool,
    pub warnings: Vec<String>,
}

pub(crate) struct DomainIntegralParser<'a> {
    label_map: &'a LabelMap,
    results: Vec<(DomainIntegralResult, Vec<u32>)>,
    pending_dumps: BTreeMap<String, DumpMeta>,
    current_dump: Option<DumpMeta>,
    current_step: usize,
    active: Option<DomainAccum>,
    active_is_schematic_interaction: bool,
    mode: ParserMode,
    awaiting_j_summary_values: bool,
    awaiting_i_summary_values: Option<PendingSummary>,
    domain_lines: usize,
    domains_truncated: bool,
    warnings: Vec<String>,
    seen_warnings: BTreeSet<String>,
}

impl<'a> DomainIntegralParser<'a> {
    pub(crate) fn new(label_map: &'a LabelMap) -> Self {
        Self {
            label_map,
            results: Vec::new(),
            pending_dumps: BTreeMap::new(),
            current_dump: None,
            current_step: 1,
            active: None,
            active_is_schematic_interaction: false,
            mode: ParserMode::None,
            awaiting_j_summary_values: false,
            awaiting_i_summary_values: None,
            domain_lines: 0,
            domains_truncated: false,
            warnings: Vec::new(),
            seen_warnings: BTreeSet::new(),
        }
    }

    fn push_warning(&mut self, msg: String) {
        if self.warnings.len() < MAX_DOMAIN_WARNINGS && self.seen_warnings.insert(msg.clone()) {
            self.warnings.push(msg);
        }
    }

    pub(crate) fn feed_line(&mut self, raw_line: &str) {
        let clamped = clamp_str_utf8(raw_line, MAX_LINE_BYTES);
        let trimmed = clamped.trim();
        let lower = trimmed.to_ascii_lowercase();

        // Capture WARP3D `didriv.f` / domain diagnostics before the active-block gate so messages
        // emitted outside an open `domain id:` block still surface to the user.
        if trimmed.starts_with(">>> Invalid domain definition")
            || lower.starts_with(">>> warning:")
            || lower.starts_with(">>> area under q-function")
            || lower.starts_with(">>>>> warning:")
            || lower.starts_with(">>>>> error:")
        {
            self.push_warning(clamp_str_utf8(trimmed, 512).to_string());
        }

        // Track WARP3D `loading: ... step: N ...` line (both outside and inside domain output)
        // without counting non-domain solver filler against `MAX_DOMAIN_LINES`.
        if lower.starts_with("loading:")
            && let Some(pos) = lower.find("step:")
            && let Some(step_tok) = trimmed.get(pos + 5..).and_then(|s| s.split_whitespace().next())
            && let Some(s) = parse_warp_usize(step_tok)
            && s > 0
        {
            self.current_step = s;
        }

        let is_domain_start =
            trimmed.starts_with("Domain definition:") || lower.starts_with("domain id:") || parse_schematic_header(trimmed).is_some();
        if self.current_dump.is_none() && self.active.is_none() && !is_domain_start {
            return;
        }
        if self.domain_lines >= MAX_DOMAIN_LINES {
            self.domains_truncated = true;
            return;
        }
        self.domain_lines += 1;

        let label_map = self.label_map;
        let current_step = self.current_step;
        let mut awaiting_j_summary_values = self.awaiting_j_summary_values;
        let mut awaiting_i_summary_values = self.awaiting_i_summary_values;

        for _single in [()] {
            // 1. WARP3D `dump` block (`Domain definition: <id>`).
            if let Some(rest) = trimmed.strip_prefix("Domain definition:") {
                let id = rest.split_whitespace().next().unwrap_or("").to_string();
                if !id.is_empty() {
                    self.current_dump = Some(DumpMeta { domain_id: id, ..DumpMeta::default() });
                }
                continue;
            }
            if let Some(dump) = self.current_dump.as_mut() {
                if trimmed.contains("** end of domain definition **") {
                    let finished = std::mem::take(dump);
                    self.current_dump = None;
                    if self.pending_dumps.len() < MAX_DOMAINS || self.pending_dumps.contains_key(&finished.domain_id) {
                        self.pending_dumps.insert(finished.domain_id.clone(), finished);
                    } else {
                        self.domains_truncated = true;
                    }
                    continue;
                }
                if dump.reading_nodes {
                    let tokens: Vec<&str> = trimmed.split_whitespace().collect();
                    if tokens.is_empty() {
                        if dump.station_fc_nodes.len() >= dump.expected_nodes && dump.expected_nodes > 0 {
                            dump.reading_nodes = false;
                        }
                        continue;
                    }
                    if tokens.iter().all(|t| parse_warp_u32(t).is_some()) {
                        let node_cap = if dump.expected_nodes > 0 { dump.expected_nodes.min(100_000) } else { 100_000 };
                        for tok in tokens {
                            if dump.station_fc_nodes.len() >= node_cap {
                                break;
                            }
                            if let Some(wid) = parse_warp_u32(tok)
                                && let Some(fc) = map_warp_node(label_map, wid)
                            {
                                dump.station_fc_nodes.push(fc);
                            }
                        }
                        if dump.expected_nodes > 0 && dump.station_fc_nodes.len() >= dump.expected_nodes {
                            dump.reading_nodes = false;
                        }
                        continue;
                    }
                    dump.reading_nodes = false;
                }
                if let Some(after) = trimmed.strip_prefix("Number of front nodes:") {
                    if let Some(n_tok) = after.split_whitespace().next()
                        && let Some(n) = parse_warp_usize(n_tok)
                    {
                        dump.expected_nodes = n.min(100_000);
                    }
                    if trimmed.contains("Nodes on front:") {
                        dump.reading_nodes = true;
                    }
                } else if let Some(after) = trimmed.strip_prefix("Direction cosines:") {
                    dump.normal = parse_vec3_tail(after);
                } else if let Some(after) = trimmed.strip_prefix("Tangent vector direction cosines:") {
                    dump.tangent = parse_vec3_tail(after);
                } else if let Some(pos) = trimmed.find("Domain type:") {
                    let after = trimmed.get(pos + "Domain type:".len()..).unwrap_or("");
                    if let Some(dt_tok) = after.split_whitespace().next()
                        && let Some(dt) = parse_warp_usize(dt_tok)
                        && (1..=4).contains(&dt)
                    {
                        dump.domain_type = Some(dt as u8);
                    }
                }
                continue;
            }

            // 2. Real WARP3D `domain id: <id>` header (`diheadr` output).
            if lower.starts_with("domain id:")
                && let Some(after) = trimmed.get("domain id:".len()..)
            {
                if let Some(prev) = self.active.take() {
                    flush_active_domain(&mut self.results, prev, self.active_is_schematic_interaction, &mut self.domains_truncated);
                }
                let id = after.split_whitespace().next().unwrap_or("").to_string();
                if !id.is_empty() {
                    let dump = self.pending_dumps.get(&id).cloned();
                    self.active = Some(DomainAccum::new(id, current_step, dump));
                    self.active_is_schematic_interaction = false;
                    self.mode = ParserMode::Warp(WarpSection::None);
                    awaiting_j_summary_values = false;
                    awaiting_i_summary_values = None;
                }
                continue;
            }

            // 3. Schematic `Domain: <id>` header.
            if let Some((id, is_interaction, step_hint)) = parse_schematic_header(trimmed) {
                if let Some(prev) = self.active.take() {
                    flush_active_domain(&mut self.results, prev, self.active_is_schematic_interaction, &mut self.domains_truncated);
                }
                let dump = self.pending_dumps.get(&id).cloned();
                let mut acc = DomainAccum::new(id, step_hint.max(current_step), dump);
                if step_hint > 1 {
                    acc.step = step_hint;
                }
                self.active = Some(acc);
                self.active_is_schematic_interaction = is_interaction;
                self.mode = ParserMode::Schematic(if is_interaction { SchematicSection::InteractionTable } else { SchematicSection::JTable });
                awaiting_j_summary_values = false;
                awaiting_i_summary_values = None;
                continue;
            }

            let Some(acc) = self.active.as_mut() else {
                continue;
            };

            if trimmed.is_empty() {
                continue;
            }

            if trimmed.contains(">>> Completed domain integral calculations") || trimmed.starts_with(">>> Invalid domain definition") {
                let finished = self.active.take();
                let was_interaction = self.active_is_schematic_interaction;
                self.active_is_schematic_interaction = false;
                self.mode = ParserMode::None;
                awaiting_j_summary_values = false;
                awaiting_i_summary_values = None;
                if let Some(prev) = finished
                    && !trimmed.starts_with(">>> Invalid domain definition")
                {
                    flush_active_domain(&mut self.results, prev, was_interaction, &mut self.domains_truncated);
                }
                continue;
            }

            // Common domain metadata fields (both real WARP3D and schematic formats).
            if let Some(after) = lower.strip_prefix("area under q along front:").and_then(|_| trimmed.get("area under q along front:".len()..)) {
                if let Some(v) = after.split_whitespace().next().and_then(parse_warp_f64) {
                    acc.area_aq = Some(v);
                }
                continue;
            }
            if lower.starts_with("young's modulus at crack front node") {
                if let Some(pos) = lower.find("node")
                    && let Some(after) = trimmed.get(pos + 4..)
                    && let Some(wid) = after.split_whitespace().next().and_then(parse_warp_u32)
                    && let Some(fc) = map_warp_node(label_map, wid)
                {
                    acc.origin_fc.get_or_insert(fc);
                }
                continue;
            }

            match self.mode {
                ParserMode::Warp(ref mut sec) => {
                    // Handle continuation lines after `average: minimum: maximum:`.
                    if awaiting_j_summary_values {
                        let nums = collect_all_floats(trimmed);
                        awaiting_j_summary_values = false;
                        if nums.len() == 3 && *sec != WarpSection::StaticJ {
                            acc.j_mean = nums.first().copied();
                            acc.j_min = nums.get(1).copied();
                            acc.j_max = nums.get(2).copied();
                        }
                        continue;
                    }
                    if let Some(ps) = awaiting_i_summary_values {
                        let nums = collect_all_floats(trimmed);
                        match ps {
                            PendingSummary::TStressRow1 => {
                                if let Some(&avg) = nums.first() {
                                    acc.t_11_mean = Some(avg);
                                }
                                awaiting_i_summary_values = Some(PendingSummary::TStressRow2);
                                continue;
                            }
                            PendingSummary::TStressRow2 => {
                                if let Some(&avg) = nums.first() {
                                    acc.t_33_mean = Some(avg);
                                }
                                awaiting_i_summary_values = None;
                                continue;
                            }
                            PendingSummary::Ki => {
                                if let Some(&avg) = nums.first() {
                                    acc.k_i_mean = Some(avg);
                                }
                                awaiting_i_summary_values = None;
                                continue;
                            }
                            PendingSummary::Kii => {
                                if let Some(&avg) = nums.first() {
                                    acc.k_ii_mean = Some(avg);
                                }
                                awaiting_i_summary_values = None;
                                continue;
                            }
                            PendingSummary::Kiii => {
                                if let Some(&avg) = nums.first() {
                                    acc.k_iii_mean = Some(avg);
                                }
                                awaiting_i_summary_values = None;
                                continue;
                            }
                            _ => {
                                awaiting_i_summary_values = None;
                                continue;
                            }
                        }
                    }

                    // Section transitions in WARP3D `di1dsf.f` output.
                    if lower.contains("j-integral components") {
                        *sec = WarpSection::JComponents;
                        continue;
                    }
                    if lower == "static j-values" {
                        *sec = WarpSection::StaticJ;
                        continue;
                    }
                    if lower.contains("stress intensity factors obtained from j-values") {
                        *sec = WarpSection::KFromJ;
                        continue;
                    }
                    if lower == "static i-values" {
                        *sec = WarpSection::StaticIHeader;
                        continue;
                    }
                    if lower.contains("values for kii, kiii and t13 set to zero") {
                        acc.symmetric_zero_modes = true;
                        continue;
                    }
                    if lower.contains("ki = 1, kii = 0, kiii = 0, plane stress") {
                        acc.interaction_stress_state.get_or_insert(InteractionStressState::PlaneStress);
                        *sec = WarpSection::Ki { plane_strain: false };
                        continue;
                    }
                    if lower.contains("ki = 1, kii = 0, kiii = 0, plane strain") {
                        acc.interaction_stress_state = Some(InteractionStressState::PlaneStrain);
                        *sec = WarpSection::Ki { plane_strain: true };
                        continue;
                    }
                    if lower.contains("ki = 0, kii = 1, kiii = 0, plane stress") {
                        acc.interaction_stress_state.get_or_insert(InteractionStressState::PlaneStress);
                        *sec = WarpSection::Kii { plane_strain: false };
                        continue;
                    }
                    if lower.contains("ki = 0, kii = 1, kiii = 0, plane strain") {
                        acc.interaction_stress_state = Some(InteractionStressState::PlaneStrain);
                        *sec = WarpSection::Kii { plane_strain: true };
                        continue;
                    }
                    if lower.contains("ki = 0, kii = 0, kiii = 1") {
                        *sec = WarpSection::Kiii;
                        continue;
                    }
                    if lower.contains("j-values computed from mixed-mode") {
                        *sec = WarpSection::JFromKs;
                        continue;
                    }
                    if lower.contains("t-stress (plane stress") {
                        acc.interaction_stress_state.get_or_insert(InteractionStressState::PlaneStress);
                        *sec = WarpSection::TStress { plane_strain: false };
                        continue;
                    }
                    if lower.contains("t-stress (\"plane strain\"") || lower.contains("t-stress (plane strain") {
                        acc.interaction_stress_state = Some(InteractionStressState::PlaneStrain);
                        *sec = WarpSection::TStress { plane_strain: true };
                        continue;
                    }
                    if lower.contains("t-stress (anti-plane shear") {
                        *sec = WarpSection::T13;
                        continue;
                    }

                    // `average: ... minimum: ... maximum:` summary lines (`9020` and `9115`).
                    if lower.contains("average:") && lower.contains("minimum:") && lower.contains("maximum:") {
                        match *sec {
                            WarpSection::JComponents | WarpSection::StaticJ => {
                                if let Some((avg, lo, hi)) = parse_avg_min_max_line(trimmed) {
                                    if *sec != WarpSection::StaticJ {
                                        acc.j_mean = Some(avg);
                                        acc.j_min = Some(lo);
                                        acc.j_max = Some(hi);
                                    }
                                    // Skip the immediately-following duplicate 3-float line (`9020`).
                                    awaiting_j_summary_values = true;
                                } else {
                                    awaiting_j_summary_values = true;
                                }
                                continue;
                            }
                            WarpSection::Ki { plane_strain } => {
                                if let Some((avg, _, _)) = parse_avg_min_max_line(trimmed) {
                                    if plane_strain || acc.k_i_mean.is_none() {
                                        acc.k_i_mean = Some(avg);
                                    }
                                    // Consume the duplicate line below `9020` cleanly.
                                    awaiting_i_summary_values = Some(PendingSummary::None);
                                } else {
                                    awaiting_i_summary_values = Some(PendingSummary::Ki);
                                }
                                continue;
                            }
                            WarpSection::Kii { plane_strain } => {
                                if let Some((avg, _, _)) = parse_avg_min_max_line(trimmed) {
                                    if plane_strain || acc.k_ii_mean.is_none() {
                                        acc.k_ii_mean = Some(avg);
                                    }
                                    awaiting_i_summary_values = Some(PendingSummary::None);
                                } else {
                                    awaiting_i_summary_values = Some(PendingSummary::Kii);
                                }
                                continue;
                            }
                            WarpSection::Kiii => {
                                if let Some((avg, _, _)) = parse_avg_min_max_line(trimmed) {
                                    acc.k_iii_mean = Some(avg);
                                    awaiting_i_summary_values = Some(PendingSummary::None);
                                } else {
                                    awaiting_i_summary_values = Some(PendingSummary::Kiii);
                                }
                                continue;
                            }
                            WarpSection::TStress { .. } => {
                                awaiting_i_summary_values = Some(PendingSummary::TStressRow1);
                                continue;
                            }
                            _ => continue,
                        }
                    }

                    // Skip explanatory footnotes (`* dm1:`, `domain`, `===`, `---`, `>>>>> warning`).
                    if trimmed.starts_with('*') || trimmed.starts_with('-') || trimmed.starts_with('=') || trimmed.starts_with('>') {
                        continue;
                    }

                    let tokens: Vec<&str> = trimmed.split_whitespace().collect();
                    let Some(ring) = tokens.first().and_then(|t| parse_warp_usize(t)).filter(|&r| r > 0) else {
                        continue;
                    };

                    match *sec {
                        WarpSection::JComponents => {
                            if tokens.len() >= 10
                                && (1..=9).all(|idx| tokens.get(idx).and_then(|t| parse_warp_f64(t)).is_some())
                                && let Some(total_j) = tokens.get(9).and_then(|t| parse_warp_f64(t))
                                && let Some(entry) = acc.ring_entry(ring)
                            {
                                entry.j = total_j;
                            }
                        }
                        WarpSection::KFromJ => {
                            if tokens.len() >= 3
                                && let (Some(k_pstrs), Some(k_pstrn)) =
                                    (tokens.get(1).and_then(|t| parse_warp_f64(t)), tokens.get(2).and_then(|t| parse_warp_f64(t)))
                            {
                                acc.k_j_pstress_sum += k_pstrs;
                                acc.k_j_pstress_cnt += 1;
                                acc.k_j_pstrain_sum += k_pstrn;
                                acc.k_j_pstrain_cnt += 1;
                            }
                        }
                        WarpSection::Ki { plane_strain } => {
                            if tokens.len() >= 11
                                && (1..=10).all(|idx| tokens.get(idx).and_then(|t| parse_warp_f64(t)).is_some())
                                && let Some(k_val) = tokens.get(10).and_then(|t| parse_warp_f64(t))
                                && let Some(entry) = acc.ring_entry(ring)
                                && (plane_strain || entry.k_i.is_none())
                            {
                                entry.k_i = Some(k_val);
                            }
                        }
                        WarpSection::Kii { plane_strain } => {
                            if tokens.len() >= 11
                                && (1..=10).all(|idx| tokens.get(idx).and_then(|t| parse_warp_f64(t)).is_some())
                                && let Some(k_val) = tokens.get(10).and_then(|t| parse_warp_f64(t))
                                && let Some(entry) = acc.ring_entry(ring)
                                && (plane_strain || entry.k_ii.is_none())
                            {
                                entry.k_ii = Some(k_val);
                            }
                        }
                        WarpSection::Kiii => {
                            if tokens.len() >= 11
                                && (1..=10).all(|idx| tokens.get(idx).and_then(|t| parse_warp_f64(t)).is_some())
                                && let Some(k_val) = tokens.get(10).and_then(|t| parse_warp_f64(t))
                                && let Some(entry) = acc.ring_entry(ring)
                            {
                                entry.k_iii = Some(k_val);
                            }
                        }
                        WarpSection::TStress { plane_strain } => {
                            if tokens.len() >= 12
                                && (1..=11).all(|idx| tokens.get(idx).and_then(|t| parse_warp_f64(t)).is_some())
                                && let (Some(t11), Some(t33)) =
                                    (tokens.get(10).and_then(|t| parse_warp_f64(t)), tokens.get(11).and_then(|t| parse_warp_f64(t)))
                                && let Some(entry) = acc.ring_entry(ring)
                                && (plane_strain || entry.t_11.is_none())
                            {
                                entry.t_11 = Some(t11);
                                entry.t_33 = Some(t33);
                            }
                        }
                        WarpSection::None | WarpSection::StaticJ | WarpSection::StaticIHeader | WarpSection::JFromKs | WarpSection::T13 => {}
                    }
                }
                ParserMode::Schematic(ref mut sec) => {
                    if let Some(after) = lower.strip_prefix("step:").or_else(|| lower.strip_prefix("load step:"))
                        && let Some(s) = after.split_whitespace().next().and_then(parse_warp_usize)
                        && s > 0
                    {
                        acc.step = s;
                        continue;
                    }
                    if lower.starts_with("front nodes:") || lower.starts_with("front_nodes:") {
                        if let Some(pos) = trimmed.find(':')
                            && let Some(after) = trimmed.get(pos + 1..)
                        {
                            for tok in after.split_whitespace() {
                                if let Some(wid) = parse_warp_u32(tok)
                                    && let Some(fc) = map_warp_node(label_map, wid)
                                {
                                    acc.explicit_front_nodes.push(fc);
                                }
                            }
                        }
                        continue;
                    }
                    if lower.starts_with("crack plane normal") || lower.starts_with("normal:") || lower.starts_with("normal =") {
                        if let Some(v) = parse_vec3_tail(trimmed) {
                            acc.normal = v;
                        }
                        continue;
                    }
                    if lower.starts_with("crack front tangent") || lower.starts_with("tangent:") || lower.starts_with("tangent =") {
                        if let Some(v) = parse_vec3_tail(trimmed) {
                            acc.tangent = v;
                        }
                        continue;
                    }
                    if lower.starts_with("crack advance") || lower.starts_with("advance:") || lower.starts_with("advance =") {
                        if let Some(v) = parse_vec3_tail(trimmed) {
                            acc.advance = v;
                        }
                        continue;
                    }
                    if lower.starts_with("s_position") || lower.starts_with("s =") || lower.starts_with("s:") {
                        if let Some((_, Some(s))) = parse_keyed_f64(&lower, trimmed, &["s_position", "s"]) {
                            acc.s_position = Some(s);
                        }
                        continue;
                    }
                    if lower.starts_with("area") || lower.starts_with("a_q") || lower.starts_with("area_aq") {
                        if let Some((_, Some(a))) = parse_keyed_f64(&lower, trimmed, &["area_aq", "area a_q", "a_q", "area"]) {
                            acc.area_aq = Some(a);
                        }
                        continue;
                    }

                    // Detect table column header in schematic blocks before summary keys.
                    if lower.contains("ring") {
                        if lower.contains("k_i") || lower.contains("ki") || lower.contains("t_11") || lower.contains("t11") {
                            *sec = SchematicSection::InteractionTable;
                        } else if lower.contains('j') {
                            *sec = SchematicSection::JTable;
                        }
                        continue;
                    }

                    // Multi-key summary lines inside schematic blocks (e.g. `J_mean: ... J_min: ... J_max: ...`).
                    let mut matched_summary = false;
                    for (keys, target) in [
                        (&["j_mean", "mean j"][..], "j_mean"),
                        (&["j_min", "min j"][..], "j_min"),
                        (&["j_max", "max j"][..], "j_max"),
                        (&["j_total", "total j"][..], "j_total"),
                        (&["k_j_plane_strain", "k_j (plane strain)"][..], "kj_pstrn"),
                        (&["k_j_plane_stress", "k_j (plane stress)"][..], "kj_pstrs"),
                        (&["k_iii_mean", "kiii_mean", "k_iii:", "k_iii ="][..], "kiii_mean"),
                        (&["k_ii_mean", "kii_mean", "k_ii:", "k_ii ="][..], "kii_mean"),
                        (&["k_i_mean", "ki_mean", "k_i:", "k_i ="][..], "ki_mean"),
                        (&["t_11_mean", "t11_mean", "t_11:", "t_11 ="][..], "t11_mean"),
                        (&["t_33_mean", "t33_mean", "t_33:", "t_33 ="][..], "t33_mean"),
                    ] {
                        if let Some((seen, val)) = parse_keyed_f64(&lower, trimmed, keys)
                            && seen
                        {
                            matched_summary = true;
                            match (target, val) {
                                ("j_mean", Some(v)) => acc.j_mean = Some(v),
                                ("j_min", Some(v)) => acc.j_min = Some(v),
                                ("j_max", Some(v)) => acc.j_max = Some(v),
                                ("j_total", Some(v)) => acc.j_total = Some(v),
                                ("kj_pstrn", Some(v)) => acc.k_j_plane_strain_explicit = Some(v),
                                ("kj_pstrs", Some(v)) => acc.k_j_plane_stress_explicit = Some(v),
                                ("ki_mean", Some(v)) => acc.k_i_mean = Some(v),
                                ("kii_mean", Some(v)) => acc.k_ii_mean = Some(v),
                                ("kiii_mean", Some(v)) => acc.k_iii_mean = Some(v),
                                ("t11_mean", Some(v)) => acc.t_11_mean = Some(v),
                                ("t33_mean", Some(v)) => acc.t_33_mean = Some(v),
                                ("j_mean", None) => acc.has_invalid_summary = true,
                                _ => {}
                            }
                        }
                    }
                    if matched_summary {
                        continue;
                    }

                    let tokens: Vec<&str> = trimmed.split_whitespace().collect();
                    let Some(ring) = tokens.first().and_then(|t| parse_warp_usize(t)).filter(|&r| r > 0) else {
                        continue;
                    };
                    // All remaining numeric columns must parse cleanly as finite floats.
                    let mut cols = Vec::with_capacity(tokens.len().saturating_sub(1));
                    let mut valid_row = true;
                    for tok in tokens.iter().skip(1) {
                        if tok.starts_with('(') {
                            break;
                        }
                        match parse_warp_f64(tok) {
                            Some(v) => cols.push(v),
                            None => {
                                valid_row = false;
                                break;
                            }
                        }
                    }
                    if !valid_row || cols.is_empty() {
                        continue;
                    }

                    match *sec {
                        SchematicSection::InteractionTable => {
                            if let Some(entry) = acc.ring_entry(ring) {
                                if cols.len() >= 3 {
                                    entry.k_i = cols.first().copied();
                                    entry.k_ii = cols.get(1).copied();
                                    entry.k_iii = cols.get(2).copied();
                                }
                                if cols.len() >= 5 {
                                    entry.t_11 = cols.get(3).copied();
                                    entry.t_33 = cols.get(4).copied();
                                }
                            }
                        }
                        SchematicSection::JTable => {
                            if cols.len() >= 9 {
                                if let (Some(&j_val), Some(entry)) = (cols.get(8), acc.ring_entry(ring)) {
                                    entry.j = j_val;
                                }
                            } else if let (Some(&j_val), Some(entry)) = (cols.first(), acc.ring_entry(ring)) {
                                entry.j = j_val;
                                if let Some(&k_pstrn) = cols.get(1) {
                                    acc.k_j_pstrain_sum += k_pstrn;
                                    acc.k_j_pstrain_cnt += 1;
                                }
                                if let Some(&k_pstrs) = cols.get(2) {
                                    acc.k_j_pstress_sum += k_pstrs;
                                    acc.k_j_pstress_cnt += 1;
                                }
                            }
                        }
                    }
                }
                ParserMode::None => {}
            }
        }

        self.awaiting_j_summary_values = awaiting_j_summary_values;
        self.awaiting_i_summary_values = awaiting_i_summary_values;
    }

    pub(crate) fn finish(mut self) -> ParsedDomainOutput {
        if let Some(prev) = self.active.take() {
            flush_active_domain(&mut self.results, prev, self.active_is_schematic_interaction, &mut self.domains_truncated);
        }

        let mut domains = Vec::with_capacity(self.results.len());
        let mut station_nodes = Vec::with_capacity(self.results.len());
        for (d, s) in self.results {
            domains.push(d);
            station_nodes.push(s);
        }
        ParsedDomainOutput { domains, station_nodes, truncated: self.domains_truncated, warnings: self.warnings }
    }
}

fn parse_domain_integrals_with_meta(stdout: &str, label_map: &LabelMap) -> ParsedDomainOutput {
    let mut parser = DomainIntegralParser::new(label_map);
    for raw_line in stdout.lines() {
        parser.feed_line(raw_line);
    }
    parser.finish()
}

/// Parse WARP3D Chapter 4 `Domain:` decomposition and real WARP3D 18.5.0 domain/interaction-integral
/// tables from `stdout`.
pub fn parse_domain_integrals(stdout: &str, label_map: &LabelMap) -> Vec<DomainIntegralResult> {
    parse_domain_integrals_with_meta(stdout, label_map).domains
}

pub(crate) fn convert_domain_integrals_to_si(domains: &mut [DomainIntegralResult], display: &DisplaySystem) -> Result<()> {
    let conv = |q: Quantity, v: f64| -> Result<f64> {
        display.to_si(q, v).map_err(|e| Warp3dError::Parse(format!("domain integral unit conversion error: {e}")))
    };
    let stress_si = conv(Quantity::Pressure, 1.0)?;
    let len_si = conv(Quantity::Length, 1.0)?;
    let k_scale = stress_si * len_si.sqrt();
    if !k_scale.is_finite() {
        return Err(Warp3dError::Parse("non-finite stress intensity unit factor".into()));
    }
    let scale_k = |opt: &mut Option<f64>| {
        if let Some(v) = opt.as_mut() {
            *v *= k_scale;
        }
    };

    for d in domains {
        d.area_aq = conv(Quantity::Area, d.area_aq)?;
        d.j_mean = conv(Quantity::ForcePerLength, d.j_mean)?;
        d.j_min = conv(Quantity::ForcePerLength, d.j_min)?;
        d.j_max = conv(Quantity::ForcePerLength, d.j_max)?;
        if let Some(jt) = d.j_total.as_mut() {
            *jt = conv(Quantity::ForcePerLength, *jt)?;
        }
        scale_k(&mut d.k_j_plane_strain);
        scale_k(&mut d.k_j_plane_stress);
        scale_k(&mut d.k_i_mean);
        scale_k(&mut d.k_ii_mean);
        scale_k(&mut d.k_iii_mean);
        if let Some(t11) = d.t_11_mean.as_mut() {
            *t11 = conv(Quantity::Pressure, *t11)?;
        }
        if let Some(t33) = d.t_33_mean.as_mut() {
            *t33 = conv(Quantity::Pressure, *t33)?;
        }
        for r in &mut d.rings {
            r.j = conv(Quantity::ForcePerLength, r.j)?;
            scale_k(&mut r.k_i);
            scale_k(&mut r.k_ii);
            scale_k(&mut r.k_iii);
            if let Some(t11) = r.t_11.as_mut() {
                *t11 = conv(Quantity::Pressure, *t11)?;
            }
            if let Some(t33) = r.t_33.as_mut() {
                *t33 = conv(Quantity::Pressure, *t33)?;
            }
        }
        d.ring_spread = compute_ring_spread(d.j_min, d.j_max, d.j_mean);
    }
    Ok(())
}

pub(crate) fn enrich_domain_geometry(domains: &mut [DomainIntegralResult], station_nodes: &[Vec<u32>], model: &fieldcraft_model::Model) {
    // Group domain indices by analysis step so we can compute arc-length `s_position` and
    // neighbor-station tangents when dump metadata was absent.
    let mut by_step: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (idx, d) in domains.iter().enumerate() {
        by_step.entry(d.step).or_default().push(idx);
    }

    for indices in by_step.values() {
        let mut s_coords: Vec<Option<[f64; 3]>> = Vec::with_capacity(indices.len());
        for &idx in indices {
            let coord = domains.get(idx).and_then(|d| d.front_nodes.first()).and_then(|&n| model.node(n)).map(|n| n.xyz);
            s_coords.push(coord);
        }

        if indices.len() >= 2 && s_coords.iter().all(Option::is_some) {
            let mut cum = Vec::with_capacity(indices.len());
            cum.push(0.0);
            for k in 1..indices.len() {
                if let (Some(a), Some(b)) = (s_coords.get(k - 1).copied().flatten(), s_coords.get(k).copied().flatten()) {
                    let seg = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
                    let prev = cum.last().copied().unwrap_or(0.0);
                    cum.push(prev + seg);
                }
            }
            let total = cum.last().copied().unwrap_or(0.0);
            if total > MIN_VEC_NORM && cum.len() == indices.len() {
                for (pos, &dom_idx) in indices.iter().enumerate() {
                    if let (Some(d), Some(&dist)) = (domains.get_mut(dom_idx), cum.get(pos))
                        && d.s_position.is_none()
                    {
                        d.s_position = Some(dist / total);
                    }
                }
            }
        }

        for (pos, &dom_idx) in indices.iter().enumerate() {
            let Some(d) = domains.get_mut(dom_idx) else {
                continue;
            };
            d.normal = normalize_vec3(d.normal);

            if !vec3_is_nonzero(d.tangent) {
                let mut raw_t = [0.0; 3];
                if let Some(sn) = station_nodes.get(dom_idx)
                    && sn.len() >= 2
                    && let (Some(first_id), Some(last_id)) = (sn.first(), sn.last())
                {
                    let end_id = if first_id == last_id { sn.get(1).unwrap_or(last_id) } else { last_id };
                    if let (Some(a), Some(b)) = (model.node(*first_id), model.node(*end_id)) {
                        raw_t = [b.xyz[0] - a.xyz[0], b.xyz[1] - a.xyz[1], b.xyz[2] - a.xyz[2]];
                    }
                }
                if !vec3_is_nonzero(raw_t) && indices.len() >= 2 {
                    let prev_pos = pos.saturating_sub(1);
                    let next_pos = (pos + 1).min(indices.len() - 1);
                    if let (Some(a), Some(b)) = (s_coords.get(prev_pos).copied().flatten(), s_coords.get(next_pos).copied().flatten()) {
                        raw_t = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                    }
                }
                if vec3_is_nonzero(raw_t) {
                    if vec3_is_nonzero(d.normal) {
                        let dot = raw_t[0] * d.normal[0] + raw_t[1] * d.normal[1] + raw_t[2] * d.normal[2];
                        let proj = [raw_t[0] - dot * d.normal[0], raw_t[1] - dot * d.normal[1], raw_t[2] - dot * d.normal[2]];
                        d.tangent = if vec3_is_nonzero(proj) { normalize_vec3(proj) } else { normalize_vec3(raw_t) };
                    } else {
                        d.tangent = normalize_vec3(raw_t);
                    }
                }
            } else {
                d.tangent = normalize_vec3(d.tangent);
            }

            if !vec3_is_nonzero(d.advance) && vec3_is_nonzero(d.normal) && vec3_is_nonzero(d.tangent) {
                d.advance = cross_unit(d.normal, d.tangent);
            } else {
                d.advance = normalize_vec3(d.advance);
            }
        }
    }
}

/// Read WARP3D flat-file results and domain integrals from `dir_or_file`, converting outputs
/// from `units` (or auto-detected `! fieldcraft-units:` header, defaulting to `Preset::MKgS`)
/// back into SI units.
pub fn read_results_dir(model: &fieldcraft_model::Model, step: &str, dir_or_file: &Path, units: Option<Preset>) -> Result<Warp3dSolution> {
    let (dir, explicit_file) = resolve_dir(dir_or_file)?;
    let mut warnings = Vec::new();
    let (stdout_log, stdout_truncated) = read_optional_stdout(&dir, explicit_file, &mut warnings);
    read_results_dir_inner(model, step, dir_or_file, units, stdout_log, stdout_truncated, None, MAX_FRAME_VALUES, warnings)
}

/// Internal entry point shared with tests or batch callers when `stdout_log` is already captured.
#[allow(dead_code)]
pub(crate) fn read_results_dir_with_stdout(
    model: &fieldcraft_model::Model,
    step: &str,
    dir_or_file: &Path,
    units: Option<Preset>,
    stdout_log: String,
    warnings: Vec<String>,
) -> Result<Warp3dSolution> {
    read_results_dir_inner(model, step, dir_or_file, units, stdout_log, false, None, MAX_FRAME_VALUES, warnings)
}

/// Internal entry point used by [`crate::runner::run_warp3d`] when stdout domain integrals were
/// already streamed through [`DomainIntegralParser`].
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn read_results_dir_with_preparsed_fracture(
    model: &fieldcraft_model::Model,
    step: &str,
    dir_or_file: &Path,
    units: Option<Preset>,
    stdout_log: String,
    preparsed_fracture: ParsedDomainOutput,
    warnings: Vec<String>,
) -> Result<Warp3dSolution> {
    read_results_dir_inner(model, step, dir_or_file, units, stdout_log, false, Some(preparsed_fracture), MAX_FRAME_VALUES, warnings)
}

#[allow(clippy::too_many_arguments)]
fn read_results_dir_inner(
    model: &fieldcraft_model::Model,
    step: &str,
    dir_or_file: &Path,
    units: Option<Preset>,
    stdout_log: String,
    stdout_truncated: bool,
    preparsed_fracture: Option<ParsedDomainOutput>,
    max_frame_values: usize,
    mut warnings: Vec<String>,
) -> Result<Warp3dSolution> {
    let si_resolved = model.resolve()?;
    for w in &si_resolved.warnings {
        warnings.push(w.to_string());
    }
    let step_opt = (!step.is_empty()).then_some(step);
    let step_ref = select_step(model, step_opt).map_err(Warp3dError::Unsupported)?;

    let (dir, explicit_file) = resolve_dir(dir_or_file)?;
    let preset = detect_preset(&dir, explicit_file, model, units, &mut warnings);
    let display = DisplaySystem::new(preset);

    let label_map = LabelMap::from_model(model);
    let n_nodes = label_map.warp_node_to_fc.len();
    let n_elems = label_map.warp_elem_to_fc.len();
    if n_nodes == 0 || n_elems == 0 {
        return Err(Warp3dError::Parse("model has no nodes or elements to map WARP3D results onto".into()));
    }

    let step_files = collect_step_files(&dir)?;
    if step_files.is_empty() {
        return Err(Warp3dError::Parse(format!("no WARP3D flat text result files (wnd*_text) found in `{}`", dir.display())));
    }

    let (constrained, mpc_nodes, constrained_dofs) = collect_constraints(model, &si_resolved, step_ref, &label_map);
    let step_nums: Vec<usize> = step_files.keys().copied().collect();
    let total_steps = step_nums.len();
    let is_nonlinear = matches!(step_ref.procedure, Procedure::NonlinearStatic(_)) || total_steps > 1;
    let frame_stress = step_ref.output.contains(&OutputVariable::Stress);

    let mut history = History::default();
    let mut frame_values = 0usize;
    let mut frames_full = false;
    let mut field_values = 0usize;
    let mut fields_full = false;

    if is_nonlinear {
        let these = n_nodes.saturating_mul(3);
        frame_values += these;
        let f0_stress = if frame_stress {
            let need = n_nodes.saturating_mul(6);
            if need <= MAX_FIELD_VALUES {
                field_values += need;
                vec![[0.0; 6]; n_nodes]
            } else {
                fields_full = true;
                Vec::new()
            }
        } else {
            Vec::new()
        };
        history.frames.push(Frame {
            load_factor: 0.0,
            displacement: vec![[0.0; 3]; n_nodes],
            reaction: Vec::new(),
            stress: f0_stress,
            contacts: Vec::new(),
        });
    }

    let mut final_disp = Vec::new();
    let mut final_rx = Vec::new();
    let mut final_stress = Vec::new();
    let mut final_vm = Vec::new();
    let mut final_elems = Vec::new();
    let mut final_factor = 1.0f64;
    let mut prev_factor = 0.0f64;
    let mut missing_wns_warned = false;
    let mut missing_wnr_warned = false;
    let mut missing_wes_warned = false;

    for (idx, &step_no) in step_nums.iter().enumerate() {
        let is_last = idx + 1 == total_steps;
        let Some(files) = step_files.get(&step_no) else {
            continue;
        };

        let load_factor = step_load_factor(step_ref, idx, total_steps);
        if !is_nonlinear && !is_last {
            continue;
        }
        if is_nonlinear && frames_full && !is_last {
            history.increments.push(Increment {
                number: idx + 1,
                load_factor,
                size: (load_factor - prev_factor).abs(),
                iterations: Vec::new(),
                converged: true,
                cutback: None,
                negative_pivots: 0,
                augmentations: 0,
            });
            prev_factor = load_factor;
            continue;
        }

        let wnd_path = files
            .wnd
            .as_ref()
            .ok_or_else(|| Warp3dError::Parse(format!("missing nodal displacement file (wnd*) for step {step_no} in `{}`", dir.display())))?;
        let disp = parse_nodal_vector_file(wnd_path, n_nodes, &display, Quantity::Length)?;

        let rx = match &files.wnr {
            Some(wnr_path) => {
                let mut raw = parse_nodal_vector_file(wnr_path, n_nodes, &display, Quantity::Force)?;
                mask_reactions(&mut raw, &constrained, &mpc_nodes);
                raw
            }
            None => {
                if is_last && !missing_wnr_warned {
                    warnings.push(format!("missing nodal reaction file (wnr*) for step {step_no}; reactions omitted"));
                    missing_wnr_warned = true;
                }
                Vec::new()
            }
        };

        let need_stress = is_last || (is_nonlinear && frame_stress && !fields_full);
        let (stress, von_mises) = if need_stress {
            match &files.wns {
                Some(wns_path) => parse_nodal_stress_file(wns_path, n_nodes, &display)?,
                None => {
                    if is_last && !missing_wns_warned {
                        warnings.push(format!("missing nodal stress file (wns*) for step {step_no}; nodal stresses omitted"));
                        missing_wns_warned = true;
                    }
                    (Vec::new(), Vec::new())
                }
            }
        } else {
            (Vec::new(), Vec::new())
        };

        if is_nonlinear {
            history.increments.push(Increment {
                number: idx + 1,
                load_factor,
                size: (load_factor - prev_factor).abs(),
                iterations: Vec::new(),
                converged: true,
                cutback: None,
                negative_pivots: 0,
                augmentations: 0,
            });
            prev_factor = load_factor;

            if !frames_full {
                let sparse_rx: Vec<(usize, [f64; 3])> =
                    rx.iter().enumerate().filter(|(_, f)| f.iter().any(|c| *c != 0.0)).map(|(i, f)| (i, *f)).collect();
                let these = n_nodes.saturating_mul(3).saturating_add(sparse_rx.len().saturating_mul(4));
                if frame_values.saturating_add(these) > max_frame_values {
                    warnings.push(format!("the frames stop at load factor {load_factor}: the step keeps at most {max_frame_values} values"));
                    frames_full = true;
                } else {
                    frame_values += these;
                    let f_stress = if frame_stress && !stress.is_empty() && !fields_full {
                        let need = n_nodes.saturating_mul(6);
                        if field_values.saturating_add(need) > MAX_FIELD_VALUES {
                            warnings.push(format!(
                                "the frames keep their stress fields up to the one before load factor {load_factor}: the step keeps at most {MAX_FIELD_VALUES} field values"
                            ));
                            fields_full = true;
                            Vec::new()
                        } else {
                            field_values += need;
                            stress.clone()
                        }
                    } else {
                        Vec::new()
                    };
                    history.frames.push(Frame {
                        load_factor,
                        displacement: disp.clone(),
                        reaction: sparse_rx,
                        stress: f_stress,
                        contacts: Vec::new(),
                    });
                }
            }
        }

        if is_last {
            final_factor = load_factor;
            final_disp = disp;
            final_rx = rx;
            if let Some(wns_path) = &files.wns {
                if stress.is_empty() {
                    let (s, vm) = parse_nodal_stress_file(wns_path, n_nodes, &display)?;
                    final_stress = s;
                    final_vm = vm;
                } else {
                    final_stress = stress;
                    final_vm = von_mises;
                }
            } else if !missing_wns_warned {
                warnings.push(format!("missing nodal stress file (wns*) for step {step_no}; nodal stresses omitted"));
                missing_wns_warned = true;
            }

            if let Some(wes_path) = &files.wes {
                final_elems = parse_element_stress_file(wes_path, model, &si_resolved, &label_map)?;
            } else if !missing_wes_warned {
                warnings.push(format!("missing element stress file (wes*) for step {step_no}; element stresses omitted"));
                missing_wes_warned = true;
            }
        }
    }

    let mut reaction_total = [0.0f64; 3];
    for r in &final_rx {
        reaction_total[0] += r[0];
        reaction_total[1] += r[1];
        reaction_total[2] += r[2];
    }
    let applied_total =
        compute_applied_total(model, &si_resolved, step_ref, final_factor).unwrap_or([-reaction_total[0], -reaction_total[1], -reaction_total[2]]);

    let parsed_domain = match preparsed_fracture {
        Some(pre) => pre,
        None => parse_domain_integrals_with_meta(&stdout_log, &label_map),
    };
    warnings.extend(parsed_domain.warnings);
    if parsed_domain.truncated {
        warnings.push("WARP3D fracture domain integral output exceeded parser bounds; fracture results may be incomplete".into());
    }

    let step1_is_elastic = if step_nums.as_slice() == [1] && !final_elems.is_empty() {
        Some(final_elems.iter().all(|e| e.plastic_strain.iter().all(|&p| p <= STEP1_ELASTIC_PEEQ_TOL)))
    } else if let Some(wes1_path) = step_files.get(&1).and_then(|f| f.wes.as_ref()) {
        scan_wes_step1_is_elastic(wes1_path, n_elems)
    } else {
        None
    };

    let total_dofs = n_nodes.saturating_mul(3);
    let equations = total_dofs.saturating_sub(constrained_dofs);
    let static_result = StaticResult {
        step: step_ref.name.clone(),
        nodes: label_map.warp_node_to_fc.clone(),
        displacement: final_disp,
        rotation: Vec::new(),
        reaction: final_rx,
        reaction_moment: Vec::new(),
        stress: final_stress,
        von_mises: final_vm,
        elements: final_elems,
        shells: Vec::new(),
        beams: Vec::new(),
        reference_points: Vec::new(),
        applied_total,
        reaction_total,
        stats: Stats { dofs: total_dofs, equations, factor_entries: 0, lanczos_vectors: 0 },
        warnings: warnings.clone(),
        table_ranges: Vec::new(),
        temperature: None,
        nonlinear: is_nonlinear.then_some(history),
        contacts: Vec::new(),
    };

    let mut fracture = parsed_domain.domains;
    let domains_truncated = stdout_truncated || parsed_domain.truncated;
    convert_domain_integrals_to_si(&mut fracture, &display)?;
    enrich_domain_geometry(&mut fracture, &parsed_domain.station_nodes, model);
    Ok(Warp3dSolution { static_result, fracture, step_numbers: step_nums, domains_truncated, step1_is_elastic, warnings, stdout_log })
}

fn resolve_dir(dir_or_file: &Path) -> Result<(PathBuf, Option<&Path>)> {
    if !dir_or_file.exists() {
        return Err(Warp3dError::Parse(format!("WARP3D results path `{}` does not exist", dir_or_file.display())));
    }
    if dir_or_file.is_dir() {
        return Ok((dir_or_file.to_path_buf(), None));
    }
    let dir = dir_or_file.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or_else(|| Path::new("."));
    Ok((dir.to_path_buf(), Some(dir_or_file)))
}

fn scan_inp_units(path: &Path) -> Option<Preset> {
    let file = File::open(path).ok()?;
    let reader = BufReader::new(file);
    for line in reader.lines().take(MAX_INP_HEADER_LINES) {
        let Ok(line) = line else {
            break;
        };
        let trimmed = line.trim();
        if let Some(rest) = trimmed.strip_prefix("! fieldcraft-units:")
            && let Some(preset) = Preset::from_label(rest.trim())
        {
            return Some(preset);
        }
    }
    None
}

fn detect_preset(
    dir: &Path,
    explicit_file: Option<&Path>,
    model: &fieldcraft_model::Model,
    explicit_units: Option<Preset>,
    warnings: &mut Vec<String>,
) -> Preset {
    if let Some(p) = explicit_units {
        return p;
    }
    if let Some(inp) = explicit_file.filter(|p| p.extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case("inp")))
        && let Some(p) = scan_inp_units(inp)
    {
        return p;
    }
    if let Ok(entries) = std::fs::read_dir(dir) {
        let mut inps: Vec<PathBuf> = entries
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.is_file() && p.extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case("inp")))
            .collect();
        inps.sort();
        let mut found: Vec<Preset> = Vec::new();
        for inp in inps {
            if let Some(p) = scan_inp_units(&inp) {
                found.push(p);
            }
        }
        if let Some(&first) = found.first() {
            let distinct: BTreeSet<&str> = found.iter().map(|p| p.label()).collect();
            if distinct.len() > 1 {
                warnings.push(format!(
                    "multiple .inp files in `{}` specify different unit presets ({distinct:?}); using `{}`",
                    dir.display(),
                    first.label()
                ));
            }
            return first;
        }
    }
    let fallback = model.display_units().map_or(Preset::MKgS, |d| d.preset());
    warnings.push(format!("no `! fieldcraft-units:` header found in `{}`; assuming `{}` units", dir.display(), fallback.label()));
    fallback
}

fn read_capped_stdout(path: &Path, max_bytes: u64, warnings: &mut Vec<String>) -> Option<(String, bool)> {
    use std::io::Read;
    let meta = std::fs::metadata(path).ok()?;
    if !meta.is_file() {
        return None;
    }
    let file = File::open(path).ok()?;
    let cap_usize = usize::try_from(max_bytes).unwrap_or(usize::MAX);
    let mut buf = Vec::with_capacity(meta.len().min(max_bytes).min(1024 * 1024) as usize);
    file.take(max_bytes.saturating_add(1)).read_to_end(&mut buf).ok()?;
    let truncated = meta.len() > max_bytes || buf.len() > cap_usize;
    if truncated {
        buf.truncate(cap_usize);
        warnings.push(format!(
            "WARP3D output file `{}` ({} bytes) exceeds {max_bytes}-byte limit; streamed first {max_bytes} bytes and fracture domain integrals may be incomplete",
            path.display(),
            meta.len()
        ));
    }
    let mut text = String::from_utf8_lossy(&buf).into_owned();
    if truncated && let Some(nl) = text.rfind('\n') {
        text.truncate(nl + 1);
    }
    Some((text, truncated))
}

fn read_optional_stdout(dir: &Path, explicit_file: Option<&Path>, warnings: &mut Vec<String>) -> (String, bool) {
    read_optional_stdout_with_cap(dir, explicit_file, MAX_STDOUT_BYTES, warnings)
}

fn read_optional_stdout_with_cap(dir: &Path, explicit_file: Option<&Path>, max_bytes: u64, warnings: &mut Vec<String>) -> (String, bool) {
    if let Some(file) = explicit_file
        && file.extension().and_then(|e| e.to_str()).is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "out" | "log" | "txt"))
        && let Some(res) = read_capped_stdout(file, max_bytes, warnings)
    {
        return res;
    }
    let Ok(entries) = std::fs::read_dir(dir) else {
        return (String::new(), false);
    };
    let mut logs: Vec<PathBuf> = entries
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.is_file() && p.extension().and_then(|e| e.to_str()).is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "out" | "log")))
        .collect();
    logs.sort();
    for log in logs {
        if let Some(res) = read_capped_stdout(&log, max_bytes, warnings) {
            return res;
        }
    }
    (String::new(), false)
}

fn parse_flat_filename(name: &str) -> Option<(&'static str, usize)> {
    let stem = name.strip_suffix("_text")?;
    let (prefix, digits) = if let Some(rest) = stem.strip_prefix("wnd") {
        ("wnd", rest)
    } else if let Some(rest) = stem.strip_prefix("wns") {
        ("wns", rest)
    } else if let Some(rest) = stem.strip_prefix("wes") {
        ("wes", rest)
    } else {
        ("wnr", stem.strip_prefix("wnr")?)
    };
    if !(1..=10).contains(&digits.len()) || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let step: usize = digits.parse().ok()?;
    (step > 0).then_some((prefix, step))
}

fn collect_step_files(dir: &Path) -> Result<BTreeMap<usize, StepFiles>> {
    collect_step_files_with_cap(dir, MAX_STEPS)
}

fn collect_step_files_with_cap(dir: &Path, max_steps: usize) -> Result<BTreeMap<usize, StepFiles>> {
    let mut entries: Vec<PathBuf> = std::fs::read_dir(dir)?.filter_map(|e| e.ok().map(|e| e.path())).filter(|p| p.is_file()).collect();
    entries.sort();

    let mut steps: BTreeMap<usize, StepFiles> = BTreeMap::new();
    for path in entries {
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
            continue;
        };
        let Some((kind, step)) = parse_flat_filename(name) else {
            continue;
        };
        if !steps.contains_key(&step) && steps.len() >= max_steps {
            return Err(Warp3dError::Parse(format!("WARP3D results directory `{}` exceeds maximum step limit ({max_steps})", dir.display())));
        }
        let entry = steps.entry(step).or_default();
        match kind {
            "wnd" => entry.wnd = Some(path),
            "wns" => entry.wns = Some(path),
            "wes" => entry.wes = Some(path),
            "wnr" => entry.wnr = Some(path),
            _ => {}
        }
    }
    Ok(steps)
}

#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn has_any_flat_files(dir: &Path) -> bool {
    collect_step_files(dir).is_ok_and(|m| !m.is_empty())
}

fn parse_flat_table(path: &Path, expected_rows: usize, expected_cols: usize) -> Result<Vec<Vec<f64>>> {
    parse_flat_table_with_caps(path, expected_rows, expected_cols, MAX_FLAT_FILE_BYTES, MAX_FLAT_LINE_BYTES, MAX_SKIPPED_LINES)
}

fn parse_flat_table_with_caps(
    path: &Path,
    expected_rows: usize,
    expected_cols: usize,
    max_file_bytes: u64,
    max_line_bytes: usize,
    max_skipped_lines: usize,
) -> Result<Vec<Vec<f64>>> {
    let meta = std::fs::metadata(path)?;
    if meta.len() > max_file_bytes {
        return Err(Warp3dError::Parse(format!(
            "`{}` ({} bytes) exceeds maximum flat file size ({max_file_bytes} bytes)",
            path.display(),
            meta.len()
        )));
    }
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    let mut rows = Vec::with_capacity(expected_rows);
    let mut scratch = Vec::new();
    let mut skipped_lines = 0usize;
    let mut line_idx = 0usize;

    while let Some((line, truncated)) = read_bounded_line(&mut reader, &mut scratch, max_line_bytes)? {
        line_idx += 1;
        if truncated {
            return Err(Warp3dError::Parse(format!("`{}` line {line_idx} exceeds maximum line length ({max_line_bytes} bytes)", path.display())));
        }
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            skipped_lines += 1;
            if skipped_lines > max_skipped_lines {
                return Err(Warp3dError::Parse(format!("`{}` exceeds maximum skipped comment/blank lines ({max_skipped_lines})", path.display())));
            }
            continue;
        }
        if rows.len() >= expected_rows {
            return Err(Warp3dError::Parse(format!("`{}` has more than expected {expected_rows} data rows (line {line_idx})", path.display())));
        }
        let mut cols = Vec::with_capacity(expected_cols);
        for tok in trimmed.split_whitespace() {
            if cols.len() >= expected_cols {
                return Err(Warp3dError::Parse(format!("`{}` line {line_idx} has more than expected {expected_cols} columns", path.display())));
            }
            let val: f64 =
                tok.parse().map_err(|_| Warp3dError::Parse(format!("`{}` line {line_idx} contains non-numeric token `{tok}`", path.display())))?;
            if !val.is_finite() {
                return Err(Warp3dError::Parse(format!("`{}` line {line_idx} contains non-finite value `{tok}`", path.display())));
            }
            cols.push(val);
        }
        if cols.len() != expected_cols {
            return Err(Warp3dError::Parse(format!("`{}` line {line_idx} has {} columns (expected {expected_cols})", path.display(), cols.len())));
        }
        rows.push(cols);
    }

    if rows.len() != expected_rows {
        return Err(Warp3dError::Parse(format!("`{}` has {} data rows (expected {expected_rows})", path.display(), rows.len())));
    }
    Ok(rows)
}

fn to_si(display: &DisplaySystem, q: Quantity, val: f64, path: &Path) -> Result<f64> {
    display.to_si(q, val).map_err(|e| Warp3dError::Parse(format!("`{}` unit conversion error: {e}", path.display())))
}

fn parse_nodal_vector_file(path: &Path, n_nodes: usize, display: &DisplaySystem, q: Quantity) -> Result<Vec<[f64; 3]>> {
    let rows = parse_flat_table(path, n_nodes, 3)?;
    let mut out = Vec::with_capacity(n_nodes);
    for row in &rows {
        let x = to_si(display, q, row.first().copied().unwrap_or(0.0), path)?;
        let y = to_si(display, q, row.get(1).copied().unwrap_or(0.0), path)?;
        let z = to_si(display, q, row.get(2).copied().unwrap_or(0.0), path)?;
        out.push([x, y, z]);
    }
    Ok(out)
}

fn reorder_voigt_stress(row: &[f64], display: &DisplaySystem, path: &Path) -> Result<[f64; 6]> {
    // WARP3D 26-column flat stress layout:
    // 0: sxx, 1: syy, 2: szz, 3: sxy, 4: syz, 5: sxz, 6: U0, 7: vm, 8: epspls (c1).
    // FieldCraft stress order is [xx, yy, zz, xy, xz, yz] (swap 4 <-> 5).
    const WARP_TO_FC: [usize; 6] = [0, 1, 2, 3, 5, 4];
    let mut stress = [0.0; 6];
    for (dst, &src) in stress.iter_mut().zip(&WARP_TO_FC) {
        let raw = row.get(src).copied().ok_or_else(|| Warp3dError::Parse(format!("`{}` missing stress column {src}", path.display())))?;
        *dst = to_si(display, Quantity::Pressure, raw, path)?;
    }
    Ok(stress)
}

fn parse_nodal_stress_file(path: &Path, n_nodes: usize, display: &DisplaySystem) -> Result<(Vec<[f64; 6]>, Vec<f64>)> {
    let rows = parse_flat_table(path, n_nodes, 26)?;
    let mut stresses = Vec::with_capacity(n_nodes);
    let mut mises = Vec::with_capacity(n_nodes);
    for row in &rows {
        stresses.push(reorder_voigt_stress(row, display, path)?);
        let raw_vm = row.get(7).copied().ok_or_else(|| Warp3dError::Parse(format!("`{}` missing von Mises column 7", path.display())))?;
        mises.push(to_si(display, Quantity::Pressure, raw_vm, path)?);
    }
    Ok((stresses, mises))
}

fn parse_element_stress_file(path: &Path, model: &fieldcraft_model::Model, resolved: &Resolved, label_map: &LabelMap) -> Result<Vec<ElementStress>> {
    let n_elems = label_map.warp_elem_to_fc.len();
    let rows = parse_flat_table(path, n_elems, 26)?;

    let mut elem_has_plastic: BTreeMap<u32, bool> = BTreeMap::new();
    for sec in model.sections().values() {
        let is_plastic = model.material(&sec.material).is_some_and(|m| m.plastic.is_some());
        if let Some(rset) = resolved.sets.get(&sec.set) {
            for &eid in &rset.elements {
                elem_has_plastic.entry(eid).or_insert(is_plastic);
            }
        }
    }

    let mut elements = Vec::with_capacity(model.elements().len());
    for (&fc_label, elem) in model.elements() {
        let w_id = label_map
            .fc_elem_to_warp
            .get(&fc_label)
            .copied()
            .ok_or_else(|| Warp3dError::Parse(format!("element {fc_label} missing from LabelMap")))?;
        let row_idx = usize::try_from(w_id)
            .ok()
            .and_then(|id| id.checked_sub(1))
            .ok_or_else(|| Warp3dError::Parse(format!("invalid WARP3D element index {w_id} for element {fc_label}")))?;
        let row = rows.get(row_idx).ok_or_else(|| Warp3dError::Parse(format!("`{}` missing row for element {fc_label}", path.display())))?;
        let epspls = row.get(8).copied().ok_or_else(|| Warp3dError::Parse(format!("`{}` missing plastic strain column 8", path.display())))?;
        let is_plastic = elem_has_plastic.get(&fc_label).copied().unwrap_or(false) || epspls != 0.0;
        let plastic_strain = if is_plastic { vec![epspls] } else { Vec::new() };
        elements.push(ElementStress { label: fc_label, kind: elem.kind, points: Vec::new(), nodal: Vec::new(), plastic_strain });
    }
    Ok(elements)
}

/// Stream column 8 (`epspls` / `peeq`) from an Increment-1 `wes*` flat file without allocating
/// a full element table.
///
/// Returns `Some(true)` when all `expected_rows` parse cleanly with `peeq <= STEP1_ELASTIC_PEEQ_TOL`,
/// `Some(false)` when any element yields (`peeq > STEP1_ELASTIC_PEEQ_TOL`), and `None` when the
/// file is unreadable, empty, truncated, or malformed.
fn scan_wes_step1_is_elastic(path: &Path, expected_rows: usize) -> Option<bool> {
    if expected_rows == 0 {
        return None;
    }
    let meta = std::fs::metadata(path).ok()?;
    if meta.len() > MAX_FLAT_FILE_BYTES {
        return None;
    }
    let file = File::open(path).ok()?;
    let mut reader = BufReader::new(file);
    let mut scratch = Vec::new();
    let mut skipped_lines = 0usize;
    let mut rows = 0usize;
    let mut saw_plastic = false;

    loop {
        let Ok(opt_line) = read_bounded_line(&mut reader, &mut scratch, MAX_FLAT_LINE_BYTES) else {
            return None;
        };
        let Some((line, truncated)) = opt_line else {
            break;
        };
        if truncated {
            return None;
        }
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            skipped_lines += 1;
            if skipped_lines > MAX_SKIPPED_LINES {
                return None;
            }
            continue;
        }
        if rows >= expected_rows {
            return None;
        }
        let mut cols = 0usize;
        let mut epspls_opt = None;
        for tok in trimmed.split_whitespace() {
            if cols >= 26 {
                return None;
            }
            let Ok(val) = tok.parse::<f64>() else {
                return None;
            };
            if !val.is_finite() {
                return None;
            }
            if cols == 8 {
                epspls_opt = Some(val);
            }
            cols += 1;
        }
        if cols != 26 {
            return None;
        }
        let epspls = epspls_opt?;
        if epspls > STEP1_ELASTIC_PEEQ_TOL {
            saw_plastic = true;
        }
        rows += 1;
    }

    (rows == expected_rows).then_some(!saw_plastic)
}

fn collect_constraints(
    model: &fieldcraft_model::Model,
    resolved: &Resolved,
    step: &Step,
    label_map: &LabelMap,
) -> (Vec<[bool; 3]>, Vec<bool>, usize) {
    let n_nodes = label_map.warp_node_to_fc.len();
    let mut constrained = vec![[false; 3]; n_nodes];
    let mut mpc_nodes = vec![false; n_nodes];

    for bc_name in &step.boundary_conditions {
        let Some(bc) = model.boundary_condition(bc_name) else {
            continue;
        };
        match bc {
            BoundaryCondition::Fixed { target: Target::Set(set_name) } => {
                if let Some(rset) = resolved.sets.get(set_name) {
                    for fc_node in &rset.nodes {
                        if let Some(&wn) = label_map.fc_node_to_warp.get(fc_node)
                            && let Some(slot) = usize::try_from(wn).ok().and_then(|id| id.checked_sub(1)).and_then(|i| constrained.get_mut(i))
                        {
                            *slot = [true, true, true];
                        }
                    }
                }
            }
            BoundaryCondition::Displacement { target: Target::Set(set_name), values } => {
                if let Some(rset) = resolved.sets.get(set_name) {
                    for fc_node in &rset.nodes {
                        if let Some(&wn) = label_map.fc_node_to_warp.get(fc_node)
                            && let Some(slot) = usize::try_from(wn).ok().and_then(|id| id.checked_sub(1)).and_then(|i| constrained.get_mut(i))
                        {
                            for (dof, val_opt) in values.iter().take(3).enumerate() {
                                if val_opt.is_some()
                                    && let Some(flag) = slot.get_mut(dof)
                                {
                                    *flag = true;
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
                                && let Some(flag) = usize::try_from(wn)
                                    .ok()
                                    .and_then(|id| id.checked_sub(1))
                                    .and_then(|i| constrained.get_mut(i))
                                    .and_then(|slot| slot.get_mut(axis))
                            {
                                *flag = true;
                            }
                        }
                    } else {
                        for fc_node in &rset.nodes {
                            if let Some(&wn) = label_map.fc_node_to_warp.get(fc_node)
                                && let Some(flag) = usize::try_from(wn).ok().and_then(|id| id.checked_sub(1)).and_then(|i| mpc_nodes.get_mut(i))
                            {
                                *flag = true;
                            }
                        }
                    }
                }
            }
            _ => {}
        }
    }

    let mut count = 0usize;
    for (c, &mpc) in constrained.iter().zip(&mpc_nodes) {
        let fixed = c.iter().filter(|&&b| b).count();
        count += fixed;
        if mpc && fixed < 3 {
            count += 1;
        }
    }
    (constrained, mpc_nodes, count)
}

fn mask_reactions(reactions: &mut [[f64; 3]], constrained: &[[bool; 3]], mpc_nodes: &[bool]) {
    for ((rx, c), &mpc) in reactions.iter_mut().zip(constrained).zip(mpc_nodes) {
        if mpc {
            continue;
        }
        for (val, &is_fixed) in rx.iter_mut().zip(c) {
            if !is_fixed {
                *val = 0.0;
            }
        }
    }
}

fn step_load_factor(step: &Step, idx: usize, total_steps: usize) -> f64 {
    let t = if total_steps == 0 { 1.0 } else { (idx + 1) as f64 / total_steps as f64 };
    match &step.procedure {
        Procedure::NonlinearStatic(nl) => match &nl.load_path {
            Some(pts) => piecewise_load_factor(pts, t),
            None => t,
        },
        _ => t,
    }
}

fn compute_applied_total(model: &fieldcraft_model::Model, resolved: &Resolved, step: &Step, final_factor: f64) -> Option<[f64; 3]> {
    let mesh = fieldcraft_solver::system::Mesh::new(model, resolved, usize::MAX).ok()?;
    if mesh.dofs() != 3 * mesh.labels.len() {
        return None;
    }
    let active = vec![true; mesh.dofs()];
    let f = fieldcraft_solver::system::loads(model, resolved, &mesh, step, &active).ok()?;
    let mut total = [0.0f64; 3];
    for chunk in f.as_chunks::<3>().0 {
        total[0] += chunk[0];
        total[1] += chunk[1];
        total[2] += chunk[2];
    }
    for c in &mut total {
        *c *= final_factor;
    }
    Some(total)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use std::fs;
    use std::sync::atomic::{AtomicU64, Ordering};

    use fieldcraft_model::{
        BoundaryCondition, Elastic, ElementKind, Hardening, Load, Material, Model, NonlinearStatic, Plastic, Procedure, Section, SectionKind, Set,
        Step, Target,
    };

    use super::*;

    fn temp_dir(tag: &str) -> PathBuf {
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let dir =
            std::env::temp_dir().join(format!("fieldcraft-warp3d-results-{tag}-{}-{}", std::process::id(), SEQ.fetch_add(1, Ordering::Relaxed)));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn format_26col_row(sxx: f64, syy: f64, szz: f64, sxy: f64, syz: f64, sxz: f64, vm: f64, epspls: f64) -> String {
        let mut cols = [0.0f64; 26];
        cols[0] = sxx;
        cols[1] = syy;
        cols[2] = szz;
        cols[3] = sxy;
        cols[4] = syz;
        cols[5] = sxz;
        cols[7] = vm;
        cols[8] = epspls;
        cols.iter().map(|v| format!("{v:.8e}")).collect::<Vec<_>>().join(" ")
    }

    #[test]
    fn parses_26col_flat_files_reorders_shear_reindexes_elems_and_converts_units() {
        let dir = temp_dir("26col");
        // Model in SI with sparse node/element labels and two sections so LabelMap reorders elements:
        // Elem 30 (Tet4, sec_t) -> WARP elem 2; Elem 10 (Hex8, sec_a) -> WARP elem 1.
        let mut model = Model::default();
        let coords = [
            (2, [0.0, 0.0, 0.0]),
            (5, [0.01, 0.0, 0.0]),
            (8, [0.01, 0.01, 0.0]),
            (11, [0.0, 0.01, 0.0]),
            (14, [0.0, 0.0, 0.01]),
            (17, [0.01, 0.0, 0.01]),
            (20, [0.01, 0.01, 0.01]),
            (23, [0.0, 0.01, 0.01]),
            (30, [0.02, 0.0, 0.0]),
        ];
        for (id, xyz) in coords {
            model.set_node(id, xyz).unwrap();
        }
        // Insert elem 30 (Tet4) before elem 10 (Hex8) in FieldCraft BTreeMap:
        // Model order: [10, 30]. Put 10 in Tet4? Let's make 10 = Tet4 and 30 = Hex8!
        // Then FieldCraft order is [10 (Tet4), 30 (Hex8)], while LabelMap sorts Hex8 before Tet4 -> WARP order [30, 10]!
        model.set_element(10, ElementKind::Tet4, vec![5, 30, 8, 17]).unwrap();
        model.set_element(30, ElementKind::Hex8, vec![2, 5, 8, 11, 14, 17, 20, 23]).unwrap();

        model
            .add_material(
                "Steel",
                Material {
                    elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }),
                    plastic: Some(Plastic::Bilinear { yield_stress: 2.5e8, hardening_modulus: 1.0e9, hardening: Hardening::Isotropic }),
                    ..Default::default()
                },
            )
            .unwrap();
        model.add_set("all_elems", Set { elements: vec![10, 30], ..Set::default() }).unwrap();
        model.add_set("fix", Set { nodes: vec![2, 11, 14, 23], ..Set::default() }).unwrap();
        model.add_set("tip", Set { nodes: vec![30], ..Set::default() }).unwrap();
        model
            .add_section(
                "solid",
                Section { set: "all_elems".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("fix".into()) }).unwrap();
        model
            .add_load("pull", Load::Force { target: Target::Set("tip".into()), vector: [1000.0, 0.0, 0.0], distribution: Default::default() })
            .unwrap();
        let mut step = Step::new(
            "NL",
            Procedure::NonlinearStatic(NonlinearStatic {
                initial_increment: 0.5,
                max_increments: 10,
                load_path: Some(vec![[0.0, 0.0], [0.5, 0.4], [1.0, 1.0]]),
                ..NonlinearStatic::default()
            }),
        );
        step.boundary_conditions.insert("clamp".into());
        step.loads.insert("pull".into());
        model.add_step(step, None).unwrap();

        let map = LabelMap::from_model(&model);
        assert_eq!(map.warp_elem_to_fc, vec![30, 10]); // WARP elem 1 = FC 30 (Hex8), WARP elem 2 = FC 10 (Tet4).

        // Write `.inp` header in mm-t-s-N-MPa so preset auto-detection picks Preset::MmTonneS.
        let inp_path = dir.join("job.inp");
        fs::write(&inp_path, "! fieldcraft-units: mm-t-s-N-MPa\n").unwrap();

        // Step 1 uses 5-digit filenames; Step 2 uses 7-digit filenames, with `#` comment headers.
        // Prescribe units = mm-t-s-N-MPa: displacement in mm (1.5 mm -> 0.0015 m), stress in MPa (100 MPa -> 1e8 Pa), force in N.
        let write_step = |step_str: &str, u_scale: f64, s_scale: f64, r_scale: f64, eps_hex: f64, eps_tet: f64| {
            let mut wnd = String::from("#\n#  WARP3D nodal results: displacements\n#\n");
            let mut wnr = String::from("#\n#  WARP3D nodal results: reactions\n#\n");
            let mut wns = String::from("#\n#  WARP3D nodal results: stresses\n#\n");
            for i in 0..9 {
                let d = (i + 1) as f64 * u_scale;
                wnd.push_str(&format!("{d:.6e} {:.6e} {:.6e}\n", 0.5 * d, -0.25 * d));
                // Include non-zero unconstrained DOF residuals (999.0) on free nodes to verify masking!
                let rx = if [0, 3, 4, 7].contains(&i) { -250.0 * r_scale } else { 999.0 };
                wnr.push_str(&format!("{rx:.6e} 123.0 456.0\n"));
                // WARP3D order: sxx, syy, szz, sxy, syz, sxz -> expect FC [sxx, syy, szz, sxy, sxz, syz].
                wns.push_str(&format_26col_row(
                    100.0 * s_scale,
                    10.0 * s_scale,
                    20.0 * s_scale,
                    30.0 * s_scale,
                    40.0 * s_scale, // syz (WARP col 4)
                    50.0 * s_scale, // sxz (WARP col 5)
                    95.0 * s_scale, // vm (col 7)
                    0.0,
                ));
                wns.push('\n');
            }
            let mut wes = String::from("#\n#  WARP3D element results: stresses\n#\n");
            // Row 0 = WARP elem 1 (FC 30 Hex8); Row 1 = WARP elem 2 (FC 10 Tet4).
            wes.push_str(&format_26col_row(100.0 * s_scale, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0 * s_scale, eps_hex));
            wes.push('\n');
            wes.push_str(&format_26col_row(50.0 * s_scale, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0 * s_scale, eps_tet));
            wes.push('\n');

            fs::write(dir.join(format!("wnd{step_str}_text")), wnd).unwrap();
            fs::write(dir.join(format!("wnr{step_str}_text")), wnr).unwrap();
            fs::write(dir.join(format!("wns{step_str}_text")), wns).unwrap();
            fs::write(dir.join(format!("wes{step_str}_text")), wes).unwrap();
        };

        write_step("00001", 0.4, 0.4, 0.4, 0.001, 0.002);
        write_step("0000002", 1.0, 1.0, 1.0, 0.012, 0.034);

        let sol = read_results_dir(&model, "NL", &inp_path, None).unwrap();
        let r = &sol.static_result;

        assert_eq!(r.step, "NL");
        assert_eq!(r.nodes, vec![2, 5, 8, 11, 14, 17, 20, 23, 30]);
        assert!(r.nodes.windows(2).all(|w| w[0] < w[1]));
        // Displacements converted from mm to m.
        assert!((r.displacement[0][0] - 0.001).abs() < 1e-12);
        assert!((r.displacement[8][0] - 0.009).abs() < 1e-12);

        // Constrained nodes [0, 3, 4, 7] (FC 2, 11, 14, 23) keep reaction; free nodes (e.g. idx 1 = FC 5, idx 8 = FC 30) are masked to 0!
        assert!((r.reaction[0][0] - (-250.0)).abs() < 1e-12);
        assert_eq!(r.reaction[1], [0.0, 0.0, 0.0]);
        assert_eq!(r.reaction[8], [0.0, 0.0, 0.0]);
        assert!((r.reaction_total[0] - (-1000.0)).abs() < 1e-12);
        assert!((r.applied_total[0] - 1000.0).abs() < 1e-9);
        assert_eq!(r.stats.dofs, 27);
        assert_eq!(r.stats.equations, 15); // 27 - 4 fixed nodes * 3

        // Voigt swap 4 <-> 5 and MPa -> Pa (1e6): [sxx, syy, szz, sxy, sxz, syz].
        assert!((r.stress[0][0] - 100.0e6).abs() < 1e-3);
        assert!((r.stress[0][3] - 30.0e6).abs() < 1e-3);
        assert!((r.stress[0][4] - 50.0e6).abs() < 1e-3); // sxz from col 5
        assert!((r.stress[0][5] - 40.0e6).abs() < 1e-3); // syz from col 4
        assert!((r.von_mises[0] - 95.0e6).abs() < 1e-3);

        // Elements sorted ascending by FieldCraft label [10, 30]:
        // FC 10 was WARP elem 2 (eps_tet = 0.034); FC 30 was WARP elem 1 (eps_hex = 0.012).
        assert_eq!(r.elements.len(), 2);
        assert_eq!(r.elements[0].label, 10);
        assert_eq!(r.elements[0].plastic_strain, vec![0.034]);
        assert_eq!(r.elements[1].label, 30);
        assert_eq!(r.elements[1].plastic_strain, vec![0.012]);

        // Nonlinear History: Frame 0 synthesized + Frame 1 (load_factor 0.4) + Frame 2 (load_factor 1.0).
        let h = r.nonlinear.as_ref().unwrap();
        assert_eq!(h.frames.len(), 3);
        assert_eq!(h.increments.len(), 2);
        assert_eq!(h.frames[0].load_factor, 0.0);
        assert!(h.frames[0].displacement.iter().all(|u| *u == [0.0; 3]));
        assert!(h.frames[0].reaction.is_empty());
        assert_eq!(h.frames[0].stress.len(), 9);
        assert!((h.frames[1].load_factor - 0.4).abs() < 1e-12);
        assert!((h.frames[2].load_factor - 1.0).abs() < 1e-12);
        assert_eq!(h.frames[2].displacement, r.displacement);
        assert_eq!(h.frames[2].stress, r.stress);
        for f in &h.frames {
            assert!(f.reaction.windows(2).all(|w| w[0].0 < w[1].0));
            assert!(f.reaction.iter().all(|(i, _)| *i < r.nodes.len()));
        }

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn refuses_malformed_non_finite_and_wrong_row_counts() {
        let dir = temp_dir("bad");
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
        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_step(Step::new("Load", Procedure::Static {}), None).unwrap();

        // Missing directory / files.
        assert!(matches!(read_results_dir(&model, "Load", &dir.join("missing"), None), Err(Warp3dError::Parse(_))));
        assert!(matches!(read_results_dir(&model, "Load", &dir, Some(Preset::MKgS)), Err(Warp3dError::Parse(_))));

        // Truncated wnd (only 3 rows instead of 8).
        fs::write(dir.join("wnd0000001_text"), "0 0 0\n0 0 0\n0 0 0\n").unwrap();
        assert!(matches!(read_results_dir(&model, "Load", &dir, Some(Preset::MKgS)), Err(Warp3dError::Parse(_))));

        // NaN value in wnd.
        fs::write(dir.join("wnd0000001_text"), "0 0 0\n".repeat(7) + "NaN 0 0\n").unwrap();
        assert!(matches!(read_results_dir(&model, "Load", &dir, Some(Preset::MKgS)), Err(Warp3dError::Parse(_))));

        // Wrong column count in wns.
        fs::write(dir.join("wnd0000001_text"), "0 0 0\n".repeat(8)).unwrap();
        fs::write(dir.join("wns0000001_text"), "1 2 3\n".repeat(8)).unwrap();
        assert!(matches!(read_results_dir(&model, "Load", &dir, Some(Preset::MKgS)), Err(Warp3dError::Parse(_))));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn parses_domain_and_interaction_integral_stdout_tables_and_converts_units() {
        let mut model = Model::default();
        // Sparse FieldCraft node labels along a 2-Hex8 bar in Z (0 -> 0.01 -> 0.02 m):
        // Front nodes are FC [2, 14, 26] -> WARP nodes [1, 5, 9].
        let coords = [
            (2, [0.0, 0.0, 0.0]),
            (5, [0.01, 0.0, 0.0]),
            (8, [0.01, 0.01, 0.0]),
            (11, [0.0, 0.01, 0.0]),
            (14, [0.0, 0.0, 0.01]),
            (17, [0.01, 0.0, 0.01]),
            (20, [0.01, 0.01, 0.01]),
            (23, [0.0, 0.01, 0.01]),
            (26, [0.0, 0.0, 0.02]),
            (29, [0.01, 0.0, 0.02]),
            (32, [0.01, 0.01, 0.02]),
            (35, [0.0, 0.01, 0.02]),
        ];
        for (id, xyz) in coords {
            model.set_node(id, xyz).unwrap();
        }
        model.set_element(10, ElementKind::Hex8, vec![2, 5, 8, 11, 14, 17, 20, 23]).unwrap();
        model.set_element(20, ElementKind::Hex8, vec![14, 17, 20, 23, 26, 29, 32, 35]).unwrap();
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 2.1e11, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("all", Set { elements: vec![10, 20], ..Set::default() }).unwrap();
        model.add_set("fix", Set { nodes: vec![2, 11, 14, 23, 26, 35], ..Set::default() }).unwrap();
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

        let label_map = LabelMap::from_model(&model);
        assert_eq!(label_map.fc_node_to_warp.get(&2), Some(&1));
        assert_eq!(label_map.fc_node_to_warp.get(&14), Some(&5));
        assert_eq!(label_map.fc_node_to_warp.get(&26), Some(&9));

        // 1. Real WARP3D 18.5.0 stdout (including `dump`, negative `maximum:-...` formatting,
        // `J-values computed from mixed-mode` decoys, and `Static I-values` KI/KII/KIII/T11/T33).
        let warp_stdout = "\
     Domain definition:  cf_0002

        Number of front nodes:    3 Nodes on front:
              1      5      9
        Direction cosines:   0.000000  1.000000  0.000000
        Tangent vector defined: T
        Tangent vector direction cosines:   0.000000  0.000000  1.000000
        Front order:  1     Verify front: T Domain type:  2
     ** end of domain definition **

                         domain-integral post-processing
                         -------------------------------
 structure: fc_model
 loading: Load     step:     1        model analysis time:  0.100000E+07
 domain id: cf_0002                   current elapsed wall time:      0.1

 area under q along front:        0.500000E+01
 length of crack front segment:   0.100000E+02
 Young's modulus at crack front node        5 :  0.210000E+06
 Poisson's ratio at crack front node        5 :  0.300000E+00

                                             J-integral components
                                          ===========================
 domain        dm1          dm2          dm3          dm4          dm5          dm6          dm7          dm8       total J   killed ele
     1     1.0000E+01   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   1.0000E+01   (  0)
     2     1.2000E+01   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   1.2000E+01   (  0)

    average:  1.1000E+01   minimum:  1.0000E+01   maximum: 1.2000E+01     (cf_0002)
  1.1000E+01  1.0000E+01  1.2000E+01

          Static J-values

    average:  1.1000E+01   minimum:  1.0000E+01   maximum: 1.2000E+01     (cf_0002)
  1.1000E+01  1.0000E+01  1.2000E+01

          Stress intensity factors obtained from J-values assuming pure mode I, mode II or mode III loading
 domain   KI plane stress   KI plane strain   KII plane stress   KII plane strain   KIII anti-plane shear
     1     1.4500E+03       1.5200E+03        1.4500E+03        1.5200E+03          1.2200E+03
     2     1.5500E+03       1.6200E+03        1.5500E+03        1.6200E+03          1.3000E+03

               Static I-values
                              I-integral components
                          -- KI = 1, KII = 0, KIII = 0, plane strain --
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I     KI (pstrn) killed ele
     1     1.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   1.0000E+00   1.5000E+03   (  0)
     2     1.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   1.0000E+00   1.6000E+03   (  0)

    average:  1.5500E+03   minimum:  1.5000E+03   maximum: 1.6000E+03     (cf_0002)
  1.5500E+03  1.5000E+03  1.6000E+03

                          -- KI = 0, KII = 1, KIII = 0, plane strain --
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I     KII (pstrn) killed ele
     1     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00  -2.5000E+01   (  0)
     2     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00  -2.5000E+01   (  0)

    average: -2.5000E+01   minimum: -2.5000E+01   maximum:-2.5000E+01     (cf_0002)
 -2.5000E+01 -2.5000E+01 -2.5000E+01

                          -- KI = 0, KII = 0, KIII = 1 --
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I        KIII    killed ele
     1     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   5.0000E+00   (  0)
     2     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   5.0000E+00   (  0)

    average:  5.0000E+00   minimum:  5.0000E+00   maximum: 5.0000E+00     (cf_0002)
  5.0000E+00  5.0000E+00  5.0000E+00

          J-values computed from mixed-mode stress intensity factors KI, KII and KIII.
 domain   J plane strain
     1     9.9999E+01
     2     9.9999E+01

                          --T-stress (\"plane strain\" auxiliary fields)--
 domain        II1          II2          II3          II4          II5          II6          II7          II8       total I     T11 (pstrn)      T33     killed ele
     1     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00  -1.2000E+01  -3.6000E+00   (  0)
     2     0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00   0.0000E+00  -1.2000E+01  -3.6000E+00   (  0)

    average:     minimum:     maximum:     (cf_0002)
 -1.2000E+01 -1.2000E+01 -1.2000E+01
 -3.6000E+00 -3.6000E+00 -3.6000E+00

>>> Completed domain integral calculations
";
        let parsed = parse_domain_integrals(warp_stdout, &label_map);
        assert_eq!(parsed.len(), 1);
        let d = &parsed[0];
        assert_eq!(d.domain_id, "cf_0002");
        assert_eq!(d.step, 1);
        assert_eq!(d.front_nodes, vec![14]);
        assert_eq!(d.normal, [0.0, 1.0, 0.0]);
        assert_eq!(d.tangent, [0.0, 0.0, 1.0]);
        assert_eq!(d.advance, [1.0, 0.0, 0.0]);
        assert!((d.area_aq - 5.0).abs() < 1e-12);
        assert_eq!(d.rings.len(), 2);
        assert!((d.j_mean - 11.0).abs() < 1e-12);
        assert!((d.j_min - 10.0).abs() < 1e-12);
        assert!((d.j_max - 12.0).abs() < 1e-12);
        assert!((d.ring_spread - (2.0 / 11.0)).abs() < 1e-12);
        assert!((d.k_j_plane_stress.unwrap() - 1500.0).abs() < 1e-9);
        assert!((d.k_j_plane_strain.unwrap() - 1570.0).abs() < 1e-9);
        assert!((d.k_i_mean.unwrap() - 1550.0).abs() < 1e-9);
        assert!((d.k_ii_mean.unwrap() - (-25.0)).abs() < 1e-9);
        assert!((d.k_iii_mean.unwrap() - 5.0).abs() < 1e-9);
        assert!((d.t_11_mean.unwrap() - (-12.0)).abs() < 1e-9);
        assert!((d.t_33_mean.unwrap() - (-3.6)).abs() < 1e-9);

        // 2. Schematic `Domain: <id>` + `(Interaction Integral Evaluation)` + `Preset::MmTonneS` -> SI check.
        let dir = temp_dir("domain-units");
        let inp_path = dir.join("job.inp");
        fs::write(&inp_path, "! fieldcraft-units: mm-t-s-N-MPa\n").unwrap();
        let zero_row = format_26col_row(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        fs::write(dir.join("wnd0000001_text"), "0 0 0\n".repeat(12)).unwrap();
        fs::write(dir.join("wnr0000001_text"), "0 0 0\n".repeat(12)).unwrap();
        fs::write(dir.join("wns0000001_text"), format!("{zero_row}\n").repeat(12)).unwrap();
        fs::write(dir.join("wes0000001_text"), format!("{zero_row}\n").repeat(2)).unwrap();
        let schematic_stdout = "\
Domain: cf_0001
  Step: 1
  Front nodes: 1
  Crack plane normal: 0.0 1.0 0.0
  Area A_q: 10.0
  Ring     J          K_J(pstrn)  K_J(pstrs)
     1   10.0         1500.0      1400.0
     2   12.0         1600.0      1500.0
  J_mean: 11.0  J_min: 10.0  J_max: 12.0  J_total: 11.0

Domain: cf_0001 (Interaction Integral Evaluation)
  Ring     K_I        K_II        K_III       T_11        T_33
     1   1500.0      -10.0        2.0       -12.0       -3.6
     2   1600.0      -10.0        2.0       -12.0       -3.6
  K_I_mean: 1550.0  K_II_mean: -10.0  K_III_mean: 2.0  T_11_mean: -12.0  T_33_mean: -3.6

Domain: cf_0002
  Step: 1
  Front nodes: 5
  Crack plane normal: 0.0 1.0 0.0
  Area A_q: 10.0
  Ring     J
     1   20.0
     2   22.0
";
        fs::write(dir.join("job.out"), schematic_stdout).unwrap();
        let sol = read_results_dir(&model, "Load", &inp_path, None).unwrap();
        assert_eq!(sol.fracture.len(), 2);

        let f0 = &sol.fracture[0];
        assert_eq!(f0.domain_id, "cf_0001");
        assert_eq!(f0.front_nodes, vec![2]);
        assert_eq!(f0.s_position, Some(0.0));
        assert_eq!(f0.normal, [0.0, 1.0, 0.0]);
        assert_eq!(f0.tangent, [0.0, 0.0, 1.0]);
        assert_eq!(f0.advance, [1.0, 0.0, 0.0]);
        assert!((f0.area_aq - 1.0e-5).abs() < 1e-16); // 10 mm^2 -> 1e-5 m^2
        assert!((f0.j_mean - 11.0e3).abs() < 1e-6); // 11 N/mm -> 11e3 N/m
        assert!((f0.rings[0].j - 10.0e3).abs() < 1e-6);
        assert!((f0.ring_spread - (2.0 / 11.0)).abs() < 1e-12);
        let expected_k_scale = 1.0e6 * 1.0e-3_f64.sqrt(); // MPa*sqrt(mm) -> Pa*sqrt(m)
        assert!((f0.k_i_mean.unwrap() - 1550.0 * expected_k_scale).abs() < 1e-3);
        assert!((f0.k_j_plane_strain.unwrap() - 1550.0 * expected_k_scale).abs() < 1e-3);
        assert!((f0.t_11_mean.unwrap() - (-12.0e6)).abs() < 1e-2); // -12 MPa -> -12e6 Pa
        assert!((f0.t_33_mean.unwrap() - (-3.6e6)).abs() < 1e-2);

        let f1 = &sol.fracture[1];
        assert_eq!(f1.front_nodes, vec![14]);
        assert_eq!(f1.s_position, Some(1.0));
        assert!((f1.j_mean - 21.0e3).abs() < 1e-6);

        // 3. Malformed / hostile stdout (NaN, Inf, ***, wrong cols, > MAX_RINGS_PER_DOMAIN) never panics.
        let mut hostile =
            String::from("Domain: bad\n  Ring J\n  1 NaN\n  2 Inf\n  3 *******\n  4 not_a_number\n  J_mean: NaN\nDomain: capped\n  Ring J\n");
        for r in 1..=100 {
            hostile.push_str(&format!("  {r} {}\n", r as f64));
        }
        let parsed_hostile = parse_domain_integrals(&hostile, &label_map);
        assert_eq!(parsed_hostile.len(), 1);
        assert_eq!(parsed_hostile[0].domain_id, "capped");
        assert_eq!(parsed_hostile[0].rings.len(), MAX_RINGS_PER_DOMAIN);

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn streaming_domain_integral_parser_matches_batch_across_chunked_logs() {
        let mut model = Model::default();
        for (id, x, y, z) in [
            (2, 0.0, 0.0, 0.0),
            (3, 1.0, 0.0, 0.0),
            (4, 1.0, 1.0, 0.0),
            (5, 0.0, 1.0, 0.0),
            (6, 0.0, 0.0, 1.0),
            (7, 1.0, 0.0, 1.0),
            (8, 1.0, 1.0, 1.0),
            (9, 0.0, 1.0, 1.0),
        ] {
            model.set_node(id, [x, y, z]).unwrap();
        }
        model.set_element(10, ElementKind::Hex8, vec![2, 3, 4, 5, 6, 7, 8, 9]).unwrap();
        let label_map = LabelMap::from_model(&model);

        let domain_log = "\
loading: test step: 2 lambda = 1.0
Domain definition: d1
  Number of front nodes: 1 Nodes on front:
  1
  Direction cosines: 0.0 1.0 0.0
  Tangent vector direction cosines: 0.0 0.0 1.0
** end of domain definition **
Domain: d1 (step 2)
  Area under q along front: 5.0
  J_mean: 12.0 J_min: 11.0 J_max: 13.0
  Ring     J      K_j_pstrain    K_j_pstress
     1   11.0     1500.0         1450.0
     2   13.0     1600.0         1550.0
Domain: d1 (interaction integral, step 2)
  Ring     K_I      K_II     K_III    T_11     T_33
     1  1500.0     0.0      0.0     -10.0    -3.0
     2  1600.0     0.0      0.0     -11.0    -3.2
";
        let batch = parse_domain_integrals_with_meta(domain_log, &label_map);
        assert!(!batch.truncated);
        let mut streamed = DomainIntegralParser::new(&label_map);
        for line in domain_log.lines() {
            streamed.feed_line(line);
        }
        let stream = streamed.finish();
        assert_eq!(stream.domains, batch.domains);
        assert_eq!(stream.station_nodes, batch.station_nodes);
        assert!(!stream.truncated);

        // Overlong non-domain filler lines beyond MAX_DOMAIN_LINES must not starve late domain output,
        // and multi-byte UTF-8 lines exceeding MAX_LINE_BYTES must clamp without panicking.
        let mut late_parser = DomainIntegralParser::new(&label_map);
        for _ in 0..=MAX_DOMAIN_LINES {
            late_parser.feed_line("equilibrium iteration residual = 1.23e-9");
        }
        let multi_byte_filler = "α".repeat(MAX_LINE_BYTES + 17);
        late_parser.feed_line(&multi_byte_filler);
        for line in domain_log.lines() {
            late_parser.feed_line(line);
        }
        let late = late_parser.finish();
        assert_eq!(late.domains, batch.domains);
        assert!(!late.truncated);
    }

    #[test]
    fn results_enforces_flat_file_bounds_step_caps_skips_full_frames_and_warns_oversized_out() {
        let dir = temp_dir("bounds");

        // 1. parse_flat_table_with_caps rejects file size, line length, skipped lines, and extra cols.
        let big_file = dir.join("big_text");
        fs::write(&big_file, "0.0 0.0 0.0\n").unwrap();
        let err = parse_flat_table_with_caps(&big_file, 1, 3, 4, MAX_FLAT_LINE_BYTES, MAX_SKIPPED_LINES).unwrap_err();
        assert!(err.to_string().contains("exceeds maximum flat file size"), "{err}");

        let long_line_file = dir.join("long_text");
        fs::write(&long_line_file, "0.0 0.0 0.0 0.0 0.0\n").unwrap();
        let err = parse_flat_table_with_caps(&long_line_file, 1, 3, MAX_FLAT_FILE_BYTES, 8, MAX_SKIPPED_LINES).unwrap_err();
        assert!(err.to_string().contains("exceeds maximum line length"), "{err}");

        let skip_file = dir.join("skip_text");
        fs::write(&skip_file, "# a\n# b\n\n0.0 0.0 0.0\n").unwrap();
        let err = parse_flat_table_with_caps(&skip_file, 1, 3, MAX_FLAT_FILE_BYTES, MAX_FLAT_LINE_BYTES, 2).unwrap_err();
        assert!(err.to_string().contains("maximum skipped"), "{err}");

        let extra_col_file = dir.join("extra_text");
        fs::write(&extra_col_file, "0.0 0.0 0.0 1.0\n").unwrap();
        let err = parse_flat_table_with_caps(&extra_col_file, 1, 3, MAX_FLAT_FILE_BYTES, MAX_FLAT_LINE_BYTES, MAX_SKIPPED_LINES).unwrap_err();
        assert!(err.to_string().contains("more than expected 3 columns"), "{err}");

        // 2. collect_step_files_with_cap rejects > max_steps.
        let steps_dir = dir.join("steps");
        fs::create_dir_all(&steps_dir).unwrap();
        fs::write(steps_dir.join("wnd00001_text"), "0.0 0.0 0.0\n").unwrap();
        fs::write(steps_dir.join("wnd00002_text"), "0.0 0.0 0.0\n").unwrap();
        let err = collect_step_files_with_cap(&steps_dir, 1).unwrap_err();
        assert!(err.to_string().contains("maximum step limit"), "{err}");

        // 3. Oversized .out streams up to cap, trims trailing partial line when newlines exist, and warns.
        let out_dir = dir.join("oversized_out");
        fs::create_dir_all(&out_dir).unwrap();
        fs::write(out_dir.join("job.out"), "Domain: d1\n  J_mean: 11.0\nDomain: d2\n").unwrap();
        let mut warnings = Vec::new();
        let (text, trunc) = read_optional_stdout_with_cap(&out_dir, None, 15, &mut warnings);
        assert!(trunc);
        assert_eq!(text, "Domain: d1\n");
        assert!(warnings.iter().any(|w| w.contains("exceeds 15-byte limit")), "{warnings:?}");

        // 4. Nonlinear step skips opening intermediate wnd*/wnr* once frames_full, while preserving
        //    Increment metadata and still reading the final step (`is_last`).
        let nl_dir = dir.join("nl_skip");
        fs::create_dir_all(&nl_dir).unwrap();
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
        for (id, p) in coords {
            model.set_node(id, p).unwrap();
        }
        model.set_element(1, ElementKind::Hex8, vec![1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 210e9, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("All", Set { elements: vec![1], ..Set::default() }).unwrap();
        model.add_set("Fix", Set { nodes: vec![1, 2, 3, 4], ..Set::default() }).unwrap();
        model.add_set("Top", Set { nodes: vec![5, 6, 7, 8], ..Set::default() }).unwrap();
        model
            .add_section(
                "Solid",
                Section { set: "All".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_boundary_condition("Clamp", BoundaryCondition::Fixed { target: Target::Set("Fix".into()) }).unwrap();
        model
            .add_load("PullLoad", Load::Force { target: Target::Set("Top".into()), vector: [0.0, 0.0, 100.0], distribution: Default::default() })
            .unwrap();
        let mut step = Step::new("Pull", Procedure::NonlinearStatic(NonlinearStatic::default()));
        step.boundary_conditions.insert("Clamp".into());
        step.loads.insert("PullLoad".into());
        model.add_step(step, None).unwrap();

        let vec_rows = "0.0 0.0 0.0\n".repeat(8);
        let stress_rows = format!("{}\n", format_26col_row(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)).repeat(8);
        let elem_rows = format!("{}\n", format_26col_row(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        // Frame 0 uses 24 values (8 * 3); step 1 fits (48); step 2 overflows cap (49), setting frames_full = true.
        // Step 3 has invalid tokens in wnd/wnr: it must be skipped rather than parsed!
        for step_idx in [1usize, 2, 4] {
            fs::write(nl_dir.join(format!("wnd{step_idx:05}_text")), &vec_rows).unwrap();
            fs::write(nl_dir.join(format!("wnr{step_idx:05}_text")), &vec_rows).unwrap();
            fs::write(nl_dir.join(format!("wns{step_idx:05}_text")), &stress_rows).unwrap();
        }
        fs::write(nl_dir.join("wes00004_text"), &elem_rows).unwrap();
        fs::write(nl_dir.join("wnd00003_text"), "CORRUPT_NOT_OPENED\n").unwrap();
        fs::write(nl_dir.join("wnr00003_text"), "CORRUPT_NOT_OPENED\n").unwrap();

        let sol = read_results_dir_inner(&model, "Pull", &nl_dir, None, String::new(), false, None, 48, Vec::new()).unwrap();
        let hist = sol.static_result.nonlinear.as_ref().unwrap();
        assert_eq!(hist.increments.len(), 4);
        assert_eq!(sol.step_numbers, vec![1, 2, 3, 4]);
        assert!(!sol.domains_truncated);
        assert!(sol.warnings.iter().any(|w| w.contains("the frames stop at load factor")), "{:?}", sol.warnings);

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn multistep_parser_retains_dumps_tracks_truncation_and_records_sparse_step_numbers() {
        let mut model = Model::default();
        for (id, x, y, z) in [
            (2u32, 0.0, 0.0, 0.0),
            (3, 1.0, 0.0, 0.0),
            (4, 1.0, 1.0, 0.0),
            (5, 0.0, 1.0, 0.0),
            (6, 0.0, 0.0, 1.0),
            (7, 1.0, 0.0, 1.0),
            (8, 1.0, 1.0, 1.0),
            (9, 0.0, 1.0, 1.0),
        ] {
            model.set_node(id, [x, y, z]).unwrap();
        }
        model.set_element(10, ElementKind::Hex8, vec![2, 3, 4, 5, 6, 7, 8, 9]).unwrap();
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 210e9, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("All", Set { elements: vec![10], ..Set::default() }).unwrap();
        model.add_set("Fix", Set { nodes: vec![2, 3, 4, 5], ..Set::default() }).unwrap();
        model.add_set("Top", Set { nodes: vec![6, 7, 8, 9], ..Set::default() }).unwrap();
        model
            .add_section(
                "Solid",
                Section { set: "All".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_boundary_condition("Clamp", BoundaryCondition::Fixed { target: Target::Set("Fix".into()) }).unwrap();
        model
            .add_load("PullLoad", Load::Force { target: Target::Set("Top".into()), vector: [0.0, 0.0, 100.0], distribution: Default::default() })
            .unwrap();
        let mut step = Step::new("Pull", Procedure::NonlinearStatic(NonlinearStatic::default()));
        step.boundary_conditions.insert("Clamp".into());
        step.loads.insert("PullLoad".into());
        model.add_step(step, None).unwrap();

        let label_map = LabelMap::from_model(&model);

        // 1. Multi-step WARP3D dump metadata is retained across steps 1, 2, 3 without being removed.
        let mut log = String::from(
            "\
loading: Pull step: 1
Domain definition: cf_0001
  Number of front nodes: 1 Nodes on front:
  1
  Direction cosines: 0.0 1.0 0.0
  Tangent vector direction cosines: 0.0 0.0 1.0
  Domain type: 1
** end of domain definition **
domain id: cf_0001
  J-integral components
  1 10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 10.0
>>> Completed domain integral calculations
",
        );
        for s in [2usize, 3] {
            log.push_str(&format!(
                "\
loading: Pull step: {s}
domain id: cf_0001
  J-integral components
  1 {j:.1} 0.0 0.0 0.0 0.0 0.0 0.0 0.0 {j:.1}
>>> Completed domain integral calculations
",
                j = (s as f64) * 10.0
            ));
        }
        let parsed = parse_domain_integrals_with_meta(&log, &label_map);
        assert!(!parsed.truncated);
        assert_eq!(parsed.domains.len(), 3);
        assert_eq!(parsed.station_nodes.len(), 3);
        for (idx, step_no) in [1usize, 2, 3].into_iter().enumerate() {
            assert_eq!(parsed.domains[idx].step, step_no);
            assert_eq!(parsed.domains[idx].front_nodes, vec![2], "step {step_no} lost dump front_nodes");
            assert_eq!(parsed.domains[idx].normal, [0.0, 1.0, 0.0], "step {step_no} lost normal");
            assert_eq!(parsed.domains[idx].tangent, [0.0, 0.0, 1.0], "step {step_no} lost tangent");
        }

        // 2. MAX_DOMAINS cap sets `domains_truncated = true`.
        let mut cap_parser = DomainIntegralParser::new(&label_map);
        for i in 0..=MAX_DOMAINS {
            cap_parser.feed_line(&format!("Domain: d{i}"));
            cap_parser.feed_line("  J_mean: 1.0");
        }
        let cap_out = cap_parser.finish();
        assert_eq!(cap_out.domains.len(), MAX_DOMAINS);
        assert!(cap_out.truncated);

        // 3. MAX_DOMAIN_LINES cap sets `domains_truncated = true`.
        let mut line_parser = DomainIntegralParser::new(&label_map);
        line_parser.domain_lines = MAX_DOMAIN_LINES;
        line_parser.feed_line("Domain: d_overflow");
        let line_out = line_parser.finish();
        assert!(line_out.truncated);

        // 4. Sparse step_numbers ([5, 10]) and oversized stdout warning propagate to Warp3dSolution.
        let dir = temp_dir("sparse-steps");
        let vec_rows = "0.0 0.0 0.0\n".repeat(8);
        let stress_rows = format!("{}\n", format_26col_row(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)).repeat(8);
        let elem_rows = format!("{}\n", format_26col_row(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        for s in [5usize, 10] {
            fs::write(dir.join(format!("wnd{s:05}_text")), &vec_rows).unwrap();
            fs::write(dir.join(format!("wnr{s:05}_text")), &vec_rows).unwrap();
            fs::write(dir.join(format!("wns{s:05}_text")), &stress_rows).unwrap();
        }
        fs::write(dir.join("wes00010_text"), &elem_rows).unwrap();
        fs::write(dir.join("job.out"), "Domain: cf_0001 (step 5)\n  J_mean: 10.0\nDomain: cf_0001 (step 10)\n  J_mean: 20.0\n").unwrap();
        let sol = read_results_dir(&model, "Pull", &dir, Some(Preset::MKgS)).unwrap();
        assert_eq!(sol.step_numbers, vec![5, 10]);
        assert!(!sol.domains_truncated);
        assert_eq!(sol.step1_is_elastic, None);
        assert_eq!(sol.fracture.len(), 2);

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn parses_stress_state_tags_captures_domain_warnings_and_scans_step1_elastic() {
        let mut model = Model::default();
        for (id, x, y, z) in [
            (1u32, 0.0, 0.0, 0.0),
            (2, 1.0, 0.0, 0.0),
            (3, 1.0, 1.0, 0.0),
            (4, 0.0, 1.0, 0.0),
            (5, 0.0, 0.0, 1.0),
            (6, 1.0, 0.0, 1.0),
            (7, 1.0, 1.0, 1.0),
            (8, 0.0, 1.0, 1.0),
        ] {
            model.set_node(id, [x, y, z]).unwrap();
        }
        model.set_element(1, ElementKind::Hex8, vec![1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: 210e9, poisson: 0.3 }), ..Default::default() }).unwrap();
        model.add_set("All", Set { elements: vec![1], ..Set::default() }).unwrap();
        model.add_set("Fix", Set { nodes: vec![1, 2, 3, 4], ..Set::default() }).unwrap();
        model.add_set("Top", Set { nodes: vec![5, 6, 7, 8], ..Set::default() }).unwrap();
        model
            .add_section(
                "Solid",
                Section { set: "All".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model.add_boundary_condition("Clamp", BoundaryCondition::Fixed { target: Target::Set("Fix".into()) }).unwrap();
        model
            .add_load("PullLoad", Load::Force { target: Target::Set("Top".into()), vector: [0.0, 0.0, 100.0], distribution: Default::default() })
            .unwrap();
        let mut step = Step::new("Pull", Procedure::NonlinearStatic(NonlinearStatic::default()));
        step.boundary_conditions.insert("Clamp".into());
        step.loads.insert("PullLoad".into());
        model.add_step(step, None).unwrap();
        let label_map = LabelMap::from_model(&model);

        // 1. Capture >>> / >>>>> warnings emitted outside an open domain id block, deduplicated and capped at 32.
        let mut warn_log = String::from(
            "\
>>> Invalid domain definition. Skipped..
>>> Warning: front nodes not monotonic
>>> Area under q-function is zero
>>>>> warning: duplicate line
>>>>> warning: duplicate line
>>>>> error: interaction integral skipped
domain id: cf_0001
  J-integral components
  1 10.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 10.0
  KI = 1, KII = 0, KIII = 0, plane stress
  1 0 0 0 0 0 0 0 0 0 25.0
>>> Completed domain integral calculations
",
        );
        for i in 0..40 {
            warn_log.push_str(&format!(">>> Warning: synthetic extra {i}\n"));
        }
        let parsed = parse_domain_integrals_with_meta(&warn_log, &label_map);
        assert_eq!(parsed.domains.len(), 1);
        assert_eq!(parsed.domains[0].interaction_stress_state, Some(InteractionStressState::PlaneStress));
        assert_eq!(parsed.warnings.len(), MAX_DOMAIN_WARNINGS);
        assert!(parsed.warnings.iter().any(|w| w.contains("Invalid domain definition")));
        assert!(parsed.warnings.iter().any(|w| w.contains("Area under q-function")));
        assert!(parsed.warnings.iter().any(|w| w.contains(">>>>> error:")));

        // 2. Multi-step Increment-1 wes* streaming sets step1_is_elastic and forwards domain warnings into static_result.warnings.
        let dir = temp_dir("step1-elastic");
        let vec_rows = "0.0 0.0 0.0\n".repeat(8);
        let stress_rows = format!("{}\n", format_26col_row(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)).repeat(8);
        for s in [1usize, 2] {
            fs::write(dir.join(format!("wnd{s:05}_text")), &vec_rows).unwrap();
            fs::write(dir.join(format!("wnr{s:05}_text")), &vec_rows).unwrap();
            fs::write(dir.join(format!("wns{s:05}_text")), &stress_rows).unwrap();
        }
        let elastic_wes = format!("{}\n", format_26col_row(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        let plastic_wes = format!("{}\n", format_26col_row(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.002));
        fs::write(dir.join("wes00001_text"), &elastic_wes).unwrap();
        fs::write(dir.join("wes00002_text"), &plastic_wes).unwrap();
        fs::write(dir.join("job.out"), ">>> Warning: out-of-block test\n").unwrap();

        let sol_elastic = read_results_dir(&model, "Pull", &dir, Some(Preset::MKgS)).unwrap();
        assert_eq!(sol_elastic.step1_is_elastic, Some(true));
        assert!(sol_elastic.static_result.warnings.iter().any(|w| w.contains("out-of-block test")));

        // Yielded Step 1 -> Some(false)
        fs::write(dir.join("wes00001_text"), &plastic_wes).unwrap();
        let sol_plastic = read_results_dir(&model, "Pull", &dir, Some(Preset::MKgS)).unwrap();
        assert_eq!(sol_plastic.step1_is_elastic, Some(false));

        // Truncated/malformed Step 1 -> None
        fs::write(dir.join("wes00001_text"), "0.0 0.0\n").unwrap();
        let sol_bad = read_results_dir(&model, "Pull", &dir, Some(Preset::MKgS)).unwrap();
        assert_eq!(sol_bad.step1_is_elastic, None);

        let _ = fs::remove_dir_all(&dir);
    }
}

//! Audited WARP3D 18.5.0 keyword matching, `*` directive splicing, include streaming,
//! constraints, and static loading parser.

use std::collections::{BTreeMap, BTreeSet};

use fieldcraft_model::{ElementKind, Integration, MAX_LABEL, progress};

use super::lex::{Lexer, Record, Token, parse_f64};
use super::{IncludeRequest, LexLimits, MAX_UNKNOWN_SHOWN, ReadError, ReadLimits, ResolvedInclude, StoppedAt, UnknownDirective};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RawMaterialKind {
    LinearElastic,
    Bilinear,
    Mises,
}

#[derive(Debug, Clone)]
pub(crate) struct RawMaterial {
    pub name: String,
    pub line: usize,
    pub kind: RawMaterialKind,
    pub young: Option<f64>,
    pub poisson: Option<f64>,
    pub rho: Option<f64>,
    pub alpha: Option<f64>,
    pub beta: Option<f64>,
    pub yld_pt: Option<f64>,
    pub tan_e: Option<f64>,
    pub n_power: Option<f64>,
    pub curve_id: Option<u32>,
}

#[derive(Debug, Clone)]
pub(crate) struct RawCurve {
    pub id: u32,
    pub line: usize,
    pub points: Vec<[f64; 2]>,
    pub non_linear_interp: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ElementOrder {
    Default,
    Hex1x1x1,
    Hex2x2x2,
    Hex3x3x3,
    Pt1,
    Pt4,
    Pt5,
    Pt11,
    Pt14,
    Unsupported(u8),
}

#[derive(Debug, Clone)]
pub(crate) struct ElementSpec {
    pub kind: ElementKind,
    pub material: String,
    pub integration: Integration,
    pub nlgeom: bool,
    pub line: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct DeferredElemList {
    pub list_name: String,
    pub seq: usize,
    pub spec: ElementSpec,
    pub line: usize,
}

#[derive(Debug, Clone)]
pub(crate) enum RawList {
    Explicit { ids: Vec<u32>, line: usize, referenced_by_elements: bool, referenced_by_nodes: bool },
    Geometric { line: usize, referenced_by_elements: bool, referenced_by_nodes: bool },
}

#[derive(Debug, Clone)]
pub(crate) enum IdSelector {
    All,
    Ids(Vec<u32>),
    List(String),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum PlaneAction {
    Fixed,
    Symmetry,
    Dofs([Option<f64>; 3]),
}

#[derive(Debug, Clone)]
pub(crate) struct RawPlaneConstraint {
    pub axis: usize,
    pub value: f64,
    pub proximity: Option<f64>,
    pub action: PlaneAction,
    pub file: String,
    pub line: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct RawNodalConstraint {
    pub selectors: Vec<IdSelector>,
    pub values: [Option<f64>; 3],
    pub file: String,
    pub line: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct RawMultipoint {
    pub node: u32,
    pub terms: Vec<(usize, f64)>,
    pub file: String,
    pub line: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct RawNodalLoad {
    pub pattern: String,
    pub selectors: Vec<IdSelector>,
    pub force: [f64; 3],
    pub file: String,
    pub line: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct RawElementPressure {
    pub pattern: String,
    pub selectors: Vec<IdSelector>,
    pub face: u8,
    pub is_abaqus: bool,
    pub pressure: f64,
    pub file: String,
    pub line: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct RawElementTraction {
    pub pattern: String,
    pub selectors: Vec<IdSelector>,
    pub face: u8,
    pub is_abaqus: bool,
    pub traction: [f64; 3],
    pub file: String,
    pub line: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct RawElementBody {
    pub pattern: String,
    pub selectors: Vec<IdSelector>,
    pub body: [f64; 3],
    pub file: String,
    pub line: usize,
}

pub(crate) struct ParsedDeck {
    pub declared_nodes: Option<usize>,
    pub declared_elements: Option<usize>,
    pub nodes: BTreeMap<u32, ([f64; 3], usize)>,
    pub incidences: Vec<(u32, Vec<u32>, usize)>,
    pub default_elem_spec: Option<(usize, ElementSpec)>,
    pub elem_overrides: BTreeMap<u32, (usize, ElementSpec)>,
    pub deferred_elem_lists: Vec<DeferredElemList>,
    pub curves: BTreeMap<u32, RawCurve>,
    pub materials: BTreeMap<String, RawMaterial>,
    pub lists: BTreeMap<String, RawList>,
    pub nodal_constraints: Vec<RawNodalConstraint>,
    pub deferred_planes: Vec<RawPlaneConstraint>,
    pub multipoints: Vec<RawMultipoint>,
    pub nodal_loads: Vec<RawNodalLoad>,
    pub elem_pressures: Vec<RawElementPressure>,
    pub elem_tractions: Vec<RawElementTraction>,
    pub elem_bodies: Vec<RawElementBody>,
    pub saw_temperature_load: bool,
    pub unknown: Vec<UnknownDirective>,
    pub unknown_count: usize,
    pub warnings: Vec<String>,
    pub stopped_at: Option<StoppedAt>,
    pub includes_read: Vec<String>,
}

/// WARP3D prefix match (`token.len() >= min_len` and ASCII case-insensitive prefix equals `word`).
pub(crate) fn matchs(token: &str, word: &str, min_len: usize) -> bool {
    let Some(t_pref) = token.as_bytes().get(..min_len) else {
        return false;
    };
    let Some(w_pref) = word.as_bytes().get(..min_len) else {
        return false;
    };
    t_pref.eq_ignore_ascii_case(w_pref)
}

pub(crate) fn matchs_exact(token: &str, word: &str) -> bool {
    token.eq_ignore_ascii_case(word)
}

fn star_directive(rec: &Record) -> Option<(&str, usize)> {
    let first = rec.tok_obj(0)?;
    if first.quoted {
        return None;
    }
    let after = first.text.strip_prefix('*')?;
    if after.is_empty() {
        let second = rec.tok(1).unwrap_or("");
        Some((second, 2))
    } else {
        Some((after, 1))
    }
}

fn parse_star_parameter(rec: &Record, start_idx: usize, lex: &mut Lexer<'_>) -> Result<(), ReadError> {
    let rest = rec.tokens.get(start_idx..).unwrap_or(&[]);
    if rest.is_empty() {
        return Err(ReadError::Parse {
            file: rec.file.clone(),
            line: rec.line,
            message: "expected `<name> = <value>` or `display` after `*parameter`".into(),
        });
    }
    if rest.len() == 1 && rest.first().is_some_and(|t| !t.quoted && matchs(&t.text, "display", 4)) {
        return Ok(());
    }
    let mut i = 0usize;
    while let Some(name_tok) = rest.get(i) {
        let name = name_tok.text.clone();
        i += 1;
        if rest.get(i).is_some_and(|t| !t.quoted && t.text == "=") {
            i += 1;
        }
        let Some(val_tok) = rest.get(i) else {
            return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: format!("missing value for `*parameter` `{name}`") });
        };
        i += 1;
        lex.set_param(name, val_tok.text.clone(), &rec.file, rec.line)?;
    }
    Ok(())
}

fn parse_star_input<F>(rec: &Record, start_idx: usize, lex: &mut Lexer<'_>, include_loader: &mut F) -> Result<(), ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    let mut i = start_idx;
    if let Some(t) = rec.tok_obj(i)
        && !t.quoted
        && matchs(&t.text, "from", 4)
        && rec.tokens.get(i + 1).is_some()
    {
        i += 1;
    }
    if let Some(t) = rec.tok_obj(i)
        && !t.quoted
        && matchs(&t.text, "file", 4)
        && rec.tokens.get(i + 1).is_some()
    {
        i += 1;
    }
    let Some(target_tok) = rec.tok_obj(i) else {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: "missing file name after `*input`".into() });
    };
    if let Some(extra) = rec.tok(i + 1) {
        return Err(ReadError::Parse {
            file: rec.file.clone(),
            line: rec.line,
            message: format!("unexpected trailing token `{extra}` after `*input` target"),
        });
    }
    let cleaned: String = target_tok.text.chars().filter(|c| !c.is_ascii_whitespace()).collect();
    if cleaned.is_empty() {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: "empty file name after `*input`".into() });
    }
    if cleaned == "~" || cleaned.starts_with("~/") || cleaned.starts_with("~\\") {
        return Err(ReadError::IncludeRefused(format!("home-relative `*input` path `{cleaned}` is refused")));
    }
    if !target_tok.quoted
        && (matchs_exact(&cleaned, "window")
            || matchs_exact(&cleaned, "terminal")
            || matchs_exact(&cleaned, "screen")
            || matchs_exact(&cleaned, "display"))
    {
        return Err(ReadError::Unsupported {
            file: rec.file.clone(),
            line: rec.line,
            feature: "interactive *input".into(),
            message: format!("interactive `*input {cleaned}` is not supported"),
        });
    }
    lex.push_include(&cleaned, &rec.file, rec.line, include_loader)
}

/// Pull the next logical record while splicing `*` directives cleanly between records.
fn next_record<F>(lex: &mut Lexer<'_>, include_loader: &mut F) -> Result<Option<Record>, ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    while let Some(rec) = lex.next_raw()? {
        if let Some((dir, start_idx)) = star_directive(&rec) {
            if matchs(dir, "echo", 4) {
                continue;
            }
            if matchs(dir, "parameter", 5) {
                parse_star_parameter(&rec, start_idx, lex)?;
                continue;
            }
            if matchs(dir, "input", 5) || matchs(dir, "inout", 5) {
                parse_star_input(&rec, start_idx, lex, include_loader)?;
                continue;
            }
            let msg = format!("unknown `*` directive `*{dir}`");
            return Err(ReadError::Parse { file: rec.file, line: rec.line, message: msg });
        }
        return Ok(Some(rec));
    }
    Ok(None)
}

fn is_stress_strain_header(rec: &Record) -> bool {
    let Some(t0) = rec.tok_obj(0) else {
        return false;
    };
    if t0.quoted {
        return false;
    }
    if matchs_exact(&t0.text, "stress-strain") {
        return true;
    }
    matchs(&t0.text, "stress", 6) && rec.tok(1).is_some_and(|t1| matchs(t1, "strain", 6))
}

fn is_top_level_directive(rec: &Record) -> bool {
    let Some(t0_obj) = rec.tok_obj(0) else {
        return false;
    };
    if t0_obj.quoted {
        return false;
    }
    let t0 = t0_obj.text.as_str();
    let t1 = rec.tok(1);

    if is_stress_strain_header(rec) {
        return true;
    }
    if matchs(t0, "number", 3) {
        return t1.is_some_and(|t| matchs(t, "of", 2) || matchs(t, "nodes", 4) || matchs(t, "elements", 4));
    }
    if matchs(t0, "nonlinear", 7) || matchs(t0, "dynamic", 7) || matchs(t0, "solution", 4) {
        // Inside `loading <name>`, bare `nonlinear` or `dynamic` starts a driver step schedule,
        // and inside `nonlinear analysis parameters`, `solution technique ...` sets solver method.
        return t1.is_some_and(|t| matchs(t, "analysis", 5) || matchs(t, "parameters", 5));
    }
    if matchs(t0, "analysis", 5) {
        return t1.is_some_and(|t| matchs(t, "parameters", 5));
    }
    if matchs(t0, "tie", 3) {
        return t1.is_some_and(|t| matchs(t, "mesh", 4));
    }

    matchs(t0, "structure", 9)
        || matchs(t0, "material", 8)
        || matchs(t0, "elements", 8)
        || matchs(t0, "coordinates", 5)
        || matchs(t0, "incidences", 5)
        || matchs(t0, "blocking", 5)
        || matchs(t0, "constraints", 11)
        || matchs(t0, "loading", 7)
        || matchs(t0, "compute", 4)
        || matchs(t0, "list", 4)
        || matchs(t0, "parameters", 5)
        || matchs(t0, "output", 6)
        || matchs(t0, "save", 4)
        || matchs(t0, "restart", 4)
        || matchs(t0, "retrieve", 4)
        || matchs(t0, "release", 7)
        || matchs(t0, "mesh", 4)
        || matchs(t0, "initial", 4)
        || matchs(t0, "contact", 4)
        || matchs(t0, "thermal", 4)
        || matchs(t0, "functionally", 4)
        || matchs(t0, "crystal", 7)
        || matchs(t0, "crack", 5)
        || matchs(t0, "domain", 5)
        || matchs(t0, "table", 5)
        || matchs(t0, "surface", 7)
        || matchs(t0, "stop", 4)
        || matchs(t0, "exit", 4)
        || matchs(t0, "quit", 4)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SkipMode {
    General,
    Parameters,
}

fn record_unknown(parsed: &mut ParsedDeck, keyword: &str, rec: &Record) {
    parsed.unknown_count = parsed.unknown_count.saturating_add(1);
    if parsed.unknown.len() < MAX_UNKNOWN_SHOWN {
        parsed.unknown.push(UnknownDirective {
            keyword: keyword.to_string(),
            file: if rec.is_root { None } else { Some(rec.file.clone()) },
            line: rec.line,
        });
    }
}

fn skip_block<F>(
    lex: &mut Lexer<'_>,
    include_loader: &mut F,
    parsed: &mut ParsedDeck,
    keyword: &str,
    rec: &Record,
    mode: SkipMode,
) -> Result<(), ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    record_unknown(parsed, keyword, rec);
    while let Some(row) = next_record(lex, include_loader)? {
        if mode == SkipMode::Parameters
            && let Some(t0_obj) = row.tok_obj(0)
            && !t0_obj.quoted
            && matchs(&t0_obj.text, "material", 8)
            && row.tok(1).is_some_and(|t1| matchs(t1, "messages", 4))
        {
            continue;
        }
        if is_top_level_directive(&row) {
            lex.push_back(row);
            break;
        }
    }
    Ok(())
}

fn parse_label(tok: &str, file: &str, line: usize) -> Result<u32, ReadError> {
    let id = tok.parse::<u32>().map_err(|_| ReadError::Parse { file: file.to_string(), line, message: format!("invalid integer label `{tok}`") })?;
    if id == 0 || id > MAX_LABEL {
        return Err(ReadError::Parse { file: file.to_string(), line, message: format!("label {id} is outside 1..={MAX_LABEL}") });
    }
    Ok(id)
}

fn parse_positive_usize(tok: &str, file: &str, line: usize) -> Result<usize, ReadError> {
    let val =
        tok.parse::<usize>().map_err(|_| ReadError::Parse { file: file.to_string(), line, message: format!("invalid integer count `{tok}`") })?;
    if val == 0 {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "count must be >= 1".into() });
    }
    Ok(val)
}

struct IdRange {
    start: u32,
    end: u32,
    step: u32,
    count: u64,
}

fn parse_id_or_range(tokens: &[Token], idx: usize, file: &str, line: usize, remaining_budget: usize) -> Result<(IdRange, usize), ReadError> {
    let tok0 = tokens.get(idx).map(|t| t.text.as_str()).unwrap_or("");
    let start = parse_label(tok0, file, line)?;
    let next = tokens.get(idx + 1).map(|t| t.text.as_str());
    if next.is_some_and(|t| t == "-" || matchs_exact(t, "to")) {
        let Some(end_tok) = tokens.get(idx + 2).map(|t| t.text.as_str()) else {
            return Err(ReadError::Parse { file: file.to_string(), line, message: "incomplete ID range".into() });
        };
        let end = parse_label(end_tok, file, line)?;
        let mut cursor = idx + 3;
        let step = if tokens.get(cursor).is_some_and(|t| matchs_exact(&t.text, "by")) {
            let Some(step_tok) = tokens.get(cursor + 1).map(|t| t.text.as_str()) else {
                return Err(ReadError::Parse { file: file.to_string(), line, message: "missing step after `by`".into() });
            };
            let s = step_tok.parse::<u32>().map_err(|_| ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("invalid range step `{step_tok}`"),
            })?;
            if s == 0 || s > MAX_LABEL {
                return Err(ReadError::Parse { file: file.to_string(), line, message: format!("range step `{s}` must be in 1..={MAX_LABEL}") });
            }
            cursor += 2;
            s
        } else {
            1
        };
        if start > end {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("reversed ID range {start}-{end} (`start` must be <= `end`)"),
            });
        }
        let count = u64::from(end - start) / u64::from(step) + 1;
        if count > remaining_budget as u64 {
            return Err(ReadError::Limit(format!(
                "range {start}-{end} by {step} ({count} entries) exceeds remaining {remaining_budget} member budget"
            )));
        }
        Ok((IdRange { start, end, step, count }, cursor))
    } else {
        if remaining_budget == 0 {
            return Err(ReadError::Limit("member budget reached".into()));
        }
        Ok((IdRange { start, end: start, step: 1, count: 1 }, idx + 1))
    }
}

fn expand_range_into(range: &IdRange, out: &mut Vec<u32>) -> Result<(), ReadError> {
    let mut cur = u64::from(range.start);
    let end = u64::from(range.end);
    let step = u64::from(range.step);
    let mut emitted = 0usize;
    while cur <= end {
        if emitted.is_multiple_of(256) {
            progress::check()?;
        }
        out.push(cur as u32);
        emitted += 1;
        cur += step;
    }
    Ok(())
}

fn parse_number_directive(rec: &Record, parsed: &mut ParsedDeck) -> Result<(), ReadError> {
    let mut idx = 1;
    if rec.tok(idx).is_some_and(|t| matchs(t, "of", 2)) {
        idx += 1;
    }
    let mut saw_any = false;
    while let Some(tok) = rec.tok(idx) {
        if matchs(tok, "nodes", 4) {
            idx += 1;
            if rec.tok(idx) == Some("=") {
                idx += 1;
            }
            let Some(val_tok) = rec.tok(idx) else {
                return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: "missing node count".into() });
            };
            parsed.declared_nodes = Some(parse_positive_usize(val_tok, &rec.file, rec.line)?);
            idx += 1;
            saw_any = true;
        } else if matchs(tok, "elements", 4) {
            idx += 1;
            if rec.tok(idx) == Some("=") {
                idx += 1;
            }
            let Some(val_tok) = rec.tok(idx) else {
                return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: "missing element count".into() });
            };
            parsed.declared_elements = Some(parse_positive_usize(val_tok, &rec.file, rec.line)?);
            idx += 1;
            saw_any = true;
        } else {
            return Err(ReadError::Parse {
                file: rec.file.clone(),
                line: rec.line,
                message: format!("unexpected token `{tok}` in `number` directive"),
            });
        }
    }
    if !saw_any {
        return Err(ReadError::Parse {
            file: rec.file.clone(),
            line: rec.line,
            message: "`number` directive requires `nodes` and/or `elements`".into(),
        });
    }
    Ok(())
}

fn parse_stress_strain_curve<F>(rec: &Record, lex: &mut Lexer<'_>, include_loader: &mut F, parsed: &mut ParsedDeck) -> Result<(), ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    let mut idx = if rec.tok(0).is_some_and(|t| matchs_exact(t, "stress-strain")) { 1 } else { 2 };
    let Some(curve_tok) = rec.tok(idx) else {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: "expected `curve` after `stress-strain`".into() });
    };
    if !matchs(curve_tok, "curve", 5) {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: format!("expected `curve`, found `{curve_tok}`") });
    }
    idx += 1;
    let Some(id_tok) = rec.tok(idx) else {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: "missing curve id".into() });
    };
    let id = parse_label(id_tok, &rec.file, rec.line)?;
    idx += 1;

    let mut non_linear_interp = false;
    while let Some(opt) = rec.tok(idx) {
        if matchs(opt, "temperature", 4) {
            return Err(ReadError::Unsupported {
                file: rec.file.clone(),
                line: rec.line,
                feature: "temperature-dependent curve".into(),
                message: "temperature-dependent `stress-strain curve` tables are not supported".into(),
            });
        } else if matchs(opt, "plastic", 4) || matchs(opt, "strain-rate", 6) || matchs(opt, "rate", 4) {
            return Err(ReadError::Unsupported {
                file: rec.file.clone(),
                line: rec.line,
                feature: "rate-dependent curve".into(),
                message: "rate-dependent `stress-strain curve` tables are not supported".into(),
            });
        } else if matchs(opt, "pchip", 5) || matchs(opt, "smooth", 5) {
            non_linear_interp = true;
            idx += 1;
        } else {
            return Err(ReadError::Parse {
                file: rec.file.clone(),
                line: rec.line,
                message: format!("unexpected token `{opt}` in `stress-strain curve` header"),
            });
        }
    }

    let max_raw_numbers = fieldcraft_model::materials::MAX_TABLE_POINTS.saturating_mul(4);
    let mut nums = Vec::new();
    while let Some(row) = next_record(lex, include_loader)? {
        if is_top_level_directive(&row) {
            lex.push_back(row);
            break;
        }
        for tok in &row.tokens {
            let Some(v) = parse_f64(&tok.text) else {
                return Err(ReadError::Parse {
                    file: row.file.clone(),
                    line: row.line,
                    message: format!("invalid number `{}` in stress-strain curve {id}", tok.text),
                });
            };
            if nums.len() >= max_raw_numbers {
                return Err(ReadError::Limit(format!("stress-strain curve {id} exceeds {} raw value limit", max_raw_numbers)));
            }
            nums.push(v);
        }
    }

    if nums.is_empty() || !nums.len().is_multiple_of(2) {
        return Err(ReadError::Parse {
            file: rec.file.clone(),
            line: rec.line,
            message: format!("stress-strain curve {id} requires one or more (strain, stress) pairs"),
        });
    }
    let points: Vec<[f64; 2]> = nums.as_chunks::<2>().0.to_vec();
    if parsed.curves.contains_key(&id) {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: format!("duplicate stress-strain curve {id}") });
    }
    parsed.curves.insert(id, RawCurve { id, line: rec.line, points, non_linear_interp });
    Ok(())
}

fn next_value_tok<'a>(tokens: &'a [Token], idx: &mut usize, file: &str, line: usize, what: &str) -> Result<&'a str, ReadError> {
    if tokens.get(*idx).is_some_and(|t| t.text == "=") {
        *idx += 1;
    }
    let Some(tok) = tokens.get(*idx) else {
        return Err(ReadError::Parse { file: file.to_string(), line, message: format!("missing value for `{what}`") });
    };
    *idx += 1;
    Ok(tok.text.as_str())
}

fn parse_material_block<F>(rec: &Record, lex: &mut Lexer<'_>, include_loader: &mut F, parsed: &mut ParsedDeck) -> Result<(), ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    let Some(name_tok) = rec.tok(1) else {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: "missing material name".into() });
    };
    let mat_name = name_tok.to_string();
    if parsed.materials.contains_key(&mat_name) {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: format!("duplicate material `{mat_name}`") });
    }
    if parsed.materials.len() >= fieldcraft_model::MAX_DEFINITIONS {
        return Err(ReadError::Limit(format!("material count exceeds {}", fieldcraft_model::MAX_DEFINITIONS)));
    }

    let mut all_tokens: Vec<(String, usize)> = Vec::new();
    for t in rec.tokens.iter().skip(2) {
        all_tokens.push((t.text.clone(), rec.line));
    }
    while let Some(row) = next_record(lex, include_loader)? {
        if is_top_level_directive(&row) {
            lex.push_back(row);
            break;
        }
        for t in row.tokens {
            all_tokens.push((t.text, row.line));
        }
    }

    let mut kind: Option<RawMaterialKind> = None;
    let mut young: Option<f64> = None;
    let mut poisson: Option<f64> = None;
    let mut rho: Option<f64> = None;
    let mut alpha: Option<f64> = None;
    let mut beta: Option<f64> = None;
    let mut yld_pt: Option<f64> = None;
    let mut tan_e: Option<f64> = None;
    let mut n_power: Option<f64> = None;
    let mut curve_id: Option<u32> = None;

    let mut i = 0usize;
    while let Some((tok, line)) = all_tokens.get(i).cloned() {
        i += 1;
        if matchs(&tok, "properties", 10) {
            continue;
        }
        // Check exact `curves` before prefix `curve`!
        if matchs_exact(&tok, "curves")
            || matchs_exact(&tok, "gurson")
            || matchs_exact(&tok, "creep")
            || matchs_exact(&tok, "cyclic")
            || matchs_exact(&tok, "mises_hydrogen")
            || matchs_exact(&tok, "link")
            || matchs(&tok, "cohesive", 6)
            || matchs(&tok, "umat", 4)
            || matchs(&tok, "deformation", 8)
            || matchs(&tok, "killable", 4)
        {
            return Err(ReadError::Unsupported {
                file: rec.file.clone(),
                line,
                feature: format!("material `{tok}`"),
                message: format!("material feature `{tok}` on `{mat_name}` is not supported"),
            });
        }

        if matchs(&tok, "linear_elastic", 6) {
            kind = Some(RawMaterialKind::LinearElastic);
        } else if matchs_exact(&tok, "bilinear") {
            kind = Some(RawMaterialKind::Bilinear);
        } else if matchs_exact(&tok, "mises") {
            kind = Some(RawMaterialKind::Mises);
        } else {
            let mut consume_val = |what: &str| -> Result<(String, usize), ReadError> {
                if all_tokens.get(i).is_some_and(|(t, _)| t == "=") {
                    i += 1;
                }
                let Some(pair) = all_tokens.get(i).cloned() else {
                    return Err(ReadError::Parse { file: rec.file.clone(), line, message: format!("missing value for `{what}`") });
                };
                i += 1;
                Ok(pair)
            };
            let parse_mat_f64 = |v: &str, vl: usize, what: &str| -> Result<f64, ReadError> {
                if matchs_exact(v, "fgm") {
                    return Err(ReadError::Unsupported {
                        file: rec.file.clone(),
                        line: vl,
                        feature: "functionally graded material".into(),
                        message: format!("functionally graded material property `{what} fgm` on `{mat_name}` is not supported"),
                    });
                }
                parse_f64(v).ok_or_else(|| ReadError::Parse { file: rec.file.clone(), line: vl, message: format!("invalid number `{v}`") })
            };

            if matchs_exact(&tok, "e") {
                let (v, vl) = consume_val("e")?;
                young = Some(parse_mat_f64(&v, vl, "e")?);
            } else if matchs_exact(&tok, "nu") {
                let (v, vl) = consume_val("nu")?;
                poisson = Some(parse_mat_f64(&v, vl, "nu")?);
            } else if matchs_exact(&tok, "rho") {
                let (v, vl) = consume_val("rho")?;
                rho = Some(parse_mat_f64(&v, vl, "rho")?);
            } else if matchs_exact(&tok, "beta") {
                let (v, vl) = consume_val("beta")?;
                beta = Some(parse_mat_f64(&v, vl, "beta")?);
            } else if matchs(&tok, "tan_e", 5) {
                let (v, vl) = consume_val("tan_e")?;
                tan_e = Some(parse_mat_f64(&v, vl, "tan_e")?);
            } else if matchs(&tok, "yld_pt", 6) || matchs(&tok, "sig_o", 5) {
                let (v, vl) = consume_val(&tok)?;
                yld_pt = Some(parse_mat_f64(&v, vl, &tok)?);
            } else if matchs(&tok, "n_power", 5) {
                let (v, vl) = consume_val("n_power")?;
                n_power = Some(parse_mat_f64(&v, vl, "n_power")?);
            } else if matchs(&tok, "curve", 5) {
                let (v, vl) = consume_val("curve")?;
                curve_id = Some(parse_label(&v, &rec.file, vl)?);
            } else if matchs(&tok, "alpha", 5) {
                let (v, vl) = consume_val("alpha")?;
                alpha = Some(parse_mat_f64(&v, vl, "alpha")?);
            } else if matchs(&tok, "ref_eps", 4) || matchs(&tok, "m_power", 4) {
                return Err(ReadError::Unsupported {
                    file: rec.file.clone(),
                    line,
                    feature: "rate-dependent plasticity".into(),
                    message: format!("rate-dependent plasticity keyword `{tok}` on `{mat_name}` is not supported"),
                });
            } else {
                return Err(ReadError::Parse {
                    file: rec.file.clone(),
                    line,
                    message: format!("unrecognized material keyword `{tok}` on `{mat_name}`"),
                });
            }
        }
    }

    let kind = kind
        .or_else(|| {
            if tan_e.is_some() {
                Some(RawMaterialKind::Bilinear)
            } else if n_power.is_some() || curve_id.is_some() || yld_pt.is_some() {
                Some(RawMaterialKind::Mises)
            } else if young.is_some() && poisson.is_some() {
                Some(RawMaterialKind::LinearElastic)
            } else {
                None
            }
        })
        .ok_or_else(|| ReadError::Parse {
            file: rec.file.clone(),
            line: rec.line,
            message: format!("material `{mat_name}` is missing a constitutive model (`linear_elastic`, `bilinear`, or `mises`)"),
        })?;

    parsed.materials.insert(
        mat_name.clone(),
        RawMaterial { name: mat_name, line: rec.line, kind, young, poisson, rho, alpha, beta, yld_pt, tan_e, n_power, curve_id },
    );
    Ok(())
}

fn is_element_spec_start(tok: &str, lists: &BTreeMap<String, RawList>) -> bool {
    if lists.contains_key(tok) {
        return false;
    }
    matchs(tok, "type", 4)
        || matchs(tok, "properties", 10)
        || matchs(tok, "order", 5)
        || matchs(tok, "material", 8)
        || matchs(tok, "linear", 6)
        || matchs(tok, "nonlinear", 6)
        || matchs(tok, "nlgeom", 5)
        || matchs(tok, "geonl", 5)
        || matchs(tok, "bbar", 4)
        || matchs(tok, "no_bbar", 6)
        || matchs(tok, "nobbar", 6)
        || matchs(tok, "center_output", 6)
        || matchs(tok, "gausspts", 8)
}

fn parse_element_type(tok: &str, file: &str, line: usize) -> Result<ElementKind, ReadError> {
    if matchs_exact(tok, "l3disop") {
        Ok(ElementKind::Hex8)
    } else if matchs_exact(tok, "q3disop") {
        Ok(ElementKind::Hex20)
    } else if matchs_exact(tok, "tet4") {
        Ok(ElementKind::Tet4)
    } else if matchs_exact(tok, "tet10") {
        Ok(ElementKind::Tet10)
    } else {
        Err(ReadError::Unsupported {
            file: file.to_string(),
            line,
            feature: format!("element type `{tok}`"),
            message: format!("WARP3D element type `{tok}` is not supported (only l3disop, q3disop, tet4, tet10)"),
        })
    }
}

fn parse_element_order(tok: &str, file: &str, line: usize) -> Result<ElementOrder, ReadError> {
    if matchs_exact(tok, "1x1x1") {
        Ok(ElementOrder::Hex1x1x1)
    } else if matchs(tok, "2x2x2", 5) {
        Ok(ElementOrder::Hex2x2x2)
    } else if matchs(tok, "3x3x3", 5) {
        Ok(ElementOrder::Hex3x3x3)
    } else if matchs(tok, "1pt_rule", 3) {
        Ok(ElementOrder::Pt1)
    } else if matchs(tok, "4pt_rule", 3) {
        Ok(ElementOrder::Pt4)
    } else if matchs(tok, "5pt_rule", 3) {
        Ok(ElementOrder::Pt5)
    } else if matchs_exact(tok, "11pt") || matchs_exact(tok, "11pt_rule") {
        Ok(ElementOrder::Pt11)
    } else if matchs(tok, "14pt_rule", 4) {
        Ok(ElementOrder::Pt14)
    } else if matchs(tok, "6pt_rule", 3) {
        Ok(ElementOrder::Unsupported(6))
    } else if matchs(tok, "7pt_rule", 3) {
        Ok(ElementOrder::Unsupported(7))
    } else if matchs(tok, "9pt_rule", 3) {
        Ok(ElementOrder::Unsupported(9))
    } else {
        Err(ReadError::Parse { file: file.to_string(), line, message: format!("unrecognized integration order `{tok}`") })
    }
}

fn resolve_integration(kind: ElementKind, order: ElementOrder, file: &str, line: usize) -> Result<Integration, ReadError> {
    match (kind, order) {
        (ElementKind::Hex8, ElementOrder::Default | ElementOrder::Hex2x2x2) => Ok(Integration::Full),
        (ElementKind::Hex8, ElementOrder::Hex1x1x1 | ElementOrder::Pt1) => Ok(Integration::Reduced),
        (ElementKind::Hex20, ElementOrder::Default | ElementOrder::Hex2x2x2 | ElementOrder::Hex3x3x3) => Ok(Integration::Full),
        (ElementKind::Tet4, ElementOrder::Default | ElementOrder::Pt1 | ElementOrder::Pt4) => Ok(Integration::Full),
        (ElementKind::Tet10, ElementOrder::Default | ElementOrder::Pt4 | ElementOrder::Pt5 | ElementOrder::Pt11 | ElementOrder::Pt14) => {
            Ok(Integration::Full)
        }
        _ => Err(ReadError::Unsupported {
            file: file.to_string(),
            line,
            feature: "element integration order".into(),
            message: format!("unsupported integration order for `{}` element", kind.name()),
        }),
    }
}

fn parse_element_assignment(
    tokens: &[Token],
    file: &str,
    line: usize,
    seq: &mut usize,
    limits: ReadLimits,
    member_budget: &mut usize,
    parsed: &mut ParsedDeck,
) -> Result<(), ReadError> {
    enum Selector {
        All,
        Ids(Vec<u32>),
        List(String),
    }

    let mut idx = 0usize;
    let mut selectors: Vec<Selector> = Vec::new();
    while let Some(tok_obj) = tokens.get(idx) {
        let tok = tok_obj.text.as_str();
        if !tok_obj.quoted && is_element_spec_start(tok, &parsed.lists) {
            break;
        }
        if !tok_obj.quoted && matchs_exact(tok, "all") {
            selectors.push(Selector::All);
            idx += 1;
        } else if !tok_obj.quoted && tok.bytes().next().is_some_and(|b| b.is_ascii_digit()) {
            let (range, next_idx) = parse_id_or_range(tokens, idx, file, line, *member_budget)?;
            if range.count > limits.elements as u64 {
                return Err(ReadError::Limit(format!("element assignment range ({}) exceeds element limit {}", range.count, limits.elements)));
            }
            *member_budget = member_budget.saturating_sub(range.count as usize);
            let mut ids = Vec::with_capacity((range.count as usize).min(4096));
            expand_range_into(&range, &mut ids)?;
            selectors.push(Selector::Ids(ids));
            idx = next_idx;
        } else {
            selectors.push(Selector::List(tok.to_string()));
            idx += 1;
        }
    }

    if selectors.is_empty() {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "missing element IDs or `all` selector in `elements`".into() });
    }

    let mut kind: Option<ElementKind> = None;
    let mut material: Option<String> = None;
    let mut order = ElementOrder::Default;
    let mut nlgeom = false;

    while let Some(tok) = tokens.get(idx).map(|t| t.text.as_str()) {
        idx += 1;
        if matchs(tok, "properties", 10)
            || matchs(tok, "bbar", 4)
            || matchs(tok, "no_bbar", 6)
            || matchs(tok, "nobbar", 6)
            || matchs(tok, "center_output", 6)
            || matchs(tok, "gausspts", 8)
            || matchs(tok, "short", 5)
            || matchs(tok, "long", 4)
        {
            continue;
        } else if matchs(tok, "linear", 6) {
            nlgeom = false;
        } else if matchs(tok, "nonlinear", 6) || matchs(tok, "nlgeom", 5) || matchs(tok, "geonl", 5) {
            nlgeom = true;
        } else if matchs(tok, "type", 4) {
            let val = next_value_tok(tokens, &mut idx, file, line, "type")?;
            kind = Some(parse_element_type(val, file, line)?);
        } else if matchs(tok, "material", 8) {
            let val = next_value_tok(tokens, &mut idx, file, line, "material")?;
            material = Some(val.to_string());
        } else if matchs(tok, "order", 5) {
            let val = next_value_tok(tokens, &mut idx, file, line, "order")?;
            order = parse_element_order(val, file, line)?;
        } else {
            return Err(ReadError::Parse { file: file.to_string(), line, message: format!("unrecognized token `{tok}` in `elements`") });
        }
    }

    let kind = kind.ok_or_else(|| ReadError::Parse { file: file.to_string(), line, message: "`elements` requires `type <kind>`".into() })?;
    let material =
        material.ok_or_else(|| ReadError::Parse { file: file.to_string(), line, message: "`elements` requires `material <name>`".into() })?;
    let integration = resolve_integration(kind, order, file, line)?;
    let spec = ElementSpec { kind, material, integration, nlgeom, line };

    for sel in selectors {
        *seq = seq.saturating_add(1);
        match sel {
            Selector::All => {
                parsed.default_elem_spec = Some((*seq, spec.clone()));
            }
            Selector::Ids(ids) => {
                for id in ids {
                    if !parsed.elem_overrides.contains_key(&id) && parsed.elem_overrides.len() >= limits.elements {
                        return Err(ReadError::Limit(format!("element override count exceeds limit {}", limits.elements)));
                    }
                    parsed.elem_overrides.insert(id, (*seq, spec.clone()));
                }
            }
            Selector::List(name) => {
                if let Some(raw) = parsed.lists.get_mut(&name) {
                    match raw {
                        RawList::Geometric { referenced_by_elements, .. } => *referenced_by_elements = true,
                        RawList::Explicit { referenced_by_elements, .. } => *referenced_by_elements = true,
                    }
                }
                parsed.deferred_elem_lists.push(DeferredElemList { list_name: name, seq: *seq, spec: spec.clone(), line });
            }
        }
    }
    Ok(())
}

fn parse_elements_block<F>(
    rec: &Record,
    lex: &mut Lexer<'_>,
    include_loader: &mut F,
    seq: &mut usize,
    limits: ReadLimits,
    member_budget: &mut usize,
    parsed: &mut ParsedDeck,
) -> Result<(), ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    if rec.tokens.len() > 1 {
        let rest = rec.tokens.get(1..).unwrap_or(&[]);
        parse_element_assignment(rest, &rec.file, rec.line, seq, limits, member_budget, parsed)?;
    }
    while let Some(row) = next_record(lex, include_loader)? {
        if is_top_level_directive(&row) {
            lex.push_back(row);
            break;
        }
        parse_element_assignment(&row.tokens, &row.file, row.line, seq, limits, member_budget, parsed)?;
    }
    Ok(())
}

fn parse_coordinates_block<F>(
    rec: &Record,
    lex: &mut Lexer<'_>,
    include_loader: &mut F,
    limits: ReadLimits,
    parsed: &mut ParsedDeck,
) -> Result<(), ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    for tok in rec.tokens.iter().skip(1) {
        if matchs(tok.text.as_str(), "from", 4) || matchs(tok.text.as_str(), "file", 4) || matchs(tok.text.as_str(), "stream", 5) {
            return Err(ReadError::Unsupported {
                file: rec.file.clone(),
                line: rec.line,
                feature: "coordinates from file".into(),
                message: "external/binary `coordinates` streams are not supported".into(),
            });
        } else if matchs(tok.text.as_str(), "clear", 5) {
            parsed.nodes.clear();
        } else {
            return Err(ReadError::Parse {
                file: rec.file.clone(),
                line: rec.line,
                message: format!("unexpected token `{}` on `coordinates` header", tok.text),
            });
        }
    }

    let mut count = 0usize;
    while let Some(row) = next_record(lex, include_loader)? {
        if is_top_level_directive(&row) {
            lex.push_back(row);
            break;
        }
        count += 1;
        if count.is_multiple_of(256) {
            progress::check()?;
        }
        if row.tokens.len() != 4 {
            return Err(ReadError::Parse {
                file: row.file.clone(),
                line: row.line,
                message: format!("expected `<node_id> <x> <y> <z>`, found {} tokens", row.tokens.len()),
            });
        }
        let id = parse_label(row.tok(0).unwrap_or(""), &row.file, row.line)?;
        if !parsed.nodes.contains_key(&id) && parsed.nodes.len() >= limits.nodes {
            return Err(ReadError::Limit(format!("node limit of {} reached", limits.nodes)));
        }
        let x = parse_f64(row.tok(1).unwrap_or("")).ok_or_else(|| ReadError::Parse {
            file: row.file.clone(),
            line: row.line,
            message: format!("invalid x coordinate `{}`", row.tok(1).unwrap_or("")),
        })?;
        let y = parse_f64(row.tok(2).unwrap_or("")).ok_or_else(|| ReadError::Parse {
            file: row.file.clone(),
            line: row.line,
            message: format!("invalid y coordinate `{}`", row.tok(2).unwrap_or("")),
        })?;
        let z = parse_f64(row.tok(3).unwrap_or("")).ok_or_else(|| ReadError::Parse {
            file: row.file.clone(),
            line: row.line,
            message: format!("invalid z coordinate `{}`", row.tok(3).unwrap_or("")),
        })?;
        parsed.nodes.insert(id, ([x, y, z], row.line));
    }
    Ok(())
}

fn parse_incidences_block<F>(
    rec: &Record,
    lex: &mut Lexer<'_>,
    include_loader: &mut F,
    limits: ReadLimits,
    parsed: &mut ParsedDeck,
) -> Result<(), ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    for tok in rec.tokens.iter().skip(1) {
        if matchs(tok.text.as_str(), "from", 4) || matchs(tok.text.as_str(), "file", 4) || matchs(tok.text.as_str(), "stream", 5) {
            return Err(ReadError::Unsupported {
                file: rec.file.clone(),
                line: rec.line,
                feature: "incidences from file".into(),
                message: "external/binary `incidences` streams are not supported".into(),
            });
        } else if matchs(tok.text.as_str(), "clear", 5) {
            parsed.incidences.clear();
        } else {
            return Err(ReadError::Parse {
                file: rec.file.clone(),
                line: rec.line,
                message: format!("unexpected token `{}` on `incidences` header", tok.text),
            });
        }
    }

    let mut count = 0usize;
    while let Some(row) = next_record(lex, include_loader)? {
        if is_top_level_directive(&row) {
            lex.push_back(row);
            break;
        }
        count += 1;
        if count.is_multiple_of(256) {
            progress::check()?;
        }
        if row.tokens.len() < 2 {
            return Err(ReadError::Parse {
                file: row.file.clone(),
                line: row.line,
                message: "element incidence line must contain element ID and node IDs".into(),
            });
        }
        if parsed.incidences.len() >= limits.elements {
            return Err(ReadError::Limit(format!("element limit of {} reached", limits.elements)));
        }
        let eid = parse_label(row.tok(0).unwrap_or(""), &row.file, row.line)?;
        let mut nodes = Vec::with_capacity(row.tokens.len() - 1);
        for tok in row.tokens.iter().skip(1) {
            nodes.push(parse_label(&tok.text, &row.file, row.line)?);
        }
        parsed.incidences.push((eid, nodes, row.line));
    }
    Ok(())
}

fn is_geometric_list_tokens(tokens: &[Token]) -> bool {
    tokens.iter().enumerate().any(|(i, tok)| {
        if tok.quoted {
            return false;
        }
        let s = tok.text.as_str();
        matchs(s, "sphere", 5)
            || matchs(s, "cylinder", 6)
            || matchs(s, "plane", 5)
            || matchs(s, "copy", 4)
            || (matches!(s, "x" | "y" | "z" | "X" | "Y" | "Z") && tokens.get(i + 1).is_some_and(|n| n.text == "="))
    })
}

fn parse_list_block<F>(
    rec: &Record,
    lex: &mut Lexer<'_>,
    include_loader: &mut F,
    member_budget: &mut usize,
    parsed: &mut ParsedDeck,
) -> Result<(), ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    let Some(first) = rec.tok_obj(1) else {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: "missing `list` name or action".into() });
    };
    if !first.quoted && matchs(&first.text, "display", 4) {
        return Ok(());
    }
    if !first.quoted && matchs(&first.text, "delete", 4) {
        if let Some(target) = rec.tok(2) {
            parsed.lists.remove(target);
        }
        return Ok(());
    }

    let name = first.text.clone();
    if parsed.lists.len() >= fieldcraft_model::MAX_DEFINITIONS && !parsed.lists.contains_key(&name) {
        return Err(ReadError::Limit(format!("list count exceeds {}", fieldcraft_model::MAX_DEFINITIONS)));
    }

    let mut rows: Vec<(Vec<Token>, String, usize)> = Vec::new();
    if rec.tokens.len() > 2 {
        rows.push((rec.tokens.get(2..).unwrap_or(&[]).to_vec(), rec.file.clone(), rec.line));
    }
    while let Some(row) = next_record(lex, include_loader)? {
        if is_top_level_directive(&row) {
            lex.push_back(row);
            break;
        }
        rows.push((row.tokens, row.file, row.line));
    }

    if rows.is_empty() {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: format!("`list {name}` has no members") });
    }

    if let Some((first_toks, _, _)) = rows.first()
        && let Some(op) = first_toks.first()
        && !op.quoted
    {
        if matchs(&op.text, "display", 4) {
            return Ok(());
        }
        if matchs(&op.text, "delete", 4) {
            parsed.lists.remove(&name);
            return Ok(());
        }
        if matchs(&op.text, "copy", 4) {
            if let Some(dst_tok) = first_toks.get(1)
                && let Some(src_list) = parsed.lists.get(&name).cloned()
            {
                let dst_name = dst_tok.text.clone();
                if parsed.lists.len() >= fieldcraft_model::MAX_DEFINITIONS && !parsed.lists.contains_key(&dst_name) {
                    return Err(ReadError::Limit(format!("list count exceeds {}", fieldcraft_model::MAX_DEFINITIONS)));
                }
                let (prev_elem, prev_node) = parsed.lists.get(&dst_name).map_or((false, false), |r| match r {
                    RawList::Explicit { referenced_by_elements, referenced_by_nodes, .. }
                    | RawList::Geometric { referenced_by_elements, referenced_by_nodes, .. } => (*referenced_by_elements, *referenced_by_nodes),
                });
                let copied = match src_list {
                    RawList::Explicit { ids, line, .. } => {
                        *member_budget = member_budget.saturating_sub(ids.len());
                        RawList::Explicit { ids, line, referenced_by_elements: prev_elem, referenced_by_nodes: prev_node }
                    }
                    RawList::Geometric { line, .. } => RawList::Geometric { line, referenced_by_elements: prev_elem, referenced_by_nodes: prev_node },
                };
                parsed.lists.insert(dst_name, copied);
            }
            return Ok(());
        }
    }

    if rows.iter().any(|(toks, _, _)| is_geometric_list_tokens(toks)) {
        let (prev_elem, prev_node) = parsed.lists.get(&name).map_or((false, false), |r| match r {
            RawList::Explicit { referenced_by_elements, referenced_by_nodes, .. }
            | RawList::Geometric { referenced_by_elements, referenced_by_nodes, .. } => (*referenced_by_elements, *referenced_by_nodes),
        });
        parsed.lists.insert(name, RawList::Geometric { line: rec.line, referenced_by_elements: prev_elem, referenced_by_nodes: prev_node });
        return Ok(());
    }

    let mut ids = Vec::new();
    for (toks, file, line) in rows {
        let mut idx = 0usize;
        while idx < toks.len() {
            let (range, next_idx) = parse_id_or_range(&toks, idx, &file, line, *member_budget)?;
            *member_budget = member_budget.saturating_sub(range.count as usize);
            expand_range_into(&range, &mut ids)?;
            idx = next_idx;
        }
    }
    let (prev_elem, prev_node) = parsed.lists.get(&name).map_or((false, false), |r| match r {
        RawList::Explicit { referenced_by_elements, referenced_by_nodes, .. }
        | RawList::Geometric { referenced_by_elements, referenced_by_nodes, .. } => (*referenced_by_elements, *referenced_by_nodes),
    });
    parsed.lists.insert(name, RawList::Explicit { ids, line: rec.line, referenced_by_elements: prev_elem, referenced_by_nodes: prev_node });
    Ok(())
}

fn parse_id_selectors(
    tokens: &[Token],
    start: usize,
    file: &str,
    line: usize,
    max_entities: usize,
    member_budget: &mut usize,
    is_stop_token: fn(&str) -> bool,
    lists: &mut BTreeMap<String, RawList>,
    mark_as_elements: bool,
) -> Result<(Vec<IdSelector>, usize), ReadError> {
    let mut idx = start;
    let mut selectors = Vec::new();
    while let Some(tok_obj) = tokens.get(idx) {
        let tok = tok_obj.text.as_str();
        if !tok_obj.quoted && is_stop_token(tok) {
            break;
        }
        if !tok_obj.quoted && matchs_exact(tok, "all") {
            selectors.push(IdSelector::All);
            idx += 1;
        } else if !tok_obj.quoted && tok.bytes().next().is_some_and(|b| b.is_ascii_digit()) {
            let (range, next_idx) = parse_id_or_range(tokens, idx, file, line, *member_budget)?;
            if range.count > max_entities as u64 {
                return Err(ReadError::Limit(format!("selector range ({}) exceeds limit {max_entities}", range.count)));
            }
            *member_budget = member_budget.saturating_sub(range.count as usize);
            let mut ids = Vec::with_capacity((range.count as usize).min(4096));
            expand_range_into(&range, &mut ids)?;
            selectors.push(IdSelector::Ids(ids));
            idx = next_idx;
        } else {
            if let Some(raw) = lists.get_mut(tok) {
                match raw {
                    RawList::Explicit { referenced_by_elements, referenced_by_nodes, .. }
                    | RawList::Geometric { referenced_by_elements, referenced_by_nodes, .. } => {
                        if mark_as_elements {
                            *referenced_by_elements = true;
                        } else {
                            *referenced_by_nodes = true;
                        }
                    }
                }
            }
            selectors.push(IdSelector::List(tok.to_string()));
            idx += 1;
        }
    }
    Ok((selectors, idx))
}

fn is_constraint_dof_tok(tok: &str) -> bool {
    matchs_exact(tok, "u") || matchs_exact(tok, "v") || matchs_exact(tok, "w") || matchs(tok, "fixed", 3)
}

fn parse_plane_constraint(tokens: &[Token], file: &str, line: usize) -> Result<RawPlaneConstraint, ReadError> {
    let mut axis: Option<usize> = None;
    let mut value: f64 = 0.0;
    let mut proximity: Option<f64> = None;
    let mut action: Option<PlaneAction> = None;

    let mut idx = 1usize;
    while let Some(tok_obj) = tokens.get(idx) {
        if tok_obj.quoted {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("unexpected quoted token `{}` in `plane` constraint", tok_obj.text),
            });
        }
        let tok = tok_obj.text.as_str();
        if matchs_exact(tok, "x") || matchs_exact(tok, "y") || matchs_exact(tok, "z") {
            let ax = if matchs_exact(tok, "x") {
                0
            } else if matchs_exact(tok, "y") {
                1
            } else {
                2
            };
            if axis.is_some() {
                return Err(ReadError::Parse { file: file.to_string(), line, message: "duplicate coordinate axis in `plane` constraint".into() });
            }
            axis = Some(ax);
            idx += 1;
            if tokens.get(idx).is_some_and(|t| !t.quoted && t.text == "=") {
                idx += 1;
                let Some(v_tok) = tokens.get(idx) else {
                    return Err(ReadError::Parse { file: file.to_string(), line, message: "missing plane coordinate after `=`".into() });
                };
                value = parse_f64(&v_tok.text).ok_or_else(|| ReadError::Parse {
                    file: file.to_string(),
                    line,
                    message: format!("invalid plane coordinate `{}`", v_tok.text),
                })?;
                idx += 1;
            } else if let Some(v_tok) = tokens.get(idx)
                && !v_tok.quoted
                && let Some(v) = parse_f64(&v_tok.text)
            {
                value = v;
                idx += 1;
            }
        } else if matchs(tok, "proximity", 4) {
            if proximity.is_some() {
                return Err(ReadError::Parse { file: file.to_string(), line, message: "duplicate `proximity` in `plane` constraint".into() });
            }
            idx += 1;
            let val_tok = next_value_tok(tokens, &mut idx, file, line, "proximity")?;
            let dist = parse_f64(val_tok).ok_or_else(|| ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("invalid `proximity` value `{val_tok}`"),
            })?;
            if dist < 0.0 {
                return Err(ReadError::Parse { file: file.to_string(), line, message: format!("`proximity` must be >= 0, got {dist}") });
            }
            proximity = Some(dist);
        } else if matchs(tok, "verify", 3) {
            idx += 1;
        } else if matchs(tok, "fixed", 3) {
            if action.is_some() {
                return Err(ReadError::Parse { file: file.to_string(), line, message: "conflicting actions in `plane` constraint".into() });
            }
            action = Some(PlaneAction::Fixed);
            idx += 1;
        } else if matchs(tok, "symmetry", 3) {
            if action.is_some() {
                return Err(ReadError::Parse { file: file.to_string(), line, message: "conflicting actions in `plane` constraint".into() });
            }
            action = Some(PlaneAction::Symmetry);
            idx += 1;
        } else if matchs_exact(tok, "u") || matchs_exact(tok, "v") || matchs_exact(tok, "w") {
            let dof = if matchs_exact(tok, "u") {
                0
            } else if matchs_exact(tok, "v") {
                1
            } else {
                2
            };
            idx += 1;
            let val_tok = next_value_tok(tokens, &mut idx, file, line, tok)?;
            let val = parse_f64(val_tok).ok_or_else(|| ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("invalid `{tok}` displacement `{val_tok}`"),
            })?;
            match &mut action {
                None => {
                    let mut arr = [None; 3];
                    if let Some(slot) = arr.get_mut(dof) {
                        *slot = Some(val);
                    }
                    action = Some(PlaneAction::Dofs(arr));
                }
                Some(PlaneAction::Dofs(arr)) => {
                    if let Some(slot) = arr.get_mut(dof) {
                        if let Some(prev) = *slot
                            && (prev - val).abs() > 1e-15
                        {
                            return Err(ReadError::Parse {
                                file: file.to_string(),
                                line,
                                message: format!("conflicting `{tok}` values in `plane` constraint"),
                            });
                        }
                        *slot = Some(val);
                    }
                }
                Some(_) => {
                    return Err(ReadError::Parse { file: file.to_string(), line, message: "conflicting actions in `plane` constraint".into() });
                }
            }
        } else {
            return Err(ReadError::Parse { file: file.to_string(), line, message: format!("unrecognized token `{tok}` in `plane` constraint") });
        }
    }

    let Some(axis) = axis else {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "`plane` constraint requires `x`, `y`, or `z`".into() });
    };
    let Some(action) = action else {
        return Err(ReadError::Parse {
            file: file.to_string(),
            line,
            message: "`plane` constraint requires `fixed`, `symmetry`, or `u`/`v`/`w`".into(),
        });
    };
    Ok(RawPlaneConstraint { axis, value, proximity, action, file: file.to_string(), line })
}

fn parse_nodal_constraint_row(
    tokens: &[Token],
    file: &str,
    line: usize,
    limits: ReadLimits,
    member_budget: &mut usize,
    parsed: &mut ParsedDeck,
) -> Result<(), ReadError> {
    let (selectors, mut idx) =
        parse_id_selectors(tokens, 0, file, line, limits.nodes, member_budget, is_constraint_dof_tok, &mut parsed.lists, false)?;
    if selectors.is_empty() {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "missing node selector in `constraints` row".into() });
    }
    let mut values: [Option<f64>; 3] = [None; 3];
    let mut any_dof = false;
    while let Some(tok_obj) = tokens.get(idx) {
        if tok_obj.quoted {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("unexpected quoted token `{}` in `constraints` row", tok_obj.text),
            });
        }
        let tok = tok_obj.text.as_str();
        if matchs(tok, "fixed", 3) {
            idx += 1;
            any_dof = true;
            for slot in &mut values {
                if let Some(prev) = *slot
                    && prev.abs() > 1e-15
                {
                    return Err(ReadError::Parse {
                        file: file.to_string(),
                        line,
                        message: "conflicting `fixed` and non-zero DOF value on `constraints` row".into(),
                    });
                }
                *slot = Some(0.0);
            }
        } else if matchs_exact(tok, "u") || matchs_exact(tok, "v") || matchs_exact(tok, "w") {
            let dof = if matchs_exact(tok, "u") {
                0
            } else if matchs_exact(tok, "v") {
                1
            } else {
                2
            };
            idx += 1;
            let val_tok = next_value_tok(tokens, &mut idx, file, line, tok)?;
            let val = parse_f64(val_tok).ok_or_else(|| ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("invalid constraint displacement `{val_tok}`"),
            })?;
            any_dof = true;
            if let Some(slot) = values.get_mut(dof) {
                if let Some(prev) = *slot
                    && (prev - val).abs() > 1e-15
                {
                    return Err(ReadError::Parse {
                        file: file.to_string(),
                        line,
                        message: format!("conflicting `{tok}` values on `constraints` row"),
                    });
                }
                *slot = Some(val);
            }
        } else {
            return Err(ReadError::Parse { file: file.to_string(), line, message: format!("unexpected token `{tok}` in `constraints` row") });
        }
    }
    if !any_dof {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "`constraints` row requires `u`, `v`, `w`, or `fixed`".into() });
    }
    parsed.nodal_constraints.push(RawNodalConstraint { selectors, values, file: file.to_string(), line });
    Ok(())
}

fn parse_multipoint_row(tokens: &[Token], file: &str, line: usize, parsed: &mut ParsedDeck) -> Result<(), ReadError> {
    if tokens.iter().any(|t| !t.quoted && matchs_exact(&t.text, "rigid")) {
        return Err(ReadError::Unsupported {
            file: file.to_string(),
            line,
            feature: "rigid multipoint constraint".into(),
            message: "`multipoint` `rigid` constraints are not supported".into(),
        });
    }

    let mut idx = 0usize;
    let mut raw_terms: Vec<(u32, usize, f64)> = Vec::new();
    let mut distinct_nodes = BTreeSet::new();
    let mut first_term = true;

    loop {
        let Some(tok_obj) = tokens.get(idx) else {
            return Err(ReadError::Parse { file: file.to_string(), line, message: "incomplete `multipoint` equation (missing `= <rhs>`)".into() });
        };
        if !tok_obj.quoted && tok_obj.text == "=" {
            if raw_terms.is_empty() {
                return Err(ReadError::Parse { file: file.to_string(), line, message: "`multipoint` equation requires at least one term".into() });
            }
            idx += 1;
            break;
        }

        let mut sign = 1.0_f64;
        let mut consumed_sign_tok = false;
        if !tok_obj.quoted && tok_obj.text == "+" {
            idx += 1;
            consumed_sign_tok = true;
        } else if !tok_obj.quoted && tok_obj.text == "-" {
            sign = -1.0;
            idx += 1;
            consumed_sign_tok = true;
        } else if !first_term && (tok_obj.quoted || (!tok_obj.text.starts_with('+') && !tok_obj.text.starts_with('-'))) {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("expected `+`, `-`, or `=` in `multipoint` equation, found `{}`", tok_obj.text),
            });
        }

        let Some(node_tok) = tokens.get(idx) else {
            return Err(ReadError::Parse { file: file.to_string(), line, message: "missing node ID in `multipoint` term".into() });
        };
        if node_tok.quoted {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("unexpected quoted token `{}` in `multipoint` equation", node_tok.text),
            });
        }
        let node_str = if !consumed_sign_tok {
            if let Some(rest) = node_tok.text.strip_prefix('+') {
                rest
            } else if let Some(rest) = node_tok.text.strip_prefix('-') {
                sign = -1.0;
                rest
            } else {
                node_tok.text.as_str()
            }
        } else {
            node_tok.text.as_str()
        };
        let node = parse_label(node_str, file, line)?;
        idx += 1;

        let Some(coeff_tok) = tokens.get(idx) else {
            return Err(ReadError::Parse { file: file.to_string(), line, message: "missing coefficient in `multipoint` term".into() });
        };
        let coeff = parse_f64(&coeff_tok.text).ok_or_else(|| ReadError::Parse {
            file: file.to_string(),
            line,
            message: format!("invalid `multipoint` coefficient `{}`", coeff_tok.text),
        })?;
        idx += 1;

        let Some(dof_tok) = tokens.get(idx) else {
            return Err(ReadError::Parse { file: file.to_string(), line, message: "missing DOF (`u`, `v`, or `w`) in `multipoint` term".into() });
        };
        let axis = if !dof_tok.quoted && matchs_exact(&dof_tok.text, "u") {
            0
        } else if !dof_tok.quoted && matchs_exact(&dof_tok.text, "v") {
            1
        } else if !dof_tok.quoted && matchs_exact(&dof_tok.text, "w") {
            2
        } else {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("expected `u`, `v`, or `w` in `multipoint` term, found `{}`", dof_tok.text),
            });
        };
        idx += 1;

        let signed_coeff = sign * coeff;
        if !signed_coeff.is_finite() {
            return Err(ReadError::Parse { file: file.to_string(), line, message: "non-finite `multipoint` coefficient".into() });
        }
        distinct_nodes.insert(node);
        raw_terms.push((node, axis, signed_coeff));
        first_term = false;
    }

    let (rhs_sign, consumed_rhs_sign) = if tokens.get(idx).is_some_and(|t| !t.quoted && t.text == "-") {
        idx += 1;
        (-1.0, true)
    } else if tokens.get(idx).is_some_and(|t| !t.quoted && t.text == "+") {
        idx += 1;
        (1.0, true)
    } else {
        (1.0, false)
    };
    let Some(rhs_tok) = tokens.get(idx) else {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "missing right-hand side after `=` in `multipoint`".into() });
    };
    if consumed_rhs_sign && (rhs_tok.text.starts_with('+') || rhs_tok.text.starts_with('-')) {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "invalid sign in `multipoint` RHS".into() });
    }
    let rhs = rhs_sign
        * parse_f64(&rhs_tok.text).ok_or_else(|| ReadError::Parse {
            file: file.to_string(),
            line,
            message: format!("invalid `multipoint` RHS `{}`", rhs_tok.text),
        })?;
    idx += 1;
    if let Some(extra) = tokens.get(idx) {
        return Err(ReadError::Parse {
            file: file.to_string(),
            line,
            message: format!("unexpected trailing token `{}` after `multipoint` RHS", extra.text),
        });
    }

    if distinct_nodes.len() > 1 {
        return Err(ReadError::Unsupported {
            file: file.to_string(),
            line,
            feature: "multi-node multipoint constraint".into(),
            message: "multi-node `multipoint` equations are not supported".into(),
        });
    }
    if rhs.abs() > 1e-15 {
        return Err(ReadError::Unsupported {
            file: file.to_string(),
            line,
            feature: "non-zero RHS multipoint constraint".into(),
            message: "`multipoint` equations with non-zero right-hand side are not supported".into(),
        });
    }

    let Some(&node) = distinct_nodes.iter().next() else {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "`multipoint` equation has no node".into() });
    };
    let terms = raw_terms.into_iter().map(|(_, axis, c)| (axis, c)).collect();
    parsed.multipoints.push(RawMultipoint { node, terms, file: file.to_string(), line });
    Ok(())
}

fn parse_constraints_block<F>(
    rec: &Record,
    lex: &mut Lexer<'_>,
    include_loader: &mut F,
    limits: ReadLimits,
    member_budget: &mut usize,
    lex_limits: LexLimits,
    parsed: &mut ParsedDeck,
) -> Result<(), ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    if rec.tokens.len() > 1 {
        let extra = rec.tok(1).unwrap_or("");
        return Err(ReadError::Parse {
            file: rec.file.clone(),
            line: rec.line,
            message: format!("unexpected token `{extra}` on `constraints` header"),
        });
    }

    let mut in_multipoint = false;
    while let Some(row) = next_record(lex, include_loader)? {
        if is_top_level_directive(&row) {
            lex.push_back(row);
            break;
        }
        let Some(t0_obj) = row.tok_obj(0) else {
            continue;
        };
        if !in_multipoint && !t0_obj.quoted && matchs(&t0_obj.text, "transformation", 5) {
            return Err(ReadError::Unsupported {
                file: row.file,
                line: row.line,
                feature: "transformation matrix".into(),
                message: "`transformation matrix` skew nodal frames are not supported".into(),
            });
        }
        if !t0_obj.quoted && matchs(&t0_obj.text, "multipoint", 5) {
            in_multipoint = true;
            if row.tokens.len() > 1 {
                parse_multipoint_row(row.tokens.get(1..).unwrap_or(&[]), &row.file, row.line, parsed)?;
            }
            continue;
        }
        if in_multipoint {
            parse_multipoint_row(&row.tokens, &row.file, row.line, parsed)?;
            continue;
        }
        if !t0_obj.quoted && matchs(&t0_obj.text, "plane", 4) {
            if parsed.deferred_planes.len() >= lex_limits.max_deferred_planes {
                return Err(ReadError::Limit(format!("`plane` constraint count exceeds {} limit", lex_limits.max_deferred_planes)));
            }
            let plane = parse_plane_constraint(&row.tokens, &row.file, row.line)?;
            parsed.deferred_planes.push(plane);
            continue;
        }
        parse_nodal_constraint_row(&row.tokens, &row.file, row.line, limits, member_budget, parsed)?;
    }
    Ok(())
}

fn is_nodal_load_attr(tok: &str) -> bool {
    matchs(tok, "force_x", 7)
        || matchs(tok, "force_y", 7)
        || matchs(tok, "force_z", 7)
        || matchs(tok, "temperature", 4)
        || matchs(tok, "user_routine", 4)
}

fn is_elem_load_clause(tok: &str) -> bool {
    matchs(tok, "face", 4) || matchs(tok, "body", 4) || matchs(tok, "temperature", 6) || matchs(tok, "user_routine", 4)
}

fn is_unsupported_face_modifier(tok: &str) -> bool {
    matchs(tok, "piston", 6)
        || matchs(tok, "local", 5)
        || matchs(tok, "projected", 4)
        || matchs(tok, "follower", 4)
        || matchs(tok, "user_routine", 4)
        || matchs_exact(tok, "tn")
        || matchs_exact(tok, "ts")
        || matchs_exact(tok, "tt")
}

fn parse_nodal_load_row(
    pattern: &str,
    tokens: &[Token],
    file: &str,
    line: usize,
    limits: ReadLimits,
    member_budget: &mut usize,
    parsed: &mut ParsedDeck,
) -> Result<(), ReadError> {
    let (selectors, mut idx) = parse_id_selectors(tokens, 0, file, line, limits.nodes, member_budget, is_nodal_load_attr, &mut parsed.lists, false)?;
    if selectors.is_empty() {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "missing node selector in `nodal loads` row".into() });
    }
    let mut force = [0.0; 3];
    let mut saw_force = false;
    let mut saw_any = false;

    while let Some(tok_obj) = tokens.get(idx) {
        if tok_obj.quoted {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("unexpected quoted token `{}` in `nodal loads` row", tok_obj.text),
            });
        }
        let tok = tok_obj.text.as_str();
        if matchs(tok, "user_routine", 4) {
            return Err(ReadError::Unsupported {
                file: file.to_string(),
                line,
                feature: "user_routine load".into(),
                message: "`user_routine` nodal loads are not supported".into(),
            });
        }
        if matchs(tok, "temperature", 4) {
            idx += 1;
            let val_tok = next_value_tok(tokens, &mut idx, file, line, "temperature")?;
            let _ = parse_f64(val_tok).ok_or_else(|| ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("invalid nodal temperature `{val_tok}`"),
            })?;
            parsed.saw_temperature_load = true;
            saw_any = true;
        } else if matchs(tok, "force_x", 7) || matchs(tok, "force_y", 7) || matchs(tok, "force_z", 7) {
            let axis = if matchs(tok, "force_x", 7) {
                0
            } else if matchs(tok, "force_y", 7) {
                1
            } else {
                2
            };
            idx += 1;
            let val_tok = next_value_tok(tokens, &mut idx, file, line, tok)?;
            let val = parse_f64(val_tok).ok_or_else(|| ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("invalid nodal force `{val_tok}`"),
            })?;
            if let Some(slot) = force.get_mut(axis) {
                *slot += val;
            }
            saw_force = true;
            saw_any = true;
        } else {
            return Err(ReadError::Parse { file: file.to_string(), line, message: format!("unrecognized token `{tok}` in `nodal loads`") });
        }
    }
    if !saw_any {
        return Err(ReadError::Parse {
            file: file.to_string(),
            line,
            message: "`nodal loads` row requires `force_x`/`force_y`/`force_z` or `temperature`".into(),
        });
    }
    if saw_force {
        parsed.nodal_loads.push(RawNodalLoad { pattern: pattern.to_string(), selectors, force, file: file.to_string(), line });
    }
    Ok(())
}

fn parse_element_load_row(
    pattern: &str,
    tokens: &[Token],
    file: &str,
    line: usize,
    limits: ReadLimits,
    member_budget: &mut usize,
    parsed: &mut ParsedDeck,
) -> Result<(), ReadError> {
    let (selectors, mut idx) =
        parse_id_selectors(tokens, 0, file, line, limits.elements, member_budget, is_elem_load_clause, &mut parsed.lists, true)?;
    if selectors.is_empty() {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "missing element selector in `element loads` row".into() });
    }

    let mut saw_any_clause = false;
    while let Some(tok_obj) = tokens.get(idx) {
        if tok_obj.quoted {
            return Err(ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("unexpected quoted token `{}` in `element loads` row", tok_obj.text),
            });
        }
        let tok = tok_obj.text.as_str();
        if matchs(tok, "user_routine", 4) {
            return Err(ReadError::Unsupported {
                file: file.to_string(),
                line,
                feature: "user_routine load".into(),
                message: "`user_routine` element loads are not supported".into(),
            });
        }
        if matchs(tok, "temperature", 6) {
            idx += 1;
            if tokens.get(idx).is_some_and(|t| !t.quoted && t.text == "=") {
                idx += 1;
            }
            let mut count = 0usize;
            while let Some(v_tok) = tokens.get(idx) {
                if !v_tok.quoted && is_elem_load_clause(&v_tok.text) {
                    break;
                }
                let _ = parse_f64(&v_tok.text).ok_or_else(|| ReadError::Parse {
                    file: file.to_string(),
                    line,
                    message: format!("invalid element temperature `{}`", v_tok.text),
                })?;
                count += 1;
                idx += 1;
            }
            if count == 0 {
                return Err(ReadError::Parse { file: file.to_string(), line, message: "missing temperature value on `element loads` row".into() });
            }
            parsed.saw_temperature_load = true;
            saw_any_clause = true;
        } else if matchs(tok, "body", 4) {
            idx += 1;
            if tokens.get(idx).is_some_and(|t| !t.quoted && matchs(&t.text, "forces", 5)) {
                idx += 1;
            }
            let mut body = [0.0; 3];
            let mut saw_b = false;
            while let Some(b_tok) = tokens.get(idx) {
                if b_tok.quoted {
                    break;
                }
                let bt = b_tok.text.as_str();
                if matchs(bt, "user_routine", 4) {
                    return Err(ReadError::Unsupported {
                        file: file.to_string(),
                        line,
                        feature: "user_routine body force".into(),
                        message: "`user_routine` body forces are not supported".into(),
                    });
                }
                let axis = if matchs_exact(bt, "bx") {
                    0
                } else if matchs_exact(bt, "by") {
                    1
                } else if matchs_exact(bt, "bz") {
                    2
                } else {
                    break;
                };
                idx += 1;
                let val_tok = next_value_tok(tokens, &mut idx, file, line, bt)?;
                let val = parse_f64(val_tok).ok_or_else(|| ReadError::Parse {
                    file: file.to_string(),
                    line,
                    message: format!("invalid `{bt}` body force `{val_tok}`"),
                })?;
                if let Some(slot) = body.get_mut(axis) {
                    *slot += val;
                }
                saw_b = true;
            }
            if !saw_b {
                return Err(ReadError::Parse { file: file.to_string(), line, message: "`body` load requires `bx`, `by`, or `bz`".into() });
            }
            parsed.elem_bodies.push(RawElementBody {
                pattern: pattern.to_string(),
                selectors: selectors.clone(),
                body,
                file: file.to_string(),
                line,
            });
            saw_any_clause = true;
        } else if matchs(tok, "face", 4) {
            idx += 1;
            let is_abaqus = if tokens.get(idx).is_some_and(|t| !t.quoted && matchs(&t.text, "abaqus", 4)) {
                idx += 1;
                true
            } else {
                false
            };
            let Some(face_tok) = tokens.get(idx) else {
                return Err(ReadError::Parse { file: file.to_string(), line, message: "missing face number after `face`".into() });
            };
            let face = face_tok.text.parse::<u8>().ok().filter(|&f| (1..=6).contains(&f)).ok_or_else(|| ReadError::Parse {
                file: file.to_string(),
                line,
                message: format!("invalid face number `{}` (expected 1..=6)", face_tok.text),
            })?;
            idx += 1;

            if tokens.get(idx).is_some_and(|t| !t.quoted && (matchs(&t.text, "tractions", 5) || matchs_exact(&t.text, "force"))) {
                idx += 1;
            }

            let mut pressure: Option<f64> = None;
            let mut traction = [0.0; 3];
            let mut saw_traction = false;

            while let Some(sub_tok) = tokens.get(idx) {
                if sub_tok.quoted {
                    return Err(ReadError::Parse {
                        file: file.to_string(),
                        line,
                        message: format!("unexpected quoted token `{}` in `face` load", sub_tok.text),
                    });
                }
                let st = sub_tok.text.as_str();
                if is_elem_load_clause(st) {
                    break;
                }
                if is_unsupported_face_modifier(st) {
                    return Err(ReadError::Unsupported {
                        file: file.to_string(),
                        line,
                        feature: format!("face load `{st}`"),
                        message: format!("element face load modifier `{st}` is not supported"),
                    });
                }
                if matchs(st, "pressure", 5) {
                    idx += 1;
                    let val_tok = next_value_tok(tokens, &mut idx, file, line, "pressure")?;
                    let p = parse_f64(val_tok).ok_or_else(|| ReadError::Parse {
                        file: file.to_string(),
                        line,
                        message: format!("invalid face pressure `{val_tok}`"),
                    })?;
                    if let Some(next_tok) = tokens.get(idx)
                        && !next_tok.quoted
                        && parse_f64(&next_tok.text).is_some()
                    {
                        return Err(ReadError::Unsupported {
                            file: file.to_string(),
                            line,
                            feature: "non-uniform face pressure".into(),
                            message: "multi-value nodal pressure on an element face is not supported".into(),
                        });
                    }
                    pressure = Some(pressure.unwrap_or(0.0) + p);
                } else if matchs_exact(st, "tx") || matchs_exact(st, "ty") || matchs_exact(st, "tz") {
                    let axis = if matchs_exact(st, "tx") {
                        0
                    } else if matchs_exact(st, "ty") {
                        1
                    } else {
                        2
                    };
                    idx += 1;
                    let val_tok = next_value_tok(tokens, &mut idx, file, line, st)?;
                    let val = parse_f64(val_tok).ok_or_else(|| ReadError::Parse {
                        file: file.to_string(),
                        line,
                        message: format!("invalid traction `{val_tok}`"),
                    })?;
                    if let Some(slot) = traction.get_mut(axis) {
                        *slot += val;
                    }
                    saw_traction = true;
                } else {
                    return Err(ReadError::Parse { file: file.to_string(), line, message: format!("unrecognized token `{st}` in `face` load") });
                }
            }
            if pressure.is_none() && !saw_traction {
                return Err(ReadError::Parse { file: file.to_string(), line, message: "`face` load requires `pressure` or `tx`/`ty`/`tz`".into() });
            }
            if let Some(p) = pressure {
                parsed.elem_pressures.push(RawElementPressure {
                    pattern: pattern.to_string(),
                    selectors: selectors.clone(),
                    face,
                    is_abaqus,
                    pressure: p,
                    file: file.to_string(),
                    line,
                });
            }
            if saw_traction {
                parsed.elem_tractions.push(RawElementTraction {
                    pattern: pattern.to_string(),
                    selectors: selectors.clone(),
                    face,
                    is_abaqus,
                    traction,
                    file: file.to_string(),
                    line,
                });
            }
            saw_any_clause = true;
        } else {
            return Err(ReadError::Parse { file: file.to_string(), line, message: format!("unrecognized token `{tok}` in `element loads`") });
        }
    }
    if !saw_any_clause {
        return Err(ReadError::Parse { file: file.to_string(), line, message: "`element loads` row has no load specification".into() });
    }
    Ok(())
}

fn parse_loading_block<F>(
    rec: &Record,
    lex: &mut Lexer<'_>,
    include_loader: &mut F,
    limits: ReadLimits,
    member_budget: &mut usize,
    parsed: &mut ParsedDeck,
) -> Result<(), ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    let Some(name_tok) = rec.tok(1) else {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: "missing load pattern name after `loading`".into() });
    };
    if let Some(extra) = rec.tok(2) {
        return Err(ReadError::Parse { file: rec.file.clone(), line: rec.line, message: format!("unexpected token `{extra}` on `loading` header") });
    }
    let pattern = name_tok.to_string();

    #[derive(Clone, Copy, PartialEq, Eq)]
    enum LoadMode {
        Unknown,
        Nodal,
        Element,
    }
    let mut mode = LoadMode::Unknown;

    while let Some(row) = next_record(lex, include_loader)? {
        if is_top_level_directive(&row) {
            lex.push_back(row);
            break;
        }
        let Some(t0_obj) = row.tok_obj(0) else {
            continue;
        };
        if !t0_obj.quoted && (matchs(&t0_obj.text, "nonlinear", 9) || matchs(&t0_obj.text, "dynamic", 7) || matchs(&t0_obj.text, "step", 4)) {
            skip_block(lex, include_loader, parsed, "loading", rec, SkipMode::General)?;
            break;
        }
        if !t0_obj.quoted
            && matchs(&t0_obj.text, "nodal", 5)
            && row.tok_obj(1).is_some_and(|t1| !t1.quoted && (matchs(&t1.text, "loads", 4) || matchs(&t1.text, "forces", 5)))
        {
            mode = LoadMode::Nodal;
            continue;
        }
        if !t0_obj.quoted && matchs(&t0_obj.text, "element", 7) && row.tok_obj(1).is_some_and(|t1| !t1.quoted && matchs(&t1.text, "loads", 4)) {
            mode = LoadMode::Element;
            continue;
        }
        match mode {
            LoadMode::Unknown => {
                return Err(ReadError::Parse {
                    file: row.file,
                    line: row.line,
                    message: format!("expected `nodal loads`, `element loads`, `nonlinear`, or `dynamic` inside `loading {pattern}`"),
                });
            }
            LoadMode::Nodal => {
                parse_nodal_load_row(&pattern, &row.tokens, &row.file, row.line, limits, member_budget, parsed)?;
            }
            LoadMode::Element => {
                parse_element_load_row(&pattern, &row.tokens, &row.file, row.line, limits, member_budget, parsed)?;
            }
        }
    }
    Ok(())
}

pub(crate) fn parse_deck<F>(
    root_name: &str,
    bytes: &[u8],
    limits: ReadLimits,
    lex_limits: LexLimits,
    mut include_loader: F,
) -> Result<ParsedDeck, ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    let mut lex = Lexer::new(root_name, bytes, lex_limits);
    let mut parsed = ParsedDeck {
        declared_nodes: None,
        declared_elements: None,
        nodes: BTreeMap::new(),
        incidences: Vec::new(),
        default_elem_spec: None,
        elem_overrides: BTreeMap::new(),
        deferred_elem_lists: Vec::new(),
        curves: BTreeMap::new(),
        materials: BTreeMap::new(),
        lists: BTreeMap::new(),
        nodal_constraints: Vec::new(),
        deferred_planes: Vec::new(),
        multipoints: Vec::new(),
        nodal_loads: Vec::new(),
        elem_pressures: Vec::new(),
        elem_tractions: Vec::new(),
        elem_bodies: Vec::new(),
        saw_temperature_load: false,
        unknown: Vec::new(),
        unknown_count: 0,
        warnings: Vec::new(),
        stopped_at: None,
        includes_read: Vec::new(),
    };

    let mut elem_seq = 0usize;
    let mut member_budget = limits.members;

    while let Some(rec) = next_record(&mut lex, &mut include_loader)? {
        let Some(t0_obj) = rec.tok_obj(0) else {
            continue;
        };
        if t0_obj.quoted {
            let msg = format!("unexpected quoted token `{}`", t0_obj.text);
            return Err(ReadError::Parse { file: rec.file, line: rec.line, message: msg });
        }
        let t0 = t0_obj.text.as_str();
        let t1 = rec.tok(1);

        if matchs(t0, "stop", 4) || matchs(t0, "exit", 4) || matchs(t0, "quit", 4) {
            break;
        }

        if matchs(t0, "compute", 4) {
            record_unknown(&mut parsed, "compute", &rec);
            parsed.stopped_at = Some(StoppedAt { file: rec.file.clone(), line: rec.line, reason: "compute".into() });
            parsed.warnings.push(format!(
                "stopped reading at `compute` ({}:{}); post-compute commands and analysis history were not imported",
                rec.file, rec.line
            ));
            break;
        }

        if matchs(t0, "structure", 9) {
            continue;
        }
        if matchs(t0, "number", 3) {
            parse_number_directive(&rec, &mut parsed)?;
            continue;
        }
        if is_stress_strain_header(&rec) {
            parse_stress_strain_curve(&rec, &mut lex, &mut include_loader, &mut parsed)?;
            continue;
        }
        if matchs(t0, "material", 8) {
            parse_material_block(&rec, &mut lex, &mut include_loader, &mut parsed)?;
            continue;
        }
        if matchs(t0, "elements", 8) {
            parse_elements_block(&rec, &mut lex, &mut include_loader, &mut elem_seq, limits, &mut member_budget, &mut parsed)?;
            continue;
        }
        if matchs(t0, "coordinates", 5) {
            parse_coordinates_block(&rec, &mut lex, &mut include_loader, limits, &mut parsed)?;
            continue;
        }
        if matchs(t0, "incidences", 5) {
            parse_incidences_block(&rec, &mut lex, &mut include_loader, limits, &mut parsed)?;
            continue;
        }
        if matchs(t0, "list", 4) {
            parse_list_block(&rec, &mut lex, &mut include_loader, &mut member_budget, &mut parsed)?;
            continue;
        }
        if matchs(t0, "constraints", 11) {
            parse_constraints_block(&rec, &mut lex, &mut include_loader, limits, &mut member_budget, lex_limits, &mut parsed)?;
            continue;
        }
        if matchs(t0, "loading", 7) {
            parse_loading_block(&rec, &mut lex, &mut include_loader, limits, &mut member_budget, &mut parsed)?;
            continue;
        }

        // Unread-recorded blocks.
        if matchs(t0, "blocking", 5) {
            skip_block(&mut lex, &mut include_loader, &mut parsed, "blocking", &rec, SkipMode::General)?;
            continue;
        }
        if (matchs(t0, "nonlinear", 7) || matchs(t0, "dynamic", 7) || matchs(t0, "solution", 4))
            && t1.is_some_and(|t| matchs(t, "analysis", 5) || matchs(t, "parameters", 5))
        {
            skip_block(&mut lex, &mut include_loader, &mut parsed, &format!("{t0} {}", t1.unwrap_or("")), &rec, SkipMode::Parameters)?;
            continue;
        }
        if (matchs(t0, "analysis", 5) && t1.is_some_and(|t| matchs(t, "parameters", 5))) || matchs(t0, "parameters", 5) {
            skip_block(&mut lex, &mut include_loader, &mut parsed, t0, &rec, SkipMode::Parameters)?;
            continue;
        }
        if matchs(t0, "output", 6) {
            skip_block(&mut lex, &mut include_loader, &mut parsed, "output", &rec, SkipMode::General)?;
            continue;
        }
        if matchs(t0, "save", 4) {
            skip_block(&mut lex, &mut include_loader, &mut parsed, "save", &rec, SkipMode::General)?;
            continue;
        }
        if matchs(t0, "domain", 5) {
            skip_block(&mut lex, &mut include_loader, &mut parsed, "domain", &rec, SkipMode::General)?;
            continue;
        }

        // Explicit Bucket B unsupported directives.
        if matchs(t0, "tie", 3) && t1.is_some_and(|t| matchs(t, "mesh", 4)) {
            return Err(ReadError::Unsupported {
                file: rec.file,
                line: rec.line,
                feature: "tie mesh".into(),
                message: "`tie mesh` constraints are not supported".into(),
            });
        }
        for (word, min_len, label) in [
            ("restart", 4, "restart"),
            ("retrieve", 4, "retrieve"),
            ("release", 7, "release constraints"),
            ("initial", 4, "initial conditions"),
            ("contact", 4, "contact"),
            ("thermal", 4, "thermal"),
            ("functionally", 4, "functionally graded"),
            ("crystal", 7, "crystal plasticity"),
            ("crack", 5, "crack"),
            ("table", 5, "table"),
            ("surface", 7, "surface"),
        ] {
            if matchs(t0, word, min_len) {
                return Err(ReadError::Unsupported {
                    file: rec.file,
                    line: rec.line,
                    feature: label.into(),
                    message: format!("WARP3D `{label}` directive is not supported"),
                });
            }
        }

        let msg = format!("unrecognized WARP3D top-level directive `{t0}`");
        return Err(ReadError::Parse { file: rec.file, line: rec.line, message: msg });
    }

    if parsed.saw_temperature_load {
        parsed.warnings.push("WARP3D `temperature` loads were ignored during static model-data import".into());
    }
    parsed.includes_read = lex.take_includes_read();

    Ok(parsed)
}

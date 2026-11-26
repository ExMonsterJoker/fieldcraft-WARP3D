//! WARP3D 18.5.0 ASCII `.inp` model-data reader.
//!
//! Parses pre-`compute` model entities (nodes, 3D solid elements, elastic/bilinear/table/power-law
//! Mises materials, solid sections, explicit integer `list` node sets, `constraints`, and static
//! `loading` patterns) into SI [`fieldcraft_model::Model`] instances using
//! [`fieldcraft_model::units::DisplaySystem::to_si`]. Supports relative `*input` includes and
//! single-pass `*parameter`/`@param` macro expansion, and halts lexing and parsing immediately on
//! the first `compute` statement (`history_imported: false`).

mod build;
mod lex;
mod parse;

#[cfg(test)]
mod tests;

use fieldcraft_model::Model;
use fieldcraft_model::units::Preset;
use thiserror::Error;

/// Maximum accepted cumulative raw input byte count across the root deck and all includes (64 MiB).
pub const MAX_INPUT_BYTES: usize = 64 * 1024 * 1024;
/// Maximum logical records processed per deck import.
pub const MAX_RECORDS: usize = 1_000_000;
/// Maximum length of one expanded logical record after `@param` and `,` continuations.
pub const MAX_EXPANDED_LINE_BYTES: usize = 16 * 1024;
/// Maximum cumulative expanded byte count across all logical records.
pub const MAX_CUMULATIVE_EXPANDED_BYTES: usize = 64 * 1024 * 1024;
/// Maximum unread keyword entries kept in [`ImportedDeck::unknown`].
pub const MAX_UNKNOWN_SHOWN: usize = 100;
/// Maximum `*input` nesting depth beyond the root deck (depth 9 = 10 open files).
pub const MAX_INCLUDE_DEPTH: usize = 9;
/// Maximum total `*input` include files opened per deck import.
pub const MAX_INCLUDES: usize = 256;
/// Maximum distinct `*parameter` macro definitions retained concurrently.
pub const MAX_PARAMETERS: usize = 300;
/// Maximum byte length of one `*parameter` macro value.
pub const MAX_PARAM_LEN: usize = 1024;
/// Maximum `@param` substitutions expanded on one physical line.
pub const MAX_SUBS_PER_LINE: usize = 30;
/// Maximum deferred `constraints` `plane ...` directives per deck import.
pub const MAX_DEFERRED_PLANES: usize = 4_096;

const HEADER_SCAN_LINES: usize = 128;

/// Storage and work caps checked before nodes, elements, or list ranges are stored or expanded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReadLimits {
    pub nodes: usize,
    pub elements: usize,
    pub members: usize,
}

impl Default for ReadLimits {
    fn default() -> Self {
        Self { nodes: fieldcraft_model::MAX_NODES, elements: fieldcraft_model::MAX_ELEMENTS, members: fieldcraft_model::MAX_ELEMENTS }
    }
}

/// Internal DoS bounds for the lexer and parser, parameterized here so tests can exercise caps quickly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct LexLimits {
    pub max_input_bytes: usize,
    pub max_records: usize,
    pub max_expanded_line_bytes: usize,
    pub max_cumulative_expanded_bytes: usize,
    pub max_include_depth: usize,
    pub max_includes: usize,
    pub max_parameters: usize,
    pub max_param_len: usize,
    pub max_subs_per_line: usize,
    pub max_deferred_planes: usize,
}

impl Default for LexLimits {
    fn default() -> Self {
        Self {
            max_input_bytes: MAX_INPUT_BYTES,
            max_records: MAX_RECORDS,
            max_expanded_line_bytes: MAX_EXPANDED_LINE_BYTES,
            max_cumulative_expanded_bytes: MAX_CUMULATIVE_EXPANDED_BYTES,
            max_include_depth: MAX_INCLUDE_DEPTH,
            max_includes: MAX_INCLUDES,
            max_parameters: MAX_PARAMETERS,
            max_param_len: MAX_PARAM_LEN,
            max_subs_per_line: MAX_SUBS_PER_LINE,
            max_deferred_planes: MAX_DEFERRED_PLANES,
        }
    }
}

/// Pull request passed to the Layer 6 include callback when encountering `*input`/`*inout`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IncludeRequest<'a> {
    pub target: &'a str,
    pub remaining_bytes: usize,
}

/// Include bytes and metadata returned by the Layer 6 include callback.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedInclude {
    pub display_name: String,
    pub canonical_key: String,
    pub bytes: Vec<u8>,
}

/// Dedicated, cloneable error taxonomy for WARP3D ASCII deck import.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum ReadError {
    #[error(transparent)]
    Model(#[from] fieldcraft_model::Error),

    #[error("WARP3D input {file}:{line}: {message}")]
    Parse { file: String, line: usize, message: String },

    #[error("WARP3D input {file}:{line} ({feature}): {message}")]
    Unsupported { file: String, line: usize, feature: String, message: String },

    #[error("WARP3D deck limit reached: {0}")]
    Limit(String),

    #[error("WARP3D include refused: {0}")]
    IncludeRefused(String),
}

/// Provenance of the consistent unit preset applied during import.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnitsSource {
    Explicit,
    Header,
    Session,
}

impl UnitsSource {
    /// Wire-format label for engine JSON replies.
    pub fn as_str(self) -> &'static str {
        match self {
            UnitsSource::Explicit => "explicit",
            UnitsSource::Header => "header",
            UnitsSource::Session => "session",
        }
    }
}

/// One unread top-level keyword block skipped during deck import.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownDirective {
    pub keyword: String,
    pub file: Option<String>,
    pub line: usize,
}

/// Location where deck reading intentionally stopped early (e.g. `compute`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StoppedAt {
    pub file: String,
    pub line: usize,
    pub reason: String,
}

/// Imported WARP3D model data, resolved unit provenance, and diagnostics.
#[derive(Debug, Clone)]
pub struct ImportedDeck {
    pub model: Model,
    pub units: Preset,
    pub units_source: UnitsSource,
    pub unknown: Vec<UnknownDirective>,
    pub unknown_count: usize,
    pub warnings: Vec<String>,
    pub stopped_at: Option<StoppedAt>,
    pub includes_read: Vec<String>,
}

/// Scan the first 128 physical lines for a `! fieldcraft-units: <preset>` header comment.
///
/// Decodes lines lossily (`\r` stripped), skips leading whitespace, strips optional inline `$`
/// comments after `:`, and returns the first valid `(Preset, one_based_line)`. Subsequent header
/// lines are ignored; an empty or unrecognized preset label returns [`ReadError::Parse`] with
/// `file: "<input>"`.
pub fn scan_units_header(bytes: &[u8]) -> Result<Option<(Preset, usize)>, ReadError> {
    const PREFIX: &str = "fieldcraft-units";
    for (idx, raw_line) in bytes.split(|&b| b == b'\n').take(HEADER_SCAN_LINES).enumerate() {
        let line_no = idx + 1;
        let line_bytes = raw_line.strip_suffix(b"\r").unwrap_or(raw_line);
        let text = String::from_utf8_lossy(line_bytes);
        let trimmed = text.trim_start();
        let Some(after_bang) = trimmed.strip_prefix('!') else {
            continue;
        };
        let after_ws = after_bang.trim_start_matches([' ', '\t']);
        let Some(head) = after_ws.get(..PREFIX.len()) else {
            continue;
        };
        if !head.eq_ignore_ascii_case(PREFIX) {
            continue;
        }
        let after_key = after_ws.get(PREFIX.len()..).unwrap_or("").trim_start_matches([' ', '\t']);
        let Some(after_colon) = after_key.strip_prefix(':') else {
            continue;
        };
        let value_slice = after_colon.split_once('$').map_or(after_colon, |(before, _)| before);
        let label = value_slice.trim_matches([' ', '\t']);
        if label.is_empty() {
            return Err(ReadError::Parse { file: "<input>".into(), line: line_no, message: "empty `fieldcraft-units` header preset".into() });
        }
        let Some(preset) = Preset::from_label(label) else {
            return Err(ReadError::Parse { file: "<input>".into(), line: line_no, message: format!("unknown `fieldcraft-units` preset `{label}`") });
        };
        return Ok(Some((preset, line_no)));
    }
    Ok(None)
}

/// Parse a single flat WARP3D ASCII deck (refusing `*input` includes) within `limits`.
pub fn read_limited(
    root_name: &str,
    bytes: &[u8],
    explicit_units: Option<Preset>,
    fallback_preset: Preset,
    limits: ReadLimits,
) -> Result<ImportedDeck, ReadError> {
    read_with_includes(root_name, bytes, explicit_units, fallback_preset, limits, |_| {
        Err(ReadError::Unsupported {
            file: root_name.to_string(),
            line: 0,
            feature: "*input".into(),
            message: "`*input`/`*inout` include files are disabled in flat reader".into(),
        })
    })
}

/// Parse a WARP3D ASCII deck (and any relative `*input`/`*inout` includes resolved by
/// `include_loader`) within `limits`.
pub fn read_with_includes<F>(
    root_name: &str,
    bytes: &[u8],
    explicit_units: Option<Preset>,
    fallback_preset: Preset,
    limits: ReadLimits,
    include_loader: F,
) -> Result<ImportedDeck, ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    read_with_includes_internal_limits(root_name, bytes, explicit_units, fallback_preset, limits, LexLimits::default(), include_loader)
}

#[cfg_attr(not(test), allow(dead_code))]
pub(crate) fn read_with_internal_limits(
    root_name: &str,
    bytes: &[u8],
    explicit_units: Option<Preset>,
    fallback_preset: Preset,
    limits: ReadLimits,
    lex_limits: LexLimits,
) -> Result<ImportedDeck, ReadError> {
    read_with_includes_internal_limits(root_name, bytes, explicit_units, fallback_preset, limits, lex_limits, |_| {
        Err(ReadError::Unsupported {
            file: root_name.to_string(),
            line: 0,
            feature: "*input".into(),
            message: "`*input`/`*inout` include files are disabled in flat reader".into(),
        })
    })
}

pub(crate) fn read_with_includes_internal_limits<F>(
    root_name: &str,
    bytes: &[u8],
    explicit_units: Option<Preset>,
    fallback_preset: Preset,
    limits: ReadLimits,
    lex_limits: LexLimits,
    include_loader: F,
) -> Result<ImportedDeck, ReadError>
where
    F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
{
    if bytes.len() > lex_limits.max_input_bytes {
        return Err(ReadError::Limit(format!("input `{root_name}` ({} bytes) exceeds {} byte input cap", bytes.len(), lex_limits.max_input_bytes)));
    }

    let header = scan_units_header(bytes).map_err(|err| match err {
        ReadError::Parse { line, message, .. } => ReadError::Parse { file: root_name.to_string(), line, message },
        other => other,
    })?;

    let (units, units_source, session_warning) = match (explicit_units, header) {
        (Some(preset), _) => (preset, UnitsSource::Explicit, None),
        (None, Some((preset, _))) => (preset, UnitsSource::Header, None),
        (None, None) => (
            fallback_preset,
            UnitsSource::Session,
            Some(format!("no `! fieldcraft-units:` header found; assuming session display preset `{}`", fallback_preset.label())),
        ),
    };

    let parsed = parse::parse_deck(root_name, bytes, limits, lex_limits, include_loader)?;
    let mut imported = build::build_model(root_name, parsed, units, units_source, limits)?;
    if let Some(w) = session_warning {
        imported.warnings.insert(0, w);
    }
    Ok(imported)
}

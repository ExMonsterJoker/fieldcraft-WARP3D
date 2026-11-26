//! Ordered 6-stage normalization, include stack, `@param` expansion, and tokenization pipeline
//! for WARP3D ASCII decks.

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};

use fieldcraft_model::progress;

use super::{IncludeRequest, LexLimits, ReadError, ResolvedInclude};

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Token {
    pub text: String,
    pub quoted: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Record {
    pub file: String,
    pub line: usize,
    pub is_root: bool,
    pub tokens: Vec<Token>,
}

impl Record {
    pub(crate) fn tok(&self, idx: usize) -> Option<&str> {
        self.tokens.get(idx).map(|t| t.text.as_str())
    }

    pub(crate) fn tok_obj(&self, idx: usize) -> Option<&Token> {
        self.tokens.get(idx)
    }
}

/// Strict 5-line float parser (`d`/`D` -> `e`, finite only, rejects `,`/`;`/`(`/`)`).
pub(crate) fn parse_f64(s: &str) -> Option<f64> {
    if s.is_empty() || s.contains([',', ';', '(', ')']) {
        return None;
    }
    let normalized = if s.contains(['d', 'D']) { Cow::Owned(s.replace(['d', 'D'], "e")) } else { Cow::Borrowed(s) };
    normalized.parse::<f64>().ok().filter(|v| v.is_finite())
}

struct IncludeFrame<'a> {
    display_name: String,
    canonical_key: String,
    is_root: bool,
    bytes: Cow<'a, [u8]>,
    offset: usize,
    line_no: usize,
}

pub(crate) struct Lexer<'a> {
    frames: Vec<IncludeFrame<'a>>,
    active_keys: BTreeSet<String>,
    includes_read: Vec<String>,
    include_count: usize,
    remaining_bytes: usize,
    params: BTreeMap<String, String>,
    limits: LexLimits,
    record_count: usize,
    cumulative_bytes: usize,
    pushed: Option<Record>,
}

impl<'a> Lexer<'a> {
    pub(crate) fn new(file: impl Into<String>, bytes: &'a [u8], limits: LexLimits) -> Self {
        let display_name = file.into();
        let remaining_bytes = limits.max_input_bytes.saturating_sub(bytes.len());
        Self {
            frames: vec![IncludeFrame {
                display_name,
                canonical_key: String::new(),
                is_root: true,
                bytes: Cow::Borrowed(bytes),
                offset: 0,
                line_no: 0,
            }],
            active_keys: BTreeSet::new(),
            includes_read: Vec::new(),
            include_count: 0,
            remaining_bytes,
            params: BTreeMap::new(),
            limits,
            record_count: 0,
            cumulative_bytes: 0,
            pushed: None,
        }
    }

    pub(crate) fn push_back(&mut self, record: Record) {
        self.pushed = Some(record);
    }

    pub(crate) fn take_includes_read(&mut self) -> Vec<String> {
        std::mem::take(&mut self.includes_read)
    }

    pub(crate) fn set_param(&mut self, name: String, value: String, file: &str, line: usize) -> Result<(), ReadError> {
        if name.is_empty() {
            return Err(ReadError::Parse { file: file.to_string(), line, message: "empty `*parameter` macro name".into() });
        }
        if value.len() > self.limits.max_param_len {
            return Err(ReadError::Limit(format!(
                "{file}:{line}: `*parameter` `{name}` value ({} bytes) exceeds {} byte limit",
                value.len(),
                self.limits.max_param_len
            )));
        }
        if !self.params.contains_key(&name) && self.params.len() >= self.limits.max_parameters {
            return Err(ReadError::Limit(format!("{file}:{line}: `*parameter` count exceeds {}", self.limits.max_parameters)));
        }
        self.params.insert(name, value);
        Ok(())
    }

    pub(crate) fn push_include<F>(&mut self, target: &str, file: &str, line: usize, loader: &mut F) -> Result<(), ReadError>
    where
        F: FnMut(IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError>,
    {
        let current_depth = self.frames.len().saturating_sub(1);
        if current_depth >= self.limits.max_include_depth {
            return Err(ReadError::Limit(format!("{file}:{line}: `*input` include depth exceeds {}", self.limits.max_include_depth)));
        }
        if self.include_count >= self.limits.max_includes {
            return Err(ReadError::Limit(format!("{file}:{line}: `*input` include count exceeds {}", self.limits.max_includes)));
        }
        let resolved = loader(IncludeRequest { target, remaining_bytes: self.remaining_bytes }).map_err(|err| match err {
            ReadError::Unsupported { line: 0, feature, message, .. } => ReadError::Unsupported { file: file.to_string(), line, feature, message },
            other => other,
        })?;
        if resolved.bytes.len() > self.remaining_bytes {
            return Err(ReadError::Limit(format!(
                "{file}:{line}: include `{}` ({} bytes) exceeds remaining {} byte input cap",
                resolved.display_name,
                resolved.bytes.len(),
                self.remaining_bytes
            )));
        }
        if self.active_keys.contains(&resolved.canonical_key) {
            return Err(ReadError::IncludeRefused(format!("include cycle detected at `{}`", resolved.display_name)));
        }
        self.remaining_bytes = self.remaining_bytes.saturating_sub(resolved.bytes.len());
        self.include_count = self.include_count.saturating_add(1);
        self.includes_read.push(resolved.display_name.clone());
        self.active_keys.insert(resolved.canonical_key.clone());
        self.frames.push(IncludeFrame {
            display_name: resolved.display_name,
            canonical_key: resolved.canonical_key,
            is_root: false,
            bytes: Cow::Owned(resolved.bytes),
            offset: 0,
            line_no: 0,
        });
        Ok(())
    }

    fn expand_params(&self, line: &str, file: &str, line_no: usize) -> Result<String, ReadError> {
        if !line.contains('@') {
            return Ok(line.to_string());
        }
        let mut out = String::with_capacity(line.len());
        let mut subs = 0usize;
        let mut rest = line;
        while let Some(at_pos) = rest.find('@') {
            let before = rest.get(..at_pos).unwrap_or("");
            out.push_str(before);
            if out.len() > self.limits.max_expanded_line_bytes {
                return Err(ReadError::Limit(format!(
                    "{file}:{line_no}: macro-expanded line exceeds {} byte expanded limit",
                    self.limits.max_expanded_line_bytes
                )));
            }
            subs = subs.saturating_add(1);
            if subs > self.limits.max_subs_per_line {
                return Err(ReadError::Parse {
                    file: file.to_string(),
                    line: line_no,
                    message: format!("exceeded {} `@param` substitutions on one line", self.limits.max_subs_per_line),
                });
            }
            let after_at = rest.get(at_pos + 1..).unwrap_or("");
            let token_end = after_at.find(|c: char| c.is_ascii_whitespace()).unwrap_or(after_at.len());
            let raw_tok = after_at.get(..token_end).unwrap_or("");
            let mut name_slice = raw_tok;
            if let Some(s) = name_slice.strip_suffix(',') {
                name_slice = s;
            }
            if let Some(s) = name_slice.strip_suffix('\'').or_else(|| name_slice.strip_suffix('"')) {
                name_slice = s;
            }
            let suffix = raw_tok.get(name_slice.len()..).unwrap_or("");
            if name_slice.is_empty() {
                return Err(ReadError::Parse { file: file.to_string(), line: line_no, message: "empty `@` parameter reference".into() });
            }
            let Some(val) = self.params.get(name_slice) else {
                return Err(ReadError::Parse { file: file.to_string(), line: line_no, message: format!("undefined `*parameter` `{name_slice}`") });
            };
            if out.len().saturating_add(val.len()).saturating_add(suffix.len()) > self.limits.max_expanded_line_bytes {
                return Err(ReadError::Limit(format!(
                    "{file}:{line_no}: macro-expanded line exceeds {} byte expanded limit",
                    self.limits.max_expanded_line_bytes
                )));
            }
            out.push_str(val);
            out.push_str(suffix);
            rest = after_at.get(token_end..).unwrap_or("");
        }
        if out.len().saturating_add(rest.len()) > self.limits.max_expanded_line_bytes {
            return Err(ReadError::Limit(format!(
                "{file}:{line_no}: macro-expanded line exceeds {} byte expanded limit",
                self.limits.max_expanded_line_bytes
            )));
        }
        out.push_str(rest);
        Ok(out)
    }

    /// Read the next active physical line after applying stages 1–4:
    /// col-1 comment skip, inline `$` split + strict active UTF-8 validation, tab normalization,
    /// and single-pass `@param` expansion.
    fn next_active_line(&mut self) -> Result<Option<(String, usize, bool, String)>, ReadError> {
        loop {
            let Some(top) = self.frames.last_mut() else {
                return Ok(None);
            };
            if top.offset >= top.bytes.len() {
                if self.frames.len() > 1 {
                    if let Some(popped) = self.frames.pop() {
                        self.active_keys.remove(&popped.canonical_key);
                    }
                    continue;
                }
                return Ok(None);
            }

            let slice = top.bytes.get(top.offset..).unwrap_or(&[]);
            let (raw_line, advance) = match slice.iter().position(|&b| b == b'\n') {
                Some(pos) => (slice.get(..pos).unwrap_or(slice), pos + 1),
                None => (slice, slice.len()),
            };
            top.offset = top.offset.saturating_add(advance);
            top.line_no = top.line_no.saturating_add(1);
            let line_no = top.line_no;
            let raw = raw_line.strip_suffix(b"\r").unwrap_or(raw_line);
            if raw.is_empty() {
                continue;
            }

            // Stage 1: col-1 `!`, `#`, or col-1 `c`/`C` followed by space, tab, or EOL.
            let first = raw.first().copied();
            let is_col1_comment = matches!(first, Some(b'!' | b'#'))
                || (matches!(first, Some(b'c' | b'C')) && (raw.len() == 1 || matches!(raw.get(1), Some(b' ' | b'\t'))));
            if is_col1_comment {
                continue;
            }

            let frame_file = top.display_name.clone();
            let is_root = top.is_root;

            // Stage 2: unconditional split at first `$`; decode post-`$` lossily (discarded),
            // validate active pre-`$` slice as strict UTF-8.
            let active_bytes = match raw.iter().position(|&b| b == b'$') {
                Some(pos) => raw.get(..pos).unwrap_or(raw),
                None => raw,
            };
            let active = std::str::from_utf8(active_bytes).map_err(|_| ReadError::Parse {
                file: frame_file.clone(),
                line: line_no,
                message: "invalid UTF-8 sequence in active deck input".into(),
            })?;

            // Stage 3: replace ASCII `\t` with space and skip blank lines.
            let normalized = if active.contains('\t') { active.replace('\t', " ") } else { active.to_string() };
            if normalized.trim().is_empty() {
                continue;
            }

            // Stage 4: single-pass `@param` expansion (skipped on `*parameter` lines).
            let expanded = if is_star_parameter_line(&normalized) { normalized } else { self.expand_params(&normalized, &frame_file, line_no)? };

            return Ok(Some((frame_file, line_no, is_root, expanded)));
        }
    }

    /// Pull the next logical record (after trailing `,` continuations and tokenization).
    pub(crate) fn next_raw(&mut self) -> Result<Option<Record>, ReadError> {
        if let Some(rec) = self.pushed.take() {
            return Ok(Some(rec));
        }

        let Some((start_file, start_line, start_is_root, first_norm)) = self.next_active_line()? else {
            return Ok(None);
        };

        // Stage 5: trailing `,` continuation across comments and blank lines (except `*parameter`).
        let mut combined = first_norm;
        if combined.len() > self.limits.max_expanded_line_bytes {
            return Err(ReadError::Limit(format!(
                "{start_file}:{start_line}: line exceeds {} byte expanded limit",
                self.limits.max_expanded_line_bytes
            )));
        }

        if !is_star_parameter_line(&combined) {
            while let Some(stripped) = combined.trim_end().strip_suffix(',') {
                let mut buf = stripped.to_string();
                buf.push(' ');
                let Some((cont_file, cont_line, _, next_norm)) = self.next_active_line()? else {
                    return Err(ReadError::Parse {
                        file: start_file,
                        line: start_line,
                        message: "unexpected end of input after `,` continuation".into(),
                    });
                };
                if next_norm.trim_start().starts_with('*') {
                    return Err(ReadError::Parse {
                        file: cont_file,
                        line: cont_line,
                        message: "`*` directive is not allowed inside a `,` continuation".into(),
                    });
                }
                if buf.len().saturating_add(next_norm.len()) > self.limits.max_expanded_line_bytes {
                    return Err(ReadError::Limit(format!(
                        "{start_file}:{start_line}: continued record exceeds {} byte expanded limit",
                        self.limits.max_expanded_line_bytes
                    )));
                }
                buf.push_str(&next_norm);
                combined = buf;
            }
        }

        self.record_count = self.record_count.saturating_add(1);
        if self.record_count > self.limits.max_records {
            return Err(ReadError::Limit(format!("logical record count exceeds {}", self.limits.max_records)));
        }
        if self.record_count.is_multiple_of(256) {
            progress::check()?;
        }

        self.cumulative_bytes = self.cumulative_bytes.saturating_add(combined.len());
        if self.cumulative_bytes > self.limits.max_cumulative_expanded_bytes {
            return Err(ReadError::Limit(format!("cumulative expanded input exceeds {} bytes", self.limits.max_cumulative_expanded_bytes)));
        }

        // Stage 6: tokenize with shared `'`/`"` quote toggle and context-sensitive `-`.
        let tokens = tokenize_line(&combined);
        Ok(Some(Record { file: start_file, line: start_line, is_root: start_is_root, tokens }))
    }
}

fn is_star_parameter_line(line: &str) -> bool {
    let trimmed = line.trim_start();
    let Some(after_star) = trimmed.strip_prefix('*') else {
        return false;
    };
    let word = after_star.trim_start().split([' ', ',', '=', '$']).next().unwrap_or("");
    word.get(..5).is_some_and(|p| p.eq_ignore_ascii_case("param"))
}

fn tokenize_line(line: &str) -> Vec<Token> {
    let mut tokens = Vec::new();
    let mut cur = String::new();
    let mut cur_quoted = false;
    let mut in_quote = false;

    let flush = |cur: &mut String, cur_quoted: &mut bool, tokens: &mut Vec<Token>| {
        if !cur.is_empty() || *cur_quoted {
            tokens.push(Token { text: std::mem::take(cur), quoted: *cur_quoted });
            *cur_quoted = false;
        }
    };

    for ch in line.chars() {
        if in_quote {
            if ch == '\'' || ch == '"' {
                in_quote = false;
            } else {
                cur.push(ch);
            }
            continue;
        }

        match ch {
            '\'' | '"' => {
                if !cur.is_empty() && !cur_quoted {
                    flush(&mut cur, &mut cur_quoted, &mut tokens);
                }
                in_quote = true;
                cur_quoted = true;
            }
            ' ' | ',' => {
                flush(&mut cur, &mut cur_quoted, &mut tokens);
            }
            '=' => {
                flush(&mut cur, &mut cur_quoted, &mut tokens);
                tokens.push(Token { text: "=".into(), quoted: false });
            }
            '-' => {
                // Split `-` standalone only when immediately preceded by digits alone (e.g. `1-6500`),
                // leaving `-1.5e-02`, `1.0d-3`, `stress-strain`, `in-lbf-s-psi`, `M1-a` intact.
                if !cur_quoted && !cur.is_empty() && cur.bytes().all(|b| b.is_ascii_digit()) {
                    flush(&mut cur, &mut cur_quoted, &mut tokens);
                    tokens.push(Token { text: "-".into(), quoted: false });
                } else {
                    cur.push('-');
                }
            }
            other => cur.push(other),
        }
    }

    flush(&mut cur, &mut cur_quoted, &mut tokens);
    tokens
}

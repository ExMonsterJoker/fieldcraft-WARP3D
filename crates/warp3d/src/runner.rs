//! WARP3D external executable discovery and subprocess execution.
//!
//! Subprocess spawning is gated to non-wasm targets so `fieldcraft-warp3d` builds cleanly for
//! `wasm32-unknown-unknown`.

use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;

use fieldcraft_model::Model;

use crate::{CrackFrontSpec, Result, Warp3dError, Warp3dSolution};

#[cfg(not(target_arch = "wasm32"))]
const MAX_BINARY_PATH_BYTES: usize = 4096;

/// How FieldCraft resolved the external `warp3d` executable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Warp3dBinarySource {
    PrefsOverride,
    EnvFieldcraftBin,
    EnvWarp3dHome,
    Path,
    NotFound,
}

/// Resolution result for the external `warp3d` executable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Warp3dBinaryStatus {
    pub source: Warp3dBinarySource,
    pub path: Option<PathBuf>,
    pub valid: bool,
    pub error: Option<String>,
}

#[cfg(not(target_arch = "wasm32"))]
const DEFAULT_SOLVER_TIMEOUT_SECS: u64 = 86_400;
#[cfg(not(target_arch = "wasm32"))]
const LOG_TAIL_LINES: usize = 40;
#[cfg(not(target_arch = "wasm32"))]
const LOG_TAIL_CHARS: usize = 4096;
#[cfg(not(target_arch = "wasm32"))]
const CANCEL_POLL_INTERVAL: std::time::Duration = std::time::Duration::from_millis(10);

#[cfg(not(target_arch = "wasm32"))]
fn candidate_binary_names() -> &'static [&'static str] {
    if cfg!(windows) {
        &["warp3d.exe", "warp3d.omp.exe", "warp3d"]
    } else {
        // WARP3D's official Linux distribution ships `$WARP3D_HOME/run_linux/warp3d.exe`.
        &["warp3d.exe", "warp3d.omp", "warp3d"]
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn home_subdirs() -> &'static [&'static str] {
    if cfg!(windows) {
        &["", "run_windows", "bin", "run_linux", "run_macOS", "run_macos"]
    } else if cfg!(target_os = "macos") {
        &["", "run_macOS", "run_macos", "bin", "run_linux", "run_windows"]
    } else {
        &["", "run_linux", "bin", "run_macOS", "run_macos", "run_windows"]
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn is_unc_or_network_path(raw: &Path) -> bool {
    let bytes = raw.as_os_str().as_encoded_bytes();
    if bytes.starts_with(b"\\\\") {
        return true;
    }
    if cfg!(windows) && bytes.starts_with(b"//") {
        return true;
    }
    matches!(
        raw.components().next(),
        Some(std::path::Component::Prefix(p))
            if matches!(p.kind(), std::path::Prefix::UNC(_, _) | std::path::Prefix::VerbatimUNC(_, _))
    )
}

#[cfg(not(target_arch = "wasm32"))]
fn validate_binary_path(raw: &Path) -> std::result::Result<(), String> {
    let os = raw.as_os_str();
    if os.is_empty() {
        return Err("executable path must not be empty".into());
    }
    let bytes = os.as_encoded_bytes();
    if bytes.len() > MAX_BINARY_PATH_BYTES {
        return Err(format!("executable path exceeds {MAX_BINARY_PATH_BYTES} bytes"));
    }
    if bytes.contains(&0) {
        return Err("executable path must not contain NUL bytes".into());
    }
    if is_unc_or_network_path(raw) {
        return Err(format!("network/UNC executable path `{}` is not allowed", raw.display()));
    }
    if !raw.is_absolute() {
        return Err(format!("executable path `{}` must be absolute", raw.display()));
    }
    if !raw.is_file() {
        return Err(format!("`{}` is not a file", raw.display()));
    }
    Ok(())
}

#[cfg(not(target_arch = "wasm32"))]
fn resolve_warp3d_binary_in(
    override_bin: Option<&Path>,
    bin_env: Option<&std::ffi::OsStr>,
    home_env: Option<&std::ffi::OsStr>,
    path_env: Option<&std::ffi::OsStr>,
) -> Warp3dBinaryStatus {
    if let Some(raw) = override_bin {
        return match validate_binary_path(raw) {
            Ok(()) => Warp3dBinaryStatus { source: Warp3dBinarySource::PrefsOverride, path: Some(raw.to_path_buf()), valid: true, error: None },
            Err(msg) => {
                Warp3dBinaryStatus { source: Warp3dBinarySource::PrefsOverride, path: Some(raw.to_path_buf()), valid: false, error: Some(msg) }
            }
        };
    }

    if let Some(bin) = bin_env.filter(|v| !v.is_empty()) {
        let p = PathBuf::from(bin);
        return match validate_binary_path(&p) {
            Ok(()) => Warp3dBinaryStatus { source: Warp3dBinarySource::EnvFieldcraftBin, path: Some(p), valid: true, error: None },
            Err(msg) => Warp3dBinaryStatus {
                source: Warp3dBinarySource::EnvFieldcraftBin,
                path: Some(p.clone()),
                valid: false,
                error: Some(format!("FIELDCRAFT_WARP3D_BIN points to `{}` ({msg})", p.display())),
            },
        };
    }

    if let Some(home) = home_env.filter(|v| !v.is_empty()) {
        let home_dir = PathBuf::from(home);
        for sub in home_subdirs() {
            let base = if sub.is_empty() { home_dir.clone() } else { home_dir.join(sub) };
            for name in candidate_binary_names() {
                let candidate = base.join(name);
                if validate_binary_path(&candidate).is_ok() {
                    return Warp3dBinaryStatus { source: Warp3dBinarySource::EnvWarp3dHome, path: Some(candidate), valid: true, error: None };
                }
            }
        }
    }

    if let Some(path_var) = path_env.filter(|v| !v.is_empty()) {
        for dir in std::env::split_paths(path_var) {
            for name in candidate_binary_names() {
                let candidate = dir.join(name);
                if validate_binary_path(&candidate).is_ok() {
                    return Warp3dBinaryStatus { source: Warp3dBinarySource::Path, path: Some(candidate), valid: true, error: None };
                }
            }
        }
    }

    Warp3dBinaryStatus {
        source: Warp3dBinarySource::NotFound,
        path: None,
        valid: false,
        error: Some("neither FIELDCRAFT_WARP3D_BIN, WARP3D_HOME, nor PATH contains a warp3d executable".into()),
    }
}

/// Resolve the external WARP3D executable and its provenance, honoring an optional override path.
#[cfg(not(target_arch = "wasm32"))]
pub fn resolve_warp3d_binary(override_bin: Option<&Path>) -> Warp3dBinaryStatus {
    resolve_warp3d_binary_in(
        override_bin,
        std::env::var_os("FIELDCRAFT_WARP3D_BIN").as_deref(),
        std::env::var_os("WARP3D_HOME").as_deref(),
        std::env::var_os("PATH").as_deref(),
    )
}

/// Resolve the external WARP3D executable (desktop-only; returns `NotFound` on wasm32 targets).
#[cfg(target_arch = "wasm32")]
pub fn resolve_warp3d_binary(_override_bin: Option<&Path>) -> Warp3dBinaryStatus {
    Warp3dBinaryStatus {
        source: Warp3dBinarySource::NotFound,
        path: None,
        valid: false,
        error: Some("WARP3D execution is available only in the desktop app".into()),
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn find_warp3d_binary_in(
    bin_env: Option<&std::ffi::OsStr>,
    home_env: Option<&std::ffi::OsStr>,
    path_env: Option<&std::ffi::OsStr>,
) -> Result<PathBuf> {
    let status = resolve_warp3d_binary_in(None, bin_env, home_env, path_env);
    if status.valid
        && let Some(path) = status.path
    {
        return Ok(path);
    }
    Err(Warp3dError::BinaryNotFound(status.error.unwrap_or_else(|| "no valid WARP3D executable found".into())))
}

/// Locate the WARP3D executable via `FIELDCRAFT_WARP3D_BIN`, `WARP3D_HOME`, or `PATH`.
#[cfg(not(target_arch = "wasm32"))]
pub fn find_warp3d_binary() -> Result<PathBuf> {
    find_warp3d_binary_in(
        std::env::var_os("FIELDCRAFT_WARP3D_BIN").as_deref(),
        std::env::var_os("WARP3D_HOME").as_deref(),
        std::env::var_os("PATH").as_deref(),
    )
}

/// Locate the WARP3D executable (unsupported on wasm32 targets).
#[cfg(target_arch = "wasm32")]
pub fn find_warp3d_binary() -> Result<PathBuf> {
    Err(Warp3dError::Unsupported("external WARP3D execution is not available on wasm targets".into()))
}

#[cfg(not(target_arch = "wasm32"))]
struct TempDirGuard(PathBuf);

#[cfg(not(target_arch = "wasm32"))]
impl Drop for TempDirGuard {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn extra_linux_lib_dir_in(bin: &Path, from_override: bool, home_env: Option<&std::ffi::OsStr>) -> Option<PathBuf> {
    let sibling = bin.parent().and_then(|p| p.parent()).map(|p| p.join("linux_packages").join("lib")).filter(|p| p.is_dir());
    if from_override && sibling.is_some() {
        return sibling;
    }
    if let Some(home) = home_env.filter(|v| !v.is_empty()) {
        let candidate = PathBuf::from(home).join("linux_packages").join("lib");
        if candidate.is_dir() {
            return Some(candidate);
        }
    }
    sibling
}

#[cfg(not(target_arch = "wasm32"))]
fn extra_linux_lib_dir(bin: &Path, from_override: bool) -> Option<PathBuf> {
    extra_linux_lib_dir_in(bin, from_override, std::env::var_os("WARP3D_HOME").as_deref())
}

#[cfg(not(target_arch = "wasm32"))]
fn prepend_search_path(cmd: &mut std::process::Command, var: &str, dir: &Path) {
    let mut dirs = vec![dir.to_path_buf()];
    if let Some(existing) = std::env::var_os(var) {
        for p in std::env::split_paths(&existing) {
            if p != dir {
                dirs.push(p);
            }
        }
    }
    if let Ok(joined) = std::env::join_paths(dirs) {
        cmd.env(var, joined);
    }
}

#[cfg(not(target_arch = "wasm32"))]
struct LogTail {
    lines: std::collections::VecDeque<String>,
    max_lines: usize,
    max_chars: usize,
}

#[cfg(not(target_arch = "wasm32"))]
impl LogTail {
    fn new(max_lines: usize, max_chars: usize) -> Self {
        Self { lines: std::collections::VecDeque::with_capacity(max_lines), max_lines, max_chars }
    }

    fn push_line(&mut self, line: String) {
        if self.lines.len() >= self.max_lines {
            self.lines.pop_front();
        }
        self.lines.push_back(line);
    }

    fn into_string(self) -> String {
        let mut out = self.lines.into_iter().collect::<Vec<_>>().join("\n");
        let char_count = out.chars().count();
        if char_count > self.max_chars {
            let skip = char_count - self.max_chars;
            if let Some((byte_idx, _)) = out.char_indices().nth(skip) {
                out = out.get(byte_idx..).unwrap_or("").to_string();
            }
        }
        out
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn detect_warp3d_step(line: &str) -> Option<usize> {
    let trimmed = line.trim();
    let lower = trimmed.to_ascii_lowercase();
    if let Some(rest) = lower.strip_prefix("loading:") {
        if let Some(pos) = rest.find("step:")
            && let Some(step_tok) = rest.get(pos + 5..).and_then(|s| s.split_whitespace().next())
            && let Some(s) = step_tok.parse::<usize>().ok().filter(|&n| n > 0)
        {
            return Some(s);
        }
    } else if let Some(rest) = lower.strip_prefix("loading step")
        && let Some(step_tok) = rest.split_whitespace().next()
        && let Some(s) = step_tok.trim_end_matches(':').parse::<usize>().ok().filter(|&n| n > 0)
    {
        return Some(s);
    } else if let Some(rest) = lower.strip_prefix("step ")
        && let Some(step_tok) = rest.split_whitespace().next()
        && let Some(s) = step_tok.trim_end_matches(':').parse::<usize>().ok().filter(|&n| n > 0)
    {
        return Some(s);
    }
    None
}

#[cfg(not(target_arch = "wasm32"))]
fn stream_stderr(stream: Option<std::process::ChildStderr>) -> String {
    let Some(s) = stream else {
        return String::new();
    };
    let mut reader = std::io::BufReader::new(s);
    let mut scratch = Vec::new();
    let mut tail = LogTail::new(LOG_TAIL_LINES, LOG_TAIL_CHARS);
    while let Ok(Some((line, _truncated))) = crate::results::read_bounded_line(&mut reader, &mut scratch, crate::results::MAX_LINE_BYTES) {
        tail.push_line(line);
    }
    tail.into_string()
}

#[cfg(not(target_arch = "wasm32"))]
fn stream_stdout(
    stream: Option<std::process::ChildStdout>,
    label_map: &crate::LabelMap,
    step_count: usize,
    progress: Option<std::sync::Arc<fieldcraft_model::progress::Progress>>,
) -> ((Vec<crate::DomainIntegralResult>, Vec<Vec<u32>>), String) {
    let mut parser = crate::results::DomainIntegralParser::new(label_map);
    let Some(s) = stream else {
        return (parser.finish(), String::new());
    };
    let mut tail = LogTail::new(LOG_TAIL_LINES, LOG_TAIL_CHARS);
    fieldcraft_model::progress::scoped(progress, || {
        let mut reader = std::io::BufReader::new(s);
        let mut scratch = Vec::new();
        while let Ok(Some((line, _truncated))) = crate::results::read_bounded_line(&mut reader, &mut scratch, crate::results::MAX_LINE_BYTES) {
            if let Some(step_no) = detect_warp3d_step(&line) {
                let capped = if step_count > 0 { step_no.min(step_count) } else { step_no };
                fieldcraft_model::progress::count(capped as f64);
            }
            parser.feed_line(&line);
            tail.push_line(line);
        }
    });
    (parser.finish(), tail.into_string())
}

#[cfg(not(target_arch = "wasm32"))]
fn format_log_tail(stdout: &str, stderr: &str) -> String {
    let combined = match (stdout.trim().is_empty(), stderr.trim().is_empty()) {
        (true, true) => String::new(),
        (false, true) => stdout.to_string(),
        (true, false) => stderr.to_string(),
        (false, false) => format!("{stdout}\n--- stderr ---\n{stderr}"),
    };
    let mut tail = LogTail::new(LOG_TAIL_LINES, LOG_TAIL_CHARS);
    for line in combined.lines() {
        tail.push_line(line.to_string());
    }
    tail.into_string()
}

/// Generate a WARP3D deck in SI (`Preset::MKgS`), run the external WARP3D solver for `step`,
/// and parse flat-file and domain-integral results back into FieldCraft types.
#[cfg(not(target_arch = "wasm32"))]
pub fn run_warp3d(
    model: &Model,
    step: &str,
    crack: Option<&CrackFrontSpec>,
    threads: usize,
    deck_path: Option<&Path>,
    cancel: &AtomicBool,
) -> Result<Warp3dSolution> {
    run_warp3d_with_override(model, step, crack, threads, deck_path, cancel, None)
}

/// Generate a WARP3D deck in SI (`Preset::MKgS`), run the external WARP3D solver using an optional
/// override executable path, and parse flat-file and domain-integral results back into FieldCraft types.
#[cfg(not(target_arch = "wasm32"))]
#[allow(clippy::too_many_arguments, reason = "matches existing run_warp3d interface plus optional override path")]
pub fn run_warp3d_with_override(
    model: &Model,
    step: &str,
    crack: Option<&CrackFrontSpec>,
    threads: usize,
    deck_path: Option<&Path>,
    cancel: &AtomicBool,
    bin_override: Option<&Path>,
) -> Result<Warp3dSolution> {
    run_warp3d_inner(model, step, crack, threads, deck_path, cancel, bin_override, std::time::Duration::from_secs(DEFAULT_SOLVER_TIMEOUT_SECS))
}

#[cfg(all(test, not(target_arch = "wasm32")))]
pub(crate) fn run_warp3d_with_bin(
    model: &Model,
    step: &str,
    crack: Option<&CrackFrontSpec>,
    threads: usize,
    deck_path: Option<&Path>,
    cancel: &AtomicBool,
    bin: &Path,
) -> Result<Warp3dSolution> {
    run_warp3d_inner(model, step, crack, threads, deck_path, cancel, Some(bin), std::time::Duration::from_secs(DEFAULT_SOLVER_TIMEOUT_SECS))
}

#[cfg(all(test, not(target_arch = "wasm32")))]
pub(crate) fn run_warp3d_with_bin_and_timeout(
    model: &Model,
    step: &str,
    crack: Option<&CrackFrontSpec>,
    threads: usize,
    deck_path: Option<&Path>,
    cancel: &AtomicBool,
    bin: &Path,
    timeout: std::time::Duration,
) -> Result<Warp3dSolution> {
    run_warp3d_inner(model, step, crack, threads, deck_path, cancel, Some(bin), timeout)
}

#[cfg(not(target_arch = "wasm32"))]
#[allow(clippy::too_many_arguments)]
fn run_warp3d_inner(
    model: &Model,
    step: &str,
    crack: Option<&CrackFrontSpec>,
    threads: usize,
    deck_path: Option<&Path>,
    cancel: &AtomicBool,
    bin_override: Option<&Path>,
    timeout: std::time::Duration,
) -> Result<Warp3dSolution> {
    use std::io::Write;
    use std::process::{Command, Stdio};
    use std::sync::atomic::{AtomicU64, Ordering};

    if cancel.load(Ordering::Relaxed) {
        return Err(Warp3dError::Cancelled);
    }

    // Validate and resolve the WARP3D executable before generating or writing any deck files.
    let bin_status = resolve_warp3d_binary(bin_override);
    let from_override = bin_status.source == Warp3dBinarySource::PrefsOverride;
    let bin = if bin_status.valid
        && let Some(path) = bin_status.path
    {
        path
    } else if from_override {
        let shown = bin_status.path.as_ref().map(|p| p.display().to_string()).unwrap_or_default();
        let detail = bin_status.error.unwrap_or_else(|| "invalid executable path".into());
        return Err(Warp3dError::BinaryNotFound(format!(
            "configured WARP3D executable `{shown}` is unavailable ({detail}); update it in Help > Settings"
        )));
    } else {
        return Err(Warp3dError::BinaryNotFound(bin_status.error.unwrap_or_else(|| "no valid WARP3D executable found".into())));
    };

    let step_opt = (!step.is_empty()).then_some(step);
    let written = crate::write_deck(model, step_opt, crack, fieldcraft_model::units::Preset::MKgS)?;

    if let Some(out_path) = deck_path {
        if let Some(parent) = out_path.parent().filter(|p| !p.as_os_str().is_empty()) {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(out_path, written.text.as_bytes())?;
    }

    static RUN_SEQ: AtomicU64 = AtomicU64::new(0);
    let seq = RUN_SEQ.fetch_add(1, Ordering::Relaxed);
    let workdir = std::env::temp_dir().join(format!("fieldcraft-warp3d-{}-{seq}", std::process::id()));
    std::fs::create_dir_all(&workdir)?;
    let _guard = TempDirGuard(workdir.clone());

    std::fs::write(workdir.join("model.inp"), written.text.as_bytes())?;

    let thread_str = threads.max(1).to_string();
    let mut cmd = Command::new(&bin);
    cmd.current_dir(&workdir)
        .env("OMP_NUM_THREADS", &thread_str)
        .env("MKL_NUM_THREADS", &thread_str)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    if cfg!(windows) {
        if let Some(bin_dir) = bin.parent().filter(|p| p.is_dir()) {
            prepend_search_path(&mut cmd, "PATH", bin_dir);
        }
    } else if let Some(lib_dir) = extra_linux_lib_dir(&bin, from_override) {
        prepend_search_path(&mut cmd, "LD_LIBRARY_PATH", &lib_dir);
    }

    let mut child = cmd.spawn()?;
    let stdin_handle = child.stdin.take();
    let stdout_handle = child.stdout.take();
    let stderr_handle = child.stderr.take();

    let progress = fieldcraft_model::progress::current();
    fieldcraft_model::progress::stage("warp3d", (written.step_count > 0).then_some(written.step_count as f64), "steps");

    let deck_bytes = written.text.into_bytes();
    let writer_thread = std::thread::spawn(move || {
        if let Some(mut stdin) = stdin_handle {
            let _ = stdin.write_all(&deck_bytes);
            let _ = stdin.flush();
        }
    });
    let label_map = written.label_map;
    let step_count = written.step_count;
    let progress_stdout = progress;
    let stdout_thread = std::thread::spawn(move || stream_stdout(stdout_handle, &label_map, step_count, progress_stdout));
    let stderr_thread = std::thread::spawn(move || stream_stderr(stderr_handle));

    enum ChildOutcome {
        Exited(std::process::ExitStatus),
        Cancelled,
        TimedOut,
        WaitError(std::io::Error),
    }

    let started = std::time::Instant::now();
    let outcome = loop {
        if cancel.load(Ordering::Relaxed) {
            let _ = child.kill();
            let _ = child.wait();
            break ChildOutcome::Cancelled;
        }
        if started.elapsed() >= timeout {
            let _ = child.kill();
            let _ = child.wait();
            break ChildOutcome::TimedOut;
        }
        match child.try_wait() {
            Ok(Some(exit)) => break ChildOutcome::Exited(exit),
            Ok(None) => {}
            Err(err) => {
                let _ = child.kill();
                let _ = child.wait();
                break ChildOutcome::WaitError(err);
            }
        }
        std::thread::sleep(CANCEL_POLL_INTERVAL);
    };

    let _ = writer_thread.join();
    let (preparsed_fracture, stdout_tail) = stdout_thread.join().unwrap_or_default();
    let stderr_tail = stderr_thread.join().unwrap_or_default();

    match outcome {
        ChildOutcome::Cancelled => return Err(Warp3dError::Cancelled),
        ChildOutcome::TimedOut => {
            return Err(Warp3dError::Timeout { seconds: timeout.as_secs(), log_tail: format_log_tail(&stdout_tail, &stderr_tail) });
        }
        ChildOutcome::WaitError(err) => return Err(err.into()),
        ChildOutcome::Exited(status) => {
            if !status.success() || !crate::results::has_any_flat_files(&workdir) {
                return Err(Warp3dError::SolverFailed { code: status.code(), log_tail: format_log_tail(&stdout_tail, &stderr_tail) });
            }
        }
    }

    crate::results::read_results_dir_with_preparsed_fracture(
        model,
        &written.step_name,
        &workdir,
        Some(fieldcraft_model::units::Preset::MKgS),
        stdout_tail,
        preparsed_fracture,
        written.warnings,
    )
}

/// Generate a WARP3D deck and run the external WARP3D solver (unsupported on wasm32 targets).
#[cfg(target_arch = "wasm32")]
pub fn run_warp3d(
    _model: &Model,
    _step: &str,
    _crack: Option<&CrackFrontSpec>,
    _threads: usize,
    _deck_path: Option<&Path>,
    _cancel: &AtomicBool,
) -> Result<Warp3dSolution> {
    Err(Warp3dError::Unsupported("external WARP3D execution is not available on wasm targets".into()))
}

/// Generate a WARP3D deck and run the external WARP3D solver with override (unsupported on wasm32 targets).
#[cfg(target_arch = "wasm32")]
#[allow(clippy::too_many_arguments, reason = "matches existing run_warp3d interface plus optional override path")]
pub fn run_warp3d_with_override(
    _model: &Model,
    _step: &str,
    _crack: Option<&CrackFrontSpec>,
    _threads: usize,
    _deck_path: Option<&Path>,
    _cancel: &AtomicBool,
    _bin_override: Option<&Path>,
) -> Result<Warp3dSolution> {
    Err(Warp3dError::Unsupported("external WARP3D execution is not available on wasm targets".into()))
}

#[cfg(all(test, not(target_arch = "wasm32")))]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use std::ffi::OsString;
    use std::fs;
    use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

    use fieldcraft_model::{
        BoundaryCondition, Elastic, ElementKind, Hardening, Load, Material, Model, NonlinearStatic, Plastic, Procedure, Section, SectionKind, Set,
        Step, Target,
    };

    use super::*;

    fn temp_dir(tag: &str) -> PathBuf {
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let dir =
            std::env::temp_dir().join(format!("fieldcraft-warp3d-runner-test-{tag}-{}-{}", std::process::id(), SEQ.fetch_add(1, Ordering::Relaxed)));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn unit_hex8_model(nonlinear: bool, plastic: bool) -> Model {
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
        let mat = Material {
            elastic: Some(Elastic::Isotropic { young: 2.0e11, poisson: 0.3 }),
            plastic: plastic.then_some(Plastic::Bilinear { yield_stress: 2.0e8, hardening_modulus: 2.0e9, hardening: Hardening::Isotropic }),
            ..Default::default()
        };
        model.add_material("Steel", mat).unwrap();
        model.add_set("all", Set { elements: vec![1], ..Set::default() }).unwrap();
        model.add_set("fix_x", Set { nodes: vec![1, 4, 5, 8], ..Set::default() }).unwrap();
        model.add_set("fix_y", Set { nodes: vec![1, 5], ..Set::default() }).unwrap();
        model.add_set("fix_z", Set { nodes: vec![1], ..Set::default() }).unwrap();
        model.add_set("tip", Set { nodes: vec![2, 3, 6, 7], ..Set::default() }).unwrap();
        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model
            .add_boundary_condition(
                "bx",
                BoundaryCondition::Displacement { target: Target::Set("fix_x".into()), values: [Some(0.0), None, None, None, None, None] },
            )
            .unwrap();
        model
            .add_boundary_condition(
                "by",
                BoundaryCondition::Displacement { target: Target::Set("fix_y".into()), values: [None, Some(0.0), None, None, None, None] },
            )
            .unwrap();
        model
            .add_boundary_condition(
                "bz",
                BoundaryCondition::Displacement { target: Target::Set("fix_z".into()), values: [None, None, Some(0.0), None, None, None] },
            )
            .unwrap();
        let total_force = if plastic { 4.0e8 } else { 1.0e8 };
        model
            .add_load("pull", Load::Force { target: Target::Set("tip".into()), vector: [total_force, 0.0, 0.0], distribution: Default::default() })
            .unwrap();
        let proc = if nonlinear {
            Procedure::NonlinearStatic(NonlinearStatic { initial_increment: 0.25, max_increments: 20, ..NonlinearStatic::default() })
        } else {
            Procedure::Static {}
        };
        let mut step = Step::new("Pull", proc);
        step.boundary_conditions.insert("bx".into());
        step.boundary_conditions.insert("by".into());
        step.boundary_conditions.insert("bz".into());
        step.loads.insert("pull".into());
        model.add_step(step, None).unwrap();
        model
    }

    fn write_mock_success_script(dir: &Path) -> PathBuf {
        let zeros_26 = vec!["0.0"; 26].join(" ");
        let mut stress_26 = vec!["0.0"; 26];
        stress_26[0] = "1.0e8";
        stress_26[7] = "1.0e8";
        let stress_row = stress_26.join(" ");

        #[cfg(windows)]
        {
            let ps_path = dir.join("mock_warp3d.ps1");
            let ps_body = format!(
                "$inp = [Console]::In.ReadToEnd()\n\
                 foreach ($line in ($inp -split \"`r?`n\")) {{\n\
                   if (-not $line.StartsWith('!') -and $line.Length -gt 80) {{ throw \"Line too long: $line\" }}\n\
                 }}\n\
                 foreach ($kw in @('structure','material','coordinates','incidences','blocking automatic','loading','compute displacements','output flat text nodal displacements','output flat text nodal stresses','output flat text nodal reactions','output flat text element stresses')) {{\n\
                   if ($inp -notmatch [regex]::Escape($kw)) {{ throw \"Missing keyword $kw\" }}\n\
                 }}\n\
                 if ($env:OMP_NUM_THREADS -ne '2' -or $env:MKL_NUM_THREADS -ne '2') {{ throw 'Missing thread env' }}\n\
                 $wnd = (1..8 | ForEach-Object {{ if ($_ -in 2,3,6,7) {{ '5.0e-4 0.0 0.0' }} else {{ '0.0 0.0 0.0' }} }}) -join \"`n\"\n\
                 $wnr = (1..8 | ForEach-Object {{ if ($_ -in 1,4,5,8) {{ '-2.5e7 0.0 0.0' }} else {{ '0.0 0.0 0.0' }} }}) -join \"`n\"\n\
                 $wns = (1..8 | ForEach-Object {{ '{stress_row}' }}) -join \"`n\"\n\
                 $wes = '{zeros_26}'\n\
                 Set-Content -Path 'wnd0000001_text' -Value \"#`n$wnd\"\n\
                 Set-Content -Path 'wnr0000001_text' -Value \"#`n$wnr\"\n\
                 Set-Content -Path 'wns0000001_text' -Value \"#`n$wns\"\n\
                 Set-Content -Path 'wes0000001_text' -Value \"#`n$wes\"\n\
                 Write-Output 'WARP3D mock complete'\n"
            );
            fs::write(&ps_path, ps_body).unwrap();
            let cmd_path = dir.join("mock_warp3d.cmd");
            fs::write(&cmd_path, format!("@echo off\r\npowershell -NoProfile -ExecutionPolicy Bypass -File \"{}\"\r\n", ps_path.display())).unwrap();
            cmd_path
        }

        #[cfg(not(windows))]
        {
            use std::os::unix::fs::PermissionsExt;
            let sh_path = dir.join("mock_warp3d.sh");
            let sh_body = format!(
                "#!/bin/sh\nset -e\n\
                 inp=$(cat)\n\
                 if [ \"$OMP_NUM_THREADS\" != \"2\" ] || [ \"$MKL_NUM_THREADS\" != \"2\" ]; then exit 2; fi\n\
                 printf '%s\n' \"$inp\" | awk '!/^!/ && length($0) > 80 {{ exit 3 }}'\n\
                 for kw in 'structure' 'material' 'coordinates' 'incidences' 'blocking automatic' 'loading' 'compute displacements' 'output flat text nodal displacements' 'output flat text nodal stresses' 'output flat text nodal reactions' 'output flat text element stresses'; do\n\
                   printf '%s\n' \"$inp\" | grep -q \"$kw\" || exit 4\n\
                 done\n\
                 printf '#\\n0 0 0\\n5.0e-4 0 0\\n5.0e-4 0 0\\n0 0 0\\n0 0 0\\n5.0e-4 0 0\\n5.0e-4 0 0\\n0 0 0\\n' > wnd0000001_text\n\
                 printf '#\\n-2.5e7 0 0\\n0 0 0\\n0 0 0\\n-2.5e7 0 0\\n-2.5e7 0 0\\n0 0 0\\n0 0 0\\n-2.5e7 0 0\\n' > wnr0000001_text\n\
                 printf '#\\n{s}\\n{s}\\n{s}\\n{s}\\n{s}\\n{s}\\n{s}\\n{s}\\n' > wns0000001_text\n\
                 printf '#\\n{z}\\n' > wes0000001_text\n\
                 echo 'WARP3D mock complete'\n",
                s = stress_row,
                z = zeros_26,
            );
            fs::write(&sh_path, sh_body).unwrap();
            let mut perms = fs::metadata(&sh_path).unwrap().permissions();
            perms.set_mode(0o755);
            fs::set_permissions(&sh_path, perms).unwrap();
            sh_path
        }
    }

    fn write_mock_slow_script(dir: &Path) -> PathBuf {
        #[cfg(windows)]
        {
            let cmd_path = dir.join("mock_cancel.cmd");
            fs::write(&cmd_path, "@echo off\r\nfor /L %%i in (1,1,5000000) do @rem\r\n").unwrap();
            cmd_path
        }
        #[cfg(not(windows))]
        {
            use std::os::unix::fs::PermissionsExt;
            let sh_path = dir.join("mock_cancel.sh");
            fs::write(&sh_path, "#!/bin/sh\necho 'mock slow start'\nexec sleep 30\n").unwrap();
            let mut perms = fs::metadata(&sh_path).unwrap().permissions();
            perms.set_mode(0o755);
            fs::set_permissions(&sh_path, perms).unwrap();
            sh_path
        }
    }

    fn write_mock_fail_script(dir: &Path) -> PathBuf {
        #[cfg(windows)]
        {
            let cmd_path = dir.join("mock_fail.cmd");
            fs::write(&cmd_path, "@echo off\r\necho WARP3D fatal error: singular matrix\r\nexit /b 1\r\n").unwrap();
            cmd_path
        }
        #[cfg(not(windows))]
        {
            use std::os::unix::fs::PermissionsExt;
            let sh_path = dir.join("mock_fail.sh");
            fs::write(&sh_path, "#!/bin/sh\necho 'WARP3D fatal error: singular matrix'\nexit 1\n").unwrap();
            let mut perms = fs::metadata(&sh_path).unwrap().permissions();
            perms.set_mode(0o755);
            fs::set_permissions(&sh_path, perms).unwrap();
            sh_path
        }
    }

    #[test]
    fn runner_finds_linux_and_windows_warp3d_exe_and_honors_cancel() {
        let dir = temp_dir("discovery");
        // Verify Unix candidate_binary_names always includes warp3d.exe as well as warp3d.
        let names = candidate_binary_names();
        assert!(names.contains(&"warp3d.exe"));
        assert!(names.contains(&"warp3d"));

        // 1. Explicit FIELDCRAFT_WARP3D_BIN overrides WARP3D_HOME and PATH.
        let explicit = dir.join("custom_warp3d.exe");
        fs::write(&explicit, b"").unwrap();
        let found = find_warp3d_binary_in(Some(explicit.as_os_str()), None, None).unwrap();
        assert_eq!(found, explicit);

        let missing = dir.join("missing.exe");
        assert!(matches!(find_warp3d_binary_in(Some(missing.as_os_str()), None, None), Err(Warp3dError::BinaryNotFound(_))));

        // 2. WARP3D_HOME platform directory lookup (`run_windows/warp3d.exe` on Windows, `run_linux/warp3d.exe` on Linux).
        let home = dir.join("warp_home");
        fs::create_dir_all(home.join("run_windows")).unwrap();
        fs::create_dir_all(home.join("run_linux")).unwrap();
        let win_exe = home.join("run_windows").join("warp3d.exe");
        let lin_exe = home.join("run_linux").join("warp3d.exe");
        fs::write(&win_exe, b"").unwrap();
        fs::write(&lin_exe, b"").unwrap();

        let home_found = find_warp3d_binary_in(None, Some(home.as_os_str()), None).unwrap();
        if cfg!(windows) {
            assert_eq!(home_found, win_exe);
        } else if !cfg!(target_os = "macos") {
            assert_eq!(home_found, lin_exe);
        }

        // 3. PATH lookup fallback.
        let path_dir = dir.join("bin_on_path");
        fs::create_dir_all(&path_dir).unwrap();
        let path_exe = path_dir.join("warp3d.exe");
        fs::write(&path_exe, b"").unwrap();
        let joined_path: OsString = std::env::join_paths([&path_dir]).unwrap();
        assert_eq!(find_warp3d_binary_in(None, None, Some(joined_path.as_os_str())).unwrap(), path_exe);

        // 4. Mock execution + deck_path output + flat result parsing.
        let model = unit_hex8_model(false, false);
        let mock_ok = write_mock_success_script(&dir);
        let deck_out = dir.join("saved").join("job.inp");
        let cancel = AtomicBool::new(false);
        let sol = run_warp3d_with_bin(&model, "Pull", None, 2, Some(&deck_out), &cancel, &mock_ok).unwrap();
        assert!(deck_out.is_file());
        assert!(sol.stdout_log.contains("WARP3D mock complete"));
        assert!((sol.static_result.displacement[1][0] - 5.0e-4).abs() < 1e-12);
        assert!((sol.static_result.reaction_total[0] - (-1.0e8)).abs() < 1e-3);

        // 5. Solver non-zero exit returns SolverFailed with log_tail.
        let mock_fail = write_mock_fail_script(&dir);
        let err = run_warp3d_with_bin(&model, "Pull", None, 1, None, &cancel, &mock_fail).unwrap_err();
        match err {
            Warp3dError::SolverFailed { log_tail, .. } => assert!(log_tail.contains("singular matrix"), "{log_tail}"),
            other => panic!("expected SolverFailed, got {other:?}"),
        }

        // 6. Pre-set cancellation and mid-run cancellation.
        let pre_cancel = AtomicBool::new(true);
        assert!(matches!(run_warp3d_with_bin(&model, "Pull", None, 1, None, &pre_cancel, &mock_ok), Err(Warp3dError::Cancelled)));

        let mock_slow = write_mock_slow_script(&dir);
        let mid_cancel = std::sync::Arc::new(AtomicBool::new(false));
        let mid_flag = std::sync::Arc::clone(&mid_cancel);
        let killer = std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_millis(60));
            mid_flag.store(true, Ordering::Relaxed);
        });
        let res = run_warp3d_with_bin(&model, "Pull", None, 1, None, &mid_cancel, &mock_slow);
        let _ = killer.join();
        assert!(matches!(res, Err(Warp3dError::Cancelled)), "expected Cancelled, got {res:?}");

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    #[ignore = "requires local WARP3D 18.5.0"]
    fn live_warp3d_patch_and_plastic_bar() {
        let cancel = AtomicBool::new(false);
        let run_live = |model: &Model, step: &str| -> Warp3dSolution {
            match run_warp3d(model, step, None, 2, None, &cancel) {
                Ok(sol) => sol,
                Err(Warp3dError::SolverFailed { ref log_tail, .. }) if log_tail.contains("libimf.so") => {
                    // On WSL without Intel Fortran runtime, fall back to the distribution's Windows binary via interop.
                    let win_bin = PathBuf::from(std::env::var_os("WARP3D_HOME").unwrap()).join("run_windows").join("warp3d.exe");
                    run_warp3d_with_bin(model, step, None, 2, None, &cancel, &win_bin).unwrap()
                }
                Err(e) => panic!("WARP3D execution failed: {e}"),
            }
        };

        // 1. Elastic 1-Hex8 uniaxial tension patch test vs FieldCraft native solver.
        let elastic_model = unit_hex8_model(false, false);
        let warp_sol = run_live(&elastic_model, "Pull");
        let native = fieldcraft_solver::solve_static(&elastic_model, Some("Pull"), &fieldcraft_solver::Limits::default()).unwrap();

        let warp_u = warp_sol.static_result.displacement[1][0];
        let native_u = native.displacement[1][0];
        assert!((warp_u - native_u).abs() / native_u.abs() < 1e-6, "warp_u={warp_u} vs native_u={native_u}");
        let warp_vm = warp_sol.static_result.von_mises[0];
        let native_vm = native.von_mises[0];
        assert!((warp_vm - native_vm).abs() / native_vm.abs() < 1e-6, "warp_vm={warp_vm} vs native_vm={native_vm}");

        // 2. Elastoplastic bilinear bar with NonlinearStatic 4 increments.
        let plastic_model = unit_hex8_model(true, true);
        let nl_sol = run_live(&plastic_model, "Pull");
        let native_nl = fieldcraft_solver::solve_nonlinear(&plastic_model, Some("Pull"), &fieldcraft_solver::Limits::default()).unwrap();
        let r = &nl_sol.static_result;
        let h = r.nonlinear.as_ref().unwrap();
        assert_eq!(h.frames.len(), 5);
        assert_eq!(h.frames[0].load_factor, 0.0);
        assert_eq!(h.frames[4].displacement, r.displacement);
        let epspls = r.elements[0].plastic_strain[0];
        let native_epspls = native_nl.elements[0].plastic_strain[0];
        assert!(epspls > 0.0, "expected active plastic yielding, got epspls={epspls}");
        assert!((epspls - native_epspls).abs() / native_epspls < 5e-2, "warp epspls={epspls} vs native={native_epspls}");
    }

    #[test]
    #[ignore = "requires local WARP3D 18.5.0"]
    fn live_warp3d_3d_crack_domain_and_sifs() {
        // 3D Single-Edge-Notched Tension (SENT) half-plate in SI:
        // x in [0, 1.0] (crack on y=0, x in [0, 0.5]; ligament on y=0, x in [0.5, 1.0]),
        // y in [0, 1.0], z in [0, 0.5], 8x8x2 = 128 Hex8 elements.
        let mut model = Model::default();
        let (nx, ny, nz) = (8usize, 8usize, 2usize);
        let node_id = |i: usize, j: usize, k: usize| -> u32 { (1 + k * (nx + 1) * (ny + 1) + j * (nx + 1) + i) as u32 };

        for k in 0..=nz {
            let z = 0.25 * k as f64;
            for j in 0..=ny {
                let y = 0.125 * j as f64;
                for i in 0..=nx {
                    let x = 0.125 * i as f64;
                    model.set_node(node_id(i, j, k), [x, y, z]).unwrap();
                }
            }
        }

        let mut all_elems = Vec::with_capacity(nx * ny * nz);
        let mut top_faces = Vec::with_capacity(nx * nz);
        let mut eid = 1u32;
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let conn = vec![
                        node_id(i, j, k),
                        node_id(i + 1, j, k),
                        node_id(i + 1, j + 1, k),
                        node_id(i, j + 1, k),
                        node_id(i, j, k + 1),
                        node_id(i + 1, j, k + 1),
                        node_id(i + 1, j + 1, k + 1),
                        node_id(i, j + 1, k + 1),
                    ];
                    model.set_element(eid, ElementKind::Hex8, conn).unwrap();
                    all_elems.push(eid);
                    if j + 1 == ny {
                        // FieldCraft Hex8 face 5 (1-based) is +eta (y = 1.0), swapped to WARP3D face 4.
                        top_faces.push(fieldcraft_model::ElementFace { element: eid, face: 5 });
                    }
                    eid += 1;
                }
            }
        }

        let mut ligament_nodes = Vec::new();
        let mut z_sym_nodes = Vec::new();
        let mut back_x_nodes = Vec::new();
        let mut crack_front_nodes = Vec::new();
        for k in 0..=nz {
            crack_front_nodes.push(node_id(4, 0, k));
            back_x_nodes.push(node_id(nx, 0, k));
            for i in 4..=nx {
                ligament_nodes.push(node_id(i, 0, k));
            }
            if k == 0 || k == nz {
                for j in 0..=ny {
                    for i in 0..=nx {
                        z_sym_nodes.push(node_id(i, j, k));
                    }
                }
            }
        }

        let e_mod = 2.0e11_f64;
        let nu = 0.3_f64;
        model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: e_mod, poisson: nu }), ..Default::default() }).unwrap();
        model.add_set("all", Set { elements: all_elems, ..Set::default() }).unwrap();
        model.add_set("ligament", Set { nodes: ligament_nodes, ..Set::default() }).unwrap();
        model.add_set("z_faces", Set { nodes: z_sym_nodes, ..Set::default() }).unwrap();
        model.add_set("back_x", Set { nodes: back_x_nodes, ..Set::default() }).unwrap();
        model.add_set("crack_front", Set { nodes: crack_front_nodes, ..Set::default() }).unwrap();
        model.add_set("top_faces", Set { faces: top_faces, ..Set::default() }).unwrap();

        model
            .add_section(
                "solid",
                Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() },
            )
            .unwrap();
        model
            .add_boundary_condition(
                "sym_y",
                BoundaryCondition::Symmetry { target: Target::Set("ligament".into()), plane: fieldcraft_model::Symmetry::Y },
            )
            .unwrap();
        model
            .add_boundary_condition(
                "plane_strain_z",
                BoundaryCondition::Displacement { target: Target::Set("z_faces".into()), values: [None, None, Some(0.0), None, None, None] },
            )
            .unwrap();
        model
            .add_boundary_condition(
                "fix_x_rig",
                BoundaryCondition::Displacement { target: Target::Set("back_x".into()), values: [Some(0.0), None, None, None, None, None] },
            )
            .unwrap();
        // Outward tension sigma = 100 MPa (negative inward pressure).
        model.add_load("tension", Load::Pressure { set: "top_faces".into(), magnitude: -1.0e8 }).unwrap();

        let mut step = Step::new("Load", Procedure::Static {});
        step.boundary_conditions.insert("sym_y".into());
        step.boundary_conditions.insert("plane_strain_z".into());
        step.boundary_conditions.insert("fix_x_rig".into());
        step.loads.insert("tension".into());
        model.add_step(step, None).unwrap();

        let crack = CrackFrontSpec {
            set: "crack_front".into(),
            normal: None,
            symmetric: None,
            rings: Some(4),
            interaction_integral: Some(true),
            function_type: crate::CrackFunctionType::Pointwise,
        };
        let cancel = AtomicBool::new(false);
        let sol = match run_warp3d(&model, "Load", Some(&crack), 2, None, &cancel) {
            Ok(sol) => sol,
            Err(Warp3dError::SolverFailed { ref log_tail, .. }) if log_tail.contains("libimf.so") => {
                let win_bin = PathBuf::from(std::env::var_os("WARP3D_HOME").unwrap()).join("run_windows").join("warp3d.exe");
                run_warp3d_with_bin(&model, "Load", Some(&crack), 2, None, &cancel, &win_bin).unwrap()
            }
            Err(e) => panic!("WARP3D fracture execution failed: {e}"),
        };

        assert_eq!(sol.fracture.len(), 3, "expected 3 crack-front stations, stdout:\n{}", sol.stdout_log);
        let e_prime = e_mod / (1.0 - nu * nu);
        let expected_s = [0.0, 0.5, 1.0];
        for (i, d) in sol.fracture.iter().enumerate() {
            println!(
                "{}: s={:?}, J={:.3e} N/m, spread={:.4}, K_J(pstrn)={:?}, K_I={:?}, T11={:?}, T33={:?}",
                d.domain_id, d.s_position, d.j_mean, d.ring_spread, d.k_j_plane_strain, d.k_i_mean, d.t_11_mean, d.t_33_mean
            );
            assert_eq!(d.rings.len(), 4);
            assert!(d.area_aq > 0.0);
            assert!(d.j_mean > 0.0 && d.j_min > 0.0 && d.j_max >= d.j_min);
            assert!(d.ring_spread.is_finite() && d.ring_spread >= 0.0);
            assert_eq!(d.front_nodes.len(), 1);
            assert!((d.s_position.unwrap() - expected_s[i]).abs() < 1e-9);
            assert!((d.normal[0]).abs() < 1e-6 && (d.normal[1] - 1.0).abs() < 1e-6 && (d.normal[2]).abs() < 1e-6);
            let t_norm = (d.tangent[0].powi(2) + d.tangent[1].powi(2) + d.tangent[2].powi(2)).sqrt();
            let a_norm = (d.advance[0].powi(2) + d.advance[1].powi(2) + d.advance[2].powi(2)).sqrt();
            assert!((t_norm - 1.0).abs() < 1e-6);
            assert!((a_norm - 1.0).abs() < 1e-6);
            let dot_nt = d.normal[0] * d.tangent[0] + d.normal[1] * d.tangent[1] + d.normal[2] * d.tangent[2];
            assert!(dot_nt.abs() < 1e-6);

            let kj = d.k_j_plane_strain.unwrap();
            let ki = d.k_i_mean.unwrap();
            let kj_from_j = (d.j_mean * e_prime).sqrt();
            assert!((kj - kj_from_j).abs() / kj_from_j < 0.05, "kj={kj} vs sqrt(J*E')={kj_from_j}");
            assert!((ki - kj).abs() / kj < 0.15, "ki={ki} vs kj={kj}");
            assert_eq!(d.k_ii_mean, Some(0.0));
            assert_eq!(d.k_iii_mean, Some(0.0));
            assert!(d.t_11_mean.unwrap().is_finite());
            assert!(d.t_33_mean.unwrap().is_finite());
        }
    }

    #[test]
    fn runner_drains_over_64mib_stdout_parses_late_domains_and_joins_on_cancel_and_timeout() {
        let dir = temp_dir("stream_timeout");
        let model = unit_hex8_model(false, false);

        // 1. Timeout kills child, joins reader/writer threads promptly, and returns Timeout with tail.
        let slow_bin = write_mock_slow_script(&dir);
        let cancel = AtomicBool::new(false);
        let t0 = std::time::Instant::now();
        let err =
            run_warp3d_with_bin_and_timeout(&model, "Pull", None, 1, None, &cancel, &slow_bin, std::time::Duration::from_millis(60)).unwrap_err();
        assert!(matches!(err, Warp3dError::Timeout { .. }), "expected Timeout, got {err}");
        assert!(t0.elapsed() < std::time::Duration::from_secs(5), "timeout waited on unjoined subprocess: {:?}", t0.elapsed());
        if let Warp3dError::Timeout { log_tail, .. } = &err {
            assert!(cfg!(windows) || log_tail.contains("mock slow start"), "tail={log_tail:?}");
        }

        // 2. Over-64 MiB stdout filler followed by late Domain output streams cleanly without truncation or OOM.
        #[cfg(not(windows))]
        let flood_bin = {
            use std::os::unix::fs::PermissionsExt;
            let sh_path = dir.join("mock_flood.sh");
            let script = r#"#!/bin/sh
cat > /dev/null
cat <<'EOF' > wnd00001_text
0 0 0
0 0 0
0 0 0
0 0 0
0 0 0
0 0 0
0 0 0
0 0 0
EOF
cat <<'EOF' > wnr00001_text
0 0 0
0 0 0
0 0 0
0 0 0
0 0 0
0 0 0
0 0 0
0 0 0
EOF
zeros="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0"
printf "%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n" \
  "$zeros" "$zeros" "$zeros" "$zeros" "$zeros" "$zeros" "$zeros" "$zeros" > wns00001_text
printf "%s\n" "$zeros" > wes00001_text
awk 'BEGIN { for (i = 0; i < 1300000; i++) printf "iter %08d residual = 1.0e-12 00000000000000000000\n", i }'
printf "loading: Pull step: 1 lambda = 1.0\n"
printf "Domain: d_late (step 1)\n"
printf "  Area under q along front: 10.0\n"
printf "  J_mean: 42.0 J_min: 40.0 J_max: 44.0\n"
printf "  Ring J\n"
printf "    1 40.0\n"
printf "    2 44.0\n"
printf "WARP3D late domain complete\n"
"#;
            fs::write(&sh_path, script).unwrap();
            let mut perms = fs::metadata(&sh_path).unwrap().permissions();
            perms.set_mode(0o755);
            fs::set_permissions(&sh_path, perms).unwrap();
            sh_path
        };

        #[cfg(not(windows))]
        {
            let progress = fieldcraft_model::progress::Progress::new();
            let sol = fieldcraft_model::progress::scoped(Some(progress.clone()), || {
                run_warp3d_with_bin(&model, "Pull", None, 1, None, &cancel, &flood_bin).unwrap()
            });
            assert_eq!(sol.fracture.len(), 1);
            assert_eq!(sol.fracture[0].domain_id, "d_late");
            assert!((sol.fracture[0].j_mean - 42.0).abs() < 1e-6);
            assert!(sol.stdout_log.len() <= LOG_TAIL_CHARS);
            assert!(sol.stdout_log.contains("WARP3D late domain complete"));
            assert!(progress.observations() > 0);
        }

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn override_precedence_path_safety_and_preflight_binary_before_deck() {
        let dir = temp_dir("override_safety");
        let ovr = dir.join("ovr").join("run_linux").join("warp3d.exe");
        fs::create_dir_all(ovr.parent().unwrap()).unwrap();
        fs::write(&ovr, b"").unwrap();
        let ovr_lib = dir.join("ovr").join("linux_packages").join("lib");
        fs::create_dir_all(&ovr_lib).unwrap();

        let env_bin = dir.join("env_warp3d.exe");
        fs::write(&env_bin, b"").unwrap();

        let home = dir.join("home");
        fs::create_dir_all(home.join("run_linux")).unwrap();
        fs::create_dir_all(home.join("run_windows")).unwrap();
        let home_lib = home.join("linux_packages").join("lib");
        fs::create_dir_all(&home_lib).unwrap();
        fs::write(home.join("run_linux").join("warp3d.exe"), b"").unwrap();
        fs::write(home.join("run_windows").join("warp3d.exe"), b"").unwrap();

        // 1. PrefsOverride beats FIELDCRAFT_WARP3D_BIN, WARP3D_HOME, and PATH.
        let s = resolve_warp3d_binary_in(Some(&ovr), Some(env_bin.as_os_str()), Some(home.as_os_str()), None);
        assert_eq!(s.source, Warp3dBinarySource::PrefsOverride);
        assert!(s.valid);
        assert_eq!(s.path.as_deref(), Some(ovr.as_path()));

        // 2. Invalid/missing override never silently falls back to env or WARP3D_HOME.
        let missing = dir.join("no_such_warp3d");
        let s_bad = resolve_warp3d_binary_in(Some(&missing), Some(env_bin.as_os_str()), Some(home.as_os_str()), None);
        assert_eq!(s_bad.source, Warp3dBinarySource::PrefsOverride);
        assert!(!s_bad.valid);

        let s_empty = resolve_warp3d_binary_in(Some(Path::new("")), Some(env_bin.as_os_str()), Some(home.as_os_str()), None);
        assert_eq!(s_empty.source, Warp3dBinarySource::PrefsOverride);
        assert!(!s_empty.valid);

        // 3. Invalid FIELDCRAFT_WARP3D_BIN keeps EnvFieldcraftBin source and does not fall through.
        let s_env_bad = resolve_warp3d_binary_in(None, Some(missing.as_os_str()), Some(home.as_os_str()), None);
        assert_eq!(s_env_bad.source, Warp3dBinarySource::EnvFieldcraftBin);
        assert!(!s_env_bad.valid);

        // 4. Relative, UNC, oversized, and NUL paths are rejected cleanly.
        assert!(!resolve_warp3d_binary_in(Some(Path::new("relative/warp3d")), None, None, None).valid);
        let unc = resolve_warp3d_binary_in(Some(Path::new(r"\\server\share\warp3d.exe")), None, None, None);
        assert!(!unc.valid);
        assert!(unc.error.as_deref().unwrap_or("").contains("UNC"));
        let too_long = PathBuf::from(format!("/{}.exe", "a".repeat(4100)));
        assert!(!resolve_warp3d_binary_in(Some(&too_long), None, None, None).valid);

        // 5. Linux sibling library directory takes precedence under override.
        assert_eq!(extra_linux_lib_dir_in(&ovr, true, Some(home.as_os_str())).as_deref(), Some(ovr_lib.as_path()));
        assert_eq!(extra_linux_lib_dir_in(&ovr, false, Some(home.as_os_str())).as_deref(), Some(home_lib.as_path()));

        // 6. Missing override fails with BinaryNotFound before write_deck / deck_path file creation.
        let bad_model = Model::default();
        let deck_out = dir.join("should_not_exist").join("out.inp");
        let cancel = AtomicBool::new(false);
        let err = run_warp3d_with_override(&bad_model, "", None, 1, Some(&deck_out), &cancel, Some(&missing)).unwrap_err();
        assert!(matches!(err, Warp3dError::BinaryNotFound(ref msg) if msg.contains("Help > Settings")), "got {err}");
        assert!(!deck_out.exists());

        let _ = fs::remove_dir_all(&dir);
    }
}

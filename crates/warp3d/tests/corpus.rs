//! Corpus sweep over the external WARP3D 18.5 distribution (`WARP3D_HOME`).
//!
//! Returns early without failing when `WARP3D_HOME` is unset or absent so CI runs without the
//! external distribution stay green. When `WARP3D_HOME` points at a WARP3D distribution checkout,
//! discovers every verification deck via `driver.inp` files plus the non-driver verification
//! suites and Chapter 1 manual example, classifies each deck into Bucket A/B/C, asserts zero
//! panics, Bucket C == 0, and Bucket A on all five PR 2 anchor suites.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::{Component, Path, PathBuf};

use fieldcraft_model::Severity;
use fieldcraft_model::steps::{Procedure, Step};
use fieldcraft_model::units::Preset;
use fieldcraft_warp3d::reader::{IncludeRequest, ReadError, ReadLimits, ResolvedInclude, read_with_includes};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Bucket {
    A,
    B,
    C,
}

#[derive(Debug)]
struct EntryOutcome {
    rel_path: String,
    bucket: Bucket,
    detail: String,
}

fn rel_slash(root: &Path, path: &Path) -> String {
    let rel = path.strip_prefix(root).unwrap_or(path);
    rel.to_string_lossy().replace('\\', "/")
}

fn collect_driver_inps(dir: &Path, out: &mut Vec<PathBuf>) -> Result<(), String> {
    let mut entries: Vec<PathBuf> =
        fs::read_dir(dir).map_err(|e| format!("read_dir {}: {e}", dir.display()))?.filter_map(|e| e.ok().map(|de| de.path())).collect();
    entries.sort();
    for path in entries {
        if path.is_dir() {
            collect_driver_inps(&path, out)?;
        } else if path.file_name().and_then(|n| n.to_str()) == Some("driver.inp") {
            out.push(path);
        }
    }
    Ok(())
}

/// Parse one `verification/**/driver.inp` using the exact rules in `verification/run_tests.py`.
fn parse_driver_inp(driver_path: &Path) -> Result<Vec<PathBuf>, String> {
    let raw = fs::read(driver_path).map_err(|e| format!("read {}: {e}", driver_path.display()))?;
    let text = String::from_utf8_lossy(&raw);
    let mut lines = text.lines();

    let next_data = |iter: &mut std::str::Lines<'_>| -> Result<String, String> {
        for raw_line in iter.by_ref() {
            let trimmed = raw_line.trim();
            if trimmed.is_empty() {
                continue;
            }
            let first = trimmed.split_whitespace().next().unwrap_or("");
            if matches!(first, "*" | "!" | "c" | "C" | "#" | "/") {
                continue;
            }
            return Ok(raw_line.trim_end_matches(&['\r', '\n'][..]).to_string());
        }
        Err(format!("unexpected EOF in {}", driver_path.display()))
    };

    // Title 1 via comment-skipping read; Title 2 is one raw line per run_tests.py.
    let _title1 = next_data(&mut lines)?;
    let _title2 = lines.next().ok_or_else(|| format!("missing title2 in {}", driver_path.display()))?;
    let num_line = next_data(&mut lines)?;
    let num_str = num_line.split_whitespace().next().ok_or_else(|| format!("missing problem count in {}", driver_path.display()))?;
    let n: usize = num_str.parse().map_err(|e| format!("bad problem count `{num_str}` in {}: {e}", driver_path.display()))?;

    // Skip N test IDs and N output filenames, then read N input filenames.
    for _ in 0..n {
        let _id = next_data(&mut lines)?;
    }
    for _ in 0..n {
        let _outfile = next_data(&mut lines)?;
    }
    let dir = driver_path.parent().ok_or_else(|| format!("no parent for {}", driver_path.display()))?;
    let mut decks = Vec::with_capacity(n);
    for _ in 0..n {
        let infile = next_data(&mut lines)?;
        let trimmed = infile.trim();
        decks.push(dir.join(trimmed));
    }
    Ok(decks)
}

fn discover_corpus(warp3d_home: &Path) -> Result<Vec<PathBuf>, String> {
    let verification_dir = warp3d_home.join("verification");
    let mut driver_files = Vec::new();
    collect_driver_inps(&verification_dir, &mut driver_files)?;

    let mut discovered = Vec::new();
    let mut seen = BTreeSet::new();
    let mut push_unique = |p: PathBuf| {
        if seen.insert(p.clone()) {
            discovered.push(p);
        }
    };

    for driver in driver_files {
        for deck in parse_driver_inp(&driver)? {
            push_unique(deck);
        }
    }

    // Two verification suites driven directly in verification/run_tests.py without driver.inp:
    for rel in [
        "verification/test44/test_44_ppr",
        "verification/test44/test_44_ppr_restart",
        "verification/test44/test_44_exp1",
        "verification/test44/test_44_exp1_restart",
        "verification/test81/test_81",
        "manual_examples_chpt1/example_1/warp3d.inp",
    ] {
        push_unique(warp3d_home.join(rel));
    }

    Ok(discovered)
}

fn load_relative_include(deck_dir_canon: &Path, req: IncludeRequest<'_>) -> Result<ResolvedInclude, ReadError> {
    let raw = req.target.trim();
    if raw.is_empty() || raw.starts_with("~/") || raw.starts_with("~\\") || raw.contains('\\') || raw.contains(':') {
        return Err(ReadError::IncludeRefused(format!("include target `{}` is outside deck directory", req.target)));
    }
    let rel = Path::new(raw);
    for comp in rel.components() {
        match comp {
            Component::CurDir | Component::Normal(_) => {}
            Component::ParentDir | Component::RootDir | Component::Prefix(_) => {
                return Err(ReadError::IncludeRefused(format!("include target `{}` escapes deck directory", req.target)));
            }
        }
    }
    let candidate = deck_dir_canon.join(rel);
    let canon = candidate.canonicalize().map_err(|_| ReadError::IncludeRefused(format!("include target `{}` could not be opened", req.target)))?;
    if !canon.starts_with(deck_dir_canon) {
        return Err(ReadError::IncludeRefused(format!("include target `{}` escapes deck directory", req.target)));
    }
    let meta = fs::metadata(&canon).map_err(|_| ReadError::IncludeRefused(format!("include target `{}` could not be stated", req.target)))?;
    if meta.len() > req.remaining_bytes as u64 {
        return Err(ReadError::Limit(format!(
            "cumulative input bytes exceed {} MiB cap",
            fieldcraft_warp3d::reader::MAX_INPUT_BYTES / (1024 * 1024)
        )));
    }
    let bytes = fs::read(&canon).map_err(|_| ReadError::IncludeRefused(format!("include target `{}` could not be read", req.target)))?;
    if bytes.len() > req.remaining_bytes {
        return Err(ReadError::Limit(format!(
            "cumulative input bytes exceed {} MiB cap",
            fieldcraft_warp3d::reader::MAX_INPUT_BYTES / (1024 * 1024)
        )));
    }
    let display_name = canon.strip_prefix(deck_dir_canon).unwrap_or(&canon).to_string_lossy().replace('\\', "/");
    let canonical_key = canon.to_string_lossy().into_owned();
    Ok(ResolvedInclude { display_name, canonical_key, bytes })
}

fn verify_imported_model(imported: &fieldcraft_warp3d::reader::ImportedDeck) -> Result<(), String> {
    let unexpected_errors: Vec<String> = imported
        .model
        .check()
        .into_iter()
        .filter(|d| d.severity == Severity::Error && !(d.subject == "model" && d.message == "there is no analysis step"))
        .map(|d| d.to_string())
        .collect();
    if !unexpected_errors.is_empty() {
        return Err(format!("model.check() errors: {}", unexpected_errors.join("; ")));
    }

    let mut clone = imported.model.clone();
    let mut step = Step::new("corpus_verify", Procedure::Static {});
    step.boundary_conditions = clone.boundary_conditions().keys().cloned().collect();
    step.loads = clone.loads().keys().cloned().collect();
    clone.add_step(step, None).map_err(|e| format!("add synthetic step: {e}"))?;
    clone.resolve().map_err(|e| format!("resolve with synthetic step: {e}"))?;
    Ok(())
}

fn run_one_deck(warp3d_home: &Path, deck_path: &Path) -> EntryOutcome {
    let rel_path = rel_slash(warp3d_home, deck_path);
    let root_bytes = match fs::read(deck_path) {
        Ok(b) => b,
        Err(e) => return EntryOutcome { rel_path, bucket: Bucket::C, detail: format!("failed to read root deck: {e}") },
    };
    let deck_dir = match deck_path.parent().and_then(|p| p.canonicalize().ok()) {
        Some(d) => d,
        None => return EntryOutcome { rel_path, bucket: Bucket::C, detail: "failed to canonicalize deck parent dir".into() },
    };

    let result = catch_unwind(AssertUnwindSafe(|| {
        read_with_includes(&rel_path, &root_bytes, None, Preset::InLbfS, ReadLimits::default(), |req| load_relative_include(&deck_dir, req))
    }));

    match result {
        Err(payload) => {
            let msg = if let Some(s) = payload.downcast_ref::<&str>() {
                (*s).to_string()
            } else if let Some(s) = payload.downcast_ref::<String>() {
                s.clone()
            } else {
                "non-string panic payload".to_string()
            };
            EntryOutcome { rel_path, bucket: Bucket::C, detail: format!("PANIC: {msg}") }
        }
        Ok(Ok(imported)) => match verify_imported_model(&imported) {
            Ok(()) => EntryOutcome {
                rel_path,
                bucket: Bucket::A,
                detail: format!(
                    "nodes={}, elems={}, bcs={}, loads={}, includes={}",
                    imported.model.nodes().len(),
                    imported.model.elements().len(),
                    imported.model.boundary_conditions().len(),
                    imported.model.loads().len(),
                    imported.includes_read.len()
                ),
            },
            Err(reason) => EntryOutcome { rel_path, bucket: Bucket::C, detail: reason },
        },
        Ok(Err(e @ ReadError::Unsupported { .. })) => EntryOutcome { rel_path, bucket: Bucket::B, detail: e.to_string() },
        Ok(Err(ReadError::IncludeRefused(r))) => EntryOutcome { rel_path, bucket: Bucket::B, detail: r },
        Ok(Err(ReadError::Parse { file, line, message })) => {
            EntryOutcome { rel_path, bucket: Bucket::C, detail: format!("Parse {file}:{line}: {message}") }
        }
        Ok(Err(ReadError::Model(e))) => EntryOutcome { rel_path, bucket: Bucket::C, detail: format!("Model: {e}") },
        Ok(Err(ReadError::Limit(e))) => EntryOutcome { rel_path, bucket: Bucket::C, detail: format!("Limit: {e}") },
    }
}

#[test]
#[ignore = "heavy: cargo xtask test --heavy (CI heavy job)"]
fn warp3d_corpus_sweep() {
    let Some(home_os) = std::env::var_os("WARP3D_HOME") else {
        eprintln!("corpus: WARP3D_HOME not set; skipping sweep cleanly.");
        return;
    };
    let home = PathBuf::from(&home_os);
    if !home.is_dir() {
        eprintln!("corpus: WARP3D_HOME={} is not a directory; skipping sweep cleanly.", home.display());
        return;
    }

    let decks = discover_corpus(&home).unwrap_or_else(|e| panic!("corpus discovery failed: {e}"));
    assert!(!decks.is_empty(), "expected non-empty corpus under {}", home.display());

    let mut by_path: BTreeMap<String, EntryOutcome> = BTreeMap::new();
    let mut bucket_a = Vec::new();
    let mut bucket_b = Vec::new();
    let mut bucket_c = Vec::new();

    for deck in &decks {
        let outcome = run_one_deck(&home, deck);
        match outcome.bucket {
            Bucket::A => bucket_a.push(format!("{} ({})", outcome.rel_path, outcome.detail)),
            Bucket::B => bucket_b.push(format!("{} => {}", outcome.rel_path, outcome.detail)),
            Bucket::C => bucket_c.push(format!("{} => {}", outcome.rel_path, outcome.detail)),
        }
        by_path.insert(outcome.rel_path.clone(), outcome);
    }

    eprintln!(
        "=== WARP3D corpus summary: total={}, Bucket A={}, Bucket B={}, Bucket C={} ===",
        decks.len(),
        bucket_a.len(),
        bucket_b.len(),
        bucket_c.len()
    );
    for a in &bucket_a {
        eprintln!("  [A] {a}");
    }
    for b in &bucket_b {
        eprintln!("  [B] {b}");
    }
    for c in &bucket_c {
        eprintln!("  [C] {c}");
    }

    let required_anchors = [
        "manual_examples_chpt1/example_1/warp3d.inp",
        "verification/test14/test_14",
        "verification/test51/test_51a",
        "verification/test51/test_51b",
        "verification/test51/test_51c",
        "verification/test51/test_51d",
        "verification/test72/test72",
        "verification/test87/test_87",
    ];
    for anchor in required_anchors {
        let Some(outcome) = by_path.get(anchor) else {
            panic!("required anchor `{anchor}` was not discovered in corpus under {}", home.display());
        };
        assert_eq!(outcome.bucket, Bucket::A, "anchor `{anchor}` expected Bucket A, got {:?}: {}", outcome.bucket, outcome.detail);
    }

    assert!(bucket_c.is_empty(), "expected Bucket C == 0 (0 panics / 0 parse errors), found {}:\n{}", bucket_c.len(), bucket_c.join("\n"));
}

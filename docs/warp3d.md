# WARP3D Integration (`fieldcraft-warp3d`)

`fieldcraft-warp3d` integrates the open-source [WARP3D](https://github.com/warp3d/warp3d) nonlinear solid and fracture mechanics solver into FieldCraft.

## Layering and Scope

`fieldcraft-warp3d` sits at **Layer 2** in [`docs/architecture.md`](architecture.md) alongside `fieldcraft-solver`, `fieldcraft-explicit`, and `fieldcraft-lbm`, with intra-layer order `solver -> warp3d` in `xtask/src/layers.rs`.

Placing it at L2 instead of inside `fieldcraft-decks` (L1) keeps L1 strictly for pure file-format serialization while allowing `fieldcraft-warp3d` to own the full solver adapter lifecycle:

1. **Implemented (Stages 1–4A + Deck Import PR 1 & PR 2):** Complete ASCII `.inp` deck generation (geometry, `LabelMap` section grouping, materials, constraints/MPCs, loads, nonlinear analysis parameters, flat-text outputs), multi-file ASCII model-data import via `deck.read` (`"format": "warp3d"`, relative `*input`/`*inout` includes with L6 dual confinement, `*parameter`/`@param` macros, nodes, 3D solid elements + collapsed `l3disop` wedges lowered to `wedge6`, elastic/bilinear/table/`n_power > 1.0` Mises materials, sections, explicit `list` node sets, `constraints` including deferred `plane` and single-node zero-RHS `multipoint` symmetry, and unscaled static `loading` patterns), crack-front node ordering and `domain` J/interaction-integral command emission, external `warp3d` binary discovery/execution with streaming stdout/stderr drain, cancellation, and timeout, flat-file displacement/reaction/stress/plastic-strain parsing and `J` / `K_I` / `K_II` / `K_III` / `T`-stress domain-integral parsing back into SI, `deck.write` (`"format": "warp3d"` + `"crack_front"`), `solve.static` / `solve.nonlinear` (`backend: "warp3d"` / `FIELDCRAFT_STRUCTURAL_BACKEND`), `results.read` (`"format": "warp3d"`), `results.export.csv` (`"what": "fracture"`), and `wasm32`-clean runner/result/reader interfaces.
2. **Future Enhancements (Beyond Gate 4):** Native `.fieldcraft` binary persistence of WARP3D fracture domain integrals (currently session-only with save warnings and CSV export) and expanded coupled/contact solver features.

## Key Design Notes for Reviewers

### SI Resolution Before Unit Conversion

FieldCraft stores every model internally in SI (`m`, `N`, `Pa`, `kg`). Because WARP3D input decks are unitless and assume a single user-chosen consistent unit system:

1. `write_deck` resolves the SI `Model` first so section assignments, material references, and loads are validated with FieldCraft's calibrated SI checks.
2. It converts numeric model quantities through `DisplaySystem::new(units)` via `Model::converted` at the boundary (consistent with `docs/units.md`, with zero hardcoded conversion factors in `crates/warp3d`).
3. It checks step procedure (`Static` / `NonlinearStatic`) and uses the step's nonlinear settings or linear-static solver defaults.

### Element Grouping (`LabelMap`) and Blocking

WARP3D requires contiguous 1-based node and element IDs and groups element properties by ranges sharing a constitutive block, formulation, and material:

- `LabelMap::from_model` assigns dense 1-based IDs and sorts/groups elements by `(ElementKind, section_name)`.
- Each contiguous run is written under `elements`, all blocks share `blocking automatic size = 128 display`, and element incidences match FieldCraft's solid node ordering directly.

### Surface Face Mapping

- Solid node incidences match WARP3D directly, while surface pressure loads convert FieldCraft's 1-based face numbers via `warp3d_face`:
- Hex faces `4` (`+xi`) and `5` (`+eta`) are swapped relative to WARP3D `l3disop`/`q3disop`; tetrahedral faces `1..=4` map 1:1.

### Material and Quasi-Static Nonlinear Conventions

- **Elastic (`mises`):** Written as `properties mises` with `yld_pt 1.0e20` and `n_power 1.0` so linear-elastic runs execute cleanly through WARP3D's constitutive pipeline without yielding.
- **Bilinear plasticity:** FieldCraft stores isotropic/kinematic plastic hardening modulus $H$, whereas WARP3D expects tangent modulus $E_t$. `write_deck` converts this as `tan_e = (young * hardening_modulus) / (young + hardening_modulus)` (or `0.0` when `hardening_modulus <= 0.0`).
- **Piecewise stress–strain tables:** WARP3D `stress-strain curve` tables expect **total strain** $\varepsilon = \sigma / E + \varepsilon^p$, which `write_deck` reconstructs from FieldCraft's `(strain, stress)` hardening points before emitting materials.
- **Mass density and time step:** Static/quasi-static decks emit `rho 0.0` on materials and a large pseudo-time step (`time step 1.0e06`), converting gravity loads into explicit body forces per section so inertial terms do not enter static equilibria.

### Line Length and Formatting

WARP3D's input parser enforces fixed record width limits. Every generated line passes through `push_wrapped`, which wraps long lines cleanly at 72 characters using WARP3D line continuation syntax.

### Crack-Front Ordering and `domain` Integral Commands

When a `CrackFrontSpec` is provided:

1. `order_crack_front` chains an unordered crack-front node set along shared hex element edges into an ordered front (handling both open fronts and closed loops, plus `Hex20` midside nodes).
2. `resolve_crack_normal_and_symmetry` resolves the crack-plane normal and half-symmetry scaling from explicit spec fields or symmetry BCs.
3. `emit_domain_commands` builds per-station ring ranges (`build_domain_stations`) with WARP3D q-function types `'a'`, `'b'`, `'c'`, or `'d'` and emits `domain` blocks (with optional interaction-integral parameters).

### Determinism, Safety, and `wasm32` Compatibility

- `#![forbid(unsafe_code)]` and strict panic-denial lints (`unwrap`, `expect`, `panic!`, `todo!`, `unreachable!`, `unimplemented!`) match FieldCraft workspace rules.
- All collections are ordered deterministically so identical models produce byte-identical decks across platforms.
- Subprocess execution in `runner.rs` is gated behind `#[cfg(not(target_arch = "wasm32"))]`, keeping `cargo check -p fieldcraft-warp3d --target wasm32-unknown-unknown` green.

### Flat ASCII Deck Import (`deck.read` for `"warp3d"`, PR 1)

`crates/warp3d/src/reader/` imports single-file WARP3D 18.5.0 ASCII decks into SI `Model`s without modifying `crates/model`, `crates/decks`, or `deck.rs`:

- **Supported model data:** `structure`, `number [of] nodes/elements` (mismatch emits a warning without pre-allocation), `stress-strain curve` (`pchip`/`smooth` warn and import as piecewise linear; `temperature` is refused), `material` (`linear_elastic`/`linear`, `bilinear` with isotropic hardening, `mises` with `curve`, `n_power == 1.0`, or `yld_pt`/`sig_o`, `rho > 0`, `beta == 1.0`, and ignored `alpha != 0` with a warning), `elements` (`l3disop` Full/Reduced, `q3disop`, `tet4`, `tet10`, with sequence-ordered `all` and explicit ID/range overrides), `coordinates`, `incidences`, and explicit integer `list` node sets (lists referenced solely by `elements` stay parser-internal; geometric lists warn-and-ignore unless referenced by `elements`).
- **Units precedence:** `explicit params["units"] > ! fieldcraft-units: <preset> header (first 128 lines) > active session display preset` (`default_units` fallback). Explicit `"units": null` is treated identically to omitted `"units"`, and `"units_source"` reports `"explicit"`, `"header"`, or `"session"` (session fallback adds a warning).
- **Lexical rules & early halt:** Full-line col-1 `!`, `#`, and `c`/`C` comments plus post-`$` inline comments decode lossily while active pre-`$` slices validate UTF-8 strictly; `,` continuations bridge comments and blanks (disallowing `*` directives mid-continuation). Parsing halts immediately on the first `compute` statement (`history_imported: false`, `"stopped_at": {"file", "line", "reason": "compute"}`) without lexing post-`compute` input; `stop`, `exit`, and `quit` end the deck cleanly.
- **Skipped blocks & explicit refusals:** `blocking`, `nonlinear|dynamic|solution [analysis] parameters`, pre-`compute` `output`/`save`/`domain`, and PR 1 `constraints`/`loading` blocks are skipped and recorded in `unknown` / `unknown_count`. PR 1 refuses `*input`/`*inout`/`*parameter`, binary/stream mesh files (`coordinates`/`incidences [from] file`), `mises` `n_power > 1.0`, `beta != 1.0`, unsupported constitutive laws (`gurson`, `umat`, `cohesive`, `creep`, `cyclic`, `curves`, `deformation`, `killable`, `mises_hydrogen`), unsupported elements (`bar2`, `link2`, `inter_8`, `trint*`, `ts*`), and unsupported top-level features (`restart`, `retrieve`, `tie mesh`, `release`, `initial`, `contact`, `thermal`, `functionally`, `crystal`, `crack`) without modifying the current session.
- **Bounds & cancellation:** Enforces `ReadLimits` (`nodes`, `elements`, `members`) before expanding `a-b [by k]` / `a to b [by k]` ranges, caps input/expanded sizes and record counts, and checks `fieldcraft_model::progress::check()` every 256 records.

### Multi-File Includes, Macros, Power-Law Hardening, BCs/Loads & Corpus Verification (PR 2)

PR 2 extends `crates/warp3d/src/reader/`, `crates/engine/src/cmd/deck.rs`, and `crates/ui-egui/src/reply.rs` without changing Layer 0–1 model contracts or the WARP3D deck writer (`deck.rs`):

- **Relative ASCII `*input`/`*inout` includes & L6 dual confinement:** Resolves relative include paths from the root deck directory up to nesting depth `9` (`10` open frames), `256` total includes, and the shared `64 MiB` cumulative raw-byte cap, rejecting active canonical-path cycles cleanly. Layer 6 validates include targets lexically (rejecting empty targets, `~/`, backslashes `\`, `:` ADS/drive markers, and `..` / root components), checks both directory confinement (`fieldcraft_io::Roots::new`) and `Session::file_access`, reads with capped size, redacts host directory prefixes from include errors, and refuses `*input` on `wasm32` (`ReadError::IncludeRefused`).
- **Single-pass `*parameter` / `@param` macro expansion:** Case-sensitive macro definitions (`300` max active definitions, `1024`-byte max value, `30` substitutions per physical line) expand inside shared single/double quotes in stage 4 of lexing without re-interpreting `$` characters inserted by macro expansion as comments.
- **Power-law hardening (`mises` `n_power > 1.0`) & implicit constitutive defaults:** Discretizes `sigma = sig_o * (eps / eps_0)^(1 / n)` over `64` log-spaced plastic-strain points (`DELTA1 = 1e-4` past yield up to `0.5` total strain, emitting a clamping warning if `n_power` yield-onset behavior clamps). Implicit constitutive declarations without `linear_elastic`/`bilinear`/`mises` keywords infer `Bilinear` (`tan_e`), `Mises` (`n_power`/`curve`/`yld_pt`), or `LinearElastic` (`e` + `nu`).
- **`constraints`, `multipoint` symmetry, and unscaled static `loading`:** Imports nodal DOF rows (`u/v/w`, `all`, `a to b by k`, conflict tolerance `1e-15`), deferred `plane` constraints evaluated in SI against model bounding boxes (`1e-5 * span` default tolerance or explicit `proximity`, warn-and-skip on zero matches), and single-node zero-RHS `multipoint` equations lowered to `BoundaryCondition::Symmetry` (refusing multi-node MPCs, non-zero RHS, `rigid`, and `transformation matrix` skew frames). Static `loading <name>` patterns accumulate nodal forces and element `pressure`/`tractions`/`body [forces]` per pattern (swapping Hex faces `4 <-> 5` unless `face abaqus`, ignoring thermal `temperature` element loads with a warning, and dropping zero net loads). Imported loads remain unscaled until placed in a step; `deck.read` replies include `"boundary_conditions"`, `"loads"`, and `"includes_read"`, and the UI context panel renders an amber unscaled-load callout with a **Create Step…** action.
- **Collapsed `l3disop` wedges (`test63`, `test87`):** Degenerate 8-node `l3disop` elements whose quad face collapses along two opposite edges leaving 6 distinct vertices (commonly used in focused quarter-point/rosette crack-tip rings) are imported as right-handed `ElementKind::Wedge6` with exact face-load mapping from original hex faces onto wedge faces.
- **External corpus verification (`crates/warp3d/tests/corpus.rs`):** An `#[ignore]` integration test suite (`cargo xtask test --heavy`) discovers WARP3D 18.5.0 distribution decks at runtime via `$WARP3D_HOME/verification/**/driver.inp` plus `test44`, `test81`, and `manual_examples_chpt1/example_1/warp3d.inp` (exiting cleanly `Ok` when `WARP3D_HOME` is unset). Against the WARP3D 18.5.0 corpus (`173` unique decks), the sweep classifies **`24` Bucket A** (full import + `check()` + `resolve()` with a synthetic static step, including all five anchors `example_1/warp3d.inp`, `test14/test_14`, `test51/test_51a..d`, `test72/test72`, and `test87/test_87`), **`149` Bucket B** (explicit `Unsupported` / `IncludeRefused` on out-of-scope features such as `gurson`, `cohesive`, `cyclic`, `creep`, `killable`, `contact`, `crack`, `restart`, `thermal`, `fgm`, `transformation`, `crystal plasticity`, or rate-dependent plasticity), and **`0` Bucket C / `0` panics**.

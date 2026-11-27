# Stationary 3D Fracture Mechanics Completion on `warp3d/develop`

*   **Author:** ExMonsterJoker (`ex.monster.joker@gmail.com`)
*   **Status:** Proposed
*   **Date:** 2026-11-27

--------------------------------------------------------------------------------

## Overview

FieldCraft's `warp3d/develop` branch (`9b9d399b`) integrates WARP3D 18.5.0 for 3D linear-static and nonlinear-static solid analysis, automatic `Hex8`/`Hex20` crack-front domain integrals (`J`, `K_I`, `K_II`, `K_III`, `T_11`, `T_33`), API 579-1 Option 1/2 Failure Assessment Diagrams (`results.ffs`), and an interactive Fitness-for-Service panel (`crates/ui-egui/src/ffs_panel.rs`).

This design completes FieldCraft's **core single-front stationary 3D fracture workflow** across four sequential pull requests (PRs 1–4) confined to Layer 2 (`crates/warp3d`), Layer 6 (`crates/engine`), and Layer 7 (`crates/ui-egui`), leaving Layer 0 (`crates/model`), Layer 4 (`crates/doc`), and Layer 5 (`crates/io`) unchanged. It closes the broken collapsed-wedge rosette round-trip, emits crack-face `Load::Traction` via WARP3D `element loads` with accurate Method-1 interaction-integral diagnostics, adds targeted `CrackFrontSpec` controls and pre-flight checks matching WARP3D's domain validators, exposes mixed-mode (`K_II`, `K_III`, `T_33`) and convergence/history charts inside container-agnostic `ffs_panel::show_contents()` (compatible with future desktop pop-out windows under [#1088](https://github.com/tangency-labs/fieldcraft/issues/1088)), and verifies accuracy against published stationary 3D fracture solutions using procedural meshes built in code.

## The Problem

Analysts running 3D stationary Fitness-for-Service assessments on `warp3d/develop` (`9b9d399b`) encounter four blockers and silent correctness defects:

1. **Focused crack-tip rosettes fail after import (`Wedge6` dead end):** Standard 3D spider-web/rosette fracture meshes (such as WARP3D `test63`) collapse 8-node `l3disop` brick elements into 6-node wedges around the crack front. Today `deck.read` imports those collapsed elements as `ElementKind::Wedge6`, after which `warp3d.check`, `deck.write`, `solve.static`, and `solve.nonlinear` reject the model. Additionally, `reader/build.rs` places imported collapsed edges on `(w0, w3)`, `(w1, w4)`, or `(w2, w5)` depending on input ordering (for example, `test63` lands on `(w1, w4)`), so a fixed-edge export moves the collapsed face off the crack tip and distorts Ring-1 Gauss integration.
2. **Crack-face `Load::Traction` silently produces zero surface-integral contribution:** `deck.rs` currently lowers `Load::Traction` to WARP3D `nodal loads` instead of `element loads`. Inside WARP3D (`dielem_c.f`), surface integrals exit early when element equivalent face loads are zero, so crack-face tractions defined via `Load::Traction` contribute `0.0` to `dm5` (WARP3D's `J`-integral routine) and `II8` (WARP3D's interaction-integral routine for `K_I`, `K_II`, `K_III`, and `T`-stress). Even with `element loads`, WARP3D Method-1 (`approx = .true.`) surface integration in `II8` (`di_calc_surface_i`) evaluates only the in-plane advance component `t_1` (`termu`) and hardcodes `termv = 0` and `termw = 0`—so crack-normal (`t_2`) and anti-plane (`t_3`) tractions still require Method-2 `crack face tractions` (`cf_tractions`) for `K_I`/`K_III`, while `dm5` (`J`) integrates all three components automatically. Meanwhile, `check.rs` performs no `warp3d_face` or `rset.faces` checks for `Load::Traction`.
3. **Hardcoded `CrackFrontSpec` ceilings and missing pre-flight / parser diagnostics:**
   * Ring evaluation is hardcoded to `1..=rings` capped at `20` (even though `results.rs` already defines `MAX_RINGS_PER_DOMAIN = 64`), blocking outer-ring evaluation on blunted large-plasticity models. Meanwhile, `ffs.rs` (`RingExclusion::Auto`) unconditionally drops the first sorted ring present regardless of ring ID—dropping Ring 5 on a `rings 5-10` run while the UI checkbox reads `"Exclude Ring 1"`.
   * Interaction integrals hardcode `plane strain`, blocking thin-plate `plane stress` assessments, and parsed results carry no stress-state provenance tag.
   * Per-step fracture evaluation is binary (`every_step: true|false`): evaluating every increment on a dense nonlinear model can exceed `MAX_DOMAINS = 4_096` and truncate late increments after WARP3D finishes, while sparse evaluation without Increment-1 anchoring breaks `ffs.rs` (`KrSource::LinearScaledStep1Ki`, which linearly scales Increment-1 elastic $K_I$ by load factor).
   * `parse_crack_front_with` ignores unknown JSON keys, `order_crack_front` permits closed-loop or non-monotonic-chord domains that WARP3D's `dickdm` validator (inside `didriv.f`) skips at runtime, and `DomainIntegralParser::feed_line` discards `>>> Invalid domain definition` and mid-run `>>>` warning lines emitted outside an open `domain id:` block.
4. **Solved quantities are hidden in the UI and CSV exports:** `StationProfile` already holds display-unit `K_II`, `K_III`, and `T_33`, yet `ffs_panel.rs` and `results.ffs` CSV export omit them. Analysts also cannot plot per-station ring convergence (`J` vs. ring #) or load-step history (`J`/mode vs. load factor) outside FAD mode, and `results.export.csv` (`what: "fracture"`) omits `warp_step` and `interaction_stress_state` on multi-increment exports.

This design resolves all four areas across four incremental PRs while consuming Increment-1 elastic status (`step1_is_elastic`) to guard `KrSource::Auto` and prepare an additive hook for a separate PR 5 (API 579 Option 3 FAD), and keeping `ffs_panel::show_contents()` decoupled from dock strip geometry for future [#1088](https://github.com/tangency-labs/fieldcraft/issues/1088) pop-out hosting.

## Goals

*   **Close the `Wedge6` rosette round-trip (PR 1):** Support `Wedge6` → collapsed `l3disop` export via import canonicalization, crack-front-aware left cyclic orientation, rotation-aware quad face-load mapping, mixed linear `Wedge6 + Hex8` crack-front chaining, and consistent `check.rs` pre-flight validation (rejecting `Wedge6 + Hex20` mixed order and `> 1` front-touching axial edges on a wedge, while exporting triangular end-cap loads for structural equilibrium with a non-blocking `Warning` inside fracture domains).
*   **Export crack-face `Load::Traction` via `element loads` with accurate Method-1 checks (PR 1):** Emit `Load::Traction` on solid faces directly from the converted model as WARP3D `element loads ... face ... tractions` (removing the unreachable nodal fallback), validate `Load::Traction` face sets alongside `Load::Pressure` in `check.rs`, warn when `Load::Gravity`/`Load::BodyForce` overlaps loaded crack-face elements, and warn pre-flight when `interaction_integral: true` accompanies `Load::Traction` near `crack_front` (noting WARP3D Method-1 `II8` omits `t_2`/`t_3`).
*   **Ship three additive `CrackFrontSpec` fields, Layer-6 canonicalization, and `didriv.f` guards (PR 2):**
    *   2-state `interaction_stress_state` (`plane_strain` default | `plane_stress`, omitted from `.def` prefill unless selected) with provenance tags on `DomainIntegralResult` and `StationProfile`.
    *   Backward-compatible `ring_start` with ring cap raised from `20` to `MAX_RINGS_PER_DOMAIN = 64`, paired with ring-ID-aware `RingExclusion::Auto` (dropping only `ring == 1` when present).
    *   Single `increment_interval` scheduling (backward-compatible with `every_step` JSON input, omitted from the flat `forms::CRACK_FRONT` UI dialog) that anchors Increment 1 and the final increment, guards `can_linear_scale_step1` on `warp_step == 1`, and warns pre-flight when `stations * evaluated_increments > MAX_DOMAINS`.
    *   Strict key validation (`ALLOWED_CRACK_FRONT_KEYS`), `Layer 6` `canonicalize_crack_front` helper backing `ui-egui`'s `CachedCrackFront`, `didriv.f`-aligned `FrontAverage` closed-loop / monotonic-chord refusal, WARP3D `>>>` / `>>>>>` warning capture surfaced through `static_result.warnings`, and flat field ordering in `forms::CRACK_FRONT`.
*   **Track and consume `step1_is_elastic` for Increment-1 safety & PR 5 Option 3 hook (PR 2–3):** Stream column 8 (`epspls`, equivalent plastic strain `peeq`) from Increment-1 `wes*` (WARP3D flat element-stress output) into tri-state `step1_is_elastic: Option<bool>`, guard `KrSource::Auto` against scaling a yielded Increment 1, and keep `FadOption` / `results.ffs` extensible for PR 5's paired `elastic_step`.
*   **Deliver bounded diagnostic charts and complete CSV columns in container-agnostic `ffs_panel.rs` (PR 3):** Reuse `RingIntegral` on `StationProfile` (`active_profiles`), add `EvaluatedStep` pairs (`warp_step`, `frame_index`, `load_factor`) and single-station `step_history` on `FfsReport` converted via `units.rs::FFS`; expose `K_II`, `K_III`, and `T_33` across `ProfileQuantity`, tooltips, and `results.ffs` CSV; add `warp_step` and `interaction_stress_state` to `results.export.csv` (`what: "fracture"`); preserve `selected_domain` on unevaluated frames with a one-click jump banner; and ship `Front Profile`, `Ring Convergence`, and `Load History` in a compact segmented tab bar inside `ui.push_id`-scoped `ffs_panel::show_contents()`.
*   **Verify against published stationary 3D fracture solutions with numeric gates (PR 4):** Add procedural meshes built in Rust strictly in `crates/warp3d/tests/fracture_benchmarks.rs` (fast trunk tests + heavy live-WARP3D runs for Sneddon's pressurized penny crack, extruded thin-plate plane stress vs. plane strain, and Newman & Raju 1981 / Wang 2004 semi-elliptical surface crack) with numeric pass/fail targets and verified tables in `docs/warp3d.md`.
*   **Maintain FieldCraft engineering invariants:** Zero L0/L4/L5 changes, `#![forbid(unsafe_code)]`, zero panics in non-test code, deterministic output, `wasm32-unknown-unknown` builds without error, `cargo xtask layers` and `cargo xtask unit-factors` green, and `docs/features.md` regenerated in commits altering command schemas.

## Non-Goals

*   **Layer 0 / Layer 4 / Layer 5 changes:** No edits to `crates/model`, `crates/doc` (beyond regenerating `docs/features.md` and updating `docs/warp3d.md`), or `crates/io`, no `.fieldcraft` binary persistence of `CrackFrontSpec` or fracture results, and no quadratic `ElementKind::Wedge15` collapsed tips.
*   **Native desktop pop-out windows ([#1088](https://github.com/tangency-labs/fieldcraft/issues/1088)):** Handled separately via `solidcraft-ui-kit` extraction; deferred 2×2 multi-pane grid layout until `#1088` lands. PR 3 delivers the container-agnostic `show_contents()` tabbed/split surface.
*   **Full API 579 Option 3 FAD ($\sqrt{J_e / J_{\text{ep}}}$):** Deferred to follow-up **PR 5** after PRs 1–4 land.
*   **Cut/deferred `CrackFrontSpec` options:** No interaction-integral `"both"` mode, no manual `crack_face_tractions: [tx, ty, tz]` (Method-2 `cf_tractions` revisited only if PR 4 benchmarks miss target tolerances), no `coincident_tolerance_ratio` (defer with `front node sets`), no manual `start_tangent` / `end_tangent` overrides (revisit only if PR 4 Newman–Raju / Wang misses target tolerances), and no symmetry-provenance UI flag (`K_II`/`K_III` values are rendered directly).
*   **UI Sample Gallery additions:** PR 4's procedural crack meshes live strictly in `crates/warp3d/tests/`—no `Help › Samples` entries in PRs 1–4.
*   **Out-of-scope fracture capabilities:** No multiple crack fronts per run, no blunt-tip `front node sets` or user-defined `q-values`, no crack-growth models (Gurson/SMCS element deletion, CTOA node release, cohesive `inter_8`), no thermal/FGM/residual-stress `J`, no cyclic/creep/hydrogen materials, no contact crack closure, and no closed-form API 579 Annex 9B SIF calculator.
*   **Commercial meshes or CI-breaking oracle cases:** Never copy WARP3D `test87` FEACrack meshes and never add WARP3D-binary-dependent cases to `tests/oracle/`.

## Open Questions

None open. All resolved specification decisions are captured below:
1. **`Wedge6` triangular cap loads (PR 1):** Export triangular end-cap face loads (`face 1` / `2`) for structural equilibrium and emit a non-blocking `Warning` in `check.rs` only when `crack_front` is active and the loaded wedge touches a fracture domain.
2. **Increment schedule in `CRACK_FRONT` (PR 2):** Use a single `increment_interval: Option<usize>` in the UI and `CrackFrontSpec` (anchoring Increment 1 and final increment `n_steps`), accepting `every_step: Option<bool>` only as a backward-compatible JSON input in `cmd/deck.rs` and rejecting `every_step == Some(true)` paired with `increment_interval.is_some()` on `crack_front/increment_interval`.
3. **Station precedence & `Load History` target (PR 3):** Keep returning `EngineError::invalid("domain_id", ...)` when both top-level `domain_id` and `fad.domain_id` are explicitly provided with different values; otherwise resolve via `top-level domain_id` → `fad.domain_id` → governing peak-`J` station (at the latest evaluated step when `active_profiles` is empty).

## Technical Plan (Architecture)

The design spans three existing crates across Layer 2 (`crates/warp3d`), Layer 6 (`crates/engine`), and Layer 7 (`crates/ui-egui`), leaving Layer 0 (`crates/model`), Layer 4 (`crates/doc` code), and Layer 5 (`crates/io`) unchanged. Layer 2 converts SI `Model` quantities via `DisplaySystem` (`Preset::ALL`) solely when writing WARP3D `.inp` solver decks, while Layer 6 (`crates/engine`) owns all command-reply, form, CSV, and UI display unit conversions.

```text
+-----------------------------------------------------------------------+
| Layer 7: crates/ui-egui                                               |
|  - CachedCrackFront wrapping Layer-6 canonical Map<String, Value>     |
|  - ffs_panel::show_contents() (ui.push_id scoped, #1088-ready)        |
|    * Tabs: Front Profile | Ring Convergence | Load History + FAD      |
+-----------------------------------+-----------------------------------+
                                    | Engine commands & FfsReport JSON
                                    v (no dependency on fieldcraft-warp3d)
+-----------------------------------------------------------------------+
| Layer 6: crates/engine                                                |
|  - lib.rs: ALLOWED_CRACK_FRONT_KEYS + canonicalize_crack_front()      |
|  - lib.rs: Warp3dFractureData + SolvedFracture (step1_is_elastic)     |
|  - forms::CRACK_FRONT (flat basic->advanced) & FORMS::RESULTS_FFS     |
|  - cmd/deck.rs, solve.rs, ffs.rs, results.rs, units.rs                |
+-----------------------------------+-----------------------------------+
                                    | SI Model, CrackFrontSpec, FfsReport
                                    v
+-----------------------------------------------------------------------+
| Layer 2: crates/warp3d                                                |
|  - wedge6.rs: cyclic collapse rotation + quad face map                |
|  - deck.rs: Wedge6->l3disop export + Load::Traction element loads     |
|  - fracture.rs: Wedge6+Hex8 chaining + anchored increment schedule    |
|  - check.rs: dickdm monotonic-chord guard + II8/dielwf warnings       |
|  - results.rs & runner.rs: stress-state tags, >>> warnings, wes* scan |
|  - ffs.rs: RingExclusion::Auto ring-1 fix, RingIntegral, history      |
|  - tests/: procedural clean-room benchmarks (trunk + heavy gated)     |
+-----------------------------------------------------------------------+
```

### 1. PR 1 — `Wedge6` Collapsed `l3disop` Round-Trip & `Load::Traction` Element Loads (`crates/warp3d`)

*   **Shared `Wedge6` Collapse Orientation & Face Mapping (`wedge6.rs`, `deck.rs`, `check.rs`, `reader/build.rs`):**
    *   In `reader/build.rs::collapse_hex8_to_wedge6`, normalize every collapsed `l3disop` pattern so the collapsed edge lands on canonical `(w2, w5)`, keeping `collapsed_hex_nodes` (the import side-table mapping original 8-node hex faces to lowered `Wedge6` faces) consistent and right-handed.
    *   Introduce `wedge6.rs` shared by `deck.rs`, `check.rs`, and `fracture.rs`: determine a `Wedge6`'s cyclic prism rotation (`rot in {0, 1, 2}`) when `crack_front` touches both endpoints of **exactly one** axial edge (`(0, 3)`, `(1, 4)`, or `(2, 5)`), apply a **left** cyclic rotation by `rot` so the crack-front edge lands on collapsed `(c, f)` in `[a, b, c, c, d, e, f, f]`, and map quad faces `3..=5` across WARP3D's non-collapsed hex faces (`{3, 5, 6}`, avoiding collapsed face `4` `[c, c, f, f]` after the `Hex8` `4 <-> 5` face swap) while preserving triangular caps `1` and `2`.
    *   Update free function `fn kind_rank` (`crates/warp3d/src/lib.rs`, where `Hex8` is rank `0`) to assign `Wedge6` rank `1` so `Wedge6` and `Hex8` form contiguous `l3disop` element blocks.
*   **Mixed `Wedge6 + Hex8` Crack-Front Chaining (`fracture.rs`):**
    *   Extend `order_crack_front` to accept `ElementKind::Wedge6` alongside `ElementKind::Hex8` as linear elements while rejecting `Wedge6 + Hex20` mixed order. Verify any front-touching `Wedge6` has exactly one axial edge on `crack_front`, rejecting wedges touching `> 1` axial edge or any triangular-cap edge.
*   **`Load::Traction` Element-Load Export & Pre-Flight Checks (`deck.rs`, `check.rs`):**
    *   Consume `Load::Traction { set, vector }` directly from the already unit-converted `expressed` model (`write_deck` converts once via `model.converted`), emit `element loads` (`<we> face <wface> tractions tx <x> ty <y> tz <z>`), update `has_loads` and face-load guards so traction-only steps emit `loading`, switch `Load::Pressure` to use the same `Wedge6`-aware face mapper, and remove the dead nodal fallback (since `crates/model` already rejects 3D `Load::Traction` on empty face sets).
    *   In `check.rs`: accept `ElementKind::Wedge6` in supported solid element checks, validate both `Load::Pressure` and `Load::Traction` faces via `wedge6::wedge6_warp3d_face`, emit a `Warning` when `interaction_integral == Some(true)` accompanies crack-face `Pressure`/`Traction` adjacent to `crack_front` (documenting WARP3D Method-1 `II8`'s `t_2`/`t_3` omission), emit a `Warning` when triangular cap `face 1`/`2` is loaded on a fracture-adjacent `Wedge6` (still applied to structural equilibrium, skipped by `dielwf`), and emit a `Warning` when non-zero `Load::Gravity`/`Load::BodyForce` overlaps loaded crack-face elements.

### 2. PR 2 — Additive `CrackFrontSpec` Controls, Stress-State Provenance & Pre-Flight Guards (`crates/warp3d`, `crates/engine`, `crates/ui-egui`)

*   **Schema, Validation & Canonicalization (`crates/warp3d/src/lib.rs`, `crates/engine/src/lib.rs`, `cmd/deck.rs`, `forms.rs`, `ui-egui/src/lib.rs`):**
    *   Add strictly additive `#[serde(default)]` dimensionless fields to `CrackFrontSpec`: `ring_start: Option<usize>` (raise ring cap from `20` to `MAX_RINGS_PER_DOMAIN = 64`), `increment_interval: Option<usize>` (`1..=10_000`), and `interaction_stress_state: Option<InteractionStressState>` (`PlaneStrain` default | `PlaneStress`, rejected if `interaction_integral != Some(true)`). Change `every_step` to `Option<bool>`.
    *   Export `ALLOWED_CRACK_FRONT_KEYS` and `canonicalize_crack_front` from `crates/engine/src/lib.rs` so `parse_crack_front_with` (rejecting unknown keys) and Layer 7's `CachedCrackFront` (which tracks the active crack front across UI sessions) share one canonical JSON validator while `ui-egui` updates `set` on `model.rename`.
    *   Add `ring_start`, `increment_interval`, and `interaction_stress_state` (without `.def("\"plane_strain\"")`) directly in flat basic-to-advanced order inside `forms::CRACK_FRONT` (used by `warp3d.check`, `deck.write`, `solve.static`, and `solve.nonlinear`), keeping `every_step` only as a backward-compatible JSON input (rejecting `every_step == Some(true)` with `increment_interval.is_some()`). Regenerate `docs/features.md` in the same commit.
*   **Deck Emission & Guards (`fracture.rs`, `deck.rs`, `check.rs`):**
    *   Emit `q-values automatic rings {ring_start}-{rings}` and `plane strain` / `plane stress output for interaction integral` across anchored evaluated increments (`{1, k, 2k, ..., n_steps}` for `increment_interval: k`, `[1]` on 1-step `Static`, with a non-blocking warning when `k > n_steps` on multi-increment runs).
    *   In `check.rs`, reject `CrackFunctionType::FrontAverage` (`type d`) when closed OR violating WARP3D `didriv.f::dickdm`'s monotonic Euclidean chord-distance check from node `0` (also warning on multi-node `Pointwise` fronts failing monotonic distance—preventing WARP3D from skipping invalid domains), and warn when `stations * eval_increments > MAX_DOMAINS` (`4096`).
*   **Result Provenance, `RingExclusion::Auto`, Diagnostics & `step1_is_elastic` (`results.rs`, `runner.rs`, `ffs.rs`, `engine/src/lib.rs`):**
    *   Tag `DomainIntegralResult` and `StationProfile` with optional `interaction_stress_state` (propagated across `merge_interaction_into_existing`).
    *   Intercept WARP3D `>>> Invalid domain definition`, `>>> Warning:`, `>>> Area under q-function`, `>>>>> warning:`, and `>>>>> error:` lines before `DomainIntegralParser::feed_line`'s early-return gate, forwarding bounded warnings through `ParsedDomainOutput` (`runner.rs` ↔ `results.rs`) into `static_result.warnings` before construction.
    *   Fix `RingExclusion::Auto` in `ffs.rs` (excluding `RingExclusion` `ring == 1` only when `ring == 1` is present and `len >= 3`; other `Count(n)` modes strip `n` inner sorted rings), and guard `can_linear_scale_step1` on `first_step.warp_step == 1 && step1_is_elastic != Some(false)`.
    *   Stream column 8 (`epspls` / `peeq`) from Increment-1 `wes*` inside `results.rs` (outside final-step-only skips, reusing final elements on 1-step runs) to populate tri-state `step1_is_elastic: Option<bool>` on `Warp3dSolution`, `Warp3dFractureData`, `SolvedFracture`, and callers (`cmd/solve.rs`, `cmd/results.rs`, `cmd/ffs.rs`).

### 3. PR 3 — Bounded `FfsReport` Charts, Complete CSV Columns & Container-Agnostic `ffs_panel.rs` (`crates/warp3d`, `crates/engine`, `crates/ui-egui`)

*   **Bounded `FfsReport` Contract (`crates/warp3d/src/ffs.rs`, `cmd/ffs.rs`, `cmd/results.rs`, `units.rs`):**
    *   Reuse existing `rings: Vec<RingIntegral>` on `StationProfile` for `active_profiles` only (`O(stations * rings)`, `<= MAX_RINGS_PER_DOMAIN`), deriving `kept` in UI from `excluded_rings`.
    *   Follow strict `domain_id` precedence (`EngineError::invalid("domain_id", ...)` when explicit top-level `domain_id` and `fad.domain_id` differ; else `top-level domain_id` → `fad.domain_id` → governing peak-`J` at the latest evaluated step when `active_profiles` is empty), populate bounded `step_history: Vec<StepStationPoint>`, `evaluated_steps: Vec<EvaluatedStep>` (`warp_step`, `frame_index`, `load_factor`), `step_history_domain_id: Option<String>`, and `step1_is_elastic: Option<bool>` on `FfsReport`, and convert all quantities through `units.rs::FFS`.
    *   Append `interaction_stress_state`, `k_ii_mean`, `k_iii_mean`, and `t_33_mean` to `ffs_csv_table`, and append `warp_step` and `interaction_stress_state` to `results.export.csv`'s `fracture_table`.
*   **Container-Agnostic UI Charts & `#1088` Readiness (`crates/ui-egui/src/ffs_panel.rs`):**
    *   Scope `show_contents()` in `ui.push_id("fc_ffs_contents", ...)`, keeping it decoupled from `shell.rs` bottom-strip height clamping and `fieldcraft-warp3d`.
    *   Expose `K_II`, `K_III`, and `T_33` in `ProfileQuantity` and tooltips; show active interaction stress state (`Plane strain` / `Plane stress`) and local-axis legend `(x1=advance, x2=normal, x3=tangent)`; guard `selected_domain` updates so unevaluated frames do not clear user station selection; disable `Exclude Ring 1` with tooltip when `min_ring > 1` alongside `Rings {min}–{max}`; emit top-level `domain_id` in `build_params` only when `fad` does not set `station_mode: "domain_id"`; show an evaluated-increment jump banner on empty frames mapping `evaluated_steps` to `results.frame`; and switch left-pane views via a segmented tab bar (`Front Profile` | `Ring Convergence` | `Load History`) beside `WIDE_SPLIT_MIN_WIDTH` optional `API 579 FAD`.

### 4. PR 4 — Clean-Room Stationary 3D Fracture Verification Suite (`crates/warp3d/tests/fracture_benchmarks.rs`, `docs/warp3d.md`)

*   **Strict Clean-Room & CI Rules:** All benchmark meshes are generated procedurally in pure Rust inside `crates/warp3d/tests/fracture_benchmarks.rs` (no commercial meshes, no `test87` copies, no `Help › Samples`, no `tests/oracle/` edits).
*   **Two-Tier Execution & Target Tolerances:**
    *   *Trunk gate (no WARP3D binary needed):* Procedural mesh/Jacobian validation, deck-golden/parser/unit-conversion (`Preset::ALL`) checks, and API 579-1 Part 9 FAD curve/verdict checks (`<= 1e-6` relative error).
    *   *Heavy live-WARP3D suite (`#[ignore = "heavy: cargo xtask test --heavy (CI heavy job)"]`, skips on `BinaryNotFound` unless `FIELDCRAFT_REQUIRE_WARP3D=1`):*
        1. Revolved 1/8-symmetry Sneddon (1946) pressurized penny-shaped crack (`Wedge6` rosette + `Hex8` + `Load::Pressure` + `Load::Traction`): `<= 3%` error on $K_J = \frac{2}{\pi} p \sqrt{\pi a}$ and `<= 2%` outer-ring $J$ spread (also recording Method-1 `II8` `K_I` vs. `K_J` behavior).
        2. Extruded thin-plate through-crack comparing `interaction_stress_state: "plane_stress"` ($|T_{33}| / |T_{11}| \le 10^{-3}$) vs. `"plane_strain"`, checking `<= 2%` agreement between interaction-integral $K_I$ and domain $K_J$.
        3. Procedural 1/4-symmetry 3D semi-elliptical surface-crack mesh (`Wedge6`+`Hex8`) verified against Newman & Raju (1981) interior $K_I(\phi)$ (`<= 5%`) and Wang (2004) $T$-stress (`<= 10%`), testing whether WARP3D's automatic tangents and Method-1 `element loads` meet tolerances without deferred overrides.
        4. Nonlinear multi-increment elastic-plastic verification exercising `ring_start`, `increment_interval`, `RingExclusion::Auto`, `step1_is_elastic == Some(false)`, `results.ffs`, and API 579-1 Part 9 reference checks (`<= 1e-6` relative FAD error against analytical reference values).

## Critical User Journeys (CUJs)

### CUJ 1: Import, Check, and Solve a Focused `Wedge6` Rosette Crack Mesh with Crack-Face Loads

*   **Given:** A user has a WARP3D ASCII fracture deck with collapsed `l3disop` crack-tip wedges (such as `test63`) or a FieldCraft mesh mixing Ring-1 `Wedge6` elements with outer `Hex8` rings, plus `Load::Pressure` or `Load::Traction` on the crack face and a `crack_front` node set.
*   **When:** The user imports via `deck.read` (optional), selects a static/nonlinear step, runs `warp3d.check` with `crack_front`, and executes `deck.write` or `solve.static` / `solve.nonlinear` (`backend: "warp3d"`).
*   **Then:** `warp3d.check` reports `ready: true` (emitting `warp3d.interaction_crack_face_load` warning if `Load::Traction` accompanies `interaction_integral: true`), `deck.rs` expands each front-touching `Wedge6` with its collapsed edge on `crack_front`, maps quad face loads without double conversion, emits `Load::Traction` as `element loads ... face ... tractions`, chains mixed `Wedge6 + Hex8` edges, and parses `J`/`K`/`T` results into SI.

### CUJ 2: Run Outer-Ring, Sparse-Increment Nonlinear Elastic-Plastic Fracture Analysis

*   **Given:** A user has a 20-increment `NonlinearStatic` elastic-plastic 3D crack model where inner rings 1–4 experience blunting and evaluating every increment across 30 stations exceeds `MAX_DOMAINS = 4_096` if run every step.
*   **When:** The user configures `crack_front` with `ring_start: 5`, `rings: 10`, and `increment_interval: 4`, runs `warp3d.check`, solves with WARP3D, and opens `results.ffs` / the FFS panel.
*   **Then:** `write_deck` emits `q-values automatic rings 5-10` at anchored increments `{1, 4, 8, 12, 16, 20}`; `RingExclusion::Auto` keeps all 6 outer rings because `min_ring == 5 > 1`; `step1_is_elastic` streams Increment-1 `peeq` to guard `KrSource::Auto`; and scrubbing `results.frame` to an unevaluated frame preserves `selected_domain` and shows a jump banner to the nearest evaluated increment.

### CUJ 3: Run Thin-Plate Plane-Stress vs. Thick-Plate Plane-Strain Interaction Integrals

*   **Given:** A user has a 3D linear-elastic cracked plate model and wants interaction integrals evaluated under plane-stress auxiliary fields rather than default plane strain.
*   **When:** The user sets `interaction_integral: true` and `interaction_stress_state: "plane_stress"` on `crack_front`, solves with WARP3D, and inspects `results.ffs`, CSV exports, and `ffs_panel.rs`.
*   **Then:** `write_deck` emits `plane stress output for interaction integral`, `results.rs` tags `interaction_stress_state = PlaneStress`, and the FFS panel and both CSV exporters record the active stress state.

### CUJ 4: Diagnose Mixed-Mode Profiles, Ring Convergence, and Load History in `ffs_panel.rs`

*   **Given:** A user has solved a multi-increment 3D mixed-mode crack model with interaction integrals enabled and opens `Results -> Fracture Integrals -> FFS Assessment...`.
*   **When:** The user switches `ProfileQuantity` to `K_II`, `K_III`, or `T_33`, clicks a crack-front station, selects `Ring Convergence` or `Load History` tabs, clicks a load-history point, or exports CSV.
*   **Then:** All values display in active output units converted through `units.rs::FFS`, tooltips and CSV columns include `k_ii_mean`, `k_iii_mean`, `t_33_mean`, `warp_step`, and `interaction_stress_state`, `Ring Convergence` renders kept vs. excluded rings using `RingIntegral` and `excluded_rings`, clicking a `Load History` point syncs `results.frame`, and `show_contents()` runs with isolated `ui.push_id` state ready for future `#1088` pop-out hosting.

### CUJ 5: Execute Clean-Room Stationary 3D Fracture Verification Benchmarks

*   **Given:** A developer runs trunk CI (`cargo test -p fieldcraft-warp3d`) without a WARP3D binary, or runs local heavy verification (`cargo xtask test --heavy` with `WARP3D_HOME` and `FIELDCRAFT_REQUIRE_WARP3D=1`).
*   **When:** The test suite executes `crates/warp3d/tests/fracture_benchmarks.rs`.
*   **Then:** Trunk tests pass deterministically without WARP3D, and heavy live-WARP3D benchmarks verify Sneddon, thin-plate plane stress/strain, Newman & Raju (1981) / Wang (2004), and nonlinear API 579-1 checks within the numeric tolerances stated above.

## Alternatives Considered

### Alternative A: Support Interaction-Integral `"both"` Mode in a Single Solve

*   **Pros:** Produces both plane-strain and plane-stress interaction integrals (`K_I`, `K_II`, `T_11`) in one WARP3D solve.
*   **Cons:** `RingIntegral`, `DomainIntegralResult`, and `StationProfile` carry one slot each for `k_i`, `k_ii`, `t_11`, and `t_33`, and `results.rs` overwrites plane-stress tables when WARP3D prints both. Avoiding data loss requires duplicating fields across Layer 2 structs, Layer 6 `units.rs` rules (`FRACTURE`, `convert_fracture_k`, `FFS`), CSV tables, and `ffs_panel.rs`.
*   **Reason for Rejection:** A single-selection 2-state enum (`plane_strain` | `plane_stress`) with an optional `interaction_stress_state` provenance tag keeps schemas, unit rules, and UI controls unambiguous without cross-layer field duplication.

### Alternative B: Add Manual `crack_face_tractions: [tx, ty, tz]` or Synthesize Method-2 `cf_tractions` Automatically Upfront

*   **Pros:** Invokes WARP3D Method-2 (`approx = .false.`) 4×4 Gauss quadrature for uniform planar crack-face loads and evaluates `t_2`/`t_3` inside `II8` (`dielem_c.f`).
*   **Cons:** WARP3D's `cf_tractions` is a single global constant vector applied across `+normal` faces; it cannot represent non-uniform or multi-set loads, introduces a global-axis `+normal` sign hazard, requires unit conversion inside `CrackFrontSpec`, and requires synthesizing per-increment load-factor domain overrides.
*   **Reason for Rejection:** Lowering `Load::Traction` to `element loads ... tractions` handles structural equilibrium, `dm5` `J`/`K_J`, and in-plane `t_1` `II8` uniformly with `Model::loads()`, paired with a `check.rs` pre-flight warning when `interaction_integral: true` accompanies face tractions; Method-2 `cf_tractions` will be reconsidered only if PR 4 benchmarks fail target tolerances.

### Alternative C: Expose `coincident_tolerance_ratio`, Explicit `increments` Lists, and Manual Endpoint Tangents Now

*   **Pros:** Exposes WARP3D `box_tol_relat`, arbitrary non-uniform increment lists, and endpoint kink overrides on coarse curved fronts.
*   **Cons:** `box_tol_relat` has low utility while blunt `front node sets` are out of scope; arbitrary increment lists complicate `CRACK_FRONT` forms and validation; and assigning `start` by lowest node ID creates sign-flip and orthogonality errors on manual endpoint tangents.
*   **Reason for Rejection:** Exposing only `increment_interval` (anchoring Increment 1 and `n_steps`) keeps all four `CRACK_FRONT` commands (`warp3d.check`, `deck.write`, `solve.static`, `solve.nonlinear`) concise; manual tangents will be reconsidered only if PR 4 Newman–Raju / Wang misses target tolerances under WARP3D's automatic tangents.

### Alternative D: Include Full API 579 Option 3 FAD ($\sqrt{J_e / J_{\text{ep}}}$) and Sample Gallery Models in PRs 1–4

*   **Pros:** Ships elastic-plastic $J$-based FAD assessment and sample gallery meshes immediately.
*   **Cons:** Option 3 requires paired elastic/elastic-plastic step matching, new FAD curve semantics, and additional validation, while gallery additions add UI screenshot maintenance outside core fracture support.
*   **Reason for Rejection:** Staged separately as PR 5 (with PRs 2–3 shipping only `step1_is_elastic` and extensible `results.ffs` fields), while PR 4 procedural meshes stay in `crates/warp3d/tests/`.

### Alternative E: Couple Stationary Completion with `#1088` Pop-Out Windows / 2×2 Grid or Crack-Growth / Multi-Front Support

*   **Pros:** Delivers detached OS windows, 2×2 dashboards, crack growth, or multi-front solves in one release.
*   **Cons:** `#1088` depends on upstream `solidcraft-ui-kit` extraction (leaving a PR 3 2×2 branch unused inside today's `160..=360 px` docked panel); crack growth requires Layer 0/Layer 5 schema changes; multi-front runs cut across domain IDs and governing FAD logic; and `front node sets` overflows flat `Set` primitives.
*   **Reason for Rejection:** Stationary single-front fracture ships in container-agnostic `show_contents()` first, deferring the 2×2 layout to `#1088` and multi-front / crack growth / `front node sets` to later work.

### Alternative F: Read Raw SI `session.fracture_results()` in `ffs_panel.rs` or Embed All Rings Across All Steps in `FfsReport`

*   **Pros:** Avoids extending `FfsReport` or fetches every increment's rings in one payload.
*   **Cons:** Direct SI reads in `crates/ui-egui` violate the Layer 6 display-unit boundary (`units.rs`) and hide convergence/history data from CLI/MCP `results.ffs`; embedding `stations × steps × rings` causes multi-MB JSON serialization stalls.
*   **Reason for Rejection:** Extending `FfsReport` with active-step `rings` (`RingIntegral`) and single-station `step_history` preserves the Layer 6 unit boundary while keeping serialization small.

### Alternative G: Copy WARP3D `test87` FEACrack Meshes or Place Live WARP3D Runs in `tests/oracle/`

*   **Pros:** Reuses upstream test decks or places benchmarks in the top-level oracle suite.
*   **Cons:** `test87` originates from commercial FEACrack (disallowed by `AGENTS.md`), `tests/oracle/` violates the 3-crate boundary, and GitHub CI lacks a WARP3D binary.
*   **Reason for Rejection:** Procedural meshes inside `crates/warp3d/tests/fracture_benchmarks.rs` satisfy `AGENTS.md` and CI gating.

### Alternative H: Store Per-Element Collapsed-Edge Metadata in Layer 0 or Add `Wedge15` Instead of `(w2, w5)` Canonicalization & Cyclic Rotation

*   **Pros:** Avoids rotating `Wedge6` connectivity on import/export and supports quadratic collapsed wedges.
*   **Cons:** Adding per-element collapse metadata or `ElementKind::Wedge15` modifies Layer 0 (`crates/model`) and Layer 5 (`crates/io`), breaking `.fieldcraft` schema stability outside this design's 3-crate boundary.
*   **Reason for Rejection:** Normalizing collapsed `l3disop` imports onto `(w2, w5)` and applying left cyclic rotation around `crack_front` completes linear `Wedge6` round-trips strictly inside `crates/warp3d` with zero L0/L5 schema changes.

## Non-Functional Requirements

*   **Bounded Payloads & Execution Guards:**
    *   `rings` is hard-capped at `MAX_RINGS_PER_DOMAIN = 64`; `FfsReport.active_profiles[*].rings` holds `O(stations × rings)` entries for the active step only, and `step_history` holds `O(eval_increments)` entries for one station. Dense fronts exceeding `MAX_DOMAINS` (`4_096` total `stations × eval_increments` domains) are soft-guarded by a pre-flight `Warning` in `check.rs` to alert users before parser truncation (`domains_truncated`).
    *   `check::quick_note` (pre-resolution status note in UI panels) stays `O(1)` in mesh size; `DomainIntegralParser` caps captured WARP3D warnings (`MAX_DOMAIN_WARNINGS = 32`) and domain lines (`MAX_DOMAIN_LINES`, verified/raised for 64-ring outputs).
*   **Safety, Determinism & Platform Compatibility:**
    *   All touched crates keep `#![forbid(unsafe_code)]` and panic-denial lints; `.inp` decks, `FfsReport`, and CSV tables use ordered containers (`BTreeMap`/`BTreeSet`/sorted `Vec`) for bit-identical determinism.
    *   `crates/warp3d`, `crates/engine`, and `crates/ui-egui` build for `wasm32-unknown-unknown`, and `cargo xtask layers` and `cargo xtask unit-factors` pass (dimensionless thresholds like `1e-12` carry `// unit-factors: ...` annotations).
*   **Backward Compatibility & Staged Rollout:**
    *   Additive fields on `CrackFrontSpec`, `DomainIntegralResult`, `StationProfile`, and `FfsReport` use `#[serde(default)]`; CSV additions are appended columns preserving existing header prefixes; PRs 1–4 land sequentially with each commit building and passing touched-crate tests.

## Risks & Mitigations

*   **WARP3D Method-1 `II8` ignores `t_2`/`t_3` face tractions (`dielem_c.f`):** Could cause unexpected divergence between `K_I`/`K_III` and `K_J` when a user applies normal or anti-plane `Load::Traction` alongside `interaction_integral: true`. *Mitigation:* Document `Load::Pressure` as preferred for normal crack pressure, emit `warp3d.interaction_crack_face_load` warning pre-flight in `check.rs`, verify $K_J$ on Sneddon, and revisit Method-2 `cf_tractions` only if PR 4 benchmarks fail target tolerances.
*   **PR 4 benchmark tolerance misses:** Automatic endpoint tangents on curved 3D semi-elliptical fronts or Method-1 equivalent nodal forces could miss `<= 5%` (`K_I`) / `<= 10%` (`T`-stress) targets. *Mitigation:* Explicit numeric pass/fail gates in `fracture_benchmarks.rs` determine whether manual tangent overrides or `cf_tractions` are needed in a follow-up.
*   **WARP3D runtime domain skipping (`didriv.f::dickdm`) or stdout truncation:** Non-monotonic chord distance on curved fronts or `64`-ring multi-step solves can lose increments after WARP3D completes. *Mitigation:* Mirror `dickdm`'s monotonic chord-distance check pre-flight in `check.rs`, pre-warn at `MAX_DOMAINS = 4_096`, verify `MAX_DOMAIN_LINES` for 64-ring outputs, and capture `>>>` / `>>>>>` warnings before `feed_line`'s early-return gate.
*   **Hostile/malformed inputs (`AGENTS.md` No-Crash):** Unknown keys in `crack_front`, `k = 0` intervals, `n_steps = 0`, truncated `wes*` files, or degenerate `Wedge6` fronts touching multiple axial edges could crash or misbehave. *Mitigation:* Enforce `ALLOWED_CRACK_FRONT_KEYS`, clamp/validate bounds (`1 <= ring_start <= rings <= 64`, `increment_interval in 1..=10_000`), guard `step_by` with `k.max(1)`, reject multi-edge `Wedge6` fronts explicitly, and return `Result` errors across all layers.

## Additional Resources

*   Workspace docs: [ROADMAP.md](ROADMAP.md), [AGENTS.md](AGENTS.md), [architecture.md](../architecture.md), [units.md](../units.md), [verification.md](../verification.md), [warp3d.md](../warp3d.md)
*   WARP3D 18.5.0 distribution sources and manual (`Chp_2/2.8 (Loads)`, `Chp_3/3.1_Hex_Elements`, `Chp_4`, `src/indom.f`, `src/didriv.f`, `src/dicmj.f`, `src/dielem_c.f`, `src/di1dsf.f`, `src/elem_load_b.f`)
*   Upstream UI windowing track: [tangency-labs/fieldcraft#1088](https://github.com/tangency-labs/fieldcraft/issues/1088)

## Future Improvements

*   **PR 5 (API 579 Option 3 FAD):** Evaluate $\sqrt{J_e / J_{\text{ep}}}$ using a paired elastic solve (`elastic_step`) and the `step1_is_elastic` / extensible `FadOption` hook prepared in PRs 2–3.
*   **Conditional `CrackFrontSpec` additions:** Revisit manual `crack_face_tractions` or endpoint tangent overrides only if PR 4 benchmarks fail target tolerances under Method-1 `element loads` and WARP3D's automatic tangents.
*   **Multi-pane `ffs_panel.rs` 2×2 dashboard:** Add detached multi-pane 2×2 grid rendering when [#1088](https://github.com/tangency-labs/fieldcraft/issues/1088) pop-out window hosting lands.

--------------------------------------------------------------------------------

## Detailed Implementation Plan

All changes across PRs 1–4 stay strictly inside `crates/warp3d`, `crates/engine`, `crates/ui-egui`, `docs/features.md`, and `docs/warp3d.md`. No files under `crates/model`, `crates/io`, `crates/solver`, `tests/oracle/`, or `apps/` are modified.

### Target Files to Create/Modify

#### 1. `crates/warp3d/src/wedge6.rs` (Create — PR 1)
*   **Type / Scope:** Layer 2 `pub(crate)` geometry/topology helper module used by `deck.rs`, `check.rs`, `fracture.rs`, and `reader/build.rs`.
*   **Proposed Changes:**
    *   Define `QUAD_WARP_FACES: [u8; 3] = [3, 5, 6]` and `wedge6_collapse_rotation(nodes: &[u32], front_nodes: &BTreeSet<u32>) -> u8`: inspects the three axial edges `(0, 3)`, `(1, 4)`, `(2, 5)` where **both** endpoints are in `front_nodes`; if `touching_axial_edge_count == 1`, returns `1` for `(0, 3)`, `2` for `(1, 4)`, or `0` for `(2, 5)` (defaulting to `0` otherwise).
    *   Define `expand_wedge6_to_l3disop(nodes: &[u32], rot: u8) -> Option<[u32; 8]>`: returns `None` if `nodes.len() < 6 || rot >= 3`; performs a **left** cyclic rotation by `rot`: `[a, b, c] = [w[rot%3], w[(rot+1)%3], w[(rot+2)%3]]`, `[d, e, f] = [w[3+rot%3], w[3+(rot+1)%3], w[3+(rot+2)%3]]`, and emits right-handed `[a, b, c, c, d, e, f, f]`.
    *   Define `wedge6_warp3d_face(face: u8, rot: u8) -> Option<u8>`: returns `None` if `rot >= 3`; maps triangular caps `1 -> Some(1)`, `2 -> Some(2)`, quad faces `3..=5 -> Some(QUAD_WARP_FACES[((face - 3 + 3 - rot) % 3) as usize])`, and `_ -> None`.
    *   Add unit tests asserting vertex-set equality across all `3` rotations × `5` faces against `ElementKind::Hex8.faces()` and `deck::warp3d_face(ElementKind::Hex8, ...)`, plus out-of-range guards.
*   **Rationale:** Establishes a single source of truth for collapsed `l3disop` incidences and face numbering, avoiding collapsed face `4`.

#### 2. `crates/warp3d/src/reader/build.rs` (Modify — PR 1)
*   **Type / Scope:** Layer 2 WARP3D ASCII deck import canonicalization.
*   **Proposed Changes:**
    *   Update `collapse_hex8_to_wedge6` (`crates/warp3d/src/reader/build.rs:922-959`) so all 12 collapsed `l3disop` branches cyclically normalize the resulting `Wedge6` with the collapsed edge at canonical `(w2, w5)` while preserving `collapsed_hex_nodes` face-load mapping.
    *   Add unit tests over all 12 collapse patterns (including `test63`-style `(h2==h6 && h3==h7)` and `(h0==h4 && h3==h7)`) asserting canonical `(2, 5)` edge landing and positive signed element volume / Jacobian.
*   **Rationale:** Preserves crack-tip alignment across `deck.read -> deck.write` round-trips even without an active `crack_front`.

#### 3. `crates/warp3d/src/lib.rs` (Modify — PR 1, PR 2)
*   **Type / Scope:** Layer 2 crate root, `kind_rank`, and fracture schemas (`CrackFrontSpec`, `DomainIntegralResult`, `Warp3dSolution`).
*   **Proposed Changes:**
    *   *(PR 1)* Expose `pub(crate) mod wedge6;`. Update free function `fn kind_rank(kind: ElementKind) -> u8` (`crates/warp3d/src/lib.rs:92-100`, where `Hex8` has rank `0`) to assign `Wedge6 -> 1`, shifting `Hex20 -> 2`, `Tet4 -> 3`, `Tet10 -> 4`, `_ -> 5` so `Wedge6` groups contiguously for WARP3D `elements` blocks.
    *   *(PR 2)* Export `pub(crate) const MAX_RINGS_PER_DOMAIN: usize = 64;` (shared with `results.rs:30`, `check.rs`, `fracture.rs`) and add `#[serde(rename_all = "snake_case")] pub enum InteractionStressState { #[default] PlaneStrain, PlaneStress }`.
    *   *(PR 2)* Update `CrackFrontSpec` (`lib.rs:161-183`): change `every_step` to `#[serde(default, skip_serializing_if = "Option::is_none")] pub every_step: Option<bool>`, and add `#[serde(default, skip_serializing_if = "Option::is_none")]` fields `ring_start: Option<usize>`, `increment_interval: Option<usize>`, and `interaction_stress_state: Option<InteractionStressState>`.
    *   *(PR 2)* Add `#[serde(default, skip_serializing_if = "Option::is_none")] pub interaction_stress_state: Option<InteractionStressState>` to `DomainIntegralResult` (`lib.rs:200-228`), and `#[serde(default, skip_serializing_if = "Option::is_none")] pub step1_is_elastic: Option<bool>` to `Warp3dSolution` (`lib.rs:243-253`). Update existing test struct literals in `crates/warp3d`.
*   **Rationale:** Keeps new Layer 2 fields strictly additive and backward-compatible with zero L0/L5 changes.

#### 4. `crates/warp3d/src/deck.rs` (Modify — PR 1, PR 2)
*   **Type / Scope:** Layer 2 `.inp` deck generator (`write_deck`).
*   **Proposed Changes:**
    *   *(PR 1)* Build `crack_front_nodes: BTreeSet<u32>` before load collection (`deck.rs:254`). In `format_element_spec` (`deck.rs:95-110`), map `ElementKind::Wedge6` to `l3disop` (honoring `nlgeom` and `Integration`), and expand `Wedge6` incidences (`deck.rs:443-458`) via `wedge6::expand_wedge6_to_l3disop` + `wedge6_collapse_rotation`.
    *   *(PR 1)* Add `warp3d_face_for_elem(elem, face, front_nodes)` delegating `Wedge6` to `wedge6::wedge6_warp3d_face` and others to `warp3d_face`; update `Load::Pressure` (`deck.rs:285`) to call `warp3d_face_for_elem`.
    *   *(PR 1)* Fix `Load::Traction` (`deck.rs:264-275`, `327`, `483`, `504-512`): read `*vector` directly from `expressed.load(load_name)` (already converted to output pressure units via `model.converted` at `deck.rs:137` and `convert.rs:239`—do NOT call `DisplaySystem::express` again), resolve `rset.faces` via `warp3d_face_for_elem`, collect into `traction_loads`, update `has_loads` (`deck.rs:327`) and line `504` with `!traction_loads.is_empty()`, emit `element loads` lines `<we> face <wface> tractions tx <x> ty <y> tz <z>`, and return `Warp3dError::Unsupported` if `rset.faces` is empty (removing the dead nodal fallback; never adding face loads to `nodal_forces`). Add an `#[ignore]` live-WARP3D `Load::Traction` smoke test in `deck.rs` tests.
    *   *(PR 2)* Replace binary `every_step` check (`deck.rs:557-561`) and remove the post-loop domain emission block (`deck.rs:564-568`), driving domain emission inside `for k in 1..=n_steps` via `fracture::evaluated_increments(spec, n_steps)`.
*   **Rationale:** Prevents double unit conversion on traction vectors, completes `Wedge6` export, ensures traction-only steps emit `loading`, and schedules sparse increments in one loop.

#### 5. `crates/warp3d/src/fracture.rs` (Modify — PR 1, PR 2)
*   **Type / Scope:** Layer 2 crack-front chaining, increment scheduling, and `domain` command emitter.
*   **Proposed Changes:**
    *   *(PR 1)* In `order_crack_front` (`fracture.rs:125-260`), accept `ElementKind::Wedge6` alongside `ElementKind::Hex8` (`has_linear = true`), rejecting mixed `Wedge6 + Hex20`; reject any `Wedge6` with `> 1` axial edge in `in_front` or any triangular-cap edge joining `in_front` nodes.
    *   *(PR 2)* Add `evaluated_increments(spec: &CrackFrontSpec, n_steps: usize) -> Vec<usize>`: return `Vec::new()` when `n_steps == 0`; `every_step == Some(true)` -> `(1..=n_steps).collect()`; `increment_interval == Some(k)` -> `{1, k', 2k', ..., n_steps}` (`k' = k.max(1)`) deduplicated/sorted (`vec![1]` when `n_steps == 1`); default -> `vec![n_steps]`.
    *   *(PR 2)* Update `emit_domain_commands` (`fracture.rs:421-474`) to validate `1 <= ring_start <= rings <= MAX_RINGS_PER_DOMAIN`, emit `q-values automatic rings {ring_start}-{rings}`, and when `interaction_integral == Some(true)` emit `plane strain output for interaction integral` or `plane stress output for interaction integral`.
*   **Rationale:** Handles degenerate edge/step inputs without panics and centralizes domain scheduling.

#### 6. `crates/warp3d/src/check.rs` (Modify — PR 1, PR 2)
*   **Type / Scope:** Layer 2 SI pre-flight validator (`validate_resolved`, `validate_model`, `quick_note`).
*   **Proposed Changes:**
    *   *(PR 1)* Accept `ElementKind::Wedge6` in `is_supported_solid_element` (`check.rs:120-122`); update `check_mesh_elements` (`check.rs:316`) error message to list `Wedge6`; migrate unsupported-element test fixtures (`check.rs:1249, 1490`) to `ElementKind::Pyramid5` and update existing `CrackFrontSpec` struct literals (`check.rs:1383, 1509`).
    *   *(PR 1)* Thread `crack` into `check_step_bc_and_load_kinds` (`check.rs:498-566, 1021, 1071`), validating both `Load::Pressure` (`510`) and `Load::Traction` (`503`, using `rset.faces` instead of `nodal_loads` at `743-754`) via `wedge6::wedge6_warp3d_face`. Define `"touching crack_front"` as sharing `>= 1` node in `crack_front`, and emit:
        *   `Warning` `warp3d.wedge_cap_face_load` when `crack_front` is active and `Pressure`/`Traction` loads `Wedge6` triangular cap `face 1` or `2` touching `crack_front`.
        *   `Warning` `warp3d.interaction_crack_face_load` when `crack_front.interaction_integral == Some(true)` and `Load::Traction` acts on faces touching `crack_front` (noting Method-1 `II8` omits `t_2`/`t_3`; use `Load::Pressure` for normal crack pressure).
        *   `Warning` `warp3d.body_load_masks_crack_face` when non-zero `Load::Gravity`/`Load::BodyForce` overlaps face-loaded elements touching `crack_front` (skipping zero vectors).
    *   *(PR 2)* Export `pub(crate) const MAX_DOMAINS: usize = 4_096` from `results.rs:29`. Share `validate_crack_front_spec_fields(spec: &CrackFrontSpec, scope: CheckScope, step_ref: Option<&Step>, diags: &mut Diagnostics)` across `validate_resolved` (`797-832`) and `validate_model` (`1074-1088`): validate `1 <= ring_start <= rings <= MAX_RINGS_PER_DOMAIN`, `increment_interval in 1..=10_000`, `every_step == Some(true) && increment_interval.is_some()` refusal, refusal of `interaction_stress_state` when `interaction_integral != Some(true)`, warning when `increment_interval > n_steps` on `NonlinearStatic`, warning `warp3d.domain_budget_exceeded` when `stations * eval_increments > MAX_DOMAINS`, error `warp3d.invalid_front_average` when `FrontAverage` is requested on `ordered.is_closed` or fails monotonic chord distance `||p_i - p_0|| > ||p_{i-1} - p_0||` (`didriv.f::dickdm`), and `Warning` when any multi-node `Pointwise` front fails monotonic chord distance.
*   **Rationale:** Catches invalid/skipped `dickdm` domains, Method-1 `II8` traps, `dielwf` omissions, and domain-budget overflows before solver launch.

#### 7. `crates/warp3d/src/results.rs` & `crates/warp3d/src/runner.rs` (Modify — PR 2)
*   **Type / Scope:** Layer 2 stdout domain parser, live-solve forwarding (`runner.rs`), and `wes*` flat-file reader.
*   **Proposed Changes:**
    *   Track `interaction_stress_state: Option<InteractionStressState>` inside `DomainAccum`, populate `DomainIntegralResult.interaction_stress_state`, and copy `incoming.interaction_stress_state` in `merge_interaction_into_existing` (`results.rs:531-586`).
    *   Replace `DomainIntegralParser::finish`'s 3-tuple (`results.rs:1218`) with `pub(crate) struct ParsedDomainOutput { domains, dump_steps, truncated, warnings }` shared across `runner.rs:343, 561, 577` and `read_results_dir_with_preparsed_fracture` (`results.rs:1408`).
    *   In `DomainIntegralParser::feed_line` (`results.rs:637-646, 770-783, 977-979`), match `>>> Invalid domain definition`, `>>> Warning:`, `>>> Area under q-function`, `>>>>> warning:`, and `>>>>> error:` lines **before** the `!is_domain_start` early-return gate into bounded deduplicated `warnings` (`MAX_DOMAIN_WARNINGS = 32`); verify/raise `MAX_DOMAIN_LINES` (`results.rs:28`, e.g. `2_000_000`) for `4_096 × 64`-ring interaction output; and extend `warnings` with domain warnings **before** constructing `static_result.warnings` at `results.rs:1666`.
    *   In `read_results_dir_inner` (`results.rs:1502-1684`), inspect `step_files.get(&1)`'s `wes` outside the `is_last`/`frames_full` skips (`1512, 1630`) using a streaming line-by-line column-8 (`epspls`) scanner (reusing parsed `final_elems` on 1-step runs) to set `step1_is_elastic: Option<bool>`: `Some(true)` iff Increment-1 `wes*` is present and completely parsed with `max_peeq <= 1e-12` (`// unit-factors: dimensionless plastic strain zero threshold`), `Some(false)` when any element has `peeq > 1e-12`, and `None` when missing, empty, or truncated.
*   **Rationale:** Ensures mid-run `didriv.f` diagnostics surface through `static_result.warnings` and avoids multi-MB `wes00001*` table allocations.

#### 8. `crates/warp3d/src/ffs.rs` (Modify — PR 2, PR 3)
*   **Type / Scope:** Layer 2 SI FFS evaluation engine (`summarize_fracture_spread`, `evaluate_step_ffs_detailed`, `FfsError`).
*   **Proposed Changes:**
    *   *(PR 2)* Fix `RingExclusion::Auto` (`ffs.rs:637-648`): exclude `sorted_rings[0]` **only** when `sorted_rings.len() >= 3 && sorted_rings[0].ring == 1`.
    *   *(PR 2)* Guard `can_linear_scale_step1` (`ffs.rs:1371-1386`) with `first_step.is_some_and(|fs| fs.warp_step == 1) && step1_is_elastic != Some(false)`.
    *   *(PR 3)* Reuse `RingIntegral` on `StationProfile` (`ffs.rs:193-216`): add `#[serde(default, skip_serializing_if = "Option::is_none")] pub interaction_stress_state: Option<InteractionStressState>` and `#[serde(default)] pub rings: Vec<RingIntegral>` for `active_profiles` only (UI checks `!prof.excluded_rings.contains(&r.ring)`).
    *   *(PR 3)* Add `StepStationPoint` (`warp_step: usize`, `frame_index: usize`, `load_factor: f64`, `j_mean: f64`, `k_j_plane_strain: f64`, `k_j_plane_stress: f64`, `k_i_mean: Option<f64>`, `k_ii_mean: Option<f64>`, `k_iii_mean: Option<f64>`, `t_11_mean: Option<f64>`, `t_33_mean: Option<f64>`) and `EvaluatedStep` (`warp_step: usize`, `frame_index: usize`, `load_factor: f64`).
    *   *(PR 3)* Extend `FfsReport` (`ffs.rs:272-284`) with `#[serde(default)] pub step_history: Vec<StepStationPoint>`, `#[serde(default)] pub evaluated_steps: Vec<EvaluatedStep>`, `#[serde(default, skip_serializing_if = "Option::is_none")] pub step_history_domain_id: Option<String>`, and `#[serde(default, skip_serializing_if = "Option::is_none")] pub step1_is_elastic: Option<bool>`.
    *   *(PR 3)* Extend `FfsRequest` (`ffs.rs:180-190`) with `pub domain_id: Option<String>` and `pub step1_is_elastic: Option<bool>`. Resolve `step_history` target by `req.domain_id` → `fad.domain_id` (when `StationSelection::DomainId`) → governing peak-`J` station across `all_profiles` at the latest evaluated step when `active_profiles` is empty; return `FfsError::UnknownDomain(String)` if a requested `domain_id` does not exist. Keep `FadOption` extensible for PR 5.
*   **Rationale:** Reuses existing ring and step-history structures, preserves Layer 2 error separation, and provides direct `frame_index` navigation.

#### 9. `crates/engine/src/lib.rs` (Modify — PR 2)
*   **Type / Scope:** Layer 6 `Session` in-memory fracture storage and canonical crack-front API.
*   **Proposed Changes:**
    *   Re-export `pub const ALLOWED_CRACK_FRONT_KEYS: &[&str]` and `pub fn canonicalize_crack_front(val: &Value) -> Option<Value>` backed by `cmd::deck::parse_crack_front_with` + `serde_json::to_value` (`skip_serializing_if = "Option::is_none"` strips unset optional keys and `every_step: Some(false)`).
    *   Add `pub step1_is_elastic: Option<bool>` to `Warp3dFractureData` (`lib.rs:343-347`, keeping `FractureInputKey` unchanged) and update `SolvedFracture` (`352-355`), `Session::store_fracture_results` (`863-885`), and `Session::store_solved_step` (`1010-1022`) plus any engine solve-worker constructors.
*   **Rationale:** Centralizes `CrackFrontSpec` key/value canonicalization at Layer 6 and propagates `step1_is_elastic` through every solve path without breaking incremental PR 2 compilation.

#### 10. `crates/engine/src/cmd/deck.rs`, `crates/engine/src/cmd/solve.rs`, `crates/engine/src/cmd/warp3d.rs` (Modify — PR 2)
*   **Type / Scope:** Layer 6 `crack_front` parameter parsing and solve result wiring.
*   **Proposed Changes:**
    *   In `cmd/deck.rs::parse_crack_front_with` (`cmd/deck.rs:298-383`): define `ALLOWED_CRACK_FRONT_KEYS` rejecting unknown keys; raise `rings` range to `1..=64`; parse `ring_start` (`1..=64`, rejecting `ring_start > rings` on `crack_front/ring_start`); parse `increment_interval` (`1..=10_000`, rejecting `every_step == Some(true)` with `increment_interval.is_some()` on `crack_front/increment_interval`); parse `interaction_stress_state` (`"plane_strain" | "plane_stress"`, rejecting when `interaction_integral != Some(true)` on `crack_front/interaction_stress_state`).
    *   Update `cmd/solve.rs` (`574, 600`) to pass `sol.step1_is_elastic` to `s.store_fracture_results(...)`.
    *   Update `cmd/warp3d.rs::CHECK` (`12`) docstring/params example to reflect updated `CRACK_FRONT`.
*   **Rationale:** Keeps strict field-attributed validation identical across CLI, MCP, scripts, and UI.

#### 11. `crates/engine/src/cmd/forms.rs` & `docs/features.md` (Modify — PR 2, PR 3)
*   **Type / Scope:** Layer 6 form definitions and generated command reference.
*   **Proposed Changes:**
    *   *(PR 2)* Keep `CRACK_FRONT` (`forms.rs:2748-2763`) flat `&[Field]` ordered basic-to-advanced (no `Kind::Group`); update `set` help to name `Hex8/Hex20/Wedge6`; raise `rings` max to `64`; add `ring_start` (`int`, `1..=64`, `def("1")`), `interaction_stress_state` (`choice`, `"plane_strain" | "plane_stress"`, using `.hint(...)` rather than `.def("\"plane_strain\"")` so `schema::prefill` omits it unless chosen), and `increment_interval` (`int`, `1..=10_000`, omitting `every_step` from the form).
    *   *(PR 3)* Update `RESULTS_FFS` (`forms.rs:3316-3329`) to add optional top-level `text("domain_id", "Station domain ID")` documenting precedence and `Load History` tracking.
    *   Regenerate `docs/features.md` via `cargo run -p fieldcraft-cli -- features > docs/features.md` in each commit modifying `forms.rs` / `CommandSpec`.
*   **Rationale:** Ensures fresh UI forms (`schema::prefill`) do not fail validation on `interaction_stress_state` and keeps flat JSON paths intact.

#### 12. `crates/engine/src/cmd/ffs.rs`, `crates/engine/src/cmd/results.rs`, `crates/engine/src/units.rs` (Modify — PR 2, PR 3)
*   **Type / Scope:** Layer 6 solve-read wiring (`PR 2`), `results.ffs`, `results.export.csv`, and display-unit conversion (`PR 3`).
*   **Proposed Changes:**
    *   *(PR 2)* Update `cmd/results.rs:1349` (`results.read`, passing `sol.step1_is_elastic`) and test helper `seeded_fracture_session` (`cmd/ffs.rs:738`, passing `None`) for the `store_fracture_results` signature update.
    *   *(PR 3)* In `cmd/ffs.rs`: update `FFS.params` docstring (`21`); accept top-level `domain_id: Option<String>` (if both top-level `domain_id` and `fad.domain_id` are explicitly provided with different values, return `EngineError::invalid("domain_id", ...)`), pass `domain_id` and `frac.step1_is_elastic` via `FfsRequest`, map `FfsError::UnknownDomain` to `EngineError::invalid("domain_id", ...)`, and append `interaction_stress_state`, `k_ii_mean`, `k_iii_mean`, `t_33_mean` columns to `ffs_csv_table` (`403-435`, preserving `cmd/ffs.rs:904`'s required header prefix, using `Cell::Empty` when `None`, `"plane_strain"`/`"plane_stress"`, and unit-titled headers).
    *   *(PR 3)* In `cmd/results.rs::fracture_table` (`1259-1316`): append `"warp_step"` (`Cell::Number(d.step as f64)`) and `"interaction_stress_state"` (`Cell::Text` / `Cell::Empty`).
    *   *(PR 3)* In `units.rs::FFS` (`944-968`): register `/active_profiles/*/rings/*/j`, `.../t_11`, `.../t_33`, `.../k_i`, `.../k_ii`, `.../k_iii`, and `/step_history/*/j_mean`, `.../t_11_mean`, `.../t_33_mean`, `.../k_i_mean`, `.../k_ii_mean`, `.../k_iii_mean`, `.../k_j_plane_strain`, `.../k_j_plane_stress` with matching `Dim` rules.
*   **Rationale:** Keeps PR 2 compilable on every commit, preserves CSV prefix tests, and keeps Layer 6 as the UI display-unit boundary.

#### 13. `crates/ui-egui/src/lib.rs` & `crates/ui-egui/src/files.rs` (Modify — PR 2)
*   **Type / Scope:** Layer 7 desktop session `active_crack_front` cache (`CachedCrackFront`) and tests.
*   **Proposed Changes:**
    *   Replace `CachedCrackFront`'s fixed fields (`ui-egui/src/lib.rs:406-461`) with a thin wrapper around a `serde_json::Map<String, Value>` normalized via `fieldcraft_engine::canonicalize_crack_front` (preserving `set(&self) -> &str` / `set_mut` for `model.rename`).
    *   Update direct `CachedCrackFront` callers and tests in `ui-egui/src/lib.rs:1628, 1632-1648` and `crates/ui-egui/src/files.rs:3785-3793, 3908-3928`.
*   **Rationale:** Eliminates Layer 7 validation duplication and silent field drops across UI forms.

#### 14. `crates/ui-egui/src/ffs_panel.rs` (Modify — PR 3)
*   **Type / Scope:** Layer 7 container-agnostic Fitness-for-Service panel.
*   **Proposed Changes:**
    *   Add `FfsChartTab` (`FrontProfile`, `RingConvergence`, `LoadHistory`) to `FfsState`, serialized in `FfsState::inspect_json` under `"chart_tab": "front_profile" | "ring_convergence" | "load_history"`. In `FfsState::build_params`, pass top-level `domain_id` from `state.selected_domain` only when `!(state.fad_enabled && state.fad_station_mode == DomainId)`. Guard `show_contents` (`706-708`) with `if !report.active_profiles.is_empty()` so unevaluated sparse frames preserve `state.selected_domain`.
    *   Extend `ProfileQuantity` (`33-72`) with `Kii`, `Kiii`, `T33`. Wrap `show_contents()` (`665-842`) in `ui.push_id("fc_ffs_contents", ...)`, keeping it decoupled from `shell.rs` strip layout.
    *   Show tooltips/legends with `K_II`, `K_III`, `T_33`, active stress-state badge (`Plane strain` / `Plane stress`), and local-axis legend `(x1=advance, x2=normal, x3=tangent)` (without displaying an unverified crack-plane symmetry note). Update `Exclude Ring 1` (`743-750`) using `ui.add_enabled(has_ring_1, ...)` with tooltip and `Rings {min}–{max}` indicator.
    *   Show a jump banner when `active_profiles` is empty on an evaluated model using `evaluated_steps` to jump via `sync_frame` (`727, 836-838`) -> `results.frame`. Render `Front Profile`, `Ring Convergence` (`J` vs. `RingIntegral::ring`, rendering `!prof.excluded_rings.contains(&r.ring)` hollow vs. filled beside `j_mean`), and `Load History` (active `state.quantity` vs. `load_factor`, threading `&mut sync_frame` on point click) inside a segmented tab bar beside the existing `WIDE_SPLIT_MIN_WIDTH` optional `API 579 FAD` split.
*   **Rationale:** Exposes mixed-mode, convergence, and load-history charts in today's bottom strip inside a `#1088`-ready `show_contents()` boundary without adding speculative 2×2 grid layout code.

#### 15. `crates/warp3d/tests/fracture_benchmarks.rs` (Create — PR 4) & `docs/warp3d.md` (Modify — PR 1–4)
*   **Type / Scope:** Clean-room stationary 3D fracture verification test suite and documentation.
*   **Proposed Changes:**
    *   Create procedural mesh builders inside `crates/warp3d/tests/fracture_benchmarks.rs`: fast trunk checks (rosette topology/Jacobian, deck golden syntax, parser/unit checks, API 579-1 Part 9 curve checks `<= 1e-6` relative) plus `#[ignore = "heavy: cargo xtask test --heavy (CI heavy job)"]` live-WARP3D runs verifying Sneddon (`<= 3%` $K_J$, `<= 2%` outer-ring spread), thin plate (`|T_{33}|/|T_{11}| <= 1e-3`, `<= 2%` $K_I$/$K_J$), Newman & Raju 1981 / Wang 2004 (`<= 5%` interior $K_I$, `<= 10%` $T$-stress), and nonlinear multi-increment API 579-1 checks.
    *   Update `docs/warp3d.md` per PR with implemented syntax, Method-1 `II8` documentation, `CrackFrontSpec` fields, and PR 4's verified benchmark tolerance table.
*   **Rationale:** Satisfies `AGENTS.md` clean-room geometry rules and keeps CI fast while establishing reproducible quantitative tolerances.

---

### Verification Plan

*   **Compilation & Wasm:**
    *   `cargo check -p fieldcraft-warp3d -p fieldcraft-engine -p fieldcraft-ui-egui`
    *   `cargo check -p fieldcraft-warp3d -p fieldcraft-engine -p fieldcraft-ui-egui --target wasm32-unknown-unknown`
*   **Unit & Integration Tests (per PR, trunk gate):**
    *   PR 1: `cargo test -p fieldcraft-warp3d` (validating `wedge6` 3×5 bijection and out-of-range guards, 12-pattern `collapse_hex8_to_wedge6` + positive Jacobian checks, mixed `Wedge6+Hex8` chaining + multi-edge refusal, single-conversion `Load::Traction` `element loads` + traction-only `has_loads`, `check.rs` cap-face/`II8`/gravity warnings).
    *   PR 2: `cargo test -p fieldcraft-warp3d -p fieldcraft-engine -p fieldcraft-ui-egui` (validating `forms::tests::*` + flat `CRACK_FRONT` prefill without `interaction_stress_state`, `ALLOWED_CRACK_FRONT_KEYS`, `canonicalize_crack_front`, `evaluated_increments` with `n_steps == 0` / `k == 0`, `RingExclusion::Auto` ring-1 check, streaming `step1_is_elastic` + `KrSource::Auto` guard, `didriv.f` monotonic-chord refusal, `>>>` warning capture before `feed_line` early return, `CachedCrackFront` `model.rename`).
    *   PR 3: `cargo test -p fieldcraft-warp3d -p fieldcraft-engine -p fieldcraft-ui-egui` (validating `units.rs::FFS` across `Preset::ALL`, `RingIntegral` + `step_history` + `EvaluatedStep` payloads, `domain_id` precedence & conflict errors, CSV column order/prefixes, `ffs_panel` headless tabbed rendering + `ui.push_id`, `selected_domain` preservation on empty frames, `Load History` `sync_frame`, jump banner).
    *   PR 4 (trunk): `cargo test -p fieldcraft-warp3d` (procedural mesh checks, deck golden checks, API 579 Part 9 checks `<= 1e-6`).
*   **Heavy Live-WARP3D Tests (`WARP3D_HOME` + `FIELDCRAFT_REQUIRE_WARP3D=1`):**
    *   PR 1: `cargo test -p fieldcraft-warp3d deck::tests::load_traction -- --ignored` (verifying live WARP3D `Load::Traction` execution and `dm5` `J` output).
    *   PR 4: `cargo test -p fieldcraft-warp3d --test fracture_benchmarks -- --ignored` (verifying Sneddon, thin plate, Newman & Raju / Wang, and nonlinear API 579-1 against numeric tolerance targets).
*   **Static Analysis, Formatting & Workspace Invariants:**
    *   `cargo fmt --all -- --check`
    *   `cargo clippy -p fieldcraft-warp3d -p fieldcraft-engine -p fieldcraft-ui-egui --all-targets -- -D warnings`
    *   `cargo xtask layers` and `cargo xtask unit-factors`
    *   Verify `docs/features.md` matches `cargo run -p fieldcraft-cli -- features` on commits changing command schemas.

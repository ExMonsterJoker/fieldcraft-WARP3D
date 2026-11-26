# WARP3D Integration (`fieldcraft-warp3d`)

`fieldcraft-warp3d` integrates the open-source [WARP3D](https://github.com/warp3d/warp3d) nonlinear solid and fracture mechanics solver into FieldCraft.

## Layering and Scope

`fieldcraft-warp3d` sits at **Layer 2** in [`docs/architecture.md`](architecture.md) alongside `fieldcraft-solver`, `fieldcraft-explicit`, and `fieldcraft-lbm`, with intra-layer order `solver -> warp3d` in `xtask/src/layers.rs`.

Placing it at L2 instead of inside `fieldcraft-decks` (L1) keeps L1 strictly for pure file-format serialization while allowing `fieldcraft-warp3d` to own the full solver adapter lifecycle:

1. **Implemented (Stages 1–4A):** Complete ASCII `.inp` deck generation (geometry, `LabelMap` section grouping, materials, constraints/MPCs, loads, nonlinear analysis parameters, flat-text outputs), crack-front node ordering and `domain` J/interaction-integral command emission, external `warp3d` binary discovery/execution with streaming stdout/stderr drain, cancellation, and timeout, flat-file displacement/reaction/stress/plastic-strain parsing and `J` / `K_I` / `K_II` / `K_III` / `T`-stress domain-integral parsing back into SI, `deck.write` (`"format": "warp3d"` + `"crack_front"`), `solve.static` / `solve.nonlinear` (`backend: "warp3d"` / `FIELDCRAFT_STRUCTURAL_BACKEND`), `results.read` (`"format": "warp3d"`), `results.export.csv` (`"what": "fracture"`), and `wasm32`-clean runner/result interfaces.
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

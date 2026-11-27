//! Clean-room stationary 3D fracture verification suite.
//!
//! All benchmark geometries and meshes are generated procedurally in pure Rust using `Model`
//! primitives (no commercial CAD/CAE meshes or external files).
//!
//! Two-tier execution:
//! - Trunk CI (`cargo test -p fieldcraft-warp3d --test fracture_benchmarks`): validates element
//!   Jacobians, outward face normals, `check::validate_model`, `.inp` deck syntax/unit presets,
//!   analytical reference helpers, and API 579-1 Part 9 closed-form checks without requiring a
//!   WARP3D executable.
//! - Heavy live-WARP3D suite (`cargo test -p fieldcraft-warp3d --test fracture_benchmarks --
//!   --ignored --nocapture`): runs WARP3D 18.5.0 on Sneddon (1946) penny crack, thin-plate CCT
//!   (plane stress vs plane strain), Newman & Raju (1981) / Wang (2003) semi-elliptical surface
//!   crack, and nonlinear elastic-plastic API 579-1 checks (skipping cleanly on `BinaryNotFound`
//!   unless `FIELDCRAFT_REQUIRE_WARP3D=1`).

use std::collections::BTreeSet;
use std::f64::consts::PI;
use std::path::PathBuf;
use std::sync::atomic::AtomicBool;

use fieldcraft_model::units::Preset;
use fieldcraft_model::{
    BoundaryCondition, Elastic, ElementFace, ElementKind, Hardening, Load, Material, Model, NonlinearStatic, Plastic, Procedure, Section,
    SectionKind, Set, Step, Symmetry, Target,
};
use fieldcraft_warp3d::{
    CheckScope, CrackFrontSpec, CrackFunctionType, FadConfig, FadOption, FfsRequest, InteractionStressState, KrSource, LrInput, Option2Curve,
    Option2Evaluator, RingExclusion, StationSelection, ToughnessInput, Warp3dError, Warp3dSolution, build_station_profiles, evaluate_fad_curve,
    evaluate_step_ffs_detailed, reserve_factor, run_warp3d, run_warp3d_with_override, validate_model, write_deck,
};

const E_STEEL: f64 = 2.0e11;
const NU_STEEL: f64 = 0.3;

// ---------------------------------------------------------------------------
// Published analytical & empirical reference equations
// ---------------------------------------------------------------------------

/// Sneddon (1946) pressurized penny-shaped crack of radius `a` under uniform pressure `p`:
/// $K_I = \frac{2}{\pi} p \sqrt{\pi a}$.
fn sneddon_penny_ki(pressure: f64, a: f64) -> f64 {
    (2.0 / PI) * pressure * (PI * a).sqrt()
}

/// Wang (2004, Eng. Fract. Mech. 71: 2283-2298) penny-shaped crack T-stress under uniform crack
/// pressure `p`: $T_{11} = -\frac{1 + 2\nu}{2} p$.
fn wang_2004_penny_t11(pressure: f64, nu: f64) -> f64 {
    -0.5 * (1.0 + 2.0 * nu) * pressure
}

/// Feddersen (1966) finite-width CCT Mode-I stress intensity factor:
/// $K_I = \sigma \sqrt{\pi a} \sqrt{\sec(\pi a / (2 W))}$.
fn feddersen_cct_ki(sigma: f64, a: f64, half_width: f64) -> f64 {
    let sec = 1.0 / ((PI * a) / (2.0 * half_width)).cos();
    sigma * (PI * a).sqrt() * sec.sqrt()
}

/// Newman & Raju (1981, Eng. Fract. Mech. 15: 185-192) empirical Mode-I SIF for a semi-elliptical
/// surface crack (`a/c <= 1.0`) under uniform remote tension `sigma`.
fn newman_raju_1981_ki(sigma: f64, a: f64, c: f64, t: f64, b: f64, phi: f64) -> f64 {
    let ac = a / c;
    let at = a / t;
    let cb = c / b;
    let q = 1.0 + 1.464 * ac.powf(1.65);
    let m1 = 1.13 - 0.09 * ac;
    let m2 = -0.54 + 0.89 / (0.2 + ac);
    let m3 = 0.5 - 1.0 / (0.65 + ac) + 14.0 * (1.0 - ac).powi(24);
    let g = 1.0 + (0.1 + 0.35 * at * at) * (1.0 - phi.sin()).powi(2);
    let f_phi = (ac * ac * phi.cos().powi(2) + phi.sin().powi(2)).powf(0.25);
    let f_w = (1.0 / ((PI * cb * 0.5) * at.sqrt()).cos()).sqrt();
    let f = (m1 + m2 * at * at + m3 * at.powi(4)) * f_phi * g * f_w;
    sigma * ((PI * a) / q).sqrt() * f
}

// ---------------------------------------------------------------------------
// Procedural 2D crack-tip rosette + 3D sweep/revolve mesh helpers
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
enum Elem2D {
    Tri { nodes: [usize; 3], crack_face_edge: bool },
    Quad { nodes: [usize; 4], crack_face_edge: Option<(usize, usize)>, top_edge: Option<(usize, usize)> },
}

struct Mesh2D {
    uv: Vec<[f64; 2]>,
    elems: Vec<Elem2D>,
    tip_idx: usize,
    u_zero_nodes: BTreeSet<usize>,
    ligament_nodes: BTreeSet<usize>,
}

fn graded_1d(start: f64, end: f64, n: usize, power: f64, cluster_at_start: bool) -> Vec<f64> {
    (0..=n)
        .map(|i| {
            let t = (i as f64) / (n as f64);
            let s = if cluster_at_start { t.powf(power) } else { 1.0 - (1.0 - t).powf(power) };
            start + (end - start) * s
        })
        .collect()
}

/// Build a conforming 2D half-plane `[0, u_max] x [0, v_max]` mesh with crack tip at `(u_tip, 0)`,
/// crack face along `v = 0, u in [0, u_tip]`, and uncracked ligament along `v = 0, u in [u_tip, u_max]`.
fn build_2d_crack_mesh(u_tip: f64, u_max: f64, v_max: f64, d_box: f64, n_left: usize, n_right: usize, n_top: usize) -> Mesh2D {
    let mut uv: Vec<[f64; 2]> = Vec::new();
    let mut elems: Vec<Elem2D> = Vec::new();

    // 7 concentric circular rings inside the square box `[u_tip - d_box, u_tip + d_box] x [0, d_box]`
    // plus 2 transition rings to match the box boundary cleanly.
    let circ_fracs = [0.03_f64, 0.06, 0.10, 0.16, 0.24, 0.34, 0.46];
    let n_ang = 8usize;
    let tip_idx = 0usize;
    uv.push([u_tip, 0.0]);

    let mut ring_nodes: Vec<Vec<usize>> = Vec::with_capacity(circ_fracs.len() + 2);
    for &frac in &circ_fracs {
        let r = frac * d_box;
        let mut row = Vec::with_capacity(n_ang + 1);
        for j in 0..=n_ang {
            let alpha = PI * (j as f64) / (n_ang as f64);
            let v = if j == 0 || j == n_ang { 0.0 } else { r * alpha.sin() };
            row.push(uv.len());
            uv.push([u_tip + r * alpha.cos(), v]);
        }
        ring_nodes.push(row);
    }

    // Two transition rings: t = 0.5 (intermediate) and t = 1.0 (exact box perimeter).
    let mut box_perim = Vec::with_capacity(n_ang + 1);
    let r_circ_outer = circ_fracs[circ_fracs.len() - 1] * d_box;
    for t in [0.5_f64, 1.0] {
        let mut row = Vec::with_capacity(n_ang + 1);
        for j in 0..=n_ang {
            let alpha = PI * (j as f64) / (n_ang as f64);
            let ca = alpha.cos();
            let sa = alpha.sin();
            let s_box = d_box / ca.abs().max(sa);
            let r = r_circ_outer + t * (s_box - r_circ_outer);
            let mut u = u_tip + r * ca;
            let mut v = r * sa;
            if j == 0 || j == n_ang {
                v = 0.0;
            }
            if (t - 1.0).abs() < 1e-12 {
                if j <= 2 {
                    u = u_tip + d_box;
                } else if j >= 6 {
                    u = u_tip - d_box;
                }
                if (2..=6).contains(&j) {
                    v = d_box;
                }
            }
            row.push(uv.len());
            uv.push([u, v]);
        }
        if (t - 1.0).abs() < 1e-12 {
            box_perim = row.clone();
        }
        ring_nodes.push(row);
    }

    // Ring 1: triangles around tip.
    for j in 0..n_ang {
        elems.push(Elem2D::Tri { nodes: [tip_idx, ring_nodes[0][j], ring_nodes[0][j + 1]], crack_face_edge: j + 1 == n_ang });
    }
    // Rings 2..=9: quads around tip.
    for r_idx in 1..ring_nodes.len() {
        for j in 0..n_ang {
            elems.push(Elem2D::Quad {
                nodes: [ring_nodes[r_idx - 1][j], ring_nodes[r_idx][j], ring_nodes[r_idx][j + 1], ring_nodes[r_idx - 1][j + 1]],
                crack_face_edge: (j + 1 == n_ang).then_some((2, 3)),
                top_edge: None,
            });
        }
    }

    // Exterior structured 3 x 2 block grid around `[u_tip - d_box, u_tip + d_box] x [0, d_box]`.
    let u_left = graded_1d(0.0, u_tip - d_box, n_left, 1.5, false);
    let u_ctr = [uv[box_perim[6]][0], uv[box_perim[5]][0], uv[box_perim[4]][0], uv[box_perim[3]][0], uv[box_perim[2]][0]];
    let u_right = graded_1d(u_tip + d_box, u_max, n_right, 1.7, true);

    let v_bot = [0.0, uv[box_perim[1]][1], d_box];
    let v_top = graded_1d(d_box, v_max, n_top, 1.7, true);

    let nu_total = n_left + 4 + n_right;
    let nv_total = 2 + n_top;

    // Build global exterior node index table `(iu, iv)` reusing `box_perim` on the box boundary.
    let mut grid: Vec<Vec<usize>> = vec![vec![usize::MAX; nv_total + 1]; nu_total + 1];
    // Box perimeter mappings:
    grid[n_left][0] = box_perim[8];
    grid[n_left][1] = box_perim[7];
    grid[n_left][2] = box_perim[6];
    grid[n_left + 1][2] = box_perim[5];
    grid[n_left + 2][2] = box_perim[4];
    grid[n_left + 3][2] = box_perim[3];
    grid[n_left + 4][2] = box_perim[2];
    grid[n_left + 4][1] = box_perim[1];
    grid[n_left + 4][0] = box_perim[0];

    let u_all = |iu: usize| -> f64 {
        if iu <= n_left {
            u_left[iu]
        } else if iu <= n_left + 4 {
            u_ctr[iu - n_left]
        } else {
            u_right[iu - (n_left + 4)]
        }
    };
    let v_all = |iv: usize| -> f64 { if iv <= 2 { v_bot[iv] } else { v_top[iv - 2] } };

    for (iu, col) in grid.iter_mut().enumerate() {
        for (iv, cell) in col.iter_mut().enumerate() {
            // Skip strictly interior points of the rosette box `[n_left..n_left+4] x [0..2]`.
            let inside_box_u = iu > n_left && iu < n_left + 4;
            let on_box_u = iu == n_left || iu == n_left + 4;
            if iv < 2 && (inside_box_u || on_box_u) {
                continue;
            }
            if *cell == usize::MAX {
                *cell = uv.len();
                uv.push([u_all(iu), v_all(iv)]);
            }
        }
    }

    for iu in 0..nu_total {
        for iv in 0..nv_total {
            if iv < 2 && iu >= n_left && iu < n_left + 4 {
                continue;
            }
            let n0 = grid[iu][iv];
            let n1 = grid[iu + 1][iv];
            let n2 = grid[iu + 1][iv + 1];
            let n3 = grid[iu][iv + 1];
            let is_crack = iv == 0 && iu < n_left;
            let is_top = iv + 1 == nv_total;
            elems.push(Elem2D::Quad { nodes: [n0, n1, n2, n3], crack_face_edge: is_crack.then_some((0, 1)), top_edge: is_top.then_some((2, 3)) });
        }
    }

    let mut u_zero_nodes = BTreeSet::new();
    let mut ligament_nodes = BTreeSet::new();

    for (idx, &[u, v]) in uv.iter().enumerate() {
        if u.abs() < 1e-12 {
            u_zero_nodes.insert(idx);
        }
        if v.abs() < 1e-12 && u >= u_tip - 1e-12 {
            ligament_nodes.insert(idx);
        }
    }

    Mesh2D { uv, elems, tip_idx, u_zero_nodes, ligament_nodes }
}

/// Map local 2D quad edge `(a, b)` in `[n0, n1, n2, n3]` extruded `m -> m+1` to 1-based `Hex8` face.
fn extruded_hex_face(edge: (usize, usize)) -> u8 {
    match edge {
        (0, 1) => 3, // [0, 1, 5, 4]
        (1, 2) => 4, // [1, 2, 6, 5]
        (2, 3) => 5, // [2, 3, 7, 6]
        (3, 0) => 6, // [3, 0, 4, 7]
        _ => panic!("unexpected 2D quad edge {edge:?}"),
    }
}

/// Build the 1/8-symmetry Sneddon (1946) pressurized penny-shaped crack cylinder (`a = 0.01 m`).
fn build_sneddon_penny_crack(use_traction: bool) -> Model {
    let a = 0.01_f64;
    let r_max = 15.0 * a;
    let h_max = 15.0 * a;
    let d_box = 0.35 * a;
    let m2d = build_2d_crack_mesh(a, r_max, h_max, d_box, 4, 8, 8);

    let n_theta = 16usize;
    let mut model = Model::default();

    // Node table `(node_2d, slice_k)` -> 1-based label; nodes on axis `r == 0` are shared across slices.
    let mut axis_labels: Vec<Option<u32>> = vec![None; m2d.uv.len()];
    let mut slice_labels: Vec<Vec<u32>> = vec![vec![0; m2d.uv.len()]; n_theta + 1];
    let mut next_nid = 1u32;

    for (i2, &[_r, y]) in m2d.uv.iter().enumerate() {
        if m2d.u_zero_nodes.contains(&i2) {
            let lbl = next_nid;
            next_nid += 1;
            model.set_node(lbl, [0.0, y, 0.0]).unwrap();
            axis_labels[i2] = Some(lbl);
            for row in &mut slice_labels {
                row[i2] = lbl;
            }
        }
    }
    for (k, row) in slice_labels.iter_mut().enumerate() {
        let theta = (PI * 0.5) * (k as f64) / (n_theta as f64);
        let ct = theta.cos();
        let st = theta.sin();
        for (i2, &[r, y]) in m2d.uv.iter().enumerate() {
            if axis_labels[i2].is_some() {
                continue;
            }
            let lbl = next_nid;
            next_nid += 1;
            let x = if k == n_theta { 0.0 } else { r * ct };
            let z = if k == 0 { 0.0 } else { r * st };
            model.set_node(lbl, [x, y, z]).unwrap();
            row[i2] = lbl;
        }
    }

    let mut all_elems = Vec::new();
    let mut crack_faces = Vec::new();
    let mut next_eid = 1u32;

    for k in 0..n_theta {
        for e2 in &m2d.elems {
            match *e2 {
                Elem2D::Tri { nodes: [a0, a1, a2], crack_face_edge } => {
                    let conn = vec![
                        slice_labels[k][a0],
                        slice_labels[k][a1],
                        slice_labels[k][a2],
                        slice_labels[k + 1][a0],
                        slice_labels[k + 1][a1],
                        slice_labels[k + 1][a2],
                    ];
                    model.set_element(next_eid, ElementKind::Wedge6, conn).unwrap();
                    all_elems.push(next_eid);
                    if crack_face_edge {
                        // Edge (a2, a0) swept k -> k+1 is Wedge6 face 5 ([2, 0, 3, 5]).
                        crack_faces.push(ElementFace { element: next_eid, face: 5 });
                    }
                    next_eid += 1;
                }
                Elem2D::Quad { nodes: [q0, q1, q2, q3], crack_face_edge, .. } => {
                    let on_axis_0 = axis_labels[q0].is_some();
                    let on_axis_3 = axis_labels[q3].is_some();
                    if on_axis_0 && on_axis_3 {
                        // Axis-adjacent quad collapses q0 and q3 across theta slices into a Wedge6
                        // whose axial edge (q0 -> q3) runs along +Y.
                        let a0 = axis_labels[q0].unwrap();
                        let b0 = axis_labels[q3].unwrap();
                        let a1_k = slice_labels[k][q1];
                        let a1_kp = slice_labels[k + 1][q1];
                        let b1_k = slice_labels[k][q2];
                        let b1_kp = slice_labels[k + 1][q2];
                        // Bottom cap (v = v0): [a0, a1_{k+1}, a1_k]; top cap (v = v1): [b0, b1_{k+1}, b1_k].
                        model.set_element(next_eid, ElementKind::Wedge6, vec![a0, a1_kp, a1_k, b0, b1_kp, b1_k]).unwrap();
                        all_elems.push(next_eid);
                        if crack_face_edge.is_some() {
                            // Bottom cap (face 1 = [0, 2, 1]) lies on y = 0 crack face; away from crack_front.
                            crack_faces.push(ElementFace { element: next_eid, face: 1 });
                        }
                        next_eid += 1;
                    } else {
                        let conn = vec![
                            slice_labels[k][q0],
                            slice_labels[k][q1],
                            slice_labels[k][q2],
                            slice_labels[k][q3],
                            slice_labels[k + 1][q0],
                            slice_labels[k + 1][q1],
                            slice_labels[k + 1][q2],
                            slice_labels[k + 1][q3],
                        ];
                        model.set_element(next_eid, ElementKind::Hex8, conn).unwrap();
                        all_elems.push(next_eid);
                        if let Some(edge) = crack_face_edge {
                            crack_faces.push(ElementFace { element: next_eid, face: extruded_hex_face(edge) });
                        }
                        next_eid += 1;
                    }
                }
            }
        }
    }

    let mut sym_z = BTreeSet::new();
    let mut sym_x = BTreeSet::new();
    let mut ligament = BTreeSet::new();
    let mut front = Vec::with_capacity(n_theta + 1);
    for slice in &slice_labels {
        front.push(slice[m2d.tip_idx]);
        for &i2 in &m2d.ligament_nodes {
            ligament.insert(slice[i2]);
        }
    }
    for &lbl in &slice_labels[0] {
        sym_z.insert(lbl);
    }
    for &lbl in &slice_labels[n_theta] {
        sym_x.insert(lbl);
    }

    model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: E_STEEL, poisson: NU_STEEL }), ..Default::default() }).unwrap();
    model.add_set("all", Set { elements: all_elems, ..Set::default() }).unwrap();
    model.add_set("sym_z", Set { nodes: sym_z.into_iter().collect(), ..Set::default() }).unwrap();
    model.add_set("sym_x", Set { nodes: sym_x.into_iter().collect(), ..Set::default() }).unwrap();
    model.add_set("ligament", Set { nodes: ligament.into_iter().collect(), ..Set::default() }).unwrap();
    model.add_set("crack_front", Set { nodes: front, ..Set::default() }).unwrap();
    model.add_set("crack_faces", Set { faces: crack_faces, ..Set::default() }).unwrap();
    model
        .add_section("solid", Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() })
        .unwrap();
    model.add_boundary_condition("bc_z", BoundaryCondition::Symmetry { target: Target::Set("sym_z".into()), plane: Symmetry::Z }).unwrap();
    model.add_boundary_condition("bc_x", BoundaryCondition::Symmetry { target: Target::Set("sym_x".into()), plane: Symmetry::X }).unwrap();
    model.add_boundary_condition("bc_y", BoundaryCondition::Symmetry { target: Target::Set("ligament".into()), plane: Symmetry::Y }).unwrap();

    let p = 1.0e8_f64;
    if use_traction {
        model.add_load("cface", Load::Traction { set: "crack_faces".into(), vector: [0.0, p, 0.0] }).unwrap();
    } else {
        model.add_load("cface", Load::Pressure { set: "crack_faces".into(), magnitude: p }).unwrap();
    }

    let mut step = Step::new("Load", Procedure::Static {});
    step.boundary_conditions.insert("bc_z".into());
    step.boundary_conditions.insert("bc_x".into());
    step.boundary_conditions.insert("bc_y".into());
    step.loads.insert("cface".into());
    model.add_step(step, None).unwrap();
    model
}

/// Build the 1/8-symmetry extruded CCT plate (`a = 0.025 m`, `W = 0.10 m`, `H = 0.20 m`).
fn build_cct_plate(constrain_z_top: bool, half_thickness: f64, plastic: Option<Plastic>, n_increments: Option<usize>, sigma: f64) -> Model {
    let a = 0.025_f64;
    let w = 0.10_f64;
    let h = 0.20_f64;
    let d_box = 0.45 * a;
    let m2d = build_2d_crack_mesh(a, w, h, d_box, 3, 8, 8);
    let nz = 2usize;

    let mut model = Model::default();
    let mut layer_labels: Vec<Vec<u32>> = vec![vec![0; m2d.uv.len()]; nz + 1];
    let mut next_nid = 1u32;
    for (k, row) in layer_labels.iter_mut().enumerate() {
        let z = half_thickness * (k as f64) / (nz as f64);
        for (i2, &[x, y]) in m2d.uv.iter().enumerate() {
            let lbl = next_nid;
            next_nid += 1;
            model.set_node(lbl, [x, y, z]).unwrap();
            row[i2] = lbl;
        }
    }

    let mut all_elems = Vec::new();
    let mut top_faces = Vec::new();
    let mut next_eid = 1u32;
    for k in 0..nz {
        for e2 in &m2d.elems {
            match *e2 {
                Elem2D::Tri { nodes: [a0, a1, a2], .. } => {
                    let conn = vec![
                        layer_labels[k][a0],
                        layer_labels[k][a1],
                        layer_labels[k][a2],
                        layer_labels[k + 1][a0],
                        layer_labels[k + 1][a1],
                        layer_labels[k + 1][a2],
                    ];
                    model.set_element(next_eid, ElementKind::Wedge6, conn).unwrap();
                    all_elems.push(next_eid);
                    next_eid += 1;
                }
                Elem2D::Quad { nodes: [q0, q1, q2, q3], top_edge, .. } => {
                    let conn = vec![
                        layer_labels[k][q0],
                        layer_labels[k][q1],
                        layer_labels[k][q2],
                        layer_labels[k][q3],
                        layer_labels[k + 1][q0],
                        layer_labels[k + 1][q1],
                        layer_labels[k + 1][q2],
                        layer_labels[k + 1][q3],
                    ];
                    model.set_element(next_eid, ElementKind::Hex8, conn).unwrap();
                    all_elems.push(next_eid);
                    if let Some(edge) = top_edge {
                        top_faces.push(ElementFace { element: next_eid, face: extruded_hex_face(edge) });
                    }
                    next_eid += 1;
                }
            }
        }
    }

    let mut sym_x = BTreeSet::new();
    let mut sym_y = BTreeSet::new();
    let mut sym_z = BTreeSet::new();
    let mut top_z = BTreeSet::new();
    let mut front = Vec::with_capacity(nz + 1);
    for layer in &layer_labels {
        front.push(layer[m2d.tip_idx]);
        for &i2 in &m2d.u_zero_nodes {
            sym_x.insert(layer[i2]);
        }
        for &i2 in &m2d.ligament_nodes {
            sym_y.insert(layer[i2]);
        }
    }
    for &lbl in &layer_labels[0] {
        sym_z.insert(lbl);
    }
    for &lbl in &layer_labels[nz] {
        top_z.insert(lbl);
    }

    model
        .add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: E_STEEL, poisson: NU_STEEL }), plastic, ..Default::default() })
        .unwrap();
    model.add_set("all", Set { elements: all_elems, ..Set::default() }).unwrap();
    model.add_set("sym_x", Set { nodes: sym_x.into_iter().collect(), ..Set::default() }).unwrap();
    model.add_set("ligament", Set { nodes: sym_y.into_iter().collect(), ..Set::default() }).unwrap();
    model.add_set("sym_z", Set { nodes: sym_z.into_iter().collect(), ..Set::default() }).unwrap();
    model.add_set("top_z", Set { nodes: top_z.into_iter().collect(), ..Set::default() }).unwrap();
    model.add_set("crack_front", Set { nodes: front, ..Set::default() }).unwrap();
    model.add_set("top_faces", Set { faces: top_faces, ..Set::default() }).unwrap();
    model
        .add_section("solid", Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() })
        .unwrap();

    model.add_boundary_condition("bc_x", BoundaryCondition::Symmetry { target: Target::Set("sym_x".into()), plane: Symmetry::X }).unwrap();
    model.add_boundary_condition("bc_y", BoundaryCondition::Symmetry { target: Target::Set("ligament".into()), plane: Symmetry::Y }).unwrap();
    model.add_boundary_condition("bc_z", BoundaryCondition::Symmetry { target: Target::Set("sym_z".into()), plane: Symmetry::Z }).unwrap();
    if constrain_z_top {
        model
            .add_boundary_condition(
                "bc_top_z",
                BoundaryCondition::Displacement { target: Target::Set("top_z".into()), values: [None, None, Some(0.0), None, None, None] },
            )
            .unwrap();
    }

    model.add_load("tension", Load::Pressure { set: "top_faces".into(), magnitude: -sigma }).unwrap();
    let proc = match n_increments {
        Some(n) => Procedure::NonlinearStatic(NonlinearStatic {
            nonlinear_geometry: false,
            initial_increment: 1.0 / (n as f64),
            max_increments: n as u32,
            ..Default::default()
        }),
        None => Procedure::Static {},
    };
    let mut step = Step::new("Load", proc);
    step.boundary_conditions.insert("bc_x".into());
    step.boundary_conditions.insert("bc_y".into());
    step.boundary_conditions.insert("bc_z".into());
    if constrain_z_top {
        step.boundary_conditions.insert("bc_top_z".into());
    }
    step.loads.insert("tension".into());
    model.add_step(step, None).unwrap();
    model
}

/// Build the 1/4-symmetry 3D semi-elliptical surface crack in a finite plate:
/// `a = 0.012 m`, `c = 0.020 m` (`a/c = 0.6`), `t = 0.060 m` (`a/t = 0.2`), `b = 0.200 m`, `h = 0.200 m`.
fn build_semi_elliptical_surface_crack() -> (Model, Vec<f64>) {
    let a = 0.012_f64;
    let c = 0.020_f64;
    let t = 0.060_f64;
    let b = 0.200_f64;
    let h = 0.200_f64;

    // 2D template parameterized on normalized advance `u in [0, 2.0]` (tip at `u_tip = 1.0`,
    // rosette box `d_box = 0.45`, outer wall at `u_max = 2.0`) and `v in [0, h]`.
    let u_tip = 1.0_f64;
    let d_box = 0.45_f64;
    let u_max = 2.0_f64;
    // Note: in `build_2d_crack_mesh`, `v_max = h` while `d_box = 0.45` is normalized along `u`,
    // so we scale `v` inside the rosette separately when mapping `(u, v) -> (x, y, z)`.
    let m2d = build_2d_crack_mesh(u_tip, u_max, 1.0, d_box, 3, 8, 8);

    let n_phi = 16usize;
    let half_phi = n_phi / 2;
    let phis: Vec<f64> = (0..=n_phi).map(|k| (PI * 0.5) * (k as f64) / (n_phi as f64)).collect();

    // Physical radial thickness of the normalized `[0.55..1.45]` rosette box in the `(x, z)` plane
    // is `s_ros = 0.35 * a = 0.0042 m`, and vertical `y`-height of the rosette box is `0.0042 m`.
    let s_ros = 0.35 * a;
    let mut model = Model::default();

    let mut axis_labels: Vec<Option<u32>> = vec![None; m2d.uv.len()];
    let mut slice_labels: Vec<Vec<u32>> = vec![vec![0; m2d.uv.len()]; n_phi + 1];
    let mut next_nid = 1u32;

    let map_xz = |u: f64, k: usize| -> (f64, f64) {
        let phi = phis[k];
        let xf = c * phi.cos();
        let zf = a * phi.sin();
        // Outward ellipse normal in (x, z)
        let nx = a * phi.cos();
        let nz = c * phi.sin();
        let norm = (nx * nx + nz * nz).sqrt();
        let (ux, uz) = (nx / norm, nz / norm);

        // Outer plate perimeter `(x_out, z_out)` matching corner `(b, t)` at `k = half_phi`.
        let (x_out, z_out) = if k <= half_phi {
            let frac = (k as f64) / (half_phi as f64);
            (b, t * frac)
        } else {
            let frac = ((n_phi - k) as f64) / (half_phi as f64);
            (b * frac, t)
        };

        let (mut x, mut z) = if u <= u_tip - d_box {
            // Crack face interior: `(0, 0)` at `u = 0` to `p_box_inner` at `u = u_tip - d_box`.
            let xi = u / (u_tip - d_box);
            let x_in = xf - s_ros * ux;
            let z_in = zf - s_ros * uz;
            (xi * x_in, xi * z_in)
        } else if u <= u_tip + d_box {
            // Inside rosette box: purely along orthogonal ellipse normal `u_vec`.
            let s = (u - u_tip) / d_box; // in [-1, 1]
            (xf + s * s_ros * ux, zf + s * s_ros * uz)
        } else {
            // Outer ligament: linear blend from `p_box_outer` at `u_tip + d_box` to `(x_out, z_out)` at `u_max`.
            let xi = (u - (u_tip + d_box)) / (u_max - (u_tip + d_box));
            let x_bo = xf + s_ros * ux;
            let z_bo = zf + s_ros * uz;
            (x_bo + xi * (x_out - x_bo), z_bo + xi * (z_out - z_bo))
        };

        if k == 0 {
            z = 0.0;
        }
        if k == n_phi {
            x = 0.0;
        }
        (x, z)
    };

    let map_y = |v_norm: f64| -> f64 {
        if v_norm <= d_box {
            (v_norm / d_box) * s_ros
        } else {
            let xi = (v_norm - d_box) / (1.0 - d_box);
            s_ros + xi * (h - s_ros)
        }
    };

    for (i2, &[_, v_norm]) in m2d.uv.iter().enumerate() {
        if m2d.u_zero_nodes.contains(&i2) {
            let lbl = next_nid;
            next_nid += 1;
            model.set_node(lbl, [0.0, map_y(v_norm), 0.0]).unwrap();
            axis_labels[i2] = Some(lbl);
            for row in &mut slice_labels {
                row[i2] = lbl;
            }
        }
    }
    for (k, row) in slice_labels.iter_mut().enumerate() {
        for (i2, &[u, v_norm]) in m2d.uv.iter().enumerate() {
            if axis_labels[i2].is_some() {
                continue;
            }
            let lbl = next_nid;
            next_nid += 1;
            let (x, z) = map_xz(u, k);
            let y = map_y(v_norm);
            model.set_node(lbl, [x, y, z]).unwrap();
            row[i2] = lbl;
        }
    }

    let mut all_elems = Vec::new();
    let mut top_faces = Vec::new();
    let mut next_eid = 1u32;
    for k in 0..n_phi {
        for e2 in &m2d.elems {
            match *e2 {
                Elem2D::Tri { nodes: [a0, a1, a2], .. } => {
                    let conn = vec![
                        slice_labels[k][a0],
                        slice_labels[k][a1],
                        slice_labels[k][a2],
                        slice_labels[k + 1][a0],
                        slice_labels[k + 1][a1],
                        slice_labels[k + 1][a2],
                    ];
                    model.set_element(next_eid, ElementKind::Wedge6, conn).unwrap();
                    all_elems.push(next_eid);
                    next_eid += 1;
                }
                Elem2D::Quad { nodes: [q0, q1, q2, q3], top_edge, .. } => {
                    let on_axis_0 = axis_labels[q0].is_some();
                    let on_axis_3 = axis_labels[q3].is_some();
                    if on_axis_0 && on_axis_3 {
                        let a0 = axis_labels[q0].unwrap();
                        let b0 = axis_labels[q3].unwrap();
                        let a1_k = slice_labels[k][q1];
                        let a1_kp = slice_labels[k + 1][q1];
                        let b1_k = slice_labels[k][q2];
                        let b1_kp = slice_labels[k + 1][q2];
                        model.set_element(next_eid, ElementKind::Wedge6, vec![a0, a1_kp, a1_k, b0, b1_kp, b1_k]).unwrap();
                        all_elems.push(next_eid);
                        if top_edge.is_some() {
                            // Top cap (face 2 = [3, 4, 5]) lies on y = h top wall!
                            top_faces.push(ElementFace { element: next_eid, face: 2 });
                        }
                        next_eid += 1;
                    } else {
                        let conn = vec![
                            slice_labels[k][q0],
                            slice_labels[k][q1],
                            slice_labels[k][q2],
                            slice_labels[k][q3],
                            slice_labels[k + 1][q0],
                            slice_labels[k + 1][q1],
                            slice_labels[k + 1][q2],
                            slice_labels[k + 1][q3],
                        ];
                        model.set_element(next_eid, ElementKind::Hex8, conn).unwrap();
                        all_elems.push(next_eid);
                        if let Some(edge) = top_edge {
                            top_faces.push(ElementFace { element: next_eid, face: extruded_hex_face(edge) });
                        }
                        next_eid += 1;
                    }
                }
            }
        }
    }

    let mut sym_x = BTreeSet::new();
    let mut ligament = BTreeSet::new();
    let mut front = Vec::with_capacity(n_phi + 1);
    for slice in &slice_labels {
        front.push(slice[m2d.tip_idx]);
        for &i2 in &m2d.ligament_nodes {
            ligament.insert(slice[i2]);
        }
    }
    for &lbl in &slice_labels[n_phi] {
        sym_x.insert(lbl);
    }

    // Pin z = 0 on one far corner `(b, h, t)` to prevent rigid-body Z translation.
    let corner_pin = slice_labels[half_phi]
        .iter()
        .copied()
        .find(|&lbl| {
            let p = model.node(lbl).unwrap().xyz;
            (p[0] - b).abs() < 1e-9 && (p[1] - h).abs() < 1e-9 && (p[2] - t).abs() < 1e-9
        })
        .expect("far corner (b, h, t) node");

    model.add_material("Steel", Material { elastic: Some(Elastic::Isotropic { young: E_STEEL, poisson: NU_STEEL }), ..Default::default() }).unwrap();
    model.add_set("all", Set { elements: all_elems, ..Set::default() }).unwrap();
    model.add_set("sym_x", Set { nodes: sym_x.into_iter().collect(), ..Set::default() }).unwrap();
    model.add_set("ligament", Set { nodes: ligament.into_iter().collect(), ..Set::default() }).unwrap();
    model.add_set("corner_z", Set { nodes: vec![corner_pin], ..Set::default() }).unwrap();
    model.add_set("crack_front", Set { nodes: front, ..Set::default() }).unwrap();
    model.add_set("top_faces", Set { faces: top_faces, ..Set::default() }).unwrap();
    model
        .add_section("solid", Section { set: "all".into(), material: "Steel".into(), kind: SectionKind::Solid {}, formulation: Default::default() })
        .unwrap();

    model.add_boundary_condition("bc_x", BoundaryCondition::Symmetry { target: Target::Set("sym_x".into()), plane: Symmetry::X }).unwrap();
    model.add_boundary_condition("bc_y", BoundaryCondition::Symmetry { target: Target::Set("ligament".into()), plane: Symmetry::Y }).unwrap();
    model
        .add_boundary_condition(
            "bc_z_pin",
            BoundaryCondition::Displacement { target: Target::Set("corner_z".into()), values: [None, None, Some(0.0), None, None, None] },
        )
        .unwrap();

    let sigma = 1.0e8_f64;
    model.add_load("tension", Load::Pressure { set: "top_faces".into(), magnitude: -sigma }).unwrap();

    let mut step = Step::new("Load", Procedure::Static {});
    step.boundary_conditions.insert("bc_x".into());
    step.boundary_conditions.insert("bc_y".into());
    step.boundary_conditions.insert("bc_z_pin".into());
    step.loads.insert("tension".into());
    model.add_step(step, None).unwrap();
    (model, phis)
}

// ---------------------------------------------------------------------------
// Element Jacobian, face normal, and live runner helpers
// ---------------------------------------------------------------------------

fn triple(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    let cx = b[1] * c[2] - b[2] * c[1];
    let cy = b[2] * c[0] - b[0] * c[2];
    let cz = b[0] * c[1] - b[1] * c[0];
    a[0] * cx + a[1] * cy + a[2] * cz
}

fn sub(p: [f64; 3], q: [f64; 3]) -> [f64; 3] {
    [p[0] - q[0], p[1] - q[1], p[2] - q[2]]
}

fn assert_all_positive_corner_jacobians(model: &Model, label: &str) {
    assert!(!model.elements().is_empty(), "{label}: empty mesh");
    for (&eid, elem) in model.elements() {
        let p = |i: usize| model.node(elem.nodes[i]).unwrap().xyz;
        match elem.kind {
            ElementKind::Hex8 => {
                // Standard right-handed triples `(e_xi, e_eta, e_zeta)` at all 8 Hex8 corners.
                let corners = [(0, 1, 3, 4), (1, 2, 0, 5), (2, 3, 1, 6), (3, 0, 2, 7), (4, 7, 5, 0), (5, 4, 6, 1), (6, 5, 7, 2), (7, 6, 4, 3)];
                for &(c, i1, i2, i3) in &corners {
                    let det = triple(sub(p(i1), p(c)), sub(p(i2), p(c)), sub(p(i3), p(c)));
                    assert!(det > 0.0, "{label} Hex8 {eid} corner {c} has non-positive Jacobian {det}");
                }
            }
            ElementKind::Wedge6 => {
                // Right-handed triples `(e1, e2, e_axial)` at all 6 Wedge6 corners.
                let corners = [(0, 1, 2, 3), (1, 2, 0, 4), (2, 0, 1, 5), (3, 5, 4, 0), (4, 3, 5, 1), (5, 4, 3, 2)];
                for &(c, i1, i2, i3) in &corners {
                    let det = triple(sub(p(i1), p(c)), sub(p(i2), p(c)), sub(p(i3), p(c)));
                    assert!(det > 0.0, "{label} Wedge6 {eid} corner {c} has non-positive Jacobian {det}");
                }
            }
            other => panic!("{label}: unexpected element kind {other:?}"),
        }
    }
}

fn assert_face_normals_point_along(model: &Model, set_name: &str, expected_dir: [f64; 3], label: &str) {
    let rset = model.resolve_set_partial(set_name).unwrap();
    assert!(!rset.faces.is_empty(), "{label}: empty face set `{set_name}`");
    for ef in &rset.faces {
        let elem = model.element(ef.element).unwrap();
        let face_local = elem.kind.faces()[(ef.face as usize) - 1];
        let p0 = model.node(elem.nodes[face_local[0]]).unwrap().xyz;
        let p1 = model.node(elem.nodes[face_local[1]]).unwrap().xyz;
        let p2 = model.node(elem.nodes[face_local[2]]).unwrap().xyz;
        let e1 = sub(p1, p0);
        let e2 = sub(p2, p0);
        let n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
        let norm = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        assert!(norm > 0.0, "{label}: degenerate face {:?}", ef);
        let dot = (n[0] * expected_dir[0] + n[1] * expected_dir[1] + n[2] * expected_dir[2]) / norm;
        assert!(dot > 0.99, "{label}: face {:?} normal {:?} misaligned with {:?}", ef, n, expected_dir);
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn run_warp3d_or_skip(model: &Model, step: &str, crack: &CrackFrontSpec, threads: usize) -> Option<Warp3dSolution> {
    let cancel = AtomicBool::new(false);
    let require_warp3d = std::env::var_os("FIELDCRAFT_REQUIRE_WARP3D").is_some();
    match run_warp3d(model, step, Some(crack), threads, None, &cancel) {
        Ok(sol) => Some(sol),
        Err(Warp3dError::BinaryNotFound(msg)) if !require_warp3d => {
            eprintln!("fracture_benchmarks: skipping cleanly ({msg})");
            None
        }
        Err(Warp3dError::SolverFailed { ref log_tail, .. }) if log_tail.contains("libimf.so") => {
            let win_bin =
                PathBuf::from(std::env::var_os("WARP3D_HOME").expect("WARP3D_HOME for libimf fallback")).join("run_windows").join("warp3d.exe");
            Some(run_warp3d_with_override(model, step, Some(crack), threads, None, &cancel, Some(&win_bin)).expect("WARP3D Windows fallback"))
        }
        Err(e) => panic!("WARP3D fracture benchmark failed: {e}"),
    }
}

// ---------------------------------------------------------------------------
// Trunk tests (no WARP3D executable required)
// ---------------------------------------------------------------------------

#[test]
fn procedural_meshes_have_positive_jacobians_and_pass_preflight() {
    let sneddon_p = build_sneddon_penny_crack(false);
    let sneddon_t = build_sneddon_penny_crack(true);
    let cct_pstrn = build_cct_plate(true, 0.025, None, None, 1.0e8);
    let cct_pstrs = build_cct_plate(false, 0.0005, None, None, 1.0e8);
    let cct_epfm = build_cct_plate(
        true,
        0.025,
        Some(Plastic::Bilinear { yield_stress: 250.0e6, hardening_modulus: 2.0e9, hardening: Hardening::Isotropic }),
        Some(4),
        1.8e8,
    );
    let (surf, _) = build_semi_elliptical_surface_crack();

    assert_all_positive_corner_jacobians(&sneddon_p, "sneddon_pressure");
    assert_all_positive_corner_jacobians(&sneddon_t, "sneddon_traction");
    assert_all_positive_corner_jacobians(&cct_pstrn, "cct_plane_strain");
    assert_all_positive_corner_jacobians(&cct_pstrs, "cct_plane_stress");
    assert_all_positive_corner_jacobians(&cct_epfm, "cct_epfm");
    assert_all_positive_corner_jacobians(&surf, "semi_elliptical");

    assert_face_normals_point_along(&sneddon_p, "crack_faces", [0.0, -1.0, 0.0], "sneddon_crack_faces");
    assert_face_normals_point_along(&cct_pstrn, "top_faces", [0.0, 1.0, 0.0], "cct_top_faces");
    assert_face_normals_point_along(&surf, "top_faces", [0.0, 1.0, 0.0], "surf_top_faces");

    let crack_ii = CrackFrontSpec {
        set: "crack_front".into(),
        normal: None,
        symmetric: None,
        rings: Some(6),
        ring_start: None,
        interaction_integral: Some(true),
        interaction_stress_state: None,
        function_type: CrackFunctionType::Pointwise,
        every_step: None,
        increment_interval: None,
    };

    for (name, m) in [("sneddon_p", &sneddon_p), ("cct_pstrn", &cct_pstrn), ("surf", &surf)] {
        let rep = validate_model(m, Some("Load"), Some(&crack_ii), None, CheckScope::Solve { want_nonlinear: false });
        assert!(rep.ready && rep.errors == 0, "{name}: {rep:?}");
    }

    let rep_trac = validate_model(&sneddon_t, Some("Load"), Some(&crack_ii), None, CheckScope::Solve { want_nonlinear: false });
    assert!(rep_trac.ready && rep_trac.errors == 0, "{rep_trac:?}");
    assert!(
        rep_trac.diagnostics.iter().any(|d| d.code == "warp3d.interaction_crack_face_load"),
        "expected Method-1 II8 traction warning, got {:?}",
        rep_trac.diagnostics
    );

    let crack_pstrs = CrackFrontSpec { interaction_stress_state: Some(InteractionStressState::PlaneStress), ..crack_ii.clone() };
    let rep_pstrs = validate_model(&cct_pstrs, Some("Load"), Some(&crack_pstrs), None, CheckScope::Solve { want_nonlinear: false });
    assert!(rep_pstrs.ready && rep_pstrs.errors == 0, "{rep_pstrs:?}");

    let crack_epfm = CrackFrontSpec {
        set: "crack_front".into(),
        normal: None,
        symmetric: None,
        rings: Some(7),
        ring_start: Some(3),
        interaction_integral: Some(false),
        interaction_stress_state: None,
        function_type: CrackFunctionType::Pointwise,
        every_step: None,
        increment_interval: Some(2),
    };
    let rep_epfm = validate_model(&cct_epfm, Some("Load"), Some(&crack_epfm), None, CheckScope::Solve { want_nonlinear: true });
    assert!(rep_epfm.ready && rep_epfm.errors == 0, "{rep_epfm:?}");
}

#[test]
fn benchmark_decks_verify_syntax_line_limits_and_all_unit_presets() {
    let sneddon_t = build_sneddon_penny_crack(true);
    let cct_pstrs = build_cct_plate(false, 0.0005, None, None, 1.0e8);
    let cct_epfm = build_cct_plate(
        true,
        0.025,
        Some(Plastic::Bilinear { yield_stress: 250.0e6, hardening_modulus: 2.0e9, hardening: Hardening::Isotropic }),
        Some(4),
        1.8e8,
    );

    let crack_sned = CrackFrontSpec {
        set: "crack_front".into(),
        normal: None,
        symmetric: None,
        rings: Some(6),
        ring_start: None,
        interaction_integral: Some(true),
        interaction_stress_state: None,
        function_type: CrackFunctionType::Pointwise,
        every_step: None,
        increment_interval: None,
    };
    for preset in Preset::ALL {
        let deck = write_deck(&sneddon_t, Some("Load"), Some(&crack_sned), preset).unwrap();
        assert!(deck.text.contains("type l3disop linear material Steel"));
        assert!(deck.text.contains("tractions tx"));
        assert!(deck.text.contains("q-values automatic rings 1-6"));
        assert!(deck.text.contains("plane strain output for interaction integral"));
        for line in deck.text.lines() {
            if !line.starts_with("! ") && line != "!" {
                assert!(line.len() <= 72, "preset {preset:?} line exceeded 72 chars ({}): {line}", line.len());
            }
        }
    }

    let crack_pstrs = CrackFrontSpec { interaction_stress_state: Some(InteractionStressState::PlaneStress), ..crack_sned };
    let deck_pstrs = write_deck(&cct_pstrs, Some("Load"), Some(&crack_pstrs), Preset::MmTonneS).unwrap();
    assert!(deck_pstrs.text.contains("plane stress output for interaction integral"));

    let crack_epfm = CrackFrontSpec {
        set: "crack_front".into(),
        normal: None,
        symmetric: None,
        rings: Some(7),
        ring_start: Some(3),
        interaction_integral: Some(false),
        interaction_stress_state: None,
        function_type: CrackFunctionType::Pointwise,
        every_step: None,
        increment_interval: Some(2),
    };
    let deck_epfm = write_deck(&cct_epfm, Some("Load"), Some(&crack_epfm), Preset::MKgS).unwrap();
    assert!(deck_epfm.text.contains("q-values automatic rings 3-7"));
    // Increments 1..=4 with interval 2 anchors {1, 2, 4} -> 3 domain blocks per station (9 stations * 3? Wait: 3 stations * 3 increments = 9).
    let n_stations = cct_epfm.resolve_set_partial("crack_front").unwrap().nodes.len();
    assert_eq!(deck_epfm.text.matches("compute domain integral").count(), 3 * n_stations);
}

#[test]
fn api579_part9_option1_and_option2_analytical_reference_checks() {
    // Option 1: K_r(L_r) = (1 - 0.14 L_r^2)(0.3 + 0.7 exp(-0.65 L_r^6)) for L_r <= L_r,max.
    let lr_max = 1.1875;
    for lr in [0.0_f64, 0.2, 0.5, 0.8, 1.0, 1.15, 1.1875] {
        let expected = (1.0 - 0.14 * lr * lr) * (0.3 + 0.7 * (-0.65 * lr.powi(6)).exp());
        let got = evaluate_fad_curve(FadOption::Option1, lr, lr_max, None);
        assert!((got - expected).abs() / expected <= 1e-12, "Option 1 lr={lr}: got {got}, expected {expected}");
    }
    assert_eq!(evaluate_fad_curve(FadOption::Option1, 1.19, lr_max, None), 0.0);

    // Option 2 (Ramberg-Osgood): K_r(L_r) = (E*eps_ref/sigma_ref + 0.5*L_r^3*sigma_ys/(E*eps_ref))^(-1/2).
    let young = 2.0e11;
    let sigma_ys = 400.0e6;
    let alpha = 1.0;
    let n = 5.0;
    let eval_ro = Option2Evaluator::RambergOsgood { young, sigma_ys, alpha, n };
    for lr in [0.1_f64, 0.4, 0.8, 1.0, 1.15] {
        let ratio = 1.0 + alpha * lr.powf(n - 1.0);
        let expected = (ratio + (0.5 * lr * lr) / ratio).powf(-0.5);
        let got = evaluate_fad_curve(FadOption::Option2, lr, lr_max, Some(&eval_ro));
        assert!((got - expected).abs() / expected <= 1e-12, "Option 2 RO lr={lr}: got {got}, expected {expected}");
    }

    // Reserve factor consistency across Fracture, ElasticPlastic, and PlasticCollapse regimes.
    let curve = |lr: f64| evaluate_fad_curve(FadOption::Option1, lr, lr_max, None);
    for (lr, kr) in [(0.08, 0.70), (0.55, 0.65), (1.05, 0.15), (1.30, 0.20)] {
        let (rf, sm, ok, _, inter) = reserve_factor(lr, kr, lr_max, curve);
        let rf = rf.unwrap();
        assert!((sm.unwrap() - (rf - 1.0)).abs() <= 1e-12);
        assert_eq!(ok, rf >= 1.0);
        let [ix, iy] = inter.unwrap();
        assert!((ix - rf * lr).abs() <= 1e-12);
        assert!((iy - rf * kr).abs() <= 1e-12);
    }
}

#[test]
fn published_reference_formulas_spot_checks() {
    // Sneddon (1946) penny crack: p = 100 MPa, a = 0.01 m -> 2/pi * 100 * sqrt(pi * 0.01) = 11.28379167 MPa*sqrt(m).
    let ki_sned = sneddon_penny_ki(1.0e8, 0.01);
    assert!((ki_sned - 11_283_791.670955126).abs() / 11_283_791.670955126 <= 1e-10);

    // Wang (2004) penny crack T11: p = 100 MPa, nu = 0.3 -> -0.8 * 100 MPa = -80 MPa.
    let t_sned = wang_2004_penny_t11(1.0e8, 0.3);
    assert!((t_sned - (-80.0e6)).abs() <= 1e-6);

    // Feddersen CCT: sigma = 100 MPa, a = 0.025 m, W = 0.10 m -> sqrt(sec(pi/8)) = 1.0403812 -> 29.156092 MPa*sqrt(m).
    let ki_cct = feddersen_cct_ki(1.0e8, 0.025, 0.10);
    let expected_cct = 1.0e8 * (PI * 0.025).sqrt() * (1.0 / (PI / 8.0).cos()).sqrt();
    assert!((ki_cct - expected_cct).abs() / expected_cct <= 1e-12);

    // Newman & Raju (1981): a=0.012, c=0.020, t=0.060, b=0.200, sigma=100 MPa at deepest point phi=pi/2 -> ~16.72 MPa*sqrt(m).
    let ki_nr = newman_raju_1981_ki(1.0e8, 0.012, 0.020, 0.060, 0.200, PI * 0.5);
    assert!(ki_nr > 16.5e6 && ki_nr < 17.0e6, "ki_nr={ki_nr}");
}

// ---------------------------------------------------------------------------
// Heavy live-WARP3D verification benchmarks
// ---------------------------------------------------------------------------

#[cfg(not(target_arch = "wasm32"))]
#[test]
#[ignore = "heavy: cargo xtask test --heavy (CI heavy job)"]
fn live_sneddon_pressurized_penny_crack() {
    let a = 0.01_f64;
    let p = 1.0e8_f64;
    let kj_ref = sneddon_penny_ki(p, a);
    let t11_wang = wang_2004_penny_t11(p, NU_STEEL);

    let crack = CrackFrontSpec {
        set: "crack_front".into(),
        normal: None,
        symmetric: None,
        rings: Some(6),
        ring_start: None,
        interaction_integral: Some(true),
        interaction_stress_state: None,
        function_type: CrackFunctionType::Pointwise,
        every_step: None,
        increment_interval: None,
    };

    let m_press = build_sneddon_penny_crack(false);
    let Some(sol_press) = run_warp3d_or_skip(&m_press, "Load", &crack, 2) else {
        return;
    };
    let m_trac = build_sneddon_penny_crack(true);
    let sol_trac = run_warp3d_or_skip(&m_trac, "Load", &crack, 2).expect("WARP3D traction run");

    let (prof_p, spread_p, _, _) = build_station_profiles(&m_press, &sol_press.fracture, RingExclusion::Auto, None);
    let (prof_t, spread_t, _, _) = build_station_profiles(&m_trac, &sol_trac.fracture, RingExclusion::Auto, None);
    assert_eq!(prof_p.len(), 17);
    assert_eq!(prof_t.len(), 17);
    println!("Sneddon ref: K_J = {:.4} MPa*sqrt(m), Wang T11 = {:.4} MPa", kj_ref / 1e6, t11_wang / 1e6);
    println!("Sneddon max outer-ring spread: pressure = {:.4}%, traction = {:.4}%", spread_p * 100.0, spread_t * 100.0);
    if let Some(p0) = prof_p.first() {
        let rings_p: Vec<(usize, f64)> = p0.rings.iter().map(|r| (r.ring, r.j)).collect();
        println!("Sneddon cf_0001 pressure rings (ring, J): {rings_p:?}");
    }

    for (pp, pt) in prof_p.iter().zip(prof_t.iter()) {
        let kj_p = pp.k_j_plane_strain.unwrap();
        let kj_t = pt.k_j_plane_strain.unwrap();
        let err_p = (kj_p - kj_ref).abs() / kj_ref;
        let err_t = (kj_t - kj_ref).abs() / kj_ref;
        let ki_p = pp.k_i_mean.unwrap();
        let ki_t = pt.k_i_mean.unwrap();
        println!(
            "  {} (s={:.3}): K_J(p)={:.4} (err={:.2}%, spr={:.2}%), K_J(t)={:.4} (err={:.2}%, spr={:.2}%), K_I(p)={:.4}, K_I(t)={:.4}, T11(p)={:?}, T11(t)={:?}",
            pp.domain_id,
            pp.s_norm,
            kj_p / 1e6,
            err_p * 100.0,
            pp.ring_spread * 100.0,
            kj_t / 1e6,
            err_t * 100.0,
            pt.ring_spread * 100.0,
            ki_p / 1e6,
            ki_t / 1e6,
            pp.t_11_mean.map(|v| v / 1e6),
            pt.t_11_mean.map(|v| v / 1e6),
        );
    }
    for (pp, pt) in prof_p.iter().zip(prof_t.iter()) {
        assert_eq!(pp.excluded_rings, vec![1]);
        assert_eq!(pp.kept_rings, 5);
        assert!(pp.ring_spread <= 0.02, "pressure ring spread {} > 2% on {}", pp.ring_spread, pp.domain_id);
        assert!(pt.ring_spread <= 0.02, "traction ring spread {} > 2% on {}", pt.ring_spread, pt.domain_id);

        let kj_p = pp.k_j_plane_strain.unwrap();
        let kj_t = pt.k_j_plane_strain.unwrap();
        let err_p = (kj_p - kj_ref).abs() / kj_ref;
        let err_t = (kj_t - kj_ref).abs() / kj_ref;
        assert!(err_p <= 0.03, "pressure K_J error {err_p} > 3% on {}", pp.domain_id);
        assert!(err_t <= 0.03, "traction K_J error {err_t} > 3% on {}", pt.domain_id);
        // J-integral matches between Load::Pressure and Load::Traction.
        assert!((pp.j_mean - pt.j_mean).abs() / pp.j_mean <= 1e-3);
    }
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
#[ignore = "heavy: cargo xtask test --heavy (CI heavy job)"]
fn live_thin_plate_plane_stress_vs_plane_strain() {
    let sigma = 1.0e8_f64;
    let ki_fedd = feddersen_cct_ki(sigma, 0.025, 0.10);

    let m_pstrs = build_cct_plate(false, 0.0005, None, None, sigma);
    let crack_pstrs = CrackFrontSpec {
        set: "crack_front".into(),
        normal: None,
        symmetric: None,
        rings: Some(7),
        ring_start: None,
        interaction_integral: Some(true),
        interaction_stress_state: Some(InteractionStressState::PlaneStress),
        function_type: CrackFunctionType::Pointwise,
        every_step: None,
        increment_interval: None,
    };
    let Some(sol_pstrs) = run_warp3d_or_skip(&m_pstrs, "Load", &crack_pstrs, 2) else {
        return;
    };

    let m_pstrn = build_cct_plate(true, 0.025, None, None, sigma);
    let crack_pstrn = CrackFrontSpec { interaction_stress_state: Some(InteractionStressState::PlaneStrain), ..crack_pstrs.clone() };
    let sol_pstrn = run_warp3d_or_skip(&m_pstrn, "Load", &crack_pstrn, 2).expect("CCT plane strain run");

    let (prof_s, _, _, _) = build_station_profiles(&m_pstrs, &sol_pstrs.fracture, RingExclusion::Auto, None);
    let (prof_n, _, _, _) = build_station_profiles(&m_pstrn, &sol_pstrn.fracture, RingExclusion::Auto, None);
    assert_eq!(prof_s.len(), 3);
    assert_eq!(prof_n.len(), 3);

    println!("Feddersen CCT ref: K_I = {:.4} MPa*sqrt(m)", ki_fedd / 1e6);
    for p in &prof_s {
        let ki = p.k_i_mean.unwrap();
        let kj = p.k_j_plane_stress.unwrap();
        let t11 = p.t_11_mean.unwrap();
        let t33 = p.t_33_mean.unwrap();
        let rel_ki_kj = (ki - kj).abs() / kj;
        let ratio_t = t33.abs() / t11.abs();
        let err_fedd = (ki - ki_fedd).abs() / ki_fedd;
        println!(
            "  PlaneStress {}: K_I={:.4}, K_J={:.4} (diff={:.2}%, Feddersen err={:.2}%), T11={:.2} MPa, |T33/T11|={:.2e}",
            p.domain_id,
            ki / 1e6,
            kj / 1e6,
            rel_ki_kj * 100.0,
            err_fedd * 100.0,
            t11 / 1e6,
            ratio_t
        );
    }
    for (idx, p) in prof_s.iter().enumerate() {
        assert_eq!(p.interaction_stress_state, Some(InteractionStressState::PlaneStress));
        let ki = p.k_i_mean.unwrap();
        let kj = p.k_j_plane_stress.unwrap();
        let t11 = p.t_11_mean.unwrap();
        let t33 = p.t_33_mean.unwrap();
        let rel_ki_kj = (ki - kj).abs() / kj;
        let ratio_t = t33.abs() / t11.abs();
        let err_fedd = (ki - ki_fedd).abs() / ki_fedd;
        assert!(ratio_t <= 1.0e-3, "|T33/T11|={ratio_t} > 1e-3 on {}", p.domain_id);
        assert!(rel_ki_kj <= 0.02, "|K_I - K_J|/K_J = {rel_ki_kj} > 2% on {}", p.domain_id);
        if idx + 1 < prof_s.len() {
            assert!(err_fedd <= 0.03, "|K_I - K_Fedd|/K_Fedd = {err_fedd} > 3% on interior {}", p.domain_id);
        }
    }
    let ki_s_trap = (0.5 * prof_s[0].k_i_mean.unwrap() + prof_s[1].k_i_mean.unwrap() + 0.5 * prof_s[2].k_i_mean.unwrap()) / 2.0;
    let err_s_trap = (ki_s_trap - ki_fedd).abs() / ki_fedd;
    println!("  PlaneStress thickness-avg K_I={:.4} (Feddersen err={:.2}%)", ki_s_trap / 1e6, err_s_trap * 100.0);
    assert!(err_s_trap <= 0.03, "PlaneStress thickness-avg Feddersen err={err_s_trap} > 3%");

    for p in &prof_n {
        let ki = p.k_i_mean.unwrap();
        let kj = p.k_j_plane_strain.unwrap();
        let t11 = p.t_11_mean.unwrap();
        let t33 = p.t_33_mean.unwrap();
        let rel_ki_kj = (ki - kj).abs() / kj;
        let err_fedd = (ki - ki_fedd).abs() / ki_fedd;
        println!(
            "  PlaneStrain {}: K_I={:.4}, K_J={:.4} (diff={:.2}%, Feddersen err={:.2}%), T11={:.2} MPa, T33={:.2} MPa (T33/(nu*T11)={:.3})",
            p.domain_id,
            ki / 1e6,
            kj / 1e6,
            rel_ki_kj * 100.0,
            err_fedd * 100.0,
            t11 / 1e6,
            t33 / 1e6,
            t33 / (NU_STEEL * t11)
        );
    }
    for p in &prof_n {
        assert_eq!(p.interaction_stress_state, Some(InteractionStressState::PlaneStrain));
        let ki = p.k_i_mean.unwrap();
        let kj = p.k_j_plane_strain.unwrap();
        let t11 = p.t_11_mean.unwrap();
        let t33 = p.t_33_mean.unwrap();
        let rel_ki_kj = (ki - kj).abs() / kj;
        let err_fedd = (ki - ki_fedd).abs() / ki_fedd;
        assert!(rel_ki_kj <= 0.02, "|K_I - K_J|/K_J = {rel_ki_kj} > 2% on {}", p.domain_id);
        assert!(err_fedd <= 0.03, "|K_I - K_Fedd|/K_Fedd = {err_fedd} > 3% on {}", p.domain_id);
        assert!((t33 / (NU_STEEL * t11) - 1.0).abs() <= 0.03);
    }
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
#[ignore = "heavy: cargo xtask test --heavy (CI heavy job)"]
fn live_semi_elliptical_surface_crack_newman_raju_and_wang() {
    let (model, phis) = build_semi_elliptical_surface_crack();
    let crack = CrackFrontSpec {
        set: "crack_front".into(),
        normal: None,
        symmetric: None,
        rings: Some(6),
        ring_start: None,
        interaction_integral: Some(true),
        interaction_stress_state: None,
        function_type: CrackFunctionType::Pointwise,
        every_step: None,
        increment_interval: None,
    };
    let Some(sol) = run_warp3d_or_skip(&model, "Load", &crack, 2) else {
        return;
    };

    let (profiles, spread, _, _) = build_station_profiles(&model, &sol.fracture, RingExclusion::Auto, None);
    assert_eq!(profiles.len(), phis.len());
    println!("Semi-elliptical surface crack (a/c=0.6, a/t=0.2): max outer-ring spread = {:.2}%", spread * 100.0);

    let sigma = 1.0e8_f64;
    for (prof, &phi) in profiles.iter().zip(phis.iter()) {
        let ki_ref = newman_raju_1981_ki(sigma, 0.012, 0.020, 0.060, 0.200, phi);
        let ki = prof.k_i_mean.unwrap();
        let kj = prof.k_j_plane_strain.unwrap();
        let t11 = prof.t_11_mean.unwrap();
        let err_ki = (ki - ki_ref).abs() / ki_ref;
        let err_kj = (kj - ki_ref).abs() / ki_ref;
        println!(
            "  {} phi={:.1}deg: K_I={:.4}, K_J={:.4}, NR81={:.4} (K_I err={:.2}%, K_J err={:.2}%), T11/sigma={:.4}, spread={:.2}%",
            prof.domain_id,
            phi.to_degrees(),
            ki / 1e6,
            kj / 1e6,
            ki_ref / 1e6,
            err_ki * 100.0,
            err_kj * 100.0,
            t11 / sigma,
            prof.ring_spread * 100.0
        );
    }
    for (idx, (prof, &phi)) in profiles.iter().zip(phis.iter()).enumerate() {
        let ki_ref = newman_raju_1981_ki(sigma, 0.012, 0.020, 0.060, 0.200, phi);
        let ki = prof.k_i_mean.unwrap();
        let kj = prof.k_j_plane_strain.unwrap();
        let t11 = prof.t_11_mean.unwrap();
        let err_ki = (ki - ki_ref).abs() / ki_ref;
        let err_kj = (kj - ki_ref).abs() / ki_ref;
        // Interior stations (phi in [5.625deg, 90deg], skipping free-surface vertex phi = 0).
        if idx >= 1 {
            assert!(err_ki <= 0.05, "Newman-Raju K_I error {err_ki} > 5% at phi={}deg", phi.to_degrees());
            assert!(err_kj <= 0.05, "Newman-Raju K_J error {err_kj} > 5% at phi={}deg", phi.to_degrees());
            assert!(t11 < 0.0, "expected compressive shallow-crack T11 at phi={}deg, got {t11}", phi.to_degrees());
        }
    }
    // Deepest point (idx = phis.len() - 1, phi = 90 deg) shallow-crack (a/t=0.2, a/c=0.6) T11/sigma
    // matches Wang (2003, Eng. Fract. Mech. 70: 731-756, published T11/sigma ~= -0.54) within <= 10%.
    let deepest_t11_norm = profiles.last().unwrap().t_11_mean.unwrap() / sigma;
    let wang_deepest_ref = -0.54_f64;
    let wang_err = (deepest_t11_norm - wang_deepest_ref).abs() / wang_deepest_ref.abs();
    println!("  Deepest-point T11/sigma = {deepest_t11_norm:.4} (Wang 2003 ref {wang_deepest_ref:.2}, err={:.2}%)", wang_err * 100.0);
    assert!(wang_err <= 0.10, "deepest T11/sigma={deepest_t11_norm} differs from Wang {wang_deepest_ref} by {wang_err}");
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
#[ignore = "heavy: cargo xtask test --heavy (CI heavy job)"]
fn live_nonlinear_epfm_sparse_rings_step1_guard_and_api579_fad() {
    let sigma = 1.8e8_f64;
    let model = build_cct_plate(
        true,
        0.025,
        Some(Plastic::Bilinear { yield_stress: 250.0e6, hardening_modulus: 2.0e9, hardening: Hardening::Isotropic }),
        Some(4),
        sigma,
    );
    let crack = CrackFrontSpec {
        set: "crack_front".into(),
        normal: None,
        symmetric: None,
        rings: Some(7),
        ring_start: Some(3),
        interaction_integral: Some(false),
        interaction_stress_state: None,
        function_type: CrackFunctionType::Pointwise,
        every_step: None,
        increment_interval: Some(2),
    };
    let Some(sol) = run_warp3d_or_skip(&model, "Load", &crack, 2) else {
        return;
    };

    // Increment 1 yields crack-tip Gauss points -> step1_is_elastic == Some(false).
    assert_eq!(sol.step1_is_elastic, Some(false), "expected Increment-1 plastic yielding");
    let eval_warp_steps: BTreeSet<usize> = sol.fracture.iter().map(|d| d.step).collect();
    assert_eq!(eval_warp_steps, BTreeSet::from([1, 2, 4]), "interval=2 on 4 steps anchors {{1, 2, 4}}");
    for d in &sol.fracture {
        let ring_ids: Vec<usize> = d.rings.iter().map(|r| r.ring).collect();
        assert_eq!(ring_ids, vec![3, 4, 5, 6, 7]);
    }

    // 1. Evaluate Option 1 FAD on final frame (frame 4): Auto RingExclusion keeps all 5 outer rings (min_ring = 3 > 1),
    //    and KrSource::Auto refuses LinearScaledStep1Ki because step1_is_elastic == Some(false).
    let req_opt1 = FfsRequest {
        step: "Load".into(),
        ring_exclusion: RingExclusion::Auto,
        active_frame: None,
        domain_id: None,
        step1_is_elastic: sol.step1_is_elastic,
        fad: Some(FadConfig {
            option: FadOption::Option1,
            option2_curve: Option2Curve::ModelMaterial,
            lr_input: LrInput::DirectLoadRatio { l_r_full: 0.72 },
            toughness: ToughnessInput::KMat { value: 120.0e6 },
            kr_source: KrSource::Auto,
            station_selection: StationSelection::Governing,
            sigma_ys: None,
            sigma_uts: Some(500.0e6),
            l_r_max: None,
            young: None,
            poisson: None,
        }),
    };
    let rep1 = evaluate_step_ffs_detailed(&model, &sol.static_result, &sol.fracture, &sol.step_numbers, sol.domains_truncated, &req_opt1).unwrap();
    assert_eq!(rep1.step1_is_elastic, Some(false));
    assert_eq!(rep1.evaluated_steps.len(), 3);
    assert_eq!(rep1.step_history.len(), 3);
    assert!(rep1.step_history.windows(2).all(|w| w[1].j_mean > w[0].j_mean), "J must increase monotonically with load");
    assert!(rep1.active_profiles.iter().all(|p| p.excluded_rings.is_empty() && p.kept_rings == 5));
    assert!(rep1.diagnostics.iter().any(|d| d.code == "ffs.epfm_double_count"));

    let fad1 = rep1.fad.as_ref().unwrap();
    assert_eq!(fad1.trajectory.len(), 3);
    for pt in &fad1.trajectory {
        assert_eq!(pt.kr_source_used, KrSource::KjPlaneStrain);
        let expected_kr_allow = evaluate_fad_curve(FadOption::Option1, pt.l_r, fad1.effective_lr_max, None);
        assert!((pt.k_r_allow - expected_kr_allow).abs() <= 1e-6 * expected_kr_allow.max(1.0));
    }

    // 2. Option 2 FAD (ModelMaterial Bilinear) matches analytical Part 9 Option 2 curve to <= 1e-6.
    let mut req_opt2 = req_opt1.clone();
    req_opt2.fad.as_mut().unwrap().option = FadOption::Option2;
    let rep2 = evaluate_step_ffs_detailed(&model, &sol.static_result, &sol.fracture, &sol.step_numbers, sol.domains_truncated, &req_opt2).unwrap();
    let fad2 = rep2.fad.as_ref().unwrap();
    let eval2 = Option2Evaluator::Bilinear { young: E_STEEL, yield_stress: 250.0e6, hardening_modulus: 2.0e9 };
    for pt in &fad2.trajectory {
        let expected_kr_allow = evaluate_fad_curve(FadOption::Option2, pt.l_r, fad2.effective_lr_max, Some(&eval2));
        assert!((pt.k_r_allow - expected_kr_allow).abs() <= 1e-6 * expected_kr_allow.max(1.0));
    }

    // 3. Unevaluated increment frame 3 returns empty active_profiles yet preserves evaluated_steps & step_history.
    let req_empty = FfsRequest { active_frame: Some(3), fad: None, ..req_opt1 };
    let rep_empty =
        evaluate_step_ffs_detailed(&model, &sol.static_result, &sol.fracture, &sol.step_numbers, sol.domains_truncated, &req_empty).unwrap();
    assert!(rep_empty.active_profiles.is_empty());
    assert_eq!(rep_empty.evaluated_steps.len(), 3);
    assert_eq!(rep_empty.step_history.len(), 3);
}

//! Collapsed `l3disop` (`Wedge6`) topology, orientation, and face-mapping helpers.
//!
//! WARP3D represents 6-node linear wedges by collapsing one quadrilateral side of an 8-node
//! `l3disop` brick onto `(h2 == h3, h6 == h7)`, i.e. `[a, b, c, c, d, e, f, f]`. When a
//! crack front touches an axial wedge edge `(w0, w3)`, `(w1, w4)`, or `(w2, w5)`, we apply a
//! left cyclic prism rotation so the crack-front edge lands on the collapsed `(c, f)` edge and
//! map quadrilateral faces across WARP3D's non-collapsed hex faces `{3, 5, 6}`.

#![cfg_attr(not(test), allow(dead_code))]

use std::collections::BTreeSet;

/// Non-collapsed WARP3D `l3disop` quadrilateral faces corresponding to Wedge6 faces `3..=5`.
pub(crate) const QUAD_WARP_FACES: [u8; 3] = [3, 5, 6];

/// Axial edges `(bottom, top)` of a `Wedge6` element in FieldCraft node order.
pub(crate) const AXIAL_EDGES: [(usize, usize); 3] = [(0, 3), (1, 4), (2, 5)];

/// Triangular end-cap edges of a `Wedge6` element in FieldCraft node order.
pub(crate) const CAP_EDGES: [(usize, usize); 6] = [(0, 1), (1, 2), (2, 0), (3, 4), (4, 5), (5, 3)];

/// Identify which of the three `Wedge6` axial edges `(0, 3)`, `(1, 4)`, `(2, 5)` have both
/// endpoints in `front_nodes`.
pub(crate) fn touching_axial_edges(nodes: &[u32], front_nodes: &BTreeSet<u32>) -> [bool; 3] {
    if nodes.len() < 6 || front_nodes.is_empty() {
        return [false; 3];
    }
    let mut touched = [false; 3];
    for (idx, &(i, j)) in AXIAL_EDGES.iter().enumerate() {
        if let (Some(&ni), Some(&nj)) = (nodes.get(i), nodes.get(j))
            && ni != nj
            && front_nodes.contains(&ni)
            && front_nodes.contains(&nj)
            && let Some(slot) = touched.get_mut(idx)
        {
            *slot = true;
        }
    }
    touched
}

/// Return `true` if both endpoints of any triangular end-cap edge belong to `front_nodes`.
pub(crate) fn touches_cap_edge(nodes: &[u32], front_nodes: &BTreeSet<u32>) -> bool {
    if nodes.len() < 6 || front_nodes.is_empty() {
        return false;
    }
    CAP_EDGES.iter().any(|&(i, j)| nodes.get(i).is_some_and(|ni| front_nodes.contains(ni)) && nodes.get(j).is_some_and(|nj| front_nodes.contains(nj)))
}

/// Determine the left cyclic prism rotation `rot in 0..=2` that places a single crack-front
/// axial edge onto the collapsed `(c, f)` edge of `[a, b, c, c, d, e, f, f]`.
///
/// Defaults to `0` (keeping `(w2, w5)` collapsed) when zero or multiple axial edges touch
/// `front_nodes`.
pub(crate) fn wedge6_collapse_rotation(nodes: &[u32], front_nodes: &BTreeSet<u32>) -> u8 {
    let touched = touching_axial_edges(nodes, front_nodes);
    let count = touched.iter().filter(|&&t| t).count();
    if count != 1 {
        return 0;
    }
    match touched {
        [true, false, false] => 1,
        [false, true, false] => 2,
        _ => 0,
    }
}

/// Expand a 6-node `Wedge6` element into a right-handed collapsed 8-node `l3disop` brick
/// `[a, b, c, c, d, e, f, f]` by left-rotating the two triangular caps by `rot in 0..=2`.
pub(crate) fn expand_wedge6_to_l3disop(nodes: &[u32], rot: u8) -> Option<[u32; 8]> {
    if nodes.len() < 6 || rot >= 3 {
        return None;
    }
    let r = (rot % 3) as usize;
    let a = *nodes.get(r)?;
    let b = *nodes.get((r + 1) % 3)?;
    let c = *nodes.get((r + 2) % 3)?;
    let d = *nodes.get(3 + r)?;
    let e = *nodes.get(3 + (r + 1) % 3)?;
    let f = *nodes.get(3 + (r + 2) % 3)?;
    Some([a, b, c, c, d, e, f, f])
}

/// Map a 1-based FieldCraft `Wedge6` face (`1..=5`) under collapse rotation `rot in 0..=2` to
/// the corresponding 1-based WARP3D `l3disop` face (`1`, `2`, `3`, `5`, or `6`), avoiding the
/// collapsed face `4`.
pub(crate) fn wedge6_warp3d_face(face: u8, rot: u8) -> Option<u8> {
    if rot >= 3 {
        return None;
    }
    match face {
        1 => Some(1),
        2 => Some(2),
        3..=5 => {
            let idx = ((face - 3 + 3 - rot) % 3) as usize;
            QUAD_WARP_FACES.get(idx).copied()
        }
        _ => None,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use fieldcraft_model::ElementKind;

    use super::*;
    use crate::deck::warp3d_face;

    #[test]
    fn rotation_and_face_map_biject_across_all_3_rotations_and_5_faces() {
        let wedge = [10u32, 20, 30, 40, 50, 60];
        let w_faces = ElementKind::Wedge6.faces();
        let h_faces = ElementKind::Hex8.faces();

        for rot in 0..3u8 {
            let hex = expand_wedge6_to_l3disop(&wedge, rot).unwrap();
            assert_eq!(hex[2], hex[3]);
            assert_eq!(hex[6], hex[7]);

            let mut mapped_quad_faces = BTreeSet::new();
            for fc_face in 1..=5u8 {
                let w_face = wedge6_warp3d_face(fc_face, rot).unwrap();
                assert_ne!(w_face, 4, "rot={rot} face={fc_face} mapped onto collapsed face 4");
                if fc_face >= 3 {
                    mapped_quad_faces.insert(w_face);
                }

                // Locate the FieldCraft Hex8 face corresponding to WARP3D face `w_face`.
                let fc_hex_face = (1..=6u8).find(|&hf| warp3d_face(ElementKind::Hex8, hf) == Some(w_face)).unwrap();
                let expected_nodes: BTreeSet<u32> = w_faces[(fc_face - 1) as usize].iter().map(|&idx| wedge[idx]).collect();
                let actual_nodes: BTreeSet<u32> = h_faces[(fc_hex_face - 1) as usize].iter().map(|&idx| hex[idx]).collect();
                assert_eq!(actual_nodes, expected_nodes, "rot={rot} fc_face={fc_face} -> w_face={w_face} (fc_hex_face={fc_hex_face})");
            }
            assert_eq!(mapped_quad_faces, BTreeSet::from([3, 5, 6]));
        }
    }

    #[test]
    fn collapse_rotation_places_single_front_axial_edge_at_collapsed_hex_edge() {
        let wedge = [10u32, 20, 30, 40, 50, 60];
        // No front nodes -> default rot 0.
        assert_eq!(wedge6_collapse_rotation(&wedge, &BTreeSet::new()), 0);
        // Single node only -> default rot 0.
        assert_eq!(wedge6_collapse_rotation(&wedge, &BTreeSet::from([10])), 0);
        // Multiple axial edges -> default rot 0.
        assert_eq!(wedge6_collapse_rotation(&wedge, &BTreeSet::from([10, 40, 20, 50])), 0);

        for (edge_idx, &(i, j)) in AXIAL_EDGES.iter().enumerate() {
            let front = BTreeSet::from([wedge[i], wedge[j]]);
            let rot = wedge6_collapse_rotation(&wedge, &front);
            let expected_rot = ((edge_idx + 1) % 3) as u8;
            assert_eq!(rot, expected_rot);
            let hex = expand_wedge6_to_l3disop(&wedge, rot).unwrap();
            assert_eq!((hex[2], hex[6]), (wedge[i], wedge[j]));
            assert_eq!((hex[3], hex[7]), (wedge[i], wedge[j]));
        }
    }

    #[test]
    fn out_of_range_inputs_return_none() {
        let wedge = [1u32, 2, 3, 4, 5, 6];
        assert!(expand_wedge6_to_l3disop(&wedge[..5], 0).is_none());
        assert!(expand_wedge6_to_l3disop(&wedge, 3).is_none());
        assert!(wedge6_warp3d_face(0, 0).is_none());
        assert!(wedge6_warp3d_face(6, 0).is_none());
        assert!(wedge6_warp3d_face(3, 3).is_none());
        assert_eq!(touching_axial_edges(&wedge[..5], &BTreeSet::from([1, 4])), [false; 3]);
        assert!(!touches_cap_edge(&wedge[..5], &BTreeSet::from([1, 2])));
    }
}

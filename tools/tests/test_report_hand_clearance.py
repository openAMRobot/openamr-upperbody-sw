# SPDX-License-Identifier: MIT
"""tools/report_hand_clearance.py: the mesh distance is exact for triangles and never reports a gap where the meshes intersect. NumPy only."""
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
import report_hand_clearance as rhc  # noqa: E402


def tri(*points):
    return np.array([points], dtype=float)


def test_crossing_triangles_are_at_zero_distance():
    """The review's case (#26): two triangles cross through their edges, no vertex touches the other one. Vertex-to-face alone said 0.1 m."""
    a = tri((-0.1, -0.1, 0), (0.1, -0.1, 0), (0, 0.1, 0))
    b = tri((-0.1, 0, -0.1), (0.1, 0, -0.1), (0, 0, 0.1))
    assert rhc.mesh_distance(a, b) == 0.0
    assert rhc.mesh_distance(b, a) == 0.0


def test_a_face_piercing_edge_is_zero():
    """An edge of the small triangle passes through the big one's interior (containment through a face)."""
    big = tri((-1, -1, 0), (1, -1, 0), (0, 1, 0))
    small = tri((0, 0, -0.01), (0, 0, 0.01), (0.01, 0, 0.01))
    assert rhc.mesh_distance(small, big) == 0.0


def test_skew_edges_give_the_edge_edge_distance():
    """Closest points in the interior of two edges: crossed edges 0.05 apart, every vertex much farther from the other triangle."""
    a = tri((-1, 0, 0), (1, 0, 0), (0, 0, -1))                # vertical, in the plane y 0: top edge along x at z 0
    b = tri((0, -1, 0.05), (0, 1, 0.05), (0, 0, 1.0))        # vertical, in the plane x 0: bottom edge along y at z 0.05
    vertex_face = min(rhc.point_triangle_distance(np.unique(a.reshape(-1, 3), axis=0), b).min(),
                      rhc.point_triangle_distance(np.unique(b.reshape(-1, 3), axis=0), a).min())
    assert vertex_face > 0.5, "the case must not be settled by vertices"
    assert rhc.mesh_distance(a, b) == pytest.approx(0.05)


def test_parallel_faces_give_the_gap_and_the_height_bound_agrees():
    below = tri((-1, -1, 0), (1, -1, 0), (0, 1, 0))
    above = tri((-0.1, -0.1, 0.099), (0.1, -0.1, 0.099), (0, 0.1, 0.099))
    assert rhc.mesh_distance(above, below) == pytest.approx(0.099)
    assert rhc.height_gap(above, below) == pytest.approx(0.099)
    assert rhc.height_gap(below, above) < 0, "the bound says nothing when the meshes overlap in height"


def test_cover_selection_does_not_parse_unrelated_collada_equipment(tmp_path):
    """The canonical platform has a colored DAE visual beside its STL cover."""
    cover = tmp_path / rhc.COVER_MESH
    cover.write_text("solid cover\nfacet normal 0 0 1\nouter loop\n"
                     "vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\n"
                     "endloop\nendfacet\nendsolid cover\n")
    equipment = tmp_path / "base_equipment.dae"
    equipment.write_text("<COLLADA/>")
    root = ET.fromstring(f'''<robot><link name="base_link">
      <visual><geometry><mesh filename="{equipment}"/></geometry></visual>
      <visual><origin xyz="0 0 0.2"/><geometry><mesh filename="{cover}" scale="2 2 2"/></geometry></visual>
    </link></robot>''')
    meshes = rhc.link_meshes(root, "base_link", "visual", mesh_filename=rhc.COVER_MESH)
    assert len(meshes) == 1
    pose, triangles = meshes[0]
    np.testing.assert_array_equal(pose[:3, 3], [0, 0, 0.2])
    np.testing.assert_array_equal(triangles, tri((0, 0, 0), (2, 0, 0), (0, 2, 0)))

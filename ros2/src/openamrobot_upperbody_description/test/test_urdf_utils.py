# SPDX-License-Identifier: MIT
"""The FK helper itself, checked against hand-computed cases (so that T05 does not rest on an unchecked helper)."""
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from urdf_utils import fixed_fk, position, rpy_to_matrix, zero_pose_poses


def _urdf(*joints):
    xml = "<robot>" + "".join(
        f'<joint name="{n}" type="{t}"><parent link="{p}"/><child link="{c}"/>{o}</joint>' for n, t, p, c, o in joints
    ) + "</robot>"
    return ET.fromstring(xml)


def test_rpy_axes_follow_the_urdf_convention():
    # yaw 90 deg: x -> y
    assert np.allclose(rpy_to_matrix(0, 0, math.pi / 2) @ [1, 0, 0], [0, 1, 0])
    # pitch 90 deg: x -> -z, z -> x
    assert np.allclose(rpy_to_matrix(0, math.pi / 2, 0) @ [1, 0, 0], [0, 0, -1])
    assert np.allclose(rpy_to_matrix(0, math.pi / 2, 0) @ [0, 0, 1], [1, 0, 0])
    # roll 90 deg: y -> z
    assert np.allclose(rpy_to_matrix(math.pi / 2, 0, 0) @ [0, 1, 0], [0, 0, 1])
    # composition order R = Rz Ry Rx (roll applied first, about fixed axes): y -Rx-> z -Ry-> x.
    # The reverse order (Rx Ry) would give z.
    r = rpy_to_matrix(math.pi / 2, math.pi / 2, 0)
    assert np.allclose(r @ [0, 1, 0], [1, 0, 0])
    r = rpy_to_matrix(0.3, -0.7, 1.9)
    assert np.allclose(r.T @ r, np.eye(3)) and math.isclose(np.linalg.det(r), 1.0)


def test_chain_composes_translation_through_a_rotation():
    # a --(z+1)--> b --(yaw 90, x+2)--> c --(x+1)--> d ; d is 1 along c's x, which is world y
    root = _urdf(
        ("j1", "fixed", "a", "b", '<origin xyz="0 0 1"/>'),
        ("j2", "fixed", "b", "c", '<origin xyz="2 0 0" rpy="0 0 1.5707963267948966"/>'),
        ("j3", "fixed", "c", "d", '<origin xyz="1 0 0"/>'),
    )
    assert np.allclose(position(fixed_fk(root, "a", "d")), [2, 1, 1])
    assert np.allclose(position(fixed_fk(root, "b", "d")), [2, 1, 0])
    assert np.allclose(fixed_fk(root, "a", "a"), np.eye(4))


def test_missing_origin_is_identity():
    assert np.allclose(fixed_fk(_urdf(("j", "fixed", "a", "b", "")), "a", "b"), np.eye(4))


def test_movable_joint_on_the_path_is_an_error():
    root = _urdf(("j1", "fixed", "a", "b", ""), ("j2", "revolute", "b", "c", ""))
    with pytest.raises(ValueError, match="not fixed"):
        fixed_fk(root, "a", "c")
    # ... but a movable joint off the path is fine
    assert np.allclose(fixed_fk(root, "a", "b"), np.eye(4))


def test_non_ancestor_is_an_error():
    root = _urdf(("j1", "fixed", "a", "b", ""), ("j2", "fixed", "a", "c", ""))
    with pytest.raises(ValueError, match="not an ancestor"):
        fixed_fk(root, "b", "c")


def test_zero_pose_poses_treat_movable_joints_as_their_origin_only():
    root = ET.fromstring(
        "<robot><link name='a'/><link name='b'/><link name='c'/><link name='d'/>"
        "<joint name='ab' type='fixed'><parent link='a'/><child link='b'/><origin xyz='1 0 0' rpy='0 0 1.5707963267948966'/></joint>"
        "<joint name='bc' type='revolute'><parent link='b'/><child link='c'/><origin xyz='1 0 0'/><axis xyz='0 0 1'/></joint>"
        "<joint name='cd' type='continuous'><parent link='c'/><child link='d'/><origin xyz='0 0 2'/></joint></robot>")
    poses = zero_pose_poses(root)
    assert set(poses) == {"a", "b", "c", "d"}
    assert np.allclose(position(poses["a"]), [0, 0, 0])
    # b is yawed 90 deg, so c's x offset of 1 lands on world y; the revolute joint adds no rotation at q = 0
    assert np.allclose(position(poses["c"]), [1, 1, 0])
    assert np.allclose(position(poses["d"]), [1, 1, 2])
    # fixed_fk, which refuses movable joints, agrees where it is allowed to answer
    assert np.allclose(poses["b"], fixed_fk(root, "a", "b"))

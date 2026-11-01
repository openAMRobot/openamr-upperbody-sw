# SPDX-License-Identifier: MIT
"""The chassis box clears the four caster-wheel spheres, and the derivation says so: an old-box-fails / new-box-clears regression (platform-sw #71).

platform-sw #70 shipped a chassis collision box whose bottom was 0.050 m above the floor; the four caster-wheel collision spheres top out at 0.052421 m, so the
box overlapped them by 2.4 mm in every state: `base_link` against each `{fl,fr,bl,br}_wheel_link` is a permanent collision, `regenerate_acm.sh` refuses to
disable such a pair (91 pairs derive against the 95 of the contract) and M03 and AUX FAIL (P19, on `d619012`). #71 raised the bottom to 0.055 m: 2.6 mm
clear. platform-sw's reviewer asked for "a small repeatable old-box-fails/new-box-clears regression" so that this cannot come back silently when the
base is bumped; this is it, here, because M03 is ours and what fails when it returns is our check.

No ROS runtime and no simulator. The geometry is read from the expanded combined URDF (the base is platform-sw's, by reference) and evaluated
analytically (an axis-aligned box against a sphere is exact). The "old box" is the SAME model with the box bottom lowered to 0.050 m (top unchanged), so
the test follows whatever base is pinned instead of carrying a copy of it. The pair oracle handed to `acm_derive.derive` answers for the five links in
question only (the four pairs); that is enough to show what the derivation does with a permanent collision, which is the finding M03 reports."""
import copy

import numpy as np
import pytest

from conftest import urdf_root
from openamrobot_upperbody_moveit import acm_derive
from openamrobot_upperbody_moveit.acm_split import pair_key
from test_acm_derive import forward_kinematics, rpy_matrix

CASTER_WHEELS = ("fl_wheel_link", "fr_wheel_link", "bl_wheel_link", "br_wheel_link")
OLD_BOX_BOTTOM_M = 0.050          # #70
NEW_BOX_BOTTOM_M = 0.055          # #71
CASTER_SPHERE_TOP_M = 0.052421    # as reported in #71's review: centre 0.026421 + radius 0.026 above the floor
NEW_CLEARANCE_M = 0.0026          # #71's own figure, 2.6 mm; the vertical gap is the minimum distance because each sphere is below the box footprint


def floats(text, default="0 0 0"):
    return [float(v) for v in (text or default).split()]


def chassis_box(root):
    """The chassis collision box of base_link: (origin element, size element) of the largest <box> collision of that link."""
    link = next(l for l in root.findall("link") if l.get("name") == "base_link")
    boxes = [(c.find("origin"), c.find("geometry/box")) for c in link.findall("collision") if c.find("geometry/box") is not None]
    origin, box = max(boxes, key=lambda ob: np.prod(floats(ob[1].get("size"))))
    return origin, box


def with_box_bottom(root, bottom_above_floor):
    """A copy of the model whose chassis box has the given bottom (above the floor) and the same top and the same footprint."""
    out = copy.deepcopy(root)
    origin, box = chassis_box(out)
    base_z = forward_kinematics(out, {})["base_link"][2, 3]
    size = floats(box.get("size"))
    xyz = floats(origin.get("xyz"))
    top = base_z + xyz[2] + size[2] / 2
    size[2] = top - bottom_above_floor
    xyz[2] = (top + bottom_above_floor) / 2 - base_z
    box.set("size", " ".join(repr(v) for v in size))
    origin.set("xyz", " ".join(repr(v) for v in xyz))
    return out


def box_bottom_and_top(root):
    origin, box = chassis_box(root)
    base_z = forward_kinematics(root, {})["base_link"][2, 3]
    z, h = floats(origin.get("xyz"))[2], floats(box.get("size"))[2]
    return base_z + z - h / 2, base_z + z + h / 2


def box_sphere_clearances(root):
    """{wheel link: distance from its collision sphere to the chassis box} (negative: they overlap), at the default state, in the base frame."""
    poses = forward_kinematics(root, {})
    origin, box = chassis_box(root)
    assert np.allclose(floats(origin.get("rpy")), 0), "the analytic box test assumes the chassis box is not rotated in base_link"
    centre_in_base = np.array(floats(origin.get("xyz")))
    half = np.array(floats(box.get("size"))) / 2
    to_base = np.linalg.inv(poses["base_link"])
    out = {}
    for link in root.findall("link"):
        if link.get("name") not in CASTER_WHEELS:
            continue
        spheres = [c for c in link.findall("collision") if c.find("geometry/sphere") is not None]
        assert len(spheres) == 1, f"{link.get('name')}: expected one collision sphere"
        s_origin = spheres[0].find("origin")
        local = np.array(floats(s_origin.get("xyz") if s_origin is not None else None) + [1.0])
        centre = (to_base @ poses[link.get("name")] @ local)[:3] - centre_in_base
        radius = float(spheres[0].find("geometry/sphere").get("radius"))
        outside = np.maximum(np.abs(centre) - half, 0.0)
        out[link.get("name")] = float(np.linalg.norm(outside)) - radius if np.any(outside > 0) else -radius - float(np.min(half - np.abs(centre)))
    assert sorted(out) == sorted(CASTER_WHEELS)
    return out


def oracle(root):
    """`collides(pairs)` for acm_derive.derive: the chassis box against each caster-wheel sphere is answered from the geometry; every other pair is clear."""
    clearance = box_sphere_clearances(root)
    return lambda pairs: {p for p in pairs if p[0] == "base_link" and p[1] in clearance and clearance[p[1]] < 0}


@pytest.fixture(scope="module")
def new_body():
    return urdf_root("bp000")


def test_the_pinned_base_has_its_box_bottom_at_0_055_and_the_caster_spheres_top_at_0_0524(new_body):
    bottom, _ = box_bottom_and_top(new_body)
    assert bottom == pytest.approx(NEW_BOX_BOTTOM_M, abs=1e-6), f"chassis box bottom {bottom:.6f} m: the pinned base is not platform-sw #71 (or a later base moved it)"
    poses = forward_kinematics(new_body, {})
    tops = []
    for link in new_body.findall("link"):
        if link.get("name") in CASTER_WHEELS:
            s = link.find("collision/geometry/sphere")
            o = link.find("collision/origin")
            tops.append((poses[link.get("name")] @ np.array(floats(o.get("xyz") if o is not None else None) + [1.0]))[2] + float(s.get("radius")))
    assert tops and all(t == pytest.approx(CASTER_SPHERE_TOP_M, abs=1e-5) for t in tops), tops


def test_the_new_box_clears_every_caster_sphere_by_about_2_6_mm(new_body):
    clearance = box_sphere_clearances(new_body)
    assert min(clearance.values()) == pytest.approx(NEW_CLEARANCE_M, abs=2e-4), clearance


def test_the_old_box_overlaps_every_caster_sphere_by_about_2_4_mm():
    old = with_box_bottom(urdf_root("bp000"), OLD_BOX_BOTTOM_M)
    clearance = box_sphere_clearances(old)
    assert all(v < 0 for v in clearance.values()), clearance
    assert max(clearance.values()) == pytest.approx(OLD_BOX_BOTTOM_M - CASTER_SPHERE_TOP_M, abs=2e-5), clearance      # -2.421 mm


def test_the_derivation_reports_the_old_box_as_four_permanent_collisions_and_the_new_box_as_none(new_body):
    """What M03 (and regenerate_acm.sh) turn into a FAIL: a constant pair that collides in every state is neither disabled nor hidden."""
    four = sorted(pair_key("base_link", w) for w in CASTER_WHEELS)
    old = with_box_bottom(new_body, OLD_BOX_BOTTOM_M)
    matrix_old, report_old = acm_derive.derive(old, {}, oracle(old))
    assert report_old["constant_and_colliding"] == four, report_old["constant_and_colliding"]
    assert not any(p in matrix_old for p in four), "a permanent collision must not be disabled"
    matrix_new, report_new = acm_derive.derive(new_body, {}, oracle(new_body))
    assert report_new["constant_and_colliding"] == []
    assert all(matrix_new.get(p) == acm_derive.CONSTANT for p in four), "with the box clear of the casters the four pairs are certified ConstantTransform"
    assert len(matrix_new) - len(matrix_old) == 4

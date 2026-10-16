"""E01: assembled envelope (height, width, depth) against the P-00 envelope datums. WARN only, never PASS.

The three extents of the robot at each calibration stop of the lift (L1000, L1175, L1350: lift_joint at 0, 0.175, 0.350 m, every
other joint at zero), in the root frame base_footprint, which IS the floor (x forward = depth, y left = width, z up = height):
  - the origin of every link frame, and
  - the extents of every box, cylinder and sphere (visual and collision), corners taken through the full link pose.
Mesh extents are NOT included: a mesh contributes only its link's frame origin. So every figure here is a LOWER BOUND:
the real robot is at least this big. A figure under the limit therefore proves nothing about the real envelope, which is
why this check can never be PASS, and a figure over the limit is a real exceedance, which is why it is reported
prominently. It is also never FAIL: E01 is a flag for A4 to look at, not a gate (VERIFICATION.md, decided 5 Oct).

Height is measured from the floor = base_footprint, the root (since platform-sw 935efc7 and option B, which measures
everything from it; the datum `base_link_height_above_floor` is asserted equal to the base_footprint -> base_link joint by T04, so
there is one floor). Width and depth are the spans of y and x over the same points; the planar `world` frame, absent in the
default expansion, is not involved. A mast no longer sets the height: the lift does, and the figures move with the stop.
Limits: datums `envelope_max_height`, `envelope_max_width`, `envelope_max_depth` (P-00 s20.2, provisional).
"""
import numpy as np
import pytest

import evidence_common as ev
from conftest import BASELINE, STOPS, UPPERBODY, expand, load_datums, write_sidecar
from urdf_utils import joint_origin, zero_pose_poses

CIRCLE_POINTS = 16          # points on each cylinder rim
AXES = {"depth": 0, "width": 1, "height": 2}     # base_link: x forward, y left, z up
LIMIT_DATUMS = {"height": "envelope_max_height", "width": "envelope_max_width", "depth": "envelope_max_depth"}


def _extent_points(geometry):
    """Local-frame points (N x 4) that bound a box / cylinder / sphere, or None for a mesh or anything else."""
    shape = geometry[0]
    if shape.tag == "box":
        sx, sy, sz = (float(v) for v in shape.get("size").split())
        return np.array([[x * sx / 2, y * sy / 2, z * sz / 2, 1.0] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    if shape.tag == "cylinder":
        r, h = float(shape.get("radius")), float(shape.get("length"))
        return np.array([[r * np.cos(a), r * np.sin(a), s * h / 2, 1.0]
                         for a in np.linspace(0, 2 * np.pi, CIRCLE_POINTS, endpoint=False) for s in (-1, 1)])
    if shape.tag == "sphere":
        r = float(shape.get("radius"))
        return np.array([[r, 0, 0, 1.0], [-r, 0, 0, 1.0], [0, r, 0, 1.0], [0, -r, 0, 1.0], [0, 0, r, 1.0], [0, 0, -r, 1.0]])
    return None


def bounding_extents(root, floor_z, positions=None):
    """{axis: {min, max, span, min_at, max_at}} over every frame origin and primitive extent of the URDF at zero pose, or with
    the joints in `positions` ({joint name: value}: the lift stops).

    `min`/`max` are coordinates in the root link frame (height: above the floor). `*_at` is "<link> (<what>)", the point
    that sets that bound. Meshes contribute only their link's frame origin.
    """
    poses = zero_pose_poses(root, positions)
    lows, highs = {}, {}

    def see(point, link, what):
        for axis, i in AXES.items():
            value = point[i] - (floor_z if axis == "height" else 0.0)
            if axis not in lows or value < lows[axis][0]:
                lows[axis] = (value, f"{link} ({what})")
            if axis not in highs or value > highs[axis][0]:
                highs[axis] = (value, f"{link} ({what})")

    for link in root.findall("link"):
        name = link.get("name")
        see(poses[name][:3, 3], name, "frame origin")
        for element in link.findall("visual") + link.findall("collision"):
            geometry = element.find("geometry")
            points = None if geometry is None or len(geometry) == 0 else _extent_points(geometry)
            if points is None:
                continue
            world = (poses[name] @ joint_origin(element) @ points.T).T
            what = f"{element.tag} {geometry[0].tag}"
            for axis, i in AXES.items():
                see(world[world[:, i].argmin()], name, what)
                see(world[world[:, i].argmax()], name, what)
    return {axis: {"min": lows[axis][0], "max": highs[axis][0], "span": highs[axis][0] - lows[axis][0],
                   "min_at": lows[axis][1], "max_at": highs[axis][1]} for axis in AXES}


def _floor_z():
    """The floor is the root frame, base_footprint (z 0): nothing is subtracted."""
    return 0.0


def _limits():
    datums = load_datums()["datums"]
    return {axis: float(datums[key]["value"]) for axis, key in LIMIT_DATUMS.items()}


def dimension_figure(axis, extent, limit):
    """Reported figure for one dimension. Height is the top above the floor; width and depth are spans."""
    value = extent["max"] if axis == "height" else extent["span"]
    return {"lower_bound_m": round(value, 4), "limit_m": limit, "margin_m": round(limit - value, 4),
            "exceeds_limit": bool(value > limit),
            "set_by": extent["max_at"] if axis == "height" else f"{extent['min_at']} .. {extent['max_at']}"}


def envelope_result(figures):
    """E01 is a flag, never a verdict: WARN whatever the figures are, because they are lower bounds."""
    return "WARN"


def test_e01_envelope_height_width_depth_are_reported_as_a_warning():
    floor, limits = _floor_z(), _limits()
    root = expand(arm_package="openarm")
    rows = {}
    for stop in STOPS:
        q = ev.lift_joint_position(UPPERBODY, BASELINE["column"], stop)
        extents = bounding_extents(root, floor, {"lift_joint": q})
        rows[stop] = {"shoulder_height_m": STOPS[stop], "lift_joint_position_m": round(q, 6),
                      **{axis: dimension_figure(axis, extents[axis], limits[axis]) for axis in ("height", "width", "depth")}}
    exceeded = {s: [a for a in ("height", "width", "depth") if r[a]["exceeds_limit"]] for s, r in rows.items()}
    result = envelope_result(rows)
    write_sidecar("E01", "envelope", "assembled envelope height, width, depth (lower bound)", result, {
        "lower_bound": True,
        "method": "per calibration stop of the lift (lift_joint at the stop, every other joint at zero), root frame base_footprint = "
                  "the floor; height = highest point above the floor, width = span of y, depth = span of x; over link frame origins "
                  "and box/cylinder/sphere extents (visual and collision); mesh extents NOT included, so every figure is a lower "
                  "bound and a figure under the limit proves nothing about the real envelope",
        "limits_m": limits,
        "limit_source": "datums envelope_max_height/width/depth (P-00 s20.2 target envelope 1700 x 600 x 800 mm, provisional)",
        "exceeds_limit_even_as_lower_bound": {s: dims for s, dims in exceeded.items() if dims},
        "why_warn": "WARN, never PASS: nobody has measured the envelope; never FAIL: a flag for A4, not a gate",
        "per_lift_stop": rows,
    })
    assert result == "WARN"


def test_e01_the_height_grows_with_the_lift_by_the_stroke_and_the_head_camera_is_the_top():
    root = expand(arm_package="openarm")
    tops = {}
    for stop in STOPS:
        q = ev.lift_joint_position(UPPERBODY, BASELINE["column"], stop)
        tops[stop] = bounding_extents(root, _floor_z(), {"lift_joint": q})["height"]
    assert tops["L1350"]["max"] - tops["L1000"]["max"] == pytest.approx(0.350, abs=1e-9)
    assert tops["L1175"]["max"] - tops["L1000"]["max"] == pytest.approx(0.175, abs=1e-9)
    # the frame of the head camera lens is the highest frame of the structure at full lift: 0.304 + 0.880 + 0.166 + 0.223
    assert tops["L1350"]["max"] == pytest.approx(0.304 + 0.880 + 0.166 + 0.223, abs=1e-6)
    assert tops["L1350"]["max_at"] == "head_camera_mount_link (frame origin)"
    assert tops["L1350"]["max"] <= float(UPPERBODY["geometry"]["max_assembled_height"])


def test_e01_the_floor_is_the_root_frame():
    root = expand(arm_package="openarm")
    assert zero_pose_poses(root)["base_footprint"][2, 3] == 0.0
    assert _floor_z() == 0.0


def test_e01_the_height_limit_matches_option_bs_max_assembled_height():
    assert _limits()["height"] == float(UPPERBODY["geometry"]["max_assembled_height"]) == 1.700


# --- the finder and the verdict, on synthetic URDFs: they must be able to fail ---------------------------------------

def _urdf(body):
    import xml.etree.ElementTree as ET
    return ET.fromstring(f"<robot>{body}</robot>")


def test_e01_finder_sees_a_box_top_a_rotated_box_a_cylinder_and_ignores_a_mesh():
    box = _urdf("<link name='a'><visual><origin xyz='0 0 1'/><geometry><box size='1 1 2'/></geometry></visual></link>")
    assert bounding_extents(box, 0.0)["height"]["max"] == pytest.approx(2.0)       # centre 1 + half-height 1
    # the same box rolled 90 degrees: its 2 m side is now horizontal, so the top is centre 1 + half of 1
    rolled = _urdf("<link name='a'><visual><origin xyz='0 0 1' rpy='1.5707963267948966 0 0'/>"
                   "<geometry><box size='1 1 2'/></geometry></visual></link>")
    assert bounding_extents(rolled, 0.0)["height"]["max"] == pytest.approx(1.5)
    cylinder = _urdf("<link name='a'><collision><geometry><cylinder radius='0.1' length='3'/></geometry></collision></link>")
    assert bounding_extents(cylinder, 0.0)["height"]["max"] == pytest.approx(1.5)
    mesh = _urdf("<link name='a'><visual><geometry><mesh filename='package://p/m.stl'/></geometry></visual></link>"
                 "<link name='b'/><joint name='j' type='fixed'><parent link='a'/><child link='b'/>"
                 "<origin xyz='0 0 0.7'/></joint>")
    height = bounding_extents(mesh, 0.0)["height"]
    assert (height["max"], height["max_at"]) == (pytest.approx(0.7), "b (frame origin)")   # the mesh adds nothing
    assert bounding_extents(mesh, -0.5)["height"]["max"] == pytest.approx(1.2)             # floor below the base origin


def test_e01_width_is_the_span_of_y_and_depth_the_span_of_x_and_neither_is_the_height():
    # a 0.2 (x) by 0.6 (y) by 1.0 (z) box offset by +0.1 in y, plus a frame origin at x = -0.5 on a second link
    urdf = _urdf("<link name='a'><visual><origin xyz='0 0.1 0.5'/><geometry><box size='0.2 0.6 1.0'/></geometry></visual></link>"
                 "<link name='b'/><joint name='j' type='fixed'><parent link='a'/><child link='b'/><origin xyz='-0.5 0 0'/></joint>")
    e = bounding_extents(urdf, 0.0)
    assert (e["width"]["min"], e["width"]["max"], e["width"]["span"]) == pytest.approx((-0.2, 0.4, 0.6))
    assert (e["depth"]["min"], e["depth"]["max"], e["depth"]["span"]) == pytest.approx((-0.5, 0.1, 0.6))
    assert e["depth"]["min_at"] == "b (frame origin)" and e["width"]["max_at"] == "a (visual box)"
    assert e["height"]["max"] == pytest.approx(1.0)
    # a rotation about z swaps which of the box's sides is along x and which along y
    turned = _urdf("<link name='a'><visual><origin xyz='0 0 0' rpy='0 0 1.5707963267948966'/>"
                   "<geometry><box size='0.2 0.6 1.0'/></geometry></visual></link>")
    e = bounding_extents(turned, 0.0)
    assert (e["width"]["span"], e["depth"]["span"]) == pytest.approx((0.2, 0.6))


def test_e01_the_figure_reports_an_exceedance_and_a_margin_from_the_lower_bound():
    extent = {"min": -0.35, "max": 0.35, "span": 0.70, "min_at": "l (frame origin)", "max_at": "r (frame origin)"}
    over = dimension_figure("width", extent, 0.6)
    assert over["exceeds_limit"] is True and over["lower_bound_m"] == pytest.approx(0.7) and over["margin_m"] == pytest.approx(-0.1)
    assert dimension_figure("width", extent, 0.7)["exceeds_limit"] is False        # exactly at the limit is not over it
    tall = {"min": 0.0, "max": 1.5756, "span": 1.5756, "min_at": "x", "max_at": "head_camera_link (visual box)"}
    assert dimension_figure("height", tall, 1.7)["set_by"] == "head_camera_link (visual box)"


@pytest.mark.parametrize("figures", [{}, {"m": {"height": {"exceeds_limit": False}}}, {"m": {"width": {"exceeds_limit": True}}}])
def test_e01_the_verdict_is_never_pass_and_never_fail(figures):
    assert envelope_result(figures) == "WARN"

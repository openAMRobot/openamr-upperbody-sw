# SPDX-License-Identifier: MIT
"""The Nav2 footprint that follows the arms."""
import math

from openamrobot_build1_scene import footprint as fp

# The octagon of openamr-platform-sw nav2_params.yaml at 0741778.
BASE = "[[0.415, 0.19], [0.415, -0.19], [0.315, -0.29], [-0.290, -0.29], [-0.390, -0.19], [-0.390, 0.19], [-0.290, 0.29], [0.315, 0.29]]"


def area(polygon):
    return 0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(polygon, polygon[1:] + polygon[:1]))


def test_arms_inside_the_base_leave_the_base_footprint():
    base = fp.parse_polygon(BASE)
    polygon = fp.footprint(base, [(0.23, 0.15), (0.23, -0.15)], 0.08)
    assert abs(area(polygon) - area(fp.convex_hull(base))) < 1e-9
    assert area(polygon) > 0, "the hull must be counter-clockwise"


def test_an_arm_reaching_forward_grows_the_footprint_and_is_inside_it_with_padding():
    base = fp.parse_polygon(BASE)
    hand = (0.80, 0.16)
    polygon = fp.footprint(base, [hand], 0.08)
    assert max(x for x, _ in polygon) >= 0.88 - 1e-9
    assert all(fp.contains(polygon, point) for point in base)
    for k in range(36):
        on_circle = (hand[0] + 0.08 * math.cos(k * math.pi / 18), hand[1] + 0.08 * math.sin(k * math.pi / 18))
        assert fp.contains(polygon, on_circle, tolerance=1e-6), "the padding circle must lie inside the footprint"
    assert not fp.contains(polygon, (0.95, 0.16))


def test_an_arm_reaching_sideways_grows_only_that_side():
    base = fp.parse_polygon(BASE)
    polygon = fp.footprint(base, [(0.2, 0.6)], 0.05)
    assert max(y for _, y in polygon) > 0.64 and min(y for _, y in polygon) == -0.29


def test_the_lift_height_is_not_an_input():
    """The same floor projection gives the same footprint whatever the height of the arm points."""
    import inspect
    assert "lift" not in inspect.signature(fp.footprint).parameters


def test_change_detection():
    a = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert fp.changed(None, a, 0.01) and not fp.changed(a, list(a), 0.01)
    assert fp.changed(a, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.02)], 0.01)
    assert not fp.changed(a, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.005)], 0.01)
    assert fp.changed(a, a + [(0.0, 1.0)], 0.01)

# SPDX-License-Identifier: MIT
"""The Nav2 footprint as a function of the arm pose. Pure geometry, no ROS import.

The base footprint is the polygon of the Nav2 parameters. Every arm point that the caller passes is projected on the
floor plane, padded, and the convex hull of all of it is the footprint. The lift moves the arms straight up, so it
changes no floor projection: the lift height is not an input.
"""
import ast
import math


def parse_polygon(text):
    """Nav2's footprint parameter, a string like "[[0.4, 0.2], ...]" -> [(x, y)]."""
    points = ast.literal_eval(text) if isinstance(text, str) else text
    return [(float(x), float(y)) for x, y in points]


def convex_hull(points):
    """Andrew's monotone chain. -> the hull, counter-clockwise, no repeated first point."""
    points = sorted(set((round(x, 6), round(y, 6)) for x, y in points))
    if len(points) <= 2:
        return points

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def padded(point, padding, sides=8):
    """`sides` points on a circle around `point` whose polygon contains the circle of radius `padding`."""
    radius = padding / math.cos(math.pi / sides)
    return [(point[0] + radius * math.cos(2 * math.pi * k / sides), point[1] + radius * math.sin(2 * math.pi * k / sides))
            for k in range(sides)]


def footprint(base_polygon, arm_points, padding):
    """Convex hull of the base polygon and of every arm point (x, y in base_link) padded by `padding` m."""
    points = list(base_polygon)
    for point in arm_points:
        points += padded(point, padding)
    return convex_hull(points)


def contains(polygon, point, tolerance=1e-9):
    """True when `point` is inside or on the convex counter-clockwise `polygon`."""
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0]) < -tolerance:
            return False
    return True


def changed(old, new, threshold):
    """True when the two polygons differ by more than `threshold` m in any vertex, or in vertex count."""
    if old is None or len(old) != len(new):
        return True
    return any(math.hypot(a[0] - b[0], a[1] - b[1]) > threshold for a, b in zip(old, new))
